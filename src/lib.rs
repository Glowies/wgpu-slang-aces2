use flume::bounded;

mod slang_macros;

pub async fn run() -> anyhow::Result<()> {
    let instance = wgpu::Instance::new(&Default::default());
    let adapter = instance.request_adapter(&Default::default()).await.unwrap();
    let (device, queue) = adapter.request_device(&Default::default()).await.unwrap();

    let shader = device.create_shader_module(wgpu_include_slang_shader!("introduction"));

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("ACES 2.0 Compute Pipeline"),
        layout: None,
        module: &shader,
        entry_point: None,
        compilation_options: Default::default(),
        cache: Default::default(),
    });

    // DECLARE TEXTURE DIMENSIONS AND INPUT DATA
    let texture_width = 256u32;
    let texture_height = 256u32;

    // Create some sample pixel data (RGBA format, 4 bytes per pixel)
    let mut input_data = vec![31f32; (texture_width * texture_height * 4) as usize];
    for y in 0..texture_height {
        for x in 0..texture_width {
            let index = ((y * texture_width + x) * 4) as usize;
            input_data[index] = (x % 256) as f32; // R
            input_data[index + 1] = (y % 256) as f32; // G
            input_data[index + 2] = 128f32; // B
            input_data[index + 3] = 255f32; // A
        }
    }

    // CREATE TEXTURES
    let texture_size = wgpu::Extent3d {
        width: texture_width,
        height: texture_height,
        depth_or_array_layers: 1,
    };
    let texture_format = wgpu::TextureFormat::Rgba32Float;

    let input_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("input_texture"),
        size: texture_size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: texture_format,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });

    let output_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("output_texture"),
        size: texture_size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: texture_format,
        usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });

    // Write input data to input texture
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &input_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        bytemuck::cast_slice(&input_data),
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(16 * texture_width),
            rows_per_image: Some(texture_height),
        },
        texture_size,
    );

    // Create a temp buffer for reading back results
    let temp_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("temp"),
        size: (texture_width * texture_height * 16) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    // CREATE TEXTURE VIEWS
    let input_texture_view = input_texture.create_view(&wgpu::TextureViewDescriptor::default());
    let output_texture_view = output_texture.create_view(&wgpu::TextureViewDescriptor::default());

    // CREATE BIND GROUP FOR THE COMPUTE SHADER
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&input_texture_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&output_texture_view),
            },
        ],
    });

    // ENQUEUE THE COMPUTE SHADER AND TEXTURE COPY
    let mut encoder = device.create_command_encoder(&Default::default());

    {
        // We specified 8x8 threads per workgroup in the shader, so we need to compute how many
        // workgroups we need to dispatch.
        let num_workgroups_x = texture_width.div_ceil(8);
        let num_workgroups_y = texture_height.div_ceil(8);

        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(num_workgroups_x, num_workgroups_y, 1);
    }

    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &output_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &temp_buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(texture_width * 16),
                rows_per_image: Some(texture_height),
            },
        },
        texture_size,
    );

    queue.submit([encoder.finish()]);

    // GET INFO BACK FROM GPU
    let output_data = {
        // The mapping process is async, so we'll need to create a channel to get
        // the success flag for our mapping
        let (tx, rx) = bounded(1);

        // We send the success or failure of our mapping via a callback
        temp_buffer.map_async(wgpu::MapMode::Read, .., move |result| {
            tx.send(result).unwrap()
        });

        // The callback we submitted to map async will only get called after the
        // device is polled or the queue submitted
        device.poll(wgpu::PollType::wait_indefinitely())?;

        // We check if the mapping was successful here
        rx.recv_async().await??;

        // We then get the bytes that were stored in the buffer
        let buffer_view = temp_buffer.get_mapped_range(..);
        let data: Vec<f32> = bytemuck::cast_slice(&buffer_view).to_vec();

        data
    };
    // We need to unmap the buffer to be able to use it again
    temp_buffer.unmap();

    // Print a sample of the output data (first 64 bytes as RGBA pixels)
    println!("First 16 pixels (RGBA):");
    for i in 0..16 {
        let offset = i * 4;
        println!(
            "Pixel {}: R={}, G={}, B={}, A={}",
            i,
            output_data[offset],
            output_data[offset + 1],
            output_data[offset + 2],
            output_data[offset + 3]
        );
    }

    Ok(())
}
