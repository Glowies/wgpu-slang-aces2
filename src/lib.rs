use clap::Parser;
use exr::prelude::*;
use flume::bounded;
use std::path::PathBuf;

mod slang_macros;

#[derive(Parser, Debug)]
#[command(name = "wgpu-slang-aces2")]
#[command(about = "Process EXR images with ACES 2.0 compute shader", long_about = None)]
pub struct Args {
    pub input: PathBuf,
    pub output: PathBuf,
}

pub async fn run(args: Args) -> anyhow::Result<()> {
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

    // READ EXR FILE
    println!("Reading EXR file: {:?}", args.input);
    let image = read_first_rgba_layer_from_file(
        &args.input,
        |resolution, _| {
            let default_pixel = [0.0, 0.0, 0.0, 0.0];
            let empty_line = vec![default_pixel; resolution.width()];
            let empty_image = vec![empty_line; resolution.height()];
            empty_image
        },
        // transfer the colors from the file to your image type,
        // requesting all values to be converted to f32 numbers
        |pixel_vector, position, (r, g, b, a): (f32, f32, f32, f32)| {
            pixel_vector[position.y()][position.x()] = [r, g, b, a]
        },
    )?;

    let texture_size = wgpu::Extent3d {
        width: image.layer_data.size.width() as u32,
        height: image.layer_data.size.height() as u32,
        depth_or_array_layers: 1,
    };
    let pixel_count = (texture_size.width * texture_size.height) as usize;
    println!("Image dimensions: {:?}", texture_size);

    // Convert EXR RGBA data to f32 array for GPU
    let mut input_data = Vec::with_capacity(pixel_count * 4);
    for row in &image.layer_data.channel_data.pixels {
        for pixel in row {
            input_data.push(pixel[0]);
            input_data.push(pixel[1]);
            input_data.push(pixel[2]);
            input_data.push(pixel[3]);
        }
    }

    // CREATE TEXTURES
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
    let texel_copy_buffer_layout = wgpu::TexelCopyBufferLayout {
        offset: 0,
        bytes_per_row: Some(16 * texture_size.width),
        rows_per_image: Some(texture_size.height),
    };
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &input_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        bytemuck::cast_slice(&input_data),
        texel_copy_buffer_layout,
        texture_size,
    );

    // Create a temp buffer for reading back results
    let temp_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("temp"),
        size: (texture_size.width * texture_size.height * 16) as u64,
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
        let num_workgroups_x = texture_size.width.div_ceil(8);
        let num_workgroups_y = texture_size.height.div_ceil(8);

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
            layout: texel_copy_buffer_layout,
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

    // Convert output data back to EXR RGBA format
    println!("Writing output to: {:?}", args.output);
    let mut output_pixels = Vec::with_capacity(pixel_count);
    for i in 0..pixel_count {
        let offset = i * 4;
        output_pixels.push((
            output_data[offset],
            output_data[offset + 1],
            output_data[offset + 2],
            output_data[offset + 3],
        ));
    }

    // Write EXR file
    write_rgba_file(
        &args.output,
        texture_size.width as usize,
        texture_size.height as usize,
        |x, y| {
            // Return the pixel at the given coordinates
            output_pixels[y * texture_size.width as usize + x]
        },
    )?;

    println!("Successfully processed image!");

    Ok(())
}
