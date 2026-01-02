/// A custom macro to include a compiled slang shader from the build output directory.
/// This simplifies the path by handling `OUT_DIR` and `concat!` internally.
#[macro_export]
macro_rules! wgpu_include_slang_shader {
    ($file_name:literal) => {
        wgpu::include_wgsl!(concat!(env!("OUT_DIR"), "/shaders/", $file_name, ".wgsl"))
    };
}
