use pollster::FutureExt;

fn main() {
    wgpu_slang_aces2::run().block_on().unwrap();
}
