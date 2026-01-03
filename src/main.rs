use clap::Parser;
use pollster::FutureExt;
use wgpu_slang_aces2::Args;

fn main() {
    let args = Args::parse();
    wgpu_slang_aces2::run(args).block_on().unwrap();
}
