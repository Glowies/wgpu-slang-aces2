Texture2D<float4> inputTexture;
WTexture2D<float4> outputTexture;

[shader("compute")]
[numthreads(8, 8, 1)]
void main(uint3 global_invocation_id: SV_DispatchThreadID) {
  uint2 pixel_coords = global_invocation_id.xy;
  uint width, height;
  inputTexture.GetDimensions(width, height);

  if (pixel_coords.x >= width || pixel_coords.y >= height) {
    return;
  }

  // a simple copy operation
  outputTexture.Store(pixel_coords, inputTexture[pixel_coords]);
}
