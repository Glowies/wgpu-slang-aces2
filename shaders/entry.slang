import "modules/aces2_output_transform.slang";
import "modules/aces_transform_common.slang";

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

  // assume input pixels are in AP0 color space
  float4 input_color = inputTexture[pixel_coords];
  // float3 output_color = outputTransform_fwd(input_color.rgb, odt_params);

  // apply gamma 2.2
  // output_color = pow(output_color, 1.0/2.2);

  // float4 output_color_with_alpha = float4(output_color, input_color.a);
  // outputTexture.Store(pixel_coords, output_color_with_alpha);

  outputTexture.Store(pixel_coords, input_color);
}
