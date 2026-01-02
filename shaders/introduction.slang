StructuredBuffer<uint> input;
RWStructuredBuffer<uint> output;

[shader("compute")]
[numthreads(64, 1, 1)]
void main(uint3 global_invocation_id: SV_DispatchThreadID) {
  uint index = global_invocation_id.x;
  uint total;
  uint stride;
  input.GetDimensions(total, stride);

  if (index >= total) {
    return;
  }

  // a simple copy operation
  output[global_invocation_id.x] = input[global_invocation_id.x];
}
