#version 450

// ============================================================================
// Shadow cascade depth pass
// ============================================================================
//
// Position only. The pipeline declares ONE vertex attribute over a buffer whose
// pitch is the full stratum::Vertex, so normals, UVs, colours and tangents are
// skipped by the input assembler rather than fetched and discarded -- the shadow
// pass is bandwidth-bound and it runs once per cascade.
//
// No material, no alpha test, no per-material anything. Everything in the
// captured draw list is treated as an opaque caster, which is the correct
// approximation for this content: the only alpha-tested geometry here is road
// markings, which lie flat on the road surface and cast nothing.
// ============================================================================

layout(location = 0) in vec3 in_position;

// SDL_GPU: vertex uniform buffers are set 1, binding = the pushed slot.
layout(set = 1, binding = 0) uniform ShadowMeshUniforms {
    mat4 light_mvp;   // cascade light view-projection * model
} uniforms;

void main() {
    gl_Position = uniforms.light_mvp * vec4(in_position, 1.0);
}
