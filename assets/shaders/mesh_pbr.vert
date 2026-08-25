#version 450

// Vertex attributes
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;
layout(location = 3) in vec4 in_color;
layout(location = 4) in vec4 in_tangent;  // xyz = tangent, w = bitangent sign
// Baked ambient occlusion, 1 = open sky. Its own channel, not folded into
// in_color, because it must attenuate AMBIENT only: a vertex colour multiplies
// albedo and would darken direct sunlight with it.
layout(location = 5) in float in_ao;

// Uniforms - SDL_GPU requires set=1 for vertex shader uniform buffers
layout(set = 1, binding = 0) uniform MeshUniforms {
    mat4 mvp;
    mat4 model;
    mat4 normal_matrix;     // Precomputed inverse-transpose for correct normals
    vec4 color_tint;
    vec4 camera_position;   // xyz = camera pos, w = time
} uniforms;

// Outputs to fragment shader
layout(location = 0) out vec3 frag_world_pos;
layout(location = 1) out vec3 frag_normal;
layout(location = 2) out vec2 frag_uv;
layout(location = 3) out vec4 frag_color;
layout(location = 4) out vec3 frag_tangent;
layout(location = 5) out vec3 frag_bitangent;
layout(location = 6) out float frag_ao;

void main() {
    gl_Position = uniforms.mvp * vec4(in_position, 1.0);

    // World-space position for lighting
    frag_world_pos = (uniforms.model * vec4(in_position, 1.0)).xyz;
    
    // Transform normal using normal matrix (handles non-uniform scaling)
    mat3 normal_mat = mat3(uniforms.normal_matrix);
    frag_normal = normalize(normal_mat * in_normal);
    
    // Tangent space for normal mapping.
    //
    // HANDEDNESS, verified against Mesh::compute_tangents() in
    // src/renderer/mesh.hpp rather than assumed. That function stores
    //     w = (dot(cross(N, T), B) < 0) ? -1 : +1
    // for the per-triangle T and B it derives from the UV gradient, so the
    // bitangent it encodes is B = cross(N, T) * w. That is the reconstruction
    // below, and it is also the glTF / MikkTSpace convention. Reversing the
    // cross operands, or dropping w, mirrors every normal-map detail.
    frag_tangent = normalize(normal_mat * in_tangent.xyz);
    frag_bitangent = cross(frag_normal, frag_tangent) * in_tangent.w;
    
    frag_uv = in_uv;
    frag_color = in_color * uniforms.color_tint;
    frag_ao = in_ao;
}
