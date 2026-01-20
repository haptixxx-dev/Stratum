#version 450

// Vertex attributes
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;
layout(location = 3) in vec4 in_color;
layout(location = 4) in vec4 in_tangent;  // xyz = tangent, w = bitangent sign

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

void main() {
    gl_Position = uniforms.mvp * vec4(in_position, 1.0);

    // World-space position for lighting
    frag_world_pos = (uniforms.model * vec4(in_position, 1.0)).xyz;
    
    // Transform normal using normal matrix (handles non-uniform scaling)
    mat3 normal_mat = mat3(uniforms.normal_matrix);
    frag_normal = normalize(normal_mat * in_normal);
    
    // Tangent space for normal mapping
    frag_tangent = normalize(normal_mat * in_tangent.xyz);
    frag_bitangent = cross(frag_normal, frag_tangent) * in_tangent.w;
    
    frag_uv = in_uv;
    frag_color = in_color * uniforms.color_tint;
}
