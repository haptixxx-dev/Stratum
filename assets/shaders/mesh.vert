#version 450

// Vertex attributes
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;
layout(location = 3) in vec4 in_color;

// Uniforms - SDL_GPU requires set=1 for vertex shader uniform buffers
layout(set = 1, binding = 0) uniform MeshUniforms {
    mat4 mvp;
    mat4 model;
    vec4 color_tint;
} uniforms;

// Outputs to fragment shader
layout(location = 0) out vec3 frag_position;
layout(location = 1) out vec3 frag_normal;
layout(location = 2) out vec2 frag_uv;
layout(location = 3) out vec4 frag_color;

void main() {
    gl_Position = uniforms.mvp * vec4(in_position, 1.0);

    // Transform position and normal to world space for lighting
    frag_position = (uniforms.model * vec4(in_position, 1.0)).xyz;
    frag_normal = mat3(uniforms.model) * in_normal;
    frag_uv = in_uv;
    frag_color = in_color * uniforms.color_tint;
}
