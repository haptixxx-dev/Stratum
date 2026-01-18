#version 450

// Inputs from vertex shader
layout(location = 0) in vec3 frag_position;
layout(location = 1) in vec3 frag_normal;
layout(location = 2) in vec2 frag_uv;
layout(location = 3) in vec4 frag_color;

// Output color
layout(location = 0) out vec4 out_color;

void main() {
    // Simple directional lighting
    vec3 light_dir = normalize(vec3(0.5, 1.0, 0.3));
    vec3 normal = normalize(frag_normal);

    // Basic diffuse lighting
    float ndotl = max(dot(normal, light_dir), 0.0);
    float ambient = 0.3;
    float diffuse = ndotl * 0.7;

    vec3 lit_color = frag_color.rgb * (ambient + diffuse);

    out_color = vec4(lit_color, frag_color.a);
}
