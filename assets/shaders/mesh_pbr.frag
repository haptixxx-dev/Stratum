#version 450

// ============================================================================
// PBR Fragment Shader - Testing uniform binding
// ============================================================================

const float PI = 3.14159265359;
const float EPSILON = 0.00001;

// Scene uniforms - SDL_GPU requires set = 3 for fragment shader uniform buffers
// (set 0 = frag samplers/textures, set 1 = vert uniforms, set 2 = frag samplers, set 3 = frag uniforms)
layout(set = 3, binding = 0) uniform SceneUniforms {
    vec4 camera_position;   // xyz = position, w = exposure
    vec4 sun_direction;     // xyz = normalized direction, w = intensity
    vec4 sun_color;         // rgb = color, a = ambient intensity
    vec4 fog_params;        // x = start, y = end, z = density, w = enabled
    vec4 fog_color;         // rgb = color, a = unused
    vec4 pbr_params;        // x = metallic, y = roughness, z = ao, w = unused
} scene;

// DEBUG: Set to 1 to show raw uniform values as colors
#define DEBUG_UNIFORMS 0

// ============================================================================
// Inputs from vertex shader
// ============================================================================
layout(location = 0) in vec3 frag_world_pos;
layout(location = 1) in vec3 frag_normal;
layout(location = 2) in vec2 frag_uv;
layout(location = 3) in vec4 frag_color;
layout(location = 4) in vec3 frag_tangent;
layout(location = 5) in vec3 frag_bitangent;

// ============================================================================
// Output
// ============================================================================
layout(location = 0) out vec4 out_color;

// ============================================================================
// PBR Functions
// ============================================================================

// GGX Normal Distribution
float D_GGX(float NdotH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH2 = NdotH * NdotH;
    float denom = NdotH2 * (a2 - 1.0) + 1.0;
    denom = PI * denom * denom;
    return a2 / max(denom, EPSILON);
}

// Schlick-GGX Geometry
float G_SchlickGGX(float NdotV, float roughness) {
    float k = (roughness * roughness) / 2.0;
    return NdotV / (NdotV * (1.0 - k) + k + EPSILON);
}

float G_Smith(float NdotV, float NdotL, float roughness) {
    return G_SchlickGGX(NdotV, roughness) * G_SchlickGGX(NdotL, roughness);
}

// Fresnel Schlick
vec3 F_Schlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}

// Linear to sRGB
vec3 linear_to_srgb(vec3 color) {
    return pow(color, vec3(1.0 / 2.2));
}

// ACES tone mapping
vec3 tonemap_aces(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

// Fog calculation
// fog_params: x = start, y = end, z = density, w = enabled (0 = off, 1 = linear, 2 = exponential, 3 = exponential squared)
vec3 apply_fog(vec3 color, float distance) {
    if (scene.fog_params.w < 0.5) {
        return color; // Fog disabled
    }
    
    float fog_start = scene.fog_params.x;
    float fog_end = scene.fog_params.y;
    float fog_density = scene.fog_params.z;
    float fog_mode = scene.fog_params.w;
    
    float fog_factor = 0.0;
    
    if (fog_mode < 1.5) {
        // Linear fog
        fog_factor = clamp((fog_end - distance) / (fog_end - fog_start), 0.0, 1.0);
    } else if (fog_mode < 2.5) {
        // Exponential fog
        fog_factor = exp(-fog_density * distance);
    } else {
        // Exponential squared fog
        float d = fog_density * distance;
        fog_factor = exp(-d * d);
    }
    
    return mix(scene.fog_color.rgb, color, fog_factor);
}

// ============================================================================
// Main
// ============================================================================
void main() {
#if DEBUG_UNIFORMS
    // Debug mode: visualize uniform values to check if they're being received
    // If uniforms work: should see colors based on uniform values
    // If uniforms broken: will see black or garbage

    // Test 1: Show sun_direction.xyz as color (should be ~(0.5, 0.8, 0.3) normalized)
    // If working: orange-ish color
    // If broken: black or random noise
    vec3 sun_dir_test = scene.sun_direction.xyz * 0.5 + 0.5; // remap -1..1 to 0..1

    // Test 2: Check if sun_direction.w (intensity) is valid
    float intensity_test = scene.sun_direction.w;

    // Test 3: Check pbr_params (metallic, roughness, ao)
    vec3 pbr_test = scene.pbr_params.xyz;

    // Show sun direction as color - if this shows proper colors, uniforms work
    // Expected: warm color since sun_direction should be normalized (0.5, 0.8, 0.3)
    out_color = vec4(sun_dir_test, 1.0);

    // Alternative tests (uncomment one at a time):
    // out_color = vec4(vec3(intensity_test), 1.0);  // Should be ~1.0 = white
    // out_color = vec4(pbr_test, 1.0);              // Should show metallic/roughness/ao
    // out_color = vec4(scene.sun_color.rgb, 1.0);  // Should be warm white
    return;
#endif

    // Albedo from vertex color
    vec3 albedo = frag_color.rgb;
    if (dot(albedo, albedo) < 0.0001) {
        albedo = vec3(0.5);
    }

    // Get values from uniforms
    vec3 light_dir = normalize(scene.sun_direction.xyz);
    vec3 light_color = scene.sun_color.rgb;
    float light_intensity = scene.sun_direction.w;
    float ambient_intensity = scene.sun_color.a;
    float metallic = scene.pbr_params.x;
    float roughness = max(scene.pbr_params.y, 0.04); // clamp to avoid divide by zero

    // Normal (with simple double-sided support)
    vec3 N = normalize(frag_normal);
    vec3 V = normalize(-frag_world_pos);  // Assume camera near origin for simplicity
    if (dot(N, V) < 0.0) {
        N = -N;
    }

    // Light and half vector
    vec3 L = light_dir;
    vec3 H = normalize(V + L);

    // Dot products
    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 0.001);
    float NdotH = max(dot(N, H), 0.0);
    float HdotV = max(dot(H, V), 0.0);

    // F0 (reflectance at normal incidence)
    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    // Cook-Torrance BRDF
    float D = D_GGX(NdotH, roughness);
    float G = G_Smith(NdotV, NdotL, roughness);
    vec3 F = F_Schlick(HdotV, F0);

    vec3 specular = (D * G * F) / max(4.0 * NdotV * NdotL, EPSILON);

    // Diffuse (energy conserving)
    vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);
    vec3 diffuse = kD * albedo / PI;

    // Direct lighting
    vec3 Lo = (diffuse + specular) * light_color * light_intensity * NdotL;

    // Ambient
    vec3 ambient = albedo * ambient_intensity;

    // Final color
    vec3 color = ambient + Lo;

    // Apply fog based on distance from camera
    float frag_distance = length(frag_world_pos - scene.camera_position.xyz);
    color = apply_fog(color, frag_distance);

    // Tone mapping and gamma correction
    color = tonemap_aces(color);
    color = linear_to_srgb(color);

    out_color = vec4(color, frag_color.a);
}
