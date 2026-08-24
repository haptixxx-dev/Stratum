#version 450

// ============================================================================
// PBR Fragment Shader -- textured Cook-Torrance
// ============================================================================
//
// SDL_GPU FIXES THE DESCRIPTOR SET NUMBERS. From SDL_CreateGPUShader's
// documentation (external/SDL/include/SDL3/SDL_gpu.h), "For SPIR-V shaders":
//
//   For vertex shaders:    set 0 = sampled textures, storage textures, storage
//                                  buffers
//                          set 1 = uniform buffers
//   For fragment shaders:  set 2 = sampled textures, storage textures, storage
//                                  buffers
//                          set 3 = uniform buffers
//
// The set index is therefore a function of STAGE and RESOURCE KIND and is never
// a free choice. Within a set, `binding` IS the slot index passed to the
// matching SDL call:
//
//   SDL_PushGPUFragmentUniformData(cmd, slot, ...)     -> set 3, binding = slot
//   SDL_BindGPUFragmentSamplers(pass, first_slot, ...) -> set 2, binding =
//                                                          first_slot + i
//
// The counts declared in SDL_GPUShaderCreateInfo must match what is declared
// below: num_uniform_buffers = 2, num_samplers = 3. See kMaterialSamplerCount
// and kPbrFragmentUniformBufferCount in src/renderer/material_library.hpp.
//
// The combined `sampler2D` is deliberate: SDL_GPU supplies texture and sampler
// together in one SDL_GPUTextureSamplerBinding and pairs them by index, so
// separate `texture2D`/`sampler` objects would not bind.
// ============================================================================

const float PI = 3.14159265359;
const float EPSILON = 0.00001;

// ----------------------------------------------------------------------------
// Fragment uniform buffers -- set 3
// ----------------------------------------------------------------------------

// Scene block. Unchanged. Pushed to fragment uniform slot 0 (kSceneUniformSlot).
layout(set = 3, binding = 0) uniform SceneUniforms {
    vec4 camera_position;   // xyz = position, w = exposure
    vec4 sun_direction;     // xyz = normalized direction, w = intensity
    vec4 sun_color;         // rgb = color, a = ambient intensity
    vec4 fog_params;        // x = start, y = end, z = density, w = enabled
    vec4 fog_color;         // rgb = color, a = unused
    vec4 pbr_params;        // x = metallic, y = roughness, z = ao, w = unused
} scene;

// Per-draw material block. Pushed to fragment uniform slot 1
// (kMaterialUniformSlot). MUST match struct MaterialUniforms in
// src/renderer/material_library.hpp field for field and byte for byte -- three
// vec4s, 48 bytes. A mismatch is neither a compile error nor a validation error,
// only silent garbage, so edit the two together.
layout(set = 3, binding = 1) uniform MaterialUniforms {
    vec4 base_color;   // rgb tint, a alpha
    vec4 pbr_params;   // x metallic, y roughness, z ao, w emissive
    vec4 uv_params;    // xy uv_scale, zw unused (keep zero)
} material;

// ----------------------------------------------------------------------------
// Fragment sampled textures -- set 2
// ----------------------------------------------------------------------------
//
// albedo_map is sampled through an sRGB view (TextureFormat::RGBA8_SRGB), so the
// hardware decodes to linear here and no pow() is wanted. normal_map and orm_map
// are LINEAR views: an sRGB normal map is wrong in a way that looks plausible.
layout(set = 2, binding = 0) uniform sampler2D albedo_map;
layout(set = 2, binding = 1) uniform sampler2D normal_map;
layout(set = 2, binding = 2) uniform sampler2D orm_map;

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
    vec3 sun_dir_test = scene.sun_direction.xyz * 0.5 + 0.5; // remap -1..1 to 0..1

    // Test 2: Check if sun_direction.w (intensity) is valid
    float intensity_test = scene.sun_direction.w;

    // Test 3: Check pbr_params (metallic, roughness, ao)
    vec3 pbr_test = scene.pbr_params.xyz;

    out_color = vec4(sun_dir_test, 1.0);

    // Alternative tests (uncomment one at a time):
    // out_color = vec4(vec3(intensity_test), 1.0);  // Should be ~1.0 = white
    // out_color = vec4(pbr_test, 1.0);              // Should show metallic/roughness/ao
    // out_color = vec4(scene.sun_color.rgb, 1.0);   // Should be warm white
    return;
#endif

    // ------------------------------------------------------------------------
    // 1. UVs
    // ------------------------------------------------------------------------
    // frag_uv already carries the plan's metre-based tiling UVs
    // (U = lateral metres / tile_u_metres, V = arc-length metres /
    // tile_v_metres) and, for MaterialId::Markings, an atlas sub-rect instead.
    // uv_params.xy is a CORRECTION on top of that, normally 1.0, for a texture
    // whose physical size does not match the convention. Markings materials
    // must leave it at 1.0: scaling an atlas sub-rect samples the neighbour.
    vec2 uv = frag_uv * material.uv_params.xy;

    // ------------------------------------------------------------------------
    // 2. Albedo and alpha
    // ------------------------------------------------------------------------
    // No "if albedo is near-black use grey" fallback: that hack stood in for the
    // absent texture path and would now hide a legitimately black material. A
    // material with no albedo map samples the 1x1 opaque white default instead.
    vec4 albedo_tex = texture(albedo_map, uv);
    vec3 albedo = frag_color.rgb * material.base_color.rgb * albedo_tex.rgb;
    float alpha = frag_color.a * material.base_color.a * albedo_tex.a;

    // ------------------------------------------------------------------------
    // 3. Normal mapping
    // ------------------------------------------------------------------------
    // Tangent frame convention, verified against Mesh::compute_tangents()
    // (src/renderer/mesh.hpp): it stores the handedness as
    //     w = (dot(cross(N, T), B) < 0) ? -1 : +1
    // so the bitangent it means is B = cross(N, T) * w -- the glTF/MikkTSpace
    // convention -- and mesh_pbr.vert reconstructs exactly that. Nothing here
    // re-derives it; frag_bitangent is used as interpolated.
    vec3 Ng = normalize(frag_normal);
    vec3 T  = normalize(frag_tangent);
    vec3 B  = frag_bitangent;   // normalised below, AFTER the degeneracy guard

    vec3 V = normalize(-frag_world_pos);  // Assume camera near origin for simplicity

    // Double-sided support. Flipping the normal alone would leave (T, B, N)
    // left-handed and mirror every normal-map detail, so the bitangent flips
    // with it.
    if (dot(Ng, V) < 0.0) {
        Ng = -Ng;
        B = -B;
    }

    // Re-orthogonalise the tangent against the (possibly flipped) normal;
    // interpolation across a triangle does not preserve orthogonality.
    T = T - Ng * dot(Ng, T);
    float t_len_sq = dot(T, T);

    // Both halves of the frame have to survive degeneracy, and the bitangent is
    // the half that actually degenerates in this project. Only the road and
    // terrain builders call Mesh::compute_tangents(); src/osm/mesh_builder.cpp
    // does not, so building and area meshes keep Vertex's default tangent
    // (1, 0, 0, 1). Every wall normal there is normalize(cross(up, edge)) with the
    // edge in the XZ plane, so an exactly north-south footprint edge -- one pair of
    // nodes at the same longitude, which any hand-drawn or axis-aligned footprint
    // has -- gives a normal of exactly (+-1, 0, 0). cross(N, T) is then EXACTLY
    // zero, every term being a product with an exact zero, and normalize(vec3(0))
    // is NaN. Leaving that NaN in B propagated through mat3(T, B, Ng) -- 0 * NaN is
    // NaN under IEEE, and glslc -O does not enable fast-math -- so the whole
    // shading normal, and the entire wall, went NaN.
    //
    // A degenerate tangent gets an arbitrary but VALID frame rather than a zero
    // one. Its orientation only rotates tangent-space detail, and a mesh with no
    // tangents is also a mesh with no normal map: it samples the flat-normal
    // default, which decodes to (0, 0, 1) and is rotation-invariant.
    if (t_len_sq > EPSILON) {
        T *= inversesqrt(t_len_sq);
    } else {
        vec3 seed = (abs(Ng.z) < 0.999) ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
        T = normalize(cross(seed, Ng));
    }

    float b_len_sq = dot(B, B);
    // NaN fails `> EPSILON`, so a bitangent that arrived NaN takes the rebuild too.
    B = (b_len_sq > EPSILON) ? B * inversesqrt(b_len_sq) : normalize(cross(Ng, T));

    // The flat-normal fallback texture decodes to (0, 0, 1), i.e. no
    // perturbation, so no branch on "has a normal map" is needed.
    vec3 n_tangent_space = texture(normal_map, uv).xyz * 2.0 - 1.0;
    vec3 N = normalize(mat3(T, B, Ng) * n_tangent_space);

    // ------------------------------------------------------------------------
    // 4. ORM
    // ------------------------------------------------------------------------
    // Occlusion / Roughness / Metallic, multiplicative on the authored scalars,
    // so the default unit ORM leaves them exactly as the material declared -- which
    // requires all THREE channels of that default to be 255, blue included. These
    // are the only source of the three values: scene.pbr_params is not read by this
    // shader at all, and the Render Settings sliders that used to drive it are gone.
    vec3 orm = texture(orm_map, uv).rgb;
    float ao        = material.pbr_params.z * orm.r;
    float roughness = clamp(material.pbr_params.y * orm.g, 0.04, 1.0);
    float metallic  = clamp(material.pbr_params.x * orm.b, 0.0, 1.0);
    float emissive  = material.pbr_params.w;

    // ------------------------------------------------------------------------
    // 5. Cook-Torrance
    // ------------------------------------------------------------------------
    vec3 light_dir = normalize(scene.sun_direction.xyz);
    vec3 light_color = scene.sun_color.rgb;
    float light_intensity = scene.sun_direction.w;
    float ambient_intensity = scene.sun_color.a;

    vec3 L = light_dir;
    vec3 H = normalize(V + L);

    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 0.001);
    float NdotH = max(dot(N, H), 0.0);
    float HdotV = max(dot(H, V), 0.0);

    // F0 (reflectance at normal incidence)
    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    float D = D_GGX(NdotH, roughness);
    float G = G_Smith(NdotV, NdotL, roughness);
    vec3 F = F_Schlick(HdotV, F0);

    vec3 specular = (D * G * F) / max(4.0 * NdotV * NdotL, EPSILON);

    // Diffuse (energy conserving)
    vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);
    vec3 diffuse = kD * albedo / PI;

    // Direct lighting
    vec3 Lo = (diffuse + specular) * light_color * light_intensity * NdotL;

    // Ambient. Ambient occlusion attenuates this term ONLY -- applying it to the
    // direct term darkens surfaces the sun actually reaches.
    vec3 ambient = albedo * ambient_intensity * ao;

    // Final color. Emissive is added after the direct term and before fog and
    // tone mapping, so an emissive surface still fogs out with distance.
    vec3 color = ambient + Lo + albedo * emissive;

    // Apply fog based on distance from camera
    float frag_distance = length(frag_world_pos - scene.camera_position.xyz);
    color = apply_fog(color, frag_distance);

    // Tone mapping and gamma correction
    color = tonemap_aces(color);
    color = linear_to_srgb(color);

    // The opaque pipeline ignores alpha; the decal pipeline blends with it, which
    // is what makes MaterialId::Markings read as paint rather than a plate.
    out_color = vec4(color, alpha);
}
