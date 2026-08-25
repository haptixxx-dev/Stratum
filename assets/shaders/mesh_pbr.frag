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

// The scene uniform block, the analytic sky, the split-sum BRDF fit and the
// output transform all live here, SHARED with sky.frag. A surface lit by a
// different sky than the one drawn behind it is the "objects from another
// scene" look; sharing the function is what prevents it.
#include "sky_common.glsl"

const float PI = 3.14159265359;
const float EPSILON = 0.00001;

// ----------------------------------------------------------------------------
// Fragment uniform buffers -- set 3
// ----------------------------------------------------------------------------
// Scene is at binding 0 and is declared by sky_common.glsl.

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

// Cascaded shadow map -- set 2, binding 3 (kShadowSamplerSlot).
//
// NOT part of the material set. bind_material() binds three consecutive samplers
// from slot 0; this one is bound once per render pass, because it is frame state.
//
// A sampler2DShadow, not a plain sampler2D: the sampler is created with
// enable_compare, so texture() returns the RESULT of the depth comparison,
// filtered in hardware. That turns each of the taps below into a bilinear
// percentage-closer sample for the cost of one fetch -- a 3x3 kernel this way is
// as smooth as a much wider one done by hand.
//
// ONE 2D TEXTURE, CASCADES TILED SIDE BY SIDE, not a 2D array. SDL_GPU rejects
// array textures with DEPTH_STENCIL_TARGET usage outright -- see the assertion in
// SDL_CreateGPUTexture, "For array textures: usage must not contain
// DEPTH_STENCIL_TARGET" -- so an array is not available whatever its merits. The
// atlas turns out to be the better shape anyway: all the cascades are filled in a
// SINGLE render pass with one clear and a viewport change per tile, rather than
// one pass and one clear each.
layout(set = 2, binding = 3) uniform sampler2DShadow shadow_map;

// ----------------------------------------------------------------------------
// Shadow block -- set 3, binding 2 (kShadowUniformSlot)
// ----------------------------------------------------------------------------
//
// MUST match struct ShadowUniforms in src/renderer/gpu_renderer.hpp field for
// field and byte for byte.
//
// shadow_params.x is the live cascade COUNT, and zero is a legal value meaning
// "shadows off". That is what lets a caller that never renders a cascade -- the
// PBR shader test suite, or Simple shader mode -- push a zeroed block and get
// fully lit geometry rather than undefined comparisons against an uninitialised
// depth texture.
layout(set = 3, binding = 2) uniform ShadowUniforms {
    mat4 light_view_proj[4];    // world -> cascade clip, one per cascade
    vec4 cascade_texel_world;   // world size of one shadow texel, per cascade
    vec4 cascade_depth_bias;    // constant depth bias in clip [0,1], per cascade
    vec4 shadow_params;         // x = cascade count, y = normal offset scale,
                                // z = strength, w = 1 / shadow map size
    vec4 shadow_fade;           // x = fade start, y = fade end, z = PCF radius in
                                // texels, w = 1 / cascade count, the atlas tile
                                // width in UV
} shadows;

const int kMaxShadowCascades = 4;

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
layout(location = 6) in float frag_ao;

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

// linear_to_srgb() and tonemap_aces() come from sky_common.glsl, so this shader
// and the sky behind it cannot end up on different output transforms.

// Fog calculation
// fog_params: x = start, y = end, z = density, w = enabled (0 = off, 1 = linear, 2 = exponential, 3 = exponential squared)
//
// @param view_dir Normalised direction from the EYE towards this fragment. Only
//                 aerial perspective uses it.
vec3 apply_fog(vec3 color, float distance, vec3 view_dir) {
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

    // AERIAL PERSPECTIVE. Distance haze is the sky seen through the air in front
    // of the subject, so its colour is the sky's colour IN THAT DIRECTION -- pale
    // near the horizon, deeper higher up, warm towards the sun. A single authored
    // fog_color cannot do that, and the wide city shot showed exactly what it
    // costs: the far half of the map lost contrast and then fell to the
    // background colour instead of meeting a horizon.
    //
    // ibl_params.z blends between the authored colour (0) and the sky (1), so a
    // deliberately stylised fog is still reachable.
    const vec3 haze = mix(scene.fog_color.rgb, sky_radiance(view_dir),
                          clamp(scene.ibl_params.z, 0.0, 1.0));

    return mix(haze, color, fog_factor);
}

// ============================================================================
// Cascaded shadows
// ============================================================================

/**
 * @brief Fraction of the sun reaching this fragment. 1 = fully lit, 0 = shadowed
 *
 * @param world_pos Fragment position in world space
 * @param n         Shading normal, already flipped for double-sidedness
 * @param n_dot_l   Cosine of the angle to the sun, clamped at 0
 * @param view_dist Distance from the eye, for the far fade
 *
 * CASCADE SELECTION IS BY PROJECTION, NOT BY DISTANCE. The obvious scheme --
 * compare the view depth against a split table -- disagrees with how the
 * cascades are actually fitted: GPURenderer bounds each split's sub-frustum with
 * a SPHERE, so that rotating the camera cannot change the fitted volume and make
 * the shadows shimmer, and a sphere covers more than the slice it was fitted to.
 * Walking the cascades in order and taking the first one the fragment actually
 * projects inside uses that slack instead of fighting it, and it cannot select a
 * cascade that does not contain the fragment.
 *
 * The border inset matters: a fragment right at the edge of a cascade would have
 * its PCF kernel reach outside the map, where there is no depth, so it is pushed
 * to the next cascade out while it still has one.
 */
float sample_sun_shadow(vec3 world_pos, vec3 n, float n_dot_l, float view_dist) {
    const int cascade_count = int(shadows.shadow_params.x);
    if (cascade_count <= 0) {
        return 1.0;   // shadows disabled -- see the note on the block above
    }

    // Texel size WITHIN A TILE. The atlas is `cascade_count` tiles wide, so a step
    // of one texel in the atlas is this divided by the count along x, and exactly
    // this along y.
    const float texel = shadows.shadow_params.w;
    const float pcf_radius = max(shadows.shadow_fade.z, 0.0);
    const float tile_width = shadows.shadow_fade.w;   // 1 / cascade count
    const vec2 atlas_texel = vec2(texel * tile_width, texel);

    // One PCF kernel radius, plus a texel of margin, expressed in TILE UV. This
    // inset does two jobs: it keeps the kernel from reaching past the edge of the
    // shadow map, and -- because the tiles abut with no gutter -- it keeps a
    // fragment near a tile edge from bleeding into its NEIGHBOURING CASCADE, which
    // would show as a hard wrong-scale shadow seam.
    const float border = texel * (pcf_radius + 1.0);

    int cascade = -1;
    vec3 coord = vec3(0.0);

    for (int i = 0; i < kMaxShadowCascades; ++i) {
        if (i >= cascade_count) break;

        // NORMAL OFFSET, applied before projection and scaled by THIS cascade's
        // texel size. Offsetting along the surface normal is what removes the
        // self-shadowing acne that a pure depth bias can only trade against peter
        // panning: it moves the sample off the surface by roughly the world size
        // of the texel that is aliasing, which is exactly the error being
        // corrected. It is scaled by (1 - NdotL) because a surface facing the sun
        // head-on has no slope error to correct.
        const float offset = shadows.cascade_texel_world[i] * shadows.shadow_params.y *
                             clamp(1.0 - n_dot_l, 0.0, 1.0);
        const vec4 light_clip = shadows.light_view_proj[i] * vec4(world_pos + n * offset, 1.0);

        // w is 1 for the orthographic cascade projections, but dividing anyway
        // costs nothing and keeps this correct if a perspective cascade is ever
        // added for a spot light.
        const vec3 ndc = light_clip.xyz / light_clip.w;

        // V IS FLIPPED, and this is not a sign slip. SDL's Vulkan backend
        // rasterises with a NEGATIVE viewport height -- the standard trick that
        // makes Vulkan's y-down clip space behave like OpenGL's y-up -- so clip
        // y = -1 ends up at the BOTTOM row of the depth image, not the top. The
        // textbook `ndc.y * 0.5 + 0.5` therefore samples the mirror image of what
        // the cascade pass wrote.
        //
        // The failure it produces is quiet and convincing: shadows still appear,
        // still have the right shape, and still move with the sun -- they are just
        // reflected about the middle of the map, so they sit on the wrong side of
        // their casters. PbrShader.a_caster_shadows_the_ground_beneath_it_and_nowhere_else
        // places its caster off-centre in both axes precisely to catch it.
        const vec2 uv = vec2(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);

        const bool inside =
            all(greaterThanEqual(uv, vec2(border))) &&
            all(lessThanEqual(uv, vec2(1.0 - border))) &&
            ndc.z >= 0.0 && ndc.z <= 1.0;

        if (inside) {
            cascade = i;
            // Into atlas space: tile i occupies u in [i/count, (i+1)/count).
            const vec2 atlas_uv = vec2((float(i) + uv.x) * tile_width, uv.y);
            coord = vec3(atlas_uv, ndc.z - shadows.cascade_depth_bias[i]);
            break;
        }
    }

    // Past the last cascade there is no shadow data at all. Returning "lit" is
    // the only honest answer; the fade below is what keeps the boundary from
    // being a visible line.
    if (cascade < 0) {
        return 1.0;
    }

    // 3x3 PCF. Each tap is already bilinear-compared in hardware, so nine of them
    // cover the same area a much larger hand-rolled kernel would.
    float sum = 0.0;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            const vec2 tap = coord.xy + vec2(float(x), float(y)) * atlas_texel * pcf_radius;
            sum += texture(shadow_map, vec3(tap, coord.z));
        }
    }
    float lit = sum * (1.0 / 9.0);

    // Fade the whole effect out towards the far edge of the shadow range, so the
    // last cascade ends in a gradient rather than in an edge.
    const float fade_start = shadows.shadow_fade.x;
    const float fade_end = max(shadows.shadow_fade.y, fade_start + 1.0);
    const float fade = clamp((view_dist - fade_start) / (fade_end - fade_start), 0.0, 1.0);
    lit = mix(lit, 1.0, fade);

    // Strength below 1 lifts the shadow towards lit, which is a stylistic control
    // and not a physical one -- in reality a shadowed surface is lit by the sky,
    // and that is the ambient term, which shadows do not touch.
    return mix(1.0, lit, clamp(shadows.shadow_params.z, 0.0, 1.0));
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

    // View vector. scene.camera_position.xyz is the ONLY correct origin for
    // this: a map is placed in local metres around a projection origin the
    // camera is typically hundreds to thousands of metres away from, so the old
    // normalize(-frag_world_pos) -- "assume the camera is near the origin" --
    // aimed the specular lobe at the map origin AND, worse, fed the
    // double-sided flip below a vector pointing at the origin instead of at the
    // eye. Every facade on the far side of the origin had its normal inverted:
    // sunlit walls read dark and shadowed walls read lit.
    vec3 V = normalize(scene.camera_position.xyz - frag_world_pos);

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
    // Three factors, and each answers a different question. material.pbr_params.z
    // is what the MATERIAL says; orm.r is what the TEXTURE says, at texture
    // resolution; frag_ao is what the GEOMETRY says -- baked per vertex, so it is
    // the only one of the three that knows a wall is standing next to another
    // wall. Multiplying is right because they are independent occluders.
    float ao        = material.pbr_params.z * orm.r * clamp(frag_ao, 0.0, 1.0);
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

    // Direct lighting, occluded by the cascaded shadow map.
    //
    // The DIRECT term only. Ambient is the sky integrated over the hemisphere,
    // and a surface in shadow is still under that sky -- multiplying it here is
    // what makes shadows read as black holes rather than as shade.
    vec3 to_eye = scene.camera_position.xyz - frag_world_pos;
    float eye_distance = length(to_eye);
    float sun_visibility = sample_sun_shadow(frag_world_pos, N, NdotL, eye_distance);

    vec3 Lo = (diffuse + specular) * light_color * light_intensity * NdotL * sun_visibility;

    // ------------------------------------------------------------------------
    // 6. Ambient -- image-based, from the same sky that is drawn behind this
    // ------------------------------------------------------------------------
    // This replaced `albedo * ambient_intensity`, a single constant applied to
    // every surface at every orientation. That constant is what made the
    // underside of a bridge deck exactly as bright as its top, a north wall
    // exactly as bright as a south one, and every metal read as grey plastic.
    //
    // ambient_intensity survives as a MASTER SCALE on both lobes rather than as
    // the light itself, so the Ambient slider still means "how much fill", and
    // the sky parameters mean "what colour, from which direction".

    // Diffuse: cosine-weighted hemisphere irradiance at this normal. Ambient
    // occlusion attenuates this ONLY -- applying it to the direct term darkens
    // surfaces the sun demonstrably reaches.
    // kD from the SUN's half-vector Fresnel would be the wrong weight here: that
    // F was evaluated at HdotV for one specific light direction, and ambient
    // arrives from the whole hemisphere. The view-angle Fresnel is the standard
    // stand-in for the hemisphere average.
    vec3 F_ambient = F_Schlick(NdotV, F0);
    vec3 kD_ambient = (vec3(1.0) - F_ambient) * (1.0 - metallic);
    vec3 ambient_diffuse = kD_ambient * albedo * sky_irradiance(N) * ao;

    // Specular: the split sum. sky_prefiltered() stands in for a prefiltered
    // environment cube's mip chain and env_brdf_approx() for the BRDF
    // integration LUT, so this is a full image-based specular term with no
    // cubemap, no prefilter pass and no lookup texture. It is what stops asphalt
    // and glass reading as matte paper, and it is the only reason a metal can
    // look like a metal here: F0-tinted sky is ALL a metal reflects.
    vec3 R = reflect(-V, N);
    vec3 prefiltered = sky_prefiltered(R, roughness);
    vec3 env_brdf = env_brdf_approx(F0, roughness, NdotV);
    float spec_ao = specular_occlusion(NdotV, ao, roughness);
    vec3 ambient_specular = prefiltered * env_brdf * spec_ao * scene.ibl_params.x;

    vec3 ambient = (ambient_diffuse + ambient_specular) * ambient_intensity;

    // Final color. Emissive is added after the direct term and before fog and
    // tone mapping, so an emissive surface still fogs out with distance.
    vec3 color = ambient + Lo + albedo * emissive;

    // Apply fog based on distance from camera. eye_distance and to_eye were
    // already needed by the shadow lookup above; the view ray is their negation.
    color = apply_fog(color, eye_distance, -to_eye / max(eye_distance, EPSILON));

    // Exposure, then tone mapping and gamma correction. camera_position.w is the
    // exposure the Render Settings slider writes; it multiplies scene-referred
    // radiance BEFORE the tone curve, which is the only place it means anything
    // -- after tonemap_aces() the value is already clamped to 0..1 and the
    // slider would only wash out or crush.
    color *= scene.camera_position.w;
    color = tonemap_aces(color);
    color = linear_to_srgb(color);

    // The opaque pipeline ignores alpha; the decal pipeline blends with it, which
    // is what makes MaterialId::Markings read as paint rather than a plate.
    out_color = vec4(color, alpha);
}
