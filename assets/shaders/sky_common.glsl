#ifndef STRATUM_SKY_COMMON_GLSL
#define STRATUM_SKY_COMMON_GLSL

// ============================================================================
// Shared sky, image-based lighting and tone mapping
// ============================================================================
//
// Included by BOTH assets/shaders/sky.frag, which paints the background, and
// assets/shaders/mesh_pbr.frag, which lights surfaces. That sharing is the whole
// point: an ambient term computed from a different sky than the one on screen is
// the classic "objects lit for a different scene" look, and it is exactly what
// the previous constant-grey ambient did.
//
// THERE IS NO PREFILTERED CUBEMAP AND NO BRDF LUT, deliberately. Both exist to
// make an ARBITRARY environment cheap to integrate. This environment is not
// arbitrary -- it is an analytic function of one direction and a handful of
// uniforms, so it can be evaluated directly at the reflection vector for less
// than a cubemap fetch, it costs no VRAM, it needs no prefilter pass, and it
// tracks the sun the instant the sun slider moves instead of a frame or a
// re-bake later. The split-sum BRDF term is Karis's analytic fit rather than a
// lookup texture, for the same reason.
//
// Y IS UP. See osm/coordinates.hpp: the whole world is Y-up local metres.
// ============================================================================

// ----------------------------------------------------------------------------
// Scene block -- set 3, binding 0 (kSceneUniformSlot)
// ----------------------------------------------------------------------------
//
// MUST match struct SceneUniforms in src/renderer/gpu_renderer.hpp field for
// field and byte for byte. Ten vec4s, 160 bytes. A mismatch is neither a compile
// error nor a validation error, only silent garbage, so edit the two together.
//
// Declared HERE rather than in each shader so the two cannot drift apart. Every
// shader that includes this file therefore declares the block, which is why the
// fragment stage of sky.frag and mesh_pbr.frag both report a uniform buffer at
// this slot.
layout(set = 3, binding = 0) uniform SceneUniforms {
    vec4 camera_position;   // xyz = position, w = exposure
    vec4 sun_direction;     // xyz = normalized direction TOWARD the sun, w = intensity
    vec4 sun_color;         // rgb = color, a = ambient master scale
    vec4 fog_params;        // x = start, y = end, z = density, w = mode
    vec4 fog_color;         // rgb = color, a = unused
    vec4 pbr_params;        // reserved, unread
    vec4 sky_zenith;        // rgb = zenith radiance, a = sky intensity
    vec4 sky_horizon;       // rgb = horizon radiance, a = zenith falloff exponent
    vec4 ground_color;      // rgb = ground bounce radiance, a = ground bounce intensity
    vec4 ibl_params;        // x = specular scale, y = sun disk cos(radius),
                            // z = aerial perspective, w = sun glow exponent
} scene;

const float SKY_PI = 3.14159265359;
const float SKY_EPSILON = 0.00001;

// ----------------------------------------------------------------------------
// The sky itself
// ----------------------------------------------------------------------------

/**
 * @brief Sky radiance along @p dir, EXCLUDING the sun's disk
 *
 * Excluding the disk is not an approximation, it is the thing that keeps the sun
 * from being counted twice: mesh_pbr.frag already integrates the sun as an
 * analytic directional light with a full Cook-Torrance lobe. If the disk were
 * left in here, every specular reflection of the sky would add a second sun on
 * top of the first one.
 *
 * The forward-scatter glow around the sun IS kept, because nothing else in the
 * renderer accounts for it and it is most of what makes a sky read as lit rather
 * than as a gradient.
 *
 * @param dir Normalised world-space direction, Y up.
 */
vec3 sky_radiance(vec3 dir) {
    const vec3  zenith    = scene.sky_zenith.rgb;
    const float intensity = scene.sky_zenith.a;
    const vec3  horizon   = scene.sky_horizon.rgb;
    // Guarded: a zero exponent makes pow() return 1 everywhere and flattens the
    // dome to a single colour, and a negative one blows up at the horizon.
    const float falloff   = max(scene.sky_horizon.a, 0.05);

    // Upper dome. pow() on the elevation rather than a straight lerp because a
    // linear gradient puts the horizon band far too high; the exponent is what
    // keeps the bright band hugging the horizon the way a real one does.
    const float up = clamp(dir.y, 0.0, 1.0);
    const vec3 dome = mix(horizon, zenith, pow(up, falloff));

    // Below the horizon the sky is not visible at all -- what is there is ground.
    // Fading over the first few degrees rather than switching at exactly y = 0
    // avoids a hard seam along the horizon line, which reads as a rendering
    // artefact even when the geometry covers it.
    const vec3 below = mix(horizon, scene.ground_color.rgb * scene.ground_color.a,
                           clamp(-dir.y * 6.0, 0.0, 1.0));

    vec3 base = mix(below, dome, step(0.0, dir.y));

    // Mie forward scattering. The exponent is the "haze tightness" the sky
    // parameters expose; the 0.35 is a scale that keeps the glow reading as air
    // rather than as a second light source.
    const float cos_sun = dot(dir, normalize(scene.sun_direction.xyz));
    const float glow_exp = max(scene.ibl_params.w, 1.0);
    const float glow = pow(max(cos_sun, 0.0), glow_exp);
    base += scene.sun_color.rgb * scene.sun_direction.w * glow * 0.35;

    return base * intensity;
}

/**
 * @brief Sky radiance along @p dir INCLUDING the sun's disk
 *
 * For the background pass only. Nothing that shades a surface may call this --
 * see the double-counting note on sky_radiance().
 */
vec3 sky_with_sun(vec3 dir) {
    vec3 base = sky_radiance(dir);

    const vec3 sun_dir = normalize(scene.sun_direction.xyz);
    const float cos_sun = dot(dir, sun_dir);
    // ibl_params.y is cos(angular radius). The real sun subtends about 0.53
    // degrees, so this is ~0.99996 and the smoothstep window has to be tighter
    // than that or the disk has no edge left.
    const float cos_radius = clamp(scene.ibl_params.y, 0.9, 0.9999999);
    const float disk = smoothstep(cos_radius, mix(cos_radius, 1.0, 0.35), cos_sun);

    // The disk is radiance, not irradiance: a small solid angle at very high
    // radiance. The tone curve is what brings it back to white.
    base += scene.sun_color.rgb * scene.sun_direction.w * disk * 12.0;
    return base;
}

/**
 * @brief Cosine-weighted sky irradiance arriving at a surface with normal @p n
 *
 * A two-lobe hemisphere: the upper dome above, the ground bounce below, lerped
 * on the normal's elevation. This is the term that replaced a single constant.
 * The constant is what made the underside of a bridge deck exactly as bright as
 * its top surface, and a roof exactly as bright as the wall under it.
 *
 * mix(horizon, zenith, 0.4) rather than 0.5: the cosine weighting of a real
 * hemisphere integral favours directions near the normal, and for an upward
 * normal that is the zenith, but the horizon band is brighter and covers more
 * solid angle. 0.4 is where the two pull even.
 */
vec3 sky_irradiance(vec3 n) {
    const vec3 dome = mix(scene.sky_horizon.rgb, scene.sky_zenith.rgb, 0.4) * scene.sky_zenith.a;
    const vec3 bounce = scene.ground_color.rgb * scene.ground_color.a;
    return mix(bounce, dome, clamp(n.y * 0.5 + 0.5, 0.0, 1.0));
}

/**
 * @brief Prefiltered specular radiance along @p r for a given @p roughness
 *
 * The stand-in for a prefiltered environment cube's mip chain. A mirror samples
 * the sky directly; a fully rough surface converges on the same hemisphere
 * average the diffuse term uses, which is the correct limit -- at roughness 1 the
 * GGX lobe IS the cosine hemisphere. Everything between is a lerp, which is the
 * same thing trilinear filtering between two mips would give.
 */
vec3 sky_prefiltered(vec3 r, float roughness) {
    return mix(sky_radiance(r), sky_irradiance(r), clamp(roughness, 0.0, 1.0));
}

// ----------------------------------------------------------------------------
// Split-sum BRDF, analytic
// ----------------------------------------------------------------------------

/**
 * @brief Environment BRDF (the split-sum's second term) without a LUT
 *
 * Karis's mobile approximation to the scale/bias pair a BRDF integration texture
 * would store. It is a polynomial in roughness and NdotV, accurate to well under
 * a per-cent over the range that matters, and it costs no texture unit, no
 * 512x512 RG16F target, and no one-time integration pass.
 */
vec3 env_brdf_approx(vec3 f0, float roughness, float n_dot_v) {
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
    vec4 r = roughness * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * n_dot_v)) * r.x + r.y;
    vec2 ab = vec2(-1.04, 1.04) * a004 + r.zw;
    return f0 * ab.x + ab.y;
}

/**
 * @brief Specular occlusion derived from diffuse ambient occlusion
 *
 * Frostbite's approximation. Applying the diffuse AO term straight to the
 * specular lobe kills reflections in every crevice, which reads as dirt; leaving
 * specular entirely unoccluded makes occluded surfaces glow. This narrows the
 * occlusion as the surface gets smoother, because a smooth surface reflects a
 * narrow cone that a nearby occluder is unlikely to cover.
 */
float specular_occlusion(float n_dot_v, float ao, float roughness) {
    return clamp(pow(n_dot_v + ao, exp2(-16.0 * roughness - 1.0)) - 1.0 + ao, 0.0, 1.0);
}

// ----------------------------------------------------------------------------
// Output transform
// ----------------------------------------------------------------------------

/// ACES filmic tone curve, on scene-referred radiance.
vec3 tonemap_aces(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

/// Linear to sRGB. The swapchain is a UNORM format, so this is not redundant
/// with a hardware encode -- dropping it washes the whole image out.
vec3 linear_to_srgb(vec3 color) {
    return pow(color, vec3(1.0 / 2.2));
}

#endif // STRATUM_SKY_COMMON_GLSL
