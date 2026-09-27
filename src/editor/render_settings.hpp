// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file render_settings.hpp
 * @brief Every lighting, sky, fog and shadow value the Render Settings panel edits
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This used to be a dozen `static` locals inside
 * `Editor::draw_render_settings()`: real state, but state that only existed
 * while that one function was on the stack, seen by nothing else, and reset to
 * its literal defaults every time the process restarted. Lifting it out into a
 * plain-data struct gives it three things statics cannot have -- a place to live
 * on `Editor` so other code can read it, a JSON encoding so it can survive a
 * restart, and a single `push_to()` that applies all of it to a renderer in one
 * call instead of four separately-guarded `if (changed)` blocks.
 *
 * The defaults below are copied from the static locals `draw_render_settings()`
 * used to declare (and, for the shadow fields, from `ShadowConfig`'s own
 * defaults in gpu_renderer.hpp) -- not rederived, copied, so a diff against the
 * panel's old initial values is the whole review.
 */

#pragma once

#include <glm/glm.hpp>
#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <filesystem>

namespace stratum {

class GPURenderer;

/**
 * @brief Plain data: the sun, sky/IBL, fog and shadow controls, nothing else
 *
 * Deliberately excludes shader mode, exposure, wireframe, MSAA and the culling
 * settings: none of those were `static` locals in the panel (they read and
 * write the renderer, or `Editor`'s own members, directly), so none of them
 * belongs here.
 */
struct RenderSettings {
    // ------------------------------------------------------------------------
    // Sun / ambient lighting
    // ------------------------------------------------------------------------
    float sun_azimuth_deg = 45.0f;
    float sun_height_deg = 60.0f;
    /// ~PI: cancels the shader's albedo / PI diffuse term. See set_scene_lighting.
    float sun_intensity = 3.14159265f;
    /// Master scale on the sky-derived ambient. 1.0 means "the sky as authored".
    float ambient_intensity = 1.0f;
    glm::vec3 sun_tint = glm::vec3(1.0f, 0.98f, 0.95f);

    // ------------------------------------------------------------------------
    // Sky and image-based lighting
    // ------------------------------------------------------------------------
    glm::vec3 sky_zenith = glm::vec3(0.16f, 0.30f, 0.62f);
    glm::vec3 sky_horizon = glm::vec3(0.56f, 0.68f, 0.86f);
    glm::vec3 ground_bounce = glm::vec3(0.14f, 0.14f, 0.12f);
    float sky_intensity = 1.0f;
    float ground_intensity = 0.7f;
    float sky_falloff = 0.45f;
    float ibl_specular = 1.0f;
    float sun_angular_deg = 0.53f;
    float aerial_perspective = 1.0f;
    float sun_glow = 64.0f;

    // ------------------------------------------------------------------------
    // Fog
    // ------------------------------------------------------------------------
    /// 0 = off, 1 = linear, 2 = exponential, 3 = exponential squared.
    int fog_mode = 2;
    float fog_start = 50.0f;
    float fog_end = 4000.0f;
    float fog_density = 0.00035f;
    glm::vec3 fog_color = glm::vec3(0.62f, 0.72f, 0.85f);

    // ------------------------------------------------------------------------
    // Shadows
    // ------------------------------------------------------------------------
    // Mirrors ShadowConfig's own field defaults in renderer/gpu_renderer.hpp,
    // deliberately duplicated rather than reused: this header must not include
    // gpu_renderer.hpp, which pulls in SDL3/SDL.h, or every translation unit
    // that includes editor.hpp (nearly all of them) would too. render_settings.cpp
    // includes the real header and builds a genuine ShadowConfig from these in
    // push_to().
    bool shadows_enabled = true;
    int shadow_cascade_count = 3;
    uint32_t shadow_map_size = 2048;
    float shadow_max_distance = 800.0f;
    float shadow_split_lambda = 0.85f;
    float shadow_normal_offset = 2.0f;
    float shadow_depth_bias_metres = 0.05f;
    float shadow_strength = 1.0f;
    float shadow_pcf_radius = 1.0f;

    /**
     * @brief Set once a control above changed and not yet applied
     *
     * Replaces the three separate `sun_pushed` / `sky_pushed` / `fog_pushed`
     * first-frame guards `draw_render_settings()` used to keep as its own
     * `static` locals. Those existed only to reconcile two separate sources of
     * truth -- the widgets' own defaults and the renderer's startup defaults --
     * on the first frame the panel drew. With one struct that is pushed once,
     * explicitly, before the first frame ever renders (see
     * `Editor::set_renderer()`), the panel only has to say "something here
     * changed"; this flag is that one signal. Not persisted: it is a live
     * editing signal, not settings state, and a freshly loaded or
     * freshly-defaulted RenderSettings has nothing pending to apply.
     */
    bool dirty = false;

    /**
     * @brief Apply every value above to @p renderer, exactly as the panel used to
     *
     * Safe to call before the first frame renders (that is the point of it --
     * see `Editor::set_renderer()`): it also marks the renderer's own one-time
     * scene-lighting seed as already done, so `GPURenderer::update_scene_uniforms()`
     * does not overwrite what was just pushed with its own hardcoded defaults on
     * the very next render pass.
     */
    void push_to(GPURenderer& renderer) const;

    /**
     * @brief Load from @p path, leaving every field untouched on any failure
     * @return false if the file is absent, unreadable, or not a JSON object;
     *         the struct is unchanged in that case, and the caller keeps
     *         whatever it already had (its compiled-in defaults, typically).
     */
    bool load(const std::filesystem::path& path);

    /**
     * @brief Write to @p path as JSON, creating parent directories if needed
     * @return false on any I/O or write failure. Never throws.
     */
    bool save(const std::filesystem::path& path) const;
};

/// nlohmann's ADL hook: lets `nlohmann::json(settings)` and
/// `j.get<RenderSettings>()` work, and is what RenderSettings::save()/load() use.
void to_json(nlohmann::json& j, const RenderSettings& settings);

/// Fields absent from @p j, or present with the wrong JSON type, leave the
/// matching field of @p settings untouched rather than throwing -- the same
/// "keep what you had" contract as RenderSettings::load().
void from_json(const nlohmann::json& j, RenderSettings& settings);

} // namespace stratum
