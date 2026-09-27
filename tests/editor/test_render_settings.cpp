// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_render_settings.cpp
 * @brief RenderSettings' JSON encoding: what a restart must get back exactly
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * RenderSettings itself needs no GPU device -- it is the plain-data lighting,
 * sky, fog and shadow values the Render Settings panel edits, formerly a dozen
 * `static` locals inside draw_render_settings() and now a struct that can be
 * saved and loaded. This suite lives in stratum_gpu_tests rather than
 * stratum_tests because render_settings.cpp includes renderer/gpu_renderer.hpp
 * for push_to() and ShadowConfig, which pulls in SDL through
 * stratum_editor_lib -- but nothing below creates a device, a window or an SDL
 * subsystem, so it is registered in the non-device suite list next to
 * MaterialBinding and MaterialUniforms.
 *
 * Run this suite with:
 * @code
 *     ./stratum_gpu_tests RenderSettings
 * @endcode
 */

#include "framework.hpp"

#include "editor/render_settings.hpp"
#include "renderer/gpu_renderer.hpp"

#include <nlohmann/json.hpp>

#include <cstdio>
#include <filesystem>
#include <system_error>

using stratum::GPURenderer;
using stratum::RenderSettings;

namespace {

/// Build-tree scratch directory; never the source tree. Created on demand.
[[nodiscard]] std::filesystem::path scratch(const char* filename) {
    std::filesystem::path dir{STRATUM_TEST_TMP_DIR};
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return dir / filename;
}

/// Every field set to something that is NOT its default, so a round trip that
/// silently fell back to a default would show up as a mismatch rather than a
/// coincidental pass.
[[nodiscard]] RenderSettings distinctive_settings() {
    RenderSettings s;
    s.sun_azimuth_deg = 123.0f;
    s.sun_height_deg = 17.5f;
    s.sun_intensity = 2.5f;
    s.ambient_intensity = 0.4f;
    s.sun_tint = glm::vec3(0.8f, 0.5f, 0.2f);

    s.sky_zenith = glm::vec3(0.05f, 0.1f, 0.9f);
    s.sky_horizon = glm::vec3(0.9f, 0.4f, 0.1f);
    s.ground_bounce = glm::vec3(0.3f, 0.2f, 0.1f);
    s.sky_intensity = 2.0f;
    s.ground_intensity = 1.3f;
    s.sky_falloff = 0.9f;
    s.ibl_specular = 0.25f;
    s.sun_angular_deg = 3.5f;
    s.aerial_perspective = 0.6f;
    s.sun_glow = 128.0f;

    s.fog_mode = 1;
    s.fog_start = 12.0f;
    s.fog_end = 999.0f;
    s.fog_density = 0.0021f;
    s.fog_color = glm::vec3(0.1f, 0.2f, 0.3f);

    s.shadows_enabled = false;
    s.shadow_cascade_count = 2;
    s.shadow_map_size = 4096;
    s.shadow_max_distance = 1500.0f;
    s.shadow_split_lambda = 0.25f;
    s.shadow_normal_offset = 5.5f;
    s.shadow_depth_bias_metres = 0.2f;
    s.shadow_strength = 0.7f;
    s.shadow_pcf_radius = 2.5f;

    // Deliberately left true: dirty is a live editing signal, not persisted
    // state, and every check below asserts it never survives a round trip.
    s.dirty = true;
    return s;
}

void check_equal(const RenderSettings& a, const RenderSettings& b) {
    CHECK_EQ(a.sun_azimuth_deg, b.sun_azimuth_deg);
    CHECK_EQ(a.sun_height_deg, b.sun_height_deg);
    CHECK_EQ(a.sun_intensity, b.sun_intensity);
    CHECK_EQ(a.ambient_intensity, b.ambient_intensity);
    CHECK_EQ(a.sun_tint.x, b.sun_tint.x);
    CHECK_EQ(a.sun_tint.y, b.sun_tint.y);
    CHECK_EQ(a.sun_tint.z, b.sun_tint.z);

    CHECK_EQ(a.sky_zenith.x, b.sky_zenith.x);
    CHECK_EQ(a.sky_zenith.y, b.sky_zenith.y);
    CHECK_EQ(a.sky_zenith.z, b.sky_zenith.z);
    CHECK_EQ(a.sky_horizon.x, b.sky_horizon.x);
    CHECK_EQ(a.sky_horizon.y, b.sky_horizon.y);
    CHECK_EQ(a.sky_horizon.z, b.sky_horizon.z);
    CHECK_EQ(a.ground_bounce.x, b.ground_bounce.x);
    CHECK_EQ(a.ground_bounce.y, b.ground_bounce.y);
    CHECK_EQ(a.ground_bounce.z, b.ground_bounce.z);
    CHECK_EQ(a.sky_intensity, b.sky_intensity);
    CHECK_EQ(a.ground_intensity, b.ground_intensity);
    CHECK_EQ(a.sky_falloff, b.sky_falloff);
    CHECK_EQ(a.ibl_specular, b.ibl_specular);
    CHECK_EQ(a.sun_angular_deg, b.sun_angular_deg);
    CHECK_EQ(a.aerial_perspective, b.aerial_perspective);
    CHECK_EQ(a.sun_glow, b.sun_glow);

    CHECK_EQ(a.fog_mode, b.fog_mode);
    CHECK_EQ(a.fog_start, b.fog_start);
    CHECK_EQ(a.fog_end, b.fog_end);
    CHECK_EQ(a.fog_density, b.fog_density);
    CHECK_EQ(a.fog_color.x, b.fog_color.x);
    CHECK_EQ(a.fog_color.y, b.fog_color.y);
    CHECK_EQ(a.fog_color.z, b.fog_color.z);

    CHECK_EQ(a.shadows_enabled, b.shadows_enabled);
    CHECK_EQ(a.shadow_cascade_count, b.shadow_cascade_count);
    CHECK_EQ(a.shadow_map_size, b.shadow_map_size);
    CHECK_EQ(a.shadow_max_distance, b.shadow_max_distance);
    CHECK_EQ(a.shadow_split_lambda, b.shadow_split_lambda);
    CHECK_EQ(a.shadow_normal_offset, b.shadow_normal_offset);
    CHECK_EQ(a.shadow_depth_bias_metres, b.shadow_depth_bias_metres);
    CHECK_EQ(a.shadow_strength, b.shadow_strength);
    CHECK_EQ(a.shadow_pcf_radius, b.shadow_pcf_radius);
}

} // namespace

// ============================================================================
// Defaults
// ============================================================================

/// Copied, not rederived, from the `static` locals draw_render_settings() used
/// to declare -- see render_settings.hpp's file comment. A default drifting from
/// what ships today would change the look of every fresh install silently.
TEST(RenderSettings, defaults_match_the_panels_old_static_locals) {
    RenderSettings s;
    CHECK_EQ(s.sun_azimuth_deg, 45.0f);
    CHECK_EQ(s.sun_height_deg, 60.0f);
    CHECK_NEAR(s.sun_intensity, 3.14159265f, 1e-6);
    CHECK_EQ(s.ambient_intensity, 1.0f);

    CHECK_EQ(s.sky_intensity, 1.0f);
    CHECK_EQ(s.ground_intensity, 0.7f);
    CHECK_EQ(s.sky_falloff, 0.45f);
    CHECK_EQ(s.ibl_specular, 1.0f);
    CHECK_NEAR(s.sun_angular_deg, 0.53f, 1e-6);
    CHECK_EQ(s.aerial_perspective, 1.0f);
    CHECK_EQ(s.sun_glow, 64.0f);

    CHECK_EQ(s.fog_mode, 2);
    CHECK_EQ(s.fog_start, 50.0f);
    CHECK_EQ(s.fog_end, 4000.0f);
    CHECK_NEAR(s.fog_density, 0.00035f, 1e-9);

    // ShadowConfig's own defaults, in renderer/gpu_renderer.hpp.
    CHECK_TRUE(s.shadows_enabled);
    CHECK_EQ(s.shadow_cascade_count, 3);
    CHECK_EQ(s.shadow_map_size, uint32_t{2048});
    CHECK_EQ(s.shadow_max_distance, 800.0f);
    CHECK_EQ(s.shadow_split_lambda, 0.85f);
    CHECK_EQ(s.shadow_normal_offset, 2.0f);
    CHECK_EQ(s.shadow_depth_bias_metres, 0.05f);
    CHECK_EQ(s.shadow_strength, 1.0f);
    CHECK_EQ(s.shadow_pcf_radius, 1.0f);

    CHECK_FALSE(s.dirty);
}

// ============================================================================
// to_json / from_json
// ============================================================================

/// Every field survives a straight to_json()/from_json() pass with no file
/// involved, EXCEPT dirty, which the encoding does not carry at all.
TEST(RenderSettings, to_json_from_json_round_trips_every_field) {
    const RenderSettings original = distinctive_settings();

    nlohmann::json doc;
    stratum::to_json(doc, original);

    RenderSettings restored;
    stratum::from_json(doc, restored);

    check_equal(original, restored);
    CHECK_FALSE(restored.dirty);
}

/// dirty is a live editing signal, not settings state, so it must not appear in
/// the encoding at all -- not even as false.
TEST(RenderSettings, dirty_is_not_serialized) {
    nlohmann::json doc;
    stratum::to_json(doc, distinctive_settings());
    CHECK_FALSE(doc.contains("dirty"));
}

/// A field missing from the JSON leaves the matching field of the destination
/// untouched rather than resetting it -- the same contract RenderSettings::load()
/// documents. Simulated here by decoding a document with only one key present.
TEST(RenderSettings, from_json_leaves_absent_fields_untouched) {
    nlohmann::json doc = nlohmann::json::object();
    doc["sun_intensity"] = 9.5f;

    RenderSettings s = distinctive_settings();
    const float untouched_azimuth = s.sun_azimuth_deg;

    stratum::from_json(doc, s);

    CHECK_EQ(s.sun_intensity, 9.5f);
    CHECK_EQ(s.sun_azimuth_deg, untouched_azimuth);
}

/// A field present with the wrong JSON type is skipped like an absent one,
/// rather than throwing or coercing.
TEST(RenderSettings, from_json_ignores_wrong_typed_fields) {
    nlohmann::json doc = nlohmann::json::object();
    doc["fog_mode"] = "exponential";   // should be a number
    doc["shadows_enabled"] = 1;        // should be a bool

    RenderSettings s = distinctive_settings();
    const int untouched_fog_mode = s.fog_mode;
    const bool untouched_shadows_enabled = s.shadows_enabled;

    stratum::from_json(doc, s);

    CHECK_EQ(s.fog_mode, untouched_fog_mode);
    CHECK_EQ(s.shadows_enabled, untouched_shadows_enabled);
}

// ============================================================================
// save() / load()
// ============================================================================

/// The file round trip: save a distinctive set of values, load them into a
/// fresh, default-constructed struct, and compare.
TEST(RenderSettings, save_then_load_round_trips_through_a_file) {
    const std::filesystem::path path = scratch("render_settings_round_trip.json");
    std::filesystem::remove(path);

    const RenderSettings original = distinctive_settings();
    CHECK_TRUE(original.save(path));
    CHECK_TRUE(std::filesystem::exists(path));

    RenderSettings loaded;
    CHECK_TRUE(loaded.load(path));

    check_equal(original, loaded);
    CHECK_FALSE(loaded.dirty);
}

/// save() creates parent directories that do not exist yet, the same contract
/// MaterialLibrary::save_to_file() documents and tests.
TEST(RenderSettings, save_creates_missing_parent_directories) {
    const std::filesystem::path dir =
        std::filesystem::path{STRATUM_TEST_TMP_DIR} / "render_settings_nested" / "deeper";
    const std::filesystem::path path = dir / "render_settings.json";
    std::error_code ec;
    std::filesystem::remove_all(std::filesystem::path{STRATUM_TEST_TMP_DIR} / "render_settings_nested", ec);

    CHECK_FALSE(std::filesystem::exists(dir));
    CHECK_TRUE(RenderSettings{}.save(path));
    CHECK_TRUE(std::filesystem::exists(path));

    std::filesystem::remove_all(std::filesystem::path{STRATUM_TEST_TMP_DIR} / "render_settings_nested", ec);
}

/// A path that does not exist is a normal, expected first run: load() reports
/// failure and leaves the struct exactly as it was.
TEST(RenderSettings, load_of_a_missing_file_leaves_settings_unchanged) {
    RenderSettings s = distinctive_settings();
    const RenderSettings before = s;

    CHECK_FALSE(s.load(scratch("render_settings_does_not_exist.json")));
    check_equal(before, s);
}

/// A file that exists but is not valid JSON must not throw and must not touch
/// the struct -- a corrupted preferences file is not a crash.
TEST(RenderSettings, load_of_malformed_json_leaves_settings_unchanged) {
    const std::filesystem::path path = scratch("render_settings_malformed.json");
    {
        FILE* f = std::fopen(path.string().c_str(), "wb");
        CHECK_TRUE(f != nullptr);
        if (f) {
            std::fputs("{ not valid json", f);
            std::fclose(f);
        }
    }

    RenderSettings s = distinctive_settings();
    const RenderSettings before = s;

    CHECK_FALSE(s.load(path));
    check_equal(before, s);
}

/// A well-formed JSON value that is not an object (e.g. a bare array) is
/// rejected the same way, rather than from_json() silently reading nothing.
TEST(RenderSettings, load_of_a_non_object_json_document_is_rejected) {
    const std::filesystem::path path = scratch("render_settings_not_an_object.json");
    {
        FILE* f = std::fopen(path.string().c_str(), "wb");
        CHECK_TRUE(f != nullptr);
        if (f) {
            std::fputs("[1, 2, 3]", f);
            std::fclose(f);
        }
    }

    RenderSettings s = distinctive_settings();
    const RenderSettings before = s;

    CHECK_FALSE(s.load(path));
    check_equal(before, s);
}

/// An empty path is the documented "no preference directory" case (SDL_GetPrefPath
/// failed): both calls must fail cleanly rather than touch the filesystem's
/// working directory or crash on an empty std::ofstream/ifstream path.
TEST(RenderSettings, empty_path_fails_both_load_and_save) {
    RenderSettings s;
    CHECK_FALSE(s.load(std::filesystem::path{}));
    CHECK_FALSE(s.save(std::filesystem::path{}));
}

// ============================================================================
// push_to() and the exposure seed
// ============================================================================
//
// A default-constructed GPURenderer never calls init(), so m_device stays
// null: set_shadow_config()'s reallocation branch is guarded on m_shadow_pipeline
// (also null), and every other setter push_to() calls just writes plain
// m_scene_uniforms fields. That makes the whole exposure regression below
// reproducible with no SDL_GPUDevice, window or Vulkan driver.

/// GPURenderer::mark_scene_lighting_initialized() seeds camera_position.w
/// (exposure) with its argument only on the FIRST call. A later call -- what
/// happens every time push_to() runs, i.e. every render-settings edit -- must
/// leave whatever set_exposure() last wrote alone. Before the fix, the second
/// call unconditionally rewrote camera_position.w to its default-argument
/// value of 1.0f, so changing Exposure and then dragging any other slider
/// snapped Exposure back to 1.0.
TEST(RenderSettings, mark_scene_lighting_initialized_seeds_exposure_once_only) {
    GPURenderer renderer;

    // First call: the Editor::set_renderer() startup seed. No exposure has
    // been set yet, so this establishes the default.
    renderer.mark_scene_lighting_initialized();
    CHECK_EQ(renderer.get_exposure(), 1.0f);

    // The user drags the Exposure slider.
    renderer.set_exposure(2.0f);
    CHECK_EQ(renderer.get_exposure(), 2.0f);

    // A second call -- what push_to() makes on every other slider edit --
    // must not reset exposure back to the default.
    renderer.mark_scene_lighting_initialized();
    CHECK_EQ(renderer.get_exposure(), 2.0f);

    // Nor does a third call, or one with an explicit non-default argument:
    // once seeded, the argument is ignored entirely.
    renderer.mark_scene_lighting_initialized(4.5f);
    CHECK_EQ(renderer.get_exposure(), 2.0f);
}

/// The same regression, exercised through push_to() itself rather than
/// calling mark_scene_lighting_initialized() directly -- the exact repro from
/// the bug report: set Exposure, then push unrelated render settings (a Sun
/// Azimuth drag) and confirm Exposure is kept.
TEST(RenderSettings, push_to_does_not_reset_exposure_on_a_later_call) {
    GPURenderer renderer;
    RenderSettings settings;

    // Editor::set_renderer()'s startup push, before the first render pass.
    settings.push_to(renderer);
    CHECK_EQ(renderer.get_exposure(), 1.0f);

    // User sets Exposure to 2.0 via the panel's own slider (push_to() has no
    // exposure field of its own -- see render_settings.hpp's file comment).
    renderer.set_exposure(2.0f);

    // User drags Sun Azimuth; the panel calls push_to() again.
    settings.sun_azimuth_deg = 123.0f;
    settings.push_to(renderer);

    CHECK_EQ(renderer.get_exposure(), 2.0f);
}
