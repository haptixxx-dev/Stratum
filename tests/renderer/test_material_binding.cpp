/**
 * @file test_material_binding.cpp
 * @brief The three renderer decisions that are a crash or a wrong sign rather than a wrong pixel
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Everything asserted here is a pure function of a few scalars, deliberately
 * lifted out of GPURenderer's draw path so it can be tested without a device, a
 * window or a render pass. Each one guards a defect whose symptom is invisible in
 * review:
 *
 * 1. **MaterialBindMode.** The PBR fragment shader is created with
 *    `num_samplers = kMaterialSamplerCount` and
 *    `num_uniform_buffers = kPbrFragmentUniformBufferCount`. SDL takes those counts
 *    at their word, so every draw through the PBR pipeline must leave all three
 *    sampler slots written. bind_material() used to `return` before
 *    SDL_BindGPUFragmentSamplers whenever materials were disabled or no library
 *    was installed -- both reachable from the shipped UI -- and the next draw hit
 *    SDL_GPU_CheckGraphicsBindings()'s `SDL_assert_release(!"Missing fragment
 *    sampler binding!")`, which is live in Release because GPURenderer::init()
 *    passes debug_mode as a literal true. The process aborted on the first mesh of
 *    the first frame after the toggle.
 *
 * 2. **decal_depth_bias().** The project is reverse-Z: the depth test is GREATER,
 *    so "towards the viewer" is a LARGER depth value and BOTH bias terms have to
 *    carry that sign. The constant was negated and the slope was left hardcoded
 *    negative, so the two opposed -- and Vulkan's `o = m * slope + r * constant`
 *    makes the slope term two orders of magnitude the larger at grazing angles.
 *    Markings were pushed away from the camera and occluded by their own
 *    carriageway, i.e. strictly worse than no bias at all.
 *
 * 3. **decal_bias_key().** MaterialDef::depth_bias is pipeline state, so honouring
 *    an edited value means one pipeline per distinct value. The key is what stops
 *    a slider drag from minting one per pixel of mouse travel.
 *
 * Run this suite with:
 * @code
 *     ./stratum_gpu_tests MaterialBinding
 * @endcode
 */

#include "framework.hpp"

#include "renderer/gpu_renderer.hpp"
#include "renderer/material_library.hpp"

#include <cstdint>
#include <limits>
#include <set>

namespace {

using stratum::GPURenderer;
using stratum::MaterialLibrary;
using stratum::ShaderMode;

using Mode = GPURenderer::MaterialBindMode;

/// bind_material()'s decision for a state, spelled out at each call site.
[[nodiscard]] Mode mode_for(ShaderMode shader, bool pbr_available, bool enabled,
                            bool has_library) {
    return GPURenderer::material_bind_mode(shader, pbr_available, enabled, has_library);
}

} // namespace

// ============================================================================
// MaterialBindMode
// ============================================================================

/**
 * The simple pipeline declares no material uniform block and no samplers, so
 * pushing to slot 1 or binding three samplers against it is a validation error
 * rather than a harmless extra. Skipping is the only correct answer there.
 */
TEST(MaterialBinding, simple_mode_binds_no_material_resources_at_all) {
    CHECK(mode_for(ShaderMode::Simple, true, true, true) == Mode::Skip);
    CHECK(mode_for(ShaderMode::Simple, true, false, false) == Mode::Skip);

    // Same answer when PBR is selected but its pipelines never came up: whatever
    // is bound in that case is a simple pipeline.
    CHECK(mode_for(ShaderMode::PBR, false, true, true) == Mode::Skip);
}

/**
 * @brief The regression: materials off must still bind SOMETHING
 *
 * This is the state the Materials panel's "Materials enabled" checkbox produces.
 * It does not change the shader mode, so the PBR pipeline -- three declared
 * fragment samplers, two declared fragment uniform buffers -- stays bound for
 * every draw of every following frame.
 */
TEST(MaterialBinding, materials_disabled_binds_the_neutral_set_rather_than_nothing) {
    CHECK(mode_for(ShaderMode::PBR, true, false, true) == Mode::Neutral);
}

/**
 * The other reachable route to the same state: Editor::init_materials() failed,
 * so no library was installed, and the user picked "PBR (Quality)" from the
 * Render Settings combo. GPURenderer::set_shader_mode() now refuses PBR while
 * there is no texture manager, which is what makes pbr_available false on that
 * path -- but if a library alone is missing, the neutral set still has to be bound.
 */
TEST(MaterialBinding, a_missing_library_binds_the_neutral_set) {
    CHECK(mode_for(ShaderMode::PBR, true, true, false) == Mode::Neutral);
    CHECK(mode_for(ShaderMode::PBR, true, false, false) == Mode::Neutral);
}

/// The ordinary case, for completeness: everything present and switched on.
TEST(MaterialBinding, a_live_library_resolves_the_key) {
    CHECK(mode_for(ShaderMode::PBR, true, true, true) == Mode::Full);
}

// ============================================================================
// Decal depth bias
// ============================================================================

/**
 * @brief Both bias terms must push the same way
 *
 * The failure this catches is not "the number is wrong", it is "the two numbers
 * disagree", which no amount of staring at either line reveals. Asserting the
 * relationship rather than the values also means a future change of magnitude
 * does not have to touch this test.
 */
TEST(MaterialBinding, decal_bias_terms_share_a_sign) {
    const auto bias = GPURenderer::decal_depth_bias(MaterialLibrary::kMarkingDepthBias);

    // Authored negative means "towards the camera"; in reverse-Z that is a LARGER
    // depth value, so the applied constant is positive.
    CHECK(MaterialLibrary::kMarkingDepthBias < 0.0f);
    CHECK(bias.constant > 0.0f);
    CHECK(bias.slope > 0.0f);
    CHECK(bias.constant * bias.slope > 0.0f);

    // A bias authored the other way round has to flip BOTH terms, not one.
    const auto away = GPURenderer::decal_depth_bias(2.0f);
    CHECK(away.constant < 0.0f);
    CHECK(away.slope < 0.0f);
    CHECK(away.constant * away.slope > 0.0f);
}

/// No bias authored means no bias applied, on either term.
TEST(MaterialBinding, a_zero_bias_applies_nothing) {
    const auto bias = GPURenderer::decal_depth_bias(0.0f);
    CHECK_EQ(bias.constant, 0.0f);
    CHECK_EQ(bias.slope, 0.0f);
}

/// The magnitude the material authored is the magnitude that reaches the
/// rasterizer, so the panel's slider and a set file's depth_bias field are not
/// stored, edited, serialised values that nothing reads.
TEST(MaterialBinding, the_constant_term_carries_the_authored_magnitude) {
    CHECK_EQ(GPURenderer::decal_depth_bias(-2.0f).constant, 2.0f);
    CHECK_EQ(GPURenderer::decal_depth_bias(-16.0f).constant, 16.0f);
    CHECK_EQ(GPURenderer::decal_depth_bias(-0.5f).constant, 0.5f);
}

// ============================================================================
// Decal pipeline cache key
// ============================================================================

/// Different authored biases must not collapse onto one pipeline, or the edit
/// silently does nothing again -- which is the whole defect the cache fixes.
TEST(MaterialBinding, distinct_biases_get_distinct_pipeline_keys) {
    std::set<uint32_t> keys;
    for (const float bias : { -16.0f, -8.0f, -4.0f, -2.0f, -1.0f, -0.5f, 0.0f, 2.0f, 16.0f }) {
        keys.insert(GPURenderer::decal_bias_key(bias));
    }
    CHECK_EQ(keys.size(), size_t{9});
}

/// A drag that moves the value by less than the quantum reuses the pipeline it
/// already has, rather than building a second identical one per frame.
TEST(MaterialBinding, biases_inside_one_quantum_share_a_pipeline) {
    CHECK_EQ(GPURenderer::decal_bias_key(-2.0f), GPURenderer::decal_bias_key(-2.001f));
    CHECK_EQ(GPURenderer::decal_bias_key(-2.0f), GPURenderer::decal_bias_key(-1.999f));
    CHECK(GPURenderer::decal_bias_key(-2.0f) != GPURenderer::decal_bias_key(-2.1f));
}

/// The key names the bias actually baked into the pipeline, so what the cache
/// hands back is what the material asked for to within the quantum.
TEST(MaterialBinding, a_key_round_trips_to_the_bias_it_bakes) {
    for (const float bias : { -16.0f, -2.0f, -0.5f, 0.0f, 3.25f, 16.0f }) {
        const float baked = GPURenderer::decal_bias_from_key(GPURenderer::decal_bias_key(bias));
        CHECK_NEAR(baked, bias, 1.0f / 32.0f);
    }
}

/// Out-of-range and non-finite values must land on a legal key rather than index
/// out of the table. A NaN compares false against both bounds and is pinned low.
TEST(MaterialBinding, extreme_biases_stay_inside_the_table) {
    const uint32_t lo = GPURenderer::decal_bias_key(-GPURenderer::kMaxDecalDepthBias);
    const uint32_t hi = GPURenderer::decal_bias_key(GPURenderer::kMaxDecalDepthBias);

    CHECK_EQ(GPURenderer::decal_bias_key(-1000.0f), lo);
    CHECK_EQ(GPURenderer::decal_bias_key(1000.0f), hi);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const uint32_t nan_key = GPURenderer::decal_bias_key(nan);
    CHECK(nan_key >= lo);
    CHECK(nan_key <= hi);
}
