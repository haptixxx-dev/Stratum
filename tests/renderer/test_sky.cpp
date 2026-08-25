/**
 * @file test_sky.cpp
 * @brief The analytic sky, rendered through the checked-in sky .spv
 *
 * The sky is not decoration here. assets/shaders/sky_common.glsl is included by
 * BOTH sky.frag, which paints the background, and mesh_pbr.frag, which lights
 * every surface, so the dome asserted below IS the ambient light in the scene.
 * A regression in it is a regression in the lighting.
 *
 * Two things about the pipeline are worth stating because they look like
 * mistakes:
 *
 *  - NO VERTEX BUFFER IS EVER BOUND, and the pipeline declares no vertex
 *    attributes. sky.vert builds its covering triangle from gl_VertexIndex. Any
 *    draw below would fail outright if that stopped being true, which is the
 *    cheapest possible test of it.
 *  - The target has no depth attachment. The real pass has one, and the real
 *    pipeline declares it, but the sky neither tests nor writes depth, so the
 *    only thing the attachment changes is a format field.
 */

#include "framework.hpp"

#include "renderer/gpu_renderer.hpp"

#include <SDL3/SDL.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

using stratum::SceneUniforms;
using stratum::SkyUniforms;

namespace {

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

struct DeviceHolder {
    SDL_GPUDevice* device = nullptr;
    bool attempted = false;
    bool video_ready = false;

    ~DeviceHolder() {
        if (device != nullptr) {
            SDL_WaitForGPUIdle(device);
            SDL_DestroyGPUDevice(device);
        }
        if (video_ready) SDL_QuitSubSystem(SDL_INIT_VIDEO);
    }
};

DeviceHolder& holder() {
    static DeviceHolder h;
    return h;
}

SDL_GPUDevice* device() {
    DeviceHolder& h = holder();
    if (h.attempted) return h.device;
    h.attempted = true;

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "[Sky] SKIPPED: SDL video init failed: %s\n", SDL_GetError());
        return nullptr;
    }
    h.video_ready = true;

    h.device = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV, false, nullptr);
    if (h.device == nullptr) {
        std::fprintf(stderr, "[Sky] SKIPPED: no SDL_GPU device: %s\n", SDL_GetError());
    }
    return h.device;
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// Plain UNORM: sky.frag does its own linear_to_srgb(), so an sRGB view would
/// encode the same image twice.
constexpr SDL_GPUTextureFormat kTargetFormat = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;

struct Texel {
    uint8_t r = 0, g = 0, b = 0, a = 0;
    [[nodiscard]] int luma() const { return (r * 299 + g * 587 + b * 114) / 1000; }
};

[[nodiscard]] std::vector<uint8_t> read_file(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) return {};
    const auto size = static_cast<size_t>(file.tellg());
    file.seekg(0);
    std::vector<uint8_t> bytes(size);
    if (!file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(size))) {
        return {};
    }
    return bytes;
}

[[nodiscard]] SDL_GPUShader* load_shader(SDL_GPUDevice* dev, const char* name,
                                         SDL_GPUShaderStage stage, uint32_t uniform_buffers) {
    const std::vector<uint8_t> code =
        read_file(std::filesystem::path{STRATUM_TEST_SHADER_DIR} / name);
    if (code.empty()) return nullptr;

    SDL_GPUShaderCreateInfo info{};
    info.code = code.data();
    info.code_size = code.size();
    info.entrypoint = "main";
    info.format = SDL_GPU_SHADERFORMAT_SPIRV;
    info.stage = stage;
    info.num_uniform_buffers = uniform_buffers;
    info.num_samplers = 0;
    return SDL_CreateGPUShader(dev, &info);
}

struct Harness {
    SDL_GPUDevice* dev = nullptr;
    SDL_GPUShader* vert = nullptr;
    SDL_GPUShader* frag = nullptr;
    SDL_GPUGraphicsPipeline* pipeline = nullptr;
    SDL_GPUTexture* target = nullptr;
    bool ok = false;

    Harness() {
        dev = device();
        if (dev == nullptr) return;

        // The counts GPURenderer::load_sky_shaders() passes: one uniform buffer
        // per stage, no samplers on either.
        vert = load_shader(dev, "sky.vert.spv", SDL_GPU_SHADERSTAGE_VERTEX, 1u);
        frag = load_shader(dev, "sky.frag.spv", SDL_GPU_SHADERSTAGE_FRAGMENT, 1u);
        if (vert == nullptr || frag == nullptr) return;

        SDL_GPUTextureCreateInfo target_info{};
        target_info.type = SDL_GPU_TEXTURETYPE_2D;
        target_info.format = kTargetFormat;
        target_info.width = 1;
        target_info.height = 1;
        target_info.layer_count_or_depth = 1;
        target_info.num_levels = 1;
        target_info.sample_count = SDL_GPU_SAMPLECOUNT_1;
        target_info.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;
        target = SDL_CreateGPUTexture(dev, &target_info);
        if (target == nullptr) return;

        SDL_GPUVertexInputState vertex_input{};   // deliberately empty
        vertex_input.num_vertex_buffers = 0;
        vertex_input.num_vertex_attributes = 0;

        SDL_GPURasterizerState rasterizer{};
        rasterizer.fill_mode = SDL_GPU_FILLMODE_FILL;
        rasterizer.cull_mode = SDL_GPU_CULLMODE_NONE;
        rasterizer.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;

        SDL_GPUColorTargetDescription color_target{};
        color_target.format = kTargetFormat;

        SDL_GPUGraphicsPipelineTargetInfo pipeline_target{};
        pipeline_target.color_target_descriptions = &color_target;
        pipeline_target.num_color_targets = 1;
        pipeline_target.has_depth_stencil_target = false;

        SDL_GPUGraphicsPipelineCreateInfo info{};
        info.vertex_shader = vert;
        info.fragment_shader = frag;
        info.vertex_input_state = vertex_input;
        info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
        info.rasterizer_state = rasterizer;
        info.target_info = pipeline_target;

        pipeline = SDL_CreateGPUGraphicsPipeline(dev, &info);
        if (pipeline == nullptr) return;

        ok = true;
    }

    ~Harness() {
        if (dev != nullptr) SDL_WaitForGPUIdle(dev);
        if (pipeline != nullptr) SDL_ReleaseGPUGraphicsPipeline(dev, pipeline);
        if (target != nullptr) SDL_ReleaseGPUTexture(dev, target);
        if (frag != nullptr) SDL_ReleaseGPUShader(dev, frag);
        if (vert != nullptr) SDL_ReleaseGPUShader(dev, vert);
    }

    /// Render the one pixel looking along @p sky.inv_view_projection's centre ray.
    Texel draw(const SceneUniforms& scene, const SkyUniforms& sky) {
        Texel out{};
        if (!ok) return out;

        SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(dev);
        if (cmd == nullptr) return out;

        SDL_GPUColorTargetInfo color_info{};
        color_info.texture = target;
        color_info.clear_color = SDL_FColor{ 0.0f, 0.0f, 0.0f, 1.0f };
        color_info.load_op = SDL_GPU_LOADOP_CLEAR;
        color_info.store_op = SDL_GPU_STOREOP_STORE;

        SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, &color_info, 1, nullptr);
        SDL_BindGPUGraphicsPipeline(pass, pipeline);

        SDL_PushGPUVertexUniformData(cmd, 0, &sky, sizeof(sky));
        SDL_PushGPUFragmentUniformData(cmd, stratum::kSceneUniformSlot, &scene, sizeof(scene));

        // Three vertices, nothing bound.
        SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
        SDL_EndGPURenderPass(pass);

        // Download the single texel.
        SDL_GPUTransferBufferCreateInfo transfer_info{};
        transfer_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_DOWNLOAD;
        transfer_info.size = 4;
        SDL_GPUTransferBuffer* transfer = SDL_CreateGPUTransferBuffer(dev, &transfer_info);
        if (transfer == nullptr) {
            SDL_SubmitGPUCommandBuffer(cmd);
            return out;
        }

        SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
        SDL_GPUTextureRegion region{};
        region.texture = target;
        region.w = 1;
        region.h = 1;
        region.d = 1;
        SDL_GPUTextureTransferInfo destination{};
        destination.transfer_buffer = transfer;
        SDL_DownloadFromGPUTexture(copy, &region, &destination);
        SDL_EndGPUCopyPass(copy);

        SDL_GPUFence* fence = SDL_SubmitGPUCommandBufferAndAcquireFence(cmd);
        if (fence != nullptr) {
            SDL_WaitForGPUFences(dev, true, &fence, 1);
            SDL_ReleaseGPUFence(dev, fence);
        }

        if (void* mapped = SDL_MapGPUTransferBuffer(dev, transfer, false)) {
            std::memcpy(&out, mapped, 4);
            SDL_UnmapGPUTransferBuffer(dev, transfer);
        }
        SDL_ReleaseGPUTransferBuffer(dev, transfer);
        return out;
    }
};

// ---------------------------------------------------------------------------
// Scene setup
// ---------------------------------------------------------------------------

/**
 * @brief Uniforms that aim the centre pixel along @p dir from the world origin
 *
 * The depth convention does not matter here and is deliberately not reproduced:
 * for the CENTRE pixel the near-plane point and the far-plane point lie on the
 * same ray, so the direction sky.vert derives is the same either way.
 */
SkyUniforms looking_at(glm::vec3 dir) {
    dir = glm::normalize(dir);
    // lookAt is degenerate when the view direction is parallel to `up`.
    const glm::vec3 up = (glm::abs(dir.y) > 0.999f) ? glm::vec3(0.0f, 0.0f, 1.0f)
                                                    : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::mat4 view = glm::lookAt(glm::vec3(0.0f), dir, up);
    const glm::mat4 proj = glm::perspective(glm::radians(60.0f), 1.0f, 0.1f, 1000.0f);

    SkyUniforms sky{};
    sky.inv_view_projection = glm::inverse(proj * view);
    sky.camera_position = glm::vec4(0.0f);
    return sky;
}

/// The renderer's own seeded sky, with the sun where the caller wants it.
SceneUniforms clear_day(glm::vec3 sun_dir) {
    SceneUniforms scene{};
    scene.camera_position = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);   // w = exposure
    scene.sun_direction = glm::vec4(glm::normalize(sun_dir), 3.14159265f);
    scene.sun_color = glm::vec4(1.0f, 0.98f, 0.95f, 1.0f);
    scene.fog_params = glm::vec4(0.0f);
    scene.fog_color = glm::vec4(0.0f);
    scene.pbr_params = glm::vec4(0.0f);
    scene.sky_zenith = glm::vec4(0.16f, 0.30f, 0.62f, 1.0f);
    scene.sky_horizon = glm::vec4(0.56f, 0.68f, 0.86f, 0.45f);
    scene.ground_color = glm::vec4(0.14f, 0.14f, 0.12f, 0.7f);
    // x = specular scale, y = cos(0.53 deg / 2), z = aerial, w = glow exponent
    scene.ibl_params = glm::vec4(1.0f, 0.9999894f, 1.0f, 64.0f);
    return scene;
}

/// A sun far from the sampled direction, so the Mie glow contributes nothing and
/// the assertion is about the dome alone.
const glm::vec3 kSunBehind{ 0.0f, 0.35f, -0.94f };

} // namespace

// ============================================================================
// The dome
// ============================================================================

/**
 * @brief The zenith is deeper and bluer than the horizon
 *
 * The gradient is the whole reason the sky replaced a constant clear colour. A
 * flat dome would light every upward-facing surface in the map with one colour,
 * which is what the old ambient constant did.
 */
TEST(Sky, the_zenith_is_deeper_and_bluer_than_the_horizon) {
    if (device() == nullptr) return;
    Harness h;
    CHECK_TRUE(h.ok);
    if (!h.ok) return;

    const SceneUniforms scene = clear_day(kSunBehind);
    const Texel zenith = h.draw(scene, looking_at({ 0.0f, 1.0f, 0.0f }));
    const Texel horizon = h.draw(scene, looking_at({ 1.0f, 0.02f, 0.0f }));

    // Both are sky, so both are lit.
    CHECK(zenith.luma() > 20);
    CHECK(horizon.luma() > 20);

    // The horizon band is the brighter one: more air, more scattering.
    CHECK(horizon.luma() > zenith.luma());

    // And the zenith is the bluer one. Comparing the blue-minus-red spread rather
    // than blue alone, because the horizon is brighter in every channel and a
    // bare blue comparison would measure that instead of the hue.
    const int zenith_blueness = int{zenith.b} - int{zenith.r};
    const int horizon_blueness = int{horizon.b} - int{horizon.r};
    CHECK(zenith_blueness > horizon_blueness);
}

/**
 * @brief Below the horizon is ground bounce, not sky
 *
 * The lower hemisphere is the ambient lobe that lights the underside of a bridge
 * deck. If it matched the upper one, a soffit would come out as bright as the
 * road surface on top of it -- exactly the artefact the constant ambient had.
 */
TEST(Sky, below_the_horizon_is_darker_than_above_it) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const SceneUniforms scene = clear_day(kSunBehind);
    const Texel above = h.draw(scene, looking_at({ 1.0f, 0.25f, 0.0f }));
    const Texel below = h.draw(scene, looking_at({ 1.0f, -0.25f, 0.0f }));

    CHECK(below.luma() < above.luma());
}

/**
 * @brief The sun's disk is the brightest thing in the sky, and only in the sky
 *
 * sky_with_sun() adds the disk; sky_radiance(), which every shading path uses,
 * does not. That split is what stops the sun being integrated twice -- once as
 * the analytic directional light in mesh_pbr.frag and again as a bright spot in
 * the environment it reflects.
 */
TEST(Sky, the_sun_disk_is_the_brightest_thing_in_the_sky) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const glm::vec3 sun = glm::normalize(glm::vec3(0.0f, 0.5f, 1.0f));
    const SceneUniforms scene = clear_day(sun);

    // A real 0.53-degree disk is far smaller than a 60-degree pixel, so the disk
    // is widened for this test rather than aimed at. The alternative is a much
    // higher resolution target and a search for the right pixel, which would
    // measure the harness rather than the shader.
    SceneUniforms wide = scene;
    wide.ibl_params.y = std::cos(glm::radians(10.0f) * 0.5f);

    const Texel at_sun = h.draw(wide, looking_at(sun));
    const Texel away = h.draw(wide, looking_at({ -sun.x, sun.y, -sun.z }));

    CHECK(at_sun.luma() > away.luma());
    // Saturated white, not merely brighter: the disk's radiance is well past what
    // the tone curve maps to 1.0.
    CHECK(at_sun.luma() > 240);

    // The same direction with the disk shrunk back to the real sun misses it
    // entirely, which is the check that the smoothstep window is a disk and not a
    // gradient covering half the sky.
    const Texel pinpoint = h.draw(scene, looking_at(glm::normalize(sun + glm::vec3(0.2f, 0.0f, 0.0f))));
    CHECK(pinpoint.luma() < at_sun.luma());
}
