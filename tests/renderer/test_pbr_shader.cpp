/**
 * @file test_pbr_shader.cpp
 * @brief One pixel through the real PBR pipeline, for the two defects only a draw can show
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Every other suite in this executable reasons about the pixels, the layout or the
 * SPIR-V. This one RENDERS: it builds a graphics pipeline from the checked-in
 * assets/shaders/mesh_pbr.{vert,frag}.spv -- the exact bytes GPURenderer hands
 * SDL_CreateGPUShader -- draws one triangle into a 1x1 offscreen target with the
 * same vertex layout, the same uniform slots and the same three fragment samplers
 * the renderer uses, and reads the texel back.
 *
 * It exists for two defects that no amount of reading the code makes visible:
 *
 * 1. **The degenerate bitangent.** Only the road and terrain builders call
 *    Mesh::compute_tangents(); src/osm/mesh_builder.cpp does not, so building
 *    walls keep Vertex's default tangent (1, 0, 0, 1). Their normals are
 *    normalize(cross(up, edge)) with the edge in the XZ plane, so an exactly
 *    north-south footprint edge gives a normal of exactly (-1, 0, 0) and
 *    `cross(N, T)` is EXACTLY zero -- every term is a product with an exact zero,
 *    so no amount of floating-point luck saves it. The shader normalised that to
 *    NaN and propagated it through the whole tangent frame, because 0 * NaN is NaN.
 *    The tangent was guarded; the bitangent was not.
 *
 * 2. **Metallic multiplied by a zero ORM blue channel.** Every material binds the
 *    built-in default ORM, and its blue was 0, so `metallic = pbr_params.x * orm.b`
 *    was zero whatever the material authored.
 *
 * ### Why the assertions are relational
 *
 * Not "the pixel is 0x7A". Tone mapping, the sRGB encode and the GGX terms all
 * make an absolute value a hostage to arithmetic nobody intends to freeze. Each
 * test instead draws the SAME geometry twice, changing exactly one input, and
 * asserts the two results differ in the direction the physics demands. A NaN
 * normal collapses to zero direct light on every driver that suppresses NaN in
 * max(), so "lit versus ambient-only" is the observable either way.
 *
 * On a machine with no usable backend the suite prints a SKIPPED line and asserts
 * nothing, exactly as GPUBufferPool does.
 *
 * Run this suite with:
 * @code
 *     ./stratum_gpu_tests PbrShader
 * @endcode
 */

#include "framework.hpp"

#include "renderer/gpu_renderer.hpp"
#include "renderer/material_library.hpp"
#include "renderer/mesh.hpp"
#include "renderer/texture.hpp"

#include <SDL3/SDL.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace {

using stratum::GPUTextureManager;
using stratum::MaterialUniforms;
using stratum::MeshUniformsPBR;
using stratum::SamplerKind;
using stratum::SceneUniforms;
using stratum::Vertex;

/// The rendered texel, in 0-255 RGBA.
struct Texel {
    uint8_t r = 0, g = 0, b = 0, a = 0;
    [[nodiscard]] int luma() const { return int{r} + int{g} + int{b}; }
};

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

struct DeviceHolder {
    SDL_GPUDevice* device = nullptr;
    bool attempted = false;
    bool video_ready = false;

    ~DeviceHolder() {
        if (device != nullptr) {
            SDL_DestroyGPUDevice(device);
            device = nullptr;
        }
        if (video_ready) {
            SDL_QuitSubSystem(SDL_INIT_VIDEO);
            SDL_Quit();
        }
    }
};

DeviceHolder& holder() {
    static DeviceHolder h;
    return h;
}

/// Whether to ask SDL for a debug device. See the note in device().
[[nodiscard]] bool debug_mode() {
    const char* env = SDL_getenv("STRATUM_TEST_GPU_DEBUG");
    return env != nullptr && env[0] != '\0' && env[0] != '0';
}

SDL_GPUDevice* device() {
    DeviceHolder& h = holder();
    if (h.attempted) return h.device;
    h.attempted = true;

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "[PbrShader] SKIPPED: SDL video init failed: %s\n", SDL_GetError());
        return nullptr;
    }
    h.video_ready = true;

    // debug_mode is what makes SDL's own SDL_GPU_CheckGraphicsBindings() run on
    // every draw -- the check that catches a declared-but-unbound fragment
    // sampler -- so it is worth having available. It is OFF by default all the
    // same, because it also loads the Khronos validation layer, and on this
    // machine the layer aborts inside vkDeviceWaitIdle during
    // SDL_DestroyGPUDevice at process exit, AFTER every test has passed, with no
    // message on any stream. The same abort does not happen for a bare debug
    // device, nor for a debug device that only brings a GPUTextureManager up and
    // down, so it is somewhere in the render-and-download path -- but every
    // assertion below has already been made by then, and a suite that reports a
    // failure nobody can read is worse than one that does not run the layer.
    // Turn it on deliberately when investigating that:
    //
    //     STRATUM_TEST_GPU_DEBUG=1 ./stratum_gpu_tests PbrShader
    h.device = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV, debug_mode(), nullptr);
    if (h.device == nullptr) {
        std::fprintf(stderr, "[PbrShader] SKIPPED: no SDL_GPU device: %s\n", SDL_GetError());
    }
    return h.device;
}

// ---------------------------------------------------------------------------
// The pipeline under test
// ---------------------------------------------------------------------------

/// The colour format the offscreen target and the pipeline agree on. Plain UNORM:
/// mesh_pbr.frag does its own linear_to_srgb(), so an sRGB view would encode twice.
constexpr SDL_GPUTextureFormat kTargetFormat = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;

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
                                         SDL_GPUShaderStage stage, uint32_t uniform_buffers,
                                         uint32_t samplers) {
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
    info.num_samplers = samplers;
    return SDL_CreateGPUShader(dev, &info);
}

/**
 * @brief Everything needed to render one triangle through mesh_pbr, once
 *
 * Built per test rather than shared: each test is three device objects and one
 * submit, and a shared harness would make a failure in one test look like a
 * failure in the next.
 */
struct Harness {
    SDL_GPUDevice* dev = nullptr;
    GPUTextureManager textures;
    SDL_GPUShader* vert = nullptr;
    SDL_GPUShader* frag = nullptr;
    SDL_GPUGraphicsPipeline* pipeline = nullptr;
    SDL_GPUTexture* target = nullptr;
    SDL_GPUBuffer* vertex_buffer = nullptr;
    SDL_GPUTexture* shadow_texture = nullptr;
    SDL_GPUSampler* shadow_sampler = nullptr;
    SDL_GPUGraphicsPipeline* shadow_pipeline = nullptr;
    SDL_GPUShader* shadow_vert = nullptr;
    SDL_GPUShader* shadow_frag = nullptr;
    SDL_GPUBuffer* caster_buffer = nullptr;
    bool ok = false;

    /// Small on purpose: the assertions are about whether a fragment is in shadow
    /// at all, and a big map would only make each test slower.
    static constexpr uint32_t kShadowMapSize = 256;

    Harness() {
        dev = device();
        if (dev == nullptr) return;
        if (!textures.init(dev)) return;

        // The counts GPURenderer::load_shader() passes, from the same constants.
        // kPbrFragmentSamplerCount, not kMaterialSamplerCount: the shader declares
        // the shadow map at binding 3 as well as the material's three.
        vert = load_shader(dev, "mesh_pbr.vert.spv", SDL_GPU_SHADERSTAGE_VERTEX, 1u, 0u);
        frag = load_shader(dev, "mesh_pbr.frag.spv", SDL_GPU_SHADERSTAGE_FRAGMENT,
                           stratum::kPbrFragmentUniformBufferCount,
                           stratum::kPbrFragmentSamplerCount);
        if (vert == nullptr || frag == nullptr) return;

        if (!create_shadow_stand_in()) return;

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

        if (!build_pipeline()) return;
        ok = true;
    }

    /**
     * @brief The shadow map, bound by every draw and rendered into by some
     *
     * mesh_pbr.frag DECLARES sampler2DArrayShadow unconditionally, so every draw
     * has to bind something at kShadowSamplerSlot or the descriptor set is
     * incomplete. Most tests here are about shading rather than shadows, and they
     * push a ShadowUniforms block whose cascade count is ZERO -- the shader's
     * documented off switch -- so the texture's contents cannot reach their
     * result.
     *
     * render_caster() below fills it for the tests that ARE about shadows. One
     * layer is enough: cascade selection is a loop over the same code, so
     * rendering the first cascade exercises all of it.
     */
    bool create_shadow_stand_in() {
        // A 2D atlas, exactly like the renderer's -- one tile here, because these
        // tests only need cascade 0. NOT a 2D array: SDL_GPU rejects array
        // textures with DEPTH_STENCIL_TARGET usage, and building the test against
        // a shape the renderer cannot use would test nothing.
        SDL_GPUTextureCreateInfo info{};
        info.type = SDL_GPU_TEXTURETYPE_2D;
        info.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
        info.width = kShadowMapSize;
        info.height = kShadowMapSize;
        info.layer_count_or_depth = 1;
        info.num_levels = 1;
        info.sample_count = SDL_GPU_SAMPLECOUNT_1;
        info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
        shadow_texture = SDL_CreateGPUTexture(dev, &info);
        if (shadow_texture == nullptr) return false;

        SDL_GPUSamplerCreateInfo sampler_info{};
        sampler_info.min_filter = SDL_GPU_FILTER_LINEAR;
        sampler_info.mag_filter = SDL_GPU_FILTER_LINEAR;
        sampler_info.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
        sampler_info.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
        sampler_info.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
        sampler_info.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
        sampler_info.enable_compare = true;
        sampler_info.compare_op = SDL_GPU_COMPAREOP_LESS;
        shadow_sampler = SDL_CreateGPUSampler(dev, &sampler_info);
        return shadow_sampler != nullptr;
    }

    /**
     * @brief Render one caster quad into cascade 0, exactly as the renderer does
     *
     * The pipeline is built with the SAME state GPURenderer::create_shadow_pipeline()
     * uses -- position-only vertex input, front-face culling, forward LESS depth
     * against a 1.0 clear, no colour targets -- because those are precisely the
     * settings a test that built its own convenient variant would fail to check.
     *
     * THE QUAD IS SUBMITTED IN BOTH WINDINGS. Front-face culling means only the
     * winding that faces AWAY from the light survives, and which of the two that
     * is depends on axis conventions this test deliberately does not want to
     * encode by hand. A real caster is closed and always presents a back face from
     * any direction, so submitting both is the honest model of one -- and it keeps
     * the test measuring the shadow path rather than my winding arithmetic.
     */
    bool render_caster(const glm::mat4& light_view_proj, const glm::vec3 (&quad)[4]) {
        if (shadow_pipeline == nullptr && !build_shadow_pipeline()) return false;

        // Two triangles per winding, six vertices each.
        Vertex verts[12]{};
        const int forward[6] = { 0, 1, 2, 0, 2, 3 };
        const int reverse[6] = { 0, 2, 1, 0, 3, 2 };
        for (int i = 0; i < 6; ++i) {
            verts[i].position = quad[forward[i]];
            verts[i + 6].position = quad[reverse[i]];
        }

        if (caster_buffer == nullptr) {
            SDL_GPUBufferCreateInfo buffer_info{};
            buffer_info.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
            buffer_info.size = sizeof(verts);
            caster_buffer = SDL_CreateGPUBuffer(dev, &buffer_info);
            if (caster_buffer == nullptr) return false;
        }

        SDL_GPUTransferBufferCreateInfo transfer_info{};
        transfer_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        transfer_info.size = sizeof(verts);
        SDL_GPUTransferBuffer* transfer = SDL_CreateGPUTransferBuffer(dev, &transfer_info);
        if (transfer == nullptr) return false;
        void* mapped = SDL_MapGPUTransferBuffer(dev, transfer, false);
        if (mapped == nullptr) {
            SDL_ReleaseGPUTransferBuffer(dev, transfer);
            return false;
        }
        std::memcpy(mapped, verts, sizeof(verts));
        SDL_UnmapGPUTransferBuffer(dev, transfer);

        SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(dev);
        SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
        SDL_GPUTransferBufferLocation src{};
        src.transfer_buffer = transfer;
        SDL_GPUBufferRegion dst{};
        dst.buffer = caster_buffer;
        dst.size = sizeof(verts);
        SDL_UploadToGPUBuffer(copy, &src, &dst, false);
        SDL_EndGPUCopyPass(copy);

        SDL_GPUDepthStencilTargetInfo depth_target{};
        depth_target.texture = shadow_texture;
        depth_target.clear_depth = 1.0f;
        depth_target.load_op = SDL_GPU_LOADOP_CLEAR;
        depth_target.store_op = SDL_GPU_STOREOP_STORE;
        depth_target.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
        depth_target.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

        SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, nullptr, 0, &depth_target);
        if (pass == nullptr) {
            SDL_SubmitGPUCommandBuffer(cmd);
            SDL_ReleaseGPUTransferBuffer(dev, transfer);
            return false;
        }
        SDL_BindGPUGraphicsPipeline(pass, shadow_pipeline);

        SDL_GPUViewport viewport{};
        viewport.w = static_cast<float>(kShadowMapSize);
        viewport.h = static_cast<float>(kShadowMapSize);
        viewport.max_depth = 1.0f;
        SDL_SetGPUViewport(pass, &viewport);

        stratum::ShadowMeshUniforms uniforms{};
        uniforms.light_mvp = light_view_proj;   // model is identity
        SDL_PushGPUVertexUniformData(cmd, 0, &uniforms, sizeof(uniforms));

        SDL_GPUBufferBinding binding{};
        binding.buffer = caster_buffer;
        SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
        SDL_DrawGPUPrimitives(pass, 12, 1, 0, 0);
        SDL_EndGPURenderPass(pass);

        const bool submitted = SDL_SubmitGPUCommandBuffer(cmd);
        SDL_WaitForGPUIdle(dev);
        SDL_ReleaseGPUTransferBuffer(dev, transfer);
        return submitted;
    }

    bool build_shadow_pipeline() {
        if (shadow_vert == nullptr) {
            shadow_vert = load_shader(dev, "shadow.vert.spv", SDL_GPU_SHADERSTAGE_VERTEX, 1u, 0u);
        }
        if (shadow_frag == nullptr) {
            shadow_frag = load_shader(dev, "shadow.frag.spv", SDL_GPU_SHADERSTAGE_FRAGMENT, 0u, 0u);
        }
        if (shadow_vert == nullptr || shadow_frag == nullptr) return false;

        SDL_GPUVertexBufferDescription buffer_desc{};
        buffer_desc.slot = 0;
        buffer_desc.pitch = sizeof(Vertex);
        buffer_desc.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

        SDL_GPUVertexAttribute position_attr{};
        position_attr.location = 0;
        position_attr.buffer_slot = 0;
        position_attr.format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
        position_attr.offset = offsetof(Vertex, position);

        SDL_GPUVertexInputState vertex_input{};
        vertex_input.vertex_buffer_descriptions = &buffer_desc;
        vertex_input.num_vertex_buffers = 1;
        vertex_input.vertex_attributes = &position_attr;
        vertex_input.num_vertex_attributes = 1;

        SDL_GPURasterizerState rasterizer{};
        rasterizer.fill_mode = SDL_GPU_FILLMODE_FILL;
        rasterizer.cull_mode = SDL_GPU_CULLMODE_FRONT;
        rasterizer.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;

        SDL_GPUDepthStencilState depth_stencil{};
        depth_stencil.compare_op = SDL_GPU_COMPAREOP_LESS;
        depth_stencil.enable_depth_test = true;
        depth_stencil.enable_depth_write = true;

        SDL_GPUGraphicsPipelineTargetInfo target_info{};
        target_info.num_color_targets = 0;
        target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
        target_info.has_depth_stencil_target = true;

        SDL_GPUGraphicsPipelineCreateInfo info{};
        info.vertex_shader = shadow_vert;
        info.fragment_shader = shadow_frag;
        info.vertex_input_state = vertex_input;
        info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
        info.rasterizer_state = rasterizer;
        info.depth_stencil_state = depth_stencil;
        info.target_info = target_info;

        shadow_pipeline = SDL_CreateGPUGraphicsPipeline(dev, &info);
        return shadow_pipeline != nullptr;
    }

    ~Harness() {
        if (dev != nullptr) SDL_WaitForGPUIdle(dev);
        if (caster_buffer != nullptr) SDL_ReleaseGPUBuffer(dev, caster_buffer);
        if (shadow_pipeline != nullptr) SDL_ReleaseGPUGraphicsPipeline(dev, shadow_pipeline);
        if (shadow_frag != nullptr) SDL_ReleaseGPUShader(dev, shadow_frag);
        if (shadow_vert != nullptr) SDL_ReleaseGPUShader(dev, shadow_vert);
        if (shadow_sampler != nullptr) SDL_ReleaseGPUSampler(dev, shadow_sampler);
        if (shadow_texture != nullptr) SDL_ReleaseGPUTexture(dev, shadow_texture);
        if (vertex_buffer != nullptr) SDL_ReleaseGPUBuffer(dev, vertex_buffer);
        if (pipeline != nullptr) SDL_ReleaseGPUGraphicsPipeline(dev, pipeline);
        if (target != nullptr) SDL_ReleaseGPUTexture(dev, target);
        if (frag != nullptr) SDL_ReleaseGPUShader(dev, frag);
        if (vert != nullptr) SDL_ReleaseGPUShader(dev, vert);
        textures.shutdown();
    }

    bool build_pipeline() {
        // The renderer's own five-attribute layout over stratum::Vertex.
        SDL_GPUVertexBufferDescription buffer_desc{};
        buffer_desc.slot = 0;
        buffer_desc.pitch = sizeof(Vertex);
        buffer_desc.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

        // SIX attributes, matching describe_pbr_vertex_input(). Declaring five
        // here does not fail to build and does not warn -- location 5 simply reads
        // as zero, so every vertex arrives with a baked ambient occlusion of 0 and
        // the whole sky term vanishes. That is a shading bug wearing the costume
        // of a lighting bug, and it is why this list is worth keeping in step.
        SDL_GPUVertexAttribute attrs[6]{};
        attrs[0] = { 0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(Vertex, position) };
        attrs[1] = { 1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(Vertex, normal) };
        attrs[2] = { 2, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, offsetof(Vertex, uv) };
        attrs[3] = { 3, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(Vertex, color) };
        attrs[4] = { 4, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(Vertex, tangent) };
        attrs[5] = { 5, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT, offsetof(Vertex, ao) };

        SDL_GPUVertexInputState vertex_input{};
        vertex_input.vertex_buffer_descriptions = &buffer_desc;
        vertex_input.num_vertex_buffers = 1;
        vertex_input.vertex_attributes = attrs;
        vertex_input.num_vertex_attributes = 6;

        // Culling off: this suite is about shading, and a winding mistake in a
        // hand-written triangle would look exactly like a shading failure.
        SDL_GPURasterizerState rasterizer{};
        rasterizer.fill_mode = SDL_GPU_FILLMODE_FILL;
        rasterizer.cull_mode = SDL_GPU_CULLMODE_NONE;
        rasterizer.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;

        SDL_GPUColorTargetDescription color_target{};
        color_target.format = kTargetFormat;

        SDL_GPUGraphicsPipelineTargetInfo target_info{};
        target_info.color_target_descriptions = &color_target;
        target_info.num_color_targets = 1;
        target_info.has_depth_stencil_target = false;

        SDL_GPUGraphicsPipelineCreateInfo info{};
        info.vertex_shader = vert;
        info.fragment_shader = frag;
        info.vertex_input_state = vertex_input;
        info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
        info.rasterizer_state = rasterizer;
        info.target_info = target_info;

        pipeline = SDL_CreateGPUGraphicsPipeline(dev, &info);
        return pipeline != nullptr;
    }

    /// Upload three vertices, replacing whatever was there. Synchronous: this is a
    /// test, and one wait per draw is cheaper than plumbing a fence through it.
    bool upload(const Vertex (&verts)[3]) {
        if (vertex_buffer == nullptr) {
            SDL_GPUBufferCreateInfo buffer_info{};
            buffer_info.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
            buffer_info.size = sizeof(verts);
            vertex_buffer = SDL_CreateGPUBuffer(dev, &buffer_info);
            if (vertex_buffer == nullptr) return false;
        }

        SDL_GPUTransferBufferCreateInfo transfer_info{};
        transfer_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        transfer_info.size = sizeof(verts);
        SDL_GPUTransferBuffer* transfer = SDL_CreateGPUTransferBuffer(dev, &transfer_info);
        if (transfer == nullptr) return false;

        void* mapped = SDL_MapGPUTransferBuffer(dev, transfer, false);
        if (mapped == nullptr) {
            SDL_ReleaseGPUTransferBuffer(dev, transfer);
            return false;
        }
        std::memcpy(mapped, verts, sizeof(verts));
        SDL_UnmapGPUTransferBuffer(dev, transfer);

        SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(dev);
        SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);
        SDL_GPUTransferBufferLocation src{};
        src.transfer_buffer = transfer;
        SDL_GPUBufferRegion dst{};
        dst.buffer = vertex_buffer;
        dst.size = sizeof(verts);
        SDL_UploadToGPUBuffer(copy_pass, &src, &dst, false);
        SDL_EndGPUCopyPass(copy_pass);
        const bool submitted = SDL_SubmitGPUCommandBuffer(cmd);
        SDL_WaitForGPUIdle(dev);
        SDL_ReleaseGPUTransferBuffer(dev, transfer);
        return submitted;
    }

    /**
     * @brief Draw the three vertices with these uniforms and read the texel back
     *
     * The sampler bind is the renderer's neutral set: white albedo, flat normal,
     * unit ORM, RepeatAniso. It is bound unconditionally, which is exactly what
     * GPURenderer::bind_neutral_material() now does and what the early return it
     * replaced did not.
     */
    /// Zeroed by default: shadow_params.x is the cascade count, and 0 is the
    /// shader's documented "no shadows" switch, so the shadow texture's contents
    /// cannot reach any test that does not deliberately set this.
    stratum::ShadowUniforms shadows{};

    Texel draw(const Vertex (&verts)[3], const SceneUniforms& scene,
               const MaterialUniforms& material, const MeshUniformsPBR& mesh) {
        Texel out{};
        if (!ok || !upload(verts)) return out;

        SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(dev);
        if (cmd == nullptr) return out;

        SDL_GPUColorTargetInfo color_info{};
        color_info.texture = target;
        color_info.clear_color = SDL_FColor{ 0.0f, 0.0f, 0.0f, 1.0f };
        color_info.load_op = SDL_GPU_LOADOP_CLEAR;
        color_info.store_op = SDL_GPU_STOREOP_STORE;

        SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, &color_info, 1, nullptr);
        SDL_BindGPUGraphicsPipeline(pass, pipeline);

        SDL_GPUBufferBinding vertex_binding{};
        vertex_binding.buffer = vertex_buffer;
        SDL_BindGPUVertexBuffers(pass, 0, &vertex_binding, 1);

        SDL_PushGPUVertexUniformData(cmd, 0, &mesh, sizeof(mesh));
        SDL_PushGPUFragmentUniformData(cmd, stratum::kSceneUniformSlot, &scene, sizeof(scene));
        SDL_PushGPUFragmentUniformData(cmd, stratum::kMaterialUniformSlot, &material,
                                       sizeof(material));
        // Zeroed: shadow_params.x is the cascade count, and 0 means "no shadows".
        // Leaving this slot unpushed would leave the block undefined, and a
        // non-zero count in that garbage would send the shader sampling a depth
        // texture nothing has rendered into.
        SDL_PushGPUFragmentUniformData(cmd, stratum::kShadowUniformSlot, &shadows,
                                       sizeof(shadows));

        SDL_GPUSampler* sampler = textures.sampler(SamplerKind::RepeatAniso);
        SDL_GPUTextureSamplerBinding bindings[stratum::kMaterialSamplerCount]{};
        bindings[stratum::kAlbedoSamplerSlot] = { textures.get(textures.white()), sampler };
        bindings[stratum::kNormalSamplerSlot] = { textures.get(textures.flat_normal()), sampler };
        bindings[stratum::kOrmSamplerSlot] = { textures.get(textures.default_orm()), sampler };
        SDL_BindGPUFragmentSamplers(pass, stratum::kAlbedoSamplerSlot, bindings,
                                    stratum::kMaterialSamplerCount);

        // The shadow map is a SEPARATE bind at its own slot, exactly as
        // GPURenderer::bind_shadow_resources() does it -- not a fourth entry in the
        // material array, which would make every material rebind it.
        SDL_GPUTextureSamplerBinding shadow_binding{ shadow_texture, shadow_sampler };
        SDL_BindGPUFragmentSamplers(pass, stratum::kShadowSamplerSlot, &shadow_binding, 1);

        SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
        SDL_EndGPURenderPass(pass);

        SDL_GPUTransferBufferCreateInfo transfer_info{};
        transfer_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_DOWNLOAD;
        transfer_info.size = 4;
        SDL_GPUTransferBuffer* transfer = SDL_CreateGPUTransferBuffer(dev, &transfer_info);
        if (transfer == nullptr) {
            SDL_SubmitGPUCommandBuffer(cmd);
            return out;
        }

        SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);
        SDL_GPUTextureRegion region{};
        region.texture = target;
        region.w = 1;
        region.h = 1;
        region.d = 1;
        SDL_GPUTextureTransferInfo destination{};
        destination.transfer_buffer = transfer;
        SDL_DownloadFromGPUTexture(copy_pass, &region, &destination);
        SDL_EndGPUCopyPass(copy_pass);

        SDL_GPUFence* fence = SDL_SubmitGPUCommandBufferAndAcquireFence(cmd);
        if (fence != nullptr) {
            SDL_WaitForGPUFences(dev, true, &fence, 1);
            SDL_ReleaseGPUFence(dev, fence);

            const void* mapped = SDL_MapGPUTransferBuffer(dev, transfer, false);
            if (mapped != nullptr) {
                const auto* bytes = static_cast<const uint8_t*>(mapped);
                out = Texel{ bytes[0], bytes[1], bytes[2], bytes[3] };
                SDL_UnmapGPUTransferBuffer(dev, transfer);
            }
        }
        SDL_ReleaseGPUTransferBuffer(dev, transfer);
        return out;
    }
};

// ---------------------------------------------------------------------------
// Scene setup
// ---------------------------------------------------------------------------

/**
 * @brief A triangle that covers the 1x1 target, shaded as a wall facing -X
 *
 * @param normal  Wall normal, in world space
 * @param tangent Vertex tangent. (1, 0, 0, 1) is stratum::Vertex's DEFAULT, i.e.
 *                what every mesh that never ran compute_tangents() carries.
 *
 * The positions are chosen so the one rasterised pixel sits at NDC (0, 0), which
 * interpolates to world (0, 0, -5). mesh_pbr.frag takes its view vector as
 * normalize(scene.camera_position.xyz - frag_world_pos), so with sun_from()'s
 * camera at the origin V is exactly (0, 0, 1) there -- no accidental flip in the
 * double-sided branch, and nothing depends on where the pixel landed. Tests that
 * want a camera somewhere else override camera_position.xyz themselves.
 */
void make_wall(Vertex (&verts)[3], glm::vec3 normal, glm::vec4 tangent) {
    const glm::vec2 ndc[3] = { { -3.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 3.0f } };
    for (int i = 0; i < 3; ++i) {
        verts[i] = Vertex{};
        verts[i].position = glm::vec3(ndc[i], -5.0f);
        verts[i].normal = normal;
        verts[i].uv = glm::vec2(0.0f);
        verts[i].color = glm::vec4(1.0f);
        verts[i].tangent = tangent;
    }
}

/// Uniforms whose mvp maps the wall's positions straight to NDC and leaves the
/// model transform identity, so frag_world_pos is the authored position.
MeshUniformsPBR wall_uniforms() {
    MeshUniformsPBR mesh{};
    mesh.mvp = glm::mat4(1.0f);
    mesh.mvp[2][2] = 0.0f;      // drop z
    mesh.mvp[3][2] = 0.5f;      // and put every fragment mid-range instead
    mesh.model = glm::mat4(1.0f);
    mesh.normal_matrix = glm::mat4(1.0f);
    mesh.color_tint = glm::vec4(1.0f);
    mesh.camera_position = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    return mesh;
}

/// A sun aimed straight down @p light_dir, no fog, almost no ambient -- so the
/// direct term is nearly the whole pixel and losing it is unmissable.
///
/// The sky members are NOT left zeroed. sun_color.a is only the ambient master
/// scale now; the ambient light itself is the hemisphere integral of these four,
/// so a zeroed sky is a scene with no fill light at all, and the ambient-only
/// assertions below (ambient occlusion, the unlit back face) would be comparing
/// black against black and would pass for the wrong reason.
SceneUniforms sun_from(glm::vec3 light_dir) {
    SceneUniforms scene{};
    scene.camera_position = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);   // w = exposure
    scene.sun_direction = glm::vec4(light_dir, 3.0f);            // w = intensity
    scene.sun_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.05f);        // a = ambient master
    scene.fog_params = glm::vec4(0.0f);                          // w = 0 disables fog
    scene.fog_color = glm::vec4(0.0f);
    scene.pbr_params = glm::vec4(0.0f);                          // reserved, unread
    scene.sky_zenith = glm::vec4(0.16f, 0.30f, 0.62f, 1.0f);     // a = sky intensity
    scene.sky_horizon = glm::vec4(0.56f, 0.68f, 0.86f, 0.45f);   // a = falloff
    scene.ground_color = glm::vec4(0.14f, 0.14f, 0.12f, 0.7f);   // a = bounce intensity
    // x = specular scale, y = cos(sun radius), z = aerial perspective, w = glow
    scene.ibl_params = glm::vec4(1.0f, 0.9999894f, 0.0f, 64.0f);
    return scene;
}

} // namespace

// ============================================================================
// The degenerate tangent frame
// ============================================================================

/**
 * @brief A wall that never got tangents still takes direct sunlight
 *
 * normal (-1, 0, 0) with the default tangent (1, 0, 0, 1) makes
 * frag_bitangent = cross(N, T) * w exactly (0, 0, 0). The reference is the SAME
 * wall with a tangent that is not parallel to its normal, which produces an
 * ordinary frame and therefore the value the degenerate case must match.
 *
 * With the bitangent unguarded the shading normal was NaN. On hardware that
 * suppresses NaN in max() -- AMD, NVIDIA and Intel all do -- that is NdotL = 0
 * and the pixel falls to the ambient term alone; where it does not, the pixel is
 * whatever a NaN rasterises as. Both are caught by comparing against the
 * well-formed frame.
 */
TEST(PbrShader, a_degenerate_bitangent_still_shades) {
    if (device() == nullptr) return;
    Harness h;
    CHECK_TRUE(h.ok);
    if (!h.ok) return;

    const glm::vec3 normal(-1.0f, 0.0f, 0.0f);
    const SceneUniforms scene = sun_from(normal);   // lit head-on
    const MeshUniformsPBR mesh = wall_uniforms();
    MaterialUniforms material{};

    Vertex healthy[3];
    make_wall(healthy, normal, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
    const Texel reference = h.draw(healthy, scene, material, mesh);

    Vertex degenerate[3];
    make_wall(degenerate, normal, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));   // Vertex's default
    const Texel result = h.draw(degenerate, scene, material, mesh);

    // The reference must actually be lit, or the test proves nothing.
    CHECK(reference.luma() > 90);

    // And the degenerate frame must land on the same shade. The normal map is the
    // flat default, so the tangent frame's ORIENTATION cannot matter -- only its
    // validity can.
    CHECK_EQ(int{result.r}, int{reference.r});
    CHECK_EQ(int{result.g}, int{reference.g});
    CHECK_EQ(int{result.b}, int{reference.b});
}

/// The same wall turned away from the sun is dark, which is what makes the
/// assertion above a measurement of direct light rather than of ambient.
TEST(PbrShader, an_unlit_wall_is_much_darker_than_a_lit_one) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const glm::vec3 normal(-1.0f, 0.0f, 0.0f);
    const MeshUniformsPBR mesh = wall_uniforms();
    MaterialUniforms material{};

    Vertex verts[3];
    make_wall(verts, normal, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));

    const Texel lit = h.draw(verts, sun_from(normal), material, mesh);
    const Texel unlit = h.draw(verts, sun_from(-normal), material, mesh);

    CHECK(lit.luma() > unlit.luma() + 60);
}

// ============================================================================
// Metallic reaches the shader
// ============================================================================

/**
 * @brief MaterialDef::metallic changes the pixel through the built-in unit ORM
 *
 * Every material in the library binds the built-in ORM, so if its blue channel is
 * not 1 then `metallic = pbr_params.x * orm.b` is zero for the entire project and
 * the panel's Metallic slider, MaterialDef::metallic and a set file's "metallic"
 * field are all silent no-ops. It was 0.
 *
 * A fully metallic surface has no diffuse lobe at all -- kD is (1 - F) * (1 -
 * metallic) -- so at these angles it must render visibly darker than the
 * dielectric. Asserting a DIFFERENCE, rather than a value, keeps the test from
 * freezing the BRDF's arithmetic.
 */
TEST(PbrShader, metallic_reaches_the_shader_through_the_default_orm) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const glm::vec3 normal(-1.0f, 0.0f, 0.0f);
    const SceneUniforms scene = sun_from(normal);
    const MeshUniformsPBR mesh = wall_uniforms();

    Vertex verts[3];
    make_wall(verts, normal, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

    MaterialUniforms dielectric{};
    dielectric.pbr_params = glm::vec4(0.0f, 0.6f, 1.0f, 0.0f);
    const Texel plastic = h.draw(verts, scene, dielectric, mesh);

    MaterialUniforms metal{};
    metal.pbr_params = glm::vec4(1.0f, 0.6f, 1.0f, 0.0f);
    const Texel shiny = h.draw(verts, scene, metal, mesh);

    CHECK(plastic.luma() != shiny.luma());
    CHECK(plastic.luma() > shiny.luma());
}

/// Roughness and ambient occlusion go through the green and red channels of the
/// same texture, so they were never broken -- but they are asserted here so a
/// future edit to that texel cannot break one of the three unnoticed.
TEST(PbrShader, roughness_and_ao_reach_the_shader_too) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const glm::vec3 normal(-1.0f, 0.0f, 0.0f);
    const MeshUniformsPBR mesh = wall_uniforms();

    Vertex verts[3];
    make_wall(verts, normal, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

    // Roughness: compared under a grazing light, where the specular lobe is what
    // separates a mirror from a matte surface.
    const SceneUniforms grazing = sun_from(glm::normalize(glm::vec3(-1.0f, 1.0f, 1.0f)));
    MaterialUniforms rough{};
    rough.pbr_params = glm::vec4(0.0f, 1.0f, 1.0f, 0.0f);
    MaterialUniforms smooth{};
    smooth.pbr_params = glm::vec4(0.0f, 0.05f, 1.0f, 0.0f);
    CHECK(h.draw(verts, grazing, rough, mesh).luma() !=
          h.draw(verts, grazing, smooth, mesh).luma());

    // Ambient occlusion attenuates the ambient term only, so it is measured with
    // the sun pointing away.
    const SceneUniforms night = sun_from(normal * -1.0f);
    MaterialUniforms open{};
    open.pbr_params = glm::vec4(0.0f, 0.6f, 1.0f, 0.0f);
    MaterialUniforms occluded{};
    occluded.pbr_params = glm::vec4(0.0f, 0.6f, 0.0f, 0.0f);
    CHECK(h.draw(verts, night, open, mesh).luma() >
          h.draw(verts, night, occluded, mesh).luma());
}


// ============================================================================
// The view vector
// ============================================================================

/**
 * @brief The double-sided flip follows the CAMERA, not the world origin
 *
 * mesh_pbr.frag used to compute V as normalize(-frag_world_pos) under the
 * comment "assume camera near origin for simplicity". A Stratum scene is laid
 * out in local metres around the projection origin and the camera is routinely
 * hundreds to thousands of metres from it, so that assumption is false for most
 * of the map -- and V does not only aim the specular lobe, it decides the
 * double-sided branch `if (dot(Ng, V) < 0.0) { Ng = -Ng; B = -B; }`. Every
 * facade whose normal pointed away from the origin had its normal inverted:
 * sunlit walls came out dark, walls in shadow came out lit.
 *
 * The wall here faces -Z at world (0, 0, -5) and the sun shines straight into
 * it. Seen from a camera at (0, 0, -1000) -- in front of it -- it is lit. The
 * origin sits BEHIND that wall, so the old formula flipped the normal and
 * dropped the pixel to ambient; the assertion is that it no longer does.
 */
TEST(PbrShader, the_view_vector_follows_the_camera_not_the_world_origin) {
    if (device() == nullptr) return;
    Harness h;
    CHECK_TRUE(h.ok);
    if (!h.ok) return;

    const glm::vec3 normal(0.0f, 0.0f, -1.0f);
    Vertex verts[3];
    make_wall(verts, normal, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));

    const MeshUniformsPBR mesh = wall_uniforms();
    MaterialUniforms material{};

    // Camera in front of the wall, far from the origin, sun head-on.
    SceneUniforms front = sun_from(normal);
    front.camera_position = glm::vec4(0.0f, 0.0f, -1000.0f, 1.0f);
    const Texel lit = h.draw(verts, front, material, mesh);

    // The same wall from behind. Here the flip is CORRECT -- the shader is
    // double-sided on purpose -- so the back face falls to ambient. This is the
    // shade the front face used to be stuck at.
    SceneUniforms behind = sun_from(normal);
    behind.camera_position = glm::vec4(0.0f, 0.0f, 1000.0f, 1.0f);
    const Texel back = h.draw(verts, behind, material, mesh);

    CHECK(lit.luma() > 90);
    CHECK(lit.luma() > back.luma() + 40);
}

/**
 * @brief Exposure reaches the tone curve
 *
 * camera_position.w is the Render Settings exposure slider. The shader read it
 * nowhere, so the slider moved no pixel. It has to multiply scene-referred
 * radiance BEFORE tonemap_aces(), which is the only point at which it means
 * anything -- after the curve the value is already clamped to 0..1.
 */
TEST(PbrShader, exposure_scales_the_image) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const glm::vec3 normal(0.0f, 0.0f, -1.0f);
    Vertex verts[3];
    make_wall(verts, normal, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));

    const MeshUniformsPBR mesh = wall_uniforms();
    MaterialUniforms material{};

    SceneUniforms dim = sun_from(normal);
    dim.camera_position = glm::vec4(0.0f, 0.0f, -1000.0f, 0.25f);

    SceneUniforms bright = sun_from(normal);
    bright.camera_position = glm::vec4(0.0f, 0.0f, -1000.0f, 2.0f);

    CHECK(h.draw(verts, dim, material, mesh).luma() <
          h.draw(verts, bright, material, mesh).luma());
}


// ============================================================================
// Image-based ambient
// ============================================================================

/**
 * @brief Ambient comes from a HEMISPHERE, not from one constant
 *
 * The old ambient term was `albedo * ambient_intensity`, applied identically at
 * every orientation. That is why the underside of a bridge deck came out exactly
 * as bright as its top surface, and why a roof matched the wall beneath it: the
 * fill light had no direction, so no surface could face away from it.
 *
 * The sun is switched off entirely here, so the pixel is the ambient term and
 * nothing else. Each face is viewed from its own front, because the double-sided
 * branch would otherwise flip the normal being measured.
 */
TEST(PbrShader, hemisphere_ambient_separates_an_upward_face_from_a_downward_one) {
    if (device() == nullptr) return;
    Harness h;
    CHECK_TRUE(h.ok);
    if (!h.ok) return;

    const MeshUniformsPBR mesh = wall_uniforms();
    MaterialUniforms material{};

    Vertex up_face[3];
    make_wall(up_face, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    Vertex down_face[3];
    make_wall(down_face, glm::vec3(0.0f, -1.0f, 0.0f), glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));

    // Sun intensity 0: no direct term at all, so what is left is the sky.
    SceneUniforms from_above = sun_from(glm::vec3(0.0f, 1.0f, 0.0f));
    from_above.sun_direction.w = 0.0f;
    from_above.sun_color.a = 1.0f;                 // ambient master, full
    from_above.camera_position = glm::vec4(0.0f, 1000.0f, -5.0f, 1.0f);

    SceneUniforms from_below = from_above;
    from_below.camera_position = glm::vec4(0.0f, -1000.0f, -5.0f, 1.0f);

    const Texel lit_by_sky = h.draw(up_face, from_above, material, mesh);
    const Texel lit_by_ground = h.draw(down_face, from_below, material, mesh);

    // Both receive something: the lower lobe is ground bounce, not black.
    CHECK(lit_by_ground.luma() > 0);

    // And the sky lobe is the brighter of the two, by a margin no 8-bit rounding
    // could produce.
    CHECK(lit_by_sky.luma() > lit_by_ground.luma() + 20);

    // The sky lobe is also the bluer one, which is what says the ambient carries
    // the sky's COLOUR and not just its brightness.
    CHECK(int{lit_by_sky.b} - int{lit_by_sky.r} >
          int{lit_by_ground.b} - int{lit_by_ground.r});
}


// ============================================================================
// Cascaded shadows
// ============================================================================

namespace {

/// The light matrix the cascade tests share: a sun straight overhead, an
/// orthographic volume 80 metres across centred on the origin, and 100 metres of
/// depth range. Built the same way GPURenderer::update_shadow_cascades() builds a
/// cascade, minus the frustum fitting the fixed geometry here does not need.
glm::mat4 overhead_light_view_proj() {
    const glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 50.0f, 0.0f), glm::vec3(0.0f),
                                       glm::vec3(0.0f, 0.0f, 1.0f));
    // GLM_FORCE_DEPTH_ZERO_TO_ONE is set project-wide, so this is a 0..1 depth
    // ortho -- the convention the D32 target and the comparison sampler expect.
    const glm::mat4 proj = glm::ortho(-40.0f, 40.0f, -40.0f, 40.0f, 0.0f, 100.0f);
    return proj * view;
}

/// A one-cascade shadow block over @p light_view_proj, matching the light above.
stratum::ShadowUniforms one_cascade(const glm::mat4& light_view_proj, uint32_t map_size) {
    stratum::ShadowUniforms shadows{};
    shadows.light_view_proj[0] = light_view_proj;
    shadows.cascade_texel_world[0] = 80.0f / static_cast<float>(map_size);
    shadows.cascade_depth_bias[0] = 0.05f / 100.0f;   // 5 cm over a 100 m range
    shadows.shadow_params = glm::vec4(1.0f,      // cascade count
                                      2.0f,      // normal offset, in texels
                                      1.0f,      // strength
                                      1.0f / static_cast<float>(map_size));
    // w is the atlas tile width in UV. One cascade means one tile, so 1.0.
    shadows.shadow_fade = glm::vec4(1.0e6f, 1.0e6f + 1.0f, 1.0f, 1.0f);   // no fade
    return shadows;
}

/**
 * @brief A ground-plane fragment at world (@p x, 0, @p z), facing up
 *
 * The trick is that MeshUniformsPBR carries mvp and model SEPARATELY: mvp maps
 * the authored positions straight to NDC so the triangle covers the 1x1 target,
 * while model is what frag_world_pos comes from. Translating only the model
 * therefore moves the fragment anywhere in the world without moving it off the
 * one pixel that gets rasterised.
 */
MeshUniformsPBR ground_at(float x, float z) {
    MeshUniformsPBR mesh = wall_uniforms();
    mesh.model = glm::translate(glm::mat4(1.0f), glm::vec3(x, 0.0f, z));
    mesh.normal_matrix = glm::mat4(1.0f);   // translation only
    return mesh;
}

void make_ground(Vertex (&verts)[3]) {
    const glm::vec2 ndc[3] = { { -3.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 3.0f } };
    for (int i = 0; i < 3; ++i) {
        verts[i] = Vertex{};
        verts[i].position = glm::vec3(ndc[i], 0.0f);
        verts[i].normal = glm::vec3(0.0f, 1.0f, 0.0f);
        verts[i].uv = glm::vec2(0.0f);
        verts[i].color = glm::vec4(1.0f);
        verts[i].tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    }
}

/// Sun straight down onto an upward-facing plane: NdotL is 1, so the direct term
/// is the whole pixel and losing it to a shadow is unmissable.
SceneUniforms overhead_sun() {
    SceneUniforms scene = sun_from(glm::vec3(0.0f, 1.0f, 0.0f));
    scene.camera_position = glm::vec4(0.0f, 200.0f, 0.0f, 1.0f);
    return scene;
}

} // namespace

/**
 * @brief A caster puts the ground beneath it in shadow, and only beneath it
 *
 * This is the end-to-end check of the shadow path: the depth pass's front-face
 * culling and forward LESS depth, the cascade projection, the UV derivation, the
 * comparison sampler's direction, and the PCF kernel. None of it is observable
 * without rendering, and every part of it is the kind of thing that silently
 * produces a plausible-looking wrong image.
 *
 * The caster is deliberately placed OFF-CENTRE in both x and z. A caster centred
 * on the origin would survive a flipped U or a flipped V unchanged, and a flipped
 * shadow map is exactly the failure this is here to catch.
 */
TEST(PbrShader, a_caster_shadows_the_ground_beneath_it_and_nowhere_else) {
    if (device() == nullptr) return;
    Harness h;
    CHECK_TRUE(h.ok);
    if (!h.ok) return;

    const glm::mat4 light = overhead_light_view_proj();

    // A 10 x 10 slab at y = 8, spanning x and z in [2, 12]. Off-centre in both.
    const glm::vec3 caster[4] = {
        { 2.0f, 8.0f, 2.0f }, { 12.0f, 8.0f, 2.0f },
        { 12.0f, 8.0f, 12.0f }, { 2.0f, 8.0f, 12.0f },
    };
    CHECK_TRUE(h.render_caster(light, caster));

    h.shadows = one_cascade(light, Harness::kShadowMapSize);

    Vertex ground[3];
    make_ground(ground);
    const SceneUniforms scene = overhead_sun();
    MaterialUniforms material{};

    const Texel under = h.draw(ground, scene, material, ground_at(7.0f, 7.0f));
    const Texel beside_x = h.draw(ground, scene, material, ground_at(-7.0f, 7.0f));
    const Texel beside_z = h.draw(ground, scene, material, ground_at(7.0f, -7.0f));

    // Outside the slab in either axis, the ground is in full sun.
    CHECK(beside_x.luma() > 90);
    CHECK(beside_z.luma() > 90);

    // Under it, the direct term is gone and only the sky ambient remains.
    CHECK(under.luma() < beside_x.luma() - 40);

    // Both off-axis samples agree, which is what says neither U nor V is flipped:
    // a mirrored map would shadow one of them and light the other.
    CHECK(std::abs(beside_x.luma() - beside_z.luma()) < 8);
}

/**
 * @brief A shadowed surface keeps its ambient light
 *
 * Shadows multiply the DIRECT term only. The ambient term is the sky integrated
 * over the hemisphere, and a surface in shadow is still under that sky -- folding
 * the shadow into it as well is what turns shadows into black holes.
 */
TEST(PbrShader, a_shadowed_surface_still_receives_sky_ambient) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const glm::mat4 light = overhead_light_view_proj();
    const glm::vec3 caster[4] = {
        { 2.0f, 8.0f, 2.0f }, { 12.0f, 8.0f, 2.0f },
        { 12.0f, 8.0f, 12.0f }, { 2.0f, 8.0f, 12.0f },
    };
    CHECK_TRUE(h.render_caster(light, caster));
    h.shadows = one_cascade(light, Harness::kShadowMapSize);

    Vertex ground[3];
    make_ground(ground);
    MaterialUniforms material{};

    SceneUniforms scene = overhead_sun();
    scene.sun_color.a = 1.0f;   // ambient master, full

    const Texel shadowed = h.draw(ground, scene, material, ground_at(7.0f, 7.0f));

    // Not black: the sky still reaches it.
    CHECK(shadowed.luma() > 10);

    // And it tracks the sky. Dropping the ambient master must darken it further,
    // which it could not do if the shadow had already zeroed everything.
    SceneUniforms dim = scene;
    dim.sun_color.a = 0.05f;
    const Texel dimmer = h.draw(ground, dim, material, ground_at(7.0f, 7.0f));
    CHECK(dimmer.luma() < shadowed.luma());
}

/**
 * @brief A cascade count of zero means no shadows, not undefined comparisons
 *
 * This is the contract every other test in this file leans on, and the one the
 * renderer leans on in Simple shader mode and before the first cascade has been
 * rendered. It is asserted directly rather than assumed.
 */
TEST(PbrShader, a_zero_cascade_count_leaves_the_surface_fully_lit) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const glm::mat4 light = overhead_light_view_proj();
    const glm::vec3 caster[4] = {
        { 2.0f, 8.0f, 2.0f }, { 12.0f, 8.0f, 2.0f },
        { 12.0f, 8.0f, 12.0f }, { 2.0f, 8.0f, 12.0f },
    };
    CHECK_TRUE(h.render_caster(light, caster));

    Vertex ground[3];
    make_ground(ground);
    const SceneUniforms scene = overhead_sun();
    MaterialUniforms material{};

    // Directly under the caster, with a valid cascade: shadowed.
    h.shadows = one_cascade(light, Harness::kShadowMapSize);
    const Texel with_shadows = h.draw(ground, scene, material, ground_at(7.0f, 7.0f));

    // The same fragment, same map, count zeroed: lit.
    h.shadows.shadow_params.x = 0.0f;
    const Texel without = h.draw(ground, scene, material, ground_at(7.0f, 7.0f));

    CHECK(without.luma() > with_shadows.luma() + 40);
}

/**
 * @brief Strength lifts a shadow towards lit without moving it
 *
 * A stylistic control rather than a physical one, so it is worth pinning that it
 * is monotonic and that 0 really means "no shadow at all" -- otherwise it becomes
 * a slider nobody trusts.
 */
TEST(PbrShader, shadow_strength_scales_between_lit_and_shadowed) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const glm::mat4 light = overhead_light_view_proj();
    const glm::vec3 caster[4] = {
        { 2.0f, 8.0f, 2.0f }, { 12.0f, 8.0f, 2.0f },
        { 12.0f, 8.0f, 12.0f }, { 2.0f, 8.0f, 12.0f },
    };
    CHECK_TRUE(h.render_caster(light, caster));

    Vertex ground[3];
    make_ground(ground);
    const SceneUniforms scene = overhead_sun();
    MaterialUniforms material{};

    h.shadows = one_cascade(light, Harness::kShadowMapSize);
    h.shadows.shadow_params.z = 1.0f;
    const int full = h.draw(ground, scene, material, ground_at(7.0f, 7.0f)).luma();

    h.shadows.shadow_params.z = 0.5f;
    const int half = h.draw(ground, scene, material, ground_at(7.0f, 7.0f)).luma();

    h.shadows.shadow_params.z = 0.0f;
    const int none = h.draw(ground, scene, material, ground_at(7.0f, 7.0f)).luma();

    CHECK(full < half);
    CHECK(half < none);
}



// ============================================================================
// Baked per-vertex ambient occlusion
// ============================================================================

/**
 * @brief Baked AO darkens the AMBIENT term and leaves direct sunlight alone
 *
 * This is the whole reason Vertex::ao is its own channel instead of a factor
 * folded into the vertex colour. A vertex colour multiplies albedo, so it would
 * darken the sun as well -- and a wall does not get dimmer in direct sunlight for
 * standing next to another wall. It gets a shadow, which is a different mechanism
 * and is already handled by the cascades.
 *
 * The two halves are measured separately because either one alone would pass for
 * the wrong reason: only checking that AO darkens the ambient would accept a
 * vertex-colour implementation, and only checking that the sun is unaffected would
 * accept an implementation that does nothing at all.
 */
TEST(PbrShader, baked_ao_attenuates_ambient_but_not_direct_sun) {
    if (device() == nullptr) return;
    Harness h;
    CHECK_TRUE(h.ok);
    if (!h.ok) return;

    const glm::vec3 normal(0.0f, 0.0f, -1.0f);
    const MeshUniformsPBR mesh = wall_uniforms();
    MaterialUniforms material{};

    Vertex open[3];
    make_wall(open, normal, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    Vertex occluded[3];
    make_wall(occluded, normal, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    for (Vertex& v : occluded) v.ao = 0.2f;

    // Ambient only: the sun is off, so the pixel IS the sky term.
    SceneUniforms ambient_only = sun_from(normal);
    ambient_only.sun_direction.w = 0.0f;
    ambient_only.sun_color.a = 1.0f;
    ambient_only.camera_position = glm::vec4(0.0f, 0.0f, -1000.0f, 1.0f);

    const Texel ambient_open = h.draw(open, ambient_only, material, mesh);
    const Texel ambient_occluded = h.draw(occluded, ambient_only, material, mesh);

    CHECK(ambient_open.luma() > ambient_occluded.luma() + 15);

    // Direct only: a strong sun head-on and the ambient master at zero, so the
    // pixel is the sun term. Baked AO must not touch it.
    SceneUniforms direct_only = sun_from(normal);
    direct_only.sun_direction.w = 6.0f;
    direct_only.sun_color.a = 0.0f;
    direct_only.camera_position = glm::vec4(0.0f, 0.0f, -1000.0f, 1.0f);

    const Texel direct_open = h.draw(open, direct_only, material, mesh);
    const Texel direct_occluded = h.draw(occluded, direct_only, material, mesh);

    // The lit reference has to be genuinely lit, or "unchanged" proves nothing.
    CHECK(direct_open.luma() > 100);
    CHECK_EQ(direct_open.luma(), direct_occluded.luma());
}

/**
 * @brief The three occlusion inputs multiply
 *
 * material.pbr_params.z is what the material says, the ORM texture's red channel
 * is what the texture says at texture resolution, and Vertex::ao is what the
 * geometry says. They are independent occluders, so the shader multiplies them --
 * and a change to either of the two that this suite can drive has to still be
 * visible when the other is not at 1.
 */
TEST(PbrShader, baked_ao_multiplies_with_the_material_ao) {
    if (device() == nullptr) return;
    Harness h;
    if (!h.ok) return;

    const glm::vec3 normal(0.0f, 0.0f, -1.0f);
    const MeshUniformsPBR mesh = wall_uniforms();

    Vertex verts[3];
    make_wall(verts, normal, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    for (Vertex& v : verts) v.ao = 0.5f;

    SceneUniforms ambient_only = sun_from(normal);
    ambient_only.sun_direction.w = 0.0f;
    ambient_only.sun_color.a = 1.0f;
    ambient_only.camera_position = glm::vec4(0.0f, 0.0f, -1000.0f, 1.0f);

    MaterialUniforms open{};
    open.pbr_params = glm::vec4(0.0f, 0.6f, 1.0f, 0.0f);   // z = material ao
    MaterialUniforms half{};
    half.pbr_params = glm::vec4(0.0f, 0.6f, 0.5f, 0.0f);

    const Texel with_open_material = h.draw(verts, ambient_only, open, mesh);
    const Texel with_half_material = h.draw(verts, ambient_only, half, mesh);

    CHECK(with_half_material.luma() < with_open_material.luma());
}
