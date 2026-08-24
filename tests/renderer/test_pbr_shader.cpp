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
    bool ok = false;

    Harness() {
        dev = device();
        if (dev == nullptr) return;
        if (!textures.init(dev)) return;

        // The counts GPURenderer::load_shader() passes, from the same constants.
        vert = load_shader(dev, "mesh_pbr.vert.spv", SDL_GPU_SHADERSTAGE_VERTEX, 1u, 0u);
        frag = load_shader(dev, "mesh_pbr.frag.spv", SDL_GPU_SHADERSTAGE_FRAGMENT,
                           stratum::kPbrFragmentUniformBufferCount,
                           stratum::kMaterialSamplerCount);
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

        if (!build_pipeline()) return;
        ok = true;
    }

    ~Harness() {
        if (dev != nullptr) SDL_WaitForGPUIdle(dev);
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

        SDL_GPUVertexAttribute attrs[5]{};
        attrs[0] = { 0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(Vertex, position) };
        attrs[1] = { 1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(Vertex, normal) };
        attrs[2] = { 2, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, offsetof(Vertex, uv) };
        attrs[3] = { 3, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(Vertex, color) };
        attrs[4] = { 4, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(Vertex, tangent) };

        SDL_GPUVertexInputState vertex_input{};
        vertex_input.vertex_buffer_descriptions = &buffer_desc;
        vertex_input.num_vertex_buffers = 1;
        vertex_input.vertex_attributes = attrs;
        vertex_input.num_vertex_attributes = 5;

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

        SDL_GPUSampler* sampler = textures.sampler(SamplerKind::RepeatAniso);
        SDL_GPUTextureSamplerBinding bindings[stratum::kMaterialSamplerCount]{};
        bindings[stratum::kAlbedoSamplerSlot] = { textures.get(textures.white()), sampler };
        bindings[stratum::kNormalSamplerSlot] = { textures.get(textures.flat_normal()), sampler };
        bindings[stratum::kOrmSamplerSlot] = { textures.get(textures.default_orm()), sampler };
        SDL_BindGPUFragmentSamplers(pass, stratum::kAlbedoSamplerSlot, bindings,
                                    stratum::kMaterialSamplerCount);

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
 * normalize(-frag_world_pos), so V is exactly (0, 0, 1) there -- no accidental
 * flip in the double-sided branch, and nothing depends on where the pixel landed.
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
SceneUniforms sun_from(glm::vec3 light_dir) {
    SceneUniforms scene{};
    scene.camera_position = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);   // w = exposure
    scene.sun_direction = glm::vec4(light_dir, 3.0f);            // w = intensity
    scene.sun_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.02f);        // a = ambient
    scene.fog_params = glm::vec4(0.0f);                          // w = 0 disables fog
    scene.fog_color = glm::vec4(0.0f);
    scene.pbr_params = glm::vec4(0.0f);                          // reserved, unread
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
