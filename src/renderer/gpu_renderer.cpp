/**
 * @file gpu_renderer.cpp
 * @brief SDL_GPU renderer implementation
 */

#include "renderer/gpu_renderer.hpp"
#include "renderer/mesh.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <unordered_set>

namespace stratum {

namespace {

/**
 * @brief Alignment demanded of a pooled VERTEX range
 *
 * The vertex stride. A binding offset that is a whole number of vertices is what
 * every backend is happy with, and it costs at most 63 bytes of padding on a
 * range that is tens of kilobytes.
 */
// 64, and NO LONGER sizeof(Vertex). The two coincided until Vertex grew its baked
// ambient occlusion channel and became 68 bytes; a 68-byte alignment is not a
// power of two, so the pool would round it up to 128 and pad every allocation by
// up to 127 bytes for no benefit. The binding offset does not have to be a
// multiple of the stride here in any case: every draw binds the range at the
// mesh's first vertex and uses a vertex offset of 0, with mesh-local indices.
constexpr uint32_t kVertexAlignment = 64u;

/**
 * @brief Alignment demanded of a pooled INDEX range
 *
 * One 32-bit index. This is the load-bearing one: draw_mesh() reaches a mesh's
 * range through `first_index`, which counts INDICES rather than bytes, so the
 * range's byte offset has to divide by 4 exactly or the base is not expressible
 * and the draw reads from the wrong place -- silently, with no validation error.
 */
constexpr uint32_t kIndexAlignment = static_cast<uint32_t>(sizeof(uint32_t));

static_assert(kIndexAlignment == 4u, "draw_mesh() converts an index range's byte offset "
                                     "to a first_index by dividing by sizeof(uint32_t)");

/**
 * @brief Describe the five-attribute PBR vertex layout
 *
 * Shared by the opaque PBR pipeline and its decal variant. It is a function
 * rather than two copies of the same twenty lines because the two pipelines
 * consume the SAME vertex shader: if their vertex input states ever disagreed,
 * one of them would read tangents from the wrong offset and shade its normals
 * from garbage, which looks like a lighting bug rather than a layout bug.
 *
 * @param buffer_desc Storage for the buffer description; caller owns it
 * @param attrs       Storage for the five attributes; caller owns it
 * @param state       Filled to point at @p buffer_desc and @p attrs
 *
 * @warning @p state borrows the caller's storage. All three must share a scope.
 */
void describe_pbr_vertex_input(SDL_GPUVertexBufferDescription& buffer_desc,
                               SDL_GPUVertexAttribute (&attrs)[6],
                               SDL_GPUVertexInputState& state) {
    buffer_desc = {};
    buffer_desc.slot = 0;
    buffer_desc.pitch = sizeof(Vertex);
    buffer_desc.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    buffer_desc.instance_step_rate = 0;

    for (auto& a : attrs) a = {};

    // Position: vec3
    attrs[0].location = 0;
    attrs[0].buffer_slot = 0;
    attrs[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    attrs[0].offset = offsetof(Vertex, position);

    // Normal: vec3
    attrs[1].location = 1;
    attrs[1].buffer_slot = 0;
    attrs[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    attrs[1].offset = offsetof(Vertex, normal);

    // UV: vec2. Carries the plan's metres-based tiling coordinates for surfaces,
    // and atlas sub-rect coordinates for MaterialId::Markings.
    attrs[2].location = 2;
    attrs[2].buffer_slot = 0;
    attrs[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
    attrs[2].offset = offsetof(Vertex, uv);

    // Color: vec4
    attrs[3].location = 3;
    attrs[3].buffer_slot = 0;
    attrs[3].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4;
    attrs[3].offset = offsetof(Vertex, color);

    // Tangent: vec4 (xyz = tangent, w = bitangent sign)
    attrs[4].location = 4;
    attrs[4].buffer_slot = 0;
    attrs[4].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4;
    attrs[4].offset = offsetof(Vertex, tangent);

    // Baked ambient occlusion: float. Its own channel rather than a factor folded
    // into `color`, because it must attenuate the ambient term ONLY -- a vertex
    // colour multiplies albedo and would darken direct sunlight with it.
    attrs[5].location = 5;
    attrs[5].buffer_slot = 0;
    attrs[5].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT;
    attrs[5].offset = offsetof(Vertex, ao);

    state = {};
    state.vertex_buffer_descriptions = &buffer_desc;
    state.num_vertex_buffers = 1;
    state.vertex_attributes = attrs;
    state.num_vertex_attributes = 6;
}

} // namespace

GPURenderer::~GPURenderer() {
    shutdown();
}

bool GPURenderer::init(SDL_Window* window) {
    if (m_device) {
        spdlog::warn("GPURenderer already initialized");
        return true;
    }

    m_window = window;

    // Create GPU device - prefer Vulkan
    m_device = SDL_CreateGPUDevice(
        SDL_GPU_SHADERFORMAT_SPIRV,  // We'll provide SPIR-V shaders
        true,                         // Enable debug mode in debug builds
        nullptr                       // No specific driver preference
    );

    if (!m_device) {
        spdlog::error("Failed to create GPU device: {}", SDL_GetError());
        return false;
    }

    const char* driver = SDL_GetGPUDeviceDriver(m_device);
    spdlog::info("GPU device created with driver: {}", driver ? driver : "unknown");

    // A mesh is a RANGE from here on, not a pair of device allocations. A city
    // extract is thousands of meshes and a driver commonly caps
    // VkPhysicalDeviceLimits::maxMemoryAllocationCount at 4096, which the old
    // two-buffers-per-mesh scheme reached with VRAM to spare. See
    // gpu_buffer_pool.hpp.
    if (!m_vertex_pool.init(m_device, SDL_GPU_BUFFERUSAGE_VERTEX, kVertexBlockBytes) ||
        !m_index_pool.init(m_device, SDL_GPU_BUFFERUSAGE_INDEX, kIndexBlockBytes)) {
        spdlog::error("Failed to initialize the GPU buffer pools");
        shutdown();
        return false;
    }

    // Claim window for GPU rendering
    if (!SDL_ClaimWindowForGPUDevice(m_device, m_window)) {
        spdlog::error("Failed to claim window for GPU device: {}", SDL_GetError());
        SDL_DestroyGPUDevice(m_device);
        m_device = nullptr;
        return false;
    }

    // Get initial swapchain size
    int w, h;
    SDL_GetWindowSizeInPixels(m_window, &w, &h);
    m_swapchain_width = static_cast<uint32_t>(w);
    m_swapchain_height = static_cast<uint32_t>(h);

    // Create depth texture
    SDL_GPUTextureCreateInfo depth_info{};
    depth_info.type = SDL_GPU_TEXTURETYPE_2D;
    depth_info.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    depth_info.width = m_swapchain_width;
    depth_info.height = m_swapchain_height;
    depth_info.layer_count_or_depth = 1;
    depth_info.num_levels = 1;
    depth_info.sample_count = SDL_GPU_SAMPLECOUNT_1;
    depth_info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;

    m_depth_texture = SDL_CreateGPUTexture(m_device, &depth_info);
    if (!m_depth_texture) {
        spdlog::error("Failed to create depth texture: {}", SDL_GetError());
        shutdown();
        return false;
    }
    m_depth_alloc_width = m_swapchain_width;
    m_depth_alloc_height = m_swapchain_height;

    // Load shaders and create pipelines
    if (!load_shaders()) {
        spdlog::error("Failed to load shaders");
        shutdown();
        return false;
    }

    if (!create_pipelines()) {
        spdlog::error("Failed to create pipelines");
        shutdown();
        return false;
    }

    spdlog::info("GPURenderer initialized ({}x{})", m_swapchain_width, m_swapchain_height);
    return true;
}

void GPURenderer::shutdown() {
    if (!m_device) return;

    // Wait for GPU to finish
    SDL_WaitForGPUIdle(m_device);

    // Release all meshes
    release_all_meshes();

    // The device is idle, so every retired range is provably unreferenced. Free
    // them now rather than waiting out kBufferRetireFrames that will never
    // elapse, or the pools would report every one of them as a leak.
    drain_retired_allocs(true);
    m_vertex_pool.shutdown();
    m_index_pool.shutdown();

    // Release transfer buffer
    if (m_transfer_buffer) {
        SDL_ReleaseGPUTransferBuffer(m_device, m_transfer_buffer);
        m_transfer_buffer = nullptr;
    }

    // The shadow map and its sampler. Released here rather than in
    // release_pipelines(), because release_pipelines() also runs on an MSAA change,
    // and the shadow map has no relationship with the sample count.
    release_shadow_resources();
    m_shadow_casters_recording.clear();
    m_shadow_casters_ready.clear();

    // Textures go before the device, and before the pipelines that sample them.
    //
    // GPURenderer does NOT own the texture manager -- the editor does -- but it
    // is the only object here that knows when the device is about to be
    // destroyed, and every SDL_GPUTexture and SDL_GPUSampler the manager holds is
    // a child of that device. Releasing them afterwards is a use-after-free of
    // the device handle. shutdown() is idempotent, so the owner calling it again
    // later is harmless; the pointer is cleared so nothing here touches a manager
    // that has already been torn down.
    //
    // force = true is valid because SDL_WaitForGPUIdle() above has already run:
    // no command buffer can still be reading a retired transfer buffer, so
    // waiting out kTransferRetireFrames that will never elapse would only leak.
    if (m_texture_manager) {
        m_texture_manager->drain_retired_transfers(m_frame_index, true);
        m_texture_manager->shutdown();
        m_texture_manager = nullptr;
    }
    // The library holds only handles into the manager, so it owns no device
    // objects -- but leaving it installed would let a later draw resolve a
    // material whose textures are gone.
    m_material_library = nullptr;
    reset_material_binding();

    // Release all pipelines and shaders
    release_pipelines();

    // Release MSAA textures
    release_msaa_textures();

    // Release depth texture
    if (m_depth_texture) {
        SDL_ReleaseGPUTexture(m_device, m_depth_texture);
        m_depth_texture = nullptr;
    }
    // Must clear alongside the texture, or a re-init would believe it still has an
    // allocation of that size and never create one.
    m_depth_alloc_width = 0;
    m_depth_alloc_height = 0;

    // Release window claim and destroy device
    if (m_window) {
        SDL_ReleaseWindowFromGPUDevice(m_device, m_window);
    }
    SDL_DestroyGPUDevice(m_device);
    m_device = nullptr;
    m_window = nullptr;

    spdlog::info("GPURenderer shutdown");
}

void GPURenderer::release_pipelines() {
    // Release simple pipelines
    if (m_mesh_pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_mesh_pipeline);
        m_mesh_pipeline = nullptr;
    }
    if (m_mesh_pipeline_wireframe) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_mesh_pipeline_wireframe);
        m_mesh_pipeline_wireframe = nullptr;
    }

    // Release simple shaders
    if (m_vertex_shader) {
        SDL_ReleaseGPUShader(m_device, m_vertex_shader);
        m_vertex_shader = nullptr;
    }
    if (m_fragment_shader) {
        SDL_ReleaseGPUShader(m_device, m_fragment_shader);
        m_fragment_shader = nullptr;
    }

    // Release PBR pipelines
    if (m_pbr_pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_pbr_pipeline);
        m_pbr_pipeline = nullptr;
    }
    if (m_pbr_pipeline_wireframe) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_pbr_pipeline_wireframe);
        m_pbr_pipeline_wireframe = nullptr;
    }
    // Every decal pipeline, including the markings one m_pbr_pipeline_decal also
    // names. Released through the map only, so the shared entry is not freed twice.
    for (auto& entry : m_decal_pipelines) {
        if (entry.second) SDL_ReleaseGPUGraphicsPipeline(m_device, entry.second);
    }
    m_decal_pipelines.clear();
    m_pbr_pipeline_decal = nullptr;
    m_decal_cap_warned = false;
    // A pipeline the cache believed was bound has just been destroyed. Clearing
    // the cache here means a rebuild -- set_msaa_level() is the caller that does
    // this mid-session -- cannot skip the first bind of the new pipelines.
    reset_material_binding();

    // The shadow pipeline is NOT rebuilt by set_msaa_level(): it renders to its own
    // single-sampled depth target. It is released here all the same, because this
    // function also runs at shutdown.
    if (m_shadow_pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_shadow_pipeline);
        m_shadow_pipeline = nullptr;
    }
    if (m_shadow_vertex_shader) {
        SDL_ReleaseGPUShader(m_device, m_shadow_vertex_shader);
        m_shadow_vertex_shader = nullptr;
    }
    if (m_shadow_fragment_shader) {
        SDL_ReleaseGPUShader(m_device, m_shadow_fragment_shader);
        m_shadow_fragment_shader = nullptr;
    }

    // Release the sky pipeline. It travels with the PBR set because it shares the
    // MSAA sample count; a set_msaa_level() that rebuilt one and not the other
    // would leave a pipeline whose sample count no longer matches its pass.
    if (m_sky_pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_sky_pipeline);
        m_sky_pipeline = nullptr;
    }
    if (m_sky_vertex_shader) {
        SDL_ReleaseGPUShader(m_device, m_sky_vertex_shader);
        m_sky_vertex_shader = nullptr;
    }
    if (m_sky_fragment_shader) {
        SDL_ReleaseGPUShader(m_device, m_sky_fragment_shader);
        m_sky_fragment_shader = nullptr;
    }

    // Release PBR shaders
    if (m_pbr_vertex_shader) {
        SDL_ReleaseGPUShader(m_device, m_pbr_vertex_shader);
        m_pbr_vertex_shader = nullptr;
    }
    if (m_pbr_fragment_shader) {
        SDL_ReleaseGPUShader(m_device, m_pbr_fragment_shader);
        m_pbr_fragment_shader = nullptr;
    }
}

bool GPURenderer::load_shaders() {
    // Load both simple and PBR shaders
    if (!load_simple_shaders()) {
        return false;
    }
    
    if (!load_pbr_shaders()) {
        spdlog::warn("PBR shaders not available - PBR mode disabled");
        // Not a fatal error - simple mode still works
    }

    // Same contract as the PBR pair: a checkout without these keeps running, it
    // just falls back to the flat clear colour.
    if (!load_sky_shaders()) {
        spdlog::warn("Sky shaders not available - flat background");
    }

    if (!load_shadow_shaders()) {
        spdlog::warn("Shadow shaders not available - shadows disabled");
    }

    return true;
}

bool GPURenderer::load_simple_shaders() {
    const char* base = SDL_GetBasePath();
    std::string base_path = base ? base : "";

    std::string vert_path = base_path + "../../assets/shaders/mesh.vert.spv";
    std::string frag_path = base_path + "../../assets/shaders/mesh.frag.spv";

    // Simple shader: 1 vertex uniform buffer, 0 fragment uniforms, 0 SSBOs
    m_vertex_shader = load_shader(vert_path.c_str(), SDL_GPU_SHADERSTAGE_VERTEX, 1, 0);
    if (!m_vertex_shader) {
        spdlog::error("Failed to load simple vertex shader: {}", vert_path);
        return false;
    }

    m_fragment_shader = load_shader(frag_path.c_str(), SDL_GPU_SHADERSTAGE_FRAGMENT, 0, 0);
    if (!m_fragment_shader) {
        spdlog::error("Failed to load simple fragment shader: {}", frag_path);
        return false;
    }

    spdlog::info("Simple shaders loaded successfully");
    return true;
}

bool GPURenderer::load_pbr_shaders() {
    const char* base = SDL_GetBasePath();
    std::string base_path = base ? base : "";

    std::string vert_path = base_path + "../../assets/shaders/mesh_pbr.vert.spv";
    std::string frag_path = base_path + "../../assets/shaders/mesh_pbr.frag.spv";

    // Check if PBR shaders exist
    if (!std::filesystem::exists(vert_path) || !std::filesystem::exists(frag_path)) {
        spdlog::warn("PBR shaders not found at {}", vert_path);
        return false;
    }

    // PBR vertex shader: 1 uniform buffer (MeshUniformsPBR)
    m_pbr_vertex_shader = load_shader(vert_path.c_str(), SDL_GPU_SHADERSTAGE_VERTEX, 1, 0);
    if (!m_pbr_vertex_shader) {
        spdlog::error("Failed to load PBR vertex shader: {}", vert_path);
        return false;
    }

    // PBR fragment shader: two uniform buffers and three samplers.
    //
    // Scene at set 3 binding 0 (kSceneUniformSlot), material at set 3 binding 1
    // (kMaterialUniformSlot), and albedo/normal/ORM at set 2 bindings 0-2. The
    // counts come from material_library.hpp rather than from literals because
    // that header is what assets/shaders/mesh_pbr.frag is documented against --
    // if the GLSL grows a fourth map, the constant moves and this call site does
    // not have to be remembered.
    // kPbrFragmentSamplerCount, NOT kMaterialSamplerCount: the shader declares the
    // material's three plus the shadow map at binding 3. SDL validates this number
    // against the SPIR-V reflection, and getting it wrong either fails outright in
    // SDL_CreateGPUShader or builds a layout in which the shadow map binds nowhere.
    m_pbr_fragment_shader =
        load_shader(frag_path.c_str(), SDL_GPU_SHADERSTAGE_FRAGMENT,
                    static_cast<int>(kPbrFragmentUniformBufferCount), 0,
                    static_cast<int>(kPbrFragmentSamplerCount));
    if (!m_pbr_fragment_shader) {
        spdlog::error("Failed to load PBR fragment shader: {}", frag_path);
        SDL_ReleaseGPUShader(m_device, m_pbr_vertex_shader);
        m_pbr_vertex_shader = nullptr;
        return false;
    }

    spdlog::info("PBR shaders loaded successfully");
    return true;
}

bool GPURenderer::load_sky_shaders() {
    const char* base = SDL_GetBasePath();
    std::string base_path = base ? base : "";

    std::string vert_path = base_path + "../../assets/shaders/sky.vert.spv";
    std::string frag_path = base_path + "../../assets/shaders/sky.frag.spv";

    if (!std::filesystem::exists(vert_path) || !std::filesystem::exists(frag_path)) {
        spdlog::warn("Sky shaders not found at {}", vert_path);
        return false;
    }

    // Vertex: one uniform buffer, SkyUniforms at set 1 binding 0. No samplers and
    // NO VERTEX ATTRIBUTES -- the three vertices come from gl_VertexIndex.
    m_sky_vertex_shader = load_shader(vert_path.c_str(), SDL_GPU_SHADERSTAGE_VERTEX, 1, 0);
    if (!m_sky_vertex_shader) {
        spdlog::error("Failed to load sky vertex shader: {}", vert_path);
        return false;
    }

    // Fragment: one uniform buffer, the SAME SceneUniforms the PBR shader reads,
    // at the same slot. sky_common.glsl declares that block for both of them, so
    // the count here is 1 and not kPbrFragmentUniformBufferCount -- the sky has
    // no per-material block.
    m_sky_fragment_shader = load_shader(frag_path.c_str(), SDL_GPU_SHADERSTAGE_FRAGMENT, 1, 0);
    if (!m_sky_fragment_shader) {
        spdlog::error("Failed to load sky fragment shader: {}", frag_path);
        SDL_ReleaseGPUShader(m_device, m_sky_vertex_shader);
        m_sky_vertex_shader = nullptr;
        return false;
    }

    spdlog::info("Sky shaders loaded successfully");
    return true;
}

bool GPURenderer::load_shadow_shaders() {
    const char* base = SDL_GetBasePath();
    std::string base_path = base ? base : "";

    std::string vert_path = base_path + "../../assets/shaders/shadow.vert.spv";
    std::string frag_path = base_path + "../../assets/shaders/shadow.frag.spv";

    if (!std::filesystem::exists(vert_path) || !std::filesystem::exists(frag_path)) {
        spdlog::warn("Shadow shaders not found at {}", vert_path);
        return false;
    }

    // Vertex: one uniform buffer, the light MVP. Fragment: nothing at all -- it
    // has no outputs, the pipeline has no colour targets, and depth comes out of
    // the rasteriser.
    m_shadow_vertex_shader = load_shader(vert_path.c_str(), SDL_GPU_SHADERSTAGE_VERTEX, 1, 0);
    if (!m_shadow_vertex_shader) {
        spdlog::error("Failed to load shadow vertex shader: {}", vert_path);
        return false;
    }
    m_shadow_fragment_shader = load_shader(frag_path.c_str(), SDL_GPU_SHADERSTAGE_FRAGMENT, 0, 0);
    if (!m_shadow_fragment_shader) {
        spdlog::error("Failed to load shadow fragment shader: {}", frag_path);
        SDL_ReleaseGPUShader(m_device, m_shadow_vertex_shader);
        m_shadow_vertex_shader = nullptr;
        return false;
    }

    spdlog::info("Shadow shaders loaded successfully");
    return true;
}

SDL_GPUShader* GPURenderer::load_shader(const char* path, SDL_GPUShaderStage stage,
                                         int num_uniform_buffers, int num_storage_buffers,
                                         int num_samplers) {
    // Read shader file
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        spdlog::error("Cannot open shader file: {}", path);
        return nullptr;
    }

    size_t size = static_cast<size_t>(file.tellg());
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> code(size);
    if (!file.read(reinterpret_cast<char*>(code.data()), size)) {
        spdlog::error("Failed to read shader file: {}", path);
        return nullptr;
    }

    SDL_GPUShaderCreateInfo shader_info{};
    shader_info.code = code.data();
    shader_info.code_size = size;
    shader_info.entrypoint = "main";
    shader_info.format = SDL_GPU_SHADERFORMAT_SPIRV;
    shader_info.stage = stage;

    // Use provided resource counts.
    //
    // num_samplers is NOT cosmetic and was the one field here that was hardcoded
    // to a lie the moment mesh_pbr.frag grew textures. SDL validates these against
    // the SPIR-V reflection: a shader that declares three combined image samplers
    // and is created with num_samplers = 0 either fails outright in
    // SDL_CreateGPUShader or, worse on some backends, builds a pipeline layout
    // with no sampler bindings so every SDL_BindGPUFragmentSamplers call lands
    // nowhere and every texture read returns garbage.
    shader_info.num_uniform_buffers = num_uniform_buffers;
    shader_info.num_storage_buffers = num_storage_buffers;
    shader_info.num_storage_textures = 0;
    shader_info.num_samplers = num_samplers;

    SDL_GPUShader* shader = SDL_CreateGPUShader(m_device, &shader_info);
    if (!shader) {
        spdlog::error("Failed to create shader from {}: {}", path, SDL_GetError());
        return nullptr;
    }

    return shader;
}

bool GPURenderer::create_pipelines() {
    // Create simple pipelines first
    if (!create_simple_pipelines()) {
        return false;
    }
    
    // Try to create PBR pipelines (optional)
    if (m_pbr_vertex_shader && m_pbr_fragment_shader) {
        if (!create_pbr_pipelines()) {
            spdlog::warn("Failed to create PBR pipelines - PBR mode disabled");
        }
    }

    if (m_sky_vertex_shader && m_sky_fragment_shader) {
        if (!create_sky_pipeline()) {
            spdlog::warn("Failed to create sky pipeline - flat background");
        }
    }

    // Guarded on already-existing rather than rebuilt unconditionally, because
    // set_msaa_level() re-enters this function. Neither the shadow pipeline nor
    // the shadow map has anything to do with the swapchain's sample count -- the
    // cascade passes render to their own single-sampled depth target -- so a
    // rebuild here would free and reallocate a 2048x2048x3 texture for no reason.
    if (m_shadow_vertex_shader && m_shadow_fragment_shader) {
        const bool pipeline_ok = m_shadow_pipeline != nullptr || create_shadow_pipeline();
        const bool resources_ok = m_shadow_texture != nullptr || create_shadow_resources();
        if (!pipeline_ok || !resources_ok) {
            spdlog::warn("Failed to create shadow resources - shadows disabled");
            m_shadow_config.enabled = false;
        }
    } else {
        m_shadow_config.enabled = false;
    }

    return true;
}

bool GPURenderer::create_shadow_pipeline() {
    // ONE vertex attribute over a full-Vertex stride. The shadow pass reads
    // position and nothing else, so declaring the other four would make the input
    // assembler fetch normals, UVs, colours and tangents on every cascade for a
    // shader that discards them. The stride still has to be sizeof(Vertex),
    // because the buffer being bound is the same one the colour pass uses.
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
    // FRONT face culling, not back. Rendering only back faces into the shadow map
    // moves the stored depth to the far side of every solid object, which puts the
    // depth comparison a whole wall thickness away from the surface being shaded
    // and removes most self-shadowing acne before any bias is applied. The cost is
    // that open, single-sided geometry -- which this scene has, in road decals and
    // terrain skirts -- casts nothing; those are flat on the ground and cast
    // nothing worth having anyway.
    rasterizer.cull_mode = SDL_GPU_CULLMODE_FRONT;
    rasterizer.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    rasterizer.enable_depth_bias = false;
    rasterizer.enable_depth_clip = true;

    // FORWARD depth here, unlike the colour pass. Reverse-Z buys floating point
    // precision against a perspective projection's 1/z distribution; a cascade is
    // ORTHOGRAPHIC, so its depth is already linear and there is nothing to win.
    // Standard LESS against a 1.0 clear keeps the shader's comparison the obvious
    // way round.
    SDL_GPUDepthStencilState depth_stencil{};
    depth_stencil.compare_op = SDL_GPU_COMPAREOP_LESS;
    depth_stencil.enable_depth_test = true;
    depth_stencil.enable_depth_write = true;
    depth_stencil.enable_stencil_test = false;

    // NO COLOUR TARGETS. Depth only.
    SDL_GPUGraphicsPipelineTargetInfo target_info{};
    target_info.color_target_descriptions = nullptr;
    target_info.num_color_targets = 0;
    target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    target_info.has_depth_stencil_target = true;

    // Always 1 sample. The shadow map is its own target and has nothing to do with
    // the swapchain's MSAA level, which is why this pipeline does NOT have to be
    // rebuilt by set_msaa_level().
    SDL_GPUMultisampleState multisample{};
    multisample.sample_count = SDL_GPU_SAMPLECOUNT_1;

    SDL_GPUGraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.vertex_shader = m_shadow_vertex_shader;
    pipeline_info.fragment_shader = m_shadow_fragment_shader;
    pipeline_info.vertex_input_state = vertex_input;
    pipeline_info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    pipeline_info.rasterizer_state = rasterizer;
    pipeline_info.multisample_state = multisample;
    pipeline_info.depth_stencil_state = depth_stencil;
    pipeline_info.target_info = target_info;

    m_shadow_pipeline = SDL_CreateGPUGraphicsPipeline(m_device, &pipeline_info);
    if (!m_shadow_pipeline) {
        spdlog::error("Failed to create shadow pipeline: {}", SDL_GetError());
        return false;
    }

    spdlog::info("Shadow pipeline created");
    return true;
}

bool GPURenderer::create_shadow_resources() {
    release_shadow_resources();

    const int layers = std::clamp(m_shadow_config.cascade_count, 1, kMaxShadowCascades);
    const uint32_t size = std::clamp(m_shadow_config.map_size, 256u, 8192u);

    // ONE 2D TEXTURE, CASCADES TILED SIDE BY SIDE. A 2D array would be the natural
    // shape and is not available: SDL_GPU rejects it outright, "For array
    // textures: usage must not contain DEPTH_STENCIL_TARGET" (see the validation
    // in SDL_CreateGPUTexture). The atlas is the better shape regardless, because
    // every cascade is then filled in a SINGLE render pass with one clear and a
    // viewport change per tile.
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    info.width = size * static_cast<uint32_t>(layers);
    info.height = size;
    info.layer_count_or_depth = 1;
    info.num_levels = 1;
    info.sample_count = SDL_GPU_SAMPLECOUNT_1;
    // Both usages: written as a depth target by the cascade pass, read as a
    // sampled texture by mesh_pbr.frag.
    info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;

    m_shadow_texture = SDL_CreateGPUTexture(m_device, &info);
    if (!m_shadow_texture) {
        spdlog::error("Failed to create {}x{} shadow atlas ({} cascades): {}",
                      info.width, info.height, layers, SDL_GetError());
        return false;
    }

    SDL_GPUSamplerCreateInfo sampler_info{};
    // LINEAR with enable_compare is what makes each PCF tap a bilinear PERCENTAGE
    // rather than a single binary in-or-out test. Nearest here would make the 3x3
    // kernel produce ten discrete shades and read as stair-stepped.
    sampler_info.min_filter = SDL_GPU_FILTER_LINEAR;
    sampler_info.mag_filter = SDL_GPU_FILTER_LINEAR;
    sampler_info.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
    // CLAMP_TO_EDGE, and the shader additionally insets its cascade test by a
    // kernel radius, so a fragment near a cascade edge moves outward to a cascade
    // that still has data instead of smearing the border texel across the gap.
    sampler_info.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    sampler_info.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    sampler_info.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    sampler_info.enable_compare = true;
    // LESS: the reference is the fragment's own depth in cascade clip space, and
    // it passes -- is lit -- when it is nearer than what the caster pass stored.
    sampler_info.compare_op = SDL_GPU_COMPAREOP_LESS;

    m_shadow_sampler = SDL_CreateGPUSampler(m_device, &sampler_info);
    if (!m_shadow_sampler) {
        spdlog::error("Failed to create shadow sampler: {}", SDL_GetError());
        SDL_ReleaseGPUTexture(m_device, m_shadow_texture);
        m_shadow_texture = nullptr;
        return false;
    }

    m_shadow_allocated_size = size;
    m_shadow_allocated_layers = layers;
    spdlog::info("Shadow atlas created: {}x{} ({} cascades of {}x{})",
                 size * static_cast<uint32_t>(layers), size, layers, size, size);
    return true;
}

void GPURenderer::release_shadow_resources() {
    if (m_shadow_sampler) {
        SDL_ReleaseGPUSampler(m_device, m_shadow_sampler);
        m_shadow_sampler = nullptr;
    }
    if (m_shadow_texture) {
        SDL_ReleaseGPUTexture(m_device, m_shadow_texture);
        m_shadow_texture = nullptr;
    }
    m_shadow_allocated_size = 0;
    m_shadow_allocated_layers = 0;
}

void GPURenderer::set_shadow_config(const ShadowConfig& config) {
    const bool needs_realloc =
        config.map_size != m_shadow_allocated_size ||
        std::clamp(config.cascade_count, 1, kMaxShadowCascades) != m_shadow_allocated_layers;

    m_shadow_config = config;
    m_shadow_config.cascade_count = std::clamp(config.cascade_count, 1, kMaxShadowCascades);

    if (needs_realloc && m_shadow_pipeline) {
        // A texture the GPU may still be reading from must not be freed under it.
        // This is a settings change, not a per-frame path, so the simplest correct
        // answer is the right one.
        SDL_WaitForGPUIdle(m_device);
        if (!create_shadow_resources()) {
            m_shadow_config.enabled = false;
        }
    }
}

bool GPURenderer::create_sky_pipeline() {
    // set_msaa_level() re-enters create_pipelines(), and this pipeline DOES depend
    // on the sample count, so it is genuinely rebuilt -- which means the previous
    // one has to go first or it leaks on every MSAA change.
    if (m_sky_pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_sky_pipeline);
        m_sky_pipeline = nullptr;
    }

    // NO VERTEX INPUT AT ALL. A zero-binding, zero-attribute input state is legal
    // and is what lets draw_sky() issue SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0)
    // with nothing bound: sky.vert derives its three positions from
    // gl_VertexIndex. Leaving the mesh vertex layout declared here instead would
    // make the pipeline require a vertex buffer that will never be bound.
    SDL_GPUVertexInputState vertex_input{};
    vertex_input.num_vertex_buffers = 0;
    vertex_input.num_vertex_attributes = 0;

    SDL_GPURasterizerState rasterizer{};
    rasterizer.fill_mode = SDL_GPU_FILLMODE_FILL;
    // The generated triangle's winding is whatever gl_VertexIndex produces, and
    // it is not worth reasoning about: culling nothing is correct for a single
    // fullscreen primitive and cannot be got wrong by a later edit.
    rasterizer.cull_mode = SDL_GPU_CULLMODE_NONE;
    rasterizer.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    rasterizer.enable_depth_bias = false;
    rasterizer.enable_depth_clip = true;

    // Depth OFF in both directions. The sky is drawn first and painted over, so
    // it needs no test; and it must not write, or every subsequent GREATER test
    // in the reverse-Z depth buffer would be compared against the sky's own
    // depth instead of the cleared far plane.
    SDL_GPUDepthStencilState depth_stencil{};
    depth_stencil.enable_depth_test = false;
    depth_stencil.enable_depth_write = false;
    depth_stencil.enable_stencil_test = false;

    SDL_GPUColorTargetDescription color_target{};
    color_target.format = SDL_GetGPUSwapchainTextureFormat(m_device, m_window);
    SDL_GPUColorTargetBlendState blend{};
    blend.enable_blend = false;
    blend.color_write_mask = SDL_GPU_COLORCOMPONENT_R | SDL_GPU_COLORCOMPONENT_G |
                             SDL_GPU_COLORCOMPONENT_B | SDL_GPU_COLORCOMPONENT_A;
    color_target.blend_state = blend;

    SDL_GPUGraphicsPipelineTargetInfo target_info{};
    target_info.color_target_descriptions = &color_target;
    target_info.num_color_targets = 1;
    // The pass this runs in HAS a depth attachment, so the pipeline must declare
    // one and its format must match, even though nothing here touches it.
    target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    target_info.has_depth_stencil_target = true;

    SDL_GPUMultisampleState multisample{};
    multisample.sample_count = m_sample_count;

    SDL_GPUGraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.vertex_shader = m_sky_vertex_shader;
    pipeline_info.fragment_shader = m_sky_fragment_shader;
    pipeline_info.vertex_input_state = vertex_input;
    pipeline_info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    pipeline_info.rasterizer_state = rasterizer;
    pipeline_info.multisample_state = multisample;
    pipeline_info.depth_stencil_state = depth_stencil;
    pipeline_info.target_info = target_info;

    m_sky_pipeline = SDL_CreateGPUGraphicsPipeline(m_device, &pipeline_info);
    if (!m_sky_pipeline) {
        spdlog::error("Failed to create sky pipeline: {}", SDL_GetError());
        return false;
    }

    spdlog::info("Sky pipeline created");
    return true;
}

bool GPURenderer::create_simple_pipelines() {
    // Vertex input layout matching our Vertex struct
    // Note: Vertex has tangent but simple shader only uses 4 attributes
    SDL_GPUVertexBufferDescription vertex_buffer_desc{};
    vertex_buffer_desc.slot = 0;
    vertex_buffer_desc.pitch = sizeof(Vertex);
    vertex_buffer_desc.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    vertex_buffer_desc.instance_step_rate = 0;

    // Simple shader only uses 4 vertex attributes (no tangent)
    SDL_GPUVertexAttribute vertex_attributes[4]{};

    // Position: vec3 at offset 0
    vertex_attributes[0].location = 0;
    vertex_attributes[0].buffer_slot = 0;
    vertex_attributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    vertex_attributes[0].offset = offsetof(Vertex, position);

    // Normal: vec3 at offset 12
    vertex_attributes[1].location = 1;
    vertex_attributes[1].buffer_slot = 0;
    vertex_attributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    vertex_attributes[1].offset = offsetof(Vertex, normal);

    // UV: vec2 at offset 24
    vertex_attributes[2].location = 2;
    vertex_attributes[2].buffer_slot = 0;
    vertex_attributes[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
    vertex_attributes[2].offset = offsetof(Vertex, uv);

    // Color: vec4 at offset 32
    vertex_attributes[3].location = 3;
    vertex_attributes[3].buffer_slot = 0;
    vertex_attributes[3].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4;
    vertex_attributes[3].offset = offsetof(Vertex, color);

    SDL_GPUVertexInputState vertex_input{};
    vertex_input.vertex_buffer_descriptions = &vertex_buffer_desc;
    vertex_input.num_vertex_buffers = 1;
    vertex_input.vertex_attributes = vertex_attributes;
    vertex_input.num_vertex_attributes = 4;

    // Rasterizer state
    SDL_GPURasterizerState rasterizer{};
    rasterizer.fill_mode = SDL_GPU_FILLMODE_FILL;
    rasterizer.cull_mode = SDL_GPU_CULLMODE_BACK;
    rasterizer.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    rasterizer.depth_bias_constant_factor = 0.0f;
    rasterizer.depth_bias_clamp = 0.0f;
    rasterizer.depth_bias_slope_factor = 0.0f;
    rasterizer.enable_depth_bias = false;
    rasterizer.enable_depth_clip = true;

    // Depth stencil state
    SDL_GPUDepthStencilState depth_stencil{};
    depth_stencil.compare_op = SDL_GPU_COMPAREOP_GREATER;  // reverse-Z
    depth_stencil.enable_depth_test = true;
    depth_stencil.enable_depth_write = true;
    depth_stencil.enable_stencil_test = false;

    // Color target (swapchain format)
    SDL_GPUColorTargetDescription color_target{};
    color_target.format = SDL_GetGPUSwapchainTextureFormat(m_device, m_window);

    // Blend state for color target
    SDL_GPUColorTargetBlendState blend{};
    blend.enable_blend = false;
    blend.src_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
    blend.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
    blend.color_blend_op = SDL_GPU_BLENDOP_ADD;
    blend.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
    blend.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
    blend.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
    blend.color_write_mask = SDL_GPU_COLORCOMPONENT_R | SDL_GPU_COLORCOMPONENT_G |
                             SDL_GPU_COLORCOMPONENT_B | SDL_GPU_COLORCOMPONENT_A;
    color_target.blend_state = blend;

    // Pipeline target info
    SDL_GPUGraphicsPipelineTargetInfo target_info{};
    target_info.color_target_descriptions = &color_target;
    target_info.num_color_targets = 1;
    target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    target_info.has_depth_stencil_target = true;

    // Multisample state
    SDL_GPUMultisampleState multisample{};
    multisample.sample_count = m_sample_count;
    multisample.sample_mask = 0;
    multisample.enable_mask = false;

    // Create the pipeline
    SDL_GPUGraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.vertex_shader = m_vertex_shader;
    pipeline_info.fragment_shader = m_fragment_shader;
    pipeline_info.vertex_input_state = vertex_input;
    pipeline_info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    pipeline_info.rasterizer_state = rasterizer;
    pipeline_info.multisample_state = multisample;
    pipeline_info.depth_stencil_state = depth_stencil;
    pipeline_info.target_info = target_info;

    m_mesh_pipeline = SDL_CreateGPUGraphicsPipeline(m_device, &pipeline_info);
    if (!m_mesh_pipeline) {
        spdlog::error("Failed to create mesh pipeline: {}", SDL_GetError());
        return false;
    }

    // Create wireframe pipeline (same as solid but with line fill mode)
    rasterizer.fill_mode = SDL_GPU_FILLMODE_LINE;
    pipeline_info.rasterizer_state = rasterizer;

    m_mesh_pipeline_wireframe = SDL_CreateGPUGraphicsPipeline(m_device, &pipeline_info);
    if (!m_mesh_pipeline_wireframe) {
        spdlog::error("Failed to create wireframe pipeline: {}", SDL_GetError());
        return false;
    }

    spdlog::info("Simple graphics pipelines created (solid + wireframe)");
    return true;
}

bool GPURenderer::create_pbr_pipelines() {
    // PBR shader uses 6 vertex attributes (tangent and baked AO included)
    SDL_GPUVertexBufferDescription vertex_buffer_desc{};
    SDL_GPUVertexAttribute vertex_attributes[6]{};
    SDL_GPUVertexInputState vertex_input{};
    describe_pbr_vertex_input(vertex_buffer_desc, vertex_attributes, vertex_input);

    // Rasterizer state
    SDL_GPURasterizerState rasterizer{};
    rasterizer.fill_mode = SDL_GPU_FILLMODE_FILL;
    rasterizer.cull_mode = SDL_GPU_CULLMODE_BACK;
    rasterizer.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    rasterizer.enable_depth_bias = false;
    rasterizer.enable_depth_clip = true;

    // Depth stencil state
    SDL_GPUDepthStencilState depth_stencil{};
    depth_stencil.compare_op = SDL_GPU_COMPAREOP_GREATER;  // reverse-Z
    depth_stencil.enable_depth_test = true;
    depth_stencil.enable_depth_write = true;
    depth_stencil.enable_stencil_test = false;

    // Color target
    SDL_GPUColorTargetDescription color_target{};
    color_target.format = SDL_GetGPUSwapchainTextureFormat(m_device, m_window);

    SDL_GPUColorTargetBlendState blend{};
    blend.enable_blend = false;
    blend.color_write_mask = SDL_GPU_COLORCOMPONENT_R | SDL_GPU_COLORCOMPONENT_G |
                             SDL_GPU_COLORCOMPONENT_B | SDL_GPU_COLORCOMPONENT_A;
    color_target.blend_state = blend;

    // Pipeline target info
    SDL_GPUGraphicsPipelineTargetInfo target_info{};
    target_info.color_target_descriptions = &color_target;
    target_info.num_color_targets = 1;
    target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    target_info.has_depth_stencil_target = true;

    // Multisample state
    SDL_GPUMultisampleState multisample{};
    multisample.sample_count = m_sample_count;

    // Create the PBR pipeline
    SDL_GPUGraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.vertex_shader = m_pbr_vertex_shader;
    pipeline_info.fragment_shader = m_pbr_fragment_shader;
    pipeline_info.vertex_input_state = vertex_input;
    pipeline_info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    pipeline_info.rasterizer_state = rasterizer;
    pipeline_info.multisample_state = multisample;
    pipeline_info.depth_stencil_state = depth_stencil;
    pipeline_info.target_info = target_info;

    m_pbr_pipeline = SDL_CreateGPUGraphicsPipeline(m_device, &pipeline_info);
    if (!m_pbr_pipeline) {
        spdlog::error("Failed to create PBR pipeline: {}", SDL_GetError());
        return false;
    }

    // Create PBR wireframe pipeline
    rasterizer.fill_mode = SDL_GPU_FILLMODE_LINE;
    pipeline_info.rasterizer_state = rasterizer;

    m_pbr_pipeline_wireframe = SDL_CreateGPUGraphicsPipeline(m_device, &pipeline_info);
    if (!m_pbr_pipeline_wireframe) {
        spdlog::error("Failed to create PBR wireframe pipeline: {}", SDL_GetError());
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_pbr_pipeline);
        m_pbr_pipeline = nullptr;
        return false;
    }

    // The decal variant travels with this set: it shares the shaders and the MSAA
    // sample count, so it must be created here and released in release_pipelines()
    // with the rest, or a set_msaa_level() would leave a pipeline behind that
    // points at freed shaders.
    m_pbr_pipeline_decal = decal_pipeline_for(MaterialLibrary::kMarkingDepthBias);
    if (!m_pbr_pipeline_decal) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_pbr_pipeline_wireframe);
        m_pbr_pipeline_wireframe = nullptr;
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_pbr_pipeline);
        m_pbr_pipeline = nullptr;
        return false;
    }

    spdlog::info("PBR graphics pipelines created (solid + wireframe + decal)");
    return true;
}

uint32_t GPURenderer::decal_bias_key(float depth_bias) {
    // NaN compares false against both bounds, so it lands on 0 rather than
    // propagating into an index. 1/16 quantum over [-16, 16] is 513 possible keys.
    float clamped = depth_bias;
    if (!(clamped > -kMaxDecalDepthBias)) clamped = -kMaxDecalDepthBias;
    if (!(clamped < kMaxDecalDepthBias)) clamped = kMaxDecalDepthBias;
    const int32_t steps = static_cast<int32_t>(std::lround(clamped * 16.0f));
    return static_cast<uint32_t>(steps + static_cast<int32_t>(kMaxDecalDepthBias) * 16);
}

float GPURenderer::decal_bias_from_key(uint32_t key) {
    const int32_t steps =
        static_cast<int32_t>(key) - static_cast<int32_t>(kMaxDecalDepthBias) * 16;
    return static_cast<float>(steps) / 16.0f;
}

SDL_GPUGraphicsPipeline* GPURenderer::decal_pipeline_for(float depth_bias) {
    const uint32_t key = decal_bias_key(depth_bias);
    if (const auto it = m_decal_pipelines.find(key); it != m_decal_pipelines.end()) {
        return it->second;
    }

    // A DragFloat can walk through a lot of distinct values; the cap is what stops
    // a drag from turning into a pipeline leak. Past it, the material still draws
    // blended and biased, just at the markings bias rather than its own.
    if (m_decal_pipelines.size() >= kMaxDecalPipelines) {
        if (!m_decal_cap_warned) {
            m_decal_cap_warned = true;
            spdlog::warn("GPURenderer: {} distinct decal depth biases in use; further values "
                         "draw with the markings bias ({})",
                         m_decal_pipelines.size(), MaterialLibrary::kMarkingDepthBias);
        }
        return m_pbr_pipeline_decal;
    }

    SDL_GPUGraphicsPipeline* pipeline = create_decal_pipeline(decal_bias_from_key(key));
    if (!pipeline) return m_pbr_pipeline_decal;

    m_decal_pipelines.emplace(key, pipeline);
    return pipeline;
}

SDL_GPUGraphicsPipeline* GPURenderer::create_decal_pipeline(float depth_bias) {
    SDL_GPUVertexBufferDescription vertex_buffer_desc{};
    SDL_GPUVertexAttribute vertex_attributes[6]{};
    SDL_GPUVertexInputState vertex_input{};
    describe_pbr_vertex_input(vertex_buffer_desc, vertex_attributes, vertex_input);

    // Rasterizer: same as opaque, plus a constant depth bias.
    //
    // Marking quads are emitted a few millimetres above the carriageway, which is
    // nowhere near enough across a city-sized reverse-Z depth range: at grazing
    // angles the two surfaces quantise to the same depth value and the paint
    // stipples in and out as the camera moves. A constant-factor bias is the fix
    // that does not depend on how far away the road happens to be.
    //
    // The sign is set by REVERSE-Z, and BOTH terms carry it -- see
    // GPURenderer::decal_depth_bias(), which is the one place that arithmetic
    // lives and which documents why a negated constant beside a hardcoded
    // negative slope was worse than no bias at all.
    const DecalDepthBias bias = decal_depth_bias(depth_bias);

    SDL_GPURasterizerState rasterizer{};
    rasterizer.fill_mode = SDL_GPU_FILLMODE_FILL;
    rasterizer.cull_mode = SDL_GPU_CULLMODE_BACK;
    rasterizer.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    rasterizer.enable_depth_bias = true;
    rasterizer.depth_bias_constant_factor = bias.constant;
    rasterizer.depth_bias_slope_factor = bias.slope;
    rasterizer.depth_bias_clamp = 0.0f;
    rasterizer.enable_depth_clip = true;

    // Depth: TEST but do not WRITE.
    //
    // A decal must still be hidden by a building in front of it, so the test
    // stays on. It must not write, because a blended fragment that wrote depth
    // would occlude the decals drawn after it and make the draw order visible as
    // missing paint at every crossing where two markings overlap.
    SDL_GPUDepthStencilState depth_stencil{};
    depth_stencil.compare_op = SDL_GPU_COMPAREOP_GREATER;   // reverse-Z, as opaque
    depth_stencil.enable_depth_test = true;
    depth_stencil.enable_depth_write = false;
    depth_stencil.enable_stencil_test = false;

    // Standard source-alpha over. The alpha channel of the target is composited
    // with ONE / ONE_MINUS_SRC_ALPHA rather than the colour factors, so drawing
    // onto a transparent target does not leave the coverage wrong.
    SDL_GPUColorTargetDescription color_target{};
    color_target.format = SDL_GetGPUSwapchainTextureFormat(m_device, m_window);

    SDL_GPUColorTargetBlendState blend{};
    blend.enable_blend = true;
    blend.src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA;
    blend.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
    blend.color_blend_op = SDL_GPU_BLENDOP_ADD;
    blend.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
    blend.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
    blend.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
    blend.color_write_mask = SDL_GPU_COLORCOMPONENT_R | SDL_GPU_COLORCOMPONENT_G |
                             SDL_GPU_COLORCOMPONENT_B | SDL_GPU_COLORCOMPONENT_A;
    color_target.blend_state = blend;

    SDL_GPUGraphicsPipelineTargetInfo target_info{};
    target_info.color_target_descriptions = &color_target;
    target_info.num_color_targets = 1;
    target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    target_info.has_depth_stencil_target = true;

    SDL_GPUMultisampleState multisample{};
    multisample.sample_count = m_sample_count;

    SDL_GPUGraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.vertex_shader = m_pbr_vertex_shader;
    pipeline_info.fragment_shader = m_pbr_fragment_shader;
    pipeline_info.vertex_input_state = vertex_input;
    pipeline_info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    pipeline_info.rasterizer_state = rasterizer;
    pipeline_info.multisample_state = multisample;
    pipeline_info.depth_stencil_state = depth_stencil;
    pipeline_info.target_info = target_info;

    SDL_GPUGraphicsPipeline* pipeline = SDL_CreateGPUGraphicsPipeline(m_device, &pipeline_info);
    if (!pipeline) {
        spdlog::error("Failed to create PBR decal pipeline (depth bias {}): {}",
                      depth_bias, SDL_GetError());
        return nullptr;
    }

    return pipeline;
}

uint32_t GPURenderer::upload_mesh(const Mesh& mesh) {
    if (!m_device || mesh.vertices.empty()) {
        return 0;
    }

    // Validate indices don't exceed vertex count
    for (uint32_t idx : mesh.indices) {
        if (idx >= mesh.vertices.size()) {
            spdlog::error("Mesh has invalid index {} (vertex count: {})", idx, mesh.vertices.size());
            return 0;
        }
    }

    const size_t vertex_size = mesh.vertices.size() * sizeof(Vertex);
    const size_t index_size = mesh.indices.size() * sizeof(uint32_t);
    const size_t total_size = vertex_size + index_size;

    // Make room BEFORE allocating, not after. Allocating first and evicting
    // afterwards would let the pool grow a block to hold this mesh -- which is
    // the device allocation the budget exists to avoid -- and only then hand the
    // freed space back.
    if (m_memory_budget.evict_under_pressure) {
        evict_to_fit(total_size, 1);
    }
    if (m_resident_bytes + total_size > m_memory_budget.max_resident_bytes ||
        m_meshes.size() + 1 > m_memory_budget.max_resident_meshes) {
        // Either eviction is switched off, or everything still resident is pinned
        // or mid-upload. Refuse rather than quietly breaching the cap: geometry
        // missing with upload_failures() climbing is diagnosable, geometry that
        // pushed the process into a driver OOM is not.
        ++m_upload_failures;
        return 0;
    }

    GPUMesh gpu_mesh{};
    gpu_mesh.vertex_count = static_cast<uint32_t>(mesh.vertices.size());
    gpu_mesh.index_count = static_cast<uint32_t>(mesh.indices.size());
    // effective_submeshes() resolves the implicit whole-mesh case, so an
    // untagged mesh records exactly one MaterialId::Default range covering
    // [0, index_count) and draws as a single call just as it did before.
    gpu_mesh.submeshes = mesh.effective_submeshes();

    // Reserve the ranges now so the handle returned here is immediately valid,
    // but DEFER the copy into the next batched flush. See flush_pending_uploads()
    // for why the copy is not submitted per mesh.
    gpu_mesh.vertex_alloc = m_vertex_pool.allocate(static_cast<uint32_t>(vertex_size),
                                                   kVertexAlignment);
    if (!gpu_mesh.vertex_alloc.valid()) {
        const GPUBufferPool::Stats vs = m_vertex_pool.stats();
        spdlog::error("Vertex pool refused {} KB ({} blocks, {} KB reserved, {} KB used, "
                      "fragmentation {:.2f}, {} meshes resident)",
                      vertex_size / 1024, vs.blocks, vs.bytes_reserved / 1024,
                      vs.bytes_used / 1024, vs.fragmentation, m_meshes.size());
        ++m_upload_failures;
        return 0;
    }

    // Enforced HERE rather than trusted at the draw. The pool floors an index
    // pool's alignment to 4 on its own, so this cannot trip -- but a draw reading
    // from a misaligned base produces plausible-looking garbage triangles rather
    // than an error, and that is not a bug anyone finds twice.
    if ((gpu_mesh.vertex_alloc.offset % kVertexAlignment) != 0u) {
        spdlog::error("Vertex range at byte offset {} is not a multiple of the {} byte vertex "
                      "stride; refusing the upload", gpu_mesh.vertex_alloc.offset, kVertexAlignment);
        m_vertex_pool.free(gpu_mesh.vertex_alloc);
        ++m_upload_failures;
        return 0;
    }

    if (index_size > 0) {
        gpu_mesh.index_alloc = m_index_pool.allocate(static_cast<uint32_t>(index_size),
                                                     kIndexAlignment);
        if (!gpu_mesh.index_alloc.valid()) {
            const GPUBufferPool::Stats is = m_index_pool.stats();
            spdlog::error("Index pool refused {} KB ({} blocks, {} KB reserved, {} KB used, "
                          "fragmentation {:.2f}, {} meshes resident)",
                          index_size / 1024, is.blocks, is.bytes_reserved / 1024,
                          is.bytes_used / 1024, is.fragmentation, m_meshes.size());
            m_vertex_pool.free(gpu_mesh.vertex_alloc);
            ++m_upload_failures;
            return 0;
        }
        if ((gpu_mesh.index_alloc.offset % kIndexAlignment) != 0u) {
            // draw_mesh() turns this offset into a first_index by dividing by 4.
            // An offset that is not a whole number of indices makes that division
            // lossy and the mesh would draw from somebody else's triangles.
            spdlog::error("Index range at byte offset {} is not a whole number of 32-bit "
                          "indices; refusing the upload", gpu_mesh.index_alloc.offset);
            m_index_pool.free(gpu_mesh.index_alloc);
            m_vertex_pool.free(gpu_mesh.vertex_alloc);
            ++m_upload_failures;
            return 0;
        }
    }

    // Stage the bytes CPU-side; the flush moves the whole batch through the
    // transfer buffer in one copy pass.
    const size_t staging_offset = m_staging.size();
    m_staging.resize(staging_offset + total_size);
    memcpy(m_staging.data() + staging_offset, mesh.vertices.data(), vertex_size);
    if (index_size > 0) {
        memcpy(m_staging.data() + staging_offset + vertex_size,
               mesh.indices.data(), index_size);
    }

    const uint32_t mesh_id = m_next_mesh_id++;
    gpu_mesh.ready = false;
    m_meshes[mesh_id] = std::move(gpu_mesh);
    m_resident_bytes += total_size;

    m_pending_uploads.push_back(PendingUpload{
        mesh_id, staging_offset,
        static_cast<uint32_t>(vertex_size),
        static_cast<uint32_t>(index_size)
    });

    return mesh_id;
}

UploadBatch plan_upload_batch(const std::vector<PendingUpload>& queue, size_t budget) {
    UploadBatch batch;
    batch.offsets.reserve(queue.size());

    for (const PendingUpload& p : queue) {
        const size_t size = p.total_bytes();
        if (!batch.offsets.empty() && batch.bytes + size > budget) break;
        batch.offsets.push_back(batch.bytes);
        batch.bytes += size;
    }
    return batch;
}

size_t staging_compaction_offset(const std::vector<PendingUpload>& queue, size_t arena_bytes) {
    if (queue.empty()) {
        return arena_bytes;   // nothing live: the whole arena is dead
    }
    const size_t dead = std::min(queue.front().staging_offset, arena_bytes);
    return dead * 2u >= arena_bytes ? dead : 0u;
}

void GPURenderer::flush_pending_uploads() {
    if (!m_device) return;

    // Textures share this function's command buffer and copy pass rather than
    // opening their own. That is the whole point of routing them through here:
    // the bug this batching exists to prevent -- one submit per resource, from
    // inside the visible-node traversal, until the driver's host allocator gave
    // out -- would come straight back if the texture manager submitted its own
    // command buffer per load during an import.
    const bool have_textures = m_texture_manager && m_texture_manager->pending_upload_count() > 0;

    // Admit as many staged meshes as this frame's budget allows; the rest stay
    // queued for later frames, which is what turns a stampeding import into a
    // stream. The batch also decides where each entry lands in the transfer
    // buffer, because the staging arena it came from is not packed -- see
    // plan_upload_batch().
    UploadBatch batch;
    size_t batch_count = 0;
    if (!m_pending_uploads.empty()) {
        batch = plan_upload_batch(m_pending_uploads, kMaxUploadBytesPerFrame);
        batch_count = batch.offsets.size();
    }

    // Nothing of either kind: no command buffer, no submit.
    if (batch_count == 0 && !have_textures) return;

    // The transfer buffer and the host copy below are the MESH path only; the
    // texture manager stages into its own transfer buffers. When only textures
    // are pending this whole block is skipped and the copy pass is opened
    // directly.
    if (batch_count > 0) {
    if (!m_transfer_buffer || m_transfer_buffer_size < batch.bytes) {
        if (m_transfer_buffer) {
            SDL_ReleaseGPUTransferBuffer(m_device, m_transfer_buffer);
        }
        SDL_GPUTransferBufferCreateInfo transfer_info{};
        transfer_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        transfer_info.size = static_cast<uint32_t>(batch.bytes);
        m_transfer_buffer = SDL_CreateGPUTransferBuffer(m_device, &transfer_info);
        m_transfer_buffer_size = m_transfer_buffer ? batch.bytes : 0;
        if (!m_transfer_buffer) {
            spdlog::error("Failed to create transfer buffer ({} KB): {}",
                          batch.bytes / 1024, SDL_GetError());
            ++m_upload_failures;
            return;
        }
    }

    // cycle = true: SDL rotates to a fresh internal allocation if the GPU is
    // still reading the previous contents, so this never stalls.
    void* mapped = SDL_MapGPUTransferBuffer(m_device, m_transfer_buffer, true);
    if (!mapped) {
        spdlog::error("Failed to map transfer buffer: {}", SDL_GetError());
        ++m_upload_failures;
        return;
    }
    // One entry at a time, into the destination the plan assigned it. NOT one
    // memcpy of a window of the arena: the arena has holes in it wherever a mesh
    // was released between being staged and being flushed.
    for (size_t i = 0; i < batch_count; ++i) {
        const PendingUpload& p = m_pending_uploads[i];
        const size_t size = p.total_bytes();
        if (p.staging_offset + size > m_staging.size()) {
            continue;   // cannot happen; a truncated arena must not be read past
        }
        memcpy(static_cast<uint8_t*>(mapped) + batch.offsets[i],
               m_staging.data() + p.staging_offset, size);
    }
    SDL_UnmapGPUTransferBuffer(m_device, m_transfer_buffer);
    }   // end mesh-only staging

    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
    if (!cmd) {
        spdlog::error("Failed to acquire upload command buffer: {}", SDL_GetError());
        ++m_upload_failures;
        return;
    }

    SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);

    // Texture copies first. Order between the two is irrelevant -- they touch
    // disjoint resources -- but recording textures before the early-`continue`
    // maze of the mesh loop keeps it obvious that they are not inside it.
    if (have_textures) {
        m_texture_manager->flush_uploads(copy_pass, m_frame_index);
    }

    for (size_t i = 0; i < batch_count; ++i) {
        const PendingUpload& p = m_pending_uploads[i];
        auto it = m_meshes.find(p.mesh_id);
        if (it == m_meshes.end()) continue;   // released before it ever flushed
        GPUMesh& gm = it->second;

        const uint32_t rel = static_cast<uint32_t>(batch.offsets[i]);

        // The batching is unchanged by pooling; only the DESTINATION moved. A
        // SDL_GPUBufferRegion already carries an offset, so a copy into a
        // suballocated range is the same call it was into a whole buffer, aimed
        // at {block, range offset} instead of {buffer, 0}. cycle stays false:
        // cycling a shared block would hand back a fresh allocation and discard
        // every OTHER mesh living in it.
        SDL_GPUTransferBufferLocation src_vertex{};
        src_vertex.transfer_buffer = m_transfer_buffer;
        src_vertex.offset = rel;
        SDL_GPUBufferRegion dst_vertex{};
        dst_vertex.buffer = gm.vertex_alloc.buffer;
        dst_vertex.offset = gm.vertex_alloc.offset;
        dst_vertex.size = p.vertex_bytes;
        SDL_UploadToGPUBuffer(copy_pass, &src_vertex, &dst_vertex, false);

        if (p.index_bytes > 0 && gm.index_alloc.valid()) {
            SDL_GPUTransferBufferLocation src_index{};
            src_index.transfer_buffer = m_transfer_buffer;
            src_index.offset = rel + p.vertex_bytes;
            SDL_GPUBufferRegion dst_index{};
            dst_index.buffer = gm.index_alloc.buffer;
            dst_index.offset = gm.index_alloc.offset;
            dst_index.size = p.index_bytes;
            SDL_UploadToGPUBuffer(copy_pass, &src_index, &dst_index, false);
        }
        gm.ready = true;
    }
    SDL_EndGPUCopyPass(copy_pass);

    // ONE submit for the whole batch.
    //
    // This replaced a submit PER MESH issued from inside the visible-node
    // traversal -- i.e. thousands of command buffers inside a single frame,
    // while that frame's own render pass was still open. Every submit allocates
    // a fence, and with no GPU progress between them none could be retired, so a
    // city-scale import piled up hundreds of live command buffers and exhausted
    // the driver's HOST allocator:
    //     vkCreateFence VK_ERROR_OUT_OF_HOST_MEMORY
    // after which buffer binds failed too, the swapchain semaphores were left
    // signalled, and the app died on VUID-vkAcquireNextImageKHR-semaphore-01286.
    // It was never a VRAM shortage.
    const bool submitted = SDL_SubmitGPUCommandBuffer(cmd);

    // Readiness on BOTH halves of the batch is contingent on that submit. The mesh
    // half rolls its own flags back below; the texture half cannot do it for
    // itself, because flush_uploads() records into a command buffer it does not
    // own and never sees the result -- hence commit_uploads(), called on both
    // paths. Without it a failed submit left every texture in the batch flagged
    // ready over uninitialised device memory, with its staged pixels already
    // freed: is_ready() true forever, bind_texture() no longer substituting the
    // fallback, and no way back short of restarting.
    if (m_texture_manager) {
        m_texture_manager->commit_uploads(submitted);
    }

    if (!submitted) {
        spdlog::error("Batched upload submit failed ({} meshes, {} KB): {}",
                      batch_count, batch.bytes / 1024, SDL_GetError());
        for (size_t i = 0; i < batch_count; ++i) {
            auto it = m_meshes.find(m_pending_uploads[i].mesh_id);
            if (it != m_meshes.end()) it->second.ready = false;
        }
        ++m_upload_failures;
        return;
    }

    m_pending_uploads.erase(m_pending_uploads.begin(),
                            m_pending_uploads.begin() + static_cast<long>(batch_count));

    if (m_pending_uploads.empty()) {
        m_staging.clear();
        m_staging.shrink_to_fit();
        return;
    }

    // Reclaim the dead prefix, but only once there is enough of it to be worth a
    // memmove of everything still queued. shrink_to_fit() is deliberately NOT
    // called: it reallocates and copies the remainder a second time, and doubles
    // peak host memory for the duration of the copy, on a path that runs every
    // frame of an import.
    const size_t keep_from = staging_compaction_offset(m_pending_uploads, m_staging.size());
    if (keep_from > 0) {
        m_staging.erase(m_staging.begin(), m_staging.begin() + static_cast<long>(keep_from));
        for (auto& p : m_pending_uploads) {
            p.staging_offset -= std::min(p.staging_offset, keep_from);
        }
    }
}

void GPURenderer::release_mesh(uint32_t mesh_id) {
    auto it = m_meshes.find(mesh_id);
    if (it == m_meshes.end()) return;

    // A mesh can be released before its staged copy ever flushed (streamed in
    // and back out within a frame). Drop the pending entry so the flush does not
    // later write into a range that now belongs to something else.
    //
    // Its bytes stay in the staging arena as a hole until the next compaction,
    // which is why the flush packs the transfer buffer itself instead of
    // treating the queue's staging offsets as contiguous.
    std::erase_if(m_pending_uploads,
                  [mesh_id](const PendingUpload& p) { return p.mesh_id == mesh_id; });

    GPUMesh& mesh = it->second;
    // The REQUESTED sizes, which is exactly what upload_mesh() added, so the
    // accounting closes even though the pool reserved a little more for
    // alignment.
    const size_t bytes = static_cast<size_t>(mesh.vertex_alloc.size)
                       + static_cast<size_t>(mesh.index_alloc.size);
    m_resident_bytes -= std::min(m_resident_bytes, bytes);

    retire_alloc(mesh.vertex_alloc, false);
    retire_alloc(mesh.index_alloc, true);

    m_meshes.erase(it);
}

void GPURenderer::release_all_meshes() {
    m_pending_uploads.clear();
    m_staging.clear();
    m_staging.shrink_to_fit();

    for (auto& [id, mesh] : m_meshes) {
        retire_alloc(mesh.vertex_alloc, false);
        retire_alloc(mesh.index_alloc, true);
    }
    m_meshes.clear();
    m_resident_bytes = 0;
}

void GPURenderer::retire_alloc(const BufferAlloc& alloc, bool is_index) {
    if (!alloc.valid()) return;

    // Not freed here. A command buffer submitted in an earlier frame may still be
    // reading these bytes, and GPUBufferPool::free() would make them immediately
    // reallocatable -- so the next flush's copy pass could overwrite geometry a
    // draw in flight is still fetching. That shows up as another mesh's triangles
    // flickering through, with nothing to attribute it to.
    m_retired_allocs.push_back(RetiredAlloc{ alloc, m_frame_index + kBufferRetireFrames, is_index });
}

void GPURenderer::drain_retired_allocs(bool force) {
    if (m_retired_allocs.empty()) return;

    size_t keep = 0;
    for (size_t i = 0; i < m_retired_allocs.size(); ++i) {
        const RetiredAlloc& r = m_retired_allocs[i];
        if (!force && m_frame_index <= r.retire_after_frame) {
            m_retired_allocs[keep++] = r;   // still possibly in flight
            continue;
        }
        if (r.is_index) {
            m_index_pool.free(r.alloc);
        } else {
            m_vertex_pool.free(r.alloc);
        }
    }
    m_retired_allocs.resize(keep);
}

// ============================================================================
// Resident memory budget
// ============================================================================

void GPURenderer::set_memory_budget(const MemoryBudget& budget) {
    m_memory_budget = budget;
    // Deliberately does not evict. A caller lowering the cap in a settings panel
    // would otherwise drop geometry from inside an ImGui widget callback, which
    // is a surprising place for the scene to change.
    m_eviction_warned = false;
}

const GPURenderer::MemoryBudget& GPURenderer::memory_budget() const {
    return m_memory_budget;
}

void GPURenderer::set_mesh_distance_fn(MeshDistanceFn fn) {
    m_mesh_distance_fn = std::move(fn);
    m_eviction_warned = false;
}

void GPURenderer::set_mesh_evicted_fn(MeshEvictedFn fn) {
    m_mesh_evicted_fn = std::move(fn);
}

size_t GPURenderer::evict_to_budget() {
    return evict_to_fit(0, 0);
}

size_t GPURenderer::evict_to_fit(size_t extra_bytes, size_t extra_meshes) {
    const auto over_budget = [&]() {
        return (m_resident_bytes + extra_bytes > m_memory_budget.max_resident_bytes)
            || (m_meshes.size() + extra_meshes > m_memory_budget.max_resident_meshes);
    };

    if (!over_budget()) {
        return 0;
    }

    if (!m_mesh_distance_fn) {
        // Once, not per frame: over budget with no distance function is a wiring
        // mistake that persists, and repeating it every frame would bury the log.
        if (!m_eviction_warned) {
            m_eviction_warned = true;
            spdlog::warn("Over the resident GPU budget ({} MB across {} meshes, cap {} MB / {}) "
                         "with no mesh distance function installed. Nothing is evicted: "
                         "evicting without a distance discards the road under the camera as "
                         "readily as one on the horizon.",
                         m_resident_bytes / (1024 * 1024), m_meshes.size(),
                         m_memory_budget.max_resident_bytes / (1024 * 1024),
                         m_memory_budget.max_resident_meshes);
        }
        return 0;
    }

    // Anything staged but not yet copied is off limits. Its bytes are still held
    // in m_staging and its ranges are the destination the next flush will copy
    // into; freeing them here would have that copy land in whatever geometry the
    // pool handed the same bytes to in the meantime.
    std::unordered_set<uint32_t> staged;
    staged.reserve(m_pending_uploads.size() * 2);
    for (const PendingUpload& p : m_pending_uploads) {
        staged.insert(p.mesh_id);
    }

    struct Candidate {
        float distance;
        uint32_t id;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(m_meshes.size());
    for (const auto& [id, mesh] : m_meshes) {
        if (staged.find(id) != staged.end()) {
            continue;
        }
        const float distance = m_mesh_distance_fn(id);
        // Negative means pinned. NaN falls through the same test and is treated as
        // pinned too, which is the safe reading of an owner that cannot answer.
        if (!(distance >= 0.0f)) {
            continue;
        }
        candidates.push_back(Candidate{ distance, id });
    }

    // Furthest first. Ties break on ascending id so the same over-budget frame
    // evicts the same meshes twice running, which is what makes a thrash
    // reproducible instead of merely intermittent.
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) {
                  if (a.distance != b.distance) return a.distance > b.distance;
                  return a.id < b.id;
              });

    size_t evicted = 0;
    for (const Candidate& c : candidates) {
        if (!over_budget()) {
            break;
        }
        // The owner is told BEFORE the id stops resolving, so it can drop its
        // handle. Iterating `candidates` rather than m_meshes is what makes the
        // release_mesh() below safe: the map is being mutated, the snapshot is
        // not.
        if (m_mesh_evicted_fn) {
            m_mesh_evicted_fn(c.id);
        }
        release_mesh(c.id);
        ++evicted;
    }

    m_evicted_meshes += evicted;

    if (over_budget() && m_frame_index >= m_next_budget_warn_frame) {
        m_next_budget_warn_frame = m_frame_index + kBudgetWarnFrameInterval;
        spdlog::warn("Still over the resident GPU budget after evicting {} mesh(es): {} MB across "
                     "{} meshes, cap {} MB / {}. The remainder is pinned or mid-upload.",
                     evicted, m_resident_bytes / (1024 * 1024), m_meshes.size(),
                     m_memory_budget.max_resident_bytes / (1024 * 1024),
                     m_memory_budget.max_resident_meshes);
    }

    return evicted;
}

GPUBufferPool::Stats GPURenderer::vertex_pool_stats() const {
    return m_vertex_pool.stats();
}

GPUBufferPool::Stats GPURenderer::index_pool_stats() const {
    return m_index_pool.stats();
}

bool GPURenderer::begin_frame() {
    if (!m_device) return false;

    // Everything staged or freed from here on is accounted against this frame.
    ++m_frame_index;

    // Close last frame's caster recording and start a new one. The shadow pass
    // that runs shortly after this replays the CLOSED list, because the recording
    // for this frame does not exist until Editor has traversed the scene, which
    // happens inside the colour pass -- after the shadow map has to be finished.
    // See render_shadow_cascades() for why replaying the visible set is preferred
    // over traversing again per cascade.
    m_shadow_casters_ready = std::move(m_shadow_casters_recording);
    m_shadow_casters_recording.clear();
    // Keep the capacity the recording had settled on, so a city-scale frame does
    // not reallocate its way up to thousands of entries every frame.
    m_shadow_casters_recording.reserve(m_shadow_casters_ready.size());

    // Drain staged mesh copies BEFORE this frame's command buffer exists, so the
    // upload is one self-contained submit rather than thousands interleaved with
    // an open render pass. Budgeted per frame, so a large import streams in.
    flush_pending_uploads();

    // Ranges freed at least kBufferRetireFrames ago cannot be referenced by any
    // command buffer still in flight, so they may go back to their pools now.
    drain_retired_allocs();

    // The texture manager's retired transfer buffers age out on the same clock
    // and for the same reason. Its kTransferRetireFrames is the same constant as
    // kBufferRetireFrames because both are bounded by the same thing: the number
    // of frames SDL_GPU can have in flight.
    if (m_texture_manager) {
        m_texture_manager->drain_retired_transfers(m_frame_index);
    }

    // Then take the resident set back under its caps. After the flush, so a mesh
    // staged last frame is a candidate rather than being skipped as mid-upload,
    // and before the scene traversal, so this frame draws only what survived.
    evict_to_budget();

    // Acquire command buffer
    m_cmd_buffer = SDL_AcquireGPUCommandBuffer(m_device);
    if (!m_cmd_buffer) {
        spdlog::error("Failed to acquire command buffer: {}", SDL_GetError());
        return false;
    }

    // Acquire swapchain texture
    uint32_t new_width, new_height;
    if (!SDL_WaitAndAcquireGPUSwapchainTexture(m_cmd_buffer, m_window,
                                                &m_swapchain_texture,
                                                &new_width, &new_height)) {
        spdlog::error("Failed to acquire swapchain texture: {}", SDL_GetError());
        SDL_CancelGPUCommandBuffer(m_cmd_buffer);
        m_cmd_buffer = nullptr;
        return false;
    }

    if (!m_swapchain_texture) {
        // Window minimized or occluded
        SDL_CancelGPUCommandBuffer(m_cmd_buffer);
        m_cmd_buffer = nullptr;
        return false;
    }

    // Handle resize
    if (new_width != m_swapchain_width || new_height != m_swapchain_height) {
        m_swapchain_width = new_width;
        m_swapchain_height = new_height;

        // Only reallocate the depth texture when the window outgrows it. A manual
        // resize drag changes the swapchain size on almost every frame, and a
        // release+create of a full-res D32 target costs ~570us each time (~3.4% of
        // a 60fps frame) -- paying that per frame is a large part of the stutter.
        // Rounding up means a drag crosses an allocation boundary rarely, and
        // shrinking never reallocates at all.
        if (m_swapchain_width > m_depth_alloc_width || m_swapchain_height > m_depth_alloc_height) {
            constexpr uint32_t GRANULARITY = 256;
            const auto round_up = [](uint32_t v) {
                return ((v + GRANULARITY - 1) / GRANULARITY) * GRANULARITY;
            };
            // Never shrink: max() against the current allocation keeps a wide-then-tall
            // drag from thrashing between two sizes.
            const uint32_t alloc_w = std::max(m_depth_alloc_width, round_up(m_swapchain_width));
            const uint32_t alloc_h = std::max(m_depth_alloc_height, round_up(m_swapchain_height));

            if (m_depth_texture) {
                SDL_ReleaseGPUTexture(m_device, m_depth_texture);
                m_depth_texture = nullptr;
            }

            SDL_GPUTextureCreateInfo depth_info{};
            depth_info.type = SDL_GPU_TEXTURETYPE_2D;
            depth_info.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
            depth_info.width = alloc_w;
            depth_info.height = alloc_h;
            depth_info.layer_count_or_depth = 1;
            depth_info.num_levels = 1;
            depth_info.sample_count = SDL_GPU_SAMPLECOUNT_1;
            depth_info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;

            m_depth_texture = SDL_CreateGPUTexture(m_device, &depth_info);
            if (!m_depth_texture) {
                spdlog::error("Failed to recreate depth texture: {}", SDL_GetError());
                m_depth_alloc_width = 0;
                m_depth_alloc_height = 0;
                SDL_CancelGPUCommandBuffer(m_cmd_buffer);
                m_cmd_buffer = nullptr;
                return false;
            }

            m_depth_alloc_width = alloc_w;
            m_depth_alloc_height = alloc_h;
            spdlog::debug("Depth target allocated {}x{}", alloc_w, alloc_h);
        }

        // MSAA textures are resolve targets, so they must track the swapchain size
        // exactly and cannot use the oversized-allocation trick above.
        if (m_sample_count != SDL_GPU_SAMPLECOUNT_1) {
            create_msaa_textures();
        }

        spdlog::debug("Resized to {}x{}", m_swapchain_width, m_swapchain_height);
    }

    return true;
}

void GPURenderer::begin_render_pass() {
    if (!m_cmd_buffer || !m_swapchain_texture) return;

    SDL_GPUColorTargetInfo color_target{};
    color_target.clear_color = {0.1f, 0.1f, 0.12f, 1.0f};  // Dark gray background
    color_target.load_op = SDL_GPU_LOADOP_CLEAR;
    color_target.cycle = false;

    // Configure for MSAA if enabled
    if (m_sample_count != SDL_GPU_SAMPLECOUNT_1 && m_msaa_color_texture) {
        color_target.texture = m_msaa_color_texture;
        color_target.resolve_texture = m_swapchain_texture;
        color_target.store_op = SDL_GPU_STOREOP_RESOLVE;
    } else {
        color_target.texture = m_swapchain_texture;
        color_target.resolve_texture = nullptr;
        color_target.store_op = SDL_GPU_STOREOP_STORE;
    }

    // Depth target for 3D rendering
    SDL_GPUDepthStencilTargetInfo depth_target{};
    depth_target.clear_depth = 0.0f;  // reverse-Z: 0 is the far plane
    depth_target.load_op = SDL_GPU_LOADOP_CLEAR;
    depth_target.store_op = SDL_GPU_STOREOP_DONT_CARE;  // Don't need to preserve depth
    depth_target.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depth_target.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth_target.cycle = false;

    // Use MSAA depth if enabled
    if (m_sample_count != SDL_GPU_SAMPLECOUNT_1 && m_msaa_depth_texture) {
        depth_target.texture = m_msaa_depth_texture;
    } else {
        depth_target.texture = m_depth_texture;
    }

    m_render_pass = SDL_BeginGPURenderPass(m_cmd_buffer, &color_target, 1, &depth_target);
    if (!m_render_pass) {
        spdlog::error("Failed to begin render pass: {}", SDL_GetError());
        return;
    }

    // A new pass inherits no bindings, so the material cache must not claim the
    // material from the previous pass is still bound.
    reset_material_binding();

    // Update scene uniforms with defaults if not set
    update_scene_uniforms();

    // Set viewport
    SDL_GPUViewport viewport{};
    viewport.x = 0;
    viewport.y = 0;
    viewport.w = static_cast<float>(m_swapchain_width);
    viewport.h = static_cast<float>(m_swapchain_height);
    viewport.min_depth = 0.0f;
    viewport.max_depth = 1.0f;
    SDL_SetGPUViewport(m_render_pass, &viewport);
}

void GPURenderer::begin_ui_render_pass() {
    if (!m_cmd_buffer || !m_swapchain_texture) return;

    // Defensively close any pass still open
    end_render_pass();

    SDL_GPUColorTargetInfo color_target{};
    // Always the resolved 1-sample swapchain image, never the MSAA color texture:
    // the 3D pass already resolved into it, and ImGui is initialized at SAMPLECOUNT_1.
    color_target.texture = m_swapchain_texture;
    color_target.mip_level = 0;
    color_target.layer_or_depth_plane = 0;
    color_target.load_op = SDL_GPU_LOADOP_LOAD;    // Preserve the 3D image drawn by the previous pass
    color_target.store_op = SDL_GPU_STOREOP_STORE;
    color_target.resolve_texture = nullptr;
    // Cycling would hand back a different backing allocation and silently discard
    // the 3D image that LOADOP_LOAD exists to preserve.
    color_target.cycle = false;

    // nullptr depth target => pDepthStencilAttachment is VK_ATTACHMENT_UNUSED,
    // which is what ImGui's pipeline was created against.
    m_render_pass = SDL_BeginGPURenderPass(m_cmd_buffer, &color_target, 1, nullptr);
    if (!m_render_pass) {
        spdlog::error("Failed to begin UI render pass: {}", SDL_GetError());
        return;
    }

    // ImGui binds its own pipeline and its own samplers into this pass. Anything
    // the material cache still believed was bound is gone.
    reset_material_binding();

    // Deliberately no SDL_SetGPUViewport here - ImGui's SetupRenderState establishes
    // its own full-framebuffer viewport and scissor inside RenderDrawData.
}

void GPURenderer::end_render_pass() {
    if (m_render_pass) {
        SDL_EndGPURenderPass(m_render_pass);
        m_render_pass = nullptr;
    }
    // Bindings die with the pass. Clearing on the way out as well as on the way
    // in means the cache is never live while no pass is open.
    reset_material_binding();
}

void GPURenderer::end_frame() {
    if (!m_cmd_buffer) return;

    // End render pass if still active
    end_render_pass();

    // Publish this frame's counters and start the next frame's at zero.
    m_frame_stats_last = m_frame_stats;
    m_frame_stats = FrameStats{};

    SDL_SubmitGPUCommandBuffer(m_cmd_buffer);
    m_cmd_buffer = nullptr;
    m_swapchain_texture = nullptr;
}

void GPURenderer::render_imgui() {
    // ImGui rendering is done by Application using the exposed render pass
    // This is a placeholder for if we want to encapsulate it later
}

SDL_GPUTextureFormat GPURenderer::get_swapchain_format() const {
    if (m_device && m_window) {
        return SDL_GetGPUSwapchainTextureFormat(m_device, m_window);
    }
    return SDL_GPU_TEXTUREFORMAT_INVALID;
}

void GPURenderer::set_view_projection(const glm::mat4& view, const glm::mat4& projection) {
    m_view = view;
    m_projection = projection;
    m_view_projection = projection * view;
}

void GPURenderer::bind_mesh_pipeline() {
    if (!m_render_pass) return;

    SDL_GPUGraphicsPipeline* pipeline = nullptr;

    if (using_pbr()) {
        // Use PBR pipeline
        pipeline = (m_current_fill_mode == FillMode::Wireframe)
            ? m_pbr_pipeline_wireframe
            : m_pbr_pipeline;
    } else {
        // Fall back to simple pipeline
        pipeline = (m_current_fill_mode == FillMode::Wireframe)
            ? m_mesh_pipeline_wireframe
            : m_mesh_pipeline;
    }

    if (pipeline) {
        SDL_BindGPUGraphicsPipeline(m_render_pass, pipeline);

        // The caller has just rebound the OPAQUE pipeline, which drops whatever
        // bind_material() had set -- including the decal pipeline, if a marking
        // was the last thing drawn. Both halves of the cache have to go.
        reset_material_binding();
        m_bound_pipeline = pipeline;

        // Push the frame-constant scene uniforms once here rather than per draw.
        // SDL_GPU uniform pushes are bound state that persists across subsequent
        // draws, so one push covers every mesh drawn with this pipeline.
        if (using_pbr()) {
            SDL_PushGPUFragmentUniformData(m_cmd_buffer, kSceneUniformSlot, &m_scene_uniforms,
                                           sizeof(m_scene_uniforms));
            // The shadow map and its block go with them, and for the same reason:
            // mesh_pbr.frag declares that sampler unconditionally, so leaving it
            // unbound after a pipeline change is a validation error rather than
            // merely a wrong-looking frame.
            bind_shadow_resources();
        }
    }
}

void GPURenderer::set_fill_mode(FillMode mode) {
    m_current_fill_mode = mode;
}

bool GPURenderer::set_shader_mode(ShaderMode mode) {
    if (mode == ShaderMode::PBR) {
        // Check if PBR pipelines are available
        if (!m_pbr_pipeline || !m_pbr_pipeline_wireframe) {
            spdlog::warn("PBR shaders not available - staying in Simple mode");
            return false;
        }
        // A texture manager is as much a precondition as the pipelines are. The
        // PBR fragment shader declares three samplers that every draw must fill,
        // and with no manager there is not one legal texture to fill them with --
        // SDL_DrawGPUIndexedPrimitives would abort on the first mesh of the first
        // frame. Refusing here is what makes Editor::init_materials()'s promise
        // true: a failed material init degrades to the pre-material look rather
        // than to a crash the user reaches through the Render Settings combo.
        if (!m_texture_manager) {
            spdlog::warn("PBR needs a texture manager for its fallback maps - staying in "
                         "Simple mode");
            return false;
        }
    }
    
    if (m_current_shader_mode != mode) {
        m_current_shader_mode = mode;
        spdlog::info("Shader mode changed to: {}", 
                     mode == ShaderMode::Simple ? "Simple" : "PBR");
    }
    return true;
}

void GPURenderer::create_msaa_textures() {
    if (m_sample_count == SDL_GPU_SAMPLECOUNT_1) return;

    // Release existing MSAA textures
    release_msaa_textures();

    SDL_GPUTextureFormat swapchain_format = SDL_GetGPUSwapchainTextureFormat(m_device, m_window);

    // Create MSAA color texture
    SDL_GPUTextureCreateInfo msaa_color_info{};
    msaa_color_info.type = SDL_GPU_TEXTURETYPE_2D;
    msaa_color_info.format = swapchain_format;
    msaa_color_info.width = m_swapchain_width;
    msaa_color_info.height = m_swapchain_height;
    msaa_color_info.layer_count_or_depth = 1;
    msaa_color_info.num_levels = 1;
    msaa_color_info.sample_count = m_sample_count;
    msaa_color_info.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;

    m_msaa_color_texture = SDL_CreateGPUTexture(m_device, &msaa_color_info);
    if (!m_msaa_color_texture) {
        spdlog::error("Failed to create MSAA color texture: {}", SDL_GetError());
        return;
    }

    // Create MSAA depth texture
    SDL_GPUTextureCreateInfo msaa_depth_info{};
    msaa_depth_info.type = SDL_GPU_TEXTURETYPE_2D;
    msaa_depth_info.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    msaa_depth_info.width = m_swapchain_width;
    msaa_depth_info.height = m_swapchain_height;
    msaa_depth_info.layer_count_or_depth = 1;
    msaa_depth_info.num_levels = 1;
    msaa_depth_info.sample_count = m_sample_count;
    msaa_depth_info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;

    m_msaa_depth_texture = SDL_CreateGPUTexture(m_device, &msaa_depth_info);
    if (!m_msaa_depth_texture) {
        spdlog::error("Failed to create MSAA depth texture: {}", SDL_GetError());
        SDL_ReleaseGPUTexture(m_device, m_msaa_color_texture);
        m_msaa_color_texture = nullptr;
        return;
    }

    spdlog::info("MSAA textures created ({}x samples)", static_cast<int>(m_sample_count));
}

void GPURenderer::release_msaa_textures() {
    if (m_msaa_color_texture) {
        SDL_ReleaseGPUTexture(m_device, m_msaa_color_texture);
        m_msaa_color_texture = nullptr;
    }
    if (m_msaa_depth_texture) {
        SDL_ReleaseGPUTexture(m_device, m_msaa_depth_texture);
        m_msaa_depth_texture = nullptr;
    }
}

bool GPURenderer::set_msaa_level(int level) {
    SDL_GPUSampleCount new_count;
    switch (level) {
        case 0: new_count = SDL_GPU_SAMPLECOUNT_1; break;
        case 1: new_count = SDL_GPU_SAMPLECOUNT_2; break;
        case 2: new_count = SDL_GPU_SAMPLECOUNT_4; break;
        case 3: new_count = SDL_GPU_SAMPLECOUNT_8; break;
        default: return false;
    }

    if (new_count == m_sample_count) return true;

    // Wait for GPU to finish before modifying resources
    SDL_WaitForGPUIdle(m_device);

    m_sample_count = new_count;

    // Recreate MSAA textures
    if (m_sample_count != SDL_GPU_SAMPLECOUNT_1) {
        create_msaa_textures();
    } else {
        release_msaa_textures();
    }

    // Recreate pipelines with new sample count
    if (m_mesh_pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_mesh_pipeline);
        m_mesh_pipeline = nullptr;
    }
    if (m_mesh_pipeline_wireframe) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_mesh_pipeline_wireframe);
        m_mesh_pipeline_wireframe = nullptr;
    }

    if (!create_pipelines()) {
        spdlog::error("Failed to recreate pipelines for MSAA level {}", level);
        return false;
    }

    // Notify callback (for ImGui reinitialization)
    if (m_msaa_changed_callback) {
        m_msaa_changed_callback(m_sample_count);
    }

    spdlog::info("MSAA level set to {}", level);
    return true;
}

int GPURenderer::get_msaa_level() const {
    switch (m_sample_count) {
        case SDL_GPU_SAMPLECOUNT_1: return 0;
        case SDL_GPU_SAMPLECOUNT_2: return 1;
        case SDL_GPU_SAMPLECOUNT_4: return 2;
        case SDL_GPU_SAMPLECOUNT_8: return 3;
        default: return 0;
    }
}

// ============================================================================
// Materials
// ============================================================================

size_t GPURenderer::texture_bytes() const {
    return m_texture_manager ? m_texture_manager->stats().bytes : 0;
}

size_t GPURenderer::texture_count() const {
    return m_texture_manager ? m_texture_manager->stats().textures : 0;
}

void GPURenderer::set_material_library(MaterialLibrary* library) {
    m_material_library = library;
    // The next bind must not be skipped on the strength of a key that was
    // resolved against the PREVIOUS library.
    reset_material_binding();
}

void GPURenderer::set_texture_manager(GPUTextureManager* textures) {
    m_texture_manager = textures;
    reset_material_binding();
}

void GPURenderer::set_materials_enabled(bool enabled) {
    if (m_materials_enabled == enabled) return;
    m_materials_enabled = enabled;
    reset_material_binding();
}

bool GPURenderer::materials_enabled() const {
    return m_materials_enabled && m_material_library != nullptr;
}

void GPURenderer::reset_material_binding() {
    m_material_bound = false;
    m_neutral_material_bound = false;
    m_bound_material = MaterialKey{};
    m_bound_pipeline = nullptr;
}

void GPURenderer::bind_neutral_material() {
    // Everything below is state SDL_GPU drops when a render pass opens or a
    // pipeline is rebound, so the cache is cleared by reset_material_binding() in
    // both cases and this cannot skip a bind the new pass needs.
    if (m_material_bound && m_neutral_material_bound) return;

    // A neutral bind is opaque by definition. If a marking left one of the decal
    // pipelines bound, give the opaque one back before drawing through it.
    SDL_GPUGraphicsPipeline* opaque = (m_current_fill_mode == FillMode::Wireframe)
                                          ? m_pbr_pipeline_wireframe
                                          : m_pbr_pipeline;
    if (opaque && opaque != m_bound_pipeline) {
        SDL_BindGPUGraphicsPipeline(m_render_pass, opaque);
        m_bound_pipeline = opaque;
        // Rebinding the pipeline drops the scene uniforms with it -- and the
        // shadow block and shadow sampler alongside them.
        SDL_PushGPUFragmentUniformData(m_cmd_buffer, kSceneUniformSlot, &m_scene_uniforms,
                                       sizeof(m_scene_uniforms));
        bind_shadow_resources();
    }

    // The header's in-class initialisers ARE the "no material" appearance: white
    // tint, dielectric, matte, unoccluded, unlit, unscaled UVs. Combined with the
    // 1x1 white albedo below, the fragment's albedo is the vertex colour alone,
    // which is what the renderer drew before materials existed.
    const MaterialUniforms neutral{};
    SDL_PushGPUFragmentUniformData(m_cmd_buffer, kMaterialUniformSlot, &neutral,
                                   sizeof(neutral));

    SDL_GPUSampler* sampler = m_texture_manager->sampler(SamplerKind::RepeatAniso);

    // bind_texture(h, h) rather than get(h): the built-ins are uploaded
    // synchronously by GPUTextureManager::init(), but if one somehow were not
    // ready this still resolves to the magenta checker rather than to nullptr,
    // and a null imageView in a COMBINED_IMAGE_SAMPLER write is exactly the
    // invalid-Vulkan case this whole function exists to prevent.
    SDL_GPUTextureSamplerBinding bindings[kMaterialSamplerCount]{};
    bindings[kAlbedoSamplerSlot].texture =
        m_texture_manager->bind_texture(m_texture_manager->white(), m_texture_manager->white());
    bindings[kAlbedoSamplerSlot].sampler = sampler;
    bindings[kNormalSamplerSlot].texture =
        m_texture_manager->bind_texture(m_texture_manager->flat_normal(),
                                        m_texture_manager->flat_normal());
    bindings[kNormalSamplerSlot].sampler = sampler;
    bindings[kOrmSamplerSlot].texture =
        m_texture_manager->bind_texture(m_texture_manager->default_orm(),
                                        m_texture_manager->default_orm());
    bindings[kOrmSamplerSlot].sampler = sampler;

    SDL_BindGPUFragmentSamplers(m_render_pass, kAlbedoSamplerSlot, bindings,
                                kMaterialSamplerCount);

    m_bound_material = MaterialKey{};
    m_material_bound = true;
    m_neutral_material_bound = true;
    ++m_frame_stats.material_binds;
}

void GPURenderer::bind_material(MaterialKey key) {
    if (!m_render_pass || !m_cmd_buffer) return;

    // The simple shader has no material uniform block and no samplers. Pushing to
    // slot 1 or binding three samplers against it is not a no-op, it is a
    // validation error, so materials are a PBR-only path by construction.
    //
    // The other two answers are NOT "return". The PBR pipeline declares three
    // fragment samplers and two fragment uniform buffers whatever the material
    // state is, and a draw that leaves them unwritten aborts inside
    // SDL_DrawGPUIndexedPrimitives. See MaterialBindMode.
    switch (material_bind_mode(m_current_shader_mode, pbr_path_available(),
                               m_materials_enabled, m_material_library != nullptr)) {
        case MaterialBindMode::Skip:
            return;
        case MaterialBindMode::Neutral:
            bind_neutral_material();
            return;
        case MaterialBindMode::Full:
            break;
    }

    // --- Redundant-bind elimination -------------------------------------------
    //
    // This is the whole reason bind_material() is cheap enough to call per submesh
    // range. A city scene issues thousands of ranges per frame; without this,
    // every one of them would push 48 bytes of uniforms and rebind three samplers
    // whether or not anything changed.
    //
    // Two things make the cache effective rather than theoretical:
    //   * Mesh::sort_submeshes_by_material() groups a mesh's ranges by material,
    //     so a mesh with eight ranges across three materials pays three binds.
    //   * The cache is not cleared between meshes, so a run of consecutive road
    //     tiles that are all asphalt pays ONE bind for the entire run.
    //
    // It is cleared whenever a render pass opens or the pipeline is rebound,
    // because SDL_GPU bindings do not survive either -- see
    // reset_material_binding(). A stale cache there would skip the bind the new
    // pass needs and draw with whatever the driver left behind.
    if (m_material_bound && !m_neutral_material_bound && m_bound_material == key) return;

    const MaterialDef& def = m_material_library->resolve(key);

    // --- Pipeline selection ----------------------------------------------------
    //
    // Blending and depth bias are PIPELINE state in SDL_GPU: they live in
    // SDL_GPUColorTargetBlendState and SDL_GPURasterizerState, both baked into
    // SDL_CreateGPUGraphicsPipeline, and there is no command to change either
    // inside a render pass. So a material that needs them is drawn through a
    // second pipeline rather than by setting a flag.
    //
    // Wireframe is deliberately exempt: in wireframe the user is inspecting
    // topology, and swapping half the draws onto a filled, blended pipeline would
    // hide the very geometry they are looking at.
    //
    // The bias MAGNITUDE is pipeline state too, not just the fact of having one,
    // so decal_pipeline_for() keeps one pipeline per distinct quantised
    // MaterialDef::depth_bias. Baking a single compile-time constant instead made
    // the panel's Depth bias slider and a set file's depth_bias field stored,
    // edited, serialised values that changed no draw state at all.
    const bool want_decal = def.needs_decal_pipeline() && m_pbr_pipeline_decal &&
                            m_current_fill_mode != FillMode::Wireframe;

    SDL_GPUGraphicsPipeline* pipeline =
        want_decal ? decal_pipeline_for(def.depth_bias)
                   : (m_current_fill_mode == FillMode::Wireframe ? m_pbr_pipeline_wireframe
                                                                 : m_pbr_pipeline);
    if (pipeline && pipeline != m_bound_pipeline) {
        SDL_BindGPUGraphicsPipeline(m_render_pass, pipeline);
        m_bound_pipeline = pipeline;

        // Rebinding the pipeline drops the scene uniforms with it, so they
        // have to be re-pushed. This is the one place a pipeline change
        // happens mid-pass, and forgetting it here is what would black out
        // every road drawn after the first marking. The shadow block and the
        // shadow sampler ride along for the same reason.
        SDL_PushGPUFragmentUniformData(m_cmd_buffer, kSceneUniformSlot, &m_scene_uniforms,
                                       sizeof(m_scene_uniforms));
        bind_shadow_resources();
    }

    // --- Uniforms --------------------------------------------------------------
    //
    // Fragment uniform slot 1 (kMaterialUniformSlot), which is set 3 binding 1 in
    // mesh_pbr.frag. Slot 0 is the frame-constant scene block, pushed in
    // bind_mesh_pipeline() and above.
    const MaterialUniforms uniforms = m_material_library->uniforms_for(key);
    SDL_PushGPUFragmentUniformData(m_cmd_buffer, kMaterialUniformSlot, &uniforms,
                                   sizeof(uniforms));

    // --- Textures --------------------------------------------------------------
    //
    // One SDL_BindGPUFragmentSamplers for all three maps, starting at first_slot
    // kAlbedoSamplerSlot (0), which is set 2 bindings 0-2 in the shader. Three
    // bindings in one call rather than three calls: the array form is why the
    // slot constants have to be contiguous.
    //
    // bind_texture() substitutes the manager's fallback for a handle that is
    // unset, invalid, or staged-but-not-yet-copied. That last case is the one
    // that matters for correctness -- a texture whose copy has not run yet holds
    // uninitialised device memory, and sampling it would be reading garbage, so a
    // material pops from white to its texture a frame or two after load rather
    // than flashing noise.
    SDL_GPUSampler* sampler = m_texture_manager->sampler(def.sampler);

    SDL_GPUTextureSamplerBinding bindings[kMaterialSamplerCount]{};
    bindings[kAlbedoSamplerSlot].texture =
        m_texture_manager->bind_texture(def.albedo, m_texture_manager->white());
    bindings[kAlbedoSamplerSlot].sampler = sampler;
    bindings[kNormalSamplerSlot].texture =
        m_texture_manager->bind_texture(def.normal, m_texture_manager->flat_normal());
    bindings[kNormalSamplerSlot].sampler = sampler;
    bindings[kOrmSamplerSlot].texture =
        m_texture_manager->bind_texture(def.orm, m_texture_manager->default_orm());
    bindings[kOrmSamplerSlot].sampler = sampler;

    SDL_BindGPUFragmentSamplers(m_render_pass, kAlbedoSamplerSlot, bindings,
                                kMaterialSamplerCount);

    m_bound_material = key;
    m_material_bound = true;
    m_neutral_material_bound = false;
    ++m_frame_stats.material_binds;
}

void GPURenderer::draw_mesh(uint32_t mesh_id, const glm::mat4& model,
                            const glm::vec4& color_tint, MaterialKey default_material) {
    if (!m_render_pass) return;

    auto it = m_meshes.find(mesh_id);
    if (it == m_meshes.end()) return;

    const GPUMesh& mesh = it->second;
    if (!mesh.is_valid()) return;
    // Its buffers exist but the copy has not run yet -- drawing now would render
    // uninitialised device memory. It appears next frame instead.
    if (!mesh.ready) return;

    // Record this draw as a shadow caster for the NEXT frame's cascade passes.
    // Recorded here, after the validity checks, so the list never contains a mesh
    // that was not worth drawing; the shadow pass re-checks anyway, because a
    // mesh can be released between the two frames.
    if (m_shadow_config.enabled && using_pbr()) {
        m_shadow_casters_recording.push_back(CapturedDraw{ mesh_id, model });
    }

    if (using_pbr()) {
        // PBR shader uniform layout: { mvp, model, normal_matrix, color_tint, camera_position }
        MeshUniformsPBR uniforms{};
        uniforms.mvp = m_view_projection * model;
        uniforms.model = model;
        uniforms.normal_matrix = compute_normal_matrix(model);
        uniforms.color_tint = color_tint;
        uniforms.camera_position = glm::vec4(m_camera_position, SDL_GetTicks() / 1000.0f);

        SDL_PushGPUVertexUniformData(m_cmd_buffer, 0, &uniforms, sizeof(uniforms));
        // Scene uniforms are frame-constant and are pushed once in
        // bind_mesh_pipeline(). Pushing them per draw cost a full SceneUniforms
        // copy on every one of the thousands of draws a city-scale scene issues.
    } else {
        // Simple shader uniform layout: { mvp, model, color_tint }
        MeshUniforms uniforms{};
        uniforms.mvp = m_view_projection * model;
        uniforms.model = model;
        uniforms.color_tint = color_tint;

        SDL_PushGPUVertexUniformData(m_cmd_buffer, 0, &uniforms, sizeof(uniforms));
    }

    // Bind the vertex RANGE. The binding offset is where this mesh's vertex 0
    // lives inside the shared block, so the mesh's indices stay zero-based and
    // mesh-local and the draw needs no vertex offset of its own.
    SDL_GPUBufferBinding vertex_binding{};
    vertex_binding.buffer = mesh.vertex_alloc.buffer;
    vertex_binding.offset = mesh.vertex_alloc.offset;
    SDL_BindGPUVertexBuffers(m_render_pass, 0, &vertex_binding, 1);

    // Draw
    if (mesh.index_alloc.valid() && mesh.index_count > 0) {
        SDL_GPUBufferBinding index_binding{};
        index_binding.buffer = mesh.index_alloc.buffer;
        // Bound at the START of the block rather than at this mesh's range: the
        // mesh base travels in first_index instead, so every mesh sharing a block
        // issues the identical bind and the backend can drop the redundant ones.
        index_binding.offset = 0;
        SDL_BindGPUIndexBuffer(m_render_pass, &index_binding, SDL_GPU_INDEXELEMENTSIZE_32BIT);

        // first_index counts INDICES, not bytes, so the range's byte offset has to
        // be divided by the index size to become one. That division is exact:
        // upload_mesh() allocates index ranges at kIndexAlignment (4), the pool
        // floors an index pool's alignment to 4 regardless of what it is asked
        // for, and an offset that somehow arrived unaligned is rejected at the
        // allocation site rather than reaching here. A truncating division here
        // would aim the draw a few bytes short of the mesh and rasterise
        // convincing nonsense, with no validation error to point at it.
        const uint32_t base_index =
            mesh.index_alloc.offset / static_cast<uint32_t>(sizeof(uint32_t));

        // One draw per material range, each preceded by its material bind. The
        // ranges tile [0, index_count), so the total primitive count is unchanged
        // and a single-range mesh issues the exact same draw as the flat call this
        // replaced -- plus at most one bind, which the cache in bind_material()
        // drops when the previous mesh already left that material bound.
        if (mesh.submeshes.empty()) {
            // A mesh with no ranges predates MaterialId entirely: terrain, water
            // and building meshes come from builders that emit no submeshes. They
            // draw as one implicit range of the caller's default material rather
            // than as MaterialId::Default grey.
            bind_material(default_material);
            SDL_DrawGPUIndexedPrimitives(m_render_pass, mesh.index_count, 1, base_index, 0, 0);
            ++m_frame_stats.draw_calls;
        } else {
            for (const auto& sub : mesh.submeshes) {
                if (sub.index_count == 0) continue;

                // A range left at the default key was never deliberately tagged,
                // so the caller's default stands in for it. A range that WAS
                // tagged always wins -- this substitution is the only place
                // default_material is consulted, and it never overrides an
                // explicit slot.
                MaterialKey key{sub.material, sub.variant};
                if (key == MaterialKey{}) key = default_material;

                bind_material(key);

                // SubMesh::index_offset is relative to the MESH; base_index is
                // where the mesh starts in the block. The sum is the absolute
                // index, and both terms are exact index counts.
                SDL_DrawGPUIndexedPrimitives(m_render_pass, sub.index_count, 1,
                                             base_index + sub.index_offset, 0, 0);
                ++m_frame_stats.draw_calls;
            }
        }
        m_frame_stats.triangles += mesh.index_count / 3;
    } else {
        // Non-indexed meshes carry no ranges, so they are one implicit range of
        // the default material, exactly as the empty-submeshes case above.
        bind_material(default_material);
        SDL_DrawGPUPrimitives(m_render_pass, mesh.vertex_count, 1, 0, 0);
        m_frame_stats.triangles += mesh.vertex_count / 3;
        ++m_frame_stats.draw_calls;
    }
}

void GPURenderer::draw_mesh_immediate(const Mesh& mesh, const glm::mat4& model) {
    // For now, just upload and draw (not cached)
    // In production, you'd want a staging buffer pool
    uint32_t id = upload_mesh(mesh);
    if (id != 0) {
        draw_mesh(id, model);
        // Note: We don't release here - caller should manage lifecycle
        // For truly immediate drawing, we'd need a different approach
    }
}

void GPURenderer::set_viewport(const SDL_GPUViewport& viewport) {
    if (m_render_pass) {
        SDL_SetGPUViewport(m_render_pass, &viewport);
    }
}

void GPURenderer::update_shadow_cascades() {
    const int count = std::clamp(m_shadow_config.cascade_count, 1, kMaxShadowCascades);
    const float map_size = static_cast<float>(std::max(m_shadow_allocated_size, 1u));

    m_shadow_uniforms = ShadowUniforms{};
    m_shadow_uniforms.shadow_params =
        glm::vec4(static_cast<float>(count), m_shadow_config.normal_offset,
                  m_shadow_config.strength, 1.0f / map_size);
    // The fade covers the last fifth of the range, so the outer cascade ends in a
    // gradient rather than in a visible circle.
    // w is the atlas tile width in UV, i.e. 1 / cascade count. The shader needs it
    // both to place a tile and to step one texel along x inside it.
    m_shadow_uniforms.shadow_fade =
        glm::vec4(m_shadow_config.max_distance * 0.8f, m_shadow_config.max_distance,
                  m_shadow_config.pcf_radius, 1.0f / static_cast<float>(count));

    // The camera frustum's half-angles, recovered from the projection matrix
    // rather than from the camera object. The camera builds a REVERSE-Z projection
    // by handing glm::perspective its far and near the wrong way round, which
    // flips signs in the third column but leaves the first two alone -- so these
    // two entries are the tangents whichever convention is in use, and this code
    // does not have to know which.
    const float tan_half_h = 1.0f / std::max(std::abs(m_projection[0][0]), 1e-6f);
    const float tan_half_v = 1.0f / std::max(std::abs(m_projection[1][1]), 1e-6f);
    const glm::mat4 inv_view = glm::inverse(m_view);

    const float near_d = 1.0f;
    const float far_d = std::max(m_shadow_config.max_distance, near_d + 1.0f);
    const float lambda = std::clamp(m_shadow_config.split_lambda, 0.0f, 1.0f);

    const glm::vec3 sun = glm::normalize(glm::vec3(m_scene_uniforms.sun_direction));

    float split_near = near_d;
    for (int i = 0; i < count; ++i) {
        // Practical split scheme. A uniform split wastes almost all of the near
        // cascade's resolution on distance; a purely logarithmic one makes the far
        // cascade cover a volume so large its texels are useless. lambda blends.
        const float p = static_cast<float>(i + 1) / static_cast<float>(count);
        const float logarithmic = near_d * std::pow(far_d / near_d, p);
        const float uniform = near_d + (far_d - near_d) * p;
        const float split_far = glm::mix(uniform, logarithmic, lambda);

        // The eight corners of this slice, in world space. View space here is
        // right-handed looking down -Z, which is what glm::lookAt produces.
        glm::vec3 corners[8];
        int c = 0;
        for (const float d : { split_near, split_far }) {
            for (const float sy : { -1.0f, 1.0f }) {
                for (const float sx : { -1.0f, 1.0f }) {
                    const glm::vec4 view_corner(sx * d * tan_half_h, sy * d * tan_half_v, -d, 1.0f);
                    corners[c++] = glm::vec3(inv_view * view_corner);
                }
            }
        }

        // A BOUNDING SPHERE, not a bounding box. This is the whole reason the
        // cascades do not shimmer: a sphere is invariant under rotation, so
        // turning the camera on the spot cannot change the fitted volume, and
        // therefore cannot change which texel any given surface point lands in. A
        // box refitted per frame changes size as the camera turns, and every
        // shadow edge in the scene crawls.
        glm::vec3 center(0.0f);
        for (const glm::vec3& corner : corners) center += corner;
        center /= 8.0f;

        float radius = 0.0f;
        for (const glm::vec3& corner : corners) {
            radius = std::max(radius, glm::length(corner - center));
        }
        // Quantised so that a sub-texel change in the fit cannot resize the
        // projection at all.
        radius = std::ceil(radius * 16.0f) / 16.0f;

        // Pull the light far enough back to clear anything that could cast into
        // the sphere. Everything between the light and the sphere is a caster.
        const float pull_back = radius + 100.0f;
        const glm::vec3 eye = center + sun * pull_back;
        const glm::vec3 up = (std::abs(sun.y) > 0.99f) ? glm::vec3(0.0f, 0.0f, 1.0f)
                                                       : glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::mat4 light_view = glm::lookAt(eye, center, up);

        const float depth_range = pull_back + radius;
        // GLM_FORCE_DEPTH_ZERO_TO_ONE is set project-wide (see CMakeLists), so this
        // is a 0..1 depth ortho, which is what the D32 target and the comparison
        // sampler expect. Forward depth, not reverse-Z: see create_shadow_pipeline().
        glm::mat4 light_proj = glm::ortho(-radius, radius, -radius, radius, 0.0f, depth_range);

        // TEXEL SNAPPING. Without it the projection slides continuously as the
        // camera moves and every shadow edge swims by up to a texel per frame, which
        // is far more visible than the aliasing it comes from. Rounding the origin
        // to whole texels makes the sampling grid move in whole-texel steps instead.
        {
            const glm::mat4 view_proj = light_proj * light_view;
            glm::vec4 origin = view_proj * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
            origin *= map_size * 0.5f;
            const glm::vec4 rounded = glm::round(origin);
            const glm::vec4 offset = (rounded - origin) * (2.0f / map_size);
            light_proj[3][0] += offset.x;
            light_proj[3][1] += offset.y;
        }

        m_shadow_uniforms.light_view_proj[i] = light_proj * light_view;
        m_shadow_uniforms.cascade_texel_world[i] = (2.0f * radius) / map_size;
        // The configured bias is in METRES, so it converts through this cascade's
        // own depth range. Expressing it in normalised units instead would make the
        // near cascade wildly over-biased relative to the far one, or the far one
        // under-biased, depending on which end it was tuned at.
        m_shadow_uniforms.cascade_depth_bias[i] =
            m_shadow_config.depth_bias_metres / std::max(depth_range, 1e-3f);

        split_near = split_far;
    }
}

void GPURenderer::render_shadow_cascades() {
    if (!m_cmd_buffer) return;
    if (!m_shadow_config.enabled || !m_shadow_pipeline || !m_shadow_texture) return;
    // Simple shader mode does not sample the map, so filling it would be pure cost.
    if (!using_pbr()) return;

    // A render pass must not already be open: these open their own.
    if (m_render_pass) return;

    update_scene_uniforms();
    update_shadow_cascades();

    const int count = std::clamp(m_shadow_config.cascade_count, 1, m_shadow_allocated_layers);
    const float tile = static_cast<float>(m_shadow_allocated_size);

    // ONE pass over the whole atlas: one depth clear instead of one per cascade,
    // and no repeated attachment transitions.
    SDL_GPUDepthStencilTargetInfo depth_target{};
    depth_target.texture = m_shadow_texture;
    depth_target.clear_depth = 1.0f;   // forward depth: 1 is the far plane
    depth_target.load_op = SDL_GPU_LOADOP_CLEAR;
    depth_target.store_op = SDL_GPU_STOREOP_STORE;   // the whole point
    depth_target.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depth_target.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth_target.cycle = false;

    // No colour targets at all, matching the pipeline.
    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(m_cmd_buffer, nullptr, 0, &depth_target);
    if (!pass) {
        spdlog::error("Failed to begin shadow pass: {}", SDL_GetError());
        return;
    }
    SDL_BindGPUGraphicsPipeline(pass, m_shadow_pipeline);

    for (int cascade = 0; cascade < count; ++cascade) {
        // The viewport maps this cascade's clip space onto its own tile. A scissor
        // matching it is belt and braces: the projection already confines the
        // geometry to the viewport rect, but a viewport does not clip in Vulkan
        // and a scissor does, so one cascade cannot write into another's tile.
        SDL_GPUViewport viewport{};
        viewport.x = static_cast<float>(cascade) * tile;
        viewport.y = 0.0f;
        viewport.w = tile;
        viewport.h = tile;
        viewport.min_depth = 0.0f;
        viewport.max_depth = 1.0f;
        SDL_SetGPUViewport(pass, &viewport);

        SDL_Rect scissor{};
        scissor.x = cascade * static_cast<int>(m_shadow_allocated_size);
        scissor.y = 0;
        scissor.w = static_cast<int>(m_shadow_allocated_size);
        scissor.h = static_cast<int>(m_shadow_allocated_size);
        SDL_SetGPUScissor(pass, &scissor);

        const glm::mat4& light_view_proj = m_shadow_uniforms.light_view_proj[cascade];

        for (const CapturedDraw& draw : m_shadow_casters_ready) {
            auto it = m_meshes.find(draw.mesh_id);
            if (it == m_meshes.end()) continue;   // released since it was recorded
            const GPUMesh& mesh = it->second;
            if (!mesh.is_valid() || !mesh.ready) continue;

            ShadowMeshUniforms uniforms{};
            uniforms.light_mvp = light_view_proj * draw.model;
            SDL_PushGPUVertexUniformData(m_cmd_buffer, 0, &uniforms, sizeof(uniforms));

            SDL_GPUBufferBinding vertex_binding{};
            vertex_binding.buffer = mesh.vertex_alloc.buffer;
            vertex_binding.offset = mesh.vertex_alloc.offset;
            SDL_BindGPUVertexBuffers(pass, 0, &vertex_binding, 1);

            SDL_GPUBufferBinding index_binding{};
            index_binding.buffer = mesh.index_alloc.buffer;
            index_binding.offset = mesh.index_alloc.offset;
            SDL_BindGPUIndexBuffer(pass, &index_binding, SDL_GPU_INDEXELEMENTSIZE_32BIT);

            // The WHOLE index range in one draw, ignoring the submesh split. The
            // submeshes exist to change material between ranges, and the depth pass
            // has no material -- so walking them here would issue several draws
            // that differ in nothing.
            SDL_DrawGPUIndexedPrimitives(pass, mesh.index_count, 1, 0, 0, 0);
        }
    }

    SDL_EndGPURenderPass(pass);
}

void GPURenderer::bind_shadow_resources() {
    if (!m_render_pass || !m_cmd_buffer) return;
    if (!using_pbr()) return;

    // mesh_pbr.frag DECLARES the shadow sampler unconditionally, so it has to be
    // bound whether or not shadows are on -- an unbound declared sampler is a
    // validation error and, on some backends, garbage reads. When shadows are off
    // the uniform block's cascade count is zero and the shader never samples it.
    if (m_shadow_texture && m_shadow_sampler) {
        SDL_GPUTextureSamplerBinding binding{};
        binding.texture = m_shadow_texture;
        binding.sampler = m_shadow_sampler;
        SDL_BindGPUFragmentSamplers(m_render_pass, kShadowSamplerSlot, &binding, 1);
    }

    // If no cascade was rendered this frame, publish a zeroed block: cascade count
    // 0 means "fully lit" rather than "compare against whatever is in the texture".
    if (m_shadow_config.enabled && m_shadow_pipeline && m_shadow_texture) {
        SDL_PushGPUFragmentUniformData(m_cmd_buffer, kShadowUniformSlot, &m_shadow_uniforms,
                                       sizeof(m_shadow_uniforms));
    } else {
        const ShadowUniforms off{};
        SDL_PushGPUFragmentUniformData(m_cmd_buffer, kShadowUniformSlot, &off, sizeof(off));
    }
}

void GPURenderer::draw_sky() {
    if (!m_render_pass || !m_cmd_buffer || !m_sky_pipeline) return;

    // Simple mode has no sky. mesh.frag reads no scene uniforms at all, so a sky
    // drawn behind it would be a background that lights nothing -- exactly the
    // mismatch this pass exists to remove.
    if (!using_pbr()) return;

    // The seeding that begin_render_pass() normally does happens before any draw;
    // calling it again here is harmless and makes draw_sky() correct even if a
    // caller ever draws the sky as the first thing in a pass it opened itself.
    update_scene_uniforms();

    SDL_BindGPUGraphicsPipeline(m_render_pass, m_sky_pipeline);

    SkyUniforms sky{};
    // The pixel-to-ray reconstruction in sky.vert needs the INVERSE of exactly
    // the matrix the geometry is drawn with, or the horizon sits at a different
    // angle from the ground plane meeting it.
    sky.inv_view_projection = glm::inverse(m_view_projection);
    sky.camera_position = glm::vec4(m_camera_position, 0.0f);

    SDL_PushGPUVertexUniformData(m_cmd_buffer, 0, &sky, sizeof(sky));
    SDL_PushGPUFragmentUniformData(m_cmd_buffer, kSceneUniformSlot, &m_scene_uniforms,
                                   sizeof(m_scene_uniforms));

    // Three vertices, no vertex buffer, no index buffer. sky.vert builds the
    // covering triangle from gl_VertexIndex.
    SDL_DrawGPUPrimitives(m_render_pass, 3, 1, 0, 0);

    // This pass now has the sky pipeline bound and no material state. Both caches
    // have to be told, or the next bind_material() will skip a bind it needs.
    m_bound_pipeline = m_sky_pipeline;
    reset_material_binding();
}

void GPURenderer::set_sky(const glm::vec3& zenith, const glm::vec3& horizon,
                          const glm::vec3& ground, float sky_intensity,
                          float ground_intensity, float falloff) {
    m_scene_uniforms.sky_zenith = glm::vec4(zenith, sky_intensity);
    m_scene_uniforms.sky_horizon = glm::vec4(horizon, falloff);
    m_scene_uniforms.ground_color = glm::vec4(ground, ground_intensity);
}

void GPURenderer::set_ibl_params(float specular_scale, float sun_angular_deg,
                                 float aerial_perspective, float sun_glow_exponent) {
    // The shader wants cos(HALF the angular diameter) so that a dot product
    // against the sun direction can be compared against it directly.
    const float half_angle = glm::radians(sun_angular_deg) * 0.5f;
    m_scene_uniforms.ibl_params = glm::vec4(specular_scale, std::cos(half_angle),
                                            aerial_perspective, sun_glow_exponent);
}

void GPURenderer::set_camera_position(const glm::vec3& position) {
    m_camera_position = position;
    m_scene_uniforms.camera_position = glm::vec4(position, m_scene_uniforms.camera_position.w);
}

void GPURenderer::set_scene_lighting(const glm::vec3& sun_dir, const glm::vec3& sun_color, 
                                     float sun_intensity, float ambient_intensity) {
    m_scene_uniforms.sun_direction = glm::vec4(glm::normalize(sun_dir), sun_intensity);
    m_scene_uniforms.sun_color = glm::vec4(sun_color, ambient_intensity);
}

void GPURenderer::set_fog(int mode, const glm::vec3& color, float start, float end, float density) {
    m_scene_uniforms.fog_params = glm::vec4(start, end, density, static_cast<float>(mode));
    m_scene_uniforms.fog_color = glm::vec4(color, 1.0f);
}

glm::mat4 GPURenderer::compute_normal_matrix(const glm::mat4& model) {
    // For correct normal transformation with non-uniform scaling,
    // we need the inverse-transpose of the upper-left 3x3 of the model matrix
    // Padded to mat4 for GPU alignment
    glm::mat3 normal_mat3 = glm::transpose(glm::inverse(glm::mat3(model)));
    return glm::mat4(normal_mat3);
}

void GPURenderer::update_scene_uniforms() {
    // Called at the start of each frame to ensure scene uniforms are current
    // The actual push happens in draw_mesh, but this ensures defaults are set
    
    // Seeded ONCE, on an explicit flag rather than on "does the field still look
    // unset". The old guards were `sun_direction.w <= 0.0f` and
    // `camera_position.w <= 0.0f`, which read the intensity and the exposure as
    // their own not-yet-initialised markers: dragging the Sun Intensity slider
    // to 0 -- a legitimate value, and the only way to preview the ambient term
    // alone -- made this function overwrite the whole light block with the
    // defaults again on the very next frame, so the slider snapped back.
    if (!m_scene_lighting_seeded) {
        m_scene_lighting_seeded = true;

        // Matches the Render Settings sliders' own initial positions (azimuth
        // 45 degrees, height 60 degrees) so the panel does not disagree with
        // what is on screen before anything is touched.
        const float az_rad = glm::radians(45.0f);
        const float h_rad  = glm::radians(60.0f);
        m_scene_uniforms.sun_direction = glm::vec4(
            glm::normalize(glm::vec3(std::cos(h_rad) * std::sin(az_rad),
                                     std::sin(h_rad),
                                     std::cos(h_rad) * std::cos(az_rad))),
            // Radiance, not a 0..1 factor. The shader's diffuse term is
            // albedo / PI, so a unit-intensity sun lights a white surface facing
            // it to 1/PI ~= 0.32 -- the flat, grey, underexposed look this scene
            // had. ~PI puts that surface back at 1.0 before the tone curve.
            3.14159265f);
        // sun_color.a is the ambient MASTER SCALE now, not the ambient light
        // itself. The light is the sky below, integrated over the hemisphere in
        // sky_common.glsl, so 1.0 means "the sky as authored" rather than the old
        // 0.3 meaning "30% grey everywhere".
        m_scene_uniforms.sun_color = glm::vec4(1.0f, 0.98f, 0.95f, 1.0f);
        m_scene_uniforms.camera_position.w = 1.0f;                         // Exposure

        // A clear midday sky, in scene-referred radiance to match the sun above.
        // Blue-heavy at the zenith, pale and bright at the horizon; the ground
        // bounce is a desaturated mid-green because most of what this renderer
        // draws stands on grass or asphalt.
        set_sky(/*zenith*/  glm::vec3(0.16f, 0.30f, 0.62f),
                /*horizon*/ glm::vec3(0.56f, 0.68f, 0.86f),
                /*ground*/  glm::vec3(0.14f, 0.14f, 0.12f),
                /*sky_intensity*/ 1.0f,
                /*ground_intensity*/ 0.7f,
                /*falloff*/ 0.45f);

        // 0.53 degrees is the real sun's angular diameter. Aerial perspective
        // full on: with a sky to fade into, there is no reason to fade into an
        // arbitrary colour instead.
        set_ibl_params(/*specular_scale*/ 1.0f, /*sun_angular_deg*/ 0.53f,
                       /*aerial_perspective*/ 1.0f, /*sun_glow_exponent*/ 64.0f);

        // Exponential distance haze, on by default. Off, the far edge of a city
        // extract keeps full contrast right up to the horizon and then stops --
        // the single strongest cue that a scene has no atmosphere. The density is
        // scaled for kilometres because that is the extent of an OSM extract.
        m_scene_uniforms.fog_params = glm::vec4(50.0f, 4000.0f, 0.00035f, 2.0f);
        m_scene_uniforms.fog_color = glm::vec4(0.62f, 0.72f, 0.85f, 1.0f);
    }
    
    // SceneUniforms::pbr_params is RESERVED and unread; mesh_pbr.frag takes
    // metallic, roughness and ao from the per-material block. It is zeroed rather
    // than seeded with plausible-looking defaults, so that anything that starts
    // reading it reads an obvious zero instead of a value that looks authored.
    m_scene_uniforms.pbr_params = glm::vec4(0.0f);
}

} // namespace stratum
