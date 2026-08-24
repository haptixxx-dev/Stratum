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
constexpr uint32_t kVertexAlignment = static_cast<uint32_t>(sizeof(Vertex));

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
                               SDL_GPUVertexAttribute (&attrs)[5],
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

    state = {};
    state.vertex_buffer_descriptions = &buffer_desc;
    state.num_vertex_buffers = 1;
    state.vertex_attributes = attrs;
    state.num_vertex_attributes = 5;
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
    m_pbr_fragment_shader =
        load_shader(frag_path.c_str(), SDL_GPU_SHADERSTAGE_FRAGMENT,
                    static_cast<int>(kPbrFragmentUniformBufferCount), 0,
                    static_cast<int>(kMaterialSamplerCount));
    if (!m_pbr_fragment_shader) {
        spdlog::error("Failed to load PBR fragment shader: {}", frag_path);
        SDL_ReleaseGPUShader(m_device, m_pbr_vertex_shader);
        m_pbr_vertex_shader = nullptr;
        return false;
    }

    spdlog::info("PBR shaders loaded successfully");
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
    // PBR shader uses 5 vertex attributes (including tangent)
    SDL_GPUVertexBufferDescription vertex_buffer_desc{};
    SDL_GPUVertexAttribute vertex_attributes[5]{};
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
    SDL_GPUVertexAttribute vertex_attributes[5]{};
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
        // Rebinding the pipeline drops the scene uniforms with it.
        SDL_PushGPUFragmentUniformData(m_cmd_buffer, kSceneUniformSlot, &m_scene_uniforms,
                                       sizeof(m_scene_uniforms));
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
        // every road drawn after the first marking.
        SDL_PushGPUFragmentUniformData(m_cmd_buffer, kSceneUniformSlot, &m_scene_uniforms,
                                       sizeof(m_scene_uniforms));
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
    
    // Default sun lighting
    if (m_scene_uniforms.sun_direction.w <= 0.0f) {
        m_scene_uniforms.sun_direction = glm::vec4(glm::normalize(glm::vec3(0.5f, 1.0f, 0.3f)), 1.0f);
        m_scene_uniforms.sun_color = glm::vec4(1.0f, 0.98f, 0.95f, 0.3f);  // Warm white, 0.3 ambient
    }
    
    // Default exposure
    if (m_scene_uniforms.camera_position.w <= 0.0f) {
        m_scene_uniforms.camera_position.w = 1.0f;
    }
    
    // SceneUniforms::pbr_params is RESERVED and unread; mesh_pbr.frag takes
    // metallic, roughness and ao from the per-material block. It is zeroed rather
    // than seeded with plausible-looking defaults, so that anything that starts
    // reading it reads an obvious zero instead of a value that looks authored.
    m_scene_uniforms.pbr_params = glm::vec4(0.0f);
}

} // namespace stratum
