/**
 * @file gpu_renderer.cpp
 * @brief SDL_GPU renderer implementation
 */

#include "renderer/gpu_renderer.hpp"
#include "renderer/mesh.hpp"
#include <spdlog/spdlog.h>
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

    // PBR fragment shader: 1 uniform buffer (SceneUniforms at set=2)
    m_pbr_fragment_shader = load_shader(frag_path.c_str(), SDL_GPU_SHADERSTAGE_FRAGMENT, 1, 0);
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
                                         int num_uniform_buffers, int num_storage_buffers) {
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

    // Use provided resource counts
    shader_info.num_uniform_buffers = num_uniform_buffers;
    shader_info.num_storage_buffers = num_storage_buffers;
    shader_info.num_storage_textures = 0;
    shader_info.num_samplers = 0;

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
    vertex_buffer_desc.slot = 0;
    vertex_buffer_desc.pitch = sizeof(Vertex);
    vertex_buffer_desc.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    vertex_buffer_desc.instance_step_rate = 0;

    SDL_GPUVertexAttribute vertex_attributes[5]{};

    // Position: vec3
    vertex_attributes[0].location = 0;
    vertex_attributes[0].buffer_slot = 0;
    vertex_attributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    vertex_attributes[0].offset = offsetof(Vertex, position);

    // Normal: vec3
    vertex_attributes[1].location = 1;
    vertex_attributes[1].buffer_slot = 0;
    vertex_attributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    vertex_attributes[1].offset = offsetof(Vertex, normal);

    // UV: vec2
    vertex_attributes[2].location = 2;
    vertex_attributes[2].buffer_slot = 0;
    vertex_attributes[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
    vertex_attributes[2].offset = offsetof(Vertex, uv);

    // Color: vec4
    vertex_attributes[3].location = 3;
    vertex_attributes[3].buffer_slot = 0;
    vertex_attributes[3].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4;
    vertex_attributes[3].offset = offsetof(Vertex, color);

    // Tangent: vec4 (xyz = tangent, w = bitangent sign)
    vertex_attributes[4].location = 4;
    vertex_attributes[4].buffer_slot = 0;
    vertex_attributes[4].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4;
    vertex_attributes[4].offset = offsetof(Vertex, tangent);

    SDL_GPUVertexInputState vertex_input{};
    vertex_input.vertex_buffer_descriptions = &vertex_buffer_desc;
    vertex_input.num_vertex_buffers = 1;
    vertex_input.vertex_attributes = vertex_attributes;
    vertex_input.num_vertex_attributes = 5;

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

    spdlog::info("PBR graphics pipelines created (solid + wireframe)");
    return true;
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
    if (!m_device || m_pending_uploads.empty()) return;

    // Admit as many staged meshes as this frame's budget allows; the rest stay
    // queued for later frames, which is what turns a stampeding import into a
    // stream. The batch also decides where each entry lands in the transfer
    // buffer, because the staging arena it came from is not packed -- see
    // plan_upload_batch().
    const UploadBatch batch = plan_upload_batch(m_pending_uploads, kMaxUploadBytesPerFrame);
    const size_t batch_count = batch.offsets.size();
    if (batch_count == 0) return;

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

    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
    if (!cmd) {
        spdlog::error("Failed to acquire upload command buffer: {}", SDL_GetError());
        ++m_upload_failures;
        return;
    }

    SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);
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
    if (!SDL_SubmitGPUCommandBuffer(cmd)) {
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

    // Deliberately no SDL_SetGPUViewport here - ImGui's SetupRenderState establishes
    // its own full-framebuffer viewport and scissor inside RenderDrawData.
}

void GPURenderer::end_render_pass() {
    if (m_render_pass) {
        SDL_EndGPURenderPass(m_render_pass);
        m_render_pass = nullptr;
    }
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

    if (m_current_shader_mode == ShaderMode::PBR && m_pbr_pipeline) {
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

        // Push the frame-constant scene uniforms once here rather than per draw.
        // SDL_GPU uniform pushes are bound state that persists across subsequent
        // draws, so one push covers every mesh drawn with this pipeline.
        if (m_current_shader_mode == ShaderMode::PBR && m_pbr_pipeline) {
            SDL_PushGPUFragmentUniformData(m_cmd_buffer, 0, &m_scene_uniforms,
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
    }
    
    if (m_current_shader_mode != mode) {
        m_current_shader_mode = mode;
        spdlog::info("Shader mode changed to: {}", 
                     mode == ShaderMode::Simple ? "Simple" : "PBR");
    }
    return true;
}

void GPURenderer::set_pbr_params(float metallic, float roughness, float ao) {
    m_scene_uniforms.pbr_params = glm::vec4(metallic, roughness, ao, 0.0f);
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

void GPURenderer::draw_mesh(uint32_t mesh_id, const glm::mat4& model, 
                            const glm::vec4& color_tint, uint32_t material_id) {
    if (!m_render_pass) return;

    auto it = m_meshes.find(mesh_id);
    if (it == m_meshes.end()) return;

    const GPUMesh& mesh = it->second;
    if (!mesh.is_valid()) return;
    // Its buffers exist but the copy has not run yet -- drawing now would render
    // uninitialised device memory. It appears next frame instead.
    if (!mesh.ready) return;

    if (m_current_shader_mode == ShaderMode::PBR && m_pbr_pipeline) {
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

        // One draw per material range. The ranges tile [0, index_count), so the
        // total primitive count is unchanged and a single-range mesh issues the
        // exact same call as the flat draw this replaced.
        //
        // NOTE: no per-material pipeline or texture binding yet. Every range is
        // drawn with the pipeline and bindings already set by the caller; the
        // material slot is carried only so a later phase can bind per range.
        if (mesh.submeshes.empty()) {
            SDL_DrawGPUIndexedPrimitives(m_render_pass, mesh.index_count, 1, base_index, 0, 0);
            ++m_frame_stats.draw_calls;
        } else {
            for (const auto& sub : mesh.submeshes) {
                if (sub.index_count == 0) continue;
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
    
    // Default PBR params (metallic=0, roughness=0.5, ao=1.0)
    // Only initialize if roughness is 0 (uninitialized) since 0 roughness = mirror = black
    if (m_scene_uniforms.pbr_params.y <= 0.0f) {
        m_scene_uniforms.pbr_params = glm::vec4(0.0f, 0.5f, 1.0f, 0.0f);
    }
}

} // namespace stratum
