/**
 * @file gpu_renderer.cpp
 * @brief SDL_GPU renderer implementation
 */

#include "renderer/gpu_renderer.hpp"
#include "renderer/mesh.hpp"
#include <spdlog/spdlog.h>
#include <fstream>
#include <filesystem>

namespace stratum {

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

    // Release transfer buffer
    if (m_transfer_buffer) {
        SDL_ReleaseGPUTransferBuffer(m_device, m_transfer_buffer);
        m_transfer_buffer = nullptr;
    }

    // Release pipeline
    if (m_mesh_pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(m_device, m_mesh_pipeline);
        m_mesh_pipeline = nullptr;
    }

    // Release shaders
    if (m_vertex_shader) {
        SDL_ReleaseGPUShader(m_device, m_vertex_shader);
        m_vertex_shader = nullptr;
    }
    if (m_fragment_shader) {
        SDL_ReleaseGPUShader(m_device, m_fragment_shader);
        m_fragment_shader = nullptr;
    }

    // Release depth texture
    if (m_depth_texture) {
        SDL_ReleaseGPUTexture(m_device, m_depth_texture);
        m_depth_texture = nullptr;
    }

    // Release window claim and destroy device
    if (m_window) {
        SDL_ReleaseWindowFromGPUDevice(m_device, m_window);
    }
    SDL_DestroyGPUDevice(m_device);
    m_device = nullptr;
    m_window = nullptr;

    spdlog::info("GPURenderer shutdown");
}

bool GPURenderer::load_shaders() {
    // Get base path for asset loading
    const char* base = SDL_GetBasePath();
    std::string base_path = base ? base : "";

    // Shader paths - look in assets/shaders/ relative to executable
    // The executable is in build/bin/, so assets are at ../../assets/
    std::string vert_path = base_path + "../../assets/shaders/mesh.vert.spv";
    std::string frag_path = base_path + "../../assets/shaders/mesh.frag.spv";

    m_vertex_shader = load_shader(vert_path.c_str(), SDL_GPU_SHADERSTAGE_VERTEX);
    if (!m_vertex_shader) {
        spdlog::error("Failed to load vertex shader: {}", vert_path);
        return false;
    }

    m_fragment_shader = load_shader(frag_path.c_str(), SDL_GPU_SHADERSTAGE_FRAGMENT);
    if (!m_fragment_shader) {
        spdlog::error("Failed to load fragment shader: {}", frag_path);
        return false;
    }

    spdlog::info("Shaders loaded successfully");
    return true;
}

SDL_GPUShader* GPURenderer::load_shader(const char* path, SDL_GPUShaderStage stage) {
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

    // Set resource counts based on our shader design
    if (stage == SDL_GPU_SHADERSTAGE_VERTEX) {
        shader_info.num_uniform_buffers = 1;  // MeshUniforms
        shader_info.num_storage_buffers = 0;
        shader_info.num_storage_textures = 0;
        shader_info.num_samplers = 0;
    } else {
        shader_info.num_uniform_buffers = 0;
        shader_info.num_storage_buffers = 0;
        shader_info.num_storage_textures = 0;
        shader_info.num_samplers = 0;
    }

    SDL_GPUShader* shader = SDL_CreateGPUShader(m_device, &shader_info);
    if (!shader) {
        spdlog::error("Failed to create shader from {}: {}", path, SDL_GetError());
        return nullptr;
    }

    return shader;
}

bool GPURenderer::create_pipelines() {
    // Vertex input layout matching our Vertex struct
    SDL_GPUVertexBufferDescription vertex_buffer_desc{};
    vertex_buffer_desc.slot = 0;
    vertex_buffer_desc.pitch = sizeof(Vertex);
    vertex_buffer_desc.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    vertex_buffer_desc.instance_step_rate = 0;

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
    depth_stencil.compare_op = SDL_GPU_COMPAREOP_LESS;
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

    // Multisample state (no MSAA for now)
    SDL_GPUMultisampleState multisample{};
    multisample.sample_count = SDL_GPU_SAMPLECOUNT_1;
    multisample.sample_mask = 0;  // Must be 0 when not using multisampling
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

    spdlog::info("Graphics pipeline created");
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

    GPUMesh gpu_mesh{};;
    gpu_mesh.vertex_count = static_cast<uint32_t>(mesh.vertices.size());
    gpu_mesh.index_count = static_cast<uint32_t>(mesh.indices.size());

    size_t vertex_size = mesh.vertices.size() * sizeof(Vertex);
    size_t index_size = mesh.indices.size() * sizeof(uint32_t);
    size_t total_size = vertex_size + index_size;

    // Create or resize transfer buffer if needed
    if (!m_transfer_buffer || m_transfer_buffer_size < total_size) {
        if (m_transfer_buffer) {
            SDL_ReleaseGPUTransferBuffer(m_device, m_transfer_buffer);
        }

        SDL_GPUTransferBufferCreateInfo transfer_info{};
        transfer_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        transfer_info.size = static_cast<uint32_t>(total_size);

        m_transfer_buffer = SDL_CreateGPUTransferBuffer(m_device, &transfer_info);
        m_transfer_buffer_size = total_size;

        if (!m_transfer_buffer) {
            spdlog::error("Failed to create transfer buffer: {}", SDL_GetError());
            return 0;
        }
    }

    // Map transfer buffer and copy data
    void* mapped = SDL_MapGPUTransferBuffer(m_device, m_transfer_buffer, false);
    if (!mapped) {
        spdlog::error("Failed to map transfer buffer: {}", SDL_GetError());
        return 0;
    }

    memcpy(mapped, mesh.vertices.data(), vertex_size);
    if (index_size > 0) {
        memcpy(static_cast<uint8_t*>(mapped) + vertex_size, mesh.indices.data(), index_size);
    }

    SDL_UnmapGPUTransferBuffer(m_device, m_transfer_buffer);

    // Create GPU buffers
    SDL_GPUBufferCreateInfo vertex_buffer_info{};
    vertex_buffer_info.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
    vertex_buffer_info.size = static_cast<uint32_t>(vertex_size);

    gpu_mesh.vertex_buffer = SDL_CreateGPUBuffer(m_device, &vertex_buffer_info);
    if (!gpu_mesh.vertex_buffer) {
        spdlog::error("Failed to create vertex buffer: {}", SDL_GetError());
        return 0;
    }

    if (index_size > 0) {
        SDL_GPUBufferCreateInfo index_buffer_info{};
        index_buffer_info.usage = SDL_GPU_BUFFERUSAGE_INDEX;
        index_buffer_info.size = static_cast<uint32_t>(index_size);

        gpu_mesh.index_buffer = SDL_CreateGPUBuffer(m_device, &index_buffer_info);
        if (!gpu_mesh.index_buffer) {
            spdlog::error("Failed to create index buffer: {}", SDL_GetError());
            SDL_ReleaseGPUBuffer(m_device, gpu_mesh.vertex_buffer);
            return 0;
        }
    }

    // Upload via copy pass
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
    if (!cmd) {
        spdlog::error("Failed to acquire command buffer for upload: {}", SDL_GetError());
        return 0;
    }

    SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);

    // Upload vertices
    SDL_GPUTransferBufferLocation src_vertex{};
    src_vertex.transfer_buffer = m_transfer_buffer;
    src_vertex.offset = 0;

    SDL_GPUBufferRegion dst_vertex{};
    dst_vertex.buffer = gpu_mesh.vertex_buffer;
    dst_vertex.offset = 0;
    dst_vertex.size = static_cast<uint32_t>(vertex_size);

    SDL_UploadToGPUBuffer(copy_pass, &src_vertex, &dst_vertex, false);

    // Upload indices
    if (index_size > 0) {
        SDL_GPUTransferBufferLocation src_index{};
        src_index.transfer_buffer = m_transfer_buffer;
        src_index.offset = static_cast<uint32_t>(vertex_size);

        SDL_GPUBufferRegion dst_index{};
        dst_index.buffer = gpu_mesh.index_buffer;
        dst_index.offset = 0;
        dst_index.size = static_cast<uint32_t>(index_size);

        SDL_UploadToGPUBuffer(copy_pass, &src_index, &dst_index, false);
    }

    SDL_EndGPUCopyPass(copy_pass);
    SDL_SubmitGPUCommandBuffer(cmd);
    
    // Wait for the upload to complete before reusing the transfer buffer
    // This ensures the GPU has finished reading from the transfer buffer
    SDL_WaitForGPUIdle(m_device);

    // Store and return ID
    uint32_t mesh_id = m_next_mesh_id++;
    m_meshes[mesh_id] = gpu_mesh;

    return mesh_id;
}

void GPURenderer::release_mesh(uint32_t mesh_id) {
    auto it = m_meshes.find(mesh_id);
    if (it == m_meshes.end()) return;

    GPUMesh& mesh = it->second;
    if (mesh.vertex_buffer) {
        SDL_ReleaseGPUBuffer(m_device, mesh.vertex_buffer);
    }
    if (mesh.index_buffer) {
        SDL_ReleaseGPUBuffer(m_device, mesh.index_buffer);
    }

    m_meshes.erase(it);
}

void GPURenderer::release_all_meshes() {
    for (auto& [id, mesh] : m_meshes) {
        if (mesh.vertex_buffer) {
            SDL_ReleaseGPUBuffer(m_device, mesh.vertex_buffer);
        }
        if (mesh.index_buffer) {
            SDL_ReleaseGPUBuffer(m_device, mesh.index_buffer);
        }
    }
    m_meshes.clear();
}

bool GPURenderer::begin_frame() {
    if (!m_device) return false;

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

    // Handle resize - recreate depth texture if size changed
    if (new_width != m_swapchain_width || new_height != m_swapchain_height) {
        m_swapchain_width = new_width;
        m_swapchain_height = new_height;

        // Release old depth texture
        if (m_depth_texture) {
            SDL_ReleaseGPUTexture(m_device, m_depth_texture);
            m_depth_texture = nullptr;
        }

        // Create new depth texture
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
            spdlog::error("Failed to recreate depth texture: {}", SDL_GetError());
            SDL_CancelGPUCommandBuffer(m_cmd_buffer);
            m_cmd_buffer = nullptr;
            return false;
        }

        spdlog::debug("Resized to {}x{}", m_swapchain_width, m_swapchain_height);
    }

    return true;
}

void GPURenderer::begin_render_pass() {
    if (!m_cmd_buffer || !m_swapchain_texture) return;

    SDL_GPUColorTargetInfo color_target{};
    color_target.texture = m_swapchain_texture;
    color_target.clear_color = {0.1f, 0.1f, 0.12f, 1.0f};  // Dark gray background
    color_target.load_op = SDL_GPU_LOADOP_CLEAR;
    color_target.store_op = SDL_GPU_STOREOP_STORE;
    color_target.cycle = false;

    // Depth target for 3D rendering
    SDL_GPUDepthStencilTargetInfo depth_target{};
    depth_target.texture = m_depth_texture;
    depth_target.clear_depth = 1.0f;
    depth_target.load_op = SDL_GPU_LOADOP_CLEAR;
    depth_target.store_op = SDL_GPU_STOREOP_DONT_CARE;  // Don't need to preserve depth
    depth_target.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depth_target.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth_target.cycle = false;

    m_render_pass = SDL_BeginGPURenderPass(m_cmd_buffer, &color_target, 1, &depth_target);
    if (!m_render_pass) {
        spdlog::error("Failed to begin render pass: {}", SDL_GetError());
        return;
    }

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
    if (m_render_pass && m_mesh_pipeline) {
        SDL_BindGPUGraphicsPipeline(m_render_pass, m_mesh_pipeline);
    }
}

void GPURenderer::draw_mesh(uint32_t mesh_id, const glm::mat4& model, const glm::vec4& color_tint) {
    if (!m_render_pass) return;

    auto it = m_meshes.find(mesh_id);
    if (it == m_meshes.end()) return;

    const GPUMesh& mesh = it->second;
    if (!mesh.is_valid()) return;

    // Push uniforms
    MeshUniforms uniforms{};
    uniforms.mvp = m_view_projection * model;
    uniforms.model = model;
    uniforms.color_tint = color_tint;

    SDL_PushGPUVertexUniformData(m_cmd_buffer, 0, &uniforms, sizeof(uniforms));

    // Bind vertex buffer
    SDL_GPUBufferBinding vertex_binding{};
    vertex_binding.buffer = mesh.vertex_buffer;
    vertex_binding.offset = 0;
    SDL_BindGPUVertexBuffers(m_render_pass, 0, &vertex_binding, 1);

    // Draw
    if (mesh.index_buffer && mesh.index_count > 0) {
        SDL_GPUBufferBinding index_binding{};
        index_binding.buffer = mesh.index_buffer;
        index_binding.offset = 0;
        SDL_BindGPUIndexBuffer(m_render_pass, &index_binding, SDL_GPU_INDEXELEMENTSIZE_32BIT);
        SDL_DrawGPUIndexedPrimitives(m_render_pass, mesh.index_count, 1, 0, 0, 0);
    } else {
        SDL_DrawGPUPrimitives(m_render_pass, mesh.vertex_count, 1, 0, 0);
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

} // namespace stratum
