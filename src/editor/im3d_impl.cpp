#include "editor/im3d_impl.hpp"
#include "renderer/gpu_renderer.hpp"
#include <im3d.h>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdint>

namespace stratum {

namespace {

/**
 * @brief Uniform block for the Im3d shaders (set 1, binding 0)
 *
 * Layout is verified against the compiled SPIR-V: member offsets are 0 / 64 / 72 / 76.
 * m_pad exists to match the shader's uPad and keep the block at 80 bytes.
 */
struct Im3dUniforms {
    glm::mat4 view_proj;   // offset 0
    glm::vec2 viewport;    // offset 64
    uint32_t first_vertex; // offset 72
    uint32_t pad = 0;      // offset 76
};
static_assert(sizeof(Im3dUniforms) == 80, "Im3dUniforms must match the shader's 80-byte block");

/// One contiguous run of vertices in the storage buffer, all of one primitive type.
struct DrawBatch {
    Im3d::DrawPrimitiveType type;
    uint32_t first_vertex;
    uint32_t vertex_count;
};

struct Im3dBackend {
    SDL_GPUDevice* device = nullptr;

    // Indexed by Im3d::DrawPrimitiveType (Triangles = 0, Lines = 1, Points = 2)
    SDL_GPUShader* vertex_shaders[Im3d::DrawPrimitive_Count]{};
    SDL_GPUShader* fragment_shaders[Im3d::DrawPrimitive_Count]{};
    SDL_GPUGraphicsPipeline* pipelines[Im3d::DrawPrimitive_Count]{};

    SDL_GPUBuffer* vertex_storage = nullptr;
    SDL_GPUTransferBuffer* transfer = nullptr;
    uint32_t capacity_bytes = 0;

    std::vector<DrawBatch> batches;

    bool frame_open = false;   // Im3D_NewFrame() ran this frame
    bool ready = false;        // shaders + pipelines created successfully
    bool warned_upload = false;
};

Im3dBackend s_gpu;

const char* primitive_name(int type) {
    switch (type) {
        case Im3d::DrawPrimitive_Triangles: return "triangles";
        case Im3d::DrawPrimitive_Lines:     return "lines";
        case Im3d::DrawPrimitive_Points:    return "points";
        default:                            return "unknown";
    }
}

/// Release the storage/transfer buffer pair. SDL defers actual destruction until
/// the resource is no longer in flight, so no SDL_WaitForGPUIdle is needed here.
void release_buffers() {
    if (s_gpu.vertex_storage) {
        SDL_ReleaseGPUBuffer(s_gpu.device, s_gpu.vertex_storage);
        s_gpu.vertex_storage = nullptr;
    }
    if (s_gpu.transfer) {
        SDL_ReleaseGPUTransferBuffer(s_gpu.device, s_gpu.transfer);
        s_gpu.transfer = nullptr;
    }
    s_gpu.capacity_bytes = 0;
}

/// Grow the storage + transfer buffers to hold at least required_bytes.
bool ensure_capacity(uint32_t required_bytes) {
    if (required_bytes <= s_gpu.capacity_bytes && s_gpu.vertex_storage && s_gpu.transfer) {
        return true;
    }

    release_buffers();

    // Grow with headroom so a slowly-growing scene does not reallocate every frame.
    uint32_t new_size = required_bytes + required_bytes / 2;
    const uint32_t kMinBytes = 64 * 1024;
    if (new_size < kMinBytes) new_size = kMinBytes;

    SDL_GPUBufferCreateInfo buffer_info{};
    buffer_info.usage = SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ;
    buffer_info.size = new_size;
    s_gpu.vertex_storage = SDL_CreateGPUBuffer(s_gpu.device, &buffer_info);
    if (!s_gpu.vertex_storage) {
        spdlog::error("Im3d: failed to create vertex storage buffer: {}", SDL_GetError());
        return false;
    }

    SDL_GPUTransferBufferCreateInfo transfer_info{};
    transfer_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    transfer_info.size = new_size;
    s_gpu.transfer = SDL_CreateGPUTransferBuffer(s_gpu.device, &transfer_info);
    if (!s_gpu.transfer) {
        spdlog::error("Im3d: failed to create transfer buffer: {}", SDL_GetError());
        SDL_ReleaseGPUBuffer(s_gpu.device, s_gpu.vertex_storage);
        s_gpu.vertex_storage = nullptr;
        return false;
    }

    s_gpu.capacity_bytes = new_size;
    spdlog::debug("Im3d: grew vertex buffers to {} bytes", new_size);
    return true;
}

bool create_pipeline_for(GPURenderer& renderer, int type) {
    // No vertex buffer is ever bound: the quad corner comes from gl_VertexIndex and
    // the vertex data is pulled from a storage buffer. So the input state stays empty.
    SDL_GPUVertexInputState vertex_input{};
    vertex_input.vertex_buffer_descriptions = nullptr;
    vertex_input.num_vertex_buffers = 0;
    vertex_input.vertex_attributes = nullptr;
    vertex_input.num_vertex_attributes = 0;

    SDL_GPURasterizerState rasterizer{};
    rasterizer.fill_mode = SDL_GPU_FILLMODE_FILL;
    // MUST be NONE: the expansion triangle strip alternates winding, and Im3d's own
    // triangles have arbitrary winding.
    rasterizer.cull_mode = SDL_GPU_CULLMODE_NONE;
    rasterizer.front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    rasterizer.enable_depth_bias = false;
    rasterizer.enable_depth_clip = true;

    SDL_GPUDepthStencilState depth_stencil{};
    depth_stencil.compare_op = SDL_GPU_COMPAREOP_LESS;
    depth_stencil.enable_depth_test = true;    // occlude behind buildings/terrain
    depth_stencil.enable_depth_write = false;  // translucent overlay must not write depth
    depth_stencil.enable_stencil_test = false;

    // Im3d overlays are translucent (the grid is alpha 0.2) and the antialiasing
    // fade is alpha-based, so blending is required. The mesh pipelines disable it.
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

    SDL_GPUColorTargetDescription color_target{};
    color_target.format = renderer.get_swapchain_format();
    color_target.blend_state = blend;

    // These three fields are what keep the pipeline render-pass-compatible with the
    // depth-attached 3D pass. Diverging here would reintroduce VUID-...-02684.
    SDL_GPUGraphicsPipelineTargetInfo target_info{};
    target_info.color_target_descriptions = &color_target;
    target_info.num_color_targets = 1;
    target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    target_info.has_depth_stencil_target = true;

    SDL_GPUMultisampleState multisample{};
    // NOTE: cached at creation time. If runtime MSAA changes are ever revived
    // (GPURenderer::set_msaa_level is currently unreachable), these pipelines must be
    // released and rebuilt too, or they will mismatch the 3D pass sample count.
    multisample.sample_count = renderer.get_sample_count();
    multisample.sample_mask = 0;
    multisample.enable_mask = false;

    SDL_GPUGraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.vertex_shader = s_gpu.vertex_shaders[type];
    pipeline_info.fragment_shader = s_gpu.fragment_shaders[type];
    pipeline_info.vertex_input_state = vertex_input;
    // Points and lines expand to a 4-vertex strip; triangles pass through as a list.
    pipeline_info.primitive_type = (type == Im3d::DrawPrimitive_Triangles)
                                       ? SDL_GPU_PRIMITIVETYPE_TRIANGLELIST
                                       : SDL_GPU_PRIMITIVETYPE_TRIANGLESTRIP;
    pipeline_info.rasterizer_state = rasterizer;
    pipeline_info.multisample_state = multisample;
    pipeline_info.depth_stencil_state = depth_stencil;
    pipeline_info.target_info = target_info;

    s_gpu.pipelines[type] = SDL_CreateGPUGraphicsPipeline(renderer.get_device(), &pipeline_info);
    if (!s_gpu.pipelines[type]) {
        spdlog::error("Im3d: failed to create {} pipeline: {}", primitive_name(type), SDL_GetError());
        return false;
    }
    return true;
}

} // namespace

void Im3D_Init() {
    // GPU resources are created separately in Im3D_InitGPU(), which needs a renderer.
}

void Im3D_Shutdown() {
    Im3D_ShutdownGPU();
}

bool Im3D_InitGPU(GPURenderer& renderer) {
    if (!renderer.is_initialized()) {
        spdlog::error("Im3d: cannot init GPU backend, renderer is not initialized");
        return false;
    }
    // Re-initialization would otherwise leak the previous shaders and pipelines.
    Im3D_ShutdownGPU();
    s_gpu.device = renderer.get_device();

    const char* base = SDL_GetBasePath();
    std::string base_path = base ? base : "";
    const char* names[Im3d::DrawPrimitive_Count] = { "triangles", "lines", "points" };

    for (int i = 0; i < Im3d::DrawPrimitive_Count; ++i) {
        std::string vert_path = base_path + "../../assets/shaders/im3d_" + names[i] + ".vert.spv";
        std::string frag_path = base_path + "../../assets/shaders/im3d_" + names[i] + ".frag.spv";

        // Degrade gracefully if the shaders are missing rather than killing the app.
        if (!std::filesystem::exists(vert_path) || !std::filesystem::exists(frag_path)) {
            spdlog::warn("Im3d shaders not found at {} - debug geometry disabled", vert_path);
            Im3D_ShutdownGPU();
            return false;
        }

        // Vertex stage: 1 uniform buffer (Im3dUniforms), 1 storage buffer (vertex data).
        s_gpu.vertex_shaders[i] = renderer.load_shader(vert_path.c_str(),
                                                       SDL_GPU_SHADERSTAGE_VERTEX, 1, 1);
        if (!s_gpu.vertex_shaders[i]) {
            spdlog::error("Failed to load Im3d vertex shader: {}", vert_path);
            Im3D_ShutdownGPU();
            return false;
        }

        // Fragment stage declares no descriptors at all.
        s_gpu.fragment_shaders[i] = renderer.load_shader(frag_path.c_str(),
                                                         SDL_GPU_SHADERSTAGE_FRAGMENT, 0, 0);
        if (!s_gpu.fragment_shaders[i]) {
            spdlog::error("Failed to load Im3d fragment shader: {}", frag_path);
            Im3D_ShutdownGPU();
            return false;
        }
    }

    for (int i = 0; i < Im3d::DrawPrimitive_Count; ++i) {
        if (!create_pipeline_for(renderer, i)) {
            Im3D_ShutdownGPU();
            return false;
        }
    }

    s_gpu.ready = true;
    spdlog::info("Im3d GPU backend initialized (points + lines + triangles)");
    return true;
}

void Im3D_ShutdownGPU() {
    if (!s_gpu.device) {
        s_gpu.ready = false;
        return;
    }

    for (int i = 0; i < Im3d::DrawPrimitive_Count; ++i) {
        if (s_gpu.pipelines[i]) {
            SDL_ReleaseGPUGraphicsPipeline(s_gpu.device, s_gpu.pipelines[i]);
            s_gpu.pipelines[i] = nullptr;
        }
        if (s_gpu.vertex_shaders[i]) {
            SDL_ReleaseGPUShader(s_gpu.device, s_gpu.vertex_shaders[i]);
            s_gpu.vertex_shaders[i] = nullptr;
        }
        if (s_gpu.fragment_shaders[i]) {
            SDL_ReleaseGPUShader(s_gpu.device, s_gpu.fragment_shaders[i]);
            s_gpu.fragment_shaders[i] = nullptr;
        }
    }

    release_buffers();
    s_gpu.batches.clear();
    s_gpu.ready = false;
    s_gpu.device = nullptr;
}

void Im3D_ProcessEvent(const SDL_Event* event) {
    (void)event;
    // Input is polled in Im3D_NewFrame() rather than handled event-wise.
}

void Im3D_NewFrame(float dt, const Camera& cam, float window_width, float window_height, bool has_focus) {
    Im3d::AppData& ad = Im3d::GetAppData();

    ad.m_deltaTime = dt;
    ad.m_viewportSize = Im3d::Vec2(window_width, window_height);
    ad.m_viewOrigin = Im3d::Vec3(cam.get_position().x, cam.get_position().y, cam.get_position().z);
    ad.m_viewDirection = Im3d::Vec3(cam.get_forward().x, cam.get_forward().y, cam.get_forward().z);
    ad.m_worldUp = Im3d::Vec3(cam.get_up().x, cam.get_up().y, cam.get_up().z);
    ad.m_projOrtho = false;

    // Scale factor for pixel size -> world size. Im3d wants tan(fov/2) for a
    // perspective projection (see AppData docs in im3d.h and Context::pixelsToWorldSize
    // in im3d.cpp); this used to store the reciprocal, which made gizmo sizing wrong.
    // World height at distance d is 2*d*tan(fov/2), so the factor of 2 is part of
    // the conversion -- see Context::pixelsToWorldSize (im3d.cpp:2383) and the
    // reference backends (examples/OpenGL33/im3d_opengl33.cpp:121-123).
    ad.m_projScaleY = tanf(glm::radians(cam.m_fov) * 0.5f) * 2.0f;

    // Input handling
    // We only capture input if the viewport has focus to avoid stealing from ImGui
    if (has_focus) {
        auto mouse_state = SDL_GetMouseState(nullptr, nullptr);

        ad.m_keyDown[Im3d::Mouse_Left] = (mouse_state & SDL_BUTTON_LMASK) != 0;

        // Map other keys if gizmos are needed (Ctrl, Shift, etc)
        const bool* keys = SDL_GetKeyboardState(NULL);
        ad.m_keyDown[Im3d::Key_L] = keys[SDL_SCANCODE_L];
        ad.m_keyDown[Im3d::Key_T] = keys[SDL_SCANCODE_T];
        ad.m_keyDown[Im3d::Key_R] = keys[SDL_SCANCODE_R];
        ad.m_keyDown[Im3d::Key_S] = keys[SDL_SCANCODE_S];
    } else {
        ad.m_keyDown[Im3d::Mouse_Left] = false;
    }

    // Ray picking setup would go here (Screen point -> Ray)

    Im3d::NewFrame();
    s_gpu.frame_open = true;
}

void Im3D_EndFrameAndUpload(GPURenderer& renderer) {
    s_gpu.batches.clear();

    // Im3D_NewFrame() is only called while the Viewport panel is drawing. If the
    // panel is closed there is no open Im3d frame, and calling EndFrame() would
    // re-push stale draw lists (and trip IM3D_ASSERT in a debug build).
    if (!s_gpu.frame_open) {
        return;
    }
    s_gpu.frame_open = false;

    Im3d::EndFrame();

    if (!s_gpu.ready) {
        return;
    }

    SDL_GPUCommandBuffer* cmd = renderer.get_command_buffer();
    if (!cmd) {
        return;
    }

    const uint32_t list_count = Im3d::GetDrawListCount();
    const Im3d::DrawList* lists = Im3d::GetDrawLists();
    if (list_count == 0 || !lists) {
        return;
    }

    uint32_t total_vertices = 0;
    for (uint32_t i = 0; i < list_count; ++i) {
        total_vertices += lists[i].m_vertexCount;
    }
    if (total_vertices == 0) {
        return;
    }

    const uint32_t required_bytes =
        static_cast<uint32_t>(total_vertices * sizeof(Im3d::VertexData));
    if (!ensure_capacity(required_bytes)) {
        return;
    }

    // cycle = true lets SDL rename the allocation instead of stalling on the
    // previous frame's in-flight reads.
    void* mapped = SDL_MapGPUTransferBuffer(s_gpu.device, s_gpu.transfer, true);
    if (!mapped) {
        if (!s_gpu.warned_upload) {
            spdlog::error("Im3d: failed to map transfer buffer: {}", SDL_GetError());
            s_gpu.warned_upload = true;
        }
        return;
    }

    uint8_t* dst = static_cast<uint8_t*>(mapped);
    uint32_t cursor_vertices = 0;

    // Iterate in array order: Context::endFrame() emits unsorted lists before
    // depth-sorted ones, and that order is load-bearing for layering.
    for (uint32_t i = 0; i < list_count; ++i) {
        const Im3d::DrawList& list = lists[i];
        if (list.m_vertexCount == 0 || !list.m_vertexData) continue;

        const size_t bytes = list.m_vertexCount * sizeof(Im3d::VertexData);
        std::memcpy(dst + cursor_vertices * sizeof(Im3d::VertexData), list.m_vertexData, bytes);

        s_gpu.batches.push_back(DrawBatch{ list.m_primType, cursor_vertices, list.m_vertexCount });
        cursor_vertices += list.m_vertexCount;
    }

    SDL_UnmapGPUTransferBuffer(s_gpu.device, s_gpu.transfer);

    // A copy pass cannot be open while a render pass is bound - this is why upload
    // and draw are two separate entry points.
    SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);
    if (!copy_pass) {
        if (!s_gpu.warned_upload) {
            spdlog::error("Im3d: failed to begin copy pass: {}", SDL_GetError());
            s_gpu.warned_upload = true;
        }
        s_gpu.batches.clear();
        return;
    }

    SDL_GPUTransferBufferLocation src{};
    src.transfer_buffer = s_gpu.transfer;
    src.offset = 0;

    SDL_GPUBufferRegion dst_region{};
    dst_region.buffer = s_gpu.vertex_storage;
    dst_region.offset = 0;
    dst_region.size = required_bytes;

    SDL_UploadToGPUBuffer(copy_pass, &src, &dst_region, true);
    SDL_EndGPUCopyPass(copy_pass);
}

void Im3D_Render(GPURenderer& renderer, const glm::mat4& view_proj,
                 float viewport_w, float viewport_h) {
    if (!s_gpu.ready || s_gpu.batches.empty() || !s_gpu.vertex_storage) {
        return;
    }
    if (viewport_w < 1.0f || viewport_h < 1.0f) {
        return;
    }

    SDL_GPURenderPass* pass = renderer.get_render_pass();
    SDL_GPUCommandBuffer* cmd = renderer.get_command_buffer();
    if (!pass || !cmd) {
        return;
    }

    for (const DrawBatch& batch : s_gpu.batches) {
        const int type = static_cast<int>(batch.type);
        if (type < 0 || type >= Im3d::DrawPrimitive_Count || !s_gpu.pipelines[type]) {
            continue;
        }

        SDL_BindGPUGraphicsPipeline(pass, s_gpu.pipelines[type]);

        // Bind after the pipeline so a pipeline change cannot invalidate the binding.
        SDL_BindGPUVertexStorageBuffers(pass, 0, &s_gpu.vertex_storage, 1);

        // first_vertex lets one concatenated storage buffer serve every draw list,
        // since SDL_BindGPUVertexStorageBuffers takes no offset argument.
        Im3dUniforms uniforms{};
        uniforms.view_proj = view_proj;
        uniforms.viewport = glm::vec2(viewport_w, viewport_h);
        uniforms.first_vertex = batch.first_vertex;
        // Uniforms are pushed on the COMMAND BUFFER, not the render pass.
        SDL_PushGPUVertexUniformData(cmd, 0, &uniforms, sizeof(uniforms));

        switch (batch.type) {
            case Im3d::DrawPrimitive_Triangles:
                // 3 vertices per instance, one instance per triangle
                SDL_DrawGPUPrimitives(pass, 3, batch.vertex_count / 3, 0, 0);
                break;
            case Im3d::DrawPrimitive_Lines:
                // 4-vertex strip per instance, one instance per line segment
                SDL_DrawGPUPrimitives(pass, 4, batch.vertex_count / 2, 0, 0);
                break;
            case Im3d::DrawPrimitive_Points:
                // 4-vertex strip per instance, one instance per point
                SDL_DrawGPUPrimitives(pass, 4, batch.vertex_count, 0, 0);
                break;
            default:
                break;
        }
    }
}

} // namespace stratum
