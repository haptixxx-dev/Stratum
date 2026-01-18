/**
 * @file gpu_renderer.hpp
 * @brief SDL_GPU based renderer for high-performance 3D rendering
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Uses SDL3's GPU API with Vulkan backend for efficient mesh rendering.
 */

#pragma once

#include <SDL3/SDL.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace stratum {

// Forward declarations
class Mesh;
struct Vertex;

/**
 * @brief GPU-side mesh representation with buffer handles
 */
struct GPUMesh {
    SDL_GPUBuffer* vertex_buffer = nullptr;
    SDL_GPUBuffer* index_buffer = nullptr;
    uint32_t vertex_count = 0;
    uint32_t index_count = 0;
    bool is_valid() const { return vertex_buffer != nullptr && vertex_count > 0; }
};

/**
 * @brief Uniform data pushed to vertex shader
 */
struct alignas(16) MeshUniforms {
    glm::mat4 mvp;           // Model-View-Projection matrix
    glm::mat4 model;         // Model matrix (for lighting)
    glm::vec4 color_tint;    // Optional color tint/override
};

/**
 * @brief SDL_GPU based renderer
 *
 * Manages GPU device, pipelines, and provides efficient mesh rendering.
 * Designed for rendering large amounts of city geometry.
 */
class GPURenderer {
public:
    GPURenderer() = default;
    ~GPURenderer();

    // Non-copyable
    GPURenderer(const GPURenderer&) = delete;
    GPURenderer& operator=(const GPURenderer&) = delete;

    /**
     * @brief Initialize the GPU device and pipelines
     * @param window SDL window to render to
     * @return true on success
     */
    bool init(SDL_Window* window);

    /**
     * @brief Shutdown and release all GPU resources
     */
    void shutdown();

    /**
     * @brief Check if renderer is initialized
     */
    bool is_initialized() const { return m_device != nullptr; }

    // === Resource Management ===

    /**
     * @brief Upload a mesh to the GPU
     * @param mesh CPU-side mesh data
     * @return GPU mesh handle (ID for later reference)
     */
    uint32_t upload_mesh(const Mesh& mesh);

    /**
     * @brief Release a GPU mesh
     * @param mesh_id Mesh handle from upload_mesh()
     */
    void release_mesh(uint32_t mesh_id);

    /**
     * @brief Release all uploaded meshes
     */
    void release_all_meshes();

    // === Frame Rendering ===

    /**
     * @brief Begin a new frame - acquire command buffer and swapchain
     * @return true if frame can proceed (swapchain acquired)
     * @note Does NOT begin render pass. Call begin_render_pass() after
     *       preparing ImGui draw data.
     */
    bool begin_frame();

    /**
     * @brief Begin the main render pass
     * @note Call after ImGui_ImplSDLGPU3_PrepareDrawData()
     */
    void begin_render_pass();

    /**
     * @brief End the current render pass
     */
    void end_render_pass();

    /**
     * @brief End frame and present
     */
    void end_frame();

    /**
     * @brief Set the view and projection matrices for this frame
     */
    void set_view_projection(const glm::mat4& view, const glm::mat4& projection);

    /**
     * @brief Bind the mesh rendering pipeline
     * @note Call before draw_mesh() calls
     */
    void bind_mesh_pipeline();

    /**
     * @brief Draw a mesh with the given transform
     * @param mesh_id Mesh handle from upload_mesh()
     * @param model Model transform matrix
     * @param color_tint Optional color tint (default white = no tint)
     */
    void draw_mesh(uint32_t mesh_id, const glm::mat4& model = glm::mat4(1.0f),
                   const glm::vec4& color_tint = glm::vec4(1.0f));

    /**
     * @brief Draw a mesh directly without caching (for dynamic geometry)
     * @param mesh CPU-side mesh to draw
     * @param model Model transform matrix
     */
    void draw_mesh_immediate(const Mesh& mesh, const glm::mat4& model = glm::mat4(1.0f));

    /**
     * @brief Set the viewport for the current render pass
     */
    void set_viewport(const SDL_GPUViewport& viewport);

    /**
     * @brief Render ImGui draw data within the current render pass
     * @note Must be called between begin_frame() and end_frame()
     */
    void render_imgui();

    // === Getters ===
    SDL_GPUDevice* get_device() const { return m_device; }
    SDL_Window* get_window() const { return m_window; }
    SDL_GPUCommandBuffer* get_command_buffer() const { return m_cmd_buffer; }
    SDL_GPURenderPass* get_render_pass() const { return m_render_pass; }
    SDL_GPUTexture* get_swapchain_texture() const { return m_swapchain_texture; }
    SDL_GPUTextureFormat get_swapchain_format() const;

private:
    bool create_pipelines();
    bool load_shaders();
    SDL_GPUShader* load_shader(const char* path, SDL_GPUShaderStage stage);

    // GPU handles
    SDL_GPUDevice* m_device = nullptr;
    SDL_Window* m_window = nullptr;

    // Pipelines
    SDL_GPUGraphicsPipeline* m_mesh_pipeline = nullptr;
    SDL_GPUShader* m_vertex_shader = nullptr;
    SDL_GPUShader* m_fragment_shader = nullptr;

    // Frame state
    SDL_GPUCommandBuffer* m_cmd_buffer = nullptr;
    SDL_GPURenderPass* m_render_pass = nullptr;
    SDL_GPUTexture* m_swapchain_texture = nullptr;
    SDL_GPUTexture* m_depth_texture = nullptr;
    uint32_t m_swapchain_width = 0;
    uint32_t m_swapchain_height = 0;

    // Matrices
    glm::mat4 m_view{1.0f};
    glm::mat4 m_projection{1.0f};
    glm::mat4 m_view_projection{1.0f};

    // Mesh storage
    std::unordered_map<uint32_t, GPUMesh> m_meshes;
    uint32_t m_next_mesh_id = 1;

    // Transfer buffer for uploads (reused)
    SDL_GPUTransferBuffer* m_transfer_buffer = nullptr;
    size_t m_transfer_buffer_size = 0;
};

} // namespace stratum
