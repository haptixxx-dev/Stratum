/**
 * @file gpu_renderer.hpp
 * @brief SDL_GPU based renderer for high-performance 3D rendering
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Uses SDL3's GPU API with Vulkan backend for efficient mesh rendering.
 * Supports PBR lighting, push constants, and bindless textures.
 */

#pragma once

#include <SDL3/SDL.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <functional>

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
 * @brief Push constants for per-draw data (fastest path)
 * Must match shader layout - max 128 bytes
 * NOTE: Currently unused - simple shader uses uniform buffer only
 */
struct alignas(16) PushConstants {
    glm::mat4 mvp;              // 64 bytes - Model-View-Projection matrix
    uint32_t material_id;       // 4 bytes - Material index for bindless
    uint32_t instance_offset;   // 4 bytes - Base instance for instancing
    glm::vec2 uv_scale;         // 8 bytes - UV tiling
    // Total: 80 bytes (within 128 byte limit)
};

/**
 * @brief Mesh uniforms - matches the simple shader layout (set 1, binding 0)
 */
struct alignas(16) MeshUniforms {
    glm::mat4 mvp;               // Model-View-Projection
    glm::mat4 model;             // World transform
    glm::vec4 color_tint;        // RGBA color multiplier
};

/**
 * @brief PBR Mesh uniforms - extended layout for PBR shader (set 1, binding 0)
 */
struct alignas(16) MeshUniformsPBR {
    glm::mat4 mvp;               // Model-View-Projection
    glm::mat4 model;             // World transform
    glm::mat4 normal_matrix;     // Inverse-transpose for correct normals
    glm::vec4 color_tint;        // RGBA color multiplier
    glm::vec4 camera_position;   // xyz = camera pos, w = time
};

/**
 * @brief Scene uniforms for PBR lighting (set 2, binding 0)
 */
struct alignas(16) SceneUniforms {
    glm::vec4 camera_position;   // xyz = position, w = exposure
    glm::vec4 sun_direction;     // xyz = normalized direction, w = intensity
    glm::vec4 sun_color;         // rgb = color, a = ambient intensity
    glm::vec4 fog_params;        // x = start, y = end, z = density, w = enabled
    glm::vec4 fog_color;         // rgb = color, a = unused
    glm::vec4 pbr_params;        // x = metallic, y = roughness, z = ao, w = unused
};

/**
 * @brief PBR Material data for material buffer
 */
struct alignas(16) GPUMaterial {
    glm::vec4 base_color;        // rgb = albedo, a = alpha
    glm::vec4 pbr_params;        // r = metallic, g = roughness, b = ao, a = emissive
    glm::vec4 emissive_color;    // rgb = emissive, a = intensity
    glm::uvec4 texture_indices;  // x = albedo, y = normal, z = metallic_roughness, w = emissive
};

/**
 * @brief Shader rendering mode - can be switched at runtime
 */
enum class ShaderMode {
    Simple,     // Basic diffuse lighting, fast, good for debugging
    PBR         // Full PBR with Cook-Torrance BRDF, tone mapping, fog
};

/**
 * @brief Specialization constants for shader variants
 */
struct ShaderSpecialization {
    bool use_textures = false;
    bool use_vertex_colors = true;
    bool use_normal_mapping = false;
    bool use_instancing = false;
    bool use_pbr = true;
    bool use_ibl = false;
};

/**
 * @brief Fill mode for mesh rendering
 */
enum class FillMode {
    Solid,
    Wireframe
};

/**
 * @brief SDL_GPU based renderer
 *
 * Manages GPU device, pipelines, and provides efficient mesh rendering.
 * Designed for rendering large amounts of city geometry with PBR lighting.
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
     * @brief Set camera position for lighting calculations
     */
    void set_camera_position(const glm::vec3& position);

    /**
     * @brief Update scene lighting parameters
     */
    void set_scene_lighting(const glm::vec3& sun_dir, const glm::vec3& sun_color, 
                            float sun_intensity, float ambient_intensity);

    /**
     * @brief Set fog parameters
     * @param mode 0 = disabled, 1 = linear, 2 = exponential, 3 = exponential squared
     * @param color Fog color
     * @param start Start distance for linear fog
     * @param end End distance for linear fog
     * @param density Density for exponential fog modes
     */
    void set_fog(int mode, const glm::vec3& color, float start, float end, float density);

    /**
     * @brief Get current fog parameters
     * @return vec4(start, end, density, mode)
     */
    glm::vec4 get_fog_params() const { return m_scene_uniforms.fog_params; }

    /**
     * @brief Get current fog color
     */
    glm::vec3 get_fog_color() const { return glm::vec3(m_scene_uniforms.fog_color); }

    /**
     * @brief Bind the mesh rendering pipeline
     * @note Call before draw_mesh() calls
     */
    void bind_mesh_pipeline();

    /**
     * @brief Set the fill mode for mesh rendering
     * @param mode Solid or Wireframe
     */
    void set_fill_mode(FillMode mode);

    /**
     * @brief Get current fill mode
     */
    FillMode get_fill_mode() const { return m_current_fill_mode; }

    /**
     * @brief Set shader mode (Simple or PBR) - can be changed at runtime
     * @param mode ShaderMode to use
     * @return true if switch was successful
     */
    bool set_shader_mode(ShaderMode mode);

    /**
     * @brief Get current shader mode
     */
    ShaderMode get_shader_mode() const { return m_current_shader_mode; }

    /**
     * @brief Set PBR material parameters (only used in PBR mode)
     */
    void set_pbr_params(float metallic, float roughness, float ao = 1.0f);

    /**
     * @brief Get current PBR parameters
     */
    glm::vec3 get_pbr_params() const { 
        return glm::vec3(m_scene_uniforms.pbr_params); 
    }

    /**
     * @brief Set MSAA level
     * @param level 0=off, 1=2x, 2=4x, 3=8x
     * @return true if successful (may fail if level not supported)
     */
    bool set_msaa_level(int level);

    /**
     * @brief Get current MSAA level
     * @return 0=off, 1=2x, 2=4x, 3=8x
     */
    int get_msaa_level() const;

    /**
     * @brief Get current sample count
     */
    SDL_GPUSampleCount get_sample_count() const { return m_sample_count; }

    /**
     * @brief Set callback for when MSAA changes (for ImGui reinitialization)
     */
    void set_msaa_changed_callback(std::function<void(SDL_GPUSampleCount)> callback) {
        m_msaa_changed_callback = callback;
    }

    /**
     * @brief Draw a mesh with the given transform
     * @param mesh_id Mesh handle from upload_mesh()
     * @param model Model transform matrix
     * @param color_tint Optional color tint (default white = no tint)
     * @param material_id Material index for PBR (default 0)
     */
    void draw_mesh(uint32_t mesh_id, const glm::mat4& model = glm::mat4(1.0f),
                   const glm::vec4& color_tint = glm::vec4(1.0f),
                   uint32_t material_id = 0);

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

    /**
     * @brief Set exposure for tone mapping
     */
    void set_exposure(float exposure) { m_scene_uniforms.camera_position.w = exposure; }

    /**
     * @brief Get current exposure value
     */
    float get_exposure() const { return m_scene_uniforms.camera_position.w; }

    // === Getters ===
    SDL_GPUDevice* get_device() const { return m_device; }
    SDL_Window* get_window() const { return m_window; }
    SDL_GPUCommandBuffer* get_command_buffer() const { return m_cmd_buffer; }
    SDL_GPURenderPass* get_render_pass() const { return m_render_pass; }
    SDL_GPUTexture* get_swapchain_texture() const { return m_swapchain_texture; }
    SDL_GPUTextureFormat get_swapchain_format() const;

    // === Scene state getters ===
    const SceneUniforms& get_scene_uniforms() const { return m_scene_uniforms; }

private:
    bool create_pipelines();
    bool create_simple_pipelines();
    bool create_pbr_pipelines();
    bool load_shaders();
    bool load_simple_shaders();
    bool load_pbr_shaders();
    SDL_GPUShader* load_shader(const char* path, SDL_GPUShaderStage stage, 
                                int num_uniform_buffers, int num_storage_buffers);
    void create_msaa_textures();
    void release_msaa_textures();
    void release_pipelines();
    void update_scene_uniforms();
    glm::mat4 compute_normal_matrix(const glm::mat4& model);

    // GPU handles
    SDL_GPUDevice* m_device = nullptr;
    SDL_Window* m_window = nullptr;

    // Simple shader pipelines
    SDL_GPUGraphicsPipeline* m_mesh_pipeline = nullptr;
    SDL_GPUGraphicsPipeline* m_mesh_pipeline_wireframe = nullptr;
    SDL_GPUShader* m_vertex_shader = nullptr;
    SDL_GPUShader* m_fragment_shader = nullptr;

    // PBR shader pipelines
    SDL_GPUGraphicsPipeline* m_pbr_pipeline = nullptr;
    SDL_GPUGraphicsPipeline* m_pbr_pipeline_wireframe = nullptr;
    SDL_GPUShader* m_pbr_vertex_shader = nullptr;
    SDL_GPUShader* m_pbr_fragment_shader = nullptr;

    // Render state
    FillMode m_current_fill_mode = FillMode::Solid;
    ShaderMode m_current_shader_mode = ShaderMode::Simple;

    // MSAA state
    SDL_GPUSampleCount m_sample_count = SDL_GPU_SAMPLECOUNT_1;
    SDL_GPUTexture* m_msaa_color_texture = nullptr;
    SDL_GPUTexture* m_msaa_depth_texture = nullptr;
    std::function<void(SDL_GPUSampleCount)> m_msaa_changed_callback;

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

    // Scene uniforms (lighting, fog, etc.)
    SceneUniforms m_scene_uniforms{};

    // Camera position (for specular calculations)
    glm::vec3 m_camera_position{0.0f};

    // Mesh storage
    std::unordered_map<uint32_t, GPUMesh> m_meshes;
    uint32_t m_next_mesh_id = 1;

    // Transfer buffer for uploads (reused)
    SDL_GPUTransferBuffer* m_transfer_buffer = nullptr;
    size_t m_transfer_buffer_size = 0;
};

} // namespace stratum
