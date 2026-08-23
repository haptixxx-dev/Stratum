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

#include "renderer/gpu_buffer_pool.hpp"
#include "renderer/mesh.hpp"

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
 * @brief GPU-side mesh representation, as two ranges inside pooled device buffers
 *
 * A mesh used to own two SDL_GPUBuffers of its own. It no longer does. Both are
 * now suballocated ranges from GPURenderer's vertex and index pools, because a
 * city extract is thousands of meshes and thousands of device allocations run into
 * VkPhysicalDeviceLimits::maxMemoryAllocationCount -- commonly 4096 -- long before
 * they run out of VRAM. See gpu_buffer_pool.hpp for the whole argument.
 *
 * Nothing about drawing changed: SDL_GPUBufferBinding carries a byte offset, so a
 * range binds exactly like a whole buffer did, and indices stay mesh-local and
 * zero-based.
 */
struct GPUMesh {
    /**
     * @brief Vertex range, `vertex_count * sizeof(Vertex)` bytes
     *
     * Bind as `SDL_GPUBufferBinding{ vertex_alloc.buffer, vertex_alloc.offset }`
     * and draw with a vertex offset of 0: the binding offset already points at
     * this mesh's first vertex.
     */
    BufferAlloc vertex_alloc;

    /**
     * @brief Index range, `index_count * sizeof(uint32_t)` bytes
     *
     * Bind as `SDL_GPUBufferBinding{ index_alloc.buffer, index_alloc.offset }`
     * with SDL_GPU_INDEXELEMENTSIZE_32BIT. SubMesh::index_offset stays relative to
     * this range, so a submesh draw passes it straight through as `first_index`.
     */
    BufferAlloc index_alloc;

    uint32_t vertex_count = 0;
    uint32_t index_count = 0;

    /**
     * @brief Material ranges over the uploaded index buffer
     *
     * Recorded by upload_mesh() from Mesh::effective_submeshes(), so the ranges
     * always tile [0, index_count) with no gaps. An untagged source mesh yields
     * exactly one MaterialId::Default range covering everything, which draws
     * identically to the single whole-buffer draw this replaced.
     */
    std::vector<SubMesh> submeshes;

    /**
     * @brief False until this mesh's staged copy has actually executed
     *
     * upload_mesh() creates the buffers immediately but DEFERS the copy into a
     * per-frame batch (see flush_pending_uploads). Drawing a mesh before its
     * copy has run would render whatever uninitialised device memory happened to
     * be there, so draw_mesh() skips meshes that are not ready yet. They appear
     * a frame later, which is invisible next to the streaming that is already
     * happening.
     */
    bool ready = false;

    /**
     * @brief Meaning unchanged: this handle refers to real GPU geometry
     *
     * Still says nothing about whether the geometry has been COPIED yet -- that is
     * what `ready` is for -- and still nothing about whether it is worth drawing.
     */
    bool is_valid() const { return vertex_alloc.valid() && vertex_count > 0; }
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
 * @brief One staged mesh copy waiting for the next batched flush
 *
 * upload_mesh() reserves the pooled ranges immediately and copies the bytes into
 * a CPU-side arena; this records where they went. flush_pending_uploads() moves a
 * budgeted prefix of the queue through one transfer buffer per frame.
 */
struct PendingUpload {
    uint32_t mesh_id = 0;
    size_t   staging_offset = 0;   ///< byte offset into the staging arena
    uint32_t vertex_bytes = 0;
    uint32_t index_bytes = 0;

    /// Bytes this entry occupies in the arena and in the transfer buffer
    [[nodiscard]] size_t total_bytes() const {
        return static_cast<size_t>(vertex_bytes) + static_cast<size_t>(index_bytes);
    }
};

/**
 * @brief Where each admitted entry of one frame's batch lands in the transfer buffer
 *
 * `offsets` is parallel to the first `offsets.size()` entries of the queue, and
 * `bytes` is the transfer buffer size those offsets are packed into.
 */
struct UploadBatch {
    size_t bytes = 0;                ///< Total transfer-buffer bytes for the batch
    std::vector<size_t> offsets;     ///< Transfer-buffer offset of each admitted entry
};

/**
 * @brief Choose the prefix of @p queue this frame uploads, and pack it
 *
 * The transfer buffer layout is BUILT here rather than inherited from the staging
 * arena, and that distinction is the whole point of the function. The arena is
 * not contiguous over the queue: release_mesh() drops an entry whose mesh was
 * released before its copy ever ran and leaves its bytes behind as a hole, so
 * `staging_offset - front().staging_offset` is not an offset into a packed copy
 * of the batch. Deriving the source offset that way reads past the end of the
 * transfer buffer by exactly the size of the hole -- an out-of-range
 * vkCmdCopyBuffer when the buffer was freshly sized, and silently stale geometry
 * on a mesh marked ready when a larger buffer was carried over from an earlier
 * frame.
 *
 * Always admits at least one entry, or a mesh larger than the budget would never
 * upload at all.
 *
 * @param queue  Staged copies, oldest first
 * @param budget Maximum bytes to admit beyond the first entry
 * @return The packed offsets and the total. Empty when @p queue is empty.
 */
[[nodiscard]] UploadBatch plan_upload_batch(const std::vector<PendingUpload>& queue, size_t budget);

/**
 * @brief Bytes of dead prefix worth reclaiming from the staging arena after a flush
 *
 * The arena is append-only and every live entry carries an absolute offset into
 * it, so reclaiming the drained prefix means moving everything still queued down
 * and rebasing those offsets. Doing that on every flush is a memcpy of the whole
 * remaining backlog per frame -- hundreds of megabytes during a city-scale import,
 * where the queue drains at kMaxUploadBytesPerFrame and the traversal refills it
 * faster.
 *
 * So it is amortised: the arena is only compacted once the dead prefix is at
 * least half of it, which bounds the arena at twice the live bytes and makes the
 * copying amortised constant per staged byte.
 *
 * @param queue       Staged copies still queued, oldest first
 * @param arena_bytes Current size of the staging arena
 * @return Byte count to erase from the front, 0 to leave the arena alone
 */
[[nodiscard]] size_t staging_compaction_offset(const std::vector<PendingUpload>& queue,
                                               size_t arena_bytes);

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

    // === Resident Memory Budget ===
    //
    // Pooling the buffers (see gpu_buffer_pool.hpp) removes the device-allocation
    // ceiling. It does not remove the memory ceiling, and it does not decide what
    // to keep. A 63 MB Lucan extract is 864 km of highway and 10,373 ways, and
    // there is no version of that which fits in VRAM alongside its buildings and
    // its terrain. Something has to leave.
    //
    // The renderer cannot decide WHICH geometry leaves, because it does not know
    // where anything is: a GPUMesh is a vertex count and two ranges, with no
    // transform and no bounds. The owner -- the quadtree traversal, which already
    // computes a distance per visible node -- does know. So the policy is split:
    // the renderer owns the budget and the eviction mechanism, and asks the owner
    // for a distance through MeshDistanceFn.

    /**
     * @brief Caps on how much geometry stays resident
     */
    struct MemoryBudget {
        /**
         * @brief Ceiling on vertex plus index bytes held on the GPU
         *
         * Measured the same way resident_bytes() reports: the sum of the requested
         * range sizes, so pool block padding and per-block slack are NOT counted.
         * The real reservation is always somewhat larger; see
         * vertex_pool_stats().bytes_reserved for the true figure.
         *
         * 768 MB leaves headroom on a 2 GB card once the swapchain, the depth
         * target, MSAA resolve targets and the driver's own working set are paid
         * for. It is not a hardware query -- SDL_GPU does not expose a VRAM size --
         * so it is a policy number, and the application is expected to lower it on
         * constrained hardware.
         */
        size_t max_resident_bytes = 768ull * 1024 * 1024;

        /**
         * @brief Ceiling on live meshes
         *
         * Independent of the byte cap and worth keeping even though pooling has
         * decoupled meshes from device allocations: every mesh still costs a hash
         * map entry, a submesh vector, two binds and at least one draw call per
         * frame it is visible. 4096 meshes of road geometry is already more draw
         * calls than the frame has room for.
         */
        size_t max_resident_meshes = 4096;

        /**
         * @brief Evict automatically when an upload would breach a cap
         *
         * When false the caps become pure diagnostics and upload_mesh() refuses the
         * upload instead, incrementing upload_failures(). That is the pre-existing
         * behaviour and is kept as a switch, because "geometry silently missing"
         * and "geometry silently unloaded" are different bugs and being able to
         * pick which one you are looking at is worth a bool.
         */
        bool evict_under_pressure = true;
    };

    /**
     * @brief Replace the resident budget
     *
     * Does not evict. Lowering the caps takes effect at the next upload_mesh() or
     * the next explicit evict_to_budget(); call that yourself if you want the new
     * caps applied immediately.
     */
    void set_memory_budget(const MemoryBudget& budget);

    /// The budget currently in force
    [[nodiscard]] const MemoryBudget& memory_budget() const;

    /**
     * @brief Answers "how far is this mesh from the camera?" for the eviction sort
     *
     * Registered by the application, because the renderer has no idea where a mesh
     * is. The quadtree traversal already has the number: it computes a distance per
     * visible leaf every frame, and a leaf owns its meshes' ids.
     *
     * @param mesh_id A live mesh handle from upload_mesh()
     * @return Distance from the camera in metres. A NEGATIVE return means PINNED:
     *         the mesh is never evicted whatever the pressure. Use it for anything
     *         whose disappearance would be a bug rather than a stream-out -- the
     *         gizmo geometry, the grid, an editor overlay. A mesh the callback does
     *         not recognise should return a large positive distance, which makes it
     *         the first thing evicted, and that is the right answer for geometry
     *         nobody is tracking any more.
     */
    using MeshDistanceFn = std::function<float(uint32_t mesh_id)>;

    /// Install the distance query. Passing an empty function disables eviction; see evict_to_budget().
    void set_mesh_distance_fn(MeshDistanceFn fn);

    /**
     * @brief Evict the furthest meshes until both caps are satisfied
     *
     * Sorts live meshes by MeshDistanceFn descending and releases from the far end
     * until resident_bytes() and resident_mesh_count() are both within
     * memory_budget(), then stops. Ties break on ascending mesh id so a run is
     * reproducible.
     *
     * Never evicted:
     * - a mesh whose distance came back negative, that is, pinned;
     * - a mesh with a staged-but-unflushed upload, because its bytes are still
     *   referenced by m_staging and freeing its ranges would leave the next flush
     *   copying into somebody else's geometry.
     *
     * If those two classes alone exceed the budget, the function evicts what it
     * can, logs, and returns. It never breaks the guarantees to satisfy a number.
     *
     * @warning Does nothing and logs once when no MeshDistanceFn is installed.
     *          Evicting without a distance means evicting at random, which reliably
     *          throws away the road directly under the camera. Refusing is the
     *          correct failure.
     *
     * @warning Ranges are not returned to the pools immediately. Freeing a range
     *          the GPU may still be reading corrupts an in-flight draw, so an
     *          evicted mesh's allocations go onto a retirement queue and are freed
     *          once the frames that could reference them have completed. See
     *          m_retired_allocs.
     *
     * @return Number of meshes evicted. 0 when already within budget.
     */
    size_t evict_to_budget();

    /**
     * @brief Told when the renderer drops a mesh, so the owner can forget its handle
     *
     * Eviction invalidates a mesh id, and everything holding that id -- a
     * QuadTreeNode's road_gpu_ids, an ECS component, a cached debug mesh -- would
     * otherwise keep drawing a handle that no longer resolves. The callback fires
     * once per evicted mesh, from inside evict_to_budget(), BEFORE the id is
     * recycled.
     *
     * @warning Do not call back into the renderer from it. release_mesh(),
     *          upload_mesh() and evict_to_budget() all mutate the mesh map that
     *          eviction is walking. Clear your handle and return.
     */
    using MeshEvictedFn = std::function<void(uint32_t mesh_id)>;

    /// Install the eviction notification. Passing an empty function disables it.
    void set_mesh_evicted_fn(MeshEvictedFn fn);

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
     * @brief Begin a color-only render pass (NO depth attachment) for UI/overlay draws
     * @note Loads the existing swapchain contents so the 3D pass output is preserved.
     *       ImGui's SDL_GPU backend builds its pipeline with has_depth_stencil_target =
     *       false, so drawing it into the depth-attached 3D pass is render-pass
     *       incompatible (VUID-vkCmdDrawIndexed-renderPass-02684). This pass matches it.
     */
    void begin_ui_render_pass();

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

    /// Per-frame draw statistics, for the stats panel and for judging changes.
    struct FrameStats {
        uint32_t draw_calls = 0;
        uint32_t triangles = 0;
    };
    /// Stats for the last COMPLETED frame. The UI panels draw before render_3d,
    /// so reporting the in-progress frame would always show zero.
    FrameStats get_frame_stats() const { return m_frame_stats_last; }

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

    /**
     * @brief Load a SPIR-V shader module from disk
     * @param path Absolute path to a .spv file
     * @param stage Vertex or fragment stage
     * @param num_uniform_buffers Uniform buffer count declared by the shader
     * @param num_storage_buffers Storage buffer count declared by the shader
     * @return Shader handle, or nullptr on failure (caller owns it)
     * @note Public so auxiliary backends (e.g. the Im3d backend) can reuse it.
     */
    SDL_GPUShader* load_shader(const char* path, SDL_GPUShaderStage stage,
                                int num_uniform_buffers, int num_storage_buffers);

private:
    bool create_pipelines();
    bool create_simple_pipelines();
    bool create_pbr_pipelines();
    bool load_shaders();
    bool load_simple_shaders();
    bool load_pbr_shaders();
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
    // Allocated size of m_depth_texture, which is >= the swapchain size and is
    // rounded up so that a resize drag does not reallocate it every frame. The
    // Vulkan backend sizes the framebuffer as the min over all attachments
    // (SDL_gpu_vulkan.c:7768-7801), so an oversized depth target is harmless.
    uint32_t m_depth_alloc_width = 0;
    uint32_t m_depth_alloc_height = 0;

    FrameStats m_frame_stats;       ///< Accumulated over the frame in progress
    FrameStats m_frame_stats_last;  ///< Completed frame, what get_frame_stats reports

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

    /// CPU-side staging for the current batch; copied into the transfer buffer at flush.
    std::vector<uint8_t> m_staging;
    std::vector<PendingUpload> m_pending_uploads;

    /**
     * @brief Upload one batched command buffer's worth of staged copies
     *
     * Called at the top of begin_frame(), before the frame's render pass opens.
     * Everything staged since the last flush goes into ONE command buffer with
     * ONE copy pass and ONE submit.
     *
     * This exists because the previous design submitted a command buffer PER
     * MESH, from inside the visible-node traversal that runs while the frame's
     * own render pass is open. Every submit allocates a fence, and with no GPU
     * progress between them none could be retired, so a city-scale import piled
     * up thousands of live command buffers and fences in one frame and the
     * driver died on vkCreateFence VK_ERROR_OUT_OF_HOST_MEMORY -- host memory,
     * not VRAM. Batching turns thousands of submits per frame into one.
     */
    void flush_pending_uploads();

    /// Max bytes copied per frame. Leftovers stay staged and flush next frame,
    /// which is what turns a stampeding import into a stream.
    static constexpr size_t kMaxUploadBytesPerFrame = 24u * 1024u * 1024u;

    // === Pooled device buffers ===
    //
    // One pool per usage; a block is created with a single SDL_GPUBufferUsageFlags
    // and a suballocated range inherits it, so vertex and index data cannot share
    // one. See gpu_buffer_pool.hpp for why meshes stopped owning their buffers.

    GPUBufferPool m_vertex_pool;
    GPUBufferPool m_index_pool;

    /// Block size of the vertex pool. Roughly 500k Vertex at 64 bytes each.
    static constexpr uint32_t kVertexBlockBytes = 32u * 1024u * 1024u;

    /// Block size of the index pool. Roughly 2M 32-bit indices.
    static constexpr uint32_t kIndexBlockBytes = 8u * 1024u * 1024u;

    /**
     * @brief One evicted or released range waiting out the frames that may still read it
     *
     * GPUBufferPool::free() makes a range immediately reusable, with no idea
     * whether the GPU has finished with it. Freeing at release_mesh() time and
     * reallocating the same bytes in the next flush would have a copy pass
     * overwrite geometry that a submitted-but-unfinished command buffer is still
     * drawing from -- which shows up as another mesh's triangles flickering
     * through, and is close to impossible to attribute after the fact.
     *
     * So a range is retired, not freed: it is queued with the frame index at which
     * it stopped being referenced, and handed back to its pool once m_frame_index
     * has advanced past it by kBufferRetireFrames.
     */
    struct RetiredAlloc {
        BufferAlloc alloc;
        uint64_t retire_after_frame = 0;   ///< Free once m_frame_index exceeds this
        bool is_index = false;             ///< Which pool it came from
    };

    std::vector<RetiredAlloc> m_retired_allocs;

    /**
     * @brief Frames a freed range must sit out before the pool may reuse it
     *
     * Must be at least the renderer's maximum frames in flight. SDL_GPU keeps up
     * to three swapchain images in flight, so three is the floor and the value
     * here.
     */
    static constexpr uint64_t kBufferRetireFrames = 3;

    /// Frames begun since init(). Drives the retirement queue.
    uint64_t m_frame_index = 0;

    /**
     * @brief Queue one range for return to its pool once the GPU can no longer read it
     *
     * The only way a range is ever given up. Freeing at release time instead would
     * make the bytes immediately reallocatable while a submitted command buffer is
     * still drawing from them. An invalid alloc is a no-op, so a mesh with no
     * index data costs nothing here.
     *
     * @param alloc    Range to retire
     * @param is_index true when it came from the index pool, false for the vertex pool
     */
    void retire_alloc(const BufferAlloc& alloc, bool is_index);

    /**
     * @brief Return every retired range whose frames have elapsed
     *
     * Called once at the top of begin_frame(), alongside flush_pending_uploads().
     *
     * @param force Free every retired range regardless of how recently it was
     *              retired. Valid ONLY after SDL_WaitForGPUIdle(), which is why
     *              shutdown() is its one caller: at that point the frames the
     *              queue is waiting for will never arrive, and leaving the ranges
     *              queued would make the pools report them as leaks.
     */
    void drain_retired_allocs(bool force = false);

    /**
     * @brief evict_to_budget(), with room reserved for an upload about to happen
     *
     * The caps are tested against `resident + extra`, so upload_mesh() can make
     * space for a mesh BEFORE allocating it. Allocating first and evicting
     * afterwards would let the pool grow a block to hold the new mesh, which is
     * the device allocation the whole budget exists to prevent.
     *
     * @param extra_bytes  Bytes the caller is about to add
     * @param extra_meshes Meshes the caller is about to add
     * @return Meshes evicted
     */
    size_t evict_to_fit(size_t extra_bytes, size_t extra_meshes);

    // === Resident GPU geometry accounting and budget ===
    //
    // Pooling removed the device-allocation ceiling that used to be the binding
    // constraint here: a mesh no longer costs two SDL_CreateGPUBuffer calls, so
    // VkPhysicalDeviceLimits::maxMemoryAllocationCount -- commonly 4096 -- is now
    // reached by pool BLOCKS, of which there are a handful, rather than by meshes,
    // of which there are thousands.
    //
    // What is left is the memory ceiling and the draw-call ceiling, and those are
    // what MemoryBudget caps. Unlike before, these are no longer diagnostics only:
    // with MemoryBudget::evict_under_pressure, upload_mesh() evicts to make room
    // rather than refusing the upload.

    size_t m_resident_bytes = 0;
    size_t m_upload_failures = 0;

    /// Meshes dropped by evict_to_budget() since startup
    size_t m_evicted_meshes = 0;

    MemoryBudget m_memory_budget{};
    MeshDistanceFn m_mesh_distance_fn;
    MeshEvictedFn m_mesh_evicted_fn;

    /// "Over budget with no distance function" has been logged; do not repeat it
    /// every frame. Cleared when either the budget or the callback is replaced.
    bool m_eviction_warned = false;

    /// Earliest frame at which the "still over budget" warning may be logged again
    uint64_t m_next_budget_warn_frame = 0;

    /// Frames between repeats of that warning. It fires on a frame that is already
    /// thrashing, so it must not add a log write to every one of them.
    static constexpr uint64_t kBudgetWarnFrameInterval = 300;

public:
    /// Bytes of vertex + index data currently resident on the GPU, requested sizes only.
    [[nodiscard]] size_t resident_bytes() const { return m_resident_bytes; }
    /// Number of live meshes. Each costs two pooled ranges, not two device allocations.
    [[nodiscard]] size_t resident_mesh_count() const { return m_meshes.size(); }
    /// Uploads refused or failed since startup. Non-zero means geometry is missing.
    [[nodiscard]] size_t upload_failures() const { return m_upload_failures; }
    /// Meshes staged but not yet copied to the GPU.
    [[nodiscard]] size_t pending_upload_count() const { return m_pending_uploads.size(); }

    /**
     * @brief Meshes evicted to stay inside the budget since startup
     *
     * Climbing steadily while the camera is still means the budget is smaller than
     * the working set and the renderer is thrashing: geometry is being evicted and
     * re-uploaded every frame. Lower the view distance or raise
     * MemoryBudget::max_resident_bytes.
     */
    [[nodiscard]] size_t evicted_mesh_count() const { return m_evicted_meshes; }

    /**
     * @brief Occupancy of the vertex buffer pool
     *
     * `bytes_reserved` is the real device footprint, always at least
     * resident_bytes()'s vertex share; the gap is alignment padding and unused
     * block tail. `blocks` is the count that matters against the driver's
     * allocation limit. `fragmentation` climbing toward 1 while allocations start
     * failing is the signature of a free list that has stopped coalescing.
     */
    [[nodiscard]] GPUBufferPool::Stats vertex_pool_stats() const;

    /// Occupancy of the index buffer pool. Read the same way as vertex_pool_stats().
    [[nodiscard]] GPUBufferPool::Stats index_pool_stats() const;

    /// Ranges freed but not yet returned to their pool; see kBufferRetireFrames.
    [[nodiscard]] size_t retired_alloc_count() const { return m_retired_allocs.size(); }
};

} // namespace stratum
