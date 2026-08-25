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
#include "renderer/material_library.hpp"
#include "renderer/mesh.hpp"
#include "renderer/texture.hpp"

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
    /// RESERVED. Nothing reads this: mesh_pbr.frag takes metallic, roughness and
    /// ao from the per-material block (MaterialUniforms::pbr_params) instead, so
    /// this member exists only to keep the std140 layout of the block matching the
    /// shader's. Do not add a control for it without adding a shader read first.
    glm::vec4 pbr_params;

    // ------------------------------------------------------------------------
    // Sky and image-based lighting
    // ------------------------------------------------------------------------
    // Read by assets/shaders/sky_common.glsl, which BOTH sky.frag and
    // mesh_pbr.frag include. These four are what replaced a hand-picked clear
    // colour sitting next to an unrelated hand-picked ambient constant: the
    // background and the fill light are now two evaluations of one function, so
    // they cannot disagree.
    //
    // The values are scene-referred radiance, not 0..1 display colours. Exposure
    // and the ACES curve bring them to the screen.

    /// rgb = zenith radiance, a = overall sky intensity.
    glm::vec4 sky_zenith;

    /// rgb = horizon radiance, a = zenith falloff exponent. The exponent is what
    /// keeps the bright band hugging the horizon; a linear gradient puts it far
    /// too high. Clamped away from 0 in the shader.
    glm::vec4 sky_horizon;

    /// rgb = ground bounce radiance, a = its intensity. This is the LOWER lobe of
    /// the hemisphere ambient, i.e. what lights the underside of a bridge deck.
    glm::vec4 ground_color;

    /// x = specular ambient scale, y = cos(sun angular radius) for the drawn sun
    /// disk, z = aerial perspective (0 = authored fog colour, 1 = sky colour),
    /// w = sun glow exponent (haze tightness).
    glm::vec4 ibl_params;
};

/// Cascades the shadow system can render. The GLSL array is sized to match, so
/// raising this means editing kMaxShadowCascades in mesh_pbr.frag too.
inline constexpr int kMaxShadowCascades = 4;

/**
 * @brief Fragment uniforms for the cascaded shadow lookup (set 3, binding 2)
 *
 * MUST match the ShadowUniforms block in assets/shaders/mesh_pbr.frag.
 *
 * shadow_params.x is the live cascade COUNT, and zero means "no shadows". That
 * is not a fallback, it is the documented off switch: a caller that pushes a
 * zeroed block gets fully lit geometry instead of comparisons against an
 * uninitialised depth texture. The PBR shader test suite relies on it.
 */
struct alignas(16) ShadowUniforms {
    /// World space to cascade clip space, one per cascade.
    glm::mat4 light_view_proj[kMaxShadowCascades];

    /// World size of one shadow texel, per cascade. Drives the normal offset,
    /// which has to shrink as the cascades get tighter or the near cascade is
    /// over-offset and its contact shadows detach.
    glm::vec4 cascade_texel_world;

    /// Constant depth bias in normalised [0, 1] clip depth, per cascade.
    glm::vec4 cascade_depth_bias;

    /// x = cascade count, y = normal offset in texels, z = strength,
    /// w = 1 / shadow map size.
    glm::vec4 shadow_params;

    /// x = fade start distance, y = fade end distance, z = PCF radius in texels,
    /// w = unused.
    glm::vec4 shadow_fade;
};

/**
 * @brief Vertex uniforms for the shadow depth pass (set 1, binding 0)
 *
 * MUST match the ShadowMeshUniforms block in assets/shaders/shadow.vert. Position
 * only: the shadow pass reads no other vertex attribute and binds no material.
 */
struct alignas(16) ShadowMeshUniforms {
    glm::mat4 light_mvp;
};

/**
 * @brief Tunables for the cascaded shadow map
 */
struct ShadowConfig {
    bool enabled = true;

    /// 1 to kMaxShadowCascades. More cascades means more depth passes over the
    /// whole caster list, so this is the main cost dial.
    int cascade_count = 3;

    /// Square edge of each cascade layer, in texels.
    uint32_t map_size = 2048;

    /// How far from the camera shadows are rendered at all. Beyond this the
    /// shader fades to fully lit; there is no data further out.
    float max_distance = 800.0f;

    /// Blend between a uniform split (0) and a logarithmic one (1). Logarithmic
    /// puts resolution where perspective needs it; a little uniform keeps the
    /// far cascade from covering an absurd volume.
    float split_lambda = 0.85f;

    /// Offset along the surface normal before the depth comparison, in texels of
    /// the selected cascade. This, not the depth bias, is what removes acne.
    float normal_offset = 2.0f;

    /// Constant depth bias, in WORLD METRES. Converted to each cascade's own
    /// normalised depth range on the way to the shader, so changing
    /// max_distance does not silently change how biased the near cascade is.
    float depth_bias_metres = 0.05f;

    /// 0 lifts shadows to fully lit, 1 is the full comparison result.
    float strength = 1.0f;

    /// PCF kernel spacing in texels. The kernel is 3x3 taps of a hardware
    /// comparison sampler, so each tap is already bilinear-filtered.
    float pcf_radius = 1.0f;
};

/**
 * @brief Vertex uniforms for the fullscreen sky pass (set 1, binding 0)
 *
 * MUST match the SkyUniforms block in assets/shaders/sky.vert. The sky needs no
 * geometry -- it turns each pixel into a world-space ray instead -- so the
 * inverse view-projection and the eye position are the whole input.
 */
struct alignas(16) SkyUniforms {
    glm::mat4 inv_view_projection;
    glm::vec4 camera_position;   // xyz = world position, w = unused
};

// GPUMaterial used to be declared here: a four-vec4 block with bindless texture
// indices, never constructed, never pushed, never read by any shader. It has been
// REMOVED rather than left as a second, plausible-looking definition of the same
// thing. The real per-draw material block is MaterialUniforms in
// material_library.hpp, which is three vec4s, matches an actual uniform block in
// assets/shaders/mesh_pbr.frag, and binds real textures through
// GPUTextureManager. Its bindless texture_indices were aspirational: SDL_GPU has
// no bindless path, textures are bound per draw through
// SDL_BindGPUFragmentSamplers, and pretending otherwise is what kept the texture
// work looking half-done for longer than it was.

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
     * @brief Render the sun's shadow cascades for this frame
     *
     * Call once per frame between begin_frame() and begin_render_pass(): it opens
     * depth-only render passes of its own, and it must finish before anything
     * samples the map.
     *
     * WHAT IT DRAWS IS LAST FRAME'S VISIBLE SET. draw_mesh() records every mesh it
     * draws, and this replays the recording from the previous frame. The
     * alternative -- traversing the scene again per cascade from the light's point
     * of view -- would mean running Editor's quadtree traversal three more times a
     * frame, which also drives level-of-detail selection, chunk streaming and the
     * per-frame statistics; those would all fire three extra times for a pass that
     * wants nothing from them.
     *
     * The cost of the shortcut is honest and worth stating: a caster that is
     * outside the camera frustum does not cast into it. With the sun high this is
     * invisible; with the sun low it shows as a missing shadow entering from a
     * screen edge. The list is also one frame old, which is not observable at
     * interactive rates -- and meshes released in between are skipped by
     * draw_mesh()'s own validity check, so a stale handle is safe rather than a
     * crash.
     */
    void render_shadow_cascades();

    /// The live shadow settings. Assigning reallocates the map if the size or the
    /// cascade count changed.
    void set_shadow_config(const ShadowConfig& config);
    const ShadowConfig& get_shadow_config() const { return m_shadow_config; }

    /// True when a shadow map exists and cascades were rendered into it.
    [[nodiscard]] bool shadows_active() const {
        return m_shadow_config.enabled && m_shadow_texture != nullptr &&
               m_shadow_pipeline != nullptr;
    }

    /**
     * @brief Paint the analytic sky over the whole viewport
     *
     * Call once per frame INSIDE the 3D render pass and BEFORE any geometry, and
     * after set_view_projection() and set_camera_position(): the pass reconstructs
     * a world ray per pixel from the inverse view-projection, so a stale matrix
     * points the sky in the wrong direction.
     *
     * Draws nothing in Simple shader mode -- mesh.frag has no scene uniforms and
     * no sky to be consistent with, so a sky behind it would be a sky lighting
     * nothing.
     */
    void draw_sky();

    /**
     * @brief Set the analytic sky, which is also the ambient light
     *
     * These are scene-referred radiances, not display colours; exposure and the
     * tone curve bring them to the screen. There is deliberately no separate
     * "ambient colour": the fill light IS this sky, integrated over the
     * hemisphere in the shader.
     *
     * @param zenith          Radiance straight up
     * @param horizon         Radiance at the horizon
     * @param ground          Ground bounce radiance, the lower ambient lobe
     * @param sky_intensity   Master scale on the dome
     * @param ground_intensity Master scale on the bounce
     * @param falloff         Zenith falloff exponent; smaller keeps the bright
     *                        band tighter to the horizon
     */
    void set_sky(const glm::vec3& zenith, const glm::vec3& horizon, const glm::vec3& ground,
                 float sky_intensity, float ground_intensity, float falloff);

    /**
     * @brief Set the image-based lighting and sun-disk parameters
     *
     * @param specular_scale    Scale on the ambient specular lobe, 1.0 = physical
     * @param sun_angular_deg   Drawn sun disk diameter in degrees; the real sun is
     *                          about 0.53
     * @param aerial_perspective 0 keeps the authored fog colour, 1 fades distance
     *                          into the sky along the view ray
     * @param sun_glow_exponent Haze tightness around the sun; larger is tighter
     */
    void set_ibl_params(float specular_scale, float sun_angular_deg,
                        float aerial_perspective, float sun_glow_exponent);

    const glm::vec4& get_sky_zenith() const { return m_scene_uniforms.sky_zenith; }
    const glm::vec4& get_sky_horizon() const { return m_scene_uniforms.sky_horizon; }
    const glm::vec4& get_ground_color() const { return m_scene_uniforms.ground_color; }
    const glm::vec4& get_ibl_params() const { return m_scene_uniforms.ibl_params; }

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
        /// Material binds actually issued. Compare against draw_calls: a value
        /// approaching it means the submesh ranges are not sorted by material and
        /// every range is paying for a uniform push and a sampler bind.
        uint32_t material_binds = 0;
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

    // set_pbr_params()/get_pbr_params() used to live here, driving
    // SceneUniforms::pbr_params from three Render Settings sliders. They have been
    // REMOVED, not deprecated: mesh_pbr.frag reads metallic, roughness and ao from
    // the MATERIAL block now, and never touches scene.pbr_params -- the compiled
    // module does not contain a single access to member 5 of the scene struct, as
    // the MaterialUniforms suite asserts. The sliders moved no pixel at any value.
    // The uniform FIELD stays in SceneUniforms because it is part of the block's
    // std140 layout; see its declaration.

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
     * @brief Draw a mesh with the given transform, one draw per material range
     *
     * The mesh's SubMesh ranges tile its index buffer, and each range is bound to
     * its own material before being drawn -- see bind_material(). A mesh with no
     * ranges is one implicit range of @p default_material.
     *
     * @param mesh_id    Mesh handle from upload_mesh()
     * @param model      Model transform matrix
     * @param color_tint Optional color tint (default white = no tint). Multiplied
     *                   into the material's base_color, not a replacement for it.
     * @param default_material Material for geometry that carries no tag of its own
     *
     * @par The parameter this replaces
     * draw_mesh() used to take `uint32_t material_id = 0`, which was threaded
     * through the whole call chain and never read by anything: no shader had a
     * material index, no material table existed, and every caller left it at 0.
     * It is REPLACED rather than deleted because there is a real need behind it --
     * terrain, water and building meshes are produced by builders that predate
     * MaterialId and emit no submeshes at all, so without a per-draw default they
     * would every one of them draw as MaterialId::Default grey. A MaterialKey is
     * the honest form of what the uint32_t was pretending to be.
     *
     * @p default_material substitutes in exactly two places, and nowhere else:
     * for the implicit whole-mesh range of a mesh with no submeshes, and for an
     * explicit range whose key is {MaterialId::Default, 0}. A range that was
     * deliberately tagged always wins over it.
     *
     * @note Materials are only bound in ShaderMode::PBR. The simple shader has no
     *       samplers and no material uniform block, so in ShaderMode::Simple this
     *       behaves exactly as before: every range draws with the bound pipeline.
     */
    void draw_mesh(uint32_t mesh_id, const glm::mat4& model = glm::mat4(1.0f),
                   const glm::vec4& color_tint = glm::vec4(1.0f),
                   MaterialKey default_material = MaterialKey{});

    // === Materials ===

    /**
     * @brief Install the material library draw_mesh() resolves keys against
     *
     * Not owned; must outlive the renderer or be cleared with nullptr first.
     * With no library installed, materials are not bound at all and drawing is
     * unchanged from before this phase -- which is also the fallback if
     * initialisation of the material system fails, so a broken material set
     * degrades to the old untextured look instead of to a black screen.
     */
    void set_material_library(MaterialLibrary* library);

    /// The installed library, or nullptr.
    [[nodiscard]] MaterialLibrary* material_library() const { return m_material_library; }

    /**
     * @brief Install the texture manager whose handles the materials name
     *
     * Not owned. Also gives the renderer the manager to drive: its staged texture
     * copies are flushed inside the renderer's existing per-frame copy pass, and
     * its retired transfer buffers are drained alongside the renderer's own
     * retired buffer ranges. See GPUTextureManager's class note.
     */
    void set_texture_manager(GPUTextureManager* textures);

    /// The installed texture manager, or nullptr.
    [[nodiscard]] GPUTextureManager* texture_manager() const { return m_texture_manager; }

    /**
     * @brief Bind the material for a key: push its uniforms and bind its samplers
     *
     * Called per submesh range from draw_mesh(). Resolves @p key through
     * MaterialLibrary::resolve(), pushes MaterialUniforms to fragment uniform slot
     * kMaterialUniformSlot, and binds albedo, normal and ORM as one
     * SDL_BindGPUFragmentSamplers call at first slot kAlbedoSamplerSlot.
     *
     * Redundant binds are skipped: a mesh whose ranges are already sorted by
     * material (Mesh::sort_submeshes_by_material) issues at most one bind per
     * distinct material, and consecutive meshes sharing a material issue none.
     * The cached key is invalidated whenever a render pass opens, because SDL_GPU
     * bindings do not survive a pass.
     *
     * Switching between an opaque material and one that needs the decal pipeline
     * rebinds the pipeline, since blending and depth bias are pipeline state in
     * SDL_GPU and cannot be changed by a bind.
     *
     * @param key (slot, variant) to bind. Resolution never fails.
     *
     * @note No-op outside a render pass and in ShaderMode::Simple. With materials
     *       disabled or no library installed it binds the NEUTRAL set instead of
     *       doing nothing -- see @ref MaterialBindMode.
     */
    void bind_material(MaterialKey key);

    /**
     * @brief What bind_material() must do for a given renderer state
     *
     * Split out of bind_material() because getting it wrong is a HARD CRASH rather
     * than a wrong pixel, and because a pure function of five booleans can be
     * tested without a device, a window or a render pass.
     *
     * The PBR fragment shader is created with num_samplers = kMaterialSamplerCount
     * and num_uniform_buffers = kPbrFragmentUniformBufferCount. SDL takes those
     * counts at their word and builds a pipeline layout from them, so EVERY draw
     * through the PBR pipeline must leave all three sampler slots and both uniform
     * slots written. SDL's own frontend enforces it:
     * SDL_GPU_CheckGraphicsBindings() fires
     * `SDL_assert_release(!"Missing fragment sampler binding!")` once per unbound
     * slot per draw, under `debug_mode`, which GPURenderer::init() passes as a
     * literal true in every build -- and SDL_assert_release survives NDEBUG. The
     * Vulkan backend behind it writes VK_NULL_HANDLE image views into a
     * COMBINED_IMAGE_SAMPLER descriptor, which is invalid Vulkan.
     *
     * So "materials are off" can never mean "bind nothing" while the PBR pipeline
     * is bound. It means bind the NEUTRAL set: a default-constructed
     * MaterialUniforms and the texture manager's three built-in 1x1 maps.
     */
    enum class MaterialBindMode : uint8_t {
        Skip,       ///< Not the PBR pipeline; the simple shader declares no material resources
        Neutral,    ///< PBR pipeline bound but no material to resolve: bind the built-in set
        Full        ///< Resolve the key and bind its material
    };

    /**
     * @brief Decide the bind mode for a renderer state
     *
     * @param mode                    Current shader mode
     * @param pbr_pipeline_available  Whether the PBR pipelines AND a texture
     *                                manager exist, i.e. GPURenderer::pbr_path_available()
     * @param materials_enabled       The set_materials_enabled() debug toggle
     * @param has_library             Whether a MaterialLibrary is installed
     * @return The mode bind_material() must take
     */
    [[nodiscard]] static constexpr MaterialBindMode material_bind_mode(
            ShaderMode mode, bool pbr_pipeline_available, bool materials_enabled,
            bool has_library) {
        if (mode != ShaderMode::PBR || !pbr_pipeline_available) return MaterialBindMode::Skip;
        if (!materials_enabled || !has_library) return MaterialBindMode::Neutral;
        return MaterialBindMode::Full;
    }

    /**
     * @brief Whether the PBR path can be drawn at all
     *
     * Both PBR pipelines AND a texture manager. The texture manager is part of the
     * answer because the PBR fragment shader declares three samplers that SOMETHING
     * must fill on every draw, and with no manager there is not one legal texture
     * to bind -- so PBR is refused rather than entered and crashed out of. This is
     * the single predicate behind set_shader_mode(), bind_mesh_pipeline(),
     * draw_mesh()'s uniform layout choice and bind_material(), so they cannot
     * disagree about which pipeline is bound.
     */
    [[nodiscard]] bool pbr_path_available() const {
        return m_pbr_pipeline != nullptr && m_pbr_pipeline_wireframe != nullptr
            && m_texture_manager != nullptr;
    }

    /// Whether this frame's meshes are drawing through the PBR pipeline.
    [[nodiscard]] bool using_pbr() const {
        return m_current_shader_mode == ShaderMode::PBR && pbr_path_available();
    }

    /**
     * @brief The depth-bias factors a decal pipeline gets for an authored bias
     *
     * @warning The SIGN is set by REVERSE-Z and both terms must carry it. The depth
     *          test is GREATER and the near plane is at depth 1, so "towards the
     *          viewer" is a LARGER depth value. MaterialDef::depth_bias is authored
     *          negative for that reason (kMarkingDepthBias = -2) and is negated
     *          here. The SLOPE factor was left at a hardcoded -1 while the constant
     *          was negated, so the two terms opposed -- and the slope term is the
     *          larger by two orders of magnitude at exactly the grazing angles the
     *          bias exists for (Vulkan computes `o = m * slope + r * constant`;
     *          with D32_FLOAT, r is ~5e-10 while m is ~4.5e-5 for a road surface
     *          seen near ground level). The result was markings pushed AWAY from
     *          the camera and occluded by their own carriageway beyond ~13 m,
     *          strictly worse than no bias at all.
     *
     * @param authored MaterialDef::depth_bias, negative to pull towards the camera
     * @return {constant, slope}, both in SDL_GPURasterizerState's units
     */
    struct DecalDepthBias {
        float constant = 0.0f;
        float slope = 0.0f;
    };
    [[nodiscard]] static constexpr DecalDepthBias decal_depth_bias(float authored) {
        if (authored == 0.0f) return DecalDepthBias{ 0.0f, 0.0f };
        const float sign = (authored < 0.0f) ? 1.0f : -1.0f;
        return DecalDepthBias{ -authored, sign };
    }

    /**
     * @brief Cache key for a decal pipeline's depth bias
     *
     * Quantised to 1/16 over the panel's [-16, 16] range so a slider DRAG cannot
     * mint a pipeline per pixel of mouse travel, and so two biases that differ
     * below the quantum share one pipeline instead of two identical ones.
     *
     * @param depth_bias MaterialDef::depth_bias
     * @return A key; equal keys mean an interchangeable pipeline
     */
    [[nodiscard]] static uint32_t decal_bias_key(float depth_bias);

    /// Quantised bias a decal_bias_key() names. The value actually baked in.
    [[nodiscard]] static float decal_bias_from_key(uint32_t key);

    /// Largest depth-bias magnitude a decal pipeline is built for. Matches the
    /// Materials panel's DragFloat range.
    static constexpr float kMaxDecalDepthBias = 16.0f;

    /// Distinct decal pipelines kept alive at once. A drag past this many distinct
    /// quantised values falls back to the markings pipeline and says so once.
    static constexpr size_t kMaxDecalPipelines = 32;

    /**
     * @brief Turn material binding off without tearing the library down
     *
     * A debug toggle for the stats panel: with materials disabled every range
     * draws with the NEUTRAL material -- a default MaterialUniforms and the three
     * built-in 1x1 maps -- which is the closest legal thing to the pre-material
     * look and therefore the A/B a bug report needs. Enabled by default.
     *
     * @note It cannot mean "bind nothing". The PBR pipeline declares three
     *       fragment samplers and two fragment uniform buffers whatever this flag
     *       says; leaving them unwritten aborts the process inside
     *       SDL_DrawGPUIndexedPrimitives. See @ref MaterialBindMode.
     */
    void set_materials_enabled(bool enabled);

    /// Whether material binding is active. False also when no library is installed.
    [[nodiscard]] bool materials_enabled() const;

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
     * @param num_samplers Combined image samplers declared by the shader
     * @return Shader handle, or nullptr on failure (caller owns it)
     *
     * The counts are not advisory: SDL validates them against the SPIR-V and
     * SDL_CreateGPUShader fails outright if a shader declares more resources than
     * it is told about. mesh_pbr.frag now declares two fragment uniform buffers
     * (scene at set 3 binding 0, material at set 3 binding 1) and three fragment
     * samplers (set 2 bindings 0-2), so it must be loaded with
     * kPbrFragmentUniformBufferCount and kMaterialSamplerCount from
     * material_library.hpp rather than with literals -- those constants are the
     * ones the GLSL is documented against.
     *
     * @p num_samplers defaults to 0, so the existing simple-shader and Im3d call
     * sites need no change.
     *
     * @note Public so auxiliary backends (e.g. the Im3d backend) can reuse it.
     */
    SDL_GPUShader* load_shader(const char* path, SDL_GPUShaderStage stage,
                                int num_uniform_buffers, int num_storage_buffers,
                                int num_samplers = 0);

private:
    bool create_pipelines();
    bool create_simple_pipelines();
    bool create_pbr_pipelines();

    /**
     * @brief Build the decal variant of the PBR pipeline
     *
     * Identical to the PBR pipeline except for alpha blending
     * (SRC_ALPHA / ONE_MINUS_SRC_ALPHA, depth writes off, depth test on) and a
     * constant depth bias.
     *
     * It exists because MaterialDef::alpha_blend and MaterialDef::depth_bias are
     * PIPELINE state in SDL_GPU -- SDL_GPUColorTargetBlendState and
     * SDL_GPURasterizerState are baked into SDL_CreateGPUGraphicsPipeline and
     * there is no command to change either inside a render pass. A material system
     * that treats them as per-draw values would silently draw every marking opaque
     * and z-fighting. Two pipelines, switched by
     * MaterialDef::needs_decal_pipeline(), is the whole mechanism.
     *
     * @note MSAA rebuilds pipelines, so this one is created and released with the
     *       rest of the PBR set, never separately.
     */
    /// @param depth_bias Authored MaterialDef::depth_bias this pipeline bakes in
    /// @return The pipeline, or nullptr after logging
    SDL_GPUGraphicsPipeline* create_decal_pipeline(float depth_bias);

    /**
     * @brief The decal pipeline for an authored depth bias, created on demand
     *
     * MaterialDef::depth_bias is PIPELINE state, so honouring an edited value
     * means a pipeline per distinct value rather than a uniform. The cache is
     * keyed on decal_bias_key() and capped at kMaxDecalPipelines; the markings
     * bias is created eagerly with the rest of the PBR set so that a failure is
     * reported at init rather than on the first marking drawn.
     *
     * @param depth_bias Authored bias, negative to pull towards the camera
     * @return A pipeline, or nullptr if none could be built
     */
    SDL_GPUGraphicsPipeline* decal_pipeline_for(float depth_bias);

    bool load_shaders();
    bool load_simple_shaders();
    bool load_pbr_shaders();
    bool load_sky_shaders();
    bool create_sky_pipeline();
    bool load_shadow_shaders();
    bool create_shadow_pipeline();
    bool create_shadow_resources();
    void release_shadow_resources();
    void update_shadow_cascades();
    void bind_shadow_resources();
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

    // ------------------------------------------------------------------------
    // Cascaded shadow map
    // ------------------------------------------------------------------------

    /// D32_FLOAT 2D ATLAS, `cascade_count` square tiles side by side, so its width
    /// is map_size * cascade_count and its height is map_size.
    ///
    /// An array texture would be the natural shape and is not available: SDL_GPU
    /// rejects array textures with DEPTH_STENCIL_TARGET usage. The atlas is the
    /// better shape anyway -- every cascade is filled in ONE render pass with a
    /// single depth clear and a viewport change per tile.
    SDL_GPUTexture* m_shadow_texture = nullptr;

    /// Comparison sampler: enable_compare with COMPAREOP_LESS, so each texture()
    /// call in the PCF kernel returns a hardware-filtered comparison result
    /// rather than a depth value to compare by hand.
    SDL_GPUSampler* m_shadow_sampler = nullptr;

    SDL_GPUGraphicsPipeline* m_shadow_pipeline = nullptr;
    SDL_GPUShader* m_shadow_vertex_shader = nullptr;
    SDL_GPUShader* m_shadow_fragment_shader = nullptr;

    ShadowConfig m_shadow_config{};
    ShadowUniforms m_shadow_uniforms{};

    /// Tile size and tile count the current m_shadow_texture was created with, so
    /// set_shadow_config() knows whether it has to reallocate.
    uint32_t m_shadow_allocated_size = 0;
    int m_shadow_allocated_layers = 0;

    /// One recorded draw_mesh() call. Material and colour tint are deliberately
    /// absent: the depth pass has no use for either.
    struct CapturedDraw {
        uint32_t mesh_id = 0;
        glm::mat4 model{1.0f};
    };

    /// Filled by draw_mesh() during the frame currently being recorded.
    std::vector<CapturedDraw> m_shadow_casters_recording;

    /// The completed recording from the previous frame, which is what
    /// render_shadow_cascades() replays. begin_frame() moves one into the other.
    std::vector<CapturedDraw> m_shadow_casters_ready;

    /**
     * @brief Fullscreen analytic sky, drawn before any geometry
     *
     * No vertex buffer and no depth interaction: three generated vertices, depth
     * test and depth write both off. It runs first and everything else paints
     * over it, which is why it needs neither.
     *
     * Created and released with the PBR set because it shares the swapchain
     * format and the MSAA sample count, so a set_msaa_level() that rebuilt one
     * and not the other would leave a pipeline whose sample count no longer
     * matches its render pass.
     */
    SDL_GPUGraphicsPipeline* m_sky_pipeline = nullptr;
    SDL_GPUShader* m_sky_vertex_shader = nullptr;
    SDL_GPUShader* m_sky_fragment_shader = nullptr;

    // PBR shader pipelines
    SDL_GPUGraphicsPipeline* m_pbr_pipeline = nullptr;
    SDL_GPUGraphicsPipeline* m_pbr_pipeline_wireframe = nullptr;
    SDL_GPUShader* m_pbr_vertex_shader = nullptr;
    SDL_GPUShader* m_pbr_fragment_shader = nullptr;

    /// Alpha-blended, depth-biased variants of m_pbr_pipeline, one per distinct
    /// quantised MaterialDef::depth_bias. See decal_pipeline_for(). Owns every
    /// pipeline in it; release_pipelines() is the only place they are destroyed.
    std::unordered_map<uint32_t, SDL_GPUGraphicsPipeline*> m_decal_pipelines;

    /// The decal pipeline for MaterialLibrary::kMarkingDepthBias, built eagerly
    /// with the rest of the PBR set and also owned by m_decal_pipelines. Non-null
    /// is what "the decal path is available" means.
    SDL_GPUGraphicsPipeline* m_pbr_pipeline_decal = nullptr;

    /// Whether the "too many decal pipelines" warning has already been logged.
    bool m_decal_cap_warned = false;

    // === Material state ===

    MaterialLibrary* m_material_library = nullptr;   ///< Not owned
    GPUTextureManager* m_texture_manager = nullptr;  ///< Not owned

    /// Debug toggle behind set_materials_enabled().
    bool m_materials_enabled = true;

    /**
     * @brief The material currently bound, for redundant-bind elimination
     *
     * Valid only while m_material_bound is true. Both are cleared by
     * reset_material_binding() whenever a render pass opens or the pipeline is
     * rebound, because SDL_GPU bindings do not survive either and a stale cache
     * would skip the bind that the new pass needs.
     */
    MaterialKey m_bound_material{};
    bool m_material_bound = false;

    /// True when the last bind was the NEUTRAL set rather than a resolved key, so
    /// that re-enabling materials cannot be skipped by a cached MaterialKey{} that
    /// happens to equal the next key asked for.
    bool m_neutral_material_bound = false;

    /// The pipeline bind_mesh_pipeline() or bind_material() last bound, so a
    /// switch between the opaque pipeline and one of several decal pipelines is
    /// decided by identity rather than by a boolean that cannot tell two decal
    /// pipelines apart.
    SDL_GPUGraphicsPipeline* m_bound_pipeline = nullptr;

    /// Bind the built-in neutral material. See @ref MaterialBindMode.
    void bind_neutral_material();

    /// Forget the cached material bind. Called when a render pass opens.
    void reset_material_binding();

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
    // Guards the one-time default seeding in update_scene_uniforms(). A separate
    // flag, not a sentinel value inside m_scene_uniforms, so that every field of
    // the light block stays a legal value the user may choose -- zero included.
    bool m_scene_lighting_seeded{false};

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
     * @brief Device bytes held by the installed texture manager, 0 if none
     *
     * Deliberately SEPARATE from resident_bytes() rather than added into it.
     * resident_bytes() is not merely a stat: it is the quantity evict_to_budget()
     * drives against MemoryBudget::max_resident_bytes by releasing MESHES.
     * Folding texture bytes into it would let a large material set push the
     * renderer over budget and make it evict geometry to reclaim bytes that no
     * amount of mesh eviction can free -- geometry would stream out, the number
     * would not move, and the renderer would thrash until the budget warning
     * fired every 300 frames. The panel that wants a single VRAM figure should
     * add the two itself.
     */
    [[nodiscard]] size_t texture_bytes() const;

    /// Live texture count in the installed manager, including its four fallbacks.
    [[nodiscard]] size_t texture_count() const;

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
