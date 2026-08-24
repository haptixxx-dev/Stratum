/**
 * @file texture.hpp
 * @brief GPU texture ownership, handle table, shared samplers, and the fallback set
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The renderer had no texture path at all before this file: the only
 * SDL_CreateGPUTexture calls in the project are the depth and MSAA targets, and
 * there was not a single SDL_GPUSampler anywhere. Road geometry has carried
 * metre-based tiling UVs since P2 and material slots since P0.3, and none of it
 * reached a shader. This is the missing half.
 *
 * ### What this file owns, and what it deliberately does not
 *
 * GPUTextureManager owns SDL_GPUTexture objects, the handle table that names
 * them, and the small fixed set of samplers every material shares. It does NOT
 * own materials -- see material_library.hpp -- and it does NOT own a command
 * buffer. Uploads are STAGED here and RECORDED into a copy pass the caller
 * supplies, because GPURenderer already batches every copy of a frame into one
 * command buffer, one copy pass and one submit, and a texture path that opened
 * its own command buffer would reintroduce exactly the fence exhaustion that
 * batching was written to fix (see GPURenderer::flush_pending_uploads).
 *
 * Everything here is stratum_editor_lib. It includes SDL and must never be
 * included from stratum_core.
 */

#pragma once

#include <SDL3/SDL.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace stratum {

// ============================================================================
// Formats and descriptions
// ============================================================================

/**
 * @brief Pixel format of a texture, in the small set this renderer actually uses
 *
 * The sRGB distinction is not cosmetic and is the one that is silently wrong when
 * it is wrong. Albedo is authored in sRGB and MUST be sampled through an sRGB
 * format so the hardware linearises it before lighting; normal maps and ORM packs
 * are DATA, not colour, and MUST be linear or every roughness and every normal is
 * gamma-skewed by 2.2 with no visible error to point at.
 *
 * @note RGBA8_SRGB leaves the ALPHA channel linear, which is what the marking
 *       atlas needs: its alpha is coverage, not colour.
 */
enum class TextureFormat : uint8_t {
    RGBA8 = 0,      ///< 8-bit RGBA, linear. Normal maps, ORM packs, masks.
    RGBA8_SRGB,     ///< 8-bit RGBA, sRGB-encoded RGB with linear alpha. Albedo.
    BC7,            ///< Block-compressed colour, LINEAR. Loaded from KTX2, never generated here.
    BC7_SRGB,       ///< Block-compressed colour, sRGB-encoded. Compressed albedo.
    BC5_Normal,     ///< Two-channel block-compressed normal (RG); Z reconstructed in shader.
    R8              ///< Single linear channel. Height fields, coverage masks.
};

/**
 * @brief Whether a format stores 4x4 blocks rather than individual texels
 *
 * Block-compressed levels have no per-texel size, cannot be downsampled on the
 * CPU by this file, and are only ever supplied whole by a KTX2 file.
 */
[[nodiscard]] constexpr bool is_block_compressed(TextureFormat format) {
    return format == TextureFormat::BC7
        || format == TextureFormat::BC7_SRGB
        || format == TextureFormat::BC5_Normal;
}

/**
 * @brief Whether the hardware must linearise this format's colour channels on read
 *
 * The alpha channel of an sRGB format is NOT transformed, which is what lets the
 * procedural generators pack a linear height field into the alpha of an sRGB
 * albedo map.
 */
[[nodiscard]] constexpr bool is_srgb_format(TextureFormat format) {
    return format == TextureFormat::RGBA8_SRGB || format == TextureFormat::BC7_SRGB;
}

/**
 * @brief Everything needed to create a texture, independent of its pixels
 *
 * @note @ref mip_levels is the number of levels the texture is CREATED with, not
 *       the number supplied. A generator returns level 0 only and asks for a full
 *       chain; the uploader BUILDS the missing levels on the CPU and uploads every
 *       one of them.
 *
 * @par Why the chain is built on the CPU
 * The obvious call is SDL_GenerateMipmapsForGPUTexture(), and it cannot be used
 * here. It takes a COMMAND BUFFER and must not be called inside any pass, but
 * flush_uploads() is handed an already-open copy pass on a command buffer
 * GPURenderer owns -- so reaching the API would mean either breaking out of the
 * frame's single copy pass or opening a second command buffer per texture, which
 * is exactly the per-resource submit that flush_pending_uploads() exists to
 * prevent. It would also force SDL_GPU_TEXTUREUSAGE_COLOR_TARGET onto every
 * sampled texture, because SDL implements generation as a chain of blits.
 * A box filter over the staging arena costs a few milliseconds once at startup,
 * needs no extra usage flag, and is the same on every backend.
 */
struct TextureDesc {
    uint32_t width = 0;                                 ///< Pixels
    uint32_t height = 0;                                ///< Pixels
    uint32_t mip_levels = 1;                            ///< 1 = no chain; see note above
    uint32_t layers = 1;                                ///< Array layers; 1 for every current use
    TextureFormat format = TextureFormat::RGBA8_SRGB;   ///< Interpretation of the bytes

    /// Bytes of level 0 for one layer, ignoring block compression. Zero for a
    /// block-compressed format, whose size the loader takes from the KTX2 file.
    [[nodiscard]] size_t level0_bytes() const {
        const size_t texels = static_cast<size_t>(width) * static_cast<size_t>(height);
        switch (format) {
            case TextureFormat::RGBA8:
            case TextureFormat::RGBA8_SRGB: return texels * 4u;
            case TextureFormat::R8:         return texels;
            case TextureFormat::BC7:
            case TextureFormat::BC7_SRGB:
            case TextureFormat::BC5_Normal: return 0u;
        }
        return 0u;
    }

    [[nodiscard]] bool is_valid() const {
        return width > 0 && height > 0 && layers > 0 && mip_levels > 0;
    }
};

/**
 * @brief Handle into GPUTextureManager
 *
 * 0 is the "missing texture" sentinel, which always resolves to a visible magenta
 * checker rather than to nothing -- a silently untextured surface is far harder to
 * diagnose than an obviously wrong one. Every accessor honours that: get(0) returns
 * the checker's SDL_GPUTexture, not nullptr, so a caller that forgot to check still
 * binds something legal and the error is on screen instead of in a validation log.
 *
 * @note Handles are dense slot indices and ARE reused after release(). A handle
 *       kept across a release may therefore name a different texture. Materials
 *       hold handles for the lifetime of the library, so this does not arise in
 *       practice; anything that caches one longer must re-resolve after a release.
 */
using TextureHandle = uint32_t;

/// The missing-texture sentinel. Never a load failure you can ignore.
inline constexpr TextureHandle kInvalidTexture = 0;

// ============================================================================
// Samplers
// ============================================================================

/**
 * @brief The shared sampler set
 *
 * Samplers are few and shared; materials reference one by enum rather than owning
 * it. SDL_GPU binds a sampler together with its texture in one
 * SDL_GPUTextureSamplerBinding, so this enum is resolved at bind time by
 * GPUTextureManager::sampler().
 *
 * | Kind         | Address mode | Min/Mag | Mip    | Anisotropy | Used by |
 * |--------------|--------------|---------|--------|------------|---------|
 * | RepeatAniso  | REPEAT       | LINEAR  | LINEAR | 8x if supported | every tiling surface |
 * | ClampLinear  | CLAMP_TO_EDGE| LINEAR  | LINEAR | off        | the markings atlas |
 * | RepeatPoint  | REPEAT       | NEAREST | NEAREST| off        | debug and checker views |
 *
 * ClampLinear exists specifically for MaterialId::Markings. An atlas sub-rect
 * cannot wrap: a REPEAT sampler on a UV that drifts a texel past the rect samples
 * a NEIGHBOURING SPRITE, which is why marking_atlas.hpp insets every rect and why
 * the atlas must never be sampled with RepeatAniso.
 */
enum class SamplerKind : uint8_t {
    RepeatAniso = 0,    ///< Tiling surfaces: asphalt, kerb, sidewalk, verge
    ClampLinear,        ///< Atlased sprites; must not wrap across sub-rects
    RepeatPoint,        ///< Unfiltered; debug views and the magenta checker
    Count               ///< Sentinel: number of sampler kinds. Not a valid kind.
};

/**
 * @brief Stable human-readable name of a sampler kind
 * @param kind Kind to name
 * @return Its name, or "Unknown" for SamplerKind::Count and out-of-range values
 */
[[nodiscard]] const char* sampler_kind_name(SamplerKind kind);

// ============================================================================
// GPUTextureManager
// ============================================================================

/**
 * @brief Owns every SDL_GPUTexture and SDL_GPUSampler the material system binds
 *
 * ### Upload discipline -- read this before adding a load path
 *
 * GPURenderer stages mesh copies into a CPU arena and moves a budgeted prefix of
 * them through ONE transfer buffer, ONE copy pass and ONE submit at the top of
 * begin_frame(). Textures join that batch rather than bypassing it:
 *
 * 1. load() and create() COPY the pixels into this manager's own staging arena,
 *    create the SDL_GPUTexture immediately, and return a handle. The texture is
 *    not readable yet: is_ready() is false and bind_texture() substitutes the
 *    fallback until the copy has run.
 * 2. GPURenderer calls flush_uploads() from inside its existing copy pass, which
 *    records SDL_UploadToGPUTexture for a budgeted prefix of the staged entries.
 *    Leftovers stay staged and flush next frame.
 * 3. Transfer buffers are RETIRED, not released, for kTransferRetireFrames frames
 *    -- the same argument as GPURenderer::retire_alloc(): releasing a transfer
 *    buffer a submitted-but-unfinished command buffer is still reading from is a
 *    use-after-free the validation layer will not always catch.
 *
 * The ONE exception is init(). The four built-in fallbacks are a handful of
 * kilobytes, and every later upload depends on them already being samplable, so
 * init() uploads them on its own command buffer and blocks on
 * SDL_WaitForGPUIdle() once, at startup, before any frame exists. That is a
 * deliberate, documented, single-occurrence stall, not a precedent.
 *
 * ### Threading
 *
 * Not thread-safe. Every method must be called from the thread that owns the
 * SDL_GPUDevice, which in this application is the main thread. Pixel GENERATION
 * (procedural_texture.hpp) is pure and may run anywhere; only the handoff here is
 * constrained.
 */
class GPUTextureManager {
public:
    GPUTextureManager() = default;
    ~GPUTextureManager();

    GPUTextureManager(const GPUTextureManager&) = delete;
    GPUTextureManager& operator=(const GPUTextureManager&) = delete;

    /**
     * @brief Create the samplers and the built-in fallback textures
     *
     * Uploads the fallbacks synchronously; see the class note on why this one
     * path is allowed its own submit. After a successful call, missing_texture(),
     * white(), flat_normal() and default_orm() are all valid and ready.
     *
     * @param device Live SDL_GPU device. Not owned; must outlive this manager.
     * @return true on success. On failure nothing is left allocated.
     */
    bool init(SDL_GPUDevice* device);

    /**
     * @brief Release every texture, sampler, transfer buffer and staged byte
     *
     * Safe to call twice and safe to call on an uninitialised manager. The caller
     * must have made the device idle first, exactly as GPURenderer::shutdown()
     * does before draining its retirement queue.
     */
    void shutdown();

    /**
     * @brief Load a KTX2 or PNG/JPG file
     *
     * KTX2 via libktx (vendored, already linked into stratum_editor_lib), others
     * via stb_image (vendored). The extension chooses the reader; a KTX2 file
     * keeps whatever compressed format and mip chain it was authored with, an
     * stb-decoded file is expanded to 8-bit RGBA and given a generated chain.
     *
     * @param path File to load. Relative paths resolve against the process CWD;
     *             MaterialLibrary resolves its own paths before calling.
     * @param srgb true to interpret the colour channels as sRGB. Pass FALSE for
     *             normal maps and ORM packs -- an sRGB normal map is wrong in a
     *             way that looks merely "a bit off" rather than broken.
     * @return A handle, or kInvalidTexture on failure AFTER logging the reason
     *         (path, reader, and the reader's own error text). A failure is
     *         counted in Stats::load_failures; it is never silent.
     */
    [[nodiscard]] TextureHandle load(const std::filesystem::path& path, bool srgb = true);

    /**
     * @brief Upload raw pixels
     *
     * Used for procedurally generated textures (procedural_texture.hpp) and for
     * the built-in fallbacks. The bytes are copied into the staging arena before
     * returning, so @p pixels need not outlive the call.
     *
     * @param desc      Dimensions and format. desc.mip_levels > 1 asks for a
     *                  generated chain from the supplied level 0.
     * @param pixels    Tightly packed level 0, or nullptr to create the texture
     *                  with undefined contents (only useful for render targets,
     *                  which this manager does not currently own).
     * @param byte_size Size of @p pixels. Must be at least desc.level0_bytes()
     *                  for an uncompressed format; a smaller buffer is refused
     *                  rather than read past, because the truncated read is a
     *                  heap overrun that usually survives long enough to corrupt
     *                  something else.
     * @return A handle, or kInvalidTexture on failure after logging.
     */
    [[nodiscard]] TextureHandle create(const TextureDesc& desc, const void* pixels, size_t byte_size);

    /**
     * @brief Give up a texture
     *
     * Releasing kInvalidTexture or one of the built-in fallbacks is a no-op: the
     * fallbacks must stay valid for the manager's whole lifetime, since every
     * unbound material map resolves to one. The slot is queued for reuse.
     *
     * @param handle Handle to release. An unknown handle is a no-op.
     */
    void release(TextureHandle handle);

    /**
     * @brief The SDL texture behind a handle
     *
     * @param handle Handle to look up
     * @return Its texture. NEVER nullptr after init(): an unknown, released or
     *         kInvalidTexture handle returns the magenta checker, so a caller
     *         cannot accidentally bind nothing.
     */
    [[nodiscard]] SDL_GPUTexture* get(TextureHandle handle) const;

    /**
     * @brief Whether a handle's pixels have actually reached the GPU
     *
     * False between create()/load() and the flush that copies it. Sampling a
     * not-ready texture reads undefined device memory, which is why
     * bind_texture() substitutes the fallback instead.
     */
    [[nodiscard]] bool is_ready(TextureHandle handle) const;

    /**
     * @brief The texture to actually bind for a handle right now
     *
     * get() for a ready handle; the appropriate fallback otherwise. This is what
     * GPURenderer::bind_material() calls.
     *
     * @param handle   Handle to bind
     * @param fallback Handle to substitute when @p handle is not ready. Pass
     *                 white() for albedo, flat_normal() for normal maps and
     *                 default_orm() for ORM so a half-loaded material is merely
     *                 plain rather than garbage.
     */
    [[nodiscard]] SDL_GPUTexture* bind_texture(TextureHandle handle, TextureHandle fallback) const;

    /// The magenta checker. Always valid after init().
    [[nodiscard]] TextureHandle missing_texture() const;

    /// 1x1 opaque white, sRGB. A material with no albedo map still samples something.
    [[nodiscard]] TextureHandle white() const;

    /// 1x1 (128,128,255,255) linear. Tangent-space "no perturbation".
    [[nodiscard]] TextureHandle flat_normal() const;

    /// 1x1 @ref kDefaultOrmTexel, linear: a true MULTIPLICATIVE IDENTITY in all
    /// three channels, so a material with no ORM map keeps the occlusion,
    /// roughness AND metallic it authored and the shader needs no branch.
    [[nodiscard]] TextureHandle default_orm() const;

    /**
     * @brief The texel of the built-in default ORM map
     *
     * All three channels are 255 because mesh_pbr.frag multiplies every one of
     * them into an authored scalar:
     *
     *     ao        = material.pbr_params.z * orm.r
     *     roughness = material.pbr_params.y * orm.g
     *     metallic  = material.pbr_params.x * orm.b
     *
     * @warning A zero in ANY channel is an annihilator, not an identity. The blue
     *          channel was 0 here, which silently multiplied every material's
     *          authored `metallic` to zero -- so MaterialDef::metallic, the panel's
     *          Metallic slider and a set file's "metallic" field were all no-ops
     *          for every material in the library, because every material binds this
     *          texture. Nothing rendered visibly wrong only because every built-in
     *          surface is deliberately dielectric; the first metallic parapet or
     *          railing would have been unexplainable.
     */
    static constexpr uint8_t kDefaultOrmTexel[4] = { 255, 255, 255, 255 };

    /**
     * @brief The shared sampler for a kind
     * @param kind Kind to resolve
     * @return Its sampler; the RepeatAniso sampler for SamplerKind::Count and
     *         out-of-range values, so a bad enum still binds something legal.
     */
    [[nodiscard]] SDL_GPUSampler* sampler(SamplerKind kind) const;

    // === Batched upload plumbing, driven by GPURenderer ===

    /**
     * @brief Record staged texture copies into a copy pass the caller owns
     *
     * Called from GPURenderer::flush_pending_uploads(), between
     * SDL_BeginGPUCopyPass and SDL_EndGPUCopyPass on the frame's single batch
     * command buffer. Admits at most kMaxUploadBytesPerFrame bytes beyond the
     * first entry; the first entry is always admitted, or a texture larger than
     * the budget would never upload at all.
     *
     * Entries copied here become ready IMMEDIATELY from the CPU's point of view,
     * which is correct: the copy is ordered before every draw in the same command
     * buffer, and the frame's render pass has not opened yet.
     *
     * @param copy_pass    Open copy pass to record into. Nullptr is a no-op.
     * @param frame_index  GPURenderer's frame counter, used to date the retirement.
     */
    void flush_uploads(SDL_GPUCopyPass* copy_pass, uint64_t frame_index);

    /**
     * @brief Settle the batch flush_uploads() recorded, once its submit is known
     *
     * MUST be called after every flush_uploads(), from
     * GPURenderer::flush_pending_uploads(), with the result of
     * SDL_SubmitGPUCommandBuffer(). Readiness is contingent on that submit and
     * cannot be decided inside flush_uploads(), which records into a command
     * buffer it does not own and does not submit.
     *
     * @param submitted true when the command buffer carrying the recorded copies
     *                  was submitted. The staged bytes are then dropped and the
     *                  textures become samplable.
     *                  false when the submit FAILED. The copies never ran, so the
     *                  entries go back on the front of the queue with their staged
     *                  bytes intact and their textures stay not-ready, which is
     *                  exactly what the mesh half of that function already does.
     *                  Without this the slots would claim to be ready forever while
     *                  holding uninitialised device memory, and bind_texture()
     *                  would stop substituting the fallback -- unrecoverable for
     *                  the rest of the session.
     */
    void commit_uploads(bool submitted);

    /**
     * @brief Release transfer buffers whose frames have elapsed
     *
     * Called once per frame from GPURenderer::begin_frame(), alongside
     * drain_retired_allocs().
     *
     * @param frame_index Current frame counter
     * @param force       Release everything regardless of age. Valid ONLY after
     *                    SDL_WaitForGPUIdle(); shutdown() is its one caller.
     */
    void drain_retired_transfers(uint64_t frame_index, bool force = false);

    /// Textures staged but not yet copied. Non-zero means some material is still
    /// binding a fallback.
    [[nodiscard]] size_t pending_upload_count() const;

    /// Max bytes of texture copies admitted per frame. Matches the mesh path's
    /// intent: a stampede of loads becomes a stream instead of a hitch.
    static constexpr size_t kMaxUploadBytesPerFrame = 8u * 1024u * 1024u;

    /**
     * @brief Frames a transfer buffer must sit out before release
     *
     * Must be at least the renderer's maximum frames in flight. SDL_GPU keeps up
     * to three swapchain images in flight, so three is the floor and the value
     * here -- the same constant and the same reason as
     * GPURenderer::kBufferRetireFrames.
     */
    static constexpr uint64_t kTransferRetireFrames = 3;

    /**
     * @brief Size of the magenta checker, and of its squares
     *
     * Small enough to be free, large enough that the checker reads as a checker
     * at any distance rather than as flat magenta.
     */
    static constexpr uint32_t kCheckerSize = 64;
    static constexpr uint32_t kCheckerSquare = 8;

    /// Live counts, for the stats panel and for asserting in tests.
    struct Stats {
        size_t textures = 0;        ///< Live handles, including the four fallbacks
        size_t bytes = 0;           ///< Device bytes, requested sizes only
        size_t load_failures = 0;   ///< load() calls that returned kInvalidTexture
    };

    [[nodiscard]] Stats stats() const;

private:
    /// One entry of the handle table. A released slot keeps its index for reuse.
    struct TextureSlot {
        SDL_GPUTexture* texture = nullptr;
        TextureDesc desc{};
        size_t bytes = 0;
        bool in_use = false;    ///< false for a free slot awaiting reuse
        bool ready = false;     ///< true once its copy has been recorded
        std::string debug_name; ///< Source path or generator name, for logging
    };

    /**
     * @brief One staged texture copy awaiting the next flush
     *
     * EVERY mip level is staged, not just level 0: the chain is built on the CPU
     * before staging (see TextureDesc's note), so a flush is a straight copy of a
     * contiguous run and one SDL_UploadToGPUTexture per level.
     *
     * The levels sit at packed_level_offset() within the run, which pads each one
     * up to kStagingAlignment. That padding is not cosmetic -- a transfer buffer
     * offset feeding vkCmdCopyBufferToImage must be a multiple of both 4 and the
     * format's block size, and a tightly packed R8 or BC chain violates that on
     * the small levels.
     */
    struct PendingTextureUpload {
        TextureHandle handle = kInvalidTexture;
        size_t staging_offset = 0;  ///< Byte offset of level 0's run in m_staging
        size_t bytes = 0;           ///< Bytes of the whole padded run, every level
        uint32_t mip_levels = 1;    ///< Levels staged in that run
    };

    /// A transfer buffer waiting out the frames that may still read it.
    struct RetiredTransfer {
        SDL_GPUTransferBuffer* buffer = nullptr;
        uint64_t retire_after_frame = 0;
    };

    SDL_GPUDevice* m_device = nullptr;

    std::vector<TextureSlot> m_slots;           ///< Index 0 is always the checker
    std::vector<uint32_t> m_free_slots;         ///< Reusable slot indices

    std::vector<uint8_t> m_staging;             ///< CPU arena for staged pixels
    std::vector<PendingTextureUpload> m_pending;

    /// Entries flush_uploads() recorded into a copy pass whose command buffer has
    /// not been submitted yet. Emptied by commit_uploads(), either into readiness
    /// or back onto the front of m_pending. Never non-empty across a frame.
    std::vector<PendingTextureUpload> m_in_flight;

    std::vector<RetiredTransfer> m_retired_transfers;

    SDL_GPUSampler* m_samplers[static_cast<size_t>(SamplerKind::Count)] = {};

    TextureHandle m_missing = kInvalidTexture;
    TextureHandle m_white = kInvalidTexture;
    TextureHandle m_flat_normal = kInvalidTexture;
    TextureHandle m_default_orm = kInvalidTexture;

    size_t m_resident_bytes = 0;
    size_t m_load_failures = 0;

    /// Whether the device reported each block-compressed format sampleable.
    /// Queried once in init(); a KTX2 in an unsupported format is transcoded to
    /// RGBA8 where libktx can, and refused with a named reason where it cannot.
    bool m_supports_bc7 = false;
    bool m_supports_bc7_srgb = false;
    bool m_supports_bc5 = false;

    bool create_samplers();
    bool create_fallbacks();
    TextureHandle allocate_slot();

    /// Log what the device supports and cache it in m_supports_*.
    void query_format_support();

    /// Reserve the padded run for @p desc at the end of m_staging.
    /// @return Byte offset of the run. m_staging does not reallocate afterwards
    ///         until the next reserve, so the caller may write through the returned
    ///         offset for as long as it takes to fill every level.
    size_t reserve_staging(const TextureDesc& desc);

    /// Create the SDL texture for an already-filled staging run and queue it.
    /// @return Its handle, or kInvalidTexture after logging.
    TextureHandle finish_staged(const TextureDesc& desc, size_t staging_offset,
                                std::string debug_name);

    /// KTX2 path. @p srgb is the CALLER's declared intent and overrides the file's
    /// own transfer-function tagging, with a warning when the two disagree.
    TextureHandle load_ktx2(const uint8_t* bytes, size_t size,
                            const std::filesystem::path& path, bool srgb);

    /// PNG/JPG/TGA/BMP path via stb_image. Always expands to 8-bit RGBA and always
    /// builds a full mip chain.
    TextureHandle load_stb(const uint8_t* bytes, size_t size,
                           const std::filesystem::path& path, bool srgb);

    // === Staging layout arithmetic ===
    //
    // Public and static because it is pure: it decides where every mip level of
    // every texture lands, it is the part most likely to be wrong by a few bytes,
    // and a wrong answer here corrupts a neighbouring level rather than failing.
    // Exposed so a test can check it without a device.
public:
    /**
     * @brief Alignment every staged mip level starts on
     *
     * 512 satisfies Vulkan (a multiple of 4 and of the 16-byte BC block size) and
     * also D3D12's 512-byte transfer offset rule, so no backend has to make a
     * realigning temporary copy of the data. The waste is bounded by 512 bytes per
     * level, which is a few kilobytes across a whole material set.
     */
    static constexpr size_t kStagingAlignment = 512;

    /// Full mip chain length for a base size: floor(log2(max(w,h))) + 1.
    [[nodiscard]] static uint32_t full_mip_count(uint32_t width, uint32_t height);

    /// Dimension of @p level of a texture whose base dimension is @p base.
    [[nodiscard]] static uint32_t level_dimension(uint32_t base, uint32_t level);

    /// Tightly packed bytes of one level, block-compressed formats included.
    [[nodiscard]] static size_t level_bytes(TextureFormat format, uint32_t width,
                                            uint32_t height);

    /// Offset of @p level within a staged run, each level padded to kStagingAlignment.
    [[nodiscard]] static size_t packed_level_offset(const TextureDesc& desc, uint32_t level);

    /// Total padded bytes of every level of @p desc.
    [[nodiscard]] static size_t packed_total_bytes(const TextureDesc& desc);
};

} // namespace stratum
