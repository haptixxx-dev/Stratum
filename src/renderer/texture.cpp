/**
 * @file texture.cpp
 * @brief GPU texture loading, CPU mip generation, batched upload, and the fallback set
 *
 * This is the first texture path in the project. Before it, the only
 * SDL_CreateGPUTexture calls were the depth and MSAA targets and there was not a
 * single SDL_GPUSampler anywhere, so there is no prior art in this repository to
 * follow -- the discipline it follows instead is GPURenderer's, whose mesh
 * uploads are batched into ONE command buffer, ONE copy pass and ONE submit per
 * frame because submitting per resource from inside a frame exhausted the
 * driver's fence pool (vkCreateFence VK_ERROR_OUT_OF_HOST_MEMORY) and killed the
 * application. Textures join that batch; they do not open command buffers.
 */

#include "renderer/texture.hpp"

#include <spdlog/spdlog.h>

#include <ktx.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>

// stb_image is header-only and this is its single translation unit in the
// project. It is compiled with warnings suppressed because it is vendored third
// party code we do not patch.
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO           // every load goes through memory; no FILE* path is wanted
#define STBI_FAILURE_USERMSG    // stbi_failure_reason() text a human can act on
#include <stb/stb_image.h>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace stratum {

namespace {

// ============================================================================
// File identification
// ============================================================================

/// KTX2 identifier, from the KTX 2.0 specification section 3.1.
constexpr uint8_t kKtx2Magic[12] = {
    0xAB, 0x4B, 0x54, 0x58, 0x20, 0x32, 0x30, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A
};

/// KTX 1.1 identifier. Recognised only so the failure can name the real problem.
constexpr uint8_t kKtx1Magic[12] = {
    0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A
};

[[nodiscard]] bool has_magic(const uint8_t* bytes, size_t size, const uint8_t (&magic)[12]) {
    return size >= 12 && std::memcmp(bytes, magic, 12) == 0;
}

// ============================================================================
// VkFormat values used by KTX2
//
// vkformat_enum.h lives in KTX-Software/lib/src, which is NOT on libktx's public
// include path, and this project does not otherwise pull in the Vulkan headers.
// The handful of values actually needed are therefore spelled out. They are
// numeric constants fixed by the Vulkan specification and cannot change.
// ============================================================================

constexpr uint32_t kVkFormatR8Unorm         = 9;
constexpr uint32_t kVkFormatR8G8Unorm       = 16;
constexpr uint32_t kVkFormatR8G8B8A8Unorm   = 37;
constexpr uint32_t kVkFormatR8G8B8A8Srgb    = 43;
constexpr uint32_t kVkFormatBC5UnormBlock   = 141;
constexpr uint32_t kVkFormatBC7UnormBlock   = 145;
constexpr uint32_t kVkFormatBC7SrgbBlock    = 146;

// ============================================================================
// sRGB transfer function
// ============================================================================

/**
 * @brief Decode table for one sRGB-encoded 8-bit channel
 *
 * A mip level is an AVERAGE, and averaging must happen in the space where the
 * quantity is linear in light. Box filtering sRGB-encoded bytes directly biases
 * every level towards the dark end -- the classic "the distance gets darker as
 * you walk away" artefact -- and on a road surface, which is mostly mid-grey and
 * is seen almost entirely at glancing angles through its own mips, it is the
 * difference between wet-looking and correct.
 */
struct SrgbDecodeTable {
    float value[256];
    SrgbDecodeTable() {
        for (int i = 0; i < 256; ++i) {
            const float c = static_cast<float>(i) / 255.0f;
            value[i] = (c <= 0.04045f) ? (c / 12.92f)
                                       : std::pow((c + 0.055f) / 1.055f, 2.4f);
        }
    }
};

[[nodiscard]] const SrgbDecodeTable& srgb_decode() {
    static const SrgbDecodeTable table;
    return table;
}

[[nodiscard]] uint8_t linear_to_srgb_u8(float linear) {
    linear = std::clamp(linear, 0.0f, 1.0f);
    const float encoded = (linear <= 0.0031308f)
                        ? (linear * 12.92f)
                        : (1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f);
    return static_cast<uint8_t>(std::lround(std::clamp(encoded, 0.0f, 1.0f) * 255.0f));
}

// ============================================================================
// Mip generation
// ============================================================================

/**
 * @brief Box-filter one level down into @p dst
 *
 * A 2x2 box over floor coordinates, clamped at the edges so a non-power-of-two
 * source still produces a level rather than reading past its last row. sRGB
 * colour channels round-trip through the linear domain; ALPHA never does, because
 * in this project alpha is either paint coverage (the markings atlas) or a
 * linearly encoded height field (every procedural surface), and neither is a
 * colour. Linear formats -- normal maps, ORM packs, masks -- are averaged as
 * stored.
 *
 * Encoded normals are averaged component-wise and NOT renormalised. The result is
 * slightly short of unit length in high-curvature regions, which reads as a small
 * flattening with distance; that is the conventional trade, and the alternative
 * costs a square root per texel of every level for an effect no one has ever
 * pointed at in a road surface.
 */
void downsample(TextureFormat format, const uint8_t* src, uint32_t sw, uint32_t sh,
                uint8_t* dst, uint32_t dw, uint32_t dh) {
    const uint32_t channels = (format == TextureFormat::R8) ? 1u : 4u;
    const bool srgb_rgb = is_srgb_format(format);
    const SrgbDecodeTable& decode = srgb_decode();

    for (uint32_t y = 0; y < dh; ++y) {
        const uint32_t y0 = std::min(y * 2u, sh - 1u);
        const uint32_t y1 = std::min(y * 2u + 1u, sh - 1u);
        for (uint32_t x = 0; x < dw; ++x) {
            const uint32_t x0 = std::min(x * 2u, sw - 1u);
            const uint32_t x1 = std::min(x * 2u + 1u, sw - 1u);

            const uint8_t* t00 = src + (static_cast<size_t>(y0) * sw + x0) * channels;
            const uint8_t* t10 = src + (static_cast<size_t>(y0) * sw + x1) * channels;
            const uint8_t* t01 = src + (static_cast<size_t>(y1) * sw + x0) * channels;
            const uint8_t* t11 = src + (static_cast<size_t>(y1) * sw + x1) * channels;
            uint8_t* out = dst + (static_cast<size_t>(y) * dw + x) * channels;

            for (uint32_t c = 0; c < channels; ++c) {
                // Channel 3 of an sRGB RGBA texture is alpha, which is linear.
                if (srgb_rgb && c < 3u) {
                    const float sum = decode.value[t00[c]] + decode.value[t10[c]]
                                    + decode.value[t01[c]] + decode.value[t11[c]];
                    out[c] = linear_to_srgb_u8(sum * 0.25f);
                } else {
                    const uint32_t sum = static_cast<uint32_t>(t00[c]) + t10[c] + t01[c] + t11[c];
                    out[c] = static_cast<uint8_t>((sum + 2u) / 4u);
                }
            }
        }
    }
}

// ============================================================================
// Format mapping
// ============================================================================

[[nodiscard]] SDL_GPUTextureFormat to_sdl_format(TextureFormat format) {
    switch (format) {
        case TextureFormat::RGBA8:      return SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
        case TextureFormat::RGBA8_SRGB: return SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM_SRGB;
        case TextureFormat::BC7:        return SDL_GPU_TEXTUREFORMAT_BC7_RGBA_UNORM;
        case TextureFormat::BC7_SRGB:   return SDL_GPU_TEXTUREFORMAT_BC7_RGBA_UNORM_SRGB;
        case TextureFormat::BC5_Normal: return SDL_GPU_TEXTUREFORMAT_BC5_RG_UNORM;
        case TextureFormat::R8:         return SDL_GPU_TEXTUREFORMAT_R8_UNORM;
    }
    return SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
}

[[nodiscard]] const char* texture_format_name(TextureFormat format) {
    switch (format) {
        case TextureFormat::RGBA8:      return "RGBA8";
        case TextureFormat::RGBA8_SRGB: return "RGBA8_SRGB";
        case TextureFormat::BC7:        return "BC7";
        case TextureFormat::BC7_SRGB:   return "BC7_SRGB";
        case TextureFormat::BC5_Normal: return "BC5_Normal";
        case TextureFormat::R8:         return "R8";
    }
    return "Unknown";
}

[[nodiscard]] size_t align_up(size_t value, size_t alignment) {
    return (value + alignment - 1u) / alignment * alignment;
}

/// Read a whole file. Empty on failure; the caller reports the path.
[[nodiscard]] std::vector<uint8_t> read_file(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) return {};
    const std::streamoff size = file.tellg();
    if (size <= 0) return {};
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> bytes(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(bytes.data()), size)) return {};
    return bytes;
}

} // namespace

// ============================================================================
// Sampler naming
// ============================================================================

const char* sampler_kind_name(SamplerKind kind) {
    switch (kind) {
        case SamplerKind::RepeatAniso: return "RepeatAniso";
        case SamplerKind::ClampLinear: return "ClampLinear";
        case SamplerKind::RepeatPoint: return "RepeatPoint";
        case SamplerKind::Count:       break;
    }
    return "Unknown";
}

// ============================================================================
// Staging layout arithmetic
// ============================================================================

uint32_t GPUTextureManager::full_mip_count(uint32_t width, uint32_t height) {
    uint32_t largest = std::max(width, height);
    uint32_t levels = 1;
    while (largest > 1u) {
        largest >>= 1u;
        ++levels;
    }
    return levels;
}

uint32_t GPUTextureManager::level_dimension(uint32_t base, uint32_t level) {
    const uint32_t shifted = (level >= 32u) ? 0u : (base >> level);
    return std::max(1u, shifted);
}

size_t GPUTextureManager::level_bytes(TextureFormat format, uint32_t width, uint32_t height) {
    if (width == 0u || height == 0u) return 0u;
    if (is_block_compressed(format)) {
        // BC7 and BC5 are both 16 bytes per 4x4 block. A level smaller than one
        // block still occupies a whole block, which is why this rounds up rather
        // than scaling the byte count with the level's texel count.
        const size_t blocks_x = (static_cast<size_t>(width) + 3u) / 4u;
        const size_t blocks_y = (static_cast<size_t>(height) + 3u) / 4u;
        return blocks_x * blocks_y * 16u;
    }
    const size_t texels = static_cast<size_t>(width) * static_cast<size_t>(height);
    return (format == TextureFormat::R8) ? texels : texels * 4u;
}

size_t GPUTextureManager::packed_level_offset(const TextureDesc& desc, uint32_t level) {
    size_t offset = 0;
    for (uint32_t i = 0; i < level; ++i) {
        const size_t bytes = level_bytes(desc.format,
                                         level_dimension(desc.width, i),
                                         level_dimension(desc.height, i));
        offset = align_up(offset + bytes, kStagingAlignment);
    }
    return offset;
}

size_t GPUTextureManager::packed_total_bytes(const TextureDesc& desc) {
    if (desc.mip_levels == 0u) return 0u;
    const uint32_t last = desc.mip_levels - 1u;
    return packed_level_offset(desc, last)
         + level_bytes(desc.format,
                       level_dimension(desc.width, last),
                       level_dimension(desc.height, last));
}

// ============================================================================
// Lifetime
// ============================================================================

GPUTextureManager::~GPUTextureManager() {
    shutdown();
}

void GPUTextureManager::query_format_support() {
    constexpr SDL_GPUTextureUsageFlags kSampled = SDL_GPU_TEXTUREUSAGE_SAMPLER;
    m_supports_bc7 = SDL_GPUTextureSupportsFormat(
        m_device, SDL_GPU_TEXTUREFORMAT_BC7_RGBA_UNORM, SDL_GPU_TEXTURETYPE_2D, kSampled);
    m_supports_bc7_srgb = SDL_GPUTextureSupportsFormat(
        m_device, SDL_GPU_TEXTUREFORMAT_BC7_RGBA_UNORM_SRGB, SDL_GPU_TEXTURETYPE_2D, kSampled);
    m_supports_bc5 = SDL_GPUTextureSupportsFormat(
        m_device, SDL_GPU_TEXTUREFORMAT_BC5_RG_UNORM, SDL_GPU_TEXTURETYPE_2D, kSampled);

    spdlog::info("Texture format support: BC7 {}, BC7_SRGB {}, BC5 {} (driver '{}')",
                 m_supports_bc7 ? "yes" : "no",
                 m_supports_bc7_srgb ? "yes" : "no",
                 m_supports_bc5 ? "yes" : "no",
                 SDL_GetGPUDeviceDriver(m_device) ? SDL_GetGPUDeviceDriver(m_device) : "unknown");

    if (!m_supports_bc7 || !m_supports_bc5) {
        // Not an error. Every BC path in this file has an RGBA8 fallback; saying
        // so once at startup is what makes a later "why is this texture 4x the
        // memory" question answerable.
        spdlog::info("Block-compressed textures will fall back to RGBA8 where the "
                     "source can be transcoded, and be refused where it cannot");
    }
}

bool GPUTextureManager::init(SDL_GPUDevice* device) {
    if (m_device) {
        spdlog::warn("GPUTextureManager already initialised");
        return true;
    }
    if (!device) {
        spdlog::error("GPUTextureManager::init called with a null device");
        return false;
    }

    m_device = device;
    query_format_support();

    if (!create_samplers()) {
        shutdown();
        return false;
    }
    if (!create_fallbacks()) {
        shutdown();
        return false;
    }

    // The one place in this file that owns a command buffer. The fallbacks are a
    // few tens of kilobytes, every later upload substitutes one of them while it
    // waits, and there is no frame yet whose copy pass could carry them -- so they
    // get a single submit and a single SDL_WaitForGPUIdle(), at startup, before
    // the first frame exists. One submit total, not one per texture, and it is
    // deliberately not a pattern anything else in this file follows.
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
    if (!cmd) {
        spdlog::error("Failed to acquire the fallback upload command buffer: {}", SDL_GetError());
        shutdown();
        return false;
    }
    SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);
    flush_uploads(copy_pass, 0);
    SDL_EndGPUCopyPass(copy_pass);
    const bool submitted = SDL_SubmitGPUCommandBuffer(cmd);
    commit_uploads(submitted);
    if (!submitted) {
        spdlog::error("Failed to submit the fallback upload: {}", SDL_GetError());
        shutdown();
        return false;
    }
    SDL_WaitForGPUIdle(m_device);
    drain_retired_transfers(0, true);

    if (!m_pending.empty()) {
        // Only reachable if the per-frame byte budget were ever set below the size
        // of the fallbacks, which would leave init() returning textures that are
        // not sampleable yet -- the one thing the rest of the class assumes cannot
        // happen.
        spdlog::error("{} fallback textures did not upload during init; the per-frame "
                      "budget ({} KB) is smaller than the fallback set",
                      m_pending.size(), kMaxUploadBytesPerFrame / 1024);
        shutdown();
        return false;
    }

    spdlog::info("GPUTextureManager ready: {} fallback textures, {} KB resident",
                 m_slots.size(), m_resident_bytes / 1024);
    return true;
}

void GPUTextureManager::shutdown() {
    if (!m_device) {
        // Still clear the CPU side: shutdown() is documented safe on an
        // uninitialised manager and safe to call twice.
        m_slots.clear();
        m_free_slots.clear();
        m_staging.clear();
        m_staging.shrink_to_fit();
        m_pending.clear();
        m_in_flight.clear();
        m_retired_transfers.clear();
        return;
    }

    // The four built-ins are expected. Anything else still resident is a caller
    // that never released, and its name is the only way to find which one.
    size_t leaked = 0;
    for (size_t i = 0; i < m_slots.size(); ++i) {
        const TextureSlot& slot = m_slots[i];
        if (!slot.in_use) continue;
        const TextureHandle handle = static_cast<TextureHandle>(i);
        if (handle == m_missing || handle == m_white
            || handle == m_flat_normal || handle == m_default_orm) {
            continue;
        }
        ++leaked;
        if (leaked <= 8) {
            spdlog::warn("GPUTextureManager::shutdown found a live texture: handle {} '{}' "
                         "({}x{} {}, {} KB)", handle, slot.debug_name, slot.desc.width,
                         slot.desc.height, texture_format_name(slot.desc.format),
                         slot.bytes / 1024);
        }
    }
    if (leaked > 8) {
        spdlog::warn("... and {} further live textures", leaked - 8);
    }
    if (leaked > 0) {
        spdlog::warn("{} textures were still live at shutdown ({} KB). They are released "
                     "here, but a caller kept a handle it never gave back",
                     leaked, m_resident_bytes / 1024);
    }

    for (TextureSlot& slot : m_slots) {
        if (slot.texture) {
            SDL_ReleaseGPUTexture(m_device, slot.texture);
            slot.texture = nullptr;
        }
        slot.in_use = false;
        slot.ready = false;
    }
    m_slots.clear();
    m_free_slots.clear();

    for (SDL_GPUSampler*& sampler : m_samplers) {
        if (sampler) {
            SDL_ReleaseGPUSampler(m_device, sampler);
            sampler = nullptr;
        }
    }

    // force = true is only legal after the device is idle. GPURenderer::shutdown()
    // calls SDL_WaitForGPUIdle() before tearing down anything it owns, and this
    // manager is torn down from there.
    drain_retired_transfers(0, true);

    m_staging.clear();
    m_staging.shrink_to_fit();
    m_pending.clear();
    m_in_flight.clear();

    m_missing = kInvalidTexture;
    m_white = kInvalidTexture;
    m_flat_normal = kInvalidTexture;
    m_default_orm = kInvalidTexture;
    m_resident_bytes = 0;
    m_supports_bc7 = false;
    m_supports_bc7_srgb = false;
    m_supports_bc5 = false;
    m_device = nullptr;
}

// ============================================================================
// Samplers
// ============================================================================

bool GPUTextureManager::create_samplers() {
    // Anisotropy is the whole reason the road surface is worth texturing at all:
    // a carriageway is a plane viewed at a few degrees off edge-on, which is the
    // worst case trilinear filtering has, and 8x is where the returns flatten.
    SDL_GPUSamplerCreateInfo repeat_aniso{};
    repeat_aniso.min_filter = SDL_GPU_FILTER_LINEAR;
    repeat_aniso.mag_filter = SDL_GPU_FILTER_LINEAR;
    repeat_aniso.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;
    repeat_aniso.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    repeat_aniso.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    repeat_aniso.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    repeat_aniso.max_lod = 1000.0f;     // no clamp; every level a texture has is usable
    repeat_aniso.enable_anisotropy = true;
    repeat_aniso.max_anisotropy = 8.0f;

    m_samplers[static_cast<size_t>(SamplerKind::RepeatAniso)] =
        SDL_CreateGPUSampler(m_device, &repeat_aniso);
    if (!m_samplers[static_cast<size_t>(SamplerKind::RepeatAniso)]) {
        // "8x if supported" in practice: SDL_GPU exposes no anisotropy limit query,
        // so the only way to find out is to ask and accept the answer.
        spdlog::warn("Anisotropic sampler creation failed ({}); retrying trilinear",
                     SDL_GetError());
        repeat_aniso.enable_anisotropy = false;
        repeat_aniso.max_anisotropy = 1.0f;
        m_samplers[static_cast<size_t>(SamplerKind::RepeatAniso)] =
            SDL_CreateGPUSampler(m_device, &repeat_aniso);
        if (!m_samplers[static_cast<size_t>(SamplerKind::RepeatAniso)]) {
            spdlog::error("Failed to create the RepeatAniso sampler: {}", SDL_GetError());
            return false;
        }
    }

    // CLAMP_TO_EDGE, and not by preference. The markings atlas packs sprites into
    // one 1024x1024 image; a UV that drifts one texel past a sub-rect under REPEAT
    // wraps to the far edge of the ATLAS, dragging an unrelated sprite into the
    // sample. Clamping keeps that error local to the rect it started in.
    SDL_GPUSamplerCreateInfo clamp_linear{};
    clamp_linear.min_filter = SDL_GPU_FILTER_LINEAR;
    clamp_linear.mag_filter = SDL_GPU_FILTER_LINEAR;
    clamp_linear.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;
    clamp_linear.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    clamp_linear.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    clamp_linear.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    clamp_linear.max_lod = 1000.0f;
    m_samplers[static_cast<size_t>(SamplerKind::ClampLinear)] =
        SDL_CreateGPUSampler(m_device, &clamp_linear);
    if (!m_samplers[static_cast<size_t>(SamplerKind::ClampLinear)]) {
        spdlog::error("Failed to create the ClampLinear sampler: {}", SDL_GetError());
        return false;
    }

    SDL_GPUSamplerCreateInfo repeat_point{};
    repeat_point.min_filter = SDL_GPU_FILTER_NEAREST;
    repeat_point.mag_filter = SDL_GPU_FILTER_NEAREST;
    repeat_point.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
    repeat_point.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    repeat_point.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    repeat_point.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    repeat_point.max_lod = 1000.0f;
    m_samplers[static_cast<size_t>(SamplerKind::RepeatPoint)] =
        SDL_CreateGPUSampler(m_device, &repeat_point);
    if (!m_samplers[static_cast<size_t>(SamplerKind::RepeatPoint)]) {
        spdlog::error("Failed to create the RepeatPoint sampler: {}", SDL_GetError());
        return false;
    }

    return true;
}

SDL_GPUSampler* GPUTextureManager::sampler(SamplerKind kind) const {
    const size_t index = static_cast<size_t>(kind);
    if (index >= static_cast<size_t>(SamplerKind::Count)) {
        return m_samplers[static_cast<size_t>(SamplerKind::RepeatAniso)];
    }
    return m_samplers[index];
}

// ============================================================================
// Fallbacks
// ============================================================================

bool GPUTextureManager::create_fallbacks() {
    // The checker goes first so that it lands in slot 0, which is kInvalidTexture.
    // That is what makes "an unresolvable handle" and "the missing texture" the
    // same value, and it is why get() can promise never to return nullptr.
    {
        TextureDesc desc{};
        desc.width = kCheckerSize;
        desc.height = kCheckerSize;
        desc.format = TextureFormat::RGBA8_SRGB;
        desc.mip_levels = full_mip_count(kCheckerSize, kCheckerSize);

        std::vector<uint8_t> pixels(static_cast<size_t>(kCheckerSize) * kCheckerSize * 4u);
        for (uint32_t y = 0; y < kCheckerSize; ++y) {
            for (uint32_t x = 0; x < kCheckerSize; ++x) {
                const bool light = ((x / kCheckerSquare) + (y / kCheckerSquare)) % 2u == 0u;
                uint8_t* texel = pixels.data() + (static_cast<size_t>(y) * kCheckerSize + x) * 4u;
                // Magenta on black. Nothing in a city is this colour, it survives
                // every mip level as an obviously wrong purple, and it is legible
                // as a CHECKER at close range, so a wrongly scaled UV is visible
                // too and not just a wrong texture.
                texel[0] = light ? 255u : 0u;
                texel[1] = 0u;
                texel[2] = light ? 255u : 0u;
                texel[3] = 255u;
            }
        }
        m_missing = create(desc, pixels.data(), pixels.size());
        // Success and failure both return 0 here, because a successful first
        // create() IS handle 0. They are told apart by whether a slot exists:
        // create() allocates its slot only after SDL_CreateGPUTexture has
        // succeeded, so an empty table means the create failed.
        if (m_slots.empty()) {
            spdlog::error("Failed to create the missing-texture checker");
            return false;
        }
        if (m_missing != kInvalidTexture) {
            spdlog::error("The missing-texture checker landed in slot {} rather than slot 0; "
                          "an unresolvable handle would no longer be visible", m_missing);
            return false;
        }
        m_slots[m_missing].debug_name = "builtin:missing";
    }

    struct Builtin {
        TextureHandle* out;
        TextureFormat format;
        uint8_t rgba[4];
        const char* name;
    };
    // 1x1 constants, one per map the PBR shader samples, chosen so that binding
    // them changes nothing: white albedo multiplies base_color through unchanged,
    // (128,128,255) is tangent-space "no perturbation", and a unit ORM leaves the
    // material's authored scalars exactly as they are. That is what lets the
    // shader sample unconditionally instead of branching on whether a map exists.
    //
    // "Unit" has to mean unit in ALL THREE ORM channels; see kDefaultOrmTexel.
    // Blue was 0 here, which is the multiplicative ANNIHILATOR, and since every
    // material in the library binds this texture it zeroed every authored
    // metallic in the project.
    const Builtin builtins[] = {
        { &m_white,       TextureFormat::RGBA8_SRGB, { 255, 255, 255, 255 }, "builtin:white" },
        { &m_flat_normal, TextureFormat::RGBA8,      { 128, 128, 255, 255 }, "builtin:flat_normal" },
        { &m_default_orm, TextureFormat::RGBA8,
          { kDefaultOrmTexel[0], kDefaultOrmTexel[1], kDefaultOrmTexel[2], kDefaultOrmTexel[3] },
          "builtin:default_orm" },
    };
    for (const Builtin& builtin : builtins) {
        TextureDesc desc{};
        desc.width = 1;
        desc.height = 1;
        desc.mip_levels = 1;
        desc.format = builtin.format;
        *builtin.out = create(desc, builtin.rgba, sizeof(builtin.rgba));
        if (*builtin.out == kInvalidTexture) {
            spdlog::error("Failed to create {}", builtin.name);
            return false;
        }
        m_slots[*builtin.out].debug_name = builtin.name;
    }

    return true;
}

TextureHandle GPUTextureManager::missing_texture() const { return m_missing; }
TextureHandle GPUTextureManager::white() const { return m_white; }
TextureHandle GPUTextureManager::flat_normal() const { return m_flat_normal; }
TextureHandle GPUTextureManager::default_orm() const { return m_default_orm; }

// ============================================================================
// Slots and staging
// ============================================================================

TextureHandle GPUTextureManager::allocate_slot() {
    if (!m_free_slots.empty()) {
        const uint32_t index = m_free_slots.back();
        m_free_slots.pop_back();
        m_slots[index] = TextureSlot{};
        return index;
    }
    m_slots.emplace_back();
    return static_cast<TextureHandle>(m_slots.size() - 1u);
}

size_t GPUTextureManager::reserve_staging(const TextureDesc& desc) {
    const size_t offset = m_staging.size();
    m_staging.resize(offset + packed_total_bytes(desc));
    return offset;
}

TextureHandle GPUTextureManager::finish_staged(const TextureDesc& desc, size_t staging_offset,
                                               std::string debug_name) {
    const size_t bytes = packed_total_bytes(desc);

    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.format = to_sdl_format(desc.format);
    info.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;
    info.width = desc.width;
    info.height = desc.height;
    info.layer_count_or_depth = desc.layers;
    info.num_levels = desc.mip_levels;
    info.sample_count = SDL_GPU_SAMPLECOUNT_1;

    SDL_GPUTexture* texture = SDL_CreateGPUTexture(m_device, &info);
    if (!texture) {
        spdlog::error("SDL_CreateGPUTexture failed for '{}' ({}x{} {}, {} levels): {}",
                      debug_name, desc.width, desc.height,
                      texture_format_name(desc.format), desc.mip_levels, SDL_GetError());
        // Give the arena its bytes back if this was the most recent reservation,
        // which it always is on this path.
        if (staging_offset + bytes == m_staging.size()) {
            m_staging.resize(staging_offset);
        }
        return kInvalidTexture;
    }
    SDL_SetGPUTextureName(m_device, texture, debug_name.c_str());

    const TextureHandle handle = allocate_slot();
    TextureSlot& slot = m_slots[handle];
    slot.texture = texture;
    slot.desc = desc;
    slot.bytes = bytes;
    slot.in_use = true;
    slot.ready = false;
    slot.debug_name = std::move(debug_name);

    m_resident_bytes += bytes;
    m_pending.push_back(PendingTextureUpload{ handle, staging_offset, bytes, desc.mip_levels });
    return handle;
}

// ============================================================================
// create()
// ============================================================================

TextureHandle GPUTextureManager::create(const TextureDesc& desc_in, const void* pixels,
                                        size_t byte_size) {
    if (!m_device) {
        spdlog::error("GPUTextureManager::create called before init()");
        return kInvalidTexture;
    }
    if (!desc_in.is_valid()) {
        spdlog::error("GPUTextureManager::create refused an invalid description "
                      "({}x{}, {} levels, {} layers)",
                      desc_in.width, desc_in.height, desc_in.mip_levels, desc_in.layers);
        return kInvalidTexture;
    }
    if (is_block_compressed(desc_in.format)) {
        // There is no CPU block encoder in this project and there is not going to
        // be one: compressed pixels arrive already compressed, in a KTX2 file.
        spdlog::error("GPUTextureManager::create cannot take block-compressed pixels "
                      "({}); load a KTX2 file instead", texture_format_name(desc_in.format));
        return kInvalidTexture;
    }

    TextureDesc desc = desc_in;
    const uint32_t max_levels = full_mip_count(desc.width, desc.height);
    if (desc.mip_levels > max_levels) {
        desc.mip_levels = max_levels;
    }
    if (desc.layers != 1u) {
        spdlog::error("GPUTextureManager::create supports single-layer 2D textures only "
                      "(asked for {} layers)", desc.layers);
        return kInvalidTexture;
    }

    const size_t needed = desc.level0_bytes();
    if (pixels && byte_size < needed) {
        // Refused rather than read past. A short buffer here is a heap overrun
        // that reads whatever follows the caller's allocation, uploads it, and
        // usually survives long enough to be blamed on something else entirely.
        spdlog::error("GPUTextureManager::create was given {} bytes for a {}x{} {} level 0 "
                      "that needs {}", byte_size, desc.width, desc.height,
                      texture_format_name(desc.format), needed);
        return kInvalidTexture;
    }

    if (!pixels) {
        // Undefined contents: there is nothing to stage, so the texture is created
        // and the queued copy is withdrawn immediately rather than being left to
        // read an arena range that was never reserved.
        const TextureHandle handle = finish_staged(desc, m_staging.size(),
                                                   "generated:uninitialised");
        if (handle != kInvalidTexture) {
            m_pending.pop_back();
            m_slots[handle].ready = true;
        }
        return handle;
    }

    const size_t base = reserve_staging(desc);
    uint8_t* arena = m_staging.data() + base;
    std::memcpy(arena, pixels, needed);

    // Build the rest of the chain in place, each level from the one above it, so a
    // 512x512 albedo costs one pass over 1.33x its own size and never allocates.
    for (uint32_t level = 1; level < desc.mip_levels; ++level) {
        const uint32_t sw = level_dimension(desc.width, level - 1u);
        const uint32_t sh = level_dimension(desc.height, level - 1u);
        const uint32_t dw = level_dimension(desc.width, level);
        const uint32_t dh = level_dimension(desc.height, level);
        downsample(desc.format,
                   arena + packed_level_offset(desc, level - 1u), sw, sh,
                   arena + packed_level_offset(desc, level), dw, dh);
    }

    return finish_staged(desc, base, "generated");
}

// ============================================================================
// load()
// ============================================================================

TextureHandle GPUTextureManager::load(const std::filesystem::path& path, bool srgb) {
    if (!m_device) {
        spdlog::error("GPUTextureManager::load('{}') called before init()", path.string());
        return kInvalidTexture;
    }

    const std::vector<uint8_t> bytes = read_file(path);
    if (bytes.empty()) {
        spdlog::error("Texture load failed: cannot read '{}'", path.string());
        ++m_load_failures;
        return kInvalidTexture;
    }

    // Dispatch on CONTENT first and extension second. A .png that is really a KTX2
    // is common enough after a bad export, and the magic bytes are the only thing
    // that is never wrong. The extension is used solely to catch the opposite
    // case, where a file claims to be KTX2 and is not, so the failure names the
    // real problem instead of letting stb_image report "unknown image type".
    TextureHandle handle = kInvalidTexture;
    if (has_magic(bytes.data(), bytes.size(), kKtx2Magic)) {
        handle = load_ktx2(bytes.data(), bytes.size(), path, srgb);
    } else if (has_magic(bytes.data(), bytes.size(), kKtx1Magic)) {
        spdlog::error("Texture load failed: '{}' is a KTX 1.1 file. libktx is built here for "
                      "KTX2 only; convert it with 'ktx convert'", path.string());
    } else {
        std::string extension = path.extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (extension == ".ktx2" || extension == ".ktx") {
            spdlog::error("Texture load failed: '{}' has a KTX extension but does not begin "
                          "with the KTX2 identifier; it is corrupt or misnamed", path.string());
        } else {
            handle = load_stb(bytes.data(), bytes.size(), path, srgb);
        }
    }

    if (handle == kInvalidTexture) {
        ++m_load_failures;
    }
    return handle;
}

TextureHandle GPUTextureManager::load_ktx2(const uint8_t* bytes, size_t size,
                                           const std::filesystem::path& path, bool srgb) {
    ktxTexture2* ktx = nullptr;
    KTX_error_code rc = ktxTexture2_CreateFromMemory(
        bytes, size, KTX_TEXTURE_CREATE_LOAD_IMAGE_DATA_BIT, &ktx);
    if (rc != KTX_SUCCESS || !ktx) {
        spdlog::error("Texture load failed: libktx could not open '{}': {}",
                      path.string(), ktxErrorString(rc));
        return kInvalidTexture;
    }

    struct KtxGuard {
        ktxTexture2* t;
        ~KtxGuard() { if (t) ktxTexture_Destroy(ktxTexture(t)); }
    } guard{ ktx };

    if (ktx->numDimensions != 2u || ktx->isArray || ktx->numFaces != 1u || ktx->numLayers != 1u) {
        spdlog::error("Texture load failed: '{}' is {}D with {} faces and {} layers; only "
                      "single-layer 2D textures are supported",
                      path.string(), ktx->numDimensions, ktx->numFaces, ktx->numLayers);
        return kInvalidTexture;
    }

    // Basis Universal / UASTC payloads are not a GPU format at all until they are
    // transcoded, and the target is chosen from what the DEVICE says it supports
    // rather than from what a desktop usually has.
    if (ktxTexture2_NeedsTranscoding(ktx)) {
        ktx_transcode_fmt_e target = KTX_TTF_RGBA32;
        const uint32_t components = ktxTexture2_GetNumComponents(ktx);
        if (!srgb && components <= 2u && m_supports_bc5) {
            // Two-channel linear data is a normal map; BC5 keeps both channels at
            // 8:1 with none of BC7's colour-space assumptions.
            target = KTX_TTF_BC5_RG;
        } else if (srgb ? (m_supports_bc7 && m_supports_bc7_srgb) : m_supports_bc7) {
            target = KTX_TTF_BC7_RGBA;
        }
        rc = ktxTexture2_TranscodeBasis(ktx, target, 0);
        if (rc != KTX_SUCCESS) {
            spdlog::error("Texture load failed: transcoding '{}' to {} failed: {}",
                          path.string(), ktxTranscodeFormatString(target), ktxErrorString(rc));
            return kInvalidTexture;
        }
        if (target == KTX_TTF_RGBA32) {
            spdlog::info("'{}' transcoded to RGBA8: the device does not sample the "
                         "block-compressed format it would otherwise have used", path.string());
        }
    }

    // The file's own transfer function is INFORMATION; the caller's srgb argument
    // is INTENT, and intent wins. A normal map exported with an sRGB tag is a
    // common authoring mistake, and honouring the tag would gamma-skew every
    // normal in a way that looks like slightly soft lighting rather than an error.
    const bool file_says_srgb = (ktxTexture2_GetOETF_e(ktx) == KHR_DF_TRANSFER_SRGB);
    if (file_says_srgb != srgb) {
        spdlog::warn("'{}' is tagged {} but is being loaded as {}; the caller's intent wins",
                     path.string(), file_says_srgb ? "sRGB" : "linear", srgb ? "sRGB" : "linear");
    }

    TextureFormat format{};
    switch (ktx->vkFormat) {
        case kVkFormatR8G8B8A8Unorm:
        case kVkFormatR8G8B8A8Srgb:
            format = srgb ? TextureFormat::RGBA8_SRGB : TextureFormat::RGBA8;
            break;
        case kVkFormatR8Unorm:
            format = TextureFormat::R8;
            break;
        case kVkFormatBC7UnormBlock:
        case kVkFormatBC7SrgbBlock:
            format = srgb ? TextureFormat::BC7_SRGB : TextureFormat::BC7;
            break;
        case kVkFormatBC5UnormBlock:
            format = TextureFormat::BC5_Normal;
            break;
        case kVkFormatR8G8Unorm:
            spdlog::error("Texture load failed: '{}' is two-channel R8G8, which this renderer "
                          "has no sampler layout for; export it as RGBA8 or BC5", path.string());
            return kInvalidTexture;
        default:
            spdlog::error("Texture load failed: '{}' is VkFormat {}, which this renderer does "
                          "not upload", path.string(), ktx->vkFormat);
            return kInvalidTexture;
    }

    // A natively block-compressed file on a device that cannot sample it. There is
    // no CPU decoder here, so this is refused by name rather than silently swapped
    // for the checker: the fix is an RGBA8 or Basis-supercompressed export, and
    // only a message that says so gets anyone there.
    const bool needs_bc7 = (format == TextureFormat::BC7 || format == TextureFormat::BC7_SRGB);
    if (needs_bc7 && !(format == TextureFormat::BC7_SRGB ? m_supports_bc7_srgb : m_supports_bc7)) {
        spdlog::error("Texture load failed: '{}' is {} and this device does not sample it. "
                      "Re-export as RGBA8, or as Basis/UASTC so it can be transcoded",
                      path.string(), texture_format_name(format));
        return kInvalidTexture;
    }
    if (format == TextureFormat::BC5_Normal && !m_supports_bc5) {
        spdlog::error("Texture load failed: '{}' is BC5 and this device does not sample it. "
                      "Re-export as RGBA8, or as Basis/UASTC so it can be transcoded",
                      path.string());
        return kInvalidTexture;
    }

    TextureDesc desc{};
    desc.width = ktx->baseWidth;
    desc.height = ktx->baseHeight;
    desc.layers = 1;
    desc.format = format;
    desc.mip_levels = std::max(1u, ktx->numLevels);

    const uint32_t max_levels = full_mip_count(desc.width, desc.height);
    if (desc.mip_levels > max_levels) {
        spdlog::warn("'{}' claims {} mip levels but {}x{} admits only {}; the extra levels are "
                     "ignored", path.string(), desc.mip_levels, desc.width, desc.height,
                     max_levels);
        desc.mip_levels = max_levels;
    }

    // A KTX2 file that carries its own chain is used as authored. One that does
    // not gets a generated chain, because an unmipped road surface at a glancing
    // angle aliases into noise and no amount of anisotropy fixes a missing level.
    const uint32_t file_levels = desc.mip_levels;
    const bool generate_chain = (file_levels == 1u) && (max_levels > 1u)
                             && !is_block_compressed(format);
    if (generate_chain) {
        desc.mip_levels = max_levels;
    } else if (file_levels == 1u && max_levels > 1u) {
        spdlog::warn("'{}' is block-compressed with a single mip level; it cannot be "
                     "downsampled here and will alias at distance. Re-export with mipmaps",
                     path.string());
    }

    const ktx_uint8_t* data = ktxTexture_GetData(ktxTexture(ktx));
    if (!data) {
        spdlog::error("Texture load failed: '{}' carries no image data", path.string());
        return kInvalidTexture;
    }

    const size_t base = reserve_staging(desc);
    uint8_t* arena = m_staging.data() + base;

    for (uint32_t level = 0; level < file_levels; ++level) {
        ktx_size_t offset = 0;
        rc = ktxTexture_GetImageOffset(ktxTexture(ktx), level, 0, 0, &offset);
        if (rc != KTX_SUCCESS) {
            spdlog::error("Texture load failed: '{}' level {} has no offset: {}",
                          path.string(), level, ktxErrorString(rc));
            m_staging.resize(base);
            return kInvalidTexture;
        }
        const size_t expected = level_bytes(format,
                                            level_dimension(desc.width, level),
                                            level_dimension(desc.height, level));
        const size_t supplied = static_cast<size_t>(ktxTexture_GetImageSize(ktxTexture(ktx), level));
        if (supplied != expected || offset + expected > ktx->dataSize) {
            // A mismatch means this file's layout is not the one the copy regions
            // below describe, and copying anyway would upload a level's worth of
            // whatever follows it.
            spdlog::error("Texture load failed: '{}' level {} is {} bytes at offset {} of a {} "
                          "byte payload; {} expects {}", path.string(), level, supplied, offset,
                          ktx->dataSize, texture_format_name(format), expected);
            m_staging.resize(base);
            return kInvalidTexture;
        }
        std::memcpy(arena + packed_level_offset(desc, level), data + offset, expected);
    }

    if (generate_chain) {
        for (uint32_t level = 1; level < desc.mip_levels; ++level) {
            downsample(format,
                       arena + packed_level_offset(desc, level - 1u),
                       level_dimension(desc.width, level - 1u),
                       level_dimension(desc.height, level - 1u),
                       arena + packed_level_offset(desc, level),
                       level_dimension(desc.width, level),
                       level_dimension(desc.height, level));
        }
    }

    return finish_staged(desc, base, path.filename().string());
}

TextureHandle GPUTextureManager::load_stb(const uint8_t* bytes, size_t size,
                                          const std::filesystem::path& path, bool srgb) {
    int width = 0;
    int height = 0;
    int source_channels = 0;
    // Forced to 4 channels: SDL_GPU has no 24-bit format, and a three-channel
    // upload would need a repack anyway.
    stbi_uc* pixels = stbi_load_from_memory(bytes, static_cast<int>(size),
                                            &width, &height, &source_channels, 4);
    if (!pixels) {
        const char* reason = stbi_failure_reason();
        spdlog::error("Texture load failed: stb_image could not decode '{}': {}",
                      path.string(), reason ? reason : "unknown");
        return kInvalidTexture;
    }
    if (width <= 0 || height <= 0) {
        stbi_image_free(pixels);
        spdlog::error("Texture load failed: '{}' decoded to {}x{}", path.string(), width, height);
        return kInvalidTexture;
    }

    TextureDesc desc{};
    desc.width = static_cast<uint32_t>(width);
    desc.height = static_cast<uint32_t>(height);
    desc.layers = 1;
    // stb gives back exactly the bytes that were authored, with no transfer
    // function applied. Whether those bytes are sRGB is not discoverable from a
    // PNG in any way worth trusting, so it is the caller's declaration -- albedo
    // true, normal and ORM false -- that picks the format.
    desc.format = srgb ? TextureFormat::RGBA8_SRGB : TextureFormat::RGBA8;
    desc.mip_levels = full_mip_count(desc.width, desc.height);

    const size_t level0 = static_cast<size_t>(width) * static_cast<size_t>(height) * 4u;
    const TextureHandle handle = create(desc, pixels, level0);
    stbi_image_free(pixels);

    if (handle != kInvalidTexture) {
        m_slots[handle].debug_name = path.filename().string();
        spdlog::debug("Loaded '{}' {}x{} {} ({} source channels), {} mip levels",
                      path.string(), width, height, texture_format_name(desc.format),
                      source_channels, desc.mip_levels);
    }
    return handle;
}

// ============================================================================
// release()
// ============================================================================

void GPUTextureManager::release(TextureHandle handle) {
    if (!m_device) return;
    if (handle >= m_slots.size()) return;
    if (handle == m_missing || handle == m_white
        || handle == m_flat_normal || handle == m_default_orm) {
        // The fallbacks outlive every material by design: every unbound map on
        // every material resolves to one, so releasing one would leave live
        // materials pointing at a dead texture.
        return;
    }

    TextureSlot& slot = m_slots[handle];
    if (!slot.in_use) return;

    // A texture can be released before its staged copy ever ran. Drop the pending
    // entry so the flush does not aim a copy at a texture that no longer exists;
    // its bytes stay in the arena as a hole until the next compaction, which is
    // why the flush packs the transfer buffer itself rather than assuming the
    // queue's staging offsets are contiguous.
    std::erase_if(m_pending, [handle](const PendingTextureUpload& p) {
        return p.handle == handle;
    });

    if (slot.texture) {
        // SDL_GPU defers the real destruction until no submitted command buffer
        // still references the texture, so unlike a pooled buffer range this needs
        // no retirement queue of its own.
        SDL_ReleaseGPUTexture(m_device, slot.texture);
    }
    m_resident_bytes -= std::min(m_resident_bytes, slot.bytes);

    slot = TextureSlot{};
    m_free_slots.push_back(handle);
}

// ============================================================================
// Lookup
// ============================================================================

SDL_GPUTexture* GPUTextureManager::get(TextureHandle handle) const {
    if (handle < m_slots.size() && m_slots[handle].in_use && m_slots[handle].texture) {
        return m_slots[handle].texture;
    }
    // Never nullptr after init(): an unknown, released or kInvalidTexture handle
    // resolves to the checker, so a caller that forgot to check still binds
    // something legal and the mistake shows up on screen instead of in a
    // validation log. Before init() there is nothing to return but nullptr.
    if (m_missing < m_slots.size() && m_slots[m_missing].in_use) {
        return m_slots[m_missing].texture;
    }
    return nullptr;
}

bool GPUTextureManager::is_ready(TextureHandle handle) const {
    return handle < m_slots.size() && m_slots[handle].in_use && m_slots[handle].ready;
}

SDL_GPUTexture* GPUTextureManager::bind_texture(TextureHandle handle,
                                                TextureHandle fallback) const {
    if (is_ready(handle)) {
        return get(handle);
    }
    // Sampling a texture whose copy has not run yet reads undefined device memory,
    // which on some drivers is the previous tenant of that allocation and on
    // others is noise. A half-loaded material is deliberately PLAIN -- white
    // albedo, flat normal, unit ORM -- rather than garbage, because a road that is
    // briefly untextured while it streams reads as loading, and a road that is
    // briefly full of static reads as a bug.
    if (is_ready(fallback)) {
        return get(fallback);
    }
    return get(m_missing);
}

GPUTextureManager::Stats GPUTextureManager::stats() const {
    Stats out{};
    for (const TextureSlot& slot : m_slots) {
        if (slot.in_use) ++out.textures;
    }
    out.bytes = m_resident_bytes;
    out.load_failures = m_load_failures;
    return out;
}

size_t GPUTextureManager::pending_upload_count() const {
    return m_pending.size();
}

// ============================================================================
// Batched upload
// ============================================================================

void GPUTextureManager::flush_uploads(SDL_GPUCopyPass* copy_pass, uint64_t frame_index) {
    if (!m_device || !copy_pass) return;

    // A batch left unsettled by a caller that skipped commit_uploads() is rolled
    // back rather than silently abandoned: its staged bytes are still intact and
    // its textures are still not ready, so re-queueing them is both safe and the
    // only outcome that does not lose the pixels.
    if (!m_in_flight.empty()) {
        spdlog::warn("GPUTextureManager: {} texture upload(s) were never committed; "
                     "re-queueing them", m_in_flight.size());
        commit_uploads(false);
    }

    if (m_pending.empty()) return;

    // Admit a budgeted prefix, always including the first entry -- a single
    // texture larger than the budget would otherwise never upload at all and its
    // material would bind the fallback forever. Each entry's run starts on
    // kStagingAlignment so that every level offset inside it, which is already
    // aligned relative to the run, is aligned absolutely too.
    std::vector<size_t> bases;
    bases.reserve(m_pending.size());
    size_t total = 0;
    for (const PendingTextureUpload& pending : m_pending) {
        const size_t base = align_up(total, kStagingAlignment);
        if (!bases.empty() && base + pending.bytes > kMaxUploadBytesPerFrame) break;
        bases.push_back(base);
        total = base + pending.bytes;
    }
    const size_t count = bases.size();
    if (count == 0 || total == 0) return;

    SDL_GPUTransferBufferCreateInfo transfer_info{};
    transfer_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    transfer_info.size = static_cast<uint32_t>(total);
    SDL_GPUTransferBuffer* transfer = SDL_CreateGPUTransferBuffer(m_device, &transfer_info);
    if (!transfer) {
        spdlog::error("Failed to create a texture transfer buffer ({} KB): {}",
                      total / 1024, SDL_GetError());
        return;
    }

    void* mapped = SDL_MapGPUTransferBuffer(m_device, transfer, false);
    if (!mapped) {
        spdlog::error("Failed to map the texture transfer buffer: {}", SDL_GetError());
        SDL_ReleaseGPUTransferBuffer(m_device, transfer);
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        const PendingTextureUpload& pending = m_pending[i];
        if (pending.staging_offset + pending.bytes > m_staging.size()) {
            continue;   // cannot happen; a truncated arena must not be read past
        }
        std::memcpy(static_cast<uint8_t*>(mapped) + bases[i],
                    m_staging.data() + pending.staging_offset, pending.bytes);
    }
    SDL_UnmapGPUTransferBuffer(m_device, transfer);

    // One SDL_UploadToGPUTexture per level, all of them inside the copy pass the
    // caller opened. No command buffer is acquired here and nothing is submitted:
    // this rides GPURenderer's single per-frame batch, which is the whole reason
    // the signature takes a copy pass rather than a device.
    size_t levels_copied = 0;
    for (size_t i = 0; i < count; ++i) {
        const PendingTextureUpload& pending = m_pending[i];
        if (pending.handle >= m_slots.size()) continue;
        TextureSlot& slot = m_slots[pending.handle];
        if (!slot.in_use || !slot.texture) continue;   // released before it flushed

        for (uint32_t level = 0; level < pending.mip_levels; ++level) {
            SDL_GPUTextureTransferInfo source{};
            source.transfer_buffer = transfer;
            source.offset = static_cast<uint32_t>(bases[i]
                          + packed_level_offset(slot.desc, level));
            source.pixels_per_row = 0;   // zero means "tightly packed at the region's size"
            source.rows_per_layer = 0;

            SDL_GPUTextureRegion region{};
            region.texture = slot.texture;
            region.mip_level = level;
            region.layer = 0;
            region.w = level_dimension(slot.desc.width, level);
            region.h = level_dimension(slot.desc.height, level);
            region.d = 1;

            // cycle = false: the texture was created for this copy and no draw has
            // ever read it, so there is nothing to cycle away from.
            SDL_UploadToGPUTexture(copy_pass, &source, &region, false);
            ++levels_copied;
        }
    }

    // NOT ready yet, and this is the whole point of the split. The copies are
    // ordered before every draw recorded later on the same command buffer, so
    // readiness is correct the moment that command buffer is SUBMITTED -- and
    // wrong if the submit fails, which is a survivable failure the mesh half of
    // GPURenderer::flush_pending_uploads() already rolls back. Marking the slots
    // ready here and dropping their staged bytes made that rollback impossible:
    // the textures would claim to be ready forever while holding uninitialised
    // device memory, with the pixels gone.
    m_in_flight.assign(m_pending.begin(), m_pending.begin() + static_cast<long>(count));
    m_pending.erase(m_pending.begin(), m_pending.begin() + static_cast<long>(count));

    // Retired, not released. A transfer buffer that a submitted-but-unfinished
    // command buffer is still reading is a use-after-free the validation layer
    // does not reliably catch -- the same argument, and the same constant, as
    // GPURenderer::retire_alloc().
    m_retired_transfers.push_back(RetiredTransfer{ transfer,
                                                   frame_index + kTransferRetireFrames });

    spdlog::debug("Texture flush: {} textures, {} mip levels, {} KB",
                  count, levels_copied, total / 1024);

    // The staging arena is deliberately NOT reclaimed here. Until the submit is
    // known, every byte behind an in-flight entry is the only copy of those pixels.
}

void GPUTextureManager::commit_uploads(bool submitted) {
    if (m_in_flight.empty()) return;

    if (!submitted) {
        // The copies never ran. Put the entries back at the FRONT of the queue,
        // in their original order, so they retry next frame -- their
        // staging_offsets are still valid because the arena was not compacted.
        m_pending.insert(m_pending.begin(), m_in_flight.begin(), m_in_flight.end());
        m_in_flight.clear();
        return;
    }

    for (const PendingTextureUpload& pending : m_in_flight) {
        if (pending.handle >= m_slots.size()) continue;
        TextureSlot& slot = m_slots[pending.handle];
        if (!slot.in_use || !slot.texture) continue;   // released before it flushed
        slot.ready = true;
    }
    m_in_flight.clear();

    if (m_pending.empty()) {
        m_staging.clear();
        m_staging.shrink_to_fit();
        return;
    }

    // Reclaim the dead prefix once at least half the arena is dead, so the memmove
    // of what is still queued is paid for. shrink_to_fit() is deliberately not
    // called: it would reallocate and copy the remainder a second time, doubling
    // peak host memory on a path that runs every frame of a material install.
    const size_t dead = std::min(m_pending.front().staging_offset, m_staging.size());
    if (dead * 2u >= m_staging.size() && dead > 0) {
        m_staging.erase(m_staging.begin(), m_staging.begin() + static_cast<long>(dead));
        for (PendingTextureUpload& pending : m_pending) {
            pending.staging_offset -= std::min(pending.staging_offset, dead);
        }
    }
}

void GPUTextureManager::drain_retired_transfers(uint64_t frame_index, bool force) {
    if (m_retired_transfers.empty()) return;

    size_t keep = 0;
    for (const RetiredTransfer& retired : m_retired_transfers) {
        if (!force && frame_index <= retired.retire_after_frame) {
            m_retired_transfers[keep++] = retired;   // still possibly in flight
            continue;
        }
        SDL_ReleaseGPUTransferBuffer(m_device, retired.buffer);
    }
    m_retired_transfers.resize(keep);
}

} // namespace stratum
