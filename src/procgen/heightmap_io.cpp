// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "procgen/heightmap_io.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <system_error>

// ============================================================================
// stb_image, compiled here as PNG-only and with internal linkage
// ============================================================================
//
// Two decisions in this block, both load-bearing.
//
// STB_IMAGE_STATIC: src/renderer/texture.cpp ALREADY instantiates
// STB_IMAGE_IMPLEMENTATION, and it does so with external linkage. That file is
// in stratum_editor_lib and this one is in stratum_core, and the `stratum`
// executable links both -- so a second external-linkage copy of stbi_load and
// friends is a duplicate-symbol link error the moment both objects are pulled
// from their archives. Making this copy static keeps the two independent. The
// alternative, hoisting stb into a shared translation unit, would put an image
// decoder shim in stratum_core that stratum_editor_lib has to reach back into,
// and stratum_core is the layer that is meant to have no such dependencies.
// The cost is one duplicated PNG decoder in the binary, which is a few tens of
// kilobytes.
//
// STBI_ONLY_PNG: this is an attack-surface decision, not a size one. stb_image
// ships ten decoders and by default sniffs all of them. A heightmap import has
// no reason to ever run the JPEG, PSD, GIF, PIC, PNM, HDR or TGA paths -- TGA
// in particular has no magic number and is stb's last-resort guess, so a
// malformed file that matches nothing else is handed to the decoder with the
// weakest validation in the set. Compiling the other nine out means a corrupt
// file cannot reach them at all.
//
// STBI_NO_STDIO: every load here goes through a buffer this file already
// bounds-checked. Letting stb open the path itself would bypass
// HeightmapImportOptions::max_file_bytes entirely.
//
// STBI_MAX_DIMENSIONS is a second line behind check_dimensions(): stb refuses
// the image during its own header parse, before any of its internal size
// arithmetic runs.
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#define STBI_FAILURE_USERMSG
#define STBI_MAX_DIMENSIONS 65536
#include <stb/stb_image.h>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace stratum::procgen {

// ============================================================================
// ElevationRange
// ============================================================================

bool ElevationRange::is_valid() const noexcept {
    if (!std::isfinite(m_min) || !std::isfinite(m_max)) {
        return false;
    }
    if (std::abs(m_min) > kMaxAbsElevationMetres || std::abs(m_max) > kMaxAbsElevationMetres) {
        return false;
    }
    // Strictly greater, not >=. See the is_valid() comment in the header: a zero
    // span is the typo, not the feature.
    return m_max > m_min;
}

// ============================================================================
// HeightmapPlacement
// ============================================================================

HeightmapPlacement HeightmapPlacement::geographic(const osm::BoundingBox& bounds,
                                                   const osm::CoordinateConverter& converter) {
    HeightmapPlacement placement;
    placement.m_kind = PlacementKind::Geographic;
    placement.m_bounds = bounds;
    placement.m_converter = converter;
    return placement;
}

HeightmapPlacement HeightmapPlacement::local(const glm::vec2& origin_local,
                                              float cell_size_x, float cell_size_y) {
    HeightmapPlacement placement;
    placement.m_kind = PlacementKind::Local;
    placement.m_origin = origin_local;
    placement.m_cell_x = cell_size_x;
    placement.m_cell_y = cell_size_y;
    return placement;
}

// ============================================================================
// Status and format names
// ============================================================================

const char* to_string(HeightmapImportStatus status) noexcept {
    switch (status) {
        case HeightmapImportStatus::Ok:                      return "Ok";
        case HeightmapImportStatus::FileNotFound:            return "FileNotFound";
        case HeightmapImportStatus::FileUnreadable:          return "FileUnreadable";
        case HeightmapImportStatus::FileTooLarge:            return "FileTooLarge";
        case HeightmapImportStatus::UnknownFormat:           return "UnknownFormat";
        case HeightmapImportStatus::MalformedHeader:         return "MalformedHeader";
        case HeightmapImportStatus::Truncated:               return "Truncated";
        case HeightmapImportStatus::SampleAboveMaxval:       return "SampleAboveMaxval";
        case HeightmapImportStatus::DimensionsOutOfRange:    return "DimensionsOutOfRange";
        case HeightmapImportStatus::UnsupportedChannelCount: return "UnsupportedChannelCount";
        case HeightmapImportStatus::DecodeFailed:            return "DecodeFailed";
        case HeightmapImportStatus::InvalidElevationRange:   return "InvalidElevationRange";
        case HeightmapImportStatus::InvalidPlacement:        return "InvalidPlacement";
        case HeightmapImportStatus::InvalidOptions:          return "InvalidOptions";
    }
    return "Unknown";
}

const char* to_string(HeightmapFormat format) noexcept {
    switch (format) {
        case HeightmapFormat::Unknown:   return "Unknown";
        case HeightmapFormat::PgmAscii:  return "PGM (P2, ASCII)";
        case HeightmapFormat::PgmBinary: return "PGM (P5, binary)";
        case HeightmapFormat::Png:       return "PNG";
    }
    return "Unknown";
}

namespace {

// ============================================================================
// Limits
// ============================================================================

/// Largest single-axis dimension accepted, independent of the sample-count cap.
///
/// A 65535-wide image is already past anything a DEM ships as, and the cap
/// exists so that `width * height` can be formed in a uint64 with no chance of
/// wrapping: two values below 2^16 multiply to below 2^32.
constexpr std::uint64_t kMaxSourceDimension = 65535;

/// Smallest usable axis.
///
/// Two samples, not one. A single-sample axis has no spacing, so `cell_size` is
/// a division by zero and the bilinear resample has no interval to interpolate
/// over. Refusing it here means no downstream code has to special-case it.
constexpr int kMinAxis = 2;

/// Ceiling on any integer parsed out of an ASCII header before it is range-checked.
///
/// Parsing stops the moment the accumulator passes this, so `999999...` with ten
/// thousand digits costs ten thousand comparisons and never overflows. The value
/// itself is meaningless beyond "far past any real dimension or maxval".
constexpr std::uint64_t kParseCeiling = 1ull << 40;

/// Largest value a PGM header may declare as its maxval, from the Netpbm spec.
constexpr std::uint64_t kMaxPgmMaxval = 65535;

// ============================================================================
// Small helpers
// ============================================================================

/// Netpbm whitespace. Deliberately not std::isspace: that is locale-dependent and
/// takes an int whose value must be representable as unsigned char, which is a
/// UB footgun on a signed-char platform fed bytes above 127 from a hostile file.
bool is_pgm_space(std::uint8_t c) noexcept {
    return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r';
}

bool is_digit(std::uint8_t c) noexcept { return c >= '0' && c <= '9'; }

/// Format a double into a short decimal string without dragging in <format>.
std::string num(double value, int decimals = 2) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
    return std::string{buffer};
}

HeightmapImportResult make_failure(HeightmapImportStatus status, std::string message) {
    HeightmapImportResult result;
    result.status = status;
    result.message = std::move(message);
    return result;
}

// ============================================================================
// Decoded intermediate
// ============================================================================

/**
 * @brief One decoded image, still in file row order and still in raw sample units
 *
 * Rows are in the order they appeared in the file: row 0 is the first row, which
 * for both PNG and PGM is the TOP of the picture. The north/south flip happens
 * during the resample, not here, so that the two decoders stay ignorant of
 * geography and the flip lives in exactly one place.
 */
struct DecodedImage {
    /// Raw samples, in file row order. A decoder that returns false leaves this
    /// in whatever state it had reached: the caller discards the whole
    /// DecodedImage on failure and copies only the header fields out of it, so
    /// no failure path clears this and none needs to. A clear that nothing can
    /// observe is not a safeguard, it is a line that makes the next reader
    /// believe there is something here to protect.
    std::vector<std::uint16_t> samples;
    int width = 0;
    int height = 0;
    std::uint32_t max_sample = 0;
    int bits_per_sample = 0;
    HeightmapFormat format = HeightmapFormat::Unknown;
    std::vector<std::string> warnings;
};

/**
 * @brief Reject a dimension pair before anything is allocated for it
 *
 * This is the guard SECURITY.md asks for and it runs on numbers that came
 * straight out of the file, before any multiplication that could wrap and before
 * any vector is sized. Order matters: the per-axis cap runs first so that the
 * `width * height` below cannot overflow.
 */
bool check_dimensions(std::uint64_t width, std::uint64_t height,
                      const HeightmapImportOptions& options,
                      HeightmapImportStatus& status, std::string& message) {
    if (width > kMaxSourceDimension || height > kMaxSourceDimension) {
        status = HeightmapImportStatus::DimensionsOutOfRange;
        message = "image claims " + std::to_string(width) + "x" + std::to_string(height) +
                  " samples; the per-axis limit is " + std::to_string(kMaxSourceDimension);
        return false;
    }
    if (width < static_cast<std::uint64_t>(kMinAxis) ||
        height < static_cast<std::uint64_t>(kMinAxis)) {
        status = HeightmapImportStatus::DimensionsOutOfRange;
        message = "image is " + std::to_string(width) + "x" + std::to_string(height) +
                  "; a height grid needs at least " + std::to_string(kMinAxis) +
                  " samples on each axis to have a cell size at all";
        return false;
    }
    // Safe: both factors are below 2^16 by the first check.
    if (width * height > options.max_samples) {
        status = HeightmapImportStatus::DimensionsOutOfRange;
        message = "image claims " + std::to_string(width * height) +
                  " samples; HeightmapImportOptions::max_samples is " +
                  std::to_string(options.max_samples);
        return false;
    }
    return true;
}

// ============================================================================
// PGM (Netpbm P2 / P5)
// ============================================================================

/**
 * @brief Byte cursor over the encoded file, used by the PGM header parser
 *
 * Every accessor is bounds-checked against the buffer the caller owns. There is
 * no path in here that indexes past `m_data.size()`, which is the whole reason
 * the parser is hand-written rather than using sscanf on a buffer that is not
 * guaranteed to be NUL-terminated.
 */
class ByteCursor {
public:
    explicit ByteCursor(std::span<const std::uint8_t> data) noexcept : m_data(data) {}

    [[nodiscard]] bool eof() const noexcept { return m_pos >= m_data.size(); }
    [[nodiscard]] std::uint8_t peek() const noexcept { return m_data[m_pos]; }
    void advance() noexcept { ++m_pos; }
    [[nodiscard]] std::size_t position() const noexcept { return m_pos; }
    [[nodiscard]] std::size_t remaining() const noexcept { return m_data.size() - m_pos; }
    [[nodiscard]] std::span<const std::uint8_t> rest() const noexcept {
        return m_data.subspan(m_pos);
    }

    /// Skip whitespace and `#` comments, as Netpbm allows between any two tokens.
    void skip_gap() noexcept {
        while (!eof()) {
            if (is_pgm_space(peek())) {
                advance();
            } else if (peek() == '#') {
                while (!eof() && peek() != '\n' && peek() != '\r') {
                    advance();
                }
            } else {
                return;
            }
        }
    }

    /**
     * @brief Read one decimal integer, stopping at the first non-digit
     *
     * Saturates at kParseCeiling rather than overflowing: a header of ten
     * thousand nines must be refused as out of range, not wrap to something small
     * and plausible. That wrap is exactly how a size check gets bypassed.
     *
     * @return false when no digit was present at the cursor
     */
    [[nodiscard]] bool read_uint(std::uint64_t& out) noexcept {
        if (eof() || !is_digit(peek())) {
            return false;
        }
        std::uint64_t value = 0;
        while (!eof() && is_digit(peek())) {
            if (value <= kParseCeiling) {
                value = value * 10 + static_cast<std::uint64_t>(peek() - '0');
            }
            advance();
        }
        out = value;
        return true;
    }

private:
    std::span<const std::uint8_t> m_data;
    std::size_t m_pos = 0;
};

bool decode_pgm(std::span<const std::uint8_t> bytes, const HeightmapImportOptions& options,
                DecodedImage& image, HeightmapImportStatus& status, std::string& message) {
    ByteCursor cursor{bytes};

    cursor.skip_gap();
    if (cursor.remaining() < 2 || cursor.peek() != 'P') {
        status = HeightmapImportStatus::UnknownFormat;
        message = "not a Netpbm file: missing the 'P' magic";
        return false;
    }
    cursor.advance();
    const std::uint8_t variant = cursor.peek();
    cursor.advance();
    if (variant != '2' && variant != '5') {
        status = HeightmapImportStatus::UnknownFormat;
        message = std::string{"Netpbm variant P"} + static_cast<char>(variant) +
                  " is not a greyscale heightmap; only P2 (ASCII) and P5 (binary) are read";
        return false;
    }
    const bool binary = (variant == '5');
    image.format = binary ? HeightmapFormat::PgmBinary : HeightmapFormat::PgmAscii;

    std::uint64_t width = 0;
    std::uint64_t height = 0;
    std::uint64_t maxval = 0;
    cursor.skip_gap();
    if (!cursor.read_uint(width)) {
        status = HeightmapImportStatus::MalformedHeader;
        message = "PGM header: expected a width after the magic";
        return false;
    }
    cursor.skip_gap();
    if (!cursor.read_uint(height)) {
        status = HeightmapImportStatus::MalformedHeader;
        message = "PGM header: expected a height after the width";
        return false;
    }
    cursor.skip_gap();
    if (!cursor.read_uint(maxval)) {
        status = HeightmapImportStatus::MalformedHeader;
        message = "PGM header: expected a maxval after the height";
        return false;
    }

    if (maxval == 0 || maxval > kMaxPgmMaxval) {
        status = HeightmapImportStatus::MalformedHeader;
        message = "PGM header: maxval is " + std::to_string(maxval) +
                  "; the Netpbm specification allows 1 to " + std::to_string(kMaxPgmMaxval);
        return false;
    }
    if (!check_dimensions(width, height, options, status, message)) {
        // Report what the header claimed even though nothing was decoded: a
        // "refused" message that does not say what it refused is unactionable.
        image.width = width <= kMaxSourceDimension ? static_cast<int>(width) : 0;
        image.height = height <= kMaxSourceDimension ? static_cast<int>(height) : 0;
        return false;
    }

    image.width = static_cast<int>(width);
    image.height = static_cast<int>(height);
    image.max_sample = static_cast<std::uint32_t>(maxval);
    image.bits_per_sample = maxval > 255 ? 16 : 8;

    const std::size_t count = static_cast<std::size_t>(width * height);

    if (binary) {
        // Netpbm: exactly ONE whitespace character separates the maxval from the
        // raster. Not "skip whitespace" -- the first raster byte of a 16-bit image
        // is very often 0x0A or 0x20, and skipping it as if it were part of the
        // separator shifts every sample by one byte and turns the whole heightmap
        // into noise.
        if (cursor.eof() || !is_pgm_space(cursor.peek())) {
            status = HeightmapImportStatus::MalformedHeader;
            message = "PGM header: the maxval must be followed by exactly one "
                      "whitespace character before the binary raster";
            return false;
        }
        cursor.advance();

        const std::size_t bytes_per_sample = image.bits_per_sample == 16 ? 2u : 1u;
        const std::uint64_t needed = static_cast<std::uint64_t>(count) * bytes_per_sample;
        const std::uint64_t available = cursor.remaining();
        if (available < needed) {
            status = HeightmapImportStatus::Truncated;
            message = "PGM header claims " + std::to_string(width) + "x" +
                      std::to_string(height) + " at " + std::to_string(bytes_per_sample * 8) +
                      " bits, needing " + std::to_string(needed) + " raster bytes, but only " +
                      std::to_string(available) + " remain in the file";
            return false;
        }
        if (available > needed + 1) {
            // One trailing byte is a final newline and is normal. More than that
            // means the file disagrees with its own header in the safe direction,
            // which is worth saying out loud without refusing a usable image.
            image.warnings.push_back("PGM has " + std::to_string(available - needed) +
                                     " bytes after the raster its header accounts for");
        }

        const std::span<const std::uint8_t> raster = cursor.rest();
        image.samples.resize(count);
        if (bytes_per_sample == 1) {
            for (std::size_t i = 0; i < count; ++i) {
                image.samples[i] = raster[i];
            }
        } else {
            // Netpbm stores 16-bit samples MOST SIGNIFICANT BYTE FIRST, which is
            // the opposite of this machine. Reading them as native little-endian
            // turns 0x1234 into 0x3412 -- a 74% error that still produces a
            // continuous-looking height field, because neighbouring DEM samples
            // usually share a high byte and so become neighbouring low bytes.
            for (std::size_t i = 0; i < count; ++i) {
                image.samples[i] = static_cast<std::uint16_t>(
                    (static_cast<std::uint16_t>(raster[i * 2]) << 8) |
                    static_cast<std::uint16_t>(raster[i * 2 + 1]));
            }
        }
    } else {
        // Sized from what the remaining bytes COULD hold, not from what the
        // header claims they hold. An ASCII sample is at least one digit and
        // needs at least one separator between it and the next, so N samples
        // cannot be encoded in fewer than 2N-1 bytes. Without this, the
        // seventeen bytes "P2\n8192 8192\n255\n" resize this vector to 128 MB
        // -- an amplification of about eight million to one, on the thread that
        // is drawing the editor -- and only then discover there is no raster at
        // all. That is the allocation the file comment promises never happens on
        // the strength of a number that came out of the file, and the P5 path
        // above already gets it right.
        //
        // The bound is deliberately loose: a real file spends four or five bytes
        // per sample, so nothing legal is ever refused by it. Its job is to cap
        // the allocation at roughly the size of the input, not to predict the
        // raster.
        const std::uint64_t minimum_bytes = static_cast<std::uint64_t>(count) * 2u - 1u;
        const std::uint64_t available = cursor.remaining();
        if (available < minimum_bytes) {
            status = HeightmapImportStatus::Truncated;
            message = "PGM header claims " + std::to_string(count) +
                      " ASCII samples, which need at least " + std::to_string(minimum_bytes) +
                      " bytes, but only " + std::to_string(available) + " remain in the file";
            return false;
        }

        image.samples.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            cursor.skip_gap();
            std::uint64_t value = 0;
            if (!cursor.read_uint(value)) {
                status = HeightmapImportStatus::Truncated;
                message = "PGM header claims " + std::to_string(count) +
                          " ASCII samples but the file ran out after " + std::to_string(i);
                return false;
            }
            if (value > maxval) {
                status = HeightmapImportStatus::SampleAboveMaxval;
                message = "PGM sample " + std::to_string(i) + " is " + std::to_string(value) +
                          ", above the declared maxval " + std::to_string(maxval);
                return false;
            }
            image.samples[i] = static_cast<std::uint16_t>(value);
        }
    }

    // A binary raster cannot be short here, but it CAN hold values above the
    // declared maxval, and those normalise to more than 1.0 and land above
    // ElevationRange::max_metres(). Checked for both variants so the guarantee
    // "no returned height is outside the range you asked for" holds whatever the
    // file did.
    if (binary && maxval < kMaxPgmMaxval) {
        for (std::size_t i = 0; i < count; ++i) {
            if (image.samples[i] > maxval) {
                status = HeightmapImportStatus::SampleAboveMaxval;
                message = "PGM sample " + std::to_string(i) + " is " +
                          std::to_string(image.samples[i]) + ", above the declared maxval " +
                          std::to_string(maxval);
                return false;
            }
        }
    }

    return true;
}

// ============================================================================
// PNG, through stb_image
// ============================================================================

std::string stb_reason() {
    const char* reason = stbi_failure_reason();
    return reason != nullptr ? std::string{reason} : std::string{"no reason given"};
}

/**
 * @brief Read the declared bit depth out of the PNG's IHDR chunk
 *
 * stb answers only "is it 16 bits?", and PNG greyscale is legally 1, 2, 4, 8 or
 * 16. Every depth below 16 therefore reads back as 8 unless the file is asked
 * directly, and stb SCALES a sub-byte sample up to fill 0..255 on the way out --
 * so the heights are right and the lie is completely silent. max_sample_value
 * would say 255 for a 1-bit file, vertical_quantum_metres would divide the range
 * by 255 instead of by 1, and the coarseness warning that exists to catch a
 * quantum this large would be suppressed because the understated figure falls
 * below kCoarseVerticalQuantumMetres. Over a 0-200 m range that is 0.78 m
 * reported against 200 m real: the factor-of-N lie this whole file is written to
 * prevent, reachable from an untrusted file.
 *
 * The layout is fixed by the PNG specification and needs no chunk walk: an
 * 8-byte signature, then IHDR as a 4-byte length, the 4-byte type "IHDR", a
 * 4-byte width and a 4-byte height. The bit depth is the next byte, byte 24 of
 * the file. IHDR must be the first chunk, so anything else there is malformed.
 *
 * @param bytes Whole encoded file, already known to carry the PNG signature
 * @param depth Set to the declared depth on success; untouched otherwise
 * @return false when the file is too short for an IHDR or does not start with one
 */
bool png_ihdr_bit_depth(std::span<const std::uint8_t> bytes, int& depth) noexcept {
    constexpr std::size_t kIhdrTypeOffset = 12;
    constexpr std::size_t kBitDepthOffset = 24;
    if (bytes.size() <= kBitDepthOffset) {
        return false;
    }
    if (std::memcmp(bytes.data() + kIhdrTypeOffset, "IHDR", 4) != 0) {
        return false;
    }
    depth = static_cast<int>(bytes[kBitDepthOffset]);
    return true;
}

bool decode_png(std::span<const std::uint8_t> bytes, const HeightmapImportOptions& options,
                DecodedImage& image, HeightmapImportStatus& status, std::string& message) {
    image.format = HeightmapFormat::Png;

    if (bytes.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        status = HeightmapImportStatus::FileTooLarge;
        message = "PNG is " + std::to_string(bytes.size()) +
                  " bytes; stb_image takes an int length";
        return false;
    }
    const int length = static_cast<int>(bytes.size());

    int width = 0;
    int height = 0;
    int channels = 0;
    if (stbi_info_from_memory(bytes.data(), length, &width, &height, &channels) == 0) {
        status = HeightmapImportStatus::MalformedHeader;
        message = "PNG header could not be read: " + stb_reason();
        return false;
    }
    if (width <= 0 || height <= 0) {
        status = HeightmapImportStatus::DimensionsOutOfRange;
        message = "PNG header reports a non-positive size";
        return false;
    }
    // Before the decode, not after. stbi_load_*_from_memory would otherwise
    // allocate width*height*channels for us on the strength of the header.
    if (!check_dimensions(static_cast<std::uint64_t>(width), static_cast<std::uint64_t>(height),
                          options, status, message)) {
        image.width = width;
        image.height = height;
        return false;
    }

    if (channels >= 3) {
        // Refused rather than converted. stb's desired_channels=1 would happily
        // collapse RGB to a luminance grey, and for the one colour DEM encoding
        // anybody actually ships -- Mapbox Terrain-RGB, where elevation is a
        // 24-bit big-endian integer split across R, G and B -- the luminance of
        // those three bytes is not a monotonic function of the elevation at all.
        // The result is terrain-shaped, wrong everywhere, and gives no hint why.
        status = HeightmapImportStatus::UnsupportedChannelCount;
        message = "PNG has " + std::to_string(channels) +
                  " channels; a heightmap must be greyscale. A packed colour DEM "
                  "(for example Mapbox Terrain-RGB) is a different encoding and is "
                  "not decoded by averaging its channels";
        return false;
    }

    // The TRUE source depth, read out of IHDR. Asking stb whether the file is
    // 16-bit answers only that question, and PNG greyscale is legally 1, 2, 4, 8
    // or 16: everything below 16 would come back as "8" and be reported as 8-bit
    // with a full scale of 255 it does not have. See png_ihdr_bit_depth().
    int depth = 0;
    if (!png_ihdr_bit_depth(bytes, depth)) {
        status = HeightmapImportStatus::MalformedHeader;
        message = "PNG does not begin with an IHDR chunk";
        return false;
    }
    if (depth != 1 && depth != 2 && depth != 4 && depth != 8 && depth != 16) {
        status = HeightmapImportStatus::MalformedHeader;
        message = "PNG declares a bit depth of " + std::to_string(depth) +
                  "; the specification allows 1, 2, 4, 8 and 16";
        return false;
    }

    const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    image.width = width;
    image.height = height;
    image.bits_per_sample = depth;
    // Full scale is what the DEPTH can represent: 1, 3, 15, 255, 65535. This is
    // the denominator vertical_quantum_metres is computed from, so it has to be
    // the file's own scale and not the scale stb hands the samples back on.
    image.max_sample = (1u << static_cast<unsigned int>(depth)) - 1u;

    if (depth == 16) {
        int out_w = 0;
        int out_h = 0;
        int out_c = 0;
        stbi_us* pixels = stbi_load_16_from_memory(bytes.data(), length, &out_w, &out_h, &out_c, 1);
        if (pixels == nullptr) {
            status = HeightmapImportStatus::DecodeFailed;
            message = "PNG raster could not be decoded: " + stb_reason();
            return false;
        }
        // Sized from the decode that SUCCEEDED, not from the header. Sizing it
        // before the call put a second buffer of width*height alongside stb's
        // own, on the strength of two numbers out of the file, and kept it for a
        // file whose IDAT could never have filled it.
        image.samples.assign(pixels, pixels + count);
        stbi_image_free(pixels);
    } else {
        int out_w = 0;
        int out_h = 0;
        int out_c = 0;
        stbi_uc* pixels = stbi_load_from_memory(bytes.data(), length, &out_w, &out_h, &out_c, 1);
        if (pixels == nullptr) {
            status = HeightmapImportStatus::DecodeFailed;
            message = "PNG raster could not be decoded: " + stb_reason();
            return false;
        }
        // stb expands a sub-byte greyscale sample to FILL 0..255 by multiplying
        // by a fixed scale -- 255, 85 and 17 for 1, 2 and 4 bits, which is
        // exactly 255 / max_sample. Dividing it back out is lossless and
        // recovers the value the file stored. Skipping it would hand a 1-bit
        // sample back as 255 against a max_sample of 1, and every set pixel
        // would normalise to 255 times full scale.
        const unsigned int expansion = 255u / image.max_sample;
        image.samples.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            image.samples[i] = static_cast<std::uint16_t>(pixels[i] / expansion);
        }
        stbi_image_free(pixels);
    }

    return true;
}

// ============================================================================
// Format sniffing
// ============================================================================

constexpr std::uint8_t kPngSignature[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};

bool looks_like_png(std::span<const std::uint8_t> bytes) noexcept {
    return bytes.size() >= sizeof(kPngSignature) &&
           std::memcmp(bytes.data(), kPngSignature, sizeof(kPngSignature)) == 0;
}

// ============================================================================
// Resample and place
// ============================================================================

/**
 * @brief Resolve the placement into the Heightmap's origin and cell sizes
 *
 * @param dst_w Destination grid width, already validated as >= kMinAxis
 * @param dst_h Destination grid height, already validated as >= kMinAxis
 */
bool apply_placement(const HeightmapPlacement& placement, int dst_w, int dst_h,
                     HeightmapImportResult& result) {
    Heightmap& grid = result.heightmap;

    if (placement.kind() == PlacementKind::Local) {
        const float cx = placement.cell_size_x();
        const float cy = placement.cell_size_y();
        if (!(cx > 0.0f) || !(cy > 0.0f) || !std::isfinite(cx) || !std::isfinite(cy)) {
            result.status = HeightmapImportStatus::InvalidPlacement;
            result.message = "local placement needs strictly positive, finite cell sizes";
            return false;
        }
        const glm::vec2 origin = placement.origin_local();
        if (!std::isfinite(origin.x) || !std::isfinite(origin.y)) {
            result.status = HeightmapImportStatus::InvalidPlacement;
            result.message = "local placement origin is not finite";
            return false;
        }
        grid.origin = origin;
        grid.cell_size_x = cx;
        grid.cell_size_z = cy;
        // No projection happened, so there is no Mercator stretch to report and
        // no centre to report. Left at the neutral values rather than at a
        // guess, so a caller that multiplies by ground_metres_per_unit gets the
        // identity instead of a number derived from a place that was never named.
        result.ground_metres_per_unit = 1.0;
        result.centre_lat = 0.0;
        result.centre_lon = 0.0;
        return true;
    }

    const osm::BoundingBox& bounds = placement.bounds();
    if (!bounds.is_valid()) {
        result.status = HeightmapImportStatus::InvalidPlacement;
        result.message = "geographic placement has invalid bounds (min_lat " +
                         num(bounds.min_lat, 6) + " > max_lat " + num(bounds.max_lat, 6) +
                         ", or min_lon > max_lon). A default-constructed BoundingBox is "
                         "deliberately inverted and reads as 'never set'";
        return false;
    }
    // Web Mercator is undefined past +/-85.051128 degrees and
    // CoordinateConverter::wgs84_to_mercator() CLAMPS silently there. A box that
    // straddles the clamp would come back with a shorter north-south span than it
    // asked for and the terrain would be squashed with no diagnostic at all.
    constexpr double kMercatorLatLimit = 85.051128;
    if (bounds.min_lat < -kMercatorLatLimit || bounds.max_lat > kMercatorLatLimit ||
        bounds.min_lon < -180.0 || bounds.max_lon > 180.0) {
        result.status = HeightmapImportStatus::InvalidPlacement;
        result.message = "geographic placement lies outside the Web Mercator domain "
                         "(latitude +/-" + num(kMercatorLatLimit, 6) + ", longitude +/-180)";
        return false;
    }
    if (!placement.converter().is_initialized()) {
        // The single most expensive trap in this codebase to debug.
        // CoordinateConverter::wgs84_to_local() returns RAW Web Mercator when
        // set_origin() was never called -- coordinates in the millions instead of
        // the thousands, no log line, no failure. The terrain lands thousands of
        // kilometres from the buildings and the first symptom looks like a
        // renderer bug. Refusing here is the whole reason the converter is a
        // required argument instead of an optional one.
        result.status = HeightmapImportStatus::InvalidPlacement;
        result.message = "the CoordinateConverter has no origin: call set_origin() first. "
                         "Without it wgs84_to_local() silently returns raw Web Mercator and "
                         "the terrain would be placed thousands of kilometres from the scene";
        return false;
    }

    const glm::dvec2 south_west =
        placement.converter().wgs84_to_local(bounds.min_lat, bounds.min_lon);
    const glm::dvec2 north_east =
        placement.converter().wgs84_to_local(bounds.max_lat, bounds.max_lon);

    const double span_x = north_east.x - south_west.x;
    const double span_y = north_east.y - south_west.y;
    if (!std::isfinite(span_x) || !std::isfinite(span_y) || span_x <= 0.0 || span_y <= 0.0) {
        result.status = HeightmapImportStatus::InvalidPlacement;
        result.message = "geographic placement projects to a degenerate extent (" +
                         num(span_x) + " x " + num(span_y) +
                         " local metres); the bounds have no area";
        return false;
    }

    // Sample (0, 0) is the SOUTH-WEST corner, because local y increases
    // northward and the resample below writes row 0 as the south edge.
    grid.origin = glm::vec2{static_cast<float>(south_west.x), static_cast<float>(south_west.y)};
    // Divided by (n - 1), not by n: the grid is pixel-is-POINT. The corner
    // samples sit exactly on the corners of the extent, so a round trip through
    // any target resolution keeps the edges on the same meridians and parallels
    // and adjacent tiles share their border row exactly. Dividing by n would put
    // the samples at cell centres and leave a half-cell of the extent
    // unrepresented on each side, which is where seams between tiles come from.
    grid.cell_size_x = static_cast<float>(span_x / (dst_w - 1));
    grid.cell_size_z = static_cast<float>(span_y / (dst_h - 1));

    // BoundingBox::center() returns (lat, lon) in a dvec2 -- .x is the LATITUDE.
    // Every other dvec2 in this codebase is (x, y) or (lon-ish, lat-ish), so this
    // one is backwards, and CLAUDE.md lists it as a trap. Unpacked into two named
    // doubles immediately, and never passed on as a vector, so the ordering
    // cannot be lost a second time downstream.
    const glm::dvec2 centre_latlon = bounds.center();
    result.centre_lat = centre_latlon.x;
    result.centre_lon = centre_latlon.y;
    result.ground_metres_per_unit = std::cos(result.centre_lat * osm::DEG_TO_RAD);
    return true;
}

/**
 * @brief Resample the decoded image onto the destination grid
 *
 * Two conventions, both deliberate.
 *
 * PIXEL-IS-POINT. Destination sample j of n maps to source coordinate
 * `j/(n-1) * (m-1)`, so the first and last destination samples land exactly on
 * the first and last source samples. Nothing outside the source is ever read and
 * no edge extrapolation rule is needed: the border of the output IS the border of
 * the input, not a half-pixel guess beyond it. The alternative, pixel-is-area,
 * puts samples at cell centres and then genuinely needs an invented value for the
 * outer half-cell -- and whatever it invents shows up as a visible lip around
 * every imported tile.
 *
 * ROW FLIP. Destination row 0 is the SOUTH edge, because a Heightmap's second
 * axis is local y and local y increases northward. Image row 0 is the NORTH edge.
 * The flip is applied here, once, and is switchable through
 * HeightmapImportOptions::source_row_zero_is_north.
 *
 * Outside the destination grid, nothing is invented either: Heightmap::sample()
 * clamps to the edge value (terrain_generator.cpp), so a query past the imported
 * extent returns the border height rather than extrapolating a slope off to
 * infinity.
 */
void resample(const DecodedImage& image, const ElevationRange& vertical,
              const HeightmapImportOptions& options, int dst_w, int dst_h, Heightmap& grid) {
    grid.width = dst_w;
    grid.height = dst_h;
    grid.data.assign(static_cast<std::size_t>(dst_w) * static_cast<std::size_t>(dst_h), 0.0f);

    const int src_w = image.width;
    const int src_h = image.height;
    const double inv_max = 1.0 / static_cast<double>(image.max_sample);

    // Clamped HERE, at the read, and not only where x1 and y1 are computed.
    //
    // The far taps are weighted by fx and fy, and on the right and top edges of
    // the source those weights are EXACTLY zero -- so an index one past the end
    // there is multiplied away before it reaches the output. No assertion on any
    // returned height can see it; only a sanitizer can, and this project has no
    // sanitizer preset. A bound whose failure is invisible is not a bound, and
    // SECURITY.md names out-of-bounds reads from a malformed file as the most
    // likely real vulnerability in this parser. So the only index that reaches
    // the vector is one this function has already clamped, and an off-by-one in
    // the interpolation arithmetic above costs a duplicated edge sample rather
    // than a read past the buffer.
    const auto raw = [&](int x, int y) -> double {
        const std::size_t cx = static_cast<std::size_t>(std::clamp(x, 0, src_w - 1));
        const std::size_t cy = static_cast<std::size_t>(std::clamp(y, 0, src_h - 1));
        return static_cast<double>(image.samples[cy * static_cast<std::size_t>(src_w) + cx]);
    };

    for (int dz = 0; dz < dst_h; ++dz) {
        // northness: 0 at the south edge of the extent, 1 at the north edge.
        const double northness = static_cast<double>(dz) / static_cast<double>(dst_h - 1);
        const double row_fraction = options.source_row_zero_is_north ? 1.0 - northness : northness;
        const double sy = row_fraction * static_cast<double>(src_h - 1);
        // y0 is clamped because fy is measured from it; y1 is not, because raw()
        // clamps every index it is given and a second clamp here would only hide
        // an arithmetic slip rather than survive one.
        const int y0 = std::clamp(static_cast<int>(std::floor(sy)), 0, src_h - 1);
        const int y1 = y0 + 1;
        const double fy = std::clamp(sy - static_cast<double>(y0), 0.0, 1.0);

        for (int dx = 0; dx < dst_w; ++dx) {
            const double eastness = static_cast<double>(dx) / static_cast<double>(dst_w - 1);
            const double sx = eastness * static_cast<double>(src_w - 1);
            const int x0 = std::clamp(static_cast<int>(std::floor(sx)), 0, src_w - 1);
            const int x1 = x0 + 1;
            const double fx = std::clamp(sx - static_cast<double>(x0), 0.0, 1.0);

            const double top = raw(x0, y0) * (1.0 - fx) + raw(x1, y0) * fx;
            const double bottom = raw(x0, y1) * (1.0 - fx) + raw(x1, y1) * fx;
            const double blended = top * (1.0 - fy) + bottom * fy;

            grid.data[static_cast<std::size_t>(dz) * static_cast<std::size_t>(dst_w) +
                      static_cast<std::size_t>(dx)] =
                static_cast<float>(vertical.to_metres(blended * inv_max));
        }
    }
}

} // namespace

// ============================================================================
// Entry points
// ============================================================================

HeightmapImportResult import_heightmap_from_memory(std::span<const std::uint8_t> bytes,
                                                    const ElevationRange& vertical,
                                                    const HeightmapPlacement& placement,
                                                    const HeightmapImportOptions& options) {
    // Inputs first, file second. There is no point decoding eight megabytes of
    // PNG to then discover the caller passed metres(0, 0).
    if (!vertical.is_valid()) {
        return make_failure(HeightmapImportStatus::InvalidElevationRange,
                            "elevation range " + num(vertical.min_metres()) + " to " +
                                num(vertical.max_metres()) +
                                " m is not usable: it must be finite, strictly increasing, and "
                                "within +/-" + num(kMaxAbsElevationMetres, 0) + " m");
    }
    if (options.max_samples == 0) {
        return make_failure(HeightmapImportStatus::InvalidOptions,
                            "HeightmapImportOptions::max_samples is 0, which refuses every image");
    }
    if (options.target_width < 0 || options.target_height < 0) {
        return make_failure(HeightmapImportStatus::InvalidOptions,
                            "target grid size may not be negative");
    }
    if (bytes.empty()) {
        return make_failure(HeightmapImportStatus::UnknownFormat, "the file is empty");
    }

    DecodedImage image;
    HeightmapImportStatus status = HeightmapImportStatus::DecodeFailed;
    std::string message;

    // Sniffed from the leading bytes, never from the extension. A DEM renamed by
    // a download manager is still the format it was, and a .png that is really a
    // JPEG should be refused by name rather than producing "corrupt PNG".
    const bool decoded = looks_like_png(bytes)
                             ? decode_png(bytes, options, image, status, message)
                             : decode_pgm(bytes, options, image, status, message);
    if (!decoded) {
        HeightmapImportResult failure = make_failure(status, std::move(message));
        failure.source.format = image.format;
        failure.source.width = image.width;
        failure.source.height = image.height;
        failure.source.bits_per_sample = image.bits_per_sample;
        failure.source.max_sample_value = image.max_sample;
        return failure;
    }

    const int dst_w = options.target_width > 0 ? options.target_width : image.width;
    const int dst_h = options.target_height > 0 ? options.target_height : image.height;
    if (!check_dimensions(static_cast<std::uint64_t>(dst_w), static_cast<std::uint64_t>(dst_h),
                          options, status, message)) {
        return make_failure(HeightmapImportStatus::InvalidOptions,
                            "target grid rejected: " + message);
    }

    HeightmapImportResult result;
    result.status = HeightmapImportStatus::Ok;
    result.source.format = image.format;
    result.source.width = image.width;
    result.source.height = image.height;
    result.source.bits_per_sample = image.bits_per_sample;
    result.source.max_sample_value = image.max_sample;
    result.warnings = std::move(image.warnings);

    if (!apply_placement(placement, dst_w, dst_h, result)) {
        // No reset needed and none written: apply_placement() validates before it
        // writes anything, and resample() has not run, so the grid is still the
        // default -- empty data, one-metre cells, origin at zero. The suite
        // asserts exactly that rather than trusting a clearing statement whose
        // removal nothing could detect.
        return result;
    }

    resample(image, vertical, options, dst_w, dst_h, result.heightmap);

    result.vertical_quantum_metres =
        vertical.span_metres() / static_cast<double>(image.max_sample);
    if (result.vertical_quantum_metres > kCoarseVerticalQuantumMetres) {
        // The point of the number, not just the number. An 8-bit PNG over a
        // 1200 m range resolves 4.71 m, so every slope shallower than that
        // becomes a flight of one-cell terraces, and the terrain normals -- which
        // are a finite difference of these heights -- go piecewise constant and
        // band the lighting.
        result.warnings.push_back(
            std::to_string(image.bits_per_sample) + "-bit source over a " +
            num(vertical.span_metres()) + " m range resolves only " +
            num(result.vertical_quantum_metres, 3) +
            " m per sample step; shallow slopes will terrace. A 16-bit source over "
            "the same range resolves " +
            num(vertical.span_metres() / 65535.0, 4) + " m");
    }

    if (dst_w * 2 < image.width || dst_h * 2 < image.height) {
        // Bilinear is a point sample of four neighbours, not an area average, so
        // downsampling by more than 2x misses whatever fell between the samples.
        // On a DEM that means a ridge line or a cutting narrower than a
        // destination cell can vanish entirely rather than being averaged in.
        result.warnings.push_back(
            "downsampling " + std::to_string(image.width) + "x" + std::to_string(image.height) +
            " to " + std::to_string(dst_w) + "x" + std::to_string(dst_h) +
            " by more than 2x per axis; bilinear sampling does not area-average, so "
            "features narrower than a destination cell may be skipped");
    }

    return result;
}

HeightmapImportResult import_heightmap(const std::filesystem::path& path,
                                        const ElevationRange& vertical,
                                        const HeightmapPlacement& placement,
                                        const HeightmapImportOptions& options) {
    // std::error_code overloads throughout: a path on an unmounted share or with
    // a permission fault must come back as a status, not as an exception thrown
    // out of a function the header promises does not throw.
    std::error_code error;
    if (!std::filesystem::exists(path, error) || error) {
        return make_failure(HeightmapImportStatus::FileNotFound,
                            "no such file: " + path.string());
    }
    if (!std::filesystem::is_regular_file(path, error) || error) {
        return make_failure(HeightmapImportStatus::FileUnreadable,
                            "not a regular file: " + path.string());
    }

    const std::uintmax_t size = std::filesystem::file_size(path, error);
    if (error) {
        return make_failure(HeightmapImportStatus::FileUnreadable,
                            "could not stat " + path.string() + ": " + error.message());
    }
    // Checked BEFORE the read, which is the only place it can do any good. A
    // 40 GB file must cost a stat, not 40 GB of resident memory.
    if (static_cast<std::uint64_t>(size) > options.max_file_bytes) {
        return make_failure(HeightmapImportStatus::FileTooLarge,
                            path.string() + " is " + std::to_string(size) +
                                " bytes; HeightmapImportOptions::max_file_bytes is " +
                                std::to_string(options.max_file_bytes));
    }

    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        return make_failure(HeightmapImportStatus::FileUnreadable,
                            "could not open " + path.string());
    }

    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
    if (size > 0) {
        stream.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(size));
        // gcount(), not the stream state: a short read means the file changed
        // under us between the stat and the read, and decoding the prefix as if
        // it were whole is precisely the truncation bug this file refuses
        // everywhere else.
        if (static_cast<std::uint64_t>(stream.gcount()) != static_cast<std::uint64_t>(size)) {
            return make_failure(HeightmapImportStatus::FileUnreadable,
                                "short read on " + path.string() + ": expected " +
                                    std::to_string(size) + " bytes, got " +
                                    std::to_string(stream.gcount()));
        }
    }

    HeightmapImportResult result =
        import_heightmap_from_memory(std::span<const std::uint8_t>{bytes}, vertical, placement,
                                     options);
    if (!result.ok() && !result.message.empty()) {
        result.message = path.string() + ": " + result.message;
    }
    return result;
}

} // namespace stratum::procgen
