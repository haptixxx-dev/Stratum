// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_heightmap_io.cpp
 * @brief Suite HeightmapIO -- elevation image import
 *
 * Every fixture in this file is BUILT IN THE TEST, byte by byte, including the
 * PNGs. Nothing binary is checked into the repository.
 *
 * That is not tidiness. A checked-in fixture proves that the decoder agrees with
 * whatever tool made the fixture, on the day it was made, and it goes stale
 * silently. A fixture assembled here states the bytes and the expected heights in
 * the same function, twenty lines apart, so the assertion is checkable by reading
 * it. It is also the only way to write the malformed cases at all: there is no
 * tool that emits "a PGM whose header claims 100x100 and then stops".
 *
 * The PNG writer below emits real PNGs -- signature, IHDR, IDAT, IEND, correct
 * CRC32 and Adler-32 -- using DEFLATE stored blocks so no compressor is needed.
 * stb_image decodes them, so the 16-bit path, the 8-bit path and the colour
 * refusal are all exercised against the real decoder rather than a stand-in.
 *
 * Note on what makes a test here real: several of these are designed so that the
 * obvious WRONG implementation still produces a valid-looking heightmap. The row
 * flip, the big-endian 16-bit read, the normalisation denominator and the
 * (lat, lon) ordering of BoundingBox::center() all have that property, so each
 * fixture is deliberately asymmetric and each expected value is one that only the
 * correct implementation produces.
 */

#include "framework.hpp"

#include "osm/coordinates.hpp"
#include "osm/types.hpp"
#include "procgen/heightmap_io.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

using stratum::procgen::ElevationRange;
using stratum::procgen::Heightmap;
using stratum::procgen::HeightmapFormat;
using stratum::procgen::HeightmapImportOptions;
using stratum::procgen::HeightmapImportResult;
using stratum::procgen::HeightmapImportStatus;
using stratum::procgen::HeightmapPlacement;
using stratum::procgen::import_heightmap;
using stratum::procgen::import_heightmap_from_memory;

namespace {

using Bytes = std::vector<std::uint8_t>;

// ============================================================================
// Fixture construction
// ============================================================================

void append_ascii(Bytes& out, const std::string& text) {
    out.insert(out.end(), text.begin(), text.end());
}

/**
 * @brief A binary (P5) PGM
 *
 * @param declared_w Width written into the header
 * @param declared_h Height written into the header
 * @param maxval     Declared maxval; also chooses 1- or 2-byte samples
 * @param samples    Raster, in file row order (row 0 first). May deliberately be
 *                   shorter than declared_w * declared_h, which is how the
 *                   truncation cases are built.
 */
Bytes make_pgm_binary(int declared_w, int declared_h, int maxval,
                      const std::vector<std::uint16_t>& samples) {
    Bytes out;
    append_ascii(out, "P5\n" + std::to_string(declared_w) + " " + std::to_string(declared_h) +
                          "\n" + std::to_string(maxval) + "\n");
    for (std::uint16_t sample : samples) {
        if (maxval > 255) {
            // Netpbm is most-significant-byte-first.
            out.push_back(static_cast<std::uint8_t>((sample >> 8) & 0xFF));
            out.push_back(static_cast<std::uint8_t>(sample & 0xFF));
        } else {
            out.push_back(static_cast<std::uint8_t>(sample & 0xFF));
        }
    }
    return out;
}

/// An ASCII (P2) PGM. Same parameters as make_pgm_binary().
Bytes make_pgm_ascii(int declared_w, int declared_h, int maxval,
                     const std::vector<std::uint16_t>& samples) {
    Bytes out;
    append_ascii(out, "P2\n" + std::to_string(declared_w) + " " + std::to_string(declared_h) +
                          "\n" + std::to_string(maxval) + "\n");
    for (std::size_t i = 0; i < samples.size(); ++i) {
        append_ascii(out, std::to_string(samples[i]));
        append_ascii(out, (i % 8 == 7) ? "\n" : " ");
    }
    return out;
}

void put_be32(Bytes& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
}

/// PNG's CRC-32 (IEEE 802.3, reflected, pre/post inverted).
std::uint32_t crc32_of(const std::uint8_t* data, std::size_t length) {
    static std::uint32_t table[256];
    static bool built = false;
    if (!built) {
        for (std::uint32_t n = 0; n < 256; ++n) {
            std::uint32_t c = n;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1u) != 0 ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            table[n] = c;
        }
        built = true;
    }
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < length; ++i) {
        crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

std::uint32_t adler32_of(const Bytes& data) {
    std::uint32_t a = 1;
    std::uint32_t b = 0;
    for (std::uint8_t byte : data) {
        a = (a + byte) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

void push_chunk(Bytes& out, const char* type, const Bytes& payload) {
    put_be32(out, static_cast<std::uint32_t>(payload.size()));
    Bytes crc_input;
    crc_input.insert(crc_input.end(), type, type + 4);
    crc_input.insert(crc_input.end(), payload.begin(), payload.end());
    out.insert(out.end(), crc_input.begin(), crc_input.end());
    put_be32(out, crc32_of(crc_input.data(), crc_input.size()));
}

/// A zlib stream wrapping DEFLATE stored (uncompressed) blocks.
///
/// Stored blocks are legal DEFLATE and need no compressor, which is what makes a
/// hand-written PNG a reasonable thing to have in a test file at all.
Bytes zlib_stored(const Bytes& raw) {
    Bytes out;
    out.push_back(0x78); // CM = deflate, CINFO = 32K window
    out.push_back(0x01); // no preset dictionary; (0x78 * 256 + 0x01) % 31 == 0
    std::size_t pos = 0;
    do {
        const std::size_t chunk = std::min<std::size_t>(65535, raw.size() - pos);
        const bool final_block = (pos + chunk == raw.size());
        out.push_back(final_block ? 0x01 : 0x00); // BFINAL, BTYPE = 00 (stored)
        out.push_back(static_cast<std::uint8_t>(chunk & 0xFF));
        out.push_back(static_cast<std::uint8_t>((chunk >> 8) & 0xFF));
        const std::uint16_t nlen = static_cast<std::uint16_t>(~static_cast<std::uint16_t>(chunk));
        out.push_back(static_cast<std::uint8_t>(nlen & 0xFF));
        out.push_back(static_cast<std::uint8_t>((nlen >> 8) & 0xFF));
        out.insert(out.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos),
                   raw.begin() + static_cast<std::ptrdiff_t>(pos + chunk));
        pos += chunk;
    } while (pos < raw.size());
    put_be32(out, adler32_of(raw));
    return out;
}

/**
 * @brief A real PNG
 *
 * @param width        Image width
 * @param height       Image height
 * @param bit_depth    1, 2, 4, 8 or 16. Greyscale PNG allows all five; the
 *                     sub-byte ones are what catch a decoder that reports the
 *                     depth stb hands the pixels back at rather than the depth
 *                     the file was written in.
 * @param colour_type  0 for greyscale, 2 for truecolour RGB
 * @param samples      width * height values for greyscale, width * height * 3 for RGB,
 *                     in file row order
 */
Bytes make_png(int width, int height, int bit_depth, int colour_type,
               const std::vector<std::uint16_t>& samples) {
    const int channels = (colour_type == 2) ? 3 : 1;
    const int bytes_per_sample = bit_depth / 8;

    Bytes raw;
    for (int y = 0; y < height; ++y) {
        raw.push_back(0x00); // filter type 0 (None)
        if (bit_depth < 8) {
            // Sub-byte samples are packed MOST SIGNIFICANT BITS FIRST and each
            // ROW is padded out to a whole byte -- rows never share one. Only
            // greyscale reaches here, so there is one channel.
            std::uint8_t accumulator = 0;
            int bits_filled = 0;
            for (int x = 0; x < width; ++x) {
                const std::uint16_t value =
                    samples[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                            static_cast<std::size_t>(x)];
                const unsigned int mask = (1u << bit_depth) - 1u;
                accumulator = static_cast<std::uint8_t>((accumulator << bit_depth) |
                                                        (static_cast<unsigned int>(value) & mask));
                bits_filled += bit_depth;
                if (bits_filled == 8) {
                    raw.push_back(accumulator);
                    accumulator = 0;
                    bits_filled = 0;
                }
            }
            if (bits_filled != 0) {
                raw.push_back(static_cast<std::uint8_t>(accumulator << (8 - bits_filled)));
            }
            continue;
        }
        for (int x = 0; x < width * channels; ++x) {
            const std::uint16_t value =
                samples[static_cast<std::size_t>(y) * static_cast<std::size_t>(width * channels) +
                        static_cast<std::size_t>(x)];
            if (bytes_per_sample == 2) {
                raw.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
            }
            raw.push_back(static_cast<std::uint8_t>(value & 0xFF));
        }
    }

    Bytes out = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};

    Bytes ihdr;
    put_be32(ihdr, static_cast<std::uint32_t>(width));
    put_be32(ihdr, static_cast<std::uint32_t>(height));
    ihdr.push_back(static_cast<std::uint8_t>(bit_depth));
    ihdr.push_back(static_cast<std::uint8_t>(colour_type));
    ihdr.push_back(0); // compression: deflate
    ihdr.push_back(0); // filter method 0
    ihdr.push_back(0); // no interlace
    push_chunk(out, "IHDR", ihdr);
    push_chunk(out, "IDAT", zlib_stored(raw));
    push_chunk(out, "IEND", Bytes{});
    return out;
}

// ============================================================================
// Shorthand
// ============================================================================

/// Placement that makes no geographic claim: origin (0,0), one metre cells.
HeightmapPlacement unit_local() {
    return HeightmapPlacement::local(glm::vec2{0.0f, 0.0f}, 1.0f, 1.0f);
}

HeightmapImportResult import_bytes(const Bytes& bytes, const ElevationRange& vertical,
                                   const HeightmapPlacement& placement,
                                   const HeightmapImportOptions& options = {}) {
    return import_heightmap_from_memory(std::span<const std::uint8_t>{bytes}, vertical, placement,
                                        options);
}

/// Status as a readable string, so a failure says "Truncated", not "<unprintable>".
std::string status_of(const HeightmapImportResult& result) {
    return std::string{stratum::procgen::to_string(result.status)};
}

std::string ok_status() { return std::string{"Ok"}; }

/**
 * @brief Whether any warning contains @p fragment
 *
 * Warnings are asserted by their TEXT here, not merely counted. A warning whose
 * wording nothing checks can be replaced with the literal "ignore me" and every
 * `CHECK_FALSE(warnings.empty())` in the file still passes -- so "a warning was
 * produced" is not the thing worth asserting. What the user is told is.
 */
bool warning_says(const HeightmapImportResult& result, const std::string& fragment) {
    for (const std::string& warning : result.warnings) {
        if (warning.find(fragment) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// Whether @p result's message contains @p fragment. Several distinct guards
/// return the same status, so the message is the only thing that says WHICH one
/// fired, and a test that checks only the status cannot tell them apart.
bool message_says(const HeightmapImportResult& result, const std::string& fragment) {
    return result.message.find(fragment) != std::string::npos;
}

/// Bounds over Dublin. Latitude and longitude are far apart in value and in
/// sign, so every ordering mistake in this file produces an obviously wrong
/// number rather than a subtly wrong one.
stratum::osm::BoundingBox dublin_bounds() {
    stratum::osm::BoundingBox bounds;
    bounds.min_lat = 53.30;
    bounds.max_lat = 53.40;
    bounds.min_lon = -6.30;
    bounds.max_lon = -6.20;
    return bounds;
}

} // namespace

// ============================================================================
// PGM: sample values, endianness, orientation
// ============================================================================

TEST(HeightmapIO, pgm_binary_8bit_samples_become_metres) {
    // 4 wide, 3 tall, every value distinct: a transposed, mirrored or shifted
    // read lands on a different number everywhere.
    //
    // File row order, row 0 first (north):
    //   north  0  51 102 153
    //        204 255  10  20
    //   south 30  40  50  60
    const Bytes pgm = make_pgm_binary(4, 3, 255,
                                      {0, 51, 102, 153, 204, 255, 10, 20, 30, 40, 50, 60});

    // 0 m at sample 0 and 255 m at full scale makes metres numerically equal to
    // the raw sample, so every expectation below is the byte that was written.
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 255.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(std::string{stratum::procgen::to_string(result.source.format)},
             std::string{"PGM (P5, binary)"});
    CHECK_EQ(result.source.width, 4);
    CHECK_EQ(result.source.height, 3);
    CHECK_EQ(result.source.bits_per_sample, 8);
    CHECK_EQ(result.source.max_sample_value, std::uint32_t{255});
    CHECK_EQ(result.heightmap.width, 4);
    CHECK_EQ(result.heightmap.height, 3);
    CHECK_EQ(result.heightmap.data.size(), std::size_t{12});
    if (result.heightmap.data.size() != 12) {
        return; // at() returns 0.0f out of bounds; do not assert against that
    }

    // Heightmap row 0 is SOUTH, so it holds the LAST file row.
    CHECK_NEAR(result.heightmap.at(0, 0), 30.0, 1e-4);
    CHECK_NEAR(result.heightmap.at(1, 0), 40.0, 1e-4);
    CHECK_NEAR(result.heightmap.at(2, 0), 50.0, 1e-4);
    CHECK_NEAR(result.heightmap.at(3, 0), 60.0, 1e-4);

    CHECK_NEAR(result.heightmap.at(0, 1), 204.0, 1e-4);
    CHECK_NEAR(result.heightmap.at(1, 1), 255.0, 1e-4);
    CHECK_NEAR(result.heightmap.at(2, 1), 10.0, 1e-4);
    CHECK_NEAR(result.heightmap.at(3, 1), 20.0, 1e-4);

    CHECK_NEAR(result.heightmap.at(0, 2), 0.0, 1e-4);
    CHECK_NEAR(result.heightmap.at(1, 2), 51.0, 1e-4);
    CHECK_NEAR(result.heightmap.at(2, 2), 102.0, 1e-4);
    CHECK_NEAR(result.heightmap.at(3, 2), 153.0, 1e-4);
}

TEST(HeightmapIO, pgm_binary_16bit_samples_are_big_endian) {
    // Each value has a different high and low byte, so a little-endian read
    // produces a completely different number. 0x1234 read backwards is 0x3412.
    const Bytes pgm = make_pgm_binary(2, 2, 65535, {0x1234, 0xFF00, 0x00FF, 0x8001});

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 65535.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(result.source.bits_per_sample, 16);
    CHECK_EQ(result.source.max_sample_value, std::uint32_t{65535});
    if (result.heightmap.data.size() != 4) {
        return;
    }
    // South row is the second file row: {0x00FF, 0x8001}.
    CHECK_NEAR(result.heightmap.at(0, 0), 255.0, 1e-2);
    CHECK_NEAR(result.heightmap.at(1, 0), 32769.0, 1e-2);
    // North row is the first file row: {0x1234, 0xFF00}.
    CHECK_NEAR(result.heightmap.at(0, 1), 4660.0, 1e-2);
    CHECK_NEAR(result.heightmap.at(1, 1), 65280.0, 1e-2);
}

TEST(HeightmapIO, image_row_zero_lands_on_the_last_heightmap_row) {
    // Dark at the top of the image, bright at the bottom. Local y increases
    // northward and image row 0 is north, so the LAST heightmap row must be the
    // dark one. Skip the flip and this test reads 1000 where it wants 0.
    const Bytes pgm = make_pgm_binary(3, 2, 255, {0, 0, 0, 255, 255, 255});

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 1000.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    if (result.heightmap.data.size() != 6) {
        return;
    }
    for (int x = 0; x < 3; ++x) {
        CHECK_NEAR(result.heightmap.at(x, 0), 1000.0, 1e-3); // south, from image row 1
        CHECK_NEAR(result.heightmap.at(x, 1), 0.0, 1e-3);    // north, from image row 0
    }
}

TEST(HeightmapIO, source_row_zero_is_north_option_inverts_the_flip) {
    const Bytes pgm = make_pgm_binary(3, 2, 255, {0, 0, 0, 255, 255, 255});

    HeightmapImportOptions options;
    options.source_row_zero_is_north = false;
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 1000.0), unit_local(), options);

    CHECK_EQ(status_of(result), ok_status());
    if (result.heightmap.data.size() != 6) {
        return;
    }
    // Exactly the opposite of the previous test. If the option were ignored,
    // one of these two tests would fail; if the flip were unconditional, the
    // other would.
    for (int x = 0; x < 3; ++x) {
        CHECK_NEAR(result.heightmap.at(x, 0), 0.0, 1e-3);
        CHECK_NEAR(result.heightmap.at(x, 1), 1000.0, 1e-3);
    }
}

// ============================================================================
// Vertical scale
// ============================================================================

TEST(HeightmapIO, elevation_range_sets_the_vertical_scale) {
    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 85, 170, 255});

    const HeightmapImportResult small =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local());
    const HeightmapImportResult large =
        import_bytes(pgm, ElevationRange::metres(0.0, 1000.0), unit_local());

    CHECK_EQ(status_of(small), ok_status());
    CHECK_EQ(status_of(large), ok_status());
    if (small.heightmap.data.size() != 4 || large.heightmap.data.size() != 4) {
        return;
    }
    // Raw 85 of 255 is one third of full scale.
    CHECK_NEAR(small.heightmap.at(1, 1), 100.0 / 3.0, 1e-3);
    CHECK_NEAR(large.heightmap.at(1, 1), 1000.0 / 3.0, 1e-2);
    // The factor-of-ten mistake this API exists to prevent, asserted directly.
    CHECK_NEAR(large.heightmap.at(1, 1), 10.0 * small.heightmap.at(1, 1), 1e-2);
}

TEST(HeightmapIO, a_negative_minimum_offsets_the_whole_field) {
    // Sea floor to hilltop: sample 0 is -50 m, full scale is +50 m. An
    // implementation that only scales, and forgets the offset, returns 0 for
    // sample 0 and passes nothing here.
    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 255, 170, 170});

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(-50.0, 50.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    if (result.heightmap.data.size() != 4) {
        return;
    }
    CHECK_NEAR(result.heightmap.at(0, 1), -50.0, 1e-3); // north-west, raw 0
    CHECK_NEAR(result.heightmap.at(1, 1), 50.0, 1e-3);  // north-east, raw 255
    CHECK_NEAR(result.heightmap.at(0, 0), -50.0 + (170.0 / 255.0) * 100.0, 1e-3);
}

TEST(HeightmapIO, normalisation_uses_the_declared_maxval_not_the_observed_one) {
    // Both files declare maxval 1000. The first happens to contain a sample at
    // full scale; the second's largest sample is only half that -- as if the
    // highest peak had been cropped out of the tile.
    //
    // A sample of 250 must mean the same elevation in both. Normalising by the
    // OBSERVED maximum would make it 500 m in the first file and 1000 m in the
    // second, so two crops of one mountain would not meet at their shared edge.
    const Bytes full_range = make_pgm_binary(2, 2, 1000, {0, 250, 500, 1000});
    const Bytes cropped = make_pgm_binary(2, 2, 1000, {0, 250, 400, 500});

    const ElevationRange vertical = ElevationRange::metres(0.0, 2000.0);
    const HeightmapImportResult a = import_bytes(full_range, vertical, unit_local());
    const HeightmapImportResult b = import_bytes(cropped, vertical, unit_local());

    CHECK_EQ(status_of(a), ok_status());
    CHECK_EQ(status_of(b), ok_status());
    CHECK_EQ(a.source.max_sample_value, std::uint32_t{1000});
    CHECK_EQ(b.source.max_sample_value, std::uint32_t{1000});
    if (a.heightmap.data.size() != 4 || b.heightmap.data.size() != 4) {
        return;
    }
    // Raw 250 of a declared 1000 is a quarter of a 2000 m range.
    CHECK_NEAR(a.heightmap.at(1, 1), 500.0, 1e-2);
    CHECK_NEAR(b.heightmap.at(1, 1), 500.0, 1e-2);
    // And full scale really is full scale in the first file.
    CHECK_NEAR(a.heightmap.at(1, 0), 2000.0, 1e-2);
}

TEST(HeightmapIO, vertical_quantum_reports_the_cost_of_eight_bits) {
    const Bytes eight_bit = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});
    const Bytes sixteen_bit = make_pgm_binary(2, 2, 65535, {0, 100, 200, 255});
    const ElevationRange vertical = ElevationRange::metres(0.0, 1200.0);

    const HeightmapImportResult coarse = import_bytes(eight_bit, vertical, unit_local());
    const HeightmapImportResult fine = import_bytes(sixteen_bit, vertical, unit_local());

    CHECK_EQ(status_of(coarse), ok_status());
    CHECK_EQ(status_of(fine), ok_status());
    CHECK_NEAR(coarse.vertical_quantum_metres, 1200.0 / 255.0, 1e-9);
    CHECK_NEAR(fine.vertical_quantum_metres, 1200.0 / 65535.0, 1e-9);
    // 4.7 m per step is coarse enough to terrace and must be said out loud;
    // 1.8 cm is not worth a word.
    //
    // Asserted by its TEXT and by its count, not by "there is at least one
    // warning". The whole body of this message can be replaced with the word
    // "nothing" and a `warnings.empty()` check still passes, so the check that
    // only counts warnings tests nothing about what the user is told.
    CHECK_EQ(coarse.warnings.size(), std::size_t{1});
    CHECK_TRUE(warning_says(coarse, "8-bit source over a 1200.00 m range"));
    CHECK_TRUE(warning_says(coarse, "resolves only 4.706 m per sample step"));
    CHECK_TRUE(warning_says(coarse, "16-bit source over the same range resolves 0.0183 m"));
    CHECK_EQ(fine.warnings.size(), std::size_t{0});
}

TEST(HeightmapIO, an_unusable_elevation_range_is_refused) {
    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});

    // Arguments the wrong way round: the terrain would come out upside down and
    // still look like terrain.
    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(500.0, 100.0), unit_local())),
             std::string{"InvalidElevationRange"});
    // Zero span: almost always a typo for a real range.
    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(100.0, 100.0), unit_local())),
             std::string{"InvalidElevationRange"});
    // Absurd magnitude: the caller passed something that is not metres.
    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(0.0, 1.0e9), unit_local())),
             std::string{"InvalidElevationRange"});
    // Control: the same bytes with a sane range still load, so the three
    // refusals above are about the range and not about the fixture.
    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local())),
             ok_status());
}

// ============================================================================
// Resampling
// ============================================================================

TEST(HeightmapIO, a_same_size_target_reproduces_the_source_exactly) {
    // A checkerboard is the worst case for an accidental smear: any filtering
    // that touches a neighbour pulls every value towards the mean.
    const Bytes pgm = make_pgm_binary(4, 4, 255,
                                      {0, 255, 0, 255,
                                       255, 0, 255, 0,
                                       0, 255, 0, 255,
                                       255, 0, 255, 0});

    HeightmapImportOptions options;
    options.target_width = 4;
    options.target_height = 4;
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 1000.0), unit_local(), options);

    CHECK_EQ(status_of(result), ok_status());
    if (result.heightmap.data.size() != 16) {
        return;
    }
    for (float value : result.heightmap.data) {
        // Exactly the endpoints, never anything between them.
        CHECK_TRUE(value == 0.0f || value == 1000.0f);
    }
    CHECK_NEAR(result.heightmap.at(0, 0), 1000.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(1, 0), 0.0, 1e-3);
}

TEST(HeightmapIO, upsampling_interpolates_between_the_four_neighbours) {
    // 2x2 source, every corner different:
    //   north   0 100
    //   south 200 255
    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});

    HeightmapImportOptions options;
    options.target_width = 3;
    options.target_height = 3;
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 255.0), unit_local(), options);

    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(result.heightmap.width, 3);
    CHECK_EQ(result.heightmap.height, 3);
    if (result.heightmap.data.size() != 9) {
        return;
    }

    // Corners are the source corners exactly -- pixel-is-point, so the grid
    // spans the extent edge to edge and nothing is extrapolated.
    CHECK_NEAR(result.heightmap.at(0, 0), 200.0, 1e-3); // south-west
    CHECK_NEAR(result.heightmap.at(2, 0), 255.0, 1e-3); // south-east
    CHECK_NEAR(result.heightmap.at(0, 2), 0.0, 1e-3);   // north-west
    CHECK_NEAR(result.heightmap.at(2, 2), 100.0, 1e-3); // north-east

    // Edge midpoints are two-neighbour means, and the centre is the mean of all
    // four. Nearest-neighbour resampling reproduces the four corners above but
    // gets every one of these three wrong.
    CHECK_NEAR(result.heightmap.at(1, 0), (200.0 + 255.0) / 2.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(0, 1), (0.0 + 200.0) / 2.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(1, 1), (0.0 + 100.0 + 200.0 + 255.0) / 4.0, 1e-3);
}

TEST(HeightmapIO, upsampling_an_asymmetric_source_pins_every_edge_sample) {
    // The other resample fixtures are square (2x2, 4x4, 8x8) and symmetric, so a
    // clamp that bounds x against the HEIGHT, or an interpolation that swaps the
    // two axes, reproduces them exactly. This one is 5 wide and 2 tall with no
    // symmetry in either axis, and every one of the 27 output samples is stated.
    //
    // It also lands destination samples exactly on the far right and top edges of
    // the source, which is where the second bilinear tap addresses one column and
    // one row past the end. That tap is weighted zero there, so its value never
    // reaches an assertion -- these checks pin what the edge samples ARE, and the
    // read itself is made harmless in resample() rather than merely unlikely.
    //
    //   file row 0 (north):   0  40  80 120 160
    //   file row 1 (south): 200 210 220 230 240
    const Bytes pgm = make_pgm_binary(5, 2, 240,
                                      {0, 40, 80, 120, 160,
                                       200, 210, 220, 230, 240});

    HeightmapImportOptions options;
    options.target_width = 9;
    options.target_height = 3;
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 240.0), unit_local(), options);

    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(result.heightmap.width, 9);
    CHECK_EQ(result.heightmap.height, 3);
    if (result.heightmap.data.size() != 27) {
        return;
    }

    // Heightmap row 0 is SOUTH: file row 1, upsampled 5 -> 9.
    const double south[9] = {200, 205, 210, 215, 220, 225, 230, 235, 240};
    // Heightmap row 2 is NORTH: file row 0.
    const double north[9] = {0, 20, 40, 60, 80, 100, 120, 140, 160};
    for (int x = 0; x < 9; ++x) {
        CHECK_NEAR(result.heightmap.at(x, 0), south[x], 1e-3);
        CHECK_NEAR(result.heightmap.at(x, 2), north[x], 1e-3);
        // The middle row is halfway between the two, which pins the vertical
        // weight: at fy 0 or 1 it would equal one of the rows above.
        CHECK_NEAR(result.heightmap.at(x, 1), (south[x] + north[x]) / 2.0, 1e-3);
    }
}

TEST(HeightmapIO, heavy_downsampling_warns_that_bilinear_does_not_area_average) {
    std::vector<std::uint16_t> samples(64, 0);
    samples[27] = 255; // a one-cell spike that a 2x2 output cannot represent
    const Bytes pgm = make_pgm_binary(8, 8, 255, samples);

    HeightmapImportOptions options;
    options.target_width = 2;
    options.target_height = 2;
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local(), options);

    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(result.heightmap.width, 2);
    CHECK_EQ(result.heightmap.height, 2);
    CHECK_EQ(result.warnings.size(), std::size_t{1});
    CHECK_TRUE(warning_says(result, "downsampling 8x8 to 2x2 by more than 2x per axis"));
    // And the warning is not idle: the spike really is gone, because a 2x2
    // output samples only the four source corners.
    CHECK_NEAR(result.heightmap.at(0, 0), 0.0, 1e-6);
    CHECK_NEAR(result.heightmap.at(1, 1), 0.0, 1e-6);
}

TEST(HeightmapIO, the_downsampling_warning_starts_past_2x_and_not_at_1x) {
    // The 8x8 -> 2x2 fixture above is a 4x reduction, which is true of
    // "more than 2x" and equally true of "any downsample at all". The threshold
    // itself needs a case on each side of it, or the rule the comment states is
    // not the rule the code implements and nothing notices.
    //
    // A 0-100 m range at 8 bits quantises to 0.39 m, below
    // kCoarseVerticalQuantumMetres, so the coarse-quantum warning stays out of
    // the way and `warnings` here is only ever about downsampling.
    const ElevationRange vertical = ElevationRange::metres(0.0, 100.0);

    const Bytes eight = make_pgm_binary(8, 8, 255, std::vector<std::uint16_t>(64, 40));
    HeightmapImportOptions exactly_two;
    exactly_two.target_width = 4;
    exactly_two.target_height = 4;
    const HeightmapImportResult at_2x = import_bytes(eight, vertical, unit_local(), exactly_two);
    CHECK_EQ(status_of(at_2x), ok_status());
    // Exactly 2x is what bilinear handles honestly: each destination sample has
    // a source sample under it. No warning.
    CHECK_EQ(at_2x.warnings.size(), std::size_t{0});

    const Bytes nine = make_pgm_binary(9, 9, 255, std::vector<std::uint16_t>(81, 40));
    HeightmapImportOptions just_past_two;
    just_past_two.target_width = 4;
    just_past_two.target_height = 4;
    const HeightmapImportResult past_2x = import_bytes(nine, vertical, unit_local(), just_past_two);
    CHECK_EQ(status_of(past_2x), ok_status());
    CHECK_EQ(past_2x.warnings.size(), std::size_t{1});
    CHECK_TRUE(warning_says(past_2x, "downsampling 9x9 to 4x4 by more than 2x per axis"));

    // Upsampling is never worth a word.
    HeightmapImportOptions upsample;
    upsample.target_width = 16;
    upsample.target_height = 16;
    const HeightmapImportResult up = import_bytes(eight, vertical, unit_local(), upsample);
    CHECK_EQ(status_of(up), ok_status());
    CHECK_EQ(up.warnings.size(), std::size_t{0});
}

TEST(HeightmapIO, a_degenerate_target_grid_is_refused) {
    const Bytes pgm = make_pgm_binary(4, 4, 255, std::vector<std::uint16_t>(16, 128));

    HeightmapImportOptions single_column;
    single_column.target_width = 1;
    single_column.target_height = 4;
    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local(),
                                    single_column)),
             std::string{"InvalidOptions"});

    HeightmapImportOptions negative;
    negative.target_height = -8;
    CHECK_EQ(
        status_of(import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local(), negative)),
        std::string{"InvalidOptions"});
}

TEST(HeightmapIO, a_target_grid_is_capped_by_the_same_guards_as_the_source) {
    // target_width and target_height size the OUTPUT allocation directly, so
    // they are an allocation guard in exactly the way the source dimensions are
    // -- and they come from the caller, which in the editor means a spin box.
    // Only the "at least two samples" end of that is covered anywhere else, so a
    // check narrowed to `dst_w < 2 || dst_h < 2` would pass the rest of the file.
    const Bytes pgm = make_pgm_binary(4, 4, 255, std::vector<std::uint16_t>(16, 128));
    const ElevationRange vertical = ElevationRange::metres(0.0, 100.0);

    // Past the per-axis cap. 70000 x 4 is only 280,000 samples, well inside
    // max_samples, so this is the axis cap and nothing else.
    HeightmapImportOptions past_axis;
    past_axis.target_width = 70000;
    past_axis.target_height = 4;
    const HeightmapImportResult wide = import_bytes(pgm, vertical, unit_local(), past_axis);
    CHECK_EQ(status_of(wide), std::string{"InvalidOptions"});
    CHECK_TRUE(message_says(wide, "target grid rejected"));
    CHECK_TRUE(message_says(wide, "per-axis limit"));

    // Past max_samples, with a source that is comfortably inside it.
    HeightmapImportOptions past_total;
    past_total.target_width = 4096;
    past_total.target_height = 4096;
    past_total.max_samples = 1000;
    const HeightmapImportResult huge = import_bytes(pgm, vertical, unit_local(), past_total);
    CHECK_EQ(status_of(huge), std::string{"InvalidOptions"});
    CHECK_TRUE(message_says(huge, "16777216 samples"));
    CHECK_TRUE(message_says(huge, "max_samples is 1000"));
    CHECK_TRUE(huge.heightmap.data.empty());

    // Control: a target inside both caps resamples.
    HeightmapImportOptions fine;
    fine.target_width = 8;
    fine.target_height = 8;
    const HeightmapImportResult ok = import_bytes(pgm, vertical, unit_local(), fine);
    CHECK_EQ(status_of(ok), ok_status());
    CHECK_EQ(ok.heightmap.width, 8);
}

// ============================================================================
// Georeferencing
// ============================================================================

TEST(HeightmapIO, geographic_placement_derives_origin_and_cell_size_from_the_bounds) {
    const stratum::osm::BoundingBox bounds = dublin_bounds();
    stratum::osm::CoordinateConverter converter;
    converter.set_origin(bounds);

    // 3x3 means two intervals per axis, so a cell is half the extent.
    const Bytes pgm = make_pgm_binary(3, 3, 255, std::vector<std::uint16_t>(9, 128));
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0),
                     HeightmapPlacement::geographic(bounds, converter));

    CHECK_EQ(status_of(result), ok_status());

    // Expected values computed here from the converter directly, not from the
    // importer, so the two have to agree for this to pass.
    const glm::dvec2 south_west = converter.wgs84_to_local(bounds.min_lat, bounds.min_lon);
    const glm::dvec2 north_east = converter.wgs84_to_local(bounds.max_lat, bounds.max_lon);

    CHECK_NEAR(result.heightmap.origin.x, south_west.x, 1e-2);
    CHECK_NEAR(result.heightmap.origin.y, south_west.y, 1e-2);
    CHECK_NEAR(result.heightmap.cell_size_x, (north_east.x - south_west.x) / 2.0, 1e-2);
    CHECK_NEAR(result.heightmap.cell_size_z, (north_east.y - south_west.y) / 2.0, 1e-2);

    // Sample (0,0) is the south-west corner, so the origin is south and west of
    // the bounds centre -- which is where the converter was anchored.
    CHECK((result.heightmap.origin.x) < (0.0f));
    CHECK((result.heightmap.origin.y) < (0.0f));

    // The two cell sizes are NOT equal, even though the box is 0.1 degrees on
    // both axes. Web Mercator stretches northing by 1/cos(latitude), so at
    // Dublin a degree of latitude spans about 1.68 times what a degree of
    // longitude does. An equirectangular shortcut would make them equal.
    CHECK((result.heightmap.cell_size_x) < (result.heightmap.cell_size_z));
    const double stretch = static_cast<double>(result.heightmap.cell_size_z) /
                           static_cast<double>(result.heightmap.cell_size_x);
    CHECK_NEAR(stretch, 1.0 / std::cos(53.35 * stratum::osm::DEG_TO_RAD), 1e-3);
}

TEST(HeightmapIO, bounding_box_centre_is_latitude_then_longitude) {
    // BoundingBox::center() returns (lat, lon) in a dvec2, the reverse of every
    // other dvec2 in this codebase. Dublin sits at 53.35 N, 6.25 W, so reading
    // the two the wrong way round gives a latitude of -6.25 -- a legal latitude,
    // in the Atlantic off Ghana, which is why the mistake survives review.
    const stratum::osm::BoundingBox bounds = dublin_bounds();
    stratum::osm::CoordinateConverter converter;
    converter.set_origin(bounds);

    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0),
                     HeightmapPlacement::geographic(bounds, converter));

    CHECK_EQ(status_of(result), ok_status());
    CHECK_NEAR(result.centre_lat, 53.35, 1e-9);
    CHECK_NEAR(result.centre_lon, -6.25, 1e-9);

    // cos(53.35) is 0.5965; cos(-6.25) is 0.9941. Swapping the two components
    // fails this by 40%.
    CHECK_NEAR(result.ground_metres_per_unit, std::cos(53.35 * stratum::osm::DEG_TO_RAD), 1e-6);
    CHECK((result.ground_metres_per_unit) < (0.7));
}

TEST(HeightmapIO, local_placement_reports_no_projection_rather_than_a_guess) {
    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0),
                     HeightmapPlacement::local(glm::vec2{10.0f, -20.0f}, 2.5f, 4.0f));

    CHECK_EQ(status_of(result), ok_status());
    CHECK_NEAR(result.heightmap.origin.x, 10.0, 1e-6);
    CHECK_NEAR(result.heightmap.origin.y, -20.0, 1e-6);
    CHECK_NEAR(result.heightmap.cell_size_x, 2.5, 1e-6);
    CHECK_NEAR(result.heightmap.cell_size_z, 4.0, 1e-6);
    // No projection happened, so the Mercator factor is the identity and no
    // place is claimed.
    CHECK_NEAR(result.ground_metres_per_unit, 1.0, 1e-12);
    CHECK_NEAR(result.centre_lat, 0.0, 1e-12);
}

TEST(HeightmapIO, an_uninitialised_converter_is_refused) {
    // CoordinateConverter::wgs84_to_local() silently returns raw Web Mercator
    // when set_origin() was never called. Accepting it would put the terrain
    // roughly 5,900 km from the scene with no diagnostic anywhere.
    const stratum::osm::CoordinateConverter untouched;
    CHECK_FALSE(untouched.is_initialized());

    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0),
                     HeightmapPlacement::geographic(dublin_bounds(), untouched));

    CHECK_EQ(status_of(result), std::string{"InvalidPlacement"});
    CHECK_TRUE(result.heightmap.data.empty());

    // Control: the same bounds and the same bytes load once an origin exists,
    // so the refusal is about the converter and nothing else.
    stratum::osm::CoordinateConverter ready;
    ready.set_origin(dublin_bounds());
    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(0.0, 100.0),
                                    HeightmapPlacement::geographic(dublin_bounds(), ready))),
             ok_status());
}

TEST(HeightmapIO, degenerate_or_out_of_domain_bounds_are_refused) {
    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});
    const ElevationRange vertical = ElevationRange::metres(0.0, 100.0);
    stratum::osm::CoordinateConverter converter;
    converter.set_origin(53.35, -6.25);

    // Default-constructed: deliberately inverted, meaning "never set".
    const stratum::osm::BoundingBox never_set;
    CHECK_FALSE(never_set.is_valid());
    const HeightmapImportResult unset =
        import_bytes(pgm, vertical, HeightmapPlacement::geographic(never_set, converter));
    CHECK_EQ(status_of(unset), std::string{"InvalidPlacement"});
    // Three different guards return InvalidPlacement here, and an inverted box
    // falls through to the degenerate-span one and yields the same status from
    // the wrong check -- so the status alone cannot tell them apart and the
    // is_valid() guard could be deleted without any of these three noticing.
    // The message can tell them apart.
    CHECK_TRUE(message_says(unset, "reads as 'never set'"));

    // Valid but with no area: every cell size would be zero.
    stratum::osm::BoundingBox sliver;
    sliver.min_lat = 53.30;
    sliver.max_lat = 53.40;
    sliver.min_lon = -6.25;
    sliver.max_lon = -6.25;
    CHECK_TRUE(sliver.is_valid());
    const HeightmapImportResult flat =
        import_bytes(pgm, vertical, HeightmapPlacement::geographic(sliver, converter));
    CHECK_EQ(status_of(flat), std::string{"InvalidPlacement"});
    CHECK_TRUE(message_says(flat, "degenerate extent"));

    // Past the Mercator latitude limit, where wgs84_to_mercator() clamps in
    // silence and would hand back a squashed extent.
    stratum::osm::BoundingBox polar;
    polar.min_lat = 84.0;
    polar.max_lat = 89.0;
    polar.min_lon = -6.30;
    polar.max_lon = -6.20;
    const HeightmapImportResult arctic =
        import_bytes(pgm, vertical, HeightmapPlacement::geographic(polar, converter));
    CHECK_EQ(status_of(arctic), std::string{"InvalidPlacement"});
    CHECK_TRUE(message_says(arctic, "outside the Web Mercator domain"));
}

TEST(HeightmapIO, local_placement_refuses_a_cell_size_that_is_not_positive_and_finite) {
    // A cell size of zero is the one that looks harmless: it produces a grid
    // whose every sample sits at the same point, and every distance measured off
    // it afterwards is zero. Nothing else in the suite passes anything but
    // 1.0 x 1.0 or 2.5 x 4.0, so this guard could be `if (false)` and the rest of
    // the file would not care.
    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});
    const ElevationRange vertical = ElevationRange::metres(0.0, 100.0);
    const float not_a_number = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    const glm::vec2 somewhere{7.0f, 9.0f};

    const HeightmapImportResult zero =
        import_bytes(pgm, vertical, HeightmapPlacement::local(somewhere, 0.0f, 1.0f));
    CHECK_EQ(status_of(zero), std::string{"InvalidPlacement"});
    CHECK_TRUE(message_says(zero, "strictly positive, finite cell sizes"));

    // A failed placement must leave the grid untouched rather than half-written.
    // Nothing clears it afterwards, so this asserts that nothing wrote to it
    // before deciding to fail: empty data, the default one-metre cells, and an
    // origin at zero rather than the (7, 9) that was asked for.
    CHECK_TRUE(zero.heightmap.data.empty());
    CHECK_EQ(zero.heightmap.width, 0);
    CHECK_NEAR(zero.heightmap.origin.x, 0.0, 1e-12);
    CHECK_NEAR(zero.heightmap.origin.y, 0.0, 1e-12);
    CHECK_NEAR(zero.heightmap.cell_size_x, 1.0, 1e-12);

    CHECK_EQ(status_of(import_bytes(pgm, vertical,
                                    HeightmapPlacement::local(somewhere, -2.0f, 1.0f))),
             std::string{"InvalidPlacement"});
    CHECK_EQ(status_of(import_bytes(pgm, vertical,
                                    HeightmapPlacement::local(somewhere, 1.0f, 0.0f))),
             std::string{"InvalidPlacement"});
    CHECK_EQ(status_of(import_bytes(pgm, vertical,
                                    HeightmapPlacement::local(somewhere, not_a_number, 1.0f))),
             std::string{"InvalidPlacement"});
    CHECK_EQ(status_of(import_bytes(pgm, vertical,
                                    HeightmapPlacement::local(somewhere, infinity, 1.0f))),
             std::string{"InvalidPlacement"});

    const HeightmapImportResult bad_origin = import_bytes(
        pgm, vertical, HeightmapPlacement::local(glm::vec2{not_a_number, 0.0f}, 1.0f, 1.0f));
    CHECK_EQ(status_of(bad_origin), std::string{"InvalidPlacement"});
    CHECK_TRUE(message_says(bad_origin, "origin is not finite"));

    // Control: the same bytes with a usable placement load, so the refusals are
    // about the placement and not about the fixture.
    CHECK_EQ(status_of(import_bytes(pgm, vertical,
                                    HeightmapPlacement::local(somewhere, 2.0f, 3.0f))),
             ok_status());
}

// ============================================================================
// Malformed input
// ============================================================================

TEST(HeightmapIO, a_truncated_binary_pgm_is_refused) {
    // The header claims 100x100 -- ten thousand samples -- and ten bytes follow.
    const Bytes pgm = make_pgm_binary(100, 100, 255, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10});

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local());

    CHECK_EQ(status_of(result), std::string{"Truncated"});
    CHECK_TRUE(result.heightmap.data.empty());
    CHECK_EQ(result.heightmap.width, 0);
    // The message has to say what was claimed, or it cannot be acted on.
    CHECK((result.message.find("100x100")) != (std::string::npos));
    // And the header block has to survive the refusal for the same reason: a
    // batch log that says "truncated" without saying how big the tile claimed to
    // be cannot be acted on either.
    CHECK_EQ(result.source.width, 100);
    CHECK_EQ(result.source.height, 100);
    CHECK_EQ(result.source.bits_per_sample, 8);
    CHECK_EQ(result.source.max_sample_value, std::uint32_t{255});
}

TEST(HeightmapIO, a_binary_pgm_short_by_exactly_one_byte_is_refused) {
    // Every other truncation fixture in this file is GROSSLY short -- 100x100
    // with ten bytes, 64x64 with three. A bound that is wrong by one or two
    // bytes refuses all of those just as happily as the correct bound does, so
    // none of them can tell a correct bound from a loose one. These can: a loose
    // bound accepts them, and then reads past the end of the buffer.
    const ElevationRange vertical = ElevationRange::metres(0.0, 100.0);

    // 2x2 at 8 bits needs 4 raster bytes. Three is short by exactly one.
    Bytes one_byte_short = make_pgm_binary(2, 2, 255, {1, 2, 3, 4});
    one_byte_short.pop_back();
    const HeightmapImportResult short_8 = import_bytes(one_byte_short, vertical, unit_local());
    CHECK_EQ(status_of(short_8), std::string{"Truncated"});
    CHECK_TRUE(short_8.heightmap.data.empty());
    CHECK_TRUE(message_says(short_8, "needing 4 raster bytes, but only 3 remain"));

    // 2x2 at 16 bits needs 8. Seven is short by one BYTE -- half a sample, which
    // is the case a bound counting samples rather than bytes lets through.
    Bytes half_a_sample_short = make_pgm_binary(2, 2, 65535, {1, 2, 3, 4});
    half_a_sample_short.pop_back();
    CHECK_EQ(status_of(import_bytes(half_a_sample_short, vertical, unit_local())),
             std::string{"Truncated"});

    // Six is short by exactly one whole sample.
    const Bytes one_sample_short = make_pgm_binary(2, 2, 65535, {1, 2, 3});
    const HeightmapImportResult short_16 = import_bytes(one_sample_short, vertical, unit_local());
    CHECK_EQ(status_of(short_16), std::string{"Truncated"});
    CHECK_TRUE(message_says(short_16, "needing 8 raster bytes, but only 6 remain"));

    // Controls: exactly enough raster loads, at both depths. Without these the
    // three refusals above are equally consistent with a bound that refuses
    // everything.
    CHECK_EQ(status_of(import_bytes(make_pgm_binary(2, 2, 255, {1, 2, 3, 4}), vertical,
                                    unit_local())),
             ok_status());
    CHECK_EQ(status_of(import_bytes(make_pgm_binary(2, 2, 65535, {1, 2, 3, 4}), vertical,
                                    unit_local())),
             ok_status());
}

TEST(HeightmapIO, a_truncated_ascii_pgm_is_refused) {
    const Bytes pgm = make_pgm_ascii(4, 4, 255, {1, 2, 3, 4, 5});

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local());

    CHECK_EQ(status_of(result), std::string{"Truncated"});
    CHECK_TRUE(result.heightmap.data.empty());
}

TEST(HeightmapIO, an_ascii_pgm_missing_only_its_last_sample_is_refused) {
    // Three samples where four were declared. The file is long enough to pass
    // the up-front byte bound, so this is the per-sample guard at the bottom of
    // the parse loop, and it has to count to four rather than "enough".
    const ElevationRange vertical = ElevationRange::metres(0.0, 100.0);
    const HeightmapImportResult result =
        import_bytes(make_pgm_ascii(2, 2, 255, {1, 2, 3}), vertical, unit_local());

    CHECK_EQ(status_of(result), std::string{"Truncated"});
    CHECK_TRUE(result.heightmap.data.empty());
    // Named to the sample, so it says WHERE the file stopped.
    CHECK_TRUE(message_says(result, "ran out after 3"));

    // Control: the fourth sample makes the identical file load.
    CHECK_EQ(status_of(import_bytes(make_pgm_ascii(2, 2, 255, {1, 2, 3, 4}), vertical,
                                    unit_local())),
             ok_status());
}

TEST(HeightmapIO, an_ascii_pgm_header_cannot_size_a_buffer_the_file_could_never_fill) {
    // Seventeen bytes of header and no raster at all. Sizing the sample vector
    // from 8192x8192 before parsing anything costs 128 MB of resident memory --
    // about eight million times the input -- on the thread drawing the editor,
    // and it is reached from a file a stranger sent. An ASCII sample is at least
    // one digit and needs a separator between it and the next, so 67 million
    // samples cannot be encoded in one byte, and that is knowable before the
    // allocation rather than after it.
    //
    // This asserts the MESSAGE, not just the status: the late per-sample guard
    // also returns Truncated for this file, after the allocation, so the status
    // alone cannot tell which guard fired.
    Bytes header_only;
    append_ascii(header_only, "P2\n8192 8192\n255\n");

    const HeightmapImportResult result =
        import_bytes(header_only, ElevationRange::metres(0.0, 100.0), unit_local());

    CHECK_EQ(status_of(result), std::string{"Truncated"});
    CHECK_TRUE(result.heightmap.data.empty());
    CHECK_TRUE(message_says(result, "67108864 ASCII samples, which need at least"));
    CHECK_TRUE(message_says(result, "134217727 bytes, but only 1 remain"));

    // The bound must not refuse anything legal. This is the DENSEST a P2 raster
    // can legally be -- one digit per sample, one separator between them -- and
    // it has to load.
    Bytes dense;
    append_ascii(dense, "P2\n2 2\n9\n1 2 3 4");
    const HeightmapImportResult tight = import_bytes(dense, ElevationRange::metres(0.0, 9.0),
                                                     unit_local());
    CHECK_EQ(status_of(tight), ok_status());
    if (tight.heightmap.data.size() == 4) {
        CHECK_NEAR(tight.heightmap.at(0, 1), 1.0, 1e-4); // north-west, the first sample
        CHECK_NEAR(tight.heightmap.at(1, 0), 4.0, 1e-4); // south-east, the last
    }
}

TEST(HeightmapIO, absurd_dimensions_are_refused_by_the_caps_that_run_first) {
    // Four billion by four billion. Nothing may be sized from these numbers, and
    // nothing may overflow while checking them.
    Bytes pgm;
    append_ascii(pgm, "P5\n4000000000 4000000000\n255\n");
    pgm.push_back(0);

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local());

    CHECK_EQ(status_of(result), std::string{"DimensionsOutOfRange"});
    CHECK_TRUE(result.heightmap.data.empty());

    // A header of a thousand nines must saturate, not wrap into something small
    // and plausible that then passes the size check.
    Bytes enormous;
    append_ascii(enormous, "P5\n" + std::string(1000, '9') + " 4\n255\n");
    enormous.push_back(0);
    CHECK_EQ(status_of(import_bytes(enormous, ElevationRange::metres(0.0, 100.0), unit_local())),
             std::string{"DimensionsOutOfRange"});

    // 2^32 by 2^32. The product is exactly 2^64, so a sample-count check done on
    // the two numbers as they came out of the file WRAPS TO ZERO and waves the
    // image through -- and the width then truncates to 0 on the way into an int.
    // This is why the per-axis cap runs first and is not redundant with
    // max_samples: it is what makes the multiplication below it safe to perform
    // at all. Asserting the specific status, not merely "not Ok", is the point:
    // dropping the per-axis cap still fails this image later, for the wrong
    // reason, and a test that only checked ok() would not notice.
    Bytes wrapping;
    append_ascii(wrapping, "P5\n4294967296 4294967296\n255\n");
    wrapping.push_back(0);
    CHECK_EQ(status_of(import_bytes(wrapping, ElevationRange::metres(0.0, 100.0), unit_local())),
             std::string{"DimensionsOutOfRange"});
}

TEST(HeightmapIO, a_single_sample_axis_is_refused) {
    // One column has no spacing, so it has no cell size and nothing to
    // interpolate along.
    const Bytes pgm = make_pgm_binary(1, 4, 255, {1, 2, 3, 4});

    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local())),
             std::string{"DimensionsOutOfRange"});
}

TEST(HeightmapIO, max_samples_caps_the_image_before_it_is_decoded) {
    const Bytes pgm = make_pgm_binary(16, 16, 255, std::vector<std::uint16_t>(256, 7));

    HeightmapImportOptions tight;
    tight.max_samples = 100;
    const HeightmapImportResult refused =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local(), tight);
    CHECK_EQ(status_of(refused), std::string{"DimensionsOutOfRange"});
    // The refusal names the claim and the limit, and the header block survives
    // it. Both are what makes a failed batch import diagnosable.
    CHECK_TRUE(message_says(refused, "claims 256 samples"));
    CHECK_TRUE(message_says(refused, "max_samples is 100"));
    CHECK_EQ(refused.source.width, 16);
    CHECK_EQ(refused.source.height, 16);

    // Control: the identical bytes load under the default cap.
    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local())),
             ok_status());
}

TEST(HeightmapIO, a_max_samples_of_zero_is_refused_as_an_option_not_as_a_dimension) {
    // A cap of zero refuses every image that could ever exist, so it is a
    // mistake in the CALL and not a property of the file. Reporting it as
    // DimensionsOutOfRange would send the user looking at their heightmap.
    const Bytes pgm = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});

    HeightmapImportOptions none;
    none.max_samples = 0;
    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local(), none);

    CHECK_EQ(status_of(result), std::string{"InvalidOptions"});
    CHECK_TRUE(message_says(result, "max_samples is 0"));
    CHECK_TRUE(result.heightmap.data.empty());
}

TEST(HeightmapIO, a_sample_above_the_declared_maxval_is_refused) {
    // Declared maxval 100, but one byte is 200. Left alone it would normalise to
    // 2.0 and place that cell at twice the elevation the caller allowed for.
    const Bytes binary = make_pgm_binary(2, 2, 100, {10, 20, 200, 40});
    CHECK_EQ(status_of(import_bytes(binary, ElevationRange::metres(0.0, 100.0), unit_local())),
             std::string{"SampleAboveMaxval"});

    const Bytes ascii = make_pgm_ascii(2, 2, 100, {10, 20, 200, 40});
    CHECK_EQ(status_of(import_bytes(ascii, ElevationRange::metres(0.0, 100.0), unit_local())),
             std::string{"SampleAboveMaxval"});

    // Control: the same shape with every sample inside the declared maxval, and
    // the top of the range really does reach max_metres.
    const Bytes clean = make_pgm_binary(2, 2, 100, {10, 20, 100, 40});
    const HeightmapImportResult result =
        import_bytes(clean, ElevationRange::metres(0.0, 100.0), unit_local());
    CHECK_EQ(status_of(result), ok_status());
    if (result.heightmap.data.size() == 4) {
        CHECK_NEAR(result.heightmap.at(0, 0), 100.0, 1e-3);
    }
}

TEST(HeightmapIO, a_file_that_is_not_a_heightmap_is_refused_by_name) {
    const ElevationRange vertical = ElevationRange::metres(0.0, 100.0);

    Bytes pbm;
    append_ascii(pbm, "P4\n8 8\n");
    pbm.resize(pbm.size() + 8, 0);
    CHECK_EQ(status_of(import_bytes(pbm, vertical, unit_local())), std::string{"UnknownFormat"});

    Bytes prose;
    append_ascii(prose, "this is not an image at all");
    CHECK_EQ(status_of(import_bytes(prose, vertical, unit_local())), std::string{"UnknownFormat"});

    // An empty buffer is refused by its OWN guard, before any decoder runs.
    // decode_pgm() would also return UnknownFormat for zero bytes, so the status
    // alone cannot distinguish the two and the early return could be deleted
    // without any test in this file noticing. The wording distinguishes them.
    const HeightmapImportResult empty = import_bytes(Bytes{}, vertical, unit_local());
    CHECK_EQ(status_of(empty), std::string{"UnknownFormat"});
    CHECK_TRUE(message_says(empty, "the file is empty"));
}

TEST(HeightmapIO, a_pgm_header_that_stops_early_is_refused) {
    Bytes no_maxval;
    append_ascii(no_maxval, "P5\n4 4\n");
    CHECK_EQ(status_of(import_bytes(no_maxval, ElevationRange::metres(0.0, 100.0), unit_local())),
             std::string{"MalformedHeader"});

    Bytes no_height;
    append_ascii(no_height, "P5\n4\n");
    CHECK_EQ(status_of(import_bytes(no_height, ElevationRange::metres(0.0, 100.0), unit_local())),
             std::string{"MalformedHeader"});

    Bytes zero_maxval;
    append_ascii(zero_maxval, "P5\n4 4\n0\n");
    zero_maxval.resize(zero_maxval.size() + 16, 1);
    CHECK_EQ(
        status_of(import_bytes(zero_maxval, ElevationRange::metres(0.0, 100.0), unit_local())),
        std::string{"MalformedHeader"});
}

TEST(HeightmapIO, header_comments_are_skipped) {
    Bytes pgm;
    append_ascii(pgm, "P5\n# made by a tool that signs its work\n2 2\n# and comments twice\n255\n");
    pgm.push_back(0);
    pgm.push_back(100);
    pgm.push_back(200);
    pgm.push_back(255);

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 255.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    if (result.heightmap.data.size() == 4) {
        CHECK_NEAR(result.heightmap.at(0, 0), 200.0, 1e-3);
        CHECK_NEAR(result.heightmap.at(1, 1), 100.0, 1e-3);
    }
}

TEST(HeightmapIO, exactly_one_whitespace_separates_the_maxval_from_the_raster) {
    // The first raster byte is 10, which is also '\n'. A parser that skips
    // whitespace after the maxval instead of consuming exactly one character
    // eats this sample, leaving three bytes for four samples, and reports the
    // file as truncated -- or worse, shifts every value by one.
    Bytes pgm;
    append_ascii(pgm, "P5\n2 2\n255\n");
    pgm.push_back(10);
    pgm.push_back(20);
    pgm.push_back(30);
    pgm.push_back(40);

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 255.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    if (result.heightmap.data.size() == 4) {
        CHECK_NEAR(result.heightmap.at(0, 1), 10.0, 1e-3); // north-west, the '\n'-valued sample
        CHECK_NEAR(result.heightmap.at(1, 1), 20.0, 1e-3);
        CHECK_NEAR(result.heightmap.at(0, 0), 30.0, 1e-3);
        CHECK_NEAR(result.heightmap.at(1, 0), 40.0, 1e-3);
    }

    // A missing separator entirely is a malformed header, not a shifted read.
    Bytes glued;
    append_ascii(glued, "P5\n2 2\n255");
    glued.push_back(1);
    glued.push_back(2);
    glued.push_back(3);
    glued.push_back(4);
    CHECK_EQ(status_of(import_bytes(glued, ElevationRange::metres(0.0, 255.0), unit_local())),
             std::string{"MalformedHeader"});
}

TEST(HeightmapIO, bytes_beyond_the_declared_raster_warn_but_still_load) {
    Bytes pgm = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});
    for (int i = 0; i < 32; ++i) {
        pgm.push_back(0xAB);
    }

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 255.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(result.warnings.size(), std::size_t{1});
    CHECK_TRUE(warning_says(result, "PGM has 32 bytes after the raster"));
    // The extra bytes must not have been read as data.
    if (result.heightmap.data.size() == 4) {
        CHECK_NEAR(result.heightmap.at(1, 0), 255.0, 1e-3);
    }
}

TEST(HeightmapIO, one_trailing_byte_is_a_final_newline_and_is_not_worth_a_warning) {
    // Almost every PGM a real tool writes ends with a newline. Warning about it
    // would put a spurious line in the import dialog for ordinary files, so the
    // rule is "more than one trailing byte", and that boundary needs a case on
    // each side of it or the rule is untested: widening it to "any trailing
    // byte" changes nothing any other test in this file can see.
    //
    // 0-255 m at 8 bits is exactly 1.00 m per step, which is not ABOVE
    // kCoarseVerticalQuantumMetres, so nothing else contributes a warning here.
    const ElevationRange vertical = ElevationRange::metres(0.0, 255.0);

    Bytes one_newline = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});
    one_newline.push_back('\n');
    const HeightmapImportResult tidy = import_bytes(one_newline, vertical, unit_local());
    CHECK_EQ(status_of(tidy), ok_status());
    CHECK_EQ(tidy.warnings.size(), std::size_t{0});

    Bytes two_extra = make_pgm_binary(2, 2, 255, {0, 100, 200, 255});
    two_extra.push_back('\n');
    two_extra.push_back('\n');
    const HeightmapImportResult chatty = import_bytes(two_extra, vertical, unit_local());
    CHECK_EQ(status_of(chatty), ok_status());
    CHECK_EQ(chatty.warnings.size(), std::size_t{1});
    CHECK_TRUE(warning_says(chatty, "PGM has 2 bytes after the raster"));

    // And an exact fit says nothing either.
    const HeightmapImportResult exact =
        import_bytes(make_pgm_binary(2, 2, 255, {0, 100, 200, 255}), vertical, unit_local());
    CHECK_EQ(status_of(exact), ok_status());
    CHECK_EQ(exact.warnings.size(), std::size_t{0});
}

TEST(HeightmapIO, ascii_and_binary_pgm_agree) {
    const std::vector<std::uint16_t> samples = {0, 51, 102, 153, 204, 255};
    const HeightmapImportResult ascii =
        import_bytes(make_pgm_ascii(3, 2, 255, samples), ElevationRange::metres(0.0, 255.0),
                     unit_local());
    const HeightmapImportResult binary =
        import_bytes(make_pgm_binary(3, 2, 255, samples), ElevationRange::metres(0.0, 255.0),
                     unit_local());

    CHECK_EQ(status_of(ascii), ok_status());
    CHECK_EQ(status_of(binary), ok_status());
    CHECK_EQ(std::string{stratum::procgen::to_string(ascii.source.format)},
             std::string{"PGM (P2, ASCII)"});
    CHECK_TRUE(ascii.heightmap.data == binary.heightmap.data);
    // Pinned to absolute values too: two decoders that agree with each other and
    // disagree with the file would otherwise pass.
    if (ascii.heightmap.data.size() == 6) {
        CHECK_NEAR(ascii.heightmap.at(0, 1), 0.0, 1e-3);
        CHECK_NEAR(ascii.heightmap.at(2, 1), 102.0, 1e-3);
        CHECK_NEAR(ascii.heightmap.at(0, 0), 153.0, 1e-3);
        CHECK_NEAR(ascii.heightmap.at(2, 0), 255.0, 1e-3);
    }
}

// ============================================================================
// PNG
// ============================================================================

TEST(HeightmapIO, png_16bit_round_trips_sample_values) {
    // Values chosen so that a byte-swap, an 8-bit truncation or a widening by
    // 257 all produce something different.
    const Bytes png = make_png(3, 2, 16, 0, {0x1234, 0x0000, 0xFFFF, 0x00FF, 0xFF00, 0x8000});

    const HeightmapImportResult result =
        import_bytes(png, ElevationRange::metres(0.0, 65535.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(std::string{stratum::procgen::to_string(result.source.format)}, std::string{"PNG"});
    CHECK_EQ(result.source.bits_per_sample, 16);
    CHECK_EQ(result.source.max_sample_value, std::uint32_t{65535});
    CHECK_EQ(result.source.width, 3);
    CHECK_EQ(result.source.height, 2);
    if (result.heightmap.data.size() != 6) {
        return;
    }
    // North row (image row 0) is heightmap row 1.
    CHECK_NEAR(result.heightmap.at(0, 1), 4660.0, 1e-1);
    CHECK_NEAR(result.heightmap.at(1, 1), 0.0, 1e-1);
    CHECK_NEAR(result.heightmap.at(2, 1), 65535.0, 1e-1);
    // South row (image row 1) is heightmap row 0.
    CHECK_NEAR(result.heightmap.at(0, 0), 255.0, 1e-1);
    CHECK_NEAR(result.heightmap.at(1, 0), 65280.0, 1e-1);
    CHECK_NEAR(result.heightmap.at(2, 0), 32768.0, 1e-1);
}

TEST(HeightmapIO, png_8bit_loads_and_is_reported_as_eight_bit) {
    const Bytes png = make_png(2, 2, 8, 0, {0, 255, 51, 204});

    const HeightmapImportResult result =
        import_bytes(png, ElevationRange::metres(0.0, 1200.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(result.source.bits_per_sample, 8);
    // 255, not 65535. stb widens an 8-bit sample to 16 bits by multiplying by
    // 257, and reporting the widened maximum would claim a vertical precision
    // the file does not have.
    CHECK_EQ(result.source.max_sample_value, std::uint32_t{255});
    CHECK_NEAR(result.vertical_quantum_metres, 1200.0 / 255.0, 1e-9);
    CHECK_FALSE(result.warnings.empty());
    if (result.heightmap.data.size() == 4) {
        CHECK_NEAR(result.heightmap.at(0, 1), 0.0, 1e-2);
        CHECK_NEAR(result.heightmap.at(1, 1), 1200.0, 1e-2);
        CHECK_NEAR(result.heightmap.at(0, 0), (51.0 / 255.0) * 1200.0, 1e-2);
    }
}

TEST(HeightmapIO, a_one_bit_png_reports_one_bit_and_a_200_metre_quantum) {
    // PNG greyscale is legally 1, 2, 4, 8 or 16 bits. Asking the decoder "is
    // this 16-bit?" answers no for all four of the others, and stb SCALES a
    // 1-bit sample up to fill 0..255, so the heights come out right and the
    // depth report is wrong in total silence: 8 bits and a full scale of 255
    // against a file with two levels in it.
    //
    // That single number is the one the caller is meant to act on. Over a
    // 0-200 m range the truth is 200 m per step -- terrain that can only be sea
    // level or hilltop -- and the 8-bit lie reports 0.78 m, which is not only
    // 255 times too small but falls BELOW kCoarseVerticalQuantumMetres, so the
    // warning written to catch exactly this case is suppressed by it.
    const Bytes png = make_png(4, 2, 1, 0,
                               {0, 1, 0, 1,
                                1, 1, 0, 0});

    const HeightmapImportResult result =
        import_bytes(png, ElevationRange::metres(0.0, 200.0), unit_local());

    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(result.source.bits_per_sample, 1);
    CHECK_EQ(result.source.max_sample_value, std::uint32_t{1});
    CHECK_NEAR(result.vertical_quantum_metres, 200.0, 1e-9);
    CHECK_EQ(result.warnings.size(), std::size_t{1});
    CHECK_TRUE(warning_says(result, "1-bit source over a 200.00 m range"));
    CHECK_TRUE(warning_says(result, "resolves only 200.000 m per sample step"));

    if (result.heightmap.data.size() != 8) {
        return;
    }
    // The heights themselves are unaffected -- which is why the wrong depth was
    // invisible. Heightmap row 0 is SOUTH, from file row 1.
    CHECK_NEAR(result.heightmap.at(0, 0), 200.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(1, 0), 200.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(2, 0), 0.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(3, 0), 0.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(0, 1), 0.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(1, 1), 200.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(2, 1), 0.0, 1e-3);
    CHECK_NEAR(result.heightmap.at(3, 1), 200.0, 1e-3);
    // Two levels and no third: the file really is one bit deep, so the reported
    // depth above is describing this and not a 255-level image that happens to
    // use two of them.
    for (float value : result.heightmap.data) {
        CHECK_TRUE(value == 0.0f || value == 200.0f);
    }
}

TEST(HeightmapIO, two_and_four_bit_pngs_report_their_own_full_scale) {
    // 3 and 15, not 255. The samples stb hands back are scaled to fill 0..255 by
    // a factor of 85 and 17, and dividing that back out is exact -- so these
    // assert the RAW values as well as the reported scale, which is what
    // separates "read the depth from the file" from "read the depth from the
    // file and then forget to undo the expansion", where every sample would come
    // back at 85 or 17 times full scale.
    const ElevationRange vertical = ElevationRange::metres(0.0, 200.0);

    const Bytes two_bit = make_png(4, 2, 2, 0,
                                   {0, 1, 2, 3,
                                    3, 2, 1, 0});
    const HeightmapImportResult two = import_bytes(two_bit, vertical, unit_local());
    CHECK_EQ(status_of(two), ok_status());
    CHECK_EQ(two.source.bits_per_sample, 2);
    CHECK_EQ(two.source.max_sample_value, std::uint32_t{3});
    CHECK_NEAR(two.vertical_quantum_metres, 200.0 / 3.0, 1e-9);
    CHECK_TRUE(warning_says(two, "2-bit source over a 200.00 m range"));
    if (two.heightmap.data.size() == 8) {
        CHECK_NEAR(two.heightmap.at(0, 1), 0.0, 1e-3);                  // north, raw 0
        CHECK_NEAR(two.heightmap.at(3, 1), 200.0, 1e-3);                // north, raw 3
        CHECK_NEAR(two.heightmap.at(0, 0), 200.0, 1e-3);                // south, raw 3
        CHECK_NEAR(two.heightmap.at(2, 0), (1.0 / 3.0) * 200.0, 1e-3);  // south, raw 1
    }

    const Bytes four_bit = make_png(4, 2, 4, 0,
                                    {0, 5, 10, 15,
                                     15, 10, 5, 0});
    const HeightmapImportResult four = import_bytes(four_bit, vertical, unit_local());
    CHECK_EQ(status_of(four), ok_status());
    CHECK_EQ(four.source.bits_per_sample, 4);
    CHECK_EQ(four.source.max_sample_value, std::uint32_t{15});
    CHECK_NEAR(four.vertical_quantum_metres, 200.0 / 15.0, 1e-9);
    CHECK_TRUE(warning_says(four, "4-bit source over a 200.00 m range"));
    if (four.heightmap.data.size() == 8) {
        CHECK_NEAR(four.heightmap.at(0, 1), 0.0, 1e-3);                   // north, raw 0
        CHECK_NEAR(four.heightmap.at(3, 1), 200.0, 1e-3);                 // north, raw 15
        CHECK_NEAR(four.heightmap.at(1, 0), (10.0 / 15.0) * 200.0, 1e-3); // south, raw 10
        CHECK_NEAR(four.heightmap.at(3, 0), 0.0, 1e-3);                   // south, raw 0
    }

    // Control at the depth above them: 8-bit still reports 8 and 255, so the
    // three reports above are the file's depth and not a fixed table indexed by
    // something that happens to line up.
    const Bytes eight_bit = make_png(4, 2, 8, 0, {0, 85, 170, 255, 255, 170, 85, 0});
    const HeightmapImportResult eight = import_bytes(eight_bit, vertical, unit_local());
    CHECK_EQ(status_of(eight), ok_status());
    CHECK_EQ(eight.source.bits_per_sample, 8);
    CHECK_EQ(eight.source.max_sample_value, std::uint32_t{255});
    if (eight.heightmap.data.size() == 8) {
        CHECK_NEAR(eight.heightmap.at(1, 1), (85.0 / 255.0) * 200.0, 1e-3);
    }
}

TEST(HeightmapIO, a_colour_png_is_refused_rather_than_averaged_to_grey) {
    // A packed colour DEM -- Mapbox Terrain-RGB and friends -- is a 24-bit
    // integer split across three channels. Collapsing it to luminance produces a
    // height field that is smooth, plausible and wrong at every sample.
    std::vector<std::uint16_t> rgb;
    for (int i = 0; i < 4; ++i) {
        rgb.push_back(static_cast<std::uint16_t>(10 + i));
        rgb.push_back(static_cast<std::uint16_t>(80 + i));
        rgb.push_back(static_cast<std::uint16_t>(200 + i));
    }
    const Bytes png = make_png(2, 2, 8, 2, rgb);

    const HeightmapImportResult result =
        import_bytes(png, ElevationRange::metres(0.0, 1000.0), unit_local());

    CHECK_EQ(status_of(result), std::string{"UnsupportedChannelCount"});
    CHECK_TRUE(result.heightmap.data.empty());
}

TEST(HeightmapIO, a_truncated_png_is_refused) {
    const Bytes whole = make_png(8, 8, 16, 0, std::vector<std::uint16_t>(64, 0x4321));

    // Control first: the fixture really is a valid PNG, so the refusals below
    // are about the truncation and not about a broken writer.
    const HeightmapImportResult intact =
        import_bytes(whole, ElevationRange::metres(0.0, 65535.0), unit_local());
    CHECK_EQ(status_of(intact), ok_status());

    // Cut inside the IDAT payload: the header still parses, the raster does not.
    Bytes cut_raster(whole.begin(), whole.end() - 40);
    const HeightmapImportResult short_raster =
        import_bytes(cut_raster, ElevationRange::metres(0.0, 65535.0), unit_local());
    CHECK_FALSE(short_raster.ok());
    CHECK_TRUE(short_raster.heightmap.data.empty());
    // The header block is filled in as far as the parse got, which for a cut
    // raster is all of it: the depth comes out of IHDR, so it is known even
    // though not one sample was decoded.
    CHECK_EQ(short_raster.source.width, 8);
    CHECK_EQ(short_raster.source.height, 8);
    CHECK_EQ(short_raster.source.bits_per_sample, 16);
    CHECK_EQ(short_raster.source.max_sample_value, std::uint32_t{65535});

    // Cut inside IHDR: nothing is knowable about the image at all.
    Bytes cut_header(whole.begin(), whole.begin() + 20);
    const HeightmapImportResult short_header =
        import_bytes(cut_header, ElevationRange::metres(0.0, 65535.0), unit_local());
    CHECK_FALSE(short_header.ok());
    CHECK_TRUE(short_header.heightmap.data.empty());
}

TEST(HeightmapIO, a_png_is_recognised_by_its_signature_not_its_extension) {
    // Sniffed from the leading bytes. The in-memory overload has no name at all,
    // which is the point: the same bytes decode whatever they are called.
    const Bytes png = make_png(2, 2, 8, 0, {0, 255, 51, 204});
    const HeightmapImportResult result =
        import_bytes(png, ElevationRange::metres(0.0, 255.0), unit_local());
    CHECK_EQ(status_of(result), ok_status());
    CHECK_EQ(std::string{stratum::procgen::to_string(result.source.format)}, std::string{"PNG"});

    // All EIGHT bytes, not the four that spell "\x89PNG". The last four exist to
    // catch a file mangled by a transfer that translated line endings or stripped
    // the high bit, and nothing else here feeds a file with a correct prefix and
    // a broken tail -- so a comparison shortened to four bytes would pass.
    Bytes prefix_only = png;
    prefix_only[4] = 0x00;
    prefix_only[5] = 0x00;
    prefix_only[6] = 0x00;
    prefix_only[7] = 0x00;
    const HeightmapImportResult mangled =
        import_bytes(prefix_only, ElevationRange::metres(0.0, 255.0), unit_local());
    // Not MalformedHeader: it never reached the PNG decoder. It is refused as
    // "this is not a format I read", which is the honest answer.
    CHECK_EQ(status_of(mangled), std::string{"UnknownFormat"});
    CHECK_TRUE(message_says(mangled, "not a Netpbm file"));
}

TEST(HeightmapIO, png_and_pgm_of_the_same_data_agree) {
    // Two independent decoders, one expectation. A change that breaks the row
    // flip or the normalisation in only one of them shows up here.
    const std::vector<std::uint16_t> samples = {0x0000, 0x4000, 0x8000, 0xC000, 0xFFFF, 0x2000};
    const HeightmapImportResult from_png =
        import_bytes(make_png(3, 2, 16, 0, samples), ElevationRange::metres(0.0, 6553.5),
                     unit_local());
    const HeightmapImportResult from_pgm =
        import_bytes(make_pgm_binary(3, 2, 65535, samples), ElevationRange::metres(0.0, 6553.5),
                     unit_local());

    CHECK_EQ(status_of(from_png), ok_status());
    CHECK_EQ(status_of(from_pgm), ok_status());
    CHECK_TRUE(from_png.heightmap.data == from_pgm.heightmap.data);
    if (from_png.heightmap.data.size() == 6) {
        // South row (image row 1) is {0xC000, 0xFFFF, 0x2000}; 0xFFFF is full scale.
        CHECK_NEAR(from_png.heightmap.at(1, 0), 6553.5, 1e-2);
        // North row (image row 0) is {0x0000, 0x4000, 0x8000}.
        CHECK_NEAR(from_png.heightmap.at(0, 1), 0.0, 1e-2);
        CHECK_NEAR(from_png.heightmap.at(2, 1), (32768.0 / 65535.0) * 6553.5, 1e-2);
    }
}

// ============================================================================
// The file path
// ============================================================================

namespace {

/// A unique path under the system temp directory, removed by the caller.
std::filesystem::path temp_fixture_path(const char* stem) {
    static int counter = 0;
    return std::filesystem::temp_directory_path() /
           (std::string{"stratum_heightmap_"} + stem + "_" + std::to_string(++counter) + ".pgm");
}

bool write_file(const std::filesystem::path& path, const Bytes& bytes) {
    std::ofstream stream(path, std::ios::binary);
    if (!stream) {
        return false;
    }
    stream.write(reinterpret_cast<const char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(stream);
}

} // namespace

TEST(HeightmapIO, a_pgm_written_to_disk_reads_back_identically) {
    const Bytes pgm = make_pgm_binary(4, 3, 255,
                                      {0, 51, 102, 153, 204, 255, 10, 20, 30, 40, 50, 60});
    const std::filesystem::path path = temp_fixture_path("roundtrip");
    if (!write_file(path, pgm)) {
        CHECK_TRUE(false); // could not stage the fixture; the test proves nothing
        return;
    }

    const ElevationRange vertical = ElevationRange::metres(0.0, 255.0);
    const HeightmapImportResult from_disk = import_heightmap(path, vertical, unit_local());
    const HeightmapImportResult from_memory = import_bytes(pgm, vertical, unit_local());

    std::error_code ignored;
    std::filesystem::remove(path, ignored);

    CHECK_EQ(status_of(from_disk), ok_status());
    CHECK_TRUE(from_disk.heightmap.data == from_memory.heightmap.data);
    // NOT covered here, and deliberately: the short-read guard in
    // import_heightmap(), which compares gcount() against the size the stat
    // reported. Reaching it needs the file to SHRINK between the stat and the
    // read, which no portable test can arrange -- the only lever on Linux is a
    // sysfs attribute whose declared size exceeds what a read returns, and
    // pinning the suite to sysfs layout buys less than it costs. The guard is
    // fail-closed, so an untested regression there refuses a good file rather
    // than accepting a bad one.
    // Absolute values as well, so "both empty" cannot pass.
    CHECK_EQ(from_disk.heightmap.data.size(), std::size_t{12});
    if (from_disk.heightmap.data.size() == 12) {
        CHECK_NEAR(from_disk.heightmap.at(0, 0), 30.0, 1e-3);
        CHECK_NEAR(from_disk.heightmap.at(3, 2), 153.0, 1e-3);
    }
}

TEST(HeightmapIO, a_missing_file_is_reported_as_missing) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "stratum_heightmap_does_not_exist.pgm";
    std::error_code ignored;
    std::filesystem::remove(path, ignored);

    const HeightmapImportResult result =
        import_heightmap(path, ElevationRange::metres(0.0, 100.0), unit_local());

    CHECK_EQ(status_of(result), std::string{"FileNotFound"});
    CHECK_TRUE(result.heightmap.data.empty());
}

TEST(HeightmapIO, a_path_that_is_not_a_regular_file_is_refused_as_unreadable) {
    // A directory exists, so the FileNotFound guard lets it through, and
    // file_size() then fails on it with the SAME status this returns -- which is
    // why the is_regular_file() check could be deleted and the status assertion
    // would not move. The message is what says which guard stopped it, and
    // "not a regular file" is the one a user can act on.
    const std::filesystem::path directory = std::filesystem::temp_directory_path();
    const HeightmapImportResult result =
        import_heightmap(directory, ElevationRange::metres(0.0, 100.0), unit_local());

    CHECK_EQ(status_of(result), std::string{"FileUnreadable"});
    CHECK_TRUE(message_says(result, "not a regular file"));
    CHECK_TRUE(result.heightmap.data.empty());
}

TEST(HeightmapIO, a_file_past_the_byte_cap_is_refused_without_being_read) {
    const Bytes pgm = make_pgm_binary(16, 16, 255, std::vector<std::uint16_t>(256, 128));
    const std::filesystem::path path = temp_fixture_path("toolarge");
    if (!write_file(path, pgm)) {
        CHECK_TRUE(false);
        return;
    }

    HeightmapImportOptions tiny;
    tiny.max_file_bytes = 16;
    const HeightmapImportResult refused =
        import_heightmap(path, ElevationRange::metres(0.0, 100.0), unit_local(), tiny);
    // Control: the same file under the default cap.
    const HeightmapImportResult accepted =
        import_heightmap(path, ElevationRange::metres(0.0, 100.0), unit_local());

    std::error_code ignored;
    std::filesystem::remove(path, ignored);

    CHECK_EQ(status_of(refused), std::string{"FileTooLarge"});
    CHECK_TRUE(refused.heightmap.data.empty());
    CHECK_EQ(status_of(accepted), ok_status());
}

TEST(HeightmapIO, a_failed_file_import_names_the_file_in_its_message) {
    const Bytes truncated = make_pgm_binary(64, 64, 255, {1, 2, 3});
    const std::filesystem::path path = temp_fixture_path("named");
    if (!write_file(path, truncated)) {
        CHECK_TRUE(false);
        return;
    }

    const HeightmapImportResult result =
        import_heightmap(path, ElevationRange::metres(0.0, 100.0), unit_local());

    std::error_code ignored;
    std::filesystem::remove(path, ignored);

    CHECK_EQ(status_of(result), std::string{"Truncated"});
    // A batch import of four hundred tiles is unusable if the diagnostic does
    // not say which tile.
    CHECK((result.message.find(path.string())) != (std::string::npos));
}
