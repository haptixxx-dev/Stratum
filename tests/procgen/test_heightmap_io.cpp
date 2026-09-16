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
 * @param bit_depth    8 or 16
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
    CHECK_FALSE(coarse.warnings.empty());
    CHECK_TRUE(fine.warnings.empty());
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
    CHECK_FALSE(result.warnings.empty());
    // And the warning is not idle: the spike really is gone, because a 2x2
    // output samples only the four source corners.
    CHECK_NEAR(result.heightmap.at(0, 0), 0.0, 1e-6);
    CHECK_NEAR(result.heightmap.at(1, 1), 0.0, 1e-6);
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
    CHECK_EQ(status_of(import_bytes(pgm, vertical,
                                    HeightmapPlacement::geographic(never_set, converter))),
             std::string{"InvalidPlacement"});

    // Valid but with no area: every cell size would be zero.
    stratum::osm::BoundingBox sliver;
    sliver.min_lat = 53.30;
    sliver.max_lat = 53.40;
    sliver.min_lon = -6.25;
    sliver.max_lon = -6.25;
    CHECK_TRUE(sliver.is_valid());
    CHECK_EQ(
        status_of(import_bytes(pgm, vertical, HeightmapPlacement::geographic(sliver, converter))),
        std::string{"InvalidPlacement"});

    // Past the Mercator latitude limit, where wgs84_to_mercator() clamps in
    // silence and would hand back a squashed extent.
    stratum::osm::BoundingBox polar;
    polar.min_lat = 84.0;
    polar.max_lat = 89.0;
    polar.min_lon = -6.30;
    polar.max_lon = -6.20;
    CHECK_EQ(
        status_of(import_bytes(pgm, vertical, HeightmapPlacement::geographic(polar, converter))),
        std::string{"InvalidPlacement"});
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
}

TEST(HeightmapIO, a_truncated_ascii_pgm_is_refused) {
    const Bytes pgm = make_pgm_ascii(4, 4, 255, {1, 2, 3, 4, 5});

    const HeightmapImportResult result =
        import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local());

    CHECK_EQ(status_of(result), std::string{"Truncated"});
    CHECK_TRUE(result.heightmap.data.empty());
}

TEST(HeightmapIO, absurd_dimensions_are_refused_before_any_allocation) {
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
    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local(), tight)),
             std::string{"DimensionsOutOfRange"});

    // Control: the identical bytes load under the default cap.
    CHECK_EQ(status_of(import_bytes(pgm, ElevationRange::metres(0.0, 100.0), unit_local())),
             ok_status());
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

    CHECK_EQ(status_of(import_bytes(Bytes{}, vertical, unit_local())),
             std::string{"UnknownFormat"});
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
    CHECK_FALSE(result.warnings.empty());
    // The extra bytes must not have been read as data.
    if (result.heightmap.data.size() == 4) {
        CHECK_NEAR(result.heightmap.at(1, 0), 255.0, 1e-3);
    }
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
