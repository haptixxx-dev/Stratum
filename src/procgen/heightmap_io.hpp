// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file heightmap_io.hpp
 * @brief Loads real elevation data from an image file into a procgen Heightmap
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Terrain in this project has been noise-only: TerrainGenerator makes a plausible
 * landscape from a seed. This is the other source -- a real DEM tile, exported
 * from a national survey or from an SRTM/Copernicus crop, dropped onto the same
 * `Heightmap` the rest of the pipeline already consumes:
 *
 * @code
 *     import_heightmap() -> Heightmap -> TerrainCarve -> TerrainMeshBuilder -> Mesh
 * @endcode
 *
 * The decoding is the easy half and the least interesting. Three things about a
 * heightmap image are genuinely dangerous, and the shape of this API is chosen
 * around them.
 *
 * ### 1. An image carries no vertical unit
 *
 * A PNG sample is a number between 0 and 255, or between 0 and 65535. It is not
 * metres. The mapping from sample to metres lives in the metadata of whatever
 * tool exported the file -- a .tfw, a GDAL band description, a line in a README,
 * or nowhere at all -- and it is the single easiest thing in this whole subsystem
 * to get wrong by a factor of ten, because the result still looks like terrain.
 * Mountains a kilometre too tall read as "dramatic", not as "broken".
 *
 * So ElevationRange has no default constructor and no default value anywhere in
 * this API. It is a required, named argument of every entry point:
 *
 * @code
 *     import_heightmap(path, ElevationRange::metres(0.0, 1200.0), placement);
 * @endcode
 *
 * There is deliberately no `ElevationRange{}`, no `= {}` default parameter and no
 * "guess it from the data" mode. Guessing from the data is worse than useless:
 * normalising to the observed minimum and maximum makes the SAME terrain change
 * scale when you crop the highest peak out of the tile, so two adjacent tiles of
 * one mountain range no longer meet. See the normalisation note on
 * HeightmapSourceInfo::max_sample_value.
 *
 * ### 2. A heightmap covers a place
 *
 * A DEM tile is not a texture; it has corners with latitudes and longitudes, and
 * it is only in the right place if those corners are projected through the SAME
 * origin the OSM geometry was projected through. HeightmapPlacement::geographic()
 * takes the extent and a CoordinateConverter and refuses an uninitialised one,
 * because `CoordinateConverter::wgs84_to_local()` silently returns raw Web
 * Mercator when `set_origin()` was never called (coordinates.cpp:63) -- the
 * terrain would land several thousand kilometres from the buildings and the first
 * symptom would look like a culling bug.
 *
 * HeightmapPlacement::local() exists for the case where there is no geographic
 * claim to make -- a hand-authored greyscale ramp used as a test surface. It
 * takes the origin and cell size directly, and is honest that no georeferencing
 * happened rather than inventing a plausible one.
 *
 * ### 3. The file is untrusted
 *
 * SECURITY.md names heightmap images explicitly: "Parser crashes, out-of-bounds
 * reads and unbounded allocations reachable from a malformed input file are in
 * scope and are the most likely class of real vulnerability here."
 *
 * Every entry point here returns a HeightmapImportResult and throws nothing. A
 * header that claims 2^31 samples, a binary PGM whose declared size exceeds the
 * bytes that follow it, a PNG truncated mid-IDAT: each is refused with a status
 * and a message, and nothing is allocated on the strength of a number that came
 * out of the file. The dimension caps are in HeightmapImportOptions and apply
 * BEFORE the allocation, not after.
 *
 * ### Formats
 *
 * PGM (P2 ASCII and P5 binary, 8- and 16-bit) is parsed here. PNG goes through
 * the vendored stb_image, restricted at compile time to the PNG decoder alone --
 * see the STBI_ONLY_PNG note in heightmap_io.cpp for why the other nine decoders
 * are compiled out rather than merely unused.
 *
 * Greyscale PNG is accepted at all five legal depths, 1, 2, 4, 8 and 16. The
 * sub-byte ones are not useful terrain and they are not refused either: they
 * decode correctly, they are reported at their real depth, and the coarse-quantum
 * warning then says out loud that a 1-bit file over a 200 m range resolves 200 m
 * per step. Refusing them would be defensible; reporting them as 8-bit, which is
 * what asking the decoder rather than the file gives you, would not.
 *
 * PGM is supported not because anyone ships DEMs in it but because it is the one
 * raster format whose bytes a test can write by hand, which is what lets the
 * whole path be tested with no binary fixture checked into the repository.
 *
 * ### Coordinates and axes
 *
 * A Heightmap's axes are LOCAL X and LOCAL Y, not render-space X and Z. There is
 * no sign flip in a Heightmap; the flip belongs one step later, in render space,
 * and TerrainMeshBuilder applies it. This file follows terrain_carve.hpp exactly:
 *
 * @code
 *     heightmap sample (ix, iz) -> local (origin.x + ix*cell_size_x,
 *                                         origin.y + iz*cell_size_z)
 * @endcode
 *
 * Local Y increases NORTHWARD, so heightmap row 0 is the SOUTH edge. Image row 0
 * is the NORTH edge in every raster convention that matters (PNG, PGM, GeoTIFF
 * north-up). The import therefore flips rows. This is invisible on any symmetric
 * test image and mirrors the terrain about the centre parallel when wrong, which
 * is why HeightmapImportOptions::source_row_zero_is_north is spelled out rather
 * than assumed, and why the suite tests it with an asymmetric gradient.
 *
 * Everything here is stratum_core: no SDL, no ImGui, no rendering API.
 */

#pragma once

#include "osm/coordinates.hpp"
#include "osm/types.hpp"
#include "procgen/terrain_generator.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace stratum::procgen {

// ============================================================================
// Vertical scale
// ============================================================================

/**
 * @brief The elevation, in metres, that sample value 0 and full scale mean
 *
 * Construct only through ElevationRange::metres(). There is no default
 * constructor on purpose: see the "an image carries no vertical unit" section of
 * the file comment. A defaulted vertical scale is the bug this type exists to
 * make unrepresentable, so adding `ElevationRange() = default` here would undo
 * the whole point of the class.
 *
 * The mapping is linear over the FORMAT's full scale:
 *
 * @code
 *     metres = min_metres + (raw / max_sample_value) * (max_metres - min_metres)
 * @endcode
 *
 * "Full scale" is the maximum the format can represent -- 255, 65535, or the
 * PGM header's declared maxval -- and never the largest value that happens to
 * appear in the data. HeightmapSourceInfo::max_sample_value carries which one
 * was used.
 *
 * Data already stored in metres is expressed by making the range match the
 * encoding rather than by adding a second mode: 16-bit samples that are metres
 * directly are `ElevationRange::metres(0.0, 65535.0)`. One model, one formula,
 * nothing to pick wrong.
 */
class ElevationRange {
public:
    /// No default: a heightmap with an unstated vertical scale is not a heightmap.
    ElevationRange() = delete;

    /**
     * @brief Build a range from the two elevations the extremes of the image mean
     *
     * @param at_zero       Elevation in metres of a sample of value 0
     * @param at_full_scale Elevation in metres of a sample at the format maximum
     *
     * Both are metres above the same datum the rest of the scene uses. Validity
     * is checked by the importer, not here, so that a bad range is reported
     * through HeightmapImportResult like every other input fault instead of
     * through an exception or an assert.
     */
    [[nodiscard]] static constexpr ElevationRange metres(double at_zero,
                                                         double at_full_scale) noexcept {
        return ElevationRange{at_zero, at_full_scale};
    }

    [[nodiscard]] constexpr double min_metres() const noexcept { return m_min; }
    [[nodiscard]] constexpr double max_metres() const noexcept { return m_max; }

    /// Metres between sample 0 and full scale. Negative or zero is refused.
    [[nodiscard]] constexpr double span_metres() const noexcept { return m_max - m_min; }

    /**
     * @brief Map a normalised sample in [0, 1] to metres
     * @param normalized raw / max_sample_value
     */
    [[nodiscard]] constexpr double to_metres(double normalized) const noexcept {
        return m_min + normalized * (m_max - m_min);
    }

    /**
     * @brief Whether this range can be used at all
     *
     * Requires both ends finite, a strictly positive span, and both ends inside
     * @ref kMaxAbsElevationMetres.
     *
     * A zero span is refused rather than treated as "flat terrain". Flat terrain
     * needs no heightmap, and `metres(100.0, 100.0)` is far more likely to be a
     * typo for `metres(100.0, 1000.0)` than a deliberate request for a plane.
     * An inverted span -- max below min -- is refused for the same reason: it is
     * the argument order mistake, and silently accepting it produces a terrain
     * that is upside down but still looks like terrain.
     */
    [[nodiscard]] bool is_valid() const noexcept;

private:
    constexpr ElevationRange(double at_zero, double at_full_scale) noexcept
        : m_min(at_zero), m_max(at_full_scale) {}

    double m_min;
    double m_max;
};

/**
 * @brief Largest elevation magnitude accepted, in metres
 *
 * Everest is 8,849 m and Challenger Deep is -10,935 m, so 100 km is two
 * decimal orders past anything real. It is not a physical limit; it is a
 * tripwire for a caller who passed feet, centimetres or raw sample counts where
 * metres were wanted, which is exactly the factor-of-N mistake this file is
 * built to catch.
 */
inline constexpr double kMaxAbsElevationMetres = 100000.0;

/**
 * @brief Vertical quantum above which the result carries a coarseness warning
 *
 * One metre per least-significant bit. Below this, quantisation is invisible in
 * a city-scale scene; above it, 8-bit terrain shows as visible terracing on
 * shallow slopes and as stair-stepping in the shadow pass, because the normal
 * computed from a quantised height field is piecewise constant.
 */
inline constexpr double kCoarseVerticalQuantumMetres = 1.0;

// ============================================================================
// Placement
// ============================================================================

/// Which of the two placement modes a HeightmapPlacement carries.
enum class PlacementKind {
    Geographic, ///< Image extent is a WGS84 bounding box projected through a converter
    Local       ///< Image extent is given directly in local metres; no geographic claim
};

/**
 * @brief Where the imported heightmap sits, and in what units
 *
 * Construct through HeightmapPlacement::geographic() or
 * HeightmapPlacement::local(). No default constructor, for the same reason
 * ElevationRange has none: a defaulted placement is a heightmap silently parked
 * at the origin with one-metre cells.
 *
 * The converter is stored BY VALUE. It is three doubles and a bool, so copying
 * costs nothing, and a placement that outlives the converter it was built from is
 * a use-after-free that would show up as terrain in the wrong hemisphere rather
 * than as a crash.
 */
class HeightmapPlacement {
public:
    /**
     * @brief Place the image on the Earth
     *
     * The image corners map to the bounding box corners, and those corners are
     * projected to local metres through @p converter -- the same converter the
     * OSM import used, so that terrain and buildings share one origin.
     *
     * @param bounds    Geographic extent the image covers, edge to edge
     * @param converter Scene projection. Must be initialised; an uninitialised one
     *                  is refused with HeightmapImportStatus::InvalidPlacement
     *                  rather than falling back to raw Mercator the way
     *                  CoordinateConverter itself does.
     *
     * @note Local units are Web Mercator metres, not ground metres. Mercator
     *       stretches by 1/cos(latitude) in both axes, so at Dublin a 10-unit
     *       local span is 5.97 m of ground. This importer does NOT correct for it,
     *       because MeshBuilder, QuadTree and the road solver all work in these
     *       units and correcting here alone would slide the terrain relative to
     *       the buildings. HeightmapImportResult::ground_metres_per_unit reports
     *       the factor for anything that shows a distance to a user.
     */
    [[nodiscard]] static HeightmapPlacement geographic(const osm::BoundingBox& bounds,
                                                       const osm::CoordinateConverter& converter);

    /**
     * @brief Place the image directly in local metres, making no geographic claim
     *
     * @param origin_local  Local (x, y) of grid sample (0, 0) -- the SOUTH-WEST
     *                      corner, since local y increases northward
     * @param cell_size_x   Metres between columns
     * @param cell_size_y   Metres between rows, along local y
     */
    [[nodiscard]] static HeightmapPlacement local(const glm::vec2& origin_local,
                                                  float cell_size_x, float cell_size_y);

    [[nodiscard]] PlacementKind kind() const noexcept { return m_kind; }
    [[nodiscard]] const osm::BoundingBox& bounds() const noexcept { return m_bounds; }
    [[nodiscard]] const osm::CoordinateConverter& converter() const noexcept { return m_converter; }
    [[nodiscard]] glm::vec2 origin_local() const noexcept { return m_origin; }
    [[nodiscard]] float cell_size_x() const noexcept { return m_cell_x; }
    [[nodiscard]] float cell_size_y() const noexcept { return m_cell_y; }

private:
    /// Private, not deleted: the two factories need it, and nobody else may have it.
    /// A default-constructed placement would be "at the origin, one metre cells",
    /// which is a claim, and this type exists so that claim must be made out loud.
    HeightmapPlacement() = default;

    PlacementKind m_kind = PlacementKind::Local;
    osm::BoundingBox m_bounds{};
    osm::CoordinateConverter m_converter{};
    glm::vec2 m_origin{0.0f};
    float m_cell_x = 1.0f;
    float m_cell_y = 1.0f;
};

// ============================================================================
// Options
// ============================================================================

/**
 * @brief Everything about an import that has a safe default
 *
 * Note what is NOT here: the vertical scale and the placement. Those are separate
 * required arguments precisely so they cannot be forgotten by writing `{}`.
 */
struct HeightmapImportOptions {
    /**
     * @brief Grid width to resample to, or 0 to keep the image width
     *
     * Set these from the terrain grid the scene already uses --
     * `TerrainConfig::resolution_x` and `resolution_z` -- when the image
     * resolution does not match it, which it almost never does. A 3601x3601 SRTM
     * tile resampled onto a 256x256 chunk grid is the normal case, not the
     * exception.
     */
    int target_width = 0;

    /// Grid height to resample to, or 0 to keep the image height. See target_width.
    int target_height = 0;

    /**
     * @brief Whether image row 0 is the north edge of the extent
     *
     * True for PNG, PGM, and any north-up GeoTIFF export, which is everything you
     * will meet in practice. False for the occasional tool that writes rows
     * bottom-up (some terrain editors, and anything that went through an OpenGL
     * framebuffer readback without a flip).
     *
     * Getting this wrong mirrors the terrain about its centre parallel. On a
     * symmetric test image that is undetectable, and on real data it produces a
     * landscape that is still perfectly plausible and completely wrong, with
     * rivers running up hillsides.
     */
    bool source_row_zero_is_north = true;

    /**
     * @brief Hard ceiling on width*height of the SOURCE image and of the target grid
     *
     * Checked against the header before a single sample is allocated. 2^26 is
     * 8192x8192, comfortably past any single DEM tile (SRTM1 is 3601x3601) and
     * 128 MB as 16-bit samples.
     *
     * This is the allocation guard SECURITY.md asks for. A PGM header is four
     * ASCII integers and anybody can write `P5 2000000000 2000000000 65535`.
     */
    std::uint64_t max_samples = 1ull << 26;

    /**
     * @brief Hard ceiling on the size of the file read from disk, in bytes
     *
     * 256 MiB. Applies before the read, so a multi-gigabyte file costs a stat and
     * not a multi-gigabyte buffer. Irrelevant to the in-memory overload, where the
     * caller already owns the bytes.
     */
    std::uint64_t max_file_bytes = 256ull * 1024 * 1024;
};

// ============================================================================
// Result
// ============================================================================

/// Container format an import actually decoded.
enum class HeightmapFormat {
    Unknown,
    PgmAscii,   ///< Netpbm P2
    PgmBinary,  ///< Netpbm P5
    Png         ///< PNG, greyscale or greyscale+alpha, 8- or 16-bit
};

/// Why an import failed, or Ok.
enum class HeightmapImportStatus {
    Ok,
    FileNotFound,
    FileUnreadable,
    FileTooLarge,
    UnknownFormat,
    MalformedHeader,
    Truncated,
    SampleAboveMaxval,
    DimensionsOutOfRange,
    UnsupportedChannelCount,
    DecodeFailed,
    InvalidElevationRange,
    InvalidPlacement,
    InvalidOptions
};

/// Stable spelling of a status, for logs and for test failure messages.
[[nodiscard]] const char* to_string(HeightmapImportStatus status) noexcept;

/// Stable spelling of a format.
[[nodiscard]] const char* to_string(HeightmapFormat format) noexcept;

/**
 * @brief What the file turned out to contain
 *
 * Reported for every SUCCESSFUL import, and filled in as far as it got for a
 * failed one, so a "truncated" message can still say what the header claimed.
 */
struct HeightmapSourceInfo {
    HeightmapFormat format = HeightmapFormat::Unknown;
    int width = 0;              ///< Source image width in samples
    int height = 0;             ///< Source image height in samples
    /**
     * @brief Bits the SOURCE stored each sample in
     *
     * 8 or 16 for a PGM, and 1, 2, 4, 8 or 16 for a PNG -- greyscale PNG allows
     * all five, and the sub-byte depths are read out of IHDR rather than guessed
     * from the decoder. stb hands a 1-bit sample back scaled to fill 0..255, so a
     * depth taken from the decoded pixels would say 8 for a file with two
     * distinct levels in it, and @ref max_sample_value and
     * HeightmapImportResult::vertical_quantum_metres would both be wrong by a
     * factor of 255 in the flattering direction.
     */
    int bits_per_sample = 0;

    /**
     * @brief Sample value that maps to ElevationRange::max_metres()
     *
     * Full scale of the SOURCE depth: 1, 3, 15, 255 or 65535 for a PNG, and the
     * DECLARED maxval from the header for a PGM -- which is a real third case,
     * because the Netpbm format lets a file say `maxval 1000` and mean it, so a
     * sample of 1000 is full scale even though the file stores 16 bits.
     *
     * It is never the largest value present in the data. Normalising by the
     * observed maximum would make the vertical scale depend on whether the
     * highest peak survived the crop, so two tiles of the same mountain range,
     * cropped differently, would not meet at their shared edge. That failure is
     * hard to see and impossible to debug from the geometry alone.
     */
    std::uint32_t max_sample_value = 0;
};

/**
 * @brief Outcome of an import
 *
 * Nothing in this file throws. Check ok() (or the explicit bool conversion)
 * before touching @ref heightmap, which is left empty on every failure path so a
 * caller that ignores the status gets nothing rather than something wrong.
 */
struct HeightmapImportResult {
    HeightmapImportStatus status = HeightmapImportStatus::DecodeFailed;

    /// Human-actionable detail: names the field, the claimed value and the limit.
    std::string message;

    /// What the source turned out to be. Partially filled on failure.
    HeightmapSourceInfo source{};

    /// The imported grid. Empty unless status == Ok.
    Heightmap heightmap{};

    /**
     * @brief Metres of elevation per least-significant sample step
     *
     * `span_metres / max_sample_value`. This is the number that makes 8-bit input
     * a real limitation rather than a stylistic one: over a 0-1200 m range it is
     * 4.71 m per step, so no slope gentler than 4.71 m per cell can be
     * represented at all and flat ground appears as terraces one cell wide.
     * The same range at 16 bits is 1.8 cm.
     */
    double vertical_quantum_metres = 0.0;

    /**
     * @brief Centre of the geographic extent, latitude, in degrees
     *
     * Reported as two named scalars rather than as the `glm::dvec2` that
     * `osm::BoundingBox::center()` returns, on purpose. That dvec2 is (lat, lon),
     * which is the reverse of every other dvec2 in this codebase -- CLAUDE.md
     * lists it as a trap, and it is one that costs an afternoon every time
     * because swapping the two still yields a legal coordinate somewhere else on
     * Earth. Naming the members removes the ordering from the type.
     *
     * Zero for PlacementKind::Local, where there is no geographic extent.
     */
    double centre_lat = 0.0;

    /// Centre of the geographic extent, longitude, in degrees. See centre_lat.
    double centre_lon = 0.0;

    /**
     * @brief Ground metres per local unit at @ref centre_lat
     *
     * `cos(centre_lat)`. Local coordinates are Web Mercator metres, which are
     * stretched by 1/cos(latitude); a distance read straight off the heightmap is
     * 68% too long in Dublin and 100% too long in Reykjavik, and it is plausible
     * enough that nobody notices. Anything that reports a length, an area or a
     * slope to a user multiplies by this.
     *
     * 1.0 for PlacementKind::Local, where no projection was applied.
     */
    double ground_metres_per_unit = 1.0;

    /**
     * @brief Non-fatal notes: coarse vertical quantum, trailing bytes, heavy
     *        downsampling
     *
     * Returned rather than logged. A library that logs decides for its caller
     * where the message goes and cannot be tested without capturing a sink; a
     * library that returns strings lets the editor put them in the import dialog,
     * lets a batch tool put them on stderr, and lets the suite assert on them.
     */
    std::vector<std::string> warnings;

    [[nodiscard]] bool ok() const noexcept { return status == HeightmapImportStatus::Ok; }
    [[nodiscard]] explicit operator bool() const noexcept { return ok(); }
};

// ============================================================================
// Entry points
// ============================================================================

/**
 * @brief Import a heightmap image from disk
 *
 * The format is chosen from the file's leading bytes, not from its extension: a
 * PNG named `.pgm` loads, and a `.png` that is actually a JPEG is refused by
 * name instead of producing a confusing decode error.
 *
 * @param path      File to read
 * @param vertical  What sample 0 and full scale mean in metres. Required; there
 *                  is no default and no inference. See ElevationRange.
 * @param placement Where the image sits. Required. See HeightmapPlacement.
 * @param options   Resampling and safety limits; every field has a safe default
 *
 * @return Result carrying the Heightmap on success, or a status and a message.
 *         Never throws; a missing file, an unreadable file and an oversized file
 *         are distinct statuses.
 */
[[nodiscard]] HeightmapImportResult import_heightmap(
    const std::filesystem::path& path,
    const ElevationRange& vertical,
    const HeightmapPlacement& placement,
    const HeightmapImportOptions& options = {});

/**
 * @brief Import a heightmap image already in memory
 *
 * The same code path as the file overload, which reads the bytes and calls this.
 * Exposed because it is what a future streamed or archived source needs, and
 * because it lets the suite feed a deliberately corrupt buffer without touching
 * the filesystem.
 *
 * @param bytes     Complete encoded image. Borrowed for the duration of the call.
 * @param vertical  See import_heightmap()
 * @param placement See import_heightmap()
 * @param options   See import_heightmap(). max_file_bytes is not consulted here.
 */
[[nodiscard]] HeightmapImportResult import_heightmap_from_memory(
    std::span<const std::uint8_t> bytes,
    const ElevationRange& vertical,
    const HeightmapPlacement& placement,
    const HeightmapImportOptions& options = {});

} // namespace stratum::procgen
