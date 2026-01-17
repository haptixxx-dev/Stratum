/**
 * @file parser.hpp
 * @brief OSM file parser using libosmium
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This file provides the OSMParser class for parsing OpenStreetMap
 * data files (.osm XML and .pbf binary formats) and extracting
 * buildings, roads, and areas.
 *
 * Supported formats (auto-detected by extension):
 *   - .osm     - XML format
 *   - .pbf     - Protobuf binary format
 *   - .osm.pbf - Protobuf binary format
 *   - .osm.bz2 - Bzip2-compressed XML
 *   - .osm.gz  - Gzip-compressed XML
 */

#pragma once

#include "osm/types.hpp"
#include "osm/coordinates.hpp"
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace stratum::osm {

// ============================================================================
// Parser Configuration
// ============================================================================

/**
 * @brief Configuration options for OSM parsing
 */
struct ParserConfig {
    // What to import
    bool import_buildings = true;       ///< Import building=* elements
    bool import_roads = true;           ///< Import highway=* elements
    bool import_water = true;           ///< Import natural=water, waterway
    bool import_landuse = true;         ///< Import landuse=* elements
    bool import_natural = true;         ///< Import natural=* elements
    bool import_amenities = false;      ///< Import amenity=* (POIs, future use)

    // Processing options
    float default_building_height = 10.0f;  ///< Height when no tags available (m)
    float meters_per_level = 3.0f;          ///< For building:levels conversion
    float min_area_size = 1.0f;             ///< Minimum area to keep (sq meters)
    bool simplify_geometry = false;         ///< Apply Douglas-Peucker simplification
    double simplify_tolerance = 0.5;        ///< Simplification tolerance (meters)

    // Area filter (optional)
    std::optional<BoundingBox> filter_bounds; ///< Only load within this box
};

// ============================================================================
// Progress Reporting
// ============================================================================

/**
 * @brief Progress information during parsing
 */
struct ParseProgress {
    /**
     * @brief Current parsing stage
     */
    enum class Stage {
        ReadingFile,        ///< Opening and reading file
        ParsingNodes,       ///< Processing node elements
        ParsingWays,        ///< Processing way elements
        ParsingRelations,   ///< Processing relation elements
        BuildingAreas,      ///< Assembling multipolygon areas
        ConvertingCoords,   ///< Converting to local coordinates
        ProcessingRoads,    ///< Extracting road data
        ProcessingBuildings,///< Extracting building data
        ProcessingAreas,    ///< Extracting landuse/natural data
        Complete            ///< Parsing finished
    };

    Stage stage = Stage::ReadingFile;   ///< Current stage
    size_t current = 0;                 ///< Current item count
    size_t total = 0;                   ///< Total items (if known)
    std::string message;                ///< Human-readable status

    /**
     * @brief Get completion percentage
     * @return Percentage 0-100, or 0 if total unknown
     */
    [[nodiscard]] float percentage() const {
        return total > 0 ? static_cast<float>(current) / static_cast<float>(total) * 100.0f : 0.0f;
    }
};

/// Callback function type for progress updates
using ProgressCallback = std::function<void(const ParseProgress&)>;

// ============================================================================
// OSM Parser
// ============================================================================

/**
 * @brief Parser for OpenStreetMap data files
 *
 * Uses libosmium to parse .osm (XML) files and extracts:
 * - Buildings with footprints, heights, and types
 * - Roads with polylines, widths, and classifications
 * - Areas (water, parks, landuse) with polygons
 *
 * Coordinates are converted from WGS84 (lat/lon) to a local
 * meter-based coordinate system centered at the data bounds.
 *
 * Usage:
 * @code
 * stratum::osm::OSMParser parser;
 * parser.set_config(config);
 * parser.set_progress_callback([](const ParseProgress& p) {
 *     std::cout << p.message << std::endl;
 * });
 *
 * if (parser.parse("city.osm")) {
 *     auto data = parser.take_data();
 *     // Use data.buildings, data.roads, data.areas
 * }
 * @endcode
 */
class OSMParser {
public:
    OSMParser();
    ~OSMParser();

    // Non-copyable, movable
    OSMParser(const OSMParser&) = delete;
    OSMParser& operator=(const OSMParser&) = delete;
    OSMParser(OSMParser&&) noexcept;
    OSMParser& operator=(OSMParser&&) noexcept;

    /// @name Configuration
    /// @{

    /**
     * @brief Set parser configuration
     * @param config Configuration options
     */
    void set_config(const ParserConfig& config) { m_config = config; }

    /**
     * @brief Get current configuration
     * @return Reference to configuration
     */
    [[nodiscard]] const ParserConfig& get_config() const { return m_config; }

    /**
     * @brief Set callback for progress updates
     * @param callback Function to call with progress info
     */
    void set_progress_callback(ProgressCallback callback) {
        m_progress_callback = std::move(callback);
    }

    /// @}

    /// @name Parsing
    /// @{

    /**
     * @brief Parse an OSM file
     * @param filepath Path to .osm file
     * @return true on success, false on failure
     *
     * After successful parsing, use get_data() or take_data() to
     * access the parsed information.
     */
    bool parse(const std::filesystem::path& filepath);

    /**
     * @brief Get last error message
     * @return Error description, empty if no error
     */
    [[nodiscard]] const std::string& get_error() const { return m_error; }

    /**
     * @brief Check if data has been loaded
     * @return true if parse() succeeded
     */
    [[nodiscard]] bool has_data() const { return m_has_data; }

    /// @}

    /// @name Data Access
    /// @{

    /**
     * @brief Get parsed data (const reference)
     * @return Reference to parsed data
     */
    [[nodiscard]] const ParsedOSMData& get_data() const { return m_data; }

    /**
     * @brief Take parsed data (move semantics)
     * @return Parsed data (moved out)
     *
     * After calling this, has_data() returns false.
     */
    ParsedOSMData take_data();

    /**
     * @brief Clear all parsed data
     */
    void clear();

    /// @}

    /// @name Logging
    /// @{

    /**
     * @brief Log parsing statistics to spdlog
     */
    void log_statistics() const;

    /**
     * @brief Log sample data (first few elements) to spdlog
     * @param count Number of each type to log
     */
    void log_sample_data(size_t count = 5) const;

    /// @}

private:
    // Processing stages
    void process_ways();
    void process_buildings();
    void process_roads();
    void process_areas();
    void convert_coordinates();

    // Classification helpers
    [[nodiscard]] static RoadType classify_road(const TagMap& tags);
    [[nodiscard]] static BuildingType classify_building(const TagMap& tags);
    [[nodiscard]] static AreaType classify_area(const TagMap& tags);
    [[nodiscard]] static RoofType classify_roof(const TagMap& tags);

    // Estimation helpers
    [[nodiscard]] float estimate_building_height(const TagMap& tags) const;
    [[nodiscard]] static float estimate_road_width(RoadType type, const TagMap& tags);
    [[nodiscard]] static int estimate_road_lanes(RoadType type, const TagMap& tags);

    // Progress reporting
    void report_progress(ParseProgress::Stage stage, const std::string& message = "",
                        size_t current = 0, size_t total = 0);

    // Resolve way node references to coordinates
    [[nodiscard]] std::vector<glm::dvec2> resolve_way_coords(const OSMWay& way) const;

    // Data members
    ParserConfig m_config;
    ParsedOSMData m_data;
    CoordinateConverter m_converter;
    ProgressCallback m_progress_callback;
    std::string m_error;
    bool m_has_data = false;
};

} // namespace stratum::osm
