/**
 * @file types.hpp
 * @brief OSM data types and structures for Stratum
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This file contains all data structures used for parsing and processing
 * OpenStreetMap data, including raw OSM elements and processed geometry.
 */

#pragma once

#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <cstdint>

namespace stratum::osm {

// ============================================================================
// Type Aliases
// ============================================================================

/// Map of string key-value pairs for OSM tags
using TagMap = std::unordered_map<std::string, std::string>;

/// OSM node ID type (signed 64-bit, can be negative for new elements)
using NodeId = int64_t;

/// OSM way ID type
using WayId = int64_t;

/// OSM relation ID type
using RelationId = int64_t;

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief Classification of road types from highway=* tags
 */
enum class RoadType {
    Motorway,       ///< highway=motorway, motorway_link
    Trunk,          ///< highway=trunk, trunk_link
    Primary,        ///< highway=primary, primary_link
    Secondary,      ///< highway=secondary, secondary_link
    Tertiary,       ///< highway=tertiary, tertiary_link
    Residential,    ///< highway=residential, living_street
    Service,        ///< highway=service
    Footway,        ///< highway=footway, pedestrian
    Cycleway,       ///< highway=cycleway
    Path,           ///< highway=path, track
    Unknown         ///< Unclassified or unsupported road type
};

/**
 * @brief Classification of building types from building=* tags
 */
enum class BuildingType {
    Residential,    ///< Generic residential
    Commercial,     ///< Commercial/business
    Industrial,     ///< Industrial/manufacturing
    Retail,         ///< Retail/shop
    Office,         ///< Office building
    Apartments,     ///< Multi-family residential
    House,          ///< Single-family house
    Detached,       ///< Detached house
    Garage,         ///< Garage/carport
    Shed,           ///< Storage shed
    Church,         ///< Religious building
    School,         ///< Educational facility
    Hospital,       ///< Medical facility
    Warehouse,      ///< Storage/warehouse
    Unknown         ///< Unclassified building
};

/**
 * @brief Classification of area types from landuse=* and natural=* tags
 */
enum class AreaType {
    Water,          ///< natural=water, waterway
    Park,           ///< leisure=park
    Forest,         ///< natural=wood, landuse=forest
    Grass,          ///< landuse=grass, natural=grassland
    Parking,        ///< amenity=parking
    Commercial,     ///< landuse=commercial
    Residential,    ///< landuse=residential
    Industrial,     ///< landuse=industrial
    Farmland,       ///< landuse=farmland
    Cemetery,       ///< landuse=cemetery
    Unknown         ///< Unclassified area
};

/**
 * @brief Roof style classification from roof:shape=* tags
 */
enum class RoofType {
    Flat,           ///< Flat roof (default)
    Gabled,         ///< Gabled/pitched roof
    Hipped,         ///< Hipped roof
    Pyramidal,      ///< Pyramidal roof
    Skillion,       ///< Single-slope roof
    Dome,           ///< Dome roof
    Unknown         ///< Unclassified roof style
};

// ============================================================================
// Raw OSM Structures (intermediate representation)
// ============================================================================

/**
 * @brief Raw OSM node with geographic coordinates and tags
 */
struct OSMNode {
    NodeId id = 0;          ///< Unique node identifier
    double lat = 0.0;       ///< Latitude in WGS84 (degrees)
    double lon = 0.0;       ///< Longitude in WGS84 (degrees)
    TagMap tags;            ///< Key-value tags
};

/**
 * @brief Raw OSM way composed of ordered node references
 */
struct OSMWay {
    WayId id = 0;                       ///< Unique way identifier
    std::vector<NodeId> node_refs;      ///< Ordered list of node IDs
    TagMap tags;                        ///< Key-value tags

    /**
     * @brief Check if this way forms a closed polygon
     * @return true if first and last node are the same
     */
    [[nodiscard]] bool is_closed() const {
        return node_refs.size() > 2 && node_refs.front() == node_refs.back();
    }
};

/**
 * @brief Member reference within an OSM relation
 */
struct OSMMember {
    /// Type of the referenced element
    enum class Type { Node, Way, Relation };

    Type type = Type::Node;     ///< Element type
    int64_t ref = 0;            ///< Element ID
    std::string role;           ///< Role in relation (e.g., "outer", "inner")
};

/**
 * @brief Raw OSM relation composed of member references
 */
struct OSMRelation {
    RelationId id = 0;                  ///< Unique relation identifier
    std::vector<OSMMember> members;     ///< Member elements
    TagMap tags;                        ///< Key-value tags
};

// ============================================================================
// Processed Structures (ready for mesh generation)
// ============================================================================

/**
 * @brief Processed road ready for mesh generation
 *
 * Contains the road centerline as a polyline in local coordinates,
 * along with extracted metadata for rendering.
 */
struct Road {
    WayId osm_id = 0;                       ///< Original OSM way ID
    std::vector<glm::dvec2> polyline;       ///< Centerline in local meters
    RoadType type = RoadType::Unknown;      ///< Road classification
    float width = 6.0f;                     ///< Road width in meters
    int lanes = 2;                          ///< Number of lanes
    std::optional<float> speed_limit;       ///< Speed limit if known (km/h)
    std::string name;                       ///< Road name from name=* tag
    bool is_oneway = false;                 ///< One-way street
    bool is_bridge = false;                 ///< Bridge segment
    bool is_tunnel = false;                 ///< Tunnel segment
};

/**
 * @brief Processed building ready for mesh generation
 *
 * Contains the building footprint as a polygon in local coordinates,
 * with optional inner rings for courtyards/holes.
 */
struct Building {
    int64_t osm_id = 0;                         ///< Original OSM way/relation ID
    std::vector<glm::dvec2> footprint;          ///< Outer ring in local meters (CCW)
    std::vector<std::vector<glm::dvec2>> holes; ///< Inner rings (CW)
    float height = 10.0f;                       ///< Building height in meters
    int levels = 3;                             ///< Number of floors
    BuildingType type = BuildingType::Unknown;  ///< Building classification
    RoofType roof_type = RoofType::Flat;        ///< Roof style
    std::string name;                           ///< Building name if known
    std::optional<std::string> roof_color;      ///< Roof color (hex or name)
    std::optional<std::string> building_color;  ///< Facade color
};

/**
 * @brief Processed area (landuse, water, park) ready for mesh generation
 */
struct Area {
    int64_t osm_id = 0;                         ///< Original OSM way/relation ID
    std::vector<glm::dvec2> polygon;            ///< Outer ring in local meters
    std::vector<std::vector<glm::dvec2>> holes; ///< Inner rings
    AreaType type = AreaType::Unknown;          ///< Area classification
    std::string name;                           ///< Area name if known
};

// ============================================================================
// Geographic Bounds and Coordinate System
// ============================================================================

/**
 * @brief Geographic bounding box in WGS84 coordinates
 */
struct BoundingBox {
    double min_lat = 90.0;      ///< Southern boundary
    double max_lat = -90.0;     ///< Northern boundary
    double min_lon = 180.0;     ///< Western boundary
    double max_lon = -180.0;    ///< Eastern boundary

    /**
     * @brief Expand bounds to include a point
     * @param lat Latitude in degrees
     * @param lon Longitude in degrees
     */
    void expand(double lat, double lon) {
        min_lat = std::min(min_lat, lat);
        max_lat = std::max(max_lat, lat);
        min_lon = std::min(min_lon, lon);
        max_lon = std::max(max_lon, lon);
    }

    /**
     * @brief Get the center point of the bounding box
     * @return Center as (lat, lon)
     */
    [[nodiscard]] glm::dvec2 center() const {
        return glm::dvec2((min_lat + max_lat) / 2.0, (min_lon + max_lon) / 2.0);
    }

    /**
     * @brief Check if bounds have been initialized with valid data
     * @return true if bounds contain at least one point
     */
    [[nodiscard]] bool is_valid() const {
        return min_lat <= max_lat && min_lon <= max_lon;
    }

    /**
     * @brief Get approximate width in meters at center latitude
     */
    [[nodiscard]] double width_meters() const;

    /**
     * @brief Get approximate height in meters
     */
    [[nodiscard]] double height_meters() const;
};

/**
 * @brief Coordinate system information for local coordinate conversion
 */
struct CoordinateSystem {
    glm::dvec2 origin_latlon{0.0, 0.0};     ///< WGS84 origin (lat, lon)
    glm::dvec2 origin_mercator{0.0, 0.0};   ///< Web Mercator origin (x, y)
    double scale = 1.0;                      ///< Optional scale factor
};

// ============================================================================
// Parsed Data Container
// ============================================================================

/**
 * @brief Container for all parsed and processed OSM data
 */
struct ParsedOSMData {
    // Raw data (preserved for reference)
    std::unordered_map<NodeId, OSMNode> nodes;          ///< All parsed nodes
    std::unordered_map<WayId, OSMWay> ways;             ///< All parsed ways
    std::unordered_map<RelationId, OSMRelation> relations; ///< All parsed relations

    // Processed data (ready for mesh generation)
    std::vector<Road> roads;            ///< Processed road polylines
    std::vector<Building> buildings;    ///< Processed building footprints
    std::vector<Area> areas;            ///< Processed area polygons

    // Coordinate system info
    BoundingBox bounds;                 ///< Geographic extent of data
    CoordinateSystem coord_system;      ///< Local coordinate system

    /**
     * @brief Parse statistics for logging and debugging
     */
    struct Statistics {
        size_t total_nodes = 0;         ///< Raw node count
        size_t total_ways = 0;          ///< Raw way count
        size_t total_relations = 0;     ///< Raw relation count
        size_t processed_roads = 0;     ///< Processed road count
        size_t processed_buildings = 0; ///< Processed building count
        size_t processed_areas = 0;     ///< Processed area count
        double parse_time_ms = 0.0;     ///< Time to parse file
        double process_time_ms = 0.0;   ///< Time to process elements
    } stats;

    /**
     * @brief Clear all data
     */
    void clear() {
        nodes.clear();
        ways.clear();
        relations.clear();
        roads.clear();
        buildings.clear();
        areas.clear();
        bounds = BoundingBox{};
        coord_system = CoordinateSystem{};
        stats = Statistics{};
    }
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Convert RoadType to human-readable string
 */
[[nodiscard]] inline const char* road_type_name(RoadType type) {
    switch (type) {
        case RoadType::Motorway:    return "Motorway";
        case RoadType::Trunk:       return "Trunk";
        case RoadType::Primary:     return "Primary";
        case RoadType::Secondary:   return "Secondary";
        case RoadType::Tertiary:    return "Tertiary";
        case RoadType::Residential: return "Residential";
        case RoadType::Service:     return "Service";
        case RoadType::Footway:     return "Footway";
        case RoadType::Cycleway:    return "Cycleway";
        case RoadType::Path:        return "Path";
        case RoadType::Unknown:     return "Unknown";
    }
    return "Unknown";
}

/**
 * @brief Convert BuildingType to human-readable string
 */
[[nodiscard]] inline const char* building_type_name(BuildingType type) {
    switch (type) {
        case BuildingType::Residential: return "Residential";
        case BuildingType::Commercial:  return "Commercial";
        case BuildingType::Industrial:  return "Industrial";
        case BuildingType::Retail:      return "Retail";
        case BuildingType::Office:      return "Office";
        case BuildingType::Apartments:  return "Apartments";
        case BuildingType::House:       return "House";
        case BuildingType::Detached:    return "Detached";
        case BuildingType::Garage:      return "Garage";
        case BuildingType::Shed:        return "Shed";
        case BuildingType::Church:      return "Church";
        case BuildingType::School:      return "School";
        case BuildingType::Hospital:    return "Hospital";
        case BuildingType::Warehouse:   return "Warehouse";
        case BuildingType::Unknown:     return "Unknown";
    }
    return "Unknown";
}

/**
 * @brief Convert AreaType to human-readable string
 */
[[nodiscard]] inline const char* area_type_name(AreaType type) {
    switch (type) {
        case AreaType::Water:       return "Water";
        case AreaType::Park:        return "Park";
        case AreaType::Forest:      return "Forest";
        case AreaType::Grass:       return "Grass";
        case AreaType::Parking:     return "Parking";
        case AreaType::Commercial:  return "Commercial";
        case AreaType::Residential: return "Residential";
        case AreaType::Industrial:  return "Industrial";
        case AreaType::Farmland:    return "Farmland";
        case AreaType::Cemetery:    return "Cemetery";
        case AreaType::Unknown:     return "Unknown";
    }
    return "Unknown";
}

} // namespace stratum::osm
