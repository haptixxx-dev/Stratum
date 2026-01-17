/**
 * @file parser.cpp
 * @brief Implementation of OSM parser using libosmium
 */

#include "osm/parser.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <algorithm>

// libosmium includes
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>

namespace stratum::osm {

// ============================================================================
// Type aliases for libosmium
// ============================================================================

using LocationIndex = osmium::index::map::FlexMem<
    osmium::unsigned_object_id_type,
    osmium::Location
>;

using LocationHandler = osmium::handler::NodeLocationsForWays<LocationIndex>;

// ============================================================================
// Internal Handler
// ============================================================================

/**
 * @brief Internal handler for processing OSM data
 */
class DataHandler : public osmium::handler::Handler {
public:
    DataHandler(ParsedOSMData& data, const ParserConfig& config)
        : m_data(data), m_config(config) {}

    void node(const osmium::Node& node) {
        // Store node for coordinate lookup
        OSMNode osm_node;
        osm_node.id = node.id();
        osm_node.lat = node.location().lat();
        osm_node.lon = node.location().lon();

        // Copy tags
        for (const auto& tag : node.tags()) {
            osm_node.tags[tag.key()] = tag.value();
        }

        m_data.nodes[osm_node.id] = std::move(osm_node);
        m_data.bounds.expand(node.location().lat(), node.location().lon());
        m_data.stats.total_nodes++;
    }

    void way(const osmium::Way& way) {
        OSMWay osm_way;
        osm_way.id = way.id();

        // Store node references and their locations
        for (const auto& node_ref : way.nodes()) {
            osm_way.node_refs.push_back(node_ref.ref());

            // If this node has a valid location (from LocationHandler),
            // update our node entry or create a minimal one
            if (node_ref.location().valid()) {
                auto it = m_data.nodes.find(node_ref.ref());
                if (it == m_data.nodes.end()) {
                    // Node wasn't stored (perhaps outside filter), add minimal entry
                    OSMNode osm_node;
                    osm_node.id = node_ref.ref();
                    osm_node.lat = node_ref.location().lat();
                    osm_node.lon = node_ref.location().lon();
                    m_data.nodes[osm_node.id] = osm_node;
                    m_data.bounds.expand(osm_node.lat, osm_node.lon);
                }
            }
        }

        // Copy tags
        for (const auto& tag : way.tags()) {
            osm_way.tags[tag.key()] = tag.value();
        }

        m_data.ways[osm_way.id] = std::move(osm_way);
        m_data.stats.total_ways++;
    }

    void relation(const osmium::Relation& relation) {
        OSMRelation osm_rel;
        osm_rel.id = relation.id();

        // Store members
        for (const auto& member : relation.members()) {
            OSMMember osm_member;
            switch (member.type()) {
                case osmium::item_type::node:
                    osm_member.type = OSMMember::Type::Node;
                    break;
                case osmium::item_type::way:
                    osm_member.type = OSMMember::Type::Way;
                    break;
                case osmium::item_type::relation:
                    osm_member.type = OSMMember::Type::Relation;
                    break;
                default:
                    continue;
            }
            osm_member.ref = member.ref();
            osm_member.role = member.role();
            osm_rel.members.push_back(std::move(osm_member));
        }

        // Copy tags
        for (const auto& tag : relation.tags()) {
            osm_rel.tags[tag.key()] = tag.value();
        }

        m_data.relations[osm_rel.id] = std::move(osm_rel);
        m_data.stats.total_relations++;
    }

private:
    ParsedOSMData& m_data;
    const ParserConfig& m_config;
};

// ============================================================================
// OSMParser Implementation
// ============================================================================

OSMParser::OSMParser() = default;
OSMParser::~OSMParser() = default;
OSMParser::OSMParser(OSMParser&&) noexcept = default;
OSMParser& OSMParser::operator=(OSMParser&&) noexcept = default;

bool OSMParser::parse(const std::filesystem::path& filepath) {
    using Clock = std::chrono::high_resolution_clock;

    // Clear any existing data
    clear();

    auto parse_start = Clock::now();

    try {
        report_progress(ParseProgress::Stage::ReadingFile,
                       "Opening " + filepath.filename().string());

        // Check file exists
        if (!std::filesystem::exists(filepath)) {
            m_error = "File not found: " + filepath.string();
            spdlog::error("OSM Parser: {}", m_error);
            return false;
        }

        // Open the file (libosmium auto-detects format from extension)
        const osmium::io::File input_file{filepath.string()};

        // Log detected format
        std::string format_name;
        switch (input_file.format()) {
            case osmium::io::file_format::xml:  format_name = "XML"; break;
            case osmium::io::file_format::pbf:  format_name = "PBF"; break;
            case osmium::io::file_format::opl:  format_name = "OPL"; break;
            case osmium::io::file_format::json: format_name = "JSON"; break;
            case osmium::io::file_format::debug: format_name = "Debug"; break;
            default: format_name = "Unknown"; break;
        }
        spdlog::info("OSM Parser: Detected {} format for {}", format_name, filepath.filename().string());

        osmium::io::Reader reader{input_file,
            osmium::osm_entity_bits::node |
            osmium::osm_entity_bits::way |
            osmium::osm_entity_bits::relation
        };

        report_progress(ParseProgress::Stage::ParsingNodes, "Parsing OSM data...");

        // Create location index and handler
        LocationIndex index;
        LocationHandler location_handler{index};
        location_handler.ignore_errors();  // Don't fail on missing nodes

        // Create our data handler
        DataHandler data_handler(m_data, m_config);

        // Apply handlers to data
        osmium::apply(reader, location_handler, data_handler);

        reader.close();

        auto parse_end = Clock::now();
        m_data.stats.parse_time_ms = std::chrono::duration<double, std::milli>(
            parse_end - parse_start).count();

        spdlog::info("OSM Parser: Read {} nodes, {} ways, {} relations in {:.1f}ms",
                    m_data.stats.total_nodes,
                    m_data.stats.total_ways,
                    m_data.stats.total_relations,
                    m_data.stats.parse_time_ms);

        // Now process the raw data into our structures
        auto process_start = Clock::now();

        report_progress(ParseProgress::Stage::ConvertingCoords, "Converting coordinates...");
        convert_coordinates();

        report_progress(ParseProgress::Stage::ProcessingRoads, "Processing roads...");
        process_roads();

        report_progress(ParseProgress::Stage::ProcessingBuildings, "Processing buildings...");
        process_buildings();

        report_progress(ParseProgress::Stage::ProcessingAreas, "Processing areas...");
        process_areas();

        auto process_end = Clock::now();
        m_data.stats.process_time_ms = std::chrono::duration<double, std::milli>(
            process_end - process_start).count();

        m_data.stats.processed_roads = m_data.roads.size();
        m_data.stats.processed_buildings = m_data.buildings.size();
        m_data.stats.processed_areas = m_data.areas.size();

        report_progress(ParseProgress::Stage::Complete, "Parsing complete");

        m_has_data = true;
        return true;

    } catch (const std::exception& e) {
        m_error = e.what();
        spdlog::error("OSM Parser error: {}", m_error);
        return false;
    }
}

void OSMParser::convert_coordinates() {
    // Set up coordinate converter with bounds center
    if (m_data.bounds.is_valid()) {
        m_converter.set_origin(m_data.bounds);
        m_data.coord_system = m_converter.get_coord_system();

        spdlog::debug("OSM Parser: Set origin at ({:.4f}, {:.4f})",
                     m_data.coord_system.origin_latlon.x,
                     m_data.coord_system.origin_latlon.y);
    }
}

void OSMParser::process_roads() {
    if (!m_config.import_roads) return;

    for (const auto& [way_id, way] : m_data.ways) {
        // Check for highway tag
        auto highway_it = way.tags.find("highway");
        if (highway_it == way.tags.end()) continue;

        // Classify road type
        RoadType road_type = classify_road(way.tags);

        // Skip unknown types that we don't want
        if (road_type == RoadType::Unknown) continue;

        // Resolve coordinates
        auto coords = resolve_way_coords(way);
        if (coords.size() < 2) continue;

        // Create road
        Road road;
        road.osm_id = way_id;
        road.polyline = std::move(coords);
        road.type = road_type;
        road.width = estimate_road_width(road_type, way.tags);
        road.lanes = estimate_road_lanes(road_type, way.tags);

        // Extract name
        auto name_it = way.tags.find("name");
        if (name_it != way.tags.end()) {
            road.name = name_it->second;
        }

        // Check for one-way
        auto oneway_it = way.tags.find("oneway");
        if (oneway_it != way.tags.end()) {
            road.is_oneway = (oneway_it->second == "yes" || oneway_it->second == "1");
        }

        // Check for bridge/tunnel
        road.is_bridge = way.tags.count("bridge") > 0 && way.tags.at("bridge") != "no";
        road.is_tunnel = way.tags.count("tunnel") > 0 && way.tags.at("tunnel") != "no";

        // Speed limit
        auto maxspeed_it = way.tags.find("maxspeed");
        if (maxspeed_it != way.tags.end()) {
            try {
                road.speed_limit = std::stof(maxspeed_it->second);
            } catch (...) {}
        }

        // Optional simplification
        if (m_config.simplify_geometry && road.polyline.size() > 2) {
            road.polyline = geometry::simplify(road.polyline, m_config.simplify_tolerance);
        }

        m_data.roads.push_back(std::move(road));
    }

    spdlog::info("OSM Parser: Processed {} roads", m_data.roads.size());
}

void OSMParser::process_buildings() {
    if (!m_config.import_buildings) return;

    for (const auto& [way_id, way] : m_data.ways) {
        // Check for building tag
        auto building_it = way.tags.find("building");
        if (building_it == way.tags.end()) continue;

        // Must be a closed way
        if (!way.is_closed()) continue;

        // Resolve coordinates
        auto coords = resolve_way_coords(way);
        if (coords.size() < 4) continue;  // Need at least 3 unique points + closing

        // Create building
        Building building;
        building.osm_id = way_id;
        building.footprint = std::move(coords);
        building.type = classify_building(way.tags);
        building.roof_type = classify_roof(way.tags);
        building.height = estimate_building_height(way.tags);

        // Extract levels
        auto levels_it = way.tags.find("building:levels");
        if (levels_it != way.tags.end()) {
            try {
                building.levels = std::stoi(levels_it->second);
            } catch (...) {
                building.levels = static_cast<int>(building.height / m_config.meters_per_level);
            }
        } else {
            building.levels = static_cast<int>(building.height / m_config.meters_per_level);
        }

        // Extract name
        auto name_it = way.tags.find("name");
        if (name_it != way.tags.end()) {
            building.name = name_it->second;
        }

        // Extract colors
        auto roof_color_it = way.tags.find("roof:colour");
        if (roof_color_it != way.tags.end()) {
            building.roof_color = roof_color_it->second;
        }

        auto building_color_it = way.tags.find("building:colour");
        if (building_color_it != way.tags.end()) {
            building.building_color = building_color_it->second;
        }

        // Ensure counter-clockwise winding for outer ring
        geometry::ensure_ccw(building.footprint);

        m_data.buildings.push_back(std::move(building));
    }

    spdlog::info("OSM Parser: Processed {} buildings", m_data.buildings.size());
}

void OSMParser::process_areas() {
    if (!m_config.import_landuse && !m_config.import_water && !m_config.import_natural) {
        return;
    }

    for (const auto& [way_id, way] : m_data.ways) {
        // Skip if already processed as building
        if (way.tags.count("building") > 0) continue;

        // Check for area-related tags
        AreaType area_type = classify_area(way.tags);
        if (area_type == AreaType::Unknown) continue;

        // Filter by config
        bool should_import = false;
        switch (area_type) {
            case AreaType::Water:
                should_import = m_config.import_water;
                break;
            case AreaType::Park:
            case AreaType::Forest:
            case AreaType::Grass:
                should_import = m_config.import_natural;
                break;
            default:
                should_import = m_config.import_landuse;
                break;
        }

        if (!should_import) continue;

        // Must be a closed way for areas
        if (!way.is_closed()) continue;

        // Resolve coordinates
        auto coords = resolve_way_coords(way);
        if (coords.size() < 4) continue;

        // Create area
        Area area;
        area.osm_id = way_id;
        area.polygon = std::move(coords);
        area.type = area_type;

        // Extract name
        auto name_it = way.tags.find("name");
        if (name_it != way.tags.end()) {
            area.name = name_it->second;
        }

        // Ensure counter-clockwise winding
        geometry::ensure_ccw(area.polygon);

        m_data.areas.push_back(std::move(area));
    }

    spdlog::info("OSM Parser: Processed {} areas", m_data.areas.size());
}

std::vector<glm::dvec2> OSMParser::resolve_way_coords(const OSMWay& way) const {
    std::vector<glm::dvec2> coords;
    coords.reserve(way.node_refs.size());

    for (NodeId node_id : way.node_refs) {
        auto it = m_data.nodes.find(node_id);
        if (it != m_data.nodes.end()) {
            const auto& node = it->second;
            glm::dvec2 local = m_converter.wgs84_to_local(node.lat, node.lon);
            coords.push_back(local);
        }
    }

    return coords;
}

// ============================================================================
// Classification Functions
// ============================================================================

RoadType OSMParser::classify_road(const TagMap& tags) {
    auto it = tags.find("highway");
    if (it == tags.end()) return RoadType::Unknown;

    const std::string& value = it->second;

    if (value == "motorway" || value == "motorway_link")
        return RoadType::Motorway;
    if (value == "trunk" || value == "trunk_link")
        return RoadType::Trunk;
    if (value == "primary" || value == "primary_link")
        return RoadType::Primary;
    if (value == "secondary" || value == "secondary_link")
        return RoadType::Secondary;
    if (value == "tertiary" || value == "tertiary_link")
        return RoadType::Tertiary;
    if (value == "residential" || value == "living_street")
        return RoadType::Residential;
    if (value == "service")
        return RoadType::Service;
    if (value == "footway" || value == "pedestrian" || value == "steps")
        return RoadType::Footway;
    if (value == "cycleway")
        return RoadType::Cycleway;
    if (value == "path" || value == "track" || value == "bridleway")
        return RoadType::Path;
    if (value == "unclassified")
        return RoadType::Residential;  // Treat as residential

    return RoadType::Unknown;
}

BuildingType OSMParser::classify_building(const TagMap& tags) {
    auto it = tags.find("building");
    if (it == tags.end()) return BuildingType::Unknown;

    const std::string& value = it->second;

    if (value == "residential")
        return BuildingType::Residential;
    if (value == "commercial")
        return BuildingType::Commercial;
    if (value == "industrial")
        return BuildingType::Industrial;
    if (value == "retail")
        return BuildingType::Retail;
    if (value == "office")
        return BuildingType::Office;
    if (value == "apartments" || value == "dormitory")
        return BuildingType::Apartments;
    if (value == "house")
        return BuildingType::House;
    if (value == "detached")
        return BuildingType::Detached;
    if (value == "garage" || value == "carport")
        return BuildingType::Garage;
    if (value == "shed" || value == "hut")
        return BuildingType::Shed;
    if (value == "church" || value == "cathedral" || value == "chapel" ||
        value == "mosque" || value == "temple" || value == "synagogue")
        return BuildingType::Church;
    if (value == "school" || value == "university" || value == "college")
        return BuildingType::School;
    if (value == "hospital" || value == "clinic")
        return BuildingType::Hospital;
    if (value == "warehouse")
        return BuildingType::Warehouse;
    if (value == "yes")
        return BuildingType::Unknown;  // Generic building

    return BuildingType::Unknown;
}

AreaType OSMParser::classify_area(const TagMap& tags) {
    // Check natural tag
    auto natural_it = tags.find("natural");
    if (natural_it != tags.end()) {
        const std::string& value = natural_it->second;
        if (value == "water" || value == "bay" || value == "coastline")
            return AreaType::Water;
        if (value == "wood" || value == "tree_row")
            return AreaType::Forest;
        if (value == "grassland" || value == "scrub" || value == "heath")
            return AreaType::Grass;
    }

    // Check waterway tag
    auto waterway_it = tags.find("waterway");
    if (waterway_it != tags.end()) {
        const std::string& value = waterway_it->second;
        if (value == "riverbank" || value == "dock" || value == "boatyard")
            return AreaType::Water;
    }

    // Check leisure tag
    auto leisure_it = tags.find("leisure");
    if (leisure_it != tags.end()) {
        const std::string& value = leisure_it->second;
        if (value == "park" || value == "garden" || value == "playground" ||
            value == "nature_reserve")
            return AreaType::Park;
    }

    // Check landuse tag
    auto landuse_it = tags.find("landuse");
    if (landuse_it != tags.end()) {
        const std::string& value = landuse_it->second;
        if (value == "residential")
            return AreaType::Residential;
        if (value == "commercial" || value == "retail")
            return AreaType::Commercial;
        if (value == "industrial")
            return AreaType::Industrial;
        if (value == "forest")
            return AreaType::Forest;
        if (value == "grass" || value == "meadow" || value == "village_green" ||
            value == "recreation_ground")
            return AreaType::Grass;
        if (value == "farmland" || value == "farmyard" || value == "orchard" ||
            value == "vineyard")
            return AreaType::Farmland;
        if (value == "cemetery")
            return AreaType::Cemetery;
        if (value == "basin" || value == "reservoir")
            return AreaType::Water;
    }

    // Check amenity tag for parking
    auto amenity_it = tags.find("amenity");
    if (amenity_it != tags.end()) {
        if (amenity_it->second == "parking")
            return AreaType::Parking;
    }

    return AreaType::Unknown;
}

RoofType OSMParser::classify_roof(const TagMap& tags) {
    auto it = tags.find("roof:shape");
    if (it == tags.end()) {
        // Also check building:roof:shape
        it = tags.find("building:roof:shape");
        if (it == tags.end()) return RoofType::Flat;
    }

    const std::string& value = it->second;

    if (value == "flat")
        return RoofType::Flat;
    if (value == "gabled" || value == "half-hipped" || value == "saltbox")
        return RoofType::Gabled;
    if (value == "hipped" || value == "hip")
        return RoofType::Hipped;
    if (value == "pyramidal")
        return RoofType::Pyramidal;
    if (value == "skillion" || value == "lean_to")
        return RoofType::Skillion;
    if (value == "dome" || value == "onion")
        return RoofType::Dome;

    return RoofType::Unknown;
}

// ============================================================================
// Estimation Functions
// ============================================================================

float OSMParser::estimate_building_height(const TagMap& tags) const {
    // Priority 1: Explicit height tag
    auto height_it = tags.find("height");
    if (height_it != tags.end()) {
        try {
            std::string h = height_it->second;
            // Remove 'm' suffix if present
            if (!h.empty() && (h.back() == 'm' || h.back() == 'M')) {
                h.pop_back();
            }
            // Also handle "X m" format
            size_t space_pos = h.find(' ');
            if (space_pos != std::string::npos) {
                h = h.substr(0, space_pos);
            }
            return std::stof(h);
        } catch (...) {}
    }

    // Priority 2: building:levels tag
    auto levels_it = tags.find("building:levels");
    if (levels_it != tags.end()) {
        try {
            int levels = std::stoi(levels_it->second);
            return static_cast<float>(levels) * m_config.meters_per_level;
        } catch (...) {}
    }

    // Priority 3: Estimate by building type
    auto building_it = tags.find("building");
    if (building_it != tags.end()) {
        const std::string& type = building_it->second;
        if (type == "garage" || type == "shed" || type == "hut" || type == "carport")
            return 3.0f;
        if (type == "house" || type == "detached" || type == "bungalow")
            return 8.0f;
        if (type == "apartments" || type == "dormitory")
            return 15.0f;
        if (type == "commercial" || type == "office")
            return 20.0f;
        if (type == "industrial" || type == "warehouse")
            return 12.0f;
        if (type == "church" || type == "cathedral")
            return 25.0f;
        if (type == "hospital")
            return 18.0f;
        if (type == "school" || type == "university")
            return 12.0f;
    }

    return m_config.default_building_height;
}

float OSMParser::estimate_road_width(RoadType type, const TagMap& tags) {
    // Check for explicit width tag
    auto width_it = tags.find("width");
    if (width_it != tags.end()) {
        try {
            std::string w = width_it->second;
            // Remove 'm' suffix
            if (!w.empty() && (w.back() == 'm' || w.back() == 'M')) {
                w.pop_back();
            }
            return std::stof(w);
        } catch (...) {}
    }

    // Check lanes tag
    auto lanes_it = tags.find("lanes");
    if (lanes_it != tags.end()) {
        try {
            int lanes = std::stoi(lanes_it->second);
            return static_cast<float>(lanes) * 3.5f;  // 3.5m per lane
        } catch (...) {}
    }

    // Default widths by type
    switch (type) {
        case RoadType::Motorway:    return 14.0f;
        case RoadType::Trunk:       return 10.5f;
        case RoadType::Primary:     return 10.0f;
        case RoadType::Secondary:   return 8.0f;
        case RoadType::Tertiary:    return 7.0f;
        case RoadType::Residential: return 6.0f;
        case RoadType::Service:     return 4.0f;
        case RoadType::Footway:     return 2.0f;
        case RoadType::Cycleway:    return 2.5f;
        case RoadType::Path:        return 1.5f;
        default:                    return 6.0f;
    }
}

int OSMParser::estimate_road_lanes(RoadType type, const TagMap& tags) {
    // Check lanes tag
    auto lanes_it = tags.find("lanes");
    if (lanes_it != tags.end()) {
        try {
            return std::stoi(lanes_it->second);
        } catch (...) {}
    }

    // Default lanes by type
    switch (type) {
        case RoadType::Motorway:    return 4;
        case RoadType::Trunk:       return 3;
        case RoadType::Primary:     return 2;
        case RoadType::Secondary:   return 2;
        case RoadType::Tertiary:    return 2;
        case RoadType::Residential: return 2;
        case RoadType::Service:     return 1;
        case RoadType::Footway:     return 1;
        case RoadType::Cycleway:    return 1;
        case RoadType::Path:        return 1;
        default:                    return 2;
    }
}

// ============================================================================
// Data Access
// ============================================================================

ParsedOSMData OSMParser::take_data() {
    m_has_data = false;
    return std::move(m_data);
}

void OSMParser::clear() {
    m_data.clear();
    m_error.clear();
    m_has_data = false;
}

// ============================================================================
// Progress Reporting
// ============================================================================

void OSMParser::report_progress(ParseProgress::Stage stage, const std::string& message,
                                size_t current, size_t total) {
    if (m_progress_callback) {
        ParseProgress progress;
        progress.stage = stage;
        progress.current = current;
        progress.total = total;
        progress.message = message;
        m_progress_callback(progress);
    }
}

// ============================================================================
// Logging
// ============================================================================

void OSMParser::log_statistics() const {
    spdlog::info("=== OSM Parse Statistics ===");
    spdlog::info("Raw data:");
    spdlog::info("  Nodes: {}", m_data.stats.total_nodes);
    spdlog::info("  Ways: {}", m_data.stats.total_ways);
    spdlog::info("  Relations: {}", m_data.stats.total_relations);
    spdlog::info("Processed:");
    spdlog::info("  Roads: {}", m_data.roads.size());
    spdlog::info("  Buildings: {}", m_data.buildings.size());
    spdlog::info("  Areas: {}", m_data.areas.size());

    if (m_data.bounds.is_valid()) {
        spdlog::info("Bounds:");
        spdlog::info("  Lat: [{:.4f}, {:.4f}]", m_data.bounds.min_lat, m_data.bounds.max_lat);
        spdlog::info("  Lon: [{:.4f}, {:.4f}]", m_data.bounds.min_lon, m_data.bounds.max_lon);
        spdlog::info("  Size: ~{:.0f}m x {:.0f}m",
                    m_data.bounds.width_meters(), m_data.bounds.height_meters());
    }

    spdlog::info("Timing:");
    spdlog::info("  Parse time: {:.1f}ms", m_data.stats.parse_time_ms);
    spdlog::info("  Process time: {:.1f}ms", m_data.stats.process_time_ms);
}

void OSMParser::log_sample_data(size_t count) const {
    // Log sample roads
    spdlog::info("--- Sample Roads ({} of {}) ---",
                std::min(count, m_data.roads.size()), m_data.roads.size());
    for (size_t i = 0; i < std::min(count, m_data.roads.size()); ++i) {
        const auto& road = m_data.roads[i];
        spdlog::info("  Road {}: '{}' (type={}, width={:.1f}m, {} points)",
                    road.osm_id,
                    road.name.empty() ? "(unnamed)" : road.name,
                    road_type_name(road.type),
                    road.width,
                    road.polyline.size());
    }

    // Log sample buildings
    spdlog::info("--- Sample Buildings ({} of {}) ---",
                std::min(count, m_data.buildings.size()), m_data.buildings.size());
    for (size_t i = 0; i < std::min(count, m_data.buildings.size()); ++i) {
        const auto& bldg = m_data.buildings[i];
        spdlog::info("  Building {}: type={}, height={:.1f}m, {} vertices, {} holes",
                    bldg.osm_id,
                    building_type_name(bldg.type),
                    bldg.height,
                    bldg.footprint.size(),
                    bldg.holes.size());
    }

    // Log sample areas
    spdlog::info("--- Sample Areas ({} of {}) ---",
                std::min(count, m_data.areas.size()), m_data.areas.size());
    for (size_t i = 0; i < std::min(count, m_data.areas.size()); ++i) {
        const auto& area = m_data.areas[i];
        spdlog::info("  Area {}: '{}' (type={}, {} vertices)",
                    area.osm_id,
                    area.name.empty() ? "(unnamed)" : area.name,
                    area_type_name(area.type),
                    area.polygon.size());
    }
}

} // namespace stratum::osm
