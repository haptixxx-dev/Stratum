/**
 * @file parser.cpp
 * @brief Implementation of OSM parser using libosmium
 */

#include "osm/parser.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <vector>

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

        // Everything past the file read is a fixed sequence, so report it as
        // step N of PROCESS_STEPS. That gives callers a determinate progress bar
        // for this half; the streaming read above has no known total, and reports
        // current=0 so a UI can show an indeterminate bar instead of a stuck 0%.
        constexpr size_t PROCESS_STEPS = 5;

        report_progress(ParseProgress::Stage::ConvertingCoords, "Converting coordinates...",
                        1, PROCESS_STEPS);
        convert_coordinates();

        report_progress(ParseProgress::Stage::ProcessingRoads, "Processing roads...",
                        2, PROCESS_STEPS);
        process_roads();

        report_progress(ParseProgress::Stage::ProcessingBuildings, "Processing buildings...",
                        3, PROCESS_STEPS);
        process_buildings();

        report_progress(ParseProgress::Stage::ProcessingAreas, "Processing areas...",
                        4, PROCESS_STEPS);
        process_areas();

        // Must run after all three process_* passes, since it moves the geometry
        // they produced.
        recenter_on_features();

        auto process_end = Clock::now();
        m_data.stats.process_time_ms = std::chrono::duration<double, std::milli>(
            process_end - process_start).count();

        m_data.stats.processed_roads = m_data.roads.size();
        m_data.stats.processed_buildings = m_data.buildings.size();
        m_data.stats.processed_areas = m_data.areas.size();

        report_progress(ParseProgress::Stage::Complete, "Parsing complete", 5, 5);

        m_has_data = true;
        return true;

    } catch (const std::exception& e) {
        m_error = e.what();
        spdlog::error("OSM Parser error: {}", m_error);
        return false;
    }
}

void OSMParser::recenter_on_features() {
    // Mean over feature vertices, not the bounding-box centre: the mean is
    // dominated by the bulk of the geometry, so a handful of distant strays
    // cannot drag the origin away from where the data actually is.
    glm::dvec2 sum(0.0);
    size_t count = 0;

    const auto accumulate = [&](const std::vector<glm::dvec2>& pts) {
        for (const auto& p : pts) { sum += p; ++count; }
    };
    for (const auto& r : m_data.roads) accumulate(r.polyline);
    for (const auto& b : m_data.buildings) {
        accumulate(b.footprint);
        for (const auto& h : b.holes) accumulate(h);
    }
    for (const auto& a : m_data.areas) {
        accumulate(a.polygon);
        for (const auto& h : a.holes) accumulate(h);
    }

    if (count == 0) return;

    const glm::dvec2 centre = sum / static_cast<double>(count);
    if (glm::length(centre) < 1.0) return;  // already centred, nothing to gain

    const auto shift = [&](std::vector<glm::dvec2>& pts) {
        for (auto& p : pts) p -= centre;
    };
    for (auto& r : m_data.roads) shift(r.polyline);
    for (auto& b : m_data.buildings) {
        shift(b.footprint);
        for (auto& h : b.holes) shift(h);
    }
    for (auto& a : m_data.areas) {
        shift(a.polygon);
        for (auto& h : a.holes) shift(h);
    }

    // local = mercator - origin_mercator, so moving local by -centre is exactly
    // moving the origin by +centre.
    const glm::dvec2 new_origin_mercator =
        m_converter.get_coord_system().origin_mercator + centre;
    // Returns (lat, lon), which is the order set_origin takes.
    const glm::dvec2 new_origin_latlon =
        CoordinateConverter::mercator_to_wgs84(new_origin_mercator.x, new_origin_mercator.y);

    m_converter.set_origin(new_origin_latlon.x, new_origin_latlon.y);
    m_data.coord_system = m_converter.get_coord_system();

    spdlog::info("OSM Parser: recentred origin by ({:.0f}, {:.0f})m to ({:.5f}, {:.5f})",
                 centre.x, centre.y, new_origin_latlon.x, new_origin_latlon.y);
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

// ============================================================================
// Tag Parsing Helpers
// ============================================================================

namespace {

/**
 * @brief Look up a tag by key
 * @param tags Tag map to search
 * @param key Tag key
 * @return Pointer to the raw value, or nullptr when the tag is absent
 */
[[nodiscard]] const std::string* find_tag(const TagMap& tags, const std::string& key) {
    auto it = tags.find(key);
    return it == tags.end() ? nullptr : &it->second;
}

/**
 * @brief Lowercase and whitespace-trim a tag value
 *
 * OSM values are free text typed by mappers, so "Yes", " lane" and "M" all
 * occur. Comparing raw values against lowercase literals loses real data.
 *
 * @param value Raw tag value
 * @return Normalized copy
 */
[[nodiscard]] std::string normalize_value(std::string_view value) {
    size_t begin = 0;
    size_t end = value.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(value[begin]))) ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) --end;

    std::string out;
    out.reserve(end - begin);
    for (size_t i = begin; i < end; ++i) {
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(value[i]))));
    }
    return out;
}

/**
 * @brief Normalized value of a tag
 * @param tags Tag map to search
 * @param key Tag key
 * @return Lowercased, trimmed value, or an empty string when the tag is absent
 */
[[nodiscard]] std::string tag_value(const TagMap& tags, const std::string& key) {
    const std::string* raw = find_tag(tags, key);
    return raw ? normalize_value(*raw) : std::string{};
}

/**
 * @brief Read a tag as an integer, tolerating junk
 *
 * Values such as "1;2" or "two" are common. They must not throw and must not
 * abort the import, so an unreadable value reads as absent.
 *
 * @param tags Tag map to search
 * @param key Tag key
 * @return The integer, or std::nullopt when the tag is absent or unreadable
 */
[[nodiscard]] std::optional<int> parse_tag_int(const TagMap& tags, const std::string& key) {
    const std::string* raw = find_tag(tags, key);
    if (!raw) return std::nullopt;
    try {
        size_t consumed = 0;
        const int value = std::stoi(*raw, &consumed);
        if (consumed == 0) return std::nullopt;
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

/**
 * @brief Read a lane count tag
 * @param tags Tag map to search
 * @param key Tag key, for example "lanes" or "lanes:forward"
 * @return Lane count, or -1 when the tag is absent, unreadable, or not positive
 */
[[nodiscard]] int parse_lane_count(const TagMap& tags, const std::string& key) {
    const auto value = parse_tag_int(tags, key);
    if (!value || *value <= 0) return -1;
    return *value;
}

/**
 * @brief Total lanes implied by lanes:forward and lanes:backward
 *
 * A one-way slip road often carries only lanes:forward, so a missing side
 * counts as zero rather than voiding the total.
 *
 * @param tags Tag map to search
 * @return Combined lane count, or -1 when neither tag is usable
 */
[[nodiscard]] int directional_lane_total(const TagMap& tags) {
    const int forward = parse_lane_count(tags, "lanes:forward");
    const int backward = parse_lane_count(tags, "lanes:backward");
    if (forward < 0 && backward < 0) return -1;
    return std::max(forward, 0) + std::max(backward, 0);
}

/**
 * @brief Read a length value that may carry a unit suffix
 *
 * Accepts a bare number in metres ("12"), an explicit metre suffix ("12 m",
 * "12 metres"), feet ("30 ft", "30 feet"), and the feet-and-inches form
 * ("30'6\""). An unrecognised suffix is read as metres, which is what a bare
 * std::stof did before.
 *
 * @param raw Raw tag value
 * @return Length in metres, or std::nullopt when no number could be read
 */
[[nodiscard]] std::optional<float> parse_length_meters(std::string_view raw) {
    constexpr double kFeetToMeters = 0.3048;
    constexpr double kInchesToMeters = 0.0254;

    const std::string value = normalize_value(raw);
    if (value.empty()) return std::nullopt;

    const char* begin = value.c_str();
    char* end = nullptr;
    const double number = std::strtod(begin, &end);
    if (end == begin || !std::isfinite(number)) return std::nullopt;

    const std::string suffix = normalize_value(std::string_view(end));

    if (suffix.empty() || suffix == "m" || suffix == "meter" || suffix == "meters" ||
        suffix == "metre" || suffix == "metres") {
        return static_cast<float>(number);
    }
    if (suffix == "ft" || suffix == "feet" || suffix == "foot") {
        return static_cast<float>(number * kFeetToMeters);
    }
    if (suffix.front() == '\'') {
        // Feet and inches, "30'6\"". Inches are optional.
        const char* inch_begin = suffix.c_str() + 1;
        char* inch_end = nullptr;
        const double inches = std::strtod(inch_begin, &inch_end);
        const double extra = (inch_end != inch_begin && std::isfinite(inches)) ? inches : 0.0;
        return static_cast<float>(number * kFeetToMeters + extra * kInchesToMeters);
    }

    // Unrecognised unit. Read as metres so the value is not thrown away.
    return static_cast<float>(number);
}

/**
 * @brief What one tag value says about a road feature
 */
enum class TagPresence : uint8_t {
    Absent,     ///< Value not understood; treat the tag as if it were not there
    Negative,   ///< Value denies the feature (no, none, separate)
    Positive    ///< Value asserts the feature
};

/// Reads one value of a sided feature, ignoring which side it came from
using ValueClassifier = TagPresence (*)(const std::string&);

/**
 * @brief Read the left/right/both tag family of one road feature
 *
 * OSM spells these two ways and both appear in the same extract. The classic
 * form puts the side in the value (sidewalk=both, shoulder=left). The newer
 * form puts the side in the key (sidewalk:left=yes, cycleway:both=lane,
 * parking:right=lane).
 *
 * @param tags      Way tags
 * @param base_keys Keys read on their own, whose value may name a side
 * @param side_keys Keys read with a ":both", ":left", and ":right" suffix
 * @param classify  Feature-specific reading of a value that does not name a side
 * @return Sides carrying the feature, None when a tag denied it on every side,
 *         Unknown when no tag of the family was present
 */
[[nodiscard]] SideFlags read_sided_tags(const TagMap& tags,
                                        std::initializer_list<const char*> base_keys,
                                        std::initializer_list<const char*> side_keys,
                                        ValueClassifier classify) {
    struct SideSuffix {
        const char* suffix;     ///< Key suffix, including the colon
        bool left;              ///< Suffix covers the left side
        bool right;             ///< Suffix covers the right side
    };
    static constexpr SideSuffix kSideSuffixes[] = {
        {":both",  true,  true},
        {":left",  true,  false},
        {":right", false, true},
    };

    bool tagged = false;
    bool left = false;
    bool right = false;

    for (const char* key : base_keys) {
        const std::string* raw = find_tag(tags, key);
        if (!raw) continue;

        const std::string value = normalize_value(*raw);
        if (value == "both" || value == "left" || value == "right") {
            tagged = true;
            left = left || value != "right";
            right = right || value != "left";
            continue;
        }

        switch (classify(value)) {
            case TagPresence::Positive:
                tagged = true;
                left = true;
                right = true;
                break;
            case TagPresence::Negative:
                tagged = true;
                break;
            case TagPresence::Absent:
                break;
        }
    }

    for (const char* key : side_keys) {
        const std::string prefix(key);
        for (const auto& side : kSideSuffixes) {
            const std::string* raw = find_tag(tags, prefix + side.suffix);
            if (!raw) continue;

            switch (classify(normalize_value(*raw))) {
                case TagPresence::Positive:
                    tagged = true;
                    left = left || side.left;
                    right = right || side.right;
                    break;
                case TagPresence::Negative:
                    tagged = true;
                    break;
                case TagPresence::Absent:
                    break;
            }
        }
    }

    if (left && right) return SideFlags::Both;
    if (left) return SideFlags::Left;
    if (right) return SideFlags::Right;
    return tagged ? SideFlags::None : SideFlags::Unknown;
}

/**
 * @brief Read a sidewalk=* or sidewalk:<side>=* value
 *
 * sidewalk=separate means the footway is mapped as its own OSM way elsewhere in
 * the extract, so it reads as Negative and therefore as SideFlags::None. That
 * stops the profile builder synthesizing a second sidewalk on top of the way
 * that is already there, which SideFlags::Unknown would have allowed.
 *
 * @param value Normalized tag value
 * @return Presence of a sidewalk
 */
[[nodiscard]] TagPresence classify_sidewalk_value(const std::string& value) {
    if (value == "no" || value == "none" || value == "separate") return TagPresence::Negative;
    if (value == "yes" || value == "both" || value == "left" || value == "right")
        return TagPresence::Positive;
    return TagPresence::Absent;
}

/**
 * @brief Read a cycleway=* or cycleway:<side>=* value
 *
 * The value names the kind of cycle infrastructure rather than a side. Only
 * values that put a cycle lane on the carriageway count as present, so
 * cycleway=opposite, which allows contraflow without marking a lane, does not.
 *
 * @param value Normalized tag value
 * @return Presence of a cycle lane
 */
[[nodiscard]] TagPresence classify_cycleway_value(const std::string& value) {
    if (value == "no" || value == "none" || value == "separate") return TagPresence::Negative;
    if (value == "lane" || value == "track" || value == "opposite_lane" ||
        value == "opposite_track" || value == "share_busway" || value == "shared_lane" ||
        value == "yes")
        return TagPresence::Positive;
    return TagPresence::Absent;
}

/**
 * @brief Read a parking:lane:<side>=* or parking:<side>=* value
 *
 * The value names the parking arrangement (parallel, diagonal, perpendicular,
 * on_street, ...), so any value that is not a denial counts as present.
 *
 * @param value Normalized tag value
 * @return Presence of on-street parking
 */
[[nodiscard]] TagPresence classify_parking_value(const std::string& value) {
    if (value.empty()) return TagPresence::Absent;
    if (value == "no" || value == "none" || value == "separate" ||
        value == "no_parking" || value == "no_stopping" || value == "no_standing")
        return TagPresence::Negative;
    return TagPresence::Positive;
}

/**
 * @brief Read a shoulder=* or shoulder:<side>=* value
 * @param value Normalized tag value
 * @return Presence of a shoulder
 */
[[nodiscard]] TagPresence classify_shoulder_value(const std::string& value) {
    if (value == "no" || value == "none") return TagPresence::Negative;
    if (value == "yes" || value == "both" || value == "left" || value == "right")
        return TagPresence::Positive;
    return TagPresence::Absent;
}

/**
 * @brief Douglas-Peucker over indices rather than points
 *
 * Appends the kept indices of points[first..last] inclusive to @p keep. The
 * result matches geometry::simplify() point for point; only the return form
 * differs, because a road has to filter node_ids with the same set of survivors
 * to keep the two vectors parallel.
 *
 * @param points  Polyline being simplified
 * @param epsilon Maximum perpendicular deviation, in metres
 * @param first   Index of the first point of the span
 * @param last    Index of the last point of the span, greater than first
 * @param keep    Receives the surviving indices, in order
 */
void simplify_indices(const std::vector<glm::dvec2>& points, double epsilon,
                      size_t first, size_t last, std::vector<size_t>& keep) {
    double max_dist = 0.0;
    size_t max_idx = first;

    for (size_t i = first + 1; i < last; ++i) {
        const double dist =
            geometry::point_to_line_distance(points[i], points[first], points[last]);
        if (dist > max_dist) {
            max_dist = dist;
            max_idx = i;
        }
    }

    if (max_dist > epsilon) {
        simplify_indices(points, epsilon, first, max_idx, keep);
        keep.pop_back();    // max_idx is re-added as the first point of the second half
        simplify_indices(points, epsilon, max_idx, last, keep);
    } else {
        keep.push_back(first);
        keep.push_back(last);
    }
}

/**
 * @brief What the simplification reference count learns about one OSM node
 *
 * Counted across every road, exactly as RoadGraph::build() counts, so the set of
 * qualifying nodes here is the set the graph will later turn into GraphNodes.
 */
struct SimplifyNodeRefs {
    /// Times any road polyline visits this node. A closed loop counts it twice.
    uint32_t refs = 0;

    /// True when the node is the first or last point of some road
    bool endpoint = false;

    /// A node the road graph derives topology from, so simplification must keep it
    [[nodiscard]] bool qualifies() const { return refs >= 2 || endpoint; }
};

/// Reference counts by OSM node id, keyed exactly as RoadGraph::build() keys them
using SimplifyRefMap = std::unordered_map<NodeId, SimplifyNodeRefs>;

/**
 * @brief Reference count every node visited by @p roads
 *
 * Mirrors RoadGraph::build() pass 1: every visit counts, whatever the layer, and
 * the first and last point of each road are flagged as endpoints.
 *
 * @param roads Roads to count over
 * @return Counts by OSM node id
 */
[[nodiscard]] SimplifyRefMap count_road_node_refs(const std::vector<Road>& roads) {
    SimplifyRefMap refs;

    size_t total_points = 0;
    for (const auto& road : roads) total_points += road.node_ids.size();
    refs.reserve(total_points);

    for (const auto& road : roads) {
        if (road.node_ids.size() != road.polyline.size()) continue;
        if (road.node_ids.empty()) continue;

        const size_t last = road.node_ids.size() - 1;
        for (size_t i = 0; i <= last; ++i) {
            SimplifyNodeRefs& info = refs[road.node_ids[i]];
            ++info.refs;
            if (i == 0 || i == last) info.endpoint = true;
        }
    }

    return refs;
}

/**
 * @brief Simplify a road centerline in place, keeping node_ids parallel
 *
 * geometry::simplify() returns points only, so calling it on polyline alone
 * would leave node_ids the wrong size and RoadGraph::build() would drop the
 * road. Both vectors are filtered by the same surviving indices instead.
 *
 * Parallelism is not the only invariant that matters. Node *identity* is what
 * the road graph derives every junction from, and Douglas-Peucker guarantees
 * only that the two ends of a span survive. A junction node interior to a
 * straight through road deviates 0 m from the chord, so an unconstrained pass
 * would delete it and turn a T-junction into a dead end floating on top of an
 * uninterrupted edge. The road is therefore split at every node the graph will
 * qualify, and Douglas-Peucker runs independently on each span between them:
 * those nodes are span endpoints, so they are always kept, while the interior
 * geometry still simplifies.
 *
 * @param road      Road to simplify
 * @param tolerance Douglas-Peucker tolerance in metres
 * @param refs      Node reference counts over every road, from count_road_node_refs()
 */
void simplify_road(Road& road, double tolerance, const SimplifyRefMap& refs) {
    if (road.polyline.size() <= 2) return;

    const bool has_node_ids = road.node_ids.size() == road.polyline.size();
    const size_t last = road.polyline.size() - 1;

    // Anchors: the two ends, plus every interior node the graph will qualify.
    // Without node_ids there is no identity to protect and the whole road is one
    // span, which is the plain Douglas-Peucker behaviour.
    std::vector<size_t> anchors;
    anchors.reserve(8);
    anchors.push_back(0);
    if (has_node_ids) {
        for (size_t i = 1; i < last; ++i) {
            const auto it = refs.find(road.node_ids[i]);
            if (it != refs.end() && it->second.qualifies()) {
                anchors.push_back(i);
            }
        }
    }
    anchors.push_back(last);

    std::vector<size_t> keep;
    keep.reserve(road.polyline.size());
    for (size_t a = 0; a + 1 < anchors.size(); ++a) {
        simplify_indices(road.polyline, tolerance, anchors[a], anchors[a + 1], keep);
        if (a + 2 < anchors.size()) {
            keep.pop_back();    // the next span re-adds this anchor as its first point
        }
    }
    if (keep.size() >= road.polyline.size()) return;

    std::vector<glm::dvec2> polyline;
    std::vector<NodeId> node_ids;
    polyline.reserve(keep.size());
    if (has_node_ids) node_ids.reserve(keep.size());

    for (size_t index : keep) {
        polyline.push_back(road.polyline[index]);
        if (has_node_ids) node_ids.push_back(road.node_ids[index]);
    }

    road.polyline = std::move(polyline);
    road.node_ids = has_node_ids ? std::move(node_ids) : std::vector<NodeId>{};
}

} // namespace

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

        // Resolve coordinates, keeping the node IDs parallel to them
        std::vector<glm::dvec2> coords;
        std::vector<NodeId> node_ids;
        resolve_way_coords_with_ids(way, coords, node_ids);
        if (coords.size() < 2) continue;

        // Create road
        Road road;
        road.osm_id = way_id;
        road.polyline = std::move(coords);
        road.node_ids = std::move(node_ids);
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

        // Topology tags. layer keeps a bridge from joining the road beneath it,
        // and the junction and link flags change how the graph solves the node.
        if (const auto layer = parse_tag_int(way.tags, "layer")) {
            road.layer = *layer;
        }

        const std::string junction = tag_value(way.tags, "junction");
        road.is_roundabout = (junction == "roundabout" || junction == "circular");

        const std::string highway_value = normalize_value(highway_it->second);
        road.is_link = highway_value.ends_with("_link");

        // Cross-section tags, read by the profile builder
        road.lanes_forward = parse_lane_count(way.tags, "lanes:forward");
        road.lanes_backward = parse_lane_count(way.tags, "lanes:backward");

        road.sidewalk = read_sided_tags(way.tags, {"sidewalk"}, {"sidewalk"},
                                        classify_sidewalk_value);
        road.cycleway = read_sided_tags(way.tags, {"cycleway"}, {"cycleway"},
                                        classify_cycleway_value);
        road.parking = read_sided_tags(way.tags, {}, {"parking:lane", "parking"},
                                       classify_parking_value);
        road.shoulder = read_sided_tags(way.tags, {"shoulder"}, {"shoulder"},
                                        classify_shoulder_value);

        road.surface = tag_value(way.tags, "surface");
        road.smoothness = tag_value(way.tags, "smoothness");

        // Speed limit
        auto maxspeed_it = way.tags.find("maxspeed");
        if (maxspeed_it != way.tags.end()) {
            try {
                road.speed_limit = std::stof(maxspeed_it->second);
            } catch (...) {}
        }

        m_data.roads.push_back(std::move(road));
    }

    // Optional simplification, run as a post-pass rather than inside the loop.
    // Filters polyline and node_ids together so the two stay parallel, and needs
    // the reference counts over *every* road to know which nodes the road graph
    // will turn into junctions; see simplify_road().
    if (m_config.simplify_geometry) {
        const SimplifyRefMap refs = count_road_node_refs(m_data.roads);
        for (auto& road : m_data.roads) {
            simplify_road(road, m_config.simplify_tolerance, refs);
        }
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
    std::vector<NodeId> node_ids;
    resolve_way_coords_with_ids(way, coords, node_ids);
    return coords;
}

void OSMParser::resolve_way_coords_with_ids(const OSMWay& way,
                                            std::vector<glm::dvec2>& out_coords,
                                            std::vector<NodeId>& out_node_ids) const {
    out_coords.clear();
    out_node_ids.clear();
    out_coords.reserve(way.node_refs.size());
    out_node_ids.reserve(way.node_refs.size());

    for (NodeId node_id : way.node_refs) {
        auto it = m_data.nodes.find(node_id);
        if (it != m_data.nodes.end()) {
            const auto& node = it->second;
            glm::dvec2 local = m_converter.wgs84_to_local(node.lat, node.lon);
            // A node with no position is skipped from both outputs together, so
            // out_node_ids[i] always describes out_coords[i].
            out_coords.push_back(local);
            out_node_ids.push_back(node_id);
        }
    }
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
    // Check for explicit width tag. Values carry units in the wild: "12",
    // "12 m", "30 ft", "30'6\"".
    if (const std::string* width_value = find_tag(tags, "width")) {
        const auto meters = parse_length_meters(*width_value);
        if (meters && *meters > 0.0f) {
            return *meters;
        }
    }

    // Check lanes tag, then the directional lane tags when lanes is absent
    int lanes = parse_lane_count(tags, "lanes");
    if (lanes < 0) {
        lanes = directional_lane_total(tags);
    }
    if (lanes > 0) {
        return static_cast<float>(lanes) * 3.5f;  // 3.5m per lane
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
    // Check lanes tag, then the directional lane tags when lanes is absent
    const int lanes = parse_lane_count(tags, "lanes");
    if (lanes > 0) return lanes;

    const int directional = directional_lane_total(tags);
    if (directional > 0) return directional;

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
