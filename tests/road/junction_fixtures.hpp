/**
 * @file junction_fixtures.hpp
 * @brief Shared synthetic networks and 2D predicates for the P4 junction suites
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The five junction suites -- trim, polygon, curb, special, integration -- all
 * need the same three things: a small road graph whose geometry is known to the
 * metre, a set of profiles whose widths are known exactly, and a handful of 2D
 * predicates (signed area, point in ring, proper segment crossing) to check the
 * result against. Duplicating those across five translation units would make a
 * change to one fixture a change to five files, so they live here.
 *
 * ### Why the networks are synthetic
 *
 * The junction solver is closed-form. A symmetric crossroads has ONE right
 * answer for its trim distance and it can be written down, which is the whole
 * value of testing this phase: an approximate assertion against a parsed extract
 * would pass for any solver that produced a plausible-looking shape.
 *
 * The graphs are therefore built by handing RoadGraph::build() a ParsedOSMData
 * assembled in memory, never by parsing a file. That skips
 * OSMParser::recenter_on_features(), so a node placed at (0, 0) really is at
 * (0, 0) and a closed-form expectation can be asserted to 1e-9 rather than to a
 * visual tolerance.
 *
 * The profiles are built by hand for the same reason. build_profile() is P2's
 * contract and is tested by tests/road/test_road_profile.cpp; using it here would
 * couple every junction expectation to the tag table, so that adding a strip rule
 * moved every trim number in this directory.
 *
 * The fixtures in tests/data are still used, but for the cases that are ABOUT
 * real topology -- the T-junction on an interior node, the roundabout cycle, the
 * cul-de-sac -- where the shape of the network is the thing under test.
 *
 * ### Coordinates
 *
 * 2D local metres throughout, matching GraphEdge::polyline and
 * Centerline::stations. Mesh vertices are world space, Y up, and are brought back
 * with world_to_local(): `(x, y, z) -> (x, -z)`. That negation is the inverse of
 * the pipeline's own `(x, y_2d) -> vec3(x, height, -y_2d)` and getting it wrong
 * mirrors every comparison about y = 0, which a symmetric fixture would hide.
 * Every fixture here that cares is deliberately asymmetric in y.
 */

#pragma once

#include "framework.hpp"

#include "osm/parser.hpp"
#include "osm/road/centerline.hpp"
#include "osm/road/junction_polygon.hpp"
#include "osm/road/junction_trim.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#ifndef STRATUM_TEST_DATA_DIR
#error "STRATUM_TEST_DATA_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

namespace stratum::test::junction {

// ============================================================================
// Constants
// ============================================================================

/// Closed-form trim expectations are exact arithmetic; this absorbs nothing but rounding
inline constexpr double kExactEps = 1e-9;

/// Positions recovered from a mesh have been through float, so they carry ~1e-7 relative error
inline constexpr double kMeshEps = 1e-4;

/// Half the carriageway of the standard two-lane fixture road, metres
inline constexpr double kLaneWidth = 3.5;

// ============================================================================
// Synthetic network construction
// ============================================================================

/**
 * @brief One Road with topology attached, ready for ParsedOSMData::roads
 *
 * @param way_id   OSM way ID; must be unique within the fixture
 * @param node_ids Node IDs, parallel to @p points
 * @param points   Centerline in 2D local metres
 * @param type     Road classification
 * @return The road
 */
inline osm::Road make_road(osm::WayId way_id,
                           const std::vector<osm::NodeId>& node_ids,
                           const std::vector<glm::dvec2>& points,
                           osm::RoadType type = osm::RoadType::Residential) {
    osm::Road road;
    road.osm_id = way_id;
    road.polyline = points;
    road.node_ids = node_ids;
    road.type = type;
    road.lanes = 2;
    road.width = 7.0f;
    return road;
}

/// Wrap roads into the ParsedOSMData RoadGraph::build() expects
inline osm::ParsedOSMData make_data(std::vector<osm::Road> roads) {
    osm::ParsedOSMData data;
    data.roads = std::move(roads);
    data.stats.processed_roads = data.roads.size();
    return data;
}

/**
 * @brief Resample tolerances used by every fixture unless a test overrides them
 *
 * Smoothing is OFF. These tests reason about arc length along lines and circular
 * arcs they placed themselves; a fitted Catmull-Rom would move every station a
 * little for no benefit and turn an exact expectation into an approximate one.
 */
inline osm::road::ResampleConfig fixture_resample() {
    osm::road::ResampleConfig cfg;
    cfg.smooth = false;
    return cfg;
}

/**
 * @brief One centerline per graph edge, in EdgeId order
 *
 * Built the way RoadNetworkBuilder builds it, so the vector is exactly parallel
 * to graph.edges() as every P4 entry point requires.
 *
 * @param graph Built road graph
 * @param cfg   Resample tolerances
 * @return Centerlines indexed by EdgeId
 */
inline std::vector<osm::road::Centerline> make_centerlines(
    const osm::road::RoadGraph& graph,
    const osm::road::ResampleConfig& cfg = fixture_resample()) {
    std::vector<osm::road::Centerline> out;
    out.reserve(graph.edges().size());
    for (const auto& edge : graph.edges()) {
        out.push_back(osm::road::build_centerline(edge.polyline, cfg));
    }
    return out;
}

// ============================================================================
// Hand-built profiles
// ============================================================================

/**
 * @brief A carriageway of N equal lanes and nothing else
 *
 * total_width(), the carriageway envelope and twice ArmRef::half_width are all
 * the same number for this profile, which is what makes the symmetric-cross
 * expectation unambiguous: there is only one width in play.
 *
 * @param lanes      Lane count, at least 1
 * @param lane_width Width of each lane in metres
 * @return The profile, left to right
 */
inline osm::road::RoadProfile lane_profile(int lanes, double lane_width = kLaneWidth) {
    osm::road::RoadProfile profile;
    for (int i = 0; i < std::max(1, lanes); ++i) {
        osm::road::Strip strip;
        strip.width = static_cast<float>(lane_width);
        strip.height_left = 0.0f;
        strip.height_right = 0.0f;
        strip.material = MaterialId::Asphalt;
        strip.kind = osm::road::StripKind::Lane;
        profile.strips.push_back(strip);
    }
    return profile;
}

/**
 * @brief A carriageway with a flat sidewalk band on either side
 *
 * The two sidewalk widths are independent so a fixture can be made deliberately
 * asymmetric about its own centreline. That asymmetry is what makes the arm-end
 * side inversion assertable: on a symmetric profile a left/right swap is
 * invisible.
 *
 * Sidewalks are emitted at height 0 rather than at curb height. These tests are
 * about lateral placement, and a raised sidewalk would need CurbFace strips whose
 * widths would then also have to appear in every expected total.
 *
 * @param lanes      Lane count
 * @param left       Left sidewalk width in metres; 0 emits no strip
 * @param right      Right sidewalk width in metres; 0 emits no strip
 * @param lane_width Width of each lane in metres
 * @return The profile, left to right
 */
inline osm::road::RoadProfile sidewalk_profile(int lanes,
                                               double left,
                                               double right,
                                               double lane_width = kLaneWidth) {
    osm::road::RoadProfile profile;
    auto push = [&profile](double width, MaterialId material, osm::road::StripKind kind) {
        osm::road::Strip strip;
        strip.width = static_cast<float>(width);
        strip.height_left = 0.0f;
        strip.height_right = 0.0f;
        strip.material = material;
        strip.kind = kind;
        profile.strips.push_back(strip);
    };

    if (left > 0.0) push(left, MaterialId::Sidewalk, osm::road::StripKind::Sidewalk);
    for (int i = 0; i < std::max(1, lanes); ++i) {
        push(lane_width, MaterialId::Asphalt, osm::road::StripKind::Lane);
    }
    if (right > 0.0) push(right, MaterialId::Sidewalk, osm::road::StripKind::Sidewalk);
    return profile;
}

/**
 * @brief Carriageway envelope half width of a profile, metres
 *
 * The same quantity ArmRef::carriageway_half is documented to carry: half the
 * inclusive span from the first Lane-or-Median strip to the last, which is
 * centred on zero by RoadProfile::left_edge_offset(). Recomputed here rather than
 * read back from the solver, so the expectation does not come from the thing
 * under test.
 *
 * @param profile Profile to measure
 * @return Half the carriageway envelope; half the total width when the profile
 *         has no Lane and no Median strip
 */
inline double carriageway_half_of(const osm::road::RoadProfile& profile) {
    size_t first = profile.strips.size();
    size_t last = 0;
    bool found = false;
    for (size_t i = 0; i < profile.strips.size(); ++i) {
        const auto kind = profile.strips[i].kind;
        if (kind == osm::road::StripKind::Lane || kind == osm::road::StripKind::Median) {
            if (!found) first = i;
            last = i;
            found = true;
        }
    }
    if (!found) {
        return 0.5 * static_cast<double>(profile.total_width());
    }
    double span = 0.0;
    for (size_t i = first; i <= last; ++i) {
        span += static_cast<double>(profile.strips[i].width);
    }
    return 0.5 * span;
}

// ============================================================================
// The fillet reserve
// ============================================================================

/**
 * @brief Straight run solve_arm_trims() reserves for one corner's fillet, metres
 *
 * The trim solve does not stop at the point where two carriageways separate: the
 * corner between them is ROUNDED, and the arc is tangent to each cut face
 * `R * tan(theta / 2)` further back. Reserving that run is what stops every
 * square corner collapsing to a chamfer a few centimetres long -- see
 * TrimConfig::fillet_radius_width_factor.
 *
 * Recomputed here from the config rather than read out of the solver, so an
 * expectation does not come from the thing under test.
 *
 * @param cfg   Trim tolerances; the four fillet_* fields are the ones read
 * @param a     One arm's carriageway half width, metres
 * @param b     The other arm's carriageway half width, metres
 * @param turn  Turn the junction ring makes over the corner, radians. This is
 *              `pi - gap` for two arms `gap` apart, so a right-angle corner is
 *              pi/2 and NOT the 90 degrees between the arms by coincidence.
 * @return Metres reserved on both arms, 0 when the corner is drawn as a chord
 */
inline double fillet_reserve(const osm::road::TrimConfig& cfg, double a, double b, double turn) {
    if (!(cfg.fillet_radius_width_factor > 0.0) || turn < cfg.fillet_min_arc_angle) {
        return 0.0;
    }
    const double radius = std::clamp(cfg.fillet_radius_width_factor * std::min(2.0 * a, 2.0 * b),
                                     cfg.fillet_min_radius, cfg.fillet_max_radius);
    // Mirrors kMaxReserveTanHalf in junction_trim.cpp: the reserve is capped at
    // twice the radius so a near-parallel fork cannot demand an unbounded run.
    return radius * std::min(std::tan(0.5 * turn), 2.0);
}

/// Trim tolerances with the fillet reserve switched off, isolating the pairwise rule
inline osm::road::TrimConfig pairwise_only_trim() {
    osm::road::TrimConfig cfg;
    cfg.fillet_radius_width_factor = 0.0;
    return cfg;
}

// ============================================================================
// Solved nodes
// ============================================================================

/**
 * @brief A built graph with everything the junction entry points need
 *
 * Held by value. RoadGraph, Centerline and RoadProfile are all plain value types,
 * so a Fixture can be returned from a builder function and every EdgeId inside it
 * stays valid.
 */
struct Fixture {
    osm::road::RoadGraph graph;
    std::vector<osm::road::Centerline> centerlines;
    std::vector<osm::road::RoadProfile> profiles;
};

/**
 * @brief Build a fixture from roads plus one profile per SOURCE WAY
 *
 * A way splits into several edges at its graph nodes, and the profiles vector the
 * junction entry points take is indexed by EdgeId, not by WayId. This maps one
 * onto the other by GraphEdge::source_way, so a caller states "way 1 is wide, way
 * 2 is narrow" once and every edge of each way inherits it.
 *
 * @param roads       Roads to build the graph from
 * @param way_profile Profile per road, parallel to @p roads
 * @param cfg         Resample tolerances
 * @return The fixture; its profiles vector is parallel to graph.edges()
 */
inline Fixture make_fixture(const std::vector<osm::Road>& roads,
                            const std::vector<osm::road::RoadProfile>& way_profile,
                            const osm::road::ResampleConfig& cfg = fixture_resample()) {
    Fixture fixture;
    const osm::ParsedOSMData data = make_data(roads);
    fixture.graph.build(data);
    fixture.centerlines = make_centerlines(fixture.graph, cfg);

    fixture.profiles.reserve(fixture.graph.edges().size());
    for (const auto& edge : fixture.graph.edges()) {
        size_t which = 0;
        for (size_t r = 0; r < roads.size(); ++r) {
            if (roads[r].osm_id == edge.source_way) {
                which = r;
                break;
            }
        }
        fixture.profiles.push_back(which < way_profile.size() ? way_profile[which]
                                                              : lane_profile(2));
    }
    return fixture;
}

/**
 * @brief The graph node whose degree is @p degree, when there is exactly one
 *
 * @param graph  Built road graph
 * @param degree Arm count to look for
 * @return Its GraphNodeId, or kInvalidId when zero or several nodes match
 */
inline osm::road::GraphNodeId sole_node_of_degree(const osm::road::RoadGraph& graph,
                                                  size_t degree) {
    osm::road::GraphNodeId found = osm::road::kInvalidId;
    size_t count = 0;
    for (size_t i = 0; i < graph.nodes().size(); ++i) {
        if (graph.nodes()[i].degree() == degree) {
            found = static_cast<osm::road::GraphNodeId>(i);
            ++count;
        }
    }
    return count == 1 ? found : osm::road::kInvalidId;
}

/// The graph node built from a given OSM node ID, or kInvalidId
inline osm::road::GraphNodeId node_with_osm_id(const osm::road::RoadGraph& graph,
                                               osm::NodeId osm_id) {
    for (size_t i = 0; i < graph.nodes().size(); ++i) {
        if (graph.nodes()[i].osm_id == osm_id) {
            return static_cast<osm::road::GraphNodeId>(i);
        }
    }
    return osm::road::kInvalidId;
}

/**
 * @brief Collect, solve and cut one node in one call
 *
 * @param fixture   Built fixture
 * @param node      Node to solve
 * @param cfg       Trim tolerances
 * @param out_arms  Receives the solved arms in ascending bearing order
 * @param out_ends  Receives the cut cross-sections, parallel to @p out_arms
 * @return What solve_arm_trims() returned
 */
inline bool solve_node(const Fixture& fixture,
                       osm::road::GraphNodeId node,
                       const osm::road::TrimConfig& cfg,
                       std::vector<osm::road::ArmRef>& out_arms,
                       std::vector<osm::road::ArmEnd>& out_ends) {
    out_arms = osm::road::collect_arms(fixture.graph, fixture.profiles, node);
    const bool ok = osm::road::solve_arm_trims(fixture.graph, fixture.centerlines, node,
                                               out_arms, cfg);
    out_ends.clear();
    out_ends.reserve(out_arms.size());
    for (const auto& arm : out_arms) {
        out_ends.push_back(
            osm::road::arm_end(fixture.graph, fixture.centerlines, fixture.profiles, arm));
    }
    return ok;
}

// ============================================================================
// Standard synthetic junctions
// ============================================================================

/**
 * @brief Four equal arms at 0, 90, 180 and 270 degrees about the origin
 *
 * The closed-form case. Each pair of adjacent arms meets at a right angle with
 * equal carriageway halves, so every pair's demand is the neighbour's half width
 * and every trim is `carriageway_half + clearance`.
 *
 * @param lanes      Lane count on every arm
 * @param arm_length Length of each arm in metres, well past any trim clamp
 * @return The fixture; its single degree-4 node is at (0, 0)
 */
inline Fixture symmetric_cross(int lanes = 2, double arm_length = 200.0) {
    const std::vector<osm::Road> roads = {
        make_road(1, {1, 100, 2}, {{-arm_length, 0.0}, {0.0, 0.0}, {arm_length, 0.0}}),
        make_road(2, {3, 100, 4}, {{0.0, -arm_length}, {0.0, 0.0}, {0.0, arm_length}}),
    };
    return make_fixture(roads, {lane_profile(lanes), lane_profile(lanes)});
}

/**
 * @brief A wide primary crossing a narrow residential at right angles
 *
 * The sign test. The NARROW arm must retreat by the WIDE road's half width and
 * the wide arm only by the narrow road's, so the narrow arm is cut back further.
 * A solver that swapped the two still produces a plausible-looking junction.
 *
 * @param primary_half     Carriageway half width of the east-west primary, metres
 * @param residential_half Carriageway half width of the north-south residential, metres
 * @return The fixture; its single degree-4 node is at (0, 0). Way 1 is the
 *         primary and runs east-west; way 2 is the residential
 */
inline Fixture asymmetric_cross(double primary_half = 7.0, double residential_half = 3.0) {
    const double length = 200.0;
    std::vector<osm::Road> roads = {
        make_road(1, {1, 100, 2}, {{-length, 0.0}, {0.0, 0.0}, {length, 0.0}},
                  osm::RoadType::Primary),
        make_road(2, {3, 100, 4}, {{0.0, -length}, {0.0, 0.0}, {0.0, length}},
                  osm::RoadType::Residential),
    };
    return make_fixture(roads, {lane_profile(2, primary_half), lane_profile(2, residential_half)});
}

/**
 * @brief Three arms: east, north, and one leaving west at @p degrees from east
 *
 * Used for the acute and near-parallel cases. @p degrees is the bearing of the
 * third arm in DEGREES, so 15 gives a sliver fork and 180 a plain T.
 *
 * @param degrees    Bearing of the third arm
 * @param arm_length Length of each arm in metres
 * @return The fixture; its single degree-3 node is at (0, 0)
 */
inline Fixture fork(double degrees, double arm_length = 400.0) {
    const double radians = degrees * 3.14159265358979323846 / 180.0;
    const glm::dvec2 third{arm_length * std::cos(radians), arm_length * std::sin(radians)};
    const std::vector<osm::Road> roads = {
        make_road(1, {100, 1}, {{0.0, 0.0}, {arm_length, 0.0}}),
        make_road(2, {100, 2}, {{0.0, 0.0}, {0.0, arm_length}}),
        make_road(3, {100, 3}, {{0.0, 0.0}, {third.x, third.y}}),
    };
    return make_fixture(roads, {lane_profile(2), lane_profile(2), lane_profile(2)});
}

/**
 * @brief N arms leaving one node at the given bearings
 *
 * The general case the closed-form fixtures above are special cases of. Unequal
 * gaps are what make a junction's corners interesting: with every gap equal, each
 * arm's two neighbours demand the same retreat and the notch between the cut face
 * and the offset-line apex is exactly TrimConfig::clearance deep at every corner,
 * which is too shallow for the fillet radius ever to bind. Give the node a 80 /
 * 100 / 80 / 100 degree bearing pattern and the wider gaps get a notch over a
 * metre deep, which is where a fillet radius has room to matter.
 *
 * @param bearings_deg Bearing of each arm in degrees; one road each
 * @param arm_length   Length of every arm in metres
 * @param lanes        Lane count on every arm
 * @return The fixture; its single node of degree bearings_deg.size() is at (0, 0)
 */
inline Fixture star(const std::vector<double>& bearings_deg,
                    double arm_length = 200.0,
                    int lanes = 2) {
    std::vector<osm::Road> roads;
    std::vector<osm::road::RoadProfile> way_profiles;
    roads.reserve(bearings_deg.size());
    way_profiles.reserve(bearings_deg.size());

    for (size_t i = 0; i < bearings_deg.size(); ++i) {
        const double radians = bearings_deg[i] * 3.14159265358979323846 / 180.0;
        const glm::dvec2 tip{arm_length * std::cos(radians), arm_length * std::sin(radians)};
        roads.push_back(make_road(static_cast<osm::WayId>(i + 1),
                                  {100, static_cast<osm::NodeId>(i + 1)},
                                  {{0.0, 0.0}, {tip.x, tip.y}}));
        way_profiles.push_back(lane_profile(lanes));
    }
    return make_fixture(roads, way_profiles);
}

// ============================================================================
// Fixture files
// ============================================================================

/// Absolute path of a fixture in tests/data
inline std::filesystem::path fixture_path(const char* filename) {
    return std::filesystem::path(STRATUM_TEST_DATA_DIR) / filename;
}

/**
 * @brief Parse one fixture with roads only
 *
 * @param filename Fixture file name in tests/data
 * @return Parsed data, or std::nullopt when the parse failed
 */
inline std::optional<osm::ParsedOSMData> parse_fixture(const char* filename) {
    const auto path = fixture_path(filename);
    if (!std::filesystem::exists(path)) {
        report_failure(__FILE__, __LINE__, "fixture exists", "missing: " + path.string());
        return std::nullopt;
    }

    osm::OSMParser parser;
    osm::ParserConfig config;
    config.import_buildings = false;
    config.import_water = false;
    config.import_landuse = false;
    config.import_natural = false;
    config.simplify_geometry = false;
    parser.set_config(config);

    if (!parser.parse(path)) {
        report_failure(__FILE__, __LINE__, "parser.parse(fixture)",
                       path.string() + ": " + parser.get_error());
        return std::nullopt;
    }
    return parser.take_data();
}

/**
 * @brief Tag-driven profiles for a parsed fixture, indexed by EdgeId
 *
 * The synthetic tests build profiles by hand so their expectations are exact.
 * The fixture-file tests cannot: the whole point of parsing a real extract is
 * that its cross-sections come from its tags. This builds them exactly as
 * RoadNetworkBuilder does, raw way tags included, so a fixture test measures the
 * same profile the pipeline would.
 *
 * @param graph Built road graph
 * @param data  The ParsedOSMData the graph was built from, for the way tags
 * @param cfg   Profile widths and heights
 * @return Profiles parallel to graph.edges()
 */
inline std::vector<osm::road::RoadProfile> profiles_from_tags(
    const osm::road::RoadGraph& graph,
    const osm::ParsedOSMData& data,
    const osm::road::ProfileConfig& cfg = {}) {
    std::vector<osm::road::RoadProfile> out;
    out.reserve(graph.edges().size());
    for (const auto& edge : graph.edges()) {
        const auto it = data.ways.find(edge.source_way);
        const osm::TagMap* tags = (it != data.ways.end()) ? &it->second.tags : nullptr;
        out.push_back(osm::road::build_profile(edge, cfg, tags));
    }
    return out;
}

/// Every fixture in tests/data, in README table order
inline constexpr const char* kAllFixtures[] = {
    "four_way.osm", "t_junction.osm", "cul_de_sac.osm", "roundabout.osm",
    "motorway_link.osm", "rural_track.osm", "bridge_over.osm",
    "bridge_abutment.osm", "duplicate_node.osm",
};

// ============================================================================
// 2D predicates
// ============================================================================

/// Signed area of a ring, positive when counter-clockwise; first point not repeated
inline double signed_area(const std::vector<glm::dvec2>& ring) {
    if (ring.size() < 3) return 0.0;
    double sum = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % ring.size()];
        sum += a.x * b.y - b.x * a.y;
    }
    return 0.5 * sum;
}

/// Twice the signed area of a triangle, positive when counter-clockwise
inline double cross2(const glm::dvec2& o, const glm::dvec2& a, const glm::dvec2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

/**
 * @brief Two segments cross at an interior point of both
 *
 * PROPER intersection only: segments that merely share an endpoint, or that touch
 * end-on, do not count. Every junction test needs that distinction, because a
 * trimmed ribbon is SUPPOSED to butt exactly against the junction ring and a test
 * that called contact an overlap would fail on correct geometry.
 *
 * @param a0 First segment start
 * @param a1 First segment end
 * @param b0 Second segment start
 * @param b1 Second segment end
 * @return True when the two segments cross properly
 */
inline bool segments_cross(const glm::dvec2& a0, const glm::dvec2& a1,
                           const glm::dvec2& b0, const glm::dvec2& b1) {
    const double d1 = cross2(a0, a1, b0);
    const double d2 = cross2(a0, a1, b1);
    const double d3 = cross2(b0, b1, a0);
    const double d4 = cross2(b0, b1, a1);
    return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
           ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
}

/**
 * @brief The ring has no pair of non-adjacent segments crossing properly
 *
 * The independent check JunctionPolygon::self_intersecting is compared against.
 * Quadratic, which is irrelevant on a ring of tens of vertices.
 *
 * @param ring Closed ring, first point not repeated
 * @return True when no two non-adjacent segments cross
 */
inline bool ring_is_simple(const std::vector<glm::dvec2>& ring) {
    const size_t n = ring.size();
    if (n < 4) return true;
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2& a0 = ring[i];
        const glm::dvec2& a1 = ring[(i + 1) % n];
        for (size_t j = i + 1; j < n; ++j) {
            if (j == i || (j + 1) % n == i || (i + 1) % n == j) continue;
            const glm::dvec2& b0 = ring[j];
            const glm::dvec2& b1 = ring[(j + 1) % n];
            if (segments_cross(a0, a1, b0, b1)) return false;
        }
    }
    return true;
}

/**
 * @brief Crossing-number point-in-ring test
 *
 * @param ring Closed ring, first point not repeated
 * @param p    Point to test
 * @return True when @p p is strictly inside; behaviour on the boundary is
 *         unspecified, so callers must keep test points off it
 */
inline bool point_in_ring(const std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    if (ring.size() < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[j];
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x;
            if (p.x < x) inside = !inside;
        }
    }
    return inside;
}

/// Distance from a point to a segment
inline double point_segment_distance(const glm::dvec2& p, const glm::dvec2& a,
                                     const glm::dvec2& b) {
    const glm::dvec2 ab = b - a;
    const double len2 = glm::dot(ab, ab);
    if (len2 <= 1e-18) return glm::length(p - a);
    const double t = std::clamp(glm::dot(p - a, ab) / len2, 0.0, 1.0);
    return glm::length(p - (a + ab * t));
}

/// Smallest distance from a point to a ring's boundary
inline double point_ring_distance(const std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    if (ring.empty()) return 0.0;
    double best = 1e300;
    for (size_t i = 0; i < ring.size(); ++i) {
        best = std::min(best,
                        point_segment_distance(p, ring[i], ring[(i + 1) % ring.size()]));
    }
    return best;
}

/// Point is inside a triangle, boundary included
inline bool point_in_triangle(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b,
                              const glm::dvec2& c) {
    const double d1 = cross2(a, b, p);
    const double d2 = cross2(b, c, p);
    const double d3 = cross2(c, a, p);
    const bool has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    const bool has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);
    return !(has_neg && has_pos);
}

// ============================================================================
// Mesh inspection
// ============================================================================

/**
 * @brief Inverse of the pipeline's world mapping
 *
 * The pipeline maps `(x, y_2d) -> vec3(x, height, -y_2d)`, so recovering the 2D
 * point negates Z. A test that forgot the negation would compare a mirrored copy
 * of every footprint and pass on any fixture symmetric about y = 0.
 *
 * @param world World-space position, Y up
 * @return The 2D local position it was mapped from
 */
inline glm::dvec2 world_to_local(const glm::vec3& world) {
    return glm::dvec2{static_cast<double>(world.x), -static_cast<double>(world.z)};
}

/// True when every component of a world position is finite
inline bool is_finite(const glm::vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

/// One triangle of a Mesh, projected into 2D local metres, with its material
struct Tri2D {
    glm::dvec2 a{0.0};
    glm::dvec2 b{0.0};
    glm::dvec2 c{0.0};
    MaterialId material = MaterialId::Default;
    /// Geometric normal of the world-space triangle, un-normalised
    glm::dvec3 world_normal{0.0};
};

/**
 * @brief Project every triangle of a mesh into 2D, tagged with its material
 *
 * Materials come from Mesh::effective_submeshes(), so a mesh that left its
 * submeshes empty reports MaterialId::Default throughout rather than being
 * treated as untagged.
 *
 * @param mesh Mesh to walk
 * @return One entry per triangle, in index-buffer order
 */
inline std::vector<Tri2D> triangles_of(const Mesh& mesh) {
    std::vector<Tri2D> out;
    if (mesh.indices.size() < 3) return out;

    std::vector<MaterialId> per_triangle(mesh.indices.size() / 3, MaterialId::Default);
    for (const SubMesh& sub : mesh.effective_submeshes()) {
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset) + sub.index_count) / 3u;
        for (size_t t = first; t < last && t < per_triangle.size(); ++t) {
            per_triangle[t] = sub.material;
        }
    }

    out.reserve(per_triangle.size());
    for (size_t t = 0; t < per_triangle.size(); ++t) {
        const uint32_t i0 = mesh.indices[t * 3 + 0];
        const uint32_t i1 = mesh.indices[t * 3 + 1];
        const uint32_t i2 = mesh.indices[t * 3 + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() ||
            i2 >= mesh.vertices.size()) {
            continue;
        }
        const glm::vec3 p0 = mesh.vertices[i0].position;
        const glm::vec3 p1 = mesh.vertices[i1].position;
        const glm::vec3 p2 = mesh.vertices[i2].position;

        Tri2D tri;
        tri.a = world_to_local(p0);
        tri.b = world_to_local(p1);
        tri.c = world_to_local(p2);
        tri.material = per_triangle[t];
        tri.world_normal = glm::cross(glm::dvec3(p1) - glm::dvec3(p0),
                                      glm::dvec3(p2) - glm::dvec3(p0));
        out.push_back(tri);
    }
    return out;
}

/**
 * @brief A point lies inside some triangle of a mesh, in plan view
 *
 * Height is ignored: this asks whether the surface covers the ground under @p p,
 * which is the question every "is there a hole here" and "does this lie on top of
 * that" assertion in the junction suites is really asking.
 *
 * @param tris Triangles from triangles_of()
 * @param p    Probe point in 2D local metres
 * @return True when any triangle contains it
 */
inline bool covered_in_plan(const std::vector<Tri2D>& tris, const glm::dvec2& p) {
    for (const Tri2D& tri : tris) {
        if (point_in_triangle(p, tri.a, tri.b, tri.c)) return true;
    }
    return false;
}

/// Total unsigned plan-view area of a triangle list
inline double plan_area(const std::vector<Tri2D>& tris) {
    double sum = 0.0;
    for (const Tri2D& tri : tris) {
        sum += 0.5 * std::fabs(cross2(tri.a, tri.b, tri.c));
    }
    return sum;
}

/**
 * @brief A triangle and a ring overlap in plan view
 *
 * True when any triangle edge crosses any ring edge properly, when the triangle's
 * centroid is inside the ring, or when any ring vertex is inside the triangle.
 * Contact along a shared boundary is deliberately NOT an overlap; see
 * segments_cross().
 *
 * @param tri  Triangle in 2D local metres
 * @param ring Closed ring, first point not repeated
 * @return True when the two share interior area
 */
inline bool triangle_overlaps_ring(const Tri2D& tri, const std::vector<glm::dvec2>& ring) {
    if (ring.size() < 3) return false;

    const glm::dvec2 corners[3] = {tri.a, tri.b, tri.c};
    for (size_t i = 0; i < 3; ++i) {
        const glm::dvec2& e0 = corners[i];
        const glm::dvec2& e1 = corners[(i + 1) % 3];
        for (size_t j = 0; j < ring.size(); ++j) {
            if (segments_cross(e0, e1, ring[j], ring[(j + 1) % ring.size()])) return true;
        }
    }

    const glm::dvec2 centroid = (tri.a + tri.b + tri.c) / 3.0;
    if (point_in_ring(ring, centroid)) return true;

    for (const glm::dvec2& p : ring) {
        if (point_in_triangle(p, tri.a, tri.b, tri.c) &&
            std::fabs(cross2(tri.a, tri.b, tri.c)) > 1e-12) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Shrink a ring towards a point
 *
 * Used to turn "the ribbon must not overlap the junction" into a test that
 * tolerates exact contact. A trimmed arm end lands ON the junction ring by
 * construction, so the raw ring and the ribbon share a boundary; pulling the ring
 * in by a few centimetres leaves a strict interior that nothing may enter.
 *
 * @param ring   Ring to shrink
 * @param about  Point to shrink towards, normally JunctionPolygon::centroid
 * @param metres How far to pull each vertex in, approximately
 * @return The shrunk ring
 */
inline std::vector<glm::dvec2> shrink_ring(const std::vector<glm::dvec2>& ring,
                                           const glm::dvec2& about, double metres) {
    std::vector<glm::dvec2> out;
    out.reserve(ring.size());
    for (const glm::dvec2& p : ring) {
        const glm::dvec2 d = p - about;
        const double len = glm::length(d);
        if (len <= metres * 2.0) {
            out.push_back(about);
        } else {
            out.push_back(about + d * ((len - metres) / len));
        }
    }
    return out;
}

} // namespace stratum::test::junction
