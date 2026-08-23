/**
 * @file test_sidewalk_dedup.cpp
 * @brief The doubly-mapped sidewalk, and the footways that are not one
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * OSM carries a sidewalk two ways at once and real extracts frequently carry
 * both for one stretch of street: a `sidewalk=both` tag on the carriageway, and
 * a separate `highway=footway` way surveyed a few metres out. Built naively,
 * the same footway comes out twice, a couple of metres apart, at slightly
 * different elevations, z-fighting across its whole overlap.
 *
 * The feature is a pure query, so the tests split cleanly in two.
 *
 * The exact half is mask_side(): a set subtraction over five enum values with a
 * documented truth table, in which the only interesting row is Unknown. Unknown
 * means "no tag, infer a class default" everywhere else in the codebase.
 * Subtracting a side from it must RESOLVE it, because leaving it Unknown makes
 * build_profile() infer the default straight back and re-synthesise exactly the
 * sidewalk that was just suppressed. That row is silent: the suppression is
 * reported, the counts look right, and the geometry is unchanged.
 *
 * The measured half is the end-to-end one, and it is the reason the fixture puts
 * the surveyed footway 5 m out rather than 3. At 5 m it sits ON the synthesised
 * strip, so the overlap is real and its disappearance is measurable: the test
 * samples the footway's own centreline against the carriageway's Sidewalk
 * triangles and asserts the cover goes from present to absent. Counting strips
 * would prove that a strip went away; this proves the DUPLICATION went away,
 * which is the thing the plan asks for.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests SidewalkDedup
 * @endcode
 */

#include "framework.hpp"
#include "road/p5_p6_fixtures.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/corridor.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/road/sidewalk_dedup.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::osm::NodeId;
using stratum::osm::ParsedOSMData;
using stratum::osm::Road;
using stratum::osm::RoadType;
using stratum::osm::SideFlags;
using stratum::osm::WayId;
using stratum::osm::side_flags_name;
using stratum::osm::road::Corridor;
using stratum::osm::road::CorridorConfig;
using stratum::osm::road::DedupConfig;
using stratum::osm::road::DedupResult;
using stratum::osm::road::EdgeId;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::ProfileConfig;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::StripKind;
using stratum::osm::road::build_centerline;
using stratum::osm::road::build_corridor;
using stratum::osm::road::build_profile;
using stratum::osm::road::dedup_sidewalks;
using stratum::osm::road::kInvalidId;
using stratum::osm::road::mask_side;

namespace p5 = stratum::test::p5;
namespace jt = stratum::test::junction;

/// The EdgeId of way @p way whose node list contains @p node
EdgeId edge_of_way_through(const stratum::osm::road::RoadGraph& graph, WayId way, NodeId node) {
    for (EdgeId id : p5::edges_of_way(graph, way)) {
        const auto& edge = graph.edge(id);
        if (std::find(edge.node_ids.begin(), edge.node_ids.end(), node) != edge.node_ids.end()) {
            return id;
        }
    }
    return kInvalidId;
}

/// Number of Sidewalk strips in a profile
size_t sidewalk_strips(const RoadProfile& profile) {
    size_t n = 0;
    for (const auto& strip : profile.strips) {
        if (strip.kind == StripKind::Sidewalk) ++n;
    }
    return n;
}

/// Report a SideFlags mismatch with both values spelled out
void check_side(SideFlags actual, SideFlags expected, const char* what, const char* file, int line) {
    if (actual != expected) {
        stratum::test::report_failure(file, line, what,
                                      std::string("actual: ") + side_flags_name(actual) +
                                          "  expected: " + side_flags_name(expected));
    }
}

#define CHECK_SIDE(actual, expected) \
    check_side((actual), (expected), #actual " == " #expected, __FILE__, __LINE__)

} // namespace

// ============================================================================
// mask_side
// ============================================================================

/**
 * The documented truth table, row by row, Unknown included.
 *
 * Every row of the table in sidewalk_dedup.hpp is asserted here, plus the rows
 * it implies. The Unknown rows are the ones worth reading: subtracting Left from
 * Unknown must give Right and never Unknown, because Unknown feeds a class
 * default back into build_profile() and re-creates the strip that was just
 * suppressed.
 */
TEST(SidewalkDedup, mask_side_matches_the_documented_truth_table) {
    // Suppressing nothing never changes the tag.
    CHECK_SIDE(mask_side(SideFlags::Both, SideFlags::None), SideFlags::Both);
    CHECK_SIDE(mask_side(SideFlags::Left, SideFlags::None), SideFlags::Left);
    CHECK_SIDE(mask_side(SideFlags::Right, SideFlags::None), SideFlags::Right);
    CHECK_SIDE(mask_side(SideFlags::None, SideFlags::None), SideFlags::None);
    CHECK_SIDE(mask_side(SideFlags::Unknown, SideFlags::None), SideFlags::Unknown);

    // Both, minus one side, is the other side.
    CHECK_SIDE(mask_side(SideFlags::Both, SideFlags::Left), SideFlags::Right);
    CHECK_SIDE(mask_side(SideFlags::Both, SideFlags::Right), SideFlags::Left);
    CHECK_SIDE(mask_side(SideFlags::Both, SideFlags::Both), SideFlags::None);

    // One side, minus itself, is nothing; minus the other side, unchanged.
    CHECK_SIDE(mask_side(SideFlags::Left, SideFlags::Left), SideFlags::None);
    CHECK_SIDE(mask_side(SideFlags::Left, SideFlags::Right), SideFlags::Left);
    CHECK_SIDE(mask_side(SideFlags::Left, SideFlags::Both), SideFlags::None);
    CHECK_SIDE(mask_side(SideFlags::Right, SideFlags::Right), SideFlags::None);
    CHECK_SIDE(mask_side(SideFlags::Right, SideFlags::Left), SideFlags::Right);
    CHECK_SIDE(mask_side(SideFlags::Right, SideFlags::Both), SideFlags::None);

    // None stays None: the tag said no, and nothing may put a sidewalk back.
    CHECK_SIDE(mask_side(SideFlags::None, SideFlags::Left), SideFlags::None);
    CHECK_SIDE(mask_side(SideFlags::None, SideFlags::Both), SideFlags::None);

    // Unknown RESOLVES rather than staying Unknown.
    CHECK_SIDE(mask_side(SideFlags::Unknown, SideFlags::Left), SideFlags::Right);
    CHECK_SIDE(mask_side(SideFlags::Unknown, SideFlags::Right), SideFlags::Left);
    CHECK_SIDE(mask_side(SideFlags::Unknown, SideFlags::Both), SideFlags::None);
}

/**
 * A masked Unknown does not grow its sidewalk back.
 *
 * The consequence of the row above, asserted where it actually bites. A
 * residential edge with no sidewalk tag has SideFlags::Unknown and
 * ProfileConfig::synthesize_sidewalks gives it two strips from its class. Masking
 * one side must leave exactly one.
 */
TEST(SidewalkDedup, masking_an_untagged_edge_leaves_exactly_one_sidewalk) {
    GraphEdge edge;
    edge.source_way = 1;
    edge.polyline = {{0.0, 0.0}, {100.0, 0.0}};
    edge.node_ids = {1, 2};
    edge.type = RoadType::Residential;
    edge.lanes = 2;
    edge.width = 7.0f;
    edge.sidewalk = SideFlags::Unknown;

    ProfileConfig cfg;
    cfg.synthesize_sidewalks = true;

    CHECK_EQ(sidewalk_strips(build_profile(edge, cfg, nullptr, SideFlags::None)), size_t{2});
    CHECK_EQ(sidewalk_strips(build_profile(edge, cfg, nullptr, SideFlags::Left)), size_t{1});
    CHECK_EQ(sidewalk_strips(build_profile(edge, cfg, nullptr, SideFlags::Both)), size_t{0});

    // The mask_side() spelling of the same thing, for a caller that alters the edge.
    GraphEdge masked = edge;
    masked.sidewalk = mask_side(edge.sidewalk, SideFlags::Left);
    CHECK_EQ(sidewalk_strips(build_profile(masked, cfg, nullptr)), size_t{1});
}

/**
 * The mask is purely subtractive: it can never add a sidewalk.
 *
 * `sidewalk=no` is SideFlags::None, and a stale or wrong mask handed to an edge
 * that has no sidewalk must degrade to a missing sidewalk rather than to one
 * appearing where the tag said there is none.
 */
TEST(SidewalkDedup, the_mask_never_adds_a_sidewalk) {
    GraphEdge edge;
    edge.source_way = 1;
    edge.polyline = {{0.0, 0.0}, {100.0, 0.0}};
    edge.node_ids = {1, 2};
    edge.type = RoadType::Residential;
    edge.lanes = 2;
    edge.width = 7.0f;
    edge.sidewalk = SideFlags::None;

    const ProfileConfig cfg;
    for (SideFlags mask : {SideFlags::None, SideFlags::Left, SideFlags::Right, SideFlags::Both,
                           SideFlags::Unknown}) {
        CHECK_EQ(sidewalk_strips(build_profile(edge, cfg, nullptr, mask)), size_t{0});
    }
}

// ============================================================================
// The query, over the fixture
// ============================================================================

/**
 * The parallel footway suppresses the left sidewalk of the carriageway edge it
 * runs beside, and nothing else.
 *
 * "Nothing else" carries as much weight as the match. Way 11000 splits at node
 * 1105 into a long western edge and a short eastern one; only the western edge
 * has the footway alongside it, and only its LEFT side is mapped separately. A
 * result that reported Both would delete the right-hand footway from a street
 * that has only the one, and a result that reached the eastern edge would delete
 * it from a stretch the surveyed footway never covers.
 */
TEST(SidewalkDedup, the_parallel_footway_suppresses_one_side_of_one_edge) {
    const p5::Network net = p5::make_network("sidewalk_dup.osm");
    if (!net.ok) return;

    const EdgeId west = edge_of_way_through(net.graph, 11000, 1101);
    const EdgeId east = edge_of_way_through(net.graph, 11000, 1106);
    CHECK_TRUE(west != kInvalidId);
    CHECK_TRUE(east != kInvalidId);
    CHECK_TRUE(west != east);
    if (west == kInvalidId || east == kInvalidId) return;

    // The carriageway really does synthesise two sidewalks before dedup runs,
    // which is what makes a suppression meaningful.
    CHECK_EQ(sidewalk_strips(net.profiles[west]), size_t{2});

    const DedupResult dd = dedup_sidewalks(net.graph, net.centerlines, net.profiles, DedupConfig{});

    CHECK_EQ(dd.suppress_side.size(), net.graph.edges().size());
    CHECK_EQ(dd.suppressed_edges, size_t{1});
    CHECK_EQ(dd.matched_footways, size_t{1});

    CHECK_SIDE(dd.suppress_side[west], SideFlags::Left);
    for (size_t i = 0; i < dd.suppress_side.size(); ++i) {
        // Unknown is never produced, on any edge.
        CHECK_TRUE(dd.suppress_side[i] != SideFlags::Unknown);
        if (static_cast<EdgeId>(i) == west) continue;
        CHECK_SIDE(dd.suppress_side[i], SideFlags::None);
    }
}

/**
 * The crossing footway is not a sidewalk, even when it is the only footway
 * present.
 *
 * Way 11020 passes the RoadType filter dedup_sidewalks() uses, since that filter
 * is the whole of what a GraphEdge can tell it, and it shares a node with the
 * carriageway so it is as near as any sidewalk. Only the bearing test separates
 * it, and this asserts the separation with the parallel footway REMOVED so there
 * is nothing else that could account for a zero result.
 */
/**
 * A footway beside PART of an edge does not delete that edge's sidewalk.
 *
 * Suppression is per edge and per side, because RoadProfile is one cross-section
 * for the whole edge. So a footway that runs alongside a fifth of a carriageway
 * edge would, with only the parallel-fraction gate, take the synthesised
 * sidewalk -- and the curb face and curb top with it -- off ALL of it, leaving
 * the rest of the street with paving from neither source. The parallel fraction
 * cannot catch this: it is measured over the FOOTWAY, and a short path beside a
 * long road has every one of its own stations matched, so it scores 1.0.
 *
 * DedupConfig::min_edge_coverage is the second gate, measured the other way
 * round. Both halves are here: the short path changes nothing, and the long one
 * on the same geometry still suppresses, so the first result is the coverage
 * rule and not a broken fixture.
 */
TEST(SidewalkDedup, a_footway_covering_part_of_an_edge_suppresses_nothing) {
    const double road_length = 200.0;

    /// One straight street with a parallel footway of @p walk_length beside it
    const auto run = [&](double walk_length) {
        const double mid = 0.5 * road_length;
        const double half = 0.5 * walk_length;

        std::vector<Road> roads;
        roads.push_back(jt::make_road(1, {1, 2}, {{0.0, 0.0}, {road_length, 0.0}},
                                      RoadType::Residential));
        roads.push_back(jt::make_road(2, {10, 11},
                                      {{mid - half, 5.0}, {mid + half, 5.0}},
                                      RoadType::Footway));

        const ParsedOSMData data = jt::make_data(roads);
        stratum::osm::road::RoadGraph graph;
        graph.build(data);

        const std::vector<stratum::osm::road::Centerline> centerlines =
            jt::make_centerlines(graph);
        std::vector<RoadProfile> profiles;
        profiles.reserve(graph.edges().size());
        for (const auto& edge : graph.edges()) {
            profiles.push_back(build_profile(edge, ProfileConfig{}, nullptr));
        }
        return std::make_pair(dedup_sidewalks(graph, centerlines, profiles, DedupConfig{}),
                              graph.edges().size());
    };

    // A fifth of the edge: the surveyed path replaces nothing, so nothing goes.
    {
        const auto [dd, edges] = run(40.0);
        CHECK_EQ(dd.suppress_side.size(), edges);
        CHECK_EQ(dd.suppressed_edges, size_t{0});
        for (const SideFlags side : dd.suppress_side) {
            CHECK_SIDE(side, SideFlags::None);
        }
    }

    // Four fifths of it: the same shape, now genuinely that street's footway.
    {
        const auto [dd, edges] = run(160.0);
        CHECK_EQ(dd.suppress_side.size(), edges);
        CHECK_EQ(dd.suppressed_edges, size_t{1});
    }
}

TEST(SidewalkDedup, the_crossing_footway_is_never_matched) {
    p5::Network net = p5::make_network("sidewalk_dup.osm");
    if (!net.ok) return;

    // Drop way 11010, the parallel footway, and rebuild. What is left is the
    // carriageway and the footway that crosses it.
    ParsedOSMData reduced = net.data;
    reduced.roads.erase(std::remove_if(reduced.roads.begin(), reduced.roads.end(),
                                       [](const Road& r) { return r.osm_id == 11010; }),
                        reduced.roads.end());
    CHECK_EQ(reduced.roads.size(), net.data.roads.size() - 1);

    stratum::osm::road::RoadGraph graph;
    graph.build(reduced);
    const std::vector<stratum::osm::road::Centerline> centerlines = jt::make_centerlines(graph);
    const std::vector<RoadProfile> profiles = jt::profiles_from_tags(graph, reduced);

    const DedupResult dd = dedup_sidewalks(graph, centerlines, profiles, DedupConfig{});
    CHECK_EQ(dd.suppressed_edges, size_t{0});
    CHECK_EQ(dd.matched_footways, size_t{0});
    for (SideFlags side : dd.suppress_side) {
        CHECK_SIDE(side, SideFlags::None);
    }
}

/**
 * The master switch reproduces the previous phase exactly.
 *
 * A disabled dedup returns a mask sized to the graph and filled with None, so a
 * caller needs no branch and applying the result is a no-op. Both halves are
 * asserted: the sizing, so an unconditional index is safe, and the contents.
 */
TEST(SidewalkDedup, disabling_dedup_returns_a_sized_all_none_mask) {
    const p5::Network net = p5::make_network("sidewalk_dup.osm");
    if (!net.ok) return;

    DedupConfig off;
    off.enabled = false;
    const DedupResult dd = dedup_sidewalks(net.graph, net.centerlines, net.profiles, off);

    CHECK_EQ(dd.suppress_side.size(), net.graph.edges().size());
    CHECK_EQ(dd.suppressed_edges, size_t{0});
    CHECK_EQ(dd.matched_footways, size_t{0});
    for (SideFlags side : dd.suppress_side) {
        CHECK_SIDE(side, SideFlags::None);
    }
}

/**
 * A footway on another layer is not a sidewalk.
 *
 * A footbridge over a road runs directly above it and is metres from it in plan,
 * so the offset and bearing tests both pass and only the layer filter separates
 * them. Built synthetically because the geometry has to be exactly parallel and
 * exactly overhead for the test to be about the layer rather than about the
 * distance.
 */
TEST(SidewalkDedup, a_footway_on_another_layer_is_not_a_sidewalk) {
    const auto make = [](WayId id, RoadType type, int layer, double offset) {
        Road road;
        road.osm_id = id;
        road.type = type;
        road.layer = layer;
        road.lanes = 2;
        road.width = 7.0f;
        road.sidewalk = (type == RoadType::Residential) ? SideFlags::Both : SideFlags::Unknown;
        road.polyline = {{0.0, offset}, {60.0, offset}, {120.0, offset}};
        road.node_ids = {static_cast<NodeId>(id * 10 + 1), static_cast<NodeId>(id * 10 + 2),
                         static_cast<NodeId>(id * 10 + 3)};
        return road;
    };

    // Same geometry twice; only the layer differs between the two runs.
    for (int footway_layer : {0, 1}) {
        ParsedOSMData data;
        data.roads = {make(1, RoadType::Residential, 0, 0.0),
                      make(2, RoadType::Footway, footway_layer, 5.0)};
        data.stats.processed_roads = data.roads.size();

        stratum::osm::road::RoadGraph graph;
        graph.build(data);
        const std::vector<stratum::osm::road::Centerline> centerlines = jt::make_centerlines(graph);
        std::vector<RoadProfile> profiles;
        for (const auto& edge : graph.edges()) {
            profiles.push_back(build_profile(edge, ProfileConfig{}));
        }

        const DedupResult dd = dedup_sidewalks(graph, centerlines, profiles, DedupConfig{});
        if (footway_layer == 0) {
            CHECK_EQ(dd.matched_footways, size_t{1});
            CHECK_EQ(dd.suppressed_edges, size_t{1});
        } else {
            CHECK_EQ(dd.matched_footways, size_t{0});
            CHECK_EQ(dd.suppressed_edges, size_t{0});
        }
    }
}

// ============================================================================
// End to end: the overlap is gone
// ============================================================================

/**
 * Applying the mask leaves ONE sidewalk over the deduped stretch, not two.
 *
 * This is the assertion the feature exists for, and it is deliberately measured
 * in GEOMETRY rather than in strip counts. The test samples points along the
 * surveyed footway's own centreline and asks whether the carriageway's
 * MaterialId::Sidewalk triangles cover them in plan:
 *
 * - Before the mask, they do. That is the z-fighting duplication.
 * - After the mask, they do not. The surveyed footway is left standing alone.
 *
 * The surveyed footway is still extruded either way: it is real surveyed
 * geometry and only the carriageway's synthesised strip goes away. And the RIGHT
 * side of the carriageway keeps its sidewalk, which the second half asserts by
 * showing that Sidewalk geometry still exists on the carriageway and that all of
 * it is now on the negative lateral side.
 */
TEST(SidewalkDedup, applying_the_mask_removes_the_overlapping_sidewalk_only) {
    const p5::Network net = p5::make_network("sidewalk_dup.osm");
    if (!net.ok) return;

    const EdgeId west = edge_of_way_through(net.graph, 11000, 1101);
    const EdgeId walk = p5::sole_edge_of_way(net.graph, 11010);
    CHECK_TRUE(west != kInvalidId);
    CHECK_TRUE(walk != kInvalidId);
    if (west == kInvalidId || walk == kInvalidId) return;

    const DedupResult dd = dedup_sidewalks(net.graph, net.centerlines, net.profiles, DedupConfig{});
    if (dd.suppress_side.size() != net.graph.edges().size()) return;

    const auto& edge = net.graph.edge(west);
    const auto* tags = p5::tags_of(net.data, edge);
    const RoadProfile before = build_profile(edge, ProfileConfig{}, tags, SideFlags::None);
    const RoadProfile after = build_profile(edge, ProfileConfig{}, tags, dd.suppress_side[west]);

    CHECK_EQ(sidewalk_strips(before), size_t{2});
    CHECK_EQ(sidewalk_strips(after), size_t{1});

    const auto& cl = net.centerlines[west];
    const Corridor corr_before = build_corridor(cl, before, CorridorConfig{});
    const Corridor corr_after = build_corridor(cl, after, CorridorConfig{});

    const std::vector<jt::Tri2D> walk_before =
        p5::triangles_with(corr_before.mesh, MaterialId::Sidewalk);
    const std::vector<jt::Tri2D> walk_after =
        p5::triangles_with(corr_after.mesh, MaterialId::Sidewalk);
    CHECK_TRUE(!walk_before.empty());
    CHECK_TRUE(!walk_after.empty());

    // Half the sidewalk area went away, and only half.
    const double area_before = jt::plan_area(walk_before);
    const double area_after = jt::plan_area(walk_after);
    CHECK_TRUE(area_before > 0.0);
    CHECK_NEAR(area_after / area_before, 0.5, 0.1);

    // Sample the surveyed footway's own centreline. Before the mask the
    // carriageway's sidewalk covers it; after, nothing of the carriageway does.
    const auto& walk_cl = net.centerlines[walk];
    size_t covered_before = 0;
    size_t covered_after = 0;
    size_t sampled = 0;
    for (const auto& station : walk_cl.stations) {
        ++sampled;
        if (jt::covered_in_plan(walk_before, station.position)) ++covered_before;
        if (jt::covered_in_plan(walk_after, station.position)) ++covered_after;
    }
    CHECK_TRUE(sampled >= 4);
    CHECK_TRUE(covered_before > sampled / 2);   // the duplication was real
    CHECK_EQ(covered_after, size_t{0});         // and it is gone

    // What remains is on the right of travel only, which is the side no separate
    // footway was surveyed on.
    for (const jt::Tri2D& tri : walk_after) {
        for (const glm::dvec2& p : {tri.a, tri.b, tri.c}) {
            CHECK_TRUE(p5::lateral_of(cl, p) < 0.0);
        }
    }

    // The surveyed footway is untouched and still extruded in its own right.
    CHECK_SIDE(dd.suppress_side[walk], SideFlags::None);
    const Corridor footway = build_corridor(walk_cl, net.profiles[walk], CorridorConfig{});
    CHECK_TRUE(!footway.mesh.vertices.empty());
    CHECK_TRUE(p5::triangles_with(footway.mesh, MaterialId::Sidewalk).size() > 0);
}
