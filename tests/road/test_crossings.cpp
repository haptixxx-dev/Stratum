/**
 * @file test_crossings.cpp
 * @brief Pedestrian crossings: locating them, clipping the zebra, dropping the kerb
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * A crossing is the one P5 feature that has to be FOUND before it can be built,
 * and finding it is where it goes wrong. OSM maps a crossing two ways at once,
 * neither of which is a point on a road: a `highway=crossing` node that is
 * usually interior to the carriageway way and therefore not a graph node at all,
 * and a `highway=footway` `footway=crossing` way that runs across the road at
 * whatever angle the desire line takes. Both have to resolve to the same thing,
 * an edge and an arclength, and neither may be resolved by proximity.
 *
 * Three assertions here are the ones that matter:
 *
 * 1. **The interior node is found at the right arclength.** A crossing node
 *    referenced by one way only stays a plain polyline vertex, so it has to be
 *    located by walking GraphEdge::node_ids. An implementation that looked at
 *    graph nodes would find nothing at all on the commonest shape in the data.
 * 2. **The zebra is clipped to the carriageway.** Crossing::width is
 *    RoadProfile::carriageway_width(), not total_width(), and stripes laid out
 *    over the total width run straight over the footway they are supposed to
 *    connect. The fixture street is 7 m of carriageway inside a profile more
 *    than 11 m wide, so an unclipped zebra is unmistakable.
 * 3. **The kerb drops, and the ring stays whole while it does.** A zebra running
 *    into a 150 mm wall is the visible failure the whole feature exists to
 *    avoid, and a kerb that STEPS to the lip instead of ramping tears the ring
 *    where it steps.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests Crossings
 * @endcode
 */

#include "framework.hpp"
#include "road/p5_p6_fixtures.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/crossings.hpp"
#include "osm/road/junction_builder.hpp"
#include "osm/road/junction_curb.hpp"
#include "osm/road/junction_polygon.hpp"
#include "osm/road/junction_trim.hpp"
#include "osm/road/marking_atlas.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::osm::NodeId;
using stratum::osm::road::ArmEnd;
using stratum::osm::road::ArmRef;
using stratum::osm::road::Crossing;
using stratum::osm::road::CrossingConfig;
using stratum::osm::road::CurbRing;
using stratum::osm::road::CurbRingConfig;
using stratum::osm::road::DroppedKerbSpan;
using stratum::osm::road::EdgeId;
using stratum::osm::road::FilletConfig;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::JunctionPolygon;
using stratum::osm::road::KerbDrops;
using stratum::osm::road::MarkingSprite;
using stratum::osm::road::RoadElevationSolver;
using stratum::osm::road::TrimConfig;
using stratum::osm::road::build_crossing;
using stratum::osm::road::build_curb_ring;
using stratum::osm::road::build_junction_polygon;
using stratum::osm::road::driveway_kerb_spans;
using stratum::osm::road::dropped_kerb_spans;
using stratum::osm::road::find_crossings;
using stratum::osm::road::kInvalidId;

namespace p5 = stratum::test::p5;
namespace jt = stratum::test::junction;

// ============================================================================
// The fixture, taken as far as located crossings
// ============================================================================

/**
 * @brief tests/data/crossing.osm with find_crossings() already run over it
 *
 * The elevation solver is left UNSOLVED. find_crossings() documents that as
 * leaving every Crossing::height at 0, which is what these tests want: the
 * question here is where a crossing is in plan and how wide it is, and a solved
 * height would only add a number every assertion would have to carry.
 */
struct Located {
    p5::Network net;
    RoadElevationSolver elevation;
    std::vector<Crossing> crossings;
    bool ok = false;
};

Located locate(const CrossingConfig& cfg = {}) {
    Located out;
    out.net = p5::make_network("crossing.osm");
    if (!out.net.ok) return out;
    out.crossings = find_crossings(out.net.graph, out.net.data, out.net.centerlines,
                                   out.net.profiles, out.elevation, cfg);
    out.ok = true;
    return out;
}

/// Every located crossing sitting on an edge of one OSM way
std::vector<Crossing> on_way(const Located& located, stratum::osm::WayId way) {
    const std::vector<EdgeId> ids = p5::edges_of_way(located.net.graph, way);
    std::vector<Crossing> out;
    for (const Crossing& c : located.crossings) {
        if (std::find(ids.begin(), ids.end(), c.edge) != ids.end()) out.push_back(c);
    }
    return out;
}

/// Chord length along a polyline up to the given vertex index
double chord_to(const std::vector<glm::dvec2>& polyline, size_t index) {
    double sum = 0.0;
    for (size_t i = 0; i + 1 <= index && i + 1 < polyline.size(); ++i) {
        sum += glm::length(polyline[i + 1] - polyline[i]);
    }
    return sum;
}

/// Index of an OSM node in an edge's node list, or npos
size_t index_of_node(const stratum::osm::road::GraphEdge& edge, NodeId node) {
    for (size_t i = 0; i < edge.node_ids.size(); ++i) {
        if (edge.node_ids[i] == node) return i;
    }
    return static_cast<size_t>(-1);
}

/// The 2D local position of a parsed OSM node, taken from a road that references it
bool local_position_of_node(const p5::Network& net, NodeId node, glm::dvec2& out) {
    for (const auto& road : net.data.roads) {
        for (size_t i = 0; i < road.node_ids.size() && i < road.polyline.size(); ++i) {
            if (road.node_ids[i] == node) {
                out = road.polyline[i];
                return true;
            }
        }
    }
    return false;
}

/// Rotate a 2D vector a quarter turn counter-clockwise
glm::dvec2 perp(const glm::dvec2& v) { return glm::dvec2{-v.y, v.x}; }

} // namespace

// ============================================================================
// Locating a crossing
// ============================================================================

/**
 * The `highway=crossing` node interior to way 10000 is found, at its own
 * arclength.
 *
 * Node 1003 is referenced by exactly one way and is not that way's endpoint, so
 * RoadGraph deliberately gives it no GraphNode: it is an ordinary shape point.
 * The only way to find it is to walk the edge's node_ids, and the only way to
 * place it is to take the arclength of that vertex. Smoothing is off in the
 * fixture pipeline, so on a straight road that arclength is exactly the chord
 * length of the polyline up to it and the expectation is arithmetic rather than
 * a tolerance around a resampled curve.
 *
 * at_junction must be false and node must be kInvalidId. A mid-block crossing
 * that reported itself as a junction crossing would be pushed CrossingConfig
 * setback metres along the road from where the surveyor put it, and would then
 * ask for a kerb drop on a ring that does not exist.
 */
TEST(Crossings, interior_crossing_node_is_found_at_its_own_arclength) {
    const Located located = locate();
    if (!located.ok) return;

    const std::vector<Crossing> found = on_way(located, 10000);
    CHECK_EQ(found.size(), size_t{1});
    if (found.size() != 1) return;

    const Crossing& c = found.front();
    const auto& edge = located.net.graph.edge(c.edge);
    const auto& cl = located.net.centerlines[c.edge];

    const size_t index = index_of_node(edge, 1003);
    CHECK_TRUE(index != static_cast<size_t>(-1));
    if (index == static_cast<size_t>(-1)) return;

    const double expected_arc = chord_to(edge.polyline, index);
    CHECK_NEAR(c.arclength, expected_arc, 0.05);
    CHECK_NEAR(glm::length(c.position - edge.polyline[index]), 0.0, 0.05);

    CHECK_FALSE(c.at_junction);
    CHECK_EQ(c.node, kInvalidId);

    // The width is the carriageway, not the whole profile. That is the number
    // build_crossing() lays its stripes across.
    const auto& profile = located.net.profiles[c.edge];
    CHECK_NEAR(c.width, profile.carriageway_width(), 1e-4);
    CHECK_TRUE(profile.total_width() > profile.carriageway_width() + 2.0);

    // The axis is the station's left normal, so it is a unit vector square to the
    // direction of travel.
    CHECK_NEAR(glm::length(c.axis), 1.0, 1e-9);
    const auto& station = cl.stations[p5::nearest_station(cl, c.position)];
    CHECK_NEAR(glm::dot(c.axis, station.tangent), 0.0, 1e-6);
}

/**
 * The skew `footway=crossing` way over way 10010 is found, on the road it
 * crosses, and is not discarded for being skew.
 *
 * Way 10020 meets way 10010 at about 53 degrees rather than square. That is the
 * ordinary state of a desire-line crossing and it is the case an implementation
 * that only accepted a perpendicular crossing way would silently drop. The
 * assertion that matters is therefore that ONE crossing comes back at all, on
 * the carriageway rather than on the footway, and that its position sits on the
 * carriageway's own centreline instead of somewhere along the footway's coarse
 * geometry.
 *
 * The AXIS, however, is the carriageway's left normal and not the footway's own
 * bearing. That is the frozen contract in crossings.hpp: stripes are laid out
 * along Station::normal and extruded along the direction of vehicle travel, so a
 * zebra is always square to the road it crosses however the footway was drawn.
 * The skew is preserved in the sense that the crossing is accepted, not in the
 * sense that the stripes lean. This test pins that reading, because it is the
 * one place the two possible readings differ visibly.
 */
TEST(Crossings, skew_footway_crossing_lands_on_the_carriageway_it_crosses) {
    const Located located = locate();
    if (!located.ok) return;

    // The fixture really is skew: if this fails the fixture drifted, not the code.
    glm::dvec2 foot_a{0.0};
    glm::dvec2 foot_b{0.0};
    glm::dvec2 shared{0.0};
    CHECK_TRUE(local_position_of_node(located.net, 1021, foot_a));
    CHECK_TRUE(local_position_of_node(located.net, 1022, foot_b));
    CHECK_TRUE(local_position_of_node(located.net, 1012, shared));
    const glm::dvec2 foot_dir = glm::normalize(foot_b - foot_a);
    const glm::dvec2 road_dir = glm::dvec2{1.0, 0.0};
    CHECK_TRUE(std::fabs(glm::dot(foot_dir, road_dir)) > 0.3);   // well off perpendicular

    const std::vector<Crossing> found = on_way(located, 10010);
    CHECK_EQ(found.size(), size_t{1});
    if (found.size() != 1) return;

    const Crossing& c = found.front();
    const auto& edge = located.net.graph.edge(c.edge);
    const auto& cl = located.net.centerlines[c.edge];

    // On the carriageway's centreline, near the node the footway shares with it.
    CHECK_TRUE(std::fabs(p5::lateral_of(cl, c.position)) < 0.05);
    CHECK_TRUE(glm::length(c.position - shared) < 4.0);

    // Node 1012 is shared by three ways, so it is a graph node of degree 4 and
    // the crossing is a junction crossing carrying that node.
    const GraphNodeId node = jt::node_with_osm_id(located.net.graph, 1012);
    CHECK_TRUE(node != kInvalidId);
    CHECK_TRUE(c.at_junction);
    CHECK_EQ(c.node, node);
    CHECK_TRUE(edge.from == node || edge.to == node);

    // The axis is the road's normal, not the footway's bearing.
    const auto& station = cl.stations[p5::nearest_station(cl, c.position)];
    CHECK_NEAR(glm::length(c.axis), 1.0, 1e-9);
    CHECK_NEAR(glm::dot(c.axis, station.tangent), 0.0, 1e-6);
}

/**
 * Nothing is attached to a footway edge, and the results come back ordered.
 *
 * The ordering is not cosmetic: find_crossings() promises ascending
 * (edge, arclength) so that a build is reproducible run to run, and a set of
 * zebras that shuffled between runs would make every golden comparison in P7
 * useless.
 */
TEST(Crossings, footway_edges_carry_no_crossing_and_results_are_ordered) {
    const Located located = locate();
    if (!located.ok) return;

    CHECK_EQ(on_way(located, 10020).size(), size_t{0});
    CHECK_EQ(located.crossings.size(), size_t{2});

    for (const Crossing& c : located.crossings) {
        CHECK_TRUE(c.edge != kInvalidId);
        const auto& edge = located.net.graph.edge(c.edge);
        CHECK_TRUE(edge.type != stratum::osm::RoadType::Footway);
        CHECK_TRUE(edge.type != stratum::osm::RoadType::Cycleway);
        CHECK_TRUE(edge.type != stratum::osm::RoadType::Path);
    }

    for (size_t i = 1; i < located.crossings.size(); ++i) {
        const Crossing& a = located.crossings[i - 1];
        const Crossing& b = located.crossings[i];
        CHECK_TRUE(a.edge < b.edge || (a.edge == b.edge && a.arclength <= b.arclength));
    }
}

// ============================================================================
// The zebra
// ============================================================================

/**
 * Every zebra stripe is inside the carriageway, and the pattern is symmetric
 * about the centreline.
 *
 * This is the running-over-the-sidewalk guard. The fixture street carries a
 * sidewalk on both sides, so its total profile is more than 4 m wider than its
 * carriageway; stripes laid out over total_width() would end on the footway at
 * both ends and the failure would read as "the crossing is a bit wide" rather
 * than as a units error.
 *
 * The symmetry assertion is the second half of the same rule. A partial stripe
 * at either end is DROPPED rather than clipped, so the outermost stripe is a
 * full one and the pattern is centred; an implementation that clipped instead
 * would leave a sliver of paint against each kerb.
 */
TEST(Crossings, zebra_stripes_are_clipped_to_the_carriageway) {
    const Located located = locate();
    if (!located.ok) return;

    const std::vector<Crossing> found = on_way(located, 10000);
    if (found.size() != 1) return;
    const Crossing& c = found.front();

    CrossingConfig cfg;
    const Mesh mesh = build_crossing(c, cfg);
    CHECK_TRUE(!mesh.vertices.empty());
    CHECK_TRUE(p5::mesh_is_finite(mesh));
    CHECK_TRUE(p5::mesh_indices_are_sane(mesh));

    const double half = 0.5 * static_cast<double>(c.width);
    const double half_depth = 0.5 * static_cast<double>(cfg.crossing_depth);
    const glm::dvec2 along = perp(c.axis);   // direction of vehicle travel

    const std::vector<p5::Component> comps = p5::components_of(mesh);
    CHECK_TRUE(comps.size() >= 5);

    std::vector<double> centres;
    double widest = 0.0;
    for (const p5::Component& comp : comps) {
        CHECK_EQ(comp.vertices.size(), size_t{4});
        CHECK_EQ(comp.material, MaterialId::Markings);
        CHECK_EQ(p5::sprite_of(mesh, comp), MarkingSprite::ZebraStripe);

        double lo = 1e300;
        double hi = -1e300;
        for (const glm::dvec2& p : p5::locals_of(mesh, comp)) {
            const double lateral = glm::dot(p - c.position, c.axis);
            const double longitudinal = glm::dot(p - c.position, along);

            // The whole point: no stripe vertex may reach past the lane edge.
            CHECK_TRUE(std::fabs(lateral) <= half + 1e-6);
            CHECK_TRUE(std::fabs(longitudinal) <= half_depth + 1e-6);

            lo = std::min(lo, lateral);
            hi = std::max(hi, lateral);
        }
        CHECK_NEAR(hi - lo, cfg.stripe_width, 1e-6);
        centres.push_back(0.5 * (lo + hi));
        widest = std::max(widest, std::fabs(0.5 * (lo + hi)));
    }

    // Every vertex on the crossing's own plane, with no offset of this
    // function's own. The paint plane offset lives in MarkingConfig.
    for (const auto& v : mesh.vertices) {
        CHECK_NEAR(v.position.y, c.height, 1e-5);
    }

    std::sort(centres.begin(), centres.end());
    for (size_t i = 1; i < centres.size(); ++i) {
        CHECK_NEAR(centres[i] - centres[i - 1],
                   static_cast<double>(cfg.stripe_width) + static_cast<double>(cfg.stripe_gap),
                   1e-6);
    }
    // 1e-4 m, not 1e-6: Mesh::vertices stores float32 world positions, and these
    // two centres are a whole carriageway apart, so the shared world offset does
    // not cancel out of the comparison the way it does for the width and spacing
    // checks above. A road 100 m from the local origin therefore carries about
    // 1e-5 m of representation error into this one difference. A pattern that was
    // genuinely off centre would be out by half a gap -- 0.25 m -- not by 0.1 mm.
    CHECK_NEAR(centres.front(), -centres.back(), 1e-4);
    CHECK_TRUE(widest + 0.5 * static_cast<double>(cfg.stripe_width) <= half + 1e-6);
}

/**
 * The zebra switch turns the paint off, and a carriageway too narrow for one
 * full stripe produces nothing rather than a sliver.
 */
/**
 * A zebra reaches the far kerb of a DIVIDED road and stops at the island.
 *
 * RoadProfile::carriageway_width() sums Lane strips alone, but on a divided
 * road the two carriageways are separated by a raised median and its two curb
 * faces, and RoadProfile::left_edge_offset() centres the whole Lane-to-Median
 * SPAN on the centreline. A run fitted to the lane sum therefore stops two
 * metres short of each kerb while painting four stripes over the island, where
 * they sit 150 mm under the island top and inside its curb faces -- invisible,
 * and the paint that should be against the kerb is missing.
 *
 * Both halves are checked here: the outer reach, and the hole in the middle.
 */
TEST(Crossings, a_zebra_crosses_a_divided_road_without_painting_its_island) {
    using stratum::osm::road::ProfileConfig;
    using stratum::osm::road::Strip;
    using stratum::osm::road::StripKind;
    using stratum::osm::road::build_profile;

    // A four-lane primary, which is what turns on the raised median, with a
    // crossing node interior to the way.
    stratum::osm::Road road = jt::make_road(1, {1, 2, 3},
                                            {{0.0, 0.0}, {60.0, 0.0}, {120.0, 0.0}},
                                            stratum::osm::RoadType::Primary);
    road.lanes = 4;
    road.width = 14.0f;

    stratum::osm::ParsedOSMData data = jt::make_data({road});
    stratum::osm::OSMNode crossing_node;
    crossing_node.id = 2;
    crossing_node.tags["highway"] = "crossing";
    data.nodes[2] = crossing_node;

    stratum::osm::road::RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.edges().size(), size_t{1});
    if (graph.edges().size() != 1) return;

    const std::vector<stratum::osm::road::Centerline> centerlines = jt::make_centerlines(graph);
    const stratum::osm::road::RoadProfile profile =
        build_profile(graph.edge(0), ProfileConfig{}, nullptr);
    const std::vector<stratum::osm::road::RoadProfile> profiles{profile};

    // The premise: this profile really does carry a RAISED median.
    double island_lo = 0.0;
    double island_hi = 0.0;
    bool raised = false;
    {
        double lateral = static_cast<double>(profile.left_edge_offset());
        for (const Strip& strip : profile.strips) {
            const double left = lateral;
            const double right = lateral - static_cast<double>(strip.width);
            lateral = right;
            if (strip.kind != StripKind::Median) continue;
            if (std::max(std::fabs(static_cast<double>(strip.height_left)),
                         std::fabs(static_cast<double>(strip.height_right))) <= 0.01) {
                continue;
            }
            island_lo = right;
            island_hi = left;
            raised = true;
        }
    }
    CHECK_TRUE(raised);
    if (!raised) return;

    RoadElevationSolver elevation;
    const CrossingConfig cfg;
    const std::vector<Crossing> found =
        find_crossings(graph, data, centerlines, profiles, elevation, cfg);
    CHECK_EQ(found.size(), size_t{1});
    if (found.size() != 1) return;

    const Crossing& c = found.front();

    // The extent is the carriageway ENVELOPE, wider than the lane sum by the
    // median and the curb faces beside it.
    CHECK_TRUE(static_cast<double>(c.width) >
               static_cast<double>(profile.carriageway_width()) + 2.0);
    CHECK_TRUE(static_cast<double>(c.island_left) >= island_hi - 1e-3);
    CHECK_TRUE(static_cast<double>(c.island_right) <= island_lo + 1e-3);

    const Mesh mesh = build_crossing(c, cfg);
    CHECK_TRUE(!mesh.indices.empty());
    if (mesh.indices.empty()) return;

    // The road runs due east, so a vertex's local y IS its lateral offset.
    double reach = 0.0;
    for (const auto& v : mesh.vertices) {
        const double lateral = jt::world_to_local(v.position).y;
        reach = std::max(reach, std::fabs(lateral));
        const bool on_island = lateral < static_cast<double>(c.island_left) - 1e-6 &&
                               lateral > static_cast<double>(c.island_right) + 1e-6;
        if (on_island) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "no stripe is painted over the raised island",
                                          "lateral " + std::to_string(lateral));
            break;
        }
    }

    // And the run does reach past the lane sum, out towards the real kerb.
    CHECK_TRUE(reach > 0.5 * static_cast<double>(profile.carriageway_width()));
    CHECK_TRUE(reach <= 0.5 * static_cast<double>(c.width) + 1e-6);
}

/**
 * A profile taper is not a junction, and must not move a mid-block crossing.
 *
 * JunctionBuilder gives every degree-2 node whose two profiles differ a taper,
 * and build_profile_taper() writes trims of up to 30 m a side through the very
 * same GraphEdge::trim_from and trim_to a junction uses. A crossing test that
 * reads "inside the trim" as "at a junction" therefore drags a mid-block zebra
 * tens of metres down the road on the most ordinary shape in the data -- a way
 * split where the lane count changes -- and sets it back 1.5 m from a node that
 * has no junction plane and no curb ring.
 */
TEST(Crossings, a_lane_drop_taper_leaves_a_mid_block_crossing_where_it_was_surveyed) {
    using stratum::osm::road::JunctionBuilder;
    using stratum::osm::road::JunctionConfig;
    using stratum::osm::road::ProfileConfig;
    using stratum::osm::road::build_profile;

    // Two ways meeting head to tail at node 2: four lanes into two, which is the
    // way split a lane drop produces. The crossing is 10 m before the join.
    stratum::osm::Road wide = jt::make_road(1, {1, 10, 2},
                                            {{-120.0, 0.0}, {-10.0, 0.0}, {0.0, 0.0}},
                                            stratum::osm::RoadType::Secondary);
    wide.lanes = 4;
    wide.width = 14.0f;

    stratum::osm::Road narrow = jt::make_road(2, {2, 3},
                                              {{0.0, 0.0}, {120.0, 0.0}},
                                              stratum::osm::RoadType::Secondary);
    narrow.lanes = 2;
    narrow.width = 7.0f;

    stratum::osm::ParsedOSMData data = jt::make_data({wide, narrow});
    stratum::osm::OSMNode crossing_node;
    crossing_node.id = 10;
    crossing_node.tags["highway"] = "crossing";
    data.nodes[10] = crossing_node;

    stratum::osm::road::RoadGraph graph;
    graph.build(data);

    const std::vector<stratum::osm::road::Centerline> centerlines = jt::make_centerlines(graph);
    std::vector<stratum::osm::road::RoadProfile> profiles;
    profiles.reserve(graph.edges().size());
    for (const auto& edge : graph.edges()) {
        profiles.push_back(build_profile(edge, ProfileConfig{}, nullptr));
    }

    RoadElevationSolver elevation;
    JunctionBuilder junctions;
    CHECK_TRUE(junctions.solve_trims(graph, centerlines, profiles, elevation, JunctionConfig{}));

    // The premise: the taper really did write a large trim, and the shared node
    // really is a plain degree-2 continuation rather than a junction.
    const GraphNodeId join = jt::node_with_osm_id(graph, NodeId{2});
    CHECK_TRUE(join != kInvalidId);
    if (join == kInvalidId) return;
    CHECK_EQ(graph.node(join).degree(), size_t{2});

    EdgeId wide_edge = kInvalidId;
    for (size_t i = 0; i < graph.edges().size(); ++i) {
        if (graph.edges()[i].source_way == stratum::osm::WayId{1}) {
            wide_edge = static_cast<EdgeId>(i);
        }
    }
    CHECK_TRUE(wide_edge != kInvalidId);
    if (wide_edge == kInvalidId) return;
    CHECK_TRUE(graph.edge(wide_edge).trim_to > 10.0);

    const CrossingConfig cfg;
    const std::vector<Crossing> found =
        find_crossings(graph, data, centerlines, profiles, elevation, cfg);
    CHECK_EQ(found.size(), size_t{1});
    if (found.size() != 1) return;

    const Crossing& c = found.front();
    const stratum::osm::road::Centerline& cl = centerlines[c.edge];
    const double surveyed = cl.stations.back().arclength - 10.0;

    CHECK_FALSE(c.at_junction);
    CHECK_NEAR(c.arclength, surveyed, 0.05);
    CHECK_NEAR(c.position.x, -10.0, 0.05);
}

TEST(Crossings, zebra_can_be_switched_off_and_narrow_crossings_emit_nothing) {
    const Located located = locate();
    if (!located.ok) return;

    const std::vector<Crossing> found = on_way(located, 10000);
    if (found.size() != 1) return;

    CrossingConfig off;
    off.emit_zebra = false;
    CHECK_EQ(build_crossing(found.front(), off).indices.size(), size_t{0});

    Crossing narrow = found.front();
    narrow.width = 0.2f;   // under one stripe_width
    CHECK_EQ(build_crossing(narrow, CrossingConfig{}).indices.size(), size_t{0});
}

// ============================================================================
// Dropped kerb spans
// ============================================================================

/**
 * A junction crossing asks for a drop at both of its ends, and a mid-block one
 * asks for nothing.
 *
 * The spans are directions from the junction centre rather than points, so the
 * assertions are angular: each span is a non-empty counter-clockwise interval of
 * unit vectors, and the two spans of one crossing point roughly along the
 * crossing's own axis in opposite directions, because that is where the crossing
 * meets the kerb line on either side of the road.
 */
TEST(Crossings, a_junction_crossing_drops_the_kerb_at_both_of_its_ends) {
    const Located located = locate();
    if (!located.ok) return;

    const std::vector<Crossing> found = on_way(located, 10010);
    if (found.size() != 1) return;
    const Crossing& c = found.front();

    const GraphNodeId node = jt::node_with_osm_id(located.net.graph, 1012);
    if (node == kInvalidId) return;
    const glm::dvec2 centre = located.net.graph.node(node).position;

    CrossingConfig cfg;
    const std::vector<DroppedKerbSpan> spans =
        dropped_kerb_spans(located.crossings, node, centre, cfg);

    CHECK_TRUE(!spans.empty());
    CHECK_TRUE(spans.size() <= 2);

    std::vector<glm::dvec2> mids;
    for (const DroppedKerbSpan& span : spans) {
        CHECK_NEAR(glm::length(span.from), 1.0, 1e-9);
        CHECK_NEAR(glm::length(span.to), 1.0, 1e-9);
        CHECK_TRUE(glm::length(span.from - span.to) > 1e-9);   // never an empty span
        CHECK_NEAR(span.height, cfg.dropped_kerb_height, 1e-9);

        const glm::dvec2 sum = span.from + span.to;
        if (glm::length(sum) > 1e-9) mids.push_back(glm::normalize(sum));
    }

    // Each drop sits where the crossing axis meets the kerb, so it points across
    // the road rather than along it.
    for (const glm::dvec2& mid : mids) {
        CHECK_TRUE(std::fabs(glm::dot(mid, c.axis)) > 0.5);
    }
    if (mids.size() == 2) {
        CHECK_TRUE(glm::dot(mids[0], mids[1]) < 0.0);   // opposite kerbs
    }
}

/**
 * Nothing is dropped where nothing asked for it.
 *
 * Three negatives, each of which a permissive implementation would get wrong in
 * a way that shows as a kerb line missing for no reason: the switch off, a node
 * no crossing reaches, and a mid-block crossing offered to a junction that is
 * nowhere near it.
 */
TEST(Crossings, no_crossing_means_no_dropped_kerb) {
    const Located located = locate();
    if (!located.ok) return;

    const GraphNodeId node = jt::node_with_osm_id(located.net.graph, 1012);
    if (node == kInvalidId) return;
    const glm::dvec2 centre = located.net.graph.node(node).position;

    CrossingConfig off;
    off.emit_dropped_kerbs = false;
    CHECK_EQ(dropped_kerb_spans(located.crossings, node, centre, off).size(), size_t{0});

    CHECK_EQ(dropped_kerb_spans({}, node, centre, CrossingConfig{}).size(), size_t{0});

    // Only the mid-block crossing from way 10000, offered to the junction on way
    // 10010. It is not at_junction and its node is kInvalidId, so it must not
    // reach this ring.
    const std::vector<Crossing> mid_block = on_way(located, 10000);
    CHECK_EQ(dropped_kerb_spans(mid_block, node, centre, CrossingConfig{}).size(), size_t{0});
}

/**
 * A driveway drops the kerb where it meets its parent road.
 *
 * tests/data/service_roads.osm hangs a `service=driveway`, a
 * `service=parking_aisle` and a `service=alley` off one residential street at
 * three interior nodes. Each is a place a car crosses the footway, so each needs
 * a flare in the ring either side of its mouth; a signalled crossroads of
 * primaries needs none.
 */
TEST(Crossings, driveway_arms_drop_the_kerb_and_ordinary_junctions_do_not) {
    const p5::Network service = p5::make_network("service_roads.osm");
    if (!service.ok) return;

    CrossingConfig cfg;
    for (NodeId osm_node : {NodeId{1402}, NodeId{1403}, NodeId{1404}}) {
        const GraphNodeId node = jt::node_with_osm_id(service.graph, osm_node);
        CHECK_TRUE(node != kInvalidId);
        if (node == kInvalidId) continue;
        CHECK_EQ(service.graph.node(node).degree(), size_t{3});

        const glm::dvec2 centre = service.graph.node(node).position;
        const std::vector<DroppedKerbSpan> spans =
            driveway_kerb_spans(service.graph, service.data, node, centre, service.profiles, cfg);
        CHECK_TRUE(!spans.empty());
        for (const DroppedKerbSpan& span : spans) {
            CHECK_NEAR(glm::length(span.from), 1.0, 1e-9);
            CHECK_NEAR(glm::length(span.to), 1.0, 1e-9);
            CHECK_NEAR(span.height, cfg.dropped_kerb_height, 1e-9);
        }
    }

    const p5::Network signalled = p5::make_network("turn_lanes.osm");
    if (!signalled.ok) return;
    const GraphNodeId cross = jt::node_with_osm_id(signalled.graph, 1203);
    if (cross == kInvalidId) return;
    CHECK_EQ(driveway_kerb_spans(signalled.graph, signalled.data, cross,
                                 signalled.graph.node(cross).position, signalled.profiles,
                                 CrossingConfig{})
                 .size(),
             size_t{0});
}

// ============================================================================
// The kerb ring the spans are applied to
// ============================================================================

namespace {

/// Highest and lowest world Y over the MaterialId::Curb triangles of a ring
void curb_height_range(const Mesh& mesh, double& out_low, double& out_high) {
    out_low = p5::min_height_of_material(mesh, MaterialId::Curb);
    out_high = p5::max_height_of_material(mesh, MaterialId::Curb);
}

/// Every world Y appearing on a MaterialId::Curb vertex, sorted and deduplicated
std::vector<double> curb_heights(const Mesh& mesh) {
    std::vector<double> out;
    const size_t tri_count = mesh.indices.size() / 3;
    std::vector<MaterialId> per_triangle(tri_count, MaterialId::Default);
    for (const stratum::SubMesh& sub : mesh.effective_submeshes()) {
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset) + sub.index_count) / 3u;
        for (size_t t = first; t < last && t < tri_count; ++t) per_triangle[t] = sub.material;
    }
    for (size_t t = 0; t < tri_count; ++t) {
        if (per_triangle[t] != MaterialId::Curb) continue;
        for (int k = 0; k < 3; ++k) {
            out.push_back(static_cast<double>(mesh.vertices[mesh.indices[t * 3 + k]].position.y));
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end(),
                          [](double a, double b) { return std::fabs(a - b) < 1e-6; }),
              out.end());
    return out;
}

/// No undirected edge of the mesh is shared by more than two triangles
bool is_edge_manifold(const Mesh& mesh) {
    std::vector<std::pair<uint64_t, int>> counts;
    const auto key = [](uint32_t a, uint32_t b) {
        const uint32_t lo = std::min(a, b);
        const uint32_t hi = std::max(a, b);
        return (static_cast<uint64_t>(lo) << 32) | hi;
    };
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        for (int k = 0; k < 3; ++k) {
            const uint64_t e = key(mesh.indices[t + k], mesh.indices[t + (k + 1) % 3]);
            auto it = std::find_if(counts.begin(), counts.end(),
                                   [e](const std::pair<uint64_t, int>& p) { return p.first == e; });
            if (it == counts.end()) {
                counts.emplace_back(e, 1);
            } else if (++it->second > 2) {
                return false;
            }
        }
    }
    return true;
}

} // namespace

/**
 * Applying a kerb drop lowers the kerb to the lip, ramps into it, and leaves the
 * ring whole.
 *
 * Built on a symmetric four-way cross with a hand-made drop spanning one
 * quadrant, so the ring is known to be a full-height ring everywhere else and
 * the comparison is against the same ring built without the drop.
 *
 * Four things have to hold at once, and each of them is a different way the
 * feature fails:
 *
 * - The kerb reaches the lip. A drop that stops halfway is a step a wheel still
 *   cannot climb.
 * - The kerb does not go BELOW the lip, and never above full height. The lip is
 *   deliberately not zero: a flush kerb is coplanar with the carriageway and
 *   z-fights.
 * - Intermediate heights exist. That is what distinguishes a ramp from a step,
 *   and it is only true if the ring was RESAMPLED across the ramp: a ramp that
 *   fell between two existing columns reads as a vertical tear.
 * - The mesh stays edge-manifold and free of degenerate triangles, which is the
 *   "no vertical gap between adjacent ring columns" property stated in the form
 *   a test can check without knowing the tessellation.
 */
TEST(Crossings, a_dropped_kerb_ramps_to_the_lip_and_leaves_the_ring_closed) {
    const jt::Fixture fixture = jt::symmetric_cross(2, 200.0);
    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 4);
    CHECK_TRUE(node != kInvalidId);
    if (node == kInvalidId) return;

    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    CHECK_TRUE(jt::solve_node(fixture, node, TrimConfig{}, arms, ends));

    const JunctionPolygon poly = build_junction_polygon(arms, ends, FilletConfig{});
    CHECK_TRUE(poly.valid);
    if (!poly.valid) return;

    CrossingConfig crossing_cfg;
    CurbRingConfig ring_cfg;
    const float height = 0.0f;

    const CurbRing plain = build_curb_ring(poly, arms, ends, height, ring_cfg, nullptr);
    CHECK_TRUE(plain.valid);
    if (!plain.valid) return;

    // A drop in the middle of the north-east corner section.
    //
    // The span AND BOTH ITS RAMPS have to land on kerb, or the ramp this test is
    // about falls in an arm mouth where there is no ring to carry it and the
    // corner reads as a single flat lip. On this fixture the north-east section
    // runs from about 21 to about 69 degrees off the junction centre -- the arms
    // leaving east and north eat the rest -- and the ring sits at a radius of
    // roughly 7 to 11 m, so a 1 m ramp is about 6 degrees of arc. A span of 40 to
    // 50 degrees therefore ramps over 34-40 and 50-56 and still leaves full-height
    // kerb at each end of the section. Widening it past about 27-63 degrees pushes
    // the ramps into the mouths and stops testing anything.
    KerbDrops drops;
    drops.center = fixture.graph.node(node).position;
    drops.ramp_length = static_cast<double>(crossing_cfg.dropped_kerb_ramp);
    constexpr double kDegree = 3.14159265358979323846 / 180.0;
    const double span_from = 40.0 * kDegree;
    const double span_to = 50.0 * kDegree;
    DroppedKerbSpan span;
    span.from = glm::dvec2{std::cos(span_from), std::sin(span_from)};
    span.to = glm::dvec2{std::cos(span_to), std::sin(span_to)};
    span.height = crossing_cfg.dropped_kerb_height;
    drops.spans.push_back(span);

    const CurbRing dropped = build_curb_ring(poly, arms, ends, height, ring_cfg, &drops);
    CHECK_TRUE(dropped.valid);
    if (!dropped.valid) return;

    CHECK_TRUE(p5::mesh_is_finite(dropped.mesh));
    CHECK_TRUE(p5::mesh_indices_are_sane(dropped.mesh));
    CHECK_TRUE(p5::no_degenerate_triangles(dropped.mesh, 1e-10));
    CHECK_TRUE(is_edge_manifold(dropped.mesh));

    double plain_low = 0.0;
    double plain_high = 0.0;
    curb_height_range(plain.mesh, plain_low, plain_high);
    CHECK_NEAR(plain_high - plain_low, ring_cfg.curb_height, 1e-3);

    double low = 0.0;
    double high = 0.0;
    curb_height_range(dropped.mesh, low, high);

    // Full height survives away from the span, and the lip is reached inside it.
    CHECK_NEAR(high, plain_high, 1e-3);
    CHECK_NEAR(low, plain_low, 1e-3);

    const double lip = static_cast<double>(height) + static_cast<double>(span.height);
    const double full = static_cast<double>(height) + ring_cfg.curb_height;

    const std::vector<double> heights = curb_heights(dropped.mesh);
    bool at_lip = false;
    bool at_full = false;
    bool intermediate = false;
    for (double h : heights) {
        if (h < static_cast<double>(height) - 1e-3) {
            stratum::test::report_failure(__FILE__, __LINE__, "curb never sinks below the road",
                                          "height " + std::to_string(h));
        }
        CHECK_TRUE(h <= full + 1e-3);
        if (std::fabs(h - lip) < 1e-3) at_lip = true;
        if (std::fabs(h - full) < 1e-3) at_full = true;
        if (h > lip + 1e-3 && h < full - 1e-3) intermediate = true;
    }
    CHECK_TRUE(at_lip);
    CHECK_TRUE(at_full);
    CHECK_TRUE(intermediate);

    // The ramp needs columns to carry it, so the dropped ring is at least as
    // finely tessellated as the plain one.
    CHECK_TRUE(dropped.mesh.vertices.size() >= plain.mesh.vertices.size());
}

/**
 * The span a junction crossing asks for lands on kerb the ring actually carries.
 *
 * build_curb_ring() emits sections from one arm's carriage corner round the
 * fillet to the next arm's corner, and leaves every arm MOUTH open -- the sector
 * between that arm's own two corners has no ring in it at all. A junction
 * crossing stands CrossingConfig::setback OUTSIDE its arm's cut line, at half the
 * carriageway laterally, so its kerb points are at a larger along-arm distance
 * and the same lateral as the corner: strictly inside the mouth, for every arm of
 * every junction. A span laid there asks for a drop where there is nothing to
 * drop, and the ring comes back at full height with a pedestrian facing a 150 mm
 * wall.
 *
 * This is the end-to-end assertion: real spans from dropped_kerb_spans(), fed to
 * the real build_curb_ring(), have to reach the lip somewhere. Nothing in the
 * suite did that before -- the ramp test above hand-picks a span it knows lands
 * on kerb, which is exactly the property under test here.
 */
TEST(Crossings, a_junction_crossing_drops_kerb_the_ring_actually_carries) {
    const jt::Fixture fixture = jt::symmetric_cross(2, 200.0);
    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 4);
    CHECK_TRUE(node != kInvalidId);
    if (node == kInvalidId) return;

    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    CHECK_TRUE(jt::solve_node(fixture, node, TrimConfig{}, arms, ends));

    const JunctionPolygon poly = build_junction_polygon(arms, ends, FilletConfig{});
    CHECK_TRUE(poly.valid);
    if (!poly.valid) return;

    const glm::dvec2 centre = fixture.graph.node(node).position;
    const CrossingConfig cfg;

    // One crossing per arm, placed exactly where find_crossings() places a
    // junction crossing: `setback` metres outside the arm's own cut line, square
    // to the road, spanning that arm's carriageway.
    std::vector<Crossing> crossings;
    for (const ArmRef& arm : arms) {
        const glm::dvec2 dir(std::cos(arm.bearing), std::sin(arm.bearing));

        Crossing c;
        c.node = node;
        c.edge = arm.edge;
        c.at_junction = true;
        c.arclength = arm.trim + static_cast<double>(cfg.setback);
        c.position = centre + dir * (arm.trim + static_cast<double>(cfg.setback));
        c.axis = glm::dvec2(-dir.y, dir.x);
        c.width = fixture.profiles[arm.edge].carriageway_width();
        crossings.push_back(c);
    }
    CHECK_EQ(crossings.size(), size_t{4});
    if (crossings.size() != 4) return;

    const std::vector<DroppedKerbSpan> spans =
        dropped_kerb_spans(crossings, node, centre, cfg);
    CHECK_TRUE(!spans.empty());
    if (spans.empty()) return;

    CurbRingConfig ring_cfg;
    const float height = 0.0f;

    KerbDrops drops;
    drops.center = centre;
    drops.ramp_length = static_cast<double>(cfg.dropped_kerb_ramp);
    drops.spans = spans;

    const CurbRing dropped = build_curb_ring(poly, arms, ends, height, ring_cfg, &drops);
    CHECK_TRUE(dropped.valid);
    if (!dropped.valid) return;

    CHECK_TRUE(p5::mesh_is_finite(dropped.mesh));
    CHECK_TRUE(p5::no_degenerate_triangles(dropped.mesh, 1e-10));
    CHECK_TRUE(is_edge_manifold(dropped.mesh));

    const double lip = static_cast<double>(height) + static_cast<double>(cfg.dropped_kerb_height);
    const double full = static_cast<double>(height) + ring_cfg.curb_height;

    bool at_lip = false;
    bool at_full = false;
    for (double h : curb_heights(dropped.mesh)) {
        CHECK_TRUE(h <= full + 1e-3);
        if (std::fabs(h - lip) < 1e-3) at_lip = true;
        if (std::fabs(h - full) < 1e-3) at_full = true;
    }

    // The drop is delivered, and the rest of the ring keeps its full kerb.
    CHECK_TRUE(at_lip);
    CHECK_TRUE(at_full);
}

/**
 * An empty drop list reproduces the undropped ring exactly.
 *
 * The switch-off property every P5 pass carries: passing a KerbDrops with no
 * span, and passing nullptr, must give the same geometry, so a build with
 * crossings disabled is byte-identical to a build without the feature.
 */
TEST(Crossings, an_empty_drop_list_reproduces_the_plain_ring) {
    const jt::Fixture fixture = jt::symmetric_cross(2, 200.0);
    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 4);
    if (node == kInvalidId) return;

    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    if (!jt::solve_node(fixture, node, TrimConfig{}, arms, ends)) return;

    const JunctionPolygon poly = build_junction_polygon(arms, ends, FilletConfig{});
    if (!poly.valid) return;

    const CurbRing plain = build_curb_ring(poly, arms, ends, 0.0f, CurbRingConfig{}, nullptr);

    KerbDrops empty;
    empty.center = fixture.graph.node(node).position;
    CHECK_FALSE(empty.any());
    const CurbRing same = build_curb_ring(poly, arms, ends, 0.0f, CurbRingConfig{}, &empty);

    CHECK_EQ(same.mesh.vertices.size(), plain.mesh.vertices.size());
    CHECK_EQ(same.mesh.indices.size(), plain.mesh.indices.size());
    for (size_t i = 0; i < same.mesh.vertices.size() && i < plain.mesh.vertices.size(); ++i) {
        CHECK_NEAR(same.mesh.vertices[i].position.x, plain.mesh.vertices[i].position.x, 1e-9);
        CHECK_NEAR(same.mesh.vertices[i].position.y, plain.mesh.vertices[i].position.y, 1e-9);
        CHECK_NEAR(same.mesh.vertices[i].position.z, plain.mesh.vertices[i].position.z, 1e-9);
    }
}
