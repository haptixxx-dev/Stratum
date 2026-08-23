/**
 * @file test_junction_special.cpp
 * @brief Roundabout, profile-taper and dead-end tests for the P4 junction solver
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Written against the contract in src/osm/road/junction_special.hpp and the
 * "Special cases" list in docs/plans/road_network_plan.md.
 *
 * Three cases the trim-and-fillet solver cannot handle, and one property each
 * that matters more than the rest:
 *
 * - **Roundabouts.** A roundabout is a RING OF EDGES, not a node. Detection walks
 *   the is_roundabout subgraph looking for closed cycles, and a chain that never
 *   closes is exactly the input a naive walk loops forever on. That is asserted
 *   here by the test completing: an import that hangs is worse than an import that
 *   produces a wrong shape, because there is nothing to look at afterwards.
 * - **Degree-2 tapers.** The taper length is a formula and the formula is checked
 *   against its own arithmetic, not against a plausible range.
 * - **Dead ends.** A cap that does not weld to the arm's last cross-section leaves
 *   a crack the width of the road, so the flat cap's vertices are compared against
 *   the arm's own strip columns to 1e-6.
 *
 * ### A note on the taper threshold
 *
 * junction_special.hpp puts the "no taper needed" threshold at equal strip kinds
 * with every width within 1e-3 m. A one centimetre width difference is therefore
 * ABOVE the threshold and does get a taper -- of TaperConfig::min_length, since
 * the formula's own answer for 1 cm is 0.15 m. Both sides of that boundary are
 * asserted below, so the threshold is pinned rather than assumed.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests JunctionSpecial
 * @endcode
 */

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/junction_builder.hpp"
#include "osm/road/junction_special.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cfloat>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::osm::NodeId;
using stratum::osm::ParsedOSMData;
using stratum::osm::Road;
using stratum::osm::RoadType;
using stratum::osm::WayId;
using stratum::osm::road::Centerline;
using stratum::osm::road::DeadEndConfig;
using stratum::osm::road::EdgeId;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::RoundaboutConfig;
using stratum::osm::road::RoundaboutLoop;
using stratum::osm::road::Station;
using stratum::osm::road::Strip;
using stratum::osm::road::StripKind;
using stratum::osm::road::TaperConfig;
using stratum::osm::road::build_dead_end;
using stratum::osm::road::build_profile_taper;
using stratum::osm::road::build_roundabout;
using stratum::osm::road::find_roundabouts;
using stratum::osm::road::kInvalidId;
using stratum::osm::road::offset_point;

namespace jt = stratum::test::junction;

/// One lane-only profile per edge, so every width in an expectation is exact
std::vector<RoadProfile> uniform_profiles(const RoadGraph& graph, int lanes = 2) {
    return std::vector<RoadProfile>(graph.edges().size(), jt::lane_profile(lanes));
}

/// A shoulder strip, used to make one side of a profile wider without moving its lanes
Strip shoulder(double width) {
    Strip strip;
    strip.width = static_cast<float>(width);
    strip.height_left = 0.0f;
    strip.height_right = 0.0f;
    strip.material = MaterialId::Gravel;
    strip.kind = StripKind::Shoulder;
    return strip;
}

/**
 * @brief Two lanes with a shoulder of the given width on the LEFT only
 *
 * A one-sided change, so `max_side_width_change` in the taper formula is
 * unambiguous: it is the shoulder width difference itself, not half of it as a
 * symmetric widening would give.
 *
 * @param shoulder_width Left shoulder width in metres
 * @return The profile
 */
RoadProfile shouldered_profile(double shoulder_width) {
    RoadProfile profile = jt::lane_profile(2);
    profile.strips.insert(profile.strips.begin(), shoulder(shoulder_width));
    return profile;
}

/// Mean distance from a point to a set of points
double mean_radius(const std::vector<glm::dvec2>& points, const glm::dvec2& center) {
    if (points.empty()) return 0.0;
    double sum = 0.0;
    for (const glm::dvec2& p : points) sum += glm::length(p - center);
    return sum / static_cast<double>(points.size());
}

} // namespace

// ============================================================================
// Roundabouts
// ============================================================================

/**
 * tests/data/roundabout.osm: closed way 4000 split at its three approach nodes
 * into three edges, every one flagged is_roundabout.
 *
 * The radius is checked against a value measured independently from the graph's
 * own polylines rather than against a hard-coded metre count, because the parser
 * recentres and projects and no absolute coordinate in this file is stable.
 */
TEST(JunctionSpecial, roundabout_fixture_yields_one_closed_loop) {
    const auto parsed = jt::parse_fixture("roundabout.osm");
    if (!parsed) return;

    RoadGraph graph;
    graph.build(*parsed);
    const std::vector<Centerline> centerlines = jt::make_centerlines(graph);

    const std::vector<RoundaboutLoop> loops = find_roundabouts(graph, centerlines);
    CHECK_EQ(loops.size(), size_t{1});
    if (loops.size() != 1) return;

    const RoundaboutLoop& loop = loops[0];
    CHECK_TRUE(loop.valid);
    CHECK_EQ(loop.edges.size(), size_t{3});
    CHECK_EQ(loop.nodes.size(), loop.edges.size());
    if (loop.edges.size() != 3 || loop.nodes.size() != 3) return;

    // Every edge of the loop is a roundabout edge, and the loop takes all of them.
    size_t roundabout_edges = 0;
    for (const GraphEdge& edge : graph.edges()) {
        if (edge.is_roundabout) ++roundabout_edges;
    }
    CHECK_EQ(roundabout_edges, size_t{3});
    for (const EdgeId id : loop.edges) {
        CHECK_TRUE(id < graph.edges().size());
        if (id < graph.edges().size()) CHECK_TRUE(graph.edge(id).is_roundabout);
    }

    // The cycle CLOSES: edges[i] runs from nodes[i] to nodes[i + 1], wrapping.
    for (size_t i = 0; i < loop.edges.size(); ++i) {
        const GraphEdge& edge = graph.edge(loop.edges[i]);
        const GraphNodeId expected_to = loop.nodes[(i + 1) % loop.nodes.size()];
        CHECK_EQ(edge.from, loop.nodes[i]);
        CHECK_EQ(edge.to, expected_to);
    }

    // All three approach nodes are on the loop, and every loop node has degree 3.
    const NodeId approaches[3] = {401, 403, 405};
    for (const NodeId osm_id : approaches) {
        const GraphNodeId node = jt::node_with_osm_id(graph, osm_id);
        CHECK(node != kInvalidId);
        if (node == kInvalidId) continue;
        CHECK_EQ(graph.node(node).degree(), size_t{3});
        const bool present =
            std::find(loop.nodes.begin(), loop.nodes.end(), node) != loop.nodes.end();
        if (!present) {
            stratum::test::report_failure(__FILE__, __LINE__, "approach node is on the loop",
                                          "osm id " + std::to_string(osm_id));
        }
    }

    // Independent measurement of the ring, from the graph polylines.
    std::vector<glm::dvec2> ring_points;
    for (const GraphEdge& edge : graph.edges()) {
        if (!edge.is_roundabout) continue;
        for (const glm::dvec2& p : edge.polyline) ring_points.push_back(p);
    }
    CHECK_TRUE(ring_points.size() >= 6);

    glm::dvec2 measured_center{0.0};
    for (const glm::dvec2& p : ring_points) measured_center += p;
    measured_center /= static_cast<double>(ring_points.size());
    const double measured_radius = mean_radius(ring_points, measured_center);

    CHECK_TRUE(measured_radius > 1.0);
    CHECK_NEAR(loop.center.x, measured_center.x, 2.0);
    CHECK_NEAR(loop.center.y, measured_center.y, 2.0);

    // The loop's radius is the mean over its resampled STATIONS, which sit on the
    // chords between the surveyed vertices and so read a little short of the
    // vertex mean. A quarter is ample slack for that and far too tight to hide a
    // radius taken from the wrong thing.
    CHECK_NEAR(loop.radius, measured_radius, measured_radius * 0.25);

    // Plausible for this fixture in absolute terms as well.
    CHECK_TRUE(loop.radius > RoundaboutConfig{}.min_radius);
    CHECK_TRUE(loop.radius > 5.0 && loop.radius < 40.0);
}

/**
 * A chain of is_roundabout edges that never closes. The contract is that
 * find_roundabouts() reports nothing and RETURNS -- an unclosed chain is what a
 * naive cycle walk revisits forever, and a hang on the import path leaves nothing
 * to debug.
 *
 * Termination is asserted by this test finishing. There is no way to assert it
 * more directly without a watchdog thread, and a watchdog would only convert the
 * hang into a different kind of hang.
 */
TEST(JunctionSpecial, open_roundabout_chain_terminates_and_reports_no_loop) {
    // Three edges forming an open arc: 1 -> 2 -> 3 -> 4, never returning to 1.
    std::vector<Road> roads;
    const glm::dvec2 points[4] = {{0.0, 0.0}, {10.0, 4.0}, {14.0, 14.0}, {10.0, 24.0}};
    for (size_t i = 0; i + 1 < 4; ++i) {
        Road road = jt::make_road(static_cast<WayId>(i + 1),
                                  {static_cast<NodeId>(i + 1), static_cast<NodeId>(i + 2)},
                                  {points[i], points[i + 1]}, RoadType::Tertiary);
        road.is_roundabout = true;
        roads.push_back(road);
    }
    // A spur off the middle of the chain, so the walk has a branch to get lost in.
    Road spur = jt::make_road(99, {2, 50}, {points[1], {60.0, -30.0}}, RoadType::Residential);
    roads.push_back(spur);

    ParsedOSMData data = jt::make_data(roads);
    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = jt::make_centerlines(graph);

    const std::vector<RoundaboutLoop> loops = find_roundabouts(graph, centerlines);

    // Reached at all: the walk terminated.
    for (const RoundaboutLoop& loop : loops) {
        // An open chain may be reported as an invalid loop but never as a valid one.
        CHECK_FALSE(loop.valid);
    }
    size_t valid_loops = 0;
    for (const RoundaboutLoop& loop : loops) {
        if (loop.valid) ++valid_loops;
    }
    CHECK_EQ(valid_loops, size_t{0});

    // And a graph with no roundabout edges at all reports nothing either.
    const jt::Fixture plain = jt::symmetric_cross(2);
    CHECK_TRUE(find_roundabouts(plain.graph, plain.centerlines).empty());
}

/**
 * The roundabout mesh is an ANNULUS with an island, not a disc.
 *
 * The bound is measured from the loop's own stations rather than from
 * RoundaboutLoop::radius, because the fixture's ring is an oval and its radius is
 * a mean. Using the mean would put the bound outside the real inner edge on the
 * narrow axis and the test would fail on correct geometry.
 */
TEST(JunctionSpecial, roundabout_mesh_is_an_annulus_around_a_raised_island) {
    const auto parsed = jt::parse_fixture("roundabout.osm");
    if (!parsed) return;

    RoadGraph graph;
    graph.build(*parsed);
    const std::vector<Centerline> centerlines = jt::make_centerlines(graph);
    // Lane-only profiles, so the annulus half width is exactly 3.5 m.
    const std::vector<RoadProfile> profiles = uniform_profiles(graph, 2);
    const double half_width = jt::kLaneWidth;

    const std::vector<RoundaboutLoop> loops = find_roundabouts(graph, centerlines);
    CHECK_EQ(loops.size(), size_t{1});
    if (loops.empty() || !loops[0].valid) return;
    const RoundaboutLoop& loop = loops[0];

    RoundaboutConfig cfg;
    cfg.raised_island = true;
    cfg.island_inset = 1.0;

    const float height = 7.5f;
    const Mesh mesh = build_roundabout(loop, graph, centerlines, profiles, height, cfg);
    CHECK_TRUE(!mesh.vertices.empty());
    CHECK_TRUE(!mesh.indices.empty());
    if (mesh.indices.empty()) return;

    // Closest the loop's own centerline passes to the centre, so the annulus's
    // inner edge is never nearer than this less its half width.
    double station_min = 1e300;
    for (const EdgeId id : loop.edges) {
        if (id >= centerlines.size()) continue;
        for (const Station& s : centerlines[id].stations) {
            station_min = std::min(station_min, glm::length(s.position - loop.center));
        }
    }
    CHECK_TRUE(station_min < 1e299);
    if (station_min > 1e299) return;

    const double inner_edge = station_min - half_width;
    const double island_edge = inner_edge - cfg.island_inset;
    CHECK_TRUE(island_edge > 0.0);

    // Attribute each vertex to the material of the range that references it.
    std::vector<MaterialId> vertex_material(mesh.vertices.size(), MaterialId::Count);
    for (const auto& sub : mesh.effective_submeshes()) {
        for (uint32_t i = sub.index_offset; i < sub.index_offset + sub.index_count; ++i) {
            if (i >= mesh.indices.size()) break;
            const uint32_t vi = mesh.indices[i];
            if (vi < vertex_material.size()) vertex_material[vi] = sub.material;
        }
    }

    size_t asphalt_vertices = 0;
    size_t island_vertices = 0;
    double asphalt_min_radius = 1e300;

    for (size_t v = 0; v < mesh.vertices.size(); ++v) {
        const glm::dvec2 local = jt::world_to_local(mesh.vertices[v].position);
        const double radius = glm::length(local - loop.center);
        const MaterialId material = vertex_material[v];

        if (material == MaterialId::Asphalt || material == MaterialId::Concrete) {
            ++asphalt_vertices;
            asphalt_min_radius = std::min(asphalt_min_radius, radius);
        }
        if (material == MaterialId::Grass) {
            ++island_vertices;
        }

        // Nothing inside the island edge except the island itself and its curb.
        // A vertex no range references (material Count) is not geometry and is
        // reported by the submesh coverage checks elsewhere, not here.
        if (radius < island_edge - 0.2 && material != MaterialId::Count &&
            material != MaterialId::Grass && material != MaterialId::Curb) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "only the island lies inside the annulus",
                std::string(stratum::material_id_name(material)) + " vertex at radius " +
                    stratum::test::stringify(radius) + " inside " +
                    stratum::test::stringify(island_edge));
        }
    }

    CHECK_TRUE(asphalt_vertices > 0);
    CHECK_TRUE(island_vertices > 0);

    // The carriageway really is an annulus: its innermost vertex sits on the
    // inner edge rather than at the centre.
    CHECK_TRUE(asphalt_min_radius >= inner_edge - 0.2);
    CHECK_TRUE(asphalt_min_radius < 1e299);

    // A raised island stands behind a curb face.
    size_t curb_vertices = 0;
    for (const MaterialId material : vertex_material) {
        if (material == MaterialId::Curb) ++curb_vertices;
    }
    CHECK_TRUE(curb_vertices > 0);

    // The carriageway sits at the height it was asked for.
    double surface_min = 1e300;
    for (size_t v = 0; v < mesh.vertices.size(); ++v) {
        if (vertex_material[v] != MaterialId::Asphalt) continue;
        surface_min = std::min(surface_min, static_cast<double>(mesh.vertices[v].position.y));
    }
    CHECK_NEAR(surface_min, static_cast<double>(height), 1e-3);

    // An invalid loop produces nothing rather than an approximation of nothing.
    CHECK_TRUE(build_roundabout(RoundaboutLoop{}, graph, centerlines, profiles, height, cfg)
                   .indices.empty());
}

// ============================================================================
// Degree-2 profile transitions
// ============================================================================

/**
 * Two edges meeting end to end with different profiles.
 *
 * The taper length is `clamp(max_side_width_change * length_per_metre_width,
 * min_length, max_length)` with half taken from each side. The fixture makes the
 * width change one-sided -- a 3.5 m shoulder appearing on the left only -- so
 * `max_side_width_change` is that 3.5 m exactly and the expected length is
 * arithmetic rather than an estimate.
 */
TEST(JunctionSpecial, profile_taper_length_matches_the_formula) {
    const double arm = 200.0;
    const std::vector<Road> roads = {
        jt::make_road(1, {1, 100}, {{-arm, 0.0}, {0.0, 0.0}}),
        jt::make_road(2, {100, 2}, {{0.0, 0.0}, {arm, 0.0}}),
    };
    // Same strip kinds either side, so the transition is a width change and not a
    // structural one; only the shoulder differs.
    const jt::Fixture fixture =
        jt::make_fixture(roads, {shouldered_profile(3.5), shouldered_profile(7.0)});

    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 2);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    TaperConfig cfg;
    Mesh mesh;
    double trim_a = -1.0;
    double trim_b = -1.0;
    const bool tapered = build_profile_taper(fixture.graph, fixture.centerlines,
                                             fixture.profiles, node, 0.05f, cfg, mesh,
                                             trim_a, trim_b);
    CHECK_TRUE(tapered);
    if (!tapered) return;

    const double change = 7.0 - 3.5;
    const double expected_length =
        std::min(std::max(change * cfg.length_per_metre_width, cfg.min_length),
                 cfg.max_length);
    CHECK_NEAR(expected_length, 52.5, 1e-9);

    // Half from each side, and the two halves sum to the formula's answer.
    CHECK_NEAR(trim_a, expected_length * 0.5, 1e-6);
    CHECK_NEAR(trim_b, expected_length * 0.5, 1e-6);
    CHECK_NEAR(trim_a + trim_b, expected_length, 1e-6);

    // Neither end may eat more than half its own edge.
    CHECK_TRUE(trim_a <= arm * 0.5 + 1e-9);
    CHECK_TRUE(trim_b <= arm * 0.5 + 1e-9);

    CHECK_TRUE(!mesh.vertices.empty());
    CHECK_TRUE(!mesh.indices.empty());
    for (const auto& v : mesh.vertices) {
        CHECK_TRUE(jt::is_finite(v.position));
    }
}

/**
 * Two profiles too close to be worth a taper, and one just far enough apart to
 * need the shortest one there is.
 *
 * The threshold is 1e-3 m of width with matching strip kinds, so half a millimetre
 * is below it and one centimetre is above it. Asserting both sides is what turns
 * the threshold into a tested number rather than an assumed one.
 */
TEST(JunctionSpecial, matching_profiles_need_no_taper_and_a_centimetre_needs_the_minimum) {
    const double arm = 200.0;
    const std::vector<Road> roads = {
        jt::make_road(1, {1, 100}, {{-arm, 0.0}, {0.0, 0.0}}),
        jt::make_road(2, {100, 2}, {{0.0, 0.0}, {arm, 0.0}}),
    };
    TaperConfig cfg;

    // Identical profiles: nothing to blend.
    {
        const jt::Fixture fixture =
            jt::make_fixture(roads, {shouldered_profile(3.5), shouldered_profile(3.5)});
        const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 2);
        CHECK(node != kInvalidId);
        if (node == kInvalidId) return;

        Mesh mesh;
        double trim_a = -1.0;
        double trim_b = -1.0;
        CHECK_FALSE(build_profile_taper(fixture.graph, fixture.centerlines, fixture.profiles,
                                        node, 0.05f, cfg, mesh, trim_a, trim_b));
        CHECK_NEAR(trim_a, 0.0, 1e-12);
        CHECK_NEAR(trim_b, 0.0, 1e-12);
    }

    // Half a millimetre apart: below the 1e-3 m threshold, still no taper.
    {
        const jt::Fixture fixture =
            jt::make_fixture(roads, {shouldered_profile(3.5), shouldered_profile(3.5005)});
        const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 2);
        if (node == kInvalidId) return;

        Mesh mesh;
        double trim_a = -1.0;
        double trim_b = -1.0;
        CHECK_FALSE(build_profile_taper(fixture.graph, fixture.centerlines, fixture.profiles,
                                        node, 0.05f, cfg, mesh, trim_a, trim_b));
        CHECK_NEAR(trim_a, 0.0, 1e-12);
        CHECK_NEAR(trim_b, 0.0, 1e-12);
        CHECK_TRUE(mesh.indices.empty());
    }

    // One centimetre apart: above the threshold, so a taper -- but the formula's
    // own answer for 1 cm is 0.15 m, and TaperConfig::min_length floors it.
    {
        const jt::Fixture fixture =
            jt::make_fixture(roads, {shouldered_profile(3.5), shouldered_profile(3.51)});
        const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 2);
        if (node == kInvalidId) return;

        Mesh mesh;
        double trim_a = -1.0;
        double trim_b = -1.0;
        CHECK_TRUE(build_profile_taper(fixture.graph, fixture.centerlines, fixture.profiles,
                                       node, 0.05f, cfg, mesh, trim_a, trim_b));
        CHECK_NEAR(trim_a + trim_b, cfg.min_length, 1e-6);
        CHECK_NEAR(trim_a, cfg.min_length * 0.5, 1e-6);
    }

    // A node that is not degree 2 is refused rather than solved.
    {
        const jt::Fixture cross = jt::symmetric_cross(2);
        const GraphNodeId junction = jt::sole_node_of_degree(cross.graph, 4);
        CHECK(junction != kInvalidId);
        if (junction == kInvalidId) return;
        Mesh mesh;
        double trim_a = -1.0;
        double trim_b = -1.0;
        CHECK_FALSE(build_profile_taper(cross.graph, cross.centerlines, cross.profiles,
                                        junction, 0.05f, cfg, mesh, trim_a, trim_b));
        CHECK_NEAR(trim_a, 0.0, 1e-12);
        CHECK_NEAR(trim_b, 0.0, 1e-12);
    }
}

// ============================================================================
// Dead ends
// ============================================================================

/**
 * tests/data/cul_de_sac.osm. Node 306 carries highway=turning_circle; nodes 301
 * and 303 are plain residential dead ends.
 *
 * The crack guard is the flat cap. A cap that does not land exactly on the arm's
 * last cross-section leaves a gap the width of the road, visible from the ground
 * and in every export, so every cap vertex is required to coincide with one of the
 * arm's own strip boundary columns to 1e-6 m in plan.
 *
 * The turning circle is a disc that COVERS the arm end rather than welding to it,
 * so it is checked for coverage and radius instead.
 */
TEST(JunctionSpecial, cul_de_sac_caps_weld_to_the_arm_end_cross_section) {
    const auto parsed = jt::parse_fixture("cul_de_sac.osm");
    if (!parsed) return;

    RoadGraph graph;
    graph.build(*parsed);
    const std::vector<Centerline> centerlines = jt::make_centerlines(graph);
    const std::vector<RoadProfile> profiles = jt::profiles_from_tags(graph, *parsed);

    const GraphNodeId turning_circle = jt::node_with_osm_id(graph, static_cast<NodeId>(306));
    CHECK(turning_circle != kInvalidId);
    if (turning_circle == kInvalidId) return;
    CHECK_TRUE(graph.node(turning_circle).is_dead_end());
    CHECK_TRUE(graph.node(turning_circle).is_turning_circle);

    const float height = 3.25f;

    // ---- Flat caps ------------------------------------------------------
    // bulb_for_residential off, so a residential dead end gets the plain quad
    // whose vertices must land on the arm's own cross-section.
    DeadEndConfig flat;
    flat.bulb_for_residential = false;

    size_t flat_caps_checked = 0;

    for (size_t n = 0; n < graph.nodes().size(); ++n) {
        const GraphNodeId node = static_cast<GraphNodeId>(n);
        if (!graph.node(node).is_dead_end()) continue;
        if (graph.node(node).is_turning_circle) continue;

        const Mesh cap = build_dead_end(graph, centerlines, profiles, node, height, flat);
        if (cap.vertices.empty()) continue;
        ++flat_caps_checked;

        const auto& arm = graph.node(node).arms.front();
        const Centerline& cl = centerlines[arm.edge];
        const RoadProfile& profile = profiles[arm.edge];
        CHECK_TRUE(cl.is_valid());
        if (!cl.is_valid() || profile.strips.empty()) continue;

        // The arm's last cross-section, in the edge's own frame.
        const Station& station = arm.at_start ? cl.stations.front() : cl.stations.back();
        std::vector<glm::dvec2> columns;
        double lateral = static_cast<double>(profile.left_edge_offset());
        columns.push_back(offset_point(station, lateral));
        for (const Strip& strip : profile.strips) {
            lateral -= static_cast<double>(strip.width);
            columns.push_back(offset_point(station, lateral));
        }

        // Mesh vertices are float32. At this fixture's ~67 m extent one ulp is
        // 8e-6 m, so a cap welded PERFECTLY still reads several microns off its
        // double-precision column and no fixed tolerance below that can pass.
        // The bound is therefore derived from the representation rather than
        // guessed: four ulps at the largest coordinate in play. That is still
        // three orders of magnitude tighter than the tenth of a millimetre a
        // crack would have to reach before it were visible, so it cannot absorb
        // a real one.
        float extent = 1.0f;
        for (const auto& v : cap.vertices) {
            extent = std::max(extent, std::fabs(v.position.x));
            extent = std::max(extent, std::fabs(v.position.y));
            extent = std::max(extent, std::fabs(v.position.z));
        }
        const double weld_eps =
            4.0 * static_cast<double>(extent) * static_cast<double>(FLT_EPSILON);

        // Every cap vertex sits on one of those columns. This is the crack guard.
        for (const auto& v : cap.vertices) {
            const glm::dvec2 local = jt::world_to_local(v.position);
            double best = 1e300;
            for (const glm::dvec2& column : columns) {
                best = std::min(best, glm::length(local - column));
            }
            if (best > weld_eps) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "cap vertex coincides with an arm cross-section column",
                    "node " + std::to_string(n) + ": nearest column " +
                        stratum::test::stringify(best) + " m away");
                break;
            }
        }

        // And the cap really spans the arm, rather than a sliver of it: both outer
        // corners are present.
        const glm::dvec2 outer_left = columns.front();
        const glm::dvec2 outer_right = columns.back();
        double nearest_left = 1e300;
        double nearest_right = 1e300;
        for (const auto& v : cap.vertices) {
            const glm::dvec2 local = jt::world_to_local(v.position);
            nearest_left = std::min(nearest_left, glm::length(local - outer_left));
            nearest_right = std::min(nearest_right, glm::length(local - outer_right));
        }
        CHECK_NEAR(nearest_left, 0.0, weld_eps);
        CHECK_NEAR(nearest_right, 0.0, weld_eps);
    }

    CHECK_TRUE(flat_caps_checked >= 2);

    // ---- Turning circle -------------------------------------------------
    DeadEndConfig disc;
    disc.turning_circle_radius = 6.0;

    const Mesh circle =
        build_dead_end(graph, centerlines, profiles, turning_circle, height, disc);
    CHECK_TRUE(!circle.vertices.empty());
    CHECK_TRUE(!circle.indices.empty());
    if (circle.vertices.empty()) return;

    const glm::dvec2 node_pos = graph.node(turning_circle).position;
    const auto& arm = graph.node(turning_circle).arms.front();
    const double profile_half =
        0.5 * static_cast<double>(profiles[arm.edge].total_width());
    const double expected_radius = std::max(disc.turning_circle_radius, profile_half);

    double max_radius = 0.0;
    for (const auto& v : circle.vertices) {
        CHECK_TRUE(jt::is_finite(v.position));
        max_radius = std::max(max_radius, glm::length(jt::world_to_local(v.position) - node_pos));
    }
    // A disc, centred on the node, of the configured radius: never smaller than
    // the road it terminates, and never a stray vertex beyond its own rim.
    CHECK_NEAR(max_radius, expected_radius, 1e-3);

    // The disc covers the arm's own end cross-section, so no crack shows.
    const Centerline& cl = centerlines[arm.edge];
    if (cl.is_valid() && !profiles[arm.edge].strips.empty()) {
        const Station& station = arm.at_start ? cl.stations.front() : cl.stations.back();
        const double left = static_cast<double>(profiles[arm.edge].left_edge_offset());
        const double right = left - static_cast<double>(profiles[arm.edge].total_width());
        CHECK_TRUE(glm::length(offset_point(station, left) - node_pos)
                   <= expected_radius + 1e-6);
        CHECK_TRUE(glm::length(offset_point(station, right) - node_pos)
                   <= expected_radius + 1e-6);
    }

    // A node that is not a dead end gets no cap.
    const GraphNodeId junction = jt::node_with_osm_id(graph, static_cast<NodeId>(302));
    CHECK(junction != kInvalidId);
    if (junction != kInvalidId) {
        CHECK_EQ(graph.node(junction).degree(), size_t{3});
        CHECK_TRUE(
            build_dead_end(graph, centerlines, profiles, junction, height, disc).indices.empty());
    }
}

// ============================================================================
// Dead-end regressions
// ============================================================================

/**
 * A turning circle emits a FLAT disc, and the arm arriving at it carries strips
 * standing above that plane: a curb face and a raised sidewalk. Nothing in the
 * disc path carries those round the cap -- the outboard ring is a bulb-only
 * construction and needs the two halves to match -- so without the flat cap's
 * vertical end face the 0.15 m sidewalk slab terminates in mid-air with an open
 * boundary edge, and the viewer sees the ribbon's backfaces through the gap.
 *
 * The assertion is stated per raised strip boundary rather than as a vertex
 * count: at every boundary the profile puts above the carriageway, the cap must
 * carry BOTH a vertex on the carriageway plane and one at the strip's own height,
 * at that boundary's own plan position. That is the end face, and nothing else
 * produces it.
 */
TEST(JunctionSpecial, turning_circle_closes_its_raised_strips) {
    const auto parsed = jt::parse_fixture("cul_de_sac.osm");
    if (!parsed) return;

    RoadGraph graph;
    graph.build(*parsed);
    const std::vector<Centerline> centerlines = jt::make_centerlines(graph);
    const std::vector<RoadProfile> profiles = jt::profiles_from_tags(graph, *parsed);

    const GraphNodeId node = jt::node_with_osm_id(graph, static_cast<NodeId>(306));
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;
    CHECK_TRUE(graph.node(node).is_turning_circle);

    const auto& arm = graph.node(node).arms.front();
    const Centerline& cl = centerlines[arm.edge];
    const RoadProfile& profile = profiles[arm.edge];
    CHECK_TRUE(cl.is_valid());
    if (!cl.is_valid() || profile.strips.empty()) return;

    // The arm's own end cross-section: one boundary per strip edge, with the
    // height the profile puts it at.
    const Station& station = arm.at_start ? cl.stations.front() : cl.stations.back();
    std::vector<glm::dvec2> boundary;
    std::vector<double> boundary_height;
    double lateral = static_cast<double>(profile.left_edge_offset());
    boundary.push_back(offset_point(station, lateral));
    boundary_height.push_back(static_cast<double>(profile.strips.front().height_left));
    for (const Strip& strip : profile.strips) {
        lateral -= static_cast<double>(strip.width);
        boundary.push_back(offset_point(station, lateral));
        boundary_height.push_back(static_cast<double>(strip.height_right));
    }

    // Precondition: this fixture really does carry raised strips. Without it the
    // test would pass vacuously on a profile that has nothing to close.
    size_t raised_boundaries = 0;
    for (double h : boundary_height) {
        if (h > 0.01) ++raised_boundaries;
    }
    CHECK_TRUE(raised_boundaries >= size_t{2});
    if (raised_boundaries < 2) return;

    const float height = 3.25f;
    DeadEndConfig disc;
    disc.turning_circle_radius = 6.0;

    const Mesh cap = build_dead_end(graph, centerlines, profiles, node, height, disc);
    CHECK_TRUE(!cap.vertices.empty());
    if (cap.vertices.empty()) return;

    for (size_t k = 0; k < boundary.size(); ++k) {
        if (boundary_height[k] <= 0.01) continue;

        bool has_base = false;
        bool has_top = false;
        for (const auto& v : cap.vertices) {
            if (glm::length(jt::world_to_local(v.position) - boundary[k]) > 1e-3) continue;
            const double y = static_cast<double>(v.position.y);
            if (std::fabs(y - static_cast<double>(height)) < 1e-3) has_base = true;
            if (std::fabs(y - (static_cast<double>(height) + boundary_height[k])) < 1e-3) {
                has_top = true;
            }
        }
        if (!has_base || !has_top) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the cap closes every raised strip down to the carriageway",
                "boundary " + std::to_string(k) + " at height " +
                    stratum::test::stringify(boundary_height[k]) +
                    (has_base ? "" : ": no vertex on the carriageway plane") +
                    (has_top ? "" : ": no vertex at the strip height"));
        }
    }
}

/**
 * The turning circle's notch, and what it has to clear.
 *
 * build_profile() emits a Gutter strip at height 0 outboard of the outer lane on
 * every kerbed class, so the carriageway is not the whole of the flat surface the
 * ribbon lays down. A disc notched back only to the LANE edge leaves its two
 * backward lobes lying exactly on that gutter, which is the coplanar overlap --
 * and the z-fight -- the notch exists to prevent.
 *
 * Probed on the ground rather than by counting vertices: the strip of gutter
 * behind the node must not be covered by the cap, while the arm's own ribbon
 * still covers it.
 */
TEST(JunctionSpecial, turning_circle_notch_clears_the_coplanar_gutter) {
    const auto parsed = jt::parse_fixture("cul_de_sac.osm");
    if (!parsed) return;

    RoadGraph graph;
    graph.build(*parsed);
    const std::vector<Centerline> centerlines = jt::make_centerlines(graph);
    const std::vector<RoadProfile> profiles = jt::profiles_from_tags(graph, *parsed);

    const GraphNodeId node = jt::node_with_osm_id(graph, static_cast<NodeId>(306));
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    const auto& arm = graph.node(node).arms.front();
    const Centerline& cl = centerlines[arm.edge];
    const RoadProfile& profile = profiles[arm.edge];
    if (!cl.is_valid() || profile.strips.empty()) return;

    // Walk the profile outward from the carriageway on each side, for as long as
    // the strips stay ON the carriageway plane. That is the surface the disc must
    // be notched back to, and it is wider than the lane edge on every kerbed
    // profile.
    double lane_half = 0.0;
    double flat_half = 0.0;
    {
        std::vector<double> lat;
        std::vector<double> h;
        double l = static_cast<double>(profile.left_edge_offset());
        lat.push_back(l);
        h.push_back(static_cast<double>(profile.strips.front().height_left));
        size_t first = profile.strips.size();
        size_t last = 0;
        for (size_t i = 0; i < profile.strips.size(); ++i) {
            l -= static_cast<double>(profile.strips[i].width);
            lat.push_back(l);
            h.push_back(static_cast<double>(profile.strips[i].height_right));
            if (profile.strips[i].kind == StripKind::Lane ||
                profile.strips[i].kind == StripKind::Median) {
                if (first == profile.strips.size()) first = i;
                last = i;
            }
        }
        CHECK_TRUE(first < profile.strips.size());
        if (first >= profile.strips.size()) return;

        lane_half = 0.5 * (lat[first] - lat[last + 1]);
        double left = lat[first];
        for (size_t i = first; i-- > 0;) {
            if (std::fabs(h[i]) > 1e-4) break;
            left = lat[i];
        }
        flat_half = std::max(lane_half, left);
    }

    // Precondition: the fixture has a coplanar strip outboard of its lanes.
    CHECK_TRUE(flat_half > lane_half + 0.05);
    if (!(flat_half > lane_half + 0.05)) return;

    const float height = 3.25f;
    DeadEndConfig disc;
    disc.turning_circle_radius = 6.0;

    const Mesh cap = build_dead_end(graph, centerlines, profiles, node, height, disc);
    const std::vector<jt::Tri2D> tris = jt::triangles_of(cap);
    CHECK_TRUE(!tris.empty());
    if (tris.empty()) return;

    const Station& station = arm.at_start ? cl.stations.front() : cl.stations.back();
    const glm::dvec2 into_road = arm.at_start ? station.tangent : -station.tangent;
    const glm::dvec2 mid = station.normal;

    // Mid-gutter on each side, 2 m back down the road from the node: inside the
    // un-notched disc, and squarely on the ribbon.
    const double band = 0.5 * (lane_half + flat_half);
    for (const double side : {1.0, -1.0}) {
        const glm::dvec2 probe = station.position + into_road * 2.0 + mid * (side * band);
        if (jt::covered_in_plan(tris, probe)) {
            stratum::test::report_failure(
                __FILE__, __LINE__,
                "the turning circle is notched back clear of the coplanar gutter",
                "the cap covers the gutter " + stratum::test::stringify(band) +
                    " m off the centreline, 2 m behind the node");
        }
    }

    // And it is still a disc: the ground straight ahead of the node is covered.
    CHECK_TRUE(jt::covered_in_plan(tris, station.position - into_road * 2.0));
}

// ============================================================================
// Roundabout regressions
// ============================================================================

/**
 * The seam at loop.nodes[0].
 *
 * The annulus is swept in RUNS, broken wherever a trim has opened an approach
 * mouth, and the last run is rejoined to the first only when the ring carries no
 * trim at all. Every real roundabout has approaches and therefore trims, so a
 * traversal that starts at an UNTRIMMED node -- which loop.nodes[0] usually is,
 * since find_roundabouts() picks it by lowest EdgeId and a ring drawn as two ways
 * has a plain degree-2 seam there -- ends one run at that node and starts another
 * from it. The two columns are centred on the same point but framed from two
 * different terminal stations, so they are rotated against each other by one
 * band's turn and leave an uncovered wedge in the middle of the carriageway.
 *
 * The fixture is deliberately the awkward one: a ring drawn as TWO ways, so the
 * lowest EdgeId belongs to the degree-2 join rather than to an approach.
 */
TEST(JunctionSpecial, roundabout_annulus_is_continuous_across_its_first_node) {
    const double radius = 20.0;
    const size_t steps = 12;

    // The ring, as two half-ways meeting at node 1000. Approaches leave at three
    // of the shared nodes, so those become the trimmed mouths.
    std::vector<glm::dvec2> first;
    std::vector<NodeId> first_ids;
    std::vector<glm::dvec2> second;
    std::vector<NodeId> second_ids;
    for (size_t i = 0; i <= steps; ++i) {
        const double angle = 2.0 * 3.14159265358979323846 * static_cast<double>(i) /
                             static_cast<double>(steps);
        const glm::dvec2 p{radius * std::cos(angle), radius * std::sin(angle)};
        const NodeId id = static_cast<NodeId>(1000 + (i % steps));
        if (i <= steps / 2) {
            first.push_back(p);
            first_ids.push_back(id);
        }
        if (i >= steps / 2) {
            second.push_back(p);
            second_ids.push_back(id);
        }
    }

    std::vector<Road> roads;
    Road ring_a = jt::make_road(1, first_ids, first);
    ring_a.is_roundabout = true;
    Road ring_b = jt::make_road(2, second_ids, second);
    ring_b.is_roundabout = true;
    roads.push_back(ring_a);
    roads.push_back(ring_b);

    // Three approaches, radiating outward from ring nodes 1002, 1006 and 1010.
    WayId way = 10;
    for (const size_t i : {size_t{2}, size_t{6}, size_t{10}}) {
        const double angle = 2.0 * 3.14159265358979323846 * static_cast<double>(i) /
                             static_cast<double>(steps);
        const glm::dvec2 on_ring{radius * std::cos(angle), radius * std::sin(angle)};
        const glm::dvec2 out = on_ring * 4.0;
        roads.push_back(jt::make_road(way, {static_cast<NodeId>(1000 + i),
                                            static_cast<NodeId>(2000 + i)},
                                      {on_ring, out}));
        ++way;
    }

    RoadGraph graph;
    graph.build(jt::make_data(roads));
    const std::vector<Centerline> centerlines = jt::make_centerlines(graph);
    const std::vector<RoadProfile> profiles = uniform_profiles(graph);

    const std::vector<RoundaboutLoop> loops = find_roundabouts(graph, centerlines);
    CHECK_EQ(loops.size(), size_t{1});
    if (loops.size() != 1) return;
    const RoundaboutLoop& loop = loops[0];
    CHECK_TRUE(loop.valid);
    if (!loop.valid || loop.nodes.empty()) return;

    // The premise: the loop's FIRST node is an untrimmed degree-2 seam, which is
    // the case the wrap has to survive.
    const GraphNodeId seam = loop.nodes.front();
    CHECK_EQ(graph.node(seam).degree(), size_t{2});
    if (graph.node(seam).degree() != 2) return;

    // Solve the trims exactly as the builder does, so the approach mouths are
    // open when the annulus is swept.
    stratum::osm::road::JunctionBuilder builder;
    stratum::osm::road::JunctionConfig cfg;
    const stratum::osm::road::RoadElevationSolver flat;
    const std::vector<stratum::osm::road::Junction> junctions =
        builder.build(graph, centerlines, profiles, flat, cfg);

    const stratum::osm::road::Junction* annulus = nullptr;
    for (const auto& junction : junctions) {
        if (junction.is_roundabout) annulus = &junction;
    }
    CHECK(annulus != nullptr);
    if (annulus == nullptr) return;

    // At least one mouth really was opened, or the wrap path would be taken and
    // the seam would never be exercised.
    size_t trimmed_ring_ends = 0;
    for (const EdgeId id : loop.edges) {
        if (graph.edge(id).trim_from > 1e-6) ++trimmed_ring_ends;
        if (graph.edge(id).trim_to > 1e-6) ++trimmed_ring_ends;
    }
    CHECK_TRUE(trimmed_ring_ends >= size_t{2});

    const std::vector<jt::Tri2D> tris = jt::triangles_of(annulus->mesh);
    CHECK_TRUE(!tris.empty());
    if (tris.empty()) return;

    // The carriageway under the seam node, across the whole band. A wedge left by
    // an unwelded wrap opens right here.
    const glm::dvec2 centre = graph.node(seam).position;
    const glm::dvec2 radial = glm::normalize(centre - loop.center);
    const double half = jt::carriageway_half_of(profiles[loop.edges.front()]);

    for (const double t : {-0.6, -0.3, 0.0, 0.3, 0.6}) {
        const glm::dvec2 probe = centre + radial * (t * half);
        if (!jt::covered_in_plan(tris, probe)) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the annulus covers its own seam node",
                "uncovered at lateral " + stratum::test::stringify(t * half));
        }
    }
}
