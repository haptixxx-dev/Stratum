/**
 * @file test_road_elevation.cpp
 * @brief Vertical solve tests: grade limits, node agreement, bridges, tunnels, convergence
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * These tests are written against the contract in src/osm/road/road_elevation.hpp.
 *
 * Every surface here is SYNTHETIC. The solver reaches the terrain only through a
 * HeightSampler callback, which is the whole reason nothing under src/osm/road
 * may include a procgen header: a plane, a ramp, a gorge and a single
 * spike are enough to pin the solver down, and none of them needs a terrain
 * generator in the build. A test that generated procedural terrain would also be
 * testing the noise function, and would move every expected number whenever that
 * function changed.
 *
 * Two of these matter more than the rest.
 *
 * **Node agreement** is the failure this phase must never ship. If one arm of a
 * junction terminates a few centimetres above another, the junction is torn: the
 * carriageways cross at different heights and the gap is visible from the ground
 * and from every export. It is asserted here to 1e-6 m, not to a visual
 * tolerance, because there is no reason for the numbers to differ at all -- the
 * arms are pinned to one shared node height by construction.
 *
 * **The spike test** is the rollercoaster case the whole solve exists to
 * prevent. Sampling terrain per station and using the answer directly makes a
 * road follow metre-scale procedural noise. The assertion is that a 40 m spike
 * two metres wide moves the road by a bounded amount and leaves its gradient
 * inside the class limit.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadElevation
 * @endcode
 */

#include "framework.hpp"

#include "osm/parser.hpp"
#include "osm/road/centerline.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"

#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#ifndef STRATUM_TEST_DATA_DIR
#error "STRATUM_TEST_DATA_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

namespace {

using stratum::osm::NodeId;
using stratum::osm::ParsedOSMData;
using stratum::osm::Road;
using stratum::osm::RoadType;
using stratum::osm::WayId;
using stratum::osm::road::Arm;
using stratum::osm::road::Centerline;
using stratum::osm::road::EdgeElevation;
using stratum::osm::road::EdgeId;
using stratum::osm::road::ElevationConfig;
using stratum::osm::road::GraphNode;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::HeightSampler;
using stratum::osm::road::ResampleConfig;
using stratum::osm::road::RoadElevationSolver;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::Station;
using stratum::osm::road::build_centerline;
using stratum::osm::road::max_grade_for;

/**
 * @brief Tolerance for "the grade limit was respected"
 *
 * The limit is a hard constraint, so this absorbs float accumulation in the
 * relaxation and nothing else. A solver that overshoots by a millimetre per
 * metre is not respecting the limit.
 */
constexpr double kGradeEps = 1e-3;

/**
 * @brief Slack on the vertical curvature bound
 *
 * ElevationConfig::max_grade_change_per_m bounds the discrete second derivative
 * of the profile with respect to arc length. A relaxation solved to a finite
 * epsilon does not hit that bound exactly at the two pinned ends, where the
 * boundary condition and the curvature limit pull against each other, so the
 * assertion allows a few times the nominal bound. It is still far tighter than a
 * kink: a road that flips from the full up-grade to the full down-grade across
 * one station gap would need twice the class limit in one step and fails this.
 */
constexpr double kCurvatureSlack = 4.0;

/// Node heights and station heights are pinned to each other; they must not differ at all
constexpr double kAgreementEps = 1e-6;

// ============================================================================
// Synthetic network construction
// ============================================================================

/**
 * @brief Build one Road with topology attached
 *
 * @param way_id   OSM way ID
 * @param node_ids Node IDs, parallel to @p points
 * @param points   Centerline in 2D local metres
 * @param type     Road classification, which selects the grade limit
 * @return The road, ready to put in ParsedOSMData::roads
 */
Road make_road(WayId way_id,
               const std::vector<NodeId>& node_ids,
               const std::vector<glm::dvec2>& points,
               RoadType type) {
    Road road;
    road.osm_id = way_id;
    road.polyline = points;
    road.node_ids = node_ids;
    road.type = type;
    road.lanes = 2;
    road.width = 7.0f;
    return road;
}

/// Wrap roads into the ParsedOSMData the graph builder expects
ParsedOSMData make_data(std::vector<Road> roads) {
    ParsedOSMData data;
    data.roads = std::move(roads);
    data.stats.processed_roads = data.roads.size();
    return data;
}

/**
 * @brief One centerline per graph edge, in EdgeId order
 *
 * The solver requires this vector to be exactly parallel to graph.edges(), so it
 * is built the same way RoadNetworkBuilder builds it. Smoothing is off: these
 * tests reason about arc length along a straight line, and a fitted spline would
 * put the stations somewhere slightly else for no benefit.
 *
 * @param graph Built road graph
 * @return Centerlines indexed by EdgeId
 */
std::vector<Centerline> make_centerlines(const RoadGraph& graph) {
    ResampleConfig cfg;
    cfg.smooth = false;

    std::vector<Centerline> out;
    out.reserve(graph.edges().size());
    for (const auto& edge : graph.edges()) {
        out.push_back(build_centerline(edge.polyline, cfg));
    }
    return out;
}

/// Absolute path of a fixture in tests/data
std::filesystem::path fixture_path(const char* filename) {
    return std::filesystem::path(STRATUM_TEST_DATA_DIR) / filename;
}

/**
 * @brief Parse one fixture with roads only
 *
 * @param filename Fixture file name in tests/data
 * @return Parsed data, or std::nullopt when the parse failed
 */
std::optional<ParsedOSMData> parse_fixture(const char* filename) {
    const auto path = fixture_path(filename);
    if (!std::filesystem::exists(path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture exists",
                                      "missing: " + path.string());
        return std::nullopt;
    }

    stratum::osm::OSMParser parser;
    stratum::osm::ParserConfig config;
    config.import_buildings = false;
    config.import_water = false;
    config.import_landuse = false;
    config.import_natural = false;
    config.simplify_geometry = false;
    parser.set_config(config);

    if (!parser.parse(path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "parser.parse(fixture)",
                                      path.string() + ": " + parser.get_error());
        return std::nullopt;
    }
    return parser.take_data();
}

// ============================================================================
// Synthetic surfaces
// ============================================================================

/// Perfectly flat surface at 12.5 m
float flat_surface(double, double) {
    return 12.5f;
}

/// Constant 20% up-grade along +X, far steeper than any class limit in the table
float ramp_surface(double x, double) {
    return static_cast<float>(0.20 * x);
}

/**
 * @brief Flat ground at 10 m carrying one 40 m spike about 4 m wide at x = 150
 *
 * The rollercoaster case. Following this surface literally would put a 40 m
 * gradient change into four metres of road.
 */
float spike_surface(double x, double) {
    const double d = x - 150.0;
    return static_cast<float>(10.0 + 40.0 * std::exp(-(d * d) / (2.0 * 1.5 * 1.5)));
}

/**
 * @brief Plateau at 20 m cut by a flat-bottomed gorge centred on x = 450
 *
 * 20 m outside |x - 450| >= 40, falling to 0 m inside |x - 450| <= 20. A bridge
 * abutted on the plateau spans a hole 20 m deep, so a deck that followed the
 * terrain would dive into it.
 */
float gorge_surface(double x, double) {
    const double d = std::fabs(x - 450.0);
    if (d >= 40.0) return 20.0f;
    if (d <= 20.0) return 0.0f;
    return static_cast<float>(20.0 * (d - 20.0) / 20.0);
}

/**
 * @brief Ground at 10 m carrying a flat-topped hill 30 m high centred on x = 450
 *
 * The inverse of gorge_surface: a tunnel bored through it must sit well below
 * the surface at mid-span.
 */
float hill_surface(double x, double) {
    const double d = std::fabs(x - 450.0);
    if (d >= 30.0) return 10.0f;
    if (d <= 15.0) return 40.0f;
    return static_cast<float>(40.0 - 30.0 * (d - 15.0) / 15.0);
}

/**
 * @brief Deterministic pseudo-noise with metre-scale detail
 *
 * A sum of incommensurate sines rather than a random field, so the surface is
 * identical on every run and on every platform, which is what lets the
 * determinism test compare bit patterns.
 */
float noisy_surface(double x, double y) {
    return static_cast<float>(30.0
                              + 5.0 * std::sin(0.31 * x)
                              + 4.0 * std::cos(0.27 * y)
                              + 3.0 * std::sin(0.11 * x + 0.19 * y)
                              + 2.0 * std::sin(0.73 * x) * std::cos(0.61 * y));
}

/// Gentle tilted plane with a slow undulation, always inside every class limit
float tilted_surface(double x, double y) {
    return static_cast<float>(20.0 + 0.05 * x - 0.03 * y
                              + 2.0 * std::sin(0.05 * x) * std::cos(0.04 * y));
}

// ============================================================================
// Profile measurement
// ============================================================================

/**
 * @brief Steepest gradient in a solved profile, rise over run
 *
 * Station gaps of zero arc length are skipped: a bevelled joint is represented
 * as two stations sharing one arclength, and the gradient across it is not
 * defined.
 *
 * @param cl      Centerline the heights belong to
 * @param heights Solved station heights, parallel to cl.stations
 * @return Largest absolute gradient, or 0 when no gap has length
 */
double max_gradient(const Centerline& cl, const std::vector<float>& heights) {
    if (heights.size() != cl.stations.size() || heights.size() < 2) return 0.0;

    double worst = 0.0;
    for (size_t i = 0; i + 1 < heights.size(); ++i) {
        const double ds = cl.stations[i + 1].arclength - cl.stations[i].arclength;
        if (ds <= 1e-9) continue;
        const double grade = (static_cast<double>(heights[i + 1])
                              - static_cast<double>(heights[i])) / ds;
        worst = std::max(worst, std::fabs(grade));
    }
    return worst;
}

/**
 * @brief Largest change in gradient per metre travelled
 *
 * The discrete second derivative of the profile with respect to arc length,
 * which is what ElevationConfig::max_grade_change_per_m bounds.
 *
 * @param cl      Centerline the heights belong to
 * @param heights Solved station heights, parallel to cl.stations
 * @return Largest absolute gradient change per metre, or 0 when undefined
 */
double max_grade_change_per_metre(const Centerline& cl, const std::vector<float>& heights) {
    if (heights.size() != cl.stations.size() || heights.size() < 3) return 0.0;

    double worst = 0.0;
    double prev_grade = 0.0;
    double prev_ds = 0.0;
    bool have_prev = false;

    for (size_t i = 0; i + 1 < heights.size(); ++i) {
        const double ds = cl.stations[i + 1].arclength - cl.stations[i].arclength;
        if (ds <= 1e-9) continue;
        const double grade = (static_cast<double>(heights[i + 1])
                              - static_cast<double>(heights[i])) / ds;
        if (have_prev) {
            const double span = 0.5 * (ds + prev_ds);
            if (span > 1e-9) {
                worst = std::max(worst, std::fabs(grade - prev_grade) / span);
            }
        }
        prev_grade = grade;
        prev_ds = ds;
        have_prev = true;
    }
    return worst;
}

/// True when every solved height is a finite number
bool all_finite(const std::vector<float>& heights) {
    for (float h : heights) {
        if (!std::isfinite(h)) return false;
    }
    return true;
}

/**
 * @brief Height where an arm terminates at its node
 *
 * @param elev Solved profile of the arm's edge
 * @param arm  The arm
 * @return First station height when the arm starts at the node, last otherwise
 */
float arm_end_height(const EdgeElevation& elev, const Arm& arm) {
    return arm.at_start ? elev.station_heights.front() : elev.station_heights.back();
}

/**
 * @brief The junction plane an arm end must land on
 *
 * RoadElevationSolver::node_height() is the CARVED TERRAIN height at the node
 * and deliberately excludes ElevationConfig::surface_offset, while
 * EdgeElevation::station_heights includes it. Every comparison between the two
 * therefore goes through this function, so no assertion below can forget the
 * offset and quietly pass on a junction that is one offset out.
 *
 * @param solver Solved solver
 * @param cfg    Config the solve ran with
 * @param id     Node to read
 * @return Road surface height at the node
 */
double node_surface_height(const RoadElevationSolver& solver,
                           const ElevationConfig& cfg,
                           GraphNodeId id) {
    return static_cast<double>(solver.node_height(id)) + static_cast<double>(cfg.surface_offset);
}

} // namespace

// ============================================================================
// Grade limit table
// ============================================================================

TEST(RoadElevation, max_grade_for_maps_every_class_to_its_limit) {
    ElevationConfig cfg;

    CHECK_NEAR(max_grade_for(RoadType::Motorway, cfg), cfg.max_grade_motorway, 1e-9);
    CHECK_NEAR(max_grade_for(RoadType::Trunk, cfg), cfg.max_grade_trunk, 1e-9);
    CHECK_NEAR(max_grade_for(RoadType::Primary, cfg), cfg.max_grade_primary, 1e-9);
    CHECK_NEAR(max_grade_for(RoadType::Secondary, cfg), cfg.max_grade_secondary, 1e-9);
    CHECK_NEAR(max_grade_for(RoadType::Tertiary, cfg), cfg.max_grade_tertiary, 1e-9);
    CHECK_NEAR(max_grade_for(RoadType::Residential, cfg), cfg.max_grade_residential, 1e-9);
    CHECK_NEAR(max_grade_for(RoadType::Service, cfg), cfg.max_grade_service, 1e-9);
    CHECK_NEAR(max_grade_for(RoadType::Path, cfg), cfg.max_grade_path, 1e-9);

    // Footway and Cycleway share the path limit; Unknown takes the residential one.
    CHECK_NEAR(max_grade_for(RoadType::Footway, cfg), cfg.max_grade_path, 1e-9);
    CHECK_NEAR(max_grade_for(RoadType::Cycleway, cfg), cfg.max_grade_path, 1e-9);
    CHECK_NEAR(max_grade_for(RoadType::Unknown, cfg), cfg.max_grade_residential, 1e-9);
}

// ============================================================================
// Flat surface
// ============================================================================

TEST(RoadElevation, flat_surface_needs_no_relaxation) {
    ParsedOSMData data = make_data({
        make_road(1, {1, 2}, {{0.0, 0.0}, {200.0, 0.0}}, RoadType::Residential),
    });

    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = make_centerlines(graph);

    CHECK_EQ(graph.edges().size(), size_t{1});
    if (graph.edges().empty()) return;

    ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines, HeightSampler{flat_surface}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    const auto stats = solver.stats();
    CHECK_EQ(stats.edges, size_t{1});
    CHECK_EQ(stats.nodes, graph.nodes().size());
    CHECK_EQ(stats.bridges, size_t{0});
    CHECK_EQ(stats.tunnels, size_t{0});

    // A flat surface satisfies every constraint on the first look, so the
    // relaxation has nothing to relax. A solver that always runs to
    // max_iterations is doing work proportional to the config rather than to the
    // terrain, and would dominate import time on a city extract.
    CHECK_TRUE(stats.iterations <= size_t{8});
    CHECK_TRUE(stats.iterations < static_cast<size_t>(cfg.max_iterations));

    const float expected = 12.5f + cfg.surface_offset;

    for (GraphNodeId n = 0; n < graph.nodes().size(); ++n) {
        // node_height() excludes surface_offset by contract, so the bare terrain
        // height is what must come back here.
        CHECK_NEAR(solver.node_height(n), 12.5f, 1e-5);
        CHECK_NEAR(node_surface_height(solver, cfg, n), expected, 1e-5);
    }

    const EdgeElevation& elev = solver.edge(0);
    CHECK_EQ(elev.station_heights.size(), centerlines[0].stations.size());
    CHECK_FALSE(elev.is_bridge);
    CHECK_FALSE(elev.is_tunnel);
    CHECK_NEAR(elev.max_grade_used, 0.0, kGradeEps);

    size_t off_surface = 0;
    for (float h : elev.station_heights) {
        if (std::fabs(static_cast<double>(h) - static_cast<double>(expected)) > 1e-5) {
            ++off_surface;
        }
    }
    CHECK_EQ(off_surface, size_t{0});
}

// ============================================================================
// Grade limiting
// ============================================================================

TEST(RoadElevation, constant_slope_is_clamped_to_the_class_limit) {
    // 300 m of residential road up a 20% ramp. The class limit is 8%, so the
    // road cannot follow the ground and must depart from it.
    ParsedOSMData data = make_data({
        make_road(1, {1, 2}, {{0.0, 0.0}, {300.0, 0.0}}, RoadType::Residential),
    });

    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = make_centerlines(graph);
    if (graph.edges().empty()) {
        CHECK_TRUE(false);
        return;
    }

    ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines, HeightSampler{ramp_surface}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    const EdgeElevation& elev = solver.edge(0);
    const Centerline& cl = centerlines[0];
    CHECK_EQ(elev.station_heights.size(), cl.stations.size());
    if (elev.station_heights.size() != cl.stations.size()) return;

    const double limit = static_cast<double>(max_grade_for(RoadType::Residential, cfg));

    CHECK_TRUE(all_finite(elev.station_heights));
    CHECK_NEAR(max_gradient(cl, elev.station_heights), limit, limit + kGradeEps);
    CHECK_TRUE(max_gradient(cl, elev.station_heights) <= limit + kGradeEps);
    CHECK_TRUE(static_cast<double>(elev.max_grade_used) <= limit + kGradeEps);

    // The grade limit bound, so the solver must say so.
    CHECK_EQ(solver.stats().grade_limited_edges, size_t{1});

    // The road still climbs from one end to the other and never doubles back:
    // clamping a monotone ramp must not introduce a dip.
    size_t non_monotone = 0;
    for (size_t i = 0; i + 1 < elev.station_heights.size(); ++i) {
        if (elev.station_heights[i + 1] < elev.station_heights[i] - 1e-4f) ++non_monotone;
    }
    CHECK_EQ(non_monotone, size_t{0});
    CHECK_TRUE(elev.station_heights.back() > elev.station_heights.front());

    // Total rise over 300 m cannot exceed the limit times the length.
    const double rise = static_cast<double>(elev.station_heights.back())
                        - static_cast<double>(elev.station_heights.front());
    CHECK_TRUE(rise <= limit * cl.length() + 0.01);

    // The ends are the node heights, which the relaxation had to pull together.
    CHECK_NEAR(elev.station_heights.front(),
               node_surface_height(solver, cfg, graph.edge(0).from), kAgreementEps);
    CHECK_NEAR(elev.station_heights.back(),
               node_surface_height(solver, cfg, graph.edge(0).to), kAgreementEps);
}

// ============================================================================
// The rollercoaster case
// ============================================================================

TEST(RoadElevation, sharp_terrain_spike_is_not_followed) {
    ParsedOSMData data = make_data({
        make_road(1, {1, 2}, {{0.0, 0.0}, {300.0, 0.0}}, RoadType::Residential),
    });

    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = make_centerlines(graph);
    if (graph.edges().empty()) {
        CHECK_TRUE(false);
        return;
    }

    ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines, HeightSampler{spike_surface}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    const EdgeElevation& elev = solver.edge(0);
    const Centerline& cl = centerlines[0];
    CHECK_EQ(elev.station_heights.size(), cl.stations.size());
    if (elev.station_heights.size() != cl.stations.size()) return;

    CHECK_TRUE(all_finite(elev.station_heights));

    const double limit = static_cast<double>(max_grade_for(RoadType::Residential, cfg));
    CHECK_TRUE(max_gradient(cl, elev.station_heights) <= limit + kGradeEps);
    CHECK_TRUE(static_cast<double>(elev.max_grade_used) <= limit + kGradeEps);

    // The spike tops out at 50 m over ground at 10 m. A road that followed it
    // would reach nearly 50; one that ignores it stays near 10. Anything between
    // is the rollercoaster.
    float highest = elev.station_heights.front();
    for (float h : elev.station_heights) highest = std::max(highest, h);
    CHECK_TRUE(highest < 25.0f);

    // No kink: the gradient may not change faster than the vertical curvature
    // limit allows.
    const double curvature = max_grade_change_per_metre(cl, elev.station_heights);
    CHECK_TRUE(curvature
               <= static_cast<double>(cfg.max_grade_change_per_m) * kCurvatureSlack + 1e-9);

    // Both ends still sit on the ground the spike did not touch.
    CHECK_NEAR(elev.station_heights.front(), 10.0f + cfg.surface_offset, 0.5);
    CHECK_NEAR(elev.station_heights.back(), 10.0f + cfg.surface_offset, 0.5);
}

// ============================================================================
// Node agreement -- the assertion this phase exists for
// ============================================================================

TEST(RoadElevation, every_arm_terminates_at_its_node_height) {
    auto parsed = parse_fixture("four_way.osm");
    if (!parsed) return;

    RoadGraph graph;
    graph.build(*parsed);
    const std::vector<Centerline> centerlines = make_centerlines(graph);

    // four_way.osm: node 100 is interior to both ways, so it carries four arms
    // and each way splits in two. Four edges, four dead ends, eight arms.
    CHECK_EQ(graph.edges().size(), size_t{4});
    CHECK_EQ(graph.stats().junctions, size_t{1});

    const ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines, HeightSampler{tilted_surface}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    CHECK_EQ(solver.node_heights().size(), graph.nodes().size());
    CHECK_EQ(solver.edges().size(), graph.edges().size());

    size_t arms_checked = 0;
    size_t torn_arms = 0;
    size_t unsolved_arms = 0;

    for (GraphNodeId n = 0; n < graph.nodes().size(); ++n) {
        const GraphNode& node = graph.node(n);
        const double node_h = node_surface_height(solver, cfg, n);

        for (const Arm& arm : node.arms) {
            const EdgeElevation& elev = solver.edge(arm.edge);
            if (elev.station_heights.empty()) {
                ++unsolved_arms;
                continue;
            }
            ++arms_checked;
            const double end_h = static_cast<double>(arm_end_height(elev, arm));
            if (std::fabs(end_h - node_h) > kAgreementEps) {
                ++torn_arms;
                stratum::test::report_failure(
                    __FILE__, __LINE__, "arm end height == node height",
                    "node " + std::to_string(node.osm_id) + " edge "
                        + std::to_string(arm.edge) + " arm end: "
                        + stratum::test::stringify(end_h)
                        + "  node: " + stratum::test::stringify(node_h));
            }
        }
    }

    // Every edge of this fixture is solvable, so no arm may be missing a profile.
    CHECK_EQ(unsolved_arms, size_t{0});
    CHECK_EQ(arms_checked, size_t{8});
    CHECK_EQ(torn_arms, size_t{0});
}

// ============================================================================
// Bridges
// ============================================================================

TEST(RoadElevation, bridge_clears_the_gorge_and_meets_its_abutments) {
    // Approach, deck, approach. Nodes 2 and 3 are abutments: every way merely
    // ends on them, so the graph keeps them as one node each whatever the layer,
    // and the deck stays connected to its approaches.
    ParsedOSMData data = make_data({
        make_road(10, {1, 2}, {{0.0, 0.0}, {400.0, 0.0}}, RoadType::Primary),
        make_road(11, {2, 3}, {{400.0, 0.0}, {500.0, 0.0}}, RoadType::Primary),
        make_road(12, {3, 4}, {{500.0, 0.0}, {900.0, 0.0}}, RoadType::Primary),
    });
    data.roads[1].is_bridge = true;
    data.roads[1].layer = 1;

    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = make_centerlines(graph);
    CHECK_EQ(graph.edges().size(), size_t{3});

    ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines, HeightSampler{gorge_surface}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    CHECK_EQ(solver.stats().bridges, size_t{1});
    CHECK_EQ(solver.stats().tunnels, size_t{0});

    EdgeId bridge_id = stratum::osm::road::kInvalidId;
    for (EdgeId e = 0; e < graph.edges().size(); ++e) {
        if (graph.edge(e).is_bridge) bridge_id = e;
    }
    CHECK_TRUE(bridge_id != stratum::osm::road::kInvalidId);
    if (bridge_id == stratum::osm::road::kInvalidId) return;

    const EdgeElevation& elev = solver.edge(bridge_id);
    const Centerline& cl = centerlines[bridge_id];
    CHECK_TRUE(elev.is_bridge);
    CHECK_FALSE(elev.is_tunnel);
    CHECK_EQ(elev.station_heights.size(), cl.stations.size());
    if (elev.station_heights.size() != cl.stations.size()) return;
    CHECK_TRUE(all_finite(elev.station_heights));

    // A deck that dived into the gorge would clear the floor by 20 m at mid-span
    // and by nothing at all on the walls, so the clearance is asserted at EVERY
    // station rather than at the middle.
    size_t stations_too_low = 0;
    for (size_t i = 0; i < elev.station_heights.size(); ++i) {
        const glm::dvec2 p = cl.stations[i].position;
        const double ground = static_cast<double>(gorge_surface(p.x, p.y));
        const double clearance = static_cast<double>(elev.station_heights[i]) - ground;
        if (clearance < static_cast<double>(cfg.bridge_clearance) - 1e-3) {
            ++stations_too_low;
        }
    }
    CHECK_EQ(stations_too_low, size_t{0});

    // Lifting the deck must not tear it off its approaches: the abutment node
    // heights move with it.
    CHECK_NEAR(elev.station_heights.front(),
               node_surface_height(solver, cfg, graph.edge(bridge_id).from), kAgreementEps);
    CHECK_NEAR(elev.station_heights.back(),
               node_surface_height(solver, cfg, graph.edge(bridge_id).to), kAgreementEps);

    // And the approaches still terminate at those same node heights.
    size_t torn_arms = 0;
    for (GraphNodeId n = 0; n < graph.nodes().size(); ++n) {
        const double node_h = node_surface_height(solver, cfg, n);
        for (const Arm& arm : graph.node(n).arms) {
            const EdgeElevation& e = solver.edge(arm.edge);
            if (e.station_heights.empty()) continue;
            if (std::fabs(static_cast<double>(arm_end_height(e, arm)) - node_h)
                > kAgreementEps) {
                ++torn_arms;
            }
        }
    }
    CHECK_EQ(torn_arms, size_t{0});
}

// ============================================================================
// Tunnels
// ============================================================================

TEST(RoadElevation, tunnel_runs_below_the_sampled_surface) {
    ParsedOSMData data = make_data({
        make_road(10, {1, 2}, {{0.0, 0.0}, {400.0, 0.0}}, RoadType::Primary),
        make_road(11, {2, 3}, {{400.0, 0.0}, {500.0, 0.0}}, RoadType::Primary),
        make_road(12, {3, 4}, {{500.0, 0.0}, {900.0, 0.0}}, RoadType::Primary),
    });
    data.roads[1].is_tunnel = true;
    data.roads[1].layer = -1;

    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = make_centerlines(graph);
    CHECK_EQ(graph.edges().size(), size_t{3});

    ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines, HeightSampler{hill_surface}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    CHECK_EQ(solver.stats().tunnels, size_t{1});
    CHECK_EQ(solver.stats().bridges, size_t{0});

    EdgeId tunnel_id = stratum::osm::road::kInvalidId;
    for (EdgeId e = 0; e < graph.edges().size(); ++e) {
        if (graph.edge(e).is_tunnel) tunnel_id = e;
    }
    CHECK_TRUE(tunnel_id != stratum::osm::road::kInvalidId);
    if (tunnel_id == stratum::osm::road::kInvalidId) return;

    const EdgeElevation& elev = solver.edge(tunnel_id);
    const Centerline& cl = centerlines[tunnel_id];
    CHECK_TRUE(elev.is_tunnel);
    CHECK_FALSE(elev.is_bridge);
    CHECK_EQ(elev.station_heights.size(), cl.stations.size());
    if (elev.station_heights.size() < 3) {
        CHECK_TRUE(false);
        return;
    }
    CHECK_TRUE(all_finite(elev.station_heights));

    // Mid-span sits under 40 m of hill. Whatever the grade limit does to the
    // approaches, the roadway there is below ground by a wide margin.
    const size_t mid = elev.station_heights.size() / 2;
    const glm::dvec2 p = cl.stations[mid].position;
    const double ground = static_cast<double>(hill_surface(p.x, p.y));
    CHECK_TRUE(static_cast<double>(elev.station_heights[mid]) < ground - 1.0);

    // The portals still meet their nodes.
    CHECK_NEAR(elev.station_heights.front(),
               node_surface_height(solver, cfg, graph.edge(tunnel_id).from), kAgreementEps);
    CHECK_NEAR(elev.station_heights.back(),
               node_surface_height(solver, cfg, graph.edge(tunnel_id).to), kAgreementEps);
}

/**
 * A tunnel mapped as TWO ways is still one tunnel.
 *
 * A mapper splits a covered way wherever a tag changes -- a name, a ref, a
 * maxspeed -- so a bore of any length arrives as several ways sharing interior
 * nodes. P1 turns each shared node into a GraphNode, and at that node every arm
 * is a tunnel arm, so it is NOT a portal: both edges ramp at their outer end and
 * are pinned at the shared one.
 *
 * That is the case where the override loop can run away. Its demand is measured
 * against the straight deck, and a portal station sits at the approach surface by
 * construction, so measuring the portal end makes the demand about tunnel_depth
 * on every pass no matter how deep the interior node already is. Exempting that
 * end from the drop is not enough -- the exemption stops it being applied there,
 * not counted -- so the interior node takes the whole amount again on each of
 * kOverridePasses passes and ends up eight tunnel depths down, with the surface
 * approaches dragged after it down the grade chain.
 *
 * Flat ground, so the arithmetic has one answer: the shared node owes exactly
 * tunnel_depth and nothing more.
 */
TEST(RoadElevation, a_tunnel_split_into_two_ways_does_not_bury_its_shared_node) {
    constexpr double kGround = 12.5;

    ParsedOSMData data = make_data({
        make_road(10, {1, 2}, {{0.0, 0.0}, {400.0, 0.0}}, RoadType::Primary),
        make_road(11, {2, 5}, {{400.0, 0.0}, {700.0, 0.0}}, RoadType::Primary),
        make_road(12, {5, 3}, {{700.0, 0.0}, {1000.0, 0.0}}, RoadType::Primary),
        make_road(13, {3, 4}, {{1000.0, 0.0}, {1400.0, 0.0}}, RoadType::Primary),
    });
    for (size_t i = 1; i <= 2; ++i) {
        data.roads[i].is_tunnel = true;
        data.roads[i].layer = -1;
    }

    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = make_centerlines(graph);

    ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines,
                 HeightSampler{[](double, double) { return static_cast<float>(kGround); }}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;
    CHECK_EQ(solver.stats().tunnels, size_t{2});

    // The node the two bores share. It is interior, so it is the one node the
    // override loop is allowed to drop.
    GraphNodeId shared = stratum::osm::road::kInvalidId;
    GraphNodeId far_ends[2] = { stratum::osm::road::kInvalidId,
                                stratum::osm::road::kInvalidId };
    for (GraphNodeId n = 0; n < graph.nodes().size(); ++n) {
        if (graph.node(n).osm_id == 5) shared = n;
        if (graph.node(n).osm_id == 1) far_ends[0] = n;
        if (graph.node(n).osm_id == 4) far_ends[1] = n;
    }
    CHECK_TRUE(shared != stratum::osm::road::kInvalidId);
    if (shared == stratum::osm::road::kInvalidId) return;

    // One tunnel depth under the ground, not eight.
    const double depth = kGround - node_surface_height(solver, cfg, shared);
    if (!(depth > 0.0) || depth > static_cast<double>(cfg.tunnel_depth) + 1.0) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "the shared node sits one tunnel depth down",
            "it is " + std::to_string(depth) + " m below ground, against a tunnel_depth of " +
                std::to_string(cfg.tunnel_depth));
    }

    // And the surface roads either side are still surface roads. A runaway drop
    // is not confined to the bore: the portals are free to move, so the whole
    // approach chain follows the buried node underground.
    for (GraphNodeId n : far_ends) {
        if (n == stratum::osm::road::kInvalidId) continue;
        const double h = node_surface_height(solver, cfg, n);
        if (std::fabs(h - kGround) > 1.0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "an approach dead end stays on the surface",
                "node " + std::to_string(graph.node(n).osm_id) + " is at " + std::to_string(h) +
                    " m against ground " + std::to_string(kGround));
        }
    }

    // Every edge still respects its class grade limit; a buried node is reached
    // by a ramp that cannot be built inside one.
    for (EdgeId e = 0; e < graph.edges().size(); ++e) {
        const EdgeElevation& elev = solver.edge(e);
        const Centerline& cl = centerlines[e];
        if (elev.station_heights.size() != cl.stations.size()) continue;
        double worst = 0.0;
        for (size_t j = 1; j < elev.station_heights.size(); ++j) {
            const double run = cl.stations[j].arclength - cl.stations[j - 1].arclength;
            if (!(run > 1e-6)) continue;
            const double rise = static_cast<double>(elev.station_heights[j])
                              - static_cast<double>(elev.station_heights[j - 1]);
            worst = std::max(worst, std::fabs(rise) / run);
        }
        if (worst > static_cast<double>(cfg.max_grade_primary) + 1e-3) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "every edge stays inside its class grade limit",
                "edge " + std::to_string(e) + " reaches " + std::to_string(worst) + " against " +
                    std::to_string(cfg.max_grade_primary));
        }
    }
}

// ============================================================================
// Determinism
// ============================================================================

TEST(RoadElevation, solving_twice_gives_bit_identical_heights) {
    auto parsed = parse_fixture("four_way.osm");
    if (!parsed) return;

    RoadGraph graph;
    graph.build(*parsed);
    const std::vector<Centerline> centerlines = make_centerlines(graph);

    RoadElevationSolver a;
    RoadElevationSolver b;
    a.solve(graph, centerlines, HeightSampler{noisy_surface}, ElevationConfig{});
    b.solve(graph, centerlines, HeightSampler{noisy_surface}, ElevationConfig{});

    CHECK_TRUE(a.is_solved());
    CHECK_TRUE(b.is_solved());
    if (!a.is_solved() || !b.is_solved()) return;

    // Bit equality, not a tolerance. Two runs of the same deterministic solve
    // over the same data have no licence to differ at all, and a tolerance here
    // would hide exactly the scheduling-dependent result this guards against.
    CHECK_TRUE(a.node_heights() == b.node_heights());

    size_t differing_edges = 0;
    for (EdgeId e = 0; e < graph.edges().size(); ++e) {
        if (a.edge(e).station_heights != b.edge(e).station_heights) ++differing_edges;
    }
    CHECK_EQ(differing_edges, size_t{0});

    // Re-solving on the same instance discards the previous result rather than
    // accumulating on it.
    a.solve(graph, centerlines, HeightSampler{noisy_surface}, ElevationConfig{});
    CHECK_TRUE(a.node_heights() == b.node_heights());
}

TEST(RoadElevation, edge_order_does_not_change_the_solution) {
    auto parsed = parse_fixture("four_way.osm");
    if (!parsed) return;

    ParsedOSMData forward = *parsed;
    ParsedOSMData reversed = *parsed;
    std::reverse(reversed.roads.begin(), reversed.roads.end());

    RoadGraph graph_a;
    graph_a.build(forward);
    RoadGraph graph_b;
    graph_b.build(reversed);

    CHECK_EQ(graph_a.nodes().size(), graph_b.nodes().size());
    CHECK_EQ(graph_a.edges().size(), graph_b.edges().size());

    RoadElevationSolver a;
    RoadElevationSolver b;
    a.solve(graph_a, make_centerlines(graph_a), HeightSampler{noisy_surface}, ElevationConfig{});
    b.solve(graph_b, make_centerlines(graph_b), HeightSampler{noisy_surface}, ElevationConfig{});

    CHECK_TRUE(a.is_solved());
    CHECK_TRUE(b.is_solved());
    if (!a.is_solved() || !b.is_solved()) return;

    // Reordering the roads renumbers every EdgeId and GraphNodeId, so the two
    // solves are compared through the one identifier that survives: the OSM node
    // ID. A road's height must not depend on where its way sat in the file.
    std::map<NodeId, float> heights_a;
    std::map<NodeId, float> heights_b;
    for (GraphNodeId n = 0; n < graph_a.nodes().size(); ++n) {
        heights_a[graph_a.node(n).osm_id] = a.node_height(n);
    }
    for (GraphNodeId n = 0; n < graph_b.nodes().size(); ++n) {
        heights_b[graph_b.node(n).osm_id] = b.node_height(n);
    }

    CHECK_EQ(heights_a.size(), graph_a.nodes().size());
    CHECK_EQ(heights_a.size(), heights_b.size());

    size_t mismatched = 0;
    for (const auto& [osm_id, height] : heights_a) {
        const auto it = heights_b.find(osm_id);
        if (it == heights_b.end()) {
            ++mismatched;
            continue;
        }
        if (std::fabs(static_cast<double>(height) - static_cast<double>(it->second)) > 1e-6) {
            ++mismatched;
        }
    }
    CHECK_EQ(mismatched, size_t{0});
}

// ============================================================================
// Convergence
// ============================================================================

TEST(RoadElevation, noisy_terrain_converges_within_the_iteration_cap) {
    // A 4x4 grid of shared nodes: twelve interior junctions of degree 3 or 4,
    // eight roads, every node height coupled to its neighbours through the grade
    // limit. That coupling is what makes the relaxation non-trivial.
    auto nid = [](int i, int j) { return static_cast<NodeId>(1000 + j * 10 + i); };
    auto pos = [](int i, int j) {
        return glm::dvec2{static_cast<double>(i) * 100.0, static_cast<double>(j) * 100.0};
    };

    std::vector<Road> roads;
    for (int j = 0; j < 4; ++j) {
        std::vector<NodeId> ids;
        std::vector<glm::dvec2> pts;
        for (int i = 0; i < 4; ++i) {
            ids.push_back(nid(i, j));
            pts.push_back(pos(i, j));
        }
        roads.push_back(make_road(static_cast<WayId>(2000 + j), ids, pts, RoadType::Residential));
    }
    for (int i = 0; i < 4; ++i) {
        std::vector<NodeId> ids;
        std::vector<glm::dvec2> pts;
        for (int j = 0; j < 4; ++j) {
            ids.push_back(nid(i, j));
            pts.push_back(pos(i, j));
        }
        roads.push_back(make_road(static_cast<WayId>(3000 + i), ids, pts, RoadType::Residential));
    }

    ParsedOSMData data = make_data(std::move(roads));

    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = make_centerlines(graph);

    CHECK_EQ(graph.nodes().size(), size_t{16});
    CHECK_EQ(graph.edges().size(), size_t{24});

    ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines, HeightSampler{noisy_surface}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    const auto stats = solver.stats();

    // Terminating because the iteration cap ran out is not convergence; the
    // residual is what says the solve actually settled.
    CHECK_TRUE(stats.iterations < static_cast<size_t>(cfg.max_iterations));
    CHECK_TRUE(static_cast<double>(stats.max_residual)
               <= static_cast<double>(cfg.convergence_epsilon) + 1e-9);
    CHECK_EQ(stats.edges, graph.edges().size());
    CHECK_EQ(stats.nodes, graph.nodes().size());

    const double limit = static_cast<double>(max_grade_for(RoadType::Residential, cfg));

    size_t over_grade = 0;
    size_t non_finite = 0;
    size_t empty_profiles = 0;
    size_t torn_arms = 0;

    for (EdgeId e = 0; e < graph.edges().size(); ++e) {
        const EdgeElevation& elev = solver.edge(e);
        if (elev.station_heights.empty()) {
            ++empty_profiles;
            continue;
        }
        if (!all_finite(elev.station_heights)) ++non_finite;
        if (max_gradient(centerlines[e], elev.station_heights) > limit + kGradeEps) {
            ++over_grade;
        }
    }

    for (GraphNodeId n = 0; n < graph.nodes().size(); ++n) {
        const double node_h = node_surface_height(solver, cfg, n);
        if (!std::isfinite(node_h)) ++non_finite;
        for (const Arm& arm : graph.node(n).arms) {
            const EdgeElevation& elev = solver.edge(arm.edge);
            if (elev.station_heights.empty()) continue;
            if (std::fabs(static_cast<double>(arm_end_height(elev, arm)) - node_h)
                > kAgreementEps) {
                ++torn_arms;
            }
        }
    }

    CHECK_EQ(empty_profiles, size_t{0});
    CHECK_EQ(non_finite, size_t{0});
    CHECK_EQ(over_grade, size_t{0});
    CHECK_EQ(torn_arms, size_t{0});
}

// ============================================================================
// layer=* is not a structure
// ============================================================================
//
// OSM `layer=*` is a rendering-order hint. The LOWER road at a grade separation
// is routinely tagged `highway=residential, layer=-1` with no tunnel tag at all,
// and a road on an embankment carries `layer=1` with no bridge tag. Reading
// either as a structure buries an ordinary street tunnel_depth metres
// underground -- with its terrain carve suppressed, so it is invisible inside
// the hill -- or floats a footway bridge_clearance metres into the air.
//
// P1 already consumes layer for what it is for: splitting a grade-separation
// node whose arms disagree (RoadGraph::slot_layer). Nothing here needs it.

TEST(RoadElevation, negative_layer_without_a_tunnel_tag_stays_on_the_surface) {
    ParsedOSMData data = make_data({
        make_road(10, {1, 2}, {{0.0, 0.0}, {400.0, 0.0}}, RoadType::Residential),
        make_road(11, {2, 3}, {{400.0, 0.0}, {500.0, 0.0}}, RoadType::Residential),
        make_road(12, {3, 4}, {{500.0, 0.0}, {900.0, 0.0}}, RoadType::Residential),
    });
    data.roads[1].layer = -1;   // and deliberately NO tunnel tag

    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = make_centerlines(graph);

    ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines, HeightSampler{flat_surface}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    CHECK_EQ(solver.stats().tunnels, size_t{0});
    CHECK_EQ(solver.stats().bridges, size_t{0});

    size_t misclassified = 0;
    size_t buried = 0;
    for (EdgeId e = 0; e < graph.edges().size(); ++e) {
        const EdgeElevation& elev = solver.edge(e);
        if (elev.is_tunnel || elev.is_bridge) ++misclassified;
        for (float h : elev.station_heights) {
            // flat_surface() plus surface_offset. A layer-derived tunnel would
            // put this at ground - tunnel_depth, eight metres down.
            if (std::fabs(static_cast<double>(h) - static_cast<double>(flat_surface(0.0, 0.0))
                          - static_cast<double>(cfg.surface_offset)) > 0.05) {
                ++buried;
            }
        }
    }
    CHECK_EQ(misclassified, size_t{0});
    CHECK_EQ(buried, size_t{0});
}

TEST(RoadElevation, positive_layer_without_a_bridge_tag_stays_on_the_surface) {
    ParsedOSMData data = make_data({
        make_road(10, {1, 2}, {{0.0, 0.0}, {400.0, 0.0}}, RoadType::Residential),
        make_road(11, {2, 3}, {{400.0, 0.0}, {500.0, 0.0}}, RoadType::Residential),
        make_road(12, {3, 4}, {{500.0, 0.0}, {900.0, 0.0}}, RoadType::Residential),
    });
    data.roads[1].layer = 1;    // and deliberately NO bridge tag

    RoadGraph graph;
    graph.build(data);
    const std::vector<Centerline> centerlines = make_centerlines(graph);

    ElevationConfig cfg;
    RoadElevationSolver solver;
    solver.solve(graph, centerlines, HeightSampler{flat_surface}, cfg);

    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    CHECK_EQ(solver.stats().bridges, size_t{0});
    CHECK_EQ(solver.stats().tunnels, size_t{0});

    size_t floating = 0;
    for (EdgeId e = 0; e < graph.edges().size(); ++e) {
        const EdgeElevation& elev = solver.edge(e);
        if (elev.is_bridge || elev.is_tunnel) ++floating;
        for (float h : elev.station_heights) {
            if (std::fabs(static_cast<double>(h) - static_cast<double>(flat_surface(0.0, 0.0))
                          - static_cast<double>(cfg.surface_offset)) > 0.05) {
                ++floating;
            }
        }
    }
    CHECK_EQ(floating, size_t{0});
}
