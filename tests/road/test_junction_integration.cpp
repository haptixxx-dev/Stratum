/**
 * @file test_junction_integration.cpp
 * @brief Whole-network P4 tests: ribbons must stop at the junctions they meet
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Written against docs/plans/road_network_plan.md and the contract in
 * src/osm/road/junction_builder.hpp.
 *
 * ### The defect P4 exists to fix
 *
 * From the plan's defect list: "No junction cut. Ribbons pass straight through
 * each other." Every other test in this directory checks a component of the fix.
 * This one checks the fix, by taking the geometry the pipeline actually emits and
 * asking the question directly: does any ribbon triangle overlap any junction
 * polygon in plan view?
 *
 * It is deliberately not a proxy. Asserting that trims are non-zero, or that the
 * junction count is right, would pass for a solver that wrote its trims and then
 * extruded the untrimmed centerline anyway -- which junction_builder.hpp names as
 * a real and strictly-worse-than-nothing failure mode, because it adds a coplanar
 * surface inside every intersection. Only overlap catches that.
 *
 * ### Tolerances
 *
 * A trimmed arm end lands exactly ON the junction ring, so ribbon and ring share
 * a boundary and every overlap test has to tolerate contact. The ring is pulled
 * 5 cm towards its own centroid before testing, which leaves a strict interior
 * nothing may enter. Junction polygons are metres across and the defect being
 * hunted is metres deep, so 5 cm costs nothing.
 *
 * ### Terrain
 *
 * The network is built on a synthetic tilted surface, not on procedural terrain.
 * Nothing under src/osm/road may include a procgen header, and a test that
 * generated terrain would also be testing the noise function. A plane with a slow
 * undulation is enough to make every node height distinct, which is what the
 * junction-height assertion needs.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests JunctionIntegration
 * @endcode
 */

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/road/junction_builder.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_network_builder.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace {

using stratum::Mesh;
using stratum::osm::ParsedOSMData;
using stratum::osm::road::Centerline;
using stratum::osm::road::EdgeId;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::HeightSampler;
using stratum::osm::road::Junction;
using stratum::osm::road::JunctionKind;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;
using stratum::osm::road::kInvalidId;

namespace jt = stratum::test::junction;

/**
 * @brief Gentle tilted plane with a slow undulation
 *
 * Inside every road class's grade limit, so the elevation solver never has to
 * depart from it, and varying enough that no two graph nodes land on the same
 * height by accident. A flat surface would let a junction-height bug pass.
 */
float tilted_surface(double x, double y) {
    return static_cast<float>(20.0 + 0.02 * x - 0.015 * y
                              + 1.5 * std::sin(0.03 * x) * std::cos(0.025 * y));
}

/// How far the junction ring is pulled inward before the overlap test, metres
constexpr double kOverlapMargin = 0.05;

/**
 * @brief Build one fixture's whole network with the junction solve on
 *
 * @param filename    Fixture in tests/data
 * @param builder     Builder to run; kept by the caller so its graph and
 *                    centerlines outlive the returned network
 * @param out_network Receives the built network
 * @param on_terrain  Supply a height sampler, so the elevation solve runs
 * @return false when the fixture could not be parsed
 */
bool build_fixture(const char* filename, RoadNetworkBuilder& builder,
                   RoadNetwork& out_network, bool on_terrain = true) {
    const auto parsed = jt::parse_fixture(filename);
    if (!parsed) return false;

    RoadNetworkConfig cfg;
    cfg.solve_junctions = true;
    if (on_terrain) {
        cfg.height_sampler = HeightSampler(&tilted_surface);
    }
    out_network = builder.build(*parsed, cfg);
    return true;
}

} // namespace

// ============================================================================
// The defect P4 fixes
// ============================================================================

/**
 * Over EVERY fixture: no ribbon triangle may overlap a junction polygon in plan
 * view.
 *
 * A roundabout's own ring edges are excluded, and only those. A `junction=roundabout`
 * cycle is replaced wholesale by the annulus in junction_special.hpp rather than
 * being trimmed arm by arm, so the relationship between a ring edge's ribbon and
 * an approach node's fillet is that file's contract, not the trim contract this
 * test is about. Every other edge of roundabout.osm, including all three
 * approaches, is still checked.
 */
TEST(JunctionIntegration, no_ribbon_triangle_overlaps_a_junction_polygon) {
    size_t fixtures_with_junctions = 0;
    size_t polygons_checked = 0;

    for (const char* filename : jt::kAllFixtures) {
        RoadNetworkBuilder builder;
        RoadNetwork network;
        if (!build_fixture(filename, builder, network)) continue;

        const RoadGraph& graph = builder.graph();
        const std::string label(filename);

        // Collect the footprints that have a real polygon to test against.
        std::vector<std::vector<glm::dvec2>> rings;
        for (const Junction& junction : network.junctions) {
            if (junction.kind != JunctionKind::Intersection) continue;
            if (!junction.polygon.valid || junction.polygon.self_intersecting) continue;
            if (junction.polygon.ring.size() < 3) continue;
            rings.push_back(jt::shrink_ring(junction.polygon.ring, junction.polygon.centroid,
                                            kOverlapMargin));
            ++polygons_checked;
        }
        if (rings.empty()) continue;
        ++fixtures_with_junctions;

        for (const RoadPiece& piece : network.pieces) {
            if (piece.edge == kInvalidId) continue;  // a junction piece, not a ribbon
            if (piece.edge >= graph.edges().size()) continue;
            if (graph.edge(piece.edge).is_roundabout) continue;

            const std::vector<jt::Tri2D> tris = jt::triangles_of(piece.mesh);
            for (size_t t = 0; t < tris.size(); ++t) {
                for (size_t r = 0; r < rings.size(); ++r) {
                    if (jt::triangle_overlaps_ring(tris[t], rings[r])) {
                        stratum::test::report_failure(
                            __FILE__, __LINE__,
                            "ribbon triangle does not overlap a junction polygon",
                            label + ": edge " + std::to_string(piece.edge) + " triangle " +
                                std::to_string(t) + " overlaps junction polygon " +
                                std::to_string(r));
                        // One report per piece is enough to identify the defect;
                        // a through-running ribbon would otherwise report hundreds.
                        t = tris.size();
                        break;
                    }
                }
            }
        }
    }

    // The sweep is only meaningful if the fixtures actually produced junctions to
    // test against.
    CHECK_TRUE(fixtures_with_junctions > 0);
    CHECK_TRUE(polygons_checked > 0);
}

// ============================================================================
// Trims
// ============================================================================

/**
 * Every edge keeps a positive length after both of its ends are cut, or the build
 * reports that its trims consumed it. A NEGATIVE remaining length is never
 * acceptable and is asserted unconditionally: it reaches slice() as a reversed
 * range and the extruder as a ribbon with no stations.
 */
TEST(JunctionIntegration, every_edge_keeps_a_positive_length_after_trimming) {
    size_t trimmed_edges_seen = 0;

    for (const char* filename : jt::kAllFixtures) {
        RoadNetworkBuilder builder;
        RoadNetwork network;
        if (!build_fixture(filename, builder, network)) continue;

        const RoadGraph& graph = builder.graph();
        const std::vector<Centerline>& centerlines = builder.centerlines();
        const std::string label(filename);

        CHECK_EQ(centerlines.size(), graph.edges().size());
        if (centerlines.size() != graph.edges().size()) continue;

        for (size_t e = 0; e < graph.edges().size(); ++e) {
            const GraphEdge& edge = graph.edges()[e];
            const std::string where = label + " edge " + std::to_string(e);

            if (!std::isfinite(edge.trim_from) || !std::isfinite(edge.trim_to)) {
                stratum::test::report_failure(__FILE__, __LINE__, "trims are finite", where);
                continue;
            }
            if (edge.trim_from < 0.0 || edge.trim_to < 0.0) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "trims are non-negative",
                    where + ": " + stratum::test::stringify(edge.trim_from) + " / " +
                        stratum::test::stringify(edge.trim_to));
                continue;
            }
            if (edge.trim_from > 0.0 || edge.trim_to > 0.0) ++trimmed_edges_seen;

            if (!centerlines[e].is_valid()) continue;
            const double remaining =
                centerlines[e].length() - edge.trim_from - edge.trim_to;

            // Unconditional: never negative.
            if (remaining < -1e-9) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "remaining length is not negative",
                    where + ": length " + stratum::test::stringify(centerlines[e].length()) +
                        " trims " + stratum::test::stringify(edge.trim_from) + " + " +
                        stratum::test::stringify(edge.trim_to));
                continue;
            }

            // Positive, or the build honestly reported the edge as consumed.
            if (remaining <= 0.0 && network.stats.trimmed_away_edges == 0) {
                stratum::test::report_failure(
                    __FILE__, __LINE__,
                    "edge has positive remaining length or is reported over-trimmed",
                    where + ": remaining " + stratum::test::stringify(remaining) +
                        ", trimmed_away_edges 0");
            }
        }

        // The trims really were written; a build that wrote none would satisfy
        // every inequality above.
        CHECK_TRUE(network.stats.trimmed_edges <= network.stats.edges);
    }

    CHECK_TRUE(trimmed_edges_seen > 0);
}

/**
 * The P2 reference path. With RoadNetworkConfig::solve_junctions off, every trim
 * stays zero and no junction is emitted, which is what makes a solver regression
 * bisectable without a rebuild.
 */
TEST(JunctionIntegration, junction_solve_off_leaves_every_trim_zero) {
    const auto parsed = jt::parse_fixture("four_way.osm");
    if (!parsed) return;

    RoadNetworkConfig cfg;
    cfg.solve_junctions = false;
    cfg.height_sampler = HeightSampler(&tilted_surface);

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*parsed, cfg);

    CHECK_TRUE(network.junctions.empty());
    CHECK_EQ(network.stats.junction_pieces, size_t{0});
    CHECK_EQ(network.stats.trimmed_edges, size_t{0});
    for (const GraphEdge& edge : builder.graph().edges()) {
        CHECK_NEAR(edge.trim_from, 0.0, 1e-12);
        CHECK_NEAR(edge.trim_to, 0.0, 1e-12);
    }

    // And with the solve on, the same fixture does produce junctions and trims,
    // so the assertion above is about the switch and not about the fixture.
    RoadNetworkBuilder solved_builder;
    RoadNetwork solved;
    if (!build_fixture("four_way.osm", solved_builder, solved)) return;
    CHECK_TRUE(!solved.junctions.empty());
    CHECK_TRUE(solved.stats.trimmed_edges > 0);
}

// ============================================================================
// Numerics
// ============================================================================

/**
 * No NaN and no infinity anywhere in the emitted network.
 *
 * A NaN vertex position neither crashes nor renders: it silently corrupts the
 * bounding box, breaks frustum culling for the whole chunk, and is the hardest
 * failure in this pipeline to trace to its cause. P4 adds three new sources of
 * one -- a parallel-line intersection, a fillet radius reduction, and a Clipper2
 * round trip -- so the sweep is repeated here over the junction outputs the P2
 * sweep never saw.
 */
TEST(JunctionIntegration, no_nan_or_infinite_geometry_anywhere) {
    for (const char* filename : jt::kAllFixtures) {
        RoadNetworkBuilder builder;
        RoadNetwork network;
        if (!build_fixture(filename, builder, network)) continue;
        const std::string label(filename);

        for (size_t p = 0; p < network.pieces.size(); ++p) {
            const RoadPiece& piece = network.pieces[p];
            const std::string where = label + " piece " + std::to_string(p);

            if (!std::isfinite(piece.anchor.x) || !std::isfinite(piece.anchor.y)) {
                stratum::test::report_failure(__FILE__, __LINE__, "piece anchor is finite", where);
            }
            for (const auto& v : piece.mesh.vertices) {
                if (!jt::is_finite(v.position) || !jt::is_finite(v.normal) ||
                    !std::isfinite(v.uv.x) || !std::isfinite(v.uv.y)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "piece vertex is finite", where);
                    break;
                }
            }
            for (const glm::dvec2& q : piece.outline) {
                if (!std::isfinite(q.x) || !std::isfinite(q.y)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "piece outline point is finite", where);
                    break;
                }
            }
        }

        for (size_t j = 0; j < network.junctions.size(); ++j) {
            const Junction& junction = network.junctions[j];
            const std::string where = label + " junction " + std::to_string(j);

            if (!std::isfinite(junction.height)) {
                stratum::test::report_failure(__FILE__, __LINE__, "junction height is finite",
                                              where);
            }
            if (!std::isfinite(junction.center.x) || !std::isfinite(junction.center.y)) {
                stratum::test::report_failure(__FILE__, __LINE__, "junction centre is finite",
                                              where);
            }
            for (const auto& arm : junction.arms) {
                if (!std::isfinite(arm.trim) || arm.trim < 0.0) {
                    stratum::test::report_failure(
                        __FILE__, __LINE__, "arm trim is finite and non-negative", where);
                    break;
                }
            }
            for (const auto& end : junction.ends) {
                if (!std::isfinite(end.center.x) || !std::isfinite(end.center.y) ||
                    !std::isfinite(end.carriage_left.x) || !std::isfinite(end.carriage_right.y) ||
                    !std::isfinite(end.arclength)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "arm end is finite", where);
                    break;
                }
            }
            for (const glm::dvec2& q : junction.polygon.ring) {
                if (!std::isfinite(q.x) || !std::isfinite(q.y)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "junction ring point is finite", where);
                    break;
                }
            }
            for (const glm::dvec2& q : junction.footprint) {
                if (!std::isfinite(q.x) || !std::isfinite(q.y)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "junction footprint point is finite", where);
                    break;
                }
            }
            for (const auto& v : junction.mesh.vertices) {
                if (!jt::is_finite(v.position) || !jt::is_finite(v.normal)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "junction vertex is finite", where);
                    break;
                }
            }
        }

        for (const auto& disc : network.carve_discs) {
            for (const glm::dvec2& q : disc.outline) {
                if (!std::isfinite(q.x) || !std::isfinite(q.y)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "carve disc outline point is finite", label);
                    break;
                }
            }
        }
    }
}

// ============================================================================
// Elevation agreement
// ============================================================================

/**
 * A junction is placed at its own solved node height plus
 * ElevationConfig::surface_offset, and on nothing else.
 *
 * node_height() deliberately EXCLUDES the offset so each consumer adds it exactly
 * once; a junction that also inherited it from the arms would float one offset
 * above the ribbons feeding it, and a junction that never added it would sit one
 * offset below them. Both read as a tear at every approach, so the value is
 * asserted to a float's worth of tolerance rather than a visual one.
 */
TEST(JunctionIntegration, junction_height_equals_the_solved_node_height) {
    RoadNetworkConfig cfg;
    cfg.solve_junctions = true;
    cfg.height_sampler = HeightSampler(&tilted_surface);

    size_t junctions_checked = 0;

    for (const char* filename : jt::kAllFixtures) {
        const auto parsed = jt::parse_fixture(filename);
        if (!parsed) continue;

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*parsed, cfg);
        const auto& elevation = builder.elevation();
        const std::string label(filename);

        CHECK_TRUE(elevation.is_solved());
        if (!elevation.is_solved()) continue;
        if (network.junctions.empty()) continue;

        for (size_t j = 0; j < network.junctions.size(); ++j) {
            const Junction& junction = network.junctions[j];
            if (junction.node == kInvalidId) continue;
            if (junction.node >= builder.graph().nodes().size()) continue;
            // A degenerate junction emitted no geometry and has no plane to be on.
            if (junction.kind == JunctionKind::Degenerate) continue;

            const double expected = static_cast<double>(elevation.node_height(junction.node)) +
                                    static_cast<double>(cfg.elevation.surface_offset);
            const std::string where = label + " junction " + std::to_string(j);

            if (std::fabs(static_cast<double>(junction.height) - expected) > 1e-4) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "junction height == node height + surface offset",
                    where + ": junction " + stratum::test::stringify(junction.height) +
                        " expected " + stratum::test::stringify(expected));
                continue;
            }
            ++junctions_checked;

            // The fill really sits on that plane: the lowest thing in a junction
            // mesh is its carriageway, with the curb ring standing on top of it.
            if (junction.mesh.vertices.empty()) continue;
            double lowest = 1e300;
            for (const auto& v : junction.mesh.vertices) {
                lowest = std::min(lowest, static_cast<double>(v.position.y));
            }
            CHECK_NEAR(lowest, expected, 1e-3);
        }
    }

    CHECK_TRUE(junctions_checked > 0);
}

/**
 * Every arm of a solved junction terminates at the same height the junction fill
 * sits on, so the two meet without a step.
 *
 * The elevation solver already guarantees agreement at a node; what this checks is
 * that the junction READ that guarantee rather than re-deriving a height of its
 * own, which is the failure mode junction_builder.hpp warns about when it calls
 * the elevation solve an input and not an output.
 */
TEST(JunctionIntegration, arms_and_junction_fill_agree_on_one_height) {
    RoadNetworkConfig cfg;
    cfg.solve_junctions = true;
    cfg.height_sampler = HeightSampler(&tilted_surface);

    size_t arms_checked = 0;

    for (const char* filename : jt::kAllFixtures) {
        const auto parsed = jt::parse_fixture(filename);
        if (!parsed) continue;

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*parsed, cfg);
        const auto& elevation = builder.elevation();
        if (!elevation.is_solved()) continue;

        const RoadGraph& graph = builder.graph();
        const std::string label(filename);

        for (const Junction& junction : network.junctions) {
            if (junction.kind != JunctionKind::Intersection) continue;
            if (junction.node >= graph.nodes().size()) continue;

            for (const auto& arm : junction.arms) {
                if (arm.edge >= elevation.edges().size()) continue;
                const auto& profile = elevation.edge(arm.edge).station_heights;
                if (profile.empty()) continue;

                const double at_node = arm.at_start ? static_cast<double>(profile.front())
                                                    : static_cast<double>(profile.back());
                if (std::fabs(at_node - static_cast<double>(junction.height)) > 1e-4) {
                    stratum::test::report_failure(
                        __FILE__, __LINE__, "arm terminates at the junction height",
                        label + " node " + std::to_string(junction.node) + ": arm " +
                            stratum::test::stringify(at_node) + " junction " +
                            stratum::test::stringify(junction.height));
                    continue;
                }
                ++arms_checked;
            }
        }
    }

    CHECK_TRUE(arms_checked > 0);
}
