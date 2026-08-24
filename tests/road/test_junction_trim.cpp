/**
 * @file test_junction_trim.cpp
 * @brief Trim-distance and cut-cross-section tests for the P4 junction solver
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Written against the contract in src/osm/road/junction_trim.hpp and the P4
 * section of docs/plans/road_network_plan.md.
 *
 * The trim distance is the first number P4 computes and every later number
 * depends on it. The junction polygon is chained from the cut cross-sections, the
 * curb ring is offset from that polygon, and the terrain re-carve uses the ring.
 * A trim that is wrong by a metre is not a metre of error downstream: it moves
 * every ring vertex, every fillet tangent and the whole carve footprint.
 *
 * ### The closed form
 *
 * For two arms leaving a node at an angle theta with equal carriageway half
 * widths h, the near-side offset lines meet at `h * cot(theta / 2)` along each
 * arm. Two consequences are used all through this file:
 *
 * - At theta = 90 degrees the demand is exactly h, so a symmetric crossroads
 *   trims every arm to `h + clearance`. That is the one case with no room for
 *   interpretation, and it is asserted to 1e-9.
 * - The demand a pair makes on an arm is the NEIGHBOUR's half width, not the
 *   arm's own. A narrow road crossing a wide one therefore retreats FURTHER than
 *   the wide road does, which is the opposite of the intuitive answer and the
 *   easiest sign error in the file to ship. It has its own test.
 *
 * ### Why these networks are synthetic
 *
 * See tests/road/junction_fixtures.hpp. A node placed at (0, 0) by hand really is
 * at (0, 0), so `h + clearance` can be asserted as an equality rather than as a
 * tolerance band.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests JunctionTrim
 * @endcode
 */

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/road/junction_trim.hpp"
#include "osm/road/road_graph.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace {

using stratum::osm::NodeId;
using stratum::osm::Road;
using stratum::osm::RoadType;
using stratum::osm::road::ArmEnd;
using stratum::osm::road::ArmRef;
using stratum::osm::road::Centerline;
using stratum::osm::road::EdgeId;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::Station;
using stratum::osm::road::TrimConfig;
using stratum::osm::road::arm_end;
using stratum::osm::road::collect_arms;
using stratum::osm::road::kInvalidId;
using stratum::osm::road::solve_arm_trims;

namespace jt = stratum::test::junction;

constexpr double kPi = 3.14159265358979323846;

/// Every trim must be a real, non-negative number whatever the input geometry
void check_trims_are_finite(const std::vector<ArmRef>& arms, const std::string& label) {
    for (size_t i = 0; i < arms.size(); ++i) {
        if (!std::isfinite(arms[i].trim) || arms[i].trim < 0.0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "arm trim is finite and non-negative",
                label + " arm " + std::to_string(i) + ": " +
                    stratum::test::stringify(arms[i].trim));
        }
    }
}

/**
 * @brief Position at an arclength along a centerline, by linear interpolation
 *
 * The independent re-measurement the curved-approach test compares ArmEnd::center
 * against. It walks the stations rather than calling slice(), so a slice() that
 * placed the cut somewhere else would be caught rather than confirmed.
 *
 * @param cl        Centerline to walk
 * @param arclength Arclength in the centerline's own parameterisation
 * @return The interpolated position; the first or last station when out of range
 */
glm::dvec2 position_at_arclength(const Centerline& cl, double arclength) {
    if (cl.stations.empty()) return glm::dvec2{0.0};
    if (arclength <= cl.stations.front().arclength) return cl.stations.front().position;
    if (arclength >= cl.stations.back().arclength) return cl.stations.back().position;

    for (size_t i = 0; i + 1 < cl.stations.size(); ++i) {
        const Station& a = cl.stations[i];
        const Station& b = cl.stations[i + 1];
        const double span = b.arclength - a.arclength;
        if (span <= 1e-12) continue;
        if (arclength >= a.arclength && arclength <= b.arclength) {
            const double u = (arclength - a.arclength) / span;
            return a.position + (b.position - a.position) * u;
        }
    }
    return cl.stations.back().position;
}

} // namespace

// ============================================================================
// The closed-form case
// ============================================================================

/**
 * Four equal arms at 0, 90, 180 and 270 degrees. Every adjacent pair meets at a
 * right angle, so `h * cot(45 degrees)` is exactly h, and the ring's turn over
 * each corner is also a right angle, so the fillet reserve is exactly its radius.
 * Every trim is therefore `carriageway_half + R + clearance`. If this number is
 * wrong nothing downstream can be right, so it is asserted as an equality rather
 * than as a band.
 */
TEST(JunctionTrim, symmetric_cross_trims_are_equal_and_closed_form) {
    const jt::Fixture fixture = jt::symmetric_cross(2);
    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 4);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    TrimConfig cfg;
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    CHECK_TRUE(jt::solve_node(fixture, node, cfg, arms, ends));
    CHECK_EQ(arms.size(), size_t{4});
    if (arms.size() != 4) return;

    // The fixture profile is lanes only, so total_width, the carriageway envelope
    // and twice half_width are all 7.0 m. There is exactly one width in play and
    // the expectation cannot be satisfied by reading the wrong one.
    const double half = jt::kLaneWidth;

    // The corner between two arms 90 degrees apart is a 90 degree turn for the
    // ring, so tan(theta/2) is 1 and the reserve is the radius itself: 5.25 m for
    // two 7 m carriageways at the shipping radius_width_factor of 0.75.
    const double reserve = jt::fillet_reserve(cfg, half, half, kPi * 0.5);
    CHECK_NEAR(reserve, 0.75 * 2.0 * half, jt::kExactEps);

    const double expected = half + reserve + cfg.clearance;

    for (size_t i = 0; i < arms.size(); ++i) {
        CHECK_NEAR(arms[i].carriageway_half, half, jt::kExactEps);
        CHECK_NEAR(arms[i].half_width, half, jt::kExactEps);
        CHECK_NEAR(arms[i].trim, expected, jt::kExactEps);
        CHECK_FALSE(arms[i].clamped);
        CHECK_TRUE(ends[i].valid);
    }

    // Equal to each other, not merely each equal to the constant: a solver that
    // mixed up its neighbours could hit the constant on average and still differ
    // arm to arm.
    for (size_t i = 1; i < arms.size(); ++i) {
        CHECK_NEAR(arms[i].trim, arms[0].trim, jt::kExactEps);
    }

    // Arms are handed over counter-clockwise, which every later step assumes.
    for (size_t i = 1; i < arms.size(); ++i) {
        CHECK_TRUE(arms[i].bearing >= arms[i - 1].bearing);
    }

    // The cut lands `trim` from the node along each arm, whichever end it is.
    for (size_t i = 0; i < arms.size(); ++i) {
        const glm::dvec2 node_pos = fixture.graph.node(node).position;
        CHECK_NEAR(glm::length(ends[i].center - node_pos), expected, 1e-6);
    }
}

/**
 * The sign test. A wide primary crossing a narrow residential: each arm must
 * retreat far enough to clear the OTHER road's edge, so the narrow arm -- which
 * has to clear the wide road -- is cut back FURTHER than the wide arm.
 *
 * A solver that used its own half width instead of its neighbour's produces the
 * reverse ordering and a junction that still looks plausible in a screenshot.
 */
TEST(JunctionTrim, asymmetric_cross_narrow_arm_trims_further_than_wide) {
    const double primary_half = 7.0;
    const double residential_half = 3.0;
    const jt::Fixture fixture = jt::asymmetric_cross(primary_half, residential_half);
    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 4);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    TrimConfig cfg;
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    CHECK_TRUE(jt::solve_node(fixture, node, cfg, arms, ends));
    CHECK_EQ(arms.size(), size_t{4});
    if (arms.size() != 4) return;

    double widest_arm_trim = -1.0;
    double narrowest_arm_trim = -1.0;
    size_t wide_arms = 0;
    size_t narrow_arms = 0;

    for (const ArmRef& arm : arms) {
        if (std::fabs(arm.carriageway_half - primary_half) < 1e-9) {
            ++wide_arms;
            // Every wide arm agrees with every other wide arm.
            if (widest_arm_trim >= 0.0) CHECK_NEAR(arm.trim, widest_arm_trim, jt::kExactEps);
            widest_arm_trim = arm.trim;
        } else if (std::fabs(arm.carriageway_half - residential_half) < 1e-9) {
            ++narrow_arms;
            if (narrowest_arm_trim >= 0.0) {
                CHECK_NEAR(arm.trim, narrowest_arm_trim, jt::kExactEps);
            }
            narrowest_arm_trim = arm.trim;
        }
    }
    CHECK_EQ(wide_arms, size_t{2});
    CHECK_EQ(narrow_arms, size_t{2});

    // THE assertion of this test, stated as an ordering so it cannot be satisfied
    // by coincidence: the narrow road retreats further.
    CHECK_TRUE(narrowest_arm_trim > widest_arm_trim);

    // And the closed form behind that ordering: each arm's demand is its
    // neighbour's half width, at 90 degrees, plus the fillet reserve both arms of
    // a pair share. The reserve is the same on all four arms here because it is
    // derived from the NARROWER of the two carriageways and every pair is
    // primary-with-residential, so it cannot be what produces the ordering.
    const double reserve = jt::fillet_reserve(cfg, primary_half, residential_half, kPi * 0.5);
    CHECK_NEAR(widest_arm_trim, residential_half + reserve + cfg.clearance, jt::kExactEps);
    CHECK_NEAR(narrowest_arm_trim, primary_half + reserve + cfg.clearance, jt::kExactEps);
}

// ============================================================================
// Real topology
// ============================================================================

/**
 * The T-junction on an interior shared node, from tests/data/t_junction.osm. This
 * is the fixture the whole graph phase exists for; here it only has to prove that
 * a trim solve over real parsed geometry and tag-derived profiles produces three
 * usable numbers.
 */
TEST(JunctionTrim, t_junction_fixture_trims_are_positive_and_finite) {
    const auto parsed = jt::parse_fixture("t_junction.osm");
    if (!parsed) return;

    RoadGraph graph;
    graph.build(*parsed);
    const std::vector<Centerline> centerlines = jt::make_centerlines(graph);
    const auto profiles = jt::profiles_from_tags(graph, *parsed);

    const GraphNodeId node = jt::node_with_osm_id(graph, static_cast<NodeId>(203));
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;
    CHECK_EQ(graph.node(node).degree(), size_t{3});

    std::vector<ArmRef> arms = collect_arms(graph, profiles, node);
    CHECK_EQ(arms.size(), size_t{3});
    TrimConfig cfg;
    CHECK_TRUE(solve_arm_trims(graph, centerlines, node, arms, cfg));
    check_trims_are_finite(arms, "t_junction");

    for (size_t i = 0; i < arms.size(); ++i) {
        // Every arm of a real three-way junction has a neighbour it must clear,
        // so no trim may be zero.
        CHECK_TRUE(arms[i].trim > 0.0);
        CHECK_TRUE(arms[i].carriageway_half > 0.0);

        const double length = centerlines[arms[i].edge].length();
        CHECK_TRUE(arms[i].trim <= length * cfg.max_trim_fraction + jt::kExactEps);

        const ArmEnd end = arm_end(graph, centerlines, profiles, arms[i]);
        CHECK_TRUE(end.valid);
        CHECK_TRUE(std::isfinite(end.center.x) && std::isfinite(end.center.y));
        // The cut station is stated in the EDGE's parameterisation, not the arm's.
        const double expected_arclength = arms[i].at_start ? arms[i].trim
                                                           : length - arms[i].trim;
        CHECK_NEAR(end.arclength, expected_arclength, 1e-9);
    }
}

// ============================================================================
// Degenerate angles
// ============================================================================

/**
 * Two arms 15 degrees apart. `cot(7.5 degrees)` is about 7.6, so the demand is
 * roughly seven and a half carriageway half widths -- large, but a finite number
 * the clamp never has to touch on an arm this long.
 */
TEST(JunctionTrim, acute_fork_trim_is_large_finite_and_bounded) {
    const double arm_length = 400.0;
    const jt::Fixture fixture = jt::fork(15.0, arm_length);
    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 3);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    // The pairwise rule alone. Every corner of this fork is a wide turn for the
    // ring -- 165 degrees at the sharp pair -- so the fillet reserve would add a
    // capped run to all three arms and swamp the cot(7.5 degrees) term this test
    // is about. The reserve has its own tests; this one isolates the intersection.
    const TrimConfig cfg = jt::pairwise_only_trim();
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    CHECK_TRUE(jt::solve_node(fixture, node, cfg, arms, ends));
    CHECK_EQ(arms.size(), size_t{3});
    if (arms.size() != 3) return;
    check_trims_are_finite(arms, "acute_fork");

    const double h = jt::kLaneWidth;
    const double theta = 15.0 * kPi / 180.0;
    const double expected = h / std::tan(theta * 0.5) + cfg.clearance;

    // The two arms 15 degrees apart both retreat by the acute demand of about
    // 26.8 m. The third arm's own two pairs are the 75 degree gap and the
    // wrap-around, neither of which demands anything like that, so a threshold of
    // three half widths separates them cleanly.
    size_t acute_arms = 0;
    for (size_t i = 0; i < arms.size(); ++i) {
        const double length = fixture.centerlines[arms[i].edge].length();
        const double clamp = length * cfg.max_trim_fraction;
        CHECK_TRUE(arms[i].trim <= clamp + jt::kExactEps);
        if (arms[i].trim > 3.0 * h) {
            ++acute_arms;
            CHECK_NEAR(arms[i].trim, expected, 1e-6);
            // Large, but not clamped: the whole point is that an acute fork is
            // still solved exactly when the arms are long enough to allow it.
            CHECK_FALSE(arms[i].clamped);
        }
    }
    CHECK_EQ(acute_arms, size_t{2});
    CHECK_TRUE(expected < arm_length * cfg.max_trim_fraction);
}

/**
 * Two arms 0.01 degrees apart. The angle is far above TrimConfig::parallel_epsilon,
 * so the exact formula would still apply and would demand about 11,000 half
 * widths. TrimConfig::min_pair_angle is what catches it: the pair is solved as
 * though its arms were 15 degrees apart, which bounds the demand at
 * `(wa + wb) / sin(min_pair_angle)` plus the fillet reserve.
 *
 * The bound is asserted rather than the clamp. Before the angle floor existed the
 * only thing standing between this fixture and a kilometre-long junction was
 * TrimConfig::max_trim_fraction, which on a 400 m arm still let 160 m of trunk
 * road be cut away for a fork; the floor now stops it at 38 m and nothing is
 * clamped at all.
 *
 * The finiteness assertion is still the point underneath. An infinity here
 * reaches the corridor extruder as a slice range and comes out of the GPU as a
 * corrupt bounding box that breaks culling for a whole chunk, with nothing in the
 * log to trace.
 */
TEST(JunctionTrim, near_parallel_arms_stay_finite_and_bounded) {
    const double arm_length = 400.0;
    const jt::Fixture fixture = jt::fork(0.01, arm_length);
    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 3);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    TrimConfig cfg;
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    CHECK_TRUE(jt::solve_node(fixture, node, cfg, arms, ends));
    CHECK_EQ(arms.size(), size_t{3});
    if (arms.size() != 3) return;

    check_trims_are_finite(arms, "near_parallel");
    for (size_t i = 0; i < arms.size(); ++i) {
        CHECK_TRUE(std::isfinite(arms[i].trim));
        CHECK_FALSE(std::isinf(arms[i].trim));
        CHECK_FALSE(std::isnan(arms[i].trim));

        const double length = fixture.centerlines[arms[i].edge].length();
        CHECK_TRUE(arms[i].trim <= length * cfg.max_trim_fraction + jt::kExactEps);
        CHECK_TRUE(arms[i].trim >= cfg.min_trim - jt::kExactEps);

        CHECK_TRUE(std::isfinite(ends[i].center.x));
        CHECK_TRUE(std::isfinite(ends[i].center.y));
        CHECK_TRUE(std::isfinite(ends[i].carriage_left.x));
        CHECK_TRUE(std::isfinite(ends[i].carriage_right.y));
    }

    // The two near-parallel arms are bounded by the angle floor, not by the
    // max_trim_fraction clamp: on a 400 m arm the floor's bound is an order of
    // magnitude inside it, so nothing is clamped and no ribbon is over-trimmed.
    const double h = jt::carriageway_half_of(fixture.profiles[arms[0].edge]);
    const double bound = (h + h) / std::sin(cfg.min_pair_angle) +
                         jt::fillet_reserve(cfg, h, h, 3.14159265358979323846) + cfg.clearance;
    CHECK_TRUE(bound < arm_length * cfg.max_trim_fraction);

    size_t clamped = 0;
    for (const ArmRef& arm : arms) {
        if (arm.clamped) ++clamped;
        CHECK_TRUE(arm.trim <= bound + jt::kExactEps);
    }
    CHECK_EQ(clamped, size_t{0});
}

/**
 * Two arms co-directional to within TrimConfig::parallel_epsilon. There is no
 * intersection to project, and the contract names the fallback explicitly: the
 * pair contributes `wa + wb`, a bounded number derived from the widths.
 *
 * This is the case the epsilon exists for, and it is separate from the
 * near-parallel test above because 0.01 degrees does NOT reach it -- sin(0.01
 * degrees) is 1.7e-4, over a hundred times the default epsilon. The angle here is
 * 1e-7 radians.
 */
TEST(JunctionTrim, codirectional_arms_fall_back_to_the_width_sum) {
    const double length = 400.0;
    const double tiny = 1e-7;
    const std::vector<Road> roads = {
        jt::make_road(1, {100, 1}, {{0.0, 0.0}, {length, 0.0}}),
        jt::make_road(2, {100, 2},
                      {{0.0, 0.0}, {length * std::cos(tiny), length * std::sin(tiny)}}),
        jt::make_road(3, {100, 3}, {{0.0, 0.0}, {-length, 0.0}}),
    };
    const jt::Fixture fixture =
        jt::make_fixture(roads, {jt::lane_profile(2), jt::lane_profile(2), jt::lane_profile(2)});

    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 3);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    TrimConfig cfg;
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    CHECK_TRUE(jt::solve_node(fixture, node, cfg, arms, ends));
    CHECK_EQ(arms.size(), size_t{3});
    if (arms.size() != 3) return;
    check_trims_are_finite(arms, "codirectional");

    const double h = jt::kLaneWidth;
    const double expected_pair = h + h + cfg.clearance;

    // The third arm is demanded nothing by either of its pairs, so it lands on
    // the floor -- which is min_trim PLUS the clearance, because the clearance is
    // added to every solved trim and not only to the ones a pair drove. That is
    // the same unconditional clearance the straight arms of
    // curved_approach_trim_is_measured_along_the_arc are pinned to.
    const double floor_trim = cfg.min_trim + cfg.clearance;

    size_t on_fallback = 0;
    size_t at_floor = 0;
    for (const ArmRef& arm : arms) {
        if (std::fabs(arm.trim - expected_pair) < 1e-6) {
            ++on_fallback;
            // A width-derived fallback is not a clamp; nothing was reduced.
            CHECK_FALSE(arm.clamped);
        } else if (std::fabs(arm.trim - floor_trim) < 1e-9) {
            ++at_floor;
        }
    }
    // The two co-directional arms take the width sum; the third arm's only
    // non-parallel neighbour is one of them, and both its pairs are anti-parallel,
    // so it stays at the floor.
    CHECK_EQ(on_fallback, size_t{2});
    CHECK_EQ(at_floor, size_t{1});
}

// ============================================================================
// Short edges
// ============================================================================

/**
 * A 6 m edge running between two junctions of wide primaries. Both ends demand
 * 7.25 m, which together would consume 14.5 m of a 6 m edge and leave a
 * negative-length ribbon. The clamps must stop that.
 *
 * The assertion is stated as an either/or, exactly as the contract allows: the
 * combined trim leaves a positive remaining length, OR the arms are reported
 * clamped. A negative remaining length is never acceptable, and that is asserted
 * unconditionally.
 */
TEST(JunctionTrim, short_edge_between_wide_junctions_keeps_positive_length) {
    const double short_length = 6.0;
    const double wing = 150.0;
    const std::vector<Road> roads = {
        jt::make_road(1, {100, 101}, {{0.0, 0.0}, {short_length, 0.0}}),
        jt::make_road(2, {200, 100, 201},
                      {{0.0, -wing}, {0.0, 0.0}, {0.0, wing}}, RoadType::Primary),
        jt::make_road(3, {300, 101, 301},
                      {{short_length, -wing}, {short_length, 0.0}, {short_length, wing}},
                      RoadType::Primary),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads, {jt::lane_profile(2, 3.5), jt::lane_profile(2, 7.0), jt::lane_profile(2, 7.0)});

    // The short edge is the one whose centerline is 6 m long.
    EdgeId short_edge = kInvalidId;
    for (size_t e = 0; e < fixture.graph.edges().size(); ++e) {
        if (std::fabs(fixture.centerlines[e].length() - short_length) < 1e-6) {
            short_edge = static_cast<EdgeId>(e);
        }
    }
    CHECK(short_edge != kInvalidId);
    if (short_edge == kInvalidId) return;

    TrimConfig cfg;
    double trim_from = 0.0;
    double trim_to = 0.0;
    bool clamped_either_end = false;

    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (!fixture.graph.node(static_cast<GraphNodeId>(n)).is_junction()) continue;
        std::vector<ArmRef> arms;
        std::vector<ArmEnd> ends;
        CHECK_TRUE(jt::solve_node(fixture, static_cast<GraphNodeId>(n), cfg, arms, ends));
        check_trims_are_finite(arms, "short_edge node " + std::to_string(n));

        for (const ArmRef& arm : arms) {
            if (arm.edge != short_edge) continue;
            if (arm.at_start) {
                trim_from = arm.trim;
            } else {
                trim_to = arm.trim;
            }
            clamped_either_end = clamped_either_end || arm.clamped;
        }
    }

    const double remaining = short_length - trim_from - trim_to;

    // Never negative. This is the failure the whole clamp exists to prevent and
    // it is unconditional.
    CHECK_TRUE(remaining >= 0.0);

    // Positive, or the arms are honestly reported as clamped -- never neither.
    CHECK_TRUE(remaining > 0.0 || clamped_either_end);

    // The demand really did exceed what the edge could give, so this fixture is
    // exercising the clamp rather than passing by having nothing to clamp.
    CHECK_TRUE(clamped_either_end);
    CHECK_TRUE(trim_from <= short_length * cfg.max_trim_fraction + jt::kExactEps);
    CHECK_TRUE(trim_to <= short_length * cfg.max_trim_fraction + jt::kExactEps);
    CHECK_TRUE(remaining > 0.0);
}

// ============================================================================
// Curved approaches
// ============================================================================

/**
 * An arm that curves hard within a few metres of the node.
 *
 * The pairwise demand is a distance along the arm's straight leaving RAY. A
 * curving arm peels away from that ray, so it has to travel further along its own
 * centerline before its projection reaches the demand: the trim ARCLENGTH is
 * strictly greater than the straight-line demand, and the cut point is closer to
 * the node than the arclength suggests.
 *
 * The cut position is then re-measured by walking the centerline independently,
 * which is what proves ArmEnd::center is the point at ArmEnd::arclength rather
 * than the nearest resampled station.
 */
TEST(JunctionTrim, curved_approach_trim_is_measured_along_the_arc) {
    // A quarter circle of radius 6 leaving the node westward and bending south,
    // then a straight tail. Sampled at 0.02 rad so the polyline IS the arc to
    // within 3e-4 m and the expected arclength can be written down.
    const double radius = 6.0;
    std::vector<glm::dvec2> curved;
    for (double angle = 0.0; angle <= kPi * 0.5 + 1e-12; angle += 0.02) {
        curved.push_back(glm::dvec2{-radius * std::sin(angle),
                                    -radius * (1.0 - std::cos(angle))});
    }
    curved.push_back(glm::dvec2{-radius, -radius - 100.0});

    std::vector<NodeId> curved_ids;
    curved_ids.reserve(curved.size());
    curved_ids.push_back(100);
    for (size_t i = 1; i < curved.size(); ++i) {
        curved_ids.push_back(static_cast<NodeId>(500 + i));
    }

    const std::vector<Road> roads = {
        jt::make_road(1, {100, 1}, {{0.0, 0.0}, {200.0, 0.0}}),
        jt::make_road(2, {100, 2}, {{0.0, 0.0}, {0.0, 200.0}}),
        jt::make_road(3, curved_ids, curved),
    };

    // A tighter deviation bound than the shipping default, so the resampled
    // stations follow a 6 m radius closely enough for an exact expectation.
    stratum::osm::road::ResampleConfig resample = jt::fixture_resample();
    resample.max_deviation = 0.005;
    resample.min_spacing = 0.25;

    const jt::Fixture fixture = jt::make_fixture(
        roads, {jt::lane_profile(2), jt::lane_profile(2), jt::lane_profile(2)}, resample);

    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 3);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    // The pairwise rule alone. This test is about the straight-line demand being
    // converted into ARCLENGTH along a bending approach, and the fillet reserve
    // is added to the demand BEFORE that conversion, so leaving it on would fold
    // a second effect into the same number. The reserve has its own tests.
    const TrimConfig cfg = jt::pairwise_only_trim();
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    CHECK_TRUE(jt::solve_node(fixture, node, cfg, arms, ends));
    CHECK_EQ(arms.size(), size_t{3});
    if (arms.size() != 3) return;
    check_trims_are_finite(arms, "curved");

    // Identify the curved arm by its edge's station count: it is the only one
    // whose centerline needed more than a handful of stations.
    size_t curved_arm = arms.size();
    for (size_t i = 0; i < arms.size(); ++i) {
        if (fixture.graph.edge(arms[i].edge).polyline.size() > 3) curved_arm = i;
    }
    CHECK(curved_arm < arms.size());
    if (curved_arm >= arms.size()) return;

    const double h = jt::kLaneWidth;
    const double straight_trim = h + cfg.clearance;

    // The node ray is the ribbon's leaving direction, which is the terminal
    // SEGMENT of the resampled centerline and not the analytic tangent of the
    // circle. A chord of a circle bisects the angle it subtends, so the first
    // 0.02 rad step leaves the ray tilted `phi = 0.01` rad off the true tangent
    // (-1, 0), towards the inside of the bend. That tilt is not an error to be
    // absorbed by a loose tolerance: the ribbon really does leave along that
    // chord, arm_end() really does place the cut corners in that frame, and a
    // trim solved against the analytic tangent would not land on the ribbon.
    // So the closed form carries phi explicitly.
    const double phi = 0.5 * 0.02;

    // The pair that drives this arm is the north one. Intersecting north's LEFT
    // offset with the curved arm's RIGHT offset, both at h, with the curved arm
    // leaving along (-cos phi, -sin phi), puts the demand at h(1 - sin phi)/cos
    // phi along that ray. Its other pair -- the east arm, very nearly
    // anti-parallel -- asks for only h(1 - cos phi)/sin phi, about 0.018 m, and
    // loses the maximum.
    const double demand = h * (1.0 - std::sin(phi)) / std::cos(phi);

    // Projecting the arc onto that tilted ray:
    //     radius*sin(t)*cos(phi) + radius*(1 - cos(t))*sin(phi)
    //   = radius*sin(t - phi) + radius*sin(phi)
    // so the demand is met at t = phi + asin((demand - radius*sin(phi))/radius),
    // and the clearance is added along the ribbon afterwards.
    const double theta = phi + std::asin((demand - radius * std::sin(phi)) / radius);
    const double expected = radius * theta + cfg.clearance;

    // 2 mm, not the 20 mm this assertion used to allow. What is left is the
    // difference between the true arc and the chordal polyline the stations
    // actually are, which measures 1.2e-4 m here.
    CHECK_NEAR(arms[curved_arm].trim, expected, 0.002);

    // The whole point: the arclength is NOT the straight-line answer.
    CHECK_TRUE(arms[curved_arm].trim > straight_trim + 0.05);

    // And the straight arms still are, so the difference is the curve and not a
    // global offset.
    for (size_t i = 0; i < arms.size(); ++i) {
        if (i == curved_arm) continue;
        CHECK_NEAR(arms[i].trim, straight_trim, 1e-6);
    }

    const ArmEnd& end = ends[curved_arm];
    CHECK_TRUE(end.valid);

    // Re-measure: the cut centre is the point at ArmEnd::arclength along the
    // centerline, found by walking the stations rather than by asking slice().
    const Centerline& cl = fixture.centerlines[arms[curved_arm].edge];
    const glm::dvec2 remeasured = position_at_arclength(cl, end.arclength);
    CHECK_NEAR(glm::length(end.center - remeasured), 0.0, 1e-6);

    // A chord is shorter than the arc it subtends, so the cut sits closer to the
    // node than its arclength. On a 6 m radius that gap is 60 mm, far above noise.
    const glm::dvec2 node_pos = fixture.graph.node(node).position;
    const double chord = glm::length(end.center - node_pos);
    CHECK_TRUE(chord < arms[curved_arm].trim - 0.02);
}

// ============================================================================
// Cut cross-sections
// ============================================================================

/**
 * ArmEnd's left and right are relative to the direction LEAVING the node. For an
 * arm at the `to` end of its edge that direction is the reverse of travel, so the
 * profile's own left and right come out SWAPPED.
 *
 * The fixture road carries a 2 m sidewalk on its left and a 1 m sidewalk on its
 * right, so the profile is asymmetric about the carriageway and a missing flip is
 * visible as a swapped pair of distances. On a symmetric profile it would not be.
 */
TEST(JunctionTrim, arm_end_sides_invert_at_the_to_end_of_an_edge) {
    const double left_walk = 2.0;
    const double right_walk = 1.0;
    const double length = 200.0;

    const std::vector<Road> roads = {
        jt::make_road(1, {1, 100, 2}, {{-length, 0.0}, {0.0, 0.0}, {length, 0.0}}),
        jt::make_road(2, {3, 100, 4}, {{0.0, -length}, {0.0, 0.0}, {0.0, length}}),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads,
        {jt::sidewalk_profile(2, left_walk, right_walk), jt::lane_profile(2)});

    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 4);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    TrimConfig cfg;
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    CHECK_TRUE(jt::solve_node(fixture, node, cfg, arms, ends));
    CHECK_EQ(arms.size(), size_t{4});
    if (arms.size() != 4) return;

    const double h = jt::kLaneWidth;
    const double outer_left = h + left_walk;    // profile left edge, from the carriageway centre
    const double outer_right = h + right_walk;  // profile right edge

    size_t checked_start = 0;
    size_t checked_end = 0;

    for (size_t i = 0; i < arms.size(); ++i) {
        const ArmRef& arm = arms[i];
        const ArmEnd& end = ends[i];
        if (fixture.graph.edge(arm.edge).source_way != 1) continue;
        CHECK_TRUE(end.valid);

        const double left_reach = glm::length(end.left - end.center);
        const double right_reach = glm::length(end.right - end.center);

        // Left really is to the left of the LEAVING direction, whichever end.
        CHECK_TRUE(jt::cross2(end.center, end.center + end.direction, end.left) > 0.0);
        CHECK_TRUE(jt::cross2(end.center, end.center + end.direction, end.right) < 0.0);

        // The four cut corners are collinear and ordered right, carriage_right,
        // carriage_left, left along the cut line.
        CHECK_NEAR(jt::cross2(end.right, end.left, end.carriage_left), 0.0, 1e-9);
        CHECK_NEAR(jt::cross2(end.right, end.left, end.carriage_right), 0.0, 1e-9);
        CHECK_TRUE(glm::length(end.carriage_left - end.right) <
                   glm::length(end.left - end.right));
        CHECK_TRUE(glm::length(end.carriage_right - end.right) <
                   glm::length(end.carriage_left - end.right));
        CHECK_NEAR(glm::length(end.carriage_left - end.center), h, 1e-6);
        CHECK_NEAR(glm::length(end.carriage_right - end.center), h, 1e-6);

        if (arm.at_start) {
            // Leaving direction equals travel: the profile's left stays left.
            CHECK_NEAR(left_reach, outer_left, 1e-6);
            CHECK_NEAR(right_reach, outer_right, 1e-6);
            ++checked_start;
        } else {
            // Leaving direction is the reverse of travel: the sides are swapped.
            CHECK_NEAR(left_reach, outer_right, 1e-6);
            CHECK_NEAR(right_reach, outer_left, 1e-6);
            ++checked_end;
        }
    }

    // Way 1 splits at the junction into one edge arriving and one leaving, so
    // both branches above were actually taken.
    CHECK_EQ(checked_start, size_t{1});
    CHECK_EQ(checked_end, size_t{1});
}
