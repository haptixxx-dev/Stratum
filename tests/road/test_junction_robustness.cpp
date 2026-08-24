/**
 * @file test_junction_robustness.cpp
 * @brief The junction solver against the shapes real OSM data actually contains
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The other P4 suites test the solver's contract on clean junctions: a symmetric
 * crossroads, a T, a fork wide enough to have a corner. This one tests what a
 * real extract is made of instead -- forks a few degrees wide, nodes a hand's
 * breadth apart, edges shorter than the junctions at their ends, ten arms on one
 * node, a motorway meeting a footpath, an arm that leaves north and is heading
 * east twenty metres later.
 *
 * ### One invariant runs through all of it
 *
 * A junction polygon must be a SIMPLE ring that contains its own node.
 *
 * Simple, because a self-crossing ring has no interior: earcut's output for it is
 * arbitrary, triangulate_junction() falls back to the convex hull, build_curb_ring()
 * refuses it outright, and a winding test against it punches holes in the terrain.
 * JunctionPolygon::self_intersecting exists to report exactly that, so every case
 * here asserts both the flag and an independent crossing sweep, and they must agree.
 *
 * Containing its node, because the node is where the arms meet. A polygon that
 * excludes it is not a junction: the ground under the meeting point is filled by
 * nothing and carved by nothing, and the fill sits off to one side of the roads it
 * is supposed to join. That was the visible symptom of the reflex corner being
 * closed with its chord.
 *
 * ### Measured, not guessed
 *
 * The numbers in the case comments come from /home/sarah/Downloads/lucan.osm --
 * 10,326 ways, 18,443 graph nodes, 12,543 junctions. Before the work these tests
 * cover, that extract produced 203 self-intersecting junction rings, a solved trim
 * of 294 m, and a junction polygon covering 38,394 m^2 whose ring ran 2.5 km from
 * its node. Those three numbers are why each fixture below is shaped the way it is.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests JunctionRobustness
 * @endcode
 */

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/road/junction_curb.hpp"
#include "osm/road/junction_polygon.hpp"
#include "osm/road/junction_trim.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace jt = stratum::test::junction;

using stratum::osm::NodeId;
using stratum::osm::Road;
using stratum::osm::WayId;
using stratum::osm::road::ArmEnd;
using stratum::osm::road::ArmRef;
using stratum::osm::road::EdgeId;
using stratum::osm::road::FilletConfig;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::JunctionPolygon;
using stratum::osm::road::kInvalidId;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::TrimConfig;
using stratum::osm::road::build_junction_polygon;
using stratum::osm::road::triangulate_junction;
using stratum::osm::road::collect_arms;
using stratum::osm::road::arm_end;
using stratum::osm::road::solve_arm_trims;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Trim tolerances as JunctionBuilder configures them: with the fillet reserve on
TrimConfig shipping_trim() {
    TrimConfig cfg;
    stratum::osm::road::apply_fillet_reserve(FilletConfig{}, cfg);
    return cfg;
}

/// One solved junction, kept together so a failure can name the fixture
struct Solved {
    std::string label;
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    JunctionPolygon poly;
    glm::dvec2 node{0.0};
    bool solved = false;
};

/**
 * @brief Solve one node all the way to its footprint
 *
 * @param label   Name for failure messages
 * @param fixture Built fixture
 * @param node    Node to solve
 * @param trim    Trim tolerances; defaults to the shipping ones
 * @param fillet  Corner rounding; defaults to the shipping ones
 * @return The solved junction
 */
Solved solve(const std::string& label, const jt::Fixture& fixture, GraphNodeId node,
             const TrimConfig& trim = shipping_trim(), const FilletConfig& fillet = {}) {
    Solved out;
    out.label = label;
    if (node == kInvalidId || node >= fixture.graph.nodes().size()) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture has the node asked for", label);
        return out;
    }
    out.node = fixture.graph.node(node).position;
    out.solved = jt::solve_node(fixture, node, trim, out.arms, out.ends);
    out.poly = build_junction_polygon(out.arms, out.ends, fillet);
    return out;
}

/**
 * @brief The invariant every junction has to satisfy, whatever the input
 *
 * Asserts, in order: the ring exists, it is counter-clockwise, it does not cross
 * itself, the flag agrees with an independent sweep, it contains the node, and
 * every coordinate is finite.
 *
 * @param s Solved junction to check
 */
void check_usable(const Solved& s) {
    CHECK_TRUE(s.poly.valid);
    if (!s.poly.valid) return;

    CHECK_TRUE(s.poly.ring.size() >= 3);
    CHECK_EQ(s.poly.arm_ring_start.size(), s.arms.size());

    for (const glm::dvec2& p : s.poly.ring) {
        CHECK_TRUE(std::isfinite(p.x));
        CHECK_TRUE(std::isfinite(p.y));
    }

    // The independent sweep and the flag must say the same thing. Trusting the
    // flag alone would let a solver that never sets it pass every case here.
    const bool simple = jt::ring_is_simple(s.poly.ring);
    CHECK_TRUE(simple);
    CHECK_EQ(s.poly.self_intersecting, !simple);

    CHECK_TRUE(jt::signed_area(s.poly.ring) > 0.0);
    CHECK_TRUE(jt::point_in_ring(s.poly.ring, s.node));
}

/// Largest distance from the node to any ring vertex, metres
double ring_reach(const Solved& s) {
    double reach = 0.0;
    for (const glm::dvec2& p : s.poly.ring) {
        reach = std::max(reach, glm::length(p - s.node));
    }
    return reach;
}

/// Three arms leaving one node at the given bearings, all the same width
jt::Fixture three_arms(double a_deg, double b_deg, double c_deg, double length = 400.0) {
    return jt::star({ a_deg, b_deg, c_deg }, length);
}

} // namespace

// ============================================================================
// 1. Very acute forks
// ============================================================================

/**
 * A trunk road running east-west with a slip road leaving it at a few degrees.
 *
 * The pairwise rule's exact answer for two arms `theta` apart is
 * `(wb + wa cos theta) / sin theta`, which diverges as `1 / theta`: at 5 degrees
 * two 7 m carriageways demand 80 m of trim, at 2 degrees 200 m, at half a degree
 * a kilometre. TrimConfig::min_pair_angle floors the angle used in that division,
 * so the demand is bounded by `(wa + wb) / sin(min_pair_angle)` plus the fillet
 * reserve however sharp the fork gets.
 *
 * Bounded is not the same as small: the two carriageways really do overlap for 80
 * m and the ribbons still do past the cut. What the bound buys is that the
 * junction POLYGON stays a junction-sized object instead of becoming a
 * 200-metre-long slab of asphalt, and that the trim stops eating whole approaches
 * -- at 2 degrees it used to take 160 m off a 400 m arm and hit the
 * max_trim_fraction clamp, which is the one condition that makes the ring cross
 * itself.
 */
TEST(JunctionRobustness, an_acute_fork_is_bounded_and_stays_simple) {
    const TrimConfig cfg = shipping_trim();

    // (wa + wb) / sin(15 degrees) + the reserve's own cap + clearance. The 15
    // degrees is written out rather than read from the config: this is the
    // EXPECTATION, and taking it from the value under test makes the assertion
    // vacuous the moment that value goes to zero.
    const double h = jt::kLaneWidth;   // one lane each side: carriageway half = 3.5
    const double bound = (h + h) / std::sin(15.0 * kPi / 180.0) +
                         jt::fillet_reserve(cfg, h, h, kPi) + cfg.clearance;
    CHECK_NEAR(cfg.min_pair_angle, 15.0 * kPi / 180.0, 1e-9);

    for (double degrees : { 0.5, 2.0, 5.0, 10.0, 15.0 }) {
        const jt::Fixture fixture = three_arms(0.0, degrees, 180.0);
        const Solved s = solve("slip fork " + std::to_string(degrees), fixture,
                               jt::sole_node_of_degree(fixture.graph, 3));
        CHECK_TRUE(s.solved);
        check_usable(s);

        for (const ArmRef& arm : s.arms) {
            CHECK_TRUE(arm.trim <= bound + jt::kExactEps);
            CHECK_TRUE(std::isfinite(arm.trim));
        }
        CHECK_TRUE(ring_reach(s) <= bound + h + jt::kExactEps);
    }
}

/**
 * The same fork with a third arm at a right angle, which is what a slip road
 * leaving a crossroads looks like.
 *
 * This shape is why the reflex corner matters. All three arms leave within one
 * half plane, so the ring's wrap-around corner spans more than 180 degrees, and
 * closing it with its chord draws a diagonal from the far cut face across the
 * junction. The diagonal passes on the wrong side of the node -- so the polygon
 * does not contain it -- and crosses the fillet of the corner between the other
 * two arms. Taking the ring through the corner point instead wraps it around the
 * back of the node, which is where the carriageways actually meet.
 */
TEST(JunctionRobustness, an_acute_fork_with_a_third_arm_wraps_behind_the_node) {
    for (double degrees : { 5.0, 10.0, 15.0, 30.0 }) {
        const jt::Fixture fixture = jt::fork(degrees);
        const Solved s = solve("fork " + std::to_string(degrees), fixture,
                               jt::sole_node_of_degree(fixture.graph, 3));
        CHECK_TRUE(s.solved);
        check_usable(s);

        // The back of the junction has to lie behind the node, opposite the two
        // acute arms. With arms leaving along +x, +x-ish and +y, that means at
        // least one ring vertex with a negative x.
        double min_x = 1e30;
        for (const glm::dvec2& p : s.poly.ring) {
            min_x = std::min(min_x, p.x - s.node.x);
        }
        CHECK_TRUE(min_x < 0.0);
    }
}

/**
 * The corner point of two arms whose cut faces are nearly parallel and far apart.
 *
 * This is the real node 1754404674 out of a Dublin extract, arm for arm: three
 * arms, two of them leaving 0.26 degrees apart, one cut at 2.7 m and the other --
 * a curving approach whose cut face is thirty metres off to the side -- at 50 m.
 * Its two tangent lines are so nearly parallel that they meet 4.5 KILOMETRES
 * behind the node, and the fillet built around that meeting point is tangent over
 * `radius * tan(theta / 2)`, which for a ring turn of 179.7 degrees is 380 times
 * the radius. The ring left the map and came back: 38,394 m^2 of junction polygon,
 * the largest in the extract by a factor of forty.
 *
 * FilletConfig::max_corner_reach_factor refuses it. A corner point further behind
 * a cut face than that arm's own trim plus a carriageway is not a corner of this
 * junction, and the corner falls back to its chord.
 *
 * The arms are built here rather than solved from a fixture because the shape is
 * the point: build_junction_polygon() is a pure function of the arms and their cut
 * cross-sections, and these are the ones that broke.
 */
TEST(JunctionRobustness, a_corner_point_a_kilometre_away_is_not_a_corner) {
    struct Spec {
        glm::dvec2 dir;
        glm::dvec2 carriage_right;
        glm::dvec2 carriage_left;
        double trim;
    };
    // Ascending bearing order, as collect_arms() produces. Metres, relative to
    // the node, which sits at the origin.
    const Spec spec[3] = {
        { { -0.920, -0.391 }, { -8.504, -0.360 }, { -6.430, -5.455 }, 8.0 },
        { { 0.931, 0.366 }, { 3.510, -1.574 }, { 1.497, 3.544 }, 2.7 },
        { { 0.929, 0.370 }, { 16.394, 31.463 }, { 14.355, 36.571 }, 50.0 },
    };

    Solved s;
    s.label = "38,394 square metre junction";
    s.node = glm::dvec2(0.0);
    for (const Spec& in : spec) {
        ArmRef arm;
        arm.edge = static_cast<EdgeId>(s.arms.size());
        arm.at_start = true;
        arm.bearing = std::atan2(in.dir.y, in.dir.x);
        arm.carriageway_half = 2.75;
        arm.half_width = 2.75;
        arm.trim = in.trim;
        s.arms.push_back(arm);

        ArmEnd end;
        end.direction = glm::normalize(in.dir);
        end.carriage_right = in.carriage_right;
        end.carriage_left = in.carriage_left;
        end.left = in.carriage_left;
        end.right = in.carriage_right;
        end.center = (in.carriage_left + in.carriage_right) * 0.5;
        end.arclength = in.trim;
        end.valid = true;
        s.ends.push_back(end);
    }

    s.poly = build_junction_polygon(s.arms, s.ends, FilletConfig{});
    s.solved = true;
    check_usable(s);

    // The junction stays a junction-sized object. Unbounded, this ring reaches
    // 2.5 km and encloses 38,394 m^2.
    CHECK_TRUE(ring_reach(s) < 100.0);
    CHECK_TRUE(std::abs(jt::signed_area(s.poly.ring)) < 2000.0);
}

// ============================================================================
// 2. Junctions too close together
// ============================================================================

/**
 * Two junctions joined by an edge far shorter than either of them is wide: the
 * staggered crossroads, the dual carriageway terminus, the service road stub.
 *
 * Neither end can be trimmed to what its pair demands, because
 * TrimConfig::max_trim_fraction leaves the edge some length and the caller's joint
 * budget leaves it more. Both arms are then cut SHORT of the point where their
 * carriageways separate, and their two cut faces cross -- which is a bowtie, and
 * on the Dublin extract was the shape of 178 of the 203 self-intersecting rings.
 *
 * Clipping the two faces against each other at their crossing point is what keeps
 * the ring simple. The ribbons still overlap the fill, which is what over-trimmed
 * means, but the fill is a polygon a curb ring and a terrain carve can use.
 */
TEST(JunctionRobustness, over_trimmed_arms_still_produce_a_simple_ring) {
    // Two T-junctions 3 m apart on one through road, each with its own side road.
    // The through edge between them is 3 m; both junctions want 9.
    const std::vector<Road> roads = {
        jt::make_road(1, { 1, 10, 11, 2 }, { { -200.0, 0.0 }, { 0.0, 0.0 }, { 3.0, 0.0 }, { 200.0, 0.0 } }),
        jt::make_road(2, { 3, 10 }, { { 0.0, -200.0 }, { 0.0, 0.0 } }),
        jt::make_road(3, { 4, 11 }, { { 3.0, 200.0 }, { 3.0, 0.0 } }),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(2), jt::lane_profile(2), jt::lane_profile(2) });

    size_t junctions = 0;
    size_t clamped_arms = 0;
    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (fixture.graph.nodes()[n].degree() < 3) continue;
        ++junctions;

        const Solved s = solve("staggered T node " + std::to_string(n), fixture,
                               static_cast<GraphNodeId>(n));
        CHECK_TRUE(s.solved);
        check_usable(s);

        for (const ArmRef& arm : s.arms) {
            if (arm.clamped) ++clamped_arms;
        }
    }

    CHECK_EQ(junctions, size_t{2});
    // The point of the fixture: the short edge really is over-trimmed at both
    // ends, so the ring above was built from crossing cut faces.
    CHECK_TRUE(clamped_arms >= size_t{2});
}

/**
 * Two junction nodes a few centimetres apart, joined by a stub edge.
 *
 * RoadGraph merges duplicate nodes only when their positions agree to 1e-6 m, so
 * a pair 2 cm apart -- a way split twice at what was meant to be one point, two
 * mappers tracing one crossroads -- survives as two graph nodes. Solved
 * separately they produce two junction polygons on the same patch of ground: two
 * fills, two curb rings, two carve footprints, all overlapping.
 *
 * collect_arms() gives the cluster's arms to its lowest-numbered member and
 * nothing at all to the other, so the pair comes out as ONE junction with all
 * four approaches -- which is what the same crossroads produces when the two
 * nodes coincide exactly.
 */
TEST(JunctionRobustness, near_coincident_junction_nodes_solve_as_one) {
    const double gap = 0.02;
    const std::vector<Road> roads = {
        jt::make_road(1, { 1, 10, 11, 2 }, { { -200.0, 0.0 }, { 0.0, 0.0 }, { gap, 0.0 }, { 200.0, 0.0 } }),
        jt::make_road(2, { 3, 10 }, { { 0.0, -200.0 }, { 0.0, 0.0 } }),
        jt::make_road(3, { 4, 11 }, { { gap, 200.0 }, { gap, 0.0 } }),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(2), jt::lane_profile(2), jt::lane_profile(2) });

    size_t with_geometry = 0;
    size_t suppressed = 0;
    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (fixture.graph.nodes()[n].degree() < 3) continue;

        std::vector<ArmRef> arms =
            collect_arms(fixture.graph, fixture.profiles, static_cast<GraphNodeId>(n));
        if (arms.empty()) {
            ++suppressed;
            continue;
        }

        ++with_geometry;

        // The primary carries every approach of both nodes, and NOT the stub that
        // held them together.
        CHECK_EQ(arms.size(), size_t{4});

        const Solved s = solve("merged pair", fixture, static_cast<GraphNodeId>(n));
        CHECK_TRUE(s.solved);
        check_usable(s);

        // Nothing is clamped: with the stub gone, every arm is a full approach.
        for (const ArmRef& arm : s.arms) {
            CHECK_FALSE(arm.clamped);
        }
    }

    CHECK_EQ(with_geometry, size_t{1});
    CHECK_EQ(suppressed, size_t{1});
}

/**
 * The merge must not fire on junctions that are merely close.
 *
 * A 12 m edge between two T-junctions is a short block, not a data defect, and
 * merging it would collapse two intersections a driver sees as two into one.
 */
TEST(JunctionRobustness, junctions_a_block_apart_are_not_merged) {
    const double gap = 12.0;
    const std::vector<Road> roads = {
        jt::make_road(1, { 1, 10, 11, 2 }, { { -200.0, 0.0 }, { 0.0, 0.0 }, { gap, 0.0 }, { 200.0, 0.0 } }),
        jt::make_road(2, { 3, 10 }, { { 0.0, -200.0 }, { 0.0, 0.0 } }),
        jt::make_road(3, { 4, 11 }, { { gap, 200.0 }, { gap, 0.0 } }),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(2), jt::lane_profile(2), jt::lane_profile(2) });

    size_t with_geometry = 0;
    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (fixture.graph.nodes()[n].degree() < 3) continue;
        const std::vector<ArmRef> arms =
            collect_arms(fixture.graph, fixture.profiles, static_cast<GraphNodeId>(n));
        CHECK_EQ(arms.size(), size_t{3});
        ++with_geometry;

        const Solved s = solve("block apart", fixture, static_cast<GraphNodeId>(n));
        check_usable(s);
    }
    CHECK_EQ(with_geometry, size_t{2});
}

/**
 * Merging is a property of the cluster, not of where the walk started, or two
 * members would disagree about which of them owns the junction -- and either both
 * would emit a fill, which is the artefact being removed, or neither would.
 *
 * Three junction nodes in a 30 cm chain, collected from each in turn.
 */
TEST(JunctionRobustness, every_member_of_a_cluster_agrees_on_the_primary) {
    const std::vector<Road> roads = {
        jt::make_road(1, { 1, 10, 11, 12, 2 },
                      { { -200.0, 0.0 }, { 0.0, 0.0 }, { 0.15, 0.0 }, { 0.30, 0.0 }, { 200.0, 0.0 } }),
        jt::make_road(2, { 3, 10 }, { { 0.0, -200.0 }, { 0.0, 0.0 } }),
        jt::make_road(3, { 4, 11 }, { { 0.15, 200.0 }, { 0.15, 0.0 } }),
        jt::make_road(4, { 5, 12 }, { { 0.30, -200.0 }, { 0.30, 0.0 } }),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(2), jt::lane_profile(2), jt::lane_profile(2), jt::lane_profile(2) });

    size_t with_geometry = 0;
    size_t total_arms = 0;
    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (fixture.graph.nodes()[n].degree() < 3) continue;
        const std::vector<ArmRef> arms =
            collect_arms(fixture.graph, fixture.profiles, static_cast<GraphNodeId>(n));
        if (arms.empty()) continue;
        ++with_geometry;
        total_arms = arms.size();

        const Solved s = solve("three-node cluster", fixture, static_cast<GraphNodeId>(n));
        CHECK_TRUE(s.solved);
        check_usable(s);
    }

    // Exactly one of the three emits a junction, and it carries all five
    // approaches: the through road's two ends and the three side roads.
    CHECK_EQ(with_geometry, size_t{1});
    CHECK_EQ(total_arms, size_t{5});
}

// ============================================================================
// 3. Many arms
// ============================================================================

/**
 * Five, six, eight, ten and twelve arms on one node.
 *
 * Complex urban intersections reach eight; a roundabout whose ring was mapped as
 * spokes to a centre point reaches whatever the mapper drew. The bearing-sorted
 * pairwise rule has to keep producing a simple ring, and the fillet radii have to
 * shrink as the gaps close so two adjacent corners never cross.
 *
 * Both an even star and a deliberately uneven one, because equal gaps give every
 * corner the same radius and hide an ordering error.
 */
TEST(JunctionRobustness, many_arms_stay_simple) {
    for (int count : { 5, 6, 8, 10, 12 }) {
        std::vector<double> bearings;
        for (int i = 0; i < count; ++i) {
            bearings.push_back(360.0 * static_cast<double>(i) / static_cast<double>(count));
        }
        const jt::Fixture fixture = jt::star(bearings, 300.0);
        const Solved s = solve(std::to_string(count) + " even arms", fixture,
                               jt::sole_node_of_degree(fixture.graph, static_cast<size_t>(count)));
        CHECK_TRUE(s.solved);
        check_usable(s);
        CHECK_EQ(s.arms.size(), static_cast<size_t>(count));
    }

    const jt::Fixture uneven = jt::star({ 0.0, 20.0, 44.0, 95.0, 130.0, 190.0, 250.0, 300.0 }, 300.0);
    const Solved s = solve("8 uneven arms", uneven, jt::sole_node_of_degree(uneven.graph, 8));
    CHECK_TRUE(s.solved);
    check_usable(s);

    // Every arm's two ring vertices are its own cut face, in right-then-left
    // order, and no two arms share one. A ring that welded two arms together
    // would put a seam at both their approaches.
    CHECK_EQ(s.poly.arm_ring_start.size(), s.arms.size());
    for (size_t k = 0; k + 1 < s.poly.arm_ring_start.size(); ++k) {
        CHECK_TRUE(s.poly.arm_ring_start[k] + 2 <= s.poly.arm_ring_start[k + 1]);
    }
}

// ============================================================================
// 4. Wildly unequal widths
// ============================================================================

/**
 * A six-lane motorway crossed by a 1 m footpath.
 *
 * The pairwise rule is symmetric in the two arms, so the risk is that the wide
 * road is cut back as though the narrow one mattered. At a right angle it must
 * not be: the motorway retreats by the FOOTPATH's half width, one metre, while
 * the footpath retreats by the motorway's eleven, and the junction is a notch in
 * a wide road rather than a 45 m square.
 *
 * The fillet radius is governed by the narrower arm for the same reason -- it is
 * the one a turning vehicle has to fit into -- so the corners here are the
 * footpath's, not the motorway's.
 */
TEST(JunctionRobustness, a_motorway_meeting_a_footpath_is_cut_by_the_footpath) {
    const double motorway_half = 3.75 * 3.0;    // 6 lanes at 3.75
    const double foot_half = 0.5;

    const std::vector<Road> roads = {
        jt::make_road(1, { 1, 100, 2 }, { { -400.0, 0.0 }, { 0.0, 0.0 }, { 400.0, 0.0 } },
                      stratum::osm::RoadType::Motorway),
        jt::make_road(2, { 100, 3 }, { { 0.0, 0.0 }, { 0.0, 400.0 } },
                      stratum::osm::RoadType::Footway),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(6, 3.75), jt::lane_profile(1, 1.0) });

    const Solved s = solve("motorway + footpath", fixture,
                           jt::sole_node_of_degree(fixture.graph, 3));
    CHECK_TRUE(s.solved);
    check_usable(s);
    CHECK_EQ(s.arms.size(), size_t{3});

    for (size_t i = 0; i < s.arms.size(); ++i) {
        const bool is_foot = s.arms[i].carriageway_half < 1.0;
        if (is_foot) {
            // The footpath has to clear the motorway's full half width.
            CHECK_TRUE(s.arms[i].trim >= motorway_half);
        } else {
            // The motorway retreats by the footpath's width and a fillet, not by
            // its own: a couple of metres, nowhere near its own half width.
            CHECK_TRUE(s.arms[i].trim < motorway_half * 0.5);
            CHECK_TRUE(s.arms[i].trim >= foot_half);
        }
    }
}

// ============================================================================
// 5. Curving approaches
// ============================================================================

/**
 * An arm that leaves the node heading north and is heading east forty metres
 * later, down to a radius tight enough that the whole approach turns inside the
 * junction's own reach.
 *
 * The pair demand is solved along the arm's LOCAL direction at the node, which is
 * a straight ray, so on a curving arm it has to be converted to an arclength
 * before the ribbon is cut -- the centerline peels away from the ray and reaches
 * the demand only after more arclength than the ray says. Getting that wrong cuts
 * a curving arm short and leaves a wedge of ribbon inside the junction polygon.
 *
 * The hairpin is the limit case: the arm turns back on itself and its projection
 * onto the node ray stops increasing altogether.
 */
TEST(JunctionRobustness, a_curving_approach_is_cut_at_the_right_arclength) {
    for (double radius : { 80.0, 40.0, 20.0, 10.0, 6.0 }) {
        std::vector<glm::dvec2> curve;
        std::vector<NodeId> ids;
        curve.push_back({ 0.0, 0.0 });
        ids.push_back(100);
        for (int i = 1; i <= 40; ++i) {
            const double a = (kPi * 0.9) * static_cast<double>(i) / 40.0;
            curve.push_back({ radius * (1.0 - std::cos(a)), radius * std::sin(a) });
            ids.push_back(static_cast<NodeId>(200 + i));
        }

        const std::vector<Road> roads = {
            jt::make_road(1, { 1, 100, 2 }, { { -200.0, 0.0 }, { 0.0, 0.0 }, { 200.0, 0.0 } }),
            jt::make_road(2, ids, curve),
        };
        const jt::Fixture fixture =
            jt::make_fixture(roads, { jt::lane_profile(2), jt::lane_profile(2) });

        const Solved s = solve("curving arm r=" + std::to_string(radius), fixture,
                               jt::sole_node_of_degree(fixture.graph, 3));
        CHECK_TRUE(s.solved);
        check_usable(s);

        // The cut lands ON the arm's own centerline, at its own trim arclength:
        // the arm end's centre is that far along the curve from the node, not
        // that far along the straight ray, so on a curve it is CLOSER to the node
        // in a straight line than the trim says.
        for (size_t i = 0; i < s.arms.size(); ++i) {
            const double straight = glm::length(s.ends[i].center - s.node);
            CHECK_TRUE(straight <= s.arms[i].trim + jt::kExactEps);
        }
    }
}

// ============================================================================
// 6. Ordering stability
// ============================================================================

/**
 * The same junction, described by ways listed in a different order.
 *
 * Every trim must come out identical. The pairwise rule takes the MAXIMUM over an
 * arm's two neighbours precisely so that the answer does not depend on which pair
 * is visited first, and a build that hashed differently run to run would make
 * every golden test in this directory meaningless.
 */
TEST(JunctionRobustness, trims_do_not_depend_on_the_order_the_ways_arrive_in) {
    const std::vector<double> bearings = { 0.0, 47.0, 118.0, 205.0, 300.0 };

    std::vector<double> first_pass;
    for (int rotation = 0; rotation < 3; ++rotation) {
        std::vector<Road> roads;
        std::vector<RoadProfile> profiles;
        for (size_t i = 0; i < bearings.size(); ++i) {
            const size_t which = (i + static_cast<size_t>(rotation)) % bearings.size();
            const double radians = bearings[which] * kPi / 180.0;
            roads.push_back(jt::make_road(static_cast<WayId>(which + 1),
                                          { 100, static_cast<NodeId>(which + 1) },
                                          { { 0.0, 0.0 },
                                            { 200.0 * std::cos(radians), 200.0 * std::sin(radians) } }));
            profiles.push_back(jt::lane_profile(2));
        }

        const jt::Fixture fixture = jt::make_fixture(roads, profiles);
        const Solved s = solve("rotation " + std::to_string(rotation), fixture,
                               jt::sole_node_of_degree(fixture.graph, 5));
        CHECK_TRUE(s.solved);
        check_usable(s);

        // Compare by (source way, trim) so the two runs are matched by identity
        // rather than by position in a list the rotation reordered.
        std::vector<double> by_way(bearings.size(), -1.0);
        for (const ArmRef& arm : s.arms) {
            const auto way = fixture.graph.edge(arm.edge).source_way;
            if (way >= 1 && way <= bearings.size()) {
                by_way[static_cast<size_t>(way) - 1] = arm.trim;
            }
        }

        if (rotation == 0) {
            first_pass = by_way;
        } else {
            CHECK_EQ(by_way.size(), first_pass.size());
            for (size_t i = 0; i < by_way.size() && i < first_pass.size(); ++i) {
                CHECK_NEAR(by_way[i], first_pass[i], jt::kExactEps);
            }
        }
    }
}

// ============================================================================
// 9. The clipping rule's own failure mode
// ============================================================================

/**
 * A ring that comes back CLOCKWISE is refused the same way a self-crossing one is.
 *
 * ### What was wrong
 *
 * The adjacent-face clipping added to build_junction_polygon() cuts two crossing
 * cut faces back to their crossing point. An arm clipped at BOTH ends can invert,
 * and the guard for that collapses the arm onto its own midpoint -- but it leaves
 * its two NEIGHBOURS holding the crossing points they were given. On a trident
 * node (three or more arms inside one narrow fan, nothing opposing them) every arm
 * inverts, the walk reverses between those neighbours, and the ring comes back
 * SIMPLE with negative area.
 *
 * `ring_self_intersects()` is right not to flag it -- the ring really is simple --
 * so `self_intersecting` stayed false, the convex-hull fallback that HEAD used for
 * the identical input no longer fired, and a backwards 0.22 m^2 sliver eight
 * metres from the node became the junction's asphalt fill AND the polygon handed
 * to the terrain carve as a winding test, while all three arms stayed trimmed
 * back by 8 m with nothing under them.
 *
 * ### What the fix is
 *
 * A clockwise ring is reported by `JunctionPolygon::inverted` and every consumer
 * asks `needs_hull_fallback()` instead of `self_intersecting` alone, so the hull
 * fill and the disc carve are available again for every input the clipping rule
 * -- or any future one -- inverts.
 *
 * ### How this test fails without the fix
 *
 * `needs_hull_fallback()` comes back false on a ring whose signed area is
 * negative, and triangulate_junction() fills the sliver instead of the hull.
 *
 * @note This is a degenerate-input guard, not a repair. The junction is not
 *       rendered correctly either way; what the fix restores is that it is
 *       rendered as a bounded approximation rather than as an inside-out sliver.
 *       Measured on /home/sarah/Downloads/lucan.osm, no real junction reaches it.
 */
TEST(JunctionRobustness, a_clockwise_ring_is_refused_like_a_crossing_one) {
    // Three residential ways inside a 30-degree fan, short enough that
    // TrimConfig::max_trim_fraction clamps every trim to 8 m. Every cut face then
    // crosses both of its neighbours and every arm is clipped at both ends.
    bool saw_inverted = false;

    const double fans[][3] = {
        { 0.0, 15.0, 30.0 },
        { 0.0, 10.0, 20.0 },
        { 0.0, 20.0, 40.0 },
    };

    for (const auto& fan : fans) {
        const jt::Fixture fixture = three_arms(fan[0], fan[1], fan[2], 20.0);
        const Solved s = solve("trident", fixture, jt::sole_node_of_degree(fixture.graph, 3));
        if (!s.solved || !s.poly.valid || s.poly.ring.size() < 3) continue;
        if (jt::signed_area(s.poly.ring) > 0.0) continue;

        saw_inverted = true;

        // The ring really is simple -- which is exactly why self_intersecting
        // cannot be the flag that catches it.
        CHECK_TRUE(jt::ring_is_simple(s.poly.ring));
        CHECK_FALSE(s.poly.self_intersecting);

        // ... and it must still be refused.
        CHECK_TRUE(s.poly.inverted);
        CHECK_TRUE(s.poly.needs_hull_fallback());

        // triangulate_junction() must still hand back a usable, positively wound
        // fill: it now takes the CONVEX HULL of the ring, exactly as it does for
        // a self-crossing one, instead of running earcut over a backwards ring
        // whose output is arbitrary.
        const stratum::Mesh fill =
            triangulate_junction(s.poly, 0.0f, stratum::MaterialId::Asphalt);
        CHECK_TRUE(fill.indices.size() >= 3);
        for (const stratum::Vertex& v : fill.vertices) {
            CHECK_TRUE(v.normal.y > 0.9f);   // every triangle faces +Y
        }

        // And the curb ring is refused rather than offset outward along a ring
        // that runs inward. It refused this input before the fix too -- through
        // its own signed-area guard one line later -- so this pins that the two
        // refusals agree rather than claiming new ground.
        const stratum::osm::road::CurbRing curb = stratum::osm::road::build_curb_ring(
            s.poly, s.arms, s.ends, 0.0f, stratum::osm::road::CurbRingConfig{});
        CHECK_TRUE(curb.outer.empty());
    }

    // If the fixtures stop producing an inverted ring the test has stopped
    // guarding anything, so say so rather than passing silently.
    CHECK_TRUE(saw_inverted);
}
