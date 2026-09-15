// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

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
    // Two T-junctions 10 m apart on one through road, each with its own side road.
    //
    // The gap was 3 m, and 3 m is now a MERGE rather than an over-trim: two
    // junctions of 3.5 m carriageway half-width standing 3 m apart are one
    // compound intersection and collect_arms() swallows the stub between them.
    // That is the better outcome for that geometry and it is covered by
    // a_staggered_pair_closer_than_their_own_width_solves_as_one below.
    //
    // 10 m keeps THIS path alive: above the 7 m merge threshold (3.5 + 3.5) so
    // the two stay separate, and still far short of what both ends demand, so the
    // clamp fires, the cut faces cross, and the bowtie clip is exercised.
    const std::vector<Road> roads = {
        jt::make_road(1, { 1, 10, 11, 2 }, { { -200.0, 0.0 }, { 0.0, 0.0 }, { 10.0, 0.0 }, { 200.0, 0.0 } }),
        jt::make_road(2, { 3, 10 }, { { 0.0, -200.0 }, { 0.0, 0.0 } }),
        jt::make_road(3, { 4, 11 }, { { 10.0, 200.0 }, { 10.0, 0.0 } }),
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

/**
 * @brief Two junctions closer together than their own width solve as ONE
 *
 * The staggered crossroads, the dual carriageway terminus, the service road
 * stub -- and above all the ordinary case of one dual carriageway crossing
 * another, which OSM leaves as four junction nodes in a rectangle 10 to 20 metres
 * across.
 *
 * Each used to solve its own polygon and its own kerb ring on the same patch of
 * ground, and the edges between them were far too short to absorb the trims all of
 * them demanded. On a Lucan extract that clamped 7505 of 24906 edges, which
 * TrimConfig::max_trim_fraction defines as the junction polygon overlapping that
 * ribbon. The visible result was a big intersection rendered as several
 * overlapping fills with fragments of kerb between them.
 *
 * The merge threshold is the SUM OF THE TWO JUNCTIONS' RADII rather than a
 * constant, because that is exactly the condition under which their polygons would
 * overlap: it merges the pairs that cannot be kept apart and needs no tuning per
 * city. junctions_a_block_apart_are_not_merged is the other half of that contract.
 */
TEST(JunctionRobustness, a_staggered_pair_closer_than_their_own_width_solves_as_one) {
    // The same shape the over-trim test used to carry: two T-junctions 3 m apart,
    // each 3.5 m of carriageway half-width, so the threshold is 7 m and 3 m is
    // comfortably inside it.
    const std::vector<Road> roads = {
        jt::make_road(1, { 1, 10, 11, 2 }, { { -200.0, 0.0 }, { 0.0, 0.0 }, { 3.0, 0.0 }, { 200.0, 0.0 } }),
        jt::make_road(2, { 3, 10 }, { { 0.0, -200.0 }, { 0.0, 0.0 } }),
        jt::make_road(3, { 4, 11 }, { { 3.0, 200.0 }, { 3.0, 0.0 } }),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(2), jt::lane_profile(2), jt::lane_profile(2) });

    // Both nodes are still degree 3: merging is a solving decision, not a change
    // to the graph.
    size_t degree_three = 0;
    for (const auto& node : fixture.graph.nodes()) {
        if (node.degree() >= 3) ++degree_three;
    }
    CHECK_EQ(degree_three, size_t{2});

    // Exactly ONE of them emits arms. The other is a non-primary cluster member
    // and emits none, which is how collect_arms() reports "already solved as part
    // of its neighbour".
    size_t emitting = 0;
    size_t arms_on_primary = 0;
    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (fixture.graph.nodes()[n].degree() < 3) continue;

        std::vector<ArmRef> arms;
        std::vector<GraphNodeId> cluster;
        arms = collect_arms(fixture.graph, fixture.profiles, static_cast<GraphNodeId>(n),
                            stratum::osm::road::kCoincidentRadius, &cluster);

        // Either way, both nodes agree on which of them is the primary.
        CHECK_EQ(cluster.size(), size_t{2});

        if (!arms.empty()) {
            ++emitting;
            arms_on_primary = arms.size();
        }
    }
    CHECK_EQ(emitting, size_t{1});

    // Four arms, not six: the two through-road halves and the two side roads. The
    // 3 m stub that joined the pair is internal and is dropped, which is the whole
    // point -- left in, it would be extruded as a ribbon running into the middle of
    // the junction that swallowed it.
    CHECK_EQ(arms_on_primary, size_t{4});
}

/**
 * @brief A dual carriageway's two mouths overlap without ever crossing
 *
 * The shape is taken off a Lucan extract, node 1232, which is what one of these
 * actually looks like: two junction nodes 2.9 m apart on a short link, each
 * carrying an eastbound and a westbound arm and one arm of its own. The width
 * rule merges them -- 1.5 m of carriageway half-width each puts the threshold at
 * 3.0 m -- so one junction is solved with six arms leaving from two points.
 *
 * The bearings are the extract's, not idealised: the two eastbound arms leave at
 * 7.0 and 5.6 degrees, not both at zero. Exactly parallel mouths are collinear
 * and the existing crossing clip catches them; 1.4 degrees apart is what real
 * data contains and what slips through it.
 *
 * ### The failure this pins
 *
 * The two eastbound mouths leave on the SAME bearing from origins 3 m apart, and
 * each is 3.5 m wide, so they overlap laterally by half a metre. They do not
 * CROSS: they are parallel and very nearly collinear, one simply lies along the
 * other. build_junction_polygon() clips adjacent cut faces that cross, which is
 * the common over-trimmed corner, and that clip correctly found nothing here.
 *
 * With nothing clipped, the ring walked out along the southern mouth's left
 * corner and back to the northern mouth's right corner, which lies BEHIND it --
 * a bowtie, so no fill, no kerb ring, and a convex hull thrown over the junction
 * instead. The same pair on the westbound side does it again at the step that
 * closes the ring, and that one reached five metres across the junction and back.
 *
 * On a Lucan extract this was the largest single cause of a crossing ring on a
 * merged junction. With it and the chain rule that follows it, merged junctions
 * that fall back to a hull went from 33 of 547 to 4 of 512. The lone junctions
 * are untouched, 7 before and 6 after, which is what says the repair reaches only
 * the shape it was written for -- it is skipped outright when every arm leaves
 * one node.
 *
 * @see JunctionRobustness.junctions_a_block_apart_are_not_merged for the other
 *      half of the merge contract.
 */
TEST(JunctionRobustness, two_parallel_carriageways_merge_into_a_simple_ring) {
    // Node 10 carries the southern carriageway, node 11 the northern, and the 3 m
    // link between them is the internal stub the merge swallows.
    const std::vector<Road> roads = {
        jt::make_road(1, { 10, 11 }, { { 0.0, -1.45 }, { 0.0, 1.45 } }),
        jt::make_road(2, { 10, 1 },  { { 0.0, -1.45 }, { 198.51, 22.92 } }),
        jt::make_road(3, { 10, 2 },  { { 0.0, -1.45 }, { -199.65, -13.31 } }),
        jt::make_road(4, { 10, 3 },  { { 0.0, -1.45 }, { 1.4, -201.45 } }),
        jt::make_road(5, { 11, 4 },  { { 0.0, 1.45 }, { 199.05, 20.97 } }),
        jt::make_road(6, { 11, 5 },  { { 0.0, 1.45 }, { -198.59, -22.23 } }),
        jt::make_road(7, { 11, 6 },  { { 0.0, 1.45 }, { -3.14, 201.43 } }),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(1, 3.0), jt::lane_profile(1, 3.0), jt::lane_profile(1, 3.0),
                 jt::lane_profile(1, 2.0), jt::lane_profile(1, 3.0), jt::lane_profile(1, 3.0),
                 jt::lane_profile(1, 2.0) });

    // The primary is whichever of the two has the lower GraphNodeId, and it is
    // the one that carries the whole junction.
    GraphNodeId primary = kInvalidId;
    size_t emitting = 0;
    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (fixture.graph.nodes()[n].degree() < 3) continue;
        const std::vector<ArmRef> arms =
            collect_arms(fixture.graph, fixture.profiles, static_cast<GraphNodeId>(n),
                         stratum::osm::road::kCoincidentRadius);
        if (arms.empty()) continue;
        primary = static_cast<GraphNodeId>(n);
        ++emitting;
    }

    // Merged, so exactly one of the two solves and it carries six arms: the two
    // eastbound, the two westbound, the one south and the one north. The 3 m link
    // is internal and is dropped.
    CHECK_EQ(emitting, size_t{1});
    CHECK_TRUE(primary != kInvalidId);
    if (primary == kInvalidId) return;

    const Solved s = solve("dual carriageway merged into one junction", fixture, primary);
    CHECK_EQ(s.arms.size(), size_t{6});

    // The whole invariant: a simple, counter-clockwise ring containing its node.
    check_usable(s);
}

/**
 * @brief A fillet corner may not land off the end of its own chord
 *
 * FilletConfig::max_corner_reach_factor asks how far the corner point C stands
 * BACK ALONG each arm, which is the right question for two arms that diverge and
 * blind to the one that matters when they barely do: which way along the gap
 * between them C lies.
 *
 * The shape is Lucan's node 957. Two junction nodes 3.9 m apart, and the pair
 * that breaks it is the two eastbound arms -- one 6 m wide leaving the southern
 * node at -2.1 degrees, one 2 m wide leaving the northern at -4.1. Two degrees of
 * convergence over a chord a metre and a half long put C seven and a half metres
 * WEST, behind a junction whose every arm here leaves east, and it still stood
 * less than a trim plus a width back along either arm, so the widths bound passed
 * it. The ring drawn through it cut straight across the junction and back.
 *
 * Measured in the chord's own length the same point projects at 4.9, where a real
 * corner projects between 0 and 1. That is what this pins, and it needs no notion
 * of how wide the roads are or how big the junction is.
 */
TEST(JunctionRobustness, a_corner_off_the_end_of_its_chord_is_refused) {
    const std::vector<Road> roads = {
        jt::make_road(1, { 10, 11 }, { { 0.0, -1.95 }, { 0.0, 1.95 } }),
        jt::make_road(2, { 10, 1 },  { { 0.0, -1.95 }, { -199.18, 16.18 } }),
        jt::make_road(3, { 10, 2 },  { { 0.0, -1.95 }, { 199.87, -9.28 } }),
        jt::make_road(4, { 11, 3 },  { { 0.0, 1.95 }, { 199.49, -12.35 } }),
        jt::make_road(5, { 11, 4 },  { { 0.0, 1.95 }, { 12.21, 201.58 } }),
        jt::make_road(6, { 11, 5 },  { { 0.0, 1.95 }, { -199.41, 17.29 } }),
    };
    // Widths are the extract's: the southern eastbound arm is 6 m across and the
    // northern one 2 m, which is what makes their edges converge so slowly.
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(1, 3.0), jt::lane_profile(1, 3.0), jt::lane_profile(1, 6.0),
                 jt::lane_profile(1, 2.0), jt::lane_profile(1, 3.0), jt::lane_profile(1, 2.0) });

    GraphNodeId primary = kInvalidId;
    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (fixture.graph.nodes()[n].degree() < 3) continue;
        if (collect_arms(fixture.graph, fixture.profiles, static_cast<GraphNodeId>(n),
                         stratum::osm::road::kCoincidentRadius).empty()) {
            continue;
        }
        primary = static_cast<GraphNodeId>(n);
    }
    CHECK_TRUE(primary != kInvalidId);
    if (primary == kInvalidId) return;

    const Solved s = solve("a corner behind a junction that leaves east", fixture, primary);
    CHECK_EQ(s.arms.size(), size_t{5});
    check_usable(s);
}

/**
 * @brief A CHAIN of junctions is not one junction
 *
 * The cluster flood is transitive; the reason for merging is not. Two junctions
 * are merged because their polygons would overlap, and that says nothing about a
 * third junction on the far side of one of them.
 *
 * Three narrow T-junctions 1.8 m apart. Each neighbouring pair is inside the
 * other's reach -- 1.0 m of carriageway half-width each puts their threshold at
 * 2.0 m -- so the flood walks the whole chain and offers a cluster 3.6 m across.
 * The two ENDS are 3.6 m apart and their threshold is the same 2.0 m, so they are
 * two junctions by the only rule that ever justified merging any of them.
 *
 * Left merged, the middle member's arms leave from INSIDE the compound junction,
 * and build_junction_polygon() cannot put a mouth that starts inside the junction
 * onto the boundary ring: it spikes inward to reach it and crosses itself. On a
 * Lucan extract this was what remained after the parallel-mouth repair -- clusters
 * of five, six and seven members, the widest 15.4 m across, contributing 11 of the
 * 15 crossing rings left on merged junctions.
 *
 * The cluster is refused WHOLE. Pruning it would have to choose a member to drop,
 * and every rule for choosing depends on GraphNodeId or on the order the flood
 * ran -- which is what every_member_of_a_cluster_agrees_on_the_primary forbids.
 */
TEST(JunctionRobustness, a_chain_of_junctions_is_not_one_junction) {
    const double step = 1.8;
    const std::vector<Road> roads = {
        jt::make_road(1, { 1, 10, 11, 12, 2 },
                      { { -200.0, 0.0 }, { 0.0, 0.0 }, { step, 0.0 },
                        { 2.0 * step, 0.0 }, { 200.0, 0.0 } }),
        jt::make_road(2, { 3, 10 }, { { 0.0, -200.0 }, { 0.0, 0.0 } }),
        jt::make_road(3, { 4, 11 }, { { step, 200.0 }, { step, 0.0 } }),
        jt::make_road(4, { 5, 12 }, { { 2.0 * step, -200.0 }, { 2.0 * step, 0.0 } }),
    };
    // 2 m wide, so each junction's radius is 1 m and any pair's threshold is 2 m.
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(1, 2.0), jt::lane_profile(1, 2.0),
                 jt::lane_profile(1, 2.0), jt::lane_profile(1, 2.0) });

    // All three solve, each on its own, each with its own three arms. Nothing is
    // absorbed, so nothing reports an empty arm list.
    size_t solved_nodes = 0;
    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (fixture.graph.nodes()[n].degree() < 3) continue;

        std::vector<GraphNodeId> cluster;
        const std::vector<ArmRef> arms =
            collect_arms(fixture.graph, fixture.profiles, static_cast<GraphNodeId>(n),
                         stratum::osm::road::kCoincidentRadius, &cluster);

        CHECK_EQ(cluster.size(), size_t{1});
        CHECK_EQ(arms.size(), size_t{3});
        ++solved_nodes;

        const Solved s = solve("one of a refused chain", fixture, static_cast<GraphNodeId>(n));
        check_usable(s);
    }
    CHECK_EQ(solved_nodes, size_t{3});
}

/**
 * @brief The compound intersection the width rule exists for still merges
 *
 * The other half of a_chain_of_junctions_is_not_one_junction. An all-pairs rule
 * that also threw away a dual carriageway crossing another would have undone the
 * thing it was added to protect.
 *
 * One dual carriageway crossing another is four junction nodes in a rectangle, a
 * shape OSM has no other way to express. Here the rectangle is 8 m on a side and
 * the carriageways are 14 m wide, so every node's radius is 7 m and every pair's
 * threshold is 14 m -- the sides at 8 m and the DIAGONALS at 11.3 m are both
 * inside it. Wide roads are allowed to stand further apart than narrow ones, which
 * is the same scaling the stub threshold uses.
 */
TEST(JunctionRobustness, a_dual_carriageway_crossing_still_merges_as_one) {
    const double h = 4.0;
    const std::vector<Road> roads = {
        // The two north-south carriageways.
        jt::make_road(1, { 1, 10, 13, 2 },
                      { { -h, -200.0 }, { -h, -h }, { -h, h }, { -h, 200.0 } }),
        jt::make_road(2, { 3, 11, 12, 4 },
                      { { h, -200.0 }, { h, -h }, { h, h }, { h, 200.0 } }),
        // And the two east-west ones, through the same four nodes.
        jt::make_road(3, { 5, 10, 11, 6 },
                      { { -200.0, -h }, { -h, -h }, { h, -h }, { 200.0, -h } }),
        jt::make_road(4, { 7, 13, 12, 8 },
                      { { -200.0, h }, { -h, h }, { h, h }, { 200.0, h } }),
    };
    const jt::Fixture fixture = jt::make_fixture(
        roads, { jt::lane_profile(2, 7.0), jt::lane_profile(2, 7.0),
                 jt::lane_profile(2, 7.0), jt::lane_profile(2, 7.0) });

    // Four junction nodes; merging is a solving decision, not a graph edit.
    size_t degree_four = 0;
    for (const auto& n : fixture.graph.nodes()) {
        if (n.degree() >= 3) ++degree_four;
    }
    CHECK_EQ(degree_four, size_t{4});

    size_t emitting = 0;
    size_t arms_on_primary = 0;
    for (size_t n = 0; n < fixture.graph.nodes().size(); ++n) {
        if (fixture.graph.nodes()[n].degree() < 3) continue;

        std::vector<GraphNodeId> cluster;
        const std::vector<ArmRef> arms =
            collect_arms(fixture.graph, fixture.profiles, static_cast<GraphNodeId>(n),
                         stratum::osm::road::kCoincidentRadius, &cluster);

        // Every member sees the same cluster of four, whichever it is asked from.
        CHECK_EQ(cluster.size(), size_t{4});
        if (!arms.empty()) {
            ++emitting;
            arms_on_primary = arms.size();
        }
    }

    // One junction, carrying the eight real approaches: two per side. The four
    // rectangle edges are internal and are dropped.
    CHECK_EQ(emitting, size_t{1});
    CHECK_EQ(arms_on_primary, size_t{8});
}

/**
 * @brief A ring that excludes the point its own arms leave from is refused
 *
 * A junction polygon is the ground where the arms MEET, so a ring that does not
 * contain the point they leave from is not that ground. The fill sits off to one
 * side of the roads it is supposed to join, and -- the reason this matters more
 * than it looks -- the terrain carve uses the ring as a winding test, so the
 * ground under the meeting point is flattened by nothing.
 *
 * `self_intersecting` cannot see it, because the ring really is simple, and
 * `inverted` cannot either, because it is wound correctly. Without its own flag
 * the polygon was handed on as a usable outline.
 *
 * ### Where it comes from
 *
 * The trims. TrimConfig::max_trim_fraction cuts a demand back on a short edge, so
 * the mouths come to rest closer in than the junction's own shape asks for, and
 * with most of the arms clamped the ring can pass on the wrong side of the node.
 * A Lucan extract had nine, one of them with two arms trimmed to 0.25 m and
 * 0.00 m -- both mouths standing on the node itself. A different one had no
 * clamped arm at all: a 14 m carriageway trimmed 3.8 m meeting two 3.5 m roads
 * trimmed 9.7 and 9.2 m pulls the ring up and away, and the node finishes 0.29 m
 * outside its own fill. No trim repairs either; the edges really are shorter than
 * the junction is wide.
 *
 * The arms are built here rather than solved, for the reason
 * a_corner_point_a_kilometre_away_is_not_a_corner gives: the flag is a pure
 * function of the arms and their cut cross-sections, and stating the shape
 * directly says what is being tested. This is an ordinary symmetric three-way
 * whose cut faces have been moved bodily 8 m north of the point its arms leave
 * from -- simple, counter-clockwise, and nowhere near its own node.
 */
TEST(JunctionRobustness, a_ring_that_excludes_its_own_node_is_refused) {
    const glm::dvec2 away(0.0, 8.0);   // how far the faces sit from the origin
    const double reach = 5.0;          // trim
    const double half = 2.0;           // carriageway half-width

    Solved s;
    s.label = "a fill beside its own junction";
    s.node = glm::dvec2(0.0);

    for (const double degrees : { -120.0, 0.0, 120.0 }) {
        const double bearing = degrees * 3.14159265358979 / 180.0;
        const glm::dvec2 dir(std::cos(bearing), std::sin(bearing));
        const glm::dvec2 left_normal(-dir.y, dir.x);
        const glm::dvec2 centre = away + dir * reach;

        ArmRef arm;
        arm.edge = static_cast<EdgeId>(s.arms.size());
        arm.at_start = true;
        arm.bearing = bearing;
        arm.origin = s.node;            // every arm leaves the origin
        arm.carriageway_half = half;
        arm.half_width = half;
        arm.trim = reach;
        s.arms.push_back(arm);

        ArmEnd end;
        end.direction = dir;
        end.carriage_left = centre + left_normal * half;
        end.carriage_right = centre - left_normal * half;
        end.left = end.carriage_left;
        end.right = end.carriage_right;
        end.center = centre;
        end.arclength = reach;
        end.valid = true;
        s.ends.push_back(end);
    }

    s.poly = build_junction_polygon(s.arms, s.ends, FilletConfig{});
    s.solved = true;
    CHECK_TRUE(s.poly.valid);
    if (!s.poly.valid) return;

    // Simple and correctly wound, which is exactly why neither of the other two
    // flags reports it.
    CHECK_TRUE(jt::ring_is_simple(s.poly.ring));
    CHECK_FALSE(s.poly.self_intersecting);
    CHECK_FALSE(s.poly.inverted);
    CHECK_TRUE(jt::signed_area(s.poly.ring) > 0.0);

    // The node really is outside, and the polygon says so.
    CHECK_FALSE(jt::point_in_ring(s.poly.ring, s.node));
    CHECK_TRUE(s.poly.excludes_origin);
    CHECK_TRUE(s.poly.needs_hull_fallback());
}
