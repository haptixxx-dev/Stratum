/**
 * @file test_junction_polygon.cpp
 * @brief Junction footprint and triangulation tests for the P4 junction solver
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Written against the contract in src/osm/road/junction_polygon.hpp and the P4
 * section of docs/plans/road_network_plan.md.
 *
 * ### The shape a junction footprint actually has
 *
 * With the arms trimmed, the footprint is a PLUS shape, not a disc and not a
 * convex polygon. Each arm contributes a cut face, and between two adjacent arms
 * the boundary runs BACK along one arm's near-side offset line, turns at the
 * point where the two offset lines meet, and runs OUT along the next arm's. That
 * turning point is a REFLEX vertex of the counter-clockwise ring: the notch
 * between two arms points inward.
 *
 * Everything about the fillet follows from that. Rounding a reflex vertex pushes
 * the boundary OUTWARD and ADDS area -- it is the wedge of carriageway a vehicle
 * turns through, not a diagonal cut across a corner -- so the header's ordering
 * `sharp corner < fillet < straight chamfer` holds and is asserted directly. A
 * test written for a convex corner, where a fillet removes area, would have every
 * one of these inequalities backwards.
 *
 * ### The analytic case
 *
 * A symmetric crossroads of half width h, cut at a reach `e` from the node, has
 * an exactly computable footprint. Writing `r = e - h` for the straight run each
 * corner has to play with:
 *
 * - sharp corners: a plus of a `2h` square plus four `2h x r` arms, `4h^2 + 8hr`
 * - straight chamfers: that plus the four corner triangles, `4e^2 - 2r^2`
 * - fillets: the sharp plus, plus `r^2(1 - pi/4)` of wedge per corner, plus the
 *   circular segments the tessellation adds by drawing each arc as chords that
 *   lie outside it
 *
 * What `e` is depends on the corner. The trim solve reserves the fillet's own
 * tangent run, `R tan(theta/2)`, so at the shipping defaults a right-angle corner
 * is cut at `h + R + clearance` and is rounded at the full nominal radius. Force
 * every corner to a chord -- min_arc_angle above pi -- and the reserve goes with
 * it, so the cut falls back to `h + clearance` and the footprint is the exact
 * octagon. Both are asserted as equalities; neither is a band.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests JunctionPolygon
 * @endcode
 */

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/road/junction_polygon.hpp"
#include "osm/road/junction_trim.hpp"
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
using stratum::osm::road::FilletConfig;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::JunctionPolygon;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::TrimConfig;
using stratum::osm::road::apply_fillet_reserve;
using stratum::osm::road::build_junction_polygon;
using stratum::osm::road::kInvalidId;
using stratum::osm::road::triangulate_junction;

namespace jt = stratum::test::junction;

/// A polygon with everything needed to check it, kept together for the sweeps
struct Case {
    std::string label;
    jt::Fixture fixture;
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    JunctionPolygon poly;
};

/**
 * @brief Solve a node and build its footprint in one step
 *
 * @param label   Name for failure messages
 * @param fixture Built fixture
 * @param degree  Degree of the node to solve; must be unique in the fixture
 * @param fillet  Corner rounding tolerances
 * @param trim    Trim tolerances
 * @return The case; its polygon is invalid when the node could not be found
 */
Case make_case(const std::string& label, jt::Fixture fixture, size_t degree,
               const FilletConfig& fillet = {}, const TrimConfig& trim = {}) {
    Case out;
    out.label = label;
    out.fixture = std::move(fixture);

    const GraphNodeId node = jt::sole_node_of_degree(out.fixture.graph, degree);
    if (node == kInvalidId) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture has one node of the degree",
                                      label + ": degree " + std::to_string(degree));
        return out;
    }
    jt::solve_node(out.fixture, node, trim, out.arms, out.ends);
    out.poly = build_junction_polygon(out.arms, out.ends, fillet);
    return out;
}

/// Every synthetic junction shape the sweeps run over
std::vector<Case> all_cases() {
    std::vector<Case> cases;
    cases.push_back(make_case("symmetric_cross", jt::symmetric_cross(2), 4));
    cases.push_back(make_case("asymmetric_cross", jt::asymmetric_cross(7.0, 3.0), 4));
    cases.push_back(make_case("t_junction", jt::fork(180.0), 3));
    cases.push_back(make_case("acute_fork", jt::fork(15.0), 3));
    cases.push_back(make_case("obtuse_fork", jt::fork(120.0), 3));
    cases.push_back(make_case("uneven_four_way", jt::star({0.0, 80.0, 180.0, 260.0}), 4));
    cases.push_back(make_case("five_way", jt::star({0.0, 65.0, 140.0, 205.0, 290.0}), 5));
    return cases;
}

/// h and clearance of the symmetric-cross fixture, for its analytic areas
constexpr double kHalf = jt::kLaneWidth;

/// Pi, for the analytic fillet areas
constexpr double kPi = 3.14159265358979323846;

} // namespace

// ============================================================================
// The analytic case
// ============================================================================

/**
 * The symmetric crossroads at the shipping defaults: counter-clockwise, simple,
 * one span per arm, and an area inside the band the two analytic extremes bound.
 */
TEST(JunctionPolygon, symmetric_cross_ring_is_ccw_simple_and_analytic) {
    TrimConfig trim;
    const Case c = make_case("symmetric_cross", jt::symmetric_cross(2), 4);
    CHECK_TRUE(c.poly.valid);
    if (!c.poly.valid) return;

    CHECK_EQ(c.poly.arm_ring_start.size(), size_t{4});
    CHECK_TRUE(c.poly.ring.size() >= 8);

    // Counter-clockwise, and not merely non-empty.
    CHECK_TRUE(jt::signed_area(c.poly.ring) > 0.0);
    CHECK_TRUE(jt::ring_is_simple(c.poly.ring));
    CHECK_FALSE(c.poly.self_intersecting);

    // A symmetric fixture must produce a centroid on the node.
    CHECK_NEAR(c.poly.centroid.x, 0.0, 1e-6);
    CHECK_NEAR(c.poly.centroid.y, 0.0, 1e-6);

    // The whole shape is written down. At the shipping defaults the trim reserves
    // the fillet's tangent run, so each arm is cut at `h + R + clearance` and each
    // corner really is rounded at the full nominal radius.
    const double clearance = trim.clearance;
    const FilletConfig fillet;
    const double radius = fillet.radius_width_factor * 2.0 * kHalf;
    const double reach = kHalf + radius + clearance;          // the cut, from the node
    const double run = reach - kHalf;                          // straight run per corner

    // A plus shape: the central square plus four arms of width 2h.
    const double sharp = 4.0 * kHalf * kHalf + 8.0 * kHalf * run;
    // Every corner cut off square instead: the octagon.
    const double chamfer = 4.0 * reach * reach - 2.0 * run * run;

    const double area = jt::signed_area(c.poly.ring);

    // sharp corner <= fillet <= straight chamfer, from junction_polygon.hpp.
    CHECK_TRUE(area >= sharp - 1e-9);
    CHECK_TRUE(area <= chamfer + 1e-9);

    // And the exact value between them. A fillet ADDS the wedge between its arc
    // and the two offset lines, `R^2 (1 - pi/4)` per corner, and the arc is drawn
    // as `segments_per_quarter_turn` chords which lie OUTSIDE it -- the region
    // excludes the arc's disc -- each adding a circular segment of
    // `(R^2 / 2)(alpha - sin alpha)`. Nothing here is approximate.
    const int segments = fillet.segments_per_quarter_turn;
    const double alpha = (kPi * 0.5) / static_cast<double>(segments);
    const double wedge = radius * radius * (1.0 - kPi * 0.25);
    const double chords = static_cast<double>(segments) * 0.5 * radius * radius *
                          (alpha - std::sin(alpha));
    CHECK_NEAR(area, sharp + 4.0 * (wedge + chords), 1e-9);

    // The point of the whole reserve: the corners are ARCS, not chamfers. Two
    // vertices per arm plus `segments - 1` interior arc points and two tangent
    // points per corner.
    CHECK_EQ(c.poly.ring.size(), size_t{4 * (2 + 1 + static_cast<size_t>(segments))});
}

/**
 * With every corner forced to its chord the footprint is exactly the octagon a
 * `2(h + c)` square with its four corners cut gives, and the ring is exactly two
 * vertices per arm in bearing order. Nothing here is approximate.
 */
TEST(JunctionPolygon, chamfered_symmetric_cross_is_the_exact_octagon) {
    FilletConfig fillet;
    fillet.min_arc_angle = 4.0;  // above pi: every corner falls back to its chord

    // Keeping the two configs in step, exactly as JunctionBuilder does. A corner
    // that will be drawn as a chord needs no tangent run, so the trim reserves
    // nothing and the cut lands at `h + clearance` -- which is what makes the
    // octagon below exact.
    TrimConfig trim;
    apply_fillet_reserve(fillet, trim);

    const Case c = make_case("chamfered_cross", jt::symmetric_cross(2), 4, fillet, trim);
    CHECK_TRUE(c.poly.valid);
    if (!c.poly.valid) return;

    CHECK_EQ(c.poly.ring.size(), size_t{8});
    CHECK_EQ(c.poly.arm_ring_start.size(), size_t{4});
    if (c.poly.ring.size() != 8 || c.poly.arm_ring_start.size() != 4) return;

    for (size_t k = 0; k < 4; ++k) {
        CHECK_EQ(c.poly.arm_ring_start[k], size_t{2 * k});
    }

    const double clearance = trim.clearance;
    const double expected = 4.0 * (kHalf + clearance) * (kHalf + clearance)
                            - 2.0 * clearance * clearance;
    CHECK_NEAR(jt::signed_area(c.poly.ring), expected, 1e-9);
    CHECK_TRUE(jt::ring_is_simple(c.poly.ring));

    // Every ring vertex is one of the eight analytic corners of the octagon.
    const double a = kHalf;                  // 3.5
    const double b = kHalf + clearance;      // 3.75
    const glm::dvec2 corners[8] = {
        {b, -a}, {b, a}, {a, b}, {-a, b}, {-b, a}, {-b, -a}, {-a, -b}, {a, -b},
    };
    for (const glm::dvec2& p : c.poly.ring) {
        double best = 1e300;
        for (const glm::dvec2& q : corners) {
            best = std::min(best, glm::length(p - q));
        }
        CHECK_NEAR(best, 0.0, 1e-9);
    }
}

// ============================================================================
// The weld seam
// ============================================================================

/**
 * The guard against a seam at every approach. `arm_ring_start[k]` must index the
 * vertex that IS arm k's `carriage_right`, with `carriage_left` immediately after
 * it, because the trimmed ribbon welds to exactly those two vertices.
 *
 * Recovering the indices by searching the ring for the nearest point is what this
 * published index replaces, and it is checked over every shape rather than over
 * one, because it is on the acute fixtures that a search would find the wrong
 * vertex.
 */
TEST(JunctionPolygon, arm_ring_start_indexes_the_arm_end_corners) {
    for (const Case& c : all_cases()) {
        if (!c.poly.valid) {
            stratum::test::report_failure(__FILE__, __LINE__, "polygon is valid", c.label);
            continue;
        }
        CHECK_EQ(c.poly.arm_ring_start.size(), c.arms.size());
        if (c.poly.arm_ring_start.size() != c.arms.size()) continue;

        for (size_t k = 0; k < c.arms.size(); ++k) {
            const size_t start = c.poly.arm_ring_start[k];
            if (start + 1 >= c.poly.ring.size()) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "arm span fits inside the ring",
                    c.label + " arm " + std::to_string(k) + ": start " +
                        std::to_string(start) + " of " + std::to_string(c.poly.ring.size()));
                continue;
            }
            // Exactly, not nearly: these vertices are copied, not recomputed.
            CHECK_NEAR(glm::length(c.poly.ring[start] - c.ends[k].carriage_right), 0.0, 1e-12);
            CHECK_NEAR(glm::length(c.poly.ring[start + 1] - c.ends[k].carriage_left), 0.0, 1e-12);
        }

        // The spans are in the arms' own order, so the walk is counter-clockwise.
        for (size_t k = 1; k < c.poly.arm_ring_start.size(); ++k) {
            CHECK_TRUE(c.poly.arm_ring_start[k] > c.poly.arm_ring_start[k - 1]);
        }
    }
}

// ============================================================================
// Fillets
// ============================================================================

/**
 * The fillet radius has to have an effect, and the effect has to have the right
 * sign. A junction with 80 / 100 / 80 / 100 degree gaps leaves a 1.48 m notch at
 * each of the wide corners -- deep enough that the nominal radius binds instead
 * of the notch -- so raising radius_width_factor really does change the shape
 * there.
 *
 * Rounding a REFLEX corner adds area, so a larger radius produces a LARGER ring.
 * This is the assertion a test written for a convex corner would get backwards.
 */
TEST(JunctionPolygon, larger_fillet_radius_grows_the_ring) {
    FilletConfig small;
    small.min_radius = 0.01;  // so a 0.7 m radius is not floored away
    small.radius_width_factor = 0.1;

    FilletConfig large = small;
    large.radius_width_factor = 0.2;

    FilletConfig chamfer;
    chamfer.min_arc_angle = 4.0;

    const std::vector<double> bearings = {0.0, 80.0, 180.0, 260.0};
    const Case c_small = make_case("uneven_small_radius", jt::star(bearings), 4, small);
    const Case c_large = make_case("uneven_large_radius", jt::star(bearings), 4, large);
    const Case c_chamfer = make_case("uneven_chamfer", jt::star(bearings), 4, chamfer);

    CHECK_TRUE(c_small.poly.valid);
    CHECK_TRUE(c_large.poly.valid);
    CHECK_TRUE(c_chamfer.poly.valid);
    if (!c_small.poly.valid || !c_large.poly.valid || !c_chamfer.poly.valid) return;

    const double area_small = jt::signed_area(c_small.poly.ring);
    const double area_large = jt::signed_area(c_large.poly.ring);
    const double area_chamfer = jt::signed_area(c_chamfer.poly.ring);

    // Rounding a reflex corner pushes the boundary outward: more radius, more area.
    // The two 100 degree corners alone move by about 0.2 m^2 each between these
    // radii, so the margin is far above any tessellation noise.
    CHECK_TRUE(area_large > area_small + 0.2);

    // And the chamfer, which fills the notch completely, bounds them both:
    // sharp corner < fillet < straight chamfer.
    CHECK_TRUE(area_chamfer > area_large + 1e-6);

    // The radius buys vertices as well as area, and never costs them.
    CHECK_TRUE(c_large.poly.ring.size() >= c_small.poly.ring.size());
    CHECK_TRUE(c_small.poly.ring.size() > 2 * c_small.arms.size());
    CHECK_EQ(c_chamfer.poly.ring.size(), size_t{2 * 4});

    // Every variant is still a usable ring.
    for (const Case* c : {&c_small, &c_large, &c_chamfer}) {
        CHECK_TRUE(jt::signed_area(c->poly.ring) > 0.0);
        CHECK_TRUE(jt::ring_is_simple(c->poly.ring));
        CHECK_FALSE(c->poly.self_intersecting);
    }
}

/**
 * A corner whose fitted radius falls below FilletConfig::min_radius degenerates to
 * a straight chamfer: no interior vertices at all, exactly two ring vertices per
 * arm, and a ring that is still valid, simple and counter-clockwise.
 *
 * min_radius is raised rather than radius_width_factor lowered, because the
 * nominal radius is CLAMPED INTO [min_radius, max_radius] before the notch
 * reduces it -- lowering the factor alone cannot produce a radius under the floor.
 * What can is a notch too shallow to accept the floor, which is what a 3 m floor
 * makes of this fixture's 0.21 m and 1.77 m corner allowances.
 *
 * Degenerating without breaking is the whole requirement. A ring that lost a
 * corner, or that emitted a zero-length arc, would take the weld seam with it.
 */
TEST(JunctionPolygon, tiny_radius_degenerates_to_a_chamfer_without_breaking) {
    FilletConfig tiny;
    tiny.radius_width_factor = 0.001;
    tiny.min_radius = 3.0;

    // Deliberately NOT in step with the trim: the reserve is switched off, so the
    // corner allowance is the bare clearance and the 3 m floor cannot fit in it.
    // That is the case this test is about -- a caller who configured the two
    // independently -- and the requirement is that it degenerates, not that it
    // throws away a corner.
    const std::vector<double> bearings = {0.0, 80.0, 180.0, 260.0};
    const Case c = make_case("tiny_radius", jt::star(bearings), 4, tiny,
                             jt::pairwise_only_trim());
    CHECK_TRUE(c.poly.valid);
    if (!c.poly.valid) return;

    // Two vertices per arm and nothing else: every corner fell back to its chord.
    CHECK_EQ(c.poly.ring.size(), size_t{8});
    CHECK_EQ(c.poly.arm_ring_start.size(), size_t{4});
    CHECK_TRUE(jt::signed_area(c.poly.ring) > 0.0);
    CHECK_TRUE(jt::ring_is_simple(c.poly.ring));
    CHECK_FALSE(c.poly.self_intersecting);

    // The arm spans survived the degeneration, so the ribbons still have
    // something to weld to.
    for (size_t k = 0; k < c.arms.size(); ++k) {
        const size_t start = c.poly.arm_ring_start[k];
        CHECK_TRUE(start + 1 < c.poly.ring.size());
        if (start + 1 >= c.poly.ring.size()) continue;
        CHECK_NEAR(glm::length(c.poly.ring[start] - c.ends[k].carriage_right), 0.0, 1e-12);
        CHECK_NEAR(glm::length(c.poly.ring[start + 1] - c.ends[k].carriage_left), 0.0, 1e-12);
    }
}

/**
 * Two arms 15 degrees apart. The corner between them is a sliver whose fillet
 * cannot fit, and the wrap-around corner opposite spans well over 180 degrees.
 *
 * The requirement is honesty, not success: whatever ring comes out, the ring must
 * not have inverted, and `self_intersecting` must agree with an independent
 * pairwise crossing test. A polygon that quietly reported false while crossing
 * itself would punch a hole in the terrain at the carve, which is the failure this
 * flag exists to prevent.
 */
TEST(JunctionPolygon, acute_fork_reports_self_intersection_honestly) {
    const Case c = make_case("acute_fork", jt::fork(15.0), 3);
    CHECK_TRUE(c.poly.valid);
    if (!c.poly.valid) return;

    CHECK_TRUE(c.poly.ring.size() >= 6);
    CHECK_EQ(c.poly.arm_ring_start.size(), size_t{3});

    // Not inverted: an acute fork still encloses its arms counter-clockwise.
    CHECK_TRUE(jt::signed_area(c.poly.ring) > 0.0);

    // Honest: the flag matches an independent test of the same property.
    const bool independently_simple = jt::ring_is_simple(c.poly.ring);
    CHECK_EQ(c.poly.self_intersecting, !independently_simple);

    // An empty ring is never flagged, per the contract.
    CHECK_FALSE(c.poly.ring.empty() && c.poly.self_intersecting);
}

// ============================================================================
// Triangulation
// ============================================================================

/**
 * The backfacing guard, run over EVERY shape rather than over one.
 *
 * The world mapping negates the second axis, `(x, y) -> vec3(x, height, -y)`, so
 * a counter-clockwise 2D triangle comes out with a +Y normal and needs no flip.
 * "Compensating" for the negation is the mistake this catches, and it produces
 * geometry that is invisible from above and perfectly normal from below -- which
 * is exactly the sort of defect a screenshot taken from the default camera
 * misses.
 */
TEST(JunctionPolygon, every_triangle_faces_up) {
    size_t total_triangles = 0;

    for (const Case& c : all_cases()) {
        if (!c.poly.valid) continue;
        const Mesh mesh = triangulate_junction(c.poly, 12.5f, MaterialId::Asphalt);
        const std::vector<jt::Tri2D> tris = jt::triangles_of(mesh);

        if (tris.empty()) {
            stratum::test::report_failure(__FILE__, __LINE__, "junction fill has triangles",
                                          c.label);
            continue;
        }
        total_triangles += tris.size();

        for (size_t t = 0; t < tris.size(); ++t) {
            if (!(tris[t].world_normal.y > 0.0)) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "triangle faces up",
                    c.label + " triangle " + std::to_string(t) + ": normal.y " +
                        stratum::test::stringify(tris[t].world_normal.y));
            }
        }

        // Everything sits on one plane at the height it was asked for, with an
        // exactly +Y vertex normal and the material it was given.
        for (const auto& v : mesh.vertices) {
            CHECK_NEAR(v.position.y, 12.5, 1e-4);
            CHECK_NEAR(v.normal.x, 0.0, 1e-6);
            CHECK_NEAR(v.normal.y, 1.0, 1e-6);
            CHECK_NEAR(v.normal.z, 0.0, 1e-6);
        }
        for (const auto& sub : mesh.effective_submeshes()) {
            CHECK_EQ(static_cast<int>(sub.material), static_cast<int>(MaterialId::Asphalt));
        }
    }

    CHECK_TRUE(total_triangles > 0);
}

/**
 * Earcut covered the ring with no gaps and no overlaps.
 *
 * A gap and an overlap are both invisible in a triangle count and both obvious in
 * the summed area: a missing ear leaves the fill short, and a double-covered ear
 * leaves it long. Comparing against the ring's own shoelace area catches either.
 *
 * Only rings the polygon builder did not flag are compared, because a flagged
 * ring is deliberately filled as its convex HULL, whose area is legitimately
 * larger.
 */
TEST(JunctionPolygon, triangulated_area_equals_polygon_area) {
    for (const Case& c : all_cases()) {
        if (!c.poly.valid || c.poly.self_intersecting) continue;

        const Mesh mesh = triangulate_junction(c.poly, 0.05f, MaterialId::Asphalt);
        const std::vector<jt::Tri2D> tris = jt::triangles_of(mesh);
        const double mesh_area = jt::plan_area(tris);
        const double ring_area = jt::signed_area(c.poly.ring);

        if (ring_area <= 0.0) {
            stratum::test::report_failure(__FILE__, __LINE__, "ring area is positive", c.label);
            continue;
        }
        // Vertices are float, so the tolerance scales with the footprint rather
        // than being a fixed epsilon.
        const double tolerance = std::max(1e-3, ring_area * 1e-5);
        if (std::fabs(mesh_area - ring_area) > tolerance) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "triangulated area == ring area",
                c.label + ": mesh " + stratum::test::stringify(mesh_area) + " ring " +
                    stratum::test::stringify(ring_area));
        }

        // A convex hull would also match on a convex ring, so the plus-shaped
        // fixtures are the ones that make this assertion bite: their hull is
        // strictly larger than their ring.
        CHECK_TRUE(mesh.indices.size() % 3u == 0u);
        CHECK_TRUE(mesh.vertices.size() >= 3);
    }
}

/**
 * A polygon the builder refused to validate produces an empty mesh and nothing
 * else. No crash, no partial fill, no triangles referring to vertices that are
 * not there.
 */
TEST(JunctionPolygon, invalid_polygon_yields_an_empty_mesh) {
    // Default-constructed: valid false, empty ring.
    const Mesh empty = triangulate_junction(JunctionPolygon{}, 3.0f, MaterialId::Asphalt);
    CHECK_TRUE(empty.vertices.empty());
    CHECK_TRUE(empty.indices.empty());

    // A ring too short to be a polygon, explicitly flagged invalid.
    JunctionPolygon degenerate;
    degenerate.ring = {{0.0, 0.0}, {1.0, 0.0}};
    degenerate.valid = false;
    const Mesh two_points = triangulate_junction(degenerate, 3.0f, MaterialId::Asphalt);
    CHECK_TRUE(two_points.vertices.empty());
    CHECK_TRUE(two_points.indices.empty());

    // And the builder itself refuses a node with too few arms rather than
    // inventing a footprint for it.
    const JunctionPolygon too_few = build_junction_polygon({}, {}, FilletConfig{});
    CHECK_FALSE(too_few.valid);
    CHECK_FALSE(too_few.self_intersecting);
    CHECK_TRUE(too_few.ring.empty());

    // Mismatched arms and ends is a caller error and must not be papered over.
    const jt::Fixture fixture = jt::symmetric_cross(2);
    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 4);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    jt::solve_node(fixture, node, TrimConfig{}, arms, ends);
    ends.pop_back();
    const JunctionPolygon mismatched = build_junction_polygon(arms, ends, FilletConfig{});
    CHECK_FALSE(mismatched.valid);
}

/**
 * The footprint over real parsed topology. tests/data/t_junction.osm has one
 * degree-3 node on an interior shared vertex, and its footprint has to come out
 * counter-clockwise, simple and filled the same way a synthetic one does.
 */
TEST(JunctionPolygon, t_junction_fixture_produces_a_usable_footprint) {
    const auto parsed = jt::parse_fixture("t_junction.osm");
    if (!parsed) return;

    RoadGraph graph;
    graph.build(*parsed);
    const auto centerlines = jt::make_centerlines(graph);
    const auto profiles = jt::profiles_from_tags(graph, *parsed);

    const GraphNodeId node = jt::node_with_osm_id(graph, static_cast<NodeId>(203));
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    std::vector<ArmRef> arms = stratum::osm::road::collect_arms(graph, profiles, node);
    CHECK_TRUE(stratum::osm::road::solve_arm_trims(graph, centerlines, node, arms,
                                                   TrimConfig{}));
    std::vector<ArmEnd> ends;
    for (const ArmRef& arm : arms) {
        ends.push_back(stratum::osm::road::arm_end(graph, centerlines, profiles, arm));
    }

    const JunctionPolygon poly = build_junction_polygon(arms, ends, FilletConfig{});
    CHECK_TRUE(poly.valid);
    if (!poly.valid) return;

    CHECK_EQ(poly.arm_ring_start.size(), size_t{3});
    CHECK_TRUE(jt::signed_area(poly.ring) > 0.0);
    CHECK_TRUE(jt::ring_is_simple(poly.ring));
    CHECK_FALSE(poly.self_intersecting);

    // The centroid lies inside its own ring; a plus-shaped footprint whose arms
    // outran its middle would not.
    CHECK_TRUE(jt::point_in_ring(poly.ring, poly.centroid));

    const Mesh mesh = triangulate_junction(poly, 0.05f, MaterialId::Asphalt);
    const std::vector<jt::Tri2D> tris = jt::triangles_of(mesh);
    CHECK_TRUE(!tris.empty());
    for (const jt::Tri2D& tri : tris) {
        CHECK_TRUE(tri.world_normal.y > 0.0);
    }
    CHECK_NEAR(jt::plan_area(tris), jt::signed_area(poly.ring),
               std::max(1e-3, jt::signed_area(poly.ring) * 1e-5));
}
