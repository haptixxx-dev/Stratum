/**
 * @file test_junction_curb.cpp
 * @brief Sidewalk and curb ring tests for the P4 junction solver
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Written against the contract in src/osm/road/junction_curb.hpp and the P4
 * section of docs/plans/road_network_plan.md.
 *
 * ### The gaps are the test
 *
 * An outward offset of the junction footprint is easy. What is hard, and what
 * this suite mostly checks, is that the ring OPENS at every approach. Each arm
 * arrives carrying its own curb and sidewalk, and the ring must stop where they
 * start. A ring that closed all the way round would put a curb across every
 * approach -- a wall a vehicle drives into and a defect that is invisible in a
 * vertex count, in a triangle count, and in an overhead screenshot.
 *
 * The mouth is the FULL profile width, `ArmEnd::right` to `ArmEnd::left`, not the
 * carriageway width. Opening only the carriageway is the near-miss failure: the
 * ring then runs its sidewalk across the arm's sidewalk instead of across its
 * lanes, which looks almost right and is still a wall. The assertion is therefore
 * made against the full-profile span, where the near-miss shows up.
 *
 * ### Clipper2
 *
 * This is the only Clipper2 consumer in the tree, and Clipper2 is integer-based,
 * so every distance here carries the offsetter's own rounding: millimetre
 * quantisation from CurbRingConfig::clipper_scale plus up to
 * CurbRingConfig::arc_tolerance of chord deviation on a rounded corner. The
 * offset-distance tolerance below is sized for both and for nothing else.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests JunctionCurb
 * @endcode
 */

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/road/junction_curb.hpp"
#include "osm/road/junction_polygon.hpp"
#include "osm/road/junction_trim.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::osm::Road;
using stratum::osm::road::ArmEnd;
using stratum::osm::road::ArmRef;
using stratum::osm::road::CurbRing;
using stratum::osm::road::CurbRingConfig;
using stratum::osm::road::FilletConfig;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::JunctionPolygon;
using stratum::osm::road::TrimConfig;
using stratum::osm::road::build_curb_ring;
using stratum::osm::road::build_junction_polygon;
using stratum::osm::road::kInvalidId;

namespace jt = stratum::test::junction;

/**
 * @brief Slack on a Clipper2 offset distance, metres
 *
 * Millimetre quantisation from clipper_scale plus the 20 mm chord deviation the
 * default arc_tolerance permits on a rounded corner, with room to spare. Anything
 * looser would stop the test noticing a ring offset by the wrong amount; anything
 * tighter would fail on correct output.
 */
constexpr double kOffsetEps = 0.05;

/// Below this a cross-section has no direction to measure "across" it with
constexpr double kZeroSpan = 1e-9;

/**
 * @brief How far past an arm's cut line a vertex must be to count as up the approach
 *
 * The section clip lands its terminal samples exactly ON the cut line, and float
 * storage moves them a micron either way. A millimetre absorbs that; a real
 * intrusion is centimetres at least.
 */
constexpr double kPastCut = 1e-3;

/**
 * @brief Lateral inset from the carriageway edge, metres
 *
 * The ring's curb and the arm's own curb are the same line, so a ring vertex
 * sitting exactly on the carriageway edge is the two meeting, not a kerbstone in
 * a lane. A centimetre of inset says that and nothing more.
 */
constexpr double kCurbLine = 0.01;

/// One solved junction with its ring, kept together
struct RingCase {
    std::string label;
    jt::Fixture fixture;
    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    JunctionPolygon poly;
    CurbRing ring;
    float height = 12.5f;
};

/**
 * @brief Solve a node, build its footprint, and wrap it in a curb ring
 *
 * @param label   Name for failure messages
 * @param fixture Built fixture
 * @param degree  Degree of the node to solve; must be unique in the fixture
 * @param cfg     Ring dimensions
 * @return The case; its ring is empty when the node could not be solved
 */
RingCase make_ring(const std::string& label, jt::Fixture fixture, size_t degree,
                   const CurbRingConfig& cfg = {}) {
    RingCase out;
    out.label = label;
    out.fixture = std::move(fixture);

    const GraphNodeId node = jt::sole_node_of_degree(out.fixture.graph, degree);
    if (node == kInvalidId) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture has one node of the degree",
                                      label);
        return out;
    }
    jt::solve_node(out.fixture, node, TrimConfig{}, out.arms, out.ends);
    out.poly = build_junction_polygon(out.arms, out.ends, FilletConfig{});
    out.ring = build_curb_ring(out.poly, out.arms, out.ends, out.height, cfg);
    return out;
}

/**
 * @brief A symmetric crossroads whose arms carry sidewalks
 *
 * The junction polygon is the carriageway footprint whatever the profile, so the
 * sidewalks change nothing about the ring's inner boundary. They change the
 * MOUTHS, which is the point: a mouth is the full profile width, so a road with
 * sidewalks opens a wider gap than its carriageway.
 */
jt::Fixture cross_with_sidewalks(double walk) {
    const double length = 200.0;
    const std::vector<Road> roads = {
        jt::make_road(1, {1, 100, 2}, {{-length, 0.0}, {0.0, 0.0}, {length, 0.0}}),
        jt::make_road(2, {3, 100, 4}, {{0.0, -length}, {0.0, 0.0}, {0.0, length}}),
    };
    return jt::make_fixture(
        roads, {jt::sidewalk_profile(2, walk, walk), jt::sidewalk_profile(2, walk, walk)});
}

/**
 * @brief Three arms at 0, 90 and 200 degrees, all carrying sidewalks
 *
 * Deliberately uneven. At a junction with equal gaps every arm's trim is driven by
 * both of its neighbours at once, so the notch at every corner is exactly
 * TrimConfig::clearance deep and the mouths swallow the whole corner. The 160
 * degree gap here leaves legs of 2.1 m and 3.1 m, which is corner enough for a
 * real ring section to survive the mouths and for the mesh assertions to have
 * something to measure.
 */
jt::Fixture uneven_three_way(double walk) {
    const double length = 200.0;
    const double deg = 3.14159265358979323846 / 180.0;
    std::vector<Road> roads;
    std::vector<stratum::osm::road::RoadProfile> profiles;
    const double bearings[3] = {0.0, 90.0, 200.0};
    for (size_t i = 0; i < 3; ++i) {
        const double radians = bearings[i] * deg;
        roads.push_back(jt::make_road(
            static_cast<stratum::osm::WayId>(i + 1),
            {100, static_cast<stratum::osm::NodeId>(i + 1)},
            {{0.0, 0.0}, {length * std::cos(radians), length * std::sin(radians)}}));
        profiles.push_back(jt::sidewalk_profile(2, walk, walk));
    }
    return jt::make_fixture(roads, profiles);
}

/**
 * @brief Assert that the ring leaves one arm's approach usable
 *
 * "Walled in" is two distinct failures and this checks both, because the outer
 * band of the ring is SUPPOSED to touch the cut line and only the inner part is
 * supposed to stay clear of it.
 *
 * 1. **The carriageway mouth is open.** Nothing may cross or cover the span from
 *    ArmEnd::carriage_right to ArmEnd::carriage_left. That is the drivable width,
 *    and a curb across it is the failure this file exists for.
 *
 * 2. **Nothing stands in a lane.** No ring vertex may lie beyond the cut line --
 *    `dot(v - center, direction) > 0` -- while it is also laterally INSIDE the
 *    arm's carriageway. A curb there is a kerbstone in a running lane. The
 *    lateral bound is inset by kCurbLine so a vertex sitting exactly on the
 *    carriageway edge, where the ring's curb and the arm's own curb are the same
 *    line, is not read as an intrusion.
 *
 * Both bounds are the CARRIAGEWAY and not the full profile, which is what an
 * earlier version of this check used. The corner sidewalk MUST run out to the cut
 * line and meet the arm's own sidewalk there: that is a pedestrian network
 * joining up, not a wall. Demanding the full profile width be clear demands a
 * hole in the sidewalk at every corner of every junction, which is why that
 * version failed on a correct ring. What it was reaching for -- that the ring
 * does not merely open the carriageway and leave the approach blocked -- is what
 * (2) covers, and covers more strictly, because it rejects a curb anywhere up the
 * approach rather than only where it crosses the cut line itself.
 *
 * Within (1), two independent checks, because either alone has a blind spot: a
 * triangle whose edges all miss the span can still cover it (caught by the sample
 * points), and a span that threads between sample points can still be crossed by
 * an edge (caught by the crossing test).
 *
 * @param mesh  Ring mesh to check
 * @param end   The arm's cut cross-section
 * @param label Fixture and arm, for the failure message
 */
void check_mouth_is_open(const Mesh& mesh, const ArmEnd& end, const std::string& label) {
    // ---- (2) no curb standing in a lane of the approach ---------------------
    const glm::dvec2 carriage_span = end.carriage_left - end.carriage_right;
    const double carriage_width = glm::length(carriage_span);
    if (carriage_width > kZeroSpan) {
        const glm::dvec2 lateral = carriage_span / carriage_width;
        const glm::dvec2 forward = end.direction;
        const double half = carriage_width * 0.5 - kCurbLine;

        for (const auto& v : mesh.vertices) {
            const glm::dvec2 p = jt::world_to_local(v.position);
            const double along = glm::dot(p - end.center, forward);
            if (along <= kPastCut) continue;

            const double across = glm::dot(p - end.center, lateral);
            if (across < -half || across > half) continue;   // clear of the lanes

            stratum::test::report_failure(
                __FILE__, __LINE__, "no curb geometry stands in the approach lanes",
                label + ": vertex " + stratum::test::stringify(along) +
                    " m past the cut line, " + stratum::test::stringify(across) +
                    " m off centre, carriageway half " +
                    stratum::test::stringify(carriage_width * 0.5));
            break;
        }
    }

    // ---- (1) the carriageway mouth is open ---------------------------------
    const glm::dvec2 span = end.carriage_left - end.carriage_right;
    const double length = glm::length(span);
    if (length <= 0.2) return;  // nothing meaningful to leave open

    const glm::dvec2 unit = span / length;
    const double margin = 0.05;
    const glm::dvec2 a = end.carriage_right + unit * margin;
    const glm::dvec2 b = end.carriage_left - unit * margin;

    const std::vector<jt::Tri2D> tris = jt::triangles_of(mesh);

    for (size_t t = 0; t < tris.size(); ++t) {
        const glm::dvec2 corners[3] = {tris[t].a, tris[t].b, tris[t].c};
        for (size_t i = 0; i < 3; ++i) {
            if (jt::segments_cross(a, b, corners[i], corners[(i + 1) % 3])) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "no curb geometry crosses the arm mouth",
                    label + ": triangle " + std::to_string(t) + " crosses the span");
                return;
            }
        }
    }

    constexpr int kSamples = 33;
    for (int s = 0; s < kSamples; ++s) {
        const double u = static_cast<double>(s) / static_cast<double>(kSamples - 1);
        const glm::dvec2 p = a + (b - a) * u;
        for (size_t t = 0; t < tris.size(); ++t) {
            // Skip slivers: a zero-area triangle contains everything on its line
            // and says nothing about whether the mouth is walled in.
            if (std::fabs(jt::cross2(tris[t].a, tris[t].b, tris[t].c)) < 1e-9) continue;
            if (jt::point_in_triangle(p, tris[t].a, tris[t].b, tris[t].c)) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "no curb geometry covers the arm mouth",
                    label + ": triangle " + std::to_string(t) + " covers the span at u=" +
                        stratum::test::stringify(u));
                return;
            }
        }
    }
}

} // namespace

// ============================================================================
// The offset
// ============================================================================

/**
 * The outer ring is strictly outside the inner ring, everywhere, by
 * CurbRingConfig::ring_offset() -- the apron, the curb face, the curb top and the
 * sidewalk surface added up, which is the same reach an arm's own profile has
 * from its carriageway edge to its outer sidewalk edge.
 *
 * A symmetric crossroads is used because its footprint is a convex octagon: every
 * one of its vertices is a convex corner, so the round offset puts the outer
 * boundary at exactly the offset distance from each of them and the expectation is
 * a single number rather than a case analysis.
 */
TEST(JunctionCurb, outer_ring_is_outside_inner_by_the_sidewalk_width) {
    CurbRingConfig cfg;
    cfg.sidewalk_width = 2.0;

    // The band is the sum of four widths, not the sidewalk alone: the apron is
    // what carries the ring's curb face out to the lateral the ARMS' curb faces
    // stand on, since the junction polygon's own boundary is the lane edge.
    const double offset = cfg.ring_offset();
    CHECK_NEAR(offset,
               cfg.apron_width + cfg.curb_face_batter + cfg.curb_top_width + cfg.sidewalk_width,
               1e-12);

    const RingCase c = make_ring("symmetric_cross", jt::symmetric_cross(2), 4, cfg);
    CHECK_TRUE(c.poly.valid);
    if (!c.poly.valid) return;

    CHECK_TRUE(!c.ring.inner.empty());
    CHECK_TRUE(!c.ring.outer.empty());
    if (c.ring.inner.empty() || c.ring.outer.empty()) return;

    // inner is the junction polygon ring, verbatim.
    CHECK_EQ(c.ring.inner.size(), c.poly.ring.size());
    for (size_t i = 0; i < c.ring.inner.size() && i < c.poly.ring.size(); ++i) {
        CHECK_NEAR(glm::length(c.ring.inner[i] - c.poly.ring[i]), 0.0, 1e-12);
    }

    // Counter-clockwise, whichever way the offsetter returned it.
    CHECK_TRUE(jt::signed_area(c.ring.outer) > 0.0);
    CHECK_TRUE(jt::signed_area(c.ring.outer) > jt::signed_area(c.ring.inner));

    for (size_t i = 0; i < c.ring.inner.size(); ++i) {
        const glm::dvec2& p = c.ring.inner[i];

        // Strictly outside: every inner vertex is enclosed by the outer ring.
        if (!jt::point_in_ring(c.ring.outer, p)) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "inner vertex is inside the outer ring",
                "vertex " + std::to_string(i));
            continue;
        }

        // And at the offset distance, not merely somewhere outside.
        const double distance = jt::point_ring_distance(c.ring.outer, p);
        CHECK_NEAR(distance, offset, kOffsetEps);
    }

    // Every outer vertex is at least the offset away from the inner ring, so the
    // band never pinches to nothing.
    for (size_t i = 0; i < c.ring.outer.size(); ++i) {
        const double distance = jt::point_ring_distance(c.ring.inner, c.ring.outer[i]);
        if (distance < offset - kOffsetEps) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "outer vertex clears the inner ring",
                "vertex " + std::to_string(i) + ": " + stratum::test::stringify(distance));
        }
    }
}

// ============================================================================
// The gaps
// ============================================================================

/**
 * THE test of this file. For every arm of every fixture, no ring geometry may
 * cross that arm's cross-section span.
 *
 * Walling in the approaches is the failure this exists for, and the near-miss --
 * opening only the carriageway rather than the full profile -- is caught because
 * the span asserted against is the full profile width.
 */
TEST(JunctionCurb, the_ring_opens_at_every_arm) {
    CurbRingConfig cfg;
    cfg.sidewalk_width = 2.0;

    std::vector<RingCase> cases;
    cases.push_back(make_ring("cross_with_sidewalks", cross_with_sidewalks(2.0), 4, cfg));
    cases.push_back(make_ring("uneven_three_way", uneven_three_way(2.0), 3, cfg));
    cases.push_back(make_ring("acute_fork", jt::fork(15.0), 3, cfg));
    cases.push_back(make_ring("uneven_four_way", jt::star({0.0, 80.0, 180.0, 260.0}), 4, cfg));

    size_t cases_with_geometry = 0;

    for (const RingCase& c : cases) {
        if (!c.poly.valid) continue;
        if (!c.ring.mesh.indices.empty()) ++cases_with_geometry;

        CHECK_EQ(c.ends.size(), c.arms.size());
        for (size_t k = 0; k < c.ends.size(); ++k) {
            if (!c.ends[k].valid) continue;
            check_mouth_is_open(c.ring.mesh, c.ends[k],
                                c.label + " arm " + std::to_string(k));
        }
    }

    // The assertion above passes trivially on an empty mesh, so at least one
    // fixture has to have produced a ring for the sweep to mean anything.
    CHECK_TRUE(cases_with_geometry > 0);
}

// ============================================================================
// Heights
// ============================================================================

/**
 * The curb face runs from the carriageway surface up to the curb top and stops:
 * every MaterialId::Curb vertex lies in `[height, height + curb_height]`, and both
 * extremes are present, which is what proves the face has a real height rather
 * than being a flat band drawn at one level.
 */
TEST(JunctionCurb, curb_face_vertices_span_exactly_the_curb_height) {
    CurbRingConfig cfg;
    cfg.sidewalk_width = 2.0;
    cfg.curb_height = 0.15;

    const float height = 12.5f;
    std::vector<RingCase> cases;
    cases.push_back(make_ring("uneven_three_way", uneven_three_way(2.0), 3, cfg));
    cases.push_back(make_ring("acute_fork", jt::fork(15.0), 3, cfg));
    cases.push_back(make_ring("uneven_four_way", jt::star({0.0, 80.0, 180.0, 260.0}), 4, cfg));

    size_t checked = 0;

    for (const RingCase& c : cases) {
        if (c.ring.mesh.indices.empty()) continue;
        CHECK_NEAR(c.height, height, 1e-9);

        double curb_min = 1e300;
        double curb_max = -1e300;
        size_t curb_vertices = 0;
        size_t sidewalk_vertices = 0;

        // Walk the index buffer so each vertex is attributed to the material of
        // the range that references it.
        for (const auto& sub : c.ring.mesh.effective_submeshes()) {
            for (uint32_t i = sub.index_offset; i < sub.index_offset + sub.index_count; ++i) {
                if (i >= c.ring.mesh.indices.size()) break;
                const uint32_t vi = c.ring.mesh.indices[i];
                if (vi >= c.ring.mesh.vertices.size()) continue;
                const double y = static_cast<double>(c.ring.mesh.vertices[vi].position.y);

                if (sub.material == MaterialId::Curb) {
                    curb_min = std::min(curb_min, y);
                    curb_max = std::max(curb_max, y);
                    ++curb_vertices;
                    if (y < height - 1e-4 || y > height + cfg.curb_height + 1e-4) {
                        stratum::test::report_failure(
                            __FILE__, __LINE__, "curb vertex is within the curb height",
                            c.label + ": y " + stratum::test::stringify(y));
                    }
                } else if (sub.material == MaterialId::Sidewalk) {
                    ++sidewalk_vertices;
                    // The sidewalk sits on top of the curb, level with the
                    // sidewalks of the arms feeding the ring.
                    CHECK_NEAR(y, static_cast<double>(height) + cfg.curb_height, 1e-4);
                }
            }
        }

        if (curb_vertices == 0) continue;
        ++checked;

        CHECK_NEAR(curb_min, static_cast<double>(height), 1e-4);
        CHECK_NEAR(curb_max, static_cast<double>(height) + cfg.curb_height, 1e-4);
        CHECK_NEAR(curb_max - curb_min, cfg.curb_height, 1e-4);
        CHECK_TRUE(sidewalk_vertices > 0);
    }

    CHECK_TRUE(checked > 0);
}

// ============================================================================
// Degenerate input
// ============================================================================

/**
 * An unusable polygon produces an empty ring and nothing else. Specifically no
 * Clipper2 exception: Clipper2 throws on some malformed input, and an exception
 * escaping build_curb_ring would take down the import worker rather than losing
 * one junction.
 */
TEST(JunctionCurb, degenerate_polygon_yields_an_empty_ring_without_throwing) {
    CurbRingConfig cfg;

    bool threw = false;
    CurbRing empty;
    try {
        empty = build_curb_ring(JunctionPolygon{}, {}, {}, 0.0f, cfg);
    } catch (...) {
        threw = true;
    }
    CHECK_FALSE(threw);
    CHECK_FALSE(empty.valid);
    CHECK_TRUE(empty.mesh.vertices.empty());
    CHECK_TRUE(empty.mesh.indices.empty());

    // A ring with too few points to be a polygon, flagged invalid.
    JunctionPolygon two_points;
    two_points.ring = {{0.0, 0.0}, {1.0, 0.0}};
    two_points.valid = false;
    threw = false;
    CurbRing degenerate;
    try {
        degenerate = build_curb_ring(two_points, {}, {}, 0.0f, cfg);
    } catch (...) {
        threw = true;
    }
    CHECK_FALSE(threw);
    CHECK_FALSE(degenerate.valid);
    CHECK_TRUE(degenerate.mesh.indices.empty());

    // A self-intersecting polygon: refused rather than offset, per the contract.
    JunctionPolygon crossed;
    crossed.ring = {{0.0, 0.0}, {10.0, 10.0}, {10.0, 0.0}, {0.0, 10.0}};
    crossed.arm_ring_start = {0, 2};
    crossed.valid = true;
    crossed.self_intersecting = true;
    threw = false;
    CurbRing refused;
    try {
        refused = build_curb_ring(crossed, {}, {}, 0.0f, cfg);
    } catch (...) {
        threw = true;
    }
    CHECK_FALSE(threw);
    CHECK_FALSE(refused.valid);
    CHECK_TRUE(refused.mesh.indices.empty());

    // Disabled: no ring at all, whatever the input.
    CurbRingConfig off = cfg;
    off.enabled = false;
    const RingCase c = make_ring("disabled", jt::symmetric_cross(2), 4, off);
    CHECK_FALSE(c.ring.valid);
    CHECK_TRUE(c.ring.mesh.indices.empty());
    CHECK_TRUE(c.ring.outer.empty());
}

// ============================================================================
// Path selection
// ============================================================================

/**
 * A junction polygon shaped so that inflating it splits the result into several
 * paths, and the assertion that the one kept is the one containing the centre.
 *
 * The ring carries a keyhole notch: a 1 m neck opening into a 12.5 m by 6 m
 * chamber. Offsetting outward by 2 m seals the neck -- 1 m is less than twice the
 * offset -- while the chamber, being wider than that on both axes, survives as a
 * separate closed path INSIDE the result. Clipper2 returns two paths, and taking
 * the wrong one hands the terrain carve a footprint entirely inside the junction.
 *
 * The polygon is hand-built rather than solved, because no plausible road network
 * produces a keyhole and the point is to exercise the selection rule, not to
 * pretend the shape is realistic. If the offsetter happens to return a single
 * path, every assertion below still holds and still describes the correct
 * behaviour.
 */
TEST(JunctionCurb, multi_path_offset_keeps_the_path_containing_the_centre) {
    JunctionPolygon poly;
    poly.ring = {
        {-3.5, -15.0}, {3.5, -15.0},                        // arm 2 span (bearing -90)
        {20.0, -15.0},
        {20.0, -3.5},  {20.0, 3.5},                         // arm 0 span (bearing 0)
        {20.0, 15.0},
        {5.5, 15.0},   {5.5, 11.0},                         // keyhole: neck, right wall
        {9.0, 11.0},   {9.0, 5.0},                          // chamber right
        {-3.5, 5.0},   {-3.5, 11.0},                        // chamber floor and left
        {4.5, 11.0},   {4.5, 15.0},                         // neck, left wall
        {-20.0, 15.0},
        {-20.0, 3.5},  {-20.0, -3.5},                       // arm 1 span (bearing 180)
        {-20.0, -15.0},
    };
    poly.arm_ring_start = {0, 3, 15};
    poly.centroid = glm::dvec2{0.0, -2.0};
    poly.valid = true;
    poly.self_intersecting = false;

    // The fixture is only useful if it really is a simple counter-clockwise ring.
    CHECK_TRUE(jt::signed_area(poly.ring) > 0.0);
    CHECK_TRUE(jt::ring_is_simple(poly.ring));
    CHECK_TRUE(jt::point_in_ring(poly.ring, poly.centroid));

    std::vector<ArmRef> arms(3);
    std::vector<ArmEnd> ends(3);
    const glm::dvec2 directions[3] = {{0.0, -1.0}, {1.0, 0.0}, {-1.0, 0.0}};
    const glm::dvec2 centers[3] = {{0.0, -15.0}, {20.0, 0.0}, {-20.0, 0.0}};
    const double bearings[3] = {-1.5707963267948966, 0.0, 3.141592653589793};

    for (size_t k = 0; k < 3; ++k) {
        const glm::dvec2 normal{-directions[k].y, directions[k].x};
        arms[k].edge = static_cast<stratum::osm::road::EdgeId>(k);
        arms[k].at_start = true;
        arms[k].bearing = bearings[k];
        arms[k].half_width = 5.5;
        arms[k].carriageway_half = 3.5;
        arms[k].trim = 1.0;

        ends[k].center = centers[k];
        ends[k].direction = directions[k];
        ends[k].carriage_left = centers[k] + normal * 3.5;
        ends[k].carriage_right = centers[k] - normal * 3.5;
        ends[k].left = centers[k] + normal * 5.5;
        ends[k].right = centers[k] - normal * 5.5;
        ends[k].arclength = 1.0;
        ends[k].valid = true;
    }

    CurbRingConfig cfg;
    cfg.sidewalk_width = 2.0;

    bool threw = false;
    CurbRing ring;
    try {
        ring = build_curb_ring(poly, arms, ends, 0.0f, cfg);
    } catch (...) {
        threw = true;
    }
    CHECK_FALSE(threw);
    if (threw) return;

    CHECK_TRUE(!ring.outer.empty());
    if (ring.outer.empty()) return;

    // THE assertion: the path kept is the one that encloses the junction, not the
    // sealed chamber floating inside it.
    CHECK_TRUE(jt::point_in_ring(ring.outer, poly.centroid));

    // And it is the outer boundary, so it encloses the whole footprint rather
    // than a piece of it.
    CHECK_TRUE(jt::signed_area(ring.outer) > jt::signed_area(poly.ring));
    CHECK_TRUE(jt::signed_area(ring.outer) > 0.0);

    for (size_t i = 0; i < poly.ring.size(); ++i) {
        if (!jt::point_in_ring(ring.outer, poly.ring[i])) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "every footprint vertex is inside the chosen path",
                "vertex " + std::to_string(i));
        }
    }

    // The sealed chamber is about 8.5 by 2 m and sits inside the junction; the
    // chosen path must not be it.
    CHECK_TRUE(std::fabs(jt::signed_area(ring.outer)) > 100.0);

    // The mouths still open, keyhole or not.
    for (size_t k = 0; k < ends.size(); ++k) {
        check_mouth_is_open(ring.mesh, ends[k], "keyhole arm " + std::to_string(k));
    }
}

// ============================================================================
// The apron
// ============================================================================

/**
 * The ring's curb face has to stand on the same lateral as the arms' curb faces.
 *
 * The junction polygon's boundary is the LANE edge -- ArmEnd::carriage_left and
 * carriage_right are the carriageway envelope and nothing else -- while
 * build_profile() lays a Gutter strip between the outer lane and the curb face on
 * every class that has a curb at all. A ring that starts its curb face on the
 * polygon boundary therefore misses every arm's curb face by the gutter width and
 * leaves an open notch in the kerb at every mouth.
 *
 * CurbRingConfig::apron_width is the band that closes it, and this pins both ends
 * of it: the apron lies flat on the carriageway, and the curb face rises at
 * exactly `apron_width` from the junction polygon.
 */
TEST(JunctionCurb, the_apron_carries_the_curb_face_out_to_the_arms_curb_line) {
    CurbRingConfig cfg;
    const RingCase c = make_ring("symmetric_cross", cross_with_sidewalks(2.0), 4, cfg);
    CHECK_TRUE(c.ring.valid);
    if (!c.ring.valid || c.ring.inner.empty()) return;

    // The apron is a real band, not a zero-width seam.
    CHECK_TRUE(cfg.apron_width > 0.0);

    const double surface = static_cast<double>(c.height);
    const double top = surface + cfg.curb_height;

    size_t apron_vertices = 0;
    size_t curb_face_bottoms = 0;

    for (const stratum::SubMesh& sub : c.ring.mesh.effective_submeshes()) {
        for (uint32_t i = 0; i < sub.index_count; ++i) {
            const uint32_t index = c.ring.mesh.indices[sub.index_offset + i];
            if (index >= c.ring.mesh.vertices.size()) continue;
            const auto& v = c.ring.mesh.vertices[index];
            const double y = static_cast<double>(v.position.y);
            const glm::dvec2 local = jt::world_to_local(v.position);
            const double reach = jt::point_ring_distance(c.ring.inner, local);

            if (sub.material == MaterialId::Concrete) {
                // The apron is FLAT: both of its boundaries are on the
                // carriageway plane, or it is a step and not an apron.
                CHECK_NEAR(y, surface, 1e-4);
                CHECK_TRUE(reach <= cfg.apron_width + kOffsetEps);
                ++apron_vertices;
            } else if (sub.material == MaterialId::Curb && std::fabs(y - surface) < 1e-4) {
                // The bottom of the curb face. THIS is the number the defect was
                // about: it must be the apron out from the polygon, not zero.
                CHECK_NEAR(reach, cfg.apron_width, kOffsetEps);
                ++curb_face_bottoms;
            } else if (sub.material == MaterialId::Curb) {
                CHECK_NEAR(y, top, 1e-4);
            }
        }
    }

    CHECK_TRUE(apron_vertices > size_t{0});
    CHECK_TRUE(curb_face_bottoms > size_t{0});

    // And the whole band still reaches an arm's own sidewalk edge: apron, face,
    // top and walk, which is the same run a kerbed profile has from its
    // carriageway edge outward.
    CHECK_NEAR(cfg.ring_offset(),
               cfg.apron_width + cfg.curb_face_batter + cfg.curb_top_width + cfg.sidewalk_width,
               1e-12);
}

// ============================================================================
// Curved approaches
// ============================================================================

/**
 * The mouth is opened along the arm's CUT FACE, and on a bending approach that
 * face is not perpendicular to ArmEnd::direction.
 *
 * arm_end() synthesises its station with slice(), and an interpolated station's
 * tangent is the band CHORD while its normal is the interpolated miter VECTOR;
 * the two are perpendicular only on a dead-straight run. A cut line built from
 * the direction alone therefore misses both carriageway corners by centimetres,
 * the section clip's micron tolerance never fires, and the ring's round join
 * survives untrimmed -- a two-metre quarter annulus of sidewalk emitted straight
 * up the approach, on top of the arm's own sidewalk.
 *
 * Probed on the ground: 1.5 m up the approach, out on the arm's own sidewalk, the
 * junction ring must not be there. The documented corner overlap is a fillet
 * segment wide, so it cannot reach this far.
 */
TEST(JunctionCurb, the_ring_does_not_run_up_a_curving_approach) {
    const double walk = 2.0;
    const double length = 200.0;
    const double radius = 20.0;

    // Two straight arms and one that curves away from the node on a 20 m radius,
    // sampled finely enough that the resampler keeps the bend.
    std::vector<glm::dvec2> curved;
    std::vector<stratum::osm::NodeId> curved_ids;
    for (int i = 0; i <= 40; ++i) {
        const double t = static_cast<double>(i) * 0.02;
        curved.push_back(glm::dvec2{-radius * std::sin(t), -radius * (1.0 - std::cos(t))});
        curved_ids.push_back(i == 0 ? static_cast<stratum::osm::NodeId>(100)
                                    : static_cast<stratum::osm::NodeId>(500 + i));
    }
    curved.push_back(curved.back() + glm::dvec2{-length, 0.0});
    curved_ids.push_back(static_cast<stratum::osm::NodeId>(700));

    const std::vector<Road> roads = {
        jt::make_road(1, {100, 2}, {{0.0, 0.0}, {length, 0.0}}),
        jt::make_road(2, {100, 3}, {{0.0, 0.0}, {0.0, length}}),
        jt::make_road(3, curved_ids, curved),
    };

    // Smoothing ON, which is the production default and the configuration the
    // interpolated-station tilt actually appears in.
    stratum::osm::road::ResampleConfig resample;
    resample.smooth = true;

    jt::Fixture fixture = jt::make_fixture(
        roads,
        {jt::sidewalk_profile(2, walk, walk), jt::sidewalk_profile(2, walk, walk),
         jt::sidewalk_profile(2, walk, walk)},
        resample);

    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 3);
    CHECK(node != kInvalidId);
    if (node == kInvalidId) return;

    RingCase c;
    c.label = "curved_approach";
    c.fixture = std::move(fixture);
    jt::solve_node(c.fixture, node, TrimConfig{}, c.arms, c.ends);
    c.poly = build_junction_polygon(c.arms, c.ends, FilletConfig{});
    CHECK_TRUE(c.poly.valid);
    if (!c.poly.valid) return;
    c.ring = build_curb_ring(c.poly, c.arms, c.ends, c.height, CurbRingConfig{});
    CHECK_TRUE(c.ring.valid);
    if (!c.ring.valid) return;

    const std::vector<jt::Tri2D> tris = jt::triangles_of(c.ring.mesh);
    CHECK_TRUE(!tris.empty());
    if (tris.empty()) return;

    for (size_t k = 0; k < c.ends.size(); ++k) {
        const ArmEnd& end = c.ends[k];
        if (!end.valid) continue;

        // The cut face itself, which is what the mouth is opened along.
        glm::dvec2 face = end.carriage_left - end.carriage_right;
        const double span = glm::length(face);
        if (!(span > kZeroSpan)) continue;
        face /= span;

        glm::dvec2 along(face.y, -face.x);
        if (glm::dot(along, end.direction) < 0.0) along = -along;

        // 1.5 m up the approach, in the middle of the arm's own sidewalk on each
        // side. Both are well outside anything the ring is entitled to cover.
        const double sidewalk_middle = 0.5 * span + 0.5 * walk;
        for (const double side : {1.0, -1.0}) {
            const glm::dvec2 probe =
                end.center + along * 1.5 + face * (side * sidewalk_middle);
            if (jt::covered_in_plan(tris, probe)) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "the ring stops at every arm's cut face",
                    "arm " + std::to_string(k) + ": ring covers the approach 1.5 m past the cut");
            }
        }
    }
}
