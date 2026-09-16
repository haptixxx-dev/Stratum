// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_straight_skeleton.cpp
 * @brief The straight skeleton of a simple polygon
 *
 * Two features read this one function -- the skeleton lot subdivision (C3) and
 * the roof operations (D5) -- so a disagreement between them is impossible only
 * for as long as there is one implementation and it is right. These tests are
 * what "right" means here.
 *
 * The suite is built around FOUR invariants rather than around remembered
 * coordinates, because a straight skeleton that has gone wrong still looks like a
 * plausible list of segments:
 *
 *   1. **The faces tile the polygon.** Their areas sum to the polygon's area and
 *      no face ring crosses itself. That is exactly the property a missing split
 *      event destroys, and it is checkable without knowing the right answer.
 *   2. **A ridge is a ridge.** A rectangle's skeleton has a segment in the middle,
 *      not a point. An implementation that only ever produces apexes passes every
 *      test written on a square and fails this one.
 *   3. **A split event happens where one is needed.** An L-shape has one reflex
 *      vertex and the wavefront must cut the far kerb in two there.
 *   4. **A refusal is a refusal.** `complete` false, `faces` empty. A caller that
 *      gets no answer falls back; a caller that gets a folded answer ships it.
 *
 * The named-coordinate assertions -- the centre of the square, the ends of the
 * rectangle's ridge -- are here as well, because an invariant suite alone cannot
 * tell a correct skeleton from a mirrored one.
 */

#include "framework.hpp"

#include "geometry/straight_skeleton.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using stratum::geometry::compute_straight_skeleton;
using stratum::geometry::signed_ring_area;
using stratum::geometry::SkeletonArc;
using stratum::geometry::SkeletonConfig;
using stratum::geometry::SkeletonFace;
using stratum::geometry::StraightSkeleton;
using stratum::geometry::vertex_offset_velocity;

namespace {

// ============================================================================
// Shapes
// ============================================================================

/// 10 x 10, anticlockwise, origin at a corner
std::vector<glm::dvec2> square() {
    return { { 0.0, 0.0 }, { 10.0, 0.0 }, { 10.0, 10.0 }, { 0.0, 10.0 } };
}

/// 30 x 10, anticlockwise. Its skeleton is a ridge, not an apex.
std::vector<glm::dvec2> rectangle() {
    return { { 0.0, 0.0 }, { 30.0, 0.0 }, { 30.0, 10.0 }, { 0.0, 10.0 } };
}

/**
 * @brief An L with one reflex vertex at (10, 10)
 *
 * Both arms are 10 wide, so both flatten at the same instant and the wavefront at
 * that instant is two ridge segments rather than a polygon. The shape therefore
 * exercises the split event AND the simultaneous-collapse path at once, which is
 * why it is the L used everywhere below.
 */
std::vector<glm::dvec2> ell() {
    return { { 0.0, 0.0 },  { 30.0, 0.0 },  { 30.0, 10.0 },
             { 10.0, 10.0 }, { 10.0, 30.0 }, { 0.0, 30.0 } };
}

/// 100 x 0.5. A real one: the median strip between two carriageways.
std::vector<glm::dvec2> sliver() {
    return { { 0.0, 0.0 }, { 100.0, 0.0 }, { 100.0, 0.5 }, { 0.0, 0.5 } };
}

/**
 * @brief A rectangle with @p teeth slots of equal width cut into its bottom edge
 *
 * EVERY SLOT THE SAME WIDTH IS THE POINT. Each tooth between two slots collapses
 * by an edge event, and because the teeth are identical those collapses are
 * simultaneous and leave two reflex vertices at exactly the same place. That is a
 * vertex event, which the simulation has no handler for -- see the scope section
 * of straight_skeleton.hpp -- and it is the only shape family known to produce
 * one. Vary the widths by a millimetre and it does not happen.
 *
 * Five slots is the smallest count that reproduces it; two, three and four do not.
 */
std::vector<glm::dvec2> comb(size_t teeth, double width, double depth, double height) {
    std::vector<glm::dvec2> ring;
    ring.push_back({ 0.0, 0.0 });
    for (size_t i = 0; i < teeth; ++i) {
        const double x0 = (2.0 * static_cast<double>(i) + 1.0) * width;
        const double x1 = (2.0 * static_cast<double>(i) + 2.0) * width;
        ring.push_back({ x0, 0.0 });
        ring.push_back({ x0, depth });
        ring.push_back({ x1, depth });
        ring.push_back({ x1, 0.0 });
    }
    const double right = (2.0 * static_cast<double>(teeth) + 1.0) * width;
    ring.push_back({ right, 0.0 });
    ring.push_back({ right, height });
    ring.push_back({ 0.0, height });
    return ring;
}

// ============================================================================
// A Deterministic Stream, for the generated families
// ============================================================================

/**
 * @brief SplitMix64, so a failure in a generated family is reproducible from source
 *
 * A generated family needs many shapes to find the ones that matter -- the guards
 * these tests exist for are reached by about one comb in twenty -- and a family
 * seeded from the clock is a test that fails on somebody else's machine and not on
 * yours. The same finaliser lots.cpp and lots_offset.cpp use, for no deeper reason
 * than that it is already the one in the tree.
 */
struct Rng {
    uint64_t state = 0;

    explicit Rng(uint64_t seed) : state(seed) {}

    uint64_t next() {
        state += 0x9E3779B97F4A7C15ull;
        uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    /// A value in [lo, hi)
    double range(double lo, double hi) {
        return lo + (hi - lo) * (static_cast<double>(next() >> 11) * 0x1.0p-53);
    }
};

/**
 * @brief A comb whose every slot is a different width and a different depth
 *
 * The comb above but with the exact symmetry taken out, so no vertex event
 * happens and every collapse is an ordinary edge or split event. What is left is a
 * shape with `teeth` reflex pairs whose wavefront splits repeatedly, which is what
 * makes it the hardest family in this file for the split path -- and in particular
 * for the test in find_event() that refuses a split against an edge the reflex
 * vertex has already gone past.
 */
std::vector<glm::dvec2> ragged_comb(Rng& rng, size_t teeth, double height) {
    std::vector<glm::dvec2> ring;
    ring.push_back({ 0.0, 0.0 });
    double x = 0.0;
    for (size_t i = 0; i < teeth; ++i) {
        x += rng.range(4.0, 12.0);
        const double x0 = x;
        x += rng.range(4.0, 12.0);
        ring.push_back({ x0, 0.0 });
        ring.push_back({ x0, rng.range(0.2, 0.85) * height });
        ring.push_back({ x, ring.back().y });
        ring.push_back({ x, 0.0 });
        // Each slot gets its own depth, and the two sides of one slot share it, so
        // the slot is a rectangle rather than a wedge.
    }
    x += rng.range(4.0, 12.0);
    ring.push_back({ x, 0.0 });
    ring.push_back({ x, height });
    ring.push_back({ 0.0, height });
    return ring;
}

/**
 * @brief A star-shaped polygon: @p points evenly spaced angles, random radii
 *
 * Simple by construction -- a ray from the origin meets the boundary once -- and
 * deeply non-convex whenever two neighbouring radii differ, so it is the cheapest
 * generator of genuine split events there is. std::cos and std::sin are used here
 * and nowhere in the library: the SHAPE may differ by an ulp between platforms,
 * which cannot change whether its faces tile.
 */
std::vector<glm::dvec2> star(Rng& rng, size_t points, double r_min, double r_max) {
    std::vector<glm::dvec2> ring;
    ring.reserve(points);
    for (size_t i = 0; i < points; ++i) {
        const double angle =
            6.283185307179586 * static_cast<double>(i) / static_cast<double>(points);
        const double r = rng.range(r_min, r_max);
        ring.push_back({ r * std::cos(angle), r * std::sin(angle) });
    }
    return ring;
}

// ============================================================================
// Invariant Helpers
// ============================================================================

double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

/// True when segments (a,b) and (c,d) cross at a point interior to both
bool segments_cross(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c,
                    const glm::dvec2& d) {
    const double d1 = cross2(b - a, c - a);
    const double d2 = cross2(b - a, d - a);
    const double d3 = cross2(d - c, a - c);
    const double d4 = cross2(d - c, b - c);
    return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
           ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
}

/// Pairs of non-adjacent segments of @p ring that properly cross
size_t ring_self_crossings(const std::vector<glm::dvec2>& ring) {
    const size_t n = ring.size();
    if (n < 4) return 0;
    size_t crossings = 0;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) continue;
            if (segments_cross(ring[i], ring[(i + 1) % n], ring[j], ring[(j + 1) % n])) {
                ++crossings;
            }
        }
    }
    return crossings;
}

/**
 * @brief Pairs of skeleton arcs that cross each other TRANSVERSALLY
 *
 * Two arcs are compared by POSITION rather than by node index, and two exclusions
 * are made. Neither is fussiness; both are there because the naive version of this
 * helper reported hundreds of crossings on skeletons whose faces tile perfectly.
 *
 *   - **Arcs that meet within the point epsilon share an end.** The simulation can
 *     emit two distinct node ids at the same coordinates -- two events that
 *     happened at the same place -- and by the file's own contract points that
 *     close together ARE one point. Compared by index, the two arcs that meet
 *     there read as a crossing at a separation of 1e-15 metres.
 *   - **Arcs that are parallel to within a microradian are the same ridge.** The
 *     same duplication, one step on: two nodes three microns apart at each end of a
 *     ridge give two arcs that lie along each other and formally cross somewhere in
 *     the middle. A REAL self-intersection is transversal, so excluding the
 *     parallel pairs costs the test nothing it was there to find.
 *
 * What is left is what a folded wavefront actually produces: two arcs at an angle,
 * crossing at a point that is not an endpoint of either.
 */
size_t arc_crossings(const StraightSkeleton& skeleton, double tol = 1e-6) {
    size_t crossings = 0;
    for (size_t i = 0; i < skeleton.arcs.size(); ++i) {
        for (size_t j = i + 1; j < skeleton.arcs.size(); ++j) {
            const glm::dvec2 a = skeleton.nodes[skeleton.arcs[i].from].position;
            const glm::dvec2 b = skeleton.nodes[skeleton.arcs[i].to].position;
            const glm::dvec2 c = skeleton.nodes[skeleton.arcs[j].from].position;
            const glm::dvec2 d = skeleton.nodes[skeleton.arcs[j].to].position;
            if (glm::length(b - a) < tol || glm::length(d - c) < tol) continue;
            if (std::fabs(cross2(glm::normalize(b - a), glm::normalize(d - c))) < 1e-6) continue;
            if (!segments_cross(a, b, c, d)) continue;

            // How far into both arcs the crossing is. At or below the tolerance the
            // two arcs meet at a shared end that the simulation happened to record
            // as two node ids, which is not a self-intersection.
            const double d3 = cross2(d - c, a - c);
            const double d4 = cross2(d - c, b - c);
            const glm::dvec2 x = a + (b - a) * (d3 / (d3 - d4));
            double depth = glm::length(x - a);
            for (const glm::dvec2& e : { b, c, d }) depth = std::min(depth, glm::length(x - e));
            if (depth > tol) ++crossings;
        }
    }
    return crossings;
}

/// Sum of the face areas. Equals the polygon's area for a correct skeleton.
double total_face_area(const StraightSkeleton& skeleton) {
    double total = 0.0;
    for (const SkeletonFace& face : skeleton.faces) total += face.area;
    return total;
}

/// The face belonging to contour edge @p edge, or nullptr when it was dropped
const SkeletonFace* face_of(const StraightSkeleton& skeleton, uint32_t edge) {
    for (const SkeletonFace& face : skeleton.faces) {
        if (face.edge == edge) return &face;
    }
    return nullptr;
}

/// Largest SkeletonNode::time over the nodes that are not on the contour
double max_interior_time(const StraightSkeleton& skeleton) {
    double best = 0.0;
    for (const auto& node : skeleton.nodes) {
        if (!node.on_contour) best = std::max(best, node.time);
    }
    return best;
}

/// Distance from @p p to the boundary of @p ring, positive inside or out
double distance_to_ring(const std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    double best = 1e300;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2 a = ring[i];
        const glm::dvec2 b = ring[(i + 1) % ring.size()];
        const glm::dvec2 ab = b - a;
        const double len2 = glm::dot(ab, ab);
        double t = 0.0;
        if (len2 > 0.0) t = std::clamp(glm::dot(p - a, ab) / len2, 0.0, 1.0);
        best = std::min(best, glm::length(p - (a + ab * t)));
    }
    return best;
}

/// Crossing-number point-in-polygon; boundary cases are not distinguished
bool point_in_ring(const std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    if (ring.size() < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[j];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

/**
 * @brief How far the furthest face vertex lies OUTSIDE @p ring, in the ring's units
 *
 * The blunt instrument, and the one that separates a skeleton that is a little off
 * from a skeleton that has folded. A face is the land nearest one edge, so every
 * vertex of it is inside the polygon; when a split event is missed the wavefront
 * marches out through the boundary and the face picks up vertices tens of metres
 * beyond it. An area sum can hide that -- two faces that overlap by as much as a
 * third is missing still add up -- and this cannot.
 */
double max_face_vertex_outside(const StraightSkeleton& skeleton,
                               const std::vector<glm::dvec2>& ring) {
    double worst = 0.0;
    for (const SkeletonFace& face : skeleton.faces) {
        for (const glm::dvec2& p : face.ring) {
            if (point_in_ring(ring, p)) continue;
            worst = std::max(worst, distance_to_ring(ring, p));
        }
    }
    return worst;
}

/**
 * @brief Assert the invariants that hold for EVERY complete skeleton
 *
 * Called from most tests below rather than repeated in them, so that a new shape
 * is one line and cannot accidentally be checked more weakly than the others.
 */
void check_tiles_polygon(const StraightSkeleton& skeleton) {
    CHECK_TRUE(skeleton.complete);
    if (!skeleton.complete) return;

    const double expected = std::fabs(signed_ring_area(skeleton.contour));
    CHECK_NEAR(total_face_area(skeleton), expected, 1e-6 * std::max(1.0, expected));
    CHECK_EQ(arc_crossings(skeleton), size_t{ 0 });

    for (const SkeletonFace& face : skeleton.faces) {
        CHECK((face.ring.size()) >= size_t{ 3 });
        CHECK((face.area) > 0.0);
        CHECK_EQ(face.times.size(), face.ring.size());
        CHECK_EQ(ring_self_crossings(face.ring), size_t{ 0 });
        CHECK_NEAR(signed_ring_area(face.ring), face.area, 1e-9 * std::max(1.0, face.area));

        // ring[0] and ring[1] are the ends of the face's own contour edge, in the
        // contour's direction. C3 anchors a lot to its street on this, so a face
        // that is right but rotated is still useless.
        const size_t n = skeleton.contour.size();
        CHECK_NEAR(glm::length(face.ring[0] - skeleton.contour[face.edge]), 0.0, 1e-9);
        CHECK_NEAR(glm::length(face.ring[1] - skeleton.contour[(face.edge + 1) % n]), 0.0, 1e-9);
        CHECK_NEAR(face.times[0], 0.0, 1e-12);
        CHECK_NEAR(face.times[1], 0.0, 1e-12);
    }
}

} // namespace

// ============================================================================
// The Square: an X to the centre
// ============================================================================

/**
 * @brief A square's skeleton is four arcs meeting at the centre
 *
 * The simplest shape there is, and the one every implementation gets right, so it
 * is here to pin the CONVENTIONS rather than the algorithm: four faces, one per
 * edge, each a triangle of a quarter of the area, and the apex at the centre with
 * a time equal to the inradius.
 */
TEST(StraightSkeleton, square_meets_at_the_centre) {
    const StraightSkeleton skeleton = compute_straight_skeleton(square());

    check_tiles_polygon(skeleton);
    CHECK_EQ(skeleton.faces.size(), size_t{ 4 });
    CHECK_EQ(skeleton.stats.split_events, size_t{ 0 });
    CHECK_EQ(skeleton.stats.reflex_vertices, size_t{ 0 });

    for (const SkeletonFace& face : skeleton.faces) {
        CHECK_EQ(face.ring.size(), size_t{ 3 });
        CHECK_NEAR(face.area, 25.0, 1e-9);
        // The third point of every face is the centre, and it is 5 from each edge.
        CHECK_NEAR(glm::length(face.ring[2] - glm::dvec2(5.0, 5.0)), 0.0, 1e-9);
        CHECK_NEAR(face.times[2], 5.0, 1e-9);
    }
    CHECK_NEAR(skeleton.stats.max_time, 5.0, 1e-9);
}

/**
 * @brief SkeletonNode::time is the distance to the nearest edge
 *
 * The property both consumers are actually built on: D5 lifts a roof by it and C3
 * reads it as the depth of a lot. Asserted against an independent
 * point-to-boundary distance rather than against the value the simulation stored,
 * so a time that is merely self-consistent does not pass.
 */
TEST(StraightSkeleton, node_time_is_the_distance_to_the_nearest_edge) {
    const std::vector<glm::dvec2> ring = ell();
    const StraightSkeleton skeleton = compute_straight_skeleton(ring);
    CHECK_TRUE(skeleton.complete);
    if (!skeleton.complete) return;

    size_t interior = 0;
    for (const auto& node : skeleton.nodes) {
        if (node.on_contour) {
            CHECK_NEAR(node.time, 0.0, 1e-12);
            continue;
        }
        ++interior;
        CHECK_NEAR(node.time, distance_to_ring(skeleton.contour, node.position), 1e-6);
    }
    CHECK((interior) > size_t{ 0 });
}

// ============================================================================
// The Rectangle: a ridge, not a point
// ============================================================================

/**
 * @brief A rectangle's skeleton is a RIDGE
 *
 * The test that catches an implementation which only ever produces apexes. A
 * 30 x 10 rectangle's skeleton is the segment from (5, 5) to (25, 5): the two
 * short ends get triangular faces, the two long sides get trapezia, and if the
 * whole thing collapsed to one point instead, three of those four numbers would
 * still look reasonable.
 */
TEST(StraightSkeleton, rectangle_produces_a_ridge_not_an_apex) {
    const StraightSkeleton skeleton = compute_straight_skeleton(rectangle());

    check_tiles_polygon(skeleton);
    CHECK_EQ(skeleton.faces.size(), size_t{ 4 });

    // Two distinct interior nodes, at the two ends of the ridge.
    std::vector<glm::dvec2> interior;
    for (const auto& node : skeleton.nodes) {
        if (node.on_contour) continue;
        bool seen = false;
        for (const glm::dvec2& p : interior) seen = seen || glm::length(p - node.position) < 1e-6;
        if (!seen) interior.push_back(node.position);
    }
    CHECK_EQ(interior.size(), size_t{ 2 });
    if (interior.size() == 2) {
        std::sort(interior.begin(), interior.end(),
                  [](const glm::dvec2& a, const glm::dvec2& b) { return a.x < b.x; });
        CHECK_NEAR(interior[0].x, 5.0, 1e-9);
        CHECK_NEAR(interior[0].y, 5.0, 1e-9);
        CHECK_NEAR(interior[1].x, 25.0, 1e-9);
        CHECK_NEAR(interior[1].y, 5.0, 1e-9);
    }

    // The long sides get four-sided faces. A collapse to a single apex would make
    // these triangles, which is the failure this whole test exists for.
    const SkeletonFace* bottom = face_of(skeleton, 0);
    const SkeletonFace* top = face_of(skeleton, 2);
    CHECK_TRUE(bottom != nullptr);
    CHECK_TRUE(top != nullptr);
    if (bottom != nullptr) {
        CHECK_EQ(bottom->ring.size(), size_t{ 4 });
        CHECK_NEAR(bottom->area, 125.0, 1e-9);
    }
    if (top != nullptr) {
        CHECK_EQ(top->ring.size(), size_t{ 4 });
        CHECK_NEAR(top->area, 125.0, 1e-9);
    }

    // And the short ends stay triangles.
    const SkeletonFace* right = face_of(skeleton, 1);
    CHECK_TRUE(right != nullptr);
    if (right != nullptr) {
        CHECK_EQ(right->ring.size(), size_t{ 3 });
        CHECK_NEAR(right->area, 25.0, 1e-9);
    }
}

// ============================================================================
// The L: a split event is mandatory
// ============================================================================

/**
 * @brief An L-shape's reflex vertex must split the far edge
 *
 * SPLIT EVENTS ARE THE PART NAIVE IMPLEMENTATIONS SKIP. The reflex vertex at
 * (10, 10) reaches the long bottom edge and cuts it in two; without that the
 * wavefront folds through itself and the faces overlap, which shows up here as
 * areas summing to more than 500 and as a face ring that crosses itself.
 *
 * Both the count and the consequence are asserted. The count alone would pass an
 * implementation that fires a split event in the wrong PLACE.
 */
TEST(StraightSkeleton, ell_shape_needs_and_gets_a_split_event) {
    const StraightSkeleton skeleton = compute_straight_skeleton(ell());

    check_tiles_polygon(skeleton);
    CHECK_EQ(skeleton.stats.reflex_vertices, size_t{ 1 });
    CHECK((skeleton.stats.split_events) >= size_t{ 1 });
    CHECK_EQ(skeleton.faces.size(), size_t{ 6 });
    CHECK_NEAR(total_face_area(skeleton), 500.0, 1e-9);

    // Both arms are 10 wide, so the wavefront everywhere stops at 5.
    CHECK_NEAR(skeleton.stats.max_time, 5.0, 1e-9);

    // The two long outer edges reach right around the corner, so their faces have
    // four vertices; the inner corner's two edges get the split's two halves.
    const SkeletonFace* bottom = face_of(skeleton, 0);
    CHECK_TRUE(bottom != nullptr);
    if (bottom != nullptr) CHECK_NEAR(bottom->area, 125.0, 1e-9);
}

/**
 * @brief A block with a deep notch splits without folding
 *
 * A second split-event shape, and a harsher one: the notch reaches most of the way
 * across, so the reflex vertex hits an edge that is nearly opposite it and the two
 * wavefront loops the split leaves behind are very unequal. The assertion is the
 * one that matters -- the result does not self-intersect.
 */
TEST(StraightSkeleton, deep_notch_splits_without_self_intersecting) {
    const std::vector<glm::dvec2> ring = {
        { 0.0, 0.0 },  { 40.0, 0.0 },  { 40.0, 30.0 }, { 22.0, 30.0 },
        { 20.0, 4.0 }, { 18.0, 30.0 }, { 0.0, 30.0 },
    };
    const StraightSkeleton skeleton = compute_straight_skeleton(ring);

    check_tiles_polygon(skeleton);
    CHECK((skeleton.stats.reflex_vertices) >= size_t{ 1 });
    CHECK((skeleton.stats.split_events) >= size_t{ 1 });
    CHECK_NEAR(total_face_area(skeleton), std::fabs(signed_ring_area(ring)), 1e-6);
}

/**
 * @brief A plus sign splits four times and still tiles
 *
 * Four reflex vertices, and every arm the same width, so all four splits and every
 * edge event happen at the same instant. The tie-break between simultaneous split
 * and edge events is what decides whether this comes out tiled or folded.
 */
TEST(StraightSkeleton, plus_sign_tiles_under_four_simultaneous_splits) {
    const std::vector<glm::dvec2> ring = {
        { 10.0, 0.0 },  { 20.0, 0.0 },  { 20.0, 10.0 }, { 30.0, 10.0 },
        { 30.0, 20.0 }, { 20.0, 20.0 }, { 20.0, 30.0 }, { 10.0, 30.0 },
        { 10.0, 20.0 }, { 0.0, 20.0 },  { 0.0, 10.0 },  { 10.0, 10.0 },
    };
    const StraightSkeleton skeleton = compute_straight_skeleton(ring);

    check_tiles_polygon(skeleton);
    CHECK_EQ(skeleton.stats.reflex_vertices, size_t{ 4 });
    CHECK((skeleton.stats.split_events) >= size_t{ 1 });
    CHECK_EQ(skeleton.faces.size(), size_t{ 12 });
    CHECK_NEAR(total_face_area(skeleton), 500.0, 1e-9);
}

/**
 * @brief A comb of equal teeth comes back with no face ring folded through itself
 *
 * THE REGRESSION TEST FOR THE ONE OUTCOME THIS FILE SAYS IT MUST NEVER PRODUCE.
 *
 * Every tooth of this comb is the same width, so every tooth collapses at the same
 * instant and leaves two reflex vertices at EXACTLY the same point. That is a
 * vertex event and there is no handler for it: both vertices carry on past each
 * other and the face of one tooth's flank picks up an out-and-back excursion along
 * the bisector they shared.
 *
 * The excursion encloses nothing, so the areas stay exact to the last bit and the
 * faces still tile -- which is why every other assertion in this suite passed
 * while it was happening, and why C3 never noticed either: Clipper2's NonZero fill
 * ignores a zero-area spur. D5 has no clipper in between and would triangulate a
 * folded ring.
 *
 * With five teeth the face of contour edge 17 came back as
 * (64.98, 0), (64.98, 30.5), (54.98, 40.5), (61.37, 34.11), (61.37, 3.61), whose
 * fourth point lies exactly on the segment between its second and third. Both the
 * fold and its repair are asserted: `ring_self_crossings` finds no crossing, and
 * `folded_spurs` is non-zero, which says the repair is what removed it rather than
 * the shape having quietly stopped producing one.
 */
TEST(StraightSkeleton, a_comb_of_equal_teeth_does_not_fold_a_face_ring) {
    for (size_t teeth = 2; teeth <= 8; ++teeth) {
        const std::vector<glm::dvec2> ring = comb(teeth, 7.22, 30.5, 50.5);
        const StraightSkeleton skeleton = compute_straight_skeleton(ring);

        check_tiles_polygon(skeleton);
        CHECK((skeleton.stats.split_events) >= size_t{ 1 });
        CHECK_EQ(skeleton.faces.size(), size_t{ 4 } * teeth + 4);
        CHECK_NEAR(total_face_area(skeleton), std::fabs(signed_ring_area(ring)), 1e-9);
        CHECK_NEAR(max_face_vertex_outside(skeleton, ring), 0.0, 1e-6);
        // Each adjacent PAIR of teeth collapses together and leaves one vertex
        // event, so a comb of n teeth leaves n - 1 spurs. Pinned exactly rather
        // than as "more than none", because a repair that fired on geometry that
        // was not folded would be deleting real face vertices, and the area sum
        // above cannot see that -- an out-and-back and a genuine sliver both
        // enclose nothing. Only five teeth and up actually CROSS; below that the
        // fold is a flap that doubles back without passing through itself, which
        // is just as wrong a ring and is repaired the same way.
        CHECK_EQ(skeleton.stats.folded_spurs, teeth - 1);
    }

    // The exact ring the defect was found on, kept at full precision. A shape
    // family that usually reproduces a defect is not a regression test.
    const StraightSkeleton five = compute_straight_skeleton(comb(5, 7.22, 30.5, 50.5));
    CHECK_TRUE(five.complete);
    if (!five.complete) return;
    CHECK_EQ(five.stats.folded_spurs, size_t{ 4 });
    const SkeletonFace* folded = face_of(five, 17);
    CHECK_TRUE(folded != nullptr);
    if (folded != nullptr) {
        CHECK_EQ(ring_self_crossings(folded->ring), size_t{ 0 });
        CHECK_EQ(folded->ring.size(), size_t{ 4 });
        CHECK_NEAR(folded->area, signed_ring_area(folded->ring), 1e-12);
    }
}

/**
 * @brief Combs of unequal teeth split many times over and still tile
 *
 * THE FAMILY THAT COVERS THE SPLIT GUARD. `find_event()` refuses a split against
 * an edge the reflex vertex has already gone past -- `gap < -eps_` -- and nothing
 * else in this suite reaches it: the L, the plus sign and the deep notch all
 * resolve before a reflex vertex can overshoot anything. Removing that one line
 * leaves every other test in this file green while 2848 combs in 3000 come back
 * with overlapping faces, areas up to 168% wrong, and face vertices sixty metres
 * outside the polygon.
 *
 * So the assertions here are the coarse ones rather than the precise ones. Area
 * conservation catches the overlap, `ring_self_crossings` catches the fold, and
 * `max_face_vertex_outside` catches the wavefront marching out of the polygon --
 * and that last one is the assertion that cannot be satisfied by a wrong answer
 * that happens to balance.
 *
 * Counted rather than asserted one shape at a time: a failure here should say how
 * many of the family broke and by how much, not print six hundred lines.
 */
TEST(StraightSkeleton, ragged_combs_split_repeatedly_and_still_tile) {
    Rng rng(0xC0FFEEull);
    size_t shapes = 0;
    size_t incomplete = 0;
    size_t with_crossings = 0;
    size_t with_splits = 0;
    double worst_area_error = 0.0;
    double worst_outside = 0.0;

    for (size_t trial = 0; trial < 400; ++trial) {
        const size_t teeth = 3 + (trial % 6);
        const std::vector<glm::dvec2> ring = ragged_comb(rng, teeth, 40.0);
        const double expected = std::fabs(signed_ring_area(ring));
        if (!(expected > 1.0)) continue;
        ++shapes;

        const StraightSkeleton skeleton = compute_straight_skeleton(ring);
        if (!skeleton.complete) {
            ++incomplete;
            continue;
        }
        if (skeleton.stats.split_events > 0) ++with_splits;

        worst_area_error =
            std::max(worst_area_error,
                     std::fabs(total_face_area(skeleton) - expected) / expected);
        worst_outside = std::max(worst_outside, max_face_vertex_outside(skeleton, ring));
        for (const SkeletonFace& face : skeleton.faces) {
            if (ring_self_crossings(face.ring) > 0) {
                ++with_crossings;
                break;
            }
        }
    }

    CHECK((shapes) > size_t{ 300 });
    CHECK_EQ(incomplete, size_t{ 0 });
    CHECK_EQ(with_crossings, size_t{ 0 });
    // The family has to be REACHING the split path or it proves nothing about it.
    CHECK((with_splits) > shapes / 2);
    CHECK((worst_area_error) < 1e-9);
    // A tenth of a millimetre. Not a tolerance on the geometry -- a face vertex is
    // inside the polygon or the skeleton is wrong -- but on the point-in-ring test
    // at a vertex that sits exactly on the boundary.
    CHECK((worst_outside) < 1e-4);
}

/**
 * @brief Every side of a rectangle jittered per vertex, over the whole scale range
 *
 * THE FAMILY THAT COVERS kMaxVertexSpeed. A vertex whose two edges are within
 * `2 / kMaxVertexSpeed` of exactly opposing has no usable mitre point, and
 * `solve_velocity()` refuses it rather than returning a position a million times
 * too far out. `resolve_flat_loops()`'s last-resort width is tied to that same
 * constant, which is why the handover names the pair a maintenance hazard.
 *
 * `densified_rectangle_tiles_at_every_jitter_scale` above cannot reach it: its
 * jitter comes from a sixteen-entry table reused at every vertex, so the angles it
 * produces are the same sixteen at every scale and none of them lands in the band
 * the cap protects. Drawing the jitter PER VERTEX does land there, and then
 * removing the cap -- or moving it from 1e6 to 1e12 -- puts self-crossing faces on
 * about one ring in seventeen.
 *
 * A ring that comes back INCOMPLETE is not counted as a failure. Refusing is the
 * documented outcome when the geometry defeats the simulation, and the cap exists
 * precisely to make that happen instead of a wrong answer. What is asserted is
 * that nothing comes back complete AND folded.
 */
TEST(StraightSkeleton, per_vertex_jittered_rectangles_never_come_back_folded) {
    const double scales[] = { 1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3 };
    Rng rng(0x5EEDull);

    size_t complete_rings = 0;
    size_t with_crossings = 0;
    size_t degenerate_vertices = 0;
    double worst_area_error = 0.0;

    for (const double scale : scales) {
        for (size_t trial = 0; trial < 300; ++trial) {
            std::vector<glm::dvec2> ring;
            const auto push = [&](double x, double y) {
                ring.push_back({ x + rng.range(-scale, scale), y + rng.range(-scale, scale) });
            };
            for (int i = 0; i < 4; ++i) push(40.0 * i / 4.0, 0.0);
            for (int i = 0; i < 3; ++i) push(40.0, 10.0 * i / 3.0);
            for (int i = 0; i < 4; ++i) push(40.0 - 40.0 * i / 4.0, 10.0);
            for (int i = 0; i < 3; ++i) push(0.0, 10.0 - 10.0 * i / 3.0);

            const StraightSkeleton skeleton = compute_straight_skeleton(ring);
            if (!skeleton.complete) {
                CHECK_TRUE(skeleton.faces.empty());
                continue;
            }
            ++complete_rings;
            degenerate_vertices += skeleton.stats.degenerate_vertices;

            const double expected = std::fabs(signed_ring_area(skeleton.contour));
            worst_area_error =
                std::max(worst_area_error,
                         std::fabs(total_face_area(skeleton) - expected) / expected);
            for (const SkeletonFace& face : skeleton.faces) {
                if (ring_self_crossings(face.ring) > 0) {
                    ++with_crossings;
                    break;
                }
            }
        }
    }

    CHECK((complete_rings) > size_t{ 2000 });
    CHECK_EQ(with_crossings, size_t{ 0 });
    CHECK((worst_area_error) < 1e-4);
    // THE FAMILY HAS TO REACH THE CAP OR IT PROVES NOTHING ABOUT IT. A jitter that
    // never produces a near-opposing pair of edges never abandons a vertex, and
    // then the whole degenerate-vertex mechanism -- the cap, `Wv::finite`, and the
    // last-resort width in resolve_flat_loops() that is tied to the same constant
    // -- is dead code as far as this test is concerned. That is exactly what the
    // sixteen-entry table above does: it yields one degenerate vertex in
    // thousands. Drawing per vertex yields thousands.
    CHECK((degenerate_vertices) > size_t{ 100 });
}

/**
 * @brief A needle sharper than the mitre cap refuses; it does not return nothing
 *
 * Two things at once, and they are the same mechanism seen from both ends.
 *
 * `kMaxVertexSpeed` is the ceiling on how fast a wavefront vertex may move before
 * its mitre point is called meaningless. A vertex's speed is one over the sine of
 * half its interior angle, so a needle 1000 metres long and `d` wide has a speed
 * of about 1000/d: at d = 1 mm the cap is at the threshold and at d = 10 microns
 * it is well past. Nothing else in this suite has a corner anywhere near it, which
 * is why `solve_velocity()` could be made to `return true` unconditionally, or the
 * constant moved from 1e6 to 1e12, with every other test still green.
 *
 * Past the cap every vertex of the needle is abandoned, so build_faces() drops
 * every face -- and the simulation still TERMINATES, because a wavefront with no
 * live vertex is a finished one. That used to come back as `complete` true with an
 * empty face list over 0.0105 square metres of real polygon: not a refusal, an
 * assertion that there is no land there. `run()` now checks that the faces it
 * built actually tile before it says yes.
 *
 * So the assertion is the contract itself, at every width: either the faces tile
 * the needle, or `complete` is false and there are none. Never complete and empty.
 */
TEST(StraightSkeleton, a_needle_sharper_than_the_mitre_cap_refuses_rather_than_lying) {
    const double widths[] = { 1.0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6 };
    size_t refused = 0;
    size_t tiled = 0;

    for (const double d : widths) {
        // A 1000 m needle of width 2d, with a blunt 50 m tail so that the sharp
        // vertex is the only degenerate thing in the ring.
        const std::vector<glm::dvec2> ring = {
            { 0.0, 0.0 }, { 1000.0, d }, { 0.0, 2.0 * d }, { -50.0, d },
        };
        const double expected = std::fabs(signed_ring_area(ring));
        const StraightSkeleton skeleton = compute_straight_skeleton(ring);

        if (!skeleton.complete) {
            // The whole point. A refusal is a refusal: no faces to build on.
            CHECK_TRUE(skeleton.faces.empty());
            ++refused;
            continue;
        }
        ++tiled;
        CHECK_EQ(skeleton.faces.size(), size_t{ 4 });
        CHECK_NEAR(total_face_area(skeleton), expected, 1e-9 * expected);
        for (const SkeletonFace& face : skeleton.faces) {
            CHECK_EQ(ring_self_crossings(face.ring), size_t{ 0 });
        }
        // The mitre point of this corner is a thousand times the width out along
        // the spine, so an implementation that placed the node there instead of
        // capping it would put it far outside a polygon 1000 m long.
        for (const auto& node : skeleton.nodes) {
            CHECK((std::fabs(node.position.x)) <= 1000.0 + 1e-6);
        }
    }

    // Both halves of the family have to be present or the test has drifted into
    // asserting one branch. The blunt end tiles; the sharp end refuses.
    CHECK((tiled) >= size_t{ 4 });
    CHECK((refused) >= size_t{ 1 });

    // WHERE the cap bites, stated as a number. At a width of 1 mm the mitre speed
    // is 1e6 -- the cap exactly -- and the vertex is abandoned while the skeleton
    // is still computable, which is the band the whole mechanism lives in. Raise
    // kMaxVertexSpeed and this reads zero; remove the test in solve_velocity() and
    // it reads zero. Neither changes any other assertion in this file.
    const StraightSkeleton at_the_cap = compute_straight_skeleton({
        { 0.0, 0.0 }, { 1000.0, 1e-3 }, { 0.0, 2e-3 }, { -50.0, 1e-3 },
    });
    CHECK_TRUE(at_the_cap.complete);
    CHECK((at_the_cap.stats.degenerate_vertices) >= size_t{ 1 });

    // And past it the answer is withheld rather than emptied.
    const StraightSkeleton past_the_cap = compute_straight_skeleton({
        { 0.0, 0.0 }, { 1000.0, 1e-5 }, { 0.0, 2e-5 }, { -50.0, 1e-5 },
    });
    CHECK_FALSE(past_the_cap.complete);
    CHECK_TRUE(past_the_cap.faces.empty());
}

/**
 * @brief Random star polygons split, tile, and keep every face inside the polygon
 *
 * The broadest family in the suite and the one that covers the event-time ceiling.
 * `Sim`'s constructor drops any candidate event later than a thousand times the
 * bounding-box diagonal, on the grounds that such a time came out of a near-zero
 * divisor rather than out of the geometry. The factor is presented in the source
 * as a measured value; nothing measured it. Setting it to four passes every other
 * test in this file while losing about one split event in twenty-five, and a lost
 * split is a folded wavefront.
 *
 * A star polygon with neighbouring radii far apart has a deep notch at every one
 * of them, so this family produces thousands of split events over its run -- which
 * is why the split count is asserted as well as the tiling. A family that stopped
 * generating splits would pass the rest of this test while testing nothing.
 */
TEST(StraightSkeleton, random_star_polygons_split_and_tile) {
    Rng rng(0xA5711Aull);
    size_t shapes = 0;
    size_t incomplete = 0;
    size_t with_crossings = 0;
    size_t splits = 0;
    double worst_area_error = 0.0;
    double worst_outside = 0.0;

    for (size_t trial = 0; trial < 400; ++trial) {
        const size_t points = 5 + (trial % 8);
        const std::vector<glm::dvec2> ring = star(rng, points, 6.0, 60.0);
        const double expected = std::fabs(signed_ring_area(ring));
        if (!(expected > 1.0)) continue;
        ++shapes;

        const StraightSkeleton skeleton = compute_straight_skeleton(ring);
        if (!skeleton.complete) {
            ++incomplete;
            continue;
        }
        splits += skeleton.stats.split_events;
        worst_area_error =
            std::max(worst_area_error,
                     std::fabs(total_face_area(skeleton) - expected) / expected);
        worst_outside = std::max(worst_outside, max_face_vertex_outside(skeleton, ring));
        for (const SkeletonFace& face : skeleton.faces) {
            if (ring_self_crossings(face.ring) > 0) {
                ++with_crossings;
                break;
            }
        }
    }

    CHECK((shapes) > size_t{ 300 });
    CHECK_EQ(incomplete, size_t{ 0 });
    CHECK_EQ(with_crossings, size_t{ 0 });
    CHECK((splits) > size_t{ 500 });
    CHECK((worst_area_error) < 1e-9);
    CHECK((worst_outside) < 1e-4);
}

// ============================================================================
// Degenerate but legal input
// ============================================================================

/**
 * @brief A sliver still tiles
 *
 * 100 metres by half a metre: a dual carriageway's median strip, which blocks.cpp
 * emits as a genuine block rather than filtering away. The ridge runs almost the
 * whole length and the end faces are slivers of their own.
 */
TEST(StraightSkeleton, long_thin_sliver_still_tiles) {
    const StraightSkeleton skeleton = compute_straight_skeleton(sliver());

    check_tiles_polygon(skeleton);
    CHECK_EQ(skeleton.faces.size(), size_t{ 4 });
    CHECK_NEAR(total_face_area(skeleton), 50.0, 1e-9);
    CHECK_NEAR(skeleton.stats.max_time, 0.25, 1e-9);
}

/**
 * @brief A near-collinear vertex on a near-rectangle does not stall the wavefront
 *
 * THIS IS THE REGRESSION TEST FOR THE DEFECT THAT MADE THIS FILE USELESS TO C3.
 *
 * A block ring carries every polyline vertex of every street that bounds it, so a
 * straight street contributes a run of vertices that are collinear to within the
 * coordinate precision -- a nanometre or two off the line, not exactly on it. On a
 * rectangular block the wavefront then flattens into a ridge whose vertices have
 * no usable velocity, and the loop has to be closed by the flat-loop sweep.
 *
 * That sweep used to compare the loop's area against a FIXED floor of 1e-12 square
 * metres. A forty-metre ridge a nanometre wide has an area of 3e-11, so it was
 * declared non-flat, was never closed, and the simulation stalled: `complete` came
 * back false and `faces` empty for roughly 40% of rectangular blocks. Worse, in
 * some of them the only candidate event left was between two edges parallel to
 * within rounding, which fired at a time of 1.5e9 and put a face of 5e10 square
 * metres on a block of 500.
 *
 * The loop is swept over a range of offsets because the defect had a WINDOW: it
 * did not fire for an exactly collinear vertex, and it did not fire for a clearly
 * bent one. Testing one offset tests one side of it.
 */
TEST(StraightSkeleton, near_collinear_vertices_do_not_stall_the_wavefront) {
    const double offsets[] = { 0.0, 1e-12, 1e-9, 1e-7, 1e-6, 1e-5, 1e-4, 1e-2 };
    for (const double offset : offsets) {
        // A 20 x 10 rectangle whose bottom edge is two edges meeting at (10, off).
        const std::vector<glm::dvec2> ring = {
            { 0.0, 0.0 }, { 10.0, offset }, { 20.0, 0.0 }, { 20.0, 10.0 }, { 0.0, 10.0 },
        };
        const StraightSkeleton skeleton = compute_straight_skeleton(ring);

        CHECK_TRUE(skeleton.complete);
        if (!skeleton.complete) continue;
        CHECK_EQ(skeleton.faces.size(), size_t{ 5 });
        const double expected = std::fabs(signed_ring_area(skeleton.contour));
        CHECK_NEAR(total_face_area(skeleton), expected, 1e-6 * expected);
        for (const SkeletonFace& face : skeleton.faces) {
            CHECK_EQ(ring_self_crossings(face.ring), size_t{ 0 });
        }
        // The wavefront of a 20 x 10 box stops at 5, and raising the middle of the
        // bottom edge by `offset` lowers that by at most half of it. Tied to the
        // offset rather than left at a flat tolerance, so the largest offset -- the
        // one that is a real bend rather than a rounding artefact -- is still held
        // to an answer that depends on the geometry.
        CHECK_NEAR(skeleton.stats.max_time, 5.0, 0.5 * offset + 1e-6);
    }
}

/**
 * @brief A rectangle whose every edge carries interior vertices still tiles
 *
 * The shape a real block has: four streets, each contributing several polyline
 * points, none of them exactly on the line their neighbours define. This is the
 * input that used to come back with a face of 5e10 square metres.
 *
 * The jitter is deterministic -- a fixed table, not a random generator -- so a
 * failure here is reproducible from the source alone.
 */
TEST(StraightSkeleton, densified_rectangle_tiles_at_every_jitter_scale) {
    const double jitter[] = { 0.0, 1e-9, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3 };
    // Sixteen fixed offsets in [-1, 1], reused scaled. Any spread does; what
    // matters is that no two edges end up exactly parallel.
    const double wobble[16] = { 0.31,  -0.72, 0.08,  0.95,  -0.44, 0.63,  -0.17, 0.51,
                                -0.88, 0.24,  0.79,  -0.36, 0.12,  -0.61, 0.47,  -0.93 };

    for (const double scale : jitter) {
        std::vector<glm::dvec2> ring;
        size_t w = 0;
        const auto push = [&](double x, double y) {
            ring.push_back({ x + wobble[w % 16] * scale, y + wobble[(w + 7) % 16] * scale });
            ++w;
        };
        for (int i = 0; i < 4; ++i) push(40.0 * i / 4.0, 0.0);
        for (int i = 0; i < 3; ++i) push(40.0, 10.0 * i / 3.0);
        for (int i = 0; i < 4; ++i) push(40.0 - 40.0 * i / 4.0, 10.0);
        for (int i = 0; i < 3; ++i) push(0.0, 10.0 - 10.0 * i / 3.0);

        const StraightSkeleton skeleton = compute_straight_skeleton(ring);
        CHECK_TRUE(skeleton.complete);
        if (!skeleton.complete) continue;

        const double expected = std::fabs(signed_ring_area(skeleton.contour));
        CHECK_NEAR(total_face_area(skeleton), expected, 1e-4 * expected);
        // 400 square metres, not 5e10. Stated as an absolute ceiling as well,
        // because a relative tolerance on a number that blew up by eight orders of
        // magnitude reads the same as one that was merely a little off.
        CHECK((total_face_area(skeleton)) < 1000.0);
        for (const SkeletonFace& face : skeleton.faces) {
            CHECK_EQ(ring_self_crossings(face.ring), size_t{ 0 });
        }
    }
}

/**
 * @brief The exact block rings that two separate defects came out wrong on
 *
 * Kept verbatim, at full precision, because a shape family that USUALLY reproduces
 * a defect is not a regression test. The generated family above covers the
 * behaviour; these six cover the two specific failures, and each was found by
 * breaking the fix and searching for an input that noticed.
 *
 * The first three are near-rectangular blocks whose sides carry interior vertices
 * a few microns off the line. With no ceiling on event times, the last thing left
 * to schedule on them is a pair of edges parallel to within rounding, closing at
 * 1e-9 -- an event at a time of 1e9, a skeleton node a billion metres away, and a
 * face area eight orders of magnitude too large. The assertion that catches that is
 * the ABSOLUTE ceiling on the total, not the relative one.
 *
 * The last three are the same shape more strongly perturbed, where every side has
 * slight reflex kinks in it. They failed while an implausible EDGE event caused its
 * vertex to be skipped entirely, taking that vertex's split-event scan with it: a
 * missing split, a folded wavefront, and face rings that cross themselves.
 */
TEST(StraightSkeleton, block_rings_that_two_fixed_defects_came_out_wrong_on) {
    const std::vector<std::vector<glm::dvec2>> rings = {
        // Event time unbounded: total face area came back as 5e10 over a block of 500.
        { { -8.1100864883683439e-06, -1.6978339571158338e-06 }, { 16.666669869303547, -4.7741935100573275e-06 }, { 33.333337682770988, -7.1416915841064226e-06 }, { 49.999992408413689, -1.5405442435586525e-06 }, { 50.000006402827957, 10.000003903526878 }, { 50.000006938467699, 19.999999402402178 }, { 33.333326069725508, 20.000008831882752 }, { 16.666663531389236, 20.00000641944122 }, { -4.4068740136710718e-06, 19.999991667892424 }, { 3.7404252242570049e-06, 9.9999991709976666 } },
        { { -3.9041665735527326e-06, 3.5989090284609559e-06 }, { 16.666669873687425, -9.4407486179525484e-07 }, { 33.333331747151625, -2.8914130390701766e-06 }, { 50.000004982105253, 2.6803593010051756e-06 }, { 50.000005311996034, 11.500002559974828 }, { 50.000001908184096, 22.999997472353673 }, { 33.333335726058131, 23.00000418333676 }, { 16.666667768974666, 22.999997652553638 }, { 5.0619633864262888e-06, 22.999997412271089 }, { 5.2426052562740719e-06, 11.499995618888725 } },
        { { 5.3863088995286042e-06, 6.1550170387054943e-06 }, { 12.500000905012683, -3.1367987198733675e-08 }, { 24.999998242802924, -1.9100995796859857e-06 }, { 37.499996691484675, 1.1965268372358226e-06 }, { 49.99999663764104, -6.2292403305530854e-07 }, { 50.000005333637709, 11.499994132768091 }, { 50.000005821656679, 23.00000329770916 }, { 37.499997063377876, 22.999994240016189 }, { 24.999994010695598, 22.999998390548491 }, { 12.50000523710686, 22.999993758746744 }, { 1.5349955345035312e-06, 23.000002659213493 }, { -1.9152127251611364e-06, 11.499995234455193 } },
        // A rejected edge event swallowing its vertex's split-event scan.
        { { -0.0036528219599508546, -0.0011739298897661876 }, { 6.6666128279105497, 0.0051579960044760261 }, { 13.331965669336324, -0.0053245071776970437 }, { 20.005420355510491, 0.0033005564598427599 }, { 19.996563267618388, 5.5056141306418249 }, { 19.993890515334293, 10.99888185888836 }, { 13.327798119521896, 11.005689352230467 }, { 6.6712272598887834, 10.998418788294869 }, { 0.0010336682590255542, 10.996526198582364 }, { -0.0057130454796038063, 5.4967704768653354 } },
        { { 0.00015634169773592541, -0.00082129415452387696 }, { 9.9957597743296418, 0.0017517740487160149 }, { 20.001847265047871, -0.0038751987786751901 }, { 29.996265628488079, 0.0024606092454016269 }, { 39.998382722330305, 1.2264297301327332e-05 }, { 39.998933646157376, 4.6625635698052728 }, { 39.996643683353035, 9.3299961519302173 }, { 39.998120319623531, 14.002146780045123 }, { 30.002092425632014, 13.997357655858895 }, { 19.996950005537499, 14.000769147849287 }, { 9.9988868814115026, 13.998619497278224 }, { -0.0040188445165599846, 13.996242714943651 }, { -0.0012696929384869767, 9.3330546574783391 }, { -0.00037442933253391568, 4.6682174346014707 } },
        { { 0.0009140691540448732, 0.00045295965652546524 }, { 14.994469468980313, 0.0013375715966381712 }, { 29.993320789684983, -0.0041811643738145863 }, { 44.997321075747607, 0.0023423311109690105 }, { 59.998416443744368, 0.0056329484683962415 }, { 60.004556987280012, 8.6700269664945644 }, { 59.995615682722857, 17.331531879714596 }, { 60.003029728536184, 26.005739072385449 }, { 45.00506753453314, 25.993602972783901 }, { 30.002749301923632, 25.996047589260332 }, { 14.995752202862887, 25.995314454063816 }, { 0.0066657323213955141, 26.00125795219461 }, { 0.003812324160157061, 17.339225523947558 }, { -0.0054798873601953025, 8.6674539733356486 } },
    };

    for (const std::vector<glm::dvec2>& ring : rings) {
        const StraightSkeleton skeleton = compute_straight_skeleton(ring);
        CHECK_TRUE(skeleton.complete);
        if (!skeleton.complete) continue;

        const double expected = std::fabs(signed_ring_area(skeleton.contour));
        const double total = total_face_area(skeleton);
        CHECK_NEAR(total, expected, 1e-4 * expected);
        // Stated absolutely as well. A relative tolerance on a number that blew up
        // by eight orders of magnitude reads exactly like one that was a little
        // off, and the whole point of these rings is which of the two it was.
        CHECK((total) < 2.0 * expected);
        for (const SkeletonFace& face : skeleton.faces) {
            CHECK_EQ(ring_self_crossings(face.ring), size_t{ 0 });
        }
        // A COARSER tolerance than the default, and the coarseness is the honest
        // part: these rings are perturbed by about ten microns, so the skeleton's
        // node positions are good to about ten microns and no better. Two nodes
        // three microns apart on one ridge are one point here. What this still
        // catches is a fold, which crosses metres deep.
        CHECK_EQ(arc_crossings(skeleton, 1e-4), size_t{ 0 });
    }
}

/**
 * @brief A sharp spike is resolved rather than run away with
 *
 * A vertex of a few degrees has a mitre velocity of tens, so the naive event time
 * is enormous and the naive node position is far outside the polygon. Either the
 * skeleton comes out tiled or it refuses; what it must not do is come back
 * complete with a node hundreds of metres away.
 */
TEST(StraightSkeleton, sharp_spike_either_tiles_or_refuses) {
    const std::vector<glm::dvec2> ring = {
        { 0.0, 0.0 }, { 60.0, 0.5 }, { 0.0, 1.0 }, { -2.0, 0.5 },
    };
    const StraightSkeleton skeleton = compute_straight_skeleton(ring);

    if (!skeleton.complete) {
        CHECK_TRUE(skeleton.faces.empty());
        return;
    }
    check_tiles_polygon(skeleton);
    for (const auto& node : skeleton.nodes) {
        CHECK((node.position.x) < 100.0);
        CHECK((node.position.x) > -100.0);
    }
}

// ============================================================================
// Refusals
// ============================================================================

/**
 * @brief A self-intersecting ring is refused, and refused means no faces
 *
 * A bowtie has no interior for a wavefront to shrink through, so every event time
 * computed against it is meaningless. The contract is that `complete` is false AND
 * `faces` is empty -- half an answer is the dangerous outcome, because a caller
 * that checks only `faces.empty()` would build on the partial arc set.
 */
TEST(StraightSkeleton, self_intersecting_ring_is_refused) {
    // Deliberately NOT the symmetric bowtie. That one has two lobes of equal
    // signed area, so its shoelace area is exactly zero and it is refused by the
    // area check before the crossing scan ever runs -- which passes an assertion
    // on `complete` while proving nothing about self-intersection. This ring has a
    // spike that crosses the bottom edge and encloses 25 square metres.
    const std::vector<glm::dvec2> crossed = {
        { 0.0, 0.0 }, { 10.0, 0.0 }, { 10.0, 10.0 }, { 5.0, -5.0 }, { 0.0, 10.0 },
    };
    CHECK((std::fabs(signed_ring_area(crossed))) > 1.0);

    const StraightSkeleton skeleton = compute_straight_skeleton(crossed);

    CHECK_FALSE(skeleton.complete);
    CHECK_TRUE(skeleton.faces.empty());
    CHECK((skeleton.stats.self_intersections) > size_t{ 0 });

    // And the symmetric bowtie is still refused, by the area check.
    const StraightSkeleton bowtie = compute_straight_skeleton({
        { 0.0, 0.0 }, { 10.0, 10.0 }, { 10.0, 0.0 }, { 0.0, 10.0 },
    });
    CHECK_FALSE(bowtie.complete);
    CHECK_TRUE(bowtie.faces.empty());
}

/**
 * @brief Degenerate input is refused rather than guessed at
 *
 * Fewer than three distinct points, and three collinear points, both enclose
 * nothing. The second is the one worth a test: it HAS three points and passes a
 * naive size check, and the wavefront over it never terminates.
 */
TEST(StraightSkeleton, degenerate_rings_are_refused) {
    const StraightSkeleton none = compute_straight_skeleton({});
    CHECK_FALSE(none.complete);
    CHECK_TRUE(none.faces.empty());

    const StraightSkeleton two =
        compute_straight_skeleton({ { 0.0, 0.0 }, { 1.0, 0.0 } });
    CHECK_FALSE(two.complete);
    CHECK_TRUE(two.faces.empty());

    const StraightSkeleton collinear =
        compute_straight_skeleton({ { 0.0, 0.0 }, { 1.0, 0.0 }, { 2.0, 0.0 } });
    CHECK_FALSE(collinear.complete);
    CHECK_TRUE(collinear.faces.empty());

    // Repeated points are removed, not refused: a ring that came through a
    // coordinate transform picks them up honestly.
    const StraightSkeleton duplicated = compute_straight_skeleton({
        { 0.0, 0.0 }, { 0.0, 0.0 }, { 10.0, 0.0 }, { 10.0, 10.0 }, { 0.0, 10.0 },
    });
    CHECK_TRUE(duplicated.complete);
    CHECK_EQ(duplicated.contour.size(), size_t{ 4 });
    CHECK_EQ(duplicated.faces.size(), size_t{ 4 });
}

/**
 * @brief The iteration cap refuses rather than returns half a skeleton
 *
 * The cap is a backstop, and what makes it a backstop rather than a hazard is that
 * hitting it produces NO faces. A plus sign needs several events; one is not
 * enough.
 */
TEST(StraightSkeleton, hitting_the_iteration_cap_produces_no_faces) {
    SkeletonConfig config;
    config.max_iterations = 1;
    const std::vector<glm::dvec2> ring = {
        { 10.0, 0.0 },  { 20.0, 0.0 },  { 20.0, 10.0 }, { 30.0, 10.0 },
        { 30.0, 20.0 }, { 20.0, 20.0 }, { 20.0, 30.0 }, { 10.0, 30.0 },
        { 10.0, 20.0 }, { 0.0, 20.0 },  { 0.0, 10.0 },  { 10.0, 10.0 },
    };
    const StraightSkeleton skeleton = compute_straight_skeleton(ring, config);

    CHECK_FALSE(skeleton.complete);
    CHECK_TRUE(skeleton.faces.empty());
}

/**
 * @brief A ring past max_vertices is refused outright
 *
 * Refusing is visible; running for minutes is not.
 */
TEST(StraightSkeleton, oversized_ring_is_refused) {
    SkeletonConfig config;
    config.max_vertices = 3;
    const StraightSkeleton skeleton = compute_straight_skeleton(square(), config);

    CHECK_FALSE(skeleton.complete);
    CHECK_TRUE(skeleton.faces.empty());
}

// ============================================================================
// Contract of the returned contour
// ============================================================================

/**
 * @brief A clockwise ring is reversed, and `contour` says so
 *
 * Block::holes are clockwise by design, so a caller holding one is not making a
 * mistake. What it must not do is read face edge indices against the ring it
 * passed in: those index StraightSkeleton::contour, which is the REVERSED ring.
 * The two skeletons are geometrically identical and their edge numbering is not.
 */
TEST(StraightSkeleton, clockwise_input_is_reversed_and_reported) {
    std::vector<glm::dvec2> ring = rectangle();
    std::reverse(ring.begin(), ring.end());
    CHECK((signed_ring_area(ring)) < 0.0);

    const StraightSkeleton skeleton = compute_straight_skeleton(ring);

    check_tiles_polygon(skeleton);
    CHECK((signed_ring_area(skeleton.contour)) > 0.0);
    CHECK_EQ(skeleton.contour.size(), ring.size());
    // Same geometry as the anticlockwise version: same face areas, same ridge.
    CHECK_NEAR(total_face_area(skeleton), 300.0, 1e-9);
    CHECK_NEAR(max_interior_time(skeleton), 5.0, 1e-9);
}

/**
 * @brief The same ring gives the same skeleton, bit for bit
 *
 * Determinism is a stated contract of this file, and a caller that caches a
 * skeleton against a block key depends on it. Compared exactly rather than with a
 * tolerance, because "the same to within an epsilon" is precisely what a
 * container-order dependence would produce.
 */
TEST(StraightSkeleton, the_same_ring_gives_the_same_skeleton) {
    const std::vector<glm::dvec2> ring = ell();
    const StraightSkeleton a = compute_straight_skeleton(ring);
    const StraightSkeleton b = compute_straight_skeleton(ring);

    CHECK_EQ(a.complete, b.complete);
    CHECK_EQ(a.nodes.size(), b.nodes.size());
    CHECK_EQ(a.arcs.size(), b.arcs.size());
    CHECK_EQ(a.faces.size(), b.faces.size());
    if (a.nodes.size() != b.nodes.size()) return;

    for (size_t i = 0; i < a.nodes.size(); ++i) {
        CHECK_EQ(a.nodes[i].position.x, b.nodes[i].position.x);
        CHECK_EQ(a.nodes[i].position.y, b.nodes[i].position.y);
        CHECK_EQ(a.nodes[i].time, b.nodes[i].time);
    }
    for (size_t i = 0; i < a.arcs.size(); ++i) {
        CHECK_EQ(a.arcs[i].from, b.arcs[i].from);
        CHECK_EQ(a.arcs[i].to, b.arcs[i].to);
        CHECK_EQ(a.arcs[i].left_edge, b.arcs[i].left_edge);
        CHECK_EQ(a.arcs[i].right_edge, b.arcs[i].right_edge);
    }
}

/**
 * @brief Every arc is equidistant from the two contour edges it names
 *
 * SkeletonArc::left_edge and ::right_edge are how a caller tells which land
 * belongs to which street, and an arc that bisects the wrong pair is invisible in
 * the geometry: the segment is still in the right place. Checked at the arc's
 * midpoint against the two edges' supporting LINES, which is the definition.
 */
TEST(StraightSkeleton, every_arc_bisects_the_two_edges_it_names) {
    const StraightSkeleton skeleton = compute_straight_skeleton(ell());
    CHECK_TRUE(skeleton.complete);
    if (!skeleton.complete) return;
    CHECK((skeleton.arcs.size()) > size_t{ 0 });

    const size_t n = skeleton.contour.size();
    const auto line_distance = [&](uint32_t edge, const glm::dvec2& p) {
        const glm::dvec2 a = skeleton.contour[edge];
        const glm::dvec2 b = skeleton.contour[(edge + 1) % n];
        const glm::dvec2 dir = glm::normalize(b - a);
        return std::fabs(cross2(dir, p - a));
    };

    for (const SkeletonArc& arc : skeleton.arcs) {
        if (arc.left_edge == stratum::geometry::kNoSkeletonEdge) continue;
        if (arc.right_edge == stratum::geometry::kNoSkeletonEdge) continue;
        const glm::dvec2 mid =
            0.5 * (skeleton.nodes[arc.from].position + skeleton.nodes[arc.to].position);
        CHECK_NEAR(line_distance(arc.left_edge, mid), line_distance(arc.right_edge, mid), 1e-6);
    }
}

// ============================================================================
// vertex_offset_velocity
// ============================================================================

/**
 * @brief The offset velocity puts the vertex on BOTH offset lines
 *
 * Exported so the offset lot subdivision and the skeleton cannot disagree at a
 * corner, so the test is the defining property rather than a remembered number:
 * move the vertex by v * d and it must be exactly d from both of its own edges'
 * lines, on the inward side.
 */
TEST(StraightSkeleton, offset_velocity_lands_on_both_offset_lines) {
    struct Corner {
        glm::dvec2 prev, vertex, next;
    };
    const Corner corners[] = {
        { { 0.0, 0.0 }, { 10.0, 0.0 }, { 10.0, 10.0 } },   // right angle
        { { 0.0, 0.0 }, { 10.0, 0.0 }, { 11.0, 6.0 } },    // acute-ish
        { { 0.0, 0.0 }, { 10.0, 0.0 }, { 30.0, 1.0 } },    // very shallow
        { { 0.0, 0.0 }, { 10.0, 0.0 }, { 5.0, -3.0 } },    // reflex
    };
    const double d = 2.0;

    for (const Corner& c : corners) {
        glm::dvec2 velocity{ 0.0 };
        CHECK_TRUE(vertex_offset_velocity(c.prev, c.vertex, c.next, velocity));
        const glm::dvec2 moved = c.vertex + velocity * d;

        const glm::dvec2 in = glm::normalize(c.vertex - c.prev);
        const glm::dvec2 out = glm::normalize(c.next - c.vertex);
        // Inward normal of an anticlockwise ring is the edge direction turned a
        // quarter anticlockwise, so a positive signed distance IS inward.
        CHECK_NEAR(cross2(in, moved - c.prev), d, 1e-9);
        CHECK_NEAR(cross2(out, moved - c.vertex), d, 1e-9);
    }
}

/**
 * @brief A sharp corner's offset velocity is longer than one
 *
 * That is the mitre, and it is the reason an offset ring is not the vertices
 * pushed along their own normals. A right angle gives sqrt(2); a shallower corner
 * gives more. Without this the offset band would pinch at every corner.
 */
TEST(StraightSkeleton, offset_velocity_lengthens_at_a_sharp_corner) {
    glm::dvec2 right{ 0.0 };
    CHECK_TRUE(vertex_offset_velocity({ 0.0, 0.0 }, { 10.0, 0.0 }, { 10.0, 10.0 }, right));
    CHECK_NEAR(glm::length(right), std::sqrt(2.0), 1e-12);

    glm::dvec2 sharp{ 0.0 };
    CHECK_TRUE(vertex_offset_velocity({ 0.0, 0.0 }, { 10.0, 0.0 }, { 1.0, 2.0 }, sharp));
    CHECK((glm::length(sharp)) > glm::length(right));

    glm::dvec2 straight{ 0.0 };
    CHECK_TRUE(vertex_offset_velocity({ 0.0, 0.0 }, { 10.0, 0.0 }, { 20.0, 0.0 }, straight));
    CHECK_NEAR(glm::length(straight), 1.0, 1e-12);
}

/**
 * @brief The offset velocity refuses rather than returning NaN
 *
 * glm::normalize of a zero vector is NaN, and a NaN coordinate propagates silently
 * through every polygon downstream -- a lot ring of NaNs still has a size() and
 * still draws. The bool return is the whole point of the function's signature and
 * the out parameter must be left alone on failure.
 */
TEST(StraightSkeleton, offset_velocity_refuses_a_degenerate_corner) {
    glm::dvec2 velocity{ 7.0, 7.0 };

    // Zero-length incoming edge.
    CHECK_FALSE(vertex_offset_velocity({ 10.0, 0.0 }, { 10.0, 0.0 }, { 10.0, 10.0 }, velocity));
    CHECK_EQ(velocity.x, 7.0);
    CHECK_EQ(velocity.y, 7.0);

    // Zero-length outgoing edge.
    CHECK_FALSE(vertex_offset_velocity({ 0.0, 0.0 }, { 10.0, 0.0 }, { 10.0, 0.0 }, velocity));
    CHECK_EQ(velocity.x, 7.0);

    // A 180-degree spike: the two edges fold exactly back on each other, so the
    // meeting point of their offset lines runs off to infinity.
    CHECK_FALSE(vertex_offset_velocity({ 0.0, 0.0 }, { 10.0, 0.0 }, { 0.0, 0.0 }, velocity));
    CHECK_EQ(velocity.x, 7.0);
}

/**
 * @brief signed_ring_area is positive anticlockwise and zero below three points
 *
 * The sign IS the winding contract that the whole file and blocks.cpp share.
 */
TEST(StraightSkeleton, signed_ring_area_carries_the_winding) {
    CHECK_NEAR(signed_ring_area(square()), 100.0, 1e-12);

    std::vector<glm::dvec2> clockwise = square();
    std::reverse(clockwise.begin(), clockwise.end());
    CHECK_NEAR(signed_ring_area(clockwise), -100.0, 1e-12);

    CHECK_NEAR(signed_ring_area({ { 0.0, 0.0 }, { 1.0, 0.0 } }), 0.0, 1e-12);
    CHECK_NEAR(signed_ring_area({}), 0.0, 1e-12);
}
