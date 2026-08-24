/**
 * @file junction_polygon.cpp
 * @brief Implementation of the junction carriageway footprint and its fill
 *
 * Two walks and nothing else.
 *
 *   1. Around the node, in ascending bearing order, appending each arm's cut
 *      cross-section and then the CORNER between it and the next arm.
 *   2. Over the resulting ring with earcut, mapping 2D local metres into world
 *      space at one constant height.
 *
 * ### The corner is the whole job
 *
 * A junction ring is trivial apart from what happens between two arms. Arm k
 * ends at a cut face two vertices wide; arm k+1 starts at another. Joining them
 * with a straight chord gives the old disc-shaped nothing. Joining them with an
 * arc tangent to both arms' carriageway edges gives an intersection.
 *
 * The corner is built from two LINES, not from the two points:
 *
 *     line A: through ends[k].carriage_left,      direction ends[k].direction
 *     line B: through ends[k+1].carriage_right,   direction ends[k+1].direction
 *
 * Both directions LEAVE the node. The ring walks INWARD along line A -- from the
 * cut face back towards the node -- turns at their intersection C, and walks
 * OUTWARD along line B to the next cut face. So the ring's own travel direction
 * over the corner is
 *
 *     u_in  = -ends[k].direction
 *     u_out = +ends[k+1].direction
 *
 * and every decision below is made on those two, never on the raw bearings.
 *
 * ### Which way the corner turns
 *
 * For the ordinary junction corner the ring turns RIGHT, that is
 * `cross2(u_in, u_out) < 0`. That is not a typo and not a winding error: the
 * carriageway footprint of a four-way junction is a PLUS shape, its four quadrant
 * corners are reflex vertices of the counter-clockwise ring, and rounding one
 * pushes the boundary OUTWARD, away from the centroid, adding the wedge of
 * asphalt a vehicle turns through. That outward bulge is the fillet.
 *
 * A corner that turns LEFT is the exception. It appears on the WRAP-AROUND pair
 * at a node whose arms all leave within a half plane -- two ways forking at a
 * very acute angle, a slip road peeling off a main road -- where the angular gap
 * the corner has to span exceeds 180 degrees, and the ground it spans is the BACK
 * of the node. The ring is taken THROUGH its corner point, unrounded, so the fill
 * wraps around the back of the node the way the cut faces wrap around its front;
 * an arc there would bulge into the junction rather than out of it. The chord is
 * the fallback, and it is a poor one -- it passes on the wrong side of the node,
 * so the polygon stops containing the node it was built for.
 *
 * Every corner point, reflex or not, has to lie near the junction to be used at
 * all. Two nearly parallel arms meet at a vanishing angle, kilometres away, and
 * the arc built around that meeting point is tangent over a distance that
 * diverges with the turn. See FilletConfig::max_corner_reach_factor. The chord is
 * also what a corner gets when its intersection lands past either cut face, which
 * is the clamped-trim version of the same failure.
 *
 * ### Cut faces that cross
 *
 * Two arms whose trims were both cut short of what their pair demanded still
 * overlap, and their cut faces then cross -- a bowtie, and the commonest
 * self-intersecting junction in a real extract. Where two adjacent faces cross,
 * both are cut back to the crossing point, so the ring stays simple and bounds
 * exactly the ground the two mouths cover between them.
 *
 * ### Winding
 *
 * The ring is counter-clockwise in 2D local metres, and each arm contributes
 * `carriage_right` then `carriage_left` in that order. Worked through for an arm
 * leaving along +x: its cut face runs from (t, -h) to (t, +h), so the walk is in
 * +y with the node -- the interior -- on its left. Counter-clockwise.
 *
 * The world mapping `(x, y_2d) -> (x, height, -y_2d)` negates one axis and so
 * reverses handedness, which invites a compensating flip. It must not have one.
 * For the counter-clockwise 2D triangle (0,0), (1,0), (0,1) the mapped points are
 * (0,h,0), (1,h,0), (0,h,-1) and their face cross product is
 * `cross((1,0,0), (0,0,-1)) = (0,+1,0)`. Straight up, in the vertex order given.
 * Emitting the same order the 2D triangle had is therefore already correct, and
 * triangulate_junction() enforces it per triangle from the 2D signed area rather
 * than trusting earcut's output orientation.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 */

#include "osm/road/junction_polygon.hpp"

// uv_tiling(): the junction fill must use the SAME tiling table as the ribbons
// feeding it, or texel density steps at every arm mouth.
#include "osm/road/corridor.hpp"

#include <mapbox/earcut.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

// ============================================================================
// Earcut adapter for glm::dvec2
//
// Token-identical to the one in src/osm/mesh_builder.cpp on purpose. These are
// explicit specialisations of a class template with inline members, so repeating
// the same definition in a second translation unit is the same as including one
// header twice; diverging from it would not be.
// ============================================================================
namespace mapbox {
namespace util {

template <>
struct nth<0, glm::dvec2> {
    inline static double get(const glm::dvec2& t) { return t.x; }
};

template <>
struct nth<1, glm::dvec2> {
    inline static double get(const glm::dvec2& t) { return t.y; }
};

} // namespace util
} // namespace mapbox

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/// |sin| between two offset lines below which they are parallel and never meet
constexpr double kParallelEpsilon = 1e-9;

/// Fillet points closer than this to their neighbour in the ring are dropped, metres
constexpr double kWeldEpsilon = 1e-9;

/// Hard ceiling on the vertices one arc may spend, whatever the config asks for
constexpr int kMaxArcSegments = 64;

/// Radians in a quarter turn; the unit FilletConfig::segments_per_quarter_turn counts in
constexpr double kQuarterTurn = 1.57079632679489661923;

/// Subtracted before the segment-count ceil() so an exact quarter turn cannot round up
constexpr double kSegmentBias = 1e-9;

/**
 * @brief Relative tolerance below which a cross product counts as exactly zero
 *
 * The determinants in segments_cross() are AREAS, so a fixed tolerance means
 * nothing at one junction size and everything at another. Scaling by the two
 * lengths that produced the determinant is what makes it a distance test again.
 *
 * Exact zero is not an option here. A fillet's tangent point lies precisely ON
 * the offset line it was constructed from, so every corner puts a run of exactly
 * collinear points into the ring, and their determinants come back as rounding
 * noise of either sign. Straddle-testing that noise reports a crossing on a
 * perfectly ordinary junction. This is the same trap corridor.cpp documents at
 * kCollinearRelEpsilon, and it is why that value is repeated rather than
 * loosened.
 */
constexpr double kCollinearRelEpsilon = 1e-12;

/// Relative tolerance on twice a triangle's area, against the ring's own extent squared
constexpr double kDegenerateAreaRel = 1e-12;

/// |signed area| below this leaves the ring without a usable area centroid, m^2
constexpr double kMinCentroidArea = 1e-12;

// ============================================================================
// Small 2D helpers
// ============================================================================

/// 2D cross product, positive when @p b turns left of @p a
[[nodiscard]] inline double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

/// Rotate a vector 90 degrees counter-clockwise
[[nodiscard]] inline glm::dvec2 perp_left(const glm::dvec2& v) {
    return glm::dvec2(-v.y, v.x);
}

/// (x, y_2d) -> world, Y up. The single mapping used by the whole road pipeline.
[[nodiscard]] inline glm::vec3 to_world(const glm::dvec2& p, float height) {
    return glm::vec3(static_cast<float>(p.x), height, static_cast<float>(-p.y));
}

/// Squared distance, kept out of line so the weld tests read as distances
[[nodiscard]] inline double dist_sq(const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 d = a - b;
    return glm::dot(d, d);
}

/**
 * @brief Normalise a direction, reporting whether it had a length to normalise
 *
 * An ArmEnd whose direction is degenerate cannot host a corner line, and the
 * corner falls back to its chord rather than dividing by zero.
 */
[[nodiscard]] bool unit_direction(const glm::dvec2& v, glm::dvec2& out) {
    const double len_sq = glm::dot(v, v);
    if (len_sq <= 0.0 || !std::isfinite(len_sq)) {
        return false;
    }
    out = v / std::sqrt(len_sq);
    return true;
}

/**
 * @brief Proper crossing test for two 2D segments
 *
 * Proper only: shared endpoints and collinear overlap do not count. Two ring
 * edges that merely touch are not a bowtie, and the collinear runs a fillet
 * necessarily produces would otherwise all read as intersections. See
 * kCollinearRelEpsilon.
 */
[[nodiscard]] bool segments_cross(const glm::dvec2& a, const glm::dvec2& b,
                                  const glm::dvec2& c, const glm::dvec2& d) {
    const glm::dvec2 ab = b - a;
    const glm::dvec2 cd = d - c;

    const double len_ab = std::sqrt(glm::dot(ab, ab));
    const double len_cd = std::sqrt(glm::dot(cd, cd));

    const auto span = [](const glm::dvec2& p, const glm::dvec2& q) {
        return std::sqrt(std::max(glm::dot(p, p), glm::dot(q, q)));
    };
    const double eps_ab = kCollinearRelEpsilon * len_ab * span(c - a, d - a);
    const double eps_cd = kCollinearRelEpsilon * len_cd * span(a - c, b - c);

    const auto snap = [](double value, double eps) {
        return (std::abs(value) <= eps) ? 0.0 : value;
    };

    const double d1 = snap(cross2(ab, c - a), eps_ab);
    const double d2 = snap(cross2(ab, d - a), eps_ab);
    const double d3 = snap(cross2(cd, a - c), eps_cd);
    const double d4 = snap(cross2(cd, b - c), eps_cd);

    const bool straddles_ab = ((d1 > 0.0) && (d2 < 0.0)) || ((d1 < 0.0) && (d2 > 0.0));
    const bool straddles_cd = ((d3 > 0.0) && (d4 < 0.0)) || ((d3 < 0.0) && (d4 > 0.0));
    return straddles_ab && straddles_cd;
}

/**
 * @brief Exhaustive proper-crossing sweep over a closed ring
 *
 * Adjacent edge pairs are skipped, including the wrap pair (n-1, 0), because
 * those legitimately share an endpoint. The cost is quadratic and deliberately
 * unbounded: a junction ring is two vertices per arm plus a handful per fillet,
 * so tens of vertices at the very worst, and a missed bowtie here punches a hole
 * in the terrain.
 *
 * @param ring Closed ring, first point NOT repeated at the end
 * @return True when two non-adjacent ring edges cross properly
 */
[[nodiscard]] bool ring_self_intersects(const std::vector<glm::dvec2>& ring) {
    const size_t n = ring.size();
    if (n < 4) {
        return false;
    }

    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % n];

        for (size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) {
                continue;   // edge n-1 wraps onto edge 0, so they are adjacent
            }
            if (segments_cross(a, b, ring[j], ring[(j + 1) % n])) {
                return true;
            }
        }
    }
    return false;
}

/// Twice the signed area of a closed ring; positive when counter-clockwise
[[nodiscard]] double ring_double_area(const std::vector<glm::dvec2>& ring) {
    const size_t n = ring.size();
    if (n < 3) {
        return 0.0;
    }
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i) {
        acc += cross2(ring[i], ring[(i + 1) % n]);
    }
    return acc;
}

/**
 * @brief Area centroid of a closed ring, with a vertex-mean fallback
 *
 * The area centroid is the one the curb ring wants, because it is stable when an
 * arm is resampled. It is undefined for a ring of vanishing area -- a fully
 * collapsed junction, or a self-crossing one whose positive and negative lobes
 * cancel -- and the vertex mean stands in there so the UV origin is always a
 * point inside the junction's neighbourhood rather than a NaN.
 */
[[nodiscard]] glm::dvec2 ring_centroid(const std::vector<glm::dvec2>& ring) {
    const size_t n = ring.size();
    if (n == 0) {
        return glm::dvec2(0.0);
    }

    const double double_area = ring_double_area(ring);
    if (std::abs(double_area) > kMinCentroidArea) {
        glm::dvec2 acc(0.0);
        for (size_t i = 0; i < n; ++i) {
            const glm::dvec2& p = ring[i];
            const glm::dvec2& q = ring[(i + 1) % n];
            acc += (p + q) * cross2(p, q);
        }
        return acc / (3.0 * double_area);
    }

    glm::dvec2 mean(0.0);
    for (const glm::dvec2& p : ring) {
        mean += p;
    }
    return mean / static_cast<double>(n);
}

/**
 * @brief Counter-clockwise convex hull, monotone chain
 *
 * The fallback fill for a ring that crosses itself. A hull is not the junction,
 * but it is simple, it is counter-clockwise, and it covers every arm mouth, which
 * is more than earcut's output for a bowtie can promise.
 */
[[nodiscard]] std::vector<glm::dvec2> convex_hull(std::vector<glm::dvec2> points) {
    std::sort(points.begin(), points.end(), [](const glm::dvec2& a, const glm::dvec2& b) {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    });
    points.erase(std::unique(points.begin(), points.end(),
                             [](const glm::dvec2& a, const glm::dvec2& b) {
                                 return a.x == b.x && a.y == b.y;
                             }),
                 points.end());

    const size_t n = points.size();
    if (n < 3) {
        return {};
    }

    std::vector<glm::dvec2> hull(2 * n);
    size_t k = 0;

    // Lower hull, then upper hull. Turning strictly left keeps collinear points
    // out, so the result has no zero-area triangles for earcut to trip over.
    for (size_t i = 0; i < n; ++i) {
        while (k >= 2 && cross2(hull[k - 1] - hull[k - 2], points[i] - hull[k - 2]) <= 0.0) {
            --k;
        }
        hull[k++] = points[i];
    }
    for (size_t i = n - 1, lower = k + 1; i-- > 0;) {
        while (k >= lower && cross2(hull[k - 1] - hull[k - 2], points[i] - hull[k - 2]) <= 0.0) {
            --k;
        }
        hull[k++] = points[i];
    }

    hull.resize(k - 1);     // the last point repeats the first
    if (hull.size() < 3) {
        return {};
    }
    return hull;
}

/// Append a point unless it repeats the one already at the back of the ring
void push_ring_welded(std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    if (!ring.empty() && dist_sq(p, ring.back()) <= kWeldEpsilon * kWeldEpsilon) {
        return;
    }
    ring.push_back(p);
}

/**
 * @brief Proper intersection point of two 2D segments
 *
 * Proper only, on both segments: the parameters must land strictly inside (0, 1)
 * on each. Two cut faces that merely touch at a corner are not overlapping and
 * must not be clipped, and a shared endpoint is the normal state of a ring.
 *
 * @param a0     First segment start
 * @param a1     First segment end
 * @param b0     Second segment start
 * @param b1     Second segment end
 * @param out_at Receives the crossing point when true is returned
 * @return True when the two segments cross at an interior point of both
 */
[[nodiscard]] bool segment_crossing(const glm::dvec2& a0, const glm::dvec2& a1,
                                    const glm::dvec2& b0, const glm::dvec2& b1,
                                    glm::dvec2& out_at) {
    const glm::dvec2 da = a1 - a0;
    const glm::dvec2 db = b1 - b0;

    const double denom = cross2(da, db);
    if (!std::isfinite(denom)) {
        return false;
    }

    // Relative, because these are areas: a scale-free epsilon would call two
    // hundred-metre motorway faces parallel and two-metre footway faces not.
    const double scale = std::sqrt(glm::dot(da, da) * glm::dot(db, db));
    if (!(std::abs(denom) > kParallelEpsilon * std::max(scale, 1.0))) {
        return false;   // parallel or degenerate: no single crossing point
    }

    const glm::dvec2 delta = b0 - a0;
    const double t = cross2(delta, db) / denom;
    const double u = cross2(delta, da) / denom;
    if (!(t > 0.0 && t < 1.0 && u > 0.0 && u < 1.0)) {
        return false;
    }

    out_at = a0 + da * t;
    return true;
}

// ============================================================================
// Corner construction
// ============================================================================

/**
 * @brief Append the corner between two consecutive arms to the ring
 *
 * On entry the ring already ends with arm A's `carriage_left`, and arm B's
 * `carriage_right` is appended by the caller straight afterwards. This function
 * appends what goes BETWEEN them: either nothing at all, which leaves the chord
 * as a straight chamfer, or the two arc tangent points with the tessellated arc
 * between them.
 *
 * The corner is dropped to its chord whenever an arc cannot be drawn honestly:
 *
 * - either arm end has a degenerate direction;
 * - the two offset lines are parallel, so there is no corner point (an arm's
 *   through-continuation on the far side of a T);
 * - the ring turns LEFT over the corner, which is the wrap-around pair at a node
 *   whose arms all leave within a half plane. Its intersection lies behind the
 *   node and any arc through it spikes out of the back of the junction;
 * - the intersection lands past either cut face, so a tangent point would have to
 *   sit off the end of the arm it is tangent to;
 * - the turn is shallower than FilletConfig::min_arc_angle, where the arc and its
 *   chord differ by less than the vertices would cost;
 * - the radius that actually fits between the two cut faces has fallen below
 *   FilletConfig::min_radius.
 *
 * @param a_end   Cut cross-section of the earlier arm, in bearing order
 * @param b_end   Cut cross-section of the next arm
 * @param a_arm   The earlier arm, for its carriageway width
 * @param b_arm   The next arm
 * @param cfg     Corner rounding tolerances
 * @param chord_target Arm B's `carriage_right`, which the caller appends next.
 *                     Passed so the final arc point can be welded against it.
 * @param ring    In/out; corner points are appended
 */
void append_corner(const ArmEnd& a_end, const ArmEnd& b_end,
                   const ArmRef& a_arm, const ArmRef& b_arm,
                   const FilletConfig& cfg,
                   const glm::dvec2& chord_target,
                   std::vector<glm::dvec2>& ring) {
    glm::dvec2 da(1.0, 0.0);
    glm::dvec2 db(1.0, 0.0);
    if (!unit_direction(a_end.direction, da) || !unit_direction(b_end.direction, db)) {
        return;     // chord
    }

    // Ring travel over the corner: inward along A, outward along B.
    const glm::dvec2 u_in = -da;
    const glm::dvec2 u_out = db;

    const double turn = cross2(u_in, u_out);
    if (turn == 0.0) {
        return;     // dead straight: the chord IS the corner
    }

    // Intersect A's outgoing left offset line with B's incoming right offset
    // line. Both pass through the points the ring already uses, so the corner is
    // built from the SAME geometry the cut faces were, not from an idealised
    // pair of lines through the node.
    const glm::dvec2 pa = a_end.carriage_left;
    const glm::dvec2 pb = b_end.carriage_right;

    const double denom = cross2(da, db);
    if (std::abs(denom) < kParallelEpsilon) {
        return;     // parallel offset lines: no corner point at all, chord
    }

    const glm::dvec2 delta = pb - pa;
    const double alpha_a = cross2(delta, db) / denom;   // C = pa + alpha_a * da
    const double alpha_b = cross2(delta, da) / denom;   // C = pb + alpha_b * db

    // The ring walks INWARD from each cut face, so both parameters must be
    // negative. A non-negative one means the offset lines already crossed
    // outboard of the cut -- the trim was clamped short, and the two arm mouths
    // overlap -- and there is no straight run for a tangent point to sit on.
    if (alpha_a >= 0.0 || alpha_b >= 0.0) {
        return;     // chord
    }

    const glm::dvec2 corner = pa + da * alpha_a;
    const double run_a = -alpha_a;      // straight run available on A's left edge
    const double run_b = -alpha_b;      // and on B's right edge

    // ------------------------------------------------------------------------
    // The corner point has to BE a corner of this junction. Two nearly parallel
    // arms meet at a vanishing angle, so C runs off towards infinity and the arc
    // built around it -- tangent over `radius * tan(theta / 2)`, which diverges
    // with the turn -- leaves the map. A real extract produced a 38,000 m^2
    // junction whose ring reached 2.5 km from its node that way. The trim solve
    // bounds its own mirror of that reserve at kMaxReserveTanHalf; this is the
    // matching bound here, expressed as a distance rather than as a tangent, so
    // it also catches the reflex corner behind the node. See
    // FilletConfig::max_corner_reach_factor.
    // ------------------------------------------------------------------------
    const double reach_slack = std::max(0.0, cfg.max_corner_reach_factor) *
                               (std::max(0.0, a_arm.carriageway_half) +
                                std::max(0.0, b_arm.carriageway_half));
    if (run_a > std::max(0.0, a_arm.trim) + reach_slack ||
        run_b > std::max(0.0, b_arm.trim) + reach_slack) {
        return;     // the "corner" is not near this junction; chord
    }

    if (turn > 0.0) {
        // ------------------------------------------------------------------
        // REFLEX corner: the ring turns LEFT here, so the angular gap this
        // corner spans is more than half a turn. It is the wrap-around pair at
        // a node whose arms all leave within one half plane -- an acute fork, a
        // slip road peeling off a trunk -- and the ground it spans is the BACK
        // of the node, where no arm arrives.
        //
        // Closing it with the chord, which is what this function used to do,
        // draws a straight line from one arm's far cut face to the other's and
        // that line cuts diagonally across the junction. It passes on the wrong
        // side of the node -- so the junction polygon does not contain its own
        // node, and the terrain carve leaves the ground under it unflattened --
        // and on a three-arm fork it crosses the fillet of the OTHER corner,
        // which is the self-intersection an acute fork reports.
        //
        // The honest closure is the corner point itself: the two carriageway
        // edges really do meet at C, behind the node, and taking the boundary
        // round through it wraps the fill around the back of the node the way
        // the cut faces wrap around its front. C is NOT rounded -- there is no
        // outward side to round towards, and an arc through a reflex corner
        // bulges into the junction rather than out of it.
        //
        // What has to be bounded is how far back C sits. Two arms a hair either
        // side of anti-parallel put it hundreds of metres away and a spike out
        // of the back of the junction is worse than the chord ever was, so C is
        // bounded by the reach test above, which rejects a C further behind a
        // cut face than that arm's trim plus FilletConfig::max_corner_reach_factor
        // combined widths -- that is, further than about one carriageway beyond
        // the node.
        // ------------------------------------------------------------------
        push_ring_welded(ring, corner);
        return;
    }

    // Turn magnitude, in [0, pi]: the angle the ring's heading swings through.
    const double theta = std::atan2(std::abs(turn), glm::dot(u_in, u_out));
    if (!std::isfinite(theta) || theta < cfg.min_arc_angle) {
        return;     // a chamfer is indistinguishable from this arc; chord
    }

    const double half_turn = theta * 0.5;
    const double tan_half = std::tan(half_turn);
    if (!std::isfinite(tan_half) || tan_half <= 0.0) {
        return;     // chord
    }

    // Nominal radius: the NARROWER carriageway governs, because that is the one a
    // turning vehicle has to fit into.
    const double nominal = cfg.radius_width_factor *
                           std::min(2.0 * a_arm.carriageway_half, 2.0 * b_arm.carriageway_half);
    double radius = std::clamp(nominal, cfg.min_radius, cfg.max_radius);

    // Fit: the tangent points are `radius * tan(theta/2)` back from the corner
    // along each line, and neither may run past its cut face. Reducing the radius
    // rather than truncating the arc is what keeps the fillet tangent to both
    // arms, and skipping this step is the single commonest way to produce a
    // self-intersecting ring.
    const double fit = std::min(run_a, run_b) / tan_half;
    radius = std::min(radius, fit);

    if (!(radius > 0.0) || radius < cfg.min_radius) {
        return;     // nothing that fits is worth drawing; chord
    }

    const double tangent_dist = radius * tan_half;
    const glm::dvec2 t_a = corner - u_in * tangent_dist;
    const glm::dvec2 t_b = corner + u_out * tangent_dist;

    // Arc centre, `radius` off both lines, on the side the ring turns towards.
    // The turn is always a right turn here, so the centre is to the RIGHT of the
    // incoming heading, which is OUTSIDE the junction: the arc therefore bulges
    // AWAY from the centroid and adds the turning wedge rather than biting into
    // the carriageway.
    const glm::dvec2 centre = t_a - perp_left(u_in) * radius;

    const glm::dvec2 ra = t_a - centre;
    const double start_angle = std::atan2(ra.y, ra.x);
    const double sweep = -theta;    // clockwise, matching the right turn

    // The kSegmentBias subtraction is not cosmetic. theta comes out of atan2, so a
    // dead-square corner lands a fraction of an ulp either side of pi/2 and the
    // bare ceil() then hands one corner of a four-way junction five segments and
    // the next four. The ring stops being symmetric, its centroid drifts off the
    // node, and two junctions that are geometrically identical hash differently.
    int segments = static_cast<int>(
        std::ceil((theta / kQuarterTurn) *
                      static_cast<double>(std::max(1, cfg.segments_per_quarter_turn)) -
                  kSegmentBias));
    segments = std::clamp(segments, 1, kMaxArcSegments);

    const auto push_welded = [&ring](const glm::dvec2& p) { push_ring_welded(ring, p); };

    // Endpoints are placed from the tangent construction rather than from the
    // arc parameterisation, so they land EXACTLY on their offset lines and the
    // collinear runs stay collinear to the last bit.
    push_welded(t_a);
    for (int i = 1; i < segments; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(segments);
        const double angle = start_angle + sweep * t;
        push_welded(centre + glm::dvec2(std::cos(angle), std::sin(angle)) * radius);
    }
    if (dist_sq(t_b, chord_target) > kWeldEpsilon * kWeldEpsilon) {
        push_welded(t_b);
    }
}

} // namespace

// ============================================================================
// Construction
// ============================================================================

void apply_fillet_reserve(const FilletConfig& fillet, TrimConfig& out_trim) {
    out_trim.fillet_radius_width_factor = fillet.radius_width_factor;
    out_trim.fillet_min_radius = fillet.min_radius;
    out_trim.fillet_max_radius = fillet.max_radius;
    out_trim.fillet_min_arc_angle = fillet.min_arc_angle;
}

/**
 * @brief Chain the arm end cross-sections in bearing order, joined by fillet arcs
 *
 * Invariant: every arm contributes EXACTLY two consecutive ring vertices,
 * `carriage_right` then `carriage_left`, and `arm_ring_start[k]` indexes the
 * first of them. Fillet points are welded against their neighbours, arm points
 * never are, even when an arm's carriageway has collapsed to zero width, because
 * the extruded ribbon welds to those two indices and a ring that sometimes has
 * one vertex per arm would put a seam at every approach.
 */
JunctionPolygon build_junction_polygon(const std::vector<ArmRef>& arms,
                                       const std::vector<ArmEnd>& ends,
                                       const FilletConfig& cfg) {
    JunctionPolygon out;

    const size_t arm_count = arms.size();
    if (arm_count < 3 || ends.size() != arm_count) {
        return out;
    }
    for (const ArmEnd& end : ends) {
        if (!end.valid) {
            return out;
        }
    }

    // ------------------------------------------------------------------------
    // Clip adjacent cut faces against each other.
    //
    // Two arms whose trims were BOTH cut short of what their pair demanded still
    // overlap, and their cut faces then cross: the ring walks out along arm k's
    // face, past arm k+1's, and back, which is a bowtie. It is the single
    // commonest way a real extract produces a self-intersecting junction -- every
    // over-trimmed corner is a candidate, and TrimConfig::max_trim_fraction
    // over-trims whenever two junctions sit closer together than one junction is
    // wide.
    //
    // The repair is local and exact: where two adjacent faces cross, both give up
    // their shared corner and meet AT the crossing point instead. The ring is then
    // simple, and it bounds precisely the region the two arm mouths cover between
    // them. The ribbon still overlaps it -- that is what over-trimmed means, and
    // no polygon can undo it -- but the fill underneath is a proper polygon rather
    // than a hull thrown over a bowtie, so the curb ring and the terrain carve can
    // both use it.
    //
    // Every crossing is computed from the UNCLIPPED faces, so the result does not
    // depend on which arm the walk happens to start at.
    // ------------------------------------------------------------------------
    std::vector<glm::dvec2> face_right(arm_count);
    std::vector<glm::dvec2> face_left(arm_count);
    std::vector<bool> corner_clipped(arm_count, false);
    for (size_t k = 0; k < arm_count; ++k) {
        face_right[k] = ends[k].carriage_right;
        face_left[k] = ends[k].carriage_left;
    }

    if (arm_count > 2) {
        std::vector<glm::dvec2> clip_left = face_left;
        std::vector<glm::dvec2> clip_right = face_right;

        for (size_t k = 0; k < arm_count; ++k) {
            const size_t next = (k + 1u) % arm_count;
            glm::dvec2 at(0.0);
            if (!segment_crossing(ends[k].carriage_right, ends[k].carriage_left,
                                  ends[next].carriage_right, ends[next].carriage_left, at)) {
                continue;
            }
            clip_left[k] = at;
            clip_right[next] = at;
            corner_clipped[k] = true;
        }

        // An arm clipped at BOTH ends can come out inverted: its right corner
        // pulled past its left one, which would put a backwards sliver of face
        // into the ring. The two clip points are then collapsed onto their own
        // midpoint, which leaves the arm one point wide -- degenerate, but
        // forwards, and still exactly two ring vertices as arm_ring_start
        // promises.
        for (size_t k = 0; k < arm_count; ++k) {
            const glm::dvec2 face = face_left[k] - face_right[k];
            const double len_sq = glm::dot(face, face);
            if (len_sq > 0.0) {
                const double t_right = glm::dot(clip_right[k] - face_right[k], face) / len_sq;
                const double t_left = glm::dot(clip_left[k] - face_right[k], face) / len_sq;
                if (t_right > t_left) {
                    const glm::dvec2 mid = (clip_right[k] + clip_left[k]) * 0.5;
                    clip_right[k] = mid;
                    clip_left[k] = mid;
                }
            }
            face_right[k] = clip_right[k];
            face_left[k] = clip_left[k];
        }
    }

    out.ring.reserve(arm_count * (2u + static_cast<size_t>(kMaxArcSegments)));
    out.arm_ring_start.reserve(arm_count);

    for (size_t k = 0; k < arm_count; ++k) {
        const size_t next = (k + 1u) % arm_count;

        // Right before left. For an arm leaving along +x that runs -y to +y, so
        // the interior stays on the left of the walk and the ring is CCW.
        out.arm_ring_start.push_back(out.ring.size());
        out.ring.push_back(face_right[k]);
        out.ring.push_back(face_left[k]);

        // A clipped corner IS a point -- the two faces meet there -- so there is
        // no straight run left for a fillet to be tangent over, and the arc's
        // construction would be solved from offset lines the ring no longer
        // reaches. It is left as the zero-length chord between the two coincident
        // ring vertices.
        if (!corner_clipped[k]) {
            append_corner(ends[k], ends[next], arms[k], arms[next], cfg,
                          face_right[next], out.ring);
        }
    }

    // The last corner runs up to arm 0's carriage_right, which is ring.front().
    // Nothing is ever inserted ahead of it, so arm_ring_start stays valid.
    while (out.ring.size() > out.arm_ring_start.back() + 2u &&
           dist_sq(out.ring.back(), out.ring.front()) <= kWeldEpsilon * kWeldEpsilon) {
        out.ring.pop_back();
    }

    if (out.ring.size() < 3) {
        out.ring.clear();
        out.arm_ring_start.clear();
        return out;
    }

    out.centroid = ring_centroid(out.ring);
    out.valid = true;
    out.self_intersecting = ring_self_intersects(out.ring);

    // A clockwise ring is NOT a cosmetic complaint, and treating it as one is how
    // an inverted sliver used to reach the fill and the carve. The corner clipping
    // above collapses an over-clipped arm onto its own midpoint while its two
    // neighbours keep the crossing points they were handed, so on a trident node
    // the walk reverses between those neighbours and the ring comes back simple
    // with negative area -- which ring_self_intersects() correctly does not flag.
    // Flagging it here restores the convex-hull fill and the disc carve for
    // exactly the inputs no polygon rule has produced a usable ring for. Tested
    // strictly negative: a zero-area ring is a junction whose arms all collapsed
    // onto the node, which is degenerate but not mis-wound.
    out.inverted = !out.self_intersecting && ring_double_area(out.ring) < 0.0;

    if (out.self_intersecting) {
        spdlog::warn("build_junction_polygon: junction ring with {} arms and {} vertices "
                     "crosses itself; the fill falls back to its convex hull and the terrain "
                     "carve must not use it as a winding test",
                     arm_count, out.ring.size());
    } else if (out.inverted) {
        spdlog::warn("build_junction_polygon: junction ring with {} arms and {} vertices is "
                     "clockwise; the fill falls back to its convex hull and the terrain carve "
                     "must not use it as a winding test",
                     arm_count, out.ring.size());
    }

    return out;
}

// ============================================================================
// Triangulation
// ============================================================================

/**
 * @brief Fill the junction footprint at one constant height
 *
 * Invariant: every emitted triangle faces +Y. Orientation is decided per triangle
 * from the 2D signed area of its source points, so it does not depend on earcut's
 * output convention, on the ring's winding, or on the hull fallback agreeing with
 * either. Zero-area triangles are dropped instead of emitted with an arbitrary
 * normal.
 */
Mesh triangulate_junction(const JunctionPolygon& poly, float height, MaterialId material) {
    Mesh mesh;

    if (!poly.valid || poly.ring.size() < 3) {
        return mesh;
    }

    // A self-crossing ring has no meaningful interior and a clockwise one bounds
    // the complement of what it looks like, so earcut's output for either is
    // arbitrary. The hull is a visible approximation and the caller counts it.
    const std::vector<glm::dvec2> source =
        poly.needs_hull_fallback() ? convex_hull(poly.ring) : poly.ring;
    if (source.size() < 3) {
        return mesh;
    }

    std::vector<std::vector<glm::dvec2>> polygon;
    polygon.push_back(source);

    const std::vector<uint32_t> tri = mapbox::earcut<uint32_t>(polygon);
    if (tri.size() < 3) {
        return mesh;
    }

    const UVTiling tiling = uv_tiling(material);
    const double u_scale = (tiling.u_metres > 0.0f) ? static_cast<double>(tiling.u_metres) : 1.0;
    const double v_scale = (tiling.v_metres > 0.0f) ? static_cast<double>(tiling.v_metres) : 1.0;

    // Planar projection in the junction's OWN frame, per the plan's UV
    // Convention: the ribbon's arclength does not continue across a junction,
    // because a junction has no single direction of travel. Anchoring on the
    // centroid rather than on an arm keeps the texture still when an arm is
    // added or removed.
    double extent = 0.0;
    mesh.vertices.reserve(source.size());
    for (const glm::dvec2& p : source) {
        Vertex v{};
        v.position = to_world(p, height);
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v.uv = glm::vec2(static_cast<float>((p.x - poly.centroid.x) / u_scale),
                         static_cast<float>((p.y - poly.centroid.y) / v_scale));
        v.color = glm::vec4(1.0f);
        mesh.vertices.push_back(v);

        extent = std::max(extent, std::max(std::abs(p.x - poly.centroid.x),
                                           std::abs(p.y - poly.centroid.y)));
    }

    // Areas scale with the square of the junction, so the degeneracy floor has to
    // as well, or it rejects real triangles on a motorway interchange and accepts
    // slivers on a driveway.
    const double area_eps = std::max(kDegenerateAreaRel * extent * extent, 1e-18);

    mesh.indices.reserve(tri.size());
    for (size_t i = 0; i + 2 < tri.size(); i += 3) {
        const uint32_t i0 = tri[i];
        const uint32_t i1 = tri[i + 1];
        const uint32_t i2 = tri[i + 2];
        if (i0 >= source.size() || i1 >= source.size() || i2 >= source.size()) {
            continue;
        }

        // Twice the 2D signed area, positive when counter-clockwise. A
        // counter-clockwise 2D triangle maps to a +Y face under
        // (x, y) -> (x, height, -y) IN THE SAME VERTEX ORDER; see the file
        // header for the worked cross product. The negated axis is not
        // compensated for, it is accounted for.
        const double twice_area = cross2(source[i1] - source[i0], source[i2] - source[i0]);
        if (std::abs(twice_area) <= area_eps) {
            continue;
        }

        mesh.indices.push_back(i0);
        if (twice_area > 0.0) {
            mesh.indices.push_back(i1);
            mesh.indices.push_back(i2);
        } else {
            mesh.indices.push_back(i2);
            mesh.indices.push_back(i1);
        }
    }

    if (mesh.indices.empty()) {
        mesh.clear();
        return mesh;
    }

    mesh.submeshes.push_back(SubMesh{ 0u, static_cast<uint32_t>(mesh.indices.size()), material });
    mesh.compute_bounds();
    mesh.compute_tangents();
    return mesh;
}

} // namespace stratum::osm::road
