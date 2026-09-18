// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_roof.cpp
 * @brief The six roof shapes behind op_roof.hpp
 *
 * The header says WHAT each kind is and why there is one operation instead of
 * five. This file is about HOW, and about the four places the arithmetic has to
 * be handled carefully:
 *
 *   1. **Every face is oriented against a REFERENCE direction, never assumed.**
 *      A roof face's outward normal is roughly "outward from its own contour
 *      edge, plus up": that one expression orients a hip plane, a shed plane, a
 *      dome band and -- the case that breaks a simpler rule -- a VERTICAL gable
 *      wall, whose normal has no upward component at all and for which "flip it
 *      if it points down" is a coin toss. add_face() takes the reference and
 *      flips the ring if it disagrees, so no builder below has to get a winding
 *      right by hand.
 *   2. **A gable is built by pushing its end edges out of reach.** See the
 *      header. The consequence here is that the enlarged ring may not be simple,
 *      which the skeleton refuses; the push distance is therefore tried at
 *      several sizes and, if none works, the gable ends are reported and the
 *      roof is hipped. Falling back visibly beats emitting a fold.
 *   3. **Heights come from a distance to a LINE, not from a vertex's skeleton
 *      time**, wherever a ring has been clipped. A clipped ring has vertices the
 *      skeleton never produced, so they have no time; but every point of one
 *      skeleton face lies on one plane, and that plane is fixed by the face's
 *      contour edge and the pitch. The unclipped hip uses SkeletonFace::times
 *      directly, because it is the same number and is already computed.
 *   4. **The removed footprint is only replaced by a floor when nothing else
 *      closed it.** Decided by a shared-edge test over quantised integer keys,
 *      not by an epsilon compare -- see the header's determinism note.
 *   5. **Where two faces meet, they meet at the same double.** Two places earn
 *      this. A gable wall takes its profile from the clipped slopes' own
 *      vertices rather than recomputing them, and it takes only the ones on its
 *      OWN edge, since a second contour edge may lie on the same line. A ridge
 *      welds every projection into one set of stations and snaps each slope's
 *      own ends to them, so one slope's last point and its neighbour's
 *      pass-through point are one number and not two within a tolerance.
 */

#include "procgen/rules/op_roof.hpp"

#include "geometry/straight_skeleton.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace stratum::procgen::rules {

namespace {

using stratum::geometry::compute_straight_skeleton;
using stratum::geometry::signed_ring_area;
using stratum::geometry::SkeletonFace;
using stratum::geometry::StraightSkeleton;
using stratum::geometry::vertex_offset_velocity;

// ============================================================================
// Tolerances
// ============================================================================

/**
 * @brief How nearly vertical a face's normal must be to count as a footprint
 *
 * About 1.15 degrees. Wide enough that a face built by extrude() -- whose normal
 * is exactly (0,1,0) -- always qualifies and that a face that has been through a
 * reframe() or two still does, narrow enough that the wall of a tapered mass does
 * not. A tilted face is REFUSED rather than roofed, because the roof rises along
 * local +y and a roof raised on a face that is not level does not meet it.
 */
constexpr double kUpCosine = 0.9998;

/// A footprint's vertices must be this close to one plane, relative to its size
constexpr double kFlatTolerance = 1e-9;

/// Below this the roof has no height and is not a roof
constexpr double kMinRise = 1e-9;

/// Relative tolerance for "this point lies on that line"
constexpr double kOnLineTolerance = 1e-9;

/// Points closer than this share a position, for the shared-edge test
constexpr double kEdgeQuantum = 1e-7;

/// cos(45 degrees): an edge more perpendicular than parallel to the ridge
constexpr double kPerpendicularCosine = 0.70710678118654752;

/// Smallest and largest dome band count. Two is the fewest that is not a cone.
constexpr int kMinDomeBands = 2;
constexpr int kMaxDomeBands = 64;

// ============================================================================
// Plan-space helpers
//
// A "plan" point is a glm::dvec2 holding the shape-local (x, z) of a point. The
// roof rises along local +y, so every outline question is two-dimensional and
// only the lift is not.
// ============================================================================

[[nodiscard]] double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

[[nodiscard]] glm::dvec2 plan_of(const glm::dvec3& p) { return glm::dvec2{p.x, p.z}; }

[[nodiscard]] glm::dvec3 raise(const glm::dvec2& p, double y) {
    return glm::dvec3{p.x, y, p.y};
}

/**
 * @brief Right of travel in the plan
 *
 * For a ring wound ANTICLOCKWISE in the (x, z) plot the interior is on the left,
 * so right of travel is outward. Every outward normal in this file goes through
 * here, and every ring that reaches here has been normalised to anticlockwise by
 * face_outline() or by the skeleton, so there is one convention and not two.
 */
[[nodiscard]] glm::dvec2 plan_right(const glm::dvec2& direction) {
    return glm::dvec2{direction.y, -direction.x};
}

[[nodiscard]] glm::dvec2 safe_unit(const glm::dvec2& v) {
    const double len = std::sqrt(v.x * v.x + v.y * v.y);
    if (!(len > 0.0)) {
        return glm::dvec2{0.0};
    }
    return v / len;
}

/// The outward direction of plan edge a->b, as a local 3D vector
[[nodiscard]] glm::dvec3 edge_outward(const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 right = safe_unit(plan_right(b - a));
    return glm::dvec3{right.x, 0.0, right.y};
}

/// Distance from @p p to the infinite line through @p a and @p b; 0 for a degenerate edge
[[nodiscard]] double distance_to_line(const glm::dvec2& p,
                                      const glm::dvec2& a,
                                      const glm::dvec2& b) {
    const glm::dvec2 direction = b - a;
    const double len = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (!(len > 0.0)) {
        return 0.0;
    }
    return std::fabs(cross2(direction / len, p - a));
}

/// Longest side of the ring's axis-aligned bounding box pair, as a diagonal length
[[nodiscard]] double ring_diameter(const std::vector<glm::dvec2>& ring) {
    if (ring.empty()) {
        return 0.0;
    }
    glm::dvec2 low = ring.front();
    glm::dvec2 high = ring.front();
    for (const glm::dvec2& p : ring) {
        low = glm::min(low, p);
        high = glm::max(high, p);
    }
    const glm::dvec2 span = high - low;
    return std::sqrt(span.x * span.x + span.y * span.y);
}

/**
 * @brief Where two infinite lines meet
 * @return false when they are parallel; @p out is untouched
 */
[[nodiscard]] bool line_intersection(const glm::dvec2& a0,
                                     const glm::dvec2& a1,
                                     const glm::dvec2& b0,
                                     const glm::dvec2& b1,
                                     glm::dvec2& out) {
    const glm::dvec2 da = a1 - a0;
    const glm::dvec2 db = b1 - b0;
    const double denominator = cross2(da, db);
    if (std::fabs(denominator) <= 1e-12) {
        return false;
    }
    const double t = cross2(b0 - a0, db) / denominator;
    out = a0 + da * t;
    return true;
}

/**
 * @brief Keep the part of @p ring on the inner side of the line through @p a with normal @p normal
 *
 * Sutherland-Hodgman against ONE half-plane. Exact arithmetic -- +, -, *, / --
 * so a clipped gable is as reproducible as an unclipped hip. A polygon boolean
 * would have been the general answer and is not needed: the enlarged footprint
 * differs from the real one by exactly the half-planes of the gable edges, which
 * is the precondition select_gable_ends() checks before a gable is attempted.
 *
 * @param tolerance Points within this of the line are treated as on it, so a
 *                  vertex that is already exactly on the cut is not duplicated
 */
[[nodiscard]] std::vector<glm::dvec2> clip_half_plane(const std::vector<glm::dvec2>& ring,
                                                      const glm::dvec2& a,
                                                      const glm::dvec2& normal,
                                                      double tolerance) {
    std::vector<glm::dvec2> out;
    if (ring.size() < 3) {
        return out;
    }
    out.reserve(ring.size() + 2);

    auto side = [&](const glm::dvec2& p) { return glm::dot(p - a, normal); };

    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& current = ring[i];
        const glm::dvec2& next = ring[(i + 1) % ring.size()];
        const double d_current = side(current);
        const double d_next = side(next);

        if (d_current <= tolerance) {
            out.push_back(current);
        }
        // A crossing, and only a real one: two points both within the tolerance
        // of the line would otherwise produce a "crossing" whose parameter is a
        // ratio of two rounding errors.
        if ((d_current > tolerance && d_next < -tolerance) ||
            (d_current < -tolerance && d_next > tolerance)) {
            const double t = d_current / (d_current - d_next);
            out.push_back(current + (next - current) * t);
        }
    }
    return out;
}

/// Signed area of a ring, doubled and halved the same way signed_ring_area() does
[[nodiscard]] double plan_area(const std::vector<glm::dvec2>& ring) {
    return signed_ring_area(ring);
}

/// Plan area of a ring of LIFTED points: the same shoelace, ignoring the height
///
/// Asked by the coverage checks, which are about ground and not about roof. A
/// roof face and its own shadow enclose the same number.
[[nodiscard]] double plan_area_of_lifted(const std::vector<glm::dvec3>& ring) {
    double twice = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec3& a = ring[i];
        const glm::dvec3& b = ring[(i + 1) % ring.size()];
        twice += a.x * b.z - b.x * a.z;
    }
    return 0.5 * twice;
}

/**
 * @brief Does the ground a roof covers disagree with the footprint's own area?
 *
 * One predicate, three callers, because "the faces cover the plan exactly once"
 * is the same invariant in all three: the straight skeleton's faces TILE the
 * polygon, a fan from a point that sees every edge tiles it, and a ridge's
 * slopes tile it only when the plan is a rectangle. Summing each face's own plan
 * area and comparing with the footprint's is what a roof that left ground bare
 * or covered it twice fails, and it is the ONLY assertion that fails for it: an
 * over-covering roof is still closed, its faces are still flat, and its volume
 * is still a plausible number.
 *
 * The tolerance is relative to the footprint, with a floor of one square unit so
 * that a small plan is not held to an absolute scale it cannot meet.
 */
[[nodiscard]] bool coverage_disagrees(double covered, double expected) {
    return std::fabs(covered - expected) > 1e-6 * std::max(1.0, expected);
}

/**
 * @brief Can @p point see every edge of @p ring -- is the ring star-shaped from it?
 *
 * The kernel of a simple polygon is the intersection of the inner half-planes of
 * its edges, so this loop is the whole test and not an approximation of it. When
 * it passes, the fan of triangles from @p point to the contour edges TILES the
 * polygon, every triangle wound the same way and none of them outside.
 *
 * `pyramid` and `dome` both need exactly that and neither asked for it. Sitting
 * on the skeleton's deepest node puts the apex INSIDE the plan, which is a
 * strictly weaker promise: on a U, a Z or a comb the apex is inside and still
 * cannot see the far arm, so the triangles raised to it cross the notch -- ground
 * that is not part of the footprint -- and overlap each other over the arm they
 * do reach. Nothing else in this file notices. The solid stays edge-manifold,
 * every face stays flat, and geometry_volume() returns a number that looks like
 * a roof. So the guard has to be this predicate and not a symptom of it.
 *
 * Preferred to comparing the emitted faces' plan area with the footprint's --
 * which is the check build_hip() makes and is mathematically the same question --
 * for two reasons: it answers BEFORE a face is emitted, so the roof can be built
 * a different way instead of unbuilt, and it is a sign test on one cross product
 * rather than a difference of two sums of areas.
 *
 * @param ring      An anticlockwise ring of at least three corners enclosing
 *                  area, which is what face_outline() and a complete skeleton
 *                  both guarantee; there is no guard here for a shorter one
 *                  because no input can reach it and a guard nothing can reach
 *                  is a line no test can hold to account
 * @param tolerance A point within this of an edge's line still counts as seeing
 *                  it. The triangle there has no area and add_face() drops it.
 */
[[nodiscard]] bool sees_every_edge(const std::vector<glm::dvec2>& ring,
                                   const glm::dvec2& point,
                                   double tolerance) {
    const size_t corners = ring.size();
    for (size_t i = 0; i < corners; ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % corners];
        // plan_right() is outward for an anticlockwise ring, and every ring that
        // reaches here has been normalised to anticlockwise. A point strictly
        // outward of any edge's line is outside that edge's half-plane, so the
        // polygon is not star-shaped from it.
        if (glm::dot(point - a, safe_unit(plan_right(b - a))) > tolerance) {
            return false;
        }
    }
    return true;
}

/// The longest edge's direction, which is the footprint's principal axis
///
/// Ties are broken by the LOWEST index so that the axis is a function of the
/// geometry and not of which edge happened to be visited first. A square has
/// four equal sides and gets the direction of edge 0, every time.
[[nodiscard]] glm::dvec2 principal_axis(const std::vector<glm::dvec2>& ring) {
    glm::dvec2 best{1.0, 0.0};
    double best_length = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2 edge = ring[(i + 1) % ring.size()] - ring[i];
        const double length = std::sqrt(edge.x * edge.x + edge.y * edge.y);
        if (length > best_length) {
            best_length = length;
            best = edge / length;
        }
    }
    return best;
}

/**
 * @brief Offset a ring in its own plane by a mitre
 *
 * Positive @p distance moves the outline OUTWARD. The per-vertex mitre comes
 * from straight_skeleton.hpp's vertex_offset_velocity(), which is exported for
 * exactly this reason: a second copy of the half-angle formula here would be
 * free to disagree with the skeleton's at the corner where it matters, and the
 * roof and its own footprint would then meet at slightly different places.
 *
 * Clipper2 was the alternative and is rejected for the same reason taper_shape()
 * rejects it: this needs a vertex-for-vertex correspondence with the input ring,
 * and a robust offset is precisely the operation that does not preserve the
 * vertex count.
 *
 * @param ring     Anticlockwise plan ring
 * @param distance Signed; positive grows the ring
 * @param out      Receives the offset ring, same vertex count
 * @param why      Receives the reason on failure
 * @return false when a vertex has no finite mitre, when an edge reverses, or
 *         when the ring collapses -- all of which mean the outline folded
 *         through itself, which is a knot and not a roof
 */
[[nodiscard]] bool offset_ring(const std::vector<glm::dvec2>& ring,
                               double distance,
                               std::vector<glm::dvec2>& out,
                               std::string& why) {
    out.clear();
    if (ring.size() < 3) {
        why = "the outline has fewer than three corners";
        return false;
    }

    out.reserve(ring.size());
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& previous = ring[(i + ring.size() - 1) % ring.size()];
        const glm::dvec2& vertex = ring[i];
        const glm::dvec2& next = ring[(i + 1) % ring.size()];
        glm::dvec2 velocity{0.0};
        if (!vertex_offset_velocity(previous, vertex, next, velocity)) {
            why = "corner " + std::to_string(i) + " of the outline is a spike with no mitre";
            return false;
        }
        // The velocity points INWARD for an anticlockwise ring, which this one
        // is, so growing the ring subtracts it.
        out.push_back(vertex - velocity * distance);
    }

    const double before = plan_area(ring);
    const double after = plan_area(out);
    if (!(after > 0.0) || !(before > 0.0)) {
        why = "an overhang of " + format_number(distance) + " leaves no outline to roof";
        return false;
    }
    for (size_t i = 0; i < ring.size(); ++i) {
        const size_t j = (i + 1) % ring.size();
        const glm::dvec2 was = ring[j] - ring[i];
        const glm::dvec2 now = out[j] - out[i];
        if (glm::dot(was, now) < 0.0) {
            why = "an overhang of " + format_number(distance) + " folds the outline at edge " +
                  std::to_string(i);
            return false;
        }
    }
    return true;
}

// ============================================================================
// Emitting faces
// ============================================================================

/// Newell normal of a ring of local points. (0,0,0) for a degenerate ring.
[[nodiscard]] glm::dvec3 ring_normal(const std::vector<glm::dvec3>& ring) {
    glm::dvec3 normal{0.0};
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec3& a = ring[i];
        const glm::dvec3& b = ring[(i + 1) % ring.size()];
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
    }
    const double length = std::sqrt(glm::dot(normal, normal));
    if (!(length > 0.0)) {
        return glm::dvec3{0.0};
    }
    return normal / length;
}

/**
 * @brief Do all of @p ring's points lie on one plane?
 *
 * Asked by build_ridge() and by nothing else. earcut triangulates a face in the
 * face's OWN plane, so a face whose corners are not on one produces triangles
 * that are warped rather than a face that is visibly wrong -- an artifact that
 * survives every area and volume assertion and shows up only as a crease in a
 * render.
 */
[[nodiscard]] bool ring_is_planar(const std::vector<glm::dvec3>& ring, double tolerance) {
    if (ring.size() < 4) {
        return true;  // three points are always on a plane
    }
    const glm::dvec3 normal = ring_normal(ring);
    if (glm::dot(normal, normal) <= 0.0) {
        return true;  // degenerate; add_face() drops it either way
    }
    const double offset = glm::dot(normal, ring.front());
    for (const glm::dvec3& p : ring) {
        if (std::fabs(glm::dot(normal, p) - offset) > tolerance) {
            return false;
        }
    }
    return true;
}

/// Drop consecutive repeats, including the wrap from last back to first
[[nodiscard]] std::vector<glm::dvec3> without_repeats(const std::vector<glm::dvec3>& ring,
                                                      double tolerance) {
    std::vector<glm::dvec3> out;
    out.reserve(ring.size());
    for (const glm::dvec3& p : ring) {
        if (!out.empty() && glm::all(glm::lessThan(glm::abs(p - out.back()),
                                                   glm::dvec3{tolerance}))) {
            continue;
        }
        out.push_back(p);
    }
    while (out.size() > 1 && glm::all(glm::lessThan(glm::abs(out.front() - out.back()),
                                                    glm::dvec3{tolerance}))) {
        out.pop_back();
    }
    return out;
}

/**
 * @brief Appends roof faces to a geometry, orienting each one against a reference
 *
 * Nothing below decides a winding. Each builder hands over a ring and the
 * direction the face should broadly look in -- outward from its contour edge
 * plus up, for a slope; straight outward, for a gable wall -- and this flips the
 * ring when Newell disagrees. One rule for six kinds, and the vertical faces,
 * where "point it up" is not a rule at all, are covered by the same line.
 */
class RoofBuilder {
public:
    RoofBuilder(ShapeGeometry& out, MaterialKey material, double tolerance)
        : out_(out), material_(material), tolerance_(tolerance) {}

    /// Number of faces appended so far
    [[nodiscard]] size_t faces() const { return faces_; }

    void add_face(const std::vector<glm::dvec3>& ring, const glm::dvec3& reference) {
        add_face_with_holes(ring, {}, reference);
    }

    void add_face_with_holes(const std::vector<glm::dvec3>& ring,
                             const std::vector<std::vector<glm::dvec3>>& holes,
                             const glm::dvec3& reference) {
        std::vector<glm::dvec3> loop = without_repeats(ring, tolerance_);
        if (loop.size() < 3) {
            return;  // a band that collapsed, or a gable wall with no height
        }
        const glm::dvec3 normal = ring_normal(loop);
        if (glm::dot(normal, normal) <= 0.0) {
            return;  // every point collinear: no surface, so nothing to emit
        }
        if (glm::dot(normal, reference) < 0.0) {
            std::reverse(loop.begin(), loop.end());
        }

        Face face;
        face.material = material_;
        face.loop = push_ring(loop);

        for (const std::vector<glm::dvec3>& hole : holes) {
            std::vector<glm::dvec3> inner = without_repeats(hole, tolerance_);
            if (inner.size() < 3) {
                continue;
            }
            // A hole is wound OPPOSITE to its loop, which is Face's contract.
            if (glm::dot(ring_normal(inner), reference) > 0.0) {
                std::reverse(inner.begin(), inner.end());
            }
            face.holes.push_back(push_ring(inner));
        }

        out_.faces.push_back(std::move(face));
        ++faces_;
    }

private:
    [[nodiscard]] std::vector<uint32_t> push_ring(const std::vector<glm::dvec3>& ring) {
        std::vector<uint32_t> indices;
        indices.reserve(ring.size());
        for (const glm::dvec3& p : ring) {
            indices.push_back(static_cast<uint32_t>(out_.positions.size()));
            out_.positions.push_back(p);
        }
        return indices;
    }

    ShapeGeometry& out_;
    MaterialKey material_;
    double tolerance_;
    size_t faces_ = 0;
};

// ============================================================================
// The footprint
// ============================================================================

/// One upward face, read as a plan outline plus the height it sits at
struct Outline {
    /// Outer ring, ANTICLOCKWISE in the (x, z) plot, first point not repeated
    std::vector<glm::dvec2> outer;

    /// Interior rings, wound the other way
    std::vector<std::vector<glm::dvec2>> holes;

    /// Local y the face lies at: the eave height
    double eave = 0.0;

    MaterialKey material{};

    /// Bounding-box diagonal of @p outer, the scale every relative tolerance uses
    double diameter = 0.0;
};

/**
 * @brief Read an upward face as a plan outline
 * @return false when the face is not flat enough to raise a roof on
 */
[[nodiscard]] bool face_outline(const ShapeGeometry& geometry,
                                const Face& face,
                                Outline& out,
                                std::string& why) {
    out = Outline{};
    out.material = face.material;
    if (face.loop.size() < 3) {
        why = "the upward face has fewer than three corners";
        return false;
    }

    out.eave = geometry.positions[face.loop.front()].y;
    double spread = 0.0;
    auto read_ring = [&](const std::vector<uint32_t>& indices, std::vector<glm::dvec2>& ring) {
        ring.clear();
        ring.reserve(indices.size());
        for (const uint32_t index : indices) {
            const glm::dvec3& p = geometry.positions[index];
            spread = std::max(spread, std::fabs(p.y - out.eave));
            ring.push_back(plan_of(p));
        }
    };

    read_ring(face.loop, out.outer);
    for (const std::vector<uint32_t>& hole : face.holes) {
        if (hole.size() < 3) {
            continue;
        }
        std::vector<glm::dvec2> ring;
        read_ring(hole, ring);
        out.holes.push_back(std::move(ring));
    }

    out.diameter = ring_diameter(out.outer);
    if (spread > kFlatTolerance * std::max(1.0, out.diameter)) {
        // A face that passed the normal test can still be non-planar. The roof
        // is raised from ONE eave height, so a face that is not level would put
        // the roof and the wall top in different places along part of the
        // boundary -- a gap in the building, several operations from here.
        why = "the upward face is not level: its corners span " + format_number(spread) +
              " in height";
        return false;
    }

    // The winding is FIXED, not trusted, exactly as shape_from_rings() fixes it:
    // everything below reads "outward" as right-of-travel, which is only outward
    // for an anticlockwise ring.
    if (plan_area(out.outer) < 0.0) {
        std::reverse(out.outer.begin(), out.outer.end());
    }
    for (std::vector<glm::dvec2>& hole : out.holes) {
        if (plan_area(hole) > 0.0) {
            std::reverse(hole.begin(), hole.end());
        }
    }

    if (!(plan_area(out.outer) > 0.0) || !(out.diameter > 0.0)) {
        why = "the upward face encloses no area";
        return false;
    }
    return true;
}

// ============================================================================
// Skeleton access
// ============================================================================

/// What went wrong, in the skeleton's own numbers
[[nodiscard]] std::string skeleton_refusal(const StraightSkeleton& skeleton) {
    if (skeleton.stats.self_intersections > 0) {
        return "the footprint crosses itself at " +
               std::to_string(skeleton.stats.self_intersections) + " places, so no roof fits it";
    }
    if (skeleton.contour.size() < 3) {
        return "the footprint has fewer than three distinct corners";
    }
    return "the roof solver did not finish over this footprint after " +
           std::to_string(skeleton.stats.iterations) + " steps";
}

/// The deepest point of the footprint: the skeleton node furthest from every edge
///
/// Ties by the lowest node index, so the apex of a square pyramid is a function
/// of the footprint and not of the order the wavefront happened to resolve in.
[[nodiscard]] bool deepest_node(const StraightSkeleton& skeleton,
                                glm::dvec2& position,
                                double& time) {
    bool found = false;
    for (const auto& node : skeleton.nodes) {
        if (!found || node.time > time) {
            found = true;
            position = node.position;
            time = node.time;
        }
    }
    return found;
}

// ============================================================================
// hip
// ============================================================================

/**
 * @brief Lift an already-solved skeleton into one sloped plane per contour edge
 *
 * Every skeleton face is one roof plane and every skeleton node rises by its own
 * time, which is its distance to the nearest contour edge. SkeletonFace::times
 * is carried for precisely this consumer, so no vertex has to be matched back to
 * a node by position.
 *
 * Split out of build_hip() because it has two more callers: `pyramid` and `dome`
 * fall back to a hip over a plan they cannot raise an apex on, and both have
 * ALREADY solved the identical skeleton. Calling build_hip() from there would
 * solve the same polygon twice and overwrite report.skeleton with the same
 * numbers reached by a second route -- which is the sort of thing that reads as
 * a discrepancy the first time a stat disagrees.
 */
[[nodiscard]] OpResult lift_skeleton(const StraightSkeleton& skeleton,
                                     const Outline& outline,
                                     double pitch_tangent,
                                     RoofBuilder& builder,
                                     RoofReport& report) {
    const double rise = skeleton.stats.max_time * pitch_tangent;
    if (!(rise > kMinRise)) {
        return OpResult::failure("the footprint is too small for this pitch: the ridge would "
                                 "rise " + format_number(rise));
    }
    report.rise = std::max(report.rise, rise);

    const size_t corners = skeleton.contour.size();
    double covered = 0.0;
    for (const SkeletonFace& face : skeleton.faces) {
        if (face.ring.size() < 3 || face.edge >= corners) {
            continue;
        }
        covered += face.area;

        std::vector<glm::dvec3> ring;
        ring.reserve(face.ring.size());
        for (size_t i = 0; i < face.ring.size(); ++i) {
            const double time = i < face.times.size() ? face.times[i] : 0.0;
            ring.push_back(raise(face.ring[i], outline.eave + time * pitch_tangent));
        }

        const glm::dvec2& a = skeleton.contour[face.edge];
        const glm::dvec2& b = skeleton.contour[(face.edge + 1) % corners];
        builder.add_face(ring, edge_outward(a, b) + glm::dvec3{0.0, 1.0, 0.0});
    }

    // The faces of a straight skeleton TILE the polygon. That is the invariant a
    // missing split event breaks, and checking it here means a roof with a hole
    // in it says so rather than being found in a viewport.
    const double expected = plan_area(skeleton.contour);
    if (coverage_disagrees(covered, expected)) {
        report.notes.push_back("the roof planes cover " + format_number(covered) +
                               " of the footprint's " + format_number(expected));
    }
    return OpResult::success();
}

/**
 * @brief The straight skeleton of the footprint, solved and lifted
 */
[[nodiscard]] OpResult build_hip(const Outline& outline,
                                 double pitch_tangent,
                                 RoofBuilder& builder,
                                 RoofReport& report) {
    const StraightSkeleton skeleton = compute_straight_skeleton(outline.outer);
    report.used_skeleton = true;
    report.skeleton = skeleton.stats;
    if (!skeleton.complete) {
        return OpResult::failure(skeleton_refusal(skeleton));
    }
    return lift_skeleton(skeleton, outline, pitch_tangent, builder, report);
}

// ============================================================================
// gable
// ============================================================================

/// One contour edge that is to become a vertical wall instead of a slope
struct GableEnd {
    uint32_t edge = 0;         ///< Index into the outline's outer ring
    glm::dvec2 a{0.0};         ///< Start of the edge
    glm::dvec2 b{0.0};         ///< End of the edge
    glm::dvec2 direction{0.0}; ///< Unit a -> b
    glm::dvec2 normal{0.0};    ///< Unit outward normal in plan
    double length = 0.0;

    /// (distance along the edge, height above the eave) sampled from the clip
    std::vector<glm::dvec2> profile;
};

/**
 * @brief Pick the two contour edges that become gable walls
 *
 * The ends of the footprint along the ridge: among the edges that are more
 * perpendicular to the ridge than parallel to it, the one furthest back and the
 * one furthest forward. For a rectangle that is exactly its two short sides.
 *
 * Two preconditions are checked rather than assumed, and a candidate that fails
 * either is reported and left hipped:
 *
 *   - **The whole footprint must lie on one side of the edge's line.** The gable
 *     is built by clipping with that line (see the header), and a line that cuts
 *     through the footprint would clip away roof that belongs there.
 *   - **Two gable ends must not be adjacent.** Pushing an edge outward moves its
 *     two neighbours' endpoints; two ends sharing a corner would each move the
 *     other's, and neither result is the one that was asked for.
 */
[[nodiscard]] std::vector<GableEnd> select_gable_ends(const Outline& outline,
                                                      const glm::dvec2& axis,
                                                      std::vector<std::string>& notes) {
    const std::vector<glm::dvec2>& ring = outline.outer;
    const size_t corners = ring.size();
    const double tolerance = kOnLineTolerance * std::max(1.0, outline.diameter);

    size_t first = corners;
    size_t last = corners;
    double lowest = 0.0;
    double highest = 0.0;
    for (size_t i = 0; i < corners; ++i) {
        const glm::dvec2 edge = ring[(i + 1) % corners] - ring[i];
        const glm::dvec2 direction = safe_unit(edge);
        if (std::fabs(glm::dot(direction, axis)) >= kPerpendicularCosine) {
            continue;  // runs along the ridge, so it is a side and not an end
        }
        const double along = glm::dot((ring[i] + ring[(i + 1) % corners]) * 0.5, axis);
        if (first == corners || along < lowest) {
            first = i;
            lowest = along;
        }
        if (last == corners || along > highest) {
            last = i;
            highest = along;
        }
    }

    std::vector<size_t> candidates;
    if (first != corners) {
        candidates.push_back(first);
    }
    if (last != corners && last != first) {
        candidates.push_back(last);
    }

    std::vector<GableEnd> ends;
    for (const size_t i : candidates) {
        GableEnd end;
        end.edge = static_cast<uint32_t>(i);
        end.a = ring[i];
        end.b = ring[(i + 1) % corners];
        end.direction = safe_unit(end.b - end.a);
        end.normal = plan_right(end.direction);
        end.length = std::sqrt(glm::dot(end.b - end.a, end.b - end.a));
        if (!(end.length > tolerance)) {
            continue;
        }

        bool one_sided = true;
        for (const glm::dvec2& p : ring) {
            if (glm::dot(p - end.a, end.normal) > tolerance) {
                one_sided = false;
                break;
            }
        }
        if (!one_sided) {
            notes.push_back("the gable end at edge " + std::to_string(i) +
                            " was hipped instead: the footprint lies on both sides of it");
            continue;
        }
        ends.push_back(end);
    }

    if (ends.size() == 2) {
        const size_t i = ends[0].edge;
        const size_t j = ends[1].edge;
        if ((i + 1) % corners == j || (j + 1) % corners == i) {
            notes.push_back("the gable end at edge " + std::to_string(j) +
                            " was hipped instead: it shares a corner with the other end");
            ends.pop_back();
        }
    }
    return ends;
}

/**
 * @brief The footprint with each gable end pushed out by @p distance
 *
 * The end's two corners are replaced by where its neighbours' lines meet the
 * moved line. The neighbours' SUPPORTING LINES are therefore untouched, which is
 * the whole reason the trick works: every plane but the gable end's is the plane
 * it always was, so the envelope over the real footprint is unchanged.
 *
 * @return false when a neighbour is parallel to the end, or when the moved
 *         corner did not actually move outward -- either of which means the
 *         enlarged ring is not the shape this is supposed to produce
 */
[[nodiscard]] bool push_gable_ends(const std::vector<glm::dvec2>& ring,
                                   const std::vector<GableEnd>& ends,
                                   double distance,
                                   std::vector<glm::dvec2>& out) {
    const size_t corners = ring.size();
    out = ring;
    for (const GableEnd& end : ends) {
        const size_t i = end.edge;
        const glm::dvec2 moved = end.a + end.normal * distance;
        const glm::dvec2 moved_to = moved + end.direction;

        glm::dvec2 start{0.0};
        glm::dvec2 finish{0.0};
        if (!line_intersection(ring[(i + corners - 1) % corners], ring[i], moved, moved_to,
                               start) ||
            !line_intersection(ring[(i + 2) % corners], ring[(i + 1) % corners], moved, moved_to,
                               finish)) {
            return false;
        }
        if (glm::dot(start - end.a, end.normal) <= 0.0 ||
            glm::dot(finish - end.b, end.normal) <= 0.0) {
            return false;
        }
        out[i] = start;
        out[(i + 1) % corners] = finish;
    }
    return true;
}

/**
 * @brief The skeleton roof with the gable ends' planes removed and vertical walls in their place
 *
 * See the header for why the removal is a push rather than a weighted skeleton.
 * The push distance is tried large first and halved on refusal, because a large
 * push can make the enlarged ring self-intersect on a footprint whose sides
 * converge; anything above the footprint's own diameter is far enough for the
 * gable plane to lose everywhere, so the smallest attempt is still exact. When
 * no distance works the roof is HIPPED and the fallback is reported, because a
 * partly-hipped roof is a visible answer and a fold is not.
 */
[[nodiscard]] OpResult build_gable(const Outline& outline,
                                   double pitch_tangent,
                                   const glm::dvec2& axis,
                                   RoofBuilder& builder,
                                   RoofReport& report) {
    std::vector<GableEnd> ends = select_gable_ends(outline, axis, report.notes);
    if (ends.empty()) {
        report.notes.push_back("no edge of the footprint could be a gable end, so the roof was "
                               "hipped");
        return build_hip(outline, pitch_tangent, builder, report);
    }

    StraightSkeleton skeleton;
    std::vector<glm::dvec2> enlarged;
    bool solved = false;
    for (const double scale : {4.0, 2.0, 1.01}) {
        std::vector<glm::dvec2> candidate;
        if (!push_gable_ends(outline.outer, ends, outline.diameter * scale, candidate)) {
            continue;
        }
        StraightSkeleton attempt = compute_straight_skeleton(candidate);
        if (attempt.complete) {
            skeleton = std::move(attempt);
            enlarged = std::move(candidate);
            solved = true;
            break;
        }
        skeleton = std::move(attempt);
    }

    report.used_skeleton = true;
    report.skeleton = skeleton.stats;
    if (!solved) {
        report.notes.push_back("the gable ends could not be pushed clear of the footprint, so "
                               "the roof was hipped");
        if (!skeleton.contour.empty()) {
            report.notes.push_back(skeleton_refusal(skeleton));
        }
        return build_hip(outline, pitch_tangent, builder, report);
    }

    const double tolerance = kOnLineTolerance * std::max(1.0, outline.diameter);
    const size_t corners = skeleton.contour.size();
    const glm::dvec3 up{0.0, 1.0, 0.0};
    double rise = 0.0;

    for (const SkeletonFace& face : skeleton.faces) {
        if (face.ring.size() < 3 || face.edge >= corners) {
            continue;
        }
        const glm::dvec2& a = skeleton.contour[face.edge];
        const glm::dvec2& b = skeleton.contour[(face.edge + 1) % corners];

        // The clip is what turns the enlarged roof back into the real one. A
        // face of a pushed edge lies entirely beyond that edge's own line, so it
        // clips away to nothing and needs no special case.
        std::vector<glm::dvec2> clipped = face.ring;
        for (const GableEnd& end : ends) {
            clipped = clip_half_plane(clipped, end.a, end.normal, tolerance);
            if (clipped.size() < 3) {
                break;
            }
        }
        if (clipped.size() < 3) {
            continue;
        }

        // Heights from the distance to the face's own edge LINE, not from a
        // skeleton time: the clip made vertices the skeleton never saw.
        std::vector<glm::dvec3> ring;
        ring.reserve(clipped.size());
        for (const glm::dvec2& p : clipped) {
            const double height = distance_to_line(p, a, b) * pitch_tangent;
            rise = std::max(rise, height);
            ring.push_back(raise(p, outline.eave + height));

            // A vertex sitting on a gable line is a point of that gable's roof
            // profile. Collecting them here rather than re-deriving the profile
            // means the wall meets the slopes at exactly the vertices the slopes
            // were built from, and not at a recomputed point half an ulp away.
            for (GableEnd& end : ends) {
                if (std::fabs(glm::dot(p - end.a, end.normal)) > tolerance) {
                    continue;  // not on this end's line at all
                }
                // ON THE LINE IS NOT ON THE EDGE. A footprint may have a second
                // contour edge collinear with the gable end -- the two prongs of
                // a comb or a C share one line -- and that edge's own slope has
                // vertices on it which belong to no gable wall. Taking them
                // would build the wall out past its own corners and down the
                // far prong. Clamping them back to the corner, which is what
                // this used to do, only exchanged one wrong wall for another;
                // the point does not belong to this end and is dropped.
                const double along = glm::dot(p - end.a, end.direction);
                if (along < -tolerance || along > end.length + tolerance) {
                    continue;
                }
                end.profile.emplace_back(along, height);
            }
        }
        builder.add_face(ring, edge_outward(a, b) + up);
    }

    if (!(rise > kMinRise)) {
        return OpResult::failure("the footprint is too small for this pitch: the ridge would "
                                 "rise " + format_number(rise));
    }
    report.rise = std::max(report.rise, rise);

    for (GableEnd& end : ends) {
        std::sort(end.profile.begin(), end.profile.end(),
                  [](const glm::dvec2& lhs, const glm::dvec2& rhs) {
                      if (lhs.x != rhs.x) {
                          return lhs.x < rhs.x;
                      }
                      return lhs.y < rhs.y;
                  });

        // The wall: along the eave from one corner to the other, then back over
        // the roof profile. A gable end is a VERTICAL polygon -- osm's E1 got
        // this wrong first time by sloping it -- so every profile point is
        // directly above the edge and the whole ring lies in one vertical plane.
        std::vector<glm::dvec3> wall;
        wall.push_back(raise(end.a, outline.eave));
        wall.push_back(raise(end.b, outline.eave));
        for (size_t i = end.profile.size(); i-- > 0;) {
            wall.push_back(raise(end.a + end.direction * end.profile[i].x,
                                 outline.eave + end.profile[i].y));
        }

        const size_t before = builder.faces();
        builder.add_face(wall, glm::dvec3{end.normal.x, 0.0, end.normal.y});
        if (builder.faces() > before) {
            ++report.gable_walls;
        }
    }
    return OpResult::success();
}

// ============================================================================
// pyramid
// ============================================================================

/// Every contour edge rises to one apex over the footprint's deepest point
///
/// Over a plan that is not star-shaped from that point there is no such solid,
/// and this raises the hip instead and reports it. See sees_every_edge().
[[nodiscard]] OpResult build_pyramid(const Outline& outline,
                                     double pitch_tangent,
                                     RoofBuilder& builder,
                                     RoofReport& report) {
    const StraightSkeleton skeleton = compute_straight_skeleton(outline.outer);
    report.used_skeleton = true;
    report.skeleton = skeleton.stats;
    if (!skeleton.complete) {
        return OpResult::failure(skeleton_refusal(skeleton));
    }

    glm::dvec2 centre{0.0};
    double depth = 0.0;
    if (!deepest_node(skeleton, centre, depth)) {
        return OpResult::failure("the footprint has no interior to raise an apex over");
    }

    // The apex follows the INRADIUS and not the bounding box, which is the
    // answer osm/mesh_builder.cpp settled on in E1: a long thin building gets a
    // shallow pyramid instead of a spike. The skeleton's deepest node is that
    // inradius, and unlike a centroid it is always inside the footprint.
    const double rise = depth * pitch_tangent;
    if (!(rise > kMinRise)) {
        return OpResult::failure("the footprint is too small for this pitch: the apex would "
                                 "rise " + format_number(rise));
    }

    // Inside the plan is not enough. One apex only works when the apex SEES
    // every contour edge, and the deepest node of a U, a Z or a comb does not:
    // the triangle raised from the far arm's edge crosses the notch, which is
    // not the footprint, and overlaps the triangles beside it. That roof is
    // still closed and still flat face by face, so it would have reached a
    // viewport before it reached a test. A plan the apex cannot see all of gets
    // the hip instead -- the same lower envelope, exact everywhere -- and is
    // TOLD, which is what build_gable() does with a gable end it cannot push.
    if (!sees_every_edge(skeleton.contour, centre,
                         kOnLineTolerance * std::max(1.0, outline.diameter))) {
        report.notes.push_back("a pyramid needs a footprint its apex can see the whole of, and "
                               "this one is not star-shaped from its deepest point, so it was "
                               "raised as a hip instead");
        return lift_skeleton(skeleton, outline, pitch_tangent, builder, report);
    }

    report.rise = std::max(report.rise, rise);

    const glm::dvec3 apex = raise(centre, outline.eave + rise);
    const glm::dvec3 up{0.0, 1.0, 0.0};
    const size_t corners = skeleton.contour.size();
    for (size_t i = 0; i < corners; ++i) {
        const glm::dvec2& a = skeleton.contour[i];
        const glm::dvec2& b = skeleton.contour[(i + 1) % corners];
        const std::vector<glm::dvec3> ring = {raise(a, outline.eave), raise(b, outline.eave),
                                              apex};
        builder.add_face(ring, edge_outward(a, b) + up);
    }
    return OpResult::success();
}

// ============================================================================
// ridge
// ============================================================================

/**
 * @brief Every contour edge rises to its projection on a ridge segment
 *
 * The cheap kind: no skeleton, so it answers for a footprint the skeleton
 * refuses, and it is the construction osm/mesh_builder.cpp uses for
 * `roof:shape=gabled` so that an imported terrace and a generated one match.
 *
 * It is a PROJECTION and not a lower envelope. On a rectangle that is the same
 * thing and the tests assert it agrees with `gable` vertex for vertex; on any
 * other plan it over-covers, because an edge reaches the ridge across ground
 * that belongs to another edge. Guarding against that would mean computing the
 * envelope, which is the skeleton, which is `gable`. What IS guarded against is
 * the artifact that would otherwise be invisible: a projected quad whose corners
 * are not on one plane.
 */
[[nodiscard]] OpResult build_ridge(const Outline& outline,
                                   double pitch_tangent,
                                   const glm::dvec2& axis,
                                   RoofBuilder& builder,
                                   RoofReport& report) {
    const glm::dvec2 across = plan_right(axis);
    double low_along = std::numeric_limits<double>::max();
    double high_along = std::numeric_limits<double>::lowest();
    double low_across = std::numeric_limits<double>::max();
    double high_across = std::numeric_limits<double>::lowest();
    for (const glm::dvec2& p : outline.outer) {
        low_along = std::min(low_along, glm::dot(p, axis));
        high_along = std::max(high_along, glm::dot(p, axis));
        low_across = std::min(low_across, glm::dot(p, across));
        high_across = std::max(high_across, glm::dot(p, across));
    }

    const double width = high_across - low_across;
    const double rise = width * 0.5 * pitch_tangent;
    if (!(rise > kMinRise)) {
        return OpResult::failure("the footprint is too small for this pitch: the ridge would "
                                 "rise " + format_number(rise));
    }
    report.rise = std::max(report.rise, rise);

    // The ridge runs along the middle of the footprint's ACROSS extent, not
    // through its centroid: a footprint with more corners at one end has a
    // centroid off the middle, and a ridge that is off the middle gives the two
    // slopes different pitches, which is not what one pitch argument means.
    const glm::dvec2 base = across * ((low_across + high_across) * 0.5);
    const glm::dvec2 ridge_start = base + axis * low_along;
    const double span = high_along - low_along;
    const double ridge_y = outline.eave + rise;
    const glm::dvec3 up{0.0, 1.0, 0.0};
    const double tolerance = kOnLineTolerance * std::max(1.0, outline.diameter);

    auto ridge_at = [&](double t) { return ridge_start + axis * t; };

    // Every contour vertex's projection is a vertex of the RIDGE LINE, not only
    // of the edge that produced it.
    //
    // Projecting each edge's two ends and joining them straight across was the
    // first version, and it left T-JUNCTIONS on every plan that is not a
    // rectangle: the two sides of the ridge project to different sets of points,
    // so one slope's top edge ran the whole way past the end of the slope facing
    // it, and the solid stopped being edge-manifold. That is invisible to
    // is_watertight() -- a T-junction is geometrically closed -- and it cracks
    // open at the first vertex weld, displacement or LOD step, while the AO
    // baker and the chunked export both want the adjacency.
    //
    // So the projections are collected once, welded, sorted, and each slope is
    // emitted THROUGH the ones that fall inside its own span. Every ridge
    // segment is then shared by exactly two slopes, edge for edge.
    const size_t corners = outline.outer.size();
    std::vector<double> parameters;
    parameters.reserve(corners);
    for (const glm::dvec2& p : outline.outer) {
        parameters.push_back(std::min(std::max(glm::dot(p - ridge_start, axis), 0.0), span));
    }

    // Welded into one canonical set of STATIONS, and each edge's own endpoints
    // replaced by the station it welded to. Two corners whose projections differ
    // by less than the footprint's own tolerance have to become ONE station:
    // leave them apart and the ridge gains a sliver segment that one slope keeps
    // and the slope facing it welds away in without_repeats(), which is the
    // T-junction again at a smaller scale. Snapping the endpoints too is what
    // makes the point one slope ends at and the point its neighbour passes
    // through the same double rather than two within a tolerance of each other.
    //
    // A rectangle welds nothing -- its four corners project to two values
    // exactly -- so the exact case stays bit for bit what it was, which is what
    // gable_and_ridge_agree_vertex_for_vertex_on_a_rectangle asserts. The
    // representative of each cluster is the first projection in ring order, so
    // the answer is a function of the footprint and not of a sort's tie-break.
    std::vector<double> stations;
    stations.reserve(corners);
    for (double& t : parameters) {
        bool welded = false;
        for (const double station : stations) {
            if (std::fabs(station - t) <= tolerance) {
                t = station;
                welded = true;
                break;
            }
        }
        if (!welded) {
            stations.push_back(t);
        }
    }
    std::sort(stations.begin(), stations.end());

    double covered = 0.0;
    for (size_t i = 0; i < corners; ++i) {
        const glm::dvec2& a = outline.outer[i];
        const glm::dvec2& b = outline.outer[(i + 1) % corners];
        const double from = parameters[i];
        const double to = parameters[(i + 1) % corners];

        // Along the eave from a to b, up at b's station, back along the ridge
        // through every station between, down at a's. An edge square to the
        // ridge has from == to and the two ridge points collapse into one, which
        // add_face() reduces to the triangle it is.
        std::vector<glm::dvec3> ring;
        ring.reserve(stations.size() + 3);
        ring.push_back(raise(a, outline.eave));
        ring.push_back(raise(b, outline.eave));
        ring.push_back(raise(ridge_at(to), ridge_y));
        if (to > from) {
            for (size_t k = stations.size(); k-- > 0;) {
                if (stations[k] > from && stations[k] < to) {
                    ring.push_back(raise(ridge_at(stations[k]), ridge_y));
                }
            }
        } else {
            for (const double station : stations) {
                if (station > to && station < from) {
                    ring.push_back(raise(ridge_at(station), ridge_y));
                }
            }
        }
        ring.push_back(raise(ridge_at(from), ridge_y));

        const glm::dvec3 reference = edge_outward(a, b) + up;

        // Ground is counted per EMITTED face and not per edge, because a slope
        // whose plan outline crosses itself -- which is what over-covering looks
        // like from above -- has a shoelace area in which the two lobes cancel.
        // The triangles the fan splits it into do not cancel.
        auto emit = [&](const std::vector<glm::dvec3>& piece) {
            covered += std::fabs(plan_area_of_lifted(piece));
            builder.add_face(piece, reference);
        };

        // An edge that is neither parallel nor perpendicular to the ridge has
        // its two ends projected to two DIFFERENT ridge points, and its corners
        // are then not on one plane. Fanning it from the eave corner is what
        // keeps every emitted face flat; the fan's diagonals are each shared by
        // the two triangles beside them, so the face still closes. The ring is
        // kept whole where it is flat, which is every edge of a rectangular
        // plan, so that the exact case stays exactly what `gable` produces.
        if (ring_is_planar(ring, kFlatTolerance * std::max(1.0, outline.diameter))) {
            emit(ring);
        } else {
            for (size_t k = 1; k + 1 < ring.size(); ++k) {
                emit({ring[0], ring[k], ring[k + 1]});
            }
        }
    }

    // A ridge is a projection and not a lower envelope, so off a rectangle its
    // slopes reach the ridge across ground that belongs to another slope. That
    // is documented in the header and is not a bug -- but an author who picked
    // `ridge` by accident should be told here rather than find it in a viewport,
    // and be told which kind is right instead.
    const double expected = plan_area(outline.outer);
    if (coverage_disagrees(covered, expected)) {
        report.notes.push_back("a ridge over this footprint covers " + format_number(covered) +
                               " of ground where the footprint is " + format_number(expected) +
                               ": a ridge is exact only over a rectangle, and 'gable' is the "
                               "kind that fits this plan");
    }
    return OpResult::success();
}

// ============================================================================
// shed
// ============================================================================

/**
 * @brief One tilted plane, with a vertical skirt down to the eave
 *
 * The only kind that carries interior rings, and it carries them exactly: the
 * lift is affine in the plan position, so a ring that was flat stays flat and
 * the tilted face is still planar with its holes in it. Each hole gets its own
 * skirt, wound by the same expression as the outer one -- a hole ring is wound
 * the other way, so right-of-travel points INTO the hole, which is outward for
 * the solid. One formula, both cases, as build_prism() puts it.
 */
[[nodiscard]] OpResult build_shed(const Outline& outline,
                                  double pitch_tangent,
                                  const glm::dvec2& downhill,
                                  RoofBuilder& builder,
                                  RoofReport& report) {
    const glm::dvec2 uphill = -downhill;
    double lowest = std::numeric_limits<double>::max();
    double highest = std::numeric_limits<double>::lowest();
    for (const glm::dvec2& p : outline.outer) {
        lowest = std::min(lowest, glm::dot(p, uphill));
        highest = std::max(highest, glm::dot(p, uphill));
    }

    const double rise = (highest - lowest) * pitch_tangent;
    if (!(rise > kMinRise)) {
        return OpResult::failure("the footprint is too small for this pitch: the high side would "
                                 "rise " + format_number(rise));
    }
    report.rise = std::max(report.rise, rise);

    auto lift = [&](const glm::dvec2& p) {
        return outline.eave + (glm::dot(p, uphill) - lowest) * pitch_tangent;
    };

    std::vector<glm::dvec3> top;
    top.reserve(outline.outer.size());
    for (const glm::dvec2& p : outline.outer) {
        top.push_back(raise(p, lift(p)));
    }
    std::vector<std::vector<glm::dvec3>> top_holes;
    for (const std::vector<glm::dvec2>& hole : outline.holes) {
        std::vector<glm::dvec3> ring;
        ring.reserve(hole.size());
        for (const glm::dvec2& p : hole) {
            ring.push_back(raise(p, lift(p)));
        }
        top_holes.push_back(std::move(ring));
        ++report.holes_carried;
    }
    builder.add_face_with_holes(top, top_holes, glm::dvec3{0.0, 1.0, 0.0});

    // The skirt between the level eave and the tilted plane. Degenerate along
    // the low edge, where the plane already meets the wall, and add_face() drops
    // it there rather than emitting a zero-area quad.
    auto skirt = [&](const std::vector<glm::dvec2>& ring) {
        for (size_t i = 0; i < ring.size(); ++i) {
            const glm::dvec2& a = ring[i];
            const glm::dvec2& b = ring[(i + 1) % ring.size()];
            const std::vector<glm::dvec3> quad = {raise(a, outline.eave), raise(b, outline.eave),
                                                  raise(b, lift(b)), raise(a, lift(a))};
            builder.add_face(quad, edge_outward(a, b));
        }
    };
    skirt(outline.outer);
    for (const std::vector<glm::dvec2>& hole : outline.holes) {
        skirt(hole);
    }
    return OpResult::success();
}

// ============================================================================
// dome
// ============================================================================

/**
 * @brief The outline shrunk toward the deepest point over a quarter-sine profile
 *
 * A true dome over an arbitrary polygon is not defined, so each latitude band is
 * the footprint itself scaled about its deepest point -- a circular footprint
 * gives a real hemisphere and anything else gives a swept version of its own
 * outline. That is the answer osm/mesh_builder.cpp reached for roof:shape=dome
 * and there is no second one worth having.
 *
 * A band is the outline SCALED about the deepest point, so this needs the same
 * star-shapedness `pyramid` does and for the same reason: a scaled outline only
 * stays inside the plan when every point of the plan can be seen from the
 * centre. A plan that fails that gets the hip and a note.
 *
 * This is the ONE kind whose coordinates go through libm more than once. The
 * band profile needs a sine and a cosine at angles that are not quarter turns,
 * so a dome is not bit-identical across C libraries. See the header.
 */
[[nodiscard]] OpResult build_dome(const Outline& outline,
                                  double pitch_tangent,
                                  int bands,
                                  RoofBuilder& builder,
                                  RoofReport& report) {
    const StraightSkeleton skeleton = compute_straight_skeleton(outline.outer);
    report.used_skeleton = true;
    report.skeleton = skeleton.stats;
    if (!skeleton.complete) {
        return OpResult::failure(skeleton_refusal(skeleton));
    }

    glm::dvec2 centre{0.0};
    double depth = 0.0;
    if (!deepest_node(skeleton, centre, depth)) {
        return OpResult::failure("the footprint has no interior to raise a dome over");
    }
    const double rise = depth * pitch_tangent;
    if (!(rise > kMinRise)) {
        return OpResult::failure("the footprint is too small for this pitch: the dome would "
                                 "rise " + format_number(rise));
    }

    // The same limit as `pyramid`, for the same reason: a band is the outline
    // scaled about the centre, and a scaled outline only stays inside the plan
    // when the plan is star-shaped from that centre. Over a U the inner bands
    // cross the notch exactly as the pyramid's triangles do -- this is a dome
    // with one band per latitude, not a dome with a different defect. Hipped and
    // reported rather than silently over-covering.
    if (!sees_every_edge(skeleton.contour, centre,
                         kOnLineTolerance * std::max(1.0, outline.diameter))) {
        report.notes.push_back("a dome needs a footprint its centre can see the whole of, and "
                               "this one is not star-shaped from its deepest point, so it was "
                               "raised as a hip instead");
        return lift_skeleton(skeleton, outline, pitch_tangent, builder, report);
    }

    report.rise = std::max(report.rise, rise);

    const std::vector<glm::dvec2>& ring = skeleton.contour;
    const size_t corners = ring.size();
    const glm::dvec3 up{0.0, 1.0, 0.0};

    auto band = [&](int k, size_t i) {
        double sine = 0.0;
        double cosine = 0.0;
        deg_sin_cos(90.0 * static_cast<double>(k) / static_cast<double>(bands), sine, cosine);
        return raise(centre + (ring[i] - centre) * cosine, outline.eave + rise * sine);
    };

    for (int k = 0; k < bands; ++k) {
        for (size_t i = 0; i < corners; ++i) {
            const size_t j = (i + 1) % corners;
            // The top band's outer pair collapses onto the centre, so this quad
            // becomes a triangle; add_face() drops the repeat rather than
            // emitting a degenerate corner.
            const std::vector<glm::dvec3> quad = {band(k, i), band(k, j), band(k + 1, j),
                                                  band(k + 1, i)};
            builder.add_face(quad, edge_outward(ring[i], ring[j]) + up);
        }
    }
    return OpResult::success();
}

// ============================================================================
// Footprint bookkeeping
// ============================================================================

/// An undirected edge between two quantised local positions
using EdgeKey = std::array<int64_t, 6>;

[[nodiscard]] EdgeKey edge_key(const glm::dvec3& a, const glm::dvec3& b) {
    auto quantise = [](double v) {
        return static_cast<int64_t>(std::llround(v / kEdgeQuantum));
    };
    const std::array<int64_t, 3> first = {quantise(a.x), quantise(a.y), quantise(a.z)};
    const std::array<int64_t, 3> second = {quantise(b.x), quantise(b.y), quantise(b.z)};
    const bool swap = second < first;
    const std::array<int64_t, 3>& low = swap ? second : first;
    const std::array<int64_t, 3>& high = swap ? first : second;
    return EdgeKey{low[0], low[1], low[2], high[0], high[1], high[2]};
}

/// Count every boundary edge of every face, keyed on quantised positions
///
/// Integers, not an epsilon compare: an epsilon compare is not transitive and
/// the container built on it would have no strict weak ordering to work with.
[[nodiscard]] std::map<EdgeKey, int> count_edges(const ShapeGeometry& geometry) {
    std::map<EdgeKey, int> counts;
    auto walk = [&](const std::vector<uint32_t>& ring) {
        for (size_t i = 0; i < ring.size(); ++i) {
            const glm::dvec3& a = geometry.positions[ring[i]];
            const glm::dvec3& b = geometry.positions[ring[(i + 1) % ring.size()]];
            ++counts[edge_key(a, b)];
        }
    };
    for (const Face& face : geometry.faces) {
        walk(face.loop);
        for (const std::vector<uint32_t>& hole : face.holes) {
            walk(hole);
        }
    }
    return counts;
}

/**
 * @brief Is this face the whole of the shape's surface there?
 *
 * True when no boundary edge of the face is shared with another face -- which is
 * what a bare footprint out of shape_from_polygon() looks like, and is not what
 * the top cap of an extruded box looks like. It decides whether the roof needs a
 * floor of its own; see the header.
 */
[[nodiscard]] bool face_is_free_standing(const ShapeGeometry& geometry,
                                         const std::map<EdgeKey, int>& counts,
                                         const Face& face) {
    auto shared = [&](const std::vector<uint32_t>& ring) {
        for (size_t i = 0; i < ring.size(); ++i) {
            const glm::dvec3& a = geometry.positions[ring[i]];
            const glm::dvec3& b = geometry.positions[ring[(i + 1) % ring.size()]];
            const auto it = counts.find(edge_key(a, b));
            if (it != counts.end() && it->second > 1) {
                return true;
            }
        }
        return false;
    };
    if (shared(face.loop)) {
        return false;
    }
    for (const std::vector<uint32_t>& hole : face.holes) {
        if (shared(hole)) {
            return false;
        }
    }
    return true;
}

/// Copy one face and the positions it uses into another geometry
void copy_face(const ShapeGeometry& source, const Face& face, ShapeGeometry& out) {
    auto copy_ring = [&](const std::vector<uint32_t>& ring) {
        std::vector<uint32_t> indices;
        indices.reserve(ring.size());
        for (const uint32_t index : ring) {
            indices.push_back(static_cast<uint32_t>(out.positions.size()));
            out.positions.push_back(source.positions[index]);
        }
        return indices;
    };

    Face copy;
    copy.material = face.material;
    copy.loop = copy_ring(face.loop);
    for (const std::vector<uint32_t>& hole : face.holes) {
        copy.holes.push_back(copy_ring(hole));
    }
    out.faces.push_back(std::move(copy));
}

// ============================================================================
// Arguments
// ============================================================================

/// The ridge, downhill or band-count argument, resolved against the kind
struct Extra {
    glm::dvec2 axis{1.0, 0.0};
    int bands = kDefaultDomeBands;
};

/**
 * @brief Work out what the fourth argument meant for this kind
 *
 * A kind that has no use for it REPORTS it. ast.hpp makes the same argument
 * about annotations: an argument nothing reacts to looks exactly like a working
 * one until someone wonders why the roof never turns.
 */
[[nodiscard]] Extra resolve_extra(const RoofParams& params,
                                  const Outline& outline,
                                  std::vector<std::string>& notes) {
    Extra out;
    const glm::dvec2 axis = principal_axis(outline.outer);

    switch (params.kind) {
    case RoofKind::Gable:
    case RoofKind::Ridge:
        out.axis = axis;
        if (params.has_extra) {
            double sine = 0.0;
            double cosine = 0.0;
            deg_sin_cos(params.extra, sine, cosine);
            out.axis = glm::dvec2{cosine, sine};
        }
        break;
    case RoofKind::Shed:
        // Default: straight down the slope of the footprint's short way, which
        // is the direction a shed on a long thin building has to fall.
        out.axis = plan_right(axis);
        if (params.has_extra) {
            double sine = 0.0;
            double cosine = 0.0;
            deg_sin_cos(params.extra, sine, cosine);
            out.axis = glm::dvec2{cosine, sine};
        }
        break;
    case RoofKind::Dome:
        if (params.has_extra) {
            const double rounded = std::nearbyint(params.extra);
            const double clamped = std::min(std::max(rounded,
                                                     static_cast<double>(kMinDomeBands)),
                                            static_cast<double>(kMaxDomeBands));
            if (clamped != rounded) {
                notes.push_back("a dome of " + format_number(params.extra) +
                                " bands was clamped to " + format_number(clamped));
            }
            out.bands = static_cast<int>(clamped);
        }
        break;
    case RoofKind::Hip:
    case RoofKind::Pyramid:
        if (params.has_extra) {
            notes.push_back(std::string{"a "} + roof_kind_name(params.kind) +
                            " has no use for a fourth argument, and it was ignored");
        }
        break;
    }
    return out;
}

} // namespace

// ============================================================================
// Kinds
// ============================================================================

const char* roof_kind_name(RoofKind kind) {
    switch (kind) {
    case RoofKind::Gable:
        return "gable";
    case RoofKind::Hip:
        return "hip";
    case RoofKind::Pyramid:
        return "pyramid";
    case RoofKind::Ridge:
        return "ridge";
    case RoofKind::Shed:
        return "shed";
    case RoofKind::Dome:
        return "dome";
    }
    return "hip";
}

bool parse_roof_kind(const std::string& text, RoofKind& out) {
    if (text == "gable") {
        out = RoofKind::Gable;
        return true;
    }
    if (text == "hip") {
        out = RoofKind::Hip;
        return true;
    }
    if (text == "pyramid") {
        out = RoofKind::Pyramid;
        return true;
    }
    if (text == "ridge") {
        out = RoofKind::Ridge;
        return true;
    }
    if (text == "shed") {
        out = RoofKind::Shed;
        return true;
    }
    if (text == "dome") {
        out = RoofKind::Dome;
        return true;
    }
    return false;
}

// ============================================================================
// The operation
// ============================================================================

OpResult roof_shape(Shape& shape, const RoofParams& params, RoofReport& report) {
    report = RoofReport{};

    if (!(params.pitch_degrees > 0.0) || !(params.pitch_degrees < 90.0)) {
        // Both ends are refused rather than clamped: 0 is a flat roof, which is
        // this operation doing nothing, and 90 is a wall. Either is a mistake
        // worth a line number.
        return OpResult::failure("a pitch of " + format_number(params.pitch_degrees) +
                                 " degrees raises no roof; it must be above 0 and below 90");
    }
    double sine = 0.0;
    double cosine = 0.0;
    deg_sin_cos(params.pitch_degrees, sine, cosine);
    if (!(std::fabs(cosine) > 0.0)) {
        return OpResult::failure("a pitch of " + format_number(params.pitch_degrees) +
                                 " degrees is vertical");
    }
    report.pitch_tangent = sine / cosine;

    if (shape.geometry.faces.empty()) {
        return OpResult::failure("the shape has no faces to raise a roof on");
    }

    // The footprint is every face pointing along the scope's own +y. Local, not
    // world: a building placed on a lot at 40 degrees carries that rotation in
    // its axes and its roof still has to sit on the building.
    std::vector<size_t> footprints;
    for (size_t i = 0; i < shape.geometry.faces.size(); ++i) {
        const glm::dvec3 normal = face_normal(shape.geometry, shape.geometry.faces[i]);
        if (glm::dot(normal, normal) > 0.25 && normal.y >= kUpCosine) {
            footprints.push_back(i);
        }
    }
    if (footprints.empty()) {
        // Name a remedy that WORKS. This used to say align_scope("geometry"),
        // which does not: Geometry mode points y at the dominant face's normal,
        // and on a shape that IS one face it is already pointing there, so the
        // call changes nothing and the author is sent round the same loop. The
        // modes that turn local +y back to world up are "y_up" and "world".
        return OpResult::failure(
            "the shape has no upward face to raise a roof on. A face picked out by "
            "'select face' has its own normal along local z, so put the frame back "
            "with align_scope(\"y_up\") before raising the roof -- or raise it on the "
            "solid instead, since roof() finds the top face of an extruded mass by "
            "itself");
    }

    const std::map<EdgeKey, int> edge_counts = count_edges(shape.geometry);

    // Built beside the old geometry and swapped in at the end. A roof that fails
    // half way through is a hole in a building, not a partial answer, so nothing
    // is written back until every footprint has succeeded.
    ShapeGeometry built;
    for (size_t i = 0; i < shape.geometry.faces.size(); ++i) {
        if (std::find(footprints.begin(), footprints.end(), i) == footprints.end()) {
            copy_face(shape.geometry, shape.geometry.faces[i], built);
        }
    }

    for (const size_t index : footprints) {
        const Face& face = shape.geometry.faces[index];

        Outline outline;
        std::string why;
        if (!face_outline(shape.geometry, face, outline, why)) {
            return OpResult::failure(why);
        }

        if (!outline.holes.empty() && params.kind != RoofKind::Shed) {
            // straight_skeleton.hpp is emphatic that its faces over a hole are
            // "land that does not exist", so roofing the courtyard is not an
            // approximation. Naming the two ways out beats a silent drop.
            return OpResult::failure(
                std::string{"a "} + roof_kind_name(params.kind) + " cannot be raised on a "
                "footprint with " + std::to_string(outline.holes.size()) +
                " interior ring(s); 'shed' carries them, and delete_holes() removes them");
        }
        if (!outline.holes.empty() && params.overhang != 0.0) {
            return OpResult::failure(
                "an overhang cannot be applied to a footprint with interior rings: the "
                "courtyard would have to shrink by the same amount and nothing says it should");
        }

        const bool free_standing = face_is_free_standing(shape.geometry, edge_counts, face);
        const std::vector<glm::dvec2> wall_top = outline.outer;

        if (params.overhang != 0.0) {
            std::vector<glm::dvec2> moved;
            if (!offset_ring(outline.outer, params.overhang, moved, why)) {
                return OpResult::failure(why);
            }
            outline.outer = std::move(moved);
            outline.diameter = ring_diameter(outline.outer);
        }

        RoofBuilder builder(built, outline.material,
                            kFlatTolerance * std::max(1.0, outline.diameter));
        const Extra extra = resolve_extra(params, outline, report.notes);

        OpResult result = OpResult::success();
        switch (params.kind) {
        case RoofKind::Gable:
            result = build_gable(outline, report.pitch_tangent, extra.axis, builder, report);
            break;
        case RoofKind::Hip:
            result = build_hip(outline, report.pitch_tangent, builder, report);
            break;
        case RoofKind::Pyramid:
            result = build_pyramid(outline, report.pitch_tangent, builder, report);
            break;
        case RoofKind::Ridge:
            result = build_ridge(outline, report.pitch_tangent, extra.axis, builder, report);
            break;
        case RoofKind::Shed:
            result = build_shed(outline, report.pitch_tangent, extra.axis, builder, report);
            break;
        case RoofKind::Dome:
            result = build_dome(outline, report.pitch_tangent, extra.bands, builder, report);
            break;
        }
        if (!result.ok) {
            return result;
        }

        // The ring between the wall top and the eave. Facing DOWN for an
        // oversail, which is a soffit, and UP for an inset, which is a ledge.
        // Either way it is what keeps the solid closed across the step.
        // Nothing to bridge when there is no wall below: a free-standing
        // footprint's own ring is not a wall top, it is the edge of a sheet, and
        // the roof's floor already covers out to the offset ring.
        if (params.overhang != 0.0 && !free_standing) {
            std::vector<glm::dvec3> outer;
            std::vector<glm::dvec3> inner;
            const bool oversail = params.overhang > 0.0;
            for (const glm::dvec2& p : (oversail ? outline.outer : wall_top)) {
                outer.push_back(raise(p, outline.eave));
            }
            for (const glm::dvec2& p : (oversail ? wall_top : outline.outer)) {
                inner.push_back(raise(p, outline.eave));
            }
            builder.add_face_with_holes(outer, {inner},
                                        glm::dvec3{0.0, oversail ? -1.0 : 1.0, 0.0});
        }

        // A footprint nothing else closes needs a floor, or the roof is an open
        // shell. One that a wall already closes must NOT get one, or the
        // building gains an interior partition and its volume is wrong by the
        // cap's own contribution.
        if (free_standing) {
            std::vector<glm::dvec3> floor;
            for (const glm::dvec2& p : outline.outer) {
                floor.push_back(raise(p, outline.eave));
            }
            std::vector<std::vector<glm::dvec3>> floor_holes;
            for (const std::vector<glm::dvec2>& hole : outline.holes) {
                std::vector<glm::dvec3> ring;
                for (const glm::dvec2& p : hole) {
                    ring.push_back(raise(p, outline.eave));
                }
                floor_holes.push_back(std::move(ring));
            }
            builder.add_face_with_holes(floor, floor_holes, glm::dvec3{0.0, -1.0, 0.0});
        }

        report.roof_faces += builder.faces();
        ++report.footprints;
    }

    shape.geometry = std::move(built);
    refit_scope(shape);
    return OpResult::success();
}

// ============================================================================
// Registration
// ============================================================================

namespace {

/// A number argument, or a reported fault. The same shape as interpreter.cpp's
/// own arg_number(), which is private to that translation unit.
[[nodiscard]] bool roof_number(OperationArgs& context, size_t index, double& out) {
    if (index >= context.args.size() || !context.args[index].is_number()) {
        context.interpreter.fail_shape(context.loc,
                                       "'roof' wants a number for argument " +
                                           std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_number();
    return true;
}

void op_roof(OperationArgs& context) {
    if (context.args.empty() || !context.args[0].is_text()) {
        context.interpreter.fail_shape(
            context.loc, "'roof' wants the roof shape as its first argument: gable, hip, "
                         "pyramid, ridge, shed or dome");
        return;
    }

    RoofParams params;
    if (!parse_roof_kind(context.args[0].as_text(), params.kind)) {
        context.interpreter.fail_shape(context.loc,
                                       "'roof' does not know the shape '" +
                                           context.args[0].as_text() +
                                           "'; it knows gable, hip, pyramid, ridge, shed and "
                                           "dome");
        return;
    }
    if (context.args.size() >= 2 && !roof_number(context, 1, params.pitch_degrees)) {
        return;
    }
    if (context.args.size() >= 3 && !roof_number(context, 2, params.overhang)) {
        return;
    }
    if (context.args.size() >= 4) {
        if (!roof_number(context, 3, params.extra)) {
            return;
        }
        params.has_extra = true;
    }

    RoofReport report;
    const OpResult result = roof_shape(context.shape, params, report);

    // The notes are reported whether or not the roof built, because the reason a
    // gable end was hipped is exactly the context a failure needs.
    for (const std::string& note : report.notes) {
        context.interpreter.report(Severity::Warning, context.loc, "'roof': " + note);
    }
    if (!result.ok) {
        context.interpreter.fail_shape(context.loc, "'roof': " + result.message);
    }
}

[[nodiscard]] OperationTable build_roof_operations() {
    OperationTable table = standard_operations();
    register_roof_operations(table);
    return table;
}

} // namespace

void register_roof_operations(OperationTable& table) {
    table.register_operation("roof", op_roof);
}

const OperationTable& roof_operations() {
    static const OperationTable table = build_roof_operations();
    return table;
}

} // namespace stratum::procgen::rules
