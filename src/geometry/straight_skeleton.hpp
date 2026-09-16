// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file straight_skeleton.hpp
 * @brief The straight skeleton of a simple polygon, and the faces it cuts the polygon into
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this exists, and why it is in src/geometry rather than src/osm/road
 *
 * Two unrelated-looking features are the same computation:
 *
 *   - **Skeleton lot subdivision (C3).** The land nearest to one street edge of a
 *     block is that edge's straight-skeleton FACE. Cutting those faces across
 *     gives lots that front a street, reach back to the middle of the block, and
 *     have sides perpendicular to the road. Nothing else produces that shape.
 *   - **Roof operations (D5).** A gabled or hipped roof over an arbitrary
 *     footprint IS the straight skeleton of that footprint, lifted: every
 *     skeleton node rises by its time, every face becomes one roof plane.
 *
 * Writing it once in road/ and again in a roof builder is how the two quietly
 * disagree -- a ridge line that a lot boundary follows and a ridge line the roof
 * is built on would differ by an epsilon and nobody would find out until a
 * building overhung its own parcel. So it lives here, knows nothing about OSM,
 * roads or rendering, and both callers read the same output.
 *
 * ### What a straight skeleton is
 *
 * Move every edge of the polygon inward, parallel to itself, at unit speed. The
 * polygon shrinks; each vertex slides along the bisector of its two edges. The
 * traces those vertices leave are the skeleton ARCS, and the times at which the
 * shrinking boundary -- the wavefront -- changes combinatorially are the EVENTS:
 *
 *   - **Edge event.** An edge shrinks to nothing and its two vertices meet. The
 *     two vertices die, one new vertex is born where they met, and the polygon
 *     loses a side. This is every event a convex polygon ever has.
 *   - **Split event.** A REFLEX vertex reaches an edge on the far side of the
 *     wavefront and cuts it in two. The wavefront splits into two independent
 *     loops that shrink on from there.
 *
 * **Split events are the part naive implementations skip, and skipping them is
 * not a degradation, it is a wrong answer.** A polygon that needs one and does
 * not get it produces a skeleton whose arcs cross each other and faces that
 * overlap: lots that overlap, or a roof that folds through itself. This file
 * implements them. Where it cannot finish it says so (`StraightSkeleton::complete`)
 * and emits no faces at all, because a caller that gets no answer falls back and
 * a caller that gets a self-intersecting answer ships it.
 *
 * ### Scope, stated plainly
 *
 *   - **HOLES ARE NOT HANDLED.** The input is ONE ring. A polygon with a
 *     courtyard needs the wavefronts of the hole boundaries to expand outward and
 *     merge with the outer wavefront, which is a different set of events (and a
 *     different validity test, since a reflex vertex may then hit an edge of
 *     ANOTHER loop). Handing this function only the outer ring of a polygon with
 *     holes returns a skeleton of the SOLID polygon, which is wrong over the
 *     hole -- it is not an approximation, the faces there are land that does not
 *     exist. Callers must deal with holes themselves; see
 *     `osm/road/lots_offset.hpp` for how the lot subdivision does.
 *   - **The input must be a simple polygon**: at least three points, no repeated
 *     point, no two non-adjacent edges crossing. Self-intersection is CHECKED for
 *     and refused (`SkeletonStats::self_intersections`), because the wavefront
 *     simulation over a self-intersecting ring does not terminate in any
 *     meaningful state and would otherwise burn the iteration budget and return
 *     rubbish.
 *   - **Winding is fixed up, not demanded.** A clockwise ring is reversed
 *     internally and `StraightSkeleton::contour` reports the ring the face edge
 *     indices actually refer to. Read `contour`, never the ring you passed in.
 *   - **Weighted skeletons are not supported.** Every edge moves at the SAME
 *     speed, so a roof built from this has one pitch. Per-edge speeds (a mansard,
 *     a shed dormer) change the velocity solve and the event times and are a
 *     different function; they are not bolted on here.
 *   - **Simultaneous and near-simultaneous events are handled by cascade, not by
 *     exact degeneracy analysis.** Four edges collapsing to one point come out as
 *     several events at the same time, which is the right answer.
 *   - **THERE IS NO VERTEX-EVENT HANDLER.** Two reflex vertices of the same loop
 *     arriving at exactly the same point -- which the textbook calls a vertex
 *     event -- is not detected. The simulation lets both carry on past each
 *     other, and the AREAS still come out exact, because the extra excursion is
 *     an out-and-back along one bisector and encloses nothing. The RING does not:
 *     it leaves and re-enters along the same line and so crosses itself, and a
 *     self-intersecting ring is the one thing this file must never hand back.
 *     `build_faces()` therefore deletes the out-and-back before the face is
 *     emitted and counts it in SkeletonStats::folded_spurs. The face that comes
 *     out is the right region with the right area; what is lost is the excursion,
 *     which was never land. Non-zero `folded_spurs` means the input had an exact
 *     symmetry in it -- a comb of equal teeth is the shape that produces one --
 *     and is reported so that a caller can tell the difference between a face
 *     this file computed and a face it repaired.
 *
 * ### The method: full re-evaluation, on purpose
 *
 * The textbook implementation keeps a priority queue of events and validates them
 * lazily on pop, because a queued event can be invalidated by an earlier one. That
 * is faster and it is also where every straight-skeleton bug report starts: a
 * stale event that passes its validity check, or a valid event that is discarded
 * and never re-queued, and the symptom is a face that is subtly wrong on one
 * polygon in a thousand.
 *
 * This implementation recomputes every candidate event from the CURRENT wavefront
 * at every step and takes the earliest. There is no queue, so there is no such
 * thing as a stale event. The cost is O(events * vertices * reflex_vertices),
 * which for the polygons this project feeds it -- block rings of tens to a few
 * hundred points -- is microseconds. It is not the algorithm to use on a
 * million-vertex coastline, and the iteration cap makes that refusal explicit
 * rather than a hang.
 *
 * ### Determinism
 *
 * Same ring, same skeleton, bit for bit, on any machine. Every event time comes
 * out of +, -, *, / and sqrt, which IEEE-754 rounds identically everywhere. No
 * transcendental function touches a coordinate: the vertex velocity is a 2x2
 * linear solve in the edge normals rather than a half-angle formula, and the
 * reflex test is a cross product rather than an angle. Ties between events at the
 * same time are broken by kind and then by index, never by container order.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stratum::geometry {

// ============================================================================
// Type Aliases
// ============================================================================

/// Index into StraightSkeleton::nodes
using SkeletonNodeId = uint32_t;

/// Sentinel for an unset node index
inline constexpr SkeletonNodeId kInvalidSkeletonNode = 0xFFFFFFFFu;

/// Sentinel for "no contour edge", in the same spirit as road::kNoBlockEdge
inline constexpr uint32_t kNoSkeletonEdge = 0xFFFFFFFFu;

// ============================================================================
// Output
// ============================================================================

/**
 * @brief One point of the skeleton: a contour vertex, or a place where the
 *        wavefront changed
 */
struct SkeletonNode {
    /// Position in the same units as the input ring
    glm::dvec2 position{0.0};

    /**
     * @brief Distance the wavefront had travelled when this node was created
     *
     * Equivalently, the distance from this point to the NEAREST edge of the
     * polygon -- which is the property that makes the skeleton useful: it is the
     * height of a unit-pitch roof at this point, and it is the depth of a lot
     * that reaches here. Zero for a contour vertex.
     */
    double time = 0.0;

    /// True when this node is one of the input ring's own vertices
    bool on_contour = false;
};

/**
 * @brief The trace of one wavefront vertex between two events
 *
 * An arc is a straight segment -- that is the whole point of a STRAIGHT skeleton,
 * as against the medial axis, whose analogue here would be a parabola wherever a
 * vertex is involved.
 */
struct SkeletonArc {
    SkeletonNodeId from = kInvalidSkeletonNode;  ///< Node the moving vertex left
    SkeletonNodeId to = kInvalidSkeletonNode;    ///< Node it arrived at

    /**
     * @brief The two contour edges this arc bisects
     *
     * Indices into StraightSkeleton::contour, naming the edge that starts at that
     * vertex. `left_edge` is the edge on the arc's left going from `from` to
     * `to`. An arc is equidistant from these two edges' supporting lines, which
     * is what a caller checks it against.
     */
    uint32_t left_edge = kNoSkeletonEdge;
    uint32_t right_edge = kNoSkeletonEdge;
};

/**
 * @brief The land nearest to one contour edge
 *
 * The faces tile the polygon: they do not overlap, and their areas sum to the
 * polygon's area. That is the invariant worth testing, because it is exactly the
 * one a missing split event breaks.
 *
 * `ring[0]` and `ring[1]` are always the two ends of contour edge `edge`, in the
 * contour's own direction, so the face is anchored to its edge without a search.
 */
struct SkeletonFace {
    /// Index into StraightSkeleton::contour of the edge this face belongs to
    uint32_t edge = kNoSkeletonEdge;

    /// Boundary, anticlockwise, first point NOT repeated -- Block::ring's convention
    std::vector<glm::dvec2> ring;

    /**
     * @brief SkeletonNode::time of each ring vertex; parallel to `ring`
     *
     * Zero on the two contour ends and positive inside. This is the roof height
     * D5 lifts the face by, and it is carried here so a caller never has to match
     * a face vertex back to a node by position.
     */
    std::vector<double> times;

    /// Enclosed area, always positive for a face this file emits
    double area = 0.0;
};

/**
 * @brief What the simulation did, for logging and for tests
 *
 * Carried for the same reason road::BlockStats is: a skeleton that came out wrong
 * still looks like a list of segments. These counts are what says which path the
 * code took.
 */
struct SkeletonStats {
    size_t edge_events = 0;   ///< Edges that shrank to nothing
    size_t split_events = 0;  ///< Reflex vertices that cut an opposite edge
    size_t iterations = 0;    ///< Event steps taken
    size_t reflex_vertices = 0;  ///< Reflex vertices in the input ring

    /**
     * @brief Wavefront loops finished by a shortcut rather than by a clean event
     *
     * Two shapes get one. A loop that falls below three vertices -- which a split
     * can leave behind -- already encloses nothing, since both its vertices are
     * the intersection of the same two moving lines. And a loop that runs out of
     * AREA while it still has four or more vertices is a wavefront that flattened
     * into a ridge, which happens whenever two opposite sides of the polygon
     * arrive at each other at the same instant: an L with arms of equal width, a
     * plus sign, a rectangle's two ends.
     *
     * Non-zero is normal for those shapes and zero for a convex polygon. It is
     * reported because it is the only place the simulation takes a shortcut, so it
     * is the first thing to look at when a face comes out wrong.
     */
    size_t degenerate_loops = 0;

    /**
     * @brief Vertices created with no finite offset velocity
     *
     * Two exactly opposing edges met, so their meeting point is the whole line
     * rather than one point. Non-zero is NORMAL: it is what the last instant of a
     * square's wavefront looks like, and it is resolved by the flat-loop sweep.
     * It is reported because a large count on a polygon that should be simple is
     * a sign the input has spikes in it.
     */
    size_t degenerate_vertices = 0;

    /// Faces that came out with non-positive area and were dropped
    size_t degenerate_faces = 0;

    /**
     * @brief Face-ring vertices deleted because the ring doubled back on itself
     *
     * The missing vertex event, repaired. See the scope section of this file's
     * header: two reflex vertices meeting exactly leave the face ring with an
     * out-and-back excursion along one bisector, which encloses no area but does
     * make the ring self-intersecting. Each such vertex is removed here and
     * counted, so the repair is visible rather than silent.
     *
     * Zero for every shape without an exact symmetry in it. Non-zero is not an
     * error and does not change any face's area; it says the input had one.
     */
    size_t folded_spurs = 0;

    /// Pairs of non-adjacent contour edges that cross. Non-zero means the input was refused.
    size_t self_intersections = 0;

    /// Largest SkeletonNode::time reached; the inradius of the polygon
    double max_time = 0.0;
};

/**
 * @brief The skeleton, plus the evidence that it is the whole skeleton
 */
struct StraightSkeleton {
    /**
     * @brief False when the simulation did not finish
     *
     * **Check it.** When it is false the input was refused (fewer than three
     * distinct points, no area, self-intersecting) or the iteration cap was hit,
     * `faces` is EMPTY, and `nodes` and `arcs` hold only whatever was resolved
     * before the stop. A partial arc set is fine to draw and is not fit to build
     * on.
     */
    bool complete = false;

    /**
     * @brief The ring the face and arc edge indices refer to
     *
     * The input with consecutive duplicate points removed, and reversed if the
     * input was clockwise. It is NOT always the vector that was passed in, which
     * is why it is returned rather than assumed.
     */
    std::vector<glm::dvec2> contour;

    /// Contour vertices first, in contour order, then event nodes in event order
    std::vector<SkeletonNode> nodes;

    /// Every arc, in the order its end event happened
    std::vector<SkeletonArc> arcs;

    /**
     * @brief One face per contour edge, in contour order; empty unless `complete`
     *
     * A face can be missing when it came out degenerate (see
     * SkeletonStats::degenerate_faces), so index into this by scanning
     * SkeletonFace::edge rather than assuming `faces[i].edge == i`.
     */
    std::vector<SkeletonFace> faces;

    SkeletonStats stats;
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tunables of the simulation. The defaults are right for map-scale metres.
 */
struct SkeletonConfig {
    /**
     * @brief Hard ceiling on event steps; 0 means `8 * vertices + 64`
     *
     * A backstop, not a design parameter. A well-formed simple polygon resolves in
     * O(vertices) events, so hitting the cap means the geometry defeated the
     * simulation -- and then `complete` comes back false and the caller falls back,
     * which is the whole reason the cap is a number rather than a `while (true)`.
     */
    size_t max_iterations = 0;

    /**
     * @brief Largest ring this will attempt; 0 means 4096
     *
     * The simulation is quadratic-ish in the vertex count (see the file header),
     * so a ring far beyond a city block is refused outright rather than run for
     * minutes. Refusing is visible; a hang is not.
     */
    size_t max_vertices = 0;

    /**
     * @brief Points closer than this are the same point, in input units
     *
     * A micrometre, matching osm/road/blocks.cpp and lots.cpp, so a ring that came
     * out of the block traversal is not re-quantised on the way in here.
     */
    double point_epsilon = 1e-6;
};

// ============================================================================
// Entry Points
// ============================================================================

/**
 * @brief Compute the straight skeleton of a simple polygon
 *
 * @param ring   Closed ring, first point NOT repeated. Either winding; see
 *               StraightSkeleton::contour. Holes are NOT supported -- read the
 *               scope section of this file's header before passing the outer ring
 *               of a polygon that has them.
 * @param config Tunables; the defaults suit map-scale metres
 * @return The skeleton. Test `complete` before reading `faces`.
 */
[[nodiscard]] StraightSkeleton compute_straight_skeleton(const std::vector<glm::dvec2>& ring,
                                                         const SkeletonConfig& config = {});

/**
 * @brief The velocity a ring vertex moves at when the ring is offset inward
 *
 * The vector @p v such that moving the vertex to `vertex + v * d` puts it exactly
 * on BOTH of its edges' lines offset inward by @p d. Its length is
 * `1 / sin(half the interior angle)`, so it is longer than one for a sharp corner
 * -- that is the mitre, and it is why an offset ring is not just the vertices
 * pushed along their normals.
 *
 * Exported because the offset lot subdivision needs exactly this to decide which
 * part of an offset band belongs to which contour edge, and a second copy of the
 * formula there would be free to disagree with the skeleton's at the corner where
 * it matters.
 *
 * The ring is assumed ANTICLOCKWISE, so "inward" is to the left of `prev -> next`.
 * Pass a clockwise ring and you get the outward velocity.
 *
 * @param prev         Previous ring vertex
 * @param vertex       The vertex to move
 * @param next         Next ring vertex
 * @param out_velocity Set on success; untouched on failure
 * @return False when the vertex has no finite velocity -- a zero-length edge, or
 *         a spike whose two edges fold exactly back on each other. Callers must
 *         test it: glm::normalize of a zero vector returns NaN, and a NaN
 *         coordinate propagates silently through every polygon downstream.
 */
[[nodiscard]] bool vertex_offset_velocity(const glm::dvec2& prev,
                                          const glm::dvec2& vertex,
                                          const glm::dvec2& next,
                                          glm::dvec2& out_velocity);

/// Signed area of a closed ring, first point not repeated. Positive anticlockwise.
[[nodiscard]] double signed_ring_area(const std::vector<glm::dvec2>& ring);

} // namespace stratum::geometry
