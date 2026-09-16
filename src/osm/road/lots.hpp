// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file lots.hpp
 * @brief Lot subdivision: cutting a block into parcels that keep their identity
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this exists
 *
 * A block (blocks.hpp) is the land a ring of streets encloses. Nothing is ever
 * built on a block: it is built on a LOT. Rules are targeted at lots (C3, D),
 * frontage is classified per lot (C4), and every attribute an author sets is set
 * on a lot. This file is the step that turns one face of the street graph into
 * the parcels the rest of the programme addresses.
 *
 * It implements C2 (the subdivision) and C7 (the identity) TOGETHER, because
 * identity cannot be retrofitted. An id bolted on afterwards is an index into
 * whatever order the subdivision happened to emit, and the first time a street
 * moves, every rule assignment and manual edit in the city shuffles onto a
 * different piece of land. The authoring loop is then useless, and the fix is a
 * rewrite of the subdivision rather than an addition to it.
 *
 * ### The method: recursive oriented-bounding-box subdivision
 *
 * The one CityEngine documents, and the one that produces the deep narrow
 * parcels real streets are lined with:
 *
 *   1. Take the MINIMUM-AREA oriented bounding box of the lot. Not the
 *      axis-aligned box -- a block on a diagonal street would then be cut into
 *      staircase slivers that front nothing.
 *   2. Pivot at the midpoint of the largest OBB edge. That midpoint lies on the
 *      box centre line, so both of the two largest edges give the same cut line
 *      and there is nothing to choose between them.
 *   3. Cut along the direction of the SMALLEST edge, which halves the long axis.
 *      Cutting the other way would halve the short axis and drive every piece
 *      towards a sliver.
 *   4. Recurse while the constraints below still hold.
 *
 * Offset subdivision and skeleton subdivision are C3. They need a straight
 * skeleton, which is shared with the roof operations and is separate work, and
 * neither is stubbed here: a half-built skeleton that silently falls back to OBB
 * is worse than not having the mode at all.
 *
 * ### C7, part one: determinism
 *
 * Same block, same seed, same lots -- to the bit, on any machine.
 *
 *   - **The seed is mixed PER NODE of the subdivision tree, never drawn from one
 *     shared stream.** A shared stream makes every lot depend on how many lots
 *     were cut before it, so adding one split at the top of a block changes the
 *     geometry of every lot below it. Each node instead derives its own key from
 *     its parent's key, and every draw it makes is a pure function of that key.
 *     Lot::node_key is that key, and subdivide_ring() re-enters the recursion at
 *     any node: running a subtree on its own reproduces it exactly, which a
 *     shared stream could not do, and which a test asserts.
 *   - **No hash container is read in iteration order anywhere in the
 *     implementation.** Pieces of a cut are ordered by their position relative to
 *     the cut line, which is a geometric fact rather than an allocation accident.
 *   - **No transcendental function ever touches a coordinate.** libm is not
 *     bit-reproducible across platforms, so sin, cos, atan2 and their relatives
 *     are absent from the whole geometric path: the OBB works in direction
 *     vectors rather than angles, the irregularity tilt is a slope rather than a
 *     rotation, and the mean-value coordinates use the half-angle tangent
 *     identity. What is left is +, -, *, / and sqrt, which IEEE-754 rounds
 *     identically everywhere. THE ONE EXCEPTION is LotParams::corner_angle_max_deg,
 *     which is turned into a cosine by std::cos once per block; it feeds a
 *     comparison, not a coordinate, so the only thing a differing libm can change
 *     is a corner that sits exactly on the threshold.
 *
 * ### C7, part two: identity that survives regeneration
 *
 * Move a street, re-subdivide, and a lot that is still the same lot must still
 * have the same id. Three mechanisms, in the order they are tried:
 *
 *   1. **The block key.** LotParams::block_key is supplied by the caller and must
 *      outlive one extraction. Block::id does NOT: it is an index into one
 *      BlockExtraction and changes the moment a way is added upstream.
 *      block_key_from_ways() derives one from the sorted, de-duplicated OSM way
 *      ids on the block boundary, which survive any amount of geometry editing.
 *   2. **The recursion path.** A lot's provisional id is derived from its
 *      node_key, which is its address in the subdivision tree. Nudge a street and
 *      the tree keeps its shape, so every id is unchanged with no matching step
 *      at all. This is the case that matters most, because it is the one that
 *      happens on every drag of a node.
 *   3. **Barycentric transfer**, for when the tree shape DOES change -- the block
 *      grew enough for one more split, or a corner lot appeared. transfer_lot_ids()
 *      writes each new lot's centroid in mean-value coordinates of the new block
 *      ring, reads those coordinates back out against the old ring, and inherits
 *      the id of the old lot that lands on. This is what CityEngine does and it
 *      is the only part that can be approximate: a new lot with no predecessor
 *      keeps its provisional id, which is correct, since it is new land.
 *
 * Mean-value coordinates rather than triangle barycentrics because a block ring
 * is an arbitrary polygon with any number of vertices, frequently non-convex, and
 * mean-value coordinates are defined for all of them, reproduce affine maps
 * exactly, and vary smoothly when one vertex moves.
 *
 * ### What this does NOT do
 *
 *   - **No frontage classification.** Which street a lot faces, and whether that
 *     frontage is primary, is C4. This file records WHICH block edge each lot
 *     segment came from (Lot::ring_edges) so C4 never has to search back for it
 *     by proximity, which is the same class of mistake as finding junctions by
 *     endpoint proximity and fails in the same place.
 *   - **No holes and no interior access roads.** A lot is a simple ring. A block
 *     whose interior cannot be reached is left as one large lot when
 *     LotParams::force_street_access is 1, rather than being given a driveway
 *     that no street graph knows about.
 *   - **No scene mutation.** subdivide_block() is a pure function of its
 *     arguments, exactly like extract_blocks(). Putting the result INTO a
 *     document is an edit, and that edit goes through scene::CommandStack like
 *     every other.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/blocks.hpp"
#include "osm/road/road_graph.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Type Aliases
// ============================================================================

/**
 * @brief Stable identity of a lot
 *
 * NOT an index. It is a 64-bit value derived from the block key and the lot's
 * address in the subdivision tree, so it is the same value on the next run, in
 * the next session, and after a street has been dragged. Two lots in the same
 * document have the same id only if they are the same lot.
 */
using LotId = uint64_t;

/// Sentinel for an unset LotId. Never produced by the subdivision.
inline constexpr LotId kInvalidLotId = 0;

/**
 * @brief Lot::ring_edges entry for a segment that is an interior cut
 *
 * Deliberately the same bit pattern as road::kInvalidId. A lot segment either
 * came off the block boundary, and then it names the BlockEdge it came from, or
 * it is a cut this file made, and then it fronts nothing.
 */
inline constexpr uint32_t kNoBlockEdge = 0xFFFFFFFFu;

// ============================================================================
// Lot
// ============================================================================

/**
 * @brief One parcel of land cut out of a block
 *
 * Units are local metres throughout, the same frame as Block::ring and
 * GraphEdge::polyline, so a lot can be compared against road geometry without a
 * transform.
 */
struct Lot {
    /**
     * @brief Stable identity; see the LotId and C7 notes above
     *
     * Survives re-subdivision after a street moves. It is what an attribute, a
     * rule assignment or a manual edit is keyed on -- never the index of this lot
     * in LotSubdivision::lots, which is an emission order and nothing more.
     */
    LotId id = kInvalidLotId;

    /**
     * @brief Address of this lot in the subdivision tree
     *
     * The seed state for this node, and the reason there is no shared random
     * stream. Carried rather than kept private because it is the handle
     * subdivide_ring() takes: re-entering the recursion here reproduces this
     * lot's subtree exactly, which is how the per-node seeding is tested.
     *
     * `id` and `node_key` are distinct on purpose. node_key is where the lot sits
     * in the tree; id is who the lot IS, and transfer_lot_ids() can replace id
     * with an inherited one while node_key keeps describing this run.
     */
    uint64_t node_key = 0;

    /**
     * @brief Boundary as a closed anticlockwise ring, first point NOT repeated
     *
     * Same convention as Block::ring and Corridor::outline, so the three go to the
     * same clipping and triangulation code with no special case.
     */
    std::vector<glm::dvec2> ring;

    /**
     * @brief Which block edge carries each ring segment; parallel to `ring`
     *
     * `ring_edges[i]` covers the segment from `ring[i]` to `ring[(i + 1) %
     * ring.size()]`. It indexes Block::edges when the lot came from
     * subdivide_block(), carries whatever tags the caller supplied when it came
     * from subdivide_ring(), and is kNoBlockEdge for a segment this file cut.
     *
     * This is what C4 classifies frontage from. Without it, a lot boundary would
     * have to be matched back to a street by proximity, and two streets a few
     * metres apart would swap their lots' frontage.
     */
    std::vector<uint32_t> ring_edges;

    /// Enclosed area in square metres. Always strictly positive.
    double area = 0.0;

    /**
     * @brief Total length of the ring segments that came off the block boundary
     *
     * Zero exactly when the lot is landlocked. Carried rather than recomputed
     * because force_street_access has to evaluate it for every candidate piece of
     * every split, and because C4 wants it anyway.
     */
    double frontage_length = 0.0;

    /// Depth in the subdivision tree. 0 for a block that was never split.
    uint32_t depth = 0;

    /**
     * @brief True when this lot was sliced off a sharp block corner
     *
     * Corner lots are cut before the recursion and are never subdivided further,
     * so they are the one kind of lot whose area may exceed LotParams::lot_area_max.
     */
    bool is_corner_lot = false;
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tunables of the subdivision
 *
 * The CityEngine name of each parameter is given in brackets. The mapping is
 * deliberately one-to-one: a scene authored against CityEngine documentation
 * should behave the way that documentation says, and a parameter that is
 * ALMOST the same as a documented one is worse than a differently named one.
 */
struct LotParams {
    /**
     * @brief Smallest lot a SPLIT may create, in square metres [lotAreaMin]
     *
     * A floor on splitting, NOT a floor on lots. A block smaller than this comes
     * out as one lot, because the alternative is discarding land: the parcel
     * exists whether or not it is big enough to build on, and a caller that wants
     * to skip it can compare Lot::area itself.
     */
    double lot_area_min = 200.0;

    /**
     * @brief Largest lot the recursion tolerates, in square metres [lotAreaMax]
     *
     * The recursion continues while a piece is strictly larger than this, so a
     * lot of exactly lot_area_max is final. Every other constraint can veto a
     * split, so a lot LARGER than this is a normal outcome and means the veto
     * fired -- read LotStats to find out which one.
     */
    double lot_area_max = 800.0;

    /**
     * @brief Narrowest lot a split may create, in metres [lotWidthMin]
     *
     * Measured as the short side of the piece's own minimum-area bounding box,
     * not as a distance between the cut lines, so an L-shaped piece is judged on
     * the shape it actually is. This is the constraint that stops a long block
     * being shaved into unbuildable ribbons once the area constraint alone would
     * still allow another cut.
     */
    double lot_width_min = 8.0;

    /**
     * @brief Probability that a split which would landlock a piece is refused,
     *        0..1 [forceStreetAccess]
     *
     * At 1 no lot is ever cut off from the street: the deep middle of a large
     * block stays one big lot rather than becoming parcels with no frontage. At 0
     * the constraint is off and interior lots appear, which is what an author
     * wants when the block interior is going to be a car park or a courtyard.
     *
     * Before refusing, the perpendicular cut is tried -- the one along the long
     * axis rather than across it. That is usually enough: the piece becomes a
     * shallower strip that still reaches the street, which is exactly the shape
     * of a real terrace, and the refusal only stands when neither cut works.
     */
    double force_street_access = 1.0;

    /**
     * @brief How far a cut may wander from the exact halving, 0..1 [irregularity]
     *
     * At 0 every cut is the perpendicular bisector of the long axis and the
     * subdivision is a pure function of the geometry -- the seed then changes only
     * the ids, never a coordinate, and a test asserts that.
     *
     * At 1 the pivot slides up to a quarter of the long extent along the axis and
     * the cut leans by up to a slope of 0.3, roughly seventeen degrees. A SLOPE
     * rather than an angle because a rotation needs sin and cos, and libm would
     * then decide whether two machines produce the same city.
     */
    double irregularity = 0.0;

    /**
     * @brief Random seed [seed]
     *
     * Mixed with block_key to make the root node key, and from there down the
     * tree. It has no effect at all when irregularity is 0 and
     * force_street_access is 0 or 1, because then nothing consults a draw.
     */
    uint64_t seed = 0;

    /**
     * @brief Corners sharper than this get their own lot, in degrees [cornerAngleMax]
     *
     * A sharp corner of a block subdivides into a wedge that no building fits in,
     * so the wedge is taken off first and becomes a corner lot in its own right.
     * The default of 20 degrees leaves an ordinary rectangular block completely
     * alone -- its corners are 90 degrees -- and only fires on the acute corner of
     * a triangular block or a fork.
     *
     * Set to 0 to disable. Only a CONVEX corner whose two sides both come off the
     * block boundary is eligible: a reflex corner has no wedge to remove, and a
     * corner between two interior cuts is not a street corner.
     */
    double corner_angle_max_deg = 20.0;

    /**
     * @brief How far a corner lot reaches along each of its two sides, in metres
     *        [cornerWidth]
     *
     * Clamped to half of each side's length, so two sharp corners at the ends of a
     * short edge can never cut overlapping wedges out of the same block.
     */
    double corner_width = 12.0;

    /**
     * @brief Caller-supplied stable key for the block being subdivided
     *
     * The root of every id in the block. It MUST be stable across re-extraction;
     * Block::id is not, since it is an index into one BlockExtraction. Use
     * block_key_from_ways(), which derives it from OSM way identity.
     *
     * Left at 0 every block in a city shares a root, and two different blocks
     * subdivided the same way produce colliding ids.
     */
    uint64_t block_key = 0;

    /**
     * @brief Hard ceiling on recursion depth
     *
     * A backstop, not a design parameter. Every real subdivision terminates on
     * lot_area_max long before this, and the ceiling exists for the pathological
     * input -- a piece whose cut keeps producing one piece of the original size
     * because the geometry is degenerate -- where the area constraint never
     * shrinks. Hitting it is counted in LotStats::depth_limit_hits, so it is
     * visible rather than silent.
     */
    uint32_t max_depth = 16;
};

/**
 * @brief How transfer_lot_ids() decides two lots are the same lot
 */
struct LotMatchConfig {
    /**
     * @brief Furthest a mapped centroid may sit from an old lot's centroid, in metres
     *
     * Only consulted for a mapped point that lands OUTSIDE every old lot, which
     * happens where the block grew. A point inside an old lot is matched to it
     * whatever the distance, because containment is the stronger evidence.
     */
    double max_centroid_drift = 30.0;
};

// ============================================================================
// Output
// ============================================================================

/**
 * @brief What the subdivision did, for logging and tests
 *
 * Carried for the same reason BlockStats is: the failure mode is silent. A block
 * that comes out as one enormous lot looks exactly like a block that was
 * correctly left alone, and only the rejection counts say which constraint
 * stopped the recursion.
 */
struct LotStats {
    size_t lots = 0;                 ///< Lots emitted. Equals LotSubdivision::lots.size().
    size_t corner_lots = 0;          ///< Of those, wedges taken off sharp corners
    size_t splits = 0;               ///< Cuts actually performed
    size_t split_attempts = 0;       ///< Nodes where a cut was tried
    size_t rejected_area = 0;        ///< Candidate cuts refused for a piece under lot_area_min
    size_t rejected_width = 0;       ///< Candidate cuts refused for a piece under lot_width_min
    size_t rejected_access = 0;      ///< Candidate cuts refused for landlocking a piece
    size_t rejected_degenerate = 0;  ///< Candidate cuts that did not divide the piece at all
    size_t dropped_pieces = 0;       ///< Pieces discarded for zero or negative area; see the note

    /**
     * @brief Lots discarded for sitting inside a hole in the block
     *
     * Non-zero only for a block that HAS holes. See the note on hole handling
     * above subdivide_block(); a lot that straddles a hole boundary is NOT
     * counted here and is NOT yet clipped.
     */
    size_t lots_in_holes = 0;
    size_t depth_limit_hits = 0;     ///< Nodes that stopped on LotParams::max_depth
    uint32_t max_depth_reached = 0;  ///< Deepest node emitted
};

/**
 * @brief Lots plus the evidence that they are all of them
 */
struct LotSubdivision {
    /**
     * @brief Lots in a deterministic emission order
     *
     * Corner lots first, then the recursion depth-first with each cut's pieces
     * ordered by their side of the cut line and their position along it. The order
     * is reproducible, but it is NOT identity -- see Lot::id.
     */
    std::vector<Lot> lots;

    LotStats stats;
};

// ============================================================================
// Subdivision
// ============================================================================

/**
 * @brief Cut @p block into lots
 *
 * Reads the block and nothing else, so it is deterministic and safe to run off
 * the main thread.
 *
 * A block of fewer than three ring points, or of no area, yields no lots. A block
 * smaller than LotParams::lot_area_min yields exactly ONE lot covering all of it,
 * never zero.
 *
 * The sum of the returned areas equals Block::area, to rounding. Land is never
 * discarded: every piece of every cut is either emitted or recursed into.
 *
 * @param block  A block from extract_blocks(), anticlockwise and positive in area
 * @param params Constraints, irregularity, seed and block key
 * @return Lots with stable ids, plus statistics
 */
/**
 * @brief Subdivide one block into lots
 *
 * ### Holes are handled only partially, on purpose
 *
 * A Block can have holes. `blocks.cpp` produces one whenever the face reaches a
 * loop across a cut edge -- an estate loop hanging off a single access road is
 * the common case, and the ring around it is genuinely not buildable land.
 *
 * The subdivision itself runs on the OUTER RING ALONE. Afterwards, any lot
 * whose centroid falls inside a hole is discarded and counted in
 * LotStats::lots_in_holes, which removes the visible artefact: lots floating
 * over a courtyard that is not part of the block.
 *
 * **A lot that STRADDLES a hole boundary is still wrong.** It keeps the part of
 * its ring that lies over the hole. Fixing that means clipping each lot against
 * the holes, which can split one lot into several or give it a hole of its own,
 * and Lot::ring is a single ring. That is tracked separately rather than bodged
 * in here -- a half-done clip that silently changes lot areas would be worse
 * than a documented gap, because every downstream area and frontage number
 * would move without anything saying so.
 *
 * @param block  Block to subdivide. Its holes are respected as described above.
 * @param params Subdivision parameters
 */
[[nodiscard]] LotSubdivision subdivide_block(const Block& block, const LotParams& params = {});

/**
 * @brief Run the recursion on one ring, as if it were a node of the tree
 *
 * The entry point subdivide_block() itself uses, exposed because it is the proof
 * that the seeding is per-node. Hand it a lot's ring, that lot's Lot::node_key
 * and that lot's Lot::depth and it reproduces that lot's subtree exactly -- same
 * geometry, same ids, same depths -- no matter what else was cut before it. A
 * shared random stream cannot do that, and the test that asserts it is the one
 * that would catch a regression back to one.
 *
 * @p start_depth is part of that contract and not a convenience. LotParams::max_depth
 * is measured from the ROOT of the block, so a node re-entered as if it were the
 * root would be handed the whole budget again rather than the remainder it had
 * left: at max_depth 3 a re-entry at a depth-1 child reproduced NONE of the full
 * run's lots. Leaving it at its default 0 is correct only for a ring that really
 * is a block root.
 *
 * Does NOT run the corner pass: corner lots belong to a block, not to a node
 * inside it, and cutting them again at every level would eat the block.
 *
 * @param ring        Anticlockwise ring in local metres, first point not repeated
 * @param ring_edges  Tag per ring segment, kNoBlockEdge for a segment that fronts
 *                    nothing. May be empty, which is read as all-kNoBlockEdge.
 * @param node_key    Address of this node in the tree; Lot::node_key, or
 *                    lot_root_key() for the whole block
 * @param params      Same constraints as subdivide_block()
 * @param start_depth Depth this node sits at in the tree it came from; Lot::depth,
 *                    or 0 for the root of a block
 * @return Lots of this subtree
 */
[[nodiscard]] LotSubdivision subdivide_ring(const std::vector<glm::dvec2>& ring,
                                            const std::vector<uint32_t>& ring_edges,
                                            uint64_t node_key,
                                            const LotParams& params = {},
                                            uint32_t start_depth = 0);

/**
 * @brief Key of the root node of a block's subdivision tree
 *
 * mix(block_key, seed). Exposed so a caller can re-enter subdivide_ring() at the
 * top of a block, and so a test can state the root key rather than infer it.
 */
[[nodiscard]] uint64_t lot_root_key(const LotParams& params);

/**
 * @brief The id a lot at @p node_key is given
 *
 * The whole of the provisional identity rule, in one function, so that the
 * contract "id is a pure function of the tree address" is checkable rather than
 * merely claimed.
 */
[[nodiscard]] LotId lot_id_for_key(uint64_t node_key);

/**
 * @brief Derive a stable block key from the OSM ways bounding @p block
 *
 * The sorted, de-duplicated set of GraphEdge::source_way over Block::edges, mixed
 * down to 64 bits. Sorted and de-duplicated so the key depends on WHICH ways
 * bound the block and not on the order the face traversal happened to walk them,
 * which changes when anything upstream of the graph changes.
 *
 * This survives every edit that moves geometry, which is the edit that happens
 * constantly. It does NOT survive an edit that adds or removes a bounding way --
 * and it should not, because that block is genuinely a different block. That case
 * is what transfer_lot_ids() is for.
 *
 * @param block Block whose edges index into @p graph
 * @param graph The graph the block was extracted from
 * @return A non-zero key, or a fixed non-zero constant for a block with no edges
 */
[[nodiscard]] uint64_t block_key_from_ways(const Block& block, const RoadGraph& graph);

// ============================================================================
// Barycentric Transfer
// ============================================================================

/**
 * @brief Floater mean-value coordinates of @p point with respect to @p ring
 *
 * Returns one weight per ring vertex, summing to 1, such that
 * `sum(w[i] * ring[i]) == point` for any affine ring. Defined for a ring of any
 * shape, convex or not, unlike triangle barycentrics.
 *
 * Computed through the half-angle tangent identity
 * `tan(a/2) = (|u||v| - dot(u,v)) / cross(u,v)`, so there is no trigonometry in
 * here at all and the result is bit-identical across platforms.
 *
 * A point on a vertex returns a one-hot vector; a point on an edge returns the
 * linear interpolation of that edge's two vertices; a point collinear with an
 * edge but off its span contributes nothing for that edge. All three are limits
 * of the formula that the formula itself computes as 0/0, and the third one is
 * reached by any non-convex ring, so it is not a curiosity: missing it returned
 * an all-NaN weight vector for ordinary interior points of a block with a reflex
 * corner.
 *
 * Weights sum to 1 but are NOT all non-negative. A point outside the kernel of a
 * non-convex ring gets negative weights, which is correct for Floater
 * coordinates and does not disturb linear precision, but means a caller cannot
 * read the result as a convex combination.
 *
 * @param ring  Ring in local metres, first point not repeated, at least 3 points
 * @param point Point to express, normally inside the ring
 * @return Weights parallel to @p ring, or empty for a ring of under 3 points
 */
[[nodiscard]] std::vector<double> mean_value_coordinates(const std::vector<glm::dvec2>& ring,
                                                         const glm::dvec2& point);

/**
 * @brief Carry a point from one ring's frame into another's
 *
 * Writes @p point in mean-value coordinates of @p from_ring and reads them back
 * out against @p to_ring. This is how a lot follows a block that was deformed
 * under it.
 *
 * The two rings must have the same vertex count, which is the case that matters:
 * dragging a street node moves a block's vertices without changing how many there
 * are. When the counts differ the point is returned unchanged, so a caller falls
 * back to plain proximity rather than to a meaningless coordinate.
 *
 * @param from_ring Ring @p point is currently expressed against
 * @param to_ring   Ring to express it in, same vertex count and correspondence
 * @param point     Point in local metres
 * @return The corresponding point in @p to_ring's frame
 */
[[nodiscard]] glm::dvec2 map_point_between_rings(const std::vector<glm::dvec2>& from_ring,
                                                 const std::vector<glm::dvec2>& to_ring,
                                                 const glm::dvec2& point);

/**
 * @brief Give each lot in @p current the id of the lot it replaces
 *
 * The third and last identity mechanism, for when the subdivision tree changed
 * shape and the provisional ids no longer line up. Each current lot's centroid is
 * carried back through the block deformation with map_point_between_rings() and
 * matched against the previous lots: containment first, then centroid distance
 * within LotMatchConfig::max_centroid_drift.
 *
 * The matching is one-to-one and greedy over a fully ordered candidate list, so
 * it is deterministic: two lots that both land in one old lot cannot both inherit
 * it, and which one does is decided by distance with the lot indices as the
 * tie-break, never by iteration order.
 *
 * A current lot with no match keeps its provisional id. That is the right answer
 * -- it is land that was not previously a lot of its own -- and it is why this
 * returns a count rather than asserting that everything matched.
 *
 * @param previous      Lots from the earlier subdivision
 * @param previous_ring Block ring those lots were cut from
 * @param current_ring  Block ring @p current was cut from
 * @param current       Lots to re-id, modified in place
 * @param config        Matching tolerance
 * @return How many lots in @p current inherited an id
 */
size_t transfer_lot_ids(const std::vector<Lot>& previous,
                        const std::vector<glm::dvec2>& previous_ring,
                        const std::vector<glm::dvec2>& current_ring,
                        std::vector<Lot>& current,
                        const LotMatchConfig& config = {});

} // namespace stratum::osm::road
