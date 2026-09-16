// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file lots_offset.hpp
 * @brief Offset and skeleton lot subdivision (C3), over the same Lot as C2
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this exists, next to lots.hpp rather than inside it
 *
 * `lots.hpp` cuts a block by recursive oriented-bounding-box halving. That is the
 * right default and the wrong shape for two common cases:
 *
 *   - **A deep block.** Halving a 200 x 60 block four times gives parcels in the
 *     middle with no street on any side. `force_street_access` then refuses the
 *     cut and the interior stays one enormous lot. What a real deep block has is a
 *     RIM of street-fronting parcels and a yard behind them, and that is the
 *     OFFSET subdivision.
 *   - **A block that is not a rectangle.** A triangular block, or one on a bend,
 *     has an OBB that shares almost none of its boundary, so the halving cuts
 *     across the streets rather than along them and every parcel fronts the road
 *     at an angle. What it should have is parcels whose sides are perpendicular to
 *     the street they front, and that is the SKELETON subdivision.
 *
 * Both are listed in lots.hpp as C3 and deliberately not stubbed there, because a
 * half-built skeleton that silently falls back to OBB is worse than not having the
 * mode. They live here, and they are stubbed nowhere: each either produces lots or
 * reports that it could not (`StripStats::skeleton_complete`), and the caller
 * chooses the fallback.
 *
 * ### The same Lot, the same ids, the same promises
 *
 * Everything this file emits is an `osm::road::Lot` with an `osm::road::LotId`,
 * counted in an `osm::road::LotStats`, and cut under an `osm::road::LotParams`.
 * There is no parallel lot type and there is not going to be one: a rule targets a
 * lot, an attribute is keyed on a lot id, and C4 classifies frontage from
 * `Lot::ring_edges`. A second lot type would mean a second of each of those.
 *
 * The C7 commitments in lots.hpp hold here unchanged, by the same mechanisms:
 *
 *   - **Determinism.** Same block, same seed, same lots, to the bit. Every draw is
 *     `mix2(node_key, salt)` against a key derived from the lot's ADDRESS -- here
 *     the contour edge it fronts and its index along that edge -- never from a
 *     shared stream, so adding a lot at one end of a block does not move the ones
 *     at the other. No hash container is read in iteration order. No transcendental
 *     function touches a coordinate: the only libm that reaches one is sqrt,
 *     through glm::length and glm::normalize, and ceil, floor and llround, which
 *     are exact. Every boolean underneath is Clipper2's integer arithmetic.
 *   - **Identity.** `lot_id_for_key()` from lots.hpp, unchanged, so an id from this
 *     file and an id from C2 live in the same space and cannot collide by
 *     construction -- the salts here are disjoint from the ones there.
 *     `transfer_lot_ids()` works on these lots exactly as it does on C2's.
 *
 * ### Both modes are built on the straight skeleton
 *
 * `geometry/straight_skeleton.hpp` cuts a polygon into one FACE per boundary edge:
 * the face of edge e is precisely the land whose nearest street is e. That is the
 * answer to "which parcel fronts which road" for both modes at once:
 *
 *   - **Skeleton subdivision** cuts each face ACROSS, perpendicular to its street.
 *     Every lot therefore touches the street, and its sides are perpendicular to
 *     it. Nothing else produces that shape.
 *   - **Offset subdivision** takes the part of each face within `offset_width` of
 *     its street. The union of those parts is the band, the rest is the yard, and
 *     because the faces TILE the block, the band pieces tile the band -- no gaps at
 *     the corners and no overlaps where two streets converge.
 *
 * Doing the offset mode by mitring the ring instead would have been shorter and is
 * wrong wherever the naive offset self-intersects: a block arm narrower than twice
 * the offset width, which is every lane and every triangular tip. The mitred quads
 * there overlap, and overlapping lots are the failure this whole area is trying to
 * avoid.
 *
 * Clipper2 does every boolean: the inward offset itself, the band, the slab cuts
 * and the hole subtraction. It is integer arithmetic on a fixed scale, so it gives
 * the same answer on every platform, which a floating-point clipper would not.
 *
 * ### Holes are handled BEFORE the lots are cut, not after
 *
 * `lots.cpp` subdivides the outer ring and then discards any lot that came out
 * wholly inside a hole, and documents that a lot straddling a hole boundary keeps
 * the part of its ring over the hole. This file subtracts `Block::holes` with
 * Clipper2 before anything is cut, so no lot ever covers a hole at all -- a
 * straddling lot comes out with the hole bitten out of it, which is the right
 * shape rather than a documented gap.
 *
 * The wholly-inside-a-hole filter still runs afterwards, with lots.cpp's exact
 * test, and normally discards nothing. It is kept as the net for the one case the
 * booleans cannot fix: a hole that the skeleton's own faces spanned, since
 * `compute_straight_skeleton()` takes one ring and knows nothing about holes.
 * `LotStats::lots_in_holes` therefore means the same thing here as there.
 *
 * ### What this does NOT do
 *
 *   - **No frontage classification.** Which street a lot faces and whether that
 *     frontage is primary is C4. This file records the block edge behind every lot
 *     segment in `Lot::ring_edges`, exactly as C2 does.
 *   - **No fallback.** If the skeleton cannot be computed for a block, no lots come
 *     back and `StripStats::skeleton_complete` is false. Silently falling back to
 *     OBB would make the mode a lie: an author who chose skeleton subdivision and
 *     got OBB parcels has no way to tell.
 *   - **No scene mutation.** Pure functions of their arguments, like
 *     `subdivide_block()` and `extract_blocks()`. Putting the result into a
 *     document is an edit and goes through `scene::CommandStack`.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/blocks.hpp"
#include "osm/road/lots.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tunables of the offset subdivision
 *
 * The CityEngine name of each parameter is in brackets, on the same one-to-one
 * rule LotParams follows.
 */
struct OffsetLotParams {
    /**
     * @brief Depth of the street-fronting band, in metres [offsetWidth]
     *
     * Measured as a true distance from the street, not along a mitre, so the band
     * has a constant depth all the way round including at the corners. A block
     * narrower than twice this has no yard left and comes out entirely as band,
     * which is correct: everything in it is within `offset_width` of a street.
     *
     * Zero or negative means no band at all, and then the whole block goes down
     * the interior path, which is the OBB recursion. That is a legitimate way to
     * ask for C2 behaviour through this entry point and is not treated as an
     * error.
     */
    double offset_width = 25.0;

    /**
     * @brief The C2 parameters, reused whole
     *
     * `block_key` and `seed` root the identity of every lot this file emits;
     * `lot_area_min`, `lot_area_max` and `lot_width_min` size the band's lots and
     * then drive the recursion on the yard behind them.
     *
     * `corner_angle_max_deg` and `corner_width` are NOT consulted in THIS mode. The
     * corner pass in lots.cpp takes a wedge off a sharp block corner because the
     * recursion would otherwise leave one; a band already follows the corner round,
     * so the parcel there is a band lot bent through the corner and the wedge never
     * appears. The skeleton mode does use `corner_width`; see
     * SkeletonLotParams::corner_alignment.
     */
    LotParams lot;
};

/**
 * @brief Tunables of the skeleton subdivision
 */
struct SkeletonLotParams {
    /**
     * @brief How deep a street lot reaches, as a fraction of its face [shallowLotFrac]
     *
     * A skeleton face runs from its street all the way back to the middle of the
     * block. At 1 the lots do too, which is the classic skeleton subdivision and
     * what a terrace looks like. Below 1 they stop short at that fraction of the
     * face's own depth and the land behind becomes BACKLAND, emitted as its own
     * lots and recursed into like any other landlocked piece.
     *
     * Per face rather than per block, because the two sides of a wedge-shaped
     * block are not the same depth and a single block-wide depth would leave the
     * shallow side with no backland at all and the deep side with most of its
     * land in it.
     *
     * Clamped to [0, 1]. At 0 there are no street lots, only backland, which is
     * degenerate but harmless and is not treated as an error.
     */
    double shallow_lot_frac = 1.0;

    /**
     * @brief How much of a block corner becomes its own lot, 0..1 [cornerAlignment]
     *
     * At 0 a street's run of lots reaches all the way to the block corner, bounded
     * there by the skeleton's own bisector. That is geometrically exact and it
     * makes the end lot a wedge that narrows towards the back -- a shape nothing
     * fits in, and the reason lots.cpp has a corner pass at all.
     *
     * Above 0, each of the two streets at a corner gives up its last
     * `corner_alignment * LotParams::corner_width` metres, and the two give-ups
     * become ONE lot fronting both streets, with `Lot::is_corner_lot` set. Every
     * lot in the runs either side of it is then square to its own street.
     *
     * The corner cannot be squared by clipping one face: at a convex corner the two
     * faces already meet along the bisector, and making one street's end lot
     * rectangular would take land from the other street rather than trim its own.
     * See corner_widths() in the implementation.
     *
     * Only CONVEX corners are eligible, which is lots.cpp's rule too -- a reflex
     * corner has no wedge to remove -- and the width is clamped to half of each
     * incident street, so two corners at the ends of a short street can never cut
     * overlapping lots out of it.
     */
    double corner_alignment = 1.0;

    /**
     * @brief Drop a lot vertex this far or less from the line of its neighbours, in metres
     *
     * The skeleton ridge down the middle of a block carries a node for every
     * polyline point of both kerbs that face it, so the lots either side of it
     * arrive with a long run of nearly-collinear vertices along their backs. This
     * is what removes them. Zero, the default, is off.
     *
     * ### What it will and will not move
     *
     * The decision is made on the SHARED boundary, once, and applied to both lots
     * that use it -- see simplify_shared_boundaries() in the implementation. Three
     * things follow, and they are the contract:
     *
     *   - **The block outline never moves**, at any tolerance. A vertex of
     *     `Block::ring` is not a candidate, so the lots still sum to the area of
     *     the block exactly.
     *   - **No gap and no overlap ever opens between two lots**, at any tolerance,
     *     because the boundary they share is one boundary with one vertex list.
     *   - **Land does move between two lots** whose shared boundary is
     *     straightened, by up to the tolerance times the length of the run. That is
     *     the whole effect of the parameter and it is what to size it against.
     *
     * So this is not the parameter that can lose land. An earlier version was: it
     * simplified each lot separately, and at 0.5 m on a block with a curved kerb it
     * deleted 0.38% of the block and left holes between lots. If a tolerance here
     * ever changes `StripStats::street_area + ::interior_area` away from the
     * block's area, that defect is back.
     *
     * ### It reaches less far than it looks, and that is not a bug
     *
     * The rule above leaves only one thing to simplify: a bisector arc between two
     * adjacent faces that the wavefront put several nodes on, where both lots that
     * use it keep the whole chain. On an axis-aligned rectangular block there is no
     * such chain at all and this removes NOTHING at any tolerance; on irregular
     * blocks it fires on a few per cent of subdivisions and takes a couple of
     * vertices.
     *
     * That is the honest reach, and it is much less than the obvious reading of
     * "the streets have a lot of vertices". A lot's kerb is a piece of ONE contour
     * edge -- the skeleton gives each ring segment its own face -- so a lot never
     * inherits a run of street polyline points in the first place. What a densely
     * densified kerb produces is many small FACES and therefore many small lots,
     * which is a real problem and is not this parameter's: fixing it means
     * simplifying `Block::ring` before the skeleton is computed, which renumbers
     * every face and so moves every lot id in the block.
     *
     * `StripStats::simplified_vertices` reports how many boundary vertices went.
     */
    double simplify = 0.0;

    /// The C2 parameters, reused whole. See OffsetLotParams::lot.
    LotParams lot;
};

// ============================================================================
// Output
// ============================================================================

/**
 * @brief What the strip subdivision did, ALONGSIDE the LotStats it also fills
 *
 * Deliberately not a replacement for LotStats: the lot counts, the rejection
 * counts and `lots_in_holes` are all in there and mean exactly what they mean in
 * C2. These are the counts that only a strip subdivision has, and they are here
 * because the failure mode is silent -- a block that came back as one lot looks
 * the same whether the skeleton refused it or the constraints did.
 */
struct StripStats {
    /**
     * @brief False when the straight skeleton could not be computed for this block
     *
     * When it is false NO lots are emitted. Both modes are built on the face
     * decomposition, so without it there is nothing to cut, and inventing an OBB
     * subdivision instead would hide the refusal from the one caller that can do
     * something about it.
     */
    bool skeleton_complete = false;

    size_t faces = 0;            ///< Skeleton faces the block was cut into

    /**
     * @brief Street-fronting LOTS emitted, not pieces offered
     *
     * A piece with no area is dropped rather than made into a lot, so a count of
     * pieces is a count of attempts. Counting what came out instead makes
     * "every street lot fronts a street" an assertion a test can make against this
     * number rather than against the shape of one particular block.
     */
    size_t street_pieces = 0;

    /// Corner LOTS emitted; see SkeletonLotParams::corner_alignment
    size_t corner_pieces = 0;

    /// Pieces behind a shallow street lot, counted BEFORE the C2 recursion cuts them
    size_t backland_pieces = 0;

    /// Yard regions behind the band, counted BEFORE the C2 recursion cuts them
    size_t interior_pieces = 0;

    size_t empty_clips = 0;      ///< Clips that came back with no area; normal at a tight corner

    /// Boundary vertices deleted by SkeletonLotParams::simplify; zero when it is off
    size_t simplified_vertices = 0;

    /**
     * @brief Land in street-fronting lots, and land behind them, in square metres
     *
     * Summed over the emitted `Lot::area`, so the two cannot describe different
     * land from the lots themselves. They add up to the block's area less its
     * holes, which is the check worth making: a mode that lost land between the
     * pieces and the lots would show it here and nowhere else.
     */
    double street_area = 0.0;
    double interior_area = 0.0;
};

/**
 * @brief Lots plus the evidence that they are all of them
 *
 * `subdivision` is the C2 type verbatim, so a caller that already handles
 * `subdivide_block()` handles this with no new code at all.
 */
struct StripSubdivision {
    LotSubdivision subdivision;
    StripStats stats;
};

// ============================================================================
// Subdivision
// ============================================================================

/**
 * @brief Cut @p block into a street-fronting band and the yard behind it
 *
 * The band is every point of the block within `OffsetLotParams::offset_width` of a
 * street, cut into lots one street at a time. The yard is what is left, subdivided
 * by the C2 recursion -- it has no frontage, so `force_street_access` leaves it
 * whole unless the caller turned that off, which is the correct shape for the
 * middle of a deep block.
 *
 * Reads the block and nothing else, so it is deterministic and safe to run off the
 * main thread.
 *
 * @param block  A block from extract_blocks(), anticlockwise and positive in area
 * @param params Band depth plus the C2 constraints, irregularity, seed and block key
 * @return Lots with stable ids, the C2 statistics, and the strip statistics
 */
[[nodiscard]] StripSubdivision subdivide_block_offset(const Block& block,
                                                      const OffsetLotParams& params = {});

/**
 * @brief Cut @p block into lots that front a street and are square to it
 *
 * One run of lots per street: each skeleton face is cut across, perpendicular to
 * its own street, into as many lots as the area and width constraints allow. Every
 * lot therefore reaches the street it fronts, which is what
 * `LotParams::force_street_access` can only achieve by refusing to cut.
 *
 * Reads the block and nothing else, so it is deterministic and safe to run off the
 * main thread.
 *
 * @param block  A block from extract_blocks(), anticlockwise and positive in area
 * @param params Depth, corner squaring, simplification, and the C2 constraints
 * @return Lots with stable ids, the C2 statistics, and the strip statistics
 */
[[nodiscard]] StripSubdivision subdivide_block_skeleton(const Block& block,
                                                        const SkeletonLotParams& params = {});

// ============================================================================
// Pieces, for tests and for callers that want the decomposition itself
// ============================================================================

/**
 * @brief The land of @p block within @p width of a street, one entry per street
 *
 * The band of subdivide_block_offset() before any lot is cut out of it. Exposed
 * because it is the thing that must tile: `band[i]` is the part of the block whose
 * nearest boundary edge is `block.ring[i] -> block.ring[i + 1]` and which is no
 * further than @p width from it, so the entries do not overlap and their areas sum
 * to the area of the whole band. A test can assert that without knowing what the
 * lots inside it should look like.
 *
 * An entry is EMPTY where that street's band has no area -- a zero-length ring
 * segment, or a street the block only touches at a point.
 *
 * @param block Block to band. Its holes are subtracted.
 * @param width Band depth in metres. Zero or negative gives all-empty entries.
 * @param out_interior Set to the yard regions behind the band; may be empty
 * @return One entry per `block.ring` segment, or empty when the skeleton refused
 */
[[nodiscard]] std::vector<std::vector<std::vector<glm::dvec2>>> block_offset_band(
    const Block& block, double width,
    std::vector<std::vector<glm::dvec2>>* out_interior = nullptr);

} // namespace stratum::osm::road
