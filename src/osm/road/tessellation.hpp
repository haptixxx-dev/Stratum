/**
 * @file tessellation.hpp
 * @brief Fewer, larger triangles: station decimation and coplanar quad merging
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The biggest single lever on total road geometry, because it shrinks LOD level
 * 0, every LOD level below it, the collision mesh and the export all at once --
 * unlike simplification, which only ever produces a cheaper COPY of geometry that
 * still has to be built and still has to be stored.
 *
 * ### The defect
 *
 * `build_centerline()` resamples at curvature-adaptive spacing bounded above by
 * `ResampleConfig::max_spacing`, which is 8 m. That bound is unconditional, so a
 * dead-straight 800 m dual carriageway gets 100 stations. A kerbed profile is
 * around 16 vertex columns wide, so that straight costs roughly 1,600 vertices
 * and 3,200 triangles to describe a shape that is exactly a flat ribbon and needs
 * two stations to describe exactly.
 *
 * Measured on the Lucan extract: 40,649 pieces, 3,819,979 vertices, 3,194,115
 * triangles. Most of those triangles are on straights and carry no information.
 *
 * ### The two passes
 *
 * 1. **Longitudinal**, select_stations(): drop a station when the chord that
 *    would replace it does not depart from the curve by more than
 *    TessellationConfig::max_chord_deviation. This runs BEFORE extrusion, so the
 *    triangles are never built in the first place.
 * 2. **Lateral**, merge_coplanar_quads(): after extrusion, merge adjacent quads
 *    that are coplanar and share a MaterialKey. This catches what the first pass
 *    structurally cannot -- four lanes of the same asphalt at the same height are
 *    four strips and therefore four columns of quads, whatever the station
 *    spacing is.
 *
 * The two are independent and either can be run alone.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "renderer/mesh.hpp"

#include <cstddef>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Budgets governing how much geometry may be removed and where
 */
struct TessellationConfig {
    /**
     * @brief Deviation budget for a merged span, metres
     *
     * Merge consecutive stations while the departure of every dropped station
     * from the straight chord replacing them stays under this. The measure is the
     * maximum distance from a DROPPED station to the SEGMENT joining the two kept
     * stations that bracket it -- a Douglas-Peucker error.
     *
     * It is a distance to the segment rather than to the infinite line through
     * it, because on a hairpin resampled at ResampleConfig::min_spacing a station
     * can sit behind its own chord: its perpendicular distance to the line is
     * small while it is nowhere near the chord.
     *
     * 0.03 m is roughly a tenth of the width of a lane marking, so a road that
     * moves by less than this over a merged span is straight as far as anything
     * the player can see is concerned.
     *
     * @note The measure is THREE-DIMENSIONAL whenever the caller has solved
     *       station heights to supply -- see the @p station_heights parameter of
     *       select_stations(). A road can be dead straight in plan and still
     *       crest a hill, and a plan-view test merges that crest away: the chord
     *       is flat, every dropped station is directly above it, and the pass
     *       hands back a flattened ribbon that undoes the elevation solve. The
     *       2D-to-world mapping `(x, y) -> (x, h, -y)` is an isometry, so the
     *       distance measured in (x, y, h) is the distance the player sees.
     *
     * @note Without heights the measure degrades to the plan-view one, which is
     *       correct only for a caller that has not solved elevation yet.
     *       RoadNetworkBuilder runs this pass after the elevation solve for
     *       exactly that reason.
     */
    double max_chord_deviation = 0.03;

    /**
     * @brief Never merge across a station carrying a feature that needs its column
     *
     * A crossing, a stop line, a junction trim point, a bridge span end and a
     * tunnel portal all attach to a specific arclength, and every one of them
     * breaks if the vertex column at that arclength is gone. The caller supplies
     * which stations those are; see the @p feature_stations parameter of
     * select_stations().
     *
     * Turning this off is a debugging switch. It will detach paint from the
     * stations it was placed against.
     */
    bool preserve_feature_stations = true;

    /**
     * @brief Longest span one merged band may cover, metres
     *
     * Without a cap a motorway becomes two triangles, and three things break at
     * once:
     *
     * - **Vertex-lit shading** has nothing to interpolate between, so a long
     *   sunlit ribbon goes flat.
     * - **Frustum culling granularity** collapses: a piece whose triangles each
     *   span 800 m is either fully in or fully out, and near the camera it is
     *   always in.
     * - **Terrain conformance** is lost. A carve follows the ground under the
     *   road; a 800 m chord does not, and the road floats or buries at the
     *   midpoint even though its two ends are correct.
     *
     * 120 m keeps a band under a typical LOD cell and under a typical carve
     * sample spacing, and still reduces an 800 m straight from 100 stations to
     * 8.
     *
     * @note BOTH passes read this. merge_coplanar_quads() applies it as a cap on
     *       how far a merged quad may extend ALONG THE DIRECTION OF THE MERGE,
     *       because its along-the-road merges would otherwise chain without limit
     *       and undo the cap select_stations() just honoured. Measured along the
     *       merge direction rather than over the whole quad, so a band already at
     *       the cap can still be widened laterally.
     */
    double max_span_length = 120.0;

    /**
     * @brief Run the lateral pass, merge_coplanar_quads()
     *
     * Independent of the longitudinal pass: either may be run without the other.
     * max_chord_deviation and preserve_feature_stations are read by
     * select_stations() alone; max_span_length is read by both.
     */
    bool merge_coplanar_strips = true;

    /**
     * @brief Coplanarity threshold as a normal dot product
     *
     * Two quads are coplanar when the dot product of their unit face normals is
     * at least this AND the second's vertices lie within the first's plane to
     * within a distance derived from the same bound. 0.999 is about 2.5 degrees,
     * which passes the cross-fall of a carriageway (typically 2%, or 1.1
     * degrees) and refuses a kerb face.
     *
     * @note This alone is not sufficient to merge. A shared MaterialKey is also
     *       required, and so is UV continuity; see merge_coplanar_quads().
     */
    double coplanar_normal_epsilon = 0.999;
};

// ============================================================================
// Longitudinal: station selection
// ============================================================================

/**
 * @brief Choose the stations worth keeping, given the deviation budget
 *
 * A pure query. It does not modify @p cl; it returns indices into
 * `cl.stations`, strictly ascending, with no duplicates.
 *
 * ### What is always kept
 *
 * - **The first and the last station.** An edge's ends meet its neighbours' ends
 *   exactly, and a junction's arm mouth is placed at the end station. Moving
 *   either would open a gap at a junction.
 * - **Every station flagged in @p feature_stations**, when
 *   TessellationConfig::preserve_feature_stations is set.
 * - **Both halves of a bevel pair.** A bevel is represented as two stations
 *   sharing one position and one arclength, carrying the incoming and outgoing
 *   normals respectively (see Station::is_bevel). They are a unit: dropping one
 *   turns the joint into a mitre with the wrong normal, and dropping neither but
 *   dropping their neighbours is fine. If either member of a pair is kept, both
 *   are.
 * - **Any station where the sign of Station::curvature changes.** An inflection
 *   is where the road stops turning one way and starts turning the other, and
 *   the chord test can pass straight through one -- an S-bend whose two halves
 *   cancel measures as almost straight while looking nothing like a straight.
 *
 * ### What the caller must do with the result
 *
 * The indices are not a centerline. Apply them with apply_station_selection(),
 * which handles the bookkeeping that a naive gather would get wrong -- see that
 * function.
 *
 * @param cl               Centerline to decimate. A centerline with fewer than 3
 *                         stations is returned whole.
 * @param cfg              Deviation budget and span cap
 * @param feature_stations Optional mask, parallel to `cl.stations`, marking
 *                         stations that must survive. Ignored when
 *                         TessellationConfig::preserve_feature_stations is false.
 *                         A mask shorter than `cl.stations` protects only the
 *                         prefix it covers; a longer one has its tail ignored.
 *                         May be null, meaning no station is protected.
 * @param station_heights  Optional solved world Y per station, parallel to
 *                         `cl.stations`, in the same form as
 *                         EdgeElevation::station_heights and
 *                         CorridorConfig::station_heights. When supplied, the
 *                         deviation test is three-dimensional and a vertical
 *                         crest is preserved; when null the test is plan-view
 *                         only and a crest that is straight in plan WILL be
 *                         flattened. A vector of any other size is treated as
 *                         absent, matching how CorridorConfig degrades a
 *                         mis-sized elevation solve to a flat road rather than to
 *                         mangled geometry.
 * @return Ascending indices into `cl.stations`. Always contains 0 and
 *         `cl.stations.size() - 1` for a non-empty centerline. Empty only when
 *         `cl.stations` is empty.
 */
[[nodiscard]] std::vector<size_t> select_stations(const Centerline& cl,
                                                  const TessellationConfig& cfg,
                                                  const std::vector<bool>* feature_stations = nullptr,
                                                  const std::vector<float>* station_heights = nullptr);

/**
 * @brief Build the decimated centerline named by a station selection
 *
 * Declared here rather than left to each caller, because a plain gather of the
 * selected stations is WRONG in three ways and every caller would have to get all
 * three right:
 *
 * 1. **The frame must be rebuilt.** `Station::normal` is the miter bisector of
 *    the two segments meeting at that station, and `Station::miter_scale` is
 *    `1 / cos(theta / 2)` for that joint. Dropping the stations between two kept
 *    ones changes both angles. Carrying the old frame across pinches the ribbon
 *    at exactly the joints the merge created.
 * 2. **The fold bounds must be tightened, not copied.**
 *    `Station::lateral_min` and `lateral_max` bound the offset at which the
 *    ribbon folds through itself. A kept station now stands for the whole span it
 *    absorbed, so its bounds become the TIGHTEST over that span --
 *    `max` of every `lateral_min` and `min` of every `lateral_max` across the
 *    dropped run on each side. Copying the kept station's own bounds would let a
 *    wide profile fold through a tight bend that no longer has a station in it.
 * 3. **Arclength must be carried, not recomputed.** `Station::arclength` is the
 *    only input to the V texture coordinate and the key that markings, crossings,
 *    trims and carve requests are all expressed in. It keeps the value it had on
 *    the ORIGINAL centerline, so a decimated ribbon has the same texture
 *    placement as an undecimated one and every arclength held elsewhere stays
 *    valid. This means the decimated centerline's arclengths are NOT the sum of
 *    its own chord lengths, exactly as slice() already leaves them.
 *
 * `Station::curvature` is likewise carried from the kept station, not
 * re-derived: it describes the road, and the road did not change.
 *
 * A fourth thing the caller would get wrong is that the tightened bounds are not
 * the whole answer. The bands the merge CREATED can fold too, so the same fold
 * algebra build_centerline() runs is re-run over the decimated stations and
 * intersected with the carried bounds. Both only ever tighten, so the result is
 * the intersection of "what the dropped bands bounded" and "what the new bands
 * bound".
 *
 * Bevel pairs are carried verbatim, frame and all. A bevel's two stations carry
 * the incoming and the outgoing leg normals with miter_scale exactly 1; deriving
 * a bisector for them would destroy the wedge they exist to describe.
 *
 * ### The two END stations are carried verbatim as well
 *
 * The first and the last station of the result are bit-identical to
 * `cl.stations[keep.front()]` and `cl.stations[keep.back()]`: frame, miter_scale
 * and lateral bounds included. Neither the frame rebuild nor either fold pass
 * touches them.
 *
 * This is a hard requirement, not a nicety. An edge's end station is the one the
 * junction solver already consumed -- arm_end() slices the same centerline at the
 * same cut and runs `stations.front()` through offset_point() to place the
 * junction polygon's arm mouth and the curb ring's arm reach -- and
 * build_corridor() emits the ribbon's first column through that identical call.
 * They register only while the station is unchanged. Because a trim almost never
 * lands on an existing station, slice() synthesises that station by interpolating
 * the miter frame across the band it cut; rebuilding the frame from the decimated
 * chord rotates it, and charging an absorbed run's fold bounds to it clamps the
 * same column inboard. Either opens a wedge between the ribbon and its junction
 * mouth at every decimated arm.
 *
 * The cost is that a bend absorbed into the FIRST or LAST band is not represented
 * in that band's end frame or fold bound. That band is by construction inside the
 * deviation budget, and the alternative is a visible gap at a junction.
 *
 * @param cl          Source centerline
 * @param keep        Ascending indices from select_stations(). Indices out of
 *                    range are ignored; a non-ascending or duplicated input is
 *                    undefined.
 * @param miter_limit The ResampleConfig::miter_limit the centerline was built
 *                    with. A joint sharper than this is a bevel, and a bevel is
 *                    two stations -- but this pass removes stations and never
 *                    inserts them, so a rebuilt joint that lands past the limit
 *                    is CLAMPED to it instead. A selection that respects
 *                    TessellationConfig::max_chord_deviation cannot produce one:
 *                    the stations either side of a joint that sharp sit metres
 *                    off any chord through it, so no span ever absorbs them.
 * @return The decimated centerline. Invalid (fewer than 2 stations) when @p keep
 *         names fewer than 2 usable stations, which the caller must treat exactly
 *         as it treats an invalid build_centerline() result: skip the edge.
 */
[[nodiscard]] Centerline apply_station_selection(const Centerline& cl,
                                                 const std::vector<size_t>& keep,
                                                 double miter_limit = 4.0);

// ============================================================================
// Lateral: coplanar quad merging
// ============================================================================

/**
 * @brief Merge coplanar adjacent quads sharing a MaterialKey into larger ones
 *
 * A post-extrusion pass over a built corridor mesh. Where the longitudinal pass
 * removes vertex COLUMNS, this removes the seams BETWEEN columns of strips: four
 * lanes of the same asphalt at the same height are four Strips and therefore four
 * quads wide, and they are one rectangle.
 *
 * ### What may be merged
 *
 * Two quads merge only when ALL of the following hold:
 *
 * - Each is really a quad: two triangles sharing a diagonal whose union is a
 *   convex quadrilateral, flat to within
 *   TessellationConfig::coplanar_normal_epsilon. A merge re-triangulates across
 *   the other diagonal, and doing that to a warped or reflex pair would move the
 *   surface.
 * - They share an entire side -- the same two vertex POSITIONS, in opposite
 *   winding order. Positions, not vertex indices: adjacent strips of a corridor
 *   never share a vertex, because the frozen UV convention gives the boundary two
 *   different U values. Two quads that merely touch at a corner do not qualify.
 * - Their unit face normals agree to within
 *   TessellationConfig::coplanar_normal_epsilon. With both already flat, that
 *   makes the union flat.
 * - They belong to the same SubMesh range, and therefore to the same MaterialKey.
 *   Slot alone is not enough: two Asphalt ranges with different variants are two
 *   materials and merging them would put cobblestone under an asphalt texture.
 * - Each of the two coincident seam vertices lies STRICTLY INSIDE the side that
 *   absorbs it, so dropping it neither gains nor loses area. The merged quad's
 *   area is checked against the sum of the two it replaces as well.
 * - Their UVs are affinely continuous across the shared side, or can be MADE so;
 *   see below.
 * - No seam vertex carries a colour, normal or tangent that the merge would
 *   discard and could not reproduce from the two corners of the side absorbing
 *   it.
 * - The merged quad does not extend further than
 *   TessellationConfig::max_span_length ALONG THE DIRECTION OF THE MERGE, which
 *   is the direction across the shared side, in the quad's own plane.
 *
 * ### The UV argument, and why it needs a whole component
 *
 * The frozen convention (corridor.hpp) is metres-based: `U = lateral metres
 * within the strip / tile_u`, `V = arclength metres / tile_v`. Both are affine in
 * position wherever the geometry the merge creates is itself affine, which the
 * seam-collinearity test establishes -- so a merged quad's corner UVs are
 * SOLVABLE rather than guessable. Writing s_BC and s_DA for where the two seam
 * vertices sit along the sides that absorb them, the merged mapping reproduces
 * the near quad exactly when
 *
 * @code
 *     uv(seam_1) == lerp(uv(B), uv(C), s_BC)
 *     uv(seam_2) == lerp(uv(D), uv(A), s_DA)
 * @endcode
 *
 * and the far quad up to one constant offset `delta`, read straight off the seam
 * as the difference between the two coincident seam vertices' UVs. Both seam
 * pairs must agree on `delta`, and `delta` must be zero in V: a V shift would
 * slide the far quad along the road and break the arclength placement that
 * markings, crossings and trims are all expressed against. All of this is
 * CHECKED against the actual vertices rather than assumed -- a fold clamp
 * (Station::lateral_max) can make the seam sit somewhere other than where the
 * nominal strip widths say, and the check is what catches it.
 *
 * `delta` is non-zero in exactly the interesting case: U restarts at 0 at every
 * strip boundary, so two lanes of one carriageway are a U discontinuity. Applying
 * it is a per-VERTEX write, and a vertex's UV is shared by every triangle that
 * references it -- so the shift is only correct if every one of those triangles
 * is shifted with it. The pass therefore solves for the shift over a connected
 * component rather than a pair: sharing a vertex forces two quads to the SAME
 * offset, a mergeable seam forces a DIFFERENCE of one U span, and a component
 * whose offsets are consistent and none of whose vertices is touched by a
 * triangle outside it may be written in one go. A component that fails either
 * test is left alone, and its seams simply do not merge.
 *
 * On a corridor that component comes out as exactly "every band of strip A at
 * offset 0, every band of strip B at offset span(A)", because bands of one strip
 * share vertices while neighbouring strips only share positions. That is why the
 * lateral merge fires on a curved road, where no along-the-road merge is
 * available to make the columns exclusive first. On a junction fill, whose
 * triangles really do share vertices with one another, the same rule collapses
 * the whole patch into one offset and refuses any rewrite, which is the safe
 * answer.
 *
 * ### What is preserved
 *
 * - **SubMesh ranges.** They still tile `[0, indices.size())` with no gap or
 *   overlap, still hold the same MaterialKeys, and no range is reordered. Ranges
 *   shrink; a range that loses every triangle is removed.
 * - **The surface.** The merged quad covers exactly the union of the two it
 *   replaced. This is a topological rewrite, not a simplification: no vertex
 *   moves and no area is gained or lost.
 * - **Winding and normals.** Every surviving triangle keeps its orientation.
 * - **`Mesh::bounds`** stays correct, since no vertex moves.
 *
 * Vertices left unreferenced by the merge are NOT compacted here -- the
 * `optimize_mesh()` fetch pass already does that, and doing it twice would
 * renumber the vertex array for nothing.
 *
 * ### What it costs
 *
 * A dropped seam vertex that a neighbouring, unmerged quad still references
 * becomes a T-vertex: the merged quad's side passes exactly THROUGH it, so there
 * is no gap and no overlap, but the two surfaces meet along an edge one of them
 * has no vertex on. That is inherent to any decimation and is bounded by the same
 * collinearity tolerance that permitted the merge. In practice it is rare on a
 * corridor, because collinearity is a property of the stations and therefore
 * fails or passes for every strip at the same time.
 *
 * UVs move, so a caller that wants tangents matching the new parameterisation
 * must call Mesh::compute_tangents() afterwards. Positions do not move, so
 * Mesh::bounds needs no attention.
 *
 * @param mesh Corridor mesh, modified in place
 * @param cfg  Coplanarity threshold; ignored entirely and the mesh left untouched
 *             when TessellationConfig::merge_coplanar_strips is false
 * @return Quad MERGES made, each of which removed two triangles, so
 *         `2 * result` is the triangle reduction. Zero when nothing merged, when
 *         the pass is disabled,
 *         when `mesh.indices.size()` is not a multiple of 3, when any index is out
 *         of range, or when the SubMesh ranges do not tile the index buffer in
 *         order -- the shapes this pass cannot preserve, which it declines rather
 *         than guesses at. The mesh is left untouched in every one of those cases,
 *         and byte-identical when nothing merged.
 *
 * @note Run this BEFORE weld_vertices() and before optimize_mesh(). The merge
 *       needs the extruder's shared-edge structure to find adjacency, and the
 *       reorder passes destroy the column ordering it relies on to find it
 *       cheaply.
 */
size_t merge_coplanar_quads(Mesh& mesh, const TessellationConfig& cfg);

} // namespace stratum::osm::road
