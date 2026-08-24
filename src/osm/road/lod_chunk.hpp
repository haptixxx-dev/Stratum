/**
 * @file lod_chunk.hpp
 * @brief LOD built from MERGED per-chunk geometry rather than per piece
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why the per-piece chain does not reduce
 *
 * `build_lod_chain()` in mesh_optimize.hpp simplifies one RoadPiece at a time
 * with `LodConfig::lock_borders` set, and it returns 85.3% of the input triangles
 * at its COARSEST level while asking for 10%. That is not a tuning problem and no
 * ratio will fix it.
 *
 * The cause is geometric. `meshopt_SimplifyLockBorder` pins every vertex on an
 * open boundary, and a road piece is a ribbon roughly 3 m wide and tens of metres
 * long. Both of its long sides are open boundary, both of its ends are open
 * boundary, and every material range inside it -- kerb against gutter, gutter
 * against lane -- is an open boundary of its own submesh as well. On a shape that
 * narrow, nearly every vertex IS a border vertex. There is almost no interior
 * left to collapse, so the simplifier correctly refuses to do anything.
 *
 * Unlocking the borders is not the answer either: two adjacent pieces simplified
 * independently would each pull their shared edge inward by a different amount
 * and crack apart.
 *
 * ### The fix
 *
 * Simplify MERGED geometry. Concatenate every piece assigned to one spatial chunk
 * into a single mesh first, then simplify that. Now the seams between pieces are
 * INTERIOR -- welded shut and free to collapse -- and only the genuine chunk
 * boundary is locked. A chunk is tens of pieces and hundreds of metres across, so
 * the locked band is a small fraction of the surface instead of nearly all of it.
 *
 * This also fixes the border lock itself. Per piece, "border" could only mean
 * "topologically open", which over-selects. Per chunk it can mean what it should
 * mean: within ChunkLodConfig::border_band metres of the chunk RECTANGLE. A
 * vertex in the middle of the chunk is free however open its topology is.
 *
 * ### What it achieves
 *
 * Measured on the Lucan extract -- 40,644 pieces, 3,251,855 triangles, binned into
 * 965 chunks of 250 m -- against the default `{0.5, 0.25, 0.1}`, summed over every
 * chunk:
 *
 * | Level | Asked | Per piece | Per chunk |
 * |---|---|---|---|
 * | 1 | 50% | 85.0% | **50.0%** |
 * | 2 | 25% | 85.0% | **25.2%** |
 * | 3 | 10% | 85.0% | **14.9%** |
 *
 * Levels 1 and 2 land on their ratios. Level 3 does not reach 10% and will not:
 * what holds it up is the locked rim, and that lock IS the crack-free guarantee.
 * Setting ChunkLodConfig::border_band to 0 takes level 3 to 13.0% and cracks every
 * chunk seam, so 10% is not reachable by tuning either -- see border_band for the
 * full trade.
 *
 * ### What this costs
 *
 * Level 0 of a ChunkLod is a merged copy of the chunk's pieces, so a chunk that
 * is drawn at full detail holds its geometry twice unless the caller drops the
 * originals. That is the caller's decision and the reason this returns a value
 * rather than mutating the pieces.
 *
 * Time is roughly 1.7 microseconds per level-0 triangle for the whole chain: the
 * extract's densest chunk, 416 pieces and 21,168 triangles, merges, welds and
 * reorders in 8.4 ms and simplifies its three levels in 27.3 ms. Chunks are
 * independent and may be built in parallel.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Ratios, error bound and border band for a chunk-level LOD chain
 */
struct ChunkLodConfig {
    /**
     * @brief Target index fraction of each level below the merged full-detail one
     *
     * Level 0 is the merged mesh and is implied, so `ratios[i]` is the target for
     * `ChunkLod::levels[i + 1]` and is always measured against LEVEL 0, not
     * against the previous level. `{0.5, 0.25, 0.1}` asks for half, a quarter and
     * a tenth of the merged triangle count.
     *
     * Must be strictly decreasing and in (0, 1). An empty vector produces a chain
     * with only the merged full-detail level, which is still useful -- the merge
     * alone removes the per-piece duplication at every seam.
     */
    std::vector<float> ratios = {0.5f, 0.25f, 0.1f};

    /**
     * @brief meshopt_simplify error bound, relative to the mesh extent
     *
     * Looser than LodConfig::target_error's 0.02 on purpose. That bound is
     * relative to the BOUNDING EXTENT of what is being simplified, so the same
     * fraction means a far tighter absolute tolerance on a 40 m piece than on a
     * 400 m chunk. 0.05 of a chunk extent is comparable in metres to 0.02 of a
     * piece extent, and a chunk is by definition only drawn simplified when it is
     * far away.
     *
     * Simplification stops at whichever of the ratio and this bound binds first,
     * so a level may come back above its ratio. That is the error bound
     * protecting the silhouette, not a failure.
     *
     * @note Measured, this bound is close to inert at the first two levels and
     *       only weakly binding at the third: over the extract's 200 densest
     *       chunks the coarsest level moves from 20.9% of level 0 at
     *       `target_error = 0.002` to 15.4% at 0.05 and 14.8% at 0.2, while levels
     *       1 and 2 do not move at all. The ratio is what governs. What actually
     *       holds the coarsest level above its 10% request is border_band -- see
     *       there.
     */
    float target_error = 0.05f;

    /**
     * @brief Lock band around the chunk rectangle, metres
     *
     * A vertex within this distance of the chunk's 2D boundary rectangle is
     * pinned and may not move; everything interior is free. THIS is what the
     * per-piece version could not express -- it had only
     * `meshopt_SimplifyLockBorder`, which asks about topology and not about
     * position, and on a ribbon those are the same question.
     *
     * The band must be wide enough that the two chunks either side of a seam both
     * pin the same vertices. A shared vertex sits exactly on the rectangle of one
     * chunk and exactly on the rectangle of its neighbour, so any positive band
     * catches it in both; 0.5 m gives room for the float error in a position that
     * has been through a projection, a carve and a weld.
     *
     * Setting this to 0 locks nothing and cracks every chunk seam. Setting it
     * larger than the chunk itself locks everything and reduces nothing.
     *
     * This is the setting that decides how close the coarsest level gets to its
     * ratio, and it is the reason the default chain lands at about 15% rather than
     * the 10% it asks for. Over the extract's 200 densest chunks the coarsest
     * level holds 13.0% of level 0 at a band of 0, 15.4% at 0.5 m, 18.3% at 2 m
     * and 29.5% at 10 m. Levels 1 and 2 do not move until the band reaches 10 m.
     * Widening the band buys tolerance for pieces that overhang their chunk and is
     * paid for almost entirely out of the coarsest level.
     *
     * @note Distance is measured in the 2D local plane, against the rectangle's
     *       PERIMETER -- a vertex deep inside the rectangle is far from it, a
     *       vertex just outside it is near it. Height is not part of the measure,
     *       so a bridge deck 8 m above a chunk edge is pinned exactly as the road
     *       under it is.
     */
    float border_band = 0.5f;

    /**
     * @brief Simplify each material range separately
     *
     * OFF, which reverses what this file first said, on measurement.
     *
     * The reason to turn it on is the one build_lod_chain() acts on: a kerb and
     * the carriageway beside it are different SubMesh ranges, and a simplifier
     * free to collapse across the boundary should merge the kerb into the road
     * surface and lose the step. Measured on the Lucan extract, it does not. Two
     * checks, both on the merged geometry with this OFF:
     *
     * - The Curb range's triangles keep their height. The tallest is exactly
     *   0.150 m -- the full kerb -- at every level, on a range decimated to 17.9%
     *   of its triangles, and the MEAN vertical extent RISES with each level,
     *   0.036 m to 0.051 m, because the flat kerb-top triangles decimate away
     *   faster than the vertical faces do. A vertical face carries far too much
     *   quadric error to be flattened while any cheaper collapse remains.
     * - No triangle of any simplified level names a vertex that did not carry
     *   that triangle's material at level 0, across 320,498 simplified triangles.
     *   weld_vertices() keys on the material set, so the merged mesh carries its
     *   material boundaries as attribute seams and meshoptimizer will not collapse
     *   across one.
     *
     * The cost of turning it on, by contrast, is large and scales the wrong way.
     * Each range becomes its own mesh with its own borders, which is the exact
     * fragmentation this file exists to undo, and it bites hardest where there is
     * least geometry to spare. Measured over all 965 chunks of the extract, the
     * coarsest level holds 36.1% of level 0 with this on and 14.9% with it off,
     * and only 270 of 965 chunks get a full four-level chain instead of 932. A
     * seven-piece chunk stalls at 65.5% and gives up after one level where the
     * merged path still reaches 13.0%.
     *
     * Left in because it is the conservative answer if a future profile puts two
     * materials on one surface with no height difference between them -- paint
     * inlaid flush into asphalt, say -- where nothing but the range boundary
     * distinguishes them and the quadric has no step to defend.
     */
    bool simplify_per_material = false;
};

// ============================================================================
// Output
// ============================================================================

/**
 * @brief The merged full-detail chunk mesh and its coarser levels
 */
struct ChunkLod {
    /**
     * @brief The levels, coarsening with index
     *
     * `levels[0]` is the MERGED full-detail chunk mesh: every input piece
     * concatenated, welded and reordered, never simplified. It is not any input
     * piece and it is not the sum of their vertex counts -- the weld closes the
     * seams the pieces used to have between them, which is where the merge pays
     * for itself even before a single level is simplified.
     *
     * Every level keeps the SubMesh structure of level 0, minus any material that
     * simplified away to zero triangles at that level.
     *
     * Size is `1 + n`, where n is the number of ratios that produced a usable
     * level. A ratio that yields no triangles, or that fails to beat the previous
     * level by a worthwhile margin, is dropped along with its switch distance, so
     * the chain can be shorter than `1 + ratios.size()` and is never longer.
     *
     * An empty `levels` means the input was unusable: no pieces, or every piece
     * empty.
     */
    std::vector<Mesh> levels;

    /**
     * @brief Camera distance in metres at which each level becomes the one to draw
     *
     * Always the same length as `levels`, ascending, with `switch_distances[0]`
     * equal to 0: level 0 is what you draw when you are close.
     *
     * Derived from the merged bounds and the achieved ratio rather than measured,
     * the same way LodChain::screen_thresholds is:
     *
     * @code
     *     switch_distances[i] = bounds.radius() * kSwitchFactor
     *                            / std::sqrt(achieved_ratio_of_level_i)
     * @endcode
     *
     * Projected screen area falls off as `1 / distance^2`, so halving the
     * triangle count is paid for at `sqrt(2)` times the distance.
     *
     * Suggested, not enforced. A renderer that has its own policy should ignore
     * these.
     *
     * @note These are CHUNK distances and are therefore much larger than the
     *       per-piece thresholds they replace, because they are derived from the
     *       chunk's radius. Do not mix the two scales in one selection pass.
     */
    std::vector<float> switch_distances;
};

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Merge the pieces of one spatial chunk, then simplify the merged result
 *
 * Steps:
 *
 * 1. Concatenate every non-null, non-empty mesh in @p pieces, preserving each
 *    one's SubMesh MaterialKeys. Ranges of the same key from different pieces are
 *    coalesced, so the merged mesh has at most one range per key.
 * 2. Weld the result. This is the step the per-piece path could not take -- see
 *    RoadNetworkConfig::weld_meshes, which explains why a per-piece weld
 *    deliberately stops at the piece boundary. Here the pieces are being fused
 *    into one owner, so the seam between an edge piece and the junction it runs
 *    into finally has a vertex that belongs to something.
 * 3. Classify every vertex as locked or free by its 2D distance to the rectangle
 *    `[chunk_min, chunk_max]`, using ChunkLodConfig::border_band.
 * 4. Simplify to each ratio in turn, per material range when
 *    ChunkLodConfig::simplify_per_material, with the locked vertices pinned.
 *
 * ### The crack-free guarantee, and its one condition
 *
 * Two adjacent chunks simplified independently still meet exactly at every level,
 * because a vertex on their shared boundary is within `border_band` of BOTH
 * rectangles and is therefore pinned in both simplifications. Nothing is
 * exchanged between chunks and they may be built in parallel or in any order.
 *
 * "Pinned" has to mean "still in the output", and meshoptimizer's `vertex_lock`
 * on its own does not promise that -- it promises only that the vertex will not
 * MOVE. Two further steps in the implementation close the gap, and both are part
 * of this contract rather than incidental:
 *
 * - A locked vertex can still be dropped when the last triangle naming it is
 *   flattened by a legal collapse of its one unlocked corner. So the lock set is
 *   grown by one triangle ring: any triangle with a locked vertex has all three
 *   locked, which leaves no legal collapse source and no way to flatten it.
 * - meshopt_SimplifyPrune deletes whole connected components by error and does
 *   not consult `vertex_lock` at all, so it can delete a component with pinned
 *   vertices in it. Pruning is therefore applied only to components that no lock
 *   touches; a component holding a pinned vertex is simplified with pruning off.
 *
 * With both in place, every vertex within `border_band` of the rectangle at level
 * 0 is present, at the same position, at every level below it. Verified on the
 * extract's densest chunk: 149 pinned vertices, 0 missing at any level. With
 * neither, 130 of those 149 were gone by the coarsest level; with only the
 * component-anchored pruning and no ring, 5 were.
 *
 * The condition is that the caller's chunk rectangles TILE the plane without
 * gaps, and that a piece is assigned to a chunk whose rectangle its geometry is
 * inside up to the band. A piece straddling a boundary and assigned wholly to one
 * chunk has geometry outside that chunk's rectangle; that geometry is more than
 * `border_band` from the perimeter on the wrong side, so it is treated as
 * interior and is free to move away from the neighbour that did not know about
 * it. Assign by geometry, not by centroid, or widen the band to cover the
 * overhang.
 *
 * @param pieces    Meshes belonging to this chunk. Null pointers and empty meshes
 *                  are skipped, not an error. The pointed-to meshes are read
 *                  only.
 * @param chunk_min Minimum corner of the chunk's 2D bounds, in local metres,
 *                  matching the (x, y) of Road::polyline -- NOT world space. The
 *                  world mapping is `(x, y) -> vec3(x, height, -y)`, so the
 *                  implementation must map before comparing against vertex
 *                  positions.
 * @param chunk_max Maximum corner, same convention. A degenerate or inverted
 *                  rectangle locks nothing and is treated as "no border".
 * @param cfg       Ratios, error bound and band width
 * @return The chain. `levels.empty()` when @p pieces contributed no geometry.
 *
 * @note Deterministic. The merge is in the order @p pieces is given, and nothing
 *       inside depends on thread scheduling, so two runs over the same input
 *       produce identical output. Callers building chunks in parallel must
 *       therefore still order the pieces of each chunk deterministically.
 */
[[nodiscard]] ChunkLod build_chunk_lod(const std::vector<const Mesh*>& pieces,
                                       const glm::dvec2& chunk_min,
                                       const glm::dvec2& chunk_max,
                                       const ChunkLodConfig& cfg = {});

} // namespace stratum::osm::road
