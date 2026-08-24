/**
 * @file mesh_optimize.hpp
 * @brief Vertex welding, meshoptimizer reordering, and per-material LOD chains
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * P7's first job. Everything upstream optimises for being CORRECT to build: the
 * corridor extruder emits a fresh vertex column per station, the junction builder
 * emits its fan independently of the arms it joins, and the marking and structure
 * passes append whole meshes into a piece without ever looking at what is already
 * there. The result is geometry that draws right and ships badly.
 *
 * The numbers are the argument. A 63 MB Lucan extract carries 864 km of highway
 * at 8 m station spacing, which is roughly 100k stations, and a kerbed profile
 * emits about 16 vertex columns per station before junctions, curb rings,
 * markings and structures are added. Nothing downstream -- the GPU buffer pool,
 * the resident budget, the exporter -- gets cheaper than the vertex count handed
 * to it.
 *
 * Three passes, in this order, and the order is load-bearing:
 *
 * 1. weld_vertices()   -- fewer vertices, continuous normals, shared topology.
 * 2. optimize_mesh()   -- same geometry, better memory and raster locality.
 * 3. build_lod_chain() -- fewer triangles at distance.
 *
 * Welding runs first because meshopt_simplify() collapses edges, and an edge only
 * exists between triangles that already share vertex indices. Simplifying an
 * unwelded mesh does almost nothing: every triangle is its own island and there
 * is no interior edge to collapse.
 *
 * ### The one thing that must not happen
 *
 * A Mesh here is not one surface. It is a carriageway, a kerb, a footway and a
 * stripe of paint sharing one index buffer, separated only by SubMesh ranges.
 * Both welding and simplification are free to destroy that separation if they are
 * run naively, and the failure is not subtle: a kerb welded into the carriageway
 * loses the 150 mm step that is the whole point of the profile, and a whole-buffer
 * meshopt_optimizeVertexCache() call reorders triangles across range boundaries so
 * that every SubMesh afterwards points at somebody else's triangles.
 *
 * So: welding is keyed on the material set (see WeldConfig::respect_material),
 * triangle reordering happens strictly WITHIN one range, and simplification runs
 * per range and reassembles. Every function below states exactly what it promises
 * about SubMesh ranges, and those promises are the contract the tests check.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "renderer/mesh.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Welding
// ============================================================================

/**
 * @brief Tolerances for deciding that two vertices are the same vertex
 *
 * Every epsilon is an inclusive slack: two attributes match when their
 * difference is at most the epsilon, so a zero epsilon means exact equality
 * rather than "never match".
 */
struct WeldConfig {
    /**
     * @brief Position tolerance, metres, per component
     *
     * 0.1 mm. Large enough to absorb the float error of two builders arriving at
     * the same station from different directions -- an arm end column and the
     * junction fan vertex it meets -- and far below anything a viewer resolves.
     *
     * Positions are quantised onto a grid of this size to find candidates, so
     * this is also the spatial hash cell size.
     */
    float position_epsilon = 1e-4f;

    /**
     * @brief Normal tolerance as dot-product slack
     *
     * Two normals match when `dot(a, b) >= 1 - normal_epsilon`. 0.01 is about
     * 8 degrees.
     *
     * @warning This is what stops welding from smoothing the mesh. A kerb face
     *          meets a kerb top at 90 degrees, a bridge parapet meets the deck at
     *          90 degrees, and both share a position and a material. Welding those
     *          two vertices would average their normals into a 45 degree bevel
     *          shading a hard edge -- the mesh would still be manifold and would
     *          still look wrong. A CREASE MUST NOT BE WELDED SMOOTH. Raising this
     *          past roughly 0.3 starts destroying kerbs.
     */
    float normal_epsilon = 0.01f;

    /**
     * @brief UV tolerance, per component, in UV units
     *
     * Guards the seam where V restarts. Arc-length V accumulates in metres over an
     * edge and restarts at each junction (see the UV convention in
     * docs/plans/road_network_plan.md), so two coincident vertices can legitimately
     * carry V = 0 and V = 137.4. Welding them would stretch one texture across the
     * whole road.
     */
    float uv_epsilon = 1e-4f;

    /**
     * @brief Never weld two vertices whose referencing materials differ
     *
     * The material key of a vertex is the BITMASK of the MaterialIds of every
     * SubMesh range that references it, and two vertices may weld only when their
     * masks are equal.
     *
     * A mask rather than a single id, because a shared column is normal and must
     * stay weldable: the corridor extruder already welds the boundary column
     * between the gutter (Asphalt) and the kerb top (Curb) into one vertex, so
     * that vertex is referenced by two materials. Its counterpart arriving from
     * the neighbouring piece is referenced by the same two. Equal masks, so they
     * weld -- which is exactly the seam this pass exists to close. A carriageway
     * vertex (Asphalt alone) and a paint vertex (Markings alone) sitting at the
     * same place have different masks and never weld, which is also correct:
     * marking quads are separate geometry floating just above the surface and
     * must never share vertices with it.
     *
     * Setting this false ignores materials entirely. The collision builder does
     * that deliberately, because its output has no materials left to protect.
     */
    bool respect_material = true;

    /**
     * @brief Vertex-colour tolerance, per component, in colour units
     *
     * Colours are authored per strip and per material, so this is rarely the
     * binding test; it exists so that a producer which tints geometry (a
     * marking colour, a surface variant) cannot have two visually different
     * vertices silently averaged into one. 1/255 is one 8-bit step, which is
     * the finest difference an exported vertex colour can carry.
     *
     * Set at or above 1.0 to ignore colour entirely, since every component of
     * a colour lies in [0, 1].
     */
    float color_epsilon = 1.0f / 255.0f;

    /**
     * @brief Tangent tolerance as dot-product slack on tangent.xyz
     *
     * Two tangents match when `dot(normalize(a.xyz), normalize(b.xyz)) >=
     * 1 - tangent_epsilon` AND the sign of `tangent.w` -- the bitangent
     * handedness -- is the same.
     *
     * Deliberately much looser than WeldConfig::normal_epsilon. Tangents are
     * not authored: Mesh::compute_tangents() accumulates them per triangle from
     * the UV gradient, so two vertices that agree on position, normal and UV to
     * within their own epsilons can still carry tangents a few degrees apart
     * purely because they were touched by different triangle fans. Testing
     * those as tightly as normals would block exactly the cross-builder welds
     * this pass exists to make, for a difference no normal map resolves.
     *
     * What the test still catches is the case that matters: a MIRRORED UV seam,
     * where the tangent is reversed (dot near -1) or the handedness flips.
     * Welding across one of those bakes the wrong bitangent into the survivor
     * and the normal map lights from the wrong side.
     *
     * Set at or above 2.0 to ignore tangents entirely, including the handedness
     * test, since a dot product never falls below -1.
     */
    float tangent_epsilon = 0.25f;
};

/**
 * @brief Deduplicate coincident vertices in place
 *
 * Vertices that agree on position, normal, UV, colour and tangent within @p cfg,
 * and (when WeldConfig::respect_material) on their material mask, collapse to one
 * vertex. Indices are remapped onto the survivors and the vertex array is
 * compacted.
 *
 * ### What is preserved exactly
 *
 * - **Every SubMesh range.** `index_offset` and `index_count` are BYTE-FOR-BYTE
 *   unchanged, for every range, in the same order. A triangle is therefore never
 *   dropped: a shorter range would shift every range after it, and callers hold
 *   those offsets.
 * - **Triangle count and order.** `indices.size()` is unchanged and index i still
 *   belongs to the same triangle of the same range.
 * - **Winding.** Each triangle keeps its vertex order.
 * - **Non-degeneracy.** No triangle comes out with two equal indices unless it
 *   went in with them. Since a triangle may not be dropped, the WELD is refused
 *   instead: when two corners of one triangle would collapse onto each other, the
 *   higher-indexed of the pair keeps its own vertex. See below.
 *
 * ### The flatten guard
 *
 * Two corners of one triangle can pass every attribute test above -- a sliver
 * whose two ends sit inside WeldConfig::position_epsilon of each other. The
 * markings, the cul-de-sac fan and the roundabout annulus all emit some. Welding
 * such a pair would leave a triangle that costs a primitive, rasterises nothing,
 * and reads as a degenerate face in every exporter and DCC tool, which the plan's
 * topology contract forbids. So those welds do not happen, and the vertices stay
 * apart. On the tests/data fixtures this refuses between 0 and 45 welds per
 * network, and the geometry is bit-identical to the input in every case, because
 * the two positions it kept apart differ by less than a tenth of a millimetre.
 *
 * ### What changes
 *
 * - `vertices` shrinks and is renumbered. Any vertex index held OUTSIDE the mesh
 *   is invalidated.
 * - `bounds` is recomputed from the surviving vertices.
 *
 * ### Determinism
 *
 * Required, because the golden tests hash the output. The survivor of a weld
 * group is always the member with the LOWEST original index, and the compacted
 * vertex array is in ascending original-index order. The result therefore does
 * not depend on hash iteration order, on thread count, or on the order candidates
 * were visited.
 *
 * @param mesh Mesh to weld, modified in place. A mesh with no indices is left
 *             untouched and returns 0.
 * @param cfg  Tolerances; the defaults are the shipping values
 * @return Number of vertices removed, that is
 *         `vertices.size()` before minus `vertices.size()` after
 */
size_t weld_vertices(Mesh& mesh, const WeldConfig& cfg = {});

// ============================================================================
// Reordering and LODs
// ============================================================================

/**
 * @brief Tunables shared by optimize_mesh() and build_lod_chain()
 */
struct LodConfig {
    /**
     * @brief Target index fraction of each level below the full-detail one
     *
     * Level 0 is the full-detail mesh and is implied, so `ratios[i]` is the target
     * for `LodChain::levels[i + 1]` and is always measured against LEVEL 0, not
     * against the previous level. `{0.5, 0.25, 0.1}` therefore asks for half,
     * a quarter and a tenth of the original triangles.
     *
     * Must be strictly decreasing and in (0, 1). An empty vector produces a chain
     * with only the full-detail level.
     */
    std::vector<float> ratios = {0.5f, 0.25f, 0.1f};

    /**
     * @brief meshopt_simplify error bound, relative to the mesh extent
     *
     * Passed straight to meshopt_simplify() as `target_error`. 0.02 is 2% of the
     * mesh's bounding extent. Simplification stops at whichever of the ratio and
     * this bound binds first, so a level can legitimately come back with more
     * triangles than its ratio asked for -- that is the error bound refusing to
     * wreck the silhouette, not a failure.
     */
    float target_error = 0.02f;

    /**
     * @brief Do not move vertices on an open boundary
     *
     * meshopt_SimplifyLockBorder. Two reasons, and both are about seams:
     *
     * - Road geometry is chunked. Two adjacent chunks simplified independently
     *   would each pull their shared edge inward by a different amount and open a
     *   visible crack between them. Locking the border keeps the chunks meeting.
     * - Because build_lod_chain() simplifies PER SUBMESH, every material boundary
     *   is an open boundary of its own submesh, so this also pins the kerb against
     *   the carriageway for free.
     *
     * Turning it off gives better reduction and cracked seams.
     */
    bool lock_borders = true;

    /// Run meshopt_optimizeVertexCache
    bool optimize_cache = true;

    /**
     * @brief Run meshopt_optimizeOverdraw
     *
     * Uses a fixed threshold of 1.05, that is, up to 5% worse vertex-cache
     * behaviour is accepted in exchange for front-to-back ordering. Roads are
     * close to flat and mostly seen at a grazing angle, so overdraw ordering
     * matters less here than it does for a building; the pass is cheap and left on.
     */
    bool optimize_overdraw = true;

    /// Run meshopt_optimizeVertexFetch, which reorders the VERTEX array
    bool optimize_fetch = true;

    /**
     * @brief Let build_lod_chain() retry a missed target with meshopt_simplifySloppy
     *
     * meshopt_simplify() preserves topology and stops short of the requested
     * triangle count whenever reaching it would require a collapse the topology
     * or LodConfig::lock_borders forbids. That is usually the right answer and
     * is why LodChain levels routinely come back above their ratio.
     *
     * When it is not -- a small submesh whose every interior edge is pinned
     * against a border, which comes back at full detail for every level of the
     * chain -- this allows one retry with meshopt_simplifySloppy(), which does
     * NOT preserve topology and is free to weld unrelated shells together.
     *
     * The retry is guarded three ways: it runs only when the constrained pass
     * missed its target, its result is kept only when it is strictly smaller
     * than the constrained result, and when LodConfig::lock_borders is set the
     * range's open-boundary vertices are passed to it as a vertex lock so chunk
     * seams and material boundaries stay pinned even in the sloppy path.
     *
     * Every retry that is KEPT increments LodChain::sloppy_simplifications, so a
     * caller can tell whether a chain was built the safe way.
     *
     * @note Off by default because it was measured and it does not pay. Across
     *       the tests/data fixtures the retry fires on 6 to 18 ranges per fixture
     *       and moves the coarsest level from 79.1% of level 0 to 78.6% -- half a
     *       percentage point of triangles, in exchange for a pass that is allowed
     *       to weld unrelated shells together on every range it touches. Turn it
     *       on for an export where a specific piece refuses to simplify and the
     *       topology of that piece does not matter.
     */
    bool allow_sloppy = false;
};

/**
 * @brief A mesh at several levels of detail, with suggested switch distances
 */
struct LodChain {
    /**
     * @brief The levels, coarsening with index
     *
     * `levels[0]` is the full-detail mesh: a copy of build_lod_chain()'s input,
     * after its own optimize_mesh() pass, never simplified. Every level keeps the
     * SubMesh structure of level 0, minus any material that simplified away to
     * zero triangles at that level.
     *
     * Size is `1 + n`, where n is the number of ratios that produced a usable
     * level. A ratio that yields no triangles, or that fails to beat the previous
     * level by a worthwhile margin, is dropped along with its threshold, so the
     * chain can be shorter than `1 + ratios.size()` and is never longer. An empty
     * chain -- `levels.empty()` -- means the input was unusable.
     */
    std::vector<Mesh> levels;

    /**
     * @brief Camera distance in metres at which each level becomes the one to draw
     *
     * Always the same length as `levels`, ascending, with `screen_thresholds[0]`
     * equal to 0: level 0 is what you draw when you are close.
     *
     * Derived from the mesh bounds and the achieved ratio, not measured:
     *
     * @code
     *     screen_thresholds[i] = bounds.radius() * kLodSwitchFactor
     *                            / std::sqrt(achieved_ratio_of_level_i)
     * @endcode
     *
     * The square root is the whole idea. Projected screen area falls off as
     * `1 / distance^2`, so halving the triangle count is paid for by the mesh
     * covering half the pixels, which happens at `sqrt(2)` times the distance.
     * Suggested, not enforced -- the renderer is free to ignore these, and
     * level_for_distance() is provided for the common case where it does not.
     */
    std::vector<float> screen_thresholds;

    /**
     * @brief How many submesh ranges fell back to meshopt_simplifySloppy
     *
     * Summed over every level and every material range of the chain. Zero is the
     * expected value and means every level was produced by the topology-preserving
     * simplifier.
     *
     * A non-zero count is not an error, but it IS a warning: sloppy
     * simplification does not preserve topology, so the affected range may have
     * had shells welded together or holes closed. See LodConfig::allow_sloppy for
     * when the fallback fires and what still constrains it.
     */
    size_t sloppy_simplifications = 0;

    /// True when there is at least one level to draw
    [[nodiscard]] bool is_valid() const { return !levels.empty(); }

    /**
     * @brief Index of the level to draw at a given camera distance
     *
     * The coarsest level whose threshold is at or below @p distance_m.
     *
     * @param distance_m Camera-to-mesh distance in metres
     * @return Index into levels(). 0 for an empty chain, so the caller can index
     *         `levels` only after checking is_valid().
     */
    [[nodiscard]] size_t level_for_distance(float distance_m) const {
        size_t chosen = 0;
        for (size_t i = 0; i < screen_thresholds.size() && i < levels.size(); ++i) {
            if (distance_m >= screen_thresholds[i]) {
                chosen = i;
            }
        }
        return chosen;
    }
};

/**
 * @brief Distance-scale constant behind LodChain::screen_thresholds
 *
 * A mesh of bounding radius r switches to a level of ratio q at
 * `r * kLodSwitchFactor / sqrt(q)` metres. At the default 8, a 40 m road piece
 * drops to its half-detail level at about 450 m. Exposed so the tests can assert
 * the formula rather than a hard-coded number.
 */
inline constexpr float kLodSwitchFactor = 8.0f;

/**
 * @brief Reorder indices and vertices for GPU locality, in place
 *
 * Runs, subject to the LodConfig flags: meshopt_optimizeVertexCache, then
 * meshopt_optimizeOverdraw, then meshopt_optimizeVertexFetch. That order is
 * required by meshoptimizer -- overdraw sorting assumes cache-optimised clusters,
 * and fetch optimisation must be last because it renumbers vertices.
 *
 * ### SubMesh contract
 *
 * `index_offset` and `index_count` of every range are UNCHANGED. The cache and
 * overdraw passes reorder triangles strictly WITHIN each range, one range at a
 * time; they are never run over the whole index buffer, because that would move
 * triangles across range boundaries and leave every SubMesh pointing at another
 * material's geometry. The fetch pass renumbers vertices globally, which is safe:
 * it moves vertices, not triangles.
 *
 * Ranges that are not valid triangle-list ranges -- offset or count not a
 * multiple of 3 -- are skipped rather than reordered.
 *
 * ### Determinism
 *
 * meshoptimizer's reordering passes are deterministic for a given input, and
 * per-range application does not change that. Two builds of the same mesh produce
 * the same index buffer.
 *
 * @param mesh Mesh to reorder, modified in place. No-op for a mesh with no
 *             indices or no vertices.
 * @param cfg  Tunables; only the three optimize_* flags are read
 *
 * @note Call weld_vertices() first. Reordering an unwelded mesh is not wrong, it
 *       is just close to pointless: the vertex cache cannot hit on vertices that
 *       are never reused.
 */
void optimize_mesh(Mesh& mesh, const LodConfig& cfg = {});

/**
 * @brief Build a level-of-detail chain by simplifying per material
 *
 * ### Why per submesh, and not once over the whole mesh
 *
 * meshopt_simplify() collapses edges between triangles that share vertices. Run
 * over the whole index buffer it sees one soup and has no idea that the kerb top
 * and the gutter are different surfaces -- it collapses the 150 mm step between
 * them because that is exactly the cheap collapse it is looking for, and the road
 * comes back flat. Worse, it returns ONE index buffer, so every SubMesh range is
 * meaningless afterwards and the material slots are gone.
 *
 * So each range is extracted into its own index buffer with its own vertex remap,
 * simplified alone, and the survivors are reassembled into one mesh with the
 * ranges rebuilt in ascending MaterialId order, exactly as
 * Mesh::sort_submeshes_by_material() leaves them. A material boundary is then an
 * open border of its submesh, which LodConfig::lock_borders pins.
 *
 * Per-level triangle budgets are apportioned by the range's share of level 0, so
 * a level's total lands near its ratio even though no single range is forced to.
 *
 * ### Steps
 *
 * 1. Copy the input, run optimize_mesh() on it, and store it as `levels[0]`.
 * 2. For each ratio in order: for each SubMesh range of level 0, simplify to that
 *    range's share of the target index count with LodConfig::target_error and
 *    LodConfig::lock_borders, drop ranges that came back empty, reassemble,
 *    run optimize_mesh() on the result, and append it.
 * 3. Drop a level that produced no triangles, or that failed to reduce the
 *    previous level's triangle count by at least 10%, since paying for a second
 *    resident copy of nearly the same mesh is a loss.
 * 4. Fill screen_thresholds from the ACHIEVED ratio of each level.
 *
 * @param mesh Source geometry. Not modified.
 * @param cfg  Ratios, error bound and border locking
 * @return The chain. `levels` is empty when @p mesh has no triangles; otherwise
 *         it always holds at least the full-detail level, so a chain that could
 *         not be simplified at all degrades to "always draw level 0".
 *
 * @note Call weld_vertices() first, and this time it is not optional. An unwelded
 *       mesh has no interior edges to collapse and every level comes back
 *       identical to level 0.
 */
[[nodiscard]] LodChain build_lod_chain(const Mesh& mesh, const LodConfig& cfg = {});

} // namespace stratum::osm::road
