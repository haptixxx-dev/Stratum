// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file building_lod.hpp
 * @brief E6: the per-building LOD chain, and why a building is not a road chunk
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * THE SCHEME THIS FOLLOWS
 * ================================================================================
 *
 * osm/road/lod_chunk.hpp already solved the LOD problem once, for road chunks,
 * and it solved it by measurement rather than by taste. This file uses that
 * vocabulary deliberately and does not invent a second one:
 *
 * - **Ratios are measured against LEVEL 0**, never against the previous level, so
 *   `{0.5, 0.25, 0.1}` means half, a quarter and a tenth of the input triangles.
 * - **`target_error` is relative to the mesh extent**, and a level is allowed to
 *   come back above its ratio when the error bound binds first. That is the
 *   silhouette being protected, not a failure.
 * - **A `vertex_lock` array pins what must not move**, and the lock set is grown
 *   by one triangle ring and propagated to every wedge of a locked position,
 *   because meshoptimizer's lock promises only that a vertex will not MOVE and
 *   not that it will survive. Both steps are lod_chunk.cpp's, for its reasons.
 * - **A level that does not beat the previous one by kMinLevelReduction is
 *   DROPPED**, along with its switch distance, so the chain is never longer than
 *   `1 + ratios.size()` and is often shorter.
 * - **Switch distances are DERIVED from the achieved ratio**, not from the
 *   requested one, using the same `radius * kSwitchFactor / sqrt(ratio)` formula
 *   and the same constant.
 *
 * One rule is this file's own and has no counterpart in lod_chunk.hpp, because a
 * road chunk has nothing it can happen to:
 *
 * - **A level that is SEE-THROUGH where level 0 was solid is not emitted at all.**
 *   Not repaired, not warned about: refused, and the chain is shorter by one. See
 *   "Its detail is a WINDOW" below for what made that necessary.
 *
 * What is NOT carried over, and why, is set out under "What a building is not"
 * below. "It is a different shape" would not be a reason; each one names a
 * behaviour that was measured on this geometry.
 *
 * ================================================================================
 * WHAT A BUILDING IS NOT
 * ================================================================================
 *
 * ### 1. It has no neighbour, so there is no border band -- but there IS a ground
 *
 * ChunkLodConfig::border_band exists so that two chunks simplified independently
 * still meet: a vertex on their shared rectangle is inside the band of BOTH and
 * is pinned in both. A building is simplified alone and welds to nothing, so
 * that machinery has no job here and the chunk rectangle is not a parameter of
 * this file.
 *
 * The GROUND CONTACT does transfer, and it is not optional. A building whose base
 * ring moves at a coarse level floats above the terrain or sinks into it, and
 * that is visible from much further away than any of the detail the level was
 * traded for. Measured on the tower fixture in tests/procgen/test_building_lod.cpp
 * -- a 12 x 8 x 20 m prism with its walls subdivided 8 x 10 per side and a pyramid
 * cap, 644 triangles welded, 32 distinct base positions -- the chain built with
 * `ground_band` at zero keeps 14 of those 32 at its first level, 8 at its second
 * and 7 at its third, and a bare meshopt_simplifySloppy() call at a tenth lifts
 * the ENTIRE base to y = 2.0 m. With the band at its default the same chain keeps
 * all 32 at y = 0 at every level. So this file keeps lod_chunk.cpp's lock
 * machinery in full and only changes the predicate that seeds it: not "near the
 * chunk rectangle" but "within `ground_band` of the lowest vertex".
 *
 * ### 2. Its geometry is a QUILT, and a topology-preserving simplifier stalls on it
 *
 * This is the same discovery lod_chunk.hpp opens with, arriving by a different
 * route, and it is the reason this file exists rather than a two-line call to
 * build_lod_chain().
 *
 * A rule-generated facade is a grid of tiles: `split(y)` cuts floors, `split(x)`
 * cuts bays, and each tile is triangulated on its own by shape_to_mesh(). Every
 * tile boundary is therefore an open border, and the boundaries do not line up --
 * a 6 m shopfront panel meets twelve separate tiles along its top edge, so that
 * edge is a T-junction that NO weld can close. meshoptimizer's border-preserving
 * quadric then lets a border vertex collapse only along its own border loop, and
 * on a quilt that is almost no freedom at all.
 *
 * Measured on the facade rule file in tests/procgen/test_building_lod.cpp:
 * meshopt_simplify returns **330 of 400 triangles, 82%, at a requested ratio of
 * 0.5, of 0.25 and of 0.1 alike**. It is not a tuning problem, exactly as it was
 * not one for a road piece.
 *
 * The road's fix was to make the seams interior by merging and welding. That
 * cannot work here: the seams are T-junctions, and a T-junction has no vertex to
 * weld to. So the fix is the other one meshoptimizer offers --
 * `meshopt_simplifySloppy`, which does not preserve topology and is free to weld
 * unrelated shells together -- with the ground lock still applied. It takes the
 * same mesh to 120, 82 and 40 triangles for the three ratios while the bounding
 * box, the ridge height and the plan outline all hold to about 1%.
 *
 * LodConfig::allow_sloppy describes exactly this case -- "a small submesh whose
 * every interior edge is pinned against a border, which comes back at full detail
 * for every level of the chain" -- and turns it OFF because on roads it bought
 * half a percentage point. The measurement here is 82% against 20%, so it is ON,
 * and it keeps all three of that flag's guards: it runs only when the constrained
 * pass missed its target, its result is kept only when it is strictly smaller,
 * and the lock array goes into it too.
 *
 * It needs a FOURTH guard that a road does not, and that guard cost this file a
 * defect before it existed. "Free to weld unrelated shells together" also means
 * free to DELETE one, and the shell it deleted was the pane of glass covering a
 * window. See the next section.
 *
 * ### 3. Its detail is a WINDOW, which must go as a unit or not at all
 *
 * A road chunk's detail is paint: flat, on the surface, and nothing goes wrong
 * when a simplifier eats it. A building's detail is an opening with depth, and an
 * opening is the one thing that must not be half-removed. Collapse a window
 * partway and the wall has a ragged gap in it that reads as a hole in the
 * building.
 *
 * It is also where all the triangles are. A bare `window()` on one tile is 28
 * triangles: 6 of reveal, 8 of frame, 2 of glazing, 12 of sill. Twelve windows on
 * the facade fixture are 336 of its 400 triangles. No ratio can be met without
 * addressing them and no simplifier can address them -- see the quilt above.
 *
 * So they are removed EXPLICITLY, by material key, whole: see DetailDrop. Which
 * parts go is the whole point and it is not arbitrary. op_facade.hpp builds an
 * opening as a REVEAL (the returns from the wall face back to the glazing line)
 * plus a COVERING PLANE at that line (frame and glazing, which tile the opening
 * exactly between them) plus a sill hanging below. Drop the reveal and the sill
 * and the opening is still covered, by a plane sitting `reveal` metres behind the
 * wall face; drop any part of the covering plane and the wall has a hole in it.
 * So default_facade_detail_drops() names the returns and nothing else.
 *
 * ### That is only half of it, and the other half is what went wrong
 *
 * A drop protects an opening against the DROP STEP. Nothing protected it against
 * the simplify step that runs immediately afterwards, and this file shipped with
 * both halves broken:
 *
 * - The default drop list named the FRAME at its second ratio -- half of the
 *   covering plane -- beside a comment saying in so many words that dropping the
 *   frame "leaves a slot one frame-width wide around the glass". It did.
 * - meshopt_simplifySloppy then deleted the glazing outright at the third,
 *   because a shell it is free to weld is a shell it is free to drop.
 *
 * Measured by casting rays at the front wall of the facade fixture: level 2 had
 * 4% of that wall see-through and level 3 had 24% -- 14.4 m2, the twelve openings
 * entire, with the inside of the building visible through them. The whole front
 * of a building was a grid of holes at the distance it is drawn from most of the
 * time, and sixteen tests passed over it, because every one of them measured the
 * drop step in isolation and none of them measured a level of a real chain.
 *
 * The repair is two mechanisms, and they are ordered on measurement rather than
 * on taste:
 *
 * 1. **The guard.** Before a level is emitted, a ray is cast outward from every
 *    point that a covering plane of level 0 covered. A level that leaves more
 *    than a few percent of them open is NOT EMITTED -- see
 *    BuildingLodConfig::covering_keys. The visibility question is asked as a
 *    visibility question, because the two cheap proxies for it both give wrong
 *    answers here: the covering plane's AREA goes to zero on a level with no hole
 *    in it at all (the constrained pass collapses the glazing into the frame, and
 *    the sloppy pass welds the wall shut over the whole opening), and a triangle
 *    count says even less.
 * 2. **The retreat.** A sloppy level that fails the guard falls back to the
 *    constrained result rather than being thrown away. The constrained pass
 *    cannot have this fault -- it preserves topology, meshopt_SimplifyPrune is
 *    deliberately not passed, and a pass that can neither delete a shell nor tear
 *    one cannot open a hole that was covered. Measured, it holds all 14.4 m2 on
 *    every configuration tried.
 *
 * WHAT THIS COSTS, because it is not free and the file used to claim otherwise:
 * twelve window frames are 96 of 400 triangles and they cannot go while the
 * openings are still cut, so the chain reaches 28.5% of level 0 and not the 10.5%
 * this header used to advertise. That 10.5% was the perforated level. A level
 * that reaches a tenth of a facade's triangles exists -- the sloppy pass finds an
 * 82-triangle one, watertight, when it is given the undropped geometry -- and
 * reaching it from a dropped source would mean CLOSING each opening in the wall
 * panel rather than covering it. That is real work in op_facade's geometry and it
 * is not done here.
 *
 * ### 4. What was deliberately left out
 *
 * - **meshopt_SimplifyPrune, and lod_chunk.cpp's anchored/floating component
 *   split with it.** Pruning is there to delete components too small to decimate,
 *   which for a road chunk means hundreds of lane arrows and zebra stripes, and
 *   lod_chunk.cpp then has to protect the locked rim from it because
 *   meshoptimizer's pruning does not consult `vertex_lock`. Measured on both
 *   fixtures here, pruning changes the output triangle count by zero -- 330, 330
 *   and 330 triangles on the facade and 321, 161 and 68 on the tower, with the
 *   flag on and with it off alike. A building is one or two large components
 *   rather than a scatter of small ones, and the small parts it does have are the
 *   windows, which DetailDrop removes by name. Turning it on would buy nothing
 *   and would oblige this file to carry the component analysis that keeps it from
 *   deleting the ground.
 * - **Per-material simplification.** Off for the same reason ChunkLodConfig::
 *   simplify_per_material is off, and harder: every facade part is
 *   MaterialId::Wall with a different variant, so per-material simplification
 *   would cut an already fragmented quilt into five finer ones.
 *
 * ================================================================================
 * WHAT IT ACHIEVES
 * ================================================================================
 *
 * The facade fixture -- a 6 x 4 m footprint extruded to 10 m, a 35 degree gable
 * roof, three floors of four bays, twelve `window()` calls, 45 terminals and 400
 * triangles -- against the default `{0.5, 0.25, 0.1}`:
 *
 * | Level | Asked | Got | Triangles | Switch | What went, and how |
 * |---|---|---|---|---|---|
 * | 0 | -- | -- | 400 | 0 m | nothing |
 * | 1 | 50% | **46%** | 184 | 79.65 m | the reveal and the sill, dropped by key |
 * | 2 | 25% | **28.5%** | 114 | 101.19 m | the glazing, collapsed into the FRAME by the constrained pass -- the opening stays covered to the square centimetre and the glazing range comes back empty |
 * | 3 | 10% | -- | -- | -- | nothing: no candidate beat level 2 without opening an opening, so the level is not emitted |
 *
 * Every level of that chain is watertight: 0 of 300 rays cast at the twelve
 * openings come through, at every level, which is what
 * tests/procgen/test_building_lod.cpp measures and what the chain that shipped
 * before this could not do.
 *
 * The same input with both mechanisms off stalls at 82% and produces ONE usable
 * level. With the drops off and the sloppy retry on it reaches 82 triangles --
 * 20.5%, also watertight, because the clusterer welds the openings shut rather
 * than deleting their cover when it is given the returns to weld across. The two
 * mechanisms are doing different jobs and they do not compose: the returns are
 * what the clusterer needs, and the drop is what takes them away. Reaching 20.5%
 * from a dropped source is the open problem this file leaves behind.
 *
 * ================================================================================
 * SUBMESH STRUCTURE ACROSS LEVELS
 * ================================================================================
 *
 * lod_chunk.hpp promises that every level keeps the SubMesh structure of level 0
 * minus any material that simplified away to zero triangles. This file keeps that
 * promise and adds one documented exception: a material named in
 * BuildingLodConfig::detail_drops is ABSENT from every level at and below its
 * `from_ratio`, on purpose, and its absence is not a defect. A consumer that
 * looks up a material slot per level rather than assuming level 0's list is
 * correct either way.
 *
 * Level 0 itself is the input mesh with its index ranges COALESCED -- one range
 * per MaterialKey, ascending by MaterialKey::packed() -- and reordered for the
 * GPU. It is never simplified and its vertices are never welded; see
 * build_building_lod() for why the weld is kept off level 0.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API beyond renderer/mesh.hpp, which is a plain data struct.
 */

#pragma once

#include "renderer/mesh.hpp"

#include <cstddef>
#include <vector>

namespace stratum::procgen {

// ============================================================================
// Detail
// ============================================================================

/**
 * @brief One material key that is removed whole, from a given ratio down
 *
 * The mechanism described under "Its detail is a WINDOW" above. A drop is not a
 * simplification: the triangles of @ref key are deleted outright before the
 * simplifier is asked for anything, and nothing is left behind in their place.
 */
struct DetailDrop {
    /// The material to remove. Matched on the full (slot, variant) pair.
    MaterialKey key{};

    /**
     * @brief Index into BuildingLodConfig::ratios from which the key is absent
     *
     * Indexes the RATIO, not the finished level, and that distinction is
     * load-bearing: a candidate level can be dropped for failing to reduce, which
     * would shift every finished index after it. Keyed on the ratio, what a
     * config asks for does not depend on what the geometry happened to allow.
     *
     * 0 means "gone from the first simplified level onwards". Level 0 is never
     * touched by a drop, whatever this says.
     */
    std::size_t from_ratio = 0;
};

/**
 * @brief The drops that suit anything op_facade.hpp built
 *
 * The RETURNS only -- reveal, sill and threshold -- from the first simplified
 * level. Nothing that covers an opening is ever named here: see
 * default_covering_plane_keys() for what that is and why.
 *
 * Derived from op_facade.hpp's own facade_material(), not from copied numbers, so
 * a change to FacadePart's values cannot leave a stale table here.
 *
 * @return The default list, freshly built. Callers are free to edit it.
 */
[[nodiscard]] std::vector<DetailDrop> default_facade_detail_drops();

/**
 * @brief The materials that stand between a viewer and the inside of a building
 *
 * The frame and the glazing tile a window's opening exactly between them; the
 * leaf fills a door's. Take any of the three away and the wall has a hole in it
 * that the inside of the building is visible through.
 *
 * The list is where the chain's rays are cast FROM. One ray per covering triangle
 * of level 0, four points across it, aimed out of the building along that
 * triangle's own normal: a candidate level that has nothing at or in front of
 * those points has a hole where level 0 had a window, and it is refused.
 *
 * Asked that way rather than by comparing areas or triangle counts because both
 * of those give the wrong answer on this geometry, in both directions. The
 * constrained pass collapses the glazing into the frame -- the glazing key's area
 * goes from 11.400 to 0.000 m2 and the opening is still covered. The sloppy pass
 * welds the wall shut over the whole opening -- these keys keep 2.268 of 14.400
 * m2 between them and there is still nothing to see through. And the level that
 * really is a hole loses area in exactly the same way. Only a ray tells them
 * apart.
 *
 * MaterialId::Wall variant Panel is deliberately NOT on this list. The panel is
 * the wall, it is the surface the chain exists to decimate, and a wall that
 * closes its own opening as it coarsens has lost nothing visible at the distance
 * a coarse level is drawn from -- measured, the sloppy pass over undropped facade
 * geometry welds every opening shut and 0 of 2400 rays get through.
 *
 * @return The default list, freshly built. Callers are free to edit it; an empty
 *         list turns both mechanisms off, which is what a caller building a
 *         deliberate billboard or an impostor wants.
 */
[[nodiscard]] std::vector<MaterialKey> default_covering_plane_keys();

/**
 * @brief Remove every triangle whose material range is named in @p keys
 *
 * The drop step on its own, exposed because it is exactly testable -- the covered
 * area of an opening before and after is a number with one right answer -- and
 * because a caller building a billboard or an impostor wants it without the rest
 * of the chain.
 *
 * Ranges that survive are coalesced into one SubMesh per MaterialKey, ascending
 * by MaterialKey::packed(), the same layout build_building_lod() gives level 0.
 *
 * Triangles that NO range claims are carried under MaterialId::Default, which is
 * what Mesh::sort_submeshes_by_material() does with the same case and what keeps
 * a roof from disappearing here -- see triangle_keys() in building_lod.cpp for
 * the measurement. Naming the Default key drops them like any other.
 *
 * @param mesh Source. Not modified.
 * @param keys Materials to remove. An empty list returns the coalesced input.
 * @return The surviving geometry. An EMPTY mesh when @p mesh has no triangles, no
 *         vertices, or indices that do not address its own vertex array -- the
 *         same refusal build_building_lod() makes, because this walks the same
 *         vertex array by index and would otherwise read out of bounds.
 *
 * @note The VERTEX ARRAY IS NOT COMPACTED: the result's indices still address
 *       @p mesh's vertex array, unchanged and in the same order. That is what
 *       lets a lock array or a position remap built against @p mesh stay valid
 *       across the drop, which is how the chain uses this. A caller that wants
 *       the vertices compacted should run meshopt_optimizeVertexFetch, or
 *       osm::road::optimize_mesh(), over the result.
 */
[[nodiscard]] Mesh drop_detail_keys(const Mesh& mesh, const std::vector<MaterialKey>& keys);

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Ratios, error bound, ground band and detail drops for one building
 */
struct BuildingLodConfig {
    /**
     * @brief Target index fraction of each level below level 0
     *
     * Level 0 is implied, so `ratios[i]` is the target for the candidate level
     * `i + 1` and is always measured against LEVEL 0. A ratio outside (0, 1) is
     * skipped with a warning.
     *
     * They should also be strictly decreasing. That is not enforced, because it
     * does not need to be: a ratio that does not ask for less than the one before
     * it produces a level that cannot beat the previous one by kMinLevelReduction
     * and is dropped, which is the same outcome by a different route.
     *
     * An empty vector produces a chain holding only level 0, which is still the
     * coalesced, reordered mesh and is still worth having.
     */
    std::vector<float> ratios = {0.5f, 0.25f, 0.1f};

    /**
     * @brief meshopt_simplify error bound, relative to the mesh extent
     *
     * The same 0.05 ChunkLodConfig uses, and for a building it is close to inert
     * for the same reason it is nearly inert there: what stops a level short is
     * the geometry's topology, not the error budget. Measured on the facade
     * fixture the constrained pass reports a relative error of 0.013 while
     * stalling at 82% of level 0, so it had 0.037 of headroom it could not use.
     *
     * A building is 10 to 30 m across, so 0.05 of its extent is half a metre to a
     * metre and a half. That sounds enormous next to a 0.15 m reveal, and it is
     * -- which is the argument for removing the reveal by name rather than
     * hoping an error bound protects it.
     */
    float target_error = 0.05f;

    /**
     * @brief Height above the lowest vertex within which a vertex is pinned, metres
     *
     * The building-specific replacement for ChunkLodConfig::border_band. A vertex
     * whose y is at most `bounds.min.y + ground_band` may not move at any level,
     * so the base ring that meets the terrain is where level 0 put it however
     * coarse the level is.
     *
     * 0.05 m. Large enough to catch a base ring whose vertices have been through
     * a projection and a triangulation, small enough that it cannot reach the
     * first floor of anything.
     *
     * Non-positive locks nothing. On the tower fixture that loses 25 of the base
     * ring's 32 positions by the coarsest level, which is what this setting
     * exists to prevent and what the suite asserts in both directions.
     *
     * @note Measured against the mesh's own lowest vertex, not against a terrain
     *       height that this file has no access to. A building whose mesh does
     *       not reach its own foundation cannot be helped from here.
     */
    float ground_band = 0.05f;

    /**
     * @brief Materials removed whole, rather than decimated
     *
     * Defaults to default_facade_detail_drops(). Set it empty to decimate
     * everything and drop nothing, which is the honest thing to do for a mesh
     * that did not come from the rule language and carries no facade variants --
     * the drop then matches nothing and costs one pass over the ranges.
     */
    std::vector<DetailDrop> detail_drops = default_facade_detail_drops();

    /**
     * @brief The materials whose cover a level is not allowed to lose
     *
     * Defaults to default_covering_plane_keys(), which is where the mechanism and
     * the measurement behind it are written down. Set it EMPTY to let the
     * simplifier treat an opening's cover as ordinary geometry, which on the
     * facade fixture opens a quarter of the front wall by the coarsest level and
     * is what a caller deliberately building a billboard or an impostor wants.
     *
     * A key named BOTH here and in @ref detail_drops is a contradiction, and it is
     * resolved in favour of the guard: the drop removes the cover, the guard sees
     * the hole, and every level from that drop's ratio is refused. A warning says
     * so by name. Clearing this list is how a caller asks for the hole on purpose;
     * a detail drop is not, because that is exactly the mistake the default table
     * used to make.
     */
    std::vector<MaterialKey> covering_keys = default_covering_plane_keys();

    /**
     * @brief Let a level that missed its target retry with meshopt_simplifySloppy
     *
     * ON, which reverses LodConfig::allow_sloppy's default, on measurement: see
     * "Its geometry is a QUILT" in the header note for the 82%-against-10% that
     * decides it.
     *
     * The three guards LodConfig::allow_sloppy describes are kept exactly. The
     * retry runs only when the constrained pass missed its target; its result is
     * kept only when it is strictly smaller than the constrained result; and the
     * ground lock is passed to it, so a sloppy level cannot lift the building off
     * the ground either.
     *
     * Turning it off gives a chain whose levels are all within a few percent of
     * level 0 on rule-generated geometry, and a correct, topology-preserving one
     * on anything welded and closed.
     */
    bool allow_sloppy = true;
};

// ============================================================================
// Output
// ============================================================================

/**
 * @brief The building at several levels of detail, with suggested switch distances
 */
struct BuildingLod {
    /**
     * @brief The levels, coarsening with index
     *
     * `levels[0]` is the input mesh with its index ranges coalesced to one per
     * MaterialKey and reordered for the GPU. It is never simplified, never
     * welded, and no detail drop applies to it.
     *
     * Size is `1 + n`, where n is the number of ratios that produced a usable
     * level. A ratio that yields no triangles, or that fails to beat the previous
     * level by a worthwhile margin, is dropped along with its switch distance --
     * lod_chunk.hpp's rule, kept.
     *
     * A ratio is ALSO dropped when no candidate for it could be simplified
     * without opening an opening the building had covered. That is this file's
     * own rule and it is the one that decides the length of the default facade
     * chain: three levels rather than four. A chain that is one level short is a
     * chain; a coarse level with holes in the front of the building is a defect.
     *
     * An empty `levels` means the input was unusable: no triangles, or indices
     * that do not address its own vertex array.
     */
    std::vector<Mesh> levels;

    /**
     * @brief Camera distance in metres at which each level becomes the one to draw
     *
     * Always the same length as `levels`, ascending, with `switch_distances[0]`
     * equal to 0.
     *
     * @code
     *     switch_distances[i] = bounds.radius() * kBuildingLodSwitchFactor
     *                             / std::sqrt(achieved_ratio_of_level_i)
     * @endcode
     *
     * Achieved, not requested: a level that stalled at 82% must not be switched
     * to at the distance a 50% level would have earned. Projected area falls off
     * as `1 / distance^2`, so halving the triangles is paid for at `sqrt(2)`
     * times the distance.
     *
     * Suggested, not enforced.
     */
    std::vector<float> switch_distances;

    /**
     * @brief How many levels were produced by meshopt_simplifySloppy
     *
     * The counterpart of LodChain::sloppy_simplifications, counted per level
     * rather than per material range because this file never simplifies per
     * range. Zero means every level preserves the input's topology.
     *
     * A non-zero count is expected on rule-generated geometry and is not an
     * error, but it IS a statement: those levels may have had shells welded
     * together. Anything that needs the topology -- a collision hull, an export
     * that promises manifoldness -- should use level 0.
     *
     * ZERO on the default facade chain, which is not what an earlier version of
     * this header implied. The sloppy result is offered first and then measured,
     * and on dropped facade geometry it loses the openings and the chain retreats
     * to the constrained result. It is non-zero on the same fixture with the
     * drops cleared, where the returns are still there for the clusterer to weld
     * across.
     */
    std::size_t sloppy_levels = 0;

    /// True when there is at least one level to draw
    [[nodiscard]] bool is_valid() const { return !levels.empty(); }

    /**
     * @brief Index of the level to draw at a given camera distance
     *
     * The coarsest level whose switch distance is at or below @p distance_m.
     *
     * @param distance_m Camera-to-building distance in metres
     * @return Index into `levels`. 0 for an empty chain, so a caller must still
     *         check is_valid() before indexing.
     */
    [[nodiscard]] std::size_t level_for_distance(float distance_m) const {
        std::size_t chosen = 0;
        for (std::size_t i = 0; i < switch_distances.size() && i < levels.size(); ++i) {
            if (distance_m >= switch_distances[i]) {
                chosen = i;
            }
        }
        return chosen;
    }
};

/**
 * @brief Distance-scale constant behind BuildingLod::switch_distances
 *
 * Deliberately the same number as osm::road::kLodSwitchFactor, so a building and
 * a road piece of the same bounding radius drop to the same fraction of their
 * triangles at the same distance. lod_chunk.hpp warns against mixing two
 * distance scales in one selection pass; agreeing on the constant is how this
 * file avoids being the second scale. A static_assert in building_lod.cpp fails
 * the build if the road's constant is ever retuned without this one.
 */
inline constexpr float kBuildingLodSwitchFactor = 8.0f;

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Build the LOD chain for one building mesh
 *
 * Steps:
 *
 * 1. Coalesce @p building's index ranges to one per MaterialKey, reorder for the
 *    GPU, and keep that as `levels[0]`. Never simplified.
 * 2. Weld a SEPARATE simplification base from it, ignoring UVs, tangents and
 *    vertex colour. See below -- this is the one structural difference from
 *    lod_chunk.cpp's pipeline and it is not optional.
 * 3. Lock every vertex of the base within BuildingLodConfig::ground_band of the
 *    lowest one, grow the lock by one triangle ring, and propagate it to every
 *    wedge of a locked position.
 * 4. For each ratio: drop the detail keys due at that ratio, simplify what is
 *    left to `ratio * level_0_triangles` with the lock applied, retry sloppy if
 *    the constrained pass missed and BuildingLodConfig::allow_sloppy allows,
 *    re-attribute the surviving triangles to materials, CHECK that the level did
 *    not open an opening -- retreating from a sloppy result to the constrained
 *    one if it did, and refusing the level outright if that fails too -- and keep
 *    what is left if it beat the previous level.
 *
 * ### Why the weld is not applied to level 0
 *
 * lod_chunk.cpp welds its merged chunk and ships that welded mesh as level 0.
 * This file must not, and the reason is in shape.hpp: a rule-generated mesh
 * duplicates vertices per face for flat shading and gives each face a planar UV
 * projection in ITS OWN basis. So every shared edge in the building is a UV seam,
 * and WeldConfig::uv_epsilon -- which exists to stop a road's arc-length V from
 * being welded across a junction restart -- refuses every weld in the building.
 * Measured on the facade fixture, weld_vertices() with the shipping defaults
 * removes **0 of 802 vertices**, and a simplifier handed that mesh has no
 * interior edge to collapse and returns level 0 three times over.
 *
 * Relaxing the UV and tangent tests takes the same mesh to 592 vertices and makes
 * simplification possible at all. But a mesh welded that way has had UV seams
 * closed, and closing a UV seam on the mesh the player stands in front of
 * stretches its texture across the join. So the relaxed weld produces the
 * SIMPLIFICATION BASE only. Level 0 keeps every seam it was given, and the coarse
 * levels -- which are what is drawn from far enough away that the facade texture
 * is a few pixels wide -- carry the closed ones. Normals and material masks are
 * still respected by the weld, so a crease between a wall and a roof is never
 * welded smooth.
 *
 * @param building Source geometry, in world metres, Y up. Not modified. Its
 *                 SubMesh ranges are read; an empty `submeshes` is the implicit
 *                 whole-mesh Default range renderer/mesh.hpp defines.
 * @param cfg      Ratios, error bound, ground band and detail drops
 * @return The chain. `levels.empty()` when @p building has no usable triangles.
 *
 * @note Deterministic. Material keys are visited in ascending
 *       MaterialKey::packed() order, the weld is order-independent by
 *       construction, and meshoptimizer is deterministic, so two runs over the
 *       same input produce identical output and a cache may key on it.
 */
[[nodiscard]] BuildingLod build_building_lod(const Mesh& building,
                                             const BuildingLodConfig& cfg = {});

} // namespace stratum::procgen
