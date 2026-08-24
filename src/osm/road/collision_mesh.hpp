/**
 * @file collision_mesh.hpp
 * @brief The road as a physics engine wants it: a surface, not a model
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The render mesh is the wrong shape for collision, and not because it is too
 * detailed. It is the wrong shape because most of what is in it is not a surface
 * anything walks or drives on.
 *
 * - **Paint is not geometry.** Marking quads float a few millimetres above the
 *   carriageway. In a render mesh that is a depth-bias trick; in a collision mesh
 *   it is a second floor 5 mm above the first, and a character controller or a
 *   wheel raycast will find it, jitter between the two, and stick.
 * - **A kerb face is not a wall.** It is a 150 mm step. Left in the mesh it is a
 *   vertical surface, and a vertical surface is something a physics engine slides
 *   along instead of stepping over. So it is deleted -- and its plan-view
 *   footprint is replaced by a horizontal sliver at road level, because the
 *   default profile leans the face outward by ProfileConfig::curb_face_batter and
 *   deleting it outright would leave a 20 mm slit in the ground that a downward
 *   wheel raycast falls straight through. What remains is a 150 mm STEP between
 *   two continuous surfaces, which every step-height implementation in every
 *   engine already knows how to handle.
 * - **Material slots are meaningless.** A physics mesh has one surface type here,
 *   or it has surface types decided by a different mechanism entirely. Carrying
 *   eleven ranges through costs draw-call bookkeeping that nothing reads.
 *
 * So the collision variant is a derivation, not a second build: it is produced
 * from the finished render mesh by deletion and simplification. That is
 * deliberate. Building it independently from the profile would let the two drift,
 * and a collision surface that disagrees with the visible road is worse than no
 * collision surface at all.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API, and no physics engine either -- the output is a plain Mesh and the
 * consuming engine builds its own broadphase from it.
 */

#pragma once

#include "osm/road/mesh_optimize.hpp"
#include "renderer/mesh.hpp"

#include <cstddef>

namespace stratum::osm::road {

/**
 * @brief What to keep, what to delete, and how hard to simplify
 */
struct CollisionConfig {
    /**
     * @brief Keep MaterialId::Sidewalk triangles
     *
     * On for anything with pedestrians. Off for a driving-only game, where the
     * footway is scenery and removing it is free.
     */
    bool include_sidewalk = true;

    /**
     * @brief Keep the horizontal top of the kerb
     *
     * MaterialId::Curb covers both the top and the face, and they are told apart
     * by orientation, not by material: a Curb triangle is a TOP when the absolute
     * Y component of its geometric normal is at least kVerticalNormalY, and a FACE
     * otherwise. This flag controls the tops. The faces are controlled by
     * drop_vertical_faces.
     *
     * With this off and include_sidewalk on, the footway floats with a 150 mm gap
     * along its road edge, which is usually fine and occasionally is not.
     */
    bool include_curb_top = true;

    /**
     * @brief Target triangle fraction of the surviving surface
     *
     * Applied AFTER the deletions, so it is a fraction of what is left, not of the
     * render mesh. Simplification uses LodConfig::lock_borders, so chunk seams and
     * the outer footprint stay put -- a collision mesh that has pulled back from
     * its own edge drops things through the world at the boundary.
     *
     * 0.3 is aggressive on purpose. A road surface is nearly developable and loses
     * almost nothing at a third of the triangles. Set to 1.0, or anything above
     * it, to skip simplification entirely; anything at or below 0 is clamped to
     * 0.01 rather than being read as "ask for no triangles at all".
     */
    float simplify_ratio = 0.3f;

    /// Delete MaterialId::Markings triangles. Paint is not collidable.
    bool drop_markings = true;

    /**
     * @brief Delete near-vertical triangles shorter than max_step_height
     *
     * A triangle is near-vertical when the absolute Y component of its geometric
     * normal is below kVerticalNormalY.
     *
     * Height is what decides, not material. A kerb face is 150 mm and becomes a
     * step; a bridge parapet, a tunnel headwall or a retaining face is metres tall
     * and is a REAL wall that must stay, or vehicles drive off the bridge.
     *
     * The height that decides is measured LOCALLY, and neither of the two obvious
     * measures works:
     *
     * - The single triangle's own vertical extent deletes a tall wall a slice at a
     *   time, because every slice of a triangulated parapet is short.
     * - The whole connected patch's extent keeps every kerb on every hill, because
     *   a kerb is one continuous patch and running 40 m downhill gives that patch
     *   a 40 m vertical extent.
     *
     * So for each vertex of a connected vertical patch, the extent of the patch
     * within kPatchWindowRadius of it IN PLAN is measured, and a triangle's step
     * height is the largest of the three windows at its own vertices. A kerb
     * reads 150 mm on any slope; a parapet reads its full height however it is
     * cut. A triangle is deleted when that step height is at most
     * max_step_height.
     *
     * The patch bounds each window to one surface. It is deliberately NOT the
     * unit of the decision: patches unite by shared position alone, so a bridge
     * deck's end cap -- deck_thickness deep, and touching the kerb face at the
     * first and last station of the edge -- would otherwise promote every kerb
     * on that bridge to a wall for the whole length of the run. A patch can
     * therefore be split between kept and dropped, at the triangle where the
     * window stops seeing the tall member.
     *
     * Every deleted triangle is then BRIDGED: its plan-view footprint is re-emitted
     * as a horizontal triangle at the local floor of its patch, wound upward, so
     * the surface stays continuous across the step instead of carrying a slit the
     * width of the kerb batter. A face with no batter projects to zero plan area,
     * leaves no gap, and is bridged by nothing.
     */
    bool drop_vertical_faces = true;

    /**
     * @brief Tallest vertical face treated as a step rather than a wall, metres
     *
     * 0.2 clears the 150 mm kerb of the residential profile with margin, and sits
     * below the step height every mainstream character controller defaults to.
     * Raising it past roughly 0.4 starts deleting real retaining faces.
     */
    float max_step_height = 0.2f;
};

/**
 * @brief Absolute normal Y at or above which a triangle counts as walkable-flat
 *
 * Below this a triangle is treated as a vertical face. 0.35 is about 70 degrees
 * from horizontal: steeper than any road surface the grade limiter permits (15%
 * on a path is 0.99) and shallower than a kerb face, a parapet or a headwall.
 * Exposed so the tests assert the classification rather than a magic number.
 */
inline constexpr float kVerticalNormalY = 0.35f;

/**
 * @brief Plan-view radius the local height of a vertical patch is measured over
 *
 * See CollisionConfig::drop_vertical_faces for why the measure is windowed at
 * all. The value is bracketed from both sides: it must exceed the plan thickness
 * of a real wall, so a wall cut into short strips still reports its full height,
 * and it must be small enough that the longitudinal fall of a kerb across the
 * window stays well under CollisionConfig::max_step_height. At 0.25 m a kerb on
 * the steepest grade the elevation solver permits -- 15% on a path -- reports
 * 0.15 + 0.0375 m and is still classified as a step.
 *
 * Exposed so the tests assert the classification rather than a magic number.
 */
inline constexpr float kPatchWindowRadius = 0.25f;

/**
 * @brief Derive a collision surface from a finished render mesh
 *
 * ### Steps
 *
 * 1. **Classify** every triangle by its SubMesh material and its geometric
 *    normal, computed from the vertex positions rather than read from the vertex
 *    normals, because a welded vertex normal is an average and a face is not.
 * 2. **Delete** the triangles ruled out by @p cfg: Markings when drop_markings;
 *    Sidewalk when not include_sidewalk; horizontal Curb when not
 *    include_curb_top; near-vertical patches whose local step height is at most
 *    max_step_height when drop_vertical_faces. Degenerate triangles -- zero area,
 *    so no orientation and nothing to collide with -- go too.
 * 3. **Bridge** each deleted step with its plan footprint laid flat at the local
 *    floor of its patch, so no deletion leaves a slit in the ground.
 * 4. **Compact** the vertex array, dropping vertices nothing references any more.
 * 5. **Weld** with `WeldConfig{respect_material = false}` and every tolerance
 *    opened up: a coarse position epsilon, and normal, UV, colour and tangent
 *    epsilons set past the value that ignores the attribute. Materials, shading
 *    creases and UV seams are all meaningless in a physics mesh, so there is
 *    nothing left to protect, and welding across what used to be material and
 *    seam boundaries is exactly what turns the remaining islands into one
 *    connected surface. Without it the next step has no interior edges to
 *    collapse.
 * 6. **Simplify** to CollisionConfig::simplify_ratio with borders locked, unless
 *    the ratio is at or above 1. The candidate is then MEASURED against the
 *    unsimplified surface -- total plan area, and count of open boundary edges --
 *    and rejected if it lost plan area or grew boundary, because either means
 *    simplification opened a hole. A rejection logs and keeps the unsimplified
 *    surface: slower collision geometry is a cost, a hole is a fall through the
 *    world.
 *
 *    The walls kept by drop_vertical_faces do not go through it at all. The
 *    surface is split first: the walkable part is simplified and measured, the
 *    near-vertical part is concatenated back untouched, and the two are welded
 *    together again. That guard measures PLAN area, and a wall covers no plan,
 *    so a simplifier is free to delete a tunnel headwall outright without moving
 *    the measurement -- which is the one deletion this file exists to refuse.
 * 7. **Recompute** bounds.
 *
 * ### Output shape
 *
 * `submeshes` is left EMPTY, which is the implicit single MaterialId::Default
 * range over the whole index buffer -- Mesh::effective_submeshes() reports exactly
 * one range. Materials have collapsed and nothing downstream should branch on
 * them.
 *
 * Winding is preserved from the render mesh, so surviving triangles still face
 * the way they faced. Vertex normals, UVs, colours and tangents are carried
 * through untouched; a physics engine ignores them, and an exporter writing this
 * mesh out for inspection does not.
 *
 * The result is NOT guaranteed watertight, manifold, or free of the small
 * degenerate triangles welding can leave behind. Physics meshes do not need to be,
 * and the guarantees that would make it so cost more than they are worth here.
 * What IS guaranteed is that no step deletion and no simplification opened a hole
 * the input did not already have: the deletions bridge their own footprint, and
 * the simplification is rejected outright when it loses plan coverage.
 *
 * @param render_mesh Finished render geometry with its SubMesh ranges intact. Not
 *                    modified.
 * @param cfg         What to keep and how hard to simplify
 * @return The collision surface. Empty when everything was deleted, which is the
 *         correct answer for a piece that was nothing but paint.
 */
[[nodiscard]] Mesh build_collision_mesh(const Mesh& render_mesh, const CollisionConfig& cfg = {});

} // namespace stratum::osm::road
