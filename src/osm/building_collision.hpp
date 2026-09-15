// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file building_collision.hpp
 * @brief The building a physics engine wants: a closed prism, and no roof at all
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Roads derive their collision surface from the finished render mesh, by
 * deletion and simplification -- see osm/road/collision_mesh.hpp, which argues
 * that building the physics geometry independently would let the two drift.
 * Buildings go the other way and are built straight from the Building footprint.
 * That is the opposite decision to the one taken next door, so it needs a
 * reason, and there are three.
 *
 * ### 1. The render mesh has nothing the road derivation can delete
 *
 * build_collision_mesh() deletes by MaterialId, and MeshBuilder::build_building_mesh()
 * tags nothing: it leaves `submeshes` empty, so every triangle of a building
 * reads as MaterialId::Default. None of the material rules fire. Its other lever
 * is simplification, which runs with LodConfig::lock_borders, and a roof's eaves
 * ring is a border: a dome's six latitude bands and a hipped roof's ridge come
 * through nearly intact. Its vertical-face rule is built for a 150 mm kerb, so a
 * 10 m facade is correctly classified as a WALL -- and walls are held out of the
 * simplification entirely. The derivation therefore keeps the whole expensive
 * roof and the whole facade, and returns roughly what it was given.
 *
 * ### 2. The render mesh is wrong at a courtyard, and a derivation inherits it
 *
 * build_building_mesh() extrudes walls from `Building::footprint` ONLY.
 * `Building::holes` reach geometry through the roof triangulation and nowhere
 * else. A courtyard block therefore renders as a roof with a hole in it and no
 * wall anywhere around that hole, which is invisible from a helicopter and fatal
 * to a collider: a character walking the roof steps into the courtyard and falls
 * through a surface that was never emitted. There is also no floor at all, at
 * any point, because nobody sees the underside of a building.
 *
 * Deriving from that mesh inherits both faults. Walking the rings directly does
 * not: a hole ring gets its own wall, wound so the facade faces INTO the
 * courtyard, and the caps are triangulated with the holes cut out of them.
 *
 * ### 3. The counts
 *
 * A 16-sided footprint with roof:shape=dome renders as 208 triangles and 592
 * vertices: 32 wall triangles and six latitude bands of roof. Running the road
 * derivation over it gives 166 triangles, because the roof is what it cannot
 * reach. The hull is 60 triangles and 32 vertices -- the same 32 walls, plus two
 * 14-triangle caps. The saving is entirely in the roof, which is the point, and
 * the roof is what a collision mesh has the least use for.
 *
 * It does not save on every building, and it is not meant to. A four-sided
 * flat-roofed box renders as 10 triangles and hulls as 12, and a courtyard box
 * renders as 16 and hulls as 32 -- because most of what the hull adds there is
 * surface the render mesh does not have at all: the courtyard's four walls and
 * the floor.
 *
 * ### What this gives up, on purpose
 *
 * The hull caps at the EAVES, at `Building::height`. Above that line there is no
 * collision: a character on a pitched roof stands on the eaves plane with the
 * render roof passing through them, and a projectile aimed at a gable passes
 * through it. That is the standard simplified-collision compromise and it is
 * chosen here for a specific reason. The five roof rises are constants inside
 * mesh_builder.cpp -- a pitch ratio, a hip inset, an inradius -- and a second
 * copy of them in this file would disagree with the first the day one of them
 * is tuned. BuildingCollisionConfig::cap_height is the escape hatch for a caller
 * that has the render mesh in hand and can measure its peak rather than predict
 * it.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API and
 * no physics engine. The output is a plain Mesh and the consuming engine builds
 * its own broadphase from it.
 */

#pragma once

#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <optional>

namespace stratum::osm {

/**
 * @brief How much of a hull to build, and where to put its two caps
 */
struct BuildingCollisionConfig {
    /**
     * @brief Emit the downward-facing floor that closes the prism
     *
     * On by default, and the extra cost is the cheapest part of the mesh: the
     * floor is the same triangle count as the roof cap and no vertices of its
     * own, because it shares the wall ring's lower vertices.
     *
     * What it buys is that the output is a CLOSED surface. Convex decomposition
     * (VHACD and everything descended from it), signed-distance baking and
     * point-in-volume queries all assume a closed input and produce silent
     * nonsense on an open one. An open-bottomed prism is also a bucket: anything
     * that gets inside it, by a spawn, a teleport or a bad frame, is standing on
     * the terrain inside a wall it cannot pass.
     *
     * Turn it off only for a consumer that closes volumes itself, or one that
     * raycasts downward exclusively and never from below.
     */
    bool close_bottom = true;

    /**
     * @brief Metres to sink the floor below y = 0, where the footprint sits
     *
     * Zero puts the floor exactly on the plane the render mesh's walls stand on,
     * which is right for flat ground. On terrain the footprint plane is a fiction
     * and a building on a slope has its uphill corner buried and its downhill
     * corner in the air; sinking the floor by more than the fall across the
     * footprint keeps the hull's walls continuous with the ground on every side.
     *
     * It costs nothing but the wall area it adds, and that area is underground.
     */
    float foundation_depth = 0.0f;

    /**
     * @brief Cap height in metres, or unset for the eaves at Building::height
     *
     * Set it only when the caller can MEASURE the roof rather than predict it --
     * from the bounds of the render mesh, typically. Predicting it means copying
     * mesh_builder.cpp's five roof-rise constants into a second file, and the
     * two copies stop agreeing the first time one of them is tuned.
     *
     * A value at or below the floor produces an empty mesh rather than an
     * inside-out prism.
     */
    std::optional<float> cap_height;
};

/**
 * @brief Plan area below which a ring is degenerate, square metres
 *
 * A square millimetre. Under this a ring has no interior to extrude and no
 * triangulation worth the name: an outer ring this small yields an empty mesh,
 * and a hole this small is dropped rather than being handed to the
 * triangulator, where a zero-area ring produces a triangulation whose boundary
 * is not the ring and the hull stops being closed.
 *
 * Exposed so the tests assert the threshold rather than a magic number.
 */
inline constexpr double kMinRingArea = 1e-6;

/**
 * @brief Build a closed collision prism from a building footprint
 *
 * ### Steps
 *
 * 1. **Normalise the rings.** The closing duplicate an OSM way carries is
 *    dropped, consecutive coincident points are dropped, and each ring's winding
 *    is forced: the outer ring counter-clockwise in local metres, every hole
 *    clockwise. Winding is FORCED rather than trusted, because Building
 *    documents the convention and an extract does not honour it; a hole that
 *    arrives counter-clockwise would otherwise get a facade pointing into the
 *    masonry.
 * 2. **Triangulate the cap** once, outer ring with the holes cut out, and fix
 *    each triangle's winding from its own plan-view signed area rather than
 *    trusting the triangulator's convention.
 * 3. **Emit two vertices per ring point**, one at the floor and one at the cap,
 *    and index the walls and both caps off that single set. Sharing is what
 *    makes the output closed by INDEX and not merely by position, so a consumer
 *    needs no weld pass to get a manifold.
 * 4. **Wind the walls outward**, which for a hole ring means into the courtyard.
 *
 * ### Output shape
 *
 * `submeshes` is left EMPTY, the implicit single MaterialId::Default range over
 * the whole index buffer, exactly as osm::road::build_collision_mesh() leaves
 * it. A physics mesh has one surface type here or it gets its surface types from
 * a different mechanism entirely.
 *
 * Every vertex normal is the outward HORIZONTAL normal of the facade at that
 * ring point. That is exactly right for the walls, which are most of the
 * surface, and wrong for the two caps, whose triangles end up with normals
 * tangent to themselves. This is deliberate and should not be "fixed": a closed
 * prism shares its corner vertices, a shared vertex has one normal, and giving
 * the caps their own would double the vertex count and break the manifold to buy
 * nothing -- a physics engine takes the geometric normal from the positions, and
 * this mesh is never rendered. UVs are a planar projection in metres, present so
 * that they are deterministic rather than because anything reads them.
 *
 * Winding is outward everywhere: walls face away from the masonry, the roof cap
 * faces up, the floor faces down.
 *
 * @param building Footprint, holes and height. Not modified.
 * @param cfg      Which caps to emit and where to put them
 * @return A prism -- closed, unless BuildingCollisionConfig::close_bottom was
 *         cleared -- or an EMPTY mesh. The output is never a partial hull:
 *         a footprint with fewer than three distinct points, a plan area under
 *         kMinRingArea, a cap at or below the floor, or a triangulation that
 *         failed all return nothing. An open-topped shell would be worse than no
 *         collider at all, because a collider that is missing is noticed the
 *         first time someone walks at the building and a bucket is not.
 */
[[nodiscard]] Mesh build_building_collision_mesh(const Building& building,
                                                 const BuildingCollisionConfig& cfg = {});

} // namespace stratum::osm
