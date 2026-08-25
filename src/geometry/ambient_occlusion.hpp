/**
 * @file ambient_occlusion.hpp
 * @brief Bake per-vertex ambient occlusion into a Mesh
 *
 * ### Why baked, and not screen space
 *
 * The alternative was SSAO, and it loses on every axis that matters here. It
 * needs a depth prepass, because the colour pass's depth buffer is an attachment
 * it is simultaneously writing and, when MSAA is on, is not sampleable at all;
 * that prepass is a second full replay of the visible set, every frame, forever.
 * Baking is paid once, at build time, on a tool whose entire purpose is producing
 * assets for another engine to load -- and the result travels WITH the mesh into
 * that engine, which a screen-space effect cannot do.
 *
 * ### What it darkens
 *
 * Exactly the contacts a directional sun and a hemisphere sky cannot: the foot of
 * a wall, the inside corner where two buildings meet, the well between a kerb face
 * and the road, the floor of a valley. Those are the places the renderer currently
 * looks wrong -- objects appear to hover, because nothing marks where they meet.
 *
 * ### It attenuates AMBIENT only
 *
 * The value lands in Vertex::ao, which mesh_pbr.frag multiplies into the same `ao`
 * the material and the ORM texture feed, and that term scales the sky term alone.
 * A surface in direct sun does not get darker for being near a wall -- it gets a
 * shadow, which is a different mechanism entirely and is already handled.
 */

#pragma once

#include "renderer/mesh.hpp"

#include <cstdint>

namespace stratum::geometry {

/**
 * @brief Tunables for bake_ambient_occlusion()
 */
struct AOSettings {
    /**
     * @brief Hemisphere rays cast per vertex
     *
     * The cost is linear in this and the quality is not: the sampling is
     * cosine-weighted and low-discrepancy, so 8 is already smooth on flat
     * surfaces and 16 is close to converged in a corner. Above ~32 the difference
     * stops being visible.
     */
    int ray_count = 12;

    /**
     * @brief How far a ray looks for an occluder, in metres
     *
     * This is the single most important setting, and it is a LOOK control rather
     * than an accuracy one. Small values give tight contact darkening; large
     * values start shading whole streets as though they were interiors. For
     * building and road scale, single-digit metres is the useful range.
     */
    float max_distance = 8.0f;

    /**
     * @brief How far along the normal a ray starts, in metres
     *
     * Without it every ray immediately hits the triangle it started on and the
     * whole mesh bakes to black. It has to exceed the geometry's own coordinate
     * precision, which after a Mercator projection to local metres is generous.
     */
    float bias = 0.02f;

    /// Darkest value any vertex may reach. A true 0 reads as a hole, not a corner.
    float min_ao = 0.15f;

    /// 0 bakes nothing (every vertex stays 1), 1 is the full computed occlusion.
    float strength = 1.0f;

    /**
     * @brief Treat everything below @ref ground_height as solid
     *
     * An OSM building mesh is walls and a roof, with NO FLOOR -- so a wall's base
     * has nothing beneath it in its own mesh to be occluded by, and the one place
     * the eye most expects contact darkening gets none. This closes the ground
     * analytically rather than by welding a slab under every footprint.
     *
     * OFF for a terrain mesh, obviously: there the ground IS the geometry, and a
     * plane through it would occlude every downhill ray.
     */
    bool use_ground_plane = false;

    /// World Y of that plane. Only read when @ref use_ground_plane is true.
    float ground_height = 0.0f;

    /**
     * @brief Worker threads. 0 asks the hardware.
     *
     * Vertices are independent and each is written exactly once, so threading
     * changes nothing about the result -- the bake is deterministic at any thread
     * count, which is what the golden tests rely on.
     */
    int thread_count = 0;
};

/**
 * @brief Bake ambient occlusion into @p mesh, in place
 *
 * Writes Vertex::ao for every vertex and touches nothing else. Occluders are the
 * mesh's OWN triangles, so a tile holding many buildings shades them against each
 * other, which is where most of the useful darkening comes from.
 *
 * A mesh with no indices, or with @ref AOSettings::strength at 0, is left with
 * every ao at 1 -- the value that means "no occlusion" and reproduces the
 * behaviour from before this channel existed.
 *
 * @param mesh     Mesh to modify. Its normals must already be valid.
 * @param settings Tunables; the defaults suit building and road scale.
 */
void bake_ambient_occlusion(Mesh& mesh, const AOSettings& settings = {});

} // namespace stratum::geometry
