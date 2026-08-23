/**
 * @file carve_request.hpp
 * @brief Neutral payload describing what the road network wants carved into terrain
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this header exists
 *
 * The road pipeline produces the corridor footprints, and the terrain pipeline
 * consumes them. Those are two modules with a hard layering rule between them:
 * Nothing under `src/osm/road/` may include anything under `src/procgen/`, so
 * that the whole road system stays testable against a synthetic height
 * function and never links against terrain generation.
 *
 * The payload types are therefore declared HERE, in the module that produces
 * them, and `procgen/terrain_carve.hpp` includes this header and aliases them
 * into `stratum::procgen`. That is CarveRibbon and CarveDisc, and
 * TunnelPortalFootprint, which osm/road/tunnel_builder.hpp fills and the carve
 * reads. The alternative -- mirroring the structs on both sides
 * and converting -- was rejected because a conversion function is one more thing
 * to keep in sync every time a field is added, and because it would copy every
 * outline of a city-sized import for no benefit.
 *
 * The direction is the safe one: procgen already includes `osm/types.hpp`, so
 * procgen depending on osm is an existing, deliberate edge in the dependency
 * graph. This header adds nothing to it beyond three plain-old-data structs.
 *
 * ### Coordinates
 *
 * Everything here is in the SAME 2D local metres as `Road::polyline`,
 * `GraphEdge::polyline`, `Centerline::stations` and `Corridor::outline`. It is
 * NOT render space. The consumer is responsible for the render mapping used
 * everywhere in this codebase, Y up:
 *
 * @code
 *     (x, y_2d) -> glm::vec3(x, height, -y_2d)
 * @endcode
 *
 * A terrain Heightmap is indexed in the SAME 2D local frame -- its second axis is
 * local y, and it applies that very same negation on its own way to render space
 * -- so a heightmap sample at (X, Z) corresponds to the 2D point (X, Z), with no
 * sign flip. Heights, by contrast, are already world Y in metres.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include <glm/glm.hpp>

#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Carve payload
// ============================================================================

/**
 * @brief One road corridor to carve into the terrain
 *
 * Produced once per emitted RoadPiece, after the elevation solve, and consumed
 * per terrain chunk. A ribbon is read-only during carving and is safe to share
 * across threads carving different heightmaps.
 */
struct CarveRibbon {
    /**
     * @brief Closed corridor footprint, local metres
     *
     * Copied from Corridor::outline: counter-clockwise, first point NOT repeated
     * at the end. Empty when the corridor produced no footprint, in which case
     * the carve falls back to the centerline and half_width alone.
     */
    std::vector<glm::dvec2> outline;

    /**
     * @brief The outline is a simple polygon and may be point-in-polygon tested
     *
     * Mirrors the negation of Corridor::outline_self_intersects. A hairpin
     * tighter than the profile's half width folds the inner offset back through
     * itself, and a winding test against such a ring punches holes in the
     * terrain. When this is false the carve must fall back to a distance test
     * against the centerline band instead of testing the ring. Always false when
     * outline is empty.
     */
    bool outline_is_simple = true;

    /// Centerline points, local metres; the resampled stations of the corridor
    std::vector<glm::dvec2> centerline;

    /**
     * @brief Miter scale per station, parallel to centerline
     *
     * Station::miter_scale, the `1 / cos(theta / 2)` factor the corridor extruder
     * multiplies every lateral offset by at a joint. The corridor's outer edge is
     * therefore `half_width * miter_scale` from the centerline, not `half_width`,
     * and at a sharp single-vertex deflection that is up to
     * ResampleConfig::miter_limit times further out. A carve band measured with
     * `half_width` alone leaves the outer corner of every such bend overhanging
     * terrain that was only partly blended, or not carved at all.
     *
     * Always >= 1.0. Empty, or not parallel to centerline, means "no miter
     * information": the carve then treats every station as 1.0, which is the
     * correct reading for a straight corridor.
     */
    std::vector<float> centerline_miter;

    /**
     * @brief World Y to carve the terrain TO, parallel to centerline
     *
     * Same size as centerline. This is the CARVE TARGET, not the road surface:
     * it is EdgeElevation::station_heights LESS ElevationConfig::surface_offset,
     * so the carved terrain sits that far under the carriageway rather than
     * exactly on it. Carrying the offset on both the mesh and the target would
     * lift the two together and leave them coplanar, which is precisely what the
     * offset exists to prevent. Exactly one of the pair may carry it, and it is
     * the mesh.
     */
    std::vector<float> centerline_heights;

    /**
     * @brief Half the total profile width, metres; the band radius
     *
     * Scaled by the interpolated centerline_miter at the point being carved, so
     * the band follows the same outline the extruder emitted.
     */
    float half_width = 5.0f;

    /**
     * @brief Do not carve this ribbon
     *
     * Set for tunnel edges, whose roadway is below ground and whose terrain must
     * stay intact, and for bridge spans, whose deck floats free above the
     * terrain it crosses. The ribbon is still carried so the caller can draw or
     * count it; the carve simply skips it.
     */
    bool suppress = false;
};

/**
 * @brief Junction footprint
 *
 * P3 had no fillet polygons, so a junction was carved as a flat disc covering the
 * arm end cross-sections. P4 fills `outline` with the real fillet-and-curb-ring
 * boundary, and the disc is KEPT as the degenerate fallback: a node whose trim
 * solve fails produces no polygon, and a disc covering its arm mouths is better
 * than carving nothing under an intersection. `center` and `radius` are therefore
 * always populated, whether or not `outline` is.
 *
 * See the "Note on terrain ordering" in docs/plans/road_network_plan.md. The
 * provisional footprint is a superset of the final one in the common case, so the
 * second carve is a refinement rather than a correction of visible error.
 */
struct CarveDisc {
    /// Junction centre in local metres, taken from GraphNode::position
    glm::dvec2 center{0.0};

    /**
     * @brief Radius in metres, from the widest arm profile meeting at the node
     *
     * Always populated, and always large enough to BOUND `outline` when that is
     * present. A consumer that ignores the outline entirely -- as the P3 carve
     * does -- therefore still carves a conservative superset of the junction
     * rather than missing part of it, which is what makes `outline` a safe
     * additive field rather than a breaking change.
     */
    float radius = 0.0f;

    /**
     * @brief Real junction footprint, local metres; empty on the degenerate path
     *
     * Counter-clockwise, first point NOT repeated, on the same contract as
     * CarveRibbon::outline. Copied from Junction::footprint, which is the curb
     * ring's outer boundary when a ring was built and the junction polygon ring
     * otherwise.
     *
     * A polygon-aware consumer carves this and blends outward from its edge; one
     * that is not falls back to `radius` about `center`, which bounds it.
     */
    std::vector<glm::dvec2> outline;

    /**
     * @brief The outline is a simple polygon and may be point-in-polygon tested
     *
     * Mirrors the negation of JunctionPolygon::self_intersecting. A ring that
     * crosses itself has no meaningful winding number, and a winding test against
     * one punches holes in the terrain -- the same failure CarveRibbon documents.
     * When this is false, or `outline` is empty, the carve must use `radius`.
     * Always false when `outline` is empty.
     */
    bool outline_is_simple = true;

    /**
     * @brief World Y to carve the terrain TO, the solved node height
     *
     * The carve TARGET, on the same contract as
     * CarveRibbon::centerline_heights: junction geometry is placed at this height
     * plus ElevationConfig::surface_offset, so the target itself carries no
     * offset and the surface clears the terrain by exactly that much.
     */
    float height = 0.0f;

    /// Do not carve; set when every arm at the node is a solved tunnel or bridge
    bool suppress = false;
};

// ============================================================================
// Tunnel portal hand-off
// ============================================================================

/**
 * @brief Where a portal stands, for the terrain carve to open a mouth around it
 *
 * ### Why this crosses the module boundary as data
 *
 * docs/plans/road_network_plan.md asks a tunnel to "carve a portal opening in
 * the terrain at each end", and nothing under `src/osm/road/` may include
 * anything under `src/procgen/`. So the portal is DESCRIBED here and carved
 * there, on exactly the pattern carve_request.hpp already uses for CarveRibbon
 * and CarveDisc. Coordinates are the same 2D local metres as
 * `GraphEdge::polyline`, `Centerline::stations` and `CarveRibbon::outline`;
 * heights are world Y in metres.
 *
 * ### What the terrain side must do with it
 *
 * A heightmap cannot express an overhang, so it can never hold a bore. The one
 * thing it can do, and the thing the portal needs, is to stop the hillside
 * closing over the mouth:
 *
 * 1. Take the plan rectangle `half_width` either side of `center`, running from
 *    `center` along `axis` for `depth` metres. That is the mouth footprint;
 *    `axis` points INTO the hillside.
 * 2. Inside it, CLAMP the terrain DOWN to `crown_height`, never up. Clamping
 *    rather than setting is the whole safety property: at the mouth itself the
 *    terrain is already at road level by construction, so a clamp changes
 *    nothing there and only bites where the hill has climbed above the arch
 *    within `depth` metres of the opening. A `set` would instead punch the
 *    ground down to the crown across the whole rectangle and leave a trench in
 *    front of the portal.
 * 3. Blend from the clamped height back to natural terrain outward from the
 *    rectangle over the carve's usual falloff band, so the notch has sides
 *    rather than a cliff.
 *
 * The carve must NOT lower terrain to `surface_height`. That is the road, and
 * cutting the ground to it would open the tunnel to the sky for `depth` metres.
 * `crown_height` is the top of the opening, which is the highest the ground may
 * be and still leave the arch visible.
 *
 * ### Why the headwall does not need the carve to have run
 *
 * The two are independent on purpose. The headwall already grows upward to meet
 * whatever ground it is set into, so a portal built against UNCARVED terrain is
 * correct on its own; and once the notch has clamped that ground down to
 * `crown_height`, the wall's own minimum height -- the opening plus its surround
 * -- already stands above it. Neither pass has to run before the other and
 * neither has to know whether the other did. A build that ignores these
 * footprints entirely gets a portal set into a hillside that closes over the
 * arch a few metres in; a build that honours them gets a mouth with a notch cut
 * around it. Both are coherent.
 *
 * A portal footprint is read-only during carving and is safe to share across
 * threads carving different heightmaps.
 */
struct TunnelPortalFootprint {
    /// Opening centre in local 2D metres, on the corridor centerline
    glm::dvec2 center{0.0};

    /**
     * @brief Unit direction pointing INTO the hillside, local 2D metres
     *
     * The station tangent at the portal, negated at the far end of the edge, so
     * both portals of one tunnel face outward and their axes point at each other.
     */
    glm::dvec2 axis{1.0, 0.0};

    /// Half the opening width INCLUDING the wall surround, metres
    double half_width = 0.0;

    /// How far along `axis` the notch runs; TunnelConfig::portal_cut_depth or the wall, whichever is deeper
    double depth = 0.0;

    /// World Y of the carriageway surface at the portal
    float surface_height = 0.0f;

    /// World Y of the top of the opening: `surface_height + TunnelConfig::portal_height`
    float crown_height = 0.0f;

    /// True for the portal nearer GraphEdge::from, false for the one nearer GraphEdge::to
    bool at_start = true;
};

} // namespace stratum::osm::road
