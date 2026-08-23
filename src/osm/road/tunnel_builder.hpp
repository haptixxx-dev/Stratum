/**
 * @file tunnel_builder.hpp
 * @brief Portal openings where a tunnel edge enters and leaves the ground
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * A tunnel needs less geometry than it first appears. The elevation solver
 * already drops a `tunnel=*` edge below the terrain, `CarveRibbon::suppress` is
 * already set for tunnel spans so the hillside above it stays intact, and the
 * corridor extruder already sweeps the running surface along underneath. From
 * outside, all of that is invisible except at the two ends.
 *
 * What is missing is the OPENING: at each end the road disappears into a
 * hillside, and without a portal it disappears into nothing, leaving the terrain
 * surface cutting straight through the carriageway. This builder emits the
 * headwall and side walls that frame that opening.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/carve_request.hpp"
#include "osm/road/centerline.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <functional>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Dimensions of tunnel portals. All distances are metres.
 */
struct TunnelConfig {
    /**
     * @brief Clearance added to each side of the profile to size the opening
     *
     * The opening is `RoadProfile::total_width() + 2 * portal_width_margin`
     * across, so the walls stand clear of the footway rather than on it.
     */
    float portal_width_margin = 1.0f;

    /// Height of the opening above the carriageway surface
    float portal_height = 5.0f;

    /// Thickness of the headwall and the side walls, along the direction of travel
    float wall_thickness = 0.5f;

    /**
     * @brief How far the road must sit under the terrain for a portal to be there
     *
     * The portal is placed where the solved road surface crosses BELOW the
     * terrain, and a crossing is only meaningful if it is deeper than the noise
     * either side of it. A procedural surface wanders by centimetres between
     * adjacent stations, so a threshold of exactly zero puts a headwall wherever
     * a flat tunnel and a flat hillside happen to graze each other, and puts it
     * somewhere different on the next build. A quarter of a metre is below
     * anything a viewer reads as cover and above anything the sampler produces
     * as noise.
     */
    float min_portal_cover = 0.25f;

    /**
     * @brief How far into the hillside the terrain notch runs, past the headwall
     *
     * Reported on TunnelPortalFootprint::depth, not used by the mesh. The
     * headwall itself is only TunnelConfig::wall_thickness deep, and a notch that
     * shallow is swallowed the moment the hill starts to rise. See
     * TunnelPortalFootprint for what the terrain side does with it.
     */
    float portal_cut_depth = 3.0f;

    /**
     * @brief Ceiling on headwall height above the carriageway, metres
     *
     * The headwall grows upward to meet the ground it is set into, so a portal
     * driven into a cliff would otherwise emit a wall as tall as the cliff. Past
     * this height the wall stops and the hillside simply stands above it, which
     * is what a real retained portal looks like anyway.
     */
    float max_headwall_height = 12.0f;

    /// Segments across the semicircular arch head; below 3 the opening is rectangular
    int arch_segments = 12;

    bool emit_portals = true;

    /**
     * @brief Emit the interior tube
     *
     * Out of scope for the first pass and DEFAULTED OFF for that reason. The
     * first-pass implementation ignores this flag entirely: setting it true emits
     * portals and nothing else, and does not fail. It exists so the eventual bore
     * has a switch already threaded through RoadNetworkConfig rather than to
     * promise geometry that is not there.
     */
    bool emit_bore = false;
};

// ============================================================================
// Terrain hand-off
// ============================================================================
//
// TunnelPortalFootprint -- what the terrain carve needs in order to open a mouth
// around a portal -- is declared in osm/road/carve_request.hpp, beside CarveRibbon
// and CarveDisc. It is a carve payload like those two, and the terrain module
// consumes all three through that one header; see its file comment for why the
// payload types live on the producing side of the module boundary.

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Portal geometry at each end of a tunnel edge
 *
 * ### Where a portal goes, and where one does not
 *
 * A portal is NOT the end of the edge. A `tunnel=*` way commonly starts on open
 * ground and only enters the hillside some way along it, and the elevation
 * solver ramps the roadway down over that stretch rather than dropping it
 * vertically. So the portal is found rather than assumed: walking inward from
 * each end, it is the point where the solved road surface first passes
 * TunnelConfig::min_portal_cover below `terrain_at`, interpolated between the
 * two stations that bracket the crossing.
 *
 * Three cases produce no portal at an end, and each of them is a case where a
 * headwall would be worse than none:
 *
 * - **No crossing anywhere on the edge.** A covered way, or bad data, or a
 *   tunnel the solver left above ground. Nothing is buried, so nothing needs
 *   framing.
 * - **The edge is ALREADY buried at that end.** A long tunnel is split into
 *   several GraphEdges at its interior nodes, and the ends of the middle ones
 *   are hundreds of metres underground. A portal there is a wall built across
 *   the middle of the tunnel.
 * - **The two portals are closer together than two opening widths.** Their
 *   surrounds would interpenetrate and the pair would read as a box rather than
 *   as two mouths. Nothing is emitted at either end, so a short underpass is left
 *   unframed rather than framed wrongly.
 *
 * The headwall spans from the carriageway surface up to whichever is higher, the
 * arch plus its surround or the terrain the wall is set into, so it always
 * reaches that ground and never leaves a slot of daylight above the arch. It
 * stops at TunnelConfig::max_headwall_height.
 *
 * ### What is emitted
 *
 * Per portal, all in MaterialId::Concrete:
 *
 * - **Headwall** -- a flat face perpendicular to the direction of travel, the
 *   full opening width plus its wall thickness on each side, from the carriageway
 *   surface to the terrain, with the opening itself left as a hole. It is built
 *   as two jamb panels and a strip following the arch, so the hole is a hole and
 *   no polygon triangulator is involved.
 * - **Side walls and soffit** -- the inner faces of the opening, running
 *   TunnelConfig::wall_thickness back into the hillside, so the opening reads as a
 *   thickness rather than as a cut-out in a plane.
 *
 * The running surface through the opening is NOT emitted. The corridor already
 * owns it for the whole tunnel edge, portal included, and a second copy z-fights.
 *
 * ### World mapping and winding
 *
 * The mapping is the pipeline's, Y up: `(x, y_2d) -> vec3(x, height, -y_2d)`,
 * which flips handedness. Winding is therefore not derived from the 2D ring at
 * all: every quad is emitted with the outward direction it is supposed to face,
 * and its index order is chosen from the sign of the triangle's own cross
 * product against that direction. A portal is seen from the outside AND from
 * inside the opening, so both the headwall's outward face and the side walls'
 * inward faces come out correct without a per-surface rule to get backwards.
 *
 * ### Which centerline to pass
 *
 * Pass the TRIMMED centerline, the same one the corridor was extruded from, so a
 * portal at a trimmed end lands where the ribbon actually stops.
 *
 * @param edge            Tunnel edge. Nothing is emitted when GraphEdge::is_tunnel
 *                        is false, so a misrouted edge produces nothing rather
 *                        than a headwall across an open street.
 * @param cl              Trimmed centerline of that edge; must be valid
 * @param profile         Cross-section of that edge; must be valid
 * @param station_heights World Y of the carriageway surface per station, the same
 *                        vector handed to CorridorConfig::station_heights. Must
 *                        have exactly `cl.stations.size()` entries; empty or
 *                        mis-sized returns an empty Mesh.
 * @param terrain_at      Terrain height under a local 2D position. Same convention
 *                        as HeightSampler. When empty, NOTHING is emitted: without
 *                        the terrain there is no way to tell an opening in a
 *                        hillside from one in open air, and guessing puts a
 *                        headwall across a road at random.
 * @param cfg             Portal dimensions and switches
 * @param footprints      Optional sink for the terrain hand-off. When non-null it
 *                        is CLEARED and then filled with one entry per emitted
 *                        portal, in the same order the geometry was built: start
 *                        end first. It stays empty whenever the Mesh is empty, so
 *                        a caller never carves a mouth for a portal that was not
 *                        built. See TunnelPortalFootprint.
 * @return Portal geometry in world space, Y up, with
 *         `Mesh::sort_submeshes_by_material()` applied. Empty when the edge is not
 *         a tunnel, TunnelConfig::emit_portals is false, an input is invalid, or
 *         neither end has ground above it.
 */
[[nodiscard]] Mesh build_tunnel_portals(const GraphEdge& edge,
                                        const Centerline& cl,
                                        const RoadProfile& profile,
                                        const std::vector<float>& station_heights,
                                        const std::function<float(double, double)>& terrain_at,
                                        const TunnelConfig& cfg,
                                        std::vector<TunnelPortalFootprint>* footprints = nullptr);

} // namespace stratum::osm::road
