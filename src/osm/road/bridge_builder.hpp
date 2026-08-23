/**
 * @file bridge_builder.hpp
 * @brief Deck, parapets and piers for a bridge edge
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The elevation solver already lifts a `bridge=*` edge to a deck height above
 * the terrain it spans, and the corridor extruder already sweeps its profile
 * along at that height. What is missing is everything that makes the result read
 * as a bridge rather than as a road floating in the air: the deck has no
 * thickness, no edge, no underside, nothing along its sides and nothing holding
 * it up.
 *
 * ### What this adds, and what it must not
 *
 * This builder adds ONLY the structure. It does NOT re-emit the running surface:
 * the corridor already owns the top of the deck, and a second copy of it a
 * millimetre away z-fights across the whole span. The three additions are:
 *
 * - **Deck slab** -- the underside, offset BridgeConfig::deck_thickness below the
 *   profile's surface, plus the vertical fascia around its perimeter closing the
 *   gap to the corridor's outer edge. MaterialId::BridgeDeck.
 * - **Parapets** -- a solid wall along both outer edges of the profile, running
 *   the length of the span. MaterialId::Parapet.
 * - **Piers** -- rectangular stubs dropped from the underside to the terrain at
 *   BridgeConfig::pier_spacing along the span. MaterialId::Concrete.
 *
 * BridgeDeck and Parapet exist in MaterialId for exactly this and are used
 * nowhere else in the pipeline.
 *
 * ### Why the terrain arrives as a callback
 *
 * Nothing under `src/osm/road/` may include anything under `src/procgen/`, and
 * piers must know where the ground is. The sampler is the same contract as
 * road_elevation.hpp's HeightSampler -- local 2D metres in, world Y out -- and is
 * usually the very same object.
 *
 * ### Bridges do not carve
 *
 * `CarveRibbon::suppress` is already set for bridge spans, so a deck never cuts
 * the ground out from under its own piers. That is the behaviour this builder
 * assumes. If bridges were ever made to carve, every pier would end up standing
 * on the flattened trench its own deck cut, at deck height less the deck
 * thickness, and would be invisible.
 *
 * The suppression covers the bridge's OWN ribbon and nothing else. The road the
 * bridge SPANS carves normally, and it carves exactly where the piers stand, so
 * the height sampler's natural surface is not the ground a pier ends up on. See
 * BridgeConfig::pier_foundation_depth, which is why the feet are carried below
 * it rather than set on it.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "renderer/mesh.hpp"

#include <functional>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Dimensions of bridge structure. All distances are metres.
 */
struct BridgeConfig {
    /**
     * @brief Depth of the deck slab, from the running surface down to the underside
     *
     * The whole structural depth, not just the wearing course, so it reads at the
     * distance a bridge is usually seen from. A deck thinner than about 0.3 m
     * disappears edge-on.
     */
    float deck_thickness = 0.8f;

    /// Height of the parapet above the running surface at the profile's outer edge
    float parapet_height = 1.1f;

    /// Thickness of the parapet wall, inward from the profile's outer edge
    float parapet_width = 0.3f;

    /**
     * @brief Largest distance along the span between pier centres
     *
     * An UPPER bound, not a stride. The span is divided into
     * `ceil(span / pier_spacing)` equal parts and a pier goes at each interior
     * division, so the pattern is symmetric, the achieved spacing is at or below
     * this value, and no pier can land next to an abutment. Laying piers out at a
     * fixed stride from one end instead puts a stub 30 cm from the abutment as
     * soon as the span is not a whole multiple of the spacing.
     *
     * A span no longer than one spacing therefore has no interior division and
     * gets NO piers: a short bridge is a clear span, and a single column under a
     * footbridge over a ditch is worse than none.
     */
    float pier_spacing = 25.0f;

    /// Side of the square pier stub, in plan
    float pier_width = 1.5f;

    /**
     * @brief Shortest pier worth emitting
     *
     * A pier is skipped where the deck underside is less than this above the
     * terrain. Near an abutment the deck meets the ground, and a pier there is a
     * few centimetres of geometry buried in the embankment that costs draw calls
     * and shows as z-fighting where it pokes through.
     *
     * The terrain it is measured against is the LOWEST of five samples -- the
     * pier's centre and its four plan corners -- which is also what the pier
     * stands on, so a pier on a slope is buried at its high corner rather than
     * hovering at its low one.
     */
    float min_pier_height = 1.0f;

    /**
     * @brief How far below the sampled ground a pier's foot is carried, metres
     *
     * The height sampler handed to build_bridge() reads the NATURAL surface. The
     * ground a pier actually stands on is whatever the terrain carve leaves, and
     * a bridge spans exactly the places other roads have carved: only the
     * bridge's OWN ribbon sets CarveRibbon::suppress, never the ribbon of the
     * road passing underneath. A motorway in a cutting under an overpass lowers
     * the finished ground by metres, and a foot placed at the natural surface
     * then hangs in mid-air over the one thing a bridge is always viewed from
     * below.
     *
     * A foundation is the cheap, order-independent answer: carry the foot down
     * far enough that a plausible cut still leaves it buried. It costs four
     * quads per pier, all of them underground, and needs no second pass over the
     * carve. It is not unbounded -- a cut deeper than this still exposes the foot
     * -- so it is a config value and not a constant.
     */
    float pier_foundation_depth = 2.5f;

    bool emit_parapets = true;  ///< Emit the side walls
    bool emit_piers = true;     ///< Emit the supports
};

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Deck slab, parapets, and piers for one bridge edge
 *
 * ### Lateral placement
 *
 * Every lateral is taken from @p profile and converted through offset_point(),
 * exactly as build_corridor() does, so the structure sits on the same mitred
 * ribbon the surface does. The deck's outer boundary is
 * `RoadProfile::left_edge_offset()` on the left and that value less
 * `RoadProfile::total_width()` on the right. Parapets stand INBOARD of those
 * boundaries by BridgeConfig::parapet_width, so a parapet never overhangs the
 * slab it stands on.
 *
 * ### Vertical placement
 *
 * @p station_heights is the same vector handed to CorridorConfig::station_heights
 * for this edge: the world Y of the running surface per station, offset included.
 * It must have exactly `cl.stations.size()` entries; an empty or mis-sized vector
 * returns an empty Mesh, because a bridge is entirely defined by its height and
 * guessing one produces a structure detached from its own road.
 *
 * The underside is `station_heights[i] - deck_thickness`, flat across the
 * section: the reference is the CARRIAGEWAY surface, so a raised sidewalk
 * thickens the slab under itself rather than stepping the underside. It is
 * subdivided laterally at the profile's own strip boundaries so that the end
 * caps, which must follow the stepped cross-section, meet real underside edges
 * instead of T-junctions.
 *
 * Parapets run from the lowest profile height under their own footprint -- so
 * the wall's foot is buried in the deck rather than hovering over a gutter --
 * up to BridgeConfig::parapet_height above the profile's OUTER edge. They carry
 * no bottom face: it would be coincident with the corridor's running surface and
 * z-fight with it, and the wall is closed against that surface anyway.
 * BridgeConfig::parapet_width is clamped to two fifths of the deck width so the
 * two walls of a narrow footbridge cannot meet in the middle.
 *
 * A pier runs from the underside down to the terrain under its own footprint and
 * is skipped when that drop is under BridgeConfig::min_pier_height. Both ends
 * are pushed a centimetre into the solid they meet, since the pier top is flat
 * while the underside is ruled along the grade.
 *
 * ### Hairpins
 *
 * Where the centerline turns tighter than the profile's half width, offset_point()
 * clamps to Station::lateral_min / lateral_max and the profile collapses onto that
 * bound. The structure inherits that collapse from the surface: at such a station
 * the two faces of a parapet land on the same point and the wall is locally zero
 * thickness. It is left that way deliberately. Skipping the collapsed band instead
 * would put a HOLE in the parapet, and a wall with no thickness at one station
 * reads correctly from every angle while a gap in it does not. The deck shell
 * itself stays a consistently oriented surface through a hairpin.
 *
 * ### Bad data
 *
 * A deck below the terrain at EVERY station is rejected: the edge is logged and
 * an empty Mesh returned, rather than burying a structure in a hill. Only the
 * every-station case is rejected, because an abutment legitimately meets the
 * embankment it springs from.
 *
 * ### World mapping and winding
 *
 * The mapping is the pipeline's, Y up: `(x, y_2d) -> vec3(x, height, -y_2d)`.
 * That flips handedness, so a ring wound counter-clockwise in local 2D is
 * clockwise in world XZ. Winding must be checked against the renderer's
 * counter-clockwise front face for each surface rather than assumed from the 2D
 * ring, and every surface here is closed: the underside faces DOWN, the fascia
 * faces OUTWARD, the parapet's four faces face outward and up, and a pier's four
 * sides face outward. A bridge is the one piece of road geometry routinely seen
 * from below, so a flipped face is visible rather than merely wrong.
 *
 * ### Which centerline to pass
 *
 * Pass the TRIMMED centerline, the same
 * `slice(cl, edge.trim_from, cl.length() - edge.trim_to)` the corridor was
 * extruded from. The structure must end where the surface ends; a deck built
 * from the untrimmed centerline runs on into the junction the ribbon was cut back
 * from.
 *
 * @param edge            Bridge edge. Nothing is emitted when GraphEdge::is_bridge
 *                        is false -- the caller's filter is checked here too, so a
 *                        misrouted edge produces nothing rather than a parapet
 *                        along an ordinary street.
 * @param cl              Trimmed centerline of that edge; must be valid
 * @param profile         Cross-section of that edge; must be valid
 * @param station_heights World Y of the running surface per station
 * @param terrain_at      Terrain height under a local 2D position, for the piers.
 *                        Same convention as HeightSampler. When empty, piers are
 *                        skipped and the deck and parapets are still emitted.
 * @param cfg             Structure dimensions and switches
 * @return Structure geometry in world space, Y up, with
 *         `Mesh::sort_submeshes_by_material()` applied. Empty when the edge is not
 *         a bridge or either input is invalid.
 */
[[nodiscard]] Mesh build_bridge(const GraphEdge& edge,
                                const Centerline& cl,
                                const RoadProfile& profile,
                                const std::vector<float>& station_heights,
                                const std::function<float(double, double)>& terrain_at,
                                const BridgeConfig& cfg);

} // namespace stratum::osm::road
