/**
 * @file crossings.hpp
 * @brief Pedestrian crossings: zebra stripes and the dropped kerbs they need
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * A crossing is two things that must agree with each other: painted stripes
 * across the carriageway, and a break in the kerb line at each end so a wheel
 * can get onto the footway. Emitting the paint without dropping the kerb leaves
 * a zebra running into a 150 mm wall, which is the visible failure this file
 * exists to avoid.
 *
 * The two halves are produced separately because they belong to different
 * meshes. build_crossing() returns paint, entirely MaterialId::Markings, on the
 * same contract as markings.hpp. dropped_kerb_spans() returns no geometry at
 * all: it returns the ANGULAR SPANS of the junction's kerb ring that must drop,
 * which the curb-ring stage consumes. See DroppedKerbSpan.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "osm/road/marking_atlas.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tunables for pedestrian crossings. All distances are metres.
 */
struct CrossingConfig {
    /// Lateral width of one painted stripe, across the road
    float stripe_width = 0.5f;

    /// Unpainted gap between two stripes
    float stripe_gap = 0.5f;

    /**
     * @brief Extent of the crossing along the direction of vehicle travel
     *
     * A zebra stripe runs ALONG the road and the stripes are laid out ACROSS it,
     * so this is the LENGTH of each stripe and MarkingSprite::ZebraStripe is
     * stretched to it. It is the depth of the corridor a pedestrian walks over,
     * not the width of the road being crossed; that comes from Crossing::width.
     */
    float crossing_depth = 3.0f;

    /**
     * @brief Setback of the crossing from the junction polygon edge
     *
     * Applies only to a crossing with Crossing::at_junction set. Measured back
     * along the arm from the arm's trim station, the same reference
     * MarkingConfig::stop_line_setback uses, so a crossing and a stop line on one
     * arm stack in a predictable order rather than overlapping.
     */
    float setback = 1.5f;

    /// Width of the kerb drop at each end of a crossing, along the kerb line
    float dropped_kerb_width = 2.0f;

    /**
     * @brief Height the kerb top drops TO, above the carriageway surface
     *
     * Not zero. A real dropped kerb keeps a residual lip so the footway still
     * drains and a white cane still finds the edge. Emitting zero here makes the
     * footway and the carriageway coplanar at the drop, which z-fights.
     */
    float dropped_kerb_height = 0.02f;

    /**
     * @brief Length the kerb ramps over, at each end of a drop
     *
     * A drop that goes from full curb to the lip between one kerb stone and the
     * next is a vertical step in the ring's curb face, and a step whose two
     * sides are a vertex column apart shows as a tear. The kerb therefore ramps
     * DOWN to the lip over this distance measured along the kerb line, holds the
     * lip across DroppedKerbSpan's own span, and ramps back up over the same
     * distance on the far side.
     *
     * Consumed by the curb-ring stage, not by anything in this file: see
     * KerbDrops in junction_curb.hpp.
     */
    float dropped_kerb_ramp = 1.0f;

    bool emit_zebra = true;         ///< Paint the stripes
    bool emit_dropped_kerbs = true; ///< Produce spans from dropped_kerb_spans()
};

// ============================================================================
// Located crossings
// ============================================================================

/**
 * @brief A pedestrian crossing located on the network
 *
 * A crossing is always attached to exactly one road EDGE, at one arclength along
 * it, whatever OSM shape it was found from. That is what lets the stripes be
 * placed with the same centerline frame the carriageway was extruded with,
 * rather than from the crossing way's own coarse geometry.
 */
struct Crossing {
    /**
     * @brief Graph node this crossing is associated with, or kInvalidId
     *
     * Two things at once, because the crossing needs both and they are never in
     * conflict:
     *
     * - When the crossing came from a `highway=crossing` node that is itself a
     *   graph node, this is that node.
     * - When @ref at_junction is set, this is the JUNCTION node the crossing's
     *   arm approaches -- `edge`'s `from` or `to`, whichever end the crossing is
     *   near. This is the key dropped_kerb_spans() filters on, which is why a
     *   mid-arm junction crossing carries a node it was not found at.
     *
     * A mid-block crossing that is neither -- the common case, since a crossing
     * node is usually interior to the carriageway way and nowhere near a
     * junction -- leaves this kInvalidId and is located by `edge` and
     * `arclength` alone.
     */
    GraphNodeId node = kInvalidId;

    /// Edge the crossing sits on. Never kInvalidId for a returned Crossing.
    EdgeId edge = kInvalidId;

    /// Where along @ref edge it sits, in the edge's own untrimmed parameterisation
    double arclength = 0.0;

    /// Centre of the crossing in 2D local metres, on the carriageway centreline
    glm::dvec2 position{0.0};

    /**
     * @brief Unit direction ACROSS the road, at @ref arclength
     *
     * The station's left normal: `Station::normal` at the crossing, pointing LEFT
     * of the edge's direction of travel. Stripes are laid out along this axis and
     * each stripe is extruded along the perpendicular, which is the direction of
     * vehicle travel.
     */
    glm::dvec2 axis{1.0, 0.0};

    /**
     * @brief Lateral extent being crossed, in metres, centred on the centreline
     *
     * The carriageway envelope: RoadProfile's inclusive Lane-to-Median span, not
     * total_width(). The paint stops at the kerb, it does not run over the
     * footway. On an undivided profile that span is exactly
     * RoadProfile::carriageway_width(); on a divided one it is wider, because the
     * median and the curb faces beside it lie BETWEEN the two carriageways and
     * the crossing has to reach across them to the far kerb.
     *
     * RoadProfile::left_edge_offset() centres that same span on the centreline,
     * so the extent runs from `-width / 2` to `+width / 2` in Crossing::axis.
     */
    float width = 0.0f;

    /**
     * @brief Lateral bounds of a raised island inside @ref width, metres
     *
     * A raised median is not carriageway: paint laid over it would be buried
     * under the island top and inside its curb faces. The interval
     * [@ref island_right, @ref island_left] is therefore left unpainted, and
     * build_crossing() fits a stripe run into each of the two carriageways either
     * side of it independently.
     *
     * Positive is LEFT of travel, matching Crossing::axis and Strip laterals, so
     * `island_left >= island_right`. Equal values -- the default -- mean no
     * island and one contiguous run across the whole extent. A FLUSH median is
     * not an island: it is painted straight across, which is what a flush median
     * is for.
     */
    float island_left = 0.0f;
    float island_right = 0.0f;    ///< See @ref island_left

    /// World Y of the carriageway surface at the crossing, offset included
    float height = 0.0f;

    /**
     * @brief The crossing sits at a junction rather than mid-block
     *
     * True when @ref node is a graph node of degree 3 or more, or when the
     * crossing falls inside the trim of an end of the edge that is ITSELF such a
     * node. A junction crossing is pushed back to CrossingConfig::setback from
     * the trim station; a mid-block one stays where OSM put it.
     *
     * The end-node test is not redundant with the trim test. A degree-2 node
     * whose two profiles differ is given a TAPER, and that writes trims of tens
     * of metres through the same GraphEdge fields; without the test an ordinary
     * lane drop would drag a mid-block crossing down the road to a node that has
     * no junction plane and no curb ring.
     */
    bool at_junction = false;
};

/**
 * @brief Find every crossing on the network
 *
 * Two OSM shapes are recognised, and both resolve to the same Crossing:
 *
 * 1. **`highway=crossing` nodes.** Read from ParsedOSMData::nodes. A crossing
 *    node is located by finding the edge whose node_ids contain it and taking the
 *    arclength of that vertex. This is the reliable shape, because the node is
 *    shared with the carriageway by identity rather than by proximity.
 * 2. **`highway=footway` + `footway=crossing` ways.** Read from
 *    ParsedOSMData::ways. Such a way crosses the carriageway rather than running
 *    along it, so it is resolved by finding a node it shares with a carriageway
 *    edge. A crossing way sharing no node with any carriageway is DISCARDED
 *    rather than snapped by distance: proximity snapping is what the whole graph
 *    was built to avoid, and a wrongly snapped crossing paints a zebra across the
 *    wrong road.
 *
 * The same physical crossing is frequently mapped both ways at once, as a node
 * on the carriageway AND as a crossing way through it. Results are deduplicated
 * by (edge, arclength) within one stripe width, keeping the node-derived one,
 * so a doubly-mapped crossing produces one Crossing and not two overlapping
 * zebras.
 *
 * Crossings on an edge with no Lane strip, on a Footway, Cycleway or Path edge,
 * or on an edge whose centerline is invalid, are skipped.
 *
 * @param graph       Built road graph
 * @param data        Parsed OSM data, for node and way tags
 * @param centerlines Parallel to graph.edges(); the untrimmed centerlines
 * @param profiles    Parallel to graph.edges()
 * @param elevation   Solved elevations, for Crossing::height. An unsolved solver
 *                    leaves every height at 0. That field is a starting point,
 *                    not the paint plane; see build_crossing() below.
 * @param cfg         Tunables. Only CrossingConfig::setback, which pushes a
 *                    junction crossing back off its arm's trim station, and
 *                    CrossingConfig::stripe_width, which is the deduplication
 *                    window, are read here; the rest are consumed by
 *                    build_crossing() and dropped_kerb_spans(). Defaulted so a
 *                    caller that only wants the shipping values may omit it.
 * @return Crossings in ascending (edge, arclength) order, so a build is
 *         reproducible run to run
 */
[[nodiscard]] std::vector<Crossing> find_crossings(const RoadGraph& graph,
                                                   const ParsedOSMData& data,
                                                   const std::vector<Centerline>& centerlines,
                                                   const std::vector<RoadProfile>& profiles,
                                                   const RoadElevationSolver& elevation,
                                                   const CrossingConfig& cfg = {});

/**
 * @brief Zebra stripes for one crossing
 *
 * Stripes are laid out along Crossing::axis on a
 * `stripe_width` / `stripe_gap` cycle, centred on Crossing::position, covering
 * Crossing::width. Each stripe is one quad, `stripe_width` across and
 * CrossingConfig::crossing_depth along the direction of vehicle travel, mapping
 * the full MarkingSprite::ZebraStripe rect -- one quad per repeat, because an
 * atlas rect cannot wrap. See marking_atlas.hpp.
 *
 * A partial stripe at either end is dropped rather than clipped, so the pattern
 * stays symmetric about the centreline and no half-width stripe appears against
 * the kerb.
 *
 * A raised island -- Crossing::island_left above Crossing::island_right -- splits
 * the extent into two carriageways, and a run is fitted into each of them
 * separately. Each run is centred in its own carriageway, so the partial-stripe
 * rule holds against all four kerb lines rather than against the outer two.
 *
 * Geometry is world space, Y up, entirely MaterialId::Markings, with
 * `Mesh::sort_submeshes_by_material()` already applied -- the same output
 * contract as markings.hpp and Corridor::mesh.
 *
 * ### The paint plane
 *
 * Quads are emitted at EXACTLY Crossing::height, with no offset of this
 * function's own. find_crossings() records that field from the vertical solve
 * ALONE, which is not the plane the corridor ends up on: it is zero when the
 * network is flat, it predates the junction plateaus, and it carries no paint
 * lift. The caller therefore sets it, from the same per-station heights the
 * corridor and the lane markings were placed at:
 *
 * @code
 *     Crossing lifted = c;
 *     lifted.height = surface_at(cl, paint_heights, c.arclength) +
 *                     marking_cfg.height_above_surface;
 *     Mesh zebra = build_crossing(lifted, crossing_cfg);
 * @endcode
 *
 * There is deliberately no second height constant in CrossingConfig. One paint
 * plane offset exists in this pipeline, it lives in
 * MarkingConfig::height_above_surface, and a zebra emitted on a different plane
 * from the stop line beside it z-fights against the road at a different camera
 * distance, which is the worst kind of this bug to find.
 *
 * This function emits PAINT ONLY. The kerb drop is not geometry here; see
 * dropped_kerb_spans().
 *
 * @param c   A located crossing
 * @param cfg Stripe dimensions and switches
 * @return Stripe geometry; empty when CrossingConfig::emit_zebra is false or the
 *         crossing is too narrow for one full stripe
 */
[[nodiscard]] Mesh build_crossing(const Crossing& c, const CrossingConfig& cfg);

// ============================================================================
// Dropped kerbs
// ============================================================================

/**
 * @brief A run of a junction's kerb ring that must drop to a lip
 *
 * Expressed in the JUNCTION'S LOCAL FRAME as a pair of directions from the
 * junction centre, not as positions and not as radians. Directions survive the
 * ring being re-offset or re-tessellated, which positions do not: CurbRing::outer
 * is produced by a Clipper2 inflate whose vertex count does not correspond to
 * CurbRing::inner's, so a span pinned to a vertex index or to a point on one ring
 * does not exist on the other.
 *
 * The span runs COUNTER-CLOCKWISE from @ref from to @ref to, matching the
 * counter-clockwise winding of both CurbRing::inner and CurbRing::outer. A
 * consumer clips the ring to the span by testing each ring vertex's direction
 * from the centre against that counter-clockwise interval.
 *
 * Both vectors are unit length. A span whose two directions are equal is empty
 * and must be ignored, never treated as the whole ring.
 */
struct DroppedKerbSpan {
    /// Unit direction from the junction centre at the counter-clockwise START
    glm::dvec2 from{0.0};

    /// Unit direction from the junction centre at the counter-clockwise END
    glm::dvec2 to{0.0};

    /**
     * @brief Height the kerb top drops to, above the carriageway surface
     *
     * CrossingConfig::dropped_kerb_height. Carried per span rather than read from
     * the config again, so a future rule that varies the lip -- a flush crossing,
     * a driveway -- needs no new field.
     */
    float height = 0.0f;
};

/**
 * @brief Angular spans of one junction's kerb ring that a crossing requires be dropped
 *
 * A crossing needs a drop at BOTH of its ends, one on each side of the
 * carriageway, so a crossing at a junction contributes up to two spans. Each is
 * CrossingConfig::dropped_kerb_width long along the kerb, converted to an
 * angular span about the junction centre.
 *
 * ### Where the span is laid, and why it is not on the crossing
 *
 * A junction crossing stands CrossingConfig::setback metres OUTSIDE its arm's
 * cut line. Measured from the junction centre, its kerb points fall inside that
 * arm's own MOUTH -- the sector between the arm's two carriage corners -- and
 * the ring carries no geometry there: build_curb_ring() runs from one arm's
 * corner round the fillet to the next arm's corner and leaves every mouth open.
 * A span laid on the crossing itself would ask for a drop in a gap.
 *
 * So each span is walked back along the arm by that same setback, which lands on
 * the arm's carriage corner, and then runs from the corner INTO the fillet. The
 * fillet is tangent to the arm's kerb line at the corner, so that is the kerb a
 * pedestrian at this crossing steps off, and it is live ring by construction.
 * The corridor kerb running the other way past the corner is not part of any
 * ring and cannot be cut from here at all.
 *
 * Only crossings whose Crossing::at_junction is set and whose arm belongs to
 * @p node contribute. A mid-block crossing needs a drop in the CORRIDOR's kerb
 * strip, not in a junction ring, and is not represented here.
 *
 * Overlapping spans are merged, so two crossings on adjacent arms sharing a
 * corner produce one continuous drop rather than two that fight over the same
 * kerb stones.
 *
 * ### Who consumes this
 *
 * This is a data product, not geometry. The road network builder hands the spans
 * to the curb-ring stage, which already knows how to leave the ring open at an
 * arm mouth and drops the ring's curb face and top to DroppedKerbSpan::height
 * across each span instead of omitting it. Where that integration is not wired
 * up, the spans are still returned and still counted, and the ring is emitted
 * unchanged -- the crossing then reads as paint against a full-height kerb,
 * which is wrong but not broken.
 *
 * @param crossings       All located crossings, from find_crossings()
 * @param node            Junction node whose ring is being cut. A crossing takes
 *                        part when Crossing::at_junction is set and
 *                        Crossing::node equals this.
 * @param junction_center Junction centre in 2D local metres: Junction::center.
 *                        Every returned direction is measured from this point,
 *                        so passing the wrong centre rotates every span.
 * @param cfg             Drop width and lip height
 * @return Merged spans in counter-clockwise order; empty when
 *         CrossingConfig::emit_dropped_kerbs is false or no crossing reaches @p node
 */
[[nodiscard]] std::vector<DroppedKerbSpan> dropped_kerb_spans(const std::vector<Crossing>& crossings,
                                                              GraphNodeId node,
                                                              glm::dvec2 junction_center,
                                                              const CrossingConfig& cfg);

/**
 * @brief Angular spans of one junction's kerb ring that a DRIVEWAY requires be dropped
 *
 * The other real-world dropped kerb, and the one the plan names beside the
 * crossing: "Driveways (`service=driveway`), parking aisles
 * (`service=parking_aisle`), and alleys get narrow profiles with no curb, and
 * drop the kerb where they meet the parent road."
 *
 * A driveway arm already opens a MOUTH in the ring, since every arm does, so
 * what is missing without this is the flare either side of that mouth: the ring
 * would otherwise run at full height right up to the mouth and stop dead, and a
 * car would have to climb a 150 mm face to reach the gap. Each driveway arm
 * therefore contributes one span centred on its mouth, as wide as the arm's own
 * carriageway plus CrossingConfig::dropped_kerb_width of flare on each side.
 *
 * A node whose every arm is a driveway gets nothing: with no parent road there
 * is no kerb line for a driveway to interrupt. A node of degree below 3 gets
 * nothing either, because no ring is built there.
 *
 * The result is on exactly the same contract as dropped_kerb_spans(), and the
 * two lists may simply be concatenated before being handed to the curb-ring
 * stage: overlapping spans there take the deepest drop, so they need not be
 * merged across the two calls.
 *
 * @param graph           Built road graph
 * @param data            Parsed OSM data. GraphEdge carries no `service=*`
 *                        field, so the driveway subtype is read from the parent
 *                        way's raw tags, keyed by GraphEdge::source_way.
 * @param node            Junction node whose ring is being cut
 * @param junction_center Junction centre in 2D local metres: Junction::center
 * @param profiles        Parallel to graph.edges(); supplies each arm's
 *                        carriageway width
 * @param cfg             Flare width and lip height
 * @return Merged spans in counter-clockwise order; empty when
 *         CrossingConfig::emit_dropped_kerbs is false or no arm of @p node is a
 *         driveway
 */
[[nodiscard]] std::vector<DroppedKerbSpan> driveway_kerb_spans(const RoadGraph& graph,
                                                               const ParsedOSMData& data,
                                                               GraphNodeId node,
                                                               glm::dvec2 junction_center,
                                                               const std::vector<RoadProfile>& profiles,
                                                               const CrossingConfig& cfg);

} // namespace stratum::osm::road
