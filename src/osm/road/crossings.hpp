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
 * same contract as markings.hpp. The kerb drop returns no geometry at all: it
 * returns the RUNS of kerb that must drop, which the two stages that own a kerb
 * line consume.
 *
 * There are two such stages, and a crossing needs BOTH:
 *
 * - dropped_kerb_spans() / driveway_kerb_spans() return ANGULAR SPANS of a
 *   junction's kerb RING, measured from the junction centre. See
 *   DroppedKerbSpan; junction_curb.hpp consumes them.
 * - corridor_kerb_drops() returns ARCLENGTH RUNS of an edge's own corridor kerb.
 *   See CorridorKerbDrop and CorridorKerbProfile; the corridor extruder consumes
 *   them. A mid-block crossing has no ring at all and is expressed ONLY here,
 *   and a junction crossing needs both, because the ring's drop stops at the
 *   arm's mouth and the arm's own kerb runs on from there at full height.
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

#include <cstdint>
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
     * Consumed by whichever stage owns the kerb being cut -- KerbDrops in
     * junction_curb.hpp for a ring, CorridorKerbProfile below for an edge's own
     * kerb -- and by nothing that emits geometry in this file.
     */
    float dropped_kerb_ramp = 1.0f;

    bool emit_zebra = true;         ///< Paint the stripes

    /// Produce runs from dropped_kerb_spans(), driveway_kerb_spans() and
    /// corridor_kerb_drops()
    bool emit_dropped_kerbs = true;
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
     *
     * ### The miter is already in this number
     *
     * A lateral offset in the profile's own frame is NOT a distance on the
     * ground. The corridor extruder puts every strip edge through
     * offset_point(), which clamps the lateral to the station's fold bounds and
     * then multiplies by Station::miter_scale, so at a mitred joint the
     * carriageway edge stands `half * miter_scale` from the centreline and not
     * `half`. This field is the extent MEASURED ON THE GROUND along
     * Crossing::axis: the profile span with that clamp and that scale already
     * applied. Laying stripes out against the raw profile span instead stops the
     * paint short of the kerb at every bend -- 1.45 m short a side at a
     * right-angle joint -- and, inside a fold where the clamp binds, runs it
     * over the kerb.
     *
     * Where the fold clamp binds asymmetrically the SHORTER of the two sides is
     * taken, so the extent stays centred on the centreline as the field's
     * contract requires and the paint can never cross a kerb.
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
     *
     * Measured on the ground along Crossing::axis, on the same terms as
     * @ref width: the miter scale and the fold clamp are already applied.
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
 * ring and cannot be cut from here at all -- corridor_kerb_drops() cuts it, and
 * a junction crossing needs BOTH calls or the two kerbs meet at the arm's mouth
 * with a full curb height between them.
 *
 * Only crossings whose Crossing::at_junction is set and whose arm belongs to
 * @p node -- or to one of @p absorbed_nodes -- contribute. A mid-block crossing
 * needs a drop in the CORRIDOR's kerb strip, not in a junction ring, and is not
 * represented here.
 *
 * ### Why @p absorbed_nodes exists
 *
 * collect_arms() merges near-coincident junction nodes: one primary takes every
 * arm and the other members are solved away. Crossing::node still names the arm's
 * OWN end node, so a crossing on an absorbed member's approach asks for a drop
 * from a node that has no ring, while the primary's ring -- the one the crossing
 * actually stands on -- never sees it. The corridor still lays its own drop right
 * up to the trim station, so the two kerbs then meet at the arm mouth with a full
 * curb height between them, which is exactly the step this pair of calls exists
 * to remove. Pass the nodes the primary absorbed and the spans land on the ring
 * that covers them.
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
 * @param absorbed_nodes  Optional. Other graph nodes this junction's ring covers,
 *                        because collect_arms() merged them into @p node. A
 *                        crossing naming any of them is treated exactly as one
 *                        naming @p node. Null or empty is the ordinary case.
 * @return Merged spans in counter-clockwise order; empty when
 *         CrossingConfig::emit_dropped_kerbs is false or no crossing reaches @p node
 */
[[nodiscard]] std::vector<DroppedKerbSpan> dropped_kerb_spans(
    const std::vector<Crossing>& crossings,
    GraphNodeId node,
    glm::dvec2 junction_center,
    const CrossingConfig& cfg,
    const std::vector<GraphNodeId>* absorbed_nodes = nullptr);

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


// ============================================================================
// Corridor kerb drops
// ============================================================================

/**
 * @brief Which of an edge's two kerb lines a drop applies to
 *
 * Sides are named against the edge's DIRECTION OF TRAVEL, the same convention
 * Strip laterals and Crossing::axis use: Left is positive lateral.
 */
enum class KerbSide : std::uint8_t {
    Left,   ///< The kerb at positive lateral
    Right,  ///< The kerb at negative lateral
    Both    ///< Both kerbs of the edge
};

/**
 * @brief A run of ONE edge's own corridor kerb that must drop to a lip
 *
 * The corridor counterpart of DroppedKerbSpan, and the half of the kerb drop
 * that a junction ring cannot express. A ring is a closed curve around a node
 * and its spans are angles about that node's centre; an edge's kerb is a ribbon
 * along a centerline and its runs are ARCLENGTHS in that centerline's own
 * untrimmed parameterisation -- the same frame GraphEdge::trim_from,
 * Crossing::arclength and the lane markings are already expressed in.
 *
 * @ref from and @ref to bound the FLAT part of the drop. The kerb ramps between
 * full height and the lip over @ref ramp_from before @ref from and over
 * @ref ramp_to after @ref to, so the whole run the drop touches is
 * `[from - ramp_from, to + ramp_to]`.
 *
 * A ramp of zero is not a step. It means the drop CONTINUES past that end into
 * something else that is already at the lip, which is exactly what happens at a
 * junction crossing: the run is laid right up to the arm's trim station, where
 * the corridor stops and the junction ring's own dropped span takes over. Both
 * ends of a mid-block drop carry a real ramp.
 */
struct CorridorKerbDrop {
    /// Edge whose kerb drops. Never kInvalidId for a returned drop.
    EdgeId edge = kInvalidId;

    /// Arclength the flat lip starts at, in the edge's untrimmed parameterisation
    double from = 0.0;

    /// Arclength the flat lip ends at; always >= @ref from
    double to = 0.0;

    /// Metres the kerb ramps over BEFORE @ref from; 0 means the drop continues
    double ramp_from = 0.0;

    /// Metres the kerb ramps over AFTER @ref to; 0 means the drop continues
    double ramp_to = 0.0;

    /// Height the kerb top drops to, above the carriageway surface
    float height = 0.0f;

    /// Which kerb line drops
    KerbSide side = KerbSide::Both;
};

/**
 * @brief Runs of corridor kerb that the located crossings require be dropped
 *
 * This is the answer to the failure crossings.hpp has always named and never
 * fixed: a mid-block crossing on a kerbed street painted a zebra into a 150 mm
 * wall, because the only kerb the pipeline knew how to drop belonged to a
 * junction ring and a mid-block crossing has no ring.
 *
 * Every crossing contributes, not only the mid-block ones:
 *
 * - A **mid-block** crossing contributes one run centred on its arclength,
 *   `max(CrossingConfig::dropped_kerb_width, CrossingConfig::crossing_depth)`
 *   long -- the drop is at least as wide as the painted corridor it serves --
 *   with a real ramp at each end.
 * - A **junction** crossing contributes one run from its own arclength out to
 *   the arm's TRIM STATION, with a ramp only on the mid-block side. That end is
 *   where the corridor stops and dropped_kerb_spans() begins, so the two drops
 *   butt at the same height instead of stepping the full curb height at the arm
 *   mouth. Emitting the ring span without this run is the tear that made a
 *   junction crossing look worse than no crossing at all.
 *
 * Runs are clamped into `[trim_from, length - trim_to]`, which is the span the
 * corridor actually occupies, so a drop can never be demanded of kerb that was
 * trimmed away. Overlapping runs on one edge and side are merged, taking the
 * deeper lip and the outer ends' ramps, so two crossings a few metres apart
 * produce one continuous drop rather than a ripple between them.
 *
 * Edges whose profile carries no kerb at all -- a rural road with a verge, a
 * motorway with a shoulder -- contribute nothing, so a build over an extract
 * with no footways pays nothing for this pass.
 *
 * @param crossings   All located crossings, from find_crossings()
 * @param graph       Built road graph; supplies each edge's solved trims
 * @param centerlines Parallel to graph.edges(); the untrimmed centerlines the
 *                    arclengths are measured in
 * @param profiles    Parallel to graph.edges(); an edge with no kerb is skipped
 * @param cfg         Drop width, lip height and ramp length
 * @return Runs sorted by (edge, from), so a build is reproducible run to run;
 *         empty when CrossingConfig::emit_dropped_kerbs is false
 */
[[nodiscard]] std::vector<CorridorKerbDrop> corridor_kerb_drops(
    const std::vector<Crossing>& crossings,
    const RoadGraph& graph,
    const std::vector<Centerline>& centerlines,
    const std::vector<RoadProfile>& profiles,
    const CrossingConfig& cfg);

/**
 * @brief One edge's kerb drops, evaluated as a continuous function of arclength
 *
 * The corridor counterpart of the DropProfile that junction_curb.cpp builds from
 * KerbDrops, and it exists for the same reason: the modulation must be
 * CONTINUOUS in position, because the extruder samples it at vertex columns and
 * anything the sampling misses reads as a step or as nothing at all.
 *
 * ### What a consumer must do with it
 *
 * A drop is a MODULATION of the profile the edge already has, never a second
 * profile. Exactly as KerbDrops describes for the ring, and in the same order
 * from the carriageway outward:
 *
 * - Strip boundaries at the carriageway surface do not move. The gutter is at
 *   road level with or without a drop.
 * - The CurbFace's RAISED edge and both edges of the CurbTop beyond it take
 *   top_height(). The face keeps its batter in proportion to its remaining
 *   height, so a 20 mm lip leans 20 mm's worth and not 150 mm's worth.
 * - The first strip OUTBOARD of that curb -- the sidewalk -- takes top_height()
 *   at its inboard edge and keeps its outboard edge at full height. It becomes
 *   the crossfall ramp, which is what a real dropped kerb does to a footway, and
 *   it is why the drop can never tear the ribbon away from the verge, the
 *   terrain or the junction ring beside it.
 * - Everything further outboard is untouched.
 *
 * Every boundary that moves must take ONE evaluation of top_height() per
 * station and side. Two evaluations of the same station cannot be relied on to
 * agree once the caller has interpolated anything, and boundaries that disagree
 * are exactly where a ribbon splits.
 *
 * ### Resampling
 *
 * required_stations() lists the arclengths the centerline MUST carry a station
 * at for the ramp to be drawn as a slope. A centerline resampled at
 * ResampleConfig::max_spacing puts its stations metres apart, and a 1 m ramp
 * laid out against those columns falls between two of them and is drawn as a
 * vertical step -- or, where a whole drop fits inside one band, is drawn not at
 * all. A consumer that cannot resample gets a step, which is the tear this class
 * exists to avoid.
 */
class CorridorKerbProfile {
public:
    /// An inactive profile: top_height() returns the full height everywhere
    CorridorKerbProfile() = default;

    /**
     * @brief Select one edge's runs out of the network-wide list
     *
     * @param drops       Runs from corridor_kerb_drops(); other edges are ignored
     * @param edge        Edge to build the profile for
     * @param curb_height Full height of the kerb above the carriageway, metres.
     *                    Every lip is clamped into [0, curb_height], so a
     *                    misconfigured lip can never raise the kerb.
     * @param ramp_floor  Shortest ramp that will be honoured, metres. A ramp
     *                    below this is raised to it, because a ramp of zero
     *                    LENGTH is an instant step; it is not the same thing as
     *                    CorridorKerbDrop's zero ramp, which means the drop
     *                    continues rather than ramping.
     */
    CorridorKerbProfile(const std::vector<CorridorKerbDrop>& drops,
                        EdgeId edge,
                        double curb_height,
                        double ramp_floor = 0.05);

    /// There is at least one usable run on this edge
    [[nodiscard]] bool active() const { return !m_runs.empty(); }

    /**
     * @brief Height the kerb top stands at, above the carriageway surface
     *
     * @param arclength     Station arclength, in the edge's untrimmed frame
     * @param left_of_travel Which kerb line is being asked about
     * @param full          The undropped curb height, metres
     * @return @p full away from every run, the run's lip inside one, and a linear
     *         ramp between the two. Where two runs overlap the deeper wins.
     */
    [[nodiscard]] double top_height(double arclength, bool left_of_travel, double full) const;

    /**
     * @brief Fraction of the way from the full kerb down to the lip, in [0, 1]
     *
     * 1 inside a run, 0 clear of every run and its ramps. Published alongside
     * top_height() because a consumer scaling a batter, a UV or a material blend
     * needs the shape of the ramp rather than its height.
     */
    [[nodiscard]] double factor(double arclength, bool left_of_travel) const;

    /**
     * @brief Arclengths the centerline must carry a station at
     *
     * Every run's four breakpoints, plus intermediate samples down each ramp, so
     * the slope is carried by real vertex columns. Ascending, deduplicated, and
     * clipped to `[lo, hi]`.
     *
     * @param lo Lowest arclength the centerline covers
     * @param hi Highest arclength the centerline covers
     * @return Arclengths in ascending order; empty when the profile is inactive
     */
    [[nodiscard]] std::vector<double> required_stations(double lo, double hi) const;

private:
    struct Run {
        double from = 0.0;
        double to = 0.0;
        double ramp_from = 0.0;
        double ramp_to = 0.0;
        double lip = 0.0;
        bool left = true;
        bool right = true;
    };

    [[nodiscard]] double run_factor(const Run& r, double s) const;

    std::vector<Run> m_runs;
};

} // namespace stratum::osm::road
