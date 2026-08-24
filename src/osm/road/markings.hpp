/**
 * @file markings.hpp
 * @brief Painted lane markings emitted from the P2 strip columns
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Markings are not a new geometry problem. The cross-section already knows where
 * every lane boundary is -- RoadProfile lays out its strips left to right and the
 * boundary between two Lane strips is a lane line by definition -- and the
 * centerline already knows where every station is and how far along the road it
 * sits. This file turns those two facts into quads.
 *
 * ### What comes out
 *
 * Every function here returns a Mesh whose geometry is entirely
 * MaterialId::Markings, in world space, Y up, with
 * `Mesh::sort_submeshes_by_material()` already applied. That is the same output
 * contract as Corridor::mesh, so a caller appends the result into a RoadPiece
 * without special-casing it.
 *
 * Marking quads are SEPARATE geometry sitting MarkingConfig::height_above_surface
 * above the carriageway. They never share vertices with the corridor, they are
 * never welded into it by P7, and they carry explicit atlas sub-rect UVs from
 * marking_atlas.hpp rather than metre-tiled UVs. See the plan's UV Convention.
 *
 * ### Direction of travel
 *
 * Left, right, forward and backward are all relative to the edge's own direction
 * of travel, from GraphEdge::from towards GraphEdge::to, exactly as in
 * road_profile.hpp. They are not compass directions and they do not depend on
 * which side of the road traffic drives. MarkingConfig::left_hand_traffic
 * decides which LANE GROUP is which, not which way the profile is ordered.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "osm/road/marking_atlas.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tunables for painted lane markings. All distances are metres.
 */
struct MarkingConfig {
    /// Painted width of a single line, overriding SpriteSize::width_m
    float line_width = 0.15f;

    /// Painted length of one dash, overriding SpriteSize::length_m
    float dash_length = 3.0f;

    /// Unpainted gap between two dashes
    float dash_gap = 6.0f;

    /**
     * @brief Inset of an edge line from the lane edge, inward
     *
     * The edge line is painted INSIDE the carriageway, so it is offset from the
     * outermost Lane strip's outer boundary towards the road's centre by this
     * distance. A zero inset puts the paint half over the gutter.
     */
    float edge_line_inset = 0.2f;

    /**
     * @brief Height of the marking plane above the carriageway surface
     *
     * Large enough to clear depth precision at map-editor camera distances, small
     * enough that the paint does not read as a raised kerb from a shallow angle.
     * Markings sit just above the carriageway, never on it, because coplanar
     * geometry z-fights.
     */
    float height_above_surface = 0.01f;

    /// Longitudinal thickness of a stop line, along the direction of travel
    float stop_line_width = 0.4f;

    /**
     * @brief Setback of the stop line from the junction polygon edge
     *
     * Measured back along the arm from the arm's TRIM STATION, which is where the
     * junction polygon cuts the ribbon. See build_approach_markings().
     */
    float stop_line_setback = 1.0f;

    /// Distance back from the stop line to the first turn arrow
    float arrow_spacing = 25.0f;

    /**
     * @brief Lateral gap between the two lines of a double centre line
     *
     * Only read under left-hand traffic, where a no-overtaking centre line is
     * TWO separate MarkingSprite::SolidWhite runs rather than one sprite that
     * already contains its own gap. Under right-hand traffic the pair and the gap
     * are painted by MarkingSprite::DoubleSolidYellow as a single quad, and this
     * is unused.
     */
    float double_line_gap = 0.15f;

    /**
     * @brief Arclength between repeats of a lane pictogram
     *
     * Governs MarkingSprite::BikeSymbol on a CycleLane strip and
     * MarkingSprite::BusSymbol on a bus lane. Both are reminders rather than
     * instructions, so they are sparse; a symbol every few metres reads as a
     * texture rather than as a marking.
     */
    float symbol_spacing = 30.0f;

    /**
     * @brief Paint a centre line on residential and service roads
     *
     * Off by default because the overwhelming majority of them carry no centre
     * line at all: a residential street is two-way over a single shared running
     * surface, and painting a divider down it makes an estate read as a
     * distributor road. Roads of class Primary and below down to Tertiary always
     * get one; Motorway and Trunk never do, because their carriageways are
     * separated by a Median strip and a line between them would be a line down
     * the middle of a physical island.
     */
    bool centre_line_on_minor_roads = false;

    /**
     * @brief Paint edge lines on residential, service, and unclassified roads
     *
     * Off by default for the same reason. An edge line marks the boundary of a
     * running surface that has a shoulder or a verge to fall off into; a kerbed
     * street already has a kerb doing that job. Motorway through Tertiary get
     * edge lines unconditionally.
     */
    bool edge_lines_on_minor_roads = false;

    bool emit_lane_lines = true;    ///< Lines between adjacent Lane strips
    bool emit_edge_lines = true;    ///< Lines just inside the outermost Lane strips
    bool emit_stop_lines = true;    ///< Transverse bars on junction approaches
    bool emit_arrows = true;        ///< Turn arrows on junction approaches

    /**
     * @brief Traffic drives on the LEFT (Ireland, UK)
     *
     * Two rules read this, and nothing else does:
     *
     * 1. **Which lane group is which.** The forward lanes -- those travelling from
     *    `from` towards `to` -- sit on the LEFT half of the profile under
     *    left-hand traffic and on the RIGHT half under right-hand traffic. This
     *    decides which boundary is the centre line, and it decides which lanes a
     *    `turn:lanes` value applies to. Getting it backwards paints the arrows on
     *    the oncoming carriageway.
     * 2. **Centre line colour.** Under left-hand traffic every line is white:
     *    MarkingSprite::DashWhite for an ordinary centre line and two
     *    MarkingSprite::SolidWhite lines where overtaking is forbidden, because
     *    yellow paint means a kerbside restriction there, not a lane division.
     *    Under right-hand traffic the centre line separating opposing flows is
     *    yellow: DashedYellow, SolidYellow, or DoubleSolidYellow. Lane lines
     *    WITHIN one direction of travel are white under both conventions.
     */
    bool left_hand_traffic = false;
};

// ============================================================================
// Emission
// ============================================================================

/**
 * @brief All longitudinal marking geometry for one edge
 *
 * ### Which lines are painted
 *
 * The strip list is walked left to right, tracking the lateral coordinate the
 * same way RoadProfile documents it. A line is painted at:
 *
 * - **Every boundary between two adjacent Lane strips**, when
 *   MarkingConfig::emit_lane_lines. The boundary dividing the forward lane group
 *   from the backward one is the CENTRE line and takes the colour rule in
 *   MarkingConfig::left_hand_traffic; every other Lane-to-Lane boundary is a
 *   white broken lane line. The dividing boundary is found from
 *   GraphEdge::lanes_forward and lanes_backward when both are set, and otherwise
 *   by splitting the Lane run in half. A one-way edge has no centre line at all.
 * - **Just inside each outermost Lane strip**, offset inward by
 *   MarkingConfig::edge_line_inset, when MarkingConfig::emit_edge_lines. Edge
 *   lines are solid white on both sides.
 *
 * A broken line is emitted as one quad per dash on a `dash_length` / `dash_gap`
 * cycle measured in arclength, because an atlas rect cannot wrap; see
 * marking_atlas.hpp. A solid line is emitted as one quad per station band, so it
 * follows the centerline's curvature and its miter rather than cutting corners.
 *
 * ### Road class decides the centre line and the edge lines
 *
 * Both are class rules, not width rules, and both default to painting LESS
 * rather than more, because a line that should not be there is a false
 * instruction while a missing one is only a missing detail.
 *
 * | Class                  | Centre line          | Edge lines |
 * |------------------------|----------------------|------------|
 * | Motorway, Trunk        | never                | always     |
 * | Primary, Secondary     | yes                  | always     |
 * | Tertiary               | yes                  | always     |
 * | Residential, Service   | centre_line_on_minor_roads | edge_lines_on_minor_roads |
 * | Unknown                | centre_line_on_minor_roads | edge_lines_on_minor_roads |
 * | Footway, Cycleway, Path| edge not painted at all           ||
 *
 * Motorway and Trunk are excluded from the centre line because their opposing
 * carriageways are separated by a Median strip: the two lane groups are not
 * adjacent, so no Lane-to-Lane boundary divides them and there is nothing to
 * paint. Where such a road IS mapped as one undivided way, the rule still holds
 * and the divider is left unpainted rather than drawn down the middle of an
 * island.
 *
 * Lane lines WITHIN a direction of travel are painted on every class, because a
 * road with two lanes going the same way always divides them.
 *
 * Where overtaking is forbidden -- `overtaking=no`, `overtaking=forward`, or
 * `overtaking=backward` in @p tags -- the centre line becomes solid over the
 * whole edge. The directional values are treated as `no` in this pass; a
 * one-sided solid-and-broken pair is not modelled.
 *
 * ### Which edges are painted at all
 *
 * An edge is skipped, and an empty Mesh returned, when its profile holds no Lane
 * strip, when its type is Footway, Cycleway, or Path, or when @p cl is invalid.
 * A CycleLane strip receives a MarkingSprite::BikeSymbol at intervals and a lane
 * whose `psv`, `bus`, or `lanes:psv` tag marks it as a bus lane receives
 * MarkingSprite::BusSymbol, in both cases centred on the strip.
 *
 * ### Heights
 *
 * @p station_heights is the same vector handed to CorridorConfig::station_heights
 * for this edge, so the paint sits on the road rather than through it. It must
 * have exactly `cl.stations.size()` entries. An empty vector, or any other size,
 * is treated as a flat road at world Y 0 -- a mis-sized elevation solve degrades
 * to flat paint rather than to paint threaded through the terrain. Every emitted
 * vertex is raised a further MarkingConfig::height_above_surface.
 *
 * @param edge            Graph edge being painted; tag-derived fields are read from it
 * @param cl              Centerline of that edge. Pass the UNTRIMMED centerline:
 *                        the paint runs the full length of the ribbon and stops
 *                        where the junction solver trimmed it, which
 *                        GraphEdge::trim_from and trim_to already record.
 * @param profile         Cross-section of that edge; the source of every lateral
 * @param station_heights World Y per station; see above
 * @param cfg             Widths, spacings, and the per-pass switches
 * @param tags            Optional raw way tags, from ParsedOSMData::ways keyed by
 *                        GraphEdge::source_way. May be null, in which case only
 *                        the fields promoted onto GraphEdge are consulted.
 * @param dash_phase      Arc length of the STREET already travelled before this
 *                        edge's arclength zero, metres. The dash pattern is
 *                        indexed against `arclength + dash_phase`, so a street
 *                        split into several edges at plain degree-2 continuations
 *                        -- which is what a `name` or `maxspeed` change produces,
 *                        with no trim and no junction between them -- keeps one
 *                        continuous pattern instead of restarting at each split
 *                        and merging two dashes into one. Zero is correct for an
 *                        edge considered on its own.
 * @return Marking geometry in MaterialId::Markings; empty when nothing is painted
 */
[[nodiscard]] Mesh build_edge_markings(const GraphEdge& edge,
                                       const Centerline& cl,
                                       const RoadProfile& profile,
                                       const std::vector<float>& station_heights,
                                       const MarkingConfig& cfg,
                                       const TagMap* tags = nullptr,
                                       double dash_phase = 0.0);

/**
 * @brief Stop line and turn arrows on one arm approaching a junction
 *
 * ### Which end, and which way traffic is going
 *
 * @p at_start selects the end of the edge the junction sits at. It decides the
 * direction of APPROACH, and getting it wrong paints the stop line on the exit:
 *
 * - `at_start == false`: the junction is at the edge's `to` node. Traffic
 *   approaches travelling FORWARD along the edge. The cut is at arclength
 *   `cl.length() - edge.trim_to`, the setback runs back towards decreasing
 *   arclength, and the approaching lanes are the FORWARD group.
 * - `at_start == true`: the junction is at the edge's `from` node. Traffic
 *   approaches travelling BACKWARD along the edge. The cut is at arclength
 *   `edge.trim_from`, the setback runs forward towards increasing arclength, and
 *   the approaching lanes are the BACKWARD group. Every quad emitted here is in
 *   the reversed frame: "left of travel" is the profile's RIGHT, and the sprite's
 *   v0 edge faces back down the edge.
 *
 * Pass the UNTRIMMED centerline. The trim values are in the edge's own untrimmed
 * parameterisation, which is the only frame in which they mean anything.
 *
 * Nothing is emitted when the approaching lane group is empty, which is the case
 * for a one-way edge approached from its exit end. An edge with no Lane strip,
 * or an invalid centerline, likewise emits nothing.
 *
 * ### What is emitted
 *
 * - **Stop line**, when MarkingConfig::emit_stop_lines. One quad per approaching
 *   lane, spanning that lane's width, MarkingConfig::stop_line_width thick along
 *   travel, its downstream edge MarkingConfig::stop_line_setback back from the
 *   cut. @p has_signals selects the sprite: MarkingSprite::StopLine at a
 *   signalled junction, MarkingSprite::GiveWayTriangles repeated across the lane
 *   at a priority junction, which is the give-way line the plan asks for.
 * - **Turn arrows**, when MarkingConfig::emit_arrows. One arrow per approaching
 *   lane, centred on the lane, its downstream tip MarkingConfig::arrow_spacing
 *   back from the stop line. The sprite comes from the `turn:lanes` family in
 *   @p tags -- `turn:lanes:forward` and `turn:lanes:backward`, selected by the
 *   approach direction, plus the undirected `turn:lanes` on a ONE-WAY edge only,
 *   since on a two-way edge it names no direction -- whose pipe-separated
 *   values are ordered LEFT TO RIGHT in the direction of travel under both
 *   traffic conventions. `through` maps to ArrowStraight, `left` to ArrowLeft,
 *   `right` to ArrowRight, `through;left` to ArrowStraightLeft, `through;right`
 *   to ArrowStraightRight, `reverse` to ArrowUTurn, and an empty or unrecognised
 *   entry to ArrowStraight. With no usable tag, no arrow is emitted at all
 *   rather than a guessed one: an invented arrow is a false instruction.
 *
 * ### Bad `turn:lanes` data
 *
 * A lane count that disagrees with the profile is the common failure of this
 * tag, and there is no safe way to guess which lanes the values meant: shifting
 * them by one paints "left only" over a lane that goes straight on. When the
 * pipe-separated entry count does not equal the approaching lane count, NO
 * arrows are emitted for that approach and the mismatch is logged. The same
 * applies to a value combining two turn directions with no through movement,
 * such as `left;right`, which no single sprite can state: that lane alone is
 * skipped, and its neighbours are still painted, because the entry-to-lane
 * mapping is unaffected.
 *
 * ### Height
 *
 * Pass @p station_heights: the same per-station heights the CORRIDOR of this edge
 * was placed at, one per station of @p cl, already carrying
 * ElevationConfig::surface_offset. The junction solver flattens each arm mouth
 * onto the junction plane, so those heights already ARE the junction plane over
 * the trim and the elements nearest the cut land on it; further back they follow
 * the solved grade, which is where the corridor is.
 *
 * An approach reaches a long way back from the cut. A turn arrow is
 * MarkingConfig::arrow_spacing (25 m by default) behind the stop line, and the
 * plateau is only as wide as the trim, so an approach emitted wholly on the
 * junction plane buries its arrows a couple of metres under a graded
 * carriageway.
 *
 * @p height is the fallback for when no per-station heights are available: the
 * world Y of the CARRIAGEWAY SURFACE at the junction, that is Junction::height.
 * The whole approach is then emitted on that single plane, which is exactly
 * right on flat ground and is the best available answer without a solve.
 * MarkingConfig::height_above_surface is added on top of either.
 *
 * @param edge            Graph edge forming the arm; trim_from and trim_to must be solved
 * @param cl              UNTRIMMED centerline of that edge
 * @param profile         Cross-section of that edge
 * @param at_start        True when the junction is at the edge's `from` node
 * @param has_signals     GraphNode::has_signals of the junction node
 * @param height          World Y of the junction carriageway surface; the fallback
 * @param cfg             Widths, spacings, and the per-pass switches
 * @param tags            Optional raw way tags; may be null. Without them no arrow is emitted.
 * @param station_heights Optional; one entry per station of @p cl. A null or
 *                        mis-sized vector falls back to @p height, matching
 *                        CorridorConfig::station_heights.
 * @return Approach geometry in MaterialId::Markings; empty when nothing is painted
 */
[[nodiscard]] Mesh build_approach_markings(const GraphEdge& edge,
                                           const Centerline& cl,
                                           const RoadProfile& profile,
                                           bool at_start,
                                           bool has_signals,
                                           float height,
                                           const MarkingConfig& cfg,
                                           const TagMap* tags = nullptr,
                                           const std::vector<float>* station_heights = nullptr);

} // namespace stratum::osm::road
