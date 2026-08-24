/**
 * @file junction_trim.hpp
 * @brief How far back each arm of a junction is cut, and where the cut lands
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This is step 2 of the P4 junction solver in docs/plans/road_network_plan.md,
 * and the reason ribbons stop running straight through each other.
 *
 * Today every corridor is extruded over the full length of its edge, so at a
 * four-way junction four ribbons overlap in the middle in a Z-fighting pile.
 * The fix is analytic and purely one-dimensional: decide, per arm, an ARCLENGTH
 * to cut off that end of the edge. Everything else in P4 -- the junction
 * polygon, the fillet arcs, the curb ring, the terrain re-carve -- is built on
 * top of the cut positions this file produces.
 *
 * ### What lives here and what does not
 *
 * This header is geometry of LINES only. It produces no polygon, no ring and no
 * triangle. It answers two questions:
 *
 * 1. `solve_arm_trims()` -- how far back is each arm cut?
 * 2. `arm_end()` -- given that cut, where exactly are the arm's cross-section
 *    corners?
 *
 * The polygon that joins those cross-sections together lives in
 * junction_polygon.hpp, and the sidewalk ring around it in junction_curb.hpp.
 *
 * ### The pairwise rule
 *
 * Arms are already sorted ascending by bearing by P1 (GraphNode::arms), which
 * is counter-clockwise order in the 2D local frame. For each ADJACENT PAIR in
 * that order the two arms' NEAR-SIDE offset lines are intersected: the LEFT
 * offset of the earlier arm and the RIGHT offset of the later one, both taken
 * at the carriageway half width, both measured in the direction LEAVING the
 * node. The intersection is the first point at which the two carriageways stop
 * overlapping, and its distance from the node along each arm is that pair's
 * demand on that arm.
 *
 * An arm has two neighbours -- the previous and the next in bearing order -- so
 * it carries two demands and takes the LARGER, plus TrimConfig::clearance. Taking
 * the maximum is what makes the result independent of which neighbour is
 * considered first, and therefore stable under any re-ordering of the input that
 * preserves the bearing cycle.
 *
 * Each pair's demand also carries the straight run the CORNER FILLET between the
 * two arms is tangent over, `R * tan(theta / 2)`. Without it the fillet has only
 * TrimConfig::clearance of run to sit on and every square corner collapses to a
 * chamfer a few centimetres long; see TrimConfig::fillet_radius_width_factor.
 *
 * ### Coordinates
 *
 * Everything is the same 2D local metres as GraphEdge::polyline and
 * Centerline::stations. No render-space mapping is applied anywhere in this
 * file; that belongs to whatever finally emits triangles.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Arms
// ============================================================================

/**
 * @brief One arm of a junction: an edge end, with the geometry needed to cut it back
 *
 * An ArmRef is a resolved GraphNode::Arm. The graph's Arm carries topology --
 * which edge, which end, which bearing -- and this adds the cross-section widths
 * the trim solve needs, so the solver never has to look at a RoadProfile itself.
 *
 * Fill order, which the whole file depends on:
 *
 * - `edge`, `at_start` and `bearing` are copied straight from GraphNode::arms.
 * - `half_width` and `carriageway_half` come from the arm's RoadProfile.
 * - `trim` and `clamped` are the OUTPUTS. They are the only fields
 *   solve_arm_trims() writes.
 *
 * collect_arms() does all of the filling except the outputs, and is the only
 * supported way to build the vector, so no caller can disagree with the solver
 * about what a width means.
 *
 * An edge whose two endpoints are the same graph node -- a closed loop, common
 * on roundabouts and cul-de-sac bulbs -- contributes TWO ArmRefs sharing one
 * EdgeId, one with at_start true and one false. Nothing here treats that as a
 * special case, but a caller writing trims back onto the edge must write both.
 */
struct ArmRef {
    /// Edge this arm belongs to; indexes RoadGraph::edges()
    EdgeId edge = kInvalidId;

    /// True when this node is the edge's `from` end, false when it is `to`
    bool at_start = true;

    /**
     * @brief Bearing of the direction LEAVING the node, radians in [-pi, pi]
     *
     * Copied verbatim from Arm::bearing, which P1 already sorted ascending. It is
     * the pairing key: arms adjacent in this ordering are adjacent around the
     * junction, counter-clockwise.
     */
    double bearing = 0.0;

    /**
     * @brief Half the arm's TOTAL profile width, metres
     *
     * `RoadProfile::total_width() * 0.5`. This is a reach, not an offset: a
     * profile with a sidewalk on one side only is not centred on its own total
     * width, so this value does NOT locate either outer edge. It is used for the
     * fillet radius (see FilletConfig::radius_width_factor) and as the curb
     * ring's reach. The exact, possibly asymmetric, outer corners are computed by
     * arm_end() from the profile itself.
     */
    double half_width = 0.0;

    /**
     * @brief Half the CARRIAGEWAY envelope, metres: lanes, median, and what lies between
     *
     * Unlike half_width this really is an offset, because
     * RoadProfile::left_edge_offset() centres the Lane+Median span on zero: the
     * carriageway occupies lateral [-carriageway_half, +carriageway_half]
     * exactly.
     *
     * It is therefore HALF THAT SPAN -- the inclusive run of strips from the
     * first Lane-or-Median to the last -- and not `carriageway_width() * 0.5`.
     * carriageway_width() sums Lane strips alone, so it omits the median and the
     * gutters and curbs inside a dual carriageway, and centring it on zero would
     * not agree with left_edge_offset(). A profile with no Lane and no Median at
     * all -- a bare footway -- is centred whole by left_edge_offset(), so its
     * envelope is its whole width.
     *
     * This, not half_width, is what the trim solve intersects. The junction
     * polygon is a CARRIAGEWAY footprint; the sidewalks around it are the curb
     * ring's job, and they are offset outward from that footprint rather than
     * cut into it.
     */
    double carriageway_half = 0.0;

    /**
     * @brief SOLVED: arclength in metres cut from THIS end of the edge
     *
     * Written by solve_arm_trims() and by nothing else. It is a distance measured
     * from the node along the arm, so for at_start it maps onto
     * GraphEdge::trim_from and otherwise onto GraphEdge::trim_to, with no sign
     * change in either direction.
     *
     * Always >= TrimConfig::min_trim and always <= the clamp described in
     * TrimConfig::max_trim_fraction, except where the both-ends safety cap in
     * that same paragraph overrides the floor on a very short edge.
     */
    double trim = 0.0;

    /**
     * @brief SOLVED: the demanded trim was reduced by a clamp
     *
     * Written by solve_arm_trims() alongside `trim`. True when the pairwise solve
     * asked to cut further than TrimConfig::max_trim_fraction, or the both-ends
     * safety cap, allowed. The arm is then OVER-TRIMMED geometry in the sense
     * TrimConfig::max_trim_fraction describes: the junction polygon overlaps that
     * ribbon slightly.
     *
     * This is the signal for JunctionBuilder::Stats::over_trimmed_edges, which
     * counts per EDGE, so a caller must fold the two arms of one edge together
     * rather than counting each.
     *
     * A trim RAISED to meet TrimConfig::min_trim is NOT a clamp: nothing overlaps
     * because of it, and it does not set this flag.
     */
    bool clamped = false;
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tolerances of the trim solve. All distances are metres.
 */
struct TrimConfig {
    /**
     * @brief Extra metres cut beyond the geometric intersection
     *
     * The intersection point is where the two carriageways stop overlapping
     * EXACTLY. Cutting there leaves the two arm ends touching along a zero-width
     * sliver, which produces coincident geometry and a visible seam. A small
     * positive clearance opens that sliver into the corner the fillet arc is
     * drawn across.
     */
    double clearance = 0.25;

    /// Floor applied to every solved trim, even where no pair demands one
    double min_trim = 0.0;

    /**
     * @brief Never cut more than this fraction of an edge's length from one end
     *
     * Short edges between close junctions -- a slip road stub, a divided
     * carriageway's crossover -- would otherwise be cut to nothing from both ends
     * at once and vanish, taking their piece of the network with them.
     *
     * The clamp is applied PER END against the arm's own centerline length, so
     * the two ends of one edge clamp independently and together can consume at
     * most twice this fraction. At the default 0.4 that leaves 20% of the
     * shortest edge still extruded. It is deliberately not a joint budget: a
     * joint one would make an arm's trim depend on the far end of its edge, and
     * therefore on a junction it does not touch.
     *
     * An arm whose demand was clamped is over-trimmed geometry: the junction
     * polygon will overlap that ribbon slightly. That is the lesser failure and
     * the caller is expected to count it (JunctionBuilder::Stats::over_trimmed_edges),
     * which ArmRef::clamped reports.
     *
     * One bound outranks both this fraction and min_trim: no end may take more
     * than HALF its edge less a hair, so however this and min_trim are configured
     * the two ends together always leave the edge a positive length. A per-end
     * solve cannot see the far end, and a negative-length ribbon is the single
     * worst thing this file could hand the extruder, so that cap is not
     * negotiable by configuration. At the default 0.4 it never binds.
     */
    double max_trim_fraction = 0.4;

    /**
     * @brief Nominal fillet radius the trim must leave room for: width factor
     *
     * MIRRORS FilletConfig::radius_width_factor, and the three fields below
     * mirror the rest of that struct. They are copied rather than referenced
     * because junction_polygon.hpp, which owns FilletConfig, is built ON TOP of
     * this header and cannot be included from it. apply_fillet_reserve(),
     * declared in junction_polygon.hpp, is the supported way to keep the two in
     * step, and JunctionBuilder calls it for every solve.
     *
     * ### Why the trim has to know about the fillet at all
     *
     * A corner fillet of radius R across a turn of theta is tangent to both cut
     * faces at `R * tan(theta / 2)` back from the point where the two offset
     * lines meet. If the arm is cut at that meeting point plus nothing but
     * `clearance`, the straight run available to the arc is `clearance` -- 0.25 m
     * -- and the radius that fits is 0.25 / tan(theta / 2), which at a square
     * corner is 0.25 m and falls straight through FilletConfig::min_radius. Every
     * right-angle junction then comes out as a plus shape with four centimetre
     * chamfers, which is exactly the disc-shaped failure P4 exists to remove.
     *
     * So each pair's demand carries the tangent run as well as the intersection
     * distance: `intersection + R * tan(theta / 2)`, with R derived from these
     * four values exactly as build_junction_polygon() derives it from
     * FilletConfig. The clearance is then added on top, so the fillet's tangent
     * point still lands `clearance` inside the cut face rather than exactly on it.
     *
     * Set radius_width_factor to zero to reserve nothing, which restores the
     * pre-fillet trim distances.
     */
    double fillet_radius_width_factor = 0.75;

    /// Mirrors FilletConfig::min_radius; a pair reserving less than this reserves nothing
    double fillet_min_radius = 0.5;

    /// Mirrors FilletConfig::max_radius
    double fillet_max_radius = 12.0;

    /**
     * @brief Mirrors FilletConfig::min_arc_angle, radians
     *
     * A corner shallower than this is drawn as a chord, so it needs no tangent
     * run and reserves nothing. This is also what keeps `tan(theta / 2)` away
     * from its pole at theta = pi.
     */
    double fillet_min_arc_angle = 0.15;

    /**
     * @brief Below this |sin(angle between arms)| two arms count as parallel
     *
     * Two parallel near-side offset lines never meet, so there is no intersection
     * to project and the exact formula would divide by zero. Dimensionless, not
     * metres. See solve_arm_trims() for what the pair contributes instead, which
     * differs between arms pointing the same way and arms pointing opposite ways.
     */
    double parallel_epsilon = 1e-6;

    /**
     * @brief Floor on the angle between two arms when solving their pair demand, radians
     *
     * The exact pairwise answer for two arms `theta` apart, with carriageway half
     * widths `wa` and `wb`, is `(wb + wa * cos theta) / sin theta`. That is right
     * and it is unbounded: it diverges as `1 / theta`, so a slip road leaving a
     * trunk at five degrees demands eighty metres of trim, one at two degrees
     * demands two hundred, and one at half a degree demands a kilometre. Measured
     * on a 63 MB Dublin extract before this floor existed, the longest solved trim
     * was 294 m and the largest junction polygon covered 38,000 m^2 -- an
     * intersection two hundred metres long, filled with asphalt.
     *
     * Nothing about that is an improvement on the overlap it removes. Worse, an
     * arm short enough to hit TrimConfig::max_trim_fraction is then cut to a
     * fraction of the demand, which is the one condition under which two adjacent
     * cut faces cross each other and the junction ring stops being simple.
     *
     * So the angle used in the division is floored here: below `min_pair_angle`
     * the pair is solved AS IF the two arms were exactly this far apart, which
     * bounds its demand at `(wa + wb) / sin(min_pair_angle)` -- 3.86 times the
     * combined half widths at the default 15 degrees, and 27 m for two ordinary
     * 7 m carriageways. The two arms then still overlap beyond the cut, which is
     * honest: a five-degree fork's carriageways really do overlap for eighty
     * metres, and no trim short of eighty removes that. What the floor buys is
     * that the overlap stays a local defect at the nose of the gore instead of
     * becoming a two-hundred-metre junction polygon.
     *
     * The floor binds only below itself, so every junction that reads as a
     * junction rather than as a fork is solved exactly as before. Set it to zero
     * to restore the unbounded exact answer.
     *
     * @note This does NOT change the exactly-parallel co-directional case, which
     *       is not solved by the formula at all and contributes `wa + wb`. The two
     *       therefore still disagree across `parallel_epsilon`, by a factor of
     *       `1 / sin(min_pair_angle)`, but the gap they disagree across is a
     *       millionth of a radian wide and both answers are now bounded.
     */
    double min_pair_angle = 0.2617993877991494;   // 15 degrees
};

// ============================================================================
// Solve
// ============================================================================

/**
 * @brief Default JunctionBuilder radius below which two junction nodes are one
 *
 * The default argument of collect_arms(), named so the builder can pass it
 * explicitly alongside the cluster out-parameter without restating the literal.
 * See the collect_arms() documentation for why it is where it is.
 */
constexpr double kCoincidentRadius = 1.0;

/**
 * @brief Resolve a node's graph arms into ArmRefs, ready for the trim solve
 *
 * Copies edge, at_start and bearing from GraphNode::arms -- preserving their
 * ascending-bearing order, which every later step depends on -- and fills
 * half_width and carriageway_half from the matching RoadProfile. `trim` is left
 * at zero for solve_arm_trims() to write.
 *
 * An arm whose EdgeId is out of range for @p profiles, or whose profile is
 * invalid, is still emitted, with both widths zero. Dropping it instead would
 * change the arm CYCLE and therefore silently pair two arms that are not
 * neighbours.
 *
 * ### Near-coincident junctions are collected as ONE junction
 *
 * RoadGraph merges two OSM nodes into one graph node only when their positions
 * agree to 1e-6 m, which is the exact-duplicate defect. A pair a few CENTIMETRES
 * apart -- two mappers tracing the same crossroads, a way split twice at what
 * was meant to be one point, a stub left behind by a conflation -- survives as
 * two graph nodes joined by a stub edge shorter than either junction is wide.
 * Solved separately they produce two junction polygons covering the same ground:
 * two fills, two curb rings, two carve footprints, all overlapping, which is a
 * worse artefact than either junction alone.
 *
 * So a node of degree 3 or more first collects its CLUSTER: itself, plus every
 * junction node reachable from it over edges shorter than @p coincident_radius.
 * The cluster is a property of the graph and not of where the walk started, so
 * every member computes the same one, and the member with the LOWEST GraphNodeId
 * is the cluster's primary.
 *
 * - The **primary** is given the arms of every node in the cluster, minus the
 *   internal stubs that hold the cluster together, re-sorted into ascending
 *   bearing order. It is one junction with all the real approaches, which is what
 *   the geometry should have been in the first place.
 * - Every **other member** is given NO arms at all. solve_arm_trims() then
 *   reports it degenerate and the caller emits nothing for it, which is the
 *   intended outcome: the primary already covers that ground.
 *
 * @note A suppressed member is counted by JunctionBuilder::Stats::degenerate,
 *       alongside genuinely unsolvable nodes. The two are not distinguished
 *       today.
 *
 * ### The cluster must be reported, not just acted on
 *
 * Returning an empty arm list is not enough on its own. Several consumers key off
 * a node id rather than off a junction -- approach markings ask
 * `node_has_junction[edge.from]`, and dropped_kerb_spans() filters crossings on
 * `Crossing::node` -- and an absorbed member answers "no junction here" to all of
 * them even though its approaches were trimmed back by the primary's solve. The
 * result is silent: an approach to a merged junction loses its stop line, and a
 * junction crossing on it demands a kerb drop from a ring that does not exist,
 * so the corridor's dropped kerb butts an undropped ring at the arm mouth.
 *
 * @p out_cluster exists so the caller can resolve a member to its primary. It is
 * filled on EVERY path -- a lone junction reports the cluster `{node}` -- so the
 * caller never has to distinguish "no cluster" from "not asked".
 *
 * The origin every pair demand is measured from is the PRIMARY's position, so an
 * absorbed arm's demand carries an error of at most @p coincident_radius. That is
 * the whole point of keeping the radius small: it must be well under a
 * carriageway width, or a merge moves geometry further than it repairs.
 *
 * Set @p coincident_radius to zero to disable merging and collect exactly the
 * node's own arms, which is the behaviour this function had before merging
 * existed.
 *
 * @param graph              Built road graph
 * @param profiles           Parallel to graph.edges(); profiles[i] belongs to edge i
 * @param node               Node to collect the arms of
 * @param coincident_radius  Metres below which a stub edge between two junction
 *                           nodes means they are one junction. Zero disables the
 *                           merge. Degree 1 and 2 nodes never merge, whatever it
 *                           is set to.
 *
 *                           The default 1.0 is deliberately short of what the
 *                           geometry alone would justify. Measured on a Dublin
 *                           extract, raising it to 2 m merges 194 nodes instead of
 *                           45 and cuts the number of OVERLAPPING junction
 *                           polygon pairs from 288 to 162, which is the largest
 *                           single win available here. Two side effects are what
 *                           hold it back, and both are outside this file:
 *
 *                           - the internal stub edge is left untrimmed, so its
 *                             ribbon lies coplanar inside the merged fill. The
 *                             caller wants to consume such an edge the way a
 *                             roundabout's ring edges are consumed.
 *                           - the suppressed member keeps its OWN solved node
 *                             height, while the fill sits at the primary's, so an
 *                             arm mouth can open a step of up to
 *                             `radius * max_grade`.
 *
 *                           Both are bounded by the radius, which is why the
 *                           default is where it is: at 1 m they are a 1 m stub and
 *                           a 10 cm step.
 * @param out_cluster        Optional. Receives the coincident cluster @p node
 *                           belongs to, in ascending GraphNodeId order, so
 *                           `front()` is the primary. Filled on every path that
 *                           reaches a valid node, including the common case of a
 *                           node that merges with nothing, which reports
 *                           `{node}`. Left EMPTY only when @p node is out of
 *                           range. Any prior contents are overwritten.
 * @return The arms in ascending bearing order; empty when @p node is out of
 *         range, or when it is a non-primary member of a coincident cluster
 */
[[nodiscard]] std::vector<ArmRef> collect_arms(const RoadGraph& graph,
                                               const std::vector<RoadProfile>& profiles,
                                               GraphNodeId node,
                                               double coincident_radius = kCoincidentRadius,
                                               std::vector<GraphNodeId>* out_cluster = nullptr);

/**
 * @brief Solve the trim distance for every arm of one node
 *
 * For each ADJACENT PAIR of arms in bearing order, intersect the near-side
 * offset lines of the two arms; an arm's trim is the maximum over its two
 * neighbours, plus TrimConfig::clearance.
 *
 * Precisely, for arms `a` and `b = next(a)` in the bearing cycle, with unit
 * leaving directions `da`, `db` and left normals `na = (-da.y, da.x)`,
 * `nb = (-db.y, db.x)`, taken from the node position `P`:
 *
 * @code
 *     La(t) = P + na * a.carriageway_half + da * t     // a's LEFT edge, facing b
 *     Lb(s) = P - nb * b.carriageway_half + db * s     // b's RIGHT edge, facing a
 * @endcode
 *
 * `t` is the pair's demand on `a` and `s` its demand on `b`. A negative
 * parameter means that arm does not have to retreat for this pair -- the two
 * carriageways already diverge -- and contributes zero rather than a negative
 * trim.
 *
 * Both parameters are then advanced by the pair's FILLET TANGENT RUN,
 * `R * tan(theta / 2)`, where theta is the turn the junction ring makes over the
 * corner between the two arms and R is the nominal radius derived from
 * TrimConfig's four `fillet_*` fields exactly as build_junction_polygon() derives
 * it from FilletConfig. A pair whose corner would be drawn as a chord -- shallower
 * than fillet_min_arc_angle, or turning the wrong way -- reserves nothing.
 *
 * The angle the pair is solved at is floored at TrimConfig::min_pair_angle, which
 * bounds a very acute pair's demand at `(wa + wb) / sin(min_pair_angle)` instead
 * of letting it diverge as `1 / theta`. See that field: the floor binds only
 * below itself, so an ordinary junction is solved exactly as it was before the
 * floor existed.
 *
 * A pair whose |cross(da, db)| is below TrimConfig::parallel_epsilon has no
 * intersection to project, and the two ways that happens behave oppositely:
 *
 * - **Anti-parallel** (`dot(da, db) <= 0`): a straight road running through the
 *   node, or an arm folded back on itself. The near-side edges run away from
 *   each other along one line, the carriageways abut instead of overlapping, and
 *   the pair contributes nothing on either side.
 * - **Co-directional** (`dot(da, db) > 0`): the two halves of a dual carriageway,
 *   or a service road leaving at a hair's-breadth angle. The carriageways overlap
 *   for their whole length and NO finite trim separates them; the exact formula
 *   would demand `(wa + wb) / |sin|` and blow up. The pair contributes
 *   `wa + wb` to both arms instead -- bounded, derived from the widths, and left
 *   for the max_trim_fraction clamp to have the final word on.
 *
 * Both cases are common in real extracts, so neither is an exotic path.
 *
 * The demands above are distances along each arm's LOCAL direction at the node,
 * which is a straight ray. A curving arm peels away from that ray, so its
 * projection grows more slowly than its arclength and the demand is reached only
 * after MORE arclength than the raw distance. Each demand is therefore converted
 * by walking the arm's own stations and interpolating the band whose projection
 * first meets it, and TrimConfig::clearance is added AFTER that conversion,
 * because the clearance is a distance along the ribbon rather than along the ray.
 *
 * Taking the MAXIMUM over an arm's two neighbours, rather than accumulating or
 * averaging, is what makes the result independent of the order the pairs are
 * visited in, so trim distances are stable under any input re-ordering that
 * preserves the bearing cycle. That property is directly tested.
 *
 * Each solved value is finally clamped into
 * `[cfg.min_trim, centerlines[edge].length() * cfg.max_trim_fraction]`, and
 * `min_trim` wins if the two bounds cross on a very short edge.
 *
 * Degree 1 and 2 nodes are handled by the caller -- see junction_special.hpp for
 * dead-end caps and profile tapers -- and are NOT handled here.
 *
 * @param graph       Built road graph
 * @param centerlines Parallel to graph.edges(); supplies each arm's length for
 *                    the max_trim_fraction clamp
 * @param node        Node being solved; supplies the arm origin position
 * @param arms        In/out. Must already hold this node's arms in ascending
 *                    bearing order, as returned by collect_arms(). Only the
 *                    `trim` field is written.
 * @param cfg         Tolerances; the defaults are the shipping values
 * @return false when the node is degenerate: fewer than 3 arms, or every
 *         adjacent pair ANTI-parallel, or @p node out of range. A co-directional
 *         pair is parallel but not degenerate -- it demands `wa + wb`, which is
 *         an answer -- so a node carrying one is solved, not rejected. Every
 *         degenerate arm's trim is
 *         then set to cfg.min_trim -- still bounded by the edge it is cut from --
 *         and ArmRef::clamped to false, so the output is always usable.
 */
bool solve_arm_trims(const RoadGraph& graph,
                     const std::vector<Centerline>& centerlines,
                     GraphNodeId node,
                     std::vector<ArmRef>& arms,
                     const TrimConfig& cfg);

// ============================================================================
// Cut cross-sections
// ============================================================================

/**
 * @brief Outer corner points of an arm's cross-section at its trim station
 *
 * All points are 2D local metres. `left` and `right` are relative to the
 * direction LEAVING the node, which is the OPPOSITE of the direction of travel
 * for an arm with at_start false. The profile's own left and right are relative
 * to travel, so arm_end() performs that flip; no consumer should flip again.
 *
 * Two pairs of corners are carried because two different consumers need two
 * different widths from the same cut:
 *
 * - `carriage_left` / `carriage_right` bound the CARRIAGEWAY. These are the
 *   points build_junction_polygon() chains into the junction ring, and the
 *   points the trimmed ribbon's outer carriageway columns must land on.
 * - `left` / `right` bound the FULL PROFILE, sidewalks and verges included.
 *   These are what build_curb_ring() uses to find where the ring must OPEN,
 *   because that is the width of the arm's own sidewalk arriving at the cut.
 *
 * Both pairs sit on the same cut line, `center + perpendicular * lateral`, so
 * the four points are collinear and ordered right, carriage_right,
 * carriage_left, left.
 */
struct ArmEnd {
    /// Centerline point at the trim station
    glm::dvec2 center{0.0};

    /// Outer LEFT corner of the full profile, left of the leaving direction
    glm::dvec2 left{0.0};

    /// Outer RIGHT corner of the full profile
    glm::dvec2 right{0.0};

    /// Carriageway's left edge at the cut; the junction ring vertex
    glm::dvec2 carriage_left{0.0};

    /// Carriageway's right edge at the cut; the junction ring vertex
    glm::dvec2 carriage_right{0.0};

    /// Unit direction LEAVING the node, at the trim station
    glm::dvec2 direction{1.0, 0.0};

    /**
     * @brief Station along the EDGE where the cut lands, metres
     *
     * In the edge's own parameterisation, measured from its `from` node, so it is
     * `arm.trim` for an arm at the start and `centerline.length() - arm.trim` for
     * one at the end. This is the value to hand slice(): the trimmed ribbon of an
     * edge is `slice(cl, arclength_at_from_end, arclength_at_to_end)`.
     */
    double arclength = 0.0;

    /**
     * @brief The cut is usable
     *
     * False when the arm's centerline is invalid, its profile is empty, or the
     * trim consumed the whole edge, in which case every point is left at the node
     * position and the junction must fall back to its degenerate path.
     */
    bool valid = false;
};

/**
 * @brief Locate an arm's cross-section corners at its solved trim station
 *
 * The station is found with slice() rather than by snapping to the nearest
 * resampled station, and the corners are then placed with offset_point(), so the
 * cut lands exactly ON the untrimmed ribbon's edge. Any other construction makes
 * the trimmed arm a different width from the corridor it was cut out of, and the
 * junction polygon then either overhangs the ribbon or leaves a gap at it. See
 * the slice() contract in centerline.hpp, which exists for this call.
 *
 * The laterals are read from the profile, not from ArmRef::half_width:
 * `RoadProfile::left_edge_offset()` for the outer left, that value less
 * `total_width()` for the outer right, and `+/- carriageway_half` for the
 * carriageway pair. A one-sided sidewalk therefore produces an asymmetric
 * ArmEnd, which is correct -- the way is the centreline of the carriageway, not
 * of the profile.
 *
 * @param graph       Built road graph
 * @param centerlines Parallel to graph.edges()
 * @param profiles    Parallel to graph.edges()
 * @param arm         A solved arm; ArmRef::trim must already be written
 * @return The cut cross-section. `valid` false, and every point at the node
 *         position, when the arm cannot be cut.
 */
[[nodiscard]] ArmEnd arm_end(const RoadGraph& graph,
                             const std::vector<Centerline>& centerlines,
                             const std::vector<RoadProfile>& profiles,
                             const ArmRef& arm);

} // namespace stratum::osm::road
