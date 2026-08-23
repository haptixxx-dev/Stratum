/**
 * @file junction_polygon.hpp
 * @brief The junction carriageway footprint, and its triangulation
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This is step 3 and step 4 of the P4 junction solver in
 * docs/plans/road_network_plan.md: build the junction polygon, then triangulate
 * it.
 *
 * junction_trim.hpp has already decided where each arm stops and where its
 * cross-section corners are. What is left is a hole in the middle of the
 * network, bounded by N cut faces with nothing between them. This file closes
 * that hole.
 *
 * ### Fillets are the point
 *
 * The old builder filled a junction with a 12-segment disc. A disc is wrong for
 * the same reason a square is wrong: a real intersection is a convex polygon
 * whose corners are ROUNDED, at a radius set by the two roads that meet there,
 * because that is the radius a vehicle turns through. The plan states it
 * directly: "the fillet is what makes an intersection read as an intersection
 * instead of a disc". A T-junction gets two large fillets and one shallow one; a
 * crossroads gets four equal ones; an acute fork gets a long sliver on the sharp
 * side and a fat curve on the obtuse side. None of those shapes is a disc.
 *
 * ### This is a CARRIAGEWAY footprint
 *
 * The ring runs through ArmEnd::carriage_left and ArmEnd::carriage_right, not
 * through the full-profile corners. Sidewalks are NOT part of this polygon; they
 * are the curb ring in junction_curb.hpp, offset outward from this ring. Keeping
 * the two apart is what lets the fill be asphalt and the ring be sidewalk with
 * one shared boundary and no overlap.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API. renderer/mesh.hpp is included for Mesh and MaterialId only; that header
 * is pure glm and is already compiled into stratum_core.
 */

#pragma once

#include "osm/road/junction_trim.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Corner rounding tolerances. All distances are metres.
 */
struct FilletConfig {
    /// Radius floor; below this the corner is drawn as a straight chamfer
    double min_radius = 0.5;

    /// Radius ceiling, so a motorway junction does not round itself into a disc
    double max_radius = 12.0;

    /**
     * @brief Fillet radius scales with the NARROWER of the two arms' widths
     *
     * `radius = radius_width_factor * min(2 * a.carriageway_half, 2 * b.carriageway_half)`,
     * clamped into [min_radius, max_radius]. The narrower arm is the one that
     * governs because it is the one a turning vehicle has to fit into; scaling by
     * the wider arm rounds a service road's mouth as though it were the main
     * road.
     *
     * The radius is further reduced when the two arm ends are too close together
     * to accept it, so a fillet never overshoots the cut faces it joins. See
     * build_junction_polygon().
     */
    double radius_width_factor = 0.75;

    /**
     * @brief Arc tessellation density
     *
     * Vertices emitted per 90 degrees of turn, so a 90 degree corner gets this
     * many segments and a 45 degree corner gets half of them, rounded up to at
     * least one. Density is therefore proportional to the turn rather than
     * constant per corner, which keeps chord error roughly uniform.
     */
    int segments_per_quarter_turn = 4;

    /**
     * @brief Below this turn angle the corner is a straight chamfer, radians
     *
     * A nearly straight join -- two arms leaving at almost 180 degrees, which is
     * what a side road produces on either side of a through road -- has an arc
     * that is indistinguishable from its chord, and tessellating it only spends
     * vertices and invites a self-intersection from floating point noise. 0.15 rad
     * is about 8.6 degrees.
     */
    double min_arc_angle = 0.15;
};

/**
 * @brief Copy the fillet parameters the trim solve has to reserve room for
 *
 * TrimConfig carries four `fillet_*` fields that mirror FilletConfig, because the
 * trim solve must leave each corner enough straight run for its arc's tangent
 * points and junction_trim.hpp -- which this header is built on top of -- cannot
 * include this one. This is the supported way to keep the two in step; calling it
 * is what makes `trim` and `fillet` describe one junction rather than two.
 *
 * JunctionBuilder::build() calls it for every solve, so a caller going through
 * the builder never has to.
 *
 * @param fillet   Corner rounding the polygon will be built with
 * @param out_trim In/out; only the four `fillet_*` fields are written
 */
void apply_fillet_reserve(const FilletConfig& fillet, TrimConfig& out_trim);

// ============================================================================
// Polygon
// ============================================================================

/**
 * @brief The closed carriageway footprint of one junction
 *
 * The ring is COUNTER-CLOCKWISE in the 2D local frame and closed implicitly: the
 * first point is NOT repeated at the end. That matches Corridor::outline and
 * CarveRibbon::outline, so a consumer needs one convention for every footprint
 * in the pipeline.
 *
 * Ring layout, walking counter-clockwise, for arms in ascending bearing order
 * (which is itself counter-clockwise):
 *
 * @code
 *     arm 0 cut face:  carriage_right, carriage_left      <- arm_ring_start[0]
 *     fillet arc 0 -> 1
 *     arm 1 cut face:  carriage_right, carriage_left      <- arm_ring_start[1]
 *     fillet arc 1 -> 2
 *     ...
 *     fillet arc N-1 -> 0   (closes the ring)
 * @endcode
 *
 * Each arm contributes EXACTLY TWO consecutive ring vertices, right first, and
 * the fillet after an arm joins that arm's LEFT corner to the next arm's RIGHT
 * corner. Right-before-left is what makes the traversal counter-clockwise: for an
 * arm leaving along +x the cut face runs from -y to +y, and the junction interior
 * then lies to the left of the walk.
 */
struct JunctionPolygon {
    /// Counter-clockwise ring in 2D local metres, first point not repeated
    std::vector<glm::dvec2> ring;

    /**
     * @brief Index into `ring` where each arm's cut cross-section starts
     *
     * Parallel to the arm list passed in, so `arm_ring_start[k]` is the index of
     * arm k's `carriage_right`, and `arm_ring_start[k] + 1` is its
     * `carriage_left`.
     *
     * The extruded ribbon must weld to exactly these two vertices, or the
     * junction shows a seam along every approach. This is the whole reason the
     * indices are published rather than left implicit: recovering them by
     * searching the ring for the nearest point works until two arms meet at an
     * acute angle and their corners land within tolerance of each other.
     */
    std::vector<size_t> arm_ring_start;

    /**
     * @brief Centroid of the ring, in 2D local metres
     *
     * The origin of the junction's own planar UV frame -- see
     * triangulate_junction() -- and the reference point the curb ring offsets
     * away from. Published so the fill and the ring cannot disagree about it.
     * Zero when the polygon is invalid.
     */
    glm::dvec2 centroid{0.0};

    /// At least 3 ring vertices and every arm accounted for in arm_ring_start
    bool valid = false;

    /**
     * @brief The ring crosses itself and is not a simple polygon
     *
     * Produced by a node whose arms are too wide for the space between them: a
     * pair of near-parallel arms cut back further than the third arm is long, a
     * fillet radius that survived clamping but still overshoots, or five arms
     * meeting inside one carriageway width. The ring is still emitted -- dropping
     * it would lose the junction entirely -- but earcut's output for it is
     * meaningless and a winding test against it punches holes in terrain, exactly
     * as documented for Corridor::outline_self_intersects.
     *
     * A consumer seeing this must fall back: triangulate_junction() emits the
     * CONVEX HULL of the ring instead, and the terrain carve falls back to the
     * CarveDisc path. Always false when the ring is empty.
     *
     * Detection is a PROPER-crossing test, so a ring that merely touches itself
     * at a point, or runs collinearly back along itself, is not reported. Its
     * tolerance is scale-relative and never exact zero: a fillet's tangent points
     * lie precisely ON the offset lines they were built from, so an ordinary
     * junction always contains exactly collinear runs whose determinants come out
     * as rounding noise of either sign.
     *
     * A ring that is simple but CLOCKWISE is a caller error -- arms not handed
     * over in ascending bearing order -- and is logged rather than flagged here,
     * because the fill still comes out correct while an outward curb offset would
     * not.
     */
    bool self_intersecting = false;
};

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Chain the arm end cross-sections in bearing order, joined by fillet arcs
 *
 * For each consecutive pair of arms (k, k+1) in the input order -- which must be
 * ascending bearing order, as collect_arms() produces -- the corner between arm
 * k's `carriage_left` and arm k+1's `carriage_right` is rounded:
 *
 * 1. The nominal radius comes from FilletConfig::radius_width_factor applied to
 *    the narrower of the two carriageways, clamped to [min_radius, max_radius].
 * 2. The radius is then reduced so the arc's two tangent points both stay ON
 *    their cut faces, that is, so neither tangent point runs past the arm's
 *    carriageway corner and back up the ribbon. An unreduced fillet that
 *    overshoots is the most common source of a self-intersecting ring.
 * 3. The turn angle is the angle between the two arms' leaving directions. Below
 *    FilletConfig::min_arc_angle the corner is emitted as a straight chamfer --
 *    the chord alone, no interior vertices -- rather than an arc.
 * 4. Otherwise the arc is tessellated at FilletConfig::segments_per_quarter_turn
 *    per 90 degrees, at least one segment.
 *
 * The arc is always convex outward, away from the junction centroid, and the
 * fillet therefore ADDS the wedge of carriageway a vehicle turns through rather
 * than cutting a diagonal across the corner. On a symmetric four-way junction
 * the three areas order strictly: sharp corner < fillet < straight chamfer.
 *
 * ### When the corner falls back to its chord
 *
 * The ring's own heading over a corner is `-ends[k].direction` on the way in and
 * `+ends[k+1].direction` on the way out, and an ordinary junction corner turns
 * RIGHT between the two. That is not a winding error: the footprint of a four-way
 * junction is a PLUS shape whose quadrant corners are reflex vertices of this
 * counter-clockwise ring, and rounding one pushes the boundary outward.
 *
 * A corner that turns LEFT instead is the wrap-around pair at a node whose arms
 * all leave within a half plane -- an acute fork, a slip road peeling away -- so
 * the gap the corner spans exceeds 180 degrees. Its two offset lines meet far
 * BEHIND the node, and any arc drawn through that intersection extrudes a spike
 * out of the back of the junction and folds the ring through itself. There is no
 * outward fillet for such a corner, so it is closed with the CHORD: no interior
 * vertices at all, exactly as for a chamfer.
 *
 * The chord is also what a corner gets when the two offset lines are parallel
 * (an arm's through-continuation on the far side of a T), when the intersection
 * lands past either cut face because a trim was clamped short, when the turn is
 * shallower than min_arc_angle, or when the radius that actually fits has fallen
 * below min_radius.
 *
 * ### Welding
 *
 * Fillet vertices coincident with their neighbour are dropped -- an arc that
 * reaches exactly to a cut face would otherwise duplicate that corner. Arm
 * vertices are NEVER dropped, even when an arm's carriageway has collapsed to
 * zero width and its two corners coincide, because the extruded ribbon welds to
 * `arm_ring_start[k]` and `arm_ring_start[k] + 1` and a ring that sometimes has
 * one vertex per arm puts a seam at every approach.
 *
 * The ring is then tested for self-intersection and `self_intersecting` set. The
 * test is an exhaustive pairwise segment crossing check; junction rings are tens
 * of vertices, so the quadratic cost is irrelevant here, unlike on a corridor
 * outline.
 *
 * @param arms Arms in ascending bearing order, trims already solved
 * @param ends Cut cross-sections, parallel to @p arms
 * @param cfg  Corner rounding tolerances; the defaults are the shipping values
 * @return The footprint. `valid` false when fewer than 3 arms are given, when
 *         @p ends is not the same size as @p arms, or when any ArmEnd is invalid.
 */
[[nodiscard]] JunctionPolygon build_junction_polygon(const std::vector<ArmRef>& arms,
                                                     const std::vector<ArmEnd>& ends,
                                                     const FilletConfig& cfg);

/**
 * @brief Triangulate the junction footprint at a constant height
 *
 * ### World mapping and winding
 *
 * The mapping is the one used everywhere in this codebase, Y up:
 *
 * @code
 *     (x, y_2d) -> glm::vec3(x, height, -y_2d)
 * @endcode
 *
 * A counter-clockwise 2D ring triangulated into counter-clockwise 2D triangles
 * comes out of that mapping with a +Y normal and a counter-clockwise front face,
 * which is what the renderer culls against. Emit the triangles in the order
 * earcut returns them for a counter-clockwise ring and the winding is already
 * right; do not flip it to "compensate" for the negated second axis.
 *
 * Every vertex normal is exactly +Y and every vertex colour is opaque white.
 * Appearance comes from the material bound per SubMesh range, not from baked
 * vertex colour, as everywhere else in the road pipeline.
 *
 * ### UVs
 *
 * Per the plan's UV Convention, the junction is PLANAR-PROJECTED in its own
 * local frame and does NOT continue the ribbon's arclength parameterisation,
 * because it has no single direction of travel:
 *
 * @code
 *     U = (p.x - poly.centroid.x) / uv_tiling(material).u_metres
 *     V = (p.y - poly.centroid.y) / uv_tiling(material).v_metres
 * @endcode
 *
 * The projection axes are the local frame's own, not any arm's, so the junction
 * texture does not rotate when an arm is added or removed. Texel density still
 * matches the ribbons feeding it, because both come from uv_tiling().
 *
 * ### Degenerate input
 *
 * A polygon with `self_intersecting` set is triangulated as the CONVEX HULL of
 * its ring instead: earcut's output for a self-crossing ring is arbitrary, and a
 * hull is guaranteed to at least cover every arm mouth. That is a visible
 * approximation, and the caller is expected to count it.
 *
 * @param poly     Footprint to fill; an invalid one yields an empty Mesh
 * @param height   World Y of the junction surface. This is the solved node height
 *                 PLUS ElevationConfig::surface_offset -- see
 *                 RoadElevationSolver::node_height(), which deliberately excludes
 *                 the offset so that each consumer adds it exactly once.
 * @param material Material slot for the fill, normally MaterialId::Asphalt
 * @return The filled junction as a single SubMesh range under @p material
 */
[[nodiscard]] Mesh triangulate_junction(const JunctionPolygon& poly,
                                        float height,
                                        MaterialId material);

} // namespace stratum::osm::road
