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

    /**
     * @brief How far past the node a corner point may lie, in combined widths
     *
     * Every corner of the ring is built from the point C where the two arms'
     * facing carriageway edges meet. C normally sits between the two cut faces
     * and the node -- that is what makes it a corner OF this junction -- and the
     * fillet is drawn tangent to both edges around it.
     *
     * Two nearly parallel arms break that. Their edges meet at a vanishing angle,
     * so C runs off to a kilometre away, and the arc built around it is tangent
     * over `radius * tan(theta / 2)`, which for a turn approaching half a circle
     * is hundreds of times the radius. Measured on a Dublin extract, one node
     * whose two arms left 0.26 degrees apart produced a junction polygon of
     * 38,000 m^2 whose ring ran 2.5 KILOMETRES away from the node and came back.
     * The trim solve already bounds its own mirror of that reserve; this is the
     * same bound on the polygon side.
     *
     * A corner point further behind a cut face than that arm's own trim plus this
     * many COMBINED carriageway half widths is therefore not treated as a corner
     * at all, and the corner is closed with its chord.
     *
     * The same bound governs the one REFLEX corner a node can have -- the
     * wrap-around pair at a node whose arms all leave within one half plane, an
     * acute fork or a slip road. That corner spans the BACK of the node, where no
     * arm arrives, and taking the ring through C wraps the fill around the back
     * of the node exactly as the cut faces wrap around its front. Closing it with
     * the bare chord instead, which is what this file used to do, draws a
     * diagonal across the junction that passes on the WRONG SIDE of the node -- so
     * the polygon does not contain the node it was built for, the terrain carve
     * leaves the ground under it unflattened, and on a three-arm fork the diagonal
     * crosses the opposite corner's fillet. A reflex corner is never rounded: an
     * arc through it would bulge into the junction rather than out of it.
     *
     * The default 1.0 allows a corner about one carriageway clear of the node.
     * Zero allows only a corner between the cut faces and the node; a negative
     * value is treated as zero.
     */
    double max_corner_reach_factor = 1.0;
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
     * A ring that is simple but CLOCKWISE is reported by `inverted`, not here, so
     * that this flag keeps meaning exactly "the ring crosses itself".
     */
    bool self_intersecting = false;

    /**
     * @brief The ring is wound CLOCKWISE, so it does not bound what it looks like
     *
     * Two things produce it. Arms handed over out of bearing order is the caller
     * error the header always warned about. The other is internal: the adjacent
     * cut-face clipping collapses an over-clipped arm onto its own midpoint, but
     * its two NEIGHBOURS keep the crossing points they were given, so on a
     * trident node -- three or more arms inside one narrow fan, nothing opposing
     * them -- the walk reverses between them and the ring comes back with
     * negative area while remaining perfectly simple.
     *
     * `self_intersecting` cannot catch that, because the ring really is simple.
     * Without a separate flag the whole junction would be handed on as a valid
     * outline: a backwards sliver metres from the node, used as the asphalt fill
     * AND as the terrain carve's winding test, with every arm still trimmed back
     * to make room for a junction that is not there.
     *
     * Consumers must treat it exactly as they treat `self_intersecting`:
     * triangulate_junction() falls back to the CONVEX HULL, build_curb_ring()
     * refuses the ring, and the terrain carve falls back to the CarveDisc path.
     * Use needs_hull_fallback() rather than testing either flag alone.
     *
     * Tested strictly negative: a ring of exactly zero area is a junction whose
     * arms all collapsed onto the node, which is degenerate but not mis-wound.
     * Always false when the ring is empty.
     */
    bool inverted = false;

    /**
     * @brief True when the ring may not be used as a simple CCW outline
     *
     * The one predicate every consumer of a JunctionPolygon should ask. A ring
     * that crosses itself has no meaningful interior; a ring wound clockwise
     * bounds the complement of what it appears to. Neither can be filled by
     * earcut, offset outward, or used as a winding test.
     */
    [[nodiscard]] bool needs_hull_fallback() const { return self_intersecting || inverted; }
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
 * the gap the corner spans exceeds 180 degrees, and the ground it spans is the
 * BACK of the node. It is closed through its corner point C, unrounded, which
 * wraps the boundary around the back of the node; see
 * FilletConfig::reflex_reach_factor, which bounds how far behind the node C may
 * lie before the chord is taken instead. No arc is ever drawn there: an arc
 * through a reflex corner bulges INTO the junction rather than out of it.
 *
 * The chord is what a corner gets when the two offset lines are parallel
 * (an arm's through-continuation on the far side of a T), when the intersection
 * lands past either cut face because a trim was clamped short, when the turn is
 * shallower than min_arc_angle, when the radius that actually fits has fallen
 * below min_radius, or when a reflex corner's C is out of reach.
 *
 * ### When two arms overlap: the cut faces are clipped
 *
 * A trim reduced by TrimConfig::max_trim_fraction, or bounded by
 * TrimConfig::min_pair_angle, leaves the two arms of that corner still
 * overlapping, and their cut faces then CROSS. Chaining them as they are gives a
 * bowtie -- the commonest self-intersecting junction in a real extract. Where two
 * adjacent cut faces cross, both are cut back to the crossing point, so the two
 * arms share one ring vertex there and the corner between them is a point rather
 * than a chord or an arc. The ring stays simple and still bounds exactly the
 * ground the two mouths cover; the ribbons still overlap it, which is what
 * over-trimmed means and no polygon can undo.
 *
 * Each arm keeps its two ring vertices, so arm_ring_start is unaffected. An arm
 * clipped at both ends far enough to invert collapses to a single point repeated
 * twice.
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
