/**
 * @file junction_curb.hpp
 * @brief The sidewalk and curb ring around a junction
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This is step 5 of the P4 junction solver in docs/plans/road_network_plan.md.
 *
 * junction_polygon.hpp produces the carriageway footprint. Every ribbon feeding
 * that footprint carries its own curb and sidewalk right up to the cut, and then
 * they stop dead in mid-air. This file wraps the junction in the ring that joins
 * them: an outward offset of the carriageway footprint, with a curb face, a curb
 * top, and a sidewalk surface, rounded at every corner.
 *
 * ### The gaps are the hard part
 *
 * A ring that closes all the way round walls the junction in with a curb across
 * every approach. Where an arm meets the junction the ring must OPEN, because
 * the arm's own sidewalk and curb continue there, and the two must butt against
 * each other rather than cross. That is why build_curb_ring() takes the arms and
 * their cut cross-sections: the arm spans tell it where to stop and start.
 *
 * The result is N open ring SECTIONS for an N-arm junction, one per corner,
 * each running from one arm's cut line round the fillet to the next arm's cut
 * line. See the Gaps section of build_curb_ring() for why the bound is the cut
 * line and not the arm's outer profile corners.
 *
 * ### Clipper2
 *
 * This is the ONLY file in the road pipeline that uses Clipper2, and the .cpp is
 * the only translation unit that includes <clipper2/clipper.h>; this header
 * deliberately does not, so nothing else in the tree picks up the dependency
 * transitively. Clipper2 is integer-based: local metres are multiplied by
 * CurbRingConfig::clipper_scale on the way in and divided on the way out. At the
 * default 1000 that is millimetre precision, which is far finer than any road
 * feature and far coarser than the int64 range can overflow at city scale.
 *
 * The offset is a round-joined outward inflate. It is used rather than a
 * hand-rolled miter offset because the input ring already contains fillet arcs
 * and near-tangent vertices, and a naive offset of such a ring self-intersects
 * at every tight corner. Clipper2 removes those self-intersections as part of
 * the operation, which is exactly the "curb offsets and cleanup" role the plan's
 * design table reserves for it.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API. renderer/mesh.hpp is included for Mesh and MaterialId only.
 */

#pragma once

#include "osm/road/crossings.hpp"     // DroppedKerbSpan
#include "osm/road/junction_polygon.hpp"
#include "osm/road/junction_trim.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Dimensions of the ring. All distances are metres.
 */
struct CurbRingConfig {
    /**
     * @brief Flat band at carriageway level between the junction ring and the curb
     *
     * The junction polygon's boundary is the LANE edge -- ArmEnd::carriage_left
     * and carriage_right sit at the carriageway envelope and nowhere else -- but
     * no tag-derived profile puts its curb face there. build_profile() lays a
     * Gutter strip between the outer lane and the curb for every class that has a
     * curb at all, so the arms' curb faces stand `gutter_width` OUTBOARD of the
     * ring's own boundary. Without this band the two curb lines miss each other
     * by that distance and every arm mouth shows an open notch in the kerb.
     *
     * Matches ProfileConfig::gutter_width, and is emitted at the carriageway
     * surface in MaterialId::Concrete, exactly as the gutter strip is. A profile
     * that also carries a parking or cycle lane inboard of its curb pushes its
     * curb further out still; that residual is a known approximation and shows as
     * a lateral step rather than as a hole.
     */
    double apron_width = 0.3;

    /**
     * @brief Width of the sidewalk SURFACE, from curb top to outer edge
     *
     * Matches ProfileConfig::sidewalk_width. The ring's whole reach is
     * ring_offset(): the apron, the curb face, the curb top and then this, laid
     * out in that order from the junction polygon outward, which is the same
     * order and the same widths build_profile() lays out on every arm.
     */
    double sidewalk_width = 2.0;

    /// Height of the curb top above the carriageway surface
    double curb_height = 0.15;

    /**
     * @brief Width of the flat top of the curb, between its face and the sidewalk
     *
     * Should match ProfileConfig::curb_top_width, or the ring's curb reads as a
     * different curb from the one arriving along every arm.
     *
     * It is one of the four bands ring_offset() adds up, alongside apron_width,
     * curb_face_batter and sidewalk_width, rather than being consumed out of any
     * of them. A band of zero width emits nothing.
     */
    double curb_top_width = 0.15;

    /**
     * @brief Outward lean of the curb face, metres over its full height
     *
     * Matches ProfileConfig::curb_face_batter. A face with zero batter is
     * perfectly vertical and still emits its quad.
     */
    double curb_face_batter = 0.02;

    /**
     * @brief Local metres are multiplied by this before clipping
     *
     * Clipper2 works in int64. 1000 gives millimetre precision, which is finer
     * than any road feature and cannot overflow at city extent.
     */
    double clipper_scale = 1000.0;

    /**
     * @brief Maximum chord deviation of a rounded corner of the offset, metres
     *
     * Handed to Clipper2 as its arc tolerance. Left at its own default the
     * offsetter derives a tolerance from the delta in SCALED units, which at
     * clipper_scale 1000 works out below a millimetre and tessellates every
     * corner of the ring into tens of vertices that no road ever needed. 20 mm is
     * finer than a curb stone and keeps a junction ring at a few dozen vertices.
     */
    double arc_tolerance = 0.02;

    /// Master switch. When false build_curb_ring() returns an invalid, empty ring.
    bool enabled = true;

    /**
     * @brief Total distance the ring reaches outward from the junction polygon
     *
     * The sum of the four lateral bands, and the distance `inner` is inflated by
     * to produce `outer`. Equal to an arm's own reach from its carriageway edge to
     * its outer sidewalk edge whenever these four values match ProfileConfig's,
     * which is what makes the ring and the ribbons one continuous kerb.
     */
    [[nodiscard]] double ring_offset() const {
        return std::max(0.0, apron_width) + std::max(0.0, curb_face_batter) +
               std::max(0.0, curb_top_width) + std::max(0.0, sidewalk_width);
    }
};

// ============================================================================
// Dropped kerbs
// ============================================================================

/**
 * @brief The runs of one junction's kerb that must drop to a lip
 *
 * A pedestrian crossing needs a break in the kerb line at each of its ends, and
 * so does a driveway where it meets its parent road. crossings.hpp locates both
 * and publishes them as DroppedKerbSpan, which is a pair of DIRECTIONS from the
 * junction centre rather than a pair of points, precisely so it survives being
 * handed to a ring whose vertices it was not computed against.
 *
 * This is that hand-off, and it is deliberately a MODULATION of the ring's
 * existing cross-section rather than a second kind of ring:
 *
 * - The curb face top and the curb top drop from `curb_height` to
 *   DroppedKerbSpan::height across a span. The face keeps its batter and still
 *   emits its quad; it is simply 20 mm tall instead of 150 mm.
 * - The curb face BOTTOM and the apron do not move. They are at the carriageway
 *   surface either way.
 * - The sidewalk's OUTER edge does not move either. It stays level with the
 *   sidewalks of the arms either side and with whatever the ring abuts, so the
 *   sidewalk band becomes the crossfall ramp a real dropped kerb has, and the
 *   drop can never tear the ring away from its neighbours.
 *
 * The kerb ramps down over CrossingConfig::dropped_kerb_ramp at each end of a
 * span rather than stepping, and the ring is resampled across the ramp so the
 * slope is carried by real vertex columns. See build_curb_ring().
 *
 * Spans from dropped_kerb_spans() and driveway_kerb_spans() may simply be
 * concatenated into @ref spans: where two overlap the deeper drop wins, so they
 * need no merging across the two sources.
 */
struct KerbDrops {
    /**
     * @brief Junction centre every span's directions are measured from
     *
     * Must be the SAME point that was passed to dropped_kerb_spans() as
     * `junction_center`, that is, Junction::center. Passing a different one --
     * the polygon centroid, say -- rotates every span around the ring.
     */
    glm::dvec2 center{0.0};

    /// Runs to drop, in any order; overlaps are allowed
    std::vector<DroppedKerbSpan> spans;

    /**
     * @brief Distance the kerb ramps between full height and the lip, metres
     *
     * CrossingConfig::dropped_kerb_ramp. Measured along the kerb line, outward
     * from each end of a span, so a span is the FLAT part of the drop and the
     * ramps sit outside it.
     */
    double ramp_length = 1.0;

    /// There is at least one span to apply
    [[nodiscard]] bool any() const { return !spans.empty(); }
};

// ============================================================================
// Output
// ============================================================================

/**
 * @brief The sidewalk and curb wrapped around one junction
 */
struct CurbRing {
    /**
     * @brief The junction polygon ring, verbatim
     *
     * Counter-clockwise, first point not repeated. Copied rather than referenced
     * so the ring is self-contained once built, and so the two boundaries the
     * curb face spans are held side by side.
     */
    std::vector<glm::dvec2> inner;

    /**
     * @brief `inner` offset outward by ring_offset(), with rounded corners
     *
     * CLOSED and counter-clockwise, first point not repeated, even though the
     * MESH is open at every arm. Two reasons the ring stays closed here: it is
     * the junction's terrain-carve footprint (Junction::footprint), and a carve
     * footprint must be a simple closed polygon; and the arm gaps are a property
     * of what is DRAWN, not of what the junction occupies -- the ground under an
     * arm mouth is still junction.
     *
     * Vertex counts do not correspond between `inner` and `outer`: the round
     * offset inserts vertices at convex corners and removes them where the offset
     * self-intersects, so the two rings must be matched by position, not by index.
     * Empty when the offset produced nothing usable.
     */
    std::vector<glm::dvec2> outer;

    /**
     * @brief Curb face, curb top, and sidewalk surface, materials tagged
     *
     * Emitted as N open sections for an N-arm junction, with nothing across an
     * arm mouth. Four surfaces per section, in MaterialId order after
     * Mesh::sort_submeshes_by_material():
     *
     * - apron, flat at the carriageway surface from the junction polygon out to
     *   the curb face -- MaterialId::Concrete
     * - curb face, from the carriageway surface up to the curb top, leaning
     *   outward by curb_face_batter -- MaterialId::Curb
     * - curb top, a narrow horizontal band at the top of the face -- MaterialId::Curb
     * - sidewalk surface, from the curb top out to `outer` -- MaterialId::Sidewalk
     */
    Mesh mesh;

    /// The ring is usable: `inner` and `outer` both non-empty and the mesh built
    bool valid = false;
};

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Offset the junction polygon outward and build the ring geometry
 *
 * ### Offset
 *
 * `poly.ring` is scaled by CurbRingConfig::clipper_scale, inflated by
 * `ring_offset() * clipper_scale` with `Clipper2Lib::JoinType::Round` and
 * `EndType::Polygon`, and scaled back. Where the inflate returns several paths --
 * possible when a very concave junction ring pinches -- the one of greatest
 * absolute area is taken and the rest discarded, and the result is re-oriented
 * counter-clockwise if the offset returned it the other way round.
 *
 * ### Gaps
 *
 * Arm k's mouth is bounded by that arm's CUT LINE: the line through
 * ArmEnd::center along its cut cross-section, that is, through
 * ArmEnd::carriage_right and ArmEnd::carriage_left. That is the miter bisector at
 * the trim station and is only perpendicular to ArmEnd::direction on a straight
 * approach, so the line is taken from the two corners rather than from the
 * direction. Ring section k is the run of
 * the ring from arm k's cut line, counter-clockwise around the fillet, to arm
 * k+1's cut line -- which is exactly the span between arm k's `carriage_left` and
 * arm k+1's `carriage_right`, so no arm's cut face is ever built over and the
 * junction can never be walled in.
 *
 * The ring turns a convex corner at each of those two vertices, so the offset
 * sweeps a ROUND JOIN through the turn, fanning outward from the vertex. The fan
 * reaches back up the approach -- over the arm's gutter, curb and sidewalk, never
 * over its carriageway -- but it does so at the ring's SIDEWALK height, so it
 * hangs above the arm's gutter and z-fights the arm's own sidewalk. The part of
 * the fan lying beyond the cut line is therefore dropped, and where a corner
 * turns far enough for the join to actually cross the cut line the crossing is
 * INTERPOLATED, so the ring ends flush against the arm's cross-section rather
 * than snapped to the nearer offset vertex.
 *
 * Dropping the fan outright, which is what a square crossroads does -- its corner
 * turns only about 45 degrees, so no part of the join reaches the cut line --
 * would start the ring's curb at the first fillet vertex instead of at the arm's
 * carriageway corner, leaving a notch in the curb line one fillet segment wide.
 * So the ring VERTEX is put back as the section's terminal cross-section, at the
 * outward direction of the sample beside it. The ring's curb therefore begins and
 * ends EXACTLY on ArmEnd::carriage_left and ArmEnd::carriage_right, which is
 * where the arm's own carriageway edge stops, and the curbs meet end to end.
 *
 * The mouth is NOT bounded by the arm's outer profile corners ArmEnd::left and
 * ArmEnd::right, which an earlier draft of this contract specified. It cannot be.
 * The carriageway is trimmed back only as far as the point where two carriageways
 * stop overlapping -- 3.75 m from the node for two 7 m roads meeting square --
 * while the arms' own sidewalks reach 5.95 m either side of their centrelines and
 * run right up to that cut. Every corner of the ring therefore lies inside both
 * neighbouring arms' full-profile corridors, and a mouth measured at the profile
 * width swallows every corner of every junction and emits nothing whatsoever.
 * That was measured on a square crossroads, a T, a Y, an acute fork and a
 * five-way, not reasoned about. The cost of the cut-line bound is that the ring
 * overlaps the far ends of its arms' sidewalks by a little at each corner, which
 * is the smaller and far less visible error of the two.
 *
 * ArmEnd::left and ArmEnd::right are therefore not read. ArmEnd::center,
 * ArmEnd::direction and JunctionPolygon::arm_ring_start are what locate a mouth.
 *
 * Sections whose two ends collapse onto each other, which happens when two arms
 * leave at an acute angle and their fillet degenerates, emit no geometry rather
 * than a degenerate sliver.
 *
 * ### Heights, world mapping and winding
 *
 * @code
 *     (x, y_2d) -> glm::vec3(x, height, -y_2d)
 * @endcode
 *
 * ### Dropped kerbs
 *
 * When @p drops is supplied, the curb face top and the curb top are lowered
 * towards DroppedKerbSpan::height across each span, ramped over
 * KerbDrops::ramp_length at each end. The ring is RESAMPLED along any section a
 * span or its ramps reach, so the ramp is carried by real vertex columns rather
 * than falling between two of them and reading as a step. Every band of the
 * cross-section takes its heights from one per-column factor, so the four
 * surfaces stay welded at their shared boundaries however deep the drop goes.
 *
 * The curb face runs from @p height at `inner` up to `height + curb_height` at
 * `inner` offset outward by curb_face_batter. The curb top and the sidewalk
 * surface both sit at `height + curb_height`, so the ring's sidewalk is level
 * with the sidewalks of its arms, which sit at the same offset above their own
 * carriageway. Every horizontal surface gets a +Y normal; the curb face gets a
 * horizontal normal pointing INWARD, towards the carriageway, matching the
 * corridor extruder's convention for the curb of a raised sidewalk.
 *
 * ### UVs
 *
 * The ring follows the CORRIDOR convention, not the planar projection the
 * junction FILL uses. The fill is planar-projected because it has no direction of
 * travel; the ring does have one -- it runs ALONG the ring -- and it has to meet
 * an arm's curb and sidewalk at every mouth without the paving rotating across
 * the join. So, exactly as build_corridor() does, with `along` the distance
 * walked along the section's inner boundary and `lateral` measured outward from
 * each surface's own inboard edge, both in metres:
 *
 * @code
 *     U = lateral / uv_tiling(material).u_metres
 *     V = along   / uv_tiling(material).v_metres
 * @endcode
 *
 * The curb FACE is the exception the plan's UV Convention freezes: its U runs UP
 * the face, `height_up_face / 0.5`, and its V is `along / 2.0`. That is what
 * keeps the ring's curb continuous with the curb faces of its arms, and it is
 * also why the curb TOP cannot be planar-projected -- top and face share one
 * MaterialId::Curb texture, authored with the face on one side and the top on the
 * other, and a planar projection would sweep the top straight across the face's
 * half of it.
 *
 * As in the corridor, U is CONSTANT per vertex column and comes from the NOMINAL
 * width of the surface -- apron_width across the apron, curb_height up the face,
 * curb_top_width across the top, sidewalk_width across the walk -- so the
 * texture does not shear where the offset stretches around a corner.
 *
 * @param poly   Junction footprint; its ring becomes CurbRing::inner, and its
 *               arm_ring_start locates every mouth in that ring
 * @param arms   Arms in ascending bearing order, parallel to @p ends
 * @param ends   Cut cross-sections; `center` and `direction` supply each mouth's
 *               cut line
 * @param height World Y of the CARRIAGEWAY surface at the junction, the same
 *               value passed to triangulate_junction()
 * @param cfg    Ring dimensions; the defaults are the shipping values
 * @param drops  Kerb drops to apply, or nullptr for a ring at full height
 *               throughout. See KerbDrops for what a drop changes and what it
 *               deliberately leaves alone.
 * @return The ring. `valid` false, and everything empty, when @p cfg is disabled,
 *         when @p poly is invalid or self-intersecting, or when the offset
 *         produced no usable path. A ring whose offset succeeded but every one of
 *         whose sections collapsed keeps `inner` and `outer` -- they are still the
 *         correct footprint -- with `valid` false and an empty mesh, so a caller
 *         wanting only the carve footprint may use `outer` whenever it is
 *         non-empty rather than gating on `valid`.
 */
[[nodiscard]] CurbRing build_curb_ring(const JunctionPolygon& poly,
                                       const std::vector<ArmRef>& arms,
                                       const std::vector<ArmEnd>& ends,
                                       float height,
                                       const CurbRingConfig& cfg,
                                       const KerbDrops* drops = nullptr);

} // namespace stratum::osm::road
