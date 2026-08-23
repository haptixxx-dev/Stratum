/**
 * @file junction_special.hpp
 * @brief Nodes that are not an ordinary N-way intersection
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The trim-and-fillet solver in junction_trim.hpp and junction_polygon.hpp
 * handles a node of degree 3 or more where every arm is a plain road. This file
 * covers the three cases from the plan's "Special cases" list that it cannot:
 *
 * - **Roundabouts.** `junction=roundabout` closed cycles. A roundabout is not a
 *   node at all -- it is a RING OF EDGES, and each of its approach nodes has
 *   degree 3 while the intersection they collectively form has none. Solving
 *   those nodes individually gives three unrelated T-junctions around a hole.
 * - **Degree-2 profile transitions.** Two edges meeting with different profiles.
 *   No trim is needed, but a hard discontinuity from four lanes to two, or from
 *   a sidewalk to none, reads as a broken mesh. It gets a taper instead.
 * - **Dead ends.** Degree-1 nodes. A ribbon that simply stops shows its open
 *   cross-section end-on. It gets a turning circle, a cul-de-sac bulb, or a flat
 *   cap.
 *
 * Motorway link gores -- `*_link` arms merging with a painted nose rather than a
 * square junction -- are named in the plan but are NOT implemented here. A link
 * arm currently goes through the ordinary trim-and-fillet path, which produces a
 * square mouth where a gore belongs. That is a known quality gap, not an
 * oversight, and the fillet mouth is at least geometrically sound.
 *
 * ### Coordinates
 *
 * 2D local metres throughout, with the usual Y-up render mapping
 * `(x, y_2d) -> vec3(x, height, -y_2d)` applied by the functions that emit
 * triangles, and by nothing else.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API. renderer/mesh.hpp is included for Mesh and MaterialId only.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Roundabout dimensions. All distances are metres.
 */
struct RoundaboutConfig {
    /**
     * @brief Radius floor for a detected loop
     *
     * A `junction=roundabout` cycle smaller than this is a mini-roundabout: a
     * painted dome, not an annulus. It is rejected as a roundabout and falls back
     * to the ordinary trim-and-fillet path, which is the right shape for one.
     */
    double min_radius = 4.0;

    /**
     * @brief Carriageway inner edge to island edge
     *
     * The island is inset this far inside the carriageway's inner boundary, so
     * the two do not share an edge and the island's own curb face has somewhere
     * to stand.
     */
    double island_inset = 1.0;

    /// Vertices around the full ring, for both the carriageway and the island
    int segments = 48;

    /// Island is raised behind a curb face rather than painted flat
    bool raised_island = true;
};

/**
 * @brief Profile taper dimensions
 */
struct TaperConfig {
    /**
     * @brief Taper length per metre of width change
     *
     * A 3.5 m lane drop at the default 15 gives a 52.5 m taper, which is the
     * right order for an urban merge. Applied to the LARGEST single-sided width
     * change, not to the total, so a symmetric widening does not taper twice as
     * long as a one-sided one.
     */
    double length_per_metre_width = 15.0;

    /// Floor, so a trivial width change still gets a visible blend
    double min_length = 4.0;

    /// Ceiling, so a motorway-to-service transition does not taper for a kilometre
    double max_length = 60.0;
};

/**
 * @brief Dead-end cap dimensions
 */
struct DeadEndConfig {
    /// Radius of the disc emitted at a `highway=turning_circle` node
    double turning_circle_radius = 6.0;

    /**
     * @brief Residential cul-de-sacs get a bulb even without a turning_circle tag
     *
     * OSM tags turning circles inconsistently, and a residential street that
     * simply stops is far more often a cul-de-sac than a truncated import. When
     * false such a node gets a flat cap instead.
     */
    bool bulb_for_residential = true;
};

/**
 * @brief Every special-case tunable, in one place
 */
struct SpecialConfig {
    RoundaboutConfig roundabout;
    TaperConfig      taper;
    DeadEndConfig    dead_end;
};

// ============================================================================
// Roundabouts
// ============================================================================

/**
 * @brief A closed cycle of edges flagged is_roundabout
 *
 * The cycle is stored as the edges in traversal order together with the nodes
 * between them, so `edges[i]` runs from `nodes[i]` to `nodes[i + 1]`, with the
 * last edge closing back onto `nodes[0]`. Both vectors are therefore the same
 * length.
 *
 * Traversal direction follows the edges' own `from -> to` orientation, which OSM
 * draws in the direction of travel: clockwise in left-hand-drive countries and
 * counter-clockwise in right-hand-drive ones. Nothing here depends on which, but
 * a consumer emitting yield lines or splitter islands does.
 */
struct RoundaboutLoop {
    /// Edges in cycle order
    std::vector<EdgeId> edges;

    /**
     * @brief Nodes on the loop, in the same order
     *
     * `nodes[i]` is the start node of `edges[i]`. A node of degree 3 or more is an
     * APPROACH node, where a road outside the loop joins it; a node of degree 2 is
     * merely a point where the OSM way was split and carries no approach.
     */
    std::vector<GraphNodeId> nodes;

    /**
     * @brief Centre of a least-squares circle fitted to the loop's stations
     *
     * Fitted, rather than averaged from the loop's NODE positions, because
     * nodes sit wherever a mapper happened to split the way -- typically all on
     * one side of the ring, at its approaches -- and their mean is dragged
     * towards that side. On a 15 m ring whose three approaches fall within a
     * quarter turn of each other the node average lands 14 m off centre while
     * the fit stays within 5 mm.
     *
     * Samples are the centerline STATIONS of every edge in the cycle, weighted
     * by their own arclength share so a curvature-adaptive resampling cannot
     * bias the fit towards the tighter end of an oval. Falls back to the
     * weighted centroid, and to the mean distance, when the sample is too
     * nearly collinear to fit.
     */
    glm::dvec2 center{0.0};

    /// Radius of that fitted circle, metres
    double radius = 0.0;

    /**
     * @brief The loop is a usable roundabout
     *
     * False when the loop carries no edges, or when its fitted radius is below
     * RoundaboutConfig::min_radius. An invalid loop must be left to the ordinary
     * trim-and-fillet path rather than dropped.
     *
     * A cycle that does not close produces no RoundaboutLoop at all rather than
     * an invalid one -- there is nothing to carry -- and a cycle of a SINGLE
     * edge is valid: a ring way that closes on itself with no approach splitting
     * it is a real roundabout, and rejecting it would drop the whole ring.
     *
     * find_roundabouts() takes no RoundaboutConfig, so it applies the DEFAULT
     * min_radius here. build_roundabout() re-checks against the config actually
     * passed to it and yields an empty Mesh for a loop below that, so a caller
     * raising min_radius still gets the mini-roundabout fallback it asked for.
     */
    bool valid = false;
};

/**
 * @brief Find every roundabout cycle in the graph
 *
 * Walks the subgraph of edges with GraphEdge::is_roundabout set and extracts its
 * closed cycles. Each edge belongs to at most one returned loop, so a figure of
 * eight -- two roundabouts sharing a node, which occurs at grade-separated
 * interchanges -- comes back as two loops rather than one.
 *
 * A roundabout way that OSM leaves open, which happens where an extract is cut
 * through one, produces no loop and its edges fall back to the ordinary path.
 *
 * Results are ordered by the lowest EdgeId in each loop, and each loop starts at
 * its own lowest EdgeId, so the output is reproducible run to run regardless of
 * traversal order.
 *
 * @param graph       Built road graph
 * @param centerlines Parallel to graph.edges(); supplies the stations that
 *                    `center` and `radius` are measured from
 * @return Every detected loop, valid and invalid alike; check RoundaboutLoop::valid
 */
[[nodiscard]] std::vector<RoundaboutLoop> find_roundabouts(const RoadGraph& graph,
                                                           const std::vector<Centerline>& centerlines);

/**
 * @brief Build the annulus geometry of one roundabout
 *
 * Emits, at @p height:
 *
 * - the carriageway annulus, from the outer edge of the loop's profile inward to
 *   its inner edge -- MaterialId::Asphalt, or the loop's own surface material
 * - the central island, inset by RoundaboutConfig::island_inset inside the
 *   carriageway's inner edge -- MaterialId::Grass, behind a
 *   MaterialId::Curb face when RoundaboutConfig::raised_island is set
 *
 * The annulus is swept from the loop's own centerlines rather than from a perfect
 * circle of `radius`. Real roundabouts are ovals far more often than circles, and
 * a circle fitted to an oval leaves the approach mouths hanging off the ring.
 * `center` and `radius` are for placement decisions and the terrain carve, not
 * for generating the ring.
 *
 * Approach mouths are not SOLVED here -- the nodes where a road joins the loop
 * are ordinary degree-3 nodes and go through the trim-and-fillet path, so the
 * mouth is a fillet against the annulus like any other junction. The annulus
 * does, however, stop where that solve has already cut a ring edge back: it is
 * swept over `slice(cl, trim_from, length - trim_to)` for each loop edge, so
 * nothing is built twice at an approach and the mouth's own fillet fills the
 * gap. With the trims still zero the ring closes all the way round, which is
 * the correct answer before they are solved.
 *
 * The CENTRAL ISLAND ignores trims and is always one closed surface built from
 * the untrimmed ring. An approach cuts the carriageway, never the island.
 *
 * Splitter islands and yield lines on those approaches are P5's markings work
 * and are not emitted.
 *
 * The world mapping, winding and normals are the same as triangulate_junction():
 * `(x, y_2d) -> vec3(x, height, -y_2d)`, counter-clockwise front faces, and +Y
 * normals on horizontal surfaces.
 *
 * UVs split, because the two surfaces differ in whether they have a direction of
 * travel. The ANNULUS is a swept ribbon and keeps the ribbon convention -- U
 * across the carriageway, V the arclength travelled round the ring -- so its
 * texture runs continuously into the roads feeding it. The ISLAND has no
 * direction of travel and is planar-projected in the loop's own frame about
 * RoundaboutLoop::center, exactly as triangulate_junction() projects a junction
 * fill. The island's curb FACE keeps the frozen curb convention: U up the face,
 * V round the ring.
 *
 * RoundaboutConfig carries no curb dimensions, so a raised island's curb uses
 * the CurbRingConfig defaults, 0.15 m high with a 0.02 m batter. An island curb
 * that disagreed with the junction curb ring would read as a different curb on
 * the same intersection.
 *
 * @param graph       Built road graph
 * @param centerlines Parallel to graph.edges()
 * @param profiles    Parallel to graph.edges()
 * @param loop        A loop from find_roundabouts(); an invalid one yields an empty Mesh
 * @param height      World Y of the roundabout carriageway surface, node height
 *                    plus ElevationConfig::surface_offset
 * @param cfg         Roundabout dimensions
 * @return The annulus and island, with one SubMesh range per material
 */
[[nodiscard]] Mesh build_roundabout(const RoundaboutLoop& loop,
                                    const RoadGraph& graph,
                                    const std::vector<Centerline>& centerlines,
                                    const std::vector<RoadProfile>& profiles,
                                    float height,
                                    const RoundaboutConfig& cfg);

// ============================================================================
// Degree-2 profile transitions
// ============================================================================

/**
 * @brief Blend between two different profiles meeting at a degree-2 node
 *
 * A degree-2 node needs no trim in the junction sense -- there is no second road
 * to overlap -- but when its two edges carry different cross-sections the ribbon
 * jumps width, and often material, at a single station. The taper replaces that
 * jump with a wedge over a length derived from the width change.
 *
 * The taper length is
 * `clamp(max_side_width_change * length_per_metre_width, min_length, max_length)`,
 * and is then capped at TrimConfig-independent halves of the two edges' own
 * lengths so a taper never consumes a whole short edge. Half of the length is
 * taken from each side, which is what @p out_trim_a and @p out_trim_b report.
 *
 * The two arms are taken in the node's own ascending-bearing order: arm 0 is
 * "a", arm 1 is "b". @p out_trim_a and @p out_trim_b are arclengths cut from
 * THAT end of each arm's edge, on exactly the same contract as ArmRef::trim, so
 * they are written onto GraphEdge::trim_from or trim_to according to the arm's
 * at_start flag.
 *
 * ### Strip matching
 *
 * Both profiles are first split at lateral zero -- the axis
 * RoadProfile::left_edge_offset() centres the carriageway envelope on -- into a
 * left and a right half-section, each ordered OUTWARD from that axis. A strip
 * lying across zero, which an odd lane count produces, is cut into two
 * half-strips of its own kind.
 *
 * Each side is then matched INDEPENDENTLY, on a longest common subsequence of
 * StripKind. Every LCS member is a matched pair that interpolates width, height
 * and material; every strip outside it keeps its own end's width and takes ZERO
 * width at the other, so it grows out of nothing or shrinks into nothing. A
 * strip is therefore only ever paired with a strip of the SAME kind: a cycle
 * lane on one edge and not the other opens from zero rather than a lane being
 * dragged sideways into a sidewalk. Ties in the reconstruction advance side A
 * first, so the alignment is a pure function of the two inputs.
 *
 * Matching runs outward from the centre rather than inward from the outer edge
 * because the outer edge is the end that MOVES in a taper; anchoring there would
 * slide every lane sideways. Heights interpolate with widths, so a curb rises
 * out of the surface rather than stepping up, and an unmatched strip inherits
 * the height of the boundary it collapses onto.
 *
 * @note The taper is emitted FLAT at @p height. Only the node height is supplied
 *       and there is no per-station elevation here, so on a steep grade the
 *       taper departs from the arms it joins by up to `grade * length / 2`. The
 *       ribbons themselves still follow the terrain.
 *
 * @param graph       Built road graph
 * @param centerlines Parallel to graph.edges()
 * @param profiles    Parallel to graph.edges()
 * @param node        The degree-2 node; any other degree returns false
 * @param height      World Y of the carriageway surface at the node, node height
 *                    plus ElevationConfig::surface_offset
 * @param cfg         Taper dimensions
 * @param out_mesh    Receives the taper geometry; untouched when false is returned
 * @param out_trim_a  Receives the arclength to cut from arm 0's end
 * @param out_trim_b  Receives the arclength to cut from arm 1's end
 * @param limit_a     Ceiling on out_trim_a, metres. The wedge is built over the
 *                    trims this function RETURNS, so a caller whose joint budget
 *                    reduced an earlier demand must rebuild with the reduced
 *                    values here rather than keep the mesh: otherwise the ribbon
 *                    is sliced short of the wedge and lies coplanar on top of it.
 * @param limit_b     Ceiling on out_trim_b, metres
 * @param out_outline Optional. Receives the taper's closed outer boundary in 2D
 *                    local metres, counter-clockwise with the first point not
 *                    repeated, on the same contract as Corridor::outline. This is
 *                    the taper's terrain-carve footprint: the two ribbons stop at
 *                    their trims, so without it the ground under the wedge is
 *                    never flattened.
 * @return false when @p node is not degree 2, when either edge is unusable, or
 *         when the two profiles are close enough that no taper is needed -- equal
 *         strip kinds with every width within 1e-3 m. Both out_trim values are
 *         then set to 0.0 and the edges are left untrimmed.
 */
bool build_profile_taper(const RoadGraph& graph,
                         const std::vector<Centerline>& centerlines,
                         const std::vector<RoadProfile>& profiles,
                         GraphNodeId node,
                         float height,
                         const TaperConfig& cfg,
                         Mesh& out_mesh,
                         double& out_trim_a,
                         double& out_trim_b,
                         double limit_a = 1e30,
                         double limit_b = 1e30,
                         std::vector<glm::dvec2>* out_outline = nullptr);

// ============================================================================
// Dead ends
// ============================================================================

/**
 * @brief Cap a degree-1 node
 *
 * Three shapes, chosen in this order:
 *
 * 1. **Turning circle.** GraphNode::is_turning_circle, from
 *    `highway=turning_circle` or `highway=turning_loop`. A disc of
 *    DeadEndConfig::turning_circle_radius centred on the node, or of the arm's
 *    own half profile width where that is larger, so the disc never sits inside
 *    the road it terminates.
 *
 *    The disc straddles the node, so its two backward lobes would otherwise lie
 *    coplanar on top of the ribbon they cap and z-fight with it. They are
 *    notched out along the ribbon's REAL offset edges, out to where those edges
 *    cross the circle. The notch is taken at the outermost strip boundary still
 *    lying ON the carriageway plane -- the gutter's outer edge, not the lane's --
 *    because a gutter is emitted coplanar with the lane and a notch at the lane
 *    edge would leave the lobes lying on top of it. What is left is a horseshoe,
 *    not a disc, and it is ear-clipped rather than fanned for that reason.
 *
 *    Everything standing ABOVE the carriageway plane is closed down to it by the
 *    same vertical end face the flat cap emits, because the disc is flat and a
 *    raised sidewalk arriving at the node would otherwise end in mid-air.
 * 2. **Cul-de-sac bulb.** DeadEndConfig::bulb_for_residential with an arm of
 *    RoadType::Residential or RoadType::Service. A disc as above, offset FORWARD
 *    along the arm by exactly `sqrt(R^2 - w^2)`, which is the offset that puts
 *    the arm's two carriageway corners ON the circle: the bulb then closes on
 *    the arm's own end cross-section and sits beyond the last surveyed point
 *    rather than swallowing the last few metres of the street.
 *
 *    Everything outboard of the carriageway -- gutter, curb face, curb top,
 *    sidewalk -- is carried round the bulb radially, with a gusset at each end
 *    of the arc closing the wedge between the ring's radial cross-section and
 *    the arm's lateral one. The ring is emitted only when the arm's two
 *    outboard half-sections MATCH, which `sidewalk=both` and every symmetric
 *    profile satisfy; a one-sided profile has no single cross-section to sweep
 *    round a circle and gets the carriageway bulb alone, plus the flat cap's
 *    vertical end face so its raised strips are still closed.
 * 3. **Flat cap.** Anything else, including every truncated edge at the boundary
 *    of an extract. A ribbon is a SHELL, so a profile lying flat on the
 *    carriageway plane has no open end to show and correctly emits nothing. What
 *    is open is every strip standing ABOVE that plane -- a raised sidewalk ends
 *    as a floating slab -- and the cap is the vertical end face closing each of
 *    those down to the carriageway, facing away from the road. That stops the
 *    ribbon showing its open end without inventing a road feature that is not
 *    there.
 *
 * No trim is emitted. A dead end EXTENDS the network rather than cutting it back,
 * so the arm's ribbon runs to its full length in every case and the cap is welded
 * onto its end.
 *
 * World mapping, winding, normals and planar UV projection are the same as
 * triangulate_junction(), projected about the node position.
 *
 * @param graph       Built road graph
 * @param centerlines Parallel to graph.edges()
 * @param profiles    Parallel to graph.edges()
 * @param node        The degree-1 node; any other degree yields an empty Mesh
 * @param height      World Y of the carriageway surface at the node, node height
 *                    plus ElevationConfig::surface_offset
 * @param cfg         Dead-end dimensions
 * @param out_outline Optional. Receives the cap's closed outer boundary in 2D
 *                    local metres, counter-clockwise with the first point not
 *                    repeated, on the same contract as Corridor::outline. A bulb
 *                    or turning circle reaches well beyond the ribbon's own end
 *                    cross-section, so this is what the terrain carve needs in
 *                    order to flatten the ground under it. Left empty for a flat
 *                    cap, which adds no footprint the ribbon does not already
 *                    cover.
 * @return The cap, with one SubMesh range per material. Empty when @p node is not
 *         a dead end or its arm is unusable.
 */
[[nodiscard]] Mesh build_dead_end(const RoadGraph& graph,
                                  const std::vector<Centerline>& centerlines,
                                  const std::vector<RoadProfile>& profiles,
                                  GraphNodeId node,
                                  float height,
                                  const DeadEndConfig& cfg,
                                  std::vector<glm::dvec2>* out_outline = nullptr);

} // namespace stratum::osm::road
