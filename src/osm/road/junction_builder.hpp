/**
 * @file junction_builder.hpp
 * @brief Orchestrates the whole P4 junction solve over a built road graph
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This is the entry point of P4 in docs/plans/road_network_plan.md, and the only
 * junction header RoadNetworkBuilder needs to know about. The four modules under
 * it each own one step and none of them knows about the others' outputs:
 *
 * | Header                | Owns                                              |
 * |-----------------------|---------------------------------------------------|
 * | junction_trim.hpp     | arm geometry, trim distances, cut cross-sections   |
 * | junction_polygon.hpp  | the carriageway footprint and its triangulation    |
 * | junction_curb.hpp     | the sidewalk ring around it (the only Clipper2 user)|
 * | junction_special.hpp  | roundabouts, profile tapers, dead ends             |
 *
 * ### What the solve actually changes
 *
 * Today every ribbon is extruded over the full length of its edge, so at a
 * junction they run straight through each other. build() does two things about
 * that, and the second is easy to miss:
 *
 * 1. It emits junction geometry -- fill, fillets, curb ring -- as a Junction per
 *    node.
 * 2. It WRITES `trim_from` and `trim_to` onto the graph's edges. Those fields
 *    have existed since P1 and have been zero ever since. This is what fills
 *    them, and it is why build() takes the graph by mutable reference.
 *
 * Corridor extrusion afterwards MUST use the trimmed centerline --
 * `slice(centerlines[e], edge.trim_from, centerlines[e].length() - edge.trim_to)`
 * -- or nothing changes: the junctions are drawn and the ribbons still overlap
 * them. Running build() and then extruding the untrimmed centerline is strictly
 * worse than not running it at all, because it adds a coplanar surface inside
 * every intersection.
 *
 * ### Ordering against terrain
 *
 * The plan carves terrain TWICE, and this is the second pass. P3 carved against
 * the P2 ribbon corridors plus a provisional CarveDisc per junction; P4 hands
 * back Junction::footprint, the real fillet-and-curb outline, so the junction
 * neighbourhoods can be re-carved. The provisional disc is a superset of the
 * final footprint in the common case, so pass 2 is a refinement rather than a
 * correction of visible error. See "Note on terrain ordering" in the plan.
 *
 * The elevation solve is an INPUT here, not an output: node heights are already
 * fixed and every arm already terminates at them, so a junction never has to
 * decide a height, only to read one.
 *
 * That last clause is what the trims break and what the caller has to repair: an
 * arm no longer ends AT its node, it ends `trim` metres away, where the solved
 * profile has moved on by `grade * trim`. A junction is one flat plane, so the
 * caller must flatten each edge's solved station heights over its own trim --
 * give every junction a plateau as wide as its cut -- or every arm mouth opens a
 * vertical step. RoadNetworkBuilder does that between the junction solve and the
 * corridor extrusion.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "osm/road/junction_curb.hpp"
#include "osm/road/junction_polygon.hpp"
#include "osm/road/junction_special.hpp"
#include "osm/road/junction_trim.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Every tunable of the junction solve, in one place
 */
struct JunctionConfig {
    TrimConfig     trim;      ///< How far back each arm is cut
    FilletConfig   fillet;    ///< Corner rounding of the junction polygon
    CurbRingConfig curb;      ///< Sidewalk ring dimensions
    SpecialConfig  special;   ///< Roundabouts, tapers, dead ends

    /**
     * @brief Emit curb rings around junctions
     *
     * When false the junction fill is still built and the arms are still trimmed;
     * only the ring is skipped, and Junction::footprint falls back to the
     * junction polygon ring itself. Distinct from CurbRingConfig::enabled, which
     * turns the ring off inside build_curb_ring(); either being false has the
     * same effect and both are honoured.
     */
    bool emit_curb_rings = true;

    /**
     * @brief Mirror of ElevationConfig::surface_offset, metres
     *
     * RoadElevationSolver::node_height() deliberately EXCLUDES this offset so
     * that each consumer adds it exactly once, and the junction solve is that one
     * consumer for junction geometry. Junction::height is
     * `node_height(node) + surface_offset`, which is exactly the plane the arms'
     * own end stations land on, because EdgeElevation::station_heights already
     * includes it.
     *
     * The solve cannot read ElevationConfig itself -- build() takes the SOLVED
     * elevation, not the configuration that produced it -- so the value is
     * mirrored here and RoadNetworkBuilder copies it across. A mismatch floats
     * every junction off its own arms by the difference.
     */
    float surface_offset = 0.05f;

    /**
     * @brief Mirror of CorridorConfig::base_height, metres
     *
     * The plane every junction is placed on when the elevation solve did not run,
     * which is the flat P2 network. Mirrored for the same reason as
     * surface_offset.
     */
    float base_height = 0.05f;
};

// ============================================================================
// Output
// ============================================================================

/**
 * @brief Which special case, if any, produced a Junction
 *
 * Carried explicitly rather than inferred from the node's degree, because the
 * fallbacks make degree an unreliable signal: a degree-3 node whose arms are too
 * wide for the space between them comes back as Degenerate, and a degree-3
 * approach node on a roundabout is an ordinary Intersection whose fillets happen
 * to meet an annulus.
 */
enum class JunctionKind : uint8_t {
    Intersection,   ///< Degree 3 or more: trimmed arms, fillet corners, curb ring
    Roundabout,     ///< A whole RoundaboutLoop, collapsed into one Junction
    Taper,          ///< Degree 2 with differing profiles
    DeadEnd,        ///< Degree 1: turning circle, cul-de-sac bulb, or flat cap
    Degenerate      ///< Solve failed; the provisional disc footprint is all there is
};

/**
 * @brief One solved junction: its geometry, its footprint, and how it was solved
 *
 * A Junction is a value type holding its own geometry, so the list survives the
 * graph being handed on and is safe to move into RoadPieces.
 */
struct Junction {
    /**
     * @brief The graph node this junction was solved for
     *
     * For JunctionKind::Roundabout this is the loop's FIRST node -- the one at
     * `RoundaboutLoop::nodes.front()` -- because a loop has no single node of its
     * own. Every other node on that loop still gets its own Junction for its
     * approach mouth, so the loop node appearing here does not mean it was
     * skipped elsewhere.
     */
    GraphNodeId node = kInvalidId;

    /// Junction centre in 2D local metres: GraphNode::position, or the loop centre
    glm::dvec2 center{0.0};

    /**
     * @brief World Y of the junction CARRIAGEWAY surface
     *
     * `RoadElevationSolver::node_height(node)` PLUS
     * ElevationConfig::surface_offset. node_height() deliberately excludes the
     * offset so that each consumer adds it exactly once; adding it here is that
     * one time, and the junction then lands on exactly the plane its arm end
     * stations land on. The terrain carve target, by contrast, is this value LESS
     * the offset -- that is, node_height() -- and carries it nowhere.
     */
    float height = 0.0f;

    /// Arms in ascending bearing order with trims solved; empty for a roundabout
    std::vector<ArmRef> arms;

    /// Cut cross-sections, parallel to `arms`
    std::vector<ArmEnd> ends;

    /// Carriageway footprint; invalid for kinds that do not build one
    JunctionPolygon polygon;

    /// Sidewalk ring; invalid when disabled or when the polygon could not be built
    CurbRing curb;

    /**
     * @brief Fill plus curb ring merged, materials tagged
     *
     * In world space, Y up, with `Mesh::sort_submeshes_by_material()` already
     * applied, so the ranges hold at most one entry per material and tile the
     * whole index buffer -- the same contract as Corridor::mesh.
     */
    Mesh mesh;

    /**
     * @brief Outer boundary for terrain carving, in 2D local metres
     *
     * Counter-clockwise, first point NOT repeated, on the same contract as
     * Corridor::outline and CarveRibbon::outline. It is CurbRing::outer when a
     * ring was built -- the ring is closed even though its mesh is open at every
     * arm, precisely so it can serve as a carve footprint -- and JunctionPolygon
     * ring otherwise. Empty for a degenerate junction, which falls back to the
     * provisional CarveDisc.
     */
    std::vector<glm::dvec2> footprint;

    /// How this junction was solved
    JunctionKind kind = JunctionKind::Intersection;

    /// Convenience for `kind == JunctionKind::Roundabout`
    bool is_roundabout = false;

    /**
     * @brief The junction produced usable geometry
     *
     * False for a node that was solved but yielded nothing to draw -- a degenerate
     * intersection, a degree-2 node whose profiles matched and needed no taper.
     * An invalid Junction is still returned, so a caller can count and debug it,
     * and its trims are still written onto the graph.
     */
    bool valid = false;
};

// ============================================================================
// Builder
// ============================================================================

/**
 * @brief Solves every node of a road graph into junction geometry and edge trims
 *
 * Usage:
 * @code
 *     JunctionBuilder junctions;
 *     std::vector<Junction> solved =
 *         junctions.build(graph, centerlines, profiles, elevation, cfg);
 *
 *     // ONLY NOW extrude, and only from the trimmed centerline
 *     const GraphEdge& e = graph.edge(id);
 *     Centerline trimmed = slice(centerlines[id],
 *                                e.trim_from,
 *                                centerlines[id].length() - e.trim_to);
 * @endcode
 *
 * The builder holds no GPU or IO state and is safe to run off the main thread, as
 * the rest of the road pipeline already is.
 */
class JunctionBuilder {
public:
    /**
     * @brief Solve every node of the graph
     *
     * Passes, in order:
     *
     * 1. **Roundabouts first.** find_roundabouts() over the whole graph. Loop
     *    edges are marked as consumed so the later passes do not also treat them
     *    as ordinary arms, and each valid loop emits one Junction of kind
     *    Roundabout. An invalid loop is left to the ordinary path.
     * 2. **Trims for every node of degree 3 or more.** collect_arms() then
     *    solve_arm_trims(). This pass runs over every such node BEFORE any
     *    geometry is built, because an edge is trimmed from both ends and the
     *    max_trim_fraction clamp at one end must be able to see what the other
     *    end asked for.
     * 3. **Write trims onto the graph.** For each arm, `arm.trim` goes to
     *    `trim_from` when at_start and `trim_to` otherwise. Where a pass-2 clamp
     *    bound, the edge is counted in Stats::over_trimmed_edges.
     * 4. **Geometry per node.** arm_end(), build_junction_polygon(),
     *    triangulate_junction(), and build_curb_ring() when enabled, merged into
     *    Junction::mesh. A taper whose demand pass 3 reduced is REBUILT here over
     *    its final trims, for the same reason the cut cross-sections are: the
     *    geometry has to end where the extruder cuts, or the surviving ribbon
     *    lies coplanar on top of the wedge.
     * 5. **Degree 2 and degree 1.** build_profile_taper() and build_dead_end() for
     *    the nodes the junction path does not cover, in pass 2. A taper writes its
     *    own two trims onto the graph; a dead end writes none. Both publish a
     *    Junction::footprint, because both cover ground no ribbon does: a taper's
     *    two ribbons are cut back off its wedge, and a bulb reaches past its arm's
     *    last station.
     *
     * Before any of it, TrimConfig's four `fillet_*` fields are filled from
     * `cfg.fillet` with apply_fillet_reserve(), so the trim solve leaves each
     * corner the straight run its arc's tangent points need. A caller invoking
     * solve_arm_trims() directly is responsible for that itself.
     *
     * A node whose trim solve returns false is emitted as JunctionKind::Degenerate
     * with an empty footprint and counted in Stats::degenerate. Its arms still get
     * `TrimConfig::min_trim`, so the graph is left in a consistent state either
     * way.
     *
     * Calling build() again discards the previous statistics. It does NOT reset
     * the graph's trims first, so calling it twice on one graph is idempotent only
     * because the solve overwrites both fields of every arm it touches; edges that
     * no pass touches keep whatever they had.
     *
     * @param graph       Built road graph. Taken MUTABLE because this is what
     *                    fills GraphEdge::trim_from and trim_to; nothing else in
     *                    the pipeline writes them.
     * @param centerlines Parallel to graph.edges(); centerlines[i] belongs to
     *                    edge i. A size mismatch yields an empty result.
     * @param profiles    Parallel to graph.edges(). A size mismatch yields an
     *                    empty result.
     * @param elevation   Solved node heights. When
     *                    RoadElevationSolver::is_solved() is false every junction
     *                    is placed at CorridorConfig::base_height instead, which
     *                    is the flat P2 behaviour.
     * @param cfg         Tunables; the defaults are the shipping values
     * @return One Junction per solved node plus one per roundabout loop, in
     *         ascending GraphNodeId order so a build is reproducible run to run.
     *         Invalid junctions are included; check Junction::valid.
     */
    [[nodiscard]] std::vector<Junction> build(RoadGraph& graph,
                                              const std::vector<Centerline>& centerlines,
                                              const std::vector<RoadProfile>& profiles,
                                              const RoadElevationSolver& elevation,
                                              const JunctionConfig& cfg = {});

    // ------------------------------------------------------------------------
    // Two-phase form
    // ------------------------------------------------------------------------

    /**
     * @brief Where one junction's dropped kerbs come from
     *
     * Invoked once per node that is about to build a curb ring, with that node's
     * id and the centre every span's directions must be measured from -- which is
     * Junction::center, and for an ordinary intersection is the graph node's own
     * position. Return a default-constructed KerbDrops to drop nothing.
     *
     * Called from SEVERAL WORKER THREADS AT ONCE, because build_geometry() solves
     * nodes in parallel. An implementation must therefore only read state that is
     * already final; dropped_kerb_spans() and driveway_kerb_spans() are pure
     * functions over const inputs and satisfy that as written.
     */
    using KerbDropProvider = std::function<KerbDrops(GraphNodeId node, glm::dvec2 center)>;

    /**
     * @brief Phase one: solve every trim and write it onto the graph
     *
     * Passes A, B and C of build(). On return, every GraphEdge::trim_from and
     * trim_to is final and Stats::over_trimmed_edges is filled; no junction
     * geometry exists yet and no Junction has been emitted.
     *
     * The split exists because of ONE ordering problem, and it is worth stating
     * plainly. A dropped kerb is a break in the curb ring, so it has to be known
     * before build_curb_ring() runs -- the ring is offset and re-tessellated
     * across each ramp, it is not punched afterwards. Crossings are what say
     * where those breaks go, and find_crossings() reads GraphEdge::trim_from and
     * trim_to to decide whether a crossing sits at a junction and how far back
     * off the trim station to push it. So crossings need the trims, and the ring
     * needs the crossings, and a single build() call has no seam between the two.
     *
     * This is that seam. The caller runs the trim solve, locates its crossings
     * against the finished trims, and hands the resulting spans back to
     * build_geometry(). Nothing is built twice and Clipper2 runs once per
     * junction, which a rebuild-the-ring-afterwards arrangement cannot claim.
     *
     * Calling solve_trims() again discards the previous phase-one state. It does
     * NOT reset the graph's trims first; see build() for what that means.
     *
     * @param graph       Built road graph, taken MUTABLE: this is what fills
     *                    GraphEdge::trim_from and trim_to
     * @param centerlines Parallel to graph.edges()
     * @param profiles    Parallel to graph.edges()
     * @param elevation   Solved node heights; an unsolved solver places every
     *                    junction at JunctionConfig::base_height
     * @param cfg         Tunables; retained for build_geometry()
     * @return True when phase one ran. False when the graph is empty or the
     *         parallel-vector contract is broken, in which case build_geometry()
     *         returns nothing.
     */
    bool solve_trims(RoadGraph& graph,
                     const std::vector<Centerline>& centerlines,
                     const std::vector<RoadProfile>& profiles,
                     const RoadElevationSolver& elevation,
                     const JunctionConfig& cfg = {});

    /**
     * @brief Phase two: build every junction's geometry against the solved trims
     *
     * Passes D and E of build(), plus the compaction. Must be preceded by a
     * solve_trims() that returned true; otherwise an empty list comes back and
     * the call is logged.
     *
     * @param graph        The same graph solve_trims() was given, with its trims
     *                     intact
     * @param centerlines  The same vector solve_trims() was given
     * @param profiles     The same vector solve_trims() was given
     * @param kerb_drops   Where each junction's dropped kerbs come from. Null
     *                     means no kerb is dropped anywhere, which is the P4
     *                     behaviour exactly.
     * @return One Junction per solved node plus one per roundabout loop, in
     *         ascending GraphNodeId order
     */
    [[nodiscard]] std::vector<Junction> build_geometry(
        RoadGraph& graph,
        const std::vector<Centerline>& centerlines,
        const std::vector<RoadProfile>& profiles,
        const KerbDropProvider& kerb_drops = nullptr);

    /**
     * @brief Counts describing the solve, for logging and tests
     */
    struct Stats {
        size_t junctions = 0;           ///< Junctions of kind Intersection
        size_t roundabouts = 0;         ///< Roundabout loops collapsed into a Junction
        size_t tapers = 0;              ///< Degree-2 nodes that needed a profile taper
        size_t dead_ends = 0;           ///< Degree-1 nodes capped

        /**
         * @brief Nodes of degree 3 or more whose trim solve failed
         *
         * Fewer than three usable arms after collection, or every adjacent pair
         * parallel. These fall back to the provisional CarveDisc footprint and
         * emit no fill.
         */
        size_t degenerate = 0;

        /// Junction polygons whose ring crossed itself and were filled as a convex hull
        size_t self_intersecting = 0;

        /**
         * @brief Edges where TrimConfig::max_trim_fraction bound the demanded trim
         *
         * The junction polygon overlaps that ribbon slightly. Counted per EDGE,
         * not per arm, so an edge clamped at both ends counts once.
         */
        size_t over_trimmed_edges = 0;

        double build_ms = 0.0;          ///< Wall-clock time of build(), milliseconds
    };

    /// Statistics of the last build(). Zeroed before the first one.
    [[nodiscard]] Stats stats() const { return m_stats; }

    /**
     * @brief Edges a valid roundabout loop consumed, indexed by EdgeId
     *
     * True for every edge of a loop that find_roundabouts() returned as valid and
     * that build_roundabout() then swept into an annulus. A consumed edge is
     * REPLACED WHOLESALE by that annulus, exactly as the plan's roundabout case
     * describes, so it is neither trimmed arm by arm at its approach nodes nor
     * extruded as an ordinary ribbon -- extruding it as well would lay a second
     * carriageway coplanar on the first.
     *
     * Approach nodes on the loop are still solved as ordinary junctions, because
     * the road joining the ring is not part of it and its own ribbon must still
     * be cut back. Only the ring's own arms are left untrimmed.
     *
     * Same size as the graph's edge list after a successful build(), and empty
     * before the first one.
     */
    [[nodiscard]] const std::vector<bool>& consumed_edges() const { return m_consumed_edges; }

    /**
     * @brief Junctions whose curb ring was actually broken by a dropped kerb
     *
     * Zero after a build() or a build_geometry() with no provider. Reported so an
     * operator can tell "no crossing was near a junction" from "the spans never
     * reached the ring", which are silent in the geometry and look identical.
     */
    [[nodiscard]] size_t dropped_kerb_junctions() const { return m_dropped_kerb_junctions; }

    JunctionBuilder();
    ~JunctionBuilder();
    JunctionBuilder(JunctionBuilder&&) noexcept;
    JunctionBuilder& operator=(JunctionBuilder&&) noexcept;

private:
    /// Phase-one state, carried from solve_trims() to build_geometry()
    struct SolveState;

    Stats m_stats;
    std::vector<bool> m_consumed_edges;
    std::unique_ptr<SolveState> m_solve;
    size_t m_dropped_kerb_junctions = 0;
};

/**
 * @brief Flatten an edge's solved station heights over its two junction trims
 *
 * The vertical solve pinned each edge's end station to its node height, because
 * before the junction solver that station sat ON the node. The trims then move it
 * `trim` metres away, where the solved profile is `grade * trim` higher or lower,
 * while the junction fill and its curb ring are one flat plane at the node
 * height. On an 8% hillside with the shipping trims that is a 30 cm open step at
 * every arm mouth, with nothing bridging it.
 *
 * Flattening each end over its own trim gives every junction a plateau exactly as
 * wide as the cut, so the resliced end station lands back on the junction plane.
 * That is also what a junction is in the ground: a level landing, with the grade
 * change pushed outside it.
 *
 * The station AT or just past each cut is flattened too, so a reslice at exactly
 * `trim` interpolates between two plateau stations and comes out at the node
 * height to the last bit.
 *
 * ### When the two plateaus meet
 *
 * A graph edge shorter than roughly `max_spacing / max_trim_fraction` has both
 * cuts landing on the same stations -- a staggered T, a dual-carriageway
 * crossover, a service-road stub. The two plateaus cannot both be honoured there:
 * an edge whose every station is inside both trims has no station left to hold a
 * grade. Applying them in sequence lets the second overwrite the first, which
 * flattens the whole edge to ONE node's height and leaves the other mouth a full
 * node-height difference below its junction plane -- a bigger step than the pass
 * exists to remove, and asymmetric, so it only ever shows at one end.
 *
 * So the disputed stations are shared instead: each is placed on the straight
 * line between the two node heights, by arclength. Both mouths are then off by
 * the same amount, and by no more than the un-plateaued solve was.
 *
 * @param centerline The UNTRIMMED centerline the heights are indexed against
 * @param trim_from  Cut at the `from` end, metres. Zero or non-finite for none.
 * @param trim_to    Cut at the `to` end, metres. Zero or non-finite for none.
 * @param level_from Height of the junction plane at the `from` node
 * @param level_to   Height of the junction plane at the `to` node
 * @param heights    Solved station heights, modified in place. Left untouched
 *                   when its size does not match @p centerline.
 */
void apply_junction_plateaus(const Centerline& centerline, double trim_from, double trim_to,
                             float level_from, float level_to, std::vector<float>& heights);

} // namespace stratum::osm::road
