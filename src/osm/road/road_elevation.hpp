/**
 * @file road_elevation.hpp
 * @brief Graph-aware vertical solve: roads follow the terrain without following its noise
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * P2 leaves every corridor flat at CorridorConfig::base_height. This is the pass
 * that gives each station a real world Y, and it is the input to both
 * CorridorConfig::station_heights and the terrain carve.
 *
 * ### Why this is a solver and not a sampler
 *
 * Sampling the terrain at each station and using the result directly produces a
 * rollercoaster: procedural terrain carries metre-scale noise that no road would
 * ever be built over. Four constraints turn the sampled profile into a road:
 *
 * 1. **Grade limit.** A maximum longitudinal gradient per road class, from 4% on
 *    a motorway to 15% on a path. This is the single most important constraint
 *    here and the one that removes the rollercoaster.
 * 2. **Vertical curvature limit.** A bound on how fast the gradient may change,
 *    so crests and sags are drivable rather than kinked.
 * 3. **Junction height agreement.** Every arm meeting at a graph node must
 *    terminate at ONE height. That cannot be decided per-way, which is the whole
 *    reason this solver takes the graph rather than a list of centerlines. Node
 *    heights are solved first and shared; arms are then solved between fixed
 *    endpoints.
 * 4. **Layer overrides.** A bridge edge lifts to a deck height above the terrain
 *    it spans and ignores the profile underneath. A tunnel edge drops below the
 *    terrain and suppresses carving.
 *
 * ### Why the terrain arrives as a callback
 *
 * Terrain is chunked and generated on demand, but road elevation must be solved
 * GLOBALLY and BEFORE any chunk is carved. That is only possible because the
 * procedural height field is a pure deterministic function of the terrain config
 * and a position: it does not need a generated chunk to be sampled. The solver
 * therefore queries that function through a HeightSampler and never learns that
 * terrain chunks exist.
 *
 * The second reason is layering. Nothing under `src/osm/road/` may include
 * anything under `src/procgen/`, so the whole road system stays unit-testable
 * against a synthetic surface -- a plane, a ramp, a single sine wave -- with no
 * terrain generator in the build at all.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"

#include <cstddef>
#include <functional>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Terrain access
// ============================================================================

/**
 * @brief Samples the underlying terrain surface
 *
 * Takes LOCAL 2D metres -- the same coordinates as GraphEdge::polyline and
 * Centerline stations -- and returns world Y in metres.
 *
 * The caller supplies the mapping onto its own height field. Stratum's
 * procedural terrain is indexed in the SAME 2D local frame -- its second axis is
 * local y, not render-space Z -- so the adapter passes both arguments through
 * unchanged:
 *
 * @code
 *     road::HeightSampler sampler = [&](double x, double y) {
 *         return generator.sample_surface(terrain_cfg,
 *                                         static_cast<float>(x),
 *                                         static_cast<float>(y));
 *     };
 * @endcode
 *
 * The Y-up render convention `(x, y_2d) -> vec3(x, height, -y_2d)` is applied
 * downstream of BOTH pipelines, by the corridor extruder and by
 * TerrainMeshBuilder, so it must not be applied a second time here. Negating the
 * second argument mirrors the whole road network against the terrain about
 * y = 0, and is not visible on a symmetric test surface.
 *
 * Deliberately a callback so the road system never links against procgen and can
 * be tested with a synthetic surface.
 *
 * @warning Must be callable from multiple threads at once. The solver runs edges
 *          in parallel, so the target must be re-entrant and must not mutate
 *          shared state. TerrainGenerator::sample_surface() satisfies this.
 */
using HeightSampler = std::function<float(double x, double y)>;

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tunables of the vertical solve
 *
 * All heights and distances are metres; all grades are dimensionless rise over
 * run.
 */
struct ElevationConfig {
    /**
     * @name Maximum longitudinal gradient by road class
     *
     * Roads that follow raw terrain noise look like rollercoasters; this is the
     * single most important constraint here. The defaults are the design values
     * from docs/plans/road_network_plan.md. See max_grade_for().
     */
    ///@{
    float max_grade_motorway    = 0.04f;
    float max_grade_trunk       = 0.05f;
    float max_grade_primary     = 0.06f;
    float max_grade_secondary   = 0.07f;
    float max_grade_tertiary    = 0.08f;
    float max_grade_residential = 0.08f;
    float max_grade_service     = 0.10f;
    float max_grade_path        = 0.15f;
    ///@}

    /**
     * @brief Maximum change in gradient per metre of travel
     *
     * Bounds crest and sag sharpness: the discrete second derivative of the
     * height profile with respect to arc length. 0.004 per metre is roughly a
     * 250 m vertical curve for a 1% gradient change, which reads as drivable at
     * urban scale.
     */
    float max_grade_change_per_m = 0.004f;

    /// Relaxation cap. The solve stops early once it converges.
    int max_iterations = 256;

    /// Metres; stop when no station height moves more than this in an iteration
    float convergence_epsilon = 0.001f;

    /// Deck height above the terrain a bridge spans, metres
    float bridge_clearance = 5.0f;

    /// Tunnel roadway depth below the terrain above it, metres
    float tunnel_depth = 8.0f;

    /**
     * @brief Keep a tunnel's portal NODES at the approach surface
     *
     * ### The defect this fixes
     *
     * A tunnel edge is currently pushed to `terrain - tunnel_depth` over its WHOLE
     * length, including its two end stations. The end stations are the portal
     * nodes -- the points where the road is supposed to enter the hillside -- so
     * the road is already 8 m underground where it meets its approach, and it is
     * underground by the same amount at both ends.
     *
     * Two things follow, and both are visible:
     *
     * - The approach edge terminates at the shared node height while the tunnel
     *   edge starts 8 m below it. The two arms of the same node do not meet, which
     *   is the exact failure junction height agreement exists to prevent.
     * - tunnel_builder places a portal where the solved road surface CROSSES below
     *   the terrain by at least TunnelConfig::min_portal_cover. A profile that
     *   starts below the terrain and stays below it never crosses, so no crossing
     *   is found, no headwall is cut, and `RoadNetwork::Stats::tunnels` is 0 on
     *   every fixture in tests/data. The portal geometry is not broken; it is never
     *   asked for.
     *
     * ### What the flag changes
     *
     * With this on, the tunnel override in step 4 of solve() applies to the
     * INTERIOR stations only. The first and last stations stay pinned at their
     * solved node heights -- the approach surface -- and the descent to
     * `terrain - tunnel_depth` is grade-limited from each end using the edge's own
     * max_grade_for(). The roadway therefore starts at the surface, dives, runs
     * deep through the middle, and climbs back, which is what a tunnel is.
     *
     * The crossing tunnel_builder looks for now exists: somewhere on each descent
     * the road passes min_portal_cover below the terrain, and that station is the
     * portal. No new tunable is introduced for the ramp length, because the grade
     * limit already determines it -- a 4% motorway takes 200 m to reach 8 m, a 10%
     * service road takes 80 m, and both are right.
     *
     * A tunnel edge SHORTER than twice its own ramp length never reaches full
     * depth. That is correct and is left alone: a 40 m tunnel under a 5 m mound is
     * a 5 m-deep tunnel, not an 8 m one, and forcing the depth would put a hole in
     * the mound. The profile is the maximum of the deep running line and one
     * descent cone per portal, so that case falls out of the shape rather than
     * needing a rule of its own, and the descent respects
     * max_grade_change_per_m as well as the class grade limit.
     *
     * ### Which ends ramp
     *
     * Only a PORTAL node. A long tunnel is split into several GraphEdges at its
     * interior nodes, and a node where nothing but tunnel edges meet is inside the
     * bore, not at its mouth: ramping there would make the roadway breach the
     * hillside every few hundred metres. Such a node keeps the pre-flag
     * behaviour -- dropped to depth and pinned -- and only an end that also
     * carries a surface arm, or a dead end, rises to meet the ground. The
     * standard tagging (approach way, `tunnel=yes` way, approach way) puts a
     * surface arm on both ends, so both ramp.
     *
     * Node heights are unaffected either way. node_height() already reports the
     * approach surface, and the change is that the tunnel edge's end stations now
     * agree with it instead of being dragged 8 m below.
     *
     * Setting this false restores the previous behaviour exactly, including the
     * zero tunnel count, and is kept only so the two can be diffed.
     *
     * @note EdgeElevation::is_tunnel is set the same way regardless, and
     *       CarveRibbon::suppress is still set for the whole tunnel span. The
     *       ramps are under the hillside like the rest of the tunnel and must not
     *       trench it open; the opening is the portal's job.
     */
    bool tunnel_portal_at_surface = true;

    /**
     * @brief Carriageway sits this far above the carved terrain, metres
     *
     * Added to every solved station height, so EdgeElevation::station_heights is
     * the road SURFACE and goes to the corridor extruder unchanged.
     *
     * The terrain carve must therefore subtract it again: CarveRibbon carries the
     * carve TARGET, `station_heights - surface_offset`, and CarveDisc carries the
     * raw node height. Applying the offset to both the mesh and the carve target
     * lifts the two together and leaves them coplanar, which is the failure this
     * field exists to prevent. Exactly one of the pair may carry it.
     */
    float surface_offset = 0.05f;

    /**
     * @brief Solve every node of a roundabout ring to ONE height
     *
     * A ring is a closed loop of edges, and the grade limit alone leaves it free
     * to follow the terrain: on a 5% hillside a 25 m roundabout's two opposite
     * approach nodes are 50 m apart and legitimately end up 2.5 m apart
     * vertically, because the half-circumference between them is 78 m and the
     * limit permits 5.5 m. The annulus, however, is ONE flat surface -- there is
     * no per-station height in a roundabout sweep -- and its terrain carve is one
     * flat disc, so every approach mouth then opens a vertical crack the size of
     * the difference across the ring.
     *
     * Setting the ring's nodes to their mean sampled height and PINNING them,
     * exactly as a bridge deck's abutments are pinned, makes the approaches climb
     * to meet the ring instead. That is also what a roundabout is in the ground:
     * a levelled site with graded approaches.
     */
    bool level_roundabouts = true;
};

/**
 * @brief Maximum gradient permitted for a road class
 *
 * Maps a RoadType onto the matching ElevationConfig field. Footway and Cycleway
 * share the path limit; Unknown takes the residential limit, which is the safe
 * middle of the table.
 *
 * @param type Road classification of the edge
 * @param cfg  Configuration holding the per-class limits
 * @return Rise over run, always > 0
 */
[[nodiscard]] float max_grade_for(RoadType type, const ElevationConfig& cfg);

// ============================================================================
// Output
// ============================================================================

/**
 * @brief Solved vertical profile for one edge
 *
 * Parallel to that edge's Centerline stations: station_heights[i] is the world Y
 * of station i. This vector is what CorridorConfig::station_heights expects, and
 * the sizes must match exactly or the corridor extruder degrades the edge to a
 * flat road.
 *
 * An edge that was skipped -- no valid centerline, or a graph edge that emitted
 * no piece -- has an empty station_heights.
 */
struct EdgeElevation {
    /// World Y of the carriageway surface per station, ElevationConfig::surface_offset included
    std::vector<float> station_heights;

    /**
     * @brief Edge is a bridge: terrain plus bridge_clearance, and never carved
     *
     * Set from the explicit `bridge=*` tag alone. `layer=*` is NOT read here: it
     * is a rendering-order hint that P1 already consumes to split
     * grade-separation nodes, and a plain `highway=residential, layer=-1` -- the
     * standard tagging for the lower road at a crossing -- is a surface road, not
     * a tunnel.
     */
    bool is_bridge = false;

    /// Edge is a tunnel: terrain minus tunnel_depth, never carved; from `tunnel=*` alone
    bool is_tunnel = false;

    /// Steepest gradient in the solved profile, rise over run, always >= 0
    float max_grade_used = 0.0f;
};

// ============================================================================
// Solver
// ============================================================================

/**
 * @brief Solves a vertically consistent surface for the whole network
 *
 * Usage:
 * @code
 *     RoadElevationSolver solver;
 *     solver.solve(graph, centerlines, sampler, cfg);
 *
 *     CorridorConfig corridor_cfg;
 *     corridor_cfg.station_heights = solver.edge(id).station_heights;
 * @endcode
 *
 * Node heights are solved FIRST and shared, because every arm meeting at a
 * junction must terminate at one height -- that cannot be done per-way, which is
 * why this is graph-aware.
 *
 * The solver holds no GPU or IO state, so it is safe to run off the main thread
 * on the import worker, as the rest of the road pipeline already does.
 */
class RoadElevationSolver {
public:
    /**
     * @brief Solve node and station heights for the whole graph
     *
     * Steps:
     * 1. Sample the terrain at every graph node position and seed the node
     *    heights with it.
     * 2. Relax the node heights against the grade limit of the edges joining
     *    them, so a node shared by a motorway and a service road respects the
     *    stricter of the two.
     * 3. For each edge, sample the terrain at every station, pin the first and
     *    last stations to the solved node heights, then iterate a monotone
     *    smoothing pass until no height moves by more than
     *    ElevationConfig::convergence_epsilon or max_iterations is reached. The
     *    pass enforces the grade limit first and the grade-change limit second.
     * 4. Override bridge edges to terrain plus bridge_clearance and tunnel edges
     *    to terrain minus tunnel_depth, then re-apply the grade limits so the
     *    approaches still meet their nodes. When
     *    ElevationConfig::tunnel_portal_at_surface, the tunnel override skips the
     *    first and last stations, which stay at their solved node heights, and the
     *    descent between them is grade-limited from each end -- so the roadway
     *    enters the ground instead of starting inside it. See that field for why
     *    the old behaviour left every fixture with zero portals.
     * 5. Add ElevationConfig::surface_offset to every solved height.
     *
     * Calling solve() again discards the previous result.
     *
     * @param graph       Built road graph
     * @param centerlines Parallel to graph.edges(); centerlines[i] belongs to
     *                    edge i. Must be the same size as graph.edges(); a
     *                    mismatch leaves the solver unsolved.
     * @param sampler     Terrain height query. Must be callable from multiple
     *                    threads. A null sampler leaves the solver unsolved.
     * @param cfg         Tunables; the defaults are the shipping values
     */
    void solve(const RoadGraph& graph,
               const std::vector<Centerline>& centerlines,
               const HeightSampler& sampler,
               const ElevationConfig& cfg = {});

    /// Drop the solved result and return to the unsolved state
    void clear();

    /**
     * @brief Solved profile of one edge
     *
     * @param id Edge index into the graph passed to solve()
     * @pre is_solved() and @p id is a valid EdgeId. No bounds check is
     *      performed, matching RoadGraph::edge().
     */
    [[nodiscard]] const EdgeElevation& edge(EdgeId id) const { return m_edges[id]; }

    /// Every solved edge profile, indexed by EdgeId. Empty before a successful solve().
    [[nodiscard]] const std::vector<EdgeElevation>& edges() const { return m_edges; }

    /**
     * @brief Shared world Y where every arm of a node meets
     *
     * This is the CARVED TERRAIN height at the junction, not the road surface:
     * ElevationConfig::surface_offset is deliberately NOT included, whereas
     * EdgeElevation::station_heights does include it. A consumer placing
     * junction geometry -- the provisional CarveDisc in P3, the real fillet
     * polygon in P4 -- adds the same offset itself, and the junction surface
     * then lands on exactly the plane the arm end stations land on. Adding it
     * here instead would make every consumer that also adds it float the
     * junction one offset above its own arms.
     *
     * @param id Node index into the graph passed to solve()
     * @pre is_solved() and @p id is a valid GraphNodeId. No bounds check is
     *      performed, matching RoadGraph::node().
     */
    [[nodiscard]] float node_height(GraphNodeId id) const { return m_node_heights[id]; }

    /// Every solved node height, indexed by GraphNodeId. Empty before a successful
    /// solve(). Excludes ElevationConfig::surface_offset; see node_height().
    [[nodiscard]] const std::vector<float>& node_heights() const { return m_node_heights; }

    /// True once solve() has produced a usable result
    [[nodiscard]] bool is_solved() const { return m_solved; }

    /**
     * @brief Counts describing the solve, for logging and tests
     */
    struct Stats {
        size_t nodes = 0;                   ///< Node heights solved
        size_t edges = 0;                   ///< Edge profiles solved
        /**
         * @brief Relaxation sweeps performed, summed over every phase
         *
         * The solve relaxes one GLOBAL height array rather than one array per
         * edge, so there is no per-edge sweep count to take a worst case of.
         * This is the node pre-pass, plus the station solve, plus the
         * re-relaxation that follows a bridge or tunnel override.
         *
         * The short smoothing pass that rounds one tunnel ramp is NOT counted.
         * It runs over a single edge's stations with both ends held, after the
         * global solve has finished, and folding it in would put a per-edge
         * count into a figure that otherwise describes whole-network sweeps.
         */
        size_t iterations = 0;
        size_t bridges = 0;                 ///< Edges lifted to a deck height
        size_t tunnels = 0;                 ///< Edges dropped below the terrain
        /**
         * @brief Edges where the grade limit actually bound
         *
         * Measured against the RAW terrain samples: an edge is counted when some
         * span of the sampled profile was steeper than its road class allows, so
         * the solved road had to depart from the ground. Bridges and tunnels are
         * never counted, because a deck's gradient comes from its abutments and
         * the ground under it is irrelevant.
         */
        size_t grade_limited_edges = 0;
        /// Largest height change in the final iteration, metres; <= convergence_epsilon on convergence
        float  max_residual = 0.0f;
        double solve_ms = 0.0;              ///< Wall-clock time of solve(), milliseconds
    };

    /// Statistics of the last solve(). Zeroed before the first one.
    [[nodiscard]] Stats stats() const { return m_stats; }

private:
    std::vector<EdgeElevation> m_edges;
    std::vector<float> m_node_heights;
    Stats m_stats;
    bool m_solved = false;
};

} // namespace stratum::osm::road
