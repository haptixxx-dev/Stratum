/**
 * @file road_network_builder.hpp
 * @brief Whole-network road geometry, built once against the graph and chunked after
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This is the entry point of the road pipeline and the fix for the defect that
 * made exported road meshes unusable: geometry used to be built per spatial-index
 * leaf, so topology stopped at leaf boundaries and every junction whose arms fell
 * in different leaves was missed or built wrong.
 *
 * The order is now the other way round. RoadNetworkBuilder runs once over the
 * complete ParsedOSMData, builds a RoadGraph, extrudes every edge against it, and
 * hands back RoadPieces. The spatial index then assigns finished pieces to leaves.
 * It never splits geometry and it never sees a road again.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/bridge_builder.hpp"
#include "osm/road/carve_request.hpp"
#include "osm/road/centerline.hpp"
#include "osm/road/collision_mesh.hpp"
#include "osm/road/corridor.hpp"
#include "osm/road/crossings.hpp"
#include "osm/road/junction_builder.hpp"
#include "osm/road/markings.hpp"
#include "osm/road/mesh_optimize.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/road/sidewalk_dedup.hpp"
#include "osm/road/tunnel_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Output
// ============================================================================

/**
 * @brief A spatially-anchored chunk of the road network
 *
 * Whole triangles only. The spatial index assigns pieces, it never splits
 * geometry, so a piece is indivisible from the index's point of view and is
 * stored in exactly one leaf. That is what stops the old per-tile duplication of
 * roads from coming back.
 *
 * A piece is one graph edge, or one solved junction. `edge == kInvalidId`
 * already means "not a single edge", and that is exactly what a junction piece
 * uses: a junction spans several edges and belongs to none of them. A consumer
 * that wants to know WHICH junction a piece came from matches its anchor against
 * Junction::center, or walks RoadNetwork::junctions directly; the piece list
 * deliberately carries no second id, so nothing downstream has to branch on a
 * piece's provenance to draw it.
 */
struct RoadPiece {
    /**
     * @brief Representative position for spatial assignment, in 2D local metres
     *
     * For an edge piece this is the position of the station nearest the middle of
     * the corridor's arclength span. The spatial index routes the whole piece by
     * this point.
     */
    glm::dvec2 anchor{0.0};

    /// Graph edge this piece was built from; kInvalidId for pieces that are not a single edge
    EdgeId edge = kInvalidId;

    /// Geometry in world space, Y up, with one SubMesh range per MaterialId
    Mesh mesh;

    /**
     * @brief Corridor footprint, for P3 terrain carving
     *
     * Closed CCW ring in 2D local metres, first point not repeated. Copied
     * straight from Corridor::outline; see corridor.hpp for the orientation
     * contract. Empty when the piece has no meaningful footprint.
     */
    std::vector<glm::dvec2> outline;

    // ------------------------------------------------------------------
    // P7 optional outputs
    //
    // Both default to empty and stay empty unless the matching
    // RoadNetworkConfig flag asked for them. Nothing in the pipeline reads
    // either one; they exist to be handed to a physics engine and to an
    // exporter, and a piece is complete without them.
    // ------------------------------------------------------------------

    /**
     * @brief Physics surface derived from `mesh`, without paint or kerb faces
     *
     * Filled when RoadNetworkConfig::build_collision. Derived from the finished
     * render mesh rather than built independently, so the two can never disagree
     * about where the road is; see collision_mesh.hpp.
     *
     * Its `submeshes` is empty -- one implicit MaterialId::Default range -- because
     * materials have collapsed. Empty mesh when the piece was nothing but markings.
     */
    Mesh collision;

    /**
     * @brief Level-of-detail chain for `mesh`
     *
     * Filled when RoadNetworkConfig::build_lods. `lods.levels[0]` is a full-detail
     * copy of `mesh` after optimisation, so a consumer that uses the chain can
     * ignore `mesh` entirely, and one that ignores the chain loses nothing.
     *
     * @warning The duplication is real: with LODs on, a piece holds its geometry
     *          roughly 1.85 times over on the CPU at the default ratios. On a city
     *          extract that is not free, which is why build_lods defaults off.
     */
    LodChain lods;
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Every tunable of the road pipeline, in one place
 *
 * Held by value so a caller can hand one default-constructed instance to
 * build() and get sensible geometry with no setup.
 */
struct RoadNetworkConfig {
    ResampleConfig  resample;   ///< Centerline resampling and miter tolerances
    ProfileConfig   profile;    ///< Cross-section widths and heights
    CorridorConfig  corridor;   ///< Vertical placement and optional outputs
    ElevationConfig elevation;  ///< Grade limits, bridge and tunnel offsets
    JunctionConfig  junction;   ///< Trim, fillet, curb ring, and special-case tunables
    MarkingConfig   markings;   ///< Lane lines, stop lines, and turn arrows
    CrossingConfig  crossings;  ///< Zebra stripes and dropped kerbs
    DedupConfig     dedup;      ///< Doubly-mapped sidewalk detection
    BridgeConfig    bridge;     ///< Deck, parapet, and pier dimensions
    TunnelConfig    tunnel;     ///< Portal dimensions

    /**
     * @brief Run the P4 junction solve
     *
     * When true, JunctionBuilder solves every node, writes GraphEdge::trim_from
     * and trim_to, and each edge is extruded from the TRIMMED centerline. Junction
     * fill, fillets and curb rings are emitted as their own pieces, and
     * RoadNetwork::junctions comes back populated.
     *
     * When false the P2 behaviour is restored exactly: every trim stays zero,
     * every edge is extruded over its full length, ribbons overlap at every
     * junction, and RoadNetwork::junctions is empty. Kept as a switch because the
     * untrimmed output is the reference the junction tests diff against, and
     * because a solver failure should be bisectable without a rebuild.
     */
    bool solve_junctions = true;

    /**
     * @brief Emit painted lane markings
     *
     * When true, each edge piece additionally carries build_edge_markings() for
     * its own edge and build_approach_markings() for each of its ends that meets a
     * junction, all in MaterialId::Markings.
     *
     * When false no marking geometry is emitted at all and the output is
     * byte-identical to the previous phase's. Each of these three switches
     * reproduces the earlier phase exactly on its own, so a visual regression can
     * be bisected to a pass without a rebuild.
     */
    bool emit_markings = true;

    /**
     * @brief Emit pedestrian crossings
     *
     * When true, find_crossings() locates every crossing and build_crossing()
     * appends its zebra into the piece of the edge it sits on. Dropped kerb spans
     * are computed per junction and counted; see crossings.hpp for what consumes
     * them.
     *
     * Independent of emit_markings even though both produce MaterialId::Markings
     * geometry: a crossing is located from OSM topology and a lane line is derived
     * from the profile, and they fail in different ways.
     */
    bool emit_crossings = true;

    /**
     * @brief Emit bridge and tunnel structure
     *
     * When true, a bridge edge's piece additionally carries build_bridge() and a
     * tunnel edge's piece carries build_tunnel_portals(). Both need a terrain
     * height under the road, so both are skipped ENTIRELY when
     * RoadNetworkConfig::height_sampler is null, whatever this flag says: a pier
     * with nothing to stand on and a portal with no hillside to cut are not worth
     * emitting, and a deck without them is a slab floating over nothing. The skip
     * is logged, so a missing bridge is never silent.
     */
    bool emit_structures = true;

    /**
     * @brief Terrain surface the network is built on
     *
     * When set, the network is built ON TERRAIN: heights are solved once for the
     * whole graph and fed into CorridorConfig::station_heights per edge, and
     * RoadNetwork::carve_ribbons and carve_discs come back filled so the terrain
     * can be carved to match.
     *
     * When null, roads stay flat at CorridorConfig::base_height, which is the P2
     * behaviour, and both carve vectors come back empty.
     *
     * The callback is invoked from several worker threads at once; see
     * HeightSampler for the coordinate convention and the thread-safety
     * requirement.
     */
    HeightSampler height_sampler = nullptr;

    // ------------------------------------------------------------------
    // P7: game-ready output
    //
    // Four independent switches over one new stage that runs after the
    // network is otherwise finished and compacted. See RoadNetworkBuilder::build()
    // for where the stage sits and why it is last.
    //
    // All four are off-by-default except welding and optimisation, which are
    // on because they only ever make the same geometry cheaper. Collision and
    // LODs are off because both ADD data, and a caller that does not want them
    // should not pay for them on a 63 MB extract.
    // ------------------------------------------------------------------

    WeldConfig      weld;       ///< Tolerances for the per-piece vertex weld
    LodConfig       lod;        ///< Reordering flags, LOD ratios, and simplify error
    CollisionConfig collision;  ///< What the collision variant keeps and how hard it simplifies

    /**
     * @brief Deduplicate coincident vertices in every piece
     *
     * Each piece is welded on its own with RoadNetworkConfig::weld. Nothing
     * downstream depends on the vertex numbering -- SubMesh ranges are preserved
     * exactly, and the outline is 2D and separate -- so this is a pure size
     * reduction and is on by default.
     *
     * @note The weld is PER PIECE, not across pieces. The seam between an edge
     *       piece and the junction piece it runs into is therefore not welded, and
     *       normals are discontinuous across it. That is deliberate and not a
     *       shortcut: pieces are assigned independently to quadtree leaves and are
     *       loaded, evicted and exported independently, so a vertex shared between
     *       two pieces would have no owner. The plan's "weld across edge and
     *       junction boundaries" is satisfied within a piece, which is where the
     *       corridor extruder's per-station duplication actually is.
     *
     * @note The weld leaves DEGENERATE triangles behind, on purpose. Collapsing
     *       two vertices of a triangle makes that triangle index-degenerate, and
     *       weld_vertices() keeps it rather than removing it, because dropping a
     *       triangle would shorten a SubMesh range and shift every range after
     *       it. On the tests/data fixtures this is 12 to 28 triangles per
     *       network, and every one of them was a sliver under 0.1 mm wide before
     *       the weld: their combined area is under 0.00001% of the network's
     *       surface. They cost one primitive each and rasterise nothing. A
     *       consumer that counts triangles by walking the index buffer must
     *       expect them.
     */
    bool weld_meshes = true;

    /**
     * @brief Run the meshoptimizer reordering passes on every piece
     *
     * optimize_mesh() with RoadNetworkConfig::lod. Same geometry, same triangle
     * count, same SubMesh ranges. Off only to isolate a suspected meshoptimizer
     * problem.
     *
     * @warning It is NOT true that only the order changes, and two of its
     *          side effects are visible from outside:
     *
     *          - The fetch pass COMPACTS the vertex array, so any vertex no
     *            triangle references is dropped. That reduction is real and is
     *            not counted in Stats::vertices_welded, which counts the WELD
     *            alone: it is counted in Stats::vertices_dropped instead, and on
     *            the tests/data fixtures it is 0 to 76 vertices per network. The
     *            pre-P7 vertex total is therefore `vertices + vertices_welded +
     *            vertices_dropped`, and that identity is exact.
     *          - The overdraw pass sorts clusters by a heuristic read off
     *            absolute vertex positions, so the same network built at two
     *            different altitudes comes back with the same triangles in a
     *            DIFFERENT index order. Nothing downstream may assume that two
     *            builds which differ only by a translation agree vertex index
     *            for vertex index; they agree as multisets of triangles.
     */
    bool optimize_meshes = true;

    /**
     * @brief Fill RoadPiece::collision
     *
     * Costs one derivation and one simplify per piece, and roughly a third of the
     * render mesh again in memory. Off by default because the editor does not run
     * physics; a game export turns it on.
     */
    bool build_collision = false;

    /**
     * @brief Fill RoadPiece::lods
     *
     * The most expensive of the four by a wide margin: simplification runs once per
     * material range per level, so a kerbed profile at the default three ratios is
     * a dozen meshopt_simplify() calls per piece. Off by default.
     */
    bool build_lods = false;
};

/**
 * @brief The extruded road network as a list of spatially-anchored pieces
 */
struct RoadNetwork {
    /// Pieces in ascending EdgeId order, so a build is reproducible run to run
    std::vector<RoadPiece> pieces;

    /**
     * @brief Corridor footprints for the terrain carve, one per emitted EDGE piece
     *
     * Parallel to the EDGE pieces: carve_ribbons[i] is the footprint of
     * pieces[i]. Junction pieces are appended after every edge piece and carry no
     * ribbon, so the parallel run covers the first
     * `stats.pieces - stats.junction_pieces` entries of `pieces` and this vector
     * is that long. With RoadNetworkConfig::solve_junctions false there are no
     * junction pieces and the two are the same length, which is the P3 contract
     * unchanged.
     *
     * The geometry described is the TRIMMED ribbon, not the untrimmed edge:
     * a carve built from the untrimmed centerline would flatten a band of terrain
     * running through every junction that no ribbon covers any more.
     *
     * Empty when RoadNetworkConfig::height_sampler is null, because a flat road
     * at base_height has no relationship with the terrain to express.
     *
     * These are declared in osm/road/carve_request.hpp rather than in procgen,
     * so the road module can hand the terrain module a footprint without
     * including a procgen header and without a conversion step. procgen aliases
     * the same types into its own namespace; see terrain_carve.hpp.
     */
    std::vector<CarveRibbon> carve_ribbons;

    /**
     * @brief Junction footprints, one per graph node of degree >= 3, plus one per roundabout
     *
     * A valid roundabout loop adds one further disc covering its whole ring,
     * because its edges emit no CarveRibbon -- the annulus replaced them -- and
     * the ring would otherwise sit on uncarved terrain. With
     * RoadNetworkConfig::solve_junctions false there are no roundabout discs and
     * the count is exactly one per degree >= 3 node, which is the P3 contract
     * unchanged.
     *
     * With RoadNetworkConfig::solve_junctions set, each disc carries the REAL
     * fillet-and-curb-ring boundary in CarveDisc::outline, taken from
     * Junction::footprint, and the terrain neighbourhood is carved against that
     * rather than against a circle.
     *
     * CarveDisc is KEPT rather than replaced. Two reasons, both about failure
     * rather than about the common case: a node whose trim solve is degenerate has
     * no polygon at all and must still carve something under its arm mouths, and a
     * consumer that has not been taught about polygons -- which every existing one
     * is -- must keep working. CarveDisc::center and radius are therefore always
     * populated and always bound the outline, so the polygon is a refinement of
     * the disc and never a contradiction of it.
     *
     * Empty when RoadNetworkConfig::height_sampler is null.
     */
    std::vector<CarveDisc> carve_discs;

    /**
     * @brief Tunnel portal mouths for the terrain carve, one per emitted portal
     *
     * Filled from build_tunnel_portals() as each tunnel edge is built, in
     * ascending EdgeId order and, within one edge, start end before far end. NOT
     * parallel to `pieces`: a tunnel edge contributes zero, one or two entries and
     * every other edge contributes none, so this vector is indexed only by itself.
     *
     * A portal's geometry frames an opening, and the terrain would otherwise close
     * over that opening a few metres in. Each entry asks the carve to clamp the
     * ground down to the crown of the arch over the mouth footprint and blend back
     * out; see TunnelPortalFootprint for the exact contract, which is a CLAMP and
     * never a set.
     *
     * Empty when RoadNetworkConfig::height_sampler is null or
     * RoadNetworkConfig::emit_structures is false, which are exactly the
     * conditions under which no portal geometry is emitted either. A mouth is
     * never carved for a portal that was not built.
     */
    std::vector<TunnelPortalFootprint> carve_portals;

    /**
     * @brief Every solved junction, in ascending GraphNodeId order
     *
     * Empty when RoadNetworkConfig::solve_junctions is false. Populated even when
     * height_sampler is null, because junction geometry does not depend on the
     * terrain -- only its carve footprint does.
     *
     * The geometry has already been copied into `pieces`, so this vector is kept
     * for the arm geometry, trim stations, polygons and rings the editor debug
     * overlay and the golden tests read. Junction::mesh is left populated rather
     * than moved out, so a Junction is self-describing when it is inspected.
     */
    std::vector<Junction> junctions;

    /**
     * @brief Counts from the P4 junction solve
     *
     * Zeroed when RoadNetworkConfig::solve_junctions is false, which is
     * indistinguishable from "solved, and there was nothing to solve" -- check
     * the flag, not the counts, to tell those apart.
     *
     * Lifted out of the JunctionBuilder, which is local to build() and dies with
     * it. The counts an operator needs are the ones no other field carries:
     * `degenerate` says how many nodes fell back to a provisional disc footprint,
     * and `over_trimmed_edges` says how many ribbons the junction polygons still
     * overlap. Both are silent in the geometry and both are what a bad import
     * looks like from the outside.
     */
    JunctionBuilder::Stats junction_stats;

    /**
     * @brief Counts describing the build, for logging and tests
     */
    struct Stats {
        size_t edges = 0;           ///< Graph edges considered
        size_t pieces = 0;          ///< Pieces emitted
        size_t vertices = 0;        ///< Vertices summed over every piece
        size_t triangles = 0;       ///< Triangles summed over every piece
        size_t skipped_edges = 0;   ///< Edges that produced no geometry
        size_t elevated_edges = 0;  ///< Edges whose heights came from the terrain solve
        size_t junction_pieces = 0; ///< Pieces emitted from solved junctions
        size_t trimmed_edges = 0;   ///< Edges extruded from a trimmed centerline
        /**
         * @brief Edges the junction trims consumed entirely
         *
         * Counted in Stats::skipped_edges as well, since they emit no piece.
         * Broken out because a non-zero value here means TrimConfig's clamps are
         * not holding and short edges are being eaten, which is a different
         * problem from an edge that was unusable to begin with.
         */
        size_t trimmed_away_edges = 0;

        // --------------------------------------------------------------------
        // P5 and P6 counts
        //
        // Marking, crossing and structure geometry is APPENDED into the piece it
        // belongs to rather than emitted as pieces of its own. That keeps the
        // carve_ribbons parallel-run contract intact -- it is defined against the
        // first `pieces - junction_pieces` entries of `pieces`, which a new class
        // of piece would silently break -- and it keeps a road and its paint in
        // one spatial leaf, which is what a consumer wants anyway.
        // --------------------------------------------------------------------

        /**
         * @brief Pieces carrying marking geometry
         *
         * A count of PIECES, not of quads or of lines: an edge piece with a
         * centre line, two edge lines and a stop line counts once. Zero when
         * RoadNetworkConfig::emit_markings is false, which is indistinguishable
         * from "emitted, and nothing was painted" -- check the flag, not the
         * count, to tell those apart. The same reading applies to every count
         * below.
         *
         * The zero is enforced by the FLAG and not only by the geometry, because
         * a crossing zebra is MaterialId::Markings as well: with the markings
         * pass off and the crossings pass on there is still paint on the road,
         * and counting it here would report the crossings pass under a name that
         * says lane line. Use Stats::crossings for that. With the markings pass
         * on, the two agree and this is exactly the number of pieces carrying any
         * Markings triangle.
         */
        size_t markings_pieces = 0;

        /// Crossings located by find_crossings() and emitted
        size_t crossings = 0;

        /**
         * @brief Dropped-kerb spans handed to the junction curb-ring stage
         *
         * Summed over every junction that built a ring, and over both sources:
         * dropped_kerb_spans() for crossings and driveway_kerb_spans() for
         * driveway mouths. Counted BEFORE build_curb_ring() merges overlaps, so
         * this is how many drops were demanded and not how many runs of kerb
         * ended up lowered.
         *
         * Zero when RoadNetworkConfig::emit_crossings or
         * CrossingConfig::emit_dropped_kerbs is false, and zero when the junction
         * solve did not run -- with no ring there is nothing to break. It is the
         * only outside evidence that the crossing pass reached the kerb at all:
         * Stats::crossings counts paint, and a zebra running into a full-height
         * kerb is the defect crossings.hpp exists to avoid.
         */
        size_t dropped_kerb_spans = 0;

        /// Bridge edges that produced structure geometry
        size_t bridges = 0;

        /// Tunnel edges that produced at least one portal
        size_t tunnels = 0;

        /**
         * @brief Carriageway sides whose synthesised sidewalk was suppressed
         *
         * Counted per SIDE, not per edge, so an edge suppressed on both sides
         * adds two. DedupResult::suppressed_edges counts edges; this is the
         * number that says how much sidewalk actually stopped being built.
         */
        size_t deduped_sidewalks = 0;

        // --------------------------------------------------------------------
        // P7 counts
        //
        // All zero when the matching RoadNetworkConfig switch is off, which is
        // indistinguishable from "ran, and there was nothing to do" -- check the
        // flag, not the count.
        // --------------------------------------------------------------------

        /**
         * @brief Vertices removed by the per-piece weld, summed over every piece
         *
         * Stats::vertices is the count AFTER welding, so the pre-weld total is
         * `vertices + vertices_welded`.
         *
         * ### What to expect, measured
         *
         * A SMALL number, and that is the correct answer rather than a broken
         * weld. On a 65 MB Lucan extract -- 40,649 pieces, 3.82 M vertices,
         * 3.19 M triangles -- the default WeldConfig removes 24,967 vertices,
         * which is 0.65%.
         *
         * The duplication is really there: welding on POSITION ALONE, at a 1 mm
         * epsilon and with every attribute test disabled, removes 1,208,694
         * vertices, or 31.6% of the network. Almost all of that is held apart on
         * purpose, and loosening the tolerances recovers very little of it --
         * position at 1 mm reaches 1.5%, ignoring UVs entirely reaches 7.5%,
         * ignoring normals as well reaches 7.8%. The rest needs the material
         * mask, the vertex colour and the tangent test switched off too.
         *
         * The reason is the cross-section, not float error. Every strip boundary
         * of the profile is a real seam: lateral U restarts at each strip, so the
         * gutter's edge column and the kerb top's edge column share a position
         * and carry different UVs; the kerb face meets the kerb top at 90
         * degrees, so they share a position and carry different normals; and the
         * two sides usually carry different MaterialIds. Welding any of those
         * would flatten a crease, stretch a texture, or merge two material
         * ranges. See WeldConfig::normal_epsilon, which says the same thing from
         * the other end.
         *
         * So the pass is worth having -- it closes the seams between geometry
         * that different builders emitted at the same place with the same
         * attributes, which is what it was written for -- but it is not where a
         * large vertex reduction on this pipeline comes from. Cutting the station
         * count or the strip count is.
         */
        size_t vertices_welded = 0;

        /**
         * @brief Unreferenced vertices dropped by the reorder, summed over pieces
         *
         * Zero when RoadNetworkConfig::optimize_meshes is off. The fetch pass of
         * optimize_mesh() compacts the vertex array, so a vertex that no triangle
         * references is dropped there rather than welded, and it is counted here
         * rather than in Stats::vertices_welded.
         *
         * The two counts together make the accounting exact:
         *
         *     vertices + vertices_welded + vertices_dropped
         *
         * is the vertex count the P7 stage was handed. Nothing else removes a
         * vertex, so the identity holds on every fixture and every extract.
         *
         * These are not float noise. The junction curb ring emits a vertex column
         * per ring station and then refuses any triangle whose area falls below
         * the degeneracy floor, which leaves the column behind with nothing
         * pointing at it; on the tests/data fixtures that is 0 to 76 vertices per
         * network. Dropping them is a real, if small, reduction.
         */
        size_t vertices_dropped = 0;

        /**
         * @brief Triangles across every piece's full-detail mesh
         *
         * Equal to Stats::triangles when build_lods is on, and 0 when it is off.
         * Kept separate so the pair below reads as a ratio without the reader
         * having to know that Stats::triangles counts level 0.
         */
        size_t triangles_before_lod = 0;

        /**
         * @brief Triangles across the COARSEST level of every chain
         *
         * Not the sum over all levels -- that would be a memory figure, and this is
         * a reduction figure. `triangles_after_lod / triangles_before_lod` is what
         * the last LOD actually achieved, and it is normally well above the
         * requested ratio, because LodConfig::target_error and
         * LodConfig::lock_borders both refuse collapses the ratio asked for. That
         * is the simplifier protecting the silhouette and the chunk seams, not a
         * failure.
         */
        size_t triangles_after_lod = 0;

        /// Triangles across every RoadPiece::collision. Zero when build_collision is off.
        size_t collision_triangles = 0;

        double elevation_ms = 0.0;  ///< Share of build_ms spent in the elevation solve
        double junction_ms = 0.0;   ///< Share of build_ms spent in the junction solve
        double build_ms = 0.0;      ///< Wall-clock time of build(), milliseconds
    } stats;
};

// ============================================================================
// Builder
// ============================================================================

/**
 * @brief Builds the whole road network from parsed OSM data
 *
 * Usage:
 * @code
 *     RoadNetworkBuilder builder;
 *     RoadNetwork network = builder.build(parsed_data);
 *     for (const RoadPiece& piece : network.pieces) {
 *         quadtree.insert_road_piece(piece);
 *     }
 * @endcode
 *
 * The builder keeps the graph it built so callers can inspect topology -- the
 * junction solver, the elevation solver, and the editor debug overlay all need
 * it -- without paying to rebuild it.
 *
 * The builder holds no GPU or IO state, so it is safe to run off the main thread
 * on the import worker, as the OSM import path already does for parsing.
 *
 * build() itself extrudes edges in parallel across the enkiTS scheduler, because
 * a city extract carries tens of thousands of independent edges and this sits on
 * the import path. The parallelism is invisible in the result: workers fill
 * pre-sized slots indexed by EdgeId and never push, so the emitted piece list is
 * identical run to run whatever the scheduling. See road_network_builder.cpp.
 */
class RoadNetworkBuilder {
public:
    /**
     * @brief Build the road network from every road in @p data
     *
     * Steps:
     * 1. Build the RoadGraph. See RoadGraph::build() for which roads it skips.
     * 2. For each graph edge, in EdgeId order: build a Centerline from its
     *    polyline and a RoadProfile from its tags.
     * 3. When cfg.height_sampler is set, run RoadElevationSolver over the whole
     *    graph. This happens after every centerline exists and before any
     *    corridor is extruded, because a junction height is shared by its arms
     *    and cannot be decided one edge at a time.
     * 4. When cfg.solve_junctions, run JunctionBuilder over the whole graph. It
     *    writes GraphEdge::trim_from and trim_to and returns the junctions. This
     *    happens after the elevation solve, because a junction is placed at its
     *    solved node height, and before any corridor is extruded, because the
     *    trims decide how much of each edge there is to extrude.
     * 5. For each graph edge: build a Corridor from the centerline and profile,
     *    with CorridorConfig::station_heights taken from the solved edge profile
     *    when there is one. When the edge carries trims, the centerline is first
     *    cut to
     *    `slice(cl, edge.trim_from, cl.length() - edge.trim_to)`, and its station
     *    heights are cut to match. slice() does not rebase arclength, so a trimmed
     *    ribbon keeps the V texture placement of the untrimmed one and re-running
     *    the solve never shifts a road's texture.
     * 6. Wrap each corridor in a RoadPiece anchored at its mid-arclength station,
     *    and emit its CarveRibbon when the network was built on terrain.
     * 7. Emit one RoadPiece per valid Junction, anchored at Junction::center with
     *    EdgeId kInvalidId, and fill each CarveDisc's outline from
     *    Junction::footprint.
     *
     * ### Where P5 and P6 sit in that order
     *
     * Three stages are interleaved into the list above rather than appended to
     * it, because each has exactly one place it can run:
     *
     * - **Sidewalk dedup** runs between step 2 and step 3, and it is the only one
     *   that runs BEFORE the profiles are final. It reads the provisional
     *   profiles, returns a suppression mask, and the affected profiles are then
     *   rebuilt with GraphEdge::sidewalk masked by mask_side(). Everything after
     *   it -- elevation, trims, junction polygons, corridor outlines, the terrain
     *   carve -- depends on the profile widths it changes, so running it any later
     *   would leave a junction solved for a road that is no longer that wide. Gated
     *   by DedupConfig::enabled.
     * - **Markings and crossings** run after step 5, against the edge's own
     *   corridor. They read the UNTRIMMED centerline and the solved trims, and
     *   their geometry is appended into that edge's piece. Approach markings need
     *   the junction node's has_signals and the arm's solved trim, so they cannot
     *   run before step 4.
     * - **Bridges and tunnels** run after step 5 as well, against the TRIMMED
     *   centerline the corridor was extruded from, and are appended into the same
     *   piece. They need the terrain sampler for piers and portals, so they are
     *   skipped entirely when cfg.height_sampler is null.
     *
     * None of the three emits a piece of its own. See RoadNetwork::Stats for why
     * that matters to the carve_ribbons contract.
     *
     * ### Where P7 sits
     *
     * 8. Each piece, once it is otherwise finished and BEFORE it is compacted, is
     *    put through, in this order: weld_vertices() with cfg.weld when
     *    cfg.weld_meshes; optimize_mesh() with cfg.lod when cfg.optimize_meshes;
     *    build_collision_mesh() with cfg.collision into RoadPiece::collision when
     *    cfg.build_collision; build_lod_chain() with cfg.lod into RoadPiece::lods
     *    when cfg.build_lods.
     *
     * The order inside step 8 is not free choice:
     *
     * - Welding is FIRST because both simplification and the collision derivation
     *   need shared topology. meshopt_simplify() collapses edges, and an unwelded
     *   mesh has no interior edge to collapse, so an unwelded chain comes back with
     *   every level identical to level 0.
     * - Collision is derived from the WELDED and OPTIMISED render mesh, not from
     *   the raw one, so it inherits the weld for free and can never disagree with
     *   what is drawn.
     * - LODs are LAST because level 0 is defined as the finished render mesh.
     *   Building the chain before optimisation would leave level 0 unoptimised
     *   while every other level was not.
     *
     * Step 8 is NOT a separate sweep over the finished piece list. It runs inside
     * the two parallel piece-building passes -- in the same worker that extruded
     * an edge piece, immediately after the markings, crossings and structures of
     * step 5 have been appended into it, and in a matching parallel pass over the
     * solved junctions. Folding it in costs nothing to read and saves a second
     * traversal of the largest data structure the build produces; the passes are
     * per piece and share nothing, so they parallelise exactly as well there as
     * they would anywhere else.
     *
     * What it must NOT be is earlier than the appends. Markings, zebras, decks and
     * portals append whole meshes into a piece without looking at what is already
     * in it, so a weld run before them would miss everything they carry.
     *
     * The compaction of step 5 still decides the piece ORDER, and still runs on
     * the calling thread in ascending EdgeId order with the junctions after it, so
     * the result is identical run to run.
     *
     * Stats::vertices and Stats::triangles are summed AFTER step 8, so they report
     * what a consumer actually receives. Note that with cfg.optimize_meshes on
     * the vertex count falls by MORE than Stats::vertices_welded, and the excess
     * is Stats::vertices_dropped; see those fields and
     * RoadNetworkConfig::optimize_meshes.
     *
     * An edge is skipped, and counted in Stats::skipped_edges, when its
     * centerline is invalid, its profile is invalid, its trims leave less than one
     * station gap to extrude, or its corridor comes back with no triangles. A
     * skipped edge never emits an empty piece.
     *
     * Raw way tags are passed to build_profile() when @p data holds the parent
     * way, looked up by GraphEdge::source_way, and nullptr otherwise.
     *
     * Calling build() again discards the previous graph and network.
     *
     * @param data Parsed OSM data with Road::node_ids populated by the parser
     * @param cfg  Pipeline tunables; the defaults are the shipping values
     * @return The extruded network. Empty pieces with zeroed stats when @p data
     *         holds no usable roads.
     */
    [[nodiscard]] RoadNetwork build(const ParsedOSMData& data, const RoadNetworkConfig& cfg = {});

    /**
     * @brief The graph the last build() produced
     *
     * Empty before the first build(). Every EdgeId in RoadPiece::edge indexes
     * this graph's edges.
     */
    [[nodiscard]] const RoadGraph& graph() const { return m_graph; }

    /**
     * @brief The resampled centerlines of the last build(), indexed by EdgeId
     *
     * Same size as graph().edges(). A skipped edge keeps an invalid, empty
     * Centerline in its slot rather than being removed, so the parallel indexing
     * holds for every EdgeId.
     *
     * Kept because the junction solver, the elevation solver and the editor
     * debug overlay all need the stations, and rebuilding them costs as much as
     * the extrusion itself.
     */
    [[nodiscard]] const std::vector<Centerline>& centerlines() const { return m_centerlines; }

    /**
     * @brief The vertical solve of the last build()
     *
     * Unsolved -- RoadElevationSolver::is_solved() false -- when
     * RoadNetworkConfig::height_sampler was null.
     */
    [[nodiscard]] const RoadElevationSolver& elevation() const { return m_elevation; }

private:
    RoadGraph m_graph;
    std::vector<Centerline> m_centerlines;
    RoadElevationSolver m_elevation;
};

} // namespace stratum::osm::road
