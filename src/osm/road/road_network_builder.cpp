/**
 * @file road_network_builder.cpp
 * @brief Whole-network road geometry, built once against the graph
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why the build is staged
 *
 * P2 could run the whole pipeline for one edge at a time, because a centerline,
 * a cross-section and a sweep depend only on that edge. P3 breaks that: every
 * arm meeting at a junction must terminate at ONE height, so the vertical solve
 * is a property of the graph rather than of an edge, and it needs every
 * centerline to exist before it can run.
 *
 * P4 breaks it again, and for the mirror-image reason: a junction is shared by
 * its arms, so how much of an edge there is to extrude is decided by the two
 * NODES at its ends and not by the edge itself.
 *
 * P5 breaks it a third time, in the other direction: sidewalk dedup decides
 * whether a profile synthesises a sidewalk at all, so it has to run BEFORE the
 * profiles are final and it needs the whole graph to run at all. Stage 1 is
 * therefore three sub-stages rather than one.
 *
 * The build is:
 *
 * @code
 *     1a. centerlines + PROVISIONAL profiles  (parallel over edges)
 *     1b. sidewalk dedup                      (global, once)
 *     1c. profiles rebuilt where dedup bit    (parallel over the masked edges)
 *     2.  RoadElevationSolver                 (global, graph-aware, once)
 *     3a. junction TRIM solve                 (global; writes the trims)
 *     3b. crossings located                   (global; READS the trims)
 *     3c. junction GEOMETRY + curb rings      (global; consumes the kerb drops)
 *     3d. junction-plane flattening           (serial over edges)
 *     4a. corridors, TRIMMED                  (parallel over edges)
 *     4b. markings, crossings, structures     (parallel over edges, same pass)
 *     4c. weld, optimise, collision, LODs     (parallel over edges, same pass)
 *     4d. junction pieces, same four passes   (parallel over junctions)
 *     5.  compaction                          (serial, ascending EdgeId)
 * @endcode
 *
 * Stage 2 is skipped entirely when RoadNetworkConfig::height_sampler is null,
 * and stage 3 when RoadNetworkConfig::solve_junctions is false, in which case
 * stage 4a sees exactly the config P2 handed it and the output is bit-identical
 * to the flat network.
 *
 * ### Why stage 3 is split around the crossings
 *
 * A dropped kerb is a BREAK in the junction's curb ring. The ring is produced by
 * a Clipper2 offset and is re-tessellated across each ramp, so a drop has to be
 * known before the ring is built rather than punched into it afterwards.
 * Crossings are what say where the drops go -- and find_crossings() reads
 * GraphEdge::trim_from and trim_to, because a crossing at a junction is pushed
 * back off its arm's trim station and is classified as a junction crossing by
 * falling inside that trim.
 *
 * So the crossings need the trims and the ring needs the crossings. The two
 * orderings that resolve that are: locate crossings early against zero trims and
 * rebuild the ring afterwards, or split the junction solve at the seam where the
 * trims are final and the geometry has not started. The seam is taken.
 * JunctionBuilder::solve_trims() finishes the trims, find_crossings() runs
 * against them, and JunctionBuilder::build_geometry() offsets each ring once with
 * the drops already in hand. Nothing is built twice, Clipper2 runs once per
 * junction, and no crossing is placed against a trim that later changes.
 *
 * ### Why P5 and P6 append rather than emit their own pieces
 *
 * Marking, crossing and structure geometry goes INTO the piece of the edge it
 * belongs to. RoadNetwork::carve_ribbons is defined as a parallel run over the
 * first `pieces - junction_pieces` entries of `pieces`, and a new class of piece
 * would silently break that contract for every existing consumer. Appending also
 * keeps a road and its paint in one spatial leaf, which is what the chunker wants
 * anyway, and it is what makes each emit_* switch reproduce the previous phase
 * exactly: with a switch off the piece list is not merely equivalent, it is the
 * same length with the same anchors in the same order.
 *
 * ### Stage 4 must use the trimmed slice, and reslice the heights with it
 *
 * Stage 3 writes GraphEdge::trim_from and trim_to; stage 4 extrudes
 * `slice(cl, trim_from, length - trim_to)`. Extruding the untrimmed centerline
 * after running the solve is strictly worse than not running it, because it adds
 * a coplanar surface inside every intersection -- see junction_builder.hpp.
 *
 * The catch is vertical. EdgeElevation::station_heights is indexed against the
 * UNTRIMMED station list, and slice() both drops stations and SYNTHESISES new
 * ones at the cut, so the two stop being parallel the moment an edge is trimmed.
 * Handing the untrimmed height vector to a trimmed centerline would be silently
 * accepted -- build_corridor() only checks the SIZE -- and would shift every
 * trimmed road's vertical profile along itself by the trim distance. The heights
 * are therefore RESAMPLED BY ARCLENGTH onto the trimmed stations, which works
 * exactly because slice() does not rebase arclength: a trimmed station's
 * arclength is still a coordinate in the untrimmed parameterisation.
 *
 * ### Why the P7 passes run inside stage 4 rather than after it
 *
 * Welding, reordering, collision derivation and LOD building are per PIECE and
 * share nothing between pieces, so they parallelise exactly as well as the
 * extrusion does and there is no reason to pay for a second sweep over the piece
 * list to run them. Stage 4c does them in the worker that built the edge piece;
 * stage 4d does the same for junction pieces, which need their own pass only
 * because a junction is not an edge and has no EdgeId slot to live in.
 *
 * They must come AFTER stage 4b, not before. Markings, zebras, decks and portals
 * append whole meshes into the piece without looking at what is already there, so
 * a weld run before them would miss everything they bring with them.
 *
 * Within the four the order is fixed: weld, then optimise, then collision, then
 * LODs. Welding first because meshopt_simplify() collapses edges and an edge only
 * exists between triangles that already share indices, so simplifying an unwelded
 * mesh does almost nothing; collision and LODs last because both are derived from
 * the finished render mesh and would otherwise be derived from a mesh the
 * renderer never draws.
 *
 * ### Why parallelism cannot change the output
 *
 * Workers never push. Each one writes into a pre-sized slot indexed by EdgeId,
 * and the main thread compacts the slots in ascending EdgeId order afterwards.
 * The emitted piece list, the carve payload, and the stats are identical
 * whatever the scheduling, so a build is reproducible and the golden-file tests
 * in P7 stay meaningful.
 */

#include "osm/road/road_network_builder.hpp"

#include <TaskScheduler.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace stratum::osm::road {

namespace {

/**
 * @brief Edge count below which a stage runs on the calling thread
 *
 * Spinning up a task scheduler costs one thread creation per hardware thread.
 * On a small extract -- and on every test fixture -- that dominates the work it
 * would distribute, so small networks stay serial. The results are identical
 * either way; only the wall clock differs.
 */
constexpr size_t kParallelMinEdges = 256;

/**
 * @brief Edges handed to one worker at a time
 *
 * One edge is a few tens of microseconds of work, so a range of one would spend
 * more time in the scheduler than in the extruder. Ranges of this size amortise
 * that while still leaving enough of them to balance across threads.
 */
constexpr uint32_t kEdgesPerRange = 32;

/**
 * @brief Fallback carve radius for a junction whose arms report no width, metres
 *
 * build_profile() is documented to return a valid profile for any edge, so this
 * is unreachable in practice. It exists so a CarveDisc always has a positive
 * radius and the terrain carve never has to defend against a zero-area
 * footprint.
 */
constexpr float kMinDiscRadius = 1.0f;

/// Fallback carve band radius for a ribbon whose profile reports no width, metres
constexpr float kMinRibbonHalfWidth = 1.0f;

/**
 * @brief Position of the station nearest the middle of a centerline's arclength
 *
 * This is the piece's spatial anchor, and it must be a point ON the road. The
 * average of the raw polyline points is not: on an L-shaped edge it lands off
 * the corner, in open space, and the spatial index would file the piece under a
 * leaf the road never touches.
 *
 * @param cl Centerline to sample; must be valid
 * @return The station position closest to half the arclength span
 */
[[nodiscard]] glm::dvec2 mid_arclength_position(const Centerline& cl) {
    const double first = cl.stations.front().arclength;
    const double last = cl.stations.back().arclength;
    const double middle = 0.5 * (first + last);

    size_t best = 0;
    double best_delta = std::numeric_limits<double>::max();
    for (size_t i = 0; i < cl.stations.size(); ++i) {
        const double delta = std::fabs(cl.stations[i].arclength - middle);
        if (delta < best_delta) {
            best_delta = delta;
            best = i;
        }
    }
    return cl.stations[best].position;
}

/**
 * @brief Raw tags of an edge's parent way, or nullptr when the way is gone
 *
 * A missing parent way is not an error. The graph is built from Road, which
 * outlives the raw way map in some import paths, and build_profile falls back to
 * the fields already promoted onto the edge.
 *
 * @param data Parsed data holding the way map
 * @param id   Way the edge was split out of
 * @return Pointer into @p data, valid for the lifetime of the build; nullptr
 *         when @p data holds no such way
 */
[[nodiscard]] const TagMap* way_tags(const ParsedOSMData& data, WayId id) {
    const auto way = data.ways.find(id);
    return way != data.ways.end() ? &way->second.tags : nullptr;
}

/**
 * @brief Run @p fn over [0, count) as sub-ranges, in parallel when it pays
 *
 * The scheduler is scoped to one stage rather than to the whole build, because
 * RoadElevationSolver::solve() sits between the two parallel stages and spins up
 * its own. Holding one open across the solve would double the worker threads for
 * the duration of the solve and gain nothing.
 *
 * @param count Number of edges
 * @param fn    Callable taking a half-open [begin, end) range of EdgeIds
 */
template <typename Fn>
void run_edge_ranges(size_t count, Fn&& fn) {
    if (count == 0) {
        return;
    }
    if (count < kParallelMinEdges) {
        fn(size_t{0}, count);
        return;
    }

    enki::TaskScheduler scheduler;
    scheduler.Initialize();

    enki::TaskSet sweep(static_cast<uint32_t>(count),
                        [&](enki::TaskSetPartition range, uint32_t /*threadnum*/) {
                            fn(static_cast<size_t>(range.start), static_cast<size_t>(range.end));
                        });
    sweep.m_MinRange = kEdgesPerRange;

    scheduler.AddTaskSetToPipe(&sweep);
    scheduler.WaitforTask(&sweep);
}

/**
 * @brief Resample per-station heights onto a trimmed centerline, by arclength
 *
 * EdgeElevation::station_heights is parallel to the UNTRIMMED station list.
 * slice() drops stations and synthesises new ones at the cut, so the two stop
 * being parallel the moment an edge is trimmed, and build_corridor() checks only
 * the SIZE of the vector it is given -- a mismatched-but-same-size vector is
 * accepted silently and shifts the whole vertical profile along the road.
 *
 * The mapping is exact rather than approximate because slice() does NOT rebase
 * arclength: every station of @p cut carries its arclength in @p source's own
 * parameterisation, so the height at a cut station is simply the source profile
 * evaluated there. Endpoints synthesised between two source stations interpolate
 * linearly, which is what the extruder does between the same two columns anyway.
 *
 * A bevel pair -- two stations sharing one arclength -- resolves to the same
 * height for both, which is correct: a bevel is a zero-length band and cannot
 * climb.
 *
 * @param source  Untrimmed centerline the heights were solved against
 * @param heights Per-station heights, parallel to @p source
 * @param cut     Trimmed centerline to resample onto
 * @return Heights parallel to @p cut, or empty when @p heights is not parallel
 *         to @p source
 */
[[nodiscard]] std::vector<float> reslice_station_heights(const Centerline& source,
                                                         const std::vector<float>& heights,
                                                         const Centerline& cut) {
    std::vector<float> out;
    if (heights.empty() || heights.size() != source.stations.size()) {
        return out;
    }

    out.reserve(cut.stations.size());

    // Monotone walk: cut arclengths are non-decreasing and lie inside the source
    // range, so the bracket only ever advances.
    size_t k = 1;
    for (const Station& station : cut.stations) {
        const double at = station.arclength;
        while (k < source.stations.size() && source.stations[k].arclength < at) {
            ++k;
        }
        if (k >= source.stations.size()) {
            out.push_back(heights.back());
            continue;
        }

        const double lo = source.stations[k - 1].arclength;
        const double hi = source.stations[k].arclength;
        const double span = hi - lo;
        if (!(std::isfinite(span) && span > 0.0)) {
            out.push_back(heights[k]);
            continue;
        }

        double t = (at - lo) / span;
        t = std::min(std::max(t, 0.0), 1.0);
        out.push_back(static_cast<float>(static_cast<double>(heights[k - 1]) +
                                         t * (static_cast<double>(heights[k]) -
                                              static_cast<double>(heights[k - 1]))));
    }
    return out;
}

/**
 * @brief Longitudinal datum for each edge's dash pattern
 *
 * A street is split into a new GraphEdge wherever an OSM way ends, and a way
 * ends at every `name`, `ref`, `maxspeed` or `source` change. Such a split lands
 * on a plain degree-2 node: the two profiles agree, so no taper is built, no trim
 * is written, and the two ribbons meet flush. The paint does not, because
 * emit_dashed_line() indexes the pattern from each edge's own arclength zero. The
 * joint then reads as a double-length dash or a short gap, depending on where the
 * first edge's length fell in the cycle -- uniformly distributed, so roughly a
 * third of such splits are visibly wrong.
 *
 * The datum here is the arc length of the street already travelled before an
 * edge's own zero, accumulated along chains of HEAD-TO-TAIL degree-2
 * continuations: the previous edge ends at the node and this one starts there, so
 * the two share a direction of travel and the pattern simply continues. A node
 * where two edges meet head to head or tail to tail is left as a chain break; the
 * pattern restarts there, which is no worse than today and needs no reversed
 * phase convention.
 *
 * Lengths come from the CENTERLINES rather than from the raw polylines, because
 * that is the parameterisation the dashes are laid out in -- resampling and
 * corner bevels move the two apart by centimetres per corner.
 *
 * @param graph       Built road graph
 * @param centerlines Parallel to graph.edges()
 * @return One phase per edge, in metres, zero at every chain head
 */
[[nodiscard]] std::vector<double> dash_phases(const RoadGraph& graph,
                                              const std::vector<Centerline>& centerlines) {
    const size_t count = graph.edges().size();
    std::vector<double> phase(count, 0.0);
    if (count == 0 || centerlines.size() != count) {
        return phase;
    }

    std::vector<EdgeId> next(count, kInvalidId);
    std::vector<EdgeId> previous(count, kInvalidId);

    for (const GraphNode& node : graph.nodes()) {
        if (node.arms.size() != 2) {
            continue;
        }
        const Arm& a = node.arms[0];
        const Arm& b = node.arms[1];
        if (a.edge >= count || b.edge >= count || a.edge == b.edge) {
            continue;
        }
        // Exactly one arriving and one leaving, or the two do not continue.
        const Arm* in = nullptr;
        const Arm* out = nullptr;
        if (!a.at_start && b.at_start) {
            in = &a;
            out = &b;
        } else if (!b.at_start && a.at_start) {
            in = &b;
            out = &a;
        } else {
            continue;
        }
        // Each is written by exactly one node -- an edge has one `to` and one
        // `from` -- so neither can be claimed twice.
        next[in->edge] = out->edge;
        previous[out->edge] = in->edge;
    }

    const auto length_of = [&](EdgeId e) {
        const Centerline& cl = centerlines[e];
        return cl.is_valid() ? cl.length() : graph.edge(e).length();
    };

    std::vector<uint8_t> done(count, 0u);

    /// Walk one chain from @p head, accumulating arc length into the phase
    const auto walk = [&](EdgeId head) {
        double travelled = 0.0;
        EdgeId e = head;
        while (e != kInvalidId && e < count && done[e] == 0u) {
            done[e] = 1u;
            phase[e] = travelled;
            const double len = length_of(e);
            travelled += std::isfinite(len) && len > 0.0 ? len : 0.0;
            e = next[e];
        }
    };

    for (size_t i = 0; i < count; ++i) {
        if (done[i] == 0u && previous[i] == kInvalidId) {
            walk(static_cast<EdgeId>(i));
        }
    }
    // Whatever is left is a closed ring -- a roundabout, or a loop road -- with
    // no head to start from. Starting at the lowest edge id keeps it reproducible.
    for (size_t i = 0; i < count; ++i) {
        if (done[i] == 0u) {
            walk(static_cast<EdgeId>(i));
        }
    }

    return phase;
}

/**
 * @brief Interpolate a per-station height vector at one arclength
 *
 * The same frame reslice_station_heights() works in, sampled at a single point
 * rather than at a whole station list. A mis-sized vector returns @p fallback,
 * matching CorridorConfig::station_heights, so a caller never gets a height
 * derived from a vector that does not belong to this centerline.
 *
 * @param cl        Centerline @p heights is indexed against
 * @param heights   One entry per station of @p cl
 * @param arclength Position along @p cl, clamped to its ends
 * @param fallback  Returned when @p heights does not match @p cl
 * @return World Y at @p arclength
 */
[[nodiscard]] float sample_station_height(const Centerline& cl,
                                          const std::vector<float>& heights,
                                          double arclength,
                                          float fallback) {
    if (heights.empty() || heights.size() != cl.stations.size()) {
        return fallback;
    }
    const std::vector<Station>& st = cl.stations;
    if (!(arclength > st.front().arclength)) {
        return heights.front();
    }
    if (!(arclength < st.back().arclength)) {
        return heights.back();
    }

    const auto it = std::lower_bound(st.begin(), st.end(), arclength,
                                     [](const Station& s, double value) {
                                         return s.arclength < value;
                                     });
    size_t hi = static_cast<size_t>(it - st.begin());
    if (hi == 0) {
        hi = 1;
    }
    if (hi >= st.size()) {
        return heights.back();
    }

    const double lo_s = st[hi - 1].arclength;
    const double hi_s = st[hi].arclength;
    const double span = hi_s - lo_s;
    if (!(std::isfinite(span) && span > 0.0)) {
        return heights[hi];
    }
    double t = (arclength - lo_s) / span;
    t = std::min(std::max(t, 0.0), 1.0);
    return static_cast<float>(static_cast<double>(heights[hi - 1]) +
                              t * (static_cast<double>(heights[hi]) -
                                   static_cast<double>(heights[hi - 1])));
}

/**
 * @brief Append @p src into @p dst keeping every one of its material ranges
 *
 * Mesh::append() forces the whole appended mesh under ONE MaterialId, which is
 * wrong for anything P5 or P6 produces: a bridge carries BridgeDeck, Parapet and
 * Concrete at once, and collapsing them loses the material slots the whole
 * submesh design exists to give an exporter.
 *
 * The destination's implicit whole-mesh range is materialised first. Without
 * that, geometry appended to a mesh with no explicit submeshes is absorbed into
 * the implicit range and silently takes the destination's material.
 *
 * @param dst Mesh appended to, in place
 * @param src Mesh to copy in; a no-op when it holds no geometry
 */
void append_keeping_materials(Mesh& dst, const Mesh& src) {
    if (src.vertices.empty() || src.indices.empty()) {
        return;
    }

    if (!dst.indices.empty() && dst.submeshes.empty()) {
        dst.submeshes.push_back(
            SubMesh{0u, static_cast<uint32_t>(dst.indices.size()), MaterialId::Default});
    }

    const uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (const Vertex& v : src.vertices) {
        dst.bounds.expand(v.position);
    }

    for (const SubMesh& range : src.effective_submeshes()) {
        const size_t begin = range.index_offset;
        const size_t end = std::min<size_t>(begin + range.index_count, src.indices.size());
        if (begin >= end) {
            continue;
        }

        const uint32_t count = static_cast<uint32_t>(end - begin);
        if (!dst.submeshes.empty() && dst.submeshes.back().material == range.material) {
            dst.submeshes.back().index_count += count;
        } else {
            dst.submeshes.push_back(
                SubMesh{static_cast<uint32_t>(dst.indices.size()), count, range.material});
        }
        for (size_t k = begin; k < end; ++k) {
            dst.indices.push_back(src.indices[k] + base);
        }
    }
}

/**
 * @brief How many sides a suppression mask names
 *
 * DedupResult counts EDGES; Stats::deduped_sidewalks counts SIDES, because that
 * is the number that says how much sidewalk actually stopped being built. An
 * edge suppressed on both sides adds two.
 *
 * @param side Mask from DedupResult::suppress_side
 * @return 0, 1 or 2
 */
[[nodiscard]] size_t suppressed_side_count(SideFlags side) {
    switch (side) {
        case SideFlags::Left:
        case SideFlags::Right:
            return 1;
        case SideFlags::Both:
            return 2;
        case SideFlags::None:
        case SideFlags::Unknown:
            break;
    }
    return 0;
}

/**
 * @brief What the P7 stage did to one piece
 *
 * Accumulated per piece by the worker that built it, then summed on the calling
 * thread during compaction. Nothing here is shared between workers, which is what
 * keeps the stage's counters independent of how the ranges were scheduled.
 */
struct PieceFinishStats {
    size_t vertices_welded = 0;      ///< Vertices removed by the weld
    size_t vertices_dropped = 0;     ///< Unreferenced vertices dropped by the reorder
    size_t triangles_before_lod = 0; ///< Triangles of LodChain::levels front
    size_t triangles_after_lod = 0;  ///< Triangles of LodChain::levels back
    size_t collision_triangles = 0;  ///< Triangles of RoadPiece::collision
};

/**
 * @brief Run the P7 passes over one finished piece, in place
 *
 * The order is load-bearing and is the reason this lives in one function rather
 * than four call sites:
 *
 * 1. **Weld.** First, because everything after it is cheaper on fewer vertices and
 *    because meshopt_simplify() collapses edges, and an edge only exists between
 *    triangles that already share indices. Reordering or simplifying an unwelded
 *    mesh is close to a no-op.
 * 2. **Optimise.** Reordering is per SubMesh range and changes no geometry, so it
 *    is safe to run before the two derivations and it means both of them start
 *    from the same buffer the renderer will draw.
 * 3. **Collision.** Derived from the finished render mesh, so the two can never
 *    disagree about where the road is.
 * 4. **LODs.** Last and by far the most expensive: one meshopt_simplify() per
 *    material range per level. `levels[0]` is defined as the finished render mesh,
 *    which only holds because the weld and the reorder already ran.
 *
 * Every step is skipped when its RoadNetworkConfig switch is off, and a piece with
 * all four off comes out byte for byte as the corridor and the detail passes left
 * it.
 *
 * @param piece Piece to finish, modified in place
 * @param cfg   The four P7 switches and their tolerances
 * @param out   Counters for this piece, overwritten
 */
void finish_piece(RoadPiece& piece, const RoadNetworkConfig& cfg, PieceFinishStats& out) {
    out = PieceFinishStats{};

    if (piece.mesh.vertices.empty() || piece.mesh.indices.empty()) {
        return;
    }

    if (cfg.weld_meshes) {
        out.vertices_welded = weld_vertices(piece.mesh, cfg.weld);
    }

    if (cfg.optimize_meshes) {
        // The fetch pass compacts the vertex array, so a vertex no triangle
        // references is dropped here. That reduction is real and is NOT part of
        // the weld, so it is counted separately: welded + dropped is then the
        // whole of what the P7 stage removed, and the two together account for
        // every vertex the upstream passes emitted.
        const size_t before_optimize = piece.mesh.vertices.size();
        optimize_mesh(piece.mesh, cfg.lod);
        const size_t after_optimize = piece.mesh.vertices.size();
        out.vertices_dropped = (before_optimize > after_optimize)
                                   ? (before_optimize - after_optimize)
                                   : 0;
    }

    if (cfg.build_collision) {
        piece.collision = build_collision_mesh(piece.mesh, cfg.collision);
        out.collision_triangles = piece.collision.indices.size() / 3;
    }

    if (cfg.build_lods) {
        piece.lods = build_lod_chain(piece.mesh, cfg.lod);
        if (piece.lods.is_valid()) {
            out.triangles_before_lod = piece.lods.levels.front().indices.size() / 3;
            out.triangles_after_lod = piece.lods.levels.back().indices.size() / 3;
        }
    }
}

/**
 * @brief One edge's stage-4 output slot
 *
 * Written by exactly one worker and never resized, so the workers share nothing
 * and the compaction order is the EdgeId order. The CarveRibbon is built HERE
 * rather than during compaction because it has to describe the TRIMMED geometry,
 * and the trimmed centerline and its resliced heights are local to the worker.
 */
struct EdgeSlot {
    RoadPiece piece;                ///< Geometry and footprint; untouched when filled is false
    CarveRibbon ribbon;             ///< Carve payload; valid only when has_ribbon
    bool filled = false;            ///< Edge produced geometry
    bool has_ribbon = false;        ///< Network was built on terrain and a ribbon was emitted
    bool elevated = false;          ///< Heights came from the elevation solve
    bool trimmed = false;           ///< Extruded from a trimmed centerline
    bool trimmed_away = false;      ///< The trims left too little to extrude
    bool consumed = false;          ///< Replaced wholesale by a roundabout annulus

    // ── P5 and P6, appended into `piece` by the same worker ──────────────
    bool painted = false;           ///< Marking geometry was appended
    bool bridged = false;           ///< Bridge structure was appended
    bool tunnelled = false;         ///< At least one tunnel portal was appended

    /**
     * @brief Portal mouths this edge asked the terrain to open
     *
     * Filled only when a portal was actually built, so the carve never opens a
     * mouth where no headwall stands. Start end first, matching the order
     * build_tunnel_portals() emits the geometry in.
     */
    std::vector<TunnelPortalFootprint> portals;
    size_t crossings = 0;           ///< Crossings whose zebra was appended

    /// P7 counters for this piece; every field stays zero when its switch is off
    PieceFinishStats finish;
};

/**
 * @brief One solved junction's piece slot
 *
 * The junction analogue of EdgeSlot, and it exists for the same reason: the P7
 * passes are the expensive part of turning a Junction into a RoadPiece, and a
 * junction mesh is the one most likely to have something to weld. Its curb ring
 * and its fill are merged from separate builders that never see each other's
 * vertices, so the seam between them is duplicated all the way round.
 *
 * Indexed by position in RoadNetwork::junctions and written by exactly one worker,
 * so the compaction order is the junction order and stays independent of the
 * scheduling.
 */
struct JunctionSlot {
    RoadPiece piece;            ///< Geometry and footprint; untouched when filled is false
    PieceFinishStats finish;    ///< P7 counters for this piece
    bool filled = false;        ///< Junction produced a usable piece
};

} // namespace

// ============================================================================
// Builder
// ============================================================================

RoadNetwork RoadNetworkBuilder::build(const ParsedOSMData& data, const RoadNetworkConfig& cfg) {
    const auto started = std::chrono::steady_clock::now();

    RoadNetwork network;

    m_centerlines.clear();
    m_elevation.clear();

    m_graph.build(data);
    const size_t edge_count = m_graph.edges().size();
    network.stats.edges = edge_count;

    if (edge_count == 0) {
        const auto finished = std::chrono::steady_clock::now();
        network.stats.build_ms =
            std::chrono::duration<double, std::milli>(finished - started).count();
        spdlog::info("RoadNetworkBuilder: Built network — 0 pieces from 0 edges in {:.1f} ms",
                     network.stats.build_ms);
        return network;
    }

    // ── Stage 1a: centerlines and PROVISIONAL cross-sections ─────────────────
    // Both depend only on the edge and the immutable config, so the stage is
    // embarrassingly parallel. Profiles are built for EVERY edge, including one
    // whose centerline welded down to a point, because a junction takes its carve
    // radius from the widest arm meeting at it and an arm that emitted no
    // geometry still occupies the road.
    //
    // These profiles are provisional in exactly one respect: their synthesised
    // sidewalks are what stage 1b measures against, and stage 1c may take some of
    // them away again.
    m_centerlines.assign(edge_count, Centerline{});
    std::vector<RoadProfile> profiles(edge_count);

    run_edge_ranges(edge_count, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            const GraphEdge& edge = m_graph.edge(static_cast<EdgeId>(i));
            m_centerlines[i] = build_centerline(edge.polyline, cfg.resample);
            profiles[i] = build_profile(edge, cfg.profile, way_tags(data, edge.source_way));
        }
    });

    // The dash datum, which only needs the graph and the centerline lengths.
    // Held for stage 4b; see dash_phases().
    const std::vector<double> dash_phase = dash_phases(m_graph, m_centerlines);

    // ── Stage 1b: sidewalk dedup ─────────────────────────────────────────────
    // Global, and it runs HERE rather than anywhere later because it changes the
    // cross-section. Everything downstream -- the elevation solve, the trims, the
    // junction polygons, the corridor outlines, the terrain carve -- is computed
    // from profile widths, so a dedup that ran after any of them would leave a
    // junction solved for a road that is no longer that wide.
    //
    // Called unconditionally: dedup_sidewalks() honours DedupConfig::enabled
    // itself and returns a correctly sized, all-None mask when it is off, so the
    // mask can be indexed without a second branch and the disabled path costs one
    // vector fill.
    const DedupResult dedup = dedup_sidewalks(m_graph, m_centerlines, profiles, cfg.dedup);

    // ── Stage 1c: final cross-sections ───────────────────────────────────────
    // Only the masked edges are rebuilt. Deterministic because the mask is a pure
    // function of the graph and the provisional profiles, and because each worker
    // writes only its own slot.
    std::vector<size_t> masked_edges;
    if (dedup.suppress_side.size() == edge_count) {
        for (size_t i = 0; i < edge_count; ++i) {
            const size_t sides = suppressed_side_count(dedup.suppress_side[i]);
            if (sides > 0) {
                masked_edges.push_back(i);
                network.stats.deduped_sidewalks += sides;
            }
        }
    }

    if (!masked_edges.empty()) {
        run_edge_ranges(masked_edges.size(), [&](size_t begin, size_t end) {
            for (size_t k = begin; k < end; ++k) {
                const size_t i = masked_edges[k];
                const GraphEdge& edge = m_graph.edge(static_cast<EdgeId>(i));
                profiles[i] = build_profile(edge, cfg.profile, way_tags(data, edge.source_way),
                                            dedup.suppress_side[i]);
            }
        });
    }

    // ── Stage 2: the vertical solve ──────────────────────────────────────────
    // Global and graph-aware: node heights are shared by their arms, so this
    // cannot be folded into either of the per-edge stages. It runs after every
    // centerline exists and before any corridor is extruded.
    //
    // Skipped entirely when no terrain was supplied, which is what keeps the flat
    // P2 path bit-identical.
    if (cfg.height_sampler) {
        const auto solve_started = std::chrono::steady_clock::now();
        m_elevation.solve(m_graph, m_centerlines, cfg.height_sampler, cfg.elevation);
        const auto solve_finished = std::chrono::steady_clock::now();
        network.stats.elevation_ms =
            std::chrono::duration<double, std::milli>(solve_finished - solve_started).count();
    }

    const bool on_terrain = m_elevation.is_solved()
                            && m_elevation.edges().size() == edge_count;

    /// True when the solve produced a height per station of this edge
    const auto has_solved_heights = [&](size_t i) {
        return on_terrain
               && m_elevation.edge(static_cast<EdgeId>(i)).station_heights.size()
                      == m_centerlines[i].stations.size()
               && !m_centerlines[i].stations.empty();
    };
    // ── Stage 3: the junction solve, split around the crossings ──────────────
    // Global and graph-aware, like the vertical solve and for the mirror-image
    // reason: how much of an edge there is to extrude is decided by the two NODES
    // at its ends. It runs after the elevation solve, because a junction is
    // placed at its solved node height, and before any corridor is extruded,
    // because it writes the trims stage 4 cuts at.
    //
    // Skipped entirely when cfg.solve_junctions is false, which leaves every trim
    // zero and reproduces the P2/P3 output exactly.
    JunctionBuilder junction_builder;
    JunctionBuilder::Stats junction_stats;
    std::vector<bool> consumed_edges;

    /// Every crossing on the network, in ascending (edge, arclength) order
    std::vector<Crossing> crossings;

    /// The trim solve ran, so the crossings were located at its seam
    bool trims_solved = false;

    if (cfg.solve_junctions) {
        const auto junction_started = std::chrono::steady_clock::now();

        // JunctionConfig cannot reach ElevationConfig or CorridorConfig -- the
        // solve is handed the SOLVED elevation, not the configuration that
        // produced it -- so the two vertical constants are mirrored across here.
        // node_height() excludes surface_offset by design; this is the one place
        // the junction path adds it.
        JunctionConfig junction_cfg = cfg.junction;
        junction_cfg.surface_offset = cfg.elevation.surface_offset;
        junction_cfg.base_height = cfg.corridor.base_height;

        // ── Stage 3a: trims ──────────────────────────────────────────────
        trims_solved = junction_builder.solve_trims(m_graph, m_centerlines, profiles, m_elevation,
                                                    junction_cfg);
        if (trims_solved) {
            // ── Stage 3b: crossings, against the FINAL trims ─────────────
            // Located here and nowhere else. Earlier and every trim is still
            // zero, so a junction crossing is neither recognised as one nor set
            // back off the arm mouth; later and the curb ring has already been
            // offset and cannot be broken without rebuilding it.
            if (cfg.emit_crossings) {
                crossings =
                    find_crossings(m_graph, data, m_centerlines, profiles, m_elevation,
                                   cfg.crossings);
            }

            // ── Stage 3c: geometry, with the kerb drops in hand ──────────
            // The provider is called once per junction that builds a ring, from
            // the solver's own worker threads. Both span queries are pure
            // functions over vectors that are final by now, so there is nothing
            // to synchronise.
            //
            // The span count is atomic for the same reason: the provider runs on
            // those workers, so a plain size_t would race even though everything
            // it reads is const by now.
            std::atomic<size_t> demanded_drops{0};
            JunctionBuilder::KerbDropProvider kerb_drops = nullptr;
            if (cfg.emit_crossings && cfg.crossings.emit_dropped_kerbs) {
                kerb_drops = [&](GraphNodeId node, glm::dvec2 center) {
                    KerbDrops drops;
                    drops.center = center;
                    drops.ramp_length = static_cast<double>(cfg.crossings.dropped_kerb_ramp);
                    drops.spans = dropped_kerb_spans(crossings, node, center, cfg.crossings);

                    // A driveway mouth needs the same flare and the two sources
                    // are documented to concatenate without merging: where they
                    // overlap the deeper drop wins inside build_curb_ring().
                    const std::vector<DroppedKerbSpan> driveways = driveway_kerb_spans(
                        m_graph, data, node, center, profiles, cfg.crossings);
                    drops.spans.insert(drops.spans.end(), driveways.begin(), driveways.end());
                    demanded_drops.fetch_add(drops.spans.size(), std::memory_order_relaxed);
                    return drops;
                };
            }

            network.junctions = junction_builder.build_geometry(m_graph, m_centerlines, profiles,
                                                                kerb_drops);
            consumed_edges = junction_builder.consumed_edges();

            junction_stats = junction_builder.stats();
            network.junction_stats = junction_stats;
            network.stats.dropped_kerb_spans = demanded_drops.load(std::memory_order_relaxed);
        }

        const auto junction_finished = std::chrono::steady_clock::now();
        network.stats.junction_ms =
            std::chrono::duration<double, std::milli>(junction_finished - junction_started).count();
    }

    // With no junction solve there is no seam to locate crossings at, so they are
    // located here instead, against the zero trims that leaves. Every crossing is
    // then mid-block, which is exactly right: with no solve there is no junction
    // polygon for one to be set back from, and no curb ring for one to break.
    if (cfg.emit_crossings && !trims_solved) {
        crossings = find_crossings(m_graph, data, m_centerlines, profiles, m_elevation,
                                   cfg.crossings);
    }

    // ── Crossings, indexed by the edge they sit on ───────────────────────────
    // Built serially, in the order find_crossings() returned them, so stage 4b
    // appends each edge's zebras in a fixed order however the ranges are
    // scheduled.
    std::vector<std::vector<const Crossing*>> crossings_on_edge(edge_count);
    for (const Crossing& crossing : crossings) {
        if (crossing.edge != kInvalidId && static_cast<size_t>(crossing.edge) < edge_count) {
            crossings_on_edge[crossing.edge].push_back(&crossing);
        }
    }

    // ── Junction height and signalling, indexed by node ──────────────────────
    // Read by the approach markings. Taken from the SOLVED junctions rather than
    // recomputed from the elevation solver: a junction that came back degenerate
    // has no plane to paint a stop line on, and Junction::height already carries
    // ElevationConfig::surface_offset exactly once.
    std::vector<float> junction_height(m_graph.nodes().size(), 0.0f);
    std::vector<bool> node_has_junction(m_graph.nodes().size(), false);
    for (const Junction& junction : network.junctions) {
        if (!junction.valid || junction.kind != JunctionKind::Intersection) {
            continue;
        }
        if (junction.node == kInvalidId || junction.node >= junction_height.size()) {
            continue;
        }
        junction_height[junction.node] = junction.height;
        node_has_junction[junction.node] = true;
    }

    /// A valid roundabout annulus replaced this edge's ribbon wholesale
    const auto is_consumed = [&](size_t i) {
        return i < consumed_edges.size() && consumed_edges[i];
    };

    // ── Stage 3b: junction plateaus ──────────────────────────────────────────
    // The vertical solve pinned every edge's FIRST station to its node height,
    // because before P4 that station sat on the node. The trims have just moved
    // it `trim` metres away, where the solved profile is `grade * trim` higher or
    // lower, while the junction fill and its curb ring are one flat plane at the
    // node height. On an 8% hillside with the shipping trims that is a 30 cm open
    // step at every arm mouth, with nothing bridging it.
    //
    // Flattening each edge's solved heights over its own trim gives every
    // junction a plateau exactly as wide as the cut, so the resliced end station
    // lands back on the junction plane. That is also what a junction is in the
    // ground: a level landing with the grade change pushed outside it. The carve
    // follows automatically, because CarveRibbon::centerline_heights is derived
    // from these same values.
    //
    // Held separately rather than written back into the solver: EdgeElevation is
    // the answer to "what did the vertical solve decide", and the plateau is a
    // consequence of the junction solve that ran after it. The rule itself lives
    // in apply_junction_plateaus(), including what happens on an edge too short
    // for the two plateaus to both fit.
    std::vector<std::vector<float>> plateau_heights(edge_count);

    if (on_terrain && cfg.solve_junctions) {
        const float offset =
            std::isfinite(cfg.elevation.surface_offset) ? cfg.elevation.surface_offset : 0.0f;

        for (size_t i = 0; i < edge_count; ++i) {
            if (!has_solved_heights(i)) {
                continue;
            }
            const GraphEdge& edge = m_graph.edge(static_cast<EdgeId>(i));
            const double trim_from = std::isfinite(edge.trim_from) ? edge.trim_from : 0.0;
            const double trim_to = std::isfinite(edge.trim_to) ? edge.trim_to : 0.0;
            if (!(trim_from > 0.0) && !(trim_to > 0.0)) {
                continue;
            }

            std::vector<float> heights = m_elevation.edge(static_cast<EdgeId>(i)).station_heights;
            const Centerline& cl = m_centerlines[i];

            // Both ends in one call, because on a short edge their ranges meet
            // and the resolution is not "whichever was written last": see
            // apply_junction_plateaus().
            const bool have_from = edge.from != kInvalidId && edge.from < m_graph.nodes().size();
            const bool have_to = edge.to != kInvalidId && edge.to < m_graph.nodes().size();
            apply_junction_plateaus(
                cl, have_from ? trim_from : 0.0, have_to ? trim_to : 0.0,
                have_from ? m_elevation.node_height(edge.from) + offset : 0.0f,
                have_to ? m_elevation.node_height(edge.to) + offset : 0.0f, heights);
            plateau_heights[i] = std::move(heights);
        }
    }

    // ── Stage 4: corridors, from the TRIMMED centerline ──────────────────────
    std::vector<EdgeSlot> slots(edge_count);

    run_edge_ranges(edge_count, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            EdgeSlot& slot = slots[i];

            if (is_consumed(i)) {
                slot.consumed = true;
                continue;
            }

            const Centerline& untrimmed = m_centerlines[i];
            if (!untrimmed.is_valid()) {
                continue;
            }

            const RoadProfile& profile = profiles[i];
            if (!profile.is_valid()) {
                continue;
            }

            const GraphEdge& edge = m_graph.edge(static_cast<EdgeId>(i));
            const double trim_from = std::isfinite(edge.trim_from) && edge.trim_from > 0.0
                                         ? edge.trim_from
                                         : 0.0;
            const double trim_to =
                std::isfinite(edge.trim_to) && edge.trim_to > 0.0 ? edge.trim_to : 0.0;
            const bool trimmed = trim_from > 0.0 || trim_to > 0.0;

            // Held by value only when a cut is actually needed, so the untrimmed
            // path copies nothing and stays bit-identical to P2/P3.
            Centerline cut;
            if (trimmed) {
                const double first = untrimmed.stations.front().arclength;
                cut = slice(untrimmed, first + trim_from, untrimmed.length() - trim_to);
                if (!cut.is_valid()) {
                    slot.trimmed_away = true;
                    continue;
                }
            }
            const Centerline& centerline = trimmed ? cut : untrimmed;

            // Copied per edge rather than shared, because station_heights is the
            // one field that differs between them.
            CorridorConfig corridor_cfg = cfg.corridor;
            bool elevated = false;
            if (has_solved_heights(i)) {
                const std::vector<float>& solved =
                    !plateau_heights[i].empty()
                        ? plateau_heights[i]
                        : m_elevation.edge(static_cast<EdgeId>(i)).station_heights;
                if (!trimmed) {
                    corridor_cfg.station_heights = solved;
                    elevated = true;
                } else {
                    // The solved heights are indexed against the UNTRIMMED
                    // stations. Handing them to a trimmed ribbon would be accepted
                    // on size alone and would slide the vertical profile along the
                    // road by the trim distance.
                    corridor_cfg.station_heights =
                        reslice_station_heights(untrimmed, solved, centerline);
                    elevated = corridor_cfg.station_heights.size() == centerline.stations.size();
                    if (!elevated) {
                        corridor_cfg.station_heights.clear();
                    }
                }
            }

            Corridor corridor = build_corridor(centerline, profile, corridor_cfg);
            if (corridor.mesh.vertices.empty() || corridor.mesh.indices.empty()) {
                continue;
            }

            slot.piece.anchor = mid_arclength_position(centerline);
            slot.piece.edge = static_cast<EdgeId>(i);
            slot.piece.mesh = std::move(corridor.mesh);
            slot.piece.outline = std::move(corridor.outline);
            slot.trimmed = trimmed;
            slot.elevated = elevated;
            slot.filled = true;

            // ── Stage 4b: markings, crossings and structures ─────────────────
            // Appended into this edge's own piece by this edge's own worker, so
            // the parallelism stays slot-local and the result stays reproducible.
            // Nothing here emits a piece; see the file header for why.
            {
                const TagMap* tags = way_tags(data, edge.source_way);
                bool appended = false;

                // Paint sits on the UNTRIMMED centerline, in the untrimmed
                // parameterisation the trims are expressed in. Its heights are
                // the ones the corridor was placed at, in that same frame: the
                // plateaued solve when the road is elevated, and the flat plane
                // otherwise. Handing it nothing would drop every line to world Y
                // zero and thread it through the road.
                std::vector<float> paint_heights;
                if (elevated) {
                    paint_heights = !plateau_heights[i].empty()
                                        ? plateau_heights[i]
                                        : m_elevation.edge(static_cast<EdgeId>(i)).station_heights;
                } else {
                    paint_heights.assign(untrimmed.stations.size(), cfg.corridor.base_height);
                }

                if (cfg.emit_markings) {
                    const Mesh lines = build_edge_markings(edge, untrimmed, profile,
                                                           paint_heights, cfg.markings, tags,
                                                           dash_phase[i]);
                    if (!lines.indices.empty()) {
                        append_keeping_materials(slot.piece.mesh, lines);
                        slot.painted = true;
                        appended = true;
                    }

                    // An approach is painted only where a junction was actually
                    // solved. A degenerate node has no plane to set a stop line
                    // back from and no fillet to set it back off.
                    const GraphNodeId ends[2] = {edge.from, edge.to};
                    for (int k = 0; k < 2; ++k) {
                        const GraphNodeId node = ends[k];
                        if (node == kInvalidId || node >= node_has_junction.size() ||
                            !node_has_junction[node]) {
                            continue;
                        }
                        const Mesh approach = build_approach_markings(
                            edge, untrimmed, profile, /*at_start=*/k == 0,
                            m_graph.nodes()[node].has_signals, junction_height[node], cfg.markings,
                            tags, &paint_heights);
                        if (!approach.indices.empty()) {
                            append_keeping_materials(slot.piece.mesh, approach);
                            slot.painted = true;
                            appended = true;
                        }
                    }
                }

                if (cfg.emit_crossings && cfg.crossings.emit_zebra) {
                    for (const Crossing* crossing : crossings_on_edge[i]) {
                        // The zebra goes on the SAME paint plane as the lane
                        // lines beside it, which is not the plane find_crossings()
                        // recorded. That height came from the raw vertical solve
                        // at stage 3b: it is zero when the network is flat, it
                        // predates the junction plateaus of stage 3d, and it
                        // carries no MarkingConfig::height_above_surface. Emitted
                        // as it stands, the stripes sit under the asphalt on the
                        // flat path and on the un-flattened grade at a junction.
                        Crossing lifted = *crossing;
                        lifted.height = sample_station_height(untrimmed, paint_heights,
                                                              crossing->arclength,
                                                              cfg.corridor.base_height) +
                                        cfg.markings.height_above_surface;
                        const Mesh zebra = build_crossing(lifted, cfg.crossings);
                        if (zebra.indices.empty()) {
                            continue;
                        }
                        append_keeping_materials(slot.piece.mesh, zebra);
                        slot.painted = true;
                        appended = true;
                        ++slot.crossings;
                    }
                }

                // Both a pier and a portal are cut against the ground under the
                // road, so neither is worth emitting without a terrain sampler:
                // a pier with nothing to stand on is a floating post and a portal
                // with no hillside is a headwall in mid-air.
                if (cfg.emit_structures && cfg.height_sampler && on_terrain) {
                    const EdgeElevation& structure_elev = m_elevation.edge(static_cast<EdgeId>(i));

                    // The TRIMMED centerline and its resliced heights: a deck must
                    // end where its ribbon ends, or it overhangs the junction it
                    // was cut back from.
                    if (structure_elev.is_bridge) {
                        const Mesh deck = build_bridge(edge, centerline, profile,
                                                       corridor_cfg.station_heights,
                                                       cfg.height_sampler, cfg.bridge);
                        if (!deck.indices.empty()) {
                            append_keeping_materials(slot.piece.mesh, deck);
                            slot.bridged = true;
                            appended = true;
                        }
                    }
                    if (structure_elev.is_tunnel) {
                        // The footprints come back beside the geometry rather
                        // than being re-derived later: only the builder knows
                        // where along the edge the road actually crossed under
                        // the terrain, and it already found that point in order
                        // to place the headwall.
                        const Mesh portals = build_tunnel_portals(edge, centerline, profile,
                                                                  corridor_cfg.station_heights,
                                                                  cfg.height_sampler, cfg.tunnel,
                                                                  &slot.portals);
                        if (!portals.indices.empty()) {
                            append_keeping_materials(slot.piece.mesh, portals);
                            slot.tunnelled = true;
                            appended = true;
                        } else {
                            // build_tunnel_portals() already leaves the sink empty
                            // when it emits nothing. Cleared again here so the
                            // invariant holds however that contract is edited.
                            slot.portals.clear();
                        }
                    }
                }

                // Only when something was actually added. An untouched piece must
                // come out of this pass byte for byte as the corridor left it,
                // which is what makes each emit_* switch reproduce the earlier
                // phase rather than merely resemble it.
                if (appended) {
                    slot.piece.mesh.sort_submeshes_by_material();
                }
            }

            // ── Stage 4c: game-ready output ──────────────────────────────────
            // Weld, reorder, and optionally derive collision and LODs. Run HERE,
            // by the worker that built the piece, rather than in a sweep after
            // compaction: the four passes are per piece and share nothing, so
            // they parallelise for free on a stage that is already parallel, and
            // they are the most expensive per-piece work in the whole build.
            //
            // It must come after stage 4b, not before. Markings, zebras, decks
            // and portals append whole meshes into this same piece without
            // looking at what is already there, so a weld run before them would
            // miss every duplicate they bring with them -- and those passes are
            // exactly where the duplicates are.
            //
            // The carve payload below reads slot.piece.outline, which is 2D and
            // is not touched by anything here, so the ordering between the two is
            // free.
            finish_piece(slot.piece, cfg, slot.finish);

            if (!on_terrain) {
                continue;
            }

            // ── Carve payload, from the TRIMMED geometry ─────────────────────
            // The carve must describe what is actually rendered. Building it from
            // the untrimmed centerline would flatten a band of terrain running
            // through every junction that no ribbon covers any more.
            CarveRibbon& ribbon = slot.ribbon;
            const EdgeElevation& edge_elevation = m_elevation.edge(static_cast<EdgeId>(i));

            // Copied, not moved: the piece keeps its own footprint for the
            // junction solver and the debug overlay.
            ribbon.outline = slot.piece.outline;
            ribbon.outline_is_simple =
                !slot.piece.outline.empty() && !corridor.outline_self_intersects;

            ribbon.centerline.reserve(centerline.stations.size());
            ribbon.centerline_miter.reserve(centerline.stations.size());
            for (const Station& station : centerline.stations) {
                ribbon.centerline.push_back(station.position);

                // The extruder offsets every lateral by this, so the carve band
                // has to widen by it too or the outer corner of a sharp bend
                // hangs over terrain that was never flattened.
                const double miter = station.miter_scale;
                ribbon.centerline_miter.push_back(
                    std::isfinite(miter) && miter > 1.0 ? static_cast<float>(miter) : 1.0f);
            }

            // The carve target is the road surface MINUS surface_offset, not the
            // road surface itself. The corridor's station heights are the
            // carriageway Y and go to the extruder unchanged; carving the terrain
            // to that same value leaves the two coplanar and z-fighting, which is
            // the one thing ElevationConfig::surface_offset exists to prevent.
            // Exactly one of {mesh Y, carve target} may carry the offset, and it
            // is the mesh.
            const float carve_drop = std::isfinite(cfg.elevation.surface_offset)
                                         ? cfg.elevation.surface_offset
                                         : 0.0f;

            if (elevated) {
                ribbon.centerline_heights.reserve(corridor_cfg.station_heights.size());
                for (float surface : corridor_cfg.station_heights) {
                    ribbon.centerline_heights.push_back(surface - carve_drop);
                }
            } else {
                // Keeps heights parallel to the centerline so the carve can index
                // them. An edge here was extruded flat at base_height, so that
                // plane, less the same clearance, is the truthful carve target.
                ribbon.centerline_heights.assign(centerline.stations.size(),
                                                 cfg.corridor.base_height - carve_drop);
            }

            ribbon.half_width = 0.5f * profile.total_width();
            if (!(ribbon.half_width > 0.0f)) {
                ribbon.half_width = kMinRibbonHalfWidth;
            }

            // A tunnel roadway is below ground and a bridge deck floats above the
            // terrain it spans. In both cases the natural surface is the correct
            // one and the ribbon is carried only so the caller can draw or count
            // it.
            ribbon.suppress = edge_elevation.is_tunnel || edge_elevation.is_bridge;

            slot.has_ribbon = true;
        }
    });

    // ── Stage 4d: junction pieces ────────────────────────────────────────────
    // Turned into pieces in their own parallel pass rather than inside the
    // compaction loop below, because finish_piece() is expensive and a junction
    // mesh is the one that gains most from it: the fill and the curb ring are
    // merged from separate builders, so every vertex of the seam between them
    // exists twice before the weld.
    //
    // Slots are pre-sized and each is written by exactly one worker, so the
    // compaction that follows still walks them in ascending junction order.
    std::vector<JunctionSlot> junction_slots(network.junctions.size());

    run_edge_ranges(network.junctions.size(), [&](size_t begin, size_t end) {
        for (size_t j = begin; j < end; ++j) {
            const Junction& junction = network.junctions[j];
            if (!junction.valid || junction.mesh.vertices.empty() ||
                junction.mesh.indices.empty()) {
                continue;
            }

            JunctionSlot& slot = junction_slots[j];
            slot.piece.anchor = junction.center;
            slot.piece.edge = kInvalidId;
            slot.piece.mesh = junction.mesh;    // copied: the Junction stays self-describing
            slot.piece.outline = junction.footprint;
            finish_piece(slot.piece, cfg, slot.finish);
            slot.filled = true;
        }
    });

    // ── Stage 5: compaction ──────────────────────────────────────────────────
    // On the calling thread, in ascending EdgeId order. This is what makes the
    // result independent of how the ranges were scheduled.
    network.pieces.reserve(edge_count + network.junctions.size());
    if (on_terrain) {
        network.carve_ribbons.reserve(edge_count);
    }

    for (size_t i = 0; i < edge_count; ++i) {
        EdgeSlot& slot = slots[i];
        if (!slot.filled) {
            ++network.stats.skipped_edges;
            if (slot.trimmed_away) {
                ++network.stats.trimmed_away_edges;
            }
            continue;
        }

        network.stats.vertices += slot.piece.mesh.vertices.size();
        network.stats.triangles += slot.piece.mesh.indices.size() / 3;
        network.stats.vertices_welded += slot.finish.vertices_welded;
        network.stats.vertices_dropped += slot.finish.vertices_dropped;
        network.stats.triangles_before_lod += slot.finish.triangles_before_lod;
        network.stats.triangles_after_lod += slot.finish.triangles_after_lod;
        network.stats.collision_triangles += slot.finish.collision_triangles;
        if (slot.trimmed) {
            ++network.stats.trimmed_edges;
        }
        if (slot.elevated) {
            ++network.stats.elevated_edges;
        }
        // Gated on the flag, not only on the geometry. A zebra is Markings
        // geometry too, so a build with the markings pass off and the crossings
        // pass on still paints; counting those here would report a crossing under
        // a name that says lane line. See RoadNetwork::Stats::markings_pieces.
        if (cfg.emit_markings && slot.painted) {
            ++network.stats.markings_pieces;
        }
        if (slot.bridged) {
            ++network.stats.bridges;
        }
        if (slot.tunnelled) {
            ++network.stats.tunnels;
        }
        network.stats.crossings += slot.crossings;
        if (slot.has_ribbon) {
            network.carve_ribbons.push_back(std::move(slot.ribbon));
        }
        // Appended in EdgeId order like everything else here, so a rebuild
        // produces the same list whatever the scheduling. Not parallel to
        // `pieces`: most edges contribute none.
        for (TunnelPortalFootprint& portal : slot.portals) {
            network.carve_portals.push_back(std::move(portal));
        }

        network.pieces.push_back(std::move(slot.piece));
    }

    // ── Junction pieces ──────────────────────────────────────────────────────
    // Appended AFTER every edge piece, in the ascending GraphNodeId order
    // JunctionBuilder emitted them in, so the edge pieces stay in ascending
    // EdgeId order and carve_ribbons stays parallel to them. A junction spans
    // several edges and belongs to none, which is what EdgeId kInvalidId means on
    // a RoadPiece.
    for (JunctionSlot& slot : junction_slots) {
        if (!slot.filled) {
            continue;
        }

        network.stats.vertices += slot.piece.mesh.vertices.size();
        network.stats.triangles += slot.piece.mesh.indices.size() / 3;
        network.stats.vertices_welded += slot.finish.vertices_welded;
        network.stats.vertices_dropped += slot.finish.vertices_dropped;
        network.stats.triangles_before_lod += slot.finish.triangles_before_lod;
        network.stats.triangles_after_lod += slot.finish.triangles_after_lod;
        network.stats.collision_triangles += slot.finish.collision_triangles;
        ++network.stats.junction_pieces;

        network.pieces.push_back(std::move(slot.piece));
    }

    network.stats.pieces = network.pieces.size();

    // ── Junction carve footprints ────────────────────────────────────────────
    // One disc per graph node of degree 3 or more, exactly as P3 emitted them,
    // plus one per taper, dead end and roundabout that produced a footprint.
    // CarveDisc SURVIVES rather than being replaced: a node whose trim solve went
    // degenerate has no polygon at all and must still carve something under its
    // arm mouths, and `center`/`radius` remain a conservative bound of `outline`
    // for any consumer that has not been taught about polygons. What P4 adds is
    // the real fillet-and-curb outline in CarveDisc::outline.
    if (on_terrain && m_elevation.node_heights().size() == m_graph.nodes().size()) {
        // Node -> solved junction, for the outlines. Roundabouts are keyed by
        // their loop's first node and are carried separately below, because that
        // node may be an ordinary approach junction in its own right.
        std::vector<const Junction*> junction_at_node(m_graph.nodes().size(), nullptr);
        for (const Junction& junction : network.junctions) {
            if (junction.kind == JunctionKind::Roundabout) {
                continue;
            }
            if (junction.node != kInvalidId && junction.node < junction_at_node.size()) {
                junction_at_node[junction.node] = &junction;
            }
        }

        for (size_t n = 0; n < m_graph.nodes().size(); ++n) {
            const GraphNode& node = m_graph.nodes()[n];
            if (!node.is_junction()) {
                continue;
            }

            // Suppression must be decided from the SAME classification the
            // ribbons use -- EdgeElevation, the solver's output -- and not from
            // the raw GraphEdge tag. The two disagree on bridges: a viaduct fork
            // suppresses every one of its ribbons but, tested against
            // GraphEdge::is_tunnel, would still emit a carving disc and raise a
            // mesa of terrain to deck height under the flyover.
            float radius = 0.0f;
            bool all_suppressed = true;
            for (const Arm& arm : node.arms) {
                if (arm.edge == kInvalidId || static_cast<size_t>(arm.edge) >= edge_count) {
                    continue;
                }
                radius = std::max(radius, 0.5f * profiles[arm.edge].total_width());

                const EdgeElevation& arm_elev = m_elevation.edge(arm.edge);
                if (!arm_elev.is_tunnel && !arm_elev.is_bridge) {
                    all_suppressed = false;
                }
            }

            CarveDisc disc;
            disc.center = node.position;
            disc.radius = radius > 0.0f ? radius : kMinDiscRadius;

            // node_height() deliberately excludes surface_offset, and a CarveDisc
            // is a carve TARGET rather than a surface, so nothing is added back:
            // the junction geometry sits at node_height plus the offset, exactly
            // the clearance the ribbons get.
            disc.height = m_elevation.node_height(static_cast<GraphNodeId>(n));
            disc.suppress = all_suppressed;

            const Junction* solved = junction_at_node[n];
            if (solved != nullptr && !solved->footprint.empty()) {
                disc.outline = solved->footprint;
                disc.outline_is_simple = !solved->polygon.self_intersecting;

                // The radius contract is that it always BOUNDS the outline, so a
                // consumer ignoring the polygon still carves a superset.
                for (const glm::dvec2& point : disc.outline) {
                    const double reach = glm::length(point - disc.center);
                    if (std::isfinite(reach)) {
                        disc.radius = std::max(disc.radius, static_cast<float>(reach));
                    }
                }
            }

            network.carve_discs.push_back(std::move(disc));
        }

        // ── Tapers and dead ends ─────────────────────────────────────────
        // P4 trims degree-2 taper nodes as well as junctions, and a dead-end
        // bulb or turning circle reaches past the ribbon's last station, so both
        // cover ground no CarveRibbon does: a lane change taper can be 26 m of
        // flat wedge sitting on raw procedural terrain. Neither node is
        // `is_junction()`, so neither is reached by the loop above. Their
        // footprints come from the junction solve itself, which is why they are
        // emitted from the junction list rather than from the graph.
        for (const Junction& junction : network.junctions) {
            if (junction.kind != JunctionKind::Taper && junction.kind != JunctionKind::DeadEnd) {
                continue;
            }
            if (!junction.valid || junction.footprint.empty()) {
                continue;
            }
            if (junction.node == kInvalidId || junction.node >= m_graph.nodes().size()) {
                continue;
            }

            CarveDisc disc;
            disc.center = junction.center;
            disc.height = m_elevation.node_height(junction.node);
            disc.outline = junction.footprint;
            disc.outline_is_simple = true;
            disc.radius = kMinDiscRadius;
            for (const glm::dvec2& point : disc.outline) {
                const double reach = glm::length(point - disc.center);
                if (std::isfinite(reach)) {
                    disc.radius = std::max(disc.radius, static_cast<float>(reach));
                }
            }

            bool all_suppressed = true;
            for (const Arm& arm : m_graph.nodes()[junction.node].arms) {
                if (arm.edge == kInvalidId || static_cast<size_t>(arm.edge) >= edge_count) {
                    continue;
                }
                const EdgeElevation& arm_elev = m_elevation.edge(arm.edge);
                if (!arm_elev.is_tunnel && !arm_elev.is_bridge) {
                    all_suppressed = false;
                }
            }
            disc.suppress = all_suppressed;

            network.carve_discs.push_back(std::move(disc));
        }

        // Roundabouts carve their own neighbourhood. Their ring edges emit no
        // ribbon -- the annulus replaced them -- so without this a roundabout
        // would sit on uncarved terrain.
        for (const Junction& junction : network.junctions) {
            if (junction.kind != JunctionKind::Roundabout || junction.footprint.empty()) {
                continue;
            }
            if (junction.node == kInvalidId || junction.node >= m_graph.nodes().size()) {
                continue;
            }

            CarveDisc disc;
            disc.center = junction.center;
            disc.height = m_elevation.node_height(junction.node);
            disc.outline = junction.footprint;
            disc.outline_is_simple = true;
            disc.radius = kMinDiscRadius;
            for (const glm::dvec2& point : disc.outline) {
                const double reach = glm::length(point - disc.center);
                if (std::isfinite(reach)) {
                    disc.radius = std::max(disc.radius, static_cast<float>(reach));
                }
            }

            bool all_suppressed = true;
            for (const Arm& arm : m_graph.nodes()[junction.node].arms) {
                if (arm.edge == kInvalidId || static_cast<size_t>(arm.edge) >= edge_count) {
                    continue;
                }
                const EdgeElevation& arm_elev = m_elevation.edge(arm.edge);
                if (!arm_elev.is_tunnel && !arm_elev.is_bridge) {
                    all_suppressed = false;
                }
            }
            disc.suppress = all_suppressed;

            network.carve_discs.push_back(std::move(disc));
        }
    }

    const auto finished = std::chrono::steady_clock::now();
    network.stats.build_ms = std::chrono::duration<double, std::milli>(finished - started).count();

    spdlog::info("RoadNetworkBuilder: Built network — {} pieces from {} edges, "
                 "{} vertices, {} triangles, in {:.1f} ms",
                 network.stats.pieces, network.stats.edges, network.stats.vertices,
                 network.stats.triangles, network.stats.build_ms);

    if (on_terrain) {
        spdlog::info("RoadNetworkBuilder: On terrain — {} elevated edges, {} carve ribbons, "
                     "{} junction discs, solve took {:.1f} ms",
                     network.stats.elevated_edges, network.carve_ribbons.size(),
                     network.carve_discs.size(), network.stats.elevation_ms);
    } else if (cfg.height_sampler) {
        spdlog::warn("RoadNetworkBuilder: A height sampler was supplied but the elevation solve "
                     "produced no usable result; the network stays flat and nothing is carved");
    }

    // P5 and P6 are logged together because they share one failure: a pass that
    // was switched on and found nothing to do reports the same zero as a pass
    // that was switched off. The flags are named so the two can be told apart.
    spdlog::info("RoadNetworkBuilder: Detail — {} pieces painted, {} crossings, {} dropped kerb "
                 "spans, {} bridges, {} tunnels, {} portals, {} sidewalk sides deduped "
                 "(markings {}, crossings {}, structures {}, dedup {})",
                 network.stats.markings_pieces, network.stats.crossings,
                 network.stats.dropped_kerb_spans, network.stats.bridges,
                 network.stats.tunnels, network.carve_portals.size(),
                 network.stats.deduped_sidewalks,
                 cfg.emit_markings ? "on" : "off", cfg.emit_crossings ? "on" : "off",
                 cfg.emit_structures ? "on" : "off", cfg.dedup.enabled ? "on" : "off");

    // ── P7 summary ───────────────────────────────────────────────────────────
    // One line, because the weld ratio is the number that says whether the phase
    // did anything. Stats::vertices is the count AFTER welding, so the pre-weld
    // total has to be reconstructed here rather than read.
    //
    // The LOD pair is the coarsest level against level 0, which is a REDUCTION
    // figure and not a memory figure: with LODs on a piece holds its geometry
    // roughly 1.85 times over at the default ratios. The flags are named for the
    // same reason they are on the detail line -- a pass that was switched off and
    // a pass that found nothing report the same zero.
    {
        // The pre-P7 total, exactly: what a consumer receives, plus what the weld
        // merged, plus what the reorder's compaction dropped. Leaving the dropped
        // vertices out under-reports the input by however many unreferenced
        // vertices the upstream passes emitted -- the degenerate triangles the
        // junction curb ring refuses still leave their columns behind.
        const size_t vertices_before_p7 = network.stats.vertices +
                                          network.stats.vertices_welded +
                                          network.stats.vertices_dropped;
        const double weld_ratio =
            vertices_before_p7 > 0
                ? 100.0 * static_cast<double>(network.stats.vertices_welded) /
                      static_cast<double>(vertices_before_p7)
                : 0.0;
        const double lod_ratio =
            network.stats.triangles_before_lod > 0
                ? 100.0 * static_cast<double>(network.stats.triangles_after_lod) /
                      static_cast<double>(network.stats.triangles_before_lod)
                : 0.0;

        spdlog::info("RoadNetworkBuilder: Game-ready — welded {} of {} vertices ({:.1f}%), dropped "
                     "{} unreferenced, down to {}, {} triangles unchanged by the weld, LOD0 {} -> "
                     "coarsest {} ({:.1f}%), {} collision triangles (weld {}, optimize {}, "
                     "collision {}, lods {})",
                     network.stats.vertices_welded, vertices_before_p7, weld_ratio,
                     network.stats.vertices_dropped,
                     network.stats.vertices, network.stats.triangles,
                     network.stats.triangles_before_lod, network.stats.triangles_after_lod,
                     lod_ratio, network.stats.collision_triangles,
                     cfg.weld_meshes ? "on" : "off", cfg.optimize_meshes ? "on" : "off",
                     cfg.build_collision ? "on" : "off", cfg.build_lods ? "on" : "off");
    }

    if (cfg.emit_structures && !cfg.height_sampler) {
        spdlog::info("RoadNetworkBuilder: Structures are on but no height sampler is configured, "
                     "so no bridge or tunnel geometry was emitted. A pier stands on the ground "
                     "and a portal is cut into it, and without a terrain height there is no "
                     "ground to place either against; the deck and parapets are skipped with "
                     "them rather than left floating over nothing");
    }

    if (cfg.solve_junctions) {
        const JunctionBuilder::Stats& junction_stats_ref = junction_stats;
        spdlog::info("RoadNetworkBuilder: Junctions — {} solved, {} pieces, {} trimmed edges, "
                     "{} trimmed away, solve took {:.1f} ms",
                     network.junctions.size(), network.stats.junction_pieces,
                     network.stats.trimmed_edges, network.stats.trimmed_away_edges,
                     network.stats.junction_ms);
        if (junction_stats_ref.over_trimmed_edges > 0) {
            spdlog::warn("RoadNetworkBuilder: {} edges were clamped short of the trim their "
                         "junctions demanded; those junction polygons overlap their ribbons",
                         junction_stats_ref.over_trimmed_edges);
        }
    }

    // Aggregated, not one line per edge: a coastline-heavy extract can skip
    // thousands of degenerate ways and the per-edge form would bury the summary.
    if (network.stats.skipped_edges > 0) {
        spdlog::warn("RoadNetworkBuilder: Skipped {} of {} edges that produced no geometry "
                     "(degenerate centerline, invalid profile, empty corridor, a trim that "
                     "consumed the edge, or a roundabout annulus that replaced it)",
                     network.stats.skipped_edges, network.stats.edges);
    }

    return network;
}

} // namespace stratum::osm::road
