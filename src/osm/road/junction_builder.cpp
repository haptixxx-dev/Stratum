/**
 * @file junction_builder.cpp
 * @brief Orchestrates the whole P4 junction solve over a built road graph
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why the solve is staged, and where the race is
 *
 * A junction is a property of a NODE, so the obvious shape is one parallel sweep
 * over nodes. That shape is wrong, and the reason is the one thing this file
 * writes outside itself: `GraphEdge::trim_from` and `trim_to`.
 *
 * An edge is cut from BOTH ends by TWO DIFFERENT NODES. Solving nodes in
 * parallel and writing the trims as they are solved is a data race on every
 * edge in the network, and a benign-looking one -- both writers are writing a
 * plausible double -- so it would corrupt geometry rather than crash. Worse, the
 * result would depend on the scheduling and a build would stop being
 * reproducible.
 *
 * The solve is therefore five passes:
 *
 * @code
 *     A. find_roundabouts                      (serial, cheap, whole graph)
 *     B. trims + tapers + dead-end caps        (PARALLEL, per-node storage only)
 *     C. apply trims to the graph              (SERIAL, ascending node id)
 *     D. junction fill, fillets, curb rings    (PARALLEL, per-node storage only)
 *     E. roundabout annuli                     (serial, one per loop)
 * @endcode
 *
 * Pass B writes ONLY into that node's own `JunctionNodeSolve` slot, never into the
 * graph. Pass C is the only writer of the graph, runs on the calling thread, and
 * visits nodes in ascending GraphNodeId order, so the trims are a pure function
 * of the input whatever the scheduling. Pass C then writes the FINAL trims back
 * into each stored `ArmRef`, so pass D builds its cross-sections from exactly
 * the value the extruder will later cut at. Deriving the geometry from the
 * pre-clamp demand instead is the subtle version of the same bug: the junction
 * polygon would sit where the arm was asked to stop rather than where it
 * actually stops.
 *
 * ### Accumulating rather than overwriting
 *
 * Each (edge, end) pair is normally claimed by exactly one node, so "accumulate"
 * and "overwrite" agree. They stop agreeing on a CLOSED LOOP edge, whose two
 * ends are the same node and which therefore contributes two arms to it, and on
 * any future path that visits a node twice. Pass C takes the MAXIMUM of every
 * demand on a given end, which is order-independent and always the safe
 * direction: a trim that is too small leaves an overlap, one that is too large
 * leaves a gap, and the combined-length clamp below only ever reduces.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#include "osm/road/junction_builder.hpp"

#include <TaskScheduler.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <utility>

namespace stratum::osm::road {

namespace {

/**
 * @brief Node count below which the parallel passes run on the calling thread
 *
 * Matches the corresponding threshold in road_network_builder.cpp and exists for
 * the same reason: spinning up a scheduler costs one thread creation per
 * hardware thread, which dominates the work it would distribute on a small
 * extract or a test fixture. Results are identical either way.
 */
constexpr size_t kParallelMinNodes = 256;

/// Nodes handed to one worker at a time
constexpr uint32_t kNodesPerRange = 16;

/**
 * @brief Ribbon length that must survive both trims of an edge, metres
 *
 * The per-end clamp in TrimConfig::max_trim_fraction cannot see the far end, so
 * two ends configured generously enough could between them consume a whole short
 * edge. This is the joint floor that stops that: a negative-length ribbon reaches
 * slice() as a reversed range, and a zero-length one reaches the extruder as a
 * ribbon with no stations.
 *
 * It is deliberately small. Leaving a stub too short to resample is honest and is
 * counted as an over-trimmed edge; inventing a large reserve here would silently
 * undo the trims the junction geometry was built against.
 */
constexpr double kMinRemainingLength = 0.25;

/// Stand-in for "no ceiling" when asking build_profile_taper() for its own demand
constexpr double kNoTrimLimit = 1e30;

/// Two trims closer together than this are the same cut, so nothing needs rebuilding
constexpr double kTrimMatchEpsilon = 1e-6;

/// Shortest edge apply_junction_plateaus() will divide a disputed station along
constexpr double kMinPlateauSpan = 1e-9;

/**
 * @brief Run @p fn over [0, count) as sub-ranges, in parallel when it pays
 *
 * The scheduler is scoped to one pass rather than to the whole solve, because
 * the serial trim application sits between the two parallel passes and holding a
 * scheduler open across it would keep every worker thread alive for nothing.
 *
 * @param count Number of graph nodes
 * @param fn    Callable taking a half-open [begin, end) range of GraphNodeIds
 */
template <typename Fn>
void run_node_ranges(size_t count, Fn&& fn) {
    if (count == 0) {
        return;
    }
    if (count < kParallelMinNodes) {
        fn(size_t{0}, count);
        return;
    }

    enki::TaskScheduler scheduler;
    scheduler.Initialize();

    enki::TaskSet sweep(static_cast<uint32_t>(count),
                        [&](enki::TaskSetPartition range, uint32_t /*threadnum*/) {
                            fn(static_cast<size_t>(range.start), static_cast<size_t>(range.end));
                        });
    sweep.m_MinRange = kNodesPerRange;

    scheduler.AddTaskSetToPipe(&sweep);
    scheduler.WaitforTask(&sweep);
}

/**
 * @brief Append one mesh into another, keeping the source's own material ranges
 *
 * Mesh::append() attributes ALL of the appended geometry to a single material,
 * which is right for a single-material source and wrong for every source here: a
 * CurbRing carries curb faces, curb tops and sidewalk surfaces in one mesh, and
 * folding those into one range would paint the whole ring with one material.
 *
 * The source's implicit whole-mesh range is resolved through
 * effective_submeshes(), so a producer that never touched `submeshes` still
 * lands in MaterialId::Default rather than being dropped.
 *
 * @param dst Mesh to append into
 * @param src Mesh to copy; its submesh ranges are preserved as ranges of @p dst
 */
void append_preserving_materials(Mesh& dst, const Mesh& src) {
    if (src.vertices.empty() || src.indices.empty()) {
        return;
    }

    // Materialise the destination's implicit range first, or the appended
    // geometry would be absorbed into it and lose its own material identity.
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

        for (size_t i = begin; i < end; ++i) {
            dst.indices.push_back(src.indices[i] + base);
        }
    }
}

/**
 * @brief Counter-clockwise circle used as a roundabout's carve footprint
 *
 * A roundabout emits an annulus rather than a ring polygon, so it has no natural
 * outer boundary to hand the terrain carve. The fitted circle, widened by the
 * widest profile on the loop, is the smallest shape that certainly covers it.
 *
 * @param center   Loop centre in 2D local metres
 * @param radius   Outer radius in metres; a non-positive value yields an empty ring
 * @param segments Vertices around the full turn, clamped to at least 8
 * @return The ring, first point not repeated
 */
[[nodiscard]] std::vector<glm::dvec2> circle_ring(const glm::dvec2& center, double radius,
                                                  int segments) {
    std::vector<glm::dvec2> ring;
    if (!(std::isfinite(radius) && radius > 0.0)) {
        return ring;
    }

    const int count = std::max(8, segments);
    ring.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double angle = 2.0 * 3.14159265358979323846 * static_cast<double>(i) /
                             static_cast<double>(count);
        ring.push_back(center + glm::dvec2(std::cos(angle), std::sin(angle)) * radius);
    }
    return ring;
}

} // namespace

/**
 * @brief Everything one node produced, before any of it reaches the graph
 *
 * One slot per GraphNodeId, written by exactly one worker in pass B and again by
 * exactly one worker in pass D, and never resized. This is what keeps the
 * parallel passes free of shared state: nothing here is visible to another node's
 * worker, and the graph itself is not written until pass C.
 */
struct JunctionNodeSolve {
    std::vector<ArmRef> arms;       ///< Bearing-ordered arms; trims solved in pass B
    std::vector<ArmEnd> ends;       ///< Cut cross-sections; filled in pass D
    JunctionPolygon polygon;        ///< Carriageway footprint; filled in pass D
    CurbRing curb;                  ///< Sidewalk ring; filled in pass D
    Mesh mesh;                      ///< Merged geometry
    std::vector<glm::dvec2> footprint;  ///< Carve boundary
    float height = 0.0f;            ///< World Y of the carriageway surface at this node

    /**
     * @brief Trims the taper wedge in `mesh` was actually built over
     *
     * Pass B builds the wedge from the taper's own demand, which pass C's joint
     * budget can still reduce. Pass D compares these against the FINAL trims and
     * rebuilds the wedge where they differ, so the geometry always ends exactly
     * where the extruder cuts. Meaningless for any other kind.
     */
    double taper_trim_a = 0.0;
    double taper_trim_b = 0.0;

    JunctionKind kind = JunctionKind::Intersection;
    bool participates = false;      ///< Node contributed arms to the trim solve
    bool emit = false;              ///< Node produces a Junction entry
    bool valid = false;             ///< Junction produced usable geometry
    bool clamped_arm = false;       ///< Some arm's demand was reduced by a clamp
};

/**
 * @brief Everything phase one produced, held until phase two consumes it
 *
 * Local to build() before the two-phase split; a member now, because
 * solve_trims() and build_geometry() are two calls and the state has to survive
 * between them. Held behind a pointer so the header never sees ArmRef, ArmEnd,
 * CurbRing or RoundaboutLoop by value.
 */
struct JunctionBuilder::SolveState {
    JunctionConfig cfg;                     ///< The config phase one was given
    std::vector<JunctionNodeSolve> solves;  ///< One slot per GraphNodeId
    std::vector<RoundaboutLoop> loops;      ///< Every loop pass A found
    std::vector<int> loop_at_node;          ///< Index into `loops`, or -1
    std::vector<float> node_heights;        ///< World Y of the surface at each node
    std::chrono::steady_clock::time_point started{};
    bool ready = false;                     ///< solve_trims() completed
};

JunctionBuilder::JunctionBuilder() = default;
JunctionBuilder::~JunctionBuilder() = default;
JunctionBuilder::JunctionBuilder(JunctionBuilder&&) noexcept = default;
JunctionBuilder& JunctionBuilder::operator=(JunctionBuilder&&) noexcept = default;

// ============================================================================
// Builder
// ============================================================================

std::vector<Junction> JunctionBuilder::build(RoadGraph& graph,
                                             const std::vector<Centerline>& centerlines,
                                             const std::vector<RoadProfile>& profiles,
                                             const RoadElevationSolver& elevation,
                                             const JunctionConfig& cfg) {
    // The one-call form is the two-phase form with no kerb drops, which is
    // exactly the P4 behaviour: nothing else in the solve differs.
    if (!solve_trims(graph, centerlines, profiles, elevation, cfg)) {
        return {};
    }
    return build_geometry(graph, centerlines, profiles, nullptr);
}

bool JunctionBuilder::solve_trims(RoadGraph& graph,
                                  const std::vector<Centerline>& centerlines,
                                  const std::vector<RoadProfile>& profiles,
                                  const RoadElevationSolver& elevation,
                                  const JunctionConfig& cfg) {
    m_stats = Stats{};
    m_consumed_edges.clear();
    m_dropped_kerb_junctions = 0;

    m_solve = std::make_unique<SolveState>();
    m_solve->cfg = cfg;
    m_solve->started = std::chrono::steady_clock::now();

    const size_t node_count = graph.nodes().size();
    const size_t edge_count = graph.edges().size();
    if (node_count == 0 || edge_count == 0) {
        return false;
    }
    if (centerlines.size() != edge_count || profiles.size() != edge_count) {
        spdlog::error("JunctionBuilder: centerlines ({}) and profiles ({}) must both be parallel "
                      "to the graph's {} edges; nothing was solved",
                      centerlines.size(), profiles.size(), edge_count);
        return false;
    }

    m_consumed_edges.assign(edge_count, false);

    // The trim solve has to leave every corner enough straight run for its fillet
    // arc's tangent points, and it cannot see FilletConfig itself. Copying the
    // four values across here is what stops the solve and the polygon describing
    // two different junctions -- without it every square corner is cut back by
    // TrimConfig::clearance alone and falls through FilletConfig::min_radius to a
    // chamfer a few centimetres long.
    TrimConfig trim_cfg = cfg.trim;
    apply_fillet_reserve(cfg.fillet, trim_cfg);

    // Node heights are an INPUT. When the vertical solve did not run every
    // junction lands on the flat plane the ribbons were extruded on.
    const bool on_terrain =
        elevation.is_solved() && elevation.node_heights().size() == node_count;
    // Resolved ONCE, here, rather than read again in phase two: the elevation
    // solver is an input to phase one and build_geometry() is not given it, so
    // the heights have to survive the seam between the two calls.
    m_solve->node_heights.assign(node_count, cfg.base_height);
    if (on_terrain) {
        for (size_t n = 0; n < node_count; ++n) {
            m_solve->node_heights[n] =
                elevation.node_height(static_cast<GraphNodeId>(n)) + cfg.surface_offset;
        }
    }
    const auto node_height = [&](GraphNodeId n) { return m_solve->node_heights[n]; };

    // ── Pass A: roundabouts ──────────────────────────────────────────────────
    // Run first so the loops are known before anything else looks at their
    // edges. A valid loop's edges are CONSUMED: the annulus in pass E replaces
    // their ribbons wholesale, so RoadNetworkBuilder must not extrude them as
    // well. Their trims are still solved and applied, because build_roundabout()
    // sweeps the annulus over `slice(cl, trim_from, length - trim_to)` and the
    // approach mouth's own fillet then fills the gap the trim opened.
    m_solve->loops = find_roundabouts(graph, centerlines);
    const std::vector<RoundaboutLoop>& loops = m_solve->loops;

    /// Index into `loops` of the loop whose FIRST node this is, or -1
    m_solve->loop_at_node.assign(node_count, -1);
    std::vector<int>& loop_at_node = m_solve->loop_at_node;
    for (size_t l = 0; l < loops.size(); ++l) {
        const RoundaboutLoop& loop = loops[l];
        if (!loop.valid || loop.edges.empty() || loop.nodes.empty()) {
            continue;
        }
        for (EdgeId e : loop.edges) {
            if (e != kInvalidId && e < edge_count) {
                m_consumed_edges[e] = true;
            }
        }
        const GraphNodeId first = loop.nodes.front();
        if (first != kInvalidId && first < node_count && loop_at_node[first] < 0) {
            loop_at_node[first] = static_cast<int>(l);
        }
    }

    // ── Pass B: trims, tapers and dead-end caps ──────────────────────────────
    // Parallel over nodes. NOTHING here touches the graph; every worker writes
    // only into its own slot.
    m_solve->solves.assign(node_count, JunctionNodeSolve{});
    std::vector<JunctionNodeSolve>& solves = m_solve->solves;

    run_node_ranges(node_count, [&](size_t begin, size_t end) {
        for (size_t n = begin; n < end; ++n) {
            const GraphNode& node = graph.nodes()[n];
            JunctionNodeSolve& slot = solves[n];
            slot.height = node_height(static_cast<GraphNodeId>(n));

            const size_t degree = node.degree();

            if (degree >= 3) {
                slot.participates = true;
                slot.emit = true;
                slot.arms = collect_arms(graph, profiles, static_cast<GraphNodeId>(n));
                const bool solved = solve_arm_trims(graph, centerlines,
                                                    static_cast<GraphNodeId>(n), slot.arms,
                                                    trim_cfg);
                if (!solved) {
                    slot.kind = JunctionKind::Degenerate;
                }
                for (const ArmRef& arm : slot.arms) {
                    if (arm.clamped) {
                        slot.clamped_arm = true;
                    }
                }
                continue;
            }

            if (degree == 2) {
                Mesh taper;
                double trim_a = 0.0;
                double trim_b = 0.0;
                std::vector<glm::dvec2> outline;
                const bool tapered = build_profile_taper(graph, centerlines, profiles,
                                                         static_cast<GraphNodeId>(n), slot.height,
                                                         cfg.special.taper, taper, trim_a, trim_b,
                                                         kNoTrimLimit, kNoTrimLimit, &outline);
                if (!tapered) {
                    continue;   // profiles agreed; nothing to blend and nothing to cut
                }

                slot.arms = collect_arms(graph, profiles, static_cast<GraphNodeId>(n));
                if (slot.arms.size() == 2) {
                    slot.arms[0].trim = trim_a;
                    slot.arms[1].trim = trim_b;
                    slot.participates = true;
                }
                slot.taper_trim_a = trim_a;
                slot.taper_trim_b = trim_b;
                slot.mesh = std::move(taper);
                slot.footprint = std::move(outline);
                slot.kind = JunctionKind::Taper;
                slot.emit = true;
                slot.valid = !slot.mesh.vertices.empty() && !slot.mesh.indices.empty();
                continue;
            }

            if (degree == 1) {
                // A dead end EXTENDS the network rather than cutting it back, so
                // it emits no trim and needs nothing from pass C. Its geometry is
                // therefore built here rather than in pass D.
                std::vector<glm::dvec2> outline;
                Mesh cap = build_dead_end(graph, centerlines, profiles,
                                          static_cast<GraphNodeId>(n), slot.height,
                                          cfg.special.dead_end, &outline);
                if (cap.vertices.empty() || cap.indices.empty()) {
                    continue;
                }
                slot.arms = collect_arms(graph, profiles, static_cast<GraphNodeId>(n));
                slot.mesh = std::move(cap);
                slot.footprint = std::move(outline);
                slot.kind = JunctionKind::DeadEnd;
                slot.emit = true;
                slot.valid = true;
            }
        }
    });

    // ── Pass C: apply the trims to the graph ─────────────────────────────────
    // The ONLY writer of GraphEdge::trim_from and trim_to, on the calling thread,
    // in ascending GraphNodeId order. See the race note at the top of this file.
    std::vector<double> want_from(edge_count, 0.0);
    std::vector<double> want_to(edge_count, 0.0);

    for (size_t n = 0; n < node_count; ++n) {
        const JunctionNodeSolve& slot = solves[n];
        if (!slot.participates) {
            continue;
        }
        for (const ArmRef& arm : slot.arms) {
            if (arm.edge == kInvalidId || arm.edge >= edge_count) {
                continue;
            }
            double demand = arm.trim;
            if (!std::isfinite(demand) || demand < 0.0) {
                demand = 0.0;
            }
            double& want = arm.at_start ? want_from[arm.edge] : want_to[arm.edge];
            want = std::max(want, demand);   // accumulate, never overwrite
        }
    }

    std::vector<GraphEdge>& edges = graph.mutable_edges();

    // Over-trimming is counted per EDGE, not per arm, so an edge clamped at both
    // ends -- or clamped once by TrimConfig::max_trim_fraction and again by the
    // joint budget below -- counts exactly once.
    std::vector<bool> over_trimmed(edge_count, false);
    for (size_t n = 0; n < node_count; ++n) {
        const JunctionNodeSolve& slot = solves[n];
        if (!slot.clamped_arm) {
            continue;
        }
        for (const ArmRef& arm : slot.arms) {
            if (arm.clamped && arm.edge != kInvalidId && arm.edge < edge_count) {
                over_trimmed[arm.edge] = true;
            }
        }
    }

    for (size_t e = 0; e < edge_count; ++e) {
        double from_trim = want_from[e];
        double to_trim = want_to[e];

        // The per-end clamp inside solve_arm_trims() cannot see the far end, so
        // the joint budget is enforced here. Reducing both ends in proportion
        // keeps the junction polygons at either end symmetric about the shortfall
        // instead of sacrificing whichever end happened to be visited second.
        const Centerline& cl = centerlines[e];
        const double span = cl.is_valid()
                                ? cl.stations.back().arclength - cl.stations.front().arclength
                                : 0.0;
        bool clamped_here = false;

        if (!(std::isfinite(span) && span > 0.0)) {
            from_trim = 0.0;
            to_trim = 0.0;
        } else if (from_trim + to_trim > span - kMinRemainingLength) {
            const double budget = span - kMinRemainingLength;
            const double demanded = from_trim + to_trim;
            if (!(budget > 0.0) || !(demanded > 0.0)) {
                // The edge is shorter than the floor: leave it whole rather than
                // emit an inverted ribbon. The junctions at its ends overlap it,
                // which is the lesser failure.
                from_trim = 0.0;
                to_trim = 0.0;
            } else {
                const double scale = budget / demanded;
                from_trim *= scale;
                to_trim *= scale;
            }
            clamped_here = true;
        }

        edges[e].trim_from = from_trim;
        edges[e].trim_to = to_trim;

        if (clamped_here) {
            over_trimmed[e] = true;
        }
        if (over_trimmed[e]) {
            ++m_stats.over_trimmed_edges;
        }
    }

    // Write the FINAL trims back onto the arms, so pass D builds its
    // cross-sections where the extruder will actually cut rather than where the
    // pairwise solve asked it to.
    for (size_t n = 0; n < node_count; ++n) {
        JunctionNodeSolve& slot = solves[n];
        if (!slot.participates) {
            continue;
        }
        for (ArmRef& arm : slot.arms) {
            if (arm.edge == kInvalidId || arm.edge >= edge_count) {
                arm.trim = 0.0;
                continue;
            }
            arm.trim = arm.at_start ? edges[arm.edge].trim_from : edges[arm.edge].trim_to;
        }
    }

    m_solve->ready = true;
    return true;
}

std::vector<Junction> JunctionBuilder::build_geometry(
    RoadGraph& graph,
    const std::vector<Centerline>& centerlines,
    const std::vector<RoadProfile>& profiles,
    const KerbDropProvider& kerb_drops) {
    std::vector<Junction> out;

    if (!m_solve || !m_solve->ready) {
        spdlog::error("JunctionBuilder: build_geometry() called without a successful "
                      "solve_trims(); no junction geometry was built");
        return out;
    }

    const JunctionConfig& cfg = m_solve->cfg;
    std::vector<JunctionNodeSolve>& solves = m_solve->solves;
    const std::vector<RoundaboutLoop>& loops = m_solve->loops;
    const std::vector<int>& loop_at_node = m_solve->loop_at_node;

    const size_t node_count = solves.size();
    const size_t edge_count = graph.edges().size();
    if (graph.nodes().size() != node_count || centerlines.size() != edge_count ||
        profiles.size() != edge_count) {
        spdlog::error("JunctionBuilder: the graph changed between solve_trims() and "
                      "build_geometry(); no junction geometry was built");
        return out;
    }

    const auto node_height = [&](GraphNodeId n) { return m_solve->node_heights[n]; };

    // Counted under a mutex rather than an atomic because it is touched at most
    // once per junction and the alternative is an extra parallel reduction for a
    // number that only ever reaches a log line.
    std::mutex kerb_mutex;

    // ── Pass D: junction geometry ────────────────────────────────────────────
    // Parallel over nodes again. Reads the graph, writes only into slots.
    run_node_ranges(node_count, [&](size_t begin, size_t end) {
        for (size_t n = begin; n < end; ++n) {
            JunctionNodeSolve& slot = solves[n];

            // A taper's wedge was built in pass B from the trims the taper itself
            // demanded, and the joint budget in pass C can have reduced them
            // since. Rebuilding over the FINAL trims is the same discipline pass
            // D already applies to arm_end(): the geometry has to end where the
            // extruder cuts, or the surviving ribbon lies coplanar on top of the
            // wedge.
            if (slot.kind == JunctionKind::Taper) {
                if (slot.arms.size() != 2) {
                    continue;
                }
                const double final_a = slot.arms[0].trim;
                const double final_b = slot.arms[1].trim;
                if (std::fabs(final_a - slot.taper_trim_a) <= kTrimMatchEpsilon &&
                    std::fabs(final_b - slot.taper_trim_b) <= kTrimMatchEpsilon) {
                    continue;   // nothing was clamped; pass B's wedge already fits
                }

                Mesh taper;
                double rebuilt_a = 0.0;
                double rebuilt_b = 0.0;
                std::vector<glm::dvec2> outline;
                const bool ok = build_profile_taper(graph, centerlines, profiles,
                                                    static_cast<GraphNodeId>(n), slot.height,
                                                    cfg.special.taper, taper, rebuilt_a, rebuilt_b,
                                                    final_a, final_b, &outline);
                if (!ok || taper.vertices.empty() || taper.indices.empty()) {
                    // The budget left no room for a wedge at all. Dropping it is
                    // the honest outcome: the two ribbons meet at a hard profile
                    // step, which is what happened before tapers existed, and it
                    // beats leaving a wedge the ribbons overlap.
                    slot.mesh.clear();
                    slot.footprint.clear();
                    slot.valid = false;
                    continue;
                }
                slot.taper_trim_a = rebuilt_a;
                slot.taper_trim_b = rebuilt_b;
                slot.mesh = std::move(taper);
                slot.footprint = std::move(outline);
                slot.valid = true;
                continue;
            }

            if (slot.kind != JunctionKind::Intersection || slot.arms.size() < 3) {
                continue;
            }

            slot.ends.reserve(slot.arms.size());
            bool ends_usable = true;
            for (const ArmRef& arm : slot.arms) {
                ArmEnd cut = arm_end(graph, centerlines, profiles, arm);
                if (!cut.valid) {
                    ends_usable = false;
                }
                slot.ends.push_back(cut);
            }

            if (!ends_usable) {
                slot.kind = JunctionKind::Degenerate;
                continue;
            }

            slot.polygon = build_junction_polygon(slot.arms, slot.ends, cfg.fillet);
            if (!slot.polygon.valid) {
                slot.kind = JunctionKind::Degenerate;
                continue;
            }

            Mesh fill = triangulate_junction(slot.polygon, slot.height, MaterialId::Asphalt);
            if (fill.vertices.empty() || fill.indices.empty()) {
                slot.kind = JunctionKind::Degenerate;
                continue;
            }
            append_preserving_materials(slot.mesh, fill);

            // A ring is only right where the roads meeting the node actually
            // carry one. A motorway merge, a rural track fork and a service yard
            // have no sidewalk on any arm, and wrapping them in one invents a
            // kerbed footway around an intersection that has none -- visible as a
            // sidewalk material appearing in a motorway's exported mesh.
            bool arms_have_curb = false;
            for (const ArmRef& arm : slot.arms) {
                if (arm.edge == kInvalidId || arm.edge >= profiles.size()) {
                    continue;
                }
                for (const Strip& strip : profiles[arm.edge].strips) {
                    if (strip.kind == StripKind::Sidewalk || strip.kind == StripKind::CurbTop ||
                        strip.kind == StripKind::CurbFace) {
                        arms_have_curb = true;
                        break;
                    }
                }
                if (arms_have_curb) {
                    break;
                }
            }

            if (cfg.emit_curb_rings && arms_have_curb) {
                // A dropped kerb is a break in the ring, so it has to be known
                // BEFORE the offset rather than punched afterwards: the ring is
                // re-tessellated across each ramp so the crossfall is carried by
                // real vertex columns. That is the whole reason solve_trims() and
                // build_geometry() are two calls; see solve_trims().
                KerbDrops drops;
                if (kerb_drops) {
                    drops = kerb_drops(static_cast<GraphNodeId>(n), graph.nodes()[n].position);
                }
                const bool any_drop = drops.any();

                slot.curb = build_curb_ring(slot.polygon, slot.arms, slot.ends, slot.height,
                                            cfg.curb, any_drop ? &drops : nullptr);
                append_preserving_materials(slot.mesh, slot.curb.mesh);

                if (any_drop) {
                    const std::lock_guard<std::mutex> lock(kerb_mutex);
                    ++m_dropped_kerb_junctions;
                }
            }

            // The ring's outer boundary is the junction's real extent; the
            // polygon ring is the carriageway alone and is the fallback when no
            // ring was built. build_curb_ring() keeps `outer` populated even when
            // every section collapsed, precisely so it can be used here.
            slot.footprint = !slot.curb.outer.empty() ? slot.curb.outer : slot.polygon.ring;

            slot.mesh.sort_submeshes_by_material();
            slot.valid = true;
        }
    });

    // ── Pass E: roundabout annuli ────────────────────────────────────────────
    // Serial: a network has a handful of loops, and build_roundabout() reads the
    // trims pass C has just written.
    std::vector<Junction> roundabout_junctions(loops.size());
    std::vector<bool> roundabout_built(loops.size(), false);

    for (size_t l = 0; l < loops.size(); ++l) {
        const RoundaboutLoop& loop = loops[l];
        if (!loop.valid || loop.edges.empty() || loop.nodes.empty()) {
            continue;
        }
        const GraphNodeId anchor = loop.nodes.front();
        if (anchor == kInvalidId || anchor >= node_count) {
            continue;
        }

        const float height = node_height(anchor);
        Mesh ring = build_roundabout(loop, graph, centerlines, profiles, height,
                                     cfg.special.roundabout);
        if (ring.vertices.empty() || ring.indices.empty()) {
            // Rejected by the config actually passed in -- a mini-roundabout, say
            // -- so its edges go back to the ordinary path and must be extruded.
            for (EdgeId e : loop.edges) {
                if (e != kInvalidId && e < edge_count) {
                    m_consumed_edges[e] = false;
                }
            }
            continue;
        }

        double widest = 0.0;
        for (EdgeId e : loop.edges) {
            if (e != kInvalidId && e < edge_count) {
                widest = std::max(widest, 0.5 * static_cast<double>(profiles[e].total_width()));
            }
        }

        Junction junction;
        junction.node = anchor;
        junction.center = loop.center;
        junction.height = height;
        junction.mesh = std::move(ring);
        junction.footprint =
            circle_ring(loop.center, loop.radius + widest, cfg.special.roundabout.segments);
        junction.kind = JunctionKind::Roundabout;
        junction.is_roundabout = true;
        junction.valid = true;

        roundabout_junctions[l] = std::move(junction);
        roundabout_built[l] = true;
        ++m_stats.roundabouts;
    }

    // ── Compaction ───────────────────────────────────────────────────────────
    // Ascending GraphNodeId, on the calling thread, so the emitted list is
    // identical run to run whatever the scheduling.
    out.reserve(node_count / 4 + loops.size());

    for (size_t n = 0; n < node_count; ++n) {
        const int loop_index = loop_at_node[n];
        if (loop_index >= 0 && roundabout_built[static_cast<size_t>(loop_index)]) {
            out.push_back(std::move(roundabout_junctions[static_cast<size_t>(loop_index)]));
        }

        JunctionNodeSolve& slot = solves[n];
        if (!slot.emit) {
            continue;
        }

        Junction junction;
        junction.node = static_cast<GraphNodeId>(n);
        junction.center = graph.nodes()[n].position;
        junction.height = slot.height;
        junction.arms = std::move(slot.arms);
        junction.ends = std::move(slot.ends);
        junction.polygon = std::move(slot.polygon);
        junction.curb = std::move(slot.curb);
        junction.mesh = std::move(slot.mesh);
        junction.footprint = std::move(slot.footprint);
        junction.kind = slot.kind;
        junction.valid = slot.valid;

        switch (junction.kind) {
            case JunctionKind::Intersection:
                ++m_stats.junctions;
                if (junction.polygon.self_intersecting) {
                    ++m_stats.self_intersecting;
                }
                break;
            case JunctionKind::Degenerate:
                ++m_stats.degenerate;
                break;
            case JunctionKind::Taper:
                ++m_stats.tapers;
                break;
            case JunctionKind::DeadEnd:
                ++m_stats.dead_ends;
                break;
            case JunctionKind::Roundabout:
                break;
        }

        out.push_back(std::move(junction));
    }

    const auto finished = std::chrono::steady_clock::now();
    m_stats.build_ms =
        std::chrono::duration<double, std::milli>(finished - m_solve->started).count();

    spdlog::info("JunctionBuilder: Solved {} junctions, {} roundabouts, {} tapers, {} dead ends "
                 "({} degenerate, {} self-intersecting, {} over-trimmed edges, "
                 "{} with a dropped kerb) in {:.1f} ms",
                 m_stats.junctions, m_stats.roundabouts, m_stats.tapers, m_stats.dead_ends,
                 m_stats.degenerate, m_stats.self_intersecting, m_stats.over_trimmed_edges,
                 m_dropped_kerb_junctions, m_stats.build_ms);

    return out;
}

// ============================================================================
// Junction plateaus
// ============================================================================

void apply_junction_plateaus(const Centerline& centerline, double trim_from, double trim_to,
                             float level_from, float level_to, std::vector<float>& heights) {
    const size_t count = centerline.stations.size();
    if (count == 0u || heights.size() != count) {
        return;
    }

    const bool cut_from = std::isfinite(trim_from) && trim_from > 0.0;
    const bool cut_to = std::isfinite(trim_to) && trim_to > 0.0;
    if (!cut_from && !cut_to) {
        return;
    }

    const double base = centerline.stations.front().arclength;
    const double back = centerline.stations.back().arclength;
    const double span = back - base;

    // The last station the from-plateau covers, and the first the to-plateau
    // covers. Both deliberately overshoot their cut by one station, so a reslice
    // at exactly `trim` interpolates between two plateau stations.
    size_t from_last = count - 1u;
    if (cut_from) {
        for (size_t j = 0; j < count; ++j) {
            if (centerline.stations[j].arclength - base >= trim_from) {
                from_last = j;
                break;
            }
        }
    }

    size_t to_first = 0u;
    if (cut_to) {
        for (size_t j = count; j-- > 0;) {
            if (back - centerline.stations[j].arclength >= trim_to) {
                to_first = j;
                break;
            }
        }
    }

    if (!cut_to) {
        for (size_t j = 0; j <= from_last; ++j) heights[j] = level_from;
        return;
    }
    if (!cut_from) {
        for (size_t j = to_first; j < count; ++j) heights[j] = level_to;
        return;
    }

    if (from_last < to_first) {
        // The ordinary case: the two plateaus have stations of their own.
        for (size_t j = 0; j <= from_last; ++j) heights[j] = level_from;
        for (size_t j = to_first; j < count; ++j) heights[j] = level_to;
        return;
    }

    // They meet. Nothing can put both mouths on their own junction plane, so the
    // disputed stations are shared by arclength rather than won by whichever
    // plateau was written last.
    for (size_t j = 0; j < to_first; ++j) heights[j] = level_from;
    for (size_t j = from_last + 1u; j < count; ++j) heights[j] = level_to;
    for (size_t j = to_first; j <= from_last; ++j) {
        const double t =
            span > kMinPlateauSpan ? (centerline.stations[j].arclength - base) / span : 0.0;
        heights[j] = static_cast<float>(static_cast<double>(level_from)
                                        + (static_cast<double>(level_to)
                                           - static_cast<double>(level_from)) * t);
    }
}

} // namespace stratum::osm::road
