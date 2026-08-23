/**
 * @file road_elevation.cpp
 * @brief Graph-aware vertical solve over one global height array
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### One array, not one array per edge
 *
 * The solve is a projection onto the intersection of convex sets. Every
 * constraint here -- a grade limit `|dh| <= g*ds`, a curvature limit
 * `|g2 - g1| <= k*ds` -- is a band around a linear functional of the heights,
 * so each one is a closed convex set and the feasible region is their
 * intersection. That intersection is never empty: a perfectly flat network
 * satisfies all of them at once. Cyclic projection onto convex sets therefore
 * converges -- for the relaxed projections used at the fine level as much as
 * for the exact ones, since both are non-expansive. Each correction is the
 * smallest one that satisfies its constraint, so the solve settles on a profile
 * close to the terrain it started from rather than on an arbitrary feasible
 * one.
 *
 * The array is GLOBAL and node heights are VARIABLES IN IT. Every edge's first
 * and last station is an alias of its from/to node variable rather than a copy,
 * so an arm that pulls a junction down pulls every other arm of that junction
 * down in the same step. Solving per-edge and reconciling the ends afterwards
 * tears the network: the reconciliation moves the ends, which violates the grade
 * limit next to them, which the per-edge solve has already finished enforcing.
 *
 * Layout:
 *
 * @code
 *     vars[0 .. node_count)                        node heights
 *     vars[node_count .. node_count + interiors)   interior stations, edge by edge
 * @endcode
 *
 * so `var_of(edge, 0) == edge.from`, `var_of(edge, last) == edge.to`, and
 * everything between is a run of consecutive indices. Flat arrays, indexed
 * loops, no hashing: the sweep is cache-friendly and the result cannot depend on
 * the traversal order of an unordered container.
 *
 * ### Gauss-Seidel, in a canonical order
 *
 * Corrections are applied in place, so a constraint sees its predecessor's
 * correction within the same sweep. That is Gauss-Seidel, and it is worth an
 * order of magnitude over Jacobi here: on a chain, an in-place sweep carries a
 * correction the whole length of an edge, where Jacobi advances it one station
 * per sweep. Measured on a 20x20 grid of residential streets over a 50% slope,
 * both forms at their best relaxation factor: Jacobi spent the whole
 * 256-iteration budget and finished 2.2x over the grade limit, this form
 * finishes 1.07x over it. On a rolling surface where both converge, Jacobi took
 * 90 sweeps to this form's 64.
 *
 * The price of applying in place is that the sweep ORDER decides which point of
 * the feasible set the solve lands on. EdgeId order -- the obvious choice -- is
 * a position in the graph's edge list, which is a position in the file, so
 * re-exporting the same area with the ways in a different order would move the
 * road: no golden-file test would hold, and a re-import of identical data would
 * produce a different mesh. The edges are therefore swept in an order sorted by
 * OSM identity, which is a property of the survey rather than of the file
 * layout, so two graphs built from the same ways in any order sweep the same
 * constraints in the same sequence and produce bit-identical heights. The sweep
 * direction alternates each iteration, forward then backward, which is symmetric
 * Gauss-Seidel and removes the bias a single direction leaves on long roads.
 *
 * Two accelerations sit on top, and both are needed to keep a long rural edge
 * inside the default 256-iteration budget: the grade constraint is applied at
 * several strides, so a correction crosses an edge in one sweep rather than
 * diffusing across it over O(n^2) of them, and the finest of those levels is
 * over-relaxed. See kFineRelaxation and the stride note below. A cheap node-only
 * pre-pass runs first and its result is carried into each edge's interior; see
 * kNodePrepassCap.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API, and nothing from src/procgen. The terrain is reached only through the
 * HeightSampler callback.
 */

#include "osm/road/road_elevation.hpp"

#include "osm/road/junction_special.hpp"

#include <TaskScheduler.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tunables that are not user-facing
// ============================================================================

/**
 * @brief Shortest span, in metres, that still has a well-defined gradient
 *
 * A bevelled joint is two stations sharing one position and one arclength (see
 * Station::is_bevel), so zero-length spans are normal input rather than bad
 * data. The grade projection handles them naturally -- the limit is zero, so the
 * pair is forced to one height, which is correct because they are one point --
 * but the curvature projection divides by the span and must skip them.
 */
constexpr double kMinSpan = 1e-6;

/// Below this, a computed correction is noise and applying it only costs a write
constexpr double kNegligibleMove = 1e-12;

/**
 * @brief Over-relaxation of the station-to-station grade projection
 *
 * 1.0 applies exactly the minimum-norm projection onto the violated constraint.
 * Values in (1, 2) are SOR: still convergent, because a relaxed projection stays
 * non-expansive across that whole range, and much faster where a sweep behaves
 * like a diffusion -- which is what a run of neighbouring grade constraints on a
 * long edge is.
 *
 * Only the NEIGHBOUR level is over-relaxed. Over-relaxation buys speed by moving
 * past the constraint, which is harmless when the neighbouring constraint on the
 * other side of a station pulls back, and harmful when a variable is governed by
 * one constraint alone and the exact projection is already the right answer. The
 * chord and the coarse strides are exactly that case: they are the levels that
 * make the big global moves, so overshooting them relocates a whole road. A
 * 300 m residential road on a 20% slope, over-relaxed at the chord, settles
 * about twice as far from the ground as it needs to, and enough overshoot tips
 * the profile from climbing to falling.
 *
 * Measured on 2 km of road over the shipping procedural surface -- 50 m of
 * relief in six octaves, whose top octave alone reaches a 190% slope between
 * samples 2 m apart. Sweeps to convergence, and the mean distance the solved
 * road ends up from the ground:
 *
 * | omega | motorway | primary | residential | service | mean offset drift |
 * |-------|----------|---------|-------------|---------|-------------------|
 * | 1.0   | 194      | 121     | 145         | 257     | baseline          |
 * | 1.7   | 162      | 97      | 125         | 235     | under 1 cm        |
 * | 1.9   | 133      | 88      | 106         | 195     | 2 to 8 cm         |
 *
 * 1.9 is faster still, but every overshoot leaves the road slightly further
 * from the terrain than it needed to be, and by 1.9 that is visible in the last
 * column. 1.7 buys most of the speed at none of that cost.
 */
constexpr double kFineRelaxation = 1.7;

/**
 * @brief Sweep budget of the node-only pre-pass
 *
 * The pre-pass relaxes node heights against edge CHORDS alone: one projection
 * per edge per sweep, against roughly three per station in the full solve. A
 * pre-pass sweep therefore costs on the order of a tenth of a station sweep, and
 * it does the one thing the station solve is slowest at -- propagating a
 * correction across the whole network -- so it is given a budget several times
 * the station budget rather than a fraction of it. It stops on the same
 * convergence test and usually long before the cap.
 *
 * The effective budget is min(ElevationConfig::max_iterations, this), so at the
 * shipping 256 iterations the pre-pass gets all of them and this ceiling only
 * binds when a caller raises the cap.
 *
 * Measured on a 20x20 grid of residential streets, 8% limit, at the end of the
 * default budget -- steepest solved gradient as a multiple of the limit:
 *
 * | ground slope | pre-pass 64 | pre-pass uncapped |
 * |--------------|-------------|-------------------|
 * | 10%          | 1.40x       | 1.003x            |
 * | 20%          | 4.77x       | 1.021x            |
 * | 50%          | 17.9x       | 1.073x            |
 *
 * Total wall clock is unchanged, because the sweeps the pre-pass adds are the
 * cheap ones and the station sweeps it saves are the expensive ones.
 */
constexpr int kNodePrepassCap = 512;

/**
 * @brief Passes of the bridge and tunnel override loop
 *
 * A node shared by two bridges takes the larger lift of the two, which changes
 * the deck of the bridge that asked for the smaller one, which may then need
 * more lift. The loop re-derives the lifts until they stop growing. Convergence
 * is monotone -- lifts only ever increase -- so this terminates; the cap is
 * there for pathological extracts, not for correctness.
 */
constexpr int kOverridePasses = 8;

/*
 * Why the grade sweep runs at several strides
 * -------------------------------------------
 *
 * A grade constraint between two stations that are not neighbours -- `|dh| <= g *
 * (arc_j - arc_i)` -- is IMPLIED by the chain of neighbour constraints between
 * them, by the triangle inequality. Projecting onto it therefore removes no
 * feasible profile: it is a redundant constraint, and redundant constraints are
 * free to add to a projection method.
 *
 * They are added because Gauss-Seidel on neighbour constraints alone is a
 * diffusion: a correction travels one station per sweep, so a chain of N
 * stations needs O(N^2) sweeps to settle. That is fine on an 80 m residential
 * block and hopeless on a 2 km motorway edge, which is exactly the case where
 * the grade limit binds hardest. Sweeping strides 1, 2, 4, ... and the full
 * chord as well couples the two ends of an edge directly, and the error falls in
 * O(log N) sweeps instead. It is the same idea as a multigrid V-cycle, done on
 * the constraint list rather than on a hierarchy of meshes.
 *
 * The cost is bounded: the levels hold N, N/2, N/4, ... constraints, so the
 * whole ladder is under 2N projections, twice the work of the fine level alone.
 */

/// Item count below which a parallel phase runs on the calling thread instead
constexpr size_t kParallelMinItems = 256;

/// Items handed to one enkiTS worker at a time; see road_network_builder.cpp
constexpr uint32_t kItemsPerRange = 32;

// ============================================================================
// Per-edge solve slot
// ============================================================================

/**
 * @brief Everything the relaxation needs to know about one graph edge
 *
 * Built once, read every sweep. Deliberately flat and small: the sweep touches
 * one of these per edge and then walks three contiguous arrays.
 */
struct EdgeSlot {
    /// Stations in this edge's centerline; 0 when the edge takes no part
    uint32_t station_count = 0;

    /// Absolute index in `vars` of station 1; stations 0 and last are node vars
    uint32_t interior_base = 0;

    /// Base index of this edge's run in the flat per-station arrays
    uint32_t station_base = 0;

    GraphNodeId from = kInvalidId;
    GraphNodeId to = kInvalidId;

    /// Maximum longitudinal gradient of this edge's road class
    double grade = 0.08;

    /// Arclength from the first station to the last, metres
    double length = 0.0;

    /// Edge participates in the solve at all
    bool active = false;

    /// Deck or bore: heights are imposed, not relaxed. See solve() step 4.
    bool is_bridge = false;
    bool is_tunnel = false;

    /**
     * @name This tunnel edge enters the ground at that end rather than starting in it
     *
     * Set only for a tunnel, only when ElevationConfig::tunnel_portal_at_surface,
     * and only at an end whose node is a PORTAL node -- one that also carries a
     * surface arm, or a dead end. An end shared with another tunnel edge is
     * INTERIOR to a longer bore and keeps the pre-ramp behaviour, because a
     * ramp there would surface the tunnel in the middle of the hill.
     */
    ///@{
    bool ramp_from = false;
    bool ramp_to = false;
    ///@}

    /// Shorthand for "relaxed station by station" -- active and not an override
    [[nodiscard]] bool relaxed() const { return active && !is_bridge && !is_tunnel; }

    /// This tunnel enters the ground at at least one of its ends
    [[nodiscard]] bool ramped() const { return ramp_from || ramp_to; }
};

// ============================================================================
// Tunnel ramp
// ============================================================================

/**
 * @brief Round the kinks out of a tunnel ramp, holding both ends
 *
 * build_tunnel_ramp() seeds a profile that is the maximum of a straight deep
 * running line and one descent cone per portal. That is grade-feasible by
 * construction -- a maximum of functions with slope at most @p grade has slope
 * at most @p grade -- but it is not CURVATURE-feasible: where a cone meets the
 * deep line the gradient steps by the whole grade limit in one station, and on a
 * tunnel too short to reach depth the two cones meet in a V.
 *
 * This is the same pair of projections the global relaxation applies, run over
 * one edge with its two end stations held: the ends are the portal node heights
 * and moving them would tear the tunnel off its approaches, which is the defect
 * ElevationConfig::tunnel_portal_at_surface exists to fix. Every kink here is a
 * SAG, so rounding one only ever lifts the profile; the deep run therefore
 * cannot be pushed further into the hillside than build_tunnel_ramp() put it.
 *
 * @param h         Profile to smooth in place; fewer than three entries is a no-op
 * @param arc       Arclength per station, rebased to zero at the first, metres
 * @param grade     Maximum longitudinal gradient of this edge's road class
 * @param curvature ElevationConfig::max_grade_change_per_m
 * @param budget    Sweep cap
 * @param epsilon   Stop once no station moves more than this in a sweep, metres
 */
void smooth_tunnel_ramp(std::vector<double>& h,
                        const double* arc,
                        double grade,
                        double curvature,
                        int budget,
                        double epsilon) {
    const size_t n = h.size();
    if (n < 3u) {
        return;
    }

    const auto held = [n](size_t i) { return i == 0u || i + 1u == n; };

    for (int iteration = 0; iteration < budget; ++iteration) {
        double move = 0.0;
        const bool forward = (iteration % 2) == 0;

        const auto shift = [&](size_t i, double step) {
            h[i] += step;
            move = std::max(move, std::fabs(step));
        };

        // Curvature first: it is what the seed actually violates.
        for (size_t k = 0; k + 2u < n; ++k) {
            const size_t j = forward ? k : n - 3u - k;
            const double ds1 = arc[j + 1u] - arc[j];
            const double ds2 = arc[j + 2u] - arc[j + 1u];
            if (ds1 < kMinSpan || ds2 < kMinSpan) {
                continue;       // a bevel pair: two stations at one point
            }

            const double inv1 = 1.0 / ds1;
            const double inv2 = 1.0 / ds2;
            const double delta = (h[j + 2u] - h[j + 1u]) * inv2 - (h[j + 1u] - h[j]) * inv1;
            const double limit = curvature * 0.5 * (ds1 + ds2);
            const double excess = delta - std::clamp(delta, -limit, limit);
            if (std::fabs(excess) <= kNegligibleMove) {
                continue;
            }

            const double wa = held(j) ? 0.0 : inv1;
            const double wb = held(j + 1u) ? 0.0 : -(inv1 + inv2);
            const double wc = held(j + 2u) ? 0.0 : inv2;
            const double denom = wa * wa + wb * wb + wc * wc;
            if (denom < kNegligibleMove) {
                continue;
            }

            const double lambda = excess / denom;
            if (wa != 0.0) {
                shift(j, -lambda * wa);
            }
            if (wb != 0.0) {
                shift(j + 1u, -lambda * wb);
            }
            if (wc != 0.0) {
                shift(j + 2u, -lambda * wc);
            }
        }

        // Then the grade limit, so a rounded sag cannot leave a span steeper
        // than the road class allows. Exact projection, no over-relaxation: this
        // is a short chain that converges in a handful of sweeps anyway, and
        // overshooting it would move the deep run off the depth it was placed at.
        for (size_t k = 0; k + 1u < n; ++k) {
            const size_t j = forward ? k : n - 2u - k;
            const double ds = arc[j + 1u] - arc[j];
            const double dh = h[j + 1u] - h[j];
            const double over = std::fabs(dh) - grade * ds;
            if (over <= kNegligibleMove) {
                continue;
            }

            const bool hold_a = held(j);
            const bool hold_b = held(j + 1u);
            if (hold_a && hold_b) {
                continue;       // a two-station edge: both ends are node heights
            }

            const double correction = (dh > 0.0 ? 1.0 : -1.0) * over;
            if (hold_a) {
                shift(j + 1u, -correction);
            } else if (hold_b) {
                shift(j, correction);
            } else {
                shift(j, correction * 0.5);
                shift(j + 1u, -correction * 0.5);
            }
        }

        if (move < epsilon) {
            break;
        }
    }
}

/**
 * @brief Grade-limited descent from a tunnel's portals to its deep running line
 *
 * ### The shape
 *
 * Three curves, and the profile is the highest of them at every station:
 *
 * - **The deep running line.** The straight line between the two node heights,
 *   shifted down by the single uniform `drop` that puts it at least @p depth
 *   below the terrain everywhere on the edge. That is EXACTLY the line the
 *   pre-ramp override produced, and it is kept straight for the same reason a
 *   bridge deck is: a bore that followed the hill above it would inherit the
 *   terrain's noise, and its gradient comes from its two ends, not from the
 *   ground it passes under.
 * - **One descent cone per portal**, `h_end - grade * distance_from_that_end`.
 *   The cone is the steepest the roadway is allowed to leave the surface at, so
 *   the profile leaves the portal at exactly the road class limit and meets the
 *   deep line where the limit says it can. No ramp-length tunable is introduced
 *   because the grade limit already fixes it: a 4% motorway takes 200 m to reach
 *   8 m, a 10% service road takes 80 m, and both are right.
 *
 * Taking the maximum is what makes the short-tunnel case fall out rather than
 * needing a rule: when the two cones cross above the deep line the profile is a
 * V that never reaches full depth, which is the documented behaviour -- a 40 m
 * tunnel under a 5 m mound is a 5 m-deep tunnel, and steepening it to reach 8 m
 * would put a hole in the mound.
 *
 * An end that is NOT ramping contributes no cone and takes no share of the drop:
 * it is interior to a longer bore, its node height was already pushed to depth
 * by the override loop, and the profile simply starts there.
 *
 * The first and last entries are assigned from @p h_from and @p h_to directly
 * rather than left to the maximum, so an arm terminates at exactly its node
 * height even if the chord relaxation stopped a hair short of feasible.
 *
 * @param arc       Arclength per station, rebased to zero at the first, metres
 * @param terrain   Terrain height under each station, metres
 * @param count     Station count; must be at least 2
 * @param h_from,h_to Solved node heights at the two ends
 * @param grade     Maximum longitudinal gradient of this edge's road class
 * @param depth     ElevationConfig::tunnel_depth
 * @param ramp_from,ramp_to Whether that end is a portal; see EdgeSlot::ramp_from
 * @param curvature ElevationConfig::max_grade_change_per_m
 * @param budget    Smoothing sweep cap
 * @param epsilon   Smoothing convergence tolerance, metres
 * @param out       Profile, resized to @p count
 */
void build_tunnel_ramp(const double* arc,
                       const float* terrain,
                       uint32_t count,
                       double h_from,
                       double h_to,
                       double grade,
                       double depth,
                       bool ramp_from,
                       bool ramp_to,
                       double curvature,
                       int budget,
                       double epsilon,
                       std::vector<double>& out) {
    out.assign(count, h_from);
    if (count < 2u) {
        return;
    }

    const uint32_t last = count - 1u;
    const double span = arc[last];

    // The uniform drop, measured against the straight deck exactly as the
    // pre-ramp override measured it. Clamped at zero: an edge already under the
    // terrain by more than `depth` asks for no drop at all.
    double drop = 0.0;
    for (uint32_t j = 0; j < count; ++j) {
        const double t = span > kMinSpan ? arc[j] / span : 0.0;
        const double deck = h_from + (h_to - h_from) * t;
        drop = std::max(drop, deck - (static_cast<double>(terrain[j]) - depth));
    }
    drop = std::max(drop, 0.0);

    const double weight_from = ramp_from ? 1.0 : 0.0;
    const double weight_to = ramp_to ? 1.0 : 0.0;

    for (uint32_t j = 0; j < count; ++j) {
        const double t = span > kMinSpan ? arc[j] / span : 0.0;
        const double deck = h_from + (h_to - h_from) * t;

        // Only a ramping end takes its share of the drop, so a half-ramped edge
        // runs from a deep interior node up to a surface portal on one straight
        // line rather than stepping at the node it shares with the next bore.
        const double share = weight_from * (1.0 - t) + weight_to * t;
        double height = deck - drop * share;

        if (ramp_from) {
            height = std::max(height, h_from - grade * arc[j]);
        }
        if (ramp_to) {
            height = std::max(height, h_to - grade * std::max(span - arc[j], 0.0));
        }
        out[j] = height;
    }

    out.front() = h_from;
    out.back() = h_to;

    smooth_tunnel_ramp(out, arc, grade, curvature, budget, epsilon);

    // The smoother holds them, but the contract that an arm terminates at
    // exactly its node height is worth more than one branch.
    out.front() = h_from;
    out.back() = h_to;
}

// ============================================================================
// Parallel helper
// ============================================================================

/**
 * @brief Run @p fn over [0, count) as sub-ranges, in parallel when it pays
 *
 * Follows the pattern in road_network_builder.cpp: the scheduler costs one
 * thread creation per hardware thread, which dominates the work on a small
 * extract, so small counts stay serial. Results are identical either way because
 * every worker writes into its own pre-sized slot.
 *
 * @param scheduler Initialised scheduler, or nullptr to force the serial path
 * @param count     Number of items
 * @param fn        Callable taking a half-open [begin, end) range
 */
template <typename Fn>
void run_ranges(enki::TaskScheduler* scheduler, size_t count, Fn&& fn) {
    if (count == 0) {
        return;
    }
    if (scheduler == nullptr) {
        fn(size_t{0}, count);
        return;
    }
    enki::TaskSet sweep(static_cast<uint32_t>(count),
                        [&](enki::TaskSetPartition range, uint32_t /*threadnum*/) {
                            fn(static_cast<size_t>(range.start), static_cast<size_t>(range.end));
                        });
    sweep.m_MinRange = kItemsPerRange;
    scheduler->AddTaskSetToPipe(&sweep);
    scheduler->WaitforTask(&sweep);
}

} // namespace

// ============================================================================
// Road class grade table
// ============================================================================

float max_grade_for(RoadType type, const ElevationConfig& cfg) {
    float grade = cfg.max_grade_residential;

    switch (type) {
        case RoadType::Motorway:    grade = cfg.max_grade_motorway;    break;
        case RoadType::Trunk:       grade = cfg.max_grade_trunk;       break;
        case RoadType::Primary:     grade = cfg.max_grade_primary;     break;
        case RoadType::Secondary:   grade = cfg.max_grade_secondary;   break;
        case RoadType::Tertiary:    grade = cfg.max_grade_tertiary;    break;
        case RoadType::Residential: grade = cfg.max_grade_residential; break;
        case RoadType::Service:     grade = cfg.max_grade_service;     break;

        // A footway or a cycleway may be as steep as a path; none of the three
        // carries a vehicle, so they share the loosest limit in the table.
        case RoadType::Footway:
        case RoadType::Cycleway:
        case RoadType::Path:        grade = cfg.max_grade_path;        break;

        // The middle of the table. An unclassified way is far more often a
        // residential street than a motorway.
        case RoadType::Unknown:     grade = cfg.max_grade_residential; break;
    }

    // A zero or negative limit would pin every edge dead flat and, worse, make
    // the feasible set of a closed loop over sloped terrain a single point that
    // the projections crawl towards. The contract is "always > 0".
    return std::max(grade, 1e-4f);
}

// ============================================================================
// Solver
// ============================================================================

void RoadElevationSolver::clear() {
    m_edges.clear();
    m_node_heights.clear();
    m_stats = Stats{};
    m_solved = false;
}

void RoadElevationSolver::solve(const RoadGraph& graph,
                                const std::vector<Centerline>& centerlines,
                                const HeightSampler& sampler,
                                const ElevationConfig& cfg) {
    const auto started = std::chrono::steady_clock::now();
    clear();

    const std::vector<GraphNode>& nodes = graph.nodes();
    const std::vector<GraphEdge>& edges = graph.edges();

    if (!sampler) {
        spdlog::warn("RoadElevationSolver: No height sampler, leaving the network unsolved");
        return;
    }
    if (centerlines.size() != edges.size()) {
        spdlog::warn("RoadElevationSolver: {} centerlines for {} edges, leaving the network "
                     "unsolved",
                     centerlines.size(), edges.size());
        return;
    }

    const size_t node_count = nodes.size();
    const size_t edge_count = edges.size();

    m_edges.assign(edge_count, EdgeElevation{});
    m_node_heights.assign(node_count, 0.0f);

    if (node_count == 0 || edge_count == 0) {
        m_solved = true;
        const auto finished = std::chrono::steady_clock::now();
        m_stats.solve_ms = std::chrono::duration<double, std::milli>(finished - started).count();
        spdlog::info("RoadElevationSolver: Nothing to solve — {} nodes, {} edges", node_count,
                     edge_count);
        return;
    }

    // ------------------------------------------------------------------------
    // Layout. One pass to size the flat arrays, so nothing reallocates later and
    // every index handed to the sweep stays valid for the whole solve.
    // ------------------------------------------------------------------------

    std::vector<EdgeSlot> slots(edge_count);

    size_t interior_total = 0;
    size_t station_total = 0;

    for (size_t i = 0; i < edge_count; ++i) {
        const GraphEdge& edge = edges[i];
        const Centerline& line = centerlines[i];
        EdgeSlot& slot = slots[i];

        const bool usable = line.is_valid() && edge.from < node_count && edge.to < node_count;
        if (!usable) {
            continue;
        }

        slot.active = true;
        slot.station_count = static_cast<uint32_t>(line.stations.size());
        slot.from = edge.from;
        slot.to = edge.to;
        slot.grade = static_cast<double>(max_grade_for(edge.type, cfg));
        slot.length = line.stations.back().arclength - line.stations.front().arclength;

        // Classified from the EXPLICIT tags only. layer=* is a rendering-order
        // hint, not a structure: `highway=residential, layer=-1` is how the LOWER
        // road at a grade separation is tagged, and reading it as a tunnel buries
        // an ordinary street tunnel_depth metres underground and suppresses its
        // carve. Layer already has its job -- P1 splits grade-separation nodes by
        // it (RoadGraph::slot_layer) -- and nothing else here needs to read it.
        //
        // A bridge deck and a tunnel bore are mutually exclusive. A way carrying
        // both tags is describing a tunnel inside an embankment; the bridge wins,
        // because burying a deck is a visible failure and a shallow bore is not.
        slot.is_bridge = edge.is_bridge;
        slot.is_tunnel = !edge.is_bridge && edge.is_tunnel;

        slot.station_base = static_cast<uint32_t>(station_total);
        slot.interior_base = static_cast<uint32_t>(node_count + interior_total);

        // is_valid() guarantees at least two stations, so the subtraction cannot
        // wrap.
        station_total += slot.station_count;
        interior_total += slot.station_count - 2u;
    }

    // ------------------------------------------------------------------------
    // Tunnel portal nodes.
    //
    // A tunnel edge's end is a PORTAL when the road actually enters the ground
    // there, and an INTERIOR node when it does not. The difference is decided by
    // what else meets the node, not by the tunnel edge:
    //
    // - A node carrying a surface arm as well is a portal. This is the standard
    //   encoding -- approach way, `tunnel=yes` way, approach way -- and the
    //   portal node is where the covered stretch meets the open road, so it sits
    //   at the approach surface exactly as a bridge abutment sits at the deck.
    // - A node where only tunnel edges meet is INTERIOR to a longer bore. A long
    //   tunnel is split into several GraphEdges at its interior nodes, and
    //   ramping to the surface at each of them would make the roadway breach the
    //   hillside every few hundred metres. Those nodes keep the pre-ramp
    //   behaviour and are dropped to depth by the override loop below.
    // - A dead end -- one arm in total -- is a portal. The way simply stops
    //   there; a mouth is the useful reading and a buried stub is not.
    // ------------------------------------------------------------------------
    if (cfg.tunnel_portal_at_surface) {
        std::vector<uint32_t> arm_count(node_count, 0u);
        std::vector<uint32_t> tunnel_arm_count(node_count, 0u);

        for (const EdgeSlot& slot : slots) {
            if (!slot.active) {
                continue;
            }
            ++arm_count[slot.from];
            ++arm_count[slot.to];
            if (slot.is_tunnel) {
                ++tunnel_arm_count[slot.from];
                ++tunnel_arm_count[slot.to];
            }
        }

        for (EdgeSlot& slot : slots) {
            if (!slot.active || !slot.is_tunnel) {
                continue;
            }
            const auto is_portal = [&](GraphNodeId n) {
                return arm_count[n] == 1u || tunnel_arm_count[n] < arm_count[n];
            };
            slot.ramp_from = is_portal(slot.from);
            slot.ramp_to = is_portal(slot.to);
        }
    }

    // vars[0, node_count) are node heights; the rest are interior stations. The
    // aliasing of every edge's end stations onto its node variables lives here
    // and nowhere else.
    std::vector<double> vars(node_count + interior_total, 0.0);
    std::vector<float> terrain(station_total, 0.0f);

    // Arclength of each station, rebased to zero at its edge's first station.
    // Kept instead of per-span lengths because the multiscale grade sweep needs
    // the distance between stations that are not neighbours, and a difference of
    // two arclengths is exact where a running sum of spans would drift.
    std::vector<double> arcs(station_total, 0.0);

    const auto var_of = [&](const EdgeSlot& slot, uint32_t station) -> uint32_t {
        if (station == 0u) {
            return slot.from;
        }
        if (station + 1u == slot.station_count) {
            return slot.to;
        }
        return slot.interior_base + (station - 1u);
    };

    // ------------------------------------------------------------------------
    // Step 1 and 2. Sample the terrain. This is the expensive part of the solve
    // and the sampler is thread-safe by contract, so nodes and stations are
    // sampled concurrently. Every worker writes only its own slots.
    // ------------------------------------------------------------------------

    std::vector<float> node_terrain(node_count, 0.0f);

    const auto sample_nodes = [&](size_t begin, size_t end) {
        for (size_t n = begin; n < end; ++n) {
            node_terrain[n] = sampler(nodes[n].position.x, nodes[n].position.y);
        }
    };

    const auto sample_stations = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            const EdgeSlot& slot = slots[i];
            if (!slot.active) {
                continue;
            }
            const std::vector<Station>& stations = centerlines[i].stations;
            for (uint32_t j = 0; j < slot.station_count; ++j) {
                const Station& station = stations[j];
                terrain[slot.station_base + j] = sampler(station.position.x, station.position.y);
            }
        }
    };

    if (node_count + edge_count < kParallelMinItems) {
        sample_nodes(0, node_count);
        sample_stations(0, edge_count);
    } else {
        enki::TaskScheduler scheduler;
        scheduler.Initialize();
        run_ranges(&scheduler, node_count, sample_nodes);
        run_ranges(&scheduler, edge_count, sample_stations);
    }

    // Arclengths are already computed by the centerline builder, so rebasing them
    // is pure arithmetic and stays on this thread.
    for (size_t i = 0; i < edge_count; ++i) {
        const EdgeSlot& slot = slots[i];
        if (!slot.active) {
            continue;
        }
        const std::vector<Station>& stations = centerlines[i].stations;
        const double base = stations.front().arclength;
        for (uint32_t j = 0; j < slot.station_count; ++j) {
            arcs[slot.station_base + j] = std::max(stations[j].arclength - base, 0.0);
        }
    }

    // Seed. Node variables take the terrain under the node, interior stations
    // the terrain under the station.
    for (size_t n = 0; n < node_count; ++n) {
        vars[n] = static_cast<double>(node_terrain[n]);
    }
    for (size_t i = 0; i < edge_count; ++i) {
        const EdgeSlot& slot = slots[i];
        if (!slot.active) {
            continue;
        }
        for (uint32_t j = 1; j + 1u < slot.station_count; ++j) {
            vars[slot.interior_base + (j - 1u)] =
                static_cast<double>(terrain[slot.station_base + j]);
        }
    }

    // ------------------------------------------------------------------------
    // Step 3. Relaxation.
    // ------------------------------------------------------------------------

    // Canonical sweep order.
    //
    // Corrections are applied in place, so the sweep order decides WHICH point
    // of the feasible set the solve lands on. EdgeId order is the obvious
    // choice and the wrong one: an EdgeId is a position in the graph's edge
    // list, which is a position in the file, so re-exporting the same area with
    // the ways in a different order would move the road. That breaks golden-file
    // testing and makes a re-import produce a different mesh from identical
    // data.
    //
    // Sorting by OSM identity instead -- the parent way, then the two endpoint
    // node IDs -- gives an order that survives any renumbering, because those
    // IDs come from the survey rather than from the file layout. Two graphs
    // built from the same ways in different orders then sweep the same
    // constraints in the same sequence and produce bit-identical heights.
    std::vector<uint32_t> order(edge_count);
    for (size_t i = 0; i < edge_count; ++i) {
        order[i] = static_cast<uint32_t>(i);
    }
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
        const GraphEdge& ea = edges[a];
        const GraphEdge& eb = edges[b];
        if (ea.source_way != eb.source_way) {
            return ea.source_way < eb.source_way;
        }
        // A way visits each node once, so the two endpoint IDs separate its
        // splits. The EdgeId is the last resort and only reached by edges that
        // are indistinguishable by OSM identity, which cannot differ in the
        // solve either.
        const NodeId a_from = nodes[ea.from].osm_id;
        const NodeId b_from = nodes[eb.from].osm_id;
        if (a_from != b_from) {
            return a_from < b_from;
        }
        const NodeId a_to = nodes[ea.to].osm_id;
        const NodeId b_to = nodes[eb.to].osm_id;
        if (a_to != b_to) {
            return a_to < b_to;
        }
        return a < b;
    });

    // Set during the override phase for every node a bridge or tunnel end lands
    // on. A pinned variable is excluded from the corrections rather than moved
    // and moved back, so the re-relaxation makes the APPROACHES meet the deck
    // instead of dragging the deck back down to the terrain.
    std::vector<uint8_t> pinned(node_count, 0u);

    // ------------------------------------------------------------------------
    // Roundabout rings are levelled BEFORE the relaxation, not corrected after.
    //
    // The grade limit constrains a ring only against its own circumference, which
    // on any real roundabout is long enough to permit metres of fall around it.
    // The annulus that replaces those edges in P4 is one flat surface at one node
    // height and its carve is one flat disc, so any fall around the ring reappears
    // as a vertical crack at every approach mouth. Seeding all of the loop's nodes
    // at their mean and pinning them uses the same machinery a bridge deck's
    // abutments use, and for the same reason: the approaches are what must move.
    // ------------------------------------------------------------------------
    size_t levelled_loops = 0;
    if (cfg.level_roundabouts) {
        for (const RoundaboutLoop& loop : find_roundabouts(graph, centerlines)) {
            if (!loop.valid || loop.nodes.size() < 2u) {
                continue;
            }
            double sum = 0.0;
            size_t counted = 0;
            for (const GraphNodeId id : loop.nodes) {
                if (id != kInvalidId && id < node_count) {
                    sum += vars[id];
                    ++counted;
                }
            }
            if (counted == 0) {
                continue;
            }
            const double level = sum / static_cast<double>(counted);
            for (const GraphNodeId id : loop.nodes) {
                if (id != kInvalidId && id < node_count) {
                    vars[id] = level;
                    pinned[id] = 1u;
                }
            }
            ++levelled_loops;
        }
        if (levelled_loops > 0) {
            spdlog::debug("RoadElevationSolver: levelled {} roundabout ring(s) to one height each",
                          levelled_loops);
        }
    }

    double sweep_move = 0.0;

    const auto is_pinned = [&](uint32_t var) {
        return var < node_count && pinned[var] != 0u;
    };

    const auto move_var = [&](uint32_t var, double step) {
        vars[var] += step;
        sweep_move = std::max(sweep_move, std::fabs(step));
    };

    /**
     * Project one span onto |h_b - h_a| <= grade * ds, moving BOTH ends.
     *
     * @p omega scales the correction; see kFineRelaxation for why the fine level
     * gets more than 1 and the coarse levels do not.
     */
    const auto apply_grade = [&](uint32_t a, uint32_t b, double ds, double grade, double omega) {
        const double dh = vars[b] - vars[a];
        const double over = std::fabs(dh) - grade * ds;
        if (over <= kNegligibleMove) {
            return;
        }

        const bool pin_a = is_pinned(a);
        const bool pin_b = is_pinned(b);
        if (pin_a && pin_b) {
            return;
        }

        // Moving only the downhill end would march the whole profile downhill:
        // every span in a steep run would push its lower station further down,
        // and a long hillside road would end up buried. Splitting the correction
        // is the minimum-norm correction and leaves the mean height alone.
        const double sign = dh > 0.0 ? 1.0 : -1.0;
        const double correction = sign * over * omega;
        if (pin_a) {
            move_var(b, -correction);
        } else if (pin_b) {
            move_var(a, correction);
        } else {
            move_var(a, correction * 0.5);
            move_var(b, -correction * 0.5);
        }
    };

    /// Project one station triple onto |g2 - g1| <= max_grade_change_per_m * ds
    const auto apply_curvature = [&](uint32_t a, uint32_t b, uint32_t c, double ds1, double ds2) {
        if (ds1 < kMinSpan || ds2 < kMinSpan) {
            return;     // a bevel pair: two stations at one point, no curvature
        }

        const double inv1 = 1.0 / ds1;
        const double inv2 = 1.0 / ds2;
        const double g1 = (vars[b] - vars[a]) * inv1;
        const double g2 = (vars[c] - vars[b]) * inv2;
        const double delta = g2 - g1;

        // The bound scales with the ground the gradient change is spread over,
        // so a dense run of stations in a curve is not penalised for being dense.
        const double limit = static_cast<double>(cfg.max_grade_change_per_m) * 0.5 * (ds1 + ds2);
        const double excess = delta - std::clamp(delta, -limit, limit);
        if (std::fabs(excess) <= kNegligibleMove) {
            return;
        }

        // (g2 - g1) is linear in the three heights with these partials. Stepping
        // along the negated gradient, scaled to cancel exactly `excess`, is the
        // minimum-norm projection: all three stations share the correction, so a
        // crest is flattened by lowering its peak AND lifting its shoulders
        // rather than by dragging the peak alone.
        const double wa = is_pinned(a) ? 0.0 : inv1;
        const double wb = is_pinned(b) ? 0.0 : -(inv1 + inv2);
        const double wc = is_pinned(c) ? 0.0 : inv2;

        const double denom = wa * wa + wb * wb + wc * wc;
        if (denom < kNegligibleMove) {
            return;
        }

        const double lambda = excess / denom;
        if (wa != 0.0) {
            move_var(a, -lambda * wa);
        }
        if (wb != 0.0) {
            move_var(b, -lambda * wb);
        }
        if (wc != 0.0) {
            move_var(c, -lambda * wc);
        }
    };

    /**
     * One relaxation run. Returns the residual of the last sweep and adds the
     * sweeps it performed to @p sweeps.
     *
     * `station_wise` false is the node pre-pass: only edge chords are projected,
     * which is the same solve on the node subgraph and costs a fraction of a
     * full sweep.
     */
    const auto relax = [&](int budget, bool station_wise, size_t& sweeps) -> double {
        double residual = 0.0;
        for (int iteration = 0; iteration < budget; ++iteration) {
            sweep_move = 0.0;
            const bool forward = (iteration % 2) == 0;

            for (size_t step = 0; step < edge_count; ++step) {
                // Canonical order, and reversed on alternate sweeps so neither
                // end of a long chain of edges is favoured.
                const size_t i = order[forward ? step : edge_count - 1 - step];
                const EdgeSlot& slot = slots[i];
                if (!slot.active) {
                    continue;
                }

                if (!station_wise || !slot.relaxed()) {
                    // Chord. For an override edge this is the deck's own grade
                    // limit; for every other edge during the pre-pass it is the
                    // coarse stand-in for its station chain.
                    apply_grade(slot.from, slot.to, slot.length, slot.grade, 1.0);
                    continue;
                }

                const uint32_t span_count = slot.station_count - 1u;
                const uint32_t base = slot.station_base;
                const auto ds_between = [&](uint32_t j0, uint32_t j1) {
                    return arcs[base + j1] - arcs[base + j0];
                };

                // Grade at every stride. See the stride note at the top of this file.
                apply_grade(slot.from, slot.to, slot.length, slot.grade, 1.0);
                uint32_t top = 1u;
                while (top * 2u <= span_count) {
                    top *= 2u;
                }
                for (uint32_t stride = top; stride >= 1u; stride >>= 1) {
                    const uint32_t blocks = span_count / stride;
                    for (uint32_t b = 0; b < blocks; ++b) {
                        const uint32_t k = forward ? b : blocks - 1u - b;
                        const uint32_t j0 = k * stride;
                        const uint32_t j1 = j0 + stride;
                        apply_grade(var_of(slot, j0), var_of(slot, j1), ds_between(j0, j1),
                                    slot.grade, stride == 1u ? kFineRelaxation : 1.0);
                    }
                    // The blocks tile only span_count - (span_count % stride) of
                    // the edge. Without this the tail stations never see a coarse
                    // constraint and lag behind the rest of the profile.
                    const uint32_t covered = blocks * stride;
                    if (stride > 1u && covered < span_count) {
                        apply_grade(var_of(slot, covered), var_of(slot, span_count),
                                    ds_between(covered, span_count), slot.grade, 1.0);
                    }
                }

                if (slot.station_count < 3u) {
                    continue;
                }
                const uint32_t triple_count = slot.station_count - 2u;
                for (uint32_t t = 0; t < triple_count; ++t) {
                    const uint32_t j = forward ? t : triple_count - 1u - t;
                    apply_curvature(var_of(slot, j), var_of(slot, j + 1u), var_of(slot, j + 2u),
                                    ds_between(j, j + 1u), ds_between(j + 1u, j + 2u));
                }
            }

            ++sweeps;
            residual = sweep_move;
            if (residual < static_cast<double>(cfg.convergence_epsilon)) {
                break;
            }
        }
        return residual;
    };

    const int budget = std::max(cfg.max_iterations, 1);

    // Node heights first, as the plan requires: every arm of a junction must end
    // at ONE height, and that height is decided by all of them together before
    // any arm is shaped.
    relax(std::min(budget, kNodePrepassCap), false, m_stats.iterations);

    // Carry the node correction into each edge's interior before the station
    // solve starts. Without this the pre-pass is wasted work: the interior
    // stations would still sit on raw terrain, the first station of an edge
    // would disagree with its node by the whole correction, and the station
    // sweep would spend its budget dragging the nodes back off the answer the
    // pre-pass just found. Shifting and tilting the sampled profile to meet the
    // solved ends keeps the local shape of the ground -- which is the part the
    // station solve is for -- and removes only the part already decided.
    for (size_t i = 0; i < edge_count; ++i) {
        const EdgeSlot& slot = slots[i];
        if (!slot.relaxed() || slot.station_count < 3u) {
            continue;
        }
        const uint32_t base = slot.station_base;
        const uint32_t last = slot.station_count - 1u;
        const double shift_from = vars[slot.from] - static_cast<double>(terrain[base]);
        const double shift_to = vars[slot.to] - static_cast<double>(terrain[base + last]);
        const double span = slot.length;
        for (uint32_t j = 1; j < last; ++j) {
            const double t = span > kMinSpan ? arcs[base + j] / span : 0.0;
            vars[slot.interior_base + (j - 1u)] = static_cast<double>(terrain[base + j]) +
                                                  shift_from + (shift_to - shift_from) * t;
        }
    }

    double residual = relax(budget, true, m_stats.iterations);

    // ------------------------------------------------------------------------
    // Step 4. Bridges and tunnels.
    //
    // The deck is a straight line between its two abutment node heights, never a
    // free-floating ribbon: P1 deliberately refuses to split a node that every
    // road merely ends on, precisely so a deck stays attached to its approaches.
    // Clearance is therefore bought by lifting the ABUTMENTS, not by detaching
    // the deck from them, and the approaches are re-relaxed afterwards so they
    // climb to meet it.
    //
    // A tunnel's PORTAL nodes have the same requirement and, with
    // ElevationConfig::tunnel_portal_at_surface, they get the same treatment from
    // the other side: the node is left exactly where the relaxation put it -- the
    // approach surface -- and it is the EDGE that moves, diving away from the
    // portal at its own class grade limit. Only a node interior to a longer bore,
    // where nothing but tunnel edges meet, is still dropped and pinned here. See
    // build_tunnel_ramp().
    // ------------------------------------------------------------------------

    size_t override_edges = 0;
    for (size_t i = 0; i < edge_count; ++i) {
        const EdgeSlot& slot = slots[i];
        if (!slot.active || !(slot.is_bridge || slot.is_tunnel)) {
            continue;
        }

        // A ramping tunnel end imposes NOTHING on its node. The portal is the
        // approach surface, so the node keeps the height the relaxation gave it
        // and stays tied to its approach arms; the descent to depth happens
        // along the edge instead. Pinning it would be worse than pointless --
        // the approaches would then be solved against a node no constraint of
        // theirs could move.
        const bool pin_from = slot.is_bridge || !slot.ramp_from;
        const bool pin_to = slot.is_bridge || !slot.ramp_to;
        if (!pin_from && !pin_to) {
            continue;
        }

        ++override_edges;
        if (pin_from) {
            pinned[slot.from] = 1u;
        }
        if (pin_to) {
            pinned[slot.to] = 1u;
        }
    }

    if (override_edges > 0) {
        std::vector<double> lift(node_count, 0.0);
        std::vector<double> drop(node_count, 0.0);

        for (int pass = 0; pass < kOverridePasses; ++pass) {
            std::fill(lift.begin(), lift.end(), 0.0);
            std::fill(drop.begin(), drop.end(), 0.0);

            for (size_t i = 0; i < edge_count; ++i) {
                const EdgeSlot& slot = slots[i];
                if (!slot.active || !(slot.is_bridge || slot.is_tunnel)) {
                    continue;
                }
                if (slot.ramp_from && slot.ramp_to) {
                    // Both ends are portals, so this edge has no node to drop.
                    // Its depth is reached along the ramp instead; see
                    // build_tunnel_ramp().
                    continue;
                }

                const uint32_t base = slot.station_base;
                const double span = slot.length;
                const double h_from = vars[slot.from];
                const double h_to = vars[slot.to];

                // Which stations this edge's demand is measured over.
                //
                // A ramping end is a PORTAL, deliberately held at the approach
                // surface, so its stations can NEVER satisfy a depth demand
                // however far the other end is pushed. Measuring them anyway
                // makes `needed` about tunnel_depth on every pass regardless of
                // what the loop already did: the exemption below only stops the
                // drop being APPLIED at that end, so the whole amount is charged
                // to the interior node again and again, `moved` never falls to
                // the convergence epsilon, and after kOverridePasses the node
                // sits kOverridePasses x tunnel_depth below ground -- dragging
                // the surface approaches down the grade chain with it, hundreds
                // of metres away from any tunnel.
                //
                // So a half-ramped edge is measured at the PINNED end's own
                // station and nowhere else. That node owes tunnel_depth under
                // its own ground and nothing more; the descent from the portal
                // is the ramp's shape, which build_tunnel_ramp() solves with its
                // own drop and its own grade cone. A fully interior bore, where
                // neither end ramps, still measures every station: both its ends
                // are free to move and the deepest one is what they owe.
                uint32_t first_station = 0;
                uint32_t station_end = slot.station_count;
                if (slot.ramp_from) {
                    first_station = slot.station_count - 1u;   // the pinned `to` end
                } else if (slot.ramp_to) {
                    station_end = 1u;                          // the pinned `from` end
                }

                double needed = 0.0;
                for (uint32_t j = first_station; j < station_end; ++j) {
                    const double t = span > kMinSpan ? arcs[base + j] / span : 0.0;
                    const double deck = h_from + (h_to - h_from) * t;
                    const double ground = static_cast<double>(terrain[base + j]);

                    if (slot.is_bridge) {
                        needed = std::max(
                            needed, ground + static_cast<double>(cfg.bridge_clearance) - deck);
                    } else {
                        needed = std::max(
                            needed, deck - (ground - static_cast<double>(cfg.tunnel_depth)));
                    }
                }
                if (needed <= 0.0) {
                    continue;
                }

                // A uniform shift of both abutments. Shifting only the station
                // that fails would bend the deck into the terrain profile, which
                // is the one thing a deck must not do.
                if (slot.is_bridge) {
                    lift[slot.from] = std::max(lift[slot.from], needed);
                    lift[slot.to] = std::max(lift[slot.to], needed);
                } else {
                    // A portal end is exempt: it is meant to be at the surface,
                    // and the whole demand falls on the interior end instead.
                    if (!slot.ramp_from) {
                        drop[slot.from] = std::max(drop[slot.from], needed);
                    }
                    if (!slot.ramp_to) {
                        drop[slot.to] = std::max(drop[slot.to], needed);
                    }
                }
            }

            double moved = 0.0;
            for (size_t n = 0; n < node_count; ++n) {
                // A node that is both a bridge abutment and a tunnel portal is a
                // tagging artefact. Clearance wins: burying a deck is a visible
                // failure, a shallow tunnel mouth is not.
                if (lift[n] > 0.0) {
                    vars[n] += lift[n];
                    moved = std::max(moved, lift[n]);
                } else if (drop[n] > 0.0) {
                    vars[n] -= drop[n];
                    moved = std::max(moved, drop[n]);
                }
            }
            if (moved <= static_cast<double>(cfg.convergence_epsilon)) {
                break;
            }
        }

        // The approaches have to follow. Abutments are pinned, so this moves the
        // roads that meet them and never the decks.
        residual = relax(budget, true, m_stats.iterations);
    }

    // ------------------------------------------------------------------------
    // Step 5 and 6. Publish, adding the surface offset.
    // ------------------------------------------------------------------------

    for (size_t n = 0; n < node_count; ++n) {
        // Deliberately WITHOUT surface_offset; see the header. A consumer placing
        // junction geometry adds the same offset the station heights carry, so
        // the junction surface and the arm ends land on one plane.
        m_node_heights[n] = static_cast<float>(vars[n]);
    }

    // Scratch for build_tunnel_ramp(), reused across edges so a network full of
    // tunnels does not allocate per edge. Serial loop, so one buffer is enough.
    std::vector<double> ramp;

    for (size_t i = 0; i < edge_count; ++i) {
        const EdgeSlot& slot = slots[i];
        EdgeElevation& out = m_edges[i];
        if (!slot.active) {
            continue;
        }

        out.is_bridge = slot.is_bridge;
        out.is_tunnel = slot.is_tunnel;
        out.station_heights.resize(slot.station_count);

        // Rounded to float FIRST and offset SECOND, so an end station is
        // bit-identical to node_height() plus the offset. Offsetting in double
        // and rounding once loses that by up to half a float ulp, which is a
        // couple of microns at city scale -- invisible on screen, but it makes
        // "every arm terminates at exactly its node height" false, and that
        // property is the whole point of the aliasing.
        const auto publish = [&](uint32_t j, double height) {
            out.station_heights[j] = static_cast<float>(height) + cfg.surface_offset;
        };

        if (slot.is_tunnel && slot.ramped()) {
            // The roadway starts at the portal, dives at its own class grade
            // limit, runs deep, and climbs back. Everything about the shape is
            // in build_tunnel_ramp(); the two ends still come straight out of
            // the node heights, so the arm terminates exactly where its
            // approach does.
            build_tunnel_ramp(&arcs[slot.station_base], &terrain[slot.station_base],
                              slot.station_count, vars[slot.from], vars[slot.to], slot.grade,
                              static_cast<double>(cfg.tunnel_depth), slot.ramp_from, slot.ramp_to,
                              static_cast<double>(cfg.max_grade_change_per_m), budget,
                              static_cast<double>(cfg.convergence_epsilon), ramp);
            for (uint32_t j = 0; j < slot.station_count; ++j) {
                publish(j, ramp[j]);
            }
        } else if (slot.is_bridge || slot.is_tunnel) {
            // A deck is a straight line between its abutments, so it is
            // reconstructed from the node heights rather than read out of the
            // relaxation: its interior variables were never swept. The two ends
            // are taken from the nodes directly rather than from the lerp, which
            // is only exact at t = 0.
            const uint32_t base = slot.station_base;
            const uint32_t last = slot.station_count - 1u;
            const double span = slot.length;
            const double h_from = vars[slot.from];
            const double h_to = vars[slot.to];
            for (uint32_t j = 0; j < slot.station_count; ++j) {
                if (j == 0u) {
                    publish(j, h_from);
                } else if (j == last) {
                    publish(j, h_to);
                } else {
                    const double t = span > kMinSpan ? arcs[base + j] / span : 0.0;
                    publish(j, h_from + (h_to - h_from) * t);
                }
            }
        } else {
            for (uint32_t j = 0; j < slot.station_count; ++j) {
                publish(j, vars[var_of(slot, j)]);
            }
        }

        // Steepest solved gradient, and whether the limit had to bind at all.
        // The limit bound exactly when the RAW terrain profile was too steep
        // somewhere, which is the honest reading of "the road departs from the
        // terrain": it is measured against the samples, before any correction.
        bool bound = false;
        for (uint32_t j = 0; j + 1u < slot.station_count; ++j) {
            const double ds = arcs[slot.station_base + j + 1] - arcs[slot.station_base + j];
            if (ds < kMinSpan) {
                continue;
            }
            const double solved = std::fabs(static_cast<double>(out.station_heights[j + 1]) -
                                            static_cast<double>(out.station_heights[j])) /
                                  ds;
            out.max_grade_used = std::max(out.max_grade_used, static_cast<float>(solved));

            const double raw = std::fabs(static_cast<double>(terrain[slot.station_base + j + 1]) -
                                         static_cast<double>(terrain[slot.station_base + j])) /
                               ds;
            // Only a relaxed edge can be grade-limited. A deck's gradient is
            // decided by its abutments, so the roughness under it says nothing.
            bound = bound || (slot.relaxed() && raw > slot.grade);
        }

        ++m_stats.edges;
        if (slot.is_bridge) {
            ++m_stats.bridges;
        }
        if (slot.is_tunnel) {
            ++m_stats.tunnels;
        }
        if (bound) {
            ++m_stats.grade_limited_edges;
        }
    }

    m_stats.nodes = node_count;
    m_stats.max_residual = static_cast<float>(residual);
    m_solved = true;

    const auto finished = std::chrono::steady_clock::now();
    m_stats.solve_ms = std::chrono::duration<double, std::milli>(finished - started).count();

    spdlog::info("RoadElevationSolver: Solved {} nodes and {} edges — {} sweeps, "
                 "residual {:.4f} m, {} grade-limited, {} bridges, {} tunnels, in {:.1f} ms",
                 m_stats.nodes, m_stats.edges, m_stats.iterations, m_stats.max_residual,
                 m_stats.grade_limited_edges, m_stats.bridges, m_stats.tunnels, m_stats.solve_ms);

    if (m_stats.max_residual > cfg.convergence_epsilon) {
        // Not a failure: the profile is closer to feasible than the one it
        // started from and every constraint is nearly met. It does mean the
        // iteration budget, not the tolerance, decided when to stop.
        spdlog::warn("RoadElevationSolver: Stopped at the {} iteration cap with a {:.4f} m "
                     "residual, above the {:.4f} m tolerance",
                     cfg.max_iterations, m_stats.max_residual, cfg.convergence_epsilon);
    }
}

} // namespace stratum::osm::road
