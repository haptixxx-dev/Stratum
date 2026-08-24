/**
 * @file junction_trim.cpp
 * @brief Implementation of the P4 arm trim solve
 *
 * The whole file is one idea repeated: a junction is the region where two or
 * more carriageways would overlap if every ribbon were extruded to its full
 * length, so the cure is to find, per arm, the arclength at which that arm's
 * carriageway stops touching its neighbours' and cut there.
 *
 * ### The pair intersection, written out
 *
 * Arms are in ascending bearing order, which is counter-clockwise, so for an
 * adjacent pair (a, b) arm b lies to the LEFT of arm a. The two edges that face
 * each other are therefore a's LEFT carriageway edge and b's RIGHT one. With P
 * the node position, `da`/`db` the unit directions LEAVING the node and
 * `na = (-da.y, da.x)` the left normal of `da`:
 *
 *     La(t) = P + na * wa + da * t
 *     Lb(s) = P - nb * wb + db * s
 *
 * Setting them equal and writing `R = -(na * wa + nb * wb)` gives
 * `da * t - db * s = R`, and crossing that with db and with da in turn isolates
 * each parameter:
 *
 *     t = cross(R, db) / cross(da, db)
 *     s = -cross(da, R) / cross(da, db)
 *
 * `t` is the pair's demand on a and `s` its demand on b, each already measured
 * along its own arm's direction, which is exactly the projection the trim wants.
 * A worked check: two 2w-wide arms at 90 degrees give t = s = w, the corner of
 * the square both carriageways share.
 *
 * A negative parameter means the pair's intersection lies BEHIND the node: the
 * two carriageways were already diverging and this pair asks for nothing. That
 * is the normal state of the reflex pair at a T-junction, and it contributes
 * zero rather than pulling the arm forward into the junction.
 *
 * ### Straight-line distance is not arclength
 *
 * `da` is the direction leaving the node and is only the arm's direction NEAR
 * the node. On a curving approach the centerline falls away from that ray, so a
 * demand of t metres along `da` is reached only after MORE than t metres of
 * arclength. projection_to_arclength() walks the real stations and finds the
 * arclength whose station projects to t, which is the value the ribbon is
 * actually cut at. Using t directly cuts a curving arm short and leaves a wedge
 * of ribbon inside the junction polygon.
 *
 * ### Coordinates
 *
 * 2D local metres throughout, the same frame as GraphEdge::polyline and
 * Centerline::stations. Nothing here maps to render space.
 */

#include "osm/road/junction_trim.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Constants
// ============================================================================

/// Squared length below which a direction vector carries no usable heading
constexpr double kDirEpsilonSq = 1e-24;

/// Arclengths closer together than this are the same station (a bevel pair)
constexpr double kArcEpsilon = 1e-9;

/// A demand reduced by more than this counts as clamped, not as rounding
constexpr double kDemandEpsilon = 1e-9;

/**
 * @brief Metres of every edge that must survive both of its trims
 *
 * TrimConfig::max_trim_fraction is applied per END and cannot see the far end,
 * so at its default 0.4 the two ends together leave 20% of the edge and the
 * question never arises. A caller raising it past 0.5, or setting a min_trim
 * longer than half a short edge, would otherwise consume the whole edge from
 * both directions at once and hand the extruder a negative-length ribbon. The
 * hard cap in apply_clamps() bounds each end at half the edge less half of this,
 * so the two ends together always leave at least this much.
 */
constexpr double kMinRibbonLength = 0.05;

/**
 * @brief Ceiling on tan(theta / 2) when reserving a pair's fillet tangent run
 *
 * The tangent run a fillet needs is `R * tan(theta / 2)`, which is well behaved
 * at an ordinary junction -- exactly R at a square corner -- and diverges as the
 * two arms close towards each other, because rounding the sliver between two
 * nearly-parallel roads at a full kerb radius really does need a very long run.
 * Reserving that literally would cut a slip road's whole approach back for a
 * corner that build_junction_polygon() will reduce to a chord anyway.
 *
 * 2.0 caps the reserve at twice the radius, which is the exact requirement for
 * every corner whose arms are at least 53 degrees apart -- that is, for every
 * junction that reads as a junction rather than as a fork. Sharper corners
 * reserve the capped run, the fit reduction in append_corner() shrinks their
 * radius to what the run actually allows, and the result is what this file
 * produced before the reserve existed: no worse, and bounded.
 */
constexpr double kMaxReserveTanHalf = 2.0;

/**
 * @brief Largest near-coincident cluster collect_arms() will merge into one junction
 *
 * Two graph nodes a few centimetres apart are a data defect. Nine of them in one
 * connected knot is a defect of a kind this file has no model for -- a collapsed
 * roundabout, a whole junction imported twice -- and merging it would produce a
 * twenty-arm junction whose bearing order means nothing. coincident_cluster()
 * gives up on such a component entirely, and every node in it is then solved
 * exactly as it was before merging existed.
 */
constexpr size_t kMaxClusterNodes = 8;

/**
 * @brief Shortest lever arm used to order a merged cluster's arms, metres
 *
 * The ordering key is the direction from the cluster centre to a point one lever
 * out along each arm, so the lever sets how much an arm's STARTING POINT counts
 * against its direction. It scales with the cluster's own spread, and this floor
 * keeps it meaningful for a cluster whose members are millimetres apart, where
 * the spread carries no information and the bearing is the whole answer.
 */
constexpr double kMinClusterLever = 4.0;

/// A quarter turn in radians; the ceiling TrimConfig::min_pair_angle is clamped to
constexpr double kHalfPi = 1.57079632679489661923;

// ============================================================================
// Small geometry helpers
// ============================================================================

/// 2D cross product; equals |a||b| sin(angle from a to b)
[[nodiscard]] inline double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

/// Left normal of a direction: a quarter turn counter-clockwise
[[nodiscard]] inline glm::dvec2 left_normal(const glm::dvec2& d) {
    return glm::dvec2(-d.y, d.x);
}

/**
 * @brief Straight run the corner fillet between two arms is tangent over, metres
 *
 * Mirrors append_corner() in junction_polygon.cpp exactly, and deliberately: the
 * two must agree or the fillet either overshoots the cut face or is reduced away
 * again by the fit step there. Both derive the SAME nominal radius from the two
 * carriageway widths, both measure the SAME turn angle, and both take the tangent
 * distance as `R * tan(theta / 2)`.
 *
 * The ring's heading over the corner is `-da` on the way in and `+db` on the way
 * out, so the turn angle is the angle between those two and NOT the angle
 * between the arms. A corner the ring turns LEFT over is the wrap-around pair at
 * a node whose arms all leave within a half plane; append_corner() closes it with
 * a chord, so nothing is reserved for it.
 *
 * @param da  Unit direction leaving the node along the earlier arm
 * @param db  Unit direction leaving the node along the next arm
 * @param wa  Earlier arm's carriageway half width, metres
 * @param wb  Next arm's carriageway half width, metres
 * @param cfg Tolerances; the four fillet_* fields mirror FilletConfig
 * @return Metres of straight run to reserve on BOTH arms, 0 when the corner will
 *         be drawn as a chord
 */
[[nodiscard]] double fillet_tangent_run(const glm::dvec2& da,
                                        const glm::dvec2& db,
                                        double wa,
                                        double wb,
                                        const TrimConfig& cfg) {
    const double factor = cfg.fillet_radius_width_factor;
    if (!(std::isfinite(factor) && factor > 0.0)) {
        return 0.0;     // reservation disabled
    }

    const glm::dvec2 u_in = -da;
    const glm::dvec2 u_out = db;

    const double turn = cross2(u_in, u_out);
    if (turn >= 0.0) {
        return 0.0;     // left turn or dead straight: append_corner() draws a chord
    }

    const double theta = std::atan2(std::fabs(turn), glm::dot(u_in, u_out));
    if (!std::isfinite(theta) || theta < cfg.fillet_min_arc_angle) {
        return 0.0;     // shallower than a chord is worth rounding
    }

    const double lower = std::max(0.0, cfg.fillet_min_radius);
    const double upper = std::max(lower, cfg.fillet_max_radius);
    const double radius = std::clamp(factor * std::min(2.0 * wa, 2.0 * wb), lower, upper);

    const double tan_half = std::min(std::tan(theta * 0.5), kMaxReserveTanHalf);
    if (!(std::isfinite(tan_half) && tan_half > 0.0)) {
        return 0.0;
    }
    return radius * tan_half;
}

/**
 * @brief Lateral half-extent of the carriageway envelope, metres
 *
 * RoadProfile::left_edge_offset() centres the INCLUSIVE span from the first
 * Lane-or-Median strip to the last one on lateral zero, so that span -- and not
 * RoadProfile::carriageway_width(), which sums Lane strips alone and would miss
 * a median and the gutters and curbs inside a dual carriageway -- is what
 * occupies [-half, +half]. The span is recomputed here exactly as
 * left_edge_offset() computes it, so the two cannot drift apart.
 *
 * A profile with no Lane and no Median strip -- a bare footway -- is centred
 * whole by left_edge_offset(), so its envelope is its whole width.
 *
 * @param p Profile to measure
 * @return Half the carriageway envelope; 0 for an empty profile
 */
[[nodiscard]] double carriageway_half_extent(const RoadProfile& p) {
    if (p.strips.empty()) return 0.0;

    const size_t none = p.strips.size();
    size_t first = none;
    size_t last = 0;
    for (size_t i = 0; i < p.strips.size(); ++i) {
        const StripKind k = p.strips[i].kind;
        if (k == StripKind::Lane || k == StripKind::Median) {
            if (first == none) first = i;
            last = i;
        }
    }

    if (first == none) return static_cast<double>(p.total_width()) * 0.5;

    double span = 0.0;
    for (size_t i = first; i <= last; ++i) {
        span += static_cast<double>(p.strips[i].width);
    }
    return span * 0.5;
}

/**
 * @brief Usable arclength span of an arm's centerline, metres
 *
 * Taken as back minus front rather than Centerline::length(), because a slice
 * does not rebase its arclengths and its length() is then an end position rather
 * than a span.
 */
[[nodiscard]] double centerline_span(const Centerline& cl) {
    if (!cl.is_valid()) return 0.0;
    const double s = cl.stations.back().arclength - cl.stations.front().arclength;
    return (std::isfinite(s) && s > 0.0) ? s : 0.0;
}

/**
 * @brief Unit direction LEAVING the node along an arm
 *
 * Read from the centerline's terminal station tangent, negated for an arm at the
 * `to` end, because that station's tangent points in the direction of TRAVEL and
 * travel arrives at the node from that end rather than leaving by it.
 *
 * The centerline tangent is preferred over Arm::bearing because the resampled,
 * smoothed centerline is the geometry the ribbon is actually built from: arm_end()
 * places the cut corners with that station's own frame, and a trim solved against
 * the raw survey chord instead would not land where those corners do. The graph
 * direction and then the bearing are used only when there is no centerline to
 * read.
 *
 * @param graph       Built road graph
 * @param centerlines Parallel to graph.edges()
 * @param arm         Arm to take the direction of
 * @return Unit direction leaving the node
 */
[[nodiscard]] glm::dvec2 leaving_direction(const RoadGraph& graph,
                                           const std::vector<Centerline>& centerlines,
                                           const ArmRef& arm) {
    if (arm.edge != kInvalidId && arm.edge < centerlines.size()) {
        const Centerline& cl = centerlines[arm.edge];
        if (cl.is_valid()) {
            const Station& s = arm.at_start ? cl.stations.front() : cl.stations.back();
            const glm::dvec2 d = arm.at_start ? s.tangent : -s.tangent;
            if (glm::dot(d, d) > kDirEpsilonSq) return glm::normalize(d);
        }
    }

    if (arm.edge != kInvalidId && arm.edge < graph.edges().size()) {
        Arm ga;
        ga.edge = arm.edge;
        ga.at_start = arm.at_start;
        ga.bearing = arm.bearing;
        const glm::dvec2 d = graph.arm_direction(ga);
        if (glm::dot(d, d) > kDirEpsilonSq) return glm::normalize(d);
    }

    return glm::dvec2(std::cos(arm.bearing), std::sin(arm.bearing));
}

/**
 * @brief Convert a distance measured along the node ray into an arclength
 *
 * The pair intersection is solved against `dir`, the arm's LOCAL direction at
 * the node, so its answer is a distance along a straight ray. The ribbon is cut
 * at an ARCLENGTH. On a straight arm the two are the same number; on a curving
 * one the centerline peels away from the ray, its projection onto the ray grows
 * more slowly than its arclength, and the arclength that satisfies the demand is
 * strictly larger.
 *
 * The walk starts at the node end of the centerline and advances outward,
 * tracking the projection `dot(station - origin, dir)`, and interpolates
 * linearly inside the band that first reaches @p distance.
 *
 * Two cases stop the walk short:
 *
 * - Stations sharing an arclength are a bevel pair. They are stepped over rather
 *   than treated as a stalled projection.
 * - An arm turning more than a quarter turn away from its own node direction has
 *   a projection that stops increasing. Past that point the ray formulation says
 *   nothing, so the walk saturates at the far end of the centerline and lets the
 *   max_trim_fraction clamp decide. That is the correct failure direction: the
 *   arm really does lie alongside its neighbour for a long way.
 *
 * @param cl       Arm's centerline
 * @param at_start True when the node is the centerline's front
 * @param origin   Node position in 2D local metres
 * @param dir      Unit direction leaving the node
 * @param distance Demand along @p dir, metres; non-positive yields 0
 * @return Arclength from the node end, metres, never negative
 */
[[nodiscard]] double projection_to_arclength(const Centerline& cl,
                                             bool at_start,
                                             const glm::dvec2& origin,
                                             const glm::dvec2& dir,
                                             double distance) {
    if (!std::isfinite(distance) || distance <= 0.0) return 0.0;
    if (!cl.is_valid()) return distance;

    const std::vector<Station>& st = cl.stations;
    const size_t count = st.size();
    const double front_arc = st.front().arclength;
    const double back_arc = st.back().arclength;

    // Index and arclength-from-the-node of the i-th step outward from the node.
    const auto step_index = [&](size_t i) { return at_start ? i : (count - 1u - i); };
    const auto step_arc = [&](size_t i) {
        const Station& s = st[step_index(i)];
        return at_start ? (s.arclength - front_arc) : (back_arc - s.arclength);
    };

    double prev_proj = glm::dot(st[step_index(0)].position - origin, dir);
    double prev_arc = 0.0;
    if (prev_proj >= distance) return 0.0;

    for (size_t i = 1; i < count; ++i) {
        const double arc = step_arc(i);
        if (!(arc > prev_arc + kArcEpsilon)) continue;   // bevel pair: same arclength

        const double proj = glm::dot(st[step_index(i)].position - origin, dir);

        if (proj >= distance) {
            const double denom = proj - prev_proj;
            const double f = (denom > 0.0) ? ((distance - prev_proj) / denom) : 0.0;
            const double hit = prev_arc + std::clamp(f, 0.0, 1.0) * (arc - prev_arc);
            return std::max(0.0, hit);
        }

        // The projection stopped advancing: the arm has turned past a quarter
        // turn from its node direction and the ray no longer describes it.
        if (!(proj > prev_proj)) break;

        prev_proj = proj;
        prev_arc = arc;
    }

    // Demand never reached along the arm. Saturate at the whole span; the caller's
    // clamps bound it to a fraction of the edge.
    return std::max(0.0, centerline_span(cl));
}

/**
 * @brief Junction nodes tied to @p node by stub edges shorter than @p radius
 *
 * The near-coincident cluster described in collect_arms(). Two graph nodes a few
 * centimetres apart are one junction that RoadGraph could not merge, because its
 * own duplicate test is exact to 1e-6 m; this finds them so the junction can be
 * solved once instead of twice.
 *
 * The walk is a breadth-first flood over edges whose LENGTH is below @p radius
 * and whose far node is itself of degree 3 or more. Length rather than endpoint
 * distance, so a genuine loop road returning to a nearby node is not swallowed:
 * that edge is a real approach and must stay an arm.
 *
 * The result is the connected component, so it does not depend on which member
 * the walk started from -- which is what lets every member agree on who the
 * primary is without any shared state. A component larger than kMaxClusterNodes
 * is not bad data but something this function does not understand, and it gives
 * up on the whole component rather than truncating it, because a truncated walk
 * WOULD depend on where it started.
 *
 * @param graph  Built road graph
 * @param node   Node to cluster around; must be a valid junction node
 * @param radius Metres below which a stub edge means "the same junction"
 * @return The cluster in ascending GraphNodeId order, always containing @p node.
 *         Just @p node when nothing is near it or the component is too large.
 */
[[nodiscard]] std::vector<GraphNodeId> coincident_cluster(const RoadGraph& graph,
                                                          GraphNodeId node,
                                                          double radius) {
    std::vector<GraphNodeId> cluster{ node };
    if (!(radius > 0.0) || !std::isfinite(radius)) return cluster;

    // Breadth-first over the stub edges. `cluster` doubles as the visited set and
    // as the queue; it is kept sorted so membership is a binary search and the
    // output order is canonical.
    for (size_t head = 0; head < cluster.size(); ++head) {
        const GraphNode& n = graph.node(cluster[head]);
        for (const Arm& a : n.arms) {
            if (a.edge == kInvalidId || a.edge >= graph.edges().size()) continue;

            const GraphEdge& e = graph.edge(a.edge);
            const GraphNodeId far = a.at_start ? e.to : e.from;
            if (far == kInvalidId || far >= graph.nodes().size()) continue;
            if (far == cluster[head]) continue;              // a closed loop
            if (graph.node(far).arms.size() < 3) continue;   // not a junction
            if (!(e.length() < radius)) continue;            // a real approach

            const auto at = std::lower_bound(cluster.begin(), cluster.end(), far);
            if (at != cluster.end() && *at == far) continue; // already in
            if (cluster.size() >= kMaxClusterNodes) {
                return { node };                             // too tangled to merge
            }
            cluster.insert(at, far);
        }
    }

    return cluster;
}

/// True when @p edge holds a cluster together: short, and both ends inside it
[[nodiscard]] bool is_internal_stub(const RoadGraph& graph,
                                    const std::vector<GraphNodeId>& cluster,
                                    EdgeId edge,
                                    double radius) {
    if (cluster.size() < 2 || edge == kInvalidId || edge >= graph.edges().size()) return false;

    const GraphEdge& e = graph.edge(edge);
    if (!(e.length() < radius)) return false;

    const auto holds = [&cluster](GraphNodeId id) {
        const auto at = std::lower_bound(cluster.begin(), cluster.end(), id);
        return at != cluster.end() && *at == id;
    };
    return holds(e.from) && holds(e.to);
}

/// Fill an ArmRef's widths from the arm's own profile; the outputs are left alone
void fill_widths(const std::vector<RoadProfile>& profiles, ArmRef& ref) {
    // An arm with no usable profile still takes its place in the cycle, at zero
    // width. Dropping it would make two arms that are not neighbours adjacent,
    // and every pair demand after it would be solved against the wrong partner.
    if (ref.edge != kInvalidId && ref.edge < profiles.size()) {
        const RoadProfile& p = profiles[ref.edge];
        if (p.is_valid()) {
            ref.half_width = static_cast<double>(p.total_width()) * 0.5;
            ref.carriageway_half = carriageway_half_extent(p);
        }
    }
}

} // namespace

// ============================================================================
// Collection
// ============================================================================

std::vector<ArmRef> collect_arms(const RoadGraph& graph,
                                 const std::vector<RoadProfile>& profiles,
                                 GraphNodeId node,
                                 double coincident_radius,
                                 std::vector<GraphNodeId>* out_cluster) {
    std::vector<ArmRef> out;
    if (out_cluster != nullptr) out_cluster->clear();
    if (node == kInvalidId || node >= graph.nodes().size()) return out;

    const GraphNode& n = graph.node(node);

    // A node that merges with nothing still reports the cluster {node}, so a
    // caller resolving a member to its primary never has to tell "no cluster"
    // apart from "not a junction".
    if (out_cluster != nullptr) out_cluster->assign(1, node);

    // ------------------------------------------------------------------------
    // Near-coincident junctions are ONE junction. Only a node that is already a
    // junction can absorb another, and a cluster of one -- the overwhelmingly
    // common case -- falls straight through to the plain collection below with
    // its arms in the order the graph sorted them.
    // ------------------------------------------------------------------------
    if (n.arms.size() >= 3 && coincident_radius > 0.0) {
        const std::vector<GraphNodeId> cluster = coincident_cluster(graph, node, coincident_radius);
        if (out_cluster != nullptr) *out_cluster = cluster;
        if (cluster.size() > 1) {
            if (cluster.front() != node) {
                // Not the primary. Emitting nothing is the whole point: the
                // primary has already taken these arms, and two junctions on one
                // patch of ground is the artefact being removed.
                spdlog::debug("collect_arms: node {} is {} m from node {} and is solved as part "
                              "of it",
                              node, glm::length(graph.node(cluster.front()).position - n.position),
                              cluster.front());
                return out;
            }

            for (GraphNodeId member : cluster) {
                for (const Arm& a : graph.node(member).arms) {
                    if (is_internal_stub(graph, cluster, a.edge, coincident_radius)) {
                        continue;   // the stub that made them one junction
                    }
                    ArmRef ref;
                    ref.edge = a.edge;
                    ref.at_start = a.at_start;
                    ref.bearing = a.bearing;
                    fill_widths(profiles, ref);
                    out.push_back(ref);
                }
            }

            // ----------------------------------------------------------------
            // Re-order the merged list around the cluster.
            //
            // The graph sorted each node's arms about ITS OWN node, so a merged
            // list is interleaved and has to be sorted again -- every later step,
            // the pairwise cycle, the ring winding, arm_ring_start, depends on
            // one ascending cyclic order.
            //
            // Sorting on Arm::bearing alone is not that order. Two arms leaving
            // different members of the cluster on the same bearing -- the two
            // halves of a dual carriageway are exactly that -- come out adjacent
            // and in an arbitrary order, and their neighbours in the cycle are
            // then not their neighbours on the ground. The pair solve trims each
            // against the wrong partner and the ring crosses itself.
            //
            // So each arm is ordered by the direction from the cluster's CENTRE
            // to a point one lever arm out along it. The lever is the cluster's
            // own size, so an arm leaving a member on the far side of the cluster
            // is ranked by where it goes AND by where it starts, and for a
            // cluster of nearly coincident nodes -- where the members are
            // centimetres apart -- it reduces to the bearing it always was.
            // ----------------------------------------------------------------
            glm::dvec2 centre(0.0);
            for (GraphNodeId member : cluster) centre += graph.node(member).position;
            centre /= static_cast<double>(cluster.size());

            double spread = 0.0;
            for (GraphNodeId member : cluster) {
                spread = std::max(spread, glm::length(graph.node(member).position - centre));
            }
            const double lever = std::max(2.0 * spread, kMinClusterLever);

            std::vector<std::pair<double, ArmRef>> keyed;
            keyed.reserve(out.size());
            for (const ArmRef& ref : out) {
                glm::dvec2 from = centre;
                if (ref.edge != kInvalidId && ref.edge < graph.edges().size()) {
                    const GraphEdge& e = graph.edge(ref.edge);
                    const GraphNodeId own = ref.at_start ? e.from : e.to;
                    if (own != kInvalidId && own < graph.nodes().size()) {
                        from = graph.node(own).position;
                    }
                }

                Arm ga;
                ga.edge = ref.edge;
                ga.at_start = ref.at_start;
                ga.bearing = ref.bearing;
                glm::dvec2 d = graph.arm_direction(ga);
                if (glm::dot(d, d) > kDirEpsilonSq) {
                    d = glm::normalize(d);
                } else {
                    d = glm::dvec2(std::cos(ref.bearing), std::sin(ref.bearing));
                }

                const glm::dvec2 out_point = (from + d * lever) - centre;
                const double key = (glm::dot(out_point, out_point) > kDirEpsilonSq)
                                       ? std::atan2(out_point.y, out_point.x)
                                       : ref.bearing;
                keyed.emplace_back(key, ref);
            }

            // The edge and end tie-break keeps two arms that really do rank
            // identically in a fixed order run to run.
            std::sort(keyed.begin(), keyed.end(), [](const auto& a, const auto& b) {
                if (a.first != b.first) return a.first < b.first;
                if (a.second.edge != b.second.edge) return a.second.edge < b.second.edge;
                return static_cast<int>(a.second.at_start) > static_cast<int>(b.second.at_start);
            });

            for (size_t i = 0; i < keyed.size(); ++i) out[i] = keyed[i].second;
            return out;
        }
    }

    out.reserve(n.arms.size());
    for (const Arm& a : n.arms) {
        ArmRef ref;
        ref.edge = a.edge;
        ref.at_start = a.at_start;
        ref.bearing = a.bearing;
        fill_widths(profiles, ref);
        out.push_back(ref);
    }

    return out;
}

// ============================================================================
// Solve
// ============================================================================

bool solve_arm_trims(const RoadGraph& graph,
                     const std::vector<Centerline>& centerlines,
                     GraphNodeId node,
                     std::vector<ArmRef>& arms,
                     const TrimConfig& cfg) {
    const size_t n = arms.size();

    // ------------------------------------------------------------------------
    // Per-arm usable span. Needed by every path, including the degenerate one,
    // because even a floor of min_trim has to be bounded by the edge it is cut
    // from.
    // ------------------------------------------------------------------------
    std::vector<double> span(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        const ArmRef& a = arms[i];
        if (a.edge != kInvalidId && a.edge < centerlines.size()) {
            span[i] = centerline_span(centerlines[a.edge]);
        }
    }

    /**
     * Turn a demand into a solved trim.
     *
     * Three bounds, in strictly increasing authority:
     *
     * 1. `lower` -- TrimConfig::min_trim, the floor every arm gets.
     * 2. `soft`  -- max_trim_fraction of the arm's own edge. min_trim wins over
     *               it, as the header states, so a configured floor is never
     *               silently discarded on a short edge.
     * 3. `hard`  -- half the edge less half of kMinRibbonLength. Nothing wins
     *               over this: it is what guarantees that an edge trimmed from
     *               BOTH ends still has positive length, which no per-end solve
     *               can otherwise see.
     *
     * ArmRef::clamped records that the demand was reduced, which is the
     * over-trim signal the caller counts. A trim RAISED to the min_trim floor is
     * not a clamp -- nothing overlaps because of it.
     */
    const auto apply_clamps = [&cfg](ArmRef& arm, double arm_span, double demand) {
        const double lower = std::max(0.0, cfg.min_trim);
        const double soft = arm_span * std::clamp(cfg.max_trim_fraction, 0.0, 1.0);
        const double hard = std::max(0.0, arm_span * 0.5 - kMinRibbonLength * 0.5);

        double t = (std::isfinite(demand) && demand > 0.0) ? demand : 0.0;
        if (t < lower) t = lower;
        if (t > soft) t = std::max(soft, lower);
        if (t > hard) t = hard;
        if (!std::isfinite(t) || t < 0.0) t = 0.0;

        arm.trim = t;
        arm.clamped = std::isfinite(demand) && (demand > t + kDemandEpsilon);
    };

    const bool node_ok = (node != kInvalidId) && (node < graph.nodes().size());

    // ------------------------------------------------------------------------
    // Degenerate: nothing to pair. Every arm gets the floor, so the graph is
    // left consistent and the caller's fallback path has usable numbers.
    // ------------------------------------------------------------------------
    if (!node_ok || n < 3) {
        for (size_t i = 0; i < n; ++i) apply_clamps(arms[i], span[i], 0.0);
        return false;
    }

    const glm::dvec2 node_origin = graph.node(node).position;

    // Every arm is measured from ITS OWN node, which for an ordinary junction is
    // the node itself and for a near-coincident cluster (see collect_arms) is a
    // point a few centimetres away. Carrying the origins separately is what makes
    // a merged cluster geometrically honest: the pair intersection below is then
    // between two lines that really do start where their arms start, instead of
    // both being slid onto the primary's position first.
    std::vector<glm::dvec2> origin(n, node_origin);
    std::vector<glm::dvec2> dir(n);
    for (size_t i = 0; i < n; ++i) {
        dir[i] = leaving_direction(graph, centerlines, arms[i]);
        if (arms[i].edge != kInvalidId && arms[i].edge < graph.edges().size()) {
            const GraphEdge& e = graph.edge(arms[i].edge);
            const GraphNodeId own = arms[i].at_start ? e.from : e.to;
            if (own != kInvalidId && own < graph.nodes().size()) {
                origin[i] = graph.node(own).position;
            }
        }
    }

    // ------------------------------------------------------------------------
    // Pair demands. Each adjacent pair in the bearing cycle -- including the one
    // wrapping from the last arm back to the first -- contributes to BOTH of its
    // arms, and each arm keeps the LARGER of its two contributions. The maximum
    // is what makes the answer independent of the order the pairs are visited
    // in, and what stops one side of a wide arm still overlapping when the other
    // side is satisfied.
    // ------------------------------------------------------------------------
    std::vector<double> demand(n, 0.0);
    size_t usable_pairs = 0;

    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1u) % n;

        const glm::dvec2& da = dir[i];
        const glm::dvec2& db = dir[j];
        const double wa = std::max(0.0, arms[i].carriageway_half);
        const double wb = std::max(0.0, arms[j].carriageway_half);

        const double denom = cross2(da, db);

        double ta = 0.0;
        double tb = 0.0;

        if (std::fabs(denom) < std::max(0.0, cfg.parallel_epsilon)) {
            // ----------------------------------------------------------------
            // Parallel: the two offset lines meet at infinity, so there is no
            // intersection to project. The two sub-cases behave oppositely and
            // conflating them is what produces either a NaN or a kilometre-long
            // trim.
            //
            // Anti-parallel (a straight road passing through the node, or a
            // hairpin folded back on itself): the near-side edges run AWAY from
            // each other along one line. The carriageways abut and never
            // overlap, so the pair demands nothing.
            //
            // Co-directional (a dual carriageway's two halves, a service road
            // leaving at a hair's-breadth angle): the two carriageways overlap
            // for their entire length and no finite trim separates them. The
            // exact formula would demand (wa + wb) / |sin| and blow up. A
            // width-derived stand-in of wa + wb -- retreat by the combined
            // half-widths, which is the least that could open a gore between
            // them -- is bounded, deterministic, and lets the max_trim_fraction
            // clamp have the final word.
            // ----------------------------------------------------------------
            if (glm::dot(da, db) > 0.0) {
                // Two co-directional arms leaving the SAME point always overlap.
                // Two leaving points far enough apart across their own direction
                // do not overlap at all -- the two halves of a dual carriageway
                // sharing one graph node, or a service road running alongside --
                // and the gap between their near-side edges says which it is.
                const glm::dvec2 gap = (origin[j] - left_normal(db) * wb) -
                                       (origin[i] + left_normal(da) * wa);
                if (glm::dot(gap, left_normal(da)) > 0.0) {
                    // b's right edge is already left of a's left edge: the two
                    // carriageways are disjoint and neither has to retreat.
                    ++usable_pairs;
                    demand[i] = std::max(demand[i], 0.0);
                    demand[j] = std::max(demand[j], 0.0);
                    continue;
                }

                ta = wa + wb;
                tb = ta;

                // A co-directional pair is parallel but NOT inert: it hands both
                // arms a real, bounded, width-derived demand. Counting it as
                // usable is what stops the "every pair parallel" bail-out below
                // from throwing that demand away and declaring a node degenerate
                // that has a perfectly good answer. Only the ANTI-parallel case
                // contributes nothing, and only it should count towards
                // degeneracy.
                ++usable_pairs;
            }
        } else {
            ++usable_pairs;

            const glm::dvec2 na = left_normal(da);
            const glm::dvec2 nb = left_normal(db);

            // La(t) = Pa + na*wa + da*t  meets  Lb(s) = Pb - nb*wb + db*s.
            // Rearranged: da*t - db*s = D, with D the vector from a's left edge
            // origin to b's right edge origin. For two arms sharing one node D
            // collapses to the -(na*wa + nb*wb) of the header's derivation.
            const glm::dvec2 r = (origin[j] - nb * wb) - (origin[i] + na * wa);

            // Both directions are unit, so the determinant IS sin(theta) and
            // flooring its magnitude is flooring the angle. Below
            // TrimConfig::min_pair_angle the pair is solved as though its two arms
            // were exactly that far apart, which bounds a demand that otherwise
            // diverges as 1/theta and produces two-hundred-metre junctions out of
            // a slip road leaving at half a degree. The SIGN is preserved, so
            // which side of the pair each arm is on does not change. See
            // TrimConfig::min_pair_angle.
            double solve_denom = denom;
            const double floor_sin = std::sin(std::clamp(cfg.min_pair_angle, 0.0, kHalfPi));
            if (std::fabs(solve_denom) < floor_sin) {
                solve_denom = std::copysign(floor_sin, solve_denom);
            }

            ta = cross2(r, db) / solve_denom;
            tb = -cross2(da, r) / solve_denom;

            // The corner between these two arms is rounded, and the arc is
            // tangent to each offset line `R * tan(theta / 2)` BACK FROM the
            // intersection. Cutting at the intersection itself leaves the arc
            // nowhere to sit, so both tangent points are reserved here, on the
            // raw parameters and before the sign test below: a corner whose
            // intersection lies far enough behind the node that even its tangent
            // point does, still asks for nothing.
            const double reserve = fillet_tangent_run(da, db, wa, wb, cfg);
            ta += reserve;
            tb += reserve;

            // A negative parameter puts the intersection behind the node: this
            // pair already diverges and asks for nothing rather than for a
            // negative retreat.
            if (!std::isfinite(ta) || ta < 0.0) ta = 0.0;
            if (!std::isfinite(tb) || tb < 0.0) tb = 0.0;
        }

        demand[i] = std::max(demand[i], ta);
        demand[j] = std::max(demand[j], tb);
    }

    // ------------------------------------------------------------------------
    // No pair produced a demand: every adjacent pair is ANTI-parallel, so the
    // arms are collinear and their carriageways abut instead of overlapping.
    // There is no junction to open up. Fall back to the floor rather than
    // cutting holes in a network the caller is about to mark degenerate anyway.
    //
    // A co-directional pair does not reach here -- it counted itself usable
    // above -- because it does have an answer, just not an intersected one.
    // ------------------------------------------------------------------------
    if (usable_pairs == 0) {
        for (size_t i = 0; i < n; ++i) apply_clamps(arms[i], span[i], 0.0);
        spdlog::debug("solve_arm_trims: node {} has {} arms and no pair that demands a trim", node,
                      n);
        return false;
    }

    // ------------------------------------------------------------------------
    // Straight-line demand -> arclength -> clearance -> clamps.
    //
    // The clearance is added AFTER the conversion because it is a distance along
    // the ribbon, not along the node ray: it exists to open the zero-width sliver
    // the exact intersection leaves into a corner the fillet arc can be drawn
    // across.
    // ------------------------------------------------------------------------
    for (size_t i = 0; i < n; ++i) {
        const ArmRef& a = arms[i];

        double arc = demand[i];
        if (a.edge != kInvalidId && a.edge < centerlines.size()) {
            arc = projection_to_arclength(centerlines[a.edge], a.at_start, origin[i], dir[i],
                                          demand[i]);
        }

        apply_clamps(arms[i], span[i], arc + std::max(0.0, cfg.clearance));
    }

    return true;
}

// ============================================================================
// Cut cross-sections
// ============================================================================

ArmEnd arm_end(const RoadGraph& graph,
               const std::vector<Centerline>& centerlines,
               const std::vector<RoadProfile>& profiles,
               const ArmRef& arm) {
    ArmEnd out;

    // Every failure path collapses the cross-section onto the node position, so
    // resolve that first and use it as the default for all five points.
    const GraphEdge* edge = nullptr;
    if (arm.edge != kInvalidId && arm.edge < graph.edges().size()) {
        edge = &graph.edge(arm.edge);
        const GraphNodeId nid = arm.at_start ? edge->from : edge->to;
        if (nid != kInvalidId && nid < graph.nodes().size()) {
            const glm::dvec2 p = graph.node(nid).position;
            out.center = p;
            out.left = p;
            out.right = p;
            out.carriage_left = p;
            out.carriage_right = p;
        }
    }
    out.direction = glm::dvec2(std::cos(arm.bearing), std::sin(arm.bearing));

    if (edge == nullptr) return out;
    if (arm.edge >= centerlines.size() || arm.edge >= profiles.size()) return out;

    const Centerline& cl = centerlines[arm.edge];
    const RoadProfile& profile = profiles[arm.edge];
    if (!cl.is_valid() || !profile.is_valid()) return out;

    const double s0 = cl.stations.front().arclength;
    const double s1 = cl.stations.back().arclength;
    const double span = s1 - s0;
    if (!(std::isfinite(span) && span > 0.0)) return out;

    double trim = arm.trim;
    if (!std::isfinite(trim) || trim < 0.0) trim = 0.0;
    if (trim >= span) return out;   // the trim consumed the whole edge

    // Station in the EDGE's own parameterisation, measured from its `from` node.
    const double cut = arm.at_start ? (s0 + trim) : (s1 - trim);

    // slice() SYNTHESISES the end station at exactly `cut`, interpolating the
    // miter vector rather than the normal and the scale apart, so the corners
    // below land on the untrimmed ribbon's own edge. Snapping to the nearest
    // resampled station instead would leave the junction polygon and the trimmed
    // ribbon a whole band's width out of register. See the slice() contract in
    // centerline.hpp -- this call is what it exists for.
    const Centerline piece = arm.at_start ? slice(cl, cut, s1) : slice(cl, s0, cut);
    if (!piece.is_valid()) return out;

    const Station& st = arm.at_start ? piece.stations.front() : piece.stations.back();

    // ------------------------------------------------------------------------
    // Laterals. Positive lateral is LEFT OF TRAVEL, which is the profile's own
    // convention, and the profile's left edge is left_edge_offset() with its
    // right edge one total_width() further down.
    // ------------------------------------------------------------------------
    const double travel_left = static_cast<double>(profile.left_edge_offset());
    const double travel_right = travel_left - static_cast<double>(profile.total_width());
    const double carriage_half = carriageway_half_extent(profile);

    glm::dvec2 dir = arm.at_start ? st.tangent : -st.tangent;
    if (glm::dot(dir, dir) > kDirEpsilonSq) {
        dir = glm::normalize(dir);
    } else {
        dir = out.direction;
    }

    // ------------------------------------------------------------------------
    // THE FLIP. ArmEnd::left and ::right are relative to the direction LEAVING
    // the node. For an arm at the edge's `to` end the leaving direction is the
    // reverse of travel, so the arm's left is the profile's RIGHT and the arm's
    // right is the profile's LEFT. The inversion happens here and nowhere else;
    // a consumer that flips again mirrors every junction in the network.
    // ------------------------------------------------------------------------
    if (arm.at_start) {
        out.left = offset_point(st, travel_left);
        out.right = offset_point(st, travel_right);
        out.carriage_left = offset_point(st, carriage_half);
        out.carriage_right = offset_point(st, -carriage_half);
    } else {
        out.left = offset_point(st, travel_right);
        out.right = offset_point(st, travel_left);
        out.carriage_left = offset_point(st, -carriage_half);
        out.carriage_right = offset_point(st, carriage_half);
    }

    // offset_point() places every one of those on `position + normal * lateral *
    // miter_scale`, so the centre is the station position itself and all five
    // points are collinear along the station normal, ordered right,
    // carriage_right, carriage_left, left.
    out.center = st.position;
    out.direction = dir;
    out.arclength = cut;
    out.valid = true;

    return out;
}

} // namespace stratum::osm::road
