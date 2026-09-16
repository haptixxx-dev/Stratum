// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file blocks.cpp
 * @brief Implementation of half-edge face traversal over the road graph
 *
 * One pass classifies edges, then one orbit walk per unvisited half-edge. There
 * is no spatial index, no sweep line and no intersection test anywhere in here,
 * and that is the point: the faces come out of the graph's TOPOLOGY. The moment
 * this file starts looking for where two polylines cross, it has started inventing
 * junctions from geometry, which is the mistake the whole road pipeline is built
 * to avoid.
 *
 * The one place geometry is consulted is the rotation at a node, and only to
 * separate two arms that RoadGraph gave the same bearing. That is not inventing a
 * junction: the node is already shared, and the question is only which of two
 * coincident arms sits on which side of the other.
 *
 * See blocks.hpp for the traversal rule, the orientation contract, and the reason
 * the outer face is rejected by sign.
 */

#include "osm/road/blocks.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/**
 * @brief Points closer than this in local metres are the same point
 *
 * A micrometre, the same value RoadGraph uses for its own dedup. It matters that
 * the two agree: the ring is built from polyline vertices RoadGraph already
 * filtered at this tolerance, so a looser value here would prune vertices the
 * graph considers distinct and a tighter one would leave duplicates the graph
 * thought it had removed.
 */
constexpr double kPointEpsilon = 1e-6;
constexpr double kPointEpsilonSq = kPointEpsilon * kPointEpsilon;

/**
 * @brief Below this many square metres a face has no interior at all
 *
 * Distinct from BlockConfig::min_area, which is a judgement about what is worth
 * keeping. This is a judgement about what is REAL: a cycle whose nodes happen to
 * be collinear walks a face whose shoelace sum lands on either side of zero
 * depending on rounding. Without this band such a face is classified by the sign
 * of its rounding error, and half of them get counted as outer faces.
 */
constexpr double kAreaEpsilon = 1e-9;

/**
 * @brief Two arms this close in bearing are a tie the sort could not order
 *
 * RoadGraph sorts arms by `a.bearing < b.bearing` with a non-stable sort, so arms
 * within rounding of each other come out in whatever order introsort left them.
 * That order is not an embedding. The band is a few ULPs of a radian rather than
 * a modelling tolerance: it catches the arms a sort genuinely could not separate,
 * and nothing that RoadGraph ordered on real angular difference. Widening it would
 * start re-ordering arms that differ by a real, if small, angle -- which the
 * geometric test below would then have to get right for no benefit.
 */
constexpr double kBearingEpsilon = 1e-12;

/// No such arm
constexpr size_t kNoArm = std::numeric_limits<size_t>::max();

// ============================================================================
// Geometry Helpers
// ============================================================================

/// True when two local-metre points coincide within kPointEpsilon
inline bool same_point(const glm::dvec2& a, const glm::dvec2& b) noexcept {
    const glm::dvec2 d = a - b;
    return (d.x * d.x + d.y * d.y) <= kPointEpsilonSq;
}

/// Euclidean distance in metres
inline double distance(const glm::dvec2& a, const glm::dvec2& b) noexcept {
    const glm::dvec2 d = a - b;
    return std::sqrt(d.x * d.x + d.y * d.y);
}

/// Point @p s metres along @p pts from its first vertex, clamped at both ends
glm::dvec2 point_at_arc(const std::vector<glm::dvec2>& pts, double s) {
    if (pts.empty()) {
        return glm::dvec2{0.0};
    }
    if (s <= 0.0) {
        return pts.front();
    }

    double run = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
        const double seg = distance(pts[i - 1], pts[i]);
        if (run + seg >= s) {
            if (seg <= 0.0) {
                return pts[i];
            }
            const double t = (s - run) / seg;
            return pts[i - 1] + (pts[i] - pts[i - 1]) * t;
        }
        run += seg;
    }
    return pts.back();
}

/// How two paths that leave a node on the same bearing relate to each other
enum class TiedOrder {
    FirstIsClockwise,   ///< `a` sorts before `b` in the anticlockwise rotation
    SecondIsClockwise,  ///< `b` sorts before `a`
    Coincident,         ///< the same geometry over the same length: no order exists
    Indeterminate,      ///< one runs along the other and stops, or they double back
};

/**
 * @brief Order two outgoing paths that share a bearing, by where they diverge
 *
 * Both paths start at the node. They run together for a while -- that is what a
 * shared bearing means -- and the one that bends LEFT at the point they part is
 * the one at the greater bearing, so it sorts later in the anticlockwise rotation.
 * That is the limit of the bearing the sort was trying to compare, taken at the
 * first place the two paths give it different values.
 *
 * Comparison is by ARC LENGTH, not vertex by vertex. Two ways can trace the same
 * ground with different vertices -- one of them split at a node the other does not
 * have -- and a vertex-by-vertex walk calls those two different at the first step.
 * Between two consecutive merged breakpoints both paths are straight, so agreeing
 * at every breakpoint is agreeing everywhere, and no sampling density has to be
 * guessed.
 *
 * @param a Outgoing path, a.front() at the node
 * @param b Outgoing path, b.front() at the same node
 */
TiedOrder compare_outgoing(const std::vector<glm::dvec2>& a, const std::vector<glm::dvec2>& b) {
    if (a.size() < 2 || b.size() < 2) {
        return TiedOrder::Indeterminate;
    }

    std::vector<double> stops;
    stops.reserve(a.size() + b.size());

    double run = 0.0;
    for (size_t i = 1; i < a.size(); ++i) {
        run += distance(a[i - 1], a[i]);
        stops.push_back(run);
    }
    const double total_a = run;

    run = 0.0;
    for (size_t i = 1; i < b.size(); ++i) {
        run += distance(b[i - 1], b[i]);
        stops.push_back(run);
    }
    const double total_b = run;

    const double shared = std::min(total_a, total_b);
    std::sort(stops.begin(), stops.end());

    // The last position at which the two paths still agreed. The turn is measured
    // from there, because that is the corner the two paths leave by.
    glm::dvec2 common = a.front();

    for (const double s : stops) {
        if (s > shared + kPointEpsilon) {
            break;
        }
        const glm::dvec2 pa = point_at_arc(a, s);
        const glm::dvec2 pb = point_at_arc(b, s);
        if (same_point(pa, pb)) {
            common = pa;
            continue;
        }

        const double cross = (pa.x - common.x) * (pb.y - common.y)
                           - (pa.y - common.y) * (pb.x - common.x);
        if (cross > 0.0) {
            return TiedOrder::FirstIsClockwise;   // b turned left, so b is the greater bearing
        }
        if (cross < 0.0) {
            return TiedOrder::SecondIsClockwise;
        }
        // Collinear and still apart: one path doubled back along the other. There
        // is no side to be on, so leave it to the caller's deterministic fallback.
        return TiedOrder::Indeterminate;
    }

    if (std::fabs(total_a - total_b) <= kPointEpsilon) {
        return TiedOrder::Coincident;
    }
    // One is a prefix of the other: a spur drawn on top of a longer road. No turn
    // exists to measure, so no side can be claimed.
    return TiedOrder::Indeterminate;
}

/**
 * @brief A ring vertex and the half-edge whose polyline carries the segment
 *        LEAVING it
 *
 * The owner travels with the point through the antenna prune, because pruning
 * changes which segment leaves a surviving vertex. Attributing segments to edges
 * afterwards, by position, would mean matching points back to polylines --
 * a proximity search, and wrong wherever two streets run close together.
 */
struct RingVertex {
    glm::dvec2 point{0.0};

    /// Index into the face's BlockEdge list, which becomes Block::edges
    uint32_t owner = 0;

    /**
     * @brief The graph node this vertex IS, or kInvalidId for a plain polyline vertex
     *
     * Carried so the pinch split can ask whether the boundary has come back to the
     * same NODE, rather than whether it has come back to the same coordinates.
     * Positions are not identity: RoadGraph splits a node per layer at a grade
     * separation, so two GraphNodes can share an osm_id and a position while being
     * different places the walk must not confuse.
     */
    GraphNodeId node = kInvalidId;
};

/**
 * @brief Drop ring vertices that repeat the position of the one before them
 *
 * A GUARD, and the only thing left of what used to remove antennae here. Antennae
 * were removed positionally, by spotting the pattern `x, tip, x`; that caught only
 * a spur whose two traversals were ADJACENT in the walk and left the stem of
 * anything bigger in the ring as a slit. Cut edges are removed topologically now,
 * in extract_blocks(), and what hangs off a single node is split off by
 * split_pinched_ring(), so nothing positional needs to cancel anything.
 *
 * What remains guards the one thing neither of those can see: a ring segment of
 * zero LENGTH. A graph straight out of RoadGraph::build() cannot produce one -- it
 * drops consecutive duplicates inside an edge and unites the slots either side of
 * a stretch that collapses to a point -- so this is for the day a resampling stage
 * writes geometry back into GraphEdge::polyline. A zero-length segment reaching C2
 * is a lot with a zero-length frontage.
 *
 * @param ring Ring to prune in place, first point not repeated
 */
void prune_antennae(std::vector<RingVertex>& ring) {
    std::vector<RingVertex> out;
    out.reserve(ring.size());

    for (const RingVertex& v : ring) {
        if (!out.empty() && same_point(out.back().point, v.point)) {
            // The same position twice running. A graph straight out of
            // RoadGraph::build() cannot produce this -- it drops consecutive
            // duplicates inside an edge, and unites the slots either side of a
            // stretch that collapses to a point, so no two adjacent ring vertices
            // coincide. This is a GUARD, for the day a resampling stage writes
            // geometry back into GraphEdge::polyline: a zero-length ring segment
            // reaching C2 would produce a lot with a zero-length frontage.
            //
            // Keeping the LATER owner is the same rule the antenna branch below
            // uses -- the segment that leaves a surviving vertex belongs to
            // whichever occurrence of it the walk left by last.
            out.back().owner = v.owner;
            continue;
        }

        out.push_back(v);
    }

    // The ring is cyclic, so the pair straddling the seam needs the same test, and
    // dropping one can expose another behind it.
    while (out.size() >= 3 && same_point(out.front().point, out.back().point)) {
        out.pop_back();
    }

    ring.swap(out);
}

/**
 * @brief Split a boundary curve that touches itself at a vertex into closed rings
 *
 * The cut-EDGE pass upstream handles a loop the face reaches along a street. This
 * is the same shape reached across a single NODE: a turning bulb or an estate loop
 * whose closed way shares one node with the road it hangs off, with no stem
 * between them. The face walks the road up to that node, all the way round the
 * loop, and back out along the road, so the boundary pinches to a point there.
 *
 * Area and perimeter come out right either way -- a pinch has no width, so nothing
 * is walked twice -- but the SHAPE does not. A pinched ring is not a polygon C2 can
 * offset or triangulate; a lot cut from it would straddle the pinch and own land on
 * both sides of a road. Split, it is what it always was: an outer boundary and a
 * loop of land inside it that belongs to someone else.
 *
 * The stack holds the curve being built and each node remembers where on it it
 * already sits, so meeting a node again closes off everything above it. Matching
 * is on the GRAPH NODE and never on the coordinates -- the same rule as everywhere
 * else in this pipeline, where a junction is shared node identity and never two
 * things that landed in the same place. A plain polyline vertex is not a node and
 * can never split a curve, which is what stops a street that merely passes over
 * its own coordinates from being cut in half.
 *
 * @param loop  Curve to split, first point not repeated
 * @param out   Closed rings, appended. A curve with no pinch appends exactly one.
 */
void split_pinched_ring(const std::vector<RingVertex>& loop, std::vector<uint32_t>& slot,
                        std::vector<std::vector<RingVertex>>& out) {
    std::vector<RingVertex> stack;
    stack.reserve(loop.size());
    std::vector<GraphNodeId> touched;   // nodes with a live entry in `slot`

    for (const RingVertex& v : loop) {
        if (v.node != kInvalidId && slot[v.node] != kInvalidId) {
            const size_t k = slot[v.node];
            out.emplace_back(stack.begin() + static_cast<std::ptrdiff_t>(k), stack.end());
            for (size_t i = k; i < stack.size(); ++i) {
                if (stack[i].node != kInvalidId) {
                    slot[stack[i].node] = kInvalidId;
                }
            }
            stack.resize(k);
        }
        if (v.node != kInvalidId) {
            // This occurrence takes the slot, and with it the owner: the segment
            // that leaves the surviving vertex is the one the walk left by last.
            slot[v.node] = static_cast<uint32_t>(stack.size());
            touched.push_back(v.node);
        }
        stack.push_back(v);
    }

    for (const GraphNodeId n : touched) {
        slot[n] = kInvalidId;       // leave the scratch clean for the next curve
    }
    out.push_back(std::move(stack));
}

/// Length of a closed ring in metres, first point not repeated
double ring_perimeter(const std::vector<glm::dvec2>& ring) {
    if (ring.size() < 2) {
        return 0.0;
    }

    double total = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        total += distance(ring[(i + 1) % ring.size()], ring[i]);
    }
    return total;
}

// ============================================================================
// Edge Filtering
// ============================================================================

/**
 * @brief Does this edge take part in the face traversal?
 *
 * Filtering here rather than after the walk is deliberate. Dropping a footway
 * AFTER the traversal would leave the sliver faces it created behind; dropping it
 * before makes the walk step straight over it, which MERGES the faces either side
 * of it into the one block the footway runs through. That is the whole point of
 * the filter.
 *
 * @param edge Edge to classify
 * @param cfg  Configuration carrying the class and layer switches
 */
bool class_accepted(const GraphEdge& edge, const BlockConfig& cfg) {
    if (!cfg.include_paths) {
        switch (edge.type) {
            case RoadType::Footway:
            case RoadType::Cycleway:
            case RoadType::Path:
                return false;
            default:
                break;
        }
    }

    if (!cfg.include_grade_separated && is_grade_separated(edge)) {
        return false;
    }

    return true;
}

/// Polyline of @p edge oriented to leave the lower-numbered of its two endpoints
std::vector<glm::dvec2> canonical_polyline(const GraphEdge& edge) {
    if (edge.from <= edge.to) {
        return edge.polyline;
    }
    return std::vector<glm::dvec2>(edge.polyline.rbegin(), edge.polyline.rend());
}

// ============================================================================
// Disjoint Set
// ============================================================================

/// Union-find over graph nodes, used only to count connected components
class NodeSets {
public:
    explicit NodeSets(size_t count) : m_parent(count) {
        for (size_t i = 0; i < count; ++i) {
            m_parent[i] = static_cast<uint32_t>(i);
        }
    }

    uint32_t find(uint32_t x) {
        while (m_parent[x] != x) {
            m_parent[x] = m_parent[m_parent[x]];    // halve the path as we go
            x = m_parent[x];
        }
        return x;
    }

    void unite(uint32_t a, uint32_t b) {
        const uint32_t ra = find(a);
        const uint32_t rb = find(b);
        if (ra != rb) {
            m_parent[ra] = rb;
        }
    }

private:
    std::vector<uint32_t> m_parent;
};

} // namespace

// ============================================================================
// Ring Helpers
// ============================================================================

double signed_ring_area(const std::vector<glm::dvec2>& ring) {
    if (ring.size() < 3) {
        return 0.0;
    }

    double twice_area = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % ring.size()];
        twice_area += a.x * b.y - b.x * a.y;
    }
    return 0.5 * twice_area;
}

bool is_grade_separated(const GraphEdge& edge) {
    // layer != 0 is included, not just the two booleans, because an edge can carry
    // layer=-1 with no tunnel=* tag -- a road in a cutting, or a way under a
    // building -- and it is just as capable of crossing another way without
    // sharing a node. The flag has to mean "this edge may not be coplanar with the
    // rest of the face", and that is the widest honest test available here.
    return edge.is_bridge || edge.is_tunnel || edge.layer != 0;
}

// ============================================================================
// Extraction
// ============================================================================

BlockExtraction extract_blocks(const RoadGraph& graph, const BlockConfig& config) {
    BlockExtraction result;

    const std::vector<GraphNode>& nodes = graph.nodes();
    const std::vector<GraphEdge>& edges = graph.edges();

    result.stats.edges_considered = edges.size();
    if (edges.empty()) {
        return result;
    }

    // ------------------------------------------------------------------------
    // Pass 1: decide which edges take part
    // ------------------------------------------------------------------------
    std::vector<uint8_t> usable(edges.size(), 0);
    for (size_t i = 0; i < edges.size(); ++i) {
        const GraphEdge& edge = edges[i];

        // An edge with a dangling endpoint or no direction would put the walk on a
        // node that does not exist. RoadGraph never emits one; this guards a
        // hand-built graph, and a test that builds one to prove it.
        if (edge.from >= nodes.size() || edge.to >= nodes.size() || edge.polyline.size() < 2) {
            continue;
        }
        if (!class_accepted(edge, config)) {
            continue;
        }

        usable[i] = 1;
    }

    // ------------------------------------------------------------------------
    // Pass 1b: drop edges that duplicate another edge's geometry exactly
    //
    // Two ways drawn along the same ground between the same two nodes give their
    // arms the SAME bearing at both ends. No tie-break can order them: an
    // embedding needs the two ends to disagree about which of the pair comes
    // first, and identical geometry gives nothing to disagree about. Left in, the
    // rotation is not an embedding, the orbit decomposition collapses, and what
    // vanishes is not the zero-width sliver between the duplicates -- it is the
    // real block around them. A mapping duplicate is not topology, so it goes.
    // ------------------------------------------------------------------------
    if (config.drop_duplicate_edges) {
        std::vector<EdgeId> candidates;
        for (size_t i = 0; i < edges.size(); ++i) {
            if (usable[i] != 0) {
                candidates.push_back(static_cast<EdgeId>(i));
            }
        }
        // Sorting by endpoint pair puts every possible duplicate next to its twin,
        // so the quadratic comparison below only ever runs inside one such group.
        std::sort(candidates.begin(), candidates.end(), [&edges](EdgeId a, EdgeId b) {
            const auto key = [&edges](EdgeId e) {
                return std::pair<GraphNodeId, GraphNodeId>{
                    std::min(edges[e].from, edges[e].to), std::max(edges[e].from, edges[e].to)};
            };
            return key(a) < key(b);
        });

        for (size_t i = 0; i < candidates.size(); ++i) {
            const EdgeId lhs = candidates[i];
            if (usable[lhs] == 0) {
                continue;
            }
            const std::vector<glm::dvec2> left = canonical_polyline(edges[lhs]);

            for (size_t j = i + 1; j < candidates.size(); ++j) {
                const EdgeId rhs = candidates[j];
                if (std::min(edges[rhs].from, edges[rhs].to) != std::min(edges[lhs].from, edges[lhs].to) ||
                    std::max(edges[rhs].from, edges[rhs].to) != std::max(edges[lhs].from, edges[lhs].to)) {
                    break;      // out of the group
                }
                if (usable[rhs] == 0) {
                    continue;
                }

                const std::vector<glm::dvec2> right = canonical_polyline(edges[rhs]);
                bool duplicate = compare_outgoing(left, right) == TiedOrder::Coincident;
                if (!duplicate && edges[lhs].from == edges[lhs].to) {
                    // A self-loop has no lower endpoint to canonicalise against, so
                    // the same bulb drawn the other way round is still a duplicate.
                    const std::vector<glm::dvec2> mirror(right.rbegin(), right.rend());
                    duplicate = compare_outgoing(left, mirror) == TiedOrder::Coincident;
                }

                if (duplicate) {
                    usable[rhs] = 0;    // keep the lower edge id: the traversal seeds in edge order
                    ++result.stats.duplicate_edges;
                }
            }
        }
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        if (usable[i] != 0) {
            ++result.stats.edges_used;
        }
    }

    // ------------------------------------------------------------------------
    // Pass 2: the rotation at every node
    //
    // GraphNode::arms is already ascending by bearing. All this does is re-order
    // the runs the sort could not order -- arms whose bearings are equal to within
    // rounding -- by where the two arms first part company. Everything else keeps
    // the order RoadGraph gave it.
    // ------------------------------------------------------------------------
    // One flat buffer with a per-node base, not a vector per node: a city extract
    // has six figures of nodes and almost none of them has a tie to re-order, so a
    // pair of heap blocks each would be the whole cost of this pass.
    std::vector<size_t> arm_base(nodes.size() + 1, 0);
    for (size_t n = 0; n < nodes.size(); ++n) {
        arm_base[n + 1] = arm_base[n] + nodes[n].arms.size();
    }
    std::vector<uint32_t> rotation(arm_base.back(), 0);
    std::vector<uint32_t> rank(arm_base.back(), 0);

    for (size_t n = 0; n < nodes.size(); ++n) {
        const std::vector<Arm>& arms = nodes[n].arms;
        uint32_t* const order = rotation.data() + arm_base[n];
        for (size_t i = 0; i < arms.size(); ++i) {
            order[i] = static_cast<uint32_t>(i);
        }

        // The path leaving this node along an arm: the edge's polyline, reversed
        // when the node is the edge's `to` end.
        const auto outgoing = [&](uint32_t slot) {
            const Arm& arm = arms[slot];
            const std::vector<glm::dvec2>& line = edges[arm.edge].polyline;
            if (arm.at_start) {
                return line;
            }
            return std::vector<glm::dvec2>(line.rbegin(), line.rend());
        };

        const auto precedes = [&](uint32_t lhs, uint32_t rhs) {
            switch (compare_outgoing(outgoing(lhs), outgoing(rhs))) {
                case TiedOrder::FirstIsClockwise:  return true;
                case TiedOrder::SecondIsClockwise: return false;
                default: break;
            }
            // Geometry cannot separate them. Fall back to something total, so the
            // rotation is at least the same on every run over the same graph -- a
            // block dump diffed against a previous run has to be stable or it is no
            // use as a regression test.
            if (arms[lhs].edge != arms[rhs].edge) {
                return arms[lhs].edge < arms[rhs].edge;
            }
            return arms[lhs].at_start && !arms[rhs].at_start;
        };

        size_t begin = 0;
        while (begin < arms.size()) {
            size_t end = begin + 1;
            while (end < arms.size() &&
                   std::fabs(arms[order[end]].bearing - arms[order[begin]].bearing) <= kBearingEpsilon) {
                ++end;
            }

            // Insertion sort, not std::sort. `precedes` is a geometric test on
            // paths that may not be mutually consistent -- three arms can each run
            // along the next -- and std::sort on a comparator that is not a strict
            // weak ordering is undefined behaviour, while insertion sort merely
            // gives an order nobody can complain about.
            for (size_t i = begin + 1; i < end; ++i) {
                for (size_t j = i; j > begin && precedes(order[j], order[j - 1]); --j) {
                    std::swap(order[j], order[j - 1]);
                }
            }

            begin = end;
        }

        for (size_t i = 0; i < arms.size(); ++i) {
            rank[arm_base[n] + order[i]] = static_cast<uint32_t>(i);
        }
    }

    // ------------------------------------------------------------------------
    // Pass 3: how many faces a planar embedding of this subgraph must have
    //
    // Euler's formula over the USABLE subgraph. The traversal cannot check its own
    // work -- every orbit it finds looks like a face -- so this is the only number
    // that says a face went missing under a non-planar crossing.
    // ------------------------------------------------------------------------
    {
        NodeSets sets(nodes.size());
        std::vector<uint8_t> node_used(nodes.size(), 0);
        for (size_t i = 0; i < edges.size(); ++i) {
            if (usable[i] == 0) {
                continue;
            }
            node_used[edges[i].from] = 1;
            node_used[edges[i].to] = 1;
            sets.unite(edges[i].from, edges[i].to);
        }

        size_t nodes_used = 0;
        size_t components = 0;
        for (size_t n = 0; n < nodes.size(); ++n) {
            if (node_used[n] == 0) {
                continue;
            }
            ++nodes_used;
            if (sets.find(static_cast<uint32_t>(n)) == n) {
                ++components;
            }
        }

        if (result.stats.edges_used > 0) {
            const long long euler = static_cast<long long>(result.stats.edges_used)
                                  - static_cast<long long>(nodes_used)
                                  + 2LL * static_cast<long long>(components);
            result.stats.expected_faces = euler > 0 ? static_cast<size_t>(euler) : 0;
        }
    }

    // Index of the arm at `node` that belongs to `edge` on the given end.
    //
    // Unique by construction: RoadGraph gives every edge exactly one arm with
    // at_start true and one with it false, so the two arms a SELF-LOOP contributes
    // to a single node stay distinguishable. Matching on edge id alone picks
    // whichever of the two comes first, which sends the walk round a closed way in
    // the wrong direction and merges its two faces into one malformed orbit.
    auto arm_slot = [&nodes](GraphNodeId node, EdgeId edge, bool at_start) -> size_t {
        const std::vector<Arm>& arms = nodes[node].arms;
        for (size_t i = 0; i < arms.size(); ++i) {
            if (arms[i].edge == edge && arms[i].at_start == at_start) {
                return i;
            }
        }
        return kNoArm;
    };

    // ------------------------------------------------------------------------
    // Pass 4: one orbit per unvisited half-edge
    //
    // Half-edge h is edge h/2, travelled from `from` to `to` when h is even. The
    // seed order is therefore edge order, which makes the block ids deterministic
    // for a given graph -- a dump diffed against a previous run has to be stable
    // or it is no use as a regression test.
    // ------------------------------------------------------------------------
    const size_t half_edge_count = edges.size() * 2;
    std::vector<uint8_t> visited(half_edge_count, 0);

    std::vector<RingVertex> raw;            // every point the face walked, in order
    std::vector<uint32_t> run_begin;        // raw index where each traversal's points start
    std::vector<BlockEdge> walked;

    // Per-face bookkeeping over edges, stamped rather than cleared: a face touches
    // a handful of edges and clearing a vector the size of the graph for each one
    // would make the whole pass quadratic.
    std::vector<uint32_t> edge_stamp(edges.size(), 0);
    std::vector<uint32_t> edge_first(edges.size(), 0);
    std::vector<uint8_t> edge_twice(edges.size(), 0);

    std::vector<RingVertex> loop;                   // one boundary curve's points, reused
    std::vector<std::vector<RingVertex>> pieces;    // that curve split at its pinches
    // Where each node currently sits on the curve being split, kInvalidId for none.
    // One buffer for the whole pass, left clean by split_pinched_ring().
    std::vector<uint32_t> pinch_slot(nodes.size(), kInvalidId);
    std::vector<std::vector<uint32_t>> groups;      // one closed sub-walk per boundary curve
    std::vector<std::vector<uint32_t>> pending;     // groups suspended while a cut edge is open
    std::vector<EdgeId> open_edge;
    std::vector<uint32_t> current;

    for (size_t seed = 0; seed < half_edge_count; ++seed) {
        if (usable[seed / 2] == 0 || visited[seed] != 0) {
            continue;
        }

        raw.clear();
        run_begin.clear();
        walked.clear();
        bool grade_separated = false;
        bool malformed = false;

        size_t h = seed;
        size_t steps = 0;
        for (;;) {
            visited[h] = 1;
            ++result.stats.half_edges_walked;

            const EdgeId edge_id = static_cast<EdgeId>(h / 2);
            const bool forward = (h % 2) == 0;
            const GraphEdge& edge = edges[edge_id];

            const uint32_t owner = static_cast<uint32_t>(walked.size());
            walked.push_back(BlockEdge{edge_id, forward});
            run_begin.push_back(static_cast<uint32_t>(raw.size()));
            grade_separated = grade_separated || is_grade_separated(edge);

            // Contribute every polyline vertex EXCEPT the last. The last is the
            // node this half-edge arrives at, and the next half-edge contributes it
            // as its own first point. That is what leaves the finished ring with
            // its first point not repeated, and it is why no seam dedup is needed.
            //
            // The FIRST of them is the node this half-edge leaves, and it is tagged
            // with that node's id. Everything after it is a plain polyline vertex
            // and is tagged with nothing, so split_pinched_ring() can tell a place
            // the boundary has genuinely returned to from a place it merely passes
            // through again.
            const GraphNodeId departure = forward ? edge.from : edge.to;
            if (forward) {
                for (size_t i = 0; i + 1 < edge.polyline.size(); ++i) {
                    raw.push_back(RingVertex{edge.polyline[i], owner,
                                             i == 0 ? departure : kInvalidId});
                }
            } else {
                for (size_t i = edge.polyline.size(); i-- > 1;) {
                    raw.push_back(RingVertex{edge.polyline[i], owner,
                                             i + 1 == edge.polyline.size() ? departure
                                                                           : kInvalidId});
                }
            }

            const GraphNodeId arrival = forward ? edge.to : edge.from;

            // The arm pointing back the way we came: at the `to` end when we
            // travelled forward, at the `from` end when we travelled backward.
            const size_t back_arm = arm_slot(arrival, edge_id, !forward);
            if (back_arm == kNoArm) {
                malformed = true;   // the edge is not registered on its own node
                break;
            }

            const std::vector<Arm>& arms = nodes[arrival].arms;
            const size_t arm_count = arms.size();
            const size_t back_rank = rank[arm_base[arrival] + back_arm];

            // Leave by the next arm CLOCKWISE. The rotation ascends by bearing,
            // which runs anticlockwise, so clockwise is backwards through it.
            // Unusable arms are stepped over, which is how excluding footways merges
            // the faces either side of one instead of leaving a gap in the boundary.
            //
            // k runs to arm_count inclusive so the last candidate is back_arm
            // itself: at a dead end that is the only arm there is, and leaving by
            // the arm we arrived on is exactly the walk out and back that a
            // cul-de-sac needs.
            size_t leave = kNoArm;
            for (size_t k = 1; k <= arm_count; ++k) {
                const size_t idx =
                    rotation[arm_base[arrival] + ((back_rank + arm_count - k) % arm_count)];
                if (arms[idx].edge < edges.size() && usable[arms[idx].edge] != 0) {
                    leave = idx;
                    break;
                }
            }
            if (leave == kNoArm) {
                malformed = true;   // impossible: the arm we arrived on is usable
                break;
            }

            // Leaving along an arm means travelling its edge AWAY from this node,
            // so the direction is forward exactly when this node is the edge's
            // `from` -- which is what Arm::at_start records.
            const Arm& out_arm = arms[leave];
            h = static_cast<size_t>(out_arm.edge) * 2 + (out_arm.at_start ? 0 : 1);

            if (h == seed) {
                break;      // orbit closed
            }
            if (visited[h] != 0 || ++steps > half_edge_count) {
                // Unreachable while next() is a bijection over the usable
                // half-edges, which it is: reversing is a bijection and stepping to
                // the previous usable arm is a cyclic shift within a node. Kept
                // because the alternative failure is an infinite loop on a graph
                // some future change makes inconsistent.
                malformed = true;
                break;
            }
        }
        run_begin.push_back(static_cast<uint32_t>(raw.size()));

        const uint32_t stamp = static_cast<uint32_t>(result.stats.faces + 1);
        ++result.stats.faces;
        if (malformed) {
            ++result.stats.malformed_faces;
            continue;
        }

        // --------------------------------------------------------------------
        // Split the face's walk into its boundary curves at the CUT EDGES
        //
        // An edge whose two half-edges are both in this face has the same land on
        // both sides, so it bounds nothing: it is a cut edge of the face. That
        // test is topological and holds however far apart the two traversals are.
        // The positional `x, tip, x` test it replaces only ever saw the two
        // traversals that happen to be ADJACENT -- a bare cul-de-sac -- and left
        // the stem of a lollipop bulb, or of an estate off one access road, in the
        // ring as a zero-width slit that no offset or triangulation can take.
        //
        // The two traversals of a cut edge bracket everything the face walked on
        // the far side of it, so the walk reads as nested pairs of brackets. What
        // is inside a pair is a boundary curve of its own -- the loop the face
        // reaches across the cut edge, which becomes a hole -- and what is outside
        // closes up behind it.
        // --------------------------------------------------------------------
        for (uint32_t o = 0; o < walked.size(); ++o) {
            const EdgeId e = walked[o].edge;
            if (edge_stamp[e] != stamp) {
                edge_stamp[e] = stamp;
                edge_first[e] = o;
                edge_twice[e] = 0;
            } else {
                edge_twice[e] = 1;
            }
        }

        groups.clear();
        pending.clear();
        open_edge.clear();
        current.clear();

        for (uint32_t o = 0; o < walked.size(); ++o) {
            const EdgeId e = walked[o].edge;   // stamped for this face by the loop above
            if (edge_twice[e] == 0) {
                current.push_back(o);
                continue;
            }

            if (o == edge_first[e]) {
                pending.push_back(std::move(current));
                current.clear();
                open_edge.push_back(e);
                continue;
            }

            if (open_edge.empty() || open_edge.back() != e) {
                // The brackets crossed. A planar face walk cannot do that, so the
                // embedding is not planar here -- two ways crossing on a layer they
                // do not share a node on. Say so rather than emit a curve that is
                // neither the boundary nor a hole.
                malformed = true;
                break;
            }
            groups.push_back(std::move(current));
            current = std::move(pending.back());
            pending.pop_back();
            open_edge.pop_back();
        }

        if (malformed || !open_edge.empty()) {
            ++result.stats.malformed_faces;
            continue;
        }
        groups.push_back(std::move(current));

        // --------------------------------------------------------------------
        // One ring per boundary curve, then classify the face by their total
        // --------------------------------------------------------------------
        std::vector<std::vector<glm::dvec2>> loops;
        std::vector<std::vector<uint32_t>> loop_owners;
        std::vector<double> loop_areas;
        double total_area = 0.0;

        for (const std::vector<uint32_t>& group : groups) {
            loop.clear();
            for (const uint32_t o : group) {
                for (uint32_t i = run_begin[o]; i < run_begin[o + 1]; ++i) {
                    loop.push_back(raw[i]);
                }
            }
            prune_antennae(loop);
            if (loop.size() < 3) {
                continue;
            }

            pieces.clear();
            split_pinched_ring(loop, pinch_slot, pieces);

            for (const std::vector<RingVertex>& piece : pieces) {
                if (piece.size() < 3) {
                    continue;
                }

                std::vector<glm::dvec2> points;
                std::vector<uint32_t> owners;
                points.reserve(piece.size());
                owners.reserve(piece.size());
                for (const RingVertex& vertex : piece) {
                    points.push_back(vertex.point);
                    owners.push_back(vertex.owner);
                }

                const double area = signed_ring_area(points);
                total_area += area;
                loop_areas.push_back(area);
                loops.push_back(std::move(points));
                loop_owners.push_back(std::move(owners));
            }
        }

        if (loops.empty()) {
            // A component that is a tree: every edge is a cut edge, so once they
            // are gone there is no boundary left. One face, no block. This is also
            // the single face an isolated dead-end road produces.
            ++result.stats.tree_faces;
            continue;
        }

        // The outer face is rejected on the SIGN, never on the size. One outer face
        // exists per connected component, and with two components in an extract the
        // smaller one's outer face is smaller than the larger one's blocks -- so
        // "drop the largest face" drops a real block and keeps an outer face.
        if (total_area <= -kAreaEpsilon) {
            ++result.stats.outer_faces;
            continue;
        }
        if (total_area < kAreaEpsilon) {
            // A boundary with no interior: a cycle whose nodes are collinear.
            ++result.stats.zero_area_faces;
            continue;
        }
        if (total_area < config.min_area) {
            ++result.stats.rejected_small;
            continue;
        }

        // Exactly one curve of a bounded face runs anticlockwise: its outer
        // boundary. Every other curve is a loop the face walked round, which
        // encloses land this block does not own, so it comes out clockwise. Two
        // anticlockwise curves would mean two separate outer boundaries in one
        // face, which a planar embedding cannot produce.
        size_t outer = loops.size();
        size_t positives = 0;
        for (size_t i = 0; i < loops.size(); ++i) {
            if (loop_areas[i] >= kAreaEpsilon) {
                ++positives;
                outer = i;
            }
        }
        if (positives != 1) {
            ++result.stats.malformed_faces;
            continue;
        }

        Block block;
        block.id = static_cast<BlockId>(result.blocks.size());
        block.ring = std::move(loops[outer]);
        block.ring_edges = std::move(loop_owners[outer]);
        block.perimeter = ring_perimeter(block.ring);
        for (size_t i = 0; i < loops.size(); ++i) {
            if (i == outer || loop_areas[i] > -kAreaEpsilon) {
                continue;       // itself, or a curve that encloses nothing
            }
            block.perimeter += ring_perimeter(loops[i]);
            block.holes.push_back(std::move(loops[i]));
            block.hole_edges.push_back(std::move(loop_owners[i]));
        }
        // Copied, not moved: `walked` is reused across faces and a move would
        // hand its buffer away, so every face after the first would reallocate.
        block.edges = walked;
        block.area = total_area;
        block.has_grade_separated_edge = grade_separated;
        result.blocks.push_back(std::move(block));
    }

    result.stats.blocks = result.blocks.size();

    spdlog::info("Blocks: {} blocks from {} faces (Euler expects {}) -- {} outer, {} tree, "
                 "{} zero-area, {} malformed, {} below {:.0f} m2, {} of {} edges used, "
                 "{} duplicates dropped",
                 result.stats.blocks, result.stats.faces, result.stats.expected_faces,
                 result.stats.outer_faces, result.stats.tree_faces, result.stats.zero_area_faces,
                 result.stats.malformed_faces, result.stats.rejected_small, config.min_area,
                 result.stats.edges_used, result.stats.edges_considered,
                 result.stats.duplicate_edges);

    if (result.stats.malformed_faces > 0 || result.stats.faces < result.stats.expected_faces) {
        // Not info. Either the graph contradicted itself, or the drawing has a
        // crossing the data does not share a node on and a block that should exist
        // does not. Both mean the output is short of blocks, and neither is
        // visible in the block list itself.
        spdlog::warn("Blocks: {} face(s) abandoned and {} of {} expected faces found -- "
                     "the street graph is not planar where it was walked",
                     result.stats.malformed_faces, result.stats.faces,
                     result.stats.expected_faces);
    }

    return result;
}

} // namespace stratum::osm::road
