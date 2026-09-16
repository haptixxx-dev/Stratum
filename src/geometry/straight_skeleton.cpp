// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file straight_skeleton.cpp
 * @brief Wavefront simulation behind straight_skeleton.hpp
 *
 * The header says WHAT a straight skeleton is and what this refuses to do. This
 * file is about HOW, and about the three places the arithmetic has to be handled
 * carefully:
 *
 *   1. **A wavefront edge never changes direction, only offset.** Every active
 *      edge is a piece of one CONTOUR edge, translated inward by the current
 *      time. So an edge's supporting line at time t is
 *      `dot(x, normal) == offset0 + t` with `normal` and `offset0` fixed at t=0,
 *      and nothing about an edge has to be re-derived as the simulation runs.
 *      Re-deriving it from the moved endpoints -- which is the obvious thing to
 *      do -- accumulates error into the direction, and a direction that drifts
 *      turns a parallel pair of edges into a converging pair and invents an event
 *      that should never happen.
 *   2. **A wavefront vertex is the intersection of its two edges' lines**, so its
 *      position at time t is `origin + velocity * t` exactly, with `velocity`
 *      solved once from the two normals. Positions are therefore never
 *      integrated step by step, and a long simulation is no less accurate than a
 *      short one.
 *   3. **Event times are computed relative to NOW, not to zero.** `t_now + dt`
 *      where `dt` comes from the current gap and the current closing rate. The
 *      absolute form loses precision once the wavefront has travelled far
 *      compared with the remaining gap, and the symptom is an event time slightly
 *      BEFORE the present, which then re-fires forever.
 */

#include "geometry/straight_skeleton.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stratum::geometry {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/**
 * @brief Below this, two unit normals are parallel and the 2x2 velocity solve is singular
 *
 * The determinant of the solve is the sine of the angle between the two edge
 * normals, so this is an angle threshold in disguise: about 1e-12 radians. It is
 * deliberately far tighter than the point epsilon. Widening it would make a
 * genuinely sharp -- but real -- corner take the "collinear" branch and move along
 * one edge's normal instead of along the mitre, which puts the vertex off both of
 * its own edges.
 */
constexpr double kParallelEpsilon = 1e-12;

/**
 * @brief Slowest closing rate that counts as closing
 *
 * Below it the two things are parallel for practical purposes and no event is
 * scheduled. Scheduling one anyway divides by a near-zero rate and produces an
 * event time of 1e18, which sorts last and never fires but does pollute the
 * comparison with an infinity when the rate is exactly zero.
 */
constexpr double kRateEpsilon = 1e-12;

/**
 * @brief Fastest a wavefront vertex may move before it is called degenerate
 *
 * A vertex's speed is `1 / sin(half the interior angle)`, so a needle-sharp
 * corner legitimately moves thousands of times faster than the wavefront. A
 * million is about a corner of two microradians, which is not a corner in any
 * map. Past it the mitre point is so far from the polygon that every event time
 * computed from it is noise, so the vertex is marked degenerate and the skeleton
 * comes back incomplete instead of coming back wrong.
 */
constexpr double kMaxVertexSpeed = 1.0e6;

/// Below this many square units a ring encloses nothing
constexpr double kAreaEpsilon = 1e-12;

/// No such index
constexpr uint32_t kNone = 0xFFFFFFFFu;

// ============================================================================
// Small Geometry
// ============================================================================

[[nodiscard]] inline double cross2(const glm::dvec2& a, const glm::dvec2& b) noexcept {
    return a.x * b.y - a.y * b.x;
}

/// Quarter turn anticlockwise. For an anticlockwise ring this maps an edge
/// direction to the inward normal, which is the sign convention the whole file
/// rests on: flip it and every event time changes sign.
[[nodiscard]] inline glm::dvec2 perp(const glm::dvec2& v) noexcept {
    return glm::dvec2{-v.y, v.x};
}

[[nodiscard]] inline double length2(const glm::dvec2& v) noexcept {
    return v.x * v.x + v.y * v.y;
}

/**
 * @brief Solve for the velocity of the vertex where two moving edges meet
 *
 * The vertex sits on both lines at all times, so `dot(v, n_left) == 1` and
 * `dot(v, n_right) == 1`: each line advances by one per unit time along its own
 * normal, and the vertex has to keep up with both. Two equations, two unknowns,
 * Cramer's rule, no trigonometry.
 *
 * @return False for a spike, whose two edges fold back on each other and whose
 *         meeting point runs off to infinity.
 */
[[nodiscard]] bool solve_velocity(const glm::dvec2& n_left, const glm::dvec2& n_right,
                                  glm::dvec2& out) noexcept {
    const double det = cross2(n_left, n_right);
    if (std::fabs(det) > kParallelEpsilon) {
        out = glm::dvec2{(n_right.y - n_left.y) / det, (n_left.x - n_right.x) / det};
        // A mitre point far enough out is numerically meaningless; see kMaxVertexSpeed.
        return length2(out) <= kMaxVertexSpeed * kMaxVertexSpeed;
    }

    // Parallel normals. Same direction means the two edges are collinear and the
    // vertex simply rides the shared normal -- a real and common case, because a
    // block ring carries every polyline vertex of a straight street. Opposite
    // directions mean a 180-degree spike, which has no finite meeting point.
    if (glm::dot(n_left, n_right) > 0.0) {
        out = n_left;
        return true;
    }
    return false;
}

/// True when segments (a,b) and (c,d) cross at a point interior to both
[[nodiscard]] bool segments_properly_cross(const glm::dvec2& a, const glm::dvec2& b,
                                           const glm::dvec2& c, const glm::dvec2& d) noexcept {
    const double d1 = cross2(b - a, c - a);
    const double d2 = cross2(b - a, d - a);
    const double d3 = cross2(d - c, a - c);
    const double d4 = cross2(d - c, b - c);
    return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
           ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
}

// ============================================================================
// Simulation State
// ============================================================================

/**
 * @brief One live piece of one contour edge
 *
 * A contour edge starts as exactly one instance. A split event cuts an instance
 * in two, and both halves keep the SAME supporting line, direction and contour
 * index -- they are still that one edge, just two disjoint stretches of it now
 * belonging to two separate wavefront loops.
 *
 * `start_chain` and `end_chain` are what make faces possible. Each records the
 * skeleton nodes its start (respectively end) vertex has occupied, oldest first.
 * When the instance finally collapses, both chains end at the same node -- the
 * apex -- and the two chains plus the contour edge bound the face.
 */
struct Inst {
    uint32_t orig = kNone;       ///< Contour edge index; never changes
    glm::dvec2 dir{0.0};         ///< Unit direction of the contour edge; never changes
    glm::dvec2 normal{0.0};      ///< Inward unit normal; never changes
    double offset0 = 0.0;        ///< dot(edge start, normal) at t = 0; never changes

    uint32_t start_v = kNone;    ///< Live vertex at the instance's start
    uint32_t end_v = kNone;      ///< Live vertex at its end

    std::vector<uint32_t> start_chain;
    std::vector<uint32_t> end_chain;

    /// Next instance of the same contour edge, in the contour's own direction.
    /// A split inserts the far half here, so walking this list from instance
    /// `e` visits every stretch of contour edge `e` in order.
    uint32_t next_inst = kNone;

    bool alive = true;
};

/// One moving wavefront vertex
struct Wv {
    glm::dvec2 origin{0.0};  ///< Position extrapolated back to t = 0
    glm::dvec2 vel{0.0};     ///< Constant velocity; position is origin + vel * t

    uint32_t node = kInvalidSkeletonNode;  ///< Skeleton node where this vertex was born
    uint32_t left_inst = kNone;            ///< Instance arriving from `prev`
    uint32_t right_inst = kNone;           ///< Instance leaving toward `next`
    uint32_t prev = kNone;
    uint32_t next = kNone;
    uint32_t loop = 0;  ///< Which wavefront loop; a split makes a new one

    bool alive = true;
    bool reflex = false;
    /// False when solve_velocity() refused. Such a vertex generates no events, so
    /// a wavefront containing one stalls and the skeleton comes back incomplete.
    bool finite = true;
};

/// A candidate change to the wavefront
struct Event {
    double time = 0.0;
    int kind = -1;  ///< 0 edge event, 1 split event, -1 none
    uint32_t a = 0; ///< The vertex that dies
    uint32_t b = 0; ///< Edge event: `a`'s successor. Split event: the instance hit.
    glm::dvec2 point{0.0};
};

/**
 * @brief Total order on events, so ties never fall to container order
 *
 * Earliest first, then SPLIT events before edge events, then by index. Two runs on
 * the same ring pick the same event every time, which is what makes the whole
 * output reproducible.
 *
 * **The kind tie-break is load-bearing and it goes this way round.** Simultaneous
 * events are not exotic -- a T-shaped block whose bar is as deep as the reflex
 * vertices are from the far kerb produces two splits and two edge events all at
 * the same instant, and so does every rectangle with a notch cut to the centre
 * line. Taking the edge event first collapses the bar into a degenerate
 * wavefront, and then both split events are silently discarded: their reflex
 * vertices now sit exactly ON the edge they were going to cut, so the
 * strictly-inside test rejects them. The wavefront folds through itself and the
 * faces overlap, which showed up as face areas summing to more than the polygon.
 *
 * Preferring the split has no matching failure. A split needs its target edge to
 * still exist, and taking it first is precisely what guarantees that; an edge
 * event needs nothing from the split, and if the split shortened its edge the
 * event is simply recomputed on the next pass, which costs one iteration.
 */
[[nodiscard]] bool better_event(const Event& candidate, const Event& best) noexcept {
    if (best.kind < 0) return true;
    if (candidate.time != best.time) return candidate.time < best.time;
    if (candidate.kind != best.kind) return candidate.kind > best.kind;
    if (candidate.a != best.a) return candidate.a < best.a;
    return candidate.b < best.b;
}

// ============================================================================
// The Simulation
// ============================================================================

class Sim {
public:
    Sim(const std::vector<glm::dvec2>& contour, const SkeletonConfig& config)
        : eps_(config.point_epsilon > 0.0 ? config.point_epsilon : 1e-6) {
        out_.contour = contour;
    }

    StraightSkeleton run(size_t max_iterations);

private:
    [[nodiscard]] glm::dvec2 pos(uint32_t v, double t) const noexcept {
        return verts_[v].origin + verts_[v].vel * t;
    }

    uint32_t add_node(const glm::dvec2& p, double t, bool on_contour);
    void add_arc(uint32_t from_node, uint32_t to_node, uint32_t left_inst, uint32_t right_inst);
    uint32_t make_vertex(uint32_t left_inst, uint32_t right_inst, const glm::dvec2& p, double t,
                         uint32_t node, uint32_t loop);
    void close_inst(uint32_t inst, uint32_t node);
    void relabel_loop(uint32_t start_vertex, uint32_t loop);

    void seed();
    void collapse_small_loops(double t);
    size_t resolve_flat_loops(double t, size_t min_members);
    [[nodiscard]] Event find_event(double t) const;
    void apply_edge_event(const Event& ev);
    void apply_split_event(const Event& ev);
    void build_faces();

    double eps_ = 1e-6;
    /// Next free wavefront-loop id. Loop 0 is the input ring; every split makes
    /// one more, and ids are never reused, so a stale id can never alias a live
    /// loop and let two disconnected wavefronts see each other.
    uint32_t next_loop_ = 1;
    std::vector<Inst> insts_;
    std::vector<Wv> verts_;
    StraightSkeleton out_;
};

uint32_t Sim::add_node(const glm::dvec2& p, double t, bool on_contour) {
    SkeletonNode node;
    node.position = p;
    node.time = t;
    node.on_contour = on_contour;
    out_.nodes.push_back(node);
    out_.stats.max_time = std::max(out_.stats.max_time, t);
    return static_cast<uint32_t>(out_.nodes.size() - 1);
}

void Sim::add_arc(uint32_t from_node, uint32_t to_node, uint32_t left_inst, uint32_t right_inst) {
    // A zero-length arc is not geometry, it is two events that happened at the
    // same place. Dropping it here keeps callers from having to filter degenerate
    // segments out of every skeleton they draw or build on.
    if (from_node == to_node) return;
    SkeletonArc arc;
    arc.from = from_node;
    arc.to = to_node;
    arc.left_edge = (left_inst == kNone) ? kNoSkeletonEdge : insts_[left_inst].orig;
    arc.right_edge = (right_inst == kNone) ? kNoSkeletonEdge : insts_[right_inst].orig;
    out_.arcs.push_back(arc);
}

uint32_t Sim::make_vertex(uint32_t left_inst, uint32_t right_inst, const glm::dvec2& p, double t,
                          uint32_t node, uint32_t loop) {
    Wv v;
    v.left_inst = left_inst;
    v.right_inst = right_inst;
    v.node = node;
    v.loop = loop;
    v.finite = solve_velocity(insts_[left_inst].normal, insts_[right_inst].normal, v.vel);
    if (!v.finite) {
        v.vel = glm::dvec2{0.0};
        ++out_.stats.degenerate_vertices;
    }
    // Extrapolated back to zero, so pos() is one multiply-add at any time rather
    // than an accumulation from the birth time.
    v.origin = p - v.vel * t;
    v.reflex = cross2(insts_[left_inst].dir, insts_[right_inst].dir) < 0.0;
    verts_.push_back(v);
    return static_cast<uint32_t>(verts_.size() - 1);
}

void Sim::close_inst(uint32_t inst, uint32_t node) {
    Inst& e = insts_[inst];
    if (!e.alive) return;
    e.start_chain.push_back(node);
    e.end_chain.push_back(node);
    e.alive = false;
    e.start_v = kNone;
    e.end_v = kNone;
}

void Sim::relabel_loop(uint32_t start_vertex, uint32_t loop) {
    uint32_t v = start_vertex;
    // Bounded by the vertex count: `next` is a permutation of the live vertices,
    // so a walk either returns to its start or the structure is already corrupt.
    for (size_t guard = 0; guard <= verts_.size(); ++guard) {
        verts_[v].loop = loop;
        v = verts_[v].next;
        if (v == start_vertex) return;
        if (v == kNone) return;
    }
}

void Sim::seed() {
    const std::vector<glm::dvec2>& ring = out_.contour;
    const uint32_t n = static_cast<uint32_t>(ring.size());

    for (uint32_t i = 0; i < n; ++i) {
        add_node(ring[i], 0.0, true);
    }

    insts_.resize(n);
    for (uint32_t i = 0; i < n; ++i) {
        const glm::dvec2 delta = ring[(i + 1) % n] - ring[i];
        const double len = std::sqrt(length2(delta));
        Inst& e = insts_[i];
        e.orig = i;
        e.dir = delta / len;  // len > eps_ is guaranteed by the caller's dedupe
        e.normal = perp(e.dir);
        e.offset0 = glm::dot(ring[i], e.normal);
        e.start_v = i;
        e.end_v = (i + 1) % n;
        e.start_chain.push_back(i);
        e.end_chain.push_back((i + 1) % n);
    }

    verts_.reserve(static_cast<size_t>(n) * 2);
    for (uint32_t i = 0; i < n; ++i) {
        Wv v;
        v.left_inst = (i + n - 1) % n;
        v.right_inst = i;
        v.prev = (i + n - 1) % n;
        v.next = (i + 1) % n;
        v.node = i;
        v.loop = 0;
        v.finite = solve_velocity(insts_[v.left_inst].normal, insts_[v.right_inst].normal, v.vel);
        if (!v.finite) {
            v.vel = glm::dvec2{0.0};
            ++out_.stats.degenerate_vertices;
        }
        v.origin = ring[i];
        v.reflex = cross2(insts_[v.left_inst].dir, insts_[v.right_inst].dir) < 0.0;
        if (v.reflex) ++out_.stats.reflex_vertices;
        verts_.push_back(v);
    }
}

void Sim::collapse_small_loops(double t) {
    // A loop of one or two vertices encloses nothing: with two vertices, both are
    // the intersection of the SAME two moving lines, so they are the same point at
    // every time. Simulating it further would produce an event at a rate of zero.
    std::vector<uint32_t> seen_loops;
    for (uint32_t i = 0; i < verts_.size(); ++i) {
        if (!verts_[i].alive) continue;
        const uint32_t loop = verts_[i].loop;
        if (std::find(seen_loops.begin(), seen_loops.end(), loop) != seen_loops.end()) continue;
        seen_loops.push_back(loop);

        std::vector<uint32_t> members;
        for (uint32_t j = 0; j < verts_.size(); ++j) {
            if (verts_[j].alive && verts_[j].loop == loop) members.push_back(j);
        }
        if (members.size() >= 3) continue;

        glm::dvec2 sum{0.0};
        for (const uint32_t m : members) sum += pos(m, t);
        const uint32_t node = add_node(sum / static_cast<double>(members.size()), t, false);

        std::vector<uint32_t> touched;
        for (const uint32_t m : members) {
            add_arc(verts_[m].node, node, verts_[m].left_inst, verts_[m].right_inst);
            touched.push_back(verts_[m].left_inst);
            touched.push_back(verts_[m].right_inst);
            verts_[m].alive = false;
        }
        for (const uint32_t inst : touched) close_inst(inst, node);
        ++out_.stats.degenerate_loops;
    }
}

/**
 * @brief Finish off any wavefront loop that has run out of area
 *
 * The case this exists for: an L-shaped polygon whose two arms are the same
 * width. Both arms flatten at the SAME instant, and the wavefront at that instant
 * is not a polygon at all -- it is the two ridge segments, traced out and back. A
 * vertex where two exactly opposing edges meet has no finite velocity (see
 * Wv::finite), so such a wavefront generates no further events and the simulation
 * stalls with the skeleton half built.
 *
 * Handling it as a shortcut rather than as an exact degeneracy analysis is
 * deliberate. A loop of zero area encloses no land, so there is nothing left to
 * decide: every vertex is already at its final position, and the only thing
 * missing is the arcs between them. Those are read straight off the loop.
 *
 * The alternative -- perturbing the input until the simultaneity goes away -- was
 * rejected because it moves the answer. A rectangle's ridge would come out a
 * micron off centre, and D5 would build a roof with a visible kink in it.
 *
 * @param min_members Only touch loops of at least this many vertices. The normal
 *                    end of a wavefront is a triangle collapsing to a point, and
 *                    that path is exact and must not be diverted here; passing 4
 *                    leaves it alone, and passing 1 is the last-resort sweep when
 *                    no event could be found at all.
 * @return How many loops were resolved
 */
size_t Sim::resolve_flat_loops(double t, size_t min_members) {
    size_t resolved = 0;
    std::vector<uint32_t> seen_loops;

    for (uint32_t i = 0; i < verts_.size(); ++i) {
        if (!verts_[i].alive) continue;
        const uint32_t loop = verts_[i].loop;
        if (std::find(seen_loops.begin(), seen_loops.end(), loop) != seen_loops.end()) continue;
        seen_loops.push_back(loop);

        // Gathered by walking `next` rather than by filtering on the loop id,
        // because the RING ORDER is what the area and the ridge arcs need.
        std::vector<uint32_t> members;
        uint32_t v = i;
        for (size_t guard = 0; guard <= verts_.size(); ++guard) {
            members.push_back(v);
            v = verts_[v].next;
            if (v == i || v == kNone) break;
        }
        const size_t count = members.size();
        if (count < min_members || count == 0) continue;

        double twice_area = 0.0;
        for (size_t k = 0; k < count; ++k) {
            const glm::dvec2 a = pos(members[k], t);
            const glm::dvec2 b = pos(members[(k + 1) % count], t);
            twice_area += a.x * b.y - b.x * a.y;
        }
        if (0.5 * twice_area > kAreaEpsilon) continue;

        // Positions that coincide are ONE skeleton point, however far apart in the
        // ring they sit. Grouping only consecutive members would emit two nodes at
        // the same place and leave a zero-width face between them.
        std::vector<glm::dvec2> group_pos;
        std::vector<uint32_t> group_of(count, 0);
        for (size_t k = 0; k < count; ++k) {
            const glm::dvec2 p = pos(members[k], t);
            uint32_t found = kNone;
            for (uint32_t g = 0; g < group_pos.size(); ++g) {
                if (length2(group_pos[g] - p) <= eps_ * eps_) {
                    found = g;
                    break;
                }
            }
            if (found == kNone) {
                found = static_cast<uint32_t>(group_pos.size());
                group_pos.push_back(p);
            }
            group_of[k] = found;
        }

        std::vector<uint32_t> group_node(group_pos.size(), kInvalidSkeletonNode);
        for (uint32_t g = 0; g < group_pos.size(); ++g) {
            // Reuse the node a member was born at when it has not moved since. The
            // alternative adds a second node at the same coordinates, which is not
            // wrong but does put a duplicate point into every face that touches it.
            for (size_t k = 0; k < count; ++k) {
                if (group_of[k] != g) continue;
                const uint32_t born = verts_[members[k]].node;
                if (born != kInvalidSkeletonNode &&
                    length2(out_.nodes[born].position - group_pos[g]) <= eps_ * eps_) {
                    group_node[g] = born;
                    break;
                }
            }
            if (group_node[g] == kInvalidSkeletonNode) {
                group_node[g] = add_node(group_pos[g], t, false);
            }
        }

        for (size_t k = 0; k < count; ++k) {
            const Wv& m = verts_[members[k]];
            add_arc(m.node, group_node[group_of[k]], m.left_inst, m.right_inst);
        }

        // The ridge. A flat loop walks each ridge segment twice, once in each
        // direction, so the second pass fills in the contour edge on the other
        // side instead of emitting the segment again.
        std::vector<std::pair<uint32_t, uint32_t>> ridge_keys;
        std::vector<size_t> ridge_arcs;
        for (size_t k = 0; k < count; ++k) {
            const uint32_t from = group_node[group_of[k]];
            const uint32_t to = group_node[group_of[(k + 1) % count]];
            if (from == to) continue;
            const std::pair<uint32_t, uint32_t> key{std::min(from, to), std::max(from, to)};
            const uint32_t inst = verts_[members[k]].right_inst;
            bool repeat = false;
            for (size_t r = 0; r < ridge_keys.size(); ++r) {
                if (ridge_keys[r] != key) continue;
                repeat = true;
                if (inst != kNone) out_.arcs[ridge_arcs[r]].right_edge = insts_[inst].orig;
                break;
            }
            if (repeat) continue;
            const size_t before = out_.arcs.size();
            add_arc(from, to, inst, kNone);
            if (out_.arcs.size() > before) {
                ridge_keys.push_back(key);
                ridge_arcs.push_back(before);
            }
        }

        // Every instance of the loop closes here, but -- unlike close_inst() -- its
        // two chains end at DIFFERENT nodes, because a flat loop has no apex.
        for (size_t k = 0; k < count; ++k) {
            const uint32_t inst = verts_[members[k]].right_inst;
            if (inst == kNone || !insts_[inst].alive) continue;
            insts_[inst].start_chain.push_back(group_node[group_of[k]]);
            insts_[inst].end_chain.push_back(group_node[group_of[(k + 1) % count]]);
            insts_[inst].alive = false;
            insts_[inst].start_v = kNone;
            insts_[inst].end_v = kNone;
        }
        for (const uint32_t m : members) verts_[m].alive = false;

        ++out_.stats.degenerate_loops;
        ++resolved;
    }

    return resolved;
}

Event Sim::find_event(double t) const {
    Event best;

    for (uint32_t vi = 0; vi < verts_.size(); ++vi) {
        const Wv& v = verts_[vi];
        if (!v.alive || !v.finite) continue;

        // --- Edge event: does v's outgoing edge shrink to nothing? ---------
        // Iterating over vertices and taking each one's RIGHT instance visits
        // every live instance exactly once, because start vertex and instance are
        // in bijection. Iterating instances instead would need the same lookup.
        const uint32_t bi = v.next;
        if (bi != kNone && bi != vi && verts_[bi].alive && verts_[bi].finite) {
            const Inst& e = insts_[v.right_inst];
            // Measured ALONG the edge direction, which is constant. The obvious
            // alternative -- the distance between the two endpoints -- is always
            // positive and so cannot tell an edge that is still shrinking from one
            // that has already passed through zero and inverted.
            const double gap = glm::dot(pos(bi, t) - pos(vi, t), e.dir);
            const double rate = glm::dot(verts_[bi].vel - v.vel, e.dir);
            if (rate < -kRateEpsilon) {
                const double dt = std::max(0.0, gap) / (-rate);
                Event candidate;
                candidate.time = t + dt;
                candidate.kind = 0;
                candidate.a = vi;
                candidate.b = bi;
                // The midpoint rather than either endpoint: the two agree to
                // rounding, and averaging them keeps the node from being biased
                // toward whichever vertex the loop happened to evaluate first.
                candidate.point = 0.5 * (pos(vi, candidate.time) + pos(bi, candidate.time));
                if (better_event(candidate, best)) best = candidate;
            }
        }

        // --- Split event: does this reflex vertex reach a far edge? --------
        if (!v.reflex) continue;
        for (uint32_t ei = 0; ei < insts_.size(); ++ei) {
            const Inst& e = insts_[ei];
            if (!e.alive) continue;
            if (ei == v.left_inst || ei == v.right_inst) continue;
            if (e.start_v == kNone || e.end_v == kNone) continue;
            // Only within the same loop. Two loops are the boundaries of disjoint
            // shrinking regions, so one can never reach the other; allowing it
            // would invent events across a gap that is not there.
            if (verts_[e.start_v].loop != v.loop) continue;

            // Signed distance from the vertex to the edge's MOVING line. Negative
            // means the vertex is already past it, which is a wavefront that has
            // folded and is not something to schedule more events against.
            const double gap = glm::dot(pos(vi, t), e.normal) - (e.offset0 + t);
            if (gap < -eps_) continue;
            // The line advances at one per unit time, so the gap closes at
            // 1 - dot(vel, normal). A vertex that is not gaining on the line has
            // no split event against it, whatever it does later.
            //
            // This test is also what makes ZERO gap safe to accept above, and the
            // zero case is not a curiosity: a T-shaped block whose two reflex
            // vertices split the same kerb at the same instant only gets its first
            // split, and the second reflex vertex is then sitting exactly on the
            // line with a gap of zero. Requiring a strictly positive gap dropped
            // that split and folded the wavefront. A vertex that is on the line
            // because it is an ENDPOINT of that edge cannot slip through: being
            // adjacent to any instance of the edge makes dot(vel, normal) exactly
            // one, so its rate is exactly zero and it is refused here.
            const double rate = glm::dot(v.vel, e.normal) - 1.0;
            if (rate >= -kRateEpsilon) continue;

            const double dt = std::max(0.0, gap) / (-rate);
            const double hit_time = t + dt;
            const glm::dvec2 hit = pos(vi, hit_time);

            // The vertex must land ON the live stretch of the edge, not on its
            // supporting line somewhere past the end. Without this test a reflex
            // vertex splits an edge it never touches, and the two loops that come
            // out overlap. The span is measured against the edge's own endpoints
            // AT THE HIT TIME, which is exactly the bisector wedge the standard
            // formulation tests against.
            const double span = glm::dot(pos(e.end_v, hit_time) - pos(e.start_v, hit_time), e.dir);
            if (span < -eps_) continue;
            const double along = glm::dot(hit - pos(e.start_v, hit_time), e.dir);
            if (along < -eps_ || along > span + eps_) continue;

            Event candidate;
            candidate.time = hit_time;
            candidate.kind = 1;
            candidate.a = vi;
            candidate.b = ei;
            candidate.point = hit;
            if (better_event(candidate, best)) best = candidate;
        }
    }

    return best;
}

void Sim::apply_edge_event(const Event& ev) {
    const uint32_t ai = ev.a;
    const uint32_t bi = ev.b;
    const uint32_t inst = verts_[ai].right_inst;
    const uint32_t loop = verts_[ai].loop;

    std::vector<uint32_t> members;
    for (uint32_t j = 0; j < verts_.size(); ++j) {
        if (verts_[j].alive && verts_[j].loop == loop) members.push_back(j);
    }

    const uint32_t node = add_node(ev.point, ev.time, false);
    ++out_.stats.edge_events;

    if (members.size() <= 3) {
        // A three-sided wavefront always collapses to ONE point -- the incentre of
        // its three lines -- and all three of its edge events are simultaneous.
        // Processing them one at a time would create two extra nodes a rounding
        // error apart and leave a sliver face between them, so the whole loop goes
        // at once.
        std::vector<uint32_t> touched;
        for (const uint32_t m : members) {
            add_arc(verts_[m].node, node, verts_[m].left_inst, verts_[m].right_inst);
            touched.push_back(verts_[m].left_inst);
            touched.push_back(verts_[m].right_inst);
            verts_[m].alive = false;
        }
        for (const uint32_t e : touched) close_inst(e, node);
        return;
    }

    add_arc(verts_[ai].node, node, verts_[ai].left_inst, verts_[ai].right_inst);
    add_arc(verts_[bi].node, node, verts_[bi].left_inst, verts_[bi].right_inst);

    const uint32_t left = verts_[ai].left_inst;
    const uint32_t right = verts_[bi].right_inst;
    const uint32_t prev = verts_[ai].prev;
    const uint32_t next = verts_[bi].next;

    close_inst(inst, node);

    const uint32_t fresh = make_vertex(left, right, ev.point, ev.time, node, loop);
    verts_[fresh].prev = prev;
    verts_[fresh].next = next;
    verts_[prev].next = fresh;
    verts_[next].prev = fresh;

    // The two surviving instances each lose one endpoint vertex and gain the new
    // one; their face chains record the handover.
    insts_[left].end_chain.push_back(node);
    insts_[left].end_v = fresh;
    insts_[right].start_chain.push_back(node);
    insts_[right].start_v = fresh;

    verts_[ai].alive = false;
    verts_[bi].alive = false;
}

void Sim::apply_split_event(const Event& ev) {
    const uint32_t vi = ev.a;
    const uint32_t hit_inst = ev.b;

    const uint32_t node = add_node(ev.point, ev.time, false);
    ++out_.stats.split_events;

    add_arc(verts_[vi].node, node, verts_[vi].left_inst, verts_[vi].right_inst);

    const uint32_t left = verts_[vi].left_inst;
    const uint32_t right = verts_[vi].right_inst;
    const uint32_t prev = verts_[vi].prev;
    const uint32_t next = verts_[vi].next;
    const uint32_t w = insts_[hit_inst].start_v;
    const uint32_t wn = insts_[hit_inst].end_v;
    const uint32_t loop = verts_[vi].loop;

    // The hit instance becomes the near half (w .. hit point). The far half
    // (hit point .. wn) is a NEW instance of the same contour edge, spliced in
    // after it so that walking `next_inst` still visits the edge in order.
    Inst far;
    far.orig = insts_[hit_inst].orig;
    far.dir = insts_[hit_inst].dir;
    far.normal = insts_[hit_inst].normal;
    far.offset0 = insts_[hit_inst].offset0;
    far.end_chain = std::move(insts_[hit_inst].end_chain);
    far.start_chain.push_back(node);
    far.next_inst = insts_[hit_inst].next_inst;
    far.end_v = wn;
    insts_.push_back(std::move(far));
    const uint32_t far_inst = static_cast<uint32_t>(insts_.size() - 1);

    insts_[hit_inst].end_chain.clear();
    insts_[hit_inst].end_chain.push_back(node);
    insts_[hit_inst].next_inst = far_inst;

    // Reconnect the wavefront. Going the way the boundary runs, the chain
    // ...prev -> v -> next... and the chain ...w -> wn... become
    // ...prev -> near_side -> wn... and ...w -> far_side -> next...
    const uint32_t far_side = make_vertex(left, far_inst, ev.point, ev.time, node, loop);
    const uint32_t near_side = make_vertex(hit_inst, right, ev.point, ev.time, node, loop);

    verts_[far_side].prev = prev;
    verts_[far_side].next = wn;
    verts_[near_side].prev = w;
    verts_[near_side].next = next;
    verts_[prev].next = far_side;
    verts_[wn].prev = far_side;
    verts_[w].next = near_side;
    verts_[next].prev = near_side;

    insts_[left].end_chain.push_back(node);
    insts_[left].end_v = far_side;
    insts_[right].start_chain.push_back(node);
    insts_[right].start_v = near_side;
    insts_[far_inst].start_v = far_side;
    insts_[hit_inst].end_v = near_side;
    // `wn` now arrives along the FAR half, not along the instance that was split.
    // Forgetting this is silent: the wavefront still marches correctly, but every
    // later event at `wn` files its face chain under the near half, and the split
    // edge's face comes out as a self-crossing ring whose area is wrong. The
    // area-conservation test over the faces is what catches it.
    verts_[wn].left_inst = far_inst;

    verts_[vi].alive = false;

    // One of the two loops keeps the old id; the other needs a fresh one, or the
    // same-loop test in find_event() would let a vertex in one half split an edge
    // in the other half, which is a hole punched through land that is no longer
    // connected to it.
    const uint32_t new_loop = next_loop_++;
    relabel_loop(near_side, new_loop);
}

void Sim::build_faces() {
    const uint32_t n = static_cast<uint32_t>(out_.contour.size());
    out_.faces.reserve(n);

    for (uint32_t e = 0; e < n; ++e) {
        std::vector<uint32_t> chain;
        for (uint32_t j = e; j != kNone; j = insts_[j].next_inst) {
            chain.push_back(j);
            if (chain.size() > insts_.size()) break;  // corrupt link; bail below
        }

        bool usable = !chain.empty() && chain.size() <= insts_.size();
        for (const uint32_t j : chain) {
            if (insts_[j].alive || insts_[j].start_chain.size() < 2 ||
                insts_[j].end_chain.size() < 2) {
                usable = false;
            }
        }
        if (!usable) {
            ++out_.stats.degenerate_faces;
            continue;
        }

        // Walk the face boundary anticlockwise: along the contour edge, then in
        // along the LAST stretch's end chain to its apex, back out along its start
        // chain to the point where the previous stretch was split off, in along
        // that one's end chain, and so on back to the contour edge's own start.
        std::vector<uint32_t> ids;
        ids.push_back(insts_[chain.front()].start_chain.front());
        ids.push_back(insts_[chain.back()].end_chain.front());
        for (size_t idx = chain.size(); idx-- > 0;) {
            const Inst& piece = insts_[chain[idx]];
            for (size_t k = 1; k < piece.end_chain.size(); ++k) {
                ids.push_back(piece.end_chain[k]);
            }
            // Then back out along the start chain. The whole chain is walked,
            // apex included, and the duplicate apex is dropped by the positional
            // de-duplication below rather than by skipping an index: a stretch that
            // was resolved as part of a FLATTENED wavefront ends its two chains at
            // two different nodes, and skipping the last one there would silently
            // drop a real face vertex.
            //
            // For the first stretch, stop above index 0: that node is the contour
            // edge's own start and is already the ring's first point.
            const size_t stop = (idx == 0) ? 1 : 0;
            for (size_t k = piece.start_chain.size(); k > stop;) {
                --k;
                ids.push_back(piece.start_chain[k]);
            }
        }

        SkeletonFace face;
        face.edge = e;
        face.ring.reserve(ids.size());
        face.times.reserve(ids.size());
        for (const uint32_t id : ids) {
            const glm::dvec2& p = out_.nodes[id].position;
            if (!face.ring.empty() && length2(p - face.ring.back()) <= eps_ * eps_) continue;
            face.ring.push_back(p);
            face.times.push_back(out_.nodes[id].time);
        }
        while (face.ring.size() >= 2 &&
               length2(face.ring.front() - face.ring.back()) <= eps_ * eps_) {
            face.ring.pop_back();
            face.times.pop_back();
        }

        face.area = signed_ring_area(face.ring);
        if (face.ring.size() < 3 || face.area <= kAreaEpsilon) {
            ++out_.stats.degenerate_faces;
            continue;
        }
        out_.faces.push_back(std::move(face));
    }
}

StraightSkeleton Sim::run(size_t max_iterations) {
    seed();

    double t = 0.0;
    bool finished = false;
    for (size_t step = 0; step <= max_iterations; ++step) {
        collapse_small_loops(t);
        // Four or more, so the ordinary three-vertex finish keeps its exact path.
        resolve_flat_loops(t, 4);

        bool any_alive = false;
        for (const Wv& v : verts_) {
            if (v.alive) {
                any_alive = true;
                break;
            }
        }
        if (!any_alive) {
            finished = true;
            break;
        }
        if (step == max_iterations) break;

        Event ev = find_event(t);
        if (ev.kind < 0) {
            // Nothing left to schedule while vertices are still alive means the
            // wavefront degenerated in a shape the four-member sweep above did not
            // cover. One last unconditional pass, and only then give up -- giving
            // up throws away every face, so it is worth one more try.
            if (resolve_flat_loops(t, 1) > 0) continue;
            break;
        }

        ++out_.stats.iterations;
        // Never step backwards. An event time marginally in the past is a rounding
        // artefact of a nearly-closed gap, and honouring it would let the same
        // event fire forever.
        t = std::max(t, ev.time);
        if (ev.kind == 0) {
            apply_edge_event(ev);
        } else {
            apply_split_event(ev);
        }
    }

    out_.complete = finished;
    if (finished) build_faces();
    return std::move(out_);
}

} // namespace

// ============================================================================
// Public Interface
// ============================================================================

double signed_ring_area(const std::vector<glm::dvec2>& ring) {
    if (ring.size() < 3) return 0.0;
    double twice = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % ring.size()];
        twice += a.x * b.y - b.x * a.y;
    }
    return 0.5 * twice;
}

bool vertex_offset_velocity(const glm::dvec2& prev, const glm::dvec2& vertex,
                            const glm::dvec2& next, glm::dvec2& out_velocity) {
    const glm::dvec2 in = vertex - prev;
    const glm::dvec2 out = next - vertex;
    const double in_len = std::sqrt(length2(in));
    const double out_len = std::sqrt(length2(out));
    // A zero-length edge has no direction, so it has no normal, so there is no
    // system to solve. Returning false rather than normalising a zero vector is
    // the whole reason this returns a bool.
    if (!(in_len > 0.0) || !(out_len > 0.0)) return false;
    return solve_velocity(perp(in / in_len), perp(out / out_len), out_velocity);
}

StraightSkeleton compute_straight_skeleton(const std::vector<glm::dvec2>& ring,
                                           const SkeletonConfig& config) {
    StraightSkeleton refused;
    const double eps = config.point_epsilon > 0.0 ? config.point_epsilon : 1e-6;
    const size_t max_vertices = config.max_vertices > 0 ? config.max_vertices : 4096;

    if (ring.size() < 3) return refused;

    // Duplicate points would give a zero-length edge, an undefined normal and a
    // NaN velocity. They are removed here rather than refused because a ring that
    // came through a coordinate transform picks them up honestly -- but the ring
    // that comes BACK, in StraightSkeleton::contour, is the one the face indices
    // refer to, so a caller must read that and not the one it passed in.
    std::vector<glm::dvec2> contour;
    contour.reserve(ring.size());
    for (const glm::dvec2& p : ring) {
        if (!contour.empty() && length2(p - contour.back()) <= eps * eps) continue;
        contour.push_back(p);
    }
    while (contour.size() >= 2 && length2(contour.front() - contour.back()) <= eps * eps) {
        contour.pop_back();
    }
    if (contour.size() < 3 || contour.size() > max_vertices) {
        refused.contour = std::move(contour);
        return refused;
    }

    const double area = signed_ring_area(contour);
    if (std::fabs(area) <= kAreaEpsilon) {
        refused.contour = std::move(contour);
        return refused;
    }
    // Reversed rather than refused: the inward normal convention is what the sign
    // of every event time depends on, so the ring has to be anticlockwise, but a
    // caller holding a clockwise ring is not making a mistake -- Block::holes are
    // clockwise by design. The ring that comes back is the reversed one.
    if (area < 0.0) std::reverse(contour.begin(), contour.end());

    const size_t n = contour.size();
    size_t crossings = 0;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) continue;  // adjacent across the seam
            if (segments_properly_cross(contour[i], contour[(i + 1) % n], contour[j],
                                        contour[(j + 1) % n])) {
                ++crossings;
            }
        }
    }
    if (crossings > 0) {
        // A self-intersecting ring has no wavefront: the "interior" it bounds is
        // not a region, so every event time computed against it is meaningless.
        // Refusing keeps the iteration cap from being the thing that stops it, and
        // keeps the caller from shipping a folded roof.
        refused.contour = std::move(contour);
        refused.stats.self_intersections = crossings;
        return refused;
    }

    const size_t max_iterations =
        config.max_iterations > 0 ? config.max_iterations : (8 * n + 64);

    Sim sim(contour, config);
    return sim.run(max_iterations);
}

} // namespace stratum::geometry
