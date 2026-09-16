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
 * See blocks.hpp for the traversal rule, the orientation contract, and the reason
 * the outer face is rejected by sign.
 */

#include "osm/road/blocks.hpp"

#include <spdlog/spdlog.h>

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
 * keeping. This is a judgement about what is REAL: the face of a component that
 * is a tree walks out and back over every edge and encloses exactly nothing, and
 * its shoelace sum lands on either side of zero depending on rounding. Without
 * this band such a face is classified by the sign of its rounding error, and half
 * of them get counted as outer faces.
 */
constexpr double kAreaEpsilon = 1e-9;

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
};

/**
 * @brief Remove consecutive duplicates and zero-width antennae from a ring
 *
 * A cul-de-sac inside a face is walked out and straight back, so the ring reads
 * `... x, tip, x ...`. That contributes nothing to the signed area, but it is not
 * a polygon: offsetting it produces garbage and triangulating it produces
 * degenerate triangles, so it cannot be handed to C2 as the shape of a parcel of
 * land. Only the ring is pruned; the spur's half-edges stay in Block::edges,
 * because the cul-de-sac really does front this block.
 *
 * Cancellation is a single stack pass. Removing a tip can expose the tip behind
 * it -- a branching dead-end tree collapses one leaf at a time -- and the stack
 * handles that without re-scanning, since the exposed tip is compared against the
 * very next incoming point.
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

        if (out.size() >= 2 && same_point(out[out.size() - 2].point, v.point)) {
            // We walked out to out.back() and came straight back here. Drop the
            // tip; the point we have re-entered now leaves by this occurrence's
            // edge, so it takes this owner.
            out.pop_back();
            out.back().owner = v.owner;
            continue;
        }

        out.push_back(v);
    }

    // The ring is cyclic, so an antenna sitting across the seam -- the walk
    // started halfway along a spur -- survives the linear pass. After it, only the
    // first and last vertices can still be tips, and removing one can expose
    // another, so this loops rather than testing once.
    //
    // Every case is a plain removal: the owner of a surviving vertex is the owner
    // of the segment leaving it, and none of these removals changes which vertex
    // that segment ends at.
    while (out.size() >= 3) {
        if (same_point(out.front().point, out.back().point)) {
            out.pop_back();     // last point duplicates the first across the seam
            continue;
        }
        if (same_point(out[out.size() - 2].point, out.front().point)) {
            out.pop_back();     // last point is a tip hanging off the first
            continue;
        }
        if (same_point(out.back().point, out[1].point)) {
            out.erase(out.begin());     // first point is a tip hanging off the second
            continue;
        }
        break;
    }

    ring.swap(out);
}

/// Length of a closed ring in metres, first point not repeated
double ring_perimeter(const std::vector<glm::dvec2>& ring) {
    if (ring.size() < 2) {
        return 0.0;
    }

    double total = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2 d = ring[(i + 1) % ring.size()] - ring[i];
        total += std::sqrt(d.x * d.x + d.y * d.y);
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
        ++result.stats.edges_used;
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
    // Pass 2: one orbit per unvisited half-edge
    //
    // Half-edge h is edge h/2, travelled from `from` to `to` when h is even. The
    // seed order is therefore edge order, which makes the block ids deterministic
    // for a given graph -- a dump diffed against a previous run has to be stable
    // or it is no use as a regression test.
    // ------------------------------------------------------------------------
    const size_t half_edge_count = edges.size() * 2;
    std::vector<uint8_t> visited(half_edge_count, 0);

    std::vector<RingVertex> raw;    // reused across faces
    std::vector<BlockEdge> walked;

    for (size_t seed = 0; seed < half_edge_count; ++seed) {
        if (usable[seed / 2] == 0 || visited[seed] != 0) {
            continue;
        }

        raw.clear();
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
            grade_separated = grade_separated || is_grade_separated(edge);

            // Contribute every polyline vertex EXCEPT the last. The last is the
            // node this half-edge arrives at, and the next half-edge contributes it
            // as its own first point. That is what leaves the finished ring with
            // its first point not repeated, and it is why no seam dedup is needed.
            if (forward) {
                for (size_t i = 0; i + 1 < edge.polyline.size(); ++i) {
                    raw.push_back(RingVertex{edge.polyline[i], owner});
                }
            } else {
                for (size_t i = edge.polyline.size(); i-- > 1;) {
                    raw.push_back(RingVertex{edge.polyline[i], owner});
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

            // Leave by the next arm CLOCKWISE. GraphNode::arms ascends by bearing,
            // which runs anticlockwise, so clockwise is backwards through the list.
            // Unusable arms are stepped over, which is how excluding footways merges
            // the faces either side of one instead of leaving a gap in the boundary.
            //
            // k runs to arm_count inclusive so the last candidate is back_arm
            // itself: at a dead end that is the only arm there is, and leaving by
            // the arm we arrived on is exactly the walk out and back that a
            // cul-de-sac needs.
            size_t leave = kNoArm;
            for (size_t k = 1; k <= arm_count; ++k) {
                const size_t idx = (back_arm + arm_count - k) % arm_count;
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

        ++result.stats.faces;
        if (malformed) {
            ++result.stats.degenerate_faces;
            continue;
        }

        prune_antennae(raw);
        if (raw.size() < 3) {
            // A component that is a tree: every edge walked out and back, nothing
            // enclosed. One face, no block. This is also the single face an
            // isolated dead-end road produces.
            ++result.stats.degenerate_faces;
            continue;
        }

        std::vector<glm::dvec2> ring;
        std::vector<uint32_t> ring_edges;
        ring.reserve(raw.size());
        ring_edges.reserve(raw.size());
        for (const RingVertex& vertex : raw) {
            ring.push_back(vertex.point);
            ring_edges.push_back(vertex.owner);
        }

        const double area = signed_ring_area(ring);

        // The outer face is rejected on the SIGN, never on the size. One outer face
        // exists per connected component, and with two components in an extract the
        // smaller one's outer face is smaller than the larger one's blocks -- so
        // "drop the largest face" drops a real block and keeps an outer face.
        if (area <= -kAreaEpsilon) {
            ++result.stats.outer_faces;
            continue;
        }
        if (area < kAreaEpsilon) {
            ++result.stats.degenerate_faces;
            continue;
        }
        if (area < config.min_area) {
            ++result.stats.rejected_small;
            continue;
        }

        Block block;
        block.id = static_cast<BlockId>(result.blocks.size());
        block.ring = std::move(ring);
        block.ring_edges = std::move(ring_edges);
        // Copied, not moved: `walked` is reused across faces and a move would
        // hand its buffer away, so every face after the first would reallocate.
        block.edges = walked;
        block.area = area;
        block.perimeter = ring_perimeter(block.ring);
        block.has_grade_separated_edge = grade_separated;
        result.blocks.push_back(std::move(block));
    }

    result.stats.blocks = result.blocks.size();

    spdlog::info("Blocks: {} blocks from {} faces — {} outer, {} degenerate, "
                 "{} below {:.0f} m², {} of {} edges used",
                 result.stats.blocks, result.stats.faces, result.stats.outer_faces,
                 result.stats.degenerate_faces, result.stats.rejected_small,
                 config.min_area, result.stats.edges_used, result.stats.edges_considered);

    return result;
}

} // namespace stratum::osm::road
