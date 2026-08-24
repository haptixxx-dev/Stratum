/**
 * @file road_graph.cpp
 * @brief Implementation of the OSM-node-identity road network graph
 *
 * Build is three linear passes over the road polylines:
 *   1. Reference count every NodeId, and record every (NodeId, layer) slot.
 *   2. Split every road at its graph nodes, creating nodes on first use.
 *   3. Register and sort the arms incident on each node.
 *
 * Qualification and identity are keyed differently on purpose. A node qualifies
 * on its OSM id alone, counted across every road whatever its layer, so a
 * crossing shared by two ways is always seen. Identity is then keyed by
 * (NodeId, layer) wherever some road passes through the node, so that crossing
 * yields one graph node per layer and a bridge never merges with the road it
 * passes over. Where every road merely ends on the node -- a bridge abutment --
 * identity ignores layer, so the deck stays connected to its approaches.
 *
 * Folding layer into the qualification test instead drops the crossing node
 * altogether: each layer on its own references it exactly once and never reaches
 * the two-reference threshold.
 */

#include "osm/road/road_graph.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Node Keys
// ============================================================================

/**
 * @brief Identity of a graph node before it is created: an OSM node on a layer
 *
 * Layer is part of the key on purpose. Two roads on different layers that share
 * an OSM node id are not a junction, so they must not collapse into one entry.
 */
struct NodeKey {
    NodeId id = 0;

    /// Layer the slot is keyed on, from NodeRefs::slot_layer(), not Road::layer
    int layer = 0;

    bool operator==(const NodeKey& other) const noexcept {
        return id == other.id && layer == other.layer;
    }
};

/// Finalizer of the SplitMix64 generator; a full 64-bit avalanche mix
inline uint64_t mix64(uint64_t x) noexcept {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

/**
 * @brief Hash for NodeKey
 *
 * Both fields are mixed independently and then combined, rather than packed into
 * one integer. Packing a 64-bit node id next to a layer would have to discard id
 * bits, and OSM node ids already run past 2^32.
 */
struct NodeKeyHash {
    size_t operator()(const NodeKey& key) const noexcept {
        const uint64_t a = mix64(static_cast<uint64_t>(key.id));
        const uint64_t b = mix64(static_cast<uint64_t>(static_cast<int64_t>(key.layer)));
        return static_cast<size_t>(a ^ (b + 0x9E3779B97F4A7C15ull + (a << 6) + (a >> 2)));
    }
};

/**
 * @brief What the reference-counting pass learns about one OSM node
 *
 * Counted across every road with no regard for layer. A crossing between a
 * bridge and the road beneath it is referenced once per layer, so a per-layer
 * count would never reach two and the crossing would vanish from the graph.
 */
struct NodeRefs {
    /// Times any road polyline visits this node. A closed loop counts it twice.
    uint32_t refs = 0;

    /// True when the node is the first or last point of some road
    bool endpoint = false;

    /**
     * @brief True when some road passes through this node rather than ending on it
     *
     * This is what separates a grade separation from a bridge abutment, and it is
     * the condition on the layer split. A road crossing over another shares a node
     * that is interior to at least one of them, so the layers must not merge. A
     * bridge deck meeting its approach shares a node that is an endpoint of both
     * ways, and the differing layer=* there is correct data describing one
     * continuous road, so those must merge. Keying every differing layer apart
     * unconditionally disconnects every bridge in the extract from its approaches.
     */
    bool interior = false;

    /// A node becomes a graph node when roads meet at it or a road ends on it
    [[nodiscard]] bool qualifies() const { return refs >= 2 || endpoint; }

    /**
     * @brief Layer this node's slot is keyed on
     *
     * The node's own layer at a grade separation, so each layer gets its own graph
     * node; a fixed 0 where every road merely ends on the node, so the arms of all
     * layers resolve to one graph node.
     *
     * @param layer Layer of the road visiting the node
     */
    [[nodiscard]] int slot_layer(int layer) const { return interior ? layer : 0; }
};

/// Reference counts by OSM node id: the input to the qualification test
using RefMap = std::unordered_map<NodeId, NodeRefs>;

/**
 * @brief Dense index of one (NodeId, layer) slot
 *
 * Pass 1b records every slot any road visits and hands each a dense index, so
 * later passes address slots by index instead of re-hashing a key.
 */
using SlotMap = std::unordered_map<NodeKey, uint32_t, NodeKeyHash>;

/**
 * @brief Disjoint set over slot indices, so coincident slots share a graph node
 *
 * Two slots are unioned when the stretch of road between them collapses to a
 * single point, which happens where a way repeats a coordinate under two node
 * ids. Without the union each side of the collapse would resolve to its own
 * graph node at the identical position and the road would be severed there.
 *
 * The union runs before any GraphNode exists, so no edge ever has to be
 * repointed afterwards and the result does not depend on road ordering.
 */
class SlotUnion {
public:
    /// Start with @p count singleton slots
    explicit SlotUnion(size_t count) : m_parent(count) {
        for (uint32_t i = 0; i < m_parent.size(); ++i) m_parent[i] = i;
    }

    /// Representative slot of @p slot, with path compression
    [[nodiscard]] uint32_t find(uint32_t slot) {
        while (m_parent[slot] != slot) {
            m_parent[slot] = m_parent[m_parent[slot]];
            slot = m_parent[slot];
        }
        return slot;
    }

    /// Merge the sets of @p a and @p b, keeping the lower representative
    void unite(uint32_t a, uint32_t b) {
        const uint32_t ra = find(a);
        const uint32_t rb = find(b);
        if (ra == rb) return;
        m_parent[std::max(ra, rb)] = std::min(ra, rb);
    }

private:
    std::vector<uint32_t> m_parent;
};

// ============================================================================
// Geometry Helpers
// ============================================================================

/// Points closer than this in local metres are treated as the same point
constexpr double kPointEpsilon = 1e-6;
constexpr double kPointEpsilonSq = kPointEpsilon * kPointEpsilon;

/// True when two local-metre points coincide within kPointEpsilon
inline bool same_point(const glm::dvec2& a, const glm::dvec2& b) noexcept {
    const glm::dvec2 d = a - b;
    return (d.x * d.x + d.y * d.y) <= kPointEpsilonSq;
}

/**
 * @brief True when pts[first..last] would dedup down to a single point
 *
 * Mirrors the consecutive-duplicate filter pass 2 applies while building an edge
 * polyline: every point of the stretch is compared against the last one kept,
 * which stays pts[first] for as long as the stretch is collapsing. The stretch is
 * therefore degenerate exactly when every point of it coincides with pts[first].
 *
 * @param pts   Road polyline
 * @param first Index of the first point of the stretch
 * @param last  Index of the last point of the stretch, not less than first
 */
bool stretch_collapses(const std::vector<glm::dvec2>& pts, size_t first, size_t last) {
    for (size_t i = first + 1; i <= last; ++i) {
        if (!same_point(pts[i], pts[first])) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Direction leaving one end of a polyline, skipping zero-length segments
 *
 * A polyline may still carry a repeated vertex that survived the split-time
 * dedup (points that differ by more than kPointEpsilon but are effectively
 * coincident are not removed), so the scan walks inward until it finds a point
 * that is genuinely elsewhere.
 *
 * @param pts Edge polyline
 * @param at_start True for the direction leaving pts.front(), false for pts.back()
 * @return Unnormalised direction, or (0, 0) when every point coincides
 */
glm::dvec2 leaving_direction(const std::vector<glm::dvec2>& pts, bool at_start) {
    if (pts.size() < 2) {
        return glm::dvec2(0.0);
    }

    if (at_start) {
        const glm::dvec2& base = pts.front();
        for (size_t i = 1; i < pts.size(); ++i) {
            if (!same_point(pts[i], base)) {
                return pts[i] - base;
            }
        }
    } else {
        const glm::dvec2& base = pts.back();
        for (size_t i = pts.size() - 1; i-- > 0;) {
            if (!same_point(pts[i], base)) {
                return pts[i] - base;
            }
        }
    }

    return glm::dvec2(0.0);
}

/// Bearing of a direction in radians, or 0 for a zero-length direction
inline double bearing_of(const glm::dvec2& dir) {
    if (dir.x == 0.0 && dir.y == 0.0) {
        return 0.0;
    }
    return std::atan2(dir.y, dir.x);
}

/**
 * @brief Read the junction-relevant tags off a raw OSM node
 *
 * A node missing from ParsedOSMData::nodes is not an error. Extracts are often
 * clipped, and the flags simply stay false.
 */
void apply_node_tags(const TagMap& tags, GraphNode& node) {
    const auto it = tags.find("highway");
    if (it == tags.end()) {
        return;
    }

    const std::string& value = it->second;
    if (value == "traffic_signals") {
        node.has_signals = true;
    } else if (value == "crossing") {
        node.has_crossing = true;
    } else if (value == "turning_circle" || value == "turning_loop") {
        node.is_turning_circle = true;
    }
}

} // namespace

// ============================================================================
// GraphEdge
// ============================================================================

double GraphEdge::length() const {
    if (polyline.size() < 2) {
        return 0.0;
    }

    double total = 0.0;
    for (size_t i = 1; i < polyline.size(); ++i) {
        const glm::dvec2 d = polyline[i] - polyline[i - 1];
        total += std::sqrt(d.x * d.x + d.y * d.y);
    }
    return total;
}

// ============================================================================
// Construction
// ============================================================================

void RoadGraph::clear() {
    m_nodes.clear();
    m_edges.clear();
}

void RoadGraph::build(const ParsedOSMData& data) {
    clear();

    // ------------------------------------------------------------------------
    // Pass 0: validate roads and size the key map
    // ------------------------------------------------------------------------
    size_t total_points = 0;
    size_t skipped_roads = 0;
    for (const auto& road : data.roads) {
        if (road.polyline.size() < 2 || road.node_ids.size() != road.polyline.size()) {
            ++skipped_roads;
            continue;
        }
        total_points += road.polyline.size();
    }

    if (total_points == 0) {
        spdlog::info("RoadGraph: Built graph — 0 nodes, 0 edges, 0 junctions, "
                     "0 dead ends, 0 roundabout edges, 0 layer splits");
        if (skipped_roads > 0) {
            spdlog::warn("RoadGraph: Skipped {} of {} roads with no usable topology "
                         "(fewer than 2 points, or node_ids not parallel to polyline)",
                         skipped_roads, data.roads.size());
        }
        return;
    }

    RefMap refs;
    refs.reserve(total_points);

    SlotMap slots;
    slots.reserve(total_points);

    // ------------------------------------------------------------------------
    // Pass 1a: reference count every node
    //
    // A road that visits the same node twice increments it twice. That is what
    // makes a closed loop split into two edges instead of one unusable ring.
    //
    // The count ignores layer on purpose. See NodeRefs.
    // ------------------------------------------------------------------------
    for (const auto& road : data.roads) {
        if (road.polyline.size() < 2 || road.node_ids.size() != road.polyline.size()) {
            continue;
        }

        const size_t last = road.node_ids.size() - 1;
        for (size_t i = 0; i <= last; ++i) {
            NodeRefs& info = refs[road.node_ids[i]];
            ++info.refs;
            if (i == 0 || i == last) {
                info.endpoint = true;
            } else {
                info.interior = true;
            }
        }
    }

    // ------------------------------------------------------------------------
    // Pass 1b: record every slot a road visits, and give each a dense index
    //
    // Separate from the count because NodeRefs::interior decides how the slot is
    // keyed, and a node's interior flag is only complete once every road has been
    // counted. A node one road ends on and another passes through would otherwise
    // be keyed one way by the first road and another by the second.
    // ------------------------------------------------------------------------
    auto slot_key = [&refs](NodeId id, int layer) -> NodeKey {
        const auto ref = refs.find(id);
        return NodeKey{id, ref == refs.end() ? layer : ref->second.slot_layer(layer)};
    };

    for (const auto& road : data.roads) {
        if (road.polyline.size() < 2 || road.node_ids.size() != road.polyline.size()) {
            continue;
        }

        for (const NodeId id : road.node_ids) {
            slots.emplace(slot_key(id, road.layer), static_cast<uint32_t>(slots.size()));
        }
    }

    // ------------------------------------------------------------------------
    // Pass 1c: union the slots that a collapsed stretch of road would coincide
    //
    // A way that repeats a coordinate under two node ids -- a common data defect
    // -- puts two qualifying nodes at the same position with nothing between
    // them. Pass 2 cannot emit that stretch as an edge, because an arm with no
    // direction is useless to the junction solver, so without this union the two
    // sides of the collapse would attach to two distinct graph nodes and the road
    // would be cut in half. Uniting the slots first makes both sides resolve to
    // one node whatever order the roads are visited in.
    // ------------------------------------------------------------------------
    SlotUnion slot_union(slots.size());

    for (const auto& road : data.roads) {
        if (road.polyline.size() < 2 || road.node_ids.size() != road.polyline.size()) {
            continue;
        }

        const size_t count = road.polyline.size();
        size_t previous = count;    // index of the last qualifying node seen

        for (size_t i = 0; i < count; ++i) {
            const auto ref = refs.find(road.node_ids[i]);
            if (ref == refs.end() || !ref->second.qualifies()) {
                continue;
            }

            if (previous != count && stretch_collapses(road.polyline, previous, i)) {
                slot_union.unite(slots.at(slot_key(road.node_ids[previous], road.layer)),
                                 slots.at(slot_key(road.node_ids[i], road.layer)));
            }
            previous = i;
        }
    }

    // Upper bound on the node count: one qualifying OSM node contributes one slot
    // per layer it appears on, which is exactly the layer split, and pass 1c can
    // only fold slots together.
    size_t node_estimate = 0;
    for (const auto& entry : slots) {
        const auto it = refs.find(entry.first.id);
        if (it != refs.end() && it->second.qualifies()) {
            ++node_estimate;
        }
    }
    m_nodes.reserve(node_estimate);
    m_edges.reserve(data.roads.size() + data.roads.size() / 2);

    // Graph node behind each slot's representative, filled on first use
    std::vector<GraphNodeId> slot_nodes(slots.size(), kInvalidId);

    auto node_for = [&](uint32_t slot, NodeId osm_id, int layer,
                        const glm::dvec2& position) -> GraphNodeId {
        const uint32_t root = slot_union.find(slot);
        if (slot_nodes[root] != kInvalidId) {
            return slot_nodes[root];
        }

        slot_nodes[root] = static_cast<GraphNodeId>(m_nodes.size());

        GraphNode& node = m_nodes.emplace_back();
        node.osm_id = osm_id;
        node.position = position;   // local metres from the polyline, never reprojected
        node.layer = layer;

        const auto it = data.nodes.find(osm_id);
        if (it != data.nodes.end()) {
            apply_node_tags(it->second.tags, node);
        }

        return slot_nodes[root];
    };

    // ------------------------------------------------------------------------
    // Pass 2: split every road at its graph nodes
    // ------------------------------------------------------------------------
    std::vector<std::pair<size_t, uint32_t>> splits;  // reused across roads
    size_t degenerate_edges = 0;

    for (const auto& road : data.roads) {
        if (road.polyline.size() < 2 || road.node_ids.size() != road.polyline.size()) {
            continue;
        }

        const size_t count = road.polyline.size();

        // Collect the split points. Index 0 and count-1 always qualify because
        // pass 1 flagged them as endpoints, so there are at least two.
        splits.clear();
        splits.reserve(8);
        for (size_t i = 0; i < count; ++i) {
            const auto ref = refs.find(road.node_ids[i]);
            if (ref == refs.end() || !ref->second.qualifies()) {
                continue;
            }

            const auto slot = slots.find(slot_key(road.node_ids[i], road.layer));
            if (slot == slots.end()) {
                continue;   // cannot happen: pass 1b recorded every visited slot
            }
            splits.emplace_back(i, slot->second);
        }
        if (splits.size() < 2) {
            continue;   // cannot happen while pass 1 flags endpoints; cheap to guard
        }

        for (size_t s = 0; s + 1 < splits.size(); ++s) {
            const size_t begin = splits[s].first;
            const size_t end = splits[s + 1].first;

            m_edges.emplace_back();
            GraphEdge& edge = m_edges.back();
            edge.polyline.reserve(end - begin + 1);
            edge.node_ids.reserve(end - begin + 1);

            for (size_t i = begin; i <= end; ++i) {
                const glm::dvec2& p = road.polyline[i];

                if (!edge.polyline.empty() && same_point(edge.polyline.back(), p)) {
                    // Consecutive duplicate. Drop it, except at the terminal node:
                    // there the graph node's identity has to win, so it overwrites
                    // the interior vertex it sits on top of.
                    if (i == end) {
                        edge.node_ids.back() = road.node_ids[i];
                    }
                    continue;
                }

                edge.polyline.push_back(p);
                edge.node_ids.push_back(road.node_ids[i]);
            }

            if (edge.polyline.size() < 2) {
                // The whole stretch collapsed to a point. Emitting it would give
                // the junction solver an arm with no direction. The road is not
                // severed by the drop: pass 1c already united the two slots, so
                // the stretches either side of this one resolve to the same node.
                m_edges.pop_back();
                ++degenerate_edges;
                continue;
            }

            edge.source_way = road.osm_id;
            edge.type = road.type;
            edge.layer = road.layer;
            edge.width = road.width;
            edge.lanes = road.lanes;
            edge.lanes_forward = road.lanes_forward;
            edge.lanes_backward = road.lanes_backward;
            edge.is_oneway = road.is_oneway;
            edge.is_bridge = road.is_bridge;
            edge.is_tunnel = road.is_tunnel;
            edge.is_roundabout = road.is_roundabout;
            edge.is_link = road.is_link;
            edge.sidewalk = road.sidewalk;
            edge.cycleway = road.cycleway;
            edge.parking = road.parking;
            edge.shoulder = road.shoulder;
            edge.surface = road.surface;
            edge.name = road.name;

            // Nodes are created only once an edge actually needs them, so a road
            // made entirely of degenerate stretches leaves no isolated nodes.
            const GraphNodeId from = node_for(splits[s].second, road.node_ids[begin],
                                              road.layer, road.polyline[begin]);
            const GraphNodeId to = node_for(splits[s + 1].second, road.node_ids[end],
                                            road.layer, road.polyline[end]);

            const EdgeId id = static_cast<EdgeId>(m_edges.size() - 1);
            m_edges[id].from = from;
            m_edges[id].to = to;
        }
    }

    // ------------------------------------------------------------------------
    // Pass 3: register arms, then sort each node's arms by outgoing bearing
    // ------------------------------------------------------------------------
    std::vector<uint32_t> arm_counts(m_nodes.size(), 0);
    for (const auto& edge : m_edges) {
        ++arm_counts[edge.from];
        ++arm_counts[edge.to];
    }
    for (size_t i = 0; i < m_nodes.size(); ++i) {
        m_nodes[i].arms.reserve(arm_counts[i]);
    }

    for (size_t i = 0; i < m_edges.size(); ++i) {
        const GraphEdge& edge = m_edges[i];
        const EdgeId id = static_cast<EdgeId>(i);

        Arm start_arm;
        start_arm.edge = id;
        start_arm.at_start = true;
        start_arm.bearing = bearing_of(leaving_direction(edge.polyline, true));
        m_nodes[edge.from].arms.push_back(start_arm);

        Arm end_arm;
        end_arm.edge = id;
        end_arm.at_start = false;
        end_arm.bearing = bearing_of(leaving_direction(edge.polyline, false));
        m_nodes[edge.to].arms.push_back(end_arm);
    }

    for (auto& node : m_nodes) {
        std::sort(node.arms.begin(), node.arms.end(),
                  [](const Arm& a, const Arm& b) { return a.bearing < b.bearing; });
    }

    const Stats s = stats();
    spdlog::info("RoadGraph: Built graph — {} nodes, {} edges, {} junctions, "
                 "{} dead ends, {} roundabout edges, {} layer splits",
                 s.nodes, s.edges, s.junctions, s.dead_ends, s.roundabout_edges,
                 s.layer_split_nodes);

    if (skipped_roads > 0 || degenerate_edges > 0) {
        spdlog::warn("RoadGraph: Skipped {} of {} roads with no usable topology and "
                     "{} zero-length edges", skipped_roads, data.roads.size(),
                     degenerate_edges);
    }
}

// ============================================================================
// Queries
// ============================================================================

glm::dvec2 RoadGraph::arm_direction(const Arm& arm) const {
    if (arm.edge >= m_edges.size()) {
        return glm::dvec2(0.0);
    }

    const glm::dvec2 dir = leaving_direction(m_edges[arm.edge].polyline, arm.at_start);
    const double len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (len <= kPointEpsilon) {
        return glm::dvec2(0.0);
    }

    return dir / len;
}

RoadGraph::Stats RoadGraph::stats() const {
    Stats s;
    s.nodes = m_nodes.size();
    s.edges = m_edges.size();

    std::unordered_set<NodeId> distinct_osm_ids;
    distinct_osm_ids.reserve(m_nodes.size());

    for (const auto& node : m_nodes) {
        const size_t degree = node.degree();
        if (degree == 1) {
            ++s.dead_ends;
        } else if (degree == 2) {
            ++s.continuations;
        } else if (degree >= 3) {
            ++s.junctions;
        }
        distinct_osm_ids.insert(node.osm_id);
    }

    for (const auto& edge : m_edges) {
        if (edge.is_roundabout) {
            ++s.roundabout_edges;
        }
    }

    // Extra nodes the layer split produced: one OSM node id that appears on two
    // layers contributes one, on three layers contributes two.
    s.layer_split_nodes = m_nodes.size() - distinct_osm_ids.size();

    return s;
}

} // namespace stratum::osm::road
