/**
 * @file road_graph.hpp
 * @brief Road network topology derived from OSM node identity
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The road graph is the foundation of the road network pipeline. Every later
 * stage -- cross-section profiles, corridor extrusion, elevation solving,
 * junction geometry -- reads the graph rather than the raw Road list.
 *
 * The graph is built from shared OSM NodeIds, never from the proximity of way
 * endpoints. Proximity clustering misses the common case of a T-junction whose
 * shared node is interior to the through road, and it invents junctions where a
 * bridge merely passes over another road.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API. It is pure geometry and topology over ParsedOSMData.
 */

#pragma once

#include "osm/types.hpp"

#include <glm/glm.hpp>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Type Aliases
// ============================================================================

/// Index into RoadGraph::edges()
using EdgeId = uint32_t;

/// Index into RoadGraph::nodes()
using GraphNodeId = uint32_t;

/// Sentinel for an unset EdgeId or GraphNodeId
inline constexpr uint32_t kInvalidId = 0xFFFFFFFFu;

// ============================================================================
// Graph Structures
// ============================================================================

/**
 * @brief One edge endpoint incident on a graph node
 *
 * A node's arms are the edges radiating out of it. An edge that begins and ends
 * at the same node (a closed loop, common on roundabouts and cul-de-sac bulbs)
 * contributes two arms to that node, one with at_start true and one false.
 */
struct Arm {
    /// Edge this arm belongs to
    EdgeId edge = kInvalidId;

    /// True when this node is the edge's `from` endpoint, false when it is `to`
    bool at_start = true;

    /**
     * @brief Bearing of the direction leaving the node, in radians
     *
     * atan2 of the first polyline segment travelling away from this node, in
     * local metres, so the range is [-pi, pi]. This is the sort key for
     * GraphNode::arms and the input to the junction solver's arm pairing.
     */
    double bearing = 0.0;
};

/**
 * @brief A junction, dead end, or profile transition point in the network
 *
 * A GraphNode is created for an OSM node that is referenced by two or more roads
 * or that is an endpoint of a road. Interior nodes referenced by exactly one road
 * stay as plain polyline vertices on a GraphEdge and get no GraphNode.
 *
 * Nodes are split per layer at a grade separation: if arms with differing
 * Road::layer meet at an OSM node that some road passes *through*, each layer
 * gets its own GraphNode carrying only the arms of that layer, so a bridge never
 * merges with the road beneath it. Several GraphNodes may therefore share one
 * osm_id.
 *
 * A node every road merely ends on is not split, whatever the layers. That is a
 * bridge abutment, where the differing layer=* describes one continuous road, and
 * splitting it would disconnect every bridge deck from its approaches.
 */
struct GraphNode {
    /// OSM node ID this graph node was built from
    NodeId osm_id = 0;

    /// Position in local metres, taken straight from the road polyline
    glm::dvec2 position{0.0};

    /**
     * @brief OSM layer=* of this node
     *
     * Shared by every arm at a grade separation, since those nodes are split per
     * layer. At a bridge abutment the arms disagree by construction and this
     * carries the layer of whichever arm created the node; each GraphEdge still
     * carries its own layer.
     */
    int layer = 0;

    /// Incident edge endpoints, sorted ascending by Arm::bearing
    std::vector<Arm> arms;

    bool has_signals = false;       ///< highway=traffic_signals on the OSM node
    bool has_crossing = false;      ///< highway=crossing on the OSM node
    bool is_turning_circle = false; ///< highway=turning_circle or highway=turning_loop

    /// Number of incident arms
    [[nodiscard]] size_t degree() const { return arms.size(); }

    /// Degree 3 or more: a real junction that needs trimming and a fillet
    [[nodiscard]] bool is_junction() const { return arms.size() >= 3; }

    /// Degree 1: a dead end that needs a cap, bulb, or turning circle
    [[nodiscard]] bool is_dead_end() const { return arms.size() == 1; }
};

/**
 * @brief A stretch of one OSM way between two graph nodes
 *
 * Ways are split at every graph node, so an edge's interior never contains a
 * junction. All the tag-derived attributes are copied down from the parent Road
 * because every split of a way shares them.
 */
struct GraphEdge {
    /// OSM way this edge was split out of
    WayId source_way = 0;

    GraphNodeId from = kInvalidId;  ///< Node at polyline.front()
    GraphNodeId to = kInvalidId;    ///< Node at polyline.back()

    /**
     * @brief Centerline from `from` to `to` inclusive, in local metres
     *
     * Always at least 2 points. Retains the original OSM vertices; resampling and
     * smoothing happen downstream and do not write back here.
     */
    std::vector<glm::dvec2> polyline;

    /// OSM node IDs parallel to polyline, same size
    std::vector<NodeId> node_ids;

    RoadType type = RoadType::Unknown;  ///< Road classification of the parent way
    int layer = 0;                      ///< OSM layer=*
    float width = 6.0f;                 ///< Carriageway width in metres

    int lanes = 2;                      ///< Total lane count
    int lanes_forward = -1;             ///< lanes:forward=*, -1 when unspecified
    int lanes_backward = -1;            ///< lanes:backward=*, -1 when unspecified

    bool is_oneway = false;             ///< One-way in the direction from -> to
    bool is_bridge = false;             ///< Bridge segment
    bool is_tunnel = false;             ///< Tunnel segment
    bool is_roundabout = false;         ///< Part of a junction=roundabout|circular cycle
    bool is_link = false;               ///< highway=*_link ramp

    SideFlags sidewalk = SideFlags::Unknown;    ///< Sidewalk presence by side
    SideFlags cycleway = SideFlags::Unknown;    ///< Cycle lane presence by side
    SideFlags parking = SideFlags::Unknown;     ///< On-street parking presence by side
    SideFlags shoulder = SideFlags::Unknown;    ///< Shoulder presence by side

    std::string surface;    ///< Raw surface=* value, lowercased. Empty when absent.
    std::string name;       ///< Road name from name=*

    /**
     * @brief Arc length in metres to cut off the `from` end of the ribbon
     *
     * Filled by the junction solver in a later phase so the corridor stops short
     * of the junction polygon instead of overlapping it. Zero until then.
     */
    double trim_from = 0.0;

    /// Arc length in metres to cut off the `to` end of the ribbon. See trim_from.
    double trim_to = 0.0;

    /**
     * @brief Total arc length of polyline in metres
     *
     * Sum of the segment lengths, ignoring trim_from and trim_to. Returns 0.0 for
     * a polyline of fewer than two points.
     */
    [[nodiscard]] double length() const;
};

// ============================================================================
// Graph
// ============================================================================

/**
 * @brief The road network as nodes and edges, built from parsed OSM data
 *
 * Usage:
 * @code
 *     RoadGraph graph;
 *     graph.build(parsed_data);
 *     for (const auto& node : graph.nodes()) {
 *         if (node.is_junction()) { ... }
 *     }
 * @endcode
 *
 * The graph is a value type with no GPU or IO state, so it is cheap to hold
 * alongside ParsedOSMData and safe to build off the main thread.
 */
class RoadGraph {
public:
    /**
     * @brief Build the graph from every road in @p data
     *
     * Clears any previous contents first, then:
     * 1. Counts NodeId references across all roads. A node referenced by two or
     *    more roads, or appearing as a road endpoint, becomes a graph node.
     * 2. Splits every road at its graph nodes into edges.
     * 3. Splits nodes whose arms disagree on layer, one graph node per layer, so a
     *    bridge does not join the road beneath it. Only nodes some road passes
     *    through are split; a node every road ends on is a bridge abutment, not a
     *    grade separation, and stays a single node.
     * 4. Sorts each node's arms ascending by outgoing bearing.
     *
     * Roads with fewer than two points, or whose node_ids vector does not match
     * polyline in size, are skipped: they carry no usable topology.
     *
     * @param data Parsed OSM data with Road::node_ids populated by the parser
     */
    void build(const ParsedOSMData& data);

    /// Drop all nodes and edges
    void clear();

    /// All graph nodes, indexed by GraphNodeId
    [[nodiscard]] const std::vector<GraphNode>& nodes() const { return m_nodes; }

    /// All graph edges, indexed by EdgeId
    [[nodiscard]] const std::vector<GraphEdge>& edges() const { return m_edges; }

    /**
     * @brief Mutable edge access for solvers that annotate edges in place
     *
     * The junction solver writes trim_from and trim_to through this. Resizing the
     * returned vector invalidates every EdgeId held elsewhere, so do not.
     */
    [[nodiscard]] std::vector<GraphEdge>& mutable_edges() { return m_edges; }

    /// Node by ID. The ID must be valid; no bounds check is performed.
    [[nodiscard]] const GraphNode& node(GraphNodeId id) const { return m_nodes[id]; }

    /// Edge by ID. The ID must be valid; no bounds check is performed.
    [[nodiscard]] const GraphEdge& edge(EdgeId id) const { return m_edges[id]; }

    /**
     * @brief Direction of the arm leaving its node, as a unit vector in local metres
     *
     * Equivalent to (cos(arm.bearing), sin(arm.bearing)), computed from the edge
     * polyline. Returns (0, 0) when the arm is invalid or its edge is degenerate.
     *
     * @param arm Arm belonging to a node of this graph
     */
    [[nodiscard]] glm::dvec2 arm_direction(const Arm& arm) const;

    /**
     * @brief Counts describing the built graph, for logging and tests
     */
    struct Stats {
        size_t nodes = 0;               ///< Total graph nodes
        size_t edges = 0;               ///< Total graph edges
        size_t junctions = 0;           ///< Nodes of degree 3 or more
        size_t dead_ends = 0;           ///< Nodes of degree 1
        size_t continuations = 0;       ///< Nodes of degree 2
        size_t roundabout_edges = 0;    ///< Edges with is_roundabout set
        /**
         * @brief Extra graph nodes produced by the layer split
         *
         * nodes minus the number of distinct GraphNode::osm_id values, so it is
         * zero when no OSM node had to be split across layers.
         */
        size_t layer_split_nodes = 0;
    };

    /// Compute the current statistics. O(nodes + edges).
    [[nodiscard]] Stats stats() const;

private:
    std::vector<GraphNode> m_nodes;
    std::vector<GraphEdge> m_edges;
};

} // namespace stratum::osm::road
