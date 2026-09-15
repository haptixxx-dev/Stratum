// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file graph_edit.cpp
 * @brief Implementation of the mutable street graph and its commands
 *
 * Two rules shape everything below, and both are stated in graph_edit.hpp:
 *
 *   1. A node IS its NodeId. No lookup in this file is keyed on a position, and
 *      nothing here compares two coordinates to decide whether two ways meet.
 *      The only distance computation in the file is the projection inside
 *      SplitSegmentCommand, which chooses WHERE to put a new node, never which
 *      existing nodes are the same one.
 *   2. Every mutator marks dirty before it returns, so apply() and revert() both
 *      mark without either having to remember to.
 */

#include "osm/road/graph_edit.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace stratum::osm::road {

namespace {

/// Bytes a segment holds beyond its own fixed size, for Command::footprint()
size_t segment_heap_bytes(const EditableSegment& segment) {
    return segment.node_ids.capacity() * sizeof(NodeId)
           + segment.attributes.surface.capacity()
           + segment.attributes.name.capacity();
}

/// Copy the per-way attributes off a solved GraphEdge. One place, so the field
/// list is checked against GraphEdge exactly once.
SegmentAttributes attributes_of(const GraphEdge& edge) {
    SegmentAttributes a;
    a.type = edge.type;
    a.layer = edge.layer;
    a.width = edge.width;
    a.lanes = edge.lanes;
    a.lanes_forward = edge.lanes_forward;
    a.lanes_backward = edge.lanes_backward;
    a.is_oneway = edge.is_oneway;
    a.is_bridge = edge.is_bridge;
    a.is_tunnel = edge.is_tunnel;
    a.is_roundabout = edge.is_roundabout;
    a.is_link = edge.is_link;
    a.sidewalk = edge.sidewalk;
    a.cycleway = edge.cycleway;
    a.parking = edge.parking;
    a.shoulder = edge.shoulder;
    a.surface = edge.surface;
    a.name = edge.name;
    return a;
}

/// Where a point falls on a polyline: which span, how far along, and how far off
struct Projection {
    size_t span = 0;            ///< Index of the span [span, span + 1]
    double t = 0.0;             ///< Parameter along that span, clamped to [0, 1]
    glm::dvec2 point{0.0};      ///< The projected point itself
    double distance_sq = 0.0;   ///< Squared distance from the query point
};

/**
 * @brief Nearest point on a polyline to @p query
 *
 * Ties go to the earliest span, so the result does not depend on iteration
 * order or on floating-point noise between two equally good answers: a split
 * that lands in a different place on a redo than on the first apply would give
 * the two halves different geometry each cycle.
 *
 * @param pts   At least two points, in local metres
 * @param query Point to project
 */
Projection project_onto(const std::vector<glm::dvec2>& pts, const glm::dvec2& query) {
    Projection best;
    best.point = pts.front();
    best.distance_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const glm::dvec2 a = pts[i];
        const glm::dvec2 span = pts[i + 1] - a;
        const double len_sq = span.x * span.x + span.y * span.y;

        // A zero-length span projects to its own start. Dividing by len_sq here
        // is how a repeated vertex used to produce a NaN parameter, which then
        // compared false against every distance and left `best` at its initial
        // sentinel.
        double t = 0.0;
        if (len_sq > 0.0) {
            const glm::dvec2 to_query = query - a;
            t = (to_query.x * span.x + to_query.y * span.y) / len_sq;
            t = std::clamp(t, 0.0, 1.0);
        }

        const glm::dvec2 point = a + span * t;
        const glm::dvec2 delta = query - point;
        const double distance_sq = delta.x * delta.x + delta.y * delta.y;

        if (distance_sq < best.distance_sq) {
            best.span = i;
            best.t = t;
            best.point = point;
            best.distance_sq = distance_sq;
        }
    }

    return best;
}

/// Squared distance between two local-metre points
inline double distance_sq(const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 d = a - b;
    return d.x * d.x + d.y * d.y;
}

/// The ids of @p ids with every occurrence of @p drop removed and every
/// consecutive duplicate that leaves behind collapsed
std::vector<NodeId> without_node(const std::vector<NodeId>& ids, NodeId drop) {
    std::vector<NodeId> out;
    out.reserve(ids.size());
    for (const NodeId id : ids) {
        if (id == drop) continue;
        // Dropping the middle of [a, b, a] leaves [a, a], which is not a line.
        // Collapsing here rather than rejecting lets the caller detect the case
        // by the size of what comes back.
        if (!out.empty() && out.back() == id) continue;
        out.push_back(id);
    }
    return out;
}

} // namespace

// ============================================================================
// SegmentAttributes
// ============================================================================

bool operator==(const SegmentAttributes& a, const SegmentAttributes& b) {
    return a.type == b.type && a.layer == b.layer && a.width == b.width
           && a.lanes == b.lanes && a.lanes_forward == b.lanes_forward
           && a.lanes_backward == b.lanes_backward && a.is_oneway == b.is_oneway
           && a.is_bridge == b.is_bridge && a.is_tunnel == b.is_tunnel
           && a.is_roundabout == b.is_roundabout && a.is_link == b.is_link
           && a.sidewalk == b.sidewalk && a.cycleway == b.cycleway
           && a.parking == b.parking && a.shoulder == b.shoulder
           && a.surface == b.surface && a.name == b.name;
}

bool operator!=(const SegmentAttributes& a, const SegmentAttributes& b) { return !(a == b); }

// ============================================================================
// EditableGraph -- lookup
// ============================================================================

bool EditableGraph::contains_node(NodeId id) const { return m_nodes.find(id) != m_nodes.end(); }

bool EditableGraph::contains_segment(SegmentId id) const {
    return m_segments.find(id) != m_segments.end();
}

const EditableNode* EditableGraph::find_node(NodeId id) const {
    const auto it = m_nodes.find(id);
    return it == m_nodes.end() ? nullptr : &it->second;
}

const EditableSegment* EditableGraph::find_segment(SegmentId id) const {
    const auto it = m_segments.find(id);
    return it == m_segments.end() ? nullptr : &it->second;
}

const std::vector<SegmentId>& EditableGraph::segments_at(NodeId id) const {
    // Function-local rather than a namespace-scope constant so that a caller who
    // holds the reference across a static-initialisation boundary cannot get one
    // to a vector that has not been constructed yet.
    static const std::vector<SegmentId> kNone;
    const auto it = m_usage.find(id);
    return it == m_usage.end() ? kNone : it->second.segments;
}

size_t EditableGraph::reference_count(NodeId id) const {
    const auto it = m_usage.find(id);
    return it == m_usage.end() ? 0u : it->second.references;
}

size_t EditableGraph::arm_count(NodeId id) const {
    const auto it = m_usage.find(id);
    return it == m_usage.end() ? 0u : it->second.arms;
}

std::vector<NodeId> EditableGraph::orphan_nodes() const {
    std::vector<NodeId> out;
    for (const auto& entry : m_nodes) {
        if (reference_count(entry.first) == 0) out.push_back(entry.first);
    }
    return out;
}

// ============================================================================
// EditableGraph -- geometry
// ============================================================================

std::vector<glm::dvec2> EditableGraph::polyline(SegmentId id) const {
    std::vector<glm::dvec2> out;
    const EditableSegment* segment = find_segment(id);
    if (segment == nullptr) return out;

    out.reserve(segment->node_ids.size());
    for (const NodeId node : segment->node_ids) {
        const EditableNode* record = find_node(node);
        if (record == nullptr) {
            // Cannot happen while erase_node() refuses a referenced node. Skipped
            // rather than substituted, because a made-up coordinate here would
            // reach the solver as real geometry.
            spdlog::error("EditableGraph: segment {} references missing node {}", id, node);
            continue;
        }
        out.push_back(record->position);
    }
    return out;
}

double EditableGraph::length(SegmentId id) const {
    const std::vector<glm::dvec2> pts = polyline(id);
    double total = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
        total += std::sqrt(distance_sq(pts[i], pts[i - 1]));
    }
    return total;
}

// ============================================================================
// EditableGraph -- dirty tracking
// ============================================================================

void EditableGraph::clear_dirty() {
    m_dirty_segments.clear();
    m_dirty_nodes.clear();
}

void EditableGraph::mark_node_dirty(NodeId id) {
    m_dirty_nodes.insert(id);
    // Every arm of a junction depends on the junction, so a node that moved
    // invalidates the solved geometry of every segment on it, not only the
    // segments whose own vertices changed.
    for (const SegmentId segment : segments_at(id)) {
        m_dirty_segments.insert(segment);
    }
}

void EditableGraph::mark_segment_dirty(SegmentId id) { m_dirty_segments.insert(id); }

// ============================================================================
// EditableGraph -- id allocation
// ============================================================================

NodeId EditableGraph::reserve_node_id() {
    const NodeId id = m_next_node_id;
    --m_next_node_id;
    return id;
}

WayId EditableGraph::reserve_way_id() {
    const WayId id = m_next_way_id;
    --m_next_way_id;
    return id;
}

SegmentId EditableGraph::reserve_segment_id() { return m_next_segment_id++; }

// ============================================================================
// EditableGraph -- mutation
// ============================================================================

bool EditableGraph::valid_nodes(const std::vector<NodeId>& ids) const {
    if (ids.size() < 2) return false;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (!contains_node(ids[i])) return false;
        if (i > 0 && ids[i] == ids[i - 1]) return false;
    }
    return true;
}

bool EditableGraph::insert_node(EditableNode node) {
    if (node.id == kInvalidNode) return false;
    if (contains_node(node.id)) return false;

    // Keep the allocator strictly below anything already present, so a graph
    // seeded from an extract that has been through an editor -- and therefore
    // already contains negative ids -- cannot have a later authored node collide
    // with one of them.
    if (node.id <= m_next_node_id) m_next_node_id = node.id - 1;

    const NodeId id = node.id;
    m_nodes.emplace(id, std::move(node));
    mark_node_dirty(id);
    return true;
}

bool EditableGraph::erase_node(NodeId id) {
    const auto it = m_nodes.find(id);
    if (it == m_nodes.end()) return false;
    if (reference_count(id) != 0) return false;

    m_nodes.erase(it);
    // No incident segments left by the guard above, so this marks the node only.
    // The id stays in the set although the node is gone: that is exactly how B4
    // learns to drop whatever it solved for it.
    m_dirty_nodes.insert(id);
    return true;
}

bool EditableGraph::set_node_position(NodeId id, glm::dvec2 position) {
    const auto it = m_nodes.find(id);
    if (it == m_nodes.end()) return false;

    it->second.position = position;
    mark_node_dirty(id);
    return true;
}

bool EditableGraph::insert_segment(EditableSegment segment) {
    if (segment.id == kInvalidSegment) return false;
    if (contains_segment(segment.id)) return false;
    if (!valid_nodes(segment.node_ids)) return false;

    if (segment.source_way <= m_next_way_id) m_next_way_id = segment.source_way - 1;
    if (segment.id >= m_next_segment_id) m_next_segment_id = segment.id + 1;

    const SegmentId id = segment.id;
    const std::vector<NodeId> ids = segment.node_ids;
    m_segments.emplace(id, std::move(segment));
    add_usage(id, ids);

    mark_segment_dirty(id);
    for (const NodeId node : ids) mark_node_dirty(node);
    return true;
}

bool EditableGraph::erase_segment(SegmentId id) {
    const auto it = m_segments.find(id);
    if (it == m_segments.end()) return false;

    const std::vector<NodeId> ids = it->second.node_ids;
    remove_usage(id, ids);
    m_segments.erase(it);

    mark_segment_dirty(id);
    // Marked AFTER the erase, so the neighbours that remain at each node are
    // marked and this segment is not re-added through them. Its own id is in the
    // set either way, which is what tells B4 to drop it.
    for (const NodeId node : ids) mark_node_dirty(node);
    return true;
}

bool EditableGraph::set_segment_nodes(SegmentId id, std::vector<NodeId> node_ids) {
    const auto it = m_segments.find(id);
    if (it == m_segments.end()) return false;
    if (!valid_nodes(node_ids)) return false;

    const std::vector<NodeId> old = it->second.node_ids;
    remove_usage(id, old);
    it->second.node_ids = node_ids;
    add_usage(id, node_ids);

    mark_segment_dirty(id);
    // Both lists: a node the segment left behind has one arm fewer and its
    // junction has to be re-solved just as much as a node it has just reached.
    for (const NodeId node : old) mark_node_dirty(node);
    for (const NodeId node : node_ids) mark_node_dirty(node);
    return true;
}

void EditableGraph::add_usage(SegmentId segment, const std::vector<NodeId>& ids) {
    for (size_t i = 0; i < ids.size(); ++i) {
        NodeUsage& usage = m_usage[ids[i]];
        ++usage.references;
        if (i == 0 || i + 1 == ids.size()) ++usage.arms;

        // The segment list holds each segment once however many vertices of it
        // sit on the node, so a way that doubles back does not read as two ways.
        auto& list = usage.segments;
        const auto at = std::lower_bound(list.begin(), list.end(), segment);
        if (at == list.end() || *at != segment) list.insert(at, segment);
    }
}

void EditableGraph::remove_usage(SegmentId segment, const std::vector<NodeId>& ids) {
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto entry = m_usage.find(ids[i]);
        if (entry == m_usage.end()) continue;

        NodeUsage& usage = entry->second;
        if (usage.references > 0) --usage.references;
        if ((i == 0 || i + 1 == ids.size()) && usage.arms > 0) --usage.arms;

        if (usage.references == 0) {
            // Last reference from any segment: the whole entry goes, which also
            // drops this segment from the list.
            m_usage.erase(entry);
            continue;
        }

        // Still referenced. Drop the segment from the list only once no vertex of
        // it remains on the node, or a way that doubles back would leave the node
        // after its first vertex was accounted for.
        bool still_here = false;
        for (size_t j = i + 1; j < ids.size(); ++j) {
            if (ids[j] == ids[i]) { still_here = true; break; }
        }
        if (still_here) continue;

        auto& list = usage.segments;
        const auto at = std::lower_bound(list.begin(), list.end(), segment);
        if (at != list.end() && *at == segment) list.erase(at);
    }
}

// ============================================================================
// EditableGraph -- loading
// ============================================================================

EditableGraph EditableGraph::from_road_graph(const RoadGraph& graph) {
    EditableGraph out;

    // ------------------------------------------------------------------------
    // Prime both allocators below everything the extract already contains.
    //
    // Done before a single node is created, not as they go: an id handed out for
    // a layer-split node must not be able to land on an OSM id that a later edge
    // still refers to. That collision would silently join two ways that do not
    // meet, which is the exact failure this whole file is built to make
    // impossible.
    // ------------------------------------------------------------------------
    // Seeded at 0 rather than at -1 so that an extract of purely positive ids
    // leaves the allocators at their natural start of -1 instead of burning one.
    NodeId min_node = 0;
    WayId min_way = 0;
    for (const GraphEdge& edge : graph.edges()) {
        for (const NodeId id : edge.node_ids) min_node = std::min(min_node, id);
        min_way = std::min(min_way, edge.source_way);
    }
    for (const GraphNode& node : graph.nodes()) min_node = std::min(min_node, node.osm_id);

    // The guard is for form's sake: an id at the bottom of the int64 range is not
    // something OSM or any editor produces, but decrementing past it is UB.
    if (min_node > std::numeric_limits<NodeId>::min()) {
        out.m_next_node_id = std::min<NodeId>(-1, min_node - 1);
    }
    if (min_way > std::numeric_limits<WayId>::min()) {
        out.m_next_way_id = std::min<WayId>(-1, min_way - 1);
    }

    // Which NodeId each GraphNodeId became. RoadGraph has already decided what
    // joins what -- including the per-layer split at a grade separation and the
    // deliberate NON-split at a bridge abutment -- so going through its node
    // handles copies that decision instead of re-deriving it from layer=*.
    std::vector<NodeId> node_of(graph.nodes().size(), kInvalidNode);

    // OSM ids already spent. The second GraphNode carrying one is the layer
    // split, and it has to get its own identity or the bridge rejoins the road
    // beneath it.
    std::set<NodeId> claimed;
    size_t rekeyed = 0;

    auto claim = [&](NodeId preferred) -> NodeId {
        if (preferred != kInvalidNode && claimed.insert(preferred).second) return preferred;
        ++rekeyed;
        const NodeId fresh = out.reserve_node_id();
        claimed.insert(fresh);
        return fresh;
    };

    size_t skipped = 0;
    for (const GraphEdge& edge : graph.edges()) {
        if (edge.polyline.size() < 2 || edge.node_ids.size() != edge.polyline.size()) {
            ++skipped;
            continue;
        }

        const size_t last = edge.polyline.size() - 1;
        std::vector<NodeId> ids;
        ids.reserve(edge.polyline.size());

        for (size_t i = 0; i <= last; ++i) {
            NodeId id = kInvalidNode;

            if (i == 0 || i == last) {
                const GraphNodeId handle = (i == 0) ? edge.from : edge.to;
                if (handle < node_of.size()) {
                    if (node_of[handle] == kInvalidNode) {
                        const GraphNode& source = graph.node(handle);
                        id = claim(source.osm_id);

                        EditableNode node;
                        node.id = id;
                        node.position = source.position;
                        node.layer = source.layer;
                        node.has_signals = source.has_signals;
                        node.has_crossing = source.has_crossing;
                        node.is_turning_circle = source.is_turning_circle;
                        out.insert_node(std::move(node));

                        node_of[handle] = id;
                    } else {
                        id = node_of[handle];
                    }
                }
            }

            if (id == kInvalidNode) {
                // An interior shape point, which by construction no other way
                // references -- RoadGraph would have made it a split point if one
                // did -- or an endpoint whose handle is unset, which is broken
                // data and is given its own identity rather than joined to
                // anything.
                id = claim(edge.node_ids[i]);

                EditableNode node;
                node.id = id;
                node.position = edge.polyline[i];
                node.layer = edge.layer;
                out.insert_node(std::move(node));
            }

            if (!ids.empty() && ids.back() == id) continue;
            ids.push_back(id);
        }

        if (ids.size() < 2) {
            // A two-point edge whose ends resolved to one node. There is no line
            // here; the node stays and orphan_nodes() lists it.
            ++skipped;
            continue;
        }

        EditableSegment segment;
        segment.id = out.reserve_segment_id();
        segment.source_way = edge.source_way;
        segment.node_ids = std::move(ids);
        segment.attributes = attributes_of(edge);
        out.insert_segment(std::move(segment));
    }

    // A freshly loaded graph is not dirty: nothing has been edited, and handing
    // B4 every segment as "changed" would make the first frame after a load do
    // the full solve twice.
    out.clear_dirty();

    spdlog::info("EditableGraph: loaded {} nodes, {} segments from the road graph "
                 "({} re-keyed for a layer split, {} edges skipped)",
                 out.node_count(), out.segment_count(), rekeyed, skipped);
    return out;
}

// ============================================================================
// GraphEditCommand
// ============================================================================

NodeId GraphEditCommand::reserve_node_id(EditableGraph& graph) { return graph.reserve_node_id(); }
WayId GraphEditCommand::reserve_way_id(EditableGraph& graph) { return graph.reserve_way_id(); }
SegmentId GraphEditCommand::reserve_segment_id(EditableGraph& graph) {
    return graph.reserve_segment_id();
}

bool GraphEditCommand::insert_node(EditableNode node) {
    return m_graph->insert_node(std::move(node));
}
bool GraphEditCommand::erase_node(NodeId id) { return m_graph->erase_node(id); }
bool GraphEditCommand::set_node_position(NodeId id, glm::dvec2 position) {
    return m_graph->set_node_position(id, position);
}
bool GraphEditCommand::insert_segment(EditableSegment segment) {
    return m_graph->insert_segment(std::move(segment));
}
bool GraphEditCommand::erase_segment(SegmentId id) { return m_graph->erase_segment(id); }
bool GraphEditCommand::set_segment_nodes(SegmentId id, std::vector<NodeId> node_ids) {
    return m_graph->set_segment_nodes(id, std::move(node_ids));
}

// ============================================================================
// AddNodeCommand
// ============================================================================

AddNodeCommand::AddNodeCommand(EditableGraph& graph, glm::dvec2 position, int layer)
    : GraphEditCommand(graph) {
    // Reserved here, not in apply(), so that node() reads before the command is
    // executed and a caller can create a node and the segment through it inside
    // one transaction. It also makes a redo land on the SAME id: a fresh id per
    // apply would leave every stored handle pointing at a node that redo deleted.
    m_node.id = reserve_node_id(graph);
    m_node.position = position;
    m_node.layer = layer;
}

bool AddNodeCommand::apply() { return insert_node(m_node); }

void AddNodeCommand::revert() {
    if (!erase_node(m_node.id)) {
        // Only reachable if something attached a segment to this node without
        // going through the stack, which the private API is there to prevent.
        spdlog::error("AddNodeCommand: node {} could not be removed at undo", m_node.id);
    }
}

std::string AddNodeCommand::describe() const { return "Add node"; }

// ============================================================================
// MoveNodeCommand
// ============================================================================

MoveNodeCommand::MoveNodeCommand(EditableGraph& graph, NodeId node, glm::dvec2 position)
    : GraphEditCommand(graph), m_node(node), m_position(position) {}

bool MoveNodeCommand::apply() {
    const EditableNode* record = graph().find_node(m_node);
    if (record == nullptr) return false;

    // A move to where the node already is records nothing. An undo menu that
    // fills up with steps that do nothing reads to a user as undo being broken,
    // and CommandStack::execute() is built to make refusing free.
    if (record->position == m_position) return false;

    m_old_position = record->position;
    return set_node_position(m_node, m_position);
}

void MoveNodeCommand::revert() {
    if (!set_node_position(m_node, m_old_position)) {
        spdlog::error("MoveNodeCommand: node {} is gone at undo", m_node);
    }
}

std::string MoveNodeCommand::describe() const { return "Move node"; }

bool MoveNodeCommand::merge(const stratum::scene::Command& next) {
    const auto* other = dynamic_cast<const MoveNodeCommand*>(&next);
    if (other == nullptr || !same_graph(*other) || other->m_node != m_node) return false;

    // Refuse a merge that would leave this command a no-op.
    //
    // Drag a node out and back and the gesture ends where it started. Absorbing
    // that blindly leaves a command whose old and new positions are equal: undo
    // does nothing visible, and the redo hits apply()'s own equal-value guard, so
    // the stack logs a refusal and DROPS the step -- taking the rest of the redo
    // branch with it. That exact bug was found in A1's RenameLayerCommand and the
    // fix is the same one.
    //
    // Refusing costs one extra undo step for a round trip and keeps both halves
    // individually reversible, which is the honest trade.
    if (other->m_position == m_old_position) return false;

    // The successor has already applied, so the graph already holds its position.
    // Taking it over means this command's revert() returns to where the drag
    // started, which is what m_old_position still holds.
    m_position = other->m_position;
    return true;
}

// ============================================================================
// DeleteNodeCommand
// ============================================================================

DeleteNodeCommand::DeleteNodeCommand(EditableGraph& graph, NodeId node)
    : GraphEditCommand(graph), m_node(node) {}

bool DeleteNodeCommand::apply() {
    const EditableNode* record = graph().find_node(m_node);
    if (record == nullptr) return false;

    // Shared: two ways meet here, or one ring arrives and leaves. Refused. See
    // the class comment for why this is not a cascade and not a retraction.
    if (graph().reference_count(m_node) > 1) return false;

    m_node_copy = *record;
    m_segment_removed = false;
    m_segment_copy = EditableSegment{};

    // Copied out by value before anything mutates: segments_at() hands back a
    // reference into the usage table, and the edits below rewrite that table.
    const std::vector<SegmentId> users = graph().segments_at(m_node);
    if (!users.empty()) {
        const EditableSegment* segment = graph().find_segment(users.front());
        if (segment == nullptr) return false;

        m_segment_copy = *segment;
        const SegmentId target = segment->id;
        const std::vector<NodeId> remaining = without_node(segment->node_ids, m_node);

        if (remaining.size() < 2) {
            // Nothing left to draw. The segment goes with the node rather than
            // being left as a one-point way that no consumer can use.
            m_segment_removed = true;
            if (!erase_segment(target)) return false;
        } else if (!set_segment_nodes(target, remaining)) {
            return false;
        }
    }

    // Last, because erase_node() refuses while anything still references the id.
    // That guard is the whole reason the segment is rewritten first.
    return erase_node(m_node);
}

void DeleteNodeCommand::revert() {
    // Node first: insert_segment() and set_segment_nodes() both refuse a list
    // naming a node the table does not have.
    if (!insert_node(m_node_copy)) {
        spdlog::error("DeleteNodeCommand: node {} could not be restored at undo", m_node);
        return;
    }

    if (m_segment_copy.id == kInvalidSegment) return;

    const bool ok = m_segment_removed ? insert_segment(m_segment_copy)
                                      : set_segment_nodes(m_segment_copy.id,
                                                          m_segment_copy.node_ids);
    if (!ok) {
        spdlog::error("DeleteNodeCommand: segment {} could not be restored at undo",
                      m_segment_copy.id);
    }
}

std::string DeleteNodeCommand::describe() const { return "Delete node"; }

size_t DeleteNodeCommand::footprint() const {
    return sizeof(DeleteNodeCommand) + segment_heap_bytes(m_segment_copy);
}

// ============================================================================
// AddSegmentCommand
// ============================================================================

AddSegmentCommand::AddSegmentCommand(EditableGraph& graph, std::vector<NodeId> node_ids,
                                     SegmentAttributes attributes)
    : GraphEditCommand(graph) {
    // Both handles reserved up front, for the same reasons as AddNodeCommand:
    // readable before execute(), and stable across an undo/redo cycle.
    m_segment.id = reserve_segment_id(graph);
    m_segment.source_way = reserve_way_id(graph);
    m_segment.node_ids = std::move(node_ids);
    m_segment.attributes = std::move(attributes);
}

bool AddSegmentCommand::apply() { return insert_segment(m_segment); }

void AddSegmentCommand::revert() {
    if (!erase_segment(m_segment.id)) {
        spdlog::error("AddSegmentCommand: segment {} could not be removed at undo",
                      m_segment.id);
    }
}

std::string AddSegmentCommand::describe() const { return "Add street segment"; }

size_t AddSegmentCommand::footprint() const {
    return sizeof(AddSegmentCommand) + segment_heap_bytes(m_segment);
}

// ============================================================================
// DeleteSegmentCommand
// ============================================================================

DeleteSegmentCommand::DeleteSegmentCommand(EditableGraph& graph, SegmentId segment)
    : GraphEditCommand(graph), m_segment(segment) {}

bool DeleteSegmentCommand::apply() {
    const EditableSegment* record = graph().find_segment(m_segment);
    if (record == nullptr) return false;

    // Copied BEFORE the erase, and copied whole: the command owns what its undo
    // needs, which is the rule scene/command.hpp states and the reason a delete
    // may not hold a reference to what it deleted.
    m_copy = *record;
    return erase_segment(m_segment);
}

void DeleteSegmentCommand::revert() {
    if (!insert_segment(m_copy)) {
        spdlog::error("DeleteSegmentCommand: segment {} could not be restored at undo",
                      m_segment);
    }
}

std::string DeleteSegmentCommand::describe() const { return "Delete street segment"; }

size_t DeleteSegmentCommand::footprint() const {
    return sizeof(DeleteSegmentCommand) + segment_heap_bytes(m_copy);
}

// ============================================================================
// SplitSegmentCommand
// ============================================================================

SplitSegmentCommand::SplitSegmentCommand(EditableGraph& graph, SegmentId segment,
                                         glm::dvec2 point, double snap)
    : GraphEditCommand(graph), m_segment(segment), m_point(point), m_snap(snap) {
    m_first = reserve_segment_id(graph);
    m_second = reserve_segment_id(graph);
    m_reserved_join = reserve_node_id(graph);
}

bool SplitSegmentCommand::apply() {
    const EditableSegment* record = graph().find_segment(m_segment);
    if (record == nullptr) return false;

    const std::vector<glm::dvec2> pts = graph().polyline(m_segment);
    const std::vector<NodeId>& ids = record->node_ids;
    if (pts.size() < 2 || pts.size() != ids.size()) return false;

    const Projection hit = project_onto(pts, m_point);
    const double snap_sq = m_snap * m_snap;

    // Does the split land on a vertex that already exists? Only then is an
    // existing node reused. Note that this compares the SPLIT POINT against
    // vertices to decide where to cut -- it never decides that two nodes are the
    // same node, which stays a question of identity alone.
    size_t on_vertex = ids.size();   // ids.size() means "between two vertices"
    if (distance_sq(hit.point, pts[hit.span]) <= snap_sq) {
        on_vertex = hit.span;
    } else if (distance_sq(hit.point, pts[hit.span + 1]) <= snap_sq) {
        on_vertex = hit.span + 1;
    }

    // An endpoint split leaves one half with no length. A zero-length arm has no
    // direction, and road_graph.cpp drops such an edge outright, so the result
    // would be a street that silently lost a piece.
    if (on_vertex == 0 || on_vertex == ids.size() - 1) return false;

    // ------------------------------------------------------------------------
    // Everything that can refuse has now refused. What follows mutates, and a
    // refusal from here would leave the graph half-split with the stack told
    // nothing happened -- the one shape scene/command.hpp says a command must
    // not have. So the rest is written so it cannot fail: the handles are fresh,
    // and both vertex lists are built from a list that was already valid.
    // ------------------------------------------------------------------------
    m_original = *record;

    std::vector<NodeId> first;
    std::vector<NodeId> second;

    if (on_vertex < ids.size()) {
        m_join = ids[on_vertex];
        m_created_join = false;
        first.assign(ids.begin(), ids.begin() + static_cast<ptrdiff_t>(on_vertex) + 1);
        second.assign(ids.begin() + static_cast<ptrdiff_t>(on_vertex), ids.end());
    } else {
        EditableNode node;
        node.id = m_reserved_join;
        node.position = hit.point;
        // The join inherits the WAY's layer, not a guess: a split in the middle
        // of a bridge deck must stay on the deck, or the piece either side of it
        // would key to a different grade on the next load.
        node.layer = record->attributes.layer;
        if (!insert_node(std::move(node))) {
            spdlog::error("SplitSegmentCommand: reserved join node {} was already taken",
                          m_reserved_join);
            return false;
        }

        m_join = m_reserved_join;
        m_created_join = true;
        first.assign(ids.begin(), ids.begin() + static_cast<ptrdiff_t>(hit.span) + 1);
        first.push_back(m_join);
        second.push_back(m_join);
        second.insert(second.end(), ids.begin() + static_cast<ptrdiff_t>(hit.span) + 1,
                      ids.end());
    }

    if (!erase_segment(m_segment)) {
        if (m_created_join) erase_node(m_join);
        return false;
    }

    EditableSegment half = m_original;
    half.id = m_first;
    half.node_ids = std::move(first);
    // The attributes ride along in the copy above. One struct assignment, so a
    // tag cannot be dropped on one half by a field the copy forgot to list.
    const bool first_ok = insert_segment(std::move(half));

    EditableSegment other = m_original;
    other.id = m_second;
    other.node_ids = std::move(second);
    const bool second_ok = insert_segment(std::move(other));

    if (!first_ok || !second_ok) {
        spdlog::error("SplitSegmentCommand: segment {} split into halves the graph refused",
                      m_segment);
        return false;
    }
    return true;
}

void SplitSegmentCommand::revert() {
    // Halves first, then the join node: erase_node() refuses while anything
    // references the id, which is the invariant that stops a dangling node id
    // ever existing.
    erase_segment(m_second);
    erase_segment(m_first);

    if (m_created_join && !erase_node(m_join)) {
        spdlog::error("SplitSegmentCommand: join node {} could not be removed at undo", m_join);
    }

    if (!insert_segment(m_original)) {
        spdlog::error("SplitSegmentCommand: segment {} could not be restored at undo",
                      m_segment);
    }
}

std::string SplitSegmentCommand::describe() const { return "Split street segment"; }

size_t SplitSegmentCommand::footprint() const {
    return sizeof(SplitSegmentCommand) + segment_heap_bytes(m_original);
}

} // namespace stratum::osm::road
