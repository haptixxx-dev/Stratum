// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file graph_edit.hpp
 * @brief The mutable street graph: authoring a network instead of importing one
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why a second graph exists
 *
 * RoadGraph is a *derived* structure. `RoadGraph::build()` clears itself and
 * rebuilds from a ParsedOSMData, its GraphNodeId and EdgeId are dense indices
 * into two vectors, and the junction solver writes trims into the edges in
 * place. That is exactly right for consuming an extract and completely wrong for
 * authoring one:
 *
 *   - A dense index cannot survive a delete. Erasing node 4 of a vector shifts
 *     every id above it, and every id a tool, a selection or an undo command was
 *     holding now names a different node. A handle that silently retargets is
 *     the single worst failure mode an editor can have.
 *   - Nothing in RoadGraph can be edited without re-running build(), and
 *     re-running build() throws away the solve.
 *
 * EditableGraph is therefore the authoring model, with stable handles, an
 * explicit dirty set for the incremental re-solve (B4), and every mutation
 * behind a Command. It does NOT re-solve anything. Marking what changed is the
 * whole of its responsibility on that front.
 *
 * ### Identity is an OSM node id, and nothing else
 *
 * CLAUDE.md states the rule the entire road pipeline rests on: **junctions are
 * found by shared OSM node identity, never by endpoint proximity.** Proximity
 * clustering misses a T-junction whose shared node is interior to the through
 * road, and invents a junction wherever a bridge passes over another road.
 *
 * So a node here IS its NodeId. There is no second handle type for nodes, no
 * index, and no position-keyed lookup anywhere in this file. Two segments meet
 * if and only if their `node_ids` vectors contain the same NodeId; two segments
 * whose endpoints sit on the same coordinate under different ids do not meet,
 * and `is_shared()` says so. Making identity and the handle the same value is
 * what stops a later contributor "helpfully" adding a proximity join: there is
 * no index to key one on.
 *
 * A consequence worth stating: **every vertex of a segment is a node.** There
 * are no anonymous polyline points. That mirrors OSM itself (a way is a list of
 * node references and nothing else), it removes the class of bug where a
 * segment's cached polyline drifts out of step with the nodes it is made of,
 * and it makes "split here" and "drag this vertex" the same operation on the
 * same kind of thing. Geometry lives on the node; `polyline()` resolves it.
 *
 * ### Ids for things that did not come from OSM
 *
 * OSM element ids are positive int64. Negative ids are the established
 * convention for locally-created elements -- JOSM uses them, and an osmChange
 * `<create>` block is full of them -- so a locally authored node takes the next
 * id from a counter that starts at -1 and only ever decreases, and a locally
 * authored way takes one from a separate counter in the WayId space. The two
 * spaces are independent in OSM and are kept independent here.
 *
 * Two properties fall out, and both are load-bearing:
 *
 *   - **No collision with real data.** A negative id can never equal an id the
 *     parser produced, whatever extract is loaded.
 *   - **Never reused.** The counters do not go back up when a node is deleted.
 *     A handle to a deleted node is *detectable* -- `contains_node()` is false
 *     -- and can never start naming some other node, which is the failure a
 *     free-list produces. Same argument as LayerId in scene/layer.hpp.
 *
 * Seeding from an import primes both counters below anything already present,
 * because an extract that has been through an editor can itself contain
 * negative ids. `from_road_graph()` is the one place that does it, and the only
 * place that can: every other id in the file comes out of the allocators
 * themselves, so a second copy of the rule further down would be a branch no
 * caller could reach and no test could hold to anything.
 *
 * ### Every mutation is a Command
 *
 * EditableGraph's mutating API is private, and GraphEditCommand is its only
 * friend. That is the same wall LayerTree puts up, for the same reason given in
 * scene/command.hpp: a mutation that skips the stack is a hole in the history,
 * and the symptom shows up as an undo three steps later restoring state that
 * was never current. Here it is doubly true, because the dirty set is
 * maintained by those same private mutators -- a direct write would leave B4
 * re-solving a network it believes is clean.
 *
 * The one thing that is *not* a command is `from_road_graph()`, and that is why
 * it is a static factory returning a fresh graph rather than a `reset()`
 * member. Loading a document is not an edit; there is no previous state to
 * return to, and a factory cannot corrupt a history because there is no history
 * yet to corrupt. The caller pairs it with `CommandStack::clear()`.
 *
 * This header includes scene/command.hpp, so src/osm/road depends upward on
 * src/scene. Both live in stratum_core and command.hpp pulls in nothing but the
 * standard library, so the dependency costs nothing; the alternative -- a
 * second, road-specific undo stack -- would mean a user's undo sometimes walked
 * back a road edit and sometimes a layer edit depending on which stack the tool
 * happened to use.
 */

#pragma once

#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"
#include "scene/command.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Handles
// ============================================================================

/**
 * @brief Stable, never-reused handle to a segment
 *
 * Deliberately NOT a WayId. Several segments share one way -- that is what a
 * split produces, and what RoadGraph produces when it cuts a way at its
 * junctions -- so a way id cannot address one of them.
 */
using SegmentId = uint64_t;

/// Not a segment. Valid segment ids start at 1.
inline constexpr SegmentId kInvalidSegment = 0;

/**
 * @brief Not a node
 *
 * 0 is not a valid OSM node id, and the local allocator runs negative, so no
 * real node can ever collide with this.
 */
inline constexpr NodeId kInvalidNode = 0;

/**
 * @brief Default distance, in metres, at which a split snaps to a vertex
 *
 * A millimetre. It is not a UI tolerance and must not be used as one: its only
 * job is to stop a split producing a half of zero length, because a zero-length
 * arm has no direction and the junction solver cannot use one -- road_graph.cpp
 * drops such edges outright. A tool that wants "split near the vertex I clicked"
 * passes its own, much larger, screen-derived tolerance.
 */
inline constexpr double kSplitSnapMetres = 1e-3;

// ============================================================================
// Data
// ============================================================================

/**
 * @brief Everything a segment carries that describes the WAY rather than the line
 *
 * One struct, not seventeen loose members, for one reason: SplitSegmentCommand
 * has to give both halves every tag the original had, and a memberwise copy of
 * a struct cannot forget a field the way a hand-written field-by-field copy can.
 * The field set mirrors GraphEdge so a segment can be handed to the existing
 * solver without a translation table growing between them.
 */
struct SegmentAttributes {
    RoadType type = RoadType::Unknown;  ///< Road classification
    int layer = 0;                      ///< OSM layer=*
    float width = 6.0f;                 ///< Carriageway width in metres

    int lanes = 2;                      ///< Total lane count
    int lanes_forward = -1;             ///< lanes:forward=*, -1 when unspecified
    int lanes_backward = -1;            ///< lanes:backward=*, -1 when unspecified

    bool is_oneway = false;             ///< One-way along the node_ids order
    bool is_bridge = false;             ///< Bridge segment
    bool is_tunnel = false;             ///< Tunnel segment
    bool is_roundabout = false;         ///< Part of a roundabout cycle
    bool is_link = false;               ///< highway=*_link ramp

    SideFlags sidewalk = SideFlags::Unknown;    ///< Sidewalk presence by side
    SideFlags cycleway = SideFlags::Unknown;    ///< Cycle lane presence by side
    SideFlags parking = SideFlags::Unknown;     ///< On-street parking by side
    SideFlags shoulder = SideFlags::Unknown;    ///< Shoulder presence by side

    std::string surface;    ///< Raw surface=* value, lowercased. Empty when absent.
    std::string name;       ///< Road name from name=*
};

/**
 * @brief Exact equality over EVERY field
 *
 * `segment_attributes_compare_field_by_field` pins this one field at a time, and
 * it has to: a test that asserts "the split copied the tags" through nothing but
 * this operator cannot tell a dropped field from an operator that answers true to
 * everything, and for a while none of them could. A field added to the struct
 * above needs a line here AND a line in that test; neither alone is a check.
 */
[[nodiscard]] bool operator==(const SegmentAttributes& a, const SegmentAttributes& b);
[[nodiscard]] bool operator!=(const SegmentAttributes& a, const SegmentAttributes& b);

/**
 * @brief One point of the network, addressed by its OSM node id
 *
 * Carries the position, because geometry lives on the node and nowhere else.
 * Two segments that reference this node share this one coordinate, so moving it
 * moves both -- which is the point.
 */
struct EditableNode {
    /// Identity. Positive for a node that came from OSM, negative for one
    /// authored here. Never 0.
    NodeId id = kInvalidNode;

    /// Local metres, the same space RoadGraph and the whole road pipeline use
    glm::dvec2 position{0.0};

    /**
     * @brief OSM layer=* of this node
     *
     * Carried so a grade separation survives a load. It is NOT part of the
     * identity: identity is the id alone, and `from_road_graph()` re-keys rather
     * than compound-keying, so that the rule "two ways meet iff they share a
     * node id" has no exceptions anywhere in this file.
     */
    int layer = 0;

    bool has_signals = false;       ///< highway=traffic_signals
    bool has_crossing = false;      ///< highway=crossing
    bool is_turning_circle = false; ///< highway=turning_circle or turning_loop
};

/**
 * @brief One stretch of street: an ordered list of node references, plus tags
 *
 * There is no polyline member. The geometry is `node_ids` resolved through the
 * node table by `EditableGraph::polyline()`, so it cannot fall out of step with
 * the nodes. A cached copy here is the obvious optimisation and is deliberately
 * not taken: the bug it buys -- a drag that moves the node but not the segment
 * that draws it -- is worse than the lookup it saves, and B4 has the dirty set
 * to cache against instead.
 */
struct EditableSegment {
    SegmentId id = kInvalidSegment; ///< Stable handle, never reused

    /**
     * @brief OSM way this segment belongs to
     *
     * Shared by every segment split out of one way, exactly as GraphEdge does
     * it, so splitting a street does not turn one street into two in an export.
     * Negative for a way authored here.
     */
    WayId source_way = 0;

    /**
     * @brief Ordered node references, at least 2, no two consecutive the same
     *
     * First and last are the endpoints; the rest are shape points. A closed ring
     * repeats its first id as its last, which is how OSM spells a loop.
     */
    std::vector<NodeId> node_ids;

    SegmentAttributes attributes;   ///< Per-way tags. Copied verbatim by a split.
};

// ============================================================================
// The graph
// ============================================================================

class GraphEditCommand;

/**
 * @brief An authored street network: nodes by identity, segments by handle
 *
 * Queries are public; every mutation is private and reachable only through a
 * Command. Not thread-safe, for the same reason CommandStack is not: edits come
 * from the UI thread, and a background job hands a command back rather than
 * pushing from its own.
 */
class EditableGraph {
public:
    EditableGraph() = default;

    /**
     * @brief Build an editable graph from a solved RoadGraph
     *
     * A load, not an edit: it returns a fresh graph, so there is nothing to
     * undo and no history to put a hole in. Pair it with CommandStack::clear().
     *
     * Every vertex of every GraphEdge becomes a node. Endpoints keep the graph's
     * own notion of identity -- two edges that RoadGraph joined at one GraphNode
     * get one node here, and two edges it kept apart stay apart -- so the layer
     * rule survives the trip. That matters in both directions and each has bitten
     * this codebase before:
     *
     *   - A bridge crossing over a road shares an OSM node id with it. RoadGraph
     *     splits that node per layer so the deck does not join the road beneath.
     *     Keying by osm_id alone here would merge them back into a junction.
     *   - A bridge deck meeting its approach also disagrees on layer=*, and there
     *     the two must stay ONE node or every bridge in the extract detaches from
     *     its ramps. RoadGraph already made that call; this copies its answer
     *     rather than second-guessing it from the tags.
     *
     * Where one osm_id has to become two nodes, one of them keeps it and the
     * others are re-keyed to fresh negative ids. Re-keying rather than
     * compound-keying is what keeps "two ways meet iff they share a node id"
     * exceptionless: a (id, layer) key would mean identity was sometimes a pair,
     * and every consumer would have to know which.
     *
     * **Which one keeps it is decided by the data, not by the order of the
     * ways.** The node with the most arms wins, ties go to the lowest layer=*.
     * Deciding it by iteration order instead -- first edge to reach the id keeps
     * it -- made the mapping from OSM node id to physical identity a property of
     * ParsedOSMData::roads: re-list the same two ways the other way round and the
     * id names the other grade. Every NodeId a selection, a rule or a saved
     * document held would then address a different piece of road after a reload,
     * which is the retargeting handle this file exists to prevent.
     *
     * Edges with fewer than two vertices, or whose node_ids are not parallel to
     * their polyline, are skipped; they carry no usable topology, which is the
     * same test RoadGraph::build() applies to a Road.
     *
     * @param graph A built RoadGraph
     * @return A graph with no dirty marks and both id counters primed
     */
    [[nodiscard]] static EditableGraph from_road_graph(const RoadGraph& graph);

    // ── Lookup ──────────────────────────────────────────────────────────────

    [[nodiscard]] bool contains_node(NodeId id) const;
    [[nodiscard]] bool contains_segment(SegmentId id) const;

    /**
     * @brief Look a node up
     *
     * @return nullptr when the node is gone or was never created. Because ids
     *         are never reused, a non-null result is always the node the caller
     *         meant. Hold the NodeId, not the pointer: the pointer survives
     *         edits to OTHER nodes but not the removal of this one.
     */
    [[nodiscard]] const EditableNode* find_node(NodeId id) const;

    /// @return nullptr when the segment is gone. See find_node() on lifetime.
    [[nodiscard]] const EditableSegment* find_segment(SegmentId id) const;

    [[nodiscard]] size_t node_count() const { return m_nodes.size(); }
    [[nodiscard]] size_t segment_count() const { return m_segments.size(); }

    /// Ordered by id, so iteration is stable rather than a hash's whim.
    [[nodiscard]] const std::map<NodeId, EditableNode>& nodes() const { return m_nodes; }
    [[nodiscard]] const std::map<SegmentId, EditableSegment>& segments() const {
        return m_segments;
    }

    // ── Topology, by identity ───────────────────────────────────────────────

    /**
     * @brief Segments that reference @p id anywhere, ascending, each listed once
     *
     * This is the junction relation, and it is computed from node ids alone.
     * Empty for an unknown node.
     */
    [[nodiscard]] const std::vector<SegmentId>& segments_at(NodeId id) const;

    /**
     * @brief Times @p id appears in any segment's node list
     *
     * A closed ring that begins and ends on the node counts twice, which is
     * correct: it arrives and it leaves.
     */
    [[nodiscard]] size_t reference_count(NodeId id) const;

    /**
     * @brief References that are the FIRST or LAST entry of a segment
     *
     * The same quantity as GraphNode::degree(). `arm_count() >= 3` is the test
     * for a junction that needs trims and a fillet ring; see
     * GraphNode::is_junction().
     */
    [[nodiscard]] size_t arm_count(NodeId id) const;

    /**
     * @brief Do two ways meet here?
     *
     * True when the node is referenced two or more times -- by two segments, or
     * twice by one closed segment. Pure identity: two coincident endpoints under
     * different node ids are NOT shared and this returns false for both.
     */
    [[nodiscard]] bool is_shared(NodeId id) const { return reference_count(id) >= 2; }

    /// Nodes no segment references. An authored point, or what a delete left behind.
    [[nodiscard]] std::vector<NodeId> orphan_nodes() const;

    // ── Geometry ────────────────────────────────────────────────────────────

    /**
     * @brief The segment's centreline, resolved from its nodes, in local metres
     *
     * Empty when the segment is unknown. A node id the table has lost is skipped
     * rather than faked, which cannot happen while the private mutators hold the
     * no-dangling-reference invariant, and is cheap to be sure of.
     */
    [[nodiscard]] std::vector<glm::dvec2> polyline(SegmentId id) const;

    /// Arc length in metres of polyline(). 0 for an unknown or single-point segment.
    [[nodiscard]] double length(SegmentId id) const;

    // ── Dirty tracking, for the incremental re-solve (B4) ───────────────────
    //
    // Marked by the private mutators, so BOTH apply() and revert() mark: an undo
    // changes the geometry exactly as much as the edit did, and a dirty set that
    // only tracked the forward direction would leave a stale solve on screen
    // after every undo. That is the bug this placement exists to prevent.

    /**
     * @brief Segments whose solved output is no longer valid
     *
     * Contains an id whose segment has since been deleted, on purpose: that is
     * how B4 learns to drop what it solved for it. Look the id up; absent means
     * "discard".
     *
     * A node's move marks every segment incident on it, not just the segments
     * whose shape changed -- a junction's geometry depends on all of its arms,
     * so moving one moves the fillet the others stop at.
     */
    [[nodiscard]] const std::set<SegmentId>& dirty_segments() const { return m_dirty_segments; }

    /// Nodes whose junction solve is no longer valid. Includes deleted ids.
    [[nodiscard]] const std::set<NodeId>& dirty_nodes() const { return m_dirty_nodes; }

    /// Forget what is dirty. B4 calls this once it has re-solved.
    void clear_dirty();

private:
    /**
     * @brief The one door into the mutating API
     *
     * GraphEditCommand forwards these to its subclasses as protected helpers, so
     * "every mutation goes through the command stack, and every mutation marks
     * dirty" is enforced by the compiler in exactly one place instead of being a
     * rule in a comment.
     */
    friend class GraphEditCommand;

    /// What the node table knows about one node's use, kept in step with node_ids
    struct NodeUsage {
        /// Ascending, each segment once, however many vertices of it use the node
        std::vector<SegmentId> segments;
        size_t references = 0;  ///< Total (segment, vertex) references
        size_t arms = 0;        ///< References that are a segment's first or last
    };

    /// Next locally-authored node id. Starts at -1 and only ever decreases.
    NodeId reserve_node_id();

    /// Next locally-authored way id. Independent counter: OSM's id spaces are.
    WayId reserve_way_id();

    /// Next segment handle. Monotonic from 1; never returns a value twice.
    SegmentId reserve_segment_id();

    /**
     * @brief Insert a fully-formed node, id included
     *
     * Takes the id from the record rather than allocating, because undo has to
     * put back the node that was deleted, not a new one that looks like it.
     *
     * @return false when the id is already present
     */
    bool insert_node(EditableNode node);

    /**
     * @brief Remove a node
     *
     * @return false when it is unknown, or when any segment still references it.
     *         A dangling node id would read to every consumer as a junction with
     *         a missing arm, so it is refused rather than tidied up afterwards.
     */
    bool erase_node(NodeId id);

    bool set_node_position(NodeId id, glm::dvec2 position);

    /// Insert a fully-formed segment, handle included. See insert_node() on why.
    bool insert_segment(EditableSegment segment);

    bool erase_segment(SegmentId id);

    /// Replace a segment's vertex list. Refuses a list that fails valid_nodes().
    bool set_segment_nodes(SegmentId id, std::vector<NodeId> node_ids);

    /// At least 2 entries, every one present, and no two consecutive the same.
    [[nodiscard]] bool valid_nodes(const std::vector<NodeId>& ids) const;

    void add_usage(SegmentId segment, const std::vector<NodeId>& ids);
    void remove_usage(SegmentId segment, const std::vector<NodeId>& ids);

    /// Mark the node AND every segment currently incident on it.
    void mark_node_dirty(NodeId id);
    void mark_segment_dirty(SegmentId id);

    std::map<NodeId, EditableNode> m_nodes;
    std::map<SegmentId, EditableSegment> m_segments;

    /// Derived from the node_ids of every segment. The reason the mutators are
    /// private: two directions of one link that only agree if one owner keeps them.
    std::map<NodeId, NodeUsage> m_usage;

    std::set<SegmentId> m_dirty_segments;
    std::set<NodeId> m_dirty_nodes;

    /// Never increases, not even when the node it was issued for is deleted.
    NodeId m_next_node_id = -1;
    WayId m_next_way_id = -1;

    /// Never decreases. 0 is kInvalidSegment, so handles start at 1.
    SegmentId m_next_segment_id = 1;
};

// ============================================================================
// Commands
// ============================================================================

/**
 * @brief Base for every street graph edit
 *
 * Owns the forwarding into EditableGraph's private API. A new operation
 * subclasses this and uses the protected helpers; it does not touch
 * EditableGraph directly, and it cannot -- GraphEditCommand is the graph's only
 * friend.
 */
class GraphEditCommand : public stratum::scene::Command {
protected:
    explicit GraphEditCommand(EditableGraph& graph) : m_graph(&graph) {}

    [[nodiscard]] EditableGraph& graph() const { return *m_graph; }

    /// True when @p other edits the same graph. The guard every merge() needs:
    /// without it, two documents open at once would coalesce each other's edits.
    [[nodiscard]] bool same_graph(const GraphEditCommand& other) const {
        return m_graph == other.m_graph;
    }

    // Forwarders into EditableGraph. See the matching private members there.

    /// Static, and takes the graph explicitly, because AddNodeCommand calls it
    /// from its own member-initialiser list -- before there is a
    /// GraphEditCommand to call it on.
    static NodeId reserve_node_id(EditableGraph& graph);
    static WayId reserve_way_id(EditableGraph& graph);
    static SegmentId reserve_segment_id(EditableGraph& graph);

    bool insert_node(EditableNode node);
    bool erase_node(NodeId id);
    bool set_node_position(NodeId id, glm::dvec2 position);
    bool insert_segment(EditableSegment segment);
    bool erase_segment(SegmentId id);
    bool set_segment_nodes(SegmentId id, std::vector<NodeId> node_ids);

    EditableGraph* m_graph;
};

/**
 * @brief Create a node
 *
 * The id is taken in the constructor, so a caller can create several nodes and
 * the segment joining them inside one transaction. A command that is built and
 * never executed burns an id, which costs nothing: the space is 63 bits wide and
 * ids were never going to be reused anyway.
 */
class AddNodeCommand : public GraphEditCommand {
public:
    AddNodeCommand(EditableGraph& graph, glm::dvec2 position, int layer = 0);

    /// Readable before execute(). Negative, and distinct from every id in the graph.
    [[nodiscard]] NodeId node() const { return m_node.id; }

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

private:
    EditableNode m_node;
};

/**
 * @brief Move a node, and with it every segment that shares the node
 *
 * Merges with a later move of the same node, so a drag is one undo step rather
 * than one per mouse-move; CommandStack::seal() on mouse-up ends it.
 */
class MoveNodeCommand : public GraphEditCommand {
public:
    MoveNodeCommand(EditableGraph& graph, NodeId node, glm::dvec2 position);

    [[nodiscard]] NodeId node() const { return m_node; }

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] bool merge(const stratum::scene::Command& next) override;

private:
    NodeId m_node;
    glm::dvec2 m_position;
    glm::dvec2 m_old_position{0.0};
};

/**
 * @brief Delete a node
 *
 * **Refused when the node is shared** -- referenced twice or more, whether by
 * two segments or by one closed ring. Cascading instead would delete streets the
 * user did not point at; retracting each way by a vertex instead, which is what
 * JOSM does, is worse still: four roads that met at one id would end at four
 * different ids a metre apart, the junction would silently cease to exist, and
 * because junctions are identity-based nothing downstream would ever rejoin
 * them. A refusal is visible, and CommandStack::execute() is built for it --
 * it records nothing and leaves the history exactly as it was.
 *
 * Removing a junction is spelled as what it is, inside one transaction:
 * @code
 *     stack.begin_transaction("Delete junction");
 *     for (SegmentId s : graph.segments_at(id)) {
 *         stack.execute(std::make_unique<DeleteSegmentCommand>(graph, s));
 *     }
 *     stack.execute(std::make_unique<DeleteNodeCommand>(graph, id));
 *     stack.commit_transaction();
 * @endcode
 *
 * For a node only one segment uses, the segment survives: an interior vertex is
 * dropped and the line shortcuts across it, an endpoint is dropped and the line
 * retracts. When that leaves fewer than two distinct vertices there is no line
 * left, and the segment goes with the node -- a cascade, but a bounded one the
 * user can see the whole of.
 */
class DeleteNodeCommand : public GraphEditCommand {
public:
    DeleteNodeCommand(EditableGraph& graph, NodeId node);

    [[nodiscard]] NodeId node() const { return m_node; }

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;

private:
    NodeId m_node;
    EditableNode m_node_copy;

    /// The one segment that referenced the node, verbatim, or an empty handle.
    EditableSegment m_segment_copy;

    /// True when the segment went too, so revert() knows to put it back whole
    /// rather than to restore its vertex list.
    bool m_segment_removed = false;
};

/**
 * @brief Create a segment through existing nodes
 *
 * The handle and the way id are taken in the constructor, so a caller can build
 * a street and set its attributes in one transaction, and so a redo lands on the
 * same handle the first apply() issued.
 *
 * Refused for fewer than two nodes, for a node the graph does not have, and for
 * a repeated consecutive node -- a zero-length step has no direction, and an arm
 * with no direction is exactly what the junction solver cannot use.
 */
class AddSegmentCommand : public GraphEditCommand {
public:
    AddSegmentCommand(EditableGraph& graph, std::vector<NodeId> node_ids,
                      SegmentAttributes attributes = {});

    /// Readable before execute().
    [[nodiscard]] SegmentId segment() const { return m_segment.id; }

    /// The negative WayId allocated for this street. Both halves of a later
    /// split keep it.
    [[nodiscard]] WayId way() const { return m_segment.source_way; }

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;

private:
    EditableSegment m_segment;
};

/**
 * @brief Delete a segment
 *
 * Its nodes STAY. A node is an identity, possibly shared with streets that are
 * not being deleted and possibly one the user placed deliberately, so removing
 * it as a side effect would kill handles the caller is still holding. An
 * unreferenced node is harmless -- OSM is full of them -- and `orphan_nodes()`
 * lists them for a caller that wants to tidy up as its own, undoable, step.
 */
class DeleteSegmentCommand : public GraphEditCommand {
public:
    DeleteSegmentCommand(EditableGraph& graph, SegmentId segment);

    [[nodiscard]] SegmentId segment() const { return m_segment; }

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;

private:
    SegmentId m_segment;
    EditableSegment m_copy;  ///< Verbatim, handle included. The undo state.
};

/**
 * @brief Cut a segment in two at the point on it nearest @p point
 *
 * The original segment is removed and replaced by two new ones. Keeping the
 * original handle for one half was the alternative and it loses: a caller
 * holding that handle would afterwards be pointing at half the street it thought
 * it had, with no way to detect the change, while a removed handle reports
 * `contains_segment() == false` and can be dealt with.
 *
 * Both halves keep the original `source_way` and a copy of every attribute. They
 * are two segments of one street, not two streets, which is exactly what
 * RoadGraph produces when it splits a way at a junction.
 *
 * The join is a node, so the halves meet by identity like anything else:
 * `is_shared()` on it is true and `arm_count()` is 2. Where the split lands on
 * an existing shape point -- within @p snap -- that node is used and no new one
 * is created.
 *
 * Refused when the nearest point is an endpoint, because one half would have no
 * length; see kSplitSnapMetres.
 *
 * Both halves' handles and the new node's id are reserved in the constructor, so
 * undo followed by redo reproduces the same handles rather than issuing fresh
 * ones that every caller's stored id would have gone stale against.
 */
class SplitSegmentCommand : public GraphEditCommand {
public:
    /**
     * @param point Anywhere near the segment, in local metres; it is projected
     *              onto the centreline rather than used as given
     * @param snap  Distance within which an existing vertex is reused instead
     */
    SplitSegmentCommand(EditableGraph& graph, SegmentId segment, glm::dvec2 point,
                        double snap = kSplitSnapMetres);

    /// Readable before execute(); valid only once apply() has returned true.
    [[nodiscard]] SegmentId first_half() const { return m_first; }
    [[nodiscard]] SegmentId second_half() const { return m_second; }

    /**
     * @brief The node the halves meet at
     *
     * The id reserved in the constructor, or -- when the split snapped to an
     * existing shape point -- that node's id. Only meaningful after a successful
     * apply().
     */
    [[nodiscard]] NodeId join_node() const { return m_join; }

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;

private:
    SegmentId m_segment;
    glm::dvec2 m_point;
    double m_snap;

    SegmentId m_first = kInvalidSegment;
    SegmentId m_second = kInvalidSegment;

    /// Reserved in the constructor; replaced by an existing id when snapped.
    NodeId m_join = kInvalidNode;

    /// The id reserved for a new join node, kept so a redo after an undo that
    /// snapped differently cannot start handing out fresh ids every cycle.
    NodeId m_reserved_join = kInvalidNode;

    /// True when apply() created the join node, so revert() knows to remove it.
    bool m_created_join = false;

    EditableSegment m_original;  ///< Verbatim, handle included. The undo state.
};

} // namespace stratum::osm::road
