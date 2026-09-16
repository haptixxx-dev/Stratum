// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_graph_edit.cpp
 * @brief EditableGraph and its commands: identity, undo, and the dirty set
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Every test here is written so that inverting the behaviour it names makes it
 * fail. The three that matter most, and what each one would catch:
 *
 *   - `two_ways_meet_by_identity_and_never_by_position` puts two nodes on
 *     exactly the same coordinate under different ids. Any implementation that
 *     joins ways by proximity reads that as a junction and fails.
 *   - `a_drag_that_returns_to_its_start_leaves_no_no_op_step` runs the whole
 *     undo/redo cycle over a round-trip drag. A merge that absorbed the return
 *     leg produces a step whose redo refuses, and CommandStack drops it; the
 *     second `redo()` then returns false.
 *   - `from_a_road_graph_a_grade_separation_stays_two_nodes` seeds from a bridge
 *     crossing a road. Keying the load by OSM id alone merges the crossing into
 *     one node, and the arm count goes from 2 to 4.
 *
 * Coordinates are deliberately asymmetric and splits deliberately off-centre, so
 * a mistake that happens to commute -- a midpoint split, a swapped x and y -- has
 * nowhere to hide.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests GraphEdit
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/graph_edit.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"
#include "scene/command.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using stratum::osm::NodeId;
using stratum::osm::ParsedOSMData;
using stratum::osm::Road;
using stratum::osm::RoadType;
using stratum::osm::SideFlags;
using stratum::osm::WayId;
using stratum::osm::road::AddNodeCommand;
using stratum::osm::road::AddSegmentCommand;
using stratum::osm::road::DeleteNodeCommand;
using stratum::osm::road::DeleteSegmentCommand;
using stratum::osm::road::EditableGraph;
using stratum::osm::road::kInvalidNode;
using stratum::osm::road::kInvalidSegment;
using stratum::osm::road::MoveNodeCommand;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::SegmentAttributes;
using stratum::osm::road::SegmentId;
using stratum::osm::road::SplitSegmentCommand;
using stratum::scene::CommandStack;

/// Tolerance for a coordinate that should be reproduced exactly, not solved for.
constexpr double kExact = 1e-12;

// ============================================================================
// Helpers
// ============================================================================

/// Create a node through the stack. Returns kInvalidNode when the edit refused.
NodeId make_node(EditableGraph& graph, CommandStack& stack, double x, double y) {
    auto command = std::make_unique<AddNodeCommand>(graph, glm::dvec2(x, y));
    const NodeId id = command->node();
    return stack.execute(std::move(command)) ? id : kInvalidNode;
}

/// Create a segment through the stack. Returns kInvalidSegment when it refused.
SegmentId make_segment(EditableGraph& graph, CommandStack& stack, std::vector<NodeId> ids,
                       SegmentAttributes attributes = {}) {
    auto command =
        std::make_unique<AddSegmentCommand>(graph, std::move(ids), std::move(attributes));
    const SegmentId id = command->segment();
    return stack.execute(std::move(command)) ? id : kInvalidSegment;
}

bool move_node(EditableGraph& graph, CommandStack& stack, NodeId node, double x, double y) {
    return stack.execute(std::make_unique<MoveNodeCommand>(graph, node, glm::dvec2(x, y)));
}

/// Position of a node, or a far-away sentinel when it is gone, so a check against
/// a real coordinate fails loudly instead of dereferencing nothing.
glm::dvec2 position_of(const EditableGraph& graph, NodeId node) {
    const auto* record = graph.find_node(node);
    return record == nullptr ? glm::dvec2(1e18, 1e18) : record->position;
}

/**
 * @brief A three-armed junction: west, east and north meeting at one node
 *
 * The hub sits at (0, 5) and the arms end at distinct, asymmetric coordinates,
 * so a bug that moves the wrong endpoint cannot land on the right number.
 */
struct Hub {
    NodeId west = kInvalidNode;
    NodeId hub = kInvalidNode;
    NodeId east = kInvalidNode;
    NodeId north = kInvalidNode;
    SegmentId west_arm = kInvalidSegment;
    SegmentId east_arm = kInvalidSegment;
    SegmentId north_arm = kInvalidSegment;
};

Hub build_hub(EditableGraph& graph, CommandStack& stack) {
    Hub h;
    h.west = make_node(graph, stack, -30.0, 5.0);
    h.hub = make_node(graph, stack, 0.0, 5.0);
    h.east = make_node(graph, stack, 40.0, 5.0);
    h.north = make_node(graph, stack, 0.0, 25.0);
    h.west_arm = make_segment(graph, stack, {h.west, h.hub});
    h.east_arm = make_segment(graph, stack, {h.hub, h.east});
    h.north_arm = make_segment(graph, stack, {h.hub, h.north});
    return h;
}

/// A Road ready for RoadGraph::build(), with node_ids parallel to the polyline.
Road make_road(WayId way, std::vector<NodeId> nodes, std::vector<glm::dvec2> points,
               int layer = 0) {
    Road road;
    road.osm_id = way;
    road.node_ids = std::move(nodes);
    road.polyline = std::move(points);
    road.layer = layer;
    road.type = RoadType::Residential;
    return road;
}

/// Build a RoadGraph over @p roads and seed an EditableGraph from it.
EditableGraph seed_from(std::vector<Road> roads) {
    ParsedOSMData data;
    data.roads = std::move(roads);

    RoadGraph graph;
    graph.build(data);
    return EditableGraph::from_road_graph(graph);
}

} // namespace

// ============================================================================
// Identity
// ============================================================================

TEST(GraphEdit, authored_nodes_take_negative_ids_that_no_osm_node_can_have) {
    EditableGraph graph;
    CommandStack stack;

    const NodeId first = make_node(graph, stack, 3.0, 7.0);
    const NodeId second = make_node(graph, stack, 11.0, -4.0);

    // OSM element ids are positive int64, so a negative id can never collide
    // with one however the graph is later seeded.
    CHECK((first < 0));
    CHECK((second < 0));
    CHECK((first != second));
    CHECK_EQ(graph.node_count(), size_t{2});
    CHECK_NEAR(position_of(graph, second).x, 11.0, kExact);
    CHECK_NEAR(position_of(graph, second).y, -4.0, kExact);
}

TEST(GraphEdit, two_ways_meet_by_identity_and_never_by_position) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);

    CHECK_EQ(graph.arm_count(h.hub), size_t{3});
    CHECK_EQ(graph.reference_count(h.hub), size_t{3});
    CHECK_TRUE(graph.is_shared(h.hub));
    CHECK_EQ(graph.segments_at(h.hub).size(), size_t{3});

    // A fourth street ending on EXACTLY the hub's coordinate, under an identity
    // of its own. It does not join. Proximity clustering would say it does, and
    // that is the whole reason junctions are keyed on node ids here.
    const NodeId twin = make_node(graph, stack, 0.0, 5.0);
    const NodeId south = make_node(graph, stack, 0.0, -20.0);
    const SegmentId south_arm = make_segment(graph, stack, {twin, south});

    CHECK((twin != h.hub));
    CHECK_NEAR(position_of(graph, twin).x, position_of(graph, h.hub).x, kExact);
    CHECK_NEAR(position_of(graph, twin).y, position_of(graph, h.hub).y, kExact);

    CHECK_EQ(graph.arm_count(h.hub), size_t{3});
    CHECK_EQ(graph.segments_at(h.hub).size(), size_t{3});
    CHECK_EQ(graph.arm_count(twin), size_t{1});
    CHECK_FALSE(graph.is_shared(twin));
    CHECK_EQ(graph.segments_at(twin).size(), size_t{1});
    CHECK_EQ(graph.segments_at(twin).front(), south_arm);
}

TEST(GraphEdit, a_coincident_node_is_not_dragged_along_by_its_twin) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);
    const NodeId twin = make_node(graph, stack, 0.0, 5.0);
    const NodeId south = make_node(graph, stack, 0.0, -20.0);
    const SegmentId south_arm = make_segment(graph, stack, {twin, south});

    CHECK_TRUE(move_node(graph, stack, h.hub, 17.0, -6.0));

    // The twin stayed where it was, because it was never the same node.
    CHECK_NEAR(position_of(graph, twin).x, 0.0, kExact);
    CHECK_NEAR(position_of(graph, twin).y, 5.0, kExact);

    const std::vector<glm::dvec2> south_line = graph.polyline(south_arm);
    CHECK_EQ(south_line.size(), size_t{2});
    if (south_line.size() == 2) {
        CHECK_NEAR(south_line[0].x, 0.0, kExact);
        CHECK_NEAR(south_line[0].y, 5.0, kExact);
    }
}

TEST(GraphEdit, an_id_is_not_reused_after_the_node_it_named_is_deleted) {
    EditableGraph graph;
    CommandStack stack;

    const NodeId first = make_node(graph, stack, 1.0, 2.0);
    CHECK_TRUE(stack.execute(std::make_unique<DeleteNodeCommand>(graph, first)));
    CHECK_FALSE(graph.contains_node(first));

    const NodeId second = make_node(graph, stack, 1.0, 2.0);
    // Same position, and the old id is free. It is still not handed out again:
    // a recycled id would make a stale handle silently name a different node.
    CHECK((second != first));
    CHECK_FALSE(graph.contains_node(first));
    CHECK_TRUE(graph.contains_node(second));
}

// ============================================================================
// Add and undo
// ============================================================================

TEST(GraphEdit, undo_removes_an_added_node_and_redo_restores_the_same_id) {
    EditableGraph graph;
    CommandStack stack;

    const NodeId node = make_node(graph, stack, 8.0, -13.0);
    CHECK_TRUE(graph.contains_node(node));

    CHECK_TRUE(stack.undo());
    CHECK_FALSE(graph.contains_node(node));
    CHECK_EQ(graph.node_count(), size_t{0});

    CHECK_TRUE(stack.redo());
    // The SAME id, not a fresh one: a redo that reallocated would leave every
    // handle taken before the undo pointing at nothing.
    CHECK_TRUE(graph.contains_node(node));
    CHECK_EQ(graph.node_count(), size_t{1});
    CHECK_NEAR(position_of(graph, node).x, 8.0, kExact);
    CHECK_NEAR(position_of(graph, node).y, -13.0, kExact);
}

TEST(GraphEdit, add_segment_refuses_a_list_that_carries_no_topology) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId b = make_node(graph, stack, 25.0, 9.0);
    const size_t depth = stack.undo_depth();

    // One node is not a line.
    CHECK_EQ(make_segment(graph, stack, {a}), kInvalidSegment);
    // A node the graph has never heard of. Accepting it would leave a dangling
    // reference that reads downstream as a junction with a missing arm.
    CHECK_EQ(make_segment(graph, stack, {a, NodeId{-999999}}), kInvalidSegment);
    // A repeated consecutive node is a zero-length step, and an arm with no
    // direction is exactly what the junction solver cannot use.
    CHECK_EQ(make_segment(graph, stack, {a, a}), kInvalidSegment);
    CHECK_EQ(make_segment(graph, stack, {a, b, b}), kInvalidSegment);

    CHECK_EQ(graph.segment_count(), size_t{0});
    CHECK_EQ(stack.undo_depth(), depth);
    CHECK_EQ(graph.arm_count(a), size_t{0});
}

TEST(GraphEdit, authored_ways_take_distinct_negative_way_ids) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId b = make_node(graph, stack, 30.0, 0.0);
    const NodeId c = make_node(graph, stack, 30.0, 40.0);

    auto first = std::make_unique<AddSegmentCommand>(graph, std::vector<NodeId>{a, b});
    const WayId first_way = first->way();
    CHECK_TRUE(stack.execute(std::move(first)));

    auto second = std::make_unique<AddSegmentCommand>(graph, std::vector<NodeId>{b, c});
    const WayId second_way = second->way();
    CHECK_TRUE(stack.execute(std::move(second)));

    CHECK((first_way < 0));
    CHECK((second_way < 0));
    CHECK((first_way != second_way));
}

TEST(GraphEdit, a_node_and_the_segment_through_it_undo_as_one_transaction) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, -12.0, 3.0);
    stack.seal();
    const size_t depth = stack.undo_depth();

    stack.begin_transaction("Draw street");
    auto node_command = std::make_unique<AddNodeCommand>(graph, glm::dvec2(45.0, 18.0));
    // Readable before execute(): this is what lets the segment be built in the
    // same transaction as the node it runs to.
    const NodeId b = node_command->node();
    CHECK_TRUE(stack.execute(std::move(node_command)));
    const SegmentId road = make_segment(graph, stack, {a, b});
    stack.commit_transaction();

    CHECK((road != kInvalidSegment));
    CHECK_EQ(stack.undo_depth(), depth + 1);

    CHECK_TRUE(stack.undo());
    CHECK_FALSE(graph.contains_segment(road));
    CHECK_FALSE(graph.contains_node(b));
    CHECK_TRUE(graph.contains_node(a));
    CHECK_EQ(stack.undo_depth(), depth);
}

// ============================================================================
// Moving a node
// ============================================================================

TEST(GraphEdit, moving_a_shared_node_moves_every_segment_on_it) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);

    CHECK_TRUE(move_node(graph, stack, h.hub, 6.0, -2.0));

    const std::vector<glm::dvec2> west = graph.polyline(h.west_arm);
    const std::vector<glm::dvec2> east = graph.polyline(h.east_arm);
    const std::vector<glm::dvec2> north = graph.polyline(h.north_arm);
    CHECK_EQ(west.size(), size_t{2});
    CHECK_EQ(east.size(), size_t{2});
    CHECK_EQ(north.size(), size_t{2});
    if (west.size() != 2 || east.size() != 2 || north.size() != 2) return;

    // The hub end of all three moved ...
    CHECK_NEAR(west[1].x, 6.0, kExact);
    CHECK_NEAR(west[1].y, -2.0, kExact);
    CHECK_NEAR(east[0].x, 6.0, kExact);
    CHECK_NEAR(east[0].y, -2.0, kExact);
    CHECK_NEAR(north[0].x, 6.0, kExact);
    CHECK_NEAR(north[0].y, -2.0, kExact);

    // ... and nothing else did.
    CHECK_NEAR(west[0].x, -30.0, kExact);
    CHECK_NEAR(east[1].x, 40.0, kExact);
    CHECK_NEAR(north[1].y, 25.0, kExact);

    CHECK_TRUE(stack.undo());

    const std::vector<glm::dvec2> west_back = graph.polyline(h.west_arm);
    const std::vector<glm::dvec2> east_back = graph.polyline(h.east_arm);
    const std::vector<glm::dvec2> north_back = graph.polyline(h.north_arm);
    CHECK_NEAR(west_back[1].x, 0.0, kExact);
    CHECK_NEAR(west_back[1].y, 5.0, kExact);
    CHECK_NEAR(east_back[0].x, 0.0, kExact);
    CHECK_NEAR(east_back[0].y, 5.0, kExact);
    CHECK_NEAR(north_back[0].x, 0.0, kExact);
    CHECK_NEAR(north_back[0].y, 5.0, kExact);
}

TEST(GraphEdit, a_move_to_where_the_node_already_is_records_nothing) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId node = make_node(graph, stack, 4.0, 4.0);
    const size_t depth = stack.undo_depth();

    CHECK_FALSE(move_node(graph, stack, node, 4.0, 4.0));
    CHECK_EQ(stack.undo_depth(), depth);
    CHECK_FALSE(stack.can_redo());
}

TEST(GraphEdit, a_drag_collapses_into_one_undo_step) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId node = make_node(graph, stack, 2.0, -3.0);
    stack.seal();
    const size_t depth = stack.undo_depth();

    // One command per mouse-move, the way a drag tool emits them.
    for (int i = 1; i <= 5; ++i) {
        CHECK_TRUE(move_node(graph, stack, node, 2.0 + i * 7.0, -3.0 + i * 1.5));
    }

    CHECK_EQ(stack.undo_depth(), depth + 1);
    CHECK_NEAR(position_of(graph, node).x, 37.0, kExact);

    // One undo takes the whole gesture back, not one pixel of it.
    CHECK_TRUE(stack.undo());
    CHECK_NEAR(position_of(graph, node).x, 2.0, kExact);
    CHECK_NEAR(position_of(graph, node).y, -3.0, kExact);
    CHECK_EQ(stack.undo_depth(), depth);
}

TEST(GraphEdit, seal_ends_the_drag) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId node = make_node(graph, stack, 0.0, 0.0);
    stack.seal();
    const size_t depth = stack.undo_depth();

    CHECK_TRUE(move_node(graph, stack, node, 10.0, 1.0));
    stack.seal();  // mouse-up
    CHECK_TRUE(move_node(graph, stack, node, 20.0, 2.0));

    CHECK_EQ(stack.undo_depth(), depth + 2);
}

TEST(GraphEdit, moves_of_two_different_nodes_do_not_merge) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId b = make_node(graph, stack, 50.0, 0.0);
    stack.seal();
    const size_t depth = stack.undo_depth();

    CHECK_TRUE(move_node(graph, stack, a, 3.0, 3.0));
    CHECK_TRUE(move_node(graph, stack, b, 53.0, 3.0));

    // Merging on "the previous command was also a move" would give one step and
    // an undo that put only one of the two nodes back.
    CHECK_EQ(stack.undo_depth(), depth + 2);
    CHECK_TRUE(stack.undo());
    CHECK_NEAR(position_of(graph, b).x, 50.0, kExact);
    CHECK_NEAR(position_of(graph, a).x, 3.0, kExact);
}

TEST(GraphEdit, a_drag_that_returns_to_its_start_leaves_no_no_op_step) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId node = make_node(graph, stack, 2.0, -3.0);
    stack.seal();
    const size_t depth = stack.undo_depth();

    CHECK_TRUE(move_node(graph, stack, node, 9.0, 4.0));
    CHECK_TRUE(move_node(graph, stack, node, 2.0, -3.0));

    // Two real steps, not one step that changes nothing. Absorbing the return
    // leg would leave a command whose old and new positions are equal: its undo
    // does nothing visible and its redo refuses, at which point CommandStack
    // drops the step and the rest of the redo branch with it.
    CHECK_EQ(stack.undo_depth(), depth + 2);

    CHECK_TRUE(stack.undo());
    CHECK_NEAR(position_of(graph, node).x, 9.0, kExact);
    CHECK_NEAR(position_of(graph, node).y, 4.0, kExact);

    CHECK_TRUE(stack.undo());
    CHECK_NEAR(position_of(graph, node).x, 2.0, kExact);
    CHECK_NEAR(position_of(graph, node).y, -3.0, kExact);

    // Both steps redo. This is the half that catches the A1 bug: a dropped step
    // makes the second redo() return false.
    CHECK_TRUE(stack.redo());
    CHECK_NEAR(position_of(graph, node).x, 9.0, kExact);
    CHECK_TRUE(stack.redo());
    CHECK_NEAR(position_of(graph, node).x, 2.0, kExact);
    CHECK_NEAR(position_of(graph, node).y, -3.0, kExact);
    CHECK_FALSE(stack.can_redo());
}

// ============================================================================
// The dirty set
// ============================================================================

TEST(GraphEdit, a_move_marks_every_segment_on_the_node_and_nothing_else) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);

    // A street nowhere near the hub and sharing no node with it.
    const NodeId far_a = make_node(graph, stack, 500.0, 500.0);
    const NodeId far_b = make_node(graph, stack, 560.0, 520.0);
    const SegmentId far = make_segment(graph, stack, {far_a, far_b});

    graph.clear_dirty();
    CHECK_TRUE(graph.dirty_segments().empty());
    CHECK_TRUE(graph.dirty_nodes().empty());

    CHECK_TRUE(move_node(graph, stack, h.hub, 6.0, -2.0));

    // All three arms, because a junction's geometry depends on all of them.
    CHECK_EQ(graph.dirty_segments().count(h.west_arm), size_t{1});
    CHECK_EQ(graph.dirty_segments().count(h.east_arm), size_t{1});
    CHECK_EQ(graph.dirty_segments().count(h.north_arm), size_t{1});
    CHECK_EQ(graph.dirty_nodes().count(h.hub), size_t{1});

    // And nothing else, or the incremental re-solve is a full re-solve.
    CHECK_EQ(graph.dirty_segments().count(far), size_t{0});
    CHECK_EQ(graph.dirty_segments().size(), size_t{3});
    CHECK_EQ(graph.dirty_nodes().size(), size_t{1});
}

TEST(GraphEdit, undoing_a_move_marks_dirty_again) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);

    CHECK_TRUE(move_node(graph, stack, h.hub, 6.0, -2.0));
    graph.clear_dirty();

    CHECK_TRUE(stack.undo());

    // An undo changes the geometry exactly as much as the edit did. A dirty set
    // maintained by the commands rather than by the graph would be empty here,
    // and the stale solve would stay on screen after every undo.
    CHECK_EQ(graph.dirty_segments().count(h.west_arm), size_t{1});
    CHECK_EQ(graph.dirty_segments().count(h.east_arm), size_t{1});
    CHECK_EQ(graph.dirty_segments().count(h.north_arm), size_t{1});
    CHECK_EQ(graph.dirty_nodes().count(h.hub), size_t{1});
}

TEST(GraphEdit, adding_a_segment_marks_what_it_now_meets) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId west = make_node(graph, stack, -30.0, 5.0);
    const NodeId hub = make_node(graph, stack, 0.0, 5.0);
    const NodeId east = make_node(graph, stack, 40.0, 5.0);
    const NodeId north = make_node(graph, stack, 0.0, 25.0);
    const SegmentId west_arm = make_segment(graph, stack, {west, hub});
    const SegmentId east_arm = make_segment(graph, stack, {hub, east});

    const NodeId far_a = make_node(graph, stack, 500.0, 500.0);
    const NodeId far_b = make_node(graph, stack, 560.0, 520.0);
    const SegmentId far = make_segment(graph, stack, {far_a, far_b});

    graph.clear_dirty();
    const SegmentId north_arm = make_segment(graph, stack, {hub, north});
    CHECK((north_arm != kInvalidSegment));

    // The new arm, and the two that now have to stop short of a junction that
    // did not exist a moment ago.
    CHECK_EQ(graph.dirty_segments().count(north_arm), size_t{1});
    CHECK_EQ(graph.dirty_segments().count(west_arm), size_t{1});
    CHECK_EQ(graph.dirty_segments().count(east_arm), size_t{1});
    CHECK_EQ(graph.dirty_segments().count(far), size_t{0});
    CHECK_EQ(graph.dirty_nodes().count(hub), size_t{1});
    CHECK_EQ(graph.dirty_nodes().count(north), size_t{1});
    CHECK_EQ(graph.dirty_nodes().count(far_a), size_t{0});
}

TEST(GraphEdit, deleting_a_segment_marks_it_so_the_solver_can_drop_it) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);
    graph.clear_dirty();

    CHECK_TRUE(stack.execute(std::make_unique<DeleteSegmentCommand>(graph, h.west_arm)));

    // Its own id is in the set although it no longer exists: that is how B4
    // learns to throw away what it had solved for it.
    CHECK_EQ(graph.dirty_segments().count(h.west_arm), size_t{1});
    CHECK_FALSE(graph.contains_segment(h.west_arm));
    // And the arms it left behind, whose junction just lost a leg.
    CHECK_EQ(graph.dirty_segments().count(h.east_arm), size_t{1});
    CHECK_EQ(graph.dirty_segments().count(h.north_arm), size_t{1});
}

// ============================================================================
// Refusals
// ============================================================================

TEST(GraphEdit, delete_node_is_refused_where_two_ways_meet_and_changes_nothing) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);
    graph.clear_dirty();
    const size_t depth = stack.undo_depth();

    CHECK_FALSE(stack.execute(std::make_unique<DeleteNodeCommand>(graph, h.hub)));

    // Refusing is not a partial edit. The graph, the history and the dirty set
    // all have to look exactly as they did.
    CHECK_TRUE(graph.contains_node(h.hub));
    CHECK_EQ(graph.node_count(), size_t{4});
    CHECK_EQ(graph.segment_count(), size_t{3});
    CHECK_EQ(graph.arm_count(h.hub), size_t{3});
    CHECK_EQ(stack.undo_depth(), depth);
    CHECK_TRUE(graph.dirty_segments().empty());
    CHECK_TRUE(graph.dirty_nodes().empty());
}

TEST(GraphEdit, split_at_an_endpoint_is_refused) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId b = make_node(graph, stack, 100.0, 0.0);
    const SegmentId road = make_segment(graph, stack, {a, b});
    const size_t depth = stack.undo_depth();

    // Both ends. A half of zero length has no direction, and road_graph.cpp
    // drops an edge like that outright, so the street would silently lose a piece.
    CHECK_FALSE(stack.execute(
        std::make_unique<SplitSegmentCommand>(graph, road, glm::dvec2(-10.0, 0.0))));
    CHECK_FALSE(stack.execute(
        std::make_unique<SplitSegmentCommand>(graph, road, glm::dvec2(105.0, 3.0))));

    CHECK_TRUE(graph.contains_segment(road));
    CHECK_EQ(graph.segment_count(), size_t{1});
    CHECK_EQ(graph.node_count(), size_t{2});
    CHECK_EQ(stack.undo_depth(), depth);
}

// ============================================================================
// Deleting
// ============================================================================

TEST(GraphEdit, deleting_an_interior_node_shortcuts_the_segment) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId m = make_node(graph, stack, 40.0, 20.0);
    const NodeId b = make_node(graph, stack, 100.0, 0.0);
    const SegmentId road = make_segment(graph, stack, {a, m, b});

    CHECK_TRUE(stack.execute(std::make_unique<DeleteNodeCommand>(graph, m)));

    CHECK_FALSE(graph.contains_node(m));
    CHECK_TRUE(graph.contains_segment(road));
    const auto* after = graph.find_segment(road);
    CHECK_TRUE(after != nullptr);
    if (after == nullptr) return;
    CHECK_EQ(after->node_ids.size(), size_t{2});
    CHECK_EQ(after->node_ids.front(), a);
    CHECK_EQ(after->node_ids.back(), b);
    CHECK_NEAR(graph.length(road), 100.0, 1e-9);

    CHECK_TRUE(stack.undo());
    CHECK_TRUE(graph.contains_node(m));
    const auto* restored = graph.find_segment(road);
    CHECK_TRUE(restored != nullptr);
    if (restored == nullptr) return;
    CHECK_EQ(restored->node_ids.size(), size_t{3});
    CHECK_EQ(restored->node_ids[1], m);
    CHECK_NEAR(position_of(graph, m).x, 40.0, kExact);
    CHECK_NEAR(position_of(graph, m).y, 20.0, kExact);
}

TEST(GraphEdit, deleting_an_endpoint_retracts_a_segment_that_still_has_a_line) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId m = make_node(graph, stack, 40.0, 20.0);
    const NodeId b = make_node(graph, stack, 100.0, 0.0);
    const SegmentId road = make_segment(graph, stack, {a, m, b});

    CHECK_TRUE(stack.execute(std::make_unique<DeleteNodeCommand>(graph, b)));

    CHECK_TRUE(graph.contains_segment(road));
    const auto* after = graph.find_segment(road);
    CHECK_TRUE(after != nullptr);
    if (after == nullptr) return;
    CHECK_EQ(after->node_ids.size(), size_t{2});
    CHECK_EQ(after->node_ids.back(), m);
    // The node that is now the end carries an arm it did not have before.
    CHECK_EQ(graph.arm_count(m), size_t{1});
    CHECK_EQ(graph.reference_count(m), size_t{1});
}

TEST(GraphEdit, deleting_an_endpoint_of_a_two_node_segment_takes_the_segment_with_it) {
    EditableGraph graph;
    CommandStack stack;

    SegmentAttributes attributes;
    attributes.name = "Ship Street Little";
    attributes.lanes = 3;

    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId b = make_node(graph, stack, 70.0, 12.0);
    const SegmentId road = make_segment(graph, stack, {a, b}, attributes);

    CHECK_TRUE(stack.execute(std::make_unique<DeleteNodeCommand>(graph, b)));

    CHECK_FALSE(graph.contains_node(b));
    CHECK_FALSE(graph.contains_segment(road));
    // The other end is not swept up with it: only the node that was pointed at
    // and the line that could no longer exist go.
    CHECK_TRUE(graph.contains_node(a));
    CHECK_EQ(graph.node_count(), size_t{1});

    CHECK_TRUE(stack.undo());
    CHECK_TRUE(graph.contains_node(b));
    CHECK_TRUE(graph.contains_segment(road));
    const auto* restored = graph.find_segment(road);
    CHECK_TRUE(restored != nullptr);
    if (restored == nullptr) return;
    CHECK_EQ(restored->node_ids.size(), size_t{2});
    CHECK_EQ(restored->node_ids.back(), b);
    CHECK_TRUE(restored->attributes == attributes);
    CHECK_EQ(graph.arm_count(b), size_t{1});
    CHECK_NEAR(position_of(graph, b).x, 70.0, kExact);
}

TEST(GraphEdit, deleting_a_segment_leaves_its_nodes_behind) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);

    CHECK_TRUE(stack.execute(std::make_unique<DeleteSegmentCommand>(graph, h.west_arm)));

    CHECK_EQ(graph.segment_count(), size_t{2});
    // Nodes stay. One of them is shared with two streets that are still there,
    // and the other is now an orphan the caller may still be holding a handle to.
    CHECK_EQ(graph.node_count(), size_t{4});
    CHECK_TRUE(graph.contains_node(h.west));
    CHECK_EQ(graph.arm_count(h.hub), size_t{2});
    CHECK_EQ(graph.segments_at(h.hub).size(), size_t{2});

    const std::vector<NodeId> orphans = graph.orphan_nodes();
    CHECK_EQ(orphans.size(), size_t{1});
    if (orphans.size() == 1) CHECK_EQ(orphans.front(), h.west);

    CHECK_TRUE(stack.undo());
    CHECK_EQ(graph.arm_count(h.hub), size_t{3});
    CHECK_EQ(graph.segment_count(), size_t{3});
    CHECK_TRUE(graph.orphan_nodes().empty());
}

TEST(GraphEdit, a_junction_is_removed_by_deleting_its_segments_then_its_node) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);
    stack.seal();
    const size_t depth = stack.undo_depth();

    // The recipe DeleteNodeCommand's documentation gives, and the reason its own
    // refusal is not a dead end. segments_at() is copied because the deletes
    // below rewrite the very list it refers to.
    const std::vector<SegmentId> arms = graph.segments_at(h.hub);
    CHECK_EQ(arms.size(), size_t{3});

    stack.begin_transaction("Delete junction");
    for (const SegmentId arm : arms) {
        CHECK_TRUE(stack.execute(std::make_unique<DeleteSegmentCommand>(graph, arm)));
    }
    CHECK_TRUE(stack.execute(std::make_unique<DeleteNodeCommand>(graph, h.hub)));
    stack.commit_transaction();

    CHECK_FALSE(graph.contains_node(h.hub));
    CHECK_EQ(graph.segment_count(), size_t{0});
    CHECK_EQ(graph.node_count(), size_t{3});
    CHECK_EQ(stack.undo_depth(), depth + 1);

    // One step takes the whole junction back.
    CHECK_TRUE(stack.undo());
    CHECK_TRUE(graph.contains_node(h.hub));
    CHECK_EQ(graph.segment_count(), size_t{3});
    CHECK_EQ(graph.arm_count(h.hub), size_t{3});
    CHECK_EQ(stack.undo_depth(), depth);
}

TEST(GraphEdit, an_aborted_transaction_leaves_the_graph_as_it_was) {
    EditableGraph graph;
    CommandStack stack;
    const Hub h = build_hub(graph, stack);
    const size_t depth = stack.undo_depth();

    stack.begin_transaction("Abandoned edit");
    CHECK_TRUE(move_node(graph, stack, h.hub, 88.0, -41.0));
    CHECK_TRUE(stack.execute(std::make_unique<DeleteSegmentCommand>(graph, h.east_arm)));
    stack.abort_transaction();

    CHECK_EQ(graph.segment_count(), size_t{3});
    CHECK_TRUE(graph.contains_segment(h.east_arm));
    CHECK_EQ(graph.arm_count(h.hub), size_t{3});
    CHECK_NEAR(position_of(graph, h.hub).x, 0.0, kExact);
    CHECK_NEAR(position_of(graph, h.hub).y, 5.0, kExact);
    CHECK_EQ(stack.undo_depth(), depth);
}

// ============================================================================
// Splitting
// ============================================================================

namespace {

/// Every attribute set to something that is not its default, so a split that
/// dropped any single one of them fails the comparison.
SegmentAttributes loud_attributes() {
    SegmentAttributes a;
    a.type = RoadType::Tertiary;
    a.layer = -2;
    a.width = 9.25f;
    a.lanes = 5;
    a.lanes_forward = 3;
    a.lanes_backward = 2;
    a.is_oneway = true;
    a.is_bridge = true;
    a.is_tunnel = true;
    a.is_roundabout = true;
    a.is_link = true;
    a.sidewalk = SideFlags::Left;
    a.cycleway = SideFlags::Right;
    a.parking = SideFlags::Both;
    a.shoulder = SideFlags::None;
    a.surface = "sett";
    a.name = "Bull Alley Street";
    return a;
}

} // namespace

TEST(GraphEdit, split_gives_both_halves_every_attribute_and_the_parent_way) {
    EditableGraph graph;
    CommandStack stack;
    const SegmentAttributes attributes = loud_attributes();

    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId b = make_node(graph, stack, 100.0, 0.0);

    auto add = std::make_unique<AddSegmentCommand>(graph, std::vector<NodeId>{a, b},
                                                   attributes);
    const SegmentId road = add->segment();
    const WayId way = add->way();
    CHECK_TRUE(stack.execute(std::move(add)));

    auto split = std::make_unique<SplitSegmentCommand>(graph, road, glm::dvec2(37.0, 12.0));
    const SegmentId first = split->first_half();
    const SegmentId second = split->second_half();
    const bool ok = stack.execute(std::move(split));
    CHECK_TRUE(ok);
    if (!ok) return;

    CHECK_FALSE(graph.contains_segment(road));
    const auto* left = graph.find_segment(first);
    const auto* right = graph.find_segment(second);
    CHECK_TRUE(left != nullptr);
    CHECK_TRUE(right != nullptr);
    if (left == nullptr || right == nullptr) return;

    CHECK_TRUE(left->attributes == attributes);
    CHECK_TRUE(right->attributes == attributes);
    // Spelled out for two of them, so a failure names the field rather than the
    // whole struct.
    CHECK_EQ(right->attributes.name, std::string{"Bull Alley Street"});
    CHECK_EQ(right->attributes.lanes_backward, 2);

    // Two segments of ONE street, which is what RoadGraph produces when it cuts a
    // way at a junction. A fresh way id on either half would read downstream as
    // two different streets.
    CHECK_EQ(left->source_way, way);
    CHECK_EQ(right->source_way, way);
}

TEST(GraphEdit, split_joins_the_halves_at_one_node_and_lands_off_centre) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId b = make_node(graph, stack, 100.0, 0.0);
    const SegmentId road = make_segment(graph, stack, {a, b});

    auto split = std::make_unique<SplitSegmentCommand>(graph, road, glm::dvec2(37.0, 12.0));
    const SegmentId first = split->first_half();
    const SegmentId second = split->second_half();
    SplitSegmentCommand* raw = split.get();
    const bool ok = stack.execute(std::move(split));
    CHECK_TRUE(ok);
    if (!ok) return;

    const NodeId join = raw->join_node();
    CHECK((join < 0));
    CHECK_TRUE(graph.contains_node(join));

    const auto* left = graph.find_segment(first);
    const auto* right = graph.find_segment(second);
    CHECK_TRUE(left != nullptr);
    CHECK_TRUE(right != nullptr);
    if (left == nullptr || right == nullptr) return;

    // The halves meet because they name the same node, exactly as two OSM ways do.
    CHECK_EQ(left->node_ids.back(), join);
    CHECK_EQ(right->node_ids.front(), join);
    CHECK_EQ(graph.arm_count(join), size_t{2});
    CHECK_TRUE(graph.is_shared(join));

    // The point was projected onto the line, not used as given, and it is not
    // the midpoint: a split that always cut in half would give 50 and 50.
    CHECK_NEAR(position_of(graph, join).x, 37.0, 1e-9);
    CHECK_NEAR(position_of(graph, join).y, 0.0, 1e-9);
    CHECK_NEAR(graph.length(first), 37.0, 1e-9);
    CHECK_NEAR(graph.length(second), 63.0, 1e-9);
}

TEST(GraphEdit, split_undo_restores_the_original_and_redo_reuses_the_same_handles) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId b = make_node(graph, stack, 100.0, 0.0);
    const SegmentId road = make_segment(graph, stack, {a, b});

    auto split = std::make_unique<SplitSegmentCommand>(graph, road, glm::dvec2(37.0, 12.0));
    const SegmentId first = split->first_half();
    const SegmentId second = split->second_half();
    SplitSegmentCommand* raw = split.get();
    const bool ok = stack.execute(std::move(split));
    CHECK_TRUE(ok);
    if (!ok) return;
    const NodeId join = raw->join_node();

    CHECK_TRUE(stack.undo());
    CHECK_TRUE(graph.contains_segment(road));
    CHECK_FALSE(graph.contains_segment(first));
    CHECK_FALSE(graph.contains_segment(second));
    CHECK_FALSE(graph.contains_node(join));
    CHECK_EQ(graph.node_count(), size_t{2});
    CHECK_EQ(graph.segment_count(), size_t{1});
    CHECK_NEAR(graph.length(road), 100.0, 1e-9);

    CHECK_TRUE(stack.redo());
    // The SAME handles the first apply issued. A command that reserved in
    // apply() would hand out fresh ones and quietly invalidate every id a caller
    // stored between the split and the undo.
    CHECK_TRUE(graph.contains_segment(first));
    CHECK_TRUE(graph.contains_segment(second));
    CHECK_EQ(raw->join_node(), join);
    CHECK_TRUE(graph.contains_node(join));
    CHECK_FALSE(graph.contains_segment(road));
    const auto* left = graph.find_segment(first);
    CHECK_TRUE(left != nullptr);
    if (left != nullptr) CHECK_EQ(left->node_ids.back(), join);
}

TEST(GraphEdit, split_within_the_snap_reuses_the_shape_point_it_landed_on) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId m = make_node(graph, stack, 40.0, 0.0);
    const NodeId b = make_node(graph, stack, 100.0, 0.0);
    const SegmentId road = make_segment(graph, stack, {a, m, b});
    const size_t before = graph.node_count();

    auto split =
        std::make_unique<SplitSegmentCommand>(graph, road, glm::dvec2(41.0, 0.3), 5.0);
    const SegmentId first = split->first_half();
    const SegmentId second = split->second_half();
    SplitSegmentCommand* raw = split.get();
    const bool ok = stack.execute(std::move(split));
    CHECK_TRUE(ok);
    if (!ok) return;

    CHECK_EQ(raw->join_node(), m);
    CHECK_EQ(graph.node_count(), before);   // no node invented next to one that exists
    CHECK_EQ(graph.arm_count(m), size_t{2});
    const auto* left = graph.find_segment(first);
    const auto* right = graph.find_segment(second);
    CHECK_TRUE(left != nullptr);
    CHECK_TRUE(right != nullptr);
    if (left == nullptr || right == nullptr) return;
    CHECK_EQ(left->node_ids.size(), size_t{2});
    CHECK_EQ(right->node_ids.size(), size_t{2});
    CHECK_NEAR(graph.length(first), 40.0, 1e-9);
}

TEST(GraphEdit, split_outside_the_snap_creates_a_node_next_to_the_shape_point) {
    EditableGraph graph;
    CommandStack stack;
    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    const NodeId m = make_node(graph, stack, 40.0, 0.0);
    const NodeId b = make_node(graph, stack, 100.0, 0.0);
    const SegmentId road = make_segment(graph, stack, {a, m, b});
    const size_t before = graph.node_count();

    // The same point as the previous test, at the default snap of a millimetre.
    auto split = std::make_unique<SplitSegmentCommand>(graph, road, glm::dvec2(41.0, 0.3));
    SplitSegmentCommand* raw = split.get();
    const bool ok = stack.execute(std::move(split));
    CHECK_TRUE(ok);
    if (!ok) return;

    CHECK((raw->join_node() != m));
    CHECK_EQ(graph.node_count(), before + 1);
    CHECK_NEAR(position_of(graph, raw->join_node()).x, 41.0, 1e-9);
    CHECK_NEAR(position_of(graph, raw->join_node()).y, 0.0, 1e-9);
    // The shape point it did not snap to is now interior to the first half.
    const auto* left = graph.find_segment(raw->first_half());
    CHECK_TRUE(left != nullptr);
    if (left == nullptr) return;
    CHECK_EQ(left->node_ids.size(), size_t{3});
    CHECK_EQ(left->node_ids[1], m);
}

// ============================================================================
// Loading from a solved RoadGraph
// ============================================================================

TEST(GraphEdit, from_a_road_graph_a_shared_osm_node_is_one_node_with_three_arms) {
    // A T: way 1 runs west to east through node 11; way 2 ends on it.
    const EditableGraph graph = seed_from({
        make_road(1, {10, 11, 12}, {{-50.0, 0.0}, {0.0, 0.0}, {60.0, 0.0}}),
        make_road(2, {11, 13}, {{0.0, 0.0}, {0.0, 45.0}}),
    });

    CHECK_EQ(graph.node_count(), size_t{4});
    CHECK_EQ(graph.segment_count(), size_t{3});

    // The OSM id is kept, so a handle taken from the extract still addresses it.
    CHECK_TRUE(graph.contains_node(NodeId{11}));
    CHECK_EQ(graph.arm_count(NodeId{11}), size_t{3});
    CHECK_TRUE(graph.is_shared(NodeId{11}));
    CHECK_EQ(graph.arm_count(NodeId{10}), size_t{1});
    CHECK_EQ(graph.arm_count(NodeId{13}), size_t{1});

    // A load is not an edit. Marking every segment dirty here would make the
    // first frame after a load do the whole solve a second time.
    CHECK_TRUE(graph.dirty_segments().empty());
    CHECK_TRUE(graph.dirty_nodes().empty());
}

TEST(GraphEdit, from_a_road_graph_a_grade_separation_stays_two_nodes) {
    // A bridge on layer 1 crossing a road on layer 0. Both ways pass THROUGH the
    // shared OSM node 21, which is what makes it a grade separation rather than
    // an abutment.
    const EditableGraph graph = seed_from({
        make_road(1, {20, 21, 22}, {{-50.0, 0.0}, {0.0, 0.0}, {60.0, 0.0}}, 0),
        make_road(2, {23, 21, 24}, {{0.0, -40.0}, {0.0, 0.0}, {0.0, 50.0}}, 1),
    });

    // Six nodes, not five: the crossing is TWO identities at one coordinate.
    // Keying the load by OSM id alone gives five, and node 21 then has four arms
    // -- a junction where a bridge merely passes over a road.
    CHECK_EQ(graph.node_count(), size_t{6});
    CHECK_EQ(graph.segment_count(), size_t{4});
    CHECK_TRUE(graph.contains_node(NodeId{21}));
    CHECK_EQ(graph.arm_count(NodeId{21}), size_t{2});

    // Find the other identity sitting on the same coordinate.
    NodeId twin = kInvalidNode;
    for (const auto& entry : graph.nodes()) {
        if (entry.first == NodeId{21}) continue;
        if (entry.second.position == glm::dvec2(0.0, 0.0)) twin = entry.first;
    }
    CHECK((twin != kInvalidNode));
    if (twin == kInvalidNode) return;

    // Re-keyed into the local space, because the OSM id was already spent.
    CHECK((twin < 0));
    CHECK_EQ(graph.arm_count(twin), size_t{2});

    // Neither crossing way can see the other's node, so nothing downstream can
    // ever decide the two meet.
    for (const auto& entry : graph.segments()) {
        const std::vector<NodeId>& ids = entry.second.node_ids;
        const bool has_osm =
            std::find(ids.begin(), ids.end(), NodeId{21}) != ids.end();
        const bool has_twin = std::find(ids.begin(), ids.end(), twin) != ids.end();
        CHECK_FALSE(has_osm && has_twin);
    }
}

TEST(GraphEdit, from_a_road_graph_a_bridge_abutment_stays_one_node) {
    // The deck and its approach disagree on layer=*, but every way ENDS on node
    // 31. That is one continuous road, and splitting it per layer would detach
    // every bridge in an extract from its ramps.
    const EditableGraph graph = seed_from({
        make_road(1, {30, 31}, {{-50.0, 0.0}, {0.0, 0.0}}, 0),
        make_road(2, {31, 32}, {{0.0, 0.0}, {60.0, 0.0}}, 1),
    });

    CHECK_EQ(graph.node_count(), size_t{3});
    CHECK_EQ(graph.segment_count(), size_t{2});
    CHECK_TRUE(graph.contains_node(NodeId{31}));
    CHECK_EQ(graph.arm_count(NodeId{31}), size_t{2});
    CHECK_TRUE(graph.is_shared(NodeId{31}));
}

TEST(GraphEdit, from_a_road_graph_a_shape_point_becomes_an_editable_node) {
    // Node 41 is referenced by one way only, so RoadGraph leaves it as a plain
    // polyline vertex. Here it is a node like any other, movable and deletable,
    // but with no arms.
    EditableGraph graph = seed_from({
        make_road(1, {40, 41, 42, 43},
                  {{-50.0, 0.0}, {-20.0, 10.0}, {0.0, 0.0}, {60.0, 0.0}}),
        make_road(2, {42, 44}, {{0.0, 0.0}, {0.0, 45.0}}),
    });
    CommandStack stack;

    CHECK_EQ(graph.node_count(), size_t{5});
    CHECK_TRUE(graph.contains_node(NodeId{41}));
    CHECK_EQ(graph.reference_count(NodeId{41}), size_t{1});
    CHECK_EQ(graph.arm_count(NodeId{41}), size_t{0});
    CHECK_FALSE(graph.is_shared(NodeId{41}));
    CHECK_EQ(graph.arm_count(NodeId{42}), size_t{3});

    const std::vector<SegmentId> users = graph.segments_at(NodeId{41});
    CHECK_EQ(users.size(), size_t{1});
    if (users.size() != 1) return;

    CHECK_TRUE(move_node(graph, stack, NodeId{41}, -20.0, 31.0));
    const std::vector<glm::dvec2> line = graph.polyline(users.front());
    CHECK_EQ(line.size(), size_t{3});
    if (line.size() == 3) {
        CHECK_NEAR(line[1].y, 31.0, kExact);
        CHECK_NEAR(line[0].x, -50.0, kExact);
        CHECK_NEAR(line[2].x, 0.0, kExact);
    }
}

TEST(GraphEdit, a_seeded_graph_never_hands_out_an_id_it_already_contains) {
    // An extract that has already been through an editor carries negative ids of
    // its own. Starting the local allocator at -1 regardless would hand back -1,
    // the insert would refuse, and the edit would fail for no visible reason.
    EditableGraph graph = seed_from({
        make_road(-5, {-1, -2}, {{0.0, 0.0}, {30.0, 0.0}}),
    });
    CommandStack stack;

    CHECK_TRUE(graph.contains_node(NodeId{-1}));
    CHECK_TRUE(graph.contains_node(NodeId{-2}));

    auto command = std::make_unique<AddNodeCommand>(graph, glm::dvec2(5.0, 5.0));
    const NodeId fresh = command->node();
    CHECK((fresh < NodeId{-2}));
    CHECK_TRUE(stack.execute(std::move(command)));
    CHECK_EQ(graph.node_count(), size_t{3});

    // The way id space is primed the same way and is independent of the node one.
    const NodeId other = make_node(graph, stack, 9.0, 9.0);
    auto segment = std::make_unique<AddSegmentCommand>(graph,
                                                       std::vector<NodeId>{fresh, other});
    const WayId way = segment->way();
    CHECK((way < WayId{-5}));
    CHECK_TRUE(stack.execute(std::move(segment)));
}

TEST(GraphEdit, a_layer_split_never_re_keys_onto_an_id_the_extract_already_uses) {
    // Ways 1 and 2 cross at node 200 on different layers, so the load has to
    // re-key the second identity out of the local id space. Way 3 carries
    // negative ids of its own, the way an extract that has been through an
    // editor does, and it is reached AFTER that re-key.
    //
    // This is what the up-front priming in from_road_graph() is for. An
    // allocator primed only as nodes are inserted is still at -1 when the
    // crossing twin asks for an id, takes -1, and way 3's own node -1 then has
    // to be re-keyed off its OSM identity -- provenance lost, silently, and only
    // for extracts that happen to contain negative ids.
    const EditableGraph graph = seed_from({
        make_road(1, {100, 200, 101}, {{-50.0, 0.0}, {0.0, 0.0}, {60.0, 0.0}}, 0),
        make_road(2, {102, 200, 103}, {{0.0, -40.0}, {0.0, 0.0}, {0.0, 50.0}}, 1),
        make_road(3, {-1, -2}, {{300.0, 700.0}, {360.0, 700.0}}, 0),
    });

    CHECK_TRUE(graph.contains_node(NodeId{-1}));
    CHECK_TRUE(graph.contains_node(NodeId{-2}));
    // ... and they are way 3's nodes, not the crossing twin wearing their ids.
    CHECK_NEAR(position_of(graph, NodeId{-1}).x, 300.0, kExact);
    CHECK_NEAR(position_of(graph, NodeId{-1}).y, 700.0, kExact);
    CHECK_EQ(graph.arm_count(NodeId{-1}), size_t{1});
    CHECK_EQ(graph.arm_count(NodeId{200}), size_t{2});
}

// ============================================================================
// What the stack is told about each edit
// ============================================================================

TEST(GraphEdit, every_edit_names_itself_for_the_undo_menu) {
    EditableGraph graph;
    CommandStack stack;

    const NodeId a = make_node(graph, stack, 0.0, 0.0);
    CHECK_EQ(stack.undo_label(), std::string{"Add node"});

    const NodeId b = make_node(graph, stack, 50.0, 0.0);
    const SegmentId road = make_segment(graph, stack, {a, b});
    CHECK_EQ(stack.undo_label(), std::string{"Add street segment"});

    CHECK_TRUE(move_node(graph, stack, a, 3.0, 4.0));
    CHECK_EQ(stack.undo_label(), std::string{"Move node"});
    CHECK_TRUE(stack.undo());
    CHECK_EQ(stack.redo_label(), std::string{"Move node"});

    auto split = std::make_unique<SplitSegmentCommand>(graph, road, glm::dvec2(25.0, 6.0));
    const SegmentId first = split->first_half();
    CHECK_TRUE(stack.execute(std::move(split)));
    CHECK_EQ(stack.undo_label(), std::string{"Split street segment"});

    CHECK_TRUE(stack.execute(std::make_unique<DeleteSegmentCommand>(graph, first)));
    CHECK_EQ(stack.undo_label(), std::string{"Delete street segment"});

    // The first half is gone, so node a is an orphan and deleting it is allowed.
    CHECK_TRUE(stack.execute(std::make_unique<DeleteNodeCommand>(graph, a)));
    CHECK_EQ(stack.undo_label(), std::string{"Delete node"});
}

TEST(GraphEdit, a_command_holding_a_deleted_street_reports_what_it_holds) {
    // Two identical edits apart from the length of one tag. The stack's byte
    // total has to tell them apart, because CommandStackConfig::max_bytes bounds
    // nothing otherwise: a command that keeps a deleted street alive for undo and
    // reports only sizeof(*this) makes the ceiling a fiction.
    auto delete_a_street_named = [](const std::string& name) -> size_t {
        EditableGraph graph;
        CommandStack stack;
        SegmentAttributes attributes;
        attributes.name = name;

        const NodeId a = make_node(graph, stack, 0.0, 0.0);
        const NodeId b = make_node(graph, stack, 50.0, 0.0);
        const SegmentId road = make_segment(graph, stack, {a, b}, attributes);

        const size_t before = stack.bytes();
        if (!stack.execute(std::make_unique<DeleteSegmentCommand>(graph, road))) return 0;
        return stack.bytes() - before;
    };

    const size_t small = delete_a_street_named("A");
    const size_t large = delete_a_street_named(std::string(4096, 'x'));

    CHECK((small > 0));
    CHECK((small < large));
    CHECK((large - small >= size_t{4000}));
}
