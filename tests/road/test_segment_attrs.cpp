// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_segment_attrs.cpp
 * @brief Per-segment attribute edits: refusal, coalescing, and what each one breaks
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Every test here is written so that inverting the behaviour it names makes it
 * fail. The six that matter most, and what each one catches:
 *
 *   - `a_surface_edit_moves_vertices_and_the_table_has_to_say_so` MEASURES the
 *     profile before and after a real surface edit instead of taking the
 *     invalidation table's word for it. It is here because the table's first
 *     version said a surface edit was a re-skin, and two tests in this file
 *     asserted that it was: build_profile() reads the surface a second time
 *     through surface_is_unpaved(), and an unpaved carriageway grows a verge,
 *     which moves total_width() and therefore both junction solves.
 *
 *   - `an_edit_keeps_the_handle_the_way_and_the_nodes` is the guard on HOW the
 *     edit is made. The attributes are replaced by removing the segment and
 *     putting a copy back, so a mistake there loses the node list, the source
 *     way or the handle itself. It checks all three, plus the neighbour's arm
 *     count at the shared node.
 *   - `a_spinner_drag_that_returns_to_its_start_leaves_no_no_op_step` runs the
 *     whole undo/redo cycle over a round trip. A merge that absorbed the return
 *     leg produces a step whose redo refuses, and CommandStack drops it; the
 *     second `redo()` then returns false.
 *   - `an_edit_may_leave_an_error_it_did_not_introduce` seeds a segment already
 *     tagged `lanes=0`. An implementation that refuses on "is the result valid"
 *     rather than "what did this edit add" cannot rename it, and one that
 *     compares a single first-error code lets a zero width through on it.
 *   - `unknown_and_none_are_two_different_edits` is the SideFlags trap. An
 *     inspector that folds the pair passes every other test in this file.
 *   - `every_field_declares_whether_it_moves_geometry` sweeps the whole field
 *     enum, so a field added without an invalidation entry fails here rather
 *     than silently telling B4 there is nothing to re-solve.
 *
 * Values are deliberately asymmetric -- the two arms have different widths,
 * names and side flags, and the lane counts are never a symmetric pair -- so an
 * edit applied to the wrong segment or written into the wrong field cannot land
 * on the right answer by coincidence.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests SegmentAttrs
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/graph_edit.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/road/segment_attrs.hpp"
#include "osm/types.hpp"
#include "scene/command.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using stratum::osm::NodeId;
using stratum::osm::RoadType;
using stratum::osm::SideFlags;
using stratum::osm::side_flags_name;
using stratum::osm::road::AddNodeCommand;
using stratum::osm::road::AddSegmentCommand;
using stratum::osm::road::build_profile;
using stratum::osm::road::EditableGraph;
using stratum::osm::road::describe_refusal;
using stratum::osm::road::EditableSegment;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::has;
using stratum::osm::road::invalidation_of;
using stratum::osm::road::kInvalidNode;
using stratum::osm::road::kInvalidSegment;
using stratum::osm::road::LaneCounts;
using stratum::osm::road::lane_counts_of;
using stratum::osm::road::field_label;
using stratum::osm::road::field_name;
using stratum::osm::road::normalise_surface;
using stratum::osm::road::ProfileConfig;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::SegmentAttributeCommand;
using stratum::osm::road::SegmentAttributes;
using stratum::osm::road::SegmentField;
using stratum::osm::road::SegmentFlag;
using stratum::osm::road::SegmentId;
using stratum::osm::road::SegmentInvalidation;
using stratum::osm::road::SegmentRefusal;
using stratum::osm::road::SegmentText;
using stratum::osm::road::SetSegmentFlagCommand;
using stratum::osm::road::SetSegmentLanesCommand;
using stratum::osm::road::SetSegmentLayerCommand;
using stratum::osm::road::SetSegmentSideCommand;
using stratum::osm::road::SetSegmentTextCommand;
using stratum::osm::road::SetSegmentTypeCommand;
using stratum::osm::road::SetSegmentWidthCommand;
using stratum::osm::road::SideFeature;
using stratum::osm::road::validate_attributes;
using stratum::scene::CommandStack;

// ============================================================================
// Fixture
// ============================================================================

/// Attributes of the west arm. Nothing here is a default: a write that lands in
/// the wrong field changes a value that the whole-struct comparisons notice.
SegmentAttributes west_attributes() {
    SegmentAttributes a;
    a.type = RoadType::Residential;
    a.layer = 0;
    a.width = 6.5f;
    a.lanes = 3;
    a.lanes_forward = 2;
    a.lanes_backward = 1;
    a.is_oneway = false;
    a.is_bridge = false;
    a.is_tunnel = false;
    a.sidewalk = SideFlags::Unknown;  // absent tag: a class default may be inferred
    a.cycleway = SideFlags::None;     // explicit no: nothing may be inferred
    a.parking = SideFlags::Left;
    a.shoulder = SideFlags::Right;
    a.surface = "asphalt";
    a.name = "Ashton Road";
    return a;
}

/// Attributes of the east arm. Different in every field the tests touch, so
/// "the edit went to the other segment" and "the edit went nowhere" are
/// distinguishable outcomes.
SegmentAttributes east_attributes() {
    SegmentAttributes a;
    a.type = RoadType::Tertiary;
    a.layer = 1;
    a.width = 9.25f;
    a.lanes = 2;
    a.lanes_forward = -1;
    a.lanes_backward = -1;
    a.is_oneway = true;
    a.sidewalk = SideFlags::Both;
    a.cycleway = SideFlags::Unknown;
    a.parking = SideFlags::None;
    a.shoulder = SideFlags::Unknown;
    a.surface = "cobblestone";
    a.name = "Dunmore Street";
    return a;
}

/**
 * @brief Two arms meeting at one node, built through the stack
 *
 * Built in place rather than returned by value: every command in the stack holds
 * a pointer to this graph, so a fixture that moved would leave them pointing at
 * the old address.
 */
struct Fixture {
    EditableGraph graph;
    CommandStack stack;

    NodeId west = kInvalidNode;
    NodeId hub = kInvalidNode;
    NodeId east = kInvalidNode;

    SegmentId west_arm = kInvalidSegment;
    SegmentId east_arm = kInvalidSegment;

    Fixture() {
        west = add_node(-30.0, 4.0);
        hub = add_node(0.0, 4.0);
        east = add_node(25.0, -9.0);

        west_arm = add_segment({west, hub}, west_attributes());
        east_arm = add_segment({hub, east}, east_attributes());

        // The history of the setup is not the history under test.
        stack.clear();
        graph.clear_dirty();
    }

    NodeId add_node(double x, double y) {
        auto command = std::make_unique<AddNodeCommand>(graph, glm::dvec2(x, y));
        const NodeId id = command->node();
        return stack.execute(std::move(command)) ? id : kInvalidNode;
    }

    SegmentId add_segment(std::vector<NodeId> ids, SegmentAttributes attributes) {
        auto command =
            std::make_unique<AddSegmentCommand>(graph, std::move(ids), std::move(attributes));
        const SegmentId id = command->segment();
        return stack.execute(std::move(command)) ? id : kInvalidSegment;
    }
};

/// Attributes of a segment, or a set marked as missing so that a comparison
/// against real attributes fails loudly instead of matching a default.
SegmentAttributes attributes_of(const EditableGraph& graph, SegmentId segment) {
    const EditableSegment* record = graph.find_segment(segment);
    if (record == nullptr) {
        SegmentAttributes missing;
        missing.name = "<no such segment>";
        return missing;
    }
    return record->attributes;
}

/**
 * @brief The cross-section build_profile() makes of one segment's attributes
 *
 * There is no to_road_graph() yet -- that is B4's -- so the hop from
 * SegmentAttributes to GraphEdge is made here by hand. It is the same 1:1 copy
 * that graph_edit.cpp's attributes_of() makes in the other direction, and every
 * field build_profile() reads is carried across, so no result below depends on a
 * field this helper quietly dropped.
 *
 * Used to MEASURE what an attribute edit does to the geometry, rather than
 * trusting the invalidation table's own account of it.
 */
RoadProfile profile_of(const SegmentAttributes& a) {
    GraphEdge edge;
    edge.type = a.type;
    edge.layer = a.layer;
    edge.width = a.width;
    edge.lanes = a.lanes;
    edge.lanes_forward = a.lanes_forward;
    edge.lanes_backward = a.lanes_backward;
    edge.is_oneway = a.is_oneway;
    edge.is_bridge = a.is_bridge;
    edge.is_tunnel = a.is_tunnel;
    edge.is_roundabout = a.is_roundabout;
    edge.is_link = a.is_link;
    edge.sidewalk = a.sidewalk;
    edge.cycleway = a.cycleway;
    edge.parking = a.parking;
    edge.shoulder = a.shoulder;
    edge.surface = a.surface;
    edge.name = a.name;
    return build_profile(edge, ProfileConfig{});
}

bool set_width(Fixture& f, SegmentId segment, float width) {
    return f.stack.execute(std::make_unique<SetSegmentWidthCommand>(f.graph, segment, width));
}

bool set_name(Fixture& f, SegmentId segment, std::string name) {
    return f.stack.execute(std::make_unique<SetSegmentTextCommand>(f.graph, segment,
                                                                   SegmentText::Name,
                                                                   std::move(name)));
}

bool set_surface(Fixture& f, SegmentId segment, std::string surface) {
    return f.stack.execute(std::make_unique<SetSegmentTextCommand>(f.graph, segment,
                                                                   SegmentText::Surface,
                                                                   std::move(surface)));
}

bool set_side(Fixture& f, SegmentId segment, SideFeature feature, SideFlags value) {
    return f.stack.execute(
        std::make_unique<SetSegmentSideCommand>(f.graph, segment, feature, value));
}

bool set_lanes(Fixture& f, SegmentId segment, LaneCounts lanes) {
    return f.stack.execute(std::make_unique<SetSegmentLanesCommand>(f.graph, segment, lanes));
}

/// Readable side-flag comparison: a failure prints "Unknown" and "None" rather
/// than two unprintable enums, which is the whole difference for this file.
std::string side_name(SideFlags value) { return std::string(side_flags_name(value)); }

bool dirty_has_segment(const EditableGraph& graph, SegmentId segment) {
    return graph.dirty_segments().count(segment) != 0;
}

bool dirty_has_node(const EditableGraph& graph, NodeId node) {
    return graph.dirty_nodes().count(node) != 0;
}

} // namespace

// ============================================================================
// The edit itself
// ============================================================================

TEST(SegmentAttrs, a_width_edit_changes_the_width_and_nothing_else) {
    Fixture f;

    CHECK_TRUE(set_width(f, f.west_arm, 11.75f));

    // Field by field would pass while quietly dropping a field this test does
    // not name. The whole struct is compared against the one field's change.
    SegmentAttributes expected = west_attributes();
    expected.width = 11.75f;
    CHECK_TRUE(attributes_of(f.graph, f.west_arm) == expected);
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, 11.75f);
}

TEST(SegmentAttrs, an_edit_keeps_the_handle_the_way_and_the_nodes) {
    Fixture f;

    const EditableSegment before = *f.graph.find_segment(f.west_arm);
    const std::vector<glm::dvec2> line_before = f.graph.polyline(f.west_arm);

    CHECK_TRUE(set_width(f, f.west_arm, 8.0f));

    // The attributes are replaced by taking the segment out and putting a copy
    // back, so everything that is NOT an attribute has to come through
    // untouched: the handle stays valid, the way id stays the same street, and
    // the vertex list is the geometry.
    CHECK_TRUE(f.graph.contains_segment(f.west_arm));
    CHECK_EQ(f.graph.segment_count(), size_t{2});

    const EditableSegment* after = f.graph.find_segment(f.west_arm);
    CHECK_EQ(after->id, f.west_arm);
    CHECK_EQ(after->source_way, before.source_way);
    CHECK_EQ(after->node_ids.size(), before.node_ids.size());
    for (size_t i = 0; i < before.node_ids.size() && i < after->node_ids.size(); ++i) {
        CHECK_EQ(after->node_ids[i], before.node_ids[i]);
    }

    const std::vector<glm::dvec2> line_after = f.graph.polyline(f.west_arm);
    CHECK_EQ(line_after.size(), line_before.size());
    for (size_t i = 0; i < line_before.size() && i < line_after.size(); ++i) {
        CHECK_NEAR(line_after[i].x, line_before[i].x, 1e-12);
        CHECK_NEAR(line_after[i].y, line_before[i].y, 1e-12);
    }

    // The usage table is rebuilt by the replacement, so the junction it feeds has
    // to survive it: both arms still meet at the hub, and neither is listed twice.
    CHECK_EQ(f.graph.arm_count(f.hub), size_t{2});
    CHECK_EQ(f.graph.segments_at(f.hub).size(), size_t{2});
    CHECK_EQ(f.graph.reference_count(f.west), size_t{1});
}

TEST(SegmentAttrs, editing_one_segment_leaves_its_neighbour_untouched) {
    Fixture f;

    CHECK_TRUE(set_width(f, f.west_arm, 14.5f));
    CHECK_TRUE(set_name(f, f.west_arm, "Marlow Row"));

    CHECK_TRUE(attributes_of(f.graph, f.east_arm) == east_attributes());
}

TEST(SegmentAttrs, undo_restores_every_field_and_redo_reinstates_the_edit) {
    Fixture f;

    CHECK_TRUE(set_width(f, f.west_arm, 12.25f));
    CHECK_TRUE(f.stack.undo());
    CHECK_TRUE(attributes_of(f.graph, f.west_arm) == west_attributes());

    CHECK_TRUE(f.stack.redo());
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, 12.25f);

    // Redo must not have created a second segment or moved the handle.
    CHECK_EQ(f.graph.segment_count(), size_t{2});
    CHECK_TRUE(f.graph.contains_segment(f.west_arm));
}

TEST(SegmentAttrs, a_command_reports_the_state_it_captured) {
    Fixture f;

    // Applied and reverted off the stack. B4 and the inspector both read
    // before() and after() to show what an edit did, and revert() has to work on
    // its own -- CommandStack::undo() calls it and cannot be told no.
    SetSegmentWidthCommand command(f.graph, f.west_arm, 10.5f);
    CHECK_TRUE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::None);
    CHECK_EQ(command.before().width, west_attributes().width);
    CHECK_EQ(command.after().width, 10.5f);
    CHECK_TRUE(command.segment() == f.west_arm);

    command.revert();
    CHECK_TRUE(attributes_of(f.graph, f.west_arm) == west_attributes());
}

TEST(SegmentAttrs, an_edit_to_a_segment_that_is_not_there_is_refused) {
    Fixture f;

    SetSegmentWidthCommand command(f.graph, kInvalidSegment, 7.0f);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::UnknownSegment);

    CHECK_FALSE(set_width(f, kInvalidSegment, 7.0f));
    CHECK_EQ(f.stack.undo_depth(), size_t{0});
}

// ============================================================================
// No-op refusal
// ============================================================================

TEST(SegmentAttrs, setting_a_value_to_what_it_already_is_records_nothing) {
    Fixture f;
    const std::uint64_t revision = f.stack.revision();

    CHECK_FALSE(set_width(f, f.west_arm, west_attributes().width));
    CHECK_FALSE(set_name(f, f.west_arm, west_attributes().name));

    CHECK_EQ(f.stack.undo_depth(), size_t{0});
    CHECK_FALSE(f.stack.can_undo());
    CHECK_EQ(f.stack.revision(), revision);
    CHECK_TRUE(attributes_of(f.graph, f.west_arm) == west_attributes());
}

TEST(SegmentAttrs, the_no_op_refusal_says_why) {
    Fixture f;

    SetSegmentWidthCommand command(f.graph, f.west_arm, west_attributes().width);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::NoChange);
}

TEST(SegmentAttrs, a_refusal_does_not_outlive_the_apply_that_succeeds) {
    Fixture f;

    // refusal() is also the answer to "did that work", so a stale value is a
    // successful edit reported as a failure. The only way to see it is one
    // command object driven through a refusal and then a success, and the only
    // way to get that is to apply by hand: CommandStack throws away a command
    // whose apply() returned false.
    SetSegmentWidthCommand command(f.graph, f.west_arm, west_attributes().width);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::NoChange);

    // Something else moves the width, so the same command is now a real change.
    // This is not a contrived state: it is what a redo meets when the value it
    // wants to reinstate became reachable again.
    CHECK_TRUE(set_width(f, f.west_arm, 8.0f));

    CHECK_TRUE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::None);
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, west_attributes().width);
}

// ============================================================================
// Coalescing
// ============================================================================

TEST(SegmentAttrs, a_spinner_drag_is_one_undo_step) {
    Fixture f;

    CHECK_TRUE(set_width(f, f.west_arm, 7.0f));
    CHECK_TRUE(set_width(f, f.west_arm, 8.5f));
    CHECK_TRUE(set_width(f, f.west_arm, 10.0f));

    CHECK_EQ(f.stack.undo_depth(), size_t{1});
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, 10.0f);

    // One undo returns to where the gesture started, not to its second step.
    CHECK_TRUE(f.stack.undo());
    CHECK_TRUE(attributes_of(f.graph, f.west_arm) == west_attributes());

    // And the redo replays the MERGED result, not the first step of the drag.
    CHECK_TRUE(f.stack.redo());
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, 10.0f);
}

TEST(SegmentAttrs, a_spinner_drag_that_returns_to_its_start_leaves_no_no_op_step) {
    Fixture f;
    const float start = west_attributes().width;

    CHECK_TRUE(set_width(f, f.west_arm, 9.0f));
    CHECK_TRUE(set_width(f, f.west_arm, start));

    // Two steps, not one. Absorbing the return leg would leave a command whose
    // old and new values are equal: its undo does nothing visible, and its redo
    // hits apply()'s equal-value guard, at which point CommandStack drops the
    // step and the rest of the redo branch with it.
    CHECK_EQ(f.stack.undo_depth(), size_t{2});

    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, 9.0f);
    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, start);

    // Both legs redo. This is the assertion the merged no-op fails.
    CHECK_TRUE(f.stack.redo());
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, 9.0f);
    CHECK_TRUE(f.stack.redo());
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, start);
}

TEST(SegmentAttrs, a_width_edit_does_not_absorb_a_rename) {
    Fixture f;

    CHECK_TRUE(set_width(f, f.west_arm, 7.5f));
    CHECK_TRUE(set_name(f, f.west_arm, "Marlow Row"));

    CHECK_EQ(f.stack.undo_depth(), size_t{2});

    // Undoing the rename leaves the width edit standing, which is what two
    // separate steps means.
    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(attributes_of(f.graph, f.west_arm).name, std::string("Ashton Road"));
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, 7.5f);
}

TEST(SegmentAttrs, a_sidewalk_edit_does_not_absorb_a_cycleway_edit) {
    Fixture f;

    // Same command class, different field. The merge key is the field, not the
    // type, and this is the test that says so.
    CHECK_TRUE(set_side(f, f.west_arm, SideFeature::Sidewalk, SideFlags::Both));
    CHECK_TRUE(set_side(f, f.west_arm, SideFeature::Cycleway, SideFlags::Left));

    CHECK_EQ(f.stack.undo_depth(), size_t{2});

    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(side_name(attributes_of(f.graph, f.west_arm).cycleway), std::string("None"));
    CHECK_EQ(side_name(attributes_of(f.graph, f.west_arm).sidewalk), std::string("Both"));
}

TEST(SegmentAttrs, edits_to_two_segments_never_merge) {
    Fixture f;

    CHECK_TRUE(set_width(f, f.west_arm, 7.5f));
    CHECK_TRUE(set_width(f, f.east_arm, 12.0f));

    CHECK_EQ(f.stack.undo_depth(), size_t{2});

    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(attributes_of(f.graph, f.east_arm).width, east_attributes().width);
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, 7.5f);
}

TEST(SegmentAttrs, edits_to_two_graphs_never_merge) {
    Fixture first;
    Fixture second;

    // The premise, and the reason the same-graph guard is not decorative:
    // EditableGraph numbers its segments from its own counter, starting at 1 in
    // every instance, so the first segment of one document carries the SAME
    // handle as the first segment of the next. Across the two edits below the
    // field matches and the segment handle matches; the graph is the only thing
    // that tells them apart.
    CHECK_EQ(first.west_arm, second.west_arm);

    // One stack driving two documents: an editor with two maps open and one undo
    // menu. Both edits are the same field of the same handle, back to back and
    // unsealed, which is exactly the shape a gesture has.
    CommandStack& stack = first.stack;
    CHECK_TRUE(stack.execute(
        std::make_unique<SetSegmentWidthCommand>(first.graph, first.west_arm, 8.0f)));
    CHECK_TRUE(stack.execute(
        std::make_unique<SetSegmentWidthCommand>(second.graph, second.west_arm, 12.0f)));

    CHECK_EQ(stack.undo_depth(), size_t{2});
    CHECK_EQ(attributes_of(first.graph, first.west_arm).width, 8.0f);
    CHECK_EQ(attributes_of(second.graph, second.west_arm).width, 12.0f);

    // One undo takes back the SECOND document's edit and leaves the first
    // standing. Coalesced, that one undo would revert the first document instead
    // and the second document's edit would have no step of its own left to undo.
    CHECK_TRUE(stack.undo());
    CHECK_EQ(attributes_of(second.graph, second.west_arm).width, west_attributes().width);
    CHECK_EQ(attributes_of(first.graph, first.west_arm).width, 8.0f);

    CHECK_TRUE(stack.undo());
    CHECK_TRUE(attributes_of(first.graph, first.west_arm) == west_attributes());
}

TEST(SegmentAttrs, a_merge_refuses_a_successor_that_never_applied) {
    Fixture f;

    SetSegmentWidthCommand first(f.graph, f.west_arm, 8.0f);
    SetSegmentWidthCommand second(f.graph, f.west_arm, 9.5f);
    CHECK_TRUE(first.apply());

    // Same graph, same segment, same field: every merge key matches. The one
    // thing missing is that `second` has never applied, so its stored result is
    // a default-constructed attribute set -- width 0, no name, no surface.
    // Absorbing it would make first's redo install THAT over the segment.
    //
    // CommandStack::execute() applies before it offers a merge, so it cannot
    // produce this call; a caller driving merge() by hand can, which is why the
    // guard stays and why this test drives it by hand too.
    CHECK_FALSE(first.merge(second));
    CHECK_EQ(first.after().width, 8.0f);
    CHECK_EQ(first.after().name, std::string("Ashton Road"));

    // Refused, so the command still undoes to where it began.
    first.revert();
    CHECK_TRUE(attributes_of(f.graph, f.west_arm) == west_attributes());
}

TEST(SegmentAttrs, seal_ends_the_gesture) {
    Fixture f;

    CHECK_TRUE(set_width(f, f.west_arm, 7.5f));
    f.stack.seal();
    CHECK_TRUE(set_width(f, f.west_arm, 8.5f));

    CHECK_EQ(f.stack.undo_depth(), size_t{2});
}

// ============================================================================
// Validation
// ============================================================================

TEST(SegmentAttrs, a_width_of_zero_is_refused_and_the_history_is_untouched) {
    Fixture f;
    const std::uint64_t revision = f.stack.revision();

    SetSegmentWidthCommand command(f.graph, f.west_arm, 0.0f);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::WidthNotPositive);

    CHECK_FALSE(set_width(f, f.west_arm, 0.0f));
    CHECK_EQ(f.stack.undo_depth(), size_t{0});
    CHECK_EQ(f.stack.revision(), revision);
    CHECK_TRUE(attributes_of(f.graph, f.west_arm) == west_attributes());
}

TEST(SegmentAttrs, a_negative_width_is_refused) {
    Fixture f;

    CHECK_FALSE(set_width(f, f.west_arm, -3.0f));
    CHECK_EQ(attributes_of(f.graph, f.west_arm).width, west_attributes().width);
}

TEST(SegmentAttrs, a_width_that_is_not_a_number_is_refused) {
    Fixture f;

    // `width > 0` is also the NaN test, since every comparison against a NaN is
    // false. A `!= 0` test would let this through and the NaN would surface much
    // later as a vertex position.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    SetSegmentWidthCommand command(f.graph, f.west_arm, nan);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::WidthNotPositive);

    const float infinity = std::numeric_limits<float>::infinity();
    CHECK_FALSE(set_width(f, f.west_arm, infinity));
    CHECK_TRUE(std::isfinite(attributes_of(f.graph, f.west_arm).width));
}

TEST(SegmentAttrs, a_lane_count_below_one_is_refused) {
    Fixture f;

    LaneCounts counts;
    counts.total = 0;
    counts.forward = -1;
    counts.backward = -1;

    SetSegmentLanesCommand command(f.graph, f.west_arm, counts);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::LanesBelowOne);

    CHECK_FALSE(set_lanes(f, f.west_arm, counts));
    CHECK_EQ(attributes_of(f.graph, f.west_arm).lanes, 3);
    CHECK_EQ(f.stack.undo_depth(), size_t{0});
}

TEST(SegmentAttrs, directional_lanes_exceeding_the_total_are_refused) {
    Fixture f;

    // The west arm is 3 lanes, 2 forward and 1 backward. Dropping the total to 2
    // leaves the pair totalling one more than exists.
    LaneCounts counts = lane_counts_of(f.graph, f.west_arm);
    counts.total = 2;

    SetSegmentLanesCommand command(f.graph, f.west_arm, counts);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::DirectionalLanesExceedTotal);

    CHECK_FALSE(set_lanes(f, f.west_arm, counts));
    CHECK_EQ(attributes_of(f.graph, f.west_arm).lanes, 3);
}

TEST(SegmentAttrs, one_direction_alone_can_exceed_the_total) {
    Fixture f;

    // lanes:backward is unspecified on the east arm, so the forward count is the
    // only evidence and 4 of 2 is still impossible. Treating "unspecified" as 0
    // and stopping there would let this through.
    LaneCounts counts = lane_counts_of(f.graph, f.east_arm);
    counts.forward = 4;

    SetSegmentLanesCommand command(f.graph, f.east_arm, counts);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::DirectionalLanesExceedTotal);
}

TEST(SegmentAttrs, a_directional_count_below_minus_one_is_refused) {
    Fixture f;

    // -1 is the one negative with a meaning: not specified.
    LaneCounts counts = lane_counts_of(f.graph, f.west_arm);
    counts.backward = -2;

    SetSegmentLanesCommand command(f.graph, f.west_arm, counts);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::DirectionalLanesNegative);
}

TEST(SegmentAttrs, lowering_the_total_is_allowed_once_the_directions_fit) {
    Fixture f;

    CHECK_TRUE(stratum::osm::road::set_directional_lanes(f.stack, f.graph, f.west_arm, 1, 1));
    CHECK_TRUE(stratum::osm::road::set_lane_total(f.stack, f.graph, f.west_arm, 2));

    const SegmentAttributes after = attributes_of(f.graph, f.west_arm);
    CHECK_EQ(after.lanes, 2);
    CHECK_EQ(after.lanes_forward, 1);
    CHECK_EQ(after.lanes_backward, 1);
}

TEST(SegmentAttrs, an_edit_may_leave_an_error_it_did_not_introduce) {
    Fixture f;

    // An extract really can carry lanes=0, and AddSegmentCommand does not judge
    // attributes. If validation asked "is the result valid" instead of "what did
    // this edit add", that one bad field would freeze every other field on the
    // segment.
    SegmentAttributes broken = west_attributes();
    broken.lanes = 0;
    const SegmentId bad = f.add_segment({f.west, f.east}, broken);
    CHECK_TRUE(bad != kInvalidSegment);
    f.stack.clear();

    CHECK_TRUE(set_name(f, bad, "Kelsall Lane"));
    CHECK_EQ(attributes_of(f.graph, bad).name, std::string("Kelsall Lane"));
    CHECK_EQ(attributes_of(f.graph, bad).lanes, 0);

    // The pre-existing error is tolerated; a NEW one on the same segment is not.
    // That only works because the errors are bits in a mask rather than one code.
    SetSegmentWidthCommand command(f.graph, bad, 0.0f);
    CHECK_FALSE(command.apply());
    CHECK_TRUE(command.refusal() == SegmentRefusal::WidthNotPositive);

    // And the lane count can still be repaired, which is the point of tolerating
    // it. The repair has to be a consistent one: this way is tagged 2 forward
    // and 1 backward, so a total of 2 is refused and a total of 3 is taken.
    CHECK_FALSE(stratum::osm::road::set_lane_total(f.stack, f.graph, bad, 2));
    CHECK_EQ(attributes_of(f.graph, bad).lanes, 0);
    CHECK_TRUE(stratum::osm::road::set_lane_total(f.stack, f.graph, bad, 3));
    CHECK_EQ(attributes_of(f.graph, bad).lanes, 3);
}

TEST(SegmentAttrs, validate_attributes_reports_every_broken_rule_at_once) {
    SegmentAttributes attributes = west_attributes();
    attributes.lanes = 0;
    attributes.width = 0.0f;

    const SegmentRefusal refusal = validate_attributes(attributes);
    CHECK_TRUE(has(refusal, SegmentRefusal::LanesBelowOne));
    CHECK_TRUE(has(refusal, SegmentRefusal::WidthNotPositive));

    // Not a refusal validate_attributes() can reach: those two are decided
    // against the graph, not against the values.
    CHECK_FALSE(has(refusal, SegmentRefusal::UnknownSegment));
    CHECK_FALSE(has(refusal, SegmentRefusal::NoChange));

    CHECK_TRUE(validate_attributes(west_attributes()) == SegmentRefusal::None);
}

TEST(SegmentAttrs, describe_refusal_names_every_bit_it_is_given) {
    CHECK_EQ(describe_refusal(SegmentRefusal::None), std::string("none"));

    // One arm per bit, and every bit checked on its own. Two of the six used to
    // be enough for this test, which meant four arms could be deleted without it
    // noticing -- and a refusal with no phrase is a refusal the inspector shows
    // as an empty tooltip beside a field that will not take the value typed
    // into it.
    CHECK_EQ(describe_refusal(SegmentRefusal::LanesBelowOne), std::string("lanes below 1"));
    CHECK_EQ(describe_refusal(SegmentRefusal::WidthNotPositive),
             std::string("width not above 0"));
    CHECK_EQ(describe_refusal(SegmentRefusal::DirectionalLanesNegative),
             std::string("directional lanes below -1"));
    CHECK_EQ(describe_refusal(SegmentRefusal::DirectionalLanesExceedTotal),
             std::string("directional lanes exceed the total"));
    CHECK_EQ(describe_refusal(SegmentRefusal::UnknownSegment), std::string("no such segment"));
    CHECK_EQ(describe_refusal(SegmentRefusal::NoChange), std::string("no change"));

    // Two bits: both named, comma separated, in bit order.
    CHECK_EQ(describe_refusal(SegmentRefusal::LanesBelowOne | SegmentRefusal::WidthNotPositive),
             std::string("lanes below 1, width not above 0"));

    // The two that validate_attributes() can never return are decided against the
    // graph, and apply() is the only thing that reports them -- so they are the
    // two arms most easily dropped and least easily missed.
    CHECK_EQ(describe_refusal(SegmentRefusal::UnknownSegment | SegmentRefusal::NoChange),
             std::string("no such segment, no change"));
}

// ============================================================================
// Invalidation
// ============================================================================

TEST(SegmentAttrs, a_width_edit_invalidates_the_corridor_and_both_junctions) {
    Fixture f;

    SetSegmentWidthCommand command(f.graph, f.west_arm, 9.5f);
    const SegmentInvalidation invalidation = command.invalidation();

    CHECK_TRUE(has(invalidation, SegmentInvalidation::Corridor));
    CHECK_TRUE(has(invalidation, SegmentInvalidation::Junctions));
    CHECK_TRUE(has(invalidation, SegmentInvalidation::Metadata));

    // The junction solve is the expensive half, and a width edit is exactly the
    // case that needs it: each arm's half-width comes from its own profile.
    CHECK_FALSE(has(invalidation, SegmentInvalidation::Structures));
}

TEST(SegmentAttrs, a_rename_invalidates_nothing_geometric) {
    Fixture f;

    SetSegmentTextCommand command(f.graph, f.west_arm, SegmentText::Name, "Marlow Row");
    const SegmentInvalidation invalidation = command.invalidation();

    CHECK_TRUE(has(invalidation, SegmentInvalidation::Metadata));
    CHECK_FALSE(has(invalidation, SegmentInvalidation::Corridor));
    CHECK_FALSE(has(invalidation, SegmentInvalidation::Junctions));
    CHECK_FALSE(has(invalidation, SegmentInvalidation::Appearance));
    CHECK_FALSE(has(invalidation, SegmentInvalidation::Structures));
}

TEST(SegmentAttrs, a_surface_edit_moves_vertices_and_the_table_has_to_say_so) {
    Fixture f;

    // This test exists because the table once called surface the cheap re-skin.
    // It is not asserted here, it is MEASURED: the profile is built before and
    // after a real surface edit, and the invalidation entry is checked against
    // what actually changed.
    //
    // The shape that shows it is a rural lane: sidewalk=no, so no kerb is
    // inferred over the explicit no, and no shoulder, so nothing else closes the
    // carriageway edge. Residential is one of the classes with rural_verge, so
    // this is where surface_is_unpaved() decides whether a Verge strip exists.
    SegmentAttributes lane = west_attributes();
    lane.sidewalk = SideFlags::None;
    lane.cycleway = SideFlags::None;
    lane.parking = SideFlags::None;
    lane.shoulder = SideFlags::None;
    lane.surface = "asphalt";
    const SegmentId rural = f.add_segment({f.west, f.east}, lane);
    CHECK_TRUE(rural != kInvalidSegment);
    f.stack.clear();

    const RoadProfile paved = profile_of(attributes_of(f.graph, rural));
    CHECK_TRUE(set_surface(f, rural, "gravel"));
    const RoadProfile unpaved = profile_of(attributes_of(f.graph, rural));

    // The premise. One strip gained on each side, and the cross-section is wider
    // than it was -- the carriageway itself did not move, the envelope did.
    CHECK_EQ(unpaved.strips.size(), paved.strips.size() + size_t{2});
    CHECK((paved.total_width()) < (unpaved.total_width()));
    CHECK_NEAR(paved.carriageway_width(), unpaved.carriageway_width(), 1e-6);

    // The conclusion. total_width() is the one number junction_trim.cpp's
    // fill_widths() halves into every arm's half_width, so an edit that changes
    // it restages the corridor sweep AND both junction solves. A consumer told
    // "Appearance only" here leaves a stale corridor and stale fillets on screen.
    const SegmentInvalidation invalidation =
        SetSegmentTextCommand(f.graph, rural, SegmentText::Surface, "sett").invalidation();
    CHECK_TRUE(has(invalidation, SegmentInvalidation::Corridor));
    CHECK_TRUE(has(invalidation, SegmentInvalidation::Junctions));

    // Appearance is kept alongside, so a consumer that runs the material pass on
    // its own is still told the variant changed rather than having to infer it.
    CHECK_TRUE(has(invalidation, SegmentInvalidation::Appearance));
    CHECK_FALSE(has(invalidation, SegmentInvalidation::Structures));
}

TEST(SegmentAttrs, every_field_declares_whether_it_moves_geometry) {
    // A sweep of the whole enum, so a field added without an entry in the
    // invalidation table fails here rather than quietly telling B4 that nothing
    // needs re-solving.
    for (uint8_t raw = 0; raw < static_cast<uint8_t>(SegmentField::Count); ++raw) {
        const auto field = static_cast<SegmentField>(raw);

        // Name is the ONLY field build_profile() does not read. Surface stood on
        // this line once and must not stand here again: it decides `unpaved`,
        // and an unpaved carriageway grows a verge. That is measured, not
        // argued, in a_surface_edit_moves_vertices_and_the_table_has_to_say_so.
        const bool geometric = field != SegmentField::Name;
        const SegmentInvalidation invalidation = invalidation_of(field);

        CHECK_TRUE(has(invalidation, SegmentInvalidation::Metadata));
        CHECK_EQ(has(invalidation, SegmentInvalidation::Corridor), geometric);
        CHECK_EQ(has(invalidation, SegmentInvalidation::Junctions), geometric);

        // Appearance is Surface's and nothing else's. A second field claiming it
        // would be claiming a material pass is part of its edit, and the only
        // field that has ever wanted that claim used it to skip the solve.
        CHECK_EQ(has(invalidation, SegmentInvalidation::Appearance),
                 field == SegmentField::Surface);

        // Both names are for people -- one for a log, one for the undo menu --
        // and both have a fallback that belongs to the sentinel alone. A real
        // field reaching either fallback is a field that was added to the enum
        // and to nothing else.
        CHECK_EQ(std::string(field_name(field)) == std::string("Unknown"), false);
        CHECK_EQ(std::string(field_label(field)) == std::string("attribute"), false);
    }

    // The sentinel is not a field. It must not claim to invalidate anything, and
    // it must not borrow a real field's name or label.
    CHECK_TRUE(invalidation_of(SegmentField::Count) == SegmentInvalidation::Nothing);
    CHECK_EQ(std::string(field_name(SegmentField::Count)), std::string("Unknown"));
    CHECK_EQ(std::string(field_label(SegmentField::Count)), std::string("attribute"));
}

TEST(SegmentAttrs, only_the_grade_fields_rebuild_structures) {
    for (uint8_t raw = 0; raw < static_cast<uint8_t>(SegmentField::Count); ++raw) {
        const auto field = static_cast<SegmentField>(raw);
        const bool structural = field == SegmentField::Layer || field == SegmentField::Bridge
                                || field == SegmentField::Tunnel;
        CHECK_EQ(has(invalidation_of(field), SegmentInvalidation::Structures), structural);
    }
}

TEST(SegmentAttrs, a_flag_command_reports_the_flag_it_writes) {
    Fixture f;

    SetSegmentFlagCommand bridge(f.graph, f.west_arm, SegmentFlag::Bridge, true);
    SetSegmentFlagCommand oneway(f.graph, f.west_arm, SegmentFlag::Oneway, true);

    CHECK_TRUE(bridge.field() == SegmentField::Bridge);
    CHECK_TRUE(oneway.field() == SegmentField::Oneway);

    // A bridge builds a deck; a one-way only changes the cross-section. Reporting
    // one flag for every boolean would lose that.
    CHECK_TRUE(has(bridge.invalidation(), SegmentInvalidation::Structures));
    CHECK_FALSE(has(oneway.invalidation(), SegmentInvalidation::Structures));
    CHECK_TRUE(has(oneway.invalidation(), SegmentInvalidation::Junctions));
}

// ============================================================================
// SideFlags: Unknown is not None
// ============================================================================

TEST(SegmentAttrs, unknown_and_none_are_two_different_edits) {
    Fixture f;

    // The west arm's sidewalk tag is ABSENT, which is Unknown: build_profile()
    // may still infer one from the road class.
    CHECK_EQ(side_name(attributes_of(f.graph, f.west_arm).sidewalk), std::string("Unknown"));

    // Setting None is a real edit, not a no-op. An inspector that folds the pair
    // into one "off" state cannot make this edit at all.
    CHECK_TRUE(set_side(f, f.west_arm, SideFeature::Sidewalk, SideFlags::None));
    CHECK_EQ(side_name(attributes_of(f.graph, f.west_arm).sidewalk), std::string("None"));

    // And now it IS a no-op, which is how the refusal proves the value landed.
    CHECK_FALSE(set_side(f, f.west_arm, SideFeature::Sidewalk, SideFlags::None));

    // The reverse is an edit too: taking the explicit no away restores the
    // licence to infer, and that is information the profile builder needs back.
    CHECK_TRUE(set_side(f, f.west_arm, SideFeature::Sidewalk, SideFlags::Unknown));
    CHECK_EQ(side_name(attributes_of(f.graph, f.west_arm).sidewalk), std::string("Unknown"));

    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(side_name(attributes_of(f.graph, f.west_arm).sidewalk), std::string("None"));
}

TEST(SegmentAttrs, a_side_edit_leaves_the_other_three_sides_alone) {
    Fixture f;

    CHECK_TRUE(set_side(f, f.west_arm, SideFeature::Parking, SideFlags::Right));

    const SegmentAttributes after = attributes_of(f.graph, f.west_arm);
    CHECK_EQ(side_name(after.parking), std::string("Right"));
    CHECK_EQ(side_name(after.sidewalk), std::string("Unknown"));
    CHECK_EQ(side_name(after.cycleway), std::string("None"));
    CHECK_EQ(side_name(after.shoulder), std::string("Right"));
}

// ============================================================================
// Text
// ============================================================================

TEST(SegmentAttrs, a_surface_is_trimmed_and_lower_cased_on_the_way_in) {
    Fixture f;

    // road_style.hpp matches lower-case literals, and the parser normalises the
    // same way. A value typed into an inspector has to go through the same door
    // or it matches no material and renders as the default.
    CHECK_TRUE(set_surface(f, f.west_arm, "  Cobblestone "));
    CHECK_EQ(attributes_of(f.graph, f.west_arm).surface, std::string("cobblestone"));
    CHECK_EQ(normalise_surface("  Sett\t"), std::string("sett"));
}

TEST(SegmentAttrs, a_surface_that_differs_only_in_case_is_no_change) {
    Fixture f;

    // Normalisation has to happen BEFORE the no-op test. Doing it afterwards
    // records an undo step for re-typing a value that was already stored.
    CHECK_FALSE(set_surface(f, f.west_arm, "ASPHALT"));
    CHECK_EQ(f.stack.undo_depth(), size_t{0});
}

TEST(SegmentAttrs, a_name_keeps_its_capitalisation) {
    Fixture f;

    // A street name is data, not a lookup key.
    CHECK_TRUE(set_name(f, f.west_arm, "  Marlow Row  "));
    CHECK_EQ(attributes_of(f.graph, f.west_arm).name, std::string("  Marlow Row  "));
}

TEST(SegmentAttrs, describe_names_the_field_it_edits) {
    Fixture f;

    SetSegmentWidthCommand width(f.graph, f.west_arm, 8.0f);
    SetSegmentTextCommand name(f.graph, f.west_arm, SegmentText::Name, "Marlow Row");
    SetSegmentLayerCommand layer(f.graph, f.west_arm, 2);

    CHECK_EQ(width.describe(), std::string("Set width"));
    CHECK_EQ(name.describe(), std::string("Set name"));
    CHECK_EQ(layer.describe(), std::string("Set layer"));

    CHECK_TRUE(set_width(f, f.west_arm, 8.0f));
    CHECK_EQ(f.stack.undo_label(), std::string("Set width"));
}

TEST(SegmentAttrs, footprint_counts_the_text_the_command_holds) {
    Fixture f;

    SetSegmentTextCommand small(f.graph, f.west_arm, SegmentText::Name, "A");
    SetSegmentTextCommand large(f.graph, f.west_arm, SegmentText::Name,
                                std::string(4096, 'x'));

    // The stack's memory bound is only a bound if a command that owns a kilobyte
    // of heap says so. Neither command has applied, so this covers ONE term --
    // SetSegmentTextCommand's own m_value. The base class's two attribute copies
    // are still empty here and are measured in the next test.
    CHECK((small.footprint()) < (large.footprint()));
    CHECK((large.footprint()) >= size_t{4096});
}

TEST(SegmentAttrs, footprint_counts_the_attributes_the_command_captured) {
    Fixture f;

    // The base class's heap is the before/after pair, and the pair does not
    // exist until the command has applied. Measuring an un-applied command
    // measures two default-constructed attribute sets and therefore measures
    // nothing -- which is how a footprint() that dropped both terms survived a
    // review round of this file.
    const std::string long_name(4096, 'x');
    SegmentAttributes attributes = west_attributes();
    attributes.name = long_name;
    const SegmentId labelled = f.add_segment({f.west, f.east}, attributes);
    CHECK_TRUE(labelled != kInvalidSegment);
    f.stack.clear();

    // A width command carries no string of its own, so every byte that appears
    // below comes from the base class.
    SetSegmentWidthCommand command(f.graph, labelled, 8.75f);

    const size_t before_apply = command.footprint();
    CHECK((before_apply) >= sizeof(SegmentAttributeCommand));

    CHECK_TRUE(command.apply());
    const size_t after_apply = command.footprint();

    // Twice the name plus the object itself, because an applied command holds
    // the name on BOTH sides of the edit. A stack trimming to max_bytes is only
    // trimming to a real number if it is told about both copies.
    //
    // Written against sizeof rather than against before_apply on purpose: an
    // empty std::string still reports its small-string capacity, so before_apply
    // already counts a few bytes per string and the growth is a little under
    // exactly twice the name.
    CHECK((after_apply) >= (sizeof(SegmentAttributeCommand) + 2u * long_name.size()));
    CHECK((before_apply) < (after_apply));
}

// ============================================================================
// Dirty marking
// ============================================================================

TEST(SegmentAttrs, an_edit_marks_the_segment_and_the_nodes_at_its_ends) {
    Fixture f;
    f.graph.clear_dirty();

    CHECK_TRUE(set_width(f, f.west_arm, 9.0f));

    CHECK_TRUE(dirty_has_segment(f.graph, f.west_arm));
    CHECK_TRUE(dirty_has_node(f.graph, f.west));
    CHECK_TRUE(dirty_has_node(f.graph, f.hub));

    // The set is conservative: replacing the segment marks its nodes, and a
    // marked node marks every OTHER segment on it. Over-reporting means B4
    // re-solves too much, which is safe; under-reporting would leave stale
    // geometry on screen, which is not. The precise answer is invalidation().
    CHECK_TRUE(dirty_has_segment(f.graph, f.east_arm));
}

TEST(SegmentAttrs, an_undo_marks_what_it_changed_back) {
    Fixture f;

    CHECK_TRUE(set_width(f, f.west_arm, 9.0f));
    f.graph.clear_dirty();

    CHECK_TRUE(f.stack.undo());

    // An undo changes the geometry exactly as much as the edit did. A dirty set
    // that only tracked the forward direction would leave the solved corridor
    // from before the undo on screen.
    CHECK_TRUE(dirty_has_segment(f.graph, f.west_arm));
    CHECK_TRUE(dirty_has_node(f.graph, f.hub));
}

// ============================================================================
// The other field commands
// ============================================================================

TEST(SegmentAttrs, the_road_class_the_layer_and_the_flags_all_round_trip) {
    Fixture f;

    CHECK_TRUE(f.stack.execute(
        std::make_unique<SetSegmentTypeCommand>(f.graph, f.west_arm, RoadType::Primary)));
    f.stack.seal();
    CHECK_TRUE(
        f.stack.execute(std::make_unique<SetSegmentLayerCommand>(f.graph, f.west_arm, -2)));
    f.stack.seal();
    CHECK_TRUE(f.stack.execute(std::make_unique<SetSegmentFlagCommand>(
        f.graph, f.west_arm, SegmentFlag::Tunnel, true)));

    SegmentAttributes expected = west_attributes();
    expected.type = RoadType::Primary;
    expected.layer = -2;
    expected.is_tunnel = true;
    CHECK_TRUE(attributes_of(f.graph, f.west_arm) == expected);

    // Three sealed steps, unwound one at a time and in order.
    CHECK_TRUE(f.stack.undo());
    CHECK_FALSE(attributes_of(f.graph, f.west_arm).is_tunnel);
    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(attributes_of(f.graph, f.west_arm).layer, 0);
    CHECK_TRUE(f.stack.undo());
    CHECK_TRUE(attributes_of(f.graph, f.west_arm) == west_attributes());
}

TEST(SegmentAttrs, setting_a_flag_to_the_value_it_holds_is_refused) {
    Fixture f;

    // The east arm is already one-way. A checkbox that reports its own state
    // back must not fill the undo menu with steps that do nothing.
    CHECK_FALSE(f.stack.execute(std::make_unique<SetSegmentFlagCommand>(
        f.graph, f.east_arm, SegmentFlag::Oneway, true)));
    CHECK_EQ(f.stack.undo_depth(), size_t{0});

    CHECK_TRUE(f.stack.execute(std::make_unique<SetSegmentFlagCommand>(
        f.graph, f.east_arm, SegmentFlag::Oneway, false)));
    CHECK_FALSE(attributes_of(f.graph, f.east_arm).is_oneway);
}

TEST(SegmentAttrs, the_lane_helper_keeps_the_directions_it_was_not_asked_about) {
    Fixture f;

    // A spinner that edits lanes=* must not silently drop lanes:forward.
    CHECK_TRUE(stratum::osm::road::set_lane_total(f.stack, f.graph, f.west_arm, 4));

    const SegmentAttributes after = attributes_of(f.graph, f.west_arm);
    CHECK_EQ(after.lanes, 4);
    CHECK_EQ(after.lanes_forward, 2);
    CHECK_EQ(after.lanes_backward, 1);

    const LaneCounts counts = lane_counts_of(f.graph, f.west_arm);
    CHECK_EQ(counts.total, 4);
    CHECK_EQ(counts.forward, 2);
    CHECK_EQ(counts.backward, 1);
}

TEST(SegmentAttrs, a_lane_edit_on_a_segment_that_is_not_there_is_refused) {
    Fixture f;

    // lane_counts_of() hands back defaults for a missing segment; the command
    // built from them must still refuse rather than create anything.
    CHECK_FALSE(stratum::osm::road::set_lane_total(f.stack, f.graph, kInvalidSegment, 3));
    CHECK_EQ(f.graph.segment_count(), size_t{2});
    CHECK_EQ(f.stack.undo_depth(), size_t{0});
}
