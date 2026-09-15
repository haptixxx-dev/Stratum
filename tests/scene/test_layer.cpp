// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_layer.cpp
 * @brief LayerTree: tree invariants, effective state, refusal, and the undo of
 *        every operation
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Three groups of properties are worth more than the rest, because each one
 * fails silently rather than loudly:
 *
 *   - **Effective state.** A layer in a hidden group that keeps drawing is not a
 *     crash; it is geometry that ignores the user. Every nesting case is checked
 *     against the OWN flag as well, so a test cannot pass by conflating the two.
 *   - **Undo of a populated delete.** Handles, nesting, sibling order and the
 *     group's own position among its siblings all have to come back. Restoring
 *     three of those four looks correct until something else holds a handle.
 *   - **Refusal.** A cycle, a stale handle or a non-group parent must leave both
 *     the tree and the history exactly as they were. A refusal that half-applied
 *     is worse than the edit it refused.
 *
 * Every tree here is built through CommandStack, never through a back door,
 * because "the only way to change the tree is a command" is one of the things
 * under test.
 */

#include "framework.hpp"

#include "scene/command.hpp"
#include "scene/layer.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

using stratum::scene::CommandStack;
using stratum::scene::CreateLayerCommand;
using stratum::scene::DeleteLayerCommand;
using stratum::scene::kAppend;
using stratum::scene::kInvalidLayer;
using stratum::scene::Layer;
using stratum::scene::LayerId;
using stratum::scene::LayerKind;
using stratum::scene::LayerTransform;
using stratum::scene::LayerTree;
using stratum::scene::RenameLayerCommand;
using stratum::scene::ReorderLayerCommand;
using stratum::scene::ReparentLayerCommand;
using stratum::scene::SetLayerColourCommand;
using stratum::scene::SetLayerLockedCommand;
using stratum::scene::SetLayerTransformCommand;
using stratum::scene::SetLayerVisibleCommand;

namespace {

/// Create a layer through the stack and hand back its handle.
LayerId make_layer(LayerTree& tree, CommandStack& stack, LayerKind kind, const std::string& name,
                   LayerId parent = kInvalidLayer, size_t index = kAppend) {
    auto command = std::make_unique<CreateLayerCommand>(tree, kind, name, parent, index);
    const LayerId id = command->layer();
    if (!stack.execute(std::move(command))) return kInvalidLayer;
    return id;
}

LayerId make_group(LayerTree& tree, CommandStack& stack, const std::string& name,
                   LayerId parent = kInvalidLayer, size_t index = kAppend) {
    return make_layer(tree, stack, LayerKind::Group, name, parent, index);
}

LayerId make_shape(LayerTree& tree, CommandStack& stack, const std::string& name,
                   LayerId parent = kInvalidLayer, size_t index = kAppend) {
    return make_layer(tree, stack, LayerKind::Shape, name, parent, index);
}

/// Count everything reachable from @p parent, checking each child's parent link
/// on the way down.
size_t count_reachable(const LayerTree& tree, LayerId parent, bool& ok) {
    size_t total = 0;
    for (LayerId child : tree.children(parent)) {
        const Layer* layer = tree.find(child);
        if (layer == nullptr) {
            ok = false;
            continue;
        }
        if (layer->parent != parent) ok = false;
        total += 1u + count_reachable(tree, child, ok);
    }
    return total;
}

/// Both directions of every link agree, and every layer in the tree is reachable
/// from the top exactly once.
///
/// `children` and `parent` are stored separately -- one for order, one for the
/// walk up -- so they can drift, and a drift shows up as geometry that belongs
/// to a layer the panel does not list.
bool links_agree(const LayerTree& tree) {
    bool ok = true;
    const size_t reachable = count_reachable(tree, kInvalidLayer, ok);
    return ok && reachable == tree.size();
}

/// The ids of @p parent's children, for comparing an order element by element.
std::vector<LayerId> order_of(const LayerTree& tree, LayerId parent) {
    return tree.children(parent);
}

} // namespace

// ============================================================================
// Handles
// ============================================================================

TEST(SceneLayer, a_create_knows_its_handle_before_it_is_executed) {
    LayerTree tree;
    CommandStack stack;

    auto command = std::make_unique<CreateLayerCommand>(tree, LayerKind::Shape, "Buildings");
    const LayerId promised = command->layer();
    CHECK((promised) != (kInvalidLayer));
    CHECK_FALSE(tree.contains(promised));

    CHECK_TRUE(stack.execute(std::move(command)));

    // The id the caller was given is the id the layer has. Anything else and a
    // create-then-configure transaction would configure the wrong layer.
    CHECK_TRUE(tree.contains(promised));
    CHECK_EQ(tree.find(promised)->name, std::string("Buildings"));
    CHECK_EQ(tree.find(promised)->kind, LayerKind::Shape);
}

TEST(SceneLayer, undo_of_a_create_removes_the_layer_and_redo_restores_the_same_handle) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");
    CHECK_EQ(tree.size(), size_t{1});

    CHECK_TRUE(stack.undo());
    CHECK_EQ(tree.size(), size_t{0});
    CHECK_FALSE(tree.contains(id));

    CHECK_TRUE(stack.redo());
    CHECK_EQ(tree.size(), size_t{1});
    CHECK_TRUE(tree.contains(id));
    CHECK_EQ(tree.find(id)->name, std::string("Buildings"));
}

// A recycled handle is the failure a free list of indices produces: an old
// reference silently starts naming a different layer. Ids only ever go up.
TEST(SceneLayer, handles_are_never_reused_after_a_delete) {
    LayerTree tree;
    CommandStack stack;

    const LayerId first = make_shape(tree, stack, "Buildings");
    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, first)));

    const LayerId second = make_shape(tree, stack, "Water");

    CHECK((second) != (first));
    CHECK_FALSE(tree.contains(first));
    CHECK_TRUE(tree.contains(second));
}

TEST(SceneLayer, a_stale_handle_is_detectable_and_answers_safely) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");
    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, id)));

    CHECK_FALSE(tree.contains(id));
    CHECK(tree.find(id) == nullptr);

    // Invisible and locked: nothing draws it, and no tool writes into it.
    CHECK_FALSE(tree.effective_visible(id));
    CHECK_TRUE(tree.effective_locked(id));
    CHECK_EQ(tree.index_of(id), kAppend);
    CHECK_EQ(tree.subtree_ids(id).size(), size_t{0});
}

TEST(SceneLayer, a_handle_survives_insertions_and_removals_of_other_layers) {
    LayerTree tree;
    CommandStack stack;

    const LayerId watched = make_shape(tree, stack, "Buildings");
    const LayerId doomed = make_shape(tree, stack, "Scratch", kInvalidLayer, 0);
    make_shape(tree, stack, "Water", kInvalidLayer, 0);

    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, doomed)));

    CHECK_TRUE(tree.contains(watched));
    CHECK_EQ(tree.find(watched)->name, std::string("Buildings"));
}

// ============================================================================
// Creation and refusal
// ============================================================================

TEST(SceneLayer, creating_under_a_stale_parent_is_refused_and_records_nothing) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, group)));

    const size_t depth_before = stack.undo_depth();
    auto command = std::make_unique<CreateLayerCommand>(tree, LayerKind::Shape, "Orphan", group);
    CHECK_FALSE(stack.execute(std::move(command)));

    CHECK_EQ(stack.undo_depth(), depth_before);
    CHECK_EQ(tree.size(), size_t{0});
}

// Only a Group nests. A shape layer holds shapes, not layers, and letting one
// hold a layer would make "what can I drop here" a question with no answer.
TEST(SceneLayer, creating_under_a_kind_that_is_not_a_group_is_refused) {
    LayerTree tree;
    CommandStack stack;

    const LayerId shapes = make_shape(tree, stack, "Buildings");
    CHECK_FALSE(tree.accepts_children(shapes));

    auto command = std::make_unique<CreateLayerCommand>(tree, LayerKind::Shape, "Nested", shapes);
    CHECK_FALSE(stack.execute(std::move(command)));

    CHECK_EQ(tree.size(), size_t{1});
    CHECK_EQ(tree.children(shapes).size(), size_t{0});
}

TEST(SceneLayer, the_top_level_always_accepts_children) {
    LayerTree tree;
    CHECK_TRUE(tree.accepts_children(kInvalidLayer));
    // kInvalidLayer names the top-level list but is not itself a layer.
    CHECK_FALSE(tree.contains(kInvalidLayer));
}

// ============================================================================
// Sibling order
// ============================================================================

TEST(SceneLayer, siblings_keep_the_order_they_were_created_in) {
    LayerTree tree;
    CommandStack stack;

    const LayerId a = make_shape(tree, stack, "A");
    const LayerId b = make_shape(tree, stack, "B");
    const LayerId c = make_shape(tree, stack, "C");

    const std::vector<LayerId> order = order_of(tree, kInvalidLayer);
    CHECK_EQ(order.size(), size_t{3});
    CHECK_EQ(order[0], a);
    CHECK_EQ(order[1], b);
    CHECK_EQ(order[2], c);
    CHECK_EQ(tree.index_of(b), size_t{1});
}

TEST(SceneLayer, creating_at_an_index_inserts_between_siblings) {
    LayerTree tree;
    CommandStack stack;

    const LayerId a = make_shape(tree, stack, "A");
    const LayerId c = make_shape(tree, stack, "C");
    const LayerId b = make_shape(tree, stack, "B", kInvalidLayer, 1);

    const std::vector<LayerId> order = order_of(tree, kInvalidLayer);
    CHECK_EQ(order.size(), size_t{3});
    CHECK_EQ(order[0], a);
    CHECK_EQ(order[1], b);
    CHECK_EQ(order[2], c);
}

// A redo has to land in the slot the layer had, not at the end of a list that
// has grown since.
TEST(SceneLayer, redo_of_a_create_puts_the_layer_back_in_the_same_slot) {
    LayerTree tree;
    CommandStack stack;

    make_shape(tree, stack, "A");
    make_shape(tree, stack, "C");
    const LayerId b = make_shape(tree, stack, "B", kInvalidLayer, 1);

    CHECK_TRUE(stack.undo());
    CHECK_EQ(tree.size(), size_t{2});

    CHECK_TRUE(stack.redo());
    CHECK_EQ(tree.index_of(b), size_t{1});
}

TEST(SceneLayer, reorder_moves_a_layer_and_undo_puts_it_back) {
    LayerTree tree;
    CommandStack stack;

    const LayerId a = make_shape(tree, stack, "A");
    const LayerId b = make_shape(tree, stack, "B");
    const LayerId c = make_shape(tree, stack, "C");

    // Index counts the position AFTER the move, in the list with A taken out.
    CHECK_TRUE(stack.execute(std::make_unique<ReorderLayerCommand>(tree, a, 2)));

    std::vector<LayerId> order = order_of(tree, kInvalidLayer);
    CHECK_EQ(order[0], b);
    CHECK_EQ(order[1], c);
    CHECK_EQ(order[2], a);

    CHECK_TRUE(stack.undo());

    order = order_of(tree, kInvalidLayer);
    CHECK_EQ(order[0], a);
    CHECK_EQ(order[1], b);
    CHECK_EQ(order[2], c);
    CHECK_TRUE(links_agree(tree));
}

TEST(SceneLayer, reordering_past_the_end_puts_the_layer_last) {
    LayerTree tree;
    CommandStack stack;

    const LayerId a = make_shape(tree, stack, "A");
    make_shape(tree, stack, "B");

    CHECK_TRUE(stack.execute(std::make_unique<ReorderLayerCommand>(tree, a, kAppend)));
    CHECK_EQ(tree.index_of(a), size_t{1});
}

// An undo menu that fills up with steps that do nothing reads as undo being
// broken, so a no-op refuses instead of recording.
TEST(SceneLayer, reordering_to_the_position_it_already_has_is_refused) {
    LayerTree tree;
    CommandStack stack;

    const LayerId a = make_shape(tree, stack, "A");
    make_shape(tree, stack, "B");
    const size_t depth_before = stack.undo_depth();

    CHECK_FALSE(stack.execute(std::make_unique<ReorderLayerCommand>(tree, a, 0)));
    CHECK_EQ(stack.undo_depth(), depth_before);
    CHECK_EQ(tree.index_of(a), size_t{0});
}

// ============================================================================
// Reparenting
// ============================================================================

TEST(SceneLayer, reparent_moves_a_layer_and_undo_restores_its_parent_and_index) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId a = make_shape(tree, stack, "A");
    const LayerId b = make_shape(tree, stack, "B");
    const LayerId c = make_shape(tree, stack, "C");
    (void)a;
    (void)c;

    // B sits between A and C at the top level; it must come back between them.
    CHECK_EQ(tree.index_of(b), size_t{2});

    CHECK_TRUE(stack.execute(std::make_unique<ReparentLayerCommand>(tree, b, group)));
    CHECK_EQ(tree.find(b)->parent, group);
    CHECK_EQ(tree.children(group).size(), size_t{1});
    CHECK_EQ(tree.depth(b), size_t{1});

    CHECK_TRUE(stack.undo());

    CHECK_EQ(tree.find(b)->parent, kInvalidLayer);
    CHECK_EQ(tree.index_of(b), size_t{2});
    CHECK_EQ(tree.children(group).size(), size_t{0});
    CHECK_TRUE(links_agree(tree));
}

TEST(SceneLayer, a_layer_cannot_become_its_own_parent) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const size_t depth_before = stack.undo_depth();

    CHECK_FALSE(tree.can_reparent(group, group));
    CHECK_FALSE(stack.execute(std::make_unique<ReparentLayerCommand>(tree, group, group)));

    CHECK_EQ(stack.undo_depth(), depth_before);
    CHECK_EQ(tree.find(group)->parent, kInvalidLayer);
    CHECK_TRUE(links_agree(tree));
}

// The case that would hang rather than look wrong: every walk up the parent
// chain becomes a loop. It is refused before the tree is touched.
TEST(SceneLayer, a_layer_cannot_be_reparented_under_its_own_descendant) {
    LayerTree tree;
    CommandStack stack;

    const LayerId outer = make_group(tree, stack, "Outer");
    const LayerId inner = make_group(tree, stack, "Inner", outer);
    const LayerId deepest = make_group(tree, stack, "Deepest", inner);

    CHECK_TRUE(tree.is_ancestor_of(outer, deepest));
    CHECK_FALSE(tree.is_ancestor_of(deepest, outer));

    CHECK_FALSE(tree.can_reparent(outer, inner));
    CHECK_FALSE(tree.can_reparent(outer, deepest));

    const size_t depth_before = stack.undo_depth();
    CHECK_FALSE(stack.execute(std::make_unique<ReparentLayerCommand>(tree, outer, deepest)));

    CHECK_EQ(stack.undo_depth(), depth_before);
    CHECK_EQ(tree.find(outer)->parent, kInvalidLayer);
    CHECK_EQ(tree.depth(deepest), size_t{2});
    CHECK_TRUE(links_agree(tree));
}

TEST(SceneLayer, reparenting_under_a_kind_that_is_not_a_group_is_refused) {
    LayerTree tree;
    CommandStack stack;

    const LayerId shapes = make_shape(tree, stack, "Buildings");
    const LayerId roads = make_layer(tree, stack, LayerKind::Graph, "Roads");

    CHECK_FALSE(tree.can_reparent(roads, shapes));
    CHECK_FALSE(stack.execute(std::make_unique<ReparentLayerCommand>(tree, roads, shapes)));
    CHECK_EQ(tree.find(roads)->parent, kInvalidLayer);
}

TEST(SceneLayer, reparenting_into_the_same_place_is_refused_but_a_reorder_is_not) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId a = make_shape(tree, stack, "A", group);
    const LayerId b = make_shape(tree, stack, "B", group);

    CHECK_FALSE(stack.execute(std::make_unique<ReparentLayerCommand>(tree, a, group, 0)));

    // Same parent, different position: a legal move, and the one a drag that
    // stays inside its group emits.
    CHECK_TRUE(stack.execute(std::make_unique<ReparentLayerCommand>(tree, a, group, 1)));
    CHECK_EQ(tree.children(group)[0], b);
    CHECK_EQ(tree.children(group)[1], a);
}

// ============================================================================
// Effective visibility and lock
// ============================================================================

TEST(SceneLayer, a_layer_in_a_hidden_group_is_not_visible_although_its_own_flag_is_set) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId child = make_shape(tree, stack, "Buildings", group);

    CHECK_TRUE(stack.execute(std::make_unique<SetLayerVisibleCommand>(tree, group, false)));

    // The distinction the API exists to make: the child's OWN flag is untouched.
    CHECK_TRUE(tree.find(child)->own_visible);
    CHECK_FALSE(tree.effective_visible(child));
    CHECK_FALSE(tree.effective_visible(group));
}

TEST(SceneLayer, visibility_is_restrictive_through_every_level_of_nesting) {
    LayerTree tree;
    CommandStack stack;

    const LayerId outer = make_group(tree, stack, "Outer");
    const LayerId inner = make_group(tree, stack, "Inner", outer);
    const LayerId leaf = make_shape(tree, stack, "Leaf", inner);

    CHECK_TRUE(tree.effective_visible(leaf));

    // Hiding the OUTER group must reach the grandchild. A one-level check would
    // pass with a broken walk.
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerVisibleCommand>(tree, outer, false)));
    CHECK_FALSE(tree.effective_visible(leaf));
    CHECK_FALSE(tree.effective_visible(inner));

    CHECK_TRUE(stack.undo());
    CHECK_TRUE(tree.effective_visible(leaf));
}

TEST(SceneLayer, a_hidden_layer_stays_hidden_inside_a_visible_group) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId child = make_shape(tree, stack, "Buildings", group);

    CHECK_TRUE(stack.execute(std::make_unique<SetLayerVisibleCommand>(tree, child, false)));
    CHECK_TRUE(tree.effective_visible(group));
    CHECK_FALSE(tree.effective_visible(child));
}

TEST(SceneLayer, a_lock_on_an_ancestor_locks_the_whole_subtree) {
    LayerTree tree;
    CommandStack stack;

    const LayerId outer = make_group(tree, stack, "Outer");
    const LayerId inner = make_group(tree, stack, "Inner", outer);
    const LayerId leaf = make_shape(tree, stack, "Leaf", inner);

    CHECK_FALSE(tree.effective_locked(leaf));

    CHECK_TRUE(stack.execute(std::make_unique<SetLayerLockedCommand>(tree, inner, true)));

    CHECK_FALSE(tree.find(leaf)->own_locked);
    CHECK_TRUE(tree.effective_locked(leaf));
    CHECK_TRUE(tree.effective_locked(inner));
    // The lock reaches down, never up.
    CHECK_FALSE(tree.effective_locked(outer));

    CHECK_TRUE(stack.undo());
    CHECK_FALSE(tree.effective_locked(leaf));
}

TEST(SceneLayer, setting_a_flag_to_the_value_it_already_has_is_refused) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");
    const size_t depth_before = stack.undo_depth();

    CHECK_FALSE(stack.execute(std::make_unique<SetLayerVisibleCommand>(tree, id, true)));
    CHECK_FALSE(stack.execute(std::make_unique<SetLayerLockedCommand>(tree, id, false)));

    CHECK_EQ(stack.undo_depth(), depth_before);
}

// ============================================================================
// Colour
// ============================================================================

TEST(SceneLayer, a_layer_with_no_colour_of_its_own_takes_the_nearest_ancestors) {
    LayerTree tree;
    CommandStack stack;

    const LayerId outer = make_group(tree, stack, "Outer");
    const LayerId inner = make_group(tree, stack, "Inner", outer);
    const LayerId leaf = make_shape(tree, stack, "Leaf", inner);

    const glm::vec3 red(1.0f, 0.0f, 0.0f);
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerColourCommand>(tree, outer, red)));

    const glm::vec3 leaf_colour = tree.effective_colour(leaf);
    CHECK_NEAR(leaf_colour.x, 1.0, 1e-6);
    CHECK_NEAR(leaf_colour.y, 0.0, 1e-6);
    CHECK_NEAR(leaf_colour.z, 0.0, 1e-6);

    // Inherited, not owned: clearing the ancestor puts the leaf back to default.
    CHECK_FALSE(tree.find(leaf)->own_colour.has_value());
}

TEST(SceneLayer, an_explicit_colour_beats_an_inherited_one) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId child = make_shape(tree, stack, "Buildings", group);

    CHECK_TRUE(stack.execute(
        std::make_unique<SetLayerColourCommand>(tree, group, glm::vec3(1.0f, 0.0f, 0.0f))));
    CHECK_TRUE(stack.execute(
        std::make_unique<SetLayerColourCommand>(tree, child, glm::vec3(0.0f, 1.0f, 0.0f))));

    const glm::vec3 child_colour = tree.effective_colour(child);
    CHECK_NEAR(child_colour.x, 0.0, 1e-6);
    CHECK_NEAR(child_colour.y, 1.0, 1e-6);

    // The group keeps its own, so the override is local to the child.
    const glm::vec3 group_colour = tree.effective_colour(group);
    CHECK_NEAR(group_colour.x, 1.0, 1e-6);
}

TEST(SceneLayer, clearing_a_colour_returns_the_layer_to_inheriting) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId child = make_shape(tree, stack, "Buildings", group);

    CHECK_TRUE(stack.execute(
        std::make_unique<SetLayerColourCommand>(tree, group, glm::vec3(1.0f, 0.0f, 0.0f))));
    CHECK_TRUE(stack.execute(
        std::make_unique<SetLayerColourCommand>(tree, child, glm::vec3(0.0f, 1.0f, 0.0f))));

    CHECK_TRUE(stack.execute(
        std::make_unique<SetLayerColourCommand>(tree, child, std::optional<glm::vec3>{})));

    CHECK_FALSE(tree.find(child)->own_colour.has_value());
    CHECK_NEAR(tree.effective_colour(child).x, 1.0, 1e-6);

    // Undo restores the ABSENCE as faithfully as it restores a value.
    CHECK_TRUE(stack.undo());
    CHECK_TRUE(tree.find(child)->own_colour.has_value());
    CHECK_NEAR(tree.effective_colour(child).y, 1.0, 1e-6);
}

TEST(SceneLayer, a_layer_with_no_coloured_ancestor_shows_the_default) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");
    const glm::vec3 shown = tree.effective_colour(id);
    const glm::vec3 fallback = stratum::scene::default_layer_colour();

    CHECK_NEAR(shown.x, fallback.x, 1e-6);
    CHECK_NEAR(shown.y, fallback.y, 1e-6);
    CHECK_NEAR(shown.z, fallback.z, 1e-6);
}

// ============================================================================
// Transform
// ============================================================================

TEST(SceneLayer, a_child_transform_composes_with_its_groups) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId child = make_shape(tree, stack, "Buildings", group);

    LayerTransform group_move;
    group_move.translation = glm::dvec3(10.0, 0.0, 0.0);
    CHECK_TRUE(
        stack.execute(std::make_unique<SetLayerTransformCommand>(tree, group, group_move)));

    LayerTransform child_move;
    child_move.translation = glm::dvec3(0.0, 0.0, 5.0);
    CHECK_TRUE(
        stack.execute(std::make_unique<SetLayerTransformCommand>(tree, child, child_move)));

    // The group's offset reaches the child; the child's does not reach back.
    const glm::dmat4 child_world = tree.effective_transform(child);
    CHECK_NEAR(child_world[3][0], 10.0, 1e-9);
    CHECK_NEAR(child_world[3][2], 5.0, 1e-9);

    const glm::dmat4 group_world = tree.effective_transform(group);
    CHECK_NEAR(group_world[3][0], 10.0, 1e-9);
    CHECK_NEAR(group_world[3][2], 0.0, 1e-9);
}

TEST(SceneLayer, an_untransformed_layer_composes_to_the_identity) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");
    CHECK_TRUE(tree.find(id)->own_transform.is_identity());

    const glm::dmat4 world = tree.effective_transform(id);
    CHECK_NEAR(world[0][0], 1.0, 1e-12);
    CHECK_NEAR(world[1][1], 1.0, 1e-12);
    CHECK_NEAR(world[3][0], 0.0, 1e-12);
}

TEST(SceneLayer, undo_of_a_transform_restores_the_previous_one) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");

    LayerTransform first;
    first.translation = glm::dvec3(1.0, 2.0, 3.0);
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, id, first)));
    stack.seal();

    LayerTransform second;
    second.translation = glm::dvec3(9.0, 9.0, 9.0);
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, id, second)));

    CHECK_TRUE(stack.undo());
    CHECK_NEAR(tree.find(id)->own_transform.translation.x, 1.0, 1e-12);
    CHECK_NEAR(tree.find(id)->own_transform.translation.z, 3.0, 1e-12);
}

// A gizmo drag emits one command per mouse-move. Without merging, undo walks the
// drag back a pixel at a time, which is not undo as anyone means it.
TEST(SceneLayer, a_transform_drag_collapses_into_one_undo_step) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");
    const size_t depth_after_create = stack.undo_depth();

    for (int i = 1; i <= 12; ++i) {
        LayerTransform step;
        step.translation = glm::dvec3(static_cast<double>(i), 0.0, 0.0);
        CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, id, step)));
    }

    CHECK_EQ(stack.undo_depth(), depth_after_create + 1u);
    CHECK_NEAR(tree.find(id)->own_transform.translation.x, 12.0, 1e-12);

    CHECK_TRUE(stack.undo());
    CHECK_NEAR(tree.find(id)->own_transform.translation.x, 0.0, 1e-12);
}

TEST(SceneLayer, a_transform_does_not_merge_across_layers) {
    LayerTree tree;
    CommandStack stack;

    const LayerId a = make_shape(tree, stack, "A");
    const LayerId b = make_shape(tree, stack, "B");
    const size_t depth_before = stack.undo_depth();

    LayerTransform move;
    move.translation = glm::dvec3(1.0, 0.0, 0.0);
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, a, move)));
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, b, move)));

    CHECK_EQ(stack.undo_depth(), depth_before + 2u);
}

// ============================================================================
// Rename
// ============================================================================

TEST(SceneLayer, rename_changes_the_name_and_undo_restores_it) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");
    stack.seal();

    CHECK_TRUE(stack.execute(std::make_unique<RenameLayerCommand>(tree, id, "Footprints")));
    CHECK_EQ(tree.find(id)->name, std::string("Footprints"));

    CHECK_TRUE(stack.undo());
    CHECK_EQ(tree.find(id)->name, std::string("Buildings"));

    CHECK_TRUE(stack.redo());
    CHECK_EQ(tree.find(id)->name, std::string("Footprints"));
}

TEST(SceneLayer, typing_a_name_collapses_into_one_undo_step) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "");
    const size_t depth_after_create = stack.undo_depth();

    for (const std::string& typed : {std::string("R"), std::string("Ro"), std::string("Roa"),
                                     std::string("Road"), std::string("Roads")}) {
        CHECK_TRUE(stack.execute(std::make_unique<RenameLayerCommand>(tree, id, typed)));
    }

    CHECK_EQ(stack.undo_depth(), depth_after_create + 1u);
    CHECK_EQ(tree.find(id)->name, std::string("Roads"));

    // One undo returns to the name from before the first keystroke.
    CHECK_TRUE(stack.undo());
    CHECK_EQ(tree.find(id)->name, std::string(""));
}

TEST(SceneLayer, renaming_to_the_name_it_already_has_is_refused) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");
    const size_t depth_before = stack.undo_depth();

    CHECK_FALSE(stack.execute(std::make_unique<RenameLayerCommand>(tree, id, "Buildings")));
    CHECK_EQ(stack.undo_depth(), depth_before);
}

// ============================================================================
// Delete
// ============================================================================

TEST(SceneLayer, deleting_a_stale_layer_is_refused) {
    LayerTree tree;
    CommandStack stack;

    const LayerId id = make_shape(tree, stack, "Buildings");
    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, id)));

    const size_t depth_before = stack.undo_depth();
    CHECK_FALSE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, id)));
    CHECK_EQ(stack.undo_depth(), depth_before);
}

TEST(SceneLayer, deleting_a_group_takes_its_children_with_it) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId a = make_shape(tree, stack, "A", group);
    const LayerId nested = make_group(tree, stack, "Nested", group);
    const LayerId deep = make_shape(tree, stack, "Deep", nested);
    const LayerId outside = make_shape(tree, stack, "Outside");

    CHECK_EQ(tree.size(), size_t{5});
    CHECK_EQ(tree.subtree_ids(group).size(), size_t{4});

    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, group)));

    CHECK_FALSE(tree.contains(group));
    CHECK_FALSE(tree.contains(a));
    CHECK_FALSE(tree.contains(nested));
    CHECK_FALSE(tree.contains(deep));

    // Nothing outside the subtree is touched.
    CHECK_TRUE(tree.contains(outside));
    CHECK_EQ(tree.size(), size_t{1});
    CHECK_TRUE(links_agree(tree));
}

// The property every other reference to a layer depends on: the handles come
// back, so a selection or an attribute set keyed by LayerId still resolves.
TEST(SceneLayer, undo_of_a_group_delete_restores_every_handle_and_the_whole_structure) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId a = make_shape(tree, stack, "A", group);
    const LayerId b = make_shape(tree, stack, "B", group);
    const LayerId nested = make_group(tree, stack, "Nested", group);
    const LayerId deep = make_shape(tree, stack, "Deep", nested);

    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, group)));
    CHECK_EQ(tree.size(), size_t{0});

    CHECK_TRUE(stack.undo());

    CHECK_EQ(tree.size(), size_t{5});
    CHECK_TRUE(tree.contains(group));
    CHECK_TRUE(tree.contains(a));
    CHECK_TRUE(tree.contains(b));
    CHECK_TRUE(tree.contains(nested));
    CHECK_TRUE(tree.contains(deep));

    // Order among siblings, not just membership.
    const std::vector<LayerId> restored = order_of(tree, group);
    CHECK_EQ(restored.size(), size_t{3});
    CHECK_EQ(restored[0], a);
    CHECK_EQ(restored[1], b);
    CHECK_EQ(restored[2], nested);

    CHECK_EQ(tree.find(deep)->parent, nested);
    CHECK_EQ(tree.depth(deep), size_t{2});
    CHECK_TRUE(links_agree(tree));
}

TEST(SceneLayer, undo_of_a_delete_restores_the_layer_between_the_siblings_it_had) {
    LayerTree tree;
    CommandStack stack;

    const LayerId a = make_shape(tree, stack, "A");
    const LayerId b = make_group(tree, stack, "B");
    const LayerId c = make_shape(tree, stack, "C");
    make_shape(tree, stack, "Inside B", b);

    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, b)));
    CHECK_EQ(order_of(tree, kInvalidLayer).size(), size_t{2});

    CHECK_TRUE(stack.undo());

    const std::vector<LayerId> order = order_of(tree, kInvalidLayer);
    CHECK_EQ(order.size(), size_t{3});
    CHECK_EQ(order[0], a);
    CHECK_EQ(order[1], b);
    CHECK_EQ(order[2], c);
}

TEST(SceneLayer, undo_of_a_delete_restores_the_values_a_layer_carried) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId child = make_shape(tree, stack, "Buildings", group);

    CHECK_TRUE(stack.execute(
        std::make_unique<SetLayerColourCommand>(tree, child, glm::vec3(0.25f, 0.5f, 0.75f))));
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerLockedCommand>(tree, child, true)));

    LayerTransform move;
    move.translation = glm::dvec3(4.0, 0.0, -2.0);
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, child, move)));
    stack.seal();

    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, group)));
    CHECK_TRUE(stack.undo());

    const Layer* restored = tree.find(child);
    CHECK(restored != nullptr);
    if (restored == nullptr) return;

    CHECK_EQ(restored->name, std::string("Buildings"));
    CHECK_TRUE(restored->own_locked);
    CHECK_TRUE(restored->own_colour.has_value());
    CHECK_NEAR(restored->own_colour->y, 0.5, 1e-6);
    CHECK_NEAR(restored->own_transform.translation.x, 4.0, 1e-12);
    CHECK_NEAR(restored->own_transform.translation.z, -2.0, 1e-12);
}

TEST(SceneLayer, redo_of_a_group_delete_removes_the_subtree_again) {
    LayerTree tree;
    CommandStack stack;

    const LayerId group = make_group(tree, stack, "District");
    const LayerId child = make_shape(tree, stack, "Buildings", group);

    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, group)));
    CHECK_TRUE(stack.undo());
    CHECK_EQ(tree.size(), size_t{2});

    CHECK_TRUE(stack.redo());
    CHECK_EQ(tree.size(), size_t{0});
    CHECK_FALSE(tree.contains(child));

    // And once more round, to catch a command that consumed its own undo state.
    CHECK_TRUE(stack.undo());
    CHECK_EQ(tree.size(), size_t{2});
    CHECK_TRUE(tree.contains(child));
    CHECK_TRUE(links_agree(tree));
}

// ============================================================================
// Transactions
// ============================================================================

// The reason CreateLayerCommand reserves its id in the constructor: the caller
// needs the handle to fill the group in the same user action that made it.
TEST(SceneLayer, a_group_and_its_contents_undo_as_one_step) {
    LayerTree tree;
    CommandStack stack;

    stack.begin_transaction("Import district");
    const LayerId group = make_group(tree, stack, "District");
    const LayerId a = make_shape(tree, stack, "Buildings", group);
    const LayerId b = make_layer(tree, stack, LayerKind::Graph, "Roads", group);
    stack.commit_transaction();

    CHECK_EQ(tree.size(), size_t{3});
    CHECK_EQ(stack.undo_depth(), size_t{1});
    CHECK_EQ(stack.undo_label(), std::string("Import district"));

    CHECK_TRUE(stack.undo());
    CHECK_EQ(tree.size(), size_t{0});

    CHECK_TRUE(stack.redo());
    CHECK_EQ(tree.size(), size_t{3});
    CHECK_TRUE(tree.contains(group));
    CHECK_EQ(tree.find(a)->parent, group);
    CHECK_EQ(tree.find(b)->parent, group);
    CHECK_TRUE(links_agree(tree));
}

// ============================================================================
// Whole-tree invariants
// ============================================================================

TEST(SceneLayer, the_two_directions_of_every_link_agree_after_a_run_of_edits) {
    LayerTree tree;
    CommandStack stack;

    const LayerId outer = make_group(tree, stack, "Outer");
    const LayerId inner = make_group(tree, stack, "Inner", outer);
    const LayerId a = make_shape(tree, stack, "A", outer);
    const LayerId b = make_shape(tree, stack, "B", inner);
    const LayerId loose = make_shape(tree, stack, "Loose");

    CHECK_TRUE(stack.execute(std::make_unique<ReparentLayerCommand>(tree, a, inner, 0)));
    CHECK_TRUE(stack.execute(std::make_unique<ReparentLayerCommand>(tree, loose, outer)));
    CHECK_TRUE(stack.execute(std::make_unique<ReorderLayerCommand>(tree, b, 0)));
    CHECK_TRUE(links_agree(tree));

    CHECK_TRUE(stack.execute(std::make_unique<DeleteLayerCommand>(tree, inner)));
    CHECK_TRUE(links_agree(tree));
    CHECK_EQ(tree.size(), size_t{2});

    while (stack.can_undo()) {
        CHECK_TRUE(stack.undo());
    }

    CHECK_EQ(tree.size(), size_t{0});
    CHECK_TRUE(links_agree(tree));
}

TEST(SceneLayer, a_subtree_walk_is_pre_order_and_follows_sibling_order) {
    LayerTree tree;
    CommandStack stack;

    const LayerId root = make_group(tree, stack, "Root");
    const LayerId first = make_group(tree, stack, "First", root);
    const LayerId first_child = make_shape(tree, stack, "FirstChild", first);
    const LayerId second = make_shape(tree, stack, "Second", root);

    const std::vector<LayerId> walk = tree.subtree_ids(root);
    CHECK_EQ(walk.size(), size_t{4});
    CHECK_EQ(walk[0], root);
    CHECK_EQ(walk[1], first);
    CHECK_EQ(walk[2], first_child);
    CHECK_EQ(walk[3], second);
}

// ============================================================================
// Composition order
// ============================================================================
//
// The existing transform tests use translations only, and translations commute.
// An implementation that multiplied the chain the wrong way round, or that
// applied the Euler angles in any order but the documented YXZ, passes every one
// of them. That is the property layer.hpp argues for hardest, and the one place
// a silently wrong answer is most likely to survive into an export.
//
// These pin it with rotations, which do not commute.

TEST(SceneLayer, a_rotated_group_rotates_its_children_about_the_group_origin) {
    LayerTree tree;
    CommandStack stack;

    auto group_cmd = std::make_unique<CreateLayerCommand>(tree, LayerKind::Group, "Group");
    const LayerId group = group_cmd->layer();
    CHECK_TRUE(stack.execute(std::move(group_cmd)));

    auto child_cmd =
        std::make_unique<CreateLayerCommand>(tree, LayerKind::Shape, "Child", group);
    const LayerId child = child_cmd->layer();
    CHECK_TRUE(stack.execute(std::move(child_cmd)));

    // 90 degrees of yaw on the group.
    LayerTransform group_xf;
    group_xf.rotation = glm::dvec3(0.0, glm::half_pi<double>(), 0.0);
    CHECK_TRUE(stack.execute(
        std::make_unique<SetLayerTransformCommand>(tree, group, group_xf)));

    // Child sits 10 m along +X in its own frame.
    LayerTransform child_xf;
    child_xf.translation = glm::dvec3(10.0, 0.0, 0.0);
    CHECK_TRUE(stack.execute(
        std::make_unique<SetLayerTransformCommand>(tree, child, child_xf)));

    // Parent first: +X yawed 90 degrees about Y lands on -Z.
    //
    // Child-first composition gives (10, 0, 0) -- the child's own offset,
    // untouched by the group. The opposite yaw sign gives (0, 0, +10). Both are
    // plausible-looking numbers, which is why this needs asserting rather than
    // eyeballing.
    const glm::dvec4 world =
        tree.effective_transform(child) * glm::dvec4(0.0, 0.0, 0.0, 1.0);

    CHECK_NEAR(world.x, 0.0, 1e-9);
    CHECK_NEAR(world.y, 0.0, 1e-9);
    CHECK_NEAR(world.z, -10.0, 1e-9);
}

TEST(SceneLayer, euler_angles_compose_as_yaw_times_pitch_times_roll) {
    LayerTree tree;
    CommandStack stack;

    auto cmd = std::make_unique<CreateLayerCommand>(tree, LayerKind::Shape, "Turned");
    const LayerId id = cmd->layer();
    CHECK_TRUE(stack.execute(std::move(cmd)));

    // 90 degrees of pitch about X and 90 of yaw about Y, together.
    //
    // "YXZ" names the MULTIPLICATION order, R = Ry * Rx * Rz, so the rightmost
    // factor reaches the vector first: roll, then pitch, then yaw. Pitch before
    // yaw, not after. That distinction is the entire reason the header fixes an
    // order instead of leaving every consumer to pick one.
    //
    // A point on +X is untouched by the pitch -- it lies on the X axis -- and
    // the yaw then carries it to -Z. Compose the other way round (XYZ, so
    // Rx * Ry * Rz) and the yaw moves it to -Z first and the pitch lifts it to
    // +Y instead. Both are unremarkable-looking numbers, which is exactly why a
    // wrong order survives unnoticed until geometry disagrees between the
    // viewport and an export.
    LayerTransform xf;
    xf.rotation = glm::dvec3(glm::half_pi<double>(), glm::half_pi<double>(), 0.0);
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, id, xf)));

    const glm::dvec4 turned = xf.matrix() * glm::dvec4(1.0, 0.0, 0.0, 1.0);

    CHECK_NEAR(turned.x, 0.0, 1e-9);
    CHECK_NEAR(turned.y, 0.0, 1e-9);
    CHECK_NEAR(turned.z, -1.0, 1e-9);
}

// ============================================================================
// Gestures that end where they started
// ============================================================================
//
// Both of these used to leave a command whose old and new values were equal:
// undo did nothing visible, and redo hit apply()'s own equal-value guard, so the
// stack logged a refusal and dropped the step. A dead undo step and a dead redo
// press, in ordinary editing.

TEST(SceneLayer, a_rename_that_returns_to_the_original_does_not_merge) {
    LayerTree tree;
    CommandStack stack;

    auto cmd = std::make_unique<CreateLayerCommand>(tree, LayerKind::Shape, "Original");
    const LayerId id = cmd->layer();
    CHECK_TRUE(stack.execute(std::move(cmd)));
    stack.seal();

    CHECK_TRUE(stack.execute(std::make_unique<RenameLayerCommand>(tree, id, "OriginalX")));
    CHECK_TRUE(stack.execute(std::make_unique<RenameLayerCommand>(tree, id, "Original")));

    // Two honest steps rather than one that does nothing.
    CHECK_EQ(tree.find(id)->name, std::string("Original"));
    CHECK_EQ(stack.undo_depth(), size_t{3});

    // And the redo that used to be dropped now works.
    CHECK_TRUE(stack.undo());
    CHECK_EQ(tree.find(id)->name, std::string("OriginalX"));
    CHECK_TRUE(stack.redo());
    CHECK_EQ(tree.find(id)->name, std::string("Original"));
    CHECK_EQ(stack.redo_depth(), size_t{0});
}

TEST(SceneLayer, a_drag_that_returns_to_the_original_transform_does_not_merge) {
    LayerTree tree;
    CommandStack stack;

    auto cmd = std::make_unique<CreateLayerCommand>(tree, LayerKind::Shape, "Dragged");
    const LayerId id = cmd->layer();
    CHECK_TRUE(stack.execute(std::move(cmd)));
    stack.seal();

    LayerTransform moved;
    moved.translation = glm::dvec3(5.0, 0.0, 0.0);
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, id, moved)));

    const LayerTransform home;  // identity, where it started
    CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, id, home)));

    CHECK_TRUE(tree.find(id)->own_transform.is_identity());
    CHECK_EQ(stack.undo_depth(), size_t{3});

    CHECK_TRUE(stack.undo());
    CHECK_FALSE(tree.find(id)->own_transform.is_identity());
    CHECK_TRUE(stack.redo());
    CHECK_TRUE(tree.find(id)->own_transform.is_identity());
}

// A drag that does NOT return home must still collapse into one step, or the fix
// above has simply broken coalescing.
TEST(SceneLayer, an_ordinary_drag_still_collapses_into_one_step) {
    LayerTree tree;
    CommandStack stack;

    auto cmd = std::make_unique<CreateLayerCommand>(tree, LayerKind::Shape, "Dragged");
    const LayerId id = cmd->layer();
    CHECK_TRUE(stack.execute(std::move(cmd)));
    stack.seal();

    for (int i = 1; i <= 10; ++i) {
        LayerTransform xf;
        xf.translation = glm::dvec3(static_cast<double>(i), 0.0, 0.0);
        CHECK_TRUE(stack.execute(std::make_unique<SetLayerTransformCommand>(tree, id, xf)));
    }

    CHECK_EQ(stack.undo_depth(), size_t{2});
    CHECK_NEAR(tree.find(id)->own_transform.translation.x, 10.0, 1e-9);

    CHECK_TRUE(stack.undo());
    CHECK_TRUE(tree.find(id)->own_transform.is_identity());
}
