// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_selection.cpp
 * @brief Selection: order, staleness, locks, and the filters
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Four groups of properties carry most of the weight, because each one fails
 * quietly rather than loudly:
 *
 *   - **Order.** "Primary" only means something if the sequence does. Every
 *     order assertion here compares the WHOLE sequence as a string, and uses at
 *     least four members where a removal is involved, because a three-member
 *     removal cannot tell an order-preserving implementation apart from a
 *     swap-with-last one: [a,b,c] minus b is [a,c] either way.
 *   - **Staleness.** A selection that hands back a handle to a deleted layer is
 *     not a crash; it is a tool writing into nothing three calls later. The
 *     recycled-slot case is checked against a handle the store genuinely reused,
 *     not a fabricated one.
 *   - **Selection is not undoable.** Two tests pin that decision from both
 *     sides: the stack never grows, and an undone delete brings the layer back
 *     unselected.
 *   - **The filters mean what they say.** The lock filter is checked against a
 *     lock on an ANCESTOR, so a naive own_locked read fails it, and the
 *     attribute filter is checked against a value that only exists on the layer,
 *     so an implementation that resolved with kNoLayer fails it.
 *
 * Every layer here is built through a CommandStack, as A1 requires. The
 * selection itself never touches that stack, which is the point of the test
 * named for it.
 */

#include "framework.hpp"

#include "scene/attributes.hpp"
#include "scene/command.hpp"
#include "scene/layer.hpp"
#include "scene/selection.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using stratum::scene::AttributeKey;
using stratum::scene::AttributeObject;
using stratum::scene::AttributeQuery;
using stratum::scene::AttributeSource;
using stratum::scene::AttributeStatus;
using stratum::scene::AttributeStore;
using stratum::scene::AttributeTarget;
using stratum::scene::AttributeType;
using stratum::scene::AttributeValue;
using stratum::scene::CommandStack;
using stratum::scene::CreateLayerCommand;
using stratum::scene::DeleteLayerCommand;
using stratum::scene::is_kind;
using stratum::scene::is_layer_kind;
using stratum::scene::is_unlocked;
using stratum::scene::kAppend;
using stratum::scene::kInvalidLayer;
using stratum::scene::kNoLayer;
using stratum::scene::LayerId;
using stratum::scene::layer_id_from_ref;
using stratum::scene::LayerKind;
using stratum::scene::layer_ref;
using stratum::scene::LayerRef;
using stratum::scene::LayerTree;
using stratum::scene::Selection;
using stratum::scene::SelectionItem;
using stratum::scene::SelectionItemHash;
using stratum::scene::SelectionKind;
using stratum::scene::SelectionSnapshot;
using stratum::scene::set_attribute;
using stratum::scene::SetLayerLockedCommand;

namespace {

/**
 * @brief The three parts of a document a Selection consults, plus the selection
 *
 * Declared in dependency order so the selection's references are bound to
 * members that already exist.
 */
struct Scene {
    LayerTree tree;
    AttributeStore store;
    CommandStack stack;
    Selection selection{tree, store, stack};

    LayerId add_layer(LayerKind kind, const std::string& name, LayerId parent = kInvalidLayer) {
        auto command = std::make_unique<CreateLayerCommand>(tree, kind, name, parent, kAppend);
        const LayerId id = command->layer();
        if (!stack.execute(std::move(command))) return kInvalidLayer;
        return id;
    }

    LayerId add_shape(const std::string& name, LayerId parent = kInvalidLayer) {
        return add_layer(LayerKind::Shape, name, parent);
    }

    LayerId add_group(const std::string& name, LayerId parent = kInvalidLayer) {
        return add_layer(LayerKind::Group, name, parent);
    }

    bool delete_layer(LayerId id) {
        return stack.execute(std::make_unique<DeleteLayerCommand>(tree, id));
    }

    bool lock_layer(LayerId id) {
        return stack.execute(std::make_unique<SetLayerLockedCommand>(tree, id, true));
    }

    AttributeObject add_object() { return store.create_object(); }

    /// Put a double on an object's Object slot, through the stack as A3 requires.
    bool set_double(AttributeObject obj, AttributeKey key, double value) {
        return set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(value));
    }

    bool set_layer_double(LayerId id, AttributeKey key, double value) {
        return set_attribute(stack, store, AttributeTarget::layer(layer_ref(id), key),
                             AttributeValue::from_double(value));
    }

    bool set_string(AttributeObject obj, AttributeKey key, const std::string& value) {
        return set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_string(value));
    }
};

SelectionItem layer_item(LayerId id) {
    return SelectionItem::of_layer(id);
}

SelectionItem object_item(AttributeObject obj) {
    return SelectionItem::of_object(obj);
}

/// One member, short enough to read in a failure line: "L3", "O0.1", "-".
std::string spell(const SelectionItem& item) {
    if (!item.valid()) return "-";
    if (item.kind == SelectionKind::Layer) return "L" + std::to_string(item.layer);
    return "O" + std::to_string(item.object.index) + "." + std::to_string(item.object.generation);
}

/// The expected sequence, written the same way the actual one is rendered.
std::string spell_all(const std::vector<SelectionItem>& items) {
    std::string text;
    for (const SelectionItem& item : items) {
        if (!text.empty()) text += ',';
        text += spell(item);
    }
    return text;
}

/// The selection's whole sequence, in order. Compared as one string so a test
/// fails on a REORDER and not only on a missing member.
std::string spell_order(const Selection& selection) {
    std::string text;
    for (const SelectionItem& item : selection) {
        if (!text.empty()) text += ',';
        text += spell(item);
    }
    return text;
}

/// Matches a resolved double strictly greater than @p threshold. Reads the
/// status and the TYPE, not just the number, so a string-valued attribute is a
/// miss rather than a zero.
stratum::scene::AttributePredicate double_above(double threshold) {
    return [threshold](const AttributeQuery& query) {
        if (!query.ok()) return false;
        if (!query.value->is(AttributeType::Double)) return false;
        return *query.value->as_double() > threshold;
    };
}

} // namespace

// ============================================================================
// Handles and identity
// ============================================================================

TEST(Selection, a_default_item_names_nothing) {
    CHECK_FALSE(SelectionItem{}.valid());
    CHECK_FALSE(SelectionItem::of_layer(kInvalidLayer).valid());
    CHECK_FALSE(SelectionItem::of_object(AttributeObject{}).valid());

    CHECK_TRUE(SelectionItem::of_layer(1).valid());
    CHECK_TRUE(SelectionItem::of_object(AttributeObject{0u, 0u}).valid());
}

TEST(Selection, a_layer_and_an_object_with_the_same_number_are_different_members) {
    const SelectionItem as_layer = SelectionItem::of_layer(5);
    const SelectionItem as_object = SelectionItem::of_object(AttributeObject{5u, 0u});

    CHECK_TRUE(as_layer != as_object);

    // The unused payload keeps its invalid sentinel, which is what keeps the
    // two apart before the kind is even looked at: as_layer holds no object
    // index and as_object holds no layer id.
    CHECK_EQ(as_layer.object.valid(), false);
    CHECK_EQ(as_object.layer, kInvalidLayer);

    // A hash collision here would still be CORRECT -- operator== separates them
    // -- but it would put two members in every bucket of a scene with one layer
    // per object.
    CHECK_TRUE(SelectionItemHash{}(as_layer) != SelectionItemHash{}(as_object));

    // ...and now the field the test is named for, on its own. The pair above
    // cannot isolate it: with the sentinels in place the two PAYLOADS already
    // differ, so dropping `kind` from operator== and from the hash still told
    // them apart, and this test still passed. These two differ in nothing else.
    // No factory mints them, which is the point -- what has to survive a change
    // of encoding is the comparison, not the factories.
    SelectionItem both_payloads;
    both_payloads.kind = SelectionKind::Layer;
    both_payloads.layer = 5;
    both_payloads.object = AttributeObject{5u, 0u};
    SelectionItem same_but_an_object = both_payloads;
    same_but_an_object.kind = SelectionKind::Object;

    CHECK_TRUE(both_payloads != same_but_an_object);
    CHECK_TRUE(SelectionItemHash{}(both_payloads) != SelectionItemHash{}(same_but_an_object));
}

TEST(Selection, two_generations_of_one_object_slot_are_different_members) {
    const SelectionItem first = SelectionItem::of_object(AttributeObject{7u, 0u});
    const SelectionItem second = SelectionItem::of_object(AttributeObject{7u, 1u});

    // This is the assertion that stops a destroyed object's handle aliasing the
    // object that took over its slot. An equality or a hash that looked only at
    // the index would pass every other test in this file and fail here.
    CHECK_TRUE(first != second);
    CHECK_TRUE(SelectionItemHash{}(first) != SelectionItemHash{}(second));
}

TEST(Selection, layer_ref_round_trips_and_refuses_a_value_that_does_not_fit) {
    CHECK_EQ(layer_ref(7), LayerRef{7});
    CHECK_EQ(layer_id_from_ref(layer_ref(7)), LayerId{7});

    // The two "nothing" spellings are both zero, which is what makes the join a
    // plain cast; see attributes.hpp.
    CHECK_EQ(layer_ref(kInvalidLayer), kNoLayer);
    CHECK_EQ(layer_id_from_ref(kNoLayer), kInvalidLayer);

    // A ref carrying more than a LayerId holds must not truncate into a valid
    // id: 0x1'0000'0007 truncated is layer 7, which is a wrong answer, not a
    // missing one.
    CHECK_EQ(layer_id_from_ref(LayerRef{0x100000007ull}), kInvalidLayer);
}

// ============================================================================
// Adding, order and the primary
// ============================================================================

TEST(Selection, an_empty_selection_has_no_members_and_no_primary) {
    Scene scene;

    CHECK_TRUE(scene.selection.empty());
    CHECK_EQ(scene.selection.size(), size_t{0});
    CHECK_FALSE(scene.selection.primary().valid());
    CHECK_EQ(spell_order(scene.selection), std::string{});
    CHECK_TRUE(scene.selection.begin() == scene.selection.end());
}

TEST(Selection, add_refuses_a_handle_the_scene_does_not_have) {
    Scene scene;
    const LayerId real = scene.add_shape("real");

    CHECK_FALSE(scene.selection.add(layer_item(9999)));
    CHECK_FALSE(scene.selection.add(layer_item(kInvalidLayer)));
    CHECK_FALSE(scene.selection.add(object_item(AttributeObject{4u, 0u})));
    CHECK_EQ(scene.selection.size(), size_t{0});

    CHECK_TRUE(scene.selection.add(layer_item(real)));

    // A handle that WAS good is refused once the layer is gone.
    const LayerId doomed = scene.add_shape("doomed");
    CHECK_TRUE(scene.delete_layer(doomed));
    CHECK_FALSE(scene.selection.add(layer_item(doomed)));
    CHECK_EQ(scene.selection.size(), size_t{1});
}

TEST(Selection, order_is_the_order_things_were_added) {
    Scene scene;
    const LayerId first = scene.add_shape("first");
    const LayerId second = scene.add_shape("second");
    const LayerId third = scene.add_shape("third");
    const LayerId fourth = scene.add_shape("fourth");

    // Deliberately not in id order. An implementation that sorted by handle
    // would pass an ascending sequence and fail this one.
    CHECK_TRUE(scene.selection.add(layer_item(third)));
    CHECK_TRUE(scene.selection.add(layer_item(first)));
    CHECK_TRUE(scene.selection.add(layer_item(fourth)));
    CHECK_TRUE(scene.selection.add(layer_item(second)));

    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(third), layer_item(first), layer_item(fourth),
                        layer_item(second)}));
    CHECK_EQ(scene.selection.size(), size_t{4});
}

TEST(Selection, the_primary_is_the_most_recent_member_and_not_the_first) {
    Scene scene;
    const LayerId first = scene.add_shape("first");
    const LayerId second = scene.add_shape("second");
    const LayerId third = scene.add_shape("third");

    scene.selection.add(layer_item(first));
    scene.selection.add(layer_item(second));
    scene.selection.add(layer_item(third));

    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(third)));

    // Stated the other way round as well: a "primary is the first one" reading
    // passes the check above only if the caller happened to add one member.
    CHECK_TRUE(scene.selection.primary() != layer_item(first));
}

TEST(Selection, adding_a_member_that_is_already_selected_changes_nothing) {
    Scene scene;
    const LayerId first = scene.add_shape("first");
    const LayerId second = scene.add_shape("second");
    const LayerId third = scene.add_shape("third");

    scene.selection.add(layer_item(first));
    scene.selection.add(layer_item(second));
    scene.selection.add(layer_item(third));

    CHECK_FALSE(scene.selection.add(layer_item(first)));

    CHECK_EQ(scene.selection.size(), size_t{3});
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(first), layer_item(second), layer_item(third)}));

    // The re-add must NOT promote: promotion is make_primary()'s job.
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(third)));
}

TEST(Selection, contains_answers_for_members_and_for_everything_else) {
    Scene scene;
    const LayerId selected = scene.add_shape("selected");
    const LayerId ignored = scene.add_shape("ignored");
    const AttributeObject object = scene.add_object();

    scene.selection.add(layer_item(selected));

    CHECK_TRUE(scene.selection.contains(layer_item(selected)));
    CHECK_FALSE(scene.selection.contains(layer_item(ignored)));
    CHECK_FALSE(scene.selection.contains(object_item(object)));
    CHECK_FALSE(scene.selection.contains(SelectionItem{}));
}

// ============================================================================
// Removing, toggling and replacing
// ============================================================================

TEST(Selection, removing_a_member_keeps_the_order_of_the_rest) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");
    const LayerId d = scene.add_shape("d");

    scene.selection.add(layer_item(a));
    scene.selection.add(layer_item(b));
    scene.selection.add(layer_item(c));
    scene.selection.add(layer_item(d));

    CHECK_TRUE(scene.selection.remove(layer_item(b)));

    // Four members on purpose. With three, a swap-with-last implementation and
    // an order-preserving one produce the same sequence, and the test cannot
    // fail. Here they differ: swap-with-last gives L1,L4,L3.
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(a), layer_item(c), layer_item(d)}));
    CHECK_EQ(scene.selection.size(), size_t{3});
    CHECK_FALSE(scene.selection.contains(layer_item(b)));
}

TEST(Selection, removing_the_primary_falls_back_to_the_member_before_it) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.add(layer_item(a));
    scene.selection.add(layer_item(b));
    scene.selection.add(layer_item(c));

    CHECK_TRUE(scene.selection.remove(layer_item(c)));

    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(b)));
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(a), layer_item(b)}));
}

TEST(Selection, removing_what_is_not_selected_reports_that_it_was_not) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");

    scene.selection.add(layer_item(a));

    CHECK_FALSE(scene.selection.remove(layer_item(b)));
    CHECK_TRUE(scene.selection.remove(layer_item(a)));
    CHECK_FALSE(scene.selection.remove(layer_item(a)));
    CHECK_TRUE(scene.selection.empty());
    CHECK_FALSE(scene.selection.primary().valid());
}

TEST(Selection, toggling_a_member_off_and_on_puts_it_at_the_end) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.add(layer_item(a));
    scene.selection.add(layer_item(b));
    scene.selection.add(layer_item(c));

    CHECK_FALSE(scene.selection.toggle(layer_item(a)));
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(b), layer_item(c)}));

    CHECK_TRUE(scene.selection.toggle(layer_item(a)));

    // Back at the END, not back where it was. An implementation that remembered
    // the old position would give L1,L2,L3 here and a primary of c.
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(b), layer_item(c), layer_item(a)}));
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(a)));
}

TEST(Selection, toggling_something_the_scene_does_not_have_selects_nothing) {
    Scene scene;
    const LayerId gone = scene.add_shape("gone");
    CHECK_TRUE(scene.delete_layer(gone));

    CHECK_FALSE(scene.selection.toggle(layer_item(gone)));
    CHECK_FALSE(scene.selection.toggle(layer_item(9999)));
    CHECK_TRUE(scene.selection.empty());
}

TEST(Selection, replace_drops_everything_before_selecting) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.add(layer_item(a));
    scene.selection.add(layer_item(b));

    CHECK_TRUE(scene.selection.replace(layer_item(c)));

    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(c)}));
    CHECK_EQ(scene.selection.size(), size_t{1});
    CHECK_FALSE(scene.selection.contains(layer_item(a)));
}

TEST(Selection, replacing_with_a_list_keeps_its_order_and_skips_what_is_gone) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.add(layer_item(a));
    CHECK_TRUE(scene.delete_layer(b));

    CHECK_EQ(scene.selection.replace({layer_item(c), layer_item(b), layer_item(a)}), size_t{2});

    // The order given, minus the member the scene lost. A short count is how the
    // caller learns its picking list was stale.
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(c), layer_item(a)}));
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(a)));
}

TEST(Selection, add_all_adds_on_top_and_reports_only_what_was_new) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.add(layer_item(a));

    CHECK_EQ(scene.selection.add_all({layer_item(b), layer_item(a), layer_item(c)}), size_t{2});

    // `a` was already there, so it keeps its position at the front rather than
    // being re-appended in the middle of the batch.
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(a), layer_item(b), layer_item(c)}));
}

TEST(Selection, make_primary_moves_a_member_to_the_end_and_refuses_a_non_member) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");
    const LayerId outside = scene.add_shape("outside");

    scene.selection.add(layer_item(a));
    scene.selection.add(layer_item(b));
    scene.selection.add(layer_item(c));

    CHECK_TRUE(scene.selection.make_primary(layer_item(a)));
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(b), layer_item(c), layer_item(a)}));
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(a)));
    CHECK_EQ(scene.selection.size(), size_t{3});

    // Promoting the primary again is a no-op, not a second copy at the end.
    CHECK_TRUE(scene.selection.make_primary(layer_item(a)));
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(b), layer_item(c), layer_item(a)}));

    // A promote never selects. That is add()'s job.
    CHECK_FALSE(scene.selection.make_primary(layer_item(outside)));
    CHECK_FALSE(scene.selection.contains(layer_item(outside)));
}

TEST(Selection, clear_empties_the_selection_and_leaves_no_primary) {
    Scene scene;
    scene.selection.add(layer_item(scene.add_shape("a")));
    scene.selection.add(layer_item(scene.add_shape("b")));

    scene.selection.clear();

    CHECK_TRUE(scene.selection.empty());
    CHECK_EQ(scene.selection.size(), size_t{0});
    CHECK_EQ(scene.selection.count_of(SelectionKind::Layer), size_t{0});
    CHECK_FALSE(scene.selection.primary().valid());
    CHECK_EQ(spell_order(scene.selection), std::string{});
}

// ============================================================================
// Stale members
// ============================================================================

TEST(Selection, a_deleted_layer_leaves_the_selection) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.add(layer_item(a));
    scene.selection.add(layer_item(b));
    scene.selection.add(layer_item(c));

    CHECK_TRUE(scene.delete_layer(b));

    CHECK_FALSE(scene.selection.contains(layer_item(b)));
    CHECK_EQ(scene.selection.size(), size_t{2});
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(a), layer_item(c)}));
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(c)));
}

TEST(Selection, deleting_the_primary_leaves_the_member_before_it_as_primary) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.add(layer_item(a));
    scene.selection.add(layer_item(b));
    scene.selection.add(layer_item(c));

    CHECK_TRUE(scene.delete_layer(c));

    // Three members, not two. With two, the survivor is both the first member
    // and the last one, so a primary() that answered "the FIRST live member"
    // passed this test while failing eleven others -- proven by mutation, which
    // is why the third member is here. The delete path reaches primary()
    // differently from remove(), so removing_the_primary_falls_back_to_the_
    // member_before_it does not cover this.
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(b)));
    CHECK_TRUE(scene.selection.primary() != layer_item(a));
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(a), layer_item(b)}));
    CHECK_EQ(scene.selection.size(), size_t{2});
}

TEST(Selection, deleting_a_group_takes_its_selected_children_with_it) {
    Scene scene;
    const LayerId group = scene.add_group("group");
    const LayerId inside = scene.add_shape("inside", group);
    const LayerId outside = scene.add_shape("outside");

    scene.selection.add(layer_item(inside));
    scene.selection.add(layer_item(outside));

    // A1 deletes a group's whole subtree, so the child's handle dies too even
    // though nothing pointed the delete at the child.
    CHECK_TRUE(scene.delete_layer(group));

    CHECK_FALSE(scene.selection.contains(layer_item(inside)));
    CHECK_TRUE(scene.selection.contains(layer_item(outside)));
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(outside)}));
}

TEST(Selection, undoing_a_delete_brings_the_layer_back_unselected) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    scene.selection.add(layer_item(a));

    CHECK_TRUE(scene.delete_layer(a));
    CHECK_FALSE(scene.selection.contains(layer_item(a)));

    CHECK_TRUE(scene.stack.undo());

    // The layer is back -- A1 restores the same handle verbatim.
    CHECK_TRUE(scene.tree.contains(a));

    // The selection is not. Undo restored the document, and the selection is
    // not the document; see the decision in selection.hpp. This is the test
    // that would fail if selection quietly acquired a history.
    CHECK_FALSE(scene.selection.contains(layer_item(a)));
    CHECK_TRUE(scene.selection.empty());
}

TEST(Selection, a_destroyed_object_leaves_the_selection) {
    Scene scene;
    const AttributeObject kept = scene.add_object();
    const AttributeObject doomed = scene.add_object();

    scene.selection.add(object_item(kept));
    scene.selection.add(object_item(doomed));
    CHECK_EQ(scene.selection.size(), size_t{2});

    CHECK_TRUE(scene.store.destroy_object(doomed));

    // destroy_object() is not a command (see attributes.hpp), so the history's
    // revision does not move. The live object count does, which is why it is
    // part of the change token.
    CHECK_FALSE(scene.selection.contains(object_item(doomed)));
    CHECK_EQ(scene.selection.size(), size_t{1});
    CHECK_EQ(spell_order(scene.selection), spell_all({object_item(kept)}));
}

TEST(Selection, a_recycled_object_slot_is_not_the_object_that_was_selected) {
    Scene scene;
    const AttributeObject first = scene.add_object();
    scene.selection.add(object_item(first));
    CHECK_TRUE(scene.selection.contains(object_item(first)));

    CHECK_TRUE(scene.store.destroy_object(first));
    const AttributeObject second = scene.add_object();

    // A3 hands the slot back out with the generation bumped. Without that, this
    // whole test would be checking nothing, so assert the premise.
    CHECK_EQ(second.index, first.index);
    CHECK_TRUE(second.generation != first.generation);

    // Neither the dead handle nor the object that took its slot is selected.
    CHECK_FALSE(scene.selection.contains(object_item(first)));
    CHECK_FALSE(scene.selection.contains(object_item(second)));
    CHECK_EQ(spell_order(scene.selection), std::string{});
    CHECK_TRUE(scene.selection.empty());

    // The documented residue: a destroy and a create with no read in between
    // leaves the live object count level, so the COUNT is still stale until
    // something sweeps. Nothing was handed out, which is the guarantee that
    // matters; prune() squares the books.
    CHECK_EQ(scene.selection.size(), size_t{1});
    CHECK_EQ(scene.selection.prune(), size_t{1});
    CHECK_EQ(scene.selection.size(), size_t{0});
}

TEST(Selection, prune_reports_how_many_members_the_scene_had_lost) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.replace({layer_item(a), layer_item(b), layer_item(c)});

    CHECK_TRUE(scene.delete_layer(a));
    CHECK_TRUE(scene.delete_layer(c));

    // Nothing has read the selection since the deletes, so prune() is the first
    // thing to notice them and reports both.
    CHECK_EQ(scene.selection.prune(), size_t{2});
    CHECK_EQ(scene.selection.prune(), size_t{0});
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(b)}));
}

// ============================================================================
// Iterating while the scene changes underneath
//
// Every other pass in this file is one uninterrupted walk over a selection
// nothing touches, which is why none of them could see that a const accessor
// used to compact or shrink the storage under an outstanding iterator. These
// four change the scene, or read the selection, from INSIDE the loop.
// ============================================================================

TEST(Selection, iterating_across_a_delete_visits_every_survivor_exactly_once) {
    Scene scene;

    std::vector<SelectionItem> everything;
    for (size_t i = 0; i < 6; ++i) {
        everything.push_back(layer_item(scene.add_shape("layer" + std::to_string(i))));
    }
    CHECK_EQ(scene.selection.replace(everything), size_t{6});

    // Delete the member the pass has ALREADY yielded, then read the selection.
    // That is the sequence a sweep from a const accessor could not survive: it
    // compacted m_order by moving every survivor down one slot, and the cursor
    // then stepped over the next member entirely.
    std::vector<SelectionItem> visited;
    size_t guard = 0;
    for (const SelectionItem& item : scene.selection) {
        visited.push_back(item);
        if (visited.size() == 1) {
            CHECK_TRUE(scene.delete_layer(item.layer));
            CHECK_EQ(scene.selection.size(), size_t{5});
        }
        if (++guard > size_t{20}) break;
    }

    // The loop has to end on its own. A cursor that walks past a shrunken
    // vector never reaches the end position it was compared against.
    CHECK_TRUE(guard <= size_t{20});

    // The first member was handed over one statement before it died, while it
    // was still live; every other member is a survivor, and each is visited
    // once, in selection order.
    CHECK_EQ(spell_all(visited), spell_all(everything));
}

TEST(Selection, iterating_across_a_destroyed_object_visits_every_survivor_exactly_once) {
    Scene scene;

    std::vector<AttributeObject> created;
    std::vector<SelectionItem> everything;
    for (size_t i = 0; i < 5; ++i) {
        created.push_back(scene.add_object());
        everything.push_back(object_item(created.back()));
    }
    CHECK_EQ(scene.selection.replace(everything), size_t{5});

    std::vector<SelectionItem> visited;
    size_t guard = 0;
    for (const SelectionItem& item : scene.selection) {
        visited.push_back(item);
        if (visited.size() == 2) {
            // destroy_object() is not a command, so this moves the change token
            // through the live object count rather than the revision -- a
            // different path into the same sweep than the delete above.
            CHECK_TRUE(scene.store.destroy_object(created[0]));
            CHECK_EQ(scene.selection.size(), size_t{4});
        }
        if (++guard > size_t{20}) break;
    }
    CHECK_TRUE(guard <= size_t{20});

    CHECK_EQ(spell_all(visited), spell_all(everything));
}

TEST(Selection, a_member_that_dies_before_the_pass_reaches_it_is_never_yielded) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId doomed = scene.add_shape("doomed");
    const LayerId d = scene.add_shape("d");

    CHECK_EQ(scene.selection.replace(
                 {layer_item(a), layer_item(b), layer_item(doomed), layer_item(d)}),
             size_t{4});

    std::vector<SelectionItem> visited;
    size_t guard = 0;
    for (const SelectionItem& item : scene.selection) {
        // The guarantee, checked on every member as it arrives rather than only
        // at the end: nothing the scene no longer has is ever handed over.
        CHECK_TRUE(scene.selection.is_live(item));
        visited.push_back(item);
        if (visited.size() == 1) {
            // A member the pass has not reached yet, so it must simply never
            // appear. The size() read is what used to move the rest of the
            // members out from under the cursor.
            CHECK_TRUE(scene.delete_layer(doomed));
            CHECK_EQ(scene.selection.size(), size_t{3});
        }
        if (++guard > size_t{20}) break;
    }
    CHECK_TRUE(guard <= size_t{20});

    CHECK_EQ(spell_all(visited), spell_all({layer_item(a), layer_item(b), layer_item(d)}));
}

TEST(Selection, calling_primary_inside_a_pass_does_not_disturb_the_pass) {
    Scene scene;
    const AttributeObject kept = scene.add_object();
    const AttributeObject first = scene.add_object();
    const AttributeObject second = scene.add_object();
    const AttributeObject third = scene.add_object();

    CHECK_EQ(scene.selection.replace({object_item(kept), object_item(first),
                                      object_item(second), object_item(third)}),
             size_t{4});

    CHECK_TRUE(scene.store.destroy_object(first));
    CHECK_TRUE(scene.store.destroy_object(second));
    CHECK_TRUE(scene.store.destroy_object(third));

    // Three creates put the live object count back where it was, so the change
    // token reads level and the selection still holds three dead members: the
    // residue window the file comment documents. It is also the window in which
    // primary() used to pop those three members off the back -- from a const
    // accessor, with a pass outstanding.
    (void)scene.add_object();
    (void)scene.add_object();
    (void)scene.add_object();

    std::vector<SelectionItem> visited;
    size_t guard = 0;
    for (const SelectionItem& item : scene.selection) {
        visited.push_back(item);

        // The inspector's highlight loop, which is what this iterator exists
        // for: "is this member the active one" asked once per member.
        CHECK_EQ(spell(scene.selection.primary()), spell(object_item(kept)));
        if (++guard > size_t{20}) break;
    }
    CHECK_TRUE(guard <= size_t{20});

    CHECK_EQ(spell_all(visited), spell_all({object_item(kept)}));

    // And nothing about the pass changed the selection: the three dead members
    // are still held, still not handed out, and prune() is still the thing that
    // squares the books.
    CHECK_EQ(scene.selection.size(), size_t{4});
    CHECK_EQ(scene.selection.prune(), size_t{3});
    CHECK_EQ(scene.selection.size(), size_t{1});
}

TEST(Selection, an_iterator_belongs_to_the_selection_it_came_from) {
    Scene one;
    Scene two;
    one.selection.add(layer_item(one.add_shape("a")));
    two.selection.add(layer_item(two.add_shape("b")));

    // Position is not identity. Both selections hold one member in slot 0 and
    // both end one past it, so an operator== that compared only the index
    // called these pairs equal -- and a loop bounded by the wrong selection's
    // end() walks off whichever one is shorter.
    CHECK_TRUE(one.selection.begin() != two.selection.begin());
    CHECK_TRUE(one.selection.end() != two.selection.end());

    // A default-constructed iterator names no selection, so it is the beginning
    // of nothing; two of them are equal, which is what a forward iterator owes.
    const Selection::const_iterator nowhere;
    CHECK_TRUE(nowhere != one.selection.begin());
    CHECK_TRUE(nowhere == Selection::const_iterator{});

    // The ordinary answers are unchanged inside one selection.
    CHECK_TRUE(one.selection.begin() != one.selection.end());
    Scene nothing_selected;
    CHECK_TRUE(nothing_selected.selection.begin() == nothing_selected.selection.end());
}

// ============================================================================
// Selection is not undoable
// ============================================================================

TEST(Selection, no_selection_operation_reaches_the_command_stack) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    const size_t depth_before = scene.stack.undo_depth();
    const uint64_t revision_before = scene.stack.revision();

    scene.selection.add(layer_item(a));
    scene.selection.add(layer_item(b));
    scene.selection.toggle(layer_item(c));
    scene.selection.toggle(layer_item(a));
    scene.selection.make_primary(layer_item(b));
    scene.selection.remove(layer_item(b));
    scene.selection.replace({layer_item(a), layer_item(c)});
    scene.selection.retain(is_kind(SelectionKind::Layer));
    scene.selection.restore(scene.selection.snapshot());
    scene.selection.clear();

    // Ten selection operations, no history. Ctrl-Z after this must still undo
    // the last layer that was CREATED.
    CHECK_EQ(scene.stack.undo_depth(), depth_before);
    CHECK_EQ(scene.stack.revision(), revision_before);
    CHECK_EQ(scene.stack.undo_label(), std::string{"Create layer"});
}

// ============================================================================
// Snapshots
// ============================================================================

TEST(Selection, a_snapshot_round_trips_the_order_and_the_primary) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.replace({layer_item(c), layer_item(a), layer_item(b)});
    const SelectionSnapshot taken = scene.selection.snapshot();
    CHECK_EQ(taken.items.size(), size_t{3});
    CHECK_FALSE(taken.empty());

    scene.selection.clear();
    CHECK_TRUE(scene.selection.empty());

    CHECK_EQ(scene.selection.restore(taken), size_t{3});
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(c), layer_item(a), layer_item(b)}));
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(b)));
}

TEST(Selection, restoring_a_snapshot_skips_what_the_scene_lost_in_between) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");

    scene.selection.replace({layer_item(a), layer_item(b), layer_item(c)});
    const SelectionSnapshot taken = scene.selection.snapshot();

    scene.selection.clear();
    CHECK_TRUE(scene.delete_layer(b));

    // A command that carries a selection in its own inverse must not resurrect
    // a handle on the way back.
    CHECK_EQ(scene.selection.restore(taken), size_t{2});
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(a), layer_item(c)}));
    CHECK_FALSE(scene.selection.contains(layer_item(b)));
}

// ============================================================================
// Locked layers
// ============================================================================

TEST(Selection, a_locked_layer_can_still_be_selected) {
    Scene scene;
    const LayerId free_layer = scene.add_shape("free");
    const LayerId locked = scene.add_shape("locked");
    CHECK_TRUE(scene.lock_layer(locked));
    CHECK_TRUE(scene.tree.effective_locked(locked));

    // Lock guards edits, not selection. Clicking a locked layer is how a user
    // finds out why it will not move.
    CHECK_TRUE(scene.selection.add(layer_item(free_layer)));
    CHECK_TRUE(scene.selection.add(layer_item(locked)));
    CHECK_TRUE(scene.selection.contains(layer_item(locked)));

    // Two members, so the primary assertion says something: the locked one went
    // in LAST and is the active one. With one member it was the first and the
    // last member at once and every reading of primary() passed it.
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(free_layer), layer_item(locked)}));
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(locked)));

    // And the lock is still a lock. The filter is what excludes it, not add().
    CHECK_EQ(spell_all(scene.selection.filter(is_unlocked(scene.tree))),
             spell_all({layer_item(free_layer)}));
}

TEST(Selection, the_unlocked_filter_honours_a_lock_on_an_ancestor) {
    Scene scene;
    const LayerId group = scene.add_group("group");
    const LayerId inside = scene.add_shape("inside", group);
    const LayerId free_layer = scene.add_shape("free");

    CHECK_TRUE(scene.lock_layer(group));

    // The CHILD's own flag is still clear. An implementation reading own_locked
    // instead of effective_locked() passes every other assertion in this file
    // and fails here.
    CHECK_FALSE(scene.tree.find(inside)->own_locked);
    CHECK_TRUE(scene.tree.effective_locked(inside));

    scene.selection.replace({layer_item(inside), layer_item(free_layer)});

    const std::vector<SelectionItem> editable = scene.selection.filter(is_unlocked(scene.tree));
    CHECK_EQ(spell_all(editable), spell_all({layer_item(free_layer)}));

    // filter() reports; it does not narrow.
    CHECK_EQ(scene.selection.size(), size_t{2});

    CHECK_EQ(scene.selection.retain(is_unlocked(scene.tree)), size_t{1});
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(free_layer)}));
}

TEST(Selection, the_unlocked_filter_judges_an_object_by_the_layer_it_is_in) {
    Scene scene;
    const LayerId locked = scene.add_shape("locked");
    const AttributeObject object = scene.add_object();
    CHECK_TRUE(scene.lock_layer(locked));

    scene.selection.add(object_item(object));

    const LayerRef owner = layer_ref(locked);
    const auto in_locked_layer = [owner](AttributeObject) { return owner; };
    CHECK_EQ(scene.selection.filter(is_unlocked(scene.tree, in_locked_layer)).size(), size_t{0});

    // With no membership function the object is in no layer, and nothing above
    // it can be locked. effective_locked() answers TRUE for an id it does not
    // know, so an implementation that forwarded kInvalidLayer straight through
    // would call this object locked.
    CHECK_EQ(scene.selection.filter(is_unlocked(scene.tree)).size(), size_t{1});

    // A function that names a layer this build cannot resolve -- a ref wider
    // than a LayerId, which is what LayerRef is 64 bits for -- is an UNKNOWN
    // layer, not "no layer". It has to lock: the two zeros used to be folded
    // together, so this object came back editable and a move tool would have
    // written into something it could not identify. A1 answers the same way for
    // an id it does not know.
    const auto in_a_layer_that_does_not_fit = [](AttributeObject) {
        return LayerRef{0x100000001ull};
    };
    CHECK_EQ(scene.selection.filter(is_unlocked(scene.tree, in_a_layer_that_does_not_fit)).size(),
             size_t{0});

    // And kNoLayer through the same function is still "in no layer": unlocked.
    const auto in_no_layer = [](AttributeObject) { return kNoLayer; };
    CHECK_EQ(scene.selection.filter(is_unlocked(scene.tree, in_no_layer)).size(), size_t{1});
}

// ============================================================================
// Filtering by type
// ============================================================================

TEST(Selection, layers_and_objects_come_back_separately_and_in_order) {
    Scene scene;
    const LayerId first = scene.add_shape("first");
    const LayerId second = scene.add_shape("second");
    const AttributeObject one = scene.add_object();
    const AttributeObject two = scene.add_object();

    scene.selection.replace(
        {object_item(one), layer_item(second), object_item(two), layer_item(first)});

    CHECK_EQ(scene.selection.size(), size_t{4});
    CHECK_EQ(scene.selection.count_of(SelectionKind::Layer), size_t{2});
    CHECK_EQ(scene.selection.count_of(SelectionKind::Object), size_t{2});

    const std::vector<LayerId> layers = scene.selection.layers();
    CHECK_EQ(layers.size(), size_t{2});
    if (layers.size() == 2) {
        // Selection order, not id order: `second` was added first.
        CHECK_EQ(layers[0], second);
        CHECK_EQ(layers[1], first);
    }

    const std::vector<AttributeObject> objects = scene.selection.objects();
    CHECK_EQ(objects.size(), size_t{2});
    if (objects.size() == 2) {
        CHECK_TRUE(objects[0] == one);
        CHECK_TRUE(objects[1] == two);
    }

    CHECK_EQ(spell_all(scene.selection.filter(is_kind(SelectionKind::Object))),
             spell_all({object_item(one), object_item(two)}));
}

TEST(Selection, the_kind_counts_follow_removals) {
    Scene scene;
    const LayerId layer = scene.add_shape("layer");
    const AttributeObject one = scene.add_object();
    const AttributeObject two = scene.add_object();

    scene.selection.replace({layer_item(layer), object_item(one), object_item(two)});
    CHECK_EQ(scene.selection.count_of(SelectionKind::Layer), size_t{1});
    CHECK_EQ(scene.selection.count_of(SelectionKind::Object), size_t{2});

    CHECK_TRUE(scene.selection.remove(object_item(one)));
    CHECK_EQ(scene.selection.count_of(SelectionKind::Layer), size_t{1});
    CHECK_EQ(scene.selection.count_of(SelectionKind::Object), size_t{1});

    CHECK_TRUE(scene.selection.remove(layer_item(layer)));
    CHECK_EQ(scene.selection.count_of(SelectionKind::Layer), size_t{0});
    CHECK_EQ(scene.selection.count_of(SelectionKind::Object), size_t{1});
}

TEST(Selection, filtering_by_layer_kind_keeps_only_that_kind) {
    Scene scene;
    const LayerId shape = scene.add_shape("shape");
    const LayerId graph = scene.add_layer(LayerKind::Graph, "graph");
    const LayerId group = scene.add_group("group");
    const AttributeObject object = scene.add_object();

    scene.selection.replace(
        {layer_item(shape), layer_item(graph), layer_item(group), object_item(object)});

    CHECK_EQ(spell_all(scene.selection.filter(is_layer_kind(scene.tree, LayerKind::Shape))),
             spell_all({layer_item(shape)}));
    CHECK_EQ(spell_all(scene.selection.filter(is_layer_kind(scene.tree, LayerKind::Group))),
             spell_all({layer_item(group)}));

    // An object has no LayerKind of its own, so it matches no kind at all --
    // filtering objects by type is filtering them by an attribute.
    CHECK_EQ(scene.selection.filter(is_layer_kind(scene.tree, LayerKind::Model)).size(),
             size_t{0});
}

// ============================================================================
// Filtering by attribute
// ============================================================================

TEST(Selection, an_attribute_filter_reads_the_status_and_the_type_not_just_the_number) {
    Scene scene;
    const AttributeKey height = scene.store.intern("height");

    const AttributeObject tall = scene.add_object();
    const AttributeObject missing = scene.add_object();
    const AttributeObject worded = scene.add_object();
    const AttributeObject low = scene.add_object();

    CHECK_TRUE(scene.set_double(tall, height, 24.0));
    CHECK_TRUE(scene.set_string(worded, height, "tall"));
    CHECK_TRUE(scene.set_double(low, height, 4.0));

    scene.selection.replace(
        {object_item(tall), object_item(missing), object_item(worded), object_item(low)});

    // The string-valued one is a MISS, not a zero. A filter that reached for
    // as_double() without checking would dereference a null pointer here.
    CHECK_EQ(spell_all(scene.selection.filter(
                 scene.selection.attribute_predicate(height, double_above(10.0)))),
             spell_all({object_item(tall)}));

    const auto is_missing = [](const AttributeQuery& query) {
        return query.status == AttributeStatus::Missing;
    };
    CHECK_EQ(spell_all(
                 scene.selection.filter(scene.selection.attribute_predicate(height, is_missing))),
             spell_all({object_item(missing)}));
}

TEST(Selection, an_attribute_filter_inherits_from_the_layer_the_function_names) {
    Scene scene;
    const AttributeKey height = scene.store.intern("height");
    const LayerId block = scene.add_shape("block");
    const AttributeObject object = scene.add_object();

    // The value lives ONLY on the layer. Nothing is on the object.
    CHECK_TRUE(scene.set_layer_double(block, height, 30.0));
    CHECK_FALSE(scene.store.resolve(object, height, kNoLayer).ok());

    scene.selection.add(object_item(object));

    const LayerRef owner = layer_ref(block);
    const auto membership = [owner](AttributeObject) { return owner; };

    const auto from_the_layer = [](const AttributeQuery& query) {
        return query.ok() && query.source == AttributeSource::Layer;
    };

    CHECK_EQ(scene.selection
                 .filter(scene.selection.attribute_predicate(height, from_the_layer, membership))
                 .size(),
             size_t{1});

    // Without the membership function the object is in no layer, so the value
    // is not reachable. An implementation that ignored the function and always
    // passed some layer would match both ways and this pair would not test it.
    CHECK_EQ(scene.selection
                 .filter(scene.selection.attribute_predicate(height, double_above(10.0)))
                 .size(),
             size_t{0});
}

TEST(Selection, an_attribute_filter_on_a_layer_member_reads_that_layer) {
    Scene scene;
    const AttributeKey height = scene.store.intern("height");
    const LayerId tall = scene.add_shape("tall");
    const LayerId plain = scene.add_shape("plain");

    CHECK_TRUE(scene.set_layer_double(tall, height, 30.0));

    scene.selection.replace({layer_item(tall), layer_item(plain)});

    CHECK_EQ(spell_all(scene.selection.filter(
                 scene.selection.attribute_predicate(height, double_above(10.0)))),
             spell_all({layer_item(tall)}));

    const auto from_the_layer = [](const AttributeQuery& query) {
        return query.ok() && query.source == AttributeSource::Layer;
    };
    CHECK_EQ(spell_all(scene.selection.filter(
                 scene.selection.attribute_predicate(height, from_the_layer))),
             spell_all({layer_item(tall)}));
}

TEST(Selection, an_unknown_attribute_key_resolves_to_missing_for_every_member) {
    Scene scene;
    const AttributeKey never_interned{};  // default-constructed: invalid
    const LayerId layer = scene.add_shape("layer");
    const AttributeObject object = scene.add_object();

    scene.selection.replace({layer_item(layer), object_item(object)});

    const auto is_missing = [](const AttributeQuery& query) {
        return query.status == AttributeStatus::Missing;
    };
    CHECK_EQ(scene.selection.filter(scene.selection.attribute_predicate(never_interned, is_missing))
                 .size(),
             size_t{2});
    CHECK_EQ(scene.selection
                 .filter(scene.selection.attribute_predicate(never_interned, double_above(0.0)))
                 .size(),
             size_t{0});
}

TEST(Selection, retain_narrows_the_selection_and_keeps_the_survivors_in_order) {
    Scene scene;
    const AttributeKey height = scene.store.intern("height");

    const AttributeObject first = scene.add_object();
    const AttributeObject second = scene.add_object();
    const AttributeObject third = scene.add_object();
    const AttributeObject fourth = scene.add_object();

    CHECK_TRUE(scene.set_double(first, height, 40.0));
    CHECK_TRUE(scene.set_double(second, height, 2.0));
    CHECK_TRUE(scene.set_double(third, height, 50.0));
    CHECK_TRUE(scene.set_double(fourth, height, 1.0));

    scene.selection.replace(
        {object_item(first), object_item(second), object_item(third), object_item(fourth)});

    CHECK_EQ(scene.selection.retain(scene.selection.attribute_predicate(height, double_above(10.0))),
             size_t{2});

    CHECK_EQ(spell_order(scene.selection), spell_all({object_item(first), object_item(third)}));
    CHECK_EQ(scene.selection.size(), size_t{2});
    CHECK_EQ(spell(scene.selection.primary()), spell(object_item(third)));
}

TEST(Selection, retaining_nothing_empties_the_selection_and_leaves_it_reusable) {
    Scene scene;
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    const LayerId c = scene.add_shape("c");
    scene.selection.replace({layer_item(a), layer_item(b), layer_item(c)});

    const auto never = [](const SelectionItem&) { return false; };
    CHECK_EQ(scene.selection.retain(never), size_t{3});

    CHECK_TRUE(scene.selection.empty());
    CHECK_EQ(scene.selection.size(), size_t{0});
    CHECK_EQ(scene.selection.count_of(SelectionKind::Layer), size_t{0});
    CHECK_EQ(spell_order(scene.selection), std::string{});
    CHECK_FALSE(scene.selection.primary().valid());

    // "primary() is invalid" was the whole of the old second half, and EVERY
    // reading of primary() answers that on an empty selection, so it could not
    // fail. Selecting again is what proves the retain emptied the index and the
    // counts rather than only the sequence: a stale index entry refuses this
    // add, and a count left high fails the count.
    CHECK_TRUE(scene.selection.add(layer_item(b)));
    CHECK_EQ(spell_order(scene.selection), spell_all({layer_item(b)}));
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(b)));
    CHECK_EQ(scene.selection.size(), size_t{1});
    CHECK_EQ(scene.selection.count_of(SelectionKind::Layer), size_t{1});
}

TEST(Selection, a_null_predicate_matches_nothing_in_both_filter_and_retain) {
    Scene scene;
    const AttributeKey height = scene.store.intern("height");
    const LayerId a = scene.add_shape("a");
    const LayerId b = scene.add_shape("b");
    scene.selection.replace({layer_item(a), layer_item(b)});

    // The two used to disagree about the same argument: filter() answered "an
    // empty list" and retain() answered 0 and kept both members. Reachable
    // without writing SelectionPredicate{} by hand, which is why it matters:
    // attribute_predicate() with no value test answers false for every member.
    CHECK_EQ(scene.selection.filter(stratum::scene::SelectionPredicate{}).size(), size_t{0});
    CHECK_EQ(scene.selection
                 .filter(scene.selection.attribute_predicate(height,
                                                             stratum::scene::AttributePredicate{}))
                 .size(),
             size_t{0});

    CHECK_EQ(scene.selection.retain(stratum::scene::SelectionPredicate{}), size_t{2});
    CHECK_TRUE(scene.selection.empty());
    CHECK_EQ(scene.selection.size(), size_t{0});
}

// ============================================================================
// Holes and compaction
// ============================================================================

TEST(Selection, many_removals_keep_the_surviving_order_exactly) {
    Scene scene;

    // Large enough that the holes outnumber the survivors and the storage is
    // squeezed, which is the path a short selection never reaches.
    constexpr size_t kCount = 40;
    std::vector<LayerId> created;
    created.reserve(kCount);
    for (size_t i = 0; i < kCount; ++i) {
        created.push_back(scene.add_shape("layer" + std::to_string(i)));
    }

    std::vector<SelectionItem> everything;
    everything.reserve(kCount);
    for (LayerId id : created) everything.push_back(layer_item(id));
    CHECK_EQ(scene.selection.replace(everything), kCount);

    std::vector<SelectionItem> expected;
    for (size_t i = 0; i < kCount; ++i) {
        if (i % 4 == 0) {
            expected.push_back(layer_item(created[i]));
            continue;
        }
        CHECK_TRUE(scene.selection.remove(layer_item(created[i])));
    }

    CHECK_EQ(scene.selection.size(), expected.size());
    CHECK_EQ(spell_order(scene.selection), spell_all(expected));
    CHECK_EQ(spell(scene.selection.primary()), spell(expected.back()));

    for (size_t i = 0; i < kCount; ++i) {
        CHECK_EQ(scene.selection.contains(layer_item(created[i])), i % 4 == 0);
    }
}

TEST(Selection, adding_after_a_compaction_still_lands_at_the_end) {
    Scene scene;

    constexpr size_t kCount = 24;
    std::vector<LayerId> created;
    created.reserve(kCount);
    for (size_t i = 0; i < kCount; ++i) {
        created.push_back(scene.add_shape("layer" + std::to_string(i)));
    }

    std::vector<SelectionItem> everything;
    everything.reserve(kCount);
    for (LayerId id : created) everything.push_back(layer_item(id));
    scene.selection.replace(everything);

    // Drop all but the first two, forcing a rebuild, then re-add one of the
    // dropped members: a rebuild that left the index pointing at old positions
    // would put it back in the wrong place, or lose it.
    for (size_t i = 2; i < kCount; ++i) {
        scene.selection.remove(layer_item(created[i]));
    }
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(created[0]), layer_item(created[1])}));

    CHECK_TRUE(scene.selection.add(layer_item(created[7])));
    CHECK_EQ(spell_order(scene.selection),
             spell_all({layer_item(created[0]), layer_item(created[1]),
                        layer_item(created[7])}));
    CHECK_EQ(spell(scene.selection.primary()), spell(layer_item(created[7])));
    CHECK_EQ(scene.selection.size(), size_t{3});

    // And the survivors are still individually findable after the storage moved.
    CHECK_TRUE(scene.selection.contains(layer_item(created[0])));
    CHECK_TRUE(scene.selection.contains(layer_item(created[1])));
    CHECK_FALSE(scene.selection.contains(layer_item(created[8])));
}

TEST(Selection, footprint_grows_with_the_selection_and_falls_back_on_clear) {
    Scene scene;
    const size_t empty_cost = scene.selection.footprint();

    std::vector<SelectionItem> everything;
    for (size_t i = 0; i < 32; ++i) {
        everything.push_back(layer_item(scene.add_shape("layer" + std::to_string(i))));
    }
    scene.selection.replace(everything);

    // Not merely "bigger". Bigger by at least what 32 members cost in the
    // vector, so a footprint() of sizeof(Selection) alone fails here.
    CHECK_TRUE(scene.selection.footprint() >= empty_cost + 32 * sizeof(SelectionItem));

    // And bigger again after sixteen PROMOTIONS, which select nothing and
    // deselect nothing: the index still holds the same 32 members, while
    // make_primary() leaves a hole and re-appends, so only m_order grew. A
    // footprint() that counted the index and forgot the storage does not move
    // here, and "it got bigger when I selected things" never catches that
    // because both grow together.
    const size_t before_promotions = scene.selection.footprint();
    for (size_t i = 0; i < 16; ++i) {
        CHECK_TRUE(scene.selection.make_primary(everything[i]));
    }
    CHECK_EQ(scene.selection.size(), size_t{32});
    CHECK_TRUE(scene.selection.footprint() > before_promotions);

    scene.selection.clear();
    CHECK_EQ(scene.selection.footprint(), empty_cost);
}
