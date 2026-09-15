// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_attributes.cpp
 * @brief Attributes: types, interning, inheritance, sources and undo
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The property under test almost everywhere below is the SOURCE, not the value.
 * A store that returns the right number from the wrong rung looks correct in a
 * value-only test and is wrong in exactly the way a user notices: their edit
 * reads back fine and then vanishes when a rule runs or a layer changes.
 *
 * So most tests arrange the same value on two rungs at once and assert which one
 * answered. Where a test does check a value, the values are chosen to differ per
 * rung, so a resolution that reaches the wrong rung shows up as a wrong number
 * rather than as a pass.
 *
 * Written against the header's guarantees, not the implementation: the intern
 * table, the slot maps and the generation scheme are all free to change.
 */

#include "framework.hpp"

#include "scene/attributes.hpp"
#include "scene/command.hpp"

#include <cstddef>
#include <string>
#include <string_view>
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
using stratum::scene::attribute_source_name;
using stratum::scene::attribute_status_name;
using stratum::scene::attribute_type_name;
using stratum::scene::clear_attribute;
using stratum::scene::CommandStack;
using stratum::scene::kNoLayer;
using stratum::scene::LayerRef;
using stratum::scene::ObjectSnapshot;
using stratum::scene::set_attribute;

namespace {

/// A layer handle. Opaque to the store, so any number does.
constexpr LayerRef kLayerA = 7u;
constexpr LayerRef kLayerB = 8u;

/// Names rather than enumerators in the checks: a failure then says
/// "actual: Layer  expected: Object" instead of "<unprintable>".
std::string_view source_of(const AttributeQuery& query) {
    return attribute_source_name(query.source);
}

std::string_view status_of(const AttributeQuery& query) {
    return attribute_status_name(query.status);
}

} // namespace

// ============================================================================
// Value types
// ============================================================================

TEST(Attributes, a_bool_round_trips_with_its_type) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("visible");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_bool(true)));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    if (!query.ok()) return;
    CHECK_EQ(attribute_type_name(query.value->type()), std::string_view{"bool"});
    CHECK_TRUE(query.value->as_bool() != nullptr);
    if (query.value->as_bool() == nullptr) return;
    CHECK_TRUE(*query.value->as_bool());
}

// false is the value most easily lost to a "missing means false" shortcut.
TEST(Attributes, a_false_bool_is_a_value_and_not_an_absence) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("visible");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_bool(false)));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    CHECK_EQ(source_of(query), std::string_view{"Object"});
    if (!query.ok() || query.value->as_bool() == nullptr) return;
    CHECK_FALSE(*query.value->as_bool());
}

TEST(Attributes, a_double_round_trips_with_its_type) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(12.5)));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_EQ(attribute_type_name(query.value->type()), std::string_view{"double"});
    CHECK_NEAR(*query.value->as_double(), 12.5, 1e-12);
}

TEST(Attributes, a_string_round_trips_with_its_type) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("name");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_string("Lucan Library")));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    if (!query.ok() || query.value->as_string() == nullptr) return;
    CHECK_EQ(attribute_type_name(query.value->type()), std::string_view{"string"});
    CHECK_EQ(*query.value->as_string(), std::string("Lucan Library"));
}

TEST(Attributes, a_bool_array_round_trips_in_order) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("floor_is_retail");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_bools(
                                 AttributeValue::BoolArray{true, false, false, true})));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    if (!query.ok()) return;
    CHECK_EQ(attribute_type_name(query.value->type()), std::string_view{"bool[]"});
    const AttributeValue::BoolArray* values = query.value->as_bool_array();
    CHECK_TRUE(values != nullptr);
    if (values == nullptr) return;
    CHECK_EQ(values->size(), size_t{4});
    CHECK_TRUE((*values)[0]);
    CHECK_FALSE((*values)[1]);
    CHECK_FALSE((*values)[2]);
    CHECK_TRUE((*values)[3]);
}

TEST(Attributes, a_double_array_round_trips_in_order) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("floor_heights");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_doubles(
                                 AttributeValue::DoubleArray{4.0, 3.0, 3.0})));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    if (!query.ok()) return;
    const AttributeValue::DoubleArray* values = query.value->as_double_array();
    CHECK_TRUE(values != nullptr);
    if (values == nullptr) return;
    CHECK_EQ(values->size(), size_t{3});
    CHECK_NEAR((*values)[0], 4.0, 1e-12);
    CHECK_NEAR((*values)[1], 3.0, 1e-12);
    CHECK_NEAR((*values)[2], 3.0, 1e-12);
    CHECK_EQ(query.value->size(), size_t{3});
}

TEST(Attributes, a_string_array_round_trips_in_order) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("materials");

    CHECK_TRUE(set_attribute(
        stack, store, AttributeTarget::object(obj, key),
        AttributeValue::from_strings(AttributeValue::StringArray{"brick", "glass"})));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    if (!query.ok()) return;
    const AttributeValue::StringArray* values = query.value->as_string_array();
    CHECK_TRUE(values != nullptr);
    if (values == nullptr) return;
    CHECK_EQ(values->size(), size_t{2});
    CHECK_EQ((*values)[0], std::string("brick"));
    CHECK_EQ((*values)[1], std::string("glass"));
}

// An empty array is a value. Collapsing it to "missing" would make a rule that
// emitted no floors indistinguishable from a rule that never ran.
TEST(Attributes, an_empty_array_is_a_value) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("floor_heights");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_doubles(AttributeValue::DoubleArray{})));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    CHECK_EQ(source_of(query), std::string_view{"Object"});
    if (!query.ok()) return;
    CHECK_TRUE(query.value->is_array());
    CHECK_EQ(query.value->size(), size_t{0});
}

TEST(Attributes, a_scalar_and_an_array_of_it_are_different_types) {
    const AttributeValue scalar = AttributeValue::from_double(3.0);
    const AttributeValue array =
        AttributeValue::from_doubles(AttributeValue::DoubleArray{3.0});

    CHECK_TRUE(scalar.is(AttributeType::Double));
    CHECK_TRUE(array.is(AttributeType::DoubleArray));
    CHECK_FALSE(array.is(AttributeType::Double));
    CHECK_TRUE(array.as_double() == nullptr);
    CHECK_TRUE(scalar.as_double_array() == nullptr);
    CHECK_TRUE(scalar != array);
}

// "9" out of an OSM tag is a string. It must not read as the number 9.
TEST(Attributes, a_string_that_looks_like_a_number_stays_a_string) {
    const AttributeValue text = AttributeValue::from_string("9");
    const AttributeValue number = AttributeValue::from_double(9.0);

    CHECK_TRUE(text.as_double() == nullptr);
    CHECK_TRUE(text != number);
    CHECK_EQ(text.to_display_string(), std::string("\"9\""));
}

// ============================================================================
// Type mismatch
// ============================================================================

TEST(Attributes, asking_for_the_wrong_type_reports_a_mismatch_and_not_a_default) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_string("9")));

    const AttributeQuery query = store.resolve_as(obj, key, kNoLayer, AttributeType::Double);
    CHECK_EQ(status_of(query), std::string_view{"TypeMismatch"});
    CHECK_FALSE(query.ok());

    // The value and the source survive the mismatch, so an inspector can say
    // what the attribute actually is and where it came from.
    CHECK_TRUE(query.value != nullptr);
    if (query.value == nullptr) return;
    CHECK_EQ(attribute_type_name(query.value->type()), std::string_view{"string"});
    CHECK_EQ(source_of(query), std::string_view{"Object"});
}

TEST(Attributes, a_mismatched_read_falls_back_rather_than_converting) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_string("9")));

    // 9.0 would mean the store parsed the string. -1.0 means it refused to.
    CHECK_NEAR(store.double_or(obj, key, kNoLayer, -1.0), -1.0, 1e-12);
}

TEST(Attributes, a_missing_attribute_is_missing_and_not_a_zero) {
    AttributeStore store;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_EQ(status_of(query), std::string_view{"Missing"});
    CHECK_EQ(source_of(query), std::string_view{"None"});
    CHECK_TRUE(query.value == nullptr);
    CHECK_NEAR(store.double_or(obj, key, kNoLayer, 42.0), 42.0, 1e-12);
}

// ============================================================================
// Inheritance and sources
// ============================================================================

TEST(Attributes, an_object_with_no_value_inherits_the_layer) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));

    const AttributeQuery query = store.resolve(obj, key, kLayerA);
    CHECK_TRUE(query.ok());
    CHECK_EQ(source_of(query), std::string_view{"Layer"});
    CHECK_EQ(query.layer, kLayerA);
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 9.0, 1e-12);
}

TEST(Attributes, an_object_value_beats_the_layer) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));

    const AttributeQuery query = store.resolve(obj, key, kLayerA);
    CHECK_EQ(source_of(query), std::string_view{"Object"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 21.0, 1e-12);
}

// The reason two object-side slots exist: a rule rewrites Object on every run,
// and a hand edit that lived there would be destroyed by the next one.
TEST(Attributes, a_user_value_beats_an_object_value) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::user(obj, key),
                             AttributeValue::from_double(30.0)));

    const AttributeQuery query = store.resolve(obj, key, kLayerA);
    CHECK_EQ(source_of(query), std::string_view{"User"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 30.0, 1e-12);

    // A later rule run overwrites the Object slot and the hand edit survives.
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(24.0)));
    const AttributeQuery after = store.resolve(obj, key, kLayerA);
    CHECK_EQ(source_of(after), std::string_view{"User"});
    if (!after.ok() || after.value->as_double() == nullptr) return;
    CHECK_NEAR(*after.value->as_double(), 30.0, 1e-12);
}

TEST(Attributes, a_default_answers_when_nothing_else_does) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::schema_default(key),
                             AttributeValue::from_double(9.0)));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_EQ(source_of(query), std::string_view{"Default"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 9.0, 1e-12);
}

TEST(Attributes, the_layer_beats_the_default) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::schema_default(key),
                             AttributeValue::from_double(9.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(15.0)));

    const AttributeQuery query = store.resolve(obj, key, kLayerA);
    CHECK_EQ(source_of(query), std::string_view{"Layer"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 15.0, 1e-12);
}

// Layer membership is the caller's, not the store's. Naming a different layer
// has to give that layer's value and nothing else.
TEST(Attributes, resolution_uses_the_layer_the_caller_named) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerB, key),
                             AttributeValue::from_double(30.0)));

    CHECK_NEAR(store.double_or(obj, key, kLayerA, -1.0), 9.0, 1e-12);
    CHECK_NEAR(store.double_or(obj, key, kLayerB, -1.0), 30.0, 1e-12);

    // kNoLayer skips the layer rung entirely rather than picking one.
    const AttributeQuery unlayered = store.resolve(obj, key, kNoLayer);
    CHECK_EQ(status_of(unlayered), std::string_view{"Missing"});
}

// ============================================================================
// Clear versus set-to-the-default-value
// ============================================================================

TEST(Attributes, setting_the_object_to_the_default_value_still_reports_object) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::schema_default(key),
                             AttributeValue::from_double(9.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(9.0)));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_EQ(source_of(query), std::string_view{"Object"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 9.0, 1e-12);
}

TEST(Attributes, clearing_the_object_value_falls_through_to_the_default) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::schema_default(key),
                             AttributeValue::from_double(9.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(9.0)));
    CHECK_TRUE(clear_attribute(stack, store, AttributeTarget::object(obj, key)));

    // Same number as the test above, different source. That difference is the
    // whole point: change the default now and this object follows, the other
    // does not.
    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_EQ(source_of(query), std::string_view{"Default"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 9.0, 1e-12);
}

TEST(Attributes, clearing_the_object_value_falls_through_to_the_layer) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(clear_attribute(stack, store, AttributeTarget::object(obj, key)));

    const AttributeQuery query = store.resolve(obj, key, kLayerA);
    CHECK_EQ(source_of(query), std::string_view{"Layer"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 9.0, 1e-12);
}

// Clearing a user override leaves the rule's value standing, which is the
// "revert to the rule" button in the inspector.
TEST(Attributes, clearing_the_user_value_leaves_the_object_value) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::user(obj, key),
                             AttributeValue::from_double(30.0)));
    CHECK_TRUE(clear_attribute(stack, store, AttributeTarget::user(obj, key)));

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_EQ(source_of(query), std::string_view{"Object"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 21.0, 1e-12);
}

TEST(Attributes, clearing_an_inherited_value_changes_nothing_and_records_nothing) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));
    stack.seal();
    const size_t depth = stack.undo_depth();

    // The object has nothing of its own to clear. Refusing keeps an entry that
    // does nothing out of the undo menu, and leaves the layer's value alone.
    CHECK_FALSE(clear_attribute(stack, store, AttributeTarget::object(obj, key)));

    CHECK_EQ(stack.undo_depth(), depth);
    CHECK_EQ(source_of(store.resolve(obj, key, kLayerA)), std::string_view{"Layer"});
}

// kNoLayer is a reserved handle, not a layer. Accepting a write to it would
// give every unparented object in the document one shared attribute bag.
TEST(Attributes, the_no_layer_handle_cannot_be_written_to) {
    AttributeStore store;
    CommandStack stack;
    const AttributeKey key = store.intern("height");

    CHECK_FALSE(set_attribute(stack, store, AttributeTarget::layer(kNoLayer, key),
                              AttributeValue::from_double(9.0)));
    CHECK_EQ(stack.undo_depth(), size_t{0});
    CHECK_FALSE(store.layer_value(kNoLayer, key).ok());
}

TEST(Attributes, peek_sees_one_rung_only) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));

    CHECK_FALSE(store.peek(AttributeTarget::object(obj, key)).ok());
    CHECK_TRUE(store.peek(AttributeTarget::layer(kLayerA, key)).ok());
    CHECK_TRUE(store.layer_value(kLayerA, key).ok());
    CHECK_FALSE(store.default_value(key).ok());
}

// ============================================================================
// Interning
// ============================================================================

TEST(Attributes, interning_the_same_name_twice_gives_the_same_key) {
    AttributeStore store;

    const AttributeKey first = store.intern("height");
    const AttributeKey second = store.intern("height");

    CHECK_TRUE(first.valid());
    CHECK_EQ(first, second);
    CHECK_EQ(store.key_count(), size_t{1});
    CHECK_EQ(store.key_name(first), std::string_view{"height"});
}

TEST(Attributes, different_names_get_different_keys) {
    AttributeStore store;

    const AttributeKey height = store.intern("height");
    const AttributeKey width = store.intern("width");

    CHECK_TRUE(height != width);
    CHECK_EQ(store.key_count(), size_t{2});
}

TEST(Attributes, a_name_that_was_never_interned_has_no_key) {
    AttributeStore store;
    (void)store.intern("height");

    CHECK_FALSE(store.find_key("width").valid());
    CHECK_EQ(store.find_key("height"), store.intern("height"));
    CHECK_EQ(store.key_count(), size_t{1});
}

// An OSM extract produces a few hundred distinct tag keys over millions of
// objects. This is the shape of that, scaled down: every name keeps its own key,
// every key keeps its own name, and re-interning finds the original.
TEST(Attributes, many_distinct_names_stay_distinct_and_keep_their_names) {
    AttributeStore store;
    constexpr int kNames = 2000;

    std::vector<AttributeKey> keys;
    keys.reserve(static_cast<size_t>(kNames));
    for (int i = 0; i < kNames; ++i) {
        keys.push_back(store.intern("attr_" + std::to_string(i)));
    }

    CHECK_EQ(store.key_count(), static_cast<size_t>(kNames));

    int wrong_name = 0;
    int wrong_reintern = 0;
    for (int i = 0; i < kNames; ++i) {
        const std::string expected = "attr_" + std::to_string(i);
        if (store.key_name(keys[static_cast<size_t>(i)]) != expected) ++wrong_name;
        if (store.intern(expected) != keys[static_cast<size_t>(i)]) ++wrong_reintern;
    }

    // Counted rather than checked in the loop: 2000 failing checks would bury
    // every other failure in the run.
    CHECK_EQ(wrong_name, 0);
    CHECK_EQ(wrong_reintern, 0);
    CHECK_EQ(store.key_count(), static_cast<size_t>(kNames));
}

// Interning must not have moved the earlier names out from under the views
// key_name() handed out.
TEST(Attributes, a_name_read_before_later_interning_is_still_readable_after) {
    AttributeStore store;
    const AttributeKey first = store.intern("height");
    const std::string_view name = store.key_name(first);

    for (int i = 0; i < 1000; ++i) {
        (void)store.intern("filler_" + std::to_string(i));
    }

    CHECK_EQ(name, std::string_view{"height"});
    CHECK_EQ(store.key_name(first), std::string_view{"height"});
}

TEST(Attributes, keys_do_not_read_each_others_values) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    constexpr int kNames = 200;

    std::vector<AttributeKey> keys;
    keys.reserve(static_cast<size_t>(kNames));
    for (int i = 0; i < kNames; ++i) {
        const AttributeKey key = store.intern("attr_" + std::to_string(i));
        keys.push_back(key);
        CHECK_TRUE(store.load_value(AttributeTarget::object(obj, key),
                                    AttributeValue::from_double(static_cast<double>(i))));
    }

    int wrong = 0;
    for (int i = 0; i < kNames; ++i) {
        const double value =
            store.double_or(obj, keys[static_cast<size_t>(i)], kNoLayer, -1.0);
        if (value != static_cast<double>(i)) ++wrong;
    }
    CHECK_EQ(wrong, 0);
    (void)stack;
}

TEST(Attributes, an_invalid_key_resolves_to_nothing_and_cannot_be_written) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey nothing;

    CHECK_FALSE(nothing.valid());
    CHECK_EQ(status_of(store.resolve(obj, nothing, kNoLayer)), std::string_view{"Missing"});
    CHECK_FALSE(set_attribute(stack, store, AttributeTarget::object(obj, nothing),
                              AttributeValue::from_double(1.0)));
    CHECK_EQ(stack.undo_depth(), size_t{0});
}

// ============================================================================
// Undo
// ============================================================================

TEST(Attributes, undo_of_the_first_set_restores_absence_and_not_a_zero) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(stack.undo());

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_EQ(status_of(query), std::string_view{"Missing"});
    CHECK_EQ(source_of(query), std::string_view{"None"});

    CHECK_TRUE(stack.redo());
    CHECK_NEAR(store.double_or(obj, key, kNoLayer, -1.0), 21.0, 1e-12);
}

TEST(Attributes, undo_of_a_second_set_restores_the_first_value) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    stack.seal();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(30.0)));
    stack.seal();

    CHECK_TRUE(stack.undo());
    CHECK_NEAR(store.double_or(obj, key, kNoLayer, -1.0), 21.0, 1e-12);

    CHECK_TRUE(stack.undo());
    CHECK_EQ(status_of(store.resolve(obj, key, kNoLayer)), std::string_view{"Missing"});
}

TEST(Attributes, undo_of_a_set_that_changed_the_type_restores_the_old_type) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_string("tall")));
    stack.seal();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    stack.seal();

    CHECK_TRUE(stack.undo());

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    if (!query.ok()) return;
    CHECK_EQ(attribute_type_name(query.value->type()), std::string_view{"string"});
    CHECK_EQ(*query.value->as_string(), std::string("tall"));
}

TEST(Attributes, undo_of_a_clear_restores_the_value_and_its_source) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));
    stack.seal();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    stack.seal();
    CHECK_TRUE(clear_attribute(stack, store, AttributeTarget::object(obj, key)));
    CHECK_EQ(source_of(store.resolve(obj, key, kLayerA)), std::string_view{"Layer"});

    CHECK_TRUE(stack.undo());

    const AttributeQuery query = store.resolve(obj, key, kLayerA);
    CHECK_EQ(source_of(query), std::string_view{"Object"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 21.0, 1e-12);

    CHECK_TRUE(stack.redo());
    CHECK_EQ(source_of(store.resolve(obj, key, kLayerA)), std::string_view{"Layer"});
}

TEST(Attributes, undo_of_a_layer_set_takes_it_back_from_every_inheriting_object) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject first = store.create_object();
    const AttributeObject second = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));
    CHECK_NEAR(store.double_or(first, key, kLayerA, -1.0), 9.0, 1e-12);
    CHECK_NEAR(store.double_or(second, key, kLayerA, -1.0), 9.0, 1e-12);

    CHECK_TRUE(stack.undo());

    CHECK_EQ(status_of(store.resolve(first, key, kLayerA)), std::string_view{"Missing"});
    CHECK_EQ(status_of(store.resolve(second, key, kLayerA)), std::string_view{"Missing"});
}

TEST(Attributes, undo_of_a_user_set_leaves_the_object_value_standing) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    stack.seal();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::user(obj, key),
                             AttributeValue::from_double(30.0)));
    stack.seal();

    CHECK_TRUE(stack.undo());

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_EQ(source_of(query), std::string_view{"Object"});
    if (!query.ok() || query.value->as_double() == nullptr) return;
    CHECK_NEAR(*query.value->as_double(), 21.0, 1e-12);
}

TEST(Attributes, undo_of_a_default_set_takes_the_fallback_away) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::schema_default(key),
                             AttributeValue::from_double(9.0)));
    CHECK_EQ(source_of(store.resolve(obj, key, kNoLayer)), std::string_view{"Default"});

    CHECK_TRUE(stack.undo());
    CHECK_EQ(status_of(store.resolve(obj, key, kNoLayer)), std::string_view{"Missing"});
}

TEST(Attributes, undo_of_an_array_set_restores_the_whole_array) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("floor_heights");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_doubles(
                                 AttributeValue::DoubleArray{4.0, 3.0, 3.0})));
    stack.seal();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_doubles(AttributeValue::DoubleArray{5.0})));
    stack.seal();

    CHECK_TRUE(stack.undo());

    const AttributeQuery query = store.resolve(obj, key, kNoLayer);
    CHECK_TRUE(query.ok());
    if (!query.ok()) return;
    const AttributeValue::DoubleArray* values = query.value->as_double_array();
    CHECK_TRUE(values != nullptr);
    if (values == nullptr) return;
    CHECK_EQ(values->size(), size_t{3});
    CHECK_NEAR((*values)[0], 4.0, 1e-12);
}

// A transaction is how one user action that touches several attributes becomes
// one undo step -- the shape the OSM import and the rule engine will both use.
TEST(Attributes, a_transaction_of_sets_undoes_as_one_step) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey height = store.intern("height");
    const AttributeKey zoning = store.intern("zoning");

    stack.begin_transaction("Apply zoning");
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, height),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, zoning),
                             AttributeValue::from_string("residential")));
    stack.commit_transaction();

    CHECK_EQ(stack.undo_depth(), size_t{1});
    CHECK_EQ(stack.undo_label(), std::string("Apply zoning"));

    CHECK_TRUE(stack.undo());
    CHECK_EQ(status_of(store.resolve(obj, height, kNoLayer)), std::string_view{"Missing"});
    CHECK_EQ(status_of(store.resolve(obj, zoning, kNoLayer)), std::string_view{"Missing"});
}

// ============================================================================
// Coalescing
// ============================================================================

// Typing 2, then 21 into an inspector field is one edit, not two.
TEST(Attributes, repeated_sets_on_one_attribute_collapse_into_one_undo_step) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(2.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.5)));

    CHECK_EQ(stack.undo_depth(), size_t{1});
    CHECK_NEAR(store.double_or(obj, key, kNoLayer, -1.0), 21.5, 1e-12);

    // Back to before the gesture, not to an intermediate keystroke.
    CHECK_TRUE(stack.undo());
    CHECK_EQ(status_of(store.resolve(obj, key, kNoLayer)), std::string_view{"Missing"});

    CHECK_TRUE(stack.redo());
    CHECK_NEAR(store.double_or(obj, key, kNoLayer, -1.0), 21.5, 1e-12);
}

TEST(Attributes, seal_ends_the_gesture) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(2.0)));
    stack.seal();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));

    CHECK_EQ(stack.undo_depth(), size_t{2});
}

TEST(Attributes, sets_on_different_targets_do_not_merge) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject first = store.create_object();
    const AttributeObject second = store.create_object();
    const AttributeKey height = store.intern("height");
    const AttributeKey width = store.intern("width");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(first, height),
                             AttributeValue::from_double(1.0)));
    // Different key.
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(first, width),
                             AttributeValue::from_double(2.0)));
    // Different object.
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(second, width),
                             AttributeValue::from_double(3.0)));
    // Different slot on the same object and key.
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::user(second, width),
                             AttributeValue::from_double(4.0)));

    CHECK_EQ(stack.undo_depth(), size_t{4});
}

TEST(Attributes, a_clear_does_not_merge_into_the_set_before_it) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(clear_attribute(stack, store, AttributeTarget::object(obj, key)));

    CHECK_EQ(stack.undo_depth(), size_t{2});

    CHECK_TRUE(stack.undo());
    CHECK_NEAR(store.double_or(obj, key, kNoLayer, -1.0), 21.0, 1e-12);
}

// ============================================================================
// Handles
// ============================================================================

TEST(Attributes, a_handle_survives_unrelated_creation_and_destruction) {
    AttributeStore store;
    CommandStack stack;
    const AttributeKey key = store.intern("height");

    const AttributeObject kept = store.create_object();
    const AttributeObject doomed = store.create_object();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(kept, key),
                             AttributeValue::from_double(21.0)));

    CHECK_TRUE(store.destroy_object(doomed));
    for (int i = 0; i < 16; ++i) {
        (void)store.create_object();
    }

    CHECK_TRUE(store.is_valid(kept));
    CHECK_NEAR(store.double_or(kept, key, kNoLayer, -1.0), 21.0, 1e-12);
}

// The aliasing this scheme exists to prevent: a recycled slot must not answer
// for the handle that used to name it.
TEST(Attributes, a_stale_handle_does_not_alias_the_object_that_replaced_it) {
    AttributeStore store;
    CommandStack stack;
    const AttributeKey key = store.intern("height");

    const AttributeObject first = store.create_object();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(first, key),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(store.destroy_object(first));

    const AttributeObject second = store.create_object();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(second, key),
                             AttributeValue::from_double(30.0)));

    CHECK_FALSE(store.is_valid(first));
    CHECK_TRUE(store.is_valid(second));
    CHECK_TRUE(first != second);

    // Reading through the stale handle must not hand back the new object's
    // value, and must not hand back the dead one's either.
    const AttributeQuery query = store.resolve(first, key, kNoLayer);
    CHECK_EQ(status_of(query), std::string_view{"Missing"});
    CHECK_NEAR(store.double_or(second, key, kNoLayer, -1.0), 30.0, 1e-12);
}

TEST(Attributes, a_stale_handle_cannot_be_written_through) {
    AttributeStore store;
    CommandStack stack;
    const AttributeKey key = store.intern("height");

    const AttributeObject obj = store.create_object();
    CHECK_TRUE(store.destroy_object(obj));
    CHECK_FALSE(store.destroy_object(obj));

    CHECK_FALSE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                              AttributeValue::from_double(21.0)));
    CHECK_EQ(stack.undo_depth(), size_t{0});
    CHECK_FALSE(clear_attribute(stack, store, AttributeTarget::object(obj, key)));
    CHECK_EQ(stack.undo_depth(), size_t{0});
}

// An invalid handle is not a stale one: it means "no object", and the layer and
// default rungs still answer. That is how the inspector shows a layer's values.
TEST(Attributes, an_invalid_handle_still_resolves_the_layer_and_the_default) {
    AttributeStore store;
    CommandStack stack;
    const AttributeKey key = store.intern("height");
    const AttributeObject none;

    CHECK_FALSE(none.valid());
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));

    const AttributeQuery query = store.resolve(none, key, kLayerA);
    CHECK_EQ(source_of(query), std::string_view{"Layer"});
}

TEST(Attributes, destroying_an_object_takes_its_attributes_with_it) {
    AttributeStore store;
    CommandStack stack;
    const AttributeKey key = store.intern("height");

    const AttributeObject obj = store.create_object();
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    CHECK_EQ(store.live_object_count(), size_t{1});

    CHECK_TRUE(store.destroy_object(obj));
    CHECK_EQ(store.live_object_count(), size_t{0});

    // The recycled slot starts empty rather than inheriting the dead object's
    // values.
    const AttributeObject reused = store.create_object();
    CHECK_EQ(status_of(store.resolve(reused, key, kNoLayer)), std::string_view{"Missing"});
}

// ============================================================================
// Snapshots, enumeration and the load path
// ============================================================================

TEST(Attributes, a_snapshot_restores_both_slots) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey height = store.intern("height");
    const AttributeKey name = store.intern("name");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, height),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::user(obj, name),
                             AttributeValue::from_string("Library")));

    const ObjectSnapshot snapshot = store.snapshot(obj);
    CHECK_EQ(snapshot.object_values.size(), size_t{1});
    CHECK_EQ(snapshot.user_values.size(), size_t{1});
    CHECK_FALSE(snapshot.empty());

    CHECK_TRUE(clear_attribute(stack, store, AttributeTarget::object(obj, height)));
    CHECK_TRUE(clear_attribute(stack, store, AttributeTarget::user(obj, name)));
    CHECK_EQ(status_of(store.resolve(obj, height, kNoLayer)), std::string_view{"Missing"});

    CHECK_TRUE(store.restore(obj, snapshot));

    CHECK_EQ(source_of(store.resolve(obj, height, kNoLayer)), std::string_view{"Object"});
    CHECK_EQ(source_of(store.resolve(obj, name, kNoLayer)), std::string_view{"User"});
    CHECK_NEAR(store.double_or(obj, height, kNoLayer, -1.0), 21.0, 1e-12);
}

TEST(Attributes, keys_on_lists_every_rung_once) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey height = store.intern("height");
    const AttributeKey zoning = store.intern("zoning");
    const AttributeKey name = store.intern("name");

    // height appears on three rungs at once and must still be listed once.
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::schema_default(height),
                             AttributeValue::from_double(9.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, height),
                             AttributeValue::from_double(12.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, height),
                             AttributeValue::from_double(21.0)));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, zoning),
                             AttributeValue::from_string("residential")));
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::user(obj, name),
                             AttributeValue::from_string("Library")));

    const std::vector<AttributeKey> keys = store.keys_on(obj, kLayerA);
    CHECK_EQ(keys.size(), size_t{3});

    int height_seen = 0;
    for (const AttributeKey key : keys) {
        if (key == height) ++height_seen;
    }
    CHECK_EQ(height_seen, 1);

    // Naming no layer drops the layer-only key.
    const std::vector<AttributeKey> unlayered = store.keys_on(obj, kNoLayer);
    CHECK_EQ(unlayered.size(), size_t{2});
}

TEST(Attributes, load_value_fills_the_store_without_touching_the_history) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(store.load_value(AttributeTarget::object(obj, key),
                                AttributeValue::from_double(21.0)));

    CHECK_NEAR(store.double_or(obj, key, kNoLayer, -1.0), 21.0, 1e-12);
    CHECK_EQ(stack.undo_depth(), size_t{0});
    CHECK_FALSE(stack.can_undo());

    // An edit made afterwards is still undoable, and undoing it lands on the
    // loaded value rather than on nothing.
    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(30.0)));
    CHECK_TRUE(stack.undo());
    CHECK_NEAR(store.double_or(obj, key, kNoLayer, -1.0), 21.0, 1e-12);
}

TEST(Attributes, zero_of_makes_a_typed_empty_value) {
    // Named locals rather than temporaries: CHECK_EQ binds a reference to its
    // operands, and a pointer into an AttributeValue that died at the end of the
    // previous statement would dangle before the comparison ran.
    const AttributeValue flag = AttributeValue::zero_of(AttributeType::Bool);
    const AttributeValue number = AttributeValue::zero_of(AttributeType::Double);
    const AttributeValue text = AttributeValue::zero_of(AttributeType::String);
    const AttributeValue numbers = AttributeValue::zero_of(AttributeType::DoubleArray);
    const AttributeValue texts = AttributeValue::zero_of(AttributeType::StringArray);

    CHECK_TRUE(flag.is(AttributeType::Bool));
    CHECK_FALSE(*flag.as_bool());
    CHECK_NEAR(*number.as_double(), 0.0, 1e-12);
    CHECK_EQ(*text.as_string(), std::string());
    CHECK_EQ(numbers.size(), size_t{0});
    CHECK_TRUE(texts.is_array());
}

TEST(Attributes, a_command_is_labelled_by_what_it_touched) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject obj = store.create_object();
    const AttributeKey key = store.intern("height");

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::object(obj, key),
                             AttributeValue::from_double(21.0)));
    CHECK_EQ(stack.undo_label(), std::string("Set height"));
    stack.seal();

    CHECK_TRUE(set_attribute(stack, store, AttributeTarget::layer(kLayerA, key),
                             AttributeValue::from_double(9.0)));
    CHECK_EQ(stack.undo_label(), std::string("Set layer height"));
    stack.seal();

    CHECK_TRUE(clear_attribute(stack, store, AttributeTarget::object(obj, key)));
    CHECK_EQ(stack.undo_label(), std::string("Clear height"));
}
