// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_document.cpp
 * @brief Save and load: does the scene that comes back equal the one that went in?
 *
 * The round-trip tests here compare EVERYTHING, on purpose. A round trip checked
 * with a handful of spot assertions passes against a loader that drops sibling
 * order, flattens inheritance or reissues a handle, and every one of those is a
 * bug whose first symptom is a scene that is subtly wrong a week later.
 *
 * So the comparisons walk the whole layer tree field by field, the whole slot
 * table, and every value on every rung -- and several tests assert the thing a
 * naive implementation gets WRONG rather than the thing it gets right:
 *
 *   - a stale object handle must still be stale after a load (a loader that
 *     restores objects without their generations makes it live again),
 *   - the next layer id must be above one the saved session already issued to a
 *     layer it then deleted (a loader that recomputes the counter reissues it),
 *   - an inherited value must still report AttributeSource::Layer (a writer that
 *     saves resolved values turns it into Object, and the number matches either
 *     way, so only the SOURCE can tell them apart).
 */

#include "framework.hpp"

#include "scene/attributes.hpp"
#include "scene/command.hpp"
#include "scene/document.hpp"
#include "scene/layer.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace stratum::scene;
using nlohmann::json;

namespace {

// ============================================================================
// Building a scene
// ============================================================================

bool run(Document& doc, CommandPtr command) {
    return doc.history().execute(std::move(command));
}

/// Create a layer through the stack and hand back the id it reserved.
LayerId make_layer(Document& doc, LayerKind kind, std::string name,
                   LayerId parent = kInvalidLayer, size_t index = kAppend) {
    auto command = std::make_unique<CreateLayerCommand>(doc.layers(), kind, std::move(name),
                                                        parent, index);
    const LayerId id = command->layer();
    if (!doc.history().execute(std::move(command))) return kInvalidLayer;
    return id;
}

/// The handles a test needs to interrogate the scene build_scene() made.
struct Scene {
    LayerId world = kInvalidLayer;      ///< Root group, coloured
    LayerId roads = kInvalidLayer;      ///< Graph, locked, its own colour, holds `lanes`
    LayerId blocks = kInvalidLayer;     ///< Group, hidden, translated, holds `height`
    LayerId buildings = kInvalidLayer;  ///< Shape under blocks, no colour of its own
    LayerId water = kInvalidLayer;      ///< Shape under blocks, its own colour
    LayerId terrain = kInvalidLayer;    ///< Map, reordered to the front of world
    LayerId imported = kInvalidLayer;   ///< Second root
    LayerId models = kInvalidLayer;     ///< Model under imported
    LayerId doomed = kInvalidLayer;     ///< Created, then deleted. Its id must never return.

    AttributeObject road_object{};    ///< In roads. Object AND user values, arrays, empties.
    AttributeObject block_object{};   ///< In blocks. Inherits `height` from the layer.
    AttributeObject cleared_object{}; ///< In blocks. Had `height`, then cleared it.
    AttributeObject orphan_object{};  ///< In `doomed`, which no longer exists.
    AttributeObject reborn_object{};  ///< Took a recycled slot: generation 1.
    AttributeObject stale_object{};   ///< The handle reborn_object displaced. Must stay dead.
    AttributeObject dead_slot{};      ///< Created, destroyed, never reused. Its slot stays dead.
};

/**
 * @brief A scene with something of every kind in it
 *
 * Deliberately awkward: nested groups, a sibling order that is not creation
 * order, a layer that was deleted, an object whose layer no longer exists, a
 * recycled object slot, values on both object rungs, values on layers, declared
 * defaults, every value type, and an empty array of each array type.
 */
Scene build_scene(Document& doc) {
    Scene scene;
    AttributeStore& store = doc.attributes();
    CommandStack& stack = doc.history();

    scene.world = make_layer(doc, LayerKind::Group, "World");
    scene.roads = make_layer(doc, LayerKind::Graph, "Roads", scene.world);
    scene.blocks = make_layer(doc, LayerKind::Group, "Blocks", scene.world);
    scene.buildings = make_layer(doc, LayerKind::Shape, "Buildings", scene.blocks);
    scene.water = make_layer(doc, LayerKind::Shape, "Water", scene.blocks);
    scene.terrain = make_layer(doc, LayerKind::Map, "Terrain", scene.world);
    scene.imported = make_layer(doc, LayerKind::Group, "Imported");
    scene.models = make_layer(doc, LayerKind::Model, "Models", scene.imported);

    // Sibling order that creation order does not produce: Terrain was made last
    // and sits first. A loader that rebuilt the tree by id order would pass every
    // other check here and fail this one.
    run(doc, std::make_unique<ReorderLayerCommand>(doc.layers(), scene.terrain, 0));

    run(doc, std::make_unique<SetLayerColourCommand>(doc.layers(), scene.world,
                                                     glm::vec3(0.9f, 0.1f, 0.25f)));
    run(doc, std::make_unique<SetLayerColourCommand>(doc.layers(), scene.roads,
                                                     glm::vec3(0.2f, 0.4f, 0.8f)));
    run(doc, std::make_unique<SetLayerColourCommand>(doc.layers(), scene.water,
                                                     glm::vec3(0.0f, 0.3f, 0.6f)));
    // buildings deliberately keeps no colour of its own: it inherits World's.

    run(doc, std::make_unique<SetLayerLockedCommand>(doc.layers(), scene.roads, true));
    run(doc, std::make_unique<SetLayerVisibleCommand>(doc.layers(), scene.blocks, false));

    LayerTransform moved;
    moved.translation = glm::dvec3(10.5, -3.25, 1e-7);
    moved.rotation = glm::dvec3(0.0, 1.5707963267948966, 0.0);
    moved.scale = glm::dvec3(2.0, 1.0, 0.5);
    run(doc, std::make_unique<SetLayerTransformCommand>(doc.layers(), scene.blocks, moved));

    // Created and deleted, so the tree's id counter ends up ABOVE every id that
    // survives. Restoring the counter from the survivors would reissue this id.
    scene.doomed = make_layer(doc, LayerKind::Shape, "Doomed", scene.imported);
    const AttributeObject orphan = doc.create_object(scene.doomed);
    scene.orphan_object = orphan;
    run(doc, std::make_unique<DeleteLayerCommand>(doc.layers(), scene.doomed));

    // ── Attributes ──────────────────────────────────────────────────────────

    const AttributeKey height = store.intern("height");
    const AttributeKey name = store.intern("name");
    const AttributeKey lanes = store.intern("lanes");
    const AttributeKey tags = store.intern("tags");
    const AttributeKey flags = store.intern("flags");
    const AttributeKey levels = store.intern("levels");
    const AttributeKey no_doubles = store.intern("no_doubles");
    const AttributeKey no_strings = store.intern("no_strings");
    const AttributeKey no_flags = store.intern("no_flags");
    const AttributeKey sealed = store.intern("sealed");

    set_attribute(stack, store, AttributeTarget::schema_default(height),
                  AttributeValue::from_double(9.0));
    set_attribute(stack, store, AttributeTarget::schema_default(name),
                  AttributeValue::from_string("unnamed"));
    set_attribute(stack, store, AttributeTarget::schema_default(sealed),
                  AttributeValue::from_bool(false));

    set_attribute(stack, store, AttributeTarget::layer(static_cast<LayerRef>(scene.roads), lanes),
                  AttributeValue::from_double(2.0));
    set_attribute(stack, store, AttributeTarget::layer(static_cast<LayerRef>(scene.blocks), height),
                  AttributeValue::from_double(12.0));
    set_attribute(stack, store,
                  AttributeTarget::layer(static_cast<LayerRef>(scene.blocks), no_strings),
                  AttributeValue::from_strings({}));

    scene.road_object = doc.create_object(scene.roads);
    set_attribute(stack, store, AttributeTarget::object(scene.road_object, height),
                  AttributeValue::from_double(15.0));
    // Same key, the other rung, a different number: User outranks Object, and the
    // two must not be folded together by a save.
    set_attribute(stack, store, AttributeTarget::user(scene.road_object, height),
                  AttributeValue::from_double(20.0));
    set_attribute(stack, store, AttributeTarget::object(scene.road_object, tags),
                  AttributeValue::from_strings({"primary", "oneway", ""}));
    set_attribute(stack, store, AttributeTarget::object(scene.road_object, flags),
                  AttributeValue::from_bools({true, false, true, true}));
    set_attribute(stack, store, AttributeTarget::object(scene.road_object, levels),
                  AttributeValue::from_doubles({1.0, -2.5, 0.1, 1e-300, 3.141592653589793}));
    set_attribute(stack, store, AttributeTarget::object(scene.road_object, no_doubles),
                  AttributeValue::from_doubles({}));
    set_attribute(stack, store, AttributeTarget::object(scene.road_object, no_flags),
                  AttributeValue::from_bools({}));
    set_attribute(stack, store, AttributeTarget::user(scene.road_object, name),
                  AttributeValue::from_string("Main Street \"quoted\"\n"));
    set_attribute(stack, store, AttributeTarget::object(scene.road_object, sealed),
                  AttributeValue::from_bool(true));

    // Owns nothing: every read of `height` on it must come from the layer.
    scene.block_object = doc.create_object(scene.blocks);

    // Owned `height`, then gave it up. Clearing is not setting the layer's value:
    // the object holds nothing afterwards and resolution falls through again.
    scene.cleared_object = doc.create_object(scene.blocks);
    set_attribute(stack, store, AttributeTarget::object(scene.cleared_object, height),
                  AttributeValue::from_double(33.0));
    clear_attribute(stack, store, AttributeTarget::object(scene.cleared_object, height));
    set_attribute(stack, store, AttributeTarget::user(scene.cleared_object, name),
                  AttributeValue::from_string("Kept"));

    // A slot recycled once, so the file has to carry a generation that is not 0.
    const AttributeObject doomed_object = doc.create_object(scene.world);
    doc.destroy_object(doomed_object);
    scene.stale_object = doomed_object;
    scene.reborn_object = doc.create_object(scene.world);
    set_attribute(stack, store, AttributeTarget::object(scene.reborn_object, name),
                  AttributeValue::from_string("Reborn"));

    // And one slot left dead, so the file's object table is longer than the list
    // of live objects. Without this the slot table and the live objects would be
    // the same thing, and every check on the table would be checking half of it.
    scene.dead_slot = doc.create_object(scene.world);
    doc.destroy_object(scene.dead_slot);

    return scene;
}

// ============================================================================
// Exhaustive comparison
// ============================================================================

std::string kind_of(const Layer& layer) { return layer_kind_name(layer.kind); }

void compare_layer(const LayerTree& a, const LayerTree& b, LayerId id, size_t depth) {
    if (depth > 64) return;  // The scenes here are shallow; this only stops a cycle.

    const Layer* left = a.find(id);
    const Layer* right = b.find(id);
    CHECK_TRUE(left != nullptr);
    CHECK_TRUE(right != nullptr);
    if (left == nullptr || right == nullptr) return;

    CHECK_EQ(left->id, right->id);
    CHECK_EQ(kind_of(*left), kind_of(*right));
    CHECK_EQ(left->name, right->name);
    CHECK_EQ(left->parent, right->parent);
    CHECK_EQ(left->own_visible, right->own_visible);
    CHECK_EQ(left->own_locked, right->own_locked);

    // has_value() first and separately: "no colour, so inherit" and "an explicit
    // colour" are different states, and comparing only the value when both are
    // present would let one become the other.
    CHECK_EQ(left->own_colour.has_value(), right->own_colour.has_value());
    if (left->own_colour.has_value() && right->own_colour.has_value()) {
        CHECK_TRUE(*left->own_colour == *right->own_colour);
    }

    // Exact componentwise equality, which is what LayerTransform::operator== is
    // for. A tolerance here would hide the precision loss a JSON round trip is
    // most likely to introduce.
    CHECK_TRUE(left->own_transform == right->own_transform);

    CHECK_EQ(a.effective_visible(id), b.effective_visible(id));
    CHECK_EQ(a.effective_locked(id), b.effective_locked(id));
    CHECK_TRUE(a.effective_colour(id) == b.effective_colour(id));
    CHECK_TRUE(a.effective_transform(id) == b.effective_transform(id));
    CHECK_EQ(a.depth(id), b.depth(id));
    CHECK_EQ(a.index_of(id), b.index_of(id));

    const std::vector<LayerId>& left_children = a.children(id);
    const std::vector<LayerId>& right_children = b.children(id);
    CHECK_EQ(left_children.size(), right_children.size());
    if (left_children.size() != right_children.size()) return;

    for (size_t i = 0; i < left_children.size(); ++i) {
        // Element by element, not as a set: this is where sibling ORDER is
        // checked, and it is the field a loader is most likely to lose.
        CHECK_EQ(left_children[i], right_children[i]);
        compare_layer(a, b, left_children[i], depth + 1);
    }
}

void compare_trees(const LayerTree& a, const LayerTree& b) {
    CHECK_EQ(a.size(), b.size());
    CHECK_EQ(a.roots().size(), b.roots().size());
    if (a.roots().size() != b.roots().size()) return;
    for (size_t i = 0; i < a.roots().size(); ++i) {
        CHECK_EQ(a.roots()[i], b.roots()[i]);
        compare_layer(a, b, a.roots()[i], 0);
    }
}

using NamedValues = std::vector<std::pair<std::string, AttributeValue>>;

NamedValues named(const AttributeStore& store,
                  const std::vector<std::pair<AttributeKey, AttributeValue>>& pairs) {
    NamedValues out;
    out.reserve(pairs.size());
    for (const auto& pair : pairs) {
        out.emplace_back(std::string(store.key_name(pair.first)), pair.second);
    }
    // By NAME, because key indices are re-interned on load and are not a property
    // the file promises to preserve; the names and the values are.
    std::sort(out.begin(), out.end(),
              [](const auto& x, const auto& y) { return x.first < y.first; });
    return out;
}

void compare_values(const NamedValues& a, const NamedValues& b) {
    CHECK_EQ(a.size(), b.size());
    if (a.size() != b.size()) return;
    for (size_t i = 0; i < a.size(); ++i) {
        CHECK_EQ(a[i].first, b[i].first);
        // The display string gives a readable failure; operator== is the actual
        // assertion, because to_display_string() rounds doubles and would pass a
        // value that lost digits on the way through JSON.
        CHECK_EQ(a[i].second.to_display_string(), b[i].second.to_display_string());
        CHECK_TRUE(a[i].second == b[i].second);
    }
}

NamedValues layer_values_of(const AttributeStore& store, LayerRef layer) {
    std::vector<std::pair<AttributeKey, AttributeValue>> pairs;
    for (const AttributeKey key : store.keys_on(AttributeObject{}, layer)) {
        const AttributeQuery query = store.layer_value(layer, key);
        if (query.ok() && query.value != nullptr) pairs.emplace_back(key, *query.value);
    }
    return named(store, pairs);
}

NamedValues default_values_of(const AttributeStore& store) {
    std::vector<std::pair<AttributeKey, AttributeValue>> pairs;
    for (const AttributeKey key : store.keys_on(AttributeObject{}, kNoLayer)) {
        const AttributeQuery query = store.default_value(key);
        if (query.ok() && query.value != nullptr) pairs.emplace_back(key, *query.value);
    }
    return named(store, pairs);
}

void compare_attributes(const Document& a, const Document& b) {
    const AttributeStore& left = a.attributes();
    const AttributeStore& right = b.attributes();

    CHECK_EQ(a.slot_count(), b.slot_count());
    CHECK_EQ(a.object_count(), b.object_count());
    // The table has to be longer than the live list, or comparing it would only
    // be comparing the live objects a second time.
    CHECK_TRUE((a.object_count() < a.slot_count()));

    const std::vector<AttributeObject> left_objects = a.objects();
    const std::vector<AttributeObject> right_objects = b.objects();
    CHECK_EQ(left_objects.size(), right_objects.size());
    if (left_objects.size() == right_objects.size()) {
        for (size_t i = 0; i < left_objects.size(); ++i) {
            // Index AND generation. Comparing only the index would pass against a
            // loader that restarts every generation at 0.
            CHECK_EQ(left_objects[i].index, right_objects[i].index);
            CHECK_EQ(left_objects[i].generation, right_objects[i].generation);
            CHECK_EQ(a.object_layer(left_objects[i]), b.object_layer(right_objects[i]));

            const ObjectSnapshot left_snapshot = left.snapshot(left_objects[i]);
            const ObjectSnapshot right_snapshot = right.snapshot(right_objects[i]);
            compare_values(named(left, left_snapshot.object_values),
                           named(right, right_snapshot.object_values));
            compare_values(named(left, left_snapshot.user_values),
                           named(right, right_snapshot.user_values));
        }
    }

    compare_values(default_values_of(left), default_values_of(right));

    // Every layer that exists in either document, plus every layer an object
    // points at, so a layer whose values were dropped is caught rather than
    // skipped along with them.
    std::vector<LayerId> refs;
    for (const Document* doc : {&a, &b}) {
        for (const LayerId root : doc->layers().roots()) {
            for (const LayerId id : doc->layers().subtree_ids(root)) refs.push_back(id);
        }
        for (const AttributeObject obj : doc->objects()) {
            if (doc->object_layer(obj) != kInvalidLayer) refs.push_back(doc->object_layer(obj));
        }
    }
    std::sort(refs.begin(), refs.end());
    refs.erase(std::unique(refs.begin(), refs.end()), refs.end());
    CHECK_TRUE(!refs.empty());
    for (const LayerId id : refs) {
        compare_values(layer_values_of(left, static_cast<LayerRef>(id)),
                       layer_values_of(right, static_cast<LayerRef>(id)));
    }
}

// ============================================================================
// JSON surgery, for the refusal tests
// ============================================================================

std::string saved(Document& doc) {
    std::string text;
    const DocumentIoResult result = doc.save_to_json(text);
    CHECK_TRUE(result.ok);
    CHECK_TRUE(result.error.empty());
    return text;
}

/// Save a well-formed scene, mutate the JSON, and report what a load makes of it.
DocumentIoResult load_mutated(const std::function<void(json&)>& mutate) {
    Document source;
    build_scene(source);
    json document = json::parse(saved(source));
    mutate(document);

    Document destination;
    return destination.load_from_json(document.dump());
}

void expect_refusal(const DocumentIoResult& result, const std::string& expected_fragment) {
    CHECK_FALSE(result.ok);
    CHECK_FALSE(result.error.empty());
    // The message has to name the problem, not just say no: these are the only
    // words a user gets when a file will not open.
    CHECK_TRUE(result.error.find(expected_fragment) != std::string::npos);
    if (result.error.find(expected_fragment) == std::string::npos) {
        std::printf("    message was: %s\n", result.error.c_str());
    }
}

std::filesystem::path scratch_path(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

} // namespace

// ============================================================================
// Round trip
// ============================================================================

TEST(Document, round_trip_reproduces_the_whole_layer_tree) {
    Document source;
    const Scene scene = build_scene(source);
    const std::string text = saved(source);

    Document loaded;
    const DocumentIoResult result = loaded.load_from_json(text);
    CHECK_TRUE(result.ok);
    CHECK_EQ(result.error, std::string{});

    compare_trees(source.layers(), loaded.layers());

    // Spot checks on top of the walk, so a failure says WHICH property broke.
    CHECK_EQ(loaded.layers().size(), size_t{8});
    CHECK_TRUE(loaded.layers().find(scene.buildings) != nullptr);
    CHECK_FALSE(loaded.layers().find(scene.buildings)->own_colour.has_value());
    CHECK_TRUE(loaded.layers().find(scene.water)->own_colour.has_value());
    CHECK_FALSE(loaded.layers().effective_visible(scene.buildings));  // hidden group above it
    CHECK_TRUE(loaded.layers().effective_locked(scene.roads));
    CHECK_FALSE(loaded.layers().contains(scene.doomed));
}

TEST(Document, round_trip_reproduces_every_attribute) {
    Document source;
    const Scene scene = build_scene(source);
    const std::string text = saved(source);

    Document loaded;
    CHECK_TRUE(loaded.load_from_json(text).ok);

    compare_attributes(source, loaded);

    // The slot nothing lives in came back dead, at the generation it died at, so
    // the handle that used to name it cannot be revived by the next create().
    CHECK_FALSE(loaded.attributes().is_valid(scene.dead_slot));
    CHECK_EQ(loaded.slot_count(), source.slot_count());
    const AttributeObject next = loaded.create_object();
    CHECK_EQ(next.index, scene.dead_slot.index);
    CHECK_EQ(next.generation, scene.dead_slot.generation + 1u);
}

TEST(Document, sibling_order_is_file_order_not_id_order) {
    Document source;
    const Scene scene = build_scene(source);
    Document loaded;
    CHECK_TRUE(loaded.load_from_json(saved(source)).ok);

    const std::vector<LayerId>& children = loaded.layers().children(scene.world);
    CHECK_EQ(children.size(), size_t{3});
    if (children.size() == 3) {
        // Terrain was created last and moved to the front. Id order would put it
        // last, and alphabetical order would put Blocks first.
        CHECK_EQ(children[0], scene.terrain);
        CHECK_EQ(children[1], scene.roads);
        CHECK_EQ(children[2], scene.blocks);
    }
    CHECK_EQ(loaded.layers().index_of(scene.terrain), size_t{0});
}

TEST(Document, an_inherited_value_is_not_flattened_onto_the_object) {
    Document source;
    const Scene scene = build_scene(source);
    Document loaded;
    CHECK_TRUE(loaded.load_from_json(saved(source)).ok);

    const AttributeStore& store = loaded.attributes();
    const AttributeKey height = store.find_key("height");
    CHECK_TRUE(height.valid());

    const AttributeQuery query =
        store.resolve(scene.block_object, height, loaded.object_layer_ref(scene.block_object));
    CHECK_EQ(std::string(attribute_status_name(query.status)), std::string("Ok"));
    // The SOURCE is the assertion. A writer that saved the resolved 12.0 onto the
    // object would give the same number from source Object, and a test that read
    // only the number could not tell the two apart.
    CHECK_EQ(std::string(attribute_source_name(query.source)), std::string("Layer"));
    CHECK_EQ(query.layer, static_cast<LayerRef>(scene.blocks));
    CHECK_TRUE(query.value != nullptr);
    if (query.value != nullptr) CHECK_NEAR(*query.value->as_double(), 12.0, 0.0);

    // And the object really does own nothing, rather than owning a copy.
    const AttributeQuery peeked =
        store.peek(AttributeTarget::object(scene.block_object, height));
    CHECK_EQ(std::string(attribute_status_name(peeked.status)), std::string("Missing"));
}

TEST(Document, a_cleared_value_stays_cleared) {
    Document source;
    const Scene scene = build_scene(source);
    Document loaded;
    CHECK_TRUE(loaded.load_from_json(saved(source)).ok);

    const AttributeStore& store = loaded.attributes();
    const AttributeKey height = store.find_key("height");

    // It held 33.0 and gave it up. If the save had written what the object once
    // held, or what it now resolves to, this would read Object.
    const AttributeQuery peeked =
        store.peek(AttributeTarget::object(scene.cleared_object, height));
    CHECK_EQ(std::string(attribute_status_name(peeked.status)), std::string("Missing"));

    const AttributeQuery query =
        store.resolve(scene.cleared_object, height, loaded.object_layer_ref(scene.cleared_object));
    CHECK_EQ(std::string(attribute_source_name(query.source)), std::string("Layer"));
    if (query.value != nullptr) CHECK_NEAR(*query.value->as_double(), 12.0, 0.0);

    // Its other value is untouched, so "cleared" did not mean "cleared the object".
    const AttributeQuery kept =
        store.peek(AttributeTarget::user(scene.cleared_object, store.find_key("name")));
    CHECK_TRUE(kept.ok());
    if (kept.value != nullptr) CHECK_EQ(*kept.value->as_string(), std::string("Kept"));
}

TEST(Document, the_user_rung_and_the_object_rung_stay_apart) {
    Document source;
    const Scene scene = build_scene(source);
    Document loaded;
    CHECK_TRUE(loaded.load_from_json(saved(source)).ok);

    const AttributeStore& store = loaded.attributes();
    const AttributeKey height = store.find_key("height");

    const AttributeQuery object_rung =
        store.peek(AttributeTarget::object(scene.road_object, height));
    CHECK_TRUE(object_rung.ok());
    if (object_rung.value != nullptr) CHECK_NEAR(*object_rung.value->as_double(), 15.0, 0.0);

    const AttributeQuery user_rung = store.peek(AttributeTarget::user(scene.road_object, height));
    CHECK_TRUE(user_rung.ok());
    if (user_rung.value != nullptr) CHECK_NEAR(*user_rung.value->as_double(), 20.0, 0.0);

    // A loader that folded the two rungs together would still resolve to ONE of
    // these numbers, so the resolution below is checked as well as the two peeks.
    const AttributeQuery resolved =
        store.resolve(scene.road_object, height, loaded.object_layer_ref(scene.road_object));
    CHECK_EQ(std::string(attribute_source_name(resolved.source)), std::string("User"));
    if (resolved.value != nullptr) CHECK_NEAR(*resolved.value->as_double(), 20.0, 0.0);
}

TEST(Document, every_value_type_survives_including_empty_arrays) {
    Document source;
    const Scene scene = build_scene(source);
    Document loaded;
    CHECK_TRUE(loaded.load_from_json(saved(source)).ok);

    const AttributeStore& store = loaded.attributes();
    const ObjectSnapshot snapshot = store.snapshot(scene.road_object);

    const auto value_of = [&](const char* key_name) -> const AttributeValue* {
        const AttributeKey key = store.find_key(key_name);
        for (const auto& pair : snapshot.object_values) {
            if (pair.first == key) return &pair.second;
        }
        return nullptr;
    };

    const AttributeValue* tags = value_of("tags");
    CHECK_TRUE(tags != nullptr);
    if (tags != nullptr && tags->as_string_array() != nullptr) {
        const AttributeValue::StringArray& strings = *tags->as_string_array();
        CHECK_EQ(strings.size(), size_t{3});
        if (strings.size() == 3) {
            CHECK_EQ(strings[0], std::string("primary"));
            CHECK_EQ(strings[1], std::string("oneway"));
            CHECK_EQ(strings[2], std::string{});
        }
    }

    const AttributeValue* flags = value_of("flags");
    CHECK_TRUE(flags != nullptr);
    if (flags != nullptr && flags->as_bool_array() != nullptr) {
        const AttributeValue::BoolArray& bits = *flags->as_bool_array();
        CHECK_EQ(bits.size(), size_t{4});
        if (bits.size() == 4) {
            CHECK_TRUE(bits[0]);
            CHECK_FALSE(bits[1]);
            CHECK_TRUE(bits[2]);
            CHECK_TRUE(bits[3]);
        }
    }

    const AttributeValue* sealed = value_of("sealed");
    CHECK_TRUE(sealed != nullptr);
    if (sealed != nullptr && sealed->as_bool() != nullptr) CHECK_TRUE(*sealed->as_bool());

    // The reason the file carries a type tag at all: three different attributes
    // all write [], and only the tag says which is which.
    const AttributeValue* no_doubles = value_of("no_doubles");
    CHECK_TRUE(no_doubles != nullptr);
    if (no_doubles != nullptr) {
        CHECK_EQ(std::string(attribute_type_name(no_doubles->type())), std::string("double[]"));
        CHECK_EQ(no_doubles->size(), size_t{0});
    }
    const AttributeValue* no_flags = value_of("no_flags");
    CHECK_TRUE(no_flags != nullptr);
    if (no_flags != nullptr) {
        CHECK_EQ(std::string(attribute_type_name(no_flags->type())), std::string("bool[]"));
    }
    const AttributeQuery no_strings = store.layer_value(static_cast<LayerRef>(scene.blocks),
                                                        store.find_key("no_strings"));
    CHECK_TRUE(no_strings.ok());
    if (no_strings.value != nullptr) {
        CHECK_EQ(std::string(attribute_type_name(no_strings.value->type())),
                 std::string("string[]"));
        CHECK_EQ(no_strings.value->size(), size_t{0});
    }
}

TEST(Document, doubles_survive_to_the_last_bit) {
    Document doc;
    const LayerId layer = make_layer(doc, LayerKind::Group, "Precision");

    // Values chosen because a printf with the default precision mangles every one
    // of them: the transform compares with exact equality, so a lost digit is a
    // failure rather than a rounding.
    LayerTransform transform;
    transform.translation = glm::dvec3(0.1, 1.0 / 3.0, -2.2250738585072014e-308);
    transform.rotation = glm::dvec3(3.141592653589793, 1e300, -1e-300);
    transform.scale = glm::dvec3(0.30000000000000004, 1.7976931348623157e308, 1.0);
    CHECK_TRUE(
        run(doc, std::make_unique<SetLayerTransformCommand>(doc.layers(), layer, transform)));

    const AttributeKey key = doc.attributes().intern("numbers");
    const AttributeObject object = doc.create_object(layer);
    set_attribute(doc.history(), doc.attributes(), AttributeTarget::object(object, key),
                  AttributeValue::from_doubles({0.1, 1.0 / 3.0, 1e-300, -0.0}));

    Document loaded;
    CHECK_TRUE(loaded.load_from_json(saved(doc)).ok);

    const Layer* restored = loaded.layers().find(layer);
    CHECK_TRUE(restored != nullptr);
    if (restored != nullptr) CHECK_TRUE(restored->own_transform == transform);

    const ObjectSnapshot snapshot = loaded.attributes().snapshot(object);
    CHECK_EQ(snapshot.object_values.size(), size_t{1});
    if (snapshot.object_values.size() == 1) {
        const AttributeValue::DoubleArray* numbers =
            snapshot.object_values[0].second.as_double_array();
        CHECK_TRUE(numbers != nullptr);
        if (numbers != nullptr && numbers->size() == 4) {
            CHECK_TRUE((*numbers)[0] == 0.1);
            CHECK_TRUE((*numbers)[1] == 1.0 / 3.0);
            CHECK_TRUE((*numbers)[2] == 1e-300);
            // -0.0 == 0.0 compares true, so the sign bit is what is checked.
            CHECK_TRUE(std::signbit((*numbers)[3]));
        }
    }
}

TEST(Document, an_empty_document_round_trips) {
    Document empty;
    const std::string text = saved(empty);

    Document loaded;
    const DocumentIoResult result = loaded.load_from_json(text);
    CHECK_TRUE(result.ok);
    CHECK_TRUE(loaded.layers().empty());
    CHECK_EQ(loaded.object_count(), size_t{0});
    CHECK_EQ(loaded.slot_count(), size_t{0});
    CHECK_FALSE(loaded.dirty());

    // And the empty document is still a Stratum document, not an empty file.
    const json document = json::parse(text);
    CHECK_EQ(document.at("format").get<std::string>(), std::string("stratum-document"));
    CHECK_EQ(document.at("version").get<uint32_t>(), kDocumentFormatVersion);
}

TEST(Document, saving_a_loaded_document_reproduces_the_same_file) {
    Document source;
    build_scene(source);
    const std::string first = saved(source);

    Document loaded;
    CHECK_TRUE(loaded.load_from_json(first).ok);
    const std::string second = saved(loaded);

    // Byte-for-byte, which pins down every ordering decision in the writer at
    // once: unordered_map iteration order leaking into the key table, the value
    // lists or the layer list would show up here and nowhere else.
    CHECK_EQ(first.size(), second.size());
    CHECK_TRUE(first == second);
}

// ============================================================================
// Allocators
// ============================================================================

TEST(Document, the_layer_id_counter_survives_a_deleted_layer) {
    Document source;
    const Scene scene = build_scene(source);
    const std::string text = saved(source);

    const auto next_in_file = json::parse(text).at("layers").at("next_id").get<LayerId>();
    CHECK_TRUE((scene.doomed < next_in_file));

    Document loaded;
    CHECK_TRUE(loaded.load_from_json(text).ok);
    CHECK_FALSE(loaded.layers().contains(scene.doomed));

    const LayerId fresh = make_layer(loaded, LayerKind::Shape, "Fresh");
    // The whole point: a loader that restarted the counter from the surviving
    // layers would hand out an id at or below `doomed`, and every LayerId held
    // elsewhere that named the deleted layer would start naming this one.
    CHECK_TRUE((scene.doomed < fresh));
    CHECK_EQ(fresh, next_in_file);

    // Saving twice takes one id each time, by design -- LayerTree's counter can
    // only be read by taking from it. Pinned here so that changing it is a
    // decision rather than an accident.
    Document again;
    CHECK_TRUE(again.load_from_json(text).ok);
    const auto second_save = json::parse(saved(again)).at("layers").at("next_id").get<LayerId>();
    const auto third_save = json::parse(saved(again)).at("layers").at("next_id").get<LayerId>();
    CHECK_EQ(second_save, next_in_file);
    CHECK_EQ(third_save, static_cast<LayerId>(next_in_file + 1u));
}

TEST(Document, a_stale_object_handle_is_still_stale_after_a_load) {
    Document source;
    const Scene scene = build_scene(source);
    CHECK_EQ(scene.stale_object.index, scene.reborn_object.index);
    CHECK_EQ(scene.reborn_object.generation, scene.stale_object.generation + 1u);

    Document loaded;
    CHECK_TRUE(loaded.load_from_json(saved(source)).ok);

    // A loader that restored objects without their generations would give the
    // recycled slot generation 0, and these two answers would swap.
    CHECK_TRUE(loaded.attributes().is_valid(scene.reborn_object));
    CHECK_FALSE(loaded.attributes().is_valid(scene.stale_object));

    const AttributeQuery name =
        loaded.attributes().peek(AttributeTarget::object(scene.reborn_object,
                                                         loaded.attributes().find_key("name")));
    CHECK_TRUE(name.ok());
    if (name.value != nullptr) CHECK_EQ(*name.value->as_string(), std::string("Reborn"));
}

TEST(Document, a_dead_slots_generation_survives_so_the_next_handle_cannot_alias) {
    Document doc;
    const AttributeObject kept = doc.create_object();
    CHECK_EQ(kept.index, uint32_t{0});

    AttributeObject recycled = doc.create_object();
    CHECK_EQ(recycled.index, uint32_t{1});

    // Three recycles of slot 1, so its generation is a number no accident
    // reproduces: a loader that pads dead slots with one create/destroy pair
    // lands on 1, and a loader that ignores them lands on 0.
    for (uint32_t round = 0; round < 3; ++round) {
        CHECK_TRUE(doc.destroy_object(recycled));
        recycled = doc.create_object();
        CHECK_EQ(recycled.index, uint32_t{1});
        CHECK_EQ(recycled.generation, round + 1u);
    }
    CHECK_TRUE(doc.destroy_object(recycled));  // Slot 1 is now dead, next generation 4.

    Document loaded;
    CHECK_TRUE(loaded.load_from_json(saved(doc)).ok);

    CHECK_EQ(loaded.slot_count(), size_t{2});
    CHECK_EQ(loaded.object_count(), size_t{1});
    CHECK_TRUE(loaded.attributes().is_valid(kept));

    // Every handle the saved session ever issued for slot 1 is dead, and stays
    // dead.
    for (uint32_t generation = 0; generation <= 3; ++generation) {
        CHECK_FALSE(loaded.attributes().is_valid(AttributeObject{1u, generation}));
    }

    const AttributeObject fresh = loaded.create_object();
    CHECK_EQ(fresh.index, uint32_t{1});
    CHECK_EQ(fresh.generation, uint32_t{4});
}

TEST(Document, object_layer_membership_survives_including_a_deleted_layer) {
    Document source;
    const Scene scene = build_scene(source);
    Document loaded;
    CHECK_TRUE(loaded.load_from_json(saved(source)).ok);

    CHECK_EQ(loaded.object_layer(scene.road_object), scene.roads);
    CHECK_EQ(loaded.object_layer(scene.block_object), scene.blocks);
    CHECK_EQ(loaded.object_layer_ref(scene.road_object), static_cast<LayerRef>(scene.roads));

    // Its layer was deleted before the save. The id is kept verbatim because A1
    // never reuses one, so the reference can never come to mean a different
    // layer; zeroing it would lose what the live session still knew.
    CHECK_EQ(loaded.object_layer(scene.orphan_object), scene.doomed);
    CHECK_FALSE(loaded.layers().contains(scene.doomed));
}

// ============================================================================
// Version and untrusted input
// ============================================================================

TEST(Document, refuses_a_newer_format_version) {
    const DocumentIoResult result =
        load_mutated([](json& document) { document["version"] = kDocumentFormatVersion + 1; });
    expect_refusal(result, "newer version");
}

TEST(Document, refuses_a_file_that_is_not_a_stratum_document) {
    expect_refusal(load_mutated([](json& document) { document["format"] = "blender"; }),
                   "not a Stratum document");

    Document doc;
    expect_refusal(doc.load_from_json("{}"), "format");
    expect_refusal(doc.load_from_json("[1, 2, 3]"), "not a JSON object");
}

TEST(Document, refuses_malformed_and_truncated_json) {
    Document doc;
    expect_refusal(doc.load_from_json(""), "not valid JSON");
    expect_refusal(doc.load_from_json("{"), "not valid JSON");
    expect_refusal(doc.load_from_json("{\"format\": }"), "not valid JSON");
    expect_refusal(doc.load_from_json("\xff\xfe\x00\x01 binary"), "not valid JSON");

    Document source;
    build_scene(source);
    const std::string text = saved(source);

    // Every truncation of a real document, at a few dozen lengths, is refused and
    // none of them crashes or half-loads.
    for (size_t cut = 1; cut < text.size(); cut += text.size() / 37 + 1) {
        Document destination;
        const DocumentIoResult result = destination.load_from_json(text.substr(0, cut));
        CHECK_FALSE(result.ok);
        CHECK_FALSE(result.error.empty());
        CHECK_TRUE(destination.layers().empty());
    }
}

TEST(Document, refuses_a_missing_or_mistyped_field) {
    expect_refusal(load_mutated([](json& document) {
                       document["layers"]["roots"][0].erase("own_visible");
                   }),
                   "own_visible: missing");

    // Silently defaulting this one would un-hide a layer the user hid.
    expect_refusal(load_mutated([](json& document) {
                       document["layers"]["roots"][0]["own_visible"] = "true";
                   }),
                   "expected true or false");

    expect_refusal(load_mutated([](json& document) {
                       document["layers"]["roots"][0]["own_transform"]["scale"] = {1.0, 2.0};
                   }),
                   "expected 3 numbers");

    expect_refusal(load_mutated([](json& document) {
                       document["layers"]["roots"][0]["own_transform"]["scale"][1] = nullptr;
                   }),
                   "expected a number");

    expect_refusal(load_mutated([](json& document) { document["layers"]["roots"][0]["id"] = -4; }),
                   "expected a non-negative whole number");

    expect_refusal(load_mutated([](json& document) {
                       document["layers"]["roots"][0]["kind"] = "Sandwich";
                   }),
                   "is not a layer kind");

    expect_refusal(load_mutated([](json& document) { document.erase("attributes"); }),
                   "attributes: missing");
}

TEST(Document, refuses_a_self_contradictory_file) {
    // Two layers with one id: the second insert would be refused by the tree and
    // the file would load as a scene missing a branch.
    expect_refusal(load_mutated([](json& document) {
                       document["layers"]["roots"][1]["id"] =
                           document["layers"]["roots"][0]["id"];
                   }),
                   "duplicate id");

    // A counter at or below an id already in the file would reissue that id.
    expect_refusal(load_mutated([](json& document) { document["layers"]["next_id"] = 1; }),
                   "not above the highest layer id");

    // An object table whose entries do not match their positions.
    expect_refusal(load_mutated([](json& document) {
                       document["attributes"]["objects"][0]["index"] = 3;
                   }),
                   "the slot table is the array order");

    // A value naming a key the file's own table never declared.
    expect_refusal(load_mutated([](json& document) {
                       document["attributes"]["defaults"][0]["key"] = "invented";
                   }),
                   "is not in attributes.keys");

    // The same key twice in one list: the second would silently win.
    expect_refusal(load_mutated([](json& document) {
                       json list = document["attributes"]["defaults"];
                       list.push_back(list[0]);
                       document["attributes"]["defaults"] = list;
                   }),
                   "appears twice in this list");

    expect_refusal(load_mutated([](json& document) {
                       document["attributes"]["keys"].push_back(
                           document["attributes"]["keys"][0]);
                   }),
                   "appears twice in the key table");
}

TEST(Document, refuses_a_dead_slot_that_claims_to_own_things) {
    // Found rather than assumed: which slot the fixture leaves dead is an
    // implementation detail of build_scene(), and hard-coding it would make this
    // test quietly stop testing anything the day the fixture changes.
    bool found_a_dead_slot = false;
    const DocumentIoResult result = load_mutated([&found_a_dead_slot](json& document) {
        for (json& slot : document["attributes"]["objects"]) {
            if (slot["alive"].get<bool>()) continue;
            slot["object_values"] = json::array();
            found_a_dead_slot = true;
            return;
        }
    });
    // Asserted, because a mutation that found nothing to mutate would leave a
    // valid document and this test would pass without testing anything.
    CHECK_TRUE(found_a_dead_slot);
    expect_refusal(result, "is set on a slot that is not alive");

    // A slot that is dead at generation 0 was never alive, which no session can
    // produce and the loader cannot reproduce.
    expect_refusal(load_mutated([](json& document) {
                       document["attributes"]["objects"][0]["alive"] = false;
                       document["attributes"]["objects"][0]["generation"] = 0;
                       document["attributes"]["objects"][0].erase("layer");
                       document["attributes"]["objects"][0].erase("object_values");
                       document["attributes"]["objects"][0].erase("user_values");
                   }),
                   "no session can produce");
}

TEST(Document, refuses_an_absurd_object_generation) {
    // Ten characters in a file, four billion destroy/create pairs on load. The
    // cap is what turns that into a message.
    expect_refusal(load_mutated([](json& document) {
                       document["attributes"]["objects"][0]["generation"] = 4000000000u;
                   }),
                   "above this build's limit");
}

TEST(Document, refuses_layer_nesting_deep_enough_to_overflow_the_stack) {
    // Built as text rather than through the writer, because the writer refuses to
    // produce it -- which is the other half of the guard.
    constexpr size_t kDepth = 20000;
    std::string nested;
    nested.reserve(kDepth * 200);
    for (size_t i = 1; i <= kDepth; ++i) {
        nested += R"({"id":)" + std::to_string(i)
                  + R"(,"kind":"Group","name":"","own_visible":true,"own_locked":false,)"
                    R"("own_colour":null,"own_transform":{"translation":[0,0,0],)"
                    R"("rotation":[0,0,0],"scale":[1,1,1]},"children":[)";
    }
    for (size_t i = 0; i < kDepth; ++i) nested += "]}";

    const std::string text = R"({"format":"stratum-document","version":1,"blobs":[],"layers":)"
                             R"({"next_id":)"
                             + std::to_string(kDepth + 1) + R"(,"roots":[)" + nested
                             + R"(]},"attributes":{"keys":[],"defaults":[],"layer_values":[],)"
                               R"("objects":[]}})";

    Document doc;
    expect_refusal(doc.load_from_json(text), "layer nesting is deeper than");
    CHECK_TRUE(doc.layers().empty());
}

TEST(Document, refuses_a_document_that_references_blobs_it_cannot_load) {
    // Ignoring the reference would open the scene with its geometry silently
    // missing, which the user discovers much later.
    expect_refusal(load_mutated([](json& document) {
                       document["blobs"].push_back({{"id", "terrain"}, {"path", "terrain.bin"}});
                   }),
                   "binary blob");
}

TEST(Document, refuses_to_save_a_double_json_cannot_write) {
    Document doc;
    const LayerId layer = make_layer(doc, LayerKind::Group, "Broken");
    LayerTransform poisoned;
    poisoned.translation = glm::dvec3(std::nan(""), 0.0, 0.0);
    CHECK_TRUE(run(doc, std::make_unique<SetLayerTransformCommand>(doc.layers(), layer, poisoned)));

    std::string text = "untouched";
    const DocumentIoResult result = doc.save_to_json(text);
    CHECK_FALSE(result.ok);
    CHECK_TRUE(result.error.find("finite") != std::string::npos);
    // The output buffer is left alone, so a caller cannot write half a document.
    CHECK_EQ(text, std::string("untouched"));

    // The same guard covers an attribute value, which is the other place a
    // double reaches the file.
    Document other;
    const AttributeObject object = other.create_object();
    set_attribute(other.history(), other.attributes(),
                  AttributeTarget::object(object, other.attributes().intern("bad")),
                  AttributeValue::from_doubles({1.0, std::numeric_limits<double>::infinity()}));
    std::string more = "untouched";
    const DocumentIoResult value_result = other.save_to_json(more);
    CHECK_FALSE(value_result.ok);
    CHECK_TRUE(value_result.error.find("finite") != std::string::npos);
    CHECK_EQ(more, std::string("untouched"));
}

TEST(Document, a_refused_load_leaves_the_document_exactly_as_it_was) {
    Document doc;
    const Scene scene = build_scene(doc);
    const std::string before = saved(doc);
    const uint64_t revision = doc.history().revision();
    const size_t undo_depth = doc.history().undo_depth();

    json broken = json::parse(before);
    broken["attributes"]["objects"][0]["generation"] = 4000000000u;
    const DocumentIoResult result = doc.load_from_json(broken.dump());
    CHECK_FALSE(result.ok);

    // Not "mostly as it was": the tree, the objects, the history and the dirty
    // state all have to be untouched, because the user is still looking at them.
    CHECK_TRUE(doc.layers().contains(scene.world));
    CHECK_EQ(doc.layers().size(), size_t{8});
    CHECK_TRUE(doc.attributes().is_valid(scene.road_object));
    CHECK_EQ(doc.history().revision(), revision);
    CHECK_EQ(doc.history().undo_depth(), undo_depth);
    CHECK_TRUE(doc.history().can_undo());

    // And it still saves to the same bytes it would have before the failed load
    // -- except for the one layer id every save takes.
    json after = json::parse(saved(doc));
    json expected = json::parse(before);
    expected["layers"]["next_id"] = after["layers"]["next_id"];
    CHECK_TRUE(after == expected);
}

TEST(Document, ignores_members_it_does_not_know) {
    // Forward compatibility within a version: a build that writes A5's selection
    // block must still be readable here. document.hpp records which reserved
    // members may be added without a version bump.
    Document source;
    build_scene(source);
    json document = json::parse(saved(source));
    document["selection"] = {{"primary", 3}};
    document["something_from_2027"] = 17;
    document["layers"]["roots"][0]["future_field"] = true;

    Document loaded;
    const DocumentIoResult result = loaded.load_from_json(document.dump());
    CHECK_TRUE(result.ok);
    CHECK_EQ(result.error, std::string{});
    compare_trees(source.layers(), loaded.layers());
}

// ============================================================================
// History and dirtiness
// ============================================================================

TEST(Document, a_load_clears_the_history_without_resetting_the_revision) {
    Document doc;
    build_scene(doc);
    CHECK_TRUE(doc.history().can_undo());
    const uint64_t before = doc.history().revision();

    Document other;
    build_scene(other);
    const std::string text = saved(other);

    CHECK_TRUE(doc.load_from_json(text).ok);

    // Cleared: a command holds the inverse of an edit to a tree and a store that
    // are now different objects holding different content, so undoing one would
    // apply it to a scene that never had the original.
    CHECK_FALSE(doc.history().can_undo());
    CHECK_FALSE(doc.history().can_redo());
    CHECK_EQ(doc.history().undo_depth(), size_t{0});
    CHECK_EQ(doc.history().bytes(), size_t{0});

    // NOT reset: revision is how dirtiness is answered, and a freshly loaded
    // document must not compare equal to an unsaved one that happened to be at
    // zero. See the comment on CommandStack::clear().
    CHECK_TRUE((before < doc.history().revision()));
    CHECK_FALSE(doc.dirty());
}

TEST(Document, dirty_follows_both_edits_and_object_lifetime) {
    Document doc;
    CHECK_FALSE(doc.dirty());

    const LayerId layer = make_layer(doc, LayerKind::Group, "One");
    CHECK_TRUE(doc.dirty());

    doc.mark_saved();
    CHECK_FALSE(doc.dirty());

    // An undo is a change too: the question is "does this differ from what was
    // saved", not "how many edits have happened".
    CHECK_TRUE(doc.history().undo());
    CHECK_TRUE(doc.dirty());
    CHECK_TRUE(doc.history().redo());
    doc.mark_saved();
    CHECK_FALSE(doc.dirty());

    // Object lifetime is not a command, so the command stack's revision cannot
    // see it. Without the structural counter, creating an object and quitting
    // would lose it with no prompt.
    const AttributeObject object = doc.create_object(layer);
    CHECK_TRUE(doc.dirty());
    doc.mark_saved();
    CHECK_FALSE(doc.dirty());

    CHECK_TRUE(doc.set_object_layer(object, kInvalidLayer));
    CHECK_TRUE(doc.dirty());
    doc.mark_saved();

    CHECK_TRUE(doc.destroy_object(object));
    CHECK_TRUE(doc.dirty());
}

// ============================================================================
// Files
// ============================================================================

TEST(Document, a_document_round_trips_through_a_file) {
    Document source;
    const Scene scene = build_scene(source);
    const std::filesystem::path path = scratch_path("stratum_document_round_trip.stratum");
    std::error_code ignored;
    std::filesystem::remove(path, ignored);

    CHECK_TRUE(source.dirty());
    const DocumentIoResult written = source.save_to_file(path);
    CHECK_TRUE(written.ok);
    CHECK_EQ(written.error, std::string{});
    CHECK_FALSE(source.dirty());
    CHECK_TRUE(std::filesystem::exists(path));
    // The temporary the writer renames from must not be left behind.
    CHECK_FALSE(std::filesystem::exists(std::filesystem::path(path).concat(".tmp")));

    Document loaded;
    const DocumentIoResult read = loaded.load_from_file(path);
    CHECK_TRUE(read.ok);
    CHECK_EQ(read.error, std::string{});
    compare_trees(source.layers(), loaded.layers());
    compare_attributes(source, loaded);
    CHECK_TRUE(loaded.attributes().is_valid(scene.reborn_object));
    CHECK_FALSE(loaded.attributes().is_valid(scene.stale_object));

    std::filesystem::remove(path, ignored);
}

TEST(Document, a_failed_save_leaves_the_file_that_was_there_alone) {
    const std::filesystem::path path = scratch_path("stratum_document_failed_save.stratum");
    std::error_code ignored;
    std::filesystem::remove(path, ignored);

    Document doc;
    const LayerId layer = make_layer(doc, LayerKind::Group, "Good");
    CHECK_TRUE(doc.save_to_file(path).ok);
    const std::string good = saved(doc);

    // Poison the scene so rendering the JSON fails, then save over the same path.
    LayerTransform poisoned;
    poisoned.translation = glm::dvec3(0.0, std::numeric_limits<double>::infinity(), 0.0);
    CHECK_TRUE(run(doc, std::make_unique<SetLayerTransformCommand>(doc.layers(), layer, poisoned)));

    const DocumentIoResult result = doc.save_to_file(path);
    CHECK_FALSE(result.ok);
    CHECK_TRUE(doc.dirty());  // A failed save must not claim the document is saved.

    // The good file is still there and still loads.
    Document loaded;
    const DocumentIoResult read = loaded.load_from_file(path);
    CHECK_TRUE(read.ok);
    CHECK_EQ(loaded.layers().size(), size_t{1});
    CHECK_TRUE(loaded.layers().contains(layer));
    CHECK_TRUE(good.size() > 0);

    std::filesystem::remove(path, ignored);
    std::filesystem::remove(std::filesystem::path(path).concat(".tmp"), ignored);
}

TEST(Document, refuses_a_file_that_is_not_there_or_is_not_a_file) {
    Document doc;
    expect_refusal(doc.load_from_file(scratch_path("stratum_no_such_document.stratum")),
                   "is not a file");
    expect_refusal(doc.load_from_file(std::filesystem::temp_directory_path()), "is not a file");
}
