// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "scene/document.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <string>
#include <system_error>
#include <unordered_set>
#include <utility>
#include <vector>

namespace stratum::scene {

namespace {

using nlohmann::json;

// ============================================================================
// Member names
//
// Spelled once each. A literal repeated between the writer and the reader is a
// place for a typo to live, and the only symptom of that typo is a field that
// silently reverts to its default the first time a document is reopened.
// ============================================================================

constexpr const char* kFormat = "format";
constexpr const char* kVersion = "version";
constexpr const char* kBlobs = "blobs";
constexpr const char* kLayers = "layers";
constexpr const char* kNextId = "next_id";
constexpr const char* kRoots = "roots";
constexpr const char* kChildren = "children";
constexpr const char* kId = "id";
constexpr const char* kKind = "kind";
constexpr const char* kName = "name";
constexpr const char* kOwnVisible = "own_visible";
constexpr const char* kOwnLocked = "own_locked";
constexpr const char* kOwnColour = "own_colour";
constexpr const char* kOwnTransform = "own_transform";
constexpr const char* kTranslation = "translation";
constexpr const char* kRotation = "rotation";
constexpr const char* kScale = "scale";
constexpr const char* kAttributes = "attributes";
constexpr const char* kKeys = "keys";
constexpr const char* kDefaults = "defaults";
constexpr const char* kLayerValues = "layer_values";
constexpr const char* kObjects = "objects";
constexpr const char* kIndex = "index";
constexpr const char* kGeneration = "generation";
constexpr const char* kAlive = "alive";
constexpr const char* kLayer = "layer";
constexpr const char* kValues = "values";
constexpr const char* kObjectValues = "object_values";
constexpr const char* kUserValues = "user_values";
constexpr const char* kKey = "key";
constexpr const char* kType = "type";
constexpr const char* kValue = "value";

/// Two spaces. The file sits in a user's project directory and, in practice, in
/// their version control: a diff that shows the one layer they renamed is worth
/// more than the bytes minifying would save. What would actually be large --
/// geometry -- is in the blobs beside the file, not in it.
constexpr int kJsonIndent = 2;

/// Highest usable layer id. 0xFFFFFFFF is excluded because LayerTree::insert()
/// advances its counter to id + 1, which would wrap to 0 -- kInvalidLayer -- and
/// hand the next create() an id that reads as "no layer".
constexpr LayerId kMaxLayerId = 0xFFFFFFFEu;

// ============================================================================
// Enum spellings
//
// Both parsers are derived from the WRITER's own naming function rather than
// from a second table. A hand-written parse table is somewhere for the two to
// drift apart, and the symptom is a file this build wrote and cannot read back.
// ============================================================================

[[nodiscard]] std::optional<AttributeType> attribute_type_from_name(std::string_view name) {
    for (uint8_t i = 0; i <= static_cast<uint8_t>(AttributeType::StringArray); ++i) {
        const auto type = static_cast<AttributeType>(i);
        if (attribute_type_name(type) == name) return type;
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<LayerKind> layer_kind_from_name(std::string_view name) {
    for (size_t i = 0; i < kLayerKindCount; ++i) {
        const auto kind = static_cast<LayerKind>(i);
        if (name == layer_kind_name(kind)) return kind;
    }
    return std::nullopt;
}

// ============================================================================
// The door into LayerTree's private mutating API
// ============================================================================

/**
 * @brief Lets the loader put a layer back with the id the FILE gives it
 *
 * Every mutation of a LayerTree goes through a LayerCommand -- that is the point
 * of A4 -- and a load is no exception to the mechanism, only to the history.
 * CreateLayerCommand cannot serve here because it mints a FRESH id, which is
 * exactly what a load must not do, so this subclass reaches the same protected
 * forwarders and inserts layers verbatim instead.
 *
 * It is never pushed onto a CommandStack: a load is not an undoable edit. See
 * the note on clearing the history in document.hpp.
 */
class LayerLoadCommand final : public LayerCommand {
public:
    explicit LayerLoadCommand(LayerTree& tree) : LayerCommand(tree, kInvalidLayer) {}

    /// Refuses. It is never executed, and refusing is the outcome that leaves
    /// the history untouched if some future caller ever tries.
    bool apply() override { return false; }
    void revert() override {}
    [[nodiscard]] std::string describe() const override { return "Load layers"; }

    /// Takes the next layer id. The only way to READ LayerTree's counter is to
    /// take from it; see the note on Document::save_to_json().
    [[nodiscard]] static LayerId take_next_id(LayerTree& tree) {
        return LayerCommand::reserve_id(tree);
    }

    using LayerCommand::detach_subtree;
    using LayerCommand::insert_layer;
};

// ============================================================================
// Failure reporting
//
// Every message names the JSON path it came from. "layers.roots[2].own_transform
// .scale: expected 3 numbers" is a bug report someone can act on; "invalid
// document" is a round trip with the user before the same question gets asked
// again.
// ============================================================================

[[nodiscard]] bool bad(std::string& error, const std::string& where, const std::string& what) {
    error = where + ": " + what;
    return false;
}

[[nodiscard]] std::string at(const std::string& where, const char* member) {
    return where.empty() ? std::string(member) : where + "." + member;
}

[[nodiscard]] std::string at(const std::string& where, size_t index) {
    return where + "[" + std::to_string(index) + "]";
}

// ============================================================================
// Typed readers
//
// Each one refuses rather than coercing. A JSON string where a bool belongs is a
// corrupt or hand-edited file, and "true" quietly reading as true is how a layer
// the user hid comes back visible.
// ============================================================================

[[nodiscard]] const json* member_of(const json& parent, const char* name) {
    const auto it = parent.find(name);
    return it == parent.end() ? nullptr : &*it;
}

[[nodiscard]] bool require_member(const json& parent, const char* name, const std::string& where,
                                  const json*& out, std::string& error) {
    out = member_of(parent, name);
    if (out == nullptr) {
        // Refusing beats defaulting. A missing member means a truncated or
        // hand-edited file, and a default silently produces a different scene
        // from the one that was saved.
        return bad(error, at(where, name), "missing");
    }
    return true;
}

[[nodiscard]] bool read_bool(const json& parent, const char* name, const std::string& where,
                             bool& out, std::string& error) {
    const json* node = nullptr;
    if (!require_member(parent, name, where, node, error)) return false;
    if (!node->is_boolean()) return bad(error, at(where, name), "expected true or false");
    out = node->get<bool>();
    return true;
}

[[nodiscard]] bool read_string(const json& parent, const char* name, const std::string& where,
                               std::string& out, std::string& error) {
    const json* node = nullptr;
    if (!require_member(parent, name, where, node, error)) return false;
    if (!node->is_string()) return bad(error, at(where, name), "expected a string");
    out = node->get<std::string>();
    return true;
}

/**
 * @brief Read an unsigned integer, refusing anything that is not one
 *
 * is_number_unsigned() rather than is_number(): -1 and 3.5 are not ids,
 * generations or indices, and letting either through would truncate into a
 * plausible-looking value. nlohmann tracks the three number kinds separately, so
 * the distinction costs nothing here.
 */
[[nodiscard]] bool read_u64(const json& node, const std::string& where, uint64_t max, uint64_t& out,
                            std::string& error) {
    if (!node.is_number_unsigned()) {
        return bad(error, where, "expected a non-negative whole number");
    }
    const uint64_t value = node.get<uint64_t>();
    if (value > max) {
        return bad(error, where,
                   "is " + std::to_string(value) + ", above this build's limit of "
                       + std::to_string(max));
    }
    out = value;
    return true;
}

[[nodiscard]] bool read_u64_member(const json& parent, const char* name, const std::string& where,
                                   uint64_t max, uint64_t& out, std::string& error) {
    const json* node = nullptr;
    if (!require_member(parent, name, where, node, error)) return false;
    return read_u64(*node, at(where, name), max, out, error);
}

/**
 * @brief Read a finite double
 *
 * JSON has no spelling for NaN or an infinity, so nlohmann writes them as null
 * and reads null as a non-number. Refusing here means a file carrying one is
 * reported rather than silently becoming 0.0 -- which, in a transform, is a
 * layer that jumps to the origin.
 */
[[nodiscard]] bool read_double(const json& node, const std::string& where, double& out,
                               std::string& error) {
    if (!node.is_number()) return bad(error, where, "expected a number");
    const double value = node.get<double>();
    if (!std::isfinite(value)) return bad(error, where, "is not a finite number");
    out = value;
    return true;
}

template <typename Vec>
[[nodiscard]] bool read_vec3(const json& parent, const char* name, const std::string& where,
                             Vec& out, std::string& error) {
    const json* node = nullptr;
    if (!require_member(parent, name, where, node, error)) return false;
    const std::string path = at(where, name);
    if (!node->is_array() || node->size() != 3) return bad(error, path, "expected 3 numbers");

    for (size_t i = 0; i < 3; ++i) {
        double component = 0.0;
        if (!read_double((*node)[i], at(path, i), component, error)) return false;
        out[static_cast<int>(i)] = static_cast<typename Vec::value_type>(component);
    }
    return true;
}

// ============================================================================
// Values
// ============================================================================

[[nodiscard]] bool write_double(double value, const std::string& where, json& out,
                                std::string& error) {
    if (!std::isfinite(value)) {
        // Writing it produces `null`, which reads back as a non-number: the file
        // this build just wrote would be one it refuses to open. Refusing the
        // SAVE is the only outcome that neither corrupts the scene nor writes an
        // unreadable file. Encoding non-finite doubles as strings was the
        // alternative and lost -- it makes "value" sometimes a number and
        // sometimes not, in a format whose job is keeping types straight.
        return bad(error, where,
                   "is not a finite number, and JSON has no spelling for NaN or an infinity");
    }
    out = value;
    return true;
}

[[nodiscard]] bool write_attribute_value(const AttributeValue& value, const std::string& where,
                                         json& out, std::string& error) {
    // The type tag is written even where JSON could carry the type on its own,
    // because an empty array is otherwise three different attributes wearing the
    // same two characters: bool[], double[] and string[] all save as [].
    out[kType] = std::string(attribute_type_name(value.type()));
    const std::string value_path = at(where, kValue);

    switch (value.type()) {
    case AttributeType::Bool:
        out[kValue] = *value.as_bool();
        return true;
    case AttributeType::Double: {
        json number;
        if (!write_double(*value.as_double(), value_path, number, error)) return false;
        out[kValue] = std::move(number);
        return true;
    }
    case AttributeType::String:
        out[kValue] = *value.as_string();
        return true;
    case AttributeType::BoolArray: {
        json array = json::array();
        for (const bool element : *value.as_bool_array()) array.push_back(element);
        out[kValue] = std::move(array);
        return true;
    }
    case AttributeType::DoubleArray: {
        json array = json::array();
        const AttributeValue::DoubleArray& elements = *value.as_double_array();
        for (size_t i = 0; i < elements.size(); ++i) {
            json number;
            if (!write_double(elements[i], at(value_path, i), number, error)) return false;
            array.push_back(std::move(number));
        }
        out[kValue] = std::move(array);
        return true;
    }
    case AttributeType::StringArray: {
        json array = json::array();
        for (const std::string& element : *value.as_string_array()) array.push_back(element);
        out[kValue] = std::move(array);
        return true;
    }
    }
    return bad(error, where, "has an attribute type this build cannot write");
}

[[nodiscard]] bool read_attribute_value(const json& node, const std::string& where,
                                        std::optional<AttributeValue>& out, std::string& error) {
    std::string type_name;
    if (!read_string(node, kType, where, type_name, error)) return false;

    const std::optional<AttributeType> type = attribute_type_from_name(type_name);
    if (!type.has_value()) {
        return bad(error, at(where, kType), "\"" + type_name + "\" is not an attribute type");
    }

    const json* value = nullptr;
    if (!require_member(node, kValue, where, value, error)) return false;
    const std::string path = at(where, kValue);

    switch (*type) {
    case AttributeType::Bool: {
        if (!value->is_boolean()) return bad(error, path, "expected true or false");
        out = AttributeValue::from_bool(value->get<bool>());
        return true;
    }
    case AttributeType::Double: {
        double number = 0.0;
        if (!read_double(*value, path, number, error)) return false;
        out = AttributeValue::from_double(number);
        return true;
    }
    case AttributeType::String: {
        if (!value->is_string()) return bad(error, path, "expected a string");
        out = AttributeValue::from_string(value->get<std::string>());
        return true;
    }
    case AttributeType::BoolArray: {
        if (!value->is_array()) return bad(error, path, "expected an array");
        AttributeValue::BoolArray elements;
        elements.reserve(value->size());
        for (size_t i = 0; i < value->size(); ++i) {
            const json& element = (*value)[i];
            if (!element.is_boolean()) return bad(error, at(path, i), "expected true or false");
            elements.push_back(element.get<bool>());
        }
        out = AttributeValue::from_bools(std::move(elements));
        return true;
    }
    case AttributeType::DoubleArray: {
        if (!value->is_array()) return bad(error, path, "expected an array");
        AttributeValue::DoubleArray elements;
        elements.reserve(value->size());
        for (size_t i = 0; i < value->size(); ++i) {
            double number = 0.0;
            if (!read_double((*value)[i], at(path, i), number, error)) return false;
            elements.push_back(number);
        }
        out = AttributeValue::from_doubles(std::move(elements));
        return true;
    }
    case AttributeType::StringArray: {
        if (!value->is_array()) return bad(error, path, "expected an array");
        AttributeValue::StringArray elements;
        elements.reserve(value->size());
        for (size_t i = 0; i < value->size(); ++i) {
            const json& element = (*value)[i];
            if (!element.is_string()) return bad(error, at(path, i), "expected a string");
            elements.push_back(element.get<std::string>());
        }
        out = AttributeValue::from_strings(std::move(elements));
        return true;
    }
    }
    return bad(error, at(where, kType), "is not an attribute type this build can read");
}

/// One (key, value) pair as the file carries it. The key travels by NAME.
using ValuePair = std::pair<AttributeKey, AttributeValue>;

[[nodiscard]] bool write_value_list(const AttributeStore& store,
                                    const std::vector<ValuePair>& values, const std::string& where,
                                    json& out, std::string& error) {
    out = json::array();
    for (size_t i = 0; i < values.size(); ++i) {
        json record = json::object();
        record[kKey] = std::string(store.key_name(values[i].first));
        if (!write_attribute_value(values[i].second, at(where, i), record, error)) return false;
        out.push_back(std::move(record));
    }
    return true;
}

/**
 * @brief Read one list of values straight into the store, with no undo history
 *
 * AttributeStore::load_value() is the audited hole in the command stack, and its
 * comment names this caller: recording a load as commands would fill the undo
 * stack with the act of opening a file.
 *
 * @param make_target Builds the write target for a key. Which rung a value lands
 *                    on -- Object, User, Layer or Default -- is the caller's
 *                    business, and it is the distinction the whole round trip
 *                    exists to preserve: writing a resolved value onto the
 *                    object would turn every inherited value into an owned one.
 */
template <typename MakeTarget>
[[nodiscard]] bool load_value_list(AttributeStore& store, const json& parent, const char* member,
                                   const std::string& where, MakeTarget make_target,
                                   std::string& error) {
    const json* list = nullptr;
    if (!require_member(parent, member, where, list, error)) return false;
    const std::string path = at(where, member);
    if (!list->is_array()) return bad(error, path, "expected an array");

    std::unordered_set<uint32_t> seen;
    for (size_t i = 0; i < list->size(); ++i) {
        const json& record = (*list)[i];
        const std::string record_path = at(path, i);
        if (!record.is_object()) return bad(error, record_path, "expected an object");

        std::string name;
        if (!read_string(record, kKey, record_path, name, error)) return false;
        const AttributeKey key = store.find_key(name);
        if (!key.valid()) {
            // The file disagrees with itself: a value names a key its own table
            // never declared. Interning it here would work, and would hide a
            // writer that forgot half its job.
            return bad(error, at(record_path, kKey),
                       "\"" + name + "\" is not in attributes.keys");
        }
        if (!seen.insert(key.index()).second) {
            // The second write would silently replace the first, so the file
            // would mean something other than what it says.
            return bad(error, at(record_path, kKey), "\"" + name + "\" appears twice in this list");
        }

        std::optional<AttributeValue> value;
        if (!read_attribute_value(record, record_path, value, error)) return false;
        if (!store.load_value(make_target(key), std::move(*value))) {
            return bad(error, record_path, "the store refused this value");
        }
    }
    return true;
}

// ============================================================================
// Layer nodes
// ============================================================================

[[nodiscard]] bool load_layer_node(const json& node, const std::string& where, LayerId parent,
                                   size_t depth, LayerLoadCommand& loader, LayerId& highest_id,
                                   std::string& error) {
    if (depth > kMaxDocumentLayerDepth) {
        // This function recurses, so a million-deep file would be a stack
        // overflow rather than a message. Checked before anything else is read,
        // because the depth is what makes the rest of the work happen.
        return bad(error, where,
                   "layer nesting is deeper than " + std::to_string(kMaxDocumentLayerDepth)
                       + " levels");
    }
    if (!node.is_object()) return bad(error, where, "expected an object");

    Layer layer;
    layer.parent = parent;

    uint64_t id = 0;
    if (!read_u64_member(node, kId, where, kMaxLayerId, id, error)) return false;
    if (id == kInvalidLayer) return bad(error, at(where, kId), "0 is not a layer id");
    layer.id = static_cast<LayerId>(id);
    highest_id = std::max(highest_id, layer.id);

    std::string kind_name;
    if (!read_string(node, kKind, where, kind_name, error)) return false;
    const std::optional<LayerKind> kind = layer_kind_from_name(kind_name);
    if (!kind.has_value()) {
        return bad(error, at(where, kKind), "\"" + kind_name + "\" is not a layer kind");
    }
    layer.kind = *kind;

    if (!read_string(node, kName, where, layer.name, error)) return false;
    if (!read_bool(node, kOwnVisible, where, layer.own_visible, error)) return false;
    if (!read_bool(node, kOwnLocked, where, layer.own_locked, error)) return false;

    const json* colour = nullptr;
    if (!require_member(node, kOwnColour, where, colour, error)) return false;
    if (colour->is_null()) {
        layer.own_colour.reset();  // Inherit -- not the default grey. See the writer.
    } else {
        glm::vec3 value{0.0f};
        if (!read_vec3(node, kOwnColour, where, value, error)) return false;
        layer.own_colour = value;
    }

    const json* transform = nullptr;
    if (!require_member(node, kOwnTransform, where, transform, error)) return false;
    const std::string transform_path = at(where, kOwnTransform);
    if (!transform->is_object()) return bad(error, transform_path, "expected an object");
    if (!read_vec3(*transform, kTranslation, transform_path, layer.own_transform.translation, error)
        || !read_vec3(*transform, kRotation, transform_path, layer.own_transform.rotation, error)
        || !read_vec3(*transform, kScale, transform_path, layer.own_transform.scale, error)) {
        return false;
    }

    const json* children = nullptr;
    if (!require_member(node, kChildren, where, children, error)) return false;
    const std::string children_path = at(where, kChildren);
    if (!children->is_array()) return bad(error, children_path, "expected an array");
    if (!children->empty() && layer.kind != LayerKind::Group) {
        return bad(error, children_path, "a layer of kind " + kind_name + " cannot hold children");
    }

    // Inserted with an EMPTY children list on purpose: each child appends itself
    // below, in file order, which is what reproduces sibling order. Copying the
    // list in as well would leave every child listed twice in its parent -- once
    // from here and once from its own insert.
    if (!loader.insert_layer(std::move(layer), kAppend)) {
        return bad(error, at(where, kId),
                   std::to_string(id)
                       + " could not be inserted; it is a duplicate id, or its parent cannot "
                         "hold children");
    }

    for (size_t i = 0; i < children->size(); ++i) {
        if (!load_layer_node((*children)[i], at(children_path, i), static_cast<LayerId>(id),
                             depth + 1, loader, highest_id, error)) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] bool write_layer(const LayerTree& tree, LayerId id, const std::string& where,
                               size_t depth, json& out, std::string& error) {
    if (depth > kMaxDocumentLayerDepth) {
        // The reader recurses over this nesting and refuses beyond the same
        // depth, so a tree too deep to load is refused at SAVE time, where the
        // user still has the document in front of them.
        return bad(error, where,
                   "layer nesting is deeper than " + std::to_string(kMaxDocumentLayerDepth)
                       + " levels");
    }

    const Layer* layer = tree.find(id);
    if (layer == nullptr) return bad(error, where, "layer " + std::to_string(id) + " is missing");

    out = json::object();
    out[kId] = layer->id;
    out[kKind] = layer_kind_name(layer->kind);
    out[kName] = layer->name;
    out[kOwnVisible] = layer->own_visible;
    out[kOwnLocked] = layer->own_locked;

    // null, not an omitted member and not the default grey. "No colour, so
    // inherit" is a state the panel shows differently from an explicit colour,
    // and writing the RESOLVED colour here would turn every inheriting layer
    // into one that has opted out of its group.
    if (layer->own_colour.has_value()) {
        const glm::vec3& colour = *layer->own_colour;
        json array = json::array();
        for (int channel = 0; channel < 3; ++channel) {
            json number;
            if (!write_double(static_cast<double>(colour[channel]),
                              at(at(where, kOwnColour), static_cast<size_t>(channel)), number,
                              error)) {
                return false;
            }
            array.push_back(std::move(number));
        }
        out[kOwnColour] = std::move(array);
    } else {
        out[kOwnColour] = nullptr;
    }

    // Always written, identity included. A1 stores the transform decomposed
    // precisely so a human can read "moved ten metres east" out of the file, and
    // an omitted-when-identity rule would make the reader guess which of two
    // meanings an absent member has.
    json transform = json::object();
    const LayerTransform& own = layer->own_transform;
    const char* const part_names[3] = {kTranslation, kRotation, kScale};
    const glm::dvec3* const parts[3] = {&own.translation, &own.rotation, &own.scale};
    const std::string transform_path = at(where, kOwnTransform);
    for (size_t part = 0; part < 3; ++part) {
        json array = json::array();
        const std::string part_path = at(transform_path, part_names[part]);
        for (int axis = 0; axis < 3; ++axis) {
            json number;
            if (!write_double((*parts[part])[axis], at(part_path, static_cast<size_t>(axis)),
                              number, error)) {
                return false;
            }
            array.push_back(std::move(number));
        }
        transform[part_names[part]] = std::move(array);
    }
    out[kOwnTransform] = std::move(transform);

    // Nested, so parentage and sibling order have exactly one spelling and
    // cannot contradict each other. `parent` is therefore not written at all.
    json children = json::array();
    const std::string children_path = at(where, kChildren);
    const std::vector<LayerId>& ids = tree.children(id);
    for (size_t i = 0; i < ids.size(); ++i) {
        json child;
        if (!write_layer(tree, ids[i], at(children_path, i), depth + 1, child, error)) return false;
        children.push_back(std::move(child));
    }
    out[kChildren] = std::move(children);
    return true;
}

} // namespace

// ============================================================================
// Document -- objects
// ============================================================================

AttributeObject Document::create_object(LayerId layer) {
    const AttributeObject handle = m_attributes.create_object();
    if (!handle.valid()) return handle;  // The store is out of slots and has said so.

    if (handle.index >= m_slots.size()) m_slots.resize(handle.index + 1u);
    ObjectSlot& slot = m_slots[handle.index];

    if (slot.alive || slot.generation != handle.generation) {
        // The store and this mirror disagree, which can only happen if something
        // called AttributeStore::create_object() or destroy_object() directly
        // instead of going through the document. The store is the authority on
        // generations, so its answer is adopted below -- but loudly, because the
        // alternative is an object table that saves a generation the store never
        // issued, and a loaded document whose handles alias.
        spdlog::error("Document: object slot {} was allocated outside the document (mirror "
                      "generation {}, store generation {}); object lifetime must go through "
                      "Document::create_object()",
                      handle.index, slot.generation, handle.generation);
    }

    slot.generation = handle.generation;
    slot.alive = true;
    slot.layer = layer;
    ++m_structural_revision;
    return handle;
}

bool Document::destroy_object(AttributeObject obj) {
    if (!m_attributes.is_valid(obj)) return false;

    if (obj.generation == 0xFFFFFFFFu) {
        // One more destroy wraps the generation to 0, at which point a handle
        // from the slot's first life validates against its next. The store
        // retires such a slot; this document refuses instead, because a retired
        // slot cannot be reproduced through the store's public API and would
        // leave a document that saves but does not load. Leaking one live record
        // beats that, and reaching this needs 2^32 recycles of one slot.
        spdlog::error("Document: refusing to destroy object slot {}; its generation would wrap",
                      obj.index);
        return false;
    }

    if (!m_attributes.destroy_object(obj)) return false;

    ObjectSlot& slot = m_slots[obj.index];
    slot.alive = false;
    // The generation the store will hand out the NEXT time this slot is taken.
    // Recording that, rather than the dead handle's own generation, is what stops
    // a loaded document re-issuing a handle the saved session already issued.
    slot.generation = obj.generation + 1u;
    slot.layer = kInvalidLayer;
    ++m_structural_revision;
    return true;
}

bool Document::set_object_layer(AttributeObject obj, LayerId layer) {
    if (!m_attributes.is_valid(obj)) return false;
    ObjectSlot& slot = m_slots[obj.index];
    if (slot.layer == layer) return true;
    slot.layer = layer;
    ++m_structural_revision;
    return true;
}

LayerId Document::object_layer(AttributeObject obj) const {
    if (!m_attributes.is_valid(obj)) return kInvalidLayer;
    return m_slots[obj.index].layer;
}

std::vector<AttributeObject> Document::objects() const {
    std::vector<AttributeObject> live;
    live.reserve(m_attributes.live_object_count());
    for (size_t i = 0; i < m_slots.size(); ++i) {
        if (!m_slots[i].alive) continue;
        live.push_back(AttributeObject{static_cast<uint32_t>(i), m_slots[i].generation});
    }
    return live;
}

void Document::adopt(Document& staging) {
    // First, and not last. Every Command on this stack holds a raw pointer into
    // the tree and the store that are about to be overwritten, so they have to be
    // destroyed while those are still the objects they point at.
    m_history.clear();

    m_layers = std::move(staging.m_layers);
    m_attributes = std::move(staging.m_attributes);
    m_slots = std::move(staging.m_slots);

    // The object set changed wholesale. Bumped before mark_saved(), so the
    // freshly loaded document reads as clean rather than as one edit behind.
    ++m_structural_revision;
    mark_saved();
}

// ============================================================================
// Writing
// ============================================================================

/// Renders a Document as `.stratum` JSON. A friend of Document because it reads
/// the slot mirror, which nothing outside the document has business seeing.
struct DocumentWriter {
    [[nodiscard]] static DocumentIoResult write(Document& doc, std::string& out);
};

DocumentIoResult DocumentWriter::write(Document& doc, std::string& out) {
    const AttributeStore& store = doc.m_attributes;
    const LayerTree& tree = doc.m_layers;
    std::string error;

    // ── Gather ──────────────────────────────────────────────────────────────
    //
    // Everything is collected before anything is written, so that a refusal (a
    // non-finite double) leaves `out` untouched, and so the key table can be
    // built from exactly what is going to be written and nothing else.

    std::vector<AttributeKey> used_keys;
    const auto note_keys = [&used_keys](const std::vector<ValuePair>& values) {
        for (const ValuePair& pair : values) used_keys.push_back(pair.first);
    };

    std::vector<ValuePair> defaults;
    for (const AttributeKey key : store.keys_on(AttributeObject{}, kNoLayer)) {
        // keys_on() with no object and no layer is the declared defaults alone.
        const AttributeQuery query = store.default_value(key);
        if (query.ok() && query.value != nullptr) defaults.emplace_back(key, *query.value);
    }
    note_keys(defaults);

    // Which layers can hold a value anything could ever read: the ones in the
    // tree, plus the ones objects point at. A value parked on any other LayerRef
    // is unreachable -- nothing can name it to resolve() -- and the store offers
    // no way to enumerate those in any case, so they are not written.
    std::vector<LayerId> layer_refs;
    for (const LayerId root : tree.roots()) {
        for (const LayerId id : tree.subtree_ids(root)) layer_refs.push_back(id);
    }
    for (const Document::ObjectSlot& slot : doc.m_slots) {
        if (slot.alive && slot.layer != kInvalidLayer) layer_refs.push_back(slot.layer);
    }
    std::sort(layer_refs.begin(), layer_refs.end());
    layer_refs.erase(std::unique(layer_refs.begin(), layer_refs.end()), layer_refs.end());

    std::vector<std::pair<LayerId, std::vector<ValuePair>>> layer_values;
    for (const LayerId id : layer_refs) {
        const auto ref = static_cast<LayerRef>(id);
        std::vector<ValuePair> values;
        for (const AttributeKey key : store.keys_on(AttributeObject{}, ref)) {
            // keys_on() folds the defaults in, so ask the layer rung itself
            // rather than assuming every key it returned belongs to the layer.
            const AttributeQuery query = store.layer_value(ref, key);
            if (query.ok() && query.value != nullptr) values.emplace_back(key, *query.value);
        }
        if (values.empty()) continue;
        note_keys(values);
        layer_values.emplace_back(id, std::move(values));
    }

    std::vector<ObjectSnapshot> snapshots(doc.m_slots.size());
    for (size_t i = 0; i < doc.m_slots.size(); ++i) {
        if (!doc.m_slots[i].alive) continue;
        snapshots[i] =
            store.snapshot(AttributeObject{static_cast<uint32_t>(i), doc.m_slots[i].generation});
        note_keys(snapshots[i].object_values);
        note_keys(snapshots[i].user_values);
    }

    // Sorted by key index, so the table is in the order the store interned them
    // and a save of a loaded document reproduces the same file byte for byte.
    std::sort(used_keys.begin(), used_keys.end(),
              [](AttributeKey a, AttributeKey b) { return a.index() < b.index(); });
    used_keys.erase(std::unique(used_keys.begin(), used_keys.end()), used_keys.end());

    // ── Build ───────────────────────────────────────────────────────────────

    json root = json::object();
    root[kFormat] = kDocumentFormatTag;
    root[kVersion] = kDocumentFormatVersion;

    // Reserved and empty in version 1. Geometry goes in sidecar files named here,
    // not base64'd into this one; see the file comment in document.hpp.
    root[kBlobs] = json::array();

    json layers = json::object();
    // Takes one layer id. See the note on Document::save_to_json() for why this
    // is the only way to read the counter and why recording the taken id -- not
    // the one after it -- is the right value.
    layers[kNextId] = LayerLoadCommand::take_next_id(doc.m_layers);

    json roots = json::array();
    const std::string roots_path = at(kLayers, kRoots);
    for (size_t i = 0; i < tree.roots().size(); ++i) {
        json node;
        if (!write_layer(tree, tree.roots()[i], at(roots_path, i), 0, node, error)) {
            return DocumentIoResult::failure(std::move(error));
        }
        roots.push_back(std::move(node));
    }
    layers[kRoots] = std::move(roots);
    root[kLayers] = std::move(layers);

    json attributes = json::object();

    json keys = json::array();
    for (const AttributeKey key : used_keys) keys.push_back(std::string(store.key_name(key)));
    attributes[kKeys] = std::move(keys);

    json defaults_json;
    if (!write_value_list(store, defaults, at(kAttributes, kDefaults), defaults_json, error)) {
        return DocumentIoResult::failure(std::move(error));
    }
    attributes[kDefaults] = std::move(defaults_json);

    json layer_values_json = json::array();
    for (size_t i = 0; i < layer_values.size(); ++i) {
        const std::string path = at(at(kAttributes, kLayerValues), i);
        json record = json::object();
        record[kLayer] = layer_values[i].first;
        json values;
        if (!write_value_list(store, layer_values[i].second, at(path, kValues), values, error)) {
            return DocumentIoResult::failure(std::move(error));
        }
        record[kValues] = std::move(values);
        layer_values_json.push_back(std::move(record));
    }
    attributes[kLayerValues] = std::move(layer_values_json);

    json objects = json::array();
    for (size_t i = 0; i < doc.m_slots.size(); ++i) {
        const Document::ObjectSlot& slot = doc.m_slots[i];
        const std::string path = at(at(kAttributes, kObjects), i);

        json record = json::object();
        // Written although it is also the array position: a reader that can
        // check the two against each other can say which entry a truncated or
        // reordered table went wrong at.
        record[kIndex] = i;
        record[kGeneration] = slot.generation;
        record[kAlive] = slot.alive;

        if (slot.alive) {
            // Written verbatim even when that layer has since been deleted. A1
            // never reuses an id, so a dangling reference can never come to mean
            // a different layer, and zeroing it would throw away information the
            // live session still has.
            record[kLayer] = slot.layer;

            json object_values;
            if (!write_value_list(store, snapshots[i].object_values, at(path, kObjectValues),
                                  object_values, error)) {
                return DocumentIoResult::failure(std::move(error));
            }
            record[kObjectValues] = std::move(object_values);

            json user_values;
            if (!write_value_list(store, snapshots[i].user_values, at(path, kUserValues),
                                  user_values, error)) {
                return DocumentIoResult::failure(std::move(error));
            }
            record[kUserValues] = std::move(user_values);
        }
        objects.push_back(std::move(record));
    }
    attributes[kObjects] = std::move(objects);
    root[kAttributes] = std::move(attributes);

    out = root.dump(kJsonIndent);
    return DocumentIoResult::success();
}

// ============================================================================
// Reading
// ============================================================================

/**
 * @brief Parses `.stratum` JSON into a staging Document
 *
 * The destination is not touched until the whole file has parsed and every
 * restored handle is in place, which is what makes a failed load a no-op rather
 * than half a scene.
 */
struct DocumentLoader {
    [[nodiscard]] static DocumentIoResult load(Document& doc, std::string_view text);

private:
    [[nodiscard]] static bool load_layers(const json& root, Document& staging, std::string& error);
    [[nodiscard]] static bool load_attributes(const json& root, Document& staging,
                                              std::string& error);
    [[nodiscard]] static bool restore_slots(Document& staging,
                                            const std::vector<Document::ObjectSlot>& slots,
                                            std::string& error);
};

bool DocumentLoader::load_layers(const json& root, Document& staging, std::string& error) {
    const json* layers = nullptr;
    if (!require_member(root, kLayers, {}, layers, error)) return false;
    if (!layers->is_object()) return bad(error, kLayers, "expected an object");

    uint64_t next_id = 0;
    if (!read_u64_member(*layers, kNextId, kLayers, 0xFFFFFFFFull, next_id, error)) return false;
    if (next_id == kInvalidLayer) {
        return bad(error, at(kLayers, kNextId),
                   "0 is not a layer id, so it cannot be the next one");
    }

    const json* roots = nullptr;
    if (!require_member(*layers, kRoots, kLayers, roots, error)) return false;
    const std::string roots_path = at(kLayers, kRoots);
    if (!roots->is_array()) return bad(error, roots_path, "expected an array");

    LayerLoadCommand loader(staging.m_layers);
    LayerId highest_id = kInvalidLayer;
    for (size_t i = 0; i < roots->size(); ++i) {
        if (!load_layer_node((*roots)[i], at(roots_path, i), kInvalidLayer, 0, loader, highest_id,
                             error)) {
            return false;
        }
    }

    if (next_id <= highest_id) {
        // The file contradicts itself: it holds a layer whose id the counter says
        // has not been issued yet. Loading it would let the tree hand that id out
        // a second time.
        return bad(error, at(kLayers, kNextId),
                   std::to_string(next_id) + " is not above the highest layer id in the file ("
                       + std::to_string(highest_id) + ")");
    }

    // Insert-then-detach rather than a loop of reserve_id(): a file may name a
    // counter billions above the highest surviving layer, and LayerTree::insert()
    // advances the counter to id + 1 in a single step. The placeholder is removed
    // immediately, and detach() never lowers the counter, so what remains is a
    // tree holding exactly the file's layers with exactly the file's next id.
    const auto next = static_cast<LayerId>(next_id);
    if (next - 1u > highest_id) {
        Layer placeholder;
        placeholder.id = next - 1u;
        placeholder.kind = LayerKind::Group;
        placeholder.parent = kInvalidLayer;
        if (!loader.insert_layer(std::move(placeholder), kAppend)) {
            return bad(error, at(kLayers, kNextId), "could not restore the layer id counter");
        }
        loader.detach_subtree(next - 1u, nullptr, nullptr);
    }
    return true;
}

bool DocumentLoader::restore_slots(Document& staging,
                                   const std::vector<Document::ObjectSlot>& slots,
                                   std::string& error) {
    AttributeStore& store = staging.m_attributes;
    staging.m_slots.assign(slots.size(), Document::ObjectSlot{});

    for (size_t i = 0; i < slots.size(); ++i) {
        const Document::ObjectSlot& want = slots[i];

        // Reaching generation G means having been recycled G times, and the
        // store's public API has exactly one way to say that: destroy, then
        // create. The alternative was a private restore entry point on
        // AttributeStore, which lost twice over -- A3's header belongs to another
        // change, and a second way to mint a handle is a second way to mint a
        // wrong one.
        AttributeObject handle = store.create_object();
        if (handle.index != i) {
            return bad(error, at(at(kAttributes, kObjects), i),
                       "the store allocated slot " + std::to_string(handle.index) + ", not "
                           + std::to_string(i));
        }

        // A dead slot's recorded generation is what the NEXT create() will hand
        // out, so it is reached by replaying to one below and destroying once, in
        // the second pass.
        const uint32_t alive_generation = want.alive ? want.generation : want.generation - 1u;
        for (uint32_t step = 0; step < alive_generation; ++step) {
            (void)store.destroy_object(handle);
            handle = store.create_object();
            if (handle.index != i) {
                return bad(error, at(at(kAttributes, kObjects), i),
                           "the store moved the slot while its generation was being restored");
            }
        }

        staging.m_slots[i].generation = handle.generation;
        staging.m_slots[i].alive = true;
        staging.m_slots[i].layer = want.layer;
    }

    // Destroyed last, and in one pass, so that every create() above takes a fresh
    // slot off the end rather than an earlier slot off the free list.
    for (size_t i = 0; i < slots.size(); ++i) {
        if (slots[i].alive) continue;
        const AttributeObject handle{static_cast<uint32_t>(i), slots[i].generation - 1u};
        if (!store.destroy_object(handle)) {
            return bad(error, at(at(kAttributes, kObjects), i), "could not retire the slot");
        }
        staging.m_slots[i].alive = false;
        staging.m_slots[i].generation = slots[i].generation;
        staging.m_slots[i].layer = kInvalidLayer;
    }
    return true;
}

bool DocumentLoader::load_attributes(const json& root, Document& staging, std::string& error) {
    const json* attributes = nullptr;
    if (!require_member(root, kAttributes, {}, attributes, error)) return false;
    if (!attributes->is_object()) return bad(error, kAttributes, "expected an object");

    AttributeStore& store = staging.m_attributes;

    // ── Keys ────────────────────────────────────────────────────────────────
    //
    // Interned in file order, so index 0 in the file is index 0 in the store and
    // a re-save writes the same table. Values name keys by STRING, so nothing in
    // the file depends on that: it is what makes the round trip byte-stable, not
    // what makes it correct.
    const json* keys = nullptr;
    if (!require_member(*attributes, kKeys, kAttributes, keys, error)) return false;
    const std::string keys_path = at(kAttributes, kKeys);
    if (!keys->is_array()) return bad(error, keys_path, "expected an array");

    for (size_t i = 0; i < keys->size(); ++i) {
        const json& entry = (*keys)[i];
        if (!entry.is_string()) return bad(error, at(keys_path, i), "expected a string");
        const std::string name = entry.get<std::string>();
        if (store.find_key(name).valid()) {
            // Two entries interning to one key would silently shift every key
            // after them, so the table would no longer mean what it says.
            return bad(error, at(keys_path, i), "\"" + name + "\" appears twice in the key table");
        }
        const AttributeKey key = store.intern(name);
        if (key.index() != i) {
            return bad(error, at(keys_path, i),
                       "interned as key " + std::to_string(key.index()) + ", not "
                           + std::to_string(i));
        }
    }

    // ── Object slots ────────────────────────────────────────────────────────

    const json* objects = nullptr;
    if (!require_member(*attributes, kObjects, kAttributes, objects, error)) return false;
    const std::string objects_path = at(kAttributes, kObjects);
    if (!objects->is_array()) return bad(error, objects_path, "expected an array");
    if (objects->size() >= static_cast<size_t>(AttributeObject::kInvalidIndex)) {
        return bad(error, objects_path, "names more object slots than a handle can address");
    }

    std::vector<Document::ObjectSlot> slots;
    slots.reserve(objects->size());
    size_t replay_steps = 0;

    for (size_t i = 0; i < objects->size(); ++i) {
        const json& record = (*objects)[i];
        const std::string path = at(objects_path, i);
        if (!record.is_object()) return bad(error, path, "expected an object");

        uint64_t index = 0;
        if (!read_u64_member(record, kIndex, path, objects->size(), index, error)) return false;
        if (index != i) {
            return bad(error, at(path, kIndex),
                       "says slot " + std::to_string(index) + ", but it is entry "
                           + std::to_string(i) + "; the slot table is the array order");
        }

        uint64_t generation = 0;
        if (!read_u64_member(record, kGeneration, path, kMaxRestorableGeneration, generation,
                             error)) {
            // The cap is the point: a generation is restored by replaying that
            // many destroy/create pairs, so an unchecked number here is an
            // arbitrarily long stall written in ten characters.
            return false;
        }

        Document::ObjectSlot slot;
        slot.generation = static_cast<uint32_t>(generation);
        if (!read_bool(record, kAlive, path, slot.alive, error)) return false;

        if (slot.alive) {
            uint64_t layer = 0;
            if (!read_u64_member(record, kLayer, path, kMaxLayerId, layer, error)) return false;
            slot.layer = static_cast<LayerId>(layer);
        } else {
            if (generation == 0) {
                // A slot is only dead after a destroy, and a destroy bumps the
                // generation, so dead-at-0 describes a slot the store retired
                // after 2^32 recycles. That state cannot be reproduced through
                // the public API, and pretending otherwise would put the loaded
                // slot one generation below where the saved one was.
                return bad(error, at(path, kGeneration),
                           "is 0 on a slot that is not alive, which no session can produce");
            }
            // A dead slot owns nothing. Values on one would be values this
            // document could never read back, so a file carrying them means
            // something other than what it says.
            for (const char* member : {kLayer, kObjectValues, kUserValues}) {
                if (member_of(record, member) != nullptr) {
                    return bad(error, at(path, member), "is set on a slot that is not alive");
                }
            }
        }

        replay_steps += static_cast<size_t>(generation);
        if (replay_steps > kMaxGenerationReplaySteps) {
            return bad(error, objects_path,
                       "would need more than " + std::to_string(kMaxGenerationReplaySteps)
                           + " steps to restore its object generations");
        }
        slots.push_back(slot);
    }

    if (!restore_slots(staging, slots, error)) return false;

    // ── Object values ───────────────────────────────────────────────────────
    //
    // After the slots, never during: destroy_object() clears a record's values,
    // so anything written before the replay finished would be thrown away.
    for (size_t i = 0; i < objects->size(); ++i) {
        if (!staging.m_slots[i].alive) continue;
        const json& record = (*objects)[i];
        const std::string path = at(objects_path, i);
        const AttributeObject handle{static_cast<uint32_t>(i), staging.m_slots[i].generation};

        // The two rungs are kept apart on purpose: User outranks Object so that a
        // human's edit survives the next rule run, and folding them together on
        // load would destroy that the first time the document was reopened.
        if (!load_value_list(
                store, record, kObjectValues, path,
                [handle](AttributeKey key) { return AttributeTarget::object(handle, key); },
                error)) {
            return false;
        }
        if (!load_value_list(
                store, record, kUserValues, path,
                [handle](AttributeKey key) { return AttributeTarget::user(handle, key); }, error)) {
            return false;
        }
    }

    // ── Layer values ────────────────────────────────────────────────────────

    const json* layer_values = nullptr;
    if (!require_member(*attributes, kLayerValues, kAttributes, layer_values, error)) return false;
    const std::string layer_values_path = at(kAttributes, kLayerValues);
    if (!layer_values->is_array()) return bad(error, layer_values_path, "expected an array");

    std::unordered_set<LayerId> seen_layers;
    for (size_t i = 0; i < layer_values->size(); ++i) {
        const json& record = (*layer_values)[i];
        const std::string path = at(layer_values_path, i);
        if (!record.is_object()) return bad(error, path, "expected an object");

        uint64_t layer = 0;
        if (!read_u64_member(record, kLayer, path, kMaxLayerId, layer, error)) return false;
        if (layer == kInvalidLayer) {
            // kNoLayer is a reserved handle in A3 and a write to it is refused,
            // precisely so that unparented objects do not all share one bucket.
            return bad(error, at(path, kLayer), "0 names no layer, so it cannot hold values");
        }
        if (!seen_layers.insert(static_cast<LayerId>(layer)).second) {
            return bad(error, at(path, kLayer),
                       "layer " + std::to_string(layer) + " appears twice");
        }

        const auto ref = static_cast<LayerRef>(layer);
        if (!load_value_list(
                store, record, kValues, path,
                [ref](AttributeKey key) { return AttributeTarget::layer(ref, key); }, error)) {
            return false;
        }
    }

    // ── Declared defaults ───────────────────────────────────────────────────

    return load_value_list(
        store, *attributes, kDefaults, kAttributes,
        [](AttributeKey key) { return AttributeTarget::schema_default(key); }, error);
}

DocumentIoResult DocumentLoader::load(Document& doc, std::string_view text) {
    json root;
    try {
        root = json::parse(text.begin(), text.end());
    } catch (const json::parse_error& e) {
        // Covers malformed and truncated input alike: nlohmann reports the byte
        // it gave up at, which is the one thing a user can act on.
        return DocumentIoResult::failure(std::string("not valid JSON: ") + e.what());
    } catch (const std::bad_alloc&) {
        return DocumentIoResult::failure("ran out of memory parsing the document");
    }

    if (!root.is_object()) {
        return DocumentIoResult::failure("the document is not a JSON object");
    }

    std::string error;
    std::string format;
    if (!read_string(root, kFormat, {}, format, error)) return DocumentIoResult::failure(error);
    if (format != kDocumentFormatTag) {
        return DocumentIoResult::failure("\"" + format + "\" is not a Stratum document; expected \""
                                         + kDocumentFormatTag + "\"");
    }

    uint64_t version = 0;
    if (!read_u64_member(root, kVersion, {}, 0xFFFFFFFFull, version, error)) {
        return DocumentIoResult::failure(error);
    }
    if (version == 0) return DocumentIoResult::failure("version 0 is not a format version");
    if (version > kDocumentFormatVersion) {
        // Refused, not attempted. A newer writer may have changed what an
        // existing member means, and a hopeful read of it produces a scene that
        // looks plausible and is wrong.
        return DocumentIoResult::failure(
            "this document is format version " + std::to_string(version)
            + ", and this build of Stratum reads up to " + std::to_string(kDocumentFormatVersion)
            + "; it was written by a newer version");
    }

    const json* blobs = nullptr;
    if (!require_member(root, kBlobs, {}, blobs, error)) return DocumentIoResult::failure(error);
    if (!blobs->is_array()) return DocumentIoResult::failure("blobs: expected an array");
    if (!blobs->empty()) {
        // Refused rather than ignored. A document that says it has geometry
        // beside it, opened by a build that drops the reference, is data loss the
        // user finds out about later.
        return DocumentIoResult::failure("this document references " + std::to_string(blobs->size())
                                         + " binary blob(s), which this build cannot load");
    }

    // Unknown top-level members are ignored, deliberately: "selection" and
    // "georeference" are reserved for A5 and A6, and a build that writes one
    // should still be readable here. document.hpp says which of the two may be
    // added without a version bump and why.

    // Built into a staging document, so that a failure anywhere below leaves the
    // destination exactly as it was -- history, handles and all.
    Document staging;
    if (!load_layers(root, staging, error)) return DocumentIoResult::failure(error);
    if (!load_attributes(root, staging, error)) return DocumentIoResult::failure(error);

    doc.adopt(staging);
    return DocumentIoResult::success();
}

// ============================================================================
// Document -- serialisation entry points
// ============================================================================

DocumentIoResult Document::save_to_json(std::string& out) {
    return DocumentWriter::write(*this, out);
}

DocumentIoResult Document::load_from_json(std::string_view text) {
    return DocumentLoader::load(*this, text);
}

DocumentIoResult Document::save_to_file(const std::filesystem::path& path) {
    std::string text;
    if (DocumentIoResult rendered = save_to_json(text); !rendered) return rendered;

    // Written beside the target and renamed over it. A write straight into the
    // destination that fails or is interrupted half way leaves a truncated
    // `.stratum` where the user's only copy of their work used to be, and rename
    // is the one step the filesystem will not do by halves.
    std::filesystem::path temporary = path;
    temporary += ".tmp";

    {
        std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
        if (!file) {
            return DocumentIoResult::failure("cannot open " + temporary.string() + " for writing");
        }
        file.write(text.data(), static_cast<std::streamsize>(text.size()));
        file.close();
        if (!file) {
            std::error_code ignored;
            std::filesystem::remove(temporary, ignored);
            return DocumentIoResult::failure("writing " + temporary.string() + " failed");
        }
    }

    std::error_code ec;
    std::filesystem::rename(temporary, path, ec);
    if (ec) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return DocumentIoResult::failure("cannot move " + temporary.string() + " onto "
                                         + path.string() + ": " + ec.message());
    }

    mark_saved();
    return DocumentIoResult::success();
}

DocumentIoResult Document::load_from_file(const std::filesystem::path& path) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec) || ec) {
        return DocumentIoResult::failure(path.string() + " is not a file");
    }

    // Checked before the file is read, not after: the point is to bound the
    // allocation a hostile path can cause, and reading it first has already paid
    // the cost.
    const uintmax_t size = std::filesystem::file_size(path, ec);
    if (ec) {
        return DocumentIoResult::failure("cannot size " + path.string() + ": " + ec.message());
    }
    if (size > kMaxDocumentFileBytes) {
        return DocumentIoResult::failure(path.string() + " is " + std::to_string(size)
                                         + " bytes, above the limit of "
                                         + std::to_string(kMaxDocumentFileBytes));
    }

    std::ifstream file(path, std::ios::binary);
    if (!file) return DocumentIoResult::failure("cannot open " + path.string() + " for reading");

    std::string text(static_cast<size_t>(size), '\0');
    file.read(text.data(), static_cast<std::streamsize>(size));
    // Short reads are not an error here: the file may have been truncated since
    // it was sized, and the parser is the thing that decides whether what arrived
    // is a document.
    text.resize(static_cast<size_t>(file.gcount()));

    return load_from_json(text);
}

} // namespace stratum::scene
