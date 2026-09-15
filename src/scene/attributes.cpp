// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "scene/attributes.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <sstream>
#include <type_traits>
#include <utility>

namespace stratum::scene {

namespace {

/**
 * @brief One double, rendered for a human
 *
 * Default stream precision, so 9.0 reads as "9" rather than "9.000000". This is
 * for the inspector and for log lines only; see the warning on
 * AttributeValue::to_display_string().
 */
std::string format_double(double value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

/// "Set layer height", "Clear default zoning", "Set height".
std::string command_label(const AttributeStore& store, const AttributeTarget& target,
                          std::string_view verb) {
    std::string label(verb);
    label += ' ';
    // User and Object are not distinguished. The undo menu answers "what did I
    // just do", and "Set height" is that answer for both; which slot it landed
    // in is the inspector's job to show, not the menu's.
    if (target.scope == AttributeSource::Layer) {
        label += "layer ";
    } else if (target.scope == AttributeSource::Default) {
        label += "default ";
    }
    label += store.key_name(target.key);
    return label;
}

} // namespace

// ============================================================================
// Names
// ============================================================================

std::string_view attribute_type_name(AttributeType type) {
    switch (type) {
    case AttributeType::Bool:        return "bool";
    case AttributeType::Double:      return "double";
    case AttributeType::String:      return "string";
    case AttributeType::BoolArray:   return "bool[]";
    case AttributeType::DoubleArray: return "double[]";
    case AttributeType::StringArray: return "string[]";
    }
    return "?";
}

std::string_view attribute_source_name(AttributeSource source) {
    switch (source) {
    case AttributeSource::None:    return "None";
    case AttributeSource::Default: return "Default";
    case AttributeSource::Layer:   return "Layer";
    case AttributeSource::Object:  return "Object";
    case AttributeSource::User:    return "User";
    }
    return "?";
}

std::string_view attribute_status_name(AttributeStatus status) {
    switch (status) {
    case AttributeStatus::Ok:           return "Ok";
    case AttributeStatus::Missing:      return "Missing";
    case AttributeStatus::TypeMismatch: return "TypeMismatch";
    }
    return "?";
}

// ============================================================================
// AttributeValue
// ============================================================================

AttributeValue AttributeValue::from_bool(bool value) { return AttributeValue(Storage(value)); }

AttributeValue AttributeValue::from_double(double value) { return AttributeValue(Storage(value)); }

AttributeValue AttributeValue::from_string(std::string value) {
    return AttributeValue(Storage(std::move(value)));
}

AttributeValue AttributeValue::from_bools(BoolArray value) {
    return AttributeValue(Storage(std::move(value)));
}

AttributeValue AttributeValue::from_doubles(DoubleArray value) {
    return AttributeValue(Storage(std::move(value)));
}

AttributeValue AttributeValue::from_strings(StringArray value) {
    return AttributeValue(Storage(std::move(value)));
}

AttributeValue AttributeValue::zero_of(AttributeType type) {
    switch (type) {
    case AttributeType::Bool:        return from_bool(false);
    case AttributeType::Double:      return from_double(0.0);
    case AttributeType::String:      return from_string(std::string{});
    case AttributeType::BoolArray:   return from_bools(BoolArray{});
    case AttributeType::DoubleArray: return from_doubles(DoubleArray{});
    case AttributeType::StringArray: return from_strings(StringArray{});
    }
    // Unreachable for any declared enumerator. A bool is the cheapest thing to
    // hand back and it is still TYPED, so a caller that got here reads a type
    // mismatch rather than a plausible number.
    return from_bool(false);
}

AttributeType AttributeValue::type() const {
    // The type tag is the variant's own index rather than a second field that
    // could disagree with it. That only holds while the two orders match, so
    // they are pinned here.
    static_assert(std::variant_size_v<Storage> == 6u,
                  "AttributeType and AttributeValue::Storage must list the same types");
    static_assert(std::is_same_v<std::variant_alternative_t<0, Storage>, bool>);
    static_assert(std::is_same_v<std::variant_alternative_t<1, Storage>, double>);
    static_assert(std::is_same_v<std::variant_alternative_t<2, Storage>, std::string>);
    static_assert(std::is_same_v<std::variant_alternative_t<3, Storage>, BoolArray>);
    static_assert(std::is_same_v<std::variant_alternative_t<4, Storage>, DoubleArray>);
    static_assert(std::is_same_v<std::variant_alternative_t<5, Storage>, StringArray>);

    return static_cast<AttributeType>(m_data.index());
}

bool AttributeValue::is_array() const {
    const AttributeType tag = type();
    return tag == AttributeType::BoolArray || tag == AttributeType::DoubleArray
           || tag == AttributeType::StringArray;
}

const bool* AttributeValue::as_bool() const { return std::get_if<bool>(&m_data); }
const double* AttributeValue::as_double() const { return std::get_if<double>(&m_data); }
const std::string* AttributeValue::as_string() const { return std::get_if<std::string>(&m_data); }

const AttributeValue::BoolArray* AttributeValue::as_bool_array() const {
    return std::get_if<BoolArray>(&m_data);
}

const AttributeValue::DoubleArray* AttributeValue::as_double_array() const {
    return std::get_if<DoubleArray>(&m_data);
}

const AttributeValue::StringArray* AttributeValue::as_string_array() const {
    return std::get_if<StringArray>(&m_data);
}

size_t AttributeValue::size() const {
    switch (type()) {
    case AttributeType::Bool:
    case AttributeType::Double:
    case AttributeType::String:
        return 1u;
    case AttributeType::BoolArray:   return as_bool_array()->size();
    case AttributeType::DoubleArray: return as_double_array()->size();
    case AttributeType::StringArray: return as_string_array()->size();
    }
    return 0u;
}

size_t AttributeValue::heap_bytes() const {
    switch (type()) {
    case AttributeType::Bool:
    case AttributeType::Double:
        return 0u;
    case AttributeType::String:
        return as_string()->capacity();
    case AttributeType::BoolArray:
        // Bit-packed by the specialisation, so bytes and not elements.
        return (as_bool_array()->size() + 7u) / 8u;
    case AttributeType::DoubleArray:
        return as_double_array()->capacity() * sizeof(double);
    case AttributeType::StringArray: {
        size_t total = as_string_array()->capacity() * sizeof(std::string);
        for (const std::string& text : *as_string_array()) {
            total += text.capacity();
        }
        return total;
    }
    }
    return 0u;
}

std::string AttributeValue::to_display_string() const {
    switch (type()) {
    case AttributeType::Bool:
        return *as_bool() ? "true" : "false";
    case AttributeType::Double:
        return format_double(*as_double());
    case AttributeType::String:
        // Quoted, so the string "true" cannot be mistaken for the bool.
        return "\"" + *as_string() + "\"";
    case AttributeType::BoolArray: {
        std::string out = "[";
        const BoolArray& values = *as_bool_array();
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) out += ", ";
            out += values[i] ? "true" : "false";
        }
        return out + "]";
    }
    case AttributeType::DoubleArray: {
        std::string out = "[";
        const DoubleArray& values = *as_double_array();
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) out += ", ";
            out += format_double(values[i]);
        }
        return out + "]";
    }
    case AttributeType::StringArray: {
        std::string out = "[";
        const StringArray& values = *as_string_array();
        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) out += ", ";
            out += "\"" + values[i] + "\"";
        }
        return out + "]";
    }
    }
    return "<unknown>";
}

// ============================================================================
// AttributeTarget
// ============================================================================

AttributeTarget AttributeTarget::object(AttributeObject obj, AttributeKey key) {
    AttributeTarget target;
    target.scope = AttributeSource::Object;
    target.object_id = obj;
    target.key = key;
    return target;
}

AttributeTarget AttributeTarget::user(AttributeObject obj, AttributeKey key) {
    AttributeTarget target;
    target.scope = AttributeSource::User;
    target.object_id = obj;
    target.key = key;
    return target;
}

AttributeTarget AttributeTarget::layer(LayerRef layer, AttributeKey key) {
    AttributeTarget target;
    target.scope = AttributeSource::Layer;
    target.layer_id = layer;
    target.key = key;
    return target;
}

AttributeTarget AttributeTarget::schema_default(AttributeKey key) {
    AttributeTarget target;
    target.scope = AttributeSource::Default;
    target.key = key;
    return target;
}

// ============================================================================
// ObjectSnapshot
// ============================================================================

size_t ObjectSnapshot::footprint() const {
    size_t total = sizeof(ObjectSnapshot);
    for (const auto& entry : object_values) {
        total += sizeof(entry.first) + entry.second.footprint();
    }
    for (const auto& entry : user_values) {
        total += sizeof(entry.first) + entry.second.footprint();
    }
    return total;
}

// ============================================================================
// AttributeStore -- keys
// ============================================================================

AttributeKey AttributeStore::intern(std::string_view name) {
    if (name.empty()) {
        // An empty name is always a caller bug -- an OSM tag with no key, or an
        // uninitialised string -- and interning it would give every such bug the
        // same key and let them read each other's values.
        spdlog::warn("AttributeStore: refusing to intern an empty attribute name");
        return AttributeKey{};
    }

    if (const auto it = m_key_index.find(name); it != m_key_index.end()) {
        return AttributeKey{it->second};
    }

    const size_t next = m_key_names.size();
    if (next >= static_cast<size_t>(AttributeKey::kInvalidIndex)) {
        spdlog::error("AttributeStore: the intern table is full; \"{}\" has no key", name);
        return AttributeKey{};
    }

    const uint32_t index = static_cast<uint32_t>(next);
    m_key_names.emplace_back(name);
    m_key_index.emplace(m_key_names.back(), index);
    return AttributeKey{index};
}

AttributeKey AttributeStore::find_key(std::string_view name) const {
    const auto it = m_key_index.find(name);
    return it == m_key_index.end() ? AttributeKey{} : AttributeKey{it->second};
}

std::string_view AttributeStore::key_name(AttributeKey key) const {
    if (!key.valid() || key.index() >= m_key_names.size()) return {};
    return m_key_names[key.index()];
}

// ============================================================================
// AttributeStore -- object lifetime
// ============================================================================

AttributeObject AttributeStore::create_object() {
    AttributeObject handle;

    if (!m_free_objects.empty()) {
        handle.index = m_free_objects.back();
        m_free_objects.pop_back();
        ObjectRecord& rec = m_objects[handle.index];
        rec.alive = true;
        handle.generation = rec.generation;
    } else {
        if (m_objects.size() >= static_cast<size_t>(AttributeObject::kInvalidIndex)) {
            // Minting index 0xFFFFFFFF would produce a handle that reads as
            // invalid while naming a live record, which is worse than refusing.
            spdlog::error("AttributeStore: out of object slots");
            return AttributeObject{};
        }
        handle.index = static_cast<uint32_t>(m_objects.size());
        handle.generation = 0;
        ObjectRecord& rec = m_objects.emplace_back();
        rec.alive = true;
        rec.generation = 0;
    }

    ++m_live_objects;
    return handle;
}

bool AttributeStore::destroy_object(AttributeObject obj) {
    ObjectRecord* rec = record(obj);
    if (rec == nullptr) return false;

    rec->object_values.clear();
    rec->user_values.clear();
    rec->alive = false;
    ++rec->generation;

    // A wrapped generation would let a handle from the slot's first life
    // validate against its 2^32-th, which is the exact aliasing this scheme
    // exists to prevent. Retire the slot instead: one leaked record beats one
    // silently wrong object.
    if (rec->generation != 0u) {
        m_free_objects.push_back(obj.index);
    } else {
        spdlog::warn("AttributeStore: object slot {} has exhausted its generations; retiring it",
                     obj.index);
    }

    --m_live_objects;
    return true;
}

const AttributeStore::ObjectRecord* AttributeStore::record(AttributeObject obj) const {
    if (!obj.valid() || obj.index >= m_objects.size()) return nullptr;
    const ObjectRecord& rec = m_objects[obj.index];
    if (!rec.alive || rec.generation != obj.generation) return nullptr;
    return &rec;
}

AttributeStore::ObjectRecord* AttributeStore::record(AttributeObject obj) {
    if (!obj.valid() || obj.index >= m_objects.size()) return nullptr;
    ObjectRecord& rec = m_objects[obj.index];
    if (!rec.alive || rec.generation != obj.generation) return nullptr;
    return &rec;
}

// ============================================================================
// AttributeStore -- slots
// ============================================================================

const AttributeValue* AttributeStore::find_value(const KeyValueMap& map, AttributeKey key) {
    if (!key.valid()) return nullptr;
    const auto it = map.find(key.index());
    return it == map.end() ? nullptr : &it->second;
}

bool AttributeStore::target_valid(const AttributeTarget& target) const {
    if (!target.key.valid() || target.key.index() >= m_key_names.size()) return false;

    switch (target.scope) {
    case AttributeSource::Object:
    case AttributeSource::User:
        return record(target.object_id) != nullptr;
    case AttributeSource::Layer:
        return target.layer_id != kNoLayer;
    case AttributeSource::Default:
        return true;
    case AttributeSource::None:
        return false;
    }
    return false;
}

const AttributeStore::KeyValueMap* AttributeStore::slot_map(const AttributeTarget& target) const {
    switch (target.scope) {
    case AttributeSource::User: {
        const ObjectRecord* rec = record(target.object_id);
        return rec == nullptr ? nullptr : &rec->user_values;
    }
    case AttributeSource::Object: {
        const ObjectRecord* rec = record(target.object_id);
        return rec == nullptr ? nullptr : &rec->object_values;
    }
    case AttributeSource::Layer: {
        if (target.layer_id == kNoLayer) return nullptr;
        const auto it = m_layers.find(target.layer_id);
        return it == m_layers.end() ? nullptr : &it->second;
    }
    case AttributeSource::Default:
        return &m_defaults;
    case AttributeSource::None:
        return nullptr;
    }
    return nullptr;
}

AttributeStore::KeyValueMap* AttributeStore::slot_map_for_write(const AttributeTarget& target) {
    if (!target_valid(target)) return nullptr;

    switch (target.scope) {
    case AttributeSource::User: {
        ObjectRecord* rec = record(target.object_id);
        return rec == nullptr ? nullptr : &rec->user_values;
    }
    case AttributeSource::Object: {
        ObjectRecord* rec = record(target.object_id);
        return rec == nullptr ? nullptr : &rec->object_values;
    }
    case AttributeSource::Layer:
        // Created on first write. Layers are not registered here -- this file
        // does not know what a layer is -- so the first value a layer is given
        // is what brings its record into existence.
        return &m_layers[target.layer_id];
    case AttributeSource::Default:
        return &m_defaults;
    case AttributeSource::None:
        return nullptr;
    }
    return nullptr;
}

bool AttributeStore::write(const AttributeTarget& target, AttributeValue value,
                           std::optional<AttributeValue>* out_previous) {
    KeyValueMap* map = slot_map_for_write(target);
    if (map == nullptr) return false;

    const auto it = map->find(target.key.index());
    if (it != map->end()) {
        if (out_previous != nullptr) *out_previous = it->second;
        it->second = std::move(value);
    } else {
        if (out_previous != nullptr) out_previous->reset();
        map->emplace(target.key.index(), std::move(value));
    }
    return true;
}

bool AttributeStore::erase(const AttributeTarget& target,
                           std::optional<AttributeValue>* out_previous) {
    if (!target_valid(target)) return false;

    // const_cast over a second write-side lookup: *this is non-const, and the
    // write-side lookup CREATES a layer record, which would leave an empty one
    // behind every time a user cleared an attribute on a layer that had none.
    KeyValueMap* map = const_cast<KeyValueMap*>(slot_map(target));
    if (map == nullptr) return false;

    const auto it = map->find(target.key.index());
    if (it == map->end()) return false;

    if (out_previous != nullptr) *out_previous = std::move(it->second);
    map->erase(it);
    return true;
}

bool AttributeStore::load_value(const AttributeTarget& target, AttributeValue value) {
    return write(target, std::move(value), nullptr);
}

// ============================================================================
// AttributeStore -- reads
// ============================================================================

AttributeQuery AttributeStore::resolve(AttributeObject obj, AttributeKey key,
                                       LayerRef layer) const {
    AttributeQuery query;
    if (!key.valid()) return query;

    if (obj.valid()) {
        const ObjectRecord* rec = record(obj);
        if (rec == nullptr) {
            // A stale handle is not the same as "no object". Falling through to
            // the layer would answer a question about an object that is gone,
            // and the caller would never learn its handle had expired.
            spdlog::warn("AttributeStore: resolve() on a stale object handle {}:{} for \"{}\"",
                         obj.index, obj.generation, key_name(key));
            return query;
        }

        if (const AttributeValue* value = find_value(rec->user_values, key); value != nullptr) {
            query.status = AttributeStatus::Ok;
            query.source = AttributeSource::User;
            query.value = value;
            return query;
        }
        if (const AttributeValue* value = find_value(rec->object_values, key); value != nullptr) {
            query.status = AttributeStatus::Ok;
            query.source = AttributeSource::Object;
            query.value = value;
            return query;
        }
    }

    if (layer != kNoLayer) {
        if (const auto it = m_layers.find(layer); it != m_layers.end()) {
            if (const AttributeValue* value = find_value(it->second, key); value != nullptr) {
                query.status = AttributeStatus::Ok;
                query.source = AttributeSource::Layer;
                query.layer = layer;
                query.value = value;
                return query;
            }
        }
    }

    if (const AttributeValue* value = find_value(m_defaults, key); value != nullptr) {
        query.status = AttributeStatus::Ok;
        query.source = AttributeSource::Default;
        query.value = value;
        return query;
    }

    return query;
}

AttributeQuery AttributeStore::resolve_as(AttributeObject obj, AttributeKey key, LayerRef layer,
                                          AttributeType wanted) const {
    AttributeQuery query = resolve(obj, key, layer);
    if (query.status == AttributeStatus::Ok && !query.value->is(wanted)) {
        // The value and the source stay put. An inspector reporting "height is a
        // string, not a double" needs both, and blanking them would leave the
        // caller with nothing to say beyond "no".
        query.status = AttributeStatus::TypeMismatch;
    }
    return query;
}

AttributeQuery AttributeStore::peek(const AttributeTarget& target) const {
    AttributeQuery query;
    if (target.scope == AttributeSource::None || !target.key.valid()) return query;

    const KeyValueMap* map = slot_map(target);
    if (map == nullptr) return query;

    const AttributeValue* value = find_value(*map, target.key);
    if (value == nullptr) return query;

    query.status = AttributeStatus::Ok;
    query.source = target.scope;
    query.layer = target.scope == AttributeSource::Layer ? target.layer_id : kNoLayer;
    query.value = value;
    return query;
}

AttributeQuery AttributeStore::layer_value(LayerRef layer, AttributeKey key) const {
    return peek(AttributeTarget::layer(layer, key));
}

AttributeQuery AttributeStore::default_value(AttributeKey key) const {
    return peek(AttributeTarget::schema_default(key));
}

namespace {

/// Shared by the *_or readers: one warning, worded the same way every time.
void warn_type_mismatch(std::string_view name, AttributeType wanted, const AttributeQuery& query) {
    spdlog::warn("AttributeStore: \"{}\" is {} from {}, not {}; using the fallback", name,
                 attribute_type_name(query.value->type()), attribute_source_name(query.source),
                 attribute_type_name(wanted));
}

} // namespace

bool AttributeStore::bool_or(AttributeObject obj, AttributeKey key, LayerRef layer,
                             bool fallback) const {
    const AttributeQuery query = resolve_as(obj, key, layer, AttributeType::Bool);
    if (query.status == AttributeStatus::TypeMismatch) {
        warn_type_mismatch(key_name(key), AttributeType::Bool, query);
        return fallback;
    }
    return query.ok() ? *query.value->as_bool() : fallback;
}

double AttributeStore::double_or(AttributeObject obj, AttributeKey key, LayerRef layer,
                                 double fallback) const {
    const AttributeQuery query = resolve_as(obj, key, layer, AttributeType::Double);
    if (query.status == AttributeStatus::TypeMismatch) {
        warn_type_mismatch(key_name(key), AttributeType::Double, query);
        return fallback;
    }
    return query.ok() ? *query.value->as_double() : fallback;
}

std::string AttributeStore::string_or(AttributeObject obj, AttributeKey key, LayerRef layer,
                                      std::string fallback) const {
    const AttributeQuery query = resolve_as(obj, key, layer, AttributeType::String);
    if (query.status == AttributeStatus::TypeMismatch) {
        warn_type_mismatch(key_name(key), AttributeType::String, query);
        return fallback;
    }
    return query.ok() ? *query.value->as_string() : std::move(fallback);
}

std::vector<AttributeKey> AttributeStore::keys_on(AttributeObject obj, LayerRef layer) const {
    std::vector<uint32_t> indices;

    const auto gather = [&indices](const KeyValueMap& map) {
        indices.reserve(indices.size() + map.size());
        for (const auto& entry : map) {
            indices.push_back(entry.first);
        }
    };

    if (obj.valid()) {
        if (const ObjectRecord* rec = record(obj); rec != nullptr) {
            gather(rec->user_values);
            gather(rec->object_values);
        }
    }
    if (layer != kNoLayer) {
        if (const auto it = m_layers.find(layer); it != m_layers.end()) {
            gather(it->second);
        }
    }
    gather(m_defaults);

    // Sorted and deduplicated by key index. Key order rather than name order:
    // this is the row list of an inspector that groups by when a name was first
    // seen, and it is stable across runs, which unordered_map order is not.
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

    std::vector<AttributeKey> keys;
    keys.reserve(indices.size());
    for (const uint32_t index : indices) {
        keys.push_back(AttributeKey{index});
    }
    return keys;
}

// ============================================================================
// AttributeStore -- snapshots
// ============================================================================

ObjectSnapshot AttributeStore::snapshot(AttributeObject obj) const {
    ObjectSnapshot snap;

    const ObjectRecord* rec = record(obj);
    if (rec == nullptr) return snap;

    const auto copy_slot = [](const KeyValueMap& map,
                              std::vector<std::pair<AttributeKey, AttributeValue>>& out) {
        out.reserve(map.size());
        for (const auto& entry : map) {
            out.emplace_back(AttributeKey{entry.first}, entry.second);
        }
        std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
            return a.first.index() < b.first.index();
        });
    };

    copy_slot(rec->object_values, snap.object_values);
    copy_slot(rec->user_values, snap.user_values);
    return snap;
}

bool AttributeStore::restore(AttributeObject obj, const ObjectSnapshot& snapshot) {
    ObjectRecord* rec = record(obj);
    if (rec == nullptr) return false;

    rec->object_values.clear();
    rec->user_values.clear();

    for (const auto& entry : snapshot.object_values) {
        if (!entry.first.valid()) continue;
        rec->object_values.emplace(entry.first.index(), entry.second);
    }
    for (const auto& entry : snapshot.user_values) {
        if (!entry.first.valid()) continue;
        rec->user_values.emplace(entry.first.index(), entry.second);
    }
    return true;
}

// ============================================================================
// Commands
// ============================================================================

SetAttributeCommand::SetAttributeCommand(AttributeStore& store, AttributeTarget target,
                                         AttributeValue value)
    : m_store(&store), m_target(target), m_value(std::move(value)) {}

bool SetAttributeCommand::apply() {
    std::optional<AttributeValue> previous;
    if (!m_store->write(m_target, m_value, &previous)) return false;

    // Captured once, on the FIRST apply. Redo runs against a store that undo put
    // back, so a recapture would find the same value -- but after a merge it
    // would find this command's own intermediate write, and revert would then
    // land in the middle of a gesture instead of before it.
    if (!m_captured) {
        m_previous = std::move(previous);
        m_captured = true;
    }
    return true;
}

void SetAttributeCommand::revert() {
    const bool ok = m_previous.has_value() ? m_store->write(m_target, *m_previous, nullptr)
                                           : m_store->erase(m_target, nullptr);
    if (!ok) {
        // revert() must not fail; this one can only get here if the object was
        // destroyed behind the history's back, which is a bug in the caller that
        // destroyed it. Say so rather than leaving a silently half-undone step.
        spdlog::warn("SetAttributeCommand: revert of \"{}\" changed nothing; the target is gone",
                     m_store->key_name(m_target.key));
    }
}

std::string SetAttributeCommand::describe() const {
    return command_label(*m_store, m_target, "Set");
}

size_t SetAttributeCommand::footprint() const {
    size_t total = sizeof(SetAttributeCommand) + m_value.heap_bytes();
    if (m_previous.has_value()) total += m_previous->heap_bytes();
    return total;
}

bool SetAttributeCommand::merge(const Command& next) {
    const auto* other = dynamic_cast<const SetAttributeCommand*>(&next);
    if (other == nullptr) return false;
    if (other->m_store != m_store) return false;
    if (other->m_target != m_target) return false;

    // The successor already applied, so the store holds its value. Taking it
    // over means adopting that value for redo while keeping m_previous, which is
    // still the state from before this gesture started.
    m_value = other->m_value;
    return true;
}

ClearAttributeCommand::ClearAttributeCommand(AttributeStore& store, AttributeTarget target)
    : m_store(&store), m_target(target) {}

bool ClearAttributeCommand::apply() {
    std::optional<AttributeValue> previous;
    if (!m_store->erase(m_target, &previous)) {
        // Nothing in the slot. Clearing an attribute the object never had is a
        // no-op, and a no-op in the undo menu is an entry that does nothing when
        // a user picks it.
        return false;
    }
    m_previous = std::move(previous);
    return true;
}

void ClearAttributeCommand::revert() {
    if (!m_previous.has_value()) return;
    if (!m_store->write(m_target, *m_previous, nullptr)) {
        spdlog::warn("ClearAttributeCommand: revert of \"{}\" changed nothing; the target is gone",
                     m_store->key_name(m_target.key));
    }
}

std::string ClearAttributeCommand::describe() const {
    return command_label(*m_store, m_target, "Clear");
}

size_t ClearAttributeCommand::footprint() const {
    size_t total = sizeof(ClearAttributeCommand);
    if (m_previous.has_value()) total += m_previous->heap_bytes();
    return total;
}

bool set_attribute(CommandStack& stack, AttributeStore& store, const AttributeTarget& target,
                   AttributeValue value) {
    return stack.execute(std::make_unique<SetAttributeCommand>(store, target, std::move(value)));
}

bool clear_attribute(CommandStack& stack, AttributeStore& store, const AttributeTarget& target) {
    return stack.execute(std::make_unique<ClearAttributeCommand>(store, target));
}

} // namespace stratum::scene
