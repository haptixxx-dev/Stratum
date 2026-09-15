// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file attributes.hpp
 * @brief Typed key/values on scene objects, resolved through layer inheritance
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### What this is for
 *
 * Three later tracks all want the same thing and would otherwise each invent it.
 * The OSM importer has a tag map per object and nowhere to put it. The rule
 * engine (Track D) reads `height`, `zoning` and whatever else a rule names.
 * Reports (Track I) aggregate those values across a selection. The inspector
 * (K1) edits them. One store, written once, serves all four.
 *
 * ### A value is a value plus where it came from
 *
 * The single most important idea here, and the one that is easy to under-build.
 * Asking "what is this building's height" has five possible answers and the
 * NUMBER is only half of each of them:
 *
 *   - **User** -- a human typed it into the inspector for this object.
 *   - **Object** -- the importer or a rule wrote it onto this object.
 *   - **Layer** -- nothing on the object has it, so the layer's value applies.
 *   - **Default** -- nothing anywhere has it, so the declared fallback applies.
 *   - **None** -- nothing has it at all.
 *
 * Resolution walks that list top to bottom and reports which rung it stopped on,
 * because that is what answers the question a user actually has: *did my edit
 * stick, or is something overriding it?* A store that returned only the value
 * makes that question unanswerable and every "why is this building still 9m"
 * bug report unfixable.
 *
 * **User outranks Object on purpose.** The rule engine rewrites Object values on
 * every re-evaluation. If a human's edit lived in the same slot it would be
 * destroyed by the next rule run, silently, and the user would see their typed
 * number revert with no explanation. Two slots is what keeps a hand edit alive
 * across a re-run, and it is what CityEngine does for the same reason.
 *
 * ### Clearing is not setting a default
 *
 * `clear` removes the object's own value so resolution falls THROUGH to the
 * layer or the default. Setting the object's value to the same number the
 * default happens to hold leaves the object owning that number. Both read back
 * the same number; they report different sources, and they behave differently
 * the moment the layer changes. Keeping them distinct is why clear is its own
 * command rather than a set of a blank value.
 *
 * ### There is no integer type
 *
 * Deliberate. OSM tags arrive as strings, CGA-style rules work in doubles, and
 * every boundary between an Int and a Double is a place to lose data quietly: a
 * rule that writes 3.5 into an Int field truncates, and a reader that accepts an
 * Int where it asked for a Double has re-introduced exactly the silent
 * conversion the type tag exists to prevent. A double represents every integer
 * up to 2^53 exactly, which covers every floor count, year, lane count and
 * storey a scene will ever hold. Values that genuinely need more -- an OSM way
 * id, a UUID -- are identifiers rather than numbers and belong in a String.
 *
 * ### Type mismatch is reported, never converted
 *
 * A caller asking for a double on a string attribute has a bug. resolve_as()
 * hands back AttributeStatus::TypeMismatch together with the value it did find,
 * so an inspector can say "this is a string". The `*_or` convenience readers go
 * further and log, because a caller that named a concrete type and supplied a
 * fallback was not probing -- it expected that type and is about to use a number
 * it did not get.
 *
 * ### Keys are interned
 *
 * Attribute names come from OSM tags and from rule attributes: tens of thousands
 * of objects share a few hundred distinct names. Storing `std::string` per
 * object per attribute would spend most of the store on repeated copies of the
 * word "building". AttributeKey is a dense uint32 index into the store's own
 * intern table, so a per-object map entry is 4 bytes of key, comparison is an
 * integer compare, and A2 serialises the table once instead of the names
 * millions of times.
 *
 * Keys are never removed. An AttributeKey handed out at import time is still
 * valid at export time, which is the property that lets a rule cache one.
 *
 * ### No EnTT here
 *
 * EnTT links into stratum_core and is used by nothing. Committing the scene
 * model to an ECS before a single system iterates components buys nothing and
 * costs A2 (save/load) a serialisation story for a registry. This is plain
 * owning data with generational handles, which is the pattern the renderer
 * already uses for GPU resources and which serialises as two integers.
 *
 * ### Handles
 *
 * AttributeObject is an index plus a generation. The index addresses a slot in a
 * dense vector; the generation is bumped when the slot is destroyed. A handle to
 * a destroyed object therefore fails validation instead of silently addressing
 * whatever was created in its place, and unrelated creations and destructions
 * never move a live object's slot. A slot whose generation would wrap back to 0
 * is retired rather than recycled -- 2^32 recycles of one slot is not a case
 * anyone will hit, and pretending it cannot happen is how a handle system starts
 * aliasing after a long session.
 *
 * ### The layer handle is opaque
 *
 * This file does not know what a layer is and deliberately does not include
 * A1's layer header. LayerRef is whatever integer identity the scene document
 * gives a layer, passed in by the caller at every resolve. Layer MEMBERSHIP
 * belongs to the document, not here; duplicating it would give two answers to
 * "which layer is this in" and they would drift.
 */

#pragma once

#include "scene/command.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace stratum::scene {

// ============================================================================
// Handles
// ============================================================================

/**
 * @brief Opaque identity of a layer, minted by the scene document
 *
 * 64 bits so that whatever A1 settles on fits without losing information: a
 * plain index today, or an index and a generation packed together later.
 * Nothing in this file interprets the value -- it is a map key and an equality
 * test -- and nothing here includes A1's layer header, because layer MEMBERSHIP
 * is the document's business and duplicating it would give two answers to
 * "which layer is this in".
 *
 * Joining the two is `static_cast<LayerRef>(layer_id)`, with no special case,
 * because kNoLayer below is 0 and so is A1's kInvalidLayer.
 */
using LayerRef = uint64_t;

/**
 * @brief "This object is in no layer", and "skip the Layer rung of resolution"
 *
 * Zero, deliberately, and it is a RESERVED handle: a write targeting it is
 * refused. That is what makes the join to A1 a plain cast -- its kInvalidLayer
 * is 0 too, so a caller that forwards an unparented object's layer id lands on
 * "no layer" rather than on a layer record numbered 0 that silently accumulates
 * the attributes of everything unparented.
 */
inline constexpr LayerRef kNoLayer = 0;

/**
 * @brief Interned attribute name
 *
 * Obtained from AttributeStore::intern() and valid for that store's lifetime.
 * A default-constructed key is invalid and resolves to nothing; it is not the
 * name "".
 *
 * @note Keys from two different stores are not interchangeable. They compare as
 *       integers, so mixing them reads the wrong attribute rather than failing.
 *       There is one store per document, so in practice this only bites tests.
 */
class AttributeKey {
public:
    static constexpr uint32_t kInvalidIndex = 0xFFFFFFFFu;

    AttributeKey() = default;

    [[nodiscard]] bool valid() const { return m_index != kInvalidIndex; }

    /// Dense index into the store's intern table. Stable; A2 serialises this.
    [[nodiscard]] uint32_t index() const { return m_index; }

    friend bool operator==(AttributeKey a, AttributeKey b) { return a.m_index == b.m_index; }
    friend bool operator!=(AttributeKey a, AttributeKey b) { return a.m_index != b.m_index; }

private:
    friend class AttributeStore;
    explicit AttributeKey(uint32_t index) : m_index(index) {}

    uint32_t m_index = kInvalidIndex;
};

/**
 * @brief Identity of one object's attribute record
 *
 * Minted by AttributeStore::create_object(). The scene object that owns the
 * attributes holds one of these; the store is the authority on whether it is
 * still live. See the handle section of the file comment for why the generation
 * is here.
 */
struct AttributeObject {
    static constexpr uint32_t kInvalidIndex = 0xFFFFFFFFu;

    uint32_t index = kInvalidIndex;
    uint32_t generation = 0;

    /// True when this handle names a slot. Says nothing about whether it is live.
    [[nodiscard]] bool valid() const { return index != kInvalidIndex; }

    friend bool operator==(AttributeObject a, AttributeObject b) {
        return a.index == b.index && a.generation == b.generation;
    }
    friend bool operator!=(AttributeObject a, AttributeObject b) { return !(a == b); }
};

// ============================================================================
// Types, sources and statuses
// ============================================================================

/**
 * @brief What an attribute holds
 *
 * The enumerator order matches the alternative order of AttributeValue's
 * variant, and attributes.cpp static_asserts that. Keep them in step: the type
 * tag is read straight off the variant index rather than stored twice.
 */
enum class AttributeType : uint8_t {
    Bool = 0,
    Double,
    String,
    BoolArray,
    DoubleArray,
    StringArray,
};

/// "bool", "double", "string", "bool[]", "double[]", "string[]".
[[nodiscard]] std::string_view attribute_type_name(AttributeType type);

/**
 * @brief Which rung of the resolution chain a value came from
 *
 * Ascending precedence: a later enumerator beats an earlier one. The inspector
 * shows this next to every value, and AttributeTarget reuses it to name which
 * rung a write goes to, so "shows Layer, override writes Object" is one
 * vocabulary rather than two.
 */
enum class AttributeSource : uint8_t {
    None = 0,   ///< Nothing anywhere holds this attribute
    Default,    ///< The declared document-wide fallback for this key
    Layer,      ///< Inherited from the layer the caller named
    Object,     ///< Written onto the object by the importer or a rule
    User,       ///< Typed by a human; survives a rule re-run
};

/// "None", "Default", "Layer", "Object", "User".
[[nodiscard]] std::string_view attribute_source_name(AttributeSource source);

/// Outcome of a read. Ok only when a value was found AND matched the type asked for.
enum class AttributeStatus : uint8_t {
    Ok = 0,
    Missing,       ///< No rung of the chain held the attribute
    TypeMismatch,  ///< Found, but it is not the type the caller asked for
};

/// "Ok", "Missing", "TypeMismatch".
[[nodiscard]] std::string_view attribute_status_name(AttributeStatus status);

// ============================================================================
// Value
// ============================================================================

/**
 * @brief One typed attribute value
 *
 * Constructed only through the named factories. There is no converting
 * constructor and no default constructor, both on purpose: `AttributeValue v =
 * "9"` silently landing in a bool alternative is the classic std::variant trap,
 * and a default-constructed value would be an untyped zero of exactly the kind
 * this class exists to prevent.
 *
 * Reads return pointers rather than std::optional. `std::optional<bool>` reads
 * as false in an `if` when the attribute is present and false, which is a bug
 * waiting on its first author; a pointer is null only when the type did not
 * match.
 */
class AttributeValue {
public:
    /**
     * @brief Array of bools
     *
     * std::vector<bool> and its proxy references, eyes open. The alternative,
     * std::vector<uint8_t>, makes "array of flags" and "array of small numbers"
     * the same C++ type -- in a system whose entire job is keeping types apart.
     * Element reads (`(*v)[i]`) and `==` behave; only `data()` and `auto&` over
     * an element do not, and nothing here needs either.
     */
    using BoolArray = std::vector<bool>;
    using DoubleArray = std::vector<double>;
    using StringArray = std::vector<std::string>;

    [[nodiscard]] static AttributeValue from_bool(bool value);
    [[nodiscard]] static AttributeValue from_double(double value);
    [[nodiscard]] static AttributeValue from_string(std::string value);
    [[nodiscard]] static AttributeValue from_bools(BoolArray value);
    [[nodiscard]] static AttributeValue from_doubles(DoubleArray value);
    [[nodiscard]] static AttributeValue from_strings(StringArray value);

    /**
     * @brief The empty value of a type: false, 0.0, "", or an empty array
     *
     * Named zero_of and not default_of, because AttributeSource::Default is a
     * different idea entirely and naming them alike is how the two get confused
     * in a review.
     */
    [[nodiscard]] static AttributeValue zero_of(AttributeType type);

    [[nodiscard]] AttributeType type() const;
    [[nodiscard]] bool is(AttributeType type_tag) const { return type() == type_tag; }
    [[nodiscard]] bool is_array() const;

    /// @return The value, or nullptr when this holds some other type.
    [[nodiscard]] const bool* as_bool() const;
    [[nodiscard]] const double* as_double() const;
    [[nodiscard]] const std::string* as_string() const;
    [[nodiscard]] const BoolArray* as_bool_array() const;
    [[nodiscard]] const DoubleArray* as_double_array() const;
    [[nodiscard]] const StringArray* as_string_array() const;

    /// Elements, for arrays. 1 for a scalar, so a report can sum either.
    [[nodiscard]] size_t size() const;

    /// Heap this value owns, excluding the object itself. Feeds Command::footprint().
    [[nodiscard]] size_t heap_bytes() const;

    /// sizeof plus heap_bytes().
    [[nodiscard]] size_t footprint() const { return sizeof(AttributeValue) + heap_bytes(); }

    /**
     * @brief Rendering for the inspector and for log lines
     *
     * Strings are quoted so that the string "true" is distinguishable from the
     * bool. NOT a serialisation format -- doubles go through the default stream
     * precision and lose digits. A2 writes JSON and must not call this.
     */
    [[nodiscard]] std::string to_display_string() const;

    friend bool operator==(const AttributeValue& a, const AttributeValue& b) {
        return a.m_data == b.m_data;
    }
    friend bool operator!=(const AttributeValue& a, const AttributeValue& b) { return !(a == b); }

private:
    using Storage = std::variant<bool, double, std::string, BoolArray, DoubleArray, StringArray>;

    explicit AttributeValue(Storage data) : m_data(std::move(data)) {}

    Storage m_data;
};

// ============================================================================
// Query result
// ============================================================================

/**
 * @brief What a read found, and where
 *
 * @warning `value` is non-null whenever SOMETHING was found -- including on a
 *          TypeMismatch, so that a caller can report what the attribute
 *          actually is. Test ok(), never `value != nullptr`.
 *
 * @warning `value` points into the store and is invalidated by the next
 *          mutation of that store. Copy it if it has to outlive the call.
 */
struct AttributeQuery {
    AttributeStatus status = AttributeStatus::Missing;
    AttributeSource source = AttributeSource::None;

    /// Which layer supplied it. Meaningful only when source is Layer.
    LayerRef layer = kNoLayer;

    const AttributeValue* value = nullptr;

    [[nodiscard]] bool ok() const { return status == AttributeStatus::Ok; }
    [[nodiscard]] explicit operator bool() const { return ok(); }
};

// ============================================================================
// Write target
// ============================================================================

/**
 * @brief The one slot a write lands in
 *
 * `scope` reuses AttributeSource so that the rung a read reports and the rung a
 * write targets are spelled the same way. AttributeSource::None is not a
 * writable scope and a target carrying it is refused.
 */
struct AttributeTarget {
    AttributeSource scope = AttributeSource::None;
    AttributeObject object_id{};          ///< Used when scope is Object or User
    LayerRef layer_id = kNoLayer;         ///< Used when scope is Layer
    AttributeKey key{};

    [[nodiscard]] static AttributeTarget object(AttributeObject obj, AttributeKey key);
    [[nodiscard]] static AttributeTarget user(AttributeObject obj, AttributeKey key);
    [[nodiscard]] static AttributeTarget layer(LayerRef layer, AttributeKey key);
    [[nodiscard]] static AttributeTarget schema_default(AttributeKey key);

    friend bool operator==(const AttributeTarget& a, const AttributeTarget& b) {
        return a.scope == b.scope && a.object_id == b.object_id && a.layer_id == b.layer_id
               && a.key == b.key;
    }
    friend bool operator!=(const AttributeTarget& a, const AttributeTarget& b) { return !(a == b); }
};

/**
 * @brief Every value one object owns, in key order
 *
 * Taken before an owner deletes the object so the owner's delete command can put
 * the attributes back on undo -- "a command owns whatever its undo needs", per
 * command.hpp. Sorted by key index so that a golden test or a save file does not
 * depend on unordered_map iteration order.
 */
struct ObjectSnapshot {
    std::vector<std::pair<AttributeKey, AttributeValue>> object_values;
    std::vector<std::pair<AttributeKey, AttributeValue>> user_values;

    [[nodiscard]] size_t footprint() const;
    [[nodiscard]] bool empty() const { return object_values.empty() && user_values.empty(); }
};

class SetAttributeCommand;
class ClearAttributeCommand;

// ============================================================================
// Store
// ============================================================================

/**
 * @brief The attributes of one document
 *
 * Owns the intern table, the per-object records, the per-layer records and the
 * declared defaults. One per scene document.
 *
 * Not thread-safe, and not copyable. Copying is deleted rather than defaulted
 * because the intern table would need rebuilding on the copy and a defaulted
 * copy that looks correct is worse than one that does not compile; move works
 * and is what a document swap needs. Nothing in the design forbids a copy, so
 * add a hand-written one the day something wants it.
 *
 * @note The store must outlive any CommandStack holding attribute commands.
 *       Commands hold a raw pointer to it, which is the same lifetime rule the
 *       rest of the command system will follow: the document owns both.
 */
class AttributeStore {
public:
    AttributeStore() = default;

    AttributeStore(const AttributeStore&) = delete;
    AttributeStore& operator=(const AttributeStore&) = delete;
    AttributeStore(AttributeStore&&) = default;
    AttributeStore& operator=(AttributeStore&&) = default;

    // ------------------------------------------------------------------
    // Keys
    // ------------------------------------------------------------------

    /// Key for @p name, creating it on first use. Idempotent and allocation-free on a hit.
    [[nodiscard]] AttributeKey intern(std::string_view name);

    /// Key for @p name, or an invalid key when nothing ever interned it.
    [[nodiscard]] AttributeKey find_key(std::string_view name) const;

    /// Name behind @p key, or "" when the key is invalid or from another store.
    [[nodiscard]] std::string_view key_name(AttributeKey key) const;

    [[nodiscard]] size_t key_count() const { return m_key_names.size(); }

    // ------------------------------------------------------------------
    // Object lifetime
    // ------------------------------------------------------------------

    /**
     * @brief Mint an attribute record
     *
     * Not an undoable edit: object lifetime belongs to whoever owns the scene
     * object, and its create/delete commands compose this with their own work
     * inside a transaction. An undoable DELETE should snapshot(), clear through
     * commands, and leave the record alive -- destroying it bumps the generation
     * and every handle to the object held elsewhere dies with it.
     */
    [[nodiscard]] AttributeObject create_object();

    /// @return false when the handle was already stale.
    bool destroy_object(AttributeObject obj);

    [[nodiscard]] bool is_valid(AttributeObject obj) const { return record(obj) != nullptr; }

    [[nodiscard]] size_t live_object_count() const { return m_live_objects; }

    // ------------------------------------------------------------------
    // Reads
    // ------------------------------------------------------------------

    /**
     * @brief Walk the chain: User, Object, Layer, Default
     *
     * @param obj   Object to read. An invalid (default-constructed) handle skips
     *              the object rungs, which is how the inspector asks what a layer
     *              alone would give. A STALE handle is a different thing: it
     *              warns and resolves to nothing, because continuing on to the
     *              layer would answer a question about an object that is gone.
     * @param key   Interned name.
     * @param layer Layer to inherit from, or kNoLayer. Required rather than
     *              defaulted: a forgotten layer argument silently loses
     *              inheritance, and there is no way to see that in the result.
     */
    [[nodiscard]] AttributeQuery resolve(AttributeObject obj, AttributeKey key,
                                         LayerRef layer) const;

    /// resolve(), with Ok downgraded to TypeMismatch when the type is not @p wanted.
    [[nodiscard]] AttributeQuery resolve_as(AttributeObject obj, AttributeKey key, LayerRef layer,
                                            AttributeType wanted) const;

    /// One rung only, no inheritance. What the inspector needs to grey out "Clear".
    [[nodiscard]] AttributeQuery peek(const AttributeTarget& target) const;

    /// The layer's own value, so a caller with a layer hierarchy can walk it itself.
    [[nodiscard]] AttributeQuery layer_value(LayerRef layer, AttributeKey key) const;

    /// The declared document-wide fallback.
    [[nodiscard]] AttributeQuery default_value(AttributeKey key) const;

    /**
     * @brief Resolve to a concrete type, or fall back
     *
     * Logs on a type mismatch. A caller that named a type and supplied a
     * fallback was not probing, so a mismatch is a bug in the caller and it
     * should hear about it even if it ignores the return.
     */
    [[nodiscard]] bool bool_or(AttributeObject obj, AttributeKey key, LayerRef layer,
                               bool fallback) const;
    [[nodiscard]] double double_or(AttributeObject obj, AttributeKey key, LayerRef layer,
                                   double fallback) const;
    [[nodiscard]] std::string string_or(AttributeObject obj, AttributeKey key, LayerRef layer,
                                        std::string fallback) const;

    /**
     * @brief Every key that resolves to something for this object, in key order
     *
     * The union of the object's own keys, the layer's and the declared defaults,
     * deduplicated. This is the inspector's row list.
     */
    [[nodiscard]] std::vector<AttributeKey> keys_on(AttributeObject obj, LayerRef layer) const;

    // ------------------------------------------------------------------
    // Snapshots
    // ------------------------------------------------------------------

    [[nodiscard]] ObjectSnapshot snapshot(AttributeObject obj) const;

    /// Replace both slots wholesale. @return false when the handle is stale.
    bool restore(AttributeObject obj, const ObjectSnapshot& snapshot);

    // ------------------------------------------------------------------
    // Load
    // ------------------------------------------------------------------

    /**
     * @brief Write with NO undo history
     *
     * @warning This is a deliberate hole in the history and the only one. It
     *          exists for the two callers that legitimately have no history to
     *          write into: the OSM importer, which sets a few million tags
     *          before the document is ever shown, and A2's loader. Recording
     *          those as commands would fill the undo stack with the act of
     *          opening a file.
     *
     *          Anything that runs while a user is looking at the document uses
     *          set_attribute(). A mutation that skips the stack is a hole whose
     *          symptom is an undo three steps later restoring state that was
     *          never current -- see command.hpp.
     */
    bool load_value(const AttributeTarget& target, AttributeValue value);

private:
    // write() and erase() are the primitives every mutation is built from, and
    // they are private so that no caller can reach them without going through a
    // command. SetAttributeCommand and ClearAttributeCommand are the only ways
    // in; load_value() is the single audited exception above.
    friend class SetAttributeCommand;
    friend class ClearAttributeCommand;

    using KeyValueMap = std::unordered_map<uint32_t, AttributeValue>;

    struct ObjectRecord {
        uint32_t generation = 0;
        bool alive = false;
        KeyValueMap object_values;
        KeyValueMap user_values;
    };

    /**
     * @brief Set @p target to @p value
     *
     * @param out_previous When non-null, receives what the slot held before, or
     *                     nullopt when it held nothing. The command needs that
     *                     for revert() and cannot read it separately without a
     *                     window where another write could land between.
     * @return false when the target is invalid -- unknown key, stale object,
     *         kNoLayer on a layer write. The store is untouched.
     */
    bool write(const AttributeTarget& target, AttributeValue value,
               std::optional<AttributeValue>* out_previous);

    /// @return false when the target is invalid or the slot held nothing.
    bool erase(const AttributeTarget& target, std::optional<AttributeValue>* out_previous);

    [[nodiscard]] bool target_valid(const AttributeTarget& target) const;

    [[nodiscard]] const ObjectRecord* record(AttributeObject obj) const;
    [[nodiscard]] ObjectRecord* record(AttributeObject obj);

    [[nodiscard]] const KeyValueMap* slot_map(const AttributeTarget& target) const;
    [[nodiscard]] KeyValueMap* slot_map_for_write(const AttributeTarget& target);

    [[nodiscard]] static const AttributeValue* find_value(const KeyValueMap& map, AttributeKey key);

    /// Transparent so intern() and find_key() can look up a string_view without allocating.
    struct StringHash {
        using is_transparent = void;
        [[nodiscard]] size_t operator()(std::string_view text) const noexcept {
            return std::hash<std::string_view>{}(text);
        }
    };

    /**
     * @brief index -> name
     *
     * A deque, not a vector. key_name() hands out a std::string_view into these,
     * and a vector reallocating on the next intern() would leave every
     * previously returned view dangling -- a short name lives in the string's
     * own SSO buffer, which moves with the string. A deque never moves an
     * element that is already in it.
     */
    std::deque<std::string> m_key_names;

    /// name -> index. Owns its own copy of the name; see the note on copying above.
    std::unordered_map<std::string, uint32_t, StringHash, std::equal_to<>> m_key_index;

    std::vector<ObjectRecord> m_objects;
    std::vector<uint32_t> m_free_objects;
    size_t m_live_objects = 0;

    std::unordered_map<LayerRef, KeyValueMap> m_layers;
    KeyValueMap m_defaults;
};

// ============================================================================
// Commands
// ============================================================================

/**
 * @brief Put a value in one slot
 *
 * Merges with a following set on the SAME target, which is what makes typing
 * into an inspector field one undo step instead of one per keystroke. The merged
 * command keeps the value captured before the FIRST apply, so undoing after a
 * gesture lands on the state the gesture started from rather than on an
 * intermediate keystroke.
 */
class SetAttributeCommand : public Command {
public:
    SetAttributeCommand(AttributeStore& store, AttributeTarget target, AttributeValue value);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;
    [[nodiscard]] bool merge(const Command& next) override;

private:
    AttributeStore* m_store;
    AttributeTarget m_target;
    AttributeValue m_value;

    /// What the slot held before the first apply(). nullopt means it held nothing.
    std::optional<AttributeValue> m_previous;
    bool m_captured = false;
};

/**
 * @brief Take the value out of one slot so resolution falls through
 *
 * Refuses when the slot holds nothing. Clearing an attribute the object never
 * had is a no-op, and a no-op recorded as an undo step is a menu entry that
 * changes nothing when a user picks it. Never merges: a clear is the end of a
 * gesture, not part of one.
 */
class ClearAttributeCommand : public Command {
public:
    ClearAttributeCommand(AttributeStore& store, AttributeTarget target);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;

private:
    AttributeStore* m_store;
    AttributeTarget m_target;
    std::optional<AttributeValue> m_previous;
};

/// Build a SetAttributeCommand and run it through @p stack. @return the stack's answer.
bool set_attribute(CommandStack& stack, AttributeStore& store, const AttributeTarget& target,
                   AttributeValue value);

/// Build a ClearAttributeCommand and run it through @p stack. False when there was nothing to clear.
bool clear_attribute(CommandStack& stack, AttributeStore& store, const AttributeTarget& target);

} // namespace stratum::scene

/// Hash for AttributeKey, so callers can key their own sets and maps by it.
template <>
struct std::hash<stratum::scene::AttributeKey> {
    [[nodiscard]] std::size_t operator()(stratum::scene::AttributeKey key) const noexcept {
        return std::hash<std::uint32_t>{}(key.index());
    }
};
