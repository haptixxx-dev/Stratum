// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file document.hpp
 * @brief The `.stratum` document: the scene A1, A3 and A4 own, and how it survives a restart
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this exists
 *
 * Nothing in Stratum currently survives a restart. The importer rebuilds the
 * scene from the `.osm` every time, which means every layer the user made, every
 * attribute they typed and every name they gave something is lost on exit. A2
 * closes M2 by giving the four M2 features -- the layer tree, the attribute
 * store, the undo stack and the object model they share -- one owner that can be
 * written to disk and read back.
 *
 * Document is that owner. It is not a new subsystem: it holds a LayerTree, an
 * AttributeStore and a CommandStack, and adds the one thing none of them could
 * own on its own -- **which objects exist and which layer each one is in**.
 * attributes.hpp is explicit that layer MEMBERSHIP belongs to the document
 * rather than the store ("duplicating it would give two answers to 'which layer
 * is this in' and they would drift"), and selection.hpp asks for exactly this
 * with its ObjectLayerFn. Document is where that answer lives.
 *
 * ### The format: JSON for structure, blobs beside it
 *
 * A `.stratum` file is UTF-8 JSON, pretty-printed. Geometry does not go in it.
 * A city's meshes base64'd into JSON inflate by a third and turn a file a human
 * can read and a VCS can diff into one neither can, so the format reserves a
 * top-level `blobs` array naming sidecar files stored next to the document. It
 * is empty in version 1 and a non-empty one is REFUSED rather than ignored: a
 * document that says it has geometry and a reader that silently drops it is data
 * loss the user only discovers later.
 *
 * ### Three invariants the loader exists to protect
 *
 * **1. Structure has one spelling.** The layer tree is written NESTED -- a
 * layer's children are inside it -- so parentage and sibling order cannot
 * disagree with each other. A flat list carrying a `parent` field alongside an
 * ordered `children` field can contradict itself, and then the loader has to
 * pick a winner and every choice is wrong for somebody.
 *
 * **2. A handle written in the file names the same thing after a load.** Layer
 * ids and object handles appear in the file; attribute keys do not (they are
 * written by NAME), so key indices are re-interned on load and need not match.
 *
 * **3. The allocators are restored, not just the objects.** This is the part
 * most likely to be got wrong, so it is stated plainly:
 *
 *   - A1's LayerTree hands out ids from a counter that never goes back and never
 *     reuses. Restoring only the surviving layers restarts that counter just
 *     above the highest SURVIVING id -- and every id above it that the saved
 *     session had already issued to a since-deleted layer gets issued a second
 *     time. A LayerId held anywhere else then names a different layer. So
 *     `next_id` is saved and restored, not recomputed.
 *   - A3's AttributeObject is an index AND a generation. Restoring an object at
 *     index 3 in a fresh store gives it generation 0, so a stale handle from the
 *     saved session -- index 3, generation 2 -- would validate against it. So
 *     every slot is saved, dead ones included, with its generation, and the
 *     loader replays the store's create/destroy history to reproduce it.
 *
 * The one thing deliberately NOT reproduced is the store's free-list ORDER, so
 * the index the next create_object() returns after a load may differ from the
 * one it would have returned before the save. That is not a correctness
 * property: a handle is an index and a generation, and every slot's generation
 * is restored, so whichever slot comes back cannot alias anything.
 *
 * ### The undo history is cleared on load
 *
 * Not kept, and not optional. A command holds the state its revert() needs and a
 * raw pointer to the tree or store it edits; after a load both are different
 * objects holding different content, so undoing a pre-load edit would apply an
 * inverse to a scene that never had the original. CommandStack::clear() is built
 * for this, and its comment is worth reading: it deliberately does NOT reset
 * revision(), because revision is how this file answers "is this document
 * dirty", and resetting it would make a freshly loaded document compare equal to
 * an unsaved one that happened to be at zero.
 *
 * Document therefore records the revision it was last saved or loaded AT, rather
 * than assuming zero -- see dirty(). Object lifetime is not a command (A3 is
 * explicit that object lifetime belongs to the document, not the stack), so
 * dirty() also watches a structural counter of its own; without it, creating an
 * object and quitting would lose it with no prompt.
 *
 * ### Untrusted input
 *
 * SECURITY.md scopes out "a deliberately corrupted internal scene file you
 * generated yourself", but a `.stratum` arrives by e-mail, download and shared
 * drive like any other project file, so the loader treats it as hostile:
 * everything is checked before it is used, a failed load leaves the destination
 * document EXACTLY as it was, and the two places where a small number in the
 * file commands a large amount of work -- nesting depth and generation replay --
 * are bounded by the constants below rather than trusted.
 *
 * ### Extension points, and what they expect
 *
 * A5 (selection) and A6 (georeference) are being written alongside this and are
 * deliberately not serialised yet. Two top-level member names are reserved for
 * them, and the loader IGNORES both so that a build which writes one is still
 * readable here:
 *
 *   - `"selection"` -- A5's SelectionSnapshot: an ordered list of SelectionItem,
 *     each a kind plus either a LayerId or an AttributeObject (index AND
 *     generation -- dropping the generation is the aliasing bug above), plus the
 *     primary item. It may be added WITHOUT a version bump: a reader that drops
 *     it loses a highlight the user can remake with one click.
 *   - `"georeference"` -- A6's GeoreferenceFrame: the origin GeoPoint and
 *     whatever else the frame carries. Adding it REQUIRES a version bump to 2,
 *     because a reader that drops it loses where on Earth the scene is, and
 *     every coordinate in the file silently becomes unlocatable.
 *
 * That asymmetry is the rule for anything added later: bump the version when
 * dropping the new member would lose something the user cannot trivially
 * recreate, and do not when it would not.
 */

#pragma once

#include "scene/attributes.hpp"
#include "scene/command.hpp"
#include "scene/layer.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace stratum::scene {

// ============================================================================
// Format constants
// ============================================================================

/**
 * @brief Version of the on-disk format this build writes and is the newest it reads
 *
 * Bumped when a change would make an older build read a newer file WRONGLY.
 * Adding a member an old reader can safely ignore does not need a bump; removing
 * one, renaming one, or adding one whose absence loses data does.
 */
inline constexpr uint32_t kDocumentFormatVersion = 1;

/// Value of the top-level `format` member. Present so that a file that is merely
/// valid JSON is not mistaken for a scene.
inline constexpr const char* kDocumentFormatTag = "stratum-document";

/// Conventional extension. Not enforced -- the tag above is what identifies a file.
inline constexpr const char* kDocumentFileExtension = ".stratum";

/**
 * @brief Top-level member names reserved for A5 and A6
 *
 * Named here rather than only described in prose, so that the extension point is
 * something a later change greps for instead of rediscovers. Nothing writes
 * either yet, and the loader ignores both. The file comment above says what each
 * one is expected to carry, and which of the two needs a version bump when it
 * arrives (georeference does; selection does not).
 */
inline constexpr const char* kReservedSelectionMember = "selection";
inline constexpr const char* kReservedGeoreferenceMember = "georeference";

/**
 * @brief Deepest layer nesting a file may contain
 *
 * The reader walks nested `children` arrays recursively, so an attacker's
 * million-deep file would be a stack overflow rather than an error message.
 * 256 is far past anything a human builds in a layer panel, and the refusal
 * names the depth, so a legitimate file that somehow hits it is diagnosable.
 */
inline constexpr size_t kMaxDocumentLayerDepth = 256;

/**
 * @brief Highest per-slot generation the loader will reproduce
 *
 * The public AttributeStore API raises a slot's generation only by replaying
 * destroy/create, one step per generation, so a file claiming generation
 * 4,000,000,000 is a four-billion-iteration stall produced by ten characters.
 * A slot reaches generation N by being recycled N times in one session; 65536 is
 * already an implausible number of recycles of a single slot, and the refusal
 * says so.
 */
inline constexpr uint32_t kMaxRestorableGeneration = 1u << 16;

/// Ceiling on the TOTAL replay work across every slot, for the same reason:
/// a million slots each just under the per-slot cap is the same stall spread out.
inline constexpr size_t kMaxGenerationReplaySteps = 1u << 24;

/**
 * @brief Largest file load_from_file() will read
 *
 * Bounds the allocation a hostile path can cause before any parsing starts. A
 * structural document is kilobytes to a few megabytes; geometry belongs in the
 * blobs that sit beside it, so a half-gigabyte of JSON is a malformed or hostile
 * file rather than a big scene.
 */
inline constexpr uintmax_t kMaxDocumentFileBytes = 512ull * 1024ull * 1024ull;

// ============================================================================
// Result
// ============================================================================

/**
 * @brief Outcome of a save or a load
 *
 * Carries a message rather than an error code because every one of these
 * failures ends up in front of a user, and "layers.roots[2].own_transform.scale:
 * expected 3 numbers" is worth more than an enumerator. `error` is non-empty
 * exactly when `ok` is false.
 */
struct DocumentIoResult {
    bool ok = false;
    std::string error;

    [[nodiscard]] explicit operator bool() const { return ok; }

    [[nodiscard]] static DocumentIoResult success() { return DocumentIoResult{true, {}}; }
    [[nodiscard]] static DocumentIoResult failure(std::string message) {
        return DocumentIoResult{false, std::move(message)};
    }
};

// ============================================================================
// Document
// ============================================================================

/**
 * @brief One open scene: its layers, its objects, their attributes, and its history
 *
 * Not thread-safe, for the reason CommandStack and LayerTree are not: edits come
 * from the UI thread and a background job hands a command back rather than
 * pushing from its own.
 *
 * Neither copyable nor movable, on purpose. Every Command on the stack holds a
 * raw pointer into this document's LayerTree or AttributeStore, so moving the
 * document would leave the history pointing at a moved-from husk -- a dangling
 * pointer that only fires on the next undo, long after the move. Loading is
 * therefore NOT implemented as "build a new document and move it in": see
 * load_from_json().
 */
class Document {
public:
    Document() = default;

    Document(const Document&) = delete;
    Document& operator=(const Document&) = delete;
    Document(Document&&) = delete;
    Document& operator=(Document&&) = delete;

    // ── The three subsystems ────────────────────────────────────────────────

    [[nodiscard]] LayerTree& layers() { return m_layers; }
    [[nodiscard]] const LayerTree& layers() const { return m_layers; }

    [[nodiscard]] AttributeStore& attributes() { return m_attributes; }
    [[nodiscard]] const AttributeStore& attributes() const { return m_attributes; }

    [[nodiscard]] CommandStack& history() { return m_history; }
    [[nodiscard]] const CommandStack& history() const { return m_history; }

    // ── Objects ─────────────────────────────────────────────────────────────

    /**
     * @brief Create a scene object and its attribute record
     *
     * NOT an undoable edit, and deliberately not a Command: attributes.hpp is
     * explicit that object lifetime belongs to whoever owns the scene object,
     * and that an undoable delete should snapshot the attributes and leave the
     * record ALIVE, because destroying it bumps the generation and kills every
     * handle to the object held elsewhere. So this is the raw lifetime call that
     * a future create/delete command composes inside a transaction.
     *
     * @param layer Layer the object belongs to, or kInvalidLayer for none. Not
     *              validated against the tree: the importer creates objects
     *              before their layers exist, and refusing here would force it
     *              to order its work around this call.
     * @return An invalid handle when the store is out of slots.
     */
    AttributeObject create_object(LayerId layer = kInvalidLayer);

    /**
     * @brief Destroy an object, its attributes and its membership
     *
     * @return false when the handle was already stale, or when destroying it
     *         would wrap the slot's generation back to 0. A wrapped generation
     *         lets a handle from the slot's first life validate against its
     *         2^32-th, so the store retires such a slot; this document refuses
     *         the destroy instead, which leaves the object alive and leaks one
     *         record rather than leaving the document in a state the loader
     *         cannot reproduce.
     */
    bool destroy_object(AttributeObject obj);

    /// Move an object to another layer. @return false when the handle is stale.
    bool set_object_layer(AttributeObject obj, LayerId layer);

    /**
     * @brief Which layer an object is in
     *
     * @return kInvalidLayer for a stale handle or an object in no layer. The id
     *         is returned verbatim even when that layer has since been deleted:
     *         A1 never reuses an id, so a dangling reference can never come to
     *         mean a different layer, and zeroing it would throw away
     *         information the session still has.
     */
    [[nodiscard]] LayerId object_layer(AttributeObject obj) const;

    /// object_layer() as the opaque handle AttributeStore::resolve() takes.
    /// The join is a plain cast, because kInvalidLayer and kNoLayer are both 0.
    [[nodiscard]] LayerRef object_layer_ref(AttributeObject obj) const {
        return static_cast<LayerRef>(object_layer(obj));
    }

    /// Live objects, in slot-index order. Stable across unrelated edits.
    [[nodiscard]] std::vector<AttributeObject> objects() const;

    [[nodiscard]] size_t object_count() const { return m_attributes.live_object_count(); }

    /// Slots ever allocated, live and dead. What the file's object table sizes.
    [[nodiscard]] size_t slot_count() const { return m_slots.size(); }

    // ── Dirty tracking ──────────────────────────────────────────────────────

    /**
     * @brief Has anything changed since the last save or load?
     *
     * Two counters, because two kinds of change exist and neither covers the
     * other. CommandStack::revision() counts edits that went through the history
     * -- and counts an UNDO as a change too, which is right: the question is
     * "does this differ from what was saved", not "how many edits happened". The
     * structural counter covers object create and destroy, which are not
     * commands; without it, creating an object and quitting would lose it with
     * no prompt.
     */
    [[nodiscard]] bool dirty() const {
        return m_history.revision() != m_saved_revision
               || m_structural_revision != m_saved_structural_revision;
    }

    /// Declare the current state saved. save_to_file() does this; a caller that
    /// writes the JSON somewhere else itself does it by hand.
    void mark_saved() {
        m_saved_revision = m_history.revision();
        m_saved_structural_revision = m_structural_revision;
    }

    // ── Serialisation ───────────────────────────────────────────────────────

    /**
     * @brief Render the document as `.stratum` JSON
     *
     * @param out Receives the JSON on success; untouched on failure.
     * @return A failure when the document holds a double that JSON cannot
     *         represent -- NaN or an infinity. nlohmann writes those as `null`,
     *         which reads back as 0.0, so saving one would quietly move a layer
     *         to the origin. The error names the field.
     *
     * @note Does NOT clear dirty(): a caller that renders the JSON has not
     *       necessarily written it anywhere. mark_saved() is the declaration.
     *
     * @note Not const, and it costs one layer id. LayerTree offers no way to
     *       READ its id counter without taking from it, and recomputing the
     *       counter from the surviving layers is precisely the bug described
     *       under invariant 3 above. So the save takes one id and writes THAT id
     *       as the loaded document's next: the taken id is never given to a
     *       layer, so a loaded document issuing it is correct, and it makes
     *       save -> load -> save byte-identical. The live tree skips it, which
     *       costs one id out of four billion, and A1 already says burning an id
     *       costs nothing.
     */
    [[nodiscard]] DocumentIoResult save_to_json(std::string& out);

    /**
     * @brief Replace this document's contents with what @p text describes
     *
     * All or nothing. On failure this document is EXACTLY as it was, down to its
     * undo history -- the file is parsed and validated into a staging document
     * first, and nothing here is touched until that has succeeded.
     *
     * On success the undo history is cleared (see the file comment) and the
     * document is not dirty.
     *
     * @warning Every LayerId, AttributeObject and AttributeKey obtained from this
     *          document before the call belongs to the old contents and must be
     *          re-acquired. The handles the FILE carries are restored exactly;
     *          handles a caller was holding are not, and the loaded allocators
     *          are restored precisely so that a forgotten one fails to validate
     *          instead of quietly naming something new.
     */
    [[nodiscard]] DocumentIoResult load_from_json(std::string_view text);

    /**
     * @brief Write the document to @p path, then mark it saved
     *
     * Writes a sibling temporary and renames it over @p path, so an interrupted
     * or failing write cannot leave a truncated `.stratum` where the user's only
     * copy of their work used to be. A failure leaves the existing file
     * untouched and does not mark the document saved.
     */
    [[nodiscard]] DocumentIoResult save_to_file(const std::filesystem::path& path);

    /// Read @p path and load it. Refuses a file over kMaxDocumentFileBytes
    /// before reading it. Failure leaves this document untouched.
    [[nodiscard]] DocumentIoResult load_from_file(const std::filesystem::path& path);

private:
    /**
     * @brief The document's mirror of one AttributeStore object slot
     *
     * The store owns the truth about generations but exposes no way to walk its
     * slots, and it does not know about layers at all. This mirror is what makes
     * the object table writable: it is maintained by create_object() and
     * destroy_object(), which is why object lifetime has to go through the
     * document rather than the store.
     */
    struct ObjectSlot {
        uint32_t generation = 0;  ///< Generation the slot is AT: the live handle's
                                  ///< generation when alive, the generation the
                                  ///< next create() would hand out when dead.
        bool alive = false;
        LayerId layer = kInvalidLayer;  ///< Meaningful only while alive.
    };

    /**
     * @brief The reader and the writer, which both need the slot mirror
     *
     * Both live entirely in document.cpp. They are friends rather than member
     * functions so that nlohmann's headers never reach this one: every file that
     * includes a scene header would otherwise pay for the JSON library, and the
     * format would leak into the interface of a class whose job is the scene.
     */
    friend struct DocumentWriter;
    friend struct DocumentLoader;

    /// Take over @p staging's contents. Clears the history FIRST, so commands
    /// pointing into the old tree and store are destroyed before either moves.
    void adopt(Document& staging);

    LayerTree m_layers;
    AttributeStore m_attributes;
    CommandStack m_history;

    /// Indexed by AttributeObject::index. Dense, and never shrinks -- the store's
    /// slots never do either.
    std::vector<ObjectSlot> m_slots;

    /// Bumped by object create and destroy, which are not commands. See dirty().
    uint64_t m_structural_revision = 0;

    uint64_t m_saved_revision = 0;
    uint64_t m_saved_structural_revision = 0;
};

} // namespace stratum::scene
