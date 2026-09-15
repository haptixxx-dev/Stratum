// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file selection.hpp
 * @brief What the tools operate on: an ordered, self-validating set of layers and objects
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this is one type and not five
 *
 * Every editing tool in Tracks B, C, E and K starts the same way: "take what the
 * user picked and do something to it". The inspector (K1) shows it, the gizmo
 * moves it, the rule engine runs on it, a report aggregates over it, and export
 * filtering narrows it. Without one selection model each of those invents its
 * own container of handles, and the moment two of them disagree about what is
 * selected the user sees a gizmo on one building and an inspector showing
 * another. One model, one answer.
 *
 * ### Selection is NOT undoable, and that is a decision, not an omission
 *
 * **No operation on Selection ever records an undo step.** Ctrl-Z never changes
 * the selection by itself. The reasons, in order of weight:
 *
 *   - **Undo is for the document; selection is not in the document.** Nothing
 *     here changes what is exported, what is rendered, or what A2 writes as
 *     scene content. It is the editor's pointer into the scene. Putting a
 *     pointer on the history of the thing it points at is a category error.
 *   - **It destroys the history a user actually wants.** A click is a mutation
 *     of the selection. Recording clicks means ctrl-Z after "move the building,
 *     then click away" undoes the click. The user pressed ctrl-Z to get the
 *     building back, and now has to press it again -- and cannot tell how many
 *     more times. CityEngine does not do this. Blender, Maya and Houdini do not
 *     do this. Every tool that tried it has since added a preference to turn it
 *     off, which is the strongest evidence available that the default is wrong.
 *   - **Coalescing cannot rescue it.** A rubber band emits a selection change
 *     per mouse-move, so merging (command.hpp) would be mandatory, and even a
 *     perfect merge still leaves one step per gesture. The problem is not the
 *     number of steps; it is that selection steps are in the list at all.
 *
 * The case the brief raises -- a tool that selects as part of a larger
 * transaction -- is real, and is handled WITHOUT making selection undoable.
 * `snapshot()` and `restore()` let a command carry the selection in its own
 * inverse, exactly as DeleteLayerCommand carries the subtree it removed and as
 * A3's ObjectSnapshot carries an object's attributes. That keeps the rule whole
 * rather than half:
 *
 *   > The selection model never pushes a command. A command may own a selection
 *   > change as part of its own state, and that is the only route by which undo
 *   > ever moves the selection.
 *
 * A consequence worth stating, because it is the one people notice: deleting a
 * selected layer deselects it, and undoing the delete brings the layer back
 * **unselected**. That follows from the rule above and is consistent -- the undo
 * restored the document, and the selection is not the document. The alternative
 * (selection quietly reappearing) would mean the selection had history after
 * all, which is the half-way state this file refuses.
 *
 * ### Stale members: filter on the way out, refuse on the way in
 *
 * A selected layer gets deleted. A selected object is destroyed. Neither A1 nor
 * A3 has a notification hook, so the selection cannot be told; it has to check.
 * Three things make that safe and cheap:
 *
 *   - **Refuse on the way in.** add(), toggle() and restore() validate the
 *     handle and drop it if the scene does not have it. A dead handle never
 *     enters the selection in the first place.
 *   - **Check on the way out, per member.** Every accessor that HANDS BACK a
 *     member -- contains(), primary(), iteration, layers(), objects(), filter(),
 *     snapshot() -- tests that member's liveness as it goes. This is the hard
 *     guarantee, and it does not depend on any bookkeeping being up to date: a
 *     handle naming something gone is never handed to a caller, whatever else
 *     has happened. It costs a constant factor on a pass those accessors were
 *     making anyway, which is why iteration is an iterator that skips the dead
 *     rather than a reference to the underlying vector.
 *   - **Forget when the scene changed.** The counting accessors -- size() and
 *     count_of(); empty() is the exception and asks the members -- cannot
 *     afford a pass, so they lean on a change token instead: CommandStack::revision() (bumped by every mutation that
 *     goes through the stack, undo and redo included) together with the layer
 *     count and the live object count. When any of the three moves, the next
 *     read forgets the dead members: each one becomes a hole where it lies and
 *     leaves the index. That keeps the counts exact without asking the caller to
 *     remember anything, and it makes a death final, so an undo cannot bring a
 *     member back still selected.
 *
 *     Forgetting is all a read may do. It may not SQUEEZE -- moving survivors
 *     into lower slots, or popping the back -- because an outstanding iterator
 *     is a raw index into that vector, and a read that rearranges it under a
 *     running pass skips live members and walks off the end. That is a defect
 *     this file shipped with; the note above the storage has the detail, and the
 *     holes wait for the next mutation or prune().
 *
 * Neither handle scheme can alias, which is what makes the test a test rather
 * than a guess. A1's LayerIds are never reused, so a deleted layer's id stays
 * dead forever. A3's AttributeObject is an index plus a generation, so a handle
 * to a destroyed object fails validation even after its slot is recycled -- the
 * recycled object is a different generation and compares unequal. A selected
 * object that dies can therefore never come back as whatever was created in its
 * place, which is the failure this design exists to make impossible.
 *
 * The residue, stated plainly rather than papered over. There are two. Only the
 * first is about correctness, and neither ever hands out a dead handle:
 *
 *   - **A destroy the token cannot see.** `AttributeStore::destroy_object()` is
 *     not a command (see attributes.hpp), so the live object count is in the
 *     token precisely to catch it. The one sequence the token cannot see is a
 *     destroy and a create of another object between two reads with no command
 *     in between -- the count comes back level, so nothing forgets anything.
 *     size() and count_of() then read one too high until the next scene change
 *     or an explicit prune(). Closing it completely needs a notification hook in
 *     A3; inventing a second change counter here would only give two answers to
 *     "did the scene change".
 *   - **Holes a read is not allowed to squeeze.** Between a scene change and the
 *     next mutation or prune(), the members the scene took are holes in m_order
 *     rather than gone from it. Nothing reports them, but they are walked over:
 *     a pass, an empty() and a primary() each step across the holes in their way
 *     until something squeezes them out. A selection that loses most of its
 *     members and is then only READ keeps paying that until it is mutated or
 *     pruned. prune() is the one-line answer where it matters.
 *
 * What holds with no bookkeeping up to date and no prune() call, in every case
 * including the masked destroy above: contains(), primary(), iteration,
 * layers(), objects(), filter(), snapshot() and empty() are EXACT, because each
 * one tests the members it touches as it touches them. Only size() and
 * count_of() can read high, only in that one window, and only until something
 * notices any other scene change.
 *
 * ### Locked layers can be selected. They cannot be edited.
 *
 * A1's effective_locked() guards EDITS, and this file deliberately does not
 * repeat that guard at selection time:
 *
 *   - Selecting is how a user inspects. Someone who clicks a locked layer to
 *     find out why it will not move must be able to click it, or the question
 *     "why is this thing stuck" has no answer in the UI.
 *   - The guard has to be at the edit anyway. A layer can be locked *after* it
 *     is selected, so every mutating tool must re-check effective_locked()
 *     regardless. A second check at selection time is redundant, and redundant
 *     guards drift.
 *   - Lock is not selectability. Blender keeps those on two separate switches
 *     for exactly this reason; A1 has only the lock, and reading it as both
 *     would silently give the lock a second meaning nobody asked for.
 *
 * So a tool that must skip locked members filters for them rather than relying
 * on them being absent: `retain(is_unlocked(tree))` before a move, and the
 * inspector shows the locked ones greyed rather than not at all.
 *
 * ### A selection is a SEQUENCE of distinct members
 *
 * Ordered, and no duplicates. Both halves matter:
 *
 *   - **Ordered**, because "primary" has to mean something. The primary is the
 *     most recently added member: the inspector shows its fields when several
 *     are selected and only one set fits, and a two-object operation ("align to
 *     the active one") needs to know which one is the reference. A set has no
 *     most-recent. Insertion order is also the only DETERMINISTIC order
 *     available -- hash order is not reproducible, and a golden test or an A2
 *     save file that depends on it is a test that fails on a rebuild.
 *   - **Distinct**, because selecting the same building twice is not a state a
 *     user can mean, and a duplicate would make a report count it twice.
 *
 * Adding a member already present is a no-op and reports false: it does not move
 * it and does not change the primary. Promotion is its own operation,
 * make_primary(), because "shift-click an already-selected thing to make it
 * active" and "add to the selection" are different gestures and folding them
 * into one call makes the reorder invisible at the call site.
 *
 * ### Cost
 *
 * Storage is one vector in selection order plus one hash index from member to
 * position, which is what buys O(1) membership on a selection with hundreds of
 * thousands of members. Removals leave a hole rather than shifting the vector --
 * shifting would be O(n) MOVES plus O(n) index updates -- and the holes are
 * squeezed out in one pass once they outnumber the live members, which is
 * amortised O(1) per removal. That squeeze happens on a MUTATION and never on a
 * read, so primary() is a backward scan for the last live member rather than a
 * back() after a trim -- and it stops on the first slot it looks at unless the
 * scene has taken members off the tail since the last mutation.
 *
 * Per operation, with n members held:
 *
 *   | Operation                          | Cost                                |
 *   |------------------------------------|-------------------------------------|
 *   | add, remove, toggle, contains      | O(1) amortised                      |
 *   | size, count_of                     | O(1)                                |
 *   | primary, empty                     | O(1) + the holes in the way         |
 *   | make_primary                       | O(1) amortised                      |
 *   | begin/end, ++it                    | O(1) amortised per step, no alloc   |
 *   | layers, objects, snapshot          | O(n)                                |
 *   | clear                              | O(n)                                |
 *   | replace, add_all (m items)         | O(n + m)                            |
 *   | filter, retain                     | O(n) predicate calls                |
 *   | first read after a scene change    | O(n) liveness tests, once per change|
 *
 * A liveness test is an array index and two integer compares for an object, and
 * a std::map lookup for a layer. A select-all over a city extract is objects,
 * not layers -- layers number in the tens. Measured on a 500,000-member
 * selection of objects, release build: the select-all itself 41ms, a full
 * iteration 2.5ms, 250,000 individual removals 36ms in total (so ~145ns each,
 * with the compaction passes included), and the liveness pass
 * over the whole selection 8.6ms, paid once per scene change and only if
 * something reads the selection before the next one. size() and primary() do not
 * register, and stay that way as long as the holes that pass leaves behind are
 * squeezed out by a mutation or a prune() rather than left to be walked over.
 *
 * Memory is 16 bytes per member in the vector plus its index node, measured at
 * 19MB for that half-million-member selection -- about 40 bytes a member. That
 * is the price of O(1) membership, and it is why there is no second index.
 */

#pragma once

#include "scene/attributes.hpp"
#include "scene/command.hpp"
#include "scene/layer.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <unordered_map>
#include <vector>

namespace stratum::scene {

// ============================================================================
// Members
// ============================================================================

/**
 * @brief What kind of thing a selected member is
 *
 * Two, because the scene has exactly two addressable identities so far: A1's
 * layers and A3's attribute-bearing objects. A third (a road edge, a face) gets
 * an enumerator and a payload field when there is something to put in it;
 * reserving one now would give every switch a case that cannot happen.
 */
enum class SelectionKind : uint8_t {
    Layer,   ///< A node of the LayerTree, addressed by LayerId
    Object,  ///< An attribute record, addressed by AttributeObject
};

/// "Layer", "Object". Stable spelling, for logs and for the panel. Never localised.
[[nodiscard]] const char* selection_kind_name(SelectionKind kind);

/**
 * @brief One selected thing
 *
 * A tagged handle, built only through the two factories. The unused payload is
 * left at its invalid value so that equality and the hash can compare the whole
 * struct without a branch, and so that a Layer member and an Object member never
 * collide even when their numbers happen to match.
 *
 * A default-constructed item names nothing. It is what primary() returns for an
 * empty selection, and it is refused by every operation that takes one.
 */
struct SelectionItem {
    SelectionKind kind = SelectionKind::Layer;
    LayerId layer = kInvalidLayer;  ///< Meaningful when kind is Layer
    AttributeObject object{};       ///< Meaningful when kind is Object

    [[nodiscard]] static SelectionItem of_layer(LayerId id);
    [[nodiscard]] static SelectionItem of_object(AttributeObject obj);

    /**
     * @brief Does this name something?
     *
     * Says nothing about whether the scene still has it -- that is
     * Selection::is_live(). A syntactically valid handle to a deleted layer is
     * still valid() and is not live.
     */
    [[nodiscard]] bool valid() const;

    friend bool operator==(const SelectionItem& a, const SelectionItem& b) {
        return a.kind == b.kind && a.layer == b.layer && a.object == b.object;
    }
    friend bool operator!=(const SelectionItem& a, const SelectionItem& b) { return !(a == b); }
};

/**
 * @brief Hash for SelectionItem
 *
 * Named, and in this namespace, rather than only a std::hash specialisation:
 * Selection's own index is declared further down this same header, and a
 * std::hash specialisation written after that declaration is too late -- the
 * container instantiates the primary template first and the specialisation is
 * then ill-formed. std::hash<SelectionItem> is specialised at the bottom of the
 * file in terms of this, so a caller keying its own set does not have to know
 * the name.
 *
 * Every field goes in, the kind included. The payloads already separate the two
 * kinds without it -- a Layer member leaves object.index at its invalid
 * sentinel and an Object member leaves layer at kInvalidLayer, so layer 7 and
 * the object in slot 7 never present the same four fields. The kind is hashed
 * anyway, so that the separation survives an encoding that stops setting those
 * sentinels; nothing about the hash would announce the change.
 *
 * The GENERATION is the field that matters for correctness, not for speed: drop
 * it and a handle to a destroyed object hashes and compares equal to the object
 * that took over its slot, which is the aliasing the whole handle scheme exists
 * to prevent.
 */
struct SelectionItemHash {
    [[nodiscard]] std::size_t operator()(const SelectionItem& item) const noexcept {
        // FNV-1a over the four fields. Cheap, and it spreads the low bits that
        // monotonic layer ids and dense slot indices otherwise crowd into: a
        // select-all mints members 1..n, which an identity hash would lay down
        // in one contiguous run of buckets.
        constexpr std::uint64_t kPrime = 1099511628211ull;
        std::uint64_t hash = 14695981039346656037ull;
        hash = (hash ^ static_cast<std::uint64_t>(item.kind)) * kPrime;
        hash = (hash ^ static_cast<std::uint64_t>(item.layer)) * kPrime;
        hash = (hash ^ static_cast<std::uint64_t>(item.object.index)) * kPrime;
        hash = (hash ^ static_cast<std::uint64_t>(item.object.generation)) * kPrime;
        return static_cast<std::size_t>(hash);
    }
};

/**
 * @brief The layer half of the A1/A3 join, spelled once
 *
 * attributes.hpp fixes the rule: a LayerRef is `static_cast<LayerRef>(layer_id)`
 * and needs no special case because kNoLayer and kInvalidLayer are both 0.
 * Written here so that the cast appears in one place rather than at every
 * resolve() call in this file.
 */
[[nodiscard]] LayerRef layer_ref(LayerId id);

/**
 * @brief The reverse of layer_ref()
 *
 * @return kInvalidLayer when @p ref is kNoLayer, or when it does not fit a
 *         LayerId. The range check is not theatre: LayerRef is 64 bits
 *         specifically so a later A1 can pack a generation into it, and a
 *         silent truncation would turn that into a wrong layer rather than no
 *         layer.
 */
[[nodiscard]] LayerId layer_id_from_ref(LayerRef ref);

// ============================================================================
// Predicates
// ============================================================================

/// Tests one member. Used by Selection::filter() and Selection::retain().
using SelectionPredicate = std::function<bool(const SelectionItem&)>;

/**
 * @brief Tests one resolved attribute
 *
 * Takes the whole AttributeQuery, not just the value, because the interesting
 * questions include "is this missing", "is this the wrong type" and "did this
 * come from the layer rather than the object" -- and a predicate handed only a
 * value cannot ask any of them. See the warning on AttributeQuery: test ok(),
 * not `value != nullptr`.
 */
using AttributePredicate = std::function<bool(const AttributeQuery&)>;

/**
 * @brief Which layer an object inherits attributes from
 *
 * Layer MEMBERSHIP belongs to the document, not to A3 and not to this file --
 * see the closing section of attributes.hpp. So an attribute filter that has to
 * resolve through inheritance asks the document, through this, rather than
 * keeping a second copy of membership that would drift from the first.
 *
 * An empty function means "no layer": objects then resolve from their own slots
 * and the declared defaults only, which is a correct answer and a quieter one
 * than guessing.
 */
using ObjectLayerFn = std::function<LayerRef(AttributeObject)>;

/// Keeps members of one kind. The coarse "filter by type".
[[nodiscard]] SelectionPredicate is_kind(SelectionKind kind);

/**
 * @brief Keeps layer members whose LayerKind matches. The fine "filter by type".
 *
 * Object members never match: an A3 object has no type of its own, and saying
 * "this object is a Shape" would invent one. Filtering objects by type is
 * filtering them by an attribute -- see Selection::attribute_predicate().
 *
 * @param layers Must outlive the returned predicate.
 */
[[nodiscard]] SelectionPredicate is_layer_kind(const LayerTree& layers, LayerKind kind);

/**
 * @brief Keeps members no lock protects, using A1's effective_locked()
 *
 * What a move or a delete tool narrows the selection with. An object is judged
 * by the layer @p object_layer puts it in; with no function, an object is in no
 * layer and nothing locks it.
 *
 * @param layers Must outlive the returned predicate.
 */
[[nodiscard]] SelectionPredicate is_unlocked(const LayerTree& layers,
                                             ObjectLayerFn object_layer = {});

// ============================================================================
// Snapshot
// ============================================================================

/**
 * @brief A selection, copied out so something can put it back
 *
 * The state a command carries when it owns a selection change as part of its
 * own inverse -- the escape hatch described in the file comment, and the same
 * shape as A3's ObjectSnapshot. Restoring validates, so a snapshot taken before
 * a delete puts back only what still exists.
 *
 * `items` is in selection order, so the last element is the primary.
 */
struct SelectionSnapshot {
    std::vector<SelectionItem> items;

    [[nodiscard]] bool empty() const { return items.empty(); }

    /// Bytes this holds, for Command::footprint().
    [[nodiscard]] size_t footprint() const;
};

// ============================================================================
// Selection
// ============================================================================

/**
 * @brief The editor's set of picked layers and objects
 *
 * Bound at construction to the three things it has to consult, all held by
 * pointer and none owned. They are the document's, and the document outlives its
 * selection.
 *
 * Not thread-safe, for the same reason CommandStack and LayerTree are not: the
 * selection changes on the UI thread, and a background job that wants to select
 * something hands the request back to that thread.
 */
class Selection {
public:
    /**
     * @param layers  Authority on whether a selected layer still exists
     * @param objects Authority on whether a selected object still exists
     * @param history Read for revision() only -- the change token that tells the
     *                selection when its cached liveness is worth re-checking.
     *                Nothing here ever pushes a command onto it.
     */
    Selection(const LayerTree& layers, const AttributeStore& objects, const CommandStack& history);

    // ── Mutation ────────────────────────────────────────────────────────────
    //
    // None of these records an undo step. See the file comment.

    /**
     * @brief Add one member. The shift-click.
     *
     * @return false when @p item names nothing, when the scene no longer has it,
     *         or when it is already selected. An already-selected member keeps
     *         its position and the primary does not move -- use make_primary()
     *         to promote it.
     */
    bool add(SelectionItem item);

    /**
     * @brief Drop one member
     *
     * @return false when the item was not selected. A held member the scene has
     *         since destroyed is dropped too, and reports false, because
     *         contains() already says it is not selected and the two must agree.
     */
    bool remove(SelectionItem item);

    /**
     * @brief Add when absent, drop when present. The ctrl-click.
     *
     * @return whether the item is selected AFTER the call. An item the scene
     *         does not have cannot be selected, so it reports false rather than
     *         reporting a toggle that did not happen.
     */
    bool toggle(SelectionItem item);

    /// Clear, then select one member. The plain click. @return add()'s answer.
    bool replace(SelectionItem item);

    /**
     * @brief Clear, then select these, in the order given. The rubber band.
     *
     * @return how many were selected. Members the scene does not have are
     *         skipped, so a short count is the caller's signal that its picking
     *         list was stale.
     */
    size_t replace(const std::vector<SelectionItem>& items);

    /// Add these on top of what is selected, in order. The shift-rubber-band.
    /// @return how many were newly selected.
    size_t add_all(const std::vector<SelectionItem>& items);

    /**
     * @brief Move an already-selected member to the end, making it the primary
     *
     * @return false when @p item is not selected. Selecting it is add()'s job:
     *         a promote that silently selects would make "make this the active
     *         one" and "add this" the same call, which is the conflation the
     *         file comment rejects.
     */
    bool make_primary(SelectionItem item);

    /// Select nothing. O(n) in members held; releases the storage.
    void clear();

    /**
     * @brief Put back a snapshot, in its recorded order
     *
     * Validated: members the scene no longer has are skipped, so a command
     * restoring a selection across a delete cannot resurrect a dead handle.
     *
     * @return how many were selected.
     */
    size_t restore(const SelectionSnapshot& snapshot);

    // ── Query ───────────────────────────────────────────────────────────────

    /// True when @p item is selected AND the scene still has it.
    [[nodiscard]] bool contains(SelectionItem item) const;

    /**
     * @brief How many members are selected
     *
     * O(1): the count is maintained, not counted. It reads HIGH in one window
     * only -- the masked destroy in the residue at the end of the staleness
     * section of the file comment, where the change token never moves, so
     * nothing forgets anything. prune() closes that window, any other scene
     * change closes it, and empty() never has it open.
     */
    [[nodiscard]] size_t size() const;

    /// True when nothing live is selected. Exact in EVERY case, size()'s residue
    /// window included, because it asks the members rather than the count: it is
    /// begin() == end(), and the iterator tests each member it lands on.
    [[nodiscard]] bool empty() const;

    /// Members held of one kind. O(1): the two counts are maintained, not
    /// counted. Same one stale window as size(), for the same reason.
    [[nodiscard]] size_t count_of(SelectionKind kind) const;

    /**
     * @brief The most recently added member, or a default item when empty
     *
     * What an inspector shows when several things are selected and only one set
     * of fields fits, and what an "align to the active one" operation uses as
     * its reference.
     *
     * Exact: the answer is the last member the scene still has. O(1) unless the
     * scene has taken members off the tail since the last MUTATION, in which
     * case it scans back over the holes they left -- it does not pop them,
     * because popping is a const accessor changing the length. See the note
     * above the storage.
     */
    [[nodiscard]] SelectionItem primary() const;

    /**
     * @brief Walks the live members in selection order, skipping everything else
     *
     * A forward iterator rather than a reference to the storage, for two
     * reasons that both matter more than the extra code:
     *
     *   - It is the only iteration that is EXACT. It tests each member's
     *     liveness as it arrives, so it cannot hand back a handle to something
     *     the scene destroyed, whatever the change token knows. A reference to
     *     the underlying vector could not do that without an O(n) pass first.
     *   - It allocates nothing. A tool highlighting a half-million-member
     *     selection every frame walks this; snapshot() is for when a copy is
     *     genuinely wanted.
     *
     * Invalidated by any MUTATION of the selection -- add(), remove(), toggle(),
     * make_primary(), replace(), restore(), retain(), clear(), prune() -- exactly
     * as a vector's iterator is, and for the same reason: those are the calls
     * that move the storage.
     *
     * It is NOT invalidated by a const read of the selection, nor by a change to
     * the SCENE. A read may only forget a dead member where it lies, so a pass
     * may ask size() or primary() as it goes, and the scene may lose a layer or
     * an object as it goes: every member the scene still has is visited exactly
     * once, in selection order, and a member that dies is skipped when the pass
     * reaches it. That is a repair and not a refinement -- reads used to squeeze
     * the storage, which both skipped live members and walked off the end.
     *
     * One honest edge, since no design can do better: if the member the cursor
     * is PARKED on is the one that dies, the next read turns that slot into a
     * hole, and the reference the caller is holding reads as an invalid item.
     * It is invalid rather than a handle to something destroyed, which is the
     * safer of the two and the one a caller can test for.
     */
    class const_iterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = SelectionItem;
        using difference_type = std::ptrdiff_t;
        using pointer = const SelectionItem*;
        using reference = const SelectionItem&;

        const_iterator() = default;

        [[nodiscard]] reference operator*() const { return m_selection->m_order[m_position]; }
        [[nodiscard]] pointer operator->() const { return &m_selection->m_order[m_position]; }

        const_iterator& operator++() {
            // Advancing the end is a no-op rather than a wrap. kEnd is npos, and
            // npos + 1 is 0, which would silently restart the pass at the front.
            if (at_end()) return *this;
            ++m_position;
            skip_gaps();
            return *this;
        }

        const_iterator operator++(int) {
            const_iterator previous = *this;
            ++*this;
            return previous;
        }

        /**
         * @brief Same selection, same place
         *
         * The selection is half of the identity. Comparing positions alone made
         * every one-member selection's begin() equal to every other's, and made
         * a default-constructed iterator equal to the beginning of anything --
         * so a loop accidentally bounded by another selection's end() stopped at
         * the wrong length instead of failing.
         *
         * Every end is the same end: at_end() re-reads the size rather than
         * trusting one recorded when end() was called, so a cursor that runs off
         * a shortened vector compares equal to end() instead of running past it.
         */
        friend bool operator==(const const_iterator& a, const const_iterator& b) {
            if (a.m_selection != b.m_selection) return false;
            if (a.at_end() || b.at_end()) return a.at_end() && b.at_end();
            return a.m_position == b.m_position;
        }
        friend bool operator!=(const const_iterator& a, const const_iterator& b) {
            return !(a == b);
        }

    private:
        friend class Selection;

        /// end()'s position. A sentinel, not a length; see Selection::end().
        static constexpr size_t kEnd = static_cast<size_t>(-1);

        const_iterator(const Selection* selection, size_t position)
            : m_selection(selection), m_position(position) {
            skip_gaps();
        }

        /// Past the last slot, or naming no selection at all. Asks the vector
        /// its CURRENT size every time, which is what keeps end() from going
        /// stale under a mutation.
        [[nodiscard]] bool at_end() const {
            return m_selection == nullptr || m_position >= m_selection->m_order.size();
        }

        /// Advance past holes and past members the scene no longer has, stopping
        /// at the end. This is where the per-member liveness test lives.
        void skip_gaps();

        const Selection* m_selection = nullptr;
        size_t m_position = 0;
    };

    [[nodiscard]] const_iterator begin() const;
    [[nodiscard]] const_iterator end() const;

    /// The layer members, in selection order. The list a layer panel highlights.
    [[nodiscard]] std::vector<LayerId> layers() const;

    /// The object members, in selection order. The list a rule or a report runs over.
    [[nodiscard]] std::vector<AttributeObject> objects() const;

    /// Copy the live members out, in order. See SelectionSnapshot.
    [[nodiscard]] SelectionSnapshot snapshot() const;

    // ── Filtering ───────────────────────────────────────────────────────────

    /// Live members satisfying @p predicate, in selection order. The selection
    /// is unchanged -- this answers "which of these", not "narrow to these".
    /// A null predicate matches NOTHING, so it answers with an empty list; see
    /// retain(), which reads it the same way.
    [[nodiscard]] std::vector<SelectionItem> filter(const SelectionPredicate& predicate) const;

    /**
     * @brief Narrow the selection to the members satisfying @p predicate
     *
     * The filter UI's operation, and the one a tool uses to drop what it may not
     * touch. Order among the survivors is preserved, so the primary only changes
     * when the primary itself was dropped.
     *
     * A null predicate matches NOTHING, so it drops every member -- the same
     * reading filter() gives it, and reachable without anyone writing
     * `SelectionPredicate{}`: attribute_predicate(key, {}) answers false for
     * every member. The other reading ("an unset filter keeps everything") is
     * also the dangerous one for a tool that then acts on what survived.
     *
     * @return how many members were dropped.
     */
    size_t retain(const SelectionPredicate& predicate);

    /**
     * @brief A predicate over one attribute, resolved through A3
     *
     * The "filter by attribute" half of the feature, built as a SelectionPredicate
     * so that filter() and retain() both take it and there is one code path
     * rather than two.
     *
     * Resolution follows attributes.hpp exactly:
     *   - An OBJECT member resolves with the layer @p object_layer names, so an
     *     attribute inherited from the layer counts -- which is the whole point
     *     of A3's chain, and the reason this takes a layer function rather than
     *     quietly passing kNoLayer.
     *   - A LAYER member resolves with a default-constructed AttributeObject,
     *     which attributes.hpp defines as "what the layer alone would give": its
     *     own value, else the declared default.
     *
     * @param key         Interned name. An invalid key resolves to Missing for
     *                    every member, so the predicate still sees a query and
     *                    decides for itself rather than the filter guessing.
     * @param value_test  What counts as a match.
     * @param object_layer Object-to-layer map; empty means objects are in no layer.
     *
     * @warning The returned predicate holds this selection's AttributeStore by
     *          pointer. It is valid for as long as the store is.
     */
    [[nodiscard]] SelectionPredicate attribute_predicate(AttributeKey key,
                                                         AttributePredicate value_test,
                                                         ObjectLayerFn object_layer = {}) const;

    // ── Staleness ───────────────────────────────────────────────────────────

    /// Does the scene still have @p item? False for an item naming nothing.
    [[nodiscard]] bool is_live(SelectionItem item) const;

    /**
     * @brief Drop every member the scene no longer has, now
     *
     * Ordinarily unnecessary -- the counting accessors forget a dead member when
     * the change token moves, and the rest check each member as they hand it
     * back. It exists for the two residues in the file comment, one per half of
     * what it does:
     *
     *   - It drops what the token cannot see: an object destroyed and another
     *     created with no read in between, which leaves the live object count
     *     level, so nothing else will ever notice.
     *   - It SQUEEZES, which no read may do. A selection that lost most of its
     *     members to the scene and has only been read since is still walking
     *     over their holes. One call puts that right; the next mutation would
     *     have done it anyway.
     *
     * Invalidates outstanding iterators, as every mutating operation does.
     *
     * @return how many members were dropped. A read that already forgot them
     *         counted them out first, so this reports 0 for those.
     */
    size_t prune();

    /// Bytes this selection holds. For a command carrying one, and for the panel.
    [[nodiscard]] size_t footprint() const;

private:
    /// Has the scene moved since the last check? Three integer compares, and no
    /// opinion about what to do next.
    [[nodiscard]] bool scene_changed() const;

    /**
     * @brief Forget what the scene has taken, when it has taken anything
     *
     * Three integer compares when it has not, which is every read inside one
     * frame. Const, because forgetting a dead member does not change what is
     * selected -- contains() already answered false for it -- and because a
     * death a reader has observed has to be final even if that reader is only
     * looking. See the note above the storage for the line this must not cross.
     */
    void ensure_fresh() const;

    /**
     * @brief Drop every member the scene no longer has, IN PLACE
     *
     * Each dead member becomes a hole where it lies and leaves the index. No
     * survivor moves, the length does not change, and nothing is reallocated,
     * so an outstanding const_iterator keeps naming the same member it named
     * before. That is the difference between this and rebuild(), and it is the
     * whole reason both exist.
     */
    void forget_dead() const;

    /**
     * @brief One pass: forget the dead AND squeeze the holes out
     *
     * Non-const, and not an accident: it moves survivors into lower slots, which
     * is what an outstanding iterator cannot survive. Only a mutating operation
     * and prune() may call it, and those invalidate the caller's iterators
     * anyway.
     *
     * @return how many LIVE members were dropped, so prune() can report it
     *         without a second pass. Holes are not members and are not counted.
     */
    size_t rebuild();

    /// Pop trailing holes and dead members. Non-const for rebuild()'s reason: it
    /// shortens the vector. An economy for the mutating paths and not an
    /// invariant -- primary() scans back and does not rely on it.
    void trim_tail();

    /// Tombstone the member at @p position and forget it. Does NOT trim or
    /// compact, which is what makes it safe from a const path.
    void drop_at(size_t position) const;

    /// Squeeze the holes out once they outnumber the live members. Amortised
    /// O(1) per removal: reaching the threshold costs that many removals.
    void compact_if_sparse();

    const LayerTree* m_layers;
    const AttributeStore* m_objects;
    const CommandStack* m_history;

    // The storage below is mutable because liveness is a fact about the SCENE,
    // not about the selection: a const read has to be allowed to notice that a
    // member died and stop reporting it, and that noticing has to be FINAL --
    // otherwise deleting a selected layer and undoing the delete would bring it
    // back still selected, and the selection would have the history the opening
    // section of this file refuses. `const` here means "does not change what is
    // selected", which is the guarantee a caller actually wants.
    //
    // There is one line inside that, and it is not a style rule; it is the fix
    // for a defect this file shipped with:
    //
    //   > A const member function may FORGET a dead member where it lies. It may
    //   > never move a survivor and never change the length.
    //
    // const_iterator is a raw index into m_order. The const accessors used to
    // call rebuild(), which compacts survivors DOWN into lower slots, and
    // trim_tail(), which pops the back. So primary() or size() called from
    // inside a range-for rearranged the vector under the cursor: live members
    // were moved past it and never visited, and the loop ran on to the length
    // end() had recorded and read slots that were no longer there -- handing
    // back precisely the destroyed handles this design exists to keep out of a
    // caller's hands. What a const read does now is forget_dead(), which writes
    // a hole over a dead member in place: every slot keeps its number, the
    // length does not move, and a pass already running skips the hole exactly as
    // it was going to skip the dead member.
    //
    // rebuild(), trim_tail() and compact_if_sparse() are non-const so that the
    // compiler keeps that line for us: the operations that MOVE the storage
    // cannot be called from a const accessor at all. The holes they would have
    // squeezed out wait for the next mutation or prune().

    /// Selection order. May contain holes -- a default-constructed, invalid item
    /// -- and, until the first read after a scene change, members the scene no
    /// longer has. Every reader tests what it lands on, so neither is reported.
    mutable std::vector<SelectionItem> m_order;

    using IndexMap = std::unordered_map<SelectionItem, size_t, SelectionItemHash>;

    /// Member to its position in m_order. Its size is the live member count --
    /// exact once ensure_fresh() has run, and high only in the masked-destroy
    /// window of the file comment's residue. `m_order.size() != m_index.size()`
    /// is exactly "there are holes".
    mutable IndexMap m_index;

    /// The same count, split by kind. They add up to m_index.size() and go stale
    /// only where it does.
    mutable size_t m_layer_count = 0;
    mutable size_t m_object_count = 0;

    // The change token, as of the last sweep. Three parts because no one of them
    // covers the scene on its own: the revision misses an object destroyed
    // outside a command, and the two counts miss a delete that another create
    // masks. None of these is read as a quantity -- only compared for change.

    /// No hint recorded. npos rather than 0, so an empty selection does not read
    /// as "the primary is slot 0".
    static constexpr size_t kNoPrimaryHint = static_cast<size_t>(-1);

    /// Last slot primary() found a live member in.
    ///
    /// A starting point, never an answer. primary() re-checks it against the
    /// live scene on every call and falls through to a full scan when it fails,
    /// which is what makes it safe to update from a const method: a stale hint
    /// costs one failed test, never a wrong result.
    ///
    /// It exists because the scan alone is quadratic in the state an editor
    /// spends most of its time in. Delete one of a thousand selected layers and
    /// the tail fills with holes; primary() then walks past every one, and the
    /// loop it exists for asks once per member.
    mutable size_t m_primary_hint = kNoPrimaryHint;

    mutable uint64_t m_checked_revision = 0;
    mutable size_t m_checked_layers = 0;
    mutable size_t m_checked_objects = 0;
};

} // namespace stratum::scene

/// Hash for SelectionItem, so callers can key their own sets and maps by it.
/// Delegates to SelectionItemHash; see the note there for why both exist.
template <>
struct std::hash<stratum::scene::SelectionItem> {
    [[nodiscard]] std::size_t operator()(const stratum::scene::SelectionItem& item) const noexcept {
        return stratum::scene::SelectionItemHash{}(item);
    }
};
