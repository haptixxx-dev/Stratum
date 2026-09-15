// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file layer.hpp
 * @brief The scene's layer tree: what every later feature filters, lists and saves
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why a tree, and why now
 *
 * The importer drops everything it parses into one undifferentiated pile. There
 * is no way to say "export only the roads", "hide the terrain while I work on
 * the blocks", or "these buildings came from a second extract and should move
 * ten metres east". Every one of those is a layer operation, and each of the
 * features that needs one -- export filtering (H3), the layer panel (K2),
 * attribute inheritance (A3), save and load (A2) -- would otherwise invent its
 * own half of the same structure.
 *
 * ### Handles, not pointers, and not an ECS
 *
 * A layer is addressed by a LayerId: a plain integer, matching the pattern the
 * renderer already uses for GPU resources. Ids are allocated from a counter that
 * only ever goes up and are **never reused**, which buys the two properties the
 * rest of the system leans on:
 *
 *   - A handle survives insertions and removals of unrelated layers. Nothing
 *     here shifts indices under a caller the way a `std::vector` position would.
 *   - A handle to a deleted layer is *detectable*: `contains()` is false and
 *     `find()` returns nullptr. It can never silently start naming some other
 *     layer, which is the failure a recycled slot produces and the reason a
 *     free-list of indices was rejected.
 *
 * EnTT is deliberately absent. It is linked into stratum_core and used by
 * nothing, and committing the scene model to an ECS before a single system
 * iterates components would buy nothing and make A2 harder: plain owning data
 * serialises as it stands.
 *
 * ### Own state versus effective state
 *
 * A layer inside a hidden group does not draw, however its own flag reads. The
 * same goes for the lock, and for the transform, which accumulates down the
 * chain. Getting this wrong is not a visible bug -- it is geometry that keeps
 * drawing after the user hid the group it is in -- so the API removes the chance
 * to get it wrong by making the two spellings impossible to confuse:
 *
 *   - `Layer::own_visible` and `Layer::own_locked` are the layer's own flags,
 *     and the `own_` prefix is in the field name, not just the documentation.
 *   - `LayerTree::effective_visible()` and `effective_locked()` answer the
 *     question a caller almost always means. They live on the TREE, because the
 *     answer depends on ancestors a Layer has no way to reach.
 *
 * There is deliberately no member called `visible` anywhere. A call site must
 * say which one it means.
 *
 * Visibility and lock compose *restrictively*: a hidden group hides its subtree
 * and no descendant can opt out. An override would mean a child escaping the
 * group it is in, which is exactly the bug a user reports as "I hid the group
 * and something is still drawing".
 *
 * ### Colour inherits, but by fallback
 *
 * CityEngine inherits layer colour down the hierarchy, and that is worth
 * keeping: colouring one group recolours everything in it, which is how a legend
 * is actually built. But unconditional inheritance would make a child's own
 * colour meaningless the moment anything is grouped. So the colour is OPTIONAL:
 * an absent colour means "take the nearest ancestor's", and an explicit one wins
 * for the layer and for everything under it that has not set its own.
 *
 * This is a different rule from visibility on purpose. A hidden group is a
 * statement about its contents; a colour is a label, and a label with no
 * explicit value resolves to the nearest one that has it.
 *
 * These colours are a display aid for the panel and the viewport overlay. They
 * are not materials and never reach an exported map.
 *
 * ### Every mutation is a Command
 *
 * LayerTree's mutating API is private. The only way in is through the command
 * classes at the bottom of this file, executed on a CommandStack. That is the
 * whole reason A4 landed first: a mutation that skips the stack is a hole in the
 * history, and the symptom shows up as an undo three steps later restoring state
 * that was never current.
 *
 * Two consequences worth knowing before writing a caller:
 *
 *   - **A create knows its id before it runs.** CreateLayerCommand reserves the
 *     id in its constructor, so `layer()` is readable the moment it is built --
 *     before `execute()` takes ownership of it. That is what lets a caller
 *     create a layer and immediately colour it inside one transaction.
 *   - **A command that would change nothing refuses.** Setting a flag to the
 *     value it already has records no step, because an undo menu that fills up
 *     with steps that do nothing reads to a user as undo being broken.
 */

#pragma once

#include "scene/command.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace stratum::scene {

// ============================================================================
// Handles
// ============================================================================

/// Stable, never-reused handle to a layer. Valid ids start at 1.
using LayerId = uint32_t;

/**
 * @brief Not a layer
 *
 * Doubles as "no parent", which is how a top-level layer spells its place in the
 * tree. There is no ambiguity in the overload: kInvalidLayer is never a valid
 * id, so `contains(kInvalidLayer)` is always false, while the sibling list it
 * names is the list of top-level layers returned by roots().
 */
inline constexpr LayerId kInvalidLayer = 0;

/**
 * @brief Position in a sibling list meaning "after the last sibling"
 *
 * Also what index_of() returns for a layer that has no position, i.e. one that
 * is not in the tree at all.
 */
inline constexpr size_t kAppend = static_cast<size_t>(-1);

// ============================================================================
// Layer kinds
// ============================================================================

/**
 * @brief What a layer holds
 *
 * A trimmed reading of CityEngine's taxonomy, kept to the kinds Stratum has
 * something to put in. Shape, Graph, Model, Map and Group map onto the importer,
 * the road solver, future model import, the terrain, and nesting respectively.
 * CityEngine's Analysis layer (viewsheds and the like) has no counterpart here
 * and is left out rather than reserved: an enumerator nothing produces is a case
 * every switch has to handle for nothing, and appending one later is cheap.
 *
 * @note Only Group may hold child layers. Reparenting under any other kind is
 *       refused -- see LayerTree::accepts_children().
 */
enum class LayerKind : uint8_t {
    Group,  ///< Nests other layers. The only kind that may have children.
    Shape,  ///< Footprints and areas: building outlines, land use, water.
    Graph,  ///< A street network and what is derived from it.
    Model,  ///< Imported geometry that no rule processes.
    Map     ///< Raster data that drives parameters. The terrain is one of these.
};

/// Number of LayerKind enumerators, for tables indexed by kind.
inline constexpr size_t kLayerKindCount = 5;

/// Stable spelling of a kind, for the panel and for A2. Never localised.
[[nodiscard]] const char* layer_kind_name(LayerKind kind);

/**
 * @brief Colour a layer shows when neither it nor any ancestor sets one
 *
 * A neutral grey on purpose: the default has to be the colour a user reads as
 * "nobody chose this", not one that looks deliberate.
 */
[[nodiscard]] glm::vec3 default_layer_colour();

// ============================================================================
// Transform
// ============================================================================

/**
 * @brief A layer's own offset from its parent's space
 *
 * Stored decomposed rather than as a matrix because that is what a panel edits
 * and what A2 writes: sixteen doubles nobody can read are a poor way to store
 * "moved ten metres east".
 *
 * Rotation is three Euler angles in RADIANS, applied Y then X then Z -- yaw,
 * pitch, roll, in a Y-up world. The order is fixed here and stated because the
 * alternative is every consumer picking its own and the geometry quietly
 * disagreeing between the viewport and the exporter.
 *
 * @note Composition through the hierarchy is NOT closed under this
 *       representation: a rotated parent with a non-uniformly scaled child
 *       produces shear, which no translation-rotation-scale triple can express.
 *       That is why LayerTree::effective_transform() returns a matrix and there
 *       is no "effective LayerTransform".
 */
struct LayerTransform {
    glm::dvec3 translation{0.0, 0.0, 0.0};  ///< Metres, in the parent's space
    glm::dvec3 rotation{0.0, 0.0, 0.0};     ///< Euler YXZ, radians
    glm::dvec3 scale{1.0, 1.0, 1.0};        ///< Per-axis multiplier

    /// True when this transform changes nothing, so composing it can be skipped.
    [[nodiscard]] bool is_identity() const;

    /// Translate * rotate(Y,X,Z) * scale, in that order.
    [[nodiscard]] glm::dmat4 matrix() const;
};

/// Exact componentwise equality. Answers "did the value change", not "are these
/// close" -- a tolerance here would swallow a deliberate one-micron nudge.
[[nodiscard]] bool operator==(const LayerTransform& a, const LayerTransform& b);
[[nodiscard]] bool operator!=(const LayerTransform& a, const LayerTransform& b);

// ============================================================================
// Layer
// ============================================================================

/**
 * @brief One node of the layer tree
 *
 * Plain data, copyable, and the unit A2 serialises. Both directions of the
 * parent link are stored: `parent` is what the effective-state walk climbs, and
 * `children` is what makes sibling ORDER explicit rather than an accident of
 * iteration. LayerTree is responsible for keeping the two in agreement, which is
 * the main reason its mutating API is not public.
 */
struct Layer {
    LayerId id = kInvalidLayer;              ///< Assigned on create, never reused
    LayerKind kind = LayerKind::Group;       ///< Fixed for the layer's life
    std::string name;                        ///< Free text. Not unique, may be empty.
    LayerId parent = kInvalidLayer;          ///< kInvalidLayer for a top-level layer
    std::vector<LayerId> children;           ///< Ordered; empty unless kind is Group

    /// This layer's own flag. Hidden ancestors override it -- see
    /// LayerTree::effective_visible(), which is what a renderer should ask.
    bool own_visible = true;

    /// This layer's own flag. A locked ancestor overrides it -- see
    /// LayerTree::effective_locked(), which is what an edit tool should ask.
    bool own_locked = false;

    /// Absent means "inherit from the nearest ancestor that set one".
    std::optional<glm::vec3> own_colour;

    /// Relative to the parent's space, not to the world.
    LayerTransform own_transform;
};

// ============================================================================
// The tree
// ============================================================================

class LayerCommand;

/**
 * @brief The scene's layers, their nesting and their sibling order
 *
 * Queries are public; every mutation is private and reachable only through a
 * Command. Not thread-safe, for the same reason CommandStack is not: edits come
 * from the UI thread and a background job hands a command back rather than
 * pushing from its own.
 */
class LayerTree {
public:
    LayerTree() = default;

    // ── Lookup ──────────────────────────────────────────────────────────────

    /// True when @p id names a layer that is currently in the tree.
    [[nodiscard]] bool contains(LayerId id) const;

    /**
     * @brief Look a layer up
     *
     * @return nullptr when @p id is stale or was never issued. Because ids are
     *         never reused, a non-null result is always the layer the caller
     *         meant, never a different one that took over the slot.
     *
     * @note The returned pointer stays valid across insertions and removals of
     *       OTHER layers -- storage is node-based -- but not across the removal
     *       of this one. Hold the LayerId, not the pointer.
     */
    [[nodiscard]] const Layer* find(LayerId id) const;

    [[nodiscard]] size_t size() const { return m_layers.size(); }
    [[nodiscard]] bool empty() const { return m_layers.empty(); }

    // ── Structure ───────────────────────────────────────────────────────────

    /// Top-level layers, in order.
    [[nodiscard]] const std::vector<LayerId>& roots() const { return m_roots; }

    /**
     * @brief Ordered children of @p parent
     *
     * @param parent kInvalidLayer for the top level, which returns roots().
     * @return An empty list for a stale id or a kind that cannot hold children.
     */
    [[nodiscard]] const std::vector<LayerId>& children(LayerId parent) const;

    /// Position of @p id among its siblings, or kAppend when it has no position.
    [[nodiscard]] size_t index_of(LayerId id) const;

    /// Number of ancestors above @p id. 0 for a top-level layer.
    [[nodiscard]] size_t depth(LayerId id) const;

    /**
     * @brief Is @p ancestor above @p descendant in the tree?
     *
     * Strict: a layer is not its own ancestor. kInvalidLayer is not a layer and
     * is never an ancestor, even though it names the top-level list.
     */
    [[nodiscard]] bool is_ancestor_of(LayerId ancestor, LayerId descendant) const;

    /// @p root and everything under it, pre-order, children in sibling order.
    [[nodiscard]] std::vector<LayerId> subtree_ids(LayerId root) const;

    /// True when @p parent may hold children: the top level, or an existing Group.
    [[nodiscard]] bool accepts_children(LayerId parent) const;

    /**
     * @brief Would a reparent of @p id under @p new_parent be allowed?
     *
     * Public so a drag can grey out an illegal drop target while the pointer is
     * still moving, rather than only refusing on release. ReparentLayerCommand
     * asks the same question, so the two can never disagree.
     *
     * False when @p id is stale, when @p new_parent cannot hold children, or
     * when the move would make @p id its own ancestor.
     */
    [[nodiscard]] bool can_reparent(LayerId id, LayerId new_parent) const;

    // ── Effective state ─────────────────────────────────────────────────────

    /**
     * @brief Does this layer draw?
     *
     * True only when the layer and EVERY ancestor is visible. False for a stale
     * id, which is the safe answer: nothing draws.
     */
    [[nodiscard]] bool effective_visible(LayerId id) const;

    /**
     * @brief Is this layer protected from editing?
     *
     * True when the layer OR ANY ancestor is locked. True for a stale id, again
     * the safe answer: an edit tool refuses rather than writing into nothing.
     */
    [[nodiscard]] bool effective_locked(LayerId id) const;

    /// Own colour, else the nearest ancestor's, else default_layer_colour().
    [[nodiscard]] glm::vec3 effective_colour(LayerId id) const;

    /// Every ancestor transform composed with this one, top-down. Identity for a
    /// stale id.
    [[nodiscard]] glm::dmat4 effective_transform(LayerId id) const;

private:
    /**
     * @brief The one door into the mutating API
     *
     * LayerCommand forwards these to its subclasses as protected helpers, so
     * "every mutation goes through the command stack" is enforced by the
     * compiler in exactly one place instead of being a rule in a comment.
     */
    friend class LayerCommand;

    /// Take the next id. Monotonic; never returns a value it returned before.
    LayerId reserve_id();

    /// Insert a fully-formed layer at @p index of its parent's list.
    bool insert(Layer layer, size_t index);

    /**
     * @brief Remove @p root and everything under it
     *
     * @param parent_out Receives the parent the subtree hung from
     * @param index_out  Receives the position it occupied there
     * @return The removed layers, pre-order, verbatim -- enough for attach() to
     *         reproduce the subtree exactly, handles included.
     */
    std::vector<Layer> detach(LayerId root, LayerId* parent_out, size_t* index_out);

    /// Put back what detach() returned, at @p index of the recorded parent.
    bool attach(const std::vector<Layer>& subtree, size_t index);

    /// Reparent and reposition in one step. Also covers a same-parent reorder.
    bool move(LayerId id, LayerId new_parent, size_t index);

    Layer* mutable_layer(LayerId id);

    std::vector<LayerId>& sibling_list(LayerId parent);

    /**
     * @brief @p id then each ancestor, nearest first; empty when @p id is stale
     *
     * Every walk up the tree goes through here so that the guard against a
     * corrupt tree lives in one place. It allocates, which is the deliberate
     * trade: these run per layer in a panel of tens or hundreds, not per vertex.
     */
    [[nodiscard]] std::vector<LayerId> self_and_ancestors(LayerId id) const;

    /// Node-based, so a Layer* survives unrelated edits, and ordered by id, so
    /// iteration is creation order rather than a hash's whim.
    std::map<LayerId, Layer> m_layers;

    std::vector<LayerId> m_roots;

    /// Never decreases, not even when the layer it was issued for is deleted.
    LayerId m_next_id = 1;
};

// ============================================================================
// Commands
// ============================================================================

/**
 * @brief Base for every layer edit
 *
 * Holds the target and owns the forwarding into LayerTree's private API. A new
 * layer operation subclasses this and uses the protected helpers; it does not
 * touch LayerTree directly, and it cannot -- LayerCommand is the tree's only
 * friend.
 */
class LayerCommand : public Command {
public:
    /**
     * @brief The layer this command acts on
     *
     * Readable before execute(), including on a create, whose id is reserved in
     * the constructor.
     */
    [[nodiscard]] LayerId layer() const { return m_layer; }

protected:
    LayerCommand(LayerTree& tree, LayerId layer) : m_tree(&tree), m_layer(layer) {}

    [[nodiscard]] LayerTree& tree() const { return *m_tree; }

    /// True when @p other edits the same layer of the same tree. The guard every
    /// merge() needs: without the tree check, two documents open at once would
    /// coalesce each other's edits.
    [[nodiscard]] bool same_target(const LayerCommand& other) const {
        return m_tree == other.m_tree && m_layer == other.m_layer;
    }

    // Forwarders into LayerTree. See the matching private members there.

    /// Static, and takes the tree explicitly, because CreateLayerCommand calls
    /// it from its own member-initialiser list -- before there is a LayerCommand
    /// to call it on, and from a scope where LayerTree's own friendship with
    /// LayerCommand does not reach.
    static LayerId reserve_id(LayerTree& tree);

    bool insert_layer(Layer layer, size_t index);
    std::vector<Layer> detach_subtree(LayerId root, LayerId* parent_out, size_t* index_out);
    bool attach_subtree(const std::vector<Layer>& subtree, size_t index);
    bool move_layer(LayerId id, LayerId new_parent, size_t index);

    /**
     * @brief Writable access to a layer's VALUE fields
     *
     * Name, flags, colour and transform only. Writing `parent` or `children`
     * through this pointer breaks the agreement between the two directions of
     * the link; structural edits go through insert_layer(), detach_subtree(),
     * attach_subtree() and move_layer(), which maintain both.
     *
     * @return nullptr when the layer is gone.
     */
    Layer* mutable_layer(LayerId id);

    LayerTree* m_tree;
    LayerId m_layer;
};

/**
 * @brief Add a layer
 *
 * The id is taken in the constructor, so a caller can configure the new layer in
 * the same transaction that creates it. A command that is built and then never
 * executed burns that id, which costs nothing: ids are 32 bits, monotonic, and
 * already never reused.
 */
class CreateLayerCommand : public LayerCommand {
public:
    /**
     * @param parent kInvalidLayer for a top-level layer, else a Group
     * @param index  Position among the new siblings; kAppend for last
     */
    CreateLayerCommand(LayerTree& tree, LayerKind kind, std::string name,
                       LayerId parent = kInvalidLayer, size_t index = kAppend);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;

private:
    LayerKind m_kind;
    std::string m_name;
    LayerId m_parent;

    /// kAppend until the first apply() resolves it, so a redo lands in the same
    /// slot rather than at the end of a list that has since grown.
    size_t m_index;
};

/**
 * @brief Remove a layer and everything under it
 *
 * A group takes its children with it. The alternative -- promoting them to the
 * grandparent -- rewrites a part of the tree the user did not point at, and
 * undoing it means restoring every promoted child's exact former position as
 * well as the group. "Delete the group but keep the contents" is a different
 * operation (ungroup) and should be written as one.
 *
 * Undo restores the subtree verbatim: the same handles, the same nesting, the
 * same sibling order, and the same position in the parent's list.
 */
class DeleteLayerCommand : public LayerCommand {
public:
    DeleteLayerCommand(LayerTree& tree, LayerId layer);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;

private:
    std::vector<Layer> m_subtree;  ///< Pre-order, root first. The undo state.
    LayerId m_parent = kInvalidLayer;
    size_t m_index = kAppend;
};

/// Change a layer's name. Merges with a later rename of the same layer, so
/// typing into the panel's field is one undo step and not one per keystroke.
class RenameLayerCommand : public LayerCommand {
public:
    RenameLayerCommand(LayerTree& tree, LayerId layer, std::string name);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;
    [[nodiscard]] bool merge(const Command& next) override;

private:
    std::string m_new_name;
    std::string m_old_name;
};

/**
 * @brief Move a layer to a different parent
 *
 * Refused when the new parent cannot hold children, and refused when the move
 * would make the layer its own ancestor. The cycle is caught BEFORE the tree is
 * touched: a cycle created and detected later is not a wrong picture, it is a
 * walk up the parent chain that never terminates.
 */
class ReparentLayerCommand : public LayerCommand {
public:
    ReparentLayerCommand(LayerTree& tree, LayerId layer, LayerId new_parent,
                         size_t index = kAppend);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

private:
    LayerId m_new_parent;
    size_t m_index;
    LayerId m_old_parent = kInvalidLayer;
    size_t m_old_index = kAppend;
};

/**
 * @brief Move a layer within its own sibling list
 *
 * @p index is the position the layer OCCUPIES AFTER the move, counted in the
 * list with the layer already taken out: moving A to index 2 in [A, B, C] gives
 * [B, C, A]. Anything past the end means last.
 */
class ReorderLayerCommand : public LayerCommand {
public:
    ReorderLayerCommand(LayerTree& tree, LayerId layer, size_t index);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

private:
    size_t m_index;
    size_t m_old_index = kAppend;
};

/// Show or hide a layer. Does NOT merge: each toggle is a deliberate click, and
/// collapsing two of them would hide a state the user chose to pass through.
class SetLayerVisibleCommand : public LayerCommand {
public:
    SetLayerVisibleCommand(LayerTree& tree, LayerId layer, bool visible);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

private:
    bool m_visible;
    bool m_old_visible = true;
};

/// Lock or unlock a layer. Does not merge, for the same reason as visibility.
class SetLayerLockedCommand : public LayerCommand {
public:
    SetLayerLockedCommand(LayerTree& tree, LayerId layer, bool locked);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

private:
    bool m_locked;
    bool m_old_locked = false;
};

/**
 * @brief Set or clear a layer's own colour
 *
 * std::nullopt clears it, which is not "make it grey": it puts the layer back to
 * inheriting from the nearest ancestor that has one.
 */
class SetLayerColourCommand : public LayerCommand {
public:
    SetLayerColourCommand(LayerTree& tree, LayerId layer, std::optional<glm::vec3> colour);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

private:
    std::optional<glm::vec3> m_colour;
    std::optional<glm::vec3> m_old_colour;
};

/// Set a layer's own transform. Merges with a later transform of the same layer,
/// so a gizmo drag is one undo step rather than one per mouse-move; seal() on
/// mouse-up ends it.
class SetLayerTransformCommand : public LayerCommand {
public:
    SetLayerTransformCommand(LayerTree& tree, LayerId layer, LayerTransform transform);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] bool merge(const Command& next) override;

private:
    LayerTransform m_transform;
    LayerTransform m_old_transform;
};

} // namespace stratum::scene
