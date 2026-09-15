// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "scene/layer.hpp"

#include <spdlog/spdlog.h>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <utility>

namespace stratum::scene {

namespace {

/// Spelling of each LayerKind, indexed by the enumerator.
constexpr std::array<const char*, kLayerKindCount> kKindNames = {
    "Group", "Shape", "Graph", "Model", "Map"};

/// Totality check for the table above. Adding an enumerator means bumping
/// kLayerKindCount, which lengthens the array and leaves the new slot null --
/// so this fails the BUILD rather than letting the panel print nothing.
constexpr bool every_kind_named(const std::array<const char*, kLayerKindCount>& names) {
    for (const char* name : names) {
        if (name == nullptr) return false;
    }
    return true;
}
static_assert(every_kind_named(kKindNames), "LayerKind gained an enumerator with no name");

/**
 * @brief Where @p index lands in a sibling list of @p count that already holds
 *        the layer being moved
 *
 * The move takes the layer out first, so the destination is clamped against
 * count - 1, not count: asking for index 2 in [A, B, C] puts the layer last,
 * and asking for anything larger does the same rather than failing.
 */
size_t destination_index(size_t index, size_t count) {
    if (count == 0) return 0;
    return std::min(index, count - 1);
}

/// std::vector::insert wants a signed offset; every index in this file is a
/// size_t, so the cast happens here once instead of at nine call sites.
std::ptrdiff_t offset_of(size_t index) { return static_cast<std::ptrdiff_t>(index); }

} // namespace

const char* layer_kind_name(LayerKind kind) {
    const size_t index = static_cast<size_t>(kind);
    return index < kKindNames.size() ? kKindNames[index] : kKindNames[0];
}

glm::vec3 default_layer_colour() { return glm::vec3(0.78f, 0.78f, 0.80f); }

// ============================================================================
// LayerTransform
// ============================================================================

bool LayerTransform::is_identity() const {
    return translation == glm::dvec3(0.0, 0.0, 0.0) && rotation == glm::dvec3(0.0, 0.0, 0.0)
           && scale == glm::dvec3(1.0, 1.0, 1.0);
}

glm::dmat4 LayerTransform::matrix() const {
    glm::dmat4 m = glm::translate(glm::dmat4(1.0), translation);
    // Yaw, then pitch, then roll. Fixed here so the viewport and the exporter
    // cannot each pick an order and disagree about what a rotated layer means.
    m = glm::rotate(m, rotation.y, glm::dvec3(0.0, 1.0, 0.0));
    m = glm::rotate(m, rotation.x, glm::dvec3(1.0, 0.0, 0.0));
    m = glm::rotate(m, rotation.z, glm::dvec3(0.0, 0.0, 1.0));
    m = glm::scale(m, scale);
    return m;
}

bool operator==(const LayerTransform& a, const LayerTransform& b) {
    return a.translation == b.translation && a.rotation == b.rotation && a.scale == b.scale;
}

bool operator!=(const LayerTransform& a, const LayerTransform& b) { return !(a == b); }

// ============================================================================
// LayerTree -- lookup
// ============================================================================

bool LayerTree::contains(LayerId id) const { return m_layers.find(id) != m_layers.end(); }

const Layer* LayerTree::find(LayerId id) const {
    const auto it = m_layers.find(id);
    return it == m_layers.end() ? nullptr : &it->second;
}

const std::vector<LayerId>& LayerTree::children(LayerId parent) const {
    // A stale parent has to return something, and a reference cannot be null.
    static const std::vector<LayerId> kNoChildren;

    if (parent == kInvalidLayer) return m_roots;
    const Layer* layer = find(parent);
    return layer == nullptr ? kNoChildren : layer->children;
}

size_t LayerTree::index_of(LayerId id) const {
    const Layer* layer = find(id);
    if (layer == nullptr) return kAppend;

    const std::vector<LayerId>& siblings = children(layer->parent);
    const auto it = std::find(siblings.begin(), siblings.end(), id);
    return it == siblings.end() ? kAppend : static_cast<size_t>(it - siblings.begin());
}

size_t LayerTree::depth(LayerId id) const {
    const std::vector<LayerId> chain = self_and_ancestors(id);
    return chain.empty() ? 0u : chain.size() - 1u;
}

bool LayerTree::is_ancestor_of(LayerId ancestor, LayerId descendant) const {
    if (ancestor == kInvalidLayer) return false;

    const std::vector<LayerId> chain = self_and_ancestors(descendant);
    // Skip element 0: that is the descendant itself, and the relation is strict.
    for (size_t i = 1; i < chain.size(); ++i) {
        if (chain[i] == ancestor) return true;
    }
    return false;
}

std::vector<LayerId> LayerTree::subtree_ids(LayerId root) const {
    std::vector<LayerId> out;
    if (!contains(root)) return out;

    std::vector<LayerId> pending{root};
    while (!pending.empty()) {
        const LayerId id = pending.back();
        pending.pop_back();
        out.push_back(id);

        // Bounded for the same reason the ancestor walk is: a cycle here is an
        // editor that never comes back, and that is worse than a wrong tree.
        if (out.size() > m_layers.size()) {
            spdlog::error("LayerTree: subtree of {} is longer than the tree; the children "
                          "links form a cycle",
                          root);
            break;
        }

        const Layer* layer = find(id);
        if (layer == nullptr) continue;

        // Pushed in reverse so the stack pops them in sibling order.
        for (auto it = layer->children.rbegin(); it != layer->children.rend(); ++it) {
            pending.push_back(*it);
        }
    }
    return out;
}

bool LayerTree::accepts_children(LayerId parent) const {
    if (parent == kInvalidLayer) return true;  // the top level
    const Layer* layer = find(parent);
    return layer != nullptr && layer->kind == LayerKind::Group;
}

bool LayerTree::can_reparent(LayerId id, LayerId new_parent) const {
    if (!contains(id)) return false;
    if (!accepts_children(new_parent)) return false;
    if (id == new_parent) return false;

    // The one check that must happen BEFORE the move. A layer made its own
    // ancestor detaches a whole branch from the tree and turns every walk up the
    // parent chain into a loop; detecting that afterwards means detecting it as
    // a hang.
    return !is_ancestor_of(id, new_parent);
}

// ============================================================================
// LayerTree -- effective state
// ============================================================================

bool LayerTree::effective_visible(LayerId id) const {
    const std::vector<LayerId> chain = self_and_ancestors(id);
    if (chain.empty()) return false;  // stale: nothing to draw

    for (LayerId current : chain) {
        const Layer* layer = find(current);
        if (layer == nullptr || !layer->own_visible) return false;
    }
    return true;
}

bool LayerTree::effective_locked(LayerId id) const {
    const std::vector<LayerId> chain = self_and_ancestors(id);
    if (chain.empty()) return true;  // stale: an edit tool should refuse

    for (LayerId current : chain) {
        const Layer* layer = find(current);
        if (layer == nullptr || layer->own_locked) return true;
    }
    return false;
}

glm::vec3 LayerTree::effective_colour(LayerId id) const {
    // Nearest first, so the first explicit colour found is the one that wins.
    for (LayerId current : self_and_ancestors(id)) {
        const Layer* layer = find(current);
        if (layer != nullptr && layer->own_colour.has_value()) return *layer->own_colour;
    }
    return default_layer_colour();
}

glm::dmat4 LayerTree::effective_transform(LayerId id) const {
    const std::vector<LayerId> chain = self_and_ancestors(id);

    glm::dmat4 result(1.0);
    // The chain runs nearest-first, so walk it backwards: the outermost group
    // has to be applied first for its children to move with it.
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const Layer* layer = find(*it);
        if (layer == nullptr || layer->own_transform.is_identity()) continue;
        result *= layer->own_transform.matrix();
    }
    return result;
}

// ============================================================================
// LayerTree -- mutation
//
// Private. The only caller is LayerCommand, so that every change to the tree is
// a step on the command stack; see the file comment in layer.hpp.
// ============================================================================

LayerId LayerTree::reserve_id() {
    const LayerId id = m_next_id;
    ++m_next_id;
    return id;
}

std::vector<LayerId>& LayerTree::sibling_list(LayerId parent) {
    if (parent == kInvalidLayer) return m_roots;

    const auto it = m_layers.find(parent);
    if (it == m_layers.end()) {
        spdlog::error("LayerTree: parent {} is missing; treating its children as top level",
                      parent);
        return m_roots;
    }
    return it->second.children;
}

Layer* LayerTree::mutable_layer(LayerId id) {
    const auto it = m_layers.find(id);
    return it == m_layers.end() ? nullptr : &it->second;
}

bool LayerTree::insert(Layer layer, size_t index) {
    if (layer.id == kInvalidLayer) return false;
    if (contains(layer.id)) {
        spdlog::error("LayerTree: refusing to insert layer {} twice", layer.id);
        return false;
    }
    if (!accepts_children(layer.parent)) return false;

    const LayerId id = layer.id;
    const LayerId parent = layer.parent;

    // Keeps the counter ahead of anything a load or an undo puts back, so an id
    // can never be issued twice even when it did not come from reserve_id().
    m_next_id = std::max(m_next_id, id + 1u);
    m_layers.emplace(id, std::move(layer));

    std::vector<LayerId>& siblings = sibling_list(parent);
    siblings.insert(siblings.begin() + offset_of(std::min(index, siblings.size())), id);
    return true;
}

std::vector<Layer> LayerTree::detach(LayerId root, LayerId* parent_out, size_t* index_out) {
    std::vector<Layer> removed;

    const Layer* root_layer = find(root);
    if (root_layer == nullptr) return removed;

    const LayerId parent = root_layer->parent;
    // Collected before anything is erased; the walk reads the children links.
    const std::vector<LayerId> ids = subtree_ids(root);

    std::vector<LayerId>& siblings = sibling_list(parent);
    const auto it = std::find(siblings.begin(), siblings.end(), root);
    const size_t index =
        it == siblings.end() ? siblings.size() : static_cast<size_t>(it - siblings.begin());
    if (it != siblings.end()) siblings.erase(it);

    removed.reserve(ids.size());
    for (LayerId id : ids) {
        const auto found = m_layers.find(id);
        if (found == m_layers.end()) continue;
        // Moved out whole: name, children order, colour and transform all travel
        // with it, which is what lets attach() reproduce the subtree exactly.
        removed.push_back(std::move(found->second));
        m_layers.erase(found);
    }

    if (parent_out != nullptr) *parent_out = parent;
    if (index_out != nullptr) *index_out = index;
    return removed;
}

bool LayerTree::attach(const std::vector<Layer>& subtree, size_t index) {
    if (subtree.empty()) return false;

    for (const Layer& layer : subtree) {
        if (contains(layer.id)) {
            spdlog::error("LayerTree: cannot restore layer {}; that id is already in the tree",
                          layer.id);
            return false;
        }
    }

    const LayerId root_id = subtree.front().id;
    LayerId parent = subtree.front().parent;
    if (parent != kInvalidLayer && !contains(parent)) {
        // Unreachable through the stack, which undoes in reverse order, so the
        // parent is always put back first. Restoring at the top level instead of
        // refusing keeps the layers -- an undo that silently drops a subtree is
        // the worst outcome available here.
        spdlog::error("LayerTree: parent {} of restored layer {} is gone; restoring at the "
                      "top level",
                      parent, root_id);
        parent = kInvalidLayer;
    }

    for (const Layer& layer : subtree) {
        Layer copy = layer;
        if (copy.id == root_id) copy.parent = parent;
        m_next_id = std::max(m_next_id, copy.id + 1u);
        m_layers.emplace(copy.id, std::move(copy));
    }

    std::vector<LayerId>& siblings = sibling_list(parent);
    siblings.insert(siblings.begin() + offset_of(std::min(index, siblings.size())), root_id);
    return true;
}

bool LayerTree::move(LayerId id, LayerId new_parent, size_t index) {
    if (!can_reparent(id, new_parent)) return false;

    Layer* layer = mutable_layer(id);
    if (layer == nullptr) return false;

    const LayerId old_parent = layer->parent;

    std::vector<LayerId>& old_siblings = sibling_list(old_parent);
    old_siblings.erase(std::remove(old_siblings.begin(), old_siblings.end(), id),
                       old_siblings.end());

    layer->parent = new_parent;

    // Fetched after the erase, and after the parent field is updated, because
    // the destination may be the same vector -- a reorder is a move whose two
    // ends coincide, and the index is counted in the shortened list.
    std::vector<LayerId>& new_siblings = sibling_list(new_parent);
    new_siblings.insert(new_siblings.begin() + offset_of(std::min(index, new_siblings.size())),
                        id);
    return true;
}

std::vector<LayerId> LayerTree::self_and_ancestors(LayerId id) const {
    std::vector<LayerId> chain;

    LayerId current = id;
    while (current != kInvalidLayer) {
        const Layer* layer = find(current);
        if (layer == nullptr) break;

        chain.push_back(current);
        if (chain.size() > m_layers.size()) {
            // can_reparent() refuses the only operation that could build a
            // cycle, so this is a corrupt tree rather than a legal edit. It is
            // still bounded: a hang here is a frozen editor with no message,
            // which is strictly harder to diagnose than a logged error.
            spdlog::error("LayerTree: ancestors of {} outnumber the tree; the parent links "
                          "form a cycle",
                          id);
            break;
        }
        current = layer->parent;
    }
    return chain;
}

// ============================================================================
// LayerCommand
// ============================================================================

LayerId LayerCommand::reserve_id(LayerTree& tree) { return tree.reserve_id(); }

bool LayerCommand::insert_layer(Layer layer, size_t index) {
    return m_tree->insert(std::move(layer), index);
}

std::vector<Layer> LayerCommand::detach_subtree(LayerId root, LayerId* parent_out,
                                                size_t* index_out) {
    return m_tree->detach(root, parent_out, index_out);
}

bool LayerCommand::attach_subtree(const std::vector<Layer>& subtree, size_t index) {
    return m_tree->attach(subtree, index);
}

bool LayerCommand::move_layer(LayerId id, LayerId new_parent, size_t index) {
    return m_tree->move(id, new_parent, index);
}

Layer* LayerCommand::mutable_layer(LayerId id) { return m_tree->mutable_layer(id); }

// ============================================================================
// CreateLayerCommand
// ============================================================================

CreateLayerCommand::CreateLayerCommand(LayerTree& tree, LayerKind kind, std::string name,
                                       LayerId parent, size_t index)
    : LayerCommand(tree, reserve_id(tree)),
      m_kind(kind),
      m_name(std::move(name)),
      m_parent(parent),
      m_index(index) {}

bool CreateLayerCommand::apply() {
    Layer layer;
    layer.id = m_layer;
    layer.kind = m_kind;
    // Copied, not moved: a redo builds the same layer again.
    layer.name = m_name;
    layer.parent = m_parent;

    if (!insert_layer(std::move(layer), m_index)) return false;

    // Pin the slot the layer actually landed in. A redo must put it back where
    // it was, not at the end of a sibling list that has grown since.
    m_index = tree().index_of(m_layer);
    return true;
}

void CreateLayerCommand::revert() {
    const std::vector<Layer> removed = detach_subtree(m_layer, nullptr, nullptr);

    if (removed.size() > 1) {
        // Cannot happen through the stack: anything nested inside this layer was
        // created later and has already been undone by the time this runs.
        spdlog::error("CreateLayerCommand: layer {} still held {} descendants at undo; they "
                      "have been removed with it",
                      m_layer, removed.size() - 1);
    }
}

std::string CreateLayerCommand::describe() const { return "Create layer"; }

size_t CreateLayerCommand::footprint() const {
    return sizeof(CreateLayerCommand) + m_name.capacity();
}

// ============================================================================
// DeleteLayerCommand
// ============================================================================

DeleteLayerCommand::DeleteLayerCommand(LayerTree& tree, LayerId layer)
    : LayerCommand(tree, layer) {}

bool DeleteLayerCommand::apply() {
    if (!tree().contains(m_layer)) return false;

    LayerId parent = kInvalidLayer;
    size_t index = kAppend;
    std::vector<Layer> captured = detach_subtree(m_layer, &parent, &index);
    if (captured.empty()) return false;

    // Assigned only once the detach succeeded, so a refused apply() never leaves
    // a stale subtree behind for revert() to put back.
    m_subtree = std::move(captured);
    m_parent = parent;
    m_index = index;
    return true;
}

void DeleteLayerCommand::revert() {
    if (m_subtree.empty()) return;
    if (!attach_subtree(m_subtree, m_index)) {
        spdlog::error("DeleteLayerCommand: could not restore layer {}", m_layer);
    }
}

std::string DeleteLayerCommand::describe() const { return "Delete layer"; }

size_t DeleteLayerCommand::footprint() const {
    // The whole deleted subtree is held until this step falls off the stack, so
    // reporting only sizeof would leave CommandStack's memory bound bounding
    // nothing -- deleting a district is the largest single thing in a history.
    size_t total = sizeof(DeleteLayerCommand) + m_subtree.capacity() * sizeof(Layer);
    for (const Layer& layer : m_subtree) {
        total += layer.name.capacity();
        total += layer.children.capacity() * sizeof(LayerId);
    }
    return total;
}

// ============================================================================
// RenameLayerCommand
// ============================================================================

RenameLayerCommand::RenameLayerCommand(LayerTree& tree, LayerId layer, std::string name)
    : LayerCommand(tree, layer), m_new_name(std::move(name)) {}

bool RenameLayerCommand::apply() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) return false;
    if (layer->name == m_new_name) return false;  // nothing to record

    m_old_name = layer->name;
    layer->name = m_new_name;
    return true;
}

void RenameLayerCommand::revert() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) {
        spdlog::error("RenameLayerCommand: layer {} is gone at undo", m_layer);
        return;
    }
    layer->name = m_old_name;
}

std::string RenameLayerCommand::describe() const { return "Rename layer"; }

size_t RenameLayerCommand::footprint() const {
    return sizeof(RenameLayerCommand) + m_new_name.capacity() + m_old_name.capacity();
}

bool RenameLayerCommand::merge(const Command& next) {
    const auto* other = dynamic_cast<const RenameLayerCommand*>(&next);
    if (other == nullptr || !same_target(*other)) return false;

    // Refuse a merge that would leave this command a no-op.
    //
    // Type "X" into a name field and backspace it and the gesture ends where it
    // started. Absorbing that blindly leaves a command whose old and new names
    // are equal: undo does nothing visible, and redo hits apply()'s own
    // equal-value guard, so the stack logs a refusal and DROPS the step. The
    // header promises exactly the opposite -- "a command that would change
    // nothing refuses", because an undo menu full of steps that do nothing reads
    // to a user as undo being broken.
    //
    // Refusing here costs one extra undo step for a round-trip and keeps both
    // halves individually reversible, which is the honest trade.
    if (other->m_new_name == m_old_name) return false;

    // Keep the name from before the first keystroke and adopt the latest one, so
    // this command's revert() undoes the whole edit and its redo replays it.
    m_new_name = other->m_new_name;
    return true;
}

// ============================================================================
// ReparentLayerCommand
// ============================================================================

ReparentLayerCommand::ReparentLayerCommand(LayerTree& tree, LayerId layer, LayerId new_parent,
                                           size_t index)
    : LayerCommand(tree, layer), m_new_parent(new_parent), m_index(index) {}

bool ReparentLayerCommand::apply() {
    if (!tree().can_reparent(m_layer, m_new_parent)) return false;

    const Layer* layer = tree().find(m_layer);
    if (layer == nullptr) return false;

    const LayerId old_parent = layer->parent;
    const size_t old_index = tree().index_of(m_layer);

    if (m_new_parent == old_parent) {
        // A drag that ended where it started. Allowed as a reorder, refused when
        // even the position is unchanged.
        const size_t target =
            destination_index(m_index, tree().children(old_parent).size());
        if (target == old_index) return false;
    }

    if (!move_layer(m_layer, m_new_parent, m_index)) return false;

    m_old_parent = old_parent;
    m_old_index = old_index;
    return true;
}

void ReparentLayerCommand::revert() {
    if (!move_layer(m_layer, m_old_parent, m_old_index)) {
        spdlog::error("ReparentLayerCommand: could not move layer {} back under {}", m_layer,
                      m_old_parent);
    }
}

std::string ReparentLayerCommand::describe() const { return "Move layer"; }

// ============================================================================
// ReorderLayerCommand
// ============================================================================

ReorderLayerCommand::ReorderLayerCommand(LayerTree& tree, LayerId layer, size_t index)
    : LayerCommand(tree, layer), m_index(index) {}

bool ReorderLayerCommand::apply() {
    const Layer* layer = tree().find(m_layer);
    if (layer == nullptr) return false;

    const LayerId parent = layer->parent;
    const size_t old_index = tree().index_of(m_layer);
    if (destination_index(m_index, tree().children(parent).size()) == old_index) return false;

    if (!move_layer(m_layer, parent, m_index)) return false;

    m_old_index = old_index;
    return true;
}

void ReorderLayerCommand::revert() {
    const Layer* layer = tree().find(m_layer);
    if (layer == nullptr) {
        spdlog::error("ReorderLayerCommand: layer {} is gone at undo", m_layer);
        return;
    }
    if (!move_layer(m_layer, layer->parent, m_old_index)) {
        spdlog::error("ReorderLayerCommand: could not put layer {} back at {}", m_layer,
                      m_old_index);
    }
}

std::string ReorderLayerCommand::describe() const { return "Reorder layer"; }

// ============================================================================
// SetLayerVisibleCommand
// ============================================================================

SetLayerVisibleCommand::SetLayerVisibleCommand(LayerTree& tree, LayerId layer, bool visible)
    : LayerCommand(tree, layer), m_visible(visible) {}

bool SetLayerVisibleCommand::apply() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) return false;
    if (layer->own_visible == m_visible) return false;

    m_old_visible = layer->own_visible;
    layer->own_visible = m_visible;
    return true;
}

void SetLayerVisibleCommand::revert() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) {
        spdlog::error("SetLayerVisibleCommand: layer {} is gone at undo", m_layer);
        return;
    }
    layer->own_visible = m_old_visible;
}

std::string SetLayerVisibleCommand::describe() const {
    return m_visible ? "Show layer" : "Hide layer";
}

// ============================================================================
// SetLayerLockedCommand
// ============================================================================

SetLayerLockedCommand::SetLayerLockedCommand(LayerTree& tree, LayerId layer, bool locked)
    : LayerCommand(tree, layer), m_locked(locked) {}

bool SetLayerLockedCommand::apply() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) return false;
    if (layer->own_locked == m_locked) return false;

    m_old_locked = layer->own_locked;
    layer->own_locked = m_locked;
    return true;
}

void SetLayerLockedCommand::revert() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) {
        spdlog::error("SetLayerLockedCommand: layer {} is gone at undo", m_layer);
        return;
    }
    layer->own_locked = m_old_locked;
}

std::string SetLayerLockedCommand::describe() const {
    return m_locked ? "Lock layer" : "Unlock layer";
}

// ============================================================================
// SetLayerColourCommand
// ============================================================================

SetLayerColourCommand::SetLayerColourCommand(LayerTree& tree, LayerId layer,
                                             std::optional<glm::vec3> colour)
    : LayerCommand(tree, layer), m_colour(colour) {}

bool SetLayerColourCommand::apply() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) return false;
    if (layer->own_colour == m_colour) return false;

    m_old_colour = layer->own_colour;
    layer->own_colour = m_colour;
    return true;
}

void SetLayerColourCommand::revert() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) {
        spdlog::error("SetLayerColourCommand: layer {} is gone at undo", m_layer);
        return;
    }
    // Restores absence as faithfully as it restores a value: a layer that was
    // inheriting before the edit goes back to inheriting, not to the colour it
    // happened to be showing.
    layer->own_colour = m_old_colour;
}

std::string SetLayerColourCommand::describe() const {
    return m_colour.has_value() ? "Set layer colour" : "Clear layer colour";
}

// ============================================================================
// SetLayerTransformCommand
// ============================================================================

SetLayerTransformCommand::SetLayerTransformCommand(LayerTree& tree, LayerId layer,
                                                   LayerTransform transform)
    : LayerCommand(tree, layer), m_transform(transform) {}

bool SetLayerTransformCommand::apply() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) return false;
    if (layer->own_transform == m_transform) return false;

    m_old_transform = layer->own_transform;
    layer->own_transform = m_transform;
    return true;
}

void SetLayerTransformCommand::revert() {
    Layer* layer = mutable_layer(m_layer);
    if (layer == nullptr) {
        spdlog::error("SetLayerTransformCommand: layer {} is gone at undo", m_layer);
        return;
    }
    layer->own_transform = m_old_transform;
}

std::string SetLayerTransformCommand::describe() const { return "Transform layer"; }

bool SetLayerTransformCommand::merge(const Command& next) {
    const auto* other = dynamic_cast<const SetLayerTransformCommand*>(&next);
    if (other == nullptr || !same_target(*other)) return false;

    // Same no-op refusal as RenameLayerCommand::merge(), for the same reason:
    // drag a gizmo out and back and the gesture ends where it started. See the
    // note there.
    if (other->m_transform == m_old_transform) return false;

    // The successor has already applied, so the tree already holds its value.
    // Taking it over means this command's revert() returns to the transform from
    // before the drag started, which is what m_old_transform still holds.
    m_transform = other->m_transform;
    return true;
}

} // namespace stratum::scene
