// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "scene/selection.hpp"

#include <limits>
#include <utility>

namespace stratum::scene {

// ============================================================================
// Members
// ============================================================================

const char* selection_kind_name(SelectionKind kind) {
    switch (kind) {
    case SelectionKind::Layer:
        return "Layer";
    case SelectionKind::Object:
        return "Object";
    }
    return "Layer";
}

SelectionItem SelectionItem::of_layer(LayerId id) {
    SelectionItem item;
    item.kind = SelectionKind::Layer;
    item.layer = id;
    return item;
}

SelectionItem SelectionItem::of_object(AttributeObject obj) {
    SelectionItem item;
    item.kind = SelectionKind::Object;
    item.object = obj;
    return item;
}

bool SelectionItem::valid() const {
    switch (kind) {
    case SelectionKind::Layer:
        return layer != kInvalidLayer;
    case SelectionKind::Object:
        return object.valid();
    }
    return false;
}

LayerRef layer_ref(LayerId id) {
    return static_cast<LayerRef>(id);
}

LayerId layer_id_from_ref(LayerRef ref) {
    if (ref > static_cast<LayerRef>(std::numeric_limits<LayerId>::max())) return kInvalidLayer;
    return static_cast<LayerId>(ref);
}

// ============================================================================
// Predicates
// ============================================================================

SelectionPredicate is_kind(SelectionKind kind) {
    return [kind](const SelectionItem& item) { return item.kind == kind; };
}

SelectionPredicate is_layer_kind(const LayerTree& layers, LayerKind kind) {
    const LayerTree* tree = &layers;
    return [tree, kind](const SelectionItem& item) {
        if (item.kind != SelectionKind::Layer) return false;
        const Layer* layer = tree->find(item.layer);
        return layer != nullptr && layer->kind == kind;
    };
}

SelectionPredicate is_unlocked(const LayerTree& layers, ObjectLayerFn object_layer) {
    const LayerTree* tree = &layers;
    return [tree, object_layer = std::move(object_layer)](const SelectionItem& item) {
        if (item.kind == SelectionKind::Layer) return !tree->effective_locked(item.layer);

        // An object in no layer has nothing above it to lock it. The explicit
        // check matters: effective_locked() answers TRUE for an id it does not
        // know, which is the safe answer for a layer and the wrong one for an
        // object that was never in a layer to begin with.
        if (!object_layer) return true;
        const LayerRef ref = object_layer(item.object);
        if (ref == kNoLayer) return true;

        // The two zeros are not the same answer, and folding them together
        // failed OPEN: a ref that does not fit a LayerId -- a generation packed
        // into the high bits, which is what LayerRef is 64 bits for -- came back
        // as kInvalidLayer and was reported UNLOCKED, so a move tool would have
        // written into an object it could not identify. An unresolvable ref
        // names an UNKNOWN layer, and A1 locks an id it does not know for
        // exactly this reason; kNoLayer above is the only "in no layer" case.
        const LayerId owner = layer_id_from_ref(ref);
        if (owner == kInvalidLayer) return false;
        return !tree->effective_locked(owner);
    };
}

// ============================================================================
// Snapshot
// ============================================================================

size_t SelectionSnapshot::footprint() const {
    return sizeof(SelectionSnapshot) + items.capacity() * sizeof(SelectionItem);
}

// ============================================================================
// Selection -- construction and liveness
// ============================================================================

Selection::Selection(const LayerTree& layers, const AttributeStore& objects,
                     const CommandStack& history)
    : m_layers(&layers),
      m_objects(&objects),
      m_history(&history),
      // Start level with the scene rather than at zero. A selection built
      // against a document that has already been edited is empty and therefore
      // already correct, and a spurious sweep on its first read would only
      // teach a reader that these fields mean something they do not.
      m_checked_revision(history.revision()),
      m_checked_layers(layers.size()),
      m_checked_objects(objects.live_object_count()) {}

bool Selection::is_live(SelectionItem item) const {
    switch (item.kind) {
    case SelectionKind::Layer:
        // contains() is false for an id that was deleted and for one that was
        // never issued. A1 never reuses an id, so this can never come back true
        // for a DIFFERENT layer that took the number over.
        return item.layer != kInvalidLayer && m_layers->contains(item.layer);
    case SelectionKind::Object:
        // is_valid() compares the generation as well as the slot, so a handle
        // to a destroyed object stays dead even after the slot is recycled.
        return item.object.valid() && m_objects->is_valid(item.object);
    }
    return false;
}

bool Selection::scene_changed() const {
    return m_history->revision() != m_checked_revision || m_layers->size() != m_checked_layers
           || m_objects->live_object_count() != m_checked_objects;
}

void Selection::ensure_fresh() const {
    if (!scene_changed()) return;

    m_checked_revision = m_history->revision();
    m_checked_layers = m_layers->size();
    m_checked_objects = m_objects->live_object_count();

    if (m_order.empty()) return;
    forget_dead();
}

void Selection::forget_dead() const {
    // In place, and that is the whole point. This used to be rebuild(), which
    // also COMPACTS -- and compaction moves survivors down into lower slots
    // while an outstanding const_iterator is holding a raw index into the same
    // vector. Here every slot keeps its number and the vector keeps its length,
    // so a pass that is already running sees exactly what it would have seen:
    // it was going to skip these members anyway, and now skips them as holes.
    for (size_t position = 0; position < m_order.size(); ++position) {
        const SelectionItem item = m_order[position];
        if (!item.valid()) continue;
        if (is_live(item)) continue;
        drop_at(position);
    }
}

// ============================================================================
// Selection -- storage maintenance
// ============================================================================

size_t Selection::rebuild() {
    // The hint indexes m_order, and this moves survivors, so it stops
    // meaning anything. Dropping it costs one scan; keeping it would point
    // at whatever landed in that slot.
    m_primary_hint = kNoPrimaryHint;

    size_t dropped = 0;
    size_t write = 0;

    for (size_t read = 0; read < m_order.size(); ++read) {
        // By value: the loop writes into m_order below the read cursor, and a
        // reference into the slot being overwritten is the classic way to make
        // a compaction pass quietly duplicate an element.
        const SelectionItem item = m_order[read];

        if (!item.valid()) continue;  // a hole left by a removal

        if (!is_live(item)) {
            m_index.erase(item);
            if (item.kind == SelectionKind::Layer) {
                --m_layer_count;
            } else {
                --m_object_count;
            }
            ++dropped;
            continue;
        }

        // Only pay for a hash write when the member actually moved. A sweep
        // that found nothing dead then costs one liveness test per member and
        // no table traffic at all, which is the common case by a wide margin.
        if (write != read) {
            m_order[write] = item;
            m_index[item] = write;
        }
        ++write;
    }

    m_order.resize(write);
    return dropped;
}

void Selection::trim_tail() {
    // The hint indexes m_order, and this moves survivors, so it stops
    // meaning anything. Dropping it costs one scan; keeping it would point
    // at whatever landed in that slot.
    m_primary_hint = kNoPrimaryHint;

    while (!m_order.empty()) {
        const SelectionItem back = m_order.back();
        if (back.valid() && is_live(back)) return;

        if (back.valid()) {
            m_index.erase(back);
            if (back.kind == SelectionKind::Layer) {
                --m_layer_count;
            } else {
                --m_object_count;
            }
        }
        m_order.pop_back();
    }
}

void Selection::drop_at(size_t position) const {
    const SelectionItem item = m_order[position];
    if (!item.valid()) return;

    m_index.erase(item);
    if (item.kind == SelectionKind::Layer) {
        --m_layer_count;
    } else {
        --m_object_count;
    }
    m_order[position] = SelectionItem{};
}

void Selection::compact_if_sparse() {
    // The hint indexes m_order, and this moves survivors, so it stops
    // meaning anything. Dropping it costs one scan; keeping it would point
    // at whatever landed in that slot.
    m_primary_hint = kNoPrimaryHint;

    // Below this, the whole vector is a couple of cache lines and a pass over it
    // costs less than the bookkeeping to avoid one, so the threshold exists to
    // stop a two-member selection rebuilding itself on every click.
    constexpr size_t kMinimumForCompaction = 16;

    if (m_order.size() < kMinimumForCompaction) return;
    if (m_index.size() * 2 >= m_order.size()) return;
    rebuild();
}

// ============================================================================
// Selection -- mutation
// ============================================================================

bool Selection::add(SelectionItem item) {
    ensure_fresh();

    // A mutating path is the only place the holes can be squeezed out, and a run
    // of scene deaths with nothing but reads in between leaves them behind --
    // forget_dead() may not move a survivor. Two integer compares when there is
    // nothing to do, which is every add but the first after such a run.
    compact_if_sparse();

    if (!is_live(item)) return false;
    if (m_index.find(item) != m_index.end()) return false;

    m_order.push_back(item);
    m_index.emplace(item, m_order.size() - 1);
    if (item.kind == SelectionKind::Layer) {
        ++m_layer_count;
    } else {
        ++m_object_count;
    }
    return true;
}

bool Selection::remove(SelectionItem item) {
    ensure_fresh();
    const auto entry = m_index.find(item);
    if (entry == m_index.end()) return false;

    // Read the answer before dropping: drop_at() erases the index entry, and a
    // held member the scene no longer has was never "selected" as far as
    // contains() is concerned, so it must not report true here either.
    const bool was_selected = is_live(item);

    drop_at(entry->second);
    trim_tail();
    compact_if_sparse();
    return was_selected;
}

bool Selection::toggle(SelectionItem item) {
    ensure_fresh();
    if (m_index.find(item) != m_index.end()) {
        remove(item);
        return false;
    }
    return add(item);
}

bool Selection::replace(SelectionItem item) {
    clear();
    return add(item);
}

size_t Selection::replace(const std::vector<SelectionItem>& items) {
    clear();
    return add_all(items);
}

size_t Selection::add_all(const std::vector<SelectionItem>& items) {
    ensure_fresh();

    // One growth step rather than log(m) of them. The rubber band over a city
    // block hands this tens of thousands of items at once.
    m_order.reserve(m_order.size() + items.size());
    m_index.reserve(m_index.size() + items.size());

    size_t added = 0;
    for (const SelectionItem& item : items) {
        if (add(item)) ++added;
    }
    return added;
}

bool Selection::make_primary(SelectionItem item) {
    ensure_fresh();
    const auto entry = m_index.find(item);
    if (entry == m_index.end() || !is_live(item)) return false;

    const size_t position = entry->second;
    if (position + 1 == m_order.size()) return true;  // already the primary

    // Leave a hole and re-append, rather than rotating the vector: a rotation is
    // O(n) moves AND O(n) index updates, for a gesture a user can repeat as fast
    // as they can click.
    m_order[position] = SelectionItem{};
    m_order.push_back(item);
    m_index[item] = m_order.size() - 1;  // same member, so the counts do not move

    compact_if_sparse();
    return true;
}

void Selection::clear() {
    // Move-assigning an empty container, rather than clear(), so the storage
    // actually goes back: clear() on a vector that held half a million members
    // keeps the capacity, and a select-all the user dropped would otherwise hold
    // its 8MB until the document closes.
    //
    // Spelled with the type rather than as `m_order = {}`, which is NOT the same
    // thing: an empty braced-init-list picks operator=(initializer_list), which
    // assigns zero elements and keeps every byte of the capacity. That is
    // exactly the bug this line exists to avoid, and it looks identical.
    m_order = std::vector<SelectionItem>{};
    m_index = IndexMap{};
    m_layer_count = 0;
    m_object_count = 0;

    // Nothing is held, so nothing can be stale: take the token as read rather
    // than leaving a sweep queued against an empty selection.
    m_checked_revision = m_history->revision();
    m_checked_layers = m_layers->size();
    m_checked_objects = m_objects->live_object_count();
}

size_t Selection::restore(const SelectionSnapshot& snapshot) {
    return replace(snapshot.items);
}

// ============================================================================
// Selection -- query
// ============================================================================

bool Selection::contains(SelectionItem item) const {
    // ensure_fresh() is not what makes this exact -- is_live() is. It is here so
    // that a death a reader has already observed is FINAL: without it, deleting
    // a selected layer and then undoing the delete would bring the layer back
    // still selected, and the selection would have acquired exactly the history
    // this file spends its opening section refusing.
    ensure_fresh();
    return m_index.find(item) != m_index.end() && is_live(item);
}

size_t Selection::size() const {
    ensure_fresh();
    return m_index.size();
}

bool Selection::empty() const {
    // Deliberately NOT size() == 0. Iteration tests each member, so this answers
    // exactly even in the one case size() reads high -- the masked destroy in
    // the file comment's residue, where the token never moves and the count is
    // never recomputed -- and it costs less: the first live member is usually
    // the first slot.
    return begin() == end();
}

size_t Selection::count_of(SelectionKind kind) const {
    ensure_fresh();
    return kind == SelectionKind::Layer ? m_layer_count : m_object_count;
}

SelectionItem Selection::primary() const {
    ensure_fresh();

    // A backward scan, not a back(). trim_tail() would make this one line, and
    // that is how it was written: primary() swept the dead members off the tail
    // and returned m_order.back(). Popping from a const accessor shrinks the
    // vector an outstanding iterator is indexing into, and the loop this
    // iterator exists for -- "is this member the active one", asked once per
    // member from inside a range-for -- then ran off the end of its own
    // selection. trim_tail() is non-const now, so the old spelling of this
    // function does not compile any more, which is the point. The scan stops on
    // the first slot it looks at unless the scene has taken members off the tail
    // since the last MUTATION -- forget_dead() leaves holes where they were --
    // so the common case is unchanged.
    // Memoised, because the scan alone is quadratic in exactly the state an
    // editor sits in most of the time.
    //
    // Delete a layer that was selected along with a thousand others and the
    // tail fills with holes. primary() then walks past every one of them, and
    // the loop it exists for asks once PER MEMBER -- so a single pass costs
    // O(members x holes) until the next mutation squeezes them out. The window
    // between a delete and the user's next click is not an edge case; it is the
    // normal state.
    //
    // The hint is only ever a starting point, never an answer: it is re-checked
    // against the live scene on every call, so a stale one costs one failed
    // test and falls through to the scan. That is why it can be updated from a
    // const method without lying.
    if (m_primary_hint < m_order.size()) {
        const SelectionItem& hinted = m_order[m_primary_hint];
        if (hinted.valid() && is_live(hinted)) {
            // Still the last live member only if everything above it is dead.
            // Cheap to confirm, because those slots are holes forget_dead()
            // already wrote.
            bool still_last = true;
            for (size_t above = m_primary_hint + 1; above < m_order.size(); ++above) {
                const SelectionItem& later = m_order[above];
                if (later.valid() && is_live(later)) {
                    still_last = false;
                    break;
                }
            }
            if (still_last) return hinted;
        }
    }

    for (size_t position = m_order.size(); position > 0; --position) {
        const SelectionItem& item = m_order[position - 1];
        if (item.valid() && is_live(item)) {
            m_primary_hint = position - 1;
            return item;
        }
    }

    m_primary_hint = kNoPrimaryHint;
    return SelectionItem{};
}

void Selection::const_iterator::skip_gaps() {
    if (m_selection == nullptr) return;
    const std::vector<SelectionItem>& order = m_selection->m_order;

    while (m_position < order.size()) {
        const SelectionItem& item = order[m_position];
        if (item.valid() && m_selection->is_live(item)) return;
        ++m_position;
    }
}

Selection::const_iterator Selection::begin() const {
    ensure_fresh();
    return const_iterator(this, 0);
}

Selection::const_iterator Selection::end() const {
    ensure_fresh();

    // A sentinel, not a position. end() used to record m_order.size(), and a
    // recorded size is wrong the moment anything shrinks the vector: the cursor
    // stopped advancing at the new size, never reached the recorded one, and the
    // loop dereferenced slots that were no longer there. at_end() re-reads the
    // size instead, so there is nothing left to go stale.
    return const_iterator(this, const_iterator::kEnd);
}

std::vector<LayerId> Selection::layers() const {
    std::vector<LayerId> result;
    result.reserve(count_of(SelectionKind::Layer));
    for (const SelectionItem& item : *this) {
        if (item.kind == SelectionKind::Layer) result.push_back(item.layer);
    }
    return result;
}

std::vector<AttributeObject> Selection::objects() const {
    std::vector<AttributeObject> result;
    result.reserve(count_of(SelectionKind::Object));
    for (const SelectionItem& item : *this) {
        if (item.kind == SelectionKind::Object) result.push_back(item.object);
    }
    return result;
}

SelectionSnapshot Selection::snapshot() const {
    SelectionSnapshot result;
    result.items.reserve(size());
    for (const SelectionItem& item : *this) {
        result.items.push_back(item);
    }
    return result;
}

// ============================================================================
// Selection -- filtering
// ============================================================================

std::vector<SelectionItem> Selection::filter(const SelectionPredicate& predicate) const {
    std::vector<SelectionItem> result;
    if (!predicate) return result;

    for (const SelectionItem& item : *this) {
        if (predicate(item)) result.push_back(item);
    }
    return result;
}

size_t Selection::retain(const SelectionPredicate& predicate) {
    ensure_fresh();

    size_t dropped = 0;
    for (size_t position = 0; position < m_order.size(); ++position) {
        const SelectionItem item = m_order[position];
        if (!item.valid()) continue;

        if (!is_live(item)) {
            // Not counted: it was already not selected as far as contains() is
            // concerned, so reporting it as dropped by the FILTER would blame
            // the predicate for a deletion somewhere else.
            drop_at(position);
            continue;
        }
        // A null predicate matches nothing, which is the reading filter() has
        // always given it. The two used to disagree -- filter() returned an
        // empty list and retain() returned 0 and kept everything -- and they can
        // be handed the same object: attribute_predicate(key, {}) answers false
        // for every member. "An unset filter keeps the whole selection" is also
        // the dangerous half of the two readings for a tool that is about to act
        // on what survives.
        if (predicate && predicate(item)) continue;

        drop_at(position);
        ++dropped;
    }

    trim_tail();
    compact_if_sparse();
    return dropped;
}

SelectionPredicate Selection::attribute_predicate(AttributeKey key, AttributePredicate value_test,
                                                  ObjectLayerFn object_layer) const {
    const AttributeStore* store = m_objects;
    return [store, key, value_test = std::move(value_test),
            object_layer = std::move(object_layer)](const SelectionItem& item) {
        if (!value_test) return false;

        if (item.kind == SelectionKind::Layer) {
            // attributes.hpp: a default-constructed object handle skips the
            // object rungs, which is exactly "what this layer alone would give".
            return value_test(store->resolve(AttributeObject{}, key, layer_ref(item.layer)));
        }

        // kNoLayer rather than a guess. Resolving an object against the wrong
        // layer is worse than resolving it against none: it reports Layer as the
        // source and names a layer the object is not in.
        const LayerRef owner = object_layer ? object_layer(item.object) : kNoLayer;
        return value_test(store->resolve(item.object, key, owner));
    };
}

// ============================================================================
// Selection -- staleness and cost
// ============================================================================

size_t Selection::prune() {
    m_checked_revision = m_history->revision();
    m_checked_layers = m_layers->size();
    m_checked_objects = m_objects->live_object_count();
    return rebuild();
}

size_t Selection::footprint() const {
    ensure_fresh();

    // An estimate for the memory bound and the panel, not an audit. The index's
    // real cost is implementation-defined; a node plus a bucket pointer per
    // member is the shape every libstdc++/libc++ unordered_map actually has.
    const size_t node = sizeof(std::pair<const SelectionItem, size_t>) + sizeof(void*);
    return sizeof(Selection) + m_order.capacity() * sizeof(SelectionItem) + m_index.size() * node
           + m_index.bucket_count() * sizeof(void*);
}

} // namespace stratum::scene
