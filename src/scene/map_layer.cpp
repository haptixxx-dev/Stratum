// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "scene/map_layer.hpp"

#include <spdlog/spdlog.h>

#include <array>
#include <cmath>
#include <utility>

namespace stratum::scene {

namespace {

/// Spelling of each MapLayerType, indexed by the enumerator.
constexpr std::array<const char*, kMapLayerTypeCount> kTypeNames = {"Obstacle", "Water", "Scalar",
                                                                    "Texture"};

/// Totality check for the table above, copied from layer.cpp's kKindNames guard
/// and for the same reason: adding an enumerator without a name should fail the
/// BUILD, not let the panel print nothing.
constexpr bool every_type_named(const std::array<const char*, kMapLayerTypeCount>& names) {
    for (const char* name : names) {
        if (name == nullptr) return false;
    }
    return true;
}
static_assert(every_type_named(kTypeNames), "MapLayerType gained an enumerator with no name");

/// True when @p value is a real, usable world extent.
bool positive_finite(double value) { return std::isfinite(value) && value > 0.0; }

/**
 * @brief May @p placement be installed on @p record?
 *
 * Two separate rules, because they protect different things:
 *
 *   - A BOUNDED placement must have a real extent whatever backs the record.
 *     A zero-width rectangle divides by zero on the way to a texel, and a
 *     record with no image today may be given one tomorrow.
 *   - An UNBOUNDED placement is only meaningful for a callable. There is no
 *     way to turn a world position into a texel without an extent, so an
 *     unbounded image layer would be a layer that can never answer.
 */
bool placement_acceptable(const MapLayerData& record, const MapPlacement& placement) {
    if (placement.bounded) return placement.valid_for_image();
    return record.image.empty();
}

} // namespace

// ============================================================================
// Names
// ============================================================================

const char* map_layer_type_name(MapLayerType type) {
    const auto index = static_cast<size_t>(type);
    return index < kTypeNames.size() ? kTypeNames[index] : "Unknown";
}

const char* map_sample_status_name(MapSampleStatus status) {
    // A switch rather than a table because MapSampleStatus is returned from the
    // hot path and is the thing a failing test prints; a wrong index here would
    // mislabel a failure, which is worse than a missing panel string.
    switch (status) {
        case MapSampleStatus::Ok: return "Ok";
        case MapSampleStatus::Outside: return "Outside";
        case MapSampleStatus::NoLayer: return "NoLayer";
        case MapSampleStatus::NoData: return "NoData";
        case MapSampleStatus::WrongType: return "WrongType";
        case MapSampleStatus::BadPlacement: return "BadPlacement";
    }
    return "Unknown";
}

// ============================================================================
// MapImage
// ============================================================================

namespace {

/**
 * @brief Shape checks for a factory, before there are any pixels to check
 *
 * Deliberately NOT image_well_formed(): that function is asked of a FINISHED
 * image, and a populated size with an empty buffer is exactly the malformed
 * state it rejects -- which is every half-built image inside a factory. Asking
 * it too early is how both factories came to return the empty image for every
 * input they were given.
 *
 * @param expected Receives width * height * channels when the shape is usable
 */
bool factory_shape_ok(uint32_t width, uint32_t height, uint8_t channels, size_t& expected) {
    if (width == 0 || height == 0) return false;
    if (channels != 1 && channels != 3 && channels != 4) return false;
    expected = static_cast<size_t>(width) * static_cast<size_t>(height) *
               static_cast<size_t>(channels);
    return true;
}

} // namespace

MapImage make_u8_image(uint32_t width, uint32_t height, uint8_t channels,
                       std::vector<uint8_t> pixels) {
    // The length is checked here rather than trusted, because a truncated
    // decode that is accepted does not fail at load -- it fails much later as
    // an out-of-range read in the middle of a generation run.
    size_t expected = 0;
    if (!factory_shape_ok(width, height, channels, expected)) return MapImage{};
    if (pixels.size() != expected) return MapImage{};

    MapImage image;
    image.width = width;
    image.height = height;
    image.channels = channels;
    image.u8 = std::move(pixels);
    return image;
}

MapImage make_f32_image(uint32_t width, uint32_t height, uint8_t channels,
                        std::vector<float> pixels) {
    size_t expected = 0;
    if (!factory_shape_ok(width, height, channels, expected)) return MapImage{};
    if (pixels.size() != expected) return MapImage{};

    MapImage image;
    image.width = width;
    image.height = height;
    image.channels = channels;
    image.f32 = std::move(pixels);
    return image;
}

bool image_well_formed(const MapImage& image) {
    // An image with no dimensions is the deliberate "unloaded" state, and it
    // must carry no pixels: dimensions and buffer disagreeing is the bug this
    // check exists to catch.
    if (image.width == 0 || image.height == 0) return image.u8.empty() && image.f32.empty();

    if (image.channels != 1 && image.channels != 3 && image.channels != 4) return false;

    // Exactly one buffer. Both populated means two different answers for every
    // texel, with fetch() silently preferring one of them.
    if (!image.u8.empty() && !image.f32.empty()) return false;

    const size_t needed = image.value_count();
    if (!image.u8.empty()) return image.u8.size() == needed;
    if (!image.f32.empty()) return image.f32.size() == needed;

    // Dimensions but no pixels. MapImage::empty() calls this empty -- that is
    // the SAFETY question, and answering it true keeps the sampler off the
    // buffer. This is the ACCEPTANCE question, and a caller who set a size and
    // forgot the data should hear about it.
    return false;
}

// ============================================================================
// MapPlacement
// ============================================================================

MapPlacement MapPlacement::rect(double min_x, double min_z, double size_x, double size_z) {
    MapPlacement placement;
    placement.min_x = min_x;
    placement.min_z = min_z;
    placement.size_x = size_x;
    placement.size_z = size_z;
    placement.bounded = true;
    return placement;
}

MapPlacement MapPlacement::unbounded() {
    MapPlacement placement;
    placement.bounded = false;
    return placement;
}

bool MapPlacement::contains(double world_x, double world_z) const {
    // NaN is rejected before the bounded test, not after: an unbounded
    // placement containing NaN would hand a not-a-number straight to a
    // function layer's callable.
    if (!std::isfinite(world_x) || !std::isfinite(world_z)) return false;
    if (!bounded) return true;

    // Half-open on both axes. See the file comment: tiles laid edge to edge
    // must cover their shared seam exactly once.
    return world_x >= min_x && world_x < min_x + size_x && world_z >= min_z &&
           world_z < min_z + size_z;
}

bool MapPlacement::valid_for_image() const {
    return bounded && std::isfinite(min_x) && std::isfinite(min_z) && positive_finite(size_x) &&
           positive_finite(size_z);
}

bool operator==(const MapPlacement& a, const MapPlacement& b) {
    return a.min_x == b.min_x && a.min_z == b.min_z && a.size_x == b.size_x &&
           a.size_z == b.size_z && a.bounded == b.bounded;
}

bool operator!=(const MapPlacement& a, const MapPlacement& b) { return !(a == b); }

// ============================================================================
// MapSampling
// ============================================================================

bool operator==(const MapSampling& a, const MapSampling& b) {
    return a.filter == b.filter && a.edge == b.edge && a.outside_value == b.outside_value &&
           a.outside_colour == b.outside_colour && a.threshold == b.threshold;
}

bool operator!=(const MapSampling& a, const MapSampling& b) { return !(a == b); }

// ============================================================================
// MapLayerStore
// ============================================================================

bool MapLayerStore::is_bound(LayerId layer) const {
    return m_records.find(layer) != m_records.end();
}

const MapLayerData* MapLayerStore::find(LayerId layer) const {
    const auto it = m_records.find(layer);
    return it == m_records.end() ? nullptr : &it->second;
}

std::vector<LayerId> MapLayerStore::bound_layers() const {
    std::vector<LayerId> ids;
    ids.reserve(m_records.size());
    for (const auto& [id, record] : m_records) {
        (void)record;
        ids.push_back(id);
    }
    return ids;
}

std::vector<LayerId> MapLayerStore::bound_layers_of_type(MapLayerType type) const {
    std::vector<LayerId> ids;
    for (const auto& [id, record] : m_records) {
        if (record.type == type) ids.push_back(id);
    }
    return ids;
}

MapScalarSample MapLayerStore::sample_scalar(LayerId layer, double world_x, double world_z) const {
    const MapLayerData* record = find(layer);
    // NoLayer, not NoData: "there is no such map layer" and "that map layer is
    // empty" are different facts and a caller may well act differently on them.
    if (record == nullptr) return MapScalarSample{MapSampleStatus::NoLayer, 0.0f};
    return scene::sample_scalar(*record, world_x, world_z);
}

MapBoolSample MapLayerStore::sample_mask(LayerId layer, double world_x, double world_z) const {
    const MapLayerData* record = find(layer);
    if (record == nullptr) return MapBoolSample{MapSampleStatus::NoLayer, false};
    return scene::sample_mask(*record, world_x, world_z);
}

MapColourSample MapLayerStore::sample_colour(LayerId layer, double world_x, double world_z) const {
    const MapLayerData* record = find(layer);
    if (record == nullptr) return MapColourSample{MapSampleStatus::NoLayer, glm::vec4(0.0f)};
    return scene::sample_colour(*record, world_x, world_z);
}

bool MapLayerStore::load(LayerId layer, MapLayerData data) {
    if (layer == kInvalidLayer) return false;

    // The commands each refuse to produce a record with two sources, so
    // refusing here is what makes "at most one source" an invariant of the
    // store rather than a convention the commands happen to keep. Without it a
    // document read off disk could hold both, and MapLayerData::source() would
    // report Image while sample_scalar() quietly read the callable.
    if (!record_consistent(data)) return false;

    m_records[layer] = std::move(data);
    return true;
}

size_t MapLayerStore::drop_records_for_deleted_layers(const LayerTree& tree) {
    size_t dropped = 0;
    for (auto it = m_records.begin(); it != m_records.end();) {
        if (tree.contains(it->first)) {
            ++it;
            continue;
        }
        it = m_records.erase(it);
        ++dropped;
    }
    return dropped;
}

void MapLayerStore::clear() { m_records.clear(); }

bool MapLayerStore::bind(LayerId layer, MapLayerType type) {
    if (layer == kInvalidLayer) return false;
    if (m_records.find(layer) != m_records.end()) return false;

    MapLayerData record;
    record.type = type;
    m_records.emplace(layer, std::move(record));
    return true;
}

std::optional<MapLayerData> MapLayerStore::unbind(LayerId layer) {
    const auto it = m_records.find(layer);
    if (it == m_records.end()) return std::nullopt;

    MapLayerData record = std::move(it->second);
    m_records.erase(it);
    return record;
}

MapLayerData* MapLayerStore::mutable_record(LayerId layer) {
    const auto it = m_records.find(layer);
    return it == m_records.end() ? nullptr : &it->second;
}

// ============================================================================
// active_map_layers
// ============================================================================

std::vector<LayerId> active_map_layers(const LayerTree& tree, const MapLayerStore& store,
                                       MapLayerType type) {
    std::vector<LayerId> active;

    // Tree order, not store order, so a consumer that composites several layers
    // gets a stable, user-visible precedence: the order the panel shows.
    for (const LayerId root : tree.roots()) {
        for (const LayerId id : tree.subtree_ids(root)) {
            const Layer* layer = tree.find(id);
            if (layer == nullptr) continue;

            // Kind is checked HERE as well as in BindMapLayerCommand, because
            // MapLayerStore::load() -- A2's loader path -- has no tree to check
            // against and could file a record on anything.
            if (layer->kind != LayerKind::Map) continue;

            // Hiding a group must really take its obstacle maps out of the
            // computation, which is the ancestor walk, not layer->own_visible.
            if (!tree.effective_visible(id)) continue;

            const MapLayerData* record = store.find(id);
            if (record == nullptr || record->type != type) continue;

            // Empty records are dropped here so the consumer's loop never has
            // to re-test NoData per sample.
            if (!record->has_data()) continue;

            active.push_back(id);
        }
    }
    return active;
}

// ============================================================================
// MapLayerCommand
// ============================================================================

bool MapLayerCommand::target_is_map_layer() const {
    const Layer* layer = m_tree->find(m_layer);
    return layer != nullptr && layer->kind == LayerKind::Map;
}

bool MapLayerCommand::bind_record(MapLayerType type) { return m_store->bind(m_layer, type); }

std::optional<MapLayerData> MapLayerCommand::unbind_record() { return m_store->unbind(m_layer); }

MapLayerData* MapLayerCommand::mutable_record() { return m_store->mutable_record(m_layer); }

// ============================================================================
// BindMapLayerCommand
// ============================================================================

BindMapLayerCommand::BindMapLayerCommand(LayerTree& tree, MapLayerStore& store, LayerId layer,
                                         MapLayerType type)
    : MapLayerCommand(tree, store, layer), m_type(type) {}

bool BindMapLayerCommand::apply() {
    // The invariant that keeps this side table attached to A1's tree rather
    // than beside it. Checked at apply() and not at construction, because a
    // transaction may legitimately create the layer in an earlier command.
    if (!target_is_map_layer()) return false;
    return bind_record(m_type);
}

void BindMapLayerCommand::revert() {
    // Deliberately does NOT consult the tree: revert() must succeed, and by the
    // time an outer transaction unwinds, the layer this bound to may already
    // have been taken back out by the create command that made it.
    if (!unbind_record().has_value()) {
        spdlog::error("BindMapLayerCommand: record for layer {} is gone at undo", m_layer);
    }
}

std::string BindMapLayerCommand::describe() const { return "Add map layer data"; }

// ============================================================================
// UnbindMapLayerCommand
// ============================================================================

UnbindMapLayerCommand::UnbindMapLayerCommand(LayerTree& tree, MapLayerStore& store, LayerId layer)
    : MapLayerCommand(tree, store, layer) {}

bool UnbindMapLayerCommand::apply() {
    if (!target_is_map_layer()) return false;

    m_record = unbind_record();
    return m_record.has_value();
}

void UnbindMapLayerCommand::revert() {
    if (!m_record.has_value()) {
        spdlog::error("UnbindMapLayerCommand: nothing recorded for layer {} at undo", m_layer);
        return;
    }
    // Moved back, not copied: the record may be a basemap, and an undo that
    // duplicated it would double the resident cost of every undo.
    //
    // load() validates, and cannot refuse this: the record came out of the
    // store, which only ever holds consistent records. The check is here
    // because revert() must not fail silently, and a future change to load()'s
    // rules would otherwise drop the undo state without a word.
    if (!store().load(m_layer, std::move(*m_record))) {
        spdlog::error("UnbindMapLayerCommand: store refused the record for layer {} at undo",
                      m_layer);
    }
    m_record.reset();
}

std::string UnbindMapLayerCommand::describe() const { return "Remove map layer data"; }

size_t UnbindMapLayerCommand::footprint() const {
    // The pixels are the undo state. Reporting only sizeof here is exactly the
    // failure Command::footprint()'s documentation warns about: the stack's
    // byte bound would stop bounding anything.
    return sizeof(UnbindMapLayerCommand) + (m_record.has_value() ? m_record->footprint() : 0);
}

// ============================================================================
// SetMapLayerImageCommand
// ============================================================================

SetMapLayerImageCommand::SetMapLayerImageCommand(LayerTree& tree, MapLayerStore& store,
                                                 LayerId layer, MapImage image)
    : MapLayerCommand(tree, store, layer), m_image(std::move(image)) {}

bool SetMapLayerImageCommand::apply() {
    if (!target_is_map_layer()) return false;

    MapLayerData* record = mutable_record();
    if (record == nullptr) return false;

    // m_image is always the INCOMING image at this point: the first apply()
    // takes it from the constructor, and every later one takes it from a
    // revert() that swapped it back. So validating it here validates the right
    // image on a redo as well as on the first run.
    if (!image_well_formed(m_image)) return false;

    if (!m_image.empty()) {
        if (record->function) return false;  // one source at a time
        if (!channels_match_type(record->type, m_image.channels)) return false;
        if (!record->placement.valid_for_image()) return false;
    } else if (record->image.empty()) {
        // Clearing an image that is already clear changes nothing.
        return false;
    }

    std::swap(record->image, m_image);
    return true;
}

void SetMapLayerImageCommand::revert() {
    MapLayerData* record = mutable_record();
    if (record == nullptr) {
        spdlog::error("SetMapLayerImageCommand: record for layer {} is gone at undo", m_layer);
        return;
    }
    std::swap(record->image, m_image);
}

std::string SetMapLayerImageCommand::describe() const {
    return m_image.empty() ? "Clear map image" : "Set map image";
}

size_t SetMapLayerImageCommand::footprint() const {
    return sizeof(SetMapLayerImageCommand) + m_image.heap_bytes();
}

// ============================================================================
// SetMapLayerPlacementCommand
// ============================================================================

SetMapLayerPlacementCommand::SetMapLayerPlacementCommand(LayerTree& tree, MapLayerStore& store,
                                                         LayerId layer, MapPlacement placement)
    : MapLayerCommand(tree, store, layer), m_placement(placement) {}

bool SetMapLayerPlacementCommand::apply() {
    if (!target_is_map_layer()) return false;

    MapLayerData* record = mutable_record();
    if (record == nullptr) return false;
    if (record->placement == m_placement) return false;
    if (!placement_acceptable(*record, m_placement)) return false;

    m_old_placement = record->placement;
    record->placement = m_placement;
    return true;
}

void SetMapLayerPlacementCommand::revert() {
    MapLayerData* record = mutable_record();
    if (record == nullptr) {
        spdlog::error("SetMapLayerPlacementCommand: record for layer {} is gone at undo", m_layer);
        return;
    }
    record->placement = m_old_placement;
}

std::string SetMapLayerPlacementCommand::describe() const { return "Move map layer"; }

bool SetMapLayerPlacementCommand::merge(const Command& next) {
    const auto* other = dynamic_cast<const SetMapLayerPlacementCommand*>(&next);
    if (other == nullptr || !same_target(*other)) return false;

    // Same no-op refusal as RenameLayerCommand::merge(): drag the placement out
    // and back and the gesture ends where it started, leaving a command whose
    // old and new values are equal -- whose redo would then hit apply()'s own
    // equal-value guard and be DROPPED from the stack.
    if (other->m_placement == m_old_placement) return false;

    // The successor has already applied, so the record already holds its value.
    // Keeping m_old_placement is what makes this command's revert() return to
    // where the drag began.
    m_placement = other->m_placement;
    return true;
}

// ============================================================================
// SetMapLayerSamplingCommand
// ============================================================================

SetMapLayerSamplingCommand::SetMapLayerSamplingCommand(LayerTree& tree, MapLayerStore& store,
                                                       LayerId layer, MapSampling sampling)
    : MapLayerCommand(tree, store, layer), m_sampling(sampling) {}

bool SetMapLayerSamplingCommand::apply() {
    if (!target_is_map_layer()) return false;

    MapLayerData* record = mutable_record();
    if (record == nullptr) return false;
    if (record->sampling == m_sampling) return false;

    m_old_sampling = record->sampling;
    record->sampling = m_sampling;
    return true;
}

void SetMapLayerSamplingCommand::revert() {
    MapLayerData* record = mutable_record();
    if (record == nullptr) {
        spdlog::error("SetMapLayerSamplingCommand: record for layer {} is gone at undo", m_layer);
        return;
    }
    record->sampling = m_old_sampling;
}

std::string SetMapLayerSamplingCommand::describe() const { return "Set map sampling"; }

bool SetMapLayerSamplingCommand::merge(const Command& next) {
    const auto* other = dynamic_cast<const SetMapLayerSamplingCommand*>(&next);
    if (other == nullptr || !same_target(*other)) return false;
    if (other->m_sampling == m_old_sampling) return false;

    m_sampling = other->m_sampling;
    return true;
}

// ============================================================================
// SetMapLayerFunctionCommand
// ============================================================================

SetMapLayerFunctionCommand::SetMapLayerFunctionCommand(LayerTree& tree, MapLayerStore& store,
                                                       LayerId layer, MapFunction function)
    : MapLayerCommand(tree, store, layer), m_function(std::move(function)) {}

bool SetMapLayerFunctionCommand::apply() {
    if (!target_is_map_layer()) return false;

    MapLayerData* record = mutable_record();
    if (record == nullptr) return false;

    if (m_function) {
        // A procedural basemap would need a callable returning a colour, which
        // is a second signature for a case nothing asks for.
        if (record->type == MapLayerType::Texture) return false;
        if (!record->image.empty()) return false;  // one source at a time
    } else if (!record->function) {
        // Clearing a callable that is already clear changes nothing.
        return false;
    }

    std::swap(record->function, m_function);
    return true;
}

void SetMapLayerFunctionCommand::revert() {
    MapLayerData* record = mutable_record();
    if (record == nullptr) {
        spdlog::error("SetMapLayerFunctionCommand: record for layer {} is gone at undo", m_layer);
        return;
    }
    std::swap(record->function, m_function);
}

std::string SetMapLayerFunctionCommand::describe() const {
    return m_function ? "Set map function" : "Clear map function";
}

// ============================================================================
// Convenience
// ============================================================================

bool bind_map_layer(CommandStack& stack, LayerTree& tree, MapLayerStore& store, LayerId layer,
                    MapLayerType type) {
    return stack.execute(std::make_unique<BindMapLayerCommand>(tree, store, layer, type));
}

bool unbind_map_layer(CommandStack& stack, LayerTree& tree, MapLayerStore& store, LayerId layer) {
    return stack.execute(std::make_unique<UnbindMapLayerCommand>(tree, store, layer));
}

bool set_map_layer_image(CommandStack& stack, LayerTree& tree, MapLayerStore& store, LayerId layer,
                         MapImage image) {
    return stack.execute(
        std::make_unique<SetMapLayerImageCommand>(tree, store, layer, std::move(image)));
}

bool set_map_layer_placement(CommandStack& stack, LayerTree& tree, MapLayerStore& store,
                             LayerId layer, MapPlacement placement) {
    return stack.execute(
        std::make_unique<SetMapLayerPlacementCommand>(tree, store, layer, placement));
}

bool set_map_layer_sampling(CommandStack& stack, LayerTree& tree, MapLayerStore& store,
                            LayerId layer, MapSampling sampling) {
    return stack.execute(
        std::make_unique<SetMapLayerSamplingCommand>(tree, store, layer, sampling));
}

bool set_map_layer_function(CommandStack& stack, LayerTree& tree, MapLayerStore& store,
                            LayerId layer, MapFunction function) {
    return stack.execute(
        std::make_unique<SetMapLayerFunctionCommand>(tree, store, layer, std::move(function)));
}

} // namespace stratum::scene
