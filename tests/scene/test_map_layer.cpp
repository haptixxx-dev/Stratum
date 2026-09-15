// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_map_layer.cpp
 * @brief Map layers: the image-to-world mapping, the bounds contract, the three
 *        empty states, and the undo of every mutation
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Four groups of properties here are worth more than the rest, because each one
 * fails by returning a plausible number rather than by crashing:
 *
 *   - **The image-to-world mapping.** A transposed or row-flipped sampler still
 *     returns values from the image, and a street network grown against it
 *     still looks like a street network. Every positional test therefore uses a
 *     3x2 grid with six DIFFERENT values, so a transpose, a flip or a
 *     half-texel shift changes the answer rather than preserving it.
 *   - **Bounds.** Outside, clamped and half-open-at-the-far-edge are three
 *     separate decisions and each is checked against a value that could not
 *     have come from the other two -- the outside value is never equal to a
 *     border texel in these tests, or the two edge modes would be
 *     indistinguishable.
 *   - **Empty versus zero.** Unbound, bound-but-empty and an image of zeros all
 *     produce the number 0. They are checked by STATUS, which is the only thing
 *     that tells them apart.
 *   - **Undo.** Every command is undone and redone, because the image and the
 *     callable are swapped rather than copied and a swap that is right once can
 *     still be wrong the second time through.
 *
 * Every mutation here goes through a CommandStack, never through a back door,
 * because "the only way to change the store is a command" is one of the things
 * under test.
 */

#include "framework.hpp"

#include "scene/command.hpp"
#include "scene/layer.hpp"
#include "scene/map_layer.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using stratum::scene::active_map_layers;
using stratum::scene::bind_map_layer;
using stratum::scene::CommandStack;
using stratum::scene::CreateLayerCommand;
using stratum::scene::DeleteLayerCommand;
using stratum::scene::image_well_formed;
using stratum::scene::is_obstacle;
using stratum::scene::is_water;
using stratum::scene::kInvalidLayer;
using stratum::scene::Layer;
using stratum::scene::LayerId;
using stratum::scene::LayerKind;
using stratum::scene::LayerTree;
using stratum::scene::make_f32_image;
using stratum::scene::make_u8_image;
using stratum::scene::map_sample_status_name;
using stratum::scene::MapColourSample;
using stratum::scene::MapEdge;
using stratum::scene::MapFilter;
using stratum::scene::MapImage;
using stratum::scene::MapLayerData;
using stratum::scene::MapLayerStore;
using stratum::scene::MapLayerType;
using stratum::scene::MapPlacement;
using stratum::scene::MapSampleStatus;
using stratum::scene::MapSampling;
using stratum::scene::MapScalarSample;
using stratum::scene::MapSource;
using stratum::scene::record_consistent;
using stratum::scene::ReorderLayerCommand;
using stratum::scene::sample_colour;
using stratum::scene::sample_mask;
using stratum::scene::sample_scalar;
using stratum::scene::set_map_layer_function;
using stratum::scene::set_map_layer_image;
using stratum::scene::set_map_layer_placement;
using stratum::scene::set_map_layer_sampling;
using stratum::scene::SetLayerVisibleCommand;
using stratum::scene::unbind_map_layer;
using stratum::scene::UnbindMapLayerCommand;

namespace {

// ── Shared fixtures ─────────────────────────────────────────────────────────

/// Status as a string, so a failure prints "actual: NoData expected: Ok"
/// instead of "<unprintable>": MapSampleStatus is an enum class and the
/// framework cannot stream it.
std::string status_name(MapSampleStatus status) { return map_sample_status_name(status); }

struct Fixture {
    LayerTree tree;
    MapLayerStore store;
    CommandStack stack;
};

LayerId make_layer(LayerTree& tree, CommandStack& stack, LayerKind kind, const std::string& name,
                   LayerId parent = kInvalidLayer) {
    auto command = std::make_unique<CreateLayerCommand>(tree, kind, name, parent);
    const LayerId id = command->layer();
    if (!stack.execute(std::move(command))) return kInvalidLayer;
    return id;
}

LayerId make_map_layer(Fixture& f, const std::string& name, LayerId parent = kInvalidLayer) {
    return make_layer(f.tree, f.stack, LayerKind::Map, name, parent);
}

/**
 * @brief The canonical test raster
 *
 *   row 0 (low z):  1  2  3
 *   row 1 (high z): 4  5  6
 *
 * Three columns by two rows on purpose: the dimensions differ, so a sampler
 * that reads columns from z runs off a shorter axis, and every one of the six
 * values differs, so a flip or a transpose cannot land on the right answer by
 * accident.
 */
MapImage grid_image() {
    return make_f32_image(3, 2, 1, {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
}

/// One metre per texel, with a non-zero origin so a sampler that forgets to
/// subtract it is wrong everywhere rather than right at the corner.
MapPlacement grid_placement() { return MapPlacement::rect(10.0, 20.0, 3.0, 2.0); }

/// Bind a Scalar layer and give it the canonical grid. Returns the layer id.
LayerId make_grid_layer(Fixture& f, MapLayerType type = MapLayerType::Scalar) {
    const LayerId id = make_map_layer(f, "grid");
    if (!bind_map_layer(f.stack, f.tree, f.store, id, type)) return kInvalidLayer;
    if (!set_map_layer_placement(f.stack, f.tree, f.store, id, grid_placement())) {
        return kInvalidLayer;
    }
    if (!set_map_layer_image(f.stack, f.tree, f.store, id, grid_image())) return kInvalidLayer;
    return id;
}

} // namespace

// ============================================================================
// Binding: a map layer is a Layer, and the store is checked against the tree
// ============================================================================

TEST(MapLayer, bind_refuses_a_layer_that_is_not_map_kind) {
    Fixture f;
    const LayerId shape = make_layer(f.tree, f.stack, LayerKind::Shape, "buildings");
    const LayerId group = make_layer(f.tree, f.stack, LayerKind::Group, "city");

    const size_t depth_before = f.stack.undo_depth();

    CHECK_FALSE(bind_map_layer(f.stack, f.tree, f.store, shape, MapLayerType::Obstacle));
    CHECK_FALSE(bind_map_layer(f.stack, f.tree, f.store, group, MapLayerType::Obstacle));

    // A refusal must leave both the store and the history exactly as they were.
    CHECK_TRUE(f.store.empty());
    CHECK_EQ(f.stack.undo_depth(), depth_before);

    // The same bind on a Map layer succeeds, so the test cannot be passing
    // because binding never works at all.
    const LayerId map = make_map_layer(f, "obstacles");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Obstacle));
    CHECK_TRUE(f.store.is_bound(map));
}

TEST(MapLayer, bind_refuses_a_stale_or_never_issued_id) {
    Fixture f;
    const LayerId map = make_map_layer(f, "obstacles");

    CHECK_TRUE(f.stack.execute(std::make_unique<DeleteLayerCommand>(f.tree, map)));
    CHECK_FALSE(f.tree.contains(map));

    CHECK_FALSE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Obstacle));
    CHECK_FALSE(bind_map_layer(f.stack, f.tree, f.store, LayerId{9999}, MapLayerType::Obstacle));
    CHECK_FALSE(bind_map_layer(f.stack, f.tree, f.store, kInvalidLayer, MapLayerType::Obstacle));
    CHECK_TRUE(f.store.empty());
}

TEST(MapLayer, bind_refuses_a_second_bind_to_the_same_layer) {
    Fixture f;
    const LayerId map = make_map_layer(f, "water");

    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Water));
    const size_t depth = f.stack.undo_depth();

    // Rebinding with a different type must not quietly retype the layer and
    // discard its pixels.
    CHECK_FALSE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Obstacle));
    CHECK_EQ(f.stack.undo_depth(), depth);
    CHECK_EQ(f.store.size(), size_t{1});

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record != nullptr) CHECK_TRUE(record->type == MapLayerType::Water);
}

TEST(MapLayer, bind_undo_removes_the_record_and_redo_puts_it_back) {
    Fixture f;
    const LayerId map = make_map_layer(f, "obstacles");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Obstacle));
    CHECK_TRUE(f.store.is_bound(map));

    CHECK_TRUE(f.stack.undo());
    CHECK_FALSE(f.store.is_bound(map));
    // The layer itself is untouched: only the payload was undone.
    CHECK_TRUE(f.tree.contains(map));

    CHECK_TRUE(f.stack.redo());
    CHECK_TRUE(f.store.is_bound(map));
}

// ============================================================================
// The three empty states
// ============================================================================

TEST(MapLayer, unbound_bound_empty_and_all_zero_are_three_distinct_states) {
    Fixture f;
    const LayerId map = make_map_layer(f, "obstacles");

    // 1. No record at all.
    CHECK_EQ(status_name(f.store.sample_scalar(map, 0.5, 0.5).status), std::string("NoLayer"));
    CHECK_FALSE(f.store.is_bound(map));

    // 2. A record with nothing in it.
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Scalar));
    CHECK_TRUE(f.store.is_bound(map));
    const MapLayerData* empty_record = f.store.find(map);
    CHECK(empty_record != nullptr);
    if (empty_record != nullptr) {
        CHECK_FALSE(empty_record->has_data());
        CHECK_TRUE(empty_record->source() == MapSource::None);
    }
    const MapScalarSample on_empty = f.store.sample_scalar(map, 0.5, 0.5);
    CHECK_EQ(status_name(on_empty.status), std::string("NoData"));
    CHECK_NEAR(on_empty.value, 0.0, 1e-9);

    // 3. A record holding an image that happens to be entirely zero. Same
    //    number, different fact -- which is the whole reason a status is
    //    returned beside the value.
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, grid_placement()));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map,
                                   make_f32_image(3, 2, 1, {0.f, 0.f, 0.f, 0.f, 0.f, 0.f})));
    const MapLayerData* zero_record = f.store.find(map);
    CHECK(zero_record != nullptr);
    if (zero_record != nullptr) {
        CHECK_TRUE(zero_record->has_data());
        CHECK_TRUE(zero_record->source() == MapSource::Image);
    }
    const MapScalarSample on_zero = f.store.sample_scalar(map, 10.5, 20.5);
    CHECK_EQ(status_name(on_zero.status), std::string("Ok"));
    CHECK_NEAR(on_zero.value, 0.0, 1e-9);
}

// ============================================================================
// The image-to-world mapping
// ============================================================================

TEST(MapLayer, nearest_sampling_maps_columns_to_x_and_rows_to_z) {
    Fixture f;
    const LayerId map = make_grid_layer(f);
    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    // Every texel centre, by name. A transpose sends the (12.5, 20.5) probe
    // looking for column 2 along an axis that has only two rows, and a row flip
    // swaps 1..3 with 4..6, so no permutation of the six values survives.
    CHECK_NEAR(sample_scalar(*record, 10.5, 20.5).value, 1.0, 1e-6);
    CHECK_NEAR(sample_scalar(*record, 11.5, 20.5).value, 2.0, 1e-6);
    CHECK_NEAR(sample_scalar(*record, 12.5, 20.5).value, 3.0, 1e-6);
    CHECK_NEAR(sample_scalar(*record, 10.5, 21.5).value, 4.0, 1e-6);
    CHECK_NEAR(sample_scalar(*record, 11.5, 21.5).value, 5.0, 1e-6);
    CHECK_NEAR(sample_scalar(*record, 12.5, 21.5).value, 6.0, 1e-6);

    // Row 0 is at MINIMUM z. Probing just inside the two z extremes pins the
    // direction down without relying on a texel centre.
    CHECK_NEAR(sample_scalar(*record, 10.01, 20.01).value, 1.0, 1e-6);
    CHECK_NEAR(sample_scalar(*record, 10.01, 21.99).value, 4.0, 1e-6);

    // Column 0 is at minimum x, by the same argument.
    CHECK_NEAR(sample_scalar(*record, 12.99, 20.01).value, 3.0, 1e-6);
}

TEST(MapLayer, the_far_edge_of_the_placement_is_exclusive) {
    Fixture f;
    const LayerId map = make_grid_layer(f);
    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    // Near edges are inclusive, far edges are not, so two maps laid edge to
    // edge claim their shared seam exactly once.
    CHECK_EQ(status_name(sample_scalar(*record, 10.0, 20.0).status), std::string("Ok"));
    CHECK_EQ(status_name(sample_scalar(*record, 13.0, 20.5).status), std::string("Outside"));
    CHECK_EQ(status_name(sample_scalar(*record, 10.5, 22.0).status), std::string("Outside"));

    // A whisker inside the far edge is still inside, and reads the last texel.
    const MapScalarSample inside = sample_scalar(*record, 12.9999, 21.9999);
    CHECK_EQ(status_name(inside.status), std::string("Ok"));
    CHECK_NEAR(inside.value, 6.0, 1e-6);
}

TEST(MapLayer, bilinear_puts_the_interpolation_nodes_on_texel_centres) {
    Fixture f;
    const LayerId map = make_map_layer(f, "ramp");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Scalar));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                       MapPlacement::rect(0.0, 0.0, 2.0, 1.0)));
    // Two texels, centres at x = 0.5 and x = 1.5.
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map,
                                   make_f32_image(2, 1, 1, {0.0f, 1.0f})));

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    // Nearest first, at the SAME probe the bilinear case uses below. x = 1.0 is
    // inside texel 1, so nearest must say 1.0 while bilinear says 0.5. Without
    // this pair the filter setting could be ignored entirely and both halves of
    // the test would still pass.
    CHECK_NEAR(sample_scalar(*record, 1.0, 0.5).value, 1.0, 1e-6);

    MapSampling sampling;
    sampling.filter = MapFilter::Bilinear;
    CHECK_TRUE(set_map_layer_sampling(f.stack, f.tree, f.store, map, sampling));

    const MapLayerData* filtered = f.store.find(map);
    CHECK(filtered != nullptr);
    if (filtered == nullptr) return;

    // Exactly on a texel centre the neighbour must contribute nothing. This is
    // what a missing half-texel offset breaks: it would report 0.25 and 0.75
    // here, and the whole image would sit half a texel off the world.
    CHECK_NEAR(sample_scalar(*filtered, 0.5, 0.5).value, 0.0, 1e-6);
    CHECK_NEAR(sample_scalar(*filtered, 1.5, 0.5).value, 1.0, 1e-6);

    // Halfway between the centres.
    CHECK_NEAR(sample_scalar(*filtered, 1.0, 0.5).value, 0.5, 1e-6);
    CHECK_NEAR(sample_scalar(*filtered, 0.75, 0.5).value, 0.25, 1e-6);

    // Before the first centre the border texel is held, not extrapolated
    // towards a negative value.
    CHECK_NEAR(sample_scalar(*filtered, 0.05, 0.5).value, 0.0, 1e-6);
    CHECK_NEAR(sample_scalar(*filtered, 1.95, 0.5).value, 1.0, 1e-6);
}

TEST(MapLayer, u8_and_f32_images_of_the_same_picture_sample_identically) {
    Fixture f;

    const LayerId bytes = make_map_layer(f, "bytes");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, bytes, MapLayerType::Scalar));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, bytes,
                                       MapPlacement::rect(0.0, 0.0, 2.0, 2.0)));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, bytes,
                                   make_u8_image(2, 2, 1, {0, 128, 255, 64})));

    const LayerId floats = make_map_layer(f, "floats");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, floats, MapLayerType::Scalar));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, floats,
                                       MapPlacement::rect(0.0, 0.0, 2.0, 2.0)));
    CHECK_TRUE(set_map_layer_image(
        f.stack, f.tree, f.store, floats,
        make_f32_image(2, 2, 1, {0.0f, 128.0f / 255.0f, 1.0f, 64.0f / 255.0f})));

    const MapLayerData* a = f.store.find(bytes);
    const MapLayerData* b = f.store.find(floats);
    CHECK(a != nullptr);
    CHECK(b != nullptr);
    if (a == nullptr || b == nullptr) return;

    const double probes[4][2] = {{0.5, 0.5}, {1.5, 0.5}, {0.5, 1.5}, {1.5, 1.5}};
    for (const auto& probe : probes) {
        const MapScalarSample from_u8 = sample_scalar(*a, probe[0], probe[1]);
        const MapScalarSample from_f32 = sample_scalar(*b, probe[0], probe[1]);
        CHECK_EQ(status_name(from_u8.status), status_name(from_f32.status));
        CHECK_NEAR(from_u8.value, from_f32.value, 1e-6);
    }

    // And the endpoints land where the normalisation promises, so the test is
    // not merely proving that two wrong readers agree with each other.
    CHECK_NEAR(sample_scalar(*a, 0.5, 0.5).value, 0.0, 1e-9);
    CHECK_NEAR(sample_scalar(*a, 0.5, 1.5).value, 1.0, 1e-9);
    CHECK_NEAR(sample_scalar(*a, 1.5, 0.5).value, 128.0 / 255.0, 1e-6);
}

// ============================================================================
// Bounds
// ============================================================================

TEST(MapLayer, outside_reports_the_outside_status_and_the_outside_value) {
    Fixture f;
    const LayerId map = make_grid_layer(f);

    MapSampling sampling;
    sampling.edge = MapEdge::Outside;
    // Deliberately unlike any texel in the grid, so a reading of -7 can only
    // have come from this field.
    sampling.outside_value = -7.0f;
    CHECK_TRUE(set_map_layer_sampling(f.stack, f.tree, f.store, map, sampling));

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    const MapScalarSample away = sample_scalar(*record, 1000.0, -1000.0);
    CHECK_EQ(status_name(away.status), std::string("Outside"));
    CHECK_NEAR(away.value, -7.0, 1e-6);
    CHECK_FALSE(away.ok());
    CHECK_TRUE(away.has_value());

    // Inside is unaffected.
    CHECK_EQ(status_name(sample_scalar(*record, 11.5, 20.5).status), std::string("Ok"));
    CHECK_NEAR(sample_scalar(*record, 11.5, 20.5).value, 2.0, 1e-6);
}

TEST(MapLayer, clamp_extends_the_border_texel_and_reports_ok) {
    Fixture f;
    const LayerId map = make_grid_layer(f);

    MapSampling sampling;
    sampling.edge = MapEdge::Clamp;
    // Set to something a clamped read can never produce, so "Clamp returned the
    // border texel" and "Clamp fell through to the outside value" cannot both
    // satisfy the checks below.
    sampling.outside_value = -7.0f;
    CHECK_TRUE(set_map_layer_sampling(f.stack, f.tree, f.store, map, sampling));

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    // Far to the low corner: column 0, row 0, which holds 1.
    const MapScalarSample low = sample_scalar(*record, -500.0, -500.0);
    CHECK_EQ(status_name(low.status), std::string("Ok"));
    CHECK_NEAR(low.value, 1.0, 1e-6);

    // Far to the high corner: column 2, row 1, which holds 6.
    const MapScalarSample high = sample_scalar(*record, 5000.0, 5000.0);
    CHECK_EQ(status_name(high.status), std::string("Ok"));
    CHECK_NEAR(high.value, 6.0, 1e-6);

    // Off one axis only: x is clamped to the last column, z still selects the
    // row. A clamp that collapsed both axes would report 6 here too.
    const MapScalarSample mixed = sample_scalar(*record, 5000.0, 20.5);
    CHECK_EQ(status_name(mixed.status), std::string("Ok"));
    CHECK_NEAR(mixed.value, 3.0, 1e-6);
}

TEST(MapLayer, a_position_that_is_not_a_number_is_outside_under_every_edge_mode) {
    Fixture f;
    const LayerId map = make_grid_layer(f);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    for (const MapEdge edge : {MapEdge::Outside, MapEdge::Clamp}) {
        MapSampling sampling;
        sampling.edge = edge;
        sampling.outside_value = -7.0f;
        // A no-op set is refused by design, so only assert progress when the
        // policy actually differs from the one already installed.
        const MapLayerData* before = f.store.find(map);
        CHECK(before != nullptr);
        if (before != nullptr && before->sampling != sampling) {
            CHECK_TRUE(set_map_layer_sampling(f.stack, f.tree, f.store, map, sampling));
        }

        const MapLayerData* record = f.store.find(map);
        CHECK(record != nullptr);
        if (record == nullptr) return;

        // Under Clamp, letting NaN through would floor to -1, clamp to texel
        // (0, 0) and report a confident Ok with the value 1.
        CHECK_EQ(status_name(sample_scalar(*record, nan, 20.5).status), std::string("Outside"));
        CHECK_EQ(status_name(sample_scalar(*record, 10.5, nan).status), std::string("Outside"));
        CHECK_EQ(status_name(sample_scalar(*record, inf, 20.5).status), std::string("Outside"));
        CHECK_NEAR(sample_scalar(*record, nan, nan).value, -7.0, 1e-6);
    }
}

TEST(MapLayer, an_image_cannot_be_installed_without_a_usable_placement) {
    Fixture f;
    const LayerId map = make_map_layer(f, "obstacles");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Obstacle));

    // A fresh record has a zero-sized placement. Accepting an image against it
    // would divide by zero on the way to a texel.
    const size_t depth = f.stack.undo_depth();
    CHECK_FALSE(set_map_layer_image(f.stack, f.tree, f.store, map, grid_image()));
    CHECK_EQ(f.stack.undo_depth(), depth);

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record != nullptr) CHECK_FALSE(record->has_data());

    // A zero-sized placement is refused outright, as is an unbounded one on a
    // layer that is going to hold pixels.
    CHECK_FALSE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                        MapPlacement::rect(0.0, 0.0, 0.0, 5.0)));
    CHECK_FALSE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                        MapPlacement::rect(0.0, 0.0, 5.0, -5.0)));

    // With a real rectangle the same image goes in.
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, grid_placement()));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map, grid_image()));
}

TEST(MapLayer, a_loaded_record_whose_placement_has_no_extent_reports_bad_placement) {
    // MapLayerStore::load() is A2's loader path and bypasses the commands, so it
    // is the one way a record with pixels and no extent can exist. The sampler
    // must say so rather than divide by zero.
    Fixture f;
    const LayerId map = make_map_layer(f, "from disk");

    MapLayerData broken;
    broken.type = MapLayerType::Scalar;
    broken.image = grid_image();
    // placement left at its default: bounded, zero-sized.
    CHECK_TRUE(f.store.load(map, std::move(broken)));

    const MapScalarSample sample = f.store.sample_scalar(map, 10.5, 20.5);
    CHECK_EQ(status_name(sample.status), std::string("BadPlacement"));
    CHECK_NEAR(sample.value, 0.0, 1e-9);
}

TEST(MapLayer, loading_a_record_with_two_sources_is_refused) {
    // MapLayerStore::load() is the only door into the store that is not a
    // command, so it is the only way a record backed by BOTH an image and a
    // callable could exist. If one did, MapLayerData::source() would report
    // Image while sample_scalar() read the callable -- two answers to the same
    // question, with nothing to say which is wrong.
    Fixture f;
    const LayerId map = make_map_layer(f, "from disk");

    MapLayerData both;
    both.type = MapLayerType::Scalar;
    both.placement = grid_placement();
    both.image = grid_image();
    both.function = [](double, double) { return 99.0f; };
    CHECK_FALSE(record_consistent(both));
    CHECK_FALSE(f.store.load(map, both));
    CHECK_FALSE(f.store.is_bound(map));

    // The store will not hold such a record, but a caller can still build one
    // by hand, so the tie-break is pinned rather than left to whichever test
    // happens to run: source() must name the backing sample_scalar() actually
    // reads. If those two ever disagree there is nothing to say which is wrong.
    CHECK_TRUE(both.source() == MapSource::Function);
    CHECK_NEAR(sample_scalar(both, 10.5, 20.5).value, 99.0, 1e-6);

    // Either source ALONE loads, so the refusal is about the combination and
    // not about load() being broken.
    MapLayerData image_only;
    image_only.type = MapLayerType::Scalar;
    image_only.placement = grid_placement();
    image_only.image = grid_image();
    CHECK_TRUE(record_consistent(image_only));
    CHECK_TRUE(f.store.load(map, std::move(image_only)));
    CHECK_NEAR(f.store.sample_scalar(map, 10.5, 20.5).value, 1.0, 1e-6);

    MapLayerData function_only;
    function_only.type = MapLayerType::Scalar;
    function_only.placement = grid_placement();
    function_only.function = [](double, double) { return 99.0f; };
    CHECK_TRUE(record_consistent(function_only));
    CHECK_TRUE(f.store.load(map, std::move(function_only)));
    CHECK_NEAR(f.store.sample_scalar(map, 10.5, 20.5).value, 99.0, 1e-6);
}

// ============================================================================
// Masks
// ============================================================================

TEST(MapLayer, the_threshold_decides_which_texels_are_set_and_is_editable) {
    Fixture f;
    const LayerId map = make_map_layer(f, "obstacles");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Obstacle));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                       MapPlacement::rect(0.0, 0.0, 2.0, 1.0)));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map,
                                   make_f32_image(2, 1, 1, {0.4f, 0.6f})));

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    // Default threshold is 0.5, which straddles the two texels.
    CHECK_FALSE(is_obstacle(*record, 0.5, 0.5));
    CHECK_TRUE(is_obstacle(*record, 1.5, 0.5));

    MapSampling low;
    low.threshold = 0.3f;
    CHECK_TRUE(set_map_layer_sampling(f.stack, f.tree, f.store, map, low));
    const MapLayerData* lowered = f.store.find(map);
    CHECK(lowered != nullptr);
    if (lowered != nullptr) {
        CHECK_TRUE(is_obstacle(*lowered, 0.5, 0.5));
        CHECK_TRUE(is_obstacle(*lowered, 1.5, 0.5));
    }

    MapSampling high;
    high.threshold = 0.7f;
    CHECK_TRUE(set_map_layer_sampling(f.stack, f.tree, f.store, map, high));
    const MapLayerData* raised = f.store.find(map);
    CHECK(raised != nullptr);
    if (raised != nullptr) {
        CHECK_FALSE(is_obstacle(*raised, 0.5, 0.5));
        CHECK_FALSE(is_obstacle(*raised, 1.5, 0.5));
    }
}

TEST(MapLayer, the_outside_value_can_make_the_world_beyond_the_map_an_obstacle) {
    Fixture f;
    const LayerId map = make_map_layer(f, "buildable area");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Obstacle));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                       MapPlacement::rect(0.0, 0.0, 1.0, 1.0)));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map, make_f32_image(1, 1, 1, {0.0f})));

    const MapLayerData* open = f.store.find(map);
    CHECK(open != nullptr);
    if (open != nullptr) {
        // Default outside_value is 0, so off the map is buildable.
        CHECK_FALSE(is_obstacle(*open, 500.0, 500.0));
        CHECK_FALSE(is_obstacle(*open, 0.5, 0.5));
    }

    MapSampling fenced;
    fenced.outside_value = 1.0f;
    CHECK_TRUE(set_map_layer_sampling(f.stack, f.tree, f.store, map, fenced));

    const MapLayerData* walled = f.store.find(map);
    CHECK(walled != nullptr);
    if (walled == nullptr) return;

    // Everything off the map is now blocked, while the single texel inside is
    // still open -- so the change really is about the outside rule and not a
    // blanket "everything is an obstacle".
    CHECK_TRUE(is_obstacle(*walled, 500.0, 500.0));
    CHECK_FALSE(is_obstacle(*walled, 0.5, 0.5));
    CHECK_EQ(status_name(sample_mask(*walled, 500.0, 500.0).status), std::string("Outside"));
}

TEST(MapLayer, obstacle_and_water_never_answer_for_each_other) {
    Fixture f;
    const LayerId obstacles = make_grid_layer(f, MapLayerType::Obstacle);
    const MapLayerData* record = f.store.find(obstacles);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    // Value 6 is well past the default threshold, so the mask IS set here.
    CHECK_TRUE(is_obstacle(*record, 12.5, 21.5));
    // Same position, same data, different question.
    CHECK_FALSE(is_water(*record, 12.5, 21.5));

    // A Scalar layer is not a mask at all.
    Fixture g;
    const LayerId scalar = make_grid_layer(g, MapLayerType::Scalar);
    const MapLayerData* plain = g.store.find(scalar);
    CHECK(plain != nullptr);
    if (plain != nullptr) {
        CHECK_EQ(status_name(sample_mask(*plain, 12.5, 21.5).status), std::string("WrongType"));
        CHECK_FALSE(sample_mask(*plain, 12.5, 21.5).flag);
        CHECK_FALSE(is_obstacle(*plain, 12.5, 21.5));
    }
}

// ============================================================================
// Function layers
// ============================================================================

TEST(MapLayer, a_function_layer_takes_its_value_from_the_callable) {
    Fixture f;
    const LayerId map = make_map_layer(f, "field");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Scalar));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                       MapPlacement::rect(0.0, 0.0, 10.0, 10.0)));

    // Deliberately NOT symmetric in its two arguments: f(x, z) = 3x + z means a
    // sampler that passes z where x belongs gets a different number, whereas
    // anything built on x + z would pass either way round.
    CHECK_TRUE(set_map_layer_function(f.stack, f.tree, f.store, map,
                                      [](double x, double z) {
                                          return static_cast<float>(3.0 * x + z);
                                      }));

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    CHECK_TRUE(record->source() == MapSource::Function);
    CHECK_NEAR(sample_scalar(*record, 1.0, 0.0).value, 3.0, 1e-6);
    CHECK_NEAR(sample_scalar(*record, 0.0, 1.0).value, 1.0, 1e-6);
    CHECK_NEAR(sample_scalar(*record, 2.0, 5.0).value, 11.0, 1e-6);

    // The placement still bounds a function layer.
    CHECK_EQ(status_name(sample_scalar(*record, 50.0, 1.0).status), std::string("Outside"));
}

TEST(MapLayer, a_function_layer_with_no_callable_is_no_data_not_zero) {
    Fixture f;
    const LayerId map = make_map_layer(f, "field");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Scalar));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                       MapPlacement::rect(0.0, 0.0, 10.0, 10.0)));

    CHECK_EQ(status_name(f.store.sample_scalar(map, 1.0, 1.0).status), std::string("NoData"));

    // A callable that returns zero everywhere is a layer WITH data whose data
    // happens to be zero -- the function-layer half of the empty-versus-zero
    // distinction.
    CHECK_TRUE(set_map_layer_function(f.stack, f.tree, f.store, map,
                                      [](double, double) { return 0.0f; }));
    const MapScalarSample sample = f.store.sample_scalar(map, 1.0, 1.0);
    CHECK_EQ(status_name(sample.status), std::string("Ok"));
    CHECK_NEAR(sample.value, 0.0, 1e-9);

    // Clearing it returns the layer to no data, and undo brings the callable
    // back rather than leaving a dead layer behind.
    CHECK_TRUE(set_map_layer_function(f.stack, f.tree, f.store, map, nullptr));
    CHECK_EQ(status_name(f.store.sample_scalar(map, 1.0, 1.0).status), std::string("NoData"));
    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(status_name(f.store.sample_scalar(map, 1.0, 1.0).status), std::string("Ok"));
    CHECK_TRUE(f.stack.redo());
    CHECK_EQ(status_name(f.store.sample_scalar(map, 1.0, 1.0).status), std::string("NoData"));
}

TEST(MapLayer, an_unbounded_function_layer_answers_everywhere) {
    Fixture f;
    const LayerId bounded = make_map_layer(f, "local");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, bounded, MapLayerType::Water));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, bounded,
                                       MapPlacement::rect(0.0, 0.0, 10.0, 10.0)));
    CHECK_TRUE(set_map_layer_function(f.stack, f.tree, f.store, bounded,
                                      [](double, double) { return 1.0f; }));

    const LayerId global = make_map_layer(f, "global");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, global, MapLayerType::Water));
    CHECK_TRUE(
        set_map_layer_placement(f.stack, f.tree, f.store, global, MapPlacement::unbounded()));
    CHECK_TRUE(set_map_layer_function(f.stack, f.tree, f.store, global,
                                      [](double, double) { return 1.0f; }));

    const MapLayerData* local = f.store.find(bounded);
    const MapLayerData* everywhere = f.store.find(global);
    CHECK(local != nullptr);
    CHECK(everywhere != nullptr);
    if (local == nullptr || everywhere == nullptr) return;

    // Same callable, same probe, different placement: the bounded one declines.
    CHECK_FALSE(is_water(*local, 1.0e6, -1.0e6));
    CHECK_TRUE(is_water(*everywhere, 1.0e6, -1.0e6));
    CHECK_TRUE(is_water(*local, 5.0, 5.0));

    // An unbounded placement is still refused once pixels are involved.
    const LayerId raster = make_map_layer(f, "raster");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, raster, MapLayerType::Scalar));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, raster, grid_placement()));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, raster, grid_image()));
    CHECK_FALSE(
        set_map_layer_placement(f.stack, f.tree, f.store, raster, MapPlacement::unbounded()));
}

TEST(MapLayer, an_image_and_a_callable_cannot_back_the_same_layer) {
    Fixture f;
    const LayerId map = make_grid_layer(f);
    const size_t depth = f.stack.undo_depth();

    // Image first, then a callable.
    CHECK_FALSE(set_map_layer_function(f.stack, f.tree, f.store, map,
                                       [](double, double) { return 1.0f; }));
    CHECK_EQ(f.stack.undo_depth(), depth);

    const MapLayerData* still_image = f.store.find(map);
    CHECK(still_image != nullptr);
    if (still_image != nullptr) CHECK_TRUE(still_image->source() == MapSource::Image);

    // Clearing the image first makes the switch legal.
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map, MapImage{}));
    CHECK_TRUE(set_map_layer_function(f.stack, f.tree, f.store, map,
                                      [](double, double) { return 1.0f; }));
    const MapLayerData* now_function = f.store.find(map);
    CHECK(now_function != nullptr);
    if (now_function != nullptr) CHECK_TRUE(now_function->source() == MapSource::Function);

    // And the other way round: pixels are refused while the callable is there.
    CHECK_FALSE(set_map_layer_image(f.stack, f.tree, f.store, map, grid_image()));

    // A callable is never allowed on a texture layer at all.
    const LayerId texture = make_map_layer(f, "basemap");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, texture, MapLayerType::Texture));
    CHECK_FALSE(set_map_layer_function(f.stack, f.tree, f.store, texture,
                                       [](double, double) { return 1.0f; }));
}

// ============================================================================
// Texture layers
// ============================================================================

TEST(MapLayer, a_texture_answers_colour_and_refuses_the_scalar_question) {
    Fixture f;
    const LayerId map = make_map_layer(f, "basemap");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Texture));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                       MapPlacement::rect(0.0, 0.0, 2.0, 1.0)));
    // Two texels, every one of the eight components different, so a channel
    // that is read from the wrong offset shows up.
    CHECK_TRUE(set_map_layer_image(
        f.stack, f.tree, f.store, map,
        make_f32_image(2, 1, 4,
                       {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f})));

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    const MapColourSample left = sample_colour(*record, 0.5, 0.5);
    CHECK_EQ(status_name(left.status), std::string("Ok"));
    CHECK_NEAR(left.colour.r, 0.1, 1e-6);
    CHECK_NEAR(left.colour.g, 0.2, 1e-6);
    CHECK_NEAR(left.colour.b, 0.3, 1e-6);
    CHECK_NEAR(left.colour.a, 0.4, 1e-6);

    const MapColourSample right = sample_colour(*record, 1.5, 0.5);
    CHECK_NEAR(right.colour.r, 0.5, 1e-6);
    CHECK_NEAR(right.colour.a, 0.8, 1e-6);

    // A basemap has no scalar value, and inventing one would give every
    // consumer a number where it should have had a refusal.
    CHECK_EQ(status_name(sample_scalar(*record, 0.5, 0.5).status), std::string("WrongType"));
    CHECK_EQ(status_name(sample_mask(*record, 0.5, 0.5).status), std::string("WrongType"));

    // The mirror image: a scalar layer refuses the colour question.
    Fixture g;
    const LayerId scalar = make_grid_layer(g);
    const MapLayerData* plain = g.store.find(scalar);
    CHECK(plain != nullptr);
    if (plain != nullptr) {
        CHECK_EQ(status_name(sample_colour(*plain, 10.5, 20.5).status), std::string("WrongType"));
    }

    // Outside a texture gives the outside colour, not black by accident.
    MapSampling sampling;
    sampling.outside_colour = glm::vec4(0.9f, 0.8f, 0.7f, 0.6f);
    CHECK_TRUE(set_map_layer_sampling(f.stack, f.tree, f.store, map, sampling));
    const MapLayerData* tinted = f.store.find(map);
    CHECK(tinted != nullptr);
    if (tinted != nullptr) {
        const MapColourSample away = sample_colour(*tinted, 100.0, 100.0);
        CHECK_EQ(status_name(away.status), std::string("Outside"));
        CHECK_NEAR(away.colour.r, 0.9, 1e-6);
        CHECK_NEAR(away.colour.a, 0.6, 1e-6);
    }
}

TEST(MapLayer, a_three_channel_texture_reports_alpha_one) {
    Fixture f;
    const LayerId map = make_map_layer(f, "basemap");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Texture));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                       MapPlacement::rect(0.0, 0.0, 1.0, 1.0)));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map,
                                   make_f32_image(1, 1, 3, {0.25f, 0.5f, 0.75f})));

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    const MapColourSample sample = sample_colour(*record, 0.5, 0.5);
    CHECK_EQ(status_name(sample.status), std::string("Ok"));
    CHECK_NEAR(sample.colour.b, 0.75, 1e-6);
    // Opaque, and not a read one component past the end of the buffer.
    CHECK_NEAR(sample.colour.a, 1.0, 1e-9);
}

TEST(MapLayer, an_image_whose_channel_count_contradicts_the_type_is_refused) {
    Fixture f;
    const LayerId mask = make_map_layer(f, "obstacles");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, mask, MapLayerType::Obstacle));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, mask,
                                       MapPlacement::rect(0.0, 0.0, 1.0, 1.0)));
    CHECK_FALSE(set_map_layer_image(f.stack, f.tree, f.store, mask,
                                    make_f32_image(1, 1, 4, {0.f, 0.f, 0.f, 1.f})));

    const LayerId texture = make_map_layer(f, "basemap");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, texture, MapLayerType::Texture));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, texture,
                                       MapPlacement::rect(0.0, 0.0, 1.0, 1.0)));
    CHECK_FALSE(set_map_layer_image(f.stack, f.tree, f.store, texture,
                                    make_f32_image(1, 1, 1, {0.5f})));

    // Neither refusal left anything behind.
    const MapLayerData* mask_record = f.store.find(mask);
    const MapLayerData* texture_record = f.store.find(texture);
    CHECK(mask_record != nullptr);
    CHECK(texture_record != nullptr);
    if (mask_record != nullptr) CHECK_FALSE(mask_record->has_data());
    if (texture_record != nullptr) CHECK_FALSE(texture_record->has_data());
}

TEST(MapLayer, a_malformed_image_is_rejected_at_construction_and_at_install) {
    // A factory that returned a half-filled image would not fail at load; it
    // would fail as an out-of-range read in the middle of a generation run.
    CHECK_TRUE(make_f32_image(2, 2, 1, {1.0f, 2.0f, 3.0f}).empty());
    CHECK_TRUE(make_u8_image(2, 2, 1, {1, 2, 3, 4, 5}).empty());
    CHECK_TRUE(make_f32_image(2, 2, 2, {1.0f, 2.0f, 3.0f, 4.0f}).empty());  // 2 channels
    CHECK_FALSE(make_f32_image(2, 2, 1, {1.0f, 2.0f, 3.0f, 4.0f}).empty());

    // Dimensions with no pixels is malformed, not "the empty image".
    MapImage hollow;
    hollow.width = 2;
    hollow.height = 2;
    hollow.channels = 1;
    CHECK_FALSE(image_well_formed(hollow));
    CHECK_TRUE(image_well_formed(MapImage{}));

    // And the command refuses one built by hand, leaving the previous pixels in
    // place rather than half-installing.
    Fixture f;
    const LayerId map = make_grid_layer(f);
    MapImage bad;
    bad.width = 2;
    bad.height = 2;
    bad.channels = 1;
    bad.f32 = {1.0f, 2.0f, 3.0f};
    CHECK_FALSE(image_well_formed(bad));
    CHECK_FALSE(set_map_layer_image(f.stack, f.tree, f.store, map, bad));

    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record != nullptr) {
        CHECK_NEAR(sample_scalar(*record, 11.5, 20.5).value, 2.0, 1e-6);
    }
}

// ============================================================================
// Undo
// ============================================================================

TEST(MapLayer, setting_an_image_undoes_and_redoes_without_losing_pixels) {
    Fixture f;
    const LayerId map = make_map_layer(f, "field");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Scalar));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                       MapPlacement::rect(0.0, 0.0, 1.0, 1.0)));

    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map, make_f32_image(1, 1, 1, {11.f})));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map, make_f32_image(1, 1, 1, {22.f})));

    const auto value_now = [&f, map]() {
        return f.store.sample_scalar(map, 0.5, 0.5);
    };

    CHECK_NEAR(value_now().value, 22.0, 1e-6);

    // Undo past the second image, then past the first, then all the way back up
    // again. The image is SWAPPED rather than copied, and a swap that is right
    // once can still be wrong on the second pass.
    CHECK_TRUE(f.stack.undo());
    CHECK_NEAR(value_now().value, 11.0, 1e-6);
    CHECK_EQ(status_name(value_now().status), std::string("Ok"));

    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(status_name(value_now().status), std::string("NoData"));

    CHECK_TRUE(f.stack.redo());
    CHECK_NEAR(value_now().value, 11.0, 1e-6);

    CHECK_TRUE(f.stack.redo());
    CHECK_NEAR(value_now().value, 22.0, 1e-6);

    // And once more down, to catch a swap that only survives one round trip.
    CHECK_TRUE(f.stack.undo());
    CHECK_NEAR(value_now().value, 11.0, 1e-6);
}

TEST(MapLayer, unbinding_keeps_the_pixels_as_undo_state_and_says_how_big_they_are) {
    Fixture f;
    const LayerId map = make_grid_layer(f);

    const size_t bytes_before = f.stack.bytes();
    CHECK_TRUE(unbind_map_layer(f.stack, f.tree, f.store, map));
    const size_t bytes_after = f.stack.bytes();

    CHECK_FALSE(f.store.is_bound(map));
    CHECK_EQ(status_name(f.store.sample_scalar(map, 10.5, 20.5).status), std::string("NoLayer"));

    // The step must account for the image it is holding. Reporting only
    // sizeof(command) is exactly the failure Command::footprint() warns about:
    // the stack's byte bound would stop bounding anything.
    CHECK((bytes_after - bytes_before) >= sizeof(UnbindMapLayerCommand) + 6u * sizeof(float));

    // Undo brings the record back whole, pixels and placement included.
    CHECK_TRUE(f.stack.undo());
    CHECK_TRUE(f.store.is_bound(map));
    const MapScalarSample restored = f.store.sample_scalar(map, 12.5, 21.5);
    CHECK_EQ(status_name(restored.status), std::string("Ok"));
    CHECK_NEAR(restored.value, 6.0, 1e-6);

    CHECK_TRUE(f.stack.redo());
    CHECK_FALSE(f.store.is_bound(map));
}

TEST(MapLayer, dragging_a_placement_is_one_undo_step_that_returns_to_where_it_started) {
    Fixture f;
    const LayerId map = make_map_layer(f, "field");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Scalar));

    const MapPlacement start = MapPlacement::rect(0.0, 0.0, 10.0, 10.0);
    const MapPlacement middle = MapPlacement::rect(5.0, 0.0, 10.0, 10.0);
    const MapPlacement end = MapPlacement::rect(9.0, 3.0, 10.0, 10.0);

    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, start));
    f.stack.seal();
    const size_t depth_after_start = f.stack.undo_depth();

    // Two more moves, unsealed: the gesture.
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, middle));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, end));
    CHECK_EQ(f.stack.undo_depth(), depth_after_start + 1u);

    const MapLayerData* dragged = f.store.find(map);
    CHECK(dragged != nullptr);
    if (dragged != nullptr) CHECK_TRUE(dragged->placement == end);

    // One undo must return to the start of the gesture, not to its middle --
    // the classic merge bug is keeping the successor's OLD value.
    CHECK_TRUE(f.stack.undo());
    const MapLayerData* reverted = f.store.find(map);
    CHECK(reverted != nullptr);
    if (reverted != nullptr) CHECK_TRUE(reverted->placement == start);

    CHECK_TRUE(f.stack.redo());
    const MapLayerData* redone = f.store.find(map);
    CHECK(redone != nullptr);
    if (redone != nullptr) CHECK_TRUE(redone->placement == end);

    // A gesture that ends where it began must NOT merge itself into a no-op
    // step, which would be dropped on redo and read as broken undo.
    f.stack.seal();
    const size_t depth = f.stack.undo_depth();
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, middle));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, end));
    CHECK_EQ(f.stack.undo_depth(), depth + 2u);
}

TEST(MapLayer, a_sampling_change_that_changes_nothing_is_refused) {
    Fixture f;
    const LayerId map = make_grid_layer(f);
    const size_t depth = f.stack.undo_depth();

    // The record already holds default sampling.
    CHECK_FALSE(set_map_layer_sampling(f.stack, f.tree, f.store, map, MapSampling{}));
    CHECK_EQ(f.stack.undo_depth(), depth);

    MapSampling changed;
    changed.threshold = 0.25f;
    CHECK_TRUE(set_map_layer_sampling(f.stack, f.tree, f.store, map, changed));
    CHECK_EQ(f.stack.undo_depth(), depth + 1u);

    // Setting the same policy again is likewise a no-op.
    CHECK_FALSE(set_map_layer_sampling(f.stack, f.tree, f.store, map, changed));
    CHECK_EQ(f.stack.undo_depth(), depth + 1u);

    CHECK_TRUE(f.stack.undo());
    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record != nullptr) CHECK_NEAR(record->sampling.threshold, 0.5, 1e-9);
}

TEST(MapLayer, clearing_an_image_that_is_already_clear_is_refused) {
    Fixture f;
    const LayerId map = make_map_layer(f, "field");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Scalar));
    const size_t depth = f.stack.undo_depth();

    CHECK_FALSE(set_map_layer_image(f.stack, f.tree, f.store, map, MapImage{}));
    CHECK_EQ(f.stack.undo_depth(), depth);

    // Clearing a real image is not a no-op, and undo brings it back.
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, grid_placement()));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map, grid_image()));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map, MapImage{}));
    CHECK_EQ(status_name(f.store.sample_scalar(map, 10.5, 20.5).status), std::string("NoData"));
    CHECK_TRUE(f.stack.undo());
    CHECK_NEAR(f.store.sample_scalar(map, 10.5, 20.5).value, 1.0, 1e-6);
}

// ============================================================================
// The consumer's entry point
// ============================================================================

TEST(MapLayer, active_map_layers_drops_a_layer_whose_layer_was_deleted) {
    Fixture f;
    const LayerId map = make_grid_layer(f, MapLayerType::Obstacle);

    std::vector<LayerId> active = active_map_layers(f.tree, f.store, MapLayerType::Obstacle);
    CHECK_EQ(active.size(), size_t{1});
    if (!active.empty()) CHECK_EQ(active.front(), map);

    CHECK_TRUE(f.stack.execute(std::make_unique<DeleteLayerCommand>(f.tree, map)));

    // The record deliberately OUTLIVES the layer, so undoing the delete does
    // not have to reconstruct the pixels...
    CHECK_TRUE(f.store.is_bound(map));
    CHECK_NEAR(f.store.sample_scalar(map, 10.5, 20.5).value, 1.0, 1e-6);

    // ...but an orphan must not keep steering street growth.
    active = active_map_layers(f.tree, f.store, MapLayerType::Obstacle);
    CHECK_EQ(active.size(), size_t{0});

    CHECK_TRUE(f.stack.undo());
    active = active_map_layers(f.tree, f.store, MapLayerType::Obstacle);
    CHECK_EQ(active.size(), size_t{1});
    if (!active.empty()) CHECK_EQ(active.front(), map);
}

TEST(MapLayer, active_map_layers_drops_a_layer_hidden_by_an_ancestor) {
    Fixture f;
    const LayerId group = make_layer(f.tree, f.stack, LayerKind::Group, "analysis");
    const LayerId map = make_map_layer(f, "obstacles", group);
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Obstacle));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, grid_placement()));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map, grid_image()));

    CHECK_EQ(active_map_layers(f.tree, f.store, MapLayerType::Obstacle).size(), size_t{1});

    CHECK_TRUE(f.stack.execute(std::make_unique<SetLayerVisibleCommand>(f.tree, group, false)));

    // The map layer's OWN flag is still true. A filter that read own_visible
    // would keep it, which is the bug this test exists to catch.
    const Layer* layer = f.tree.find(map);
    CHECK(layer != nullptr);
    if (layer != nullptr) CHECK_TRUE(layer->own_visible);
    CHECK_FALSE(f.tree.effective_visible(map));

    CHECK_EQ(active_map_layers(f.tree, f.store, MapLayerType::Obstacle).size(), size_t{0});

    CHECK_TRUE(f.stack.undo());
    CHECK_EQ(active_map_layers(f.tree, f.store, MapLayerType::Obstacle).size(), size_t{1});
}

TEST(MapLayer, active_map_layers_drops_an_empty_record_but_keeps_an_all_zero_one) {
    Fixture f;

    const LayerId empty = make_map_layer(f, "not loaded yet");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, empty, MapLayerType::Obstacle));

    const LayerId zeros = make_map_layer(f, "loaded, all clear");
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, zeros, MapLayerType::Obstacle));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, zeros, grid_placement()));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, zeros,
                                   make_f32_image(3, 2, 1, {0.f, 0.f, 0.f, 0.f, 0.f, 0.f})));

    // Both are bound, so the filter cannot be passing on is_bound() alone.
    CHECK_EQ(f.store.bound_layers_of_type(MapLayerType::Obstacle).size(), size_t{2});

    const std::vector<LayerId> active = active_map_layers(f.tree, f.store, MapLayerType::Obstacle);
    CHECK_EQ(active.size(), size_t{1});
    if (!active.empty()) CHECK_EQ(active.front(), zeros);
}

TEST(MapLayer, active_map_layers_follows_tree_order_and_filters_by_type) {
    Fixture f;
    const LayerId first = make_grid_layer(f, MapLayerType::Obstacle);
    const LayerId water = make_grid_layer(f, MapLayerType::Water);
    const LayerId second = make_grid_layer(f, MapLayerType::Obstacle);

    std::vector<LayerId> obstacles = active_map_layers(f.tree, f.store, MapLayerType::Obstacle);
    CHECK_EQ(obstacles.size(), size_t{2});
    if (obstacles.size() == 2u) {
        CHECK_EQ(obstacles[0], first);
        CHECK_EQ(obstacles[1], second);
    }

    const std::vector<LayerId> waters = active_map_layers(f.tree, f.store, MapLayerType::Water);
    CHECK_EQ(waters.size(), size_t{1});
    if (!waters.empty()) CHECK_EQ(waters.front(), water);

    // Reorder the tree. Ids are unchanged, so a result built by walking the
    // store -- which is keyed by id -- would not move, while the panel order the
    // user sees has.
    CHECK_TRUE(f.stack.execute(std::make_unique<ReorderLayerCommand>(f.tree, second, 0)));
    obstacles = active_map_layers(f.tree, f.store, MapLayerType::Obstacle);
    CHECK_EQ(obstacles.size(), size_t{2});
    if (obstacles.size() == 2u) {
        CHECK_EQ(obstacles[0], second);
        CHECK_EQ(obstacles[1], first);
    }
    // The ids really are in the other order, so the check above is a statement
    // about ordering and not about which two layers came back.
    CHECK((first < second));
}

TEST(MapLayer, dropping_records_for_deleted_layers_removes_only_the_dead) {
    Fixture f;
    const LayerId kept = make_grid_layer(f, MapLayerType::Obstacle);
    const LayerId doomed = make_grid_layer(f, MapLayerType::Water);

    CHECK_TRUE(f.stack.execute(std::make_unique<DeleteLayerCommand>(f.tree, doomed)));
    CHECK_EQ(f.store.size(), size_t{2});

    f.stack.clear();
    CHECK_EQ(f.store.drop_records_for_deleted_layers(f.tree), size_t{1});

    CHECK_TRUE(f.store.is_bound(kept));
    CHECK_FALSE(f.store.is_bound(doomed));
    CHECK_EQ(f.store.size(), size_t{1});

    // Running it again drops nothing, so it is not simply emptying the store.
    CHECK_EQ(f.store.drop_records_for_deleted_layers(f.tree), size_t{0});
    CHECK_TRUE(f.store.is_bound(kept));
}

TEST(MapLayer, the_hoisted_record_and_the_id_lookup_give_the_same_answers) {
    Fixture f;
    const LayerId map = make_grid_layer(f);
    const MapLayerData* record = f.store.find(map);
    CHECK(record != nullptr);
    if (record == nullptr) return;

    // Inside, on an edge, and well outside: the two paths must agree on the
    // status as well as the value, or the cheap path is not the same function.
    const double probes[4][2] = {{10.5, 20.5}, {12.5, 21.5}, {10.0, 20.0}, {-100.0, 900.0}};
    for (const auto& probe : probes) {
        const MapScalarSample hoisted = sample_scalar(*record, probe[0], probe[1]);
        const MapScalarSample looked_up = f.store.sample_scalar(map, probe[0], probe[1]);
        CHECK_EQ(status_name(hoisted.status), status_name(looked_up.status));
        CHECK_NEAR(hoisted.value, looked_up.value, 1e-9);
    }

    // An id nobody bound is NoLayer through the store, which is the one answer
    // the hoisted path cannot give because there is no record to hold.
    CHECK_EQ(status_name(f.store.sample_scalar(LayerId{4242}, 10.5, 20.5).status),
             std::string("NoLayer"));
}

TEST(MapLayer, editing_a_map_layer_whose_layer_is_gone_is_refused) {
    Fixture f;
    const LayerId map = make_grid_layer(f);
    CHECK_TRUE(f.stack.execute(std::make_unique<DeleteLayerCommand>(f.tree, map)));

    // The orphaned record is still there, but it is no longer a layer anybody
    // can point at, so an edit that reached it would be an edit to something
    // the user cannot see.
    const size_t depth = f.stack.undo_depth();
    CHECK_FALSE(set_map_layer_image(f.stack, f.tree, f.store, map, MapImage{}));
    CHECK_FALSE(set_map_layer_sampling(f.stack, f.tree, f.store, map, MapSampling{}));
    CHECK_FALSE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                        MapPlacement::rect(0.0, 0.0, 1.0, 1.0)));
    CHECK_FALSE(unbind_map_layer(f.stack, f.tree, f.store, map));
    CHECK_EQ(f.stack.undo_depth(), depth);

    // Undoing the delete makes all of it legal again.
    CHECK_TRUE(f.stack.undo());
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map,
                                       MapPlacement::rect(0.0, 0.0, 1.0, 1.0)));
}

TEST(MapLayer, creating_binding_and_loading_in_one_transaction_undoes_as_one_step) {
    Fixture f;

    f.stack.begin_transaction("Import obstacle map");
    auto create = std::make_unique<CreateLayerCommand>(f.tree, LayerKind::Map, "obstacles");
    const LayerId map = create->layer();
    CHECK_TRUE(f.stack.execute(std::move(create)));
    CHECK_TRUE(bind_map_layer(f.stack, f.tree, f.store, map, MapLayerType::Obstacle));
    CHECK_TRUE(set_map_layer_placement(f.stack, f.tree, f.store, map, grid_placement()));
    CHECK_TRUE(set_map_layer_image(f.stack, f.tree, f.store, map, grid_image()));
    f.stack.commit_transaction();

    CHECK_EQ(f.stack.undo_depth(), size_t{1});
    CHECK_EQ(active_map_layers(f.tree, f.store, MapLayerType::Obstacle).size(), size_t{1});

    // One undo takes the whole import back: the layer, the record and the
    // pixels. This is the pattern A1 documents for configuring a layer inside
    // the transaction that creates it.
    CHECK_TRUE(f.stack.undo());
    CHECK_FALSE(f.tree.contains(map));
    CHECK_FALSE(f.store.is_bound(map));
    CHECK_EQ(active_map_layers(f.tree, f.store, MapLayerType::Obstacle).size(), size_t{0});

    CHECK_TRUE(f.stack.redo());
    CHECK_TRUE(f.tree.contains(map));
    CHECK_NEAR(f.store.sample_scalar(map, 12.5, 21.5).value, 6.0, 1e-6);
}
