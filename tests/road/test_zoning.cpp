// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_zoning.cpp
 * @brief Zoning: inference from landuse, hand painting, and which one wins
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Two things are under test and they fail in different ways, so they are tested
 * differently.
 *
 * **Precedence** fails silently and identically to success. A store that reads
 * the right zone off the wrong rung looks correct until a rule re-runs, so every
 * precedence test below arranges a DIFFERENT zone on each rung it involves --
 * Residential inferred, Commercial painted, Industrial on the layer, Park on the
 * default. Resolution reaching the wrong rung then returns a wrong zone rather
 * than the right one by luck. Several tests also assert which rung answered, not
 * only the value, because that is the question a user is really asking.
 *
 * **Overlap arithmetic** fails by picking a plausible winner for the wrong
 * reason. So the fixtures are built so that each wrong algorithm gives a
 * DIFFERENT answer from the right one:
 *
 *   - "largest wins" versus "first in the vector wins": the same fixture is run
 *     with the sources in both orders, and separately with the larger area moved
 *     to the second slot.
 *   - "pool by zone" versus "rank by polygon": two 30% residential areas against
 *     one 40% commercial one. Per-polygon ranking says commercial; pooling says
 *     residential.
 *   - "union the pieces" versus "sum the areas": two IDENTICAL 40% residential
 *     areas against one 50% commercial one. Summing says residential at 80%;
 *     unioning says commercial.
 *   - "clip against the polygon" versus "test the outer ring": a block sitting
 *     inside the hole of a landuse area.
 *
 * **Boundaries and tie-breaks are pinned ON the boundary, and in both orders.**
 * A mutation run proved the cost of not doing that: ten single-line defects
 * survived an earlier version of this suite, and six of them were behaviours the
 * header states in prose. A fixture near a threshold cannot tell `<` from `<=`;
 * a tie fixture listed in one order cannot tell "lower id wins" from "first
 * wins". So the boundary tests below use coverages that are exact binary
 * fractions -- 0.25, 0.625, 0.375 -- and every tie fixture is run with its
 * sources reversed.
 *
 * **Quantisation is tested where the rules disagree.** Every other coordinate in
 * this file is a whole number of millimetres, where rounding and truncation give
 * the same answer and the tie epsilon can never fire. Three fixtures deliberately
 * are not: a ring at 99.9996 m, a polygon with a 1 mm chamfer, and a ring
 * 2000 km from the origin.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests Zoning
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/zoning.hpp"
#include "osm/types.hpp"
#include "scene/attributes.hpp"
#include "scene/command.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

using stratum::osm::Area;
using stratum::osm::AreaType;
using stratum::osm::road::apply_inferred_zones;
using stratum::osm::road::Block;
using stratum::osm::road::clear_inferred_zone;
using stratum::osm::road::clear_painted_zone;
using stratum::osm::road::infer_target_zone;
using stratum::osm::road::infer_zone;
using stratum::osm::road::infer_zones;
using stratum::osm::road::inferred_zone_target;
using stratum::osm::road::kAllZoneTypes;
using stratum::osm::road::kZoneTypeCount;
using stratum::osm::road::load_inferred_zone;
using stratum::osm::road::load_inferred_zones;
using stratum::osm::road::paint_zone;
using stratum::osm::road::painted_zone_target;
using stratum::osm::road::peek_zone;
using stratum::osm::road::resolve_zone;
using stratum::osm::road::set_default_zone;
using stratum::osm::road::set_inferred_zone;
using stratum::osm::road::set_layer_zone;
using stratum::osm::road::zone_target_from_block;
using stratum::osm::road::zone_from_area_type;
using stratum::osm::road::zone_key;
using stratum::osm::road::zone_outcome_name;
using stratum::osm::road::zone_sources_from_areas;
using stratum::osm::road::zone_status_name;
using stratum::osm::road::zone_type_from_name;
using stratum::osm::road::zone_type_name;
using stratum::osm::road::ZoneInference;
using stratum::osm::road::ZoneOutcome;
using stratum::osm::road::ZoneQuery;
using stratum::osm::road::ZoneSourceArea;
using stratum::osm::road::ZoneTarget;
using stratum::osm::road::ZoneType;
using stratum::osm::road::ZoningConfig;
using stratum::osm::road::ZoningReport;
using stratum::scene::AttributeObject;
using stratum::scene::AttributeStore;
using stratum::scene::AttributeValue;
using stratum::scene::attribute_source_name;
using stratum::scene::CommandStack;
using stratum::scene::kNoLayer;
using stratum::scene::LayerRef;

/// A layer handle. Opaque to the store, so any non-zero number does.
constexpr LayerRef kLayerA = 11u;

// ============================================================================
// Readable check helpers
//
// Names rather than enumerators, so a failure reads "actual: Object  expected:
// User" instead of "<unprintable>". Same idiom as tests/scene/test_attributes.cpp.
// ============================================================================

// These return std::string rather than the std::string_view the named functions
// hand back. The view would be perfectly safe -- every one of those names is a
// string literal -- but GCC's -Wdangling-reference cannot see that, and it fires
// on any reference-like return taken from a query, burying real warnings under
// twenty false ones. Owning the bytes ends the argument, and a copy in a test
// costs nothing.
std::string zone_of(const ZoneQuery& query) { return std::string(zone_type_name(query.zone)); }
std::string status_of(const ZoneQuery& query) { return std::string(zone_status_name(query.status)); }
std::string source_of(const ZoneQuery& query) {
    return std::string(attribute_source_name(query.source));
}
std::string outcome_of(const ZoneInference& inference) {
    return std::string(zone_outcome_name(inference.outcome));
}

// ============================================================================
// Geometry fixtures
// ============================================================================

/// Axis-aligned rectangle as an anticlockwise ring, first point not repeated
std::vector<glm::dvec2> rect(double x0, double y0, double x1, double y1) {
    return {glm::dvec2(x0, y0), glm::dvec2(x1, y0), glm::dvec2(x1, y1), glm::dvec2(x0, y1)};
}

/// The block every geometric test uses: 100 m square at the origin, 10000 m^2
std::vector<glm::dvec2> unit_block() { return rect(0.0, 0.0, 100.0, 100.0); }

/// The same rectangle wound CLOCKWISE, which is the orientation Block::holes uses
std::vector<glm::dvec2> hole_rect(double x0, double y0, double x1, double y1) {
    std::vector<glm::dvec2> ring = rect(x0, y0, x1, y1);
    std::reverse(ring.begin(), ring.end());
    return ring;
}

/// Compare two rings point by point. CHECK_EQ on a glm::dvec2 prints
/// "<unprintable>" because GLM has no stream operator without its experimental
/// headers, and a size-only check passes for a ring of the right length full of
/// the wrong numbers.
void check_ring_equals(const std::vector<glm::dvec2>& actual,
                       const std::vector<glm::dvec2>& expected) {
    CHECK_EQ(actual.size(), expected.size());
    const size_t common = std::min(actual.size(), expected.size());
    for (size_t i = 0; i < common; ++i) {
        CHECK_NEAR(actual[i].x, expected[i].x, 0.0);
        CHECK_NEAR(actual[i].y, expected[i].y, 0.0);
    }
}

ZoneSourceArea source(ZoneType zone, std::vector<glm::dvec2> ring, int64_t id) {
    ZoneSourceArea area;
    area.zone = zone;
    area.ring = std::move(ring);
    area.source_id = id;
    return area;
}

} // namespace

// ============================================================================
// The vocabulary
// ============================================================================

TEST(Zoning, every_zone_type_has_a_distinct_canonical_name_that_parses_back) {
    // Totality and injectivity in one pass. A new enumerator added to ZoneType
    // and forgotten in zone_type_name() falls through to "unknown", which this
    // catches as a duplicate name rather than as a missing one -- the fall-through
    // is the failure mode that would otherwise silently make two zones the same
    // string and therefore the same saved value.
    CHECK_EQ(kAllZoneTypes.size(), kZoneTypeCount);

    std::vector<std::string> seen;
    for (const ZoneType zone : kAllZoneTypes) {
        const std::string name(zone_type_name(zone));
        CHECK_FALSE(name.empty());

        for (const std::string& earlier : seen) {
            CHECK(earlier != name);
        }
        seen.push_back(name);

        const std::optional<ZoneType> parsed = zone_type_from_name(name);
        CHECK_TRUE(parsed.has_value());
        if (parsed.has_value()) {
            CHECK_EQ(zone_type_name(*parsed), zone_type_name(zone));
        }
    }
    CHECK_EQ(seen.size(), kZoneTypeCount);
}

TEST(Zoning, unknown_parses_as_a_zone_but_a_misspelling_does_not) {
    // The distinction the whole "unzoned is not zoned as unknown" contract rests
    // on: "unknown" is a successful parse, not a parse failure.
    const std::optional<ZoneType> unknown = zone_type_from_name("unknown");
    CHECK_TRUE(unknown.has_value());
    if (unknown.has_value()) {
        CHECK_EQ(zone_type_name(*unknown), std::string("unknown"));
    }

    CHECK_FALSE(zone_type_from_name("resedential").has_value());
    CHECK_FALSE(zone_type_from_name("").has_value());

    // Case-sensitive on purpose. A lenient parse would silently normalise a
    // broken importer's output and hide it forever.
    CHECK_FALSE(zone_type_from_name("Residential").has_value());
}

TEST(Zoning, each_area_type_maps_to_the_zone_named_here_and_unknown_maps_to_none) {
    // Written out per enumerator rather than as a loop over a table built from
    // the function itself, which would assert nothing. Water is included
    // deliberately: dropping water from the source set lets a 10% residential
    // strip on the bank zone a block that is 90% river.
    const auto expect = [](AreaType type, ZoneType zone) {
        const std::optional<ZoneType> mapped = zone_from_area_type(type);
        CHECK_TRUE(mapped.has_value());
        if (mapped.has_value()) {
            CHECK_EQ(zone_type_name(*mapped), zone_type_name(zone));
        }
    };

    expect(AreaType::Water, ZoneType::Water);
    expect(AreaType::Park, ZoneType::Park);
    expect(AreaType::Forest, ZoneType::Forest);
    expect(AreaType::Grass, ZoneType::Grass);
    expect(AreaType::Parking, ZoneType::Parking);
    expect(AreaType::Commercial, ZoneType::Commercial);
    expect(AreaType::Residential, ZoneType::Residential);
    expect(AreaType::Industrial, ZoneType::Industrial);
    expect(AreaType::Farmland, ZoneType::Farmland);
    expect(AreaType::Cemetery, ZoneType::Cemetery);

    // An area the parser could not classify must zone nothing. Mapping it to
    // ZoneType::Unknown would let every unclassified polygon actively claim the
    // land under it and beat a real landuse area beside it.
    CHECK_FALSE(zone_from_area_type(AreaType::Unknown).has_value());
}

TEST(Zoning, unclassifiable_and_degenerate_areas_are_dropped_from_the_source_set) {
    std::vector<Area> areas;

    Area good;
    good.osm_id = 7;
    good.type = AreaType::Residential;
    good.polygon = rect(0.0, 0.0, 200.0, 200.0);
    // A courtyard. It has to survive the conversion: an Area that arrives with a
    // hole and leaves without one silently zones the land inside the hole, and
    // the only place that shows up is in the finished city.
    good.holes.push_back(rect(20.0, 20.0, 180.0, 180.0));
    areas.push_back(good);

    Area unclassified;
    unclassified.osm_id = 8;
    unclassified.type = AreaType::Unknown;
    unclassified.polygon = rect(0.0, 0.0, 10.0, 10.0);
    areas.push_back(unclassified);

    Area sliver;
    sliver.osm_id = 9;
    sliver.type = AreaType::Park;
    sliver.polygon = {glm::dvec2(0.0, 0.0), glm::dvec2(1.0, 0.0)};
    areas.push_back(sliver);

    const std::vector<ZoneSourceArea> sources = zone_sources_from_areas(areas);
    CHECK_EQ(sources.size(), size_t{1});
    if (sources.size() == 1) {
        CHECK_EQ(zone_type_name(sources[0].zone), std::string("residential"));
        CHECK_EQ(sources[0].source_id, int64_t{7});
        CHECK_EQ(sources[0].holes.size(), size_t{1});

        // Asserted through the solver and not only by counting rings: a hole that
        // is copied across but never reaches the clip reads identically to one
        // that is copied and used, and a ring count cannot tell them apart.
        const ZoneInference in_the_courtyard =
            infer_zone(rect(50.0, 50.0, 150.0, 150.0), sources);
        CHECK_EQ(outcome_of(in_the_courtyard), std::string("NoOverlap"));
    }
}

// ============================================================================
// From a Block to a target
//
// zone_target_from_block() is the only bridge between C1's faces and this file,
// and for a while it had no test at all. Two mutations proved what that cost: a
// version returning an empty ring and a version returning a default-constructed
// AttributeObject both passed the whole suite, and the version that shipped
// silently dropped Block::holes.
// ============================================================================

TEST(Zoning, a_target_built_from_a_block_carries_its_ring_its_holes_and_its_record) {
    AttributeStore store;
    const AttributeObject record = store.create_object();

    Block block;
    block.ring = unit_block();
    // Wound clockwise, because blocks.hpp says Block::holes are, and a test that
    // fed anticlockwise holes would not be testing what extract_blocks() emits.
    block.holes.push_back(hole_rect(10.0, 10.0, 90.0, 90.0));
    block.holes.push_back(hole_rect(95.0, 1.0, 99.0, 5.0));
    block.area = 10000.0 - 6400.0 - 16.0;

    const ZoneTarget target = zone_target_from_block(block, record);

    // The handle. A target that lost it names no slot, so every zone the solve
    // decides for this block is dropped by both appliers and the block comes out
    // unzoned -- which looks exactly like a block no landuse area touched.
    CHECK_TRUE(target.object == record);
    CHECK_TRUE(target.object.valid());

    check_ring_equals(target.ring, block.ring);

    CHECK_EQ(target.holes.size(), block.holes.size());
    const size_t holes = std::min(target.holes.size(), block.holes.size());
    for (size_t h = 0; h < holes; ++h) {
        check_ring_equals(target.holes[h], block.holes[h]);
    }
}

TEST(Zoning, the_land_inside_a_block_hole_is_not_the_blocks_land_and_does_not_zone_it) {
    // A block with a cul-de-sac bulb cut out of it. blocks.hpp is explicit that
    // the land inside the hole is its own block and that Block::area already has
    // it subtracted, so a target carrying the ring alone is a DIFFERENT parcel.
    //
    // The two sources are arranged so that honouring the hole and ignoring it
    // give different ZONES, not merely different numbers -- a coverage assertion
    // alone would read as a tolerance argument. The park fills the hole exactly,
    // so it contributes nothing to this block. The residential strip is 1000 of
    // the 3600 m^2 the block owns, which is 27.8% and clears the quarter floor.
    // Ignore the hole and the park covers 64% of a 10000 m^2 parcel and wins.
    AttributeStore store;

    Block block;
    block.ring = unit_block();
    block.holes.push_back(hole_rect(10.0, 10.0, 90.0, 90.0));
    block.area = 3600.0;

    const ZoneTarget target = zone_target_from_block(block, store.create_object());
    const std::vector<ZoneSourceArea> sources = {
        source(ZoneType::Park, rect(10.0, 10.0, 90.0, 90.0), 1),
        source(ZoneType::Residential, rect(0.0, 0.0, 100.0, 10.0), 2),
    };

    const ZoneInference inference = infer_target_zone(target, sources);
    CHECK_EQ(outcome_of(inference), std::string("Inferred"));
    CHECK_EQ(zone_type_name(inference.zone), std::string("residential"));
    CHECK_EQ(inference.source_id, int64_t{2});
    // The denominator is the block's own area, so the solver and Block::area
    // agree about how much land is being zoned.
    CHECK_NEAR(inference.target_area, block.area, 1e-6);
    CHECK_NEAR(inference.coverage, 1000.0 / 3600.0, 1e-9);
    // The park contributed nothing at all, so it is not even the runner-up.
    CHECK_EQ(zone_type_name(inference.runner_up), std::string("unknown"));
    CHECK_NEAR(inference.runner_up_coverage, 0.0, 1e-12);

    // The batch path has its own loop, so assert it reaches the same answer
    // rather than trusting that the two cannot drift.
    const ZoningReport report = infer_zones({target}, sources);
    CHECK_EQ(report.inferences.size(), size_t{1});
    if (report.inferences.size() == 1) {
        CHECK_EQ(zone_type_name(report.inferences[0].zone), std::string("residential"));
        CHECK_NEAR(report.inferences[0].target_area, block.area, 1e-6);
    }
}

// ============================================================================
// Precedence: which rung answered
// ============================================================================

TEST(Zoning, a_painted_zone_beats_the_inferred_one) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject block = store.create_object();

    CHECK_TRUE(load_inferred_zone(store, block, ZoneType::Residential));
    CHECK_TRUE(paint_zone(stack, store, block, ZoneType::Commercial));

    const ZoneQuery query = resolve_zone(store, block, kNoLayer);
    CHECK_EQ(status_of(query), std::string("Zoned"));
    // A different zone per rung, so reaching the wrong rung is a wrong VALUE and
    // not a pass by coincidence.
    CHECK_EQ(zone_of(query), std::string("commercial"));
    CHECK_EQ(source_of(query), std::string("User"));
}

TEST(Zoning, clearing_the_paint_falls_back_to_the_inferred_zone_and_no_further) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject block = store.create_object();

    // Four rungs, four different zones. A fall-through that overshoots the
    // Object rung lands on "industrial" or "park" and is caught.
    CHECK_TRUE(set_default_zone(stack, store, ZoneType::Park));
    CHECK_TRUE(set_layer_zone(stack, store, kLayerA, ZoneType::Industrial));
    CHECK_TRUE(load_inferred_zone(store, block, ZoneType::Residential));
    CHECK_TRUE(paint_zone(stack, store, block, ZoneType::Commercial));

    CHECK_EQ(zone_of(resolve_zone(store, block, kLayerA)), std::string("commercial"));

    CHECK_TRUE(clear_painted_zone(stack, store, block));

    const ZoneQuery after = resolve_zone(store, block, kLayerA);
    CHECK_EQ(zone_of(after), std::string("residential"));
    CHECK_EQ(source_of(after), std::string("Object"));

    // Clearing a zone that is not there is not an edit, so it records no step.
    CHECK_FALSE(clear_painted_zone(stack, store, block));
}

TEST(Zoning, clearing_the_inferred_zone_reveals_the_layer_and_clearing_twice_is_not_an_edit) {
    // clear_inferred_zone() had no test at all. It is the call a panel makes to
    // un-zone a block without painting anything, and a regression that cleared
    // the wrong rung would destroy a hand-painted zone instead.
    AttributeStore store;
    CommandStack stack;
    const AttributeObject block = store.create_object();

    // A different zone on each rung, so clearing the wrong one shows up as a
    // wrong VALUE rather than passing by coincidence.
    CHECK_TRUE(set_layer_zone(stack, store, kLayerA, ZoneType::Industrial));
    CHECK_TRUE(paint_zone(stack, store, block, ZoneType::Commercial));
    CHECK_TRUE(set_inferred_zone(stack, store, block, ZoneType::Forest));

    const size_t depth_before = stack.undo_depth();
    CHECK_TRUE(clear_inferred_zone(stack, store, block));
    CHECK_EQ(stack.undo_depth(), depth_before + 1);

    // The paint is on the User rung and must be untouched; ask for it directly
    // rather than through resolve_zone(), which would report it either way.
    CHECK_EQ(zone_of(peek_zone(store, painted_zone_target(store, block))),
             std::string("commercial"));
    // And the Object rung really is empty now, so the layer shows through.
    CHECK_EQ(status_of(peek_zone(store, inferred_zone_target(store, block))),
             std::string("Unzoned"));

    CHECK_TRUE(clear_painted_zone(stack, store, block));
    const ZoneQuery after = resolve_zone(store, block, kLayerA);
    CHECK_EQ(zone_of(after), std::string("industrial"));
    CHECK_EQ(source_of(after), std::string("Layer"));

    // Clearing what is not there is not an edit, so it records no step -- a menu
    // entry that changes nothing reads as broken undo.
    const size_t depth_now = stack.undo_depth();
    CHECK_FALSE(clear_inferred_zone(stack, store, block));
    CHECK_EQ(stack.undo_depth(), depth_now);
}

TEST(Zoning, a_layer_zone_applies_only_where_the_object_has_none) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject inferred_block = store.create_object();
    const AttributeObject bare_block = store.create_object();

    CHECK_TRUE(set_layer_zone(stack, store, kLayerA, ZoneType::Industrial));
    CHECK_TRUE(load_inferred_zone(store, inferred_block, ZoneType::Residential));

    const ZoneQuery on_object = resolve_zone(store, inferred_block, kLayerA);
    CHECK_EQ(zone_of(on_object), std::string("residential"));
    CHECK_EQ(source_of(on_object), std::string("Object"));

    const ZoneQuery from_layer = resolve_zone(store, bare_block, kLayerA);
    CHECK_EQ(zone_of(from_layer), std::string("industrial"));
    CHECK_EQ(source_of(from_layer), std::string("Layer"));
    CHECK_EQ(from_layer.layer, kLayerA);

    // The same object, resolved without a layer, must lose the inheritance
    // rather than keep it. Passing kNoLayer is how a caller asks for that.
    CHECK_EQ(status_of(resolve_zone(store, bare_block, kNoLayer)), std::string("Unzoned"));
}

TEST(Zoning, undo_restores_the_inferred_zone_and_redo_restores_the_paint) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject block = store.create_object();

    CHECK_TRUE(load_inferred_zone(store, block, ZoneType::Residential));
    CHECK_TRUE(paint_zone(stack, store, block, ZoneType::Commercial));
    CHECK_EQ(zone_of(resolve_zone(store, block, kNoLayer)), std::string("commercial"));

    CHECK_TRUE(stack.undo());
    const ZoneQuery undone = resolve_zone(store, block, kNoLayer);
    CHECK_EQ(zone_of(undone), std::string("residential"));
    CHECK_EQ(source_of(undone), std::string("Object"));

    CHECK_TRUE(stack.redo());
    const ZoneQuery redone = resolve_zone(store, block, kNoLayer);
    CHECK_EQ(zone_of(redone), std::string("commercial"));
    CHECK_EQ(source_of(redone), std::string("User"));
}

// ============================================================================
// Unzoned is not "zoned as unknown"
// ============================================================================

TEST(Zoning, unzoned_and_zoned_as_unknown_read_the_same_zone_and_different_statuses) {
    AttributeStore store;
    CommandStack stack;
    const AttributeObject never_zoned = store.create_object();
    const AttributeObject marked_unknown = store.create_object();

    CHECK_TRUE(paint_zone(stack, store, marked_unknown, ZoneType::Unknown));

    const ZoneQuery bare = resolve_zone(store, never_zoned, kNoLayer);
    const ZoneQuery marked = resolve_zone(store, marked_unknown, kNoLayer);

    // The trap this test exists for: the ZONE field is identical on both, so a
    // caller reading only `zone` cannot tell them apart. Assert that they do
    // read the same, so the test fails if someone "fixes" it by inventing a
    // ZoneType::None -- which would put the null back in the enum.
    CHECK_EQ(zone_of(bare), zone_of(marked));
    CHECK_EQ(zone_of(bare), std::string("unknown"));

    // What actually distinguishes them.
    CHECK_EQ(status_of(bare), std::string("Unzoned"));
    CHECK_EQ(status_of(marked), std::string("Zoned"));
    CHECK_FALSE(bare.zoned());
    CHECK_TRUE(marked.zoned());
    CHECK_EQ(source_of(bare), std::string("None"));
    CHECK_EQ(source_of(marked), std::string("User"));
    CHECK_TRUE(bare.raw.empty());
    CHECK_EQ(marked.raw, std::string("unknown"));
}

TEST(Zoning, a_zone_read_in_a_document_that_never_zoned_anything_is_unzoned) {
    // No zone_key() call anywhere here, so the key is never interned. This is the
    // path a viewport takes on a freshly imported extract, and it must answer
    // Unzoned rather than fail.
    AttributeStore store;
    const AttributeObject block = store.create_object();

    const ZoneQuery query = resolve_zone(store, block, kLayerA);
    CHECK_EQ(status_of(query), std::string("Unzoned"));
    CHECK_EQ(source_of(query), std::string("None"));
    CHECK_EQ(store.key_count(), size_t{0});
}

TEST(Zoning, a_number_in_the_zoning_slot_is_unrecognised_and_reports_what_it_found) {
    AttributeStore store;
    const AttributeObject block = store.create_object();

    // The shape of a broken importer or a hand-edited save file.
    CHECK_TRUE(store.load_value(inferred_zone_target(store, block), AttributeValue::from_double(7)));

    const ZoneQuery query = resolve_zone(store, block, kNoLayer);
    CHECK_EQ(status_of(query), std::string("Unrecognised"));
    CHECK_FALSE(query.zoned());
    // Reported, not converted: a caller seeing Unzoned here would go hunting for
    // a missing inference instead of for the code that wrote a double.
    CHECK_EQ(query.raw, std::string("7"));
    CHECK_EQ(source_of(query), std::string("Object"));
}

TEST(Zoning, a_misspelled_zone_is_unrecognised_and_keeps_its_text) {
    AttributeStore store;
    const AttributeObject block = store.create_object();

    CHECK_TRUE(store.load_value(inferred_zone_target(store, block),
                                AttributeValue::from_string("resedential")));

    const ZoneQuery query = resolve_zone(store, block, kNoLayer);
    CHECK_EQ(status_of(query), std::string("Unrecognised"));
    // Unquoted, so an inspector can show it and a caller can compare it.
    CHECK_EQ(query.raw, std::string("resedential"));
    // Distinct from both other failure states: it is not Unzoned, and its zone
    // is not a real classification.
    CHECK_FALSE(query.zoned());
}

// ============================================================================
// Inference: overlap arithmetic
// ============================================================================

TEST(Zoning, a_block_inside_one_landuse_area_takes_its_type_at_full_coverage) {
    const std::vector<ZoneSourceArea> sources = {
        source(ZoneType::Residential, rect(-50.0, -50.0, 150.0, 150.0), 1)};

    const ZoneInference inference = infer_zone(unit_block(), sources);
    CHECK_EQ(outcome_of(inference), std::string("Inferred"));
    CHECK_EQ(zone_type_name(inference.zone), std::string("residential"));
    CHECK_NEAR(inference.coverage, 1.0, 1e-9);
    CHECK_NEAR(inference.target_area, 10000.0, 1e-6);
    CHECK_EQ(inference.source_id, int64_t{1});
    // Nothing else competed, so nothing is contested.
    CHECK_FALSE(inference.contested);
    CHECK_NEAR(inference.runner_up_coverage, 0.0, 1e-12);
}

TEST(Zoning, the_larger_overlap_wins_whichever_order_the_sources_arrive_in) {
    const ZoneSourceArea sixty = source(ZoneType::Residential, rect(0.0, 0.0, 60.0, 100.0), 1);
    const ZoneSourceArea forty = source(ZoneType::Commercial, rect(60.0, 0.0, 100.0, 100.0), 2);

    const ZoneInference forwards = infer_zone(unit_block(), {sixty, forty});
    const ZoneInference backwards = infer_zone(unit_block(), {forty, sixty});

    // Run both ways round. "First in the vector wins" passes one and fails the
    // other, which is exactly what one ordering alone could not detect.
    for (const ZoneInference& inference : {forwards, backwards}) {
        CHECK_EQ(outcome_of(inference), std::string("Inferred"));
        CHECK_EQ(zone_type_name(inference.zone), std::string("residential"));
        CHECK_NEAR(inference.coverage, 0.6, 1e-6);
        CHECK_EQ(zone_type_name(inference.runner_up), std::string("commercial"));
        CHECK_NEAR(inference.runner_up_coverage, 0.4, 1e-6);
        CHECK_EQ(inference.source_id, int64_t{1});
        // 0.6 - 0.4 = 0.2, clear of the 0.1 default margin.
        CHECK_FALSE(inference.contested);
    }
}

TEST(Zoning, the_winner_changes_when_the_geometry_changes_not_when_the_order_does) {
    // The mirror of the test above, with the larger area on the OTHER zone. A
    // solver that always returned the first source, the lowest enumerator or the
    // lowest id passes one of these two tests and fails this one.
    const ZoneSourceArea thirty = source(ZoneType::Residential, rect(0.0, 0.0, 30.0, 100.0), 1);
    const ZoneSourceArea seventy = source(ZoneType::Commercial, rect(30.0, 0.0, 100.0, 100.0), 2);

    const ZoneInference inference = infer_zone(unit_block(), {thirty, seventy});
    CHECK_EQ(zone_type_name(inference.zone), std::string("commercial"));
    CHECK_NEAR(inference.coverage, 0.7, 1e-6);
    CHECK_EQ(inference.source_id, int64_t{2});
}

TEST(Zoning, two_small_areas_of_one_zone_beat_one_larger_area_of_another) {
    // Pooling by zone, not ranking by polygon. Each residential polygon is
    // SMALLER than the commercial one, so per-polygon ranking says commercial and
    // this test fails. Real OSM splits a district across many polygons, so
    // per-polygon ranking loses to a mapping accident.
    const ZoneSourceArea west = source(ZoneType::Residential, rect(0.0, 0.0, 30.0, 100.0), 1);
    const ZoneSourceArea middle = source(ZoneType::Commercial, rect(30.0, 0.0, 70.0, 100.0), 2);
    const ZoneSourceArea east = source(ZoneType::Residential, rect(70.0, 0.0, 100.0, 100.0), 3);

    // Both orders, and that is the whole point of the second one. Provenance is
    // the LARGEST single contributor to the winner; the two residential polygons
    // are exactly the same size, so the tie goes to the lower id. An earlier
    // version of this test listed id 1 first and asserted source_id == 1, which a
    // within-pool rule of "whichever arrived first" also satisfies -- the
    // assertion could not check the property its own comment claimed. Deleting
    // the tie-break leg from the solver passed the whole suite. Reversed, a
    // first-wins implementation answers 3.
    const std::vector<std::vector<ZoneSourceArea>> both_orders = {{west, middle, east},
                                                                 {east, middle, west}};
    for (const std::vector<ZoneSourceArea>& sources : both_orders) {
        const ZoneInference inference = infer_zone(unit_block(), sources);
        CHECK_EQ(zone_type_name(inference.zone), std::string("residential"));
        CHECK_NEAR(inference.coverage, 0.6, 1e-6);
        CHECK_EQ(zone_type_name(inference.runner_up), std::string("commercial"));
        CHECK_NEAR(inference.runner_up_coverage, 0.4, 1e-6);
        CHECK_EQ(inference.source_id, int64_t{1});
    }
}

TEST(Zoning, sources_that_never_set_a_source_id_break_their_tie_on_the_enumerator) {
    // The last leg of the pool ordering, and it is not dead code.
    // ZoneSourceArea::source_id defaults to 0, so a producer that forgets to set
    // it puts EVERY tie on this leg -- one missing line in an importer away from
    // deciding a whole extract. Nothing reached it before.
    ZoneSourceArea park;
    park.zone = ZoneType::Park;
    park.ring = rect(0.0, 0.0, 50.0, 100.0);
    // source_id deliberately left at its default.

    ZoneSourceArea industrial;
    industrial.zone = ZoneType::Industrial;
    industrial.ring = rect(50.0, 0.0, 100.0, 100.0);

    // Stated here so the expectation below is a rule and not a memorised answer.
    CHECK((static_cast<int>(ZoneType::Industrial)) < (static_cast<int>(ZoneType::Park)));

    // Both orders, because a leg that consults the caller's vector instead of the
    // enumerator agrees with one of them and not the other.
    const std::vector<std::vector<ZoneSourceArea>> both_orders = {{park, industrial},
                                                                 {industrial, park}};
    for (const std::vector<ZoneSourceArea>& sources : both_orders) {
        const ZoneInference inference = infer_zone(unit_block(), sources);
        CHECK_EQ(outcome_of(inference), std::string("Inferred"));
        CHECK_EQ(zone_type_name(inference.zone), std::string("industrial"));
        CHECK_EQ(zone_type_name(inference.runner_up), std::string("park"));
        CHECK_EQ(inference.source_id, int64_t{0});
        CHECK_NEAR(inference.coverage, 0.5, 1e-6);
    }
}

TEST(Zoning, a_sub_millimetre_difference_in_area_decides_the_winner_in_either_order) {
    // The area comparison has no tolerance, and this is the fixture that says so.
    // The right-hand source is the right half of the block with a right triangle
    // of 1 mm legs chamfered off its inner corner: half a square millimetre
    // smaller, the smallest difference Clipper2's integer grid can express. It is
    // also the only fixture in this file whose coordinates are not whole
    // millimetres, so it is the only one that could see a tolerance at all.
    //
    // A tolerance looks kinder here and is not. "Equal to within epsilon" is not
    // transitive, so with one in place a chain of sources a few tenths of a
    // millimetre apart resolves differently depending on which end of it the
    // caller's vector starts from -- the input-order dependence this whole file
    // is built to avoid. An exact comparison is a total order, and because
    // Clipper2's areas come from integer arithmetic an exact tie is still
    // recognised as a tie; an_exact_tie_breaks_on_the_smaller_source_id_in_either_order
    // covers that side.
    //
    // The epsilon that used to be here was computed in square metres and compared
    // against areas in scaled units, a factor of a million out, so it never fired
    // and nothing in the suite noticed when it was set to zero.
    const ZoneSourceArea left = source(ZoneType::Residential, rect(0.0, 0.0, 50.0, 100.0), 20);
    const ZoneSourceArea right =
        source(ZoneType::Commercial,
               {glm::dvec2(50.001, 0.0), glm::dvec2(100.0, 0.0), glm::dvec2(100.0, 100.0),
                glm::dvec2(50.0, 100.0), glm::dvec2(50.0, 0.001)},
               10);

    // Commercial has the LOWER id, so a tolerance wide enough to call these equal
    // hands it the block. The half square millimetre is what must decide instead.
    const std::vector<std::vector<ZoneSourceArea>> both_orders = {{left, right}, {right, left}};
    for (const std::vector<ZoneSourceArea>& sources : both_orders) {
        const ZoneInference inference = infer_zone(unit_block(), sources);
        CHECK_EQ(outcome_of(inference), std::string("Inferred"));
        CHECK_EQ(zone_type_name(inference.zone), std::string("residential"));
        CHECK_EQ(inference.source_id, int64_t{20});
    }

    // And inside one pool, where the same rule names the provenance rather than
    // the winner. This is the comparison that walks the caller's vector, so both
    // orders matter more here than anywhere else in the file.
    const ZoneSourceArea right_same_zone = source(ZoneType::Residential, right.ring, 10);
    const std::vector<std::vector<ZoneSourceArea>> pooled_orders = {{left, right_same_zone},
                                                                    {right_same_zone, left}};
    for (const std::vector<ZoneSourceArea>& sources : pooled_orders) {
        const ZoneInference pooled = infer_zone(unit_block(), sources);
        CHECK_EQ(zone_type_name(pooled.zone), std::string("residential"));
        // The larger of the two, not the lower id and not the first one seen.
        CHECK_EQ(pooled.source_id, int64_t{20});
    }
}

TEST(Zoning, overlapping_areas_of_one_zone_are_unioned_and_not_summed) {
    // Two IDENTICAL residential polygons, each covering 40%. Summing gives
    // residential 80% and it wins; unioning gives residential 40% and commercial
    // takes it. Duplicate landuse polygons are common in real extracts.
    const std::vector<ZoneSourceArea> sources = {
        source(ZoneType::Residential, rect(0.0, 0.0, 40.0, 100.0), 1),
        source(ZoneType::Residential, rect(0.0, 0.0, 40.0, 100.0), 2),
        source(ZoneType::Commercial, rect(40.0, 0.0, 90.0, 100.0), 3),
    };

    const ZoneInference inference = infer_zone(unit_block(), sources);
    CHECK_EQ(zone_type_name(inference.zone), std::string("commercial"));
    CHECK_NEAR(inference.coverage, 0.5, 1e-6);
    CHECK_EQ(zone_type_name(inference.runner_up), std::string("residential"));
    // 0.4, not 0.8. The number is the assertion; the winner alone could be right
    // for the wrong reason.
    CHECK_NEAR(inference.runner_up_coverage, 0.4, 1e-6);
}

TEST(Zoning, a_block_inside_the_hole_of_a_landuse_area_is_not_zoned_by_it) {
    // The block is entirely within the outer ring and entirely within the hole.
    // A solver that tested the outer ring only, or that fed the hole to a
    // non-zero fill, reports full coverage here.
    ZoneSourceArea park = source(ZoneType::Park, rect(-100.0, -100.0, 200.0, 200.0), 1);
    park.holes.push_back(rect(-10.0, -10.0, 110.0, 110.0));

    const ZoneInference inference = infer_zone(unit_block(), {park});
    CHECK_EQ(outcome_of(inference), std::string("NoOverlap"));
    CHECK_NEAR(inference.coverage, 0.0, 1e-12);
    // Still measured, so a caller can tell a degenerate block from an unzoned one.
    CHECK_NEAR(inference.target_area, 10000.0, 1e-6);
}

TEST(Zoning, a_hole_that_covers_only_part_of_the_block_removes_only_that_part) {
    // Guards the opposite mistake from the test above: a hole must not cause the
    // whole area to be discarded. The hole takes the left 70% of the block, so
    // 30% of park survives and clears the default floor.
    ZoneSourceArea park = source(ZoneType::Park, rect(-100.0, -100.0, 200.0, 200.0), 1);
    park.holes.push_back(rect(-100.0, -100.0, 70.0, 200.0));

    const ZoneInference inference = infer_zone(unit_block(), {park});
    CHECK_EQ(outcome_of(inference), std::string("Inferred"));
    CHECK_EQ(zone_type_name(inference.zone), std::string("park"));
    CHECK_NEAR(inference.coverage, 0.3, 1e-6);
}

TEST(Zoning, an_exact_tie_breaks_on_the_smaller_source_id_in_either_order) {
    // Two zones covering exactly half the block each. The tie-break is arbitrary;
    // what it must NOT be is the position in the caller's vector, which is the
    // importer's iteration order.
    const ZoneSourceArea high_id = source(ZoneType::Residential, rect(0.0, 0.0, 50.0, 100.0), 20);
    const ZoneSourceArea low_id = source(ZoneType::Commercial, rect(50.0, 0.0, 100.0, 100.0), 10);

    const ZoneInference forwards = infer_zone(unit_block(), {high_id, low_id});
    const ZoneInference backwards = infer_zone(unit_block(), {low_id, high_id});

    for (const ZoneInference& inference : {forwards, backwards}) {
        CHECK_EQ(outcome_of(inference), std::string("Inferred"));
        CHECK_EQ(zone_type_name(inference.zone), std::string("commercial"));
        CHECK_EQ(inference.source_id, int64_t{10});
        CHECK_NEAR(inference.coverage, 0.5, 1e-6);
        // A dead heat is the definition of contested.
        CHECK_TRUE(inference.contested);
    }
}

TEST(Zoning, a_narrow_win_is_flagged_contested_and_a_clear_one_is_not) {
    const ZoneInference narrow = infer_zone(
        unit_block(), {source(ZoneType::Residential, rect(0.0, 0.0, 52.0, 100.0), 1),
                       source(ZoneType::Commercial, rect(52.0, 0.0, 100.0, 100.0), 2)});
    CHECK_EQ(outcome_of(narrow), std::string("Inferred"));
    CHECK_EQ(zone_type_name(narrow.zone), std::string("residential"));
    // Flagged, and still zoned. Refusing here would leave the block generating
    // nothing, which is worse than being slightly wrong.
    CHECK_TRUE(narrow.contested);

    const ZoneInference clear = infer_zone(
        unit_block(), {source(ZoneType::Residential, rect(0.0, 0.0, 90.0, 100.0), 1),
                       source(ZoneType::Commercial, rect(90.0, 0.0, 100.0, 100.0), 2)});
    CHECK_EQ(zone_type_name(clear.zone), std::string("residential"));
    CHECK_FALSE(clear.contested);
}

TEST(Zoning, a_corner_clip_is_below_the_floor_and_still_names_its_candidate) {
    // 10 m square in the corner of a 100 m block: 1% coverage.
    const ZoneInference inference =
        infer_zone(unit_block(), {source(ZoneType::Parking, rect(90.0, 90.0, 100.0, 100.0), 4)});

    CHECK_EQ(outcome_of(inference), std::string("BelowFloor"));
    CHECK_FALSE(inference.inferred());
    CHECK_NEAR(inference.coverage, 0.01, 1e-6);
    // The candidate is carried for diagnosis. It is the OUTCOME that stops it
    // being written, which the applier test below pins down.
    CHECK_EQ(zone_type_name(inference.zone), std::string("parking"));

    // The floor is a tunable, not a law: a caller that wants every scrap of
    // evidence used sets it to zero and gets the same candidate as a decision.
    ZoningConfig no_floor;
    no_floor.min_coverage = 0.0;
    const ZoneInference forced =
        infer_zone(unit_block(), {source(ZoneType::Parking, rect(90.0, 90.0, 100.0, 100.0), 4)},
                   no_floor);
    CHECK_EQ(outcome_of(forced), std::string("Inferred"));
    CHECK_EQ(zone_type_name(forced.zone), std::string("parking"));
}

TEST(Zoning, coverage_exactly_at_the_floor_is_inferred_and_a_hair_under_it_is_not) {
    // "Must cover a quarter" and "must EXCEED a quarter" are different rules, and
    // every other fixture in this file sits far enough from the boundary that
    // both give the same answer. 25 m of a 100 m block is 2500 of 10000 m^2;
    // both are exact in binary, so the ratio is exactly the double 0.25 -- the
    // same one ZoningConfig::min_coverage holds.
    const ZoneInference at_the_floor =
        infer_zone(unit_block(), {source(ZoneType::Parking, rect(0.0, 0.0, 25.0, 100.0), 4)});
    // A zero tolerance, because "near 0.25" is what makes this test blind.
    CHECK_NEAR(at_the_floor.coverage, 0.25, 0.0);
    CHECK_EQ(outcome_of(at_the_floor), std::string("Inferred"));

    // One decimetre narrower, and a quarter is exactly what it fails to reach.
    const ZoneInference under =
        infer_zone(unit_block(), {source(ZoneType::Parking, rect(0.0, 0.0, 24.9, 100.0), 4)});
    CHECK_NEAR(under.coverage, 0.249, 1e-12);
    CHECK_EQ(outcome_of(under), std::string("BelowFloor"));
    // Still named, so a user can see what was nearly chosen.
    CHECK_EQ(zone_type_name(under.zone), std::string("parking"));
}

TEST(Zoning, a_gap_exactly_equal_to_the_contested_margin_is_contested) {
    // The default 0.10 margin cannot be hit exactly in doubles: a nominal 0.10
    // gap between two coverages evaluates to 0.10000000000000003, so `<` and
    // `<=` cannot be told apart with it and the documented boundary is
    // unreachable. 0.625 and 0.375 are both exact binary fractions and their
    // difference is exactly 0.25, so a margin of 0.25 puts the gap ON the
    // boundary and the rule becomes checkable.
    ZoningConfig config;
    config.contested_margin = 0.25;

    const ZoneInference on_the_margin =
        infer_zone(unit_block(), {source(ZoneType::Residential, rect(0.0, 0.0, 62.5, 100.0), 1),
                                  source(ZoneType::Commercial, rect(62.5, 0.0, 100.0, 100.0), 2)},
                   config);
    CHECK_NEAR(on_the_margin.coverage, 0.625, 0.0);
    CHECK_NEAR(on_the_margin.runner_up_coverage, 0.375, 0.0);
    CHECK_NEAR(on_the_margin.coverage - on_the_margin.runner_up_coverage, 0.25, 0.0);
    // "Within the margin" includes being equal to it.
    CHECK_TRUE(on_the_margin.contested);

    // A decimetre of the block either way, and the gap is 0.252 -- outside.
    const ZoneInference outside =
        infer_zone(unit_block(), {source(ZoneType::Residential, rect(0.0, 0.0, 62.6, 100.0), 1),
                                  source(ZoneType::Commercial, rect(62.6, 0.0, 100.0, 100.0), 2)},
                   config);
    CHECK_FALSE(outside.contested);
}

TEST(Zoning, ring_coordinates_are_rounded_to_the_nearest_millimetre_not_truncated) {
    // 99.9996 m is 99999.6 millimetres. Rounded that is 100000 and the block
    // measures 10000 m^2; truncated it is 99999 and the block measures
    // 9999.800001 m^2 -- smaller, which is the systematic shrink towards
    // BelowFloor the quantisation comment claims to avoid. Every other coordinate
    // in this file is a whole number of millimetres, where the two rules agree,
    // so nothing else here can see the difference.
    const std::vector<ZoneSourceArea> everywhere = {
        source(ZoneType::Residential, rect(-500.0, -500.0, 500.0, 500.0), 1)};

    const ZoneInference inference = infer_zone(rect(0.0, 0.0, 99.9996, 99.9996), everywhere);
    CHECK_EQ(outcome_of(inference), std::string("Inferred"));
    CHECK_NEAR(inference.target_area, 10000.0, 1e-6);
}

TEST(Zoning, a_ring_too_far_from_the_origin_to_represent_is_degenerate) {
    // Two thousand kilometres out. Scaled to millimetres that is 2e9, past the
    // 1e9 cap Clipper2's internal squaring needs, and a coordinate that wrapped
    // would put this block somewhere else in the world where an unrelated landuse
    // area is. Refused instead.
    //
    // Without the cap the ring is simply a block nothing overlaps, which is
    // NoOverlap -- a different answer, which is what makes the guard checkable.
    CHECK_EQ(outcome_of(infer_zone(rect(2.0e6, 2.0e6, 2.0e6 + 100.0, 2.0e6 + 100.0), {})),
             std::string("Degenerate"));

    // The same guard catches a coordinate that is not a number at all: an
    // unprojected point, or a divide by zero somewhere upstream.
    const std::vector<glm::dvec2> not_a_number = {
        glm::dvec2(0.0, 0.0), glm::dvec2(100.0, 0.0),
        glm::dvec2(std::numeric_limits<double>::quiet_NaN(), 100.0)};
    CHECK_EQ(outcome_of(infer_zone(not_a_number, {})), std::string("Degenerate"));

    // And the cap is a cap, not a ban on being far from the origin: the same
    // block at 500 km is well inside it and zones normally.
    const ZoneInference far_but_legal =
        infer_zone(rect(5.0e5, 5.0e5, 5.0e5 + 100.0, 5.0e5 + 100.0),
                   {source(ZoneType::Forest, rect(5.0e5 - 10.0, 5.0e5 - 10.0, 5.0e5 + 110.0,
                                                  5.0e5 + 110.0),
                           1)});
    CHECK_EQ(outcome_of(far_but_legal), std::string("Inferred"));
    CHECK_NEAR(far_but_legal.target_area, 10000.0, 1e-3);
}

TEST(Zoning, a_self_intersecting_ring_is_measured_the_same_way_its_overlaps_are) {
    // blocks.hpp says a face with Block::has_grade_separated_edge set may have a
    // self-intersecting ring, so a bowtie reaches this solver on real data.
    //
    // There are two ways to measure one. The signed shoelace area counts the two
    // lobes with opposite signs; an even-odd fill counts both. Every numerator
    // here comes from an even-odd clip, so the denominator has to come from one
    // too. Mixing them put coverage at 1.13 on the first ring below, hidden by a
    // clamp to 1, which also made coverage and runner_up_coverage
    // non-comparable and so made the contested flag meaningless.
    const std::vector<ZoneSourceArea> everywhere = {
        source(ZoneType::Residential, rect(-500.0, -500.0, 500.0, 500.0), 1)};

    // Lobes of 2000 and 1400 m^2. Shoelace says 600; even-odd says 3400.
    const std::vector<glm::dvec2> lopsided = {glm::dvec2(0.0, 0.0), glm::dvec2(100.0, 20.0),
                                              glm::dvec2(100.0, 0.0), glm::dvec2(0.0, 80.0)};
    const ZoneInference lopsided_result = infer_zone(lopsided, everywhere);
    CHECK_EQ(outcome_of(lopsided_result), std::string("Inferred"));
    CHECK_NEAR(lopsided_result.target_area, 3400.0, 1e-6);
    // A source covering the whole plane covers all of the target and no more,
    // and without a clamp to hide it that has to come out as exactly one.
    CHECK_NEAR(lopsided_result.coverage, 1.0, 1e-12);

    // The symmetric case is the one shoelace cannot see at all: two 2500 m^2
    // lobes cancel to zero, so the block was called Degenerate -- unzoned, and
    // generating nothing -- although it encloses 5000 m^2 of real land.
    const std::vector<glm::dvec2> hourglass = {glm::dvec2(0.0, 0.0), glm::dvec2(100.0, 100.0),
                                               glm::dvec2(100.0, 0.0), glm::dvec2(0.0, 100.0)};
    const ZoneInference hourglass_result = infer_zone(hourglass, everywhere);
    CHECK_EQ(outcome_of(hourglass_result), std::string("Inferred"));
    CHECK_NEAR(hourglass_result.target_area, 5000.0, 1e-6);

    // Half of it, so the ratio has to be built from the same rule as the area
    // rather than merely land on 1 by covering everything.
    const ZoneInference upper_lobe =
        infer_zone(hourglass, {source(ZoneType::Retail, rect(0.0, 50.0, 100.0, 100.0), 1)});
    CHECK_EQ(outcome_of(upper_lobe), std::string("Inferred"));
    CHECK_EQ(zone_type_name(upper_lobe.zone), std::string("retail"));
    CHECK_NEAR(upper_lobe.coverage, 0.5, 1e-9);

    // Degenerate still means what it says: a ring enclosing nothing at all.
    const std::vector<glm::dvec2> collapsed = {glm::dvec2(0.0, 0.0), glm::dvec2(50.0, 0.0),
                                               glm::dvec2(100.0, 0.0), glm::dvec2(50.0, 0.0)};
    CHECK_EQ(outcome_of(infer_zone(collapsed, everywhere)), std::string("Degenerate"));
}

TEST(Zoning, a_block_no_landuse_area_touches_has_no_overlap) {
    const ZoneInference inference =
        infer_zone(unit_block(), {source(ZoneType::Forest, rect(500.0, 500.0, 600.0, 600.0), 1)});

    CHECK_EQ(outcome_of(inference), std::string("NoOverlap"));
    CHECK_FALSE(inference.inferred());
    // The trap: `zone` reads "unknown" here, and "unknown" is a REAL zone. The
    // applier must key off inferred(), not off this field.
    CHECK_EQ(zone_type_name(inference.zone), std::string("unknown"));
}

TEST(Zoning, areas_that_only_touch_the_block_edge_do_not_zone_it) {
    // Shares the whole right-hand edge and encloses none of the block.
    const ZoneInference inference =
        infer_zone(unit_block(), {source(ZoneType::Industrial, rect(100.0, 0.0, 200.0, 100.0), 1)});
    CHECK_EQ(outcome_of(inference), std::string("NoOverlap"));
}

TEST(Zoning, a_ring_that_is_not_a_polygon_is_degenerate_rather_than_unzoned) {
    const std::vector<ZoneSourceArea> everywhere = {
        source(ZoneType::Residential, rect(-500.0, -500.0, 500.0, 500.0), 1)};

    const std::vector<glm::dvec2> two_points = {glm::dvec2(0.0, 0.0), glm::dvec2(10.0, 0.0)};
    CHECK_EQ(outcome_of(infer_zone(two_points, everywhere)), std::string("Degenerate"));

    // Three points that enclose nothing. Separately worth testing: this one
    // survives the point count and would divide by a zero area.
    const std::vector<glm::dvec2> collinear = {glm::dvec2(0.0, 0.0), glm::dvec2(10.0, 0.0),
                                               glm::dvec2(20.0, 0.0)};
    const ZoneInference flat = infer_zone(collinear, everywhere);
    CHECK_EQ(outcome_of(flat), std::string("Degenerate"));
    CHECK_NEAR(flat.target_area, 0.0, 1e-12);

    CHECK_EQ(outcome_of(infer_zone({}, everywhere)), std::string("Degenerate"));
}

TEST(Zoning, a_clockwise_ring_measures_the_same_as_an_anticlockwise_one) {
    // Lots (C2) are cut by code that has no obligation to wind the way blocks
    // do, and rejecting a clockwise ring would silently unzone every one of them.
    std::vector<glm::dvec2> reversed = unit_block();
    std::reverse(reversed.begin(), reversed.end());

    const std::vector<ZoneSourceArea> sources = {
        source(ZoneType::Retail, rect(0.0, 0.0, 80.0, 100.0), 1)};

    const ZoneInference forwards = infer_zone(unit_block(), sources);
    const ZoneInference backwards = infer_zone(reversed, sources);

    CHECK_EQ(zone_type_name(backwards.zone), zone_type_name(forwards.zone));
    CHECK_NEAR(backwards.coverage, forwards.coverage, 1e-9);
    CHECK_NEAR(backwards.coverage, 0.8, 1e-6);
    CHECK_NEAR(backwards.target_area, 10000.0, 1e-6);
}

// ============================================================================
// The batch solve
// ============================================================================

namespace {

/// Two blocks a kilometre apart, each with its own landuse area over it
struct SeparatedFixture {
    std::vector<ZoneTarget> targets;
    std::vector<ZoneSourceArea> sources;
};

SeparatedFixture make_separated(AttributeStore& store) {
    SeparatedFixture fixture;

    ZoneTarget near_origin;
    near_origin.object = store.create_object();
    near_origin.ring = unit_block();
    fixture.targets.push_back(std::move(near_origin));

    ZoneTarget far_away;
    far_away.object = store.create_object();
    far_away.ring = rect(1000.0, 1000.0, 1100.0, 1100.0);
    fixture.targets.push_back(std::move(far_away));

    fixture.sources.push_back(
        source(ZoneType::Residential, rect(-10.0, -10.0, 110.0, 110.0), 1));
    fixture.sources.push_back(
        source(ZoneType::Industrial, rect(990.0, 990.0, 1110.0, 1110.0), 2));

    return fixture;
}

} // namespace

TEST(Zoning, the_report_is_parallel_to_the_targets_and_its_counts_cover_them_all) {
    AttributeStore store;
    const SeparatedFixture fixture = make_separated(store);

    const ZoningReport report = infer_zones(fixture.targets, fixture.sources);

    CHECK_EQ(report.inferences.size(), fixture.targets.size());
    CHECK_EQ(report.stats.targets, size_t{2});
    CHECK_EQ(report.stats.sources, size_t{2});
    CHECK_EQ(report.stats.inferred, size_t{2});

    // Every target lands in exactly one bucket. A count that silently dropped a
    // target would look identical to a correct run without this.
    const size_t accounted = report.stats.inferred + report.stats.no_overlap
                             + report.stats.below_floor + report.stats.degenerate;
    CHECK_EQ(accounted, report.stats.targets);

    // Parallel means index i is target i, not "some target". The two blocks are a
    // kilometre apart and differently zoned, so a swap is visible.
    CHECK_EQ(zone_type_name(report.inferences[0].zone), std::string("residential"));
    CHECK_EQ(zone_type_name(report.inferences[1].zone), std::string("industrial"));
}

TEST(Zoning, the_bounding_box_prefilter_skips_pairs_that_cannot_overlap) {
    AttributeStore store;
    const SeparatedFixture fixture = make_separated(store);

    const ZoningReport report = infer_zones(fixture.targets, fixture.sources);

    // Two targets and two sources is four pairs, but only the two matching ones
    // can overlap. Asserting both bounds: clips must not be zero (the prefilter
    // rejecting everything would also satisfy "fewer than pairs") and must be
    // strictly fewer than pairs (no prefilter at all satisfies "not zero").
    CHECK_EQ(report.stats.pairs, size_t{4});
    CHECK_EQ(report.stats.clips, size_t{2});
    CHECK((report.stats.clips) < (report.stats.pairs));
}

TEST(Zoning, a_report_that_does_not_match_its_targets_zones_nothing) {
    AttributeStore store;
    CommandStack stack;
    const SeparatedFixture fixture = make_separated(store);

    ZoningReport truncated = infer_zones(fixture.targets, fixture.sources);
    truncated.inferences.pop_back();

    CHECK_EQ(apply_inferred_zones(stack, store, fixture.targets, truncated), size_t{0});
    CHECK_EQ(load_inferred_zones(store, fixture.targets, truncated), size_t{0});

    // Refusing must mean refusing everything, not zoning the prefix. Zoning the
    // prefix is the shape of bug that looks right in a viewport.
    CHECK_EQ(status_of(resolve_zone(store, fixture.targets[0].object, kNoLayer)),
             std::string("Unzoned"));
    CHECK_EQ(stack.undo_depth(), size_t{0});
}

// ============================================================================
// Applying an inference
// ============================================================================

namespace {

/// One block a landuse area covers, and one a kilometre away that nothing touches
struct MixedFixture {
    std::vector<ZoneTarget> targets;
    std::vector<ZoneSourceArea> sources;
};

MixedFixture make_mixed(AttributeStore& store) {
    MixedFixture fixture;

    ZoneTarget covered;
    covered.object = store.create_object();
    covered.ring = unit_block();
    fixture.targets.push_back(std::move(covered));

    ZoneTarget untouched;
    untouched.object = store.create_object();
    untouched.ring = rect(1000.0, 1000.0, 1100.0, 1100.0);
    fixture.targets.push_back(std::move(untouched));

    fixture.sources.push_back(source(ZoneType::Residential, rect(-10.0, -10.0, 110.0, 110.0), 1));
    return fixture;
}

} // namespace

// The two appliers below are separate loops over the same report, and the next
// two tests are deliberately near-duplicates of each other because of it. An
// earlier draft of this suite tested only the undoable path; a mutation that made
// the IMPORT path stamp every target with `ZoneType::Unknown` passed the whole
// suite. One test per applier is the price of catching that.

TEST(Zoning, the_undoable_applier_writes_only_the_targets_that_got_a_zone) {
    AttributeStore store;
    CommandStack stack;
    const MixedFixture fixture = make_mixed(store);

    const ZoningReport report = infer_zones(fixture.targets, fixture.sources);
    // Stated up front, so the test cannot pass by both targets being unzonable.
    CHECK_EQ(report.stats.inferred, size_t{1});
    CHECK_EQ(report.stats.no_overlap, size_t{1});

    CHECK_EQ(apply_inferred_zones(stack, store, fixture.targets, report), size_t{1});

    CHECK_EQ(zone_of(resolve_zone(store, fixture.targets[0].object, kNoLayer)),
             std::string("residential"));

    // The NoOverlap target must stay genuinely unzoned. Writing the inference's
    // `zone` field unconditionally would stamp it "unknown", which reads back as
    // a deliberate classification and is the exact confusion this design is built
    // to prevent.
    CHECK_EQ(status_of(resolve_zone(store, fixture.targets[1].object, kNoLayer)),
             std::string("Unzoned"));
}

TEST(Zoning, the_import_applier_writes_only_the_targets_that_got_a_zone) {
    AttributeStore store;
    const MixedFixture fixture = make_mixed(store);

    const ZoningReport report = infer_zones(fixture.targets, fixture.sources);
    CHECK_EQ(report.stats.inferred, size_t{1});
    CHECK_EQ(report.stats.no_overlap, size_t{1});

    CHECK_EQ(load_inferred_zones(store, fixture.targets, report), size_t{1});

    CHECK_EQ(zone_of(resolve_zone(store, fixture.targets[0].object, kNoLayer)),
             std::string("residential"));
    CHECK_EQ(status_of(resolve_zone(store, fixture.targets[1].object, kNoLayer)),
             std::string("Unzoned"));
}

TEST(Zoning, applying_an_inference_is_one_undo_step_however_many_blocks_it_zones) {
    AttributeStore store;
    CommandStack stack;
    const SeparatedFixture fixture = make_separated(store);

    const ZoningReport report = infer_zones(fixture.targets, fixture.sources);
    CHECK_EQ(apply_inferred_zones(stack, store, fixture.targets, report), size_t{2});

    CHECK_EQ(stack.undo_depth(), size_t{1});
    CHECK_EQ(stack.undo_label(), std::string("Apply zoning"));

    CHECK_TRUE(stack.undo());
    // One undo takes back BOTH blocks. A per-block step would leave the second
    // one zoned here.
    CHECK_EQ(status_of(resolve_zone(store, fixture.targets[0].object, kNoLayer)),
             std::string("Unzoned"));
    CHECK_EQ(status_of(resolve_zone(store, fixture.targets[1].object, kNoLayer)),
             std::string("Unzoned"));
    CHECK_EQ(stack.undo_depth(), size_t{0});
}

TEST(Zoning, re_applying_an_unchanged_inference_records_no_undo_step) {
    // The workflow this function is written for: a user edits one street and
    // re-runs zoning from a panel. Nearly every block decides the zone it already
    // had. scene::set_attribute() never compares against the current value, so
    // writing those unchanged zones would still fill the transaction and still
    // record a step, and the user would get an "Apply zoning" entry whose undo
    // visibly does nothing -- the no-op step CommandStack exists to avoid.
    AttributeStore store;
    CommandStack stack;
    const SeparatedFixture fixture = make_separated(store);

    const ZoningReport report = infer_zones(fixture.targets, fixture.sources);
    CHECK_EQ(apply_inferred_zones(stack, store, fixture.targets, report), size_t{2});
    CHECK_EQ(stack.undo_depth(), size_t{1});
    const uint64_t revision_after_first = stack.revision();

    // The same decisions again, which is what a re-solve after an unrelated edit
    // produces for most of the extract.
    CHECK_EQ(apply_inferred_zones(stack, store, fixture.targets, report), size_t{0});
    CHECK_EQ(stack.undo_depth(), size_t{1});
    // revision() is what A2 compares to decide the document is dirty, so a re-solve
    // that changed nothing must not make the document look edited either.
    CHECK_EQ(stack.revision(), revision_after_first);

    // The zones are still there -- skipping is not clearing.
    CHECK_EQ(zone_of(resolve_zone(store, fixture.targets[0].object, kNoLayer)),
             std::string("residential"));
    CHECK_EQ(zone_of(resolve_zone(store, fixture.targets[1].object, kNoLayer)),
             std::string("industrial"));

    // And the one step on the stack is still the FIRST apply, not an empty one
    // sitting in front of it.
    CHECK_TRUE(stack.undo());
    CHECK_EQ(status_of(resolve_zone(store, fixture.targets[0].object, kNoLayer)),
             std::string("Unzoned"));
    CHECK_EQ(status_of(resolve_zone(store, fixture.targets[1].object, kNoLayer)),
             std::string("Unzoned"));
    CHECK_EQ(stack.undo_depth(), size_t{0});
}

TEST(Zoning, a_re_solve_writes_the_blocks_that_changed_and_only_those) {
    // The other half of the test above: skipping the unchanged blocks must not
    // skip the changed one, and one undo must take the change back without
    // disturbing the block that was left alone.
    AttributeStore store;
    CommandStack stack;
    const SeparatedFixture fixture = make_separated(store);

    CHECK_EQ(apply_inferred_zones(stack, store, fixture.targets,
                                  infer_zones(fixture.targets, fixture.sources)),
             size_t{2});

    // Re-map the near block as commercial and leave the far one alone.
    std::vector<ZoneSourceArea> edited = fixture.sources;
    edited[0] = source(ZoneType::Commercial, rect(-10.0, -10.0, 110.0, 110.0), 3);

    CHECK_EQ(apply_inferred_zones(stack, store, fixture.targets,
                                  infer_zones(fixture.targets, edited)),
             size_t{1});
    CHECK_EQ(stack.undo_depth(), size_t{2});
    CHECK_EQ(zone_of(resolve_zone(store, fixture.targets[0].object, kNoLayer)),
             std::string("commercial"));
    CHECK_EQ(zone_of(resolve_zone(store, fixture.targets[1].object, kNoLayer)),
             std::string("industrial"));

    CHECK_TRUE(stack.undo());
    CHECK_EQ(zone_of(resolve_zone(store, fixture.targets[0].object, kNoLayer)),
             std::string("residential"));
    // Untouched by the second apply, so untouched by undoing it.
    CHECK_EQ(zone_of(resolve_zone(store, fixture.targets[1].object, kNoLayer)),
             std::string("industrial"));
}

TEST(Zoning, a_re_solve_overwrites_a_value_it_cannot_read_rather_than_calling_it_unchanged) {
    // The trap in skipping unchanged writes: the comparison has to be a ZONE
    // comparison. A rung holding "resedential" out of a hand-edited save file, or
    // a number out of a broken importer, is not an unchanged residential zone --
    // it is exactly the value a re-solve exists to repair, and skipping it would
    // leave the document permanently Unrecognised with no way to fix it from the
    // panel.
    AttributeStore store;
    CommandStack stack;

    ZoneTarget block;
    block.object = store.create_object();
    block.ring = unit_block();
    const std::vector<ZoneTarget> targets = {block};

    store.load_value(inferred_zone_target(store, block.object),
                     AttributeValue::from_string("resedential"));
    CHECK_EQ(status_of(resolve_zone(store, block.object, kNoLayer)),
             std::string("Unrecognised"));

    const ZoningReport report = infer_zones(
        targets, {source(ZoneType::Residential, rect(-10.0, -10.0, 110.0, 110.0), 1)});
    CHECK_EQ(apply_inferred_zones(stack, store, targets, report), size_t{1});

    const ZoneQuery repaired = resolve_zone(store, block.object, kNoLayer);
    CHECK_EQ(status_of(repaired), std::string("Zoned"));
    CHECK_EQ(zone_of(repaired), std::string("residential"));
    CHECK_EQ(stack.undo_depth(), size_t{1});
}

TEST(Zoning, the_import_path_writes_zones_without_touching_the_history) {
    AttributeStore store;
    CommandStack stack;
    const SeparatedFixture fixture = make_separated(store);

    const ZoningReport report = infer_zones(fixture.targets, fixture.sources);
    CHECK_EQ(load_inferred_zones(store, fixture.targets, report), size_t{2});

    CHECK_EQ(zone_of(resolve_zone(store, fixture.targets[1].object, kNoLayer)),
             std::string("industrial"));
    // The deliberate hole. Recording a city's worth of inferred zones as commands
    // would fill the undo stack with the act of opening a file.
    CHECK_EQ(stack.undo_depth(), size_t{0});
    CHECK_EQ(stack.revision(), uint64_t{0});
}

TEST(Zoning, re_running_inference_rewrites_the_inferred_zone_and_leaves_the_paint_alone) {
    // The requirement in one test. Painted must WIN, and must survive the
    // re-solve that a road edit triggers.
    AttributeStore store;
    CommandStack stack;

    ZoneTarget block;
    block.object = store.create_object();
    block.ring = unit_block();
    const std::vector<ZoneTarget> targets = {block};

    const std::vector<ZoneSourceArea> first = {
        source(ZoneType::Residential, rect(-10.0, -10.0, 110.0, 110.0), 1)};
    CHECK_EQ(load_inferred_zones(store, targets, infer_zones(targets, first)), size_t{1});
    CHECK_EQ(zone_of(peek_zone(store, inferred_zone_target(store, block.object))),
             std::string("residential"));

    CHECK_TRUE(paint_zone(stack, store, block.object, ZoneType::Commercial));

    // The re-solve produces a THIRD zone, so the Object rung visibly changes and
    // the test cannot pass by the re-solve doing nothing at all.
    const std::vector<ZoneSourceArea> second = {
        source(ZoneType::Industrial, rect(-10.0, -10.0, 110.0, 110.0), 2)};
    const ZoningReport report = infer_zones(targets, second);
    CHECK_EQ(apply_inferred_zones(stack, store, targets, report), size_t{1});

    // The inferred rung really was rewritten...
    CHECK_EQ(zone_of(peek_zone(store, inferred_zone_target(store, block.object))),
             std::string("industrial"));
    // ...and the paint is untouched and still wins.
    CHECK_EQ(zone_of(peek_zone(store, painted_zone_target(store, block.object))),
             std::string("commercial"));

    const ZoneQuery resolved = resolve_zone(store, block.object, kNoLayer);
    CHECK_EQ(zone_of(resolved), std::string("commercial"));
    CHECK_EQ(source_of(resolved), std::string("User"));

    // And clearing the paint now reveals the NEW inference, not the old one.
    CHECK_TRUE(clear_painted_zone(stack, store, block.object));
    CHECK_EQ(zone_of(resolve_zone(store, block.object, kNoLayer)),
             std::string("industrial"));
}

TEST(Zoning, a_target_with_no_attribute_record_is_skipped_rather_than_zoning_something_else) {
    AttributeStore store;
    CommandStack stack;

    ZoneTarget real_block;
    real_block.object = store.create_object();
    real_block.ring = unit_block();

    ZoneTarget handleless;  // default-constructed AttributeObject: names no slot
    handleless.ring = rect(10.0, 10.0, 90.0, 90.0);

    const std::vector<ZoneTarget> targets = {handleless, real_block};
    const std::vector<ZoneSourceArea> sources = {
        source(ZoneType::Farmland, rect(-10.0, -10.0, 110.0, 110.0), 1)};

    const ZoningReport report = infer_zones(targets, sources);
    // Both targets INFER a zone -- the geometry does not care about handles.
    CHECK_EQ(report.stats.inferred, size_t{2});
    // Only one can be written.
    CHECK_EQ(apply_inferred_zones(stack, store, targets, report), size_t{1});
    CHECK_EQ(zone_of(resolve_zone(store, real_block.object, kNoLayer)),
             std::string("farmland"));
}

TEST(Zoning, the_zoning_key_is_stable_and_is_the_name_the_rule_engine_reads) {
    AttributeStore store;
    const AttributeObject block = store.create_object();

    const auto key = zone_key(store);
    CHECK_TRUE(key.valid());
    // Idempotent: interning twice must not mint a second key, or a rule that
    // cached one would read a different attribute from the inspector.
    CHECK_EQ(zone_key(store), key);
    CHECK_EQ(store.key_name(key), std::string("zoning"));
    CHECK_EQ(store.key_count(), size_t{1});

    // Every writer targets that one key, so all four rungs share it.
    CHECK_EQ(painted_zone_target(store, block).key, key);
    CHECK_EQ(inferred_zone_target(store, block).key, key);
    CHECK_EQ(store.key_count(), size_t{1});
}
