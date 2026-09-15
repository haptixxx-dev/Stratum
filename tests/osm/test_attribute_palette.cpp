// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_attribute_palette.cpp
 * @brief The colour-by-attribute palette is total, distinguishable and readable
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The palette is a debug aid, so its failures are quiet ones: two classes that
 * happen to share a colour do not crash, they just make a user believe a wrong
 * thing about their data for an afternoon. Every assertion here is aimed at that
 * class of failure rather than at anything that would ever throw.
 *
 * ### How the loops stay honest when the enums grow
 *
 * Nothing below lists enumerators by hand. Every family is walked as the integer
 * range `[0, kFamilyCount)`, and the count comes from `Unknown + 1` in
 * attribute_palette.hpp, so a value added to BuildingType, RoadType or AreaType
 * is covered by these tests the moment it exists, with no edit here.
 *
 * That closes the loop on totality from the other side. The `static_assert` in
 * attribute_palette.cpp already refuses to compile a table that has fallen behind
 * its enum; these tests then prove the table that did compile has no duplicate
 * and no accidental Unknown in it. A new value whose colour was added by copying
 * the line above and forgetting to change the colour compiles fine and fails
 * no_two_*_colours_collide.
 *
 * ### The separation thresholds
 *
 * kMinSeparation is plain Euclidean distance in linear RGB. It is not a
 * perceptual metric and does not pretend to be one -- it is a floor that catches
 * "these two are nearly the same colour", which is the mistake that actually gets
 * made when someone adds a fourteenth building type. The real palette's tightest
 * pair sits at 0.257, so the 0.20 floor has room for a tweak without becoming a
 * test nobody can change.
 *
 * kTerrainGreen and kSkyBlue are representative, not exact. The terrain green is
 * the AreaType::Grass fill from osm/mesh_builder.cpp and the sky blue is a mid
 * value from the analytic sky. Being approximate is fine for their purpose: they
 * pin that nobody adds a palette entry that disappears into the background.
 */

#include "framework.hpp"

#include "osm/attribute_palette.hpp"
#include "osm/types.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

using stratum::osm::AreaType;
using stratum::osm::AttributeMode;
using stratum::osm::BuildingType;
using stratum::osm::RoadType;

using stratum::osm::area_type_colour;
using stratum::osm::attribute_mode_name;
using stratum::osm::building_type_colour;
using stratum::osm::road_type_colour;
using stratum::osm::tile_colour;
using stratum::osm::tile_colour_index;
using stratum::osm::unknown_attribute_colour;

using stratum::osm::kAreaTypeCount;
using stratum::osm::kAttributeModeCount;
using stratum::osm::kBuildingTypeCount;
using stratum::osm::kRoadTypeCount;
using stratum::osm::kTileColourCount;

namespace {

/// Floor on the Euclidean RGB distance between any two colours a user must tell
/// apart. See the file header for why this number and not a perceptual one.
constexpr float kMinSeparation = 0.20f;

/// Representative terrain fill, from build_area_mesh()'s AreaType::Grass colour.
const glm::vec3 kTerrainGreen{0.45f, 0.58f, 0.40f};

/// Representative sky, mid way between horizon and zenith.
const glm::vec3 kSkyBlue{0.25f, 0.45f, 0.85f};

/// Exact equality is the right test for a collision: both sides come from the
/// same table of literals, so two entries either are the same entry or are not.
bool same_colour(const glm::vec4& a, const glm::vec4& b) {
    return a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a;
}

float separation(const glm::vec4& a, const glm::vec4& b) {
    const float dr = a.r - b.r;
    const float dg = a.g - b.g;
    const float db = a.b - b.b;
    return std::sqrt(dr * dr + dg * dg + db * db);
}

float separation_from(const glm::vec4& a, const glm::vec3& b) {
    return separation(a, glm::vec4(b.x, b.y, b.z, 1.0f));
}

bool is_opaque_and_in_range(const glm::vec4& c) {
    const bool rgb_ok = c.r >= 0.0f && c.r <= 1.0f &&
                        c.g >= 0.0f && c.g <= 1.0f &&
                        c.b >= 0.0f && c.b <= 1.0f;
    return rgb_ok && c.a == 1.0f;
}

/// Every colour of one family, in enumerator order, Unknown last.
template <typename Enum, typename Fn>
std::vector<glm::vec4> family(size_t count, Fn lookup) {
    std::vector<glm::vec4> colours;
    colours.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        colours.push_back(lookup(static_cast<Enum>(i)));
    }
    return colours;
}

std::vector<glm::vec4> building_family() {
    return family<BuildingType>(kBuildingTypeCount, building_type_colour);
}

std::vector<glm::vec4> road_family() {
    return family<RoadType>(kRoadTypeCount, road_type_colour);
}

std::vector<glm::vec4> area_family() {
    return family<AreaType>(kAreaTypeCount, area_type_colour);
}

std::vector<glm::vec4> tile_family() {
    std::vector<glm::vec4> colours;
    colours.reserve(kTileColourCount);
    for (size_t i = 0; i < kTileColourCount; ++i) {
        colours.push_back(tile_colour(i));
    }
    return colours;
}

void check_all_opaque_and_in_range(const std::vector<glm::vec4>& colours) {
    for (const glm::vec4& c : colours) {
        CHECK_TRUE(is_opaque_and_in_range(c));
    }
}

void check_no_collisions(const std::vector<glm::vec4>& colours) {
    for (size_t i = 0; i < colours.size(); ++i) {
        for (size_t j = i + 1; j < colours.size(); ++j) {
            CHECK_FALSE(same_colour(colours[i], colours[j]));
        }
    }
}

void check_separated(const std::vector<glm::vec4>& colours) {
    for (size_t i = 0; i < colours.size(); ++i) {
        for (size_t j = i + 1; j < colours.size(); ++j) {
            CHECK((separation(colours[i], colours[j])) >= kMinSeparation);
        }
    }
}

void check_readable_against_background(const std::vector<glm::vec4>& colours) {
    for (const glm::vec4& c : colours) {
        CHECK((separation_from(c, kTerrainGreen)) >= kMinSeparation);
        CHECK((separation_from(c, kSkyBlue)) >= kMinSeparation);
    }
}

} // namespace

// ============================================================================
// Every enumerator maps to a usable colour
// ============================================================================

TEST(AttributePalette, every_building_type_maps_to_an_opaque_colour_in_range) {
    check_all_opaque_and_in_range(building_family());
}

TEST(AttributePalette, every_road_type_maps_to_an_opaque_colour_in_range) {
    check_all_opaque_and_in_range(road_family());
}

TEST(AttributePalette, every_area_type_maps_to_an_opaque_colour_in_range) {
    check_all_opaque_and_in_range(area_family());
}

TEST(AttributePalette, every_tile_cycle_position_is_an_opaque_colour_in_range) {
    check_all_opaque_and_in_range(tile_family());
}

// ============================================================================
// No two values of one family share a colour
// ============================================================================

TEST(AttributePalette, no_two_building_type_colours_collide) {
    check_no_collisions(building_family());
}

TEST(AttributePalette, no_two_road_type_colours_collide) {
    check_no_collisions(road_family());
}

TEST(AttributePalette, no_two_area_type_colours_collide) {
    check_no_collisions(area_family());
}

TEST(AttributePalette, no_two_tile_cycle_colours_collide) {
    check_no_collisions(tile_family());
}

// ============================================================================
// ... and are far enough apart to be told apart, and to be seen at all
// ============================================================================

TEST(AttributePalette, building_type_colours_are_visually_separated) {
    check_separated(building_family());
}

TEST(AttributePalette, road_type_colours_are_visually_separated) {
    check_separated(road_family());
}

TEST(AttributePalette, area_type_colours_are_visually_separated) {
    check_separated(area_family());
}

TEST(AttributePalette, tile_cycle_colours_are_visually_separated) {
    check_separated(tile_family());
}

TEST(AttributePalette, every_colour_reads_against_the_terrain_and_the_sky) {
    check_readable_against_background(building_family());
    check_readable_against_background(road_family());
    check_readable_against_background(area_family());
    check_readable_against_background(tile_family());
    CHECK((separation_from(unknown_attribute_colour(), kTerrainGreen)) >= kMinSeparation);
    CHECK((separation_from(unknown_attribute_colour(), kSkyBlue)) >= kMinSeparation);
}

// ============================================================================
// Unknown
// ============================================================================

TEST(AttributePalette, unknown_is_every_familys_last_enumerator) {
    // The tables are indexed by the enumerator and sized from Unknown + 1, so
    // this is what makes that sizing correct rather than merely consistent. If
    // someone appends a value AFTER Unknown, Unknown stops being last and this
    // fails -- which is the one arrangement the static_assert in
    // attribute_palette.cpp cannot see.
    CHECK_TRUE(same_colour(building_type_colour(BuildingType::Unknown), unknown_attribute_colour()));
    CHECK_TRUE(same_colour(road_type_colour(RoadType::Unknown), unknown_attribute_colour()));
    CHECK_TRUE(same_colour(area_type_colour(AreaType::Unknown), unknown_attribute_colour()));

    CHECK_EQ(static_cast<size_t>(BuildingType::Unknown) + 1, kBuildingTypeCount);
    CHECK_EQ(static_cast<size_t>(RoadType::Unknown) + 1, kRoadTypeCount);
    CHECK_EQ(static_cast<size_t>(AreaType::Unknown) + 1, kAreaTypeCount);
}

TEST(AttributePalette, unknown_differs_from_every_classified_value) {
    const glm::vec4 unknown = unknown_attribute_colour();

    // Stops one short of Unknown itself in each family, so this is exactly the
    // "no real classification is allowed to look unclassified" assertion.
    for (size_t i = 0; i + 1 < kBuildingTypeCount; ++i) {
        CHECK_FALSE(same_colour(building_type_colour(static_cast<BuildingType>(i)), unknown));
    }
    for (size_t i = 0; i + 1 < kRoadTypeCount; ++i) {
        CHECK_FALSE(same_colour(road_type_colour(static_cast<RoadType>(i)), unknown));
    }
    for (size_t i = 0; i + 1 < kAreaTypeCount; ++i) {
        CHECK_FALSE(same_colour(area_type_colour(static_cast<AreaType>(i)), unknown));
    }
}

TEST(AttributePalette, the_tile_cycle_never_borrows_the_unknown_colour) {
    // Both modes can be used in one session, minutes apart, on the same city. A
    // tile that happened to be magenta would read as a parser failure.
    const glm::vec4 unknown = unknown_attribute_colour();
    for (size_t i = 0; i < kTileColourCount; ++i) {
        CHECK_FALSE(same_colour(tile_colour(i), unknown));
    }
}

TEST(AttributePalette, a_value_past_the_end_of_a_table_resolves_to_unknown) {
    // Documents the failure mode the header describes: a value appended after
    // Unknown lands past the table and renders magenta rather than black, garbage
    // or a plausible-looking colour belonging to something else.
    const glm::vec4 unknown = unknown_attribute_colour();
    CHECK_TRUE(same_colour(building_type_colour(static_cast<BuildingType>(kBuildingTypeCount)),
                           unknown));
    CHECK_TRUE(same_colour(road_type_colour(static_cast<RoadType>(kRoadTypeCount)), unknown));
    CHECK_TRUE(same_colour(area_type_colour(static_cast<AreaType>(kAreaTypeCount)), unknown));

    // And the same for a value below the table, which is what a stale cast of -1
    // produces.
    CHECK_TRUE(same_colour(building_type_colour(static_cast<BuildingType>(-1)), unknown));
}

// ============================================================================
// Tile identity
// ============================================================================

TEST(AttributePalette, tile_index_is_always_inside_the_cycle) {
    for (int64_t x = -40; x <= 40; ++x) {
        for (int64_t z = -40; z <= 40; ++z) {
            CHECK((tile_colour_index(x, z)) < kTileColourCount);
        }
    }
}

TEST(AttributePalette, no_tile_shares_a_colour_with_any_of_its_eight_neighbours) {
    // The whole reason the index is built from the two axes separately. A cycle
    // taken off a running tile id fails this, and where it fails the seam the
    // mode exists to show is invisible.
    constexpr int64_t kOffsets[8][2] = {
        {-1, -1}, {0, -1}, {1, -1},
        {-1,  0},          {1,  0},
        {-1,  1}, {0,  1}, {1,  1},
    };

    for (int64_t x = -12; x <= 12; ++x) {
        for (int64_t z = -12; z <= 12; ++z) {
            const size_t here = tile_colour_index(x, z);
            for (const auto& offset : kOffsets) {
                const size_t there = tile_colour_index(x + offset[0], z + offset[1]);
                CHECK((here) != (there));
                CHECK_FALSE(same_colour(tile_colour(here), tile_colour(there)));
            }
        }
    }
}

TEST(AttributePalette, tile_colour_wraps_rather_than_reading_off_the_end) {
    // tile_colour() takes any index at all, so a caller with its own numbering
    // scheme -- a quadtree node id, a chunk hash -- never has to pre-modulo.
    for (size_t i = 0; i < kTileColourCount; ++i) {
        CHECK_TRUE(same_colour(tile_colour(i), tile_colour(i + kTileColourCount)));
        CHECK_TRUE(same_colour(tile_colour(i), tile_colour(i + 7 * kTileColourCount)));
    }
}

TEST(AttributePalette, tile_index_handles_negative_coordinates) {
    // The local origin sits inside the extract, so roughly a quarter of every
    // import has negative tile coordinates. C++ `%` would hand back a negative
    // remainder and index before the start of the table.
    CHECK_EQ(tile_colour_index(-3, -4), tile_colour_index(0, 0));
    CHECK_EQ(tile_colour_index(-1, -1), tile_colour_index(2, 3));
    CHECK((tile_colour_index(-7, -9)) < kTileColourCount);
}

// ============================================================================
// Mode naming
// ============================================================================

TEST(AttributePalette, every_mode_has_a_non_empty_label) {
    for (size_t i = 0; i < kAttributeModeCount; ++i) {
        const char* label = attribute_mode_name(static_cast<AttributeMode>(i));
        CHECK_TRUE(label != nullptr);
        if (label != nullptr) {
            CHECK((std::strlen(label)) > size_t{0});
        }
    }
}

TEST(AttributePalette, mode_labels_are_distinct) {
    // The labels are the combo box entries. Two identical entries would be two
    // modes a user cannot choose between.
    for (size_t i = 0; i < kAttributeModeCount; ++i) {
        for (size_t j = i + 1; j < kAttributeModeCount; ++j) {
            const char* a = attribute_mode_name(static_cast<AttributeMode>(i));
            const char* b = attribute_mode_name(static_cast<AttributeMode>(j));
            CHECK((std::strcmp(a, b)) != 0);
        }
    }
}
