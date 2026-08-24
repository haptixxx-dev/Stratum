/**
 * @file test_road_style.cpp
 * @brief The tag-to-MaterialKey mapping, and the stability of its variant numbers
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * road_style.cpp is the geometry half of a contract with the renderer. The
 * renderer resolves a MaterialKey to textures and PBR parameters and never reads
 * an OSM tag; this module reads the tags and never touches a texture. The two
 * halves meet at an integer.
 *
 * That integer is why this suite asserts LITERAL variant numbers rather than
 * comparing against the named constants. Comparing `surface_material(...).variant`
 * against `variants::kCobblestone` proves only that the function agrees with
 * itself: renumber the constant and the test still passes while every exported
 * mesh and every material the renderer pre-built silently shifts by one. The
 * numbers below are the frozen half of the contract, so they are written out.
 *
 * The four questions:
 *
 * 1. **Does the documented table hold.** Every `surface=*` value the header names,
 *    mapped to the slot AND variant it names.
 * 2. **Are the numbers stable.** Literal integers, per the paragraph above.
 * 3. **Is the enumeration complete.** all_material_keys() is what the renderer
 *    pre-builds its material library from, so a key a mapping function can return
 *    and the enumeration does not list is a missing texture at run time, on some
 *    extract, months later. The check drives every mapping function over a table
 *    of inputs and demands the result be in the enumeration.
 * 4. **Are the names unique.** material_key_name() is written into an OBJ
 *    `usemtl` line and a glTF material name, where a collision merges two
 *    materials into one.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadStyle
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/road_style.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::MaterialKey;
using stratum::material_id_name;
using stratum::osm::BuildingType;
using stratum::osm::RoadType;
using stratum::osm::RoofType;
using stratum::osm::TagMap;
using stratum::osm::road::StripKind;
using stratum::osm::road::all_material_keys;
using stratum::osm::road::amenity_style_id;
using stratum::osm::road::building_roof_material;
using stratum::osm::road::building_wall_material;
using stratum::osm::road::material_key_name;
using stratum::osm::road::strip_material;
using stratum::osm::road::surface_material;
using stratum::osm::road::variant_count;

namespace variants = stratum::osm::road::variants;
namespace amenity_styles = stratum::osm::road::amenity_styles;

// ============================================================================
// Reporting helpers
// ============================================================================

/// A key rendered for a failure message, as slot name plus raw variant number
std::string describe(MaterialKey key) {
    return std::string(material_id_name(key.material)) + "/" +
           std::to_string(key.variant);
}

/// Report a mismatch against the documented table, naming the input that produced it
void expect_key(MaterialKey actual, MaterialId slot, uint16_t variant,
                const std::string& context, const char* file, int line) {
    if (actual.material == slot && actual.variant == variant) return;
    stratum::test::report_failure(
        file, line, "mapping matches the documented key",
        context + ": got " + describe(actual) + ", expected " +
            describe(MaterialKey{slot, variant}));
}

#define EXPECT_KEY(actual, slot, variant, context) \
    expect_key((actual), (slot), (variant), (context), __FILE__, __LINE__)

/// Every slot before MaterialId::Count, in enum order
const MaterialId kSlots[] = {
    MaterialId::Default,    MaterialId::Asphalt, MaterialId::Concrete,
    MaterialId::Curb,       MaterialId::Sidewalk, MaterialId::Markings,
    MaterialId::Gravel,     MaterialId::Dirt,    MaterialId::Grass,
    MaterialId::BridgeDeck, MaterialId::Parapet, MaterialId::Wall,
    MaterialId::Roof,
};

/// Every strip kind before StripKind::Count, in enum order
const StripKind kKinds[] = {
    StripKind::Lane,   StripKind::Gutter, StripKind::CurbFace,  StripKind::CurbTop,
    StripKind::Sidewalk, StripKind::Shoulder, StripKind::Median, StripKind::Verge,
    StripKind::CycleLane, StripKind::ParkingLane,
};

/// Every road class, in enum order
const RoadType kRoadTypes[] = {
    RoadType::Motorway, RoadType::Trunk,    RoadType::Primary,  RoadType::Secondary,
    RoadType::Tertiary, RoadType::Residential, RoadType::Service, RoadType::Footway,
    RoadType::Cycleway, RoadType::Path,     RoadType::Unknown,
};

/// Every building class, in enum order
const BuildingType kBuildingTypes[] = {
    BuildingType::Residential, BuildingType::Commercial, BuildingType::Industrial,
    BuildingType::Retail,      BuildingType::Office,     BuildingType::Apartments,
    BuildingType::House,       BuildingType::Detached,   BuildingType::Garage,
    BuildingType::Shed,        BuildingType::Church,     BuildingType::School,
    BuildingType::Hospital,    BuildingType::Warehouse,  BuildingType::Unknown,
};

/// Every roof shape, in enum order
const RoofType kRoofTypes[] = {
    RoofType::Flat,  RoofType::Gabled, RoofType::Hipped,
    RoofType::Pyramidal, RoofType::Skillion, RoofType::Dome, RoofType::Unknown,
};

/**
 * @brief Surface values worth driving every mapping function with
 *
 * The documented table, plus values that must MISS: an unknown word, a
 * multi-value tag, and a substance belonging to a different slot.
 */
const char* const kSurfaceProbes[] = {
    "",
    "asphalt", "paved", "chipseal", "tar", "bitumen",
    "cobblestone", "unhewn_cobblestone", "sett", "cobblestone:flattened",
    "paving_stones", "bricks", "brick",
    "concrete", "concrete:plates", "concrete:lanes",
    "gravel", "compacted", "fine_gravel", "pebblestone", "unpaved",
    "ground", "dirt", "earth", "soil", "mud", "sand",
    "grass", "grass_paver",
    "metal", "wood", "stone", "masonry", "steel",
    "asphalt;gravel", "woodchips", "not_a_surface",
};

/// smoothness values: the worn grades, the smooth grade, the neutral ones, absent
const char* const kSmoothnessProbes[] = {
    "", "excellent", "good", "intermediate",
    "bad", "very_bad", "horrible", "very_horrible", "impassable",
    "unknown_grade",
};

} // namespace

// ============================================================================
// Variant counts
// ============================================================================

/**
 * Rule 3 of the numbering rule: values are dense from 0, so a count is the whole
 * truth about a slot. The renderer sizes a flat per-slot array with it, and an
 * array one short is an out-of-bounds read the moment an extract carries the tag.
 */
TEST(RoadStyle, variant_counts_are_the_frozen_ones) {
    CHECK_EQ(variant_count(MaterialId::Default), size_t{1});
    CHECK_EQ(variant_count(MaterialId::Asphalt), size_t{8});
    CHECK_EQ(variant_count(MaterialId::Concrete), size_t{6});
    CHECK_EQ(variant_count(MaterialId::Curb), size_t{5});
    CHECK_EQ(variant_count(MaterialId::Sidewalk), size_t{6});
    CHECK_EQ(variant_count(MaterialId::Markings), size_t{3});
    CHECK_EQ(variant_count(MaterialId::Gravel), size_t{5});
    CHECK_EQ(variant_count(MaterialId::Dirt), size_t{5});
    CHECK_EQ(variant_count(MaterialId::Grass), size_t{5});
    CHECK_EQ(variant_count(MaterialId::BridgeDeck), size_t{4});
    CHECK_EQ(variant_count(MaterialId::Parapet), size_t{4});
    CHECK_EQ(variant_count(MaterialId::Wall), size_t{8});
    CHECK_EQ(variant_count(MaterialId::Roof), size_t{8});
}

/// Count is a sentinel, never a slot, and an out-of-range cast must not index a table
TEST(RoadStyle, variant_count_of_a_non_slot_is_zero) {
    CHECK_EQ(variant_count(MaterialId::Count), size_t{0});
    CHECK_EQ(variant_count(static_cast<MaterialId>(200)), size_t{0});
}

/**
 * The named constants and the counts must agree, or the renderer's array is sized
 * for one set of values and written with another.
 */
TEST(RoadStyle, named_constants_are_inside_their_slot_counts) {
    CHECK_TRUE(variants::kAsphaltColoured < variant_count(MaterialId::Asphalt));
    CHECK_TRUE(variants::kConcreteIsland < variant_count(MaterialId::Concrete));
    CHECK_TRUE(variants::kCurbMountable < variant_count(MaterialId::Curb));
    CHECK_TRUE(variants::kSidewalkBrick < variant_count(MaterialId::Sidewalk));
    CHECK_TRUE(variants::kMarkingsWorn < variant_count(MaterialId::Markings));
    CHECK_TRUE(variants::kGravelUnpaved < variant_count(MaterialId::Gravel));
    CHECK_TRUE(variants::kDirtSand < variant_count(MaterialId::Dirt));
    CHECK_TRUE(variants::kGrassPlanted < variant_count(MaterialId::Grass));
    CHECK_TRUE(variants::kBridgeDeckStone < variant_count(MaterialId::BridgeDeck));
    CHECK_TRUE(variants::kParapetConcrete < variant_count(MaterialId::Parapet));
    CHECK_TRUE(variants::kWallWood < variant_count(MaterialId::Wall));
    CHECK_TRUE(variants::kRoofGlass < variant_count(MaterialId::Roof));
}

// ============================================================================
// surface=* -- the documented table, with literal variant numbers
// ============================================================================

/**
 * The frozen table. LITERAL integers on purpose; see the file header.
 *
 * Note the slot changes, not just the variant: `gravel` is not asphalt with a
 * different texture, and a renderer handed {Asphalt, n} for a gravel road would
 * give it a bound-surface response to rain.
 */
TEST(RoadStyle, surface_table_maps_to_the_documented_key) {
    struct Row {
        const char* surface;
        MaterialId  slot;
        uint16_t    variant;
    };

    const Row rows[] = {
        // Bound bituminous -> Asphalt slot
        {"asphalt",               MaterialId::Asphalt,  0},
        {"paved",                 MaterialId::Asphalt,  0},
        {"chipseal",              MaterialId::Asphalt,  6},
        {"tar",                   MaterialId::Asphalt,  6},
        {"bitumen",               MaterialId::Asphalt,  6},

        // Modular running surfaces: driveable, so still the Asphalt SLOT
        {"cobblestone",           MaterialId::Asphalt,  3},
        {"unhewn_cobblestone",    MaterialId::Asphalt,  3},
        {"sett",                  MaterialId::Asphalt,  4},
        {"cobblestone:flattened", MaterialId::Asphalt,  4},
        {"paving_stones",         MaterialId::Asphalt,  5},
        {"bricks",                MaterialId::Asphalt,  5},
        {"brick",                 MaterialId::Asphalt,  5},

        // Rigid -> Concrete slot
        {"concrete",              MaterialId::Concrete, 0},
        {"concrete:plates",       MaterialId::Concrete, 3},
        {"concrete:lanes",        MaterialId::Concrete, 4},

        // Loose and compacted mineral -> Gravel slot
        {"gravel",                MaterialId::Gravel,   0},
        {"compacted",             MaterialId::Gravel,   1},
        {"fine_gravel",           MaterialId::Gravel,   2},
        {"pebblestone",           MaterialId::Gravel,   3},
        {"unpaved",               MaterialId::Gravel,   4},

        // Unbound earth -> Dirt slot
        {"ground",                MaterialId::Dirt,     1},
        {"dirt",                  MaterialId::Dirt,     2},
        {"earth",                 MaterialId::Dirt,     2},
        {"soil",                  MaterialId::Dirt,     2},
        {"mud",                   MaterialId::Dirt,     3},
        {"sand",                  MaterialId::Dirt,     4},

        // Planted -> Grass slot
        {"grass",                 MaterialId::Grass,    0},
        {"grass_paver",           MaterialId::Grass,    3},
    };

    for (const Row& row : rows) {
        // The road class must not be able to override a surface the table knows,
        // so every row is driven from a class whose default is a DIFFERENT slot.
        EXPECT_KEY(surface_material(RoadType::Residential, row.surface, ""),
                   row.slot, row.variant,
                   std::string("surface=") + row.surface + " on a residential road");
        EXPECT_KEY(surface_material(RoadType::Path, row.surface, ""),
                   row.slot, row.variant,
                   std::string("surface=") + row.surface + " on a path");
    }
}

/**
 * An unrecognised surface falls through to the road class, and never fails.
 *
 * A multi-value tag is deliberately in the list: `surface=asphalt;gravel` is not
 * split, so it does not match, so the class default applies. That is the
 * documented behaviour and not an accident to be silently 'fixed' into matching
 * its first element.
 */
TEST(RoadStyle, an_unknown_surface_falls_back_to_the_class_default) {
    const char* const unknown[] = {"", "asphalt;gravel", "woodchips", "not_a_surface",
                                   "ASPHALT", "  asphalt"};

    for (const char* surface : unknown) {
        for (const RoadType type : kRoadTypes) {
            const MaterialKey key = surface_material(type, surface, "");
            const std::string context =
                std::string("surface='") + surface + "' on class " +
                std::to_string(static_cast<int>(type));
            if (type == RoadType::Path) {
                // A path or track with no surface tag is not sealed.
                EXPECT_KEY(key, MaterialId::Dirt, 1, context);
            } else {
                EXPECT_KEY(key, MaterialId::Asphalt, 0, context);
            }
        }
    }
}

/**
 * smoothness refines the variant LAST, and only a variant that is still the slot
 * default, and only on a slot that defines the grades.
 *
 * The cobblestone row is the one that matters. `surface=cobblestone` with
 * `smoothness=bad` must stay cobblestone: the cobbles are WHY it is rough, and
 * replacing them with worn asphalt would throw away the actual surface in favour
 * of a guess about its condition.
 */
TEST(RoadStyle, smoothness_refines_only_a_default_variant) {
    // Asphalt, still default: the grades apply.
    EXPECT_KEY(surface_material(RoadType::Residential, "asphalt", "bad"),
               MaterialId::Asphalt, 1, "asphalt + bad");
    EXPECT_KEY(surface_material(RoadType::Residential, "asphalt", "very_bad"),
               MaterialId::Asphalt, 1, "asphalt + very_bad");
    EXPECT_KEY(surface_material(RoadType::Residential, "asphalt", "horrible"),
               MaterialId::Asphalt, 1, "asphalt + horrible");
    EXPECT_KEY(surface_material(RoadType::Residential, "asphalt", "very_horrible"),
               MaterialId::Asphalt, 1, "asphalt + very_horrible");
    EXPECT_KEY(surface_material(RoadType::Residential, "asphalt", "impassable"),
               MaterialId::Asphalt, 1, "asphalt + impassable");
    EXPECT_KEY(surface_material(RoadType::Residential, "asphalt", "excellent"),
               MaterialId::Asphalt, 2, "asphalt + excellent");

    // Neutral grades leave the default alone.
    EXPECT_KEY(surface_material(RoadType::Residential, "asphalt", "good"),
               MaterialId::Asphalt, 0, "asphalt + good");
    EXPECT_KEY(surface_material(RoadType::Residential, "asphalt", "intermediate"),
               MaterialId::Asphalt, 0, "asphalt + intermediate");
    EXPECT_KEY(surface_material(RoadType::Residential, "asphalt", "unknown_grade"),
               MaterialId::Asphalt, 0, "asphalt + an unknown grade");

    // Concrete has its own grades.
    EXPECT_KEY(surface_material(RoadType::Residential, "concrete", "bad"),
               MaterialId::Concrete, 1, "concrete + bad");
    EXPECT_KEY(surface_material(RoadType::Residential, "concrete", "excellent"),
               MaterialId::Concrete, 2, "concrete + excellent");

    // The class default is a default variant, so it takes the grade too.
    EXPECT_KEY(surface_material(RoadType::Primary, "", "bad"),
               MaterialId::Asphalt, 1, "no surface tag + bad on a primary");

    // A non-default variant is NOT overwritten.
    EXPECT_KEY(surface_material(RoadType::Residential, "cobblestone", "bad"),
               MaterialId::Asphalt, 3, "cobblestone + bad stays cobblestone");
    EXPECT_KEY(surface_material(RoadType::Residential, "sett", "excellent"),
               MaterialId::Asphalt, 4, "sett + excellent stays sett");
    EXPECT_KEY(surface_material(RoadType::Residential, "concrete:plates", "bad"),
               MaterialId::Concrete, 3, "concrete:plates + bad stays plates");

    // A slot with no grades is untouched however rough it is said to be.
    EXPECT_KEY(surface_material(RoadType::Path, "gravel", "very_bad"),
               MaterialId::Gravel, 0, "gravel + very_bad");
    EXPECT_KEY(surface_material(RoadType::Path, "ground", "impassable"),
               MaterialId::Dirt, 1, "ground + impassable");
    EXPECT_KEY(surface_material(RoadType::Path, "", "excellent"),
               MaterialId::Dirt, 1, "path default + excellent");
}

// ============================================================================
// strip_material
// ============================================================================

/**
 * The slot the profile builder chose is NOT negotiable.
 *
 * Returning a different slot would put the strip in a different SubMesh range
 * from the one the extruder opened for it, which is a corrupt index buffer rather
 * than a wrong colour. Swept over every kind, every slot and every probe surface,
 * with and without tags.
 */
TEST(RoadStyle, strip_material_never_changes_the_slot) {
    TagMap tags;
    tags["surface"] = "gravel";
    tags["kerb:material"] = "granite";
    tags["material"] = "stone";
    tags["footway:surface"] = "paving_stones";

    for (const StripKind kind : kKinds) {
        for (const MaterialId slot : kSlots) {
            for (const char* surface : kSurfaceProbes) {
                const MaterialKey with = strip_material(kind, slot, surface, &tags);
                const MaterialKey without = strip_material(kind, slot, surface, nullptr);
                CHECK_EQ(static_cast<int>(with.material), static_cast<int>(slot));
                CHECK_EQ(static_cast<int>(without.material), static_cast<int>(slot));

                // And whatever it chose must be a variant the slot actually has,
                // or the renderer indexes past the end of its per-slot array.
                CHECK_TRUE(with.variant < variant_count(slot));
                CHECK_TRUE(without.variant < variant_count(slot));
            }
        }
    }
}

/// A footway's own surface, and the carriageway's as weak evidence behind it
TEST(RoadStyle, sidewalk_variants_come_from_the_footway_surface_first) {
    EXPECT_KEY(strip_material(StripKind::Sidewalk, MaterialId::Sidewalk, "", nullptr),
               MaterialId::Sidewalk, 0, "an untagged footway");

    EXPECT_KEY(strip_material(StripKind::Sidewalk, MaterialId::Sidewalk, "paving_stones",
                              nullptr),
               MaterialId::Sidewalk, 1, "paving_stones footway");
    EXPECT_KEY(strip_material(StripKind::Sidewalk, MaterialId::Sidewalk, "asphalt", nullptr),
               MaterialId::Sidewalk, 3, "asphalt footway");
    EXPECT_KEY(strip_material(StripKind::Sidewalk, MaterialId::Sidewalk, "concrete", nullptr),
               MaterialId::Sidewalk, 4, "concrete footway");
    EXPECT_KEY(strip_material(StripKind::Sidewalk, MaterialId::Sidewalk, "sett", nullptr),
               MaterialId::Sidewalk, 5, "sett footway");

    // The narrow key beats the parent way's blanket surface tag.
    TagMap tags;
    tags["footway:surface"] = "concrete";
    EXPECT_KEY(strip_material(StripKind::Sidewalk, MaterialId::Sidewalk, "asphalt", &tags),
               MaterialId::Sidewalk, 4,
               "footway:surface=concrete over a way tagged surface=asphalt");

    // A substance from another slot has no near match inside Sidewalk.
    EXPECT_KEY(strip_material(StripKind::Sidewalk, MaterialId::Sidewalk, "gravel", nullptr),
               MaterialId::Sidewalk, 0, "surface=gravel on a Sidewalk strip");
}

/**
 * A kerb's variant comes from a tag about the KERB, never from the surface of the
 * road it stands beside.
 *
 * A cobbled street with a granite kerb is the common European case, and reading
 * the carriageway's `surface=cobblestone` onto the kerb would paint the granite
 * as cobbles. The carriageway surface is therefore not consulted for a kerb at
 * all; see surface_applies_to() in road_style.cpp.
 */
TEST(RoadStyle, kerb_variants_come_from_kerb_tags_only) {
    EXPECT_KEY(strip_material(StripKind::CurbTop, MaterialId::Curb, "", nullptr),
               MaterialId::Curb, 0, "an untagged kerb");
    EXPECT_KEY(strip_material(StripKind::CurbFace, MaterialId::Curb, "cobblestone", nullptr),
               MaterialId::Curb, 0, "a kerb beside a cobbled carriageway");

    TagMap granite;
    granite["kerb:material"] = "granite";
    EXPECT_KEY(strip_material(StripKind::CurbTop, MaterialId::Curb, "cobblestone", &granite),
               MaterialId::Curb, 1, "kerb:material=granite");
    EXPECT_KEY(strip_material(StripKind::CurbFace, MaterialId::Curb, "", &granite),
               MaterialId::Curb, 1, "kerb:material=granite on the face");

    TagMap stone;
    stone["material"] = "stone";
    EXPECT_KEY(strip_material(StripKind::CurbTop, MaterialId::Curb, "", &stone),
               MaterialId::Curb, 1, "material=stone");

    TagMap concrete;
    concrete["kerb:material"] = "concrete";
    EXPECT_KEY(strip_material(StripKind::CurbTop, MaterialId::Curb, "", &concrete),
               MaterialId::Curb, 0, "kerb:material=concrete is the slot default");

    TagMap rolled;
    rolled["kerb"] = "rolled";
    EXPECT_KEY(strip_material(StripKind::CurbTop, MaterialId::Curb, "", &rolled),
               MaterialId::Curb, 4, "kerb=rolled is a mountable kerb");

    TagMap lowered;
    lowered["kerb"] = "lowered";
    EXPECT_KEY(strip_material(StripKind::CurbFace, MaterialId::Curb, "", &lowered),
               MaterialId::Curb, 3, "kerb=lowered is a dropped kerb");

    // Case: a raw TagMap has not been through the parser's normalisation.
    TagMap shouty;
    shouty["kerb:material"] = "Granite";
    EXPECT_KEY(strip_material(StripKind::CurbTop, MaterialId::Curb, "", &shouty),
               MaterialId::Curb, 1, "kerb:material=Granite");
}

/**
 * A raised median on the Concrete slot is a pedestrian refuge, and is not painted
 * like a carriageway slab.
 *
 * Structural rather than inferred: the profile builder only ever emits that
 * pairing for a raised refuge, so the pairing itself is the evidence and it
 * outranks the way's surface tag.
 */
TEST(RoadStyle, a_concrete_median_is_an_island) {
    EXPECT_KEY(strip_material(StripKind::Median, MaterialId::Concrete, "", nullptr),
               MaterialId::Concrete, 5, "a raised concrete median");
    EXPECT_KEY(strip_material(StripKind::Median, MaterialId::Concrete, "asphalt", nullptr),
               MaterialId::Concrete, 5, "a raised concrete median on an asphalt road");

    // A painted median is not on the Concrete slot and is unaffected.
    EXPECT_KEY(strip_material(StripKind::Median, MaterialId::Asphalt, "asphalt", nullptr),
               MaterialId::Asphalt, 0, "a painted median");
    EXPECT_KEY(strip_material(StripKind::Median, MaterialId::Grass, "", nullptr),
               MaterialId::Grass, 0, "a planted median");
}

/// Structure slots read `material=*` and, unlike a kerb, their own running surface
TEST(RoadStyle, structure_slots_read_material_and_surface) {
    TagMap steel;
    steel["material"] = "steel";
    EXPECT_KEY(strip_material(StripKind::Lane, MaterialId::BridgeDeck, "", &steel),
               MaterialId::BridgeDeck, 1, "material=steel deck");
    EXPECT_KEY(strip_material(StripKind::Lane, MaterialId::Parapet, "", &steel),
               MaterialId::Parapet, 1, "material=steel parapet");

    EXPECT_KEY(strip_material(StripKind::Lane, MaterialId::BridgeDeck, "wood", nullptr),
               MaterialId::BridgeDeck, 2, "surface=wood deck");
    EXPECT_KEY(strip_material(StripKind::Lane, MaterialId::BridgeDeck, "metal", nullptr),
               MaterialId::BridgeDeck, 1, "surface=metal deck");
    EXPECT_KEY(strip_material(StripKind::Lane, MaterialId::BridgeDeck, "", nullptr),
               MaterialId::BridgeDeck, 0, "an untagged deck is concrete");

    TagMap masonry;
    masonry["material"] = "masonry";
    EXPECT_KEY(strip_material(StripKind::Lane, MaterialId::Parapet, "", &masonry),
               MaterialId::Parapet, 2, "material=masonry parapet");
}

/// Loose and unbound slots keep their own tables and their own defaults
TEST(RoadStyle, unpaved_slots_resolve_within_themselves) {
    EXPECT_KEY(strip_material(StripKind::Lane, MaterialId::Gravel, "compacted", nullptr),
               MaterialId::Gravel, 1, "compacted on a Gravel strip");
    EXPECT_KEY(strip_material(StripKind::Lane, MaterialId::Dirt, "mud", nullptr),
               MaterialId::Dirt, 3, "mud on a Dirt strip");
    EXPECT_KEY(strip_material(StripKind::Verge, MaterialId::Grass, "grass_paver", nullptr),
               MaterialId::Grass, 0,
               "a verge does not read the carriageway surface");
    EXPECT_KEY(strip_material(StripKind::Lane, MaterialId::Grass, "grass_paver", nullptr),
               MaterialId::Grass, 3, "grass_paver on a Grass running surface");
}

// ============================================================================
// Buildings
// ============================================================================

TEST(RoadStyle, building_wall_material_follows_the_documented_order) {
    struct Row { const char* material; uint16_t variant; };
    const Row rows[] = {
        {"brick", 1}, {"brick_block", 1}, {"stone", 2}, {"granite", 2},
        {"limestone", 2}, {"sandstone", 2}, {"concrete", 3}, {"plaster", 4},
        {"render", 4}, {"stucco", 4}, {"glass", 5}, {"metal", 6},
        {"wood", 7}, {"timber", 7},
    };

    for (const Row& row : rows) {
        TagMap tags;
        tags["building:material"] = row.material;
        // Driven from a class whose default is a different variant, so the tag is
        // proved to win rather than to coincide.
        EXPECT_KEY(building_wall_material(BuildingType::Church, &tags),
                   MaterialId::Wall, row.variant,
                   std::string("building:material=") + row.material);
    }

    // Class defaults, when no tag resolves.
    EXPECT_KEY(building_wall_material(BuildingType::House, nullptr), MaterialId::Wall, 1, "house");
    EXPECT_KEY(building_wall_material(BuildingType::Detached, nullptr), MaterialId::Wall, 1, "detached");
    EXPECT_KEY(building_wall_material(BuildingType::Residential, nullptr), MaterialId::Wall, 1, "residential");
    EXPECT_KEY(building_wall_material(BuildingType::Apartments, nullptr), MaterialId::Wall, 1, "apartments");
    EXPECT_KEY(building_wall_material(BuildingType::School, nullptr), MaterialId::Wall, 1, "school");
    EXPECT_KEY(building_wall_material(BuildingType::Hospital, nullptr), MaterialId::Wall, 1, "hospital");
    EXPECT_KEY(building_wall_material(BuildingType::Commercial, nullptr), MaterialId::Wall, 5, "commercial");
    EXPECT_KEY(building_wall_material(BuildingType::Office, nullptr), MaterialId::Wall, 5, "office");
    EXPECT_KEY(building_wall_material(BuildingType::Retail, nullptr), MaterialId::Wall, 5, "retail");
    EXPECT_KEY(building_wall_material(BuildingType::Industrial, nullptr), MaterialId::Wall, 6, "industrial");
    EXPECT_KEY(building_wall_material(BuildingType::Warehouse, nullptr), MaterialId::Wall, 6, "warehouse");
    EXPECT_KEY(building_wall_material(BuildingType::Garage, nullptr), MaterialId::Wall, 6, "garage");
    EXPECT_KEY(building_wall_material(BuildingType::Shed, nullptr), MaterialId::Wall, 6, "shed");
    EXPECT_KEY(building_wall_material(BuildingType::Church, nullptr), MaterialId::Wall, 2, "church");
    EXPECT_KEY(building_wall_material(BuildingType::Unknown, nullptr), MaterialId::Wall, 0, "unknown");

    // building:colour is deliberately NOT read: colour is a renderer-side tint over
    // a resolved material and does not belong on a finite, dense axis.
    TagMap coloured;
    coloured["building:colour"] = "#ff0000";
    EXPECT_KEY(building_wall_material(BuildingType::Church, &coloured),
               MaterialId::Wall, 2, "building:colour does not create a variant");
}

TEST(RoadStyle, building_roof_material_follows_the_documented_order) {
    struct Row { const char* material; uint16_t variant; };
    const Row rows[] = {
        {"roof_tiles", 1}, {"tiles", 1}, {"tile", 1}, {"slate", 2},
        {"metal", 3}, {"copper", 3}, {"zinc", 3},
        {"tar_paper", 4}, {"bitumen", 4}, {"gravel", 4},
        {"concrete", 5}, {"thatch", 6}, {"glass", 7},
    };

    for (const Row& row : rows) {
        TagMap tags;
        tags["roof:material"] = row.material;
        EXPECT_KEY(building_roof_material(RoofType::Unknown, &tags),
                   MaterialId::Roof, row.variant,
                   std::string("roof:material=") + row.material);
    }

    // A flat roof is a waterproofing layer and never tiles.
    EXPECT_KEY(building_roof_material(RoofType::Flat, nullptr), MaterialId::Roof, 4, "flat");
    EXPECT_KEY(building_roof_material(RoofType::Gabled, nullptr), MaterialId::Roof, 1, "gabled");
    EXPECT_KEY(building_roof_material(RoofType::Hipped, nullptr), MaterialId::Roof, 1, "hipped");
    EXPECT_KEY(building_roof_material(RoofType::Pyramidal, nullptr), MaterialId::Roof, 1, "pyramidal");
    EXPECT_KEY(building_roof_material(RoofType::Skillion, nullptr), MaterialId::Roof, 1, "skillion");
    EXPECT_KEY(building_roof_material(RoofType::Dome, nullptr), MaterialId::Roof, 3, "dome");
    EXPECT_KEY(building_roof_material(RoofType::Unknown, nullptr), MaterialId::Roof, 0, "unknown");

    // A tag beats the shape default, including on a flat roof.
    TagMap tiles;
    tiles["roof:material"] = "slate";
    EXPECT_KEY(building_roof_material(RoofType::Flat, &tiles), MaterialId::Roof, 2,
               "roof:material=slate on a flat roof");
}

// ============================================================================
// Amenities -- the forward hook
// ============================================================================

/**
 * Unresolved by design.
 *
 * Nothing in this branch places a bench or a lamp post: prop placement needs an
 * asset catalogue, an instancing path and a placement pass, and none of that
 * exists. What is asserted here is only that the IDENTIFIERS are stable, because
 * they are the half of the eventual feature that has to be frozen first -- they
 * end up in saved scenes the moment anything consumes them.
 */
TEST(RoadStyle, amenity_style_ids_are_the_frozen_numbers) {
    struct Row { const char* key; const char* value; uint16_t id; };
    const Row rows[] = {
        {"amenity", "bench", 1},
        {"amenity", "waste_basket", 2},
        {"highway", "street_lamp", 3},
        {"highway", "traffic_signals", 4},
        {"highway", "stop", 5},
        {"highway", "give_way", 6},
        {"barrier", "bollard", 7},
        {"barrier", "gate", 8},
        {"barrier", "lift_gate", 8},
        {"highway", "bus_stop", 9},
        {"public_transport", "platform", 9},
        {"amenity", "shelter", 10},
        {"amenity", "post_box", 11},
        {"amenity", "telephone", 12},
        {"amenity", "drinking_water", 13},
        {"amenity", "bicycle_parking", 14},
        {"emergency", "fire_hydrant", 16},
        {"natural", "tree", 17},
        {"amenity", "planter", 18},
        {"amenity", "fountain", 19},
        {"leisure", "picnic_table", 20},
        {"man_made", "street_cabinet", 21},
        {"power", "pole", 22},
        {"man_made", "utility_pole", 22},
    };

    for (const Row& row : rows) {
        TagMap tags;
        tags[row.key] = row.value;
        const uint16_t id = amenity_style_id(&tags);
        if (id != row.id) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "amenity id matches the frozen number",
                std::string(row.key) + "=" + row.value + ": got " +
                    std::to_string(id) + ", expected " + std::to_string(row.id));
        }
    }

    // A parking meter needs both halves of its tag pair.
    TagMap meter;
    meter["amenity"] = "vending_machine";
    meter["vending"] = "parking_tickets";
    CHECK_EQ(amenity_style_id(&meter), uint16_t{15});

    TagMap other_vending;
    other_vending["amenity"] = "vending_machine";
    other_vending["vending"] = "drinks";
    CHECK_EQ(amenity_style_id(&other_vending), amenity_styles::kNone);
}

/// Most OSM nodes are geometry carriers with no tags, and this runs per node
TEST(RoadStyle, an_unrecognised_node_resolves_to_none) {
    CHECK_EQ(amenity_style_id(nullptr), amenity_styles::kNone);

    TagMap empty;
    CHECK_EQ(amenity_style_id(&empty), amenity_styles::kNone);

    TagMap unrelated;
    unrelated["name"] = "Main Street";
    unrelated["source"] = "survey";
    CHECK_EQ(amenity_style_id(&unrelated), amenity_styles::kNone);
}

/// Declaration order decides, so a shelter at a bus stop is a bus stop
TEST(RoadStyle, first_declared_key_wins_when_several_match) {
    TagMap both;
    both["amenity"] = "shelter";
    both["highway"] = "bus_stop";
    CHECK_EQ(amenity_style_id(&both), amenity_styles::kBusStop);
}

// ============================================================================
// all_material_keys -- the enumeration the renderer pre-builds from
// ============================================================================

/**
 * Sorted ascending by packed(), no duplicates, and exactly
 * `sum over slots of variant_count(slot)` entries.
 *
 * Sorted matters because packed() orders by slot and then by variant, so a
 * renderer holding a flat per-slot array can walk this list once and fill it in
 * place instead of indexing per entry.
 */
TEST(RoadStyle, all_material_keys_is_sorted_dense_and_complete) {
    const std::vector<MaterialKey> keys = all_material_keys();

    size_t expected = 0;
    for (const MaterialId slot : kSlots) expected += variant_count(slot);
    CHECK_EQ(keys.size(), expected);

    std::set<uint32_t> seen;
    uint32_t previous = 0;
    bool first = true;
    for (const MaterialKey& key : keys) {
        CHECK_TRUE(seen.insert(key.packed()).second);
        if (!first) CHECK_TRUE(key.packed() > previous);
        previous = key.packed();
        first = false;

        // Dense from 0 within the slot.
        CHECK_TRUE(key.variant < variant_count(key.material));
    }

    // Every declared (slot, variant) pair is present, not merely the right count.
    for (const MaterialId slot : kSlots) {
        for (size_t v = 0; v < variant_count(slot); ++v) {
            const MaterialKey key{slot, static_cast<uint16_t>(v)};
            if (seen.count(key.packed()) == 0) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "every declared key is enumerated",
                    describe(key) + " is missing from all_material_keys()");
            }
        }
    }
}

/**
 * THE test that matters for the renderer.
 *
 * The renderer pre-builds its material library from all_material_keys() at
 * startup and never waits to see which keys an extract happens to produce. A key
 * a mapping function can return and the enumeration does not list is therefore
 * not a wrong colour -- it is a missing material at run time, on somebody else's
 * extract, months from now.
 *
 * So this drives EVERY mapping function over a table of inputs, rather than
 * trusting that the enumeration and the tables were edited together.
 */
TEST(RoadStyle, every_key_a_mapping_can_return_is_enumerated) {
    std::set<uint32_t> enumerated;
    for (const MaterialKey& key : all_material_keys()) enumerated.insert(key.packed());

    size_t checked = 0;
    const auto demand = [&](MaterialKey key, const std::string& context) {
        ++checked;
        if (enumerated.count(key.packed()) != 0) return;
        stratum::test::report_failure(
            __FILE__, __LINE__, "the returned key is enumerated",
            context + " produced " + describe(key) +
                ", which all_material_keys() does not list");
    };

    // surface_material over class x surface x smoothness
    for (const RoadType type : kRoadTypes) {
        for (const char* surface : kSurfaceProbes) {
            for (const char* smoothness : kSmoothnessProbes) {
                demand(surface_material(type, surface, smoothness),
                       std::string("surface_material(") + surface + ", " + smoothness + ")");
            }
        }
    }

    // strip_material over kind x slot x surface, with and without tags
    TagMap tags;
    tags["kerb:material"] = "granite";
    tags["kerb"] = "rolled";
    tags["material"] = "masonry";
    tags["footway:surface"] = "bricks";
    tags["sidewalk:both:surface"] = "concrete";

    for (const StripKind kind : kKinds) {
        for (const MaterialId slot : kSlots) {
            for (const char* surface : kSurfaceProbes) {
                const std::string context = std::string("strip_material(kind ") +
                                            std::to_string(static_cast<int>(kind)) + ", " +
                                            material_id_name(slot) + ", " + surface + ")";
                demand(strip_material(kind, slot, surface, nullptr), context);
                demand(strip_material(kind, slot, surface, &tags), context + " tagged");
            }
        }
    }

    // building_wall_material and building_roof_material over class x material
    for (const char* material : kSurfaceProbes) {
        TagMap wall_tags;
        wall_tags["building:material"] = material;
        TagMap roof_tags;
        roof_tags["roof:material"] = material;
        for (const BuildingType type : kBuildingTypes) {
            demand(building_wall_material(type, nullptr), "building_wall_material untagged");
            demand(building_wall_material(type, &wall_tags),
                   std::string("building_wall_material(") + material + ")");
        }
        for (const RoofType roof : kRoofTypes) {
            demand(building_roof_material(roof, nullptr), "building_roof_material untagged");
            demand(building_roof_material(roof, &roof_tags),
                   std::string("building_roof_material(") + material + ")");
        }
    }

    // A sweep that checked nothing would read as a pass.
    CHECK_TRUE(checked > 1000);
}

// ============================================================================
// material_key_name
// ============================================================================

/**
 * Unique across slots AND variants, because the name is written into an OBJ
 * `usemtl` line and a glTF material name, where two keys sharing a name become
 * one material and the geometry of one of them silently changes appearance.
 */
TEST(RoadStyle, material_key_name_is_unique_per_key) {
    std::map<std::string, MaterialKey> by_name;

    for (const MaterialKey& key : all_material_keys()) {
        const std::string name = material_key_name(key);
        CHECK_FALSE(name.empty());

        // An exporter writes this straight into a token-delimited line.
        CHECK_TRUE(name.find(' ') == std::string::npos);
        CHECK_TRUE(name.find('\n') == std::string::npos);
        CHECK_TRUE(name.find('\t') == std::string::npos);

        const auto it = by_name.find(name);
        if (it != by_name.end()) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "material names are unique",
                "\"" + name + "\" is shared by " + describe(it->second) + " and " +
                    describe(key));
            continue;
        }
        by_name.emplace(name, key);
    }

    CHECK_EQ(by_name.size(), all_material_keys().size());
}

/**
 * Variant 0 keeps exactly the name the slot had before the variant axis existed,
 * so every mesh produced by an earlier build names its materials identically.
 */
TEST(RoadStyle, variant_zero_is_named_by_the_slot_alone) {
    for (const MaterialId slot : kSlots) {
        CHECK_EQ(material_key_name(MaterialKey{slot, 0}), std::string(material_id_name(slot)));
    }

    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Asphalt, 0}), std::string("Asphalt"));
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Sidewalk, 0}), std::string("Sidewalk"));
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Wall, 0}), std::string("Wall"));
}

/// The frozen spellings. These travel in exported files; renaming one breaks a user's engine.
TEST(RoadStyle, non_zero_variants_are_named_slot_dot_variant) {
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Asphalt, 3}),
             std::string("Asphalt.Cobblestone"));
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Asphalt, 5}),
             std::string("Asphalt.PavingStones"));
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Sidewalk, 2}),
             std::string("Sidewalk.Tactile"));
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Curb, 1}),
             std::string("Curb.Granite"));
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Concrete, 5}),
             std::string("Concrete.Island"));
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Roof, 6}),
             std::string("Roof.Thatch"));
}

/**
 * A mesh saved by a newer build and opened by an older one must still name its
 * materials distinguishably, rather than collapsing every unknown variant onto
 * the slot default and merging geometry that was never one material.
 */
TEST(RoadStyle, an_out_of_range_key_still_gets_a_distinct_name) {
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Asphalt, 97}),
             std::string("Asphalt.Variant97"));
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Asphalt, 98}),
             std::string("Asphalt.Variant98"));

    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Count, 0}), std::string("Unknown"));
    CHECK_EQ(material_key_name(MaterialKey{MaterialId::Count, 4}),
             std::string("Unknown.Variant4"));
    CHECK_EQ(material_key_name(MaterialKey{static_cast<MaterialId>(200), 0}),
             std::string("Unknown"));
    CHECK_EQ(material_key_name(MaterialKey{static_cast<MaterialId>(200), 7}),
             std::string("Unknown.Variant7"));

    // And an out-of-range name must not collide with an in-range one.
    std::set<std::string> names;
    for (const MaterialKey& key : all_material_keys()) names.insert(material_key_name(key));
    CHECK_TRUE(names.count("Asphalt.Variant97") == 0);
    CHECK_TRUE(names.count("Unknown.Variant4") == 0);
}
