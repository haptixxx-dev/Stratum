/**
 * @file road_style.cpp
 * @brief Implementation of the single tag-to-appearance mapping
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The whole file is table lookup. That is deliberate: every rule that decides
 * what a surface looks like is a row somewhere below, so adding a surface value
 * is adding a row rather than writing a branch, and the tables can be walked by a
 * test to prove that every key a mapping function can return is also enumerated
 * by all_material_keys().
 *
 * ### Why linear scans and not a hash map
 *
 * The tables are between four and thirty entries. A `std::string` hash costs more
 * than the scan does, the scan needs no static initialisation, and none of these
 * functions is called per vertex -- surface_material() and strip_material() run
 * once per EDGE and once per STRIP respectively, so the whole Lucan extract is
 * roughly ten thousand calls. A map would trade a measurable startup cost and a
 * thread-safety question for nothing.
 *
 * ### Case
 *
 * The @p surface and @p smoothness parameters arrive already lowercased from the
 * parser, and the header says so. Values pulled out of a raw TagMap have NOT been
 * through the parser's normalisation, so those are lowercased here, in the one
 * place it costs nothing: after a key has already matched.
 */

#include "osm/road/road_style.hpp"

#include <cctype>
#include <string_view>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Small helpers
// ============================================================================

/**
 * @brief ASCII lowercase copy
 *
 * OSM tag values are conventionally lowercase but not guaranteed to be, and a
 * `building:material=Brick` that fell through to the class default would be an
 * invisible defect. Only applied to values read out of a raw TagMap.
 */
std::string to_lower_ascii(std::string_view in) {
    std::string out;
    out.reserve(in.size());
    for (const char c : in) {
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return out;
}

/**
 * @brief Lowercased value of one tag
 *
 * @param tags Tag map; may be null
 * @param key  Key to read
 * @return The value, lowercased; empty when @p tags is null or the key is absent
 */
std::string tag_value(const TagMap* tags, const char* key) {
    if (tags == nullptr) return {};
    const auto it = tags->find(key);
    if (it == tags->end() || it->second.empty()) return {};
    return to_lower_ascii(it->second);
}

/// True when @p value equals any of @p count alternatives
bool value_is_any_of(std::string_view value, const char* const* alternatives, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (value == alternatives[i]) return true;
    }
    return false;
}

/// Array-length helper, so a table and its size can never disagree
template <typename T, size_t N>
constexpr size_t array_size(const T (&)[N]) {
    return N;
}

// ============================================================================
// Variant counts
//
// Rule 3 of the numbering rule: values are dense from 0, so a count is the whole
// truth about which variants exist in a slot. Every one of these is one more than
// the last constant declared in the slot's block in road_style.hpp, and the two
// must be edited together.
// ============================================================================

constexpr size_t kVariantCounts[] = {
    1,  // Default
    8,  // Asphalt      0..7  default, worn, smooth, cobblestone, sett, paving stones, chipseal, coloured
    6,  // Concrete     0..5  default, worn, smooth, plates, lanes, island
    5,  // Curb         0..4  default, granite, asphalt, dropped, mountable
    6,  // Sidewalk     0..5  default, paved, tactile, asphalt, concrete, brick
    3,  // Markings     0..2  default, yellow, worn
    5,  // Gravel       0..4  default, compacted, fine, pebble, unpaved
    5,  // Dirt         0..4  default, ground, earth, mud, sand
    5,  // Grass        0..4  default, mown, rough, paver, planted
    4,  // BridgeDeck   0..3  default, steel, wood, stone
    4,  // Parapet      0..3  default, steel, stone, concrete
    8,  // Wall         0..7  default, brick, stone, concrete, render, glass, metal, wood
    8,  // Roof         0..7  default, tile, slate, metal, membrane, concrete, thatch, glass
};

static_assert(array_size(kVariantCounts) == static_cast<size_t>(MaterialId::Count),
              "kVariantCounts must have one entry per MaterialId slot before Count");

// ============================================================================
// Variant names
//
// The constant's name with its `k` prefix and its redundant slot prefix removed,
// exactly as material_key_name() documents. FROZEN: these travel in exported OBJ
// `usemtl` lines and glTF material names, so renaming one breaks a material
// assignment a user already made in their engine.
//
// Index 0 of each row is never read -- variant 0 is named by the slot alone --
// but is present so the row index equals the variant number.
// ============================================================================

const char* const kDefaultNames[]    = {""};
const char* const kAsphaltNames[]    = {"", "Worn", "Smooth", "Cobblestone", "Sett",
                                        "PavingStones", "Chipseal", "Coloured"};
const char* const kConcreteNames[]   = {"", "Worn", "Smooth", "Plates", "Lanes", "Island"};
const char* const kCurbNames[]       = {"", "Granite", "Asphalt", "Dropped", "Mountable"};
const char* const kSidewalkNames[]   = {"", "Paved", "Tactile", "Asphalt", "Concrete", "Brick"};
const char* const kMarkingsNames[]   = {"", "Yellow", "Worn"};
const char* const kGravelNames[]     = {"", "Compacted", "Fine", "Pebble", "Unpaved"};
const char* const kDirtNames[]       = {"", "Ground", "Earth", "Mud", "Sand"};
const char* const kGrassNames[]      = {"", "Mown", "Rough", "Paver", "Planted"};
const char* const kBridgeDeckNames[] = {"", "Steel", "Wood", "Stone"};
const char* const kParapetNames[]    = {"", "Steel", "Stone", "Concrete"};
const char* const kWallNames[]       = {"", "Brick", "Stone", "Concrete", "Render",
                                        "Glass", "Metal", "Wood"};
const char* const kRoofNames[]       = {"", "Tile", "Slate", "Metal", "Membrane",
                                        "Concrete", "Thatch", "Glass"};

const char* const* const kVariantNames[] = {
    kDefaultNames, kAsphaltNames, kConcreteNames, kCurbNames,  kSidewalkNames,
    kMarkingsNames, kGravelNames, kDirtNames,     kGrassNames, kBridgeDeckNames,
    kParapetNames,  kWallNames,   kRoofNames,
};

static_assert(array_size(kVariantNames) == static_cast<size_t>(MaterialId::Count),
              "kVariantNames must have one row per MaterialId slot before Count");

// ============================================================================
// surface=* -> running surface
//
// The table surface_material() step 1 consults. The SLOT changes, not just the
// variant: a gravel road is not asphalt with a different texture, and putting it
// in the Asphalt slot would hand the renderer a bound-surface material for a
// loose one.
//
// Only RUNNING SURFACES appear here. `surface=metal` and `surface=wood` are real
// OSM values but they describe a bridge deck rather than a carriageway, so they
// are reached through strip_material() with slot BridgeDeck instead; resolving
// them here would move an ordinary road into the BridgeDeck SubMesh range.
// ============================================================================

struct SurfaceRow {
    const char* value;
    MaterialId  slot;
    uint16_t    variant;
};

constexpr SurfaceRow kSurfaceTable[] = {
    // Bound bituminous
    {"asphalt",                MaterialId::Asphalt,  variants::kAsphaltDefault},
    {"paved",                  MaterialId::Asphalt,  variants::kAsphaltDefault},
    {"asphalt:lanes",          MaterialId::Asphalt,  variants::kAsphaltDefault},
    {"bitmac",                 MaterialId::Asphalt,  variants::kAsphaltDefault},
    {"tarmac",                 MaterialId::Asphalt,  variants::kAsphaltDefault},
    {"chipseal",               MaterialId::Asphalt,  variants::kChipseal},
    {"tar",                    MaterialId::Asphalt,  variants::kChipseal},
    {"bitumen",                MaterialId::Asphalt,  variants::kChipseal},

    // Modular running surfaces: driveable, so the Asphalt slot, different variants
    {"cobblestone",            MaterialId::Asphalt,  variants::kCobblestone},
    {"unhewn_cobblestone",     MaterialId::Asphalt,  variants::kCobblestone},
    {"sett",                   MaterialId::Asphalt,  variants::kSett},
    {"cobblestone:flattened",  MaterialId::Asphalt,  variants::kSett},
    {"paving_stones",          MaterialId::Asphalt,  variants::kPavingStones},
    {"bricks",                 MaterialId::Asphalt,  variants::kPavingStones},
    {"brick",                  MaterialId::Asphalt,  variants::kPavingStones},

    // Rigid
    {"concrete",               MaterialId::Concrete, variants::kConcreteDefault},
    {"cement",                 MaterialId::Concrete, variants::kConcreteDefault},
    {"concrete:plates",        MaterialId::Concrete, variants::kConcretePlates},
    {"concrete:lanes",         MaterialId::Concrete, variants::kConcreteLanes},

    // Loose and compacted mineral
    {"gravel",                 MaterialId::Gravel,   variants::kGravelDefault},
    {"compacted",              MaterialId::Gravel,   variants::kGravelCompacted},
    {"fine_gravel",            MaterialId::Gravel,   variants::kGravelFine},
    {"pebblestone",            MaterialId::Gravel,   variants::kGravelPebble},
    {"unpaved",                MaterialId::Gravel,   variants::kGravelUnpaved},

    // Unbound earth
    {"ground",                 MaterialId::Dirt,     variants::kDirtGround},
    {"dirt",                   MaterialId::Dirt,     variants::kDirtEarth},
    {"earth",                  MaterialId::Dirt,     variants::kDirtEarth},
    {"soil",                   MaterialId::Dirt,     variants::kDirtEarth},
    {"mud",                    MaterialId::Dirt,     variants::kDirtMud},
    {"sand",                   MaterialId::Dirt,     variants::kDirtSand},

    // Planted
    {"grass",                  MaterialId::Grass,    variants::kGrassDefault},
    {"grass_paver",            MaterialId::Grass,    variants::kGrassPaver},
};

/**
 * @brief Look up a surface value in the running-surface table
 *
 * @param surface Lowercased `surface=*` value
 * @param out     Receives the key on a hit; untouched on a miss
 * @return True on a hit
 */
bool lookup_surface(std::string_view surface, MaterialKey& out) {
    if (surface.empty()) return false;
    for (const SurfaceRow& row : kSurfaceTable) {
        if (surface == row.value) {
            out = MaterialKey{row.slot, row.variant};
            return true;
        }
    }
    return false;
}

// ============================================================================
// smoothness=*
// ============================================================================

const char* const kWornSmoothness[] = {"bad", "very_bad", "horrible", "very_horrible",
                                       "impassable"};
const char* const kSmoothSmoothness[] = {"excellent"};

/// True for `bad` and everything below it
bool smoothness_is_worn(std::string_view s) {
    return value_is_any_of(s, kWornSmoothness, array_size(kWornSmoothness));
}

/// True for `excellent` only; `good` and `intermediate` leave the default alone
bool smoothness_is_smooth(std::string_view s) {
    return value_is_any_of(s, kSmoothSmoothness, array_size(kSmoothSmoothness));
}

// ============================================================================
// Per-slot "nearest variant" tables for strip_material()
//
// strip_material() may NEVER change the slot the profile builder chose, so a
// surface value naming a substance from another slot has to be resolved to the
// nearest variant WITHIN the given slot, or to that slot's default when there is
// no near match. These rows are that mapping, one table per slot that has one.
// ============================================================================

struct VariantRow {
    const char* value;
    uint16_t    variant;
};

/// Sidewalk slot: a footway is paving, asphalt, concrete or brick
constexpr VariantRow kSidewalkSurfaces[] = {
    {"paving_stones",         variants::kSidewalkPaved},
    {"paving_stones:30",      variants::kSidewalkPaved},
    {"bricks",                variants::kSidewalkBrick},
    {"brick",                 variants::kSidewalkBrick},
    {"sett",                  variants::kSidewalkBrick},
    {"cobblestone",           variants::kSidewalkBrick},
    {"unhewn_cobblestone",    variants::kSidewalkBrick},
    {"cobblestone:flattened", variants::kSidewalkBrick},
    {"asphalt",               variants::kSidewalkAsphalt},
    {"paved",                 variants::kSidewalkAsphalt},
    {"concrete",              variants::kSidewalkConcrete},
    {"concrete:plates",       variants::kSidewalkConcrete},
    {"concrete:lanes",        variants::kSidewalkConcrete},
};

/// Asphalt slot: the modular running surfaces and the dressings
constexpr VariantRow kAsphaltSurfaces[] = {
    {"cobblestone",           variants::kCobblestone},
    {"unhewn_cobblestone",    variants::kCobblestone},
    {"sett",                  variants::kSett},
    {"cobblestone:flattened", variants::kSett},
    {"paving_stones",         variants::kPavingStones},
    {"bricks",                variants::kPavingStones},
    {"brick",                 variants::kPavingStones},
    {"chipseal",              variants::kChipseal},
    {"tar",                   variants::kChipseal},
    {"bitumen",               variants::kChipseal},
};

/// Concrete slot
constexpr VariantRow kConcreteSurfaces[] = {
    {"concrete:plates", variants::kConcretePlates},
    {"concrete:lanes",  variants::kConcreteLanes},
};

/// Gravel slot
constexpr VariantRow kGravelSurfaces[] = {
    {"gravel",      variants::kGravelDefault},
    {"compacted",   variants::kGravelCompacted},
    {"fine_gravel", variants::kGravelFine},
    {"pebblestone", variants::kGravelPebble},
    {"unpaved",     variants::kGravelUnpaved},
};

/// Dirt slot
constexpr VariantRow kDirtSurfaces[] = {
    {"ground", variants::kDirtGround},
    {"dirt",   variants::kDirtEarth},
    {"earth",  variants::kDirtEarth},
    {"soil",   variants::kDirtEarth},
    {"mud",    variants::kDirtMud},
    {"sand",   variants::kDirtSand},
};

/// Grass slot
constexpr VariantRow kGrassSurfaces[] = {
    {"grass_paver", variants::kGrassPaver},
};

/// Curb slot, from kerb:material / material / kerb
constexpr VariantRow kCurbSurfaces[] = {
    {"granite",   variants::kCurbGranite},
    {"stone",     variants::kCurbGranite},
    {"limestone", variants::kCurbGranite},
    {"sandstone", variants::kCurbGranite},
    {"basalt",    variants::kCurbGranite},
    {"asphalt",   variants::kCurbAsphalt},
    {"bitumen",   variants::kCurbAsphalt},
    {"concrete",  variants::kCurbDefault},
    // Shape values of kerb=*, which say what the kerb DOES rather than what it is
    {"lowered",   variants::kCurbDropped},
    {"flush",     variants::kCurbDropped},
    {"rolled",    variants::kCurbMountable},
    {"raised",    variants::kCurbDefault},
};

/// BridgeDeck slot, from surface=* or material=*
constexpr VariantRow kBridgeDeckSurfaces[] = {
    {"metal",    variants::kBridgeDeckSteel},
    {"steel",    variants::kBridgeDeckSteel},
    {"wood",     variants::kBridgeDeckWood},
    {"timber",   variants::kBridgeDeckWood},
    {"stone",    variants::kBridgeDeckStone},
    {"masonry",  variants::kBridgeDeckStone},
    {"brick",    variants::kBridgeDeckStone},
    {"concrete", variants::kBridgeDeckDefault},
};

/// Parapet slot, from material=*
constexpr VariantRow kParapetSurfaces[] = {
    {"metal",    variants::kParapetSteel},
    {"steel",    variants::kParapetSteel},
    {"iron",     variants::kParapetSteel},
    {"stone",    variants::kParapetStone},
    {"granite",  variants::kParapetStone},
    {"masonry",  variants::kParapetStone},
    {"brick",    variants::kParapetStone},
    {"concrete", variants::kParapetConcrete},
};

/// Building facade, from building:material=*
constexpr VariantRow kWallMaterials[] = {
    {"brick",       variants::kWallBrick},
    {"brick_block", variants::kWallBrick},
    {"bricks",      variants::kWallBrick},
    {"stone",       variants::kWallStone},
    {"granite",     variants::kWallStone},
    {"limestone",   variants::kWallStone},
    {"sandstone",   variants::kWallStone},
    {"concrete",    variants::kWallConcrete},
    {"plaster",     variants::kWallRender},
    {"render",      variants::kWallRender},
    {"stucco",      variants::kWallRender},
    {"glass",       variants::kWallGlass},
    {"metal",       variants::kWallMetal},
    {"wood",        variants::kWallWood},
    {"timber",      variants::kWallWood},
};

/// Building roof, from roof:material=*
constexpr VariantRow kRoofMaterials[] = {
    {"roof_tiles", variants::kRoofTile},
    {"tiles",      variants::kRoofTile},
    {"tile",       variants::kRoofTile},
    {"slate",      variants::kRoofSlate},
    {"metal",      variants::kRoofMetal},
    {"copper",     variants::kRoofMetal},
    {"zinc",       variants::kRoofMetal},
    {"tar_paper",  variants::kRoofMembrane},
    {"bitumen",    variants::kRoofMembrane},
    {"gravel",     variants::kRoofMembrane},
    {"concrete",   variants::kRoofConcrete},
    {"thatch",     variants::kRoofThatch},
    {"glass",      variants::kRoofGlass},
};

/**
 * @brief Look a value up in a per-slot variant table
 *
 * @param value Lowercased tag value; an empty value never matches
 * @param table Rows to scan
 * @param count Row count
 * @param out   Receives the variant on a hit; untouched on a miss
 * @return True on a hit
 */
bool lookup_variant(std::string_view value, const VariantRow* table, size_t count,
                    uint16_t& out) {
    if (value.empty()) return false;
    for (size_t i = 0; i < count; ++i) {
        if (value == table[i].value) {
            out = table[i].variant;
            return true;
        }
    }
    return false;
}

/**
 * @brief First RECOGNISED value across a priority list of keys
 *
 * Not the first key that is PRESENT. A way carrying both `material=steel` and
 * `kerb=lowered` has a value for the higher-priority key that this slot's table
 * does not know, and stopping there would throw away the one it does know.
 *
 * @param tags        Tag map; may be null
 * @param keys        Keys to try, in priority order
 * @param key_count   Number of keys
 * @param table       Slot's variant table
 * @param table_count Rows in @p table
 * @param out         Receives the variant on a hit; untouched on a miss
 * @return True on a hit
 */
bool lookup_tag_variant(const TagMap* tags, const char* const* keys, size_t key_count,
                        const VariantRow* table, size_t table_count, uint16_t& out) {
    if (tags == nullptr) return false;
    for (size_t i = 0; i < key_count; ++i) {
        const std::string value = tag_value(tags, keys[i]);
        if (lookup_variant(value, table, table_count, out)) return true;
    }
    return false;
}

/**
 * @brief The variant table for a slot, or null when the slot has no tag-driven variants
 *
 * MaterialId::Markings has variants but none of them is tag-derived -- the paint
 * colour comes from what is being drawn, not from a `surface=*` on the way -- so
 * it is deliberately absent, as are Default, Wall and Roof, which are reached
 * through their own entry points.
 */
const VariantRow* variant_table_for(MaterialId slot, size_t& out_count) {
    switch (slot) {
        case MaterialId::Asphalt:
            out_count = array_size(kAsphaltSurfaces);
            return kAsphaltSurfaces;
        case MaterialId::Concrete:
            out_count = array_size(kConcreteSurfaces);
            return kConcreteSurfaces;
        case MaterialId::Curb:
            out_count = array_size(kCurbSurfaces);
            return kCurbSurfaces;
        case MaterialId::Sidewalk:
            out_count = array_size(kSidewalkSurfaces);
            return kSidewalkSurfaces;
        case MaterialId::Gravel:
            out_count = array_size(kGravelSurfaces);
            return kGravelSurfaces;
        case MaterialId::Dirt:
            out_count = array_size(kDirtSurfaces);
            return kDirtSurfaces;
        case MaterialId::Grass:
            out_count = array_size(kGrassSurfaces);
            return kGrassSurfaces;
        case MaterialId::BridgeDeck:
            out_count = array_size(kBridgeDeckSurfaces);
            return kBridgeDeckSurfaces;
        case MaterialId::Parapet:
            out_count = array_size(kParapetSurfaces);
            return kParapetSurfaces;
        default:
            out_count = 0;
            return nullptr;
    }
}

/**
 * @brief Whether the parent way's `surface=*` is credible evidence for this strip
 *
 * The carriageway's surface tag says nothing about a kerb, a verge or a central
 * reservation -- OSM tags those separately when it tags them at all -- so
 * consulting it there would paint a granite kerb as cobblestone the moment the
 * street it sits on is cobbled. It IS credible for anything that is part of the
 * paved running surface, and for a footway on a way whose whole surface is tagged
 * (a pedestrianised street).
 */
bool surface_applies_to(StripKind kind) {
    switch (kind) {
        case StripKind::Lane:
        case StripKind::Gutter:
        case StripKind::Shoulder:
        case StripKind::CycleLane:
        case StripKind::ParkingLane:
        case StripKind::Sidewalk:
            return true;
        case StripKind::CurbFace:
        case StripKind::CurbTop:
        case StripKind::Median:
        case StripKind::Verge:
        case StripKind::Count:
            return false;
    }
    return false;
}

/// Keys carrying a footway's own surface, in priority order
const char* const kFootwaySurfaceKeys[] = {
    "footway:surface",
    "sidewalk:both:surface",
    "sidewalk:left:surface",
    "sidewalk:right:surface",
};

/// Keys carrying a kerb's material or shape, in priority order
const char* const kKerbKeys[] = {
    "kerb:material",
    "material",
    "kerb",
};

/// Keys carrying a structure's material, in priority order
const char* const kStructureKeys[] = {
    "material",
};

} // namespace

// ============================================================================
// Variant counts
// ============================================================================

size_t variant_count(MaterialId slot) {
    const auto index = static_cast<size_t>(slot);
    if (index >= static_cast<size_t>(MaterialId::Count)) return 0;
    return kVariantCounts[index];
}

// ============================================================================
// Roads
// ============================================================================

bool surface_material_from_tag(const std::string& surface, MaterialKey& out) {
    return lookup_surface(surface, out);
}

MaterialKey surface_material(RoadType type,
                             const std::string& surface,
                             const std::string& smoothness) {
    MaterialKey key{};

    // Step 1: the surface tag decides the slot AND the variant when it is known.
    const bool from_tag = surface_material_from_tag(surface, key);

    // Step 2: the road class decides both when no surface tag resolved.
    if (!from_tag) {
        if (type == RoadType::Path) {
            // A path or a track with no surface tag is not sealed. Everything else
            // -- including a footway, which is paved far more often than not -- is.
            key = MaterialKey{MaterialId::Dirt, variants::kDirtGround};
        } else {
            key = MaterialKey{MaterialId::Asphalt, variants::kAsphaltDefault};
        }
    }

    // Step 3: smoothness refines the variant, and only a variant that is still
    // the slot default, and only on a slot that defines the grades. Cobblestone
    // with smoothness=bad stays cobblestone: the cobbles are WHY it is rough, and
    // replacing them with worn asphalt would lose the actual surface.
    const bool grades_exist =
        key.material == MaterialId::Asphalt || key.material == MaterialId::Concrete;
    if (grades_exist && key.variant == 0 && !smoothness.empty()) {
        if (smoothness_is_worn(smoothness)) {
            key.variant = (key.material == MaterialId::Asphalt) ? variants::kAsphaltWorn
                                                                : variants::kConcreteWorn;
        } else if (smoothness_is_smooth(smoothness)) {
            key.variant = (key.material == MaterialId::Asphalt) ? variants::kAsphaltSmooth
                                                                : variants::kConcreteSmooth;
        }
    }

    return key;
}

MaterialKey strip_material(StripKind kind,
                           MaterialId slot,
                           const std::string& surface,
                           const TagMap* tags) {
    // Structural, and it outranks a tag: the profile builder only ever pairs
    // StripKind::Median with MaterialId::Concrete for a RAISED refuge, so the
    // pairing itself is the evidence.
    if (kind == StripKind::Median && slot == MaterialId::Concrete) {
        return MaterialKey{MaterialId::Concrete, variants::kConcreteIsland};
    }

    size_t table_count = 0;
    const VariantRow* table = variant_table_for(slot, table_count);
    if (table == nullptr) {
        return MaterialKey{slot, 0};
    }

    uint16_t variant = 0;

    // Evidence order runs narrowest first. A key naming this very strip beats the
    // parent way's blanket surface tag, which is only ever weak evidence here.
    if (slot == MaterialId::Curb) {
        if (lookup_tag_variant(tags, kKerbKeys, array_size(kKerbKeys), table, table_count,
                               variant)) {
            return MaterialKey{slot, variant};
        }
        return MaterialKey{slot, variants::kCurbDefault};
    }

    if (slot == MaterialId::BridgeDeck || slot == MaterialId::Parapet) {
        if (lookup_tag_variant(tags, kStructureKeys, array_size(kStructureKeys), table,
                               table_count, variant)) {
            return MaterialKey{slot, variant};
        }
        // A deck's own surface=* is the running surface of the deck, so it is
        // credible here in a way it is not for a kerb.
        if (lookup_variant(surface, table, table_count, variant)) {
            return MaterialKey{slot, variant};
        }
        return MaterialKey{slot, 0};
    }

    if (kind == StripKind::Sidewalk &&
        lookup_tag_variant(tags, kFootwaySurfaceKeys, array_size(kFootwaySurfaceKeys),
                           table, table_count, variant)) {
        return MaterialKey{slot, variant};
    }

    if (surface_applies_to(kind) && lookup_variant(surface, table, table_count, variant)) {
        return MaterialKey{slot, variant};
    }

    return MaterialKey{slot, 0};
}

// ============================================================================
// Naming and enumeration
// ============================================================================

std::string material_key_name(MaterialKey key) {
    const auto slot_index = static_cast<size_t>(key.material);
    const bool slot_valid = slot_index < static_cast<size_t>(MaterialId::Count);

    if (!slot_valid) {
        // material_id_name() already answers "Unknown" here, and a mesh written by
        // a newer build must still name its materials distinguishably.
        if (key.variant == 0) return "Unknown";
        return "Unknown.Variant" + std::to_string(key.variant);
    }

    const std::string slot_name = material_id_name(key.material);
    if (key.variant == 0) {
        return slot_name;
    }

    if (static_cast<size_t>(key.variant) < kVariantCounts[slot_index]) {
        return slot_name + "." + kVariantNames[slot_index][key.variant];
    }

    return slot_name + ".Variant" + std::to_string(key.variant);
}

std::vector<MaterialKey> all_material_keys() {
    std::vector<MaterialKey> keys;

    size_t total = 0;
    for (const size_t count : kVariantCounts) total += count;
    keys.reserve(total);

    // MaterialId values ascend and variants are dense from 0, so walking slots in
    // enum order and variants in numeric order is already ascending by packed().
    for (size_t slot = 0; slot < static_cast<size_t>(MaterialId::Count); ++slot) {
        const auto material = static_cast<MaterialId>(slot);
        for (size_t v = 0; v < kVariantCounts[slot]; ++v) {
            keys.push_back(MaterialKey{material, static_cast<uint16_t>(v)});
        }
    }

    return keys;
}

// ============================================================================
// Buildings
// ============================================================================

MaterialKey building_wall_material(BuildingType type, const TagMap* tags) {
    uint16_t variant = 0;

    const std::string material = tag_value(tags, "building:material");
    if (lookup_variant(material, kWallMaterials, array_size(kWallMaterials), variant)) {
        return MaterialKey{MaterialId::Wall, variant};
    }

    switch (type) {
        case BuildingType::House:
        case BuildingType::Detached:
        case BuildingType::Residential:
        case BuildingType::Apartments:
        case BuildingType::School:
        case BuildingType::Hospital:
            variant = variants::kWallBrick;
            break;
        case BuildingType::Commercial:
        case BuildingType::Office:
        case BuildingType::Retail:
            variant = variants::kWallGlass;
            break;
        case BuildingType::Industrial:
        case BuildingType::Warehouse:
        case BuildingType::Garage:
        case BuildingType::Shed:
            variant = variants::kWallMetal;
            break;
        case BuildingType::Church:
            variant = variants::kWallStone;
            break;
        case BuildingType::Unknown:
            variant = variants::kWallDefault;
            break;
    }

    return MaterialKey{MaterialId::Wall, variant};
}

MaterialKey building_roof_material(RoofType roof, const TagMap* tags) {
    uint16_t variant = 0;

    const std::string material = tag_value(tags, "roof:material");
    if (lookup_variant(material, kRoofMaterials, array_size(kRoofMaterials), variant)) {
        return MaterialKey{MaterialId::Roof, variant};
    }

    switch (roof) {
        case RoofType::Flat:
            // A flat roof is a waterproofing layer, never tiles.
            variant = variants::kRoofMembrane;
            break;
        case RoofType::Gabled:
        case RoofType::Hipped:
        case RoofType::Pyramidal:
        case RoofType::Skillion:
            variant = variants::kRoofTile;
            break;
        case RoofType::Dome:
            variant = variants::kRoofMetal;
            break;
        case RoofType::Unknown:
            variant = variants::kRoofDefault;
            break;
    }

    return MaterialKey{MaterialId::Roof, variant};
}

// ============================================================================
// Amenities
// ============================================================================

uint16_t amenity_style_id(const TagMap* tags) {
    if (tags == nullptr || tags->empty()) return amenity_styles::kNone;

    // Read each key once, not once per candidate value. Most nodes carry none of
    // them, and this runs per node on a 63 MB extract.
    const std::string amenity = tag_value(tags, "amenity");
    const std::string highway = tag_value(tags, "highway");
    const std::string barrier = tag_value(tags, "barrier");

    // Declaration order of amenity_styles, so a node carrying both
    // amenity=shelter and highway=bus_stop resolves as the bus stop.
    if (amenity == "bench")           return amenity_styles::kBench;
    if (amenity == "waste_basket")    return amenity_styles::kWasteBasket;
    if (highway == "street_lamp")     return amenity_styles::kStreetLamp;
    if (highway == "traffic_signals") return amenity_styles::kTrafficSignal;
    if (highway == "stop")            return amenity_styles::kStopSign;
    if (highway == "give_way")        return amenity_styles::kGiveWaySign;
    if (barrier == "bollard")         return amenity_styles::kBollard;
    if (barrier == "gate" || barrier == "lift_gate") return amenity_styles::kGate;

    if (highway == "bus_stop" ||
        tag_value(tags, "public_transport") == "platform") {
        return amenity_styles::kBusStop;
    }

    if (amenity == "shelter")          return amenity_styles::kShelter;
    if (amenity == "post_box")         return amenity_styles::kPostBox;
    if (amenity == "telephone")        return amenity_styles::kTelephone;
    if (amenity == "drinking_water")   return amenity_styles::kDrinkingWater;
    if (amenity == "bicycle_parking")  return amenity_styles::kBicycleParking;

    if (amenity == "vending_machine" &&
        tag_value(tags, "vending") == "parking_tickets") {
        return amenity_styles::kParkingMeter;
    }

    if (tag_value(tags, "emergency") == "fire_hydrant") return amenity_styles::kFireHydrant;
    if (tag_value(tags, "natural") == "tree")           return amenity_styles::kTree;

    if (amenity == "planter")  return amenity_styles::kPlanter;
    if (amenity == "fountain") return amenity_styles::kFountain;

    if (tag_value(tags, "leisure") == "picnic_table") return amenity_styles::kPicnicTable;

    const std::string man_made = tag_value(tags, "man_made");
    if (man_made == "street_cabinet") return amenity_styles::kStreetCabinet;
    if (tag_value(tags, "power") == "pole" || man_made == "utility_pole") {
        return amenity_styles::kUtilityPole;
    }

    return amenity_styles::kNone;
}

} // namespace stratum::osm::road
