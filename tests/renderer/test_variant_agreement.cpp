/**
 * @file test_variant_agreement.cpp
 * @brief The renderer's variant numbers must equal the geometry side's
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this suite exists
 *
 * `SubMesh::variant` is written by the geometry side and read by the renderer, and
 * the two never see each other's headers. `osm/road/road_style.hpp` derives a
 * variant from OSM tags; `renderer/material_library.hpp` resolves it to textures
 * and PBR scalars. Neither includes the other, deliberately -- `MaterialKey` lives
 * in `renderer/mesh.hpp` precisely so that neither layer owns the vocabulary.
 *
 * The cost of that separation is a mirrored table, and a mirror can drift.
 *
 * Drift here is SILENT. It produces no crash, no validation error and no wrong-looking
 * geometry: a carriageway tagged `smoothness=bad` simply draws with the freshly-laid
 * material, which is a perfectly plausible road. Nobody files a bug against a road
 * that looks fine.
 *
 * This is not hypothetical. The two tables were authored on parallel branches and
 * disagreed on exactly that axis when they met: the geometry side had
 * `1 = worn, 2 = smooth`, the renderer had `1 = smooth, 2 = worn`. Every worn road
 * in the world would have rendered as new and every new one as worn, and the only
 * symptom would have been that the map looked slightly wrong in a way no one could
 * name. The renderer was re-keyed onto the geometry side's numbering, and this
 * suite is what stops it happening again.
 *
 * These are `static_assert`s in test clothing. They cost nothing at runtime and
 * they fail at COMPILE time if the two headers disagree, which is the earliest and
 * loudest place to catch it.
 */

#include "framework.hpp"

#include "osm/road/road_style.hpp"
#include "renderer/material_library.hpp"
#include "renderer/mesh.hpp"

namespace {

namespace geom = stratum::osm::road::variants;
namespace rend = stratum::material_variant;

// ============================================================================
// Compile-time agreement
// ============================================================================
//
// One static_assert per mirrored constant. The message names both sides so a
// failure says which header to edit rather than just which line broke.

#define AGREE(rend_name, geom_name)                                            \
    static_assert(rend::rend_name == geom::geom_name,                          \
                  "material_variant::" #rend_name " disagrees with "           \
                  "osm::road::variants::" #geom_name " -- the geometry side "  \
                  "is the authority; re-key the renderer, not the parser")

// --- MaterialId::Asphalt ----------------------------------------------------
AGREE(kAsphaltDefault,  kAsphaltDefault);
AGREE(kAsphaltWorn,     kAsphaltWorn);
AGREE(kAsphaltSmooth,   kAsphaltSmooth);
AGREE(kAsphaltCobble,   kCobblestone);
AGREE(kAsphaltSett,     kSett);
AGREE(kAsphaltPaving,   kPavingStones);
AGREE(kAsphaltChipseal, kChipseal);
AGREE(kAsphaltColoured, kAsphaltColoured);

// --- MaterialId::Concrete ---------------------------------------------------
AGREE(kConcreteDefault, kConcreteDefault);
AGREE(kConcreteWorn,    kConcreteWorn);
AGREE(kConcreteSmooth,  kConcreteSmooth);
AGREE(kConcretePlates,  kConcretePlates);
AGREE(kConcreteLanes,   kConcreteLanes);
AGREE(kConcreteIsland,  kConcreteIsland);

// --- MaterialId::Curb -------------------------------------------------------
AGREE(kCurbDefault,   kCurbDefault);
AGREE(kCurbGranite,   kCurbGranite);
AGREE(kCurbAsphalt,   kCurbAsphalt);
AGREE(kCurbDropped,   kCurbDropped);
AGREE(kCurbMountable, kCurbMountable);

// --- MaterialId::Sidewalk ---------------------------------------------------
AGREE(kSidewalkDefault,  kSidewalkDefault);
AGREE(kSidewalkPaved,    kSidewalkPaved);
AGREE(kSidewalkTactile,  kSidewalkTactile);
AGREE(kSidewalkAsphalt,  kSidewalkAsphalt);
AGREE(kSidewalkConcrete, kSidewalkConcrete);
AGREE(kSidewalkBrick,    kSidewalkBrick);

// --- MaterialId::Markings ---------------------------------------------------
AGREE(kMarkingsDefault, kMarkingsDefault);
AGREE(kMarkingsYellow,  kMarkingsYellow);
AGREE(kMarkingsWorn,    kMarkingsWorn);

// --- MaterialId::Gravel -----------------------------------------------------
AGREE(kGravelDefault,   kGravelDefault);
AGREE(kGravelCompacted, kGravelCompacted);
AGREE(kGravelFine,      kGravelFine);
AGREE(kGravelPebble,    kGravelPebble);
AGREE(kGravelUnpaved,   kGravelUnpaved);

// --- MaterialId::Dirt -------------------------------------------------------
AGREE(kDirtDefault, kDirtDefault);
AGREE(kDirtGround,  kDirtGround);
AGREE(kDirtEarth,   kDirtEarth);
AGREE(kDirtMud,     kDirtMud);
AGREE(kDirtSand,    kDirtSand);

// --- MaterialId::Grass ------------------------------------------------------
AGREE(kGrassDefault, kGrassDefault);
AGREE(kGrassMown,    kGrassMown);
AGREE(kGrassRough,   kGrassRough);
AGREE(kGrassPaver,   kGrassPaver);
AGREE(kGrassPlanted, kGrassPlanted);

// --- MaterialId::BridgeDeck -------------------------------------------------
AGREE(kBridgeDeckDefault, kBridgeDeckDefault);
AGREE(kBridgeDeckSteel,   kBridgeDeckSteel);
AGREE(kBridgeDeckWood,    kBridgeDeckWood);
AGREE(kBridgeDeckStone,   kBridgeDeckStone);

// --- MaterialId::Parapet ----------------------------------------------------
AGREE(kParapetDefault,  kParapetDefault);
AGREE(kParapetSteel,    kParapetSteel);
AGREE(kParapetStone,    kParapetStone);
AGREE(kParapetConcrete, kParapetConcrete);

// --- MaterialId::Wall -------------------------------------------------------
AGREE(kWallDefault,  kWallDefault);
AGREE(kWallBrick,    kWallBrick);
AGREE(kWallStone,    kWallStone);
AGREE(kWallConcrete, kWallConcrete);
AGREE(kWallRender,   kWallRender);
AGREE(kWallGlass,    kWallGlass);
AGREE(kWallMetal,    kWallMetal);
AGREE(kWallWood,     kWallWood);

// --- MaterialId::Roof -------------------------------------------------------
AGREE(kRoofDefault,  kRoofDefault);
AGREE(kRoofTile,     kRoofTile);
AGREE(kRoofSlate,    kRoofSlate);
AGREE(kRoofMetal,    kRoofMetal);
AGREE(kRoofMembrane, kRoofMembrane);
AGREE(kRoofConcrete, kRoofConcrete);
AGREE(kRoofThatch,   kRoofThatch);
AGREE(kRoofGlass,    kRoofGlass);

#undef AGREE

} // namespace

// ============================================================================
// Runtime coverage
// ============================================================================
//
// The static_asserts above prove the numbers agree. These prove the renderer
// actually HAS a material for what the geometry side can emit -- agreement on a
// number is worthless if nothing is installed under it.

TEST(VariantAgreement, every_key_the_geometry_side_emits_has_a_material) {
    using namespace stratum;

    MaterialLibrary lib;
    GPUTextureManager textures;   // uninitialised on purpose; handles are integers
    lib.init(&textures);

    // BOTH halves of the startup path. load_defaults() is deliberately the frozen
    // slot table -- it excludes variants so that the fallback-chain suite always has
    // a variant number that is reliably absent to assert against, which is the most
    // important behaviour in the file. Variants live in load_variant_defaults(),
    // which install_procedural_textures() calls for real at startup.
    //
    // So this asserts what actually reaches a running editor, not what either half
    // installs alone. Calling only load_defaults() here would fail for every variant
    // in the table and would be asserting a contract nobody makes.
    lib.load_defaults();
    lib.load_variant_defaults();

    const auto keys = osm::road::all_material_keys();
    CHECK_TRUE(!keys.empty());

    size_t missing = 0;
    for (const MaterialKey& k : keys) {
        if (!lib.has(k)) {
            ++missing;
            ::stratum::test::report_failure(
                __FILE__, __LINE__,
                "the geometry side can emit a key the renderer never installed",
                std::string{material_id_name(k.material)} + " variant "
                    + std::to_string(k.variant)
                    + " -- it would silently draw as its slot default");
        }
    }
    CHECK_EQ(missing, size_t{0});
}

TEST(VariantAgreement, every_slot_has_a_default_including_the_building_slots) {
    using namespace stratum;

    MaterialLibrary lib;
    GPUTextureManager textures;
    lib.init(&textures);
    lib.load_defaults();

    // Wall and Roof were added to MaterialId for building_wall_material() and
    // building_roof_material(). Nothing emits them yet, which is exactly why a
    // missing default here would go unnoticed until the day something does.
    for (uint8_t slot = 0; slot < static_cast<uint8_t>(MaterialId::Count); ++slot) {
        const auto id = static_cast<MaterialId>(slot);
        if (!lib.has(MaterialKey{ id, 0 })) {
            ::stratum::test::report_failure(
                __FILE__, __LINE__, "load_defaults() must define every slot",
                std::string{"no default for MaterialId::"} + material_id_name(id));
        }
    }
    CHECK_TRUE(lib.has(MaterialKey{ MaterialId::Wall, 0 }));
    CHECK_TRUE(lib.has(MaterialKey{ MaterialId::Roof, 0 }));
}
