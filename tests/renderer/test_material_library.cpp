/**
 * @file test_material_library.cpp
 * @brief The MaterialKey -> MaterialDef lookup, its fallback chain, and its JSON set files
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this suite needs no GPU
 *
 * MaterialLibrary resolves a (slot, variant) pair to PBR scalars, a sampler kind,
 * two pipeline flags and three texture HANDLES. Handles are integers; the
 * SDL_GPUTexture behind one is GPUTextureManager's business, not this class's.
 * So the whole resolution path -- the part that runs once per submesh range, per
 * draw, per frame -- is pure integer and float work over a hash map, and is
 * tested here with no device, no window and no SDL initialisation at all.
 *
 * The fixture attaches an UNINITIALISED GPUTextureManager. That is deliberate and
 * it is not a mock: GPUTextureManager's handle accessors (white(), flat_normal(),
 * default_orm(), missing_texture()) are plain member reads that answer
 * kInvalidTexture before init(), and MaterialLibrary is not allowed to care --
 * every unbound map resolves to a fallback at BIND time, inside
 * GPUTextureManager::bind_texture(), not here. A library that crashed or refused
 * without a live device would be a library that cannot be tested, and would also
 * be one that could not be populated before the device exists, which is when the
 * editor wants to populate it.
 *
 * ### What this suite does NOT cover, and why
 *
 * - install_procedural_textures() creates real GPU textures and needs a device.
 *   Its inputs -- the generators -- are covered by the ProceduralTexture suite;
 *   its output -- handles landing in the right material fields -- is not covered.
 * - There is no fallback COUNTER on the frozen MaterialLibrary interface, so the
 *   "unknown variant increments a counter" behaviour cannot be asserted. The
 *   fallback RESULT is asserted instead, which is the part that affects a frame.
 *
 * Run this suite with:
 * @code
 *     ./stratum_gpu_tests MaterialLibrary
 * @endcode
 */

#include "framework.hpp"

#include "renderer/material_library.hpp"
#include "renderer/mesh.hpp"
#include "renderer/texture.hpp"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

using stratum::MaterialDef;
using stratum::MaterialId;
using stratum::MaterialKey;
using stratum::MaterialLibrary;
using stratum::MaterialUniforms;
using stratum::GPUTextureManager;
using stratum::SamplerKind;

/// Every real slot, excluding the Count sentinel.
constexpr uint8_t kSlotCount = static_cast<uint8_t>(MaterialId::Count);

/**
 * @brief A library with a texture manager attached and no device anywhere
 *
 * See the file header: the manager is intentionally never init()ed. It exists so
 * MaterialLibrary::init() has the non-null pointer it demands, and so any
 * handle the library asks it for comes back as kInvalidTexture.
 */
struct Fixture {
    GPUTextureManager textures;
    MaterialLibrary lib;

    Fixture() { lib.init(&textures); }
};

[[nodiscard]] MaterialKey key(MaterialId slot, uint16_t variant) {
    return MaterialKey{slot, variant};
}

/// A material whose every field differs from every default, so a field that
/// fails to round-trip cannot coincidentally match.
[[nodiscard]] MaterialDef distinctive(const char* name) {
    MaterialDef def;
    def.name = name;
    def.base_color = glm::vec4(0.123f, 0.456f, 0.789f, 0.5f);
    def.metallic = 0.25f;
    def.roughness = 0.375f;
    def.ao = 0.625f;
    def.emissive = 0.125f;
    def.uv_scale = glm::vec2(2.5f, 4.0f);
    def.sampler = SamplerKind::RepeatPoint;
    def.alpha_blend = false;
    def.depth_bias = 0.0f;
    return def;
}

[[nodiscard]] bool same_def(const MaterialDef& a, const MaterialDef& b) {
    return a.name == b.name && a.base_color == b.base_color && a.metallic == b.metallic &&
           a.roughness == b.roughness && a.ao == b.ao && a.emissive == b.emissive &&
           a.uv_scale == b.uv_scale && a.albedo == b.albedo && a.normal == b.normal &&
           a.orm == b.orm && a.sampler == b.sampler && a.alpha_blend == b.alpha_blend &&
           a.depth_bias == b.depth_bias;
}

/// Field-by-field comparison that names the field that differs.
void check_same_def(const char* what, const MaterialDef& a, const MaterialDef& b) {
    const std::string prefix{what};
    if (a.name != b.name) {
        ::stratum::test::report_failure(__FILE__, __LINE__, "name", prefix + ": '" + a.name +
                                                                        "' vs '" + b.name + "'");
    }
    CHECK_EQ(a.base_color.x, b.base_color.x);
    CHECK_EQ(a.base_color.y, b.base_color.y);
    CHECK_EQ(a.base_color.z, b.base_color.z);
    CHECK_EQ(a.base_color.w, b.base_color.w);
    CHECK_EQ(a.metallic, b.metallic);
    CHECK_EQ(a.roughness, b.roughness);
    CHECK_EQ(a.ao, b.ao);
    CHECK_EQ(a.emissive, b.emissive);
    CHECK_EQ(a.uv_scale.x, b.uv_scale.x);
    CHECK_EQ(a.uv_scale.y, b.uv_scale.y);
    CHECK_EQ(a.albedo, b.albedo);
    CHECK_EQ(a.normal, b.normal);
    CHECK_EQ(a.orm, b.orm);
    CHECK_EQ(static_cast<int>(a.sampler), static_cast<int>(b.sampler));
    CHECK_EQ(a.alpha_blend, b.alpha_blend);
    CHECK_EQ(a.depth_bias, b.depth_bias);
}

/// Build-tree scratch directory; never the source tree. Created on demand.
[[nodiscard]] std::filesystem::path scratch(const char* filename) {
    std::filesystem::path dir{STRATUM_TEST_TMP_DIR};
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return dir / filename;
}

} // namespace

// ============================================================================
// The fallback chain
// ============================================================================

/// Link 1: an exact (slot, variant) entry wins over everything below it.
TEST(MaterialLibrary, resolve_returns_the_exact_entry_when_one_exists) {
    Fixture f;
    f.lib.load_defaults();

    MaterialDef def = distinctive("Asphalt/cobblestone");
    f.lib.set(key(MaterialId::Asphalt, 7), def);

    const MaterialDef& got = f.lib.resolve(key(MaterialId::Asphalt, 7));
    check_same_def("exact hit", got, def);

    // And it did not overwrite the slot default it sits beside.
    const MaterialDef& base = f.lib.resolve(key(MaterialId::Asphalt, 0));
    CHECK_FALSE(same_def(base, def));
}

/**
 * @brief Link 2: an unknown VARIANT resolves to variant 0 of the same slot
 *
 * The link that matters. Variants come from whatever surface= and kerb= tags an
 * extract happens to carry; this library has never seen most of them and never
 * will. An unrecognised asphalt variant is still asphalt.
 */
TEST(MaterialLibrary, unknown_variant_falls_back_to_the_slot_default) {
    Fixture f;
    f.lib.load_defaults();

    const MaterialDef& slot_default = f.lib.resolve(key(MaterialId::Asphalt, 0));
    const MaterialDef expected = slot_default;   // copy before any rehash

    for (const uint16_t variant : {uint16_t{1}, uint16_t{9}, uint16_t{4321}, uint16_t{65535}}) {
        CHECK_FALSE(f.lib.has(key(MaterialId::Asphalt, variant)));
        check_same_def("unknown asphalt variant", f.lib.resolve(key(MaterialId::Asphalt, variant)),
                       expected);
    }

    // The same holds for a slot with structurally different defaults, which is
    // where a fallback that dropped to Default rather than to the slot would show.
    const MaterialDef markings = f.lib.resolve(key(MaterialId::Markings, 0));
    check_same_def("unknown markings variant", f.lib.resolve(key(MaterialId::Markings, 900)),
                   markings);
    CHECK_TRUE(f.lib.resolve(key(MaterialId::Markings, 900)).alpha_blend);
}

/// Link 3: a slot with no entry at all resolves to Default, not to nothing.
TEST(MaterialLibrary, unknown_slot_falls_back_to_default) {
    Fixture f;

    MaterialDef fallback = distinctive("Default");
    f.lib.set(key(MaterialId::Default, 0), fallback);

    // Curb was never installed.
    CHECK_FALSE(f.lib.has(key(MaterialId::Curb, 0)));
    check_same_def("missing slot", f.lib.resolve(key(MaterialId::Curb, 0)), fallback);
    check_same_def("missing slot, odd variant", f.lib.resolve(key(MaterialId::Curb, 33)), fallback);

    // Out of range is a slot too: MaterialId::Count and beyond must not read past
    // the end of anything.
    const auto out_of_range = static_cast<MaterialId>(kSlotCount);
    check_same_def("Count sentinel", f.lib.resolve(key(out_of_range, 0)), fallback);
}

/// Link 4: a library that was never init()ed still answers, with the built-in.
TEST(MaterialLibrary, resolve_works_before_init_and_after_shutdown) {
    MaterialLibrary lib;   // no init(), no texture manager, no device

    const MaterialDef& before = lib.resolve(key(MaterialId::Asphalt, 0));
    CHECK_FALSE(before.name.empty());
    CHECK_EQ(lib.size(), size_t{0});

    GPUTextureManager textures;
    CHECK_TRUE(lib.init(&textures));
    lib.load_defaults();
    CHECK(lib.size() >= size_t{kSlotCount});

    lib.shutdown();
    CHECK_EQ(lib.size(), size_t{0});

    const MaterialDef& after = lib.resolve(key(MaterialId::Markings, 3));
    CHECK_FALSE(after.name.empty());
}

/// init(nullptr) is documented as refused. A library that accepted it would
/// resolve fine and then crash the first time it wanted a fallback handle.
TEST(MaterialLibrary, init_refuses_a_null_texture_manager) {
    MaterialLibrary lib;
    CHECK_FALSE(lib.init(nullptr));
}

/**
 * @brief resolve() never fails, over the whole key space a draw can present
 *
 * A submesh range carries a uint8 slot and a uint16 variant, both attacker-free
 * but both derived from file data. Every pair must produce a usable material, on
 * a populated library and on an empty one.
 */
TEST(MaterialLibrary, resolve_never_fails_over_a_full_slot_and_variant_sweep) {
    Fixture full;
    full.lib.load_defaults();
    Fixture empty;

    size_t checked = 0;
    for (uint8_t slot = 0; slot <= kSlotCount; ++slot) {
        for (uint16_t variant = 0; variant <= 64; ++variant) {
            const MaterialKey k = key(static_cast<MaterialId>(slot), variant);

            const MaterialDef& a = full.lib.resolve(k);
            const MaterialDef& b = empty.lib.resolve(k);

            // "Usable" means a real material, not a zeroed one: a name to log, a
            // sampler kind that maps to a real sampler, and finite scalars.
            CHECK_FALSE(a.name.empty());
            CHECK_FALSE(b.name.empty());
            CHECK(static_cast<uint8_t>(a.sampler) < static_cast<uint8_t>(SamplerKind::Count));
            CHECK(static_cast<uint8_t>(b.sampler) < static_cast<uint8_t>(SamplerKind::Count));
            CHECK(a.roughness >= 0.0f && a.roughness <= 1.0f);
            CHECK(a.metallic >= 0.0f && a.metallic <= 1.0f);
            CHECK(a.ao >= 0.0f && a.ao <= 1.0f);

            // uniforms_for() shares the chain and must not fail either.
            const MaterialUniforms u = full.lib.uniforms_for(k);
            CHECK(u.uv_params.x > 0.0f);
            CHECK(u.uv_params.y > 0.0f);

            ++checked;
        }
    }
    CHECK_EQ(checked, size_t{(kSlotCount + 1) * 65});
}

/// has() reports EXACT entries only. Conflating it with resolve() would make
/// every key look present and make the fallback chain untestable.
TEST(MaterialLibrary, has_reports_exact_entries_only) {
    Fixture f;
    f.lib.load_defaults();

    CHECK_TRUE(f.lib.has(key(MaterialId::Asphalt, 0)));
    CHECK_FALSE(f.lib.has(key(MaterialId::Asphalt, 12)));

    f.lib.set(key(MaterialId::Asphalt, 12), distinctive("Asphalt/sett"));
    CHECK_TRUE(f.lib.has(key(MaterialId::Asphalt, 12)));
}

// ============================================================================
// The default set
// ============================================================================

/**
 * @brief Every MaterialId slot gets a definition
 *
 * A slot with no default is an untextured, unlit-looking surface at run time --
 * geometry the road builder emitted and the renderer silently drew as the
 * Default grey. Iterating the enum rather than listing the slots means a slot
 * ADDED to mesh.hpp by the concurrent branch fails here instead of shipping.
 */
TEST(MaterialLibrary, load_defaults_covers_every_material_slot) {
    Fixture f;
    f.lib.load_defaults();

    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        const auto id = static_cast<MaterialId>(slot);
        const MaterialKey k = key(id, 0);

        if (!f.lib.has(k)) {
            ::stratum::test::report_failure(
                __FILE__, __LINE__, "load_defaults() defines every slot",
                std::string{"no default for MaterialId::"} + stratum::material_id_name(id));
            continue;
        }

        const MaterialDef& def = f.lib.resolve(k);
        CHECK_FALSE(def.name.empty());
        CHECK(def.roughness > 0.0f && def.roughness <= 1.0f);
        CHECK_NEAR(def.metallic, 0.0, 1e-6);   // nothing on a road is metal
        CHECK_NEAR(def.ao, 1.0, 1e-6);         // nothing here has baked occlusion
        CHECK_NEAR(def.emissive, 0.0, 1e-6);
    }

    CHECK(f.lib.size() >= size_t{kSlotCount});
}

/// The frozen table from material_library.hpp. These values are what "asphalt"
/// and "kerb" mean in this project; changing one changes every screenshot, so it
/// should have to be changed here too.
TEST(MaterialLibrary, load_defaults_installs_the_frozen_colour_table) {
    Fixture f;
    f.lib.load_defaults();

    struct Row {
        MaterialId slot;
        float r, g, b;
        float roughness;
    };
    const Row rows[] = {
        {MaterialId::Default,    0.60f, 0.60f, 0.60f, 0.85f},
        {MaterialId::Asphalt,    0.11f, 0.11f, 0.12f, 0.82f},
        {MaterialId::Concrete,   0.52f, 0.51f, 0.49f, 0.78f},
        {MaterialId::Curb,       0.62f, 0.61f, 0.58f, 0.70f},
        {MaterialId::Sidewalk,   0.55f, 0.54f, 0.52f, 0.80f},
        {MaterialId::Markings,   0.95f, 0.95f, 0.93f, 0.55f},
        {MaterialId::Gravel,     0.45f, 0.42f, 0.38f, 0.95f},
        {MaterialId::Dirt,       0.36f, 0.28f, 0.20f, 0.97f},
        {MaterialId::Grass,      0.20f, 0.33f, 0.14f, 0.90f},
        {MaterialId::BridgeDeck, 0.46f, 0.46f, 0.45f, 0.75f},
        {MaterialId::Parapet,    0.58f, 0.58f, 0.57f, 0.65f},
    };

    for (const Row& row : rows) {
        const MaterialDef& def = f.lib.resolve(key(row.slot, 0));
        CHECK_NEAR(def.base_color.x, row.r, 1e-5);
        CHECK_NEAR(def.base_color.y, row.g, 1e-5);
        CHECK_NEAR(def.base_color.z, row.b, 1e-5);
        CHECK_NEAR(def.base_color.w, 1.0, 1e-5);
        CHECK_NEAR(def.roughness, row.roughness, 1e-5);
    }

    // Asphalt must actually be darker than every pale surface, which is the
    // thing a transcription error in the table above would break visibly.
    const float asphalt = f.lib.resolve(key(MaterialId::Asphalt, 0)).base_color.y;
    CHECK(asphalt < f.lib.resolve(key(MaterialId::Concrete, 0)).base_color.y);
    CHECK(asphalt < f.lib.resolve(key(MaterialId::Sidewalk, 0)).base_color.y);
    CHECK(asphalt < f.lib.resolve(key(MaterialId::Curb, 0)).base_color.y);
}

/**
 * @brief Markings is the one structurally different slot
 *
 * Paint is a coverage mask sitting a few millimetres above the carriageway. It
 * needs the decal pipeline (alpha blend plus depth bias) and the clamping
 * sampler, and it must NOT carry a uv_scale, because its geometry has atlas
 * sub-rect UVs and scaling a sub-rect samples the neighbouring sprite.
 */
TEST(MaterialLibrary, markings_default_asks_for_the_decal_pipeline) {
    Fixture f;
    f.lib.load_defaults();

    const MaterialDef& m = f.lib.resolve(key(MaterialId::Markings, 0));
    CHECK_TRUE(m.alpha_blend);
    CHECK_NEAR(m.depth_bias, MaterialLibrary::kMarkingDepthBias, 1e-6);
    CHECK(m.depth_bias < 0.0f);   // toward the camera, or it still z-fights
    CHECK_TRUE(m.needs_decal_pipeline());
    CHECK_EQ(static_cast<int>(m.sampler), static_cast<int>(SamplerKind::ClampLinear));
    CHECK_NEAR(m.uv_scale.x, 1.0, 1e-6);
    CHECK_NEAR(m.uv_scale.y, 1.0, 1e-6);

    // Every other slot is opaque, unbiased, and tiling.
    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        const auto id = static_cast<MaterialId>(slot);
        if (id == MaterialId::Markings) {
            continue;
        }
        const MaterialDef& def = f.lib.resolve(key(id, 0));
        CHECK_FALSE(def.alpha_blend);
        CHECK_NEAR(def.depth_bias, 0.0, 1e-6);
        CHECK_FALSE(def.needs_decal_pipeline());
        CHECK_EQ(static_cast<int>(def.sampler), static_cast<int>(SamplerKind::RepeatAniso));
    }
}

/// needs_decal_pipeline() is what GPURenderer branches on, so it must be true
/// for either reason on its own, not only for both together.
TEST(MaterialLibrary, needs_decal_pipeline_triggers_on_either_flag) {
    MaterialDef def;
    CHECK_FALSE(def.needs_decal_pipeline());

    def.alpha_blend = true;
    CHECK_TRUE(def.needs_decal_pipeline());

    def.alpha_blend = false;
    def.depth_bias = MaterialLibrary::kMarkingDepthBias;
    CHECK_TRUE(def.needs_decal_pipeline());
}

// ============================================================================
// uniforms_for()
// ============================================================================

/**
 * @brief The scalars land in the documented vector lanes
 *
 * pbr_params is (metallic, roughness, ao, emissive) and nothing else. Every value
 * used is distinct and none is 0 or 1, so a swapped pair cannot pass.
 */
TEST(MaterialLibrary, uniforms_for_packs_the_documented_lanes) {
    Fixture f;

    MaterialDef def;
    def.name = "probe";
    def.base_color = glm::vec4(0.1f, 0.2f, 0.3f, 0.4f);
    def.metallic = 0.11f;
    def.roughness = 0.22f;
    def.ao = 0.33f;
    def.emissive = 0.44f;
    def.uv_scale = glm::vec2(0.55f, 0.66f);
    f.lib.set(key(MaterialId::Concrete, 5), def);

    const MaterialUniforms u = f.lib.uniforms_for(key(MaterialId::Concrete, 5));

    CHECK_NEAR(u.base_color.x, 0.1, 1e-6);
    CHECK_NEAR(u.base_color.y, 0.2, 1e-6);
    CHECK_NEAR(u.base_color.z, 0.3, 1e-6);
    CHECK_NEAR(u.base_color.w, 0.4, 1e-6);

    CHECK_NEAR(u.pbr_params.x, 0.11, 1e-6);   // metallic
    CHECK_NEAR(u.pbr_params.y, 0.22, 1e-6);   // roughness
    CHECK_NEAR(u.pbr_params.z, 0.33, 1e-6);   // ao
    CHECK_NEAR(u.pbr_params.w, 0.44, 1e-6);   // emissive

    CHECK_NEAR(u.uv_params.x, 0.55, 1e-6);
    CHECK_NEAR(u.uv_params.y, 0.66, 1e-6);
    CHECK_NEAR(u.uv_params.z, 0.0, 1e-6);     // declared unused; must stay zero
    CHECK_NEAR(u.uv_params.w, 0.0, 1e-6);
}

/// The same twelve floats, checked at their byte offsets. offsetof pins the
/// members; this pins the ORDER of the lanes inside them, which is the half a
/// GLSL edit can break without touching the C++ struct at all.
TEST(MaterialLibrary, uniforms_for_writes_the_lanes_at_their_byte_offsets) {
    Fixture f;

    MaterialDef def;
    def.name = "probe";
    def.base_color = glm::vec4(1.0f, 2.0f, 3.0f, 4.0f);
    def.metallic = 5.0f;
    def.roughness = 6.0f;
    def.ao = 7.0f;
    def.emissive = 8.0f;
    def.uv_scale = glm::vec2(9.0f, 10.0f);
    f.lib.set(key(MaterialId::Gravel, 0), def);

    const MaterialUniforms u = f.lib.uniforms_for(key(MaterialId::Gravel, 0));
    const auto* bytes = reinterpret_cast<const unsigned char*>(&u);

    const float expected[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 0};
    for (size_t i = 0; i < 12; ++i) {
        float value = 0.0f;
        std::memcpy(&value, bytes + i * sizeof(float), sizeof(float));
        CHECK_EQ(value, expected[i]);
    }
}

/// uniforms_for() shares the fallback chain, so an unknown variant produces the
/// slot default's uniforms rather than a zeroed block that renders black.
TEST(MaterialLibrary, uniforms_for_follows_the_fallback_chain) {
    Fixture f;
    f.lib.load_defaults();

    const MaterialUniforms known = f.lib.uniforms_for(key(MaterialId::Grass, 0));
    const MaterialUniforms unknown = f.lib.uniforms_for(key(MaterialId::Grass, 512));

    CHECK_NEAR(unknown.base_color.x, known.base_color.x, 1e-6);
    CHECK_NEAR(unknown.base_color.y, known.base_color.y, 1e-6);
    CHECK_NEAR(unknown.base_color.z, known.base_color.z, 1e-6);
    CHECK_NEAR(unknown.pbr_params.y, known.pbr_params.y, 1e-6);
    CHECK(unknown.base_color.y > unknown.base_color.x);   // grass is green
}

// ============================================================================
// keys() and size()
// ============================================================================

/// keys() is documented as ascending packed order so a listing is diffable.
TEST(MaterialLibrary, keys_are_returned_in_ascending_packed_order) {
    Fixture f;
    f.lib.load_defaults();
    f.lib.set(key(MaterialId::Sidewalk, 9), distinctive("Sidewalk/tactile"));
    f.lib.set(key(MaterialId::Asphalt, 3), distinctive("Asphalt/sett"));
    f.lib.set(key(MaterialId::Asphalt, 1), distinctive("Asphalt/cobblestone"));

    const std::vector<MaterialKey> keys = f.lib.keys();
    CHECK_EQ(keys.size(), f.lib.size());
    CHECK(keys.size() >= size_t{kSlotCount + 3});

    for (size_t i = 1; i < keys.size(); ++i) {
        CHECK(keys[i - 1].packed() < keys[i].packed());
    }

    // packed() puts the slot in the high half, so the slot is the primary sort
    // key and variants within a slot are contiguous and ascending.
    const MaterialKey packed_probe = key(MaterialId::Asphalt, 1);
    const uint32_t expected_packed =
        (uint32_t{static_cast<uint8_t>(MaterialId::Asphalt)} << 16) | 1u;
    CHECK_EQ(packed_probe.packed(), expected_packed);

    for (const MaterialKey& k : keys) {
        CHECK_TRUE(f.lib.has(k));
    }
}

/// set() on an existing key replaces rather than accumulates.
TEST(MaterialLibrary, set_replaces_an_existing_key_without_growing) {
    Fixture f;
    f.lib.set(key(MaterialId::Dirt, 2), distinctive("first"));
    const size_t after_first = f.lib.size();

    MaterialDef second = distinctive("second");
    second.roughness = 0.9f;
    f.lib.set(key(MaterialId::Dirt, 2), second);

    CHECK_EQ(f.lib.size(), after_first);
    check_same_def("replaced", f.lib.resolve(key(MaterialId::Dirt, 2)), second);
}

// ============================================================================
// JSON set files
// ============================================================================

/**
 * @brief Save then load into a fresh library, and compare every field
 *
 * The fresh library gets NO defaults, so anything present after the load came out
 * of the file. Texture paths are not exercised: nothing here has a bound texture,
 * which is the documented state of a procedurally textured set on disk anyway --
 * generated maps are regenerated, not stored.
 */
TEST(MaterialLibrary, json_round_trip_preserves_every_field) {
    const std::filesystem::path path = scratch("round_trip.materials.json");

    Fixture writer;
    writer.lib.load_defaults();

    // A handful of variants alongside the defaults, covering every field that
    // has a non-default value to lose.
    MaterialDef sett = distinctive("Asphalt/sett");
    sett.sampler = SamplerKind::RepeatPoint;
    writer.lib.set(key(MaterialId::Asphalt, 4), sett);

    MaterialDef granite = distinctive("Curb/granite");
    granite.base_color = glm::vec4(0.71f, 0.70f, 0.68f, 1.0f);
    granite.uv_scale = glm::vec2(1.5f, 0.75f);
    granite.sampler = SamplerKind::RepeatAniso;
    writer.lib.set(key(MaterialId::Curb, 11), granite);

    MaterialDef decal = distinctive("Markings/temporary");
    decal.uv_scale = glm::vec2(1.0f, 1.0f);   // mandatory for Markings
    decal.alpha_blend = true;
    decal.depth_bias = MaterialLibrary::kMarkingDepthBias;
    decal.sampler = SamplerKind::ClampLinear;
    writer.lib.set(key(MaterialId::Markings, 2), decal);

    CHECK_TRUE(writer.lib.save_to_file(path));
    CHECK_TRUE(std::filesystem::exists(path));

    Fixture reader;
    CHECK_EQ(reader.lib.size(), size_t{0});
    CHECK_TRUE(reader.lib.load_from_file(path));

    const std::vector<MaterialKey> written = writer.lib.keys();
    const std::vector<MaterialKey> read = reader.lib.keys();
    CHECK_EQ(read.size(), written.size());

    for (const MaterialKey& k : written) {
        if (!reader.lib.has(k)) {
            ::stratum::test::report_failure(
                __FILE__, __LINE__, "every saved key reloads",
                std::string{"lost "} + stratum::material_id_name(k.material) + "/" +
                    std::to_string(k.variant));
            continue;
        }
        check_same_def("round trip", reader.lib.resolve(k), writer.lib.resolve(k));
    }

    std::error_code ec;
    std::filesystem::remove(path, ec);
}

/**
 * @brief A malformed file leaves the library exactly as it was
 *
 * Half-applying a broken set is the worst outcome: the editor shows some
 * materials from the old set and some from the new, and nothing says which.
 * Refusing outright is recoverable; a half-applied set is not diagnosable.
 */
TEST(MaterialLibrary, malformed_json_leaves_the_library_untouched) {
    const std::filesystem::path path = scratch("malformed.materials.json");
    {
        std::ofstream out(path, std::ios::binary);
        out << "{ \"version\": 1, \"materials\": [ { \"slot\": \"Asphalt\", ,,, ";
    }

    Fixture f;
    f.lib.load_defaults();
    f.lib.set(key(MaterialId::Grass, 6), distinctive("Grass/meadow"));

    const std::vector<MaterialKey> before = f.lib.keys();
    std::vector<MaterialDef> snapshot;
    snapshot.reserve(before.size());
    for (const MaterialKey& k : before) {
        snapshot.push_back(f.lib.resolve(k));
    }

    CHECK_FALSE(f.lib.load_from_file(path));

    const std::vector<MaterialKey> after = f.lib.keys();
    CHECK_EQ(after.size(), before.size());
    if (after.size() != before.size()) {
        std::error_code ec;
        std::filesystem::remove(path, ec);
        return;
    }
    for (size_t i = 0; i < before.size(); ++i) {
        CHECK_EQ(after[i].packed(), before[i].packed());
        check_same_def("untouched by a malformed load", f.lib.resolve(after[i]), snapshot[i]);
    }

    std::error_code ec;
    std::filesystem::remove(path, ec);
}

/// A file that is not there at all is the same class of failure and must behave
/// the same way, not throw out of a filesystem call.
TEST(MaterialLibrary, missing_file_leaves_the_library_untouched) {
    Fixture f;
    f.lib.load_defaults();
    const size_t before = f.lib.size();

    CHECK_FALSE(f.lib.load_from_file(scratch("this_file_does_not_exist.json")));
    CHECK_EQ(f.lib.size(), before);
}

/// Saving into a directory that does not exist yet must create it, since the
/// editor's "save material set" writes wherever the user points it.
TEST(MaterialLibrary, save_creates_missing_parent_directories) {
    const std::filesystem::path dir =
        std::filesystem::path{STRATUM_TEST_TMP_DIR} / "nested" / "deeper";
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);

    const std::filesystem::path path = dir / "set.json";

    Fixture f;
    f.lib.load_defaults();
    CHECK_TRUE(f.lib.save_to_file(path));
    CHECK_TRUE(std::filesystem::exists(path));

    std::filesystem::remove_all(std::filesystem::path{STRATUM_TEST_TMP_DIR} / "nested", ec);
}

// ============================================================================
// Free helpers
// ============================================================================

/// material_id_from_name() must be the exact inverse of material_id_name(), for
/// every slot, or a saved set silently loses whichever slot disagrees.
TEST(MaterialLibrary, material_id_name_round_trips_for_every_slot) {
    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        const auto id = static_cast<MaterialId>(slot);
        const char* name = stratum::material_id_name(id);

        MaterialId parsed = MaterialId::Count;
        const bool ok = stratum::material_id_from_name(name, parsed);
        CHECK_TRUE(ok);
        if (ok) {
            CHECK_EQ(static_cast<int>(parsed), static_cast<int>(id));
        }
    }

    // The sentinel and anything unrecognised are failures, and must leave the
    // out parameter alone so the caller's default survives.
    MaterialId untouched = MaterialId::Sidewalk;
    CHECK_FALSE(stratum::material_id_from_name("Count", untouched));
    CHECK_FALSE(stratum::material_id_from_name("Unknown", untouched));
    CHECK_FALSE(stratum::material_id_from_name("", untouched));
    CHECK_FALSE(stratum::material_id_from_name("asphalt", untouched));   // case-sensitive
    CHECK_EQ(static_cast<int>(untouched), static_cast<int>(MaterialId::Sidewalk));
}

/// The same contract for sampler kinds.
TEST(MaterialLibrary, sampler_kind_name_round_trips_for_every_kind) {
    for (uint8_t kind = 0; kind < static_cast<uint8_t>(SamplerKind::Count); ++kind) {
        const auto k = static_cast<SamplerKind>(kind);
        const char* name = stratum::sampler_kind_name(k);

        SamplerKind parsed = SamplerKind::Count;
        const bool ok = stratum::sampler_kind_from_name(name, parsed);
        CHECK_TRUE(ok);
        if (ok) {
            CHECK_EQ(static_cast<int>(parsed), static_cast<int>(k));
        }
    }

    SamplerKind untouched = SamplerKind::RepeatPoint;
    CHECK_FALSE(stratum::sampler_kind_from_name("Count", untouched));
    CHECK_FALSE(stratum::sampler_kind_from_name("Unknown", untouched));
    CHECK_FALSE(stratum::sampler_kind_from_name("repeataniso", untouched));
    CHECK_EQ(static_cast<int>(untouched), static_cast<int>(SamplerKind::RepeatPoint));
}

/**
 * @brief The tile sizes are the plan's frozen UV Convention table
 *
 * The geometry side already divides by these when it writes UVs, so nothing in
 * the renderer needs them to draw. They are duplicated in the renderer for the
 * editor's texel-density readout, and a duplicated constant that drifts is worse
 * than no constant at all -- hence this table, copied from
 * docs/plans/road_network_plan.md rather than from the C++.
 */
TEST(MaterialLibrary, material_tile_metres_matches_the_frozen_uv_convention) {
    struct Row {
        MaterialId slot;
        float u, v;
    };
    const Row rows[] = {
        {MaterialId::Asphalt,    8.0f, 8.0f},
        {MaterialId::Concrete,   4.0f, 4.0f},
        {MaterialId::Sidewalk,   2.0f, 2.0f},
        {MaterialId::Curb,       0.5f, 2.0f},
        {MaterialId::Gravel,     4.0f, 4.0f},
        {MaterialId::Dirt,       4.0f, 4.0f},
        {MaterialId::Grass,      4.0f, 4.0f},
        {MaterialId::BridgeDeck, 4.0f, 4.0f},   // follows Concrete
        {MaterialId::Parapet,    4.0f, 4.0f},   // follows Concrete
    };

    for (const Row& row : rows) {
        const glm::vec2 tile = stratum::material_tile_metres(row.slot);
        CHECK_NEAR(tile.x, row.u, 1e-6);
        CHECK_NEAR(tile.y, row.v, 1e-6);
    }

    // Markings is atlased and has no tile size at all. Reporting anything
    // non-zero would invite someone to divide by it.
    const glm::vec2 markings = stratum::material_tile_metres(MaterialId::Markings);
    CHECK_NEAR(markings.x, 0.0, 1e-9);
    CHECK_NEAR(markings.y, 0.0, 1e-9);

    const glm::vec2 sentinel = stratum::material_tile_metres(MaterialId::Count);
    CHECK_NEAR(sentinel.x, 0.0, 1e-9);
    CHECK_NEAR(sentinel.y, 0.0, 1e-9);

    // The curb is the asymmetric one, and the asymmetry is the whole point: U
    // runs UP a 0.5 m face while V runs 2 m along the kerb line.
    const glm::vec2 curb = stratum::material_tile_metres(MaterialId::Curb);
    CHECK(curb.x < curb.y);
}

// ============================================================================
// Fallback accounting
// ============================================================================

/**
 * @brief Each link of the chain is attributed to itself
 *
 * The counters exist because falling back is both CORRECT and INVISIBLE: a whole
 * variant set going missing draws every affected surface as its slot default,
 * which looks like a perfectly ordinary road. Nothing warns, nothing is magenta,
 * and nothing is wrong on screen. The count is the only signal, so a counter
 * charged to the wrong link is worse than no counter -- it reads as healthy.
 */
TEST(MaterialLibrary, resolve_stats_attribute_each_link_of_the_chain) {
    // Link 1 and link 2, on a fully populated library.
    Fixture full;
    full.lib.load_defaults();
    full.lib.reset_resolve_stats();

    (void)full.lib.resolve(key(MaterialId::Asphalt, 0));
    {
        const auto& s = full.lib.resolve_stats();
        CHECK_EQ(s.resolves, uint64_t{1});
        CHECK_EQ(s.exact, uint64_t{1});
        CHECK_EQ(s.variant_fallbacks, uint64_t{0});
        CHECK_EQ(s.fallback_keys, size_t{0});
    }

    (void)full.lib.resolve(key(MaterialId::Asphalt, 777));
    {
        const auto& s = full.lib.resolve_stats();
        CHECK_EQ(s.resolves, uint64_t{2});
        CHECK_EQ(s.exact, uint64_t{1});
        CHECK_EQ(s.variant_fallbacks, uint64_t{1});
        CHECK_EQ(s.slot_fallbacks, uint64_t{0});
        CHECK_EQ(s.hard_fallbacks, uint64_t{0});
        CHECK_EQ(s.fallback_keys, size_t{1});
    }

    // Link 3, on a library that has a Default and nothing else.
    Fixture bare;
    bare.lib.set(key(MaterialId::Default, 0), distinctive("Default"));
    bare.lib.reset_resolve_stats();
    (void)bare.lib.resolve(key(MaterialId::Parapet, 0));
    {
        const auto& s = bare.lib.resolve_stats();
        CHECK_EQ(s.slot_fallbacks, uint64_t{1});
        CHECK_EQ(s.variant_fallbacks, uint64_t{0});
        CHECK_EQ(s.hard_fallbacks, uint64_t{0});
        CHECK_EQ(s.fallback_keys, size_t{1});
    }

    // Link 4, on a library with nothing at all.
    Fixture empty;
    empty.lib.reset_resolve_stats();
    (void)empty.lib.resolve(key(MaterialId::Grass, 4));
    {
        const auto& s = empty.lib.resolve_stats();
        CHECK_EQ(s.hard_fallbacks, uint64_t{1});
        CHECK_EQ(s.exact, uint64_t{0});
        CHECK_EQ(s.fallback_keys, size_t{1});
    }
}

/// The four links partition the calls: every resolve is charged exactly once.
TEST(MaterialLibrary, every_resolve_is_charged_to_exactly_one_link) {
    Fixture f;
    f.lib.load_defaults();
    f.lib.reset_resolve_stats();

    uint64_t calls = 0;
    for (uint8_t slot = 0; slot <= kSlotCount; ++slot) {
        for (uint16_t variant = 0; variant <= 8; ++variant) {
            (void)f.lib.resolve(key(static_cast<MaterialId>(slot), variant));
            ++calls;
        }
    }

    const auto& s = f.lib.resolve_stats();
    CHECK_EQ(s.resolves, calls);
    CHECK_EQ(s.exact + s.variant_fallbacks + s.slot_fallbacks + s.hard_fallbacks, calls);
}

/// uniforms_for() is a resolve too, and is what the render loop actually calls.
/// A counter that only saw resolve() would report zero on a running editor.
TEST(MaterialLibrary, uniforms_for_is_counted_like_a_resolve) {
    Fixture f;
    f.lib.load_defaults();
    f.lib.reset_resolve_stats();

    (void)f.lib.uniforms_for(key(MaterialId::Concrete, 0));
    (void)f.lib.uniforms_for(key(MaterialId::Concrete, 99));

    const auto& s = f.lib.resolve_stats();
    CHECK_EQ(s.resolves, uint64_t{2});
    CHECK_EQ(s.exact, uint64_t{1});
    CHECK_EQ(s.variant_fallbacks, uint64_t{1});
    CHECK_EQ(s.fallback_keys, size_t{1});
}

/**
 * @brief fallback_keys counts DISTINCT keys, not calls
 *
 * The distinction is the whole value of the number. Call counts scale with frame
 * rate and mesh count and mean nothing; "12 distinct materials are missing" is
 * the sentence a user can act on.
 */
TEST(MaterialLibrary, fallback_keys_are_distinct_and_ascending) {
    Fixture f;
    f.lib.load_defaults();
    f.lib.reset_resolve_stats();

    // The same missing key, many times over, is one missing material.
    for (int i = 0; i < 50; ++i) {
        (void)f.lib.resolve(key(MaterialId::Sidewalk, 21));
    }
    CHECK_EQ(f.lib.resolve_stats().variant_fallbacks, uint64_t{50});
    CHECK_EQ(f.lib.resolve_stats().fallback_keys, size_t{1});

    (void)f.lib.resolve(key(MaterialId::Asphalt, 5));
    (void)f.lib.resolve(key(MaterialId::Asphalt, 2));
    (void)f.lib.resolve(key(MaterialId::Grass, 1));

    const std::vector<MaterialKey> missing = f.lib.fallback_keys();
    CHECK_EQ(missing.size(), f.lib.resolve_stats().fallback_keys);
    CHECK_EQ(missing.size(), size_t{4});

    for (size_t i = 1; i < missing.size(); ++i) {
        CHECK(missing[i - 1].packed() < missing[i].packed());
    }
    for (const MaterialKey& k : missing) {
        CHECK_FALSE(f.lib.has(k));
    }
}

/// Resetting forgets the accounting and nothing else, so the counts after a set
/// is installed describe that set rather than the one before it.
TEST(MaterialLibrary, reset_resolve_stats_clears_counts_but_not_materials) {
    Fixture f;
    f.lib.load_defaults();

    (void)f.lib.resolve(key(MaterialId::Dirt, 44));
    CHECK(f.lib.resolve_stats().fallback_keys > size_t{0});

    const size_t materials = f.lib.size();
    f.lib.reset_resolve_stats();

    const auto& s = f.lib.resolve_stats();
    CHECK_EQ(s.resolves, uint64_t{0});
    CHECK_EQ(s.exact, uint64_t{0});
    CHECK_EQ(s.variant_fallbacks, uint64_t{0});
    CHECK_EQ(s.slot_fallbacks, uint64_t{0});
    CHECK_EQ(s.hard_fallbacks, uint64_t{0});
    CHECK_EQ(s.fallback_keys, size_t{0});
    CHECK_TRUE(f.lib.fallback_keys().empty());

    CHECK_EQ(f.lib.size(), materials);
    CHECK_TRUE(f.lib.has(key(MaterialId::Dirt, 0)));
}

/**
 * @brief Installing the missing variant stops the fallbacks
 *
 * The loop closing: the count is only actionable if acting on it changes the
 * count. A stale distinct-key set would keep reporting a material that is now
 * present, which is the same disease as not counting at all.
 */
TEST(MaterialLibrary, installing_a_missing_variant_stops_the_fallback) {
    Fixture f;
    f.lib.load_defaults();

    (void)f.lib.resolve(key(MaterialId::Asphalt, 6));
    CHECK_EQ(f.lib.resolve_stats().variant_fallbacks, uint64_t{1});

    f.lib.set(key(MaterialId::Asphalt, 6), distinctive("Asphalt/cobblestone"));
    f.lib.reset_resolve_stats();

    (void)f.lib.resolve(key(MaterialId::Asphalt, 6));
    const auto& s = f.lib.resolve_stats();
    CHECK_EQ(s.exact, uint64_t{1});
    CHECK_EQ(s.variant_fallbacks, uint64_t{0});
    CHECK_EQ(s.fallback_keys, size_t{0});
}

// ============================================================================
// The documented JSON schema, read rather than written
// ============================================================================

/**
 * @brief A hand-written set in the documented schema loads
 *
 * The round-trip test only proves the reader accepts the writer's output, which
 * two halves of one bug agree on perfectly. This one is written by hand from the
 * schema block in material_library.hpp, so it fails if the documented format and
 * the implemented format have diverged -- which is what anyone authoring a set
 * file outside this program would hit.
 */
TEST(MaterialLibrary, a_hand_written_set_in_the_documented_schema_loads) {
    const std::filesystem::path path = scratch("hand_written.materials.json");
    {
        std::ofstream out(path, std::ios::binary);
        out << R"({
  "version": 1,
  "materials": [
    {
      "slot": "Asphalt",
      "variant": 0,
      "name": "Asphalt",
      "base_color": [0.11, 0.11, 0.12, 1.0],
      "metallic": 0.0,
      "roughness": 0.82,
      "ao": 1.0,
      "emissive": 0.0,
      "uv_scale": [1.0, 1.0],
      "sampler": "RepeatAniso",
      "alpha_blend": false,
      "depth_bias": 0.0
    },
    {
      "slot": "Markings",
      "variant": 0,
      "name": "Markings",
      "base_color": [0.95, 0.95, 0.93, 1.0],
      "metallic": 0.0,
      "roughness": 0.55,
      "ao": 1.0,
      "emissive": 0.0,
      "uv_scale": [1.0, 1.0],
      "sampler": "ClampLinear",
      "alpha_blend": true,
      "depth_bias": -2.0
    }
  ]
})";
    }

    Fixture f;
    CHECK_TRUE(f.lib.load_from_file(path));
    CHECK_EQ(f.lib.size(), size_t{2});

    const MaterialDef& asphalt = f.lib.resolve(key(MaterialId::Asphalt, 0));
    CHECK_EQ(asphalt.name, std::string{"Asphalt"});
    CHECK_NEAR(asphalt.base_color.z, 0.12, 1e-5);
    CHECK_NEAR(asphalt.roughness, 0.82, 1e-5);
    CHECK_EQ(static_cast<int>(asphalt.sampler), static_cast<int>(SamplerKind::RepeatAniso));
    CHECK_FALSE(asphalt.alpha_blend);

    const MaterialDef& markings = f.lib.resolve(key(MaterialId::Markings, 0));
    CHECK_TRUE(markings.alpha_blend);
    CHECK_NEAR(markings.depth_bias, MaterialLibrary::kMarkingDepthBias, 1e-5);
    CHECK_EQ(static_cast<int>(markings.sampler), static_cast<int>(SamplerKind::ClampLinear));

    std::error_code ec;
    std::filesystem::remove(path, ec);
}

/**
 * @brief A parseable file with one bad entry still installs the good ones
 *
 * Documented behaviour, and the right one: a set with a slot this build does not
 * know about -- because it was authored against the concurrent branch's material
 * list -- should lose that one entry and keep the other nine, not lose all ten.
 * The distinction from the malformed case is exact: JSON that PARSES is applied
 * entry by entry; JSON that does not parse is not applied at all.
 */
TEST(MaterialLibrary, a_parseable_set_with_a_bad_entry_installs_the_rest) {
    const std::filesystem::path path = scratch("partial.materials.json");
    {
        std::ofstream out(path, std::ios::binary);
        out << R"({
  "version": 1,
  "materials": [
    { "slot": "NotARealSlot", "variant": 0, "name": "Nonsense", "roughness": 0.5 },
    { "slot": "Gravel", "variant": 0, "name": "Gravel",
      "base_color": [0.45, 0.42, 0.38, 1.0], "roughness": 0.95 },
    { "slot": "Grass", "variant": 3, "name": "Grass/meadow",
      "base_color": [0.20, 0.33, 0.14, 1.0], "roughness": 0.90 }
  ]
})";
    }

    Fixture f;
    CHECK_TRUE(f.lib.load_from_file(path));

    CHECK_TRUE(f.lib.has(key(MaterialId::Gravel, 0)));
    CHECK_TRUE(f.lib.has(key(MaterialId::Grass, 3)));
    CHECK_EQ(f.lib.size(), size_t{2});
    CHECK_EQ(f.lib.resolve(key(MaterialId::Grass, 3)).name, std::string{"Grass/meadow"});

    std::error_code ec;
    std::filesystem::remove(path, ec);
}

/// The set files this program writes carry a version, so a future reader can
/// tell a v1 set from a v2 one instead of guessing from the shape.
TEST(MaterialLibrary, a_saved_set_carries_its_schema_version) {
    const std::filesystem::path path = scratch("versioned.materials.json");

    Fixture f;
    f.lib.load_defaults();
    CHECK_TRUE(f.lib.save_to_file(path));

    std::ifstream in(path, std::ios::binary);
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};

    CHECK(text.find("\"version\"") != std::string::npos);
    CHECK(text.find("\"materials\"") != std::string::npos);
    CHECK(text.find("\"slot\"") != std::string::npos);
    CHECK(text.find("\"variant\"") != std::string::npos);
    CHECK(text.find("\"base_color\"") != std::string::npos);
    CHECK(text.find("\"uv_scale\"") != std::string::npos);
    CHECK(text.find("\"sampler\"") != std::string::npos);

    std::error_code ec;
    std::filesystem::remove(path, ec);
}
