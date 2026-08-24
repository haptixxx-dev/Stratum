/**
 * @file material_library.cpp
 * @brief MaterialKey -> MaterialDef resolution, the built-in table, and set I/O
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * See material_library.hpp for the shader contract, the fallback chain and the
 * variant numbering. This file is the implementation and nothing more: it adds no
 * policy the header does not already state, on purpose, because the header is the
 * document two branches are reading.
 */

#include "renderer/material_library.hpp"

#include "osm/road/marking_atlas.hpp"
#include "renderer/procedural_texture.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <system_error>
#include <utility>

namespace stratum {

namespace {

/// Schema version written by save_to_file() and required by load_from_file().
constexpr uint32_t kSetFileVersion = 1;

/**
 * @brief Derive an independent seed from the master seed and a stream index
 *
 * Every generator gets its own field. Reusing the master seed across generators
 * makes asphalt and gravel share a noise field, which is invisible in isolation
 * and glaringly obvious the moment a gravel verge runs beside a road: the two
 * surfaces mottle in lockstep.
 */
[[nodiscard]] uint32_t derive_seed(uint32_t master, uint32_t stream) {
    uint32_t h = master ^ (stream * 0x9E3779B9u);
    h ^= h >> 16;
    h *= 0x7FEB352Du;
    h ^= h >> 15;
    h *= 0x846CA68Bu;
    h ^= h >> 16;
    return h;
}

/// Whether @p v is a power of two and at least @p minimum.
[[nodiscard]] bool is_valid_texture_size(uint32_t v, uint32_t minimum) {
    return v >= minimum && (v & (v - 1u)) == 0u;
}

/// Full mip chain length for a square texture of side @p size.
[[nodiscard]] uint32_t mip_count_for(uint32_t size) {
    uint32_t levels = 1;
    while (size > 1u) {
        size >>= 1u;
        ++levels;
    }
    return levels;
}

/// Read a fixed-length float array from a JSON object, leaving @p out untouched
/// on any failure. Returns false so the caller can log which field was dropped.
[[nodiscard]] bool read_floats(const nlohmann::json& j, const char* key, float* out, size_t n) {
    const auto it = j.find(key);
    if (it == j.end() || !it->is_array() || it->size() != n) {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        const auto& e = (*it)[i];
        if (!e.is_number()) {
            return false;
        }
        out[i] = e.get<float>();
    }
    return true;
}

/// Read a scalar float, leaving @p out untouched when the field is absent or not
/// a number.
void read_float(const nlohmann::json& j, const char* key, float& out) {
    const auto it = j.find(key);
    if (it != j.end() && it->is_number()) {
        out = it->get<float>();
    }
}

/// Read a bool, leaving @p out untouched when the field is absent or not a bool.
void read_bool(const nlohmann::json& j, const char* key, bool& out) {
    const auto it = j.find(key);
    if (it != j.end() && it->is_boolean()) {
        out = it->get<bool>();
    }
}

/// Read a string field, or an empty string when absent.
[[nodiscard]] std::string read_string(const nlohmann::json& j, const char* key) {
    const auto it = j.find(key);
    if (it != j.end() && it->is_string()) {
        return it->get<std::string>();
    }
    return {};
}

/**
 * @brief Express @p target relative to @p base, falling back to the absolute path
 *
 * std::filesystem::relative() fails across drives and on some malformed inputs,
 * and an absolute path in the set file is far better than an empty one: it still
 * loads, it just does not move with the set.
 */
[[nodiscard]] std::string relative_or_absolute(const std::filesystem::path& target,
                                               const std::filesystem::path& base) {
    std::error_code ec;
    const std::filesystem::path rel = std::filesystem::relative(target, base, ec);
    if (ec || rel.empty()) {
        return target.generic_string();
    }
    return rel.generic_string();
}

} // namespace

// ============================================================================
// Free helpers
// ============================================================================

bool material_id_from_name(std::string_view name, MaterialId& out) {
    for (uint8_t i = 0; i < static_cast<uint8_t>(MaterialId::Count); ++i) {
        const auto id = static_cast<MaterialId>(i);
        if (name == material_id_name(id)) {
            out = id;
            return true;
        }
    }
    return false;
}

bool sampler_kind_from_name(std::string_view name, SamplerKind& out) {
    for (uint8_t i = 0; i < static_cast<uint8_t>(SamplerKind::Count); ++i) {
        const auto kind = static_cast<SamplerKind>(i);
        if (name == sampler_kind_name(kind)) {
            out = kind;
            return true;
        }
    }
    return false;
}

glm::vec2 material_tile_metres(MaterialId material) {
    switch (material) {
        // Not in the plan's table. Untagged geometry gets Concrete's 4 m, which is
        // the middle of the range and therefore the least wrong default.
        case MaterialId::Default:    return { 4.0f, 4.0f };
        case MaterialId::Asphalt:    return { 8.0f, 8.0f };
        case MaterialId::Concrete:   return { 4.0f, 4.0f };
        case MaterialId::Curb:       return { 0.5f, 2.0f };   // U runs up the face
        case MaterialId::Sidewalk:   return { 2.0f, 2.0f };
        case MaterialId::Markings:   return { 0.0f, 0.0f };   // atlased; no tile size
        case MaterialId::Gravel:     return { 4.0f, 4.0f };
        case MaterialId::Dirt:       return { 4.0f, 4.0f };
        case MaterialId::Grass:      return { 4.0f, 4.0f };
        case MaterialId::BridgeDeck: return { 4.0f, 4.0f };   // follows Concrete
        case MaterialId::Parapet:    return { 4.0f, 4.0f };   // follows Concrete
        case MaterialId::Count:      break;
    }
    return { 0.0f, 0.0f };
}

// ============================================================================
// Lifetime
// ============================================================================

bool MaterialLibrary::init(GPUTextureManager* textures) {
    if (textures == nullptr) {
        spdlog::error("MaterialLibrary::init called with a null GPUTextureManager");
        return false;
    }

    m_textures = textures;
    m_materials.clear();
    m_texture_paths.clear();
    reset_resolve_stats();

    // Link 4 of the fallback chain. Bound to the manager's own fallbacks rather
    // than to kInvalidTexture: kInvalidTexture resolves to the MAGENTA CHECKER,
    // which is right for "this material is missing" but wrong for "this material
    // exists and simply has no albedo map".
    m_fallback = MaterialDef{};
    m_fallback.name = "<fallback>";
    m_fallback.base_color = glm::vec4(0.6f, 0.6f, 0.6f, 1.0f);
    m_fallback.roughness = 0.85f;
    m_fallback.metallic = 0.0f;
    m_fallback.albedo = m_textures->white();
    m_fallback.normal = m_textures->flat_normal();
    m_fallback.orm = m_textures->default_orm();
    m_fallback.sampler = SamplerKind::RepeatAniso;

    return true;
}

void MaterialLibrary::shutdown() {
    // Textures are deliberately NOT released here; GPUTextureManager owns them and
    // a handle may be shared with another library or with an editor panel. See the
    // declaration.
    m_materials.clear();
    m_texture_paths.clear();
    m_textures = nullptr;
    reset_resolve_stats();
}

// ============================================================================
// Resolution
// ============================================================================

const MaterialDef& MaterialLibrary::resolve(MaterialKey key) const {
    ++m_resolve_stats.resolves;

    // Link 1: the exact (slot, variant).
    if (const auto it = m_materials.find(key.packed()); it != m_materials.end()) {
        ++m_resolve_stats.exact;
        return it->second;
    }

    // Everything past here is a fallback. Record the key once so the editor can
    // report "N materials resolved by fallback" and so the log does not repeat the
    // same warning once per draw per frame.
    const auto [slot_it, first_seen] = m_fallback_keys.try_emplace(key.packed(), 0u);
    ++slot_it->second;
    if (first_seen) {
        m_resolve_stats.fallback_keys = m_fallback_keys.size();
        spdlog::warn("MaterialLibrary: no material for {}/variant {}; falling back",
                     material_id_name(key.material), key.variant);
    }

    // Link 2: the slot's default variant. This is the link that matters. The
    // geometry side derives variants from whatever tags an extract carries, so an
    // unknown variant of Asphalt is routine, is still asphalt, and must draw as
    // asphalt rather than as an error colour.
    if (key.variant != 0) {
        const MaterialKey slot_default{ key.material, 0 };
        if (const auto it = m_materials.find(slot_default.packed()); it != m_materials.end()) {
            ++m_resolve_stats.variant_fallbacks;
            return it->second;
        }
    }

    // Link 3: the Default slot.
    if (key.material != MaterialId::Default) {
        const MaterialKey base{ MaterialId::Default, 0 };
        if (const auto it = m_materials.find(base.packed()); it != m_materials.end()) {
            ++m_resolve_stats.slot_fallbacks;
            return it->second;
        }
    }

    // Link 4: the built-in. Valid before init() and never removed, so this cannot
    // fail and a draw is never lost to a missing material.
    ++m_resolve_stats.hard_fallbacks;
    return m_fallback;
}

void MaterialLibrary::set(MaterialKey key, MaterialDef def) {
    // Validation, not correction. Both of these are authoring mistakes whose
    // symptom is subtle -- a marking sampling its neighbouring sprite, or a bias
    // that silently does nothing -- so they are named rather than quietly fixed.
    if (key.material == MaterialId::Markings && def.uv_scale != glm::vec2(1.0f)) {
        spdlog::warn("MaterialLibrary: Markings material '{}' has uv_scale ({}, {}); marking "
                     "geometry carries atlas sub-rect UVs, which cannot be scaled without "
                     "sampling the neighbouring sprite",
                     def.name, def.uv_scale.x, def.uv_scale.y);
    }
    if (def.depth_bias != 0.0f && !def.alpha_blend) {
        spdlog::warn("MaterialLibrary: material '{}' has depth_bias {} but alpha_blend false; "
                     "both come from the decal pipeline, so the bias will have no effect",
                     def.name, def.depth_bias);
    }

    m_materials[key.packed()] = std::move(def);
}

bool MaterialLibrary::has(MaterialKey key) const {
    return m_materials.find(key.packed()) != m_materials.end();
}

std::vector<MaterialKey> MaterialLibrary::keys() const {
    std::vector<uint32_t> packed;
    packed.reserve(m_materials.size());
    for (const auto& kv : m_materials) {
        packed.push_back(kv.first);
    }
    std::sort(packed.begin(), packed.end());

    std::vector<MaterialKey> out;
    out.reserve(packed.size());
    for (const uint32_t p : packed) {
        out.push_back(MaterialKey{ static_cast<MaterialId>(p >> 16),
                                   static_cast<uint16_t>(p & 0xFFFFu) });
    }
    return out;
}

MaterialUniforms MaterialLibrary::uniforms_for(MaterialKey key) const {
    const MaterialDef& def = resolve(key);

    MaterialUniforms u;
    u.base_color = def.base_color;
    u.pbr_params = glm::vec4(def.metallic, def.roughness, def.ao, def.emissive);
    u.uv_params = glm::vec4(def.uv_scale.x, def.uv_scale.y, 0.0f, 0.0f);
    return u;
}

size_t MaterialLibrary::size() const {
    return m_materials.size();
}

const MaterialLibrary::ResolveStats& MaterialLibrary::resolve_stats() const {
    return m_resolve_stats;
}

std::vector<MaterialKey> MaterialLibrary::fallback_keys() const {
    std::vector<uint32_t> packed;
    packed.reserve(m_fallback_keys.size());
    for (const auto& kv : m_fallback_keys) {
        packed.push_back(kv.first);
    }
    std::sort(packed.begin(), packed.end());

    std::vector<MaterialKey> out;
    out.reserve(packed.size());
    for (const uint32_t p : packed) {
        out.push_back(MaterialKey{ static_cast<MaterialId>(p >> 16),
                                   static_cast<uint16_t>(p & 0xFFFFu) });
    }
    return out;
}

void MaterialLibrary::reset_resolve_stats() {
    m_resolve_stats = ResolveStats{};
    m_fallback_keys.clear();
}

// ============================================================================
// The built-in table
// ============================================================================

void MaterialLibrary::load_defaults() {
    const TextureHandle white = m_textures ? m_textures->white() : kInvalidTexture;
    const TextureHandle flat = m_textures ? m_textures->flat_normal() : kInvalidTexture;
    const TextureHandle orm = m_textures ? m_textures->default_orm() : kInvalidTexture;

    if (m_textures == nullptr) {
        spdlog::warn("MaterialLibrary::load_defaults called before init(); materials will "
                     "bind the missing-texture checker until init() and load_defaults() are "
                     "called in that order");
    }

    m_materials.clear();
    m_texture_paths.clear();

    // Every built-in surface is dielectric: asphalt, stone, concrete, soil and
    // vegetation all have metallic 0, and a non-zero metallic on any of them is an
    // authoring error rather than a stylistic choice.
    const auto surface = [&](MaterialId slot, uint16_t variant, const char* name,
                             glm::vec3 base, float roughness) {
        MaterialDef def;
        def.name = name;
        def.base_color = glm::vec4(base, 1.0f);
        def.metallic = 0.0f;
        def.roughness = roughness;
        def.ao = 1.0f;
        def.emissive = 0.0f;
        def.uv_scale = glm::vec2(1.0f);
        def.albedo = white;
        def.normal = flat;
        def.orm = orm;
        def.sampler = SamplerKind::RepeatAniso;
        def.alpha_blend = false;
        def.depth_bias = 0.0f;
        set(MaterialKey{ slot, variant }, std::move(def));
    };

    // --- slot defaults, the frozen table in the header -----------------------
    surface(MaterialId::Default,    0, "Default",    { 0.60f, 0.60f, 0.60f }, 0.85f);
    surface(MaterialId::Asphalt,    0, "Asphalt",    { 0.11f, 0.11f, 0.12f }, 0.82f);
    surface(MaterialId::Concrete,   0, "Concrete",   { 0.52f, 0.51f, 0.49f }, 0.78f);
    surface(MaterialId::Curb,       0, "Curb",       { 0.62f, 0.61f, 0.58f }, 0.70f);
    surface(MaterialId::Sidewalk,   0, "Sidewalk",   { 0.55f, 0.54f, 0.52f }, 0.80f);
    surface(MaterialId::Gravel,     0, "Gravel",     { 0.45f, 0.42f, 0.38f }, 0.95f);
    surface(MaterialId::Dirt,       0, "Dirt",       { 0.36f, 0.28f, 0.20f }, 0.97f);
    surface(MaterialId::Grass,      0, "Grass",      { 0.20f, 0.33f, 0.14f }, 0.90f);
    surface(MaterialId::BridgeDeck, 0, "BridgeDeck", { 0.46f, 0.46f, 0.45f }, 0.75f);
    surface(MaterialId::Parapet,    0, "Parapet",    { 0.58f, 0.58f, 0.57f }, 0.65f);

    // --- Markings: the one structurally different slot -----------------------
    // Paint is a coverage mask over the carriageway, not an opaque surface, and its
    // quads are coplanar with the surface they sit on. Both facts are PIPELINE
    // state; see the report and MaterialDef::needs_decal_pipeline().
    {
        MaterialDef def;
        def.name = "Markings";
        def.base_color = glm::vec4(0.95f, 0.95f, 0.93f, 1.0f);
        def.metallic = 0.0f;
        def.roughness = 0.55f;   // thermoplastic paint is smoother than the asphalt under it
        def.ao = 1.0f;
        def.emissive = 0.0f;
        def.uv_scale = glm::vec2(1.0f);   // MUST stay {1,1}: these are atlas sub-rects
        def.albedo = white;
        def.normal = flat;
        def.orm = orm;
        def.sampler = SamplerKind::ClampLinear;   // an atlas rect must never wrap
        def.alpha_blend = true;
        def.depth_bias = kMarkingDepthBias;
        set(MaterialKey{ MaterialId::Markings, 0 }, std::move(def));
    }

    spdlog::info("MaterialLibrary: installed {} built-in materials", m_materials.size());
    reset_resolve_stats();
}

void MaterialLibrary::load_variant_defaults() {
    const TextureHandle white = m_textures ? m_textures->white() : kInvalidTexture;
    const TextureHandle flat = m_textures ? m_textures->flat_normal() : kInvalidTexture;
    const TextureHandle orm = m_textures ? m_textures->default_orm() : kInvalidTexture;

    const auto surface = [&](MaterialId slot, uint16_t variant, const char* name,
                             glm::vec3 base, float roughness) {
        MaterialDef def;
        def.name = name;
        def.base_color = glm::vec4(base, 1.0f);
        def.metallic = 0.0f;
        def.roughness = roughness;
        def.ao = 1.0f;
        def.emissive = 0.0f;
        def.uv_scale = glm::vec2(1.0f);
        def.albedo = white;
        def.normal = flat;
        def.orm = orm;
        def.sampler = SamplerKind::RepeatAniso;
        def.alpha_blend = false;
        def.depth_bias = 0.0f;
        set(MaterialKey{ slot, variant }, std::move(def));
    };

    const size_t before = m_materials.size();

    // --- variants ------------------------------------------------------------
    // See material_variant in the header for the numbering and, more importantly,
    // for the warning that the numbering is an ASSUMPTION to be reconciled with
    // osm/road/road_style.hpp at merge time. Nothing below is load-bearing: a
    // number that turns out to be wrong produces a surface drawn as its slot
    // default, which resolve_stats() reports and which nobody has to debug.
    using namespace material_variant;

    // Carriageway. Cobbled and setted streets are Asphalt-slot geometry with a
    // surface= tag, not a separate slot, because they are the same running surface.
    surface(MaterialId::Asphalt, kSmooth,       "Asphalt/smooth",        { 0.13f, 0.13f, 0.14f }, 0.68f);
    surface(MaterialId::Asphalt, kWorn,         "Asphalt/worn",          { 0.17f, 0.165f, 0.16f }, 0.90f);
    surface(MaterialId::Asphalt, kCobblestone,  "Asphalt/cobblestone",   { 0.30f, 0.28f, 0.26f }, 0.80f);
    surface(MaterialId::Asphalt, kSett,         "Asphalt/sett",          { 0.26f, 0.25f, 0.24f }, 0.72f);
    surface(MaterialId::Asphalt, kPavingStones, "Asphalt/paving-stones", { 0.42f, 0.40f, 0.38f }, 0.78f);

    // Concrete carriageway and hard standing. Cobbles have no meaning here, so
    // 3..5 are left out and fall back to Concrete/0.
    surface(MaterialId::Concrete, kSmooth, "Concrete/smooth", { 0.56f, 0.55f, 0.53f }, 0.62f);
    surface(MaterialId::Concrete, kWorn,   "Concrete/worn",   { 0.44f, 0.43f, 0.41f }, 0.88f);

    // Footways. The smooth variant is a bitumen footway, which is dark; that is not
    // a copy-paste error from the Asphalt slot.
    surface(MaterialId::Sidewalk, kSmooth,        "Sidewalk/smooth",        { 0.22f, 0.22f, 0.23f }, 0.78f);
    surface(MaterialId::Sidewalk, kWorn,          "Sidewalk/worn",          { 0.46f, 0.45f, 0.43f }, 0.90f);
    surface(MaterialId::Sidewalk, kCobblestone,   "Sidewalk/cobblestone",   { 0.34f, 0.32f, 0.30f }, 0.82f);
    surface(MaterialId::Sidewalk, kSett,          "Sidewalk/sett",          { 0.30f, 0.29f, 0.28f }, 0.74f);
    surface(MaterialId::Sidewalk, kPavingStones,  "Sidewalk/paving-stones", { 0.58f, 0.57f, 0.55f }, 0.76f);
    surface(MaterialId::Sidewalk, kTactilePaving, "Sidewalk/tactile",       { 0.62f, 0.55f, 0.30f }, 0.72f);

    // Kerbs. 1 and 2 carry the fine/coarse axis as granite and worn concrete; see
    // the header block. Granite is darker and markedly glossier than a concrete
    // kerb, which is the difference someone is looking for when they tag it.
    surface(MaterialId::Curb, kSmooth, "Curb/granite",  { 0.42f, 0.42f, 0.44f }, 0.45f);
    surface(MaterialId::Curb, kWorn,   "Curb/concrete", { 0.55f, 0.54f, 0.51f }, 0.82f);

    // Bridge structure follows Concrete's axis.
    surface(MaterialId::BridgeDeck, kSmooth, "BridgeDeck/smooth", { 0.50f, 0.50f, 0.49f }, 0.60f);
    surface(MaterialId::BridgeDeck, kWorn,   "BridgeDeck/worn",   { 0.40f, 0.40f, 0.39f }, 0.88f);

    spdlog::info("MaterialLibrary: installed {} built-in variants",
                 m_materials.size() - before);
}

// ============================================================================
// Procedural textures
// ============================================================================

bool MaterialLibrary::install_procedural_textures(uint32_t texture_size, uint32_t seed) {
    if (m_textures == nullptr) {
        spdlog::error("MaterialLibrary::install_procedural_textures called before init()");
        return false;
    }
    if (m_materials.empty()) {
        spdlog::error("MaterialLibrary::install_procedural_textures called before "
                      "load_defaults(); there is nothing to attach textures to");
        return false;
    }
    if (!is_valid_texture_size(texture_size, 4u)) {
        spdlog::error("MaterialLibrary::install_procedural_textures: texture_size {} is not a "
                      "power of two >= 4", texture_size);
        return false;
    }

    // The stone and paving fields below exist for the VARIANTS, so the variant
    // entries have to be present before there is anything to attach them to. See
    // load_variant_defaults() for why this is not folded into load_defaults().
    load_variant_defaults();

    bool all_ok = true;

    /// Stage one generated map and return its handle, or kInvalidTexture.
    const auto upload = [&](const ProcTexResult& r, const char* what,
                            uint32_t mip_cap) -> TextureHandle {
        if (!r.is_valid()) {
            spdlog::error("MaterialLibrary: generator for '{}' returned an invalid result", what);
            all_ok = false;
            return kInvalidTexture;
        }
        TextureDesc desc = r.desc;
        desc.mip_levels = std::min(mip_count_for(std::max(desc.width, desc.height)), mip_cap);
        const TextureHandle h = m_textures->create(desc, r.pixels.data(), r.pixels.size());
        if (h == kInvalidTexture) {
            spdlog::error("MaterialLibrary: failed to create the '{}' texture", what);
            all_ok = false;
        }
        return h;
    };

    /// An albedo map and the normal map derived from the height it carries in alpha.
    struct SurfaceMaps {
        TextureHandle albedo = kInvalidTexture;
        TextureHandle normal = kInvalidTexture;
    };

    const auto make_surface = [&](const ProcTexResult& r, float normal_strength,
                                  const char* what) -> SurfaceMaps {
        SurfaceMaps maps;
        maps.albedo = upload(r, what, 32u);
        maps.normal = upload(make_normal_from_height(r, normal_strength), what, 32u);
        if (maps.albedo == kInvalidTexture) { maps.albedo = m_textures->white(); }
        if (maps.normal == kInvalidTexture) { maps.normal = m_textures->flat_normal(); }
        return maps;
    };

    /**
     * Attach maps to an EXISTING entry, leaving the PBR scalars exactly as
     * load_defaults() and load_variant_defaults() authored them.
     *
     * base_color is NOT left alone, and that is the point. It changes MEANING the
     * moment a real albedo map replaces the 1x1 white fallback:
     *
     *   * With the white fallback bound, base_color IS the surface albedo. The
     *     slot table authors it that way on purpose -- Asphalt (0.11, 0.11, 0.12)
     *     -- so load_defaults() alone produces a usable, differentiated world with
     *     no assets present at all.
     *   * With a generated map bound, the MAP is the surface albedo. The
     *     generators are authored as the final surface colour (procedural_texture.hpp),
     *     e.g. asphalt around sRGB 0.235, which the sRGB view hands the shader as
     *     ~0.045 linear.
     *
     * mesh_pbr.frag computes `albedo = frag_color.rgb * base_color.rgb * texture`,
     * so keeping both meant multiplying two independently-authored full albedos:
     * 0.11 * 0.045 = 0.005, about a NINTH of either, and road geometry carries
     * vertex colour (1,1,1) so nothing else lifted it. Every textured slot was
     * darkened by the same squaring.
     *
     * base_color therefore becomes the neutral TINT its documentation
     * (material_library.hpp) says it is. Alpha is preserved, because it is a
     * coverage multiplier rather than a colour. Materials whose generation FAILED
     * keep their authored colour, because they are still binding the white
     * fallback and the first bullet still applies to them.
     */
    const auto attach = [&](MaterialKey key, const SurfaceMaps& maps, glm::vec2 uv_scale) {
        const auto it = m_materials.find(key.packed());
        if (it == m_materials.end()) {
            spdlog::warn("MaterialLibrary: no entry for {}/variant {} to attach a texture to",
                         material_id_name(key.material), key.variant);
            return;
        }
        const bool textured =
            maps.albedo != kInvalidTexture && maps.albedo != m_textures->white();
        if (textured) {
            it->second.base_color = glm::vec4(1.0f, 1.0f, 1.0f, it->second.base_color.a);
        }
        it->second.albedo = maps.albedo;
        it->second.normal = maps.normal;
        it->second.orm = m_textures->default_orm();
        it->second.uv_scale = uv_scale;
    };

    const uint32_t s = texture_size;

    // --- generation ----------------------------------------------------------
    // The generators are authored to tile at a stated number of metres. Where that
    // does not match the slot's tile size from the plan's UV Convention, the
    // difference is carried by uv_scale rather than by regenerating geometry --
    // which is exactly what MaterialDef::uv_scale is documented to be for.
    //
    // The cobble and sett fields are authored at 4 m rather than at Asphalt's 8 m,
    // because a realistic 0.16 m sett across an 8 m tile is 50 stones, which at 512
    // texels is 10 texels a stone and reads as noise. At 4 m it is 25 stones and 20
    // texels, and uv_scale {2,2} puts it back at the right world size.
    const SurfaceMaps asphalt = make_surface(
        make_asphalt(s, derive_seed(seed, 1), 0.5f), 0.6f, "asphalt");
    const SurfaceMaps asphalt_smooth = make_surface(
        make_asphalt(s, derive_seed(seed, 2), 0.12f), 0.35f, "asphalt-smooth");
    const SurfaceMaps asphalt_worn = make_surface(
        make_asphalt(s, derive_seed(seed, 3), 0.92f), 0.85f, "asphalt-worn");
    const SurfaceMaps concrete = make_surface(
        make_concrete(s, derive_seed(seed, 4)), 0.45f, "concrete");
    const SurfaceMaps cobble = make_surface(
        make_cobblestone(s, derive_seed(seed, 5), 0.06f), 1.40f, "cobblestone");
    const SurfaceMaps sett = make_surface(
        make_cobblestone(s, derive_seed(seed, 6), 0.04f), 1.00f, "sett");
    const SurfaceMaps paving_slab = make_surface(
        make_paving(s, derive_seed(seed, 7), { 0.5f, 0.25f }), 0.90f, "paving-slab");
    const SurfaceMaps paving_block = make_surface(
        make_paving(s, derive_seed(seed, 8), { 0.25f, 0.25f }), 0.90f, "paving-block");
    const SurfaceMaps tactile = make_surface(
        make_paving(s, derive_seed(seed, 9), { 0.125f, 0.125f }), 1.30f, "tactile-paving");
    const SurfaceMaps gravel = make_surface(
        make_gravel(s, derive_seed(seed, 10)), 1.20f, "gravel");
    const SurfaceMaps dirt = make_surface(
        make_dirt(s, derive_seed(seed, 11)), 0.80f, "dirt");
    const SurfaceMaps grass = make_surface(
        make_grass(s, derive_seed(seed, 12)), 0.70f, "grass");
    const SurfaceMaps kerb = make_surface(
        make_kerb(s, derive_seed(seed, 13)), 1.00f, "kerb");

    // --- attachment ----------------------------------------------------------
    using namespace material_variant;

    attach(MaterialKey{ MaterialId::Default, 0 }, concrete, { 1.0f, 1.0f });

    // Asphalt tiles at 8 m; the asphalt generator is authored at 8 m, the stone
    // fields at 4 m and the block paving at 4 m.
    attach(MaterialKey{ MaterialId::Asphalt, kDefault },      asphalt,        { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Asphalt, kSmooth },       asphalt_smooth, { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Asphalt, kWorn },         asphalt_worn,   { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Asphalt, kCobblestone },  cobble,         { 2.0f, 2.0f });
    attach(MaterialKey{ MaterialId::Asphalt, kSett },         sett,           { 2.0f, 2.0f });
    attach(MaterialKey{ MaterialId::Asphalt, kPavingStones }, paving_block,   { 2.0f, 2.0f });

    // Concrete tiles at 4 m and the concrete generator is authored at 4 m.
    attach(MaterialKey{ MaterialId::Concrete, kDefault }, concrete, { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Concrete, kSmooth },  concrete, { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Concrete, kWorn },    concrete, { 1.0f, 1.0f });

    // Sidewalk tiles at 2 m. The slab paving is authored for exactly that, so it
    // needs no correction; the 4 m stone fields are halved onto it.
    attach(MaterialKey{ MaterialId::Sidewalk, kDefault },       paving_slab,    { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Sidewalk, kSmooth },        asphalt_smooth, { 0.25f, 0.25f });
    attach(MaterialKey{ MaterialId::Sidewalk, kWorn },          concrete,       { 0.5f, 0.5f });
    attach(MaterialKey{ MaterialId::Sidewalk, kCobblestone },   cobble,         { 0.5f, 0.5f });
    attach(MaterialKey{ MaterialId::Sidewalk, kSett },          sett,           { 0.5f, 0.5f });
    attach(MaterialKey{ MaterialId::Sidewalk, kPavingStones },  paving_block,   { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Sidewalk, kTactilePaving }, tactile,        { 1.0f, 1.0f });

    // Curb is the split texture: face on the left half of U, top on the right. It
    // must never be corrected, because the halves are not interchangeable and a
    // uv_scale other than 1 slides the split off the geometry's 0.5 boundary.
    attach(MaterialKey{ MaterialId::Curb, kDefault }, kerb, { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Curb, kSmooth },  kerb, { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Curb, kWorn },    kerb, { 1.0f, 1.0f });

    attach(MaterialKey{ MaterialId::Gravel, kDefault }, gravel, { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Dirt,   kDefault }, dirt,   { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Grass,  kDefault }, grass,  { 1.0f, 1.0f });

    attach(MaterialKey{ MaterialId::BridgeDeck, kDefault }, concrete, { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::BridgeDeck, kSmooth },  concrete, { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::BridgeDeck, kWorn },    concrete, { 1.0f, 1.0f });
    attach(MaterialKey{ MaterialId::Parapet,    kDefault }, concrete, { 1.0f, 1.0f });

    // --- the markings atlas --------------------------------------------------
    // Generated at the atlas header's own resolution so the 64 px cell layout is
    // exact rather than resampled, and given a SHORT mip chain on purpose: the
    // atlas is inset by one pixel, which survives a couple of halvings and does not
    // survive ten. Capping at four levels stops a distant marking from bleeding its
    // neighbours in while still avoiding the shimmer an unmipped atlas produces.
    {
        const ProcTexResult atlas =
            make_markings_atlas(static_cast<uint32_t>(osm::road::kAtlasSizePixels));
        const TextureHandle handle = upload(atlas, "markings-atlas", kMarkingAtlasMipLevels);

        const auto it = m_materials.find(MaterialKey{ MaterialId::Markings, 0 }.packed());
        if (it != m_materials.end()) {
            it->second.albedo = (handle != kInvalidTexture) ? handle : m_textures->white();
            it->second.normal = m_textures->flat_normal();   // paint has no relief worth a map
            it->second.orm = m_textures->default_orm();
            it->second.uv_scale = glm::vec2(1.0f);
            it->second.sampler = SamplerKind::ClampLinear;
            if (handle != kInvalidTexture) {
                // Same reasoning as attach(): the atlas carries the paint colour
                // per sprite -- white for most, yellow for the yellow lines -- so a
                // near-white base_color would both dim the white paint and shift
                // the yellow. Alpha is coverage and is preserved.
                it->second.base_color =
                    glm::vec4(1.0f, 1.0f, 1.0f, it->second.base_color.a);
            }
        }
    }

    if (all_ok) {
        spdlog::info("MaterialLibrary: generated procedural textures at {}x{} (seed {})",
                     texture_size, texture_size, seed);
    } else {
        spdlog::warn("MaterialLibrary: some procedural textures failed; affected materials "
                     "keep the manager's flat fallbacks");
    }
    return all_ok;
}

// ============================================================================
// Set files
// ============================================================================

bool MaterialLibrary::load_map_from_file(MaterialKey key, TextureMap map,
                                        const std::filesystem::path& path) {
    if (!m_textures) {
        spdlog::error("MaterialLibrary: no texture manager; cannot load '{}'", path.string());
        return false;
    }

    // Albedo is colour and is sampled through an sRGB view; normal and ORM are
    // DATA -- a tangent vector and three scalars -- and must not be gamma-decoded.
    const bool srgb = (map == TextureMap::Albedo);

    // Resolve the path before loading so the recorded path is the same one a later
    // relative save is computed against, whatever the caller passed in.
    std::error_code ec;
    const std::filesystem::path canonical = std::filesystem::weakly_canonical(path, ec);
    const std::filesystem::path use = ec ? path : canonical;

    const TextureHandle handle = m_textures->load(use, srgb);
    if (handle == kInvalidTexture) {
        // load() has already logged the reason and counted the failure into
        // GPUTextureManager::Stats::load_failures, which the editor displays.
        // Leave the material completely untouched: a mistyped path must cost
        // nothing, not clear the map that was working.
        return false;
    }

    // Seed from resolve() rather than from a blank MaterialDef, so loading a
    // texture onto a key that only existed through the fallback chain keeps the
    // colour and roughness it was drawing with and gains a texture, instead of
    // snapping to white.
    MaterialDef def = resolve(key);

    switch (map) {
        case TextureMap::Albedo: def.albedo = handle; break;
        case TextureMap::Normal: def.normal = handle; break;
        case TextureMap::Orm:    def.orm = handle; break;
    }

    // The handle a material previously held is NOT released here; see the header.
    m_texture_paths[handle] = use;
    set(key, std::move(def));

    spdlog::info("MaterialLibrary: loaded '{}' into {}/{} as {}", use.string(),
                 material_id_name(key.material), key.variant,
                 map == TextureMap::Albedo ? "albedo"
                     : map == TextureMap::Normal ? "normal" : "orm");
    return true;
}

std::filesystem::path MaterialLibrary::texture_source_path(TextureHandle handle) const {
    const auto it = m_texture_paths.find(handle);
    return it == m_texture_paths.end() ? std::filesystem::path{} : it->second;
}

bool MaterialLibrary::load_from_file(const std::filesystem::path& path) {
    if (m_textures == nullptr) {
        spdlog::error("MaterialLibrary::load_from_file called before init()");
        return false;
    }

    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) {
        spdlog::error("MaterialLibrary: material set '{}' does not exist; the library is "
                      "unchanged", path.string());
        return false;
    }

    std::ifstream in(path);
    if (!in.is_open()) {
        spdlog::error("MaterialLibrary: cannot open material set '{}'; the library is unchanged",
                      path.string());
        return false;
    }

    nlohmann::json doc;
    try {
        in >> doc;
    } catch (const std::exception& e) {
        spdlog::error("MaterialLibrary: material set '{}' is not valid JSON ({}); the library "
                      "is unchanged", path.string(), e.what());
        return false;
    }

    if (!doc.is_object()) {
        spdlog::error("MaterialLibrary: material set '{}' is not a JSON object; the library is "
                      "unchanged", path.string());
        return false;
    }

    const auto version_it = doc.find("version");
    if (version_it == doc.end() || !version_it->is_number_unsigned()) {
        spdlog::error("MaterialLibrary: material set '{}' has no numeric \"version\"; the "
                      "library is unchanged", path.string());
        return false;
    }
    if (version_it->get<uint32_t>() != kSetFileVersion) {
        spdlog::error("MaterialLibrary: material set '{}' is version {}, this build reads "
                      "version {}; the library is unchanged",
                      path.string(), version_it->get<uint32_t>(), kSetFileVersion);
        return false;
    }

    const auto materials_it = doc.find("materials");
    if (materials_it == doc.end() || !materials_it->is_array()) {
        spdlog::error("MaterialLibrary: material set '{}' has no \"materials\" array; the "
                      "library is unchanged", path.string());
        return false;
    }

    const std::filesystem::path base = path.parent_path();

    // Everything is built into a STAGING set first. A half-applied material set --
    // some slots from the file, the rest left over from whatever was loaded before
    // -- is worse than no set at all, because it looks like a working set and the
    // wrong half is the half nobody looks at.
    std::unordered_map<uint32_t, MaterialDef> staged;
    std::unordered_map<TextureHandle, std::filesystem::path> staged_paths;
    size_t skipped = 0;

    // Resolve one optional texture field. A load failure skips THAT FIELD and
    // leaves the map unbound, which draws as plain rather than as nothing.
    const auto load_map = [&](const nlohmann::json& entry, const char* field, bool srgb,
                              TextureHandle fallback) -> TextureHandle {
        const std::string rel = read_string(entry, field);
        if (rel.empty()) {
            return fallback;
        }
        const std::filesystem::path abs =
            std::filesystem::weakly_canonical(base / std::filesystem::path(rel), ec);
        const std::filesystem::path use = ec ? (base / std::filesystem::path(rel)) : abs;
        ec.clear();

        const TextureHandle h = m_textures->load(use, srgb);
        if (h == kInvalidTexture) {
            spdlog::warn("MaterialLibrary: '{}' failed to load for field \"{}\"; that map is "
                         "left unbound", use.string(), field);
            return fallback;
        }
        staged_paths[h] = use;
        return h;
    };

    for (const auto& entry : *materials_it) {
        if (!entry.is_object()) {
            spdlog::warn("MaterialLibrary: skipping a non-object entry in '{}'", path.string());
            ++skipped;
            continue;
        }

        const std::string slot_name = read_string(entry, "slot");
        MaterialId slot = MaterialId::Default;
        if (slot_name.empty() || !material_id_from_name(slot_name, slot)) {
            spdlog::warn("MaterialLibrary: skipping entry with unknown slot \"{}\" in '{}'",
                         slot_name, path.string());
            ++skipped;
            continue;
        }

        uint16_t variant = 0;
        if (const auto it = entry.find("variant"); it != entry.end()) {
            if (!it->is_number_unsigned() || it->get<uint64_t>() > 0xFFFFull) {
                spdlog::warn("MaterialLibrary: skipping {} entry with a bad \"variant\" in '{}'",
                             slot_name, path.string());
                ++skipped;
                continue;
            }
            variant = static_cast<uint16_t>(it->get<uint64_t>());
        }

        MaterialDef def;
        def.name = read_string(entry, "name");
        if (def.name.empty()) {
            def.name = variant == 0 ? slot_name
                                    : slot_name + "/" + std::to_string(variant);
        }

        float rgba[4] = { def.base_color.r, def.base_color.g, def.base_color.b, def.base_color.a };
        if (read_floats(entry, "base_color", rgba, 4)) {
            def.base_color = glm::vec4(rgba[0], rgba[1], rgba[2], rgba[3]);
        } else if (entry.contains("base_color")) {
            spdlog::warn("MaterialLibrary: '{}' has a malformed \"base_color\"; keeping the "
                         "default", def.name);
        }

        float uv[2] = { def.uv_scale.x, def.uv_scale.y };
        if (read_floats(entry, "uv_scale", uv, 2)) {
            def.uv_scale = glm::vec2(uv[0], uv[1]);
        } else if (entry.contains("uv_scale")) {
            spdlog::warn("MaterialLibrary: '{}' has a malformed \"uv_scale\"; keeping {{1, 1}}",
                         def.name);
        }

        read_float(entry, "metallic", def.metallic);
        read_float(entry, "roughness", def.roughness);
        read_float(entry, "ao", def.ao);
        read_float(entry, "emissive", def.emissive);
        read_float(entry, "depth_bias", def.depth_bias);
        read_bool(entry, "alpha_blend", def.alpha_blend);

        const std::string sampler_name = read_string(entry, "sampler");
        if (!sampler_name.empty() && !sampler_kind_from_name(sampler_name, def.sampler)) {
            spdlog::warn("MaterialLibrary: '{}' names unknown sampler \"{}\"; using {}",
                         def.name, sampler_name, sampler_kind_name(def.sampler));
        }

        // Albedo is COLOUR and must be sampled through an sRGB format; normal and
        // ORM are DATA and must not be. Getting this wrong is invisible in a
        // screenshot and wrong in every lighting calculation.
        def.albedo = load_map(entry, "albedo", true, m_textures->white());
        def.normal = load_map(entry, "normal", false, m_textures->flat_normal());
        def.orm = load_map(entry, "orm", false, m_textures->default_orm());

        staged[MaterialKey{ slot, variant }.packed()] = std::move(def);
    }

    if (staged.empty()) {
        spdlog::error("MaterialLibrary: material set '{}' installed no materials ({} entries "
                      "skipped); the library is unchanged", path.string(), skipped);
        // These textures were loaded for a set that is being thrown away, so nothing
        // else can be holding them.
        for (const auto& kv : staged_paths) {
            m_textures->release(kv.first);
        }
        return false;
    }

    // Commit. Textures the OLD set held are not released, for the same reason
    // shutdown() does not release them: a handle may be shared with another
    // library or an editor panel, and this library cannot know.
    m_materials.clear();
    m_texture_paths.clear();
    for (auto& [packed, def] : staged) {
        set(MaterialKey{ static_cast<MaterialId>(packed >> 16),
                         static_cast<uint16_t>(packed & 0xFFFFu) },
            std::move(def));
    }
    m_texture_paths = std::move(staged_paths);
    reset_resolve_stats();

    spdlog::info("MaterialLibrary: loaded {} materials from '{}' ({} entries skipped)",
                 m_materials.size(), path.string(), skipped);
    return true;
}

bool MaterialLibrary::save_to_file(const std::filesystem::path& path) const {
    std::error_code ec;
    const std::filesystem::path base = path.parent_path();
    if (!base.empty()) {
        std::filesystem::create_directories(base, ec);
        if (ec) {
            spdlog::error("MaterialLibrary: cannot create '{}' ({})", base.string(),
                          ec.message());
            return false;
        }
    }

    // Write a texture path only when this library knows one. A procedurally
    // generated map has none, so its field is omitted and the material reloads with
    // that map unbound: generated maps are regenerated, never stored.
    const auto emit_path = [&](nlohmann::json& entry, const char* field, TextureHandle h) {
        const auto it = m_texture_paths.find(h);
        if (it != m_texture_paths.end()) {
            entry[field] = relative_or_absolute(it->second, base);
        }
    };

    nlohmann::json materials = nlohmann::json::array();
    for (const MaterialKey key : keys()) {
        const auto it = m_materials.find(key.packed());
        if (it == m_materials.end()) {
            continue;
        }
        const MaterialDef& def = it->second;

        nlohmann::json entry;
        entry["slot"] = material_id_name(key.material);
        entry["variant"] = key.variant;
        entry["name"] = def.name;
        entry["base_color"] = { def.base_color.r, def.base_color.g, def.base_color.b,
                                def.base_color.a };
        entry["metallic"] = def.metallic;
        entry["roughness"] = def.roughness;
        entry["ao"] = def.ao;
        entry["emissive"] = def.emissive;
        entry["uv_scale"] = { def.uv_scale.x, def.uv_scale.y };
        emit_path(entry, "albedo", def.albedo);
        emit_path(entry, "normal", def.normal);
        emit_path(entry, "orm", def.orm);
        entry["sampler"] = sampler_kind_name(def.sampler);
        entry["alpha_blend"] = def.alpha_blend;
        entry["depth_bias"] = def.depth_bias;

        materials.push_back(std::move(entry));
    }

    nlohmann::json doc;
    doc["version"] = kSetFileVersion;
    doc["materials"] = std::move(materials);

    std::ofstream out(path);
    if (!out.is_open()) {
        spdlog::error("MaterialLibrary: cannot write '{}'", path.string());
        return false;
    }
    out << doc.dump(2) << '\n';
    if (!out.good()) {
        spdlog::error("MaterialLibrary: failed while writing '{}'", path.string());
        return false;
    }

    spdlog::info("MaterialLibrary: wrote {} materials to '{}'", m_materials.size(),
                 path.string());
    return true;
}

} // namespace stratum
