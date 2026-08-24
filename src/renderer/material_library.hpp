/**
 * @file material_library.hpp
 * @brief Resolves a MaterialKey to textures and PBR parameters, and the shader contract
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The road system has emitted geometry tagged with material slots since P0.3 and
 * metre-based tiling UVs since P2, and nothing rendered them differently:
 * GPURenderer::draw_mesh() already loops the SubMesh ranges and issues one draw
 * per range, but every range drew with the same pipeline, the same uniforms and
 * no textures at all. This file is the missing lookup.
 *
 * ### The split with feat/road-optimization
 *
 * SubMesh::variant and MaterialKey (mesh.hpp) are a SHARED VOCABULARY. The
 * geometry side derives a variant from OSM tags; this side resolves the resulting
 * (slot, variant) pair to something drawable. Neither owns the other, and this
 * library is written so it works for ANY pair, INCLUDING variants it has never
 * heard of -- resolve() falls back rather than failing. Do not add a dependency
 * from here on osm/road/road_style.hpp or on any variant enumeration.
 *
 * Everything here is stratum_editor_lib. It includes SDL through texture.hpp and
 * must never be included from stratum_core.
 */

#pragma once

#include "renderer/mesh.hpp"
#include "renderer/texture.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace stratum {

// ============================================================================
// SHADER INTERFACE -- assets/shaders/mesh_pbr.frag
//
// EDIT THIS SECTION AND THE GLSL IN THE SAME COMMIT. A mismatch between the C++
// struct and the GLSL block is not a compile error and not a validation error; it
// is silent garbage in the fragment shader.
//
// ### The SDL_GPU binding rule, verbatim from SDL_CreateGPUShader's documentation
// (external/SDL/include/SDL3/SDL_gpu.h, the "For SPIR-V shaders" list):
//
//   For vertex shaders:
//     - set 0: Sampled textures, then storage textures, then storage buffers
//     - set 1: Uniform buffers
//   For fragment shaders:
//     - set 2: Sampled textures, then storage textures, then storage buffers
//     - set 3: Uniform buffers
//
// So the set number is FIXED BY STAGE AND RESOURCE KIND, not chosen by us. The
// CLAUDE.md shorthand "Set 0 = scene, Set 1 = per-mesh" describes the logical
// grouping, not the descriptor sets: physically the scene block is a FRAGMENT
// uniform buffer and therefore lives in set 3, and the per-mesh block is a VERTEX
// uniform buffer and therefore lives in set 1. That is what the shipped GLSL
// already does; the comment at the top of mesh_pbr.frag says so too.
//
// Within a set, the `binding` number is the SLOT INDEX passed to the matching SDL
// call. There is no separate slot-to-binding mapping to maintain:
//   SDL_PushGPUFragmentUniformData(cmd, slot, ...)      -> set 3, binding = slot
//   SDL_BindGPUFragmentSamplers(pass, first_slot, ...)  -> set 2, binding = first_slot + i
//   SDL_PushGPUVertexUniformData(cmd, slot, ...)        -> set 1, binding = slot
//
// GLSL declares a combined `sampler2D`; SDL_GPU supplies texture and sampler
// together in one SDL_GPUTextureSamplerBinding, and SDL's shader cross-compiler
// pairs them by index. Do NOT declare separate `texture2D`/`sampler` objects.
//
// ### The frozen layout
//
//   VERTEX STAGE
//     set 1, binding 0   MeshUniforms / MeshUniformsPBR   (existing, unchanged)
//
//   FRAGMENT STAGE -- uniform buffers, set 3
//     binding 0   SceneUniforms     (existing, unchanged) -> push slot 0
//     binding 1   MaterialUniforms  (NEW, this file)      -> push slot 1
//
//   FRAGMENT STAGE -- sampled textures, set 2
//     binding 0   sampler2D albedo_map   -> bind slot 0
//     binding 1   sampler2D normal_map   -> bind slot 1
//     binding 2   sampler2D orm_map      -> bind slot 2
//
// Consequently SDL_CreateGPUShader for mesh_pbr.frag.spv must declare
// num_uniform_buffers = 2 (was 1) and num_samplers = 3 (was 0). Under-declaring
// either is a hard SDL error at pipeline creation; over-declaring wastes a
// descriptor. GPURenderer::load_shader() gained a num_samplers parameter for this.
//
// ### What mesh_pbr.frag must gain, exactly
//
//   layout(set = 3, binding = 1) uniform MaterialUniforms {
//       vec4 base_color;   // rgb tint, a alpha
//       vec4 pbr_params;   // x metallic, y roughness, z ao, w emissive
//       vec4 uv_params;    // xy uv_scale, zw unused (keep zero)
//   } material;
//
//   layout(set = 2, binding = 0) uniform sampler2D albedo_map;
//   layout(set = 2, binding = 1) uniform sampler2D normal_map;
//   layout(set = 2, binding = 2) uniform sampler2D orm_map;
//
// and, in main(), in this order:
//   1. vec2 uv = frag_uv * material.uv_params.xy;
//      frag_uv already carries the plan's metre-based tiling UVs (U = lateral
//      metres / tile_u_metres, V = arc-length metres / tile_v_metres) and, for
//      MaterialId::Markings, an atlas sub-rect instead. uv_params is a CORRECTION
//      on top of that, normally 1.0. Markings materials MUST leave it at 1.0 --
//      scaling an atlas sub-rect samples the neighbouring sprite.
//   2. vec4 tex = texture(albedo_map, uv);
//      albedo = frag_color.rgb * material.base_color.rgb * tex.rgb;
//      alpha  = frag_color.a  * material.base_color.a  * tex.a;
//      The existing "if albedo is near-black, use 0.5 grey" hack must GO: it was
//      standing in for the absent texture path and now hides a black material.
//   3. Normal mapping from normal_map, using frag_tangent/frag_bitangent, which
//      mesh_pbr.vert already computes and passes at locations 4 and 5. Decode as
//      n = normalize(texture(normal_map, uv).xyz * 2.0 - 1.0), then
//      N = normalize(mat3(T, B, N) * n). The flat-normal fallback decodes to
//      (0,0,1), i.e. no perturbation, so no branch is needed.
//   4. vec3 orm = texture(orm_map, uv).rgb;
//      ao        = material.pbr_params.z * orm.r;
//      roughness = clamp(material.pbr_params.y * orm.g, 0.04, 1.0);
//      metallic  = clamp(material.pbr_params.x * orm.b, 0.0, 1.0);
//      Multiplicative, so the default unit ORM leaves the scalars as authored.
//      These REPLACE scene.pbr_params.xy for textured draws; scene.pbr_params
//      stays as the global fallback the editor sliders drive.
//   5. ao multiplies the ambient term only, never the direct term.
//      emissive (material.pbr_params.w) is added after the direct term and before
//      fog and tone mapping.
//   6. out_color.a = alpha. The opaque pipeline ignores it; the decal pipeline
//      blends with it, which is what makes MaterialId::Markings work.
//
// Nothing above changes mesh_pbr.vert: it already forwards in_uv to frag_uv at
// location 2 and computes the tangent frame. Do not touch it.
//
// ### Compiling the GLSL -- there is no shader build step today
//
// assets/shaders holds GLSL sources AND checked-in .spv, and CMakeLists.txt has
// no rule connecting them, so editing mesh_pbr.frag changes nothing until someone
// runs glslc by hand. That must be fixed in the same change, because a checked-in
// .spv that no longer matches its source is a trap that costs an afternoon.
//
// glslc is installed at /usr/sbin/glslc. The CMake rule to add:
//
//   find_program(STRATUM_GLSLC NAMES glslc HINTS /usr/bin /usr/sbin)
//   if(STRATUM_GLSLC)
//       set(STRATUM_SHADERS mesh.vert mesh.frag mesh_pbr.vert mesh_pbr.frag)
//       foreach(shader IN LISTS STRATUM_SHADERS)
//           add_custom_command(
//               OUTPUT  ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders/${shader}.spv
//               COMMAND ${STRATUM_GLSLC} --target-env=vulkan1.0 -O
//                       ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders/${shader}
//                    -o ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders/${shader}.spv
//               DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders/${shader}
//               COMMENT "glslc ${shader}"
//               VERBATIM)
//           list(APPEND STRATUM_SPV ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders/${shader}.spv)
//       endforeach()
//       add_custom_target(stratum_shaders DEPENDS ${STRATUM_SPV})
//       add_dependencies(stratum stratum_shaders)
//   endif()
//
// Three constraints on that rule, each of which has a wrong-looking alternative:
//  - The output goes back into the SOURCE tree, not the build tree, because the
//    runtime path is `SDL_GetBasePath() + "../../assets/shaders/..."` and the
//    .spv are checked in. Writing to the build tree instead would compile shaders
//    nobody loads.
//  - glslc must stay OPTIONAL. A checkout without it must still build against the
//    committed .spv, which is why the whole block is guarded by find_program.
//  - --target-env=vulkan1.0 matches what SDL's Vulkan backend consumes. A newer
//    target env emits SPIR-V the backend may reject at SDL_CreateGPUShader.
// ============================================================================

/// Fragment uniform-buffer slot of the scene block. Set 3, binding 0.
inline constexpr uint32_t kSceneUniformSlot = 0;

/// Fragment uniform-buffer slot of the material block. Set 3, binding 1.
inline constexpr uint32_t kMaterialUniformSlot = 1;

/// Fragment sampler slot of the albedo map. Set 2, binding 0.
inline constexpr uint32_t kAlbedoSamplerSlot = 0;

/// Fragment sampler slot of the normal map. Set 2, binding 1.
inline constexpr uint32_t kNormalSamplerSlot = 1;

/// Fragment sampler slot of the ORM map. Set 2, binding 2.
inline constexpr uint32_t kOrmSamplerSlot = 2;

/// Fragment samplers mesh_pbr.frag declares. SDL_GPUShaderCreateInfo::num_samplers.
inline constexpr uint32_t kMaterialSamplerCount = 3;

/// Fragment uniform buffers mesh_pbr.frag declares. num_uniform_buffers.
inline constexpr uint32_t kPbrFragmentUniformBufferCount = 2;

// ============================================================================
// MaterialUniforms
// ============================================================================

/**
 * @brief The per-draw fragment uniform block
 *
 * MUST match the `MaterialUniforms` block in assets/shaders/mesh_pbr.frag
 * exactly, field for field and byte for byte; see the shader interface section
 * above, and edit the two together.
 *
 * Three vec4s, 48 bytes, std140-clean by construction: every member is a vec4, so
 * every member is already 16-byte aligned and there is no padding to get wrong.
 * SDL_PushGPUFragmentUniformData explicitly requires std140 conventions
 * ("you must ensure that vec3 and vec4 fields are 16-byte aligned"), which is why
 * the scalars are PACKED INTO VECTORS rather than declared as floats. Do not
 * "tidy" this into `float metallic; float roughness; ...`.
 */
struct alignas(16) MaterialUniforms {
    glm::vec4 base_color{1.0f};     ///< rgb tint, a alpha
    glm::vec4 pbr_params{0.0f, 0.8f, 1.0f, 0.0f};   ///< x metallic, y roughness, z ao, w emissive
    glm::vec4 uv_params{1.0f, 1.0f, 0.0f, 0.0f};    ///< xy uv_scale, zw unused
};

static_assert(sizeof(MaterialUniforms) == 48,
              "MaterialUniforms must stay three tightly packed vec4s to match the std140 "
              "block in assets/shaders/mesh_pbr.frag");

// ============================================================================
// MaterialDef
// ============================================================================

/**
 * @brief One resolved material: what to bind and what to push for a MaterialKey
 *
 * Everything a submesh range needs in order to draw. The texture members are
 * HANDLES, not owned textures: GPUTextureManager owns them and a handle may be
 * shared by many materials, which is the normal case -- eight surface materials
 * commonly share one default ORM.
 */
struct MaterialDef {
    /// Human-readable name for the editor and for exported material slots.
    /// Convention: "Slot" for variant 0, "Slot/variant-name" otherwise, e.g.
    /// "Asphalt" and "Asphalt/cobblestone".
    std::string name;

    glm::vec4 base_color{1.0f};     ///< Multiplied into the albedo sample and the vertex colour
    float metallic = 0.0f;          ///< Multiplied by the ORM blue channel
    float roughness = 0.8f;         ///< Multiplied by the ORM green channel
    float ao = 1.0f;                ///< Multiplied by the ORM red channel; ambient only
    float emissive = 0.0f;          ///< Added after direct lighting

    /**
     * @brief Extra tiling multiplier ON TOP of the metres-based UVs the geometry carries
     *
     * 1.0 means "the plan's UV convention as authored": the geometry already
     * divides by the per-material tile size in metres (Asphalt 8/8, Concrete 4/4,
     * Sidewalk 2/2, Curb 0.5/2, Gravel/Dirt/Grass 4/4), so texel density is
     * already uniform across the network. This exists so a texture whose physical
     * size does not match the convention -- a cobblestone scan that is really 4 m
     * across, bound into an 8 m Asphalt slot -- can be corrected without
     * regenerating geometry.
     *
     * @warning MUST stay {1,1} for MaterialId::Markings. Marking geometry carries
     *          atlas sub-rect UVs, not tiling UVs, and an atlas rect cannot wrap:
     *          scaling it samples a neighbouring sprite. MaterialLibrary::set()
     *          logs a warning if a Markings material is given anything else.
     */
    glm::vec2 uv_scale{1.0f};

    TextureHandle albedo = kInvalidTexture;  ///< sRGB colour
    TextureHandle normal = kInvalidTexture;  ///< Linear tangent-space normal
    TextureHandle orm = kInvalidTexture;     ///< Linear occlusion / roughness / metallic packed

    SamplerKind sampler = SamplerKind::RepeatAniso; ///< ClampLinear for Markings

    /**
     * @brief Draw through the alpha-blended pipeline instead of the opaque one
     *
     * MaterialId::Markings needs this: paint is a coverage mask over the
     * carriageway, not an opaque surface.
     *
     * @note Blending is PIPELINE state in SDL_GPU (SDL_GPUColorTargetBlendState),
     *       not something a bind can change mid-pass. GPURenderer therefore keeps
     *       a second pipeline and switches to it, rather than trying to set a flag.
     */
    bool alpha_blend = false;

    /**
     * @brief Depth bias, in the units of SDL_GPURasterizerState::depth_bias_constant_factor
     *
     * Marking quads are emitted just above the carriageway and would still z-fight
     * with it at grazing angles across a city-sized depth range.
     *
     * @note Depth bias is also PIPELINE state, for the same reason as
     *       @ref alpha_blend, and the two travel together: the same secondary
     *       "decal" pipeline provides both. A non-zero bias on a material with
     *       alpha_blend == false therefore has no effect and is logged as a
     *       configuration error rather than silently ignored.
     */
    float depth_bias = 0.0f;

    /// Whether this material wants the decal pipeline rather than the opaque one.
    [[nodiscard]] bool needs_decal_pipeline() const { return alpha_blend || depth_bias != 0.0f; }
};

// ============================================================================
// Variant numbering -- THE ONE PLACE TO RECONCILE WITH osm/road/road_style.hpp
// ============================================================================

/**
 * @brief The variant numbers this library assumes the geometry side emits
 *
 * @warning THESE NUMBERS ARE AN ASSUMPTION, NOT A CONTRACT. SubMesh::variant is a
 *          plain integer whose meaning is assigned by the tag-to-style mapping in
 *          osm/road/road_style.hpp, which lives on feat/road-optimization and does
 *          not exist in this branch. Nothing here reads that header and nothing
 *          here may: the two sides are being written in parallel. This block is
 *          the ONLY place a variant number appears in the renderer, so reconciling
 *          the two numberings at merge time is an edit to these constants and
 *          nothing else. If the numbering turns out to disagree, the symptom is a
 *          cobbled street drawn as plain asphalt -- resolve() falls back to the
 *          slot default -- and MaterialLibrary::resolve_stats() reports it. It is
 *          never a crash and never a missing draw. That is the whole point of the
 *          fallback chain.
 *
 * ### Why one shared numbering across slots rather than a private list per slot
 *
 * The geometry side derives a variant from `surface=`, and `surface=cobblestone`
 * means the same thing on a carriageway, a footway and a bridge deck. Giving
 * "cobblestone" one number in every slot means a slot that has no cobbled entry
 * falls back to its own default rather than to some unrelated material that
 * happens to share the number. Per-slot private lists would make variant 3 of
 * Sidewalk and variant 3 of Asphalt unrelated, which mesh.hpp permits but which
 * turns every numbering mistake into a wrong-looking surface instead of a plain
 * one.
 *
 * | Value | Meaning                    | Typical OSM tags                          |
 * |-------|----------------------------|-------------------------------------------|
 * | 0     | the slot default           | absent, or unrecognised                   |
 * | 1     | smooth / fine              | smoothness=excellent, surface=asphalt new |
 * | 2     | worn / coarse              | smoothness=bad, surface=asphalt worn      |
 * | 3     | cobblestone                | surface=cobblestone, unhewn_cobblestone   |
 * | 4     | sett                       | surface=sett, cobblestone:flattened       |
 * | 5     | paving stones              | surface=paving_stones, bricks, concrete:plates |
 * | 6+    | slot-specific extension    | see below                                 |
 *
 * MaterialId::Curb reuses 1 and 2 for the two kerb stones the survey data
 * distinguishes, which are the same "fine versus coarse" axis in a slot where
 * "smooth asphalt" is meaningless: 1 is a granite kerb (darker, tighter, glossier)
 * and 2 is a worn concrete one. Curb 3..5 are deliberately NOT installed -- a
 * "cobblestone kerb" is not a thing -- so they fall back to Curb/0.
 *
 * Values at 6 and above are slot-specific and cannot be expressed on the shared
 * axis. Only one exists today: MaterialId::Sidewalk variant 6, tactile paving,
 * which is a distinct manufactured product rather than a grade of an existing
 * surface. It is placed above the shared range precisely so it never collides
 * with a future shared entry.
 */
namespace material_variant {

/// The slot's default. Always installed for every slot; never falls back.
inline constexpr uint16_t kDefault = 0;

/// Fine, well-maintained, freshly laid. Granite for MaterialId::Curb.
inline constexpr uint16_t kSmooth = 1;

/// Coarse, weathered, patched. Worn concrete for MaterialId::Curb.
inline constexpr uint16_t kWorn = 2;

/// Rounded cobbles in wide mortar joints.
inline constexpr uint16_t kCobblestone = 3;

/// Squared setts in tight joints; flatter and more regular than cobbles.
inline constexpr uint16_t kSett = 4;

/// Rectangular paving slabs or blocks.
inline constexpr uint16_t kPavingStones = 5;

/// Blister tactile paving. MaterialId::Sidewalk only; see the block note.
inline constexpr uint16_t kTactilePaving = 6;

/// First variant number this library never assigns. Everything at or above this
/// resolves through the fallback chain.
inline constexpr uint16_t kVariantCount = 7;

} // namespace material_variant

// ============================================================================
// MaterialLibrary
// ============================================================================

/**
 * @brief Resolves MaterialKey -> MaterialDef, with a fallback chain that cannot fail
 *
 * ### The fallback chain, and why it has three links
 *
 * resolve() tries, in order:
 *   1. the exact (slot, variant) key;
 *   2. (slot, 0), the slot's default variant;
 *   3. (Default, 0);
 *   4. a static built-in that exists even before load_defaults().
 *
 * Link 2 is the one that matters. Variants are assigned by the geometry side from
 * whatever tags an extract happens to carry -- cobblestone, sett, paving stones,
 * worn asphalt, smooth asphalt, tactile paving, granite kerb -- and this library
 * has never seen most of them and never will. An unknown variant of Asphalt is
 * still asphalt, and drawing it as asphalt is right; refusing to draw it, or
 * dropping it to magenta, would make a perfectly ordinary OSM extract look broken.
 *
 * Link 3 and link 4 are the paranoid ones: a slot with no entry at all, and a
 * library that was never initialised. Neither should happen, and neither may
 * stop a draw.
 *
 * ### Keying
 *
 * The map is keyed on MaterialKey::packed(), NOT on MaterialKey itself, so no
 * std::hash specialisation has to be added to mesh.hpp. mesh.hpp is shared with
 * feat/road-optimization and is left untouched deliberately.
 */
class MaterialLibrary {
public:
    MaterialLibrary() = default;
    ~MaterialLibrary() = default;

    MaterialLibrary(const MaterialLibrary&) = delete;
    MaterialLibrary& operator=(const MaterialLibrary&) = delete;

    /**
     * @brief Attach the texture manager and prime the fallback material
     *
     * Does NOT install the per-slot defaults; call load_defaults() for that, or
     * load_from_file() to install a saved set instead.
     *
     * @param textures Initialised GPUTextureManager. Not owned; must outlive this
     *                 library. Nullptr is refused.
     * @return true on success.
     */
    bool init(GPUTextureManager* textures);

    /// Drop every material. Textures are NOT released: the manager owns them and
    /// other libraries or panels may still hold the same handles.
    void shutdown();

    /**
     * @brief Resolve a key to something drawable
     *
     * Never fails and never returns a dangling reference -- a missing material
     * must not stop a draw. See the class note for the fallback chain.
     *
     * @param key (slot, variant) to resolve
     * @return Reference to a MaterialDef owned by this library, valid until the
     *         next set(), load_defaults(), load_from_file() or shutdown().
     */
    [[nodiscard]] const MaterialDef& resolve(MaterialKey key) const;

    /**
     * @brief Install or replace one material
     *
     * @param key Key to bind it to
     * @param def The material. Validated on the way in: a Markings material with
     *            uv_scale != {1,1}, and a non-zero depth_bias with alpha_blend
     *            false, are both logged as configuration errors.
     */
    void set(MaterialKey key, MaterialDef def);

    /// Whether an EXACT entry exists for @p key. False for a key that only
    /// resolves through the fallback chain; that distinction is the whole point.
    [[nodiscard]] bool has(MaterialKey key) const;

    /**
     * @brief Install the built-in defaults for every MaterialId slot
     *
     * So roads look materially distinct with NO assets present at all. Every map
     * is left at the manager's white / flat-normal / default-ORM, so this alone
     * produces flat but correctly differentiated surfaces; call
     * install_procedural_textures() afterwards to replace the flats with
     * generated tiling detail.
     *
     * The frozen table -- these are what "asphalt" and "kerb" mean in this
     * project, and changing them changes every screenshot:
     *
     * | Slot       | base_color (linear)      | rough | metal | Sampler      | Notes |
     * |------------|--------------------------|-------|-------|--------------|-------|
     * | Default    | 0.60, 0.60, 0.60         | 0.85  | 0.0   | RepeatAniso  | untagged geometry |
     * | Asphalt    | 0.11, 0.11, 0.12         | 0.82  | 0.0   | RepeatAniso  | dark, slightly blue |
     * | Concrete   | 0.52, 0.51, 0.49         | 0.78  | 0.0   | RepeatAniso  | |
     * | Curb       | 0.62, 0.61, 0.58         | 0.70  | 0.0   | RepeatAniso  | lighter than concrete |
     * | Sidewalk   | 0.55, 0.54, 0.52         | 0.80  | 0.0   | RepeatAniso  | |
     * | Markings   | 0.95, 0.95, 0.93         | 0.55  | 0.0   | ClampLinear  | alpha_blend, depth_bias |
     * | Gravel     | 0.45, 0.42, 0.38         | 0.95  | 0.0   | RepeatAniso  | |
     * | Dirt       | 0.36, 0.28, 0.20         | 0.97  | 0.0   | RepeatAniso  | |
     * | Grass      | 0.20, 0.33, 0.14         | 0.90  | 0.0   | RepeatAniso  | |
     * | BridgeDeck | 0.46, 0.46, 0.45         | 0.75  | 0.0   | RepeatAniso  | greyer concrete |
     * | Parapet    | 0.58, 0.58, 0.57         | 0.65  | 0.0   | RepeatAniso  | |
     *
     * Markings is the only slot that differs structurally: alpha_blend = true,
     * depth_bias = kMarkingDepthBias, sampler = ClampLinear, uv_scale = {1,1}.
     *
     * ao is 1.0 and emissive 0.0 everywhere; nothing on a road is emissive and
     * nothing here has baked occlusion.
     */
    void load_defaults();

    /**
     * @brief Install the built-in table of VARIANTS, on top of the slot defaults
     *
     * Separate from load_defaults() on purpose, and the separation is load-bearing
     * in two directions.
     *
     * Downward: load_defaults() is the frozen slot table and nothing else. A test
     * that asserts "variant 1 of Asphalt does not exist, so it falls back to
     * variant 0" is asserting the fallback chain, which is the single most
     * important behaviour in this file. Folding speculative variants into
     * load_defaults() would make that assertion untestable -- there would be no
     * variant number left that is reliably absent.
     *
     * Upward: the variants have to exist somewhere, or a cobbled street imported
     * the day feat/road-optimization merges renders as plain asphalt. So they live
     * here, and install_procedural_textures() calls this itself, because a variant
     * without its stone texture is only a slightly different shade of grey and is
     * not worth installing on its own.
     *
     * @warning The variant NUMBERS are an assumption. See the material_variant
     *          namespace for the table and for what to reconcile at merge time.
     *
     * Calls set() per entry, so this replaces rather than accumulates and is safe
     * to call twice. It does NOT install slot defaults; call load_defaults() first.
     */
    void load_variant_defaults();

    /**
     * @brief Replace the built-in flat colours with generated tiling textures
     *
     * No texture assets exist in this repository, so rather than ship a grey world
     * the surface materials get procedurally generated albedo, normal and ORM maps
     * from procedural_texture.hpp, plus the drawn markings atlas.
     *
     * Generation is deterministic in @p seed, so two runs produce byte-identical
     * textures and a golden test can assert on them. Uploads go through
     * GPUTextureManager's normal staged path, so this is safe to call before the
     * first frame and the textures simply become ready over the first few frames.
     *
     * Must be called after load_defaults(); it edits the materials that call
     * installed, leaving base_color and the PBR scalars alone.
     *
     * @param texture_size Side of each generated surface texture, in pixels.
     *                     Powers of two only. The markings atlas ignores this and
     *                     always uses osm::road::kAtlasSizePixels.
     * @param seed         Master seed. Each generator derives its own from it, so
     *                     asphalt and gravel do not share a noise field.
     * @return true when every texture generated and staged.
     */
    bool install_procedural_textures(uint32_t texture_size = 512, uint32_t seed = 1337);

    /**
     * @brief Load a material set as JSON
     *
     * nlohmann/json is vendored and already linked. Texture paths are stored
     * RELATIVE TO THE SET FILE, so a set and its textures move together; they are
     * resolved against @p path's parent directory before being handed to
     * GPUTextureManager::load().
     *
     * Schema:
     * @code
     * {
     *   "version": 1,
     *   "materials": [
     *     {
     *       "slot": "Asphalt",          // MaterialId name, per material_id_name()
     *       "variant": 0,               // uint16; 0 is the slot default
     *       "name": "Asphalt",
     *       "base_color": [0.11, 0.11, 0.12, 1.0],
     *       "metallic": 0.0,
     *       "roughness": 0.82,
     *       "ao": 1.0,
     *       "emissive": 0.0,
     *       "uv_scale": [1.0, 1.0],
     *       "albedo": "textures/asphalt_albedo.ktx2",   // optional, relative
     *       "normal": "textures/asphalt_normal.ktx2",   // optional, LINEAR
     *       "orm":    "textures/asphalt_orm.ktx2",      // optional, LINEAR
     *       "sampler": "RepeatAniso",
     *       "alpha_blend": false,
     *       "depth_bias": 0.0
     *     }
     *   ]
     * }
     * @endcode
     *
     * An unknown "slot" string, an unparseable entry, or a texture that fails to
     * load, skips THAT FIELD OR ENTRY and is logged; the rest of the set still
     * loads. A partially valid set is more useful than none, and the log names
     * exactly what was dropped.
     *
     * @param path Set file to read
     * @return true when the file parsed and at least one material was installed.
     */
    bool load_from_file(const std::filesystem::path& path);

    /**
     * @brief Save the current set as JSON, in the schema load_from_file() reads
     *
     * Texture paths are written relative to @p path's parent directory. A
     * PROCEDURAL texture has no path: its field is omitted, and the material
     * reloads with that map unbound. Round-tripping a procedural set therefore
     * loses the generated maps by design -- they are regenerated, not stored.
     *
     * @param path Destination. Parent directories are created.
     * @return true on success.
     */
    [[nodiscard]] bool save_to_file(const std::filesystem::path& path) const;

    /**
     * @brief Which of a material's three maps a call refers to
     *
     * Exists only so load_map_from_file() does not need three near-identical
     * overloads. The ordering matches the sampler slots in mesh_pbr.frag.
     */
    enum class TextureMap : uint8_t {
        Albedo = 0, ///< sRGB colour
        Normal,     ///< Linear tangent-space normal
        Orm         ///< Linear occlusion / roughness / metallic
    };

    /**
     * @brief Load an image from disk and bind it into one map of one material
     *
     * @par Why this is not just textures()->load() at the call site
     * It could be, and the material would look right, and the next save_to_file()
     * would silently drop the texture. save_to_file() writes a path only for
     * handles present in the private m_texture_paths map -- that is how it
     * distinguishes a loaded texture from a procedurally generated one, which has
     * no path and is deliberately regenerated rather than stored. A texture the
     * editor loaded through GPUTextureManager directly is absent from that map and
     * is therefore indistinguishable from a generated one, so it would round-trip
     * to nothing. Going through here is what records the path.
     *
     * sRGB is chosen from @p map, not by the caller: albedo is colour and the other
     * two are data, and getting that backwards is a mistake that looks like a
     * lighting bug rather than like a load bug.
     *
     * On failure the material is left completely untouched, so a mistyped path
     * costs nothing. The previous texture in that map is NOT released on success:
     * a handle may be shared with another material -- eight surface materials
     * commonly share one default ORM -- and this library cannot know.
     *
     * @param key  Material to edit. An entry is created if none existed, seeded
     *             from whatever resolve() would have returned for it, so loading a
     *             texture onto a fallback-resolved key promotes it to a real one.
     * @param map  Which of the three maps to replace
     * @param path Image file. Anything GPUTextureManager::load() accepts.
     * @return true when the image loaded and the material was updated.
     */
    bool load_map_from_file(MaterialKey key, TextureMap map,
                            const std::filesystem::path& path);

    /// The texture manager passed to init(), or nullptr. The editor needs it to
    /// display a material's albedo and to report load failures.
    [[nodiscard]] GPUTextureManager* textures() const { return m_textures; }

    /**
     * @brief Source path a map was loaded from, or an empty path
     *
     * Empty for a procedurally generated map and for an unbound one -- the same
     * distinction save_to_file() makes. The editor shows it so "no path" is
     * visible before the save rather than after it.
     */
    [[nodiscard]] std::filesystem::path texture_source_path(TextureHandle handle) const;

    /**
     * @brief Every key with an exact entry
     * @return Keys in ascending MaterialKey::packed() order, so the listing is
     *         stable across runs and diffable in tests.
     */
    [[nodiscard]] std::vector<MaterialKey> keys() const;

    /**
     * @brief The uniform block for a key, ready to push
     *
     * resolve() followed by the field-to-vector packing. Uses the same fallback
     * chain, so this never fails either.
     */
    [[nodiscard]] MaterialUniforms uniforms_for(MaterialKey key) const;

    /// Materials with an exact entry.
    [[nodiscard]] size_t size() const;

    // === Fallback accounting ===

    /**
     * @brief How resolve() has been answering
     *
     * A whole variant set going missing -- because the geometry side renumbered,
     * or because a set file was loaded that predates a slot -- does not fail, does
     * not warn per draw, and does not look broken: every affected surface quietly
     * draws as its slot default. That is the correct behaviour and it is also
     * completely invisible, which is why it is counted. The editor renders
     * @ref ResolveStats::fallback_keys as "N materials resolved by fallback", and
     * that number going from 0 to 12 after a merge is how someone finds out.
     */
    struct ResolveStats {
        uint64_t resolves = 0;          ///< resolve() and uniforms_for() calls
        uint64_t exact = 0;             ///< Answered by link 1, the exact key
        uint64_t variant_fallbacks = 0; ///< Answered by link 2, (slot, 0)
        uint64_t slot_fallbacks = 0;    ///< Answered by link 3, (Default, 0)
        uint64_t hard_fallbacks = 0;    ///< Answered by link 4, the built-in

        /// Distinct keys that have EVER fallen back. This is the number to show a
        /// user: the call counts scale with frame rate and mesh count and say
        /// nothing, whereas "12 distinct materials are missing" is actionable.
        size_t fallback_keys = 0;
    };

    /// Live resolve accounting. Cheap; updated on every resolve().
    [[nodiscard]] const ResolveStats& resolve_stats() const;

    /**
     * @brief Every distinct key that resolve() could not answer exactly
     *
     * In ascending MaterialKey::packed() order, so the listing is stable across
     * runs and diffable in a test. This is the list to print when
     * ResolveStats::fallback_keys is non-zero.
     */
    [[nodiscard]] std::vector<MaterialKey> fallback_keys() const;

    /// Forget the accounting, including the distinct-key set. Does not touch the
    /// materials. Call after installing a set, so the counts describe the set in
    /// use rather than the one before it.
    void reset_resolve_stats();

    /**
     * @brief Depth bias applied to road markings
     *
     * Chosen against the project's depth range rather than picked by feel: the
     * scene is Y-up metres over a city-sized extent, so a constant-factor bias is
     * the right tool and a slope-scaled one alone is not -- marking quads are
     * coplanar with the carriageway, so their slope relative to it is zero and a
     * purely slope-scaled bias would do nothing at exactly the angles that
     * z-fight.
     */
    static constexpr float kMarkingDepthBias = -2.0f;

    /**
     * @brief Mip levels the generated markings atlas is created with
     *
     * Deliberately SHORT rather than a full chain. marking_atlas.hpp insets every
     * sprite rect by kAtlasInsetPixels = 1 px at 1024 px, which is one texel of
     * guard band; halving the atlas halves the guard band with it, so by level 4
     * there is a sixteenth of a texel left and a distant dashed line starts
     * sampling the arrow in the next block. Four levels take a 1024 px atlas down
     * to 128 px -- far enough to stop the shimmer an unmipped atlas produces at
     * grazing angles, and not far enough to bleed. A real fix is a mip chain
     * generated per sub-rect, which is a texture-tool job, not a renderer one.
     */
    static constexpr uint32_t kMarkingAtlasMipLevels = 4;

private:
    GPUTextureManager* m_textures = nullptr;

    /// Keyed on MaterialKey::packed(); see the class note on why not on the key.
    std::unordered_map<uint32_t, MaterialDef> m_materials;

    /// Link 4 of the fallback chain: valid before init(), never removed.
    /// Brace-initialised so it carries a name even before init() has run -- an
    /// editor listing a nameless material is indistinguishable from a bug.
    MaterialDef m_fallback{ "<fallback>" };

    /**
     * @brief Absolute source path of every texture this library loaded from disk
     *
     * Keyed on TextureHandle. save_to_file() needs a path to write and MaterialDef
     * deliberately does not carry one -- it is a RENDER-time structure and a path
     * is a LOAD-time fact. A handle absent from this map is procedural, and
     * save_to_file() omits its field, which is the documented round-trip
     * behaviour: generated maps are regenerated, never stored.
     *
     * Paths are stored ABSOLUTE and made relative at save time, because the set
     * may be saved to a different directory than it was loaded from.
     */
    std::unordered_map<TextureHandle, std::filesystem::path> m_texture_paths;

    /// See ResolveStats. Mutable because resolve() is const and must stay const:
    /// it is called from the render loop and counting is not a state change any
    /// caller can observe through the returned MaterialDef.
    mutable ResolveStats m_resolve_stats;

    /// Distinct packed keys that have fallen back, and how often. Also serves as
    /// the "already warned about this one" set, so a missing material logs once
    /// rather than once per draw per frame.
    mutable std::unordered_map<uint32_t, uint64_t> m_fallback_keys;
};

// ============================================================================
// Free helpers
// ============================================================================

/**
 * @brief Parse a MaterialId from the name material_id_name() produces
 *
 * The inverse of mesh.hpp's material_id_name(), needed by load_from_file().
 * Declared HERE and not in mesh.hpp on purpose: mesh.hpp is shared with
 * feat/road-optimization and is not being touched by this branch.
 *
 * @param name Slot name, case-sensitive, exactly as material_id_name() writes it
 * @param out  Set to the parsed slot on success; untouched on failure
 * @return true when @p name named a real slot. "Unknown" and "Count" are failures.
 */
[[nodiscard]] bool material_id_from_name(std::string_view name, MaterialId& out);

/**
 * @brief Parse a SamplerKind from the name sampler_kind_name() produces
 * @param name Sampler name, case-sensitive
 * @param out  Set on success; untouched on failure
 * @return true when @p name named a real kind
 */
[[nodiscard]] bool sampler_kind_from_name(std::string_view name, SamplerKind& out);

/**
 * @brief Metres one tile of a material's texture covers, per the plan's UV Convention
 *
 * The geometry side already divides by these when it writes UVs, so nothing in the
 * renderer needs them to DRAW. They are here so the editor can display the
 * effective texel density of a material, and so a test can assert that a
 * material's uv_scale and its texture's resolution agree with the convention.
 *
 * Frozen values, from docs/plans/road_network_plan.md:
 * Asphalt 8/8, Concrete 4/4, Sidewalk 2/2, Curb 0.5/2, Gravel/Dirt/Grass 4/4.
 * BridgeDeck follows Concrete and Parapet follows Concrete; Markings is atlased
 * and has no tile size, reported as {0, 0}.
 *
 * @param material Slot to look up
 * @return {tile_u_metres, tile_v_metres}; {0,0} for Markings and out-of-range
 */
[[nodiscard]] glm::vec2 material_tile_metres(MaterialId material);

} // namespace stratum
