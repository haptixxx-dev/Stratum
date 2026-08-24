/**
 * @file procedural_texture.hpp
 * @brief Generates the surface textures this repository does not ship
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * There are no texture assets in this project and there is no marking texture for
 * the atlas that marking_atlas.hpp specifies. The alternative to generating them
 * is a grey world, which is not a neutral choice: a uniformly grey city hides
 * every UV bug the P2 arc-length work could have, and the UV Convention has never
 * once been looked at through a texture. Generated textures are not final art,
 * but they make texel density, tiling seams, kerb face orientation and marking
 * sprite orientation VISIBLE, which is what this phase is for.
 *
 * ### Contract every generator obeys
 *
 * - **Pure.** No SDL, no device, no global state, no file I/O. Safe to call from
 *   any thread, before a GPU device exists.
 * - **Deterministic.** The same (size, seed, parameters) produce byte-identical
 *   pixels on every run and every platform. Use an explicit integer hash seeded
 *   from @p seed and the texel coordinate; do NOT use std::rand, and do not use
 *   std::uniform_real_distribution, whose output is implementation-defined. This
 *   is what lets a golden test hash the output.
 * - **Tileable.** Every surface generator produces a texture that tiles seamlessly
 *   with itself in BOTH axes. The whole point of the metres-based UV convention is
 *   that one texture repeats across a whole road network; a generator with a visible
 *   seam every 8 m is worse than flat colour. Use periodic noise -- wrap the
 *   lattice coordinates modulo the texture size -- not a windowed crossfade.
 * - **Albedo carries HEIGHT IN ALPHA.** Every surface generator writes its height
 *   field, linearly encoded 0..255, into the alpha channel of its RGBA output.
 *   That is what make_normal_from_height() consumes, and it means a caller gets a
 *   matching albedo and normal from one generation pass instead of two. The alpha
 *   channel of an RGBA8_SRGB texture is not sRGB-encoded, so the height survives
 *   the format unchanged. The MARKINGS ATLAS is the exception: its alpha is paint
 *   coverage, and it has no height field.
 * - **Size** must be a power of two and at least 4. Anything else returns an empty
 *   result rather than guessing.
 *
 * ### Formats
 *
 * | Generator                  | Format      | Channels                          |
 * |----------------------------|-------------|-----------------------------------|
 * | make_asphalt .. make_kerb  | RGBA8_SRGB  | rgb albedo, a height              |
 * | make_normal_from_height    | RGBA8       | rgb encoded normal, a 255         |
 * | make_orm                   | RGBA8       | r occlusion, g roughness, b metallic, a 255 |
 * | make_markings_atlas        | RGBA8_SRGB  | rgb paint colour, a coverage      |
 *
 * The normal and ORM outputs are LINEAR. An sRGB normal map is wrong in a way
 * that looks merely "a bit off", which is why the format is stated per generator
 * rather than left to the caller.
 *
 * This header lives in src/renderer and is part of stratum_editor_lib, because
 * TextureDesc comes from texture.hpp which includes SDL. The generation code
 * itself uses nothing from SDL and could move to stratum_core if TextureDesc
 * ever did.
 */

#pragma once

#include "renderer/texture.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

namespace stratum {

/**
 * @brief A generated texture: its pixels and how to interpret them
 *
 * `pixels` holds level 0 only, tightly packed, `desc.level0_bytes()` long. Mip
 * levels are generated on the GPU by GPUTextureManager, not here.
 */
struct ProcTexResult {
    std::vector<uint8_t> pixels;    ///< Level 0, tightly packed
    TextureDesc desc{};             ///< Dimensions and format of @ref pixels

    /// True when generation succeeded and the buffer matches the description.
    [[nodiscard]] bool is_valid() const {
        return desc.is_valid() && !pixels.empty() && pixels.size() == desc.level0_bytes();
    }
};

// ============================================================================
// Feature periods
// ============================================================================

/**
 * @brief Clamp a feature period to the range a texture of this size can carry
 *
 * Every generator picks its lattice and grain periods in TEXELS from a constant
 * authored for a 512-texel tile, then caps them at some fraction of the actual
 * size -- one aggregate cell per four texels, one paving cell per eight, and so
 * on -- because a period finer than that stops being a feature and starts being
 * aliasing.
 *
 * @warning The obvious spelling of that cap is
 *          `std::clamp(desired, min_period, int(size / divisor))`, and it is
 *          UNDEFINED BEHAVIOUR at the smallest sizes the header itself declares
 *          valid. procedural_texture.hpp documents "power of two, >= 4",
 *          valid_surface_size() accepts 4, and MaterialLibrary::install_procedural_textures()
 *          validates against a floor of 4 -- but at size 4 with divisor 4 the
 *          upper bound is 1, BELOW the lower bound of 4. std::clamp's precondition
 *          is `!(hi < lo)`; violating it aborts on a hardened libstdc++
 *          (`-D_GLIBCXX_ASSERTIONS`: "stl_algo.h: Assertion !(__hi < __lo) failed")
 *          and, in an unhardened build like this project's, silently returns the
 *          INVERTED bound -- collapsing a whole noise layer to period 1, i.e. to a
 *          constant, with no diagnostic anywhere.
 *
 * This is the one place that arithmetic lives, so the guard cannot be applied to
 * five of six call sites again.
 *
 * @param desired    Period the generator would like, in texels
 * @param min_period Smallest period that still reads as the feature. Must be >= 1.
 * @param size       Texture size in texels
 * @param divisor    Texels per feature cell the cap allows. Must be >= 1.
 * @return @p desired clamped into [min_period, max(min_period, size / divisor)]
 */
[[nodiscard]] constexpr int feature_period(int desired, int min_period, uint32_t size,
                                           uint32_t divisor) {
    const int allowed = (divisor == 0u) ? min_period : static_cast<int>(size / divisor);
    const int hi = (allowed > min_period) ? allowed : min_period;
    return (desired < min_period) ? min_period : ((desired > hi) ? hi : desired);
}

// ============================================================================
// Tileability
// ============================================================================

/**
 * @brief Seam check: does this texture wrap without a discontinuity?
 *
 * ### Why this is not a byte comparison of opposite edges
 *
 * The obvious test -- "column 0 must equal column width - 1" -- is not merely
 * weak, it is WRONG, and a generator written to pass it would be worse than one
 * that fails it. A periodic field sampled at texel centres puts column 0 at
 * u = 0.5 / width and the last column at u = (width - 0.5) / width. Those are the
 * two ENDS of one period, one texel apart across the wrap, and they are supposed
 * to differ by exactly as much as any other pair of neighbouring columns. Forcing
 * them equal duplicates a row of texels at every tile boundary, which is itself a
 * visible artefact.
 *
 * What must hold instead is CONTINUITY: the step across the wrap must be
 * unremarkable compared with the steps around it. Two measures run per axis, and
 * BOTH must find the wrap pair unremarkable:
 *
 *  - the mean ABSOLUTE difference between adjacent lines, which catches a
 *    discontinuity whose sign varies along the seam -- a noise lattice that does
 *    not wrap looks different on each side in a different direction at every texel;
 *  - the difference of the LINE MEANS, which catches a smooth offset across the
 *    wrap that the first measure buries in the texture's own grain.
 *
 * Each is judged against the local neighbourhood of the wrap AND against the
 * spread of the whole interior population, and passing either is enough. That
 * two-sided reference is not decoration. A paving or cobble lattice puts a cell
 * boundary at u = 0 by construction, so the wrap pair straddles a MORTAR JOINT and
 * is legitimately several times the texture's average -- only the local reference
 * forgives that. A small texture's line averages are a noisy estimate and a local
 * median of eight of them is noisier still -- only the distributional reference
 * forgives that. Judged by either one alone, this rejects textures that tile
 * perfectly.
 *
 * ### What it will and will not see
 *
 * Measured by injecting a linear brightness ramp, which is the shape a
 * non-periodic layer actually produces, the smallest seam detected is about
 * 4/255 on a 1024 asphalt, 6/255 at 512 and 12/255 at 256; a lattice texture like
 * paving, whose own joints raise the bar, needs roughly 20/255 at 1024. So this is
 * a guard against a MISTAKE -- a clamped gradient, a feature placed at an
 * unwrapped distance -- which produces a seam far larger than any of those. It is
 * not a certificate that a texture is seamless, and the exact half of that claim
 * lives in the generators, not here.
 *
 * The EXACT half of the proof lives in the generators, not here: every periodic
 * primitive reduces its lattice coordinates with an integer modulo before hashing,
 * so the underlying field is periodic by construction rather than by measurement.
 * A debug build asserts that directly on the primitives, and asserts this seam
 * check on every surface it returns.
 *
 * @param tex     Texture to check. An invalid result returns false.
 * @param check_u true to require the vertical (left/right) seam to be continuous
 * @param check_v true to require the horizontal (top/bottom) seam to be continuous
 * @return true when every requested seam is continuous
 *
 * @note make_kerb() is the one surface that must be called with @p check_u false.
 *       Its two halves in U are the curb FACE and the curb TOP, which are
 *       deliberately different surfaces; nothing in the corridor builder tiles a
 *       curb laterally, so the U seam is meaningless rather than broken.
 */
[[nodiscard]] bool is_tileable(const ProcTexResult& tex, bool check_u = true, bool check_v = true);

// ============================================================================
// Surface generators
// ============================================================================

/**
 * @brief Bituminous road surface: dark aggregate in a darker binder
 *
 * Two scales of value noise -- fine aggregate grain and a broad patchiness that
 * reads as wear and repair -- over a near-black base. Height is the aggregate.
 *
 * Authored to tile at 8 m, matching MaterialId::Asphalt's tile_u/tile_v of 8.0,
 * so a material using it needs uv_scale {1,1}.
 *
 * @param size       Side in texels. Power of two, >= 4.
 * @param seed       Deterministic seed.
 * @param coarseness 0 = fine, smooth, freshly laid; 1 = coarse chippings and
 *                   heavy patch variation. Clamped to [0, 1]. This is the axis
 *                   the surface=asphalt variants differ along, so a "worn" and a
 *                   "smooth" variant are the same generator at two values.
 * @return The texture, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_asphalt(uint32_t size, uint32_t seed, float coarseness);

/**
 * @brief Poured concrete: pale, faintly mottled, with a fine surface tooth
 *
 * Tiles at 4 m, matching MaterialId::Concrete. Also the basis for BridgeDeck and
 * Parapet, which differ only in base_color.
 *
 * @param size Side in texels. Power of two, >= 4.
 * @param seed Deterministic seed.
 * @return The texture, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_concrete(uint32_t size, uint32_t seed);

/**
 * @brief Rounded setts in mortar joints
 *
 * A Worley/cellular field gives the stones; the cell distance gives both the
 * joint darkening in albedo and the dome in height, so the normal map derived
 * from it produces stones that catch light individually. The cell lattice must
 * WRAP so the result tiles.
 *
 * The generator behind the cobblestone, sett and paving-stone variants of
 * MaterialId::Asphalt and MaterialId::Sidewalk that feat/road-optimization
 * derives from surface= tags.
 *
 * @param size       Side in texels. Power of two, >= 4.
 * @param seed       Deterministic seed.
 * @param stone_size Stone diameter as a FRACTION OF THE TEXTURE, not metres --
 *                   this function knows nothing about the world scale the texture
 *                   is tiled at. 0.08 gives roughly twelve stones across, which
 *                   at Asphalt's 8 m tile is a 0.65 m sett and at Sidewalk's 2 m
 *                   tile is a 0.16 m one. Clamped to [0.02, 0.5].
 * @return The texture, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_cobblestone(uint32_t size, uint32_t seed, float stone_size);

/**
 * @brief Rectangular paving slabs with recessed joints
 *
 * The sidewalk default. Unlike cobblestone the lattice is a regular grid, so the
 * joint lines must land on exact texel boundaries or the tiling seam shows as a
 * joint of the wrong width -- round the slab count to a whole number of slabs per
 * tile and derive the slab size from that, rather than the other way round.
 *
 * @param size      Side in texels. Power of two, >= 4.
 * @param seed      Deterministic seed; drives per-slab value variation only.
 * @param slab_size Slab width and height as a FRACTION OF THE TEXTURE. {0.5, 0.25}
 *                  is two slabs across and four along, which at Sidewalk's 2 m tile
 *                  is the 1.0 x 0.5 m unit the plan's "paving unit scale" note
 *                  means. Each component clamped to [0.05, 1.0] and then rounded
 *                  to the nearest whole division.
 * @return The texture, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_paving(uint32_t size, uint32_t seed, glm::vec2 slab_size);

/**
 * @brief Loose graded gravel: many small stones, high frequency, high roughness
 * @param size Side in texels. Power of two, >= 4.
 * @param seed Deterministic seed.
 * @return The texture, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_gravel(uint32_t size, uint32_t seed);

/**
 * @brief Compacted earth: brown, low contrast, broad undulation, no distinct grains
 * @param size Side in texels. Power of two, >= 4.
 * @param seed Deterministic seed.
 * @return The texture, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_dirt(uint32_t size, uint32_t seed);

/**
 * @brief Verge and median planting, seen from above at map-editor distances
 *
 * Clumped value noise in green, not individual blades: at the camera distances
 * this editor uses, a blade is far below a texel, and drawing blades produces
 * shimmering rather than grass. Height is the clump variation.
 *
 * @param size Side in texels. Power of two, >= 4.
 * @param seed Deterministic seed.
 * @return The texture, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_grass(uint32_t size, uint32_t seed);

/**
 * @brief Kerb stone, authored for the two-sided Curb convention
 *
 * MaterialId::Curb is the one surface material whose texture is not uniform,
 * because the plan splits it across the axis: the curb TOP strip uses the normal
 * lateral convention, while the curb FACE is vertical and runs U UP THE FACE,
 * with tile_u 0.5 m and tile_v 2.0 m. Both share one material and therefore one
 * texture, "which must be authored with the face on one side and the top on the
 * other".
 *
 * This generator honours that: the LEFT HALF in U (u in [0, 0.5)) is the face --
 * a cleaner, more vertical stone with a chamfer highlight near u = 0.5 -- and the
 * RIGHT HALF (u in [0.5, 1)) is the top, which is dirtier and flatter. Because
 * the halves differ, the texture tiles seamlessly in V but MUST NOT be relied on
 * to tile in U; nothing in the corridor builder tiles a kerb laterally.
 *
 * @param size Side in texels. Power of two, >= 4.
 * @param seed Deterministic seed.
 * @return The texture, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_kerb(uint32_t size, uint32_t seed);

// ============================================================================
// Derived maps
// ============================================================================

/**
 * @brief Tangent-space normal map from a height field
 *
 * Sobel gradient of the height, encoded as (x, y, z) * 0.5 + 0.5 into RGB with
 * alpha 255. The output is RGBA8, LINEAR -- never sRGB.
 *
 * The gradient must WRAP at the edges, sampling texel (width - 1) as the
 * neighbour of texel 0. A clamped gradient produces a flat one-texel border,
 * which becomes a visible hard line at every tile boundary across the network --
 * exactly the seam the tileability contract exists to prevent.
 *
 * Sign convention: +Y in the normal map corresponds to +V in UV space, matching
 * mesh_pbr.vert's `frag_bitangent = cross(frag_normal, frag_tangent) * in_tangent.w`
 * and Mesh::compute_tangents(). Get this backwards and every surface lights as if
 * lit from the opposite side, which is subtle enough to ship by accident: check it
 * against make_kerb(), whose chamfer must catch the sun on the upward-facing edge.
 *
 * @param height   A generator's output. The height is read from the ALPHA channel
 *                 for an RGBA8 or RGBA8_SRGB input, and from the single channel of
 *                 an R8 input. Any other format returns an empty result.
 * @param strength Gradient multiplier. 1.0 is a natural relief for these
 *                 generators; below 0 is clamped to 0, which yields a flat normal.
 * @return The normal map, or an empty result when @p height is not valid.
 */
[[nodiscard]] ProcTexResult make_normal_from_height(const ProcTexResult& height, float strength);

/**
 * @brief A uniform ORM pack
 *
 * r = occlusion, g = roughness, b = metallic, a = 255. RGBA8, LINEAR.
 *
 * The shader multiplies these by the material's scalars, so a uniform pack is
 * mathematically identical to no pack at all; it exists so the sampler binding is
 * always populated and mesh_pbr.frag needs no branch. A per-material ORM with real
 * variation is a later refinement that changes nothing about the binding.
 *
 * @param size      Side in texels. Power of two, >= 1. A 1x1 result is legal and
 *                  is what GPUTextureManager::default_orm() uses.
 * @param ao        Occlusion, clamped to [0, 1]
 * @param roughness Roughness, clamped to [0, 1]
 * @param metallic  Metallic, clamped to [0, 1]
 * @return The pack, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_orm(float ao, float roughness, float metallic, uint32_t size);

// ============================================================================
// The markings atlas
// ============================================================================

/**
 * @brief Draw the marking sprite table that osm/road/marking_atlas.hpp specifies
 *
 * Dashes, arrows, stop lines, zebra stripes, give-way triangles, hatching and the
 * lane pictograms, drawn procedurally into one RGBA8_SRGB texture with straight
 * (non-premultiplied) alpha. The background is fully transparent BLACK (0,0,0,0),
 * never opaque black: an opaque background darkens every sprite edge as soon as
 * mips are generated, which the atlas header calls out explicitly.
 *
 * ### The rects must agree by construction, not by transcription
 *
 * Do not re-type the pixel layout from marking_atlas.hpp's comment block. Derive
 * every block from osm::road::sprite_rect(), which is the authority, and undo its
 * inset to recover the block the artist would paint edge to edge:
 *
 * @code
 * const float px = float(size) / float(osm::road::kAtlasSizePixels);
 * const auto  r  = osm::road::sprite_rect(s);
 * const float x0 = r.u0 * float(size) - osm::road::kAtlasInsetPixels * px;
 * const float y0 = r.v0 * float(size) - osm::road::kAtlasInsetPixels * px;
 * const float x1 = r.u1 * float(size) + osm::road::kAtlasInsetPixels * px;
 * const float y1 = r.v1 * float(size) + osm::road::kAtlasInsetPixels * px;
 * @endcode
 *
 * Sprites that must read as continuous when repeated -- SolidWhite and
 * SolidYellow along their length, StopLine and ZebraStripe across their width,
 * BoxJunctionHatch on all four sides -- are painted to the BLOCK edge, so the
 * inset rect the emitter samples lands strictly inside the paint. Everything else
 * is painted inside the inset with transparent margin.
 *
 * ### Orientation, which is the thing this will get wrong
 *
 * From marking_atlas.hpp: direction of travel points UP the image, left-of-travel
 * is towards the image's LEFT edge, and an arrow's tip touches the TOP of its
 * block. GiveWayTriangles has its BASE at the top and its apex pointing DOWN.
 * ArrowLeft and ArrowRight must be exact mirrors of each other. Both mistakes the
 * convention exists to catch are invisible on a symmetric sprite, so verify
 * against those three and not against DashWhite.
 *
 * Paint colour is white (0.95, 0.95, 0.93) except SolidYellow, DashedYellow and
 * DoubleSolidYellow, which are (0.90, 0.72, 0.12).
 *
 * @param size Side in texels. Power of two, >= 256 -- below that a 64 px cell is
 *             under 16 px and the arrows are unreadable. Passing
 *             osm::road::kAtlasSizePixels (1024) is the intended call and makes
 *             the pixel layout exact rather than resampled.
 * @return The atlas, or an empty result when @p size is invalid.
 */
[[nodiscard]] ProcTexResult make_markings_atlas(uint32_t size);

} // namespace stratum
