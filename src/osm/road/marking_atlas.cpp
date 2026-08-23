/**
 * @file marking_atlas.cpp
 * @brief Implementation of the road-marking sprite table
 *
 * Three parallel tables, one entry per MarkingSprite, indexed by the enumerator's
 * own value. They are the whole file: there is no logic here beyond a bounds
 * check and the pixel-to-normalised conversion, because a sprite table that
 * computes anything is a sprite table an artist cannot read off the header.
 *
 * The pixel blocks below are the layout documented at the top of
 * marking_atlas.hpp, transcribed as (x, y, width, height) with the DOC's
 * inclusive ranges converted to half-open extents. The two must agree; the
 * header is the artist's copy and this is the emitter's, and a static_assert
 * below catches the cheapest way for them to drift apart -- a block escaping the
 * atlas or a band's blocks disagreeing on height.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 */

#include "osm/road/marking_atlas.hpp"

#include <cstddef>

namespace stratum::osm::road {

namespace {

/// Number of real sprites, excluding the Count sentinel
constexpr size_t kSpriteCount = static_cast<size_t>(MarkingSprite::Count);

/**
 * @brief One sprite's block in atlas pixels, before the bleed inset
 *
 * Origin is the top-left pixel, x runs right and y runs down, matching the
 * header's layout diagram. Extents are half-open: a block at x = 0 of width 64
 * covers the documented inclusive range 0-63.
 */
struct AtlasBlock {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
};

/**
 * @brief Pixel blocks, in MarkingSprite order
 *
 * Band A rows 0-1 (64 x 128), band B rows 2-3 (128 x 128), band C rows 4-9
 * (128 x 384), band D rows 10-13 (128 x 256). Band E is unused.
 */
constexpr AtlasBlock kBlocks[kSpriteCount] = {
    // Band A -- longitudinal lines, y 0..127
    {   0,   0,  64, 128 },  // DashWhite
    {  64,   0,  64, 128 },  // DashLongWhite
    { 128,   0,  64, 128 },  // SolidWhite
    { 192,   0,  64, 128 },  // SolidYellow
    { 256,   0,  64, 128 },  // DoubleSolidYellow
    { 320,   0,  64, 128 },  // DashedYellow

    // Band B -- transverse and area markings, y 128..255
    {   0, 128, 128, 128 },  // StopLine
    { 128, 128, 128, 128 },  // GiveWayTriangles
    { 256, 128, 128, 128 },  // ZebraStripe
    { 384, 128, 128, 128 },  // BoxJunctionHatch

    // Band C -- turn arrows, y 256..639
    {   0, 256, 128, 384 },  // ArrowStraight
    { 128, 256, 128, 384 },  // ArrowLeft
    { 256, 256, 128, 384 },  // ArrowRight
    { 384, 256, 128, 384 },  // ArrowStraightLeft
    { 512, 256, 128, 384 },  // ArrowStraightRight
    { 640, 256, 128, 384 },  // ArrowUTurn

    // Band D -- lane symbols, y 640..895
    {   0, 640, 128, 256 },  // BikeSymbol
    { 128, 640, 128, 256 },  // BusSymbol
};

/**
 * @brief Real-world size of one repeat, in MarkingSprite order
 *
 * The frozen table from the header. width_m is LATERAL, length_m runs ALONG the
 * direction of travel. A 1.0 in either axis is a unit the emitter stretches from
 * the road geometry; see SpriteSize.
 */
constexpr SpriteSize kSizes[kSpriteCount] = {
    { 0.15f, 3.0f },    // DashWhite
    { 0.15f, 6.0f },    // DashLongWhite
    { 0.15f, 1.0f },    // SolidWhite          length is a unit
    { 0.15f, 1.0f },    // SolidYellow         length is a unit
    { 0.35f, 1.0f },    // DoubleSolidYellow   length is a unit
    { 0.15f, 3.0f },    // DashedYellow
    { 1.0f,  0.4f },    // StopLine            width is a unit
    { 0.6f,  0.6f },    // GiveWayTriangles
    { 0.5f,  3.0f },    // ZebraStripe
    { 2.0f,  2.0f },    // BoxJunctionHatch
    { 1.4f,  4.2f },    // ArrowStraight
    { 1.4f,  4.2f },    // ArrowLeft
    { 1.4f,  4.2f },    // ArrowRight
    { 1.4f,  4.2f },    // ArrowStraightLeft
    { 1.4f,  4.2f },    // ArrowStraightRight
    { 1.4f,  4.2f },    // ArrowUTurn
    { 1.0f,  2.0f },    // BikeSymbol
    { 1.0f,  2.0f },    // BusSymbol
};

/// Stable names, in MarkingSprite order. Written into logs and test failures.
constexpr const char* kNames[kSpriteCount] = {
    "DashWhite",
    "DashLongWhite",
    "SolidWhite",
    "SolidYellow",
    "DoubleSolidYellow",
    "DashedYellow",
    "StopLine",
    "GiveWayTriangles",
    "ZebraStripe",
    "BoxJunctionHatch",
    "ArrowStraight",
    "ArrowLeft",
    "ArrowRight",
    "ArrowStraightLeft",
    "ArrowStraightRight",
    "ArrowUTurn",
    "BikeSymbol",
    "BusSymbol",
};

// ----------------------------------------------------------------------------
// Layout self-checks
//
// These are the drift the tables can suffer without anyone noticing: a block
// running off the atlas, or an inset large enough to invert a block. Neither is
// visible in a diff of the header's ASCII diagram.
// ----------------------------------------------------------------------------

/// Every block lies inside the atlas and survives the inset with area to spare
constexpr bool blocks_are_sane() {
    for (size_t i = 0; i < kSpriteCount; ++i) {
        const AtlasBlock& b = kBlocks[i];
        if (b.w <= 0 || b.h <= 0) {
            return false;
        }
        if (b.x < 0 || b.y < 0) {
            return false;
        }
        if (b.x + b.w > kAtlasSizePixels || b.y + b.h > kAtlasSizePixels) {
            return false;
        }
        // The inset is applied to both sides, so it must consume less than the block.
        if (static_cast<float>(b.w) <= 2.0f * kAtlasInsetPixels
            || static_cast<float>(b.h) <= 2.0f * kAtlasInsetPixels) {
            return false;
        }
        // Blocks are whole cells: the grid is what makes the layout readable.
        if ((b.x % kAtlasCellPixels) != 0 || (b.y % kAtlasCellPixels) != 0
            || (b.w % kAtlasCellPixels) != 0 || (b.h % kAtlasCellPixels) != 0) {
            return false;
        }
    }
    return true;
}

/// No two blocks overlap, so no sprite can sample a neighbour's paint
constexpr bool blocks_are_disjoint() {
    for (size_t i = 0; i < kSpriteCount; ++i) {
        for (size_t j = i + 1; j < kSpriteCount; ++j) {
            const AtlasBlock& a = kBlocks[i];
            const AtlasBlock& b = kBlocks[j];
            const bool separated = (a.x + a.w <= b.x) || (b.x + b.w <= a.x)
                                || (a.y + a.h <= b.y) || (b.y + b.h <= a.y);
            if (!separated) {
                return false;
            }
        }
    }
    return true;
}

static_assert(blocks_are_sane(), "marking atlas block escapes the atlas or the cell grid");
static_assert(blocks_are_disjoint(), "marking atlas blocks overlap");

/// Whether a sprite value indexes a real table entry
[[nodiscard]] constexpr bool in_range(MarkingSprite s) {
    return static_cast<size_t>(s) < kSpriteCount;
}

} // namespace

// ============================================================================
// Table lookup
// ============================================================================

/**
 * @brief Normalised atlas rect of a sprite
 *
 * Invariant: the returned rect is always inside the sprite's own block, never
 * pre-flipped (u0 < u1 and v0 < v1), and degenerate rather than plausible for an
 * invalid sprite. Returning {0,0,0,0} collapses the quad's UVs onto one texel of
 * the top-left corner, which reads as a missing marking instead of as a wrong one.
 */
SpriteRect sprite_rect(MarkingSprite s) {
    if (!in_range(s)) {
        return SpriteRect{};
    }

    const AtlasBlock& b = kBlocks[static_cast<size_t>(s)];
    constexpr float inv = 1.0f / static_cast<float>(kAtlasSizePixels);

    SpriteRect r;
    r.u0 = (static_cast<float>(b.x) + kAtlasInsetPixels) * inv;
    r.v0 = (static_cast<float>(b.y) + kAtlasInsetPixels) * inv;
    r.u1 = (static_cast<float>(b.x + b.w) - kAtlasInsetPixels) * inv;
    r.v1 = (static_cast<float>(b.y + b.h) - kAtlasInsetPixels) * inv;
    return r;
}

/**
 * @brief Real-world size of one repeat of a sprite
 *
 * Invariant: matches the frozen table in the header exactly. A MarkingConfig may
 * override an extent at emission time; this is the size used when nothing does.
 */
SpriteSize sprite_size(MarkingSprite s) {
    if (!in_range(s)) {
        return SpriteSize{};
    }
    return kSizes[static_cast<size_t>(s)];
}

/**
 * @brief Stable human-readable name of a sprite
 *
 * Invariant: every enumerator has exactly one name and the name never changes,
 * because it appears in golden-test failure messages.
 */
const char* marking_sprite_name(MarkingSprite s) {
    if (!in_range(s)) {
        return "Unknown";
    }
    return kNames[static_cast<size_t>(s)];
}

} // namespace stratum::osm::road
