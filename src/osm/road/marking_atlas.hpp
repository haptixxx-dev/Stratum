/**
 * @file marking_atlas.hpp
 * @brief The road-marking sprite table: the one atlased material in the pipeline
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Every surface material in this pipeline tiles in metres and gets its own
 * SubMesh range, so no atlas is needed for asphalt, curb, sidewalk or verge.
 * MaterialId::Markings is the single exception, frozen in the plan's UV
 * Convention: dashes, arrows, stop lines and zebra stripes are discrete sprites,
 * not tiling patterns, so marking geometry carries explicit atlas sub-rect UVs
 * instead of metre-based tiling UVs. This file is that sprite table.
 *
 * ### No texture exists yet
 *
 * There is no marking texture asset in the repository, and this header does not
 * pretend otherwise. What follows is a DATA CONTRACT: a fixed pixel layout an
 * artist authors a 1024x1024 RGBA texture against, and which the marking and
 * crossing emitters generate UVs against in the meantime. The two halves are
 * developed independently and meet at these rects. Until the texture lands,
 * marking geometry renders as untextured quads in the Markings slot, which is
 * still correct geometry in the correct material range.
 *
 * ### Atlas format
 *
 * - 1024 x 1024 pixels, RGBA8, straight (non-premultiplied) alpha.
 * - Paint is white or yellow with alpha; the background is fully transparent
 *   black (0,0,0,0), never opaque black, or mip filtering darkens every edge.
 * - Laid out on a 16 x 16 grid of 64 x 64 pixel CELLS. A sprite occupies a
 *   rectangular BLOCK of whole cells; blocks never overlap.
 * - Target texel density is roughly 64 to 91 px per metre of painted road, so a
 *   marking holds up at the camera distances a map editor uses.
 *
 * ### Orientation, and the two ways to get it wrong
 *
 * A marking quad is emitted in the frame of the road it sits on. The emitter
 * maps that frame onto the sprite rect as:
 *
 * - `u0` edge  -> the LEFT of the direction of travel
 * - `u1` edge  -> the RIGHT of the direction of travel
 * - `v0` edge  -> the DOWNSTREAM (forward) end of the quad
 * - `v1` edge  -> the UPSTREAM (trailing) end of the quad
 *
 * So the artist authors every sprite with the direction of travel pointing UP
 * the image and left-of-travel towards the image's left edge. An arrow's tip
 * touches the TOP of its block.
 *
 * Both mistakes this convention exists to catch are silent on a symmetric
 * sprite. Check against MarkingSprite::ArrowLeft and ArrowRight, which are each
 * other's mirror, and against ArrowStraight, whose tip must point downstream.
 * Note that the world mapping `(x, y) -> vec3(x, height, -y)` flips handedness,
 * so an emitter that derives u from a cross product rather than from the
 * profile's own left-to-right ordering will mirror every asymmetric sprite.
 *
 * ### Pixel layout
 *
 * Origin is the TOP-LEFT pixel; x runs right, y runs down. Ranges are inclusive.
 *
 * ```
 * BAND A -- longitudinal lines           rows 0-1     y   0- 127   blocks 64 x 128
 *   x    0- 63  DashWhite            a single dash, solid, fills the block
 *   x   64-127  DashLongWhite        a single long dash, solid
 *   x  128-191  SolidWhite           uniform white, edge to edge in y
 *   x  192-255  SolidYellow          uniform yellow, edge to edge in y
 *   x  256-319  DoubleSolidYellow    two yellow lines with a transparent gap
 *   x  320-383  DashedYellow         a single yellow dash, solid
 *   x  384-1023 FREE
 *
 * BAND B -- transverse and area markings rows 2-3     y 128- 255   blocks 128 x 128
 *   x    0-127  StopLine             uniform white, edge to edge in x
 *   x  128-255  GiveWayTriangles     ONE triangle, base at the top (downstream), apex down
 *   x  256-383  ZebraStripe          ONE stripe, uniform white, edge to edge in y
 *   x  384-511  BoxJunctionHatch     one 2 x 2 m hatch cell, diagonals meeting the block
 *                                    edges so adjacent quads read as continuous hatching
 *   x  512-1023 FREE
 *
 * BAND C -- turn arrows                  rows 4-9     y 256- 639   blocks 128 x 384
 *   x    0-127  ArrowStraight
 *   x  128-255  ArrowLeft
 *   x  256-383  ArrowRight
 *   x  384-511  ArrowStraightLeft
 *   x  512-639  ArrowStraightRight
 *   x  640-767  ArrowUTurn
 *   x  768-1023 FREE
 *
 * BAND D -- lane symbols                 rows 10-13   y 640- 895   blocks 128 x 256
 *   x    0-127  BikeSymbol
 *   x  128-255  BusSymbol
 *   x  256-1023 FREE
 *
 * BAND E -- reserved                     rows 14-15   y 896-1023   FREE, full width
 * ```
 *
 * Roughly half the atlas is deliberately unpainted. Growth goes into the free
 * run of the band it belongs to first -- another arrow into band C, another
 * area marking into band B -- so related sprites stay adjacent and a future
 * artist can find them.
 *
 * ### Bleed
 *
 * sprite_rect() returns the block INSET by kAtlasInsetPixels on all four sides,
 * so bilinear sampling can never reach into a neighbouring block. The artist
 * paints edge to edge across the whole block for any sprite that must read as
 * continuous -- SolidWhite along its length, StopLine across its width,
 * BoxJunctionHatch on all four sides -- and the inset then samples strictly
 * inside that paint rather than clipping it.
 *
 * ### Tiling
 *
 * An atlas sub-rect cannot wrap, so nothing here may be tiled by scaling UVs
 * past the rect. A marking that repeats -- a dash run, a zebra, a box junction --
 * is emitted as one QUAD PER REPEAT, each mapping the full sprite rect. That is
 * why the sprite sizes below are per-repeat sizes.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include <cstdint>

namespace stratum::osm::road {

// ============================================================================
// Atlas geometry constants
// ============================================================================

/// Width and height of the marking atlas in pixels
inline constexpr int kAtlasSizePixels = 1024;

/// Side of one grid cell in pixels
inline constexpr int kAtlasCellPixels = 64;

/// Cells per row and per column: kAtlasSizePixels / kAtlasCellPixels
inline constexpr int kAtlasGridCells = kAtlasSizePixels / kAtlasCellPixels;

/**
 * @brief Inset applied to every returned rect, in pixels per side
 *
 * One pixel is enough to keep bilinear sampling inside the block at every mip
 * level the geometry is ever seen at, and small enough that a sprite painted
 * edge to edge still reads as continuous. See the bleed note above.
 */
inline constexpr float kAtlasInsetPixels = 1.0f;

// ============================================================================
// Sprites
// ============================================================================

/**
 * @brief Every discrete painted marking the pipeline can emit
 *
 * The numeric values are the sprite table's index and are baked into the
 * authored texture's layout, so entries must not be reordered or removed once
 * an artist has drawn against them. New sprites go on the end, before Count.
 *
 * @note Count is a sentinel for array sizing and iteration bounds. It is never a
 *       valid sprite.
 */
enum class MarkingSprite : uint8_t {
    DashWhite,          ///< One white dash of a broken lane line
    DashLongWhite,      ///< One long white dash: hazard or lane-drop warning line
    SolidWhite,         ///< Uniform white; stretched along a continuous line
    SolidYellow,        ///< Uniform yellow; stretched along a continuous line
    DoubleSolidYellow,  ///< Two yellow lines and their gap, as one sprite
    DashedYellow,       ///< One yellow dash
    StopLine,           ///< Transverse bar at a signalled stop
    GiveWayTriangles,   ///< ONE give-way triangle; repeated across the lane
    ZebraStripe,        ///< ONE zebra stripe; repeated across the crossing
    BoxJunctionHatch,   ///< One hatch cell; repeated over the box junction area
    ArrowStraight,      ///< Ahead-only lane arrow
    ArrowLeft,          ///< Left-turn-only lane arrow
    ArrowRight,         ///< Right-turn-only lane arrow
    ArrowStraightLeft,  ///< Ahead or left
    ArrowStraightRight, ///< Ahead or right
    ArrowUTurn,         ///< U-turn lane arrow
    BikeSymbol,         ///< Cycle pictogram on a cycle lane
    BusSymbol,          ///< Bus pictogram on a bus lane
    Count               ///< Sentinel: number of sprites. Not a valid sprite.
};

/**
 * @brief Normalised UV sub-rect of one sprite inside the atlas
 *
 * All four values are in [0, 1]. u0 < u1 and v0 < v1 always; the rect is never
 * returned pre-flipped, because flipping is the emitter's decision and doing it
 * in two places is how a sprite ends up upside down.
 */
struct SpriteRect {
    float u0 = 0.0f;    ///< Left edge, at the LEFT of the direction of travel
    float v0 = 0.0f;    ///< Top edge, at the DOWNSTREAM end of the quad
    float u1 = 0.0f;    ///< Right edge, at the RIGHT of the direction of travel
    float v1 = 0.0f;    ///< Bottom edge, at the UPSTREAM end of the quad

    /// Normalised width of the rect
    [[nodiscard]] float du() const { return u1 - u0; }

    /// Normalised height of the rect
    [[nodiscard]] float dv() const { return v1 - v0; }
};

/**
 * @brief Real-world size one repeat of a sprite is drawn at
 *
 * Metres, in the road's own frame: `width_m` is LATERAL, across the direction of
 * travel, and `length_m` runs ALONG the direction of travel. This is what keeps
 * a marking placed in metres rather than in texels, so the same atlas serves a
 * 3 m service road and a 3.75 m motorway lane without rescaling.
 *
 * A value of 1.0 in either axis is a UNIT, meaning the emitter sets that extent
 * from the road geometry and the art must tolerate being stretched along it.
 * The sprites that carry a unit axis:
 *
 * | Sprite            | Unit axis  | Set from                              |
 * |-------------------|------------|---------------------------------------|
 * | SolidWhite        | length_m   | the run of the continuous line        |
 * | SolidYellow       | length_m   | the run of the continuous line        |
 * | DoubleSolidYellow | length_m   | the run of the continuous line        |
 * | StopLine          | width_m    | the width of the lane it stops        |
 *
 * Every other sprite is drawn at its stated size and repeated as whole quads.
 */
struct SpriteSize {
    float width_m = 0.0f;   ///< Lateral extent in metres, across travel
    float length_m = 0.0f;  ///< Longitudinal extent in metres, along travel
};

// ============================================================================
// Table lookup
// ============================================================================

/**
 * @brief Normalised atlas rect of a sprite
 *
 * Computed from the pixel layout documented at the top of this file, inset by
 * kAtlasInsetPixels on every side.
 *
 * @param s Sprite to look up
 * @return Its sub-rect; {0, 0, 0, 0} for MarkingSprite::Count and any
 *         out-of-range value, which produces a degenerate quad rather than
 *         sampling a neighbouring sprite
 */
[[nodiscard]] SpriteRect sprite_rect(MarkingSprite s);

/**
 * @brief Real-world size of one repeat of a sprite
 *
 * The frozen table:
 *
 * | Sprite             | width_m | length_m | Note                              |
 * |--------------------|---------|----------|-----------------------------------|
 * | DashWhite          | 0.15    | 3.0      | one dash; gap comes from config   |
 * | DashLongWhite      | 0.15    | 6.0      | one long dash                     |
 * | SolidWhite         | 0.15    | 1.0      | length is a unit; stretched       |
 * | SolidYellow        | 0.15    | 1.0      | length is a unit; stretched       |
 * | DoubleSolidYellow  | 0.35    | 1.0      | two 0.1 lines and a 0.15 gap      |
 * | DashedYellow       | 0.15    | 3.0      | one dash                          |
 * | StopLine           | 1.0     | 0.4      | width is a unit; spans the lane   |
 * | GiveWayTriangles   | 0.6     | 0.6      | ONE triangle                      |
 * | ZebraStripe        | 0.5     | 3.0      | ONE stripe; length is the depth   |
 * | BoxJunctionHatch   | 2.0     | 2.0      | one hatch cell                    |
 * | ArrowStraight      | 1.4     | 4.2      | 1:3, matching its 128 x 384 block |
 * | ArrowLeft          | 1.4     | 4.2      |                                   |
 * | ArrowRight         | 1.4     | 4.2      |                                   |
 * | ArrowStraightLeft  | 1.4     | 4.2      |                                   |
 * | ArrowStraightRight | 1.4     | 4.2      |                                   |
 * | ArrowUTurn         | 1.4     | 4.2      |                                   |
 * | BikeSymbol         | 1.0     | 2.0      | 1:2, matching its 128 x 256 block |
 * | BusSymbol          | 1.0     | 2.0      |                                   |
 *
 * Every arrow shares one size so the six blocks are interchangeable and no
 * arrow is distorted relative to its neighbours in a lane group.
 *
 * A MarkingConfig may override an extent -- MarkingConfig::line_width overrides
 * a line sprite's width_m, MarkingConfig::dash_length its length_m -- so these
 * are the sizes used when nothing overrides them.
 *
 * @param s Sprite to look up
 * @return Its size in metres; {0, 0} for MarkingSprite::Count and out-of-range
 */
[[nodiscard]] SpriteSize sprite_size(MarkingSprite s);

/**
 * @brief Convert a MarkingSprite to a stable human-readable string
 *
 * Used for logging, test failure messages, and editor debug overlays. The
 * returned pointer is a string literal with static storage duration.
 *
 * @param s Sprite to name
 * @return Its name, or "Unknown" for MarkingSprite::Count and out-of-range values
 */
[[nodiscard]] const char* marking_sprite_name(MarkingSprite s);

} // namespace stratum::osm::road
