// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file attribute_palette.hpp
 * @brief Classification enum in, debug colour out. The whole of the colour-by-attribute mode.
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The viewport can be asked to stop showing what the world looks like and start
 * showing how it was CLASSIFIED: every building painted by its BuildingType,
 * every road by its RoadType, every area by its AreaType, every tile by which
 * tile it is. That mode exists to answer one question fast -- "why is this piece
 * of geometry wrong?" -- and the answer is nearly always that it was classified
 * as something other than what it is, or not classified at all.
 *
 * ### Why this is a table in stratum_core and not a switch in the editor
 *
 * The same rule that put road_style.hpp here. A mapping written inline next to
 * the ImGui call is a mapping nothing can test, and it grows a second copy the
 * first time another consumer -- an exporter, a headless diff, the rule engine --
 * needs the same colours. So the mapping is a pure function of an enumerator,
 * with no SDL, no ImGui and no renderer state anywhere near it, and the editor
 * side is a combo box and a tint.
 *
 * This is a SIBLING of osm/road/road_style.hpp rather than an extension of it,
 * for two reasons. road_style.hpp states as a contract that its half of the split
 * emits "no textures, no colours, no PBR parameters" -- a colour table inside it
 * would contradict the first line of its own documentation. And it is scoped to
 * road and building MATERIALS, whereas this covers area types and tile identity
 * too, neither of which has a MaterialKey.
 *
 * ### These colours are not materials
 *
 * Nothing here ever reaches a shipped map. A MaterialKey says what a surface is
 * made of and survives export; an attribute colour is a debug overlay that exists
 * only while a human is looking at the viewport. Do not resolve one from the
 * other in either direction.
 *
 * ### The palette
 *
 * Hand-picked, not generated. Slicing the hue circle into N even steps is the
 * obvious way to get N colours and it is wrong for this: an eleven-way slice puts
 * four greens next to each other, and the viewport already has a large green
 * thing in it called the terrain. The colours below are chosen against three
 * constraints, in this order:
 *
 * 1. **Separation inside a family.** No two values a user has to tell apart may
 *    be close. tests/osm/test_attribute_palette.cpp pins a minimum RGB distance
 *    of 0.20 between any two entries of one family; the tightest real pair
 *    (Orange and Vermillion) sits at 0.257.
 * 2. **Separation from the background.** Every entry stays clear of the terrain
 *    green and of the sky blue, which are the two things the coloured geometry is
 *    always seen against. Same test, same threshold.
 * 3. **Convention, where it is free.** RoadType follows the OSM-carto ordering a
 *    road person already reads without a legend -- motorway red, trunk
 *    vermillion, primary orange, secondary amber, tertiary lime -- because a
 *    palette someone has to learn is a palette someone gets wrong.
 *
 * Where a family genuinely needs several greens -- Park, Forest and Grass are all
 * vegetation and painting Grass purple helps nobody -- they are separated by
 * LIGHTNESS and chroma rather than by hue, which is what keeps them apart against
 * a green ground.
 *
 * Unknown is magenta in every family, and magenta is used for nothing else. That
 * is the point of the mode: unclassified geometry has to shout, because half the
 * reason to turn this on is to find geometry the parser gave up on.
 *
 * ### Totality
 *
 * Every family is a `std::array` INDEXED by the enumerator, with a `static_assert`
 * tying the array's length to the enumerator count. There is no default branch to
 * fall through, and adding a value to BuildingType, RoadType or AreaType without
 * adding its colour is a compile error in attribute_palette.cpp, not a warning
 * and not a silent grey at run time.
 *
 * That works because `Unknown` is the LAST enumerator of all three enums, so the
 * count is `Unknown + 1`. **Append new values BEFORE Unknown.** A value appended
 * after it is the one case the static_assert cannot see; it lands past the end of
 * the table and resolves to the Unknown colour, which is deliberately the least
 * quiet failure available -- it renders magenta, and magenta means "nobody
 * classified this".
 */

#pragma once

#include "osm/types.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>

namespace stratum::osm {

// ============================================================================
// Mode
// ============================================================================

/**
 * @brief Which attribute the viewport is currently painting by
 *
 * The enumerators are named for the ATTRIBUTE, not for the type that carries it:
 * `Building` means "colour by BuildingType". Count is a sentinel for iterating
 * the combo box and is never a mode anything renders in.
 */
enum class AttributeMode : uint8_t {
    None = 0,       ///< Normal shading; the palette is not consulted at all
    Building,       ///< Colour every building by its BuildingType
    Road,           ///< Colour every road by its RoadType
    Area,           ///< Colour every area by its AreaType
    Tile,           ///< Colour every piece by which tile or chunk it came from
    Count           ///< Sentinel: number of modes. Not a renderable mode.
};

/// Number of AttributeMode values, including None and excluding Count.
inline constexpr size_t kAttributeModeCount = static_cast<size_t>(AttributeMode::Count);

/**
 * @brief Short label for a mode, for the mode selector and for logs
 *
 * @param mode Mode to name
 * @return A string literal with static storage duration. AttributeMode::Count and
 *         any out-of-range value name themselves "Off", since neither renders.
 */
[[nodiscard]] const char* attribute_mode_name(AttributeMode mode);

// ============================================================================
// Family sizes
// ============================================================================
//
// Derived from Unknown rather than from a Count sentinel, because none of the
// three enums in osm/types.hpp has one and adding one would change the meaning of
// every existing exhaustive switch over them. See the totality note at the top of
// this file for what that costs and how the cost is contained.

/// Number of BuildingType values, Unknown included.
inline constexpr size_t kBuildingTypeCount = static_cast<size_t>(BuildingType::Unknown) + 1;

/// Number of RoadType values, Unknown included.
inline constexpr size_t kRoadTypeCount = static_cast<size_t>(RoadType::Unknown) + 1;

/// Number of AreaType values, Unknown included.
inline constexpr size_t kAreaTypeCount = static_cast<size_t>(AreaType::Unknown) + 1;

// ============================================================================
// Lookups
// ============================================================================

/**
 * @brief The colour that means "nobody classified this"
 *
 * Magenta. Shared by BuildingType::Unknown, RoadType::Unknown and
 * AreaType::Unknown, and used by no other value in any family, so a magenta
 * building and a magenta road mean the same thing and mean only that.
 *
 * @return Opaque magenta
 */
[[nodiscard]] glm::vec4 unknown_attribute_colour();

/**
 * @brief Colour for a building classification
 * @param type Value to look up
 * @return Opaque colour, every channel in [0,1]. Unknown, and any value outside
 *         the enumeration, give unknown_attribute_colour().
 */
[[nodiscard]] glm::vec4 building_type_colour(BuildingType type);

/**
 * @brief Colour for a road classification
 *
 * Ordered after OSM-carto so the hierarchy reads without a legend: motorway
 * crimson through tertiary lime, then the unclassified and non-vehicle classes in
 * colours that are deliberately NOT part of that warm ramp.
 *
 * @param type Value to look up
 * @return Opaque colour, every channel in [0,1]. Unknown, and any value outside
 *         the enumeration, give unknown_attribute_colour().
 */
[[nodiscard]] glm::vec4 road_type_colour(RoadType type);

/**
 * @brief Colour for an area classification
 * @param type Value to look up
 * @return Opaque colour, every channel in [0,1]. Unknown, and any value outside
 *         the enumeration, give unknown_attribute_colour().
 */
[[nodiscard]] glm::vec4 area_type_colour(AreaType type);

// ============================================================================
// Tile identity
// ============================================================================

/**
 * @brief Length of the tile colour cycle
 *
 * Twelve, and the value is load-bearing rather than decorative: it factors as
 * 3 x 4, which is what lets tile_colour_index() guarantee that no two touching
 * tiles ever draw the same colour. Changing it breaks that guarantee unless the
 * two moduli in tile_colour_index() change with it.
 */
inline constexpr size_t kTileColourCount = 12;

/**
 * @brief Colour for a position in the tile cycle
 *
 * @param index Cycle position; wraps, so any tile index at all is valid
 * @return Opaque colour, every channel in [0,1]. Never magenta -- an unclassified
 *         feature must stay distinguishable from a tile boundary even with both
 *         modes' colours on screen in the same session.
 */
[[nodiscard]] glm::vec4 tile_colour(size_t index);

/**
 * @brief Cycle position for a tile at integer grid coordinates
 *
 * A tile colouring is only useful if the SEAMS are visible, and a cycle taken
 * straight off a tile id fails at exactly that: two tiles that happen to be
 * `kTileColourCount` apart in id are frequently neighbours on the ground, and
 * where they are the boundary vanishes.
 *
 * So the index is built from the two axes separately, `(x mod 3) * 4 + (z mod 4)`.
 * Stepping one tile in x always changes the first term, stepping one tile in z
 * always changes the second, and the two terms cannot cancel on a diagonal
 * because the x term moves in steps of 4 or 8 and the z term in steps of 1 or 3.
 * All eight neighbours of any tile therefore differ from it, which is the
 * property the mode exists for.
 *
 * @param tile_x Tile column. Negative values are handled; C++ `%` is not.
 * @param tile_z Tile row
 * @return Cycle position in [0, kTileColourCount)
 */
[[nodiscard]] size_t tile_colour_index(int64_t tile_x, int64_t tile_z);

} // namespace stratum::osm
