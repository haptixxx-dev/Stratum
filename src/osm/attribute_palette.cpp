// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file attribute_palette.cpp
 * @brief The colour tables themselves, and the static_asserts that keep them total
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Read attribute_palette.hpp first: it carries the rules the tables below obey.
 */

#include "osm/attribute_palette.hpp"

#include <array>

namespace stratum::osm {

namespace {

/**
 * @brief One palette entry, as three plain floats
 *
 * Not a glm::vec4. The tables have to be `constexpr` so the static_asserts below
 * can see their length at compile time, and glm's constexpr support varies with
 * the GLM_FORCE_* configuration a build happens to be using. Three floats and an
 * alpha added at the call site have no such problem, and the alpha is 1 for every
 * entry anyway -- a translucent debug colour reads as a different colour, which
 * defeats the whole mode.
 */
struct Rgb {
    float r;
    float g;
    float b;
};

/// Widen a table entry to the opaque colour callers get.
///
/// Deliberately not constexpr: nothing needs a colour at compile time, and
/// marking it so would tie this file to whether the vendored glm's vec4
/// constructor happens to be constexpr under the build's GLM_FORCE_* settings.
inline glm::vec4 opaque(const Rgb& c) {
    return glm::vec4(c.r, c.g, c.b, 1.0f);
}

// ----------------------------------------------------------------------------
// The named colours
//
// Every one of these is used by at least one family; there are no spares. Adding
// a colour here that no table names is dead weight, and picking a colour inline
// in a table instead of naming it here is how two families end up a hundredth
// apart in one channel and indistinguishable on screen.
//
// The numbers are tuned, not nominal. Amber was lightened away from Orange and
// Azure was deepened away from the sky after measuring the separations that
// tests/osm/test_attribute_palette.cpp now pins.
// ----------------------------------------------------------------------------

constexpr Rgb kVermillion{0.84f, 0.37f, 0.00f};  ///< Deep warm orange-red
constexpr Rgb kOrange    {0.90f, 0.62f, 0.00f};  ///< Saturated mid orange
constexpr Rgb kAmber     {0.98f, 0.86f, 0.15f};  ///< Light yellow; kept clear of kOrange
constexpr Rgb kLime      {0.62f, 0.82f, 0.22f};  ///< Yellow-green, far lighter than terrain grass
constexpr Rgb kEmerald   {0.00f, 0.64f, 0.46f};  ///< Blue-shifted mid green
constexpr Rgb kForest    {0.09f, 0.38f, 0.22f};  ///< Dark green; the low end of the vegetation ramp
constexpr Rgb kTeal      {0.10f, 0.74f, 0.62f};  ///< Bright blue-green
constexpr Rgb kCyan      {0.20f, 0.88f, 0.92f};  ///< Pale bright cyan
constexpr Rgb kAzure     {0.03f, 0.35f, 0.90f};  ///< Deep blue; kept clear of the sky
constexpr Rgb kNavy      {0.08f, 0.16f, 0.52f};  ///< Very dark blue
constexpr Rgb kViolet    {0.64f, 0.34f, 0.80f};  ///< Mid purple
constexpr Rgb kPink      {0.98f, 0.48f, 0.72f};  ///< Light warm pink
constexpr Rgb kCrimson   {0.88f, 0.08f, 0.26f};  ///< Saturated red
constexpr Rgb kBrown     {0.52f, 0.32f, 0.22f};  ///< Dark earth brown
constexpr Rgb kTan       {0.82f, 0.68f, 0.48f};  ///< Light warm brown
constexpr Rgb kSlate     {0.40f, 0.46f, 0.62f};  ///< Desaturated blue-grey
constexpr Rgb kCream     {0.96f, 0.93f, 0.82f};  ///< Near-white warm

/// Reserved. Means "unclassified" and nothing else, in every family.
constexpr Rgb kMagenta   {1.00f, 0.00f, 0.85f};

// ----------------------------------------------------------------------------
// Families
//
// Each table is indexed BY the enumerator, so the order of the entries is the
// order of the enumerators in osm/types.hpp and is not free. The static_assert
// under each one is the whole totality guarantee: insert a value into the enum
// and this file stops compiling until its colour is inserted at the same place.
// ----------------------------------------------------------------------------

constexpr std::array kBuildingPalette = {
    kOrange,        // Residential
    kAzure,         // Commercial
    kSlate,         // Industrial
    kCrimson,       // Retail
    kCyan,          // Office
    kVermillion,    // Apartments
    kAmber,         // House
    kCream,         // Detached
    kBrown,         // Garage
    kTan,           // Shed
    kViolet,        // Church
    kTeal,          // School
    kPink,          // Hospital
    kNavy,          // Warehouse
    kMagenta,       // Unknown
};
static_assert(kBuildingPalette.size() == kBuildingTypeCount,
              "BuildingType gained or lost a value. Add or remove its colour in "
              "kBuildingPalette, at the same position it holds in the enum. New "
              "values go BEFORE BuildingType::Unknown; see attribute_palette.hpp.");

constexpr std::array kRoadPalette = {
    kCrimson,       // Motorway
    kVermillion,    // Trunk
    kOrange,        // Primary
    kAmber,         // Secondary
    kLime,          // Tertiary
    kCream,         // Residential
    kSlate,         // Service
    kViolet,        // Footway
    kAzure,         // Cycleway
    kBrown,         // Path
    kMagenta,       // Unknown
};
static_assert(kRoadPalette.size() == kRoadTypeCount,
              "RoadType gained or lost a value. Add or remove its colour in "
              "kRoadPalette, at the same position it holds in the enum. New "
              "values go BEFORE RoadType::Unknown; see attribute_palette.hpp.");

constexpr std::array kAreaPalette = {
    kAzure,         // Water
    kEmerald,       // Park
    kForest,        // Forest
    kLime,          // Grass
    kSlate,         // Parking
    kPink,          // Commercial
    kCream,         // Residential
    kViolet,        // Industrial
    kAmber,         // Farmland
    kBrown,         // Cemetery
    kMagenta,       // Unknown
};
static_assert(kAreaPalette.size() == kAreaTypeCount,
              "AreaType gained or lost a value. Add or remove its colour in "
              "kAreaPalette, at the same position it holds in the enum. New "
              "values go BEFORE AreaType::Unknown; see attribute_palette.hpp.");

/**
 * @brief The tile cycle
 *
 * Ordered so that consecutive entries alternate warm and cool. Neighbouring tiles
 * usually land on entries a few apart rather than adjacent -- see
 * tile_colour_index() -- but a cycle that drifts smoothly through the spectrum
 * still produces near-matching neighbours wherever it does not, and a seam you
 * have to squint at is a seam you miss.
 *
 * kMagenta is absent on purpose. It is the unclassified colour and must stay
 * unambiguous.
 */
constexpr std::array kTilePalette = {
    kVermillion, kAzure, kAmber,  kViolet,
    kTeal,       kCrimson, kCream, kNavy,
    kOrange,     kCyan,  kPink,   kSlate,
};
static_assert(kTilePalette.size() == kTileColourCount,
              "kTileColourCount and kTilePalette disagree. The count is also the "
              "product of the two moduli in tile_colour_index(); changing one "
              "without the others loses the no-two-neighbours-alike guarantee.");

/// Euclidean remainder. C++ `%` follows the sign of the dividend, and a tile at a
/// negative coordinate is ordinary -- the local origin sits inside the extract.
constexpr int64_t positive_mod(int64_t value, int64_t modulus) {
    const int64_t rem = value % modulus;
    return rem < 0 ? rem + modulus : rem;
}

} // namespace

// ============================================================================
// Mode
// ============================================================================

const char* attribute_mode_name(AttributeMode mode) {
    switch (mode) {
        case AttributeMode::None:     return "Off";
        case AttributeMode::Building: return "Building Type";
        case AttributeMode::Road:     return "Road Type";
        case AttributeMode::Area:     return "Area Type";
        case AttributeMode::Tile:     return "Tile / Chunk";
        case AttributeMode::Count:    break;
    }
    return "Off";
}

// ============================================================================
// Lookups
// ============================================================================

glm::vec4 unknown_attribute_colour() {
    return opaque(kMagenta);
}

glm::vec4 building_type_colour(BuildingType type) {
    const auto index = static_cast<size_t>(type);
    if (index >= kBuildingPalette.size()) {
        return unknown_attribute_colour();
    }
    return opaque(kBuildingPalette[index]);
}

glm::vec4 road_type_colour(RoadType type) {
    const auto index = static_cast<size_t>(type);
    if (index >= kRoadPalette.size()) {
        return unknown_attribute_colour();
    }
    return opaque(kRoadPalette[index]);
}

glm::vec4 area_type_colour(AreaType type) {
    const auto index = static_cast<size_t>(type);
    if (index >= kAreaPalette.size()) {
        return unknown_attribute_colour();
    }
    return opaque(kAreaPalette[index]);
}

// ============================================================================
// Tile identity
// ============================================================================

glm::vec4 tile_colour(size_t index) {
    return opaque(kTilePalette[index % kTilePalette.size()]);
}

size_t tile_colour_index(int64_t tile_x, int64_t tile_z) {
    return static_cast<size_t>(positive_mod(tile_x, 3) * 4 + positive_mod(tile_z, 4));
}

} // namespace stratum::osm
