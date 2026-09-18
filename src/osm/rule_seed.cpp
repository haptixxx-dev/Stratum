// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "osm/rule_seed.hpp"

namespace stratum::osm {
namespace {

using procgen::rules::Shape;
using procgen::rules::Value;

/**
 * @brief BuildingType as the word a rule file compares against
 *
 * Spelled out rather than derived from the enumerator name, because the
 * enumerator names are C++ and these are the language's vocabulary. A rule
 * file says `attrs.get("osm.type") == "apartments"`, and renaming a C++
 * enumerator must not silently change what that rule matches.
 *
 * Unknown returns "unknown" rather than an empty string: a rule that wants to
 * treat unclassified buildings specially needs a word to compare against, and
 * an empty string reads like a missing attribute.
 */
[[nodiscard]] const char* building_type_word(BuildingType type) {
    switch (type) {
        case BuildingType::Residential: return "residential";
        case BuildingType::Commercial:  return "commercial";
        case BuildingType::Industrial:  return "industrial";
        case BuildingType::Retail:      return "retail";
        case BuildingType::Office:      return "office";
        case BuildingType::Apartments:  return "apartments";
        case BuildingType::House:       return "house";
        case BuildingType::Detached:    return "detached";
        case BuildingType::Garage:      return "garage";
        case BuildingType::Shed:        return "shed";
        case BuildingType::Church:      return "church";
        case BuildingType::School:      return "school";
        case BuildingType::Hospital:    return "hospital";
        case BuildingType::Warehouse:   return "warehouse";
        case BuildingType::Unknown:     break;
    }
    return "unknown";
}

/**
 * @brief RoofType as the word a rule file compares against
 *
 * These deliberately match `roof()`'s own kind names where the shapes are the
 * same thing, so `roof(attrs.get("osm.roof", "hip"))` works with no
 * translation table in the rule file. OSM's "skillion" is `roof()`'s "shed"
 * and its "pyramidal" is "pyramid"; both are translated here rather than
 * leaving every rule author to discover the mismatch.
 */
[[nodiscard]] const char* roof_type_word(RoofType type) {
    switch (type) {
        case RoofType::Flat:      return "flat";
        case RoofType::Gabled:    return "gable";
        case RoofType::Hipped:    return "hip";
        case RoofType::Pyramidal: return "pyramid";
        case RoofType::Skillion:  return "shed";
        case RoofType::Dome:      return "dome";
        case RoofType::Unknown:   break;
    }
    return "unknown";
}

/**
 * @brief AreaType as the word a rule file compares against
 *
 * NOT `area_type_name()`, which returns "Park" and "Water" for a human to
 * read. A rule file comparing `attrs.get("osm.area_type") == "park"` against
 * that never matches and reports nothing, and `osm.type` beside it is lower
 * case, so the two would disagree about their own convention.
 */
[[nodiscard]] const char* area_type_word(AreaType type) {
    switch (type) {
        case AreaType::Water:       return "water";
        case AreaType::Park:        return "park";
        case AreaType::Forest:      return "forest";
        case AreaType::Grass:       return "grass";
        case AreaType::Parking:     return "parking";
        case AreaType::Commercial:  return "commercial";
        case AreaType::Residential: return "residential";
        case AreaType::Industrial:  return "industrial";
        case AreaType::Farmland:    return "farmland";
        case AreaType::Cemetery:    return "cemetery";
        case AreaType::Unknown:     break;
    }
    return "unknown";
}

} // namespace

Shape seed_from_building(const Building& building, double y) {
    Shape shape = procgen::rules::shape_from_rings(building.footprint, building.holes, y);

    shape.attributes[kSeedOsmId] = Value::number(static_cast<double>(building.osm_id));
    shape.attributes["osm.kind"] = Value::text("building");
    shape.attributes["osm.height"] = Value::number(static_cast<double>(building.height));
    shape.attributes["osm.levels"] = Value::number(static_cast<double>(building.levels));
    shape.attributes["osm.type"] = Value::text(building_type_word(building.type));
    shape.attributes["osm.roof"] = Value::text(roof_type_word(building.roof_type));

    // Only when the source had one. `attrs.has("osm.name")` is then a real
    // question rather than a test against an empty string that every unnamed
    // building would also pass.
    if (!building.name.empty()) {
        shape.attributes["osm.name"] = Value::text(building.name);
    }
    if (building.building_color.has_value()) {
        shape.attributes["osm.colour"] = Value::text(*building.building_color);
    }
    if (building.roof_color.has_value()) {
        shape.attributes["osm.roof_colour"] = Value::text(*building.roof_color);
    }
    return shape;
}

Shape seed_from_area(const Area& area, double y) {
    Shape shape = procgen::rules::shape_from_rings(area.polygon, area.holes, y);
    shape.attributes[kSeedOsmId] = Value::number(static_cast<double>(area.osm_id));
    shape.attributes["osm.kind"] = Value::text("area");
    shape.attributes["osm.area_type"] = Value::text(area_type_word(area.type));
    if (!area.name.empty()) {
        shape.attributes["osm.name"] = Value::text(area.name);
    }
    return shape;
}

std::vector<Shape> seeds_from_buildings(const ParsedOSMData& data, double y) {
    std::vector<Shape> seeds;
    seeds.reserve(data.buildings.size());
    for (const Building& building : data.buildings) {
        seeds.push_back(seed_from_building(building, y));
    }
    return seeds;
}

} // namespace stratum::osm
