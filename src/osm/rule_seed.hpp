// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file rule_seed.hpp
 * @brief Turn imported OSM features into seed shapes the rule engine can run on
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS IS FOR
 * ================================================================================
 *
 * The rule engine has been able to build a building since D1, and until now the
 * only thing anyone could point it at was a test rectangle. That is the whole
 * gap between "Stratum has a shape grammar" and "Stratum replaces an imported
 * city with a generated one", which is the workflow the project exists for:
 * import an extract, and run rules over what came back.
 *
 * The geometry needed no adapting. `Building::footprint` is a counter-clockwise
 * outer ring and `Building::holes` are clockwise inner rings, which is exactly
 * `shape_from_rings()`'s contract -- the two were written to the same
 * convention independently. So this file is not a converter. It is the
 * ATTRIBUTES.
 *
 * ================================================================================
 * WHY THE ATTRIBUTES ARE THE POINT
 * ================================================================================
 *
 * A rule run on a bare footprint can only make the same building everywhere.
 * What makes a generated city look imported is that the rule can READ what OSM
 * knew: how tall the mapped building was, how many levels, what it was for,
 * what roof it had. Then one rule file produces a warehouse on the industrial
 * plot and a terrace on the residential one, from the data rather than from a
 * draw.
 *
 * So every seed carries its feature's tags as shape attributes, and a rule
 * reads them with `attrs.get("levels", 3)` or `attrs.has("name")` exactly as it
 * reads its own declarations. The names are fixed here and documented, because
 * a rule file that guesses at them silently gets the default instead:
 *
 *   osm.id        int     The OSM way or relation id. Stable across imports.
 *   osm.height    float   Metres. The mapped height, or the import default.
 *   osm.levels    int     Storeys.
 *   osm.type      string  BuildingType, lower case: house, apartments, ...
 *   osm.roof      string  RoofType, lower case: flat, gabled, hipped, ...
 *   osm.name      string  Present only when the feature is named.
 *   osm.kind      string  "building" or "area".
 *   osm.area_type string  AreaType for an area seed: park, water, ...
 *
 * An attribute is only set when the source had it. `attrs.has("osm.name")` is
 * therefore a real question, and `attrs.get("osm.name", "")` is how a rule
 * asks without caring.
 *
 * ================================================================================
 * WHAT THIS DELIBERATELY DOES NOT DO
 * ================================================================================
 *
 * It does not run anything. Seeding and generating are separate so that a
 * caller can seed once and generate many times -- with a different rule file,
 * a different seed number, or on a worker -- without re-reading the import.
 *
 * It does not filter. A caller that wants only the industrial buildings
 * filters `ParsedOSMData::buildings` itself; a predicate here would be one
 * more thing to keep in step with the type enums.
 *
 * It does not project or recentre. The footprints are already in local metres
 * with the import's origin applied, and a shape whose coordinates disagreed
 * with the rest of the scene would be worse than useless.
 */

#pragma once

#include "osm/types.hpp"
#include "procgen/rules/shape.hpp"

#include <vector>

namespace stratum::osm {

/**
 * @brief One building footprint as a rule-engine seed shape
 *
 * The outer ring and holes go through `shape_from_rings()`, which fixes the
 * winding rather than trusting it, so a footprint that came in the wrong way
 * round still produces a shape whose normal points up.
 *
 * @param building Source feature; not modified
 * @param y        Ground height for the footprint, in metres
 * @return A shape with no volume yet -- a rule gives it one. Its geometry is
 *         empty when the footprint has fewer than three points, which is what
 *         `shape_from_rings()` does with a degenerate ring; the attributes are
 *         set regardless, so a caller can still report which feature it was.
 */
[[nodiscard]] procgen::rules::Shape seed_from_building(const Building& building,
                                                       double y = 0.0);

/**
 * @brief One area footprint as a rule-engine seed shape
 *
 * For the rules that dress a park or a car park rather than a building.
 */
[[nodiscard]] procgen::rules::Shape seed_from_area(const Area& area, double y = 0.0);

/**
 * @brief Every building in an import, as seeds
 *
 * In `ParsedOSMData::buildings` order, which is the parse order, so two runs
 * over the same extract produce the same seeds in the same sequence. That
 * ordering is what makes a generated city reproducible at all -- see the
 * determinism note in procgen/rules/interpreter.hpp.
 *
 * A building whose footprint is degenerate still yields a seed, with empty
 * geometry. Dropping it here would renumber every seed after it and change
 * what the whole city generates.
 */
[[nodiscard]] std::vector<procgen::rules::Shape> seeds_from_buildings(
    const ParsedOSMData& data, double y = 0.0);

/// The attribute name a seed carries its OSM id under. See the header note.
inline constexpr const char* kSeedOsmId = "osm.id";

} // namespace stratum::osm
