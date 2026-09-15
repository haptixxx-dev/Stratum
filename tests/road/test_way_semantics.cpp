// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_way_semantics.cpp
 * @brief Reading what a way IS from its full tag set
 *
 * The bug this guards against is one tag standing in for the whole tag set.
 * `highway=footway` was taken to mean "a 2 metre paved ribbon", so every
 * `footway=crossing` -- a pedestrian route ACROSS a carriageway -- was swept into
 * a slab of paving laid over the asphalt at every junction that had one mapped.
 */

#include "framework.hpp"

#include "osm/road/way_semantics.hpp"

using stratum::osm::RoadType;
using stratum::osm::TagMap;
using stratum::osm::road::classify_way;
using stratum::osm::road::WayRole;
using stratum::osm::road::WaySemantics;

namespace {

TagMap tags(std::initializer_list<std::pair<const char*, const char*>> pairs) {
    TagMap out;
    for (const auto& [key, value] : pairs) out.emplace(key, value);
    return out;
}

} // namespace

/**
 * @brief A crossing way carries no surface, and stays in the graph
 *
 * Both halves matter and they pull in opposite directions. Sweeping it lays
 * pavement over the road; removing it from the graph deletes the crossing, because
 * find_crossings() locates the zebra where the crossing WAY meets the carriageway.
 * So: no ribbon, but still an edge.
 */
TEST(WaySemantics, a_footway_crossing_has_no_surface_of_its_own) {
    const TagMap t = tags({ { "highway", "footway" }, { "footway", "crossing" } });
    const WaySemantics s = classify_way(RoadType::Footway, &t);

    CHECK(s.role == WayRole::Crossing);
    CHECK_FALSE(s.extrude_surface);
}

/// OSM spells a crossing three ways depending on the class and the mapper's era.
TEST(WaySemantics, every_spelling_of_a_crossing_is_recognised) {
    const TagMap footway = tags({ { "highway", "footway" }, { "footway", "crossing" } });
    CHECK(classify_way(RoadType::Footway, &footway).role == WayRole::Crossing);

    const TagMap cycleway = tags({ { "highway", "cycleway" }, { "cycleway", "crossing" } });
    CHECK(classify_way(RoadType::Cycleway, &cycleway).role == WayRole::Crossing);

    // The bare `crossing` key, which appears on paths and on older footways.
    const TagMap marked = tags({ { "highway", "path" }, { "crossing", "marked" } });
    CHECK(classify_way(RoadType::Path, &marked).role == WayRole::Crossing);

    const TagMap signals = tags({ { "highway", "footway" }, { "crossing", "traffic_signals" } });
    CHECK(classify_way(RoadType::Footway, &signals).role == WayRole::Crossing);
}

/**
 * @brief `crossing=no` is a statement that it is NOT one
 *
 * A tag present with a negative value is not the same as a tag present. Treating
 * any `crossing` key as a crossing would delete the surface of every footway a
 * mapper had explicitly marked as not crossing anything.
 */
TEST(WaySemantics, crossing_no_is_not_a_crossing) {
    const TagMap t = tags({ { "highway", "footway" }, { "crossing", "no" } });
    const WaySemantics s = classify_way(RoadType::Footway, &t);

    CHECK(s.role == WayRole::Sidewalk);
    CHECK_TRUE(s.extrude_surface);
}

/// The ordinary case, and the one that must not regress: a pavement is a surface.
TEST(WaySemantics, a_sidewalk_keeps_its_surface) {
    const TagMap tagged = tags({ { "highway", "footway" }, { "footway", "sidewalk" } });
    const WaySemantics s = classify_way(RoadType::Footway, &tagged);
    CHECK(s.role == WayRole::Sidewalk);
    CHECK_TRUE(s.extrude_surface);

    // An untagged footway is a footpath, which is also a real surface.
    const TagMap bare = tags({ { "highway", "footway" } });
    CHECK_TRUE(classify_way(RoadType::Footway, &bare).extrude_surface);

    // And no tags at all must not be read as "no surface".
    CHECK_TRUE(classify_way(RoadType::Footway, nullptr).extrude_surface);
}

/**
 * @brief Steps keep their surface deliberately, and are still recorded as steps
 *
 * A flat ramp where a staircase belongs is wrong, but it is CONNECTED. Dropping it
 * leaves a hole in the pedestrian network at underpasses and station entrances --
 * exactly where the network is load-bearing. The role is recorded so a later change
 * can build real steps without having to re-derive it.
 */
TEST(WaySemantics, steps_are_recorded_but_still_extruded) {
    const TagMap t = tags({ { "highway", "steps" } });
    const WaySemantics s = classify_way(RoadType::Footway, &t);

    CHECK(s.role == WayRole::Steps);
    CHECK_TRUE(s.extrude_surface);
}

/**
 * @brief A traffic island or a link is topology without a surface
 *
 * They exist to connect the pedestrian network across a central reservation. A
 * ribbon swept along one puts pavement in the middle of the carriageway.
 */
TEST(WaySemantics, traffic_islands_and_links_carry_no_surface) {
    const TagMap island = tags({ { "highway", "footway" }, { "footway", "traffic_island" } });
    CHECK(classify_way(RoadType::Footway, &island).role == WayRole::Link);
    CHECK_FALSE(classify_way(RoadType::Footway, &island).extrude_surface);

    const TagMap link = tags({ { "highway", "footway" }, { "footway", "link" } });
    CHECK_FALSE(classify_way(RoadType::Footway, &link).extrude_surface);
}

/**
 * @brief `area=yes` means the geometry is an outline, not a centreline
 *
 * Sweeping a corridor along the outline of a pedestrian square traces its
 * PERIMETER as though it were a path, leaving a loop of pavement round an empty
 * middle. Filling it properly is triangulation, not extrusion.
 */
TEST(WaySemantics, an_area_is_not_swept_along_its_outline) {
    const TagMap plaza = tags({ { "highway", "pedestrian" }, { "area", "yes" } });
    const WaySemantics s = classify_way(RoadType::Footway, &plaza);

    CHECK_TRUE(s.is_area);
    CHECK_FALSE(s.extrude_surface);

    // Also on a vehicle class -- `highway=service` + `area=yes` is a car park
    // aisle drawn as a polygon.
    const TagMap yard = tags({ { "highway", "service" }, { "area", "yes" } });
    CHECK_FALSE(classify_way(RoadType::Service, &yard).extrude_surface);
}

/// OSM's boolean spellings. `1` is common in older data and in imports.
TEST(WaySemantics, area_accepts_every_spelling_of_yes) {
    for (const char* value : { "yes", "true", "1" }) {
        const TagMap t = tags({ { "highway", "pedestrian" }, { "area", value } });
        CHECK_TRUE(classify_way(RoadType::Footway, &t).is_area);
    }
    const TagMap no = tags({ { "highway", "pedestrian" }, { "area", "no" } });
    CHECK_FALSE(classify_way(RoadType::Footway, &no).is_area);
}

/**
 * @brief A carriageway is untouched by any of this
 *
 * The pedestrian keys are only read for pedestrian classes. A road tagged
 * `crossing=marked` -- which happens, meaning the road HAS a crossing on it -- must
 * not lose its own surface.
 */
TEST(WaySemantics, a_carriageway_keeps_its_surface_whatever_pedestrian_tags_it_carries) {
    const TagMap t = tags({ { "highway", "primary" }, { "crossing", "marked" },
                            { "footway", "crossing" } });
    const WaySemantics s = classify_way(RoadType::Primary, &t);

    CHECK(s.role == WayRole::Carriageway);
    CHECK_TRUE(s.extrude_surface);
}
