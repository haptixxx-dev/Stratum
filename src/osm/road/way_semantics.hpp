/**
 * @file way_semantics.hpp
 * @brief What an OSM way actually IS, read from its full tag set
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### The problem this exists to fix
 *
 * The pipeline classified a way from ONE tag. `highway=footway` became
 * RoadType::Footway, which build_profile() turns into a single 2 metre
 * MaterialId::Sidewalk strip, which the corridor extruder sweeps into a paved
 * ribbon. That is right for a pavement beside a street and wrong for most of the
 * other things OSM spells `highway=footway`:
 *
 * - `footway=crossing` is a pedestrian route ACROSS a carriageway. It is not a
 *   surface. Extruding it lays a 2 metre slab of paving over the asphalt, and --
 *   because it joins the carriageway at a shared node -- it also becomes an ARM of
 *   that junction, which drags the junction polygon out into a starburst.
 * - `area=yes` marks a way whose geometry is a POLYGON, not a centreline. A
 *   ribbon swept along the outline of a plaza is not the plaza.
 *
 * OSM's answer to "which is it" is in the tags the way already carries, and
 * ParsedOSMData::ways keeps every one of them. Nothing needs storing; the
 * pipeline simply was not reading them.
 *
 * ### Read the whole tag set, not one key
 *
 * Every function here takes the full TagMap. Adding a distinction later means
 * reading another key in one place, rather than threading another bool from the
 * parser through Road, the graph and the builder.
 */

#pragma once

#include "osm/types.hpp"

namespace stratum::osm::road {

/**
 * @brief What a pedestrian or cycle way is FOR
 *
 * Only meaningful for a way whose RoadType is Footway, Cycleway or Path. Anything
 * else is Carriageway.
 */
enum class WayRole {
    /// A vehicle way. Everything not tagged as one of the pedestrian roles.
    Carriageway,

    /// `footway=sidewalk`, or an untagged footway. A real paved surface.
    Sidewalk,

    /// `footway=crossing`, `cycleway=crossing`, or any pedestrian way carrying
    /// `crossing=*`. A route over a carriageway, NOT a surface of its own.
    Crossing,

    /// `highway=steps`. A surface, but a staircase rather than a ramp.
    Steps,

    /// `footway=traffic_island`, `footway=link`. Connective topology with no
    /// surface of its own.
    Link,
};

/**
 * @brief Everything the geometry pipeline needs to know about one way
 *
 * A pure function of the tags. No geometry, no graph, no configuration.
 */
struct WaySemantics {
    WayRole role = WayRole::Carriageway;

    /// `area=yes`, or `highway=pedestrian` with `area=yes`. The geometry is an
    /// outline to be filled, not a centreline to be swept.
    bool is_area = false;

    /**
     * @brief Does this way get a swept surface of its own?
     *
     * False for a crossing, a link and an area. Those are the three cases where
     * sweeping a ribbon along the centreline produces geometry that should not
     * exist at all -- as opposed to geometry that is merely approximate, which is
     * what Steps is.
     */
    bool extrude_surface = true;

};

// THERE IS NO `admit_to_graph`. Removing a crossing way from the road graph was
// tried and is wrong: find_crossings() locates a zebra where a crossing WAY meets
// the carriageway, not only from `highway=crossing` nodes, so a crossing way that
// never reaches the graph takes its own crossing with it. Crossings.
// skew_footway_crossing_lands_on_the_carriageway_it_crosses pins exactly that --
// it asserts the shared node is a graph node of degree 4, which it can only be if
// the footway is an edge.
//
// The way stays in the graph as topology and simply does not get a surface swept
// along it, which is the whole of what was wrong on screen.

/**
 * @brief Read @p tags to decide what @p type of way this really is
 *
 * @param type The RoadType the parser derived from `highway=*`
 * @param tags The way's complete tag set, or nullptr if it has none
 * @return Semantics. A null @p tags gives the defaults for @p type, which is a
 *         plain carriageway for a vehicle road and a sidewalk for a footway.
 */
[[nodiscard]] WaySemantics classify_way(RoadType type, const TagMap* tags);

// There is deliberately NO overload taking (ParsedOSMData, WayId). It would have
// to find the Road to recover its RoadType, and the only way to do that is a scan
// of data.roads -- which turns an O(1) tag lookup into O(roads) and would be
// called once per edge. Callers already hold both halves: the RoadType from the
// Road or GraphEdge, and the tags from ParsedOSMData::ways.

} // namespace stratum::osm::road
