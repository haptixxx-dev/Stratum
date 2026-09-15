#include "osm/road/way_semantics.hpp"

namespace stratum::osm::road {
namespace {

/// Value of @p key, or nullptr. A missing tag and a null map read the same.
[[nodiscard]] const std::string* value_of(const TagMap* tags, const char* key) {
    if (tags == nullptr) return nullptr;
    const auto it = tags->find(key);
    return it == tags->end() ? nullptr : &it->second;
}

[[nodiscard]] bool equals(const std::string* value, const char* expected) {
    return value != nullptr && *value == expected;
}

/// OSM's boolean spellings. `1` is common in older data and in imports.
[[nodiscard]] bool is_yes(const std::string* value) {
    return value != nullptr && (*value == "yes" || *value == "true" || *value == "1");
}

/// Footway, Cycleway and Path are the classes whose role the tags refine.
[[nodiscard]] bool is_pedestrian_class(RoadType type) {
    return type == RoadType::Footway || type == RoadType::Cycleway || type == RoadType::Path;
}

} // namespace

WaySemantics classify_way(RoadType type, const TagMap* tags) {
    WaySemantics out;

    // `area=yes` applies to any class. A way carrying it is an outline to be
    // filled; sweeping a corridor along that outline traces the PERIMETER of a
    // plaza as though it were a path, which is why a pedestrian square currently
    // renders as a loop of pavement around an empty middle.
    out.is_area = is_yes(value_of(tags, "area"));

    if (!is_pedestrian_class(type)) {
        // A carriageway tagged as an area is still not a centreline. Everything
        // else about it stays as it was.
        if (out.is_area) {
            out.extrude_surface = false;
        }
        return out;
    }

    // `highway=steps` reaches here as RoadType::Footway -- the parser folds the
    // two together -- so the staircase has to be recovered from the tag.
    const std::string* highway = value_of(tags, "highway");
    if (equals(highway, "steps")) {
        out.role = WayRole::Steps;
        // Still extruded, deliberately. A flat ramp where a staircase belongs is
        // wrong, but it is CONNECTED and walkable, whereas dropping it leaves a
        // hole in the pedestrian network at exactly the places -- underpasses,
        // station entrances -- where the network matters most. Recording the role
        // is what lets a later change build real steps here.
        return out;
    }

    // The crossing test reads three keys, because OSM spells it three ways
    // depending on which class the way is and how old the mapping is:
    //   highway=footway  + footway=crossing
    //   highway=cycleway + cycleway=crossing
    //   highway=path     + crossing=marked|unmarked|traffic_signals|...
    // The bare `crossing` key also appears on footways alongside `footway`, so it
    // is checked for every pedestrian class rather than only for paths.
    const std::string* footway = value_of(tags, "footway");
    const std::string* cycleway = value_of(tags, "cycleway");
    const std::string* crossing = value_of(tags, "crossing");

    const bool is_crossing = equals(footway, "crossing") || equals(cycleway, "crossing") ||
                             (crossing != nullptr && *crossing != "no");

    if (is_crossing) {
        out.role = WayRole::Crossing;
        // Not swept, but STILL A GRAPH EDGE. find_crossings() finds a zebra where a
        // crossing way meets the carriageway, so dropping the way from the graph
        // would delete the very crossing it describes.
        out.extrude_surface = false;
        return out;
    }

    if (equals(footway, "traffic_island") || equals(footway, "link")) {
        out.role = WayRole::Link;
        // Connective only: it has no surface, but it is real topology and other
        // pedestrian ways route through it, so it stays in the graph.
        out.extrude_surface = false;
        return out;
    }

    out.role = WayRole::Sidewalk;
    if (out.is_area) {
        out.extrude_surface = false;
    }
    return out;
}

} // namespace stratum::osm::road
