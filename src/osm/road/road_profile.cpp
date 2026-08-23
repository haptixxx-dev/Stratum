/**
 * @file road_profile.cpp
 * @brief Implementation of the tag-driven cross-section model
 *
 * The whole file is one table plus one assembler. rules_for() turns a RoadType
 * into a small policy record, and build_profile() walks a fixed left-to-right
 * order pushing the strips that policy and the tags enable:
 *
 *     verge sidewalk curbtop curbface gutter parking cycle shoulder busway
 *     LANES [median] LANES
 *     busway shoulder cycle parking gutter curbface curbtop sidewalk verge
 *
 * Adding a road feature is adding one push in that order plus one field in the
 * policy record. No mesh code changes, which is the point of the strip model.
 *
 * Height continuity is maintained by construction rather than by a fix-up pass:
 * the only strips that change height are CurbFace strips, and every raised strip
 * is bracketed by a pair of them. is_valid() re-checks the invariant so a future
 * strip rule that breaks it is caught before the extruder folds geometry.
 */

#include "osm/road/road_profile.hpp"

#include <algorithm>
#include <cstdlib>
#include <string>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Constants
// ============================================================================

/// Tolerance for the adjacent-strip height agreement invariant, metres
constexpr float kHeightEpsilon = 1.0e-4f;

/// Tolerance for comparing a tag-derived width against a synthesized one, metres
constexpr float kWidthEpsilon = 1.0e-3f;

/// Narrowest per-lane width accepted from a width=* tag, metres
constexpr float kMinLaneWidth = 1.0f;

/// Widest per-lane width accepted from a width=* tag, metres
constexpr float kMaxLaneWidth = 20.0f;

/// Narrowest single-strip width for a footway, cycleway, or path, metres
constexpr float kMinPathWidth = 0.5f;

/// Widest single-strip width for a footway, cycleway, or path, metres
constexpr float kMaxPathWidth = 20.0f;

/// Metres per lane the parser assumes when it synthesizes a width from lanes=*
constexpr float kParserMetresPerLane = 3.5f;

// ============================================================================
// Tag Access
// ============================================================================

/**
 * @brief Look up one tag, tolerating a null map
 * @return Pointer to the value, or nullptr when the map or the key is absent
 */
[[nodiscard]] const std::string* find_tag(const TagMap* tags, const char* key) {
    if (tags == nullptr) return nullptr;
    const auto it = tags->find(key);
    return it == tags->end() ? nullptr : &it->second;
}

/**
 * @brief Whether a tag value denies the feature it describes
 *
 * "separate" is negative on purpose: it means the feature is mapped as its own
 * OSM way, so synthesizing one here would duplicate real geometry.
 */
[[nodiscard]] bool tag_denies(const std::string& value) {
    return value == "no" || value == "none" || value == "false" ||
           value == "0" || value == "separate";
}

/**
 * @brief Parse a non-negative integer tag value
 * @return The value, or -1 when the tag is absent or not a plain integer
 */
[[nodiscard]] int parse_int_tag(const TagMap* tags, const char* key) {
    const std::string* value = find_tag(tags, key);
    if (value == nullptr || value->empty()) return -1;
    char* end = nullptr;
    const long parsed = std::strtol(value->c_str(), &end, 10);
    if (end == value->c_str() || parsed < 0 || parsed > 32) return -1;
    return static_cast<int>(parsed);
}

/**
 * @brief Parse a plain metre value, tolerating a trailing " m"
 * @return The value in metres, or -1.0f when it is absent or unparseable
 */
[[nodiscard]] float parse_metres(const std::string& value) {
    if (value.empty()) return -1.0f;
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || !std::isfinite(parsed) || parsed <= 0.0) return -1.0f;
    return static_cast<float>(parsed);
}

/**
 * @brief Read a side-scoped tag family into SideFlags
 *
 * Handles both OSM spellings at once: the side in the value (`verge=left`) and
 * the side in the key (`verge:left=grass`, `verge:both=yes`). A value that names
 * neither a side nor a denial -- `verge=grass` -- is presence on both sides,
 * since it describes the feature rather than its extent.
 *
 * @param tags Way tags, may be null
 * @param base Tag key without a side suffix, for example "verge"
 * @return Presence by side; Unknown when no tag of the family is present
 */
[[nodiscard]] SideFlags read_sided_tag(const TagMap* tags, const char* base) {
    if (tags == nullptr) return SideFlags::Unknown;

    bool left = false;
    bool right = false;
    bool seen = false;

    const std::string base_key(base);
    if (const std::string* value = find_tag(tags, base_key.c_str())) {
        seen = true;
        if (!tag_denies(*value)) {
            if (*value == "left") {
                left = true;
            } else if (*value == "right") {
                right = true;
            } else {
                left = true;
                right = true;
            }
        }
    }

    const struct { const char* suffix; bool on_left; bool on_right; } sides[] = {
        {":left", true, false},
        {":right", false, true},
        {":both", true, true},
    };
    for (const auto& side : sides) {
        if (const std::string* value = find_tag(tags, (base_key + side.suffix).c_str())) {
            seen = true;
            if (!tag_denies(*value)) {
                left = left || side.on_left;
                right = right || side.on_right;
            }
        }
    }

    if (!seen) return SideFlags::Unknown;
    if (left && right) return SideFlags::Both;
    if (left) return SideFlags::Left;
    if (right) return SideFlags::Right;
    return SideFlags::None;
}

// ============================================================================
// Class Policy
// ============================================================================

/**
 * @brief Everything the strip order needs to know about a road classification
 *
 * This record is the class table. A new road class is a new row here; a new
 * per-class behaviour is a new field here plus one branch in build_profile().
 */
struct ClassRules {
    bool single_strip = false;      ///< Footway, Cycleway, Path: one strip, nothing else
    bool allow_sidewalk = false;    ///< A positive sidewalk=* tag may emit a sidewalk
    bool synth_sidewalk = false;    ///< sidewalk=* absent may still emit one
    bool has_gutter = false;        ///< Gutter strips outboard of the outer lanes
    bool has_shoulder = false;      ///< Shoulder strips on both sides regardless of tags
    bool allow_median = false;      ///< A wide two-way carriageway may carry a median
    bool urban_median = false;      ///< Kerbed street: a median is a refuge, raised at a lower width
    bool has_verge = false;         ///< Grass verge on both sides regardless of tags
    bool rural_verge = false;       ///< An unpaved carriageway of this class may grow a verge
    float lane_width = 3.5f;        ///< Per-lane width when no width=* tag resolves one
    MaterialId default_material = MaterialId::Asphalt;   ///< Surface when surface=* is absent
    StripKind single_kind = StripKind::Lane;             ///< Kind of the single strip
};

/**
 * @brief The class table
 *
 * Motorway and Trunk are grade-separated rural sections: hard shoulders, no
 * curb, no footway, and a median once the carriageway is wide enough to be
 * dual. Primary through Residential are the kerbed urban profile, and Primary
 * alone may infer a median from its lane count -- the rest need the tag.
 * Service is narrowed further by apply_service_rules() once service=* is read.
 * Unknown follows Residential but never invents a footway, because a class we
 * failed to recognise is not evidence that pedestrians walk along it.
 */
[[nodiscard]] ClassRules rules_for(RoadType type, const ProfileConfig& cfg) {
    ClassRules r;
    switch (type) {
        case RoadType::Motorway:
        case RoadType::Trunk:
            r.has_shoulder = true;
            r.allow_median = true;
            r.lane_width = cfg.motorway_lane_width;
            break;

        case RoadType::Primary:
            // The only kerbed class that may infer a median from its lane count.
            // A four-lane primary is a dual carriageway far more often than it is
            // four undivided lanes; a four-lane secondary is not.
            r.allow_sidewalk = true;
            r.synth_sidewalk = true;
            r.has_gutter = true;
            r.allow_median = cfg.urban_dual_median;
            r.urban_median = true;
            r.rural_verge = true;
            r.lane_width = cfg.lane_width_default;
            break;

        case RoadType::Secondary:
        case RoadType::Tertiary:
        case RoadType::Residential:
            r.allow_sidewalk = true;
            r.synth_sidewalk = true;
            r.has_gutter = true;
            r.urban_median = true;
            r.rural_verge = true;
            r.lane_width = cfg.lane_width_default;
            break;

        case RoadType::Service:
            r.allow_sidewalk = true;
            r.has_gutter = true;
            r.rural_verge = true;
            r.lane_width = cfg.service_lane_width;
            break;

        case RoadType::Footway:
            r.single_strip = true;
            r.default_material = MaterialId::Sidewalk;
            r.single_kind = StripKind::Sidewalk;
            break;

        case RoadType::Cycleway:
            r.single_strip = true;
            r.default_material = MaterialId::Asphalt;
            r.single_kind = StripKind::CycleLane;
            break;

        case RoadType::Path:
            // highway=track and highway=path. The plan's rural cross-section is
            // verge, running surface, verge: an unsurfaced track has no edge to
            // speak of, and the verge is what stops it ending in a hard line
            // against the terrain.
            r.single_strip = true;
            r.has_verge = true;
            r.default_material = MaterialId::Dirt;
            r.single_kind = StripKind::Lane;
            break;

        case RoadType::Unknown:
            r.allow_sidewalk = true;
            r.has_gutter = true;
            r.rural_verge = true;
            r.lane_width = cfg.lane_width_default;
            break;
    }
    return r;
}

// ============================================================================
// Service Roads
// ============================================================================

/**
 * @brief What a highway=service way actually is
 *
 * highway=service covers everything from a farm access to a supermarket aisle,
 * and service=* is the only thing that separates them. Built at street width a
 * driveway swallows the front garden it runs through, and its kerb and gutter
 * are geometry no driveway has.
 */
enum class ServiceKind : uint8_t {
    Generic,        ///< service=* absent or unrecognised: a minor access road
    Driveway,       ///< service=driveway, service=drive-through
    ParkingAisle,   ///< service=parking_aisle
    Alley,          ///< service=alley
};

/**
 * @brief Read service=* into a ServiceKind
 *
 * @param tags Way tags, may be null
 * @return The kind; Generic when the tag is absent or names something else
 */
[[nodiscard]] ServiceKind read_service_kind(const TagMap* tags) {
    const std::string* value = find_tag(tags, "service");
    if (value == nullptr) return ServiceKind::Generic;

    if (*value == "driveway" || *value == "drive-through" || *value == "drive_through")
        return ServiceKind::Driveway;
    if (*value == "parking_aisle") return ServiceKind::ParkingAisle;
    if (*value == "alley") return ServiceKind::Alley;
    return ServiceKind::Generic;
}

/**
 * @brief Narrow the Service class rules down to the service=* subtype
 *
 * All three named subtypes drop the gutter and the sidewalk together: none of
 * them is kerbed, and a kerb is what a gutter drains into. Dropping the sidewalk
 * also drops the curb, since build_profile() only emits a curb to carry one.
 *
 * Only called for RoadType::Service, so no other class can be reshaped here.
 *
 * @param r    Class rules to narrow, modified in place
 * @param kind Service subtype
 * @param cfg  Widths for the subtypes
 */
void apply_service_rules(ClassRules& r, ServiceKind kind, const ProfileConfig& cfg) {
    switch (kind) {
        case ServiceKind::Driveway:
            r.allow_sidewalk = false;
            r.has_gutter = false;
            r.lane_width = cfg.driveway_width;
            break;

        case ServiceKind::ParkingAisle:
            r.allow_sidewalk = false;
            r.has_gutter = false;
            r.lane_width = cfg.parking_aisle_width;
            break;

        case ServiceKind::Alley:
            r.allow_sidewalk = false;
            r.has_gutter = false;
            r.lane_width = cfg.alley_width;
            break;

        case ServiceKind::Generic:
            break;
    }
}

// ============================================================================
// Medians
// ============================================================================

/**
 * @brief What median=* or divider=* says about the central reservation
 *
 * Two tag families for one feature, because OSM has never settled on which.
 * Both are read and the first that resolves wins.
 */
enum class MedianTag : uint8_t {
    Absent,     ///< No tag: fall back to the class rule and the width threshold
    Deny,       ///< median=no: a wide two-way carriageway that is NOT divided
    Painted,    ///< Hatching on the carriageway surface, flush with it
    Raised,     ///< A kerbed island, whatever its width
    Present,    ///< median=yes: divided, but the tag does not say how
};

/**
 * @brief Read median=* and divider=* into a MedianTag
 *
 * @param tags Way tags, may be null
 * @return The tag's verdict; Absent when neither family is present
 */
[[nodiscard]] MedianTag read_median_tag(const TagMap* tags) {
    const char* keys[] = {"median", "divider"};
    for (const char* key : keys) {
        const std::string* value = find_tag(tags, key);
        if (value == nullptr) continue;

        if (tag_denies(*value)) return MedianTag::Deny;
        if (*value == "painted" || *value == "flush" || *value == "marking" ||
            *value == "paint" || *value == "hatching")
            return MedianTag::Painted;
        if (*value == "raised" || *value == "kerb" || *value == "curb" ||
            *value == "island" || *value == "barrier" || *value == "depressed" ||
            *value == "grass" || *value == "planted")
            return MedianTag::Raised;
        return MedianTag::Present;
    }
    return MedianTag::Absent;
}

/**
 * @brief The width OSMParser::estimate_road_width() falls back to per class
 *
 * Mirrored here so build_profile() can tell a surveyed width=* tag from a width
 * the parser synthesized. GraphEdge carries no "the tag was present" bit, and a
 * synthesized width must not be redistributed across the lanes as though it had
 * been measured.
 */
[[nodiscard]] float parser_default_width(RoadType type) {
    switch (type) {
        case RoadType::Motorway:    return 14.0f;
        case RoadType::Trunk:       return 10.5f;
        case RoadType::Primary:     return 10.0f;
        case RoadType::Secondary:   return 8.0f;
        case RoadType::Tertiary:    return 7.0f;
        case RoadType::Residential: return 6.0f;
        case RoadType::Service:     return 4.0f;
        case RoadType::Footway:     return 2.0f;
        case RoadType::Cycleway:    return 2.5f;
        case RoadType::Path:        return 1.5f;
        case RoadType::Unknown:     return 6.0f;
    }
    return 6.0f;
}

// ============================================================================
// Surface
// ============================================================================

/**
 * @brief Whether a surface=* value names an unpaved running surface
 *
 * The rural strip rules key off this rather than off the material, because two
 * different questions are being asked. surface_material() picks a texture;
 * this decides whether the road has a constructed edge at all. A gravel lane
 * and a grass track both end in soft ground and need a verge to blend into it;
 * a paving-stone lane does not, even though neither is asphalt.
 *
 * An ABSENT surface tag is not unpaved. Most OSM ways carry no surface tag and
 * most of those are paved, so treating absence as unpaved would grow verges on
 * the whole network.
 *
 * @param surface Lowercased surface=* value; empty when the tag is absent
 * @return True when the value names an unpaved family
 */
[[nodiscard]] bool surface_is_unpaved(const std::string& surface) {
    if (surface.empty()) return false;

    return surface == "unpaved" || surface == "gravel" || surface == "fine_gravel" ||
           surface == "compacted" || surface == "pebblestone" || surface == "rock" ||
           surface == "stone" || surface == "chippings" || surface == "shells" ||
           surface == "dirt" || surface == "ground" || surface == "earth" ||
           surface == "mud" || surface == "sand" || surface == "clay" ||
           surface == "soil" || surface == "woodchips" || surface == "grass" ||
           surface == "grass_paver";
}

/**
 * @brief Map a raw surface=* value onto a material slot
 *
 * The mapping is deliberately coarse. A game map needs one texture per surface
 * family, not the several hundred distinct surface values OSM carries. The
 * families are: bituminous, cementitious, laid paving units, loose aggregate,
 * bare soil, and vegetation.
 *
 * Paving units -- setts, cobbles, bricks, slabs -- map to MaterialId::Sidewalk
 * because that is the paving-unit texture in the material set, whatever the
 * strip using it happens to be. A cobbled carriageway is cobbles.
 *
 * @param surface Lowercased surface=* value; empty when the tag is absent
 * @param fallback Material to use for an absent or unrecognised value
 */
[[nodiscard]] MaterialId surface_material(const std::string& surface, MaterialId fallback) {
    if (surface.empty()) return fallback;

    if (surface == "asphalt" || surface == "paved" || surface == "chipseal" ||
        surface == "bitmac" || surface == "tarmac" || surface == "asphalt:lanes")
        return MaterialId::Asphalt;
    // concrete:plates is cast concrete laid in bays, not asphalt.
    if (surface == "concrete" || surface == "concrete:plates" ||
        surface == "concrete:lanes" || surface == "cement")
        return MaterialId::Concrete;
    if (surface == "paving_stones" || surface == "sett" || surface == "cobblestone" ||
        surface == "unhewn_cobblestone" || surface == "bricks" || surface == "brick" ||
        surface == "metal" || surface == "wood")
        return MaterialId::Sidewalk;
    if (surface == "gravel" || surface == "compacted" || surface == "fine_gravel" ||
        surface == "pebblestone" || surface == "unpaved" || surface == "rock" ||
        surface == "stone" || surface == "chippings" || surface == "shells")
        return MaterialId::Gravel;
    if (surface == "dirt" || surface == "ground" || surface == "earth" ||
        surface == "mud" || surface == "sand" || surface == "clay" ||
        surface == "soil" || surface == "woodchips")
        return MaterialId::Dirt;
    if (surface == "grass" || surface == "grass_paver")
        return MaterialId::Grass;

    return fallback;
}

// ============================================================================
// Strip Assembly
// ============================================================================

/**
 * @brief Append one strip, dropping it when it would contribute nothing
 *
 * A zero-width strip is dropped unless it changes height, which is the
 * perfectly vertical curb face produced when curb_face_batter is zero. That one
 * must survive: dropping it would put a raised strip next to a strip at road
 * level and break the height invariant.
 */
void push_strip(std::vector<Strip>& out, float width, float height_left, float height_right,
                MaterialId material, StripKind kind) {
    const bool changes_height = std::fabs(height_right - height_left) > kHeightEpsilon;
    if (!(width > 0.0f) && !changes_height) return;

    Strip s;
    s.width = std::max(0.0f, width);
    s.height_left = height_left;
    s.height_right = height_right;
    s.material = material;
    s.kind = kind;
    out.push_back(s);
}

/// Append a strip that is level across its width
void push_flat(std::vector<Strip>& out, float width, float height,
               MaterialId material, StripKind kind) {
    push_strip(out, width, height, height, material, kind);
}

/**
 * @brief Append the LEFT-hand curb: top first, then a face falling to the road
 *
 * Walking left to right on the left of the road descends, so the face runs from
 * curb_height down to 0 and the batter leans the top outward, which is to the
 * left. push_curb_right() is the mirror image of this, and the pair is why the
 * profile never needs a per-side sign flip anywhere else.
 */
void push_curb_left(std::vector<Strip>& out, const ProfileConfig& cfg) {
    push_flat(out, cfg.curb_top_width, cfg.curb_height, MaterialId::Curb, StripKind::CurbTop);
    push_strip(out, cfg.curb_face_batter, cfg.curb_height, 0.0f,
               MaterialId::Curb, StripKind::CurbFace);
}

/// Append the RIGHT-hand curb: a face rising from the road, then the top
void push_curb_right(std::vector<Strip>& out, const ProfileConfig& cfg) {
    push_strip(out, cfg.curb_face_batter, 0.0f, cfg.curb_height,
               MaterialId::Curb, StripKind::CurbFace);
    push_flat(out, cfg.curb_top_width, cfg.curb_height, MaterialId::Curb, StripKind::CurbTop);
}

// ============================================================================
// Lane Resolution
// ============================================================================

/**
 * @brief Total running lanes across the carriageway
 *
 * lanes:forward plus lanes:backward wins when both are present, because the pair
 * is more often maintained than a bare lanes=* on a road that has been surveyed
 * in detail. A one-way way is NOT halved: OSM lanes=* on a one-way already
 * counts only the lanes that exist.
 */
[[nodiscard]] int resolve_lane_count(const GraphEdge& edge) {
    if (edge.lanes_forward > 0 && edge.lanes_backward > 0) {
        return edge.lanes_forward + edge.lanes_backward;
    }
    return std::max(1, edge.lanes);
}

/**
 * @brief Per-lane width, honouring a surveyed width=* over the class default
 *
 * A width tag describes the carriageway, not one lane, so it is distributed
 * across the lane count rather than replacing it. Two widths are rejected first:
 * the ones OSMParser synthesizes from lanes=* and from its own class table. Both
 * reach GraphEdge::width indistinguishable from a surveyed value, and treating
 * either as surveyed would silently rescale every untagged road.
 */
[[nodiscard]] float resolve_lane_width(const GraphEdge& edge, const ClassRules& r,
                                       int lane_count) {
    if (lane_count <= 0) return r.lane_width;

    const float w = edge.width;
    if (!std::isfinite(w) || w <= 0.0f) return r.lane_width;

    const float lanes_now = static_cast<float>(lane_count);
    const float lanes_parsed = static_cast<float>(std::max(1, edge.lanes));
    if (std::fabs(w - lanes_parsed * kParserMetresPerLane) < kWidthEpsilon) return r.lane_width;
    if (std::fabs(w - lanes_now * kParserMetresPerLane) < kWidthEpsilon) return r.lane_width;
    if (std::fabs(w - parser_default_width(edge.type)) < kWidthEpsilon) return r.lane_width;
    if (std::fabs(w - lanes_now * r.lane_width) < kWidthEpsilon) return r.lane_width;

    const float per_lane = w / lanes_now;
    if (per_lane < kMinLaneWidth || per_lane > kMaxLaneWidth) return r.lane_width;
    return per_lane;
}

/**
 * @brief Per-lane widths from width:lanes=*, for example "3|3|4.5"
 *
 * Applied left to right across the running lanes. OSM orders the list by lane
 * order, which is direction dependent, and the profile has no driving side, so
 * left-to-right is the only order available here.
 *
 * @param tags Way tags, may be null
 * @param lane_count Number of running lanes the profile will emit
 * @param fallback Width for every lane when the tag is absent or does not match
 * @return One width per lane, always of size lane_count
 */
[[nodiscard]] std::vector<float> resolve_per_lane_widths(const TagMap* tags, int lane_count,
                                                         float fallback) {
    std::vector<float> widths(static_cast<size_t>(std::max(0, lane_count)), fallback);

    const std::string* value = find_tag(tags, "width:lanes");
    if (value == nullptr || widths.empty()) return widths;

    std::vector<float> parsed;
    size_t start = 0;
    while (start <= value->size()) {
        const size_t bar = value->find('|', start);
        const size_t end = (bar == std::string::npos) ? value->size() : bar;
        const float metres = parse_metres(value->substr(start, end - start));
        parsed.push_back(metres);
        if (bar == std::string::npos) break;
        start = bar + 1;
    }

    if (parsed.size() != widths.size()) return widths;
    for (size_t i = 0; i < widths.size(); ++i) {
        if (parsed[i] >= kMinLaneWidth && parsed[i] <= kMaxLaneWidth) {
            widths[i] = parsed[i];
        }
    }
    return widths;
}

/**
 * @brief Whether a busway lane exists on a given side
 *
 * Only the side-scoped spellings busway:left, busway:right, and busway:both are
 * read. A bare busway=lane names no side, and guessing one would widen the
 * carriageway on whichever side happens to be wrong half the time.
 */
[[nodiscard]] bool has_busway(const TagMap* tags, bool left) {
    const char* side_key = left ? "busway:left" : "busway:right";
    if (const std::string* value = find_tag(tags, side_key)) {
        if (!tag_denies(*value)) return true;
    }
    if (const std::string* value = find_tag(tags, "busway:both")) {
        if (!tag_denies(*value)) return true;
    }
    return false;
}

} // namespace

// ============================================================================
// Names
// ============================================================================

const char* strip_kind_name(StripKind kind) {
    switch (kind) {
        case StripKind::Lane:        return "Lane";
        case StripKind::Gutter:      return "Gutter";
        case StripKind::CurbFace:    return "CurbFace";
        case StripKind::CurbTop:     return "CurbTop";
        case StripKind::Sidewalk:    return "Sidewalk";
        case StripKind::Shoulder:    return "Shoulder";
        case StripKind::Median:      return "Median";
        case StripKind::Verge:       return "Verge";
        case StripKind::CycleLane:   return "CycleLane";
        case StripKind::ParkingLane: return "ParkingLane";
        case StripKind::Count:       return "Unknown";
    }
    return "Unknown";
}

// ============================================================================
// RoadProfile
// ============================================================================

float RoadProfile::total_width() const {
    float sum = 0.0f;
    for (const Strip& s : strips) {
        sum += s.width;
    }
    return sum;
}

float RoadProfile::carriageway_width() const {
    float sum = 0.0f;
    for (const Strip& s : strips) {
        if (s.kind == StripKind::Lane) {
            sum += s.width;
        }
    }
    return sum;
}

float RoadProfile::left_edge_offset() const {
    if (strips.empty()) return 0.0f;

    // Inclusive span from the first Lane-or-Median strip to the last one. Strips
    // between them count whatever their kind: a gutter or curb inside a dual
    // carriageway belongs to the carriageway envelope, not outside it.
    const size_t none = strips.size();
    size_t first = none;
    size_t last = 0;
    for (size_t i = 0; i < strips.size(); ++i) {
        if (strips[i].kind == StripKind::Lane || strips[i].kind == StripKind::Median) {
            if (first == none) first = i;
            last = i;
        }
    }

    // No carriageway at all -- a bare footway or cycleway -- so centre the whole
    // profile instead.
    if (first == none) return total_width() * 0.5f;

    float span = 0.0f;
    for (size_t i = first; i <= last; ++i) {
        span += strips[i].width;
    }

    float outboard = 0.0f;
    for (size_t i = 0; i < first; ++i) {
        outboard += strips[i].width;
    }

    return span * 0.5f + outboard;
}

bool RoadProfile::is_valid() const {
    if (strips.empty()) return false;

    for (const Strip& s : strips) {
        if (!std::isfinite(s.width) || s.width < 0.0f) return false;
        if (!std::isfinite(s.height_left) || !std::isfinite(s.height_right)) return false;
    }

    // The height invariant. Every legitimate height change is its own CurbFace
    // strip, so any gap here is a strip rule that forgot one.
    for (size_t i = 0; i + 1 < strips.size(); ++i) {
        if (std::fabs(strips[i].height_right - strips[i + 1].height_left) > kHeightEpsilon) {
            return false;
        }
    }

    return true;
}

// ============================================================================
// build_profile
// ============================================================================

RoadProfile build_profile(const GraphEdge& edge, const ProfileConfig& cfg, const TagMap* tags,
                          SideFlags suppress_sidewalk) {
    RoadProfile profile;
    std::vector<Strip>& out = profile.strips;

    ClassRules rules = rules_for(edge.type, cfg);
    if (edge.type == RoadType::Service) {
        apply_service_rules(rules, read_service_kind(tags), cfg);
    }

    const MaterialId surface = surface_material(edge.surface, rules.default_material);
    const bool unpaved = surface_is_unpaved(edge.surface);

    // ------------------------------------------------------------------------
    // Single-strip classes
    // ------------------------------------------------------------------------
    // A footway, cycleway, or path is its own way with its own width. There is
    // no carriageway to sit beside, so there is no curb, no gutter, and no lane
    // subdivision. GraphEdge::width is used directly because the parser's class
    // defaults for these three are already the right order of magnitude. A track
    // is the one that is not literally single-strip: it carries a verge on each
    // side, which is class geometry rather than a tagged feature.
    if (rules.single_strip) {
        float width = edge.width;
        if (!std::isfinite(width) || width < kMinPathWidth || width > kMaxPathWidth) {
            width = parser_default_width(edge.type);
        }
        // The verge sits at running-surface height: there is no curb on a track,
        // so nothing lifts it and no face is needed to close a step.
        if (rules.has_verge) {
            push_flat(out, cfg.verge_width, 0.0f, MaterialId::Grass, StripKind::Verge);
        }
        push_flat(out, width, 0.0f, surface, rules.single_kind);
        if (rules.has_verge) {
            push_flat(out, cfg.verge_width, 0.0f, MaterialId::Grass, StripKind::Verge);
        }
        if (out.empty()) {
            push_flat(out, cfg.lane_width_default, 0.0f, surface, rules.single_kind);
        }
        return profile;
    }

    // ------------------------------------------------------------------------
    // Lanes
    // ------------------------------------------------------------------------
    const int lane_count = resolve_lane_count(edge);

    // lanes:both_ways is a central turning lane, and OSM's identity is
    // lanes = lanes:forward + lanes:backward + lanes:both_ways. So a bare lanes=*
    // ALREADY counts the turning lane, while resolve_lane_count()'s directional
    // branch does not. Adding a strip for it unconditionally makes the same
    // physical road come back one whole lane wider through the bare-lanes branch
    // than through the directional one, and widens Corridor::outline with it.
    const int both_ways = std::max(0, parse_int_tag(tags, "lanes:both_ways"));
    const bool count_excludes_both_ways = edge.lanes_forward > 0 && edge.lanes_backward > 0;

    // Every lane on the carriageway, turning lane included: what a width=* tag
    // describes and therefore what it is divided across.
    const int total_lanes = count_excludes_both_ways ? (lane_count + both_ways) : lane_count;

    // The running lanes alone: total minus the turning lane, never below one.
    const int running_lanes = std::max(1, total_lanes - both_ways);

    const float lane_width = resolve_lane_width(edge, rules, total_lanes);
    const std::vector<float> lane_widths =
        resolve_per_lane_widths(tags, running_lanes, lane_width);

    // A turning lane and a median cannot occupy the same ground.
    const MedianTag median_tag = read_median_tag(tags);
    const bool median_allowed = !edge.is_oneway && both_ways == 0 &&
                                median_tag != MedianTag::Deny;

    // The class rule needs the lane count as evidence; the tag is evidence on its
    // own, which is the only way a divided two-lane street -- one carriageway
    // each way with a refuge between them -- ever gets its island.
    const bool want_median = median_allowed &&
                             ((rules.allow_median && running_lanes >= 4) ||
                              median_tag == MedianTag::Painted ||
                              median_tag == MedianTag::Raised ||
                              median_tag == MedianTag::Present);

    // The turning lane goes in the MIDDLE of the carriageway, which is the only
    // place a centre turn lane exists, so the running lanes are split around it
    // exactly as they are split around a median.
    const size_t left_lanes = (want_median || both_ways > 0) ? lane_widths.size() / 2
                                                             : lane_widths.size();

    // ------------------------------------------------------------------------
    // Sides
    // ------------------------------------------------------------------------
    // SideFlags::Unknown means the tag was absent, so a class default may be
    // inferred. SideFlags::None means the tag said no, or said "separate" and the
    // footway is mapped as its own way. None is never overridden; that is what
    // stops the network growing a duplicate sidewalk beside a real one.
    bool sidewalk_left = false;
    bool sidewalk_right = false;
    if (rules.allow_sidewalk) {
        if (edge.sidewalk == SideFlags::Unknown) {
            const bool synth = cfg.synthesize_sidewalks && rules.synth_sidewalk;
            sidewalk_left = synth;
            sidewalk_right = synth;
        } else {
            sidewalk_left = side_has_left(edge.sidewalk);
            sidewalk_right = side_has_right(edge.sidewalk);
        }
    }

    // The dedup mask from sidewalk_dedup.hpp: this side already has a surveyed
    // footway of its own, so synthesizing one here would build the same sidewalk
    // twice, a metre apart. Applied AFTER the tag and the class default rather
    // than folded into edge.sidewalk, which makes it strictly subtractive: a
    // stale or wrong mask can only cost a sidewalk, never invent one.
    if (side_has_left(suppress_sidewalk)) sidewalk_left = false;
    if (side_has_right(suppress_sidewalk)) sidewalk_right = false;

    // A sidewalk always brings its curb. The curb is what carries the sidewalk
    // from road level up to its own, so emitting one without the other would
    // leave a height discontinuity with no face to close it.
    const bool curb_left = sidewalk_left;
    const bool curb_right = sidewalk_right;

    const bool cycle_left = side_has_left(edge.cycleway);
    const bool cycle_right = side_has_right(edge.cycleway);
    const bool parking_left = side_has_left(edge.parking);
    const bool parking_right = side_has_right(edge.parking);

    const bool shoulder_left = rules.has_shoulder || side_has_left(edge.shoulder);
    const bool shoulder_right = rules.has_shoulder || side_has_right(edge.shoulder);

    const bool busway_left = has_busway(tags, true);
    const bool busway_right = has_busway(tags, false);

    // An unpaved carriageway has no constructed edge, so without a verge it ends
    // in a hard line against the terrain. Only where nothing else already closes
    // that edge: a curb carries the surface up to a sidewalk and a shoulder runs
    // out to the verge's own material, so neither needs one.
    const bool rural_verge = cfg.rural_verges && rules.rural_verge && unpaved;

    const SideFlags verge = read_sided_tag(tags, "verge");
    const bool verge_left = (verge == SideFlags::Unknown)
                                ? (rural_verge && !curb_left && !shoulder_left)
                                : side_has_left(verge);
    const bool verge_right = (verge == SideFlags::Unknown)
                                 ? (rural_verge && !curb_right && !shoulder_right)
                                 : side_has_right(verge);

    // A verge outboard of a sidewalk sits at sidewalk level, so its own outer
    // edge needs no extra face. Without a sidewalk it stays at road level.
    const float verge_left_height = sidewalk_left ? cfg.curb_height : 0.0f;
    const float verge_right_height = sidewalk_right ? cfg.curb_height : 0.0f;

    // ------------------------------------------------------------------------
    // Left shoulder of the profile, outermost first
    // ------------------------------------------------------------------------
    if (verge_left) {
        push_flat(out, cfg.verge_width, verge_left_height, MaterialId::Grass, StripKind::Verge);
    }
    if (sidewalk_left) {
        push_flat(out, cfg.sidewalk_width, cfg.curb_height,
                  MaterialId::Sidewalk, StripKind::Sidewalk);
    }
    if (curb_left) {
        push_curb_left(out, cfg);
    }
    if (rules.has_gutter) {
        push_flat(out, cfg.gutter_width, 0.0f, MaterialId::Concrete, StripKind::Gutter);
    }
    if (parking_left) {
        push_flat(out, cfg.parking_lane_width, 0.0f, surface, StripKind::ParkingLane);
    }
    if (cycle_left) {
        push_flat(out, cfg.cycle_lane_width, 0.0f, surface, StripKind::CycleLane);
    }
    if (shoulder_left) {
        push_flat(out, cfg.shoulder_width, 0.0f, surface, StripKind::Shoulder);
    }
    if (busway_left) {
        push_flat(out, lane_width, 0.0f, surface, StripKind::Lane);
    }

    // ------------------------------------------------------------------------
    // Carriageway
    // ------------------------------------------------------------------------
    for (size_t i = 0; i < left_lanes; ++i) {
        push_flat(out, lane_widths[i], 0.0f, surface, StripKind::Lane);
    }

    if (want_median) {
        // The tag is explicit evidence and wins outright. Otherwise the width
        // decides, against a threshold that depends on where the road is: a rural
        // dual carriageway with a 1 m gap has a painted separator, while an urban
        // primary with the same gap has a kerbed refuge, because that is what
        // pedestrians cross in two stages.
        bool raised = false;
        if (median_tag == MedianTag::Painted) {
            raised = false;
        } else if (median_tag == MedianTag::Raised) {
            raised = true;
        } else {
            const float threshold =
                rules.urban_median ? cfg.min_median_raised_urban : cfg.min_median_raised;
            raised = cfg.median_width >= threshold;
        }

        if (raised) {
            // Raised island: a curb face on each side, and between them grass when
            // there is room to plant it, paving when it is only a refuge.
            const MaterialId top = (cfg.median_width >= cfg.min_median_grass)
                                       ? MaterialId::Grass
                                       : MaterialId::Concrete;
            push_strip(out, cfg.curb_face_batter, 0.0f, cfg.curb_height,
                       MaterialId::Curb, StripKind::CurbFace);
            push_flat(out, cfg.median_width, cfg.curb_height, top, StripKind::Median);
            push_strip(out, cfg.curb_face_batter, cfg.curb_height, 0.0f,
                       MaterialId::Curb, StripKind::CurbFace);
        } else {
            // Painted median: hatching on the carriageway surface, not an island.
            push_flat(out, cfg.median_width, 0.0f, MaterialId::Asphalt, StripKind::Median);
        }
    }

    for (int i = 0; i < both_ways; ++i) {
        push_flat(out, lane_width, 0.0f, surface, StripKind::Lane);
    }

    for (size_t i = left_lanes; i < lane_widths.size(); ++i) {
        push_flat(out, lane_widths[i], 0.0f, surface, StripKind::Lane);
    }

    // ------------------------------------------------------------------------
    // Right shoulder of the profile, innermost first
    // ------------------------------------------------------------------------
    if (busway_right) {
        push_flat(out, lane_width, 0.0f, surface, StripKind::Lane);
    }
    if (shoulder_right) {
        push_flat(out, cfg.shoulder_width, 0.0f, surface, StripKind::Shoulder);
    }
    if (cycle_right) {
        push_flat(out, cfg.cycle_lane_width, 0.0f, surface, StripKind::CycleLane);
    }
    if (parking_right) {
        push_flat(out, cfg.parking_lane_width, 0.0f, surface, StripKind::ParkingLane);
    }
    if (rules.has_gutter) {
        push_flat(out, cfg.gutter_width, 0.0f, MaterialId::Concrete, StripKind::Gutter);
    }
    if (curb_right) {
        push_curb_right(out, cfg);
    }
    if (sidewalk_right) {
        push_flat(out, cfg.sidewalk_width, cfg.curb_height,
                  MaterialId::Sidewalk, StripKind::Sidewalk);
    }
    if (verge_right) {
        push_flat(out, cfg.verge_width, verge_right_height,
                  MaterialId::Grass, StripKind::Verge);
    }

    // ------------------------------------------------------------------------
    // Degenerate fallback
    // ------------------------------------------------------------------------
    // Every edge must come back extrudable, including one whose every width
    // config was set to zero. One default lane is always better than a profile
    // the corridor builder silently skips.
    if (!profile.is_valid()) {
        out.clear();
        push_flat(out, cfg.lane_width_default * static_cast<float>(std::max(1, total_lanes)),
                  0.0f, surface, StripKind::Lane);
        if (out.empty()) {
            push_flat(out, 3.5f, 0.0f, surface, StripKind::Lane);
        }
    }

    return profile;
}

} // namespace stratum::osm::road
