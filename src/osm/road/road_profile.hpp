/**
 * @file road_profile.hpp
 * @brief Tag-driven cross-section model for one road edge
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * A RoadProfile is the lateral cross-section of a road: an ordered list of
 * strips running left to right across the carriageway and its surroundings.
 * The corridor extruder sweeps this cross-section along a Centerline, so every
 * road feature -- gutter, curb, sidewalk, median, verge, cycle lane, parking
 * bay -- is a strip rule here rather than new mesh code in corridor.cpp.
 *
 * Left and right are relative to the direction of travel along the edge, that
 * is, from GraphEdge::from towards GraphEdge::to. They are not compass
 * directions and they do not depend on which side of the road traffic drives.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API. renderer/mesh.hpp is included for MaterialId only; that header is pure
 * glm and is already compiled into stratum_core.
 */

#pragma once

#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Strips
// ============================================================================

/**
 * @brief What a strip represents, independent of the material it is built from
 *
 * The kind drives behaviour -- which strips count as carriageway, which get a
 * curb face, which markings attach to which boundary -- while MaterialId drives
 * appearance. A Lane may be asphalt, concrete, gravel, or dirt and is still a
 * Lane.
 *
 * @note Count is a sentinel for array sizing and iteration bounds. It is never a
 *       valid kind for a Strip.
 */
enum class StripKind : uint8_t {
    Lane,           ///< Running lane of the carriageway
    Gutter,         ///< Drainage channel between the outer lane and the curb
    CurbFace,       ///< Vertical or near-vertical riser at a height change
    CurbTop,        ///< Horizontal top of a curb, between face and sidewalk
    Sidewalk,       ///< Raised footway beside the carriageway
    Shoulder,       ///< Unmarked hard strip beside the carriageway
    Median,         ///< Central reservation, painted or raised
    Verge,          ///< Soft planted strip outside the paved surface
    CycleLane,      ///< Dedicated cycle lane, on or beside the carriageway
    ParkingLane,    ///< On-street parking bay
    Count           ///< Sentinel: number of strip kinds. Not a valid kind.
};

/**
 * @brief Convert a StripKind to a stable human-readable string
 *
 * Used for logging, test failure messages, and editor debug overlays. The
 * returned pointer is a string literal with static storage duration.
 *
 * @param kind Strip kind to name
 * @return Name of the kind, or "Unknown" for StripKind::Count and out-of-range values
 */
[[nodiscard]] const char* strip_kind_name(StripKind kind);

/**
 * @brief One lateral band of the cross-section
 *
 * Strips are ordered LEFT to RIGHT relative to the direction of travel along
 * the way. Lateral coordinates are signed and positive to the LEFT, so walking
 * the strips in order walks the lateral coordinate DOWNWARDS:
 *
 * @code
 *     double lateral = profile.left_edge_offset();
 *     for (const Strip& s : profile.strips) {
 *         // s spans [lateral - s.width, lateral], left edge first
 *         lateral -= s.width;
 *     }
 * @endcode
 *
 * Heights are metres above the carriageway surface, not above the world origin.
 * The corridor extruder adds the per-station carriageway height on top.
 *
 * Adjacent strips must agree at their shared boundary:
 * `strips[i].height_right == strips[i + 1].height_left`. Any height change is
 * therefore an explicit CurbFace strip, which may be zero-width when the riser
 * is perfectly vertical.
 */
struct Strip {
    float width = 0.0f;         ///< lateral extent in metres; a pure vertical curb face may be 0
    float height_left = 0.0f;   ///< metres above the carriageway surface at this strip's left edge
    float height_right = 0.0f;  ///< metres above the carriageway surface at this strip's right edge
    MaterialId material = MaterialId::Asphalt;
    StripKind kind = StripKind::Lane;

    /**
     * @brief Which variant of @ref material this strip wants
     *
     * The second axis of the material key. @ref material says "this is a running
     * surface"; the variant says WHICH running surface, so a cobbled street and
     * an asphalt one occupy the same SubMesh slot and still render differently.
     *
     * Zero is the slot's default and is what a profile built by hand carries, so
     * every producer written before the variant axis existed keeps exactly the
     * appearance it had. build_profile() fills it from the way's tags through
     * road_style.hpp, which is the only place in the build allowed to decide a
     * variant.
     *
     * Declared LAST on purpose. Strip is an aggregate and is brace-initialised
     * positionally all over the tests and the fixtures, so a new field anywhere
     * but the end would silently rebind `kind` to a number.
     *
     * @note The PAIR is the lookup key. Variant 3 of MaterialId::Asphalt and
     *       variant 3 of MaterialId::Sidewalk are unrelated; see @ref key().
     */
    uint16_t variant = 0;

    /// The (slot, variant) pair the corridor extruder tags this strip's SubMesh with
    [[nodiscard]] MaterialKey key() const { return MaterialKey{ material, variant }; }
};

// ============================================================================
// Profile
// ============================================================================

/**
 * @brief The complete cross-section of one road edge
 *
 * Examples, left to right:
 *
 * - Residential: sidewalk 2.0 @ +0.15, curb top 0.15 @ +0.15, curb face @ 0.15 -> 0,
 *   gutter 0.3, lane 3.5, lane 3.5, gutter 0.3, curb face @ 0 -> 0.15,
 *   curb top 0.15 @ +0.15, sidewalk 2.0 @ +0.15
 * - Motorway: shoulder 2.5, lane 3.75 x N, median 3.0, lane 3.75 x N, shoulder 2.5
 * - Rural track: verge 1.0, dirt lane 2.5, verge 1.0
 * - Footway: sidewalk 2.0, nothing else
 */
struct RoadProfile {
    /// Strips ordered left to right; strips[i].height_right == strips[i+1].height_left
    std::vector<Strip> strips;

    /// Sum of every strip width in metres
    [[nodiscard]] float total_width() const;

    /// Sum of Lane widths only, ignoring shoulders, medians, and everything outboard
    [[nodiscard]] float carriageway_width() const;

    /**
     * @brief Signed lateral coordinate of the leftmost strip's left edge, positive to the left
     *
     * The OSM way is the centreline of the CARRIAGEWAY, not of the total profile.
     * A road with a sidewalk on one side only must therefore sit off-centre inside
     * its own profile, otherwise the painted surface drifts away from the surveyed
     * geometry.
     *
     * The offset is derived by centring the Lane+Median span on zero, then walking
     * outward:
     *
     * 1. Let [first, last] be the inclusive index range from the first strip whose
     *    kind is Lane or Median to the last such strip. Every strip between them
     *    counts towards the span even if it is neither, since a gutter or curb
     *    inside a dual carriageway is part of the carriageway envelope.
     * 2. Let span be the sum of widths over [first, last]. Its left edge sits at
     *    +span/2.
     * 3. Add the widths of every strip before `first`.
     *
     * When the profile contains no Lane and no Median strip -- a bare footway, for
     * instance -- the whole profile is centred instead, so the result is
     * total_width() / 2.
     *
     * @return Lateral coordinate of strips.front()'s left edge; 0 for an empty profile
     */
    [[nodiscard]] float left_edge_offset() const;

    /**
     * @brief Whether the profile is fit to extrude
     *
     * True when the profile is non-empty, every width is finite and >= 0, every
     * height is finite, and adjacent strips agree at their shared boundary to
     * within 1e-4 m. An invalid profile is skipped by the corridor builder rather
     * than producing folded or cracked geometry.
     */
    [[nodiscard]] bool is_valid() const;
};

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Tunable widths and heights used when tags do not resolve a value
 *
 * Every field is metres unless stated otherwise. Values here are defaults for
 * absent tags; an explicit OSM tag always wins over the corresponding default.
 */
struct ProfileConfig {
    float lane_width_default = 3.5f;      ///< when no width/lanes tag resolves one
    float motorway_lane_width = 3.75f;
    float service_lane_width = 3.0f;      ///< driveways and parking aisles, narrower than a street
    float sidewalk_width = 2.0f;
    float curb_height = 0.15f;
    float curb_top_width = 0.15f;         ///< flat top of the curb, between its face and the sidewalk
    float curb_face_batter = 0.02f;       ///< slight outward lean of the curb face, metres
    float gutter_width = 0.3f;
    float shoulder_width = 2.5f;
    float median_width = 3.0f;
    float verge_width = 1.0f;
    float cycle_lane_width = 1.5f;
    float parking_lane_width = 2.2f;
    float min_median_raised = 1.5f;       ///< medians at least this wide become raised islands
    bool  synthesize_sidewalks = true;    ///< infer sidewalks from road class when tag is Unknown

    // ------------------------------------------------------------------------
    // Service roads
    // ------------------------------------------------------------------------
    // highway=service covers everything from a farm access to a supermarket
    // aisle, and the service=* subtag is the only thing that separates them. A
    // driveway built at street width swallows the front garden it runs through.

    float driveway_width = 3.0f;          ///< service=driveway: one car, no kerb
    float parking_aisle_width = 5.5f;     ///< service=parking_aisle: two-way between bays
    float alley_width = 3.5f;             ///< service=alley: rear access between plots

    // ------------------------------------------------------------------------
    // Rural detail
    // ------------------------------------------------------------------------

    /**
     * @brief Synthesise a grass verge beside an unpaved road that has no kerb
     *
     * An unpaved carriageway has no edge to speak of. Without a verge it ends in
     * a hard line against the terrain, which is the same defect the plan's rural
     * track profile exists to fix -- tracks already carry one from their class.
     * A verge=* tag always wins over this, denials included.
     */
    bool rural_verges = true;

    // ------------------------------------------------------------------------
    // Medians and traffic islands
    // ------------------------------------------------------------------------

    /**
     * @brief Raised-island threshold on a kerbed street, metres
     *
     * Separate from min_median_raised, and lower. A rural dual carriageway with a
     * 1 m gap between its carriageways has a painted separator; an urban primary
     * with the same gap has a kerbed refuge, because that is what pedestrians
     * cross in two stages. One threshold cannot serve both.
     */
    float min_median_raised_urban = 1.0f;

    /**
     * @brief Widest raised median still surfaced as paving rather than grass, metres
     *
     * A wide central reservation is planted; a narrow refuge is a concrete island
     * with a kerb round it. Below this width a raised median is Concrete, at or
     * above it Grass.
     */
    float min_median_grass = 2.0f;

    /**
     * @brief Allow an untagged median on a wide two-way primary
     *
     * Motorways and trunk roads get one from their class. A primary needs the
     * lane count as evidence, since a four-lane primary is a dual carriageway far
     * more often than it is four undivided lanes. Secondary and below need an
     * explicit median=* or divider=* tag.
     */
    bool urban_dual_median = true;
};

/**
 * @brief Build the cross-section for one graph edge
 *
 * Reads the tag-derived fields already promoted onto GraphEdge -- type, width,
 * lanes, lanes_forward, lanes_backward, is_oneway, is_link, surface, and the
 * SideFlags family -- and turns them into an ordered strip list.
 *
 * Rules the corridor and the tests rely on:
 *
 * - The returned profile is valid (is_valid() is true) for any edge, including a
 *   degenerate one. An edge with no usable width falls back to
 *   `lane_width_default * max(1, lanes)` of Lane strips.
 * - Every height change is an explicit CurbFace strip, so height continuity holds
 *   by construction.
 * - SideFlags::Unknown may be filled in from the road class, but only when the
 *   matching ProfileConfig switch allows it. SideFlags::None is never overridden:
 *   the tag said no.
 * - A Median strip at least min_median_raised wide -- min_median_raised_urban on a
 *   kerbed street -- is emitted raised, as CurbFace + Median top at curb_height +
 *   CurbFace. A narrower one stays flat at height 0 and is a painted median on the
 *   carriageway surface. `median=*` and `divider=*` override the width rule in
 *   either direction, and deny a median outright when negative.
 * - Curb faces lean outward by curb_face_batter, so their width is
 *   curb_face_batter rather than 0 whenever that value is positive.
 * - `service=*` selects the service-road width: driveway_width, parking_aisle_width,
 *   or alley_width, none of which carry a kerb, a gutter, or a sidewalk.
 * - An unpaved road with no kerb and no shoulder on a side grows a Verge there,
 *   unless a verge=* tag says otherwise. See ProfileConfig::rural_verges.
 * - Every strip carries a Strip::variant resolved through road_style.hpp, so a
 *   cobbled street and an asphalt one differ in appearance without differing in
 *   MaterialId. The slot is decided here and the variant there; nothing else in
 *   the pipeline invents a variant. @p tags is what carries `smoothness=*`,
 *   `kerb:material=*` and `footway:surface=*` into that decision, so a null
 *   @p tags degrades to slot defaults rather than to a wrong appearance.
 *
 * @param edge Graph edge to build the cross-section for
 * @param cfg  Widths and heights for values no tag resolves
 * @param tags Optional raw way tags for rare features not promoted onto GraphEdge.
 *             ParsedOSMData::ways holds them keyed by GraphEdge::source_way. May be null.
 * @param suppress_sidewalk Sides whose sidewalk is already mapped as its own OSM
 *             way, from DedupResult::suppress_side in sidewalk_dedup.hpp. Purely
 *             SUBTRACTIVE: it can remove a sidewalk the tags or the class default
 *             asked for, and can never add one, so passing a stale or wrong mask
 *             degrades to a missing sidewalk rather than to a duplicated one.
 *             SideFlags::None and SideFlags::Unknown both suppress nothing.
 * @return The cross-section, ordered left to right
 */
[[nodiscard]] RoadProfile build_profile(const GraphEdge& edge,
                                        const ProfileConfig& cfg,
                                        const TagMap* tags = nullptr,
                                        SideFlags suppress_sidewalk = SideFlags::None);

} // namespace stratum::osm::road
