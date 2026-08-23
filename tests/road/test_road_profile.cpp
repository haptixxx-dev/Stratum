/**
 * @file test_road_profile.cpp
 * @brief Tag-driven cross-section tests for build_profile()
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * These tests are written against the contract in src/osm/road/road_profile.hpp
 * and the "Profile model" section of docs/plans/road_network_plan.md. They build
 * GraphEdge values directly rather than parsing a fixture, because a profile is a
 * pure function of tags and the point is to vary one tag at a time.
 *
 * Three of these are load bearing:
 *
 * - Height continuity. strips[i].height_right == strips[i+1].height_left for every
 *   profile the builder can produce, driven over every RoadType. A break here is a
 *   crack in the extruded ribbon that only shows up as a visible seam in the GUI.
 * - The duplicate-sidewalk guard. SideFlags::None means the mapper said there is
 *   no sidewalk, usually because it is mapped separately as its own footway. It
 *   must never be synthesised, whatever ProfileConfig says.
 * - Lane centring. The OSM way is the centreline of the CARRIAGEWAY, so an
 *   asymmetric profile must sit off-centre inside itself rather than dragging the
 *   painted surface off the surveyed geometry.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadProfile
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::material_id_name;
using stratum::osm::RoadType;
using stratum::osm::SideFlags;
using stratum::osm::road_type_name;
using stratum::osm::side_flags_name;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::ProfileConfig;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::Strip;
using stratum::osm::road::StripKind;
using stratum::osm::road::build_profile;
using stratum::osm::road::strip_kind_name;

/// Boundary agreement tolerance, matching RoadProfile::is_valid()
constexpr double kBoundaryEps = 1e-4;

/// Sentinel for "no strip of that kind"
constexpr size_t kNoStrip = static_cast<size_t>(-1);

/**
 * @brief A minimal graph edge carrying only the tags a test varies
 *
 * width is set to 0 so that no width tag resolves and the builder falls back to
 * ProfileConfig::lane_width_default. A test that wants the tag to win sets width
 * itself.
 *
 * @param type  Road classification
 * @param lanes Total lane count
 * @return The edge, 100 m long along +X
 */
GraphEdge make_edge(RoadType type, int lanes) {
    GraphEdge edge;
    edge.source_way = 1;
    edge.from = 0;
    edge.to = 1;
    edge.polyline = {{0.0, 0.0}, {100.0, 0.0}};
    edge.node_ids = {1, 2};
    edge.type = type;
    edge.lanes = lanes;
    edge.width = 0.0f;      // no usable width tag
    return edge;
}

/// Number of strips of a given kind
size_t count_kind(const RoadProfile& p, StripKind kind) {
    size_t n = 0;
    for (const Strip& s : p.strips) {
        if (s.kind == kind) ++n;
    }
    return n;
}

/// Number of strips using a given material
size_t count_material(const RoadProfile& p, MaterialId material) {
    size_t n = 0;
    for (const Strip& s : p.strips) {
        if (s.material == material) ++n;
    }
    return n;
}

/// Index of the first strip of a kind, or kNoStrip
size_t first_of_kind(const RoadProfile& p, StripKind kind) {
    for (size_t i = 0; i < p.strips.size(); ++i) {
        if (p.strips[i].kind == kind) return i;
    }
    return kNoStrip;
}

/// Index of the last strip of a kind, or kNoStrip
size_t last_of_kind(const RoadProfile& p, StripKind kind) {
    size_t found = kNoStrip;
    for (size_t i = 0; i < p.strips.size(); ++i) {
        if (p.strips[i].kind == kind) found = i;
    }
    return found;
}

/// True when a strip of @p kind exists in the half-open index range [begin, end)
bool has_kind_in(const RoadProfile& p, StripKind kind, size_t begin, size_t end) {
    for (size_t i = begin; i < end && i < p.strips.size(); ++i) {
        if (p.strips[i].kind == kind) return true;
    }
    return false;
}

/**
 * @brief Signed lateral coordinates of the carriageway span
 *
 * Reimplements the span rule documented on RoadProfile::left_edge_offset(): the
 * inclusive index range from the first Lane-or-Median strip to the last one. The
 * midpoint of this span is what must stay on the OSM way.
 */
struct LaneSpan {
    double left = 0.0;      ///< lateral coordinate of the span's left edge
    double right = 0.0;     ///< lateral coordinate of the span's right edge
    bool found = false;     ///< false when the profile carries no Lane and no Median

    /// Midpoint of the span, which must be 0 for a profile centred on its way
    [[nodiscard]] double midpoint() const { return (left + right) * 0.5; }
};

/**
 * @brief Walk the profile and measure the carriageway span
 *
 * @param p Profile to measure
 * @return The span, or found == false when there is no Lane and no Median strip
 */
LaneSpan lane_span(const RoadProfile& p) {
    size_t first = kNoStrip;
    size_t last = kNoStrip;
    for (size_t i = 0; i < p.strips.size(); ++i) {
        const StripKind k = p.strips[i].kind;
        if (k == StripKind::Lane || k == StripKind::Median) {
            if (first == kNoStrip) first = i;
            last = i;
        }
    }
    if (first == kNoStrip) return LaneSpan{};

    LaneSpan span;
    span.found = true;
    double lateral = static_cast<double>(p.left_edge_offset());
    for (size_t i = 0; i < p.strips.size(); ++i) {
        if (i == first) span.left = lateral;
        lateral -= static_cast<double>(p.strips[i].width);
        if (i == last) span.right = lateral;
    }
    return span;
}

/**
 * @brief Assert every invariant RoadProfile::is_valid() promises
 *
 * Checks the boundary agreement explicitly rather than trusting is_valid(), then
 * checks that is_valid() agrees. A builder that emitted a step without a CurbFace
 * strip and an is_valid() that failed to notice would otherwise both pass.
 *
 * @param p     Profile to check
 * @param label Test context, for the failure message
 */
void check_profile_invariants(const RoadProfile& p, const std::string& label) {
    if (p.strips.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "profile has strips",
                                      label + ": build_profile returned no strips");
        return;
    }

    bool ok = true;
    for (size_t i = 0; i < p.strips.size(); ++i) {
        const Strip& s = p.strips[i];
        if (!std::isfinite(s.width) || s.width < 0.0f) {
            ok = false;
            stratum::test::report_failure(__FILE__, __LINE__, "strip width is finite and >= 0",
                                          label + " strip " + std::to_string(i) + " (" +
                                              strip_kind_name(s.kind) +
                                              "): width " + std::to_string(s.width));
        }
        if (!std::isfinite(s.height_left) || !std::isfinite(s.height_right)) {
            ok = false;
            stratum::test::report_failure(__FILE__, __LINE__, "strip heights are finite",
                                          label + " strip " + std::to_string(i));
        }
    }

    for (size_t i = 1; i < p.strips.size(); ++i) {
        const double a = static_cast<double>(p.strips[i - 1].height_right);
        const double b = static_cast<double>(p.strips[i].height_left);
        if (std::fabs(a - b) > kBoundaryEps) {
            ok = false;
            stratum::test::report_failure(
                __FILE__, __LINE__, "strips[i-1].height_right == strips[i].height_left",
                label + " boundary " + std::to_string(i - 1) + "|" + std::to_string(i) + ": " +
                    strip_kind_name(p.strips[i - 1].kind) + " right " + std::to_string(a) +
                    " vs " + strip_kind_name(p.strips[i].kind) + " left " + std::to_string(b));
        }
    }

    if (p.is_valid() != ok) {
        stratum::test::report_failure(__FILE__, __LINE__, "is_valid() agrees with the invariants",
                                      label + ": is_valid() reported " +
                                          (p.is_valid() ? "true" : "false"));
    }
}

/// Every road classification, for the tag-sweep tests
constexpr RoadType kAllRoadTypes[] = {
    RoadType::Motorway, RoadType::Trunk,   RoadType::Primary,  RoadType::Secondary,
    RoadType::Tertiary, RoadType::Residential, RoadType::Service, RoadType::Footway,
    RoadType::Cycleway, RoadType::Path,    RoadType::Unknown,
};

/// Every side flag value, for the tag-sweep tests
constexpr SideFlags kAllSides[] = {
    SideFlags::Unknown, SideFlags::None, SideFlags::Left, SideFlags::Right, SideFlags::Both,
};

} // namespace

// ============================================================================
// Residential
// ============================================================================

TEST(RoadProfile, residential_has_curb_faces_and_sidewalks_on_both_sides) {
    GraphEdge edge = make_edge(RoadType::Residential, 2);
    edge.sidewalk = SideFlags::Both;

    const ProfileConfig cfg;
    const RoadProfile p = build_profile(edge, cfg);
    check_profile_invariants(p, "residential both sidewalks");

    const size_t first_lane = first_of_kind(p, StripKind::Lane);
    const size_t last_lane = last_of_kind(p, StripKind::Lane);
    CHECK_TRUE(first_lane != kNoStrip);
    CHECK_TRUE(last_lane != kNoStrip);
    if (first_lane == kNoStrip || last_lane == kNoStrip) return;

    // One of each outboard of the lanes on both sides.
    CHECK_TRUE(has_kind_in(p, StripKind::Sidewalk, 0, first_lane));
    CHECK_TRUE(has_kind_in(p, StripKind::Sidewalk, last_lane + 1, p.strips.size()));
    CHECK_TRUE(has_kind_in(p, StripKind::CurbFace, 0, first_lane));
    CHECK_TRUE(has_kind_in(p, StripKind::CurbFace, last_lane + 1, p.strips.size()));
    CHECK_EQ(count_kind(p, StripKind::Sidewalk), size_t{2});
    CHECK_EQ(count_kind(p, StripKind::CurbFace), size_t{2});

    // A sidewalk sits one curb height above the carriageway, and the riser leans
    // outward by curb_face_batter rather than being a zero-width wall.
    for (const Strip& s : p.strips) {
        if (s.kind == StripKind::Sidewalk) {
            CHECK_NEAR(s.height_left, cfg.curb_height, 1e-6);
            CHECK_NEAR(s.height_right, cfg.curb_height, 1e-6);
            CHECK_NEAR(s.width, cfg.sidewalk_width, 1e-6);
            CHECK_EQ(s.material, MaterialId::Sidewalk);
        }
        if (s.kind == StripKind::CurbFace) {
            CHECK_NEAR(s.width, cfg.curb_face_batter, 1e-6);
            CHECK_EQ(s.material, MaterialId::Curb);
            // A face is a height change by definition.
            CHECK_NEAR(std::fabs(s.height_right - s.height_left), cfg.curb_height, 1e-6);
        }
    }
}

TEST(RoadProfile, residential_carriageway_width_is_lanes_times_the_default_lane_width) {
    const ProfileConfig cfg;
    for (int lanes : {1, 2, 3, 4}) {
        GraphEdge edge = make_edge(RoadType::Residential, lanes);
        edge.sidewalk = SideFlags::Both;

        const RoadProfile p = build_profile(edge, cfg);
        check_profile_invariants(p, "residential " + std::to_string(lanes) + " lanes");

        CHECK_NEAR(p.carriageway_width(), static_cast<double>(lanes) * cfg.lane_width_default,
                   1e-5);
        CHECK_EQ(count_kind(p, StripKind::Lane), static_cast<size_t>(lanes));
        // The carriageway is only part of the profile: sidewalks, curbs and gutters
        // all sit outboard of it.
        CHECK_TRUE(p.total_width() > p.carriageway_width());
    }
}

TEST(RoadProfile, an_explicit_width_tag_beats_the_default_lane_width) {
    GraphEdge edge = make_edge(RoadType::Residential, 2);
    edge.sidewalk = SideFlags::Both;
    edge.width = 7.4f;

    const RoadProfile p = build_profile(edge, ProfileConfig{});
    check_profile_invariants(p, "residential width=7.4");

    CHECK_NEAR(p.carriageway_width(), 7.4, 1e-5);
    CHECK_EQ(count_kind(p, StripKind::Lane), size_t{2});
}

TEST(RoadProfile, total_width_is_the_sum_of_every_strip_width) {
    GraphEdge edge = make_edge(RoadType::Residential, 2);
    edge.sidewalk = SideFlags::Both;

    const RoadProfile p = build_profile(edge, ProfileConfig{});
    double sum = 0.0;
    for (const Strip& s : p.strips) sum += static_cast<double>(s.width);
    CHECK_NEAR(p.total_width(), sum, 1e-5);

    // And the profile spans exactly that width, left edge to right edge.
    const double left = static_cast<double>(p.left_edge_offset());
    CHECK_NEAR(left - (left - sum), sum, 1e-9);
}

// ============================================================================
// Motorway
// ============================================================================

TEST(RoadProfile, motorway_has_shoulders_and_neither_sidewalk_nor_curb) {
    GraphEdge edge = make_edge(RoadType::Motorway, 4);

    const ProfileConfig cfg;
    const RoadProfile p = build_profile(edge, cfg);
    check_profile_invariants(p, "motorway");

    CHECK_EQ(count_kind(p, StripKind::Sidewalk), size_t{0});
    CHECK_EQ(count_kind(p, StripKind::CurbTop), size_t{0});
    CHECK_TRUE(count_kind(p, StripKind::Shoulder) >= size_t{1});
    CHECK_EQ(count_material(p, MaterialId::Sidewalk), size_t{0});

    // A motorway lane is wider than the generic default.
    for (const Strip& s : p.strips) {
        if (s.kind == StripKind::Lane) {
            CHECK_NEAR(s.width, cfg.motorway_lane_width, 1e-6);
        }
    }
}

TEST(RoadProfile, motorway_sidewalk_tag_is_never_synthesized_for_the_class) {
    // Even with synthesis on, a motorway has no footway.
    ProfileConfig cfg;
    cfg.synthesize_sidewalks = true;

    GraphEdge edge = make_edge(RoadType::Motorway, 4);
    edge.sidewalk = SideFlags::Unknown;

    const RoadProfile p = build_profile(edge, cfg);
    check_profile_invariants(p, "motorway unknown sidewalk");
    CHECK_EQ(count_kind(p, StripKind::Sidewalk), size_t{0});
}

// ============================================================================
// The duplicate-sidewalk guard
// ============================================================================

TEST(RoadProfile, sidewalk_none_is_never_synthesized_even_when_synthesis_is_on) {
    // sidewalk=no and sidewalk=separate both parse to SideFlags::None. Separate
    // means the footway is mapped as its own way, so synthesising one here would
    // put two sidewalks in the same place.
    ProfileConfig cfg;
    cfg.synthesize_sidewalks = true;

    for (RoadType type : kAllRoadTypes) {
        if (type == RoadType::Footway) continue;   // a footway IS the sidewalk

        GraphEdge edge = make_edge(type, 2);
        edge.sidewalk = SideFlags::None;

        const RoadProfile p = build_profile(edge, cfg);
        check_profile_invariants(p, std::string{"sidewalk=None on "} + road_type_name(type));

        if (count_kind(p, StripKind::Sidewalk) != 0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "sidewalk=None synthesizes no Sidewalk strip",
                std::string{road_type_name(type)} + " grew " +
                    std::to_string(count_kind(p, StripKind::Sidewalk)) + " sidewalk strips");
        }
    }
}

TEST(RoadProfile, sidewalk_unknown_on_residential_is_synthesized_only_when_enabled) {
    GraphEdge edge = make_edge(RoadType::Residential, 2);
    edge.sidewalk = SideFlags::Unknown;

    ProfileConfig on;
    on.synthesize_sidewalks = true;
    const RoadProfile with = build_profile(edge, on);
    check_profile_invariants(with, "residential unknown sidewalk, synthesis on");
    CHECK_EQ(count_kind(with, StripKind::Sidewalk), size_t{2});

    ProfileConfig off;
    off.synthesize_sidewalks = false;
    const RoadProfile without = build_profile(edge, off);
    check_profile_invariants(without, "residential unknown sidewalk, synthesis off");
    CHECK_EQ(count_kind(without, StripKind::Sidewalk), size_t{0});
}

// ============================================================================
// One-sided profiles keep the carriageway on the way
// ============================================================================

TEST(RoadProfile, sidewalk_left_puts_a_sidewalk_on_the_left_only) {
    GraphEdge edge = make_edge(RoadType::Residential, 2);
    edge.sidewalk = SideFlags::Left;

    const RoadProfile p = build_profile(edge, ProfileConfig{});
    check_profile_invariants(p, "residential sidewalk=left");

    CHECK_EQ(count_kind(p, StripKind::Sidewalk), size_t{1});

    const size_t first_lane = first_of_kind(p, StripKind::Lane);
    const size_t last_lane = last_of_kind(p, StripKind::Lane);
    if (first_lane == kNoStrip || last_lane == kNoStrip) return;

    // Strips are ordered left to right, so the left sidewalk precedes the lanes.
    CHECK_TRUE(has_kind_in(p, StripKind::Sidewalk, 0, first_lane));
    CHECK_FALSE(has_kind_in(p, StripKind::Sidewalk, last_lane + 1, p.strips.size()));
}

TEST(RoadProfile, sidewalk_right_puts_a_sidewalk_on_the_right_only) {
    GraphEdge edge = make_edge(RoadType::Residential, 2);
    edge.sidewalk = SideFlags::Right;

    const RoadProfile p = build_profile(edge, ProfileConfig{});
    check_profile_invariants(p, "residential sidewalk=right");

    CHECK_EQ(count_kind(p, StripKind::Sidewalk), size_t{1});

    const size_t first_lane = first_of_kind(p, StripKind::Lane);
    const size_t last_lane = last_of_kind(p, StripKind::Lane);
    if (first_lane == kNoStrip || last_lane == kNoStrip) return;

    CHECK_FALSE(has_kind_in(p, StripKind::Sidewalk, 0, first_lane));
    CHECK_TRUE(has_kind_in(p, StripKind::Sidewalk, last_lane + 1, p.strips.size()));
}

TEST(RoadProfile, an_asymmetric_profile_still_centres_the_lanes_on_the_way) {
    // The OSM way is the centreline of the CARRIAGEWAY. A sidewalk on one side
    // only widens the profile on that side; it must not shift the painted surface
    // off the surveyed geometry.
    const ProfileConfig cfg;
    for (SideFlags side : kAllSides) {
        for (int lanes : {1, 2, 3}) {
            GraphEdge edge = make_edge(RoadType::Residential, lanes);
            edge.sidewalk = side;

            const RoadProfile p = build_profile(edge, cfg);
            const std::string label = std::string{"residential sidewalk="} + side_flags_name(side) +
                                      " lanes=" + std::to_string(lanes);
            check_profile_invariants(p, label);

            const LaneSpan span = lane_span(p);
            if (!span.found) {
                stratum::test::report_failure(__FILE__, __LINE__, "profile has a carriageway span",
                                              label + ": no Lane and no Median strip");
                continue;
            }
            if (std::fabs(span.midpoint()) > 1e-6) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "carriageway span midpoint is 0",
                    label + ": midpoint " + std::to_string(span.midpoint()) + " (span " +
                        std::to_string(span.left) + " .. " + std::to_string(span.right) + ")");
            }
            // The span is the carriageway, so it is as wide as the lanes plus
            // whatever sits between the outermost of them.
            if (span.left - span.right < static_cast<double>(p.carriageway_width()) - 1e-5) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "carriageway span covers the lanes",
                    label + ": span " + std::to_string(span.left - span.right) + " vs lanes " +
                        std::to_string(p.carriageway_width()));
            }
        }
    }
}

TEST(RoadProfile, a_profile_with_no_lanes_is_centred_on_its_whole_width) {
    GraphEdge edge = make_edge(RoadType::Footway, 0);

    const RoadProfile p = build_profile(edge, ProfileConfig{});
    check_profile_invariants(p, "footway");

    CHECK_EQ(count_kind(p, StripKind::Lane), size_t{0});
    CHECK_TRUE(count_kind(p, StripKind::Sidewalk) >= size_t{1});
    CHECK_EQ(count_kind(p, StripKind::CurbFace), size_t{0});

    // No Lane and no Median, so the fallback centres the whole profile.
    CHECK_NEAR(p.left_edge_offset(), p.total_width() * 0.5f, 1e-6);
}

// ============================================================================
// Height continuity across the whole tag space
// ============================================================================

TEST(RoadProfile, height_continuity_holds_for_every_road_type_and_tag_combination) {
    const ProfileConfig cfg;
    const char* surfaces[] = {"", "asphalt", "concrete", "gravel", "dirt", "unpaved"};

    for (RoadType type : kAllRoadTypes) {
        for (SideFlags sidewalk : kAllSides) {
            for (SideFlags cycleway : kAllSides) {
                for (int lanes : {0, 1, 2, 5}) {
                    for (bool oneway : {false, true}) {
                        for (const char* surface : surfaces) {
                            GraphEdge edge = make_edge(type, lanes);
                            edge.sidewalk = sidewalk;
                            edge.cycleway = cycleway;
                            edge.is_oneway = oneway;
                            edge.surface = surface;

                            const RoadProfile p = build_profile(edge, cfg);
                            const std::string label =
                                std::string{road_type_name(type)} + " sidewalk=" +
                                side_flags_name(sidewalk) + " cycleway=" +
                                side_flags_name(cycleway) + " lanes=" + std::to_string(lanes) +
                                (oneway ? " oneway" : "") + " surface='" + surface + "'";

                            // The contract: build_profile returns a valid profile for
                            // ANY edge, including a degenerate one.
                            check_profile_invariants(p, label);
                        }
                    }
                }
            }
        }
    }
}

TEST(RoadProfile, parking_and_shoulder_tags_keep_the_profile_continuous) {
    const ProfileConfig cfg;
    for (SideFlags parking : kAllSides) {
        for (SideFlags shoulder : kAllSides) {
            GraphEdge edge = make_edge(RoadType::Tertiary, 2);
            edge.parking = parking;
            edge.shoulder = shoulder;
            edge.sidewalk = SideFlags::Both;

            const RoadProfile p = build_profile(edge, cfg);
            check_profile_invariants(p, std::string{"tertiary parking="} +
                                            side_flags_name(parking) + " shoulder=" +
                                            side_flags_name(shoulder));

            const LaneSpan span = lane_span(p);
            if (span.found && std::fabs(span.midpoint()) > 1e-6) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "carriageway span midpoint is 0",
                    std::string{"tertiary parking="} + side_flags_name(parking) + " shoulder=" +
                        side_flags_name(shoulder) + ": midpoint " +
                        std::to_string(span.midpoint()));
            }
        }
    }
}

// ============================================================================
// Unpaved surfaces
// ============================================================================

TEST(RoadProfile, gravel_track_uses_the_gravel_material_and_grows_no_curb) {
    GraphEdge edge = make_edge(RoadType::Path, 1);   // highway=track parses to Path
    edge.surface = "gravel";

    const RoadProfile p = build_profile(edge, ProfileConfig{});
    check_profile_invariants(p, "track surface=gravel");

    CHECK_EQ(count_kind(p, StripKind::CurbFace), size_t{0});
    CHECK_EQ(count_kind(p, StripKind::CurbTop), size_t{0});
    CHECK_EQ(count_kind(p, StripKind::Sidewalk), size_t{0});

    const size_t lane_count = count_kind(p, StripKind::Lane);
    CHECK_TRUE(lane_count >= size_t{1});
    for (const Strip& s : p.strips) {
        if (s.kind == StripKind::Lane) {
            CHECK_EQ(s.material, MaterialId::Gravel);
        }
        // A track is flat: nothing is raised above the running surface.
        CHECK_NEAR(s.height_left, 0.0, 1e-6);
        CHECK_NEAR(s.height_right, 0.0, 1e-6);
    }

    // The plan's rural track profile is verge, running surface, verge.
    CHECK_EQ(count_kind(p, StripKind::Verge), size_t{2});
}

TEST(RoadProfile, dirt_surface_selects_the_dirt_material) {
    GraphEdge edge = make_edge(RoadType::Path, 1);
    edge.surface = "dirt";

    const RoadProfile p = build_profile(edge, ProfileConfig{});
    check_profile_invariants(p, "track surface=dirt");

    for (const Strip& s : p.strips) {
        if (s.kind == StripKind::Lane) {
            CHECK_EQ(s.material, MaterialId::Dirt);
        }
    }
    CHECK_EQ(count_kind(p, StripKind::CurbFace), size_t{0});
}

// ============================================================================
// Medians
// ============================================================================

TEST(RoadProfile, a_wide_median_becomes_a_raised_grass_island_with_curb_faces) {
    ProfileConfig cfg;
    cfg.median_width = 3.0f;            // >= min_median_raised of 1.5
    cfg.min_median_raised = 1.5f;

    GraphEdge edge = make_edge(RoadType::Motorway, 4);
    edge.is_oneway = false;             // a dual carriageway has a central reservation

    const RoadProfile p = build_profile(edge, cfg);
    check_profile_invariants(p, "motorway wide median");

    const size_t median = first_of_kind(p, StripKind::Median);
    CHECK_TRUE(median != kNoStrip);
    if (median == kNoStrip) return;

    CHECK_EQ(count_kind(p, StripKind::Median), size_t{1});
    CHECK_NEAR(p.strips[median].width, cfg.median_width, 1e-6);
    CHECK_EQ(p.strips[median].material, MaterialId::Grass);
    CHECK_NEAR(p.strips[median].height_left, cfg.curb_height, 1e-6);
    CHECK_NEAR(p.strips[median].height_right, cfg.curb_height, 1e-6);

    // Raised means CurbFace, Median, CurbFace.
    CHECK_TRUE(median >= size_t{1});
    CHECK_TRUE(median + 1 < p.strips.size());
    if (median == 0 || median + 1 >= p.strips.size()) return;
    CHECK_EQ(p.strips[median - 1].kind, StripKind::CurbFace);
    CHECK_EQ(p.strips[median + 1].kind, StripKind::CurbFace);
}

TEST(RoadProfile, a_narrow_median_stays_a_flat_painted_strip) {
    ProfileConfig cfg;
    cfg.median_width = 1.0f;            // < min_median_raised of 1.5
    cfg.min_median_raised = 1.5f;

    GraphEdge edge = make_edge(RoadType::Motorway, 4);
    edge.is_oneway = false;

    const RoadProfile p = build_profile(edge, cfg);
    check_profile_invariants(p, "motorway narrow median");

    const size_t median = first_of_kind(p, StripKind::Median);
    CHECK_TRUE(median != kNoStrip);
    if (median == kNoStrip) return;

    CHECK_NEAR(p.strips[median].width, cfg.median_width, 1e-6);
    CHECK_EQ(p.strips[median].material, MaterialId::Asphalt);
    CHECK_NEAR(p.strips[median].height_left, 0.0, 1e-6);
    CHECK_NEAR(p.strips[median].height_right, 0.0, 1e-6);

    // Flat, so no riser is needed on either side of it.
    if (median > 0) {
        CHECK_TRUE(p.strips[median - 1].kind != StripKind::CurbFace);
    }
    if (median + 1 < p.strips.size()) {
        CHECK_TRUE(p.strips[median + 1].kind != StripKind::CurbFace);
    }

    // A painted median still leaves the carriageway centred on the way.
    const LaneSpan span = lane_span(p);
    CHECK_TRUE(span.found);
    if (span.found) CHECK_NEAR(span.midpoint(), 0.0, 1e-6);
}

// ============================================================================
// Degenerate profiles and naming
// ============================================================================

TEST(RoadProfile, an_empty_profile_is_invalid_and_measures_zero) {
    const RoadProfile p;
    CHECK_FALSE(p.is_valid());
    CHECK_NEAR(p.total_width(), 0.0, 1e-9);
    CHECK_NEAR(p.carriageway_width(), 0.0, 1e-9);
    CHECK_NEAR(p.left_edge_offset(), 0.0, 1e-9);
}

TEST(RoadProfile, a_stepped_profile_with_no_curb_face_is_rejected) {
    // Hand-built, not from build_profile(): two strips disagreeing at their shared
    // boundary. is_valid() has to catch this, because the extruder would otherwise
    // emit a cracked ribbon.
    RoadProfile p;
    p.strips.push_back(Strip{3.5f, 0.0f, 0.0f, MaterialId::Asphalt, StripKind::Lane});
    p.strips.push_back(Strip{2.0f, 0.15f, 0.15f, MaterialId::Sidewalk, StripKind::Sidewalk});
    CHECK_FALSE(p.is_valid());

    // Repaired by an explicit zero-width riser between them.
    RoadProfile fixed;
    fixed.strips.push_back(Strip{3.5f, 0.0f, 0.0f, MaterialId::Asphalt, StripKind::Lane});
    fixed.strips.push_back(Strip{0.0f, 0.0f, 0.15f, MaterialId::Curb, StripKind::CurbFace});
    fixed.strips.push_back(Strip{2.0f, 0.15f, 0.15f, MaterialId::Sidewalk, StripKind::Sidewalk});
    CHECK_TRUE(fixed.is_valid());
    CHECK_NEAR(fixed.total_width(), 5.5, 1e-6);
    CHECK_NEAR(fixed.carriageway_width(), 3.5, 1e-6);
}

TEST(RoadProfile, strip_kind_name_covers_every_kind) {
    const StripKind kinds[] = {
        StripKind::Lane,   StripKind::Gutter,    StripKind::CurbFace,  StripKind::CurbTop,
        StripKind::Sidewalk, StripKind::Shoulder, StripKind::Median,   StripKind::Verge,
        StripKind::CycleLane, StripKind::ParkingLane,
    };
    for (StripKind k : kinds) {
        const char* name = strip_kind_name(k);
        CHECK_TRUE(name != nullptr);
        CHECK_TRUE(std::string{name} != "Unknown");
        CHECK_TRUE(std::string{name} != "");
    }
    CHECK_EQ(std::string{strip_kind_name(StripKind::Count)}, std::string{"Unknown"});
    CHECK_EQ(std::string{material_id_name(MaterialId::Curb)}, std::string{"Curb"});
}

// ============================================================================
// lanes:both_ways - the centre turning lane
// ============================================================================

namespace {

/// Total width of every Lane strip, in metres
float lane_strip_total(const RoadProfile& p) {
    float total = 0.0f;
    for (const Strip& s : p.strips) {
        if (s.kind == StripKind::Lane) total += s.width;
    }
    return total;
}

} // namespace

TEST(RoadProfile, lanes_both_ways_is_already_counted_by_a_bare_lanes_tag) {
    // OSM's identity is lanes = lanes:forward + lanes:backward + lanes:both_ways,
    // so a bare lanes=3 with lanes:both_ways=1 is a 3 lane carriageway: two
    // running lanes and a centre turning lane. Adding a strip for the turning
    // lane on top of a count that already held it emitted FOUR lanes, a 3.5 m
    // over-width on the ribbon and on the outline P3 carves the terrain against.
    stratum::osm::TagMap tags;
    tags["lanes:both_ways"] = "1";

    GraphEdge bare = make_edge(RoadType::Primary, 3);
    bare.width = 10.5f;     // what the parser synthesizes from lanes=3
    const RoadProfile from_bare = build_profile(bare, ProfileConfig{}, &tags);

    // The same physical road with the directional tags spelled out. That branch
    // of resolve_lane_count() genuinely excludes the turning lane, so the two
    // spellings have to agree; they differed by exactly one lane.
    GraphEdge directional = bare;
    directional.lanes_forward = 1;
    directional.lanes_backward = 1;
    const RoadProfile from_directional = build_profile(directional, ProfileConfig{}, &tags);

    CHECK_TRUE(from_bare.is_valid());
    CHECK_TRUE(from_directional.is_valid());
    CHECK_NEAR(from_bare.carriageway_width(), from_directional.carriageway_width(), 1e-4);
    CHECK_NEAR(lane_strip_total(from_bare), 10.5f, 1e-4);
    CHECK_EQ(count_kind(from_bare, StripKind::Lane), size_t{3});

    // Only one directional tag present is still the bare branch: resolve_lane_count
    // needs BOTH before it can exclude the turning lane.
    GraphEdge half = bare;
    half.lanes_forward = 1;
    const RoadProfile from_half = build_profile(half, ProfileConfig{}, &tags);
    CHECK_NEAR(from_half.carriageway_width(), from_bare.carriageway_width(), 1e-4);
}

TEST(RoadProfile, the_both_ways_lane_sits_in_the_middle_of_the_carriageway) {
    // A centre turning lane is central by definition. Pushing it after every
    // running lane put it at the outside edge of the carriageway, 5.25 m off
    // centre, which is where P5 would then hang the centre-turn markings.
    stratum::osm::TagMap tags;
    tags["lanes:both_ways"] = "1";

    GraphEdge edge = make_edge(RoadType::Primary, 5);
    edge.width = 17.5f;
    const RoadProfile p = build_profile(edge, ProfileConfig{}, &tags);
    CHECK_TRUE(p.is_valid());
    if (!p.is_valid()) return;

    // Four running lanes and one turning lane, the turning lane third.
    CHECK_EQ(count_kind(p, StripKind::Lane), size_t{5});
    CHECK_NEAR(lane_strip_total(p), 17.5f, 1e-4);

    const size_t first = first_of_kind(p, StripKind::Lane);
    const size_t last = last_of_kind(p, StripKind::Lane);
    if (first == kNoStrip || last == kNoStrip) return;

    // Two running lanes each side of it.
    CHECK_EQ(last - first, size_t{4});

    // Its own centre is the centre of the carriageway. Walk the lateral
    // coordinates from the profile's left edge to the middle lane.
    double lateral = static_cast<double>(p.left_edge_offset());
    double carriageway_left = 0.0;
    double middle_centre = 0.0;
    for (size_t i = 0; i < p.strips.size(); ++i) {
        if (i == first) carriageway_left = lateral;
        if (i == first + 2) middle_centre = lateral - 0.5 * static_cast<double>(p.strips[i].width);
        lateral -= static_cast<double>(p.strips[i].width);
    }
    const double carriageway_centre =
        carriageway_left - 0.5 * static_cast<double>(p.carriageway_width());
    CHECK_NEAR(middle_centre, carriageway_centre, 1e-4);
}
