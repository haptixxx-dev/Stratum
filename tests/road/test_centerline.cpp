/**
 * @file test_centerline.cpp
 * @brief Centerline resampling, framing, miter, and slice tests
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * These tests are written against the contract in src/osm/road/centerline.hpp
 * and the "Centerline processing" and "UV Convention" sections of
 * docs/plans/road_network_plan.md. They never read a fixture: every polyline is
 * built in the test body, in 2D local metres, so an expected value can be stated
 * exactly rather than approximately.
 *
 * The miter is the highest-value thing here. src/osm/mesh_builder.cpp:456
 * averages normalised tangents and never divides by cos(theta/2), so the old
 * ribbon pinches inward at every corner. The right-angle test below is the direct
 * regression test for that defect: it asserts miter_scale is sqrt(2) and that the
 * offset point sits exactly w metres from BOTH original segments.
 *
 * Every test sets ResampleConfig::smooth to false unless it is testing smoothing.
 * A Catmull-Rom fit moves the space between input vertices, which would make an
 * exact expected offset impossible to state.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests Centerline
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/centerline.hpp"

#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace {

using stratum::osm::road::Centerline;
using stratum::osm::road::ResampleConfig;
using stratum::osm::road::Station;
using stratum::osm::road::build_centerline;
using stratum::osm::road::offset_point;
using stratum::osm::road::slice;

/// Tolerance for values the contract states exactly
constexpr double kExact = 1e-9;

/// Tolerance for values that survive one renormalisation
constexpr double kTight = 1e-12;

/**
 * @brief A ResampleConfig with smoothing off
 *
 * Smoothing invents geometry between the input vertices, so a test that states an
 * exact expected offset must switch it off.
 *
 * @return The default config with smooth set to false
 */
ResampleConfig raw_config() {
    ResampleConfig cfg;
    cfg.smooth = false;
    return cfg;
}

/// True when both components of @p v are finite
bool is_finite(const glm::dvec2& v) {
    return std::isfinite(v.x) && std::isfinite(v.y);
}

/**
 * @brief Assert that no station carries a NaN or an infinity
 *
 * A NaN that reaches the corridor extruder becomes a NaN vertex position, which
 * is the failure mode that is hardest to diagnose once it is on the GPU.
 *
 * @param cl    Centerline to check
 * @param label Test context, for the failure message
 */
void check_no_nan(const Centerline& cl, const char* label) {
    for (size_t i = 0; i < cl.stations.size(); ++i) {
        const Station& s = cl.stations[i];
        const bool ok = is_finite(s.position) && is_finite(s.tangent) && is_finite(s.normal) &&
                        std::isfinite(s.arclength) && std::isfinite(s.curvature) &&
                        std::isfinite(s.miter_scale);
        if (!ok) {
            stratum::test::report_failure(__FILE__, __LINE__, "station is finite",
                                          std::string{label} + " station " + std::to_string(i));
        }
    }
}

/**
 * @brief Assert that every frame is orthonormal
 *
 * The tangent is a unit vector and the normal is a unit vector. The normal is the
 * miter bisector, so it is NOT perpendicular to the tangent at a joint; only its
 * length is asserted here.
 *
 * @param cl    Centerline to check
 * @param label Test context, for the failure message
 */
void check_unit_frames(const Centerline& cl, const char* label) {
    for (size_t i = 0; i < cl.stations.size(); ++i) {
        const Station& s = cl.stations[i];
        const double tl = glm::length(s.tangent);
        const double nl = glm::length(s.normal);
        if (std::fabs(tl - 1.0) > 1e-9 || std::fabs(nl - 1.0) > 1e-9) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "frame vectors are unit length",
                std::string{label} + " station " + std::to_string(i) + ": |tangent| " +
                    std::to_string(tl) + " |normal| " + std::to_string(nl));
        }
    }
}

/**
 * @brief Find the single station within @p eps of @p p
 *
 * Reports a failure and returns nullptr when no station is close enough, so a
 * resampler that drops a corner vertex fails loudly rather than silently skipping
 * the miter assertions that follow.
 *
 * @param cl  Centerline to search
 * @param p   Position to look for, in 2D local metres
 * @param eps Match radius in metres
 * @return The first matching station, or nullptr
 */
const Station* station_at(const Centerline& cl, const glm::dvec2& p, double eps) {
    for (const auto& s : cl.stations) {
        if (glm::length(s.position - p) <= eps) return &s;
    }
    stratum::test::report_failure(__FILE__, __LINE__, "a station sits at the expected position",
                                  "no station within " + std::to_string(eps) + " m of (" +
                                      std::to_string(p.x) + ", " + std::to_string(p.y) + ")");
    return nullptr;
}

/**
 * @brief Perpendicular distance from a point to the infinite line through a and b
 *
 * @param p Point to measure from
 * @param a First point on the line
 * @param b Second point on the line
 * @return Distance in metres, or 0 when a and b coincide
 */
double distance_to_line(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 d = b - a;
    const double len = glm::length(d);
    if (len <= 0.0) return 0.0;
    return std::fabs(d.x * (p.y - a.y) - d.y * (p.x - a.x)) / len;
}

/**
 * @brief Position on a centerline at an arc length, by linear interpolation
 *
 * Exact for a centerline whose stations are collinear, which is the only case it
 * is used for.
 *
 * @param cl Centerline to sample
 * @param s  Arc length in metres, in the centerline's own parameterisation
 * @return The interpolated position, or the nearest end for an out-of-range value
 */
glm::dvec2 position_at_arclength(const Centerline& cl, double s) {
    if (cl.stations.empty()) return glm::dvec2{0.0};
    if (s <= cl.stations.front().arclength) return cl.stations.front().position;
    for (size_t i = 1; i < cl.stations.size(); ++i) {
        const Station& a = cl.stations[i - 1];
        const Station& b = cl.stations[i];
        if (s <= b.arclength) {
            const double span = b.arclength - a.arclength;
            if (span <= 0.0) return b.position;
            const double t = (s - a.arclength) / span;
            return a.position + (b.position - a.position) * t;
        }
    }
    return cl.stations.back().position;
}

/// Total length of a polyline, in metres
double polyline_length(const std::vector<glm::dvec2>& poly) {
    double total = 0.0;
    for (size_t i = 1; i < poly.size(); ++i) total += glm::length(poly[i] - poly[i - 1]);
    return total;
}

/// Number of stations flagged is_bevel
size_t count_bevel_stations(const Centerline& cl) {
    return static_cast<size_t>(std::count_if(cl.stations.begin(), cl.stations.end(),
                                             [](const Station& s) { return s.is_bevel; }));
}

/**
 * @brief Points along a circular arc, centred on the origin
 *
 * @param radius       Arc radius in metres
 * @param sweep        Total swept angle in radians
 * @param point_count  Number of points to emit, at least 2
 * @return The arc as a polyline in 2D local metres
 */
std::vector<glm::dvec2> arc_polyline(double radius, double sweep, size_t point_count) {
    std::vector<glm::dvec2> poly;
    poly.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(point_count - 1);
        const double a = t * sweep;
        poly.push_back(glm::dvec2{radius * std::cos(a), radius * std::sin(a)});
    }
    return poly;
}

/**
 * @brief An evenly sampled straight line along +X
 *
 * @param length      Total length in metres
 * @param point_count Number of points to emit, at least 2
 * @return The straight as a polyline in 2D local metres
 */
std::vector<glm::dvec2> straight_polyline(double length, size_t point_count) {
    std::vector<glm::dvec2> poly;
    poly.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(point_count - 1);
        poly.push_back(glm::dvec2{length * t, 0.0});
    }
    return poly;
}

} // namespace

// ============================================================================
// Right-angle miter - the regression test for the pinch at mesh_builder.cpp:456
// ============================================================================

TEST(Centerline, right_angle_corner_miter_scale_is_root_two) {
    const std::vector<glm::dvec2> poly = {{-10.0, 0.0}, {0.0, 0.0}, {0.0, 10.0}};
    const Centerline cl = build_centerline(poly, raw_config());

    CHECK_TRUE(cl.is_valid());
    check_no_nan(cl, "right angle");
    check_unit_frames(cl, "right angle");

    const Station* corner = station_at(cl, glm::dvec2{0.0, 0.0}, kExact);
    if (corner == nullptr) return;

    // theta is the exterior turn angle, a quarter turn here, so the offset must be
    // scaled by 1 / cos(45 degrees) = sqrt(2). The old extruder used 1.0 and pinched.
    CHECK_NEAR(corner->miter_scale, std::sqrt(2.0), kExact);
    CHECK_FALSE(corner->is_bevel);
    CHECK_NEAR(corner->arclength, 10.0, kExact);

    // The normal is the bisector of the two left normals, (0,1) and (-1,0).
    CHECK_NEAR(corner->normal.x, -std::sqrt(0.5), kExact);
    CHECK_NEAR(corner->normal.y, std::sqrt(0.5), kExact);
}

TEST(Centerline, right_angle_offset_point_is_equidistant_from_both_segments) {
    const std::vector<glm::dvec2> poly = {{-10.0, 0.0}, {0.0, 0.0}, {0.0, 10.0}};
    const Centerline cl = build_centerline(poly, raw_config());

    const Station* corner = station_at(cl, glm::dvec2{0.0, 0.0}, kExact);
    if (corner == nullptr) return;

    // A correct miter puts the offset column exactly |w| from both original
    // segments. A missing 1/cos(theta/2) puts it |w| * cos(45 deg) from each,
    // which is the visible pinch.
    for (double w : {1.0, 2.5, 4.0, -1.0, -2.5, -4.0}) {
        const glm::dvec2 p = offset_point(*corner, w);
        CHECK_NEAR(distance_to_line(p, poly[0], poly[1]), std::fabs(w), kExact);
        CHECK_NEAR(distance_to_line(p, poly[1], poly[2]), std::fabs(w), kExact);

        // Stated the other way round: the left offset of a left turn lands at (-w, w).
        CHECK_NEAR(p.x, -w, kExact);
        CHECK_NEAR(p.y, w, kExact);
    }
}

// ============================================================================
// Straight polyline
// ============================================================================

TEST(Centerline, straight_polyline_has_unit_miter_and_constant_normal) {
    const std::vector<glm::dvec2> poly = {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {30.0, 0.0}};
    const ResampleConfig cfg = raw_config();
    const Centerline cl = build_centerline(poly, cfg);

    CHECK_TRUE(cl.is_valid());
    check_no_nan(cl, "straight");
    check_unit_frames(cl, "straight");
    CHECK_NEAR(cl.length(), 30.0, kExact);

    for (const Station& s : cl.stations) {
        CHECK_EQ(s.miter_scale, 1.0);
        CHECK_FALSE(s.is_bevel);
        CHECK_NEAR(s.tangent.x, 1.0, kTight);
        CHECK_NEAR(s.tangent.y, 0.0, kTight);
        CHECK_NEAR(s.normal.x, 0.0, kTight);
        CHECK_NEAR(s.normal.y, 1.0, kTight);
        CHECK_NEAR(s.curvature, 0.0, 1e-9);
    }
}

TEST(Centerline, straight_polyline_arclength_is_strictly_increasing_and_respects_spacing) {
    const std::vector<glm::dvec2> poly = {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {30.0, 0.0}};
    const ResampleConfig cfg = raw_config();
    const Centerline cl = build_centerline(poly, cfg);
    if (!cl.is_valid()) return;

    CHECK_NEAR(cl.stations.front().arclength, 0.0, kExact);
    CHECK_NEAR(cl.stations.front().position.x, 0.0, kExact);
    CHECK_NEAR(cl.stations.back().position.x, 30.0, kExact);

    for (size_t i = 1; i < cl.stations.size(); ++i) {
        const double gap = cl.stations[i].arclength - cl.stations[i - 1].arclength;
        CHECK_TRUE(gap > 0.0);
        CHECK_TRUE(gap <= cfg.max_spacing + kExact);
        CHECK_TRUE(gap >= cfg.min_spacing - kExact);

        // arclength must be the real distance travelled, since it is the only
        // input to V and a drift there is a texture stretch.
        const double step = glm::length(cl.stations[i].position - cl.stations[i - 1].position);
        CHECK_NEAR(gap, step, kExact);
    }
}

// ============================================================================
// Hairpin - past the miter limit
// ============================================================================

TEST(Centerline, hairpin_bevels_instead_of_mitring) {
    // Exterior turn angle of 175 degrees: 1/cos(87.5 deg) is about 22.9, far past
    // the default miter limit of 4, so the joint must bevel.
    const double turn = 175.0 * 3.14159265358979323846 / 180.0;
    const std::vector<glm::dvec2> poly = {
        {-10.0, 0.0}, {0.0, 0.0}, {10.0 * std::cos(turn), 10.0 * std::sin(turn)}};

    ResampleConfig cfg = raw_config();
    cfg.miter_limit = 4.0;
    const Centerline cl = build_centerline(poly, cfg);

    CHECK_TRUE(cl.is_valid());
    check_no_nan(cl, "hairpin");
    check_unit_frames(cl, "hairpin");

    // One bevelled joint is represented as exactly two coincident stations.
    CHECK_EQ(count_bevel_stations(cl), size_t{2});

    // No station may ever report a scale above the limit, and none below 1.
    for (const Station& s : cl.stations) {
        CHECK_TRUE(s.miter_scale >= 1.0 - kExact);
        CHECK_TRUE(s.miter_scale <= cfg.miter_limit + kExact);
    }
}

TEST(Centerline, hairpin_bevel_pair_is_coincident_and_carries_both_segment_normals) {
    const double turn = 175.0 * 3.14159265358979323846 / 180.0;
    const std::vector<glm::dvec2> poly = {
        {-10.0, 0.0}, {0.0, 0.0}, {10.0 * std::cos(turn), 10.0 * std::sin(turn)}};

    ResampleConfig cfg = raw_config();
    cfg.miter_limit = 4.0;
    const Centerline cl = build_centerline(poly, cfg);
    if (!cl.is_valid()) return;

    size_t first = cl.stations.size();
    for (size_t i = 0; i < cl.stations.size(); ++i) {
        if (cl.stations[i].is_bevel) { first = i; break; }
    }
    if (first + 1 >= cl.stations.size()) {
        stratum::test::report_failure(__FILE__, __LINE__, "a bevel pair exists",
                                      "hairpin produced no adjacent bevel pair");
        return;
    }

    const Station& a = cl.stations[first];
    const Station& b = cl.stations[first + 1];
    CHECK_TRUE(b.is_bevel);

    // The pair shares one position and one arclength; the zero-length band between
    // them is the bevel wedge.
    CHECK_NEAR(a.position.x, b.position.x, kExact);
    CHECK_NEAR(a.position.y, b.position.y, kExact);
    CHECK_NEAR(a.arclength, b.arclength, kExact);
    CHECK_NEAR(a.position.x, 0.0, kExact);
    CHECK_NEAR(a.position.y, 0.0, kExact);
    CHECK_NEAR(a.arclength, 10.0, kExact);

    // A bevelled joint is not mitred, so the compensation factor returns to 1.
    CHECK_EQ(a.miter_scale, 1.0);
    CHECK_EQ(b.miter_scale, 1.0);

    // First station carries the incoming segment's left normal, second the outgoing
    // segment's. Left normal of a unit tangent t is (-t.y, t.x).
    CHECK_NEAR(a.normal.x, 0.0, 1e-9);
    CHECK_NEAR(a.normal.y, 1.0, 1e-9);
    CHECK_NEAR(b.normal.x, -std::sin(turn), 1e-9);
    CHECK_NEAR(b.normal.y, std::cos(turn), 1e-9);
}

// ============================================================================
// Degenerate input
// ============================================================================

TEST(Centerline, duplicate_and_near_duplicate_points_are_welded) {
    // Two exact duplicates and one point 5e-5 m away, all under the 1e-4 m weld
    // epsilon. A zero-length segment has no tangent, so an unwelded input produces
    // a NaN frame.
    const std::vector<glm::dvec2> poly = {
        {0.0, 0.0}, {0.0, 0.0}, {10.0, 0.0}, {10.0, 0.0},
        {10.0 + 5e-5, 0.0}, {20.0, 0.0},
    };
    const Centerline cl = build_centerline(poly, raw_config());

    CHECK_TRUE(cl.is_valid());
    check_no_nan(cl, "welded");
    check_unit_frames(cl, "welded");
    CHECK_NEAR(cl.length(), 20.0, 1e-3);

    // No zero-length step survives the weld, so arclength never stalls except
    // across a bevel pair, of which a straight line has none.
    CHECK_EQ(count_bevel_stations(cl), size_t{0});
    for (size_t i = 1; i < cl.stations.size(); ++i) {
        CHECK_TRUE(cl.stations[i].arclength > cl.stations[i - 1].arclength);
    }
}

TEST(Centerline, two_point_polyline_yields_exactly_two_stations) {
    // 7 m is under the 8 m max spacing, so no interior station is needed.
    const std::vector<glm::dvec2> poly = {{0.0, 0.0}, {7.0, 0.0}};
    const Centerline cl = build_centerline(poly, raw_config());

    CHECK_TRUE(cl.is_valid());
    CHECK_EQ(cl.stations.size(), size_t{2});
    if (cl.stations.size() != 2) return;

    CHECK_NEAR(cl.length(), 7.0, kExact);
    CHECK_EQ(cl.stations[0].miter_scale, 1.0);
    CHECK_EQ(cl.stations[1].miter_scale, 1.0);
    CHECK_NEAR(cl.stations[0].arclength, 0.0, kExact);
    CHECK_NEAR(cl.stations[1].arclength, 7.0, kExact);
    CHECK_NEAR(cl.stations[0].position.x, 0.0, kExact);
    CHECK_NEAR(cl.stations[1].position.x, 7.0, kExact);
}

TEST(Centerline, one_point_and_empty_polylines_are_invalid) {
    const Centerline empty = build_centerline({}, raw_config());
    CHECK_FALSE(empty.is_valid());
    CHECK_TRUE(empty.stations.size() < 2);
    CHECK_NEAR(empty.length(), 0.0, kExact);

    const Centerline single = build_centerline({{3.0, 4.0}}, raw_config());
    CHECK_FALSE(single.is_valid());
    CHECK_TRUE(single.stations.size() < 2);

    // Welds down to one distinct point, so it is invalid for the same reason.
    const Centerline collapsed =
        build_centerline({{1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0 + 1e-6}}, raw_config());
    CHECK_FALSE(collapsed.is_valid());
    check_no_nan(collapsed, "collapsed");
}

// ============================================================================
// Curvature-adaptive spacing
// ============================================================================

TEST(Centerline, tight_arc_produces_more_stations_than_a_straight_of_equal_length) {
    constexpr double kRadius = 5.0;
    constexpr double kSweep = 3.14159265358979323846 * 0.5;   // quarter circle
    const double arc_length = kRadius * kSweep;               // about 7.854 m

    // Both inputs carry the same number of vertices, so the difference in station
    // count can only come from curvature, not from input density.
    const std::vector<glm::dvec2> arc = arc_polyline(kRadius, kSweep, 16);
    const std::vector<glm::dvec2> straight = straight_polyline(arc_length, 16);

    const ResampleConfig cfg = raw_config();
    const Centerline arc_cl = build_centerline(arc, cfg);
    const Centerline straight_cl = build_centerline(straight, cfg);

    CHECK_TRUE(arc_cl.is_valid());
    CHECK_TRUE(straight_cl.is_valid());
    check_no_nan(arc_cl, "arc");

    CHECK_TRUE(arc_cl.stations.size() > straight_cl.stations.size());

    // The straight is shorter than max_spacing, so it needs no interior station.
    CHECK_EQ(straight_cl.stations.size(), size_t{2});

    // Chord-to-arc deviation of 0.05 m on a 5 m radius allows about 1.41 m between
    // stations, so the arc needs at least five.
    CHECK_TRUE(arc_cl.stations.size() >= size_t{5});

    // Curvature of a circle of radius R is 1/R. The sign is positive because the
    // arc turns left.
    for (const Station& s : arc_cl.stations) {
        CHECK_NEAR(std::fabs(s.curvature), 1.0 / kRadius, 0.05);
    }
}

// ============================================================================
// slice()
// ============================================================================

TEST(Centerline, slice_spans_the_requested_arclength_range) {
    const std::vector<glm::dvec2> poly = {{0.0, 0.0}, {100.0, 0.0}};
    const Centerline cl = build_centerline(poly, raw_config());
    if (!cl.is_valid()) return;

    constexpr double kFrom = 12.5;
    constexpr double kTo = 71.25;
    const Centerline cut = slice(cl, kFrom, kTo);

    CHECK_TRUE(cut.is_valid());
    if (!cut.is_valid()) return;
    check_no_nan(cut, "slice");
    check_unit_frames(cut, "slice");

    // Arclengths are NOT rebased: a trimmed ribbon must keep the V placement of the
    // untrimmed one, so re-trimming never shifts the texture. See centerline.hpp.
    CHECK_NEAR(cut.stations.front().arclength, kFrom, kExact);
    CHECK_NEAR(cut.stations.back().arclength, kTo, kExact);
    CHECK_NEAR(cut.stations.back().arclength - cut.stations.front().arclength, kTo - kFrom,
               kExact);

    // Endpoints land exactly where the source centerline is at those arc lengths.
    const glm::dvec2 want_from = position_at_arclength(cl, kFrom);
    const glm::dvec2 want_to = position_at_arclength(cl, kTo);
    CHECK_NEAR(cut.stations.front().position.x, want_from.x, kExact);
    CHECK_NEAR(cut.stations.front().position.y, want_from.y, kExact);
    CHECK_NEAR(cut.stations.back().position.x, want_to.x, kExact);
    CHECK_NEAR(cut.stations.back().position.y, want_to.y, kExact);
    CHECK_NEAR(cut.stations.front().position.x, kFrom, kExact);
    CHECK_NEAR(cut.stations.back().position.x, kTo, kExact);

    // A synthesised endpoint is never a bevel.
    CHECK_FALSE(cut.stations.front().is_bevel);
    CHECK_FALSE(cut.stations.back().is_bevel);

    // Every interior station is inside the range and still ordered.
    for (size_t i = 0; i < cut.stations.size(); ++i) {
        CHECK_TRUE(cut.stations[i].arclength >= kFrom - kExact);
        CHECK_TRUE(cut.stations[i].arclength <= kTo + kExact);
        if (i > 0) {
            CHECK_TRUE(cut.stations[i].arclength >= cut.stations[i - 1].arclength);
        }
    }
}

TEST(Centerline, slice_offset_points_match_the_source_offset_points) {
    const std::vector<glm::dvec2> poly = {{0.0, 0.0}, {60.0, 0.0}};
    const Centerline cl = build_centerline(poly, raw_config());
    if (!cl.is_valid()) return;

    const Centerline cut = slice(cl, 10.0, 50.0);
    if (!cut.is_valid()) return;

    // A synthesised endpoint interpolates position, tangent, normal and miter, so
    // its offset column has to land on the source's offset column.
    for (double lateral : {0.0, 3.5, -3.5}) {
        const glm::dvec2 got_from = offset_point(cut.stations.front(), lateral);
        const glm::dvec2 got_to = offset_point(cut.stations.back(), lateral);
        CHECK_NEAR(got_from.x, 10.0, kExact);
        CHECK_NEAR(got_from.y, lateral, kExact);
        CHECK_NEAR(got_to.x, 50.0, kExact);
        CHECK_NEAR(got_to.y, lateral, kExact);
    }
}

TEST(Centerline, slice_clamps_swaps_and_rejects_an_empty_range) {
    const std::vector<glm::dvec2> poly = {{0.0, 0.0}, {40.0, 0.0}};
    const Centerline cl = build_centerline(poly, raw_config());
    if (!cl.is_valid()) return;

    // Reversed ranges are swapped, out-of-range ends are clamped.
    const Centerline swapped = slice(cl, 30.0, 5.0);
    CHECK_TRUE(swapped.is_valid());
    if (swapped.is_valid()) {
        CHECK_NEAR(swapped.stations.front().arclength, 5.0, kExact);
        CHECK_NEAR(swapped.stations.back().arclength, 30.0, kExact);
    }

    const Centerline clamped = slice(cl, -100.0, 1000.0);
    CHECK_TRUE(clamped.is_valid());
    if (clamped.is_valid()) {
        CHECK_NEAR(clamped.stations.front().arclength, 0.0, kExact);
        CHECK_NEAR(clamped.stations.back().arclength, cl.length(), kExact);
    }

    // A zero-width range cannot produce a band to extrude.
    const Centerline empty = slice(cl, 20.0, 20.0);
    CHECK_FALSE(empty.is_valid());
    check_no_nan(empty, "empty slice");

    const Centerline from_invalid = slice(Centerline{}, 0.0, 10.0);
    CHECK_FALSE(from_invalid.is_valid());
}

// ============================================================================
// Left-normal sign - the one nobody may flip later
// ============================================================================

TEST(Centerline, left_normal_of_a_plus_x_road_points_to_plus_y_and_minus_z_in_world) {
    const std::vector<glm::dvec2> poly = {{0.0, 0.0}, {25.0, 0.0}};
    const Centerline cl = build_centerline(poly, raw_config());
    if (!cl.is_valid()) return;

    for (const Station& s : cl.stations) {
        // 2D: travelling along +X, left is +Y.
        CHECK_NEAR(s.tangent.x, 1.0, kTight);
        CHECK_NEAR(s.tangent.y, 0.0, kTight);
        CHECK_NEAR(s.normal.x, 0.0, kTight);
        CHECK_NEAR(s.normal.y, 1.0, kTight);

        // A positive lateral offset is to the left.
        const glm::dvec2 left = offset_point(s, 4.0);
        CHECK_NEAR(left.y, 4.0, kExact);
        const glm::dvec2 right = offset_point(s, -4.0);
        CHECK_NEAR(right.y, -4.0, kExact);

        // World space is Y up under (x, y_2d) -> (x, height, -y_2d), so left of a
        // +X road is -Z. See src/osm/mesh_builder.cpp:480 for the same mapping.
        const glm::dvec3 world_normal{s.normal.x, 0.0, -s.normal.y};
        CHECK_NEAR(world_normal.x, 0.0, kTight);
        CHECK_NEAR(world_normal.z, -1.0, kTight);

        const glm::dvec3 world_left{left.x, 0.0, -left.y};
        CHECK_NEAR(world_left.z, -4.0, kExact);
    }
}

TEST(Centerline, left_normal_follows_the_tangent_around_a_turn) {
    // Heading +Y after the corner, so left is -X.
    const std::vector<glm::dvec2> poly = {{-10.0, 0.0}, {0.0, 0.0}, {0.0, 20.0}};
    const Centerline cl = build_centerline(poly, raw_config());
    if (!cl.is_valid()) return;

    const Station& last = cl.stations.back();
    CHECK_NEAR(last.tangent.x, 0.0, 1e-9);
    CHECK_NEAR(last.tangent.y, 1.0, 1e-9);
    CHECK_NEAR(last.normal.x, -1.0, 1e-9);
    CHECK_NEAR(last.normal.y, 0.0, 1e-9);

    const glm::dvec2 left = offset_point(last, 2.0);
    CHECK_NEAR(left.x, -2.0, 1e-9);
}

// ============================================================================
// Fold guard - the self-intersection guard the plan asks for at P2
// ============================================================================

namespace {

/// Unit vector along @p v, or zero when it has no usable direction
glm::dvec2 unit_or_zero(const glm::dvec2& v) {
    const double len = glm::length(v);
    return (len > 1e-12) ? (v / len) : glm::dvec2{0.0};
}

/**
 * @brief Largest backwards step any offset edge takes against the direction of travel
 *
 * A parallel offset that is not folded advances along the road at every band. A
 * fold is the moment it stops doing so: the offset column at the far end of a
 * band lands BEHIND the one at the near end, measured along that band's own
 * chord. This returns the worst such reversal in metres, so 0 means no fold.
 *
 * The chord, not the station tangents: a mitred joint's tangent is the bisector
 * of its two legs and does not describe the band it opens.
 *
 * @param cl      Centerline to walk
 * @param lateral Signed lateral coordinate of the offset edge, positive to the left
 * @return Metres of the deepest reversal, or 0 when the edge never reverses
 */
double worst_offset_reversal(const Centerline& cl, double lateral) {
    double worst = 0.0;
    for (size_t i = 0; i + 1 < cl.stations.size(); ++i) {
        const Station& a = cl.stations[i];
        const Station& b = cl.stations[i + 1];
        const glm::dvec2 dir = unit_or_zero(b.position - a.position);
        if (dir == glm::dvec2{0.0}) continue;   // the zero-length band of a bevel pair
        const glm::dvec2 step = offset_point(b, lateral) - offset_point(a, lateral);
        const double along = glm::dot(step, dir);
        if (along < -worst) worst = -along;
    }
    return worst;
}

} // namespace

TEST(Centerline, a_mitred_bend_never_folds_its_inner_offset_backwards) {
    // A 100 degree bend on 30 m arms with a 12 m residential profile. The corner
    // station's miter_scale is 1.56, so the unbounded inner offset at lateral +6
    // retreated 1.15 m PAST the previous station's column and the inner ribbon
    // edge ran backwards. Nothing in the plan permits that: P2 owes a fold guard.
    const std::vector<glm::dvec2> poly = {{-30.0, 0.0}, {0.0, 0.0}, {-5.209445, 29.544233}};
    const Centerline cl = build_centerline(poly, ResampleConfig{});
    CHECK_TRUE(cl.is_valid());
    if (!cl.is_valid()) return;
    check_no_nan(cl, "100 degree bend");

    for (double lateral : {6.0, 5.0, 3.0, -3.0, -5.0, -6.0}) {
        CHECK_NEAR(worst_offset_reversal(cl, lateral), 0.0, 1e-9);
    }
}

TEST(Centerline, a_hairpin_never_folds_either_offset_backwards) {
    // Past the miter limit the joint bevels, and the bevel's own zero-length
    // wedge band was the fold: its inner half swept backwards while its outer
    // half was a correct wedge.
    const std::vector<std::vector<glm::dvec2>> hairpins = {
        {{0.0, 0.0}, {100.0, 0.0}, {0.0, 1.0}},          // 179 degrees, doubling back
        {{0.0, 0.0}, {60.0, 0.0}, {4.5, 20.5}},          // about 160 degrees
        {{0.0, 0.0}, {60.0, 0.0}, {60.0, 10.0}, {0.0, 10.0}},   // switchback
    };

    for (size_t h = 0; h < hairpins.size(); ++h) {
        const Centerline cl = build_centerline(hairpins[h], ResampleConfig{});
        CHECK_TRUE(cl.is_valid());
        if (!cl.is_valid()) continue;
        check_no_nan(cl, "hairpin");

        for (double lateral : {6.0, 4.0, -4.0, -6.0}) {
            CHECK_NEAR(worst_offset_reversal(cl, lateral), 0.0, 1e-9);
        }
    }
}

TEST(Centerline, the_fold_bound_leaves_a_slack_joint_untouched) {
    // The guard must not narrow a corner that had room to mitre. A right angle on
    // 40 m arms resampled at 8 m allows 8 m of inner lateral, so a 6 m half width
    // still reaches its full mitred offset on BOTH sides.
    const std::vector<glm::dvec2> poly = {{-40.0, 0.0}, {0.0, 0.0}, {0.0, 40.0}};
    const Centerline cl = build_centerline(poly, raw_config());
    const Station* corner = station_at(cl, glm::dvec2{0.0, 0.0}, kExact);
    if (corner == nullptr) return;

    CHECK_NEAR(corner->miter_scale, std::sqrt(2.0), kExact);
    CHECK_NEAR(corner->lateral_max, 8.0, 1e-3);
    CHECK_EQ(corner->lateral_min, -stratum::osm::road::kUnboundedLateral);

    for (double w : {1.0, 3.0, 6.0, -1.0, -3.0, -6.0}) {
        const glm::dvec2 p = offset_point(*corner, w);
        CHECK_NEAR(p.x, -w, kExact);
        CHECK_NEAR(p.y, w, kExact);
    }
}

TEST(Centerline, a_straight_carries_no_lateral_bound_at_all) {
    const Centerline cl = build_centerline({{0.0, 0.0}, {80.0, 0.0}}, raw_config());
    CHECK_TRUE(cl.is_valid());
    for (const Station& s : cl.stations) {
        CHECK_EQ(s.lateral_max, stratum::osm::road::kUnboundedLateral);
        CHECK_EQ(s.lateral_min, -stratum::osm::road::kUnboundedLateral);
    }
}

// ============================================================================
// Smoothing fidelity
// ============================================================================

namespace {

/**
 * @brief Farthest any station sits from the surveyed polyline, in metres
 *
 * @param cl   Centerline whose stations are measured
 * @param poly The surveyed polyline they must stay near
 * @return The worst perpendicular distance to the nearest polyline segment
 */
double worst_deviation_from_polyline(const Centerline& cl, const std::vector<glm::dvec2>& poly) {
    double worst = 0.0;
    for (const Station& s : cl.stations) {
        double best = 1e30;
        for (size_t i = 1; i < poly.size(); ++i) {
            const glm::dvec2 a = poly[i - 1];
            const glm::dvec2 d = poly[i] - a;
            const double len_sq = glm::dot(d, d);
            const double t = (len_sq > 0.0)
                                 ? std::clamp(glm::dot(s.position - a, d) / len_sq, 0.0, 1.0)
                                 : 0.0;
            best = std::min(best, glm::length(s.position - (a + d * t)));
        }
        worst = std::max(worst, best);
    }
    return worst;
}

} // namespace

TEST(Centerline, smoothing_stays_within_max_smoothing_offset_of_the_survey) {
    // A 21 degree bend between two long rural arms. The Catmull-Rom bows off each
    // chord in proportion to its LENGTH, so this put the road 5.3 m from its own
    // survey, and tightening max_deviation never moved it: that tolerance bounds
    // the stations against the spline, never the spline against the survey.
    const std::vector<glm::dvec2> poly = {{0.0, 0.0}, {200.0, 0.0}, {375.877, 68.404}};

    for (double offset : {2.0, 0.5, 0.1}) {
        ResampleConfig cfg;
        cfg.max_smoothing_offset = offset;
        const Centerline cl = build_centerline(poly, cfg);
        CHECK_TRUE(cl.is_valid());
        if (!cl.is_valid()) continue;
        check_no_nan(cl, "long rural bend");

        // 1e-3 covers the flattener's own chord sampling of the bounded curve.
        CHECK_TRUE(worst_deviation_from_polyline(cl, poly) <= offset + 1e-3);
    }
}

TEST(Centerline, the_smoothing_bound_scales_with_neither_arm_length_nor_turn) {
    // The defect grew linearly with arm length and with turn angle. The bound has
    // to hold on every combination, not just the one it was tuned on.
    ResampleConfig cfg;
    cfg.max_smoothing_offset = 0.5;

    for (double arm : {100.0, 200.0, 400.0}) {
        for (double degrees : {20.0, 40.0, 55.0}) {
            const double turn = degrees * 3.14159265358979323846 / 180.0;
            const std::vector<glm::dvec2> poly = {
                {0.0, 0.0}, {arm, 0.0}, {arm + arm * std::cos(turn), arm * std::sin(turn)}};
            const Centerline cl = build_centerline(poly, cfg);
            if (!cl.is_valid()) continue;
            CHECK_TRUE(worst_deviation_from_polyline(cl, poly) <= cfg.max_smoothing_offset + 1e-3);
        }
    }
}

TEST(Centerline, smoothing_still_rounds_a_coarsely_surveyed_curve) {
    // The bound must not turn the smoother off. A 200 m radius arc surveyed every
    // 5.7 degrees bows a couple of centimetres per chord, far inside the bound, so
    // the fit still pulls the stations off the chords and onto the real arc.
    const std::vector<glm::dvec2> poly = arc_polyline(200.0, 1.6, 17);
    const Centerline smoothed = build_centerline(poly, ResampleConfig{});
    const Centerline raw = build_centerline(poly, raw_config());
    if (!smoothed.is_valid() || !raw.is_valid()) return;

    const auto worst_radial_error = [](const Centerline& cl) {
        double worst = 0.0;
        for (const Station& s : cl.stations) {
            worst = std::max(worst, std::fabs(glm::length(s.position) - 200.0));
        }
        return worst;
    };

    // The fit pulls the stations off the chords and towards the real arc.
    CHECK_TRUE(worst_radial_error(smoothed) < 0.6 * worst_radial_error(raw));

    // And the bound is not what shaped it: an effectively unbounded fit lands on
    // the same stations, so nothing was clipped on a curve this well surveyed.
    ResampleConfig loose;
    loose.max_smoothing_offset = 1e6;
    const Centerline unbounded = build_centerline(poly, loose);
    CHECK_EQ(unbounded.stations.size(), smoothed.stations.size());
    if (unbounded.stations.size() == smoothed.stations.size()) {
        for (size_t i = 0; i < smoothed.stations.size(); ++i) {
            CHECK_NEAR(smoothed.stations[i].position.x, unbounded.stations[i].position.x, 1e-9);
            CHECK_NEAR(smoothed.stations[i].position.y, unbounded.stations[i].position.y, 1e-9);
        }
    }
}

// ============================================================================
// slice() through a joint
// ============================================================================

TEST(Centerline, a_slice_endpoint_lands_on_the_untrimmed_ribbon_edge) {
    // The ribbon edge inside a band is the straight chord between the two
    // bracketing offset columns, because that is what the extruder draws. A
    // synthesised station whose normal and miter_scale were interpolated apart
    // missed that chord by 0.73 m per 6 m of half width at a right angle, making
    // the trimmed arm WIDER than the corridor it was cut from.
    ResampleConfig cfg = raw_config();
    cfg.max_spacing = 10.0;
    const Centerline cl = build_centerline({{0.0, 0.0}, {50.0, 0.0}, {50.0, 50.0}}, cfg);
    if (!cl.is_valid()) return;

    for (size_t i = 0; i + 1 < cl.stations.size(); ++i) {
        const Station& a = cl.stations[i];
        const Station& b = cl.stations[i + 1];
        const double span = b.arclength - a.arclength;
        if (!(span > 1e-6)) continue;

        for (double u : {0.25, 0.5, 0.75}) {
            const double cut_arc = a.arclength + span * u;
            const Centerline cut = slice(cl, cut_arc, cl.length());
            if (!cut.is_valid()) continue;
            const Station& s = cut.stations.front();
            if (std::fabs(s.arclength - cut_arc) > 1e-6) continue;

            for (double lateral : {6.0, 3.0, 0.0, -3.0, -6.0}) {
                const glm::dvec2 want =
                    offset_point(a, lateral) +
                    (offset_point(b, lateral) - offset_point(a, lateral)) * u;
                const glm::dvec2 got = offset_point(s, lateral);
                CHECK_NEAR(got.x, want.x, 1e-9);
                CHECK_NEAR(got.y, want.y, 1e-9);
            }
        }
    }
}
