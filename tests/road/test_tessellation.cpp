/**
 * @file test_tessellation.cpp
 * @brief Station decimation and coplanar quad merging
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The two passes tessellation.hpp describes remove triangles that carry no
 * information, which makes them the only reduction in the build that shrinks LOD
 * level 0, every level below it, the collision mesh and the export at once. That
 * also makes them the only reduction that can silently change what the player
 * sees, so every test here is a bound rather than a count.
 *
 * ### What each test is guarding
 *
 * - **The headline case.** A dead-straight run of 20 stations is a flat ribbon
 *   and needs 2 stations to describe exactly. If it comes back with 20 the pass
 *   is not running; if it comes back with 3 the merge is timid and the Lucan
 *   numbers do not move.
 * - **The deviation bound.** Nothing merged may depart from the curve by more
 *   than TessellationConfig::max_chord_deviation. Measured the way the header
 *   defines it -- perpendicular distance from each DROPPED station to the chord
 *   joining the two kept stations that bracket it.
 * - **The span cap.** Without it a motorway becomes two triangles and takes
 *   vertex lighting, culling granularity and terrain conformance with it.
 * - **What must survive.** Feature stations and bevel pairs. A crossing, a stop
 *   line and a junction trim all attach to an arclength, and every one of them
 *   breaks if the vertex column at that arclength is gone.
 * - **The lateral pass.** Four lanes of the same asphalt at the same height are
 *   four strips and therefore four columns of quads, and they are one rectangle.
 *   A kerb face is not part of that rectangle and must not be merged into it.
 *
 * ### A note on the vertical
 *
 * TessellationConfig::max_chord_deviation is a 2D measure and the header is
 * explicit that it must be: station heights are solved AFTER the centerline
 * exists (road_elevation.hpp), so Station carries no height and select_stations()
 * has none to read. A crest is therefore not protected by the deviation test --
 * it is protected by the two mechanisms that do exist, and both are asserted
 * below:
 *
 * 1. The caller passes the crest's station in @p feature_stations, which is what
 *    the mask is for and what every other arclength-anchored feature uses.
 * 2. TessellationConfig::max_span_length caps how much road one chord may stand
 *    in for, which bounds the vertical error of an unprotected crest at
 *    grade x span rather than leaving it unbounded.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests Tessellation
 * @endcode
 */

#include "framework.hpp"
#include "road/p7_fixtures.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/corridor.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/road/tessellation.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::SubMesh;
using stratum::Vertex;
using stratum::material_id_name;
using stratum::osm::road::Centerline;
using stratum::osm::road::Corridor;
using stratum::osm::road::CorridorConfig;
using stratum::osm::road::ResampleConfig;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::Station;
using stratum::osm::road::Strip;
using stratum::osm::road::StripKind;
using stratum::osm::road::TessellationConfig;
using stratum::osm::road::apply_station_selection;
using stratum::osm::road::build_centerline;
using stratum::osm::road::build_corridor;
using stratum::osm::road::merge_coplanar_quads;
using stratum::osm::road::select_stations;
using stratum::osm::road::uv_tiling;

namespace p7 = stratum::test::p7;

constexpr double kPi = 3.14159265358979323846;

// ============================================================================
// Centerlines built by hand
//
// Built rather than resampled wherever the test needs an exact station count.
// build_centerline() chooses its own density, so "20 stations" would otherwise be
// a property of the resampler rather than of the input.
// ============================================================================

/**
 * @brief A dead-straight run east with exactly @p n evenly spaced stations
 *
 * Every frame is the identity: tangent +x, left normal +y, no miter, no
 * curvature, no fold bound. This is the shape select_stations() must reduce to
 * two.
 */
Centerline make_straight(size_t n, double spacing) {
    Centerline cl;
    cl.stations.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        Station s;
        s.position = glm::dvec2{spacing * static_cast<double>(i), 0.0};
        s.tangent = glm::dvec2{1.0, 0.0};
        s.normal = glm::dvec2{0.0, 1.0};
        s.arclength = spacing * static_cast<double>(i);
        s.curvature = 0.0;
        s.miter_scale = 1.0;
        cl.stations.push_back(s);
    }
    return cl;
}

/// A circular arc, sampled densely enough that the resampler has something to thin
Centerline make_arc(double radius, double sweep_deg, double step_deg) {
    std::vector<glm::dvec2> poly;
    for (double a = 0.0; a <= sweep_deg + 1e-9; a += step_deg) {
        const double r = a * kPi / 180.0;
        poly.emplace_back(radius * std::sin(r), radius * (1.0 - std::cos(r)));
    }
    ResampleConfig cfg;
    cfg.smooth = false;
    return build_centerline(poly, cfg);
}

/// The hairpin the Centerline suite already proves bevels, at 175 degrees of turn
Centerline make_hairpin() {
    const double turn = 175.0 * kPi / 180.0;
    const std::vector<glm::dvec2> poly = {
        {-40.0, 0.0}, {0.0, 0.0}, {40.0 * std::cos(turn), 40.0 * std::sin(turn)}};
    ResampleConfig cfg;
    cfg.smooth = false;
    cfg.miter_limit = 4.0;
    return build_centerline(poly, cfg);
}

// ============================================================================
// Measurement
// ============================================================================

/// Perpendicular distance from @p p to the infinite line through @p a and @p b
double distance_to_chord(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 d = b - a;
    const double len = glm::length(d);
    if (!(len > 1e-12)) return glm::length(p - a);
    return std::fabs(d.x * (p.y - a.y) - d.y * (p.x - a.x)) / len;
}

/**
 * @brief The worst departure any dropped station makes from the chord replacing it
 *
 * The header's measure, exactly: for each pair of consecutive KEPT stations, the
 * maximum perpendicular distance from any station between them to the chord
 * joining them.
 */
double worst_deviation(const Centerline& cl, const std::vector<size_t>& keep) {
    double worst = 0.0;
    for (size_t k = 0; k + 1 < keep.size(); ++k) {
        const glm::dvec2& a = cl.stations[keep[k]].position;
        const glm::dvec2& b = cl.stations[keep[k + 1]].position;
        for (size_t i = keep[k] + 1; i < keep[k + 1]; ++i) {
            worst = std::max(worst, distance_to_chord(cl.stations[i].position, a, b));
        }
    }
    return worst;
}

/// True when @p keep is strictly ascending, in range, and starts and ends at the ends
bool selection_is_well_formed(const Centerline& cl, const std::vector<size_t>& keep) {
    if (cl.stations.empty()) return keep.empty();
    if (keep.size() < 2) return false;
    if (keep.front() != 0) return false;
    if (keep.back() != cl.stations.size() - 1) return false;
    for (size_t i = 0; i + 1 < keep.size(); ++i) {
        if (keep[i] >= keep[i + 1]) return false;
        if (keep[i + 1] >= cl.stations.size()) return false;
    }
    return true;
}

// ============================================================================
// Mesh measurement
// ============================================================================

/// 3D surface area of every triangle in one SubMesh range
double area_of_material(const Mesh& mesh, MaterialId material) {
    double total = 0.0;
    for (const SubMesh& range : mesh.effective_submeshes()) {
        if (range.material != material) continue;
        const size_t end = static_cast<size_t>(range.index_offset) + range.index_count;
        for (size_t i = range.index_offset; i + 2 < end && i + 2 < mesh.indices.size(); i += 3) {
            const glm::vec3& a = mesh.vertices[mesh.indices[i]].position;
            const glm::vec3& b = mesh.vertices[mesh.indices[i + 1]].position;
            const glm::vec3& c = mesh.vertices[mesh.indices[i + 2]].position;
            total += 0.5 * static_cast<double>(glm::length(glm::cross(b - a, c - a)));
        }
    }
    return total;
}

/// Highest and lowest world Y over the triangles of one material
void height_range_of_material(const Mesh& mesh, MaterialId material, double& out_min,
                              double& out_max) {
    out_min = 1e30;
    out_max = -1e30;
    for (const SubMesh& range : mesh.effective_submeshes()) {
        if (range.material != material) continue;
        const size_t end = static_cast<size_t>(range.index_offset) + range.index_count;
        for (size_t i = range.index_offset; i < end && i < mesh.indices.size(); ++i) {
            const double y = static_cast<double>(mesh.vertices[mesh.indices[i]].position.y);
            out_min = std::min(out_min, y);
            out_max = std::max(out_max, y);
        }
    }
}

/**
 * @brief Gradient of a per-vertex UV channel across one triangle, in the XZ plane
 *
 * "Linear UVs" means exactly this: the map from position to texture coordinate is
 * affine, so its gradient is a constant vector rather than something that changes
 * from triangle to triangle. A merge that rewrote U by stretching one half onto
 * the union without touching the other would leave the gradient different on
 * either side of the seam, which is what this measures.
 *
 * @param mesh    Mesh the triangle belongs to
 * @param i       Index of the triangle's first index
 * @param channel 0 for U, 1 for V
 * @param out     Receives the gradient in units per metre
 * @return False when the triangle is degenerate in plan and has no gradient
 */
bool uv_gradient(const Mesh& mesh, size_t i, int channel, glm::dvec2& out) {
    const Vertex& v0 = mesh.vertices[mesh.indices[i]];
    const Vertex& v1 = mesh.vertices[mesh.indices[i + 1]];
    const Vertex& v2 = mesh.vertices[mesh.indices[i + 2]];

    const glm::dvec2 e1{v1.position.x - v0.position.x, v1.position.z - v0.position.z};
    const glm::dvec2 e2{v2.position.x - v0.position.x, v2.position.z - v0.position.z};
    const double det = e1.x * e2.y - e1.y * e2.x;
    if (std::fabs(det) < 1e-9) return false;

    const double t0 = (channel == 0) ? v0.uv.x : v0.uv.y;
    const double t1 = (channel == 0) ? v1.uv.x : v1.uv.y;
    const double t2 = (channel == 0) ? v2.uv.x : v2.uv.y;
    const double d1 = t1 - t0;
    const double d2 = t2 - t0;

    out = glm::dvec2{(d1 * e2.y - d2 * e1.y) / det, (d2 * e1.x - d1 * e2.x) / det};
    return true;
}

/**
 * @brief Check that every triangle of one material carries the same UV gradient
 *
 * @param mesh     Mesh to inspect
 * @param material Material range to inspect
 * @param expected Expected gradient magnitude, units per metre, for both channels
 * @param label    Context for a failure message
 * @param file     Source file of the caller
 * @param line     Line of the caller
 */
void check_uvs_are_linear(const Mesh& mesh, MaterialId material, double expected,
                          const std::string& label, const char* file, int line) {
    size_t checked = 0;
    for (const SubMesh& range : mesh.effective_submeshes()) {
        if (range.material != material) continue;
        const size_t end = static_cast<size_t>(range.index_offset) + range.index_count;
        for (size_t i = range.index_offset; i + 2 < end && i + 2 < mesh.indices.size(); i += 3) {
            glm::dvec2 gu{0.0};
            glm::dvec2 gv{0.0};
            if (!uv_gradient(mesh, i, 0, gu)) continue;
            if (!uv_gradient(mesh, i, 1, gv)) continue;
            ++checked;

            const double lu = glm::length(gu);
            const double lv = glm::length(gv);
            if (std::fabs(lu - expected) > 1e-3 || std::fabs(lv - expected) > 1e-3) {
                stratum::test::report_failure(
                    file, line, "UVs stay linear across a merged quad",
                    label + ": " + material_id_name(material) + " triangle at index " +
                        std::to_string(i) + " has |grad u| " + std::to_string(lu) +
                        " and |grad v| " + std::to_string(lv) + ", expected " +
                        std::to_string(expected) + " for both");
                return;
            }
        }
    }
    if (checked == 0) {
        stratum::test::report_failure(file, line, "the material range has triangles to check",
                                      label + ": no " + material_id_name(material) +
                                          " triangle had a measurable gradient");
    }
}

#define CHECK_UVS_LINEAR(mesh, material, expected, label) \
    check_uvs_are_linear((mesh), (material), (expected), (label), __FILE__, __LINE__)

// ============================================================================
// Corridors
// ============================================================================

/// Four lanes of the same asphalt at the same height: one rectangle, drawn as four
RoadProfile flat_four_lane_profile() {
    RoadProfile profile;
    for (int i = 0; i < 4; ++i) {
        profile.strips.push_back(
            Strip{3.5f, 0.0f, 0.0f, MaterialId::Asphalt, StripKind::Lane});
    }
    return profile;
}

/// Straight, flat, extruded at base height 0 so a height assertion means something
Corridor extrude(const Centerline& cl, const RoadProfile& profile) {
    CorridorConfig cfg;
    cfg.base_height = 0.0f;
    return build_corridor(cl, profile, cfg);
}

} // namespace

// ============================================================================
// Longitudinal: the headline case
// ============================================================================

/**
 * A dead-straight run of 20 stations reduces to 2.
 *
 * This is the whole defect in one assertion. build_centerline() resamples at
 * ResampleConfig::max_spacing unconditionally, so a straight road pays for a
 * vertex column every 8 m to describe a shape that two columns describe exactly.
 * Measured on Lucan that is most of 3.19 million triangles.
 *
 * 95 m total, so TessellationConfig::max_span_length is not what is being tested
 * here; the deviation budget alone must take it to two.
 */
TEST(Tessellation, a_dead_straight_run_of_twenty_stations_reduces_to_two) {
    const Centerline cl = make_straight(20, 5.0);
    const TessellationConfig cfg;

    const std::vector<size_t> keep = select_stations(cl, cfg, nullptr);

    CHECK_EQ(keep.size(), size_t{2});
    CHECK_TRUE(selection_is_well_formed(cl, keep));
    if (keep.size() >= 2) {
        CHECK_EQ(keep.front(), size_t{0});
        CHECK_EQ(keep.back(), size_t{19});
    }

    // And the decimated centerline is usable: two stations is one band.
    const Centerline reduced = apply_station_selection(cl, keep);
    CHECK_TRUE(reduced.is_valid());
    CHECK_EQ(reduced.stations.size(), keep.size());

    // Arclength is carried, not recomputed. Everything held elsewhere -- markings,
    // crossings, trims, carve requests -- is expressed in the ORIGINAL
    // parameterisation and stays valid only because of this.
    if (reduced.stations.size() == 2) {
        CHECK_NEAR(reduced.stations.front().arclength, 0.0, 1e-12);
        CHECK_NEAR(reduced.stations.back().arclength, 95.0, 1e-9);
    }
}

/// A centerline too short to decimate comes back whole rather than empty
TEST(Tessellation, a_two_station_centerline_is_returned_whole) {
    const Centerline cl = make_straight(2, 5.0);
    const std::vector<size_t> keep = select_stations(cl, TessellationConfig{}, nullptr);
    CHECK_EQ(keep.size(), size_t{2});

    const Centerline empty;
    CHECK_TRUE(select_stations(empty, TessellationConfig{}, nullptr).empty());
}

// ============================================================================
// Longitudinal: the vertical
// ============================================================================

/**
 * A station on a crest -- straight in plan, curved in elevation -- is not merged
 * away.
 *
 * This is the one most likely to be got wrong, and the reason is structural
 * rather than arithmetical. The deviation test is a 2D measure and the header
 * says it must be: heights are solved after the centerline exists, so
 * select_stations() has none to read. A crest is dead straight in plan and the
 * chord test passes straight through it.
 *
 * The protection is the feature mask, which is the same mechanism a crossing, a
 * stop line and a junction trim use, and the caller is the one that knows where
 * the crest is. So the assertion is in two halves: the marked station survives,
 * and -- with the mask deliberately disabled -- it does NOT. Without the second
 * half the first would pass on a decimator that had simply given up.
 */
TEST(Tessellation, a_crest_station_marked_by_the_caller_is_not_merged_away) {
    const Centerline cl = make_straight(21, 5.0);   // 100 m, dead straight in plan

    // The caller solved elevation and knows station 10 is the crest.
    std::vector<bool> features(cl.stations.size(), false);
    features[10] = true;

    TessellationConfig cfg;
    cfg.preserve_feature_stations = true;
    const std::vector<size_t> kept = select_stations(cl, cfg, &features);

    CHECK_TRUE(selection_is_well_formed(cl, kept));
    const bool crest_survived =
        std::find(kept.begin(), kept.end(), size_t{10}) != kept.end();
    CHECK_TRUE(crest_survived);

    // The mask is what saved it, not luck: with the mask off, the same plan-straight
    // run collapses to its two ends and the crest goes.
    cfg.preserve_feature_stations = false;
    const std::vector<size_t> unprotected = select_stations(cl, cfg, &features);
    CHECK_EQ(unprotected.size(), size_t{2});
    CHECK_TRUE(std::find(unprotected.begin(), unprotected.end(), size_t{10}) ==
               unprotected.end());
}

/**
 * The vertical error of an UNMARKED crest is bounded by the span cap.
 *
 * The second of the two mechanisms named in the file header. A crest nobody
 * flagged is swallowed by a chord, and what stops that being unbounded is that
 * the chord may not be longer than TessellationConfig::max_span_length. At the
 * default 120 m and a residential grade limit of 8%, the worst a swallowed crest
 * can be out by is bounded rather than arbitrary.
 */
TEST(Tessellation, the_span_cap_bounds_an_unmarked_crest) {
    const Centerline cl = make_straight(101, 8.0);   // 800 m of dead straight
    TessellationConfig cfg;
    cfg.max_span_length = 120.0;

    const std::vector<size_t> keep = select_stations(cl, cfg, nullptr);
    CHECK_TRUE(selection_is_well_formed(cl, keep));

    for (size_t k = 0; k + 1 < keep.size(); ++k) {
        const double span = cl.stations[keep[k + 1]].arclength - cl.stations[keep[k]].arclength;
        if (span > cfg.max_span_length + 1e-6) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "no merged band exceeds max_span_length",
                "span of " + std::to_string(span) + " m between stations " +
                    std::to_string(keep[k]) + " and " + std::to_string(keep[k + 1]));
        }
    }

    // 800 m at 120 m per band needs at least 7 bands, so at least 8 stations.
    CHECK_TRUE(keep.size() >= size_t{8});

    // And it is still a real reduction: 101 stations must not survive as 101.
    CHECK_TRUE(keep.size() < size_t{20});
}

/// A tighter cap cuts more finely, a looser one less: the knob is actually wired up
TEST(Tessellation, the_span_cap_is_the_thing_that_sets_the_count) {
    const Centerline cl = make_straight(201, 4.0);   // 800 m again, denser

    TessellationConfig tight;
    tight.max_span_length = 40.0;
    TessellationConfig loose;
    loose.max_span_length = 200.0;

    const std::vector<size_t> tight_keep = select_stations(cl, tight, nullptr);
    const std::vector<size_t> loose_keep = select_stations(cl, loose, nullptr);

    CHECK_TRUE(tight_keep.size() > loose_keep.size());
    CHECK_TRUE(tight_keep.size() >= size_t{21});   // 800 / 40
    CHECK_TRUE(loose_keep.size() <= size_t{10});
}

// ============================================================================
// Longitudinal: the deviation bound on a real curve
// ============================================================================

/**
 * Nothing merged departs from the curve by more than max_chord_deviation.
 *
 * Measured on a 100 m radius arc, which is a real urban bend rather than a
 * synthetic one, and measured the way the header defines the budget: the
 * perpendicular distance from every DROPPED station to the chord joining the two
 * kept stations that bracket it.
 */
TEST(Tessellation, deviation_never_exceeds_the_budget_on_a_curve) {
    const Centerline cl = make_arc(100.0, 90.0, 2.0);
    if (!cl.is_valid() || cl.stations.size() < 8) {
        stratum::test::report_failure(__FILE__, __LINE__, "the arc fixture resampled",
                                      "make_arc produced " +
                                          std::to_string(cl.stations.size()) + " stations");
        return;
    }

    for (const double budget : {0.01, 0.03, 0.10, 0.50}) {
        TessellationConfig cfg;
        cfg.max_chord_deviation = budget;
        cfg.max_span_length = 1e9;   // isolate the deviation test from the span cap

        const std::vector<size_t> keep = select_stations(cl, cfg, nullptr);
        CHECK_TRUE(selection_is_well_formed(cl, keep));

        const double worst = worst_deviation(cl, keep);
        if (worst > budget + 1e-9) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "deviation stays inside the budget",
                "budget " + std::to_string(budget) + " m, worst departure " +
                    std::to_string(worst) + " m");
        }
    }
}

/// A looser budget must actually buy a coarser result, or the budget is decorative
TEST(Tessellation, a_looser_budget_keeps_fewer_stations) {
    const Centerline cl = make_arc(100.0, 90.0, 2.0);
    if (cl.stations.size() < 8) return;

    TessellationConfig tight;
    tight.max_chord_deviation = 0.005;
    tight.max_span_length = 1e9;
    TessellationConfig loose;
    loose.max_chord_deviation = 0.5;
    loose.max_span_length = 1e9;

    const size_t tight_count = select_stations(cl, tight, nullptr).size();
    const size_t loose_count = select_stations(cl, loose, nullptr).size();

    CHECK_TRUE(loose_count <= tight_count);
    CHECK_TRUE(loose_count < cl.stations.size());
}

// ============================================================================
// Longitudinal: what must survive
// ============================================================================

/**
 * Every station the caller flagged survives, wherever it is and however straight
 * the road around it is.
 *
 * A crossing, a stop line, a junction trim, a bridge span end and a tunnel portal
 * all attach to a specific arclength. The paint is placed against the station
 * column at that arclength, so losing the column detaches the paint from the road.
 */
TEST(Tessellation, every_feature_station_survives) {
    const Centerline cl = make_straight(41, 4.0);

    std::vector<bool> features(cl.stations.size(), false);
    const size_t marked[] = {3, 7, 8, 19, 33, 40};
    for (const size_t i : marked) features[i] = true;

    const std::vector<size_t> keep = select_stations(cl, TessellationConfig{}, &features);
    CHECK_TRUE(selection_is_well_formed(cl, keep));

    const std::set<size_t> kept(keep.begin(), keep.end());
    for (const size_t i : marked) {
        if (kept.count(i) == 0) {
            stratum::test::report_failure(__FILE__, __LINE__, "the feature station survived",
                                          "station " + std::to_string(i) + " was dropped");
        }
    }

    // The reduction still happened around them.
    CHECK_TRUE(keep.size() < cl.stations.size());
}

/// A short mask protects only the prefix it covers, and a long one has its tail ignored
TEST(Tessellation, a_mismatched_feature_mask_is_tolerated) {
    const Centerline cl = make_straight(21, 5.0);

    std::vector<bool> shorter(5, false);
    shorter[3] = true;
    const std::vector<size_t> from_short = select_stations(cl, TessellationConfig{}, &shorter);
    CHECK_TRUE(selection_is_well_formed(cl, from_short));
    CHECK_TRUE(std::find(from_short.begin(), from_short.end(), size_t{3}) != from_short.end());

    std::vector<bool> longer(60, false);
    longer[50] = true;
    const std::vector<size_t> from_long = select_stations(cl, TessellationConfig{}, &longer);
    CHECK_TRUE(selection_is_well_formed(cl, from_long));
    CHECK_EQ(from_long.size(), size_t{2});
}

/**
 * A bevel pair is a unit. Keeping one half and dropping the other turns the joint
 * into a mitre carrying the wrong normal, which is a visible crease pointing the
 * wrong way at exactly the sharpest corner in the network.
 */
TEST(Tessellation, bevel_pairs_are_kept_or_dropped_together) {
    const Centerline cl = make_hairpin();
    if (!cl.is_valid()) {
        stratum::test::report_failure(__FILE__, __LINE__, "the hairpin fixture built",
                                      "build_centerline returned an invalid centerline");
        return;
    }

    size_t bevels = 0;
    for (const Station& s : cl.stations) {
        if (s.is_bevel) ++bevels;
    }
    if (bevels == 0) {
        stratum::test::report_failure(__FILE__, __LINE__, "the hairpin bevelled",
                                      "no station is flagged is_bevel");
        return;
    }

    const std::vector<size_t> keep = select_stations(cl, TessellationConfig{}, nullptr);
    CHECK_TRUE(selection_is_well_formed(cl, keep));
    const std::set<size_t> kept(keep.begin(), keep.end());

    size_t pairs_seen = 0;
    for (size_t i = 0; i + 1 < cl.stations.size(); ++i) {
        if (!cl.stations[i].is_bevel || !cl.stations[i + 1].is_bevel) continue;
        ++pairs_seen;
        const bool a = kept.count(i) != 0;
        const bool b = kept.count(i + 1) != 0;
        if (a != b) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "a bevel pair is kept or dropped as a unit",
                "station " + std::to_string(i) + " kept=" + (a ? "yes" : "no") +
                    ", station " + std::to_string(i + 1) + " kept=" + (b ? "yes" : "no"));
        }
        // A 175 degree turn cannot be chorded away inside 0.03 m, so the apex must
        // survive on the deviation test alone.
        CHECK_TRUE(a);
        ++i;   // consumed both halves of this pair
    }
    CHECK_TRUE(pairs_seen > 0);

    // The decimated frame must still be sane after the merge rebuilt it.
    const Centerline reduced = apply_station_selection(cl, keep);
    for (const Station& s : reduced.stations) {
        CHECK_TRUE(std::isfinite(s.position.x) && std::isfinite(s.position.y));
        CHECK_TRUE(std::isfinite(s.miter_scale));
        CHECK_TRUE(s.miter_scale >= 1.0 - 1e-9);
        CHECK_NEAR(glm::length(s.tangent), 1.0, 1e-9);
        CHECK_NEAR(glm::length(s.normal), 1.0, 1e-9);
    }
}

/**
 * The fold bounds of a kept INTERIOR station are tightened over the span it
 * absorbed, not copied from the station itself.
 *
 * A kept station now stands for a whole run of dropped ones, so its bound has to
 * be the tightest over that run. Copying its own bound would let a wide profile
 * fold through a bend that no longer has a station in it -- and the fold guard
 * lives in offset_point(), so nothing downstream would ever notice.
 *
 * The two END stations are deliberately excluded, and the exclusion is asserted
 * by the test below rather than merely tolerated here: they are the stations the
 * junction solver already consumed, and they must come back bit-identical or the
 * ribbon stops registering with its junction mouth.
 */
TEST(Tessellation, fold_bounds_tighten_over_the_absorbed_span) {
    const Centerline cl = make_arc(30.0, 120.0, 2.0);   // tight enough to bound the inside
    if (cl.stations.size() < 6) return;

    TessellationConfig cfg;
    cfg.max_chord_deviation = 0.5;   // merge aggressively so spans really are absorbed
    const std::vector<size_t> keep = select_stations(cl, cfg, nullptr);
    if (keep.size() < 4 || keep.size() == cl.stations.size()) return;

    const Centerline reduced = apply_station_selection(cl, keep);
    CHECK_EQ(reduced.stations.size(), keep.size());

    for (size_t k = 1; k + 1 < keep.size(); ++k) {
        // The span this kept station stands for: everything dropped either side of
        // it, up to its kept neighbours.
        const size_t lo = keep[k - 1] + 1;
        const size_t hi = keep[k + 1] - 1;

        double tightest_max = cl.stations[keep[k]].lateral_max;
        double tightest_min = cl.stations[keep[k]].lateral_min;
        for (size_t i = lo; i <= hi && i < cl.stations.size(); ++i) {
            tightest_max = std::min(tightest_max, cl.stations[i].lateral_max);
            tightest_min = std::max(tightest_min, cl.stations[i].lateral_min);
        }

        // Never looser than the tightest bound over the absorbed span.
        CHECK_TRUE(reduced.stations[k].lateral_max <= tightest_max + 1e-9);
        CHECK_TRUE(reduced.stations[k].lateral_min >= tightest_min - 1e-9);
    }
}

/**
 * The first and last stations come back BIT-IDENTICAL, frame and fold bounds.
 *
 * ### What was wrong
 *
 * The pass rebuilt every station's frame from the decimated chord and charged the
 * absorbed runs' fold bounds to every kept station, the two ends included. That
 * is correct for an interior station and wrong for an end one.
 *
 * An edge's end station is the one the JUNCTION SOLVER already consumed:
 * arm_end() slices the same centerline at the same trim and offsets
 * `stations.front()` to place the junction polygon's arm mouth and the curb
 * ring's arm reach, and build_corridor() emits the ribbon's first column through
 * the identical offset_point() call. They register only while the station is
 * unchanged.
 *
 * Because a trim almost never lands on an existing station, slice() SYNTHESISES
 * that station: interpolate_station() sets its normal to the interpolated miter
 * and its miter_scale to that vector's length, while leaving the tangent equal to
 * the band chord. So the pass's `e == 0` branch used to overwrite a rotated miter
 * with the plain band normal and a miter_scale of 1 while the TANGENT looked
 * unchanged -- nothing else flagged it. On a 25 m kerb line the rotation is about
 * 2.3 degrees, which moves a 7 m offset corner a quarter of a metre along the
 * road, and the absorbed-run tightening could clamp the same column metres
 * further inboard on top of that.
 *
 * ### How this test fails without the fix
 *
 * The centerline curves at its start and runs straight afterwards, so the
 * straight decimates hard while the first band absorbs a real bend. Both halves
 * of the old behaviour then fire on station 0: the frame is re-derived and the
 * run bounds are charged to it.
 */
TEST(Tessellation, the_end_stations_survive_decimation_untouched) {
    // A tight bend followed by a long straight: the straight is what decimates,
    // the bend is what the end frame would be rebuilt against.
    std::vector<glm::dvec2> poly;
    const double radius = 25.0;
    for (double a = 0.0; a <= 40.0 + 1e-9; a += 2.0) {
        const double r = a * kPi / 180.0;
        poly.emplace_back(radius * std::sin(r), radius * (1.0 - std::cos(r)));
    }
    const glm::dvec2 last = poly.back();
    const glm::dvec2 dir = glm::normalize(last - poly[poly.size() - 2]);
    for (int i = 1; i <= 12; ++i) {
        poly.push_back(last + dir * (25.0 * static_cast<double>(i)));
    }

    ResampleConfig rcfg;
    rcfg.smooth = false;
    const Centerline full = build_centerline(poly, rcfg);
    if (full.stations.size() < 8) return;

    // Cut mid-band, the way a solved trim does, so the end stations are
    // INTERPOLATED and carry a miter frame the band normal does not reproduce.
    const double s0 = full.stations.front().arclength;
    const Centerline swept =
        stratum::osm::road::slice(full, s0 + 6.5, full.stations.back().arclength - 6.5);
    if (!swept.is_valid() || swept.stations.size() < 6) return;

    TessellationConfig cfg;
    const std::vector<size_t> keep = select_stations(swept, cfg, nullptr);
    CHECK_TRUE(keep.size() >= 2);
    CHECK_TRUE(keep.size() < swept.stations.size());   // decimation really happened
    if (keep.size() < 2 || keep.size() >= swept.stations.size()) return;

    const Centerline reduced = apply_station_selection(swept, keep, rcfg.miter_limit);
    CHECK_EQ(reduced.stations.size(), keep.size());
    if (reduced.stations.size() != keep.size()) return;

    const Station* ends[2] = {&reduced.stations.front(), &reduced.stations.back()};
    const Station* source[2] = {&swept.stations.front(), &swept.stations.back()};

    for (size_t e = 0; e < 2; ++e) {
        CHECK_TRUE(glm::distance(ends[e]->position, source[e]->position) <= 1e-12);
        CHECK_TRUE(glm::distance(ends[e]->tangent, source[e]->tangent) <= 1e-12);
        CHECK_TRUE(glm::distance(ends[e]->normal, source[e]->normal) <= 1e-12);
        CHECK_TRUE(std::fabs(ends[e]->miter_scale - source[e]->miter_scale) <= 1e-12);
        CHECK_TRUE(std::fabs(ends[e]->lateral_min - source[e]->lateral_min) <= 1e-12);
        CHECK_TRUE(std::fabs(ends[e]->lateral_max - source[e]->lateral_max) <= 1e-12);
    }

    // The consequence, measured the way the junction solver and the extruder
    // measure it: offset_point() on the two stations must agree exactly, at every
    // half-width a real profile uses.
    for (const double half : {3.5, 5.0, 7.0}) {
        for (size_t e = 0; e < 2; ++e) {
            for (const double lat : {half, -half}) {
                const glm::dvec2 a = stratum::osm::road::offset_point(*source[e], lat);
                const glm::dvec2 b = stratum::osm::road::offset_point(*ends[e], lat);
                CHECK_TRUE(glm::distance(a, b) <= 1e-9);
            }
        }
    }
}

// ============================================================================
// Lateral: merge_coplanar_quads
// ============================================================================

/**
 * Four lanes of the same asphalt at the same height are one rectangle.
 *
 * The longitudinal pass structurally cannot catch this: the four lanes are four
 * Strips whatever the station spacing is, so they are four columns of quads at
 * any density.
 */
TEST(Tessellation, a_flat_lane_band_merges_into_one_rectangle) {
    Corridor corridor = extrude(make_straight(26, 8.0), flat_four_lane_profile());
    const size_t before = p7::triangle_count(corridor.mesh);
    CHECK_TRUE(before > 0);

    const double area_before = area_of_material(corridor.mesh, MaterialId::Asphalt);

    const size_t merged = merge_coplanar_quads(corridor.mesh, TessellationConfig{});
    const size_t after = p7::triangle_count(corridor.mesh);

    CHECK_TRUE(merged > 0);

    // Each merge replaces two quads with one, so it removes exactly two triangles.
    CHECK_EQ(after + 2 * merged, before);
    CHECK_TRUE(after < before);

    // Four columns collapsing to one is a 75% cut before any longitudinal merging;
    // half is a floor that a correct implementation clears comfortably.
    CHECK_TRUE(after * 2 <= before);

    // A topological rewrite, not a simplification: no vertex moved and the surface
    // covers exactly what it covered.
    CHECK_NEAR(area_of_material(corridor.mesh, MaterialId::Asphalt), area_before, 1e-3);

    std::string reason;
    if (!p7::submeshes_tile_exactly(corridor.mesh, reason)) {
        stratum::test::report_failure(__FILE__, __LINE__, "submesh ranges still tile", reason);
    }
    CHECK_TRUE(p7::indices_are_sane(corridor.mesh));
    CHECK_TRUE(p7::mesh_is_finite(corridor.mesh));
}

/**
 * UVs stay linear across a merged quad.
 *
 * A strip boundary restarts U at 0, so two adjacent lanes are a U discontinuity
 * and the merge has to rewrite the survivor's U to span the union. Rewriting one
 * half and not the other, or spanning the union with the wrong scale, gives a
 * texture that stretches on one side of a seam that no longer exists -- and
 * nothing else in the pipeline would notice, because the geometry is right.
 *
 * The Asphalt tiling constant is 8 m per repeat in both axes, so the gradient of
 * both channels is 1/8 per metre everywhere, before the merge and after it.
 */
TEST(Tessellation, uvs_stay_linear_across_a_merged_quad) {
    Corridor corridor = extrude(make_straight(26, 8.0), flat_four_lane_profile());

    const auto tiling = uv_tiling(MaterialId::Asphalt);
    const double expected = 1.0 / static_cast<double>(tiling.u_metres);
    CHECK_NEAR(tiling.u_metres, tiling.v_metres, 1e-6);   // the premise of one expectation

    CHECK_UVS_LINEAR(corridor.mesh, MaterialId::Asphalt, expected, "before the merge");
    merge_coplanar_quads(corridor.mesh, TessellationConfig{});
    CHECK_UVS_LINEAR(corridor.mesh, MaterialId::Asphalt, expected, "after the merge");
}

/**
 * A kerb face does not merge into the lane beside it.
 *
 * The face is vertical and the lane is horizontal, so the coplanarity test is what
 * refuses it -- and if it did not, the merged quad would span from the
 * carriageway at y = 0 up to the kerb top at y = 0.15 and the step would vanish.
 * That is the single most visible thing P2 added, so it gets its own assertion
 * rather than being covered by an area total.
 */
TEST(Tessellation, a_kerb_does_not_merge_into_the_carriageway) {
    Corridor corridor = p7::kerbed_corridor(200.0);
    const Mesh before = corridor.mesh;

    const double asphalt_before = area_of_material(before, MaterialId::Asphalt);
    const double curb_before = area_of_material(before, MaterialId::Curb);
    const double sidewalk_before = area_of_material(before, MaterialId::Sidewalk);

    const size_t merged = merge_coplanar_quads(corridor.mesh, TessellationConfig{});
    CHECK_TRUE(merged > 0);

    // Nothing gained, nothing lost, per material.
    CHECK_NEAR(area_of_material(corridor.mesh, MaterialId::Asphalt), asphalt_before, 1e-3);
    CHECK_NEAR(area_of_material(corridor.mesh, MaterialId::Curb), curb_before, 1e-4);
    CHECK_NEAR(area_of_material(corridor.mesh, MaterialId::Sidewalk), sidewalk_before, 1e-3);

    // The carriageway is flat at 0 and must stay flat at 0. A kerb face merged into
    // it would drag a vertex to 0.15.
    double lo = 0.0;
    double hi = 0.0;
    height_range_of_material(corridor.mesh, MaterialId::Asphalt, lo, hi);
    CHECK_NEAR(lo, 0.0, 1e-4);
    CHECK_NEAR(hi, 0.0, 1e-4);

    // Every material present before is present after: a merge may not empty a range.
    const std::set<MaterialId> materials_before = p7::materials_of(before);
    const std::set<MaterialId> materials_after = p7::materials_of(corridor.mesh);
    CHECK_TRUE(materials_before == materials_after);

    std::string reason;
    if (!p7::submeshes_tile_exactly(corridor.mesh, reason)) {
        stratum::test::report_failure(__FILE__, __LINE__, "submesh ranges still tile", reason);
    }
    CHECK_TRUE(p7::indices_are_sane(corridor.mesh));
}

/// The switch switches off: no merge, no change, and the mesh is left alone
TEST(Tessellation, the_lateral_pass_can_be_turned_off) {
    Corridor corridor = extrude(make_straight(26, 8.0), flat_four_lane_profile());
    const Mesh before = corridor.mesh;

    TessellationConfig cfg;
    cfg.merge_coplanar_strips = false;

    CHECK_EQ(merge_coplanar_quads(corridor.mesh, cfg), size_t{0});
    CHECK_EQ(corridor.mesh.indices.size(), before.indices.size());
    CHECK_EQ(corridor.mesh.vertices.size(), before.vertices.size());
    CHECK_EQ(corridor.mesh.submeshes.size(), before.submeshes.size());
    CHECK_TRUE(p7::triangle_multiset(corridor.mesh) == p7::triangle_multiset(before));
}

/// An index buffer that is not a triangle list is refused rather than half-processed
TEST(Tessellation, a_malformed_index_buffer_is_refused) {
    Mesh mesh = p7::make_quad_mesh(MaterialId::Asphalt);
    mesh.indices.push_back(0);   // no longer a multiple of 3
    mesh.submeshes[0].index_count = static_cast<uint32_t>(mesh.indices.size());

    CHECK_EQ(merge_coplanar_quads(mesh, TessellationConfig{}), size_t{0});
    CHECK_EQ(mesh.indices.size(), size_t{7});
}

/// Two quads that merely touch at a corner share no edge and must not merge
TEST(Tessellation, quads_touching_at_a_corner_do_not_merge) {
    Mesh mesh = p7::make_quad_mesh(MaterialId::Asphalt);
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());

    // A second unit quad diagonally opposite, sharing exactly one corner at (1,0,-1).
    const glm::vec3 corners[4] = {
        {1.0f, 0.0f, -1.0f}, {2.0f, 0.0f, -1.0f},
        {2.0f, 0.0f, -2.0f}, {1.0f, 0.0f, -2.0f},
    };
    for (const glm::vec3& c : corners) {
        Vertex v{};
        v.position = c;
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v.uv = glm::vec2(c.x, -c.z);
        mesh.vertices.push_back(v);
    }
    for (const uint32_t i : {0u, 1u, 2u, 0u, 2u, 3u}) mesh.indices.push_back(base + i);
    mesh.submeshes[0].index_count = static_cast<uint32_t>(mesh.indices.size());

    CHECK_EQ(merge_coplanar_quads(mesh, TessellationConfig{}), size_t{0});
    CHECK_EQ(p7::triangle_count(mesh), size_t{4});
}
