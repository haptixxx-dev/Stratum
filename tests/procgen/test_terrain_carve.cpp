/**
 * @file test_terrain_carve.cpp
 * @brief Terrain carve tests: footprint, falloff, height interpolation, overlap, concurrency
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * These tests are written against the contract in src/procgen/terrain_carve.hpp.
 *
 * The carve is the half of P3 that the player actually sees. The elevation solve
 * decides where the road surface is; this decides whether the terrain meets it or
 * cuts through it. Three of its failure modes are invisible in a screenshot of a
 * flat test map and obvious on real terrain, so they are pinned here:
 *
 * 1. **Scalloping.** Snapping each cell to the nearest centerline STATION rather
 *    than projecting onto the nearest centerline SEGMENT quantises the corridor
 *    into a chain of circular arcs, worst on straights, where curvature-adaptive
 *    resampling puts the stations furthest apart. The interpolation test samples
 *    midway between two centerline points, which is exactly where the two
 *    implementations disagree most.
 * 2. **Averaging overlaps.** Blending two footprints at different heights gives a
 *    surface matching neither, with a step at both kerbs. The overlap test asserts
 *    the carved value is one of the two road heights and never the mean.
 * 3. **Order dependence.** The carve runs per chunk and chunks are generated on
 *    demand from several threads. If the result depended on which thread got
 *    there first, terrain would differ between runs of the same import. The
 *    concurrency test carves the same const CarveInput from many threads and
 *    compares against the serial result bit for bit.
 *
 * Coordinates: the payload is 2D local metres and a Heightmap is indexed in the
 * SAME 2D local frame -- its second axis IS local y, with no sign flip. The
 * render negation `(x, y_2d) -> vec3(x, h, -y_2d)` is applied downstream, by the
 * corridor extruder and by TerrainMeshBuilder independently, so it must not be
 * applied here. Every expected cell index below is derived through
 * iz_of_local_y() rather than assumed.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests TerrainCarve
 * @endcode
 */

#include "framework.hpp"

#include "procgen/terrain_carve.hpp"
#include "procgen/terrain_generator.hpp"

#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace {

using stratum::procgen::CarveConfig;
using stratum::procgen::CarveDisc;
using stratum::procgen::CarveInput;
using stratum::procgen::CarvePortal;
using stratum::procgen::CarveRibbon;
using stratum::procgen::CarveStats;
using stratum::procgen::Heightmap;
using stratum::procgen::TerrainConfig;
using stratum::procgen::TerrainGenerator;
using stratum::procgen::TerrainType;
using stratum::procgen::carve_terrain;

// ============================================================================
// Heightmap helpers
// ============================================================================

/**
 * @brief Build a heightmap sampled from a closed-form surface
 *
 * @param width    Samples along world x
 * @param height   Samples along world z
 * @param cell     Metres per cell in both axes
 * @param origin   World position of grid node (0, 0)
 * @param surface  Natural surface, called with world x and world z
 * @return The filled heightmap
 */
Heightmap make_heightmap(int width, int height, float cell, glm::vec2 origin,
                         const std::function<float(float, float)>& surface) {
    Heightmap hm;
    hm.width = width;
    hm.height = height;
    hm.cell_size_x = cell;
    hm.cell_size_z = cell;
    hm.origin = origin;
    hm.data.resize(static_cast<size_t>(width) * static_cast<size_t>(height), 0.0f);

    for (int iz = 0; iz < height; ++iz) {
        for (int ix = 0; ix < width; ++ix) {
            const float wx = origin.x + static_cast<float>(ix) * cell;
            const float wz = origin.y + static_cast<float>(iz) * cell;
            hm.data[static_cast<size_t>(iz) * static_cast<size_t>(width)
                    + static_cast<size_t>(ix)] = surface(wx, wz);
        }
    }
    return hm;
}

/// A flat surface at 0 m
float flat_ground(float, float) {
    return 0.0f;
}

/**
 * @brief Gentle plane rising 2% along world x
 *
 * Gentle on purpose. CarveConfig::max_embankment_slope widens the blend band
 * locally when the road-to-ground difference is too large to close inside
 * falloff_metres, and a widened band would reach past the cells this suite
 * asserts are untouched. At 2% the difference stays under 4 m and the required
 * embankment stays under 0.4, well inside the 0.6 default.
 */
float gentle_slope(float wx, float) {
    return 1.0f + 0.02f * wx;
}

/// Grid index of a heightmap node
size_t cell_index(const Heightmap& hm, int ix, int iz) {
    return static_cast<size_t>(iz) * static_cast<size_t>(hm.width) + static_cast<size_t>(ix);
}

/**
 * @brief Grid column index for a world x
 *
 * @param hm World_x is resolved against this heightmap's origin and cell size
 * @param wx World x in metres, which must land on a grid node
 * @return Column index
 */
int ix_of(const Heightmap& hm, double wx) {
    return static_cast<int>(std::lround((wx - static_cast<double>(hm.origin.x))
                                        / static_cast<double>(hm.cell_size_x)));
}

/**
 * @brief Grid row index for a 2D local y
 *
 * A Heightmap's second axis IS the 2D local y, so no sign flip is applied.
 * Written once here so no test below can get it wrong silently.
 *
 * @param hm Heightmap to index
 * @param y2d 2D local y in metres
 * @return Row index
 */
int iz_of_local_y(const Heightmap& hm, double y2d) {
    return static_cast<int>(std::lround((y2d - static_cast<double>(hm.origin.y))
                                        / static_cast<double>(hm.cell_size_z)));
}

/// Height at a 2D local position that lands on a grid node
float at_local(const Heightmap& hm, double x, double y2d) {
    return hm.at(ix_of(hm, x), iz_of_local_y(hm, y2d));
}

/// True when two heightmaps hold the same values, compared exactly
bool identical(const Heightmap& a, const Heightmap& b) {
    if (a.width != b.width || a.height != b.height) return false;
    if (a.data.size() != b.data.size()) return false;
    for (size_t i = 0; i < a.data.size(); ++i) {
        if (!(a.data[i] == b.data[i])) return false;
    }
    return true;
}

/// True when every height is a finite number
bool all_finite(const Heightmap& hm) {
    for (float h : hm.data) {
        if (!std::isfinite(h)) return false;
    }
    return true;
}

// ============================================================================
// Ribbon helpers
// ============================================================================

/**
 * @brief A ribbon running along +X at a fixed 2D y
 *
 * The outline is the exact rectangle the profile would sweep: along the RIGHT
 * edge from the first station to the last, then back along the LEFT edge, which
 * is counter-clockwise in the 2D plane where positive lateral is to the left.
 *
 * @param x0         Start x in metres
 * @param x1         End x in metres
 * @param y2d        Constant 2D y in metres
 * @param height     Road surface world Y in metres
 * @param half_width Half the profile width in metres
 * @return The ribbon
 */
CarveRibbon straight_ribbon(double x0, double x1, double y2d, float height, float half_width) {
    CarveRibbon r;
    r.centerline = {{x0, y2d}, {x1, y2d}};
    r.centerline_heights = {height, height};
    r.half_width = half_width;
    r.outline = {
        {x0, y2d - half_width},
        {x1, y2d - half_width},
        {x1, y2d + half_width},
        {x0, y2d + half_width},
    };
    r.outline_is_simple = true;
    return r;
}

/**
 * @brief A ribbon along +X whose surface height varies per centerline point
 *
 * @param xs         Centerline x coordinates, ascending
 * @param heights    World Y per point, parallel to @p xs
 * @param half_width Half the profile width in metres
 * @return The ribbon, at 2D y = 0
 */
CarveRibbon ramped_ribbon(const std::vector<double>& xs,
                          const std::vector<float>& heights,
                          float half_width) {
    CarveRibbon r;
    r.half_width = half_width;
    for (double x : xs) r.centerline.push_back({x, 0.0});
    r.centerline_heights = heights;

    for (double x : xs) r.outline.push_back({x, -static_cast<double>(half_width)});
    for (auto it = xs.rbegin(); it != xs.rend(); ++it) {
        r.outline.push_back({*it, static_cast<double>(half_width)});
    }
    r.outline_is_simple = true;
    return r;
}

/// Wrap ribbons into an indexed CarveInput
CarveInput make_input(std::vector<CarveRibbon> ribbons, const CarveConfig& config) {
    CarveInput input;
    input.ribbons = std::move(ribbons);
    input.config = config;
    input.build_index();
    return input;
}

// ============================================================================
// Junction helpers
// ============================================================================

/**
 * @brief A junction footprint shaped like an axis-aligned square ring
 *
 * A square is the sharpest test shape available: every point of it that is not a
 * corner is nearer the boundary than a circumscribing circle would put it, so a
 * carve that quietly fell back to the disc reads differently at four known
 * places rather than nowhere.
 *
 * Wound counter-clockwise and with the first point not repeated, matching the
 * CarveDisc::outline contract.
 *
 * @param center Junction centre in 2D local metres
 * @param half   Half the square's side, metres
 * @param height World Y to carve the junction to
 * @param radius Stated fallback radius; must bound the ring unless a test is
 *               deliberately lying about it
 * @return The junction footprint
 */
CarveDisc square_junction(glm::dvec2 center, double half, float height, float radius) {
    CarveDisc d;
    d.center = center;
    d.height = height;
    d.radius = radius;
    d.outline = {
        {center.x - half, center.y - half},
        {center.x + half, center.y - half},
        {center.x + half, center.y + half},
        {center.x - half, center.y + half},
    };
    d.outline_is_simple = true;
    return d;
}

/// The radius of the circle that just contains square_junction()'s ring
double square_bounding_radius(double half) {
    return half * std::sqrt(2.0);
}

/// Wrap ribbons and junctions into an indexed CarveInput
CarveInput make_input(std::vector<CarveRibbon> ribbons,
                      std::vector<CarveDisc> discs,
                      const CarveConfig& config) {
    CarveInput input;
    input.ribbons = std::move(ribbons);
    input.discs = std::move(discs);
    input.config = config;
    input.build_index();
    return input;
}

// ============================================================================
// Tunnel portal helpers
// ============================================================================

/**
 * @brief A portal mouth facing back along -X, cut into a hillside that rises with +X
 *
 * @param face   Centre of the OPENING in 2D local metres, on the centerline
 * @param axis   Unit direction INTO the hillside
 * @param half_w Half the opening width including its surround, metres
 * @param depth  How far along @p axis the notch runs, metres
 * @param crown  World Y of the top of the arch: the height the ground is clamped to
 * @return The footprint
 */
CarvePortal portal_mouth(glm::dvec2 face, glm::dvec2 axis, double half_w, double depth,
                         float crown) {
    CarvePortal p;
    p.center = face;
    p.axis = axis;
    p.half_width = half_w;
    p.depth = depth;
    p.surface_height = crown - 5.0f;
    p.crown_height = crown;
    p.at_start = true;
    return p;
}

/// Wrap ribbons, junctions and portal mouths into an indexed CarveInput
CarveInput make_input(std::vector<CarveRibbon> ribbons,
                      std::vector<CarveDisc> discs,
                      std::vector<CarvePortal> portals,
                      const CarveConfig& config) {
    CarveInput input;
    input.ribbons = std::move(ribbons);
    input.discs = std::move(discs);
    input.portals = std::move(portals);
    input.config = config;
    input.build_index();
    return input;
}

/**
 * @brief A hillside standing on level ground: flat below x = 0, rising 2:1 above it
 *
 * Steep on purpose. A portal mouth only bites where the hill has climbed above
 * the crown of the arch, so a gentle slope would leave most of the assertions
 * testing the inert case by accident.
 */
float hillside(float wx, float) {
    return wx > 0.0f ? 2.0f * wx : 0.0f;
}

} // namespace

// ============================================================================
// Footprint, falloff, and the band between them
// ============================================================================

TEST(TerrainCarve, straight_ribbon_flattens_its_footprint_and_blends_outward) {
    // World x in [0, 140], world z in [-20, 20], one sample per metre.
    Heightmap hm = make_heightmap(141, 41, 1.0f, glm::vec2{0.0f, -20.0f}, gentle_slope);
    const Heightmap before = hm;

    CarveConfig config;
    config.falloff_metres = 10.0f;

    const float road_height = 5.0f;
    const float half_width = 5.0f;

    // The ribbon overhangs the heightmap at both ends, so the column sampled
    // below is influenced only by lateral distance and never by an end cap.
    CarveInput input = make_input({straight_ribbon(-10.0, 150.0, 0.0, road_height, half_width)},
                                  config);
    CHECK_TRUE(input.has_index());
    CHECK_EQ(input.item_count(), size_t{1});

    const CarveStats stats = carve_terrain(hm, input);

    CHECK_TRUE(stats.cells_modified > size_t{0});
    CHECK_TRUE(stats.max_delta > 0.0f);
    CHECK_TRUE(all_finite(hm));

    const double probe_x = 70.0;

    // Inside the footprint the terrain IS the road surface, exactly.
    size_t inside_wrong = 0;
    for (int y = -4; y <= 4; ++y) {
        if (std::fabs(static_cast<double>(at_local(hm, probe_x, y)) - road_height) > 1e-4) {
            ++inside_wrong;
        }
    }
    CHECK_EQ(inside_wrong, size_t{0});

    // Beyond half_width + falloff nothing may have moved. This is what keeps a
    // road from flattening the hill it runs along.
    size_t outside_touched = 0;
    for (int iz = 0; iz < hm.height; ++iz) {
        const double y2d = static_cast<double>(hm.origin.y)
                         + static_cast<double>(iz) * static_cast<double>(hm.cell_size_z);
        if (std::fabs(y2d) < static_cast<double>(half_width + config.falloff_metres) + 0.5) {
            continue;
        }
        for (int ix = 0; ix < hm.width; ++ix) {
            if (!(hm.data[cell_index(hm, ix, iz)] == before.data[cell_index(hm, ix, iz)])) {
                ++outside_touched;
            }
        }
    }
    CHECK_EQ(outside_touched, size_t{0});

    // Across the band the height falls monotonically from the road down to the
    // natural surface as lateral distance grows.
    const double natural = static_cast<double>(gentle_slope(static_cast<float>(probe_x), 0.0f));
    std::vector<double> band;
    for (int y = 5; y <= 16; ++y) {
        band.push_back(static_cast<double>(at_local(hm, probe_x, y)));
    }

    size_t non_monotone = 0;
    size_t out_of_range = 0;
    for (size_t i = 0; i < band.size(); ++i) {
        if (band[i] > static_cast<double>(road_height) + 1e-4
            || band[i] < natural - 1e-4) {
            ++out_of_range;
        }
        if (i + 1 < band.size() && band[i + 1] > band[i] + 1e-5) ++non_monotone;
    }
    CHECK_EQ(non_monotone, size_t{0});
    CHECK_EQ(out_of_range, size_t{0});

    // The band ends on the natural surface, not part way down it.
    CHECK_NEAR(band.back(), natural, 1e-3);

    // No crease. A hard step from road height to natural would put the whole
    // drop into one second difference; a smoothstep or even a linear ramp puts
    // in at most the slope change at the joints.
    const double drop = static_cast<double>(road_height) - natural;
    const double crease_bound = 2.0 * drop / static_cast<double>(config.falloff_metres);
    size_t creases = 0;
    for (int y = 4; y <= 16; ++y) {
        const double a = static_cast<double>(at_local(hm, probe_x, y - 1));
        const double b = static_cast<double>(at_local(hm, probe_x, y));
        const double c = static_cast<double>(at_local(hm, probe_x, y + 1));
        if (std::fabs(a - 2.0 * b + c) > crease_bound) ++creases;
    }
    CHECK_EQ(creases, size_t{0});

    // The carve is symmetric about the centerline: nothing in the payload
    // distinguishes left from right.
    size_t asymmetric = 0;
    for (int y = 1; y <= 16; ++y) {
        const double left = static_cast<double>(at_local(hm, probe_x, y));
        const double right = static_cast<double>(at_local(hm, probe_x, -y));
        if (std::fabs(left - right) > 1e-4) ++asymmetric;
    }
    CHECK_EQ(asymmetric, size_t{0});
}

// ============================================================================
// Height interpolation along the ribbon -- the scalloping test
// ============================================================================

TEST(TerrainCarve, carved_height_follows_the_centerline_ramp_without_scalloping) {
    Heightmap hm = make_heightmap(81, 41, 1.0f, glm::vec2{0.0f, -20.0f}, flat_ground);

    // Centerline points every 20 m with an uneven height ramp. Nearest-station
    // snapping would return a neighbouring vertex height at every midpoint, which
    // is off by 1.0 m at x = 10 and by 2.5 m at x = 50.
    const std::vector<double> xs = {0.0, 20.0, 40.0, 60.0, 80.0};
    const std::vector<float> hs = {0.0f, 2.0f, 3.0f, 8.0f, 9.0f};

    CarveConfig config;
    config.falloff_metres = 10.0f;
    CarveInput input = make_input({ramped_ribbon(xs, hs, 5.0f)}, config);
    CHECK_TRUE(input.has_index());

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_TRUE(stats.cells_modified > size_t{0});
    CHECK_TRUE(all_finite(hm));

    // On the centerline points themselves the carved height is the given height.
    for (size_t i = 0; i < xs.size(); ++i) {
        CHECK_NEAR(at_local(hm, xs[i], 0.0), hs[i], 0.05);
    }

    // Midway between points it is the LINEAR interpolation of the two, which is
    // the assertion a nearest-vertex implementation cannot pass.
    CHECK_NEAR(at_local(hm, 10.0, 0.0), 1.0, 0.1);
    CHECK_NEAR(at_local(hm, 30.0, 0.0), 2.5, 0.1);
    CHECK_NEAR(at_local(hm, 50.0, 0.0), 5.5, 0.1);
    CHECK_NEAR(at_local(hm, 70.0, 0.0), 8.5, 0.1);

    // And at every metre along the ribbon, not only at the four midpoints: a
    // scallop is a bulge between stations, so it is the whole run that has to be
    // straight.
    size_t off_ramp = 0;
    for (int x = 0; x <= 80; ++x) {
        const double t = static_cast<double>(x);
        const size_t seg = std::min<size_t>(static_cast<size_t>(t / 20.0), xs.size() - 2);
        const double u = (t - xs[seg]) / (xs[seg + 1] - xs[seg]);
        const double expected = static_cast<double>(hs[seg])
                                + u * (static_cast<double>(hs[seg + 1])
                                       - static_cast<double>(hs[seg]));
        if (std::fabs(static_cast<double>(at_local(hm, t, 0.0)) - expected) > 0.1) {
            ++off_ramp;
        }
    }
    CHECK_EQ(off_ramp, size_t{0});

    // The same ramp holds across the width of the carriageway, not only on the
    // centerline itself.
    size_t off_ramp_offset = 0;
    for (int x = 10; x <= 70; x += 10) {
        const double centre = static_cast<double>(at_local(hm, x, 0.0));
        for (int y = -3; y <= 3; ++y) {
            if (std::fabs(static_cast<double>(at_local(hm, x, y)) - centre) > 1e-3) {
                ++off_ramp_offset;
            }
        }
    }
    CHECK_EQ(off_ramp_offset, size_t{0});
}

// ============================================================================
// Overlapping corridors
// ============================================================================

TEST(TerrainCarve, overlapping_ribbons_never_average) {
    Heightmap hm = make_heightmap(81, 41, 1.0f, glm::vec2{0.0f, -20.0f}, flat_ground);

    const float low = 10.0f;
    const float high = 20.0f;

    CarveConfig config;
    config.falloff_metres = 10.0f;

    // Two parallel corridors 6 m apart, each 10 m wide, so their footprints
    // overlap over a 4 m strip. Ten metres of height between them makes an
    // average unmistakable.
    CarveInput input = make_input({
                                      straight_ribbon(-10.0, 90.0, 0.0, low, 5.0f),
                                      straight_ribbon(-10.0, 90.0, 6.0, high, 5.0f),
                                  },
                                  config);
    CHECK_TRUE(input.has_index());
    CHECK_EQ(input.item_count(), size_t{2});

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_TRUE(stats.cells_modified > size_t{0});
    CHECK_TRUE(all_finite(hm));

    const double probe_x = 40.0;

    // Unambiguously nearer one or the other.
    CHECK_NEAR(at_local(hm, probe_x, 0.0), low, 1e-3);
    CHECK_NEAR(at_local(hm, probe_x, 6.0), high, 1e-3);
    CHECK_NEAR(at_local(hm, probe_x, 2.0), low, 1e-3);
    CHECK_NEAR(at_local(hm, probe_x, 4.0), high, 1e-3);

    // Across the whole overlap strip, including the tie at y = 3, every carved
    // cell is one road height or the other and never something between.
    size_t averaged = 0;
    for (int y = 1; y <= 5; ++y) {
        const double v = static_cast<double>(at_local(hm, probe_x, y));
        const bool is_low = std::fabs(v - static_cast<double>(low)) <= 1e-3;
        const bool is_high = std::fabs(v - static_cast<double>(high)) <= 1e-3;
        if (!is_low && !is_high) {
            ++averaged;
            stratum::test::report_failure(
                __FILE__, __LINE__, "overlap cell is one road height or the other",
                "2D y " + std::to_string(y) + " carved to "
                    + stratum::test::stringify(v));
        }
    }
    CHECK_EQ(averaged, size_t{0});

    // The mean of the two is the specific wrong answer being ruled out.
    size_t mean_valued = 0;
    for (int y = 1; y <= 5; ++y) {
        if (std::fabs(static_cast<double>(at_local(hm, probe_x, y)) - 15.0) < 1.0) {
            ++mean_valued;
        }
    }
    CHECK_EQ(mean_valued, size_t{0});
}

// ============================================================================
// Suppression
// ============================================================================

TEST(TerrainCarve, suppressed_items_leave_the_heightmap_bit_identical) {
    Heightmap hm = make_heightmap(81, 41, 1.0f, glm::vec2{0.0f, -20.0f}, gentle_slope);
    const Heightmap before = hm;

    CarveConfig config;

    CarveRibbon tunnel = straight_ribbon(-10.0, 90.0, 0.0, -6.0f, 5.0f);
    tunnel.suppress = true;

    CarveRibbon deck = straight_ribbon(-10.0, 90.0, 30.0, 40.0f, 6.0f);
    deck.suppress = true;

    CarveInput input;
    input.ribbons = {tunnel, deck};

    CarveDisc disc;
    disc.center = {40.0, 0.0};
    disc.radius = 12.0f;
    disc.height = 40.0f;
    disc.suppress = true;
    input.discs = {disc};

    input.config = config;
    input.build_index();
    CHECK_TRUE(input.has_index());
    CHECK_EQ(input.item_count(), size_t{3});
    CHECK_TRUE(input.is_disc(2));
    CHECK_EQ(input.disc_index(2), size_t{0});

    const CarveStats stats = carve_terrain(hm, input);

    // A tunnel roadway is below ground and a bridge deck floats above it; in
    // both cases the natural surface is the correct one, so not one cell moves.
    CHECK_EQ(stats.cells_modified, size_t{0});
    CHECK_NEAR(stats.max_delta, 0.0, 0.0);
    CHECK_TRUE(identical(hm, before));
}

// ============================================================================
// Concurrency
// ============================================================================

TEST(TerrainCarve, concurrent_carves_match_the_serial_carve) {
    CarveConfig config;
    config.falloff_metres = 8.0f;

    // Several corridors at different headings and heights, so the per-cell
    // winner is contested rather than obvious.
    CarveRibbon a = straight_ribbon(-20.0, 120.0, 0.0, 6.0f, 5.0f);
    CarveRibbon b = straight_ribbon(-20.0, 120.0, 14.0, 9.0f, 4.0f);

    CarveRibbon diagonal;
    diagonal.centerline = {{0.0, -30.0}, {50.0, 10.0}, {100.0, 40.0}};
    diagonal.centerline_heights = {2.0f, 7.0f, 11.0f};
    diagonal.half_width = 4.5f;
    diagonal.outline.clear();
    diagonal.outline_is_simple = false;

    CarveRibbon cross;
    cross.centerline = {{60.0, -40.0}, {60.0, 40.0}};
    cross.centerline_heights = {3.0f, 12.0f};
    cross.half_width = 6.0f;
    cross.outline = {{55.0, -40.0}, {65.0, -40.0}, {65.0, 40.0}, {55.0, 40.0}};
    cross.outline_is_simple = true;

    const CarveInput input = make_input({a, b, diagonal, cross}, config);
    CHECK_TRUE(input.has_index());
    CHECK_EQ(input.item_count(), size_t{4});

    const Heightmap pristine =
        make_heightmap(101, 81, 1.0f, glm::vec2{0.0f, -40.0f}, gentle_slope);

    Heightmap reference = pristine;
    const CarveStats serial_stats = carve_terrain(reference, input);
    CHECK_TRUE(serial_stats.cells_modified > size_t{0});
    CHECK_TRUE(all_finite(reference));

    // Enough rounds and enough threads that a data race or an order-dependent
    // tie-break has a real chance to show. A single pass would prove nothing.
    constexpr int kRounds = 16;
    constexpr int kThreads = 8;

    size_t mismatched = 0;
    size_t wrong_stats = 0;

    for (int round = 0; round < kRounds; ++round) {
        std::vector<Heightmap> maps(kThreads, pristine);
        std::vector<CarveStats> results(kThreads);
        std::vector<std::thread> workers;
        workers.reserve(kThreads);

        for (int t = 0; t < kThreads; ++t) {
            workers.emplace_back([&input, &maps, &results, t]() {
                results[static_cast<size_t>(t)] =
                    carve_terrain(maps[static_cast<size_t>(t)], input);
            });
        }
        for (auto& w : workers) w.join();

        for (int t = 0; t < kThreads; ++t) {
            if (!identical(maps[static_cast<size_t>(t)], reference)) ++mismatched;
            if (results[static_cast<size_t>(t)].cells_modified != serial_stats.cells_modified) {
                ++wrong_stats;
            }
        }
    }

    CHECK_EQ(mismatched, size_t{0});
    CHECK_EQ(wrong_stats, size_t{0});
}

// ============================================================================
// Degenerate input
// ============================================================================

TEST(TerrainCarve, empty_input_changes_nothing) {
    Heightmap hm = make_heightmap(41, 41, 1.0f, glm::vec2{0.0f, -20.0f}, gentle_slope);
    const Heightmap before = hm;

    CarveInput input;
    input.build_index();

    CHECK_TRUE(input.has_index());
    CHECK_EQ(input.item_count(), size_t{0});

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_EQ(stats.cells_modified, size_t{0});
    CHECK_NEAR(stats.max_delta, 0.0, 0.0);
    CHECK_TRUE(identical(hm, before));
}

TEST(TerrainCarve, unindexed_input_carves_nothing) {
    Heightmap hm = make_heightmap(41, 41, 1.0f, glm::vec2{0.0f, -20.0f}, gentle_slope);
    const Heightmap before = hm;

    // A missing build_index() must show up as flat terrain and zeroed stats, not
    // as a silent fall back to a linear scan that turns a city import quadratic.
    CarveInput input;
    input.ribbons = {straight_ribbon(-10.0, 50.0, 0.0, 8.0f, 5.0f)};
    CHECK_FALSE(input.has_index());

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_EQ(stats.cells_modified, size_t{0});
    CHECK_TRUE(identical(hm, before));
}

TEST(TerrainCarve, disabled_config_carves_nothing) {
    Heightmap hm = make_heightmap(41, 41, 1.0f, glm::vec2{0.0f, -20.0f}, gentle_slope);
    const Heightmap before = hm;

    CarveConfig config;
    config.enabled = false;
    CarveInput input = make_input({straight_ribbon(-10.0, 50.0, 0.0, 8.0f, 5.0f)}, config);

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_EQ(stats.cells_modified, size_t{0});
    CHECK_TRUE(identical(hm, before));
}

TEST(TerrainCarve, ribbon_outside_the_chunk_changes_nothing) {
    Heightmap hm = make_heightmap(41, 41, 1.0f, glm::vec2{0.0f, -20.0f}, gentle_slope);
    const Heightmap before = hm;

    CarveConfig config;
    config.falloff_metres = 10.0f;

    // Far outside the chunk in both axes, and further than the widest band the
    // slope limit can open up.
    CarveInput input = make_input({straight_ribbon(4000.0, 4200.0, 5000.0, 8.0f, 5.0f)},
                                  config);
    CHECK_TRUE(input.has_index());

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_EQ(stats.cells_modified, size_t{0});
    CHECK_TRUE(identical(hm, before));
    CHECK_TRUE(all_finite(hm));
}

TEST(TerrainCarve, degenerate_ribbons_change_nothing) {
    Heightmap hm = make_heightmap(41, 41, 1.0f, glm::vec2{0.0f, -20.0f}, gentle_slope);
    const Heightmap before = hm;

    // A one-point centerline has no direction and no segment to project onto, so
    // there is no corridor to carve. It sits in the middle of the chunk here on
    // purpose: putting it outside would prove only that the index works.
    CarveRibbon single;
    single.centerline = {{20.0, 0.0}};
    single.centerline_heights = {30.0f};
    single.half_width = 6.0f;
    single.outline.clear();
    single.outline_is_simple = false;

    // No centerline at all.
    CarveRibbon empty;
    empty.centerline.clear();
    empty.centerline_heights.clear();
    empty.half_width = 6.0f;

    // Heights not parallel to the centerline: a caller error the carve must
    // refuse rather than read past.
    CarveRibbon mismatched;
    mismatched.centerline = {{0.0, 0.0}, {40.0, 0.0}, {40.0, 20.0}};
    mismatched.centerline_heights = {5.0f};
    mismatched.half_width = 6.0f;

    CarveInput input = make_input({single, empty, mismatched}, CarveConfig{});
    CHECK_TRUE(input.has_index());
    CHECK_EQ(input.item_count(), size_t{3});

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_EQ(stats.cells_modified, size_t{0});
    CHECK_TRUE(identical(hm, before));
    CHECK_TRUE(all_finite(hm));
}

// ============================================================================
// Coordinate frame
// ============================================================================
//
// A Heightmap's second axis IS the 2D local y of the carve payload. Both
// pipelines negate independently on their way to render space --
// TerrainMeshBuilder places cell (ix, iz) at vec3(world_x, h, -world_z), the
// corridor extruder places local (x, y) at vec3(x, h, -y) -- so applying a third
// negation inside the carve mirrors the whole network about y = 0. That error is
// exactly zero at the origin and grows with |y|, which is why every other test in
// this file, centred on y = 0, is blind to it. These two are not.

TEST(TerrainCarve, carve_lands_on_the_y_side_it_was_asked_for) {
    // Rows span 2D y in [-300, 300]. The ribbon is at y = +200 only.
    Heightmap hm = make_heightmap(41, 121, 5.0f, glm::vec2{0.0f, -300.0f}, flat_ground);

    CarveConfig config;
    config.falloff_metres = 10.0f;

    const float road_height = 40.0f;
    CarveInput input = make_input({straight_ribbon(-50.0, 250.0, 200.0, road_height, 6.0f)},
                                  config);
    CHECK_TRUE(input.has_index());

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_TRUE(stats.cells_modified > size_t{0});
    CHECK_TRUE(all_finite(hm));

    // Under the road: carved to the road surface.
    CHECK_NEAR(static_cast<double>(at_local(hm, 100.0, 200.0)), road_height, 1e-3);

    // At the mirror image, where no road exists: untouched. A negated axis puts
    // the whole trench here instead.
    CHECK_NEAR(static_cast<double>(at_local(hm, 100.0, -200.0)), 0.0, 1e-6);

    // Stronger than two probes: EVERY modified row must be on the positive side.
    size_t rows_negative = 0;
    size_t rows_positive = 0;
    for (int iz = 0; iz < hm.height; ++iz) {
        const double y2d = static_cast<double>(hm.origin.y)
                         + static_cast<double>(iz) * static_cast<double>(hm.cell_size_z);
        bool row_moved = false;
        for (int ix = 0; ix < hm.width; ++ix) {
            if (hm.data[cell_index(hm, ix, iz)] != 0.0f) row_moved = true;
        }
        if (!row_moved) continue;
        if (y2d < 0.0) ++rows_negative;
        else ++rows_positive;
    }
    CHECK_EQ(rows_negative, size_t{0});
    CHECK_TRUE(rows_positive > size_t{0});
}

// ============================================================================
// Mitred bends
// ============================================================================

TEST(TerrainCarve, mitred_bend_carves_under_its_outer_corner) {
    // A single-vertex 120-degree deflection: two long legs meeting at the origin,
    // the second turned by 120 degrees. The extruder offsets that joint by
    // half_width / cos(theta / 2) = 2 * half_width, so the corridor's outer corner
    // is 12 m from the centerline while half_width is 6 m. A band measured with
    // half_width alone leaves that corner hanging over terrain it never flattened.
    //
    // The road sits 5 m above the natural surface on purpose: at that height
    // CarveConfig::max_embankment_slope does not widen the band, so the only
    // thing that can move the footprint edge is the miter under test.
    const double kHalfWidth = 6.0;
    const double kMiter = 2.0;
    const float kRoad = 5.0f;

    // Interior angle 60 degrees about +X: the outgoing leg heads up and left, so
    // the bisector -- and the outer corner -- points along -X.
    CarveRibbon r;
    r.centerline = {{200.0, 0.0}, {0.0, 0.0}, {-100.0, 173.205080756887}};
    r.centerline_heights = {kRoad, kRoad, kRoad};
    r.centerline_miter = {1.0f, static_cast<float>(kMiter), 1.0f};
    r.half_width = static_cast<float>(kHalfWidth);
    r.outline_is_simple = false;  // the carve tests the band, not the ring

    CarveConfig config;
    config.falloff_metres = 10.0f;

    Heightmap hm = make_heightmap(61, 61, 1.0f, glm::vec2{-30.0f, -30.0f}, flat_ground);
    CarveInput input = make_input({r}, config);
    CHECK_TRUE(input.has_index());

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_TRUE(stats.cells_modified > size_t{0});
    CHECK_TRUE(all_finite(hm));

    // Just inside the outer mitre vertex, 11.5 m from the joint along -X. It must
    // sit AT the road surface, not part-way down an embankment.
    CHECK_NEAR(static_cast<double>(at_local(hm, -11.5, 0.0)), kRoad, 1e-3);

    // Well inside the mitre, past where an unmitred band would already be
    // blending.
    CHECK_NEAR(static_cast<double>(at_local(hm, -9.0, 0.0)), kRoad, 1e-3);

    // Past the mitre the band must still close: the carve widens the footprint,
    // it does not remove the falloff.
    CHECK_TRUE(at_local(hm, -25.0, 0.0) < kRoad);

    // The same ribbon carrying NO miter information keeps the old, narrower band.
    // This is what proves the assertions above test the miter and not some
    // unrelated widening.
    CarveRibbon plain = r;
    plain.centerline_miter.clear();

    Heightmap plain_hm = make_heightmap(61, 61, 1.0f, glm::vec2{-30.0f, -30.0f}, flat_ground);
    CarveInput plain_input = make_input({plain}, config);
    (void)carve_terrain(plain_hm, plain_input);

    CHECK_TRUE(at_local(plain_hm, -9.0, 0.0) < kRoad - 0.1f);
    CHECK_TRUE(at_local(plain_hm, -11.5, 0.0) < at_local(hm, -11.5, 0.0));
}

// ============================================================================
// The terrain sampler's frame
// ============================================================================
//
// The road elevation solve reads the terrain through a HeightSampler taking 2D
// LOCAL metres, and the editor implements that over
// TerrainGenerator::sample_surface(). Whether the adapter must negate its second
// argument is decided entirely by this question: is sample_surface()'s second
// argument the heightmap's second axis, or is it render-space Z?
//
// It is the heightmap's second axis, and this pins it. Combined with
// carve_lands_on_the_y_side_it_was_asked_for above -- which pins the carve to the
// same axis -- the adapter must pass local y straight through, with no negation
// anywhere in the chain.

TEST(TerrainCarve, sample_surface_matches_the_chunk_it_would_generate) {
    TerrainConfig cfg;
    cfg.type = TerrainType::Rolling;
    cfg.seed = 12345;
    cfg.resolution_x = 17;
    cfg.resolution_z = 17;

    TerrainGenerator generator(cfg.seed);

    // An origin far off the y axis in ONE direction: on a symmetric fixture this
    // whole question is invisible, which is how the mirror survived review.
    const glm::vec2 origin{100.0f, 400.0f};
    const Heightmap chunk = generator.generate_chunk(cfg, origin, 160.0f, 160.0f);

    CHECK_EQ(static_cast<size_t>(chunk.width), size_t{17});
    CHECK_EQ(static_cast<size_t>(chunk.height), size_t{17});

    size_t mismatched = 0;
    for (int iz = 0; iz < chunk.height; iz += 4) {
        for (int ix = 0; ix < chunk.width; ix += 4) {
            const float wx = chunk.origin.x + static_cast<float>(ix) * chunk.cell_size_x;
            const float wz = chunk.origin.y + static_cast<float>(iz) * chunk.cell_size_z;

            // Second argument passed through unchanged, exactly as the road
            // sampler must pass local y.
            if (std::fabs(generator.sample_surface(cfg, wx, wz) - chunk.at(ix, iz)) > 1e-4f) {
                ++mismatched;
            }
        }
    }
    CHECK_EQ(mismatched, size_t{0});

    // And the surface really is asymmetric in that axis, so the check above is
    // not passing by coincidence.
    const float north = generator.sample_surface(cfg, 100.0f, 400.0f);
    const float south = generator.sample_surface(cfg, 100.0f, -400.0f);
    CHECK_TRUE(std::fabs(north - south) > 1.0f);
}

// ============================================================================
// Junction polygons: the P4 carve
// ============================================================================
// P3 carved a junction as a flat disc covering the arm mouths. P4 fills
// CarveDisc::outline with the real fillet-and-curb-ring boundary, and these
// tests are what keep the ring from silently degrading back into the circle it
// replaced: a square ring and its own bounding circle disagree everywhere except
// at four corners, so a fallback shows up as a value, not as a shrug.

TEST(TerrainCarve, junction_polygon_flattens_its_ring_and_blends_outward) {
    // World x and 2D y both in [-40, 40], one sample per metre.
    Heightmap hm = make_heightmap(81, 81, 1.0f, glm::vec2{-40.0f, -40.0f}, flat_ground);
    const Heightmap before = hm;

    CarveConfig config;
    config.falloff_metres = 10.0f;

    const double half = 8.0;
    const float junction_height = 5.0f;

    CarveInput input = make_input(
        {},
        {square_junction({0.0, 0.0}, half, junction_height,
                         static_cast<float>(square_bounding_radius(half)))},
        config);

    CHECK_TRUE(input.has_index());
    CHECK_EQ(input.item_count(), size_t{1});

    const CarveStats stats = carve_terrain(hm, input);

    CHECK_TRUE(stats.cells_modified > size_t{0});
    CHECK_TRUE(all_finite(hm));

    // Inside the ring the terrain IS the junction surface, exactly. A junction is
    // planar, so there is no interpolation to get wrong here.
    size_t inside_wrong = 0;
    for (int y = -7; y <= 7; ++y) {
        for (int x = -7; x <= 7; ++x) {
            if (std::fabs(static_cast<double>(at_local(hm, x, y)) - junction_height) > 1e-4) {
                ++inside_wrong;
            }
        }
    }
    CHECK_EQ(inside_wrong, size_t{0});

    // The discriminator. (11, 0) is 3 m OUTSIDE the ring, so it must be part way
    // down the embankment -- but it is inside the bounding circle of radius
    // 11.31, so a carve that had fallen back to the disc would have flattened it
    // to the junction height.
    const double edge_probe = static_cast<double>(at_local(hm, 11, 0));
    CHECK_TRUE(edge_probe < static_cast<double>(junction_height) - 0.1);
    CHECK_TRUE(edge_probe > 0.0);

    // Corners read as corners. A point diagonally off a corner is further from
    // the ring than a point the same distance off an edge, so it is carved less.
    // On a disc the two would be indistinguishable.
    CHECK_TRUE(static_cast<double>(at_local(hm, 12, 12))
               < static_cast<double>(at_local(hm, 12, 0)) - 0.1);

    // Beyond the ring plus the band nothing may have moved. The band here is the
    // configured falloff: closing 5 m over 10 m needs a 0.5 gradient, inside the
    // 0.6 limit, so no widening applies.
    const double reach = half + static_cast<double>(config.falloff_metres);
    size_t outside_touched = 0;
    for (int iz = 0; iz < hm.height; ++iz) {
        const double y2d = static_cast<double>(hm.origin.y)
                         + static_cast<double>(iz) * static_cast<double>(hm.cell_size_z);
        for (int ix = 0; ix < hm.width; ++ix) {
            const double x2d = static_cast<double>(hm.origin.x)
                             + static_cast<double>(ix) * static_cast<double>(hm.cell_size_x);
            // Distance from the square, Chebyshev-style: outside the reach box on
            // either axis is outside the reach of the footprint.
            if (std::fabs(x2d) <= reach + 0.5 && std::fabs(y2d) <= reach + 0.5) continue;
            if (!(hm.data[cell_index(hm, ix, iz)] == before.data[cell_index(hm, ix, iz)])) {
                ++outside_touched;
            }
        }
    }
    CHECK_EQ(outside_touched, size_t{0});

    // Across the band the height falls monotonically from the junction surface to
    // the natural one, and lands on it rather than part way down.
    std::vector<double> band;
    for (int y = 8; y <= 19; ++y) band.push_back(static_cast<double>(at_local(hm, 0, y)));

    size_t non_monotone = 0;
    for (size_t i = 0; i + 1 < band.size(); ++i) {
        if (band[i + 1] > band[i] + 1e-5) ++non_monotone;
    }
    CHECK_EQ(non_monotone, size_t{0});
    CHECK_NEAR(band.back(), 0.0, 1e-3);
}

TEST(TerrainCarve, a_ring_that_cannot_be_tested_falls_back_to_the_disc) {
    CarveConfig config;
    config.falloff_metres = 10.0f;

    const double half = 8.0;
    const float junction_height = 5.0f;
    const float radius = static_cast<float>(square_bounding_radius(half));

    // A ring the carve must refuse: a winding test against a self-intersecting
    // outline has no meaningful answer and punches holes in the terrain, which is
    // exactly what outline_is_simple exists to prevent.
    CarveDisc folded = square_junction({0.0, 0.0}, half, junction_height, radius);
    folded.outline_is_simple = false;

    // A ring with too few vertices is not a ring at all.
    CarveDisc sliver = square_junction({0.0, 0.0}, half, junction_height, radius);
    sliver.outline.resize(2);

    // A ring carrying a non-finite vertex. One bad vertex poisons both the inside
    // test and the bounds the disc is binned by, so the whole ring is dropped.
    CarveDisc poisoned = square_junction({0.0, 0.0}, half, junction_height, radius);
    poisoned.outline[2].x = std::numeric_limits<double>::quiet_NaN();

    // The reference: the same junction with no polygon at all.
    CarveDisc plain;
    plain.center = {0.0, 0.0};
    plain.height = junction_height;
    plain.radius = radius;

    Heightmap reference = make_heightmap(81, 81, 1.0f, glm::vec2{-40.0f, -40.0f}, flat_ground);
    CarveInput plain_input = make_input({}, {plain}, config);
    const CarveStats reference_stats = carve_terrain(reference, plain_input);
    CHECK_TRUE(reference_stats.cells_modified > size_t{0});

    // A disc carve flattens the whole circle, so the point 11 m out on the axis --
    // the one the polygon carve leaves part way down the bank -- is at the
    // junction height here. That is what makes the comparison below meaningful
    // rather than a tautology.
    CHECK_NEAR(static_cast<double>(at_local(reference, 11, 0)),
               static_cast<double>(junction_height), 1e-4);

    size_t mismatched = 0;
    for (const CarveDisc& unusable : {folded, sliver, poisoned}) {
        Heightmap hm = make_heightmap(81, 81, 1.0f, glm::vec2{-40.0f, -40.0f}, flat_ground);
        CarveInput input = make_input({}, {unusable}, config);
        const CarveStats stats = carve_terrain(hm, input);

        CHECK_TRUE(all_finite(hm));
        CHECK_EQ(stats.cells_modified, reference_stats.cells_modified);
        if (!identical(hm, reference)) ++mismatched;
    }
    CHECK_EQ(mismatched, size_t{0});
}

TEST(TerrainCarve, a_ring_wider_than_its_stated_radius_is_still_fully_binned) {
    // The trap this guards is the one the miter widening already sprang once: an
    // item binned by one reach and carved with a larger one influences cells it
    // was never binned into, and the result depends on the grid rather than on
    // the geometry. Here the disc claims a radius of 1 m while carrying an 8 m
    // ring, and the index has to believe the ring.
    Heightmap hm = make_heightmap(81, 81, 1.0f, glm::vec2{-40.0f, -40.0f}, flat_ground);

    CarveConfig config;
    config.falloff_metres = 10.0f;

    const double half = 8.0;
    const float junction_height = 5.0f;

    CarveInput input = make_input({}, {square_junction({0.0, 0.0}, half, junction_height, 1.0f)},
                                  config);
    const CarveStats stats = carve_terrain(hm, input);

    CHECK_TRUE(stats.cells_modified > size_t{0});
    CHECK_TRUE(all_finite(hm));

    // Inside the ring, well outside the claimed radius.
    CHECK_NEAR(static_cast<double>(at_local(hm, 7, 0)),
               static_cast<double>(junction_height), 1e-4);
    CHECK_NEAR(static_cast<double>(at_local(hm, 0, 7)),
               static_cast<double>(junction_height), 1e-4);

    // Out in the band, 17 m from the centre: sixteen metres beyond the claimed
    // radius plus a nominal falloff, and only reachable if the ring sized the box.
    CHECK_TRUE(static_cast<double>(at_local(hm, 0, 17)) > 1e-4);
    CHECK_TRUE(static_cast<double>(at_local(hm, 0, 17))
               < static_cast<double>(junction_height));
}

TEST(TerrainCarve, an_arm_hands_over_to_its_junction_without_a_gap_or_a_step) {
    // The composition case. A trimmed arm stops at the junction's edge, and the
    // junction takes over from there. Both are carved to the same solved node
    // height, and the winner at any cell is whichever footprint it is nearer to in
    // normalised distance -- so the hand-over must leave no uncarved cell between
    // the two and no step at either kerb.
    Heightmap hm = make_heightmap(121, 61, 1.0f, glm::vec2{-80.0f, -30.0f}, flat_ground);

    CarveConfig config;
    config.falloff_metres = 10.0f;

    const double half = 8.0;
    const float surface = 5.0f;
    const float half_width = 5.0f;

    // The arm runs in from the west and is trimmed exactly at the ring's edge,
    // which is what JunctionBuilder's trim solve produces.
    CarveRibbon arm = straight_ribbon(-70.0, -half, 0.0, surface, half_width);
    CarveDisc junction = square_junction({0.0, 0.0}, half, surface,
                                         static_cast<float>(square_bounding_radius(half)));

    CarveInput input = make_input({arm}, {junction}, config);
    CHECK_EQ(input.item_count(), size_t{2});

    const CarveStats stats = carve_terrain(hm, input);
    CHECK_TRUE(stats.cells_modified > size_t{0});
    CHECK_TRUE(all_finite(hm));

    // No gap and no step: every cell along the centre of the arm, from well
    // inside the ribbon through the trim station and on into the junction, sits
    // exactly on the shared surface.
    size_t wrong = 0;
    for (int x = -60; x <= 7; ++x) {
        if (std::fabs(static_cast<double>(at_local(hm, x, 0))
                      - static_cast<double>(surface)) > 1e-4) {
            ++wrong;
        }
    }
    CHECK_EQ(wrong, size_t{0});

    // The same across the full width of the arm at the trim station, where the
    // two footprints abut.
    size_t seam_wrong = 0;
    for (int y = -4; y <= 4; ++y) {
        if (std::fabs(static_cast<double>(at_local(hm, -8, y))
                      - static_cast<double>(surface)) > 1e-4) {
            ++seam_wrong;
        }
    }
    CHECK_EQ(seam_wrong, size_t{0});

    // The junction is WIDER than the arm, so just outside the arm's kerb and
    // alongside the junction the terrain is still on its way down, never below
    // the natural surface and never above the road.
    size_t out_of_range = 0;
    for (int x = -20; x <= 7; ++x) {
        for (int y = 5; y <= 20; ++y) {
            const double h = static_cast<double>(at_local(hm, x, y));
            if (h < -1e-4 || h > static_cast<double>(surface) + 1e-4) ++out_of_range;
        }
    }
    CHECK_EQ(out_of_range, size_t{0});

    // Where the arm alone governs, the result is the ribbon carve unchanged: the
    // junction is 40 m away and out of reach entirely.
    Heightmap arm_only = make_heightmap(121, 61, 1.0f, glm::vec2{-80.0f, -30.0f}, flat_ground);
    CarveInput arm_input = make_input({arm}, config);
    const CarveStats arm_only_stats = carve_terrain(arm_only, arm_input);
    CHECK_TRUE(arm_only_stats.cells_modified > size_t{0});

    size_t drifted = 0;
    for (int x = -60; x <= -40; ++x) {
        for (int y = -20; y <= 20; ++y) {
            if (!(at_local(hm, x, y) == at_local(arm_only, x, y))) ++drifted;
        }
    }
    CHECK_EQ(drifted, size_t{0});
}

// ============================================================================
// Tunnel portal mouths
// ============================================================================
//
// A portal is the only carve primitive that is a CEILING rather than a target.
// Every test below is really one of two questions: does it cut the hillside back
// off the arch, and does it leave alone every square metre where the ground was
// never in the way.

TEST(TerrainCarve, portal_mouth_notches_the_hillside_back_to_the_crown) {
    // The mouth runs from the opening at x = 0 ten metres into a hill rising 2:1,
    // so the ground inside it stands as much as 20 m above a crown at 5 m. Every
    // one of those metres has to come off, or the tunnel opens into solid ground.
    Heightmap hm = make_heightmap(121, 121, 1.0f, glm::vec2{-60.0f, -60.0f}, hillside);

    CarveConfig config;
    config.falloff_metres = 10.0f;

    const float crown = 5.0f;
    CarveInput input =
        make_input({}, {}, {portal_mouth({0.0, 0.0}, {1.0, 0.0}, 6.0, 10.0, crown)}, config);

    const CarveStats stats = carve_terrain(hm, input);

    CHECK_TRUE(stats.cells_modified > size_t{0});
    CHECK_TRUE(all_finite(hm));

    // Inside the mouth, where the hill stood above the arch: clamped to the crown
    // exactly, never to the road surface below it.
    CHECK_NEAR(static_cast<double>(at_local(hm, 5, 0)), static_cast<double>(crown), 1e-4);
    CHECK_NEAR(static_cast<double>(at_local(hm, 9, 0)), static_cast<double>(crown), 1e-4);
    CHECK_NEAR(static_cast<double>(at_local(hm, 5, 5)), static_cast<double>(crown), 1e-4);

    // Inside the mouth, where the hill was ALREADY below the arch. A clamp leaves
    // this alone; a set would dig it out to the crown and leave a step in front of
    // the opening.
    CHECK_NEAR(static_cast<double>(at_local(hm, 2, 0)), 4.0, 1e-4);

    // Past the far end of the mouth the notch does not stop dead: it slopes back
    // up to the hill at the embankment limit, so this sits above the crown and
    // below the untouched hillside.
    const double past = static_cast<double>(at_local(hm, 11, 0));
    CHECK_TRUE(past > static_cast<double>(crown));
    CHECK_TRUE(past < 22.0);

    // Well off to the side the hill is untouched at its natural 10 m.
    CHECK_NEAR(static_cast<double>(at_local(hm, 5, 25)), 10.0, 1e-4);
}

TEST(TerrainCarve, a_portal_on_ground_below_its_arch_changes_nothing) {
    // The inertness property, and the reason the mouth is a min() and not a set().
    // A tunnel that surfaces onto level ground still emits a portal footprint at
    // the point it crossed under; carving that as a target would trench the
    // approach in front of the headwall.
    Heightmap hm = make_heightmap(81, 81, 1.0f, glm::vec2{-40.0f, -40.0f}, flat_ground);
    const Heightmap before = hm;

    CarveConfig config;
    config.falloff_metres = 10.0f;

    CarveInput input =
        make_input({}, {}, {portal_mouth({0.0, 0.0}, {1.0, 0.0}, 6.0, 10.0, 5.0f)}, config);

    const CarveStats stats = carve_terrain(hm, input);

    CHECK_EQ(stats.cells_modified, size_t{0});
    CHECK_TRUE(identical(hm, before));
}

TEST(TerrainCarve, a_portal_never_cuts_below_its_own_crown) {
    // The invariant that separates a ceiling from a second carve: the mouth may
    // lower the ground to the crown and no further. Asserted over every cell of
    // the chunk rather than at chosen points, because the failure it guards --
    // blending the ceiling back towards the NATURAL surface instead of the carved
    // one -- gouges below the crown out in the band, nowhere near the opening.
    Heightmap hm = make_heightmap(121, 121, 1.0f, glm::vec2{-60.0f, -60.0f}, hillside);
    const Heightmap before = hm;

    CarveConfig config;
    config.falloff_metres = 10.0f;

    const float crown = 5.0f;
    CarveInput input =
        make_input({}, {}, {portal_mouth({0.0, 0.0}, {1.0, 0.0}, 6.0, 10.0, crown)}, config);

    (void)carve_terrain(hm, input);

    bool floor_held = true;
    bool never_raised = true;
    for (size_t i = 0; i < hm.data.size(); ++i) {
        // Either the cell was left where it was, or it was lowered to somewhere at
        // or above the crown.
        if (hm.data[i] < std::min(before.data[i], crown) - 1e-4f) floor_held = false;
        if (hm.data[i] > before.data[i] + 1e-4f) never_raised = false;
    }
    CHECK_TRUE(floor_held);
    CHECK_TRUE(never_raised);
}

TEST(TerrainCarve, overlapping_portal_mouths_do_not_depend_on_index_order) {
    // Two mouths facing into the same hill, close enough that their bands overlap,
    // and one deeper and lower than the other. Combining them with min() makes the
    // answer order-free; combining them by folding each into a running height
    // would not, because the second mouth's band width is derived from the height
    // the first one left.
    CarveConfig config;
    config.falloff_metres = 10.0f;

    const CarvePortal a = portal_mouth({0.0, -4.0}, {1.0, 0.0}, 6.0, 10.0, 5.0f);
    const CarvePortal b = portal_mouth({0.0, 4.0}, {1.0, 0.0}, 6.0, 16.0, 3.0f);

    Heightmap forward = make_heightmap(121, 121, 1.0f, glm::vec2{-60.0f, -60.0f}, hillside);
    Heightmap reversed = forward;

    CarveInput input_ab = make_input({}, {}, {a, b}, config);
    CarveInput input_ba = make_input({}, {}, {b, a}, config);

    (void)carve_terrain(forward, input_ab);
    (void)carve_terrain(reversed, input_ba);

    CHECK_TRUE(identical(forward, reversed));
    CHECK_TRUE(all_finite(forward));

    // And the deeper, lower mouth wins where the two overlap.
    CHECK_NEAR(static_cast<double>(at_local(forward, 12, 4)), 3.0, 1e-4);
}

TEST(TerrainCarve, a_portal_mouth_clamps_what_the_ribbon_pass_left) {
    // Ordering. The mouth is a ceiling on the height the ribbon and disc pass
    // produced, not on the natural surface, so it has to run after that pass and
    // read its result. Here a surface road crosses the hill directly above the
    // first ten metres of tunnel and FILLS the ground there to 14 m, four metres
    // above the natural hillside and nine above the crown. A mouth applied to the
    // natural surface would find 10 m, clamp that, and leave the fill standing
    // across the opening.
    CarveConfig config;
    config.falloff_metres = 10.0f;

    const float crown = 5.0f;

    // A ribbon running along 2D y, across the portal axis, embanked at 14 m.
    CarveRibbon crossing;
    crossing.centerline = {{5.0, -40.0}, {5.0, 40.0}};
    crossing.centerline_heights = {14.0f, 14.0f};
    crossing.half_width = 5.0f;
    crossing.outline = {{0.0, -40.0}, {10.0, -40.0}, {10.0, 40.0}, {0.0, 40.0}};
    crossing.outline_is_simple = true;

    // The same fill twice, with and without the mouth, so the difference between
    // the two heightmaps is the mouth and nothing else.
    Heightmap without = make_heightmap(121, 121, 1.0f, glm::vec2{-60.0f, -60.0f}, hillside);
    Heightmap with = without;

    CarveInput fill_only = make_input({crossing}, config);
    const CarveStats fill_stats = carve_terrain(without, fill_only);
    CHECK_TRUE(fill_stats.cells_modified > size_t{0});

    CarveInput fill_and_mouth =
        make_input({crossing}, {}, {portal_mouth({0.0, 0.0}, {1.0, 0.0}, 6.0, 10.0, crown)},
                   config);
    const CarveStats both_stats = carve_terrain(with, fill_and_mouth);
    CHECK_TRUE(both_stats.cells_modified > fill_stats.cells_modified);

    // The embankment really does close over the opening when nothing stops it.
    CHECK_NEAR(static_cast<double>(at_local(without, 5, 0)), 14.0, 1e-4);

    // And the mouth takes it back to the crown -- past the natural 10 m the hill
    // stood at, which is the number a mouth reading the wrong surface would stop
    // at.
    CHECK_NEAR(static_cast<double>(at_local(with, 5, 0)), static_cast<double>(crown), 1e-4);

    // Well clear of the mouth in 2D y, the same embankment is untouched: a mouth
    // clamps its own footprint and its band, not every cell of every road that
    // happens to pass through it.
    CHECK_NEAR(static_cast<double>(at_local(with, 5, 25)), 14.0, 1e-4);
    CHECK_TRUE(all_finite(with));
}

TEST(TerrainCarve, degenerate_portals_change_nothing) {
    // Each of these describes no rectangle, and a rectangle guessed from one would
    // notch terrain where no headwall stands. Rejected whole, never repaired.
    Heightmap hm = make_heightmap(81, 81, 1.0f, glm::vec2{-40.0f, -40.0f}, hillside);
    const Heightmap before = hm;

    CarveConfig config;
    config.falloff_metres = 10.0f;

    const double nan = std::numeric_limits<double>::quiet_NaN();

    std::vector<CarvePortal> broken;
    broken.push_back(portal_mouth({0.0, 0.0}, {1.0, 0.0}, 6.0, 0.0, 1.0f));    // no depth
    broken.push_back(portal_mouth({4.0, 0.0}, {1.0, 0.0}, 0.0, 10.0, 1.0f));   // no width
    broken.push_back(portal_mouth({8.0, 0.0}, {0.0, 0.0}, 6.0, 10.0, 1.0f));   // no axis
    broken.push_back(portal_mouth({12.0, 0.0}, {nan, 0.0}, 6.0, 10.0, 1.0f));  // axis is NaN
    broken.push_back(portal_mouth({16.0, 0.0}, {1.0, 0.0}, 6.0, 10.0,
                                  std::numeric_limits<float>::quiet_NaN()));   // crown is NaN

    CarveInput input = make_input({}, {}, std::move(broken), config);
    const CarveStats stats = carve_terrain(hm, input);

    CHECK_EQ(stats.cells_modified, size_t{0});
    CHECK_TRUE(identical(hm, before));
    CHECK_TRUE(all_finite(hm));
}

TEST(TerrainCarve, a_portal_mouth_reaches_no_further_than_it_was_binned) {
    // The trap this file has already sprung twice, in its third costume. The mouth
    // is carved with a slope-widened band, so it can reach far past
    // falloff_metres; the index has to have binned it by that same widened reach.
    // A mouth binned by its rectangle alone would leave a hard edge at whichever
    // grid cell boundary it happened to stop at, so the notch is walked outward
    // and required to fall monotonically back to the hill with no cliff in it.
    Heightmap hm = make_heightmap(161, 161, 1.0f, glm::vec2{-80.0f, -80.0f}, hillside);

    CarveConfig config;
    config.falloff_metres = 10.0f;

    CarveInput input =
        make_input({}, {}, {portal_mouth({0.0, 0.0}, {1.0, 0.0}, 6.0, 10.0, 5.0f)}, config);
    (void)carve_terrain(hm, input);

    CHECK_TRUE(all_finite(hm));

    // Along the axis, from the far end of the mouth out to well past any band the
    // slope limit can open. The carved profile must rise back to the natural hill
    // without a step: a step is exactly what an item carved past its bins leaves.
    double previous = static_cast<double>(at_local(hm, 10, 0));
    bool monotone = true;
    bool no_cliff = true;
    for (int x = 11; x <= 70; ++x) {
        const double here = static_cast<double>(at_local(hm, x, 0));
        if (here < previous - 1e-4) monotone = false;

        // The notch legitimately climbs faster than the 2 m per cell the hill
        // does: the blend weight is opening at the same time as the natural
        // surface rises, and on this hill the two together peak just under 4 m
        // per cell. Six is the separator, because the failure being guarded is
        // not a steep profile but a hand-over: an item that stops influencing
        // cells at an index cell boundary drops TENS of metres in one cell, and
        // index cells here are at least sixteen metres wide.
        if (here - previous > 6.0) no_cliff = false;
        previous = here;
    }
    CHECK_TRUE(monotone);
    CHECK_TRUE(no_cliff);

    // And it really has rejoined the hill by the end of that walk.
    CHECK_NEAR(static_cast<double>(at_local(hm, 70, 0)), 140.0, 1e-4);
}
