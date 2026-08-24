/**
 * @file test_road_terrain_dump.cpp
 * @brief P3 visual dump and stats: roads on rolling terrain, and the terrain they carve
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * RoadDump writes the FLAT P2 network. This suite writes the P3 one, and it
 * exists because the numeric assertions in RoadElevation and TerrainCarve cannot
 * catch the one defect that matters most here: a road that is correct against
 * its own solved profile and a terrain that is correct against its own carve,
 * which nonetheless do not meet. A road floating a metre above its cutting, or
 * sunk into it, satisfies every per-module test in the tree. Only loading the two
 * meshes together shows it.
 *
 * Three outputs, all under build/road_dump:
 *
 * 1. `<fixture>_elevated.obj` -- every fixture built through the terrain path
 *    with a synthetic rolling surface, so the solve actually has relief to fight.
 * 2. `<fixture>_terrain.obj` -- the CARVED heightmap of the same surface, as a
 *    mesh, for the fixtures where road-vs-terrain agreement is worth looking at.
 *    Load it alongside the matching `_elevated.obj`: they are in the same world
 *    space and the same metres, so the road must sit just above its cutting
 *    along its whole length.
 * 3. A stats table per fixture -- solver iterations, max grade before and after,
 *    bridges, tunnels, carved cells, max carve delta.
 *
 * ### The surface is deliberately steeper than any road may climb
 *
 * rolling_surface() reaches roughly 45% gradient. Every class limit in the table
 * is between 4% and 15%, so the grade limiter binds on essentially every edge.
 * That is the point: a gentle surface the roads could simply follow would exercise
 * the sampler and nothing else, and would pass just as well with the solver
 * deleted.
 *
 * ### The convergence check is the load-bearing assertion
 *
 * A post-solve gradient above the edge's class limit does not mean the road is a
 * bit steep. It means the relaxation did not converge, and every number in the
 * table below it is describing an unsolved profile. It is asserted per edge, and
 * named in the failure, rather than being averaged into a column.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadTerrainDump
 * @endcode
 */

#include "framework.hpp"
#include "obj_dump.hpp"

#include "osm/parser.hpp"
#include "osm/road/centerline.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_network_builder.hpp"
#include "osm/types.hpp"
#include "procgen/terrain_carve.hpp"
#include "procgen/terrain_generator.hpp"
#include "procgen/terrain_mesh_builder.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#ifndef STRATUM_TEST_DATA_DIR
#error "STRATUM_TEST_DATA_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

#ifndef STRATUM_TEST_DUMP_DIR
#error "STRATUM_TEST_DUMP_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

namespace {

using stratum::Mesh;
using stratum::osm::ParsedOSMData;
using stratum::osm::road::Centerline;
using stratum::osm::road::EdgeElevation;
using stratum::osm::road::EdgeId;
using stratum::osm::road::ElevationConfig;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::HeightSampler;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::max_grade_for;
using stratum::procgen::CarveInput;
using stratum::procgen::CarveStats;
using stratum::procgen::Heightmap;
using stratum::procgen::TerrainMeshBuilder;
using stratum::procgen::TerrainMeshConfig;
using stratum::procgen::carve_terrain;
using stratum::test::ObjDumpStats;
using stratum::test::write_obj;

/// Every fixture in tests/data, in README table order
constexpr const char* kAllFixtures[] = {
    "four_way.osm", "t_junction.osm", "cul_de_sac.osm", "roundabout.osm",
    "motorway_link.osm", "rural_track.osm", "bridge_over.osm",
    "bridge_abutment.osm", "duplicate_node.osm",
};

/**
 * @brief Fixtures whose carved terrain is also dumped as a mesh
 *
 * Not all nine: a terrain mesh is one vertex per heightmap cell, so dumping every
 * fixture's would write far more than a reader will ever open. These two are the
 * ones where road-vs-terrain agreement says something.
 *
 * - four_way: a junction disc carved against the four arms feeding it. If the
 *   disc and the arms disagree, the step is at the kerb line and visible.
 * - bridge_over: the deck is suppressed and must NOT carve, so the terrain under
 *   it stays natural while the approaches trench. That is the open behavioural
 *   question in the plan, and this is the file that answers it by eye.
 */
constexpr const char* kTerrainFixtures[] = {"four_way.osm", "bridge_over.osm"};

/// Metres of terrain kept around the network bounds in a dumped heightmap
constexpr double kTerrainMargin = 40.0;

/// Heightmap cell size for the dumped terrain, metres
constexpr float kTerrainCell = 1.0f;

// ============================================================================
// The synthetic surface
// ============================================================================

/**
 * @brief Rolling terrain in 2D LOCAL metres, ~45% peak gradient
 *
 * Three superposed sinusoids so the surface has no axis the road can run along to
 * escape the slope, and no periodicity short enough for the grade limiter to
 * average away. Peak gradient is roughly 0.45, which is three times the loosest
 * class limit in the table and eleven times the motorway one, so the solver has
 * to depart from the ground on every fixture rather than tracking it.
 *
 * Pure and re-entrant, as HeightSampler requires.
 */
float rolling_surface(double x, double y) {
    return static_cast<float>(50.0
                              + 12.0 * std::sin(x / 60.0)
                              + 9.0 * std::cos(y / 45.0)
                              + 3.0 * std::sin((x + y) / 22.0));
}

/**
 * @brief rolling_surface() sampled at a Heightmap cell
 *
 * A Heightmap's second axis IS the 2D local y -- the render negation
 * `(x, y_2d) -> vec3(x, height, -y_2d)` is applied downstream by
 * TerrainMeshBuilder and by the corridor extruder independently -- so the cell
 * at (X, Z) samples the 2D point (X, Z), unmirrored. Getting this sign wrong
 * mirrors the terrain against the roads, and on a surface this symmetric it
 * would still look plausible in isolation -- which is exactly why the two meshes
 * are dumped together.
 */
float rolling_surface_world(float world_x, float world_z) {
    return rolling_surface(static_cast<double>(world_x), static_cast<double>(world_z));
}

// ============================================================================
// Fixtures
// ============================================================================

/// Absolute path of a fixture in tests/data
std::filesystem::path fixture_path(const char* filename) {
    return std::filesystem::path(STRATUM_TEST_DATA_DIR) / filename;
}

/// Absolute path of an output file in build/road_dump
std::filesystem::path dump_path(const std::string& name) {
    return std::filesystem::path(STRATUM_TEST_DUMP_DIR) / name;
}

/// Fixture file name without its extension
std::string stem_of(const char* filename) {
    return std::filesystem::path(filename).stem().string();
}

/// Parse one file with roads only
std::optional<ParsedOSMData> parse_roads(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture exists",
                                      "missing: " + path.string());
        return std::nullopt;
    }

    stratum::osm::OSMParser parser;
    stratum::osm::ParserConfig config;
    config.import_buildings = false;
    config.import_water = false;
    config.import_landuse = false;
    config.import_natural = false;
    config.simplify_geometry = false;
    parser.set_config(config);

    if (!parser.parse(path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "parser.parse(fixture)",
                                      path.string() + ": " + parser.get_error());
        return std::nullopt;
    }
    return parser.take_data();
}

// ============================================================================
// Stats
// ============================================================================

/// One row of the reported table
struct FixtureStats {
    std::string fixture;

    size_t pieces = 0;
    size_t triangles = 0;

    size_t solver_iterations = 0;
    size_t bridges = 0;
    size_t tunnels = 0;
    size_t grade_limited_edges = 0;
    float  max_residual = 0.0f;

    /// Steepest gradient of the RAW sampled terrain along any ground-following edge
    float max_grade_before = 0.0f;
    /**
     * @brief Steepest solved gradient among GROUND-FOLLOWING edges only
     *
     * Bridges and tunnels are excluded and reported separately in
     * max_grade_deck. Mixing them into one column makes the table lie: the
     * limit beside it belongs to a ground-following edge, and a deck gradient
     * printed against a ground-following limit reads as a 1.4x overshoot on
     * bridge_over.osm when the two numbers describe different edges.
     */
    float max_grade_after = 0.0f;
    /// Steepest solved gradient among bridge and tunnel edges; 0 when there are none
    float max_grade_deck = 0.0f;
    /**
     * @brief max_grade_after divided by the class limit of the edge that produced it
     *
     * Above 1.0 means the solve did not converge. This is the number the table is
     * really reporting; the raw gradient beside it is only readable.
     */
    float worst_limit_ratio = 0.0f;
    /// Class limit of the edge that produced worst_limit_ratio
    float worst_limit = 0.0f;
    /// Edges whose solved gradient exceeded their class limit beyond tolerance
    size_t unconverged_edges = 0;

    size_t carved_cells = 0;
    float  max_carve_delta = 0.0f;
    size_t heightmap_cells = 0;

    double elevation_ms = 0.0;
    double carve_ms = 0.0;
    double build_ms = 0.0;
};

/**
 * @brief Steepest raw terrain gradient along one centerline
 *
 * Sampled at the same stations the solver used, so it is comparable with
 * EdgeElevation::max_grade_used rather than being a different measurement of a
 * different curve.
 *
 * A bevelled joint is represented as two stations sharing one arclength, so
 * zero-length spans are skipped rather than dividing by zero.
 */
float raw_max_grade(const Centerline& centerline) {
    float worst = 0.0f;
    for (size_t i = 1; i < centerline.stations.size(); ++i) {
        const auto& a = centerline.stations[i - 1];
        const auto& b = centerline.stations[i];
        const double ds = b.arclength - a.arclength;
        if (!(ds > 1e-6)) continue;

        const double ha = static_cast<double>(rolling_surface(a.position.x, a.position.y));
        const double hb = static_cast<double>(rolling_surface(b.position.x, b.position.y));
        worst = std::max(worst, static_cast<float>(std::fabs(hb - ha) / ds));
    }
    return worst;
}

// ============================================================================
// Terrain
// ============================================================================

/**
 * @brief A heightmap of the natural surface covering the whole network
 *
 * Bounds come from the carve ribbons rather than the meshes, because a ribbon
 * already carries the corridor in the 2D local frame the carve works in.
 *
 * @param network Built network; its carve payload defines the region
 * @param out     Receives the heightmap
 * @return True when the network had a carve payload to bound
 */
bool make_natural_heightmap(const RoadNetwork& network, Heightmap& out) {
    glm::dvec2 lo{0.0};
    glm::dvec2 hi{0.0};
    bool have = false;

    for (const auto& ribbon : network.carve_ribbons) {
        for (const glm::dvec2& p : ribbon.centerline) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
            if (!have) {
                lo = hi = p;
                have = true;
            } else {
                lo = glm::min(lo, p);
                hi = glm::max(hi, p);
            }
        }
    }
    if (!have) return false;

    lo -= kTerrainMargin;
    hi += kTerrainMargin;

    // A Heightmap is indexed in the same 2D local frame as the payload, so the
    // row range is the Y range as it stands -- no negation, no swapped ends.
    const double world_min_x = lo.x;
    const double world_min_z = lo.y;
    const double world_max_x = hi.x;
    const double world_max_z = hi.y;

    out.cell_size_x = kTerrainCell;
    out.cell_size_z = kTerrainCell;
    out.origin = glm::vec2{static_cast<float>(world_min_x), static_cast<float>(world_min_z)};
    out.width = static_cast<int>((world_max_x - world_min_x) / kTerrainCell) + 1;
    out.height = static_cast<int>((world_max_z - world_min_z) / kTerrainCell) + 1;
    out.data.assign(static_cast<size_t>(out.width) * static_cast<size_t>(out.height), 0.0f);

    for (int z = 0; z < out.height; ++z) {
        for (int x = 0; x < out.width; ++x) {
            const float wx = out.origin.x + static_cast<float>(x) * out.cell_size_x;
            const float wz = out.origin.y + static_cast<float>(z) * out.cell_size_z;
            out.set(x, z, rolling_surface_world(wx, wz));
        }
    }
    return true;
}

/// A CarveInput over a built network's payload, indexed and ready to carve
CarveInput make_carve_input(const RoadNetwork& network) {
    CarveInput input;
    input.ribbons = network.carve_ribbons;
    input.discs = network.carve_discs;

    // Portal mouths belong here too, or the dumped terrain shows a hillside
    // closed over every tunnel arch that the shipping pipeline opens.
    input.portals = network.carve_portals;
    input.build_index();
    return input;
}

// ============================================================================
// One fixture
// ============================================================================

/**
 * @brief Build, dump, carve, and measure one fixture
 *
 * @param filename     Fixture file name in tests/data
 * @param stats        Receives the row
 * @param want_terrain Also dump the carved heightmap as `<fixture>_terrain.obj`
 * @param paths        Receives every file written
 * @return True when the fixture parsed and built
 */
bool run_fixture(const char* filename, FixtureStats& stats, bool want_terrain,
                 std::vector<std::string>& paths) {
    const std::optional<ParsedOSMData> data = parse_roads(fixture_path(filename));
    if (!data) return false;

    stats.fixture = filename;

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{rolling_surface};

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*data, cfg);

    stats.pieces = network.stats.pieces;
    stats.triangles = network.stats.triangles;
    stats.elevation_ms = network.stats.elevation_ms;
    stats.build_ms = network.stats.build_ms;

    if (network.pieces.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture produced geometry",
                                      std::string{filename} + ": empty network");
        return false;
    }
    if (!builder.elevation().is_solved()) {
        stratum::test::report_failure(__FILE__, __LINE__, "elevation solved with a sampler",
                                      std::string{filename} + ": solver reports unsolved");
        return false;
    }

    // ---- the elevated road dump -------------------------------------------
    std::vector<const Mesh*> meshes;
    meshes.reserve(network.pieces.size());
    for (const auto& piece : network.pieces) meshes.push_back(&piece.mesh);

    const auto road_path = dump_path(stem_of(filename) + "_elevated.obj");
    ObjDumpStats dump;
    std::string error;
    if (!write_obj(meshes, road_path, &dump, &error)) {
        stratum::test::report_failure(__FILE__, __LINE__, "write_obj(elevated)",
                                      std::string{filename} + ": " + error);
        return false;
    }
    paths.push_back(road_path.string());

    // Every triangle the extruder emitted must reach the file, or the thing being
    // looked at is not the thing that was built.
    if (dump.triangles != network.stats.triangles) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "every emitted triangle reaches the elevated OBJ",
            std::string{filename} + ": wrote " + std::to_string(dump.triangles) + " of " +
                std::to_string(network.stats.triangles));
    }

    // ---- solver stats ------------------------------------------------------
    const auto solver_stats = builder.elevation().stats();
    stats.solver_iterations = solver_stats.iterations;
    stats.bridges = solver_stats.bridges;
    stats.tunnels = solver_stats.tunnels;
    stats.grade_limited_edges = solver_stats.grade_limited_edges;
    stats.max_residual = solver_stats.max_residual;

    // ---- grade, before and after ------------------------------------------
    const auto& graph = builder.graph();
    const auto& centerlines = builder.centerlines();

    for (size_t e = 0; e < graph.edges().size(); ++e) {
        const EdgeElevation& elev = builder.elevation().edge(e);
        if (elev.station_heights.empty()) continue;

        const GraphEdge& edge = graph.edge(e);

        if (e < centerlines.size() && !elev.is_bridge && !elev.is_tunnel) {
            stats.max_grade_before = std::max(stats.max_grade_before, raw_max_grade(centerlines[e]));
        }

        // A bridge deck's gradient is set by its abutments and the ground beneath
        // it is irrelevant, so it is neither held to the ground-following limit
        // nor mixed into the column that limit is printed beside.
        if (elev.is_bridge || elev.is_tunnel) {
            stats.max_grade_deck = std::max(stats.max_grade_deck, elev.max_grade_used);
            continue;
        }

        stats.max_grade_after = std::max(stats.max_grade_after, elev.max_grade_used);

        const float limit = max_grade_for(edge.type, cfg.elevation);
        const float ratio = (limit > 0.0f) ? elev.max_grade_used / limit : 0.0f;
        if (ratio > stats.worst_limit_ratio) {
            stats.worst_limit_ratio = ratio;
            stats.worst_limit = limit;
        }

        // The load-bearing assertion. Above its class limit the profile is not a
        // steep road, it is an unconverged one, and every other number reported
        // for this fixture is describing garbage. 2% of the limit absorbs float
        // accumulation over a long profile and nothing else.
        if (elev.max_grade_used > limit * 1.02f) {
            ++stats.unconverged_edges;
            stratum::test::report_failure(
                __FILE__, __LINE__, "solved grade is within the class limit",
                std::string{filename} + " edge " + std::to_string(e) + " (way "
                    + std::to_string(edge.source_way) + "): solved grade "
                    + stratum::test::stringify(elev.max_grade_used * 100.0f)
                    + "% against a class limit of "
                    + stratum::test::stringify(limit * 100.0f)
                    + "% -- THE ELEVATION SOLVE DID NOT CONVERGE, residual "
                    + stratum::test::stringify(solver_stats.max_residual) + " m after "
                    + std::to_string(solver_stats.iterations) + " sweeps");
        }
    }

    // ---- the carve ---------------------------------------------------------
    Heightmap heightmap;
    if (!make_natural_heightmap(network, heightmap)) return true;
    stats.heightmap_cells = heightmap.data.size();

    const CarveInput input = make_carve_input(network);
    if (!input.has_index()) {
        stratum::test::report_failure(__FILE__, __LINE__, "carve input is indexed",
                                      std::string{filename} + ": build_index() produced nothing");
        return true;
    }

    const CarveStats carved = carve_terrain(heightmap, input);
    stats.carved_cells = carved.cells_modified;
    stats.max_carve_delta = carved.max_delta;
    stats.carve_ms = carved.carve_ms;

    // A network with unsuppressed ribbons that carves nothing has either lost its
    // payload or lost its index, and the terrain dump below would be a picture of
    // the natural surface with a road hovering over it.
    size_t carveable = 0;
    for (const auto& ribbon : network.carve_ribbons) {
        if (!ribbon.suppress) ++carveable;
    }
    if (carveable > 0 && carved.cells_modified == 0) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "an unsuppressed corridor carves the terrain",
            std::string{filename} + ": " + std::to_string(carveable)
                + " carveable ribbons changed no cells");
    }

    // ---- the terrain dump --------------------------------------------------
    if (!want_terrain) return true;

    TerrainMeshConfig mesh_cfg;
    mesh_cfg.generate_water_mesh = false;
    const Mesh terrain = TerrainMeshBuilder::build_terrain_mesh(heightmap, mesh_cfg);

    const auto terrain_path = dump_path(stem_of(filename) + "_terrain.obj");
    if (!write_obj(terrain, terrain_path, nullptr, &error)) {
        stratum::test::report_failure(__FILE__, __LINE__, "write_obj(terrain)",
                                      std::string{filename} + ": " + error);
        return true;
    }
    paths.push_back(terrain_path.string());

    CHECK_TRUE(!terrain.vertices.empty());
    return true;
}

// ============================================================================
// Table
// ============================================================================

void print_table(const std::vector<FixtureStats>& rows) {
    std::cout << "\n  P3 elevation and carve, synthetic rolling terrain"
              << " (peak grade ~45%)\n\n"
              << "  " << std::left << std::setw(20) << "fixture" << std::right
              << std::setw(7) << "pieces"
              << std::setw(8) << "tris"
              << std::setw(7) << "iters"
              << std::setw(11) << "grade pre"
              << std::setw(12) << "grade post"
              << std::setw(9) << "limit"
              << std::setw(8) << "ratio"
              << std::setw(9) << "deck"
              << std::setw(7) << "brdg"
              << std::setw(6) << "tun"
              << std::setw(10) << "carved"
              << std::setw(11) << "max delta" << '\n';

    for (const FixtureStats& r : rows) {
        std::cout << "  " << std::left << std::setw(20) << r.fixture << std::right
                  << std::setw(7) << r.pieces
                  << std::setw(8) << r.triangles
                  << std::setw(7) << r.solver_iterations
                  << std::fixed << std::setprecision(1)
                  << std::setw(10) << (r.max_grade_before * 100.0f) << "%"
                  << std::setw(11) << (r.max_grade_after * 100.0f) << "%"
                  << std::setw(8) << (r.worst_limit * 100.0f) << "%"
                  << std::setprecision(3) << std::setw(8) << r.worst_limit_ratio
                  << std::setprecision(1) << std::setw(8) << (r.max_grade_deck * 100.0f) << "%"
                  << std::setw(7) << r.bridges
                  << std::setw(6) << r.tunnels
                  << std::setw(10) << r.carved_cells
                  << std::setprecision(2) << std::setw(10) << r.max_carve_delta << "m"
                  << std::defaultfloat << '\n';
    }

    std::cout << "\n  " << std::left << std::setw(20) << "fixture" << std::right
              << std::setw(12) << "elev ms"
              << std::setw(12) << "carve ms"
              << std::setw(12) << "build ms"
              << std::setw(12) << "hm cells"
              << std::setw(14) << "limited edges" << '\n';
    for (const FixtureStats& r : rows) {
        std::cout << "  " << std::left << std::setw(20) << r.fixture << std::right
                  << std::fixed << std::setprecision(3)
                  << std::setw(12) << r.elevation_ms
                  << std::setw(12) << r.carve_ms
                  << std::setw(12) << r.build_ms
                  << std::setw(12) << r.heightmap_cells
                  << std::setw(14) << r.grade_limited_edges
                  << std::defaultfloat << '\n';
    }
    std::cout << '\n';
}

} // namespace

// ============================================================================
// The dump
// ============================================================================

TEST(RoadTerrainDump, every_fixture_builds_on_terrain_and_carves_it) {
    std::vector<FixtureStats> rows;
    std::vector<std::string> paths;

    for (const char* fixture : kAllFixtures) {
        const bool want_terrain =
            std::find_if(std::begin(kTerrainFixtures), std::end(kTerrainFixtures),
                         [&](const char* f) { return std::string{f} == fixture; })
            != std::end(kTerrainFixtures);

        FixtureStats stats;
        if (!run_fixture(fixture, stats, want_terrain, paths)) continue;
        rows.push_back(stats);
    }

    print_table(rows);

    std::cout << "  files -> " << STRATUM_TEST_DUMP_DIR << "\n\n";
    for (const std::string& path : paths) std::cout << "  " << path << '\n';
    std::cout << "\n  Load <fixture>_elevated.obj and <fixture>_terrain.obj together:"
              << " same world space, same metres.\n"
              << "  The carriageway must sit just above its cutting along its whole"
              << " length.\n\n";

    CHECK_EQ(rows.size(), std::size(kAllFixtures));

    // Restated outside the per-edge loop so the suite fails on the summary too,
    // not only on whichever edge happened to be reported first.
    size_t unconverged = 0;
    for (const FixtureStats& r : rows) unconverged += r.unconverged_edges;
    CHECK_EQ(unconverged, size_t{0});
}

// ============================================================================
// Road against terrain
// ============================================================================

/**
 * The agreement check the OBJ pair exists to let a human make, made numerically
 * so it also fails in CI. For every station of every unsuppressed ribbon, the
 * carved terrain directly under the centerline must be at the road surface
 * height. A road floating above its cutting or sunk into it fails here, and both
 * are invisible to every other suite in the tree.
 */
TEST(RoadTerrainDump, the_carved_terrain_meets_the_road_it_was_carved_for) {
    for (const char* fixture : kTerrainFixtures) {
        const std::optional<ParsedOSMData> data = parse_roads(fixture_path(fixture));
        if (!data) continue;

        RoadNetworkConfig cfg;
        cfg.height_sampler = HeightSampler{rolling_surface};

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*data, cfg);

        Heightmap heightmap;
        if (!make_natural_heightmap(network, heightmap)) continue;

        const CarveInput input = make_carve_input(network);
        const CarveStats carved = carve_terrain(heightmap, input);
        CHECK_TRUE(carved.cells_modified > size_t{0});

        const std::string label{fixture};
        const double offset = static_cast<double>(cfg.elevation.surface_offset);

        size_t checked = 0;
        size_t floating = 0;
        size_t sunk = 0;
        float worst_gap = 0.0f;
        float worst_clearance = std::numeric_limits<float>::max();

        // Ribbons are parallel to the EDGE pieces, which come first; P4 appends
        // one piece per solved junction after them, and a junction carries a
        // disc rather than a ribbon.
        size_t edge_pieces = 0;
        for (const auto& piece : network.pieces) {
            if (piece.edge != stratum::osm::road::kInvalidId) ++edge_pieces;
        }
        CHECK_EQ(network.carve_ribbons.size(), edge_pieces);

        for (size_t r = 0; r < network.carve_ribbons.size(); ++r) {
            const auto& ribbon = network.carve_ribbons[r];
            if (ribbon.suppress) continue;
            if (ribbon.centerline.size() != ribbon.centerline_heights.size()) continue;

            const EdgeId edge = network.pieces[r].edge;
            const EdgeElevation& elev = builder.elevation().edge(edge);
            const bool have_surface = elev.station_heights.size() == ribbon.centerline.size();

            for (size_t i = 0; i < ribbon.centerline.size(); ++i) {
                const glm::dvec2& p = ribbon.centerline[i];

                // Stations at the very ends of a ribbon sit inside the junction
                // disc, which carves to the node height rather than to the arm's
                // own last station. That is correct -- the disc wins by carve
                // weight -- so the ends are not part of this check.
                if (i == 0 || i + 1 == ribbon.centerline.size()) continue;

                const float wx = static_cast<float>(p.x);
                const float wz = static_cast<float>(p.y);
                const float ground = heightmap.sample(wx, wz);

                ++checked;

                // The terrain must land on the CARVE TARGET.
                const float gap = std::fabs(ribbon.centerline_heights[i] - ground);
                worst_gap = std::max(worst_gap, gap);
                if (gap > 0.35f) ++floating;

                // And the ROAD SURFACE must then clear it. This is the whole
                // point of ElevationConfig::surface_offset, and it is zero the
                // moment the offset is applied to the carve target as well as to
                // the mesh -- the two lift together and z-fight.
                if (!have_surface) continue;
                const float clearance = elev.station_heights[i] - ground;
                worst_clearance = std::min(worst_clearance, clearance);
                if (static_cast<double>(clearance) < 0.5 * offset) ++sunk;
            }
        }

        CHECK_TRUE(checked > size_t{0});
        if (floating > 0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "carved terrain sits under the road it carved",
                label + ": " + std::to_string(floating) + " of " + std::to_string(checked)
                    + " stations disagree with the terrain beneath them, worst "
                    + stratum::test::stringify(worst_gap) + " m");
        }
        if (sunk > 0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "road surface clears the terrain it carved",
                label + ": " + std::to_string(sunk) + " of " + std::to_string(checked)
                    + " stations have less than half of surface_offset ("
                    + stratum::test::stringify(static_cast<float>(offset))
                    + " m) of clearance, worst "
                    + stratum::test::stringify(worst_clearance) + " m");
        }
    }
}

// ============================================================================
// Scale
// ============================================================================
//
// The nine fixtures are between 130 m and 550 m across and carry between one and
// four edges. Timing the elevation solve on four edges says nothing at all about
// the import path, which is where both of these run: a city extract carries tens
// of thousands. So the cost question is asked against a synthetic grid instead,
// sized like a small town, and the carve is measured the way the shipping code
// actually does it -- per chunk, at TerrainTileConfig's default 500 m / 64
// vertices, over every chunk the network touches.
//
// This is a measurement, not a threshold. It asserts only that the work completed
// and prints what it cost; a machine-dependent millisecond budget in a test is a
// flaky test. The number is for the report.

namespace {

/// Streets per axis in the synthetic grid
constexpr int kGridStreets = 24;

/// Metres between adjacent parallel streets
constexpr double kGridSpacing = 160.0;

/// Chunk edge in metres, matching TerrainTileConfig::chunk_size
constexpr float kChunkSize = 500.0f;

/// Vertices per chunk edge, matching TerrainTileConfig::chunk_resolution
constexpr int kChunkResolution = 64;

/**
 * @brief Write a grid of residential streets as OSM XML
 *
 * kGridStreets ways east-west and kGridStreets north-south, every crossing a
 * shared node, so the graph gets real interior junctions rather than a pile of
 * disjoint ways. Written into the build tree, not the source tree: it is
 * regenerated on every run and must never be mistaken for a committed fixture.
 *
 * @param path Destination .osm path
 * @return True when the file was written
 */
bool write_grid_osm(const std::filesystem::path& path) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);

    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;

    // Degrees per metre near 53.34 N, good enough for a synthetic extract.
    constexpr double kLat0 = 53.30;
    constexpr double kLon0 = -6.30;
    constexpr double kDegPerMetreLat = 1.0 / 111320.0;
    const double deg_per_metre_lon = kDegPerMetreLat / std::cos(kLat0 * 3.14159265358979 / 180.0);

    out << "<?xml version='1.0' encoding='UTF-8'?>\n<osm version='0.6' generator='stratum_tests'>\n";
    out << std::setprecision(9) << std::fixed;

    const auto node_id = [](int ix, int iy) { return 1000000 + iy * 1000 + ix; };

    for (int iy = 0; iy < kGridStreets; ++iy) {
        for (int ix = 0; ix < kGridStreets; ++ix) {
            const double lat = kLat0 + static_cast<double>(iy) * kGridSpacing * kDegPerMetreLat;
            const double lon = kLon0 + static_cast<double>(ix) * kGridSpacing * deg_per_metre_lon;
            out << "  <node id='" << node_id(ix, iy) << "' lat='" << lat << "' lon='" << lon
                << "' version='1'/>\n";
        }
    }

    int way_id = 2000000;
    for (int iy = 0; iy < kGridStreets; ++iy) {
        out << "  <way id='" << way_id++ << "' version='1'>\n";
        for (int ix = 0; ix < kGridStreets; ++ix) {
            out << "    <nd ref='" << node_id(ix, iy) << "'/>\n";
        }
        out << "    <tag k='highway' v='residential'/>\n"
               "    <tag k='lanes' v='2'/>\n"
               "    <tag k='sidewalk' v='both'/>\n"
               "  </way>\n";
    }
    for (int ix = 0; ix < kGridStreets; ++ix) {
        out << "  <way id='" << way_id++ << "' version='1'>\n";
        for (int iy = 0; iy < kGridStreets; ++iy) {
            out << "    <nd ref='" << node_id(ix, iy) << "'/>\n";
        }
        out << "    <tag k='highway' v='residential'/>\n"
               "    <tag k='lanes' v='2'/>\n"
               "    <tag k='sidewalk' v='both'/>\n"
               "  </way>\n";
    }

    out << "</osm>\n";
    return out.good();
}

} // namespace

TEST(RoadTerrainDump, elevation_and_carve_cost_on_a_town_sized_network) {
    const auto grid_path = dump_path("scale_grid.osm");
    if (!write_grid_osm(grid_path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "write the synthetic grid",
                                      grid_path.string());
        return;
    }

    const std::optional<ParsedOSMData> data = parse_roads(grid_path);
    if (!data) return;

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{rolling_surface};

    RoadNetworkBuilder builder;
    const auto build_start = std::chrono::steady_clock::now();
    const RoadNetwork network = builder.build(*data, cfg);
    const double build_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start)
            .count();

    CHECK_TRUE(!network.pieces.empty());
    if (network.pieces.empty()) return;
    CHECK_TRUE(builder.elevation().is_solved());

    // The grid must actually be a network. A pile of disjoint ways would make the
    // whole measurement meaningless: the node pre-pass and the shared-height
    // relaxation are most of the solve, and neither runs without junctions.
    size_t junctions = 0;
    for (const auto& node : builder.graph().nodes()) {
        if (node.is_junction()) ++junctions;
    }
    CHECK_TRUE(junctions > size_t{100});

    // ---- carve, chunk by chunk, as the import path does --------------------
    glm::dvec2 lo{0.0};
    glm::dvec2 hi{0.0};
    bool have = false;
    for (const auto& ribbon : network.carve_ribbons) {
        for (const glm::dvec2& p : ribbon.centerline) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
            if (!have) { lo = hi = p; have = true; }
            else { lo = glm::min(lo, p); hi = glm::max(hi, p); }
        }
    }
    CHECK_TRUE(have);
    if (!have) return;

    const CarveInput input = make_carve_input(network);
    CHECK_TRUE(input.has_index());

    const auto index_start = std::chrono::steady_clock::now();
    CarveInput timed_index;
    timed_index.ribbons = network.carve_ribbons;
    timed_index.discs = network.carve_discs;
    timed_index.build_index();
    const double index_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - index_start)
            .count();

    const float world_min_x = static_cast<float>(lo.x) - 100.0f;
    const float world_max_x = static_cast<float>(hi.x) + 100.0f;
    const float world_min_z = static_cast<float>(lo.y) - 100.0f;
    const float world_max_z = static_cast<float>(hi.y) + 100.0f;

    const float cell = kChunkSize / static_cast<float>(kChunkResolution - 1);
    const int chunks_x = static_cast<int>((world_max_x - world_min_x) / kChunkSize) + 1;
    const int chunks_z = static_cast<int>((world_max_z - world_min_z) / kChunkSize) + 1;

    double total_carve_ms = 0.0;
    double total_generate_ms = 0.0;
    size_t total_cells = 0;
    size_t total_carved = 0;
    size_t chunks_touched = 0;
    float worst_delta = 0.0f;

    Heightmap chunk;
    chunk.width = kChunkResolution;
    chunk.height = kChunkResolution;
    chunk.cell_size_x = cell;
    chunk.cell_size_z = cell;
    chunk.data.resize(static_cast<size_t>(kChunkResolution) * kChunkResolution);

    for (int cz = 0; cz < chunks_z; ++cz) {
        for (int cx = 0; cx < chunks_x; ++cx) {
            chunk.origin = glm::vec2{world_min_x + static_cast<float>(cx) * kChunkSize,
                                     world_min_z + static_cast<float>(cz) * kChunkSize};

            // Generating the natural surface is timed separately: it is the cost
            // the carve is being compared AGAINST, not part of it.
            const auto gen_start = std::chrono::steady_clock::now();
            for (int z = 0; z < chunk.height; ++z) {
                for (int x = 0; x < chunk.width; ++x) {
                    const float wx = chunk.origin.x + static_cast<float>(x) * chunk.cell_size_x;
                    const float wz = chunk.origin.y + static_cast<float>(z) * chunk.cell_size_z;
                    chunk.set(x, z, rolling_surface_world(wx, wz));
                }
            }
            total_generate_ms += std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - gen_start)
                                     .count();

            const CarveStats carved = carve_terrain(chunk, input);
            total_carve_ms += carved.carve_ms;
            total_cells += chunk.data.size();
            total_carved += carved.cells_modified;
            worst_delta = std::max(worst_delta, carved.max_delta);
            if (carved.cells_modified > 0) ++chunks_touched;
        }
    }

    const auto solver_stats = builder.elevation().stats();

    // The only trustworthy convergence signal. Stats::iterations is SUMMED over
    // the node pre-pass, the station solve, and the post-override re-relaxation,
    // so it routinely exceeds ElevationConfig::max_iterations without any single
    // phase having hit the cap -- comparing it against the cap would report a
    // false alarm. The residual is per-phase and is the number the cap is there
    // to bound.
    const bool converged = solver_stats.max_residual <= cfg.elevation.convergence_epsilon;
    if (!converged) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "the elevation solve converged on a town-sized network",
            "residual " + stratum::test::stringify(solver_stats.max_residual)
                + " m exceeds convergence_epsilon "
                + stratum::test::stringify(cfg.elevation.convergence_epsilon)
                + " m after " + std::to_string(solver_stats.iterations)
                + " sweeps -- THE SOLVE HIT ITS ITERATION CAP AND RETURNED AN"
                  " UNSOLVED PROFILE. Every grade in the table above is describing"
                  " an unconverged road.");
    }

    const double network_km2 = (static_cast<double>(world_max_x - world_min_x) / 1000.0)
                               * (static_cast<double>(world_max_z - world_min_z) / 1000.0);

    std::cout << "\n  P3 cost on a town-sized network (synthetic grid, "
              << kGridStreets << "x" << kGridStreets << " streets)\n\n"
              << std::fixed << std::setprecision(2)
              << "    area                   " << network_km2 << " km2\n"
              << "    graph edges            " << builder.graph().edges().size() << '\n'
              << "    junctions              " << junctions << '\n'
              << "    pieces                 " << network.stats.pieces << '\n'
              << "    triangles              " << network.stats.triangles << '\n'
              << "    solver sweeps          " << solver_stats.iterations << '\n'
              << "    grade-limited edges    " << solver_stats.grade_limited_edges << '\n'
              << "    max residual           " << std::setprecision(4) << solver_stats.max_residual
              << " m  (epsilon " << cfg.elevation.convergence_epsilon << " m, "
              << (converged ? "CONVERGED" : "NOT CONVERGED") << ")\n"
              << std::setprecision(2)
              << "\n    ELEVATION SOLVE        " << network.stats.elevation_ms << " ms  ("
              << (network.stats.elevation_ms / std::max(1.0, build_ms) * 100.0)
              << "% of the " << build_ms << " ms road build)\n"
              << "    carve index build      " << index_ms << " ms\n"
              << "    CARVE, ALL CHUNKS      " << total_carve_ms << " ms  over " << (chunks_x * chunks_z)
              << " chunks, " << chunks_touched << " touched\n"
              << "    carve per touched chunk " << (total_carve_ms / std::max<size_t>(1, chunks_touched))
              << " ms\n"
              << "    natural surface gen    " << total_generate_ms
              << " ms  (the cost the carve is added to)\n"
              << "    cells                  " << total_carved << " carved of " << total_cells
              << '\n'
              << "    max carve delta        " << worst_delta << " m\n"
              << std::defaultfloat << '\n';

    CHECK_TRUE(total_carved > size_t{0});
    CHECK_TRUE(chunks_touched > size_t{0});
    CHECK_TRUE(network.stats.elevation_ms >= 0.0);
}

// ============================================================================
// Bridge abutments
// ============================================================================

TEST(RoadTerrainDump, the_approach_embankment_climbs_to_meet_the_deck) {
    // A bridge deck is suppressed from carving, so the terrain under a span stays
    // natural. That is deliberate. What it leaves open is the ABUTMENT: the deck
    // has been lifted to buy clearance, and if the terrain does not climb with the
    // approach that lifted it, the road arrives at the bridge on a shelf of thin
    // air and the deck reads as starting nowhere.
    //
    // bridge_abutment.osm is the standard encoding: approach, deck, approach, all
    // one road, meeting at two degree-2 nodes. Nothing here asserts anything about
    // the deck. It asserts that the ground at each abutment has been carved up to
    // the height the approach arrives at, and that it falls away again under the
    // span rather than filling it in.
    const std::optional<ParsedOSMData> data = parse_roads(fixture_path("bridge_abutment.osm"));
    if (!data) return;

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{rolling_surface};

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*data, cfg);

    CHECK_TRUE(!network.pieces.empty());
    CHECK_TRUE(builder.elevation().is_solved());

    // The deck, and the two nodes it ends on.
    const auto& graph = builder.graph();
    size_t deck_edge = graph.edges().size();
    for (size_t e = 0; e < graph.edges().size(); ++e) {
        if (builder.elevation().edge(e).is_bridge) {
            deck_edge = e;
            break;
        }
    }
    CHECK_TRUE(deck_edge < graph.edges().size());
    if (deck_edge >= graph.edges().size()) return;

    const GraphEdge& deck = graph.edge(deck_edge);
    const stratum::osm::road::GraphNodeId abutments[2] = {deck.from, deck.to};

    Heightmap hm;
    CHECK_TRUE(make_natural_heightmap(network, hm));
    if (hm.data.empty()) return;

    const Heightmap natural = hm;
    const CarveInput input = make_carve_input(network);
    const CarveStats carve = carve_terrain(hm, input);
    CHECK_TRUE(carve.cells_modified > size_t{0});

    // Height at a 2D local position, nearest cell. The heightmap covers the whole
    // network with a margin, so every position tested below is inside it.
    const auto height_at = [](const Heightmap& map, glm::dvec2 p) {
        const int ix = std::clamp(
            static_cast<int>(std::lround((p.x - static_cast<double>(map.origin.x))
                                         / static_cast<double>(map.cell_size_x))),
            0, map.width - 1);
        const int iz = std::clamp(
            static_cast<int>(std::lround((p.y - static_cast<double>(map.origin.y))
                                         / static_cast<double>(map.cell_size_z))),
            0, map.height - 1);
        return static_cast<double>(map.at(ix, iz));
    };

    const double drop = static_cast<double>(cfg.elevation.surface_offset);

    // Unit vector from the first abutment to the second: the direction the span
    // runs, and the direction the terrain has to fall away in.
    const glm::dvec2 span = graph.nodes()[abutments[1]].position
                          - graph.nodes()[abutments[0]].position;
    const double span_len = glm::length(span);
    CHECK_TRUE(span_len > 1.0);
    if (!(span_len > 1.0)) return;
    const glm::dvec2 along = span / span_len;

    std::cout << "\n  bridge_abutment.osm, span " << std::fixed << std::setprecision(2)
              << span_len << " m\n";

    for (int end = 0; end < 2; ++end) {
        const stratum::osm::road::GraphNodeId node = abutments[end];
        const glm::dvec2 at = graph.nodes()[node].position;
        const double deck_y = static_cast<double>(builder.elevation().node_height(node));
        const double target = deck_y - drop;

        const double carved = height_at(hm, at);
        const double raw = height_at(natural, at);

        // A metre in under the span, where the deck is suppressed and the
        // embankment should already be falling away.
        const glm::dvec2 inward = at + along * (end == 0 ? 8.0 : -8.0);
        const double carved_under = height_at(hm, inward);

        std::cout << "    abutment " << end << "  deck " << deck_y << " m"
                  << "  natural " << raw << " m"
                  << "  carved " << carved << " m"
                  << "  8 m under the span " << carved_under << " m\n";

        // The load-bearing one. The ground at the abutment is the approach's own
        // carve target, so it has to land on it: an embankment that stops short
        // leaves the deck starting off a cliff.
        CHECK_NEAR(carved, target, 0.35);

        // And it really is an embankment: the deck was lifted for clearance, so
        // the natural surface was well below it.
        CHECK_TRUE(target - raw > 1.0);

        // Under the span the ground has begun to fall back. Without this the
        // "embankment" is a plateau filling the gap the bridge exists to cross.
        CHECK_TRUE(carved_under < carved - 0.25);
    }

    // The profile under the span, for the shape of the thing rather than two
    // points of it. The embankment nose is allowed to reach in from each end --
    // it is the same blend that gives every dead end a nose instead of a cliff --
    // but the middle of a span must still be spanning something.
    std::cout << "    under the span:";
    double mid_gap = 0.0;
    for (int i = 0; i <= 8; ++i) {
        const double t = static_cast<double>(i) / 8.0;
        const glm::dvec2 at = graph.nodes()[abutments[0]].position + along * (span_len * t);
        const double gap = height_at(natural, at) - height_at(hm, at);
        std::cout << ' ' << std::setprecision(1) << (height_at(hm, at) - height_at(natural, at));
        if (i == 4) mid_gap = -gap;
    }
    std::cout << "  m of fill\n" << std::defaultfloat;

    // Mid-span the deck must still be over ground close to natural. A fill that
    // reaches the middle has closed the void the bridge exists to cross.
    CHECK_TRUE(std::fabs(mid_gap) < 2.0);

    std::cout << std::defaultfloat;
}

// ============================================================================
// Tunnel portals
// ============================================================================

TEST(RoadTerrainDump, every_portal_mouth_that_reaches_the_carve_is_opened_by_it) {
    // The P6 hand-off, end to end, over every fixture that carries a tunnel.
    // Three things have to line up for a tunnel to have a mouth: the builder finds
    // a portal, its footprint survives compaction into RoadNetwork::carve_portals,
    // and the carve bins and applies it. Each is asserted here, because the
    // failure of any one of them looks identical from outside -- a headwall set
    // into a hillside that has closed over the arch.
    //
    // ### The count this prints is currently ZERO, and that is a producer defect
    //
    // build_tunnel_portals() places a portal where the solved road surface first
    // passes below the terrain, walking inward from each end of the TUNNEL EDGE.
    // On the standard OSM encoding -- approach way, `tunnel=yes` way, approach way,
    // meeting at two shared nodes -- the elevation solver drops BOTH of those
    // shared nodes by ElevationConfig::tunnel_depth, so the tunnel edge is already
    // eight metres under at its own first station. find_start_portal() reads that
    // as "an interior slice of a longer tunnel" and emits nothing, at either end.
    // The surface crossing is on the APPROACH edges, which are not tunnels and are
    // never searched.
    //
    // So this test is vacuous today. It is written to be vacuous LOUDLY -- the
    // portal total is printed -- rather than to assert the current behaviour,
    // because the current behaviour is wrong and a test that locked it in would
    // have to be deleted to fix it. Every assertion below becomes live the moment
    // a footprint is produced.
    static const char* const kFixtures[] = {
        "tunnel.osm", "bridge_over.osm", "bridge_abutment.osm", "four_way.osm",
    };

    const auto height_at = [](const Heightmap& map, glm::dvec2 p) {
        const int ix = std::clamp(
            static_cast<int>(std::lround((p.x - static_cast<double>(map.origin.x))
                                         / static_cast<double>(map.cell_size_x))),
            0, map.width - 1);
        const int iz = std::clamp(
            static_cast<int>(std::lround((p.y - static_cast<double>(map.origin.y))
                                         / static_cast<double>(map.cell_size_z))),
            0, map.height - 1);
        return static_cast<double>(map.at(ix, iz));
    };

    size_t portals_seen = 0;
    size_t tunnel_edges_seen = 0;
    size_t obstructed = 0;

    for (const char* filename : kFixtures) {
        const std::optional<ParsedOSMData> data = parse_roads(fixture_path(filename));
        if (!data) continue;

        RoadNetworkConfig cfg;
        cfg.height_sampler = HeightSampler{rolling_surface};

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*data, cfg);
        if (network.pieces.empty()) continue;

        tunnel_edges_seen += builder.elevation().stats().tunnels;
        portals_seen += network.carve_portals.size();

        if (network.carve_portals.empty()) continue;

        Heightmap hm;
        if (!make_natural_heightmap(network, hm) || hm.data.empty()) continue;

        const CarveInput input = make_carve_input(network);

        // The plumbing: a portal footprint that never became an index item can
        // never open anything.
        CHECK_EQ(input.item_count(),
                 network.carve_ribbons.size() + network.carve_discs.size()
                     + network.carve_portals.size());

        const CarveStats carve = carve_terrain(hm, input);
        CHECK_TRUE(carve.cells_modified > size_t{0});

        // Nowhere inside a mouth may the ground stand above the crown of its arch.
        // That is the whole contract: the headwall frames an opening, and terrain
        // above the crown closes it.
        for (const auto& portal : network.carve_portals) {
            const glm::dvec2 axis = glm::length(portal.axis) > 1e-9
                                        ? glm::normalize(portal.axis)
                                        : glm::dvec2{1.0, 0.0};

            for (int i = 0; i <= 6; ++i) {
                const double t = portal.depth * static_cast<double>(i) / 6.0;
                const glm::dvec2 at = portal.center + axis * t;

                // A tenth of a metre of tolerance: the mouth is sampled at the
                // nearest heightmap node, not at the exact query point.
                if (height_at(hm, at) > static_cast<double>(portal.crown_height) + 0.1) {
                    ++obstructed;
                }
            }
        }
    }

    std::cout << "\n  tunnel portals: " << portals_seen << " mouth(s) from "
              << tunnel_edges_seen << " solved tunnel edge(s) across "
              << (sizeof(kFixtures) / sizeof(kFixtures[0])) << " fixtures\n";
    if (portals_seen == 0 && tunnel_edges_seen > 0) {
        std::cout << "    NOTE: no portal footprints were produced. See the comment on this "
                     "test:\n"
                     "    the solver buries a tunnel edge at both of its own ends, so "
                     "build_tunnel_portals()\n"
                     "    finds no surface crossing to place a mouth at. The carve side is "
                     "unexercised.\n";
    }

    CHECK_EQ(obstructed, size_t{0});
}
