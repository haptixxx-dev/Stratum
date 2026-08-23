/**
 * @file test_road_dump.cpp
 * @brief Writes every fixture's road network to OBJ and checks the material mix
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Two jobs, and the first one is not an assertion.
 *
 * 1. Dump. OSM import is GUI-only and there is no CLI, so this suite is the only
 *    way to look at what the corridor extruder produced. Every fixture in
 *    tests/data is parsed, built, and written to build/road_dump/<fixture>.obj
 *    with one usemtl group per MaterialId. The paths are printed so a reader can
 *    open them. Nothing else in the test tree produces a file a human can inspect.
 *
 * 2. Material mix. A road network that comes back with triangles is not evidence
 *    that the profile builder ran: a flat asphalt ribbon with no curb and no
 *    sidewalk passes every count-based check in the other suites. The residential
 *    and urban fixtures therefore assert that Asphalt, Curb, and Sidewalk are all
 *    present, and that asphalt is the largest surface by AREA.
 *
 * Area, not triangle count, is the measure. The extruder tessellates every strip
 * identically, so a 0.15 m curb top costs exactly as many triangles as a 3.5 m
 * lane and curb geometry outnumbers asphalt geometry on any kerbed profile. That
 * is correct behaviour, not a defect, and a triangle-count dominance check would
 * be asserting something false.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadDump
 * @endcode
 */

#include "framework.hpp"
#include "obj_dump.hpp"

#include "osm/parser.hpp"
#include "osm/road/road_network_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#ifndef STRATUM_TEST_DATA_DIR
#error "STRATUM_TEST_DATA_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

#ifndef STRATUM_TEST_DUMP_DIR
#error "STRATUM_TEST_DUMP_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::material_id_name;
using stratum::osm::ParsedOSMData;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::test::ObjDumpStats;
using stratum::test::write_obj;

/// Every fixture in tests/data, in README table order
constexpr const char* kAllFixtures[] = {
    "four_way.osm", "t_junction.osm", "cul_de_sac.osm", "roundabout.osm",
    "motorway_link.osm", "rural_track.osm", "bridge_over.osm",
    "bridge_abutment.osm", "duplicate_node.osm",
};

/**
 * @brief Fixtures whose ways are kerbed urban classes with sidewalk=both
 *
 * These are the ones where a missing curb or sidewalk is a defect rather than
 * correct output. A motorway has no curb by design and a track has no sidewalk.
 */
constexpr const char* kKerbedFixtures[] = {
    "four_way.osm", "t_junction.osm", "cul_de_sac.osm",
};

/// Parse one fixture with roads only
std::optional<ParsedOSMData> parse_fixture(const char* filename) {
    const auto path = std::filesystem::path(STRATUM_TEST_DATA_DIR) / filename;
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

/**
 * @brief Build one fixture's network and write it to build/road_dump
 *
 * @param filename Fixture file name in tests/data
 * @param stats    Receives the counts of what was written
 * @param path_out Receives the .obj path
 * @return True when the fixture parsed, built, and wrote
 */
bool dump_fixture(const char* filename, ObjDumpStats& stats, std::filesystem::path& path_out) {
    const std::optional<ParsedOSMData> data = parse_fixture(filename);
    if (!data) return false;

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*data);

    std::vector<const Mesh*> meshes;
    meshes.reserve(network.pieces.size());
    for (const auto& piece : network.pieces) meshes.push_back(&piece.mesh);

    path_out = std::filesystem::path(STRATUM_TEST_DUMP_DIR) /
               (std::filesystem::path(filename).stem().string() + ".obj");

    std::string error;
    if (!write_obj(meshes, path_out, &stats, &error)) {
        stratum::test::report_failure(__FILE__, __LINE__, "write_obj(fixture)",
                                      std::string{filename} + ": " + error);
        return false;
    }

    // The dumper drops degenerate and out-of-range faces silently, so a mismatch
    // here means the extruder emitted triangles the file does not contain.
    if (stats.triangles != network.stats.triangles) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "every emitted triangle reaches the OBJ",
            std::string{filename} + ": wrote " + std::to_string(stats.triangles) + " of " +
                std::to_string(network.stats.triangles));
    }
    return true;
}

/// One row of the printed table
void print_row(const std::string& fixture, const ObjDumpStats& s) {
    std::cout << "  " << std::left << std::setw(20) << fixture << std::right << std::setw(8)
              << s.vertices << std::setw(8) << s.triangles << "   ";

    bool first = true;
    for (size_t i = 0; i < static_cast<size_t>(MaterialId::Count); ++i) {
        const auto material = static_cast<MaterialId>(i);
        if (s.count(material) == 0) continue;
        if (!first) std::cout << ", ";
        first = false;
        std::cout << material_id_name(material) << ' ' << s.count(material) << " ("
                  << std::fixed << std::setprecision(1) << s.area(material) << " m2)";
    }
    std::cout << std::defaultfloat << '\n';
}

} // namespace

// ============================================================================
// The dump
// ============================================================================

TEST(RoadDump, every_fixture_writes_an_obj_grouped_by_material) {
    // Every fixture is built and written first, then the table is printed in one
    // block. The parser and the builder log to spdlog as they run, so a row
    // printed as it is produced ends up interleaved with log lines and the table
    // is unreadable, which is the one thing this test exists to produce.
    std::vector<std::pair<std::string, ObjDumpStats>> rows;
    std::vector<std::string> paths;

    for (const char* fixture : kAllFixtures) {
        ObjDumpStats stats;
        std::filesystem::path path;
        if (!dump_fixture(fixture, stats, path)) continue;

        rows.emplace_back(fixture, stats);
        paths.push_back(path.string());

        // A fixture that writes nothing has nothing to look at, which defeats the
        // point of the dump.
        if (stats.triangles == 0 || stats.vertices == 0) {
            stratum::test::report_failure(__FILE__, __LINE__, "fixture produced geometry",
                                          std::string{fixture} + ": empty network");
        }
        if (!std::filesystem::exists(path)) {
            stratum::test::report_failure(__FILE__, __LINE__, "obj file exists", path.string());
        }
    }

    std::cout << "\n  road network dump -> " << STRATUM_TEST_DUMP_DIR << "\n\n"
              << "  " << std::left << std::setw(20) << "fixture" << std::right << std::setw(8)
              << "verts" << std::setw(8) << "tris" << "   triangles per material (area)\n";
    for (const auto& [fixture, stats] : rows) print_row(fixture, stats);

    std::cout << '\n';
    for (const std::string& path : paths) std::cout << "  " << path << '\n';
    std::cout << '\n';
}

// ============================================================================
// Material mix
// ============================================================================

TEST(RoadDump, kerbed_fixtures_grow_asphalt_curb_and_sidewalk) {
    for (const char* fixture : kKerbedFixtures) {
        ObjDumpStats stats;
        std::filesystem::path path;
        if (!dump_fixture(fixture, stats, path)) continue;

        const std::string label{fixture};

        // The three that say the profile builder ran at all. A ribbon with none of
        // them is exactly the output this phase replaces.
        if (stats.count(MaterialId::Asphalt) == 0) {
            stratum::test::report_failure(__FILE__, __LINE__, "kerbed fixture has asphalt",
                                          label + ": no Asphalt geometry");
        }
        if (stats.count(MaterialId::Curb) == 0) {
            stratum::test::report_failure(__FILE__, __LINE__, "kerbed fixture has a curb",
                                          label + ": no Curb geometry");
        }
        if (stats.count(MaterialId::Sidewalk) == 0) {
            stratum::test::report_failure(__FILE__, __LINE__, "kerbed fixture has a sidewalk",
                                          label + ": no Sidewalk geometry");
        }

        // Asphalt is the carriageway, so it must be the largest surface by area.
        // By triangle count it is not, and must not be asserted to be: every strip
        // is tessellated alike, so four curb strips out-triangle two lanes.
        double largest = 0.0;
        const char* largest_name = "none";
        for (size_t i = 0; i < static_cast<size_t>(MaterialId::Count); ++i) {
            const auto material = static_cast<MaterialId>(i);
            if (stats.area(material) > largest) {
                largest = stats.area(material);
                largest_name = material_id_name(material);
            }
        }
        if (stats.area(MaterialId::Asphalt) < largest) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "asphalt is the largest surface by area",
                label + ": " + largest_name + " covers more than Asphalt");
        }

        // Sidewalk is 2.0 m a side against 7.0 m of carriageway, so it is a large
        // fraction of the asphalt but never larger than it.
        const double asphalt = stats.area(MaterialId::Asphalt);
        const double sidewalk = stats.area(MaterialId::Sidewalk);
        if (asphalt > 0.0 && !(sidewalk > asphalt * 0.2 && sidewalk < asphalt)) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "sidewalk area is a plausible fraction of the carriageway",
                label + ": sidewalk " + std::to_string(sidewalk) + " m2 against asphalt " +
                    std::to_string(asphalt) + " m2");
        }
    }
}

TEST(RoadDump, the_rural_track_is_unpaved_and_verged_with_no_curb) {
    ObjDumpStats stats;
    std::filesystem::path path;
    if (!dump_fixture("rural_track.osm", stats, path)) return;

    // surface=gravel, so the running surface is Gravel and never Asphalt.
    CHECK_TRUE(stats.count(MaterialId::Gravel) > size_t{0});
    CHECK_EQ(stats.count(MaterialId::Asphalt), size_t{0});

    // The plan's rural cross-section is verge, running surface, verge.
    CHECK_TRUE(stats.count(MaterialId::Grass) > size_t{0});

    // A track has no kerb and no footway.
    CHECK_EQ(stats.count(MaterialId::Curb), size_t{0});
    CHECK_EQ(stats.count(MaterialId::Sidewalk), size_t{0});
}

TEST(RoadDump, the_motorway_has_shoulders_but_no_curb_or_sidewalk) {
    ObjDumpStats stats;
    std::filesystem::path path;
    if (!dump_fixture("motorway_link.osm", stats, path)) return;

    CHECK_TRUE(stats.count(MaterialId::Asphalt) > size_t{0});
    CHECK_EQ(stats.count(MaterialId::Curb), size_t{0});
    CHECK_EQ(stats.count(MaterialId::Sidewalk), size_t{0});
}
