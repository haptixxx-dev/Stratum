/**
 * @file test_junction_dump.cpp
 * @brief Writes the P4 junction geometry to OBJ and reports what the solver decided
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Three jobs, and only the third is an assertion.
 *
 * 1. **Junction-only dump.** `build/road_dump/<fixture>_junctions.obj` holds the
 *    solved junction meshes and nothing else. `<fixture>.obj`, written by the
 *    RoadDump suite, already contains junction geometry mixed in with the ribbons
 *    -- RoadNetworkConfig::solve_junctions defaults to true, so the junction
 *    pieces are in the piece list -- and mixed in is exactly the problem. A fill
 *    that is a few centimetres too small hides between two ribbons; the same fill
 *    on its own next to the combined file is obvious. Open the two together.
 *
 * 2. **Ring dump.** For the four-way and the roundabout,
 *    `build/road_dump/<fixture>_junction_rings.obj` holds the DECISIONS rather
 *    than the geometry, as `l` elements only: the junction polygon ring, the curb
 *    ring's inner and outer boundaries, and one line per arm across its cut
 *    cross-section. Those four objects are the trim distance and the fillet
 *    radius made visible. A fill can look plausible while its ring is wrong; the
 *    ring file is where that shows.
 *
 * 3. **Stats and sanity.** One row per fixture: junctions, roundabouts, tapers,
 *    dead ends, degenerate nodes, over-trimmed edges, and the share of the
 *    network's triangles that is junction geometry. The counts the fixtures were
 *    authored to produce are asserted -- four_way is one junction, roundabout is
 *    one roundabout, t_junction is one junction of degree three -- and a
 *    degenerate or over-trimmed junction is reported as a failure rather than
 *    left as a number in a table for nobody to read.
 *
 * A fourth, non-asserting job rides along: the junction solve is timed, on the
 * largest fixture and on a synthetic street grid, because it runs on the import
 * path. Timings are printed, never asserted -- a wall-clock threshold in a test
 * suite fails on a loaded machine and teaches everyone to ignore it.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests JunctionDump
 * @endcode
 */

#include "framework.hpp"
#include "obj_dump.hpp"

#include "osm/parser.hpp"
#include "osm/road/junction_builder.hpp"
#include "osm/road/road_network_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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
using stratum::osm::road::Junction;
using stratum::osm::road::JunctionKind;
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

/// Fixtures whose junction rings are dumped as lines
constexpr const char* kRingFixtures[] = {"four_way.osm", "roundabout.osm"};

// ============================================================================
// Fixture plumbing
// ============================================================================

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
 * @brief One fixture built to a network, kept whole
 *
 * The builder owns the graph and the centerlines the network refers to, so it has
 * to outlive the network. Holding both together is what lets a caller ask a
 * junction for its arms after build() has returned.
 */
struct Built {
    RoadNetworkBuilder builder;
    RoadNetwork network;
    bool ok = false;
};

/// Parse and build one fixture; `ok` is false when the fixture did not parse
Built build_fixture(const char* filename) {
    Built out;
    const std::optional<ParsedOSMData> data = parse_fixture(filename);
    if (!data) return out;

    out.network = out.builder.build(*data);
    out.ok = true;
    return out;
}

// ============================================================================
// Line OBJ
// ============================================================================

/**
 * @brief One named polyline or ring, in 2D local metres at a fixed height
 *
 * `closed` repeats the first index at the end of the `l` element rather than
 * duplicating the vertex, so a viewer that measures the ring's perimeter gets the
 * right answer.
 */
struct LineRing {
    std::string name;
    std::vector<glm::dvec2> points;
    double height = 0.0;
    bool closed = true;
};

/**
 * @brief Write rings to a line-only Wavefront OBJ
 *
 * `v` and `l` elements only: no faces, no normals, no material. A viewer draws
 * them as wireframe over the solid dump, which is the whole point -- these are
 * the solver's decisions, not its output.
 *
 * The 2D-to-world mapping is the pipeline's own, `(x, y) -> (x, height, -y)`,
 * with the SECOND local axis negated into Z. Writing `(x, height, y)` instead
 * mirrors every ring about the road and lines them up with nothing, and the
 * mirror is invisible on a symmetric fixture, so it is spelled out here rather
 * than left to a reader to assume.
 *
 * @param rings Rings to write; empty rings are skipped
 * @param path  Destination .obj path. Parent directories are created.
 * @param error Optional; receives a message when the write fails
 * @return True when the file was written
 */
bool write_obj_lines(const std::vector<LineRing>& rings, const std::filesystem::path& path,
                     std::string* error = nullptr) {
    std::error_code ec;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path(), ec);
    }

    std::ofstream file(path);
    if (!file) {
        if (error) *error = "cannot open " + path.string();
        return false;
    }

    file << "# Stratum junction rings: solver decisions, not geometry\n"
         << "# local (x, y) -> world (x, height, -y)\n";
    file << std::fixed << std::setprecision(6);

    size_t base = 1;
    for (const LineRing& ring : rings) {
        if (ring.points.size() < 2) continue;

        file << "o " << ring.name << '\n';
        for (const glm::dvec2& p : ring.points) {
            file << "v " << p.x << ' ' << ring.height << ' ' << -p.y << '\n';
        }

        file << 'l';
        for (size_t i = 0; i < ring.points.size(); ++i) {
            file << ' ' << (base + i);
        }
        if (ring.closed) file << ' ' << base;
        file << '\n';

        base += ring.points.size();
    }

    if (!file.good()) {
        if (error) *error = "write failed for " + path.string();
        return false;
    }
    return true;
}

/// Collect every ring a fixture's junctions decided, ready for write_obj_lines
std::vector<LineRing> rings_of(const RoadNetwork& network) {
    std::vector<LineRing> rings;

    for (size_t i = 0; i < network.junctions.size(); ++i) {
        const Junction& j = network.junctions[i];
        const std::string tag = std::to_string(i) + "_node" + std::to_string(j.node);
        const double height = static_cast<double>(j.height);

        if (j.polygon.valid && j.polygon.ring.size() >= 3) {
            rings.push_back({"polygon_" + tag, j.polygon.ring, height, true});
        }
        if (j.curb.inner.size() >= 3) {
            rings.push_back({"curb_inner_" + tag, j.curb.inner, height, true});
        }
        if (j.curb.outer.size() >= 3) {
            rings.push_back({"curb_outer_" + tag, j.curb.outer, height, true});
        }
        if (j.footprint.size() >= 3) {
            rings.push_back({"footprint_" + tag, j.footprint, height, true});
        }

        // One open line per arm, across its cut cross-section, from the full
        // profile's right corner to its left. This is where the trim landed, so
        // the gap between it and the polygon edge is the clearance and the gap
        // between it and the ribbon end is the error.
        for (size_t k = 0; k < j.ends.size(); ++k) {
            if (!j.ends[k].valid) continue;
            rings.push_back({"cut_" + tag + "_arm" + std::to_string(k),
                             {j.ends[k].right, j.ends[k].carriage_right,
                              j.ends[k].carriage_left, j.ends[k].left},
                             height, false});
        }
    }

    return rings;
}

// ============================================================================
// Stats
// ============================================================================

/// One row of the printed table
struct Row {
    std::string fixture;
    size_t junctions = 0;
    size_t roundabouts = 0;
    size_t tapers = 0;
    size_t dead_ends = 0;
    size_t degenerate = 0;
    size_t over_trimmed = 0;
    size_t self_intersecting = 0;
    size_t junction_triangles = 0;
    size_t fill_triangles = 0;      ///< Intersection and roundabout fills plus their curb rings
    size_t cap_triangles = 0;       ///< Dead-end caps and degree-2 tapers

    /**
     * @brief Triangles the junction SOLVER produced, before the per-piece finish
     *
     * The denominator the kind split is checked against. junction_triangles is
     * counted from the pieces that reached the network, and the P7 finish is
     * allowed to remove triangles from those -- merge_coplanar_quads() collapses
     * a flat fill -- so the split can only be reconciled against the meshes the
     * split itself is read from.
     */
    size_t solver_triangles = 0;

    /**
     * @brief Kerbed intersections that came back with no corner sidewalk at all
     *
     * An intersection whose arms carry sidewalks, whose junction polygon is valid,
     * and whose CurbRing produced inner and outer boundaries but not one triangle:
     * every corner section was swallowed by the arm mouths or fell under
     * kMinSectionLength. The junction is then a bare carriageway slab with the
     * arms' sidewalks stopping dead at its edge.
     */
    size_t intersections_without_curb = 0;

    /// Kerbed intersections examined, so the count above has a denominator
    size_t kerbed_intersections = 0;
    size_t total_triangles = 0;
    double junction_ms = 0.0;
    double build_ms = 0.0;

    [[nodiscard]] double junction_share() const {
        return (total_triangles > 0)
                   ? (static_cast<double>(junction_triangles) /
                      static_cast<double>(total_triangles))
                   : 0.0;
    }
};

/**
 * @brief Triangles emitted by junction pieces
 *
 * A junction piece is one with no single owning edge, which RoadPiece documents
 * as `edge == kInvalidId`. Counted from the pieces rather than from
 * Junction::mesh so the number is the one that actually reached the network.
 */
size_t junction_triangles_of(const RoadNetwork& network) {
    size_t total = 0;
    for (const auto& piece : network.pieces) {
        if (piece.edge != stratum::osm::road::kInvalidId) continue;
        total += piece.mesh.indices.size() / 3;
    }
    return total;
}

Row row_of(const char* fixture, const RoadNetwork& network) {
    Row r;
    r.fixture = fixture;
    r.junctions = network.junction_stats.junctions;
    r.roundabouts = network.junction_stats.roundabouts;
    r.tapers = network.junction_stats.tapers;
    r.dead_ends = network.junction_stats.dead_ends;
    r.degenerate = network.junction_stats.degenerate;
    r.over_trimmed = network.junction_stats.over_trimmed_edges;
    r.self_intersecting = network.junction_stats.self_intersecting;
    r.junction_triangles = junction_triangles_of(network);
    r.total_triangles = network.stats.triangles;

    // The share of triangles that is "junction geometry" lumps two very different
    // things together, and read as one number it is misleading: a fixture with
    // four cul-de-sac caps and one crossroads reports a large share of which the
    // crossroads is the smaller part. Split by kind so the table says which.
    for (const Junction& j : network.junctions) {
        const size_t tris = j.mesh.indices.size() / 3;
        r.solver_triangles += tris;
        switch (j.kind) {
            case JunctionKind::Intersection:
            case JunctionKind::Roundabout:
                r.fill_triangles += tris;
                break;
            case JunctionKind::Taper:
            case JunctionKind::DeadEnd:
                r.cap_triangles += tris;
                break;
            case JunctionKind::Degenerate:
                break;
        }

        // An intersection is "kerbed" here when the offsetter produced a ring at
        // all: build_curb_ring() fills inner and outer before it sweeps any
        // geometry, and returns them even when every section collapses, precisely
        // so the footprint survives. So inner non-empty with an empty mesh is the
        // signature of a junction that wanted a corner sidewalk and got none.
        if (j.kind == JunctionKind::Intersection && j.polygon.valid &&
            j.curb.inner.size() >= 3) {
            ++r.kerbed_intersections;
            if (j.curb.mesh.indices.empty()) ++r.intersections_without_curb;
        }
    }
    r.junction_ms = network.stats.junction_ms;
    r.build_ms = network.stats.build_ms;
    return r;
}

/// Junctions of a given kind
size_t count_kind(const RoadNetwork& network, JunctionKind kind) {
    size_t n = 0;
    for (const Junction& j : network.junctions) {
        if (j.kind == kind) ++n;
    }
    return n;
}

// ============================================================================
// Synthetic grid
// ============================================================================

/**
 * @brief A street grid of `side` by `side` blocks, as parsed OSM data
 *
 * Every fixture in tests/data is a handful of ways, so a timing taken on the
 * largest of them measures the parser's warm-up and not the solve. This produces
 * a network with `side * side` interior four-way junctions from `2 * side + 2`
 * ways, which is the shape the junction solver actually meets on an import.
 *
 * Node ids are laid out on a grid so a crossing really does share a node: that
 * shared identity is what makes a junction, and generating the two directions
 * independently would produce a network with no junctions at all and a timing of
 * zero.
 *
 * @param side   Streets in each direction; junctions are (side - 2)^2 interior
 * @param spacing Metres between parallel streets
 * @return Parsed data carrying only roads
 */
ParsedOSMData make_grid(int side, double spacing) {
    ParsedOSMData data;

    const auto node_id = [side](int ix, int iy) {
        return static_cast<stratum::osm::NodeId>(1 + iy * side + ix);
    };

    stratum::osm::WayId way = 1;

    for (int iy = 0; iy < side; ++iy) {
        stratum::osm::Road road;
        road.osm_id = way++;
        road.type = stratum::osm::RoadType::Residential;
        road.lanes = 2;
        road.width = 7.0f;
        for (int ix = 0; ix < side; ++ix) {
            road.polyline.push_back({static_cast<double>(ix) * spacing,
                                     static_cast<double>(iy) * spacing});
            road.node_ids.push_back(node_id(ix, iy));
        }
        data.roads.push_back(std::move(road));
    }

    for (int ix = 0; ix < side; ++ix) {
        stratum::osm::Road road;
        road.osm_id = way++;
        road.type = stratum::osm::RoadType::Residential;
        road.lanes = 2;
        road.width = 7.0f;
        for (int iy = 0; iy < side; ++iy) {
            road.polyline.push_back({static_cast<double>(ix) * spacing,
                                     static_cast<double>(iy) * spacing});
            road.node_ids.push_back(node_id(ix, iy));
        }
        data.roads.push_back(std::move(road));
    }

    return data;
}

} // namespace

// ============================================================================
// The dump
// ============================================================================

TEST(JunctionDump, every_fixture_writes_a_junction_obj_and_a_stats_row) {
    // Built and dumped first, printed second: the parser and the builder both log
    // to spdlog while they run, and a row printed as it is produced lands in the
    // middle of a log line. The table is the output of this test.
    std::vector<Row> rows;
    std::vector<std::string> paths;

    for (const char* fixture : kAllFixtures) {
        Built built = build_fixture(fixture);
        if (!built.ok) continue;

        rows.push_back(row_of(fixture, built.network));

        std::vector<const Mesh*> meshes;
        for (const auto& piece : built.network.pieces) {
            if (piece.edge != stratum::osm::road::kInvalidId) continue;
            meshes.push_back(&piece.mesh);
        }

        const auto path = std::filesystem::path(STRATUM_TEST_DUMP_DIR) /
                          (std::filesystem::path(fixture).stem().string() + "_junctions.obj");

        if (meshes.empty()) {
            // rural_track has no node of degree three. Writing an empty file
            // would be worse than not writing one: a stale file from a previous
            // run would be read as this run's output.
            std::error_code ec;
            std::filesystem::remove(path, ec);
            continue;
        }

        ObjDumpStats stats;
        std::string error;
        if (!write_obj(meshes, path, &stats, &error)) {
            stratum::test::report_failure(__FILE__, __LINE__, "write_obj(junctions)",
                                          std::string{fixture} + ": " + error);
            continue;
        }
        paths.push_back(path.string());

        // The dumper drops degenerate and out-of-range faces silently, so a
        // shortfall here is junction triangles the file does not contain.
        if (stats.triangles != rows.back().junction_triangles) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "every junction triangle reaches the OBJ",
                std::string{fixture} + ": wrote " + std::to_string(stats.triangles) + " of " +
                    std::to_string(rows.back().junction_triangles));
        }
    }

    // ---- the table ------------------------------------------------------
    std::cout << "\n  junction solve -> " << STRATUM_TEST_DUMP_DIR << "\n\n"
              << "  " << std::left << std::setw(20) << "fixture" << std::right << std::setw(6)
              << "junc" << std::setw(6) << "rndb" << std::setw(7) << "taper" << std::setw(7)
              << "dead" << std::setw(7) << "degen" << std::setw(8) << "overtr" << std::setw(8)
              << "selfx" << std::setw(10) << "nocurb" << std::setw(9) << "fill_tri"
              << std::setw(8) << "cap_tri" << std::setw(9) << "junc_tri" << std::setw(9)
              << "tri" << std::setw(9) << "junc_%" << '\n';

    for (const Row& r : rows) {
        std::cout << "  " << std::left << std::setw(20) << r.fixture << std::right
                  << std::setw(6) << r.junctions << std::setw(6) << r.roundabouts
                  << std::setw(7) << r.tapers << std::setw(7) << r.dead_ends << std::setw(7)
                  << r.degenerate << std::setw(8) << r.over_trimmed << std::setw(8)
                  << r.self_intersecting
                  << std::setw(10)
                  << (std::to_string(r.intersections_without_curb) + "/" +
                      std::to_string(r.kerbed_intersections))
                  << std::setw(9) << r.fill_triangles
                  << std::setw(8) << r.cap_triangles << std::setw(9) << r.junction_triangles
                  << std::setw(9) << r.total_triangles << std::setw(8) << std::fixed
                  << std::setprecision(1) << (r.junction_share() * 100.0) << "%"
                  << std::defaultfloat << '\n';
    }

    std::cout << '\n';
    for (const std::string& path : paths) std::cout << "  " << path << '\n';
    std::cout << '\n';

    // ---- the loud NON-failure --------------------------------------------
    // Not asserted, because it is a known quality gap in the curb ring rather
    // than a regression, and a red suite that nobody can turn green teaches
    // people to ignore red suites. Printed where it cannot be missed instead of
    // left as a column somebody has to notice.
    size_t no_curb_total = 0;
    size_t kerbed_total = 0;
    for (const Row& r : rows) {
        no_curb_total += r.intersections_without_curb;
        kerbed_total += r.kerbed_intersections;
    }
    if (no_curb_total > 0) {
        std::cout << "  ** " << no_curb_total << " of " << kerbed_total
                  << " kerbed intersections produced NO corner sidewalk.\n"
                  << "     The junction polygon and the offset ring are both there; every\n"
                  << "     corner section was swallowed by an arm mouth or fell under\n"
                  << "     kMinSectionLength, so the arms' sidewalks stop dead at a bare\n"
                  << "     carriageway slab. Expect this where two arms leave at a shallow\n"
                  << "     angle: the corner between them is a sliver whatever the trim\n"
                  << "     reserves for it.\n\n";
    }

    // ---- the loud failures ----------------------------------------------
    // A degenerate junction is a node the solver gave up on: it emits no fill and
    // falls back to the provisional disc, so ribbons run through each other there
    // exactly as they did before P4. A self-intersecting ring was filled as a
    // convex hull, which covers the corners it should have cut. Neither is
    // acceptable on a fixture authored to be solvable, and neither is a number to
    // be skimmed past in the table above.
    for (const Row& r : rows) {
        if (r.degenerate > 0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "no fixture has a degenerate junction",
                r.fixture + ": " + std::to_string(r.degenerate) +
                    " node(s) fell back to the provisional disc, so ribbons still cross there");
        }
        if (r.self_intersecting > 0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "no junction ring crosses itself",
                r.fixture + ": " + std::to_string(r.self_intersecting) +
                    " ring(s) filled as a convex hull instead of as a fillet polygon");
        }
        if (r.over_trimmed > 0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "no edge is over-trimmed",
                r.fixture + ": " + std::to_string(r.over_trimmed) +
                    " edge(s) hit the max_trim_fraction clamp, so the junction polygon "
                    "overlaps that ribbon");
        }
    }

    // The split has to account for every junction triangle, or the table's two
    // detail columns and its total describe different things.
    for (const Row& r : rows) {
        if (r.fill_triangles + r.cap_triangles != r.solver_triangles) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the kind split accounts for every junction triangle",
                r.fixture + ": fill " + std::to_string(r.fill_triangles) + " + cap " +
                    std::to_string(r.cap_triangles) + " != " +
                    std::to_string(r.solver_triangles));
        }
    }

    // A run that dumped nothing has proved nothing.
    CHECK_TRUE(!paths.empty());
}

// ============================================================================
// Fixture expectations
// ============================================================================

/**
 * The three fixtures whose junction counts are the reason they exist. A four-way
 * that reports two junctions has split a node; one that reports none has lost the
 * shared node identity P1 is built on.
 */
TEST(JunctionDump, the_authored_fixtures_solve_the_junctions_they_were_authored_for) {
    // ---- four_way: exactly one intersection -----------------------------
    {
        const Built built = build_fixture("four_way.osm");
        if (built.ok) {
            CHECK_EQ(built.network.junction_stats.junctions, size_t{1});
            CHECK_EQ(built.network.junction_stats.roundabouts, size_t{0});
            CHECK_EQ(count_kind(built.network, JunctionKind::Intersection), size_t{1});

            for (const Junction& j : built.network.junctions) {
                if (j.kind != JunctionKind::Intersection) continue;
                CHECK_EQ(j.arms.size(), size_t{4});
                CHECK_TRUE(j.polygon.valid);
                CHECK_TRUE(!j.mesh.indices.empty());
            }
        }
    }

    // ---- roundabout: exactly one loop -----------------------------------
    {
        const Built built = build_fixture("roundabout.osm");
        if (built.ok) {
            CHECK_EQ(built.network.junction_stats.roundabouts, size_t{1});
            CHECK_EQ(count_kind(built.network, JunctionKind::Roundabout), size_t{1});

            for (const Junction& j : built.network.junctions) {
                if (j.kind != JunctionKind::Roundabout) continue;
                CHECK_TRUE(j.is_roundabout);
                CHECK_TRUE(!j.mesh.indices.empty());
                CHECK_TRUE(j.footprint.size() >= 3);
            }
        }
    }

    // ---- t_junction: exactly one intersection, of degree three ----------
    {
        const Built built = build_fixture("t_junction.osm");
        if (built.ok) {
            CHECK_EQ(built.network.junction_stats.junctions, size_t{1});
            CHECK_EQ(count_kind(built.network, JunctionKind::Intersection), size_t{1});

            size_t degree_three = 0;
            for (const Junction& j : built.network.junctions) {
                if (j.kind != JunctionKind::Intersection) continue;
                if (j.arms.size() == 3) ++degree_three;
                // The T's shared node is INTERIOR to the through way, which is the
                // case proximity clustering missed and node identity catches.
                CHECK_EQ(j.arms.size(), size_t{3});
                CHECK_EQ(j.ends.size(), j.arms.size());
            }
            CHECK_EQ(degree_three, size_t{1});
        }
    }
}

// ============================================================================
// Ring dump
// ============================================================================

TEST(JunctionDump, the_four_way_and_the_roundabout_write_their_rings_as_lines) {
    std::vector<std::string> paths;

    for (const char* fixture : kRingFixtures) {
        const Built built = build_fixture(fixture);
        if (!built.ok) continue;

        const std::vector<LineRing> rings = rings_of(built.network);
        if (rings.empty()) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "fixture produced junction rings to dump",
                                          std::string{fixture} + ": no rings");
            continue;
        }

        const auto path =
            std::filesystem::path(STRATUM_TEST_DUMP_DIR) /
            (std::filesystem::path(fixture).stem().string() + "_junction_rings.obj");

        std::string error;
        if (!write_obj_lines(rings, path, &error)) {
            stratum::test::report_failure(__FILE__, __LINE__, "write_obj_lines(rings)",
                                          std::string{fixture} + ": " + error);
            continue;
        }

        CHECK_TRUE(std::filesystem::exists(path));
        paths.push_back(path.string() + "  (" + std::to_string(rings.size()) + " rings)");
    }

    std::cout << "\n  junction rings, line-only OBJ\n\n";
    for (const std::string& path : paths) std::cout << "  " << path << '\n';
    std::cout << '\n';

    CHECK_EQ(paths.size(), size_t{2});
}

// ============================================================================
// Cost
// ============================================================================

/**
 * The junction solve runs on the import path, so its cost is reported rather than
 * left to be discovered on a city-sized extract.
 *
 * Nothing here is asserted. A wall-clock bound in a unit suite fails on a loaded
 * machine, gets marked flaky, and then hides the regression it was added to catch.
 * The numbers are printed for a human to read.
 */
TEST(JunctionDump, the_junction_solve_cost_is_reported) {
    // ---- the largest fixture --------------------------------------------
    // "Largest" by the triangles the network produced, not by file size: a fixture
    // can be verbose in XML and trivial in geometry.
    std::string biggest;
    size_t biggest_triangles = 0;
    Row biggest_row;

    for (const char* fixture : kAllFixtures) {
        const Built built = build_fixture(fixture);
        if (!built.ok) continue;
        if (built.network.stats.triangles <= biggest_triangles) continue;
        biggest_triangles = built.network.stats.triangles;
        biggest = fixture;
        biggest_row = row_of(fixture, built.network);
    }

    std::cout << "\n  junction solve cost\n\n";

    if (!biggest.empty()) {
        // One measurement of a sub-millisecond solve is mostly scheduler noise, so
        // the fixture is rebuilt a few times and the best run reported: the best
        // run is the one least contaminated by whatever else the machine was doing.
        const std::optional<ParsedOSMData> data = parse_fixture(biggest.c_str());
        double best_junction = biggest_row.junction_ms;
        double best_build = biggest_row.build_ms;

        if (data) {
            for (int i = 0; i < 8; ++i) {
                RoadNetworkBuilder builder;
                const RoadNetwork network = builder.build(*data);
                best_junction = std::min(best_junction, network.stats.junction_ms);
                best_build = std::min(best_build, network.stats.build_ms);
            }
        }

        std::cout << "  largest fixture: " << biggest << "  (" << biggest_triangles
                  << " triangles, " << biggest_row.junctions << " junctions, "
                  << biggest_row.roundabouts << " roundabouts)\n"
                  << "    junction solve " << std::fixed << std::setprecision(3)
                  << best_junction << " ms of " << best_build << " ms build ("
                  << std::setprecision(1)
                  << ((best_build > 0.0) ? (best_junction / best_build * 100.0) : 0.0)
                  << "% of build)\n"
                  << std::defaultfloat;
    }

    // ---- a synthetic grid -----------------------------------------------
    // The fixtures are a handful of ways each, so the number above says more about
    // start-up than about the solve. A grid gives it something to do.
    std::cout << "\n  synthetic street grid (Residential, 2 lanes, 120 m spacing)\n\n"
              << "  " << std::left << std::setw(10) << "grid" << std::right << std::setw(8)
              << "ways" << std::setw(8) << "junc" << std::setw(10) << "tris"
              << std::setw(12) << "junction" << std::setw(12) << "build"
              << std::setw(10) << "share" << '\n';

    for (const int side : {6, 12, 24}) {
        const ParsedOSMData grid = make_grid(side, 120.0);

        double best_junction = 1e300;
        double best_build = 1e300;
        size_t junctions = 0;
        size_t triangles = 0;

        for (int i = 0; i < 3; ++i) {
            RoadNetworkBuilder builder;
            const RoadNetwork network = builder.build(grid);
            best_junction = std::min(best_junction, network.stats.junction_ms);
            best_build = std::min(best_build, network.stats.build_ms);
            junctions = network.junction_stats.junctions;
            triangles = network.stats.triangles;
        }

        std::cout << "  " << std::left << std::setw(10)
                  << (std::to_string(side) + "x" + std::to_string(side)) << std::right
                  << std::setw(8) << grid.roads.size() << std::setw(8) << junctions
                  << std::setw(10) << triangles << std::setw(11) << std::fixed
                  << std::setprecision(3) << best_junction << " " << std::setw(11)
                  << best_build << " " << std::setw(9) << std::setprecision(1)
                  << ((best_build > 0.0) ? (best_junction / best_build * 100.0) : 0.0) << "%"
                  << std::defaultfloat << '\n';

        // Not a timing assertion: a grid of N streets each way must produce
        // (N-2)^2 interior four-way nodes, and if it does not the timing above was
        // measured on a network with no junctions in it.
        const size_t interior = static_cast<size_t>(side - 2) * static_cast<size_t>(side - 2);
        if (junctions < interior) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the timed grid really has junctions in it",
                std::to_string(side) + "x" + std::to_string(side) + ": " +
                    std::to_string(junctions) + " junctions, expected at least " +
                    std::to_string(interior));
        }
    }

    std::cout << '\n';
}
