/**
 * @file test_p5_p6_dump.cpp
 * @brief Writes the FULL road network -- paint, crossings and structures -- to OBJ
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The RoadDump suite writes what P2 and P4 produce: surfaces only, with every P5
 * and P6 pass at its default. That was the whole network when it was written and
 * it is not any more. A dash, a zebra, a deck underside, a parapet, a pier and a
 * portal are all invisible in those files, and every one of them is geometry no
 * numeric assertion can tell apart from a plausible one in the wrong place.
 *
 * So this suite does three things, and only the third is an assertion:
 *
 * 1. **`<fixture>_full.obj`.** Every fixture built with markings, crossings and
 *    structures ON, on rolling terrain so the elevation solve, the pier drop and
 *    the portal carve all actually run. One `usemtl` group per MaterialId, so a
 *    viewer shows Markings, BridgeDeck, Parapet and Concrete as their own objects
 *    and a stripe on the footway or a deck the wrong way up is visible at a
 *    glance.
 * 2. **`bridge_over_structure.obj` and `tunnel_structure.obj`.** The same two
 *    networks with every SURFACE material stripped, leaving only what P6 built.
 *    The deck underside is the one surface in this pipeline that is always hidden
 *    behind the thing standing on it -- the running surface above, the parapets
 *    beside it, the terrain below -- and separating it out is the only way to
 *    look at it, at the piers under it, and at the portal mouths.
 * 3. **The counts a reader would otherwise have to trust.** Four fixtures exist
 *    to produce one specific thing each, and a zero there is a silent failure of
 *    the pass rather than of the fixture: sidewalk_dup has exactly one side to
 *    suppress, turn_lanes exactly three arrows, tunnel exactly two portals, and
 *    bridge_over at least one bridge with a parapet on each side.
 *
 * The table is printed whether or not anything failed, because the numbers are
 * the deliverable and a passing run is when they are most worth reading.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests P5P6Dump
 * @endcode
 */

#include "framework.hpp"
#include "obj_dump.hpp"
#include "road/p5_p6_fixtures.hpp"

#include "osm/road/marking_atlas.hpp"
#include "osm/parser.hpp"
#include "osm/road/road_network_builder.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#ifndef STRATUM_TEST_DUMP_DIR
#error "STRATUM_TEST_DUMP_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::SubMesh;
using stratum::material_id_name;
using stratum::osm::road::EdgeId;
using stratum::osm::road::HeightSampler;
using stratum::osm::road::MarkingSprite;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;
using stratum::osm::road::kInvalidId;
using stratum::test::ObjDumpStats;
using stratum::test::write_obj;

namespace p5 = stratum::test::p5;
namespace jt = stratum::test::junction;

// ============================================================================
// Terrain and configuration
// ============================================================================

/**
 * @brief The same rolling ground the integration suite uses
 *
 * Deliberately identical to P5P6Integration::rolling(), so a fixture that looks
 * wrong in one of these files can be reasoned about against the assertions that
 * suite makes on the very same geometry. Flat ground would leave the grade solve,
 * the carve and every pier at one height, which is where a sign error hides.
 */
HeightSampler rolling() {
    return [](double x, double y) {
        return static_cast<float>(9.0 * std::sin(x / 130.0) + 6.0 * std::cos(y / 95.0) + 20.0);
    };
}

/// Markings, crossings, structures and dedup all on, on @p sampler
RoadNetworkConfig full_config(const HeightSampler& sampler) {
    RoadNetworkConfig cfg;
    cfg.solve_junctions = true;
    cfg.emit_markings = true;
    cfg.emit_crossings = true;
    cfg.emit_structures = true;
    cfg.dedup.enabled = true;
    cfg.height_sampler = sampler;
    return cfg;
}

/**
 * @brief A ridge laid over the covered way of tests/data/tunnel.osm
 *
 * rolling() has no hill anywhere near this fixture, and a tunnel with nothing
 * over it is not a tunnel: the file would show a road at grade with a tunnel tag
 * and nothing to look at. The fixture's own README says the hill has to come from
 * the caller, because a .osm extract carries no terrain. Sized exactly as
 * test_structures.cpp's load_tunnel() sizes it -- crest over the middle station,
 * half-length four fifths of the edge -- so the two agree about where the ground
 * is and a reader can compare the file against those assertions.
 *
 * Built from a throwaway first pass over the fixture, because the ridge has to be
 * positioned against the PARSED geometry and OSMParser recentres every
 * coordinate.
 *
 * @return The sampler, or rolling() when the fixture has no tunnel edge
 */
HeightSampler tunnel_ridge() {
    const p5::Network net = p5::make_network("tunnel.osm");
    if (!net.ok) return rolling();

    for (size_t i = 0; i < net.graph.edges().size(); ++i) {
        if (!net.graph.edges()[i].is_tunnel) continue;
        const auto& cl = net.centerlines[i];
        if (!cl.is_valid() || cl.stations.size() < 3) continue;

        const double span = cl.stations.back().arclength - cl.stations.front().arclength;
        const glm::dvec2 crest = cl.stations[cl.stations.size() / 2].position;
        const glm::dvec2 axis = cl.stations.front().tangent;
        return p5::ridge_sampler(crest, axis, 0.5 * span * 0.8, 10.0f, 25.0f);
    }
    return rolling();
}

/// The terrain a fixture is dumped over: its own ridge for the tunnel, rolling for the rest
HeightSampler terrain_for(const std::string& filename) {
    return filename == "tunnel.osm" ? tunnel_ridge() : rolling();
}

/// Materials the structure dump reports, in the order it prints them
constexpr MaterialId kStructureMaterials[] = {
    MaterialId::BridgeDeck,
    MaterialId::Parapet,
    MaterialId::Concrete,
};

// ============================================================================
// Mesh filtering
// ============================================================================

/**
 * @brief A copy of @p mesh holding only the connected components @p keep accepts
 *
 * Vertices are copied wholesale and the index buffer is rebuilt, so the result
 * carries unreferenced vertices. write_obj() writes them and no viewer minds; a
 * compaction pass here would only be a second thing to get wrong in a file whose
 * whole purpose is to be looked at.
 *
 * Components rather than submesh ranges, because the thing that has to be
 * separated here is not a material. MaterialId::Concrete is the corridor's GUTTER
 * strip as well as a pier and a portal wall, so a range filter cannot tell a
 * structure from the kerbside drainage channel running past it.
 *
 * @param mesh Source mesh
 * @param keep Predicate on one component of @p mesh
 * @return The filtered copy, with one submesh range per surviving material
 */
Mesh keep_components(const Mesh& mesh, const std::function<bool(const p5::Component&)>& keep) {
    std::vector<std::vector<size_t>> by_material(static_cast<size_t>(MaterialId::Count));
    for (const p5::Component& comp : p5::components_of(mesh)) {
        if (!keep(comp)) continue;
        const auto slot = static_cast<size_t>(comp.material);
        if (slot >= by_material.size()) continue;
        by_material[slot].insert(by_material[slot].end(), comp.triangles.begin(),
                                 comp.triangles.end());
    }

    Mesh out;
    out.vertices = mesh.vertices;
    for (size_t m = 0; m < by_material.size(); ++m) {
        std::vector<size_t>& tris = by_material[m];
        if (tris.empty()) continue;
        std::sort(tris.begin(), tris.end());

        const auto start = static_cast<uint32_t>(out.indices.size());
        for (size_t t : tris) {
            out.indices.push_back(mesh.indices[t * 3 + 0]);
            out.indices.push_back(mesh.indices[t * 3 + 1]);
            out.indices.push_back(mesh.indices[t * 3 + 2]);
        }
        out.submeshes.push_back(SubMesh{start,
                                        static_cast<uint32_t>(out.indices.size()) - start,
                                        static_cast<MaterialId>(m)});
    }
    return out;
}

/// Plan bounding box of a component, in 2D local metres
struct PlanBox {
    double x_lo = 1e300;
    double x_hi = -1e300;
    double y_lo = 1e300;
    double y_hi = -1e300;

    [[nodiscard]] bool overlaps(const PlanBox& o, double slack) const {
        return x_lo <= o.x_hi + slack && o.x_lo <= x_hi + slack && y_lo <= o.y_hi + slack &&
               o.y_lo <= y_hi + slack;
    }
};

PlanBox plan_box(const Mesh& mesh, const p5::Component& comp) {
    PlanBox box;
    for (const glm::dvec2& p : p5::locals_of(mesh, comp)) {
        box.x_lo = std::min(box.x_lo, p.x);
        box.x_hi = std::max(box.x_hi, p.x);
        box.y_lo = std::min(box.y_lo, p.y);
        box.y_hi = std::max(box.y_hi, p.y);
    }
    return box;
}

/**
 * @brief Is this Concrete component part of a pier stub?
 *
 * By PLAN SIZE, because the material slot cannot answer it. MaterialId::Concrete
 * carries three unrelated things in this pipeline: the corridor's gutter strip,
 * the junction apron, and P6's piers and portal walls. A pier is a box
 * BridgeConfig::pier_width square in plan, so every one of its six faces fits
 * inside a box twice that on a side. A gutter runs the whole length of an edge
 * and a junction apron wraps a whole junction, so neither does.
 *
 * A portal wall is deliberately NOT matched: it spans the full width of the
 * opening, some twelve metres, and is counted from RoadNetwork::carve_portals
 * instead, which is exact.
 *
 * @param box        Plan bounding box of the component
 * @param pier_width BridgeConfig::pier_width
 */
bool is_pier_shaped(const PlanBox& box, double pier_width) {
    const double limit = 2.0 * pier_width;
    return (box.x_hi - box.x_lo) <= limit && (box.y_hi - box.y_lo) <= limit;
}

/// True when any triangle of a component faces more sideways than up
bool has_a_vertical_face(const Mesh& mesh, const p5::Component& comp) {
    for (size_t t : comp.triangles) {
        const glm::dvec3 a(mesh.vertices[mesh.indices[t * 3 + 0]].position);
        const glm::dvec3 b(mesh.vertices[mesh.indices[t * 3 + 1]].position);
        const glm::dvec3 c(mesh.vertices[mesh.indices[t * 3 + 2]].position);
        const glm::dvec3 n = glm::cross(b - a, c - a);
        const double len = glm::length(n);
        if (len < 1e-12) continue;
        if (std::fabs(n.y) / len < 0.5) return true;
    }
    return false;
}

// ============================================================================
// Counting what came out
// ============================================================================

/// True when a sprite is one of the six turn arrows
bool is_arrow(MarkingSprite s) {
    return s == MarkingSprite::ArrowStraight || s == MarkingSprite::ArrowLeft ||
           s == MarkingSprite::ArrowRight || s == MarkingSprite::ArrowStraightLeft ||
           s == MarkingSprite::ArrowStraightRight || s == MarkingSprite::ArrowUTurn;
}

/**
 * @brief Count pier stubs on one bridge piece, by plan footprint
 *
 * Two steps, and the first is what makes the second mean anything.
 *
 * 1. Keep only the Concrete components that are pier-shaped; see is_pier_shaped().
 *    Without this the corridor's Concrete gutter strips are counted as piers, and
 *    because a gutter's plan box spans the whole edge it also swallows every real
 *    pier into one group.
 * 2. Merge what is left by overlapping plan boxes, transitively. A pier is a
 *    closed box whose six faces carry their own vertices, so vertex connectivity
 *    reports six components per pier; what makes them one pier is a shared
 *    footprint. Two piers are a bay apart and never merge.
 *
 * @param mesh       Mesh of one bridge piece
 * @param pier_width BridgeConfig::pier_width
 * @return Number of distinct pier footprints
 */
size_t count_pier_footprints(const Mesh& mesh, double pier_width) {
    std::vector<PlanBox> boxes;
    for (const p5::Component& comp : p5::components_of(mesh)) {
        if (comp.material != MaterialId::Concrete) continue;
        const PlanBox box = plan_box(mesh, comp);
        if (!is_pier_shaped(box, pier_width)) continue;
        boxes.push_back(box);
    }

    const double slack = 1e-3;
    std::vector<size_t> parent(boxes.size());
    for (size_t i = 0; i < parent.size(); ++i) parent[i] = i;
    const std::function<size_t(size_t)> find = [&](size_t v) {
        while (parent[v] != v) {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        return v;
    };
    for (size_t i = 0; i < boxes.size(); ++i) {
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (!boxes[i].overlaps(boxes[j], slack)) continue;
            const size_t ra = find(i);
            const size_t rb = find(j);
            if (ra != rb) parent[ra] = rb;
        }
    }

    size_t groups = 0;
    std::vector<bool> seen(boxes.size(), false);
    for (size_t i = 0; i < boxes.size(); ++i) {
        const size_t root = find(i);
        if (seen[root]) continue;
        seen[root] = true;
        ++groups;
    }
    return groups;
}

/// Everything the table reports for one fixture
struct Row {
    std::string fixture;
    ObjDumpStats obj;
    size_t markings_pieces = 0;
    size_t marking_quads = 0;
    size_t arrows = 0;
    size_t crossings = 0;
    size_t dropped_kerb_spans = 0;
    size_t bridges = 0;
    size_t piers = 0;
    size_t tunnels = 0;
    size_t portals = 0;
    size_t deduped_sidewalks = 0;
    double build_ms = 0.0;
    double elevation_ms = 0.0;
    double junction_ms = 0.0;
};

/**
 * @brief Build one fixture with everything on, write its OBJ, and tally it
 *
 * @param filename Fixture file name in tests/data
 * @param row      Receives the counts
 * @param path_out Receives the written .obj path
 * @param network  Receives the built network, for the callers that assert on it
 * @param builder  Receives the builder, which owns the graph and centerlines
 * @return True when the fixture parsed and built
 */
bool dump_full(const std::string& filename, Row& row, std::filesystem::path& path_out,
               RoadNetwork& network, RoadNetworkBuilder& builder) {
    const auto parsed = jt::parse_fixture(filename.c_str());
    if (!parsed) return false;

    const RoadNetworkConfig cfg = full_config(terrain_for(filename));
    network = builder.build(*parsed, cfg);

    row.fixture = filename;
    row.markings_pieces = network.stats.markings_pieces;
    row.crossings = network.stats.crossings;
    row.dropped_kerb_spans = network.stats.dropped_kerb_spans;
    row.bridges = network.stats.bridges;
    row.tunnels = network.stats.tunnels;
    row.portals = network.carve_portals.size();
    row.deduped_sidewalks = network.stats.deduped_sidewalks;
    row.build_ms = network.stats.build_ms;
    row.elevation_ms = network.stats.elevation_ms;
    row.junction_ms = network.stats.junction_ms;

    for (const RoadPiece& piece : network.pieces) {
        const bool is_bridge = piece.edge != kInvalidId &&
                               piece.edge < builder.graph().edges().size() &&
                               builder.graph().edges()[piece.edge].is_bridge;
        if (is_bridge) {
            row.piers += count_pier_footprints(piece.mesh, cfg.bridge.pier_width);
        }

        for (const p5::Component& comp : p5::components_of(piece.mesh)) {
            if (comp.material != MaterialId::Markings) continue;
            ++row.marking_quads;
            if (is_arrow(p5::sprite_of(piece.mesh, comp))) ++row.arrows;
        }
    }

    std::vector<const Mesh*> meshes;
    meshes.reserve(network.pieces.size());
    for (const RoadPiece& piece : network.pieces) meshes.push_back(&piece.mesh);

    path_out = std::filesystem::path(STRATUM_TEST_DUMP_DIR) /
               (std::filesystem::path(filename).stem().string() + "_full.obj");

    std::string error;
    if (!write_obj(meshes, path_out, &row.obj, &error)) {
        stratum::test::report_failure(__FILE__, __LINE__, "write_obj(full network)",
                                      filename + ": " + error);
        return false;
    }
    if (row.obj.triangles != network.stats.triangles) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "every emitted triangle reaches the OBJ",
            filename + ": wrote " + std::to_string(row.obj.triangles) + " of " +
                std::to_string(network.stats.triangles));
    }
    return true;
}

/**
 * @brief Write only the P6 structure geometry of an already-built network
 *
 * BridgeDeck and Parapet are kept wherever they appear: no other pass emits
 * either. MaterialId::Concrete is kept only from a piece built from a bridge or
 * tunnel edge, and then only from a component that is pier-shaped or carries a
 * vertical face -- a portal headwall and side wall. That is what leaves out the
 * corridor's Concrete GUTTER strip, which runs the whole length of every kerbed
 * edge including a bridge, and the junction apron and dead-end caps, which a
 * two-ended tunnel fixture has two of.
 *
 * @param network  Built network
 * @param builder  Its builder, for the graph the bridge and tunnel flags live on
 * @param cfg      The config it was built with, for BridgeConfig::pier_width
 * @param stem     Fixture stem; the file is written as `<stem>_structure.obj`
 * @param path_out Receives the written path
 * @param stats    Receives the counts of what was written
 * @return True when the file was written
 */
bool dump_structures(const RoadNetwork& network, const RoadNetworkBuilder& builder,
                     const RoadNetworkConfig& cfg, const std::string& stem,
                     std::filesystem::path& path_out, ObjDumpStats& stats) {
    const auto pier_width = static_cast<double>(cfg.bridge.pier_width);

    std::vector<Mesh> kept;
    kept.reserve(network.pieces.size());
    for (const RoadPiece& piece : network.pieces) {
        const bool structural = piece.edge != kInvalidId &&
                                piece.edge < builder.graph().edges().size() &&
                                (builder.graph().edges()[piece.edge].is_bridge ||
                                 builder.graph().edges()[piece.edge].is_tunnel);

        const Mesh& mesh = piece.mesh;
        Mesh only = keep_components(mesh, [&](const p5::Component& comp) {
            if (comp.material == MaterialId::BridgeDeck || comp.material == MaterialId::Parapet) {
                return true;
            }
            if (comp.material != MaterialId::Concrete || !structural) return false;
            return is_pier_shaped(plan_box(mesh, comp), pier_width) ||
                   has_a_vertical_face(mesh, comp);
        });
        if (!only.indices.empty()) kept.push_back(std::move(only));
    }

    std::vector<const Mesh*> meshes;
    meshes.reserve(kept.size());
    for (const Mesh& m : kept) meshes.push_back(&m);

    path_out = std::filesystem::path(STRATUM_TEST_DUMP_DIR) / (stem + "_structure.obj");

    std::string error;
    if (!write_obj(meshes, path_out, &stats, &error)) {
        stratum::test::report_failure(__FILE__, __LINE__, "write_obj(structures)",
                                      stem + ": " + error);
        return false;
    }
    return true;
}

/// One line of the counts table
void print_counts(const Row& r) {
    std::cout << "  " << std::left << std::setw(22) << r.fixture << std::right << std::setw(7)
              << r.markings_pieces << std::setw(8) << r.marking_quads << std::setw(8) << r.arrows
              << std::setw(7) << r.crossings << std::setw(8) << r.dropped_kerb_spans << std::setw(8)
              << r.bridges << std::setw(7) << r.piers << std::setw(8) << r.tunnels << std::setw(9)
              << r.portals << std::setw(8) << r.deduped_sidewalks << '\n';
}

/// One line of the per-material triangle table
void print_materials(const Row& r) {
    std::cout << "  " << std::left << std::setw(22) << r.fixture << std::right;
    for (size_t i = 0; i < static_cast<size_t>(MaterialId::Count); ++i) {
        std::cout << std::setw(10) << r.obj.count(static_cast<MaterialId>(i));
    }
    std::cout << '\n';
}

} // namespace

// ============================================================================
// The dump
// ============================================================================

TEST(P5P6Dump, every_fixture_writes_a_full_obj_with_paint_and_structures) {
    // Built and written first, printed afterwards in one block: the parser and
    // the builder log to spdlog as they run, and a row printed as it is produced
    // ends up interleaved with log lines, which ruins the one thing this suite
    // exists to produce.
    std::vector<Row> rows;
    std::vector<std::string> paths;

    for (const std::string& fixture : p5::all_fixtures()) {
        Row row;
        std::filesystem::path path;
        RoadNetwork network;
        RoadNetworkBuilder builder;
        if (!dump_full(fixture, row, path, network, builder)) continue;

        rows.push_back(row);
        paths.push_back(path.string());

        if (row.obj.triangles == 0 || row.obj.vertices == 0) {
            stratum::test::report_failure(__FILE__, __LINE__, "fixture produced geometry",
                                          fixture + ": empty network");
        }
        if (!std::filesystem::exists(path)) {
            stratum::test::report_failure(__FILE__, __LINE__, "obj file exists", path.string());
        }
    }

    std::cout << "\n  full road network dump -> " << STRATUM_TEST_DUMP_DIR << "\n\n"
              << "  " << std::left << std::setw(22) << "fixture" << std::right << std::setw(7)
              << "paint" << std::setw(8) << "quads" << std::setw(8) << "arrows" << std::setw(7)
              << "cross" << std::setw(8) << "kerbs" << std::setw(8) << "bridge" << std::setw(7)
              << "pier" << std::setw(8) << "tunnel" << std::setw(9) << "portal" << std::setw(8)
              << "dedup" << '\n';
    for (const Row& row : rows) print_counts(row);

    std::cout << "\n  triangles per material\n"
              << "  " << std::left << std::setw(22) << "fixture" << std::right;
    for (size_t i = 0; i < static_cast<size_t>(MaterialId::Count); ++i) {
        std::cout << std::setw(10) << material_id_name(static_cast<MaterialId>(i));
    }
    std::cout << '\n';
    for (const Row& row : rows) print_materials(row);

    std::cout << "\n  build cost, milliseconds\n"
              << "  " << std::left << std::setw(22) << "fixture" << std::right << std::setw(10)
              << "tris" << std::setw(10) << "build" << std::setw(11) << "elevation" << std::setw(10)
              << "junction" << '\n';
    for (const Row& row : rows) {
        std::cout << "  " << std::left << std::setw(22) << row.fixture << std::right << std::setw(10)
                  << row.obj.triangles << std::fixed << std::setprecision(3) << std::setw(10)
                  << row.build_ms << std::setw(11) << row.elevation_ms << std::setw(10)
                  << row.junction_ms << std::defaultfloat << '\n';
    }

    std::cout << '\n';
    for (const std::string& path : paths) std::cout << "  " << path << '\n';
    std::cout << '\n';
}

// ============================================================================
// Structures on their own
// ============================================================================

TEST(P5P6Dump, the_bridge_and_the_tunnel_dump_their_structures_separately) {
    for (const char* fixture : {"bridge_over.osm", "tunnel.osm"}) {
        Row row;
        std::filesystem::path full_path;
        RoadNetwork network;
        RoadNetworkBuilder builder;
        if (!dump_full(fixture, row, full_path, network, builder)) continue;

        const std::string stem = std::filesystem::path(fixture).stem().string();
        std::filesystem::path path;
        ObjDumpStats stats;
        if (!dump_structures(network, builder, full_config(terrain_for(fixture)), stem,
                             path, stats)) continue;

        // A bridge's structure file with nothing in it is the failure this suite
        // is here to make loud: it means P6 emitted no deck, no parapet and no
        // pier on a fixture that exists for exactly that.
        //
        // The TUNNEL's file has to carry its two portals for the same reason.
        // It was empty until ElevationConfig::tunnel_portal_at_surface stopped the
        // solver burying the edge at its own ends; see
        // `the_pipeline_emits_a_tunnel_portal_at_each_end` below, which asserts
        // the counts. This one asserts that the geometry reached the dump file.
        if ((stem == "bridge_over" || stem == "tunnel") && stats.triangles == 0) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "the structure dump is not empty",
                                          stem + ": no structure geometry");
        }
        if (!std::filesystem::exists(path)) {
            stratum::test::report_failure(__FILE__, __LINE__, "structure obj exists", path.string());
        }

        std::cout << "\n  " << stem << " structures -> " << path.string() << '\n'
                  << "    " << stats.triangles << " triangles:";
        for (MaterialId m : kStructureMaterials) {
            std::cout << ' ' << material_id_name(m) << ' ' << stats.count(m);
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}

// ============================================================================
// The counts each fixture exists to produce
// ============================================================================

/**
 * sidewalk_dup.osm has exactly ONE separately-mapped sidewalk, so exactly one
 * side is suppressed.
 *
 * A zero here is the dedup pass silently not running, and a two is it eating the
 * side that has no footway beside it -- the failure that leaves a street with no
 * pavement at all. Neither shows up in a triangle count.
 */
TEST(P5P6Dump, sidewalk_dup_suppresses_exactly_one_side) {
    Row row;
    std::filesystem::path path;
    RoadNetwork network;
    RoadNetworkBuilder builder;
    if (!dump_full("sidewalk_dup.osm", row, path, network, builder)) return;

    CHECK_EQ(row.deduped_sidewalks, size_t{1});
}

/**
 * turn_lanes.osm carries `turn:lanes=left|through|through;right`, so exactly
 * three arrows are painted.
 *
 * Counted over the WHOLE network rather than over the one approach, because an
 * emitter that painted the approach twice -- once per end of the arm, or once
 * per junction the arm touches -- produces three correct-looking arrows on one
 * end and three more somewhere else, and a per-approach count would not see it.
 */
TEST(P5P6Dump, turn_lanes_paints_exactly_three_arrows) {
    Row row;
    std::filesystem::path path;
    RoadNetwork network;
    RoadNetworkBuilder builder;
    if (!dump_full("turn_lanes.osm", row, path, network, builder)) return;

    CHECK_EQ(row.arrows, size_t{3});
}

/**
 * THE WHOLE PIPELINE EMITS A TUNNEL PORTAL AT EACH END, end to end.
 *
 * ### What this test used to say
 *
 * It was `the_pipeline_currently_emits_no_tunnel_portal_because_the_solver_buries_it`
 * and it pinned the opposite: RoadElevationSolver step 4 dropped a tunnel edge as
 * a straight chord until EVERY station, its two ends included, cleared
 * ElevationConfig::tunnel_depth below the terrain. A road already buried at its
 * own portals never crosses the ground, build_tunnel_portals() correctly declined
 * to put a headwall in the middle of a tunnel, and the pipeline was internally
 * consistent while its output was still wrong -- a road disappearing into an
 * unbroken hillside. That test named itself as the one to rewrite, and named the
 * two lines to replace its assertions with; those two lines are what it now
 * asserts.
 *
 * ### What closes the gap
 *
 * ElevationConfig::tunnel_portal_at_surface. The tunnel edge's portal NODES stay
 * at the approach roadway surface, tied to their approach edges exactly as a
 * bridge's abutments are, and the roadway dives away from each of them at the
 * edge's own max_grade_for() until it reaches depth. The crossing the portal rule
 * looks for then exists on each descent.
 *
 * The three facts this holds together:
 *
 * 1. tunnel.osm produces exactly one tunnel edge, and it is recognised as one all
 *    the way through the build.
 * 2. The solved surface is now ABOVE the terrain at both ends of the tunnel edge,
 *    which is the condition that makes a portal possible. Measured here rather
 *    than assumed, so this test fails the moment the solver stops doing it.
 * 3. build_tunnel_portals() does its job on that shape, and the two mouths reach
 *    the dump. Where a portal LANDS on the descent is asserted in the Structures
 *    suite, on this same fixture and this same ridge, by
 *    `tunnel_portals_sit_where_the_road_goes_under_not_at_the_way_end`.
 */
TEST(P5P6Dump, the_pipeline_emits_a_tunnel_portal_at_each_end) {
    Row row;
    std::filesystem::path path;
    RoadNetwork network;
    RoadNetworkBuilder builder;
    if (!dump_full("tunnel.osm", row, path, network, builder)) return;

    // The edge is there and is a tunnel: this is not a fixture or a graph problem.
    size_t tunnel_edges = 0;
    for (const auto& edge : builder.graph().edges()) {
        if (edge.is_tunnel) ++tunnel_edges;
    }
    CHECK_EQ(tunnel_edges, size_t{1});
    CHECK_TRUE(builder.elevation().is_solved());

    // The solved surface reaches the terrain at BOTH ends of the tunnel edge,
    // which is the condition that makes a portal possible. Measured here rather
    // than assumed, so this test fails the moment the solver stops doing it. The
    // road is ElevationConfig::surface_offset ABOVE the ground there, so the
    // cover is slightly negative and the threshold is a small positive number.
    for (size_t i = 0; i < builder.graph().edges().size(); ++i) {
        if (!builder.graph().edges()[i].is_tunnel) continue;
        const auto& cl = builder.centerlines()[i];
        const std::vector<float>& solved =
            builder.elevation().edge(static_cast<EdgeId>(i)).station_heights;
        if (!cl.is_valid() || solved.size() != cl.stations.size()) continue;

        const HeightSampler ground = terrain_for("tunnel.osm");
        const glm::dvec2 head = cl.stations.front().position;
        const glm::dvec2 tail = cl.stations.back().position;
        CHECK_TRUE(static_cast<double>(ground(head.x, head.y)) - solved.front() < 0.25);
        CHECK_TRUE(static_cast<double>(ground(tail.x, tail.y)) - solved.back() < 0.25);

        // And it really is a tunnel in between, not a road left on the surface.
        const glm::dvec2 middle = cl.stations[cl.stations.size() / 2].position;
        CHECK_TRUE(static_cast<double>(ground(middle.x, middle.y))
                       - solved[solved.size() / 2] > 5.0);
    }

    // Both mouths reach the dump.
    CHECK_EQ(row.tunnels, size_t{1});
    CHECK_EQ(row.portals, size_t{2});
}

/**
 * bridge_over.osm produces at least one bridge, with a parapet on BOTH sides.
 *
 * Both sides is the assertion. A builder that walked the profile once and offset
 * by `+ half_width` gets one wall in the right place and one down the middle of
 * the carriageway, and on a symmetric profile that reads at a glance as a central
 * reservation rather than as a bug. Sidedness is measured in the edge's own
 * frame, so it holds whichever way the way was drawn.
 */
TEST(P5P6Dump, the_bridge_has_parapets_on_both_sides) {
    Row row;
    std::filesystem::path path;
    RoadNetwork network;
    RoadNetworkBuilder builder;
    if (!dump_full("bridge_over.osm", row, path, network, builder)) return;

    CHECK_TRUE(row.bridges >= size_t{1});

    size_t checked = 0;
    for (const RoadPiece& piece : network.pieces) {
        if (piece.edge == kInvalidId || piece.edge >= builder.graph().edges().size()) continue;
        if (!builder.graph().edges()[piece.edge].is_bridge) continue;
        if (piece.edge >= builder.centerlines().size()) continue;

        const auto& cl = builder.centerlines()[piece.edge];
        if (!cl.is_valid()) continue;

        bool on_left = false;
        bool on_right = false;
        for (const jt::Tri2D& tri : p5::triangles_with(piece.mesh, MaterialId::Parapet)) {
            for (const glm::dvec2& p : {tri.a, tri.b, tri.c}) {
                const double lateral = p5::lateral_of(cl, p);
                if (lateral > 0.5) on_left = true;
                if (lateral < -0.5) on_right = true;
            }
        }
        ++checked;
        if (!on_left || !on_right) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "a bridge has a parapet on each side",
                "edge " + std::to_string(piece.edge) + ": left " + (on_left ? "yes" : "NO") +
                    ", right " + (on_right ? "yes" : "NO"));
        }
    }
    CHECK_TRUE(checked > 0);
}

// ============================================================================
// What the new passes cost
// ============================================================================

namespace {

/// Streets each way in the synthetic town, and the spacing between them, metres
constexpr int kGridStreets = 24;
constexpr double kGridSpacing = 160.0;

/**
 * @brief Write a town-sized synthetic extract that exercises every P5 and P6 pass
 *
 * A grid of `kGridStreets` residential ways each way, every crossing a shared
 * node and therefore a real four-way junction, which is what makes the trims, the
 * curb rings and the approach markings run at scale. On top of the plain grid:
 *
 * - `highway=crossing` on one node of every third east-west street, so the
 *   crossing pass and the dropped kerbs it demands have work to do.
 * - `turn:lanes` and `oneway` on those same streets, so arrows and stop lines are
 *   painted rather than only lane lines.
 * - Every sixth north-south street tagged `bridge=yes layer=1`, so the structure
 *   pass builds decks, parapets and piers.
 *
 * Deliberately NOT a copy of the RoadTerrainDump grid. That one measures P3 and is
 * plain residential throughout; a P5 and P6 measurement on it would time three
 * passes that had almost nothing to do.
 *
 * @param path Destination .osm path; parent directories are created
 * @return True when the file was written
 */
bool write_detail_grid(const std::filesystem::path& path) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);

    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;

    // Degrees per metre near 53.30 N, good enough for a synthetic extract.
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
                << "' version='1'>\n";
            // A crossing every third street, a third of the way along it.
            if (iy % 3 == 0 && ix == kGridStreets / 3) {
                out << "    <tag k='highway' v='crossing'/>\n";
            }
            out << "  </node>\n";
        }
    }

    int way_id = 2000000;
    for (int iy = 0; iy < kGridStreets; ++iy) {
        out << "  <way id='" << way_id++ << "' version='1'>\n";
        for (int ix = 0; ix < kGridStreets; ++ix) {
            out << "    <nd ref='" << node_id(ix, iy) << "'/>\n";
        }
        out << "    <tag k='highway' v='residential'/>\n"
               "    <tag k='sidewalk' v='both'/>\n";
        if (iy % 3 == 0) {
            out << "    <tag k='lanes' v='2'/>\n"
                   "    <tag k='oneway' v='yes'/>\n"
                   "    <tag k='turn:lanes' v='left|through'/>\n";
        } else {
            out << "    <tag k='lanes' v='2'/>\n";
        }
        out << "  </way>\n";
    }
    for (int ix = 0; ix < kGridStreets; ++ix) {
        out << "  <way id='" << way_id++ << "' version='1'>\n";
        for (int iy = 0; iy < kGridStreets; ++iy) {
            out << "    <nd ref='" << node_id(ix, iy) << "'/>\n";
        }
        out << "    <tag k='highway' v='residential'/>\n"
               "    <tag k='lanes' v='2'/>\n"
               "    <tag k='sidewalk' v='both'/>\n";
        if (ix % 6 == 0) {
            out << "    <tag k='bridge' v='yes'/>\n"
                   "    <tag k='layer' v='1'/>\n";
        }
        out << "  </way>\n";
    }
    out << "</osm>\n";
    return static_cast<bool>(out);
}

/// Median wall-clock cost of building @p data with @p cfg, over @p runs builds
double median_build_ms(const stratum::osm::ParsedOSMData& data, const RoadNetworkConfig& cfg,
                       int runs, RoadNetwork& last) {
    std::vector<double> samples;
    samples.reserve(static_cast<size_t>(runs));
    for (int i = 0; i < runs; ++i) {
        RoadNetworkBuilder builder;
        const auto start = std::chrono::steady_clock::now();
        last = builder.build(data, cfg);
        samples.push_back(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count());
    }
    std::sort(samples.begin(), samples.end());
    return samples.empty() ? 0.0 : samples[samples.size() / 2];
}

} // namespace

/**
 * What markings, crossings, structures and dedup cost on a town-sized network.
 *
 * The number that matters is the DELTA, because everything else in build() was
 * already being paid before this phase existed. Both configurations are built
 * several times and the median is taken: a single build of a network this size is
 * a few tens of milliseconds and the first one pays for page faults and for the
 * enkiTS scheduler's first task, which is not a cost the passes are responsible
 * for.
 *
 * Not a threshold. There is no budget to assert against and inventing one would
 * make this test fail on someone else's machine rather than tell them anything.
 * The only assertion is that the passes actually ran, so the delta is the cost of
 * doing the work and not the cost of skipping it.
 */
TEST(P5P6Dump, the_new_passes_cost_on_a_town_sized_network) {
    const auto grid_path = std::filesystem::path(STRATUM_TEST_DUMP_DIR) / "p5_p6_scale_grid.osm";
    if (!write_detail_grid(grid_path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "write the synthetic detail grid",
                                      grid_path.string());
        return;
    }

    stratum::osm::OSMParser parser;
    stratum::osm::ParserConfig parser_cfg;
    parser_cfg.import_buildings = false;
    parser_cfg.import_water = false;
    parser_cfg.import_landuse = false;
    parser_cfg.import_natural = false;
    parser_cfg.simplify_geometry = false;
    parser.set_config(parser_cfg);
    if (!parser.parse(grid_path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "parse the synthetic detail grid",
                                      parser.get_error());
        return;
    }
    const stratum::osm::ParsedOSMData data = parser.take_data();

    RoadNetworkConfig off;
    off.solve_junctions = true;
    off.emit_markings = false;
    off.emit_crossings = false;
    off.emit_structures = false;
    off.dedup.enabled = false;
    off.height_sampler = rolling();

    const RoadNetworkConfig on = full_config(rolling());

    constexpr int kRuns = 5;
    RoadNetwork off_net;
    RoadNetwork on_net;
    const double off_ms = median_build_ms(data, off, kRuns, off_net);
    const double on_ms = median_build_ms(data, on, kRuns, on_net);

    // The passes have to have done something, or the delta measures nothing.
    CHECK_TRUE(on_net.stats.markings_pieces > size_t{0});
    CHECK_TRUE(on_net.stats.crossings > size_t{0});
    CHECK_TRUE(on_net.stats.dropped_kerb_spans > size_t{0});
    CHECK_TRUE(on_net.stats.bridges > size_t{0});
    CHECK_TRUE(on_net.pieces.size() > size_t{100});

    size_t off_tris = 0;
    for (const RoadPiece& piece : off_net.pieces) off_tris += piece.mesh.indices.size() / 3;
    size_t on_tris = 0;
    for (const RoadPiece& piece : on_net.pieces) on_tris += piece.mesh.indices.size() / 3;

    std::cout << "\n  P5 and P6 cost on a " << kGridStreets << "x" << kGridStreets
              << " street grid (" << on_net.pieces.size() << " pieces, median of " << kRuns
              << " builds)\n"
              << std::fixed << std::setprecision(2)
              << "    P4 only                 " << std::setw(9) << off_ms << " ms   "
              << std::setw(8) << off_tris << " triangles\n"
              << "    with P5 and P6          " << std::setw(9) << on_ms << " ms   "
              << std::setw(8) << on_tris << " triangles\n"
              << "    the new passes cost     " << std::setw(9) << (on_ms - off_ms) << " ms   "
              << std::setw(8) << (on_tris - off_tris) << " triangles ("
              << std::setprecision(1)
              << (off_ms > 0.0 ? 100.0 * (on_ms - off_ms) / off_ms : 0.0) << "% of the P4 build)\n"
              << std::setprecision(2)
              // Both of these are inside BOTH builds, not inside the delta. Printed
              // for scale: the delta is small next to them, which is the point.
              << "    elevation solve, in both" << std::setw(9) << on_net.stats.elevation_ms
              << " ms\n"
              << "    junction solve, in both " << std::setw(9) << on_net.stats.junction_ms
              << " ms\n"
              << std::defaultfloat
              << "    emitted: " << on_net.stats.markings_pieces << " painted pieces, "
              << on_net.stats.crossings << " crossings, " << on_net.stats.dropped_kerb_spans
              << " dropped kerb spans, " << on_net.stats.bridges << " bridges, "
              << on_net.stats.tunnels << " tunnels, " << on_net.carve_portals.size()
              << " portals, " << on_net.stats.deduped_sidewalks << " sidewalk sides deduped\n\n";
}
