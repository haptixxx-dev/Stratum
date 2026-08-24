/**
 * @file test_road_network.cpp
 * @brief End-to-end RoadNetworkBuilder tests over the .osm fixtures
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * These tests run the whole P2 pipeline -- parse, graph, centerline, profile,
 * corridor -- against the hand-authored fixtures in tests/data, and assert the
 * properties that must hold for every road in every extract rather than the
 * properties of one hand-built polyline.
 *
 * The NaN sweep is the important one. A NaN vertex position reaching the GPU
 * neither crashes nor renders: it silently corrupts the bounding box, breaks
 * frustum culling for the whole chunk, and is the hardest failure in this
 * pipeline to trace back to its cause. It is cheap to catch here and expensive to
 * catch anywhere else, so it runs over EVERY fixture, not only the two the other
 * tests name.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadNetwork
 * @endcode
 */

#include "framework.hpp"

#include "osm/parser.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_network_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#ifndef STRATUM_TEST_DATA_DIR
#error "STRATUM_TEST_DATA_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::SubMesh;
using stratum::Vertex;
using stratum::material_id_name;
using stratum::osm::ParsedOSMData;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;
using stratum::osm::road::kInvalidId;

/// Every fixture in tests/data, in README table order
constexpr const char* kAllFixtures[] = {
    "four_way.osm", "t_junction.osm", "cul_de_sac.osm", "roundabout.osm",
    "motorway_link.osm", "rural_track.osm", "bridge_over.osm",
    "bridge_abutment.osm", "duplicate_node.osm",
};

/// Absolute path of a fixture in tests/data
std::filesystem::path fixture_path(const char* filename) {
    return std::filesystem::path(STRATUM_TEST_DATA_DIR) / filename;
}

/**
 * @brief Parse one fixture with roads only
 *
 * @param filename Fixture file name in tests/data
 * @return Parsed data, or std::nullopt when the parse failed
 */
std::optional<ParsedOSMData> parse_fixture(const char* filename) {
    const auto path = fixture_path(filename);
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

/// True when every component of a world position is finite
bool is_finite(const glm::vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

/// True when both components of a 2D local position are finite
bool is_finite(const glm::dvec2& v) {
    return std::isfinite(v.x) && std::isfinite(v.y);
}

/**
 * @brief Assert that a piece's submesh ranges tile its index buffer exactly once
 *
 * @param mesh  Mesh to check
 * @param label Fixture and piece, for the failure message
 */
void check_submeshes_tile(const Mesh& mesh, const std::string& label) {
    if (mesh.submeshes.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "piece mesh carries submeshes",
                                      label + ": submeshes is empty");
        return;
    }

    uint32_t expected_offset = 0;
    int previous = -1;
    for (const SubMesh& sub : mesh.submeshes) {
        if (sub.index_offset != expected_offset || sub.index_count == 0 ||
            (sub.index_count % 3u) != 0u) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "submesh ranges are contiguous whole triangles",
                label + " (" + material_id_name(sub.material) + "): offset " +
                    std::to_string(sub.index_offset) + " count " +
                    std::to_string(sub.index_count) + ", expected offset " +
                    std::to_string(expected_offset));
            return;
        }
        const int slot = static_cast<int>(sub.material);
        if (slot <= previous) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "materials ascend and appear exactly once",
                                          label + ": slot " + std::to_string(slot) + " after " +
                                              std::to_string(previous));
            return;
        }
        previous = slot;
        expected_offset += sub.index_count;
    }

    if (expected_offset != mesh.indices.size()) {
        stratum::test::report_failure(__FILE__, __LINE__,
                                      "submeshes cover the whole index buffer",
                                      label + ": covered " + std::to_string(expected_offset) +
                                          " of " + std::to_string(mesh.indices.size()));
    }
}

/**
 * @brief Every invariant a built network must satisfy, whatever the fixture
 *
 * @param network Network to check
 * @param graph   Graph the network was built against
 * @param label   Fixture name, for the failure message
 */
void check_network_invariants(const RoadNetwork& network, const RoadGraph& graph,
                              const std::string& label) {
    // Stats describe the pieces that were actually emitted.
    if (network.stats.pieces != network.pieces.size()) {
        stratum::test::report_failure(__FILE__, __LINE__, "stats.pieces == pieces.size()",
                                      label + ": " + std::to_string(network.stats.pieces) +
                                          " vs " + std::to_string(network.pieces.size()));
    }
    if (network.stats.edges != graph.edges().size()) {
        stratum::test::report_failure(__FILE__, __LINE__, "stats.edges == graph.edges().size()",
                                      label + ": " + std::to_string(network.stats.edges) +
                                          " vs " + std::to_string(graph.edges().size()));
    }
    // P4 appends one piece per solved junction, so the per-edge accounting is
    // over the EDGE pieces alone. stats.junction_pieces is how many of the
    // pieces are not a single edge.
    const size_t edge_pieces = network.stats.pieces - network.stats.junction_pieces;
    if (edge_pieces + network.stats.skipped_edges != network.stats.edges) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "every edge is either a piece or skipped",
            label + ": edge pieces " + std::to_string(edge_pieces) + " + skipped " +
                std::to_string(network.stats.skipped_edges) + " != edges " +
                std::to_string(network.stats.edges));
    }
    if (!std::isfinite(network.stats.build_ms) || network.stats.build_ms < 0.0) {
        stratum::test::report_failure(__FILE__, __LINE__, "stats.build_ms is a sane duration",
                                      label + ": " + std::to_string(network.stats.build_ms));
    }

    size_t vertices = 0;
    size_t triangles = 0;
    size_t junction_pieces_seen = 0;
    uint32_t previous_edge = 0;
    bool first = true;

    for (size_t p = 0; p < network.pieces.size(); ++p) {
        const RoadPiece& piece = network.pieces[p];
        const std::string where = label + " piece " + std::to_string(p);

        vertices += piece.mesh.vertices.size();
        triangles += piece.mesh.indices.size() / 3;

        // A skipped edge never emits an empty piece.
        if (piece.mesh.vertices.empty() || piece.mesh.indices.empty()) {
            stratum::test::report_failure(__FILE__, __LINE__, "piece carries geometry",
                                          where + ": empty mesh was emitted");
            continue;
        }
        if ((piece.mesh.indices.size() % 3u) != 0u) {
            stratum::test::report_failure(__FILE__, __LINE__, "index count is a multiple of 3",
                                          where + ": " +
                                              std::to_string(piece.mesh.indices.size()));
        }

        if (!piece.mesh.bounds.is_valid()) {
            stratum::test::report_failure(__FILE__, __LINE__, "piece bounds are valid",
                                          where + ": bounding box was never expanded");
        }
        check_submeshes_tile(piece.mesh, where);

        // A piece is one graph edge, or one solved junction. A junction spans
        // several edges and belongs to none, which is what kInvalidId means here;
        // any OTHER out-of-range id is a real defect. Junction pieces are
        // appended after every edge piece, so they never interleave with the
        // ascending-EdgeId run below.
        if (piece.edge == kInvalidId) {
            ++junction_pieces_seen;
        } else if (piece.edge >= graph.edges().size()) {
            stratum::test::report_failure(__FILE__, __LINE__, "piece names a valid graph edge",
                                          where + ": edge " + std::to_string(piece.edge) +
                                              " of " + std::to_string(graph.edges().size()));
        } else {
            if (junction_pieces_seen > 0) {
                stratum::test::report_failure(__FILE__, __LINE__,
                                              "edge pieces come before junction pieces",
                                              where + ": edge " + std::to_string(piece.edge) +
                                                  " after a junction piece");
            }
            // Pieces are in ascending EdgeId order so a build is reproducible.
            if (!first && piece.edge <= previous_edge) {
                stratum::test::report_failure(__FILE__, __LINE__,
                                              "pieces are in ascending EdgeId order",
                                              where + ": edge " + std::to_string(piece.edge) +
                                                  " after " + std::to_string(previous_edge));
            }
            previous_edge = piece.edge;
            first = false;
        }

        if (!is_finite(piece.anchor)) {
            stratum::test::report_failure(__FILE__, __LINE__, "piece anchor is finite",
                                          where + ": anchor is NaN or infinite");
        }

        for (size_t i = 0; i < piece.mesh.indices.size(); ++i) {
            if (piece.mesh.indices[i] >= piece.mesh.vertices.size()) {
                stratum::test::report_failure(__FILE__, __LINE__, "index is in range",
                                              where + ": index " + std::to_string(i));
                break;
            }
        }

        for (const glm::dvec2& pt : piece.outline) {
            if (!is_finite(pt)) {
                stratum::test::report_failure(__FILE__, __LINE__, "outline point is finite",
                                              where + ": outline carries a NaN");
                break;
            }
        }
    }

    if (network.stats.vertices != vertices) {
        stratum::test::report_failure(__FILE__, __LINE__, "stats.vertices sums the pieces",
                                      label + ": " + std::to_string(network.stats.vertices) +
                                          " vs " + std::to_string(vertices));
    }
    if (network.stats.triangles != triangles) {
        stratum::test::report_failure(__FILE__, __LINE__, "stats.triangles sums the pieces",
                                      label + ": " + std::to_string(network.stats.triangles) +
                                          " vs " + std::to_string(triangles));
    }
    if (network.stats.junction_pieces != junction_pieces_seen) {
        stratum::test::report_failure(__FILE__, __LINE__,
                                      "stats.junction_pieces counts the junction pieces",
                                      label + ": " +
                                          std::to_string(network.stats.junction_pieces) + " vs " +
                                          std::to_string(junction_pieces_seen));
    }
}

/// Pieces built from one graph edge, as opposed to from a solved junction
size_t edge_piece_count(const RoadNetwork& network) {
    size_t count = 0;
    for (const RoadPiece& piece : network.pieces) {
        if (piece.edge != kInvalidId) ++count;
    }
    return count;
}

/**
 * @brief Assert that no vertex position in any piece is a NaN or an infinity
 *
 * @param network Network to sweep
 * @param label   Fixture name, for the failure message
 */
void check_no_nan_positions(const RoadNetwork& network, const std::string& label) {
    for (size_t p = 0; p < network.pieces.size(); ++p) {
        const Mesh& mesh = network.pieces[p].mesh;
        for (size_t v = 0; v < mesh.vertices.size(); ++v) {
            const Vertex& vertex = mesh.vertices[v];
            if (!is_finite(vertex.position)) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "vertex position is finite",
                    label + " piece " + std::to_string(p) + " vertex " + std::to_string(v) +
                        ": (" + std::to_string(vertex.position.x) + ", " +
                        std::to_string(vertex.position.y) + ", " +
                        std::to_string(vertex.position.z) + ")");
                return;     // one report per fixture is enough to fail the run
            }
            if (!is_finite(vertex.normal) || !std::isfinite(vertex.uv.x) ||
                !std::isfinite(vertex.uv.y)) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "vertex normal and uv are finite",
                    label + " piece " + std::to_string(p) + " vertex " + std::to_string(v));
                return;
            }
        }
    }
}

} // namespace

// ============================================================================
// four_way.osm
// ============================================================================

TEST(RoadNetwork, four_way_builds_a_piece_for_every_edge) {
    auto data = parse_fixture("four_way.osm");
    if (!data.has_value()) return;

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*data, RoadNetworkConfig{});

    CHECK_TRUE(!network.pieces.empty());
    CHECK_TRUE(network.stats.edges > size_t{0});
    CHECK_EQ(network.stats.skipped_edges, size_t{0});

    check_network_invariants(network, builder.graph(), "four_way.osm");
    check_no_nan_positions(network, "four_way.osm");

    // Two roads crossing, each split at the shared node: four edges, four edge
    // pieces. P4 adds one piece per solved junction on top of those.
    CHECK_EQ(edge_piece_count(network), size_t{4});
    CHECK_EQ(builder.graph().edges().size(), size_t{4});

    for (const RoadPiece& piece : network.pieces) {
        CHECK_TRUE(!piece.mesh.submeshes.empty());
        CHECK_TRUE(piece.mesh.bounds.is_valid());

        // A junction piece's footprint is its curb ring or fillet polygon, and a
        // dead-end cap or a taper has none, so only an EDGE piece is guaranteed a
        // corridor outline.
        if (piece.edge != kInvalidId) {
            CHECK_TRUE(!piece.outline.empty());
        }
    }
}

TEST(RoadNetwork, four_way_stats_triangles_sums_the_piece_index_buffers) {
    auto data = parse_fixture("four_way.osm");
    if (!data.has_value()) return;

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*data, RoadNetworkConfig{});

    size_t triangles = 0;
    size_t vertices = 0;
    for (const RoadPiece& piece : network.pieces) {
        triangles += piece.mesh.indices.size() / 3;
        vertices += piece.mesh.vertices.size();
    }
    CHECK_EQ(network.stats.triangles, triangles);
    CHECK_EQ(network.stats.vertices, vertices);
    CHECK_TRUE(triangles > size_t{0});
}

// ============================================================================
// t_junction.osm
// ============================================================================

TEST(RoadNetwork, t_junction_builds_a_piece_for_every_edge) {
    auto data = parse_fixture("t_junction.osm");
    if (!data.has_value()) return;

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*data, RoadNetworkConfig{});

    CHECK_TRUE(!network.pieces.empty());
    CHECK_EQ(network.stats.skipped_edges, size_t{0});
    CHECK_EQ(edge_piece_count(network), builder.graph().edges().size());

    check_network_invariants(network, builder.graph(), "t_junction.osm");
    check_no_nan_positions(network, "t_junction.osm");

    // Way 2000 splits at the interior shared node, way 2010 stays whole: 3 edges.
    CHECK_EQ(builder.graph().edges().size(), size_t{3});

    for (const RoadPiece& piece : network.pieces) {
        CHECK_TRUE(!piece.mesh.submeshes.empty());
        CHECK_TRUE(piece.mesh.bounds.is_valid());
        if (piece.edge != kInvalidId) {
            CHECK_TRUE(!piece.outline.empty());
        }

        // Every road piece carries a running surface.
        bool has_surface = false;
        for (const SubMesh& sub : piece.mesh.submeshes) {
            if (sub.material == MaterialId::Asphalt || sub.material == MaterialId::Concrete ||
                sub.material == MaterialId::Gravel || sub.material == MaterialId::Dirt) {
                has_surface = true;
            }
        }
        CHECK_TRUE(has_surface);
    }
}

// ============================================================================
// Every fixture
// ============================================================================

TEST(RoadNetwork, no_fixture_produces_a_nan_or_infinite_vertex) {
    for (const char* fixture : kAllFixtures) {
        auto data = parse_fixture(fixture);
        if (!data.has_value()) continue;

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*data, RoadNetworkConfig{});
        check_no_nan_positions(network, fixture);
    }
}

TEST(RoadNetwork, every_fixture_satisfies_the_network_invariants) {
    for (const char* fixture : kAllFixtures) {
        auto data = parse_fixture(fixture);
        if (!data.has_value()) continue;

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*data, RoadNetworkConfig{});
        check_network_invariants(network, builder.graph(), fixture);

        // Every fixture holds at least one road, so every fixture must produce at
        // least one piece of geometry.
        if (network.pieces.empty()) {
            stratum::test::report_failure(__FILE__, __LINE__, "fixture produced geometry",
                                          std::string{fixture} + ": no pieces emitted from " +
                                              std::to_string(network.stats.edges) + " edges");
        }
    }
}

TEST(RoadNetwork, every_fixture_bounds_agree_with_its_vertices) {
    for (const char* fixture : kAllFixtures) {
        auto data = parse_fixture(fixture);
        if (!data.has_value()) continue;

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*data, RoadNetworkConfig{});

        for (size_t p = 0; p < network.pieces.size(); ++p) {
            const Mesh& mesh = network.pieces[p].mesh;
            if (mesh.vertices.empty()) continue;

            stratum::BoundingBox3D recomputed;
            for (const Vertex& v : mesh.vertices) recomputed.expand(v.position);

            const bool agrees =
                std::fabs(mesh.bounds.min.x - recomputed.min.x) < 1e-3f &&
                std::fabs(mesh.bounds.min.y - recomputed.min.y) < 1e-3f &&
                std::fabs(mesh.bounds.min.z - recomputed.min.z) < 1e-3f &&
                std::fabs(mesh.bounds.max.x - recomputed.max.x) < 1e-3f &&
                std::fabs(mesh.bounds.max.y - recomputed.max.y) < 1e-3f &&
                std::fabs(mesh.bounds.max.z - recomputed.max.z) < 1e-3f;
            if (!agrees) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "piece bounds match its vertices",
                    std::string{fixture} + " piece " + std::to_string(p) +
                        ": stored bounds do not enclose the geometry exactly");
                break;
            }
        }
    }
}

TEST(RoadNetwork, a_piece_anchor_sits_inside_its_own_geometry) {
    for (const char* fixture : kAllFixtures) {
        auto data = parse_fixture(fixture);
        if (!data.has_value()) continue;

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*data, RoadNetworkConfig{});

        for (size_t p = 0; p < network.pieces.size(); ++p) {
            const RoadPiece& piece = network.pieces[p];
            if (!piece.mesh.bounds.is_valid()) continue;

            // The anchor is 2D local metres; world space is (x, height, -y_2d).
            const double world_x = piece.anchor.x;
            const double world_z = -piece.anchor.y;
            const bool inside = world_x >= piece.mesh.bounds.min.x - 1e-3 &&
                                world_x <= piece.mesh.bounds.max.x + 1e-3 &&
                                world_z >= piece.mesh.bounds.min.z - 1e-3 &&
                                world_z <= piece.mesh.bounds.max.z + 1e-3;
            if (!inside) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "anchor lies within the piece bounds",
                    std::string{fixture} + " piece " + std::to_string(p) + ": anchor (" +
                        std::to_string(world_x) + ", " + std::to_string(world_z) +
                        ") outside the mesh footprint");
                break;
            }
        }
    }
}

// ============================================================================
// Degenerate input
// ============================================================================

TEST(RoadNetwork, empty_data_produces_an_empty_network) {
    const ParsedOSMData empty;

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(empty, RoadNetworkConfig{});

    CHECK_TRUE(network.pieces.empty());
    CHECK_EQ(network.stats.pieces, size_t{0});
    CHECK_EQ(network.stats.edges, size_t{0});
    CHECK_EQ(network.stats.vertices, size_t{0});
    CHECK_EQ(network.stats.triangles, size_t{0});
    CHECK_EQ(network.stats.skipped_edges, size_t{0});
    CHECK_TRUE(builder.graph().edges().empty());
    CHECK_TRUE(builder.graph().nodes().empty());
}

TEST(RoadNetwork, rebuilding_discards_the_previous_network) {
    auto four_way = parse_fixture("four_way.osm");
    auto t_junction = parse_fixture("t_junction.osm");
    if (!four_way.has_value() || !t_junction.has_value()) return;

    RoadNetworkBuilder builder;
    const RoadNetwork first = builder.build(*four_way, RoadNetworkConfig{});
    const size_t first_edges = builder.graph().edges().size();
    CHECK_EQ(first.stats.edges, first_edges);

    const RoadNetwork second = builder.build(*t_junction, RoadNetworkConfig{});
    CHECK_EQ(second.stats.edges, builder.graph().edges().size());
    CHECK_EQ(builder.graph().edges().size(), size_t{3});
    check_network_invariants(second, builder.graph(), "t_junction.osm after rebuild");

    // Building the same data twice must produce the same counts.
    RoadNetworkBuilder again;
    const RoadNetwork repeat = again.build(*t_junction, RoadNetworkConfig{});
    CHECK_EQ(repeat.stats.pieces, second.stats.pieces);
    CHECK_EQ(repeat.stats.vertices, second.stats.vertices);
    CHECK_EQ(repeat.stats.triangles, second.stats.triangles);
}
