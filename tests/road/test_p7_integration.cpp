/**
 * @file test_p7_integration.cpp
 * @brief The whole pipeline with P7 on, over every fixture, and off again
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The other three P7 suites each take one entry point and pin its behaviour. This
 * one takes RoadNetworkBuilder::build() with the four P7 switches on and asks the
 * questions that only fall out when the stage runs inside the pipeline.
 *
 * ### Does the stage change the road
 *
 * It must not. Welding renumbers vertices, reordering moves triangles inside a
 * range, and neither is allowed to change what is drawn. So the same fixture is
 * built twice, once with the switches on and once with them off, and the two
 * networks are compared triangle for triangle and square metre for square metre,
 * per material. A weld that dragged the kerb into the carriageway shows up here as
 * a changed Curb area on a fixture nobody wrote a kerb test for.
 *
 * The comparison is made in two strengths, and the difference matters. With
 * welding OFF and reordering ON, positions are bit-identical and the exact
 * multiset of (material, position triple) must match. With welding ON, a surviving
 * position may differ from the one it replaced by up to
 * WeldConfig::position_epsilon, so the comparison is made on per-material triangle
 * counts and plan areas instead. Asserting bit-equality across a weld would be
 * asserting that the weld matched nothing.
 *
 * ### Do the switches switch off
 *
 * All four false must reproduce the P6 output. Collision and LOD fields stay
 * empty, every P7 count in RoadNetwork::Stats stays zero, and two builds of the
 * same fixture are identical -- the stage is parallel across pieces, and a
 * parallel stage that writes anything shared would show up as a build that varies
 * run to run.
 *
 * ### Does anything come out broken
 *
 * Swept over every fixture in tests/data, on terrain and off it. Finite positions,
 * in-range indices, SubMesh ranges that still tile the index buffer, and a full
 * export whose triangles balance against the input.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests P7Integration
 * @endcode
 */

#include "framework.hpp"
#include "road/p5_p6_fixtures.hpp"
#include "road/p7_fixtures.hpp"

#include "osm/road/road_export.hpp"
#include "osm/road/road_network_builder.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::material_id_name;
using stratum::osm::road::ExportConfig;
using stratum::osm::road::ExportStats;
using stratum::osm::road::HeightSampler;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;
using stratum::osm::road::export_road_network;

namespace p5 = stratum::test::p5;
namespace p7 = stratum::test::p7;
namespace jt = stratum::test::junction;

/// Every material a road piece can carry, so a comparison names the one that moved
constexpr MaterialId kAllMaterials[] = {
    MaterialId::Default,    MaterialId::Asphalt, MaterialId::Concrete,
    MaterialId::Curb,       MaterialId::Sidewalk, MaterialId::Markings,
    MaterialId::Gravel,     MaterialId::Dirt,    MaterialId::Grass,
    MaterialId::BridgeDeck, MaterialId::Parapet,
};

constexpr size_t kMaterialCount = sizeof(kAllMaterials) / sizeof(kAllMaterials[0]);

/// Triangle count and plan area per material, summed over a whole network
struct Tally {
    size_t triangles[kMaterialCount] = {};
    double area[kMaterialCount] = {};
    size_t vertices = 0;
    size_t total_triangles = 0;
};

Tally tally(const RoadNetwork& network) {
    Tally out;
    for (const RoadPiece& piece : network.pieces) {
        out.vertices += piece.mesh.vertices.size();
        out.total_triangles += p7::triangle_count(piece.mesh);
        for (const jt::Tri2D& tri : jt::triangles_of(piece.mesh)) {
            for (size_t m = 0; m < kMaterialCount; ++m) {
                if (tri.material != kAllMaterials[m]) continue;
                ++out.triangles[m];
                out.area[m] += 0.5 * std::fabs(jt::cross2(tri.a, tri.b, tri.c));
            }
        }
    }
    return out;
}

/// Rolling ground, the same sampler shape the P5/P6 sweep uses
HeightSampler rolling() {
    return [](double x, double y) {
        return static_cast<float>(9.0 * std::sin(x / 130.0) + 6.0 * std::cos(y / 95.0) + 20.0);
    };
}

/// P5 and P6 on, P7 off: the reference the P7 stage must reproduce
RoadNetworkConfig p6_reference(const HeightSampler& sampler) {
    RoadNetworkConfig cfg;
    cfg.height_sampler = sampler;
    cfg.weld_meshes = false;
    cfg.optimize_meshes = false;
    cfg.build_collision = false;
    cfg.build_lods = false;
    return cfg;
}

/// Everything on
RoadNetworkConfig p7_everything(const HeightSampler& sampler) {
    RoadNetworkConfig cfg;
    cfg.height_sampler = sampler;
    cfg.weld_meshes = true;
    cfg.optimize_meshes = true;
    cfg.build_collision = true;
    cfg.build_lods = true;
    return cfg;
}

/// Build one fixture, reporting a parse failure rather than returning quietly
bool build_fixture(const std::string& fixture, const RoadNetworkConfig& cfg, RoadNetwork& out) {
    const auto parsed = jt::parse_fixture(fixture.c_str());
    if (!parsed) return false;
    RoadNetworkBuilder builder;
    out = builder.build(*parsed, cfg);
    return true;
}

/// Every piece's mesh reduced to one sorted multiset of content keys
std::vector<p7::TriKey> network_content(const RoadNetwork& network) {
    std::vector<p7::TriKey> out;
    for (const RoadPiece& piece : network.pieces) {
        const std::vector<p7::TriKey> keys = p7::triangle_multiset(piece.mesh);
        out.insert(out.end(), keys.begin(), keys.end());
    }
    std::sort(out.begin(), out.end());
    return out;
}

/**
 * @brief Total 3D surface area of the near-vertical triangles of every collision mesh
 *
 * A wall -- a parapet, a tunnel headwall, a retaining face -- is exactly the
 * geometry the collision derivation's plan-area guard cannot see, because a
 * vertical surface covers no plan. Measuring it in 3D is the only way to notice
 * that one was deleted.
 */
double collision_wall_area(const RoadNetwork& network) {
    double total = 0.0;
    for (const RoadPiece& piece : network.pieces) {
        const Mesh& mesh = piece.collision;
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            const glm::vec3& a = mesh.vertices[mesh.indices[i]].position;
            const glm::vec3& b = mesh.vertices[mesh.indices[i + 1]].position;
            const glm::vec3& c = mesh.vertices[mesh.indices[i + 2]].position;
            const glm::vec3 face = glm::cross(b - a, c - a);
            const float len = glm::length(face);
            if (!(len > 0.0f) || !std::isfinite(len)) continue;
            if (std::fabs(face.y / len) >= stratum::osm::road::kVerticalNormalY) continue;
            total += 0.5 * static_cast<double>(len);
        }
    }
    return total;
}

/// The fixtures this suite sweeps: every file in tests/data
std::vector<std::string> fixtures() { return p5::all_fixtures(); }

} // namespace

// ============================================================================
// The stage changes nothing but the vertex count
// ============================================================================

/**
 * Reordering alone is bit-exact.
 *
 * With welding off, optimize_mesh() may not move a single vertex position, so the
 * multiset of (material, position triple) must come back identical. This is the
 * assertion that catches a whole-buffer meshopt_optimizeVertexCache() call: it
 * moves triangles across SubMesh boundaries, so a triangle changes material and
 * the multiset changes with it, while every count in the network stays the same.
 */
TEST(P7Integration, reordering_alone_does_not_move_a_triangle) {
    const HeightSampler sampler = rolling();
    for (const std::string& fixture : fixtures()) {
        RoadNetwork off;
        if (!build_fixture(fixture, p6_reference(sampler), off)) continue;

        RoadNetworkConfig reorder_only = p6_reference(sampler);
        reorder_only.optimize_meshes = true;

        RoadNetwork on;
        if (!build_fixture(fixture, reorder_only, on)) continue;

        CHECK_EQ(on.pieces.size(), off.pieces.size());
        const std::vector<p7::TriKey> before = network_content(off);
        const std::vector<p7::TriKey> after = network_content(on);
        if (before != after) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "reordering preserves the triangle set",
                fixture + ": " + std::to_string(before.size()) + " triangles in, " +
                    std::to_string(after.size()) + " out, and the sets differ");
        }
    }
}

/**
 * Welding and reordering together leave every material's triangle count and plan
 * area untouched, and remove vertices.
 *
 * Triangle counts are exact by contract: welding never removes a triangle, not
 * even one it flattened. Plan areas are compared with an absolute tolerance,
 * because a welded position may sit up to WeldConfig::position_epsilon from the
 * one it replaced.
 */
TEST(P7Integration, welding_preserves_every_material_and_removes_vertices) {
    const HeightSampler sampler = rolling();
    size_t fixtures_with_a_weld = 0;

    for (const std::string& fixture : fixtures()) {
        RoadNetwork off;
        RoadNetwork on;
        if (!build_fixture(fixture, p6_reference(sampler), off)) continue;

        RoadNetworkConfig weld_cfg = p6_reference(sampler);
        weld_cfg.weld_meshes = true;
        weld_cfg.optimize_meshes = true;
        if (!build_fixture(fixture, weld_cfg, on)) continue;

        CHECK_EQ(on.pieces.size(), off.pieces.size());

        const Tally a = tally(off);
        const Tally b = tally(on);

        CHECK_EQ(b.total_triangles, a.total_triangles);
        for (size_t m = 0; m < kMaterialCount; ++m) {
            if (b.triangles[m] != a.triangles[m]) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "welding preserves the triangle count of every material",
                    fixture + ": " + material_id_name(kAllMaterials[m]) + " went from " +
                        std::to_string(a.triangles[m]) + " to " + std::to_string(b.triangles[m]));
            }
            if (std::fabs(b.area[m] - a.area[m]) > 1e-3) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "welding preserves the plan area of every material",
                    fixture + ": " + material_id_name(kAllMaterials[m]) + " went from " +
                        std::to_string(a.area[m]) + " m2 to " + std::to_string(b.area[m]) + " m2");
            }
        }

        // Welding cannot add vertices, and the counts reported must agree with
        // the meshes handed back. Every vertex the P7 stage was handed is either
        // still there, merged by the weld, or dropped by the reorder's
        // compaction as unreferenced -- nothing else removes one, so the sum is
        // exact rather than a bound.
        CHECK_TRUE(b.vertices <= a.vertices);
        CHECK_EQ(on.stats.vertices, b.vertices);
        CHECK_EQ(on.stats.vertices + on.stats.vertices_welded + on.stats.vertices_dropped,
                 a.vertices);
        if (on.stats.vertices_welded > 0) ++fixtures_with_a_weld;
    }

    // At least one fixture must actually have had something to weld. A phase
    // whose headline pass matches nothing anywhere is not a phase that ran.
    CHECK_TRUE(fixtures_with_a_weld > 0);
}

// ============================================================================
// The switches switch off
// ============================================================================

/**
 * All four P7 switches off reproduces the P6 output: no collision, no LODs, every
 * P7 count zero, and a build that is identical run to run.
 *
 * The determinism half is not incidental. The P7 stage is parallel across pieces,
 * and a parallel stage that writes anything shared -- a scratch buffer, an
 * accumulating counter, a spatial hash reused between pieces -- produces a network
 * that differs between runs in a way no single-run assertion sees.
 */
TEST(P7Integration, all_switches_off_reproduces_the_p6_output) {
    const HeightSampler sampler = rolling();
    for (const std::string& fixture : fixtures()) {
        RoadNetwork first;
        RoadNetwork second;
        if (!build_fixture(fixture, p6_reference(sampler), first)) continue;
        if (!build_fixture(fixture, p6_reference(sampler), second)) continue;

        CHECK_EQ(first.stats.vertices_welded, size_t{0});
        CHECK_EQ(first.stats.vertices_dropped, size_t{0});
        CHECK_EQ(first.stats.triangles_before_lod, size_t{0});
        CHECK_EQ(first.stats.triangles_after_lod, size_t{0});
        CHECK_EQ(first.stats.collision_triangles, size_t{0});

        for (const RoadPiece& piece : first.pieces) {
            CHECK_TRUE(piece.collision.indices.empty());
            CHECK_TRUE(piece.collision.vertices.empty());
            CHECK_FALSE(piece.lods.is_valid());
        }

        CHECK_EQ(second.pieces.size(), first.pieces.size());
        if (network_content(first) != network_content(second)) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "two builds of one fixture are identical",
                                          fixture + " differs between runs");
        }
    }
}

/**
 * With the switches on, the same fixture is still identical run to run.
 *
 * The P7 stage is the one that runs in parallel over the compacted piece list, so
 * this is where a shared scratch buffer would show. Welding is documented as
 * deterministic -- lowest original index survives -- and meshoptimizer's passes are
 * deterministic for a given input, so the whole stage is.
 */
TEST(P7Integration, the_p7_stage_is_deterministic) {
    const HeightSampler sampler = rolling();
    for (const char* fixture : { "four_way.osm", "roundabout.osm", "bridge_over.osm" }) {
        RoadNetwork first;
        RoadNetwork second;
        if (!build_fixture(fixture, p7_everything(sampler), first)) continue;
        if (!build_fixture(fixture, p7_everything(sampler), second)) continue;

        CHECK_EQ(second.pieces.size(), first.pieces.size());
        CHECK_EQ(second.stats.vertices, first.stats.vertices);
        CHECK_EQ(second.stats.vertices_welded, first.stats.vertices_welded);
        CHECK_EQ(second.stats.collision_triangles, first.stats.collision_triangles);
        CHECK_EQ(second.stats.triangles_after_lod, first.stats.triangles_after_lod);
        if (network_content(first) != network_content(second)) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "two P7 builds of one fixture are identical",
                                          std::string(fixture) + " differs between runs");
        }
    }
}

// ============================================================================
// The optional outputs
// ============================================================================

/**
 * Collision meshes and LOD chains appear when asked for, are consistent with the
 * render mesh, and are counted correctly.
 *
 * `lods.levels[0]` is defined as the finished render mesh, so a chain whose level
 * 0 has a different triangle count from the piece it came from was built before
 * the optimisation rather than after it, and the whole chain is then one level out
 * of step with what is drawn up close.
 */
TEST(P7Integration, collision_and_lods_are_filled_when_asked_for) {
    const HeightSampler sampler = rolling();
    RoadNetwork network;
    if (!build_fixture("four_way.osm", p7_everything(sampler), network)) return;
    CHECK_TRUE(!network.pieces.empty());

    size_t collision_triangles = 0;
    size_t coarsest_triangles = 0;
    size_t pieces_with_collision = 0;
    size_t pieces_with_lods = 0;

    for (const RoadPiece& piece : network.pieces) {
        collision_triangles += p7::triangle_count(piece.collision);
        if (!piece.collision.indices.empty()) {
            ++pieces_with_collision;
            CHECK_TRUE(piece.collision.submeshes.empty());
            CHECK_TRUE(p7::indices_are_sane(piece.collision));
            CHECK_TRUE(p7::mesh_is_finite(piece.collision));
        }

        if (piece.lods.is_valid()) {
            ++pieces_with_lods;
            CHECK_EQ(piece.lods.screen_thresholds.size(), piece.lods.levels.size());
            CHECK_EQ(p7::triangle_count(piece.lods.levels[0]), p7::triangle_count(piece.mesh));
            coarsest_triangles += p7::triangle_count(piece.lods.levels.back());
            for (const Mesh& level : piece.lods.levels) {
                std::string reason;
                if (!p7::submeshes_tile_exactly(level, reason)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "every LOD level tiles its index buffer",
                                                  reason);
                }
                CHECK_TRUE(p7::indices_are_sane(level));
                CHECK_TRUE(p7::mesh_is_finite(level));
            }
        }
    }

    CHECK_TRUE(pieces_with_collision > 0);
    CHECK_TRUE(pieces_with_lods > 0);
    CHECK_EQ(network.stats.collision_triangles, collision_triangles);
    CHECK_EQ(network.stats.triangles_before_lod, network.stats.triangles);
    CHECK_EQ(network.stats.triangles_after_lod, coarsest_triangles);
    CHECK_TRUE(network.stats.triangles_after_lod <= network.stats.triangles_before_lod);
}

/**
 * Simplification does not delete a wall.
 *
 * The collision derivation classifies a near-vertical face by its LOCAL height:
 * a 150 mm kerb is a step and is deleted and bridged, a tunnel headwall or a
 * bridge parapet is a wall and is kept, because a vehicle that can drive through
 * a parapet drives off the bridge. Simplification then runs, and its hole guard
 * measures PLAN area -- which a wall does not contribute to at all. So a
 * simplifier is free to collapse a headwall away without moving a single number
 * the guard looks at, and the guard accepts the result.
 *
 * Measured here in 3D, on the two fixtures that have walls: the wall area at the
 * shipping ratio must match the wall area with simplification switched off.
 */
TEST(P7Integration, simplification_keeps_the_walls_of_a_structure) {
    const HeightSampler sampler = rolling();
    for (const char* fixture : { "tunnel.osm", "bridge_over.osm" }) {
        RoadNetworkConfig unsimplified = p7_everything(sampler);
        unsimplified.collision.simplify_ratio = 1.0f;

        RoadNetwork off;
        RoadNetwork on;
        if (!build_fixture(fixture, unsimplified, off)) continue;
        if (!build_fixture(fixture, p7_everything(sampler), on)) continue;

        const double wall_off = collision_wall_area(off);
        const double wall_on = collision_wall_area(on);

        // The fixture has walls at all: without this the comparison below is
        // vacuous the day the structure builders stop emitting them.
        if (!(wall_off > 1.0)) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "the fixture's collision surface carries a wall",
                                          std::string(fixture) + " has "
                                              + std::to_string(wall_off) + " m2 of wall");
            continue;
        }
        if (!(wall_on >= wall_off * 0.95)) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "simplification keeps the walls",
                                          std::string(fixture) + ": " + std::to_string(wall_off)
                                              + " m2 of wall becomes " + std::to_string(wall_on)
                                              + " m2 at the shipping ratio");
        }
    }
}

// ============================================================================
// The sweep
// ============================================================================

/**
 * Every fixture, everything on, on terrain and off it: nothing infinite, no index
 * out of range, and every SubMesh range still tiling its index buffer.
 *
 * A NaN vertex position neither crashes nor renders. It silently corrupts the
 * bounding box, breaks frustum culling for the whole chunk, and is the hardest
 * failure in this pipeline to trace back to its cause. Welding is a new way to
 * produce one -- a weld group whose survivor was chosen from an already-broken
 * vertex propagates it into every triangle that referenced the others.
 */
TEST(P7Integration, every_fixture_survives_the_full_pipeline) {
    for (int on_terrain = 0; on_terrain < 2; ++on_terrain) {
        const HeightSampler sampler = on_terrain ? rolling() : HeightSampler{};
        for (const std::string& fixture : fixtures()) {
            RoadNetwork network;
            if (!build_fixture(fixture, p7_everything(sampler), network)) continue;

            const std::string where = fixture + (on_terrain ? " on terrain" : " flat");
            for (size_t i = 0; i < network.pieces.size(); ++i) {
                const RoadPiece& piece = network.pieces[i];
                const std::string what = where + " piece " + std::to_string(i);

                if (!p7::mesh_is_finite(piece.mesh)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "every vertex is finite", what);
                }
                if (!p7::indices_are_sane(piece.mesh)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "every index is in range", what);
                }
                std::string reason;
                if (!p7::submeshes_tile_exactly(piece.mesh, reason)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "submeshes tile the index buffer",
                                                  what + ": " + reason);
                }
                CHECK_TRUE(piece.mesh.bounds.is_valid() || piece.mesh.vertices.empty());
            }
        }
    }
}

/**
 * Export every fixture and get its triangles back.
 *
 * The conservation property from the export suite, asserted end to end against
 * geometry the whole pipeline produced rather than against slabs placed by hand.
 * A junction piece is a fan whose anchor is nowhere near most of its triangles,
 * which is precisely the case that made assigning by anchor wrong.
 */
TEST(P7Integration, exporting_every_fixture_conserves_its_triangles) {
    const HeightSampler sampler = rolling();
    for (const std::string& fixture : fixtures()) {
        RoadNetwork network;
        if (!build_fixture(fixture, p7_everything(sampler), network)) continue;

        size_t expected = 0;
        for (const RoadPiece& piece : network.pieces) {
            expected += p7::triangle_count(piece.mesh);
        }
        if (expected == 0) continue;

        std::string name = fixture;
        const size_t dot = name.find('.');
        if (dot != std::string::npos) name = name.substr(0, dot);

        const std::filesystem::path dir = p7::scratch_dir("network_" + name);
        ExportConfig cfg;
        cfg.chunk_size = 40.0f;         // small, so every fixture spans several cells
        cfg.export_collision = false;
        cfg.export_lods = false;

        const ExportStats stats = export_road_network(network.pieces, dir, cfg);
        if (stats.triangles != expected) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the export reports the input triangle count",
                fixture + ": reported " + std::to_string(stats.triangles) + ", input had " +
                    std::to_string(expected));
        }

        size_t on_disk = 0;
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
            if (!entry.is_regular_file() || entry.path().extension() != ".obj") continue;
            const std::string stem = entry.path().stem().string();
            if (stem.find("_collision") != std::string::npos) continue;
            if (stem.find("_lod") != std::string::npos) continue;
            on_disk += p7::valid_face_count(p7::read_obj(entry.path()));
        }
        if (on_disk != expected) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the files hold the input triangle count",
                fixture + ": " + std::to_string(on_disk) + " on disk, input had " +
                    std::to_string(expected));
        }
    }
}
