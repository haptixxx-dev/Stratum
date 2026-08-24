/**
 * @file test_mesh_optimize.cpp
 * @brief Welding, reordering and LOD chains: what they may change and what they may not
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Every entry point in mesh_optimize.hpp rewrites a mesh in place or returns a
 * rewritten copy, and every one of them is allowed to renumber every vertex. So
 * none of these tests can compare index buffers. They compare CONTENT: the
 * multiset of (material, position triple) that p7_fixtures.hpp's
 * triangle_multiset() reduces a mesh to, which survives renumbering and
 * reordering and still distinguishes a flipped winding.
 *
 * The assertions fall into three groups.
 *
 * ### The invariants the callers hold offsets against
 *
 * weld_vertices() and optimize_mesh() both promise that `index_offset` and
 * `index_count` come back byte-for-byte unchanged, for every range, in the same
 * order. That is why welding does not remove the degenerate triangles it creates:
 * a shorter range would shift every range after it. RoadPiece consumers, the
 * exporter and the renderer all hold those offsets, so this is checked on every
 * mesh every test touches.
 *
 * ### The crease guard
 *
 * The one assertion in this file that stops a silent, catastrophic, visually
 * plausible failure. A kerb face meets a kerb top at right angles, at coincident
 * positions, under the same MaterialId::Curb. Nothing but WeldConfig::normal_epsilon
 * separates them. Weld them and the mesh is still manifold, still draws, still
 * passes every count-based test in this repository -- and every kerb in the
 * network has become a 45 degree shading bevel with no step. The corridor
 * extruder emits exactly this configuration for every kerbed profile, so the
 * fixture is production geometry rather than a shape invented to trip the test.
 *
 * ### Material survival
 *
 * build_lod_chain() simplifies per SubMesh range and reassembles. A level that
 * came back without its Curb range has lost the kerb from the whole road at that
 * distance, which reads as the road flattening as the camera pulls back. Every
 * level is checked for the full material set of level 0.
 *
 * ### Why a slab and not a corridor
 *
 * The LOD tests run against p7::make_slab_mesh(), not against a corridor. A
 * corridor strip is two vertex columns wide, so every one of its vertices lies on
 * an open boundary of its own strip and LodConfig::lock_borders correctly forbids
 * collapsing any of them. Asserting "each level has fewer triangles" against a
 * corridor would be asserting that the border lock is broken. See the note in
 * p7_fixtures.hpp.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests MeshOptimize
 * @endcode
 */

#include "framework.hpp"
#include "road/p7_fixtures.hpp"

#include "osm/road/corridor.hpp"
#include "osm/road/mesh_optimize.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
using stratum::osm::road::LodChain;
using stratum::osm::road::LodConfig;
using stratum::osm::road::WeldConfig;
using stratum::osm::road::build_lod_chain;
using stratum::osm::road::kLodSwitchFactor;
using stratum::osm::road::optimize_mesh;
using stratum::osm::road::weld_vertices;

namespace p7 = stratum::test::p7;

/// Cosine below which two normals are a crease rather than the same surface
constexpr double kCreaseCos = 0.9;

/// Every SubMesh range of @p before is present unchanged in @p after
void check_ranges_unchanged(const Mesh& before, const Mesh& after, const char* what) {
    CHECK_EQ(after.submeshes.size(), before.submeshes.size());
    const size_t n = std::min(after.submeshes.size(), before.submeshes.size());
    for (size_t i = 0; i < n; ++i) {
        if (after.submeshes[i].index_offset != before.submeshes[i].index_offset ||
            after.submeshes[i].index_count != before.submeshes[i].index_count ||
            after.submeshes[i].material != before.submeshes[i].material) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "submesh range unchanged",
                std::string(what) + ": range " + std::to_string(i) + " was {" +
                    std::to_string(before.submeshes[i].index_offset) + ", " +
                    std::to_string(before.submeshes[i].index_count) + ", " +
                    material_id_name(before.submeshes[i].material) + "} and is now {" +
                    std::to_string(after.submeshes[i].index_offset) + ", " +
                    std::to_string(after.submeshes[i].index_count) + ", " +
                    material_id_name(after.submeshes[i].material) + "}");
        }
    }
}

/// The mesh's ranges tile its index buffer with no gap and no overlap
void check_tiling(const Mesh& mesh, const char* what) {
    std::string reason;
    if (!p7::submeshes_tile_exactly(mesh, reason)) {
        stratum::test::report_failure(__FILE__, __LINE__, "submeshes tile the index buffer",
                                      std::string(what) + ": " + reason);
    }
}

/// Normals of every vertex sitting at each quantised position
std::map<p7::PosKey, std::vector<glm::dvec3>> normals_by_position(const Mesh& mesh) {
    std::map<p7::PosKey, std::vector<glm::dvec3>> out;
    for (const Vertex& v : mesh.vertices) {
        out[p7::pos_key(v.position)].push_back(glm::dvec3(v.normal));
    }
    return out;
}

/// True when some pair in @p normals differs by more than the crease threshold
bool holds_a_crease(const std::vector<glm::dvec3>& normals) {
    for (size_t i = 0; i < normals.size(); ++i) {
        for (size_t j = i + 1; j < normals.size(); ++j) {
            const double la = glm::length(normals[i]);
            const double lb = glm::length(normals[j]);
            if (la <= 0.0 || lb <= 0.0) continue;
            if (glm::dot(normals[i] / la, normals[j] / lb) < kCreaseCos) return true;
        }
    }
    return false;
}

/**
 * @brief Three coincident vertices with different referencing material sets
 *
 * The one configuration that tells a material MASK apart from a single material
 * id. All three sit at the origin with identical position, normal, UV, colour and
 * tangent, so nothing but WeldConfig::respect_material can separate them:
 *
 * - vertex 0 is referenced by an Asphalt triangle and a Curb triangle
 * - vertex 1 is referenced by an Asphalt triangle and a Curb triangle
 * - vertex 2 is referenced by an Asphalt triangle only
 *
 * With masks, 0 and 1 weld and 2 does not, so two vertices remain at the origin.
 * A weld that ignores materials leaves one. A weld that refuses any vertex
 * referenced by more than one material leaves three.
 */
Mesh make_mask_mesh() {
    Mesh mesh;
    const glm::vec3 positions[6] = {
        { 0.0f, 0.0f,  0.0f },      // 0: shared point, Asphalt + Curb
        { 0.0f, 0.0f,  0.0f },      // 1: shared point, Asphalt + Curb
        { 0.0f, 0.0f,  0.0f },      // 2: shared point, Asphalt only
        { 1.0f, 0.0f,  0.0f },      // 3
        { 1.0f, 0.0f, -1.0f },      // 4
        { 0.0f, 0.0f, -1.0f },      // 5
    };
    for (const glm::vec3& p : positions) {
        Vertex v{};
        v.position = p;
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v.uv = glm::vec2(0.0f);
        v.color = glm::vec4(1.0f);
        v.tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        mesh.vertices.push_back(v);
    }
    mesh.indices = {
        0, 3, 4,  1, 3, 4,  2, 3, 4,        // Asphalt
        0, 4, 5,  1, 4, 5,                  // Curb
    };
    mesh.submeshes.push_back(SubMesh{ 0u, 9u, MaterialId::Asphalt });
    mesh.submeshes.push_back(SubMesh{ 9u, 6u, MaterialId::Curb });
    mesh.compute_bounds();
    return mesh;
}

/// Vertices of @p mesh sitting at the origin
size_t vertices_at_origin(const Mesh& mesh) {
    size_t n = 0;
    const p7::PosKey origin = p7::pos_key(glm::vec3(0.0f));
    for (const Vertex& v : mesh.vertices) {
        if (p7::pos_key(v.position) == origin) ++n;
    }
    return n;
}

} // namespace

// ============================================================================
// Welding
// ============================================================================

/**
 * A mesh holding the same corridor twice welds down to one copy's worth of
 * vertices, and nothing else about it moves.
 *
 * The duplication is the case the pass exists for. An arm end column and the
 * junction fan vertex it meets, a marking quad appended over the corridor it was
 * derived from, a bridge deck edge and the parapet foot on it: every one of them
 * is two builders arriving at the same position from different code. Appending
 * one corridor to itself is that situation with the answer known in advance.
 */
TEST(MeshOptimize, weld_removes_duplicates_without_touching_triangles_or_ranges) {
    const stratum::osm::road::Corridor corridor = p7::kerbed_corridor();
    CHECK_TRUE(p7::triangle_count(corridor.mesh) > 0);
    if (corridor.mesh.indices.empty()) return;

    Mesh doubled;
    doubled.append(corridor.mesh, MaterialId::Asphalt);
    doubled.append(corridor.mesh, MaterialId::Asphalt);

    const Mesh before = doubled;
    const std::vector<p7::TriKey> content_before = p7::triangle_multiset(doubled);

    const size_t removed = weld_vertices(doubled);

    // One whole copy is redundant, so at least that many vertices go.
    CHECK_TRUE(removed >= corridor.mesh.vertices.size());
    CHECK_EQ(doubled.vertices.size(), before.vertices.size() - removed);

    // The invariants every caller holds offsets against.
    CHECK_EQ(doubled.indices.size(), before.indices.size());
    CHECK_EQ(p7::triangle_count(doubled), p7::triangle_count(before));
    check_ranges_unchanged(before, doubled, "weld of a doubled corridor");
    check_tiling(doubled, "welded doubled corridor");
    CHECK_TRUE(p7::indices_are_sane(doubled));

    // Same triangles, same windings, same materials.
    CHECK_TRUE(p7::triangle_multiset(doubled) == content_before);

    // Bounds are recomputed from the survivors, so they still contain them.
    CHECK_TRUE(doubled.bounds.is_valid());
    for (const Vertex& v : doubled.vertices) {
        CHECK_TRUE(v.position.x >= doubled.bounds.min.x - 1e-4f &&
                   v.position.x <= doubled.bounds.max.x + 1e-4f);
        CHECK_TRUE(v.position.y >= doubled.bounds.min.y - 1e-4f &&
                   v.position.y <= doubled.bounds.max.y + 1e-4f);
        CHECK_TRUE(v.position.z >= doubled.bounds.min.z - 1e-4f &&
                   v.position.z <= doubled.bounds.max.z + 1e-4f);
    }
}

/**
 * THE CREASE GUARD.
 *
 * A kerb face meets a kerb top at right angles, at coincident positions, under
 * one material. Welding those two vertices averages a hard edge into a bevel: the
 * mesh stays manifold, the triangle count is untouched, every other test in this
 * file still passes, and every kerb in the network has quietly stopped being a
 * kerb.
 *
 * The fixture is the real extruder's real output for the shipping residential
 * profile, so this is not a shape invented to trip the test -- it is what the
 * pipeline emits for every kerbed street in an extract.
 */
TEST(MeshOptimize, weld_does_not_smooth_a_kerb_crease) {
    const stratum::osm::road::Corridor corridor = p7::kerbed_corridor();
    Mesh mesh = corridor.mesh;
    if (mesh.vertices.empty()) {
        CHECK_TRUE(false);
        return;
    }

    // Precondition: the fixture really does carry creases. Without this the test
    // would pass vacuously on a profile that lost its kerb upstream.
    const auto before = normals_by_position(mesh);
    std::vector<p7::PosKey> creases;
    for (const auto& entry : before) {
        if (entry.second.size() >= 2 && holds_a_crease(entry.second)) {
            creases.push_back(entry.first);
        }
    }
    CHECK_TRUE(!creases.empty());
    if (creases.empty()) return;

    weld_vertices(mesh);

    const auto after = normals_by_position(mesh);
    size_t destroyed = 0;
    for (const p7::PosKey& key : creases) {
        const auto it = after.find(key);
        if (it == after.end() || it->second.size() < 2 || !holds_a_crease(it->second)) {
            ++destroyed;
        }
    }
    if (destroyed != 0) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "no kerb crease was welded smooth",
            std::to_string(destroyed) + " of " + std::to_string(creases.size()) +
                " creased positions lost their second normal");
    }
}

/**
 * Two vertices whose referencing material sets differ never weld, and the key is
 * a mask rather than a single id.
 *
 * Three coincident vertices, identical in every attribute. Two are referenced by
 * Asphalt and Curb, one by Asphalt alone. Exactly the first two may weld.
 */
TEST(MeshOptimize, weld_respects_the_material_mask) {
    Mesh mesh = make_mask_mesh();
    CHECK_EQ(vertices_at_origin(mesh), size_t{3});

    const Mesh before = mesh;
    const size_t removed = weld_vertices(mesh, WeldConfig{});

    CHECK_EQ(removed, size_t{1});
    CHECK_EQ(vertices_at_origin(mesh), size_t{2});
    CHECK_EQ(mesh.indices.size(), before.indices.size());
    check_ranges_unchanged(before, mesh, "mask weld");
    check_tiling(mesh, "mask weld");
    CHECK_TRUE(p7::indices_are_sane(mesh));
}

/**
 * With respect_material off, the same three vertices collapse to one.
 *
 * This is the collision builder's configuration, and it is what proves the
 * previous test measured the material rule rather than some other epsilon
 * refusing the weld.
 */
TEST(MeshOptimize, weld_ignores_materials_when_asked_to) {
    Mesh mesh = make_mask_mesh();
    const Mesh before = mesh;

    WeldConfig cfg;
    cfg.respect_material = false;
    const size_t removed = weld_vertices(mesh, cfg);

    CHECK_EQ(removed, size_t{2});
    CHECK_EQ(vertices_at_origin(mesh), size_t{1});
    CHECK_EQ(mesh.indices.size(), before.indices.size());
    check_ranges_unchanged(before, mesh, "material-blind weld");
}

/**
 * A mesh with nothing to weld comes back byte-identical, and a mesh with no
 * indices is left alone.
 */
TEST(MeshOptimize, weld_of_clean_and_empty_meshes_is_a_no_op) {
    Mesh slab = p7::make_slab_mesh(3, 4);
    const Mesh before = slab;
    CHECK_EQ(weld_vertices(slab), size_t{0});
    CHECK_EQ(slab.vertices.size(), before.vertices.size());
    check_ranges_unchanged(before, slab, "clean slab");

    Mesh empty;
    CHECK_EQ(weld_vertices(empty), size_t{0});
    CHECK_TRUE(empty.vertices.empty());
    CHECK_TRUE(empty.indices.empty());

    Mesh no_indices;
    no_indices.vertices.resize(4);
    CHECK_EQ(weld_vertices(no_indices), size_t{0});
    CHECK_EQ(no_indices.vertices.size(), size_t{4});
}

// ============================================================================
// Reordering
// ============================================================================

/**
 * optimize_mesh() may move every triangle and renumber every vertex, and must
 * still hand back the same triangles under the same materials.
 *
 * A whole-buffer meshopt_optimizeVertexCache() call passes a triangle-count test
 * and fails this one: it moves triangles across range boundaries, so afterwards
 * the Curb range points at carriageway triangles. Comparing the multiset of
 * (material, position triple) is what catches it.
 */
TEST(MeshOptimize, optimize_preserves_the_triangle_set_and_its_materials) {
    Mesh mesh = p7::make_slab_mesh();
    const Mesh before = mesh;
    const std::vector<p7::TriKey> content_before = p7::triangle_multiset(mesh);

    optimize_mesh(mesh);

    CHECK_EQ(mesh.indices.size(), before.indices.size());
    CHECK_EQ(mesh.vertices.size(), before.vertices.size());
    check_ranges_unchanged(before, mesh, "optimize of a slab");
    check_tiling(mesh, "optimized slab");
    CHECK_TRUE(p7::indices_are_sane(mesh));

    const std::vector<p7::TriKey> content_after = p7::triangle_multiset(mesh);
    CHECK_EQ(content_after.size(), content_before.size());
    CHECK_TRUE(content_after == content_before);

    // Stated separately from the multiset compare so a failure names the material
    // that moved rather than only reporting that something did.
    for (MaterialId m : { MaterialId::Asphalt, MaterialId::Curb, MaterialId::Sidewalk }) {
        CHECK_EQ(p7::triangles_with_material(mesh, m),
                 p7::triangles_with_material(before, m));
    }
}

/**
 * The same guarantee on real corridor geometry, which has ten strips across three
 * materials and unevenly sized ranges.
 */
TEST(MeshOptimize, optimize_preserves_a_corridor) {
    Mesh mesh = p7::kerbed_corridor().mesh;
    if (mesh.indices.empty()) {
        CHECK_TRUE(false);
        return;
    }
    const Mesh before = mesh;
    const std::vector<p7::TriKey> content_before = p7::triangle_multiset(mesh);

    optimize_mesh(mesh);

    CHECK_EQ(mesh.indices.size(), before.indices.size());
    check_ranges_unchanged(before, mesh, "optimize of a corridor");
    CHECK_TRUE(p7::triangle_multiset(mesh) == content_before);
    CHECK_TRUE(p7::mesh_is_finite(mesh));
}

/**
 * A range whose offset or count is not a whole number of triangles is skipped
 * rather than reordered, and the mesh survives.
 */
TEST(MeshOptimize, optimize_skips_a_malformed_range) {
    Mesh mesh = p7::make_slab_mesh(2, 2);
    mesh.submeshes.clear();
    mesh.submeshes.push_back(SubMesh{ 0u, 4u, MaterialId::Asphalt });    // not a multiple of 3
    mesh.submeshes.push_back(SubMesh{ 4u, static_cast<uint32_t>(mesh.indices.size()) - 4u,
                                      MaterialId::Curb });
    const Mesh before = mesh;

    optimize_mesh(mesh);

    CHECK_EQ(mesh.indices.size(), before.indices.size());
    check_ranges_unchanged(before, mesh, "malformed range");
    CHECK_TRUE(p7::indices_are_sane(mesh));
}

// ============================================================================
// LOD chains
// ============================================================================

/**
 * Every level is coarser than the one before it, and no level has lost a
 * material.
 *
 * The second half is the one that matters. Simplification runs per range and
 * reassembles, so a range that came back empty is silently dropped -- and a chain
 * whose second level has no Curb range is a network whose kerbs vanish at 450 m.
 */
TEST(MeshOptimize, lod_levels_shrink_and_keep_every_material) {
    const Mesh slab = p7::make_slab_mesh();
    const std::set<MaterialId> expected = p7::materials_of(slab);
    CHECK_EQ(expected.size(), size_t{3});

    const LodChain chain = build_lod_chain(slab);
    CHECK_TRUE(chain.is_valid());
    if (!chain.is_valid()) return;

    // Level 0 is the full-detail mesh: same triangles as the input.
    CHECK_EQ(p7::triangle_count(chain.levels[0]), p7::triangle_count(slab));
    CHECK_TRUE(p7::triangle_multiset(chain.levels[0]) == p7::triangle_multiset(slab));

    // The slab has real interior vertices, so it must simplify at all.
    CHECK_TRUE(chain.levels.size() >= 2);

    for (size_t i = 0; i < chain.levels.size(); ++i) {
        const Mesh& level = chain.levels[i];
        const std::string what = "level " + std::to_string(i);

        check_tiling(level, what.c_str());
        CHECK_TRUE(p7::indices_are_sane(level));
        CHECK_TRUE(p7::mesh_is_finite(level));
        CHECK_TRUE(p7::triangle_count(level) > 0);

        const std::set<MaterialId> present = p7::materials_of(level);
        for (MaterialId m : expected) {
            if (present.count(m) == 0) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "level keeps every material of level 0",
                    what + " lost " + material_id_name(m));
            }
        }

        if (i > 0) {
            const size_t previous = p7::triangle_count(chain.levels[i - 1]);
            const size_t current = p7::triangle_count(level);
            if (!(current < previous)) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "level is coarser than the one before it",
                    what + " has " + std::to_string(current) + " triangles, level " +
                        std::to_string(i - 1) + " has " + std::to_string(previous));
            }
            // A level that failed to beat the previous one by 10% is documented
            // as dropped, so any level that survived beat it.
            CHECK_TRUE(static_cast<double>(current) <= 0.9 * static_cast<double>(previous));
        }
    }
}

/**
 * With lock_borders on, no level introduces a boundary position level 0 did not
 * already have.
 *
 * This is the crack test. Road geometry is chunked, and two adjacent chunks
 * simplified independently would each pull their shared edge inward by a
 * different amount. A pulled-in border shows up here as a boundary position that
 * is not in level 0's boundary set, because a border vertex that moved left its
 * old position behind.
 *
 * Boundaries are computed PER SUBMESH, since build_lod_chain() simplifies per
 * range and a material boundary is an open border of its own range.
 */
TEST(MeshOptimize, lod_locks_borders) {
    LodConfig cfg;
    cfg.lock_borders = true;

    const LodChain chain = build_lod_chain(p7::make_slab_mesh(), cfg);
    CHECK_TRUE(chain.levels.size() >= 2);
    if (chain.levels.size() < 2) return;

    const std::set<p7::PosKey> level0 = p7::border_positions(chain.levels[0]);
    CHECK_TRUE(!level0.empty());

    for (size_t i = 1; i < chain.levels.size(); ++i) {
        const std::set<p7::PosKey> border = p7::border_positions(chain.levels[i]);
        size_t escaped = 0;
        for (const p7::PosKey& key : border) {
            if (level0.count(key) == 0) ++escaped;
        }
        if (escaped != 0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "no new border position at this level",
                "level " + std::to_string(i) + " has " + std::to_string(escaped) +
                    " boundary positions that level 0 did not, of " +
                    std::to_string(border.size()));
        }
    }
}

/**
 * Switch distances are as long as the level list, ascend, and start at zero.
 *
 * The first level is what you draw when you are standing on it, so its threshold
 * is 0 by definition. Everything after it is derived from the achieved ratio, and
 * a chain whose thresholds do not ascend would make level_for_distance() pick a
 * finer level further away.
 */
TEST(MeshOptimize, lod_thresholds_ascend_from_zero) {
    const LodChain chain = build_lod_chain(p7::make_slab_mesh());
    CHECK_TRUE(chain.is_valid());
    if (!chain.is_valid()) return;

    CHECK_EQ(chain.screen_thresholds.size(), chain.levels.size());
    if (chain.screen_thresholds.empty()) return;

    CHECK_NEAR(chain.screen_thresholds[0], 0.0, 1e-6);
    for (size_t i = 1; i < chain.screen_thresholds.size(); ++i) {
        CHECK_TRUE(chain.screen_thresholds[i] > chain.screen_thresholds[i - 1]);
        CHECK_TRUE(std::isfinite(chain.screen_thresholds[i]));
    }

    // The switch factor is the documented scale, so a level of ratio q on a mesh
    // of radius r switches at r * kLodSwitchFactor / sqrt(q). The achieved ratio
    // is never worse than 1, so the first switch cannot be nearer than r * 8.
    const float radius = chain.levels[0].bounds.radius();
    CHECK_TRUE(radius > 0.0f);
    if (chain.screen_thresholds.size() >= 2) {
        CHECK_TRUE(chain.screen_thresholds[1] >= radius * kLodSwitchFactor * 0.999f);
    }
}

/**
 * level_for_distance() picks the coarsest level whose threshold has been reached,
 * and answers 0 for an empty chain rather than indexing an empty vector.
 */
TEST(MeshOptimize, level_for_distance_picks_the_coarsest_reached) {
    LodChain chain;
    CHECK_EQ(chain.level_for_distance(1e9f), size_t{0});
    CHECK_FALSE(chain.is_valid());

    chain.levels.resize(3);
    chain.screen_thresholds = { 0.0f, 100.0f, 400.0f };
    CHECK_EQ(chain.level_for_distance(0.0f), size_t{0});
    CHECK_EQ(chain.level_for_distance(99.9f), size_t{0});
    CHECK_EQ(chain.level_for_distance(100.0f), size_t{1});
    CHECK_EQ(chain.level_for_distance(399.9f), size_t{1});
    CHECK_EQ(chain.level_for_distance(400.0f), size_t{2});
    CHECK_EQ(chain.level_for_distance(1e9f), size_t{2});
}

/**
 * A single quad has nothing to simplify, so the chain is exactly one level and
 * nothing crashes on the way there.
 *
 * Two triangles at a ratio of 0.5 is one triangle, which is not a surface; every
 * proposed level is therefore dropped and the chain degrades to "always draw
 * level 0". That degradation is the documented behaviour and is what a piece made
 * of a single dead-end cap actually hits.
 */
TEST(MeshOptimize, tiny_mesh_gives_a_one_level_chain) {
    const Mesh quad = p7::make_quad_mesh();
    CHECK_EQ(p7::triangle_count(quad), size_t{2});

    const LodChain chain = build_lod_chain(quad);
    CHECK_TRUE(chain.is_valid());
    CHECK_EQ(chain.levels.size(), size_t{1});
    CHECK_EQ(chain.screen_thresholds.size(), size_t{1});
    if (!chain.levels.empty()) {
        CHECK_EQ(p7::triangle_count(chain.levels[0]), size_t{2});
        CHECK_EQ(p7::materials_of(chain.levels[0]).size(), size_t{1});
    }
    CHECK_EQ(chain.level_for_distance(10000.0f), size_t{0});
}

/**
 * A mesh with no triangles produces an empty chain rather than a level 0 nothing
 * can be drawn from.
 */
TEST(MeshOptimize, empty_mesh_gives_an_empty_chain) {
    const Mesh empty;
    const LodChain chain = build_lod_chain(empty);
    CHECK_FALSE(chain.is_valid());
    CHECK_TRUE(chain.levels.empty());
}

/**
 * An empty ratio list asks for no simplification at all, and gets a chain holding
 * only the full-detail level.
 */
TEST(MeshOptimize, no_ratios_gives_only_the_full_detail_level) {
    LodConfig cfg;
    cfg.ratios.clear();

    const LodChain chain = build_lod_chain(p7::make_slab_mesh(4, 6), cfg);
    CHECK_TRUE(chain.is_valid());
    CHECK_EQ(chain.levels.size(), size_t{1});
    CHECK_EQ(chain.screen_thresholds.size(), size_t{1});
}
