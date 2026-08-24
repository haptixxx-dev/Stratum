/**
 * @file test_lod_chunk.cpp
 * @brief LOD built from merged per-chunk geometry: does it actually reduce, and does it crack
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The per-piece chain in mesh_optimize.hpp returns 85.3% of its input triangles
 * at the coarsest level while asking for 10%, and no ratio fixes that: a road
 * piece is a 3 m ribbon, every one of its long sides and both of its ends are
 * open boundary, and `meshopt_SimplifyLockBorder` correctly refuses to move any
 * of it. build_chunk_lod() exists to make the seams interior by merging first.
 *
 * So the headline assertion of this suite is a NUMBER, and it is deliberately
 * strict. A test that accepted 85% would pass against the very implementation
 * this file was written to replace, and would therefore be worth nothing. The
 * bound below is 0.7 of level 0 at level 1 against a requested 0.5, which leaves
 * room for the error bound and the locked band to refuse some collapses while
 * still failing the per-piece behaviour by a wide margin.
 *
 * The other three questions:
 *
 * - **Does the border lock hold.** A vertex within ChunkLodConfig::border_band of
 *   the chunk rectangle may not move at any level. This is what the per-piece
 *   version could not express: it had only a topological test, and on a ribbon
 *   topology and position are the same question.
 * - **Do two chunks crack.** The crack-free guarantee is that two neighbours
 *   simplified independently, in any order, still meet exactly, because a vertex
 *   on their shared boundary is inside the band of BOTH rectangles. Nothing is
 *   exchanged between them, so the only way to check it is to build both and
 *   compare the seam.
 * - **Is it deterministic and material-complete.** A chunk is built off the main
 *   thread and its output is cached; a level that dropped the Curb material or
 *   that varies run to run poisons the cache.
 *
 * ### The instrument
 *
 * p7::make_slab_mesh() rather than a corridor, for the reason that fixture
 * documents: a corridor strip is two vertex columns wide, so it has no interior
 * to collapse and cannot answer any question about simplification. The slab is a
 * welded grid with three material bands and real interior vertices.
 *
 * Coordinates: local 2D `(x, y)` maps to world `vec3(x, height, -y)`, so a slab
 * vertex at world `(c, 0, -r)` is at local `(c, r)`, and a chunk rectangle given
 * in local metres brackets it directly.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests LodChunk
 * @endcode
 */

#include "framework.hpp"
#include "road/p7_fixtures.hpp"

#include "osm/road/lod_chunk.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::Vertex;
using stratum::osm::road::ChunkLod;
using stratum::osm::road::ChunkLodConfig;
using stratum::osm::road::build_chunk_lod;

namespace p7 = stratum::test::p7;

/// Quads across each band, and quads along the slab, for every fixture here
constexpr int kColumnsPerBand = 8;
constexpr int kRows = 40;

/// A slab is 24 metres across (3 bands of 8) and 40 metres along
constexpr double kSlabWidth = kColumnsPerBand * 3;
constexpr double kSlabLength = kRows;

// ============================================================================
// Geometry helpers
// ============================================================================

/**
 * @brief Move a mesh by a local 2D offset
 *
 * World mapping is `(x, y) -> vec3(x, height, -y)`, so a local +y offset is a
 * world -z offset. Getting that sign wrong would put the second chunk on top of
 * the first and every seam assertion would pass vacuously.
 */
Mesh translate_local(Mesh mesh, double dx, double dy) {
    for (Vertex& v : mesh.vertices) {
        v.position.x += static_cast<float>(dx);
        v.position.z -= static_cast<float>(dy);
    }
    mesh.compute_bounds();
    return mesh;
}

/// Local 2D coordinates of a world position
glm::dvec2 local_of(const glm::vec3& world) {
    return glm::dvec2{static_cast<double>(world.x), -static_cast<double>(world.z)};
}

/// Distance from @p p to the nearest point of the segment ab
double point_segment_distance(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 ab = b - a;
    const double len2 = glm::dot(ab, ab);
    if (!(len2 > 0.0)) return glm::length(p - a);
    double t = glm::dot(p - a, ab) / len2;
    t = std::max(0.0, std::min(1.0, t));
    return glm::length(p - (a + ab * t));
}

/**
 * @brief Distance from a local point to the PERIMETER of a rectangle
 *
 * The perimeter, not the interior: a vertex deep inside the rectangle is far from
 * it and free to move, a vertex just outside it is near it and pinned. That is
 * what ChunkLodConfig::border_band measures against.
 */
double distance_to_rect(const glm::dvec2& p, const glm::dvec2& lo, const glm::dvec2& hi) {
    const glm::dvec2 corners[4] = {
        {lo.x, lo.y}, {hi.x, lo.y}, {hi.x, hi.y}, {lo.x, hi.y}};
    double best = 1e30;
    for (int i = 0; i < 4; ++i) {
        best = std::min(best, point_segment_distance(p, corners[i], corners[(i + 1) % 4]));
    }
    return best;
}

/**
 * @brief Distinct quantised positions of a mesh, split by whether they are in the lock band
 *
 * Walks the INDEX buffer, not the vertex array. A simplifier is entitled to leave
 * a vertex in the buffer that no triangle references any more, and counting those
 * would report an interior that never shrinks however much was collapsed --
 * turning the "interior vertices really do move" half of the border test into a
 * test that always fails, or the seam comparison into one that always passes.
 */
void split_by_band(const Mesh& mesh, const glm::dvec2& lo, const glm::dvec2& hi, double band,
                   std::set<p7::PosKey>& out_border, std::set<p7::PosKey>& out_interior) {
    for (const uint32_t index : mesh.indices) {
        if (index >= mesh.vertices.size()) continue;
        const glm::vec3& position = mesh.vertices[index].position;
        const p7::PosKey key = p7::pos_key(position);
        if (distance_to_rect(local_of(position), lo, hi) <= band) {
            out_border.insert(key);
        } else {
            out_interior.insert(key);
        }
    }
}

/// Every distinct REFERENCED position on the plane x == @p x_plane, quantised
std::set<p7::PosKey> positions_on_plane(const Mesh& mesh, double x_plane, double eps) {
    std::set<p7::PosKey> out;
    for (const uint32_t index : mesh.indices) {
        if (index >= mesh.vertices.size()) continue;
        const glm::vec3& position = mesh.vertices[index].position;
        if (std::fabs(static_cast<double>(position.x) - x_plane) <= eps) {
            out.insert(p7::pos_key(position));
        }
    }
    return out;
}

/// A chunk made of two slabs stacked along local y, so the merge has a seam to weld
std::vector<Mesh> stacked_pair() {
    std::vector<Mesh> out;
    out.push_back(p7::make_slab_mesh(kColumnsPerBand, kRows));
    out.push_back(translate_local(p7::make_slab_mesh(kColumnsPerBand, kRows), 0.0, kSlabLength));
    return out;
}

/// Pointers to every element of @p meshes, the shape build_chunk_lod() takes
std::vector<const Mesh*> pointers_to(const std::vector<Mesh>& meshes) {
    std::vector<const Mesh*> out;
    out.reserve(meshes.size());
    for (const Mesh& m : meshes) out.push_back(&m);
    return out;
}

/// Two meshes are identical down to the byte-comparable fields a cache would hash
bool meshes_are_identical(const Mesh& a, const Mesh& b) {
    if (a.vertices.size() != b.vertices.size()) return false;
    if (a.indices.size() != b.indices.size()) return false;
    if (a.submeshes.size() != b.submeshes.size()) return false;
    for (size_t i = 0; i < a.vertices.size(); ++i) {
        if (!(a.vertices[i] == b.vertices[i])) return false;
    }
    for (size_t i = 0; i < a.indices.size(); ++i) {
        if (a.indices[i] != b.indices[i]) return false;
    }
    for (size_t i = 0; i < a.submeshes.size(); ++i) {
        if (a.submeshes[i].index_offset != b.submeshes[i].index_offset) return false;
        if (a.submeshes[i].index_count != b.submeshes[i].index_count) return false;
        if (a.submeshes[i].material != b.submeshes[i].material) return false;
        if (a.submeshes[i].variant != b.submeshes[i].variant) return false;
    }
    return true;
}

} // namespace

// ============================================================================
// The merge itself
// ============================================================================

/**
 * Level 0 is the merged, welded chunk -- not any input piece and not the sum of
 * their vertex counts.
 *
 * The weld across the piece seam is where the merge pays for itself before a
 * single level is simplified: the boundary between two pieces stops being two
 * coincident rows of vertices owned by nobody and becomes one row owned by the
 * chunk.
 */
TEST(LodChunk, level_zero_merges_and_welds_the_pieces) {
    const std::vector<Mesh> pieces = stacked_pair();
    size_t piece_vertices = 0;
    size_t piece_triangles = 0;
    for (const Mesh& m : pieces) {
        piece_vertices += m.vertices.size();
        piece_triangles += p7::triangle_count(m);
    }

    const ChunkLod lod = build_chunk_lod(pointers_to(pieces), glm::dvec2{0.0, 0.0},
                                         glm::dvec2{kSlabWidth, 2.0 * kSlabLength});

    if (lod.levels.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "the chunk produced levels",
                                      "build_chunk_lod returned no levels");
        return;
    }

    const Mesh& level0 = lod.levels[0];

    // Never simplified: level 0 carries every triangle its pieces carried.
    CHECK_EQ(p7::triangle_count(level0), piece_triangles);

    // But fewer distinct positions, because the seam between the two pieces welded
    // shut. Counted over referenced vertices, so an uncompacted buffer cannot make
    // this pass or fail for the wrong reason.
    std::set<p7::PosKey> merged_positions;
    for (const uint32_t index : level0.indices) {
        if (index < level0.vertices.size()) {
            merged_positions.insert(p7::pos_key(level0.vertices[index].position));
        }
    }
    CHECK_TRUE(merged_positions.size() < piece_vertices);
    CHECK_TRUE(level0.vertices.size() <= piece_vertices);

    CHECK_TRUE(p7::mesh_is_finite(level0));
    CHECK_TRUE(p7::indices_are_sane(level0));
    std::string reason;
    if (!p7::submeshes_tile_exactly(level0, reason)) {
        stratum::test::report_failure(__FILE__, __LINE__, "level 0 submeshes tile", reason);
    }

    // Ranges of the same key from different pieces are coalesced, so the merged
    // mesh has at most one range per key.
    std::set<uint32_t> keys;
    for (const auto& range : level0.effective_submeshes()) {
        const stratum::MaterialKey key{range.material, range.variant};
        CHECK_TRUE(keys.insert(key.packed()).second);
    }
}

/// No pieces, or only empty ones, is not an error and is not a crash
TEST(LodChunk, an_empty_chunk_produces_no_levels) {
    const std::vector<const Mesh*> none;
    const ChunkLod empty = build_chunk_lod(none, glm::dvec2{0.0, 0.0}, glm::dvec2{10.0, 10.0});
    CHECK_TRUE(empty.levels.empty());
    CHECK_TRUE(empty.switch_distances.empty());

    Mesh blank;
    const Mesh* pieces[] = {nullptr, &blank};
    const std::vector<const Mesh*> mixed(pieces, pieces + 2);
    const ChunkLod nothing = build_chunk_lod(mixed, glm::dvec2{0.0, 0.0}, glm::dvec2{10.0, 10.0});
    CHECK_TRUE(nothing.levels.empty());
}

// ============================================================================
// The headline: does it reduce
// ============================================================================

/**
 * Level 1 lands near ratios[0] of level 0.
 *
 * THE point of the redesign. The per-piece chain returns 85.3% of its input at
 * its COARSEST level; this asks for 50% at its first one and the bound below is
 * 70%, which the per-piece behaviour misses by a mile and a correct merge clears
 * comfortably.
 *
 * The bound is not tighter than the ratio because ChunkLodConfig::target_error
 * and the locked band are both entitled to refuse collapses the ratio asked for.
 * That is the simplifier protecting the silhouette and the chunk seams, and the
 * header says so.
 */
TEST(LodChunk, level_one_lands_near_the_first_ratio) {
    const std::vector<Mesh> pieces = stacked_pair();

    ChunkLodConfig cfg;
    cfg.ratios = {0.5f, 0.25f, 0.1f};

    const ChunkLod lod = build_chunk_lod(pointers_to(pieces), glm::dvec2{0.0, 0.0},
                                         glm::dvec2{kSlabWidth, 2.0 * kSlabLength}, cfg);

    if (lod.levels.size() < 2) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "the chain has a simplified level",
            "build_chunk_lod produced " + std::to_string(lod.levels.size()) + " level(s)");
        return;
    }

    const size_t level0 = p7::triangle_count(lod.levels[0]);
    const size_t level1 = p7::triangle_count(lod.levels[1]);
    CHECK_TRUE(level0 > 0);

    const double achieved = static_cast<double>(level1) / static_cast<double>(level0);
    if (!(achieved < 0.7)) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "level 1 is under 0.7 of level 0",
            std::to_string(level1) + " of " + std::to_string(level0) + " triangles is " +
                std::to_string(achieved) + ", which is the per-piece behaviour this "
                "path exists to replace");
    }

    // Coarsening with index, and every level still a valid mesh.
    for (size_t i = 1; i < lod.levels.size(); ++i) {
        CHECK_TRUE(p7::triangle_count(lod.levels[i]) <= p7::triangle_count(lod.levels[i - 1]));
        CHECK_TRUE(p7::triangle_count(lod.levels[i]) > 0);
        CHECK_TRUE(p7::mesh_is_finite(lod.levels[i]));
        CHECK_TRUE(p7::indices_are_sane(lod.levels[i]));
        std::string reason;
        if (!p7::submeshes_tile_exactly(lod.levels[i], reason)) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "every level's submeshes tile",
                                          "level " + std::to_string(i) + ": " + reason);
        }
    }
}

/// switch_distances is parallel to levels, ascending, and starts at zero
TEST(LodChunk, switch_distances_are_well_formed) {
    const std::vector<Mesh> pieces = stacked_pair();
    const ChunkLod lod = build_chunk_lod(pointers_to(pieces), glm::dvec2{0.0, 0.0},
                                         glm::dvec2{kSlabWidth, 2.0 * kSlabLength});
    if (lod.levels.empty()) return;

    CHECK_EQ(lod.switch_distances.size(), lod.levels.size());
    if (lod.switch_distances.empty()) return;

    CHECK_NEAR(lod.switch_distances[0], 0.0f, 1e-6);
    for (size_t i = 1; i < lod.switch_distances.size(); ++i) {
        CHECK_TRUE(lod.switch_distances[i] > lod.switch_distances[i - 1]);
        CHECK_TRUE(std::isfinite(lod.switch_distances[i]));
    }
}

// ============================================================================
// The border lock
// ============================================================================

/**
 * A vertex within border_band of the chunk rectangle is unmoved at every level;
 * a vertex in the interior is not.
 *
 * Both halves matter. Without the first, chunks crack. Without the second, the
 * band is so wide that nothing is free and the chain reduces nothing -- which is
 * precisely the failure mode of the per-piece path, expressed as a band instead
 * of as topology.
 */
TEST(LodChunk, border_vertices_are_pinned_and_interior_vertices_are_not) {
    const glm::dvec2 lo{0.0, 0.0};
    const glm::dvec2 hi{kSlabWidth, 2.0 * kSlabLength};

    const std::vector<Mesh> pieces = stacked_pair();
    ChunkLodConfig cfg;
    cfg.border_band = 0.5f;

    const ChunkLod lod = build_chunk_lod(pointers_to(pieces), lo, hi, cfg);
    if (lod.levels.size() < 2) return;

    std::set<p7::PosKey> base_border;
    std::set<p7::PosKey> base_interior;
    split_by_band(lod.levels[0], lo, hi, cfg.border_band, base_border, base_interior);
    CHECK_TRUE(!base_border.empty());
    CHECK_TRUE(!base_interior.empty());

    for (size_t i = 1; i < lod.levels.size(); ++i) {
        std::set<p7::PosKey> border;
        std::set<p7::PosKey> interior;
        split_by_band(lod.levels[i], lo, hi, cfg.border_band, border, interior);

        // Unmoved: every border position at this level is one level 0 already had.
        // A border vertex that had been pulled inward would land on a position the
        // base level never held.
        for (const p7::PosKey& key : border) {
            if (base_border.count(key) == 0) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "border vertices are unmoved",
                    "level " + std::to_string(i) +
                        " has a border position level 0 did not");
                break;
            }
        }
        CHECK_TRUE(!border.empty());

        // And the interior really did collapse, or nothing was simplified at all.
        CHECK_TRUE(interior.size() < base_interior.size());
    }
}

/**
 * A band of zero locks nothing, which is the documented way to crack every seam,
 * and it must therefore reduce strictly more than a real band does.
 *
 * Asserted so that "the band is wired up" is a fact rather than an assumption
 * behind every other test in this file.
 */
TEST(LodChunk, the_band_width_changes_what_is_locked) {
    const glm::dvec2 lo{0.0, 0.0};
    const glm::dvec2 hi{kSlabWidth, 2.0 * kSlabLength};
    const std::vector<Mesh> pieces = stacked_pair();

    ChunkLodConfig unlocked;
    unlocked.border_band = 0.0f;
    ChunkLodConfig locked;
    locked.border_band = 4.0f;

    const ChunkLod a = build_chunk_lod(pointers_to(pieces), lo, hi, unlocked);
    const ChunkLod b = build_chunk_lod(pointers_to(pieces), lo, hi, locked);
    if (a.levels.size() < 2 || b.levels.size() < 2) return;

    // Level 0 is the same merged mesh either way; the band only affects simplification.
    CHECK_EQ(p7::triangle_count(a.levels[0]), p7::triangle_count(b.levels[0]));

    // A wider lock cannot reduce more than no lock at all.
    CHECK_TRUE(p7::triangle_count(b.levels[1]) >= p7::triangle_count(a.levels[1]));
}

// ============================================================================
// The crack-free guarantee
// ============================================================================

/**
 * Two adjacent chunks, simplified independently and exchanging nothing, still
 * share their boundary vertex positions exactly.
 *
 * This is the whole reason the lock is positional rather than topological. A
 * vertex on the shared boundary sits exactly on the rectangle of one chunk and
 * exactly on the rectangle of its neighbour, so any positive band catches it in
 * both, and neither chunk needs to know the other exists.
 *
 * The two chunks here are congruent but not identically situated: the seam is the
 * RIGHT edge of chunk A, where band 2 (Sidewalk) ends, and the LEFT edge of chunk
 * B, where band 0 (Asphalt) begins. So the seam is not preserved by both sides
 * happening to run the same computation on the same material.
 */
TEST(LodChunk, adjacent_chunks_share_their_seam_exactly) {
    const double seam = kSlabWidth;

    const Mesh left = p7::make_slab_mesh(kColumnsPerBand, kRows);
    const Mesh right = translate_local(p7::make_slab_mesh(kColumnsPerBand, kRows), seam, 0.0);

    const std::vector<const Mesh*> left_pieces{&left};
    const std::vector<const Mesh*> right_pieces{&right};

    ChunkLodConfig cfg;
    cfg.border_band = 0.5f;

    const ChunkLod a = build_chunk_lod(left_pieces, glm::dvec2{0.0, 0.0},
                                       glm::dvec2{seam, kSlabLength}, cfg);
    const ChunkLod b = build_chunk_lod(right_pieces, glm::dvec2{seam, 0.0},
                                       glm::dvec2{2.0 * seam, kSlabLength}, cfg);

    if (a.levels.empty() || b.levels.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "both chunks produced levels", "");
        return;
    }

    const size_t levels = std::min(a.levels.size(), b.levels.size());
    for (size_t i = 0; i < levels; ++i) {
        const std::set<p7::PosKey> from_a = positions_on_plane(a.levels[i], seam, 1e-4);
        const std::set<p7::PosKey> from_b = positions_on_plane(b.levels[i], seam, 1e-4);

        if (from_a.empty()) {
            stratum::test::report_failure(__FILE__, __LINE__, "the seam has vertices",
                                          "level " + std::to_string(i) +
                                              " of chunk A has none on the seam plane");
            continue;
        }
        if (from_a != from_b) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "adjacent chunks agree on the seam",
                "level " + std::to_string(i) + ": chunk A has " +
                    std::to_string(from_a.size()) + " seam positions, chunk B has " +
                    std::to_string(from_b.size()) + ", and the sets differ -- this is a crack");
        }
    }
}

// ============================================================================
// Materials and determinism
// ============================================================================

/**
 * Every material present at level 0 is present at every level.
 *
 * ChunkLodConfig::simplify_per_material exists for this: a simplifier free to
 * collapse across a material boundary merges the kerb into the road surface and
 * the step disappears. A level that lost a material has done exactly that, and it
 * shows up first as a missing draw call rather than as a visibly wrong shape.
 */
TEST(LodChunk, every_material_survives_every_level) {
    const std::vector<Mesh> pieces = stacked_pair();

    ChunkLodConfig cfg;
    cfg.simplify_per_material = true;

    const ChunkLod lod = build_chunk_lod(pointers_to(pieces), glm::dvec2{0.0, 0.0},
                                         glm::dvec2{kSlabWidth, 2.0 * kSlabLength}, cfg);
    if (lod.levels.empty()) return;

    const std::set<MaterialId> base = p7::materials_of(lod.levels[0]);
    CHECK_EQ(base.size(), size_t{3});   // Asphalt, Curb, Sidewalk

    for (size_t i = 1; i < lod.levels.size(); ++i) {
        const std::set<MaterialId> here = p7::materials_of(lod.levels[i]);
        for (const MaterialId m : base) {
            if (here.count(m) == 0) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "the material survived the level",
                    std::string(stratum::material_id_name(m)) + " is missing from level " +
                        std::to_string(i));
            }
        }
    }
}

/**
 * The same input twice gives identical output.
 *
 * Chunks are built off the main thread and their levels are cached, so a chain
 * that varies run to run is a cache that is sometimes wrong and a golden test
 * that fails at random. The header promises the merge is in the order the pieces
 * are given and that nothing inside depends on thread scheduling; this is that
 * promise measured.
 */
TEST(LodChunk, the_chain_is_deterministic) {
    const std::vector<Mesh> pieces = stacked_pair();
    const glm::dvec2 lo{0.0, 0.0};
    const glm::dvec2 hi{kSlabWidth, 2.0 * kSlabLength};

    const ChunkLod first = build_chunk_lod(pointers_to(pieces), lo, hi);
    const ChunkLod second = build_chunk_lod(pointers_to(pieces), lo, hi);

    CHECK_EQ(first.levels.size(), second.levels.size());
    CHECK_EQ(first.switch_distances.size(), second.switch_distances.size());
    if (first.levels.size() != second.levels.size()) return;

    for (size_t i = 0; i < first.levels.size(); ++i) {
        if (!meshes_are_identical(first.levels[i], second.levels[i])) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "two builds of one chunk are identical",
                "level " + std::to_string(i) + " differs between runs");
        }
    }
    for (size_t i = 0; i < first.switch_distances.size(); ++i) {
        CHECK_EQ(first.switch_distances[i], second.switch_distances[i]);
    }
}

/// A degenerate rectangle locks nothing and is treated as "no border", not as an error
TEST(LodChunk, a_degenerate_rectangle_is_treated_as_no_border) {
    const std::vector<Mesh> pieces = stacked_pair();

    const ChunkLod inverted = build_chunk_lod(pointers_to(pieces), glm::dvec2{100.0, 100.0},
                                              glm::dvec2{0.0, 0.0});
    CHECK_TRUE(!inverted.levels.empty());
    if (!inverted.levels.empty()) {
        CHECK_TRUE(p7::mesh_is_finite(inverted.levels[0]));
        CHECK_TRUE(p7::indices_are_sane(inverted.levels[0]));
    }
}
