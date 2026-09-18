// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file building_lod.cpp
 * @brief Implementation of the per-building LOD chain
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * See building_lod.hpp for the contract and for the measurements that shaped it.
 * This file records what the implementation had to do that the header only
 * states, and it borrows three pieces of osm/road/lod_chunk.cpp outright:
 *
 * 1. **The lock dilation.** meshoptimizer's `vertex_lock` promises that a vertex
 *    will not MOVE. It does not promise the vertex will survive: collapsing the
 *    one unlocked corner of a triangle flattens it, and a flattened triangle is
 *    dropped, taking its locked vertices with it if nothing else names them.
 *    Locking all three corners of any triangle that has one leaves no legal
 *    collapse source. lod_chunk.cpp measured 130 of 149 pinned vertices lost
 *    without it.
 * 2. **The position-wedge propagation.** A position carrying several wedges --
 *    a wall corner and a roof corner at the same point, which the weld keeps
 *    apart because their normals differ -- must have all of its wedges locked or
 *    none. meshopt_simplifySloppy's own documentation requires exactly this:
 *    "vertices that can't be moved should set 1 consistently for all indices with
 *    the same position". A building leans on the sloppy path far more than a road
 *    chunk does, so this step is load-bearing here rather than defensive.
 * 3. **Re-attribution after a whole-mesh simplify.** Simplifying every range
 *    together returns one index buffer with no materials in it. Each vertex is
 *    tagged with the key of the lowest-packed range that referenced it, and a
 *    triangle takes the key its three vertices agree on most, with a three-way
 *    tie going to the lowest packed key. Both rules are total orders, so the
 *    result does not depend on visitation order.
 *
 * They are re-implemented rather than shared because lod_chunk.cpp keeps them in
 * an anonymous namespace. Lifting the three into a header both files include
 * would be the right follow-up; it is a change to a file this work was asked not
 * to touch.
 *
 * ### What is this file's own
 *
 * **The covering-plane guard**: cover_samples(), sample_is_covered() and
 * open_fraction(), and the retreat from a sloppy result to a constrained one that
 * build_building_lod() makes when they fail. A road chunk has no openings, so
 * lod_chunk.cpp has nothing of the kind and nothing to borrow. It is here because
 * without it this file emitted a building whose front wall was a grid of holes --
 * see building_lod.hpp, "Its detail is a WINDOW", for the measurement and for why
 * the two cheap proxies for it are both wrong.
 *
 * ### What is NOT borrowed
 *
 * lod_chunk.cpp's component analysis -- build_position_remap() feeding a
 * DisjointSet feeding split_by_anchor() -- exists so meshopt_SimplifyPrune can be
 * applied to the components no lock touches and withheld from the ones it does.
 * This file does not prune (see the header), so it needs the position remap and
 * not the components. The remap is still built, because the wedge propagation
 * needs it.
 *
 * ### Determinism
 *
 * Material keys are visited in ascending MaterialKey::packed() order everywhere
 * they are visited; the one std::unordered_map is drained into a sorted vector
 * before it is read; weld_vertices() is order-independent by construction; and
 * meshoptimizer is deterministic. Nothing here varies run to run.
 */

#include "procgen/building_lod.hpp"

#include "osm/road/mesh_optimize.hpp"
#include "procgen/rules/op_facade.hpp"

#include <meshoptimizer.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace stratum::procgen {

// The two files must agree on the distance scale or a scene that mixes roads and
// buildings switches them on two different curves. See kBuildingLodSwitchFactor.
static_assert(kBuildingLodSwitchFactor == osm::road::kLodSwitchFactor,
              "building and road LOD switch distances must share one constant");

namespace {

using osm::road::LodConfig;
using osm::road::WeldConfig;

// ============================================================================
// Constants
// ============================================================================

/// Sentinel for "no vertex", used as a remap hole.
constexpr uint32_t kNoVertex = 0xFFFFFFFFu;

/// Sentinel for "this vertex is in no surviving material range".
constexpr uint32_t kNoKey = 0xFFFFFFFFu;

/// Floor on the bounding radius, so a degenerate building cannot divide by zero.
constexpr float kMinBoundsRadius = 1e-3f;

/**
 * @brief A level must beat the previous one by this fraction to earn its keep
 *
 * lod_chunk.cpp's kMinLevelReduction, same value and same argument: a second
 * resident copy of nearly the same mesh costs memory and an upload and saves
 * nothing. It matters more here, because the constrained simplifier stalls near
 * 82% on rule-generated geometry, and 82% is a level that must not be kept.
 */
constexpr float kMinLevelReduction = 0.10f;

/**
 * @brief Position quantisation for the wedge analysis, metres
 *
 * Matches WeldConfig::position_epsilon. Used only to decide which vertices are
 * the same surface point, never to move geometry, so a position landing on the
 * wrong side of a cell boundary costs one point being treated as two.
 */
constexpr double kConnectivityCell = 1e-4;

// ============================================================================
// Shared helpers
// ============================================================================

/// True when @p sub is a usable triangle-list range of @p mesh
[[nodiscard]] bool is_triangle_range(const Mesh& mesh, const SubMesh& sub) {
    if (sub.index_count == 0u) {
        return false;
    }
    if ((sub.index_offset % 3u) != 0u || (sub.index_count % 3u) != 0u) {
        return false;
    }
    const size_t end = static_cast<size_t>(sub.index_offset) + sub.index_count;
    return end <= mesh.indices.size();
}

/**
 * @brief True when every index of @p mesh addresses a vertex that exists
 *
 * meshoptimizer asserts on an out-of-range index in debug and reads out of bounds
 * in release, so a malformed building is refused rather than simplified.
 */
[[nodiscard]] bool indices_are_in_range(const Mesh& mesh) {
    const uint32_t vertex_count = static_cast<uint32_t>(mesh.vertices.size());
    for (uint32_t index : mesh.indices) {
        if (index >= vertex_count) {
            return false;
        }
    }
    return true;
}

/// First float of the position channel. Vertex::position is the first member.
[[nodiscard]] const float* position_stream(const Mesh& mesh) {
    return &mesh.vertices[0].position.x;
}

/**
 * @brief Compact @p mesh so it carries only the vertices its indices reference
 *
 * Survivors keep ascending original order, so the result does not depend on the
 * order triangles are visited in. A coarse level would otherwise drag level 0's
 * whole vertex buffer around, which is most of what the level was built to save.
 */
void compact_vertices(Mesh& mesh) {
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        return;
    }
    const size_t vertex_count = mesh.vertices.size();
    std::vector<uint32_t> remap(vertex_count, kNoVertex);
    for (uint32_t index : mesh.indices) {
        if (index < vertex_count) {
            remap[index] = 0u;
        }
    }
    std::vector<Vertex> kept;
    kept.reserve(vertex_count);
    for (size_t i = 0; i < vertex_count; ++i) {
        if (remap[i] != kNoVertex) {
            remap[i] = static_cast<uint32_t>(kept.size());
            kept.push_back(mesh.vertices[i]);
        }
    }
    if (kept.size() == vertex_count) {
        return;
    }
    for (uint32_t& index : mesh.indices) {
        if (index < vertex_count && remap[index] != kNoVertex) {
            index = remap[index];
        }
    }
    mesh.vertices = std::move(kept);
}

/// The reordering config used for level 0 and for every simplified level.
[[nodiscard]] LodConfig reorder_config() {
    LodConfig cfg;
    cfg.optimize_cache = true;
    cfg.optimize_overdraw = true;
    cfg.optimize_fetch = true;
    return cfg;
}

/**
 * @brief The weld that makes a rule-generated building simplifiable at all
 *
 * UV, tangent and colour tests relaxed to "ignore"; position, normal and material
 * kept at their shipping values.
 *
 * shape_to_mesh() duplicates vertices per face and projects UVs in each face's
 * own basis, so every shared edge of a rule building is a UV seam and the
 * shipping WeldConfig refuses every weld on it -- measured, 0 of 802 vertices.
 * Relaxing UV and tangent takes the same mesh to 592 vertices, which is what
 * gives the simplifier an interior edge to collapse.
 *
 * Normals stay at 0.01, which is the test that must not be relaxed: it is what
 * stops a wall being welded smooth into a roof. Material stays respected for the
 * same reason lod_chunk.cpp respects it.
 *
 * This weld is applied to the SIMPLIFICATION BASE only, never to level 0. A
 * welded UV seam stretches a texture across the join, and level 0 is the mesh
 * seen from close enough for that to show.
 */
[[nodiscard]] WeldConfig base_weld_config() {
    WeldConfig cfg;
    cfg.uv_epsilon = 1.0e9f;      // ignore: every face carries its own projection
    cfg.tangent_epsilon = 4.0f;   // ignore, including handedness: derived from the UVs
    cfg.color_epsilon = 2.0f;     // ignore: a facade part's tint is its material's job
    return cfg;
}

// ============================================================================
// Material ranges
// ============================================================================

/// Unpack a MaterialKey::packed() back into the pair it was made from.
[[nodiscard]] MaterialKey key_from_packed(uint32_t packed) {
    return MaterialKey{static_cast<MaterialId>(packed >> 16),
                       static_cast<uint16_t>(packed & 0xFFFFu)};
}

/**
 * @brief The MaterialKey of every triangle of @p mesh, gaps included
 *
 * Per TRIANGLE rather than per range, and that is not a stylistic choice. A mesh
 * from GenerationResult::build_mesh() does not tile its own index buffer: a
 * terminal whose faces are all MaterialId::Default opens no SubMesh at all, so
 * its triangles sit in a GAP between two ranges. Measured on the facade fixture,
 * 14 of its 400 triangles are in such gaps -- the roof and the three unglazed
 * walls -- and an implementation that walks effective_submeshes() and copies what
 * it finds deletes the roof off the building without a word.
 *
 * So gaps keep their geometry as the Default slot, which is exactly what
 * Mesh::sort_submeshes_by_material() promises for the same case, and this
 * function is a deliberate restatement of that rule.
 *
 * A range that is not a valid triangle list is skipped, which leaves its
 * triangles reading as Default. That loses a material tag on malformed input and
 * keeps the geometry; the other way round loses the geometry.
 *
 * @param mesh Source
 * @return One packed key per triangle, `indices.size() / 3` long
 */
[[nodiscard]] std::vector<uint32_t> triangle_keys(const Mesh& mesh) {
    const size_t triangle_count = mesh.indices.size() / 3u;
    std::vector<uint32_t> keys(triangle_count, MaterialKey{}.packed());
    for (const SubMesh& sub : mesh.effective_submeshes()) {
        if (!is_triangle_range(mesh, sub)) {
            continue;
        }
        const uint32_t packed = MaterialKey{sub.material, sub.variant}.packed();
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset) + sub.index_count) / 3u;
        for (size_t t = first; t < last && t < triangle_count; ++t) {
            keys[t] = packed;
        }
    }
    return keys;
}

/**
 * @brief Rebuild @p mesh with its triangles grouped into one SubMesh per key
 *
 * GenerationResult::build_mesh() appends one range per terminal, so a 45-terminal
 * building arrives with 61 SubMesh entries naming 5 distinct keys. Every level of
 * the chain has to state its materials once, and a renderer issuing 61 draw calls
 * for 5 materials pays the whole cost of the submesh system for none of its
 * benefit.
 *
 * Keys ascend by MaterialKey::packed() and triangles keep their relative order
 * inside a key, so the layout is a function of the input alone.
 *
 * @param mesh    Source geometry, read only. Its vertices are copied as they are.
 * @param keys    Per-triangle keys from triangle_keys()
 * @param dropped Keys to omit entirely. Their triangles are not emitted.
 * @return The regrouped mesh. Empty when every triangle was dropped.
 */
[[nodiscard]] Mesh group_by_key(const Mesh& mesh, const std::vector<uint32_t>& keys,
                                const std::vector<MaterialKey>& dropped) {
    Mesh out;
    const size_t triangle_count = std::min(keys.size(), mesh.indices.size() / 3u);
    if (triangle_count == 0u) {
        return out;
    }

    std::vector<uint32_t> dropped_packed;
    dropped_packed.reserve(dropped.size());
    for (const MaterialKey& key : dropped) {
        dropped_packed.push_back(key.packed());
    }
    std::sort(dropped_packed.begin(), dropped_packed.end());

    std::vector<uint32_t> present(keys.begin(), keys.begin() + static_cast<long>(triangle_count));
    std::sort(present.begin(), present.end());
    present.erase(std::unique(present.begin(), present.end()), present.end());

    out.vertices = mesh.vertices;
    out.indices.reserve(triangle_count * 3u);

    for (uint32_t packed : present) {
        if (std::binary_search(dropped_packed.begin(), dropped_packed.end(), packed)) {
            continue;
        }
        const uint32_t offset = static_cast<uint32_t>(out.indices.size());
        for (size_t t = 0; t < triangle_count; ++t) {
            if (keys[t] != packed) {
                continue;
            }
            out.indices.push_back(mesh.indices[t * 3u + 0u]);
            out.indices.push_back(mesh.indices[t * 3u + 1u]);
            out.indices.push_back(mesh.indices[t * 3u + 2u]);
        }
        const uint32_t appended = static_cast<uint32_t>(out.indices.size()) - offset;
        if (appended == 0u) {
            continue;
        }
        const MaterialKey key = key_from_packed(packed);
        out.submeshes.push_back(SubMesh{offset, appended, key.material, key.variant});
    }

    if (out.indices.empty()) {
        out.vertices.clear();
        return out;
    }

    out.bounds = BoundingBox3D{};
    for (uint32_t index : out.indices) {
        out.bounds.expand(out.vertices[index].position);
    }
    return out;
}

// ============================================================================
// The ground lock
// ============================================================================

/**
 * @brief Pin every vertex within @p band of the lowest one
 *
 * Written in meshoptimizer's `vertex_lock` encoding: meshopt_SimplifyVertex_Lock
 * for a pinned vertex, 0 for a free one.
 *
 * The predicate is the one thing this file changes about lod_chunk.cpp's lock
 * set. There the question was "is this vertex near the chunk rectangle", because
 * the answer decided whether a neighbouring chunk had its own copy of it. Here
 * there is no neighbour and the question is "is this vertex what the building
 * stands on", because the answer decides whether the building stays on the
 * ground.
 *
 * @param mesh Simplification base
 * @param band Metres above the lowest vertex. Non-positive locks nothing.
 * @param out  Resized to the vertex count and overwritten
 * @return How many vertices were locked
 */
size_t build_ground_lock(const Mesh& mesh, float band, std::vector<unsigned char>& out) {
    out.assign(mesh.vertices.size(), 0u);
    if (!(band > 0.0f) || mesh.vertices.empty()) {
        return 0;
    }

    float lowest = mesh.vertices[0].position.y;
    for (const Vertex& v : mesh.vertices) {
        lowest = std::min(lowest, v.position.y);
    }

    const float ceiling = lowest + band;
    size_t locked = 0;
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        if (mesh.vertices[i].position.y <= ceiling) {
            out[i] = static_cast<unsigned char>(meshopt_SimplifyVertex_Lock);
            ++locked;
        }
    }
    return locked;
}

/// Integer cell coordinate on the wedge grid, in double because a map coordinate
/// can be tens of kilometres from the origin and a float at 1e-4 stops resolving.
[[nodiscard]] inline int64_t quantise(float value) {
    return static_cast<int64_t>(std::floor(static_cast<double>(value) / kConnectivityCell));
}

/// FNV-1a over the three cell coordinates.
[[nodiscard]] inline uint64_t hash_cell(int64_t x, int64_t y, int64_t z) {
    uint64_t h = 1469598103934665603ull;
    const auto mix = [&h](uint64_t v) {
        h ^= v;
        h *= 1099511628211ull;
        h ^= (h >> 29);
    };
    mix(static_cast<uint64_t>(x));
    mix(static_cast<uint64_t>(y));
    mix(static_cast<uint64_t>(z));
    return h;
}

/**
 * @brief Map every vertex to the lowest-indexed vertex sharing its position
 *
 * Two vertices at one position are two wedges of one surface point: the weld
 * keeps a wall corner and a roof corner apart because their normals differ, and
 * both are still the same point as far as the ground is concerned. The
 * representative is the LOWEST index of the group and groups are visited in
 * ascending order, so the result does not depend on hash bucket order.
 */
[[nodiscard]] std::vector<uint32_t> build_position_remap(const Mesh& mesh) {
    const size_t vertex_count = mesh.vertices.size();
    std::vector<uint32_t> remap(vertex_count);
    std::unordered_map<uint64_t, uint32_t> first_at_cell;
    first_at_cell.reserve(vertex_count * 2u);

    for (uint32_t i = 0; i < static_cast<uint32_t>(vertex_count); ++i) {
        const glm::vec3& p = mesh.vertices[i].position;
        const uint64_t key = hash_cell(quantise(p.x), quantise(p.y), quantise(p.z));
        const auto inserted = first_at_cell.emplace(key, i);
        remap[i] = inserted.first->second;
    }
    return remap;
}

/**
 * @brief Grow the lock set by one triangle ring, then to every wedge of a locked point
 *
 * Both steps are lod_chunk.cpp's and both are needed here for the reasons this
 * file's header note gives: a lock stops a vertex moving and not disappearing,
 * and meshopt_simplifySloppy requires a locked position to be locked at every one
 * of its indices.
 *
 * One ring is enough and one ring is cheap. It grows the tower fixture's lock
 * from 38 vertices to about a hundred out of 424.
 *
 * @note MEASURED INERT on both fixtures, and kept anyway. Removing this call
 *       entirely leaves the whole BuildingLod suite passing, including the
 *       base-ring test, because build_ground_lock() is already positional -- it
 *       locks by height, so every wedge of a base position is in the seed set and
 *       the ring only adds the row above it. The two reasons to keep it are that
 *       the promise it defends is one meshoptimizer does not make (a lock stops a
 *       vertex moving, not a triangle being flattened out from under it, which is
 *       what lod_chunk.cpp measured at 130 of 149 rim vertices) and that
 *       meshopt_simplifySloppy's own contract REQUIRES the wedge half of it. A
 *       ground contact that holds because a simplifier happened not to take a
 *       legal collapse is not a guarantee. Recorded here rather than left as an
 *       unexplained pass, so that a later reader knows the suite does not cover
 *       it and does not conclude from a green run that it is load-bearing.
 *
 * @param mesh      Simplification base
 * @param pos_remap Position-canonical vertex ids
 * @param lock      Grown in place
 * @return How many vertices are locked afterwards
 */
size_t dilate_lock_set(const Mesh& mesh, const std::vector<uint32_t>& pos_remap,
                       std::vector<unsigned char>& lock) {
    const size_t vertex_count = mesh.vertices.size();
    const size_t triangle_end = mesh.indices.size() - (mesh.indices.size() % 3u);

    // Read from a copy so the growth stops at one ring rather than flooding along
    // the surface.
    const std::vector<unsigned char> seed = lock;
    for (size_t t = 0; t < triangle_end; t += 3u) {
        const uint32_t a = mesh.indices[t];
        const uint32_t b = mesh.indices[t + 1u];
        const uint32_t c = mesh.indices[t + 2u];
        if (a >= vertex_count || b >= vertex_count || c >= vertex_count) {
            continue;
        }
        if (seed[a] == 0u && seed[b] == 0u && seed[c] == 0u) {
            continue;
        }
        lock[a] = static_cast<unsigned char>(meshopt_SimplifyVertex_Lock);
        lock[b] = static_cast<unsigned char>(meshopt_SimplifyVertex_Lock);
        lock[c] = static_cast<unsigned char>(meshopt_SimplifyVertex_Lock);
    }

    std::vector<unsigned char> locked_position(vertex_count, 0u);
    for (size_t i = 0; i < vertex_count; ++i) {
        if (lock[i] != 0u) {
            locked_position[pos_remap[i]] = 1u;
        }
    }
    size_t locked = 0;
    for (size_t i = 0; i < vertex_count; ++i) {
        if (locked_position[pos_remap[i]] != 0u) {
            lock[i] = static_cast<unsigned char>(meshopt_SimplifyVertex_Lock);
            ++locked;
        }
    }
    return locked;
}

// ============================================================================
// The covering-plane guard
// ============================================================================

/**
 * @brief One "can you still see through here" question, as a ray
 *
 * @p origin sits just BEHIND a point that level 0's covering plane covered and
 * @p direction points out of the building, so the ray asks: is there still a
 * surface at, or in front of, this point.
 */
struct CoverSample {
    glm::dvec3 origin{};
    glm::dvec3 direction{};
};

/// How far behind its own surface a sample starts, as a fraction of the bounding radius
constexpr double kCoverBackOff = 1.0e-3;

/**
 * @brief How far a corner sample is pulled towards its triangle's centroid
 *
 * @note MEASURED INERT, and kept anyway, on the same terms as dilate_lock_set()
 *       below. Replacing all four sample points of a triangle with its centroid
 *       four times over leaves the whole BuildingLod suite passing, because every
 *       failure either fixture produces opens a whole pane rather than half of
 *       one. The corners are there for the half-pane case, which is the one a
 *       centroid grid is blind to, and four rays per triangle against a
 *       hundred-triangle level is not a cost worth trading it for. Recorded here
 *       so a later reader knows the suite does not cover it.
 */
constexpr double kCornerPullIn = 0.1;

/**
 * @brief Fraction of level 0's covered points a level may leave open before refusal
 *
 * Not zero, because a sample sits a hair behind a surface the simplifier is
 * allowed to move, and a surface that moved further than the back-off distance
 * takes its own sample with it without a hole existing anywhere.
 *
 * Not loose either, because the failure this exists to catch is not subtle.
 * Measured on the facade fixture with the guard switched off, the level that
 * loses the frame leaves a third of the covered points open and the one that
 * loses the glazing as well leaves all of them, while an intact level measures
 * zero. There is no middle ground to tune against, so this is chosen to be
 * obviously below the first failure rather than fitted to anything.
 */
constexpr double kCoverLossTolerance = 0.02;

/**
 * @brief Where level 0's covering planes were, and which way is out
 *
 * Built once, from level 0, and asked of every candidate level. FOUR points per
 * covering triangle rather than one, because a window pane is two triangles and
 * one centroid each would not notice half a pane opening.
 *
 * The outward direction is the triangle's VERTEX normal, averaged. op_facade.cpp
 * sets it deliberately -- it is what lights the facade -- and it is the one part
 * of a vertex that says which side of a single-sided plane a viewer is on. A
 * triangle whose normals cancel falls back to its winding.
 *
 * @param level0   The reference level
 * @param covering Packed covering keys, ascending
 * @param radius   Bounding radius, which sets the back-off distance
 * @return One sample per point, in index order, so the result is a function of
 *         the input alone
 */
[[nodiscard]] std::vector<CoverSample> cover_samples(const Mesh& level0,
                                                     const std::vector<uint32_t>& covering,
                                                     double radius) {
    std::vector<CoverSample> samples;
    if (covering.empty()) {
        return samples;
    }
    const double back_off = std::max(1.0e-4, kCoverBackOff * radius);

    for (const SubMesh& sub : level0.effective_submeshes()) {
        if (!is_triangle_range(level0, sub)) {
            continue;
        }
        const uint32_t packed = MaterialKey{sub.material, sub.variant}.packed();
        if (!std::binary_search(covering.begin(), covering.end(), packed)) {
            continue;
        }
        const size_t end = static_cast<size_t>(sub.index_offset) + sub.index_count;
        for (size_t i = sub.index_offset; i + 2u < end; i += 3u) {
            const uint32_t ia = level0.indices[i];
            const uint32_t ib = level0.indices[i + 1u];
            const uint32_t ic = level0.indices[i + 2u];
            if (ia >= level0.vertices.size() || ib >= level0.vertices.size() ||
                ic >= level0.vertices.size()) {
                continue;
            }
            const glm::dvec3 a{level0.vertices[ia].position};
            const glm::dvec3 b{level0.vertices[ib].position};
            const glm::dvec3 c{level0.vertices[ic].position};

            glm::dvec3 out = glm::dvec3{level0.vertices[ia].normal} +
                             glm::dvec3{level0.vertices[ib].normal} +
                             glm::dvec3{level0.vertices[ic].normal};
            if (glm::length(out) < 1.0e-6) {
                out = glm::cross(b - a, c - a);
            }
            if (glm::length(out) < 1.0e-12) {
                continue;
            }
            out = glm::normalize(out);

            const glm::dvec3 centroid = (a + b + c) / 3.0;
            const glm::dvec3 points[4] = {centroid, a + (centroid - a) * kCornerPullIn,
                                          b + (centroid - b) * kCornerPullIn,
                                          c + (centroid - c) * kCornerPullIn};
            for (const glm::dvec3& point : points) {
                samples.push_back(CoverSample{point - out * back_off, out});
            }
        }
    }
    return samples;
}

/**
 * @brief True when some triangle of @p mesh is at or in front of @p sample
 *
 * Moller-Trumbore, two-sided and unbounded. Two-sided on purpose: the surface
 * that ends up covering the point may be the covering plane itself, the wall
 * welded shut over it, or a shell the sloppy clusterer built out of both, and
 * only the first of those has a winding this file can predict.
 */
[[nodiscard]] bool sample_is_covered(const Mesh& mesh, const CoverSample& sample) {
    for (size_t t = 0; t + 2u < mesh.indices.size(); t += 3u) {
        const uint32_t ia = mesh.indices[t];
        const uint32_t ib = mesh.indices[t + 1u];
        const uint32_t ic = mesh.indices[t + 2u];
        if (ia >= mesh.vertices.size() || ib >= mesh.vertices.size() ||
            ic >= mesh.vertices.size()) {
            continue;
        }
        const glm::dvec3 a{mesh.vertices[ia].position};
        const glm::dvec3 edge1 = glm::dvec3{mesh.vertices[ib].position} - a;
        const glm::dvec3 edge2 = glm::dvec3{mesh.vertices[ic].position} - a;
        const glm::dvec3 h = glm::cross(sample.direction, edge2);
        const double det = glm::dot(edge1, h);
        if (std::fabs(det) < 1.0e-12) {
            continue;
        }
        const double inv = 1.0 / det;
        const glm::dvec3 s = sample.origin - a;
        const double u = glm::dot(s, h) * inv;
        if (u < -1.0e-9 || u > 1.0 + 1.0e-9) {
            continue;
        }
        const glm::dvec3 q = glm::cross(s, edge1);
        const double v = glm::dot(sample.direction, q) * inv;
        if (v < -1.0e-9 || u + v > 1.0 + 1.0e-9) {
            continue;
        }
        if (glm::dot(edge2, q) * inv >= 0.0) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Fraction of level 0's covered points that @p level leaves open
 *
 * THE measurement this file's central promise rests on, and it asks the
 * visibility question as a visibility question.
 *
 * An earlier attempt at this compared the AREA of each covering key against
 * level 0's, and it was wrong in both directions. The constrained pass collapses
 * the glazing into the frame: the glazing key's area goes to zero, the frame's
 * goes from 3.000 to 14.400 m2, and the opening is covered to the square
 * centimetre. The sloppy pass welds the wall shut OVER an opening: the covering
 * keys between them keep 2.268 of 14.400 m2 and there is still nothing to see
 * through. Neither is a hole, and an area test refuses both. A ray does not.
 *
 * @param level   The candidate level
 * @param samples From cover_samples(), built against level 0
 * @return Fraction in [0, 1]. Zero when @p samples is empty, so a building with
 *         no covering planes is never refused by this.
 */
[[nodiscard]] double open_fraction(const Mesh& level, const std::vector<CoverSample>& samples) {
    if (samples.empty()) {
        return 0.0;
    }
    size_t open = 0;
    for (const CoverSample& sample : samples) {
        if (!sample_is_covered(level, sample)) {
            ++open;
        }
    }
    return static_cast<double>(open) / static_cast<double>(samples.size());
}

// ============================================================================
// Simplification
// ============================================================================

/// Rounded-down multiple of three, never below three unless the input is empty.
[[nodiscard]] size_t triangle_target(size_t count, float ratio) {
    if (count == 0u) {
        return 0u;
    }
    size_t target = static_cast<size_t>(std::llround(static_cast<double>(count) * ratio));
    target -= target % 3u;
    return target < 3u ? 3u : target;
}

/**
 * @brief The topology-preserving pass, with the ground lock applied
 *
 * meshopt_simplifyWithAttributes() with an attribute count of zero is the same
 * algorithm as meshopt_simplify(); it is the only entry point that takes a
 * `vertex_lock` array. meshopt_SimplifyPrune is deliberately not passed -- see
 * the header note on what was left out.
 */
void simplify_strict(const Mesh& base, const std::vector<uint32_t>& source,
                     const std::vector<unsigned char>& lock, size_t target, float target_error,
                     std::vector<uint32_t>& out) {
    if (source.size() < 3u) {
        out.clear();
        return;
    }
    // meshopt's worst case output is the INPUT index count, not the target.
    out.resize(source.size());
    float result_error = 0.0f;
    const size_t result = meshopt_simplifyWithAttributes(
        out.data(), source.data(), source.size(), position_stream(base), base.vertices.size(),
        sizeof(Vertex), nullptr, 0u, nullptr, 0u, lock.data(), target, target_error, 0u,
        &result_error);
    out.resize(result);
}

/**
 * @brief The topology-free retry, with the ground lock still applied
 *
 * Only ever called when simplify_strict() missed its target and
 * BuildingLodConfig::allow_sloppy is set, and its result is only ever kept when
 * it is strictly smaller. Those are two of LodConfig::allow_sloppy's three
 * guards; passing @p lock is the third.
 *
 * @note The error bound handed to this is the config's own, not a free hand. The
 *       point of the retry is to get past a TOPOLOGY constraint the constrained
 *       pass could not, and letting it past the error bound as well would trade
 *       the silhouette for triangles nobody asked it to trade.
 */
void simplify_sloppy(const Mesh& base, const std::vector<uint32_t>& source,
                     const std::vector<unsigned char>& lock, size_t target, float target_error,
                     std::vector<uint32_t>& out) {
    if (source.size() < 3u) {
        out.clear();
        return;
    }
    out.resize(source.size());
    float result_error = 0.0f;
    const size_t result = meshopt_simplifySloppy(
        out.data(), source.data(), source.size(), position_stream(base), base.vertices.size(),
        sizeof(Vertex), lock.data(), target, target_error, &result_error);
    out.resize(result);
}

/// The three vertex indices of a triangle, ascending, as an identity for lookup.
[[nodiscard]] std::array<uint32_t, 3> canonical_triangle(uint32_t a, uint32_t b, uint32_t c) {
    std::array<uint32_t, 3> t{a, b, c};
    std::sort(t.begin(), t.end());
    return t;
}

/**
 * @brief Give a simplified index buffer its materials back
 *
 * A whole-mesh simplify returns one index buffer and no ranges, so each surviving
 * triangle has to be told which material it is. Two rules, in this order:
 *
 * 1. **A triangle that survived unchanged keeps the key it had.** The simplifier
 *    collapses edges, so most of what comes out is triangles that went in, with
 *    the same three vertex indices. Looking them up is exact and costs one map.
 * 2. **Anything else takes a per-vertex vote**: each vertex carries the key of
 *    the lowest-packed range that referenced it, a triangle takes the key its
 *    three vertices agree on most, and a three-way disagreement goes to the
 *    lowest packed key. Both are total orders, so the result does not depend on
 *    visitation order.
 *
 * lod_chunk.cpp has rule 2 alone, and rule 2 alone is WRONG on a facade. The vote
 * is exact when two materials meet along a LINE -- a kerb against a gutter, where
 * only the shared column is ambiguous -- and it fails when one material is
 * entirely ENCLOSED by another. A window's glazing sits inside its frame, so
 * every one of the glazing quad's four corners is also a frame vertex; the vote
 * hands all of them to the frame, the glazing range comes back empty, and the
 * window renders as whatever the frame is made of. Measured on the facade
 * fixture, all 24 glazing triangles were attributed to the frame before rule 1
 * existed, on a level where the simplifier had not touched a single triangle.
 *
 * @param source  The geometry that went into the simplifier: supplies the
 *                vertices, the material ranges and the identities for rule 1
 * @param keys    Per-triangle keys of @p source, from triangle_keys()
 * @param indices What came out of the simplifier
 * @return A mesh carrying @p source's vertices, @p indices grouped by key in
 *         ascending packed order, and one SubMesh per key. Vertices compacted.
 */
[[nodiscard]] Mesh reattribute(const Mesh& source, const std::vector<uint32_t>& keys,
                               const std::vector<uint32_t>& indices) {
    Mesh level;
    if (indices.size() < 3u) {
        return level;
    }

    // Rule 1's table: the identity of every input triangle and the key it carried.
    // std::map rather than a hash: the identity is three 32-bit words, the tables
    // are hundreds of entries, and an ordered container cannot make the output
    // depend on a hash seed.
    std::map<std::array<uint32_t, 3>, uint32_t> key_of_triangle;
    const size_t source_triangles = std::min(keys.size(), source.indices.size() / 3u);
    for (size_t t = 0; t < source_triangles; ++t) {
        key_of_triangle.emplace(canonical_triangle(source.indices[t * 3u + 0u],
                                                   source.indices[t * 3u + 1u],
                                                   source.indices[t * 3u + 2u]),
                                keys[t]);
    }

    // Rule 2's table: the lowest-packed key referencing each vertex.
    std::vector<uint32_t> vertex_key(source.vertices.size(), kNoKey);
    for (size_t t = 0; t < source_triangles; ++t) {
        for (size_t corner = 0; corner < 3u; ++corner) {
            const uint32_t index = source.indices[t * 3u + corner];
            if (index < vertex_key.size() && keys[t] < vertex_key[index]) {
                vertex_key[index] = keys[t];
            }
        }
    }

    std::unordered_map<uint32_t, std::vector<uint32_t>> buckets;
    for (size_t t = 0; t + 2u < indices.size(); t += 3u) {
        const uint32_t a = indices[t];
        const uint32_t b = indices[t + 1u];
        const uint32_t c = indices[t + 2u];
        if (a >= vertex_key.size() || b >= vertex_key.size() || c >= vertex_key.size()) {
            continue;
        }

        uint32_t chosen = kNoKey;
        const auto exact = key_of_triangle.find(canonical_triangle(a, b, c));
        if (exact != key_of_triangle.end()) {
            chosen = exact->second;
        } else {
            const uint32_t ka = vertex_key[a];
            const uint32_t kb = vertex_key[b];
            const uint32_t kc = vertex_key[c];
            chosen = std::min({ka, kb, kc});
            if (kb == kc && kb != kNoKey) {
                chosen = kb;
            }
            if (ka == kc && ka != kNoKey) {
                chosen = ka;
            }
            if (ka == kb && ka != kNoKey) {
                chosen = ka;
            }
        }
        if (chosen == kNoKey) {
            continue;
        }
        std::vector<uint32_t>& bucket = buckets[chosen];
        bucket.push_back(a);
        bucket.push_back(b);
        bucket.push_back(c);
    }
    if (buckets.empty()) {
        return level;
    }

    // Drain the map into a sorted list before reading it: an unordered_map's
    // iteration order is not part of the program's meaning and this output is
    // cached and compared.
    std::vector<uint32_t> ordered;
    ordered.reserve(buckets.size());
    for (const auto& entry : buckets) {
        ordered.push_back(entry.first);
    }
    std::sort(ordered.begin(), ordered.end());

    level.vertices = source.vertices;
    for (uint32_t packed : ordered) {
        const std::vector<uint32_t>& bucket = buckets[packed];
        const uint32_t offset = static_cast<uint32_t>(level.indices.size());
        level.indices.insert(level.indices.end(), bucket.begin(), bucket.end());
        const MaterialKey key = key_from_packed(packed);
        level.submeshes.push_back(
            SubMesh{offset, static_cast<uint32_t>(bucket.size()), key.material, key.variant});
    }

    compact_vertices(level);
    return level;
}

/// The keys due to be gone by the time @p ratio_index is built
[[nodiscard]] std::vector<MaterialKey> keys_dropped_at(const std::vector<DetailDrop>& drops,
                                                       size_t ratio_index) {
    std::vector<MaterialKey> keys;
    for (const DetailDrop& drop : drops) {
        if (ratio_index >= drop.from_ratio) {
            keys.push_back(drop.key);
        }
    }
    return keys;
}

} // namespace

// ----------------------------------------------------------------------------

std::vector<DetailDrop> default_facade_detail_drops() {
    using rules::FacadePart;
    using rules::facade_material;

    // The RETURNS, and nothing else. They read only in raking light, they are
    // most of the triangles, and dropping them leaves the opening covered by a
    // plane sitting `reveal` metres behind the wall face.
    //
    // The frame was here, at from_ratio 1, and it was a hole. The comment that
    // shipped with it said in its own words that "dropping the frame leaves a
    // slot one frame-width wide around the glass" and then dropped it anyway:
    // measured on the facade fixture, level 2 of the default chain had 96 of
    // 2400 sample rays coming through the front wall, one slot per pane. Frame,
    // Panel, Glass and Leaf are the covering plane -- see
    // default_covering_plane_keys() -- and a covering plane is never dropped.
    return {
        DetailDrop{facade_material(FacadePart::Reveal), 0},
        DetailDrop{facade_material(FacadePart::Sill), 0},
        DetailDrop{facade_material(FacadePart::Threshold), 0},
    };
}

std::vector<MaterialKey> default_covering_plane_keys() {
    using rules::FacadePart;
    using rules::facade_material;

    // Frame and Glass tile a window's opening exactly between them; Leaf is a
    // door's. Panel is the wall itself, and it is deliberately NOT here: the wall
    // is the surface the chain exists to decimate, and a wall that closes its own
    // opening as it coarsens has lost nothing a player can see from 100 m.
    return {
        facade_material(FacadePart::Frame),
        facade_material(FacadePart::Glass),
        facade_material(FacadePart::Leaf),
    };
}

Mesh drop_detail_keys(const Mesh& mesh, const std::vector<MaterialKey>& keys) {
    if (mesh.indices.size() < 3u || mesh.vertices.empty()) {
        return Mesh{};
    }
    // The same refusal build_building_lod() makes, and for a sharper reason: this
    // is a public entry point a caller reaches for on its own, and group_by_key()
    // dereferences out.vertices[index] while computing bounds. Without this an
    // index that addresses no vertex is an out-of-bounds READ rather than a
    // refusal -- caught by a caller passing indices {0, 1, 9999} over a three
    // vertex mesh.
    if (!indices_are_in_range(mesh)) {
        spdlog::warn("drop_detail_keys: refusing a mesh with out-of-range indices");
        return Mesh{};
    }
    return group_by_key(mesh, triangle_keys(mesh), keys);
}

BuildingLod build_building_lod(const Mesh& building, const BuildingLodConfig& cfg) {
    BuildingLod lod;

    if (building.vertices.empty() || building.indices.size() < 3u) {
        return lod;
    }
    if (!indices_are_in_range(building)) {
        spdlog::warn("build_building_lod: refusing a mesh with out-of-range indices");
        return lod;
    }

    // ------------------------------------------------------------------
    // 1. Level 0
    //
    // The input with its ranges coalesced and its triangles reordered. Never
    // simplified and never welded: this is the mesh a player stands in front of,
    // and both of those would cost it something it cannot get back.
    // ------------------------------------------------------------------
    Mesh level0 = group_by_key(building, triangle_keys(building), {});
    if (level0.indices.size() < 3u) {
        return lod;
    }
    osm::road::optimize_mesh(level0, reorder_config());
    level0.compute_bounds();

    const size_t base_indices = level0.indices.size();
    const float radius = std::max(level0.bounds.radius(), kMinBoundsRadius);

    lod.levels.push_back(level0);
    lod.switch_distances.push_back(0.0f);

    // ------------------------------------------------------------------
    // 2. The simplification base
    //
    // A separate, UV-free weld of level 0. See base_weld_config(): without it a
    // rule-generated building has no interior edge at all and every level comes
    // back identical to level 0.
    // ------------------------------------------------------------------
    Mesh base = level0;
    const size_t base_vertices_before = base.vertices.size();
    osm::road::weld_vertices(base, base_weld_config());

    // weld_vertices() leaves every SubMesh range byte-for-byte as it found it, and
    // level 0's ranges tile the index buffer exactly, so the base's per-triangle
    // keys are level 0's and are computed once.
    const std::vector<uint32_t> base_keys = triangle_keys(base);

    std::vector<unsigned char> lock;
    const size_t seeded = build_ground_lock(base, cfg.ground_band, lock);
    const std::vector<uint32_t> pos_remap = build_position_remap(base);
    const size_t locked = seeded > 0u ? dilate_lock_set(base, pos_remap, lock) : 0u;

    std::vector<uint32_t> covering_packed;
    covering_packed.reserve(cfg.covering_keys.size());
    for (const MaterialKey& key : cfg.covering_keys) {
        covering_packed.push_back(key.packed());
    }
    std::sort(covering_packed.begin(), covering_packed.end());
    covering_packed.erase(std::unique(covering_packed.begin(), covering_packed.end()),
                          covering_packed.end());
    const std::vector<CoverSample> cover =
        cover_samples(lod.levels[0], covering_packed, static_cast<double>(radius));

    // A drop that names a covering plane is a hole the caller asked for, and it
    // is the exact shape of the defect that shipped here -- the default table
    // named the frame, which is half of a window's cover. It is not silently
    // honoured and it is not silently refused: the guard below will refuse every
    // level at and past that ratio, and this is where a reader is told why.
    for (const DetailDrop& drop : cfg.detail_drops) {
        if (std::binary_search(covering_packed.begin(), covering_packed.end(),
                               drop.key.packed())) {
            spdlog::warn("build_building_lod: detail drop names covering material {}:{}, so every "
                         "level from ratio {} will be refused -- clear covering_keys as well to "
                         "build an impostor deliberately",
                         material_id_name(drop.key.material), drop.key.variant, drop.from_ratio);
        }
    }

    const float target_error = std::max(cfg.target_error, 0.0f);

    // ------------------------------------------------------------------
    // 3. The levels
    // ------------------------------------------------------------------
    std::vector<uint32_t> strict_indices;
    std::vector<uint32_t> sloppy_indices;
    size_t previous_indices = base_indices;

    for (size_t i = 0; i < cfg.ratios.size(); ++i) {
        const float ratio = cfg.ratios[i];
        if (!(ratio > 0.0f) || !(ratio < 1.0f)) {
            spdlog::warn("build_building_lod: ignoring ratio {} -- ratios must be in (0, 1)", ratio);
            continue;
        }

        // The detail drop comes BEFORE the simplifier, not after it. A window is
        // 28 triangles of which 26 are returns, and asking a simplifier to shed
        // them is asking it to decide which half of an opening to keep.
        const std::vector<MaterialKey> dropped = keys_dropped_at(cfg.detail_drops, i);
        const Mesh source = dropped.empty() ? base : group_by_key(base, base_keys, dropped);
        if (source.indices.size() < 3u) {
            continue;
        }
        const std::vector<uint32_t> source_keys =
            dropped.empty() ? base_keys : triangle_keys(source);

        // Measured against LEVEL 0, so a drop that met the target on its own is
        // allowed to stop there rather than decimating what is left as well.
        const size_t target = triangle_target(base_indices, ratio);

        const std::vector<uint32_t>* chosen = nullptr;
        bool used_sloppy = false;

        if (source.indices.size() <= target) {
            chosen = &source.indices;
        } else {
            simplify_strict(base, source.indices, lock, target, target_error, strict_indices);
            chosen = &strict_indices;

            if (cfg.allow_sloppy && strict_indices.size() > target) {
                simplify_sloppy(base, source.indices, lock, target, target_error, sloppy_indices);
                if (sloppy_indices.size() >= 3u && sloppy_indices.size() < strict_indices.size()) {
                    chosen = &sloppy_indices;
                    used_sloppy = true;
                }
            }
        }

        if (chosen == nullptr || chosen->size() < 3u) {
            continue;
        }

        Mesh level = reattribute(source, source_keys, *chosen);
        if (level.indices.size() < 3u) {
            continue;
        }

        // ------------------------------------------------------------------
        // The covering-plane guard, and the retreat it exists to make
        //
        // meshopt_simplifySloppy is free to weld unrelated shells together, and
        // on a facade that freedom is what lets it delete an opening's covering
        // plane while leaving the opening in the wall. It did: the chain that
        // shipped here lost the glazing whole at its coarsest level and 576 of
        // 2400 rays cast at the front wall came out the other side.
        //
        // So a sloppy level is CHECKED, and a sloppy level that opened an opening
        // retreats to the constrained result rather than being emitted. The
        // constrained result is bigger, and it cannot have this fault: it
        // preserves topology, this file does not pass meshopt_SimplifyPrune, and
        // a pass that can neither delete a shell nor tear one cannot open a hole
        // that was covered. Measured, it stalls at 14.400 of 14.400 m2 of cover
        // on every configuration tried, including ones where the sloppy result
        // was at 0.000.
        //
        // If the constrained result fails too, the level is not emitted at all. A
        // shorter chain is the honest answer and a coarse level with holes in it
        // is not.
        // ------------------------------------------------------------------
        double open = open_fraction(level, cover);
        if (open > kCoverLossTolerance) {
            if (!used_sloppy) {
                spdlog::warn("build_building_lod: refusing the level for ratio {} -- it leaves "
                             "{:.1f}% of the openings level 0 covered open",
                             ratio, open * 100.0);
                continue;
            }
            spdlog::debug("build_building_lod: ratio {} -- the sloppy result left {:.1f}% of the "
                          "openings open, retreating to the constrained result",
                          ratio, open * 100.0);
            level = reattribute(source, source_keys, strict_indices);
            used_sloppy = false;
            if (level.indices.size() < 3u) {
                continue;
            }
            open = open_fraction(level, cover);
            if (open > kCoverLossTolerance) {
                spdlog::warn("build_building_lod: refusing the level for ratio {} -- it leaves "
                             "{:.1f}% of the openings level 0 covered open",
                             ratio, open * 100.0);
                continue;
            }
        }

        // A level that is nearly the previous one costs a second resident copy of
        // the same geometry and saves nothing, so it is dropped along with its
        // switch distance. A later, more aggressive ratio still gets its turn.
        const double reduction =
            1.0 - static_cast<double>(level.indices.size()) / static_cast<double>(previous_indices);
        if (reduction < static_cast<double>(kMinLevelReduction)) {
            continue;
        }

        osm::road::optimize_mesh(level, reorder_config());
        level.compute_bounds();

        // Achieved, not requested: a level that stalled must not be switched to at
        // the distance the ratio it missed would have earned.
        const float achieved = static_cast<float>(static_cast<double>(level.indices.size()) /
                                                  static_cast<double>(base_indices));
        const float distance =
            radius * kBuildingLodSwitchFactor / std::sqrt(std::max(achieved, 1e-6f));

        previous_indices = level.indices.size();
        lod.levels.push_back(std::move(level));
        lod.switch_distances.push_back(distance);
        if (used_sloppy) {
            ++lod.sloppy_levels;
        }
    }

    spdlog::debug("build_building_lod: {} tris, {} verts welded to {}, {} locked; "
                  "{} levels, {} sloppy",
                  base_indices / 3u, base_vertices_before, base.vertices.size(), locked,
                  lod.levels.size(), lod.sloppy_levels);

    return lod;
}

} // namespace stratum::procgen
