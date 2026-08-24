/**
 * @file lod_chunk.cpp
 * @brief Implementation of merged-per-chunk LOD construction
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * See lod_chunk.hpp for the contract and for why the per-piece chain in
 * mesh_optimize.cpp cannot reduce a road ribbon. This file records what the
 * measurements actually showed, because two of the three causes named in the
 * header turned out to matter and the third did not matter the way it reads.
 *
 * ### The three things that were pinning the per-piece chain
 *
 * 1. **`meshopt_SimplifyLockBorder`.** This is the dominant one by a wide margin.
 *    The flag pins every vertex on an OPEN boundary, and a corridor strip is two
 *    vertex columns wide, so both columns are open boundary and the strip has no
 *    unpinned vertex at all. Replacing the flag with an explicit `vertex_lock`
 *    array -- pinned only within `border_band` of the chunk rectangle -- is what
 *    unlocks the reduction. Without the flag meshoptimizer still classifies those
 *    columns as border vertices and still applies its border-preserving quadric,
 *    so a border vertex may collapse ALONG the border loop but not away from it.
 *    That is exactly the freedom a ribbon needs: it decimates along its length and
 *    keeps its two edges where they were.
 *
 * 2. **The seams between pieces.** Two pieces that met at a junction each carried
 *    their own copy of the shared column, so the ribbon was topologically cut
 *    every few tens of metres and every cut was another border loop terminus. The
 *    merge plus `weld_vertices()` fuses those, which is why the merge has to come
 *    before the simplify rather than after it.
 *
 * 3. **Per-material splitting.** Real, and turned OFF here, which reverses what
 *    the header first said. Measured, the kerb it was there to protect is
 *    protected without it -- by the weld's material seams and by the quadric cost
 *    of flattening a 150 mm vertical face -- while the fragmentation it causes
 *    costs the coarsest level 36.1% of level 0 against 14.9%. See
 *    ChunkLodConfig::simplify_per_material for both measurements.
 *
 * ### Two things `vertex_lock` does not do on its own
 *
 * The lock array pins a vertex against MOVING. It does not pin it against being
 * dropped, and the crack-free guarantee needs the second promise as well:
 *
 * - A collapse of the one unlocked corner of a triangle flattens that triangle,
 *   and meshoptimizer drops flattened triangles. dilate_lock_set() closes this.
 * - meshopt_SimplifyPrune deletes whole components without consulting the lock
 *   array at all. split_by_anchor() closes this.
 *
 * Both are documented at their definitions. Measured on the extract's densest
 * chunk, the two of them take the count of pinned vertices missing from the
 * coarsest level from 130 of 149 down to 0.
 *
 * ### Determinism
 *
 * The merge visits pieces in the order given and material keys in ascending
 * `MaterialKey::packed()` order, which is a total order over the keys present.
 * `weld_vertices()` is order-independent by construction (see mesh_optimize.cpp)
 * and meshoptimizer is deterministic, so nothing here varies run to run or with
 * thread count.
 */

#include "osm/road/lod_chunk.hpp"

#include "osm/road/mesh_optimize.hpp"

#include <meshoptimizer.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Constants
// ============================================================================

/// Sentinel for "no vertex", used as a remap hole.
constexpr uint32_t kNoVertex = 0xFFFFFFFFu;

/// Floor on the bounding radius, so a degenerate chunk cannot divide by zero.
constexpr float kMinBoundsRadius = 1e-3f;

/// A level must beat the previous one by this fraction to earn its resident copy.
constexpr float kMinLevelReduction = 0.10f;

/**
 * @brief Position quantisation used to decide which vertices are the same surface
 *
 * Matches WeldConfig::position_epsilon. Only used for the connectivity analysis
 * that decides which components may be pruned, never to move geometry, so a
 * position landing on the wrong side of a cell boundary costs one component being
 * treated as two and nothing else.
 */
constexpr double kConnectivityCell = 1e-4;

/**
 * @brief meshopt options common to every simplify call this file makes
 *
 * `meshopt_SimplifyLockBorder` is deliberately NOT here, and that absence is the
 * whole point of the file. It is the flag that pins every open-boundary vertex,
 * which on a road ribbon is all of them; the `vertex_lock` array takes its place
 * and asks the positional question that was meant.
 *
 * `meshopt_SimplifyPrune` is not here either, because it is not safe to apply to
 * every component -- see split_by_anchor(). It is added, per call, only to the
 * half of the geometry that no lock touches.
 */
constexpr unsigned int kSimplifyOptions = 0u;

// ============================================================================
// Shared helpers
// ============================================================================

/**
 * @brief True when @p sub is a usable triangle-list range of @p mesh
 *
 * Same test as mesh_optimize.cpp's: a range straddling a triangle is geometry
 * this file cannot reason about, and is carried through untouched rather than
 * reordered or simplified.
 */
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
 * in release, so a malformed piece is skipped rather than merged.
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
 * order triangles are visited in. A simplified level would otherwise drag the
 * full-detail vertex buffer around, which defeats the point of building it.
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

// ============================================================================
// Merging
// ============================================================================

/// One (piece, submesh) contribution, resolved to a merged-space index range.
struct KeyedRange {
    uint32_t packed = 0;         ///< MaterialKey::packed(), the sort key
    MaterialKey key{};           ///< The key itself
    const Mesh* piece = nullptr; ///< Owning piece, read only
    uint32_t index_offset = 0;   ///< Range start in the piece's index buffer
    uint32_t index_count = 0;    ///< Range length
    uint32_t vertex_base = 0;    ///< What to add to each index in merged space
};

/**
 * @brief Concatenate the pieces of one chunk into a single mesh, keyed by material
 *
 * Vertices are copied in the order @p pieces gives; index ranges are emitted
 * grouped by MaterialKey in ascending packed() order, so the merged mesh has at
 * most one SubMesh per key and its layout does not depend on which piece happened
 * to introduce a key first.
 *
 * @param pieces Candidate meshes. Null, empty and malformed entries are skipped.
 * @return The merged mesh. Empty when nothing contributed.
 */
[[nodiscard]] Mesh merge_pieces(const std::vector<const Mesh*>& pieces) {
    Mesh merged;

    // Pass 1: collect every usable range, in a deterministic order, and with it
    // the list of pieces that actually contributed one. A piece whose ranges were
    // all unusable never gets a vertex base assigned, so its vertices must not be
    // written either or every later base is wrong.
    std::vector<KeyedRange> ranges;
    std::vector<const Mesh*> contributors;
    size_t total_vertices = 0;
    size_t total_indices = 0;

    for (const Mesh* piece : pieces) {
        if (piece == nullptr || piece->vertices.empty() || piece->indices.size() < 3u) {
            continue;
        }
        if (!indices_are_in_range(*piece)) {
            spdlog::warn("build_chunk_lod: skipping a piece with out-of-range indices");
            continue;
        }

        const uint32_t base = static_cast<uint32_t>(total_vertices);
        bool contributed = false;

        for (const SubMesh& sub : piece->effective_submeshes()) {
            if (!is_triangle_range(*piece, sub)) {
                continue;
            }
            KeyedRange range;
            range.key = MaterialKey{sub.material, sub.variant};
            range.packed = range.key.packed();
            range.piece = piece;
            range.index_offset = sub.index_offset;
            range.index_count = sub.index_count;
            range.vertex_base = base;
            ranges.push_back(range);
            total_indices += sub.index_count;
            contributed = true;
        }

        if (contributed) {
            contributors.push_back(piece);
            total_vertices += piece->vertices.size();
        }
    }

    if (ranges.empty()) {
        return merged;
    }

    // Pass 2: the vertex buffer, in the same piece order the bases were assigned in.
    merged.vertices.reserve(total_vertices);
    for (const Mesh* piece : contributors) {
        merged.vertices.insert(merged.vertices.end(), piece->vertices.begin(),
                               piece->vertices.end());
    }

    // Pass 3: the index buffer, grouped by key. stable_sort keeps the piece order
    // inside a key, so the merge is reproducible.
    std::stable_sort(ranges.begin(), ranges.end(),
                     [](const KeyedRange& a, const KeyedRange& b) { return a.packed < b.packed; });

    merged.indices.reserve(total_indices);
    for (const KeyedRange& range : ranges) {
        const uint32_t offset = static_cast<uint32_t>(merged.indices.size());
        const uint32_t* src = range.piece->indices.data() + range.index_offset;
        for (uint32_t i = 0; i < range.index_count; ++i) {
            merged.indices.push_back(src[i] + range.vertex_base);
        }
        if (!merged.submeshes.empty() && merged.submeshes.back().material == range.key.material &&
            merged.submeshes.back().variant == range.key.variant) {
            merged.submeshes.back().index_count += range.index_count;
        } else {
            merged.submeshes.push_back(
                SubMesh{offset, range.index_count, range.key.material, range.key.variant});
        }
    }

    merged.compute_bounds();
    return merged;
}

// ============================================================================
// The lock set
// ============================================================================

/**
 * @brief Distance from a local-space point to the PERIMETER of a rectangle
 *
 * The unsigned form of the usual box distance: zero on the rectangle itself and
 * growing in both directions, so a vertex deep inside the chunk and a vertex well
 * outside it are both far from the perimeter and both free.
 */
[[nodiscard]] double distance_to_rect_perimeter(const glm::dvec2& point,
                                                const glm::dvec2& rect_min,
                                                const glm::dvec2& rect_max) {
    const glm::dvec2 center = (rect_min + rect_max) * 0.5;
    const glm::dvec2 half = (rect_max - rect_min) * 0.5;
    const glm::dvec2 q = glm::abs(point - center) - half;

    // Outside distance: the length of the positive part of q.
    const double outside_x = std::max(q.x, 0.0);
    const double outside_y = std::max(q.y, 0.0);
    const double outside = std::sqrt(outside_x * outside_x + outside_y * outside_y);

    // Inside distance: how far the nearest side is, which is the negative part.
    const double inside = std::min(std::max(q.x, q.y), 0.0);

    return outside > 0.0 ? outside : -inside;
}

/**
 * @brief Mark every vertex within @p band of the chunk rectangle as unmovable
 *
 * Written in meshoptimizer's `vertex_lock` encoding: `meshopt_SimplifyVertex_Lock`
 * for a pinned vertex, 0 for a free one.
 *
 * @param mesh     Merged mesh, world space, Y up
 * @param min_xy   Chunk minimum, LOCAL 2D metres
 * @param max_xy   Chunk maximum, LOCAL 2D metres
 * @param band     Lock band width, metres. Non-positive locks nothing.
 * @param out      Resized to the vertex count and overwritten
 * @return How many vertices were locked
 */
size_t build_lock_set(const Mesh& mesh, const glm::dvec2& min_xy, const glm::dvec2& max_xy,
                      float band, std::vector<unsigned char>& out) {
    out.assign(mesh.vertices.size(), 0u);

    // A degenerate or inverted rectangle has no perimeter to speak of, and a
    // non-positive band was asked to lock nothing. Both mean "no border": every
    // vertex is free. That cracks chunk seams, which is the caller's choice to
    // make and is documented on ChunkLodConfig::border_band.
    if (!(band > 0.0f) || !(max_xy.x > min_xy.x) || !(max_xy.y > min_xy.y)) {
        return 0;
    }

    const double band_d = static_cast<double>(band);
    size_t locked = 0;
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const glm::vec3& p = mesh.vertices[i].position;

        // World (x, height, -y) back to local (x, y). Height plays no part: a
        // bridge deck above a chunk edge is pinned exactly as the road under it is,
        // because the neighbouring chunk pins its own copy the same way.
        const glm::dvec2 local(static_cast<double>(p.x), -static_cast<double>(p.z));

        if (distance_to_rect_perimeter(local, min_xy, max_xy) <= band_d) {
            out[i] = static_cast<unsigned char>(meshopt_SimplifyVertex_Lock);
            ++locked;
        }
    }
    return locked;
}

/**
 * @brief Grow the lock set to cover every triangle that already touches it
 *
 * meshoptimizer's `vertex_lock` says a vertex may not MOVE. It does not say the
 * vertex may not disappear, and those are different promises. Take a triangle
 * with two locked vertices and one free one: collapsing the free vertex onto
 * either locked one is legal, it flattens the triangle, and meshoptimizer drops
 * the flattened triangle. If that was the last triangle naming a locked vertex,
 * the vertex is gone from the output even though nothing ever moved it -- and a
 * vertex on the chunk rectangle that survives in one chunk and not in its
 * neighbour is exactly the crack the lock set exists to prevent. Measured on the
 * Lucan extract before this pass, 3 of 122 rim vertices in the densest chunk
 * vanished that way.
 *
 * Locking all three vertices of any triangle that has one closes it: with no
 * unlocked vertex left there is no legal collapse source, the triangle cannot be
 * flattened, and every vertex it names survives. One ring is enough, and one ring
 * is cheap -- it takes the densest chunk from 122 pinned vertices to a few
 * hundred out of nineteen thousand.
 *
 * @param mesh      The merged mesh
 * @param pos_remap Position-canonical vertex ids, so every wedge of a pinned
 *                  point is pinned with it
 * @param lock      Grown in place
 * @return How many vertices are locked afterwards
 */
size_t dilate_lock_set(const Mesh& mesh, const std::vector<uint32_t>& pos_remap,
                       std::vector<unsigned char>& lock) {
    const size_t vertex_count = mesh.vertices.size();
    const size_t triangle_end = mesh.indices.size() - (mesh.indices.size() % 3u);

    // One ring outward, read from the ORIGINAL flags so the growth stops at one
    // ring rather than flooding along the mesh.
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

    // Propagate to every wedge of a locked position. build_lock_set() is already
    // positional, but the ring above locked vertices by INDEX, and split_by_anchor()
    // reads the flag per index. Without this a second wedge of a newly locked point
    // could read as free and let its component be pruned.
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
// Component anchoring
//
// meshopt_SimplifyPrune is the only mechanism that reduces a component too small
// to decimate -- a lane arrow, a stop line, one zebra stripe -- and a chunk
// carries hundreds of them. Without it the Markings range returns 100% of its
// triangles at every level and holds the chunk total up on its own; the measured
// coarsest level goes from 10% of the material to 100% of it.
//
// But meshoptimizer's pruning does NOT consult `vertex_lock`. pruneComponents()
// in simplifier.cpp takes the component errors and nothing else, so a component
// that straddles the chunk rectangle can be deleted with its locked vertices
// still in it, and the neighbouring chunk -- which kept its own copy of that
// boundary -- is left facing a hole. That silently breaks the crack-free
// guarantee the lock set exists to provide.
//
// Pruning is a per-COMPONENT decision, so the fix is to make the split per
// component too. A component holding at least one locked vertex is ANCHORED and
// is simplified with pruning off; everything else is FLOATING and is simplified
// with pruning on. Components are disjoint by definition, so separating them into
// two calls costs no connectivity: nothing that could have collapsed together is
// pulled apart. In practice the carriageway spans the chunk and is anchored, the
// paint in the middle of it floats, and only the handful of markings within
// `border_band` of the rim are held at full detail.
// ============================================================================

/// Integer cell coordinate on the connectivity grid. Double precision, because
/// local map coordinates reach tens of kilometres and a float division at 1e-4
/// stops resolving consecutive cells well before that.
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
 * Two vertices at one position are two wedges of one surface point -- the weld
 * refuses to merge a kerb top into a kerb face, and refuses to merge across a
 * material -- and meshoptimizer's own `remap` treats them as one vertex for
 * connectivity. This reproduces that, so a component here is a component there.
 *
 * The representative is the LOWEST index of the group, and groups are visited in
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

/// Union-find over vertices, path-compressed, union by lowest representative so
/// the root of a set is always its lowest member and the labelling is stable.
class DisjointSet {
public:
    explicit DisjointSet(size_t count) : m_parent(count) {
        for (size_t i = 0; i < count; ++i) {
            m_parent[i] = static_cast<uint32_t>(i);
        }
    }

    [[nodiscard]] uint32_t find(uint32_t v) {
        while (m_parent[v] != v) {
            m_parent[v] = m_parent[m_parent[v]];
            v = m_parent[v];
        }
        return v;
    }

    void unite(uint32_t a, uint32_t b) {
        const uint32_t ra = find(a);
        const uint32_t rb = find(b);
        if (ra == rb) {
            return;
        }
        // Lowest wins, so the root does not depend on the order edges arrive in.
        if (ra < rb) {
            m_parent[rb] = ra;
        } else {
            m_parent[ra] = rb;
        }
    }

private:
    std::vector<uint32_t> m_parent;
};

/**
 * @brief Split one index range into its anchored and floating triangles
 *
 * A triangle is anchored when its connected component holds at least one locked
 * vertex. Both outputs keep the input's triangle order, so the split is
 * deterministic.
 *
 * @param base      The mesh the range belongs to
 * @param pos_remap Position-canonical vertex ids from build_position_remap()
 * @param lock      Per-vertex lock flags
 * @param begin     Range start in `base.indices`
 * @param count     Range length, a multiple of 3
 * @param anchored  Overwritten with the triangles that must not be pruned
 * @param floating  Overwritten with the triangles that may be
 */
void split_by_anchor(const Mesh& base, const std::vector<uint32_t>& pos_remap,
                     const std::vector<unsigned char>& lock, size_t begin, size_t count,
                     std::vector<uint32_t>& anchored, std::vector<uint32_t>& floating) {
    anchored.clear();
    floating.clear();

    DisjointSet sets(base.vertices.size());
    for (size_t t = begin; t + 2u < begin + count; t += 3u) {
        const uint32_t a = pos_remap[base.indices[t]];
        const uint32_t b = pos_remap[base.indices[t + 1u]];
        const uint32_t c = pos_remap[base.indices[t + 2u]];
        sets.unite(a, b);
        sets.unite(b, c);
    }

    // A component is anchored when any vertex of it is locked. Locks are assigned
    // by POSITION, so every wedge of a locked point carries the flag and it does
    // not matter which one the remap chose as representative.
    std::unordered_map<uint32_t, bool> anchored_root;
    anchored_root.reserve(count / 3u + 1u);
    for (size_t i = begin; i < begin + count; ++i) {
        const uint32_t v = base.indices[i];
        if (lock[v] != 0u) {
            anchored_root[sets.find(pos_remap[v])] = true;
        }
    }

    for (size_t t = begin; t + 2u < begin + count; t += 3u) {
        const uint32_t root = sets.find(pos_remap[base.indices[t]]);
        const bool is_anchored = anchored_root.find(root) != anchored_root.end();
        std::vector<uint32_t>& into = is_anchored ? anchored : floating;
        into.push_back(base.indices[t]);
        into.push_back(base.indices[t + 1u]);
        into.push_back(base.indices[t + 2u]);
    }
}

// ============================================================================
// Simplification
// ============================================================================

/// One simplified index range, waiting to be reassembled into a level.
struct SimplifiedRange {
    MaterialKey key{};
    std::vector<uint32_t> indices;
};

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
 * @brief Run meshoptimizer over one triangle list with the chunk lock set applied
 *
 * @param base          Level 0. Supplies positions, normals and the vertex count.
 * @param source        Indices to simplify. Not necessarily a range of base.
 * @param count         Length of @p source
 * @param lock          Per-vertex lock flags, `base.vertices.size()` long
 * @param target        Target index count, already a multiple of 3
 * @param target_error  Relative error bound
 * @param allow_prune   Add meshopt_SimplifyPrune. Only ever true for geometry no
 *                      lock touches -- see split_by_anchor().
 * @param out           Overwritten with the simplified indices
 *
 * @note meshopt_simplifyWithAttributes() with an attribute count of zero is the
 *       same algorithm as meshopt_simplify(); it is the only entry point that
 *       takes a `vertex_lock` array, which is the whole reason this file exists.
 *       Feeding the vertex NORMAL in as an attribute was tried and removed: over
 *       weights from 0 to 1.0 it moved the output triangle count of the densest
 *       Lucan chunk by less than half a percent and left the kerb step
 *       bit-for-bit as tall. The position quadric already refuses to collapse a
 *       150 mm vertical face into the surface beside it, so the attribute error
 *       was paying for a promise the geometry was keeping on its own.
 */
void simplify_indices(const Mesh& base, const uint32_t* source, size_t count,
                      const std::vector<unsigned char>& lock, size_t target, float target_error,
                      bool allow_prune, std::vector<uint32_t>& out) {
    if (count < 3u) {
        out.clear();
        return;
    }

    // meshopt's worst case output is the INPUT index count, not the target, so the
    // destination has to be sized for no reduction at all.
    out.resize(count);

    const unsigned int options =
        kSimplifyOptions | (allow_prune ? static_cast<unsigned int>(meshopt_SimplifyPrune) : 0u);
    float result_error = 0.0f;

    const size_t result = meshopt_simplifyWithAttributes(
        out.data(), source, count, position_stream(base), base.vertices.size(), sizeof(Vertex),
        nullptr, 0u, nullptr, 0u, lock.data(), target, target_error, options, &result_error);

    out.resize(result);
}

/**
 * @brief Simplify one index range, pruning only the components no lock touches
 *
 * The anchored and floating halves get the same ratio, so the range as a whole
 * still lands near it, and neither half is asked to make up for the other.
 *
 * @param out Overwritten with the simplified indices of the whole range
 */
void simplify_range(const Mesh& base, const std::vector<uint32_t>& pos_remap,
                    const std::vector<unsigned char>& lock, size_t begin, size_t count,
                    float ratio, float target_error, std::vector<uint32_t>& out) {
    std::vector<uint32_t> anchored;
    std::vector<uint32_t> floating;
    split_by_anchor(base, pos_remap, lock, begin, count, anchored, floating);

    std::vector<uint32_t> anchored_out;
    std::vector<uint32_t> floating_out;

    simplify_indices(base, anchored.data(), anchored.size(), lock,
                     triangle_target(anchored.size(), ratio), target_error, false, anchored_out);
    simplify_indices(base, floating.data(), floating.size(), lock,
                     triangle_target(floating.size(), ratio), target_error, true, floating_out);

    out.clear();
    out.reserve(anchored_out.size() + floating_out.size());
    out.insert(out.end(), anchored_out.begin(), anchored_out.end());
    out.insert(out.end(), floating_out.begin(), floating_out.end());
}

/**
 * @brief Simplify each material range of @p base separately and reassemble a level
 *
 * The range's share of the level target is its share of level 0, so the level
 * total lands near the requested ratio without any one material being forced to
 * hit it alone.
 *
 * Off by default. See ChunkLodConfig::simplify_per_material for what it costs and
 * why the protection it buys turned out to be unnecessary.
 */
[[nodiscard]] Mesh simplify_per_material(const Mesh& base, float ratio, float target_error,
                                         const std::vector<uint32_t>& pos_remap,
                                         const std::vector<unsigned char>& lock) {
    std::vector<SimplifiedRange> produced;
    std::vector<uint32_t> scratch;

    for (const SubMesh& sub : base.effective_submeshes()) {
        const size_t begin = sub.index_offset;
        const size_t count = sub.index_count;
        const MaterialKey key{sub.material, sub.variant};

        // Not a triangle list this file can reason about: carry it through at full
        // detail rather than dropping the geometry it names.
        if (!is_triangle_range(base, sub)) {
            if (count > 0 && begin + count <= base.indices.size()) {
                produced.push_back({key, std::vector<uint32_t>(base.indices.begin() + begin,
                                                               base.indices.begin() + begin +
                                                                   count)});
            }
            continue;
        }

        if (triangle_target(count, ratio) >= count) {
            produced.push_back({key, std::vector<uint32_t>(base.indices.begin() + begin,
                                                           base.indices.begin() + begin + count)});
            continue;
        }

        simplify_range(base, pos_remap, lock, begin, count, ratio, target_error, scratch);
        if (scratch.size() < 3u) {
            continue;
        }
        produced.push_back({key, scratch});
    }

    Mesh level;
    if (produced.empty()) {
        return level;
    }

    // Ascending packed key, which is the order merge_pieces() already left behind,
    // restated here so a level's layout does not depend on the order
    // effective_submeshes() happened to report.
    std::stable_sort(produced.begin(), produced.end(),
                     [](const SimplifiedRange& a, const SimplifiedRange& b) {
                         return a.key.packed() < b.key.packed();
                     });

    level.vertices = base.vertices;
    size_t total = 0;
    for (const SimplifiedRange& range : produced) {
        total += range.indices.size();
    }
    level.indices.reserve(total);

    for (const SimplifiedRange& range : produced) {
        const uint32_t offset = static_cast<uint32_t>(level.indices.size());
        level.indices.insert(level.indices.end(), range.indices.begin(), range.indices.end());
        const uint32_t appended = static_cast<uint32_t>(range.indices.size());
        if (!level.submeshes.empty() && level.submeshes.back().material == range.key.material &&
            level.submeshes.back().variant == range.key.variant) {
            level.submeshes.back().index_count += appended;
        } else {
            level.submeshes.push_back(
                SubMesh{offset, appended, range.key.material, range.key.variant});
        }
    }

    compact_vertices(level);
    return level;
}

/**
 * @brief Simplify every range of @p base together and re-attribute the survivors
 *
 * The default, and the mode that actually reaches the requested ratios. The whole
 * index buffer goes in as one surface, so the kerb, the gutter and the lane are
 * one collapsible sheet rather than three ribbons that each have to be decimated
 * against their own borders.
 *
 * ### Why this does not destroy the kerb
 *
 * It was expected to, and it does not. Two independent measurements on the Lucan
 * extract:
 *
 * - The vertical extent of the Curb range's triangles is unchanged at every
 *   level. Mean 0.076 m at level 0 and 0.079 m at the coarsest, maximum exactly
 *   0.150 m throughout -- the full kerb height, still there, on a range decimated
 *   to 19% of its triangles. A 150 mm vertical face carries enormous quadric
 *   error, so the position metric refuses to flatten it without being told to.
 * - No triangle of any simplified level names a vertex that did not carry that
 *   triangle's material at level 0, over 158,499 simplified triangles. The weld
 *   keys on the material set (WeldConfig::respect_material), so the merged mesh
 *   already carries the material boundaries as attribute seams and meshoptimizer
 *   will not collapse across one.
 *
 * So the protection per-material simplification buys by fragmenting the mesh is
 * protection the mesh already had.
 *
 * ### Re-attribution
 *
 * A surviving triangle may still name vertices from two ranges, because the weld
 * leaves the boundary COLUMN shared -- a gutter/kerb vertex is referenced by both
 * and carries both bits. Each vertex is tagged with the key of the lowest-packed
 * range that referenced it, and a triangle takes the key its three vertices agree
 * on most; a three-way tie goes to the lowest packed key. Both rules are total
 * orders, so the result does not depend on visitation order.
 */
[[nodiscard]] Mesh simplify_whole_mesh(const Mesh& base, float ratio, float target_error,
                                       const std::vector<uint32_t>& pos_remap,
                                       const std::vector<unsigned char>& lock) {
    Mesh level;

    const size_t count = base.indices.size();
    if (count < 3u || (count % 3u) != 0u) {
        return level;
    }

    // Per-vertex material tag, taken from the lowest-packed range that names it.
    constexpr uint32_t kNoKey = 0xFFFFFFFFu;
    std::vector<uint32_t> vertex_key(base.vertices.size(), kNoKey);
    std::unordered_map<uint32_t, MaterialKey> key_of_packed;
    for (const SubMesh& sub : base.effective_submeshes()) {
        if (!is_triangle_range(base, sub)) {
            continue;
        }
        const MaterialKey key{sub.material, sub.variant};
        const uint32_t packed = key.packed();
        key_of_packed.emplace(packed, key);
        const size_t begin = sub.index_offset;
        for (size_t i = begin; i < begin + sub.index_count; ++i) {
            const uint32_t index = base.indices[i];
            if (index < vertex_key.size() && packed < vertex_key[index]) {
                vertex_key[index] = packed;
            }
        }
    }

    std::vector<uint32_t> simplified;
    if (triangle_target(count, ratio) >= count) {
        simplified.assign(base.indices.begin(), base.indices.end());
    } else {
        simplify_range(base, pos_remap, lock, 0, count, ratio, target_error, simplified);
    }
    if (simplified.size() < 3u) {
        return level;
    }

    // Bucket the survivors by key, in ascending packed order.
    std::unordered_map<uint32_t, std::vector<uint32_t>> buckets;
    for (size_t t = 0; t + 2u < simplified.size(); t += 3u) {
        const uint32_t a = simplified[t];
        const uint32_t b = simplified[t + 1u];
        const uint32_t c = simplified[t + 2u];
        if (a >= vertex_key.size() || b >= vertex_key.size() || c >= vertex_key.size()) {
            continue;
        }
        const uint32_t ka = vertex_key[a];
        const uint32_t kb = vertex_key[b];
        const uint32_t kc = vertex_key[c];

        uint32_t chosen = std::min({ka, kb, kc});
        // Majority beats the minimum when there is one.
        if (kb == kc && kb != kNoKey) {
            chosen = kb;
        }
        if (ka == kc && ka != kNoKey) {
            chosen = ka;
        }
        if (ka == kb && ka != kNoKey) {
            chosen = ka;
        }
        if (chosen == kNoKey) {
            continue;
        }
        auto& bucket = buckets[chosen];
        bucket.push_back(a);
        bucket.push_back(b);
        bucket.push_back(c);
    }
    if (buckets.empty()) {
        return level;
    }

    std::vector<uint32_t> ordered;
    ordered.reserve(buckets.size());
    for (const auto& entry : buckets) {
        ordered.push_back(entry.first);
    }
    std::sort(ordered.begin(), ordered.end());

    level.vertices = base.vertices;
    for (uint32_t packed : ordered) {
        const std::vector<uint32_t>& bucket = buckets[packed];
        const uint32_t offset = static_cast<uint32_t>(level.indices.size());
        level.indices.insert(level.indices.end(), bucket.begin(), bucket.end());
        const auto found = key_of_packed.find(packed);
        const MaterialKey key =
            found != key_of_packed.end() ? found->second : MaterialKey{MaterialId::Default, 0u};
        level.submeshes.push_back(
            SubMesh{offset, static_cast<uint32_t>(bucket.size()), key.material, key.variant});
    }

    compact_vertices(level);
    return level;
}

} // namespace

// ----------------------------------------------------------------------------

ChunkLod build_chunk_lod(const std::vector<const Mesh*>& pieces, const glm::dvec2& chunk_min,
                         const glm::dvec2& chunk_max, const ChunkLodConfig& cfg) {
    ChunkLod chunk;

    // ------------------------------------------------------------------
    // 1. Merge, then weld
    //
    // The weld is the half of the merge that does the work. Concatenation alone
    // leaves the shared column of every junction seam duplicated, and the
    // simplifier cannot collapse an edge that is not there.
    // ------------------------------------------------------------------
    Mesh base = merge_pieces(pieces);
    if (base.indices.size() < 3u || base.vertices.empty()) {
        return chunk;
    }

    const size_t merged_vertices = base.vertices.size();
    weld_vertices(base);

    // Level 0 is the finished render mesh: merged, welded and reordered, never
    // simplified. optimize_mesh()'s fetch pass also drops any vertex the merge
    // carried across that no surviving index names.
    optimize_mesh(base, reorder_config());
    base.compute_bounds();

    const size_t base_indices = base.indices.size();
    if (base_indices < 3u) {
        return chunk;
    }

    const float radius = std::max(base.bounds.radius(), kMinBoundsRadius);

    // A chunk with a single small piece gets here and leaves with exactly one
    // level. Every ratio below will either fail to beat kMinLevelReduction or
    // produce nothing, and a one-level chain is the right answer -- it is still
    // the merged, welded, reordered mesh, which is strictly better than the
    // pieces that went in.
    chunk.levels.push_back(base);
    chunk.switch_distances.push_back(0.0f);

    // ------------------------------------------------------------------
    // 2. The lock set
    //
    // This replaces meshopt_SimplifyLockBorder entirely. The flag asks a
    // topological question -- "is this vertex on an open boundary" -- and on a 3 m
    // ribbon the answer is always yes. The array asks the positional question that
    // was meant all along, and only the rim of the chunk answers yes to it.
    // ------------------------------------------------------------------
    std::vector<unsigned char> lock;
    const size_t band_locked = build_lock_set(base, chunk_min, chunk_max, cfg.border_band, lock);

    // Connectivity, computed once and reused by every level. The pruning decision
    // reads it -- see split_by_anchor() -- and so does the lock dilation.
    const std::vector<uint32_t> pos_remap = build_position_remap(base);
    const size_t locked = band_locked > 0 ? dilate_lock_set(base, pos_remap, lock) : 0u;

    const float target_error = std::max(cfg.target_error, 0.0f);

    // ------------------------------------------------------------------
    // 3. The levels
    // ------------------------------------------------------------------
    size_t previous_indices = base_indices;
    for (float ratio : cfg.ratios) {
        if (!(ratio > 0.0f) || !(ratio < 1.0f)) {
            continue;
        }

        Mesh level = cfg.simplify_per_material
                         ? simplify_per_material(base, ratio, target_error, pos_remap, lock)
                         : simplify_whole_mesh(base, ratio, target_error, pos_remap, lock);
        if (level.indices.size() < 3u) {
            continue;
        }

        // A level that is nearly the previous one costs a second resident copy of
        // the same geometry and saves nothing, so it is dropped along with its
        // switch distance. A later, more aggressive ratio still gets its turn.
        const double reduction =
            1.0 - static_cast<double>(level.indices.size()) / static_cast<double>(previous_indices);
        if (reduction < static_cast<double>(kMinLevelReduction)) {
            continue;
        }

        optimize_mesh(level, reorder_config());
        level.compute_bounds();

        // Achieved, not requested. The error bound and the lock set both refuse
        // collapses the ratio asked for, so a distance derived from the request
        // would switch to a level that never got that cheap.
        const float achieved = static_cast<float>(static_cast<double>(level.indices.size()) /
                                                  static_cast<double>(base_indices));
        const float distance = radius * kLodSwitchFactor / std::sqrt(std::max(achieved, 1e-6f));

        previous_indices = level.indices.size();
        chunk.levels.push_back(std::move(level));
        chunk.switch_distances.push_back(distance);
    }

    spdlog::debug("build_chunk_lod: {} pieces -> {} verts merged, {} welded, {} tris; "
                  "{} locked; {} levels",
                  pieces.size(), merged_vertices, base.vertices.size(), base_indices / 3u, locked,
                  chunk.levels.size());

    return chunk;
}

} // namespace stratum::osm::road
