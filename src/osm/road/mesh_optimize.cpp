/**
 * @file mesh_optimize.cpp
 * @brief Implementation of vertex welding, meshoptimizer reordering, and LOD chains
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * See mesh_optimize.hpp for the contract. This file is where the three promises
 * that make that contract non-trivial are actually kept:
 *
 * 1. **A crease survives welding.** The weld key carries the vertex NORMAL, so
 *    the two coincident vertices the corridor extruder deliberately emits where
 *    the kerb face meets the kerb top -- same position, same material, normals 90
 *    degrees apart -- hash into the same spatial cell and then fail the normal
 *    test. They stay two vertices and the kerb stays a kerb.
 *
 * 2. **SubMesh ranges survive reordering.** Welding never adds, drops or moves an
 *    index, so `index_offset` and `index_count` are untouched by construction.
 *    optimize_mesh() reorders triangles strictly inside one range at a time, so a
 *    triangle can never migrate across a range boundary. Only the vertex-fetch
 *    pass touches the whole buffer, and it renumbers vertices rather than moving
 *    triangles, which no range offset depends on.
 *
 * 3. **A material boundary survives simplification.** Each range is extracted,
 *    simplified alone, and reassembled, so meshopt_simplify() never sees the kerb
 *    and the carriageway as one collapsible surface.
 *
 * ### Determinism
 *
 * Every golden test in tests/road hashes mesh output, so nothing here may depend
 * on hash iteration order, allocation addresses or thread count. The two places
 * that could:
 *
 * - The weld picks the survivor of a group by MINIMUM original vertex index, not
 *   by whichever candidate the hash table happened to yield first, and vertices
 *   are visited in ascending index order. Both are needed: ascending visitation
 *   alone still lets the chosen candidate depend on bucket order when a vertex
 *   matches several representatives.
 * - The LOD reassembly emits ranges in ascending MaterialId order, which is a
 *   total order, rather than in the order effective_submeshes() reported them.
 */

#include "osm/road/mesh_optimize.hpp"

#include <meshoptimizer.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

namespace stratum::osm::road {

namespace {

/// Sentinel for "no vertex", used as both a chain terminator and a remap hole.
constexpr uint32_t kNoVertex = 0xFFFFFFFFu;

/// meshopt_optimizeOverdraw threshold: accept up to 5% worse vertex-cache behaviour.
constexpr float kOverdrawThreshold = 1.05f;

/// A level must beat the previous one by this fraction to earn its resident copy.
constexpr float kMinLevelReduction = 0.10f;

/// Floor on the bounding radius used for LOD thresholds, so a degenerate mesh
/// cannot produce a divide-by-zero switch distance.
constexpr float kMinBoundsRadius = 1e-3f;

// ============================================================================
// Shared helpers
// ============================================================================

/**
 * @brief True when @p sub is a usable triangle-list range of @p mesh
 *
 * A producer that leaves a range straddling a triangle -- offset or count not a
 * multiple of 3 -- has geometry this file cannot reason about as triangles, so
 * the range is passed through untouched rather than reordered or simplified.
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
 * meshoptimizer asserts on out-of-range indices in debug and reads out of bounds
 * in release, so a malformed mesh is left alone rather than handed to it.
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

/// First float of the vertex array's position channel, for meshoptimizer's
/// `vertex_positions` parameter. Vertex::position is the first member, so the
/// stride is sizeof(Vertex) and the offset is zero.
[[nodiscard]] const float* position_stream(const Mesh& mesh) {
    return &mesh.vertices[0].position.x;
}

/**
 * @brief Unit-length copy of @p v, or the zero vector when it has no direction
 *
 * Separating "no direction" from "some direction" matters for the weld tests: two
 * vertices that both carry a zero normal agree with each other, but a zero normal
 * agrees with nothing else, and glm::normalize would answer both questions with a
 * NaN.
 */
[[nodiscard]] glm::vec3 normalize_or_zero(const glm::vec3& v) {
    const float length_sq = glm::dot(v, v);
    if (!(length_sq > 1e-20f) || !std::isfinite(length_sq)) {
        return glm::vec3(0.0f);
    }
    return v / std::sqrt(length_sq);
}

// ============================================================================
// Welding
// ============================================================================

/**
 * @brief Precomputed, per-vertex form of the weld key's directional channels
 *
 * Normal and tangent are compared by dot product, which needs them normalised.
 * Normalising once per vertex rather than once per candidate pair turns an
 * O(candidates) square root into an O(vertices) one, and -- more importantly --
 * makes the comparison symmetric, so match(a, b) and match(b, a) can never
 * disagree because of rounding.
 */
struct WeldChannels {
    std::vector<glm::vec3> normal;    ///< Unit normal, or zero when degenerate
    std::vector<glm::vec3> tangent;   ///< Unit tangent.xyz, or zero when degenerate
    std::vector<uint8_t> handedness;  ///< 1 when tangent.w < 0, else 0
};

/// Resolved comparison tolerances, hoisted out of the inner loop.
struct WeldTolerances {
    double cell_size = 1e-4;         ///< Spatial hash cell edge, metres
    float position_epsilon = 1e-4f;  ///< Per-component position slack, metres
    float uv_epsilon = 1e-4f;        ///< Per-component UV slack
    float color_epsilon = 0.0f;      ///< Per-component colour slack
    float normal_dot_min = 0.99f;    ///< Minimum dot product for a normal match
    float tangent_dot_min = 0.75f;   ///< Minimum dot product for a tangent match
    bool compare_normal = true;      ///< False when normal_epsilon disables the test
    bool compare_tangent = true;     ///< False when tangent_epsilon disables the test
};

/**
 * @brief Integer cell coordinate of a position on the weld grid
 *
 * Quantisation runs in double precision on purpose. Local map coordinates reach
 * tens of kilometres, and `50000.0f / 1e-4f` is 5e8 -- far past the 2^24 where a
 * float stops representing consecutive integers, so a float division here would
 * quantise distinct cells onto the same value in a way that varies with the
 * origin of the extract.
 */
[[nodiscard]] inline int64_t quantise(float value, double cell_size) {
    return static_cast<int64_t>(std::floor(static_cast<double>(value) / cell_size));
}

/// FNV-1a over the three cell coordinates. Collisions cost a redundant candidate
/// test and nothing else: every candidate is re-verified against the real
/// tolerances before it can win.
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
 * @brief Decide whether two vertices are the same vertex
 *
 * Ordered cheapest-test-first: position rejects almost every candidate that a
 * neighbouring cell produced, and the material mask rejects the paint-over-
 * asphalt case without touching any float.
 */
[[nodiscard]] bool weld_match(const Mesh& mesh,
                              const WeldChannels& channels,
                              const std::vector<uint16_t>& material_mask,
                              const WeldTolerances& tol,
                              uint32_t a,
                              uint32_t b) {
    const Vertex& va = mesh.vertices[a];
    const Vertex& vb = mesh.vertices[b];

    if (std::fabs(va.position.x - vb.position.x) > tol.position_epsilon ||
        std::fabs(va.position.y - vb.position.y) > tol.position_epsilon ||
        std::fabs(va.position.z - vb.position.z) > tol.position_epsilon) {
        return false;
    }

    if (material_mask[a] != material_mask[b]) {
        return false;
    }

    if (std::fabs(va.uv.x - vb.uv.x) > tol.uv_epsilon ||
        std::fabs(va.uv.y - vb.uv.y) > tol.uv_epsilon) {
        return false;
    }

    // The crease test. Everything else in this function guards against welding
    // things that were never the same vertex; this one guards against welding two
    // vertices that ARE at the same place and are still not the same vertex.
    if (tol.compare_normal) {
        const glm::vec3& na = channels.normal[a];
        const glm::vec3& nb = channels.normal[b];
        const bool a_zero = (na.x == 0.0f && na.y == 0.0f && na.z == 0.0f);
        const bool b_zero = (nb.x == 0.0f && nb.y == 0.0f && nb.z == 0.0f);
        if (a_zero != b_zero) {
            return false;
        }
        if (!a_zero && glm::dot(na, nb) < tol.normal_dot_min) {
            return false;
        }
    }

    if (std::fabs(va.color.x - vb.color.x) > tol.color_epsilon ||
        std::fabs(va.color.y - vb.color.y) > tol.color_epsilon ||
        std::fabs(va.color.z - vb.color.z) > tol.color_epsilon ||
        std::fabs(va.color.w - vb.color.w) > tol.color_epsilon) {
        return false;
    }

    if (tol.compare_tangent) {
        if (channels.handedness[a] != channels.handedness[b]) {
            return false;
        }
        const glm::vec3& ta = channels.tangent[a];
        const glm::vec3& tb = channels.tangent[b];
        const bool a_zero = (ta.x == 0.0f && ta.y == 0.0f && ta.z == 0.0f);
        const bool b_zero = (tb.x == 0.0f && tb.y == 0.0f && tb.z == 0.0f);
        if (a_zero != b_zero) {
            return false;
        }
        if (!a_zero && glm::dot(ta, tb) < tol.tangent_dot_min) {
            return false;
        }
    }

    return true;
}

} // namespace

// ----------------------------------------------------------------------------

size_t weld_vertices(Mesh& mesh, const WeldConfig& cfg) {
    if (mesh.indices.empty() || mesh.vertices.empty()) {
        return 0;
    }

    const size_t vertex_count = mesh.vertices.size();

    // ------------------------------------------------------------------
    // Material masks
    //
    // The mask of a vertex is the OR of a bit per MaterialId of every range that
    // references it, so the shared column between the gutter (Asphalt) and the
    // kerb top (Curb) carries both bits and welds against its counterpart from a
    // neighbouring piece, which carries both bits too. A vertex nothing
    // references keeps mask 0 and can only ever weld with another such vertex.
    // ------------------------------------------------------------------
    std::vector<uint16_t> material_mask(vertex_count, 0u);
    if (cfg.respect_material) {
        for (const SubMesh& sub : mesh.effective_submeshes()) {
            const uint8_t slot = static_cast<uint8_t>(sub.material);
            if (slot >= static_cast<uint8_t>(MaterialId::Count)) {
                continue;
            }
            const uint16_t bit = static_cast<uint16_t>(1u << slot);
            const size_t begin = std::min<size_t>(sub.index_offset, mesh.indices.size());
            const size_t end = std::min<size_t>(
                static_cast<size_t>(sub.index_offset) + sub.index_count, mesh.indices.size());
            for (size_t i = begin; i < end; ++i) {
                const uint32_t index = mesh.indices[i];
                if (index < vertex_count) {
                    material_mask[index] |= bit;
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // Tolerances and per-vertex directional channels
    // ------------------------------------------------------------------
    WeldTolerances tol;
    tol.cell_size = std::max(static_cast<double>(cfg.position_epsilon), 1e-9);
    tol.position_epsilon = std::max(cfg.position_epsilon, 0.0f);
    tol.uv_epsilon = std::max(cfg.uv_epsilon, 0.0f);
    tol.color_epsilon = std::max(cfg.color_epsilon, 0.0f);
    // A dot product never falls below -1, so a slack of 2 or more accepts every
    // pair; that is the documented way to switch the test off entirely.
    tol.compare_normal = cfg.normal_epsilon < 2.0f;
    tol.compare_tangent = cfg.tangent_epsilon < 2.0f;
    tol.normal_dot_min = 1.0f - std::max(cfg.normal_epsilon, 0.0f);
    tol.tangent_dot_min = 1.0f - std::max(cfg.tangent_epsilon, 0.0f);

    WeldChannels channels;
    if (tol.compare_normal) {
        channels.normal.resize(vertex_count);
    }
    if (tol.compare_tangent) {
        channels.tangent.resize(vertex_count);
        channels.handedness.resize(vertex_count);
    }
    for (size_t i = 0; i < vertex_count; ++i) {
        const Vertex& v = mesh.vertices[i];
        if (tol.compare_normal) {
            channels.normal[i] = normalize_or_zero(v.normal);
        }
        if (tol.compare_tangent) {
            channels.tangent[i] = normalize_or_zero(glm::vec3(v.tangent));
            channels.handedness[i] = (v.tangent.w < 0.0f) ? 1u : 0u;
        }
    }

    // ------------------------------------------------------------------
    // Spatial hash over representatives only
    //
    // Only survivors are inserted, so a chain holds one entry per distinct vertex
    // at that position rather than one per duplicate. `next` is a parallel array
    // instead of a per-cell vector: one allocation for the whole grid.
    //
    // A matching pair can straddle a cell boundary when the two positions differ
    // by less than the epsilon but land either side of a grid line, so the 3x3x3
    // neighbourhood is searched rather than the single cell. Skipping the
    // neighbours would make the weld depend on where the extract's origin happens
    // to sit relative to the grid.
    // ------------------------------------------------------------------
    std::unordered_map<uint64_t, uint32_t> cell_head;
    cell_head.reserve(vertex_count * 2u);
    std::vector<uint32_t> chain_next(vertex_count, kNoVertex);

    // Group survivor per vertex, in ORIGINAL indices, not compacted slots. The
    // compaction is a second pass because the flatten guard below has to be able
    // to release a vertex from its group after every group is known, and a slot
    // handed out during the search would already have been folded away.
    std::vector<uint32_t> survivor_of(vertex_count, kNoVertex);

    for (uint32_t i = 0; i < static_cast<uint32_t>(vertex_count); ++i) {
        const glm::vec3& p = mesh.vertices[i].position;
        const int64_t cx = quantise(p.x, tol.cell_size);
        const int64_t cy = quantise(p.y, tol.cell_size);
        const int64_t cz = quantise(p.z, tol.cell_size);

        // Minimum over ALL matches, not the first match found. Chains are
        // newest-first and therefore descend in index, and the neighbourhood is
        // walked in a fixed order, but neither fact is enough on its own: a
        // vertex can match representatives in two different cells, and which of
        // those the hash table yields first is an implementation detail. The
        // minimum is not, so the whole neighbourhood is scanned and the smallest
        // match wins.
        uint32_t survivor = kNoVertex;
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const auto it = cell_head.find(hash_cell(cx + dx, cy + dy, cz + dz));
                    if (it == cell_head.end()) {
                        continue;
                    }
                    for (uint32_t c = it->second; c != kNoVertex; c = chain_next[c]) {
                        if (c < survivor && weld_match(mesh, channels, material_mask, tol, i, c)) {
                            survivor = c;
                        }
                    }
                }
            }
        }

        if (survivor != kNoVertex) {
            survivor_of[i] = survivor_of[survivor];
            continue;
        }

        survivor_of[i] = i;

        // Push onto the cell's chain. emplace rather than operator[] because a
        // value-initialised head would be 0, which is a legitimate vertex index
        // and indistinguishable from an empty cell.
        const auto [slot, inserted] = cell_head.emplace(hash_cell(cx, cy, cz), i);
        if (inserted) {
            chain_next[i] = kNoVertex;
        } else {
            chain_next[i] = slot->second;
            slot->second = i;
        }
    }

    // ------------------------------------------------------------------
    // Flatten guard
    //
    // Two corners of one triangle can pass every attribute test -- a sliver
    // whose two ends sit inside position_epsilon of each other, which the
    // markings, the cul-de-sac fan and the roundabout annulus all emit. Welding
    // that pair collapses the triangle to a line: it keeps its slot in the index
    // buffer, costs a primitive, rasterises nothing, and turns up in every
    // exporter as a degenerate face. The plan's topology contract says there are
    // to be none, and the range contract above says a triangle may not simply be
    // dropped, so the weld that would flatten one is REFUSED instead.
    //
    // A release only ever splits a group, so it cannot create a new collision
    // and one pass over the triangles is enough. The vertex released is the one
    // with the HIGHER original index, which keeps the survivor of every group the
    // lowest-indexed member and leaves the result independent of iteration order.
    // ------------------------------------------------------------------
    size_t refused = 0;
    const size_t triangle_end = mesh.indices.size() - (mesh.indices.size() % 3u);
    for (size_t t = 0; t < triangle_end; t += 3u) {
        for (size_t k = 0; k < 3u; ++k) {
            const uint32_t u = mesh.indices[t + k];
            const uint32_t v = mesh.indices[t + ((k + 1u) % 3u)];
            if (u == v || u >= vertex_count || v >= vertex_count) {
                continue;   // already degenerate on the way in; not the weld's doing
            }
            if (survivor_of[u] != survivor_of[v]) {
                continue;
            }
            // Release the higher-indexed of the pair. It cannot be the group's
            // survivor, because a survivor's index equals its own group id and
            // the two group ids are equal here.
            const uint32_t release = std::max(u, v);
            survivor_of[release] = release;
            ++refused;
        }
    }
    if (refused > 0) {
        spdlog::debug("weld_vertices: refused {} welds that would have flattened a triangle",
                      refused);
    }

    // ------------------------------------------------------------------
    // Compaction, in ascending original-index order
    // ------------------------------------------------------------------
    std::vector<uint32_t> remap(vertex_count, kNoVertex);
    std::vector<Vertex> welded;
    welded.reserve(vertex_count);

    for (uint32_t i = 0; i < static_cast<uint32_t>(vertex_count); ++i) {
        if (survivor_of[i] == i) {
            remap[i] = static_cast<uint32_t>(welded.size());
            welded.push_back(mesh.vertices[i]);
        } else {
            remap[i] = remap[survivor_of[i]];
        }
    }

    const size_t removed = vertex_count - welded.size();

    for (uint32_t& index : mesh.indices) {
        if (index < vertex_count && remap[index] != kNoVertex) {
            index = remap[index];
        }
    }

    mesh.vertices = std::move(welded);
    mesh.compute_bounds();

    return removed;
}

// ============================================================================
// Reordering
// ============================================================================

void optimize_mesh(Mesh& mesh, const LodConfig& cfg) {
    if (mesh.indices.empty() || mesh.vertices.empty()) {
        return;
    }
    if (!indices_are_in_range(mesh)) {
        return;
    }
    if (!cfg.optimize_cache && !cfg.optimize_overdraw && !cfg.optimize_fetch) {
        return;
    }

    const size_t vertex_count = mesh.vertices.size();
    const float* positions = position_stream(mesh);

    // ------------------------------------------------------------------
    // Triangle reordering, strictly within one range at a time
    //
    // Running either pass over the whole index buffer would sort triangles across
    // range boundaries, and every SubMesh would afterwards point at some other
    // material's geometry. meshoptimizer says the same thing in its own header:
    // "If index buffer contains multiple ranges for multiple draw calls, this
    // function needs to be called on each range individually."
    // ------------------------------------------------------------------
    if (cfg.optimize_cache || cfg.optimize_overdraw) {
        std::vector<uint32_t> scratch;
        for (const SubMesh& sub : mesh.effective_submeshes()) {
            if (!is_triangle_range(mesh, sub)) {
                continue;
            }
            uint32_t* range = mesh.indices.data() + sub.index_offset;
            const size_t count = sub.index_count;

            if (cfg.optimize_cache) {
                // In-place is supported: meshoptimizer copies the input when
                // destination == indices.
                meshopt_optimizeVertexCache(range, range, count, vertex_count);
            }
            if (cfg.optimize_overdraw) {
                scratch.assign(range, range + count);
                meshopt_optimizeOverdraw(range, scratch.data(), count, positions, vertex_count,
                                         sizeof(Vertex), kOverdrawThreshold);
            }
        }
    }

    // ------------------------------------------------------------------
    // Vertex reordering, over the whole buffer
    //
    // This is the one pass that is allowed to see every index at once, because it
    // renumbers VERTICES and rewrites indices to match. Triangles keep their
    // positions in the buffer, so every range offset and count is still exactly
    // what it was. Doing it per range instead would be wrong: a vertex shared by
    // two ranges would be renumbered twice.
    //
    // It also drops vertices no index references, which is why the vertex array
    // can shrink here and the bounds are recomputed.
    // ------------------------------------------------------------------
    if (cfg.optimize_fetch && (mesh.indices.size() % 3u) == 0u) {
        const size_t unique = meshopt_optimizeVertexFetch(
            mesh.vertices.data(), mesh.indices.data(), mesh.indices.size(), mesh.vertices.data(),
            vertex_count, sizeof(Vertex));
        if (unique < vertex_count) {
            mesh.vertices.resize(unique);
            mesh.compute_bounds();
        }
    }
}

// ============================================================================
// LOD chain
// ============================================================================

namespace {

/**
 * @brief Compact @p mesh so it carries only the vertices its indices reference
 *
 * Survivors keep ascending original order, so the result does not depend on the
 * order triangles happen to be visited in. optimize_mesh()'s fetch pass would do
 * the same compaction, but only when LodConfig::optimize_fetch is on, and a LOD
 * level that dragged the full-detail vertex buffer around would defeat the point
 * of building it.
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

/**
 * @brief Mark the open-boundary vertices of one index range for meshopt_simplifySloppy
 *
 * meshopt_simplify() finds its own borders; meshopt_simplifySloppy() does not, and
 * takes an explicit vertex lock instead. Without this the fallback would be free to
 * pull a chunk edge inward and open the crack between neighbouring quadtree leaves
 * that LodConfig::lock_borders exists to prevent.
 *
 * A border edge is a directed edge with no opposing twin inside the range. Both of
 * its endpoints lock, and -- as meshoptimizer requires -- the lock is then spread to
 * every vertex sharing the endpoint's position, since two vertices split apart by a
 * crease are one point as far as the sloppy simplifier's grid is concerned.
 *
 * @param mesh   Mesh owning the vertex array; only positions are read
 * @param begin  First index of the range
 * @param count  Number of indices in the range, a multiple of 3
 * @param lock   Output, one byte per vertex, resized and cleared by this function
 */
void mark_border_locks(const Mesh& mesh, size_t begin, size_t count, std::vector<uint8_t>& lock) {
    const size_t vertex_count = mesh.vertices.size();
    lock.assign(vertex_count, 0u);

    std::unordered_map<uint64_t, int32_t> edge_balance;
    edge_balance.reserve(count * 2u);

    const auto edge_key = [](uint32_t a, uint32_t b) {
        return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
    };

    for (size_t t = begin; t + 2 < begin + count; t += 3) {
        const uint32_t v[3] = {mesh.indices[t], mesh.indices[t + 1], mesh.indices[t + 2]};
        for (int e = 0; e < 3; ++e) {
            const uint32_t a = v[e];
            const uint32_t b = v[(e + 1) % 3];
            // One counter per undirected edge, signed by direction: +1 for a->b,
            // -1 for b->a. A paired interior edge nets to zero.
            if (a < b) {
                edge_balance[edge_key(a, b)] += 1;
            } else if (b < a) {
                edge_balance[edge_key(b, a)] -= 1;
            }
        }
    }

    for (const auto& [key, balance] : edge_balance) {
        if (balance == 0) {
            continue;
        }
        const uint32_t a = static_cast<uint32_t>(key >> 32);
        const uint32_t b = static_cast<uint32_t>(key & 0xFFFFFFFFull);
        if (a < vertex_count) {
            lock[a] = 1u;
        }
        if (b < vertex_count) {
            lock[b] = 1u;
        }
    }

    // Spread each lock to every vertex at the same position. Quantised at 0.1 mm,
    // matching WeldConfig::position_epsilon: two vertices closer than that are one
    // point to the sloppy simplifier's grid whatever their normals say.
    constexpr double kLockCell = 1e-4;
    std::unordered_map<uint64_t, uint8_t> locked_cells;
    locked_cells.reserve(vertex_count);
    for (size_t i = 0; i < vertex_count; ++i) {
        if (lock[i] == 0u) {
            continue;
        }
        const glm::vec3& p = mesh.vertices[i].position;
        locked_cells[hash_cell(quantise(p.x, kLockCell), quantise(p.y, kLockCell),
                               quantise(p.z, kLockCell))] = 1u;
    }
    if (locked_cells.empty()) {
        return;
    }
    for (size_t i = 0; i < vertex_count; ++i) {
        if (lock[i] != 0u) {
            continue;
        }
        const glm::vec3& p = mesh.vertices[i].position;
        const uint64_t key = hash_cell(quantise(p.x, kLockCell), quantise(p.y, kLockCell),
                                       quantise(p.z, kLockCell));
        if (locked_cells.find(key) != locked_cells.end()) {
            lock[i] = 1u;
        }
    }
}

/// One simplified material range, before reassembly.
struct SimplifiedRange {
    MaterialId material = MaterialId::Default;
    std::vector<uint32_t> indices;
};

/**
 * @brief Simplify every material range of @p base and reassemble one LOD level
 *
 * @param base         Full-detail, already-optimised level 0. Not modified.
 * @param ratio        Target index fraction of level 0, in (0, 1)
 * @param cfg          Error bound, border locking and the sloppy fallback switch
 * @param sloppy_count Incremented once per range whose sloppy retry was KEPT
 * @return The level. Its indices reference a compacted copy of base's vertices;
 *         empty when nothing survived.
 */
[[nodiscard]] Mesh simplify_level(const Mesh& base, float ratio, const LodConfig& cfg,
                                  size_t& sloppy_count) {
    const size_t vertex_count = base.vertices.size();
    const float* positions = position_stream(base);
    const unsigned int options =
        cfg.lock_borders ? static_cast<unsigned int>(meshopt_SimplifyLockBorder) : 0u;
    const float target_error = std::max(cfg.target_error, 0.0f);

    std::vector<SimplifiedRange> produced;
    std::vector<uint32_t> destination;
    std::vector<uint32_t> sloppy_destination;
    std::vector<uint8_t> lock;

    for (const SubMesh& sub : base.effective_submeshes()) {
        const size_t begin = sub.index_offset;
        const size_t count = sub.index_count;

        // Not a triangle list this file can reason about: carry it through at full
        // detail rather than dropping the geometry it names.
        if (!is_triangle_range(base, sub)) {
            if (count > 0 && begin + count <= base.indices.size()) {
                produced.push_back({sub.material,
                                    std::vector<uint32_t>(base.indices.begin() + begin,
                                                          base.indices.begin() + begin + count)});
            }
            continue;
        }

        // The range's share of the level target is its share of level 0, so the
        // level total lands near the requested ratio without any single material
        // being forced to hit it.
        size_t target = static_cast<size_t>(std::llround(static_cast<double>(count) * ratio));
        target -= target % 3u;
        if (target < 3u) {
            target = 3u;
        }
        if (target >= count) {
            produced.push_back({sub.material,
                                std::vector<uint32_t>(base.indices.begin() + begin,
                                                      base.indices.begin() + begin + count)});
            continue;
        }

        // meshopt_simplify's worst case output is the INPUT index count, not the
        // target, so the destination has to be sized for no reduction at all.
        destination.resize(count);
        float result_error = 0.0f;
        size_t result = meshopt_simplify(destination.data(), base.indices.data() + begin, count,
                                         positions, vertex_count, sizeof(Vertex), target,
                                         target_error, options, &result_error);

        // The constrained simplifier stopped short of the target. Usually that is
        // topology or the locked border refusing a collapse, and the right answer
        // is to accept the bigger level. LodConfig::allow_sloppy permits one retry
        // with the topology-destroying simplifier, kept only if it is strictly
        // better.
        if (result > target && cfg.allow_sloppy) {
            const uint8_t* lock_data = nullptr;
            if (cfg.lock_borders) {
                mark_border_locks(base, begin, count, lock);
                lock_data = lock.data();
            }
            sloppy_destination.resize(count);
            float sloppy_error = 0.0f;
            const size_t sloppy_result = meshopt_simplifySloppy(
                sloppy_destination.data(), base.indices.data() + begin, count, positions,
                vertex_count, sizeof(Vertex), lock_data, target, target_error, &sloppy_error);
            if (sloppy_result >= 3u && sloppy_result < result) {
                destination.swap(sloppy_destination);
                result = sloppy_result;
                ++sloppy_count;
            }
        }

        if (result < 3u) {
            continue;
        }
        produced.push_back(
            {sub.material,
             std::vector<uint32_t>(destination.begin(), destination.begin() + result)});
    }

    Mesh level;
    if (produced.empty()) {
        return level;
    }

    // Ascending MaterialId, merging repeats, which is exactly the shape
    // Mesh::sort_submeshes_by_material() leaves behind. Sorting by the enum rather
    // than by the order effective_submeshes() reported keeps the output stable when
    // a producer emits the same material twice.
    std::stable_sort(produced.begin(), produced.end(),
                     [](const SimplifiedRange& a, const SimplifiedRange& b) {
                         return static_cast<uint8_t>(a.material) < static_cast<uint8_t>(b.material);
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
        if (!level.submeshes.empty() && level.submeshes.back().material == range.material) {
            level.submeshes.back().index_count += appended;
        } else {
            level.submeshes.push_back(SubMesh{offset, appended, range.material});
        }
    }

    compact_vertices(level);
    level.compute_bounds();
    return level;
}

} // namespace

// ----------------------------------------------------------------------------

LodChain build_lod_chain(const Mesh& mesh, const LodConfig& cfg) {
    LodChain chain;
    if (mesh.indices.size() < 3u || mesh.vertices.empty()) {
        return chain;
    }
    if (!indices_are_in_range(mesh)) {
        return chain;
    }

    // Level 0 is the finished render mesh: the input plus its own reordering pass,
    // never simplified. A caller that uses the chain can therefore ignore the mesh
    // it passed in.
    Mesh base = mesh;
    optimize_mesh(base, cfg);
    base.compute_bounds();

    const size_t base_indices = base.indices.size();
    const float radius = std::max(base.bounds.radius(), kMinBoundsRadius);

    chain.levels.push_back(base);
    chain.screen_thresholds.push_back(0.0f);

    if (base_indices < 3u) {
        return chain;
    }

    size_t previous_indices = base_indices;
    for (float ratio : cfg.ratios) {
        if (!(ratio > 0.0f) || !(ratio < 1.0f)) {
            continue;
        }

        Mesh level = simplify_level(base, ratio, cfg, chain.sloppy_simplifications);
        if (level.indices.size() < 3u) {
            continue;
        }

        // A level that is nearly the previous one costs a second resident copy of
        // the same geometry and saves nothing, so it is dropped along with its
        // threshold. A later, more aggressive ratio still gets its turn.
        const double reduction =
            1.0 - static_cast<double>(level.indices.size()) / static_cast<double>(previous_indices);
        if (reduction < static_cast<double>(kMinLevelReduction)) {
            continue;
        }

        optimize_mesh(level, cfg);
        level.compute_bounds();

        // Achieved, not requested. target_error and lock_borders both refuse
        // collapses the ratio asked for, so a threshold derived from the request
        // would switch to a level that never got that cheap.
        //
        // Projected area falls off as 1/d^2, so a level covering `q` of the
        // triangles is worth drawing once the mesh covers `q` of the pixels, which
        // is sqrt(1/q) times as far away.
        const float achieved =
            static_cast<float>(static_cast<double>(level.indices.size()) /
                               static_cast<double>(base_indices));
        const float threshold = radius * kLodSwitchFactor / std::sqrt(std::max(achieved, 1e-6f));

        previous_indices = level.indices.size();
        chain.levels.push_back(std::move(level));
        chain.screen_thresholds.push_back(threshold);
    }

    return chain;
}

} // namespace stratum::osm::road
