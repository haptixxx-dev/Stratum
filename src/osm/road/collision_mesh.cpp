/**
 * @file collision_mesh.cpp
 * @brief Implementation of the render-mesh-to-collision-surface derivation
 *
 * The header says what comes out. This file is about the two decisions that are
 * not obvious from the outside, because both are places where the naive
 * implementation produces a collision surface that is worse than none.
 *
 * ### 1. How tall a vertical face is
 *
 * The rule is "delete a vertical face that is a step, keep one that is a wall",
 * and the test is the face's height. The question is height of WHAT.
 *
 * - Per TRIANGLE is wrong. A bridge parapet is a metre and a half tall and is
 *   triangulated into a strip of quads; every one of those quads is short, so a
 *   per-triangle test deletes the parapet a slice at a time and vehicles drive
 *   off the bridge.
 * - Per connected PATCH is also wrong, and this is the trap the obvious fix
 *   walks into. A kerb runs continuously for the whole length of a road. Down a
 *   hill that single connected patch spans forty metres of fall, so its overall
 *   vertical extent is forty metres, and a patch test keeps every kerb on every
 *   slope in the network as a "wall".
 *
 * So height is measured LOCALLY: for each vertex of a patch, over the other
 * vertices of the same patch within kPatchWindowRadius of it in PLAN, and a
 * triangle's step height is the largest of the three windows at its own
 * vertices. A kerb on a slope is 150 mm tall in every window regardless of the
 * slope. A parapet is 1.5 m tall in the window at any point along it, whatever
 * it is cut into. That is the measure the classification actually wants, and
 * neither degenerate case survives it.
 *
 * The patch bounds the window; it is not the unit of the decision. Taking the
 * maximum over a whole patch would let one tall member promote everything
 * connected to it, and connectivity here is by shared position alone: a bridge
 * deck's end cap meets the kerb face at the first station of the edge, so a
 * patch-wide test turns every kerb on that bridge into a wall.
 *
 * ### 2. What replaces a deleted face
 *
 * A kerb face in the default profile is 150 mm tall and, because
 * ProfileConfig::curb_face_batter leans it outward, 20 mm wide in plan. Deleting
 * it leaves a 20 mm slit in the surface between the gutter and the kerb top:
 * nothing walks through it, but a downward wheel raycast can pass straight down
 * it and report no ground.
 *
 * So each deleted triangle is replaced by its own plan-view footprint, laid flat
 * at the height of the LOWER edge of the patch beneath it -- the same local
 * window, this time reporting its minimum rather than its extent. The result is
 * a horizontal sliver that closes the slit at road level and follows the road's
 * longitudinal grade, and the kerb top is left sitting one step above it. A kerb
 * therefore becomes a step of at most CollisionConfig::max_step_height, which is
 * what every character controller already handles, rather than a wall it slides
 * along or a hole it falls through.
 *
 * A face with no batter at all projects to zero plan area, so it produces no
 * bridge -- correctly, because a face with no plan width leaves no plan gap.
 *
 * ### The guard
 *
 * Simplification is the one step here that can invent a hole, and a hole in a
 * collision surface drops things through the world. The output is therefore
 * measured before and after: total plan area, and the number of open boundary
 * edges. A candidate that lost plan area or grew boundary is rejected and the
 * unsimplified surface is kept. Slower geometry is a cost; a hole is a bug
 * report from a player.
 *
 * The guard is blind in exactly one direction, and it is structural rather than
 * a tolerance that could be tightened: a WALL covers no plan. A tunnel headwall
 * that is deleted in full moves the plan total by nothing, so the guard sees a
 * clean simplification of the ground around it and accepts. That is the precise
 * deletion the whole vertical-face classification exists to prevent, so the
 * walls are held out of the simplification instead of being measured through it
 * -- split off, kept verbatim, and welded back on afterwards.
 */

#include "osm/road/collision_mesh.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace stratum::osm::road {
namespace {

// ============================================================================
// Constants
// ============================================================================

/// Number of material slots, for bounds-checking a SubMesh's material
constexpr size_t kMaterialCount = static_cast<size_t>(MaterialId::Count);

/// Sentinel for "this vertex is referenced by no surviving triangle"
constexpr uint32_t kUnusedVertex = std::numeric_limits<uint32_t>::max();

// kPatchWindowRadius, the plan-view radius the local patch height is measured
// over, is declared in the header so the tests can assert against it.

/// Spatial hash cell for patch connectivity, metres. Ten times the weld epsilon.
constexpr float kPatchWeldCell = 1e-3f;

/// Plan-view area below which a bridge triangle closes no gap and is skipped
constexpr double kMinBridgeArea = 1e-9;

/// Fraction of the pre-simplification plan area the simplified output must retain
constexpr double kMinAreaRetention = 0.95;

/// Position tolerance of the collision weld, metres. Coarse, deliberately.
constexpr float kWeldPositionEpsilon = 1e-3f;

/// Smallest simplification ratio honoured; below this the request is clamped
constexpr float kMinSimplifyRatio = 0.01f;

// ============================================================================
// Small helpers
// ============================================================================

/// Mix one integer into a running 64-bit hash
[[nodiscard]] uint64_t hash_mix(uint64_t h, int64_t value) {
    h ^= static_cast<uint64_t>(value) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    return h;
}

/// Quantised position key, so two builders arriving at the same point agree
[[nodiscard]] uint64_t position_key(const glm::vec3& p) {
    uint64_t h = 0xcbf29ce484222325ull;
    h = hash_mix(h, std::llround(static_cast<double>(p.x) / kPatchWeldCell));
    h = hash_mix(h, std::llround(static_cast<double>(p.y) / kPatchWeldCell));
    h = hash_mix(h, std::llround(static_cast<double>(p.z) / kPatchWeldCell));
    return h;
}

/// Key of a plan-view grid cell
[[nodiscard]] uint64_t cell_key(int64_t cx, int64_t cz) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32)
         ^ static_cast<uint64_t>(static_cast<uint32_t>(cz));
}

/// Undirected edge key over a welded index buffer, orientation-independent
[[nodiscard]] uint64_t edge_key(uint32_t a, uint32_t b) {
    const uint32_t lo = std::min(a, b);
    const uint32_t hi = std::max(a, b);
    return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

/**
 * @brief Union-find over triangle indices
 *
 * The representative of a set is always its lowest member, so the patch
 * numbering does not depend on the order the unions were applied and the whole
 * derivation stays deterministic.
 */
class DisjointSet {
public:
    explicit DisjointSet(size_t count) : m_parent(count) {
        for (size_t i = 0; i < count; ++i) {
            m_parent[i] = static_cast<uint32_t>(i);
        }
    }

    [[nodiscard]] uint32_t find(uint32_t a) {
        while (m_parent[a] != a) {
            m_parent[a] = m_parent[m_parent[a]];
            a = m_parent[a];
        }
        return a;
    }

    void unite(uint32_t a, uint32_t b) {
        const uint32_t ra = find(a);
        const uint32_t rb = find(b);
        if (ra == rb) return;
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
 * @brief Per-triangle material slot, resolving the implicit whole-mesh range
 *
 * Triangles covered by no range keep MaterialId::Default rather than being
 * dropped, which matches Mesh::sort_submeshes_by_material().
 */
[[nodiscard]] std::vector<uint8_t> triangle_materials(const Mesh& mesh) {
    const size_t tri_count = mesh.indices.size() / 3u;
    std::vector<uint8_t> out(tri_count, static_cast<uint8_t>(MaterialId::Default));

    for (const SubMesh& sub : mesh.effective_submeshes()) {
        const uint8_t slot = static_cast<uint8_t>(sub.material);
        if (static_cast<size_t>(slot) >= kMaterialCount) {
            continue;
        }
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset)
                             + static_cast<size_t>(sub.index_count)) / 3u;
        for (size_t t = first; t < last && t < tri_count; ++t) {
            out[t] = slot;
        }
    }
    return out;
}

/// Surface measurements the hole guard compares before and after simplification
struct SurfaceStats {
    double plan_area = 0.0;     ///< Sum of the plan-view areas of every triangle, m^2
    size_t boundary_edges = 0;  ///< Edges used by exactly one triangle
};

/**
 * @brief Measure plan coverage and open boundary
 *
 * Plan area, not surface area, because that is what a downward raycast sees: a
 * hole in the surface is a hole in the plan coverage, whereas a vertical face
 * carries surface area and covers nothing.
 */
[[nodiscard]] SurfaceStats measure_surface(const Mesh& mesh) {
    SurfaceStats stats;
    std::unordered_map<uint64_t, uint32_t> edge_use;
    edge_use.reserve(mesh.indices.size());

    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const uint32_t i0 = mesh.indices[i];
        const uint32_t i1 = mesh.indices[i + 1];
        const uint32_t i2 = mesh.indices[i + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size()
            || i2 >= mesh.vertices.size()) {
            continue;
        }

        const glm::vec3& a = mesh.vertices[i0].position;
        const glm::vec3& b = mesh.vertices[i1].position;
        const glm::vec3& c = mesh.vertices[i2].position;

        // Twice the signed plan area of the triangle projected onto XZ.
        const double cross = static_cast<double>(b.x - a.x) * static_cast<double>(c.z - a.z)
                           - static_cast<double>(c.x - a.x) * static_cast<double>(b.z - a.z);
        stats.plan_area += std::fabs(cross) * 0.5;

        ++edge_use[edge_key(i0, i1)];
        ++edge_use[edge_key(i1, i2)];
        ++edge_use[edge_key(i2, i0)];
    }

    for (const auto& [key, uses] : edge_use) {
        (void)key;
        if (uses == 1u) {
            ++stats.boundary_edges;
        }
    }
    return stats;
}

/**
 * @brief Absolute Y of a triangle's geometric normal, 1.0 for a degenerate one
 *
 * The same measure step 1 classifies the render mesh with, re-applied to the
 * welded output, where the original triangle identities no longer exist.
 * A degenerate triangle has no orientation; reporting it as flat keeps it out of
 * the wall set, where an unorientable triangle would be pinned forever.
 */
[[nodiscard]] float triangle_normal_y(const Mesh& mesh, size_t t) {
    const size_t base = t * 3u;
    if (base + 2u >= mesh.indices.size()) {
        return 1.0f;
    }
    const uint32_t i0 = mesh.indices[base + 0u];
    const uint32_t i1 = mesh.indices[base + 1u];
    const uint32_t i2 = mesh.indices[base + 2u];
    if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size()) {
        return 1.0f;
    }
    const glm::vec3& a = mesh.vertices[i0].position;
    const glm::vec3& b = mesh.vertices[i1].position;
    const glm::vec3& c = mesh.vertices[i2].position;
    const glm::vec3 face = glm::cross(b - a, c - a);
    const float len = glm::length(face);
    if (!(len > 0.0f) || !std::isfinite(len)) {
        return 1.0f;
    }
    return std::fabs(face.y / len);
}

/**
 * @brief Copy the named triangles into a mesh of their own, compacting vertices
 *
 * Vertices are emitted in ascending ORIGINAL index order, so the split is as
 * deterministic as the compaction in step 3.
 */
[[nodiscard]] Mesh extract_triangles(const Mesh& src, const std::vector<uint32_t>& triangles) {
    Mesh out;
    if (triangles.empty()) {
        return out;
    }

    std::vector<uint32_t> remap(src.vertices.size(), kUnusedVertex);
    for (uint32_t t : triangles) {
        for (size_t k = 0; k < 3u; ++k) {
            remap[src.indices[t * 3u + k]] = 0u;
        }
    }

    size_t kept = 0;
    for (size_t v = 0; v < remap.size(); ++v) {
        if (remap[v] != kUnusedVertex) {
            remap[v] = static_cast<uint32_t>(kept++);
        }
    }

    out.vertices.reserve(kept);
    for (size_t v = 0; v < remap.size(); ++v) {
        if (remap[v] != kUnusedVertex) {
            out.vertices.push_back(src.vertices[v]);
        }
    }

    out.indices.reserve(triangles.size() * 3u);
    for (uint32_t t : triangles) {
        for (size_t k = 0; k < 3u; ++k) {
            out.indices.push_back(remap[src.indices[t * 3u + k]]);
        }
    }
    return out;
}

/**
 * @brief The weld that fuses the surviving islands into one surface
 *
 * Every tolerance is opened up: materials, shading creases and UV seams are all
 * meaningless in a physics mesh, so there is nothing left to protect and a weld
 * that respected any of them would fuse nothing.
 */
void weld_collision_surface(Mesh& mesh) {
    WeldConfig weld;
    weld.position_epsilon = kWeldPositionEpsilon;
    weld.normal_epsilon = 2.0f;    // dot >= -1: every normal matches every normal
    weld.uv_epsilon = std::numeric_limits<float>::max();
    weld.color_epsilon = 1.0f;     // at or above 1 ignores colour entirely
    weld.tangent_epsilon = 2.0f;   // at or above 2 ignores tangents and handedness
    weld.respect_material = false;
    weld_vertices(mesh, weld);
}

/**
 * @brief Simplify @p surface in place, rejecting a candidate that opened a hole
 *
 * Plan area and open boundary edge count are measured before and after. A
 * candidate that lost plan coverage or grew its boundary is dropped and the
 * unsimplified surface kept: slower collision geometry is a cost, a hole is a
 * fall through the world.
 */
void simplify_surface(Mesh& surface, const CollisionConfig& cfg) {
    if (surface.indices.size() < 6u) {
        return;
    }

    const SurfaceStats before = measure_surface(surface);

    LodConfig lod;
    lod.ratios = { std::clamp(cfg.simplify_ratio, kMinSimplifyRatio, 0.99f) };
    lod.lock_borders = true;

    const LodChain chain = build_lod_chain(surface, lod);
    if (!chain.is_valid()) {
        return;
    }

    const Mesh& candidate = chain.levels.back();
    const SurfaceStats after = measure_surface(candidate);
    const bool lost_area = after.plan_area < before.plan_area * kMinAreaRetention;
    const bool opened_up = after.boundary_edges > before.boundary_edges;

    if (lost_area || opened_up) {
        spdlog::warn("build_collision_mesh: rejected the simplified surface -- plan area "
                     "{:.1f} -> {:.1f} m2, boundary edges {} -> {}. Keeping {} triangles "
                     "unsimplified rather than shipping a hole.",
                     before.plan_area, after.plan_area, before.boundary_edges,
                     after.boundary_edges, surface.indices.size() / 3u);
        return;
    }
    surface = candidate;
}

} // namespace

// ============================================================================
// Entry point
// ============================================================================

Mesh build_collision_mesh(const Mesh& render_mesh, const CollisionConfig& cfg) {
    Mesh out;

    if (render_mesh.vertices.empty() || render_mesh.indices.size() < 3u) {
        return out;
    }

    const size_t tri_count = render_mesh.indices.size() / 3u;
    const size_t vertex_count = render_mesh.vertices.size();
    const std::vector<uint8_t> tri_material = triangle_materials(render_mesh);

    // ------------------------------------------------------------------------
    // 1. Classify. The geometric normal comes from the positions, never from the
    //    vertex normals: a welded vertex normal is an average over everything
    //    that meets there, and a face is not an average.
    // ------------------------------------------------------------------------
    std::vector<uint8_t> keep(tri_count, 1u);
    std::vector<uint8_t> vertical(tri_count, 0u);

    for (size_t t = 0; t < tri_count; ++t) {
        const uint32_t i0 = render_mesh.indices[t * 3u + 0u];
        const uint32_t i1 = render_mesh.indices[t * 3u + 1u];
        const uint32_t i2 = render_mesh.indices[t * 3u + 2u];
        if (i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count) {
            keep[t] = 0u;   // malformed input triangle; nothing to collide with
            continue;
        }

        const glm::vec3& a = render_mesh.vertices[i0].position;
        const glm::vec3& b = render_mesh.vertices[i1].position;
        const glm::vec3& c = render_mesh.vertices[i2].position;
        const glm::vec3 face = glm::cross(b - a, c - a);
        const float face_len = glm::length(face);
        if (!(face_len > 0.0f) || !std::isfinite(face_len)) {
            keep[t] = 0u;   // degenerate: no area, no orientation, no collision
            continue;
        }

        const float normal_y = std::fabs(face.y / face_len);
        vertical[t] = normal_y < kVerticalNormalY ? 1u : 0u;

        const MaterialId material = static_cast<MaterialId>(tri_material[t]);
        if (cfg.drop_markings && material == MaterialId::Markings) {
            keep[t] = 0u;
        } else if (!cfg.include_sidewalk && material == MaterialId::Sidewalk) {
            keep[t] = 0u;
        } else if (!cfg.include_curb_top && material == MaterialId::Curb && !vertical[t]) {
            keep[t] = 0u;
        }
    }

    // ------------------------------------------------------------------------
    // 2. Group the surviving vertical triangles into connected patches, then
    //    measure each patch's height in a plan-view window rather than over the
    //    whole patch. See the file comment for why the whole-patch extent is the
    //    wrong measure.
    // ------------------------------------------------------------------------
    std::vector<uint32_t> bridge_triangles;   // triangles deleted as steps
    std::unordered_map<uint32_t, float> local_min_y;   // original vertex -> lower edge height

    if (cfg.drop_vertical_faces) {
        std::vector<uint32_t> candidates;
        for (size_t t = 0; t < tri_count; ++t) {
            if (keep[t] != 0u && vertical[t] != 0u) {
                candidates.push_back(static_cast<uint32_t>(t));
            }
        }

        if (!candidates.empty()) {
            // Connectivity by quantised POSITION, not by index: the render mesh
            // is welded by the time the pipeline calls this, but a caller that
            // hands over raw builder output must still get one patch per surface
            // rather than one per triangle.
            DisjointSet patches(candidates.size());
            {
                std::unordered_map<uint64_t, uint32_t> first_at;
                first_at.reserve(candidates.size() * 3u);
                for (size_t n = 0; n < candidates.size(); ++n) {
                    const uint32_t t = candidates[n];
                    for (size_t k = 0; k < 3u; ++k) {
                        const uint32_t vi = render_mesh.indices[t * 3u + k];
                        const uint64_t key = position_key(render_mesh.vertices[vi].position);
                        const auto [it, inserted] =
                            first_at.emplace(key, static_cast<uint32_t>(n));
                        if (!inserted) {
                            patches.unite(it->second, static_cast<uint32_t>(n));
                        }
                    }
                }
            }

            // One entry per distinct vertex of the vertical set, tagged with its
            // patch, indexed by a plan-view grid.
            struct PatchVertex {
                glm::vec3 position{0.0f};
                uint32_t patch = 0u;
            };
            std::vector<PatchVertex> patch_vertices;
            std::unordered_map<uint32_t, uint32_t> vertex_entry;   // original vertex -> entry
            patch_vertices.reserve(candidates.size() * 3u);
            vertex_entry.reserve(candidates.size() * 3u);

            for (size_t n = 0; n < candidates.size(); ++n) {
                const uint32_t t = candidates[n];
                const uint32_t patch = patches.find(static_cast<uint32_t>(n));
                for (size_t k = 0; k < 3u; ++k) {
                    const uint32_t vi = render_mesh.indices[t * 3u + k];
                    const auto [it, inserted] =
                        vertex_entry.emplace(vi, static_cast<uint32_t>(patch_vertices.size()));
                    if (inserted) {
                        patch_vertices.push_back(
                            PatchVertex{ render_mesh.vertices[vi].position, patch });
                    }
                }
            }

            std::unordered_map<uint64_t, std::vector<uint32_t>> grid;
            grid.reserve(patch_vertices.size());
            const auto grid_cell = [](float v) {
                return static_cast<int64_t>(std::floor(static_cast<double>(v)
                                                       / static_cast<double>(kPatchWindowRadius)));
            };
            for (size_t e = 0; e < patch_vertices.size(); ++e) {
                const glm::vec3& p = patch_vertices[e].position;
                grid[cell_key(grid_cell(p.x), grid_cell(p.z))].push_back(static_cast<uint32_t>(e));
            }

            // Local extent and local floor, per vertex, over its own patch only.
            std::vector<float> entry_min_y(patch_vertices.size(), 0.0f);
            std::vector<float> entry_extent(patch_vertices.size(), 0.0f);
            const double radius_sq = static_cast<double>(kPatchWindowRadius)
                                   * static_cast<double>(kPatchWindowRadius);

            for (size_t e = 0; e < patch_vertices.size(); ++e) {
                const PatchVertex& self = patch_vertices[e];
                float lo = self.position.y;
                float hi = self.position.y;
                const int64_t cx = grid_cell(self.position.x);
                const int64_t cz = grid_cell(self.position.z);

                for (int64_t dx = -1; dx <= 1; ++dx) {
                    for (int64_t dz = -1; dz <= 1; ++dz) {
                        const auto it = grid.find(cell_key(cx + dx, cz + dz));
                        if (it == grid.end()) continue;
                        for (uint32_t other : it->second) {
                            const PatchVertex& o = patch_vertices[other];
                            if (o.patch != self.patch) continue;
                            const double ddx = static_cast<double>(o.position.x - self.position.x);
                            const double ddz = static_cast<double>(o.position.z - self.position.z);
                            if (ddx * ddx + ddz * ddz > radius_sq) continue;
                            lo = std::min(lo, o.position.y);
                            hi = std::max(hi, o.position.y);
                        }
                    }
                }
                entry_min_y[e] = lo;
                entry_extent[e] = hi - lo;
            }

            // The keep/delete decision is per TRIANGLE, over the local extents
            // at its own three vertices. The patch is what bounds each window to
            // one surface -- a kerb must not measure itself against the wall on
            // the far side of the road -- but it is not the unit of the
            // decision. Taking the maximum over the whole patch lets one tall
            // member promote everything connected to it: a bridge deck's end cap
            // is deck_thickness deep and shares the kerb face's vertex column at
            // the first and last station, so a single shared position would keep
            // every kerb on that bridge as a wall to slide along, for the whole
            // length of the edge, instead of as a 150 mm step.
            //
            // ALL THREE vertices have to read as a step. Deciding on one of them,
            // or on the centroid, would cut a real wall in half at the triangle
            // where its window first stops seeing the full height.
            for (size_t n = 0; n < candidates.size(); ++n) {
                const uint32_t t = candidates[n];
                float step = 0.0f;
                bool measured = true;
                for (size_t k = 0; k < 3u; ++k) {
                    const auto it = vertex_entry.find(render_mesh.indices[t * 3u + k]);
                    if (it == vertex_entry.end()) {
                        measured = false;
                        break;
                    }
                    step = std::max(step, entry_extent[it->second]);
                }
                if (!measured || step > cfg.max_step_height) {
                    continue;   // a wall, not a step: it stays
                }
                keep[t] = 0u;
                bridge_triangles.push_back(t);
            }

            for (const auto& [vi, entry] : vertex_entry) {
                local_min_y[vi] = entry_min_y[entry];
            }
        }
    }

    // ------------------------------------------------------------------------
    // 3. Compact. Vertices are emitted in ascending ORIGINAL index order so the
    //    output is byte-identical run to run.
    // ------------------------------------------------------------------------
    std::vector<uint32_t> remap(vertex_count, kUnusedVertex);
    for (size_t t = 0; t < tri_count; ++t) {
        if (keep[t] == 0u) continue;
        for (size_t k = 0; k < 3u; ++k) {
            remap[render_mesh.indices[t * 3u + k]] = 0u;
        }
    }

    size_t surviving_vertices = 0;
    for (size_t v = 0; v < vertex_count; ++v) {
        if (remap[v] != kUnusedVertex) {
            remap[v] = static_cast<uint32_t>(surviving_vertices++);
        }
    }

    out.vertices.reserve(surviving_vertices + bridge_triangles.size() * 3u);
    for (size_t v = 0; v < vertex_count; ++v) {
        if (remap[v] != kUnusedVertex) {
            out.vertices.push_back(render_mesh.vertices[v]);
        }
    }

    out.indices.reserve((tri_count - bridge_triangles.size()) * 3u
                        + bridge_triangles.size() * 3u);
    for (size_t t = 0; t < tri_count; ++t) {
        if (keep[t] == 0u) continue;
        for (size_t k = 0; k < 3u; ++k) {
            const uint32_t mapped = remap[render_mesh.indices[t * 3u + k]];
            assert(mapped != kUnusedVertex && "kept triangle references a compacted-out vertex");
            out.indices.push_back(mapped);
        }
    }

    // ------------------------------------------------------------------------
    // 4. Bridge. Each deleted step becomes its own plan footprint, laid flat at
    //    the local floor of the patch it came from, wound upward.
    // ------------------------------------------------------------------------
    size_t bridged = 0;
    for (uint32_t t : bridge_triangles) {
        glm::vec3 corner[3];
        Vertex source[3];
        bool usable = true;

        for (size_t k = 0; k < 3u; ++k) {
            const uint32_t vi = render_mesh.indices[t * 3u + k];
            source[k] = render_mesh.vertices[vi];
            const auto it = local_min_y.find(vi);
            if (it == local_min_y.end()) {
                usable = false;
                break;
            }
            corner[k] = glm::vec3(source[k].position.x, it->second, source[k].position.z);
        }
        if (!usable) continue;

        // Plan area of the projection. A face with no batter projects to a line
        // and leaves no gap to close, so it produces nothing.
        const double cross =
            static_cast<double>(corner[1].x - corner[0].x) * static_cast<double>(corner[2].z - corner[0].z)
          - static_cast<double>(corner[2].x - corner[0].x) * static_cast<double>(corner[1].z - corner[0].z);
        if (std::fabs(cross) * 0.5 < kMinBridgeArea) {
            continue;
        }

        const uint32_t base = static_cast<uint32_t>(out.vertices.size());
        for (size_t k = 0; k < 3u; ++k) {
            Vertex v = source[k];
            v.position = corner[k];
            v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            out.vertices.push_back(v);
        }

        // The geometric normal of a flat triangle in this Y-up frame is
        //
        //     n = (p1 - p0) x (p2 - p0),  n.y = u.z * v.x - u.x * v.z
        //
        // and `cross` above is `u.x * v.z - u.z * v.x`, so n.y is exactly
        // -cross. A NEGATIVE XZ cross product is therefore the upward face, and
        // a positive one has to be reversed. Winding it the other way leaves the
        // patch that closes a deleted kerb face pointing at the ground, and a
        // physics engine that culls backfaces drives straight through it.
        if (cross < 0.0) {
            out.indices.push_back(base + 0u);
            out.indices.push_back(base + 1u);
            out.indices.push_back(base + 2u);
        } else {
            out.indices.push_back(base + 0u);
            out.indices.push_back(base + 2u);
            out.indices.push_back(base + 1u);
        }
        ++bridged;
    }

    if (out.indices.empty()) {
        out.clear();
        return out;
    }

    // ------------------------------------------------------------------------
    // 5. Weld. Materials are gone, so nothing is left to protect and every
    //    tolerance is opened up: the point of this weld is to fuse the islands
    //    the deletions left into one surface with interior edges, and a weld
    //    that respects UV seams or shading creases fuses nothing.
    // ------------------------------------------------------------------------
    weld_collision_surface(out);

    // ------------------------------------------------------------------------
    // 6. Simplify, then check that the surface did not develop a hole. Collision
    //    tolerates far more shape error than render geometry does, and none of
    //    the one failure that matters.
    //
    //    The near-vertical triangles that survived step 2 are the REAL walls --
    //    a parapet, a headwall, a retaining face -- and they are held OUT of the
    //    simplification entirely, because the guard cannot see them. The guard
    //    measures PLAN area, and a wall covers no plan: meshopt_simplify() is
    //    free to collapse a 5 m tunnel headwall away without moving the plan
    //    total by a tenth of a percent, and the guard then accepts the candidate
    //    and vehicles drive through the portal. Measuring wall area as a second
    //    channel would catch it, but only by rejecting the whole simplification
    //    of every bridge and tunnel piece.
    //
    //    So the surface is SPLIT. Only the walkable part is simplified, with its
    //    borders locked -- which now includes the seam the wall stood on, so the
    //    ground does not pull away from the foot of it -- and the walls are
    //    concatenated back untouched. The re-weld afterwards fuses that seam
    //    back into shared vertices, so the output has no more open boundary than
    //    the unsimplified surface does.
    // ------------------------------------------------------------------------
    if (cfg.simplify_ratio < 1.0f && out.indices.size() >= 6u) {
        std::vector<uint32_t> wall_triangles;
        std::vector<uint32_t> walkable_triangles;
        const size_t out_triangles = out.indices.size() / 3u;
        for (size_t t = 0; t < out_triangles; ++t) {
            if (triangle_normal_y(out, t) < kVerticalNormalY) {
                wall_triangles.push_back(static_cast<uint32_t>(t));
            } else {
                walkable_triangles.push_back(static_cast<uint32_t>(t));
            }
        }

        if (wall_triangles.empty()) {
            simplify_surface(out, cfg);
        } else {
            const Mesh walls = extract_triangles(out, wall_triangles);
            Mesh walkable = extract_triangles(out, walkable_triangles);
            simplify_surface(walkable, cfg);

            walkable.append(walls, MaterialId::Default);
            out = std::move(walkable);
            weld_collision_surface(out);

            spdlog::debug("build_collision_mesh: held {} wall triangles out of the "
                          "simplification", wall_triangles.size());
        }
    }

    // ------------------------------------------------------------------------
    // 7. Collapse the material ranges and recompute bounds. An empty submeshes
    //    vector is the implicit single MaterialId::Default range.
    // ------------------------------------------------------------------------
    out.submeshes.clear();
    out.compute_bounds();

    assert(out.indices.size() % 3u == 0u && "collision mesh is not a triangle list");
    for ([[maybe_unused]] uint32_t index : out.indices) {
        assert(index < out.vertices.size() && "collision mesh index is out of range");
    }

    spdlog::debug("build_collision_mesh: {} -> {} triangles ({} steps bridged, {} vertices)",
                  tri_count, out.indices.size() / 3u, bridged, out.vertices.size());

    return out;
}

} // namespace stratum::osm::road
