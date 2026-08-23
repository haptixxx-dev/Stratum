/**
 * @file p7_fixtures.hpp
 * @brief Shared helpers for the P7 game-ready-output suites
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The four P7 suites -- mesh optimisation, collision derivation, export, and the
 * integration sweep -- ask a different kind of question from every phase before
 * them. P2 through P6 asked "is this geometry in the right place". P7 asks "is
 * this geometry still the same geometry after it has been rewritten", and the
 * only way to answer that is to compare a mesh against itself across a
 * transformation that is allowed to renumber every vertex and reorder every
 * triangle.
 *
 * So the helpers here are:
 *
 * 1. **Content-addressed triangles.** weld_vertices() renumbers vertices,
 *    optimize_mesh() reorders triangles and renumbers vertices again, and
 *    export chunks copy vertices per file. Index-based comparison is therefore
 *    meaningless throughout the phase. triangle_multiset() reduces a mesh to a
 *    sorted multiset of (material, canonical position triple), which survives all
 *    three and still distinguishes a flipped winding.
 * 2. **Submesh tiling.** Every entry point in this phase promises that the
 *    SubMesh ranges still tile the index buffer with no gap and no overlap.
 *    submeshes_tile_exactly() is that promise, checked the same way everywhere.
 * 3. **Boundary edges.** Both the LOD border-lock contract and the collision
 *    "no hole" contract are statements about open boundary edges, computed per
 *    submesh, because build_lod_chain() simplifies per submesh and a material
 *    boundary is an open border of its own range.
 * 4. **Synthetic geometry with interior vertices.** A corridor strip is two
 *    vertex columns wide, so every one of its vertices is on an open boundary and
 *    a border-locked simplifier can do nothing with it. make_slab_mesh() is a
 *    welded grid with real interior vertices and three material bands, which is
 *    what the LOD contract can actually be exercised against.
 * 5. **A tolerant OBJ reader.** The export suite has to read back what it wrote.
 *
 * ### Coordinates
 *
 * World space, Y up, throughout: local 2D `(x, y)` maps to `vec3(x, height, -y)`.
 * junction::world_to_local() is the inverse and is used wherever a plan-view
 * predicate from tests/road/junction_fixtures.hpp is applied.
 */

#pragma once

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/corridor.hpp"
#include "osm/road/road_profile.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace stratum::test::p7 {

namespace junction = ::stratum::test::junction;

// ============================================================================
// Constants
// ============================================================================

/// Quantisation step used to compare positions across a rewrite, metres
inline constexpr double kPositionQuantum = 1e-4;

/// Plan-view tolerance when asking whether a point sits on a ring, metres
inline constexpr double kPlanEps = 1e-3;

// ============================================================================
// Position and triangle keys
// ============================================================================

/**
 * @brief A vertex position reduced to an exact, orderable key
 *
 * Quantised rather than bit-exact. weld_vertices() keeps the LOWEST-indexed
 * member of a weld group, so a surviving position is one of the input positions
 * unchanged and a bit-exact key would work for the weld; but the export path
 * writes decimal text and reads it back, and that round trip is not bit-exact.
 * One quantum is WeldConfig::position_epsilon, so two positions that share a key
 * are two positions the pipeline itself considers the same point.
 */
struct PosKey {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    bool operator<(const PosKey& o) const {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
    bool operator==(const PosKey& o) const { return x == o.x && y == o.y && z == o.z; }
    bool operator!=(const PosKey& o) const { return !(*this == o); }
};

/// Quantise one coordinate onto the position grid
inline int64_t quantise(double v) {
    return static_cast<int64_t>(std::llround(v / kPositionQuantum));
}

/// Quantise a world position onto the position grid
inline PosKey pos_key(const glm::vec3& p) {
    return PosKey{ quantise(static_cast<double>(p.x)),
                   quantise(static_cast<double>(p.y)),
                   quantise(static_cast<double>(p.z)) };
}

/// Quantise a double-precision world position onto the position grid
inline PosKey pos_key(const glm::dvec3& p) {
    return PosKey{ quantise(p.x), quantise(p.y), quantise(p.z) };
}

/**
 * @brief One triangle reduced to its material and its three positions
 *
 * The triple is rotated so the smallest key comes first, which is a CYCLIC
 * rotation and therefore preserves winding: a triangle whose winding was flipped
 * produces a different key, which is the point. Reflection would not, so the
 * triple is never sorted outright.
 */
struct TriKey {
    PosKey a;
    PosKey b;
    PosKey c;
    uint8_t material = 0;

    bool operator<(const TriKey& o) const {
        if (material != o.material) return material < o.material;
        if (!(a == o.a)) return a < o.a;
        if (!(b == o.b)) return b < o.b;
        return c < o.c;
    }
    bool operator==(const TriKey& o) const {
        return material == o.material && a == o.a && b == o.b && c == o.c;
    }
};

/// Rotate a triple so the smallest key leads, preserving the cyclic order
inline TriKey make_tri_key(PosKey a, PosKey b, PosKey c, MaterialId material) {
    TriKey k;
    k.material = static_cast<uint8_t>(material);
    if (b < a && b < c) {
        k.a = b; k.b = c; k.c = a;
    } else if (c < a && c < b) {
        k.a = c; k.b = a; k.c = b;
    } else {
        k.a = a; k.b = b; k.c = c;
    }
    return k;
}

/// Per-triangle material of a mesh, resolving the implicit whole-mesh range
inline std::vector<MaterialId> triangle_materials(const Mesh& mesh) {
    std::vector<MaterialId> out(mesh.indices.size() / 3u, MaterialId::Default);
    for (const SubMesh& sub : mesh.effective_submeshes()) {
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset) + sub.index_count) / 3u;
        for (size_t t = first; t < last && t < out.size(); ++t) out[t] = sub.material;
    }
    return out;
}

/**
 * @brief Every triangle of a mesh as a sorted multiset of content keys
 *
 * The comparison instrument of the whole phase. Two meshes with equal multisets
 * hold the same triangles, with the same windings, attributed to the same
 * materials, however their vertices are numbered and however their index buffers
 * are ordered.
 *
 * @param mesh Mesh to reduce
 * @return Sorted keys, one per triangle with in-range indices
 */
inline std::vector<TriKey> triangle_multiset(const Mesh& mesh) {
    const std::vector<MaterialId> per_triangle = triangle_materials(mesh);
    std::vector<TriKey> out;
    out.reserve(per_triangle.size());
    for (size_t t = 0; t < per_triangle.size(); ++t) {
        const uint32_t i0 = mesh.indices[t * 3 + 0];
        const uint32_t i1 = mesh.indices[t * 3 + 1];
        const uint32_t i2 = mesh.indices[t * 3 + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() ||
            i2 >= mesh.vertices.size()) {
            continue;
        }
        out.push_back(make_tri_key(pos_key(mesh.vertices[i0].position),
                                   pos_key(mesh.vertices[i1].position),
                                   pos_key(mesh.vertices[i2].position),
                                   per_triangle[t]));
    }
    std::sort(out.begin(), out.end());
    return out;
}

// ============================================================================
// Structural predicates
// ============================================================================

/**
 * @brief Submesh ranges tile the index buffer exactly once
 *
 * @param mesh   Mesh to check
 * @param reason Receives a description of the first violation found
 * @return True when the ranges are contiguous from 0, each a whole number of
 *         triangles, and together exactly as long as the index buffer
 */
inline bool submeshes_tile_exactly(const Mesh& mesh, std::string& reason) {
    if (mesh.submeshes.empty()) {
        if (mesh.indices.empty()) return true;
        return true;    // one implicit whole-mesh range
    }
    uint32_t expected = 0;
    for (const SubMesh& sub : mesh.submeshes) {
        if (sub.index_offset != expected) {
            reason = "range starts at " + std::to_string(sub.index_offset) + ", expected " +
                     std::to_string(expected);
            return false;
        }
        if ((sub.index_count % 3u) != 0u) {
            reason = "range holds " + std::to_string(sub.index_count) +
                     " indices, not a whole number of triangles";
            return false;
        }
        expected += sub.index_count;
    }
    if (expected != mesh.indices.size()) {
        reason = "ranges cover " + std::to_string(expected) + " of " +
                 std::to_string(mesh.indices.size()) + " indices";
        return false;
    }
    return true;
}

/// Every index is in range and the index count is a whole number of triangles
inline bool indices_are_sane(const Mesh& mesh) {
    if ((mesh.indices.size() % 3u) != 0u) return false;
    for (uint32_t i : mesh.indices) {
        if (i >= mesh.vertices.size()) return false;
    }
    return true;
}

/// Every vertex position, normal and UV is finite
inline bool mesh_is_finite(const Mesh& mesh) {
    for (const Vertex& v : mesh.vertices) {
        if (!junction::is_finite(v.position)) return false;
        if (!junction::is_finite(v.normal)) return false;
        if (!std::isfinite(v.uv.x) || !std::isfinite(v.uv.y)) return false;
    }
    return true;
}

/// Triangles of a mesh, counted
inline size_t triangle_count(const Mesh& mesh) { return mesh.indices.size() / 3u; }

/// The distinct materials a mesh's ranges attribute at least one triangle to
inline std::set<MaterialId> materials_of(const Mesh& mesh) {
    std::set<MaterialId> out;
    for (const SubMesh& sub : mesh.effective_submeshes()) {
        if (sub.index_count >= 3u) out.insert(sub.material);
    }
    return out;
}

/// Triangles attributed to one material
inline size_t triangles_with_material(const Mesh& mesh, MaterialId material) {
    size_t n = 0;
    for (MaterialId m : triangle_materials(mesh)) {
        if (m == material) ++n;
    }
    return n;
}

// ============================================================================
// Boundary edges
// ============================================================================

/// An undirected edge between two quantised positions
struct EdgeKey {
    PosKey a;
    PosKey b;

    bool operator<(const EdgeKey& o) const {
        if (!(a == o.a)) return a < o.a;
        return b < o.b;
    }
    bool operator==(const EdgeKey& o) const { return a == o.a && b == o.b; }
};

/// Order the two ends so the edge is undirected
inline EdgeKey make_edge_key(const PosKey& p, const PosKey& q) {
    return (q < p) ? EdgeKey{ q, p } : EdgeKey{ p, q };
}

/**
 * @brief Edges used by exactly one triangle, computed over a chosen triangle set
 *
 * Keyed on POSITION, not on vertex index, so an unwelded mesh whose two triangles
 * meet at coincident-but-distinct vertices still reports that edge as interior.
 * That is the right relation here: the question both callers ask is geometric --
 * "is there a hole in the surface" -- not "did the producer share an index".
 *
 * @param mesh      Mesh to walk
 * @param predicate Called with the triangle index; only triangles it accepts
 *                  contribute
 * @return The open boundary edges
 */
template <typename Predicate>
inline std::set<EdgeKey> boundary_edges_if(const Mesh& mesh, Predicate predicate) {
    std::map<EdgeKey, int> use;
    const size_t tris = triangle_count(mesh);
    for (size_t t = 0; t < tris; ++t) {
        if (!predicate(t)) continue;
        const uint32_t i0 = mesh.indices[t * 3 + 0];
        const uint32_t i1 = mesh.indices[t * 3 + 1];
        const uint32_t i2 = mesh.indices[t * 3 + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() ||
            i2 >= mesh.vertices.size()) {
            continue;
        }
        const PosKey p0 = pos_key(mesh.vertices[i0].position);
        const PosKey p1 = pos_key(mesh.vertices[i1].position);
        const PosKey p2 = pos_key(mesh.vertices[i2].position);
        if (p0 == p1 || p1 == p2 || p0 == p2) continue;   // degenerate, no real edge
        ++use[make_edge_key(p0, p1)];
        ++use[make_edge_key(p1, p2)];
        ++use[make_edge_key(p2, p0)];
    }
    std::set<EdgeKey> out;
    for (const auto& entry : use) {
        if (entry.second == 1) out.insert(entry.first);
    }
    return out;
}

/// Open boundary edges over the whole mesh
inline std::set<EdgeKey> boundary_edges(const Mesh& mesh) {
    return boundary_edges_if(mesh, [](size_t) { return true; });
}

/**
 * @brief Positions sitting on an open boundary of their OWN submesh
 *
 * Per submesh, because build_lod_chain() simplifies per submesh: a material
 * boundary is an open border of its range even though the whole mesh is closed
 * across it, and LodConfig::lock_borders pins exactly these.
 *
 * @param mesh Mesh to walk
 * @return Every quantised position on a per-range open boundary
 */
inline std::set<PosKey> border_positions(const Mesh& mesh) {
    std::set<PosKey> out;
    const std::vector<MaterialId> per_triangle = triangle_materials(mesh);
    std::set<MaterialId> present;
    for (MaterialId m : per_triangle) present.insert(m);

    for (MaterialId material : present) {
        const std::set<EdgeKey> edges = boundary_edges_if(mesh, [&](size_t t) {
            return per_triangle[t] == material;
        });
        for (const EdgeKey& e : edges) {
            out.insert(e.a);
            out.insert(e.b);
        }
    }
    return out;
}

// ============================================================================
// Synthetic geometry
// ============================================================================

/**
 * @brief A residential kerbed cross-section, built by hand
 *
 * build_profile() produces the same shape from tags, but a hand-built profile is
 * what the crease and step-height assertions can quote exact numbers against: the
 * kerb is 0.15 m tall and its face leans 0.02 m, so the face's geometric normal
 * has |y| = 0.02 / sqrt(0.02^2 + 0.15^2) = 0.132, which is below
 * kVerticalNormalY, and the step it becomes once the face is deleted is exactly
 * 0.15 m, which is below CollisionConfig::max_step_height. Both numbers appear in
 * the assertions rather than being read back out of the mesh.
 *
 * Left to right: sidewalk, kerb top, kerb face, gutter, two lanes, gutter, kerb
 * face, kerb top, sidewalk. Adjacent strips agree at their shared boundary, which
 * RoadProfile::is_valid() requires.
 *
 * @return The profile
 */
inline osm::road::RoadProfile kerbed_profile() {
    using osm::road::Strip;
    using osm::road::StripKind;

    const float kerb = 0.15f;
    const float batter = 0.02f;

    osm::road::RoadProfile profile;
    profile.strips = {
        Strip{ 2.0f,    kerb, kerb, MaterialId::Sidewalk, StripKind::Sidewalk },
        Strip{ 0.15f,   kerb, kerb, MaterialId::Curb,     StripKind::CurbTop  },
        Strip{ batter,  kerb, 0.0f, MaterialId::Curb,     StripKind::CurbFace },
        Strip{ 0.3f,    0.0f, 0.0f, MaterialId::Asphalt,  StripKind::Gutter   },
        Strip{ 3.5f,    0.0f, 0.0f, MaterialId::Asphalt,  StripKind::Lane     },
        Strip{ 3.5f,    0.0f, 0.0f, MaterialId::Asphalt,  StripKind::Lane     },
        Strip{ 0.3f,    0.0f, 0.0f, MaterialId::Asphalt,  StripKind::Gutter   },
        Strip{ batter,  0.0f, kerb, MaterialId::Curb,     StripKind::CurbFace },
        Strip{ 0.15f,   kerb, kerb, MaterialId::Curb,     StripKind::CurbTop  },
        Strip{ 2.0f,    kerb, kerb, MaterialId::Sidewalk, StripKind::Sidewalk },
    };
    return profile;
}

/**
 * @brief A straight centerline running east, resampled at the shipping spacing
 *
 * Smoothing is off: a straight line has nothing to smooth, and leaving it on
 * would make the station count depend on the spline fitter rather than on
 * ResampleConfig::max_spacing.
 *
 * @param length_m Length of the run, metres
 * @return The stations
 */
inline osm::road::Centerline straight_centerline(double length_m = 200.0) {
    osm::road::ResampleConfig cfg;
    cfg.smooth = false;
    return osm::road::build_centerline({ glm::dvec2{ 0.0, 0.0 }, glm::dvec2{ length_m, 0.0 } },
                                       cfg);
}

/**
 * @brief A kerbed corridor: the real output of the real extruder
 *
 * Used wherever an assertion has to hold against production geometry rather than
 * against a shape invented for the test. Its kerb creases are the ones
 * weld_vertices() must not smooth, and its kerb faces are the ones
 * build_collision_mesh() must turn into steps.
 *
 * @param length_m Length of the run, metres
 * @return The swept corridor
 */
inline osm::road::Corridor kerbed_corridor(double length_m = 200.0) {
    osm::road::CorridorConfig cfg;
    cfg.base_height = 0.0f;
    return osm::road::build_corridor(straight_centerline(length_m), kerbed_profile(), cfg);
}

/**
 * @brief A welded grid slab carrying three material bands
 *
 * The LOD contract cannot be exercised against a corridor. Every corridor strip
 * is two vertex columns wide, so every one of its vertices sits on an open
 * boundary of its own strip, and a border-locked simplifier is correctly unable
 * to remove any of them. That is a true property of the extruder, not a defect of
 * the simplifier, and it means a corridor is the wrong instrument for asking
 * whether simplification preserves what it promises.
 *
 * This slab is the right one. Vertices are shared across the whole grid, so each
 * band has real interior vertices to collapse, while the band boundaries and the
 * outer rim are open borders that LodConfig::lock_borders must pin. The three
 * bands are Asphalt, Curb and Sidewalk, so "a level silently dropped the Curb
 * material" is a question that can be asked.
 *
 * Geometry: @p columns_per_band * 3 quads across, @p rows quads along, one metre
 * per quad, flat at y = 0, normals +Y, UVs in metres. Winding is
 * counter-clockwise seen from above, matching the corridor extruder.
 *
 * @param columns_per_band Quads across each of the three bands
 * @param rows             Quads along the slab
 * @return The slab, with one SubMesh range per band in ascending MaterialId order
 */
inline Mesh make_slab_mesh(int columns_per_band = 8, int rows = 40) {
    const MaterialId bands[3] = { MaterialId::Asphalt, MaterialId::Curb, MaterialId::Sidewalk };
    const int columns = columns_per_band * 3;

    Mesh mesh;
    mesh.vertices.reserve(static_cast<size_t>(columns + 1) * static_cast<size_t>(rows + 1));
    for (int r = 0; r <= rows; ++r) {
        for (int c = 0; c <= columns; ++c) {
            Vertex v{};
            v.position = glm::vec3(static_cast<float>(c), 0.0f, -static_cast<float>(r));
            v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            v.uv = glm::vec2(static_cast<float>(c), static_cast<float>(r));
            v.color = glm::vec4(1.0f);
            v.tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
            mesh.vertices.push_back(v);
        }
    }

    const auto index_of = [&](int c, int r) {
        return static_cast<uint32_t>(r * (columns + 1) + c);
    };

    // One range per band, emitted in band order so the ranges are already
    // contiguous; sort_submeshes_by_material() then puts them in MaterialId order.
    for (int band = 0; band < 3; ++band) {
        const uint32_t start = static_cast<uint32_t>(mesh.indices.size());
        for (int r = 0; r < rows; ++r) {
            for (int k = 0; k < columns_per_band; ++k) {
                const int c = band * columns_per_band + k;
                const uint32_t v00 = index_of(c, r);
                const uint32_t v10 = index_of(c + 1, r);
                const uint32_t v01 = index_of(c, r + 1);
                const uint32_t v11 = index_of(c + 1, r + 1);
                mesh.indices.push_back(v00);
                mesh.indices.push_back(v10);
                mesh.indices.push_back(v11);
                mesh.indices.push_back(v00);
                mesh.indices.push_back(v11);
                mesh.indices.push_back(v01);
            }
        }
        const uint32_t added = static_cast<uint32_t>(mesh.indices.size()) - start;
        mesh.submeshes.push_back(SubMesh{ start, added, bands[band] });
    }

    mesh.sort_submeshes_by_material();
    mesh.compute_bounds();
    return mesh;
}

/**
 * @brief One flat quad, the smallest mesh that is still a mesh
 *
 * @param material Material to tag the range with
 * @return A two-triangle mesh with one SubMesh range
 */
inline Mesh make_quad_mesh(MaterialId material = MaterialId::Asphalt) {
    Mesh mesh;
    const glm::vec3 corners[4] = {
        { 0.0f, 0.0f,  0.0f }, { 1.0f, 0.0f,  0.0f },
        { 1.0f, 0.0f, -1.0f }, { 0.0f, 0.0f, -1.0f },
    };
    for (int i = 0; i < 4; ++i) {
        Vertex v{};
        v.position = corners[i];
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v.uv = glm::vec2(corners[i].x, -corners[i].z);
        mesh.vertices.push_back(v);
    }
    mesh.indices = { 0, 1, 2, 0, 2, 3 };
    mesh.submeshes.push_back(SubMesh{ 0u, 6u, material });
    mesh.compute_bounds();
    return mesh;
}

// ============================================================================
// Output directories
// ============================================================================

/**
 * @brief A directory under the build tree for one suite's output
 *
 * STRATUM_TEST_DUMP_DIR is build output and is regenerated on every run, so a
 * stale file can never be mistaken for a committed fixture. The directory is
 * cleared before it is handed back, which is what lets a test count the files it
 * finds there.
 *
 * @param name Subdirectory name
 * @return The created, empty directory
 */
inline std::filesystem::path scratch_dir(const std::string& name) {
    const std::filesystem::path dir = std::filesystem::path(STRATUM_TEST_DUMP_DIR) / "p7" / name;
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir, ec);
    return dir;
}

// ============================================================================
// OBJ reader
// ============================================================================

/// One triangle read back from an OBJ file, with the `usemtl` group it fell under
struct ObjFace {
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t c = 0;
    std::string material;
};

/// What read_obj() recovered
struct ObjFile {
    std::vector<glm::dvec3> positions;      ///< `v` records, in file order
    std::vector<ObjFace> faces;             ///< triangles, polygons fanned
    std::vector<std::string> groups;        ///< `usemtl` names, first appearance order
    std::vector<std::string> mtllibs;       ///< `mtllib` references, in file order
    bool ok = false;                        ///< the file opened and parsed
};

/**
 * @brief Read an OBJ file back
 *
 * Tolerant on purpose: the exporter is another agent's code and the test must
 * fail on what it wrote, not on how it spelled it. Handles `v`, `usemtl`,
 * `mtllib` and `f` with any of the `a`, `a/b`, `a//c` and `a/b/c` vertex forms,
 * positive or negative (relative) indices, and polygons of more than three
 * corners, which are fanned from the first corner.
 *
 * Everything else -- `vn`, `vt`, `o`, `g`, `s`, comments -- is skipped, because
 * none of it changes the two questions the export suite asks: how many triangles
 * came back, and which material group each one landed in.
 *
 * @param path File to read
 * @return The contents; `ok` false when the file could not be opened
 */
inline ObjFile read_obj(const std::filesystem::path& path) {
    ObjFile out;
    std::ifstream in(path);
    if (!in) return out;

    std::string current_material = "";
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;

        if (tag == "v") {
            double x = 0.0, y = 0.0, z = 0.0;
            ls >> x >> y >> z;
            out.positions.emplace_back(x, y, z);
        } else if (tag == "usemtl") {
            std::string name;
            ls >> name;
            current_material = name;
            if (std::find(out.groups.begin(), out.groups.end(), name) == out.groups.end()) {
                out.groups.push_back(name);
            }
        } else if (tag == "mtllib") {
            std::string name;
            ls >> name;
            out.mtllibs.push_back(name);
        } else if (tag == "f") {
            std::vector<uint32_t> corners;
            std::string token;
            while (ls >> token) {
                const size_t slash = token.find('/');
                const std::string first = (slash == std::string::npos) ? token
                                                                       : token.substr(0, slash);
                if (first.empty()) continue;
                long value = 0;
                try {
                    value = std::stol(first);
                } catch (...) {
                    continue;
                }
                if (value > 0) {
                    corners.push_back(static_cast<uint32_t>(value - 1));
                } else if (value < 0) {
                    const long resolved = static_cast<long>(out.positions.size()) + value;
                    if (resolved >= 0) corners.push_back(static_cast<uint32_t>(resolved));
                }
            }
            for (size_t k = 2; k < corners.size(); ++k) {
                out.faces.push_back(ObjFace{ corners[0], corners[k - 1], corners[k],
                                             current_material });
            }
        }
    }
    out.ok = true;
    return out;
}

/// Triangles of an OBJ file whose corner indices are all in range
inline size_t valid_face_count(const ObjFile& obj) {
    size_t n = 0;
    for (const ObjFace& f : obj.faces) {
        if (f.a < obj.positions.size() && f.b < obj.positions.size() &&
            f.c < obj.positions.size()) {
            ++n;
        }
    }
    return n;
}

/// Mean of an OBJ face's three corner positions
inline glm::dvec3 face_centroid(const ObjFile& obj, const ObjFace& f) {
    return (obj.positions[f.a] + obj.positions[f.b] + obj.positions[f.c]) / 3.0;
}

} // namespace stratum::test::p7
