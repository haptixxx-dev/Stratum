#pragma once

#include <vector>
#include <limits>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <glm/glm.hpp>

namespace stratum {

/**
 * @brief Vertex structure matching the optimized shader layout
 * 
 * Layout matches mesh.vert:
 * - location 0: position (vec3)
 * - location 1: normal (vec3)
 * - location 2: uv (vec2)
 * - location 3: color (vec4)
 * - location 4: tangent (vec4) - xyz = tangent, w = bitangent sign
 */
struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
    glm::vec4 color{1.0f};
    glm::vec4 tangent{1.0f, 0.0f, 0.0f, 1.0f};  // xyz = tangent direction, w = bitangent sign

    /**
     * @brief Baked ambient occlusion. 1 = fully open sky, 0 = fully enclosed
     *
     * MULTIPLIES the material's ao, and therefore attenuates the AMBIENT term
     * only -- see mesh_pbr.frag. That is the whole reason this is its own channel
     * instead of being folded into `color`: a vertex colour multiplies albedo, so
     * it would darken direct sunlight too, and a surface in full sun does not get
     * darker for being near a wall.
     *
     * Defaults to 1, so geometry from a builder that never bakes it is lit exactly
     * as it was before this channel existed. Baked by AmbientOcclusionBaker (see
     * core/ambient_occlusion.hpp), never authored by hand.
     */
    float ao = 1.0f;

    bool operator==(const Vertex& other) const {
        return position == other.position && 
               normal == other.normal && 
               uv == other.uv && 
               color == other.color &&
               tangent == other.tangent &&
               ao == other.ao;
    }
};

struct BoundingBox3D {
    glm::vec3 min{std::numeric_limits<float>::max()};
    glm::vec3 max{std::numeric_limits<float>::lowest()};

    void expand(const glm::vec3& point) {
        min = glm::min(min, point);
        max = glm::max(max, point);
    }

    glm::vec3 center() const { return (min + max) * 0.5f; }
    glm::vec3 extents() const { return (max - min) * 0.5f; }
    float radius() const { return glm::length(extents()); }

    bool is_valid() const { return min.x <= max.x; }
};

/**
 * @brief Material slot identifier for a range of triangles inside a Mesh
 *
 * A mesh produced by the road network builder is a single vertex/index buffer
 * carrying several surfaces at once (asphalt, curb, sidewalk, markings, ...).
 * Rather than emitting one Mesh per surface, the builder tags contiguous index
 * ranges with a MaterialId and the renderer issues one draw call per range.
 *
 * The numeric values are the sort key used by Mesh::sort_submeshes_by_material(),
 * so entries must not be reordered once meshes are cached or exported.
 *
 * @note Count is a sentinel for array sizing and iteration bounds. It is never a
 *       valid material for a SubMesh.
 */
enum class MaterialId : uint8_t {
    Default = 0,    ///< Untagged geometry; what an implicit whole-mesh range reports
    Asphalt,        ///< Carriageway running surface
    Concrete,       ///< Concrete carriageway, junction slabs, hard standing
    Curb,           ///< Curb top and curb face
    Sidewalk,       ///< Footway surface beside the carriageway
    Markings,       ///< Painted lane lines, crossings, stop lines, arrows
    Gravel,         ///< surface=gravel / compacted unpaved
    Dirt,           ///< surface=dirt / ground / unpaved track
    Grass,          ///< Verge and median planting
    BridgeDeck,     ///< Bridge deck slab, including its underside and edges
    Parapet,        ///< Bridge parapet or railing solid
    Wall,           ///< Building facade; see osm/road/road_style.hpp building_wall_material()
    Roof,           ///< Building roof surface; see osm/road/road_style.hpp building_roof_material()
    Count           ///< Sentinel: number of material slots. Not a valid material.
};

/**
 * @brief Convert a MaterialId to a stable human-readable string
 *
 * Used for logging, editor material slot labels, and exported material names.
 * The returned pointer is a string literal with static storage duration.
 *
 * @param material Material slot to name
 * @return Name of the slot, or "Unknown" for MaterialId::Count and out-of-range values
 */
[[nodiscard]] const char* material_id_name(MaterialId material);

/**
 * @brief A contiguous range of the owning Mesh's index buffer sharing one material
 *
 * Ranges are half-open over the index buffer: indices in
 * [index_offset, index_offset + index_count). Both values are index counts, not
 * triangle counts, so both are expected to be multiples of 3 for triangle lists.
 */
struct SubMesh {
    uint32_t index_offset = 0;                      ///< First index of the range
    uint32_t index_count = 0;                       ///< Number of indices in the range
    MaterialId material = MaterialId::Default;      ///< Material slot for the range

    /**
     * @brief Which variant of @ref material this range wants
     *
     * MaterialId is a coarse slot: "this is carriageway", "this is kerb". It cannot
     * say *which* carriageway, and OSM knows the difference -- surface=asphalt,
     * surface=cobblestone and surface=gravel are all a driveable running surface but
     * three different-looking ones, and the same is true of building facades and
     * amenity furniture.
     *
     * The variant is that second axis. Zero always means "the slot's default", so
     * every producer written before this field existed stays correct. Non-zero values
     * are assigned by the tag-to-style mapping in osm/road/road_style.hpp and resolved
     * against a material library by the renderer.
     *
     * Deliberately a plain integer rather than an enum: the set of variants is data,
     * derived from whatever tags an extract happens to carry, and must be extensible
     * without recompiling the mesh layer.
     *
     * @note (material, variant) together are the lookup key. A variant is only
     *       meaningful within its slot -- variant 3 of Asphalt and variant 3 of
     *       Sidewalk are unrelated.
     */
    uint16_t variant = 0;
};

/**
 * @brief The (slot, variant) pair that identifies one concrete material
 *
 * Shared vocabulary between the geometry side, which derives it from OSM tags, and
 * the renderer, which resolves it to textures and PBR parameters. Kept here rather
 * than in either of those, so neither owns it and both can depend on it.
 */
struct MaterialKey {
    MaterialId material = MaterialId::Default;
    uint16_t   variant = 0;

    bool operator==(const MaterialKey& o) const {
        return material == o.material && variant == o.variant;
    }
    bool operator!=(const MaterialKey& o) const { return !(*this == o); }

    /// Packed form, for use as a map key or a shader-visible index.
    [[nodiscard]] uint32_t packed() const {
        return (static_cast<uint32_t>(material) << 16) | variant;
    }
};

class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    BoundingBox3D bounds;

    /**
     * @brief Material ranges over the index buffer
     *
     * An empty vector means the mesh has one implicit range covering every index
     * with MaterialId::Default. Every pre-existing producer of Mesh leaves this
     * empty and keeps working unchanged; use effective_submeshes() to consume a
     * mesh without special-casing the empty state.
     */
    std::vector<SubMesh> submeshes;

    void clear() {
        vertices.clear();
        indices.clear();
        submeshes.clear();
        bounds = BoundingBox3D{};
    }

    /**
     * @brief Get the material ranges to draw, resolving the implicit whole-mesh case
     *
     * @return A copy of submeshes when it is non-empty. When submeshes is empty and
     *         the mesh has indices, a single synthesized range
     *         {0, indices.size(), MaterialId::Default}. When the mesh has no indices,
     *         an empty vector.
     */
    [[nodiscard]] std::vector<SubMesh> effective_submeshes() const;

    /**
     * @brief Append another mesh's geometry into this one under a single material
     *
     * Vertices are copied and appended; @p other's indices are copied with the
     * current vertex count added to each. The bounding box is expanded to include
     * the appended vertices.
     *
     * Submesh bookkeeping:
     * - If this mesh has indices but no submeshes, the implicit whole-mesh range is
     *   first materialized as {0, indices.size(), MaterialId::Default} so the
     *   pre-existing geometry keeps its identity.
     * - If the last submesh already uses @p material, its index_count is extended.
     * - Otherwise a new submesh is opened at the current end of the index buffer.
     *
     * @param other    Mesh to append. Its own submeshes are ignored; all of its
     *                 geometry is attributed to @p material.
     * @param material Material slot to attribute the appended geometry to.
     *
     * @note Does nothing when @p other has no vertices or no indices.
     */
    void append(const Mesh& other, MaterialId material);

    /**
     * @brief Append under a full (slot, variant) key
     *
     * Identical to the MaterialId overload except that the trailing-range merge
     * requires the variant to match too. Two ranges that share a slot but differ in
     * variant are different materials and must stay separate.
     */
    void append(const Mesh& other, MaterialKey key);

    /**
     * @brief Reorder the index buffer so each material occupies exactly one range
     *
     * Triangles are moved as whole index triples into ascending MaterialId order,
     * and duplicate ranges of the same material are merged. Vertices are not moved
     * or renumbered, so vertex indices in the caller's hands stay valid.
     *
     * After this call, submeshes holds at most one entry per distinct material,
     * sorted ascending by the underlying MaterialId value, and the ranges tile the
     * whole index buffer without gaps or overlap.
     *
     * @note No-op when submeshes is empty (the mesh is already one implicit range)
     *       or when it already holds a single entry.
     * @note Ranges whose index_offset or index_count is not a multiple of 3 are not
     *       valid triangle-list ranges; behaviour for those is undefined.
     */
    void sort_submeshes_by_material();

    bool is_valid() const {
        return !vertices.empty();
    }

    void compute_bounds() {
        bounds = BoundingBox3D{};
        for (const auto& v : vertices) {
            bounds.expand(v.position);
        }
    }

    /**
     * @brief Compute tangents for normal mapping using MikkTSpace algorithm (simplified)
     * 
     * For each triangle, computes tangent vectors based on UV gradients.
     * Assumes the mesh has valid UVs.
     */
    void compute_tangents() {
        if (indices.empty() || vertices.empty()) return;

        // Reset tangents
        for (auto& v : vertices) {
            v.tangent = glm::vec4(0.0f);
        }

        // Accumulate tangents per-triangle
        for (size_t i = 0; i < indices.size(); i += 3) {
            uint32_t i0 = indices[i];
            uint32_t i1 = indices[i + 1];
            uint32_t i2 = indices[i + 2];

            const glm::vec3& p0 = vertices[i0].position;
            const glm::vec3& p1 = vertices[i1].position;
            const glm::vec3& p2 = vertices[i2].position;

            const glm::vec2& uv0 = vertices[i0].uv;
            const glm::vec2& uv1 = vertices[i1].uv;
            const glm::vec2& uv2 = vertices[i2].uv;

            glm::vec3 edge1 = p1 - p0;
            glm::vec3 edge2 = p2 - p0;

            glm::vec2 duv1 = uv1 - uv0;
            glm::vec2 duv2 = uv2 - uv0;

            float f = 1.0f / (duv1.x * duv2.y - duv2.x * duv1.y + 1e-8f);

            glm::vec3 tangent;
            tangent.x = f * (duv2.y * edge1.x - duv1.y * edge2.x);
            tangent.y = f * (duv2.y * edge1.y - duv1.y * edge2.y);
            tangent.z = f * (duv2.y * edge1.z - duv1.y * edge2.z);

            glm::vec3 bitangent;
            bitangent.x = f * (-duv2.x * edge1.x + duv1.x * edge2.x);
            bitangent.y = f * (-duv2.x * edge1.y + duv1.x * edge2.y);
            bitangent.z = f * (-duv2.x * edge1.z + duv1.x * edge2.z);

            // Accumulate
            vertices[i0].tangent += glm::vec4(tangent, 0.0f);
            vertices[i1].tangent += glm::vec4(tangent, 0.0f);
            vertices[i2].tangent += glm::vec4(tangent, 0.0f);

            // Store bitangent sign (handedness)
            float sign = (glm::dot(glm::cross(vertices[i0].normal, tangent), bitangent) < 0.0f) ? -1.0f : 1.0f;
            vertices[i0].tangent.w = sign;
            vertices[i1].tangent.w = sign;
            vertices[i2].tangent.w = sign;
        }

        // Normalize tangents (Gram-Schmidt orthogonalize)
        //
        // A vertex that no triangle references -- one welded out of a degenerate
        // triangle the producer dropped, which the junction and dead-end builders
        // emit routinely -- still carries the zero tangent the reset above left.
        // glm::normalize of a zero vector is a division by zero, so it returns a
        // NaN that then propagates into the exported mesh and, because NaN never
        // compares equal to itself, makes two identical builds compare different.
        // The same happens when the accumulated tangent is exactly parallel to
        // the normal and orthogonalisation cancels it out.
        for (auto& v : vertices) {
            const glm::vec3 n = v.normal;
            glm::vec3 t = glm::vec3(v.tangent) - n * glm::dot(n, glm::vec3(v.tangent));

            float length_sq = glm::dot(t, t);
            if (!(length_sq > 1e-20f) || !std::isfinite(length_sq)) {
                // Any unit vector perpendicular to the normal is a valid tangent
                // for a vertex with no UV gradient to derive one from. The axis is
                // chosen by the normal alone, so it is stable run to run.
                const glm::vec3 axis = std::fabs(n.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f)
                                                             : glm::vec3(0.0f, 1.0f, 0.0f);
                t = axis - n * glm::dot(n, axis);
                length_sq = glm::dot(t, t);
                if (!(length_sq > 1e-20f) || !std::isfinite(length_sq)) {
                    t = glm::vec3(1.0f, 0.0f, 0.0f);
                    length_sq = 1.0f;
                }
            }
            t /= std::sqrt(length_sq);

            // An untouched vertex also kept w = 0, which is neither handedness.
            const float sign = v.tangent.w < 0.0f ? -1.0f : 1.0f;
            v.tangent = glm::vec4(t, sign);
        }
    }

    // Future: GPU buffer handles
};

// ============================================================================
// Out-of-line definitions
//
// mesh.hpp is header-only: there is no src/renderer/mesh.cpp and no CMakeLists
// entry for one, and the header is compiled into both stratum_core and
// stratum_editor_lib. Definitions for material_id_name(), Mesh::append(),
// Mesh::effective_submeshes(), and Mesh::sort_submeshes_by_material() therefore
// belong below this line and MUST be marked `inline`. Putting them in a new .cpp
// instead would be an ODR trap for one library and a link error for the other.
// ============================================================================

/**
 * @brief Convert a MaterialId to a stable human-readable string
 *
 * Invariant: every enumerator has exactly one name, and the name never changes,
 * because it is written into exported material slots.
 */
[[nodiscard]] inline const char* material_id_name(MaterialId material) {
    switch (material) {
        case MaterialId::Default:    return "Default";
        case MaterialId::Asphalt:    return "Asphalt";
        case MaterialId::Concrete:   return "Concrete";
        case MaterialId::Curb:       return "Curb";
        case MaterialId::Sidewalk:   return "Sidewalk";
        case MaterialId::Markings:   return "Markings";
        case MaterialId::Gravel:     return "Gravel";
        case MaterialId::Dirt:       return "Dirt";
        case MaterialId::Grass:      return "Grass";
        case MaterialId::BridgeDeck: return "BridgeDeck";
        case MaterialId::Parapet:    return "Parapet";
        case MaterialId::Wall:       return "Wall";
        case MaterialId::Roof:       return "Roof";
        case MaterialId::Count:      return "Unknown";
    }
    return "Unknown";
}

/**
 * @brief Resolve the implicit whole-mesh range
 *
 * Invariant: the returned ranges always tile [0, indices.size()) exactly, so a
 * consumer never has to know whether the producer tagged its geometry or not.
 * A mesh with no indices yields no ranges.
 */
inline std::vector<SubMesh> Mesh::effective_submeshes() const {
    if (!submeshes.empty()) {
        return submeshes;
    }
    if (indices.empty()) {
        return {};
    }
    return { SubMesh{ 0u, static_cast<uint32_t>(indices.size()), MaterialId::Default } };
}

/**
 * @brief Append another mesh's geometry under a single material
 *
 * Invariant: after the call the submesh ranges still tile the whole index buffer
 * with no gaps, and every index still refers to a vertex of this mesh. The
 * pre-existing implicit range is materialized before the first tagged append so
 * that older geometry is never silently absorbed into the new material.
 */
inline void Mesh::append(const Mesh& other, MaterialKey key) {
    if (other.vertices.empty() || other.indices.empty()) {
        return;
    }

    // Self-append would read from vectors that push_back is reallocating.
    // Copy once and recurse; this path is not expected to be hot.
    if (&other == this) {
        const Mesh copy = other;
        append(copy, key);
        return;
    }

    const uint32_t vertex_offset = static_cast<uint32_t>(vertices.size());
    const uint32_t index_start = static_cast<uint32_t>(indices.size());

    // Geometry already present but untagged: give it its implicit identity back
    // before opening a range for the appended material.
    if (submeshes.empty() && index_start > 0) {
        submeshes.push_back(SubMesh{ 0u, index_start, MaterialId::Default });
    }

    vertices.reserve(vertices.size() + other.vertices.size());
    for (const auto& v : other.vertices) {
        vertices.push_back(v);
        bounds.expand(v.position);
    }

    indices.reserve(indices.size() + other.indices.size());
    for (uint32_t idx : other.indices) {
        indices.push_back(idx + vertex_offset);
    }

    const uint32_t appended = static_cast<uint32_t>(other.indices.size());
    if (!submeshes.empty() && submeshes.back().material == key.material
        && submeshes.back().variant == key.variant) {
        submeshes.back().index_count += appended;
    } else {
        submeshes.push_back(SubMesh{ index_start, appended, key.material, key.variant });
    }
}

inline void Mesh::append(const Mesh& other, MaterialId material) {
    append(other, MaterialKey{ material, 0 });
}

/**
 * @brief Group triangles so each material occupies exactly one contiguous range
 *
 * Invariant: the multiset of triangles is unchanged, each triangle keeps its
 * original vertex order (and therefore its winding), the relative order of
 * triangles sharing a material is preserved, and the rebuilt ranges tile the
 * whole index buffer in ascending MaterialId order. Vertices are never moved,
 * so indices held elsewhere stay valid.
 */
inline void Mesh::sort_submeshes_by_material() {
    if (submeshes.size() <= 1) {
        return;
    }
    if (indices.empty() || (indices.size() % 3u) != 0u) {
        return;
    }

    const size_t triangle_count = indices.size() / 3u;

    // Per-triangle key. Gaps between ranges keep their geometry as the Default slot
    // rather than being dropped.
    //
    // Keyed on the PACKED (slot, variant) pair, not the slot alone: two ranges that
    // share a slot but differ in variant are different materials, and collapsing them
    // here would silently merge an asphalt carriageway with a cobblestone one.
    std::vector<uint32_t> triangle_key(triangle_count, MaterialKey{}.packed());
    for (const auto& sub : submeshes) {
        if (static_cast<size_t>(sub.material) >= static_cast<size_t>(MaterialId::Count)) {
            continue;
        }
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset) + sub.index_count) / 3u;
        const uint32_t key = MaterialKey{ sub.material, sub.variant }.packed();
        for (size_t t = first; t < last && t < triangle_count; ++t) {
            triangle_key[t] = key;
        }
    }

    // Compact the keys actually present. The variant axis is data-driven and sparse,
    // so a bucket per possible key is not an option; there are only ever a handful of
    // distinct materials in one mesh.
    std::vector<uint32_t> distinct(triangle_key);
    std::sort(distinct.begin(), distinct.end());
    distinct.erase(std::unique(distinct.begin(), distinct.end()), distinct.end());

    const size_t bucket_count = distinct.size();
    if (bucket_count <= 1u) {
        // One material: the ranges already tile the buffer contiguously. Rewrite the
        // range list so it is exactly one entry and leave the indices untouched.
        submeshes.clear();
        if (!indices.empty() && !distinct.empty()) {
            submeshes.push_back(SubMesh{
                0u, static_cast<uint32_t>(indices.size()),
                static_cast<MaterialId>(distinct[0] >> 16),
                static_cast<uint16_t>(distinct[0] & 0xFFFFu)
            });
        }
        return;
    }

    auto bucket_of = [&distinct](uint32_t key) -> size_t {
        return static_cast<size_t>(
            std::lower_bound(distinct.begin(), distinct.end(), key) - distinct.begin());
    };

    // Counting sort over the compacted buckets: stable, single pass, O(n).
    std::vector<uint32_t> counts(bucket_count, 0u);
    for (uint32_t key : triangle_key) {
        ++counts[bucket_of(key)];
    }

    std::vector<uint32_t> cursor(bucket_count, 0u);
    uint32_t running = 0u;
    for (size_t m = 0; m < bucket_count; ++m) {
        cursor[m] = running;
        running += counts[m];
    }

    std::vector<uint32_t> sorted(indices.size());
    for (size_t t = 0; t < triangle_count; ++t) {
        const uint32_t dst = cursor[bucket_of(triangle_key[t])]++;
        sorted[dst * 3u + 0u] = indices[t * 3u + 0u];
        sorted[dst * 3u + 1u] = indices[t * 3u + 1u];
        sorted[dst * 3u + 2u] = indices[t * 3u + 2u];
    }
    indices.swap(sorted);

    submeshes.clear();
    uint32_t offset = 0u;
    for (size_t m = 0; m < bucket_count; ++m) {
        if (counts[m] == 0u) {
            continue;
        }
        const uint32_t range_indices = counts[m] * 3u;
        submeshes.push_back(SubMesh{
            offset, range_indices,
            static_cast<MaterialId>(distinct[m] >> 16),
            static_cast<uint16_t>(distinct[m] & 0xFFFFu)
        });
        offset += range_indices;
    }
}

} // namespace stratum
