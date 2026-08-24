/**
 * @file road_export.cpp
 * @brief Implementation of the chunked, material-preserving network exporter
 *
 * Three things this file has to get right, and one it deliberately does not do.
 *
 * ### Every triangle exactly once
 *
 * The chunk of a triangle is decided by its centroid and nothing else. No
 * clipping, no per-piece routing, no "touches this cell" test -- each of those is
 * a way to emit a triangle twice or not at all, and emitting it twice is the bug
 * P0 deleted TileManager over. A triangle whose centroid is in a cell is written
 * to that cell whole and overhangs the boundary by at most its own size, which is
 * metres at worst against a 500 m cell.
 *
 * The consequence is the invariant the tests check: summing ExportStats::triangles
 * over an export returns the input triangle count exactly.
 *
 * ### Material slots survive
 *
 * The whole point of the SubMesh work from P0.3 onward is that a game engine
 * receives real material slots rather than one grey soup. So a chunk is assembled
 * per material -- one index list per MaterialId, concatenated in ascending order
 * -- and both writers turn those ranges into the thing their format calls a
 * material group: a `usemtl` block in OBJ, a separate primitive with its own
 * material index in glTF.
 *
 * A vertex referenced from two chunks is written into both. That is duplication
 * of VERTICES, which is unavoidable when a mesh is cut up and costs a few bytes;
 * it is not duplication of GEOMETRY, which is what the old code did.
 *
 * ### glTF is written here, by hand
 *
 * assimp is vendored and can export glTF2, and it is NOT used, for a reason that
 * has nothing to do with assimp's quality: assimp links into stratum_editor_lib,
 * not stratum_core (see CMakeLists.txt), and export has to stay in core or it
 * stops being testable without a GPU and a window. Pulling assimp down into core
 * to write a JSON file would drag an entire import framework, its own material
 * model and its scene graph across the library boundary the project is built
 * around.
 *
 * What is actually needed is small: glTF 2.0 with one buffer, one mesh, one
 * primitive per material, five vertex accessors and an index accessor per
 * primitive. nlohmann_json already links into core. So it is written directly,
 * and the output is plain `.gltf` + `.bin` rather than `.glb` so a chunk's
 * geometry can be looked at without a JSON parser in the way.
 *
 * draco also links into core and is also not used. Compressing geometry that is
 * still being debugged hides exactly the errors an export exists to reveal.
 */

#include "osm/road/road_export.hpp"

#include "osm/road/road_style.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace stratum::osm::road {
namespace {

namespace fs = std::filesystem;

// ============================================================================
// Constants
// ============================================================================

/// Number of material slots, for bounds-checking a SubMesh's material
constexpr size_t kMaterialCount = static_cast<size_t>(MaterialId::Count);

/// Text is buffered to this many bytes before it reaches the stream
constexpr size_t kTextFlushBytes = 1u << 20;

/// glTF component type for a 32-bit float
constexpr int kGltfFloat = 5126;

/// glTF component type for a 32-bit unsigned integer index
constexpr int kGltfUnsignedInt = 5125;

/// glTF bufferView target for vertex attributes (ARRAY_BUFFER)
constexpr int kGltfArrayBuffer = 34962;

/// glTF bufferView target for indices (ELEMENT_ARRAY_BUFFER)
constexpr int kGltfElementArrayBuffer = 34963;

/// glTF primitive mode for a triangle list
constexpr int kGltfTriangles = 4;

/**
 * @brief Debug base colour per material slot
 *
 * Not an attempt at a material library -- a consuming engine binds its own
 * shaders to these slots by name. It exists so that a chunk opened in a viewer
 * before any material is bound reads as a road rather than as one grey mass, and
 * so that a mis-assigned submesh is visible immediately.
 */
[[nodiscard]] glm::vec3 material_base_color(MaterialId material) {
    switch (material) {
        case MaterialId::Default:    return { 0.60f, 0.60f, 0.60f };
        case MaterialId::Asphalt:    return { 0.22f, 0.22f, 0.24f };
        case MaterialId::Concrete:   return { 0.62f, 0.62f, 0.60f };
        case MaterialId::Curb:       return { 0.74f, 0.73f, 0.70f };
        case MaterialId::Sidewalk:   return { 0.68f, 0.66f, 0.62f };
        case MaterialId::Markings:   return { 0.94f, 0.94f, 0.90f };
        case MaterialId::Gravel:     return { 0.50f, 0.47f, 0.42f };
        case MaterialId::Dirt:       return { 0.45f, 0.35f, 0.26f };
        case MaterialId::Grass:      return { 0.30f, 0.45f, 0.20f };
        case MaterialId::BridgeDeck: return { 0.55f, 0.55f, 0.56f };
        case MaterialId::Parapet:    return { 0.70f, 0.70f, 0.68f };
        // Building slots. No road piece carries either; the arms exist so the
        // switch stays exhaustive and a new slot is a warning, not a grey mass.
        case MaterialId::Wall:       return { 0.72f, 0.62f, 0.52f };
        case MaterialId::Roof:       return { 0.48f, 0.28f, 0.26f };
        case MaterialId::Count:      break;
    }
    return { 0.60f, 0.60f, 0.60f };
}

/// Unpack a MaterialKey::packed() value back into its two halves
[[nodiscard]] MaterialKey unpack_material_key(uint32_t packed) {
    return MaterialKey{ static_cast<MaterialId>(packed >> 16), static_cast<uint16_t>(packed) };
}

/**
 * @brief Name an emitted material, prefix included
 *
 * material_key_name(), NOT material_id_name(): the pipeline distinguishes an
 * ordinary asphalt carriageway from a worn or a cobbled one by the VARIANT half
 * of its MaterialKey, and naming an export by the slot alone collapses every such
 * pair into one `usemtl` line, one glTF material and one `newmtl` block. The
 * distinction then cannot be re-made in the target engine at all, because the
 * ranges were merged before any file was written. road_style.hpp documents these
 * names as frozen precisely because they travel in exported files.
 */
[[nodiscard]] std::string material_name(MaterialKey key, const ExportConfig& cfg) {
    return cfg.material_prefix + material_key_name(key);
}

// ============================================================================
// Frame conversion
// ============================================================================

/**
 * @brief Convert a direction or position out of the pipeline's Y-up frame
 *
 * A rotation, not a mirror, so winding is untouched either way.
 */
[[nodiscard]] glm::vec3 to_export_frame(const glm::vec3& v, bool y_up) {
    if (y_up) return v;
    return { v.x, -v.z, v.y };
}

// ============================================================================
// Chunking
// ============================================================================

/// Integer grid cell on the world X and Z axes
struct ChunkKey {
    int32_t x = 0;
    int32_t z = 0;

    bool operator<(const ChunkKey& other) const {
        if (x != other.x) return x < other.x;
        return z < other.z;
    }
    bool operator==(const ChunkKey& other) const { return x == other.x && z == other.z; }
};

/**
 * @brief One chunk under construction
 *
 * Indices are kept in per-material lists and concatenated only at the end, so a
 * chunk's SubMesh ranges come out contiguous and in ascending MaterialKey order
 * without any post-hoc sorting pass.
 *
 * Keyed on MaterialKey::packed(), not on the slot: two ranges sharing a slot and
 * differing in variant are two materials, and a fixed slot-indexed bucket array
 * merged them before any writer saw them. std::map keeps the ascending packed
 * order -- slot first, then variant -- that the ranges are documented to come out
 * in, and the variant axis is sparse enough that a bucket per possible key is not
 * an option.
 */
struct ChunkAccum {
    std::vector<Vertex> vertices;
    std::unordered_map<uint64_t, uint32_t> vertex_map;   ///< (source id, vertex) -> local index
    std::map<uint32_t, std::vector<uint32_t>> by_material;  ///< packed MaterialKey -> indices
    size_t triangles = 0;
};

using ChunkMap = std::map<ChunkKey, ChunkAccum>;

/// Identity of a source vertex across every mesh contributing to an export
[[nodiscard]] uint64_t source_vertex_key(uint32_t source_id, uint32_t vertex) {
    return (static_cast<uint64_t>(source_id) << 32) | static_cast<uint64_t>(vertex);
}

/// Cell a world-space point falls in. Everything lands in (0, 0) when unchunked.
[[nodiscard]] ChunkKey cell_of(const glm::vec3& point, float chunk_size) {
    if (!(chunk_size > 0.0f)) {
        return ChunkKey{ 0, 0 };
    }
    const double size = static_cast<double>(chunk_size);
    return ChunkKey{
        static_cast<int32_t>(std::floor(static_cast<double>(point.x) / size)),
        static_cast<int32_t>(std::floor(static_cast<double>(point.z) / size))
    };
}

/**
 * @brief Per-triangle packed MaterialKey, resolving the implicit whole-mesh range
 *
 * Triangles covered by no range keep `MaterialKey{}` -- MaterialId::Default,
 * variant 0 -- rather than being dropped, which matches
 * Mesh::sort_submeshes_by_material().
 *
 * The key is the (slot, variant) PAIR, exactly as sort_submeshes_by_material()
 * keys it. Keying on the slot alone silently merged a worn asphalt range with an
 * ordinary one before the writers ran, so the two could not be given different
 * textures in the target engine.
 */
[[nodiscard]] std::vector<uint32_t> triangle_materials(const Mesh& mesh) {
    const size_t tri_count = mesh.indices.size() / 3u;
    std::vector<uint32_t> out(tri_count, MaterialKey{}.packed());

    for (const SubMesh& sub : mesh.effective_submeshes()) {
        if (static_cast<size_t>(sub.material) >= kMaterialCount) {
            continue;
        }
        const uint32_t key = MaterialKey{ sub.material, sub.variant }.packed();
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset)
                             + static_cast<size_t>(sub.index_count)) / 3u;
        for (size_t t = first; t < last && t < tri_count; ++t) {
            out[t] = key;
        }
    }
    return out;
}

/**
 * @brief Route every triangle of one mesh into its chunk, whole
 *
 * @param chunks     Accumulating chunk map
 * @param mesh       Source geometry, world space
 * @param source_id  Identity of the mesh, so its vertices are deduplicated
 *                   within a chunk without colliding with another mesh's
 * @param chunk_size Grid cell size; 0 or less puts everything in one chunk
 * @param used       Packed MaterialKeys seen anywhere in the export
 * @return Triangles routed
 */
size_t accumulate_mesh(ChunkMap& chunks, const Mesh& mesh, uint32_t source_id,
                       float chunk_size, std::set<uint32_t>& used) {
    if (mesh.vertices.empty() || mesh.indices.size() < 3u) {
        return 0;
    }

    const std::vector<uint32_t> tri_material = triangle_materials(mesh);
    const size_t tri_count = mesh.indices.size() / 3u;
    const size_t vertex_count = mesh.vertices.size();
    size_t routed = 0;

    for (size_t t = 0; t < tri_count; ++t) {
        const uint32_t index[3] = {
            mesh.indices[t * 3u + 0u],
            mesh.indices[t * 3u + 1u],
            mesh.indices[t * 3u + 2u]
        };
        if (index[0] >= vertex_count || index[1] >= vertex_count || index[2] >= vertex_count) {
            continue;
        }

        const glm::vec3 centroid = (mesh.vertices[index[0]].position
                                  + mesh.vertices[index[1]].position
                                  + mesh.vertices[index[2]].position) / 3.0f;
        if (!std::isfinite(centroid.x) || !std::isfinite(centroid.y) || !std::isfinite(centroid.z)) {
            continue;
        }

        ChunkAccum& chunk = chunks[cell_of(centroid, chunk_size)];
        const uint32_t key = tri_material[t];
        std::vector<uint32_t>& target = chunk.by_material[key];

        for (uint32_t vi : index) {
            const uint64_t key = source_vertex_key(source_id, vi);
            const auto [it, inserted] =
                chunk.vertex_map.emplace(key, static_cast<uint32_t>(chunk.vertices.size()));
            if (inserted) {
                chunk.vertices.push_back(mesh.vertices[vi]);
            }
            target.push_back(it->second);
        }

        ++chunk.triangles;
        ++routed;
        used.insert(key);
    }

    return routed;
}

/// Concatenate a chunk's per-material index lists into one mesh with SubMesh ranges
[[nodiscard]] Mesh finish_chunk(const ChunkAccum& chunk) {
    Mesh mesh;
    mesh.vertices = chunk.vertices;

    size_t total = 0;
    for (const auto& [key, list] : chunk.by_material) {
        (void)key;
        total += list.size();
    }
    mesh.indices.reserve(total);

    for (const auto& [key, list] : chunk.by_material) {
        if (list.empty()) continue;

        const MaterialKey material = unpack_material_key(key);

        SubMesh range;
        range.index_offset = static_cast<uint32_t>(mesh.indices.size());
        range.index_count = static_cast<uint32_t>(list.size());
        range.material = material.material;
        range.variant = material.variant;
        mesh.submeshes.push_back(range);
        mesh.indices.insert(mesh.indices.end(), list.begin(), list.end());
    }

    mesh.compute_bounds();
    return mesh;
}

// ============================================================================
// Buffered text output
// ============================================================================

/// A write-only text file that batches into one large buffer before it hits disk
class TextWriter {
public:
    explicit TextWriter(const fs::path& path)
        : m_out(path, std::ios::binary | std::ios::trunc) {
        m_buffer.reserve(kTextFlushBytes + 256u);
    }

    ~TextWriter() { flush(); }

    TextWriter(const TextWriter&) = delete;
    TextWriter& operator=(const TextWriter&) = delete;

    [[nodiscard]] bool ok() const { return m_out.is_open() && m_out.good(); }

    void put(const char* text, size_t length) {
        m_buffer.append(text, length);
        if (m_buffer.size() >= kTextFlushBytes) {
            flush();
        }
    }

    void put(const std::string& text) { put(text.data(), text.size()); }

    void flush() {
        if (!m_buffer.empty() && m_out.is_open()) {
            m_out.write(m_buffer.data(), static_cast<std::streamsize>(m_buffer.size()));
            m_buffer.clear();
        }
    }

private:
    std::ofstream m_out;
    std::string m_buffer;
};

/**
 * @brief Bytes of a fixed buffer that snprintf actually wrote
 *
 * snprintf returns the length the output WOULD have had, not the length it
 * wrote. Handing that straight to TextWriter::put reads off the end of the stack
 * array and appends whatever was next to it to the file, which is undefined
 * behaviour and a corrupt line in the same step. Everything formatted here is
 * bounded except the caller-supplied names, and those are built as std::string
 * instead; this clamps the rest.
 *
 * @param n   Return value of snprintf, which may be negative on an encoding error
 * @param cap Size of the buffer, including the terminator snprintf always writes
 * @return Number of bytes at the front of the buffer that are real output
 */
[[nodiscard]] size_t formatted_length(int n, size_t cap) {
    if (n <= 0 || cap == 0u) {
        return 0u;
    }
    return std::min(static_cast<size_t>(n), cap - 1u);
}

// ============================================================================
// Wavefront OBJ
// ============================================================================

/**
 * @brief Write one mesh as an OBJ, one `usemtl` group per SubMesh range
 *
 * Positions, texture coordinates and normals are written one per vertex and in
 * the same order, so a face triple is always `i/i/i`. That is redundant by a few
 * bytes per face and is worth it: every OBJ importer in existence reads it
 * without guessing.
 */
[[nodiscard]] bool write_obj_file(const Mesh& mesh, const fs::path& obj_path,
                                  const std::string& mtllib_name,
                                  const std::string& object_name,
                                  const ExportConfig& cfg) {
    if (mesh.vertices.empty() || mesh.indices.size() < 3u) {
        return false;
    }

    TextWriter out(obj_path);
    if (!out.ok()) {
        spdlog::error("export: cannot open {} for writing", obj_path.string());
        return false;
    }

    char line[256];
    int n = 0;

    out.put("# Stratum road export\n");
    // Built as strings, not formatted into `line`: both names come from the
    // caller -- mtllib_name is the export stem, object_name the chunk name -- and
    // a name longer than the buffer would be silently truncated into a directive
    // pointing at a file that does not exist.
    out.put("mtllib " + mtllib_name + "\n");
    out.put("o " + object_name + "\n");

    for (const Vertex& v : mesh.vertices) {
        const glm::vec3 p = to_export_frame(v.position, cfg.y_up);
        n = std::snprintf(line, sizeof(line), "v %.4f %.4f %.4f\n",
                          static_cast<double>(p.x), static_cast<double>(p.y),
                          static_cast<double>(p.z));
        out.put(line, formatted_length(n, sizeof(line)));
    }
    for (const Vertex& v : mesh.vertices) {
        n = std::snprintf(line, sizeof(line), "vt %.6f %.6f\n",
                          static_cast<double>(v.uv.x), static_cast<double>(v.uv.y));
        out.put(line, formatted_length(n, sizeof(line)));
    }
    for (const Vertex& v : mesh.vertices) {
        const glm::vec3 nv = to_export_frame(v.normal, cfg.y_up);
        n = std::snprintf(line, sizeof(line), "vn %.4f %.4f %.4f\n",
                          static_cast<double>(nv.x), static_cast<double>(nv.y),
                          static_cast<double>(nv.z));
        out.put(line, formatted_length(n, sizeof(line)));
    }

    for (const SubMesh& range : mesh.effective_submeshes()) {
        if (range.index_count < 3u) continue;

        // Same reasoning as the header lines: this one carries object_name AND
        // ExportConfig::material_prefix, whose documented purpose is to namespace
        // the materials into another project, so its length is not bounded by
        // anything this file controls.
        const MaterialKey key{ range.material, range.variant };
        const std::string name = material_name(key, cfg);
        out.put("g " + object_name + "_" + material_key_name(key)
                + "\nusemtl " + name + "\n");

        const size_t first = range.index_offset;
        const size_t last = std::min<size_t>(mesh.indices.size(),
                                             static_cast<size_t>(range.index_offset)
                                             + static_cast<size_t>(range.index_count));
        for (size_t i = first; i + 3u <= last; i += 3) {
            // OBJ indices are 1-based and shared across v / vt / vn.
            const unsigned long a = static_cast<unsigned long>(mesh.indices[i]) + 1ul;
            const unsigned long b = static_cast<unsigned long>(mesh.indices[i + 1]) + 1ul;
            const unsigned long c = static_cast<unsigned long>(mesh.indices[i + 2]) + 1ul;
            n = std::snprintf(line, sizeof(line), "f %lu/%lu/%lu %lu/%lu/%lu %lu/%lu/%lu\n",
                              a, a, a, b, b, b, c, c, c);
            out.put(line, formatted_length(n, sizeof(line)));
        }
    }

    out.flush();
    return out.ok();
}

/// Write the material library shared by every OBJ of one export
[[nodiscard]] bool write_mtl_file(const std::set<uint32_t>& materials, const fs::path& mtl_path,
                                  const ExportConfig& cfg) {
    TextWriter out(mtl_path);
    if (!out.ok()) {
        spdlog::error("export: cannot open {} for writing", mtl_path.string());
        return false;
    }

    out.put("# Stratum road export material library\n");
    char line[256];
    for (const uint32_t packed : materials) {
        const MaterialKey key = unpack_material_key(packed);
        const glm::vec3 base = material_base_color(key.material);
        // The name carries ExportConfig::material_prefix and is unbounded, so it
        // goes out as a string; only the fixed numeric block is formatted.
        out.put("\nnewmtl " + material_name(key, cfg) + "\n");
        const int n = std::snprintf(line, sizeof(line),
                                    "Ka 0.000 0.000 0.000\n"
                                    "Kd %.3f %.3f %.3f\nKs 0.000 0.000 0.000\n"
                                    "Ns 16.0\nd 1.0\nillum 2\n",
                                    static_cast<double>(base.r), static_cast<double>(base.g),
                                    static_cast<double>(base.b));
        out.put(line, formatted_length(n, sizeof(line)));
    }

    out.flush();
    return out.ok();
}

// ============================================================================
// glTF 2.0
// ============================================================================

/// Growable little-endian byte buffer backing a glTF `.bin`
class BinaryBuffer {
public:
    /// Append raw bytes, returning the offset they were written at
    size_t append(const void* data, size_t length) {
        const size_t offset = m_bytes.size();
        const auto* begin = static_cast<const uint8_t*>(data);
        m_bytes.insert(m_bytes.end(), begin, begin + length);
        return offset;
    }

    [[nodiscard]] const std::vector<uint8_t>& bytes() const { return m_bytes; }

private:
    std::vector<uint8_t> m_bytes;
};

/**
 * @brief Write one mesh as `.gltf` plus its `.bin`
 *
 * One buffer, one bufferView per attribute, one shared set of vertex accessors,
 * and one primitive per SubMesh range with its own index accessor and its own
 * material. That is the smallest structure that gives a consuming engine real
 * material slots, and it is exactly what the pipeline's SubMesh ranges mean.
 *
 * glTF 2.0 requires Y up, so ExportConfig::y_up is ignored here; the caller
 * warns.
 */
[[nodiscard]] bool write_gltf_file(const Mesh& mesh, const fs::path& gltf_path,
                                   const fs::path& bin_path, const std::string& object_name,
                                   const ExportConfig& cfg) {
    if (mesh.vertices.empty() || mesh.indices.size() < 3u) {
        return false;
    }

    const size_t vertex_count = mesh.vertices.size();

    // De-interleave into the five attribute arrays glTF wants.
    std::vector<float> positions(vertex_count * 3u);
    std::vector<float> normals(vertex_count * 3u);
    std::vector<float> uvs(vertex_count * 2u);
    std::vector<float> colors(vertex_count * 4u);
    std::vector<float> tangents(vertex_count * 4u);

    glm::vec3 min_pos(std::numeric_limits<float>::max());
    glm::vec3 max_pos(std::numeric_limits<float>::lowest());

    for (size_t i = 0; i < vertex_count; ++i) {
        const Vertex& v = mesh.vertices[i];
        positions[i * 3u + 0u] = v.position.x;
        positions[i * 3u + 1u] = v.position.y;
        positions[i * 3u + 2u] = v.position.z;
        min_pos = glm::min(min_pos, v.position);
        max_pos = glm::max(max_pos, v.position);

        normals[i * 3u + 0u] = v.normal.x;
        normals[i * 3u + 1u] = v.normal.y;
        normals[i * 3u + 2u] = v.normal.z;

        uvs[i * 2u + 0u] = v.uv.x;
        uvs[i * 2u + 1u] = v.uv.y;

        colors[i * 4u + 0u] = v.color.r;
        colors[i * 4u + 1u] = v.color.g;
        colors[i * 4u + 2u] = v.color.b;
        colors[i * 4u + 3u] = v.color.a;

        tangents[i * 4u + 0u] = v.tangent.x;
        tangents[i * 4u + 1u] = v.tangent.y;
        tangents[i * 4u + 2u] = v.tangent.z;
        tangents[i * 4u + 3u] = v.tangent.w < 0.0f ? -1.0f : 1.0f;
    }

    BinaryBuffer buffer;
    nlohmann::json views = nlohmann::json::array();

    const auto add_view = [&](const void* data, size_t length, int target) -> int {
        const size_t offset = buffer.append(data, length);
        nlohmann::json view;
        view["buffer"] = 0;
        view["byteOffset"] = offset;
        view["byteLength"] = length;
        view["target"] = target;
        views.push_back(view);
        return static_cast<int>(views.size()) - 1;
    };

    const int view_position = add_view(positions.data(), positions.size() * sizeof(float),
                                       kGltfArrayBuffer);
    const int view_normal = add_view(normals.data(), normals.size() * sizeof(float),
                                     kGltfArrayBuffer);
    const int view_uv = add_view(uvs.data(), uvs.size() * sizeof(float), kGltfArrayBuffer);
    const int view_color = add_view(colors.data(), colors.size() * sizeof(float),
                                    kGltfArrayBuffer);
    const int view_tangent = add_view(tangents.data(), tangents.size() * sizeof(float),
                                      kGltfArrayBuffer);
    const int view_index = add_view(mesh.indices.data(), mesh.indices.size() * sizeof(uint32_t),
                                    kGltfElementArrayBuffer);

    nlohmann::json accessors = nlohmann::json::array();
    const auto add_accessor = [&](int view, int component_type, const char* type, size_t count,
                                  size_t byte_offset) -> int {
        nlohmann::json accessor;
        accessor["bufferView"] = view;
        accessor["byteOffset"] = byte_offset;
        accessor["componentType"] = component_type;
        accessor["count"] = count;
        accessor["type"] = type;
        accessors.push_back(accessor);
        return static_cast<int>(accessors.size()) - 1;
    };

    const int acc_position = add_accessor(view_position, kGltfFloat, "VEC3", vertex_count, 0);
    accessors[static_cast<size_t>(acc_position)]["min"] =
        { min_pos.x, min_pos.y, min_pos.z };
    accessors[static_cast<size_t>(acc_position)]["max"] =
        { max_pos.x, max_pos.y, max_pos.z };

    const int acc_normal = add_accessor(view_normal, kGltfFloat, "VEC3", vertex_count, 0);
    const int acc_uv = add_accessor(view_uv, kGltfFloat, "VEC2", vertex_count, 0);
    const int acc_color = add_accessor(view_color, kGltfFloat, "VEC4", vertex_count, 0);
    const int acc_tangent = add_accessor(view_tangent, kGltfFloat, "VEC4", vertex_count, 0);

    // One primitive and one material per SubMesh range, in the order the ranges
    // appear, which finish_chunk() leaves in ascending MaterialKey order.
    nlohmann::json primitives = nlohmann::json::array();
    nlohmann::json materials = nlohmann::json::array();

    for (const SubMesh& range : mesh.effective_submeshes()) {
        if (range.index_count < 3u) continue;
        if (static_cast<size_t>(range.index_offset) + range.index_count > mesh.indices.size()) {
            continue;
        }

        const int acc_index = add_accessor(view_index, kGltfUnsignedInt, "SCALAR",
                                           range.index_count,
                                           static_cast<size_t>(range.index_offset)
                                           * sizeof(uint32_t));

        const glm::vec3 base = material_base_color(range.material);
        nlohmann::json material;
        material["name"] = material_name(MaterialKey{ range.material, range.variant }, cfg);
        material["doubleSided"] = false;
        material["pbrMetallicRoughness"] = {
            { "baseColorFactor", { base.r, base.g, base.b, 1.0f } },
            { "metallicFactor", 0.0 },
            { "roughnessFactor", 0.9 }
        };
        materials.push_back(material);

        nlohmann::json primitive;
        primitive["attributes"] = {
            { "POSITION", acc_position },
            { "NORMAL", acc_normal },
            { "TEXCOORD_0", acc_uv },
            { "COLOR_0", acc_color },
            { "TANGENT", acc_tangent }
        };
        primitive["indices"] = acc_index;
        primitive["material"] = static_cast<int>(materials.size()) - 1;
        primitive["mode"] = kGltfTriangles;
        primitives.push_back(primitive);
    }

    if (primitives.empty()) {
        return false;
    }

    nlohmann::json doc;
    doc["asset"] = { { "version", "2.0" }, { "generator", "Stratum road exporter" } };
    doc["scene"] = 0;
    doc["scenes"] = nlohmann::json::array({ nlohmann::json{ { "nodes", { 0 } } } });
    doc["nodes"] = nlohmann::json::array({
        nlohmann::json{ { "mesh", 0 }, { "name", object_name } }
    });
    doc["meshes"] = nlohmann::json::array({
        nlohmann::json{ { "name", object_name }, { "primitives", primitives } }
    });
    doc["materials"] = materials;
    doc["accessors"] = accessors;
    doc["bufferViews"] = views;
    doc["buffers"] = nlohmann::json::array({
        nlohmann::json{ { "uri", bin_path.filename().string() },
                        { "byteLength", buffer.bytes().size() } }
    });

    {
        std::ofstream bin(bin_path, std::ios::binary | std::ios::trunc);
        if (!bin.is_open()) {
            spdlog::error("export: cannot open {} for writing", bin_path.string());
            return false;
        }
        bin.write(reinterpret_cast<const char*>(buffer.bytes().data()),
                  static_cast<std::streamsize>(buffer.bytes().size()));
        if (!bin.good()) {
            spdlog::error("export: failed while writing {}", bin_path.string());
            return false;
        }
    }

    {
        std::ofstream json_out(gltf_path, std::ios::binary | std::ios::trunc);
        if (!json_out.is_open()) {
            spdlog::error("export: cannot open {} for writing", gltf_path.string());
            return false;
        }
        const std::string text = doc.dump(2);
        json_out.write(text.data(), static_cast<std::streamsize>(text.size()));
        if (!json_out.good()) {
            spdlog::error("export: failed while writing {}", gltf_path.string());
            return false;
        }
    }

    return true;
}

// ============================================================================
// Shared plumbing
// ============================================================================

/// File extension of a format, dot included
[[nodiscard]] const char* format_extension(ExportFormat format) {
    return format == ExportFormat::Gltf ? ".gltf" : ".obj";
}

/// `road_<cx>_<cz>` when chunked, `road` when not
[[nodiscard]] std::string chunk_stem(const ChunkKey& key, bool chunked) {
    if (!chunked) return "road";
    return "road_" + std::to_string(key.x) + "_" + std::to_string(key.z);
}

/// Absolute form of a path, falling back to the path itself when it cannot be taken
[[nodiscard]] std::string absolute_string(const fs::path& path) {
    std::error_code ec;
    const fs::path abs = fs::absolute(path, ec);
    return ec ? path.string() : abs.string();
}

/**
 * @brief Write one chunk in the configured format and record what was produced
 *
 * @return true when the render file (and its `.bin`, for glTF) was written
 */
bool write_chunk(const Mesh& mesh, const fs::path& out_dir, const std::string& stem,
                 const std::string& mtllib_name, const ExportConfig& cfg,
                 ExportStats& stats) {
    const fs::path path = out_dir / (stem + format_extension(cfg.format));

    if (cfg.format == ExportFormat::Gltf) {
        const fs::path bin_path = out_dir / (stem + ".bin");
        if (!write_gltf_file(mesh, path, bin_path, stem, cfg)) {
            return false;
        }
        stats.written_files.push_back(absolute_string(path));
        stats.written_files.push_back(absolute_string(bin_path));
        stats.files += 2;
        return true;
    }

    if (!write_obj_file(mesh, path, mtllib_name, stem, cfg)) {
        return false;
    }
    stats.written_files.push_back(absolute_string(path));
    stats.files += 1;
    return true;
}

} // namespace

// ============================================================================
// Entry points
// ============================================================================

ExportStats export_road_network(const std::vector<RoadPiece>& pieces,
                                const fs::path& out_dir,
                                const ExportConfig& cfg) {
    const auto start = std::chrono::steady_clock::now();
    ExportStats stats;

    const auto finish = [&stats, start]() -> ExportStats& {
        const auto end = std::chrono::steady_clock::now();
        stats.export_ms =
            std::chrono::duration<double, std::milli>(end - start).count();
        return stats;
    };

    if (pieces.empty()) {
        return finish();
    }

    ExportConfig effective = cfg;
    if (effective.format == ExportFormat::Gltf && !effective.y_up) {
        spdlog::warn("export_road_network: glTF 2.0 requires Y up; ignoring y_up = false");
        effective.y_up = true;
    }
    const float chunk_size = effective.chunk_size > 0.0f ? effective.chunk_size : 0.0f;
    const bool chunked = chunk_size > 0.0f;

    try {
        // --------------------------------------------------------------------
        // Route every triangle. Render, collision and LOD levels are chunked on
        // the same lattice, so a consumer streaming one cell gets the render
        // mesh, its collision surface and its LODs together.
        // --------------------------------------------------------------------
        std::set<uint32_t> used_materials;   ///< Packed MaterialKeys, ascending
        ChunkMap render_chunks;

        for (size_t p = 0; p < pieces.size(); ++p) {
            const size_t routed = accumulate_mesh(render_chunks, pieces[p].mesh,
                                                  static_cast<uint32_t>(p), chunk_size,
                                                  used_materials);
            if (routed > 0) {
                ++stats.meshes;
                stats.triangles += routed;
            }
        }

        if (render_chunks.empty()) {
            return finish();
        }

        ChunkMap collision_chunks;
        if (effective.export_collision) {
            for (size_t p = 0; p < pieces.size(); ++p) {
                accumulate_mesh(collision_chunks, pieces[p].collision,
                                static_cast<uint32_t>(p), chunk_size, used_materials);
            }
        }

        // LOD level 0 is the render mesh and is never written again.
        // The level count comes from the LONGEST chain present, and a piece whose
        // own chain is shorter contributes its COARSEST level to every level past
        // its end.
        //
        // Neither half of that is arbitrary. Clamping to the SHORTEST chain
        // instead writes no LOD file at all on any real network: build_lod_chain()
        // drops a level that failed to reduce by 10%, a corridor strip two vertex
        // columns wide cannot be reduced at all with LodConfig::lock_borders on,
        // and every fixture in tests/data therefore holds at least one piece with
        // a one-level chain. And simply SKIPPING a piece at a level it does not
        // have writes a LOD file with a hole in it where that piece's road was.
        // Repeating its coarsest level is what a chain means: a mesh that cannot
        // simplify further keeps the detail it has at distance.
        std::vector<ChunkMap> lod_chunks;
        if (effective.export_lods && effective.lod_levels >= 2) {
            size_t longest = 0;
            for (const RoadPiece& piece : pieces) {
                longest = std::max(longest, piece.lods.levels.size());
            }
            if (longest >= 2) {
                const size_t levels = std::min(static_cast<size_t>(effective.lod_levels), longest);
                lod_chunks.resize(levels - 1u);
                for (size_t level = 1; level < levels; ++level) {
                    for (size_t p = 0; p < pieces.size(); ++p) {
                        const LodChain& chain = pieces[p].lods;
                        if (chain.levels.empty()) continue;
                        const size_t use = std::min(level, chain.levels.size() - 1u);
                        accumulate_mesh(lod_chunks[level - 1u], chain.levels[use],
                                        static_cast<uint32_t>(p), chunk_size, used_materials);
                    }
                }
            }
        }

        // --------------------------------------------------------------------
        // Create the destination. A path that cannot be created is reported, not
        // thrown: this runs on the import thread.
        // --------------------------------------------------------------------
        std::error_code ec;
        if (!out_dir.empty() && !fs::exists(out_dir, ec)) {
            fs::create_directories(out_dir, ec);
            if (ec) {
                spdlog::error("export_road_network: cannot create {}: {}",
                              out_dir.string(), ec.message());
                return finish();
            }
        }
        if (!out_dir.empty() && !fs::is_directory(out_dir, ec)) {
            spdlog::error("export_road_network: {} is not a directory", out_dir.string());
            return finish();
        }

        // --------------------------------------------------------------------
        // Write, in the documented order: chunks ascending by grid X then Z, and
        // within a chunk the render file, its sidecar, collision, then LODs.
        // --------------------------------------------------------------------
        const std::string mtllib_name = "road.mtl";
        bool mtl_written = false;

        std::set<ChunkKey> keys;
        for (const auto& [key, chunk] : render_chunks) { (void)chunk; keys.insert(key); }
        for (const auto& [key, chunk] : collision_chunks) { (void)chunk; keys.insert(key); }
        for (const ChunkMap& level : lod_chunks) {
            for (const auto& [key, chunk] : level) { (void)chunk; keys.insert(key); }
        }

        for (const ChunkKey& key : keys) {
            const std::string stem = chunk_stem(key, chunked);

            const auto render_it = render_chunks.find(key);
            if (render_it != render_chunks.end()) {
                const Mesh mesh = finish_chunk(render_it->second);
                if (write_chunk(mesh, out_dir, stem, mtllib_name, effective, stats)) {
                    ++stats.chunks;
                    stats.vertices += mesh.vertices.size();

                    // The MTL is shared by every OBJ of the export, so it is
                    // written once -- as the first chunk's sidecar.
                    if (effective.format == ExportFormat::Obj && !mtl_written) {
                        const fs::path mtl_path = out_dir / mtllib_name;
                        if (write_mtl_file(used_materials, mtl_path, effective)) {
                            stats.written_files.push_back(absolute_string(mtl_path));
                            stats.files += 1;
                        }
                        mtl_written = true;
                    }
                }
            }

            const auto collision_it = collision_chunks.find(key);
            if (collision_it != collision_chunks.end()) {
                const Mesh mesh = finish_chunk(collision_it->second);
                write_chunk(mesh, out_dir, stem + "_collision", mtllib_name, effective, stats);
            }

            for (size_t level = 0; level < lod_chunks.size(); ++level) {
                const auto lod_it = lod_chunks[level].find(key);
                if (lod_it == lod_chunks[level].end()) continue;
                const Mesh mesh = finish_chunk(lod_it->second);
                write_chunk(mesh, out_dir, stem + "_lod" + std::to_string(level + 1u),
                            mtllib_name, effective, stats);
            }
        }
    } catch (const std::exception& e) {
        spdlog::error("export_road_network: aborted after {} files: {}", stats.files, e.what());
        return finish();
    }

    finish();
    spdlog::info("export_road_network: {} chunks, {} meshes, {} triangles, {} files in {:.1f} ms",
                 stats.chunks, stats.meshes, stats.triangles, stats.files, stats.export_ms);
    return stats;
}

bool export_mesh(const Mesh& mesh, const fs::path& out_path, const ExportConfig& cfg) {
    if (mesh.vertices.empty() || mesh.indices.size() < 3u) {
        return false;
    }

    ExportConfig effective = cfg;
    if (effective.format == ExportFormat::Gltf && !effective.y_up) {
        spdlog::warn("export_mesh: glTF 2.0 requires Y up; ignoring y_up = false");
        effective.y_up = true;
    }

    try {
        std::error_code ec;
        const fs::path parent = out_path.parent_path();
        if (!parent.empty() && !fs::exists(parent, ec)) {
            fs::create_directories(parent, ec);
            if (ec) {
                spdlog::error("export_mesh: cannot create {}: {}", parent.string(), ec.message());
                return false;
            }
        }

        const std::string stem = out_path.stem().string();

        if (effective.format == ExportFormat::Gltf) {
            fs::path bin_path = out_path;
            bin_path.replace_extension(".bin");
            return write_gltf_file(mesh, out_path, bin_path, stem, effective);
        }

        fs::path mtl_path = out_path;
        mtl_path.replace_extension(".mtl");

        std::set<uint32_t> used;
        for (const SubMesh& range : mesh.effective_submeshes()) {
            if (static_cast<size_t>(range.material) < kMaterialCount) {
                used.insert(MaterialKey{ range.material, range.variant }.packed());
            }
        }
        const bool mtl_ok = write_mtl_file(used, mtl_path, effective);
        const bool obj_ok = write_obj_file(mesh, out_path, mtl_path.filename().string(),
                                           stem, effective);
        return obj_ok && mtl_ok;
    } catch (const std::exception& e) {
        spdlog::error("export_mesh: {} failed: {}", out_path.string(), e.what());
        return false;
    }
}

} // namespace stratum::osm::road
