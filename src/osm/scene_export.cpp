// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file scene_export.cpp
 * @brief Implementation of the whole-scene, mesh-kind-agnostic exporter
 *
 * The road exporter's shape, with two changes and one thing carefully left alone.
 *
 * ### Left alone: the chunk decision
 *
 * A triangle's chunk is decided by its centroid and by nothing else. No clipping,
 * no bounding-box overlap test, no per-object routing -- each of those is a way to
 * emit a triangle twice or lose it, and emitting it twice is the bug P0 deleted
 * TileManager over. Generalising from roads to every mesh kind does not touch this
 * step at all: accumulate_object() is the road exporter's accumulate_mesh() with
 * the RoadPiece taken out, because the thing that made the invariant true was never
 * road-specific.
 *
 * ### Changed: grouping carries the object
 *
 * The road exporter buckets a chunk's triangles by MaterialKey. This one buckets
 * them by (object, MaterialKey), so a group is always one object's triangles and
 * can be labelled with that object's metadata. The grouping key is the ONLY thing
 * that changed, and it cannot affect conservation: a triangle still lands in
 * exactly one bucket of exactly one chunk, and the buckets are concatenated whole.
 *
 * ### Changed: what is dropped is counted
 *
 * A triangle with an out-of-range index or a non-finite centroid cannot be written
 * and is skipped in both exporters. Here it is counted into
 * SceneExportStats::dropped_triangles, so `triangles + dropped_triangles` is the
 * input exactly. Without that, "written == input" is untestable in the presence of
 * any bad geometry: the sum silently agrees with itself.
 *
 * ### Material colours are derived, never tabled
 *
 * road_export.cpp carries a hard-coded debug colour per MaterialId in its anonymous
 * namespace. Copying that table here would be a second copy to drift, and it cannot
 * be shared without editing that file. So the exported `Kd` / `baseColorFactor` is
 * the MEAN VERTEX COLOUR of the triangles that used the material, which is derived
 * from the data and has nothing to drift from. Buildings and areas carry real
 * per-feature colours from attribute_palette.cpp, so this is usually better than a
 * table would have been; roads carry a debug tint, so it is no worse.
 *
 * ### glTF is written here, by hand, and assimp is not used
 *
 * Same reason as the road exporter: assimp links into stratum_editor_lib, not
 * stratum_core, and an export that needs a GPU-side library stops being testable
 * headless. nlohmann_json already links into core and the document needed is small.
 */

#include "osm/scene_export.hpp"

// material_key_name() is reached across the road/ boundary on purpose. Those names
// are frozen because they travel in exported files, and a scene export and a road
// export of the same city MUST name the same material identically or a consuming
// engine binds two materials for one surface.
#include "osm/road/road_style.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace stratum::osm {
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

/// Fallback file and object stem when SceneExportConfig::name_prefix is unusable
constexpr const char* kDefaultNamePrefix = "scene";

// ============================================================================
// Names and sanitisation
// ============================================================================

/**
 * @brief Make a string safe to use as a single OBJ token
 *
 * OBJ is whitespace-delimited, so a space inside an `o` name silently becomes a
 * second argument and the name a reader gets back is the first word only. Control
 * characters are worse: a newline in a feature name -- which OSM does contain --
 * splits one directive into two lines, the second of which is garbage the parser
 * either skips or chokes on.
 *
 * Bytes above 0x7F are passed through untouched. They are UTF-8 continuation bytes
 * of a name that is legitimately not ASCII, and mangling them would rename half the
 * features in any extract outside the English-speaking world.
 *
 * @param in Raw name
 * @return @p in with every whitespace and control byte replaced by '_'. Empty in,
 *         empty out; the caller decides the fallback.
 */
[[nodiscard]] std::string sanitize_token(std::string_view in) {
    std::string out;
    out.reserve(in.size());
    for (const char c : in) {
        const auto byte = static_cast<unsigned char>(c);
        out.push_back(byte <= 0x20u || byte == 0x7Fu ? '_' : c);
    }
    return out;
}

/**
 * @brief Make a string safe to put inside one OBJ comment line
 *
 * Looser than sanitize_token(): a comment runs to end of line, so spaces are fine
 * and only the line break has to go. A tab is folded to a space as well, since the
 * documented `key=value` grammar is read by splitting on the first '='.
 *
 * @param in Raw text
 * @return @p in with every control byte replaced by a space
 */
[[nodiscard]] std::string sanitize_line(std::string_view in) {
    std::string out;
    out.reserve(in.size());
    for (const char c : in) {
        const auto byte = static_cast<unsigned char>(c);
        out.push_back(byte < 0x20u || byte == 0x7Fu ? ' ' : c);
    }
    return out;
}

/**
 * @brief A metadata key that cannot break the `key=value` grammar
 *
 * The OBJ reader splits a `# stratum:meta` line on its FIRST '=', so an '=' inside
 * the key would move the split and hand the reader a truncated key and a value with
 * a stray prefix. glTF `extras` has no such problem and keeps the key verbatim;
 * this is the OBJ-only half of that difference, and it is why the header documents
 * glTF as the lossless channel.
 */
[[nodiscard]] std::string sanitize_meta_key(std::string_view in) {
    std::string out = sanitize_line(in);
    std::replace(out.begin(), out.end(), '=', '_');
    return out;
}

/// SceneExportConfig::name_prefix, or "scene" when it is empty or all whitespace
[[nodiscard]] std::string effective_prefix(const SceneExportConfig& cfg) {
    const std::string token = sanitize_token(cfg.name_prefix);
    const bool usable = std::any_of(token.begin(), token.end(),
                                    [](char c) { return c != '_'; });
    return usable ? token : std::string{ kDefaultNamePrefix };
}

/**
 * @brief The name one object's records carry in the output
 *
 * `<prefix>_o<index>_<name or kind>`, for example `scene_o12_Town_Hall`.
 *
 * The INDEX is in there because SceneObject::name is not an identity: two churches
 * in one extract are both "Church", and an importer that keys a collection off the
 * object name would merge them into one object with two disjoint halves. The index
 * is the only thing the exporter knows to be unique.
 *
 * @param object Object being named
 * @param index  Its position in the caller's vector
 * @param prefix effective_prefix() of the config
 */
[[nodiscard]] std::string object_record_name(const SceneObject& object, size_t index,
                                             const std::string& prefix) {
    const std::string base = object.name.empty()
                                 ? std::string{ scene_object_kind_name(object.kind) }
                                 : sanitize_token(object.name);
    return prefix + "_o" + std::to_string(index) + "_" + (base.empty() ? "unnamed" : base);
}

/// Name an emitted material, prefix included. Shared with the road exporter by design.
[[nodiscard]] std::string material_name(MaterialKey key, const SceneExportConfig& cfg) {
    return cfg.material_prefix + road::material_key_name(key);
}

/// Unpack a MaterialKey::packed() value back into its two halves
[[nodiscard]] MaterialKey unpack_material_key(uint32_t packed) {
    return MaterialKey{ static_cast<MaterialId>(packed >> 16), static_cast<uint16_t>(packed) };
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
};

/**
 * @brief What a chunk's triangles are bucketed by
 *
 * Object first, material second, and the ordering follows: a chunk emits all of
 * object 0's ranges, then all of object 1's. That contiguity is what lets the OBJ
 * writer open one `o` record per object instead of reopening it per material.
 */
struct GroupKey {
    uint32_t object = 0;    ///< Index into the caller's object vector
    uint32_t material = 0;  ///< MaterialKey::packed()

    bool operator<(const GroupKey& other) const {
        if (object != other.object) return object < other.object;
        return material < other.material;
    }
};

/**
 * @brief One chunk under construction
 *
 * Indices are kept in per-group lists and concatenated only at the end, so the
 * finished ranges come out contiguous and in ascending GroupKey order with no
 * post-hoc sorting pass.
 *
 * `by_group` is an ordered map and not a hash map deliberately. Its iteration order
 * IS the order of the ranges in the file, so a hash map would make two exports of
 * one scene differ byte for byte depending on nothing. `vertex_map` may be a hash
 * map because it is only ever looked up, never iterated.
 */
struct ChunkAccum {
    std::vector<Vertex> vertices;
    std::unordered_map<uint64_t, uint32_t> vertex_map;  ///< (object, source vertex) -> local index
    std::map<GroupKey, std::vector<uint32_t>> by_group;
    size_t triangles = 0;
};

using ChunkMap = std::map<ChunkKey, ChunkAccum>;

/**
 * @brief Running mean vertex colour of one material, across the whole export
 *
 * Summed in double because a city has millions of vertices and a float accumulator
 * stops moving once the running total is large enough that one more sample rounds
 * to nothing.
 */
struct MaterialAccum {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    size_t samples = 0;
};

/// packed MaterialKey -> its accumulated colour. Ordered, so the MTL is deterministic.
using MaterialTable = std::map<uint32_t, MaterialAccum>;

/// A finished chunk: geometry plus which object each of its ranges came from
struct ChunkMesh {
    Mesh mesh;

    /**
     * @brief Parallel to Mesh::submeshes: the object index behind each range
     *
     * A parallel vector rather than a field on SubMesh, because SubMesh is the
     * renderer's type and an exporter has no business adding to it. Always exactly
     * `mesh.submeshes.size()` entries.
     */
    std::vector<uint32_t> range_object;
};

/// Identity of a source vertex across every object contributing to an export
[[nodiscard]] uint64_t source_vertex_key(uint32_t object_index, uint32_t vertex) {
    return (static_cast<uint64_t>(object_index) << 32) | static_cast<uint64_t>(vertex);
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
 * Triangles covered by no range keep `MaterialKey{}` -- Default, variant 0 --
 * rather than being dropped, matching Mesh::sort_submeshes_by_material() and the
 * road exporter. The key is the (slot, variant) PAIR: keying on the slot alone
 * merges a cobbled carriageway with an asphalt one before any writer sees them, and
 * the two can then never be given different textures downstream.
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
 * @brief Route every triangle of one object into its chunk, whole
 *
 * The only place a triangle is assigned anywhere, and therefore the only place the
 * conservation invariant can be broken. Nothing here loops over cells, and nothing
 * here looks at an object's extent: a triangle is looked at once, its centroid is
 * computed once, and it is pushed into one bucket.
 *
 * @param chunks       Accumulating chunk map
 * @param object       Object to route. A null or empty mesh routes nothing.
 * @param object_index Identity of the object, so its vertices deduplicate within a
 *                     chunk without colliding with another object's
 * @param chunk_size   Grid cell size; 0 or less puts everything in one chunk
 * @param materials    Accumulating per-material colour table
 * @param dropped      Incremented per triangle that could not be assigned
 * @return Triangles routed
 */
size_t accumulate_object(ChunkMap& chunks, const SceneObject& object, uint32_t object_index,
                         float chunk_size, MaterialTable& materials, size_t& dropped) {
    if (object.mesh == nullptr) {
        return 0;
    }
    const Mesh& mesh = *object.mesh;
    if (mesh.vertices.empty() || mesh.indices.size() < 3u) {
        return 0;
    }

    const std::vector<uint32_t> tri_material = triangle_materials(mesh);
    const size_t tri_count = mesh.indices.size() / 3u;
    const size_t vertex_count = mesh.vertices.size();
    size_t routed = 0;

    for (size_t t = 0; t < tri_count; ++t) {
        const uint32_t corner[3] = {
            mesh.indices[t * 3u + 0u],
            mesh.indices[t * 3u + 1u],
            mesh.indices[t * 3u + 2u]
        };
        if (corner[0] >= vertex_count || corner[1] >= vertex_count
            || corner[2] >= vertex_count) {
            ++dropped;
            continue;
        }

        const glm::vec3 centroid = (mesh.vertices[corner[0]].position
                                  + mesh.vertices[corner[1]].position
                                  + mesh.vertices[corner[2]].position) / 3.0f;
        // A NaN position cannot be floored into a cell, and writing it would poison
        // the chunk's bounding box for everything else in the file.
        if (!std::isfinite(centroid.x) || !std::isfinite(centroid.y)
            || !std::isfinite(centroid.z)) {
            ++dropped;
            continue;
        }

        const uint32_t material = tri_material[t];
        ChunkAccum& chunk = chunks[cell_of(centroid, chunk_size)];
        std::vector<uint32_t>& target = chunk.by_group[GroupKey{ object_index, material }];

        for (const uint32_t vi : corner) {
            const uint64_t vertex_key = source_vertex_key(object_index, vi);
            const auto [it, inserted] =
                chunk.vertex_map.emplace(vertex_key, static_cast<uint32_t>(chunk.vertices.size()));
            if (inserted) {
                chunk.vertices.push_back(mesh.vertices[vi]);
            }
            target.push_back(it->second);
        }

        // Colour is sampled per triangle corner rather than per mesh vertex: a
        // vertex shared by two materials would otherwise vote in both tables with
        // equal weight regardless of how much of it each material actually uses.
        MaterialAccum& accum = materials[material];
        for (const uint32_t vi : corner) {
            const glm::vec4& c = mesh.vertices[vi].color;
            if (!std::isfinite(c.r) || !std::isfinite(c.g) || !std::isfinite(c.b)) {
                continue;
            }
            accum.r += static_cast<double>(c.r);
            accum.g += static_cast<double>(c.g);
            accum.b += static_cast<double>(c.b);
            ++accum.samples;
        }

        ++chunk.triangles;
        ++routed;
    }

    return routed;
}

/// Mean vertex colour of a material, clamped, or mid grey when nothing was sampled
[[nodiscard]] glm::vec3 material_color(const MaterialTable& materials, uint32_t packed) {
    const auto it = materials.find(packed);
    if (it == materials.end() || it->second.samples == 0u) {
        return { 0.6f, 0.6f, 0.6f };
    }
    const double inv = 1.0 / static_cast<double>(it->second.samples);
    const auto channel = [inv](double sum) {
        return static_cast<float>(std::clamp(sum * inv, 0.0, 1.0));
    };
    return { channel(it->second.r), channel(it->second.g), channel(it->second.b) };
}

/// Concatenate a chunk's per-group index lists into one mesh with SubMesh ranges
[[nodiscard]] ChunkMesh finish_chunk(const ChunkAccum& chunk) {
    ChunkMesh out;
    out.mesh.vertices = chunk.vertices;

    size_t total = 0;
    for (const auto& [key, list] : chunk.by_group) {
        (void)key;
        total += list.size();
    }
    out.mesh.indices.reserve(total);

    for (const auto& [key, list] : chunk.by_group) {
        if (list.empty()) continue;

        const MaterialKey material = unpack_material_key(key.material);

        SubMesh range;
        range.index_offset = static_cast<uint32_t>(out.mesh.indices.size());
        range.index_count = static_cast<uint32_t>(list.size());
        range.material = material.material;
        range.variant = material.variant;
        out.mesh.submeshes.push_back(range);
        out.range_object.push_back(key.object);
        out.mesh.indices.insert(out.mesh.indices.end(), list.begin(), list.end());
    }

    out.mesh.compute_bounds();
    return out;
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
 * snprintf returns the length the output WOULD have had, not the length it wrote.
 * Handing that straight to TextWriter::put() reads off the end of the stack array
 * and appends whatever was next to it to the file -- undefined behaviour and a
 * corrupt line in one step. Everything formatted through a fixed buffer here is
 * numeric and bounded; every caller-supplied name is built as a std::string instead
 * precisely so it cannot be truncated into a directive that points at nothing.
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
// Metadata
// ============================================================================

/**
 * @brief The `# stratum:object` / `# stratum:meta` block for one object
 *
 * The grammar, which Track I's reader is expected to parse and which is therefore
 * written down rather than left to be inferred:
 *
 * @code
 *     # stratum:object index=<n> kind=<Kind> osm_id=<id> layer=<layer> name=<name>
 *     # stratum:meta <key>=<value>
 *     o <prefix>_o<n>_<name>
 * @endcode
 *
 * Fields on the object line are `key=value`, space separated, and `name` is LAST
 * because it is the one field that may legally contain a space -- so a reader takes
 * everything after `name=` to end of line and never has to guess where it stopped.
 * `layer` is omitted entirely when empty rather than written as `layer=`, which a
 * naive split would read as a key with an empty value and a name field that moved.
 *
 * A meta line carries exactly one pair, so a value may contain spaces and '=' and
 * still be recovered by splitting on the FIRST '='.
 *
 * @return The block, newline-terminated, or just the `o` line when metadata is off
 */
[[nodiscard]] std::string obj_object_block(const SceneObject& object, size_t index,
                                           const std::string& record_name,
                                           const SceneExportConfig& cfg) {
    std::string out;
    if (cfg.write_metadata) {
        out += "# stratum:object index=" + std::to_string(index)
             + " kind=" + scene_object_kind_name(object.kind)
             + " osm_id=" + std::to_string(object.osm_id);
        if (!object.layer.empty()) {
            out += " layer=" + sanitize_token(object.layer);
        }
        if (!object.name.empty()) {
            out += " name=" + sanitize_line(object.name);
        }
        out += "\n";

        for (const auto& [key, value] : object.metadata) {
            out += "# stratum:meta " + sanitize_meta_key(key) + "=" + sanitize_line(value) + "\n";
        }
    }
    out += "o " + record_name + "\n";
    return out;
}

/**
 * @brief The `extras` object glTF carries for one SceneObject
 *
 * Everything is nested under a single `stratum` key. glTF `extras` is a free-for-all
 * that any exporter in a pipeline may write into, so a flat `{"osm_id": ...}` is a
 * collision waiting for the second tool that touches the file.
 *
 * Values go in VERBATIM: JSON escapes newlines and quotes, so unlike the OBJ writer
 * this one has nothing to sanitise and nothing to lose.
 */
[[nodiscard]] nlohmann::json gltf_object_extras(const SceneObject& object, size_t index) {
    nlohmann::json info;
    info["index"] = index;
    info["kind"] = scene_object_kind_name(object.kind);
    info["osm_id"] = object.osm_id;
    if (!object.name.empty()) {
        info["name"] = object.name;
    }
    if (!object.layer.empty()) {
        info["layer"] = object.layer;
    }
    if (!object.metadata.empty()) {
        nlohmann::json meta = nlohmann::json::object();
        for (const auto& [key, value] : object.metadata) {
            meta[key] = value;
        }
        info["metadata"] = std::move(meta);
    }

    nlohmann::json extras;
    extras["stratum"] = std::move(info);
    return extras;
}

/// The object a range belongs to, or a placeholder when the index is somehow stale
[[nodiscard]] const SceneObject& range_owner(const std::vector<SceneObject>& objects,
                                             const ChunkMesh& chunk, size_t range) {
    static const SceneObject kUnknown{};
    if (range >= chunk.range_object.size()) {
        return kUnknown;
    }
    const uint32_t index = chunk.range_object[range];
    return index < objects.size() ? objects[index] : kUnknown;
}

// ============================================================================
// Wavefront OBJ
// ============================================================================

/**
 * @brief Write one chunk as an OBJ, one `o` record per object and one group per material
 *
 * Positions, texture coordinates and normals are written one per vertex and in the
 * same order, so a face triple is always `i/i/i`. Redundant by a few bytes per face,
 * and worth it: every OBJ importer in existence reads that without guessing.
 *
 * The whole chunk's vertices are written BEFORE the first `o` record. OBJ's vertex
 * list is a single file-global pool and `o` only partitions the faces that follow,
 * so this is the ordinary spelling and not a trick; writing a vertex block per
 * object instead would mean tracking a per-object index base, which is one more
 * place to be off by one for no gain.
 */
[[nodiscard]] bool write_obj_file(const ChunkMesh& chunk, const std::vector<SceneObject>& objects,
                                  const fs::path& obj_path, const std::string& mtllib_name,
                                  const std::string& prefix, const SceneExportConfig& cfg) {
    const Mesh& mesh = chunk.mesh;
    if (mesh.vertices.empty() || mesh.indices.size() < 3u) {
        return false;
    }

    TextWriter out(obj_path);
    if (!out.ok()) {
        spdlog::error("scene export: cannot open {} for writing", obj_path.string());
        return false;
    }

    char line[256];
    int n = 0;

    out.put("# Stratum scene export\n");
    // Built as a string, not formatted into `line`: the name comes from the caller
    // and a name longer than the buffer would be truncated into an mtllib directive
    // pointing at a file that does not exist.
    out.put("mtllib " + mtllib_name + "\n");

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

    const std::vector<SubMesh> ranges = mesh.effective_submeshes();

    // Ranges arrive in ascending (object, material) order, so an object's ranges are
    // contiguous and the record is opened exactly once per object. `has_open` rather
    // than comparing against index 0, because 0 is a real object index.
    bool has_open = false;
    uint32_t open_object = 0;

    for (size_t r = 0; r < ranges.size(); ++r) {
        const SubMesh& range = ranges[r];
        if (range.index_count < 3u) continue;

        const uint32_t owner_index =
            r < chunk.range_object.size() ? chunk.range_object[r] : 0u;
        if (!has_open || owner_index != open_object) {
            const SceneObject& owner = range_owner(objects, chunk, r);
            const std::string record = object_record_name(owner, owner_index, prefix);
            out.put(obj_object_block(owner, owner_index, record, cfg));
            has_open = true;
            open_object = owner_index;
        }

        const MaterialKey key{ range.material, range.variant };
        const std::string name = material_name(key, cfg);
        // Unbounded for the same reason the header lines are: this one carries both
        // the material prefix, whose documented job is to namespace materials into
        // another project, and the frozen material name.
        out.put("g " + object_record_name(range_owner(objects, chunk, r), owner_index, prefix)
                + "_" + road::material_key_name(key) + "\nusemtl " + name + "\n");

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
[[nodiscard]] bool write_mtl_file(const MaterialTable& materials, const fs::path& mtl_path,
                                  const SceneExportConfig& cfg) {
    TextWriter out(mtl_path);
    if (!out.ok()) {
        spdlog::error("scene export: cannot open {} for writing", mtl_path.string());
        return false;
    }

    out.put("# Stratum scene export material library\n");
    char line[256];
    for (const auto& [packed, accum] : materials) {
        (void)accum;
        const MaterialKey key = unpack_material_key(packed);
        const glm::vec3 base = material_color(materials, packed);
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

/// Growable byte buffer backing a glTF `.bin`
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

/// The glTF material for one MaterialKey
[[nodiscard]] nlohmann::json gltf_material_json(MaterialKey key, const glm::vec3& base,
                                                const SceneExportConfig& cfg) {
    nlohmann::json material;
    material["name"] = material_name(key, cfg);
    material["doubleSided"] = false;
    material["pbrMetallicRoughness"] = {
        { "baseColorFactor", { base.r, base.g, base.b, 1.0f } },
        { "metallicFactor", 0.0 },
        { "roughnessFactor", 0.9 }
    };
    return material;
}

/**
 * @brief Write one chunk as `.gltf` plus its `.bin`
 *
 * One buffer, one bufferView per attribute, one shared set of vertex accessors, and
 * one primitive per (object, material) range carrying that object's `extras`.
 *
 * Materials are DEDUPLICATED across the chunk: twenty buildings sharing the Wall
 * slot produce twenty primitives and one material, not twenty materials that a user
 * then has to retexture one by one. The road writer emits one per primitive, which
 * costs nothing there because a chunk holds a handful of ranges; here a chunk holds
 * one range per object per material and the difference is thousands of entries.
 *
 * glTF 2.0 requires Y up, so SceneExportConfig::y_up is ignored here; the caller
 * warns.
 */
[[nodiscard]] bool write_gltf_file(const ChunkMesh& chunk, const std::vector<SceneObject>& objects,
                                   const MaterialTable& materials, const fs::path& gltf_path,
                                   const fs::path& bin_path, const std::string& chunk_name,
                                   const SceneExportConfig& cfg) {
    const Mesh& mesh = chunk.mesh;
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
        // The bitangent sign is one of exactly two values; a 0 left by a producer
        // that never computed tangents is neither, and a shader reading it gets a
        // flat black normal map.
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
    accessors[static_cast<size_t>(acc_position)]["min"] = { min_pos.x, min_pos.y, min_pos.z };
    accessors[static_cast<size_t>(acc_position)]["max"] = { max_pos.x, max_pos.y, max_pos.z };

    const int acc_normal = add_accessor(view_normal, kGltfFloat, "VEC3", vertex_count, 0);
    const int acc_uv = add_accessor(view_uv, kGltfFloat, "VEC2", vertex_count, 0);
    const int acc_color = add_accessor(view_color, kGltfFloat, "VEC4", vertex_count, 0);
    const int acc_tangent = add_accessor(view_tangent, kGltfFloat, "VEC4", vertex_count, 0);

    nlohmann::json primitives = nlohmann::json::array();
    nlohmann::json material_array = nlohmann::json::array();
    std::map<uint32_t, int> material_index;   ///< packed MaterialKey -> index in `materials`

    const std::vector<SubMesh> ranges = mesh.effective_submeshes();
    for (size_t r = 0; r < ranges.size(); ++r) {
        const SubMesh& range = ranges[r];
        if (range.index_count < 3u) continue;
        if (static_cast<size_t>(range.index_offset) + range.index_count > mesh.indices.size()) {
            continue;
        }

        const MaterialKey key{ range.material, range.variant };
        const uint32_t packed = key.packed();
        auto slot = material_index.find(packed);
        if (slot == material_index.end()) {
            material_array.push_back(
                gltf_material_json(key, material_color(materials, packed), cfg));
            slot = material_index.emplace(packed,
                                          static_cast<int>(material_array.size()) - 1).first;
        }

        const int acc_index = add_accessor(view_index, kGltfUnsignedInt, "SCALAR",
                                           range.index_count,
                                           static_cast<size_t>(range.index_offset)
                                           * sizeof(uint32_t));

        nlohmann::json primitive;
        primitive["attributes"] = {
            { "POSITION", acc_position },
            { "NORMAL", acc_normal },
            { "TEXCOORD_0", acc_uv },
            { "COLOR_0", acc_color },
            { "TANGENT", acc_tangent }
        };
        primitive["indices"] = acc_index;
        primitive["material"] = slot->second;
        primitive["mode"] = kGltfTriangles;
        if (cfg.write_metadata) {
            const uint32_t owner_index =
                r < chunk.range_object.size() ? chunk.range_object[r] : 0u;
            primitive["extras"] = gltf_object_extras(range_owner(objects, chunk, r), owner_index);
        }
        primitives.push_back(primitive);
    }

    if (primitives.empty()) {
        return false;
    }

    nlohmann::json doc;
    doc["asset"] = { { "version", "2.0" }, { "generator", "Stratum scene exporter" } };
    doc["scene"] = 0;
    doc["scenes"] = nlohmann::json::array({ nlohmann::json{ { "nodes", { 0 } } } });
    doc["nodes"] = nlohmann::json::array({
        nlohmann::json{ { "mesh", 0 }, { "name", chunk_name } }
    });
    doc["meshes"] = nlohmann::json::array({
        nlohmann::json{ { "name", chunk_name }, { "primitives", std::move(primitives) } }
    });
    doc["materials"] = std::move(material_array);
    doc["accessors"] = std::move(accessors);
    doc["bufferViews"] = std::move(views);
    doc["buffers"] = nlohmann::json::array({
        nlohmann::json{ { "uri", bin_path.filename().string() },
                        { "byteLength", buffer.bytes().size() } }
    });

    // The buffer first, so a `.gltf` on disk always has its buffer beside it. A
    // reader that finds the JSON and no buffer cannot tell that from a truncated
    // export.
    {
        std::ofstream bin(bin_path, std::ios::binary | std::ios::trunc);
        if (!bin.is_open()) {
            spdlog::error("scene export: cannot open {} for writing", bin_path.string());
            return false;
        }
        bin.write(reinterpret_cast<const char*>(buffer.bytes().data()),
                  static_cast<std::streamsize>(buffer.bytes().size()));
        if (!bin.good()) {
            spdlog::error("scene export: failed while writing {}", bin_path.string());
            return false;
        }
    }
    {
        std::ofstream json_out(gltf_path, std::ios::binary | std::ios::trunc);
        if (!json_out.is_open()) {
            spdlog::error("scene export: cannot open {} for writing", gltf_path.string());
            return false;
        }
        const std::string text = doc.dump(2);
        json_out.write(text.data(), static_cast<std::streamsize>(text.size()));
        if (!json_out.good()) {
            spdlog::error("scene export: failed while writing {}", gltf_path.string());
            return false;
        }
    }

    return true;
}

// ============================================================================
// Shared plumbing
// ============================================================================

/// File extension of a format, dot included
[[nodiscard]] const char* format_extension(SceneExportFormat format) {
    return format == SceneExportFormat::Gltf ? ".gltf" : ".obj";
}

/// `<prefix>_<cx>_<cz>` when chunked, `<prefix>` when not
[[nodiscard]] std::string chunk_stem(const ChunkKey& key, const std::string& prefix, bool chunked) {
    if (!chunked) return prefix;
    return prefix + "_" + std::to_string(key.x) + "_" + std::to_string(key.z);
}

/// Absolute form of a path, falling back to the path itself when it cannot be taken
[[nodiscard]] std::string absolute_string(const fs::path& path) {
    std::error_code ec;
    const fs::path abs = fs::absolute(path, ec);
    return ec ? path.string() : abs.string();
}

/// Fix up a config whose options the chosen format cannot honour
[[nodiscard]] SceneExportConfig effective_config(const SceneExportConfig& cfg, const char* who) {
    SceneExportConfig out = cfg;
    if (out.format == SceneExportFormat::Gltf && !out.y_up) {
        spdlog::warn("{}: glTF 2.0 requires Y up; ignoring y_up = false", who);
        out.y_up = true;
    }
    return out;
}

/**
 * @brief Write one chunk in the configured format and record what was produced
 *
 * @return true when the geometry file (and its `.bin`, for glTF) was written
 */
bool write_chunk_files(const ChunkMesh& chunk, const std::vector<SceneObject>& objects,
                       const MaterialTable& materials, const fs::path& geometry_path,
                       const fs::path& bin_path, const std::string& mtllib_name,
                       const std::string& chunk_name, const std::string& prefix,
                       const SceneExportConfig& cfg, SceneExportStats& stats) {
    if (cfg.format == SceneExportFormat::Gltf) {
        if (!write_gltf_file(chunk, objects, materials, geometry_path, bin_path,
                             chunk_name, cfg)) {
            return false;
        }
        stats.written_files.push_back(absolute_string(geometry_path));
        stats.written_files.push_back(absolute_string(bin_path));
        stats.files += 2;
        return true;
    }

    if (!write_obj_file(chunk, objects, geometry_path, mtllib_name, prefix, cfg)) {
        return false;
    }
    stats.written_files.push_back(absolute_string(geometry_path));
    stats.files += 1;
    return true;
}

/// Create @p dir if missing and report whether it is usable
[[nodiscard]] bool ensure_directory(const fs::path& dir, const char* who) {
    if (dir.empty()) {
        return true;
    }
    std::error_code ec;
    if (!fs::exists(dir, ec)) {
        fs::create_directories(dir, ec);
        if (ec) {
            spdlog::error("{}: cannot create {}: {}", who, dir.string(), ec.message());
            return false;
        }
    }
    if (!fs::is_directory(dir, ec)) {
        spdlog::error("{}: {} is not a directory", who, dir.string());
        return false;
    }
    return true;
}

} // namespace

// ============================================================================
// Kind names
// ============================================================================

const char* scene_object_kind_name(SceneObjectKind kind) {
    switch (kind) {
        case SceneObjectKind::Unknown:  return "Unknown";
        case SceneObjectKind::Building: return "Building";
        case SceneObjectKind::Area:     return "Area";
        case SceneObjectKind::Road:     return "Road";
        case SceneObjectKind::Terrain:  return "Terrain";
        case SceneObjectKind::Count:    break;
    }
    return "Unknown";
}

// ============================================================================
// Entry points
// ============================================================================

SceneExportStats export_scene(const std::vector<SceneObject>& objects,
                              const fs::path& out_dir,
                              const SceneExportConfig& cfg) {
    const auto start = std::chrono::steady_clock::now();
    SceneExportStats stats;

    const auto finish = [&stats, start]() -> SceneExportStats& {
        const auto end = std::chrono::steady_clock::now();
        stats.export_ms = std::chrono::duration<double, std::milli>(end - start).count();
        return stats;
    };

    if (objects.empty()) {
        return finish();
    }

    const SceneExportConfig effective = effective_config(cfg, "export_scene");
    const float chunk_size = effective.chunk_size > 0.0f ? effective.chunk_size : 0.0f;
    const bool chunked = chunk_size > 0.0f;
    const std::string prefix = effective_prefix(effective);

    try {
        ChunkMap chunks;
        MaterialTable materials;

        for (size_t i = 0; i < objects.size(); ++i) {
            const size_t routed = accumulate_object(chunks, objects[i], static_cast<uint32_t>(i),
                                                    chunk_size, materials,
                                                    stats.dropped_triangles);
            if (routed > 0) {
                ++stats.objects;
                stats.triangles += routed;
            }
        }

        if (chunks.empty()) {
            return finish();
        }

        if (!ensure_directory(out_dir, "export_scene")) {
            return finish();
        }

        const std::string mtllib_name = prefix + ".mtl";
        bool mtl_written = false;

        // std::map, so this walks the cells in ascending x then z -- the order
        // SceneExportStats::written_files is documented to be in.
        for (const auto& [key, accum] : chunks) {
            const ChunkMesh chunk = finish_chunk(accum);
            const std::string stem = chunk_stem(key, prefix, chunked);
            const fs::path geometry_path = out_dir / (stem + format_extension(effective.format));
            const fs::path bin_path = out_dir / (stem + ".bin");

            if (!write_chunk_files(chunk, objects, materials, geometry_path, bin_path,
                                   mtllib_name, stem, prefix, effective, stats)) {
                continue;
            }
            ++stats.chunks;
            stats.vertices += chunk.mesh.vertices.size();

            // The MTL is shared by every OBJ of the export, so it is written once,
            // as the first successful chunk's sidecar. Written after that chunk and
            // not before the loop so that an export whose every file failed to open
            // leaves no orphan material library behind.
            if (effective.format == SceneExportFormat::Obj && !mtl_written) {
                const fs::path mtl_path = out_dir / mtllib_name;
                if (write_mtl_file(materials, mtl_path, effective)) {
                    stats.written_files.push_back(absolute_string(mtl_path));
                    stats.files += 1;
                }
                mtl_written = true;
            }
        }
    } catch (const std::exception& e) {
        spdlog::error("export_scene: aborted after {} files: {}", stats.files, e.what());
        return finish();
    }

    finish();
    spdlog::info("export_scene: {} chunks, {} objects, {} triangles ({} dropped), "
                 "{} files in {:.1f} ms",
                 stats.chunks, stats.objects, stats.triangles, stats.dropped_triangles,
                 stats.files, stats.export_ms);
    return stats;
}

bool export_scene_object(const SceneObject& object, const fs::path& out_path,
                         const SceneExportConfig& cfg) {
    if (object.mesh == nullptr || object.mesh->vertices.empty()
        || object.mesh->indices.size() < 3u) {
        return false;
    }

    const SceneExportConfig effective = effective_config(cfg, "export_scene_object");

    try {
        if (!ensure_directory(out_path.parent_path(), "export_scene_object")) {
            return false;
        }

        // Routed through exactly the same accumulator as a full export, with
        // chunking off. Writing the mesh out directly instead would be a second
        // path that could disagree with the chunked one about grouping, material
        // ranges or metadata -- and it is the single-object path the golden tests
        // look at, so a disagreement would hide in the half nobody inspects.
        const std::vector<SceneObject> one{ object };
        ChunkMap chunks;
        MaterialTable materials;
        size_t dropped = 0;
        if (accumulate_object(chunks, one[0], 0u, 0.0f, materials, dropped) == 0u) {
            return false;
        }

        const auto it = chunks.begin();
        if (it == chunks.end()) {
            return false;
        }
        const ChunkMesh chunk = finish_chunk(it->second);
        const std::string stem = out_path.stem().string();
        const std::string prefix = effective_prefix(effective);

        if (effective.format == SceneExportFormat::Gltf) {
            fs::path bin_path = out_path;
            bin_path.replace_extension(".bin");
            return write_gltf_file(chunk, one, materials, out_path, bin_path, stem, effective);
        }

        fs::path mtl_path = out_path;
        mtl_path.replace_extension(".mtl");
        const bool mtl_ok = write_mtl_file(materials, mtl_path, effective);
        const bool obj_ok = write_obj_file(chunk, one, out_path,
                                           mtl_path.filename().string(), prefix, effective);
        return obj_ok && mtl_ok;
    } catch (const std::exception& e) {
        spdlog::error("export_scene_object: {} failed: {}", out_path.string(), e.what());
        return false;
    }
}

// ============================================================================
// Adapters from the OSM types
// ============================================================================

SceneObject describe_building(const Building& building, const Mesh& mesh) {
    SceneObject out;
    out.mesh = &mesh;
    out.kind = SceneObjectKind::Building;
    out.name = building.name;
    out.osm_id = building.osm_id;
    out.metadata.emplace_back("building_type", building_type_name(building.type));
    // Formatted rather than streamed: std::to_string on a float gives six decimals
    // of a value that is metres, and "10.000000" in every building record is noise
    // in a diff of two exports.
    char number[32];
    const int n = std::snprintf(number, sizeof(number), "%.2f",
                                static_cast<double>(building.height));
    out.metadata.emplace_back("height_m", std::string(number, n > 0 ? static_cast<size_t>(n) : 0u));
    out.metadata.emplace_back("levels", std::to_string(building.levels));
    return out;
}

SceneObject describe_area(const Area& area, const Mesh& mesh) {
    SceneObject out;
    out.mesh = &mesh;
    out.kind = SceneObjectKind::Area;
    out.name = area.name;
    out.osm_id = area.osm_id;
    out.metadata.emplace_back("area_type", area_type_name(area.type));
    return out;
}

} // namespace stratum::osm
