// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_scene_export.cpp
 * @brief Whole-scene export: every triangle written once, in one file, with its object
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The road exporter exists because `TileManager` used to copy a whole road into
 * every tile its polyline touched and extrude it in full: a road crossing four
 * tiles was written out four times, and a consumer loading two adjacent tiles drew
 * the same carriageway twice, z-fighting with itself. tests/road/test_road_export.cpp
 * asserts that this cannot come back for roads.
 *
 * Generalising the exporter to buildings, areas and terrain is the moment it can
 * come back for everything else, so this suite asserts the same property across
 * mesh kinds -- and asserts it harder, because a sum can balance while being
 * wrong:
 *
 *  - The conservation test compares the MULTISET of triangle centroids recovered
 *    from the files against the multiset the input contained. A sum going up by one
 *    and down by one somewhere else passes an equality of counts and fails this.
 *  - Every recovered triangle must sit in the cell its file name claims, which is
 *    what catches an implementation that put everything in one chunk, or that
 *    truncated a negative cell index towards zero instead of flooring it.
 *  - The fixture itself is checked: at least one object must contribute to two
 *    different chunk files. Without that, "no object was duplicated" is a statement
 *    about a scene where nothing straddled a boundary, which is no statement at all.
 *
 * Everything is read back off DISK. A count reported by SceneExportStats is the
 * exporter marking its own homework.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests SceneExport
 * @endcode
 */

#include "framework.hpp"

#include "osm/scene_export.hpp"
#include "osm/road/road_style.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <nlohmann/json.hpp>

#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace {

namespace fs = std::filesystem;

using stratum::MaterialId;
using stratum::MaterialKey;
using stratum::Mesh;
using stratum::SubMesh;
using stratum::Vertex;
using stratum::osm::SceneExportConfig;
using stratum::osm::SceneExportFormat;
using stratum::osm::SceneExportStats;
using stratum::osm::SceneObject;
using stratum::osm::SceneObjectKind;
using stratum::osm::describe_area;
using stratum::osm::describe_building;
using stratum::osm::export_scene;
using stratum::osm::export_scene_object;

/// Cell size used by every chunking test, metres
constexpr float kChunk = 50.0f;

// ============================================================================
// Scratch space
// ============================================================================

/**
 * @brief An empty directory to export into
 *
 * Under the BUILD tree via STRATUM_TEST_DUMP_DIR, which tests/CMakeLists.txt
 * defines for this target: an export is regenerated output and must never be
 * mistaken for a committed fixture. The fallback exists only so the suite can be
 * compiled and run standalone with a direct g++ invocation, which is how it was
 * developed.
 */
fs::path scratch_dir(const std::string& name) {
#ifdef STRATUM_TEST_DUMP_DIR
    const fs::path base = fs::path(STRATUM_TEST_DUMP_DIR);
#else
    const fs::path base = fs::temp_directory_path() / "stratum_test_dump";
#endif
    const fs::path dir = base / "scene_export" / name;
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);
    return dir;
}

// ============================================================================
// Geometry fixtures
// ============================================================================

/**
 * @brief An axis-aligned quad on the XZ plane, two triangles, one material
 *
 * Corners are (x0,z0) (x1,z0) (x1,z1) (x0,z1) and the split is 0-1-2 / 0-2-3, so
 * the two triangle centroids sit on opposite sides of the quad's diagonal. That is
 * what makes a quad straddling a cell boundary land in two different cells, which
 * is the case the whole suite is about.
 */
Mesh make_quad(float x0, float z0, float x1, float z1, float y,
               const glm::vec4& color, MaterialId material) {
    Mesh mesh;
    const glm::vec3 corners[4] = {
        { x0, y, z0 }, { x1, y, z0 }, { x1, y, z1 }, { x0, y, z1 }
    };
    for (size_t i = 0; i < 4u; ++i) {
        const glm::vec3& p = corners[i];
        Vertex v;
        v.position = p;
        v.normal = { 0.0f, 1.0f, 0.0f };
        v.uv = { p.x * 0.1f, p.z * 0.1f };
        v.color = color;
        // The bitangent sign is deliberately NEITHER +1 nor -1. Both writers snap it
        // to one or the other, and a fixture that only ever supplies a value already
        // snapped cannot tell a writer that snaps from one that passes the raw float
        // through -- which is a flat black normal map in any consumer that reads it.
        // 0 is what a producer that never computed tangents leaves behind.
        v.tangent = { 1.0f, 0.0f, 0.0f, (i % 2u == 0u) ? 0.0f : -0.5f };
        mesh.vertices.push_back(v);
    }
    mesh.indices = { 0, 1, 2, 0, 2, 3 };
    mesh.submeshes.push_back(SubMesh{ 0u, 6u, material, 0u });
    mesh.compute_bounds();
    return mesh;
}

/// A grid of @p nx by @p nz quads, each @p step metres, anchored at (x0, z0)
Mesh make_grid(float x0, float z0, int nx, int nz, float step,
               const glm::vec4& color, MaterialId material) {
    Mesh mesh;
    for (int ix = 0; ix < nx; ++ix) {
        for (int iz = 0; iz < nz; ++iz) {
            const float qx = x0 + static_cast<float>(ix) * step;
            const float qz = z0 + static_cast<float>(iz) * step;
            const Mesh quad = make_quad(qx, qz, qx + step, qz + step, 0.0f, color, material);
            mesh.append(quad, material);
        }
    }
    mesh.compute_bounds();
    return mesh;
}

/// Triangles in a mesh
size_t triangle_count(const Mesh& mesh) { return mesh.indices.size() / 3u; }

/**
 * @brief A scene and the meshes it borrows
 *
 * SceneObject holds a NON-OWNING Mesh pointer, so the meshes have to outlive the
 * objects and must not move. std::unique_ptr and not std::vector<Mesh>: a vector
 * reallocates as it grows and every pointer taken into it before that point dangles,
 * which is a use-after-free that shows up as a corrupt export rather than a crash.
 */
struct Scene {
    std::vector<std::unique_ptr<Mesh>> meshes;
    std::vector<SceneObject> objects;

    Mesh& add(Mesh mesh) {
        meshes.push_back(std::make_unique<Mesh>(std::move(mesh)));
        return *meshes.back();
    }

    size_t triangles() const {
        size_t n = 0;
        for (const SceneObject& object : objects) {
            if (object.mesh != nullptr) n += triangle_count(*object.mesh);
        }
        return n;
    }
};

/**
 * @brief A mixed scene: terrain, buildings, an area and a road, straddling cells
 *
 * Coordinates are chosen so that NO triangle centroid falls exactly on a 50 m cell
 * boundary. A centroid on the line is a genuine tie whose side depends on the last
 * bit of a float, and a test that happens to land on one fails for a reason that
 * has nothing to do with the exporter.
 *
 * The negative quadrant is deliberate. The world is Y up and the 2D-to-3D mapping
 * negates the second local coordinate, so most real geometry sits at negative Z;
 * an implementation that truncates a cell index towards zero instead of flooring it
 * merges cell -1 into cell 0 and is only visible there.
 */
Scene make_city() {
    Scene scene;

    // Terrain: 6 by 6 cells of 25 m from the origin, covering chunks (0,0) to (2,2).
    //
    // TWO materials over one index buffer, and that is load-bearing. Every other
    // object here carries exactly one, and with a one-material scene "the OBJ writer
    // opens one `o` record per object" is forced by the fixture rather than by the
    // writer's run logic -- a writer that opened a record per RANGE would be
    // indistinguishable from a correct one. Splitting the terrain in two is the
    // cheapest way to make that difference visible in every test that counts `o`
    // records. Triangle count, centroids and chunk assignment are all unchanged.
    {
        Mesh mesh = make_grid(0.0f, 0.0f, 6, 6, 25.0f,
                              { 0.3f, 0.5f, 0.2f, 1.0f }, MaterialId::Grass);
        const auto indices = static_cast<uint32_t>(mesh.indices.size());
        const uint32_t half = (indices / 6u) * 3u;   // a triangle boundary, not a byte one
        mesh.submeshes.clear();
        mesh.submeshes.push_back(SubMesh{ 0u, half, MaterialId::Grass, 0u });
        mesh.submeshes.push_back(SubMesh{ half, indices - half, MaterialId::Gravel, 0u });

        SceneObject object;
        object.mesh = &scene.add(std::move(mesh));
        object.kind = SceneObjectKind::Terrain;
        object.name = "tile_0_0";
        object.layer = "terrain";
        scene.objects.push_back(object);
    }

    // A building straddling x = 50: one triangle each side.
    {
        Mesh mesh = make_quad(45.0f, 10.0f, 55.0f, 20.0f, 6.0f,
                              { 0.7f, 0.6f, 0.5f, 1.0f }, MaterialId::Wall);
        SceneObject object;
        object.mesh = &scene.add(std::move(mesh));
        object.kind = SceneObjectKind::Building;
        object.name = "Town Hall";
        object.osm_id = 4242;
        object.layer = "buildings";
        object.metadata.emplace_back("height_m", "18.00");
        scene.objects.push_back(object);
    }

    // A building straddling z = 50 as well, so both axes are exercised.
    {
        Mesh mesh = make_quad(10.0f, 45.0f, 20.0f, 55.0f, 9.0f,
                              { 0.7f, 0.6f, 0.5f, 1.0f }, MaterialId::Wall);
        SceneObject object;
        object.mesh = &scene.add(std::move(mesh));
        object.kind = SceneObjectKind::Building;
        object.name = "Church";
        object.osm_id = 77;
        object.layer = "buildings";
        scene.objects.push_back(object);
    }

    // A second "Church": same name, different object. The exported record names
    // must still tell them apart.
    {
        Mesh mesh = make_quad(120.0f, 120.0f, 130.0f, 130.0f, 4.0f,
                              { 0.7f, 0.6f, 0.5f, 1.0f }, MaterialId::Wall);
        SceneObject object;
        object.mesh = &scene.add(std::move(mesh));
        object.kind = SceneObjectKind::Building;
        object.name = "Church";
        object.osm_id = 78;
        object.layer = "buildings";
        scene.objects.push_back(object);
    }

    // An area in the negative quadrant, crossing x = -50 and z = -50.
    {
        Mesh mesh = make_quad(-55.0f, -55.0f, -45.0f, -45.0f, 0.0f,
                              { 0.2f, 0.4f, 0.8f, 1.0f }, MaterialId::Grass);
        SceneObject object;
        object.mesh = &scene.add(std::move(mesh));
        object.kind = SceneObjectKind::Area;
        object.name = "Duck Pond";
        object.osm_id = 9001;
        object.layer = "areas";
        scene.objects.push_back(object);
    }

    // A road-shaped strip: long enough to cross two boundaries on its own, which is
    // exactly the geometry the old TileManager duplicated.
    {
        Mesh mesh = make_grid(-20.0f, -10.0f, 8, 1, 20.0f,
                              { 0.2f, 0.2f, 0.2f, 1.0f }, MaterialId::Asphalt);
        SceneObject object;
        object.mesh = &scene.add(std::move(mesh));
        object.kind = SceneObjectKind::Road;
        object.name = "High Street";
        object.osm_id = 12345;
        object.layer = "roads";
        scene.objects.push_back(object);
    }

    return scene;
}

// ============================================================================
// Triangle identity
// ============================================================================

/**
 * @brief A triangle's centroid, quantised, so two exports can be compared as sets
 *
 * Centimetres. The OBJ writer prints positions to four decimals, so a centroid
 * recovered from a file differs from the one computed in memory by at most a
 * ten-thousandth of a metre; quantising to a centimetre is two orders of magnitude
 * clear of that, and every triangle in the fixtures is metres away from its
 * neighbours, so distinct triangles cannot collide onto one key.
 */
using TriKey = std::array<long long, 3>;

TriKey quantise(const glm::vec3& p) {
    const auto q = [](float v) {
        return static_cast<long long>(std::llround(static_cast<double>(v) * 100.0));
    };
    return TriKey{ q(p.x), q(p.y), q(p.z) };
}

/// Centroids of every triangle of every object, as a multiset
std::multiset<TriKey> input_centroids(const Scene& scene) {
    std::multiset<TriKey> out;
    for (const SceneObject& object : scene.objects) {
        if (object.mesh == nullptr) continue;
        const Mesh& mesh = *object.mesh;
        for (size_t t = 0; t + 2u < mesh.indices.size(); t += 3u) {
            const glm::vec3 c = (mesh.vertices[mesh.indices[t]].position
                               + mesh.vertices[mesh.indices[t + 1]].position
                               + mesh.vertices[mesh.indices[t + 2]].position) / 3.0f;
            out.insert(quantise(c));
        }
    }
    return out;
}

// ============================================================================
// OBJ reader
// ============================================================================

/// One triangle read back, with the object record and material group it fell under
struct ObjFace {
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t c = 0;
    std::string object;
    std::string material;
};

/// What read_obj() recovered
struct ObjFile {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> uvs;
    std::vector<ObjFace> faces;
    std::vector<std::string> objects;       ///< `o` names, in file order
    std::vector<std::string> materials;     ///< `usemtl` names, first appearance order
    std::vector<std::string> comments;      ///< comment lines, '#' stripped, in file order
    std::string mtllib;
    bool ok = false;
};

/**
 * @brief Read an OBJ back
 *
 * Tolerant about how the file is spelled -- `f a/b/c`, `f a//c` and `f a` all read
 * as the same triangle -- so a failure in this suite is a failure of WHAT was
 * written and never of HOW. Polygons with more than three corners are fanned, which
 * the writer never emits but a reader that silently dropped them would hide.
 */
ObjFile read_obj(const fs::path& path) {
    ObjFile out;
    std::ifstream in(path);
    if (!in.is_open()) {
        return out;
    }

    std::string line;
    std::string object;
    std::string material;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;

        if (line[0] == '#') {
            std::string text = line.substr(1);
            while (!text.empty() && text.front() == ' ') text.erase(text.begin());
            out.comments.push_back(text);
            continue;
        }

        std::istringstream stream(line);
        std::string tag;
        stream >> tag;

        if (tag == "v") {
            glm::vec3 p{ 0.0f };
            stream >> p.x >> p.y >> p.z;
            out.positions.push_back(p);
        } else if (tag == "vn") {
            glm::vec3 n{ 0.0f };
            stream >> n.x >> n.y >> n.z;
            out.normals.push_back(n);
        } else if (tag == "vt") {
            // Read so that a writer which swapped u and v can be caught. The suite
            // used to parse v and vn only, so the `vt` block was written into every
            // file and inspected by nothing.
            glm::vec2 t{ 0.0f };
            stream >> t.x >> t.y;
            out.uvs.push_back(t);
        } else if (tag == "o") {
            std::string name;
            stream >> name;
            object = name;
            out.objects.push_back(name);
        } else if (tag == "usemtl") {
            std::string name;
            stream >> name;
            material = name;
            if (std::find(out.materials.begin(), out.materials.end(), name)
                == out.materials.end()) {
                out.materials.push_back(name);
            }
        } else if (tag == "mtllib") {
            stream >> out.mtllib;
        } else if (tag == "f") {
            std::vector<uint32_t> corners;
            std::string token;
            while (stream >> token) {
                const size_t slash = token.find('/');
                const std::string first = slash == std::string::npos ? token
                                                                     : token.substr(0, slash);
                const long value = std::strtol(first.c_str(), nullptr, 10);
                if (value <= 0) continue;   // negative (relative) indices are not written
                corners.push_back(static_cast<uint32_t>(value - 1));
            }
            for (size_t i = 2; i < corners.size(); ++i) {
                out.faces.push_back(ObjFace{ corners[0], corners[i - 1], corners[i],
                                             object, material });
            }
        }
    }

    out.ok = true;
    return out;
}

/// Centroid of one recovered face, or false when an index is out of range
bool face_centroid(const ObjFile& obj, const ObjFace& face, glm::vec3& out) {
    const size_t n = obj.positions.size();
    if (face.a >= n || face.b >= n || face.c >= n) {
        return false;
    }
    out = (obj.positions[face.a] + obj.positions[face.b] + obj.positions[face.c]) / 3.0f;
    return true;
}

/// A chunk file and the grid cell its name claims
struct ChunkFile {
    fs::path path;
    long cx = 0;
    long cz = 0;
};

/**
 * @brief Chunk files of one export, with their grid coordinates parsed
 *
 * Matches `<prefix>_<cx>_<cz><ext>` and nothing else, so a sidecar such as the MTL
 * is excluded -- counting one into a triangle sum would make a correct export look
 * like a duplicating one.
 */
std::vector<ChunkFile> chunk_files(const fs::path& dir, const std::string& prefix,
                                   const std::string& extension) {
    std::vector<ChunkFile> out;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() != extension) continue;

        const std::string stem = entry.path().stem().string();
        if (stem.size() <= prefix.size() + 1u) continue;
        if (stem.compare(0, prefix.size(), prefix) != 0) continue;
        if (stem[prefix.size()] != '_') continue;

        const std::string coords = stem.substr(prefix.size() + 1u);
        const size_t split = coords.find('_', 1);
        if (split == std::string::npos) continue;

        char* end = nullptr;
        const std::string first = coords.substr(0, split);
        const std::string second = coords.substr(split + 1);
        const long cx = std::strtol(first.c_str(), &end, 10);
        if (end == nullptr || *end != '\0') continue;
        const long cz = std::strtol(second.c_str(), &end, 10);
        if (end == nullptr || *end != '\0') continue;

        out.push_back(ChunkFile{ entry.path(), cx, cz });
    }
    std::sort(out.begin(), out.end(), [](const ChunkFile& a, const ChunkFile& b) {
        return a.path.filename().string() < b.path.filename().string();
    });
    return out;
}

// ============================================================================
// glTF reader
// ============================================================================

/// A parsed `.gltf` and the `.bin` its buffer names
struct GltfFile {
    nlohmann::json doc;
    std::vector<uint8_t> bin;
    bool ok = false;
};

GltfFile read_gltf(const fs::path& path) {
    GltfFile out;
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return out;
    try {
        in >> out.doc;
    } catch (const std::exception&) {
        return out;
    }

    if (!out.doc.contains("buffers") || out.doc["buffers"].empty()) return out;
    const std::string uri = out.doc["buffers"][0].value("uri", std::string{});
    if (uri.empty()) return out;

    std::ifstream bin(path.parent_path() / uri, std::ios::binary);
    if (!bin.is_open()) return out;
    out.bin.assign(std::istreambuf_iterator<char>(bin), std::istreambuf_iterator<char>());

    // The declared byteLength has to match the file, or every offset below is being
    // read out of a buffer that is not the one the document describes.
    if (out.bin.size() != out.doc["buffers"][0].value("byteLength", size_t{ 0 })) return out;

    out.ok = true;
    return out;
}

/// First byte of an accessor's data in the buffer, or false when it does not fit
bool accessor_span(const GltfFile& gltf, size_t accessor_index, size_t element_bytes,
                   size_t& offset, size_t& count) {
    const auto& accessors = gltf.doc["accessors"];
    if (accessor_index >= accessors.size()) return false;
    const auto& accessor = accessors[accessor_index];
    if (!accessor.contains("bufferView")) return false;

    const size_t view_index = accessor["bufferView"].get<size_t>();
    const auto& views = gltf.doc["bufferViews"];
    if (view_index >= views.size()) return false;

    const size_t view_offset = views[view_index].value("byteOffset", size_t{ 0 });
    const size_t view_length = views[view_index].value("byteLength", size_t{ 0 });
    const size_t inner = accessor.value("byteOffset", size_t{ 0 });
    count = accessor.value("count", size_t{ 0 });

    if (inner + count * element_bytes > view_length) return false;
    offset = view_offset + inner;
    return offset + count * element_bytes <= gltf.bin.size();
}

std::vector<glm::vec3> read_vec3(const GltfFile& gltf, size_t accessor_index) {
    std::vector<glm::vec3> out;
    size_t offset = 0;
    size_t count = 0;
    if (!accessor_span(gltf, accessor_index, sizeof(float) * 3u, offset, count)) return out;
    out.resize(count);
    std::memcpy(out.data(), gltf.bin.data() + offset, count * sizeof(float) * 3u);
    return out;
}

std::vector<glm::vec2> read_vec2(const GltfFile& gltf, size_t accessor_index) {
    std::vector<glm::vec2> out;
    size_t offset = 0;
    size_t count = 0;
    if (!accessor_span(gltf, accessor_index, sizeof(float) * 2u, offset, count)) return out;
    out.resize(count);
    std::memcpy(out.data(), gltf.bin.data() + offset, count * sizeof(float) * 2u);
    return out;
}

std::vector<glm::vec4> read_vec4(const GltfFile& gltf, size_t accessor_index) {
    std::vector<glm::vec4> out;
    size_t offset = 0;
    size_t count = 0;
    if (!accessor_span(gltf, accessor_index, sizeof(float) * 4u, offset, count)) return out;
    out.resize(count);
    std::memcpy(out.data(), gltf.bin.data() + offset, count * sizeof(float) * 4u);
    return out;
}

std::vector<uint32_t> read_indices(const GltfFile& gltf, size_t accessor_index) {
    std::vector<uint32_t> out;
    size_t offset = 0;
    size_t count = 0;
    if (!accessor_span(gltf, accessor_index, sizeof(uint32_t), offset, count)) return out;
    out.resize(count);
    std::memcpy(out.data(), gltf.bin.data() + offset, count * sizeof(uint32_t));
    return out;
}

/**
 * @brief The runs of (object record, material) the faces of a file fall under
 *
 * Consecutive faces sharing both keys collapse into one entry, so the result is the
 * GROUP STRUCTURE of the file rather than its face list. That is what makes the
 * grouping contract checkable: "object 0's ranges, then object 1's" is a statement
 * about this sequence and about nothing a face count can see.
 *
 * A group that is re-entered later in the file appears as a second run, which is
 * exactly the failure the contiguity assertions look for.
 */
std::vector<std::pair<std::string, std::string>> face_groups(const ObjFile& obj) {
    std::vector<std::pair<std::string, std::string>> out;
    for (const ObjFace& face : obj.faces) {
        if (out.empty() || out.back().first != face.object
            || out.back().second != face.material) {
            out.emplace_back(face.object, face.material);
        }
    }
    return out;
}

/// The object-record index `n` out of a `<prefix>_o<n>_<name>` record name, or -1
long record_index(const std::string& record) {
    const size_t marker = record.find("_o");
    if (marker == std::string::npos) return -1;
    const std::string digits = record.substr(marker + 2u);
    if (digits.empty() || std::isdigit(static_cast<unsigned char>(digits[0])) == 0) return -1;
    return std::strtol(digits.c_str(), nullptr, 10);
}

/// Every `# stratum:object` line of a file, in order
std::vector<std::string> object_comments(const ObjFile& obj) {
    std::vector<std::string> out;
    for (const std::string& comment : obj.comments) {
        if (comment.rfind("stratum:object ", 0) == 0) out.push_back(comment);
    }
    return out;
}

/// The first comment line starting with @p tag, or an empty string
std::string find_comment(const ObjFile& obj, const std::string& tag) {
    for (const std::string& comment : obj.comments) {
        if (comment.rfind(tag, 0) == 0) return comment;
    }
    return {};
}

/// Whole file as bytes, for comparing two exports
std::string file_bytes(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return {};
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

} // namespace

// ============================================================================
// Conservation
// ============================================================================

/**
 * THE CONSERVATION TEST, stated as a multiset rather than a sum.
 *
 * Every triangle of the input appears in exactly one chunk file. A duplicated
 * triangle adds a key; a dropped one removes a key; a triangle split at the
 * boundary replaces one key with two that are nowhere near it. A count alone
 * catches only the first two, and only when they do not cancel out.
 */
TEST(SceneExport, chunking_conserves_every_triangle_across_mesh_kinds) {
    const Scene scene = make_city();
    const std::multiset<TriKey> expected = input_centroids(scene);
    CHECK_EQ(expected.size(), scene.triangles());
    CHECK((size_t{ 0 }) < expected.size());

    const fs::path dir = scratch_dir("conservation");
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;

    const SceneExportStats stats = export_scene(scene.objects, dir, cfg);

    // stats.triangles counts what reached DISK, so these two together say "every
    // input triangle was written and no chunk was silently missing". Either one
    // alone is the exporter marking its own homework; the multiset comparison
    // against the files below is what actually proves it.
    CHECK_EQ(stats.triangles, scene.triangles());
    CHECK_EQ(stats.dropped_triangles, size_t{ 0 });
    CHECK_EQ(stats.unwritten_triangles, size_t{ 0 });
    CHECK_EQ(stats.objects, scene.objects.size());
    CHECK((size_t{ 1 }) < stats.chunks);
    CHECK_EQ(stats.written_files.size(), stats.files);

    const std::vector<ChunkFile> chunks = chunk_files(dir, "scene", ".obj");
    CHECK((size_t{ 1 }) < chunks.size());

    std::multiset<TriKey> recovered;
    for (const ChunkFile& chunk : chunks) {
        const ObjFile obj = read_obj(chunk.path);
        CHECK_TRUE(obj.ok);
        for (const ObjFace& face : obj.faces) {
            glm::vec3 centroid{ 0.0f };
            CHECK_TRUE(face_centroid(obj, face, centroid));
            recovered.insert(quantise(centroid));
        }
    }

    CHECK_EQ(recovered.size(), expected.size());
    if (recovered != expected) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "the chunk files hold exactly the input triangles",
            std::to_string(recovered.size()) + " triangles across "
                + std::to_string(chunks.size()) + " files, input had "
                + std::to_string(expected.size()));
    }
}

/**
 * The fixture actually exercises straddling.
 *
 * Without this, the conservation test above is a statement about a scene in which
 * nothing crossed a boundary, and it would pass against an exporter that routed
 * each object whole into the cell containing its centre -- which is the bug.
 */
TEST(SceneExport, an_object_that_straddles_a_boundary_reaches_both_chunks) {
    const Scene scene = make_city();
    const fs::path dir = scratch_dir("straddle");
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;

    export_scene(scene.objects, dir, cfg);

    // Object record name -> the set of chunk files it appears in.
    std::map<std::string, std::set<std::string>> appearances;
    for (const ChunkFile& chunk : chunk_files(dir, "scene", ".obj")) {
        const ObjFile obj = read_obj(chunk.path);
        for (const std::string& name : obj.objects) {
            appearances[name].insert(chunk.path.filename().string());
        }
    }

    size_t split_objects = 0;
    for (const auto& [name, files] : appearances) {
        (void)name;
        if (files.size() > 1u) ++split_objects;
    }
    CHECK((size_t{ 0 }) < split_objects);

    // And the two halves of the 45..55 building really are one triangle each, so
    // "it reached both chunks" is not one triangle in each of two copies.
    size_t town_hall_faces = 0;
    for (const ChunkFile& chunk : chunk_files(dir, "scene", ".obj")) {
        const ObjFile obj = read_obj(chunk.path);
        for (const ObjFace& face : obj.faces) {
            if (face.object.find("Town_Hall") != std::string::npos) ++town_hall_faces;
        }
    }
    CHECK_EQ(town_hall_faces, size_t{ 2 });
}

/**
 * Every triangle sits in the cell its file name claims.
 *
 * The negative case is the one that fails silently: a cell index taken by
 * truncating towards zero puts everything in (-50 .. 0) into cell 0 alongside
 * (0 .. 50), so two cells' geometry lands in one file and the neighbour is short.
 * The totals still balance, so the conservation test cannot see it.
 */
TEST(SceneExport, every_triangle_lands_in_the_chunk_its_name_claims) {
    const Scene scene = make_city();
    const fs::path dir = scratch_dir("cells");
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;

    export_scene(scene.objects, dir, cfg);

    const std::vector<ChunkFile> chunks = chunk_files(dir, "scene", ".obj");
    CHECK_FALSE(chunks.empty());

    bool saw_negative_cell = false;
    size_t misplaced = 0;
    std::string first_bad;

    for (const ChunkFile& chunk : chunks) {
        if (chunk.cx < 0 || chunk.cz < 0) saw_negative_cell = true;
        const ObjFile obj = read_obj(chunk.path);
        for (const ObjFace& face : obj.faces) {
            glm::vec3 centroid{ 0.0f };
            if (!face_centroid(obj, face, centroid)) continue;
            const long cx = static_cast<long>(std::floor(centroid.x / kChunk));
            const long cz = static_cast<long>(std::floor(centroid.z / kChunk));
            if (cx != chunk.cx || cz != chunk.cz) {
                ++misplaced;
                if (first_bad.empty()) {
                    first_bad = chunk.path.filename().string() + " holds a centroid in cell ("
                              + std::to_string(cx) + ", " + std::to_string(cz) + ")";
                }
            }
        }
    }

    CHECK_TRUE(saw_negative_cell);
    CHECK_EQ(misplaced, size_t{ 0 });
    if (misplaced != 0) {
        stratum::test::report_failure(__FILE__, __LINE__,
                                      "every triangle is in its centroid's cell", first_bad);
    }
}

/// Chunking off puts the whole scene in one file, and names it without coordinates.
TEST(SceneExport, unchunked_export_writes_one_file) {
    const Scene scene = make_city();
    const fs::path dir = scratch_dir("unchunked");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;

    const SceneExportStats stats = export_scene(scene.objects, dir, cfg);

    CHECK_EQ(stats.chunks, size_t{ 1 });
    CHECK_EQ(stats.triangles, scene.triangles());
    CHECK_EQ(stats.unwritten_triangles, size_t{ 0 });
    CHECK_TRUE(fs::exists(dir / "scene.obj"));
    CHECK_TRUE(fs::exists(dir / "scene.mtl"));
    CHECK_TRUE(chunk_files(dir, "scene", ".obj").empty());

    const ObjFile obj = read_obj(dir / "scene.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.faces.size(), scene.triangles());
    CHECK_EQ(obj.mtllib, std::string{ "scene.mtl" });

    // One `o` record per contributing object, and no object opened twice.
    //
    // This only says anything because make_city()'s terrain carries TWO materials:
    // it contributes two ranges to this file, so a writer that opened a record per
    // range rather than per object emits seven records here instead of six. With one
    // material per object the count was forced by the fixture and the run logic in
    // write_obj_file() was unasserted.
    CHECK_EQ(obj.objects.size(), scene.objects.size());
    const std::set<std::string> unique(obj.objects.begin(), obj.objects.end());
    CHECK_EQ(unique.size(), obj.objects.size());

    // And the records are in ascending object order, each opened once, with that
    // object's material groups contiguous underneath it.
    const std::vector<std::pair<std::string, std::string>> groups = face_groups(obj);
    std::set<std::string> seen;
    std::string current;
    long previous_index = -1;
    size_t reopened = 0;
    size_t out_of_order = 0;
    for (const auto& [record, material] : groups) {
        (void)material;
        if (record == current) continue;
        if (!seen.insert(record).second) ++reopened;
        const long index = record_index(record);
        if (!(previous_index < index)) ++out_of_order;
        previous_index = index;
        current = record;
    }
    CHECK_EQ(reopened, size_t{ 0 });
    CHECK_EQ(out_of_order, size_t{ 0 });
    CHECK_EQ(seen.size(), scene.objects.size());
}

/// Nothing in, nothing out -- and no empty file left behind.
TEST(SceneExport, an_empty_scene_writes_nothing) {
    const fs::path dir = scratch_dir("empty");

    const SceneExportStats none = export_scene({}, dir, SceneExportConfig{});
    CHECK_EQ(none.files, size_t{ 0 });
    CHECK_EQ(none.triangles, size_t{ 0 });
    CHECK_EQ(none.chunks, size_t{ 0 });
    CHECK_TRUE(none.written_files.empty());

    // A null mesh and an empty mesh are skipped, not dereferenced and not written.
    Mesh empty;
    std::vector<SceneObject> objects(2);
    objects[0].mesh = nullptr;
    objects[1].mesh = &empty;

    const SceneExportStats skipped = export_scene(objects, dir, SceneExportConfig{});
    CHECK_EQ(skipped.files, size_t{ 0 });
    CHECK_EQ(skipped.objects, size_t{ 0 });

    size_t entries = 0;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        (void)entry;
        ++entries;
    }
    CHECK_EQ(entries, size_t{ 0 });
}

// ============================================================================
// Metadata
// ============================================================================

/// A building's identity reaches the OBJ, and two objects of the same name stay apart.
TEST(SceneExport, object_metadata_survives_into_obj) {
    const Scene scene = make_city();
    const fs::path dir = scratch_dir("obj_metadata");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;

    export_scene(scene.objects, dir, cfg);
    const ObjFile obj = read_obj(dir / "scene.obj");
    CHECK_TRUE(obj.ok);

    // The Town Hall is object 1 of the fixture.
    const std::string line = find_comment(obj, "stratum:object index=1 ");
    CHECK_FALSE(line.empty());
    CHECK((line.find("kind=Building")) != std::string::npos);
    CHECK((line.find("osm_id=4242")) != std::string::npos);
    CHECK((line.find("layer=buildings")) != std::string::npos);
    // name is last on the line precisely so a space in it survives.
    CHECK((line.find("name=Town Hall")) != std::string::npos);

    CHECK_FALSE(find_comment(obj, "stratum:meta height_m=18.00").empty());

    // One object line per object, and the record names are unique even though two
    // objects are both called "Church".
    CHECK_EQ(object_comments(obj).size(), scene.objects.size());
    size_t churches = 0;
    std::set<std::string> church_records;
    for (const std::string& name : obj.objects) {
        if (name.find("Church") != std::string::npos) {
            ++churches;
            church_records.insert(name);
        }
    }
    CHECK_EQ(churches, size_t{ 2 });
    CHECK_EQ(church_records.size(), size_t{ 2 });
}

/// The same identity reaches glTF, per primitive, under its own `extras` key.
TEST(SceneExport, object_metadata_survives_into_gltf) {
    const Scene scene = make_city();
    const fs::path dir = scratch_dir("gltf_metadata");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;
    cfg.format = SceneExportFormat::Gltf;

    export_scene(scene.objects, dir, cfg);

    const GltfFile gltf = read_gltf(dir / "scene.gltf");
    CHECK_TRUE(gltf.ok);
    if (!gltf.ok) return;

    // ONE PRIMITIVE PER (OBJECT, MATERIAL) GROUP, which is what the header promises
    // and not one per object: make_city()'s terrain carries two materials, so it
    // contributes two primitives to this file. Asserting one per object would have
    // been asserting a property of the fixture.
    const auto& primitives = gltf.doc["meshes"][0]["primitives"];
    CHECK_EQ(primitives.size(), scene.objects.size() + 1u);

    bool found_town_hall = false;
    std::set<int64_t> ids;
    std::map<long, size_t> primitives_per_object;
    std::vector<long> primitive_objects;
    for (const auto& primitive : primitives) {
        CHECK_TRUE(primitive.contains("extras"));
        if (!primitive.contains("extras")) continue;
        const auto& info = primitive["extras"]["stratum"];
        ids.insert(info.value("osm_id", int64_t{ -1 }));
        const long index = info.value("index", -1L);
        ++primitives_per_object[index];
        primitive_objects.push_back(index);
        if (info.value("osm_id", int64_t{ 0 }) == 4242) {
            found_town_hall = true;
            CHECK_EQ(info.value("kind", std::string{}), std::string{ "Building" });
            CHECK_EQ(info.value("name", std::string{}), std::string{ "Town Hall" });
            CHECK_EQ(info.value("layer", std::string{}), std::string{ "buildings" });
            CHECK_EQ(info["metadata"].value("height_m", std::string{}), std::string{ "18.00" });
        }
    }
    CHECK_TRUE(found_town_hall);
    // Both churches are present with their own ids, so a primitive is not reporting
    // the wrong object's metadata.
    CHECK((ids.count(77)) == size_t{ 1 });
    CHECK((ids.count(78)) == size_t{ 1 });

    // Every object contributed, and the two-material terrain contributed twice while
    // both of its primitives named the SAME object -- a primitive taking its extras
    // from its own position in the range list rather than from its range's owner
    // would give them different ones.
    CHECK_EQ(primitives_per_object.size(), scene.objects.size());
    CHECK_EQ(primitives_per_object[0], size_t{ 2 });

    // Primitives come out object-major, so an object's primitives are adjacent.
    size_t out_of_order = 0;
    for (size_t i = 1; i < primitive_objects.size(); ++i) {
        if (primitive_objects[i] < primitive_objects[i - 1]) ++out_of_order;
    }
    CHECK_EQ(out_of_order, size_t{ 0 });
}

/**
 * Turning metadata off changes the comments and NOTHING about the geometry.
 *
 * Stated as an equality of the recovered triangle multisets, so a writer that took
 * a different grouping path when metadata was off would be caught rather than
 * merely producing a smaller file.
 */
TEST(SceneExport, metadata_off_keeps_the_geometry_identical) {
    const Scene scene = make_city();
    SceneExportConfig on;
    on.chunk_size = 0.0f;
    SceneExportConfig off = on;
    off.write_metadata = false;

    const fs::path dir_on = scratch_dir("meta_on");
    const fs::path dir_off = scratch_dir("meta_off");
    export_scene(scene.objects, dir_on, on);
    export_scene(scene.objects, dir_off, off);

    const ObjFile with = read_obj(dir_on / "scene.obj");
    const ObjFile without = read_obj(dir_off / "scene.obj");
    CHECK_TRUE(with.ok);
    CHECK_TRUE(without.ok);

    CHECK_FALSE(object_comments(with).empty());
    CHECK_TRUE(object_comments(without).empty());
    CHECK_EQ(without.objects.size(), with.objects.size());
    CHECK_EQ(without.faces.size(), with.faces.size());

    std::multiset<TriKey> a;
    std::multiset<TriKey> b;
    for (const ObjFace& face : with.faces) {
        glm::vec3 c{ 0.0f };
        if (face_centroid(with, face, c)) a.insert(quantise(c));
    }
    for (const ObjFace& face : without.faces) {
        glm::vec3 c{ 0.0f };
        if (face_centroid(without, face, c)) b.insert(quantise(c));
    }
    CHECK_TRUE(a == b);
}

/**
 * A name carrying a newline cannot inject a directive.
 *
 * OSM names contain newlines, and an unsanitised one in an `o` record or a metadata
 * comment ends that line early and leaves the rest of the string as a line of its
 * own. A string chosen to look like a face directive is the sharp version: without
 * sanitisation the reader finds one more triangle than was exported, in a file that
 * still parses perfectly.
 */
TEST(SceneExport, a_name_cannot_inject_an_obj_directive) {
    Mesh mesh = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                          { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Wall);
    SceneObject object;
    object.mesh = &mesh;
    object.kind = SceneObjectKind::Building;
    object.name = "Evil\nf 1/1/1 2/2/2 3/3/3";
    object.metadata.emplace_back("key\nf 1/1/1 2/2/2 4/4/4", "value\nusemtl injected");

    const fs::path dir = scratch_dir("injection");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;
    const SceneExportStats stats = export_scene({ object }, dir, cfg);

    CHECK_EQ(stats.triangles, size_t{ 2 });

    const ObjFile obj = read_obj(dir / "scene.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.faces.size(), size_t{ 2 });
    CHECK_EQ(obj.objects.size(), size_t{ 1 });
    CHECK_EQ(obj.materials.size(), size_t{ 1 });
}

// ============================================================================
// Materials
// ============================================================================

/**
 * Material ranges survive, named exactly as the road exporter names them.
 *
 * The shared naming is the point: a scene export and a road export of one city must
 * call the same surface the same thing, or an engine binds two materials for it.
 */
TEST(SceneExport, material_ranges_survive_with_the_shared_names) {
    // One mesh, two materials: four wall triangles and two roof ones.
    Mesh mesh = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                          { 0.8f, 0.7f, 0.6f, 1.0f }, MaterialId::Wall);
    const Mesh more_wall = make_quad(4.0f, 1.0f, 6.0f, 3.0f, 0.0f,
                                     { 0.8f, 0.7f, 0.6f, 1.0f }, MaterialId::Wall);
    const Mesh roof = make_quad(1.0f, 4.0f, 3.0f, 6.0f, 3.0f,
                                { 0.4f, 0.2f, 0.2f, 1.0f }, MaterialId::Roof);
    mesh.append(more_wall, MaterialId::Wall);
    mesh.append(roof, MaterialId::Roof);

    SceneObject object;
    object.mesh = &mesh;
    object.kind = SceneObjectKind::Building;
    object.name = "Hall";

    const fs::path dir = scratch_dir("materials");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;
    export_scene({ object }, dir, cfg);

    const ObjFile obj = read_obj(dir / "scene.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.faces.size(), size_t{ 6 });

    std::map<std::string, size_t> per_material;
    for (const ObjFace& face : obj.faces) ++per_material[face.material];
    CHECK_EQ(per_material.size(), size_t{ 2 });
    CHECK_EQ(per_material[std::string{ "stratum_" }
                          + stratum::osm::road::material_key_name({ MaterialId::Wall, 0 })],
             size_t{ 4 });
    CHECK_EQ(per_material[std::string{ "stratum_" }
                          + stratum::osm::road::material_key_name({ MaterialId::Roof, 0 })],
             size_t{ 2 });

    // The MTL declares exactly those two, and the OBJ points at it.
    const std::string mtl = file_bytes(dir / "scene.mtl");
    CHECK((mtl.find("newmtl stratum_Wall")) != std::string::npos);
    CHECK((mtl.find("newmtl stratum_Roof")) != std::string::npos);
    CHECK_EQ(obj.mtllib, std::string{ "scene.mtl" });
}

/**
 * The exported material colour is the MEAN vertex colour of what used it.
 *
 * It is derived rather than tabled, so there is no second copy of road_export.cpp's
 * debug palette to drift. The averaging is what the test has to pin down: an
 * implementation that took the first or the last contributor would give 1,0,0 or
 * 0,0,1 here, and only the mean gives 0.5,0,0.5.
 */
TEST(SceneExport, material_colour_is_the_mean_of_the_vertices_that_used_it) {
    const Mesh red = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                               { 1.0f, 0.0f, 0.0f, 1.0f }, MaterialId::Wall);
    const Mesh blue = make_quad(4.0f, 1.0f, 6.0f, 3.0f, 0.0f,
                                { 0.0f, 0.0f, 1.0f, 1.0f }, MaterialId::Wall);
    const Mesh green = make_quad(1.0f, 4.0f, 3.0f, 6.0f, 0.0f,
                                 { 0.0f, 1.0f, 0.0f, 1.0f }, MaterialId::Roof);

    std::vector<SceneObject> objects(3);
    objects[0].mesh = &red;
    objects[1].mesh = &blue;
    objects[2].mesh = &green;

    const fs::path dir = scratch_dir("colours");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;
    export_scene(objects, dir, cfg);

    const std::string mtl = file_bytes(dir / "scene.mtl");
    const size_t wall = mtl.find("newmtl stratum_Wall");
    const size_t roof = mtl.find("newmtl stratum_Roof");
    CHECK((wall) != std::string::npos);
    CHECK((roof) != std::string::npos);
    CHECK((mtl.find("Kd 0.500 0.000 0.500", wall)) != std::string::npos);
    CHECK((mtl.find("Kd 0.000 1.000 0.000", roof)) != std::string::npos);

    // glTF carries the same number, so the two writers cannot disagree about it.
    const fs::path gltf_dir = scratch_dir("colours_gltf");
    cfg.format = SceneExportFormat::Gltf;
    export_scene(objects, gltf_dir, cfg);

    const GltfFile gltf = read_gltf(gltf_dir / "scene.gltf");
    CHECK_TRUE(gltf.ok);
    if (!gltf.ok) return;

    // Three primitives, one per object, but only TWO materials: the wall is shared.
    CHECK_EQ(gltf.doc["meshes"][0]["primitives"].size(), size_t{ 3 });
    CHECK_EQ(gltf.doc["materials"].size(), size_t{ 2 });
    for (const auto& material : gltf.doc["materials"]) {
        const auto& factor = material["pbrMetallicRoughness"]["baseColorFactor"];
        if (material.value("name", std::string{}) == "stratum_Wall") {
            CHECK_NEAR(factor[0].get<double>(), 0.5, 1e-6);
            CHECK_NEAR(factor[1].get<double>(), 0.0, 1e-6);
            CHECK_NEAR(factor[2].get<double>(), 0.5, 1e-6);
        } else {
            CHECK_NEAR(factor[0].get<double>(), 0.0, 1e-6);
            CHECK_NEAR(factor[1].get<double>(), 1.0, 1e-6);
            CHECK_NEAR(factor[2].get<double>(), 0.0, 1e-6);
        }
    }
}

// ============================================================================
// glTF
// ============================================================================

/**
 * The glTF writer conserves triangles too, checked by decoding the `.bin`.
 *
 * The positions and the indices are read out of the buffer through the accessors'
 * own offsets, so a bufferView offset that is off by one attribute produces
 * centroids that are somewhere else entirely and the multiset comparison fails.
 * Counting `primitives` would not have noticed.
 */
TEST(SceneExport, the_gltf_writer_conserves_every_triangle) {
    const Scene scene = make_city();
    const std::multiset<TriKey> expected = input_centroids(scene);

    const fs::path dir = scratch_dir("gltf_conservation");
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;
    cfg.format = SceneExportFormat::Gltf;

    const SceneExportStats stats = export_scene(scene.objects, dir, cfg);
    CHECK_EQ(stats.triangles, scene.triangles());
    CHECK_EQ(stats.unwritten_triangles, size_t{ 0 });
    // One `.gltf` and one `.bin` per chunk.
    CHECK_EQ(stats.files, stats.chunks * 2u);

    const std::vector<ChunkFile> chunks = chunk_files(dir, "scene", ".gltf");
    CHECK_EQ(chunks.size(), stats.chunks);

    std::multiset<TriKey> recovered;
    size_t wrong_mode = 0;
    size_t wrong_bounds = 0;
    size_t unsnapped_tangents = 0;
    size_t wrong_uvs = 0;

    for (const ChunkFile& chunk : chunks) {
        const GltfFile gltf = read_gltf(chunk.path);
        CHECK_TRUE(gltf.ok);
        if (!gltf.ok) continue;

        for (const auto& primitive : gltf.doc["meshes"][0]["primitives"]) {
            // 4 is TRIANGLES. A primitive left at 0 is POINTS, which every viewer
            // honours: the geometry is all present and the file draws as a dust
            // cloud, so a test that only decodes POSITION and the indices sees a
            // perfect export.
            if (primitive.value("mode", -1) != 4) ++wrong_mode;

            const std::vector<uint32_t> indices =
                read_indices(gltf, primitive["indices"].get<size_t>());
            const size_t position_accessor =
                primitive["attributes"]["POSITION"].get<size_t>();
            const std::vector<glm::vec3> positions = read_vec3(gltf, position_accessor);
            CHECK_FALSE(indices.empty());
            CHECK_FALSE(positions.empty());
            CHECK_EQ(indices.size() % 3u, size_t{ 0 });

            // The POSITION accessor's min and max are REQUIRED by glTF 2.0 and are
            // what a viewer frames the scene with. Exchanged, they describe an
            // inverted box and the model is off screen, while every byte of geometry
            // is still correct -- so nothing that reads the buffer can notice.
            const auto& accessor = gltf.doc["accessors"][position_accessor];
            if (!accessor.contains("min") || !accessor.contains("max")) {
                ++wrong_bounds;
            } else {
                glm::vec3 low(std::numeric_limits<float>::max());
                glm::vec3 high(std::numeric_limits<float>::lowest());
                for (const glm::vec3& p : positions) {
                    low = glm::min(low, p);
                    high = glm::max(high, p);
                }
                for (int axis = 0; axis < 3; ++axis) {
                    const auto declared_min = accessor["min"][static_cast<size_t>(axis)]
                                                  .get<double>();
                    const auto declared_max = accessor["max"][static_cast<size_t>(axis)]
                                                  .get<double>();
                    if (std::abs(declared_min - static_cast<double>(low[axis])) > 1e-3
                        || std::abs(declared_max - static_cast<double>(high[axis])) > 1e-3) {
                        ++wrong_bounds;
                    }
                }
            }

            // TANGENT.w is a handedness flag and is one of exactly two values. The
            // fixture supplies 0 and -0.5, so a writer passing the raw float through
            // is visible here and nowhere else.
            const std::vector<glm::vec4> tangents =
                read_vec4(gltf, primitive["attributes"]["TANGENT"].get<size_t>());
            for (const glm::vec4& t : tangents) {
                if (t.w != 1.0f && t.w != -1.0f) ++unsnapped_tangents;
            }

            // make_quad() derives uv from the position, so u and v exchanged is
            // recoverable from the file alone.
            const std::vector<glm::vec2> uvs =
                read_vec2(gltf, primitive["attributes"]["TEXCOORD_0"].get<size_t>());
            if (uvs.size() != positions.size()) {
                ++wrong_uvs;
            } else {
                for (size_t i = 0; i < uvs.size(); ++i) {
                    if (std::abs(uvs[i].x - positions[i].x * 0.1f) > 1e-3f
                        || std::abs(uvs[i].y - positions[i].z * 0.1f) > 1e-3f) {
                        ++wrong_uvs;
                    }
                }
            }

            for (size_t t = 0; t + 2u < indices.size(); t += 3u) {
                if (indices[t] >= positions.size() || indices[t + 1] >= positions.size()
                    || indices[t + 2] >= positions.size()) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "a glTF index is inside its accessor",
                                                  chunk.path.filename().string());
                    continue;
                }
                recovered.insert(quantise((positions[indices[t]] + positions[indices[t + 1]]
                                         + positions[indices[t + 2]]) / 3.0f));
            }
        }
    }

    CHECK_EQ(recovered.size(), expected.size());
    CHECK_TRUE(recovered == expected);
    CHECK_EQ(wrong_mode, size_t{ 0 });
    CHECK_EQ(wrong_bounds, size_t{ 0 });
    CHECK_EQ(unsnapped_tangents, size_t{ 0 });
    CHECK_EQ(wrong_uvs, size_t{ 0 });
}

// ============================================================================
// Frames
// ============================================================================

/**
 * y_up = false rotates the frame for OBJ: (x, y, z) -> (x, -z, y).
 *
 * The vertex is chosen with three different, non-zero, differently signed
 * components so that the identity, a mirror and the inverse rotation all produce a
 * different line. A symmetric point would pass against every one of them.
 */
TEST(SceneExport, z_up_rotates_positions_for_obj) {
    Mesh mesh;
    const glm::vec3 corners[3] = {
        { 1.0f, 2.0f, 3.0f }, { 5.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 7.0f }
    };
    for (const glm::vec3& p : corners) {
        Vertex v;
        v.position = p;
        v.normal = { 0.0f, 1.0f, 0.0f };
        mesh.vertices.push_back(v);
    }
    mesh.indices = { 0, 1, 2 };
    mesh.compute_bounds();

    SceneObject object;
    object.mesh = &mesh;

    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;

    const fs::path up = scratch_dir("y_up");
    export_scene({ object }, up, cfg);
    const ObjFile as_written = read_obj(up / "scene.obj");
    CHECK_TRUE(as_written.ok);
    CHECK_EQ(as_written.positions.size(), size_t{ 3 });
    CHECK_NEAR(as_written.positions[0].x, 1.0, 1e-4);
    CHECK_NEAR(as_written.positions[0].y, 2.0, 1e-4);
    CHECK_NEAR(as_written.positions[0].z, 3.0, 1e-4);

    cfg.y_up = false;
    const fs::path flipped = scratch_dir("z_up");
    export_scene({ object }, flipped, cfg);
    const ObjFile rotated = read_obj(flipped / "scene.obj");
    CHECK_TRUE(rotated.ok);
    CHECK_EQ(rotated.positions.size(), size_t{ 3 });
    CHECK_NEAR(rotated.positions[0].x, 1.0, 1e-4);
    CHECK_NEAR(rotated.positions[0].y, -3.0, 1e-4);
    CHECK_NEAR(rotated.positions[0].z, 2.0, 1e-4);
    // The normal is rotated the same way, or every surface is lit as though it
    // faced somewhere else: (0, 1, 0) -> (0, 0, 1). Checked on the NORMAL records,
    // because a writer that rotated only the positions passes every assertion above.
    CHECK_EQ(as_written.normals.size(), size_t{ 3 });
    CHECK_EQ(rotated.normals.size(), size_t{ 3 });
    CHECK_NEAR(as_written.normals[0].y, 1.0, 1e-4);
    CHECK_NEAR(as_written.normals[0].z, 0.0, 1e-4);
    CHECK_NEAR(rotated.normals[0].x, 0.0, 1e-4);
    CHECK_NEAR(rotated.normals[0].y, 0.0, 1e-4);
    CHECK_NEAR(rotated.normals[0].z, 1.0, 1e-4);

    // glTF 2.0 requires Y up, so the same request is ignored there rather than
    // written out as a file that is valid and wrong in every viewer.
    //
    // The glTF writer now READS cfg.y_up like the OBJ writer does, and relies on
    // effective_config() having forced it true for this format. That is what makes
    // these assertions able to fail: while the writer ignored the flag outright, the
    // fixup could be deleted entirely and every line below still passed, because
    // nothing downstream of it could tell the difference.
    cfg.format = SceneExportFormat::Gltf;
    const fs::path gltf_dir = scratch_dir("z_up_gltf");
    export_scene({ object }, gltf_dir, cfg);
    const GltfFile gltf = read_gltf(gltf_dir / "scene.gltf");
    CHECK_TRUE(gltf.ok);
    if (!gltf.ok) return;
    const auto& attributes = gltf.doc["meshes"][0]["primitives"][0]["attributes"];
    const std::vector<glm::vec3> positions =
        read_vec3(gltf, attributes["POSITION"].get<size_t>());
    CHECK_EQ(positions.size(), size_t{ 3 });
    CHECK_NEAR(positions[0].x, 1.0, 1e-4);
    CHECK_NEAR(positions[0].y, 2.0, 1e-4);
    CHECK_NEAR(positions[0].z, 3.0, 1e-4);

    // The normal is not rotated either, or the geometry is Y up and the shading is
    // Z up, which is worse than either frame consistently.
    const std::vector<glm::vec3> gltf_normals =
        read_vec3(gltf, attributes["NORMAL"].get<size_t>());
    CHECK_EQ(gltf_normals.size(), size_t{ 3 });
    CHECK_NEAR(gltf_normals[0].x, 0.0, 1e-4);
    CHECK_NEAR(gltf_normals[0].y, 1.0, 1e-4);
    CHECK_NEAR(gltf_normals[0].z, 0.0, 1e-4);

    // And asking for Y up explicitly produces the very same bytes, so "ignored" is
    // checked as an equality and not only against hand-written expectations.
    cfg.y_up = true;
    const fs::path gltf_up = scratch_dir("y_up_gltf");
    export_scene({ object }, gltf_up, cfg);
    CHECK_FALSE(file_bytes(gltf_up / "scene.bin").empty());
    CHECK_EQ(file_bytes(gltf_dir / "scene.bin"), file_bytes(gltf_up / "scene.bin"));
}

// ============================================================================
// Accounting
// ============================================================================

/**
 * A triangle that cannot be assigned is dropped AND counted.
 *
 * `triangles + dropped_triangles` is the input, so the conservation invariant stays
 * checkable in the presence of bad geometry. Without the second number, an exporter
 * that silently lost half the scene would still report that it wrote everything it
 * kept.
 */
TEST(SceneExport, unassignable_triangles_are_dropped_and_counted) {
    Mesh mesh = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                          { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Wall);

    // A fourth vertex carrying a NaN, referenced by a third triangle.
    Vertex bad;
    bad.position = { std::nanf(""), 0.0f, 0.0f };
    mesh.vertices.push_back(bad);
    mesh.indices.push_back(0);
    mesh.indices.push_back(1);
    mesh.indices.push_back(4);

    // And a fourth triangle referencing a vertex that does not exist.
    mesh.indices.push_back(0);
    mesh.indices.push_back(1);
    mesh.indices.push_back(99);

    // The submesh still claims the first six indices only; the two added triangles
    // fall through to the implicit Default range, which must not drop them silently
    // either.
    SceneObject object;
    object.mesh = &mesh;

    const fs::path dir = scratch_dir("dropped");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;
    const SceneExportStats stats = export_scene({ object }, dir, cfg);

    CHECK_EQ(triangle_count(mesh), size_t{ 4 });
    CHECK_EQ(stats.triangles, size_t{ 2 });
    CHECK_EQ(stats.dropped_triangles, size_t{ 2 });
    CHECK_EQ(stats.triangles + stats.dropped_triangles, triangle_count(mesh));

    const ObjFile obj = read_obj(dir / "scene.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.faces.size(), size_t{ 2 });
    // Nothing non-finite reached the file, which would poison a reader's bounds.
    //
    // Checked on the NORMALS and TEXTURE COORDINATES as well, and then on the raw
    // bytes. This loop used to read obj.positions alone, which is the one attribute
    // the centroid guard already protects -- so it could not see a `vn nan` or a
    // `vt nan` line, and both were being written. See
    // a_nan_in_any_vertex_attribute_never_reaches_a_file for the fixture that
    // actually drives one in.
    for (const glm::vec3& p : obj.positions) {
        CHECK_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
    }
    for (const glm::vec3& n : obj.normals) {
        CHECK_TRUE(std::isfinite(n.x) && std::isfinite(n.y) && std::isfinite(n.z));
    }
    for (const glm::vec2& uv : obj.uvs) {
        CHECK_TRUE(std::isfinite(uv.x) && std::isfinite(uv.y));
    }
    const std::string bytes = file_bytes(dir / "scene.obj");
    CHECK((bytes.find("nan")) == std::string::npos);
    CHECK((bytes.find("inf")) == std::string::npos);
}

/**
 * Two exports of one scene are byte-identical.
 *
 * The grouping and the material table are ordered containers for this reason: their
 * iteration order IS the order of the file, and a hash map there would make a diff
 * of two runs meaningless without changing a single count.
 */
TEST(SceneExport, an_export_is_deterministic) {
    const Scene scene = make_city();
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;

    const fs::path first = scratch_dir("determinism_a");
    const fs::path second = scratch_dir("determinism_b");
    const SceneExportStats a = export_scene(scene.objects, first, cfg);
    const SceneExportStats b = export_scene(scene.objects, second, cfg);

    CHECK_EQ(a.files, b.files);
    CHECK_EQ(a.chunks, b.chunks);

    const std::vector<ChunkFile> chunks = chunk_files(first, "scene", ".obj");
    CHECK_FALSE(chunks.empty());
    size_t differing = 0;
    size_t groups_out_of_order = 0;
    for (const ChunkFile& chunk : chunks) {
        const std::string one = file_bytes(chunk.path);
        const std::string two = file_bytes(second / chunk.path.filename());
        CHECK_FALSE(one.empty());
        if (one != two) ++differing;

        // Byte equality between two runs IN ONE PROCESS does not prove the grouping
        // container is ordered: an unordered_map fed the same keys in the same order
        // reproduces its own iteration order exactly, so both files agree and both
        // are wrong. The ORDER itself is the thing to assert.
        //
        // Object index ascending, and no (object, material) group ever re-entered.
        // Only the object half is checked here: within an object the groups ascend by
        // PACKED MaterialKey, which is not the order their names sort in, and the
        // file carries the names. a_chunk_groups_by_object_then_material pins the
        // material half down against a fixture whose packed order is known.
        long previous_object = -1;
        std::set<std::pair<std::string, std::string>> seen_groups;
        for (const auto& [record, material] : face_groups(read_obj(chunk.path))) {
            const long index = record_index(record);
            if (index < previous_object) ++groups_out_of_order;
            if (!seen_groups.emplace(record, material).second) ++groups_out_of_order;
            previous_object = index;
        }
    }
    CHECK_EQ(differing, size_t{ 0 });
    CHECK_EQ(groups_out_of_order, size_t{ 0 });

    // written_files is documented to ascend by grid x then z. Parse the cells back
    // out of the names and check the sequence really ascends: comparing the list
    // against a sorted copy of ITSELF would pass whatever order it was in, and a
    // list in map order looks plausible either way.
    std::vector<std::pair<long, long>> cells;
    for (const std::string& path : a.written_files) {
        const fs::path file(path);
        if (file.extension() != ".obj") continue;   // the shared MTL has no cell
        const std::string stem = file.stem().string();
        const size_t first = stem.find('_');
        if (first == std::string::npos) continue;
        const size_t second = stem.find('_', first + 2u);
        if (second == std::string::npos) continue;
        cells.emplace_back(std::strtol(stem.substr(first + 1u, second - first - 1u).c_str(),
                                       nullptr, 10),
                           std::strtol(stem.substr(second + 1u).c_str(), nullptr, 10));
    }
    CHECK_EQ(cells.size(), a.chunks);
    size_t out_of_order = 0;
    for (size_t i = 1; i < cells.size(); ++i) {
        if (!(cells[i - 1] < cells[i])) ++out_of_order;
    }
    CHECK_EQ(out_of_order, size_t{ 0 });
}

/**
 * A cell index that int32_t cannot hold is refused, not wrapped.
 *
 * `static_cast<int32_t>(std::floor(coord / size))` is UNDEFINED outside the int32
 * range, and on x86 it produces INT32_MIN. Every such triangle therefore used to land
 * in one cell named `scene_-2147483648_-2147483648`, and the sums still balanced: the
 * conservation test passed, and "every triangle lands in the chunk its name claims"
 * quietly stopped being true while two unrelated regions merged into one file.
 *
 * The centroid being FINITE is not enough to prevent it, which is the whole point --
 * the quotient is what has to fit, and it depends on chunk_size as much as on the
 * coordinate.
 */
TEST(SceneExport, a_cell_index_that_cannot_be_represented_is_dropped_and_counted) {
    struct Probe {
        const char* what;
        float x0;
        float z0;
        float chunk;
    };
    // Three ways to overflow from values that are each individually ordinary. The
    // third is the set_origin() trap in CLAUDE.md: wgs84_to_local() returns raw
    // Mercator metres when no origin was set, and 2e7 of them against a small cell is
    // not an absurd thing for a caller to ask for.
    const Probe probes[] = {
        { "a far-away coordinate", 1e20f, 1e20f, 500.0f },
        { "a vanishingly small cell", 1.0f, 1.0f, 1e-30f },
        { "raw Mercator against a 5 mm cell", 2e7f, 2e7f, 0.005f }
    };

    for (const Probe& probe : probes) {
        const Mesh mesh = make_quad(probe.x0, probe.z0, probe.x0 + 2.0f, probe.z0 + 2.0f,
                                    0.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Wall);
        SceneObject object;
        object.mesh = &mesh;
        object.name = probe.what;

        const fs::path dir = scratch_dir("unindexable");
        SceneExportConfig cfg;
        cfg.chunk_size = probe.chunk;
        const SceneExportStats stats = export_scene({ object }, dir, cfg);

        CHECK_EQ(stats.triangles, size_t{ 0 });
        CHECK_EQ(stats.dropped_triangles, triangle_count(mesh));
        CHECK_EQ(stats.chunks, size_t{ 0 });
        CHECK_EQ(stats.files, size_t{ 0 });
        // The wrapped cell, spelled out: this is the file the defect produced.
        CHECK_FALSE(fs::exists(dir / "scene_-2147483648_-2147483648.obj"));

        size_t entries = 0;
        std::error_code ec;
        for (const auto& entry : fs::directory_iterator(dir, ec)) {
            (void)entry;
            ++entries;
        }
        CHECK_EQ(entries, size_t{ 0 });
    }

    // A refusal is per triangle, not per export: geometry that CAN be indexed is
    // still written, and the three buckets still add up to the input.
    const Mesh good = make_quad(10.0f, 10.0f, 20.0f, 20.0f, 0.0f,
                                { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Wall);
    const Mesh far = make_quad(1e20f, 1e20f, 1e20f + 2.0f, 1e20f + 2.0f, 0.0f,
                               { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Wall);
    std::vector<SceneObject> mixed(2);
    mixed[0].mesh = &good;
    mixed[1].mesh = &far;

    const fs::path dir = scratch_dir("unindexable_mixed");
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;
    const SceneExportStats stats = export_scene(mixed, dir, cfg);

    CHECK_EQ(stats.triangles, size_t{ 2 });
    CHECK_EQ(stats.dropped_triangles, size_t{ 2 });
    CHECK_EQ(stats.unwritten_triangles, size_t{ 0 });
    CHECK_EQ(stats.triangles + stats.dropped_triangles + stats.unwritten_triangles,
             triangle_count(good) + triangle_count(far));
    CHECK_EQ(stats.chunks, size_t{ 1 });

    const std::vector<ChunkFile> chunks = chunk_files(dir, "scene", ".obj");
    CHECK_EQ(chunks.size(), size_t{ 1 });
    CHECK_EQ(chunks[0].cx, 0L);
    CHECK_EQ(chunks[0].cz, 0L);
}

/**
 * A chunk whose file could not be written is not reported as written.
 *
 * SceneExportStats::triangles used to be credited where a triangle was ROUTED, which
 * is before any file is opened. A chunk that then failed to open was skipped, and its
 * triangles stayed in the total: a caller checking `triangles + dropped == input` was
 * told the export had conserved everything while a whole file was missing from disk.
 * stats.vertices and stats.chunks were already credited on success, so the two halves
 * of the same stats struct disagreed about the same failure.
 *
 * A DIRECTORY standing where a chunk file should go is the cheapest reliable way to
 * make one open fail without making them all fail, so the success path stays covered
 * in the same export.
 */
TEST(SceneExport, a_chunk_that_cannot_be_written_is_counted_as_unwritten) {
    const Mesh blocked = make_quad(10.0f, 10.0f, 20.0f, 20.0f, 0.0f,
                                   { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Wall);
    const Mesh writable = make_quad(110.0f, 110.0f, 120.0f, 120.0f, 0.0f,
                                    { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Roof);
    std::vector<SceneObject> objects(2);
    objects[0].mesh = &blocked;
    objects[1].mesh = &writable;

    const fs::path dir = scratch_dir("open_failure");
    std::error_code ec;
    fs::create_directories(dir / "scene_0_0.obj", ec);
    CHECK_TRUE(fs::is_directory(dir / "scene_0_0.obj"));

    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;
    const SceneExportStats stats = export_scene(objects, dir, cfg);

    const size_t input = triangle_count(blocked) + triangle_count(writable);

    // Only the chunk that reached disk is counted as written, and the blocked one is
    // accounted for rather than absorbed.
    CHECK_EQ(stats.chunks, size_t{ 1 });
    CHECK_EQ(stats.triangles, triangle_count(writable));
    CHECK_EQ(stats.unwritten_triangles, triangle_count(blocked));
    CHECK_EQ(stats.dropped_triangles, size_t{ 0 });
    CHECK_EQ(stats.triangles + stats.dropped_triangles + stats.unwritten_triangles, input);

    // And stats.triangles agrees with what a consumer can actually load.
    size_t on_disk = 0;
    for (const ChunkFile& chunk : chunk_files(dir, "scene", ".obj")) {
        const ObjFile obj = read_obj(chunk.path);
        CHECK_TRUE(obj.ok);
        on_disk += obj.faces.size();
    }
    CHECK_EQ(on_disk, stats.triangles);
    CHECK_EQ(stats.written_files.size(), stats.files);
}

/**
 * A name prefix cannot move the output out of the directory it was given.
 *
 * name_prefix is an editor-facing config field, and the stem it produces is
 * concatenated into `out_dir / (stem + ext)`. sanitize_token() -- which is all it used
 * to get -- makes a string safe as an OBJ TOKEN and passes '/', '\\' and '.' straight
 * through, so:
 *
 *  - "../escaped" wrote `out_dir/../escaped.obj`, one level ABOVE out_dir, and
 *    reported success with a path the caller never asked for;
 *  - "sub/thing" named a directory ensure_directory() never created, so every open
 *    failed and the export produced nothing while still reporting triangles.
 */
TEST(SceneExport, a_name_prefix_cannot_escape_the_output_directory) {
    const Mesh mesh = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                                { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Wall);
    SceneObject object;
    object.mesh = &mesh;
    object.name = "Hall";

    const fs::path root = scratch_dir("prefix_escape");
    const fs::path dir = root / "inside";
    std::error_code ec;
    fs::create_directories(dir, ec);

    const char* const hostile[] = { "../escaped", "sub/thing", "..", ".", "a\\b", "c:name" };

    for (const char* prefix : hostile) {
        for (const auto& entry : fs::directory_iterator(dir, ec)) {
            fs::remove_all(entry.path(), ec);
        }

        SceneExportConfig cfg;
        cfg.chunk_size = kChunk;
        cfg.name_prefix = prefix;
        const SceneExportStats stats = export_scene({ object }, dir, cfg);

        // It still exports. A stem that cannot name a file is a bug, not a refusal.
        CHECK_EQ(stats.triangles, triangle_count(mesh));
        CHECK_EQ(stats.unwritten_triangles, size_t{ 0 });
        CHECK((size_t{ 0 }) < stats.files);

        // Every file landed DIRECTLY inside out_dir, under a single-component name.
        size_t escaped = 0;
        for (const std::string& written : stats.written_files) {
            const fs::path file(written);
            if (fs::weakly_canonical(file.parent_path(), ec) != fs::weakly_canonical(dir, ec)) {
                ++escaped;
            }
            if (written.find("..") != std::string::npos) ++escaped;
            if (!fs::exists(file)) ++escaped;
        }
        CHECK_EQ(escaped, size_t{ 0 });

        // Nothing appeared beside out_dir, which is where "../escaped" used to write.
        size_t siblings = 0;
        for (const auto& entry : fs::directory_iterator(root, ec)) {
            if (entry.path().filename() != "inside") ++siblings;
        }
        CHECK_EQ(siblings, size_t{ 0 });

        // And no subdirectory was invented under out_dir either.
        size_t subdirectories = 0;
        for (const auto& entry : fs::directory_iterator(dir, ec)) {
            if (entry.is_directory()) ++subdirectories;
        }
        CHECK_EQ(subdirectories, size_t{ 0 });
    }
}

/**
 * A NaN in ANY vertex attribute is scrubbed, not written.
 *
 * The centroid guard rejects a triangle whose POSITION is non-finite and rejects
 * nothing else, so a vertex with a finite position and a NaN normal, uv, tangent or
 * colour reached both writers untouched: the OBJ got `vt nan` and `vn nan` lines that
 * strict importers refuse, and the glTF `.bin` got NaN floats, which the validator
 * reports as ACCESSOR_INVALID_FLOAT. One bad vertex cost a whole file its validity.
 *
 * road_export.cpp documents this exact hole and scrubs per component before encoding.
 * One bad vertex should cost one vertex.
 */
TEST(SceneExport, a_nan_in_any_vertex_attribute_never_reaches_a_file) {
    const float nan = std::nanf("");

    // Five corners, one broken attribute each, and every position finite so that not
    // one of these triangles is dropped by the centroid guard.
    Mesh mesh = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                          { 0.5f, 0.5f, 0.5f, 1.0f }, MaterialId::Wall);
    mesh.vertices[0].normal.x = nan;
    mesh.vertices[1].uv.x = nan;
    mesh.vertices[2].tangent.x = nan;
    mesh.vertices[3].color.r = nan;
    // A normal that is ENTIRELY non-finite scrubs to (0, 0, 0), which is not a
    // normal. It falls back to straight up, exactly as road_export.cpp does.
    Vertex all_bad = mesh.vertices[0];
    all_bad.position = { 2.0f, 0.0f, 2.5f };
    all_bad.normal = { nan, nan, nan };
    mesh.vertices.push_back(all_bad);
    mesh.indices.push_back(0);
    mesh.indices.push_back(1);
    mesh.indices.push_back(4);
    mesh.submeshes.clear();
    mesh.submeshes.push_back(SubMesh{ 0u, static_cast<uint32_t>(mesh.indices.size()),
                                      MaterialId::Wall, 0u });

    SceneObject object;
    object.mesh = &mesh;
    object.name = "Broken";

    const fs::path dir = scratch_dir("nan_attributes");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;
    const SceneExportStats stats = export_scene({ object }, dir, cfg);

    // Nothing is dropped: every centroid is finite, so this is a scrub and not a
    // refusal, and the triangle count is untouched.
    CHECK_EQ(stats.triangles, triangle_count(mesh));
    CHECK_EQ(stats.dropped_triangles, size_t{ 0 });

    const ObjFile obj = read_obj(dir / "scene.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.faces.size(), triangle_count(mesh));

    const std::string bytes = file_bytes(dir / "scene.obj");
    CHECK((bytes.find("nan")) == std::string::npos);
    CHECK((bytes.find("NaN")) == std::string::npos);
    CHECK((bytes.find("inf")) == std::string::npos);

    CHECK_EQ(obj.normals.size(), mesh.vertices.size());
    CHECK_EQ(obj.uvs.size(), mesh.vertices.size());
    for (const glm::vec3& n : obj.normals) {
        CHECK_TRUE(std::isfinite(n.x) && std::isfinite(n.y) && std::isfinite(n.z));
    }
    for (const glm::vec2& uv : obj.uvs) {
        CHECK_TRUE(std::isfinite(uv.x) && std::isfinite(uv.y));
    }
    // The all-NaN normal came back as straight up rather than as a zero vector.
    CHECK_NEAR(obj.normals[4].x, 0.0, 1e-4);
    CHECK_NEAR(obj.normals[4].y, 1.0, 1e-4);
    CHECK_NEAR(obj.normals[4].z, 0.0, 1e-4);
    // A scrubbed component becomes 0 and its neighbours are untouched: the fix costs
    // one component, not the whole vertex.
    CHECK_NEAR(obj.normals[0].x, 0.0, 1e-4);
    CHECK_NEAR(obj.normals[0].y, 1.0, 1e-4);

    // The same vertices through the other writer, read out of the .bin.
    const fs::path gltf_dir = scratch_dir("nan_attributes_gltf");
    cfg.format = SceneExportFormat::Gltf;
    export_scene({ object }, gltf_dir, cfg);

    const GltfFile gltf = read_gltf(gltf_dir / "scene.gltf");
    CHECK_TRUE(gltf.ok);
    if (!gltf.ok) return;

    const auto& attributes = gltf.doc["meshes"][0]["primitives"][0]["attributes"];
    size_t non_finite = 0;
    for (const glm::vec3& v : read_vec3(gltf, attributes["POSITION"].get<size_t>())) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) ++non_finite;
    }
    for (const glm::vec3& v : read_vec3(gltf, attributes["NORMAL"].get<size_t>())) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) ++non_finite;
    }
    for (const glm::vec2& v : read_vec2(gltf, attributes["TEXCOORD_0"].get<size_t>())) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) ++non_finite;
    }
    for (const glm::vec4& v : read_vec4(gltf, attributes["COLOR_0"].get<size_t>())) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)
            || !std::isfinite(v.w)) {
            ++non_finite;
        }
    }
    for (const glm::vec4& v : read_vec4(gltf, attributes["TANGENT"].get<size_t>())) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)
            || !std::isfinite(v.w)) {
            ++non_finite;
        }
    }
    CHECK_EQ(non_finite, size_t{ 0 });
}

/**
 * A chunk holding two multi-material objects groups by OBJECT first, then material.
 *
 * This is the one thing the scene exporter changed relative to the road exporter, and
 * every other fixture in this suite hides it: make_city()'s objects carried a single
 * material each until this test was written, and the two multi-material fixtures hold
 * exactly ONE object. With one object, or with one material per object, "object then
 * material" and "material then object" produce the same file, so the ordering that
 * write_obj_file()'s one-record-per-object run logic depends on was unasserted.
 *
 * Four groups in one chunk, chosen so the two orderings disagree: object 0 takes
 * Asphalt (slot 1) and Wall (slot 11), object 1 takes Concrete (slot 2) and Roof
 * (slot 12). Grouping by material first would interleave them as
 * Asphalt/Concrete/Wall/Roof, i.e. objects 0, 1, 0, 1.
 */
TEST(SceneExport, a_chunk_groups_by_object_then_material) {
    const auto two_material_quad = [](float x0, float z0, MaterialId first,
                                      MaterialId second) {
        Mesh mesh = make_quad(x0, z0, x0 + 4.0f, z0 + 4.0f, 0.0f,
                              { 0.5f, 0.5f, 0.5f, 1.0f }, first);
        const Mesh other = make_quad(x0, z0 + 6.0f, x0 + 4.0f, z0 + 10.0f, 0.0f,
                                     { 0.5f, 0.5f, 0.5f, 1.0f }, second);
        mesh.append(other, second);
        mesh.compute_bounds();
        return mesh;
    };

    // Both well inside cell (0, 0), so this is one chunk and the grouping is the only
    // thing deciding the file's shape.
    const Mesh first = two_material_quad(2.0f, 2.0f, MaterialId::Asphalt, MaterialId::Wall);
    const Mesh second = two_material_quad(20.0f, 2.0f, MaterialId::Concrete, MaterialId::Roof);

    std::vector<SceneObject> objects(2);
    objects[0].mesh = &first;
    objects[0].name = "Alpha";
    objects[1].mesh = &second;
    objects[1].name = "Beta";

    const fs::path dir = scratch_dir("grouping");
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;
    const SceneExportStats stats = export_scene(objects, dir, cfg);

    CHECK_EQ(stats.chunks, size_t{ 1 });
    CHECK_EQ(stats.triangles, size_t{ 8 });

    const ObjFile obj = read_obj(dir / "scene_0_0.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.faces.size(), size_t{ 8 });

    // Exactly one `o` record per object, in ascending object order.
    CHECK_EQ(obj.objects.size(), size_t{ 2 });
    CHECK_EQ(obj.objects[0], std::string{ "scene_o0_Alpha" });
    CHECK_EQ(obj.objects[1], std::string{ "scene_o1_Beta" });

    // Four groups, contiguous, object-major. Within an object the groups ascend by
    // PACKED MaterialKey, which is (slot << 16) | variant -- so Asphalt (1) before
    // Wall (11), and Concrete (2) before Roof (12).
    const std::vector<std::pair<std::string, std::string>> groups = face_groups(obj);
    CHECK_EQ(groups.size(), size_t{ 4 });
    if (groups.size() == 4u) {
        CHECK_EQ(groups[0].first, std::string{ "scene_o0_Alpha" });
        CHECK_EQ(groups[0].second, std::string{ "stratum_Asphalt" });
        CHECK_EQ(groups[1].first, std::string{ "scene_o0_Alpha" });
        CHECK_EQ(groups[1].second, std::string{ "stratum_Wall" });
        CHECK_EQ(groups[2].first, std::string{ "scene_o1_Beta" });
        CHECK_EQ(groups[2].second, std::string{ "stratum_Concrete" });
        CHECK_EQ(groups[3].first, std::string{ "scene_o1_Beta" });
        CHECK_EQ(groups[3].second, std::string{ "stratum_Roof" });
    }

    // Two faces per group, so no group was emptied or doubled by the reordering.
    std::map<std::pair<std::string, std::string>, size_t> per_group;
    for (const ObjFace& face : obj.faces) ++per_group[{ face.object, face.material }];
    CHECK_EQ(per_group.size(), size_t{ 4 });
    for (const auto& [key, count] : per_group) {
        (void)key;
        CHECK_EQ(count, size_t{ 2 });
    }

    // While here: make_quad() derives uv from the position, so the OBJ `vt` block can
    // be checked against the `v` block. Nothing else in the suite read `vt` at all, so
    // a writer that exchanged u and v was invisible.
    CHECK_EQ(obj.uvs.size(), obj.positions.size());
    size_t wrong_uvs = 0;
    for (size_t i = 0; i < obj.uvs.size(); ++i) {
        if (std::abs(obj.uvs[i].x - obj.positions[i].x * 0.1f) > 1e-3f
            || std::abs(obj.uvs[i].y - obj.positions[i].z * 0.1f) > 1e-3f) {
            ++wrong_uvs;
        }
    }
    CHECK_EQ(wrong_uvs, size_t{ 0 });

    // The same grouping reaches glTF: one primitive per (object, material).
    const fs::path gltf_dir = scratch_dir("grouping_gltf");
    cfg.format = SceneExportFormat::Gltf;
    export_scene(objects, gltf_dir, cfg);
    const GltfFile gltf = read_gltf(gltf_dir / "scene_0_0.gltf");
    CHECK_TRUE(gltf.ok);
    if (!gltf.ok) return;
    CHECK_EQ(gltf.doc["meshes"][0]["primitives"].size(), size_t{ 4 });
    CHECK_EQ(gltf.doc["materials"].size(), size_t{ 4 });

    std::vector<long> primitive_objects;
    for (const auto& primitive : gltf.doc["meshes"][0]["primitives"]) {
        primitive_objects.push_back(
            primitive["extras"]["stratum"]["index"].get<long>());
    }
    CHECK_EQ(primitive_objects.size(), size_t{ 4 });
    if (primitive_objects.size() == 4u) {
        CHECK_EQ(primitive_objects[0], 0L);
        CHECK_EQ(primitive_objects[1], 0L);
        CHECK_EQ(primitive_objects[2], 1L);
        CHECK_EQ(primitive_objects[3], 1L);
    }
}

/**
 * The material key is the (slot, variant) PAIR, and the variant reaches the file.
 *
 * Keying on the slot alone merges a cobbled carriageway with an asphalt one before any
 * writer sees them, and the two can then never be given different textures downstream.
 * Every other fixture in this suite passes variant 0, so the second half of the key was
 * never exercised: a key built as `{ sub.material, 0 }` passed the whole suite.
 */
TEST(SceneExport, a_material_variant_is_part_of_the_key) {
    // One slot, two variants. Asphalt variant 7 is "Coloured" in road_style.cpp's
    // frozen table, and those names travel in exported files.
    Mesh mesh = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                          { 0.5f, 0.5f, 0.5f, 1.0f }, MaterialId::Asphalt);
    const Mesh coloured = make_quad(5.0f, 1.0f, 7.0f, 3.0f, 0.0f,
                                    { 0.5f, 0.5f, 0.5f, 1.0f }, MaterialId::Asphalt);
    mesh.append(coloured, MaterialId::Asphalt);
    mesh.submeshes.clear();
    mesh.submeshes.push_back(SubMesh{ 0u, 6u, MaterialId::Asphalt, 0u });
    mesh.submeshes.push_back(SubMesh{ 6u, 6u, MaterialId::Asphalt, 7u });
    mesh.compute_bounds();

    SceneObject object;
    object.mesh = &mesh;
    object.name = "Street";

    const fs::path dir = scratch_dir("variants");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;
    const SceneExportStats stats = export_scene({ object }, dir, cfg);
    CHECK_EQ(stats.triangles, size_t{ 4 });

    const std::string expected_plain =
        std::string{ "stratum_" }
        + stratum::osm::road::material_key_name({ MaterialId::Asphalt, 0 });
    const std::string expected_variant =
        std::string{ "stratum_" }
        + stratum::osm::road::material_key_name({ MaterialId::Asphalt, 7 });
    CHECK_EQ(expected_plain, std::string{ "stratum_Asphalt" });
    CHECK_EQ(expected_variant, std::string{ "stratum_Asphalt.Coloured" });

    // TWO usemtl groups, not one merged range.
    const ObjFile obj = read_obj(dir / "scene.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.faces.size(), size_t{ 4 });
    CHECK_EQ(obj.materials.size(), size_t{ 2 });

    std::map<std::string, size_t> per_material;
    for (const ObjFace& face : obj.faces) ++per_material[face.material];
    CHECK_EQ(per_material.size(), size_t{ 2 });
    CHECK_EQ(per_material[expected_plain], size_t{ 2 });
    CHECK_EQ(per_material[expected_variant], size_t{ 2 });

    // And two newmtl entries, so a consumer can bind them separately.
    const std::string mtl = file_bytes(dir / "scene.mtl");
    CHECK((mtl.find("newmtl " + expected_plain + "\n")) != std::string::npos);
    CHECK((mtl.find("newmtl " + expected_variant + "\n")) != std::string::npos);

    // glTF agrees: two materials, not one.
    const fs::path gltf_dir = scratch_dir("variants_gltf");
    cfg.format = SceneExportFormat::Gltf;
    export_scene({ object }, gltf_dir, cfg);
    const GltfFile gltf = read_gltf(gltf_dir / "scene.gltf");
    CHECK_TRUE(gltf.ok);
    if (!gltf.ok) return;
    CHECK_EQ(gltf.doc["materials"].size(), size_t{ 2 });
    std::set<std::string> names;
    for (const auto& material : gltf.doc["materials"]) {
        names.insert(material.value("name", std::string{}));
    }
    CHECK_EQ(names.size(), size_t{ 2 });
    CHECK_TRUE(names.count(expected_plain) == 1u);
    CHECK_TRUE(names.count(expected_variant) == 1u);
}

/**
 * A centroid exactly on a cell boundary lands in the HIGHER cell, every time.
 *
 * floor() semantics, so 50.0 against a 50 m grid is cell 1 and not cell 0. Every other
 * fixture here deliberately avoids the line -- a tie whose side depends on the last bit
 * of a float makes a test fail for reasons that have nothing to do with the exporter --
 * which left the tie itself undocumented and unasserted. It is worth pinning down
 * exactly because it is the case a reader will wonder about.
 */
TEST(SceneExport, a_centroid_on_a_boundary_lands_in_the_higher_cell) {
    // A triangle whose three corners average to exactly (50, 0, 50).
    Mesh mesh;
    const glm::vec3 corners[3] = {
        { 40.0f, 0.0f, 50.0f }, { 60.0f, 0.0f, 40.0f }, { 50.0f, 0.0f, 60.0f }
    };
    for (const glm::vec3& p : corners) {
        Vertex v;
        v.position = p;
        v.normal = { 0.0f, 1.0f, 0.0f };
        mesh.vertices.push_back(v);
    }
    mesh.indices = { 0, 1, 2 };
    mesh.compute_bounds();

    const glm::vec3 centroid = (corners[0] + corners[1] + corners[2]) / 3.0f;
    CHECK_NEAR(centroid.x, 50.0, 1e-5);
    CHECK_NEAR(centroid.z, 50.0, 1e-5);

    SceneObject object;
    object.mesh = &mesh;
    object.name = "OnTheLine";

    const fs::path dir = scratch_dir("boundary");
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;
    const SceneExportStats stats = export_scene({ object }, dir, cfg);

    CHECK_EQ(stats.chunks, size_t{ 1 });
    CHECK_EQ(stats.triangles, size_t{ 1 });
    // The higher cell. floor(50 / 50) is 1, and the lower cell must not be written.
    CHECK_TRUE(fs::exists(dir / "scene_1_1.obj"));
    CHECK_FALSE(fs::exists(dir / "scene_0_0.obj"));

    const std::vector<ChunkFile> chunks = chunk_files(dir, "scene", ".obj");
    CHECK_EQ(chunks.size(), size_t{ 1 });
    if (chunks.size() == 1u) {
        CHECK_EQ(chunks[0].cx, 1L);
        CHECK_EQ(chunks[0].cz, 1L);
    }
}

// ============================================================================
// Naming
// ============================================================================

/// A negative cell is spelled with a minus, and holds what it should.
TEST(SceneExport, negative_cells_are_named_with_a_minus) {
    const Mesh mesh = make_quad(-140.0f, -140.0f, -130.0f, -130.0f, 0.0f,
                                { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Grass);
    SceneObject object;
    object.mesh = &mesh;
    object.name = "far";

    const fs::path dir = scratch_dir("negative");
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;
    const SceneExportStats stats = export_scene({ object }, dir, cfg);

    CHECK_EQ(stats.chunks, size_t{ 1 });
    CHECK_TRUE(fs::exists(dir / "scene_-3_-3.obj"));

    const ObjFile obj = read_obj(dir / "scene_-3_-3.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.faces.size(), size_t{ 2 });
}

/// The name prefix renames the files, the material library and the object records.
TEST(SceneExport, the_name_prefix_renames_everything_it_should) {
    const Mesh mesh = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                                { 1.0f, 1.0f, 1.0f, 1.0f }, MaterialId::Wall);
    SceneObject object;
    object.mesh = &mesh;
    object.name = "Hall";

    const fs::path dir = scratch_dir("prefix");
    SceneExportConfig cfg;
    cfg.chunk_size = kChunk;
    cfg.name_prefix = "buildings";
    export_scene({ object }, dir, cfg);

    CHECK_TRUE(fs::exists(dir / "buildings_0_0.obj"));
    CHECK_TRUE(fs::exists(dir / "buildings.mtl"));
    CHECK_FALSE(fs::exists(dir / "scene_0_0.obj"));

    const ObjFile obj = read_obj(dir / "buildings_0_0.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.mtllib, std::string{ "buildings.mtl" });
    CHECK_EQ(obj.objects.size(), size_t{ 1 });
    CHECK_EQ(obj.objects[0], std::string{ "buildings_o0_Hall" });

    // An unusable prefix falls back rather than writing a file called ".obj".
    SceneExportConfig blank = cfg;
    blank.name_prefix = "   ";
    const fs::path fallback = scratch_dir("prefix_blank");
    export_scene({ object }, fallback, blank);
    CHECK_TRUE(fs::exists(fallback / "scene_0_0.obj"));
}

// ============================================================================
// Single object
// ============================================================================

/// One object, one file, metadata included, and the same grouping as a full export.
TEST(SceneExport, a_single_object_exports_to_one_named_file) {
    Mesh mesh = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                          { 0.8f, 0.7f, 0.6f, 1.0f }, MaterialId::Wall);
    const Mesh roof = make_quad(1.0f, 4.0f, 3.0f, 6.0f, 3.0f,
                                { 0.4f, 0.2f, 0.2f, 1.0f }, MaterialId::Roof);
    mesh.append(roof, MaterialId::Roof);

    SceneObject object;
    object.mesh = &mesh;
    object.kind = SceneObjectKind::Building;
    object.name = "Solo";
    object.osm_id = 7;
    object.metadata.emplace_back("levels", "3");

    const fs::path dir = scratch_dir("single");
    const fs::path path = dir / "nested" / "thing.obj";

    CHECK_TRUE(export_scene_object(object, path, SceneExportConfig{}));
    CHECK_TRUE(fs::exists(path));
    CHECK_TRUE(fs::exists(dir / "nested" / "thing.mtl"));

    const ObjFile obj = read_obj(path);
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.faces.size(), triangle_count(mesh));
    CHECK_EQ(obj.materials.size(), size_t{ 2 });
    CHECK_EQ(obj.mtllib, std::string{ "thing.mtl" });
    CHECK_FALSE(find_comment(obj, "stratum:meta levels=3").empty());
    CHECK((find_comment(obj, "stratum:object ").find("osm_id=7")) != std::string::npos);

    // A null mesh is a refusal, not an empty file.
    SceneObject empty;
    CHECK_FALSE(export_scene_object(empty, dir / "nothing.obj", SceneExportConfig{}));
    CHECK_FALSE(fs::exists(dir / "nothing.obj"));
}

// ============================================================================
// Adapters
// ============================================================================

/// The OSM adapters fill in the identity a consumer would otherwise have to re-derive.
TEST(SceneExport, the_osm_adapters_carry_the_feature_identity) {
    const Mesh mesh = make_quad(1.0f, 1.0f, 3.0f, 3.0f, 0.0f,
                                { 0.8f, 0.7f, 0.6f, 1.0f }, MaterialId::Wall);

    stratum::osm::Building building;
    building.osm_id = 555;
    building.name = "Library";
    building.type = stratum::osm::BuildingType::Office;
    building.height = 12.5f;
    building.levels = 4;

    stratum::osm::Area area;
    area.osm_id = 556;
    area.name = "Park";
    area.type = stratum::osm::AreaType::Park;

    const SceneObject as_building = describe_building(building, mesh);
    CHECK_EQ(as_building.osm_id, int64_t{ 555 });
    CHECK_TRUE(as_building.kind == SceneObjectKind::Building);
    CHECK_EQ(as_building.name, std::string{ "Library" });
    CHECK_TRUE(as_building.mesh == &mesh);

    const SceneObject as_area = describe_area(area, mesh);
    CHECK_EQ(as_area.osm_id, int64_t{ 556 });
    CHECK_TRUE(as_area.kind == SceneObjectKind::Area);

    const fs::path dir = scratch_dir("adapters");
    SceneExportConfig cfg;
    cfg.chunk_size = 0.0f;
    export_scene({ as_building, as_area }, dir, cfg);

    const ObjFile obj = read_obj(dir / "scene.obj");
    CHECK_TRUE(obj.ok);
    CHECK_FALSE(find_comment(obj, "stratum:meta building_type=Office").empty());
    CHECK_FALSE(find_comment(obj, "stratum:meta height_m=12.50").empty());
    CHECK_FALSE(find_comment(obj, "stratum:meta levels=4").empty());
    CHECK_FALSE(find_comment(obj, "stratum:meta area_type=Park").empty());
}

/// Every kind names itself, and the names are the ones written into files.
TEST(SceneExport, every_kind_has_a_stable_name) {
    CHECK_EQ(std::string{ stratum::osm::scene_object_kind_name(SceneObjectKind::Unknown) },
             std::string{ "Unknown" });
    CHECK_EQ(std::string{ stratum::osm::scene_object_kind_name(SceneObjectKind::Building) },
             std::string{ "Building" });
    CHECK_EQ(std::string{ stratum::osm::scene_object_kind_name(SceneObjectKind::Area) },
             std::string{ "Area" });
    CHECK_EQ(std::string{ stratum::osm::scene_object_kind_name(SceneObjectKind::Road) },
             std::string{ "Road" });
    CHECK_EQ(std::string{ stratum::osm::scene_object_kind_name(SceneObjectKind::Terrain) },
             std::string{ "Terrain" });

    // Totality: no kind below Count returns the empty string or a null pointer, so a
    // kind added later without a case arm is caught here rather than in a file.
    for (uint8_t i = 0; i < static_cast<uint8_t>(SceneObjectKind::Count); ++i) {
        const char* name = stratum::osm::scene_object_kind_name(static_cast<SceneObjectKind>(i));
        CHECK_TRUE(name != nullptr && name[0] != '\0');
    }
}
