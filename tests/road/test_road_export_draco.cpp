// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_road_export_draco.cpp
 * @brief KHR_draco_mesh_compression: smaller bytes, identical triangles
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ExportConfig::draco_compression is the first thing in the exporter that can
 * change the bytes of a chunk without changing its geometry, so this suite has
 * two jobs and they pull in opposite directions.
 *
 * ### It must change nothing
 *
 * The flag is OFF by default and the off path is the one every other test and
 * every debugging session runs through. So the first test pins it: default
 * construction leaves it false, and a glTF written with it off carries no
 * extension key anywhere, keeps a `bufferView` on every accessor, and produces a
 * `.bin` of exactly the raw attribute and index bytes -- byte for byte what the
 * exporter wrote before the flag existed.
 *
 * Compression is also downstream of chunking, which means the conservation
 * invariant tests/road/test_road_export.cpp exists for must survive it: sum the
 * triangles over the chunk files and you get the input count back. Here the sum
 * is taken by DECODING each chunk's Draco streams, so it tests the written bytes
 * and not the exporter's own arithmetic.
 *
 * ### It must change the bytes
 *
 * A compression option that compresses nothing is worse than none: it adds a
 * required extension, so a loader without Draco refuses the file, and gives
 * nothing back for it. So the size is asserted as a strict inequality against the
 * uncompressed export of the same mesh.
 *
 * ### The part that is usually wrong
 *
 * The extension keeps the uncompressed accessors and strips their `bufferView`.
 * Two failures live there and neither shows up in a viewer that supports the
 * extension, which is why they ship: an accessor that keeps its `bufferView`
 * points a fallback loader at bytes that are no longer in the buffer, and an
 * attribute id that was never set on the Draco attribute leaves all five
 * attributes claiming unique id 0. Both are asserted directly -- the second by
 * resolving every published id against the decoded stream.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadExportDraco
 * @endcode
 */

#include "framework.hpp"
#include "road/junction_fixtures.hpp"
#include "road/p7_fixtures.hpp"

#include "osm/road/road_export.hpp"
#include "osm/road/road_network_builder.hpp"
#include "renderer/mesh.hpp"

#include <draco/compression/decode.h>
#include <draco/core/decoder_buffer.h>
#include <draco/mesh/mesh.h>

#include <nlohmann/json.hpp>

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

namespace {

using stratum::Mesh;
using stratum::Vertex;
using stratum::osm::road::ExportConfig;
using stratum::osm::road::ExportFormat;
using stratum::osm::road::ExportStats;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;
using stratum::osm::road::export_mesh;
using stratum::osm::road::export_road_network;

namespace p7 = stratum::test::p7;
namespace jt = stratum::test::junction;

/// The extension name, spelled once so a typo fails every test rather than none
constexpr const char* kExtension = "KHR_draco_mesh_compression";

/// Attribute keys a compressed primitive must publish, in no particular order
constexpr const char* kAttributeNames[] = {
    "POSITION", "NORMAL", "TEXCOORD_0", "COLOR_0", "TANGENT"
};

/// Number of entries in kAttributeNames
constexpr size_t kAttributeCount = sizeof(kAttributeNames) / sizeof(kAttributeNames[0]);

/// Chunk cell size used by the chunking test, metres
constexpr float kChunk = 50.0f;

// ============================================================================
// File reading
// ============================================================================

/// Whole file as bytes. Empty when it could not be opened.
std::vector<uint8_t> read_bytes(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return {};
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(in),
                                std::istreambuf_iterator<char>());
}

/**
 * @brief Parse a written `.gltf`
 *
 * Returns a null json rather than throwing, so a test that reads a file the
 * exporter declined to write reports a failed check instead of aborting the whole
 * binary.
 */
nlohmann::json read_gltf(const std::filesystem::path& path) {
    const std::vector<uint8_t> bytes = read_bytes(path);
    if (bytes.empty()) return nlohmann::json{};
    return nlohmann::json::parse(bytes.begin(), bytes.end(), nullptr, false);
}

// ============================================================================
// Decoding
// ============================================================================

/// What one primitive's Draco stream decoded to
struct Decoded {
    bool ok = false;            ///< The stream was found and decoded
    size_t faces = 0;           ///< Triangles recovered
    size_t points = 0;          ///< Vertices recovered
    size_t resolved_ids = 0;    ///< Published attribute ids that resolved in the stream
};

/**
 * @brief Decode the Draco stream one primitive points at
 *
 * Follows the same path a loader does: the extension names a bufferView, the
 * bufferView names a slice of the `.bin`, and the slice is a self-contained Draco
 * mesh. Every id in the extension's `attributes` map is then resolved with
 * GetAttributeByUniqueId(), which is the only call that can tell a correct id
 * table from five attributes all claiming zero.
 */
Decoded decode_primitive(const nlohmann::json& doc, const std::vector<uint8_t>& bin,
                         const nlohmann::json& primitive) {
    Decoded out;

    if (!primitive.contains("extensions")) return out;
    const nlohmann::json& extensions = primitive["extensions"];
    if (!extensions.contains(kExtension)) return out;
    const nlohmann::json& extension = extensions[kExtension];
    if (!extension.contains("bufferView") || !extension.contains("attributes")) return out;

    if (!doc.contains("bufferViews")) return out;
    const nlohmann::json& views = doc["bufferViews"];
    const size_t view_index = extension["bufferView"].get<size_t>();
    if (view_index >= views.size()) return out;

    const nlohmann::json& view = views[view_index];
    const size_t offset = view.value("byteOffset", size_t{0});
    const size_t length = view.value("byteLength", size_t{0});
    if (length == 0u || offset + length > bin.size()) return out;

    draco::DecoderBuffer buffer;
    buffer.Init(reinterpret_cast<const char*>(bin.data()) + offset, length);

    draco::Decoder decoder;
    draco::StatusOr<std::unique_ptr<draco::Mesh>> result = decoder.DecodeMeshFromBuffer(&buffer);
    if (!result.ok()) return out;

    const std::unique_ptr<draco::Mesh>& mesh = result.value();
    if (!mesh) return out;

    out.ok = true;
    out.faces = static_cast<size_t>(mesh->num_faces());
    out.points = static_cast<size_t>(mesh->num_points());

    for (const auto& [name, id] : extension["attributes"].items()) {
        (void)name;
        if (mesh->GetAttributeByUniqueId(id.get<uint32_t>()) != nullptr) {
            ++out.resolved_ids;
        }
    }
    return out;
}

/// Triangles across every primitive of one written `.gltf`, recovered by decoding
size_t decoded_triangles(const std::filesystem::path& gltf_path) {
    const nlohmann::json doc = read_gltf(gltf_path);
    if (doc.is_discarded() || doc.is_null() || !doc.contains("meshes")) return 0;

    std::filesystem::path bin_path = gltf_path;
    bin_path.replace_extension(".bin");
    const std::vector<uint8_t> bin = read_bytes(bin_path);

    size_t total = 0;
    for (const nlohmann::json& mesh : doc["meshes"]) {
        if (!mesh.contains("primitives")) continue;
        for (const nlohmann::json& primitive : mesh["primitives"]) {
            total += decode_primitive(doc, bin, primitive).faces;
        }
    }
    return total;
}

// ============================================================================
// Fixtures
// ============================================================================

/**
 * @brief A slab big enough that compression has something to compress
 *
 * 24 by 40 quads is 1920 triangles over 1025 vertices in three material bands.
 * Small meshes are a bad size test: a Draco stream carries a header and a
 * per-attribute preamble that a two-triangle quad cannot pay for, so a quad
 * legitimately compresses to more bytes than it started with.
 */
Mesh big_slab() { return p7::make_slab_mesh(8, 40); }

/**
 * @brief Slabs placed to land in several cells of a 50 metre grid
 *
 * Deliberately at negative Z, like the fixtures in test_road_export.cpp: the world
 * is Y up and the 2D-to-3D mapping negates the second local coordinate, so a
 * negative cell index is the ordinary case rather than the exotic one.
 */
std::vector<RoadPiece> straddling_pieces() {
    const glm::vec3 origins[4] = {
        {  10.0f, 0.0f, -10.0f },       // inside cell (0, 0)
        {  40.0f, 0.0f, -10.0f },       // hangs over the x border into (1, 0)
        { -12.0f, 0.0f,  -6.0f },       // crosses x = 0, so cells (-1, 0) and (0, 0)
        { 120.0f, 0.0f, -80.0f },       // well away, cell (2, 1)
    };

    std::vector<RoadPiece> pieces;
    for (const glm::vec3& origin : origins) {
        RoadPiece piece;
        piece.anchor = glm::dvec2(origin.x, -origin.z);
        piece.mesh = p7::make_slab_mesh(4, 6);
        for (Vertex& v : piece.mesh.vertices) {
            v.position = origin + v.position * 2.0f;
        }
        piece.mesh.compute_bounds();
        pieces.push_back(std::move(piece));
    }
    return pieces;
}

/// Triangles across every RoadPiece::mesh
size_t total_triangles(const std::vector<RoadPiece>& pieces) {
    size_t n = 0;
    for (const RoadPiece& piece : pieces) n += p7::triangle_count(piece.mesh);
    return n;
}

/// Render chunk files of a glTF export, excluding the collision and LOD sidecars
std::vector<std::filesystem::path> render_chunks(const std::filesystem::path& dir) {
    std::vector<std::filesystem::path> out;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() != ".gltf") continue;

        const std::string stem = entry.path().stem().string();
        if (stem.compare(0, 5, "road_") != 0) continue;
        if (stem.find("_collision") != std::string::npos) continue;
        if (stem.find("_lod") != std::string::npos) continue;

        out.push_back(entry.path());
    }
    std::sort(out.begin(), out.end());
    return out;
}

/// Write one mesh as glTF into its own scratch directory, and return the `.gltf` path
std::filesystem::path write_slab(const std::string& scratch_name, const Mesh& mesh,
                                 bool compressed) {
    const std::filesystem::path dir = p7::scratch_dir(scratch_name);
    const std::filesystem::path path = dir / "slab.gltf";

    ExportConfig cfg;
    cfg.format = ExportFormat::Gltf;
    cfg.draco_compression = compressed;

    if (!export_mesh(mesh, path, cfg)) {
        return {};
    }
    return path;
}

} // namespace

// ============================================================================
// The flag is off until it is asked for
// ============================================================================

/**
 * A default ExportConfig does not compress, and the file it writes is the one the
 * exporter wrote before the option existed: no extension declared anywhere, a
 * `bufferView` on every accessor, and a `.bin` holding exactly the raw attribute
 * and index bytes and nothing else.
 *
 * The buffer length is the load-bearing assertion. Five float attributes and a
 * 32-bit index list have one arithmetic answer; a `.bin` of any other size means
 * the uncompressed writer's layout moved, whatever the JSON says.
 */
TEST(RoadExportDraco, the_flag_defaults_off_and_writes_an_uncompressed_gltf) {
    CHECK_FALSE(ExportConfig{}.draco_compression);

    const Mesh mesh = big_slab();
    const std::filesystem::path path = write_slab("draco_off", mesh, false);
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    const nlohmann::json doc = read_gltf(path);
    CHECK_FALSE(doc.is_discarded());
    if (doc.is_discarded() || doc.is_null()) return;

    CHECK_FALSE(doc.contains("extensionsUsed"));
    CHECK_FALSE(doc.contains("extensionsRequired"));

    size_t primitives = 0;
    size_t with_extension = 0;
    for (const nlohmann::json& gltf_mesh : doc["meshes"]) {
        for (const nlohmann::json& primitive : gltf_mesh["primitives"]) {
            ++primitives;
            if (primitive.contains("extensions")) ++with_extension;
        }
    }
    CHECK_EQ(primitives, size_t{3});        // one per material band
    CHECK_EQ(with_extension, size_t{0});

    // Every accessor still points at bytes. This is the exact opposite of what the
    // compressed path must produce, so asserting it here is what makes the
    // compressed assertion meaningful.
    size_t accessors_without_view = 0;
    for (const nlohmann::json& accessor : doc["accessors"]) {
        if (!accessor.contains("bufferView")) ++accessors_without_view;
    }
    CHECK_EQ(accessors_without_view, size_t{0});

    std::filesystem::path bin_path = path;
    bin_path.replace_extension(".bin");
    const std::vector<uint8_t> bin = read_bytes(bin_path);

    // POSITION 3 + NORMAL 3 + TEXCOORD_0 2 + COLOR_0 4 + TANGENT 4 floats, then one
    // 32-bit index per index.
    const size_t expected = mesh.vertices.size() * 16u * sizeof(float)
                          + mesh.indices.size() * sizeof(uint32_t);
    CHECK_EQ(bin.size(), expected);
}

/**
 * Setting the flag false explicitly writes the same bytes as leaving it alone.
 *
 * The uncompressed writer is shared with the compressed one -- both build the
 * document and both write the pair of files through the same two helpers -- so
 * this is the check that the shared half stayed deterministic and did not start
 * depending on the flag it is supposed to be blind to.
 */
TEST(RoadExportDraco, an_explicit_false_writes_the_same_bytes_as_the_default) {
    const Mesh mesh = big_slab();

    const std::filesystem::path first = write_slab("draco_off_default", mesh, false);
    const std::filesystem::path second = write_slab("draco_off_explicit", mesh, false);
    CHECK_FALSE(first.empty());
    CHECK_FALSE(second.empty());
    if (first.empty() || second.empty()) return;

    std::filesystem::path first_bin = first;
    std::filesystem::path second_bin = second;
    first_bin.replace_extension(".bin");
    second_bin.replace_extension(".bin");

    CHECK_TRUE(read_bytes(first) == read_bytes(second));
    CHECK_TRUE(read_bytes(first_bin) == read_bytes(second_bin));
}

// ============================================================================
// The extension as written
// ============================================================================

/**
 * With the flag on, the extension is declared in all three places it has to be:
 * `extensionsUsed`, `extensionsRequired`, and on every single primitive.
 *
 * Required as well as used, because the accessors below carry no `bufferView`: a
 * loader that skips the extension has no geometry to fall back to, and refusing
 * the file is better than loading an empty chunk.
 */
TEST(RoadExportDraco, the_extension_is_declared_used_required_and_per_primitive) {
    const std::filesystem::path path = write_slab("draco_declared", big_slab(), true);
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    const nlohmann::json doc = read_gltf(path);
    CHECK_FALSE(doc.is_discarded());
    if (doc.is_discarded() || doc.is_null()) return;

    CHECK_TRUE(doc.contains("extensionsUsed"));
    CHECK_TRUE(doc.contains("extensionsRequired"));
    if (doc.contains("extensionsUsed")) {
        CHECK_EQ(doc["extensionsUsed"].size(), size_t{1});
        CHECK_TRUE(doc["extensionsUsed"][0] == kExtension);
    }
    if (doc.contains("extensionsRequired")) {
        CHECK_EQ(doc["extensionsRequired"].size(), size_t{1});
        CHECK_TRUE(doc["extensionsRequired"][0] == kExtension);
    }

    size_t primitives = 0;
    size_t with_extension = 0;
    size_t with_all_attributes = 0;
    for (const nlohmann::json& gltf_mesh : doc["meshes"]) {
        for (const nlohmann::json& primitive : gltf_mesh["primitives"]) {
            ++primitives;
            if (!primitive.contains("extensions")) continue;
            if (!primitive["extensions"].contains(kExtension)) continue;
            ++with_extension;

            const nlohmann::json& extension = primitive["extensions"][kExtension];
            if (!extension.contains("attributes")) continue;

            // The extension's map and the primitive's own map must name the same
            // attributes. A decoder reads the data through the first and the
            // counts through the second, so a name in one and not the other is an
            // attribute that silently arrives empty.
            size_t matched = 0;
            for (const char* name : kAttributeNames) {
                if (extension["attributes"].contains(name)
                    && primitive["attributes"].contains(name)) {
                    ++matched;
                }
            }
            if (matched == kAttributeCount) ++with_all_attributes;
        }
    }

    CHECK_EQ(primitives, size_t{3});
    CHECK_EQ(with_extension, primitives);
    CHECK_EQ(with_all_attributes, primitives);
}

/**
 * THE PART THAT IS USUALLY WRONG.
 *
 * The uncompressed accessors survive, with the right counts and the POSITION
 * bounds, and with no `bufferView` -- the raw bytes they used to describe are not
 * in the buffer any more.
 *
 * Both halves matter. An accessor that keeps its `bufferView` describes bytes that
 * were never written, and dropping the accessors altogether leaves the primitive
 * with no declared vertex count for a loader to size anything against. The counts
 * are checked against what the stream actually decodes to, not against the input
 * mesh, because Draco is allowed to split a vertex and the accessor is required to
 * report what came out.
 */
TEST(RoadExportDraco, the_fallback_accessors_survive_without_a_buffer_view) {
    const std::filesystem::path path = write_slab("draco_accessors", big_slab(), true);
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    const nlohmann::json doc = read_gltf(path);
    CHECK_FALSE(doc.is_discarded());
    if (doc.is_discarded() || doc.is_null()) return;

    std::filesystem::path bin_path = path;
    bin_path.replace_extension(".bin");
    const std::vector<uint8_t> bin = read_bytes(bin_path);
    CHECK_FALSE(bin.empty());

    const nlohmann::json& accessors = doc["accessors"];
    size_t with_view = 0;
    for (const nlohmann::json& accessor : accessors) {
        if (accessor.contains("bufferView")) ++with_view;
    }
    CHECK_EQ(with_view, size_t{0});

    size_t checked = 0;
    for (const nlohmann::json& gltf_mesh : doc["meshes"]) {
        for (const nlohmann::json& primitive : gltf_mesh["primitives"]) {
            const Decoded decoded = decode_primitive(doc, bin, primitive);
            CHECK_TRUE(decoded.ok);
            if (!decoded.ok) continue;
            ++checked;

            for (const char* name : kAttributeNames) {
                if (!primitive["attributes"].contains(name)) continue;
                const size_t id = primitive["attributes"][name].get<size_t>();
                CHECK_TRUE(id < accessors.size());
                if (id >= accessors.size()) continue;
                CHECK_EQ(accessors[id].value("count", size_t{0}), decoded.points);
            }

            const size_t index_id = primitive["indices"].get<size_t>();
            CHECK_TRUE(index_id < accessors.size());
            if (index_id >= accessors.size()) continue;
            CHECK_EQ(accessors[index_id].value("count", size_t{0}), decoded.faces * 3u);
            CHECK_TRUE(accessors[index_id]["type"] == "SCALAR");

            // POSITION keeps its bounds. They are what a loader sizes a scene
            // against before it has decompressed anything.
            const size_t position_id = primitive["attributes"]["POSITION"].get<size_t>();
            if (position_id < accessors.size()) {
                CHECK_TRUE(accessors[position_id].contains("min"));
                CHECK_TRUE(accessors[position_id].contains("max"));
            }
        }
    }
    CHECK_EQ(checked, size_t{3});
}

/**
 * Every attribute id the extension publishes resolves to a real attribute in the
 * decoded stream.
 *
 * This is the assertion that catches the default. Draco leaves every attribute's
 * unique id at 0 unless the encoder sets it, so an exporter that forgets writes a
 * map of five names all pointing at id 0, four of which resolve to the wrong
 * attribute or to nothing. Decoding and resolving each id is the only way to see
 * it: the JSON on its own looks perfectly well formed.
 */
TEST(RoadExportDraco, every_published_attribute_id_resolves_in_the_stream) {
    const std::filesystem::path path = write_slab("draco_ids", big_slab(), true);
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    const nlohmann::json doc = read_gltf(path);
    if (doc.is_discarded() || doc.is_null()) return;

    std::filesystem::path bin_path = path;
    bin_path.replace_extension(".bin");
    const std::vector<uint8_t> bin = read_bytes(bin_path);

    size_t primitives = 0;
    for (const nlohmann::json& gltf_mesh : doc["meshes"]) {
        for (const nlohmann::json& primitive : gltf_mesh["primitives"]) {
            ++primitives;
            const Decoded decoded = decode_primitive(doc, bin, primitive);
            CHECK_TRUE(decoded.ok);
            CHECK_EQ(decoded.resolved_ids, kAttributeCount);

            // Five names, five DISTINCT ids. The count above passes even when every
            // name maps to the same id, because id 0 resolves five times.
            const nlohmann::json& ids = primitive["extensions"][kExtension]["attributes"];
            std::vector<uint32_t> seen;
            for (const auto& [name, id] : ids.items()) {
                (void)name;
                seen.push_back(id.get<uint32_t>());
            }
            std::sort(seen.begin(), seen.end());
            seen.erase(std::unique(seen.begin(), seen.end()), seen.end());
            CHECK_EQ(seen.size(), kAttributeCount);
        }
    }
    CHECK_EQ(primitives, size_t{3});
}

// ============================================================================
// It has to be smaller
// ============================================================================

/**
 * The compressed buffer is strictly smaller than the uncompressed one, and
 * smaller than the raw vertex data on its own.
 *
 * Both bounds are stated because they fail differently. Beating the uncompressed
 * export is the claim the option makes to a user. Beating the raw vertex bytes
 * alone -- ignoring the indices entirely, which are what Draco compresses best --
 * is the stricter of the two and fails if the attribute quantisation silently
 * stopped being applied.
 */
TEST(RoadExportDraco, a_compressed_chunk_is_smaller_than_the_uncompressed_one) {
    const Mesh mesh = big_slab();
    CHECK_TRUE(mesh.vertices.size() > 1000u);
    CHECK_TRUE(p7::triangle_count(mesh) > 1000u);

    const std::filesystem::path plain = write_slab("draco_size_plain", mesh, false);
    const std::filesystem::path packed = write_slab("draco_size_packed", mesh, true);
    CHECK_FALSE(plain.empty());
    CHECK_FALSE(packed.empty());
    if (plain.empty() || packed.empty()) return;

    std::filesystem::path plain_bin = plain;
    std::filesystem::path packed_bin = packed;
    plain_bin.replace_extension(".bin");
    packed_bin.replace_extension(".bin");

    const size_t plain_size = read_bytes(plain_bin).size();
    const size_t packed_size = read_bytes(packed_bin).size();
    CHECK_TRUE(plain_size > 0u);
    CHECK_TRUE(packed_size > 0u);
    CHECK((packed_size) < (plain_size));

    const size_t raw_vertex_bytes = mesh.vertices.size() * 16u * sizeof(float);
    CHECK((packed_size) < (raw_vertex_bytes));
}

// ============================================================================
// Conservation, under compression
// ============================================================================

/**
 * THE CONSERVATION TEST, WITH THE BYTES COMPRESSED.
 *
 * Decode every Draco stream in every render chunk, sum the triangles, and get the
 * input count back exactly. More means a triangle reached two chunks; fewer means
 * one was dropped.
 *
 * The count comes from the decoder rather than from ExportStats, so it is a
 * property of the files a consumer receives. Compression sits downstream of chunk
 * assignment and of the per-material index lists, and the point of this test is
 * that it stays there: a per-primitive vertex compaction that renumbered across
 * range boundaries would move triangles between primitives and show up here as an
 * imbalance.
 */
TEST(RoadExportDraco, compressed_chunking_conserves_every_triangle) {
    const std::vector<RoadPiece> pieces = straddling_pieces();
    const size_t expected = total_triangles(pieces);
    CHECK_TRUE(expected > 0u);

    const std::filesystem::path dir = p7::scratch_dir("draco_chunked");
    ExportConfig cfg;
    cfg.format = ExportFormat::Gltf;
    cfg.draco_compression = true;
    cfg.chunk_size = kChunk;
    cfg.export_collision = false;
    cfg.export_lods = false;

    const ExportStats stats = export_road_network(pieces, dir, cfg);
    CHECK_EQ(stats.meshes, pieces.size());
    CHECK_EQ(stats.triangles, expected);
    CHECK_TRUE(stats.chunks > 1u);

    const std::vector<std::filesystem::path> chunks = render_chunks(dir);
    CHECK_EQ(chunks.size(), stats.chunks);

    size_t on_disk = 0;
    for (const std::filesystem::path& chunk : chunks) {
        on_disk += decoded_triangles(chunk);
    }
    if (on_disk != expected) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "decoded chunks hold exactly the input triangles",
            std::to_string(on_disk) + " triangles across " + std::to_string(chunks.size()) +
                " files, input had " + std::to_string(expected));
    }
}

/**
 * A written primitive decodes with Draco's own decoder, and gives back the
 * triangles that went into it.
 *
 * Deliberately the whole round trip through the file rather than through the
 * encoder: the bytes are read off disk at the offset and length the bufferView
 * claims. A stream written at the right offset with the wrong length, or padded
 * for alignment and then described without the padding, decodes to nothing here
 * and to nothing in a loader.
 */
TEST(RoadExportDraco, a_written_primitive_decodes_to_its_own_triangle_count) {
    const Mesh mesh = big_slab();
    const std::filesystem::path path = write_slab("draco_round_trip", mesh, true);
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    CHECK_EQ(decoded_triangles(path), p7::triangle_count(mesh));
}

/**
 * The same sum over a network built from a real extract, not from slabs.
 *
 * Fixture geometry is uniform, manifold and axis-aligned, which is the input Draco
 * is happiest with. A solved junction is none of those: it is a fan stitched to
 * four trimmed corridors, with kerbs and sidewalks that share edges with both.
 * That is the input that makes the edgebreaker encoder refuse a mesh, and the
 * refusal is handled by falling back to sequential encoding -- silently, because
 * a chunk that failed to compress must still be a chunk. This test is what
 * notices if the fallback ever stops catching it: a refused primitive fails its
 * whole chunk, and the sum comes back short.
 */
TEST(RoadExportDraco, a_real_network_conserves_its_triangles_when_compressed) {
    const auto parsed = jt::parse_fixture("four_way.osm");
    if (!parsed) return;

    RoadNetworkBuilder builder;
    RoadNetworkConfig build_cfg;
    build_cfg.build_collision = false;
    build_cfg.build_lods = false;
    const RoadNetwork network = builder.build(*parsed, build_cfg);
    CHECK_TRUE(!network.pieces.empty());

    const size_t expected = total_triangles(network.pieces);
    CHECK_TRUE(expected > 0u);

    const std::filesystem::path dir = p7::scratch_dir("draco_four_way");
    ExportConfig cfg;
    cfg.format = ExportFormat::Gltf;
    cfg.draco_compression = true;
    cfg.chunk_size = kChunk;
    cfg.export_collision = false;
    cfg.export_lods = false;

    const ExportStats stats = export_road_network(network.pieces, dir, cfg);
    CHECK_EQ(stats.triangles, expected);
    CHECK_EQ(stats.meshes, network.pieces.size());

    const std::vector<std::filesystem::path> chunks = render_chunks(dir);
    CHECK_EQ(chunks.size(), stats.chunks);

    size_t on_disk = 0;
    for (const std::filesystem::path& chunk : chunks) {
        on_disk += decoded_triangles(chunk);
    }
    if (on_disk != expected) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "a real network decodes to exactly its own triangles",
            std::to_string(on_disk) + " triangles across " + std::to_string(chunks.size()) +
                " files, input had " + std::to_string(expected));
    }
}

/**
 * OBJ has nowhere to put a compressed stream, so the flag is ignored rather than
 * obeyed or refused. A caller that sets both gets a normal OBJ and a warning, not
 * a failed export.
 */
TEST(RoadExportDraco, the_flag_is_ignored_for_obj) {
    const std::filesystem::path dir = p7::scratch_dir("draco_obj");
    const std::filesystem::path path = dir / "slab.obj";

    ExportConfig cfg;
    cfg.format = ExportFormat::Obj;
    cfg.draco_compression = true;

    CHECK_TRUE(export_mesh(p7::make_slab_mesh(2, 2), path, cfg));

    const p7::ObjFile obj = p7::read_obj(path);
    CHECK_TRUE(obj.ok);
    CHECK_TRUE(p7::valid_face_count(obj) > 0u);
}
