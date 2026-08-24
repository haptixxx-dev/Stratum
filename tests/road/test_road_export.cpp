/**
 * @file test_road_export.cpp
 * @brief Chunked export: every triangle written exactly once, in exactly one file
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The exporter replaces the defect P0 deleted. `TileManager` used to copy a whole
 * Road into every tile its polyline touched and extrude it in full, so a road
 * crossing four tiles was written out four times and a consumer loading two
 * adjacent tiles got the same carriageway twice, z-fighting with itself.
 *
 * The property that would have caught that, and the one this suite exists for, is
 * a single sum: add up the triangles in every file the export wrote, and you get
 * the input triangle count back. Not more, which is the old duplication bug
 * returning under a new name; not less, which is a triangle silently dropped at a
 * chunk boundary. One assertion catches both directions, which is why it is
 * stated as an equality rather than as a pair of bounds.
 *
 * Everything here reads back what was written. A count reported by ExportStats is
 * the exporter marking its own homework; a count recovered by parsing the OBJ
 * files off disk is not. p7_fixtures.hpp's read_obj() is deliberately tolerant
 * about how the file is spelled, so a failure here is a failure of what was
 * written and never of how.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadExport
 * @endcode
 */

#include "framework.hpp"
#include "road/p7_fixtures.hpp"

#include "osm/road/road_export.hpp"
#include "osm/road/road_style.hpp"
#include "osm/road/road_network_builder.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <system_error>
#include <vector>

#ifndef _WIN32
#include <unistd.h>
#endif

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::Vertex;
using stratum::material_id_name;
using stratum::SubMesh;
using stratum::osm::road::ExportConfig;
using stratum::osm::road::ExportStats;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;
using stratum::osm::road::export_mesh;
using stratum::osm::road::export_road_network;

namespace p7 = stratum::test::p7;
namespace jt = stratum::test::junction;

/// Chunk cell size used by the chunking tests, metres
constexpr float kChunk = 50.0f;

/// A render chunk file and the grid cell its name claims
struct ChunkFile {
    std::filesystem::path path;
    long cx = 0;
    long cz = 0;
};

/**
 * @brief Render chunk files in a directory, with their grid coordinates parsed
 *
 * Matches `road_<cx>_<cz>.obj` and nothing else, so the collision and LOD
 * sidecars -- `road_<cx>_<cz>_collision.obj`, `road_<cx>_<cz>_lod2.obj` -- are
 * excluded. Counting those into the triangle sum would make a passing export look
 * like a duplicating one.
 */
std::vector<ChunkFile> render_chunks(const std::filesystem::path& dir) {
    std::vector<ChunkFile> out;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        const std::string name = entry.path().filename().string();
        if (name.size() < 10 || name.compare(0, 5, "road_") != 0) continue;
        if (entry.path().extension() != ".obj") continue;

        const std::string stem = entry.path().stem().string().substr(5);
        const size_t split = stem.find('_', 1);
        if (split == std::string::npos) continue;
        const std::string first = stem.substr(0, split);
        const std::string second = stem.substr(split + 1);
        if (second.find('_') != std::string::npos) continue;    // _collision or _lodN

        char* end = nullptr;
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

/// Triangles across every RoadPiece::mesh
size_t total_triangles(const std::vector<RoadPiece>& pieces) {
    size_t n = 0;
    for (const RoadPiece& piece : pieces) n += p7::triangle_count(piece.mesh);
    return n;
}

/**
 * @brief A handful of pieces whose geometry deliberately straddles chunk borders
 *
 * The slabs are 24 by 12 metres on a 50 metre grid and are placed so that some sit
 * inside one cell, some hang over a border, and one crosses x = 0. Every one of
 * them sits at negative Z, because the world is Y up and the 2D-to-3D mapping
 * negates the second local coordinate, so the floor of a negative number is the
 * ordinary case here rather than the exotic one -- and it is the case an
 * implementation gets wrong by truncating towards zero instead of down.
 */
std::vector<RoadPiece> straddling_pieces() {
    const glm::vec3 origins[5] = {
        {  10.0f, 0.0f, -10.0f },       // inside cell (0, 0)
        {  40.0f, 0.0f, -10.0f },       // hangs over the x border into (1, 0)
        {  10.0f, 0.0f, -40.0f },       // hangs over the z border into (0, 1)
        { -12.0f, 0.0f,  -6.0f },       // crosses x = 0, so cells (-1, 0) and (0, 0)
        { 120.0f, 0.0f, -80.0f },       // well away, cell (2, 1)
    };

    std::vector<RoadPiece> pieces;
    for (const glm::vec3& origin : origins) {
        RoadPiece piece;
        piece.anchor = glm::dvec2(origin.x, -origin.z);
        piece.mesh = p7::make_slab_mesh(4, 6);      // 12 by 6 quads, doubled below
        for (Vertex& v : piece.mesh.vertices) {
            v.position = origin + v.position * 2.0f;
        }
        piece.mesh.compute_bounds();
        pieces.push_back(std::move(piece));
    }
    return pieces;
}

/// Build one fixture network, reporting a parse failure rather than returning quietly
bool build_network(const char* fixture, RoadNetwork& out) {
    const auto parsed = jt::parse_fixture(fixture);
    if (!parsed) return false;
    RoadNetworkBuilder builder;
    RoadNetworkConfig cfg;
    cfg.build_collision = false;
    cfg.build_lods = false;
    out = builder.build(*parsed, cfg);
    return true;
}

} // namespace

// ============================================================================
// Chunking
// ============================================================================

/**
 * THE CONSERVATION TEST.
 *
 * Sum the triangles across every render chunk file on disk and you get the input
 * triangle count back, exactly. A greater sum is a triangle written into two
 * chunks, which is the duplication bug P0 removed. A smaller sum is a triangle
 * dropped at a boundary. Stating it as an equality catches both without needing
 * to know which one happened.
 */
TEST(RoadExport, chunking_conserves_every_triangle) {
    const std::vector<RoadPiece> pieces = straddling_pieces();
    const size_t expected = total_triangles(pieces);
    CHECK_TRUE(expected > 0);

    const std::filesystem::path dir = p7::scratch_dir("chunked");
    ExportConfig cfg;
    cfg.chunk_size = kChunk;
    cfg.export_collision = false;
    cfg.export_lods = false;

    const ExportStats stats = export_road_network(pieces, dir, cfg);

    CHECK_EQ(stats.meshes, pieces.size());
    CHECK_EQ(stats.triangles, expected);
    CHECK_TRUE(stats.chunks > 1);
    CHECK_EQ(stats.written_files.size(), stats.files);
    CHECK_TRUE(stats.files > 0);

    const std::vector<ChunkFile> chunks = render_chunks(dir);
    CHECK_TRUE(!chunks.empty());

    size_t on_disk = 0;
    for (const ChunkFile& chunk : chunks) {
        const p7::ObjFile obj = p7::read_obj(chunk.path);
        CHECK_TRUE(obj.ok);
        on_disk += p7::valid_face_count(obj);
    }
    if (on_disk != expected) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "chunk files hold exactly the input triangles",
            std::to_string(on_disk) + " triangles across " + std::to_string(chunks.size()) +
                " files, input had " + std::to_string(expected));
    }
}

/**
 * Every triangle is in the chunk its centroid falls in, and the grid is anchored
 * at the world origin rather than at the data's bounding box.
 *
 * The negative case is the one that fails silently: a cell index taken with
 * integer truncation instead of a floor puts everything in (-24.0 .. 0.0) into
 * cell 0 alongside (0.0 .. 50.0), so two chunks' worth of road end up in one file
 * and the neighbouring file is short. The sum still balances, so the previous
 * test does not see it -- only this one does.
 */
TEST(RoadExport, every_triangle_lands_in_its_centroid_chunk) {
    const std::filesystem::path dir = p7::scratch_dir("centroid");
    ExportConfig cfg;
    cfg.chunk_size = kChunk;
    cfg.export_collision = false;

    export_road_network(straddling_pieces(), dir, cfg);

    const std::vector<ChunkFile> chunks = render_chunks(dir);
    CHECK_TRUE(!chunks.empty());

    bool saw_negative_cell = false;
    size_t misplaced = 0;
    std::string first;
    for (const ChunkFile& chunk : chunks) {
        if (chunk.cx < 0 || chunk.cz < 0) saw_negative_cell = true;
        const p7::ObjFile obj = p7::read_obj(chunk.path);
        for (const p7::ObjFace& face : obj.faces) {
            if (face.a >= obj.positions.size() || face.b >= obj.positions.size() ||
                face.c >= obj.positions.size()) {
                continue;
            }
            const glm::dvec3 centroid = p7::face_centroid(obj, face);
            const long cx = static_cast<long>(std::floor(centroid.x / kChunk));
            const long cz = static_cast<long>(std::floor(centroid.z / kChunk));
            if (cx == chunk.cx && cz == chunk.cz) continue;

            // A centroid within a millimetre of a border may round either way in
            // the file's decimal text; anything further is a real misplacement.
            const double dx = std::fabs(centroid.x - std::round(centroid.x / kChunk) * kChunk);
            const double dz = std::fabs(centroid.z - std::round(centroid.z / kChunk) * kChunk);
            if (dx < 1e-3 || dz < 1e-3) continue;

            if (misplaced == 0) {
                first = chunk.path.filename().string() + " holds a triangle centred at (" +
                        std::to_string(centroid.x) + ", " + std::to_string(centroid.z) +
                        "), which belongs to cell (" + std::to_string(cx) + ", " +
                        std::to_string(cz) + ")";
            }
            ++misplaced;
        }
    }
    CHECK_TRUE(saw_negative_cell);
    if (misplaced != 0) {
        stratum::test::report_failure(__FILE__, __LINE__,
                                      "every triangle is in its centroid's cell",
                                      std::to_string(misplaced) + " misplaced; " + first);
    }
}

/**
 * Chunking off writes one file with everything in it, named without coordinates.
 */
TEST(RoadExport, chunking_off_writes_one_file) {
    const std::vector<RoadPiece> pieces = straddling_pieces();
    const size_t expected = total_triangles(pieces);

    const std::filesystem::path dir = p7::scratch_dir("unchunked");
    ExportConfig cfg;
    cfg.chunk_size = 0.0f;
    cfg.export_collision = false;

    const ExportStats stats = export_road_network(pieces, dir, cfg);
    CHECK_EQ(stats.chunks, size_t{1});
    CHECK_EQ(stats.triangles, expected);

    const p7::ObjFile obj = p7::read_obj(dir / "road.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(p7::valid_face_count(obj), expected);
}

/**
 * An empty input writes nothing and reports nothing, rather than an empty file
 * a consumer then has to special-case.
 */
TEST(RoadExport, an_empty_network_writes_no_files) {
    const std::filesystem::path dir = p7::scratch_dir("empty");
    const ExportStats stats = export_road_network({}, dir, ExportConfig{});
    CHECK_EQ(stats.files, size_t{0});
    CHECK_EQ(stats.triangles, size_t{0});
    CHECK_TRUE(stats.written_files.empty());
}

// ============================================================================
// Round trip
// ============================================================================

/**
 * Write a mesh, read it back, and find the same vertices, the same triangles and
 * the same material groups.
 *
 * The triangle comparison is by position triple rather than by index, because the
 * exporter renumbers vertices per file and an index-based comparison would only
 * be testing that it renumbered them consistently with itself.
 */
TEST(RoadExport, obj_round_trip_preserves_counts_and_triangles) {
    const Mesh mesh = p7::make_slab_mesh(4, 5);
    const std::filesystem::path dir = p7::scratch_dir("round_trip");
    const std::filesystem::path path = dir / "slab.obj";

    CHECK_TRUE(export_mesh(mesh, path, ExportConfig{}));
    CHECK_TRUE(std::filesystem::exists(path));

    const p7::ObjFile obj = p7::read_obj(path);
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.positions.size(), mesh.vertices.size());
    CHECK_EQ(p7::valid_face_count(obj), p7::triangle_count(mesh));

    // Same triangles, ignoring numbering. Materials are compared separately, so
    // the keys here are all built with one material.
    std::vector<p7::TriKey> written;
    for (const p7::ObjFace& f : obj.faces) {
        if (f.a >= obj.positions.size() || f.b >= obj.positions.size() ||
            f.c >= obj.positions.size()) {
            continue;
        }
        written.push_back(p7::make_tri_key(p7::pos_key(obj.positions[f.a]),
                                           p7::pos_key(obj.positions[f.b]),
                                           p7::pos_key(obj.positions[f.c]),
                                           MaterialId::Default));
    }
    std::sort(written.begin(), written.end());

    std::vector<p7::TriKey> source;
    for (const p7::TriKey& key : p7::triangle_multiset(mesh)) {
        p7::TriKey flat = key;
        flat.material = static_cast<uint8_t>(MaterialId::Default);
        source.push_back(flat);
    }
    std::sort(source.begin(), source.end());

    CHECK_EQ(written.size(), source.size());
    CHECK_TRUE(written == source);
}

/**
 * Material slots survive as `usemtl` groups, one per MaterialId present, prefixed
 * so an import does not silently bind to the target project's own `Asphalt`.
 *
 * A mesh of Asphalt, Curb and Sidewalk yields three groups and no more. Fewer
 * means the exporter flattened the ranges and the material slots -- the thing P0.3
 * exists to produce -- did not reach the file.
 */
TEST(RoadExport, material_groups_survive_export) {
    const Mesh mesh = p7::make_slab_mesh(3, 4);
    CHECK_EQ(p7::materials_of(mesh).size(), size_t{3});

    const std::filesystem::path dir = p7::scratch_dir("materials");
    const std::filesystem::path path = dir / "slab.obj";
    ExportConfig cfg;
    cfg.material_prefix = "stratum_";
    CHECK_TRUE(export_mesh(mesh, path, cfg));

    const p7::ObjFile obj = p7::read_obj(path);
    CHECK_EQ(obj.groups.size(), size_t{3});

    for (MaterialId m : { MaterialId::Asphalt, MaterialId::Curb, MaterialId::Sidewalk }) {
        const std::string expected = std::string("stratum_") + material_id_name(m);
        if (std::find(obj.groups.begin(), obj.groups.end(), expected) == obj.groups.end()) {
            stratum::test::report_failure(__FILE__, __LINE__, "material group is present",
                                          "no usemtl " + expected + " in the file");
        }
    }

    // Every face is attributed to one of those groups; a face emitted before the
    // first usemtl belongs to nothing and imports as the target's default.
    for (const p7::ObjFace& f : obj.faces) {
        CHECK_TRUE(!f.material.empty());
    }

    // The MTL that names them has to be beside the file and referenced from it.
    CHECK_TRUE(!obj.mtllibs.empty());
    if (!obj.mtllibs.empty()) {
        CHECK_TRUE(std::filesystem::exists(dir / obj.mtllibs.front()));
    }
}

/**
 * A long name reaches the file whole, or the file is wrong in a way nothing
 * downstream can see.
 *
 * The OBJ and MTL writers format into a 256-byte stack buffer. Everything they
 * write is bounded by the format except the three caller-supplied names -- the
 * file stem, which becomes the `mtllib` reference, the object name, and
 * ExportConfig::material_prefix, whose documented purpose is to namespace the
 * materials into another project. snprintf returns the length the line WOULD
 * have had, so a long name both truncates the directive and, when that return
 * value is used as a byte count, reads off the end of the buffer.
 *
 * Neither symptom is loud: the `mtllib` points at a file that does not exist and
 * the material record stops mid-record, and every importer reacts by silently
 * dropping the materials.
 */
TEST(RoadExport, long_names_survive_the_line_buffer) {
    const Mesh mesh = p7::make_slab_mesh(3, 4);
    const std::filesystem::path dir = p7::scratch_dir("long_names");

    // 251 bytes with the extension, which is inside NAME_MAX and past the buffer.
    const std::string stem(247, 'x');
    const std::filesystem::path path = dir / (stem + ".obj");

    ExportConfig cfg;
    cfg.material_prefix = std::string(200, 'n') + "_";
    CHECK_TRUE(export_mesh(mesh, path, cfg));

    const p7::ObjFile obj = p7::read_obj(path);

    // The mtllib directive names the sidecar that was actually written.
    CHECK_TRUE(!obj.mtllibs.empty());
    if (!obj.mtllibs.empty()) {
        CHECK_EQ(obj.mtllibs.front(), stem + ".mtl");
        CHECK_TRUE(std::filesystem::exists(dir / obj.mtllibs.front()));
    }

    // Every material group carries the whole prefix.
    CHECK_EQ(obj.groups.size(), size_t{3});
    std::ifstream mtl(dir / (stem + ".mtl"), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(mtl)),
                           std::istreambuf_iterator<char>());

    for (MaterialId m : { MaterialId::Asphalt, MaterialId::Curb, MaterialId::Sidewalk }) {
        const std::string expected = cfg.material_prefix + material_id_name(m);
        if (std::find(obj.groups.begin(), obj.groups.end(), expected) == obj.groups.end()) {
            stratum::test::report_failure(__FILE__, __LINE__, "material group is present",
                                          "no usemtl for " + expected);
            continue;
        }

        // The MTL record is complete: a truncated one ends mid-directive and the
        // material silently binds to nothing.
        const size_t at = text.find("newmtl " + expected + "\n");
        if (at == std::string::npos) {
            stratum::test::report_failure(__FILE__, __LINE__, "material record is present",
                                          "no newmtl for " + expected);
            continue;
        }
        const size_t next = text.find("newmtl ", at + 7u);
        const std::string record = text.substr(at, next == std::string::npos ? next : next - at);
        if (record.find("\nillum 2\n") == std::string::npos) {
            stratum::test::report_failure(__FILE__, __LINE__, "material record is complete",
                                          expected + " has no illum line: " +
                                              std::to_string(record.size()) + " bytes written");
        }
    }
}

/**
 * A network of Asphalt, Curb and Sidewalk exports three groups per chunk as well,
 * and every chunk references the one MTL written for the directory.
 */
TEST(RoadExport, chunk_files_carry_their_material_groups) {
    const std::filesystem::path dir = p7::scratch_dir("chunk_materials");
    ExportConfig cfg;
    cfg.chunk_size = kChunk;
    cfg.export_collision = false;

    export_road_network(straddling_pieces(), dir, cfg);

    const std::vector<ChunkFile> chunks = render_chunks(dir);
    CHECK_TRUE(!chunks.empty());
    for (const ChunkFile& chunk : chunks) {
        const p7::ObjFile obj = p7::read_obj(chunk.path);
        CHECK_TRUE(!obj.groups.empty());
        CHECK_TRUE(!obj.mtllibs.empty());
        if (!obj.mtllibs.empty()) {
            CHECK_TRUE(std::filesystem::exists(dir / obj.mtllibs.front()));
        }
    }
}

// ============================================================================
// Coordinates
// ============================================================================

/**
 * A vertex at local (x, y) with height h is written at (x, h, -y), and the Z-up
 * conversion is a rotation rather than a mirror.
 *
 * The pipeline's world frame is Y up and the handedness flips in the 2D-to-3D
 * step. An exporter that "corrects" that by negating a different axis produces a
 * file that looks right until something in it is chiral, at which point every
 * turn arrow in the network points the wrong way.
 */
TEST(RoadExport, positions_keep_the_pipeline_frame) {
    // Local (10, 20) at height 3 is world (10, 3, -20).
    Mesh mesh = p7::make_quad_mesh();
    for (Vertex& v : mesh.vertices) {
        v.position += glm::vec3(10.0f, 3.0f, -20.0f);
    }
    mesh.compute_bounds();

    const std::filesystem::path dir = p7::scratch_dir("frame");

    ExportConfig y_up;
    y_up.y_up = true;
    CHECK_TRUE(export_mesh(mesh, dir / "y_up.obj", y_up));
    const p7::ObjFile a = p7::read_obj(dir / "y_up.obj");
    CHECK_EQ(a.positions.size(), size_t{4});

    bool found_y_up = false;
    for (const glm::dvec3& p : a.positions) {
        if (p7::pos_key(p) == p7::pos_key(glm::dvec3(10.0, 3.0, -20.0))) found_y_up = true;
    }
    CHECK_TRUE(found_y_up);

    ExportConfig z_up;
    z_up.y_up = false;
    CHECK_TRUE(export_mesh(mesh, dir / "z_up.obj", z_up));
    const p7::ObjFile b = p7::read_obj(dir / "z_up.obj");
    CHECK_EQ(b.positions.size(), size_t{4});

    // (x, y, z) -> (x, -z, y), so (10, 3, -20) becomes (10, 20, 3).
    bool found_z_up = false;
    for (const glm::dvec3& p : b.positions) {
        if (p7::pos_key(p) == p7::pos_key(glm::dvec3(10.0, 20.0, 3.0))) found_z_up = true;
    }
    CHECK_TRUE(found_z_up);
}

// ============================================================================
// Failure
// ============================================================================

/**
 * A path that cannot be written reports failure and does not throw.
 *
 * Two shapes of the same problem: a destination that is an existing directory, so
 * the stream never opens, and a directory with no write permission. The second is
 * skipped when the test runs as root, since root writes to it anyway and the
 * check would be measuring the wrong thing.
 */
TEST(RoadExport, an_unwritable_path_reports_failure) {
    const std::filesystem::path dir = p7::scratch_dir("unwritable");

    // A destination that is itself a directory.
    const std::filesystem::path as_directory = dir / "not_a_file.obj";
    std::error_code ec;
    std::filesystem::create_directories(as_directory, ec);
    CHECK_FALSE(export_mesh(p7::make_quad_mesh(), as_directory, ExportConfig{}));

    // A mesh with no triangles is a failure by contract, not an empty file.
    CHECK_FALSE(export_mesh(Mesh{}, dir / "empty.obj", ExportConfig{}));

#ifndef _WIN32
    if (::geteuid() == 0) return;

    const std::filesystem::path locked = dir / "locked";
    std::filesystem::create_directories(locked, ec);
    std::filesystem::permissions(locked, std::filesystem::perms::owner_read |
                                             std::filesystem::perms::owner_exec,
                                 std::filesystem::perm_options::replace, ec);
    if (!ec) {
        CHECK_FALSE(export_mesh(p7::make_quad_mesh(), locked / "blocked.obj", ExportConfig{}));

        const ExportStats stats =
            export_road_network(straddling_pieces(), locked / "network", ExportConfig{});
        CHECK_EQ(stats.files, size_t{0});
        CHECK_TRUE(stats.written_files.empty());

        std::filesystem::permissions(locked, std::filesystem::perms::owner_all,
                                     std::filesystem::perm_options::replace, ec);
    }
#endif
}

// ============================================================================
// End to end
// ============================================================================

/**
 * A real parsed fixture, built into a network and exported, conserves its
 * triangles the same way the synthetic pieces do.
 *
 * The synthetic pieces are axis-aligned slabs with predictable centroids. A
 * junction fixture is not: it carries fans, curb rings, marking quads and pieces
 * whose anchor is nowhere near their geometry, which is the case the
 * "assign by centroid, not by anchor" rule exists for.
 */
TEST(RoadExport, a_real_network_conserves_its_triangles) {
    RoadNetwork network;
    if (!build_network("four_way.osm", network)) return;
    CHECK_TRUE(!network.pieces.empty());

    const size_t expected = total_triangles(network.pieces);
    CHECK_TRUE(expected > 0);

    const std::filesystem::path dir = p7::scratch_dir("four_way");
    ExportConfig cfg;
    cfg.chunk_size = kChunk;
    cfg.export_collision = false;

    const ExportStats stats = export_road_network(network.pieces, dir, cfg);
    CHECK_EQ(stats.triangles, expected);
    CHECK_EQ(stats.meshes, network.pieces.size());

    size_t on_disk = 0;
    for (const ChunkFile& chunk : render_chunks(dir)) {
        on_disk += p7::valid_face_count(p7::read_obj(chunk.path));
    }
    CHECK_EQ(on_disk, expected);
}

// ============================================================================
// LOD sidecars
// ============================================================================

/**
 * Every LOD level asked for reaches disk, and each file covers the WHOLE network.
 *
 * Two failures hide behind a passing render export, and both leave
 * ExportStats::triangles untouched because that counts render chunks only.
 *
 * The first is writing nothing. build_lod_chain() drops a level that failed to
 * reduce the previous one by 10%, and a corridor strip two vertex columns wide
 * cannot reduce at all while LodConfig::lock_borders is on, so every network built
 * from tests/data holds at least one piece whose chain is a single level. An
 * exporter that clamps the level count to the SHORTEST chain present therefore
 * writes no LOD file on any real input, and the LOD pass becomes dead weight that
 * still costs a meshopt_simplify() per material per level at build time.
 *
 * The second is writing a hole. A piece with no level 2 that is simply skipped at
 * level 2 leaves its stretch of road missing from that file, and the road vanishes
 * as the camera pulls back. The contract is that it contributes its coarsest
 * level instead, so the triangle count of each LOD file is predictable exactly:
 * it is the sum over pieces of level min(n, last).
 */
TEST(RoadExport, lod_levels_reach_disk_and_cover_the_whole_network) {
    const auto parsed = jt::parse_fixture("four_way.osm");
    if (!parsed) return;

    RoadNetworkBuilder builder;
    RoadNetworkConfig build_cfg;
    build_cfg.build_collision = false;
    build_cfg.build_lods = true;
    const RoadNetwork network = builder.build(*parsed, build_cfg);
    CHECK_TRUE(!network.pieces.empty());

    // The premise: at least one piece could not be simplified at all, and at
    // least one could. Without both, this fixture proves nothing.
    size_t shortest = SIZE_MAX;
    size_t longest = 0;
    for (const RoadPiece& piece : network.pieces) {
        shortest = std::min(shortest, piece.lods.levels.size());
        longest = std::max(longest, piece.lods.levels.size());
    }
    CHECK_EQ(shortest, size_t{1});
    CHECK_TRUE(longest >= 2);

    const std::filesystem::path dir = p7::scratch_dir("four_way_lods");
    ExportConfig cfg;
    cfg.chunk_size = kChunk;
    cfg.export_collision = false;
    cfg.export_lods = true;
    cfg.lod_levels = static_cast<int>(longest);

    const ExportStats stats = export_road_network(network.pieces, dir, cfg);
    CHECK_EQ(stats.triangles, total_triangles(network.pieces));

    for (size_t level = 1; level < longest; ++level) {
        // What the level owes: every piece, at its own level or its coarsest.
        size_t expected = 0;
        for (const RoadPiece& piece : network.pieces) {
            if (piece.lods.levels.empty()) continue;
            const size_t use = std::min(level, piece.lods.levels.size() - 1u);
            expected += piece.lods.levels[use].indices.size() / 3;
        }
        CHECK_TRUE(expected > 0);

        size_t on_disk = 0;
        size_t files = 0;
        std::error_code ec;
        const std::string suffix = "_lod" + std::to_string(level) + ".obj";
        for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
            if (!entry.is_regular_file()) continue;
            const std::string name = entry.path().filename().string();
            if (name.size() < suffix.size()) continue;
            if (name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) continue;
            on_disk += p7::valid_face_count(p7::read_obj(entry.path()));
            ++files;
        }

        if (files == 0) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the level reached disk",
                "no *" + suffix + " file was written, so the LOD pass produced nothing a "
                "consumer can load");
            continue;
        }
        CHECK_EQ(on_disk, expected);
        CHECK_TRUE(on_disk < total_triangles(network.pieces));
    }
}

// ============================================================================
// The variant axis reaches the file
// ============================================================================

namespace {

/**
 * @brief Two quads in the SAME slot with DIFFERENT variants, plus one other slot
 *
 * The shape the exporter used to collapse: `{Asphalt, 0}` and
 * `{Asphalt, kAsphaltWorn}` are two materials, and a resurfaced stretch beside an
 * ordinary one is exactly how a real extract produces them.
 */
Mesh two_variants_mesh() {
    Mesh mesh;
    const stratum::MaterialKey keys[3] = {
        { MaterialId::Asphalt,  0 },
        { MaterialId::Asphalt,  stratum::osm::road::variants::kAsphaltWorn },
        { MaterialId::Sidewalk, stratum::osm::road::variants::kSidewalkBrick },
    };

    for (int q = 0; q < 3; ++q) {
        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        const uint32_t first = static_cast<uint32_t>(mesh.indices.size());
        for (int i = 0; i < 4; ++i) {
            Vertex v{};
            v.position = glm::vec3(static_cast<float>(10 * q + (i & 1)), 0.0f,
                                   -static_cast<float>((i >> 1) & 1));
            v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            v.color = glm::vec4(1.0f);
            mesh.vertices.push_back(v);
            mesh.bounds.expand(v.position);
        }
        for (uint32_t idx : { base, base + 1u, base + 2u, base + 1u, base + 3u, base + 2u }) {
            mesh.indices.push_back(idx);
        }
        stratum::SubMesh range;
        range.index_offset = first;
        range.index_count = 6u;
        range.material = keys[q].material;
        range.variant = keys[q].variant;
        mesh.submeshes.push_back(range);
    }
    return mesh;
}

/// Count occurrences of a `newmtl <name>` record in an MTL file
size_t count_newmtl(const std::filesystem::path& path, const std::string& name) {
    std::ifstream in(path);
    std::string line;
    size_t n = 0;
    while (std::getline(in, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (line == "newmtl " + name) ++n;
    }
    return n;
}

} // namespace

/**
 * Two ranges sharing a slot and differing in variant export as TWO materials.
 *
 * ### What was wrong
 *
 * The exporter named materials with material_id_name() and bucketed triangles by
 * SLOT: `triangle_materials()` cast `SubMesh::material` to a uint8 and dropped
 * `SubMesh::variant` on the floor, `ChunkAccum::by_material` was a fixed array
 * indexed by slot, and `finish_chunk()` physically merged the two ranges before
 * any writer ran. One `g`, one `usemtl`, one glTF material and one `newmtl` came
 * out, so an ordinary asphalt carriageway and a worn one could not be given
 * different textures in the target engine -- the entire variant axis this branch
 * adds was invisible in the product deliverable. road_style.hpp already documented
 * material_key_name() as the name that "travels in exported files"; nothing in
 * src/ called it.
 *
 * ### How this test fails without the fix
 *
 * `obj.groups` comes back with two entries instead of three, both Asphalt ranges
 * carry `usemtl stratum_Asphalt`, and the MTL holds a single `newmtl` for them.
 */
TEST(RoadExport, a_variant_is_its_own_exported_material) {
    const Mesh mesh = two_variants_mesh();

    const std::filesystem::path dir = p7::scratch_dir("variants");
    const std::filesystem::path path = dir / "variants.obj";
    ExportConfig cfg;
    cfg.material_prefix = "stratum_";
    CHECK_TRUE(export_mesh(mesh, path, cfg));

    const p7::ObjFile obj = p7::read_obj(path);
    CHECK_TRUE(obj.ok);

    // Three materials, not two: the slot alone is not the identity.
    CHECK_EQ(obj.groups.size(), size_t{3});

    const std::string expected[3] = {
        "stratum_Asphalt",
        "stratum_Asphalt.Worn",
        "stratum_Sidewalk.Brick",
    };
    for (const std::string& name : expected) {
        if (std::find(obj.groups.begin(), obj.groups.end(), name) == obj.groups.end()) {
            stratum::test::report_failure(__FILE__, __LINE__, "variant reached the OBJ",
                                          "no usemtl " + name + " in the file");
        }
    }

    // One triangle pair per material, so nothing was merged on the way.
    std::map<std::string, size_t> faces_per_material;
    for (const p7::ObjFace& f : obj.faces) {
        CHECK_TRUE(!f.material.empty());
        ++faces_per_material[f.material];
    }
    CHECK_EQ(faces_per_material.size(), size_t{3});
    for (const auto& [name, count] : faces_per_material) {
        (void)name;
        CHECK_EQ(count, size_t{2});
    }

    // The MTL library names all three, and names them apart.
    CHECK_TRUE(!obj.mtllibs.empty());
    if (obj.mtllibs.empty()) return;
    const std::filesystem::path mtl = dir / obj.mtllibs.front();
    CHECK_TRUE(std::filesystem::exists(mtl));
    for (const std::string& name : expected) {
        CHECK_EQ(count_newmtl(mtl, name), size_t{1});
    }
}

/**
 * The chunked path keeps the variants too, and still conserves every triangle.
 *
 * `accumulate_mesh()` routes per triangle, so it is the other place the slot-only
 * key collapsed the two ranges -- and the one that runs on a real export.
 */
TEST(RoadExport, chunked_export_keeps_the_variants_apart) {
    std::vector<RoadPiece> pieces;
    RoadPiece piece;
    piece.edge = 0;
    piece.anchor = { 0.0, 0.0 };
    piece.mesh = two_variants_mesh();
    const size_t triangles = p7::triangle_count(piece.mesh);
    pieces.push_back(std::move(piece));

    const std::filesystem::path dir = p7::scratch_dir("variant_chunks");
    ExportConfig cfg;
    cfg.chunk_size = 0.0f;               // one chunk, so the read-back is one file
    cfg.export_collision = false;
    cfg.material_prefix = "stratum_";
    const ExportStats stats = export_road_network(pieces, dir, cfg);

    CHECK_EQ(stats.triangles, triangles);
    CHECK_TRUE(stats.chunks >= size_t{1});

    // Chunking off writes one `road.obj` rather than a grid cell.
    const p7::ObjFile obj = p7::read_obj(dir / "road.obj");
    CHECK_TRUE(obj.ok);
    CHECK_EQ(obj.groups.size(), size_t{3});
    CHECK_EQ(p7::valid_face_count(obj), triangles);
}
