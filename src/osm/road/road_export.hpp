/**
 * @file road_export.hpp
 * @brief Writing the finished network to disk: clip once, assign once, one file per chunk
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The last step of P7, and the one the whole pipeline was rearranged for.
 *
 * The original road builder produced geometry PER SPATIAL-INDEX TILE, and
 * `TileManager` copied a whole Road into every tile its polyline touched before
 * extruding it in full. A road crossing four tiles was extruded four times, four
 * copies were written out, and a consumer loading two adjacent tiles got the same
 * carriageway twice, z-fighting with itself. P0 deleted that. This file is what
 * replaces it, and it is careful not to reinvent the same bug in a new place:
 *
 * - **Whole triangles only.** No triangle is ever split by a chunk boundary. A
 *   triangle spanning two chunks belongs to the chunk containing its CENTROID and
 *   overhangs the boundary a little, which is invisible and costs nothing.
 * - **Assigned exactly once.** Every triangle of the input appears in exactly one
 *   output file. Sum the triangle counts over the chunks and you get the input
 *   count back -- that is the property the tests assert, and it is the one the old
 *   code broke.
 *
 * ### Formats
 *
 * OBJ because it is the format you can read in a text editor when the mesh is
 * wrong, and because the existing tests already dump OBJ (`tests/obj_dump.hpp`).
 * glTF because it is what a game engine actually ingests.
 *
 * glTF is written by hand as JSON plus a binary buffer, using nlohmann_json,
 * which stratum_core already links. It deliberately does NOT go through assimp:
 * assimp is linked into stratum_editor_lib, not stratum_core, and export must stay
 * core so it is testable with no GPU and no window. draco is linked into core and
 * is NOT used here; mesh compression is a later concern and an uncompressed glTF
 * is what you want while the geometry is still being debugged.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/road_network_builder.hpp"
#include "renderer/mesh.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Output container
 */
enum class ExportFormat : uint8_t {
    /**
     * @brief Wavefront OBJ plus a sidecar MTL
     *
     * One `usemtl` directive per SubMesh range, so material slots survive. Text,
     * diffable, and loadable by everything. Large: a city extract is hundreds of
     * megabytes of ASCII.
     */
    Obj,

    /**
     * @brief glTF 2.0, `.gltf` JSON with an adjacent `.bin` buffer
     *
     * One primitive per SubMesh range, one material per MaterialId actually used.
     * Positions, normals, UV0, COLOR_0 and TANGENT are written; indices are
     * unsigned int. Not `.glb`, because a separate `.bin` lets a chunk's geometry
     * be inspected without a JSON parser.
     */
    Gltf
};

/**
 * @brief What to write, how to cut it up, and what to call the materials
 */
struct ExportConfig {
    ExportFormat format = ExportFormat::Obj;

    /**
     * @brief Chunk grid cell size in metres, on the world X and Z axes
     *
     * 0 disables chunking: the whole network goes into one file. Anything else
     * must be positive; a negative value is treated as 0.
     *
     * 500 m is chosen against the streaming granularity, not against a triangle
     * budget. A chunk is the unit a consuming engine loads and unloads, and at
     * 500 m a chunk of dense urban road is a few megabytes.
     *
     * @note The grid is anchored at the world origin, not at the network's
     *       bounding box. Two exports of overlapping extracts therefore produce
     *       chunks on the same lattice and line up, which they would not if the
     *       origin moved with the data.
     */
    float chunk_size = 500.0f;

    /**
     * @brief Also write the collision variant of each chunk
     *
     * Reads RoadPiece::collision. A piece with an empty collision mesh contributes
     * nothing, so this is a no-op unless the network was built with
     * RoadNetworkConfig::build_collision.
     */
    bool export_collision = true;

    /**
     * @brief Also write the LOD levels of each chunk
     *
     * Reads RoadPiece::lods. Level 0 is already the render mesh and is not written
     * again, so this adds up to `lod_levels - 1` files per chunk. A no-op unless
     * the network was built with RoadNetworkConfig::build_lods.
     */
    bool export_lods = false;

    /**
     * @brief How many levels to write, counting the full-detail one
     *
     * Clamped to the length of the LONGEST LodChain present, since level 0 is the
     * render file and is never rewritten. Values below 2 write no LOD files.
     *
     * A piece whose own chain is shorter than the level being written contributes
     * its COARSEST level, so every LOD file covers the whole network. It has to:
     * build_lod_chain() drops a level that failed to reduce by 10%, a corridor
     * strip two vertex columns wide cannot reduce at all under
     * LodConfig::lock_borders, and a piece like that is present in every network.
     * Clamping to the shortest chain would write no LOD file at all, and skipping
     * the piece would leave a hole in the one that was written.
     */
    int lod_levels = 3;

    /**
     * @brief Write positions in the Y-up frame the meshes are already in
     *
     * The pipeline's world frame is Y up: local 2D `(x, y)` maps to
     * `vec3(x, height, -y)`, and the handedness flips in that step. Mesh positions
     * arrive here already in that frame, so true writes them through unchanged.
     *
     * false converts to Z up on the way out -- `(x, y, z) -> (x, -z, y)` -- for
     * tools that want it. Normals and tangents are rotated the same way. This is a
     * rotation, not a mirror, so winding is unaffected either way.
     *
     * @note glTF 2.0 REQUIRES Y up. Setting this false with ExportFormat::Gltf
     *       produces a file that is valid but wrong in every viewer, so it is
     *       ignored for glTF and a warning is logged.
     */
    bool y_up = true;

    /**
     * @brief Prefix for every emitted material name
     *
     * A material is named `<material_prefix><MaterialId name>`, using
     * material_id_name(), so MaterialId::Asphalt becomes `stratum_Asphalt` by
     * default. The prefix exists so an import into a project with its own
     * `Asphalt` material does not silently bind to it.
     */
    std::string material_prefix = "stratum_";
};

// ============================================================================
// Result
// ============================================================================

/**
 * @brief What an export produced, for logging and for the tests
 */
struct ExportStats {
    /**
     * @brief Grid cells whose render file was written
     *
     * A cell holding at least one triangle, minus any whose file could not be
     * opened, so this counts what a consumer actually receives rather than what
     * was intended. 1 when chunking is disabled and anything was written.
     */
    size_t chunks = 0;

    /**
     * @brief Meshes consumed from the input
     *
     * Counts render meshes only, one per contributing RoadPiece. Collision meshes
     * and LOD levels are not counted here; `files` reflects them.
     */
    size_t meshes = 0;

    /// Vertices written across every render chunk, after per-chunk deduplication
    size_t vertices = 0;

    /**
     * @brief Triangles written across every render chunk
     *
     * INVARIANT, and the one the tests exist to check: this equals the total
     * triangle count of every RoadPiece::mesh in the input. Not more, which would
     * mean a triangle was duplicated across chunks, and not less, which would mean
     * one was dropped. Collision and LOD triangles are excluded.
     */
    size_t triangles = 0;

    /// Files written, including MTL sidecars and glTF `.bin` buffers
    size_t files = 0;

    /// Wall-clock time of the export, milliseconds
    double export_ms = 0.0;

    /**
     * @brief Absolute path of every file written, in the order written
     *
     * Always `files` entries long. The order is deterministic: chunks ascend by
     * grid X then grid Z, and within a chunk the render file comes first, then its
     * sidecar, then collision, then LODs in ascending level.
     */
    std::vector<std::string> written_files;
};

// ============================================================================
// Entry points
// ============================================================================

/**
 * @brief Chunk a road network by triangle centroid and write one file per chunk
 *
 * ### Chunk assignment
 *
 * For each triangle of each RoadPiece::mesh, the centroid is the mean of its three
 * world-space vertex positions. Its cell is
 *
 * @code
 *     cx = floor(centroid.x / chunk_size)
 *     cz = floor(centroid.z / chunk_size)
 * @endcode
 *
 * on the WORLD X and Z axes -- the ground plane, since the world is Y up. The
 * triangle goes into that cell whole. RoadPiece::anchor is deliberately not used:
 * anchoring routes a whole piece by one point, which is right for the quadtree,
 * where a piece must stay indivisible, and wrong here, where a 400 m edge piece
 * would otherwise land entirely in the chunk containing its middle.
 *
 * Each chunk's file is assembled by copying only the vertices its triangles
 * reference, so vertices shared by triangles in two chunks are written to both.
 * That is duplication of VERTICES, which is unavoidable and cheap, and is not
 * duplication of GEOMETRY. SubMesh ranges are rebuilt per chunk in ascending
 * MaterialId order.
 *
 * With `chunk_size` 0 the whole network is one chunk at grid (0, 0) and the file
 * is named without coordinates.
 *
 * ### File names
 *
 * Relative to @p out_dir, which is created if it does not exist:
 *
 * | What | Chunked | Unchunked |
 * |---|---|---|
 * | Render | `road_<cx>_<cz>.obj` | `road.obj` |
 * | Collision | `road_<cx>_<cz>_collision.obj` | `road_collision.obj` |
 * | LOD n >= 1 | `road_<cx>_<cz>_lod<n>.obj` | `road_lod<n>.obj` |
 * | OBJ materials | `road.mtl`, once for the directory | `road.mtl` |
 * | glTF buffer | `road_<cx>_<cz>.bin` | `road.bin` |
 *
 * Negative grid coordinates are written with a leading minus, so `road_-3_12.obj`.
 * The extension follows ExportConfig::format. The MTL is written once per export,
 * lists every MaterialId that appears anywhere in the network, and every OBJ
 * references it with one `mtllib road.mtl` line, so a chunk is not tied to a
 * per-chunk material file.
 *
 * ### Failure
 *
 * A file that cannot be opened is logged and skipped; the export continues with
 * the remaining chunks and the failed file does not appear in
 * ExportStats::written_files. A completely failed export therefore comes back with
 * `files == 0` rather than throwing.
 *
 * @param pieces  Finished network pieces, typically RoadNetwork::pieces. Both edge
 *                and junction pieces are exported; a piece's provenance is
 *                irrelevant to the exporter and nothing branches on
 *                RoadPiece::edge.
 * @param out_dir Directory to write into. Created if missing.
 * @param cfg     Format, chunking and naming
 * @return Counts and the file list. Zeroed with an empty file list when @p pieces
 *         is empty or holds no triangles.
 */
ExportStats export_road_network(const std::vector<RoadPiece>& pieces,
                                const std::filesystem::path& out_dir,
                                const ExportConfig& cfg = {});

/**
 * @brief Write a single mesh to a single file
 *
 * The convenience the golden tests and the OBJ dumps use. No chunking, no
 * collision, no LODs: ExportConfig::chunk_size, export_collision and export_lods
 * are ignored, and only `format`, `y_up` and `material_prefix` are read. The
 * format is taken from @p cfg rather than from the path extension, so a caller
 * that asks for glTF and names the file `.obj` gets glTF in a badly named file
 * rather than a surprise.
 *
 * Parent directories of @p out_path are created if missing. An OBJ export writes
 * a sidecar MTL beside it, named after the output file with the extension
 * replaced, and a glTF export writes the matching `.bin`. Those sidecars are not
 * reported separately -- the return value is a single success flag.
 *
 * @param mesh     Geometry to write. SubMesh ranges become material groups; an
 *                 untagged mesh writes one MaterialId::Default group.
 * @param out_path Destination file
 * @param cfg      Format and naming
 * @return true when the file and its sidecar were written. false when @p mesh has
 *         no triangles, or the file could not be opened.
 */
bool export_mesh(const Mesh& mesh, const std::filesystem::path& out_path,
                 const ExportConfig& cfg = {});

} // namespace stratum::osm::road
