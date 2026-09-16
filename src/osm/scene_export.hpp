// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file scene_export.hpp
 * @brief Whole-scene export: any mesh, any kind, chunked by triangle centroid
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Roads export well and nothing else does. `osm/road/road_export.hpp` takes a
 * finished road network and writes it out chunked, material-preserving and
 * without ever duplicating a triangle; buildings, areas and terrain have no way
 * to reach that path at all, because its entry point takes `RoadPiece` and a
 * RoadPiece is something only the road network builder produces.
 *
 * This file is the same machine with the road-shaped socket replaced by a
 * mesh-shaped one. A SceneObject is a Mesh plus whatever the caller knows about
 * it, so a building, a park, a terrain tile and a road chunk all go in through
 * one door and come out of one exporter.
 *
 * ### The invariant, which is the whole reason the road exporter is shaped the
 * ### way it is
 *
 * Every triangle of the input appears in exactly ONE output chunk, chosen by its
 * CENTROID, and no triangle is ever split at a chunk boundary. Sum the triangle
 * counts over the chunks and you get the input count back.
 *
 * That is not a nicety. The pipeline's predecessor, `TileManager`, copied a whole
 * road into every tile its polyline touched and extruded it in full: a road
 * crossing four tiles was written out four times, and a consumer loading two
 * adjacent tiles drew the same carriageway twice, z-fighting with itself. P0
 * deleted that code. Generalising the exporter is exactly the moment the same bug
 * could come back wearing a different hat -- a building whose footprint straddles
 * a cell, emitted into both -- so the invariant is restated here and it is
 * asserted across mesh kinds rather than only within roads. See
 * tests/osm/test_scene_export.cpp, which recovers the triangles from the files on
 * disk rather than trusting the stats this header returns.
 *
 * A triangle whose centroid is in a cell is written there WHOLE and overhangs the
 * boundary by at most its own size, which is metres against a 500 m cell and is
 * invisible. Clipping at the boundary is the alternative, and it loses: it splits
 * one triangle into two or three, so the counts no longer match, the new vertices
 * carry interpolated attributes nobody authored, and a T-junction crack appears
 * between neighbouring chunks the moment either side is simplified.
 *
 * ### Per-object identity survives
 *
 * The road exporter merges everything in a chunk that shares a material into one
 * range, which is right for roads: a carriageway has no identity worth keeping
 * past the export, and one range per material is the fewest draw calls.
 *
 * A scene is not like that. Track I wants to read an export back and find the
 * building again, so a chunk here is grouped by (OBJECT, material) and each group
 * carries its object's metadata into the file -- an `o` record with comment lines
 * in OBJ, a primitive with `extras` in glTF. The cost is more primitives per
 * chunk, and it is the right way round: a consumer that wants fewer draw calls can
 * merge primitives sharing a material on import, but nothing can un-merge a file
 * to recover which building a triangle came from.
 *
 * ### Formats
 *
 * OBJ and glTF, matching the road writers, and deliberately no FBX -- that is H2.
 *
 * Also deliberately no KHR_draco_mesh_compression. The Draco encoder lives in the
 * anonymous namespace of `road_export.cpp`, so sharing it would mean editing that
 * file; see the note in the design section of the H1 report. An uncompressed
 * export is the one you want while geometry is being debugged anyway, which is why
 * the road exporter defaults it off.
 *
 * ### Library
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API and
 * no assimp. That is what makes an export testable with no GPU and no window, and
 * it is stated as a design constraint in road_export.hpp. `renderer/mesh.hpp` is a
 * plain data struct and is allowed.
 */

#pragma once

#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace stratum::osm {

// ============================================================================
// What goes in
// ============================================================================

/**
 * @brief What a SceneObject is, for grouping and for the metadata written out
 *
 * Coarse on purpose. It answers "what kind of thing is this" for a consumer
 * sorting an import into collections, and nothing in the exporter branches on it
 * except the name it writes. Anything finer -- the OSM `building=*` value, the
 * `landuse=*` value -- belongs in SceneObject::metadata, which is open-ended.
 */
enum class SceneObjectKind : uint8_t {
    Unknown = 0,    ///< Not stated. Exports normally; the metadata just says "Unknown".
    Building,       ///< Extruded footprint from osm::MeshBuilder::build_building_mesh()
    Area,           ///< Landuse, park, water, any filled polygon
    Road,           ///< Carriageway and its furniture, from osm::road::RoadPiece::mesh
    Terrain,        ///< Heightmap tile from procgen::TerrainMeshBuilder
    Count           ///< Sentinel: number of kinds. Never a valid kind.
};

/**
 * @brief Stable name of a kind, for the exported metadata
 *
 * These strings travel in exported files, so they are frozen exactly as
 * material_key_name()'s are: renaming one breaks whatever a consumer keyed off it.
 *
 * @param kind Kind to name
 * @return Name of the kind, or "Unknown" for SceneObjectKind::Count and
 *         out-of-range values. Never empty, never null.
 */
[[nodiscard]] const char* scene_object_kind_name(SceneObjectKind kind);

/**
 * @brief One exportable thing: a mesh, plus what the caller knows about it
 *
 * @warning This is a VIEW. @ref mesh is a non-owning pointer, and the Mesh it
 *          points at must outlive the export call. Meshes at city scale are the
 *          largest thing in the process and live in a quadtree leaf, a terrain
 *          tile or a RoadPiece; copying them into a vector to call an exporter
 *          would double the peak footprint of an import for the duration of the
 *          write. A null pointer is skipped, not dereferenced.
 *
 * Deliberately NOT a road type, an OSM type or a scene-graph type. The exporter
 * has to serve four producers that share nothing but triangles, and every one of
 * them can fill this in.
 */
struct SceneObject {
    /**
     * @brief Geometry in world space, Y up. Borrowed, never owned.
     *
     * World space matters: chunk assignment reads vertex positions directly and
     * applies no transform, so a mesh still in its own local frame lands in
     * whatever cell the origin happens to be in. Bake the placement in first.
     */
    const Mesh* mesh = nullptr;

    /// What this is. Written into the metadata; nothing branches on it.
    SceneObjectKind kind = SceneObjectKind::Unknown;

    /**
     * @brief Human name, from `name=*` or from the editor. May be empty.
     *
     * NOT an identity: two buildings may share a name, and many have none. The
     * exported object name appends the object's index for that reason, so two
     * "Church" objects stay distinguishable in the file.
     */
    std::string name;

    /**
     * @brief Originating OSM way or relation id, 0 when this came from nowhere
     *
     * The one field a consumer can round-trip against the source extract, which
     * is why it is a field of its own rather than one more entry in @ref metadata.
     * Terrain and editor-authored geometry leave it 0.
     */
    int64_t osm_id = 0;

    /**
     * @brief Layer this object belongs to, as a name. May be empty.
     *
     * A name and not a `scene::LayerId`: an id means nothing to a consumer reading
     * the file, and scene/layer.hpp is the editor's model of the tree rather than
     * something the exporter should need to be handed.
     */
    std::string layer;

    /**
     * @brief Open-ended key/value metadata, written out in order
     *
     * A vector of pairs and not a map, because the order is the caller's and it
     * survives into the file, which makes a diff of two exports readable. Keys are
     * not deduplicated -- a repeated key is written twice, exactly as given.
     *
     * Both halves are sanitised on the way into an OBJ comment, where a newline or
     * an `=` in a key would produce a line a reader cannot parse; glTF `extras`
     * takes them verbatim, since JSON escapes anything. Where the two disagree,
     * glTF is the lossless channel.
     */
    std::vector<std::pair<std::string, std::string>> metadata;
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Output container
 *
 * Spelled separately from osm::road::ExportFormat rather than reused. Reusing it
 * would mean including road_export.hpp -- and with it road_network_builder.hpp and
 * the whole road pipeline -- from a header whose entire point is that it does not
 * know what a road is. The two enums are kept in step by hand; see the design note
 * in the H1 report about factoring a shared config out of road_export.hpp, which
 * this task was not allowed to edit.
 */
enum class SceneExportFormat : uint8_t {
    /**
     * @brief Wavefront OBJ plus a sidecar MTL
     *
     * One `o` record per object in the chunk, one `usemtl` group per material
     * range inside it, and `# stratum:object` / `# stratum:meta` comment lines
     * carrying the metadata. Text, diffable, loadable by everything, and large.
     */
    Obj,

    /**
     * @brief glTF 2.0, `.gltf` JSON with an adjacent `.bin` buffer
     *
     * One primitive per (object, material) group, each with its own `extras`
     * holding that object's metadata, and one material per distinct MaterialKey in
     * the chunk. POSITION, NORMAL, TEXCOORD_0, COLOR_0 and TANGENT are written;
     * indices are unsigned int. Not `.glb`, so a chunk's geometry can be inspected
     * without a JSON parser, matching the road writer.
     */
    Gltf
};

/// What to write, how to cut it up, and what to call it
struct SceneExportConfig {
    SceneExportFormat format = SceneExportFormat::Obj;

    /**
     * @brief Chunk grid cell size in metres, on the world X and Z axes
     *
     * 0 disables chunking: the whole scene goes into one file. A negative value is
     * treated as 0.
     *
     * @note The grid is anchored at the WORLD ORIGIN, not at the scene's bounding
     *       box, so two exports of overlapping extracts produce chunks on the same
     *       lattice and line up. Anchoring on the data would move every cell
     *       boundary whenever the extract changed.
     *
     * 500 m matches road_export.hpp, and for the same reason: a chunk is the unit
     * a consuming engine streams, not a triangle budget.
     */
    float chunk_size = 500.0f;

    /**
     * @brief Write positions in the Y-up frame the meshes are already in
     *
     * The pipeline's world frame is Y up. false converts on the way out --
     * `(x, y, z) -> (x, -z, y)`, with normals and tangents rotated the same way --
     * for tools that want Z up. A rotation and not a mirror, so winding is
     * unaffected either way.
     *
     * @note glTF 2.0 REQUIRES Y up. Setting this false with SceneExportFormat::Gltf
     *       produces a file that is valid and wrong in every viewer, so it is
     *       ignored for glTF and a warning is logged.
     */
    bool y_up = true;

    /**
     * @brief Prefix for every emitted material name
     *
     * A material is named `<material_prefix><material_key_name(key)>`, the same
     * function the road exporter names its materials with. That is the point of
     * reaching into osm/road/road_style.hpp for it: a scene export and a road
     * export of the same city must produce the SAME material names, or an engine
     * binds textures twice and the road under a building is a different asphalt.
     */
    std::string material_prefix = "stratum_";

    /**
     * @brief Stem of every file name, and of the object names inside them
     *
     * `scene_<cx>_<cz>.obj`, `scene.mtl`, and object records named `scene_o<n>_...`.
     * Changing it lets two exports of different subsets -- say buildings and
     * terrain -- share one directory without colliding, which they otherwise would
     * because the chunk grid is the same for both.
     *
     * Sanitised like any other name before it reaches a path: an empty or
     * all-whitespace value falls back to "scene".
     */
    std::string name_prefix = "scene";

    /**
     * @brief Write per-object metadata into the output
     *
     * ON by default, because a file a consumer cannot attribute to anything is
     * most of the value of the export gone. Turning it off drops the OBJ comment
     * lines and the glTF `extras` and changes NOTHING about the geometry: the
     * (object, material) grouping, the chunk assignment and the triangle counts are
     * identical either way, so the conservation invariant does not depend on it.
     */
    bool write_metadata = true;
};

// ============================================================================
// Result
// ============================================================================

/// What an export produced, for logging and for the tests
struct SceneExportStats {
    /**
     * @brief Grid cells whose file was written
     *
     * Cells holding at least one triangle, minus any whose file could not be
     * opened, so this counts what a consumer actually receives. 1 when chunking is
     * disabled and anything was written.
     */
    size_t chunks = 0;

    /// Objects that contributed at least one triangle. An empty or null mesh is not counted.
    size_t objects = 0;

    /// Vertices written across every chunk, after per-chunk, per-object deduplication
    size_t vertices = 0;

    /**
     * @brief Triangles written across every chunk
     *
     * INVARIANT, and the one the tests exist to check: this equals the input
     * triangle count minus @ref dropped_triangles. Not more, which would mean a
     * triangle was duplicated across chunks, and not less, which would mean one was
     * dropped without being counted.
     */
    size_t triangles = 0;

    /**
     * @brief Input triangles that reached no chunk, and why they are counted
     *
     * A triangle is dropped when an index is out of range for its mesh, or when its
     * centroid is not finite -- a NaN position cannot be assigned to a cell, and
     * writing it would poison the chunk's bounding box.
     *
     * Reported rather than swallowed. `triangles + dropped_triangles` is the input
     * count exactly, so a caller can tell "the exporter conserved everything" from
     * "the exporter conserved what it kept", which a single written-triangle count
     * cannot. The road exporter drops the same geometry and does not say so, which
     * is the one thing this generalisation changes about the accounting.
     */
    size_t dropped_triangles = 0;

    /// Files written, including MTL sidecars and glTF `.bin` buffers
    size_t files = 0;

    /// Wall-clock time of the export, milliseconds
    double export_ms = 0.0;

    /**
     * @brief Absolute path of every file written, in the order written
     *
     * Always @ref files entries long. The order is deterministic: chunks ascend by
     * grid X then grid Z, and within a chunk the geometry file comes first, then its
     * `.bin`. The shared MTL is written once, after the first chunk that needed it.
     */
    std::vector<std::string> written_files;
};

// ============================================================================
// Entry points
// ============================================================================

/**
 * @brief Chunk a whole scene by triangle centroid and write one file per chunk
 *
 * ### Chunk assignment
 *
 * For each triangle of each SceneObject::mesh the centroid is the mean of its
 * three world-space vertex positions, and its cell is
 *
 * @code
 *     cx = floor(centroid.x / chunk_size)
 *     cz = floor(centroid.z / chunk_size)
 * @endcode
 *
 * on the WORLD X and Z axes -- the ground plane, since the world is Y up. The
 * triangle goes into that cell whole. Nothing routes by an object's centre, its
 * bounding box or its anchor: those assign a whole object to one cell, which puts
 * a 400 m terrain tile entirely in the chunk containing its middle, and they are
 * also how the old duplication bug was spelled.
 *
 * A chunk's file is assembled by copying only the vertices its triangles
 * reference, per object, so a vertex shared by triangles in two chunks is written
 * to both. That is duplication of VERTICES, which is unavoidable when a mesh is cut
 * up and costs a few bytes. It is not duplication of GEOMETRY.
 *
 * With `chunk_size` 0 the whole scene is one chunk at grid (0, 0), named without
 * coordinates.
 *
 * ### Grouping and order
 *
 * Inside a chunk, triangles are grouped by (object, MaterialKey) and the groups are
 * emitted in ascending object index, then ascending packed MaterialKey. Both keys
 * are ordered containers, so two exports of the same scene produce byte-identical
 * files; a hash map here would leak its iteration order into the output and make a
 * diff of two runs meaningless.
 *
 * ### File names
 *
 * Relative to @p out_dir, which is created if it does not exist, with `scene`
 * standing for SceneExportConfig::name_prefix:
 *
 * | What | Chunked | Unchunked |
 * |---|---|---|
 * | Geometry | `scene_<cx>_<cz>.obj` | `scene.obj` |
 * | OBJ materials | `scene.mtl`, once for the directory | `scene.mtl` |
 * | glTF buffer | `scene_<cx>_<cz>.bin` | `scene.bin` |
 *
 * Negative grid coordinates are written with a leading minus, so `scene_-3_12.obj`.
 * The extension follows SceneExportConfig::format. The MTL is written once per
 * export, lists every MaterialKey that appears anywhere in the scene, and every OBJ
 * references it with one `mtllib` line, so a chunk is not tied to a per-chunk
 * material file.
 *
 * ### Failure
 *
 * A file that cannot be opened is logged and skipped; the export continues with the
 * remaining chunks and the failed file does not appear in
 * SceneExportStats::written_files. A completely failed export comes back with
 * `files == 0` rather than throwing.
 *
 * @param objects Things to write. Entries with a null or empty mesh are skipped
 *                and are not counted in SceneExportStats::objects. The INDEX of an
 *                entry is its identity in the output, so the exported object names
 *                change if the caller reorders the vector.
 * @param out_dir Directory to write into. Created if missing.
 * @param cfg     Format, chunking and naming
 * @return Counts and the file list. Zeroed with an empty file list when @p objects
 *         is empty or holds no triangles.
 */
SceneExportStats export_scene(const std::vector<SceneObject>& objects,
                              const std::filesystem::path& out_dir,
                              const SceneExportConfig& cfg = {});

/**
 * @brief Write one object to one file, metadata included
 *
 * The convenience an "export this building" menu item and the golden tests want. No
 * chunking: SceneExportConfig::chunk_size is ignored and everything lands in one
 * file, whatever its extent. `format`, `y_up`, `material_prefix` and
 * `write_metadata` are read as usual. `name_prefix` does NOT name the file -- that
 * comes from @p out_path -- but it still names the object record inside it, so a
 * single-object file and the same object inside a chunked export are called the
 * same thing and a consumer can match them up.
 *
 * The format is taken from @p cfg and NOT from the path extension, so a caller who
 * asks for glTF and names the file `.obj` gets glTF in a badly named file rather
 * than a surprise. That is the road exporter's rule too.
 *
 * Parent directories of @p out_path are created if missing. An OBJ export writes a
 * sidecar MTL beside it, named after the output file with the extension replaced,
 * and a glTF export writes the matching `.bin`.
 *
 * @param object   Thing to write. A null or empty mesh is a failure, not a no-op
 *                 file.
 * @param out_path Destination file
 * @param cfg      Format and naming
 * @return true when the file and its sidecar were written. false when the mesh is
 *         null, holds no triangles, or a file could not be opened.
 */
bool export_scene_object(const SceneObject& object, const std::filesystem::path& out_path,
                         const SceneExportConfig& cfg = {});

// ============================================================================
// Adapters from the OSM types
// ============================================================================

/**
 * @brief Describe a parsed building, so its mesh exports with its identity
 *
 * Fills kind, name, osm_id and the metadata a consumer would otherwise have to
 * re-derive from the extract: the building type, its height and its level count.
 *
 * It exists so that the mapping from an OSM feature to exported metadata is
 * written ONCE. The editor, the CLI and Track I's reader all want the same keys,
 * and three hand-rolled copies drift.
 *
 * @param building Parsed feature. Its geometry is not read; only its attributes.
 * @param mesh     Mesh built for it, typically by MeshBuilder::build_building_mesh().
 *                 BORROWED -- it must outlive the returned SceneObject and the
 *                 export call that consumes it.
 * @return The described object, with SceneObject::layer left empty for the caller.
 */
[[nodiscard]] SceneObject describe_building(const Building& building, const Mesh& mesh);

/**
 * @brief Describe a parsed area, so its mesh exports with its identity
 *
 * As describe_building(), carrying the area type. @p mesh is BORROWED and must
 * outlive the returned SceneObject.
 *
 * @param area Parsed feature. Its geometry is not read; only its attributes.
 * @param mesh Mesh built for it, typically by MeshBuilder::build_area_mesh()
 * @return The described object, with SceneObject::layer left empty for the caller.
 */
[[nodiscard]] SceneObject describe_area(const Area& area, const Mesh& mesh);

} // namespace stratum::osm
