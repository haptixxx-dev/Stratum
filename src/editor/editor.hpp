#pragma once

#include <imgui.h>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "osm/parser.hpp"
#include "osm/mesh_builder.hpp"
#include "osm/quadtree.hpp"
#include "osm/road/road_export.hpp"
#include "osm/road/road_network_builder.hpp"
#include "procgen/terrain_generator.hpp"
#include "procgen/terrain_mesh_builder.hpp"
#include "procgen/terrain_tile_manager.hpp"
#include "renderer/mesh.hpp"
#include "editor/camera.hpp"

namespace stratum {

// Forward declaration
class GPURenderer;

class Editor {
public:
    Editor() = default;
    ~Editor() = default;

    void init();
    void shutdown();
    void update();
    void render();
    void render_3d(GPURenderer& renderer);

    void set_quit_callback(std::function<void()> callback) { m_quit_callback = callback; }
    void set_window_handle(void* window) { m_window_handle = window; }

    /**
     * @brief Attach the GPU renderer and create renderer-dependent resources
     * @note Out-of-line because it also initializes the Im3d GPU backend, which
     *       cannot happen in init() - that runs before the renderer is attached.
     */
    void set_renderer(GPURenderer* renderer);

    /**
     * @brief End the Im3d frame and upload its geometry
     * @note Must be called with NO render pass active (it opens a copy pass).
     */
    void im3d_end_frame_and_upload(GPURenderer& renderer);
    void set_msaa_change_callback(std::function<void(int)> callback) { m_msaa_change_callback = callback; }

    bool is_viewport_focused() const { return m_viewport_focused; }
    bool is_viewport_hovered() const { return m_viewport_hovered; }

private:
    void setup_dockspace();
    void draw_menu_bar();
    void draw_viewport();
    void draw_scene_hierarchy();
    void draw_properties();
    void draw_console();
    void draw_osm_panel();
    void draw_procgen_panel();
    void draw_toolbar();
    void draw_render_settings();
    void draw_memory_panel();
    
    // Procgen helpers
    void generate_terrain();
    void clear_terrain();
    void generate_chunked_terrain();
    void clear_chunked_terrain();
    void draw_chunked_terrain_ui();
    void draw_legacy_terrain_ui();

    bool m_viewport_focused = false;
    bool m_viewport_hovered = false;
    bool m_show_demo_window = false;
    bool m_show_style_editor = false;

    // Panel visibility
    bool m_show_viewport = true;
    bool m_show_scene_hierarchy = true;
    bool m_show_properties = true;
    bool m_show_console = true;
    bool m_show_osm_panel = true;
    bool m_show_procgen_panel = true;
    bool m_show_render_settings = false;
    bool m_show_memory_panel = false;

    // Render toggles
    bool m_render_areas = true;
    bool m_render_roads = true;
    bool m_render_buildings = true;
    bool m_show_tile_grid = false;
    // Re-submit the batched OSM geometry through Im3d as debug triangles. Off by
    // default: render_3d() already draws the same quadtree geometry as GPU meshes,
    // so enabling this double-draws the whole scene.
    bool m_im3d_debug_geometry = false;

    // Console log
    ImGuiTextBuffer m_console_buffer;
    bool m_console_scroll_to_bottom = true;

    // Core systems
    Camera m_camera;
    float m_last_time = 0.0f;

    // Callbacks
    std::function<void()> m_quit_callback;
    std::function<void(int)> m_msaa_change_callback;
    void* m_window_handle = nullptr;

    // Window dragging state
    bool m_dragging_window = false;
    ImVec2 m_drag_start_mouse;
    int m_drag_start_window_x = 0;
    int m_drag_start_window_y = 0;

    GPURenderer* m_gpu_renderer = nullptr;

    // Window resizing state
    enum ResizeEdge { RESIZE_NONE = 0, RESIZE_LEFT, RESIZE_RIGHT, RESIZE_TOP, RESIZE_BOTTOM,
                      RESIZE_TOPLEFT, RESIZE_TOPRIGHT, RESIZE_BOTTOMLEFT, RESIZE_BOTTOMRIGHT };
    ResizeEdge m_resize_edge = RESIZE_NONE;
    int m_resize_start_w = 0;
    int m_resize_start_h = 0;
    // Drag origin in GLOBAL (desktop) coordinates. Window-relative coordinates
    // shift underneath the cursor as the window moves, which feeds back into the
    // delta and makes left/top edge drags oscillate.
    float m_resize_start_global_x = 0.0f;
    float m_resize_start_global_y = 0.0f;

    bool m_fullscreen = false;

    void handle_window_resize();
    void toggle_fullscreen();

    // OSM Parser and QuadTree
    osm::OSMParser m_osm_parser;
    osm::QuadTree m_quadtree;
    std::string m_osm_import_path;
    bool m_use_tile_culling = true;
    bool m_use_distance_culling = true;
    bool m_use_contribution_culling = true;
    float m_view_radius = 2000.0f;
    float m_contribution_threshold = 4.0f;

    // Cached meshes for rendering (legacy - the live path is m_quadtree)
    std::vector<Mesh> m_building_meshes;
    std::vector<Mesh> m_road_meshes;
    std::vector<Mesh> m_area_meshes;

    // ── Async OSM import ────────────────────────────────────────────────────
    // Parsing and road network building run on worker threads; everything that
    // touches the quadtree, the camera or GPU resources stays on the main
    // thread. Progress is surfaced as a four-stage bar:
    // parse -> road network -> spatial index -> mesh build.
    enum class ImportStage {
        Idle,
        Parsing,
        BuildingRoads,   ///< RoadNetworkBuilder on a worker, over the whole graph
        Indexing,
        BuildingMeshes,
        CarvingTerrain,  ///< Corridors indexed on a worker, then carved into the chunks
        Done,
        Failed
    };

    struct OSMImportJob {
        // Owns its own parser so the worker never touches Editor::m_osm_parser,
        // which the UI reads every frame. Moved into place once parsing succeeds.
        std::unique_ptr<osm::OSMParser> parser;
        std::string filepath;

        std::mutex mutex;             ///< Guards `progress` (written on the worker)
        osm::ParseProgress progress;

        // MUST be declared last. Members destruct in reverse declaration order, and
        // ~future on an std::async future blocks until the worker finishes. Declared
        // last means it is destroyed first, so the worker is joined while `parser`
        // and `filepath` are still alive. Move it earlier and a job destroyed
        // mid-parse is a use-after-free.
        std::future<bool> future;
    };

    std::unique_ptr<OSMImportJob> m_import_job;

    // Road geometry is solved once against the whole road graph, not per quadtree
    // leaf -- junctions and miters are topology and topology does not stop at a
    // leaf boundary. The solve is pure CPU work in stratum_core with no GPU or UI
    // state, so it runs on a worker between parsing and indexing.
    //
    // The worker reads m_osm_parser's ParsedOSMData by pointer, so nothing may
    // clear or reassign the parser while this future is valid. begin_osm_import()
    // and the Clear Data button both refuse to run while the import is in flight.
    /**
     * @brief What one RoadNetworkBuilder run hands back to the main thread
     *
     * The elevation solver lives inside the builder, and the builder is local to
     * the worker lambda, so its statistics have to be lifted out before the
     * builder dies. They are what the OSM panel reads out.
     */
    struct RoadBuildResult {
        osm::road::RoadNetwork network;
        osm::road::RoadElevationSolver::Stats elevation;

        /// Steepest solved gradient over every edge, rise over run
        float max_grade = 0.0f;

        /// A height sampler was supplied, so the network follows the terrain
        bool elevated = false;

        /**
         * @brief The P4 junction solve ran, so RoadNetwork::junction_stats is real
         *
         * Stamped from the config the worker was launched with, never re-read from
         * the toggle when the future lands: the user may flip the checkbox while a
         * build is in flight, and the readout would then describe a solve that did
         * not happen.
         */
        bool solved_junctions = false;

        /**
         * @brief The P5 and P6 detail passes the worker was launched with
         *
         * Stamped from the config for the same reason solved_junctions is. Each
         * count in RoadNetwork::Stats is zero both when its pass was off and when
         * its pass found nothing, so the readout needs the flag to tell a
         * disabled pass from an empty one.
         */
        bool emitted_markings = false;
        bool emitted_crossings = false;
        bool emitted_structures = false;

        /**
         * @brief Fingerprint of the terrain this build was SOLVED against
         *
         * Stamped when the future is launched, from the same TerrainConfig the
         * height sampler captured -- never re-read when the future lands. The
         * user may press "Generate Chunked Terrain" while a build is in flight,
         * and reading the manager's config at the drain would then record a
         * surface the network was never solved against. The staleness check
         * would match, the drift warning would go quiet, and the roads would
         * stay wrong until some later terrain edit happened to disagree.
         */
        uint64_t terrain_fingerprint = 0;
    };

    std::future<RoadBuildResult> m_road_build_future;

    /**
     * @brief This road build is a rebuild, not a fresh import
     *
     * Set when terrain generation re-solves an already-imported network against
     * the new surface. It suppresses the camera re-frame in begin_mesh_rebuild():
     * the user pressed "Generate Chunked Terrain", not "Import", and having the
     * view jump back to the middle of the OSM data reads as a bug.
     */
    bool m_road_rebuild_only = false;

    /**
     * @brief A road rebuild was asked for while one was already in flight
     *
     * begin_road_network_rebuild() refuses to launch a second build, and the
     * request that was refused is the NEWER one -- the terrain the user just
     * generated. Dropping it leaves the network solved against the older surface
     * with nothing left to notice. finish_osm_import() honours this flag once
     * the in-flight build has landed, so a refused rebuild is deferred rather
     * than lost.
     */
    bool m_road_rebuild_owed = false;

    ImportStage m_import_stage = ImportStage::Idle;
    std::string m_import_message;
    float m_import_fraction = 0.0f;
    std::vector<osm::QuadTreeNode*> m_import_pending_nodes;
    size_t m_import_nodes_total = 0;

    // ── Native file picker ──────────────────────────────────────────────────
    // SDL_ShowOpenFileDialog is asynchronous and its callback may run on another
    // thread, so the result is parked here under a mutex and picked up by the UI
    // on the next frame rather than touching ImGui state from the callback.
    struct FilePickResult {
        std::mutex mutex;
        std::string path;        ///< Chosen file, empty if cancelled
        std::string error;       ///< Non-empty if the dialog itself failed
        bool has_result = false; ///< A callback landed and has not been consumed
        bool pending = false;    ///< A dialog is currently open
    };
    FilePickResult m_file_pick;
    char m_osm_filepath[512] = "";

    void open_osm_file_dialog();
    void poll_file_dialog();

    // ── Road network export ─────────────────────────────────────────────────
    //
    // Export re-solves the network rather than keeping a copy of it. The pieces
    // an import produces are MOVED into the quadtree, which merges them per leaf
    // and keeps only the render mesh -- the collision variant and the LOD chain
    // are dropped there, because nothing on screen draws them. Holding a second
    // full copy of a 63 MB extract's geometry against the chance of an export is
    // the wrong trade; re-solving costs a second of worker time and produces
    // exactly the data the exporter needs, including whichever of collision and
    // LODs the user asked for.
    //
    // The re-solve reads m_osm_parser's data by pointer on a worker, the same
    // way the import's road stage does, so the same rule applies: nothing may
    // clear or reassign the parser while the future is valid.

    /// A directory chosen through SDL_ShowOpenFolderDialog, parked for the UI thread
    FilePickResult m_dir_pick;

    /**
     * @brief One export running on a worker thread
     *
     * Held by pointer so `valid()` on the future is not the only liveness signal:
     * the destination and the flags the run was launched with are reported when
     * it lands, and re-reading the UI state at that point would describe a run
     * that did not happen.
     */
    struct RoadExportJob {
        std::string directory;
        osm::road::ExportConfig config;
        bool build_collision = false;
        bool build_lods = false;

        /// MUST be declared last: ~future joins the worker, and the worker's
        /// captures must outlive it. Same rule as OSMImportJob::future.
        std::future<osm::road::ExportStats> future;
    };

    std::unique_ptr<RoadExportJob> m_export_job;

    /// Format, chunk size and naming, edited by the memory panel
    osm::road::ExportConfig m_export_config;

    /// Fill RoadPiece::collision during the export re-solve
    bool m_export_build_collision = false;

    /// Fill RoadPiece::lods during the export re-solve
    bool m_export_build_lods = false;

    /// Destination directory, typed or chosen. Empty until one is picked.
    char m_export_dir[512] = "";

    /// Last export outcome, shown under the button
    std::string m_export_status;

    /// True while an export worker is running
    [[nodiscard]] bool export_in_flight() const { return m_export_job != nullptr; }

    /// Open the folder picker for the export destination
    void open_export_dir_dialog();

    /// Apply a folder the picker returned, on the main thread
    void poll_export_dir_dialog();

    /// Launch the export. Refuses while an import, a road build or another export runs.
    void begin_road_export();

    /// Drive the export across frames and report the result once
    void poll_road_export();

    void begin_osm_import(const std::string& filepath, const osm::ParserConfig& config);
    void poll_osm_import();
    /// @param road_pieces Prebuilt road geometry, handed to the quadtree once its
    ///                    leaves exist and before any node mesh build is queued.
    /// @param recenter_camera Frame the imported data. False for a road rebuild,
    ///                    which must leave the user's viewpoint alone.
    void begin_mesh_rebuild(std::vector<osm::road::RoadPiece>&& road_pieces = {},
                            bool recenter_camera = true);

    // ── Terrain-aware roads (P3) ────────────────────────────────────────────
    // Road elevation is solved GLOBALLY, over the whole graph, BEFORE any terrain
    // chunk is carved. That is only possible because the procedural height field
    // is a pure function of (TerrainConfig, x, z) and needs no generated chunk to
    // be sampled, so the solver reaches the terrain through a callback and the
    // carve happens later, per chunk, at chunk generation time.

    /**
     * @brief Solve road heights against the terrain surface, and carve it to match
     *
     * When off, roads come out flat at the corridor base height -- the P2
     * behaviour -- and any installed carve data is dropped so the terrain returns
     * to its procedural surface.
     */
    bool m_terrain_aware_roads = true;

    /**
     * @brief Run the P4 junction solve
     *
     * Maps straight onto RoadNetworkConfig::solve_junctions. When off, every edge
     * is extruded over its full length and ribbons overlap at every junction --
     * the P2 output, and the reference the junction work is diffed against.
     *
     * Exposed because a junction defect that only shows on one extract is
     * bisectable by flipping this and re-solving, which takes a second, rather
     * than by rebuilding with the solver compiled out.
     */
    bool m_solve_junctions = true;

    /**
     * @brief Emit painted lane markings
     *
     * RoadNetworkConfig::emit_markings. Off restores the P4 surfaces exactly:
     * no centre lines, edge lines, stop lines or turn arrows.
     */
    bool m_emit_markings = true;

    /**
     * @brief Emit pedestrian crossings
     *
     * RoadNetworkConfig::emit_crossings. Independent of m_emit_markings even
     * though a zebra is Markings geometry too: a crossing is located from OSM
     * topology and a lane line is derived from the profile, so they fail in
     * different ways and are bisected separately.
     */
    bool m_emit_crossings = true;

    /**
     * @brief Emit bridge decks and tunnel portals
     *
     * RoadNetworkConfig::emit_structures. Both need a terrain height under the
     * road, so both are skipped whatever this says when terrain-aware roads are
     * off -- the panel says so rather than leaving the toggle looking broken.
     */
    bool m_emit_structures = true;

    /**
     * @brief Run the tessellation reduction passes
     *
     * RoadNetworkConfig::reduce_tessellation. Off restores the pre-reduction
     * geometry exactly, which is what the golden tests diff against, so a
     * geometry defect can be attributed to the decimator by flipping this and
     * re-solving.
     */
    bool m_reduce_tessellation = true;

    /**
     * @brief Build a chunk-level LOD chain per quadtree leaf
     *
     * QuadTree::set_chunk_lod(). Read at ASSIGNMENT, not at draw time, because it
     * also decides how road geometry is routed into the tree -- triangle by
     * triangle when on, whole pieces when off -- so flipping it re-solves the
     * network rather than only changing what is drawn.
     */
    bool m_chunk_lod = true;

    /**
     * @brief Multiplier on every ChunkLod::switch_distances entry
     *
     * 1 is the chain's own suggestion. Larger holds full detail further out and
     * costs memory; smaller drops to a coarse level sooner.
     */
    float m_road_lod_distance_scale = 1.0f;

    /**
     * @brief Force every chunk to one LOD level, or -1 to select by distance
     *
     * An inspection control. A level forced beyond a chunk's chain is clamped to
     * that chunk's coarsest, so a leaf with a short chain still draws.
     */
    int m_road_lod_override = -1;

    /**
     * @brief What one completed traversal actually had resident, per LOD level
     *
     * Separate from QuadTree::RoadLodStats, which describes the chain that was
     * BUILT and does not change until the next import. This describes what the
     * camera is looking at right now, and it is the only number that says whether
     * selection is working at all: a build stat of four levels means nothing if
     * every visible leaf sits at level 0.
     *
     * Counted over VISIBLE leaves only, because the traversal is where it is
     * gathered and the traversal visits nothing else.
     */
    struct RoadLodFrameStats {
        /// Visible leaves whose resident level is this index
        std::vector<size_t> leaves_per_level;
        /// Visible leaves carrying a chain
        size_t leaves_with_chain = 0;
        /// Visible leaves carrying road geometry but no chain (chunk LOD off)
        size_t leaves_no_chain = 0;
        /// Triangles of the resident levels, summed over visible leaves
        size_t resident_triangles = 0;
        /// Vertices of the resident levels, summed over visible leaves
        size_t resident_vertices = 0;
        /// Level changes this traversal performed: one release plus one upload each
        size_t swaps = 0;

        void reset() {
            leaves_per_level.clear();
            leaves_with_chain = 0;
            leaves_no_chain = 0;
            resident_triangles = 0;
            resident_vertices = 0;
            swaps = 0;
        }
    };

    /**
     * @brief Accumulator for the traversal in flight
     *
     * Reset immediately before QuadTree::traverse_visible() and moved into
     * m_road_lod_frame when it returns. The panel reads the published copy, so it
     * never shows a half-gathered frame -- the OSM panel and render_3d() run in
     * an order ImGui decides, not one this class controls.
     */
    RoadLodFrameStats m_road_lod_frame_build;

    /// Residency of the last COMPLETED traversal; what draw_chunk_lod_stats() reads
    RoadLodFrameStats m_road_lod_frame;

    /**
     * @brief Add one visible leaf's residency to m_road_lod_frame_build
     *
     * Called after sync_node_road_lod() rather than inside it, so a leaf whose
     * upload was refused this frame is still counted at whatever it really has.
     */
    void record_road_lod_residency(const osm::QuadTreeNode& node);

    /**
     * @brief Choose, upload and release the one LOD level a leaf keeps resident
     *
     * Only the selected level is ever on the device. Uploading the whole chain
     * and picking per draw would cost more memory than no LOD at all.
     *
     * @param node     Leaf to update; does nothing when it carries no chain
     * @param renderer Upload target
     * @param distance Camera distance to the leaf, metres
     */
    void sync_node_road_lod(osm::QuadTreeNode& node, GPURenderer& renderer, float distance);

    /**
     * @brief Draw the chunk-LOD section of the OSM panel
     *
     * Reports the chain the last assignment built and, separately, what is
     * resident RIGHT NOW -- which is the number that says whether selection is
     * working, because the chain is fixed and the residency is not.
     */
    void draw_chunk_lod_stats();

    /**
     * @brief Height query handed to the road elevation solver, or null
     *
     * Null when terrain-aware roads are off, when the legacy single-terrain mode
     * is selected, or when no terrain chunk has been generated yet. A null
     * sampler is how RoadNetworkConfig asks for the flat P2 network.
     *
     * The returned callable owns its OWN TerrainGenerator. TerrainTileManager's
     * generator is private, and sampling one that another thread may reseed
     * through generate_chunk() is explicitly unsafe -- see the @note on
     * TerrainGenerator::sample_surface(). A private generator seeded from the
     * same config gives the identical surface with no such coupling, and it stays
     * valid for the whole life of the async build because the closure holds it by
     * shared_ptr.
     */
    [[nodiscard]] osm::road::HeightSampler make_terrain_height_sampler() const;

    /// True when the chunked terrain manager holds at least one generated chunk
    [[nodiscard]] bool has_generated_terrain() const;

    /**
     * @brief Identity of a terrain surface, for detecting a stale road solve
     *
     * Combines only the fields TerrainGenerator::sample_surface() reads. Two
     * configs with the same fingerprint produce the same surface, so a road
     * network solved against one is still correct against the other.
     *
     * Hashed field by field rather than over the raw bytes: TerrainConfig carries
     * padding, whose contents are indeterminate, and a byte hash would report
     * spurious changes.
     */
    [[nodiscard]] static uint64_t terrain_surface_fingerprint(const procgen::TerrainConfig& cfg);

    /**
     * @brief Fingerprint of the surface roads would be solved against right now
     *
     * Zero means "flat": terrain-aware roads off, legacy terrain mode, or no
     * chunk generated. Read at BUILD LAUNCH and compared against
     * m_road_terrain_fingerprint, which records the surface the live network was
     * actually solved against.
     */
    [[nodiscard]] uint64_t live_road_terrain_fingerprint() const;

    /**
     * @brief Re-solve the already-imported road network against the current terrain
     *
     * Runs the same staged, asynchronous path an import uses from the road stage
     * onward, so the progress bar and the "no blocking work on the UI thread"
     * rule both still hold. Does nothing when an import is already in flight or
     * when there is no parsed OSM data.
     */
    void begin_road_network_rebuild();

    /**
     * @brief Re-solve the roads if the terrain they were solved against changed
     *
     * Called after terrain generation. Two cases need it:
     *  - roads were imported before any terrain existed, so they were built flat
     *    and produced no carve data at all;
     *  - terrain was regenerated from a different config, so the solved heights
     *    belong to a surface that no longer exists.
     *
     * A rebuild is the right answer rather than a message telling the user to
     * re-import, because the parsed OSM data is already in memory and only the
     * road solve is stale. Re-importing would re-parse a file that can be
     * hundreds of megabytes to redo work that costs a second.
     */
    void maybe_rebuild_roads_for_terrain();

    /**
     * @brief Assemble the road pipeline config for a build about to be launched
     *
     * Main thread only: it reads the terrain settings and the terrain-aware
     * toggle. The result is captured by value into the worker lambda, so a later
     * edit of the terrain panel cannot change the surface a build in flight is
     * being solved against.
     */
    [[nodiscard]] osm::road::RoadNetworkConfig make_road_network_config() const;

    /**
     * @brief Run one road network build and lift the solver statistics out
     *
     * Static and free of Editor state: this is the body of the worker lambda, and
     * it must not touch anything the UI thread reads.
     */
    [[nodiscard]] static RoadBuildResult run_road_network_build(
        const osm::ParsedOSMData& data, const osm::road::RoadNetworkConfig& cfg);

    /// Enter the carve stage: index the corridors, or skip straight to Done
    void begin_road_carve();

    /// Drive the carve stage across frames
    void poll_road_carve();

    /// Leave the import state machine in the Done state
    void finish_osm_import();

    /**
     * @brief Hand the solved corridors to the terrain, or drop stale ones
     *
     * Main thread: TerrainTileManager::set_road_carve_data() regenerates every
     * existing chunk, and those chunks are walked by render_3d() every frame.
     * The spatial index over the corridors is built on a worker first, because
     * that part is pure CPU work over every ribbon in the import.
     */
    void install_road_carve_data();

    /**
     * @brief Solved corridors between the road build and the terrain install
     *
     * Filled when the road network lands, moved to a worker to be indexed, and
     * moved into the terrain tile manager once the meshes are done. Null outside
     * that window.
     */
    std::unique_ptr<procgen::CarveInput> m_pending_carve;

    /// Corridors waiting to be indexed on a worker, then installed
    std::future<std::unique_ptr<procgen::CarveInput>> m_carve_index_future;

    /// Embankment tunables applied to every corridor
    procgen::CarveConfig m_carve_config;

    /**
     * @brief The indexed carve input is ready and is applied on the NEXT frame
     *
     * Deliberately deferred by one frame. Installing it regenerates every terrain
     * chunk in one blocking call, and running that in the same frame that sets
     * the stage message means the message never reaches the screen and the hitch
     * looks like a freeze.
     */
    bool m_carve_apply_pending = false;

    /// Statistics of the last road build, for the OSM panel readout
    osm::road::RoadNetwork::Stats m_road_stats{};
    osm::road::RoadElevationSolver::Stats m_road_elevation_stats{};
    osm::road::JunctionBuilder::Stats m_road_junction_stats{};
    float m_road_max_grade = 0.0f;
    bool m_road_built_on_terrain = false;

    /// The last build ran the junction solve, so m_road_junction_stats means something
    bool m_road_solved_junctions = false;

    /// The last build ran each detail pass, so its count means "found none" rather than "off"
    bool m_road_emitted_markings = false;
    bool m_road_emitted_crossings = false;
    bool m_road_emitted_structures = false;

    /// Portal mouths the last build handed to the terrain carve
    size_t m_road_portal_mouths = 0;

    bool m_have_road_stats = false;

    /// Surface the current road network was solved against; 0 when it is flat
    uint64_t m_road_terrain_fingerprint = 0;
    void upload_node_to_gpu(osm::QuadTreeNode& node, GPURenderer& renderer);
    void release_node_from_gpu(osm::QuadTreeNode& node, GPURenderer& renderer);

    // ── Resident GPU geometry: who owns which mesh id ───────────────────────
    //
    // The renderer holds a byte budget and evicts the furthest geometry when it
    // is breached, but a GPUMesh is a vertex count and two ranges: it has no
    // transform and no bounds, so the renderer cannot say which mesh is furthest
    // and cannot tell anyone that a handle has stopped resolving. Both answers
    // live here, and this map is what supplies them.
    //
    // The consequence of getting it wrong is not a missing tile. A quadtree leaf
    // holding an evicted id and still drawing it is a use-after-free.

    /**
     * @brief Who owns one uploaded mesh id, and where in the world its geometry is
     */
    struct MeshOwner {
        /// What the owner is, which decides how the handle is cleared on eviction
        enum class Kind : uint8_t {
            /**
             * @brief Never evicted, whatever the pressure
             *
             * The legacy single terrain and its water plane. There is exactly one
             * of each, the user generated them deliberately, and neither streams
             * back in on its own -- an eviction would simply make them vanish
             * until the user pressed Generate again.
             */
            Pinned,

            /// A leaf of m_quadtree; `node` names it and stays valid while the tree does
            QuadTreeLeaf,

            /// A chunk of m_terrain_tile_manager; `coord` names it
            TerrainChunk
        };

        Kind kind = Kind::Pinned;

        /**
         * @brief Owning leaf, for Kind::QuadTreeLeaf
         *
         * A raw pointer into the quadtree, which is why every path that destroys
         * the tree -- begin_mesh_rebuild() and the Clear Data button -- must
         * release every leaf's meshes FIRST. release_node_from_gpu() unregisters
         * as it goes, so after that pass no entry can name a dead node.
         */
        osm::QuadTreeNode* node = nullptr;

        /// Owning chunk, for Kind::TerrainChunk. An index, so it cannot dangle.
        procgen::TerrainChunkCoord coord{};

        /// World-space point the eviction distance is measured to
        glm::vec3 anchor{0.0f};
    };

    std::unordered_map<uint32_t, MeshOwner> m_mesh_owners;

    /**
     * @brief Upload a mesh and record who owns the handle
     *
     * The only upload path in the editor. An untracked id would be evicted first
     * -- an owner the renderer cannot ask about is reported as infinitely far
     * away -- and nothing would be told to drop it.
     *
     * @return The mesh id, or 0 when the upload failed. A failed upload registers
     *         nothing.
     */
    uint32_t upload_tracked_mesh(GPURenderer& renderer, const Mesh& mesh, const MeshOwner& owner);

    /**
     * @brief Release a tracked mesh and zero the caller's handle
     *
     * @param mesh_id Handle, set to 0 on return. 0 in is a no-op.
     */
    void release_tracked_mesh(GPURenderer& renderer, uint32_t& mesh_id);

    /**
     * @brief Distance from the camera to a mesh, for the renderer's eviction sort
     *
     * Installed as GPURenderer::MeshDistanceFn. Negative means pinned. An id with
     * no owner comes back at the largest finite float, which makes it the first
     * thing evicted -- correct, because nothing is holding it any more.
     */
    [[nodiscard]] float mesh_distance_to_camera(uint32_t mesh_id) const;

    /**
     * @brief Drop a handle the renderer has just evicted
     *
     * Installed as GPURenderer::MeshEvictedFn, and called from inside
     * evict_to_budget(), so it must not call back into the renderer.
     *
     * A quadtree leaf that loses one of its meshes has `gpu_uploaded` cleared and
     * streams back in whole the next time it is visible; upload_node_to_gpu()
     * releases whichever of its handles survived before re-uploading, so the
     * partial state never leaks. A terrain chunk is handled the same way.
     */
    void on_mesh_evicted(uint32_t mesh_id);

    /// Centre of a leaf's 3D bounds, falling back to its 2D centre when the leaf
    /// holds no geometry to have grown bounds from.
    [[nodiscard]] static glm::vec3 node_anchor(const osm::QuadTreeNode& node);

    /// Rate limit for the "a node failed to upload" warning, in SDL ticks
    uint64_t m_next_upload_warn_ms = 0;

    // Procedural Generation (single terrain - legacy)
    procgen::TerrainGenerator m_terrain_generator;
    procgen::TerrainConfig m_terrain_config;
    procgen::TerrainMeshConfig m_terrain_mesh_config;
    procgen::Heightmap m_terrain_heightmap;
    Mesh m_terrain_mesh;
    Mesh m_water_mesh;
    uint32_t m_terrain_gpu_id = 0;
    uint32_t m_water_gpu_id = 0;
    bool m_render_terrain = true;
    bool m_render_water = true;
    
    // Chunked terrain system (new)
    procgen::TerrainTileManager m_terrain_tile_manager;
    procgen::TerrainTileConfig m_terrain_tile_config;
    bool m_use_chunked_terrain = true;  // Use new chunked system vs legacy single terrain
};

} // namespace stratum
