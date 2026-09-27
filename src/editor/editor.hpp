// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#pragma once

#include <imgui.h>
#include <spdlog/sinks/dist_sink.h>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "app/import_pipeline.hpp"
#include "osm/attribute_palette.hpp"
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
#include "editor/editor_model.hpp"
#include "editor/log_sink.hpp"
#include "editor/viewport_input.hpp"

namespace stratum {

// Forward declaration
class GPURenderer;
class MaterialLibrary;
class GPUTextureManager;

// Forward-declared for the Materials panel's helper signatures. material_library.hpp
// pulls in SDL through texture.hpp, and editor.hpp is included by nearly every
// editor translation unit; the panel's own .cpp includes the real headers.
struct MaterialDef;

class Editor {
public:
    Editor();

    /**
     * @brief Out-of-line so the unique_ptr members can hold incomplete types
     *
     * m_texture_manager and m_material_library are forward-declared above to keep
     * material_library.hpp and texture.hpp -- both of which pull in SDL -- out of
     * every translation unit that includes editor.hpp. unique_ptr's deleter needs
     * the complete type at the point the destructor is DEFINED, so it cannot be
     * `= default` in the header.
     */
    ~Editor();

    void init();
    void shutdown();
    void update();
    void render();
    void render_3d(GPURenderer& renderer);

    /**
     * @brief Publish this frame's camera to the renderer
     *
     * Separated out of render_3d() because the shadow cascades are fitted to
     * slices of the camera frustum and are rendered BEFORE the colour pass opens,
     * so they need the view and projection earlier than render_3d() runs.
     * render_3d() still calls it, so the two paths cannot disagree.
     */
    void publish_camera(GPURenderer& renderer);

    void set_quit_callback(std::function<void()> callback) { m_quit_callback = callback; }
    void set_window_handle(void* window) { m_window_handle = window; }

    /**
     * @brief Attach the GPU renderer and create renderer-dependent resources
     * @note Out-of-line because it also initializes the Im3d GPU backend, which
     *       cannot happen in init() - that runs before the renderer is attached.
     */
    void set_renderer(GPURenderer* renderer);

    /**
     * @brief Create the texture manager and material library and install them
     *
     * Called from set_renderer(), the first point at which a device exists.
     * Every failure path is non-fatal and leaves the renderer with no material
     * library, which draws exactly as it did before materials existed.
     */
    void init_materials(GPURenderer& renderer);

    /**
     * @brief End the Im3d frame and upload its geometry
     * @note Must be called with NO render pass active (it opens a copy pass).
     */
    void im3d_end_frame_and_upload(GPURenderer& renderer);
    void set_msaa_change_callback(std::function<void(int)> callback) { m_msaa_change_callback = callback; }

    bool is_viewport_focused() const { return m_viewport_input.focused; }
    bool is_viewport_hovered() const { return m_viewport_input.hovered; }

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

    /**
     * @brief The rule editor: write a rule file, run it, see the building
     *
     * Defined in src/editor/panels/rule_panel.cpp, with the four helpers below.
     * D1 to D4 built a language that until now could only be exercised from a
     * test. This panel is the first place a person writes a rule and sees the
     * geometry, which is the whole point of Track D.
     */
    void draw_rule_panel();

    /// Source box, load/save, and the generate controls.
    void draw_rule_source();

    /// Every diagnostic from the last parse or run, with its caret line.
    void draw_rule_diagnostics();

    /// Terminal count, shape count, depth, caps hit, and the `print` log.
    void draw_rule_stats();

    /**
     * @brief Parse the source, and run it when the parse allows
     *
     * Cheap enough to call on every edit for the parse; the RUN is what this
     * guards. See m_model.m_rule_auto_run.
     */
    void run_rule_source();

    /// Drop the preview mesh and its GPU handle. Safe to call with none.
    void clear_rule_preview();

    /// Read `path` into m_model.m_rule_source, update m_model.m_rule_path/m_rule_status and
    /// re-run it. Returns false (with m_rule_status set) if the file could not
    /// be opened. Called from poll_file_dialog() on FilePickTarget::RuleFileLoad.
    bool load_rule_source(const std::string& path);

    /// Write m_model.m_rule_source to `path` and update m_model.m_rule_path/m_rule_status.
    /// Returns false (with m_rule_status set, and logged on a write failure) if
    /// the file could not be opened or the write failed. Called from
    /// poll_file_dialog() on FilePickTarget::RuleFileSave.
    bool save_rule_source(const std::string& path);

    // ------------------------------------------------------------------------
    // Colour-by-attribute viewport mode
    //
    // Three thin pieces around one pure table. Which attribute is selected, the
    // ImGui that selects it, and an Im3d pass that paints it -- the enum-to-colour
    // mapping itself is osm/attribute_palette.hpp and lives in stratum_core so it
    // can be tested without a window. Nothing here decides a colour.
    // ------------------------------------------------------------------------

    /// Mode combo, legend, and the overlay's per-frame cost. Drawn inside the OSM panel.
    void draw_attribute_mode_selector();

    /**
     * @brief Paint this frame's classified features in their attribute colours
     *
     * An Im3d overlay rather than a recolour of the uploaded meshes, and that is a
     * deliberate limitation rather than a shortcut: a quadtree leaf holds ONE
     * merged building mesh and ONE merged area mesh (see
     * QuadTree::build_node_meshes_internal), so there is no per-feature draw call
     * to tint and no per-feature vertex range to rewrite. Per-feature colour would
     * mean re-uploading every leaf's vertex buffer on every mode change.
     *
     * The outlines come from the leaf's CPU-side Building, Road and Area records,
     * which is also the only place the classification still exists -- the merged
     * mesh does not carry it.
     *
     * Does nothing in AttributeMode::None. In AttributeMode::Tile the SOLID
     * geometry is already tinted by render_3d(), so this draws only the leaf
     * boundary that the tint is showing.
     */
    void draw_attribute_overlay();

    /**
     * @brief Colour tint render_3d() passes for one leaf's geometry
     *
     * White -- that is, no tint -- in every mode but Tile. Tile is the one mode
     * whose colour is constant across a whole leaf, which is exactly what a
     * per-draw tint can express, so it is the one mode that recolours the real
     * surfaces rather than drawing an overlay.
     *
     * @param node Leaf being drawn
     * @return Multiplied into the material's base colour by draw_mesh()
     */
    [[nodiscard]] glm::vec4 attribute_leaf_tint(const osm::QuadTreeNode& node) const;

    /**
     * @brief The same tint for a procedural terrain chunk
     *
     * Terrain chunks are a separate tile system from the quadtree with their own
     * integer coordinates, and streaming bugs land in both, so the mode covers
     * both. The two use the same cycle and will sometimes agree on a colour where
     * they overlap; the grids are different sizes, so the seams still read.
     *
     * @param coord Chunk coordinate
     * @return Multiplied into the material's base colour by draw_mesh()
     */
    [[nodiscard]] glm::vec4 attribute_chunk_tint(const procgen::TerrainChunkCoord& coord) const;

    /**
     * @brief The material library editor
     *
     * Defined in src/editor/panels/material_panel.cpp, together with the five
     * helpers below. Draws nothing but a notice when the material system failed to
     * come up, which is a survivable state -- see init_materials().
     */
    void draw_material_panel();

    /**
     * @brief Which of a material's three maps a map row edits
     *
     * Mirrors MaterialLibrary::TextureMap, which cannot be named here: it is
     * nested in a class this header only forward-declares, and including
     * material_library.hpp would drag SDL into every translation unit that
     * includes editor.hpp. The two are converted in one switch in the panel.
     */
    enum class MaterialMapSlot { Albedo = 0, Normal, Orm };

    /// Panel header: the two switches between an edit and a pixel, the fallback
    /// count, the texture stats, and save/load.
    void draw_material_panel_header();

    /// The keys resolve() could not answer exactly, with a button to promote one.
    void draw_material_fallback_list();

    /// Left pane: every MaterialId slot, with its installed variants under it.
    void draw_material_slot_tree();

    /// Right pane: every field of the selected MaterialDef, applied live.
    void draw_material_editor();

    /// Albedo, normal and ORM swatches for @p def.
    void draw_material_preview(const MaterialDef& def);

    /// One map's name, source, Load and Clear controls.
    /// @return true when @p def was edited in place and needs writing back.
    bool draw_material_map_row(const char* label, MaterialDef& def, MaterialMapSlot map,
                               MaterialKey key);
    
    // Procgen helpers
    void generate_terrain();
    void clear_terrain();
    void generate_chunked_terrain();
    void clear_chunked_terrain();
    void draw_chunked_terrain_ui();
    void draw_legacy_terrain_ui();

    /// This frame's viewport input (focus, hover, size, mouse delta, wheel, dt),
    /// filled from ImGui by draw_viewport() and consumed by Camera::handle_input()
    /// and Im3D_NewFrame() so neither has to query ImGui itself.
    ViewportInput m_viewport_input;

    /// Screen-space rect of the Viewport panel, in ImGui logical points.
    ///
    /// Written by draw_viewport() (zeroed first by render() so a closed panel
    /// reads as empty), and read by render_3d() to size the 3D pass's viewport
    /// and scissor. A member rather than a file-static because the write and the
    /// two reads now live in different translation units.
    ImVec4 m_viewport_rect{};

    /**
     * @brief Application/session state: import options, paths, export and
     *        render settings, and the rule editor's authoring fields
     *
     * See editor_model.hpp for exactly what lives here versus on Editor
     * itself. Every member that used to be `Editor::m_foo` for one of those
     * groups is now `m_model.m_foo`.
     */
    EditorModel m_model;

    bool m_show_demo_window = false;
    bool m_show_style_editor = false;

    /**
     * @brief The dockspace's default layout has been built for this run
     *
     * setup_dockspace() (src/editor/ui/layout.cpp) checks this alongside
     * DockBuilderGetNode() so the default split only runs once per process,
     * never re-splitting a layout the user has since rearranged. A member
     * rather than a function-local static because it is editor state like any
     * other, not a one-off first-call latch.
     */
    bool m_dock_initialized = false;

    // Panel visibility
    bool m_show_viewport = true;
    bool m_show_scene_hierarchy = true;
    bool m_show_properties = true;
    bool m_show_console = true;
    bool m_show_osm_panel = true;
    bool m_show_procgen_panel = true;
    bool m_show_render_settings = false;
    bool m_show_memory_panel = false;
    bool m_show_material_panel = false;
    bool m_show_rule_panel = false;

    /// Scene Hierarchy search field's typed text. Not yet wired to filter the
    /// tree; kept as editor state rather than a function-local static so the
    /// panel's own .cpp is not the only place that could read it later.
    char m_scene_search_buffer[256] = "";

    // Render toggles
    bool m_render_areas = true;
    bool m_render_roads = true;
    bool m_render_buildings = true;
    bool m_show_tile_grid = false;

    /// Which attribute the viewport paints by. None is normal shading.
    osm::AttributeMode m_attribute_mode = osm::AttributeMode::None;

    /// Show the swatch-and-name key for the selected mode.
    bool m_attribute_show_legend = true;

    // What last frame's overlay actually drew, and what it wanted to draw. They
    // differ when the budget below clamped it, and the panel says so -- an
    // overlay that silently stops halfway across a city reads as "these buildings
    // are unclassified", which is the one wrong conclusion this mode must never
    // invite.
    size_t m_attribute_features_drawn = 0;
    size_t m_attribute_features_seen = 0;

    /**
     * @brief Features the attribute overlay will outline in one frame
     *
     * Im3d buffers every vertex on the CPU and uploads the lot once per frame, so
     * an unbounded overlay over a city-sized import is a multi-megabyte upload and
     * a visible stall. Leaves are visited front to back, so the budget spends
     * itself on what is nearest, which is what a user is looking at.
     */
    static constexpr size_t kAttributeOverlayBudget = 20000;

    // Re-submit the batched OSM geometry through Im3d as debug triangles. Off by
    // default: render_3d() already draws the same quadtree geometry as GPU meshes,
    // so enabling this double-draws the whole scene.
    bool m_im3d_debug_geometry = false;

    // Console log. m_log_ring is the single source of truth -- everything that
    // used to go straight to an ImGuiTextBuffer now goes through spdlog, and
    // m_log_sink (registered on the default logger in init()) mirrors every
    // record into the ring for draw_console() to display.
    //
    // m_log_sink is registered through m_log_dist_sink rather than pushed
    // directly onto spdlog::default_logger()->sinks(): that vector has no
    // lock of its own, so add/remove there would race the logger's own
    // sink_it_() loop on whatever thread is mid-log (road build, export,
    // carve-index and quadtree work all log from std::async workers).
    // dist_sink_mt::add_sink()/remove_sink() take its own mutex against its
    // sink_it_(), so shutdown() can unregister m_log_sink while a worker is
    // still logging with no race.
    LogRing m_log_ring{2000};
    std::shared_ptr<RingSinkMt> m_log_sink;
    std::shared_ptr<spdlog::sinks::dist_sink_mt> m_log_dist_sink;
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

    /**
     * @brief Where m_model.m_render_settings is loaded from and saved to
     *
     * `<SDL pref path>/render_settings.json`, computed once in init() (SDL_GetPrefPath
     * allocates). Left empty if SDL_GetPrefPath fails, in which case
     * RenderSettings::load()/save() are both no-ops and the settings simply do
     * not persist across a restart.
     */
    std::filesystem::path m_render_settings_path;

    /**
     * @brief The texture set and the material set, owned here
     *
     * The Editor owns them because GPURenderer deliberately does not: it takes
     * both as non-owning pointers so that a tool, a test, or a headless exporter
     * can install a different material set without the renderer having an opinion
     * about where it came from.
     *
     * Both are created in set_renderer(), which is the first point at which an
     * SDL_GPUDevice exists, and torn down in shutdown(), which Application runs
     * BEFORE GPURenderer::shutdown() -- every SDL_GPUTexture and SDL_GPUSampler
     * inside them is a child of that device and cannot outlive it.
     */
    std::unique_ptr<GPUTextureManager> m_texture_manager;
    std::unique_ptr<MaterialLibrary> m_material_library;

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
    //
    // The parse itself is owned by m_import_pipeline (see below); an
    // osm::OSMParser member used to live here directly, but three workers read
    // its ParsedOSMData by pointer, and the pipeline is the one object that can
    // enforce "nothing reassigns or clears the parser while a future is valid".
    // Read it through m_import_pipeline.parser().
    osm::QuadTree m_quadtree;

    /**
     * @brief The "Import OSM File" button's own local status line
     *
     * Distinct from m_import_pipeline.state().message, which mirrors the
     * pipeline's stage as the worker runs: this pair is set synchronously by
     * the button itself (e.g. an empty file path) before any job exists, and
     * cleared when a job is launched.
     */
    std::string m_import_status;
    bool m_import_error = false;

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

    /**
     * @brief What the in-flight file dialog is FOR
     *
     * There is ONE file-dialog path in this editor -- one FilePickResult, one
     * callback, one poll_file_dialog() -- and several things that want to use it.
     * The target says where poll_file_dialog() delivers the path it collected,
     * rather than a second copy of the mutex-and-park machinery existing per
     * feature.
     *
     * Written on the main thread in open_file_dialog() BEFORE the dialog is
     * marked pending, and read on the main thread in poll_file_dialog() after the
     * result lands. It is not under the mutex because it never races: the pending
     * flag means at most one dialog exists at a time, and the SDL callback does
     * not touch it.
     */
    enum class FilePickTarget {
        OsmFile = 0,     ///< The OSM/PBF extract to import
        MaterialAlbedo,  ///< Albedo map for m_model.m_material_pick_key
        MaterialNormal,  ///< Normal map for m_model.m_material_pick_key
        MaterialOrm,     ///< ORM map for m_model.m_material_pick_key
        MaterialSetLoad, ///< A material set JSON to replace the whole library
        MaterialSetSave, ///< Destination to write the whole library to
        RuleFileLoad,    ///< A .rule file to load into the rule editor
        RuleFileSave,    ///< Destination to write the rule editor's source to
    };
    FilePickTarget m_file_pick_target = FilePickTarget::OsmFile;

    /**
     * @brief Open the ONE native file dialog, for @p target
     *
     * Asynchronous: returns immediately and does nothing if a dialog is already
     * open. FilePickTarget::MaterialSetSave uses SDL_ShowSaveFileDialog and every
     * other target uses SDL_ShowOpenFileDialog; both land in the same callback and
     * are collected by the same poll_file_dialog().
     */
    void open_file_dialog(FilePickTarget target);

    /// Shorthand for open_file_dialog(FilePickTarget::OsmFile), kept because the
    /// OSM panel reads better for it.
    void open_osm_file_dialog();

    /// Apply a landed dialog result on the main thread, routed by m_file_pick_target.
    void poll_file_dialog();

    // ── Material library ────────────────────────────────────────────────────

    /// Last path a material set was saved to or loaded from. Shown in the panel
    /// and used as the save dialog's starting location.
    std::string m_material_set_path;

    /// Result of the last material-set save or load, shown under the buttons.
    std::string m_material_set_status;

    // ── Road network export ─────────────────────────────────────────────────
    //
    // Export re-solves the network rather than keeping a copy of it -- see
    // ImportPipeline::begin_export() (src/app/import_pipeline.cpp) for why.
    // The launch, the worker and the result now live entirely in the pipeline;
    // the editor keeps only the folder picker and the button's status line.

    /// A directory chosen through SDL_ShowOpenFolderDialog, parked for the UI thread
    FilePickResult m_dir_pick;

    /// Last export outcome, shown under the button. Filled from Launch::reason
    /// (a refusal) or PipelineListener::on_export_done() (a completed run).
    std::string m_export_status;

    /// Open the folder picker for the export destination
    void open_export_dir_dialog();

    /// Apply a folder the picker returned, on the main thread
    void poll_export_dir_dialog();

    /// Tear down all imported OSM data. Forwards to m_import_pipeline.cancel()
    /// (a no-op unless a cancellable stage happens to be running) and
    /// m_import_pipeline.clear(), which releases the quadtree's GPU meshes
    /// through PipelineListener::on_before_quadtree_reset() before dropping the
    /// CPU-side data. Called from the import panel's "Clear Data" button, which
    /// disables itself while m_import_pipeline.state().busy() so clear() is
    /// never refused for still being busy.
    void clear_imported_data();

    /// Point the camera at the quadtree's current geometry. Called from
    /// PipelineListener::on_quadtree_ready(), never directly.
    /// @param recenter_camera False for a road rebuild, which must leave the
    ///                    user's viewpoint alone.
    void frame_camera_on_data(bool recenter_camera);

    // ── Terrain-aware roads (P3) ────────────────────────────────────────────
    // Road elevation is solved GLOBALLY, over the whole graph, BEFORE any terrain
    // chunk is carved. That is only possible because the procedural height field
    // is a pure function of (TerrainConfig, x, z) and needs no generated chunk to
    // be sampled, so the solver reaches the terrain through a callback and the
    // carve happens later, per chunk, at chunk generation time.
    //
    // The toggles themselves (m_model.m_terrain_aware_roads,
    // m_model.m_solve_junctions, m_model.m_emit_markings,
    // m_model.m_emit_crossings, m_model.m_emit_structures,
    // m_model.m_reduce_tessellation) and the chunk LOD fields
    // (m_model.m_chunk_lod, m_model.m_road_lod_distance_scale,
    // m_model.m_road_lod_override) live on EditorModel; see editor_model.hpp.

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

    /// True when the chunked terrain manager holds at least one generated chunk.
    /// Everything else this used to gate (the height sampler, the fingerprint,
    /// the road rebuild) now lives in app::ImportPipeline; this one survives on
    /// Editor because import_panel.cpp reads it directly, with no pipeline
    /// state involved.
    [[nodiscard]] bool has_generated_terrain() const;

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
         * the tree -- ImportPipeline::rebuild_index() (via
         * PipelineListener::on_before_quadtree_reset()) and the Clear Data
         * button -- must release every leaf's meshes FIRST.
         * release_node_from_gpu() unregisters as it goes, so after that pass no
         * entry can name a dead node.
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

    // ------------------------------------------------------------------------
    // Import pipeline (Phase 3 of the UI restructure)
    //
    // The OSM import / road build / spatial index / terrain carve / export
    // state machine now lives in app::ImportPipeline (stratum_core), not here.
    // See docs/plans/import-pipeline-design.md. m_pipeline_listener and
    // m_import_pipeline are declared LAST of the systems above -- after
    // m_quadtree and after m_terrain_tile_manager -- because the pipeline
    // holds references to both and members construct (and, in reverse,
    // destruct) in declaration order.
    // ------------------------------------------------------------------------

    /**
     * @brief Adapts app::ImportPipeline::Listener callbacks onto Editor
     *
     * Declared before m_import_pipeline and constructed first (both are
     * default member initializers, run in declaration order), so it is ready
     * before the pipeline could ever call it and still alive for as long as
     * the pipeline is. See docs/plans/import-pipeline-design.md section 2 for
     * what each handler does and where the behaviour used to live.
     */
    class PipelineListener final : public app::ImportPipeline::Listener {
    public:
        explicit PipelineListener(Editor& editor) : editor_(editor) {}

        /// SceneGpuSync: release every leaf's GPU meshes before the tree
        /// they belong to is torn down. Was ip.cpp:756-760 and :711-715.
        void on_before_quadtree_reset() override;

        /// CameraFraming, then re-enable the streaming culling modes. Was
        /// ip.cpp:783-788.
        void on_quadtree_ready(bool recenter_camera) override;

        /// Filled from the live camera. Was ip.cpp:795-800.
        std::optional<app::InitialView> initial_view() override;

        /// Was every ip.cpp site that logs: m_console_scroll_to_bottom = true.
        void on_log(const app::PipelineLogLine& line) override;

        /// Was poll_road_export(), ip.cpp:106-131.
        void on_export_done(const app::ExportResult& result) override;

    private:
        Editor& editor_;
    };

    PipelineListener m_pipeline_listener{*this};

    /**
     * @brief The OSM import / road build / spatial index / terrain carve /
     *        export state machine
     *
     * Owns the committed parse and the pending carve; references m_quadtree
     * and m_terrain_tile_manager, which is why this is declared after both of
     * them (see docs/plans/import-pipeline-design.md section 1.1 and 4).
     * set_listener() is called once, in init().
     */
    app::ImportPipeline m_import_pipeline{m_quadtree, m_terrain_tile_manager};

    /// Every road-build toggle, packed from m_model and m_use_chunked_terrain
    /// into the one value m_import_pipeline reads. Called once per frame
    /// before tick(), and again before any request_road_rebuild() or
    /// begin_import(), so a deferred read always sees the live model.
    [[nodiscard]] app::RoadOptions road_options() const;

    /// set_road_options(road_options()), then forward to
    /// m_import_pipeline.request_road_rebuild(). Keeps "the model changed
    /// this frame, the request sees it" true at every call site that used to
    /// call begin_road_network_rebuild() or maybe_rebuild_roads_for_terrain().
    app::RebuildOutcome request_road_rebuild(app::RebuildPolicy policy);

    // ------------------------------------------------------------------------
    // Rule editor (D10)
    //
    // Deliberately PLAIN types only. A RuleFile or a GenerationResult member
    // would drag procgen/rules/ast.hpp and interpreter.hpp into every
    // translation unit that includes editor.hpp, for a panel that parses on an
    // edit and runs on a button. rule_panel.cpp holds both as locals and keeps
    // only what it draws, which is the same discipline material_panel.cpp
    // follows with MaterialLibrary.
    // ------------------------------------------------------------------------

    // The source text, load/save path, auto-run and dirty flags, the seed
    // source/size/value and the imported-building cap are authoring state and
    // live on EditorModel (m_model.m_rule_source, m_model.m_rule_path,
    // m_model.m_rule_auto_run, m_model.m_rule_dirty, m_model.m_rule_seed_source
    // of type EditorModel::RuleSeedSource, m_model.m_rule_building_limit,
    // m_model.m_rule_seed_width, m_model.m_rule_seed_depth,
    // m_model.m_rule_seed); see editor_model.hpp.

    /// One line under the load/save row: what the last file action did
    std::string m_rule_status;

    /// Buildings the last run actually generated, and how many it skipped
    uint32_t m_rule_buildings_built = 0;
    uint32_t m_rule_buildings_skipped = 0;

    /**
     * @brief Diagnostics from the last parse or run, already rendered
     *
     * Rendered rather than stored as Diagnostic for the include reason above.
     * Each entry is the full multi-line render_diagnostic() output, caret and
     * all, so a parse error and a runtime error look identical to the reader --
     * which is the promise interpreter.hpp makes.
     */
    std::vector<std::string> m_rule_diagnostics;

    /// True when any diagnostic was Severity::Error. Colours the list header.
    bool m_rule_has_error = false;

    /// Lines the `print` operation wrote, in evaluation order
    std::vector<std::string> m_rule_log;

    /// GenerationStats from the last run, unpacked so no header is needed
    uint32_t m_rule_terminals = 0;
    uint32_t m_rule_shapes = 0;
    uint32_t m_rule_rules_invoked = 0;
    uint32_t m_rule_operations = 0;
    uint32_t m_rule_max_depth = 0;
    bool m_rule_depth_capped = false;
    bool m_rule_shape_capped = false;

    /// True once a run has happened, so the output pane can say "not run yet"
    bool m_rule_has_run = false;

    /**
     * @brief The generated preview
     *
     * Uploaded as MeshOwner::Kind::Pinned, for the reason that kind exists:
     * there is exactly one, the user pressed a button to make it, and it does
     * not stream back in on its own. An eviction would make it vanish with no
     * way to tell that from a rule that generated nothing.
     */
    Mesh m_rule_preview_mesh;
    uint32_t m_rule_preview_gpu_id = 0;
    bool m_render_rule_preview = true;
};

} // namespace stratum
