#pragma once

#include <imgui.h>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "osm/parser.hpp"
#include "osm/mesh_builder.hpp"
#include "osm/tile_manager.hpp"
#include "osm/quadtree.hpp"
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

    // Cached meshes for rendering (legacy - now managed by TileManager)
    std::vector<Mesh> m_building_meshes;
    std::vector<Mesh> m_road_meshes;
    std::vector<Mesh> m_area_meshes;

    // ── Async OSM import ────────────────────────────────────────────────────
    // Parsing runs on a worker thread; everything that touches the quadtree, the
    // camera or GPU resources stays on the main thread. Progress is surfaced as a
    // three-stage bar: parse -> spatial index -> mesh build.
    enum class ImportStage { Idle, Parsing, Indexing, BuildingMeshes, Done, Failed };

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

    void begin_osm_import(const std::string& filepath, const osm::ParserConfig& config);
    void poll_osm_import();
    void begin_mesh_rebuild();
    void upload_node_to_gpu(osm::QuadTreeNode& node, GPURenderer& renderer);
    void release_node_from_gpu(osm::QuadTreeNode& node, GPURenderer& renderer);

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
