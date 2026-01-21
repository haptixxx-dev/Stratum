#pragma once

#include <imgui.h>
#include <functional>
#include "osm/parser.hpp"
#include "osm/mesh_builder.hpp"
#include "osm/tile_manager.hpp"
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
    void set_renderer(GPURenderer* renderer) { m_gpu_renderer = renderer; }
    void set_msaa_change_callback(std::function<void(int)> callback) { m_msaa_change_callback = callback; }
    void render_im3d_callback();

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

    void handle_window_resize();

    // OSM Parser and Tile Manager
    osm::OSMParser m_osm_parser;
    osm::TileManager m_tile_manager;
    std::string m_osm_import_path;
    bool m_use_tile_culling = true;
    bool m_use_distance_culling = true;  // Cull tiles outside view radius
    float m_view_radius = 2000.0f;       // Max distance from camera to render
    float m_tile_size = 500.0f;

    // Cached meshes for rendering (legacy - now managed by TileManager)
    std::vector<Mesh> m_building_meshes;
    std::vector<Mesh> m_road_meshes;
    std::vector<Mesh> m_area_meshes;

    // Pre-batched geometry for fast rendering (combined from all meshes)
    struct BatchedTriangle {
        glm::vec3 p0, p1, p2;
        glm::vec4 color;
    };
    std::vector<BatchedTriangle> m_batched_building_tris;
    std::vector<BatchedTriangle> m_batched_road_tris;
    std::vector<BatchedTriangle> m_batched_area_tris;

    // Dirty flag for batch rebuilding - only rebuild when camera moves significantly
    bool m_batches_dirty = true;
    glm::vec3 m_last_camera_pos{0.0f};
    glm::vec3 m_last_camera_dir{0.0f};
    float m_dirty_threshold_pos = 10.0f;   // Rebuild if camera moves this far
    float m_dirty_threshold_rot = 0.1f;    // Rebuild if camera rotates this much (dot product)

    void rebuild_osm_meshes();
    void rebuild_visible_batches();
    bool check_camera_dirty();  // Returns true if camera moved enough to warrant rebuild
    void upload_tile_to_gpu(osm::Tile& tile, GPURenderer& renderer);
    void release_tile_from_gpu(osm::Tile& tile, GPURenderer& renderer);

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
