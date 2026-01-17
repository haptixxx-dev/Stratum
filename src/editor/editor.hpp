#pragma once

#include <imgui.h>
#include <functional>
#include "osm/parser.hpp"
#include "osm/mesh_builder.hpp"
#include "osm/tile_manager.hpp"
#include "renderer/mesh.hpp"
#include "editor/camera.hpp"

struct SDL_Renderer;

namespace stratum {

class Editor {
public:
    Editor() = default;
    ~Editor() = default;

    void init();
    void shutdown();
    void update();
    void render();

    void set_quit_callback(std::function<void()> callback) { m_quit_callback = callback; }
    void set_window_handle(void* window) { m_window_handle = window; }
    void set_renderer(SDL_Renderer* renderer) { m_renderer = renderer; }
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
    void draw_toolbar();

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

    // Render toggles
    bool m_render_areas = true;
    bool m_render_roads = true;
    bool m_render_buildings = true;
    bool m_show_tile_grid = false;

    // Console log
    ImGuiTextBuffer m_console_buffer;
    bool m_console_scroll_to_bottom = true;

    // Core systems
    SDL_Renderer* m_renderer = nullptr;
    Camera m_camera;
    float m_last_time = 0.0f;

    // Callbacks
    std::function<void()> m_quit_callback;
    void* m_window_handle = nullptr;

    // Window dragging state
    bool m_dragging_window = false;
    ImVec2 m_drag_start_mouse;
    int m_drag_start_window_x = 0;
    int m_drag_start_window_y = 0;

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

    void rebuild_osm_meshes();
    void rebuild_visible_batches();
};

} // namespace stratum
