#include "editor/editor.hpp"
#include "editor/im3d_impl.hpp"
#include "renderer/gpu_renderer.hpp"
#include <im3d.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <spdlog/spdlog.h>
#include <SDL3/SDL.h>
#include <ImGuiFileDialog/ImGuiFileDialog.h>
#include <map>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace stratum {

static ImVec4 s_viewport_rect;

void Editor::render_im3d_callback() {
    if (m_gpu_renderer) {
        render_3d(*m_gpu_renderer);
    }
}

static void DrawIm3D_Callback(const ImDrawList* parent_list, const ImDrawCmd* cmd) {
    Editor* editor = static_cast<Editor*>(cmd->UserCallbackData);
    if (editor) {
        editor->render_im3d_callback();
    }
}

void Editor::init() {
    spdlog::info("Editor initialized");
    Im3D_Init();
}

void Editor::shutdown() {
    Im3D_Shutdown();
    spdlog::info("Editor shutdown");
}

void Editor::update() {
    // Update visible tile batches based on camera position
    // Note: Camera matrices are updated in draw_viewport, so we rebuild batches there
    // to ensure frustum is current
}

void Editor::render() {
    // Global keyboard shortcuts
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        if (m_quit_callback) m_quit_callback();
    }

    // Handle window resizing from edges
    handle_window_resize();

    setup_dockspace();

    
    // Update Camera (moved to draw_viewport to sync with focus, but could be here)
    // We do it in draw_viewport to update aspects correctly

    if (m_show_demo_window) {
        ImGui::ShowDemoWindow(&m_show_demo_window);
    }

    if (m_show_style_editor) {
        ImGui::Begin("Style Editor", &m_show_style_editor);
        ImGui::ShowStyleEditor();
        ImGui::End();
    }

    if (m_show_viewport) draw_viewport();
    if (m_show_scene_hierarchy) draw_scene_hierarchy();
    if (m_show_properties) draw_properties();
    if (m_show_console) draw_console();
    if (m_show_osm_panel) draw_osm_panel();
    if (m_show_procgen_panel) draw_procgen_panel();
    if (m_show_render_settings) draw_render_settings();
}

void Editor::setup_dockspace() {
    // Configure dockspace window flags
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse;
    window_flags |= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    ImGui::Begin("DockSpace", nullptr, window_flags);
    ImGui::PopStyleVar(3);

    // Submit the DockSpace
    ImGuiID dockspace_id = ImGui::GetID("StratumDockSpace");

    // First time setup - create default layout (only once)
    static bool dock_initialized = false;
    if (!dock_initialized && ImGui::DockBuilderGetNode(dockspace_id) == nullptr) {
        dock_initialized = true;
        ImGui::DockBuilderRemoveNode(dockspace_id);
        ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);

        // Split the dockspace
        ImGuiID dock_main = dockspace_id;
        ImGuiID dock_left = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Left, 0.2f, nullptr, &dock_main);
        ImGuiID dock_right = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Right, 0.25f, nullptr, &dock_main);
        ImGuiID dock_bottom = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Down, 0.25f, nullptr, &dock_main);

        // Dock windows to nodes
        ImGui::DockBuilderDockWindow("Scene Hierarchy", dock_left);
        ImGui::DockBuilderDockWindow("OSM", dock_left);
        ImGui::DockBuilderDockWindow("Viewport", dock_main);
        ImGui::DockBuilderDockWindow("Properties", dock_right);
        ImGui::DockBuilderDockWindow("Console", dock_bottom);

        ImGui::DockBuilderFinish(dockspace_id);
    }

    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);

    draw_menu_bar();

    ImGui::End();
}

void Editor::draw_menu_bar() {
    if (ImGui::BeginMenuBar()) {
        // Handle window dragging on menu bar
        if (m_window_handle) {
            ImVec2 mouse_pos = ImGui::GetMousePos();
            ImVec2 bar_min = ImGui::GetWindowPos();
            ImVec2 bar_max = ImVec2(bar_min.x + ImGui::GetWindowWidth(), bar_min.y + ImGui::GetFrameHeight());

            bool mouse_in_bar = mouse_pos.x >= bar_min.x && mouse_pos.x < bar_max.x &&
                                mouse_pos.y >= bar_min.y && mouse_pos.y < bar_max.y;

            if (mouse_in_bar && ImGui::IsMouseClicked(0) && !ImGui::IsAnyItemHovered()) {
                m_dragging_window = true;
                m_drag_start_mouse = mouse_pos;
                SDL_GetWindowPosition(static_cast<SDL_Window*>(m_window_handle),
                                      &m_drag_start_window_x, &m_drag_start_window_y);
            }

            if (m_dragging_window) {
                if (ImGui::IsMouseDown(0)) {
                    ImVec2 delta = ImVec2(mouse_pos.x - m_drag_start_mouse.x,
                                          mouse_pos.y - m_drag_start_mouse.y);
                    SDL_SetWindowPosition(static_cast<SDL_Window*>(m_window_handle),
                                          m_drag_start_window_x + (int)delta.x,
                                          m_drag_start_window_y + (int)delta.y);
                } else {
                    m_dragging_window = false;
                }
            }
        }

        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New Scene", "Ctrl+N")) {
                // TODO: New scene
            }
            if (ImGui::MenuItem("Open Scene...", "Ctrl+O")) {
                // TODO: Open scene
            }
            if (ImGui::MenuItem("Save Scene", "Ctrl+S")) {
                // TODO: Save scene
            }
            if (ImGui::MenuItem("Save Scene As...", "Ctrl+Shift+S")) {
                // TODO: Save scene as
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Import OSM...", "Ctrl+I")) {
                // TODO: Import OSM
            }
            if (ImGui::MenuItem("Export...", "Ctrl+E")) {
                // TODO: Export
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit", "Cmd+Q") || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
                if (m_quit_callback) m_quit_callback();
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Undo", "Ctrl+Z")) {}
            if (ImGui::MenuItem("Redo", "Ctrl+Y")) {}
            ImGui::Separator();
            if (ImGui::MenuItem("Cut", "Ctrl+X")) {}
            if (ImGui::MenuItem("Copy", "Ctrl+C")) {}
            if (ImGui::MenuItem("Paste", "Ctrl+V")) {}
            if (ImGui::MenuItem("Delete", "Delete")) {}
            ImGui::Separator();
            if (ImGui::MenuItem("Select All", "Ctrl+A")) {}
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Viewport", nullptr, &m_show_viewport);
            ImGui::MenuItem("Scene Hierarchy", nullptr, &m_show_scene_hierarchy);
            ImGui::MenuItem("Properties", nullptr, &m_show_properties);
            ImGui::MenuItem("Console", nullptr, &m_show_console);
            ImGui::MenuItem("OSM Panel", nullptr, &m_show_osm_panel);
            ImGui::MenuItem("Procgen Panel", nullptr, &m_show_procgen_panel);
            ImGui::MenuItem("Render Settings", nullptr, &m_show_render_settings);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &m_show_demo_window);
            ImGui::MenuItem("Style Editor", nullptr, &m_show_style_editor);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Tools")) {
            if (ImGui::MenuItem("OSM Downloader")) {}
            if (ImGui::MenuItem("Material Editor")) {}
            if (ImGui::MenuItem("LOD Generator")) {}
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem("Documentation")) {}
            if (ImGui::MenuItem("About Stratum")) {}
            ImGui::EndMenu();
        }

        // Right side: FPS, render settings toggle, and window controls
        float right_offset = 240.0f;
        ImGui::SetCursorPosX(ImGui::GetWindowWidth() - right_offset);
        ImGui::Text("%.0f FPS", ImGui::GetIO().Framerate);

        ImGui::SameLine();

        // Window control buttons
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.3f, 1.0f));

        // Render settings toggle button
        bool render_active = m_show_render_settings;
        if (render_active) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.4f, 0.6f, 1.0f));
        }
        if (ImGui::Button("Render")) {
            m_show_render_settings = !m_show_render_settings;
        }
        if (render_active) {
            ImGui::PopStyleColor();
        }

        ImGui::SameLine();
        if (ImGui::Button(" - ")) {
            if (m_window_handle) {
                SDL_MinimizeWindow(static_cast<SDL_Window*>(m_window_handle));
            }
        }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
        if (ImGui::Button(" X ")) {
            if (m_quit_callback) m_quit_callback();
        }
        ImGui::PopStyleColor(3);
        ImGui::EndMenuBar();
    }
}

void Editor::draw_viewport() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("Viewport");

    m_viewport_focused = ImGui::IsWindowFocused();
    m_viewport_hovered = ImGui::IsWindowHovered();

    ImVec2 viewport_size = ImGui::GetContentRegionAvail();
    ImVec2 pos = ImGui::GetCursorScreenPos();
    
    // Save rect for callback
    s_viewport_rect = ImVec4(pos.x, pos.y, viewport_size.x, viewport_size.y);

    // Update Camera
    float aspect = viewport_size.x / viewport_size.y;
    if (aspect <= 0.001f) aspect = 1.0f;
    
    // Calculate dt (this is a hack, usually passed in update)
    if (m_last_time == 0.0f) m_last_time = SDL_GetTicks() / 1000.0f;
    float current_time = SDL_GetTicks() / 1000.0f;
    float dt = current_time - m_last_time;
    m_last_time = current_time;

    m_camera.update(aspect);
    if (m_viewport_focused) {
        m_camera.handle_input(dt);
    }

    // Handle scroll wheel for camera speed adjustment while right-click is held
    if (m_viewport_hovered || m_viewport_focused) {
        ImGuiIO& io = ImGui::GetIO();
        bool right_mouse_held = io.MouseDown[1];  // Right mouse button
        if (right_mouse_held && io.MouseWheel != 0.0f) {
            m_camera.adjust_speed(io.MouseWheel);
        }
    }

    // Poll for completed async tile builds
    if (m_tile_manager.tile_count() > 0) {
        size_t completed = m_tile_manager.poll_async_builds();
        if (completed > 0) {
            m_batches_dirty = true;  // New meshes available, need to rebatch
        }
    }

    // Rebuild visible batches only when camera moves significantly or new meshes ready
    if (m_tile_manager.tile_count() > 0 && (m_batches_dirty || check_camera_dirty())) {
        rebuild_visible_batches();
        m_batches_dirty = false;
    }

    // Im3D Frame
    Im3D_NewFrame(dt, m_camera, viewport_size.x, viewport_size.y, m_viewport_focused);

    // Draw Content
    // Grid
    const int grid_lines = 20;
    const float grid_spacing = 2.0f;
    for (int i = -grid_lines; i <= grid_lines; ++i) {
        Im3d::DrawLine(Im3d::Vec3(i * grid_spacing, 0, -grid_lines * grid_spacing), Im3d::Vec3(i * grid_spacing, 0, grid_lines * grid_spacing), 1.0f, Im3d::Color(1.0f, 1.0f, 1.0f, 0.2f));
        Im3d::DrawLine(Im3d::Vec3(-grid_lines * grid_spacing, 0, i * grid_spacing), Im3d::Vec3(grid_lines * grid_spacing, 0, i * grid_spacing), 1.0f, Im3d::Color(1.0f, 1.0f, 1.0f, 0.2f));
    }
    
    // Draw Origin Axis
    Im3d::DrawLine(Im3d::Vec3(0,0,0), Im3d::Vec3(1,0,0), 2.0f, Im3d::Color(255, 0, 0));
    Im3d::DrawLine(Im3d::Vec3(0,0,0), Im3d::Vec3(0,1,0), 2.0f, Im3d::Color(0, 255, 0));
    Im3d::DrawLine(Im3d::Vec3(0,0,0), Im3d::Vec3(0,0,1), 2.0f, Im3d::Color(0, 0, 255));

    // Draw tile grid if enabled
    if (m_show_tile_grid && m_tile_manager.tile_count() > 0) {
        Frustum frustum = m_camera.get_frustum();
        for (const auto& coord : m_tile_manager.get_all_tiles()) {
            const auto* tile = m_tile_manager.get_tile(coord);
            if (!tile || !tile->has_valid_bounds()) continue;

            // Color based on state: green=visible+built, yellow=pending, red=culled
            bool in_frustum = frustum.intersects_aabb(tile->bounds_min, tile->bounds_max);
            Im3d::Color grid_color;
            if (!in_frustum) {
                grid_color = Im3d::Color(255, 0, 0, 100);      // Red = culled
            } else if (tile->meshes_pending) {
                grid_color = Im3d::Color(255, 200, 0, 200);    // Yellow = building
            } else if (tile->meshes_built) {
                grid_color = Im3d::Color(0, 255, 0, 200);      // Green = ready
            } else {
                grid_color = Im3d::Color(100, 100, 100, 150);  // Gray = not yet queued
            }

            // Draw tile bounding box edges
            glm::vec3 mn = tile->bounds_min;
            glm::vec3 mx = tile->bounds_max;

            // Bottom face
            Im3d::DrawLine(Im3d::Vec3(mn.x, mn.y, mn.z), Im3d::Vec3(mx.x, mn.y, mn.z), 1.5f, grid_color);
            Im3d::DrawLine(Im3d::Vec3(mx.x, mn.y, mn.z), Im3d::Vec3(mx.x, mn.y, mx.z), 1.5f, grid_color);
            Im3d::DrawLine(Im3d::Vec3(mx.x, mn.y, mx.z), Im3d::Vec3(mn.x, mn.y, mx.z), 1.5f, grid_color);
            Im3d::DrawLine(Im3d::Vec3(mn.x, mn.y, mx.z), Im3d::Vec3(mn.x, mn.y, mn.z), 1.5f, grid_color);

            // Top face
            Im3d::DrawLine(Im3d::Vec3(mn.x, mx.y, mn.z), Im3d::Vec3(mx.x, mx.y, mn.z), 1.5f, grid_color);
            Im3d::DrawLine(Im3d::Vec3(mx.x, mx.y, mn.z), Im3d::Vec3(mx.x, mx.y, mx.z), 1.5f, grid_color);
            Im3d::DrawLine(Im3d::Vec3(mx.x, mx.y, mx.z), Im3d::Vec3(mn.x, mx.y, mx.z), 1.5f, grid_color);
            Im3d::DrawLine(Im3d::Vec3(mn.x, mx.y, mx.z), Im3d::Vec3(mn.x, mx.y, mn.z), 1.5f, grid_color);

            // Vertical edges
            Im3d::DrawLine(Im3d::Vec3(mn.x, mn.y, mn.z), Im3d::Vec3(mn.x, mx.y, mn.z), 1.5f, grid_color);
            Im3d::DrawLine(Im3d::Vec3(mx.x, mn.y, mn.z), Im3d::Vec3(mx.x, mx.y, mn.z), 1.5f, grid_color);
            Im3d::DrawLine(Im3d::Vec3(mx.x, mn.y, mx.z), Im3d::Vec3(mx.x, mx.y, mx.z), 1.5f, grid_color);
            Im3d::DrawLine(Im3d::Vec3(mn.x, mn.y, mx.z), Im3d::Vec3(mn.x, mx.y, mx.z), 1.5f, grid_color);
        }
    }

    // Draw Area Meshes - pre-batched for performance (draw first, at bottom layer)
    if (m_render_areas && !m_batched_area_tris.empty()) {
        Im3d::BeginTriangles();
        for (const auto& tri : m_batched_area_tris) {
            Im3d::Color color(tri.color.r, tri.color.g, tri.color.b, tri.color.a);
            Im3d::Vertex(Im3d::Vec3(tri.p0.x, tri.p0.y, tri.p0.z), color);
            Im3d::Vertex(Im3d::Vec3(tri.p1.x, tri.p1.y, tri.p1.z), color);
            Im3d::Vertex(Im3d::Vec3(tri.p2.x, tri.p2.y, tri.p2.z), color);
        }
        Im3d::End();
    }

    // Draw Road Meshes - pre-batched for performance (draw second, over areas)
    if (m_render_roads && !m_batched_road_tris.empty()) {
        Im3d::BeginTriangles();
        for (const auto& tri : m_batched_road_tris) {
            Im3d::Color color(tri.color.r, tri.color.g, tri.color.b, tri.color.a);
            Im3d::Vertex(Im3d::Vec3(tri.p0.x, tri.p0.y, tri.p0.z), color);
            Im3d::Vertex(Im3d::Vec3(tri.p1.x, tri.p1.y, tri.p1.z), color);
            Im3d::Vertex(Im3d::Vec3(tri.p2.x, tri.p2.y, tri.p2.z), color);
        }
        Im3d::End();
    }

    // Draw Building Meshes - pre-batched for performance (draw last, on top)
    if (m_render_buildings && !m_batched_building_tris.empty()) {
        Im3d::BeginTriangles();
        for (const auto& tri : m_batched_building_tris) {
            Im3d::Color color(tri.color.r, tri.color.g, tri.color.b, tri.color.a);
            Im3d::Vertex(Im3d::Vec3(tri.p0.x, tri.p0.y, tri.p0.z), color);
            Im3d::Vertex(Im3d::Vec3(tri.p1.x, tri.p1.y, tri.p1.z), color);
            Im3d::Vertex(Im3d::Vec3(tri.p2.x, tri.p2.y, tri.p2.z), color);
        }
        Im3d::End();
    }

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Background (Gradient) - DISABLED to show 3D underlay
    /*
    ImU32 col_top = IM_COL32(40, 44, 52, 255);
    ImU32 col_bottom = IM_COL32(30, 33, 39, 255);
    draw_list->AddRectFilledMultiColor(
        pos,
        ImVec2(pos.x + viewport_size.x, pos.y + viewport_size.y),
        col_top, col_top, col_bottom, col_bottom
    );
    */

    // Render Im3D via callback

    draw_list->AddCallback(DrawIm3D_Callback, this);
    
    // Reset callback (optional but good practice)
    draw_list->AddCallback(ImDrawCallback_ResetRenderState, nullptr);

    // Overlay Text
    const char* text_overlay = "3D Viewport (Im3D + SDL3)";
    draw_list->AddText(ImVec2(pos.x + 10, pos.y + 10), IM_COL32(200, 200, 200, 255), text_overlay);

    // Toolbar overlay
    ImGui::SetCursorPos(ImVec2(10, 30));
    ImGui::BeginGroup();
    if (ImGui::Button("Translate")) {}
    ImGui::SameLine();
    if (ImGui::Button("Rotate")) {}
    ImGui::SameLine();
    if (ImGui::Button("Scale")) {}
    ImGui::SameLine();
    ImGui::Spacing();
    ImGui::SameLine();
    if (ImGui::Button("Local")) {}
    ImGui::SameLine();
    if (ImGui::Button("World")) {}
    ImGui::EndGroup();

    ImGui::End();
    ImGui::PopStyleVar();
}

void Editor::draw_scene_hierarchy() {
    ImGui::Begin("Scene Hierarchy");

    // Search bar
    static char search_buffer[256] = "";
    ImGui::SetNextItemWidth(-1);
    ImGui::InputTextWithHint("##search", "Search...", search_buffer, sizeof(search_buffer));

    ImGui::Separator();

    // Scene tree
    if (ImGui::TreeNodeEx("Scene", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::TreeNodeEx("Environment", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TreeNodeEx("Directional Light", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
            ImGui::TreeNodeEx("Sky", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
            ImGui::TreePop();
        }

        if (ImGui::TreeNodeEx("Terrain", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TreeNodeEx("Ground Plane", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
            ImGui::TreePop();
        }

        if (ImGui::TreeNodeEx("Buildings", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TreeNodeEx("Building_001", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
            ImGui::TreeNodeEx("Building_002", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
            ImGui::TreeNodeEx("Building_003", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
            ImGui::TreePop();
        }

        if (ImGui::TreeNodeEx("Roads", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TreeNodeEx("Main Street", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
            ImGui::TreeNodeEx("Side Road", ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
            ImGui::TreePop();
        }

        ImGui::TreePop();
    }

    ImGui::End();
}

void Editor::draw_properties() {
    ImGui::Begin("Properties");

    ImGui::Text("Transform");
    ImGui::Separator();

    static float position[3] = {0.0f, 0.0f, 0.0f};
    static float rotation[3] = {0.0f, 0.0f, 0.0f};
    static float scale[3] = {1.0f, 1.0f, 1.0f};

    ImGui::DragFloat3("Position", position, 0.1f);
    ImGui::DragFloat3("Rotation", rotation, 1.0f);
    ImGui::DragFloat3("Scale", scale, 0.01f);

    ImGui::Spacing();
    ImGui::Text("Material");
    ImGui::Separator();

    static int material_idx = 0;
    const char* materials[] = {"Default", "Concrete", "Asphalt", "Grass", "Metal"};
    ImGui::Combo("Material", &material_idx, materials, IM_ARRAYSIZE(materials));

    static float color[4] = {0.8f, 0.8f, 0.8f, 1.0f};
    ImGui::ColorEdit4("Color", color);

    static float roughness = 0.5f;
    static float metallic = 0.0f;
    ImGui::SliderFloat("Roughness", &roughness, 0.0f, 1.0f);
    ImGui::SliderFloat("Metallic", &metallic, 0.0f, 1.0f);

    ImGui::Spacing();
    ImGui::Text("Rendering");
    ImGui::Separator();

    static bool cast_shadows = true;
    static bool receive_shadows = true;
    static int lod_level = 0;

    ImGui::Checkbox("Cast Shadows", &cast_shadows);
    ImGui::Checkbox("Receive Shadows", &receive_shadows);
    ImGui::SliderInt("LOD Level", &lod_level, 0, 4);

    ImGui::End();
}

void Editor::draw_console() {
    ImGui::Begin("Console");

    // Options
    if (ImGui::BeginPopup("Options")) {
        ImGui::Checkbox("Auto-scroll", &m_console_scroll_to_bottom);
        ImGui::EndPopup();
    }

    // Buttons
    if (ImGui::Button("Clear")) {
        m_console_buffer.clear();
    }
    ImGui::SameLine();
    if (ImGui::Button("Options")) {
        ImGui::OpenPopup("Options");
    }

    ImGui::Separator();

    // Log content
    ImGui::BeginChild("ScrollingRegion", ImVec2(0, 0), ImGuiChildFlags_None,
                      ImGuiWindowFlags_HorizontalScrollbar);

    // Sample log messages
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.8f, 0.4f, 1.0f));
    ImGui::TextUnformatted("[INFO] Stratum v0.1.0 initialized");
    ImGui::PopStyleColor();

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.8f, 0.4f, 1.0f));
    ImGui::TextUnformatted("[INFO] SDL3 backend ready");
    ImGui::PopStyleColor();

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.8f, 0.4f, 1.0f));
    ImGui::TextUnformatted("[INFO] ImGui docking enabled");
    ImGui::PopStyleColor();

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.3f, 1.0f));
    ImGui::TextUnformatted("[WARN] GPU renderer not implemented yet");
    ImGui::PopStyleColor();

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
    ImGui::TextUnformatted("[DEBUG] Ready for OSM import");
    ImGui::PopStyleColor();

    if (m_console_scroll_to_bottom) {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();
    ImGui::End();
}

void Editor::draw_osm_panel() {
    ImGui::Begin("OSM");

    // Render toggles at top
    ImGui::Text("Show:");
    ImGui::SameLine();
    ImGui::Checkbox("##show_areas", &m_render_areas);
    ImGui::SameLine(); ImGui::Text("Areas");
    ImGui::SameLine();
    ImGui::Checkbox("##show_roads", &m_render_roads);
    ImGui::SameLine(); ImGui::Text("Roads");
    ImGui::SameLine();
    ImGui::Checkbox("##show_buildings", &m_render_buildings);
    ImGui::SameLine(); ImGui::Text("Bldgs");

    // Culling controls
    ImGui::Checkbox("Frustum Culling", &m_use_tile_culling);
    ImGui::SameLine();
    ImGui::Checkbox("Tile Grid", &m_show_tile_grid);
    ImGui::SetNextItemWidth(120);
    ImGui::SliderFloat("Tile Size", &m_tile_size, 50.0f, 2000.0f, "%.0f m");
    if (ImGui::IsItemDeactivatedAfterEdit() && m_osm_parser.has_data()) {
        rebuild_osm_meshes(); // Rebuild tiles with new size
    }
    if (m_tile_manager.tile_count() > 0) {
        // Count visible tiles using frustum
        size_t visible_count = 0;
        if (m_use_tile_culling) {
            Frustum frustum = m_camera.get_frustum();
            for (const auto& coord : m_tile_manager.get_all_tiles()) {
                const auto* tile = m_tile_manager.get_tile(coord);
                if (tile && tile->has_valid_bounds() &&
                    frustum.intersects_aabb(tile->bounds_min, tile->bounds_max)) {
                    visible_count++;
                }
            }
        } else {
            visible_count = m_tile_manager.tile_count();
        }
        ImGui::Text("(%zu/%zu tiles)", visible_count, m_tile_manager.tile_count());
    }

    ImGui::Separator();
    ImGui::Text("OpenStreetMap Import");
    ImGui::Separator();

    // Import options section
    ImGui::Text("Import Options");

    // Get mutable reference to config
    static osm::ParserConfig config;
    ImGui::Checkbox("Buildings", &config.import_buildings);
    ImGui::Checkbox("Roads", &config.import_roads);
    ImGui::Checkbox("Water", &config.import_water);
    ImGui::Checkbox("Landuse", &config.import_landuse);
    ImGui::Checkbox("Natural", &config.import_natural);

    ImGui::Spacing();
    ImGui::DragFloat("Default Height (m)", &config.default_building_height, 0.5f, 1.0f, 100.0f);
    ImGui::DragFloat("Meters/Level", &config.meters_per_level, 0.1f, 2.0f, 5.0f);

    ImGui::Separator();

    // File path input
    static char filepath[512] = "";
    ImGui::InputText("File Path", filepath, sizeof(filepath));
    ImGui::SameLine();
    if (ImGui::Button("Browse...")) {
        IGFD::FileDialogConfig fdConfig;
        fdConfig.path = ".";
        fdConfig.flags = ImGuiFileDialogFlags_Modal;
        ImGuiFileDialog::Instance()->OpenDialog(
            "ChooseOSMFile",
            "Choose OSM File",
            ".osm,.pbf,.osm.bz2,.osm.gz",
            fdConfig
        );
    }

    // File dialog display
    ImVec2 maxSize = ImVec2(800, 600);
    ImVec2 minSize = ImVec2(400, 300);
    if (ImGuiFileDialog::Instance()->Display("ChooseOSMFile", ImGuiWindowFlags_NoCollapse, minSize, maxSize)) {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            std::string selectedPath = ImGuiFileDialog::Instance()->GetFilePathName();
            strncpy(filepath, selectedPath.c_str(), sizeof(filepath) - 1);
            filepath[sizeof(filepath) - 1] = '\0';
        }
        ImGuiFileDialog::Instance()->Close();
    }

    // Import button with status feedback
    static std::string import_status;
    static bool import_error = false;

    if (ImGui::Button("Import OSM File", ImVec2(-1, 0))) {
        if (strlen(filepath) == 0) {
            import_status = "Please enter a file path first";
            import_error = true;
        } else {
            import_status = "Parsing...";
            import_error = false;

            m_osm_parser.set_config(config);
            if (m_osm_parser.parse(filepath)) {
                m_osm_parser.log_statistics();
                m_osm_parser.log_sample_data();

                // Build meshes from OSM data
                rebuild_osm_meshes();

                // Log to console
                const auto& data = m_osm_parser.get_data();
                char msg[256];
                snprintf(msg, sizeof(msg), "[OSM] Loaded: %zu roads, %zu buildings, %zu areas\n",
                        data.roads.size(), data.buildings.size(), data.areas.size());
                m_console_buffer.append(msg);
                m_console_scroll_to_bottom = true;

                import_status = "Import successful!";
                import_error = false;
            } else {
                char msg[512];
                snprintf(msg, sizeof(msg), "[OSM] Error: %s\n", m_osm_parser.get_error().c_str());
                m_console_buffer.append(msg);
                m_console_scroll_to_bottom = true;

                import_status = m_osm_parser.get_error();
                import_error = true;
            }
        }
    }

    // Show status message
    if (!import_status.empty()) {
        if (import_error) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", import_status.c_str());
        } else {
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "%s", import_status.c_str());
        }
    }

    ImGui::Separator();

    // Display loaded data statistics
    if (m_osm_parser.has_data()) {
        const auto& data = m_osm_parser.get_data();

        ImGui::Text("Loaded Data:");
        ImGui::BulletText("Nodes: %zu", data.stats.total_nodes);
        ImGui::BulletText("Ways: %zu", data.stats.total_ways);
        ImGui::BulletText("Relations: %zu", data.stats.total_relations);

        ImGui::Spacing();
        ImGui::Text("Processed:");
        ImGui::BulletText("Roads: %zu", data.roads.size());
        ImGui::BulletText("Buildings: %zu", data.buildings.size());
        ImGui::BulletText("Areas: %zu", data.areas.size());

        if (data.bounds.is_valid()) {
            ImGui::Spacing();
            ImGui::Text("Bounds:");
            ImGui::BulletText("Lat: [%.4f, %.4f]", data.bounds.min_lat, data.bounds.max_lat);
            ImGui::BulletText("Lon: [%.4f, %.4f]", data.bounds.min_lon, data.bounds.max_lon);
            ImGui::BulletText("Size: ~%.0fm x %.0fm",
                            data.bounds.width_meters(), data.bounds.height_meters());
        }

        ImGui::Spacing();
        ImGui::Text("Timing:");
        ImGui::BulletText("Parse: %.1f ms", data.stats.parse_time_ms);
        ImGui::BulletText("Process: %.1f ms", data.stats.process_time_ms);

        // Road type breakdown
        if (!data.roads.empty() && ImGui::TreeNode("Road Types")) {
            std::map<osm::RoadType, int> road_counts;
            for (const auto& road : data.roads) {
                road_counts[road.type]++;
            }
            for (const auto& [type, count] : road_counts) {
                ImGui::BulletText("%s: %d", osm::road_type_name(type), count);
            }
            ImGui::TreePop();
        }

        // Building type breakdown
        if (!data.buildings.empty() && ImGui::TreeNode("Building Types")) {
            std::map<osm::BuildingType, int> building_counts;
            for (const auto& bldg : data.buildings) {
                building_counts[bldg.type]++;
            }
            for (const auto& [type, count] : building_counts) {
                ImGui::BulletText("%s: %d", osm::building_type_name(type), count);
            }
            ImGui::TreePop();
        }

        // Area type breakdown
        if (!data.areas.empty() && ImGui::TreeNode("Area Types")) {
            std::map<osm::AreaType, int> area_counts;
            for (const auto& area : data.areas) {
                area_counts[area.type]++;
            }
            for (const auto& [type, count] : area_counts) {
                ImGui::BulletText("%s: %d", osm::area_type_name(type), count);
            }
            ImGui::TreePop();
        }

        ImGui::Separator();
        if (ImGui::Button("Clear Data", ImVec2(-1, 0))) {
            m_osm_parser.clear();
            m_tile_manager.clear();
            m_building_meshes.clear();
            m_road_meshes.clear();
            m_area_meshes.clear();
            m_batched_building_tris.clear();
            m_batched_road_tris.clear();
            m_batched_area_tris.clear();
        }
    }

    ImGui::End();
}

void Editor::draw_toolbar() {
    // Implemented as overlay in viewport
}

void Editor::draw_render_settings() {
    ImGui::Begin("Render Settings", &m_show_render_settings);

    if (m_gpu_renderer) {
        // Shader Mode selection
        ImGui::Text("Shader Mode");
        int shader_mode = static_cast<int>(m_gpu_renderer->get_shader_mode());
        const char* shader_options[] = { "Simple (Fast)", "PBR (Quality)" };
        if (ImGui::Combo("##ShaderMode", &shader_mode, shader_options, 2)) {
            m_gpu_renderer->set_shader_mode(static_cast<ShaderMode>(shader_mode));
        }
        
        // PBR settings (only visible in PBR mode)
        if (m_gpu_renderer->get_shader_mode() == ShaderMode::PBR) {
            ImGui::Separator();
            ImGui::Text("PBR Material");
            
            glm::vec3 pbr = m_gpu_renderer->get_pbr_params();
            bool changed = false;
            changed |= ImGui::SliderFloat("Metallic", &pbr.x, 0.0f, 1.0f);
            changed |= ImGui::SliderFloat("Roughness", &pbr.y, 0.04f, 1.0f);
            changed |= ImGui::SliderFloat("Ambient Occlusion", &pbr.z, 0.0f, 1.0f);
            if (changed) {
                m_gpu_renderer->set_pbr_params(pbr.x, pbr.y, pbr.z);
            }
            
            ImGui::Separator();
            ImGui::Text("Lighting");
            
            float exposure = m_gpu_renderer->get_exposure();
            if (ImGui::SliderFloat("Exposure", &exposure, 0.1f, 5.0f)) {
                m_gpu_renderer->set_exposure(exposure);
            }
            
            // Sun direction (simplified - azimuth angle)
            static float sun_angle = 45.0f;
            static float sun_height = 60.0f;
            bool sun_changed = false;
            sun_changed |= ImGui::SliderFloat("Sun Azimuth", &sun_angle, 0.0f, 360.0f, "%.0f°");
            sun_changed |= ImGui::SliderFloat("Sun Height", &sun_height, 5.0f, 90.0f, "%.0f°");
            if (sun_changed) {
                float az_rad = glm::radians(sun_angle);
                float h_rad = glm::radians(sun_height);
                glm::vec3 sun_dir = glm::normalize(glm::vec3(
                    cos(h_rad) * sin(az_rad),
                    sin(h_rad),
                    cos(h_rad) * cos(az_rad)
                ));
                m_gpu_renderer->set_scene_lighting(sun_dir, glm::vec3(1.0f, 0.98f, 0.95f), 1.0f, 0.1f);
            }
            
            // Fog
            ImGui::Separator();
            ImGui::Text("Fog");
            
            static int fog_mode = 0;  // 0 = off, 1 = linear, 2 = exp, 3 = exp squared
            static float fog_start = 50.0f;
            static float fog_end = 500.0f;
            static float fog_density = 0.005f;
            static glm::vec3 fog_color = glm::vec3(0.7f, 0.8f, 0.9f);
            bool fog_changed = false;
            
            const char* fog_modes[] = { "Off", "Linear", "Exponential", "Exponential Squared" };
            fog_changed |= ImGui::Combo("Fog Mode", &fog_mode, fog_modes, 4);
            
            if (fog_mode > 0) {
                fog_changed |= ImGui::ColorEdit3("Fog Color", &fog_color.x);
                
                if (fog_mode == 1) {
                    // Linear fog - use start/end distances
                    fog_changed |= ImGui::SliderFloat("Fog Start", &fog_start, 0.0f, 500.0f, "%.0f m");
                    fog_changed |= ImGui::SliderFloat("Fog End", &fog_end, 10.0f, 2000.0f, "%.0f m");
                    if (fog_start >= fog_end) fog_end = fog_start + 10.0f;
                } else {
                    // Exponential fog modes - use density
                    fog_changed |= ImGui::SliderFloat("Fog Density", &fog_density, 0.0001f, 0.05f, "%.4f", ImGuiSliderFlags_Logarithmic);
                }
            }
            
            if (fog_changed) {
                m_gpu_renderer->set_fog(fog_mode, fog_color, fog_start, fog_end, fog_density);
            }
        }

        ImGui::Separator();
        
        // Wireframe mode
        bool wireframe = (m_gpu_renderer->get_fill_mode() == FillMode::Wireframe);
        if (ImGui::Checkbox("Wireframe Mode", &wireframe)) {
            m_gpu_renderer->set_fill_mode(wireframe ? FillMode::Wireframe : FillMode::Solid);
        }

        // MSAA - disabled for now (requires app restart to change)
        // TODO: Implement offscreen MSAA rendering to allow runtime changes
        ImGui::Separator();
        ImGui::BeginDisabled();
        ImGui::Text("Anti-Aliasing");
        const char* msaa_options[] = { "Off", "2x MSAA", "4x MSAA", "8x MSAA" };
        int current_msaa = m_gpu_renderer->get_msaa_level();
        ImGui::Combo("MSAA", &current_msaa, msaa_options, 4);
        ImGui::EndDisabled();
        ImGui::TextDisabled("(Requires restart)");
    }

    // Culling settings
    ImGui::Separator();
    ImGui::Text("Culling");

    ImGui::Checkbox("Frustum Culling", &m_use_tile_culling);
    ImGui::Checkbox("Distance Culling", &m_use_distance_culling);

    if (m_use_distance_culling) {
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("View Radius", &m_view_radius, 500.0f, 20000.0f, "%.0f m");
    }

    // Stats
    if (m_tile_manager.tile_count() > 0) {
        ImGui::Separator();
        ImGui::Text("Statistics");

        size_t visible_count = 0;
        size_t total_tris = 0;
        Frustum frustum = m_camera.get_frustum();
        glm::vec3 cam_pos = m_camera.get_position();
        float radius_sq = m_view_radius * m_view_radius;

        for (const auto& coord : m_tile_manager.get_all_tiles()) {
            const auto* tile = m_tile_manager.get_tile(coord);
            if (!tile || !tile->has_valid_bounds()) continue;

            bool visible = true;
            if (m_use_tile_culling && !frustum.intersects_aabb(tile->bounds_min, tile->bounds_max)) {
                visible = false;
            }
            if (visible && m_use_distance_culling) {
                glm::vec3 tile_center = (tile->bounds_min + tile->bounds_max) * 0.5f;
                float dist_sq = glm::dot(tile_center - cam_pos, tile_center - cam_pos);
                if (dist_sq > radius_sq) visible = false;
            }
            if (visible) {
                visible_count++;
            }
        }

        total_tris = m_batched_area_tris.size() + m_batched_building_tris.size() + m_batched_road_tris.size();

        ImGui::BulletText("Visible Tiles: %zu / %zu", visible_count, m_tile_manager.tile_count());
        ImGui::BulletText("Batched Triangles: %zu", total_tris);
    }

    ImGui::End();
}

void Editor::handle_window_resize() {
    if (!m_window_handle) return;

    SDL_Window* window = static_cast<SDL_Window*>(m_window_handle);
    ImVec2 mouse = ImGui::GetMousePos();

    int win_x, win_y, win_w, win_h;
    SDL_GetWindowPosition(window, &win_x, &win_y);
    SDL_GetWindowSize(window, &win_w, &win_h);

    const float border = 8.0f;  // Resize border thickness
    const int min_size = 400;   // Minimum window size

    // Determine which edge/corner the mouse is over
    bool on_left = mouse.x < border;
    bool on_right = mouse.x > win_w - border;
    bool on_top = mouse.y < border;
    bool on_bottom = mouse.y > win_h - border;

    // Set cursor based on position
    ResizeEdge hover_edge = RESIZE_NONE;
    if (on_top && on_left) hover_edge = RESIZE_TOPLEFT;
    else if (on_top && on_right) hover_edge = RESIZE_TOPRIGHT;
    else if (on_bottom && on_left) hover_edge = RESIZE_BOTTOMLEFT;
    else if (on_bottom && on_right) hover_edge = RESIZE_BOTTOMRIGHT;
    else if (on_left) hover_edge = RESIZE_LEFT;
    else if (on_right) hover_edge = RESIZE_RIGHT;
    else if (on_top) hover_edge = RESIZE_TOP;
    else if (on_bottom) hover_edge = RESIZE_BOTTOM;

    // Set cursor
    if (hover_edge != RESIZE_NONE || m_resize_edge != RESIZE_NONE) {
        ResizeEdge active = (m_resize_edge != RESIZE_NONE) ? m_resize_edge : hover_edge;
        switch (active) {
            case RESIZE_LEFT:
            case RESIZE_RIGHT:
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                break;
            case RESIZE_TOP:
            case RESIZE_BOTTOM:
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
                break;
            case RESIZE_TOPLEFT:
            case RESIZE_BOTTOMRIGHT:
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNWSE);
                break;
            case RESIZE_TOPRIGHT:
            case RESIZE_BOTTOMLEFT:
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNESW);
                break;
            default:
                break;
        }
    }

    // Start resize on click
    if (hover_edge != RESIZE_NONE && ImGui::IsMouseClicked(0) && !ImGui::IsAnyItemHovered()) {
        m_resize_edge = hover_edge;
        m_drag_start_mouse = mouse;
        m_drag_start_window_x = win_x;
        m_drag_start_window_y = win_y;
        m_resize_start_w = win_w;
        m_resize_start_h = win_h;
    }

    // Handle active resize
    if (m_resize_edge != RESIZE_NONE) {
        if (ImGui::IsMouseDown(0)) {
            float dx = mouse.x - m_drag_start_mouse.x;
            float dy = mouse.y - m_drag_start_mouse.y;

            int new_x = m_drag_start_window_x;
            int new_y = m_drag_start_window_y;
            int new_w = m_resize_start_w;
            int new_h = m_resize_start_h;

            switch (m_resize_edge) {
                case RESIZE_RIGHT:
                    new_w = std::max(min_size, m_resize_start_w + (int)dx);
                    break;
                case RESIZE_BOTTOM:
                    new_h = std::max(min_size, m_resize_start_h + (int)dy);
                    break;
                case RESIZE_LEFT:
                    new_w = std::max(min_size, m_resize_start_w - (int)dx);
                    new_x = m_drag_start_window_x + m_resize_start_w - new_w;
                    break;
                case RESIZE_TOP:
                    new_h = std::max(min_size, m_resize_start_h - (int)dy);
                    new_y = m_drag_start_window_y + m_resize_start_h - new_h;
                    break;
                case RESIZE_BOTTOMRIGHT:
                    new_w = std::max(min_size, m_resize_start_w + (int)dx);
                    new_h = std::max(min_size, m_resize_start_h + (int)dy);
                    break;
                case RESIZE_BOTTOMLEFT:
                    new_w = std::max(min_size, m_resize_start_w - (int)dx);
                    new_h = std::max(min_size, m_resize_start_h + (int)dy);
                    new_x = m_drag_start_window_x + m_resize_start_w - new_w;
                    break;
                case RESIZE_TOPRIGHT:
                    new_w = std::max(min_size, m_resize_start_w + (int)dx);
                    new_h = std::max(min_size, m_resize_start_h - (int)dy);
                    new_y = m_drag_start_window_y + m_resize_start_h - new_h;
                    break;
                case RESIZE_TOPLEFT:
                    new_w = std::max(min_size, m_resize_start_w - (int)dx);
                    new_h = std::max(min_size, m_resize_start_h - (int)dy);
                    new_x = m_drag_start_window_x + m_resize_start_w - new_w;
                    new_y = m_drag_start_window_y + m_resize_start_h - new_h;
                    break;
                default:
                    break;
            }

            SDL_SetWindowPosition(window, new_x, new_y);
            SDL_SetWindowSize(window, new_w, new_h);
        } else {
            m_resize_edge = RESIZE_NONE;
        }
    }
}

void Editor::rebuild_osm_meshes() {
    m_building_meshes.clear();
    m_road_meshes.clear();
    m_area_meshes.clear();
    m_batched_building_tris.clear();
    m_batched_road_tris.clear();
    m_batched_area_tris.clear();

    const auto& osm_data = m_osm_parser.get_data();

    // Initialize tile manager
    spdlog::info("Initializing tile manager...");
    m_tile_manager.clear();
    m_tile_manager.init(osm_data.bounds, m_tile_size);
    m_tile_manager.assign_data(osm_data);
    
    // Meshes are now built lazily on-demand when tiles become visible

    spdlog::info("Tile manager: {} tiles, {} roads, {} buildings, {} areas",
                 m_tile_manager.tile_count(),
                 m_tile_manager.total_roads(),
                 m_tile_manager.total_buildings(),
                 m_tile_manager.total_areas());

    // Find center of first tile with actual geometry for camera positioning
    glm::vec3 data_center(0.0f);
    bool found_geometry = false;
    
    for (const auto& coord : m_tile_manager.get_all_tiles()) {
        const auto* tile = m_tile_manager.get_tile(coord);
        if (!tile) continue;
        
        // Use tile bounds center if we have valid bounds
        if (tile->has_valid_bounds()) {
            data_center = (tile->bounds_min + tile->bounds_max) * 0.5f;
            found_geometry = true;
            spdlog::info("Found tile at ({}, {}) with bounds: ({},{},{}) to ({},{},{})",
                        coord.x, coord.y,
                        tile->bounds_min.x, tile->bounds_min.y, tile->bounds_min.z,
                        tile->bounds_max.x, tile->bounds_max.y, tile->bounds_max.z);
            break;
        }
    }

    if (!found_geometry) {
        spdlog::warn("No geometry found in any tile!");
    }

    spdlog::info("Data center: ({}, {}, {})", data_center.x, data_center.y, data_center.z);

    // Center camera on data FIRST (before culling uses camera position)
    if (found_geometry) {
        float view_height = 300.0f;
        float view_distance = 300.0f;
        glm::vec3 cam_pos = data_center + glm::vec3(0.0f, view_height, view_distance);

        m_camera.set_position(cam_pos);
        m_camera.set_target(data_center);
        m_camera.m_far = 50000.0f;
        m_camera.m_base_speed = 200.0f;
        m_view_radius = 5000.0f;  // Start with larger view radius

        spdlog::info("Camera at ({}, {}, {}) looking at ({}, {}, {})",
                     cam_pos.x, cam_pos.y, cam_pos.z,
                     data_center.x, data_center.y, data_center.z);
    }

    // Enable culling for performance (camera is now positioned correctly)
    m_use_tile_culling = true;
    m_use_distance_culling = true;
    m_batches_dirty = true;  // Force rebuild on next frame
    m_last_camera_pos = m_camera.get_position();
    m_last_camera_dir = m_camera.get_forward();
    rebuild_visible_batches();

    spdlog::info("Initial batch: {} area tris, {} building tris, {} road tris",
                 m_batched_area_tris.size(), m_batched_building_tris.size(), m_batched_road_tris.size());
}

bool Editor::check_camera_dirty() {
    glm::vec3 cam_pos = m_camera.get_position();
    glm::vec3 cam_dir = m_camera.get_forward();

    // Check if position changed enough
    float dist_sq = glm::dot(cam_pos - m_last_camera_pos, cam_pos - m_last_camera_pos);
    if (dist_sq > m_dirty_threshold_pos * m_dirty_threshold_pos) {
        m_last_camera_pos = cam_pos;
        m_last_camera_dir = cam_dir;
        return true;
    }

    // Check if rotation changed enough (using dot product)
    float dot = glm::dot(cam_dir, m_last_camera_dir);
    if (dot < 1.0f - m_dirty_threshold_rot) {
        m_last_camera_pos = cam_pos;
        m_last_camera_dir = cam_dir;
        return true;
    }

    return false;
}

void Editor::rebuild_visible_batches() {
    m_batched_area_tris.clear();
    m_batched_building_tris.clear();
    m_batched_road_tris.clear();

    glm::vec3 cam_pos = m_camera.get_position();
    float radius_sq = m_view_radius * m_view_radius;
    Frustum frustum = m_camera.get_frustum();

    // Helper to check if mesh is within view distance (XZ plane)
    auto is_mesh_in_range = [&](const Mesh& mesh) -> bool {
        if (!m_use_distance_culling) return true;
        if (!mesh.bounds.is_valid()) return true;
        
        glm::vec3 mesh_center = mesh.bounds.center();
        glm::vec3 diff = mesh_center - cam_pos;
        float dist_sq = diff.x * diff.x + diff.z * diff.z;
        return dist_sq <= radius_sq;
    };

    // Helper to batch a mesh's triangles
    auto batch_mesh = [](const Mesh& mesh, std::vector<BatchedTriangle>& batch) {
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            const auto& v0 = mesh.vertices[mesh.indices[i]];
            const auto& v1 = mesh.vertices[mesh.indices[i + 1]];
            const auto& v2 = mesh.vertices[mesh.indices[i + 2]];
            batch.push_back({v0.position, v1.position, v2.position, v0.color});
        }
    };

    // Process only visible tiles (tile-level frustum culling first)
    for (const auto& coord : m_tile_manager.get_all_tiles()) {
        auto* tile = m_tile_manager.get_tile(coord);
        if (!tile || !tile->has_valid_bounds()) continue;

        // Tile-level frustum cull - skip entire tile if outside frustum
        if (m_use_tile_culling && !frustum.intersects_aabb(tile->bounds_min, tile->bounds_max)) {
            continue;
        }

        // Async lazy mesh building - queue if not built/pending
        if (!tile->meshes_built && !tile->meshes_pending) {
            m_tile_manager.queue_tile_build_async(coord);
            continue;  // Skip this tile until meshes are ready
        }

        // Skip tiles still being built
        if (!tile->meshes_built) {
            continue;
        }

        // Tile is visible - batch all its meshes (skip per-mesh frustum check since tile passed)
        for (const auto& mesh : tile->area_meshes) {
            if (is_mesh_in_range(mesh)) {
                batch_mesh(mesh, m_batched_area_tris);
            }
        }
        for (const auto& mesh : tile->building_meshes) {
            if (is_mesh_in_range(mesh)) {
                batch_mesh(mesh, m_batched_building_tris);
            }
        }
        for (const auto& mesh : tile->road_meshes) {
            if (is_mesh_in_range(mesh)) {
                batch_mesh(mesh, m_batched_road_tris);
            }
        }
    }
}

void Editor::upload_tile_to_gpu(osm::Tile& tile, GPURenderer& renderer) {
    if (tile.gpu_uploaded) return;

    // Upload area meshes
    tile.area_gpu_ids.clear();
    for (const auto& mesh : tile.area_meshes) {
        uint32_t id = renderer.upload_mesh(mesh);
        tile.area_gpu_ids.push_back(id);
    }

    // Upload road meshes
    tile.road_gpu_ids.clear();
    for (const auto& mesh : tile.road_meshes) {
        uint32_t id = renderer.upload_mesh(mesh);
        tile.road_gpu_ids.push_back(id);
    }

    // Upload building meshes
    tile.building_gpu_ids.clear();
    for (const auto& mesh : tile.building_meshes) {
        uint32_t id = renderer.upload_mesh(mesh);
        tile.building_gpu_ids.push_back(id);
    }

    tile.gpu_uploaded = true;
}

void Editor::release_tile_from_gpu(osm::Tile& tile, GPURenderer& renderer) {
    if (!tile.gpu_uploaded) return;

    for (uint32_t id : tile.area_gpu_ids) {
        renderer.release_mesh(id);
    }
    tile.area_gpu_ids.clear();

    for (uint32_t id : tile.road_gpu_ids) {
        renderer.release_mesh(id);
    }
    tile.road_gpu_ids.clear();

    for (uint32_t id : tile.building_gpu_ids) {
        renderer.release_mesh(id);
    }
    tile.building_gpu_ids.clear();

    tile.gpu_uploaded = false;
}

void Editor::render_3d(GPURenderer& renderer) {
    // upload meshes
    for (const auto& coord : m_tile_manager.get_all_tiles()) {
        osm::Tile* tile = const_cast<osm::Tile*>(m_tile_manager.get_tile(coord));
        if (tile && tile->meshes_built && !tile->gpu_uploaded) {
            tile->area_gpu_ids.clear();
            for (const auto& mesh : tile->area_meshes) {
                uint32_t id = renderer.upload_mesh(mesh);
                if (id) tile->area_gpu_ids.push_back(id);
            }
            tile->road_gpu_ids.clear();
            for (const auto& mesh : tile->road_meshes) {
                uint32_t id = renderer.upload_mesh(mesh);
                if (id) tile->road_gpu_ids.push_back(id);
            }
            tile->building_gpu_ids.clear();
            for (const auto& mesh : tile->building_meshes) {
                uint32_t id = renderer.upload_mesh(mesh);
                if (id) tile->building_gpu_ids.push_back(id);
            }
            tile->gpu_uploaded = true;
        }
    }

    // Set Viewport
    SDL_GPUViewport viewport;
    viewport.x = s_viewport_rect.x;
    viewport.y = s_viewport_rect.y;
    viewport.w = s_viewport_rect.z;
    viewport.h = s_viewport_rect.w;
    viewport.min_depth = 0.0f;
    viewport.max_depth = 1.0f;
    renderer.set_viewport(viewport);

    SDL_Rect scissor;
    scissor.x = (int)s_viewport_rect.x;
    scissor.y = (int)s_viewport_rect.y;
    scissor.w = (int)s_viewport_rect.z;
    scissor.h = (int)s_viewport_rect.w;
    if (renderer.get_render_pass()) {
        SDL_SetGPUScissor(renderer.get_render_pass(), &scissor);
    }

    renderer.bind_mesh_pipeline();
    glm::mat4 view = m_camera.get_view();
    glm::mat4 proj = m_camera.get_projection();
    renderer.set_view_projection(view, proj);
    
    // Set camera position for PBR lighting calculations
    glm::vec3 cam_pos = m_camera.get_position();
    renderer.set_camera_position(cam_pos);

    Frustum frustum = m_camera.get_frustum();
    glm::mat4 model(1.0f);
    float radius_sq = m_view_radius * m_view_radius;

    for (const auto& coord : m_tile_manager.get_all_tiles()) {
        const osm::Tile* tile = m_tile_manager.get_tile(coord);
        if (!tile || !tile->gpu_uploaded) continue;

        if (m_use_tile_culling && !frustum.intersects_aabb(tile->bounds_min, tile->bounds_max)) continue;

        // Distance culling at tile level
        if (m_use_distance_culling) {
            glm::vec3 tile_center = (tile->bounds_min + tile->bounds_max) * 0.5f;
            float dist_sq = glm::dot(tile_center - cam_pos, tile_center - cam_pos);
            if (dist_sq > radius_sq) continue;
        }

        if (m_render_areas) {
            for (uint32_t id : tile->area_gpu_ids) renderer.draw_mesh(id, model, glm::vec4(1.0f));
        }
        if (m_render_roads) {
            for (uint32_t id : tile->road_gpu_ids) renderer.draw_mesh(id, model, glm::vec4(1.0f));
        }
        if (m_render_buildings) {
            for (uint32_t id : tile->building_gpu_ids) renderer.draw_mesh(id, model, glm::vec4(1.0f));
        }
    }

    // Render procedural terrain
    if (m_use_chunked_terrain) {
        // Render chunked terrain
        if (m_render_terrain) {
            for (const auto& coord : m_terrain_tile_manager.get_all_chunks()) {
                auto* chunk = const_cast<procgen::TerrainChunk*>(m_terrain_tile_manager.get_chunk(coord));
                if (!chunk || !chunk->mesh_built) continue;
                
                // Upload to GPU if needed
                if (!chunk->gpu_uploaded && m_gpu_renderer) {
                    if (chunk->terrain_mesh.is_valid()) {
                        chunk->terrain_gpu_id = m_gpu_renderer->upload_mesh(chunk->terrain_mesh);
                    }
                    if (chunk->water_mesh.is_valid()) {
                        chunk->water_gpu_id = m_gpu_renderer->upload_mesh(chunk->water_mesh);
                    }
                    chunk->gpu_uploaded = true;
                }
                
                // Frustum culling
                if (m_use_tile_culling && !frustum.intersects_aabb(chunk->bounds_min, chunk->bounds_max)) {
                    continue;
                }
                
                // Distance culling
                if (m_use_distance_culling) {
                    glm::vec3 chunk_center = (chunk->bounds_min + chunk->bounds_max) * 0.5f;
                    float dist_sq = glm::dot(chunk_center - cam_pos, chunk_center - cam_pos);
                    if (dist_sq > radius_sq) continue;
                }
                
                // Draw terrain
                if (chunk->terrain_gpu_id != 0) {
                    renderer.draw_mesh(chunk->terrain_gpu_id, model, glm::vec4(1.0f));
                }
                
                // Draw water
                if (m_render_water && chunk->water_gpu_id != 0) {
                    renderer.draw_mesh(chunk->water_gpu_id, model, glm::vec4(1.0f));
                }
            }
        }
    } else {
        // Legacy single terrain rendering
        if (m_render_terrain && m_terrain_gpu_id != 0) {
            renderer.draw_mesh(m_terrain_gpu_id, model, glm::vec4(1.0f));
        }
        if (m_render_water && m_water_gpu_id != 0) {
            renderer.draw_mesh(m_water_gpu_id, model, glm::vec4(1.0f));
        }
    }
}

} // namespace stratum
