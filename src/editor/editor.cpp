#include "editor/editor.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <spdlog/spdlog.h>
#include <SDL3/SDL.h>
#include <ImGuiFileDialog/ImGuiFileDialog.h>
#include <map>
#include <cstring>

namespace stratum {

void Editor::init() {
    spdlog::info("Editor initialized");
}

void Editor::shutdown() {
    spdlog::info("Editor shutdown");
}

void Editor::update() {
    // Update logic here
}

void Editor::render() {
    // Global keyboard shortcuts
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        if (m_quit_callback) m_quit_callback();
    }

    // Handle window resizing from edges
    handle_window_resize();

    setup_dockspace();

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

        // Right side: FPS and window controls
        float right_offset = 200.0f;
        ImGui::SetCursorPosX(ImGui::GetWindowWidth() - right_offset);
        ImGui::Text("%.0f FPS", ImGui::GetIO().Framerate);

        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetWindowWidth() - 90);

        // Window control buttons
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.3f, 1.0f));

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

    // Placeholder viewport content
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Draw a gradient background to simulate a 3D viewport
    ImU32 col_top = IM_COL32(40, 44, 52, 255);
    ImU32 col_bottom = IM_COL32(30, 33, 39, 255);
    draw_list->AddRectFilledMultiColor(
        pos,
        ImVec2(pos.x + viewport_size.x, pos.y + viewport_size.y),
        col_top, col_top, col_bottom, col_bottom
    );

    // Draw grid
    ImU32 grid_color = IM_COL32(60, 63, 70, 100);
    float grid_step = 50.0f;
    for (float x = fmodf(pos.x, grid_step); x < viewport_size.x; x += grid_step) {
        draw_list->AddLine(
            ImVec2(pos.x + x, pos.y),
            ImVec2(pos.x + x, pos.y + viewport_size.y),
            grid_color
        );
    }
    for (float y = fmodf(pos.y, grid_step); y < viewport_size.y; y += grid_step) {
        draw_list->AddLine(
            ImVec2(pos.x, pos.y + y),
            ImVec2(pos.x + viewport_size.x, pos.y + y),
            grid_color
        );
    }

    // Center text
    const char* text = "3D Viewport (GPU Rendering Coming Soon)";
    ImVec2 text_size = ImGui::CalcTextSize(text);
    draw_list->AddText(
        ImVec2(pos.x + (viewport_size.x - text_size.x) * 0.5f,
               pos.y + (viewport_size.y - text_size.y) * 0.5f),
        IM_COL32(150, 150, 150, 255),
        text
    );

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
        }
    }

    ImGui::End();
}

void Editor::draw_toolbar() {
    // Implemented as overlay in viewport
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

} // namespace stratum
