#include "editor/editor.hpp"
#include "editor/im3d_impl.hpp"
#include "renderer/gpu_renderer.hpp"
#include "renderer/material_library.hpp"
#include "renderer/procedural_texture.hpp"
#include "renderer/texture.hpp"
#include <im3d.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <spdlog/spdlog.h>
#include <SDL3/SDL.h>
#include <map>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <utility>

namespace stratum {

static ImVec4 s_viewport_rect;

// Defined here, not in the header: the unique_ptr members hold types that are
// only forward-declared there. See the note on ~Editor().
Editor::Editor() = default;
Editor::~Editor() = default;

void Editor::init() {
    spdlog::info("Editor initialized");
    Im3D_Init();
}

void Editor::set_renderer(GPURenderer* renderer) {
    m_gpu_renderer = renderer;
    if (renderer) {
        // Non-fatal: on failure Im3d simply renders nothing.
        Im3D_InitGPU(*renderer);

        // The renderer owns the budget and the eviction mechanism; the editor owns
        // the answers to "how far away is this?" and "your handle is gone". Without
        // both installed the renderer refuses to evict at all, because evicting
        // without a distance discards the road under the camera as readily as one
        // on the horizon.
        renderer->set_mesh_distance_fn([this](uint32_t id) { return mesh_distance_to_camera(id); });
        renderer->set_mesh_evicted_fn([this](uint32_t id) { on_mesh_evicted(id); });

        init_materials(*renderer);
    }
}

void Editor::init_materials(GPURenderer& renderer) {
    SDL_GPUDevice* device = renderer.get_device();
    if (!device) {
        spdlog::warn("No GPU device; roads will draw untextured");
        return;
    }

    // Every failure below is NON-FATAL and leaves the renderer with no material
    // library installed, which it treats as "draw exactly as before materials
    // existed". A broken material set must degrade to the old untextured look,
    // never to a black screen or a missing scene.
    auto textures = std::make_unique<GPUTextureManager>();
    if (!textures->init(device)) {
        spdlog::error("Texture manager init failed; roads will draw untextured");
        return;
    }

    auto materials = std::make_unique<MaterialLibrary>();
    if (!materials->init(textures.get())) {
        spdlog::error("Material library init failed; roads will draw untextured");
        textures->shutdown();
        return;
    }

    // The frozen slot table first, so every MaterialId is at least the right
    // colour and roughness even if texture generation below fails.
    materials->load_defaults();

    // Then the generated tiling detail. install_procedural_textures() also
    // installs the variant table, because a cobblestone variant without its stone
    // texture is only a slightly different shade of grey. Failure here is
    // survivable: the flat defaults above remain.
    if (!materials->install_procedural_textures()) {
        spdlog::warn("Procedural texture generation failed; materials stay flat");
    }

    m_texture_manager = std::move(textures);
    m_material_library = std::move(materials);

    // Order matters only in that both must be installed before the first draw.
    renderer.set_texture_manager(m_texture_manager.get());
    renderer.set_material_library(m_material_library.get());

    // Materials are a PBR-ONLY path by construction: the simple shader declares no
    // material uniform block and no samplers, so GPURenderer::bind_material()
    // returns immediately in ShaderMode::Simple. The renderer starts in Simple, so
    // without this the whole material system would be installed, populated, and
    // completely invisible until someone found the Render Settings combo -- which
    // is exactly the "threaded through and never read" failure this phase exists to
    // end. Switching here, and only once the library actually came up, is what
    // makes the world materially distinct at startup.
    //
    // set_shader_mode() refuses if the PBR pipelines failed to build and says so;
    // that leaves the editor in Simple mode drawing untextured, which is the
    // correct degraded state rather than a black screen.
    if (!renderer.set_shader_mode(ShaderMode::PBR)) {
        spdlog::warn("Materials installed but PBR is unavailable; roads draw untextured");
    }

    // The panel is worth opening by default the first time there is something in
    // it. It is a normal dockable panel afterwards and remembers nothing, so this
    // costs a keystroke to undo and saves a hunt through the View menu.
    m_show_material_panel = true;

    const auto stats = m_texture_manager->stats();
    spdlog::info("Materials ready: {} materials, {} textures, {} KB",
                 m_material_library->size(), stats.textures, stats.bytes / 1024);
}

void Editor::im3d_end_frame_and_upload(GPURenderer& renderer) {
    Im3D_EndFrameAndUpload(renderer);
}

void Editor::shutdown() {
    Im3D_Shutdown();

    // Before GPURenderer::shutdown() destroys the device these textures and
    // samplers belong to. Application calls us first, which is what makes this
    // the right place; the renderer also tears them down defensively if some
    // other caller skips this path.
    if (m_gpu_renderer) {
        m_gpu_renderer->set_material_library(nullptr);
        m_gpu_renderer->set_texture_manager(nullptr);
    }
    if (m_material_library) {
        m_material_library->shutdown();
        m_material_library.reset();
    }
    if (m_texture_manager) {
        m_texture_manager->shutdown();
        m_texture_manager.reset();
    }

    spdlog::info("Editor shutdown");
}

void Editor::update() {
    // Update visible tile batches based on camera position
    // Note: Camera matrices are updated in draw_viewport, so we rebuild batches there
    // to ensure frustum is current
}

void Editor::render() {
    // Invalidate the viewport rect every frame. draw_viewport() republishes it
    // below, but only when the panel is actually drawn -- so if the Viewport is
    // closed, this leaves it zeroed and render_3d() bails out instead of
    // rendering the whole 3D scene into a stale rect.
    s_viewport_rect = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);

    // Global keyboard shortcuts
    if (ImGui::IsKeyPressed(ImGuiKey_F11)) {
        toggle_fullscreen();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        // Escape leaves fullscreen first; quitting outright is a nasty surprise
        // when the window is covering the whole screen.
        if (m_fullscreen) {
            toggle_fullscreen();
        } else if (m_quit_callback) {
            m_quit_callback();
        }
    }

    // Handle window resizing from edges
    handle_window_resize();

    // Advance any in-flight OSM import. Must run before the panels draw so the
    // progress bar reflects this frame's state.
    poll_osm_import();

    // Apply a native file-dialog result on the main thread; the SDL callback that
    // produced it may have run on another one.
    poll_file_dialog();
    poll_export_dir_dialog();

    // Advance an in-flight export. Same rule as the import: before the panels draw.
    poll_road_export();

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
    if (m_show_memory_panel) draw_memory_panel();
    if (m_show_material_panel) draw_material_panel();
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
    // The 3D scene is now drawn in an earlier render pass. Without NoBackground the
    // dockspace host window would paint over it.
    window_flags |= ImGuiWindowFlags_NoBackground;

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

    // NOTE: deliberately NOT ImGuiDockNodeFlags_PassthruCentralNode. That flag only
    // punches a transparent hole when the central node is EMPTY (imgui.cpp:19068);
    // the "Viewport" window is docked into the central node, so instead it would fill
    // the whole dockspace with ImGuiCol_WindowBg and paint over the 3D render pass.
    // Transparency here comes from ImGuiWindowFlags_NoBackground on the Viewport window.
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
            if (ImGui::MenuItem("Fullscreen", "F11", m_fullscreen)) {
                toggle_fullscreen();
            }
            ImGui::Separator();
            ImGui::MenuItem("Viewport", nullptr, &m_show_viewport);
            ImGui::MenuItem("Scene Hierarchy", nullptr, &m_show_scene_hierarchy);
            ImGui::MenuItem("Properties", nullptr, &m_show_properties);
            ImGui::MenuItem("Console", nullptr, &m_show_console);
            ImGui::MenuItem("OSM Panel", nullptr, &m_show_osm_panel);
            ImGui::MenuItem("Procgen Panel", nullptr, &m_show_procgen_panel);
            ImGui::MenuItem("Render Settings", nullptr, &m_show_render_settings);
            ImGui::MenuItem("GPU Memory", nullptr, &m_show_memory_panel);
            ImGui::MenuItem("Materials", nullptr, &m_show_material_panel);
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
        // ASCII only: the default ImGui font (ProggyClean, via AddFontDefault in
        // application.cpp) carries no glyphs beyond Basic Latin, so a Unicode
        // fullscreen symbol renders as the missing-glyph box. Same 3-character
        // width as the other title-bar buttons so nothing shifts when it toggles.
        if (ImGui::Button(m_fullscreen ? "] [" : "[ ]")) {
            toggle_fullscreen();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(m_fullscreen ? "Exit fullscreen (F11)" : "Fullscreen (F11)");
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
    // NoBackground: the 3D pass has already drawn the scene into the swapchain
    ImGui::Begin("Viewport", nullptr, ImGuiWindowFlags_NoBackground);

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

    // Poll for completed async quadtree node builds
    if (m_quadtree.leaf_count() > 0) {
        m_quadtree.poll_async_builds();
    }
    // Streaming (queueing builds for newly-visible nodes) now rides on the
    // traversal render_3d already performs, so there is no second traversal here.

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
    // Use the packed 0xRRGGBBAA constants: Im3d::Color(int,int,int) resolves to the
    // FLOAT constructor (components in 0..1), so Color(255,0,0) overflows to near-black.
    Im3d::DrawLine(Im3d::Vec3(0,0,0), Im3d::Vec3(1,0,0), 2.0f, Im3d::Color_Red);
    Im3d::DrawLine(Im3d::Vec3(0,0,0), Im3d::Vec3(0,1,0), 2.0f, Im3d::Color_Green);
    Im3d::DrawLine(Im3d::Vec3(0,0,0), Im3d::Vec3(0,0,1), 2.0f, Im3d::Color_Blue);

    // Draw quadtree node boxes if enabled
    if (m_show_tile_grid && m_quadtree.leaf_count() > 0) {
        Frustum frustum = m_camera.get_frustum();
        for (auto* leaf : m_quadtree.get_all_leaves()) {
            if (!leaf || !leaf->has_valid_bounds()) continue;

            // Color based on state and depth
            bool in_frustum = frustum.intersects_aabb(leaf->bounds_min, leaf->bounds_max);
            Im3d::Color grid_color;
            // NOTE: Im3d::Color takes FLOAT components in 0..1, not 0..255 bytes.
            if (!in_frustum) {
                grid_color = Im3d::Color(1.0f, 0.0f, 0.0f, 0.39f);   // Red = culled
            } else if (leaf->meshes_pending) {
                grid_color = Im3d::Color(1.0f, 0.78f, 0.0f, 0.78f);  // Yellow = building
            } else if (leaf->meshes_built) {
                // Color by depth: deeper = brighter green
                float g = (100.0f + std::min(155, leaf->depth * 20)) / 255.0f;
                grid_color = Im3d::Color(0.0f, g, 0.0f, 0.78f);
            } else {
                grid_color = Im3d::Color(0.39f, 0.39f, 0.39f, 0.59f); // Gray = not yet queued
            }

            glm::vec3 mn = leaf->bounds_min;
            glm::vec3 mx = leaf->bounds_max;

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

    // 3D content (including Im3D) is rendered by Application::render() in its own
    // depth-attached render pass, before ImGui's depth-less pass draws over it.

    // Overlay Text
    const char* text_overlay = "3D Viewport";
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

void Editor::draw_chunk_lod_stats() {
    ImGui::Spacing();
    ImGui::Text("Chunk LOD:");

    if (!m_chunk_lod) {
        // Not "no levels were built": the chain is not built at all, and the
        // leaves hold whole pieces routed by anchor. Say which of the two it is.
        ImGui::BulletText("Off: every leaf keeps one full-detail mesh");
        if (m_road_lod_frame.leaves_no_chain > 0) {
            ImGui::BulletText("Visible leaves with roads: %zu",
                              m_road_lod_frame.leaves_no_chain);
        }
        return;
    }

    const osm::QuadTree::RoadLodStats& built = m_quadtree.road_lod_stats();

    if (built.chunks == 0) {
        ImGui::BulletText("On: no road geometry has been assigned yet");
        return;
    }

    ImGui::BulletText("Chunks: %zu built from %zu (%.1f ms)", built.chunks_with_lod,
                      built.chunks, built.build_ms);

    // The merge is the half of the win that costs nothing: level 0 is the same
    // triangles with the per-piece seams welded shut, so it is already smaller
    // than what was handed in before a single level is simplified.
    const size_t level0_tris =
        built.triangles_per_level.empty() ? 0 : built.triangles_per_level.front();
    ImGui::BulletText("Merged: %zu -> %zu triangles, %zu -> %zu vertices",
                      built.triangles_in, level0_tris, built.vertices_in,
                      built.vertices_per_level.empty() ? 0 : built.vertices_per_level.front());

    for (size_t l = 0; l < built.triangles_per_level.size(); ++l) {
        const size_t tris = built.triangles_per_level[l];
        const double pct = level0_tris ? 100.0 * static_cast<double>(tris)
                                             / static_cast<double>(level0_tris)
                                       : 100.0;
        ImGui::BulletText("  L%zu: %zu tri (%.1f%% of L0) in %zu chunks", l, tris, pct,
                          l < built.chunks_per_level.size() ? built.chunks_per_level[l] : 0);
    }

    // The seam band is the reduction the crack-free guarantee is paid for with.
    // A large one means a triangle reached a long way outside the leaf that owns
    // it, and the coarsest level is what absorbs the cost.
    ImGui::BulletText("Seam band: %.2f m widest, %zu straddling triangles",
                      built.max_seam_band, built.straddling_triangles);

    // Residency. This is the number that says selection is working: the chain
    // above is fixed at import, what follows changes as the camera moves.
    ImGui::Spacing();
    ImGui::Text("Resident (last frame, visible leaves):");

    if (m_road_lod_frame.leaves_with_chain == 0) {
        ImGui::BulletText("No visible leaf carries a chain");
    } else {
        size_t resident_leaves = 0;
        for (size_t l = 0; l < m_road_lod_frame.leaves_per_level.size(); ++l) {
            resident_leaves += m_road_lod_frame.leaves_per_level[l];
            if (m_road_lod_frame.leaves_per_level[l] == 0) continue;
            ImGui::BulletText("  L%zu: %zu leaves", l, m_road_lod_frame.leaves_per_level[l]);
        }
        // The two counts differ while a leaf waits for an upload the budget
        // refused, which is a streaming state and not an error.
        ImGui::BulletText("%zu of %zu visible leaves resident, %zu tri, %zu vtx",
                          resident_leaves, m_road_lod_frame.leaves_with_chain,
                          m_road_lod_frame.resident_triangles,
                          m_road_lod_frame.resident_vertices);
        // A swap is a release plus an upload. A steady non-zero number with the
        // camera still means the hysteresis band is being crossed every frame.
        ImGui::BulletText("Level swaps last frame: %zu", m_road_lod_frame.swaps);
    }

    // Inspection controls. Neither re-solves: the chain is already built and both
    // only change which level of it is asked for on the next frame.
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("LOD Distance", &m_road_lod_distance_scale, 0.25f, 4.0f, "%.2fx");
    ImGui::SetItemTooltip(
        "Multiplier on every switch distance the chain suggests.\n"
        "Larger holds full detail further out and costs resident memory.");

    bool forced = (m_road_lod_override >= 0);
    if (ImGui::Checkbox("Force Level", &forced)) {
        m_road_lod_override = forced ? 0 : -1;
    }
    ImGui::SetItemTooltip(
        "Pin every chunk to one level regardless of distance, for inspection.\n"
        "A chunk with a shorter chain is clamped to its own coarsest level.");

    if (forced) {
        ImGui::SameLine();
        const int max_level =
            built.triangles_per_level.empty()
                ? 0
                : static_cast<int>(built.triangles_per_level.size()) - 1;
        ImGui::SetNextItemWidth(120.0f);
        ImGui::SliderInt("##road_lod_level", &m_road_lod_override, 0, max_level, "Level %d");
    }
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
    ImGui::Checkbox("Node Grid", &m_show_tile_grid);
    ImGui::Checkbox("Contribution Culling", &m_use_contribution_culling);
    if (m_use_contribution_culling) {
        ImGui::SetNextItemWidth(120);
        ImGui::SliderFloat("Threshold (px)", &m_contribution_threshold, 1.0f, 20.0f, "%.1f");
    }
    if (m_quadtree.leaf_count() > 0) {
        ImGui::Text("Leaves: %zu, Max Depth: %d", m_quadtree.leaf_count(), m_quadtree.max_depth());
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

    ImGui::Spacing();
    if (ImGui::Checkbox("Terrain-Aware Roads", &m_terrain_aware_roads)) {
        // The toggle changes which surface the roads belong on, so the current
        // solve is stale either way round. Re-solve now rather than waiting for
        // the next terrain generate.
        maybe_rebuild_roads_for_terrain();
    }
    ImGui::SetItemTooltip(
        "Solve road heights against the terrain and carve the terrain to match.\n"
        "Off: roads stay flat and the terrain keeps its procedural surface.\n"
        "Requires chunked terrain to have been generated.");

    if (m_terrain_aware_roads && !has_generated_terrain()) {
        ImGui::TextDisabled("No terrain generated: roads will be flat.");
    } else if (m_terrain_aware_roads && !m_use_chunked_terrain) {
        ImGui::TextDisabled("Legacy terrain mode has no carve: roads will be flat.");
    }

    if (ImGui::Checkbox("Solve Junctions", &m_solve_junctions)) {
        // The toggle changes the geometry of every edge that meets another, not
        // only the junction fills: the arms are extruded from a trimmed
        // centerline. Nothing in the current network survives it, so re-solve now
        // rather than leaving the panel describing a network the toggle no longer
        // matches.
        begin_road_network_rebuild();
    }
    ImGui::SetItemTooltip(
        "Trim each arm back from its node, fill the intersection, and fillet the corners.\n"
        "Off: every road is extruded full length and ribbons overlap at every junction.\n"
        "Off is the P2 reference output, kept so a junction defect can be bisected.");

    // Detail passes. Each one reproduces the previous phase exactly on its own, so
    // a visual defect can be bisected to a pass by flipping one box and
    // re-solving, which is a second, rather than by rebuilding with it compiled
    // out. All three change the geometry of the pieces themselves, so each has to
    // re-solve the network rather than only redraw it.
    if (ImGui::Checkbox("Lane Markings", &m_emit_markings)) {
        begin_road_network_rebuild();
    }
    ImGui::SetItemTooltip(
        "Centre lines, edge lines, stop lines, give-way triangles and turn arrows.\n"
        "Painted into the Markings material as separate quads above the surface.\n"
        "Off: the carriageway keeps its surfaces and carries no paint.");

    ImGui::SameLine();
    if (ImGui::Checkbox("Crossings", &m_emit_crossings)) {
        begin_road_network_rebuild();
    }
    ImGui::SetItemTooltip(
        "Zebra stripes at highway=crossing nodes, and dropped kerbs in the curb ring.\n"
        "Independent of Lane Markings: a crossing is found from OSM topology, a lane\n"
        "line is derived from the profile, and the two fail in different ways.");

    if (ImGui::Checkbox("Bridges and Tunnels", &m_emit_structures)) {
        begin_road_network_rebuild();
    }
    ImGui::SetItemTooltip(
        "Bridge deck slabs, parapets and piers; tunnel portal headwalls.\n"
        "Both are cut against the ground under the road, so both need terrain-aware\n"
        "roads. Off: a bridge is a bare ribbon and a tunnel has no mouth.");

    if (m_emit_structures && !m_terrain_aware_roads) {
        ImGui::TextDisabled("Structures need terrain-aware roads: none will be emitted.");
    }

    // Geometry reduction. Both change what is built rather than what is drawn, so
    // both re-solve, and both are bisectable the same way the detail passes are.
    if (ImGui::Checkbox("Reduce Tessellation", &m_reduce_tessellation)) {
        begin_road_network_rebuild();
    }
    ImGui::SetItemTooltip(
        "Drop stations a straight road does not need, and merge coplanar strip quads.\n"
        "Bounded by a chord deviation and a span cap, so the centerline never moves far.\n"
        "Off: the pre-reduction geometry, which the golden tests diff against.");

    ImGui::SameLine();
    if (ImGui::Checkbox("Chunk LOD", &m_chunk_lod)) {
        // Not a draw-time switch. It decides how pieces are routed into the
        // leaves -- triangle by triangle when on -- so the tree has to be
        // rebuilt, not merely redrawn.
        begin_road_network_rebuild();
    }
    ImGui::SetItemTooltip(
        "Merge each leaf's road pieces and simplify the merged mesh into a level chain.\n"
        "Only the level the camera distance selects is ever uploaded.\n"
        "Off: every leaf keeps one full-detail mesh, routed whole by piece anchor.");

    ImGui::Separator();

    // File path input. Kept alongside the picker so a path can still be pasted or
    // typed, which is also the fallback if the platform has no dialog available.
    ImGui::InputText("File Path", m_osm_filepath, sizeof(m_osm_filepath));
    ImGui::SameLine();
    ImGui::BeginDisabled(m_file_pick.pending);
    if (ImGui::Button(m_file_pick.pending ? "Browsing..." : "Browse...")) {
        open_osm_file_dialog();
    }
    ImGui::EndDisabled();

    // Import button with status feedback
    static std::string import_status;
    static bool import_error = false;

    // An export re-solves the network from m_osm_parser's data on a worker, so it
    // locks the parser for exactly the same reason an import in flight does.
    const bool importing = (m_import_stage == ImportStage::Parsing ||
                            m_import_stage == ImportStage::BuildingRoads ||
                            m_import_stage == ImportStage::Indexing ||
                            m_import_stage == ImportStage::BuildingMeshes ||
                            m_import_stage == ImportStage::CarvingTerrain) ||
                           export_in_flight();

    ImGui::BeginDisabled(importing);
    if (ImGui::Button("Import OSM File", ImVec2(-1, 0))) {
        if (strlen(m_osm_filepath) == 0) {
            import_status = "Please enter a file path first";
            import_error = true;
        } else {
            import_status.clear();
            import_error = false;
            begin_osm_import(m_osm_filepath, config);
        }
    }
    ImGui::EndDisabled();

    // Progress, driven by poll_osm_import()
    if (importing) {
        const char* stage_label =
            m_import_stage == ImportStage::Parsing        ? "1/5 Parsing" :
            m_import_stage == ImportStage::BuildingRoads  ? "2/5 Road network" :
            m_import_stage == ImportStage::Indexing       ? "3/5 Spatial index" :
            m_import_stage == ImportStage::BuildingMeshes ? "4/5 Building meshes" :
                                                            "5/5 Terrain carve";

        // The parser reports item counts only for some stages, so a zero fraction
        // means "unknown", not "nothing done" -- show an indeterminate bar rather
        // than one frozen at 0%.
        if (m_import_fraction > 0.0f) {
            ImGui::ProgressBar(m_import_fraction, ImVec2(-1, 0));
        } else {
            const float t = fmodf((float)ImGui::GetTime() * 0.8f, 1.0f);
            ImGui::ProgressBar(-1.0f * t, ImVec2(-1, 0), "working...");
        }
        ImGui::Text("%s", stage_label);
        if (!m_import_message.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("- %s", m_import_message.c_str());
        }
        if (m_import_stage == ImportStage::BuildingMeshes && m_import_nodes_total > 0) {
            ImGui::Text("Nodes: %zu / %zu",
                        (size_t)(m_import_fraction * m_import_nodes_total + 0.5f),
                        m_import_nodes_total);
        }
    } else if (m_import_stage == ImportStage::Failed) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", m_import_message.c_str());
    } else if (m_import_stage == ImportStage::Done) {
        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "%s", m_import_message.c_str());
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

        if (m_have_road_stats) {
            ImGui::Spacing();
            ImGui::Text("Road Network:");
            ImGui::BulletText("Pieces: %zu (%zu triangles)",
                              m_road_stats.pieces, m_road_stats.triangles);
            ImGui::BulletText("Build: %.1f ms", m_road_stats.build_ms);

            ImGui::Spacing();
            if (m_road_built_on_terrain) {
                ImGui::Text("Elevation Solve:");
                ImGui::BulletText("Edges: %zu of %zu elevated in %.1f ms",
                                  m_road_stats.elevated_edges, m_road_stats.edges,
                                  m_road_stats.elevation_ms);
                ImGui::BulletText("Iterations: %zu (residual %.3f m)",
                                  m_road_elevation_stats.iterations,
                                  m_road_elevation_stats.max_residual);
                ImGui::BulletText("Max grade: %.1f%% (%zu edges grade-limited)",
                                  m_road_max_grade * 100.0f,
                                  m_road_elevation_stats.grade_limited_edges);
                ImGui::BulletText("Bridges: %zu   Tunnels: %zu",
                                  m_road_elevation_stats.bridges,
                                  m_road_elevation_stats.tunnels);
                if (m_terrain_tile_manager.has_road_carve_data()) {
                    ImGui::BulletText("Terrain carved: yes");
                } else {
                    ImGui::BulletText("Terrain carved: no");
                }
            } else {
                ImGui::TextDisabled("Roads are flat: no terrain surface to follow.");
            }

            ImGui::Spacing();
            if (m_road_solved_junctions) {
                ImGui::Text("Junctions:");
                ImGui::BulletText("Solved: %zu   Roundabouts: %zu",
                                  m_road_junction_stats.junctions,
                                  m_road_junction_stats.roundabouts);
                ImGui::BulletText("Tapers: %zu   Dead ends: %zu",
                                  m_road_junction_stats.tapers,
                                  m_road_junction_stats.dead_ends);
                ImGui::BulletText("Pieces: %zu   Trimmed edges: %zu",
                                  m_road_stats.junction_pieces,
                                  m_road_stats.trimmed_edges);
                ImGui::BulletText("Solve: %.1f ms", m_road_stats.junction_ms);

                // A degenerate node fell back to a provisional disc and emitted no
                // fill; an over-trimmed edge is one the junction polygon still
                // overlaps. Neither is visible in the geometry without looking for
                // it, so both are called out rather than buried in the list above.
                if (m_road_junction_stats.degenerate > 0) {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                       "  %zu degenerate: arms too wide for the node.",
                                       m_road_junction_stats.degenerate);
                }
                if (m_road_junction_stats.over_trimmed_edges > 0) {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                       "  %zu over-trimmed: the trim clamp bound.",
                                       m_road_junction_stats.over_trimmed_edges);
                }
                if (m_road_stats.trimmed_away_edges > 0) {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                       "  %zu edges consumed entirely by their trims.",
                                       m_road_stats.trimmed_away_edges);
                }
            } else {
                ImGui::TextDisabled("Junction solver off: ribbons overlap at every node.");
            }

            ImGui::Spacing();
            ImGui::Text("Detail:");

            // Every count below is zero both when its pass was switched off and
            // when the pass ran and found nothing. The two mean opposite things to
            // anyone reading the panel, so the flag decides which is shown and the
            // count is never left to imply it.
            if (m_road_emitted_markings) {
                ImGui::BulletText("Marking pieces: %zu", m_road_stats.markings_pieces);
            } else {
                ImGui::BulletText("Marking pieces: off");
            }

            if (m_road_emitted_crossings) {
                ImGui::BulletText("Crossings: %zu", m_road_stats.crossings);
            } else {
                ImGui::BulletText("Crossings: off");
            }

            if (m_road_emitted_structures) {
                ImGui::BulletText("Bridges: %zu   Tunnels: %zu (%zu portal mouths)",
                                  m_road_stats.bridges, m_road_stats.tunnels,
                                  m_road_portal_mouths);
            } else if (!m_emit_structures) {
                ImGui::BulletText("Bridges and tunnels: off");
            } else {
                // The flag was on but the builder skipped the pass, which it does
                // whenever there is no terrain to cut a pier or a portal against.
                ImGui::BulletText("Bridges and tunnels: skipped, no terrain surface");
            }

            // Counted per SIDE, not per edge: an edge whose sidewalk is separately
            // mapped on both sides adds two.
            ImGui::BulletText("Sidewalk sides deduped: %zu", m_road_stats.deduped_sidewalks);

            ImGui::Spacing();
            ImGui::Text("Tessellation:");
            if (m_reduce_tessellation) {
                const size_t before = m_road_stats.stations_before;
                const size_t after = m_road_stats.stations_after;
                ImGui::BulletText("Stations: %zu -> %zu (%.1f%%)", before, after,
                                  before ? 100.0 * static_cast<double>(after)
                                                 / static_cast<double>(before)
                                         : 100.0);
                // A merge removes two triangles, so the pair is what the lateral
                // pass actually contributed and the count alone is half the story.
                ImGui::BulletText("Quads merged: %zu (%zu triangles)",
                                  m_road_stats.quads_merged, m_road_stats.quads_merged * 2);
                const size_t tri_before = m_road_stats.triangles_before_tess;
                ImGui::BulletText("Triangles: %zu -> %zu (%.1f%%)", tri_before,
                                  m_road_stats.triangles,
                                  tri_before ? 100.0 * static_cast<double>(m_road_stats.triangles)
                                                     / static_cast<double>(tri_before)
                                             : 100.0);
                ImGui::BulletText("Corridor kerbs dropped on %zu edges",
                                  m_road_stats.corridor_kerb_edges);
            } else {
                ImGui::BulletText("Reduction: off (%zu stations, %zu triangles)",
                                  m_road_stats.stations_before, m_road_stats.triangles);
            }

            draw_chunk_lod_stats();

            // A portal mouth is the only carve primitive the road geometry cannot
            // stand without: the headwall frames an opening the hillside would
            // otherwise close over. Say so when portals were built and the terrain
            // never received them.
            if (m_road_portal_mouths > 0 && !m_terrain_tile_manager.has_road_carve_data()) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                   "  %zu portal mouths are not carved: the terrain has no "
                                   "carve data.",
                                   m_road_portal_mouths);
            }

            // The surface the roads WOULD be solved against right now. It differs
            // from the one they WERE solved against whenever terrain was
            // generated or regenerated while an import was in flight, or the
            // terrain-aware toggle was flipped and the rebuild was refused.
            const uint64_t live_surface = live_road_terrain_fingerprint();

            if (live_surface != m_road_terrain_fingerprint) {
                // A live surface of zero is not "a different terrain", it is no
                // terrain at all: the chunks were cleared, chunked mode was
                // switched off, or terrain-aware roads were switched off. The
                // rebuild in that direction flattens the network rather than
                // elevating it, so say which one the button does.
                const bool to_flat = (live_surface == 0);
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                   to_flat ? "Roads are elevated but no terrain surface is active."
                                           : "Roads were solved against a different terrain.");
                ImGui::BeginDisabled(importing);
                if (ImGui::Button(to_flat ? "Re-solve Roads Flat" : "Re-solve Against Terrain",
                                  ImVec2(-1, 0))) {
                    begin_road_network_rebuild();
                }
                ImGui::EndDisabled();
            }
        }

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
        // Clearing mid-import would drop the quadtree nodes that m_import_pending_nodes
        // still points at, and poll_osm_import() would then read freed memory.
        ImGui::BeginDisabled(importing);
        if (ImGui::Button("Clear Data", ImVec2(-1, 0))) {
            m_import_stage = ImportStage::Idle;
            m_import_pending_nodes.clear();
            m_import_nodes_total = 0;
            m_import_message.clear();
            m_osm_parser.clear();

            // Every leaf about to be destroyed may still own GPU meshes, and
            // m_mesh_owners holds a raw pointer to each of them. Dropping the tree
            // without this leaks the geometry AND leaves entries naming freed
            // nodes, which the next eviction would follow.
            if (m_gpu_renderer) {
                for (auto* leaf : m_quadtree.get_all_leaves()) {
                    if (leaf) release_node_from_gpu(*leaf, *m_gpu_renderer);
                }
            }
            m_quadtree.clear();
            m_building_meshes.clear();
            m_road_meshes.clear();
            m_area_meshes.clear();

            // The carve describes a road network that no longer exists, so leaving
            // it installed would keep cutting trenches for roads the user just
            // deleted. Dropping it regenerates the affected chunks.
            m_pending_carve.reset();
            m_carve_apply_pending = false;
            m_terrain_tile_manager.clear_road_carve_data();

            m_have_road_stats = false;
            m_road_built_on_terrain = false;
            m_road_terrain_fingerprint = 0;
            m_road_stats = {};
            m_road_elevation_stats = {};
            m_road_max_grade = 0.0f;
            m_road_solved_junctions = false;
            m_road_junction_stats = {};
            m_road_emitted_markings = false;
            m_road_emitted_crossings = false;
            m_road_emitted_structures = false;
            m_road_portal_mouths = 0;
        }
        ImGui::EndDisabled();
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

            // The global Metallic / Roughness / Ambient Occlusion sliders that used
            // to sit here have been REMOVED rather than left as decoration. They
            // drove SceneUniforms::pbr_params, and mesh_pbr.frag stopped reading it
            // the moment materials owned those three values: the compiled module
            // contains no access to that member at all, so the sliders moved no
            // pixel at any value. They are per-material controls now.
            ImGui::TextDisabled("Metallic, roughness and AO are per-material.");
            if (ImGui::SmallButton("Open Materials panel")) {
                m_show_material_panel = true;
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

    // Contribution culling
    ImGui::Checkbox("Contribution Culling", &m_use_contribution_culling);
    if (m_use_contribution_culling) {
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("Threshold (px)", &m_contribution_threshold, 1.0f, 20.0f, "%.1f");
    }

    // Stats
    if (m_quadtree.leaf_count() > 0) {
        ImGui::Separator();
        ImGui::Text("QuadTree Statistics");


        ImGui::BulletText("Leaves: %zu", m_quadtree.leaf_count());
        ImGui::BulletText("Max Depth: %d", m_quadtree.max_depth());
        ImGui::BulletText("Roads: %zu", m_quadtree.total_roads());
        ImGui::BulletText("Buildings: %zu", m_quadtree.total_buildings());
        ImGui::BulletText("Areas: %zu", m_quadtree.total_areas());
        if (m_gpu_renderer) {
            const auto stats = m_gpu_renderer->get_frame_stats();
            ImGui::BulletText("Draw calls: %u", stats.draw_calls);
            ImGui::BulletText("Triangles drawn: %u", stats.triangles);
        }
    }

    ImGui::End();
}

void Editor::toggle_fullscreen() {
    if (!m_window_handle) return;

    SDL_Window* window = static_cast<SDL_Window*>(m_window_handle);
    m_fullscreen = !m_fullscreen;

    if (!SDL_SetWindowFullscreen(window, m_fullscreen)) {
        spdlog::error("Failed to toggle fullscreen: {}", SDL_GetError());
        m_fullscreen = !m_fullscreen;  // Roll back; the window did not change
        return;
    }

    // Any edge drag in progress refers to the pre-toggle geometry.
    m_resize_edge = RESIZE_NONE;
    m_dragging_window = false;

    spdlog::info("Fullscreen {}", m_fullscreen ? "enabled" : "disabled");
}

void Editor::handle_window_resize() {
    if (!m_window_handle) return;

    // The window has no border to drag in fullscreen, and resizing out from under
    // the fullscreen state fights the window manager.
    if (m_fullscreen) {
        m_resize_edge = RESIZE_NONE;
        return;
    }

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
        SDL_GetGlobalMouseState(&m_resize_start_global_x, &m_resize_start_global_y);
        m_drag_start_window_x = win_x;
        m_drag_start_window_y = win_y;
        m_resize_start_w = win_w;
        m_resize_start_h = win_h;
    }

    // Handle active resize
    if (m_resize_edge != RESIZE_NONE) {
        if (ImGui::IsMouseDown(0)) {
            // Measure the drag against the desktop, not the window. ImGui's mouse
            // position is window-relative, so when a left/top drag moves the window
            // origin the reported position shifts too -- the delta then partly
            // cancels itself and the edge stutters instead of tracking the cursor.
            float gx, gy;
            SDL_GetGlobalMouseState(&gx, &gy);
            float dx = gx - m_resize_start_global_x;
            float dy = gy - m_resize_start_global_y;

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

            // Only talk to the window manager when something actually changed.
            // These are round-trips to the WM/compositor, and calling both of them
            // unconditionally every frame for the whole duration of a drag was the
            // main source of the resize lag -- it cost a pair of round-trips per
            // frame even while the cursor was completely still.
            if (new_x != win_x || new_y != win_y) {
                SDL_SetWindowPosition(window, new_x, new_y);
            }
            if (new_w != win_w || new_h != win_h) {
                SDL_SetWindowSize(window, new_w, new_h);
            }
        } else {
            m_resize_edge = RESIZE_NONE;
        }
    }
}

void Editor::open_osm_file_dialog() {
    open_file_dialog(FilePickTarget::OsmFile);
}

void Editor::open_file_dialog(FilePickTarget target) {
    if (m_file_pick.pending) return;  // a dialog is already up

    // These must outlive the call: SDL requires the filter array stay valid until
    // the callback fires, and this function returns immediately.
    static const SDL_DialogFileFilter kOsmFilters[] = {
        { "OpenStreetMap data", "osm;pbf;osm.bz2;osm.gz" },
        { "OSM XML",            "osm" },
        { "OSM PBF",            "pbf" },
        { "All files",          "*" },
    };
    // KTX2 first because it is the format the texture manager reads without
    // recompressing; the stb formats are accepted but arrive uncompressed.
    static const SDL_DialogFileFilter kTextureFilters[] = {
        { "Textures",     "ktx2;ktx;png;jpg;jpeg;tga;bmp;hdr" },
        { "KTX2",         "ktx2;ktx" },
        { "All files",    "*" },
    };
    static const SDL_DialogFileFilter kMaterialSetFilters[] = {
        { "Stratum material set", "json" },
        { "All files",            "*" },
    };

    const SDL_DialogFileFilter* filters = kOsmFilters;
    int filter_count = static_cast<int>(SDL_arraysize(kOsmFilters));
    switch (target) {
        case FilePickTarget::MaterialAlbedo:
        case FilePickTarget::MaterialNormal:
        case FilePickTarget::MaterialOrm:
            filters = kTextureFilters;
            filter_count = static_cast<int>(SDL_arraysize(kTextureFilters));
            break;
        case FilePickTarget::MaterialSetLoad:
        case FilePickTarget::MaterialSetSave:
            filters = kMaterialSetFilters;
            filter_count = static_cast<int>(SDL_arraysize(kMaterialSetFilters));
            break;
        case FilePickTarget::OsmFile:
            break;
    }

    // Set the target BEFORE marking pending: poll_file_dialog() only reads it once
    // a result has landed, and a result cannot land before the dialog is shown.
    m_file_pick_target = target;
    m_file_pick.pending = true;

    // One callback for every target. It may run on a different thread than the
    // main loop, so it does nothing but record the outcome; poll_file_dialog()
    // applies it on the main thread next frame.
    const SDL_DialogFileCallback callback =
        [](void* userdata, const char* const* filelist, int /*filter*/) {
            auto* self = static_cast<Editor*>(userdata);
            std::lock_guard<std::mutex> lock(self->m_file_pick.mutex);
            self->m_file_pick.has_result = true;
            self->m_file_pick.path.clear();
            self->m_file_pick.error.clear();

            if (!filelist) {
                const char* err = SDL_GetError();
                self->m_file_pick.error = (err && *err) ? err : "file dialog failed";
            } else if (filelist[0]) {
                self->m_file_pick.path = filelist[0];  // single-select
            }
            // filelist non-null with a null first entry means the user cancelled;
            // both strings stay empty and the UI simply does nothing.
        };

    auto* parent = static_cast<SDL_Window*>(m_window_handle);  // for modality

    if (target == FilePickTarget::MaterialSetSave) {
        // Start in the directory the set was last saved to or loaded from, so a
        // re-save lands beside its textures rather than in the home directory --
        // the paths inside the file are written RELATIVE to it.
        SDL_ShowSaveFileDialog(callback, this, parent, filters, filter_count,
                               m_material_set_path.empty() ? nullptr
                                                           : m_material_set_path.c_str());
    } else {
        SDL_ShowOpenFileDialog(callback, this, parent, filters, filter_count,
                               nullptr,  // platform default location
                               false);   // single selection
    }
}

void Editor::poll_file_dialog() {
    std::string path, error;
    {
        std::lock_guard<std::mutex> lock(m_file_pick.mutex);
        if (!m_file_pick.has_result) return;
        m_file_pick.has_result = false;
        m_file_pick.pending = false;
        path = std::move(m_file_pick.path);
        error = std::move(m_file_pick.error);
        m_file_pick.path.clear();
        m_file_pick.error.clear();
    }

    if (!error.empty()) {
        // Most likely on Linux with no XDG desktop portal and no zenity/kdialog.
        // The OSM path field is still there to type into, so this is not fatal
        // there; for the material targets it means the button simply does nothing,
        // which is why the reason is put on the console rather than only in the log.
        spdlog::error("File dialog unavailable: {}", error);
        char msg[512];
        snprintf(msg, sizeof(msg),
                 "[Editor] File dialog unavailable (%s) - type a path instead\n",
                 error.c_str());
        m_console_buffer.append(msg);
        m_console_scroll_to_bottom = true;
        if (m_file_pick_target != FilePickTarget::OsmFile) {
            m_material_set_status = "file dialog unavailable: " + error;
        }
        return;
    }

    // Cancelled. Every target treats that as "do nothing", so it is handled once
    // here rather than in each branch below.
    if (path.empty()) return;

    switch (m_file_pick_target) {
        case FilePickTarget::OsmFile:
            std::snprintf(m_osm_filepath, sizeof(m_osm_filepath), "%s", path.c_str());
            spdlog::info("Selected OSM file: {}", path);
            break;

        case FilePickTarget::MaterialAlbedo:
        case FilePickTarget::MaterialNormal:
        case FilePickTarget::MaterialOrm: {
            if (!m_material_library) break;
            const auto map =
                m_file_pick_target == FilePickTarget::MaterialAlbedo
                    ? MaterialLibrary::TextureMap::Albedo
                    : m_file_pick_target == FilePickTarget::MaterialNormal
                          ? MaterialLibrary::TextureMap::Normal
                          : MaterialLibrary::TextureMap::Orm;
            // Goes through the library, not through GPUTextureManager directly, so
            // the source path is recorded and survives the next save. See
            // MaterialLibrary::load_map_from_file().
            if (m_material_library->load_map_from_file(m_material_pick_key, map, path)) {
                m_material_set_status = "loaded " + path;
            } else {
                m_material_set_status = "failed to load " + path;
            }
            break;
        }

        case FilePickTarget::MaterialSetLoad: {
            if (!m_material_library) break;
            if (m_material_library->load_from_file(path)) {
                m_material_set_path = path;
                m_material_set_status =
                    "loaded " + std::to_string(m_material_library->size()) + " materials";
                // The new set is a different set: counts collected against the old
                // one describe materials that no longer exist.
                m_material_library->reset_resolve_stats();
            } else {
                m_material_set_status = "load failed - see console";
            }
            break;
        }

        case FilePickTarget::MaterialSetSave: {
            if (!m_material_library) break;
            if (m_material_library->save_to_file(path)) {
                m_material_set_path = path;
                m_material_set_status = "saved to " + path;
            } else {
                m_material_set_status = "save failed - see console";
            }
            break;
        }
    }
}

// ============================================================================
// Road network export
// ============================================================================

void Editor::open_export_dir_dialog() {
    if (m_dir_pick.pending) return;  // a dialog is already up

    m_dir_pick.pending = true;

    SDL_ShowOpenFolderDialog(
        [](void* userdata, const char* const* filelist, int /*filter*/) {
            // May run on a different thread than the main loop, so record the
            // outcome and nothing else. poll_export_dir_dialog() applies it.
            auto* self = static_cast<Editor*>(userdata);
            std::lock_guard<std::mutex> lock(self->m_dir_pick.mutex);
            self->m_dir_pick.has_result = true;
            self->m_dir_pick.path.clear();
            self->m_dir_pick.error.clear();

            if (!filelist) {
                const char* err = SDL_GetError();
                self->m_dir_pick.error = (err && *err) ? err : "folder dialog failed";
            } else if (filelist[0]) {
                self->m_dir_pick.path = filelist[0];
            }
            // A non-null list with a null first entry is a cancel: both strings
            // stay empty and the UI does nothing.
        },
        this,
        static_cast<SDL_Window*>(m_window_handle),  // parent, for modality
        nullptr,                                    // platform default location
        false);                                     // single selection
}

void Editor::poll_export_dir_dialog() {
    std::string path, error;
    {
        std::lock_guard<std::mutex> lock(m_dir_pick.mutex);
        if (!m_dir_pick.has_result) return;
        m_dir_pick.has_result = false;
        m_dir_pick.pending = false;
        path = std::move(m_dir_pick.path);
        error = std::move(m_dir_pick.error);
        m_dir_pick.path.clear();
        m_dir_pick.error.clear();
    }

    if (!error.empty()) {
        // Most likely on Linux with no XDG desktop portal and no zenity/kdialog.
        // The path field is still there to type into, so this is not fatal.
        spdlog::error("Folder dialog unavailable: {}", error);
        m_export_status = "Folder dialog unavailable - type a path instead";
        return;
    }

    if (!path.empty()) {
        std::snprintf(m_export_dir, sizeof(m_export_dir), "%s", path.c_str());
        spdlog::info("Export directory: {}", path);
    }
}

void Editor::begin_road_export() {
    if (export_in_flight()) {
        return;
    }
    if (m_import_job || m_road_build_future.valid() || m_carve_index_future.valid()) {
        m_export_status = "An import is still running";
        return;
    }
    if (!m_osm_parser.has_data() || m_osm_parser.get_data().roads.empty()) {
        m_export_status = "No road data to export";
        return;
    }
    if (m_export_dir[0] == '\0') {
        m_export_status = "Choose an output directory first";
        return;
    }

    // Safe to hold a pointer into m_osm_parser for the same reason the import's
    // road stage does: begin_osm_import(), begin_road_network_rebuild() and the
    // Clear Data button all refuse to run while this future is valid.
    const osm::ParsedOSMData* parsed = &m_osm_parser.get_data();

    // The network is re-solved rather than kept: an import MOVES its pieces into
    // the quadtree, which merges them per leaf and keeps only the render mesh, so
    // the collision variant and the LOD chain no longer exist by the time anyone
    // asks to export them. Solving again is a second of worker time; holding a
    // second copy of a city's geometry is permanent.
    osm::road::RoadNetworkConfig cfg = make_road_network_config();
    cfg.build_collision = m_export_build_collision;
    cfg.build_lods = m_export_build_lods;

    auto job = std::make_unique<RoadExportJob>();
    job->directory = m_export_dir;
    job->config = m_export_config;
    // The exporter only writes what the build produced, so the two pairs of flags
    // are one decision and are stamped together.
    job->config.export_collision = m_export_build_collision;
    job->config.export_lods = m_export_build_lods;
    job->build_collision = m_export_build_collision;
    job->build_lods = m_export_build_lods;

    const std::string dir = job->directory;
    const osm::road::ExportConfig export_cfg = job->config;

    // Off the UI thread, like the import. The solve alone is seconds on a city
    // extract and the write is hundreds of megabytes of file I/O.
    job->future = std::async(std::launch::async, [parsed, cfg, export_cfg, dir]() {
        osm::road::RoadNetworkBuilder builder;
        osm::road::RoadNetwork network = builder.build(*parsed, cfg);
        return osm::road::export_road_network(network.pieces, std::filesystem::path(dir),
                                              export_cfg);
    });

    m_export_job = std::move(job);
    m_export_status = "Exporting...";
    spdlog::info("Exporting the road network to {}", m_export_dir);
}

void Editor::poll_road_export() {
    if (!m_export_job) return;

    if (!m_export_job->future.valid()) {
        m_export_job.reset();
        return;
    }
    if (m_export_job->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;  // still running; the indeterminate bar keeps animating
    }

    osm::road::ExportStats stats;
    std::string failure;
    try {
        stats = m_export_job->future.get();
    } catch (const std::exception& e) {
        // std::filesystem throws on an unwritable or unreachable destination, and
        // an uncaught exception out of a future's get() takes the editor with it.
        failure = e.what();
    }

    const std::string directory = m_export_job->directory;
    const bool wrote_collision = m_export_job->build_collision;
    const bool wrote_lods = m_export_job->build_lods;
    m_export_job.reset();

    char msg[512];
    if (!failure.empty()) {
        m_export_status = "Export failed: " + failure;
        snprintf(msg, sizeof(msg), "[Export] Failed: %s\n", failure.c_str());
    } else if (stats.files == 0) {
        // Not an exception: every file was refused, or the network held no
        // triangles. Either way nothing reached the disk, and saying "done" would
        // be a lie the user only discovers in the file manager.
        m_export_status = "Export wrote no files - check the destination is writable";
        snprintf(msg, sizeof(msg), "[Export] Wrote no files to %s\n", directory.c_str());
    } else {
        snprintf(msg, sizeof(msg),
                 "[Export] %zu chunk(s), %zu meshes, %zu triangles, %zu vertices, %zu file(s) "
                 "in %.0f ms -> %s%s%s\n",
                 stats.chunks, stats.meshes, stats.triangles, stats.vertices, stats.files,
                 stats.export_ms, directory.c_str(),
                 wrote_collision ? " (+collision)" : "",
                 wrote_lods ? " (+LODs)" : "");
        char status[256];
        snprintf(status, sizeof(status), "Wrote %zu file(s), %zu triangles in %.0f ms",
                 stats.files, stats.triangles, stats.export_ms);
        m_export_status = status;
    }

    m_console_buffer.append(msg);
    m_console_scroll_to_bottom = true;
    spdlog::info("{}", msg);
}

// ============================================================================
// GPU memory panel
// ============================================================================

namespace {

/// Bytes as megabytes, for a readout that is never more precise than it is honest
float as_mb(size_t bytes) {
    return static_cast<float>(bytes) / (1024.0f * 1024.0f);
}

} // namespace

void Editor::draw_memory_panel() {
    ImGui::Begin("GPU Memory", &m_show_memory_panel);

    if (!m_gpu_renderer) {
        ImGui::TextDisabled("No renderer attached");
        ImGui::End();
        return;
    }

    GPURenderer& renderer = *m_gpu_renderer;
    const GPURenderer::MemoryBudget budget = renderer.memory_budget();
    const size_t resident = renderer.resident_bytes();

    // ── Resident set ────────────────────────────────────────────────────────
    ImGui::Text("Resident");
    ImGui::Separator();

    const float byte_fraction = budget.max_resident_bytes > 0
        ? static_cast<float>(resident) / static_cast<float>(budget.max_resident_bytes)
        : 0.0f;
    char overlay[64];
    snprintf(overlay, sizeof(overlay), "%.1f / %.0f MB", as_mb(resident),
             as_mb(budget.max_resident_bytes));
    ImGui::ProgressBar(std::clamp(byte_fraction, 0.0f, 1.0f), ImVec2(-1, 0), overlay);

    const size_t meshes = renderer.resident_mesh_count();
    const float mesh_fraction = budget.max_resident_meshes > 0
        ? static_cast<float>(meshes) / static_cast<float>(budget.max_resident_meshes)
        : 0.0f;
    snprintf(overlay, sizeof(overlay), "%zu / %zu meshes", meshes, budget.max_resident_meshes);
    ImGui::ProgressBar(std::clamp(mesh_fraction, 0.0f, 1.0f), ImVec2(-1, 0), overlay);

    ImGui::Text("Tracked handles: %zu", m_mesh_owners.size());

    // ── Budget ──────────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::Text("Budget");
    ImGui::Separator();

    GPURenderer::MemoryBudget edited = budget;
    int budget_mb = static_cast<int>(edited.max_resident_bytes / (1024 * 1024));
    int mesh_cap = static_cast<int>(edited.max_resident_meshes);
    bool changed = false;
    changed |= ImGui::SliderInt("Max MB", &budget_mb, 64, 4096);
    changed |= ImGui::SliderInt("Max Meshes", &mesh_cap, 256, 32768);
    changed |= ImGui::Checkbox("Evict under pressure", &edited.evict_under_pressure);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Off makes the caps diagnostics only: an upload that would breach one\n"
                          "is refused instead, and the refusal shows up as an upload failure.");
    }
    if (changed) {
        edited.max_resident_bytes = static_cast<size_t>(budget_mb) * 1024ull * 1024ull;
        edited.max_resident_meshes = static_cast<size_t>(mesh_cap);
        renderer.set_memory_budget(edited);
    }
    ImGui::SameLine();
    if (ImGui::Button("Evict Now")) {
        const size_t evicted = renderer.evict_to_budget();
        char msg[128];
        snprintf(msg, sizeof(msg), "[GPU] Evicted %zu mesh(es) to budget\n", evicted);
        m_console_buffer.append(msg);
        m_console_scroll_to_bottom = true;
    }

    // ── Pools ───────────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::Text("Buffer pools");
    ImGui::Separator();

    const GPUBufferPool::Stats vertex_stats = renderer.vertex_pool_stats();
    const GPUBufferPool::Stats index_stats = renderer.index_pool_stats();

    if (ImGui::BeginTable("##pools", 6, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Pool");
        ImGui::TableSetupColumn("Blocks");
        ImGui::TableSetupColumn("Reserved");
        ImGui::TableSetupColumn("Used");
        ImGui::TableSetupColumn("Ranges");
        ImGui::TableSetupColumn("Frag");
        ImGui::TableHeadersRow();

        const auto row = [](const char* name, const GPUBufferPool::Stats& st) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(name);
            ImGui::TableNextColumn(); ImGui::Text("%zu", st.blocks);
            ImGui::TableNextColumn(); ImGui::Text("%.1f MB", as_mb(st.bytes_reserved));
            ImGui::TableNextColumn(); ImGui::Text("%.1f MB", as_mb(st.bytes_used));
            ImGui::TableNextColumn(); ImGui::Text("%zu", st.live_allocations);
            ImGui::TableNextColumn();
            // Past ~0.8 with allocations failing is the signature of a free list
            // that has stopped coalescing, so it gets a colour rather than a number
            // nobody reads.
            if (st.fragmentation > 0.8f) {
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%.2f", st.fragmentation);
            } else {
                ImGui::Text("%.2f", st.fragmentation);
            }
        };
        row("Vertex", vertex_stats);
        row("Index", index_stats);
        ImGui::EndTable();
    }

    ImGui::TextDisabled("Blocks are the count that hits the driver's allocation limit.");

    // ── Streaming ───────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::Text("Streaming");
    ImGui::Separator();

    ImGui::Text("Pending uploads: %zu", renderer.pending_upload_count());
    ImGui::Text("Retired ranges:  %zu", renderer.retired_alloc_count());
    ImGui::Text("Evictions:       %zu", renderer.evicted_mesh_count());

    const size_t failures = renderer.upload_failures();
    if (failures > 0) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "Upload failures: %zu", failures);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Geometry is missing. Either the budget refused it or the pool "
                              "could not grow a block.");
        }
    } else {
        ImGui::Text("Upload failures: 0");
    }

    const GPURenderer::FrameStats frame = renderer.get_frame_stats();
    ImGui::Text("Last frame: %u draw calls, %u triangles", frame.draw_calls, frame.triangles);

    // material_binds next to draw_calls is the diagnostic for the redundant-bind
    // cache. Far below draw_calls is healthy -- the submesh ranges are sorted and
    // consecutive meshes share materials. Creeping up towards draw_calls means the
    // sorting has stopped happening somewhere upstream and every range is paying
    // for a uniform push and three sampler binds it did not need.
    ImGui::Text("Material binds: %u (of %u draws)", frame.material_binds, frame.draw_calls);

    // ── Textures ────────────────────────────────────────────────────────────
    //
    // Read straight off the manager rather than through GPURenderer::texture_*(),
    // because load_failures has no renderer accessor and should not get one: the
    // renderer does not load textures, it binds them. Deliberately reported as a
    // SEPARATE figure from the mesh budget above -- resident_bytes() is what
    // evict_to_budget() drives against, and a texture set folded into it would make
    // the renderer evict geometry to reclaim bytes no mesh eviction can free.
    if (m_texture_manager) {
        const auto tex = m_texture_manager->stats();
        ImGui::Separator();
        ImGui::Text("Textures: %zu resident, %.1f MB",
                    tex.textures, static_cast<double>(tex.bytes) / (1024.0 * 1024.0));

        if (tex.load_failures > 0) {
            // Not a warning-coloured line by accident: every failed load left a
            // material with an unbound map that is silently drawing plain white,
            // flat-normal or unit-ORM instead. Nothing else on screen says so.
            ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.25f, 1.0f),
                               "Texture load failures: %zu", tex.load_failures);
        } else {
            ImGui::TextDisabled("Texture load failures: 0");
        }

        const size_t pending = m_texture_manager->pending_upload_count();
        if (pending > 0) {
            ImGui::TextDisabled("%zu texture uploads staged", pending);
        }
    } else if (renderer.texture_bytes() > 0) {
        // No manager owned here, but one is installed on the renderer from
        // somewhere else -- a tool or a test harness. Report what is reachable.
        ImGui::Separator();
        ImGui::Text("Textures: %zu, %.1f MB", renderer.texture_count(),
                    static_cast<double>(renderer.texture_bytes()) / (1024.0 * 1024.0));
    }

    // The material system's own headline number lives in the Materials panel; this
    // is the pointer to it, because a stale material set shows up here first as
    // material_binds behaving oddly.
    if (m_material_library && ImGui::SmallButton("Open Materials panel")) {
        m_show_material_panel = true;
    }

    // ── Export ──────────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::Text("Export road network");
    ImGui::Separator();

    const bool busy = export_in_flight();
    const bool importing = m_import_job || m_road_build_future.valid() ||
                           m_carve_index_future.valid();
    const bool have_roads = m_osm_parser.has_data() && !m_osm_parser.get_data().roads.empty();

    ImGui::BeginDisabled(busy);

    ImGui::SetNextItemWidth(-90.0f);
    ImGui::InputText("##ExportDir", m_export_dir, sizeof(m_export_dir));
    ImGui::SameLine();
    if (ImGui::Button("Browse##Export", ImVec2(-1, 0))) {
        open_export_dir_dialog();
    }

    int format = static_cast<int>(m_export_config.format);
    const char* formats[] = { "OBJ + MTL", "glTF 2.0 + .bin" };
    if (ImGui::Combo("Format", &format, formats, 2)) {
        m_export_config.format = static_cast<osm::road::ExportFormat>(format);
    }

    ImGui::SliderFloat("Chunk size", &m_export_config.chunk_size, 0.0f, 2000.0f, "%.0f m");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("0 writes the whole network as one file. The grid is anchored at the\n"
                          "world origin, so two overlapping exports line up.");
    }

    ImGui::Checkbox("Collision mesh", &m_export_build_collision);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Derives a flat collision variant per piece during the re-solve.\n"
                          "Costs roughly a third of the render mesh again.");
    }

    ImGui::Checkbox("LOD chain", &m_export_build_lods);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Simplifies once per material range per level. The most expensive\n"
                          "option by a wide margin on a city extract.");
    }
    if (m_export_build_lods) {
        ImGui::SliderInt("LOD levels", &m_export_config.lod_levels, 2, 4);
    }

    ImGui::BeginDisabled(!have_roads || importing);
    if (ImGui::Button("Export", ImVec2(-1, 0))) {
        begin_road_export();
    }
    ImGui::EndDisabled();

    ImGui::EndDisabled();

    if (busy) {
        // Indeterminate: the exporter reports nothing until it returns, and a bar
        // that sat at 0% would read as a hang.
        ImGui::ProgressBar(-1.0f * static_cast<float>(ImGui::GetTime()), ImVec2(-1, 0),
                           "Re-solving and writing...");
    } else if (!have_roads) {
        ImGui::TextDisabled("Import an OSM file with roads first");
    }

    if (!m_export_status.empty()) {
        ImGui::TextWrapped("%s", m_export_status.c_str());
    }

    ImGui::End();
}

// ============================================================================
// Terrain-aware roads
// ============================================================================

namespace {

/// Boost-style mixer, so the fingerprint depends on field ORDER as well as value
void hash_combine(uint64_t& seed, uint64_t value) {
    seed ^= value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
}

/// Bit pattern of a float, with the two zeroes folded together so they hash alike
uint64_t hash_float(float v) {
    if (v == 0.0f) v = 0.0f;
    uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    return bits;
}

} // namespace

uint64_t Editor::terrain_surface_fingerprint(const procgen::TerrainConfig& cfg) {
    // Only the fields TerrainGenerator::sample_surface() reads: the raw height
    // field, plus the urban flattening modifier, which is positioned from
    // size_x/size_z. Resolution, water level, erosion and mesh settings do not
    // change the surface a road is solved against, so a change to any of them
    // must NOT invalidate the solve.
    uint64_t h = 0xcbf29ce484222325ull;
    hash_combine(h, static_cast<uint64_t>(cfg.seed));
    hash_combine(h, static_cast<uint64_t>(cfg.type));
    hash_combine(h, hash_float(cfg.base_height));
    hash_combine(h, hash_float(cfg.max_height));
    hash_combine(h, hash_float(cfg.noise_scale));
    hash_combine(h, static_cast<uint64_t>(cfg.octaves));
    hash_combine(h, hash_float(cfg.lacunarity));
    hash_combine(h, hash_float(cfg.persistence));
    hash_combine(h, static_cast<uint64_t>(cfg.flatten_center ? 1 : 0));
    hash_combine(h, hash_float(cfg.flatten_radius));
    hash_combine(h, hash_float(cfg.flatten_falloff));
    hash_combine(h, hash_float(cfg.size_x));
    hash_combine(h, hash_float(cfg.size_z));

    // Never collide with the "no terrain" sentinel.
    return h == 0 ? 1 : h;
}

bool Editor::has_generated_terrain() const {
    return m_terrain_tile_manager.generated_count() > 0;
}

uint64_t Editor::live_road_terrain_fingerprint() const {
    return (m_terrain_aware_roads && m_use_chunked_terrain && has_generated_terrain())
               ? terrain_surface_fingerprint(m_terrain_tile_manager.get_config().terrain)
               : 0;
}

osm::road::HeightSampler Editor::make_terrain_height_sampler() const {
    if (!m_terrain_aware_roads) return nullptr;

    // The legacy single-terrain path has no carve hook, so elevating roads
    // against it would leave them following a surface nothing ever cuts. Flat
    // roads on flat-looking terrain beat roads buried in an uncarved hillside.
    if (!m_use_chunked_terrain) return nullptr;
    if (!has_generated_terrain()) return nullptr;

    // The config the CHUNKS were generated from, not the panel's live edit
    // buffer: the buffer may already hold settings the user has not pressed
    // Generate on, and solving against a surface that does not exist yet would
    // float every road.
    const procgen::TerrainConfig terrain = m_terrain_tile_manager.get_config().terrain;

    // A generator of our own. TerrainTileManager's is private, and
    // TerrainGenerator::sample_surface() is only re-entrant as long as nothing
    // reseeds the same instance -- which generate_chunk() does, on the main
    // thread, while this sampler is being called from the solver's workers.
    // Seeded from the same config, a separate instance gives bit-identical
    // heights with none of that coupling. Held by shared_ptr so it outlives the
    // async build even if the editor's terrain settings change meanwhile.
    auto generator = std::make_shared<procgen::TerrainGenerator>(terrain.seed);

    return [terrain, generator](double x, double y) -> float {
        // Roads carry LOCAL 2D metres, and so does the terrain height field: its
        // second argument is the same local y, NOT render-space Z. Both sides
        // negate independently on their way to render space -- terrain through
        // TerrainMeshBuilder's vec3(world_x, h, -world_z), roads through
        // vec3(x, h, -y_2d) -- so the two agree exactly when the second argument
        // is passed through unchanged.
        //
        // Negate it and every road is elevated from the height at its own mirror
        // image across the equator of the import: still smooth, still plausible,
        // and completely wrong everywhere the terrain is not symmetric.
        return generator->sample_surface(terrain,
                                         static_cast<float>(x),
                                         static_cast<float>(y));
    };
}

osm::road::RoadNetworkConfig Editor::make_road_network_config() const {
    osm::road::RoadNetworkConfig cfg;
    cfg.height_sampler = make_terrain_height_sampler();
    cfg.solve_junctions = m_solve_junctions;
    cfg.emit_markings = m_emit_markings;
    cfg.emit_crossings = m_emit_crossings;
    cfg.emit_structures = m_emit_structures;
    cfg.reduce_tessellation = m_reduce_tessellation;
    return cfg;
}

Editor::RoadBuildResult Editor::run_road_network_build(const osm::ParsedOSMData& data,
                                                       const osm::road::RoadNetworkConfig& cfg) {
    RoadBuildResult result;
    result.elevated = static_cast<bool>(cfg.height_sampler);
    result.solved_junctions = cfg.solve_junctions;

    // Structures need a terrain height under the road, so the builder skips them
    // whatever the flag says when there is no sampler. Recording the flag alone
    // would leave the panel reporting a pass that never ran.
    result.emitted_markings = cfg.emit_markings;
    result.emitted_crossings = cfg.emit_crossings;
    result.emitted_structures = cfg.emit_structures && static_cast<bool>(cfg.height_sampler);

    osm::road::RoadNetworkBuilder builder;
    result.network = builder.build(data, cfg);

    // The builder owns the elevation solver and dies with this function, so
    // everything the panel reads out has to be copied here.
    const osm::road::RoadElevationSolver& solver = builder.elevation();
    if (solver.is_solved()) {
        result.elevation = solver.stats();
        for (const osm::road::EdgeElevation& edge : solver.edges()) {
            result.max_grade = std::max(result.max_grade, edge.max_grade_used);
        }
    } else {
        // A sampler was supplied but the solve did not produce a result (no
        // usable roads, or a centerline/graph size mismatch). The network is
        // flat, so say so rather than showing an empty elevation readout.
        result.elevated = false;
    }

    return result;
}

void Editor::begin_road_network_rebuild() {
    if (m_import_job || m_road_build_future.valid() || m_carve_index_future.valid() ||
        export_in_flight()) {
        // Deferred, not dropped: the refused request is the NEWER one, and the
        // build in flight is about to stamp a fingerprint that will then look
        // current. finish_osm_import() picks this up.
        m_road_rebuild_owed = true;
        spdlog::warn("Road rebuild deferred: an import is already running");
        return;
    }
    m_road_rebuild_owed = false;
    if (!m_osm_parser.has_data() || m_osm_parser.get_data().roads.empty()) {
        return;
    }

    // Safe to hold a pointer into m_osm_parser for the same reason the import
    // path does: begin_osm_import() and the Clear Data button both refuse to run
    // while a road build is in flight.
    const osm::ParsedOSMData* parsed = &m_osm_parser.get_data();

    m_road_rebuild_only = true;
    m_import_stage = ImportStage::BuildingRoads;
    m_import_message = "Re-solving the road network...";
    m_import_fraction = 0.0f;
    m_import_nodes_total = 0;
    m_import_pending_nodes.clear();

    // Both captures read the terrain config on THIS thread, in this statement, so
    // the sampler and the fingerprint describe one and the same surface however
    // the panel is driven while the build runs.
    m_road_build_future = std::async(std::launch::async,
                                     [parsed,
                                      cfg = make_road_network_config(),
                                      surface = live_road_terrain_fingerprint()]() {
        RoadBuildResult result = run_road_network_build(*parsed, cfg);
        result.terrain_fingerprint = surface;
        return result;
    });

    spdlog::info("Rebuilding the road network against the current terrain");
}

void Editor::maybe_rebuild_roads_for_terrain() {
    if (!m_osm_parser.has_data() || m_osm_parser.get_data().roads.empty()) {
        return;
    }

    // The surface the roads WOULD be solved against now.
    const uint64_t surface = live_road_terrain_fingerprint();

    if (surface == m_road_terrain_fingerprint) {
        return;  // already solved against exactly this surface
    }

    // Rebuilding beats telling the user to re-import. The parsed OSM data is
    // already in memory and only the vertical solve is stale, so a rebuild costs
    // the road pass alone; a re-import costs re-parsing a file that may be
    // hundreds of megabytes. It also cannot be skipped: a network built with no
    // sampler emits NO carve data at all, so without this the terrain would have
    // nothing to carve and every road would sit buried in the hillside.
    begin_road_network_rebuild();
}

void Editor::begin_osm_import(const std::string& filepath, const osm::ParserConfig& config) {
    if (m_import_job) {
        spdlog::warn("An OSM import is already running");
        return;
    }
    // The road worker reads m_osm_parser's data by pointer, and a second import
    // would reassign that parser out from under it.
    if (m_road_build_future.valid() || m_carve_index_future.valid()) {
        spdlog::warn("A road network build is still running; import refused");
        return;
    }
    if (export_in_flight()) {
        spdlog::warn("A road export is still running; import refused");
        return;
    }

    auto job = std::make_unique<OSMImportJob>();
    job->filepath = filepath;
    job->parser = std::make_unique<osm::OSMParser>();
    job->parser->set_config(config);

    // The callback fires on the worker thread. Snapshot under the mutex; the UI
    // reads the same field on the main thread.
    OSMImportJob* j = job.get();
    job->parser->set_progress_callback([j](const osm::ParseProgress& p) {
        std::lock_guard<std::mutex> lock(j->mutex);
        j->progress = p;
    });

    // Safe to capture `j`: the job is only destroyed after its future is ready,
    // and ~OSMImportJob joins the worker before releasing anything it touches.
    job->future = std::async(std::launch::async, [j]() {
        return j->parser->parse(j->filepath);
    });

    m_import_job = std::move(job);
    m_import_stage = ImportStage::Parsing;
    m_import_message = "Parsing...";
    m_import_fraction = 0.0f;
    m_import_nodes_total = 0;
    m_import_pending_nodes.clear();

    spdlog::info("Started async OSM import: {}", filepath);
}

void Editor::poll_osm_import() {
    if (m_import_job) {
        // ── Stage 1: parsing on the worker ──
        {
            std::lock_guard<std::mutex> lock(m_import_job->mutex);
            const auto& p = m_import_job->progress;
            if (!p.message.empty()) m_import_message = p.message;
            m_import_fraction = p.percentage() / 100.0f;
        }

        if (m_import_job->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return;  // still parsing; the UI keeps animating
        }

        const bool ok = m_import_job->future.get();
        if (!ok) {
            const std::string err = m_import_job->parser->get_error();
            m_import_job.reset();
            m_import_stage = ImportStage::Failed;
            m_import_message = err;

            char msg[512];
            snprintf(msg, sizeof(msg), "[OSM] Error: %s\n", err.c_str());
            m_console_buffer.append(msg);
            m_console_scroll_to_bottom = true;
            spdlog::error("OSM import failed: {}", err);
            return;
        }

        // Hand the parsed data over on the main thread, then release the job.
        m_osm_parser = std::move(*m_import_job->parser);
        m_import_job.reset();

        m_osm_parser.log_statistics();
        m_osm_parser.log_sample_data();

        // ── Stage 2: road network ──
        // Solved once over the whole graph rather than per quadtree leaf, so
        // junctions, miters and profile transitions survive leaf boundaries. It
        // is pure stratum_core CPU work, so it goes on a worker like parsing did;
        // the quadtree is not touched until it lands.
        //
        // The lambda holds a pointer into m_osm_parser. That is safe because
        // nothing may reassign or clear the parser while this future is valid:
        // begin_osm_import() and the Clear Data button are both gated on the
        // import being idle.
        m_import_stage = ImportStage::BuildingRoads;
        m_import_message = "Building road network...";
        m_import_fraction = 0.0f;

        const osm::ParsedOSMData* parsed = &m_osm_parser.get_data();
        m_road_rebuild_only = false;
        m_road_build_future = std::async(std::launch::async,
                                         [parsed,
                                          cfg = make_road_network_config(),
                                          surface = live_road_terrain_fingerprint()]() {
            RoadBuildResult result = run_road_network_build(*parsed, cfg);
            result.terrain_fingerprint = surface;
            return result;
        });
        return;
    }

    if (m_import_stage == ImportStage::BuildingRoads) {
        if (!m_road_build_future.valid()) {
            // Cannot happen through the state machine above; recover rather than
            // wedge the panel in a stage that never advances.
            spdlog::error("Road network stage entered with no job; aborting import");
            m_import_stage = ImportStage::Failed;
            m_import_message = "Road network build lost";
            return;
        }

        if (m_road_build_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return;  // still solving; the indeterminate bar keeps animating
        }

        RoadBuildResult result = m_road_build_future.get();
        osm::road::RoadNetwork& network = result.network;

        m_road_stats = network.stats;
        m_road_elevation_stats = result.elevation;
        m_road_junction_stats = network.junction_stats;
        m_road_max_grade = result.max_grade;
        m_road_built_on_terrain = result.elevated;
        m_road_solved_junctions = result.solved_junctions;
        m_road_emitted_markings = result.emitted_markings;
        m_road_emitted_crossings = result.emitted_crossings;
        m_road_emitted_structures = result.emitted_structures;
        m_road_portal_mouths = network.carve_portals.size();
        m_have_road_stats = true;
        // The surface this build was SOLVED against, captured at launch. Reading
        // the manager here instead would record whatever terrain is loaded when
        // the future happens to land, which is a different surface entirely
        // whenever the user regenerated terrain mid-build.
        m_road_terrain_fingerprint = result.elevated ? result.terrain_fingerprint : 0;

        char road_msg[320];
        snprintf(road_msg, sizeof(road_msg),
                 "[OSM] Road network: %zu pieces, %zu triangles from %zu edges in %.0f ms\n",
                 network.stats.pieces, network.stats.triangles, network.stats.edges,
                 network.stats.build_ms);
        m_console_buffer.append(road_msg);

        if (result.elevated) {
            snprintf(road_msg, sizeof(road_msg),
                     "[OSM] Elevation: %zu edges solved in %.0f ms, %zu iterations, "
                     "max grade %.1f%%, %zu bridges, %zu tunnels\n",
                     network.stats.elevated_edges, network.stats.elevation_ms,
                     result.elevation.iterations, result.max_grade * 100.0f,
                     result.elevation.bridges, result.elevation.tunnels);
        } else {
            snprintf(road_msg, sizeof(road_msg),
                     "[OSM] Elevation: skipped, roads are flat (no terrain to follow)\n");
        }
        m_console_buffer.append(road_msg);

        if (result.solved_junctions) {
            snprintf(road_msg, sizeof(road_msg),
                     "[OSM] Junctions: %zu solved, %zu roundabouts, %zu tapers, "
                     "%zu dead ends, %zu degenerate, %zu over-trimmed edges in %.0f ms\n",
                     network.junction_stats.junctions, network.junction_stats.roundabouts,
                     network.junction_stats.tapers, network.junction_stats.dead_ends,
                     network.junction_stats.degenerate,
                     network.junction_stats.over_trimmed_edges,
                     network.stats.junction_ms);
        } else {
            snprintf(road_msg, sizeof(road_msg),
                     "[OSM] Junctions: solver off, ribbons run through every node\n");
        }
        m_console_buffer.append(road_msg);

        snprintf(road_msg, sizeof(road_msg),
                 "[OSM] Detail: %zu marking pieces, %zu crossings, %zu bridges, "
                 "%zu tunnels (%zu portal mouths), %zu sidewalk sides deduped\n",
                 network.stats.markings_pieces, network.stats.crossings,
                 network.stats.bridges, network.stats.tunnels,
                 network.carve_portals.size(), network.stats.deduped_sidewalks);
        m_console_buffer.append(road_msg);
        m_console_scroll_to_bottom = true;

        // The corridors outlive the network: they are indexed and carved once the
        // meshes are done. Moved out before the pieces are handed to the quadtree,
        // because that call consumes the network.
        m_pending_carve = std::make_unique<procgen::CarveInput>();
        m_pending_carve->ribbons = std::move(network.carve_ribbons);
        m_pending_carve->discs   = std::move(network.carve_discs);
        m_pending_carve->portals = std::move(network.carve_portals);
        m_pending_carve->config  = m_carve_config;

        // ── Stage 3: spatial index ──
        // Blocking, but far cheaper than parsing, and it mutates the quadtree that
        // render_3d walks every frame, so it has to stay on the main thread.
        m_import_stage = ImportStage::Indexing;
        m_import_message = "Building spatial index...";
        m_import_fraction = 0.0f;

        begin_mesh_rebuild(std::move(network.pieces), !m_road_rebuild_only);

        m_import_stage = ImportStage::BuildingMeshes;
        m_import_message = "Building meshes...";
        return;
    }

    // ── Stage 5: carve the solved corridors into the terrain ──
    if (m_import_stage == ImportStage::CarvingTerrain) {
        poll_road_carve();
        return;
    }

    if (m_import_stage != ImportStage::BuildingMeshes) {
        return;
    }

    // ── Stage 3: drain the async node builds ──
    // Also polled by render_3d, but do it here too so progress advances even when
    // the Viewport panel is closed.
    m_quadtree.poll_async_builds();

    size_t done = 0;
    for (const auto* node : m_import_pending_nodes) {
        if (node->meshes_built) ++done;
    }

    m_import_fraction = m_import_nodes_total > 0
        ? static_cast<float>(done) / static_cast<float>(m_import_nodes_total)
        : 1.0f;

    if (done < m_import_nodes_total) {
        return;
    }

    // ── Meshes done: hand the corridors to the terrain ──
    m_import_pending_nodes.clear();

    const auto& data = m_osm_parser.get_data();
    char msg[256];
    snprintf(msg, sizeof(msg), "[OSM] Loaded: %zu roads, %zu buildings, %zu areas\n",
             data.roads.size(), data.buildings.size(), data.areas.size());
    m_console_buffer.append(msg);
    m_console_scroll_to_bottom = true;

    m_import_stage = ImportStage::CarvingTerrain;
    m_import_fraction = 0.0f;
    m_import_message = "Preparing terrain carve...";
    begin_road_carve();
}

// ============================================================================
// Terrain carve stage
// ============================================================================

void Editor::begin_road_carve() {
    m_carve_apply_pending = false;

    const bool have_corridors = m_pending_carve && m_pending_carve->item_count() > 0;

    if (!have_corridors) {
        // Roads were built flat, so there is nothing to carve. Any carve data
        // still installed belongs to a network that no longer exists and would
        // hold trenches under roads that have moved, so drop it. Dropping it
        // regenerates the affected chunks, which is the same blocking work an
        // install is, so it goes through the same deferred-by-one-frame path.
        m_pending_carve.reset();
        if (m_terrain_tile_manager.has_road_carve_data()) {
            m_import_message = "Clearing terrain carve...";
            m_carve_apply_pending = true;
        } else {
            finish_osm_import();
        }
        return;
    }

    // Indexing is proportional to the number of corridors, which is proportional
    // to the size of the import, so it goes on a worker like every other
    // whole-network pass. It touches nothing the main thread reads.
    m_import_message = "Indexing road corridors...";
    m_carve_index_future = std::async(std::launch::async,
                                      [input = std::move(m_pending_carve)]() mutable {
        input->build_index();
        return std::move(input);
    });
}

void Editor::poll_road_carve() {
    // Second half of the deferred apply: the stage message set last frame has
    // been drawn, so the blocking regeneration may now run.
    if (m_carve_apply_pending) {
        m_carve_apply_pending = false;
        install_road_carve_data();
        finish_osm_import();
        return;
    }

    if (!m_carve_index_future.valid()) {
        // No index job and no pending apply: nothing left to do in this stage.
        finish_osm_import();
        return;
    }

    if (m_carve_index_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;  // still indexing; the indeterminate bar keeps animating
    }

    m_pending_carve = m_carve_index_future.get();

    // Apply on the NEXT frame. set_road_carve_data() regenerates every existing
    // chunk in one call; doing that in the frame that names the stage means the
    // name never reaches the screen.
    m_import_message = "Carving terrain...";
    m_carve_apply_pending = true;
}

void Editor::install_road_carve_data() {
    if (!m_pending_carve) {
        if (m_terrain_tile_manager.has_road_carve_data()) {
            m_terrain_tile_manager.clear_road_carve_data();
            m_console_buffer.append("[Terrain] Road carve cleared; terrain returned to procedural\n");
            m_console_scroll_to_bottom = true;
        }
        return;
    }

    char msg[256];
    snprintf(msg, sizeof(msg), "[Terrain] Carving %zu corridors and %zu junctions into %zu chunks\n",
             m_pending_carve->ribbons.size(), m_pending_carve->discs.size(),
             m_terrain_tile_manager.chunk_count());
    m_console_buffer.append(msg);
    m_console_scroll_to_bottom = true;

    // Regenerates every already-generated chunk, which is why this is on the main
    // thread: render_3d() walks those chunks every frame. Chunks generated LATER
    // pick the carve up inside generate_chunk(), so an install with no terrain yet
    // is nearly free and still correct.
    m_terrain_tile_manager.set_road_carve_data(std::move(*m_pending_carve));
    m_pending_carve.reset();
}

void Editor::finish_osm_import() {
    m_pending_carve.reset();
    m_carve_apply_pending = false;
    m_import_stage = ImportStage::Done;
    m_import_fraction = 1.0f;
    m_import_message = m_road_rebuild_only ? "Roads re-solved against the terrain"
                                           : "Import successful";
    m_road_rebuild_only = false;
    spdlog::info("OSM import complete");

    // A rebuild asked for while this one was in flight. Honoured through
    // maybe_rebuild_roads_for_terrain() rather than launched directly, so a
    // request that the just-landed build already satisfied costs nothing.
    if (m_road_rebuild_owed) {
        m_road_rebuild_owed = false;
        maybe_rebuild_roads_for_terrain();
    }
}

void Editor::begin_mesh_rebuild(std::vector<osm::road::RoadPiece>&& road_pieces,
                                bool recenter_camera) {
    m_building_meshes.clear();
    m_road_meshes.clear();
    m_area_meshes.clear();

    const auto& osm_data = m_osm_parser.get_data();

    // Initialize quadtree
    spdlog::info("Initializing quadtree...");
    // Every leaf about to be destroyed may still own GPU buffers. clear() only
    // drops the CPU-side tree, so without this the previous tree's buffers stay
    // alive with nothing referencing them for the rest of the session. A road
    // rebuild makes this path routine rather than once-per-import.
    if (m_gpu_renderer) {
        for (auto* leaf : m_quadtree.get_all_leaves()) {
            if (leaf) release_node_from_gpu(*leaf, *m_gpu_renderer);
        }
    }
    m_quadtree.clear();
    // Sized from the features, not osm_data.bounds -- see QuadTree::init(ParsedOSMData).
    m_quadtree.init(osm_data);
    m_quadtree.assign_data(osm_data);

    // Roads are not rebuilt per leaf any more. Hand the already-solved geometry
    // over now, while the leaves exist and before any node build is queued, so
    // the first upload of a leaf already carries its roads.
    //
    // Set BEFORE the hand-off: the flag decides how pieces are routed into the
    // leaves as well as whether a chain is built afterwards, and both happen
    // inside assign_road_pieces().
    m_quadtree.set_chunk_lod(m_chunk_lod, osm::road::ChunkLodConfig{});
    m_quadtree.assign_road_pieces(std::move(road_pieces));

    spdlog::info("QuadTree: {} leaves, {} roads, {} buildings, {} areas, max depth {}",
                 m_quadtree.leaf_count(),
                 m_quadtree.total_roads(),
                 m_quadtree.total_buildings(),
                 m_quadtree.total_areas(),
                 m_quadtree.max_depth());

    // Find geometry center for camera positioning
    glm::vec3 bounds_min, bounds_max;
    m_quadtree.get_bounds(bounds_min, bounds_max);
    bool found_geometry = (bounds_min.x < bounds_max.x || bounds_min.z < bounds_max.z);
    glm::vec3 data_center = (bounds_min + bounds_max) * 0.5f;

    if (!found_geometry) {
        spdlog::warn("No geometry found in quadtree!");
    }

    spdlog::info("Data center: ({}, {}, {})", data_center.x, data_center.y, data_center.z);

    // Center camera on data FIRST (before culling uses camera position)
    glm::vec3 focus_centre;
    float focus_radius = 0.0f;
    const bool have_focus = m_quadtree.get_focus(focus_centre, focus_radius);

    if (have_focus && recenter_camera) {
        // Frame where the features actually are, not the centre of their bounding
        // box. Those differ wildly for an Overpass export, whose box is stretched
        // by the nodes it pulls in for ways crossing the query area.
        data_center = focus_centre;

        // Scale to the data. The old fixed 300m/5000m numbers meant a large import
        // put the camera thousands of metres from anything, so distance culling
        // rejected every node, nothing was ever queued, and the viewport stayed
        // empty with no error shown.
        const float view_distance = std::clamp(focus_radius, 300.0f, 8000.0f);
        glm::vec3 cam_pos = data_center + glm::vec3(0.0f, view_distance * 0.8f, view_distance);

        m_camera.set_position(cam_pos);
        m_camera.set_target(data_center);
        // Depth precision is governed by far/near, so keep that ratio sane rather
        // than pairing a 0.1m near plane with a far plane tens of km out -- that
        // combination puts almost the whole depth buffer in the first few metres
        // and leaves coplanar roads, landuse and building footprints z-fighting.
        // Only draw as far as nodes are actually built, plus headroom.
        m_camera.m_far = std::clamp(view_distance * 6.0f, 20000.0f, 80000.0f);
        m_camera.m_near = std::clamp(m_camera.m_far / 20000.0f, 0.1f, 5.0f);
        m_camera.m_base_speed = std::clamp(focus_radius * 0.1f, 200.0f, 5000.0f);
        // Must reach past the camera's own distance from the data, or distance
        // culling rejects everything before it can be built. Bounded so a huge
        // import does not try to mesh the whole dataset at once -- the remainder
        // streams in as the camera moves.
        m_view_radius = std::clamp(view_distance * 3.0f, 5000.0f, 30000.0f);

        spdlog::info("Camera at ({:.0f}, {:.0f}, {:.0f}) looking at ({:.0f}, {:.0f}, {:.0f}), "
                     "focus radius {:.0f}m, view radius {:.0f}m",
                     cam_pos.x, cam_pos.y, cam_pos.z,
                     data_center.x, data_center.y, data_center.z,
                     focus_radius, m_view_radius);
    } else if (have_focus) {
        // A road rebuild after terrain generation. The geometry is the same data
        // in the same place, so re-framing it would only throw away wherever the
        // user was looking.
        data_center = focus_centre;
        spdlog::info("Road rebuild: leaving the camera where it is");
    } else if (found_geometry) {
        spdlog::warn("No populated quadtree leaves; leaving the camera where it is");
    }

    // Force camera matrix recalculation so frustum matches new position
    // (update() is normally called in draw_viewport, but we need it now for traversal)
    m_camera.update(1.0f); // aspect doesn't matter much, just need valid frustum

    // Enable culling for performance
    m_use_tile_culling = true;
    m_use_distance_culling = true;
    m_use_contribution_culling = false; // disable initially — camera is far, nodes appear small

    // Queue the initially-visible leaves. These builds are already asynchronous;
    // poll_osm_import() drains them across frames and reports progress. Blocking
    // here on a spin-wait used to freeze the UI for the whole build.
    m_import_pending_nodes.clear();

    Frustum frustum = m_camera.get_frustum();
    glm::vec3 cam_pos = m_camera.get_position();

    m_quadtree.traverse_visible(
        frustum.planes, cam_pos, m_view_radius,
        600.0f, m_camera.m_fov, m_contribution_threshold,
        true, true, false, // frustum + distance, no contribution cull
        [&](osm::QuadTreeNode* node, float /*dist_sq*/) {
            if (!node->meshes_built && !node->meshes_pending) {
                if (m_quadtree.queue_node_build_async(node)) {
                    m_import_pending_nodes.push_back(node);
                }
            }
        }
    );

    m_import_nodes_total = m_import_pending_nodes.size();
    spdlog::info("Queued {} initial node builds", m_import_nodes_total);
}



// ============================================================================
// Resident GPU geometry: ownership, distance, and eviction
// ============================================================================

glm::vec3 Editor::node_anchor(const osm::QuadTreeNode& node) {
    if (node.has_valid_bounds()) {
        return (node.bounds_min + node.bounds_max) * 0.5f;
    }
    // No geometry ever grew the AABB, but the cell still locates the leaf. Local
    // 2D (x, y) maps to world (x, height, -y); the height is unknown, so 0.
    return glm::vec3(static_cast<float>(node.center.x), 0.0f,
                     static_cast<float>(-node.center.y));
}

uint32_t Editor::upload_tracked_mesh(GPURenderer& renderer, const Mesh& mesh,
                                     const MeshOwner& owner) {
    const uint32_t id = renderer.upload_mesh(mesh);
    if (id == 0) {
        // Not a handle. Registering it would make the owner of mesh 0 whichever
        // upload failed most recently.
        return 0;
    }
    m_mesh_owners[id] = owner;
    return id;
}

void Editor::release_tracked_mesh(GPURenderer& renderer, uint32_t& mesh_id) {
    if (mesh_id == 0) return;
    m_mesh_owners.erase(mesh_id);
    renderer.release_mesh(mesh_id);
    mesh_id = 0;
}

float Editor::mesh_distance_to_camera(uint32_t mesh_id) const {
    const auto it = m_mesh_owners.find(mesh_id);
    if (it == m_mesh_owners.end()) {
        // Nothing is holding this handle, so nothing will miss it. Reporting it as
        // infinitely far away puts it at the front of the eviction order, which is
        // the right answer for geometry nobody is tracking any more.
        return std::numeric_limits<float>::max();
    }
    if (it->second.kind == MeshOwner::Kind::Pinned) {
        return -1.0f;
    }
    return glm::length(it->second.anchor - m_camera.get_position());
}

void Editor::on_mesh_evicted(uint32_t mesh_id) {
    const auto it = m_mesh_owners.find(mesh_id);
    if (it == m_mesh_owners.end()) return;

    const MeshOwner owner = it->second;
    m_mesh_owners.erase(it);

    // No call back into the renderer from here: it is mid-eviction and walking
    // its own mesh map. Clearing the handle is all this has to do.
    switch (owner.kind) {
        case MeshOwner::Kind::QuadTreeLeaf: {
            if (!owner.node) break;
            std::erase(owner.node->area_gpu_ids, mesh_id);
            if (std::erase(owner.node->road_gpu_ids, mesh_id) > 0) {
                // The resident LOD level went with it. Saying so is what makes
                // sync_node_road_lod() upload again instead of trusting a level
                // that is no longer on the device.
                owner.node->road_lod_resident = -1;
            }
            std::erase(owner.node->building_gpu_ids, mesh_id);
            // The leaf is no longer whole, so it is no longer uploaded. It streams
            // back in the next time it is visible, and upload_node_to_gpu()
            // releases whichever of its handles survived before re-uploading.
            owner.node->gpu_uploaded = false;
            break;
        }
        case MeshOwner::Kind::TerrainChunk: {
            auto* chunk = m_terrain_tile_manager.get_chunk(owner.coord);
            if (!chunk) break;
            if (chunk->terrain_gpu_id == mesh_id) chunk->terrain_gpu_id = 0;
            if (chunk->water_gpu_id == mesh_id) chunk->water_gpu_id = 0;
            // Same contract as a leaf: the re-upload path in render_3d() releases
            // the surviving handle before replacing both.
            chunk->gpu_uploaded = false;
            break;
        }
        case MeshOwner::Kind::Pinned:
            // Unreachable: a pinned mesh reports a negative distance and is never
            // a candidate. If it happens, the handle has already been forgotten
            // above, which is the most that can be done from here.
            spdlog::warn("A pinned mesh ({}) was evicted", mesh_id);
            break;
    }
}

void Editor::upload_node_to_gpu(osm::QuadTreeNode& node, GPURenderer& renderer) {
    if (node.gpu_uploaded) return;

    // A leaf can reach here still holding handles: eviction takes one of its
    // meshes and clears gpu_uploaded, leaving the others live. Clearing the id
    // vectors without releasing them -- which is what this used to do -- would
    // strand that geometry on the GPU for the rest of the session.
    release_node_from_gpu(node, renderer);

    MeshOwner owner;
    owner.kind = MeshOwner::Kind::QuadTreeLeaf;
    owner.node = &node;
    owner.anchor = node_anchor(node);

    size_t failed = 0;
    size_t uploaded = 0;
    const auto upload_all = [&](const std::vector<Mesh>& meshes, std::vector<uint32_t>& ids) {
        ids.reserve(meshes.size());
        for (const Mesh& mesh : meshes) {
            if (!mesh.is_valid()) {
                continue;  // nothing to draw, and not a failure either
            }
            const uint32_t id = upload_tracked_mesh(renderer, mesh, owner);
            if (id == 0) {
                ++failed;
                continue;
            }
            ids.push_back(id);
            ++uploaded;
        }
    };

    upload_all(node.area_meshes, node.area_gpu_ids);
    // Empty when the leaf carries a chunk LOD chain: the chain replaced this mesh
    // and sync_node_road_lod() uploads exactly one level of it, per frame, by
    // distance. Roads are therefore NOT part of the completeness test below --
    // they are not uploaded here and their absence is not a failure.
    upload_all(node.road_meshes, node.road_gpu_ids);
    upload_all(node.building_meshes, node.building_gpu_ids);

    // Only a leaf that uploaded IN FULL is uploaded.
    //
    // This used to push the 0 that upload_mesh() returns on FAILURE straight into
    // the id vector and set gpu_uploaded = true regardless, so a leaf that lost an
    // upload -- to a budget refusal, a pool refusal, or a device OOM -- was never
    // retried. draw_mesh() then discarded the 0 silently every frame and the leaf
    // rendered nothing for the rest of the session, with no error anywhere.
    //
    // The second half of the test covers eviction landing DURING this upload: an
    // upload under pressure evicts to make room, and the victim it picks can be a
    // mesh this very leaf uploaded a moment ago, which on_mesh_evicted() then
    // erases from the vectors below. Counting what is still held against what was
    // uploaded catches that without any extra state, and the leaf is retried
    // whole rather than latching as complete while missing a mesh.
    const size_t held = node.area_gpu_ids.size() + node.road_gpu_ids.size()
                      + node.building_gpu_ids.size();
    node.gpu_uploaded = (failed == 0) && (held == uploaded);

    if (failed > 0) {
        // Retried on every frame the leaf stays visible, so the warning is rate
        // limited rather than the retry: a failure that persists must not turn
        // into a 60 Hz log write.
        const uint64_t now = SDL_GetTicks();
        if (now >= m_next_upload_warn_ms) {
            m_next_upload_warn_ms = now + 2000;
            spdlog::warn("{} mesh(es) of quadtree leaf {} failed to upload; retrying while it "
                         "stays visible ({} renderer upload failures so far, {} MB resident)",
                         failed, node.node_id, renderer.upload_failures(),
                         renderer.resident_bytes() / (1024 * 1024));
        }
    }
}

void Editor::release_node_from_gpu(osm::QuadTreeNode& node, GPURenderer& renderer) {
    // Deliberately NOT guarded on gpu_uploaded. A leaf that lost one mesh to
    // eviction has the flag cleared while still holding the others, and a guard
    // here would leave exactly those behind -- which is the leak this function is
    // called to prevent, on every path that destroys the tree.
    const auto release_all = [&](std::vector<uint32_t>& ids) {
        for (uint32_t& id : ids) {
            release_tracked_mesh(renderer, id);
        }
        ids.clear();
    };

    release_all(node.area_gpu_ids);
    release_all(node.road_gpu_ids);
    release_all(node.building_gpu_ids);

    // The chain is still on the CPU, but nothing of it is on the device any
    // more. Leaving the level set would make sync_node_road_lod() believe the
    // right geometry was already resident and skip the re-upload.
    node.road_lod_resident = -1;
    node.gpu_uploaded = false;
}

void Editor::sync_node_road_lod(osm::QuadTreeNode& node, GPURenderer& renderer,
                                float distance) {
    if (!node.has_road_lod()) return;

    const int levels = static_cast<int>(node.road_lod.levels.size());

    // A forced level is clamped per chunk. Chains are not all the same length --
    // a chunk of seven pieces gives up after one level where a dense one gets
    // four -- so an override of 3 has to mean "the coarsest you have" rather than
    // "draw nothing".
    const int desired = (m_road_lod_override >= 0)
                      ? std::min(m_road_lod_override, levels - 1)
                      : osm::select_road_lod_level(node.road_lod, distance,
                                                   node.road_lod_resident,
                                                   m_road_lod_distance_scale);

    if (desired == node.road_lod_resident && !node.road_gpu_ids.empty()) {
        return;
    }

    ++m_road_lod_frame_build.swaps;

    // Release first, upload second. The other order would hold two levels of the
    // same chunk resident at once, and under a tight budget that is what makes an
    // upload evict some other leaf to make room for geometry about to be freed.
    for (uint32_t& id : node.road_gpu_ids) {
        release_tracked_mesh(renderer, id);
    }
    node.road_gpu_ids.clear();
    node.road_lod_resident = -1;

    const Mesh& mesh = node.road_lod.levels[static_cast<size_t>(desired)];
    if (!mesh.is_valid()) {
        // A level that simplified down to nothing is not a failure and must not
        // latch: leaving the level unset means the next frame tries again, which
        // is wrong. Record it as resident with no handle instead.
        node.road_lod_resident = desired;
        return;
    }

    MeshOwner owner;
    owner.kind = MeshOwner::Kind::QuadTreeLeaf;
    owner.node = &node;
    owner.anchor = node_anchor(node);

    const uint32_t id = upload_tracked_mesh(renderer, mesh, owner);
    if (id == 0) {
        return;  // retried on the next frame the leaf stays visible
    }

    // The upload may have evicted to make room, and the victim it picked can be
    // the mesh it just uploaded. m_mesh_owners is the record of what survived, so
    // pushing a handle that is no longer in it would leave the leaf drawing a
    // freed buffer.
    if (m_mesh_owners.find(id) == m_mesh_owners.end()) {
        return;
    }

    node.road_gpu_ids.push_back(id);
    node.road_lod_resident = desired;
}

void Editor::record_road_lod_residency(const osm::QuadTreeNode& node) {
    if (!node.has_road_lod()) {
        // A leaf with no chain but with road geometry is the chunk-LOD-off path,
        // not an empty leaf, and the panel has to be able to tell the two apart.
        if (!node.road_meshes.empty()) {
            ++m_road_lod_frame_build.leaves_no_chain;
        }
        return;
    }

    ++m_road_lod_frame_build.leaves_with_chain;

    const int level = node.road_lod_resident;
    if (level < 0 || level >= static_cast<int>(node.road_lod.levels.size())) {
        return;  // nothing resident: the upload was refused or the leaf was evicted
    }

    auto& per_level = m_road_lod_frame_build.leaves_per_level;
    const size_t idx = static_cast<size_t>(level);
    if (per_level.size() <= idx) per_level.resize(idx + 1, 0);
    ++per_level[idx];

    const Mesh& mesh = node.road_lod.levels[idx];
    m_road_lod_frame_build.resident_triangles += mesh.indices.size() / 3;
    m_road_lod_frame_build.resident_vertices += mesh.vertices.size();
}

void Editor::render_3d(GPURenderer& renderer) {
    // s_viewport_rect is only written while the Viewport panel is drawing. This
    // function is now called unconditionally from Application::render(), so bail
    // out if the panel is closed/collapsed - a zero-size viewport or scissor is a
    // Vulkan validation error.
    if (s_viewport_rect.z < 1.0f || s_viewport_rect.w < 1.0f) {
        return;
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

    // Gathered by the visitor below and published when the traversal returns, so
    // the panel never reads a partially counted frame.
    m_road_lod_frame_build.reset();

    // Use quadtree traversal for GPU rendering (front-to-back sorted)
    m_quadtree.traverse_visible(
        frustum.planes,
        cam_pos,
        m_view_radius,
        viewport.h,
        m_camera.m_fov,
        m_contribution_threshold,
        m_use_tile_culling,
        m_use_distance_culling,
        m_use_contribution_culling,
        [&](osm::QuadTreeNode* node, float dist_sq) {
            // Stream: a node that just became visible gets its mesh build queued
            // here. This traversal is the only one per frame, so it has to do the
            // queueing that rebuild_visible_batches() used to.
            if (!node->meshes_built) {
                if (!node->meshes_pending) {
                    m_quadtree.queue_node_build_async(node);
                }
                return;
            }

            // Upload to GPU if needed
            if (!node->gpu_uploaded) {
                upload_node_to_gpu(*node, renderer);
            }

            // Roads go through their own path when the leaf carries a chunk LOD
            // chain, because which level belongs on the device depends on where
            // the camera is and the rest of the leaf does not. dist_sq is the
            // squared XZ distance the traversal already computed for its
            // front-to-back sort, which is the same measure ChunkLod's switch
            // distances are expressed in.
            sync_node_road_lod(*node, renderer, std::sqrt(dist_sq));
            record_road_lod_residency(*node);

            // The trailing MaterialKey is the DEFAULT for geometry that carries no
            // material tag of its own -- these builders predate MaterialId and emit
            // no submeshes at all. Road meshes from the new road network DO carry
            // tagged ranges, and those always win over the default; passing Asphalt
            // here only affects the legacy flat road ribbons.
            if (m_render_areas) {
                for (uint32_t id : node->area_gpu_ids)
                    renderer.draw_mesh(id, model, glm::vec4(1.0f), MaterialKey{MaterialId::Grass, 0});
            }
            if (m_render_roads) {
                for (uint32_t id : node->road_gpu_ids)
                    renderer.draw_mesh(id, model, glm::vec4(1.0f), MaterialKey{MaterialId::Asphalt, 0});
            }
            if (m_render_buildings) {
                // Buildings are not a road surface. Concrete is the closest of the
                // eleven slots and is at least not the untagged grey.
                for (uint32_t id : node->building_gpu_ids)
                    renderer.draw_mesh(id, model, glm::vec4(1.0f), MaterialKey{MaterialId::Concrete, 0});
            }
        }
    );

    m_road_lod_frame = m_road_lod_frame_build;

    // Render procedural terrain
    float radius_sq = m_view_radius * m_view_radius;
    if (m_use_chunked_terrain) {
        // Render chunked terrain
        if (m_render_terrain) {
            for (const auto& coord : m_terrain_tile_manager.get_all_chunks()) {
                auto* chunk = const_cast<procgen::TerrainChunk*>(m_terrain_tile_manager.get_chunk(coord));
                if (!chunk || !chunk->mesh_built) continue;

                // Cull BEFORE uploading. The upload used to run for every chunk in
                // the manager whatever the camera could see, which was merely
                // wasteful when nothing was ever unloaded; with an eviction budget
                // it is a thrash loop, because a chunk evicted for being far away
                // would re-upload on the very next frame.
                if (m_use_tile_culling && !frustum.intersects_aabb(chunk->bounds_min, chunk->bounds_max)) {
                    continue;
                }

                // Distance culling
                if (m_use_distance_culling) {
                    glm::vec3 chunk_center = (chunk->bounds_min + chunk->bounds_max) * 0.5f;
                    float dist_sq = glm::dot(chunk_center - cam_pos, chunk_center - cam_pos);
                    if (dist_sq > radius_sq) continue;
                }

                // Upload to GPU if needed
                if (!chunk->gpu_uploaded && m_gpu_renderer) {
                    // A cleared gpu_uploaded on a chunk that still carries handles
                    // means either its mesh was REBUILT -- by a road carve install,
                    // or by a terrain settings change -- or one of its two meshes
                    // was evicted. Overwriting the handles without releasing them
                    // first strands the old ranges for the rest of the session, and
                    // a carve regenerates every chunk at once.
                    release_tracked_mesh(*m_gpu_renderer, chunk->terrain_gpu_id);
                    release_tracked_mesh(*m_gpu_renderer, chunk->water_gpu_id);

                    MeshOwner owner;
                    owner.kind = MeshOwner::Kind::TerrainChunk;
                    owner.coord = coord;
                    owner.anchor = (chunk->bounds_min + chunk->bounds_max) * 0.5f;

                    if (chunk->terrain_mesh.is_valid()) {
                        chunk->terrain_gpu_id =
                            upload_tracked_mesh(*m_gpu_renderer, chunk->terrain_mesh, owner);
                    }
                    if (chunk->water_mesh.is_valid()) {
                        chunk->water_gpu_id =
                            upload_tracked_mesh(*m_gpu_renderer, chunk->water_mesh, owner);
                    }

                    // Same rule as a quadtree leaf: a partial upload is not an
                    // upload and must be retried rather than latched. Read back
                    // from the handles AFTER both uploads rather than from each
                    // return value, because the second upload can evict the first
                    // one to make room -- on_mesh_evicted() then zeroes a handle
                    // that was good when it was returned.
                    chunk->gpu_uploaded =
                        (!chunk->terrain_mesh.is_valid() || chunk->terrain_gpu_id != 0) &&
                        (!chunk->water_mesh.is_valid() || chunk->water_gpu_id != 0);
                }
                
                // Draw terrain
                if (chunk->terrain_gpu_id != 0) {
                    renderer.draw_mesh(chunk->terrain_gpu_id, model, glm::vec4(1.0f),
                                       MaterialKey{MaterialId::Grass, 0});
                }

                // Draw water. No water slot exists in MaterialId -- it is a road
                // material set -- so water keeps the untagged default rather than
                // borrowing a surface that would make it look like wet tarmac.
                if (m_render_water && chunk->water_gpu_id != 0) {
                    renderer.draw_mesh(chunk->water_gpu_id, model, glm::vec4(1.0f));
                }
            }
        }
    } else {
        // Legacy single terrain rendering
        if (m_render_terrain && m_terrain_gpu_id != 0) {
            renderer.draw_mesh(m_terrain_gpu_id, model, glm::vec4(1.0f),
                               MaterialKey{MaterialId::Grass, 0});
        }
        if (m_render_water && m_water_gpu_id != 0) {
            renderer.draw_mesh(m_water_gpu_id, model, glm::vec4(1.0f));
        }
    }

    // Im3d debug geometry last, so it depth-tests against the opaque scene above.
    // Inherits the viewport and scissor already set at the top of this function.
    Im3D_Render(renderer, m_camera.get_view_projection(), viewport.w, viewport.h);
}

} // namespace stratum
