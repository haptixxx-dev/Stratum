// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

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
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>

namespace stratum {

// Defined here, not in the header: the unique_ptr members hold types that are
// only forward-declared there. See the note on ~Editor().
Editor::Editor() = default;
Editor::~Editor() = default;

void Editor::init() {
    spdlog::info("Editor initialized");
    Im3D_Init();

    // Mirror every logged record into the console panel's ring. Registered on
    // the default logger (not a dedicated one) so the console shows exactly
    // what every subsystem already logs -- nothing needs to know the editor
    // exists to show up there.
    m_log_sink = std::make_shared<RingSinkMt>(m_log_ring);
    spdlog::default_logger()->sinks().push_back(m_log_sink);

    // Computed once here rather than per save/load: SDL_GetPrefPath allocates,
    // and the path does not change during a run. "Haptixxx" is a placeholder
    // organisation string -- flagged in the PR description as Sarah's call, not
    // fixed anywhere else in the tree.
    if (char* pref = SDL_GetPrefPath("Haptixxx", "Stratum")) {
        m_render_settings_path = std::filesystem::path(pref) / "render_settings.json";
        SDL_free(pref);
    } else {
        spdlog::warn("SDL_GetPrefPath failed ({}); render settings will not persist",
                     SDL_GetError());
    }

    if (m_model.m_render_settings.load(m_render_settings_path)) {
        spdlog::info("Loaded render settings from '{}'", m_render_settings_path.string());
    }
    // No else: RenderSettings::load() leaves m_model.m_render_settings at its compiled-in
    // defaults on any failure, which is exactly what a first run should see.
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

        // Apply the loaded-or-default render settings now, before the render loop
        // starts. This is what used to happen implicitly, one frame late and only
        // if the panel was open, via the sun_pushed/sky_pushed/fog_pushed statics
        // in draw_render_settings(); see RenderSettings::push_to() for why the
        // renderer also needs telling that this already happened.
        m_model.m_render_settings.push_to(*renderer);
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

void Editor::publish_camera(GPURenderer& renderer) {
    renderer.set_view_projection(m_camera.get_view(), m_camera.get_projection());
    renderer.set_camera_position(m_camera.get_position());
}

void Editor::im3d_end_frame_and_upload(GPURenderer& renderer) {
    Im3D_EndFrameAndUpload(renderer);
}

void Editor::shutdown() {
    Im3D_Shutdown();

    // Unregister before the ring it writes into goes away. Application logs a
    // few more lines after this call returns (ImGui/SDL teardown); those are
    // fine to lose from the console since the panel is gone with the editor.
    if (m_log_sink) {
        auto logger = spdlog::default_logger();
        auto& sinks = logger->sinks();
        sinks.erase(std::remove(sinks.begin(), sinks.end(), m_log_sink), sinks.end());
        m_log_sink.reset();
    }

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
    m_viewport_rect = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);

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
    if (m_show_rule_panel) draw_rule_panel();
}

} // namespace stratum
