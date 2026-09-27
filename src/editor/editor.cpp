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

void Editor::publish_camera(GPURenderer& renderer) {
    renderer.set_view_projection(m_camera.get_view(), m_camera.get_projection());
    renderer.set_camera_position(m_camera.get_position());
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
    // m_viewport_rect is only written while the Viewport panel is drawing. This
    // function is now called unconditionally from Application::render(), so bail
    // out if the panel is closed/collapsed - a zero-size viewport or scissor is a
    // Vulkan validation error.
    if (m_viewport_rect.z < 1.0f || m_viewport_rect.w < 1.0f) {
        return;
    }

    // m_viewport_rect comes from ImGui, which lays out in LOGICAL POINTS.
    // SDL_GPUViewport and SDL_SetGPUScissor address the swapchain, which is sized
    // in PIXELS (gpu_renderer.cpp, SDL_GetWindowSizeInPixels). The two coincide
    // only where the pixel density is 1: on a Retina Mac it is 2, and passing
    // points through unconverted scissors the 3D pass to a rect half as wide and
    // half as tall as the panel, anchored at half its offset. Most of that rect
    // then falls under the left dock panel and the tab bar, which the ImGui pass
    // paints over afterwards -- so the scene appears squeezed into the top-left
    // corner of the viewport. Linux never showed it because density is 1 there.
    //
    // DisplayFramebufferScale is the same factor the ImGui backend uses to map
    // its draw data onto the swapchain (imgui_impl_sdlgpu3.cpp: fb_width =
    // DisplaySize * FramebufferScale), so scaling by it keeps the 3D pass aligned
    // with the panel ImGui draws around it, whatever the backend reports.
    const ImGuiIO& io = ImGui::GetIO();
    const float scale_x = io.DisplayFramebufferScale.x > 0.0f ? io.DisplayFramebufferScale.x : 1.0f;
    const float scale_y = io.DisplayFramebufferScale.y > 0.0f ? io.DisplayFramebufferScale.y : 1.0f;

    const float px_x = m_viewport_rect.x * scale_x;
    const float px_y = m_viewport_rect.y * scale_y;
    const float px_w = m_viewport_rect.z * scale_x;
    const float px_h = m_viewport_rect.w * scale_y;

    // Set Viewport
    SDL_GPUViewport viewport;
    viewport.x = px_x;
    viewport.y = px_y;
    viewport.w = px_w;
    viewport.h = px_h;
    viewport.min_depth = 0.0f;
    viewport.max_depth = 1.0f;
    renderer.set_viewport(viewport);

    // The scissor must stay inside the render target -- a rect that pokes past
    // the edge is a Vulkan validation error, and the panel can hang off the
    // window edge mid-resize drag. The viewport above needs no such clamp: it
    // only defines the clip-space mapping, and the scissor is what clips.
    const int fb_w = static_cast<int>(renderer.get_swapchain_width());
    const int fb_h = static_cast<int>(renderer.get_swapchain_height());
    const int x0 = std::clamp(static_cast<int>(px_x), 0, fb_w);
    const int y0 = std::clamp(static_cast<int>(px_y), 0, fb_h);
    const int x1 = std::clamp(static_cast<int>(px_x + px_w), x0, fb_w);
    const int y1 = std::clamp(static_cast<int>(px_y + px_h), y0, fb_h);
    if (x1 <= x0 || y1 <= y0) {
        return;  // scrolled fully off-screen: nothing to draw, and a zero-area
                 // scissor is a validation error
    }

    SDL_Rect scissor;
    scissor.x = x0;
    scissor.y = y0;
    scissor.w = x1 - x0;
    scissor.h = y1 - y0;
    if (renderer.get_render_pass()) {
        SDL_SetGPUScissor(renderer.get_render_pass(), &scissor);
    }

    // Camera state FIRST. bind_mesh_pipeline() pushes SceneUniforms -- which
    // carries camera_position, and the PBR shader derives the view vector, the
    // double-sided normal flip and the fog distance from it -- so setting the
    // camera after the bind published last frame's position to this frame's
    // draws.
    //
    // Application::render() has already called this before the shadow cascades,
    // which need the same matrices earlier still. Repeating it is two assignments
    // and keeps this function correct on its own.
    publish_camera(renderer);
    glm::vec3 cam_pos = m_camera.get_position();

    // Sky BEFORE geometry and after the camera state above: it reconstructs a
    // world ray per pixel from the inverse view-projection, and it neither tests
    // nor writes depth, so everything drawn afterwards simply covers it.
    renderer.draw_sky();

    renderer.bind_mesh_pipeline();

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
            //
            // The tint is white in every mode but Tile, so this is a no-op
            // multiply unless the user asked for it. Tile is the only attribute
            // whose value is constant across a whole leaf, which is the only shape
            // a per-draw tint can express -- the classification modes cannot use
            // this path at all, because every building in the leaf shares one
            // merged mesh and one draw call. They go through draw_attribute_overlay().
            const glm::vec4 tint = attribute_leaf_tint(*node);

            if (m_render_areas) {
                for (uint32_t id : node->area_gpu_ids)
                    renderer.draw_mesh(id, model, tint, MaterialKey{MaterialId::Grass, 0});
            }
            if (m_render_roads) {
                for (uint32_t id : node->road_gpu_ids)
                    renderer.draw_mesh(id, model, tint, MaterialKey{MaterialId::Asphalt, 0});
            }
            if (m_render_buildings) {
                // Buildings are not a road surface. Concrete is the closest of the
                // eleven slots and is at least not the untagged grey.
                for (uint32_t id : node->building_gpu_ids)
                    renderer.draw_mesh(id, model, tint, MaterialKey{MaterialId::Concrete, 0});
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
                
                // Terrain chunks are a second tile system, with their own grid and
                // their own streaming bugs, so the Tile mode covers them too. White
                // in every other mode.
                const glm::vec4 chunk_tint = attribute_chunk_tint(coord);

                // Draw terrain
                if (chunk->terrain_gpu_id != 0) {
                    renderer.draw_mesh(chunk->terrain_gpu_id, model, chunk_tint,
                                       MaterialKey{MaterialId::Grass, 0});
                }

                // Draw water. No water slot exists in MaterialId -- it is a road
                // material set -- so water keeps the untagged default rather than
                // borrowing a surface that would make it look like wet tarmac.
                if (m_render_water && chunk->water_gpu_id != 0) {
                    renderer.draw_mesh(chunk->water_gpu_id, model, chunk_tint);
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

    // The rule editor's preview, drawn with the rest of the opaque scene so it
    // lights and shadows like a building rather than floating as debug geometry.
    // Concrete because a rule that sets no material should still look like a
    // massing model, not like untextured default grey.
    if (m_render_rule_preview && m_rule_preview_gpu_id != 0) {
        renderer.draw_mesh(m_rule_preview_gpu_id, model, glm::vec4(1.0f),
                           MaterialKey{MaterialId::Concrete, 0});
    }

    // Im3d debug geometry last, so it depth-tests against the opaque scene above.
    // Inherits the viewport and scissor already set at the top of this function.
    Im3D_Render(renderer, m_camera.get_view_projection(), viewport.w, viewport.h);
}

} // namespace stratum
