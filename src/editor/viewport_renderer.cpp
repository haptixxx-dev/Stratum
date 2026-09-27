// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include "editor/im3d_impl.hpp"
#include "renderer/gpu_renderer.hpp"
#include <imgui.h>
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>

namespace stratum {

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
