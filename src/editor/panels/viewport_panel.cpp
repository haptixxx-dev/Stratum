// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include "editor/im3d_impl.hpp"
#include <im3d.h>
#include <imgui.h>
#include <SDL3/SDL.h>
#include <algorithm>

namespace stratum {

void Editor::draw_viewport() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    // NoBackground: the 3D pass has already drawn the scene into the swapchain
    ImGui::Begin("Viewport", nullptr, ImGuiWindowFlags_NoBackground);

    m_viewport_focused = ImGui::IsWindowFocused();
    m_viewport_hovered = ImGui::IsWindowHovered();

    ImVec2 viewport_size = ImGui::GetContentRegionAvail();
    ImVec2 pos = ImGui::GetCursorScreenPos();
    
    // Save rect for callback
    m_viewport_rect = ImVec4(pos.x, pos.y, viewport_size.x, viewport_size.y);

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

    // Colour-by-attribute outlines. After the node grid so that with both on the
    // attribute colours win the overlap, and inside the same Im3d frame as
    // everything else here -- Im3D_Render() at the end of render_3d() draws the
    // lot in one pass.
    draw_attribute_overlay();




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

    // Toolbar overlay, stacked under the label above.
    //
    // Both are placed in SCREEN space off `pos`. The label is drawn through the
    // draw list, which only takes screen coordinates, so positioning the toolbar
    // with SetCursorPos() -- which is window-LOCAL -- measured the two from
    // different origins, and the gap between them silently became "whatever the
    // tab bar height happens to be". At a small enough font the label landed on
    // top of the buttons. Measuring both from `pos` and spacing them by the
    // actual text line height keeps them apart at any font size or dock state.
    ImGui::SetCursorScreenPos(ImVec2(pos.x + 10,
                                     pos.y + 10 + ImGui::GetTextLineHeight() +
                                         ImGui::GetStyle().ItemSpacing.y));
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

void Editor::draw_attribute_mode_selector() {
    using osm::AttributeMode;

    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("Colour By", osm::attribute_mode_name(m_attribute_mode))) {
        for (size_t i = 0; i < osm::kAttributeModeCount; ++i) {
            const auto mode = static_cast<AttributeMode>(i);
            const bool selected = (mode == m_attribute_mode);
            if (ImGui::Selectable(osm::attribute_mode_name(mode), selected)) {
                m_attribute_mode = mode;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SetItemTooltip(
        "Paint the scene by classification instead of by material.\n"
        "Magenta always means Unknown: nothing classified that feature.\n"
        "Tile / Chunk recolours the solid surfaces; the other modes\n"
        "draw outlines, because a leaf's buildings share one merged mesh.");

    if (m_attribute_mode == AttributeMode::None) {
        return;
    }

    ImGui::SameLine();
    ImGui::Checkbox("Legend", &m_attribute_show_legend);

    // Say plainly when the budget clipped the overlay. Silence here would read as
    // "the rest of the city has no buildings".
    if (m_attribute_features_drawn < m_attribute_features_seen) {
        ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.0f, 1.0f), "Showing %zu of %zu (budget)",
                           m_attribute_features_drawn, m_attribute_features_seen);
    } else {
        ImGui::Text("Showing %zu", m_attribute_features_drawn);
    }

    if (!m_attribute_show_legend) {
        return;
    }

    const auto swatch = [](const char* label, const glm::vec4& colour) {
        ImGui::ColorButton(label, ImVec4(colour.r, colour.g, colour.b, colour.a),
                           ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop,
                           ImVec2(14.0f, 14.0f));
        ImGui::SameLine();
        ImGui::TextUnformatted(label);
    };

    switch (m_attribute_mode) {
        case AttributeMode::Building:
            for (size_t i = 0; i < osm::kBuildingTypeCount; ++i) {
                const auto type = static_cast<osm::BuildingType>(i);
                swatch(osm::building_type_name(type), osm::building_type_colour(type));
            }
            break;

        case AttributeMode::Road:
            for (size_t i = 0; i < osm::kRoadTypeCount; ++i) {
                const auto type = static_cast<osm::RoadType>(i);
                swatch(osm::road_type_name(type), osm::road_type_colour(type));
            }
            break;

        case AttributeMode::Area:
            for (size_t i = 0; i < osm::kAreaTypeCount; ++i) {
                const auto type = static_cast<osm::AreaType>(i);
                swatch(osm::area_type_name(type), osm::area_type_colour(type));
            }
            break;

        case AttributeMode::Tile:
            // No names to legend: the colours mean "a different tile from the one
            // next to it" and nothing else. Showing the cycle is still worth it,
            // because it tells the eye which twelve colours to read as boundaries.
            for (size_t i = 0; i < osm::kTileColourCount; ++i) {
                const glm::vec4 colour = osm::tile_colour(i);
                // PushID per swatch: every one of the twelve carries the same
                // label, and ImGui derives a widget's identity from its label.
                ImGui::PushID(static_cast<int>(i));
                ImGui::ColorButton("##tile_swatch", ImVec4(colour.r, colour.g, colour.b, colour.a),
                                   ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop,
                                   ImVec2(14.0f, 14.0f));
                ImGui::PopID();
                if (i + 1 < osm::kTileColourCount) ImGui::SameLine();
            }
            break;

        case AttributeMode::None:
        case AttributeMode::Count:
            break;
    }
}

} // namespace stratum
