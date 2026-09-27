// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include "renderer/gpu_renderer.hpp"
#include "renderer/texture.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cstdio>

namespace stratum {

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
        spdlog::info("[GPU] Evicted {} mesh(es) to budget", evicted);
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

    // Export road network controls used to be duplicated here; they now live
    // solely in the OSM import panel (draw_osm_panel(), export_options.hpp),
    // still wired to the same begin_road_export().

    ImGui::End();
}

} // namespace stratum
