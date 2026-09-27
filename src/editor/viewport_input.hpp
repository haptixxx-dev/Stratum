// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#pragma once

namespace stratum {

/**
 * @brief Per-frame viewport input, sampled from ImGui by the Viewport panel.
 *
 * Deliberately ImGui-free (no imgui.h include, no ImVec2/ImVec4 members). This
 * is the boundary that lets Camera::handle_input() and the Im3d focus/size
 * handoff consume viewport input without including ImGui themselves.
 * src/editor/panels/viewport_panel.cpp is the only place that fills one of
 * these, once per frame, and stores it in an Editor member; everything
 * downstream (camera.cpp, im3d_impl.cpp) just reads it.
 */
struct ViewportInput {
    /// Viewport content-region size, in ImGui logical points.
    float width = 0.0f;
    float height = 0.0f;

    /// Same meaning as ImGui::IsWindowFocused() / IsWindowHovered() for the
    /// Viewport panel this frame.
    bool focused = false;
    bool hovered = false;

    /// Mouse motion since last frame (ImGui's io.MouseDelta), in pixels.
    float mouse_dx = 0.0f;
    float mouse_dy = 0.0f;

    /// Scroll wheel delta this frame (ImGui's io.MouseWheel).
    float wheel = 0.0f;

    /// Frame delta time in seconds.
    float dt = 0.0f;
};

} // namespace stratum
