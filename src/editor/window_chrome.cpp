// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include <imgui.h>
#include <SDL3/SDL.h>
#include <spdlog/spdlog.h>
#include <algorithm>

namespace stratum {

void Editor::handle_window_drag() {
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

} // namespace stratum
