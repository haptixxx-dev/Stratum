// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include <imgui.h>

namespace stratum {

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

} // namespace stratum
