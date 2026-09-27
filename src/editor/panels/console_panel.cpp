// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include "editor/log_sink.hpp"
#include <imgui.h>

namespace stratum {

namespace {

ImVec4 level_color(LogLine::Level level) {
    switch (level) {
        case LogLine::Warn:
            return ImVec4(1.0f, 0.8f, 0.3f, 1.0f);
        case LogLine::Error:
            return ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
        case LogLine::Info:
        default:
            return ImVec4(0.4f, 0.8f, 0.4f, 1.0f);
    }
}

} // namespace

void Editor::draw_console() {
    ImGui::Begin("Console");

    // Options
    if (ImGui::BeginPopup("Options")) {
        ImGui::Checkbox("Auto-scroll", &m_console_scroll_to_bottom);
        ImGui::EndPopup();
    }

    // Buttons
    if (ImGui::Button("Clear")) {
        m_log_ring.clear();
    }
    ImGui::SameLine();
    if (ImGui::Button("Options")) {
        ImGui::OpenPopup("Options");
    }

    ImGui::Separator();

    // Log content
    ImGui::BeginChild("ScrollingRegion", ImVec2(0, 0), ImGuiChildFlags_None,
                      ImGuiWindowFlags_HorizontalScrollbar);

    m_log_ring.for_each([](const LogLine& line) {
        ImGui::TextDisabled("%s", line.time.c_str());
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Text, level_color(line.level));
        ImGui::TextUnformatted(line.text.c_str());
        ImGui::PopStyleColor();
    });

    if (m_console_scroll_to_bottom) {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();
    ImGui::End();
}

} // namespace stratum
