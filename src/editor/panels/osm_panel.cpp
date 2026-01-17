#include "editor/editor.hpp"
#include <imgui.h>
#include "ImGuiFileDialog/ImGuiFileDialog.h"

namespace stratum {

void Editor::draw_osm_panel() {
    if (!m_show_osm_panel) return;

    if (ImGui::Begin("OSM Data", &m_show_osm_panel)) {
        
        if (ImGui::Button("Import OSM File...")) {
            IGFD::FileDialogConfig config;
            config.path = ".";
            ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose OSM File", ".osm,.pbf,.osm.bz2,.osm.gz", config);
        }

        // Display dialog
        if (ImGuiFileDialog::Instance()->Display("ChooseFileDlgKey")) {
            if (ImGuiFileDialog::Instance()->IsOk()) {
                std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
                m_osm_import_path = filePathName;
                
                // Trigger parse
                try {
                    bool success = m_osm_parser.parse(filePathName);
                    if (!success) {
                        // TODO: Log error to console
                    }
                } catch (const std::exception& e) {
                    // TODO: Log exception
                }
            }
            ImGuiFileDialog::Instance()->Close();
        }

        ImGui::Separator();

        // Render toggles - always visible
        ImGui::Checkbox("Areas", &m_render_areas);
        ImGui::SameLine();
        ImGui::Checkbox("Roads", &m_render_roads);
        ImGui::SameLine();
        ImGui::Checkbox("Buildings", &m_render_buildings);

        ImGui::Separator();

        // Show Stats
        const auto& data = m_osm_parser.get_data();

        if (data.stats.total_nodes > 0 || data.stats.total_ways > 0) {
            ImGui::Text("File: %s", m_osm_import_path.c_str());
            ImGui::Spacing();

            if (ImGui::CollapsingHeader("Statistics", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::BeginTable("osm_stats", 2)) {
                    ImGui::TableNextColumn(); ImGui::Text("Nodes (Raw)");
                    ImGui::TableNextColumn(); ImGui::Text("%zu", data.stats.total_nodes);
                    
                    ImGui::TableNextColumn(); ImGui::Text("Ways (Raw)");
                    ImGui::TableNextColumn(); ImGui::Text("%zu", data.stats.total_ways);
                    
                    ImGui::TableNextColumn(); ImGui::Text("Relations (Raw)");
                    ImGui::TableNextColumn(); ImGui::Text("%zu", data.stats.total_relations);
                    
                    ImGui::TableNextColumn(); ImGui::Text("Roads (Processed)");
                    ImGui::TableNextColumn(); ImGui::Text("%zu", data.stats.processed_roads);
                    
                    ImGui::TableNextColumn(); ImGui::Text("Buildings (Processed)");
                    ImGui::TableNextColumn(); ImGui::Text("%zu", data.stats.processed_buildings);
                    
                    ImGui::TableNextColumn(); ImGui::Text("Areas (Processed)");
                    ImGui::TableNextColumn(); ImGui::Text("%zu", data.stats.processed_areas);
                    
                    ImGui::EndTable();
                }
                
                ImGui::Separator();
                ImGui::Text("Timing:");
                ImGui::BulletText("Parse: %.2f ms", data.stats.parse_time_ms);
                ImGui::BulletText("Process: %.2f ms", data.stats.process_time_ms);
            }
            
            if (ImGui::CollapsingHeader("Coordinate System")) {
                ImGui::Text("Origin (Lat/Lon): %.6f, %.6f", 
                    data.coord_system.origin_latlon.x, 
                    data.coord_system.origin_latlon.y);
                ImGui::Text("Bounds Width: %.2f m", data.bounds.width_meters());
                ImGui::Text("Bounds Height: %.2f m", data.bounds.height_meters());
            }
        } else {
            ImGui::TextDisabled("No OSM data loaded.");
        }
    }
    ImGui::End();
}

} // namespace stratum
