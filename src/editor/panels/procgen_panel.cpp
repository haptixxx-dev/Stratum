#include "editor/editor.hpp"
#include "renderer/gpu_renderer.hpp"
#include <imgui.h>

namespace stratum {

void Editor::draw_procgen_panel() {
    if (!m_show_procgen_panel) return;

    if (ImGui::Begin("Procedural Generation", &m_show_procgen_panel)) {
        
        // Mode selection
        ImGui::Checkbox("Use Chunked Terrain", &m_use_chunked_terrain);
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Chunked terrain generates tiles that blend with OSM data.\nLegacy mode generates a single terrain mesh.");
        }
        
        ImGui::Separator();
        
        if (m_use_chunked_terrain) {
            draw_chunked_terrain_ui();
        } else {
            draw_legacy_terrain_ui();
        }
    }
    ImGui::End();
}

void Editor::draw_chunked_terrain_ui() {
    auto& config = m_terrain_tile_config;
    
    // Terrain Type Selection
    ImGui::Text("Terrain Type:");
    const char* terrain_types[] = { "Flat", "Rolling", "Hilly", "Mountainous" };
    int terrain_type_int = static_cast<int>(config.terrain.type);
    if (ImGui::Combo("##TerrainType", &terrain_type_int, terrain_types, 4)) {
        config.terrain.type = static_cast<procgen::TerrainType>(terrain_type_int);
    }

    ImGui::Separator();

    // World Bounds
    if (ImGui::CollapsingHeader("World Bounds", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragFloat2("Min (m)", &config.world_min.x, 10.0f, -10000.0f, 0.0f, "%.0f");
        ImGui::DragFloat2("Max (m)", &config.world_max.x, 10.0f, 0.0f, 10000.0f, "%.0f");
        ImGui::SliderFloat("Chunk Size", &config.chunk_size, 100.0f, 1000.0f, "%.0f m");
        ImGui::SliderInt("Chunk Resolution", &config.chunk_resolution, 16, 128);
        
        // Auto-fit to OSM bounds button
        if (m_tile_manager.tile_count() > 0) {
            if (ImGui::Button("Fit to OSM Data")) {
                // Find bounds of all OSM tiles
                // OSM tile bounds are in rendering coords: (x, y_height, -z_world)
                glm::vec3 osm_min(std::numeric_limits<float>::max());
                glm::vec3 osm_max(std::numeric_limits<float>::lowest());
                
                for (const auto& coord : m_tile_manager.get_all_tiles()) {
                    const auto* tile = m_tile_manager.get_tile(coord);
                    if (tile && tile->has_valid_bounds()) {
                        osm_min = glm::min(osm_min, tile->bounds_min);
                        osm_max = glm::max(osm_max, tile->bounds_max);
                    }
                }
                
                if (osm_min.x < osm_max.x) {
                    // Convert from rendering coords (x, y, -z) to terrain world coords (x, z)
                    // In rendering: bounds_min.z is -max_world_z, bounds_max.z is -min_world_z
                    // So world_z_min = -bounds_max.z, world_z_max = -bounds_min.z
                    float padding = config.chunk_size;
                    config.world_min = glm::vec2(osm_min.x - padding, -osm_max.z - padding);
                    config.world_max = glm::vec2(osm_max.x + padding, -osm_min.z + padding);
                }
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered()) {
                // Show current OSM bounds for debugging
                glm::vec3 osm_min(std::numeric_limits<float>::max());
                glm::vec3 osm_max(std::numeric_limits<float>::lowest());
                for (const auto& coord : m_tile_manager.get_all_tiles()) {
                    const auto* tile = m_tile_manager.get_tile(coord);
                    if (tile && tile->has_valid_bounds()) {
                        osm_min = glm::min(osm_min, tile->bounds_min);
                        osm_max = glm::max(osm_max, tile->bounds_max);
                    }
                }
                ImGui::SetTooltip("OSM bounds (render coords):\nMin: %.1f, %.1f, %.1f\nMax: %.1f, %.1f, %.1f",
                    osm_min.x, osm_min.y, osm_min.z, osm_max.x, osm_max.y, osm_max.z);
            }
        }
    }

    // Height Parameters
    if (ImGui::CollapsingHeader("Height", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Base Height", &config.terrain.base_height, -100.0f, 100.0f, "%.1f");
        ImGui::SliderFloat("Max Height", &config.terrain.max_height, 10.0f, 500.0f, "%.1f");
        ImGui::SliderFloat("Water Level", &config.terrain.water_level, -50.0f, 50.0f, "%.1f");
    }

    // Noise Parameters
    if (ImGui::CollapsingHeader("Noise Parameters")) {
        int seed_int = static_cast<int>(config.terrain.seed);
        if (ImGui::InputInt("Seed", &seed_int)) {
            config.terrain.seed = static_cast<uint32_t>(seed_int);
        }
        ImGui::SameLine();
        if (ImGui::Button("Random")) {
            config.terrain.seed = static_cast<uint32_t>(rand());
        }
        
        ImGui::SliderFloat("Noise Scale", &config.terrain.noise_scale, 0.0001f, 0.01f, "%.5f");
        ImGui::SliderInt("Octaves", &config.terrain.octaves, 1, 10);
        ImGui::SliderFloat("Lacunarity", &config.terrain.lacunarity, 1.5f, 3.0f, "%.2f");
        ImGui::SliderFloat("Persistence", &config.terrain.persistence, 0.2f, 0.8f, "%.2f");
    }

    // OSM Blending
    if (ImGui::CollapsingHeader("OSM Blending", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Flatten Radius", &config.osm_flatten_radius, 0.0f, 50.0f, "%.1f m");
        ImGui::SetItemTooltip("Radius around roads/buildings to flatten");
        
        ImGui::SliderFloat("Blend Distance", &config.osm_blend_distance, 10.0f, 200.0f, "%.1f m");
        ImGui::SetItemTooltip("Distance over which terrain blends from flat to procedural");
        
        ImGui::SliderFloat("OSM Base Height", &config.osm_base_height, -10.0f, 10.0f, "%.1f m");
        ImGui::SetItemTooltip("Height level for flattened OSM areas");
        
        // Show OSM data status
        const auto& osm_data = m_osm_parser.get_data();
        if (osm_data.stats.total_nodes > 0) {
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "OSM data loaded: %zu roads, %zu buildings",
                osm_data.stats.processed_roads, osm_data.stats.processed_buildings);
        } else {
            ImGui::TextDisabled("No OSM data loaded");
        }
    }

    // Mesh coloring options
    if (ImGui::CollapsingHeader("Coloring")) {
        const char* color_modes[] = { "Solid", "Height Gradient", "Slope Gradient", "Biome" };
        int color_mode_int = static_cast<int>(config.mesh.color_mode);
        if (ImGui::Combo("Color Mode", &color_mode_int, color_modes, 4)) {
            config.mesh.color_mode = static_cast<procgen::TerrainColorMode>(color_mode_int);
        }
        
        if (config.mesh.color_mode == procgen::TerrainColorMode::Solid) {
            ImGui::ColorEdit4("Solid Color", &config.mesh.solid_color.x);
        }
        
        ImGui::Checkbox("Generate Water", &config.mesh.generate_water_mesh);
        if (config.mesh.generate_water_mesh) {
            ImGui::ColorEdit4("Water Color", &config.mesh.water_color.x);
        }
    }

    ImGui::Separator();

    // Generate button
    if (ImGui::Button("Generate Chunked Terrain", ImVec2(-1, 40))) {
        generate_chunked_terrain();
    }

    // Clear button
    if (m_terrain_tile_manager.chunk_count() > 0) {
        if (ImGui::Button("Clear All Chunks", ImVec2(-1, 0))) {
            clear_chunked_terrain();
        }
    }

    ImGui::Separator();

    // Stats
    if (m_terrain_tile_manager.chunk_count() > 0) {
        ImGui::Text("Chunks: %zu generated, %zu with meshes", 
            m_terrain_tile_manager.generated_count(),
            m_terrain_tile_manager.mesh_count());
        
        // Count GPU uploaded chunks
        size_t gpu_count = 0;
        for (const auto& coord : m_terrain_tile_manager.get_all_chunks()) {
            const auto* chunk = m_terrain_tile_manager.get_chunk(coord);
            if (chunk && chunk->gpu_uploaded) ++gpu_count;
        }
        ImGui::Text("GPU uploaded: %zu", gpu_count);

        ImGui::Separator();

        // Navigation
        if (ImGui::Button("Go to Terrain Center")) {
            glm::vec2 center = (config.world_min + config.world_max) * 0.5f;
            glm::vec3 cam_target(center.x, 0.0f, -center.y);
            glm::vec3 cam_pos = cam_target + glm::vec3(0.0f, config.terrain.max_height * 3.0f, 
                (config.world_max.y - config.world_min.y) * 0.3f);
            m_camera.set_position(cam_pos);
            m_camera.set_target(cam_target);
        }
    } else {
        ImGui::TextDisabled("No terrain generated.");
        ImGui::TextDisabled("Click 'Generate Chunked Terrain' to create.");
    }
}

void Editor::draw_legacy_terrain_ui() {
    // Terrain Type Selection
    ImGui::Text("Terrain Type:");
    const char* terrain_types[] = { "Flat", "Rolling", "Hilly", "Mountainous" };
    int terrain_type_int = static_cast<int>(m_terrain_config.type);
    if (ImGui::Combo("##TerrainType", &terrain_type_int, terrain_types, 4)) {
        m_terrain_config.type = static_cast<procgen::TerrainType>(terrain_type_int);
    }

    ImGui::Separator();

    // Size and Resolution
    if (ImGui::CollapsingHeader("Size & Resolution", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Width (m)", &m_terrain_config.size_x, 100.0f, 5000.0f, "%.0f");
        ImGui::SliderFloat("Depth (m)", &m_terrain_config.size_z, 100.0f, 5000.0f, "%.0f");
        ImGui::SliderInt("Resolution X", &m_terrain_config.resolution_x, 16, 512);
        ImGui::SliderInt("Resolution Z", &m_terrain_config.resolution_z, 16, 512);
    }

    // Height Parameters
    if (ImGui::CollapsingHeader("Height", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Base Height", &m_terrain_config.base_height, -100.0f, 100.0f, "%.1f");
        ImGui::SliderFloat("Max Height", &m_terrain_config.max_height, 10.0f, 500.0f, "%.1f");
        ImGui::SliderFloat("Water Level", &m_terrain_config.water_level, -50.0f, 50.0f, "%.1f");
    }

    // Noise Parameters
    if (ImGui::CollapsingHeader("Noise Parameters")) {
        int seed_int = static_cast<int>(m_terrain_config.seed);
        if (ImGui::InputInt("Seed", &seed_int)) {
            m_terrain_config.seed = static_cast<uint32_t>(seed_int);
        }
        ImGui::SameLine();
        if (ImGui::Button("Random")) {
            m_terrain_config.seed = static_cast<uint32_t>(rand());
        }
        
        ImGui::SliderFloat("Noise Scale", &m_terrain_config.noise_scale, 0.0001f, 0.01f, "%.5f");
        ImGui::SliderInt("Octaves", &m_terrain_config.octaves, 1, 10);
        ImGui::SliderFloat("Lacunarity", &m_terrain_config.lacunarity, 1.5f, 3.0f, "%.2f");
        ImGui::SliderFloat("Persistence", &m_terrain_config.persistence, 0.2f, 0.8f, "%.2f");
    }

    // City Flattening
    if (ImGui::CollapsingHeader("City Area")) {
        ImGui::Checkbox("Flatten Center", &m_terrain_config.flatten_center);
        if (m_terrain_config.flatten_center) {
            ImGui::SliderFloat("Flat Radius", &m_terrain_config.flatten_radius, 50.0f, 500.0f, "%.0f");
            ImGui::SliderFloat("Falloff", &m_terrain_config.flatten_falloff, 10.0f, 200.0f, "%.0f");
        }
    }

    // Erosion
    if (ImGui::CollapsingHeader("Erosion")) {
        ImGui::Checkbox("Apply Erosion", &m_terrain_config.apply_erosion);
        if (m_terrain_config.apply_erosion) {
            ImGui::SliderInt("Iterations", &m_terrain_config.erosion_iterations, 1, 50);
            ImGui::SliderFloat("Strength", &m_terrain_config.erosion_strength, 0.01f, 0.5f, "%.2f");
        }
    }

    ImGui::Separator();

    // Mesh coloring options
    if (ImGui::CollapsingHeader("Coloring")) {
        const char* color_modes[] = { "Solid", "Height Gradient", "Slope Gradient", "Biome" };
        int color_mode_int = static_cast<int>(m_terrain_mesh_config.color_mode);
        if (ImGui::Combo("Color Mode", &color_mode_int, color_modes, 4)) {
            m_terrain_mesh_config.color_mode = static_cast<procgen::TerrainColorMode>(color_mode_int);
        }
        
        if (m_terrain_mesh_config.color_mode == procgen::TerrainColorMode::Solid) {
            ImGui::ColorEdit4("Solid Color", &m_terrain_mesh_config.solid_color.x);
        }
        
        ImGui::Checkbox("Generate Water", &m_terrain_mesh_config.generate_water_mesh);
        if (m_terrain_mesh_config.generate_water_mesh) {
            ImGui::ColorEdit4("Water Color", &m_terrain_mesh_config.water_color.x);
        }
    }

    ImGui::Separator();

    // Generate button
    if (ImGui::Button("Generate Terrain", ImVec2(-1, 40))) {
        generate_terrain();
    }

    // Clear button
    if (m_terrain_gpu_id != 0) {
        if (ImGui::Button("Clear Terrain", ImVec2(-1, 0))) {
            clear_terrain();
        }
    }

    ImGui::Separator();

    // Stats
    if (m_terrain_gpu_id != 0) {
        ImGui::Text("Terrain Active");
        ImGui::Text("Vertices: %zu", m_terrain_mesh.vertices.size());
        ImGui::Text("Triangles: %zu", m_terrain_mesh.indices.size() / 3);
        
        auto [min_h, max_h] = m_terrain_heightmap.get_height_range();
        ImGui::Text("Height Range: %.1f - %.1f m", min_h, max_h);
        
        if (m_water_gpu_id != 0) {
            ImGui::Text("Water plane at %.1f m", m_terrain_mesh_config.water_level);
        }

        ImGui::Separator();

        // Navigation
        if (ImGui::Button("Go to Terrain")) {
            glm::vec3 center = m_terrain_mesh.bounds.center();
            glm::vec3 cam_pos = center + glm::vec3(0.0f, m_terrain_config.max_height * 2.0f, m_terrain_config.size_z * 0.5f);
            m_camera.set_position(cam_pos);
            m_camera.set_target(center);
        }
    } else {
        ImGui::TextDisabled("No terrain generated.");
        ImGui::TextDisabled("Click 'Generate Terrain' to create one.");
    }
}

void Editor::generate_chunked_terrain() {
    // Clear existing chunked terrain
    clear_chunked_terrain();
    
    // Initialize the terrain tile manager
    m_terrain_tile_manager.init(m_terrain_tile_config);
    
    // Import OSM data for flattening if available
    const auto& osm_data = m_osm_parser.get_data();
    if (osm_data.stats.processed_roads > 0 || osm_data.stats.processed_buildings > 0) {
        // Collect all OSM elements from tiles
        std::vector<osm::Road> all_roads;
        std::vector<osm::Building> all_buildings;
        std::vector<osm::Area> all_areas;
        
        for (const auto& coord : m_tile_manager.get_all_tiles()) {
            const auto* tile = m_tile_manager.get_tile(coord);
            if (tile) {
                all_roads.insert(all_roads.end(), tile->roads.begin(), tile->roads.end());
                all_buildings.insert(all_buildings.end(), tile->buildings.begin(), tile->buildings.end());
                all_areas.insert(all_areas.end(), tile->areas.begin(), tile->areas.end());
            }
        }
        
        m_terrain_tile_manager.import_osm_data(all_roads, all_buildings, all_areas);
    }
    
    // Generate all chunks
    m_terrain_tile_manager.generate_all_chunks();
    
    // Build meshes
    m_terrain_tile_manager.build_all_meshes();
}

void Editor::clear_chunked_terrain() {
    // Release GPU resources for all chunks
    if (m_gpu_renderer) {
        for (const auto& coord : m_terrain_tile_manager.get_all_chunks()) {
            auto* chunk = m_terrain_tile_manager.get_chunk(coord);
            if (chunk && chunk->gpu_uploaded) {
                if (chunk->terrain_gpu_id != 0) {
                    m_gpu_renderer->release_mesh(chunk->terrain_gpu_id);
                }
                if (chunk->water_gpu_id != 0) {
                    m_gpu_renderer->release_mesh(chunk->water_gpu_id);
                }
            }
        }
    }
    
    m_terrain_tile_manager.clear();
}

void Editor::generate_terrain() {
    // Clear existing terrain
    clear_terrain();

    // Generate heightmap
    m_terrain_heightmap = m_terrain_generator.generate(m_terrain_config);

    // Update mesh config water level from terrain config
    m_terrain_mesh_config.water_level = m_terrain_config.water_level;

    // Build terrain mesh
    m_terrain_mesh = procgen::TerrainMeshBuilder::build_terrain_mesh(m_terrain_heightmap, m_terrain_mesh_config);

    // Upload to GPU
    if (m_gpu_renderer && m_terrain_mesh.is_valid()) {
        m_terrain_gpu_id = m_gpu_renderer->upload_mesh(m_terrain_mesh);
    }

    // Generate water mesh if requested
    if (m_terrain_mesh_config.generate_water_mesh && m_gpu_renderer) {
        m_water_mesh = procgen::TerrainMeshBuilder::build_water_mesh(
            m_terrain_heightmap, 
            m_terrain_config.water_level,
            m_terrain_mesh_config.water_color
        );
        if (m_water_mesh.is_valid()) {
            m_water_gpu_id = m_gpu_renderer->upload_mesh(m_water_mesh);
        }
    }
}

void Editor::clear_terrain() {
    if (m_gpu_renderer) {
        if (m_terrain_gpu_id != 0) {
            m_gpu_renderer->release_mesh(m_terrain_gpu_id);
            m_terrain_gpu_id = 0;
        }
        if (m_water_gpu_id != 0) {
            m_gpu_renderer->release_mesh(m_water_gpu_id);
            m_water_gpu_id = 0;
        }
    }
    m_terrain_mesh.clear();
    m_water_mesh.clear();
    m_terrain_heightmap = procgen::Heightmap{};
}

} // namespace stratum
