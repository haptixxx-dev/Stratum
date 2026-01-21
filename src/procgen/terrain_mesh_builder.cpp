#include "procgen/terrain_mesh_builder.hpp"
#include <cmath>
#include <algorithm>

namespace stratum::procgen {

glm::vec4 TerrainMeshBuilder::lerp_color(const glm::vec4& a, const glm::vec4& b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return a * (1.0f - t) + b * t;
}

glm::vec4 TerrainMeshBuilder::compute_vertex_color(
    const TerrainMeshConfig& config,
    float height,
    float slope,
    float height_normalized,
    float water_level
) {
    // If below water level, use water-adjacent color (slightly darker)
    if (height < water_level) {
        return glm::vec4(0.35f, 0.4f, 0.35f, 1.0f); // Underwater terrain
    }
    
    glm::vec4 base_color;
    
    switch (config.color_mode) {
        case TerrainColorMode::Solid:
            base_color = config.solid_color;
            break;
            
        case TerrainColorMode::HeightGradient: {
            // Simple height-based gradient
            if (height_normalized < config.height_low) {
                base_color = config.color_low;
            } else if (height_normalized < config.height_mid) {
                float t = (height_normalized - config.height_low) / (config.height_mid - config.height_low);
                base_color = lerp_color(config.color_low, config.color_mid, t);
            } else if (height_normalized < config.height_high) {
                float t = (height_normalized - config.height_mid) / (config.height_high - config.height_mid);
                base_color = lerp_color(config.color_mid, config.color_high, t);
            } else if (height_normalized < config.height_peak) {
                float t = (height_normalized - config.height_high) / (config.height_peak - config.height_high);
                base_color = lerp_color(config.color_high, config.color_peak, t);
            } else {
                base_color = config.color_peak;
            }
            break;
        }
        
        case TerrainColorMode::SlopeGradient: {
            // Color based on slope steepness
            if (slope < config.steep_threshold - config.steep_blend) {
                base_color = config.color_low; // Flat areas = grass
            } else if (slope > config.steep_threshold + config.steep_blend) {
                base_color = config.color_steep; // Steep = rock
            } else {
                float t = (slope - (config.steep_threshold - config.steep_blend)) / (2.0f * config.steep_blend);
                base_color = lerp_color(config.color_low, config.color_steep, t);
            }
            break;
        }
        
        case TerrainColorMode::Biome: {
            // Combined height + slope for realistic terrain
            
            // First get height-based color
            glm::vec4 height_color;
            if (height_normalized < config.height_low) {
                height_color = config.color_low;
            } else if (height_normalized < config.height_mid) {
                float t = (height_normalized - config.height_low) / (config.height_mid - config.height_low);
                height_color = lerp_color(config.color_low, config.color_mid, t);
            } else if (height_normalized < config.height_high) {
                float t = (height_normalized - config.height_mid) / (config.height_high - config.height_mid);
                height_color = lerp_color(config.color_mid, config.color_high, t);
            } else if (height_normalized < config.height_peak) {
                float t = (height_normalized - config.height_high) / (config.height_peak - config.height_high);
                height_color = lerp_color(config.color_high, config.color_peak, t);
            } else {
                height_color = config.color_peak;
            }
            
            // Blend with slope-based rock color
            float slope_blend = 0.0f;
            if (slope > config.steep_threshold - config.steep_blend) {
                if (slope > config.steep_threshold + config.steep_blend) {
                    slope_blend = 1.0f;
                } else {
                    slope_blend = (slope - (config.steep_threshold - config.steep_blend)) / (2.0f * config.steep_blend);
                }
            }
            
            // Steep slopes become rocky regardless of height
            base_color = lerp_color(height_color, config.color_steep, slope_blend);
            break;
        }
    }
    
    return base_color;
}

Mesh TerrainMeshBuilder::build_terrain_mesh(const Heightmap& heightmap, const TerrainMeshConfig& config) {
    Mesh mesh;
    
    if (heightmap.width < 2 || heightmap.height < 2) {
        return mesh;
    }
    
    // Apply LOD reduction
    int step = 1 << config.lod_level;
    int width = (heightmap.width - 1) / step + 1;
    int height = (heightmap.height - 1) / step + 1;
    
    if (width < 2 || height < 2) {
        return mesh;
    }
    
    // Get height range for normalization
    auto [min_h, max_h] = heightmap.get_height_range();
    float height_range = max_h - min_h;
    if (height_range < 0.001f) height_range = 1.0f;
    
    // Reserve space for vertices and indices
    mesh.vertices.reserve(width * height);
    mesh.indices.reserve((width - 1) * (height - 1) * 6);
    
    // Generate vertices
    for (int z = 0; z < height; ++z) {
        for (int x = 0; x < width; ++x) {
            int src_x = std::min(x * step, heightmap.width - 1);
            int src_z = std::min(z * step, heightmap.height - 1);
            
            float world_x = heightmap.origin.x + src_x * heightmap.cell_size_x;
            float world_z = heightmap.origin.y + src_z * heightmap.cell_size_z;
            float h = heightmap.at(src_x, src_z);
            
            // Position (Y-up coordinate system, Z inverted for rendering)
            glm::vec3 position(world_x, h, -world_z);
            
            // Normal from heightmap gradient
            glm::vec3 normal = heightmap.compute_normal(world_x, world_z);
            // Flip Z for rendering coordinate system
            normal.z = -normal.z;
            
            // UV coordinates
            glm::vec2 uv(world_x * config.uv_scale, world_z * config.uv_scale);
            
            // Compute color based on height and slope
            float slope = heightmap.get_slope(world_x, world_z);
            float height_normalized = (h - min_h) / height_range;
            glm::vec4 color = compute_vertex_color(config, h, slope, height_normalized, config.water_level);
            
            mesh.vertices.push_back({position, normal, uv, color});
        }
    }
    
    // Generate indices (two triangles per grid cell)
    // Winding order: CW when viewed from above (matching OSM road mesh winding)
    for (int z = 0; z < height - 1; ++z) {
        for (int x = 0; x < width - 1; ++x) {
            uint32_t i00 = z * width + x;
            uint32_t i10 = z * width + x + 1;
            uint32_t i01 = (z + 1) * width + x;
            uint32_t i11 = (z + 1) * width + x + 1;
            
            // Triangle 1
            mesh.indices.push_back(i00);
            mesh.indices.push_back(i10);
            mesh.indices.push_back(i01);
            
            // Triangle 2
            mesh.indices.push_back(i10);
            mesh.indices.push_back(i11);
            mesh.indices.push_back(i01);
        }
    }
    
    mesh.compute_bounds();
    mesh.compute_tangents();
    
    return mesh;
}

Mesh TerrainMeshBuilder::build_water_mesh(const Heightmap& heightmap, float water_level, const glm::vec4& color) {
    Mesh mesh;
    
    if (heightmap.width < 2 || heightmap.height < 2) {
        return mesh;
    }
    
    // Calculate water plane bounds
    float min_x = heightmap.origin.x;
    float max_x = heightmap.origin.x + (heightmap.width - 1) * heightmap.cell_size_x;
    float min_z = heightmap.origin.y;
    float max_z = heightmap.origin.y + (heightmap.height - 1) * heightmap.cell_size_z;
    
    glm::vec3 up_normal(0.0f, 1.0f, 0.0f);
    
    // Create a simple quad for water
    // Note: Z is inverted for rendering
    mesh.vertices.push_back({glm::vec3(min_x, water_level, -min_z), up_normal, glm::vec2(0.0f, 0.0f), color});
    mesh.vertices.push_back({glm::vec3(max_x, water_level, -min_z), up_normal, glm::vec2(1.0f, 0.0f), color});
    mesh.vertices.push_back({glm::vec3(max_x, water_level, -max_z), up_normal, glm::vec2(1.0f, 1.0f), color});
    mesh.vertices.push_back({glm::vec3(min_x, water_level, -max_z), up_normal, glm::vec2(0.0f, 1.0f), color});
    
    // Two triangles
    mesh.indices = {0, 1, 2, 0, 2, 3};
    
    mesh.compute_bounds();
    
    return mesh;
}

Mesh TerrainMeshBuilder::build_terrain_mesh_custom(
    const Heightmap& heightmap,
    std::function<glm::vec4(float, float, float, float)> color_func,
    float uv_scale
) {
    Mesh mesh;
    
    if (heightmap.width < 2 || heightmap.height < 2) {
        return mesh;
    }
    
    int width = heightmap.width;
    int height = heightmap.height;
    
    mesh.vertices.reserve(width * height);
    mesh.indices.reserve((width - 1) * (height - 1) * 6);
    
    // Generate vertices
    for (int z = 0; z < height; ++z) {
        for (int x = 0; x < width; ++x) {
            float world_x = heightmap.origin.x + x * heightmap.cell_size_x;
            float world_z = heightmap.origin.y + z * heightmap.cell_size_z;
            float h = heightmap.at(x, z);
            
            glm::vec3 position(world_x, h, -world_z);
            glm::vec3 normal = heightmap.compute_normal(world_x, world_z);
            normal.z = -normal.z;
            
            glm::vec2 uv(world_x * uv_scale, world_z * uv_scale);
            
            float slope = heightmap.get_slope(world_x, world_z);
            glm::vec4 color = color_func(world_x, world_z, h, slope);
            
            mesh.vertices.push_back({position, normal, uv, color});
        }
    }
    
    // Generate indices (CW winding for upward-facing)
    for (int z = 0; z < height - 1; ++z) {
        for (int x = 0; x < width - 1; ++x) {
            uint32_t i00 = z * width + x;
            uint32_t i10 = z * width + x + 1;
            uint32_t i01 = (z + 1) * width + x;
            uint32_t i11 = (z + 1) * width + x + 1;
            
            mesh.indices.push_back(i00);
            mesh.indices.push_back(i10);
            mesh.indices.push_back(i01);
            
            mesh.indices.push_back(i10);
            mesh.indices.push_back(i11);
            mesh.indices.push_back(i01);
        }
    }
    
    mesh.compute_bounds();
    mesh.compute_tangents();
    
    return mesh;
}

} // namespace stratum::procgen
