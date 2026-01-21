#pragma once

#include "procgen/terrain_generator.hpp"
#include "renderer/mesh.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <functional>

namespace stratum::procgen {

/**
 * @brief Terrain coloring mode
 */
enum class TerrainColorMode {
    Solid,          // Single color
    HeightGradient, // Color based on elevation
    SlopeGradient,  // Color based on steepness
    Biome           // Color based on height + slope (grass/rock/snow)
};

/**
 * @brief Configuration for terrain mesh generation
 */
struct TerrainMeshConfig {
    TerrainColorMode color_mode = TerrainColorMode::Biome;
    
    // Solid color (used when color_mode is Solid)
    glm::vec4 solid_color{0.4f, 0.5f, 0.3f, 1.0f};  // Greenish brown
    
    // Height gradient colors (low to high)
    glm::vec4 color_low{0.35f, 0.45f, 0.35f, 1.0f};   // Green (grass)
    glm::vec4 color_mid{0.55f, 0.5f, 0.4f, 1.0f};     // Brown (dirt)
    glm::vec4 color_high{0.7f, 0.7f, 0.72f, 1.0f};    // Gray (rock)
    glm::vec4 color_peak{0.95f, 0.95f, 0.98f, 1.0f};  // White (snow)
    
    // Height thresholds (normalized 0-1 of height range)
    float height_low = 0.2f;    // Below this: low color
    float height_mid = 0.5f;    // Transition to mid
    float height_high = 0.75f;  // Transition to high
    float height_peak = 0.9f;   // Above this: peak color
    
    // Slope coloring
    glm::vec4 color_steep{0.5f, 0.45f, 0.4f, 1.0f};  // Rocky steep slopes
    float steep_threshold = 35.0f;  // Degrees - slopes steeper than this use steep color
    float steep_blend = 10.0f;      // Blend range in degrees
    
    // Water
    glm::vec4 water_color{0.2f, 0.4f, 0.6f, 0.9f};
    float water_level = 0.0f;
    bool generate_water_mesh = true;
    
    // UV scaling for texturing
    float uv_scale = 0.1f;  // UV = world_pos * uv_scale
    
    // LOD settings
    int lod_level = 0;  // 0 = full resolution, 1 = half, 2 = quarter, etc.
};

/**
 * @brief Builds renderable meshes from heightmap data
 */
class TerrainMeshBuilder {
public:
    /**
     * @brief Build terrain mesh from heightmap
     * @param heightmap Source height data
     * @param config Mesh generation settings
     * @return Generated mesh ready for rendering
     */
    static Mesh build_terrain_mesh(const Heightmap& heightmap, const TerrainMeshConfig& config = {});
    
    /**
     * @brief Build water plane mesh at specified level
     * @param heightmap Source for bounds
     * @param water_level Y coordinate of water surface
     * @param color Water color
     * @return Flat water mesh
     */
    static Mesh build_water_mesh(const Heightmap& heightmap, float water_level, const glm::vec4& color);
    
    /**
     * @brief Build terrain mesh with custom coloring function
     * @param heightmap Source height data
     * @param color_func Function taking (world_x, world_z, height, slope) and returning color
     * @param uv_scale Scale for UV coordinates
     * @return Generated mesh
     */
    static Mesh build_terrain_mesh_custom(
        const Heightmap& heightmap,
        std::function<glm::vec4(float, float, float, float)> color_func,
        float uv_scale = 0.1f
    );
    
private:
    /**
     * @brief Get vertex color based on config and terrain properties
     */
    static glm::vec4 compute_vertex_color(
        const TerrainMeshConfig& config,
        float height,
        float slope,
        float height_normalized,
        float water_level
    );
    
    /**
     * @brief Smooth interpolation between colors
     */
    static glm::vec4 lerp_color(const glm::vec4& a, const glm::vec4& b, float t);
};

} // namespace stratum::procgen
