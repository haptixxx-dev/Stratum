#pragma once

#include "procgen/noise.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
#include <functional>

namespace stratum::procgen {

/**
 * @brief Terrain type for different biome/surface characteristics
 */
enum class TerrainType {
    Flat,       // Flat plains, good for urban areas
    Rolling,    // Gentle hills
    Hilly,      // More pronounced hills
    Mountainous // Steep mountains with ridges
};

/**
 * @brief Configuration for terrain generation
 */
struct TerrainConfig {
    // Size and resolution
    float size_x = 1000.0f;           // Terrain width in meters
    float size_z = 1000.0f;           // Terrain depth in meters  
    int resolution_x = 128;           // Number of vertices along X
    int resolution_z = 128;           // Number of vertices along Z
    
    // Height parameters
    float base_height = 0.0f;         // Base terrain elevation
    float max_height = 50.0f;         // Maximum height variation
    float water_level = 0.0f;         // Sea level (for water detection)
    
    // Noise parameters
    TerrainType type = TerrainType::Rolling;
    uint32_t seed = 12345;
    float noise_scale = 0.002f;       // Base noise frequency
    int octaves = 6;                  // Fractal octaves
    float lacunarity = 2.0f;          // Frequency multiplier per octave
    float persistence = 0.5f;         // Amplitude multiplier per octave
    
    // Erosion simulation (simple thermal erosion)
    bool apply_erosion = false;
    int erosion_iterations = 10;
    float erosion_strength = 0.1f;
    
    // Flattening for urban areas
    bool flatten_center = false;      // Create flat area in center for city
    float flatten_radius = 200.0f;    // Radius of flat area
    float flatten_falloff = 100.0f;   // Transition zone width
};

/**
 * @brief 2D heightmap data structure
 */
struct Heightmap {
    std::vector<float> data;          // Height values (row-major)
    int width = 0;                    // Number of samples in X
    int height = 0;                   // Number of samples in Z
    float cell_size_x = 1.0f;         // Meters per cell in X
    float cell_size_z = 1.0f;         // Meters per cell in Z
    glm::vec2 origin{0.0f};           // World position of corner (0,0)
    
    /**
     * @brief Sample height at world coordinates (bilinear interpolation)
     */
    float sample(float world_x, float world_z) const;
    
    /**
     * @brief Sample height at grid coordinates (no interpolation)
     */
    float at(int x, int z) const;
    
    /**
     * @brief Set height at grid coordinates
     */
    void set(int x, int z, float value);
    
    /**
     * @brief Compute normal at world coordinates
     */
    glm::vec3 compute_normal(float world_x, float world_z) const;
    
    /**
     * @brief Get slope angle in degrees at world coordinates
     */
    float get_slope(float world_x, float world_z) const;
    
    /**
     * @brief Check if coordinates are within bounds
     */
    bool in_bounds(int x, int z) const {
        return x >= 0 && x < width && z >= 0 && z < height;
    }
    
    /**
     * @brief Get min/max height values
     */
    std::pair<float, float> get_height_range() const;
};

/**
 * @brief Procedural terrain heightmap generator
 */
class TerrainGenerator {
public:
    TerrainGenerator();
    explicit TerrainGenerator(uint32_t seed);
    
    /**
     * @brief Generate heightmap from configuration
     * @param config Terrain generation parameters
     * @return Generated heightmap
     */
    Heightmap generate(const TerrainConfig& config);
    
    /**
     * @brief Generate heightmap for a specific region (for tiled terrain)
     * @param config Terrain parameters
     * @param origin World origin of this chunk
     * @param chunk_size_x Chunk width in meters
     * @param chunk_size_z Chunk depth in meters
     * @return Generated heightmap chunk
     */
    Heightmap generate_chunk(const TerrainConfig& config, 
                             const glm::vec2& origin,
                             float chunk_size_x, float chunk_size_z);
    
    /**
     * @brief Reseed the generator
     */
    void reseed(uint32_t seed);
    
    /**
     * @brief Get current seed
     */
    uint32_t get_seed() const { return m_noise.get_seed(); }
    
private:
    Noise m_noise;
    
    /**
     * @brief Sample raw height at world position based on terrain type
     */
    float sample_height(const TerrainConfig& config, float x, float z) const;
    
    /**
     * @brief Apply flattening modifier for city areas
     */
    float apply_flattening(const TerrainConfig& config, float x, float z, float height) const;
    
    /**
     * @brief Simple thermal erosion pass
     */
    void apply_erosion(Heightmap& heightmap, int iterations, float strength);
};

} // namespace stratum::procgen
