#include "procgen/terrain_generator.hpp"
#include <algorithm>
#include <cmath>

namespace stratum::procgen {

// ============================================================================
// Heightmap implementation
// ============================================================================

float Heightmap::sample(float world_x, float world_z) const {
    // Convert world coordinates to grid coordinates
    float grid_x = (world_x - origin.x) / cell_size_x;
    float grid_z = (world_z - origin.y) / cell_size_z;
    
    // Get integer cell indices
    int x0 = static_cast<int>(std::floor(grid_x));
    int z0 = static_cast<int>(std::floor(grid_z));
    int x1 = x0 + 1;
    int z1 = z0 + 1;
    
    // Clamp to valid range
    x0 = std::clamp(x0, 0, width - 1);
    x1 = std::clamp(x1, 0, width - 1);
    z0 = std::clamp(z0, 0, height - 1);
    z1 = std::clamp(z1, 0, height - 1);
    
    // Fractional part for interpolation
    float fx = grid_x - std::floor(grid_x);
    float fz = grid_z - std::floor(grid_z);
    
    // Bilinear interpolation
    float h00 = at(x0, z0);
    float h10 = at(x1, z0);
    float h01 = at(x0, z1);
    float h11 = at(x1, z1);
    
    float h0 = h00 * (1.0f - fx) + h10 * fx;
    float h1 = h01 * (1.0f - fx) + h11 * fx;
    
    return h0 * (1.0f - fz) + h1 * fz;
}

float Heightmap::at(int x, int z) const {
    if (!in_bounds(x, z)) return 0.0f;
    return data[z * width + x];
}

void Heightmap::set(int x, int z, float value) {
    if (in_bounds(x, z)) {
        data[z * width + x] = value;
    }
}

glm::vec3 Heightmap::compute_normal(float world_x, float world_z) const {
    // Sample heights at neighboring points for gradient
    float epsilon_x = cell_size_x;
    float epsilon_z = cell_size_z;
    
    float hL = sample(world_x - epsilon_x, world_z);
    float hR = sample(world_x + epsilon_x, world_z);
    float hD = sample(world_x, world_z - epsilon_z);
    float hU = sample(world_x, world_z + epsilon_z);
    
    // Compute normal from gradient
    glm::vec3 normal(
        (hL - hR) / (2.0f * epsilon_x),
        1.0f,
        (hD - hU) / (2.0f * epsilon_z)
    );
    
    return glm::normalize(normal);
}

float Heightmap::get_slope(float world_x, float world_z) const {
    glm::vec3 normal = compute_normal(world_x, world_z);
    // Slope is angle from vertical (y-axis)
    float dot = glm::dot(normal, glm::vec3(0.0f, 1.0f, 0.0f));
    return glm::degrees(std::acos(std::clamp(dot, -1.0f, 1.0f)));
}

std::pair<float, float> Heightmap::get_height_range() const {
    if (data.empty()) return {0.0f, 0.0f};
    
    auto [min_it, max_it] = std::minmax_element(data.begin(), data.end());
    return {*min_it, *max_it};
}

// ============================================================================
// TerrainGenerator implementation
// ============================================================================

TerrainGenerator::TerrainGenerator() : m_noise(12345) {}

TerrainGenerator::TerrainGenerator(uint32_t seed) : m_noise(seed) {}

void TerrainGenerator::reseed(uint32_t seed) {
    m_noise.reseed(seed);
}

float TerrainGenerator::sample_height(const TerrainConfig& config, float x, float z) const {
    float nx = x * config.noise_scale;
    float nz = z * config.noise_scale;
    
    float height = 0.0f;
    
    switch (config.type) {
        case TerrainType::Flat: {
            // Very subtle variation for flat terrain
            height = m_noise.fbm2d(nx, nz, 2, config.lacunarity, 0.3f);
            height *= 0.1f; // Very low amplitude
            break;
        }
        
        case TerrainType::Rolling: {
            // Gentle, smooth hills
            height = m_noise.fbm2d(nx, nz, config.octaves, config.lacunarity, config.persistence);
            // Apply smoothing curve (ease in/out)
            height = height * 0.5f + 0.5f; // Normalize to [0, 1]
            height = height * height * (3.0f - 2.0f * height); // Smoothstep
            height = height * 2.0f - 1.0f; // Back to [-1, 1]
            break;
        }
        
        case TerrainType::Hilly: {
            // More pronounced hills with some variety
            float base = m_noise.fbm2d(nx, nz, config.octaves, config.lacunarity, config.persistence);
            float detail = m_noise.fbm2d(nx * 2.0f, nz * 2.0f, 3, 2.0f, 0.5f);
            height = base * 0.8f + detail * 0.2f;
            break;
        }
        
        case TerrainType::Mountainous: {
            // Ridged mountains with sharp peaks
            float ridged = m_noise.ridged2d(nx, nz, config.octaves, config.lacunarity, 0.5f);
            float base = m_noise.fbm2d(nx * 0.5f, nz * 0.5f, 4, 2.0f, 0.5f);
            
            // Blend: use base noise to control where mountains appear
            float mountain_mask = (base + 1.0f) * 0.5f; // [0, 1]
            mountain_mask = std::pow(mountain_mask, 1.5f); // Make mountains less frequent
            
            height = ridged * mountain_mask * 2.0f - 1.0f;
            break;
        }
    }
    
    return config.base_height + height * config.max_height;
}

float TerrainGenerator::apply_flattening(const TerrainConfig& config, float x, float z, float height) const {
    if (!config.flatten_center) {
        return height;
    }
    
    // Distance from center of terrain
    float center_x = config.size_x * 0.5f;
    float center_z = config.size_z * 0.5f;
    float dx = x - center_x;
    float dz = z - center_z;
    float dist = std::sqrt(dx * dx + dz * dz);
    
    if (dist < config.flatten_radius) {
        // Fully flat inside radius
        return config.base_height;
    } else if (dist < config.flatten_radius + config.flatten_falloff) {
        // Smooth transition
        float t = (dist - config.flatten_radius) / config.flatten_falloff;
        t = t * t * (3.0f - 2.0f * t); // Smoothstep
        return config.base_height * (1.0f - t) + height * t;
    }
    
    return height;
}

void TerrainGenerator::apply_erosion(Heightmap& heightmap, int iterations, float strength) {
    // Simple thermal erosion: material flows from high to low areas
    const float talus_angle = 0.5f; // Maximum stable slope
    
    std::vector<float> temp(heightmap.data.size());
    
    for (int iter = 0; iter < iterations; ++iter) {
        // Copy current heights
        temp = heightmap.data;
        
        for (int z = 1; z < heightmap.height - 1; ++z) {
            for (int x = 1; x < heightmap.width - 1; ++x) {
                float h = heightmap.at(x, z);
                
                // Check 4 neighbors
                const int dx[] = {-1, 1, 0, 0};
                const int dz[] = {0, 0, -1, 1};
                
                float max_diff = 0.0f;
                int max_idx = -1;
                
                for (int i = 0; i < 4; ++i) {
                    int nx = x + dx[i];
                    int nz = z + dz[i];
                    float nh = heightmap.at(nx, nz);
                    float diff = h - nh;
                    
                    if (diff > talus_angle && diff > max_diff) {
                        max_diff = diff;
                        max_idx = i;
                    }
                }
                
                // Move material to lowest neighbor
                if (max_idx >= 0) {
                    float transfer = (max_diff - talus_angle) * strength * 0.5f;
                    int nx = x + dx[max_idx];
                    int nz = z + dz[max_idx];
                    
                    temp[z * heightmap.width + x] -= transfer;
                    temp[nz * heightmap.width + nx] += transfer;
                }
            }
        }
        
        heightmap.data = temp;
    }
}

Heightmap TerrainGenerator::generate(const TerrainConfig& config) {
    Heightmap heightmap;
    heightmap.width = config.resolution_x;
    heightmap.height = config.resolution_z;
    heightmap.cell_size_x = config.size_x / (config.resolution_x - 1);
    heightmap.cell_size_z = config.size_z / (config.resolution_z - 1);
    heightmap.origin = glm::vec2(0.0f, 0.0f);
    heightmap.data.resize(heightmap.width * heightmap.height);
    
    // Reseed noise with config seed
    m_noise.reseed(config.seed);
    
    // Generate height values
    for (int z = 0; z < heightmap.height; ++z) {
        for (int x = 0; x < heightmap.width; ++x) {
            float world_x = x * heightmap.cell_size_x;
            float world_z = z * heightmap.cell_size_z;
            
            float h = sample_height(config, world_x, world_z);
            h = apply_flattening(config, world_x, world_z, h);
            
            heightmap.set(x, z, h);
        }
    }
    
    // Apply erosion if requested
    if (config.apply_erosion) {
        apply_erosion(heightmap, config.erosion_iterations, config.erosion_strength);
    }
    
    return heightmap;
}

Heightmap TerrainGenerator::generate_chunk(const TerrainConfig& config,
                                            const glm::vec2& origin,
                                            float chunk_size_x, float chunk_size_z) {
    Heightmap heightmap;
    heightmap.width = config.resolution_x;
    heightmap.height = config.resolution_z;
    heightmap.cell_size_x = chunk_size_x / (config.resolution_x - 1);
    heightmap.cell_size_z = chunk_size_z / (config.resolution_z - 1);
    heightmap.origin = origin;
    heightmap.data.resize(heightmap.width * heightmap.height);
    
    // Use consistent seed for seamless chunks
    m_noise.reseed(config.seed);
    
    // Generate height values for this chunk
    for (int z = 0; z < heightmap.height; ++z) {
        for (int x = 0; x < heightmap.width; ++x) {
            float world_x = origin.x + x * heightmap.cell_size_x;
            float world_z = origin.y + z * heightmap.cell_size_z;
            
            float h = sample_height(config, world_x, world_z);
            h = apply_flattening(config, world_x, world_z, h);
            
            heightmap.set(x, z, h);
        }
    }
    
    return heightmap;
}

} // namespace stratum::procgen
