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

    /**
     * @brief Sample the procedural surface at a world position
     *
     * Pure: the same config and position always give the same height,
     * independent of any generated chunk. This is what makes it possible to
     * solve road elevation GLOBALLY, before a single terrain chunk exists --
     * terrain is chunked and generated on demand, but the height field itself is
     * just a function of (TerrainConfig, x, z). The road elevation solver queries
     * this through its HeightSampler callback; see osm/road/road_elevation.hpp.
     *
     * Equivalent to the sample_height() plus apply_flattening() pair that
     * generate_chunk() runs per cell, and it must stay equivalent: a road solved
     * against a different surface from the one the terrain mesh is built from
     * would float or sink.
     *
     * @warning Erosion is NOT reflected here. apply_erosion() is a whole-heightmap
     *          thermal relaxation: each iteration moves material between adjacent
     *          cells of an existing Heightmap, so its result at a position depends
     *          on that position's neighbourhood over every prior iteration. It has
     *          no per-position closed form and cannot be evaluated without a
     *          generated heightmap, which is exactly what sample_surface() exists
     *          to avoid needing.
     *
     *          The consequence, with TerrainConfig::apply_erosion enabled: road
     *          elevations are solved against the PRE-erosion surface, so the road
     *          and the eroded terrain can disagree by the erosion delta.
     *
     *          The mitigation is ordering. The carve pass runs AFTER erosion and
     *          forces the corridor cells to the solved road height regardless of
     *          what erosion left there, so the visible result stays consistent;
     *          the embankment falloff band then blends into the eroded terrain.
     *          Only the grade solve sees the pre-erosion shape, which means
     *          erosion can shift where a road wanted to sit but cannot make it
     *          float above or sink below the terrain it is carved into.
     *
     *          Note also that generate() applies erosion and generate_chunk() does
     *          not, so on the chunked path -- the one the editor uses -- erosion is
     *          never applied at all and this caveat is vacuous.
     *
     * @warning @p config.seed must match the seed this generator currently holds
     *          (get_seed()), which is what generate()/generate_chunk() leave behind
     *          for the config they were called with. When it does not match, the
     *          fallback builds the correct permutation table in thread-local
     *          storage instead of reseeding m_noise, so the returned height is
     *          still correct and the call is still re-entrant -- it just costs a
     *          table rebuild the first time each thread sees a new seed.
     *
     * @note Thread-safe and re-entrant. It reads m_noise and @p config and mutates
     *       no shared state, so it may be used as a HeightSampler by the parallel
     *       elevation solve. Noise::simplex2d() and the fbm/ridged wrappers are
     *       const and touch only the permutation tables, which are built eagerly
     *       in the Noise constructor and in reseed() -- there is no lazy
     *       initialisation or cache to race on. Concurrent sample_surface() calls
     *       are therefore safe; a concurrent generate()/generate_chunk()/reseed()
     *       on the SAME TerrainGenerator is NOT, because those reseed m_noise. Do
     *       not generate terrain while the elevation solve is running.
     *
     * @param config  Terrain parameters; must be the same config the chunks are
     *                generated from
     * @param world_x World X in metres
     * @param world_z World Z in metres
     * @return World Y of the procedural surface in metres
     */
    float sample_surface(const TerrainConfig& config, float world_x, float world_z) const;

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
