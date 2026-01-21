#pragma once

#include "procgen/terrain_generator.hpp"
#include "procgen/terrain_mesh_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"
#include <glm/glm.hpp>
#include <unordered_map>
#include <vector>
#include <functional>

namespace stratum::procgen {

/**
 * @brief Terrain chunk coordinate
 */
struct TerrainChunkCoord {
    int x = 0;
    int z = 0;
    
    bool operator==(const TerrainChunkCoord& other) const {
        return x == other.x && z == other.z;
    }
};

} // namespace stratum::procgen

// Hash for TerrainChunkCoord
namespace std {
template<>
struct hash<stratum::procgen::TerrainChunkCoord> {
    size_t operator()(const stratum::procgen::TerrainChunkCoord& c) const {
        return hash<int>()(c.x) ^ (hash<int>()(c.z) << 16);
    }
};
}

namespace stratum::procgen {

/**
 * @brief A single terrain chunk with heightmap and mesh
 */
struct TerrainChunk {
    TerrainChunkCoord coord;
    Heightmap heightmap;
    Mesh terrain_mesh;
    Mesh water_mesh;
    
    // GPU handles
    uint32_t terrain_gpu_id = 0;
    uint32_t water_gpu_id = 0;
    
    // Bounds for culling
    glm::vec3 bounds_min{0.0f};
    glm::vec3 bounds_max{0.0f};
    
    // State
    bool heightmap_generated = false;
    bool mesh_built = false;
    bool gpu_uploaded = false;
    
    // OSM influence data
    bool has_osm_data = false;      // True if OSM features exist in this chunk
    float osm_coverage = 0.0f;      // 0-1, how much of chunk has OSM features
};

/**
 * @brief Flattening mask for OSM integration
 * 
 * Stores per-vertex weights indicating how much to flatten the terrain
 * 0 = full procedural height, 1 = fully flat at base_height
 */
struct FlattenMask {
    std::vector<float> weights;     // Flattening weight per heightmap cell
    int width = 0;
    int height = 0;
    float cell_size_x = 1.0f;
    float cell_size_z = 1.0f;
    glm::vec2 origin{0.0f};
    
    float sample(float world_x, float world_z) const;
    void set(int x, int z, float value);
    float at(int x, int z) const;
    bool in_bounds(int x, int z) const {
        return x >= 0 && x < width && z >= 0 && z < height;
    }
};

/**
 * @brief Configuration for terrain tile manager
 */
struct TerrainTileConfig {
    // Chunk sizing
    float chunk_size = 500.0f;          // Size of each chunk in meters (should match OSM tile size)
    int chunk_resolution = 64;          // Vertices per chunk edge
    
    // World bounds
    glm::vec2 world_min{-2000.0f};      // Minimum world coordinates
    glm::vec2 world_max{2000.0f};       // Maximum world coordinates
    
    // Terrain generation (passed to TerrainGenerator)
    TerrainConfig terrain;
    
    // Mesh generation
    TerrainMeshConfig mesh;
    
    // OSM blending
    float osm_flatten_radius = 10.0f;   // Radius around OSM features to flatten
    float osm_blend_distance = 50.0f;   // Distance over which to blend to procedural
    float osm_base_height = 0.0f;       // Height to flatten OSM areas to
};

/**
 * @brief Manages chunked terrain with OSM integration
 * 
 * Creates terrain chunks that seamlessly tile together and blend
 * with imported OSM data by flattening areas where buildings/roads exist.
 */
class TerrainTileManager {
public:
    TerrainTileManager();
    
    /**
     * @brief Initialize the terrain system
     * @param config Configuration for terrain generation
     */
    void init(const TerrainTileConfig& config);
    
    /**
     * @brief Clear all terrain data
     */
    void clear();
    
    /**
     * @brief Update configuration (regenerates all chunks)
     */
    void set_config(const TerrainTileConfig& config);
    
    /**
     * @brief Get current configuration
     */
    const TerrainTileConfig& get_config() const { return m_config; }
    TerrainTileConfig& get_config() { return m_config; }
    
    /**
     * @brief Import OSM data to create flattening masks
     * 
     * Analyzes OSM buildings, roads, and areas to determine where
     * terrain should be flattened for clean blending.
     * 
     * @param roads OSM roads
     * @param buildings OSM buildings  
     * @param areas OSM areas (parks, water, etc.)
     */
    void import_osm_data(const std::vector<osm::Road>& roads,
                         const std::vector<osm::Building>& buildings,
                         const std::vector<osm::Area>& areas);
    
    /**
     * @brief Generate terrain for a specific chunk
     * @param coord Chunk coordinate
     * @return true if chunk was generated
     */
    bool generate_chunk(const TerrainChunkCoord& coord);
    
    /**
     * @brief Generate all chunks within world bounds
     */
    void generate_all_chunks();
    
    /**
     * @brief Generate chunks visible from a camera position
     * @param camera_pos Camera world position
     * @param view_distance Maximum distance to generate chunks
     */
    void generate_visible_chunks(const glm::vec3& camera_pos, float view_distance);
    
    /**
     * @brief Build mesh for a chunk (call after generate_chunk)
     * @param coord Chunk coordinate
     * @return true if mesh was built
     */
    bool build_chunk_mesh(const TerrainChunkCoord& coord);
    
    /**
     * @brief Build meshes for all generated chunks
     */
    void build_all_meshes();
    
    /**
     * @brief Get a terrain chunk
     * @return Pointer to chunk or nullptr
     */
    TerrainChunk* get_chunk(const TerrainChunkCoord& coord);
    const TerrainChunk* get_chunk(const TerrainChunkCoord& coord) const;
    
    /**
     * @brief Get all chunk coordinates
     */
    std::vector<TerrainChunkCoord> get_all_chunks() const;
    
    /**
     * @brief Get chunks that intersect a bounding box
     */
    std::vector<TerrainChunkCoord> get_chunks_in_bounds(const glm::vec2& min, const glm::vec2& max) const;
    
    /**
     * @brief Convert world position to chunk coordinate
     */
    TerrainChunkCoord world_to_chunk(const glm::vec2& world_pos) const;
    
    /**
     * @brief Get world bounds of a chunk
     */
    void get_chunk_bounds(const TerrainChunkCoord& coord, glm::vec2& out_min, glm::vec2& out_max) const;
    
    /**
     * @brief Sample terrain height at world position (across all chunks)
     * @return Height at position, or base_height if no chunk exists
     */
    float sample_height(float world_x, float world_z) const;
    
    /**
     * @brief Get statistics
     */
    size_t chunk_count() const { return m_chunks.size(); }
    size_t generated_count() const;
    size_t mesh_count() const;
    
private:
    TerrainTileConfig m_config;
    TerrainGenerator m_generator;
    std::unordered_map<TerrainChunkCoord, TerrainChunk> m_chunks;
    
    // Global flattening mask (covers entire world bounds)
    FlattenMask m_flatten_mask;
    bool m_has_osm_data = false;
    
    /**
     * @brief Create flattening mask from OSM features
     */
    void build_flatten_mask(const std::vector<osm::Road>& roads,
                            const std::vector<osm::Building>& buildings,
                            const std::vector<osm::Area>& areas);
    
    /**
     * @brief Apply flattening mask to a heightmap
     */
    void apply_flatten_mask(Heightmap& heightmap) const;
    
    /**
     * @brief Expand a point into the flatten mask with falloff
     */
    void paint_flatten_point(float world_x, float world_z, float radius, float falloff);
    
    /**
     * @brief Expand a line segment into the flatten mask
     */
    void paint_flatten_line(const glm::dvec2& p0, const glm::dvec2& p1, float radius, float falloff);
    
    /**
     * @brief Expand a polygon into the flatten mask
     */
    void paint_flatten_polygon(const std::vector<glm::dvec2>& polygon, float falloff);
};

} // namespace stratum::procgen
