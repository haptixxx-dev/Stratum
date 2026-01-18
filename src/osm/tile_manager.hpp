#pragma once

#include "osm/types.hpp"
#include "renderer/mesh.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <unordered_map>
#include <functional>
#include <limits>
#include <future>
#include <mutex>

namespace stratum::osm {

/**
 * @brief Tile coordinate (x, y at a given zoom level)
 */
struct TileCoord {
    int x = 0;
    int y = 0;
    int zoom = 0;

    bool operator==(const TileCoord& other) const {
        return x == other.x && y == other.y && zoom == other.zoom;
    }

    bool operator<(const TileCoord& other) const {
        if (zoom != other.zoom) return zoom < other.zoom;
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

} // namespace stratum::osm

// Hash for TileCoord
namespace std {
template<>
struct hash<stratum::osm::TileCoord> {
    size_t operator()(const stratum::osm::TileCoord& t) const {
        return hash<int>()(t.x) ^ (hash<int>()(t.y) << 1) ^ (hash<int>()(t.zoom) << 2);
    }
};
}

namespace stratum::osm {

/**
 * @brief Data contained in a single tile
 */
struct Tile {
    TileCoord coord;
    BoundingBox bounds;           // Geographic bounds of this tile

    // OSM elements in this tile
    std::vector<Road> roads;
    std::vector<Building> buildings;
    std::vector<Area> areas;

    // Generated meshes (cached)
    std::vector<Mesh> road_meshes;
    std::vector<Mesh> building_meshes;
    std::vector<Mesh> area_meshes;

    // GPU mesh IDs (parallel arrays to *_meshes above)
    std::vector<uint32_t> road_gpu_ids;
    std::vector<uint32_t> building_gpu_ids;
    std::vector<uint32_t> area_gpu_ids;
    bool gpu_uploaded = false;  // True if meshes are uploaded to GPU

    // 3D bounding box for frustum culling (set from grid position)
    glm::vec3 bounds_min{0.0f};
    glm::vec3 bounds_max{0.0f};

    bool is_loaded = false;
    bool meshes_built = false;
    bool meshes_pending = false;  // True if async build is in progress

    bool has_valid_bounds() const {
        return bounds_min.x < bounds_max.x || bounds_min.z < bounds_max.z;
    }
};

/**
 * @brief Manages spatial tiling of OSM data for efficient rendering
 *
 * Divides the world into a grid of tiles. Each tile contains the OSM
 * elements that fall within its bounds. Supports frustum culling and
 * level-of-detail.
 */
class TileManager {
public:
    TileManager() = default;

    /**
     * @brief Initialize the tile grid for a given bounding box
     * @param bounds Geographic bounds to tile
     * @param tile_size_meters Approximate size of each tile in meters
     */
    void init(const BoundingBox& bounds, double tile_size_meters = 500.0);

    /**
     * @brief Clear all tiles and data
     */
    void clear();

    /**
     * @brief Assign OSM elements to appropriate tiles
     * @param data Parsed OSM data to tile
     */
    void assign_data(const ParsedOSMData& data);

    /**
     * @brief Get tiles that intersect a bounding box (for frustum culling)
     * @param min_local Minimum corner in local coordinates
     * @param max_local Maximum corner in local coordinates
     * @return Vector of tile coordinates that intersect the box
     */
    std::vector<TileCoord> get_visible_tiles(const glm::dvec2& min_local,
                                              const glm::dvec2& max_local) const;

    /**
     * @brief Get all tile coordinates
     */
    std::vector<TileCoord> get_all_tiles() const;

    /**
     * @brief Get a tile by coordinate
     * @return Pointer to tile or nullptr if not found
     */
    Tile* get_tile(const TileCoord& coord);
    const Tile* get_tile(const TileCoord& coord) const;

    /**
     * @brief Build meshes for a specific tile (blocking)
     */
    void build_tile_meshes(TileCoord coord);

    /**
     * @brief Queue async mesh building for a tile
     * @return true if queued, false if already built/pending
     */
    bool queue_tile_build_async(TileCoord coord);

    /**
     * @brief Check and collect completed async builds
     * @return Number of tiles that completed this frame
     */
    size_t poll_async_builds();

    /**
     * @brief Build meshes for all tiles (blocking)
     */
    void build_all_meshes();

    /**
     * @brief Get total counts across all tiles
     */
    size_t total_roads() const;
    size_t total_buildings() const;
    size_t total_areas() const;
    size_t tile_count() const { return m_tiles.size(); }

    // Grid info
    int grid_width() const { return m_grid_width; }
    int grid_height() const { return m_grid_height; }
    double tile_size() const { return m_tile_size; }

private:
    TileCoord local_to_tile(const glm::dvec2& local) const;

    // Build meshes for a tile (returns built meshes, thread-safe)
    struct BuiltMeshes {
        std::vector<Mesh> road_meshes;
        std::vector<Mesh> building_meshes;
        std::vector<Mesh> area_meshes;
    };
    BuiltMeshes build_tile_meshes_internal(const Tile& tile);

    std::unordered_map<TileCoord, Tile> m_tiles;

    // Async build tracking
    struct PendingBuild {
        TileCoord coord;
        std::future<BuiltMeshes> future;
    };
    std::vector<PendingBuild> m_pending_builds;
    std::mutex m_pending_mutex;

    // Grid parameters
    glm::dvec2 m_origin{0.0};     // Local coord origin (min corner)
    double m_tile_size = 500.0;   // Tile size in meters
    int m_grid_width = 0;
    int m_grid_height = 0;
};

} // namespace stratum::osm
