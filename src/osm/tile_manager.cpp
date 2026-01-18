#include "osm/tile_manager.hpp"
#include "osm/mesh_builder.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <algorithm>
#include <set>

namespace stratum::osm {

void TileManager::init(const BoundingBox& bounds, double tile_size_meters) {
    clear();

    if (!bounds.is_valid()) {
        spdlog::warn("TileManager: Invalid bounds, cannot initialize");
        return;
    }

    m_tile_size = tile_size_meters;

    // Compute local coordinate extents
    double width = bounds.width_meters();
    double height = bounds.height_meters();

    m_grid_width = static_cast<int>(std::ceil(width / m_tile_size));
    m_grid_height = static_cast<int>(std::ceil(height / m_tile_size));

    // Ensure at least 1x1 grid
    m_grid_width = std::max(1, m_grid_width);
    m_grid_height = std::max(1, m_grid_height);

    // Origin is at min corner (local coords are centered, so this is negative)
    m_origin = glm::dvec2(-width / 2.0, -height / 2.0);

    spdlog::info("TileManager: Initialized {}x{} grid ({} tiles, {:.0f}m each)",
                 m_grid_width, m_grid_height, m_grid_width * m_grid_height, m_tile_size);
}

void TileManager::clear() {
    m_tiles.clear();
    m_origin = glm::dvec2(0.0);
    m_grid_width = 0;
    m_grid_height = 0;
}

TileCoord TileManager::local_to_tile(const glm::dvec2& local) const {
    TileCoord coord;
    coord.x = static_cast<int>(std::floor((local.x - m_origin.x) / m_tile_size));
    coord.y = static_cast<int>(std::floor((local.y - m_origin.y) / m_tile_size));
    coord.zoom = 0; // Single zoom level for now

    // Clamp to grid bounds
    coord.x = std::clamp(coord.x, 0, m_grid_width - 1);
    coord.y = std::clamp(coord.y, 0, m_grid_height - 1);

    return coord;
}

void TileManager::assign_data(const ParsedOSMData& data) {
    // Helper to set tile bounds from grid position
    auto set_tile_bounds = [this](Tile& tile, const TileCoord& coord) {
        // Compute 2D bounds in local space
        double min_x = m_origin.x + coord.x * m_tile_size;
        double min_y = m_origin.y + coord.y * m_tile_size;
        double max_x = min_x + m_tile_size;
        double max_y = min_y + m_tile_size;

        // Convert to 3D (note: local Y becomes -Z in 3D)
        tile.bounds_min = glm::vec3(static_cast<float>(min_x), 0.0f, static_cast<float>(-max_y));
        tile.bounds_max = glm::vec3(static_cast<float>(max_x), 200.0f, static_cast<float>(-min_y));
    };

    // Helper to get or create tile
    auto get_or_create_tile = [&](const TileCoord& coord) -> Tile& {
        auto& tile = m_tiles[coord];
        tile.coord = coord;
        tile.is_loaded = true;
        set_tile_bounds(tile, coord);
        return tile;
    };

    // Assign roads to ALL tiles they pass through
    for (const auto& road : data.roads) {
        if (road.polyline.empty()) continue;

        // Track which tiles this road has been assigned to (avoid duplicates)
        std::set<TileCoord> assigned_tiles;

        for (const auto& pt : road.polyline) {
            TileCoord coord = local_to_tile(pt);
            if (assigned_tiles.insert(coord).second) {
                auto& tile = get_or_create_tile(coord);
                tile.roads.push_back(road);
            }
        }
    }

    // Assign buildings to ALL tiles they intersect
    for (const auto& building : data.buildings) {
        if (building.footprint.empty()) continue;

        std::set<TileCoord> assigned_tiles;

        for (const auto& pt : building.footprint) {
            TileCoord coord = local_to_tile(pt);
            if (assigned_tiles.insert(coord).second) {
                auto& tile = get_or_create_tile(coord);
                tile.buildings.push_back(building);
            }
        }
    }

    // Assign areas to ALL tiles they intersect
    for (const auto& area : data.areas) {
        if (area.polygon.empty()) continue;

        std::set<TileCoord> assigned_tiles;

        for (const auto& pt : area.polygon) {
            TileCoord coord = local_to_tile(pt);
            if (assigned_tiles.insert(coord).second) {
                auto& tile = get_or_create_tile(coord);
                tile.areas.push_back(area);
            }
        }
    }

    spdlog::info("TileManager: Assigned data to {} tiles", m_tiles.size());
}

std::vector<TileCoord> TileManager::get_visible_tiles(const glm::dvec2& min_local,
                                                       const glm::dvec2& max_local) const {
    std::vector<TileCoord> visible;

    TileCoord min_tile = local_to_tile(min_local);
    TileCoord max_tile = local_to_tile(max_local);

    for (int y = min_tile.y; y <= max_tile.y; ++y) {
        for (int x = min_tile.x; x <= max_tile.x; ++x) {
            TileCoord coord{x, y, 0};
            if (m_tiles.count(coord) > 0) {
                visible.push_back(coord);
            }
        }
    }

    return visible;
}

std::vector<TileCoord> TileManager::get_all_tiles() const {
    std::vector<TileCoord> all;
    all.reserve(m_tiles.size());
    for (const auto& [coord, tile] : m_tiles) {
        all.push_back(coord);
    }
    return all;
}

Tile* TileManager::get_tile(const TileCoord& coord) {
    auto it = m_tiles.find(coord);
    return it != m_tiles.end() ? &it->second : nullptr;
}

const Tile* TileManager::get_tile(const TileCoord& coord) const {
    auto it = m_tiles.find(coord);
    return it != m_tiles.end() ? &it->second : nullptr;
}

void TileManager::build_tile_meshes(TileCoord coord) {
    auto* tile = get_tile(coord);
    if (!tile || tile->meshes_built) return;

    // Build road meshes
    tile->road_meshes.reserve(tile->roads.size());
    for (const auto& road : tile->roads) {
        Mesh mesh = MeshBuilder::build_road_mesh(road);
        if (mesh.is_valid()) {
            tile->road_meshes.push_back(std::move(mesh));
        }
    }

    // Build junction meshes for roads in this tile and add to road_meshes
    if (!tile->roads.empty()) {
        auto junctions = MeshBuilder::build_junction_meshes(tile->roads);
        for (auto& junction : junctions) {
            if (junction.is_valid()) {
                tile->road_meshes.push_back(std::move(junction));
            }
        }
    }

    // Build building meshes
    tile->building_meshes.reserve(tile->buildings.size());
    for (const auto& building : tile->buildings) {
        Mesh mesh = MeshBuilder::build_building_mesh(building);
        if (mesh.is_valid()) {
            tile->building_meshes.push_back(std::move(mesh));
        }
    }

    // Build area meshes
    tile->area_meshes.reserve(tile->areas.size());
    for (const auto& area : tile->areas) {
        Mesh mesh = MeshBuilder::build_area_mesh(area);
        if (mesh.is_valid()) {
            tile->area_meshes.push_back(std::move(mesh));
        }
    }

    tile->meshes_built = true;
}

void TileManager::build_all_meshes() {
    size_t total = m_tiles.size();
    size_t current = 0;
    size_t total_meshes = 0;

    spdlog::info("Building meshes for {} tiles...", total);

    for (auto& [coord, tile] : m_tiles) {
        build_tile_meshes(coord);
        total_meshes += tile.road_meshes.size() + tile.building_meshes.size() + tile.area_meshes.size();
        current++;
        
        // Log progress every 10 tiles or at the end
        if (current % 10 == 0 || current == total) {
            spdlog::info("  Built meshes for {}/{} tiles ({} meshes so far)", current, total, total_meshes);
        }
    }

    spdlog::info("Mesh building complete: {} total meshes", total_meshes);
}

size_t TileManager::total_roads() const {
    size_t count = 0;
    for (const auto& [coord, tile] : m_tiles) {
        count += tile.roads.size();
    }
    return count;
}

size_t TileManager::total_buildings() const {
    size_t count = 0;
    for (const auto& [coord, tile] : m_tiles) {
        count += tile.buildings.size();
    }
    return count;
}

size_t TileManager::total_areas() const {
    size_t count = 0;
    for (const auto& [coord, tile] : m_tiles) {
        count += tile.areas.size();
    }
    return count;
}

TileManager::BuiltMeshes TileManager::build_tile_meshes_internal(const Tile& tile) {
    BuiltMeshes result;

    // Build road meshes
    result.road_meshes.reserve(tile.roads.size());
    for (const auto& road : tile.roads) {
        Mesh mesh = MeshBuilder::build_road_mesh(road);
        if (mesh.is_valid()) {
            result.road_meshes.push_back(std::move(mesh));
        }
    }

    // Build junction meshes
    if (!tile.roads.empty()) {
        auto junctions = MeshBuilder::build_junction_meshes(tile.roads);
        for (auto& junction : junctions) {
            if (junction.is_valid()) {
                result.road_meshes.push_back(std::move(junction));
            }
        }
    }

    // Build building meshes
    result.building_meshes.reserve(tile.buildings.size());
    for (const auto& building : tile.buildings) {
        Mesh mesh = MeshBuilder::build_building_mesh(building);
        if (mesh.is_valid()) {
            result.building_meshes.push_back(std::move(mesh));
        }
    }

    // Build area meshes
    result.area_meshes.reserve(tile.areas.size());
    for (const auto& area : tile.areas) {
        Mesh mesh = MeshBuilder::build_area_mesh(area);
        if (mesh.is_valid()) {
            result.area_meshes.push_back(std::move(mesh));
        }
    }

    return result;
}

bool TileManager::queue_tile_build_async(TileCoord coord) {
    auto* tile = get_tile(coord);
    if (!tile || tile->meshes_built || tile->meshes_pending) {
        return false;
    }

    tile->meshes_pending = true;

    // Capture tile data by copy for thread safety
    Tile tile_copy = *tile;

    std::lock_guard<std::mutex> lock(m_pending_mutex);
    m_pending_builds.push_back({
        coord,
        std::async(std::launch::async, [this, tile_copy]() {
            return build_tile_meshes_internal(tile_copy);
        })
    });

    return true;
}

size_t TileManager::poll_async_builds() {
    std::lock_guard<std::mutex> lock(m_pending_mutex);

    size_t completed = 0;

    // Check each pending build
    for (auto it = m_pending_builds.begin(); it != m_pending_builds.end(); ) {
        // Check if ready (non-blocking)
        if (it->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            // Get the built meshes
            BuiltMeshes meshes = it->future.get();

            // Apply to tile
            auto* tile = get_tile(it->coord);
            if (tile) {
                tile->road_meshes = std::move(meshes.road_meshes);
                tile->building_meshes = std::move(meshes.building_meshes);
                tile->area_meshes = std::move(meshes.area_meshes);
                tile->meshes_built = true;
                tile->meshes_pending = false;
            }

            it = m_pending_builds.erase(it);
            completed++;
        } else {
            ++it;
        }
    }

    return completed;
}

} // namespace stratum::osm
