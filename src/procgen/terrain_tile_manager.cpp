#include "procgen/terrain_tile_manager.hpp"
#include <algorithm>
#include <cmath>

namespace stratum::procgen {

// ============================================================================
// FlattenMask implementation
// ============================================================================

float FlattenMask::sample(float world_x, float world_z) const {
    if (weights.empty()) return 0.0f;
    
    float grid_x = (world_x - origin.x) / cell_size_x;
    float grid_z = (world_z - origin.y) / cell_size_z;
    
    int x0 = static_cast<int>(std::floor(grid_x));
    int z0 = static_cast<int>(std::floor(grid_z));
    int x1 = x0 + 1;
    int z1 = z0 + 1;
    
    x0 = std::clamp(x0, 0, width - 1);
    x1 = std::clamp(x1, 0, width - 1);
    z0 = std::clamp(z0, 0, height - 1);
    z1 = std::clamp(z1, 0, height - 1);
    
    float fx = grid_x - std::floor(grid_x);
    float fz = grid_z - std::floor(grid_z);
    
    float w00 = at(x0, z0);
    float w10 = at(x1, z0);
    float w01 = at(x0, z1);
    float w11 = at(x1, z1);
    
    float w0 = w00 * (1.0f - fx) + w10 * fx;
    float w1 = w01 * (1.0f - fx) + w11 * fx;
    
    return w0 * (1.0f - fz) + w1 * fz;
}

void FlattenMask::set(int x, int z, float value) {
    if (in_bounds(x, z)) {
        weights[z * width + x] = std::clamp(value, 0.0f, 1.0f);
    }
}

float FlattenMask::at(int x, int z) const {
    if (!in_bounds(x, z)) return 0.0f;
    return weights[z * width + x];
}

// ============================================================================
// TerrainTileManager implementation
// ============================================================================

TerrainTileManager::TerrainTileManager() : m_generator(12345) {}

void TerrainTileManager::init(const TerrainTileConfig& config) {
    m_config = config;
    m_generator.reseed(config.terrain.seed);
    
    // Initialize flatten mask to cover world bounds
    float world_width = config.world_max.x - config.world_min.x;
    float world_height = config.world_max.y - config.world_min.y;
    
    // Use same resolution as terrain chunks for the mask
    float mask_cell_size = config.chunk_size / config.chunk_resolution;
    
    m_flatten_mask.width = static_cast<int>(std::ceil(world_width / mask_cell_size)) + 1;
    m_flatten_mask.height = static_cast<int>(std::ceil(world_height / mask_cell_size)) + 1;
    m_flatten_mask.cell_size_x = mask_cell_size;
    m_flatten_mask.cell_size_z = mask_cell_size;
    m_flatten_mask.origin = config.world_min;
    m_flatten_mask.weights.resize(m_flatten_mask.width * m_flatten_mask.height, 0.0f);
}

void TerrainTileManager::clear() {
    m_chunks.clear();
    m_flatten_mask.weights.clear();
    m_flatten_mask.width = 0;
    m_flatten_mask.height = 0;
    m_has_osm_data = false;
}

void TerrainTileManager::set_config(const TerrainTileConfig& config) {
    clear();
    init(config);
}

void TerrainTileManager::import_osm_data(const std::vector<osm::Road>& roads,
                                          const std::vector<osm::Building>& buildings,
                                          const std::vector<osm::Area>& areas) {
    // Reset flatten mask
    std::fill(m_flatten_mask.weights.begin(), m_flatten_mask.weights.end(), 0.0f);
    
    build_flatten_mask(roads, buildings, areas);
    m_has_osm_data = !roads.empty() || !buildings.empty() || !areas.empty();
    
    // Mark affected chunks for regeneration
    for (auto& [coord, chunk] : m_chunks) {
        chunk.heightmap_generated = false;
        chunk.mesh_built = false;
    }
}

void TerrainTileManager::build_flatten_mask(const std::vector<osm::Road>& roads,
                                             const std::vector<osm::Building>& buildings,
                                             const std::vector<osm::Area>& areas) {
    float radius = m_config.osm_flatten_radius;
    float falloff = m_config.osm_blend_distance;
    
    // Paint roads into flatten mask
    for (const auto& road : roads) {
        float road_radius = radius + road.width * 0.5f;
        for (size_t i = 0; i + 1 < road.polyline.size(); ++i) {
            paint_flatten_line(road.polyline[i], road.polyline[i + 1], road_radius, falloff);
        }
    }
    
    // Paint buildings
    for (const auto& building : buildings) {
        paint_flatten_polygon(building.footprint, falloff);
    }
    
    // Paint areas (parks, parking, etc. - but not water)
    for (const auto& area : areas) {
        if (area.type != osm::AreaType::Water) {
            paint_flatten_polygon(area.polygon, falloff);
        }
    }
}

void TerrainTileManager::paint_flatten_point(float world_x, float world_z, float radius, float falloff) {
    float total_radius = radius + falloff;
    
    // Convert to grid coordinates
    int min_x = static_cast<int>(std::floor((world_x - total_radius - m_flatten_mask.origin.x) / m_flatten_mask.cell_size_x));
    int max_x = static_cast<int>(std::ceil((world_x + total_radius - m_flatten_mask.origin.x) / m_flatten_mask.cell_size_x));
    int min_z = static_cast<int>(std::floor((world_z - total_radius - m_flatten_mask.origin.y) / m_flatten_mask.cell_size_z));
    int max_z = static_cast<int>(std::ceil((world_z + total_radius - m_flatten_mask.origin.y) / m_flatten_mask.cell_size_z));
    
    min_x = std::max(0, min_x);
    max_x = std::min(m_flatten_mask.width - 1, max_x);
    min_z = std::max(0, min_z);
    max_z = std::min(m_flatten_mask.height - 1, max_z);
    
    for (int z = min_z; z <= max_z; ++z) {
        for (int x = min_x; x <= max_x; ++x) {
            float cell_x = m_flatten_mask.origin.x + x * m_flatten_mask.cell_size_x;
            float cell_z = m_flatten_mask.origin.y + z * m_flatten_mask.cell_size_z;
            
            float dx = cell_x - world_x;
            float dz = cell_z - world_z;
            float dist = std::sqrt(dx * dx + dz * dz);
            
            float weight = 0.0f;
            if (dist <= radius) {
                weight = 1.0f;
            } else if (dist <= radius + falloff) {
                // Smooth falloff using smoothstep
                float t = (dist - radius) / falloff;
                t = t * t * (3.0f - 2.0f * t);  // smoothstep
                weight = 1.0f - t;
            }
            
            // Max blend - take highest flatten weight
            float current = m_flatten_mask.at(x, z);
            m_flatten_mask.set(x, z, std::max(current, weight));
        }
    }
}

void TerrainTileManager::paint_flatten_line(const glm::dvec2& p0, const glm::dvec2& p1, float radius, float falloff) {
    // Sample points along the line
    double length = glm::length(p1 - p0);
    int samples = std::max(2, static_cast<int>(length / (m_flatten_mask.cell_size_x * 0.5)));
    
    for (int i = 0; i <= samples; ++i) {
        double t = static_cast<double>(i) / samples;
        glm::dvec2 pt = p0 + (p1 - p0) * t;
        paint_flatten_point(static_cast<float>(pt.x), static_cast<float>(pt.y), radius, falloff);
    }
}

void TerrainTileManager::paint_flatten_polygon(const std::vector<glm::dvec2>& polygon, float falloff) {
    if (polygon.size() < 3) return;
    
    // Find polygon bounds
    glm::dvec2 poly_min(std::numeric_limits<double>::max());
    glm::dvec2 poly_max(std::numeric_limits<double>::lowest());
    for (const auto& pt : polygon) {
        poly_min = glm::min(poly_min, pt);
        poly_max = glm::max(poly_max, pt);
    }
    
    // Expand bounds by falloff
    poly_min -= glm::dvec2(falloff);
    poly_max += glm::dvec2(falloff);
    
    // Convert to grid coordinates
    int min_x = static_cast<int>(std::floor((poly_min.x - m_flatten_mask.origin.x) / m_flatten_mask.cell_size_x));
    int max_x = static_cast<int>(std::ceil((poly_max.x - m_flatten_mask.origin.x) / m_flatten_mask.cell_size_x));
    int min_z = static_cast<int>(std::floor((poly_min.y - m_flatten_mask.origin.y) / m_flatten_mask.cell_size_z));
    int max_z = static_cast<int>(std::ceil((poly_max.y - m_flatten_mask.origin.y) / m_flatten_mask.cell_size_z));
    
    min_x = std::max(0, min_x);
    max_x = std::min(m_flatten_mask.width - 1, max_x);
    min_z = std::max(0, min_z);
    max_z = std::min(m_flatten_mask.height - 1, max_z);
    
    // For each cell, compute distance to polygon
    for (int z = min_z; z <= max_z; ++z) {
        for (int x = min_x; x <= max_x; ++x) {
            float cell_x = m_flatten_mask.origin.x + x * m_flatten_mask.cell_size_x;
            float cell_z = m_flatten_mask.origin.y + z * m_flatten_mask.cell_size_z;
            glm::dvec2 cell_pt(cell_x, cell_z);
            
            // Point-in-polygon test (ray casting)
            bool inside = false;
            size_t n = polygon.size();
            for (size_t i = 0, j = n - 1; i < n; j = i++) {
                const auto& pi = polygon[i];
                const auto& pj = polygon[j];
                
                if (((pi.y > cell_pt.y) != (pj.y > cell_pt.y)) &&
                    (cell_pt.x < (pj.x - pi.x) * (cell_pt.y - pi.y) / (pj.y - pi.y) + pi.x)) {
                    inside = !inside;
                }
            }
            
            float weight = 0.0f;
            if (inside) {
                weight = 1.0f;
            } else {
                // Compute distance to polygon edges
                float min_dist = std::numeric_limits<float>::max();
                for (size_t i = 0; i < n; ++i) {
                    size_t j = (i + 1) % n;
                    const auto& pi = polygon[i];
                    const auto& pj = polygon[j];
                    
                    // Distance to line segment
                    glm::dvec2 edge = pj - pi;
                    double edge_len_sq = glm::dot(edge, edge);
                    if (edge_len_sq < 0.0001) continue;
                    
                    double t = std::clamp(glm::dot(cell_pt - pi, edge) / edge_len_sq, 0.0, 1.0);
                    glm::dvec2 closest = pi + edge * t;
                    float dist = static_cast<float>(glm::length(cell_pt - closest));
                    min_dist = std::min(min_dist, dist);
                }
                
                if (min_dist <= falloff) {
                    float t = min_dist / falloff;
                    t = t * t * (3.0f - 2.0f * t);  // smoothstep
                    weight = 1.0f - t;
                }
            }
            
            float current = m_flatten_mask.at(x, z);
            m_flatten_mask.set(x, z, std::max(current, weight));
        }
    }
}

void TerrainTileManager::apply_flatten_mask(Heightmap& heightmap) const {
    if (!m_has_osm_data || m_flatten_mask.weights.empty()) return;
    
    float base_height = m_config.osm_base_height;
    
    for (int z = 0; z < heightmap.height; ++z) {
        for (int x = 0; x < heightmap.width; ++x) {
            float world_x = heightmap.origin.x + x * heightmap.cell_size_x;
            float world_z = heightmap.origin.y + z * heightmap.cell_size_z;
            
            float flatten_weight = m_flatten_mask.sample(world_x, world_z);
            if (flatten_weight > 0.0f) {
                float current_height = heightmap.at(x, z);
                float blended_height = current_height * (1.0f - flatten_weight) + base_height * flatten_weight;
                heightmap.set(x, z, blended_height);
            }
        }
    }
}

TerrainChunkCoord TerrainTileManager::world_to_chunk(const glm::vec2& world_pos) const {
    TerrainChunkCoord coord;
    coord.x = static_cast<int>(std::floor((world_pos.x - m_config.world_min.x) / m_config.chunk_size));
    coord.z = static_cast<int>(std::floor((world_pos.y - m_config.world_min.y) / m_config.chunk_size));
    return coord;
}

void TerrainTileManager::get_chunk_bounds(const TerrainChunkCoord& coord, glm::vec2& out_min, glm::vec2& out_max) const {
    out_min.x = m_config.world_min.x + coord.x * m_config.chunk_size;
    out_min.y = m_config.world_min.y + coord.z * m_config.chunk_size;
    out_max.x = out_min.x + m_config.chunk_size;
    out_max.y = out_min.y + m_config.chunk_size;
}

bool TerrainTileManager::generate_chunk(const TerrainChunkCoord& coord) {
    // Check if chunk is within world bounds
    glm::vec2 chunk_min, chunk_max;
    get_chunk_bounds(coord, chunk_min, chunk_max);
    
    if (chunk_max.x < m_config.world_min.x || chunk_min.x > m_config.world_max.x ||
        chunk_max.y < m_config.world_min.y || chunk_min.y > m_config.world_max.y) {
        return false;
    }
    
    // Get or create chunk
    TerrainChunk& chunk = m_chunks[coord];
    chunk.coord = coord;
    
    // Generate heightmap for this chunk
    chunk.heightmap = m_generator.generate_chunk(
        m_config.terrain,
        chunk_min,
        m_config.chunk_size,
        m_config.chunk_size
    );
    
    // Override resolution to match config
    // (generate_chunk uses config.resolution_x/z which might differ)
    
    // Apply flattening mask for OSM areas
    apply_flatten_mask(chunk.heightmap);
    
    chunk.heightmap_generated = true;
    chunk.mesh_built = false;
    
    return true;
}

void TerrainTileManager::generate_all_chunks() {
    // Calculate chunk grid
    int min_chunk_x = static_cast<int>(std::floor((m_config.world_min.x - m_config.world_min.x) / m_config.chunk_size));
    int max_chunk_x = static_cast<int>(std::ceil((m_config.world_max.x - m_config.world_min.x) / m_config.chunk_size));
    int min_chunk_z = static_cast<int>(std::floor((m_config.world_min.y - m_config.world_min.y) / m_config.chunk_size));
    int max_chunk_z = static_cast<int>(std::ceil((m_config.world_max.y - m_config.world_min.y) / m_config.chunk_size));
    
    for (int z = min_chunk_z; z < max_chunk_z; ++z) {
        for (int x = min_chunk_x; x < max_chunk_x; ++x) {
            generate_chunk({x, z});
        }
    }
}

void TerrainTileManager::generate_visible_chunks(const glm::vec3& camera_pos, float view_distance) {
    glm::vec2 cam_2d(camera_pos.x, -camera_pos.z);  // Note: Z is inverted in rendering
    
    glm::vec2 view_min = cam_2d - glm::vec2(view_distance);
    glm::vec2 view_max = cam_2d + glm::vec2(view_distance);
    
    auto coords = get_chunks_in_bounds(view_min, view_max);
    for (const auto& coord : coords) {
        if (m_chunks.find(coord) == m_chunks.end() || !m_chunks[coord].heightmap_generated) {
            generate_chunk(coord);
        }
    }
}

bool TerrainTileManager::build_chunk_mesh(const TerrainChunkCoord& coord) {
    auto it = m_chunks.find(coord);
    if (it == m_chunks.end() || !it->second.heightmap_generated) {
        return false;
    }
    
    TerrainChunk& chunk = it->second;
    
    // Build terrain mesh
    chunk.terrain_mesh = TerrainMeshBuilder::build_terrain_mesh(chunk.heightmap, m_config.mesh);
    
    // Build water mesh if enabled and water level is set
    if (m_config.mesh.generate_water_mesh) {
        chunk.water_mesh = TerrainMeshBuilder::build_water_mesh(
            chunk.heightmap,
            m_config.terrain.water_level,
            m_config.mesh.water_color
        );
    }
    
    // Update bounds
    chunk.bounds_min = chunk.terrain_mesh.bounds.min;
    chunk.bounds_max = chunk.terrain_mesh.bounds.max;
    
    chunk.mesh_built = true;
    chunk.gpu_uploaded = false;
    
    return true;
}

void TerrainTileManager::build_all_meshes() {
    for (auto& [coord, chunk] : m_chunks) {
        if (chunk.heightmap_generated && !chunk.mesh_built) {
            build_chunk_mesh(coord);
        }
    }
}

TerrainChunk* TerrainTileManager::get_chunk(const TerrainChunkCoord& coord) {
    auto it = m_chunks.find(coord);
    return (it != m_chunks.end()) ? &it->second : nullptr;
}

const TerrainChunk* TerrainTileManager::get_chunk(const TerrainChunkCoord& coord) const {
    auto it = m_chunks.find(coord);
    return (it != m_chunks.end()) ? &it->second : nullptr;
}

std::vector<TerrainChunkCoord> TerrainTileManager::get_all_chunks() const {
    std::vector<TerrainChunkCoord> coords;
    coords.reserve(m_chunks.size());
    for (const auto& [coord, chunk] : m_chunks) {
        coords.push_back(coord);
    }
    return coords;
}

std::vector<TerrainChunkCoord> TerrainTileManager::get_chunks_in_bounds(const glm::vec2& min, const glm::vec2& max) const {
    std::vector<TerrainChunkCoord> coords;
    
    int min_x = static_cast<int>(std::floor((min.x - m_config.world_min.x) / m_config.chunk_size));
    int max_x = static_cast<int>(std::ceil((max.x - m_config.world_min.x) / m_config.chunk_size));
    int min_z = static_cast<int>(std::floor((min.y - m_config.world_min.y) / m_config.chunk_size));
    int max_z = static_cast<int>(std::ceil((max.y - m_config.world_min.y) / m_config.chunk_size));
    
    for (int z = min_z; z <= max_z; ++z) {
        for (int x = min_x; x <= max_x; ++x) {
            coords.push_back({x, z});
        }
    }
    
    return coords;
}

float TerrainTileManager::sample_height(float world_x, float world_z) const {
    TerrainChunkCoord coord = world_to_chunk(glm::vec2(world_x, world_z));
    const TerrainChunk* chunk = get_chunk(coord);
    
    if (chunk && chunk->heightmap_generated) {
        return chunk->heightmap.sample(world_x, world_z);
    }
    
    return m_config.osm_base_height;
}

size_t TerrainTileManager::generated_count() const {
    size_t count = 0;
    for (const auto& [coord, chunk] : m_chunks) {
        if (chunk.heightmap_generated) ++count;
    }
    return count;
}

size_t TerrainTileManager::mesh_count() const {
    size_t count = 0;
    for (const auto& [coord, chunk] : m_chunks) {
        if (chunk.mesh_built) ++count;
    }
    return count;
}

} // namespace stratum::procgen
