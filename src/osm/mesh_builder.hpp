#pragma once

#include "osm/types.hpp"
#include "renderer/mesh.hpp"
#include <vector>

namespace stratum::osm {

class MeshBuilder {
public:
    MeshBuilder() = default;

    /**
     * @brief Generates a mesh for a road segment
     */
    static Mesh build_road_mesh(const Road& road);

    /**
     * @brief Generates a mesh for a building
     */
    static Mesh build_building_mesh(const Building& building);
    
    /**
     * @brief Generates a mesh for an area (park, water, etc.)
     */
    static Mesh build_area_mesh(const Area& area);

private:
   
};

} // namespace stratum::osm
