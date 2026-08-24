#pragma once

#include "osm/types.hpp"
#include "renderer/mesh.hpp"
#include <vector>

namespace stratum::osm {

/**
 * @brief Per-feature mesh construction for buildings and areas
 *
 * Roads are NOT built here. Road geometry is topology-driven and has to be
 * solved against the whole network at once, so it is produced by
 * stratum::osm::road::RoadNetworkBuilder and handed to the spatial index as
 * finished pieces via QuadTree::assign_road_pieces(). The old per-feature
 * build_road_mesh() and the endpoint-clustering build_junction_meshes() were
 * removed with that change; see docs/plans/road_network_plan.md, P0.2.
 */
class MeshBuilder {
public:
    MeshBuilder() = default;

    /**
     * @brief Generates a mesh for a building
     */
    static Mesh build_building_mesh(const Building& building);

    /**
     * @brief Generates a mesh for an area (park, water, etc.)
     */
    static Mesh build_area_mesh(const Area& area);

    /**
     * @brief Merge multiple meshes into a single mesh
     *
     * Combines all vertices and indices (with proper index offsetting) into
     * one mesh to reduce draw calls. Per-vertex colors are preserved.
     */
    static Mesh merge_meshes(const std::vector<Mesh>& meshes);

private:

};

} // namespace stratum::osm
