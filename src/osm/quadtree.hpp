#pragma once

#include "osm/types.hpp"
#include "renderer/mesh.hpp"
#include <glm/glm.hpp>
#include <array>
#include <vector>
#include <memory>
#include <functional>
#include <future>
#include <mutex>
#include <cstdint>

namespace stratum::osm {

struct QuadTreeConfig {
    static constexpr int MAX_DEPTH = 8;
    static constexpr int MAX_FEATURES_PER_LEAF = 64;
    static constexpr double MIN_NODE_SIZE = 30.0;
    static constexpr float CONTRIBUTION_THRESHOLD_DEFAULT = 4.0f;
};

struct QuadTreeNode {
    // Spatial region (2D, in local meters)
    glm::dvec2 center{0.0};
    double half_size = 0.0;

    // 3D AABB for frustum culling
    glm::vec3 bounds_min{0.0f};
    glm::vec3 bounds_max{0.0f};

    // Children: NW=0, NE=1, SW=2, SE=3 (null if leaf)
    std::unique_ptr<QuadTreeNode> children[4];

    // Features (leaf nodes only)
    std::vector<Road> roads;
    std::vector<Building> buildings;
    std::vector<Area> areas;

    // Merged meshes (one per category)
    std::vector<Mesh> road_meshes;
    std::vector<Mesh> building_meshes;
    std::vector<Mesh> area_meshes;

    // GPU mesh IDs
    std::vector<uint32_t> road_gpu_ids;
    std::vector<uint32_t> building_gpu_ids;
    std::vector<uint32_t> area_gpu_ids;

    bool gpu_uploaded = false;
    bool meshes_built = false;
    bool meshes_pending = false;

    uint32_t node_id = 0;
    uint8_t depth = 0;

    bool is_leaf() const { return !children[0] && !children[1] && !children[2] && !children[3]; }

    bool has_valid_bounds() const {
        return bounds_min.x < bounds_max.x || bounds_min.z < bounds_max.z;
    }

    size_t feature_count() const {
        return roads.size() + buildings.size() + areas.size();
    }
};

// Visitor receives (node pointer, distance squared from camera)
using QuadTreeVisitor = std::function<void(QuadTreeNode*, float dist_sq)>;

class QuadTree {
public:
    QuadTree() = default;

    /// Size the root from the raw geographic bounds, centred on the projection
    /// origin. Only correct when the bounds tightly enclose the imported features.
    void init(const BoundingBox& bounds);

    /// Size the root from the local-space extent of the features that will
    /// actually be stored, centred on them.
    ///
    /// Prefer this. ParsedOSMData::bounds is expanded by every raw node, including
    /// the ones Overpass pulls in because a way or relation references them, which
    /// can be hundreds of kilometres outside the queried area. Sizing the root from
    /// those bounds pushes the real geometry into a corner far from the origin and
    /// leaves MAX_DEPTH unable to subdivide down to a useful leaf size.
    void init(const ParsedOSMData& data);
    void clear();
    void assign_data(const ParsedOSMData& data);

    // Hierarchical traversal: frustum + distance + contribution culling
    // Collects visible leaves sorted front-to-back, then visits them
    void traverse_visible(
        const std::array<glm::vec4, 6>& frustum_planes,
        const glm::vec3& cam_pos,
        float view_radius,
        float screen_height,
        float fov_y,
        float contribution_threshold,
        bool use_frustum_culling,
        bool use_distance_culling,
        bool use_contribution_culling,
        const QuadTreeVisitor& visitor
    );

    // Async mesh building
    bool queue_node_build_async(QuadTreeNode* node);
    size_t poll_async_builds();

    // Accessors
    std::vector<QuadTreeNode*> get_all_leaves();
    void get_bounds(glm::vec3& out_min, glm::vec3& out_max) const;

    /// Where the data actually is: the centroid of populated leaves, weighted by
    /// feature count, with the radius that encloses most of that mass.
    ///
    /// Use this for framing rather than the centre of get_bounds(). A handful of
    /// far-flung features stretches the bounding box enormously -- an Overpass
    /// export of Dublin can span 123km with its box centre out in the Atlantic --
    /// but weighting by feature count puts the focus on the bulk of the geometry.
    /// Returns false if there is no populated leaf.
    bool get_focus(glm::vec3& out_centre, float& out_radius);
    size_t leaf_count() const;
    size_t total_roads() const;
    size_t total_buildings() const;
    size_t total_areas() const;
    uint8_t max_depth() const;

    QuadTreeNode* root() { return m_root.get(); }

private:
    void subdivide(QuadTreeNode* node);
    void insert_road(QuadTreeNode* node, const Road& road);
    void insert_building(QuadTreeNode* node, const Building& building);
    void insert_area(QuadTreeNode* node, const Area& area);
    int child_index(const QuadTreeNode* node, const glm::dvec2& point) const;
    void compute_3d_bounds(QuadTreeNode* node);

    void traverse_recursive(
        QuadTreeNode* node,
        const std::array<glm::vec4, 6>& frustum_planes,
        const glm::vec3& cam_pos,
        float radius_sq,
        float screen_height,
        float fov_y,
        float contribution_threshold,
        bool use_frustum,
        bool use_distance,
        bool use_contribution,
        std::vector<std::pair<QuadTreeNode*, float>>& visible
    );

    bool frustum_intersects_aabb(const std::array<glm::vec4, 6>& planes,
                                  const glm::vec3& mn, const glm::vec3& mx) const;

    void collect_leaves(QuadTreeNode* node, std::vector<QuadTreeNode*>& out);
    void count_leaves(const QuadTreeNode* node, size_t& count) const;
    void count_features(const QuadTreeNode* node, size_t& roads, size_t& buildings, size_t& areas) const;
    void find_max_depth(const QuadTreeNode* node, uint8_t& depth) const;

    // Mesh building (thread-safe internals)
    struct BuiltMeshes {
        std::vector<Mesh> road_meshes;
        std::vector<Mesh> building_meshes;
        std::vector<Mesh> area_meshes;
    };
    BuiltMeshes build_node_meshes_internal(const QuadTreeNode& node);

    std::unique_ptr<QuadTreeNode> m_root;
    uint32_t m_next_id = 0;

    // Async build tracking
    struct PendingBuild {
        QuadTreeNode* node;
        std::future<BuiltMeshes> future;
    };
    std::vector<PendingBuild> m_pending_builds;
    std::mutex m_pending_mutex;
};

} // namespace stratum::osm
