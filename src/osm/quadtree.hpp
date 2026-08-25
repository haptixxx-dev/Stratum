#pragma once

#include "osm/road/lod_chunk.hpp"
#include "osm/road/road_network_builder.hpp"
#include "geometry/ambient_occlusion.hpp"
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
    //
    // road_meshes is NOT built from this node's `roads`. It is filled once by
    // QuadTree::assign_road_pieces() from geometry the road network builder
    // solved against the whole graph, and the per-leaf mesh build never touches
    // it. See assign_road_pieces().
    std::vector<Mesh> road_meshes;
    std::vector<Mesh> building_meshes;
    std::vector<Mesh> area_meshes;

    /**
     * @brief Chunk-level LOD chain over this leaf's road geometry
     *
     * Built by QuadTree::assign_road_pieces() when chunk LOD is enabled, from the
     * merged mesh that road_meshes held, with this leaf's own rectangle as the
     * chunk rectangle. `levels[0]` supersedes road_meshes exactly -- it is the
     * same triangles welded and reordered -- so road_meshes is CLEARED once the
     * chain exists and the draw path reads levels[road_lod_resident] instead.
     * Holding both would double the CPU-side cost of the whole network, which is
     * the one thing lod_chunk.hpp warns about.
     *
     * Empty when chunk LOD is off, when the leaf received no road geometry, or
     * when build_chunk_lod() found nothing usable. In all three cases road_meshes
     * is the geometry and is drawn as before.
     */
    road::ChunkLod road_lod;

    /**
     * @brief Which level of road_lod is on the GPU, or -1 for none
     *
     * Only ONE level is ever resident. Uploading the whole chain and choosing at
     * draw time would spend more memory than no LOD at all, which is the opposite
     * of the point; the level is chosen from the camera distance and swapped
     * through the same release/upload path a leaf already uses.
     *
     * Reset to -1 by every path that releases the leaf's road handles, so a leaf
     * that lost its geometry to eviction re-uploads the level it should have
     * rather than believing a stale one is still there.
     */
    int road_lod_resident = -1;

    /// True when road_lod carries at least one level, so the LOD path owns the roads
    bool has_road_lod() const { return !road_lod.levels.empty(); }

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

/**
 * @brief Fraction a switch distance is overshot or undershot before a level changes
 *
 * A swap is not free: it releases one device buffer range and uploads another,
 * and a camera parked on a switch distance would do that every frame for as long
 * as it sat there. Widening the threshold when coarsening and narrowing it when
 * refining leaves a band around each switch distance in which whichever level is
 * already resident stays resident.
 */
inline constexpr float kRoadLodHysteresis = 0.15f;

/**
 * @brief Which level of @p lod to draw at @p distance
 *
 * ChunkLod::switch_distances is ascending with a leading zero, so the answer is
 * the last entry the distance has reached. @p current biases that with
 * kRoadLodHysteresis so a level already resident is kept across the threshold it
 * sits on.
 *
 * @param lod      The chain; an empty one selects level 0, which does not exist
 *                 and which the caller must not upload
 * @param distance Camera distance to the chunk, metres
 * @param current  Level currently resident, or -1 for none
 * @param scale    User multiplier on every switch distance; 1 is the chain's own
 *                 suggestion, larger holds detail further out
 * @return Level index in [0, lod.levels.size())
 */
[[nodiscard]] int select_road_lod_level(const road::ChunkLod& lod, float distance,
                                        int current, float scale = 1.0f);

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

    /**
     * @brief Take ownership of prebuilt road geometry and route it to leaves
     *
     * Road geometry is solved once against the whole road graph, not per leaf,
     * because junctions, miters and profile transitions are topology and
     * topology does not stop at a leaf boundary. This is the hand-off point:
     * the spatial index receives finished triangles and never sees a Road
     * again.
     *
     * There are two routings, and set_chunk_lod() picks between them.
     *
     * ### Chunk LOD off: route each piece whole, by its anchor
     *
     * Each piece is routed by its RoadPiece::anchor -- a point that lies on the
     * road -- down to the leaf containing that point, and appended into that
     * leaf's road_meshes with its MaterialKey ranges preserved. A piece is
     * indivisible: whole triangles only, never split across leaves. A road
     * therefore may hang over the boundary of the leaf that owns it. That is
     * correct and expected, and the leaf's AABB is grown to cover the overhang
     * so frustum culling does not clip it.
     *
     * A P4 JUNCTION piece is routed by the same rule and by nothing else. It
     * carries `RoadPiece::edge == road::kInvalidId`, because a junction spans
     * several edges and belongs to none of them, and is anchored at the graph
     * node -- so this function never reads `edge` at all. A junction straddling a
     * leaf boundary is the overhang case again, not a new one: it lands whole in
     * the leaf containing its node, and the AABB growth that covers a road
     * hanging over the boundary covers a fillet hanging over it too.
     *
     * ### Chunk LOD on: route each TRIANGLE, by its centroid
     *
     * Anchor routing cannot be simplified without cracking, and that is not a
     * tuning problem. build_chunk_lod() pins a vertex by its distance to the
     * chunk RECTANGLE, so its crack-free guarantee is conditional on a piece's
     * geometry lying inside the rectangle it was assigned to, up to the band. An
     * anchor-routed piece breaks that condition by construction: an edge between
     * two junctions is anchored at its own mid-arclength, so half its length
     * hangs outside the leaf, and on the Lucan extract that overhang is tens of
     * metres against a leaf 60 to 120 m across. Geometry that far outside the
     * rectangle reads as interior, is free to move, and pulls away from the
     * neighbouring chunk that shares those vertices and never knew about it.
     *
     * So with LOD on, a piece is split: every triangle goes to the leaf holding
     * its own centroid, keeping its MaterialKey. Nothing is clipped and no
     * triangle is duplicated -- assign-once, at triangle granularity rather than
     * at piece granularity -- so the triangle count is preserved exactly and the
     * only new boundaries are the leaf lines themselves.
     *
     * ### Why no seam can crack
     *
     * Write R_out(A) for the furthest any vertex of chunk A's own geometry lies
     * OUTSIDE A's rectangle, and R_in(A) for the furthest inside A's rectangle
     * that any OTHER chunk's geometry reaches. Both are measured, not assumed,
     * by measure_seam_bands(), and the band handed to build_chunk_lod() for A is
     * `max(R_out(A), R_in(A))` (floored at ChunkLodConfig::border_band).
     *
     * Take any vertex position v carried by both chunk A and chunk B. Leaf
     * rectangles tile the plane and do not overlap, so v is inside at most one of
     * them.
     *
     *  - If v is outside A's rectangle, it is a vertex of A's geometry lying
     *    outside A, so its distance to A's perimeter is at most R_out(A) and it
     *    is locked in A.
     *  - If v is inside A's rectangle, then it is outside B's rectangle, and v is
     *    also a vertex of B's geometry -- so v is one of the intrusions into A
     *    that R_in(A) measures, and its distance to A's perimeter is at most
     *    R_in(A). Locked in A again.
     *
     * The same two cases with A and B exchanged lock it in B. A vertex locked in
     * both chunks neither moves nor disappears at any level -- lod_chunk.hpp's
     * ring dilation and component anchoring are what make "locked" mean
     * "present", and that is part of its contract -- so the two chunks meet
     * exactly at every level of every chain. Nothing is exchanged between chunks
     * and they are built in parallel.
     *
     * Triangle routing is what keeps those bands small: R is then bounded by the
     * reach of a single straddling triangle rather than by half a piece.
     * RoadLodStats::max_seam_band reports the widest band any chunk needed, and a
     * large value there means reduction is being paid for tolerance -- see
     * ChunkLodConfig::border_band for that trade.
     *
     * Call AFTER init() and assign_data(), so the leaves the pieces route to
     * already exist, and BEFORE any node mesh build is queued. Pieces are moved
     * from, so @p pieces is left empty.
     *
     * @param pieces Output of road::RoadNetworkBuilder::build(); consumed.
     */
    void assign_road_pieces(std::vector<road::RoadPiece>&& pieces);

    /**
     * @brief Statistics of the last chunk-LOD build
     *
     * All zero when chunk LOD is off or no road geometry has been assigned.
     */
    struct RoadLodStats {
        /// Leaves that received road geometry
        size_t chunks = 0;
        /// Leaves that came out of build_chunk_lod() with at least one level
        size_t chunks_with_lod = 0;
        /// Triangles at each level, summed over every chunk; index 0 is full detail
        std::vector<size_t> triangles_per_level;
        /// Vertices at each level, summed over every chunk
        std::vector<size_t> vertices_per_level;
        /// Chunks that produced a level at each index
        std::vector<size_t> chunks_per_level;
        /// Triangles handed in, before the merge welded the seams between pieces
        size_t triangles_in = 0;
        /// Vertices handed in, before the merge welded the seams between pieces
        size_t vertices_in = 0;
        /// Widest seam band any chunk needed, metres; see seam_band_for()
        double max_seam_band = 0.0;
        /// Triangles whose vertices reached outside the leaf that owns them
        size_t straddling_triangles = 0;
        /// Wall-clock time of the whole chunk-LOD build, milliseconds
        double build_ms = 0.0;
    };

    /**
     * @brief Turn chunk-level LOD on or off, and configure it
     *
     * Must be called BEFORE assign_road_pieces(); the chain is built there and
     * nowhere else. Enabling it also changes how road geometry is routed -- see
     * assign_road_pieces() -- so the two are one switch rather than two.
     *
     * @param enabled Build the chain
     * @param cfg     Ratios and error bound. ChunkLodConfig::border_band is a
     *                FLOOR: the real band is measured per chunk, because a
     *                triangle straddling a leaf boundary reaches further than any
     *                fixed band would.
     */
    void set_chunk_lod(bool enabled, const road::ChunkLodConfig& cfg);

    /**
     * @brief Settings for the per-vertex ambient occlusion bake on building meshes
     *
     * Baked once per leaf, on the MERGED mesh, so the buildings in a leaf occlude
     * each other -- which is where most of the useful darkening comes from, and
     * which baking per building before the merge would lose.
     *
     * Set AOSettings::strength to 0 to skip the bake entirely; every vertex then
     * keeps the 1.0 that means "no occlusion".
     */
    void set_building_ao(const stratum::geometry::AOSettings& settings) { m_building_ao = settings; }
    [[nodiscard]] const stratum::geometry::AOSettings& building_ao() const { return m_building_ao; }

    /// Whether the chain will be built by the next assign_road_pieces()
    bool chunk_lod_enabled() const { return m_chunk_lod_enabled; }

    /// Statistics of the last chunk-LOD build
    const RoadLodStats& road_lod_stats() const { return m_road_lod_stats; }

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
    /**
     * @brief Ambient occlusion applied to building meshes as they are built
     *
     * Defaults are tuned for building scale, and for IMPORT TIME: 8 rays is
     * already smooth on a flat wall, and the bake runs on whichever worker thread
     * is building the leaf, so it asks for one thread rather than fighting the
     * pool it is inside.
     */
    stratum::geometry::AOSettings m_building_ao = [] {
        stratum::geometry::AOSettings settings;
        settings.ray_count = 8;
        settings.max_distance = 10.0f;
        settings.min_ao = 0.25f;
        settings.use_ground_plane = true;
        settings.thread_count = 1;
        return settings;
    }();

    void subdivide(QuadTreeNode* node);
    void insert_road(QuadTreeNode* node, const Road& road);
    void insert_building(QuadTreeNode* node, const Building& building);
    void insert_area(QuadTreeNode* node, const Area& area);
    int child_index(const QuadTreeNode* node, const glm::dvec2& point) const;
    QuadTreeNode* find_leaf(const glm::dvec2& point);
    void compute_3d_bounds(QuadTreeNode* node);
    void recompute_bounds(QuadTreeNode* node);

    /// Route each piece whole, by its anchor, into one leaf. The no-LOD path.
    void assign_pieces_by_anchor(std::vector<road::RoadPiece>& pieces,
                                 std::vector<QuadTreeNode*>& touched);

    /// Route each TRIANGLE by its centroid. The LOD path; see assign_road_pieces().
    void assign_triangles_by_centroid(std::vector<road::RoadPiece>& pieces,
                                      std::vector<QuadTreeNode*>& touched);

    /**
     * @brief The lock band each chunk needs so no seam vertex is ever free
     *
     * See assign_road_pieces() for the argument this implements.
     *
     * @param touched   Leaves holding road geometry
     * @param out_bands Band per entry of @p touched, metres
     * @return The widest band, metres
     */
    double measure_seam_bands(const std::vector<QuadTreeNode*>& touched,
                              std::vector<double>& out_bands);

    /// Build one ChunkLod per touched leaf and drop the merged mesh it replaced
    void build_chunk_lods(const std::vector<QuadTreeNode*>& touched);

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
    // Roads are absent by design: they are assigned whole by
    // assign_road_pieces() and must survive every later per-leaf build.
    struct BuiltMeshes {
        std::vector<Mesh> building_meshes;
        std::vector<Mesh> area_meshes;
    };
    BuiltMeshes build_node_meshes_internal(const QuadTreeNode& node);

    std::unique_ptr<QuadTreeNode> m_root;
    uint32_t m_next_id = 0;

    bool m_chunk_lod_enabled = false;
    road::ChunkLodConfig m_chunk_lod_config;
    RoadLodStats m_road_lod_stats;

    // Async build tracking
    struct PendingBuild {
        QuadTreeNode* node;
        std::future<BuiltMeshes> future;
    };
    std::vector<PendingBuild> m_pending_builds;
    std::mutex m_pending_mutex;
};

} // namespace stratum::osm
