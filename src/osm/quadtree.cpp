#include "osm/quadtree.hpp"
#include "osm/mesh_builder.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <algorithm>
#include <limits>

namespace stratum::osm {

namespace {

/**
 * @brief Append @p src into @p dst, keeping @p src's material ranges intact
 *
 * Mesh::append() attributes everything it copies to a single MaterialId, which
 * would flatten a road piece's asphalt, curb and sidewalk ranges into one slot
 * and undo the whole point of P0.3. This walks the source's effective ranges
 * instead and extends or opens a matching range in the destination, so the
 * material slots survive the merge of many pieces into one leaf mesh.
 *
 * Submesh bookkeeping otherwise follows Mesh::append() exactly: a destination
 * that has indices but no submeshes first materialises its implicit whole-mesh
 * range, so pre-existing geometry keeps its identity.
 */
void append_preserving_materials(Mesh& dst, const Mesh& src) {
    if (src.vertices.empty() || src.indices.empty()) return;

    if (!dst.indices.empty() && dst.submeshes.empty()) {
        dst.submeshes.push_back({0u, static_cast<uint32_t>(dst.indices.size()),
                                 MaterialId::Default});
    }

    const uint32_t base_vertex = static_cast<uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (const auto& v : src.vertices) {
        dst.bounds.expand(v.position);
    }

    for (const SubMesh& range : src.effective_submeshes()) {
        if (range.index_count == 0) continue;

        const size_t end = static_cast<size_t>(range.index_offset) + range.index_count;
        if (end > src.indices.size()) continue;  // malformed range; drop rather than over-read

        const uint32_t start = static_cast<uint32_t>(dst.indices.size());

        // Grow geometrically, never to the exact size needed. reserve() allocates
        // EXACTLY what is asked for, and the loop below then fills the buffer to
        // capacity precisely, so an exact reserve per range reallocates and copies
        // the whole leaf index buffer on every single append -- quadratic in the
        // ranges routed to one leaf, and seconds of frozen UI on a city extract.
        const size_t needed = dst.indices.size() + range.index_count;
        if (needed > dst.indices.capacity()) {
            dst.indices.reserve(std::max(needed, dst.indices.capacity() * 2));
        }
        for (size_t i = range.index_offset; i < end; ++i) {
            dst.indices.push_back(src.indices[i] + base_vertex);
        }

        SubMesh* back = dst.submeshes.empty() ? nullptr : &dst.submeshes.back();
        if (back && back->material == range.material &&
            back->index_offset + back->index_count == start) {
            back->index_count += range.index_count;
        } else {
            dst.submeshes.push_back({start, range.index_count, range.material});
        }
    }
}

} // namespace

// ============================================================================
// Initialization
// ============================================================================

void QuadTree::init(const BoundingBox& bounds) {
    clear();

    if (!bounds.is_valid()) {
        spdlog::warn("QuadTree: Invalid bounds, cannot initialize");
        return;
    }

    double width = bounds.width_meters();
    double height = bounds.height_meters();

    // Use half the larger dimension to make a square root node
    double half = std::max(width, height) / 2.0;

    m_root = std::make_unique<QuadTreeNode>();
    m_root->center = glm::dvec2(0.0, 0.0); // local coords are centered
    m_root->half_size = half;
    m_root->node_id = m_next_id++;
    m_root->depth = 0;
    compute_3d_bounds(m_root.get());

    spdlog::info("QuadTree: Initialized root node, half_size={:.0f}m", half);
}

void QuadTree::init(const ParsedOSMData& data) {
    clear();

    glm::dvec2 mn(std::numeric_limits<double>::max());
    glm::dvec2 mx(std::numeric_limits<double>::lowest());
    bool any = false;

    const auto accumulate = [&](const std::vector<glm::dvec2>& pts) {
        for (const auto& p : pts) {
            mn = glm::min(mn, p);
            mx = glm::max(mx, p);
            any = true;
        }
    };

    for (const auto& r : data.roads) accumulate(r.polyline);
    for (const auto& b : data.buildings) {
        accumulate(b.footprint);
        for (const auto& h : b.holes) accumulate(h);
    }
    for (const auto& a : data.areas) {
        accumulate(a.polygon);
        for (const auto& h : a.holes) accumulate(h);
    }

    if (!any) {
        spdlog::warn("QuadTree: no feature geometry, cannot initialize");
        return;
    }

    const glm::dvec2 centre = (mn + mx) * 0.5;
    // Square root node, with a small margin so features exactly on the boundary
    // still land inside it.
    double half = std::max(mx.x - mn.x, mx.y - mn.y) * 0.5 * 1.01;
    half = std::max(half, QuadTreeConfig::MIN_NODE_SIZE);

    m_root = std::make_unique<QuadTreeNode>();
    m_root->center = centre;
    m_root->half_size = half;
    m_root->node_id = m_next_id++;
    m_root->depth = 0;
    compute_3d_bounds(m_root.get());

    spdlog::info("QuadTree: root centred on ({:.0f}, {:.0f}), half_size={:.0f}m "
                 "(feature extent {:.0f} x {:.0f}m)",
                 centre.x, centre.y, half, mx.x - mn.x, mx.y - mn.y);
}

bool QuadTree::get_focus(glm::vec3& out_centre, float& out_radius) {
    std::vector<QuadTreeNode*> leaves;
    collect_leaves(m_root.get(), leaves);

    // Feature-count-weighted mean of leaf centres.
    glm::dvec3 accum(0.0);
    double total = 0.0;
    for (const auto* leaf : leaves) {
        const double w = static_cast<double>(leaf->feature_count());
        if (w <= 0.0) continue;
        const glm::vec3 c = (leaf->bounds_min + leaf->bounds_max) * 0.5f;
        accum += glm::dvec3(c) * w;
        total += w;
    }
    if (total <= 0.0) return false;

    const glm::dvec3 centre = accum / total;
    out_centre = glm::vec3(centre);

    // Radius covering the bulk of the mass: grow outward until most features are
    // enclosed, so a few distant strays cannot drag the framing out to the horizon.
    constexpr double MASS_FRACTION = 0.9;

    std::vector<std::pair<double, double>> by_distance;  // (distance, weight)
    by_distance.reserve(leaves.size());
    for (const auto* leaf : leaves) {
        const double w = static_cast<double>(leaf->feature_count());
        if (w <= 0.0) continue;
        const glm::vec3 c = (leaf->bounds_min + leaf->bounds_max) * 0.5f;
        const glm::dvec3 d = glm::dvec3(c) - centre;
        by_distance.emplace_back(std::sqrt(d.x * d.x + d.z * d.z), w);
    }
    std::sort(by_distance.begin(), by_distance.end());

    double running = 0.0;
    double radius = 0.0;
    for (const auto& [dist, w] : by_distance) {
        running += w;
        radius = dist;
        if (running >= total * MASS_FRACTION) break;
    }

    out_radius = static_cast<float>(std::max(radius, QuadTreeConfig::MIN_NODE_SIZE));
    return true;
}

void QuadTree::clear() {
    {
        std::lock_guard<std::mutex> lock(m_pending_mutex);
        m_pending_builds.clear();
    }
    m_root.reset();
    m_next_id = 0;
}

// ============================================================================
// Data Assignment
// ============================================================================

void QuadTree::assign_data(const ParsedOSMData& data) {
    if (!m_root) return;

    for (const auto& road : data.roads) {
        if (!road.polyline.empty()) {
            insert_road(m_root.get(), road);
        }
    }

    for (const auto& building : data.buildings) {
        if (!building.footprint.empty()) {
            insert_building(m_root.get(), building);
        }
    }

    for (const auto& area : data.areas) {
        if (!area.polygon.empty()) {
            insert_area(m_root.get(), area);
        }
    }

    // Recompute 3D bounds bottom-up after all insertions
    // (bounds need to encompass actual building heights)
    recompute_bounds(m_root.get());

    spdlog::info("QuadTree: Assigned data — {} leaves, {} roads, {} buildings, {} areas",
                 leaf_count(), total_roads(), total_buildings(), total_areas());
}

// ============================================================================
// Road Geometry Assignment
// ============================================================================

void QuadTree::assign_road_pieces(std::vector<road::RoadPiece>&& pieces) {
    if (!m_root) {
        spdlog::warn("QuadTree: assign_road_pieces() before init(); {} road pieces dropped",
                     pieces.size());
        pieces.clear();
        return;
    }

    // This is the only writer of QuadTreeNode::road_meshes, so an empty
    // road_meshes means the leaf has not been touched yet on this pass. That is
    // what makes the first-touch test below O(1).
    std::vector<QuadTreeNode*> touched;
    size_t assigned = 0;
    size_t skipped = 0;
    size_t triangles = 0;

    for (auto& piece : pieces) {
        if (piece.mesh.vertices.empty() || piece.mesh.indices.empty()) {
            ++skipped;
            continue;
        }

        // Route by the anchor only. The piece is never split, so part of the
        // road may lie outside this leaf; recompute_bounds() below grows the
        // leaf AABB to cover it.
        //
        // RoadPiece::edge is deliberately not consulted. A junction piece carries
        // kInvalidId and is anchored at its graph node, so branching on
        // provenance here would be a way to get junctions wrong and no way to get
        // anything right.
        QuadTreeNode* leaf = find_leaf(piece.anchor);
        if (!leaf) {
            ++skipped;
            continue;
        }

        if (leaf->road_meshes.empty()) {
            leaf->road_meshes.emplace_back();
            touched.push_back(leaf);
        }

        append_preserving_materials(leaf->road_meshes.front(), piece.mesh);
        triangles += piece.mesh.indices.size() / 3;
        ++assigned;

        // Release the source as it is consumed, so peak memory holds one copy of
        // the road network rather than two. Mesh::clear() empties the vectors but
        // keeps their capacity, which releases nothing: the whole source network
        // would stay allocated beside the copy being built until pieces.clear()
        // below. Move-assigning a fresh Mesh frees the storage instead.
        piece.mesh = Mesh{};
    }

    // Pieces arrive interleaved by material, so each leaf mesh now holds one
    // range per material per piece. Collapse them to one range per material so
    // the leaf costs a handful of draw calls rather than thousands.
    for (QuadTreeNode* leaf : touched) {
        leaf->road_meshes.front().sort_submeshes_by_material();
    }

    // A piece may overhang the leaf that owns it, and a leaf that received one
    // may sit outside the AABB assign_data() computed from features alone.
    recompute_bounds(m_root.get());

    pieces.clear();

    spdlog::info("QuadTree: assigned {} road pieces ({} triangles) to {} leaves, {} skipped",
                 assigned, triangles, touched.size(), skipped);
}

// ============================================================================
// Feature Insertion
// ============================================================================

int QuadTree::child_index(const QuadTreeNode* node, const glm::dvec2& point) const {
    // NW=0, NE=1, SW=2, SE=3
    int idx = 0;
    if (point.x >= node->center.x) idx |= 1; // East
    if (point.y < node->center.y) idx |= 2;  // South
    return idx;
}

QuadTreeNode* QuadTree::find_leaf(const glm::dvec2& point) {
    QuadTreeNode* node = m_root.get();
    if (!node) return nullptr;

    // Same descent insert_road() uses, minus the insertion: walk children by
    // quadrant until a leaf. child_index() clamps nothing, so a point outside
    // the root still resolves to the nearest boundary leaf rather than falling
    // out of the tree.
    while (!node->is_leaf()) {
        QuadTreeNode* child = node->children[child_index(node, point)].get();
        if (!child) break;
        node = child;
    }
    return node;
}

void QuadTree::subdivide(QuadTreeNode* node) {
    double quarter = node->half_size / 2.0;

    // NW=0, NE=1, SW=2, SE=3
    glm::dvec2 offsets[4] = {
        {-quarter,  quarter}, // NW
        { quarter,  quarter}, // NE
        {-quarter, -quarter}, // SW
        { quarter, -quarter}, // SE
    };

    for (int i = 0; i < 4; i++) {
        node->children[i] = std::make_unique<QuadTreeNode>();
        node->children[i]->center = node->center + offsets[i];
        node->children[i]->half_size = quarter;
        node->children[i]->node_id = m_next_id++;
        node->children[i]->depth = node->depth + 1;
        compute_3d_bounds(node->children[i].get());
    }

    // Redistribute features from parent to children by centroid
    for (auto& road : node->roads) {
        glm::dvec2 centroid(0.0);
        for (const auto& pt : road.polyline) centroid += pt;
        centroid /= static_cast<double>(road.polyline.size());
        int idx = child_index(node, centroid);
        node->children[idx]->roads.push_back(std::move(road));
    }
    node->roads.clear();
    node->roads.shrink_to_fit();

    for (auto& building : node->buildings) {
        glm::dvec2 centroid(0.0);
        for (const auto& pt : building.footprint) centroid += pt;
        centroid /= static_cast<double>(building.footprint.size());
        int idx = child_index(node, centroid);
        node->children[idx]->buildings.push_back(std::move(building));
    }
    node->buildings.clear();
    node->buildings.shrink_to_fit();

    for (auto& area : node->areas) {
        glm::dvec2 centroid(0.0);
        for (const auto& pt : area.polygon) centroid += pt;
        centroid /= static_cast<double>(area.polygon.size());
        int idx = child_index(node, centroid);
        node->children[idx]->areas.push_back(std::move(area));
    }
    node->areas.clear();
    node->areas.shrink_to_fit();
}

void QuadTree::insert_road(QuadTreeNode* node, const Road& road) {
    if (!node) return;

    // Compute centroid
    glm::dvec2 centroid(0.0);
    for (const auto& pt : road.polyline) centroid += pt;
    centroid /= static_cast<double>(road.polyline.size());

    // If internal node, route to correct child
    if (!node->is_leaf()) {
        int idx = child_index(node, centroid);
        insert_road(node->children[idx].get(), road);
        return;
    }

    // Leaf node — add feature
    node->roads.push_back(road);

    // Check if we need to subdivide
    if (node->feature_count() > QuadTreeConfig::MAX_FEATURES_PER_LEAF &&
        node->half_size > QuadTreeConfig::MIN_NODE_SIZE &&
        node->depth < QuadTreeConfig::MAX_DEPTH) {
        subdivide(node);
    }
}

void QuadTree::insert_building(QuadTreeNode* node, const Building& building) {
    if (!node) return;

    glm::dvec2 centroid(0.0);
    for (const auto& pt : building.footprint) centroid += pt;
    centroid /= static_cast<double>(building.footprint.size());

    if (!node->is_leaf()) {
        int idx = child_index(node, centroid);
        insert_building(node->children[idx].get(), building);
        return;
    }

    node->buildings.push_back(building);

    if (node->feature_count() > QuadTreeConfig::MAX_FEATURES_PER_LEAF &&
        node->half_size > QuadTreeConfig::MIN_NODE_SIZE &&
        node->depth < QuadTreeConfig::MAX_DEPTH) {
        subdivide(node);
    }
}

void QuadTree::insert_area(QuadTreeNode* node, const Area& area) {
    if (!node) return;

    glm::dvec2 centroid(0.0);
    for (const auto& pt : area.polygon) centroid += pt;
    centroid /= static_cast<double>(area.polygon.size());

    if (!node->is_leaf()) {
        int idx = child_index(node, centroid);
        insert_area(node->children[idx].get(), area);
        return;
    }

    node->areas.push_back(area);

    if (node->feature_count() > QuadTreeConfig::MAX_FEATURES_PER_LEAF &&
        node->half_size > QuadTreeConfig::MIN_NODE_SIZE &&
        node->depth < QuadTreeConfig::MAX_DEPTH) {
        subdivide(node);
    }
}

void QuadTree::compute_3d_bounds(QuadTreeNode* node) {
    double hs = node->half_size;
    double min_x = node->center.x - hs;
    double min_y = node->center.y - hs;
    double max_x = node->center.x + hs;
    double max_y = node->center.y + hs;

    // Find max building height in this node for Y extent
    float max_height = 50.0f; // default reasonable height
    if (node->is_leaf()) {
        for (const auto& b : node->buildings) {
            max_height = std::max(max_height, b.height);
        }
    }

    // Convert from local 2D (x = east, y = north) to 3D rendering coords
    // In rendering: X = east, Y = up, Z = -north
    node->bounds_min = glm::vec3(
        static_cast<float>(min_x),
        0.0f,
        static_cast<float>(-max_y)
    );
    node->bounds_max = glm::vec3(
        static_cast<float>(max_x),
        max_height,
        static_cast<float>(-min_y)
    );
}

void QuadTree::recompute_bounds(QuadTreeNode* node) {
    if (!node) return;

    if (node->is_leaf()) {
        compute_3d_bounds(node);

        // Road pieces are anchored inside this leaf but are never split, so a
        // road may hang over the boundary. Grow the AABB to cover it, otherwise
        // frustum culling drops the leaf while part of its geometry is still on
        // screen.
        //
        // This is what covers a junction straddling a leaf boundary as well. Its
        // fill and curb ring reach out to the arm mouths in every direction from
        // the node it is anchored at, so a node near a boundary always overhangs;
        // the bounds are grown from the accumulated vertex bounds of the leaf's
        // road mesh, which append_preserving_materials() maintains, so the reach
        // is measured rather than assumed.
        for (const auto& mesh : node->road_meshes) {
            if (!mesh.bounds.is_valid()) continue;
            node->bounds_min = glm::min(node->bounds_min, mesh.bounds.min);
            node->bounds_max = glm::max(node->bounds_max, mesh.bounds.max);
        }
        return;
    }

    node->bounds_min = glm::vec3(std::numeric_limits<float>::max());
    node->bounds_max = glm::vec3(std::numeric_limits<float>::lowest());
    for (auto& child : node->children) {
        if (!child) continue;
        recompute_bounds(child.get());
        node->bounds_min = glm::min(node->bounds_min, child->bounds_min);
        node->bounds_max = glm::max(node->bounds_max, child->bounds_max);
    }
}

// ============================================================================
// Traversal
// ============================================================================

bool QuadTree::frustum_intersects_aabb(const std::array<glm::vec4, 6>& planes,
                                        const glm::vec3& mn, const glm::vec3& mx) const {
    for (const auto& plane : planes) {
        glm::vec3 p = mn;
        if (plane.x >= 0) p.x = mx.x;
        if (plane.y >= 0) p.y = mx.y;
        if (plane.z >= 0) p.z = mx.z;

        if (glm::dot(glm::vec3(plane), p) + plane.w < 0) {
            return false;
        }
    }
    return true;
}

void QuadTree::traverse_visible(
    const std::array<glm::vec4, 6>& frustum_planes,
    const glm::vec3& cam_pos,
    float view_radius,
    float screen_height,
    float fov_y,
    float contribution_threshold,
    bool use_frustum_culling,
    bool use_distance_culling,
    bool use_contribution_culling,
    const QuadTreeVisitor& visitor)
{
    if (!m_root) return;

    float radius_sq = view_radius * view_radius;

    std::vector<std::pair<QuadTreeNode*, float>> visible;
    visible.reserve(128);

    traverse_recursive(m_root.get(), frustum_planes, cam_pos, radius_sq,
                       screen_height, fov_y, contribution_threshold,
                       use_frustum_culling, use_distance_culling, use_contribution_culling,
                       visible);

    // Sort front-to-back by distance for early-Z benefit
    std::sort(visible.begin(), visible.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    for (auto& [node, dist_sq] : visible) {
        visitor(node, dist_sq);
    }
}

void QuadTree::traverse_recursive(
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
    std::vector<std::pair<QuadTreeNode*, float>>& visible)
{
    if (!node) return;

    // 1. Frustum cull
    if (use_frustum && !frustum_intersects_aabb(frustum_planes, node->bounds_min, node->bounds_max)) {
        return;
    }

    // Centre distance, used for front-to-back sorting and for the contribution
    // estimate below.
    glm::vec3 node_center_3d = (node->bounds_min + node->bounds_max) * 0.5f;
    glm::vec3 diff = node_center_3d - cam_pos;
    float dist_sq = diff.x * diff.x + diff.z * diff.z; // XZ plane

    // 2. Distance cull, measured to the NEAREST POINT of the node's XZ footprint.
    //
    // Testing the centre instead prunes the entire subtree of any internal node
    // whose midpoint sits beyond the radius, even when the node's extent reaches
    // the camera -- and internal nodes are large. On a 123km import that rejected
    // every node at depth 1, so nothing was ever built and the viewport stayed
    // empty even though thousands of leaves were within the view radius.
    const float near_dx = cam_pos.x - std::clamp(cam_pos.x, node->bounds_min.x, node->bounds_max.x);
    const float near_dz = cam_pos.z - std::clamp(cam_pos.z, node->bounds_min.z, node->bounds_max.z);
    const float nearest_sq = near_dx * near_dx + near_dz * near_dz;

    if (use_distance && nearest_sq > radius_sq) {
        return;
    }

    // 3. Contribution cull — skip if projected size on screen is too small
    if (use_contribution && contribution_threshold > 0.0f) {
        // Approximate: project the node's world-space size to screen pixels
        float node_world_size = static_cast<float>(node->half_size * 2.0);
        float dist = std::sqrt(dist_sq);
        if (dist > 1.0f) {
            float projected_pixels = (node_world_size / dist) * (screen_height / (2.0f * std::tan(glm::radians(fov_y) * 0.5f)));
            if (projected_pixels < contribution_threshold) {
                return;
            }
        }
    }

    // 4. If leaf: collect
    if (node->is_leaf()) {
        // road_meshes is checked separately: a road piece is routed by its
        // anchor, so a leaf can own road geometry without owning any feature.
        if (node->feature_count() > 0 || !node->road_meshes.empty()) {
            visible.emplace_back(node, dist_sq);
        }
        return;
    }

    // 5. If internal: recurse children
    for (auto& child : node->children) {
        if (child) {
            traverse_recursive(child.get(), frustum_planes, cam_pos, radius_sq,
                               screen_height, fov_y, contribution_threshold,
                               use_frustum, use_distance, use_contribution, visible);
        }
    }
}

// ============================================================================
// Mesh Building
// ============================================================================

QuadTree::BuiltMeshes QuadTree::build_node_meshes_internal(const QuadTreeNode& node) {
    BuiltMeshes result;

    // No roads here. Road geometry is solved once against the whole road graph
    // and delivered by assign_road_pieces(); building it per leaf is what made
    // junctions stop at leaf boundaries. Buildings and areas keep the per-leaf
    // path because they carry no cross-leaf topology.

    // Buildings
    {
        std::vector<Mesh> individual;
        individual.reserve(node.buildings.size());
        for (const auto& building : node.buildings) {
            Mesh mesh = MeshBuilder::build_building_mesh(building);
            if (mesh.is_valid()) individual.push_back(std::move(mesh));
        }
        Mesh merged = MeshBuilder::merge_meshes(individual);
        if (merged.is_valid()) {
            result.building_meshes.push_back(std::move(merged));
        }
    }

    // Areas
    {
        std::vector<Mesh> individual;
        individual.reserve(node.areas.size());
        for (const auto& area : node.areas) {
            Mesh mesh = MeshBuilder::build_area_mesh(area);
            if (mesh.is_valid()) individual.push_back(std::move(mesh));
        }
        Mesh merged = MeshBuilder::merge_meshes(individual);
        if (merged.is_valid()) {
            result.area_meshes.push_back(std::move(merged));
        }
    }

    return result;
}

bool QuadTree::queue_node_build_async(QuadTreeNode* node) {
    if (!node || node->meshes_built || node->meshes_pending) return false;

    node->meshes_pending = true;

    // Copy node data for thread safety. Roads are deliberately not copied: the
    // worker no longer builds road geometry, so copying every Road per node was
    // pure cost.
    QuadTreeNode node_copy;
    node_copy.buildings = node->buildings;
    node_copy.areas = node->areas;

    std::lock_guard<std::mutex> lock(m_pending_mutex);
    m_pending_builds.push_back({
        node,
        std::async(std::launch::async, [this, nc = std::move(node_copy)]() {
            return build_node_meshes_internal(nc);
        })
    });

    return true;
}

size_t QuadTree::poll_async_builds() {
    std::lock_guard<std::mutex> lock(m_pending_mutex);
    size_t completed = 0;

    for (auto it = m_pending_builds.begin(); it != m_pending_builds.end(); ) {
        if (it->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            BuiltMeshes meshes = it->future.get();
            QuadTreeNode* node = it->node;
            if (node) {
                // road_meshes is untouched on purpose: it was filled by
                // assign_road_pieces() before any build was queued, and this
                // result carries no road geometry to replace it with.
                node->building_meshes = std::move(meshes.building_meshes);
                node->area_meshes = std::move(meshes.area_meshes);
                node->meshes_built = true;
                node->meshes_pending = false;
            }
            it = m_pending_builds.erase(it);
            completed++;
        } else {
            ++it;
        }
    }

    return completed;
}

// ============================================================================
// Accessors
// ============================================================================

void QuadTree::collect_leaves(QuadTreeNode* node, std::vector<QuadTreeNode*>& out) {
    if (!node) return;
    if (node->is_leaf()) {
        out.push_back(node);
        return;
    }
    for (auto& child : node->children) {
        collect_leaves(child.get(), out);
    }
}

std::vector<QuadTreeNode*> QuadTree::get_all_leaves() {
    std::vector<QuadTreeNode*> leaves;
    collect_leaves(m_root.get(), leaves);
    return leaves;
}

void QuadTree::get_bounds(glm::vec3& out_min, glm::vec3& out_max) const {
    if (m_root) {
        out_min = m_root->bounds_min;
        out_max = m_root->bounds_max;
    } else {
        out_min = glm::vec3(0.0f);
        out_max = glm::vec3(0.0f);
    }
}

void QuadTree::count_leaves(const QuadTreeNode* node, size_t& count) const {
    if (!node) return;
    if (node->is_leaf()) { count++; return; }
    for (const auto& child : node->children) {
        count_leaves(child.get(), count);
    }
}

size_t QuadTree::leaf_count() const {
    size_t count = 0;
    count_leaves(m_root.get(), count);
    return count;
}

void QuadTree::count_features(const QuadTreeNode* node, size_t& roads, size_t& buildings, size_t& areas) const {
    if (!node) return;
    if (node->is_leaf()) {
        roads += node->roads.size();
        buildings += node->buildings.size();
        areas += node->areas.size();
        return;
    }
    for (const auto& child : node->children) {
        count_features(child.get(), roads, buildings, areas);
    }
}

size_t QuadTree::total_roads() const {
    size_t r = 0, b = 0, a = 0;
    count_features(m_root.get(), r, b, a);
    return r;
}

size_t QuadTree::total_buildings() const {
    size_t r = 0, b = 0, a = 0;
    count_features(m_root.get(), r, b, a);
    return b;
}

size_t QuadTree::total_areas() const {
    size_t r = 0, b = 0, a = 0;
    count_features(m_root.get(), r, b, a);
    return a;
}

void QuadTree::find_max_depth(const QuadTreeNode* node, uint8_t& depth) const {
    if (!node) return;
    depth = std::max(depth, node->depth);
    for (const auto& child : node->children) {
        find_max_depth(child.get(), depth);
    }
}

uint8_t QuadTree::max_depth() const {
    uint8_t d = 0;
    find_max_depth(m_root.get(), d);
    return d;
}

} // namespace stratum::osm
