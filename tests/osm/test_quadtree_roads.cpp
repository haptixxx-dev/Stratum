/**
 * @file test_quadtree_roads.cpp
 * @brief QuadTree::assign_road_pieces() under the piece counts P5 and P6 produce
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * P5 and P6 do not add a new class of RoadPiece -- markings, crossings, bridge
 * decks and tunnel portals are appended into the piece of the edge they belong to
 * -- so the piece COUNT is unchanged from P4. What did change is what a piece
 * carries: several material ranges instead of one or two, and among them
 * MaterialId::Markings, which is the only atlased material in the whole pipeline
 * and therefore the only one that cannot be quietly merged into a neighbour.
 *
 * Two things are checked here, and they are the two ways the hand-off from the
 * road builder to the spatial index can go wrong:
 *
 * 1. **Cost.** Every piece is appended into its leaf's single road mesh. An exact
 *    reserve per appended range would make that quadratic in the ranges routed to
 *    one leaf, and a city extract routes tens of thousands. The geometric growth
 *    in append_preserving_materials() is what stops it, and nothing else in the
 *    tree exercises it.
 * 2. **Material identity.** Pieces arrive interleaved by material, so a leaf holds
 *    one range per material per piece until sort_submeshes_by_material() collapses
 *    them. Markings has to come out of that as its own range, holding exactly the
 *    triangles that went in as Markings -- not folded into Asphalt, not relabelled
 *    Default.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests QuadTreeRoads
 * @endcode
 */

#include "framework.hpp"

#include "osm/quadtree.hpp"
#include "osm/road/road_network_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::SubMesh;
using stratum::Vertex;
using stratum::osm::ParsedOSMData;
using stratum::osm::QuadTree;
using stratum::osm::QuadTreeNode;
using stratum::osm::Road;
using stratum::osm::RoadType;
using stratum::osm::road::RoadPiece;

/// Half the side of the synthetic network, metres
constexpr double kExtent = 1000.0;

/**
 * @brief A parsed network of short roads on a grid, enough to force subdivision
 *
 * The pieces assigned later are routed by anchor into whatever leaves this
 * produces. Subdivision has to come from the FEATURES, because
 * assign_road_pieces() deliberately never subdivides: a piece is appended into an
 * existing leaf and the tree shape is fixed before it runs.
 *
 * @param per_axis Roads along each axis; the total is the square of this
 * @return The data
 */
ParsedOSMData grid_network(int per_axis) {
    ParsedOSMData data;

    for (int iy = 0; iy < per_axis; ++iy) {
        for (int ix = 0; ix < per_axis; ++ix) {
            const double x = -kExtent + 2.0 * kExtent * (ix + 0.5) / per_axis;
            const double y = -kExtent + 2.0 * kExtent * (iy + 0.5) / per_axis;

            Road road;
            road.osm_id = static_cast<stratum::osm::WayId>(iy * per_axis + ix + 1);
            road.type = RoadType::Residential;
            road.width = 6.0f;
            road.lanes = 2;
            road.polyline = {{x - 8.0, y}, {x + 8.0, y}};
            data.roads.push_back(std::move(road));
        }
    }
    return data;
}

/**
 * @brief One quad in one material, at a given height
 *
 * The height is the marker: every assertion about which triangles ended up in
 * which range is made by reading the Y of the vertices the range indexes, so a
 * range that has been relabelled is caught rather than merely counted.
 */
void append_quad(Mesh& mesh, glm::dvec2 at, float y, MaterialId material) {
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    const uint32_t first_index = static_cast<uint32_t>(mesh.indices.size());

    const float x = static_cast<float>(at.x);
    const float z = static_cast<float>(-at.y);  // (x, y_2d) -> vec3(x, h, -y_2d)

    for (int i = 0; i < 4; ++i) {
        Vertex v;
        v.position = glm::vec3(x + static_cast<float>(i & 1),
                               y,
                               z + static_cast<float>((i >> 1) & 1));
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        mesh.vertices.push_back(v);
        mesh.bounds.expand(v.position);
    }

    const uint32_t quad[6] = {base, base + 1, base + 2, base + 1, base + 3, base + 2};
    for (uint32_t idx : quad) mesh.indices.push_back(idx);

    mesh.submeshes.push_back(SubMesh{first_index, 6u, material});
}

/// Y marking a triangle as carriageway
constexpr float kAsphaltY = 1.0f;

/// Y marking a triangle as paint
constexpr float kMarkingsY = 2.0f;

/**
 * @brief One edge piece: a carriageway quad with a marking quad above it
 *
 * The two are separate geometry sharing no vertices, exactly as the UV convention
 * in docs/plans/road_network_plan.md requires of a Markings quad.
 */
RoadPiece surfaced_piece(glm::dvec2 anchor) {
    RoadPiece piece;
    piece.anchor = anchor;
    piece.edge = 0;
    append_quad(piece.mesh, anchor, kAsphaltY, MaterialId::Asphalt);
    append_quad(piece.mesh, anchor, kMarkingsY, MaterialId::Markings);
    return piece;
}

/// Total triangles held across every leaf's road meshes
size_t leaf_triangles(QuadTree& tree) {
    size_t total = 0;
    for (const QuadTreeNode* leaf : tree.get_all_leaves()) {
        for (const Mesh& mesh : leaf->road_meshes) total += mesh.indices.size() / 3;
    }
    return total;
}

/// Leaves that received at least one road piece
size_t touched_leaves(QuadTree& tree) {
    size_t count = 0;
    for (const QuadTreeNode* leaf : tree.get_all_leaves()) {
        if (!leaf->road_meshes.empty()) ++count;
    }
    return count;
}

} // namespace

// ============================================================================
// Cost
// ============================================================================

TEST(QuadTreeRoads, many_small_pieces_are_assigned_without_reshaping_the_tree) {
    // Twenty thousand pieces the size of a zebra stripe, over a tree whose shape
    // was already fixed by its features. Two properties: every triangle arrives,
    // and the tree comes out of it with exactly the leaves it went in with --
    // assign_road_pieces() routes by anchor and never subdivides, so a piece count
    // that changed the shape would mean geometry had moved between leaves after
    // the meshes were sized.
    ParsedOSMData data = grid_network(24);

    QuadTree tree;
    tree.init(data);
    tree.assign_data(data);

    const size_t leaves_before = tree.leaf_count();
    const uint8_t depth_before = tree.max_depth();
    CHECK_TRUE(leaves_before > size_t{1});

    constexpr size_t kPieces = 20000;
    std::vector<RoadPiece> pieces;
    pieces.reserve(kPieces);
    for (size_t i = 0; i < kPieces; ++i) {
        // A deterministic spread over the whole extent, so the pieces land in many
        // leaves rather than piling into one.
        const double t = static_cast<double>(i);
        const double x = -kExtent + std::fmod(t * 37.0, 2.0 * kExtent);
        const double y = -kExtent + std::fmod(t * 71.0, 2.0 * kExtent);
        pieces.push_back(surfaced_piece({x, y}));
    }

    const auto started = std::chrono::steady_clock::now();
    tree.assign_road_pieces(std::move(pieces));
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();

    CHECK_EQ(tree.leaf_count(), leaves_before);
    CHECK_EQ(static_cast<int>(tree.max_depth()), static_cast<int>(depth_before));

    // Four triangles per piece: a carriageway quad and a marking quad.
    CHECK_EQ(leaf_triangles(tree), kPieces * 4);
    CHECK_TRUE(touched_leaves(tree) > size_t{1});

    // Not a benchmark, a shape check. The append path grows its index buffer
    // geometrically; an exact reserve per range makes this quadratic in the ranges
    // routed to one leaf, which at this count is minutes rather than milliseconds.
    // The bound is loose enough that only that failure can trip it.
    CHECK_TRUE(elapsed_ms < 5000.0);
}

// ============================================================================
// Material identity
// ============================================================================

TEST(QuadTreeRoads, markings_survive_the_per_leaf_merge_as_their_own_range) {
    // Every piece here lands in one leaf, so the leaf mesh holds one Asphalt and
    // one Markings range per piece until the merge collapses them. Markings is the
    // only atlased material in the pipeline: folded into Asphalt it would be drawn
    // with a tiling texture, and relabelled Default it would lose its material
    // slot on export. Both failures are invisible in a triangle count, so the
    // ranges are read back against the geometry they index.
    ParsedOSMData data = grid_network(4);

    QuadTree tree;
    tree.init(data);
    tree.assign_data(data);

    constexpr size_t kPieces = 64;
    std::vector<RoadPiece> pieces;
    pieces.reserve(kPieces);
    for (size_t i = 0; i < kPieces; ++i) {
        // A tight cluster: one leaf, so the merge has something to merge.
        pieces.push_back(surfaced_piece({0.25 * static_cast<double>(i), 0.0}));
    }
    tree.assign_road_pieces(std::move(pieces));

    CHECK_EQ(touched_leaves(tree), size_t{1});

    const Mesh* leaf_mesh = nullptr;
    for (const QuadTreeNode* leaf : tree.get_all_leaves()) {
        if (!leaf->road_meshes.empty()) leaf_mesh = &leaf->road_meshes.front();
    }
    CHECK_TRUE(leaf_mesh != nullptr);
    if (leaf_mesh == nullptr) return;

    // Exactly two ranges: one per material that went in, and no Default range for
    // triangles that lost their label on the way.
    const auto ranges = leaf_mesh->effective_submeshes();
    CHECK_EQ(ranges.size(), size_t{2});

    size_t asphalt_indices = 0;
    size_t markings_indices = 0;
    bool markings_hold_only_paint = true;
    bool asphalt_holds_only_surface = true;
    bool no_default_range = true;

    for (const SubMesh& range : ranges) {
        if (range.material == MaterialId::Default) no_default_range = false;

        const size_t end = static_cast<size_t>(range.index_offset) + range.index_count;
        CHECK_TRUE(end <= leaf_mesh->indices.size());
        if (end > leaf_mesh->indices.size()) continue;

        for (size_t i = range.index_offset; i < end; ++i) {
            const float y = leaf_mesh->vertices[leaf_mesh->indices[i]].position.y;
            if (range.material == MaterialId::Markings && y != kMarkingsY) {
                markings_hold_only_paint = false;
            }
            if (range.material == MaterialId::Asphalt && y != kAsphaltY) {
                asphalt_holds_only_surface = false;
            }
        }

        if (range.material == MaterialId::Markings) markings_indices += range.index_count;
        if (range.material == MaterialId::Asphalt) asphalt_indices += range.index_count;
    }

    CHECK_TRUE(no_default_range);
    CHECK_TRUE(markings_hold_only_paint);
    CHECK_TRUE(asphalt_holds_only_surface);

    // One quad of each per piece: six indices each.
    CHECK_EQ(asphalt_indices, kPieces * 6);
    CHECK_EQ(markings_indices, kPieces * 6);
}

TEST(QuadTreeRoads, a_leaf_holding_only_paint_still_reports_it_as_paint) {
    // The single-range case, which sort_submeshes_by_material() returns from
    // early. Early-returning is correct only if the one range it leaves alone is
    // already labelled, and the label it would otherwise fall back to -- Default,
    // the implicit whole-mesh material -- is exactly the wrong one for the only
    // atlased material in the pipeline.
    ParsedOSMData data = grid_network(4);

    QuadTree tree;
    tree.init(data);
    tree.assign_data(data);

    RoadPiece piece;
    piece.anchor = {0.0, 0.0};
    piece.edge = 0;
    append_quad(piece.mesh, piece.anchor, kMarkingsY, MaterialId::Markings);

    std::vector<RoadPiece> pieces;
    pieces.push_back(std::move(piece));
    tree.assign_road_pieces(std::move(pieces));

    const Mesh* leaf_mesh = nullptr;
    for (const QuadTreeNode* leaf : tree.get_all_leaves()) {
        if (!leaf->road_meshes.empty()) leaf_mesh = &leaf->road_meshes.front();
    }
    CHECK_TRUE(leaf_mesh != nullptr);
    if (leaf_mesh == nullptr) return;

    const auto ranges = leaf_mesh->effective_submeshes();
    CHECK_EQ(ranges.size(), size_t{1});
    CHECK_TRUE(ranges[0].material == MaterialId::Markings);
    CHECK_EQ(static_cast<size_t>(ranges[0].index_count), leaf_mesh->indices.size());
}
