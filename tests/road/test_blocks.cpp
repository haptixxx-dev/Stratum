// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_blocks.cpp
 * @brief Face traversal tests against graphs whose answer is known by hand
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Every graph here is built in code from a node table and a way list rather than
 * parsed from a fixture, because the point of each test is an answer stated in
 * advance -- "a 100 m square is one block of 10000 m^2" -- and a parsed fixture
 * recentres its coordinates, which would leave the areas right but the ring
 * positions unassertable.
 *
 * Each test names the failure it exists to catch. A face traversal fails
 * silently: four blocks where there should be five looks exactly like success, so
 * a test that only counts a non-zero number of blocks tests nothing.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests Blocks
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/blocks.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using stratum::osm::NodeId;
using stratum::osm::ParsedOSMData;
using stratum::osm::Road;
using stratum::osm::RoadType;
using stratum::osm::WayId;
using stratum::osm::road::Block;
using stratum::osm::road::BlockConfig;
using stratum::osm::road::BlockEdge;
using stratum::osm::road::BlockExtraction;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::extract_blocks;

// ============================================================================
// Graph Construction
// ============================================================================

/// Node positions in local metres, so a way is written as a list of node ids
using NodeMap = std::unordered_map<NodeId, glm::dvec2>;

/// The way attributes any test varies; everything else is a plain residential road
struct WayOptions {
    RoadType type = RoadType::Residential;
    int layer = 0;
    bool is_bridge = false;
};

/**
 * @brief Append one way to @p data, taking its geometry from @p positions
 *
 * node_ids is set from @p ids and the polyline is looked up point by point, so
 * the two are parallel by construction. A road whose node_ids does not match its
 * polyline is silently skipped by RoadGraph::build(), and a test built on one
 * would assert against an empty graph.
 */
void add_way(ParsedOSMData& data, const NodeMap& positions, WayId id,
             const std::vector<NodeId>& ids, const WayOptions& options = {}) {
    Road road;
    road.osm_id = id;
    road.type = options.type;
    road.layer = options.layer;
    road.is_bridge = options.is_bridge;
    road.node_ids = ids;
    road.polyline.reserve(ids.size());

    for (const NodeId node : ids) {
        const auto it = positions.find(node);
        if (it == positions.end()) {
            stratum::test::report_failure(__FILE__, __LINE__, "node has a position",
                                          "node " + std::to_string(node));
            return;
        }
        road.polyline.push_back(it->second);
    }

    data.roads.push_back(std::move(road));
}

/// Unit square scaled to 100 m, corners 1..4 anticlockwise from the origin
NodeMap square_100m() {
    return NodeMap{{1, {0.0, 0.0}}, {2, {100.0, 0.0}}, {3, {100.0, 100.0}}, {4, {0.0, 100.0}}};
}

/// Four ways, one per side of square_100m(), way ids 10..13
void add_square_100m(ParsedOSMData& data, const NodeMap& positions) {
    add_way(data, positions, 10, {1, 2});
    add_way(data, positions, 11, {2, 3});
    add_way(data, positions, 12, {3, 4});
    add_way(data, positions, 13, {4, 1});
}

/// Nine nodes on a 100 m lattice, numbered west to east then south to north
NodeMap grid_3x3_nodes() {
    return NodeMap{
        {1, {0.0, 0.0}},   {2, {100.0, 0.0}},   {3, {200.0, 0.0}},
        {4, {0.0, 100.0}}, {5, {100.0, 100.0}}, {6, {200.0, 100.0}},
        {7, {0.0, 200.0}}, {8, {100.0, 200.0}}, {9, {200.0, 200.0}},
    };
}

/**
 * @brief Three east-west ways and three north-south ways over grid_3x3_nodes()
 *
 * Four bounded faces and one outer face, which is why the orientation tests live
 * on this and not on a single square: a square's two faces have the same four
 * corners in the same anticlockwise order, so neither traversal convention can be
 * told from the other on one. Here the outer face is a 200 m square of 40000 m^2
 * and no bounded face looks anything like it.
 */
void add_grid_3x3(ParsedOSMData& data, const NodeMap& positions) {
    add_way(data, positions, 10, {1, 2, 3});
    add_way(data, positions, 11, {4, 5, 6});
    add_way(data, positions, 12, {7, 8, 9});
    add_way(data, positions, 13, {1, 4, 7});
    add_way(data, positions, 14, {2, 5, 8});
    add_way(data, positions, 15, {3, 6, 9});
}

// ============================================================================
// Assertion Helpers
// ============================================================================

/// True when two local-metre points agree to within @p eps on both axes
bool points_equal(const glm::dvec2& a, const glm::dvec2& b, double eps = 1e-9) {
    return std::fabs(a.x - b.x) <= eps && std::fabs(a.y - b.y) <= eps;
}

/// True when @p ring holds a point within @p eps of @p point
bool ring_contains(const std::vector<glm::dvec2>& ring, const glm::dvec2& point,
                   double eps = 1e-9) {
    for (const glm::dvec2& p : ring) {
        if (points_equal(p, point, eps)) {
            return true;
        }
    }
    return false;
}

/// Count of blocks whose area is within @p eps of @p area
size_t count_area(const std::vector<Block>& blocks, double area, double eps = 1e-6) {
    size_t found = 0;
    for (const Block& block : blocks) {
        if (std::fabs(block.area - area) <= eps) ++found;
    }
    return found;
}

/// Times @p edge appears in @p block's boundary with the given direction
size_t count_half_edge(const Block& block, stratum::osm::road::EdgeId edge, bool forward) {
    size_t found = 0;
    for (const BlockEdge& half : block.edges) {
        if (half.edge == edge && half.forward == forward) ++found;
    }
    return found;
}

/**
 * @brief Does the directed segment a->b appear on this half-edge's polyline?
 *
 * Walks the edge in the half-edge's direction and looks for a and b as a
 * CONSECUTIVE pair. Consecutive, not merely present, so a ring segment attributed
 * to an edge that happens to pass near both ends does not count as a match.
 */
bool segment_lies_on(const RoadGraph& graph, const BlockEdge& half,
                     const glm::dvec2& a, const glm::dvec2& b, double eps = 1e-9) {
    if (half.edge >= graph.edges().size()) return false;

    const GraphEdge& edge = graph.edge(half.edge);
    std::vector<glm::dvec2> line = edge.polyline;
    if (!half.forward) {
        std::vector<glm::dvec2> reversed(line.rbegin(), line.rend());
        line.swap(reversed);
    }

    for (size_t i = 0; i + 1 < line.size(); ++i) {
        if (points_equal(line[i], a, eps) && points_equal(line[i + 1], b, eps)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Check that every ring segment lies on the half-edge Block::ring_edges names
 *
 * This is the assertion that catches an owner table left stale by the antenna
 * prune: at the node a cul-de-sac hangs off, the unpruned owner is the spur, and
 * the spur does not carry the segment that leaves that node once the spur is gone.
 */
void check_ring_edges(const RoadGraph& graph, const Block& block) {
    CHECK_EQ(block.ring_edges.size(), block.ring.size());
    if (block.ring_edges.size() != block.ring.size()) return;

    for (size_t i = 0; i < block.ring.size(); ++i) {
        const glm::dvec2& a = block.ring[i];
        const glm::dvec2& b = block.ring[(i + 1) % block.ring.size()];

        CHECK(block.ring_edges[i] < block.edges.size());
        if (block.ring_edges[i] >= block.edges.size()) continue;

        CHECK_TRUE(segment_lies_on(graph, block.edges[block.ring_edges[i]], a, b));
    }
}

/// Cross product of (b - a) and (c - a); positive when c is left of a->b
double cross_z(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/// Id of the graph node at @p point, or nodes.size() when there is none
size_t ring_index_of_node(const RoadGraph& graph, const glm::dvec2& point) {
    for (size_t i = 0; i < graph.nodes().size(); ++i) {
        if (points_equal(graph.nodes()[i].position, point)) return i;
    }
    return graph.nodes().size();
}

/// Mean of a ring's vertices. Inside any convex ring, which is all this is used on.
glm::dvec2 ring_centroid(const std::vector<glm::dvec2>& ring) {
    glm::dvec2 sum{0.0};
    for (const glm::dvec2& p : ring) sum += p;
    return ring.empty() ? sum : sum / static_cast<double>(ring.size());
}

/// True when no two points of @p ring are the same place
bool ring_has_no_repeated_point(const std::vector<glm::dvec2>& ring, double eps = 1e-9) {
    for (size_t i = 0; i < ring.size(); ++i) {
        for (size_t j = i + 1; j < ring.size(); ++j) {
            if (points_equal(ring[i], ring[j], eps)) return false;
        }
    }
    return true;
}

/// Every (edge, direction) pair any block lists, repeats kept
std::vector<std::pair<stratum::osm::road::EdgeId, bool>>
all_half_edges(const std::vector<Block>& blocks) {
    std::vector<std::pair<stratum::osm::road::EdgeId, bool>> all;
    for (const Block& block : blocks) {
        for (const BlockEdge& half : block.edges) all.emplace_back(half.edge, half.forward);
    }
    return all;
}

/// The block whose area is nearest @p area, or blocks.size() when there are none
size_t block_nearest_area(const std::vector<Block>& blocks, double area) {
    size_t best = blocks.size();
    double gap = 0.0;
    for (size_t i = 0; i < blocks.size(); ++i) {
        const double d = std::fabs(blocks[i].area - area);
        if (best == blocks.size() || d < gap) { best = i; gap = d; }
    }
    return best;
}

/// check_ring_edges() for a hole: the same contract, on holes[h] and hole_edges[h]
void check_hole_edges(const RoadGraph& graph, const Block& block, size_t h) {
    CHECK_EQ(block.hole_edges.size(), block.holes.size());
    if (h >= block.holes.size() || h >= block.hole_edges.size()) return;
    CHECK_EQ(block.hole_edges[h].size(), block.holes[h].size());
    if (block.hole_edges[h].size() != block.holes[h].size()) return;

    for (size_t i = 0; i < block.holes[h].size(); ++i) {
        const glm::dvec2& a = block.holes[h][i];
        const glm::dvec2& b = block.holes[h][(i + 1) % block.holes[h].size()];
        CHECK(block.hole_edges[h][i] < block.edges.size());
        if (block.hole_edges[h][i] >= block.edges.size()) continue;
        CHECK_TRUE(segment_lies_on(graph, block.edges[block.hole_edges[h][i]], a, b));
    }
}

} // namespace

// ============================================================================
// A Single Square
// ============================================================================

/**
 * The smallest graph with a bounded face. Catches a traversal that keeps the
 * outer face as well (two blocks), and one that keeps only the outer face
 * (one block of the wrong ring), because the area and the vertex order are both
 * asserted.
 */
TEST(Blocks, single_square_yields_exactly_one_block) {
    const NodeMap positions = square_100m();
    ParsedOSMData data;
    add_square_100m(data, positions);

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{4});
    CHECK_EQ(graph.edges().size(), size_t{4});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    CHECK_EQ(extraction.stats.faces, size_t{2});
    CHECK_EQ(extraction.stats.outer_faces, size_t{1});
    CHECK_EQ(extraction.stats.rejected_small, size_t{0});
    CHECK_EQ(extraction.stats.tree_faces, size_t{0});
    CHECK_EQ(extraction.stats.zero_area_faces, size_t{0});
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
    CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
    if (extraction.blocks.empty()) return;

    const Block& block = extraction.blocks.front();
    CHECK_EQ(block.id, uint32_t{0});
    CHECK_NEAR(block.area, 10000.0, 1e-9);
    CHECK_NEAR(block.perimeter, 400.0, 1e-9);
    CHECK_EQ(block.ring.size(), size_t{4});
    CHECK_EQ(block.edges.size(), size_t{4});
    CHECK_TRUE(block.holes.empty());
    CHECK_FALSE(block.has_grade_separated_edge);
}

/**
 * The orientation contract, asserted as a vertex ORDER rather than as a sign, so
 * it cannot be satisfied by a ring that happens to have the corners in some other
 * rotation.
 *
 * On the GRID and not on a single square, and that is the whole point. A square
 * has two faces over the same four corners, and an anticlockwise-next traversal
 * merely swaps which of them is discarded: the survivor is the same ring, the same
 * area and the same perimeter, so a square cannot tell the two conventions apart
 * and asserting a ring order on one asserts a symmetry. Here the outer face is a
 * 200 m square, so under the wrong convention there is one block of 40000 m^2 and
 * these checks have nothing to match.
 */
TEST(Blocks, block_ring_runs_anticlockwise) {
    const NodeMap positions = grid_3x3_nodes();
    ParsedOSMData data;
    add_grid_3x3(data, positions);

    RoadGraph graph;
    graph.build(data);
    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{4});
    CHECK_EQ(count_area(extraction.blocks, 40000.0), size_t{0});   // never the outer face

    for (const Block& block : extraction.blocks) {
        CHECK_EQ(block.ring.size(), size_t{4});
        if (block.ring.size() != 4) continue;

        // The corner nearest the origin, then anticlockwise: east, north, west.
        size_t corner = 0;
        for (size_t i = 1; i < 4; ++i) {
            if (block.ring[i].x + block.ring[i].y < block.ring[corner].x + block.ring[corner].y) {
                corner = i;
            }
        }
        const glm::dvec2 base = block.ring[corner];
        CHECK_TRUE(points_equal(block.ring[(corner + 1) % 4], base + glm::dvec2{100.0, 0.0}));
        CHECK_TRUE(points_equal(block.ring[(corner + 2) % 4], base + glm::dvec2{100.0, 100.0}));
        CHECK_TRUE(points_equal(block.ring[(corner + 3) % 4], base + glm::dvec2{0.0, 100.0}));
    }
}

/**
 * The property C4 depends on: the block is on the LEFT of each bounding
 * half-edge. Asserted per half-edge from the graph geometry, not from the ring,
 * so it also catches a BlockEdge::forward flag written the wrong way round while
 * the ring itself came out right.
 *
 * On the grid, for the reason block_ring_runs_anticlockwise gives, and with one
 * more assertion a single square cannot make: an interior edge of the grid fronts
 * TWO blocks, once each way, and a boundary edge fronts one. A forward flag
 * written the wrong way round breaks that split even where the rings survive.
 */
TEST(Blocks, block_lies_left_of_every_bounding_half_edge) {
    const NodeMap positions = grid_3x3_nodes();
    ParsedOSMData data;
    add_grid_3x3(data, positions);

    RoadGraph graph;
    graph.build(data);
    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{4});

    for (const Block& block : extraction.blocks) {
        const glm::dvec2 interior = ring_centroid(block.ring);
        for (const BlockEdge& half : block.edges) {
            const GraphEdge& edge = graph.edge(half.edge);
            const glm::dvec2 start = half.forward ? edge.polyline.front() : edge.polyline.back();
            const glm::dvec2 end = half.forward ? edge.polyline.back() : edge.polyline.front();
            CHECK((0.0) < (cross_z(start, end, interior)));
        }
    }

    // The four inner edges -- the ones with a block on each side -- are used twice,
    // once each way. The eight outer edges are used once, because the other side of
    // them is the outer face.
    size_t used_twice = 0;
    size_t used_once = 0;
    for (size_t e = 0; e < graph.edges().size(); ++e) {
        const auto id = static_cast<stratum::osm::road::EdgeId>(e);
        size_t forward = 0;
        size_t backward = 0;
        for (const Block& block : extraction.blocks) {
            forward += count_half_edge(block, id, true);
            backward += count_half_edge(block, id, false);
        }
        if (forward == 1 && backward == 1) ++used_twice;
        else if (forward + backward == 1) ++used_once;
    }
    CHECK_EQ(used_twice, size_t{4});
    CHECK_EQ(used_once, size_t{8});
}

// ============================================================================
// A 2x2 Grid
// ============================================================================

/**
 * Four blocks, not five. The fifth face is the outer one, and an implementation
 * that keeps it reports five here while every other count still looks plausible.
 */
TEST(Blocks, grid_2x2_yields_four_blocks_not_five) {
    const NodeMap positions = grid_3x3_nodes();
    ParsedOSMData data;
    add_grid_3x3(data, positions);

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{9});
    CHECK_EQ(graph.edges().size(), size_t{12});   // every way splits at its middle node

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{4});
    CHECK_EQ(extraction.stats.blocks, extraction.blocks.size());
    CHECK_EQ(extraction.stats.faces, size_t{5});
    CHECK_EQ(extraction.stats.outer_faces, size_t{1});

    double total = 0.0;
    for (size_t i = 0; i < extraction.blocks.size(); ++i) {
        const Block& block = extraction.blocks[i];
        // On a four-element container, unlike on the one-block fixtures: an
        // implementation that hands every block the id 0 is invisible on those.
        CHECK_EQ(block.id, static_cast<uint32_t>(i));
        CHECK_NEAR(block.area, 10000.0, 1e-9);
        CHECK_EQ(block.ring.size(), size_t{4});
        CHECK_EQ(block.edges.size(), size_t{4});
        total += block.area;
    }
    // The four blocks tile the grid exactly: no face is counted twice and none is
    // the outer face wearing a positive area.
    CHECK_NEAR(total, 40000.0, 1e-9);
}

/**
 * That the orbits the traversal found are the faces the graph HAS.
 *
 * Not `half_edges_walked == 2 * edges`, which was here before and cannot fail:
 * the count is incremented once per `visited[h] = 1`, and the seed loop visits
 * every half-edge of every usable edge whatever next() does. An implementation
 * that turns straight round at every node -- no rotation step at all -- still
 * lands on it exactly.
 *
 * Euler's formula can fail, and does. F = E - N + 2C over the usable subgraph is
 * computed without reference to the traversal, so a traversal that merges two
 * faces into one, or that shatters the graph into one orbit per edge, disagrees
 * with it. The second assertion is the other half: no half-edge may front two
 * different blocks, since a half-edge has one face on its left and only one.
 */
TEST(Blocks, the_orbits_found_are_the_faces_euler_predicts) {
    NodeMap positions = grid_3x3_nodes();
    positions[20] = glm::dvec2{150.0, 50.0};    // cul-de-sac tip inside the lower right block

    ParsedOSMData data;
    add_grid_3x3(data, positions);
    add_way(data, positions, 16, {5, 20});

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.edges().size(), size_t{13});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.edges_used, graph.edges().size());
    CHECK_EQ(extraction.stats.half_edges_walked, graph.edges().size() * 2);

    CHECK_EQ(extraction.stats.expected_faces, size_t{5});    // 13 - 10 + 2 * 1
    CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
    CHECK_EQ(extraction.blocks.size(), size_t{4});

    // Four blocks of four half-edges, plus the spur once each way in the block it
    // dead-ends into. The eight remaining half-edges are the outer face's.
    std::vector<std::pair<stratum::osm::road::EdgeId, bool>> used =
        all_half_edges(extraction.blocks);
    CHECK_EQ(used.size(), size_t{18});
    for (size_t i = 0; i < used.size(); ++i) {
        for (size_t j = i + 1; j < used.size(); ++j) {
            CHECK_FALSE(used[i] == used[j]);
        }
    }
}

// ============================================================================
// Dangling Ways
// ============================================================================

/**
 * A cul-de-sac inside a block. The spur is walked out and back within the block's
 * own face, so it must change neither the area nor the ring, and it must still be
 * listed as bounding the block -- in BOTH directions, because both sides of a
 * dead-end street front the same land.
 */
TEST(Blocks, cul_de_sac_inside_a_block_leaves_area_and_ring_untouched) {
    NodeMap positions = square_100m();
    positions[5] = glm::dvec2{50.0, 50.0};      // tip, inside the square

    ParsedOSMData data;
    add_square_100m(data, positions);
    add_way(data, positions, 14, {2, 5});

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.edges().size(), size_t{5});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    CHECK_EQ(extraction.stats.faces, size_t{2});
    CHECK_EQ(extraction.stats.outer_faces, size_t{1});
    if (extraction.blocks.empty()) return;

    const Block& block = extraction.blocks.front();
    CHECK_NEAR(block.area, 10000.0, 1e-9);      // the spur encloses nothing
    CHECK_NEAR(block.perimeter, 400.0, 1e-9);   // and adds nothing to the boundary
    CHECK_EQ(block.ring.size(), size_t{4});     // the antenna is pruned out of the ring

    // Four sides plus the spur once each way. The spur is the only edge whose two
    // half-edges share a face.
    CHECK_EQ(block.edges.size(), size_t{6});

    const stratum::osm::road::EdgeId spur = [&]() {
        for (size_t i = 0; i < graph.edges().size(); ++i) {
            if (graph.edge(static_cast<stratum::osm::road::EdgeId>(i)).source_way == 14) {
                return static_cast<stratum::osm::road::EdgeId>(i);
            }
        }
        return stratum::osm::road::kInvalidId;
    }();
    CHECK(spur != stratum::osm::road::kInvalidId);
    if (spur == stratum::osm::road::kInvalidId) return;

    CHECK_EQ(count_half_edge(block, spur, true), size_t{1});
    CHECK_EQ(count_half_edge(block, spur, false), size_t{1});

    // A spur with no loop on the end of it encloses nothing, so it is a cut edge
    // that leaves no hole behind -- unlike the lollipop two tests below.
    CHECK_TRUE(block.holes.empty());
}

/**
 * The same cul-de-sac, but the spur way is drawn FROM its tip and listed first, so
 * the traversal seeds on the half-edge leaving the dead end and the spur straddles
 * the START of the walk: the walk's first traversal is the spur coming back, and
 * its last is the spur going out.
 *
 * The cut-edge pass pairs the two traversals of an edge by which of them the walk
 * reached FIRST, not by which of them runs along the edge forwards, and that is
 * what makes this case the same case as any other. Pair them the other way round
 * and the walk opens with a traversal that closes a pair nothing opened.
 */
TEST(Blocks, cul_de_sac_walked_from_its_tip_still_closes_the_ring) {
    NodeMap positions = square_100m();
    positions[5] = glm::dvec2{50.0, 50.0};

    ParsedOSMData data;
    add_way(data, positions, 9, {5, 2});    // listed first, so it is edge 0
    add_square_100m(data, positions);

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.edge(0).source_way, stratum::osm::WayId{9});
    CHECK_EQ(graph.edge(0).from, stratum::osm::road::GraphNodeId{0});   // the tip

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    CHECK_EQ(extraction.stats.faces, size_t{2});
    if (extraction.blocks.empty()) return;

    const Block& block = extraction.blocks.front();
    CHECK_NEAR(block.area, 10000.0, 1e-9);
    CHECK_EQ(block.ring.size(), size_t{4});
    CHECK_FALSE(ring_contains(block.ring, glm::dvec2{50.0, 50.0}));  // the tip is gone
    CHECK_EQ(block.edges.size(), size_t{6});
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
    check_ring_edges(graph, block);

    // The spur is the first half-edge the walk took and still bounds no land, so
    // no ring segment may be attributed to it.
    for (const uint32_t owner : block.ring_edges) {
        if (owner >= block.edges.size()) continue;
        CHECK(graph.edge(block.edges[owner].edge).source_way != stratum::osm::WayId{9});
    }
}

// ============================================================================
// Closed Ways
// ============================================================================

/**
 * A closed way with nothing else attached -- an isolated loop road or roundabout.
 * RoadGraph gives it ONE edge whose two ends are the SAME node, so that node
 * carries two arms for one edge and they are told apart only by Arm::at_start.
 * An arm lookup that matches on edge id alone picks whichever comes first, sends
 * the walk round the loop the wrong way, and merges the two faces into one.
 */
TEST(Blocks, a_closed_way_is_a_self_loop_with_one_bounded_face) {
    const NodeMap positions = square_100m();

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2, 3, 4, 1});

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{1});
    CHECK_EQ(graph.edges().size(), size_t{1});
    CHECK_EQ(graph.node(0).degree(), size_t{2});    // both ends of one edge

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    CHECK_EQ(extraction.stats.faces, size_t{2});
    CHECK_EQ(extraction.stats.outer_faces, size_t{1});
    CHECK_EQ(extraction.stats.half_edges_walked, size_t{2});
    if (extraction.blocks.empty()) return;

    const Block& block = extraction.blocks.front();
    CHECK_NEAR(block.area, 10000.0, 1e-9);
    CHECK_EQ(block.ring.size(), size_t{4});
    CHECK_EQ(block.edges.size(), size_t{1});
    CHECK_TRUE(block.edges.front().forward);    // the way is drawn anticlockwise
    check_ring_edges(graph, block);
}

/**
 * The lollipop cul-de-sac: one stem out to a turning bulb mapped as a closed way.
 * It is the case the positional `x, tip, x` cancellation cannot see, because the
 * stem's two traversals are not adjacent in the walk -- the whole bulb is walked
 * between them -- so the stem survived as a zero-width slit across the block.
 *
 * Every number here is stated in advance. The land is the square less the bulb,
 * 10000 - 200. Its boundary is the square and the bulb, 400 + 4 * sqrt(200); the
 * stem is walked twice and bounds nothing, so it is in neither. The unpruned ring
 * had ten points with (100,0) and (50,50) in it twice each, a perimeter of
 * 597.99 -- the distance WALKED -- and a compactness of 0.344 where the block's
 * own shape gives 0.591.
 */
TEST(Blocks, a_spur_ending_in_a_loop_leaves_no_slit_in_the_ring) {
    NodeMap positions = square_100m();
    positions[5] = glm::dvec2{50.0, 50.0};      // where the stem meets the bulb
    positions[6] = glm::dvec2{60.0, 40.0};
    positions[7] = glm::dvec2{70.0, 50.0};
    positions[8] = glm::dvec2{60.0, 60.0};

    ParsedOSMData data;
    add_square_100m(data, positions);
    add_way(data, positions, 14, {2, 5});           // the stem
    add_way(data, positions, 15, {5, 6, 7, 8, 5});  // the bulb, a closed way

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.edges().size(), size_t{6});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.expected_faces, size_t{3});   // 6 - 5 + 2 * 1
    CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
    CHECK_EQ(extraction.blocks.size(), size_t{2});

    const size_t big = block_nearest_area(extraction.blocks, 9800.0);
    if (big >= extraction.blocks.size()) return;
    const Block& block = extraction.blocks[big];

    CHECK_NEAR(block.area, 9800.0, 1e-9);                       // the square less the bulb
    CHECK_NEAR(block.perimeter, 400.0 + 4.0 * std::sqrt(200.0), 1e-9);
    CHECK_EQ(block.ring.size(), size_t{4});
    CHECK_TRUE(ring_has_no_repeated_point(block.ring));
    CHECK_FALSE(ring_contains(block.ring, glm::dvec2{50.0, 50.0}));   // the bulb is not on it
    CHECK_NEAR(stratum::osm::road::signed_ring_area(block.ring), 10000.0, 1e-9);

    // The bulb is land this block does not own, so it comes out as a hole, wound
    // the other way round.
    CHECK_EQ(block.holes.size(), size_t{1});
    if (block.holes.size() == 1) {
        CHECK_EQ(block.holes.front().size(), size_t{4});
        CHECK_NEAR(stratum::osm::road::signed_ring_area(block.holes.front()), -200.0, 1e-9);
        CHECK_TRUE(ring_contains(block.holes.front(), glm::dvec2{50.0, 50.0}));
        check_hole_edges(graph, block, 0);
    }
    check_ring_edges(graph, block);

    // The stem is off the boundary but still fronts the block, both ways.
    const auto stem = [&]() {
        for (size_t i = 0; i < graph.edges().size(); ++i) {
            if (graph.edge(static_cast<stratum::osm::road::EdgeId>(i)).source_way == 14) {
                return static_cast<stratum::osm::road::EdgeId>(i);
            }
        }
        return stratum::osm::road::kInvalidId;
    }();
    CHECK(stem != stratum::osm::road::kInvalidId);
    if (stem != stratum::osm::road::kInvalidId) {
        CHECK_EQ(count_half_edge(block, stem, true), size_t{1});
        CHECK_EQ(count_half_edge(block, stem, false), size_t{1});
    }

    // The land inside the bulb is its own block, so nothing was double counted.
    const size_t bulb = block_nearest_area(extraction.blocks, 200.0);
    if (bulb < extraction.blocks.size()) {
        CHECK_NEAR(extraction.blocks[bulb].area, 200.0, 1e-9);
        CHECK_TRUE(extraction.blocks[bulb].holes.empty());
    }
}

/**
 * The same defect at estate scale: a loop road reached by one access way, which is
 * the other everyday shape a spur-into-a-cycle takes. Here the loop is three ways
 * rather than one closed way, so the stem's two traversals have six half-edges
 * between them and nothing positional could pair them up.
 */
TEST(Blocks, an_estate_loop_off_one_access_road_is_a_hole_not_a_slit) {
    const NodeMap positions{
        {1, {0.0, 0.0}},     {2, {200.0, 0.0}},   {3, {200.0, 200.0}}, {4, {0.0, 200.0}},
        {5, {150.0, 100.0}}, {6, {100.0, 100.0}}, {7, {100.0, 50.0}},
    };

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2});
    add_way(data, positions, 11, {2, 3});
    add_way(data, positions, 12, {3, 4});
    add_way(data, positions, 13, {4, 1});
    add_way(data, positions, 20, {2, 5});   // the access road
    add_way(data, positions, 21, {5, 6});   // the loop
    add_way(data, positions, 22, {6, 7});
    add_way(data, positions, 23, {7, 5});

    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.expected_faces, size_t{3});    // 8 - 7 + 2 * 1
    CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
    CHECK_EQ(extraction.blocks.size(), size_t{2});

    const size_t big = block_nearest_area(extraction.blocks, 38750.0);
    if (big >= extraction.blocks.size()) return;
    const Block& block = extraction.blocks[big];

    CHECK_NEAR(block.area, 38750.0, 1e-9);                   // 40000 less the 1250 loop
    CHECK_NEAR(block.perimeter, 800.0 + 100.0 + std::sqrt(5000.0), 1e-9);
    CHECK_EQ(block.ring.size(), size_t{4});
    CHECK_TRUE(ring_has_no_repeated_point(block.ring));
    CHECK_EQ(block.holes.size(), size_t{1});
    if (block.holes.size() == 1) {
        CHECK_EQ(block.holes.front().size(), size_t{3});
        CHECK_NEAR(stratum::osm::road::signed_ring_area(block.holes.front()), -1250.0, 1e-9);
        check_hole_edges(graph, block, 0);
    }
    check_ring_edges(graph, block);

    // Nine half-edges: four square sides, the access road both ways, and the
    // three loop ways on their outward side.
    CHECK_EQ(block.edges.size(), size_t{9});
}

/**
 * The same bulb with NO stem: the closed way shares one node with the street and
 * hangs straight off it. There is no cut edge to find here -- the join is a cut
 * VERTEX -- so the face walks the road up to that node, round the bulb, and back
 * out along the road, and its boundary pinches to a point at the node.
 *
 * Area and perimeter survive a pinch, because a point has no width and nothing is
 * walked twice. The SHAPE does not: a ring that visits one point twice is not a
 * polygon, and a lot cut from it would straddle the pinch and own land on both
 * sides of a street. The answer is the same as for the lollipop -- an outer
 * boundary and a hole -- reached by a different route.
 *
 * Found by fuzzing random lattice graphs against Euler's formula, not by hand.
 */
TEST(Blocks, a_loop_joined_at_one_node_is_a_hole_not_a_pinched_ring) {
    NodeMap positions = square_100m();
    positions[5] = glm::dvec2{50.0, 0.0};       // on the southern side
    positions[6] = glm::dvec2{40.0, 10.0};
    positions[7] = glm::dvec2{50.0, 20.0};
    positions[8] = glm::dvec2{60.0, 10.0};

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 5, 2});        // southern side, split at the join
    add_way(data, positions, 11, {2, 3});
    add_way(data, positions, 12, {3, 4});
    add_way(data, positions, 13, {4, 1});
    add_way(data, positions, 14, {5, 8, 7, 6, 5});  // the bulb, joined at node 5 only

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{5});
    CHECK_EQ(graph.edges().size(), size_t{6});
    CHECK_EQ(graph.node(ring_index_of_node(graph, glm::dvec2{50.0, 0.0})).degree(), size_t{4});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.expected_faces, size_t{3});    // 6 - 5 + 2 * 1
    CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
    CHECK_EQ(extraction.blocks.size(), size_t{2});

    const size_t big = block_nearest_area(extraction.blocks, 9800.0);
    if (big >= extraction.blocks.size()) return;
    const Block& block = extraction.blocks[big];

    CHECK_NEAR(block.area, 9800.0, 1e-9);
    CHECK_NEAR(block.perimeter, 400.0 + 4.0 * std::sqrt(200.0), 1e-9);
    CHECK_EQ(block.ring.size(), size_t{5});                  // four corners plus the join
    CHECK_TRUE(ring_has_no_repeated_point(block.ring));      // the pinch is gone
    CHECK_NEAR(stratum::osm::road::signed_ring_area(block.ring), 10000.0, 1e-9);
    CHECK_EQ(block.holes.size(), size_t{1});
    if (block.holes.size() == 1) {
        CHECK_EQ(block.holes.front().size(), size_t{4});
        CHECK_NEAR(stratum::osm::road::signed_ring_area(block.holes.front()), -200.0, 1e-9);
        check_hole_edges(graph, block, 0);
    }
    check_ring_edges(graph, block);
}

/**
 * A way with vertices that are not graph nodes. Those vertices stay in the ring --
 * a curved street has to stay curved -- and several consecutive ring segments then
 * share one owning half-edge, which is the case a per-vertex owner table gets
 * wrong if it is filled in per EDGE rather than per POINT.
 */
TEST(Blocks, ring_keeps_interior_polyline_vertices_and_attributes_them) {
    NodeMap positions = square_100m();
    positions[30] = glm::dvec2{120.0, 50.0};    // bulge on the eastern side

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2});
    add_way(data, positions, 11, {2, 30, 3});   // node 30 is on one way only
    add_way(data, positions, 12, {3, 4});
    add_way(data, positions, 13, {4, 1});

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{4});      // node 30 stays a plain vertex
    CHECK_EQ(graph.edges().size(), size_t{4});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    if (extraction.blocks.empty()) return;

    const Block& block = extraction.blocks.front();
    CHECK_EQ(block.ring.size(), size_t{5});
    CHECK_TRUE(ring_contains(block.ring, glm::dvec2{120.0, 50.0}));
    // Square plus the triangle the bulge adds: 10000 + 0.5 * 100 * 20.
    CHECK_NEAR(block.area, 11000.0, 1e-9);
    check_ring_edges(graph, block);
}

// ============================================================================
// Multiple Components
// ============================================================================

/**
 * Two squares that share no node. The small one is DELIBERATELY smaller than the
 * large one's block, so an implementation that discards the largest face, or the
 * face with the largest absolute area, loses the large block and keeps a face it
 * should have thrown away. Only the sign test gets this right.
 */
TEST(Blocks, disconnected_components_each_lose_their_own_outer_face) {
    const NodeMap positions{
        {1, {0.0, 0.0}},     {2, {100.0, 0.0}},   {3, {100.0, 100.0}}, {4, {0.0, 100.0}},
        {5, {500.0, 500.0}}, {6, {520.0, 500.0}}, {7, {520.0, 520.0}}, {8, {500.0, 520.0}},
    };

    ParsedOSMData data;
    add_square_100m(data, positions);
    add_way(data, positions, 20, {5, 6});
    add_way(data, positions, 21, {6, 7});
    add_way(data, positions, 22, {7, 8});
    add_way(data, positions, 23, {8, 5});

    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{2});
    CHECK_EQ(extraction.stats.faces, size_t{4});
    CHECK_EQ(extraction.stats.outer_faces, size_t{2});
    CHECK_EQ(count_area(extraction.blocks, 10000.0), size_t{1});
    CHECK_EQ(count_area(extraction.blocks, 400.0), size_t{1});

    // The two rings are not mixed: the small block is entirely in its own corner.
    for (const Block& block : extraction.blocks) {
        const bool small = block.area < 1000.0;
        for (const glm::dvec2& p : block.ring) {
            CHECK_EQ(small, (p.x > 400.0));
        }
    }
}

/**
 * A component that is a tree encloses nothing and must produce no block at all.
 *
 * Every edge of a tree is a cut edge -- the same face is on both sides of all of
 * them -- so once they are taken off the boundary there is no boundary left. That
 * is the branch this lands on, and it is counted as a tree face and not as a
 * malformed one: a dead-end road is the commonest thing in an extract and must not
 * read as a broken graph. The zero-area BAND is a different branch and a different
 * fixture; see a_collinear_cycle_is_a_face_with_no_interior.
 */
TEST(Blocks, a_tree_component_produces_no_block) {
    const NodeMap positions{
        {1, {0.0, 0.0}}, {2, {100.0, 0.0}}, {3, {100.0, 80.0}}, {4, {200.0, 0.0}},
    };

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2});
    add_way(data, positions, 11, {2, 3});
    add_way(data, positions, 12, {2, 4});

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.edges().size(), size_t{3});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{0});
    CHECK_EQ(extraction.stats.faces, size_t{1});
    CHECK_EQ(extraction.stats.expected_faces, size_t{1});   // 3 - 4 + 2 * 1
    CHECK_EQ(extraction.stats.tree_faces, size_t{1});
    CHECK_EQ(extraction.stats.zero_area_faces, size_t{0});
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
    CHECK_EQ(extraction.stats.outer_faces, size_t{0});
    CHECK_EQ(extraction.stats.half_edges_walked, size_t{6});
}

/**
 * The same tree, with a BEND in one of its dead ends.
 *
 * This is the fixture that says which pass is doing the work. A straight dead end
 * walked out and back leaves a two-point curve behind, which is dropped for being
 * too short whatever removed it. Put a plain polyline vertex in the middle and the
 * curve is four points long -- out along the bend and back -- so it is no longer
 * too short, only zero in area, and an implementation that leans on the geometry
 * reports a face with a boundary and no interior.
 *
 * It has neither. Every edge of a tree is a cut edge, and removing cut edges
 * leaves no boundary at all. That is a tree face, and it is the answer here
 * because the removal is topological: whether a dead end bends is not a fact
 * about whether it encloses anything.
 */
TEST(Blocks, a_dead_end_with_a_bend_in_it_still_encloses_nothing) {
    const NodeMap positions{
        {1, {0.0, 0.0}},   {2, {100.0, 0.0}}, {3, {150.0, 80.0}},
        {4, {200.0, 0.0}}, {30, {120.0, 20.0}},
    };

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2});
    add_way(data, positions, 11, {2, 30, 3});   // node 30 stays a plain polyline vertex
    add_way(data, positions, 12, {2, 4});

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{4});
    CHECK_EQ(graph.edges().size(), size_t{3});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{0});
    CHECK_EQ(extraction.stats.expected_faces, size_t{1});   // 3 - 4 + 2 * 1
    CHECK_EQ(extraction.stats.faces, size_t{1});
    CHECK_EQ(extraction.stats.tree_faces, size_t{1});
    CHECK_EQ(extraction.stats.zero_area_faces, size_t{0});
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
    CHECK_EQ(extraction.stats.rejected_small, size_t{0});
}

TEST(Blocks, an_empty_graph_yields_no_blocks) {
    ParsedOSMData data;
    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{0});
    CHECK_EQ(extraction.stats.faces, size_t{0});
    CHECK_EQ(extraction.stats.edges_considered, size_t{0});
}

// ============================================================================
// Non-convex Blocks
// ============================================================================

/**
 * An L-shaped block. Its convex hull encloses 8750 m^2 and the L itself 7500, so
 * any implementation that orders the ring by angle about a centre point -- which
 * is the usual shortcut for "put these boundary points in order" -- reports the
 * hull area and fails here.
 */
TEST(Blocks, non_convex_block_keeps_its_reflex_corner) {
    const NodeMap positions{
        {1, {0.0, 0.0}},   {2, {100.0, 0.0}}, {3, {100.0, 50.0}},
        {4, {50.0, 50.0}}, {5, {50.0, 100.0}}, {6, {0.0, 100.0}},
    };

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2});
    add_way(data, positions, 11, {2, 3});
    add_way(data, positions, 12, {3, 4});
    add_way(data, positions, 13, {4, 5});
    add_way(data, positions, 14, {5, 6});
    add_way(data, positions, 15, {6, 1});

    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    if (extraction.blocks.empty()) return;

    const Block& block = extraction.blocks.front();
    CHECK_NEAR(block.area, 7500.0, 1e-9);       // hull would be 8750
    CHECK_NEAR(block.perimeter, 400.0, 1e-9);
    CHECK_EQ(block.ring.size(), size_t{6});
    CHECK_TRUE(ring_contains(block.ring, glm::dvec2{50.0, 50.0}));   // the reflex corner
    check_ring_edges(graph, block);
}

// ============================================================================
// Degenerate Faces
// ============================================================================

/**
 * Two ways between the same pair of nodes. The sliver between them is a real face
 * with a real positive area, so nothing about the traversal rejects it -- only the
 * area floor does. Both halves are asserted: rejected at the default, kept when
 * the floor is lowered, which is what proves the floor is the thing doing the
 * rejecting.
 */
TEST(Blocks, a_sliver_between_duplicate_ways_falls_under_the_area_floor) {
    const NodeMap positions{
        {1, {0.0, 0.0}}, {2, {100.0, 0.0}}, {3, {50.0, 1.0}},
    };

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2});       // straight
    add_way(data, positions, 11, {1, 3, 2});    // bowed by 1 m

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{2});
    CHECK_EQ(graph.edges().size(), size_t{2});

    const BlockExtraction rejected = extract_blocks(graph);
    CHECK_EQ(rejected.blocks.size(), size_t{0});
    CHECK_EQ(rejected.stats.rejected_small, size_t{1});
    CHECK_EQ(rejected.stats.outer_faces, size_t{1});
    CHECK_EQ(rejected.stats.faces, size_t{2});

    BlockConfig permissive;
    permissive.min_area = 1.0;
    const BlockExtraction kept = extract_blocks(graph, permissive);
    CHECK_EQ(kept.blocks.size(), size_t{1});
    CHECK_EQ(kept.stats.rejected_small, size_t{0});
    if (kept.blocks.empty()) return;

    // Triangle of base 100 m and height 1 m.
    CHECK_NEAR(kept.blocks.front().area, 50.0, 1e-9);
}

// ============================================================================
// Bridges and Tunnels
// ============================================================================

/**
 * A bridge crossing the block without sharing a node. The two ways cross on
 * screen, so any implementation that intersects geometry cuts the block in two or
 * adds vertices to its ring. The block must come out untouched, and the bridge
 * must come out as a tree component of its own.
 */
TEST(Blocks, a_bridge_crossing_without_a_shared_node_does_not_split_the_block) {
    NodeMap positions = square_100m();
    positions[20] = glm::dvec2{-20.0, 50.0};
    positions[21] = glm::dvec2{120.0, 50.0};

    ParsedOSMData data;
    add_square_100m(data, positions);

    WayOptions bridge;
    bridge.layer = 1;
    bridge.is_bridge = true;
    add_way(data, positions, 14, {20, 21}, bridge);   // passes clean over the square

    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    CHECK_EQ(extraction.stats.faces, size_t{3});            // block, its outer, the bridge's
    CHECK_EQ(extraction.stats.outer_faces, size_t{1});
    CHECK_EQ(extraction.stats.tree_faces, size_t{1});       // the bridge is a tree
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
    CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
    if (extraction.blocks.empty()) return;

    const Block& block = extraction.blocks.front();
    CHECK_NEAR(block.area, 10000.0, 1e-9);
    CHECK_EQ(block.ring.size(), size_t{4});     // no vertex invented where the ways cross
    CHECK_EQ(block.edges.size(), size_t{4});
    CHECK_FALSE(block.has_grade_separated_edge);
}

/**
 * A bridge that IS part of the boundary. The flag has to be set, and switching the
 * edge class off has to take the boundary with it -- the remaining three sides are
 * a path, which encloses nothing.
 */
TEST(Blocks, a_bridge_on_the_boundary_is_flagged_and_can_be_excluded) {
    const NodeMap positions = square_100m();

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2});
    WayOptions bridge;
    bridge.layer = 1;
    bridge.is_bridge = true;
    add_way(data, positions, 11, {2, 3}, bridge);
    add_way(data, positions, 12, {3, 4});
    add_way(data, positions, 13, {4, 1});

    RoadGraph graph;
    graph.build(data);
    // Nodes 2 and 3 are endpoints of every way that touches them, so RoadGraph
    // treats them as bridge abutments and does NOT split them by layer. The square
    // stays one component.
    CHECK_EQ(graph.nodes().size(), size_t{4});
    CHECK_EQ(graph.edges().size(), size_t{4});

    const BlockExtraction included = extract_blocks(graph);
    CHECK_EQ(included.blocks.size(), size_t{1});
    if (!included.blocks.empty()) {
        CHECK_TRUE(included.blocks.front().has_grade_separated_edge);
        CHECK_NEAR(included.blocks.front().area, 10000.0, 1e-9);
    }

    BlockConfig surface_only;
    surface_only.include_grade_separated = false;
    const BlockExtraction excluded = extract_blocks(graph, surface_only);
    CHECK_EQ(excluded.blocks.size(), size_t{0});
    CHECK_EQ(excluded.stats.edges_used, size_t{3});
    CHECK_EQ(excluded.stats.faces, size_t{1});              // the remaining path
    CHECK_EQ(excluded.stats.tree_faces, size_t{1});
    CHECK_EQ(excluded.stats.half_edges_walked, size_t{6});
}

// ============================================================================
// Class Filtering
// ============================================================================

/**
 * A footway across the middle of a block. By default it must not divide the land,
 * and turning it on must divide it into exactly two halves. Both directions are
 * asserted, because a filter that drops the footway from the OUTPUT rather than
 * from the TRAVERSAL leaves the two half-blocks behind and reports two either way.
 */
TEST(Blocks, footways_do_not_divide_a_block_unless_asked) {
    const NodeMap positions{
        {1, {0.0, 0.0}},  {2, {100.0, 0.0}}, {3, {100.0, 100.0}}, {4, {0.0, 100.0}},
        {5, {0.0, 50.0}}, {6, {100.0, 50.0}},
    };

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2});
    add_way(data, positions, 11, {2, 6, 3});
    add_way(data, positions, 12, {3, 4});
    add_way(data, positions, 13, {4, 5, 1});

    WayOptions footway;
    footway.type = RoadType::Footway;
    add_way(data, positions, 14, {5, 6}, footway);

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{6});
    CHECK_EQ(graph.edges().size(), size_t{7});

    const BlockExtraction excluded = extract_blocks(graph);
    CHECK_EQ(excluded.stats.edges_used, size_t{6});
    CHECK_EQ(excluded.blocks.size(), size_t{1});
    if (!excluded.blocks.empty()) {
        CHECK_NEAR(excluded.blocks.front().area, 10000.0, 1e-9);
        // The footway's end nodes are still graph nodes, so they stay in the ring.
        CHECK_EQ(excluded.blocks.front().ring.size(), size_t{6});
        check_ring_edges(graph, excluded.blocks.front());
    }

    BlockConfig with_paths;
    with_paths.include_paths = true;
    const BlockExtraction included = extract_blocks(graph, with_paths);
    CHECK_EQ(included.stats.edges_used, size_t{7});
    CHECK_EQ(included.blocks.size(), size_t{2});
    CHECK_EQ(count_area(included.blocks, 5000.0), size_t{2});
}

// ============================================================================
// Duplicate Ways
// ============================================================================

/**
 * Two ways mapped along the same ground between the same two nodes. RoadGraph
 * gives their arms IDENTICAL bearings at both ends and sorts arms by bearing with
 * a non-stable sort, so the rotation it produces is not an embedding: the orbits
 * collapse and what disappears is not the zero-width sliver between the two ways,
 * it is the 10000 m^2 block around them. The only signal was one more face that
 * came to nothing, which is what a healthy dead-end road also looks like.
 *
 * The existing sliver test cannot reach this, because it bows its second way by a
 * metre and a bow gives the two arms different bearings.
 */
TEST(Blocks, an_exact_duplicate_way_does_not_take_the_block_with_it) {
    const NodeMap positions = square_100m();
    ParsedOSMData data;
    add_square_100m(data, positions);
    add_way(data, positions, 14, {1, 2});   // the same ground as way 10

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.edges().size(), size_t{5});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.duplicate_edges, size_t{1});
    CHECK_EQ(extraction.stats.edges_used, size_t{4});
    CHECK_EQ(extraction.stats.expected_faces, size_t{2});
    CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    if (extraction.blocks.empty()) return;

    CHECK_NEAR(extraction.blocks.front().area, 10000.0, 1e-9);
    CHECK_EQ(extraction.blocks.front().ring.size(), size_t{4});

    // The dropped edge is not topology, so it fronts nothing either.
    for (const BlockEdge& half : extraction.blocks.front().edges) {
        CHECK(graph.edge(half.edge).source_way != stratum::osm::WayId{14});
    }
}

/**
 * The same duplicate drawn the other way round, which is what a second survey of
 * the same street usually looks like. The arms still tie -- a bearing does not
 * care which end of the way the drawing started at -- so this has to be caught by
 * the geometry and not by comparing node id lists.
 */
TEST(Blocks, a_duplicate_way_drawn_backwards_is_still_a_duplicate) {
    const NodeMap positions = square_100m();
    ParsedOSMData data;
    add_square_100m(data, positions);
    add_way(data, positions, 14, {2, 1});

    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.duplicate_edges, size_t{1});
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    CHECK_EQ(count_area(extraction.blocks, 10000.0), size_t{1});
}

/**
 * Two ways that leave a node along the SAME ground and only part company further
 * out. Their arms carry exactly the same bearing, because a bearing is taken from
 * the first polyline segment and the first segment is shared, so the sort that
 * orders arms by bearing has nothing to order them by. It is not a stable sort,
 * so what it leaves is whatever introsort happened to produce.
 *
 * Unlike a full duplicate this pair is real topology -- the chord genuinely
 * divides the square into 3750 and 6250 -- so it cannot be dropped; it has to be
 * ORDERED. The order is decided where the two paths separate: the chord turns
 * north at (50,0), so it leaves the node at the greater bearing and sits after the
 * southern side in the rotation. Put it before and the rotation is not an
 * embedding: the three faces collapse into one orbit that abandons itself.
 *
 * All four ways of writing the same fixture are built, because the defect is only
 * visible in the ones the sort happens to get wrong, and which ones those are is a
 * property of the standard library rather than of this file. Asserting one layout
 * would be asserting today's introsort.
 */
TEST(Blocks, ways_leaving_a_node_on_one_bearing_are_ordered_by_where_they_part) {
    const NodeMap positions{
        {1, {0.0, 0.0}},   {2, {100.0, 0.0}}, {3, {100.0, 100.0}},
        {4, {0.0, 100.0}}, {5, {50.0, 50.0}}, {6, {50.0, 0.0}},
    };

    for (int layout = 0; layout < 4; ++layout) {
        const bool chord_first = (layout & 1) != 0;
        const bool chord_reversed = (layout & 2) != 0;

        ParsedOSMData data;
        const auto add_chord = [&]() {
            // Runs along the southern side as far as (50,0), then turns north.
            if (chord_reversed) add_way(data, positions, 14, {5, 6, 1});
            else                add_way(data, positions, 14, {1, 6, 5});
            add_way(data, positions, 15, {5, 3});
        };
        if (chord_first) add_chord();
        add_square_100m(data, positions);
        if (!chord_first) add_chord();

        RoadGraph graph;
        graph.build(data);
        CHECK_EQ(graph.nodes().size(), size_t{5});      // node 6 stays a plain vertex
        CHECK_EQ(graph.edges().size(), size_t{6});

        // The tie is real: two arms at (0,0) on bit-for-bit the same bearing.
        size_t tied = 0;
        for (const auto& node : graph.nodes()) {
            for (size_t i = 0; i < node.arms.size(); ++i) {
                for (size_t j = i + 1; j < node.arms.size(); ++j) {
                    if (node.arms[i].bearing == node.arms[j].bearing) ++tied;
                }
            }
        }
        CHECK_EQ(tied, size_t{1});

        const BlockExtraction extraction = extract_blocks(graph);
        CHECK_EQ(extraction.stats.duplicate_edges, size_t{0});   // real topology, not a duplicate
        CHECK_EQ(extraction.stats.expected_faces, size_t{3});    // 6 - 5 + 2 * 1
        CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
        CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
        CHECK_EQ(extraction.blocks.size(), size_t{2});
        CHECK_EQ(count_area(extraction.blocks, 6250.0), size_t{1});
        CHECK_EQ(count_area(extraction.blocks, 3750.0), size_t{1});
        CHECK_EQ(count_area(extraction.blocks, 10000.0), size_t{0});   // the two never merge
    }
}

/**
 * The same tie, with the second way peeling off on the OTHER side: it runs along
 * the southern boundary to (50,0) and then turns away from the square. Here the
 * square must survive whole and the chord must cut a 1250 m^2 triangle off the
 * land below it -- the mirror answer to the test above, from the mirror turn, so
 * a tie-break that always picks the same side of the pair gets one of the two
 * wrong.
 */
TEST(Blocks, a_way_that_peels_off_the_other_way_takes_the_other_side_of_the_tie) {
    const NodeMap positions{
        {1, {0.0, 0.0}},   {2, {100.0, 0.0}},   {3, {100.0, 100.0}},
        {4, {0.0, 100.0}}, {5, {50.0, -50.0}},  {6, {50.0, 0.0}},
    };

    for (int layout = 0; layout < 4; ++layout) {
        const bool chord_first = (layout & 1) != 0;
        const bool chord_reversed = (layout & 2) != 0;

        ParsedOSMData data;
        const auto add_chord = [&]() {
            if (chord_reversed) add_way(data, positions, 14, {5, 6, 1});
            else                add_way(data, positions, 14, {1, 6, 5});
            add_way(data, positions, 15, {5, 2});
        };
        if (chord_first) add_chord();
        add_square_100m(data, positions);
        if (!chord_first) add_chord();

        RoadGraph graph;
        graph.build(data);

        const BlockExtraction extraction = extract_blocks(graph);
        CHECK_EQ(extraction.stats.expected_faces, size_t{3});
        CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
        CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
        CHECK_EQ(extraction.blocks.size(), size_t{2});
        CHECK_EQ(count_area(extraction.blocks, 10000.0), size_t{1});   // the square is untouched
        CHECK_EQ(count_area(extraction.blocks, 1250.0), size_t{1});
    }
}

/**
 * A duplicate in the middle of a network, where the damage is a MERGE rather than
 * a loss: the two western cells of the grid came out as one 20000 m^2 block and
 * the count fell from four to three, with no counter anywhere saying so.
 */
TEST(Blocks, a_duplicate_of_one_grid_edge_does_not_merge_two_blocks) {
    const NodeMap positions = grid_3x3_nodes();
    ParsedOSMData data;
    add_grid_3x3(data, positions);
    add_way(data, positions, 16, {4, 5});   // duplicates the western half of way 11

    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.duplicate_edges, size_t{1});
    CHECK_EQ(extraction.blocks.size(), size_t{4});
    CHECK_EQ(count_area(extraction.blocks, 10000.0), size_t{4});
    CHECK_EQ(count_area(extraction.blocks, 20000.0), size_t{0});
    CHECK_EQ(extraction.stats.faces, extraction.stats.expected_faces);
}

/**
 * A way drawn over another with an extra vertex in the middle of it. The two
 * polylines are different lists of points and the same piece of ground, so a
 * duplicate test that compares vertices misses it and the bearings tie anyway.
 * Comparison is by arc length for exactly this.
 */
TEST(Blocks, a_duplicate_with_a_different_vertex_count_is_still_a_duplicate) {
    NodeMap positions = square_100m();
    positions[30] = glm::dvec2{50.0, 0.0};      // midpoint of the southern side

    ParsedOSMData data;
    add_square_100m(data, positions);
    add_way(data, positions, 14, {1, 30, 2});

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{4});      // node 30 stays a plain vertex

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.duplicate_edges, size_t{1});
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    CHECK_EQ(count_area(extraction.blocks, 10000.0), size_t{1});
}

/**
 * Turning the duplicate filter off restores the old behaviour exactly, which is
 * what proves the filter is the thing doing the work above and not some other
 * change. It is a diagnostic switch, so what it shows is the damage: no block, and
 * a count that says why.
 */
TEST(Blocks, keeping_duplicates_is_what_loses_the_block) {
    const NodeMap positions = square_100m();
    ParsedOSMData data;
    add_square_100m(data, positions);
    add_way(data, positions, 14, {1, 2});

    RoadGraph graph;
    graph.build(data);

    BlockConfig keep;
    keep.drop_duplicate_edges = false;
    const BlockExtraction extraction = extract_blocks(graph, keep);
    CHECK_EQ(extraction.stats.duplicate_edges, size_t{0});
    CHECK_EQ(extraction.stats.edges_used, size_t{5});
    CHECK_EQ(extraction.blocks.size(), size_t{0});
    // Euler says three faces for five edges over four nodes. The traversal finds
    // fewer, because the rotation it was handed is not an embedding.
    CHECK_EQ(extraction.stats.expected_faces, size_t{3});
    CHECK((extraction.stats.faces) < (extraction.stats.expected_faces));
}

// ============================================================================
// Faces That Come To Nothing
// ============================================================================

/**
 * A cycle whose nodes are collinear. It has a boundary of three points and no
 * interior at all, which is the case the zero-area band exists for and the only
 * one that reaches it: a tree component is caught a branch earlier, because a tree
 * has no boundary left once its cut edges are removed.
 *
 * Without the band the shoelace sum here is exactly 0.0 and the face falls through
 * to the area floor, counted as a block that was merely too small. That is the
 * wrong answer to the wrong question: it is not small, it is not a face of any
 * land, and on a fixture whose rounding lands the other side of zero the same
 * implementation calls it an outer face instead.
 */
TEST(Blocks, a_collinear_cycle_is_a_face_with_no_interior) {
    const NodeMap positions{
        {1, {0.0, 0.0}}, {2, {100.0, 0.0}}, {3, {50.0, 0.0}},
    };

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 3});
    add_way(data, positions, 11, {3, 2});
    add_way(data, positions, 12, {2, 1});

    RoadGraph graph;
    graph.build(data);
    CHECK_EQ(graph.nodes().size(), size_t{3});
    CHECK_EQ(graph.edges().size(), size_t{3});

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{0});
    CHECK_EQ(extraction.stats.expected_faces, size_t{2});   // 3 - 3 + 2 * 1
    CHECK_EQ(extraction.stats.faces, size_t{2});
    CHECK_EQ(extraction.stats.zero_area_faces, size_t{2});
    CHECK_EQ(extraction.stats.tree_faces, size_t{0});
    CHECK_EQ(extraction.stats.rejected_small, size_t{0});
    CHECK_EQ(extraction.stats.outer_faces, size_t{0});
    CHECK_EQ(extraction.stats.malformed_faces, size_t{0});
}

/**
 * A tunnel that shares BOTH its end nodes with the square but is drawn looping
 * away outside it, so it crosses two sides of the square without sharing a node
 * there. The topology says three faces; the drawing has a crossing the data does
 * not know about, so the rotation is not a planar embedding and the traversal
 * finds fewer orbits than there are faces.
 *
 * Nothing in the block list can say this. A block that does not exist looks
 * exactly like a block that was never there, and before Euler's formula was
 * carried the only trace was one more face that came to nothing -- the same count
 * a healthy dead-end road increments. What this test pins is that the shortfall is
 * VISIBLE, not that the traversal recovers: it cannot, and inventing the junction
 * from the crossing is the one thing this pipeline must never do.
 */
TEST(Blocks, a_crossing_without_a_shared_node_shows_up_as_a_missing_face) {
    NodeMap positions = square_100m();
    positions[30] = glm::dvec2{-400.0, 50.0};

    ParsedOSMData data;
    add_square_100m(data, positions);

    WayOptions tunnel;
    tunnel.layer = -1;
    add_way(data, positions, 14, {2, 30, 4}, tunnel);

    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.edges_used, size_t{5});
    CHECK_EQ(extraction.stats.expected_faces, size_t{3});    // 5 - 4 + 2 * 1
    CHECK((extraction.stats.faces) < (extraction.stats.expected_faces));
    CHECK((extraction.blocks.size()) < (size_t{2}));
}

// ============================================================================
// Ring Helpers
// ============================================================================

/**
 * The orientation of signed_ring_area() is part of the published contract: C2
 * builds lot rings and has to agree with this file about which way round positive
 * is. A helper that returned an absolute area would pass every block test above
 * and silently break that agreement.
 */
TEST(Blocks, signed_ring_area_is_positive_anticlockwise) {
    const std::vector<glm::dvec2> anticlockwise{
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
    const std::vector<glm::dvec2> clockwise{
        {0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}};

    CHECK_NEAR(stratum::osm::road::signed_ring_area(anticlockwise), 100.0, 1e-12);
    CHECK_NEAR(stratum::osm::road::signed_ring_area(clockwise), -100.0, 1e-12);
    CHECK_NEAR(stratum::osm::road::signed_ring_area({{0.0, 0.0}, {1.0, 1.0}}), 0.0, 1e-12);
}

TEST(Blocks, grade_separation_covers_layer_as_well_as_the_tags) {
    GraphEdge plain;
    CHECK_FALSE(stratum::osm::road::is_grade_separated(plain));

    GraphEdge bridged;
    bridged.is_bridge = true;
    CHECK_TRUE(stratum::osm::road::is_grade_separated(bridged));

    GraphEdge tunnelled;
    tunnelled.is_tunnel = true;
    CHECK_TRUE(stratum::osm::road::is_grade_separated(tunnelled));

    // A road in a cutting carries layer=-1 and neither tag, and it crosses other
    // ways without sharing a node exactly as a tunnel does.
    GraphEdge sunken;
    sunken.layer = -1;
    CHECK_TRUE(stratum::osm::road::is_grade_separated(sunken));
}

