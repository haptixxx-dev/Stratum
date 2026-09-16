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

/// Index of the ring point within @p eps of @p point, or ring.size() when absent
size_t ring_index_of(const std::vector<glm::dvec2>& ring, const glm::dvec2& point,
                     double eps = 1e-9) {
    for (size_t i = 0; i < ring.size(); ++i) {
        if (points_equal(ring[i], point, eps)) {
            return i;
        }
    }
    return ring.size();
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
    CHECK_EQ(extraction.stats.degenerate_faces, size_t{0});
    if (extraction.blocks.empty()) return;

    const Block& block = extraction.blocks.front();
    CHECK_EQ(block.id, uint32_t{0});
    CHECK_NEAR(block.area, 10000.0, 1e-9);
    CHECK_NEAR(block.perimeter, 400.0, 1e-9);
    CHECK_EQ(block.ring.size(), size_t{4});
    CHECK_EQ(block.edges.size(), size_t{4});
    CHECK_FALSE(block.has_grade_separated_edge);
}

/**
 * The orientation contract, asserted as a vertex ORDER rather than as a sign, so
 * it cannot be satisfied by a ring that happens to have the corners in some other
 * rotation. An anticlockwise-next traversal produces exactly the reverse of this
 * sequence, which this test fails on the first comparison.
 */
TEST(Blocks, block_ring_runs_anticlockwise) {
    const NodeMap positions = square_100m();
    ParsedOSMData data;
    add_square_100m(data, positions);

    RoadGraph graph;
    graph.build(data);
    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    if (extraction.blocks.empty()) return;

    const std::vector<glm::dvec2>& ring = extraction.blocks.front().ring;
    CHECK_EQ(ring.size(), size_t{4});
    if (ring.size() != 4) return;

    const size_t origin = ring_index_of(ring, glm::dvec2{0.0, 0.0});
    CHECK((origin) < (ring.size()));
    if (origin >= ring.size()) return;

    // Anticlockwise from (0,0): east, then north, then west. A clockwise ring
    // visits the same three corners in the opposite order, so each of these fails.
    CHECK_TRUE(points_equal(ring[(origin + 1) % 4], glm::dvec2{100.0, 0.0}));
    CHECK_TRUE(points_equal(ring[(origin + 2) % 4], glm::dvec2{100.0, 100.0}));
    CHECK_TRUE(points_equal(ring[(origin + 3) % 4], glm::dvec2{0.0, 100.0}));
}

/**
 * The property C4 depends on: the block is on the LEFT of each bounding
 * half-edge. Asserted per half-edge from the graph geometry, not from the ring,
 * so it also catches a BlockEdge::forward flag written the wrong way round while
 * the ring itself came out right.
 */
TEST(Blocks, block_lies_left_of_every_bounding_half_edge) {
    const NodeMap positions = square_100m();
    ParsedOSMData data;
    add_square_100m(data, positions);

    RoadGraph graph;
    graph.build(data);
    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    if (extraction.blocks.empty()) return;

    const Block& block = extraction.blocks.front();
    const glm::dvec2 interior{50.0, 50.0};   // inside the square by inspection

    for (const BlockEdge& half : block.edges) {
        const GraphEdge& edge = graph.edge(half.edge);
        const glm::dvec2 start = half.forward ? edge.polyline.front() : edge.polyline.back();
        const glm::dvec2 end = half.forward ? edge.polyline.back() : edge.polyline.front();
        CHECK((0.0) < (cross_z(start, end, interior)));
    }
}

// ============================================================================
// A 2x2 Grid
// ============================================================================

/**
 * Four blocks, not five. The fifth face is the outer one, and an implementation
 * that keeps it reports five here while every other count still looks plausible.
 */
TEST(Blocks, grid_2x2_yields_four_blocks_not_five) {
    const NodeMap positions{
        {1, {0.0, 0.0}},   {2, {100.0, 0.0}},   {3, {200.0, 0.0}},
        {4, {0.0, 100.0}}, {5, {100.0, 100.0}}, {6, {200.0, 100.0}},
        {7, {0.0, 200.0}}, {8, {100.0, 200.0}}, {9, {200.0, 200.0}},
    };

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2, 3});
    add_way(data, positions, 11, {4, 5, 6});
    add_way(data, positions, 12, {7, 8, 9});
    add_way(data, positions, 13, {1, 4, 7});
    add_way(data, positions, 14, {2, 5, 8});
    add_way(data, positions, 15, {3, 6, 9});

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
    for (const Block& block : extraction.blocks) {
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
 * The bijection the whole traversal rests on. If any half-edge were visited twice
 * or missed, this count would not land on 2 * edges_used, and the block count
 * above could still be right by accident.
 */
TEST(Blocks, every_half_edge_is_walked_exactly_once) {
    const NodeMap positions{
        {1, {0.0, 0.0}},   {2, {100.0, 0.0}},   {3, {200.0, 0.0}},
        {4, {0.0, 100.0}}, {5, {100.0, 100.0}}, {6, {200.0, 100.0}},
        {7, {0.0, 200.0}}, {8, {100.0, 200.0}}, {9, {200.0, 200.0}},
        {20, {150.0, 50.0}},    // cul-de-sac tip inside the lower right block
    };

    ParsedOSMData data;
    add_way(data, positions, 10, {1, 2, 3});
    add_way(data, positions, 11, {4, 5, 6});
    add_way(data, positions, 12, {7, 8, 9});
    add_way(data, positions, 13, {1, 4, 7});
    add_way(data, positions, 14, {2, 5, 8});
    add_way(data, positions, 15, {3, 6, 9});
    add_way(data, positions, 16, {5, 20});

    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.stats.edges_used, graph.edges().size());
    CHECK_EQ(extraction.stats.half_edges_walked, graph.edges().size() * 2);
    CHECK_EQ(extraction.blocks.size(), size_t{4});
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
}

/**
 * The same cul-de-sac, but the spur way is drawn FROM its tip and listed first, so
 * the traversal seeds on the half-edge leaving the dead end and the antenna
 * straddles the start of the ring. The linear cancellation pass cannot see an
 * antenna split between the head and the tail of the walk -- only the cyclic pass
 * after it can -- and without that pass the ring keeps the tip and the closing
 * segment doubles back.
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
    check_ring_edges(graph, block);
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
 * The prune moves segment ownership as well as points. At the node the spur hangs
 * off, the surviving ring vertex leaves by the NEXT street, not by the spur it
 * originally walked into, and a stale owner names a half-edge that does not carry
 * the segment at all.
 */
TEST(Blocks, ring_edges_survive_the_antenna_prune) {
    NodeMap positions = square_100m();
    positions[5] = glm::dvec2{50.0, 50.0};

    ParsedOSMData data;
    add_square_100m(data, positions);
    add_way(data, positions, 14, {2, 5});

    RoadGraph graph;
    graph.build(data);

    const BlockExtraction extraction = extract_blocks(graph);
    CHECK_EQ(extraction.blocks.size(), size_t{1});
    if (extraction.blocks.empty()) return;

    check_ring_edges(graph, extraction.blocks.front());
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
 * Its single face has an area of zero, which is neither positive nor negative, so
 * an implementation that only tests `area < 0` for the outer face reports it as a
 * block of zero area instead of discarding it.
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
    CHECK_EQ(extraction.stats.degenerate_faces, size_t{1});
    CHECK_EQ(extraction.stats.outer_faces, size_t{0});
    CHECK_EQ(extraction.stats.half_edges_walked, size_t{6});
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
    CHECK_EQ(extraction.stats.degenerate_faces, size_t{1}); // the bridge is a tree
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
    CHECK_EQ(excluded.stats.degenerate_faces, size_t{1});
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
