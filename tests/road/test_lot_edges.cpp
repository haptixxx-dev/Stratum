// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_lot_edges.cpp
 * @brief Front / side / back / interior classification against lots drawn by hand
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Every fixture is a REAL block: a road graph built from a node table, run through
 * extract_blocks(), with the lots cut out of the resulting ring by hand. Nothing
 * here fabricates a Block, because the whole contract of this stage is the join
 * between `Block::edges` and `LotOutline::edge_sources`, and a hand-built Block
 * would let that join be wrong and the tests still pass. The one test that hands
 * over a block nobody could extract -- one half-edge rewritten to point past the
 * end of the graph -- starts from a real extraction and says so, because the check
 * it covers exists for a block that has gone stale.
 *
 * The main block is 100 m by 60 m and never square, and the lots inside it are
 * never square either. That is deliberate: a classifier that transposed x and y,
 * or that mirrored the ring, would still produce the right roles on a symmetric
 * fixture, and a suite built on one could not tell. The same reasoning is why the
 * bend of make_bent_bottom_block() is off centre and why the two frontage runs of
 * the notch lot are 20 m and 30 m -- symmetry hides weighting and tie-breaks. Where
 * a test needs a symmetric fixture, it is because the TIE is the thing under test,
 * and it says so.
 *
 * Two fixtures exist for shapes nothing else reaches. make_spur_block() hangs a
 * cul-de-sac inside the block, so one street bounds it twice and a lot at the head
 * of the spur fronts that one street from both sides. make_loop_block() rings a
 * block with a single closed way, so a lot filling it is frontage the whole way
 * round with no gap and no net direction.
 *
 * Roles are asserted as a comma-joined STRING covering every edge of the lot at
 * once. A failure then prints `actual: Front,Side,Front,Side  expected:
 * Front,Side,Back,Side`, which names the edge that moved; asserting the count of
 * one role instead would pass while two roles swapped.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests LotEdges
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/blocks.hpp"
#include "osm/road/lot_edges.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
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
using stratum::osm::road::BlockExtraction;
using stratum::osm::road::EdgeId;
using stratum::osm::road::LotEdgeClassification;
using stratum::osm::road::LotEdgeClassificationSet;
using stratum::osm::road::LotEdgeConfig;
using stratum::osm::road::LotEdgeRole;
using stratum::osm::road::LotFrontage;
using stratum::osm::road::LotOutline;
using stratum::osm::road::PrimaryFrontRule;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::StreetSide;
using stratum::osm::road::classify_block_lots;
using stratum::osm::road::classify_lot_edges;
using stratum::osm::road::extract_blocks;
using stratum::osm::road::is_street_class;
using stratum::osm::road::kNoBoundaryEdge;
using stratum::osm::road::lot_edge_role_name;
using stratum::osm::road::street_class_rank;
using stratum::osm::road::street_side_name;

// ============================================================================
// Way ids of the rectangular fixture
// ============================================================================

constexpr WayId kBottomWay = 10;
constexpr WayId kRightWay = 11;
constexpr WayId kTopWay = 12;
constexpr WayId kLeftWay = 13;

/// Dead-end spur hanging inside the block; see make_spur_block()
constexpr WayId kSpurWay = 14;

/// The single closed way of make_loop_block()
constexpr WayId kLoopWay = 20;

// ============================================================================
// Graph Construction
// ============================================================================

using NodeMap = std::unordered_map<NodeId, glm::dvec2>;

/// Append one way, taking its geometry point by point from @p positions
void add_way(ParsedOSMData& data, const NodeMap& positions, WayId id,
             const std::vector<NodeId>& ids, RoadType type) {
    Road road;
    road.osm_id = id;
    road.type = type;
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

/**
 * @brief A built graph and the single block extracted from it
 *
 * Holds the graph by value because every `Block` field that matters is an INDEX
 * into it. A fixture that returned only the extraction would leave every EdgeId
 * pointing at a destroyed graph, and the frontage assertions would read freed
 * memory rather than fail.
 */
struct BlockFixture {
    RoadGraph graph;
    BlockExtraction extraction;

    [[nodiscard]] bool ok() const { return extraction.blocks.size() == 1; }

    [[nodiscard]] const Block& block() const { return extraction.blocks.front(); }

    /**
     * @brief Index into Block::edges of the half-edge carrying OSM way @p way
     *
     * Looked up by OSM way id rather than hard-coded, because edge ids and the
     * traversal's starting half-edge are both implementation details of stages
     * this suite is not testing. A test that hard-codes "block edge 0 is the
     * bottom street" starts failing when RoadGraph changes its edge ordering, and
     * the failure says nothing about lot edges.
     */
    [[nodiscard]] uint32_t block_edge_for_way(WayId way) const {
        uint32_t found = kNoBoundaryEdge;
        size_t matches = 0;
        for (size_t i = 0; i < block().edges.size(); ++i) {
            const auto edge_id = block().edges[i].edge;
            if (edge_id < graph.edges().size() && graph.edge(edge_id).source_way == way) {
                found = static_cast<uint32_t>(i);
                ++matches;
            }
        }
        if (matches != 1) {
            stratum::test::report_failure(__FILE__, __LINE__, "exactly one block edge per way",
                                          "way " + std::to_string(way) + " matched " +
                                              std::to_string(matches));
        }
        return found;
    }

    /**
     * @brief Index into Block::edges of the half-edge carrying @p way in @p forward
     *
     * block_edge_for_way() above insists on exactly one match, which is the right
     * check for a street that bounds the block once and the wrong one for a
     * cul-de-sac spur, which bounds it twice -- once each way. The two half-edges
     * of a spur are what a lot at its head fronts from its two sides, so they have
     * to be told apart by direction rather than by way id.
     */
    [[nodiscard]] uint32_t block_edge_for_way_side(WayId way, bool forward) const {
        uint32_t found = kNoBoundaryEdge;
        size_t matches = 0;
        for (size_t i = 0; i < block().edges.size(); ++i) {
            const auto edge_id = block().edges[i].edge;
            if (edge_id < graph.edges().size() && graph.edge(edge_id).source_way == way &&
                block().edges[i].forward == forward) {
                found = static_cast<uint32_t>(i);
                ++matches;
            }
        }
        if (matches != 1) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "exactly one block edge per way and direction",
                                          "way " + std::to_string(way) + " matched " +
                                              std::to_string(matches));
        }
        return found;
    }

    /// OSM way id behind a frontage, for asserting WHICH street a lot fronts
    [[nodiscard]] WayId way_of(const LotFrontage& frontage) const {
        if (frontage.street >= graph.edges().size()) {
            return 0;
        }
        return graph.edge(frontage.street).source_way;
    }
};

/// Per-side road classes of the rectangular fixture
struct RectOptions {
    RoadType bottom = RoadType::Residential;
    RoadType right = RoadType::Residential;
    RoadType top = RoadType::Residential;
    RoadType left = RoadType::Residential;
    bool include_paths = false;
};

/**
 * @brief A 100 m by 60 m block bounded by four streets
 *
 * Corners: (0,0), (100,0), (100,60), (0,60).
 *
 * The top way is drawn LEFT TO RIGHT while the other three run anticlockwise, so
 * the block sits on the RIGHT of the top way's own direction. That asymmetry is
 * the only way this suite can tell StreetSide::Right from a constant.
 */
BlockFixture make_rect_block(const RectOptions& options = {}) {
    const NodeMap positions{{1, {0.0, 0.0}}, {2, {100.0, 0.0}}, {3, {100.0, 60.0}},
                            {4, {0.0, 60.0}}};

    ParsedOSMData data;
    add_way(data, positions, kBottomWay, {1, 2}, options.bottom);
    add_way(data, positions, kRightWay, {2, 3}, options.right);
    add_way(data, positions, kTopWay, {4, 3}, options.top);
    add_way(data, positions, kLeftWay, {4, 1}, options.left);

    BlockFixture fixture;
    fixture.graph.build(data);

    BlockConfig config;
    config.include_paths = options.include_paths;
    fixture.extraction = extract_blocks(fixture.graph, config);
    return fixture;
}

/**
 * @brief The same block with the bottom street bent up to (10, 10)
 *
 * Node 5 is referenced by one way only, so RoadGraph keeps it as a plain polyline
 * vertex and the bottom street stays ONE graph edge carrying two segments. That is
 * the case the frontage grouping has to survive.
 *
 * The bend is OFF CENTRE, and that is the whole value of the fixture. With it at
 * (50, 10) the two segments are the same length and the shape is mirror-symmetric,
 * so the length-weighted average of their normals and the plain average of their
 * normals are bit-identical -- and a test on the averaged normal cannot tell an
 * implementation that weights by length from one that does not. At (10, 10) the
 * segments are 14.14 m and 90.55 m, the weighted average is exactly -Y, and the
 * unweighted one is 19 degrees off it.
 */
BlockFixture make_bent_bottom_block() {
    const NodeMap positions{{1, {0.0, 0.0}},   {5, {10.0, 10.0}}, {2, {100.0, 0.0}},
                            {3, {100.0, 60.0}}, {4, {0.0, 60.0}}};

    ParsedOSMData data;
    add_way(data, positions, kBottomWay, {1, 5, 2}, RoadType::Residential);
    add_way(data, positions, kRightWay, {2, 3}, RoadType::Residential);
    add_way(data, positions, kTopWay, {4, 3}, RoadType::Residential);
    add_way(data, positions, kLeftWay, {4, 1}, RoadType::Residential);

    BlockFixture fixture;
    fixture.graph.build(data);
    fixture.extraction = extract_blocks(fixture.graph);
    return fixture;
}

/**
 * @brief The rectangular block with a dead-end spur hanging inside it
 *
 * Way 14 runs from (50, 0) -- a node the bottom street shares, so RoadGraph splits
 * the bottom street there -- north to (50, 25) and stops. The face traversal walks
 * UP one side of that spur and back DOWN the other, so the spur's graph edge
 * appears in `Block::edges` TWICE, once forward and once reversed, and it reaches
 * the ring neither time because it is a cut edge.
 *
 * This is the only shape in which two frontages of one lot can carry the same
 * `EdgeId`, so it is the only shape that can test the last stage of the
 * primary-front tie-break, and the only shape that can tell grouping by
 * `Block::edges` index from grouping by `EdgeId`.
 */
BlockFixture make_spur_block() {
    const NodeMap positions{{1, {0.0, 0.0}},    {6, {50.0, 0.0}},  {2, {100.0, 0.0}},
                            {3, {100.0, 60.0}}, {4, {0.0, 60.0}},  {7, {50.0, 25.0}}};

    ParsedOSMData data;
    add_way(data, positions, kBottomWay, {1, 6, 2}, RoadType::Residential);
    add_way(data, positions, kRightWay, {2, 3}, RoadType::Residential);
    add_way(data, positions, kTopWay, {4, 3}, RoadType::Residential);
    add_way(data, positions, kLeftWay, {4, 1}, RoadType::Residential);
    add_way(data, positions, kSpurWay, {6, 7}, RoadType::Residential);

    BlockFixture fixture;
    fixture.graph.build(data);
    fixture.extraction = extract_blocks(fixture.graph);
    return fixture;
}

/**
 * @brief A 40 m square block ringed by ONE closed way -- an estate loop road
 *
 * The way starts and ends at the same node, so RoadGraph keeps it as a single
 * graph edge whose polyline closes, and the block it encloses has exactly one
 * entry in `Block::edges`. Every ring segment therefore belongs to that one
 * half-edge, which is what makes a lot filling the block a frontage with no gap in
 * it and no net direction -- the two branches nothing else in this suite reaches.
 */
BlockFixture make_loop_block() {
    const NodeMap positions{{1, {0.0, 0.0}}, {2, {40.0, 0.0}}, {3, {40.0, 40.0}},
                            {4, {0.0, 40.0}}};

    ParsedOSMData data;
    add_way(data, positions, kLoopWay, {1, 2, 3, 4, 1}, RoadType::Residential);

    BlockFixture fixture;
    fixture.graph.build(data);
    fixture.extraction = extract_blocks(fixture.graph);
    return fixture;
}

// ============================================================================
// Assertion Helpers
// ============================================================================

/**
 * @brief Roles joined with commas, one entry per ring segment
 *
 * LotEdgeRole is a scoped enum with no operator<<, so CHECK_EQ on the enum itself
 * would report "<unprintable>" twice and name neither the actual nor the expected
 * role. Comparing the joined names costs a string and makes every failure legible.
 */
std::string role_string(const LotEdgeClassification& classification) {
    std::string out;
    for (const LotEdgeRole role : classification.roles) {
        if (!out.empty()) {
            out += ',';
        }
        out += lot_edge_role_name(role);
    }
    return out;
}

/// Lot A: 20 m wide, 30 m deep, sitting on the bottom street from x=20 to x=40
LotOutline lot_on_bottom(uint32_t bottom_edge) {
    LotOutline lot;
    lot.ring = {{20.0, 0.0}, {40.0, 0.0}, {40.0, 30.0}, {20.0, 30.0}};
    lot.edge_sources = {bottom_edge, kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};
    return lot;
}

/// A lot entirely inside the block, touching nothing
LotOutline lot_interior() {
    LotOutline lot;
    lot.ring = {{45.0, 20.0}, {60.0, 20.0}, {60.0, 35.0}, {45.0, 35.0}};
    lot.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};
    return lot;
}

/// Corner lot: @p along_bottom metres on the bottom street, @p along_left up the left
LotOutline lot_on_corner(uint32_t bottom_edge, uint32_t left_edge, double along_bottom,
                         double along_left) {
    LotOutline lot;
    lot.ring = {{0.0, 0.0}, {along_bottom, 0.0}, {along_bottom, along_left}, {0.0, along_left}};
    lot.edge_sources = {bottom_edge, kNoBoundaryEdge, kNoBoundaryEdge, left_edge};
    return lot;
}

} // namespace

// ============================================================================
// The Four Roles
// ============================================================================

/**
 * The base case, and the one every other test is a variation of. A mid-block lot
 * has one front on the street, one back away from it, and two sides.
 *
 * The lot is 20 by 30 rather than square so that a classifier which took the
 * normal of the wrong segment -- an off-by-one between `ring[i]` and the segment
 * leaving it -- would move a role rather than land on an identical answer.
 */
TEST(LotEdges, street_edge_is_front_and_opposite_edge_is_back) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const LotOutline lot = lot_on_bottom(fixture.block_edge_for_way(kBottomWay));
    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_FALSE(result.malformed);
    CHECK_FALSE(result.landlocked);
    CHECK_EQ(role_string(result), std::string("Front,Side,Back,Side"));
    CHECK_EQ(result.frontages.size(), size_t{1});
    CHECK_EQ(result.count(LotEdgeRole::Interior), size_t{0});
}

/**
 * The normal has to point at the STREET, not into the lot. Both signs produce the
 * same role array -- every normal flips together, so every dot product keeps its
 * sign -- so the role array cannot catch an inverted convention and this is the
 * only assertion that can.
 *
 * The street here is the bottom of the block, so the outward normal is -Y.
 */
TEST(LotEdges, front_normal_points_from_the_lot_towards_the_street) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const LotOutline lot = lot_on_bottom(fixture.block_edge_for_way(kBottomWay));
    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_NEAR(result.front_normal().x, 0.0, 1e-12);
    CHECK_NEAR(result.front_normal().y, -1.0, 1e-12);

    // Midway along the 20 m frontage from x=20 to x=40, on the street itself.
    CHECK_NEAR(result.front_midpoint().x, 30.0, 1e-9);
    CHECK_NEAR(result.front_midpoint().y, 0.0, 1e-9);
}

/**
 * A frontage names the street and which side of its centreline the lot is on, so
 * E3 never has to cross-product its way back to the answer.
 *
 * The lot on the TOP street is the one that proves `side` is read from
 * BlockEdge::forward rather than hard-coded: the top way is drawn left to right,
 * so the block -- and every lot in it -- is on the RIGHT of that centreline, while
 * the lot on the bottom street is on the LEFT.
 */
TEST(LotEdges, frontage_carries_the_street_id_and_the_side_of_its_centreline) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const LotOutline bottom_lot = lot_on_bottom(fixture.block_edge_for_way(kBottomWay));
    const LotEdgeClassification from_bottom =
        classify_lot_edges(bottom_lot, fixture.block(), fixture.graph);
    const LotFrontage* bottom_front = from_bottom.primary_front();
    CHECK_TRUE(bottom_front != nullptr);

    LotOutline top_lot;
    top_lot.ring = {{20.0, 40.0}, {40.0, 40.0}, {40.0, 60.0}, {20.0, 60.0}};
    top_lot.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, fixture.block_edge_for_way(kTopWay),
                            kNoBoundaryEdge};
    const LotEdgeClassification from_top =
        classify_lot_edges(top_lot, fixture.block(), fixture.graph);
    const LotFrontage* top_front = from_top.primary_front();
    CHECK_TRUE(top_front != nullptr);

    if (bottom_front == nullptr || top_front == nullptr) {
        return;
    }

    CHECK_EQ(fixture.way_of(*bottom_front), kBottomWay);
    CHECK_EQ(fixture.way_of(*top_front), kTopWay);
    CHECK_EQ(std::string(street_side_name(bottom_front->side)), std::string("Left"));
    CHECK_EQ(std::string(street_side_name(top_front->side)), std::string("Right"));

    // The top lot's front faces +Y, the opposite of the bottom lot's.
    CHECK_NEAR(from_top.front_normal().y, 1.0, 1e-12);
    CHECK_EQ(role_string(from_top), std::string("Back,Side,Front,Side"));
}

/**
 * A corner lot has two fronts and that is not an error. Both are reported, one is
 * marked primary, and the remaining edges are still classified against the primary
 * rather than left unclassified.
 */
TEST(LotEdges, corner_lot_has_two_fronts) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const LotOutline lot = lot_on_corner(fixture.block_edge_for_way(kBottomWay),
                                         fixture.block_edge_for_way(kLeftWay), 30.0, 12.0);
    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_FALSE(result.landlocked);
    CHECK_EQ(role_string(result), std::string("Front,Side,Back,Front"));
    CHECK_EQ(result.frontages.size(), size_t{2});

    size_t primaries = 0;
    for (const LotFrontage& frontage : result.frontages) {
        primaries += frontage.primary ? 1 : 0;
    }
    CHECK_EQ(primaries, size_t{1});

    CHECK_NEAR(result.frontages[0].length, 30.0, 1e-9);
    CHECK_NEAR(result.frontages[1].length, 12.0, 1e-9);
}

// ============================================================================
// Which Front Is Primary
// ============================================================================

/**
 * The primary front follows the geometry, not the order the frontages were found
 * in. Both halves of this test build the SAME corner, differing only in which
 * frontage is longer, so an implementation that returns `frontages[0]` passes the
 * first half and fails the second.
 */
TEST(LotEdges, primary_front_is_the_longer_frontage) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);
    const uint32_t left = fixture.block_edge_for_way(kLeftWay);

    const LotEdgeClassification wide =
        classify_lot_edges(lot_on_corner(bottom, left, 30.0, 12.0), fixture.block(),
                           fixture.graph);
    const LotEdgeClassification deep =
        classify_lot_edges(lot_on_corner(bottom, left, 12.0, 40.0), fixture.block(),
                           fixture.graph);

    const LotFrontage* wide_front = wide.primary_front();
    const LotFrontage* deep_front = deep.primary_front();
    CHECK_TRUE(wide_front != nullptr);
    CHECK_TRUE(deep_front != nullptr);
    if (wide_front == nullptr || deep_front == nullptr) {
        return;
    }

    CHECK_EQ(fixture.way_of(*wide_front), kBottomWay);
    CHECK_EQ(fixture.way_of(*deep_front), kLeftWay);

    // And the normals follow the primary, which is what a facade rule reads.
    CHECK_NEAR(wide.front_normal().y, -1.0, 1e-12);
    CHECK_NEAR(deep.front_normal().x, -1.0, 1e-12);
}

/**
 * A square corner lot at a crossroads has two frontages of identical length, which
 * is common and must not be settled by rounding. The more important road wins.
 *
 * The lot is 20 by 20 on purpose: the tie is exact, so an implementation whose
 * tie-break is "whichever compared greater" has nothing to compare.
 */
TEST(LotEdges, primary_front_breaks_a_length_tie_on_road_class) {
    RectOptions options;
    options.left = RoadType::Primary;   // the bottom stays Residential
    const BlockFixture fixture = make_rect_block(options);
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const LotOutline lot = lot_on_corner(fixture.block_edge_for_way(kBottomWay),
                                         fixture.block_edge_for_way(kLeftWay), 20.0, 20.0);
    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    const LotFrontage* front = result.primary_front();
    CHECK_TRUE(front != nullptr);
    if (front == nullptr) {
        return;
    }
    CHECK_EQ(fixture.way_of(*front), kLeftWay);
    CHECK_NEAR(front->length, 20.0, 1e-9);
}

/**
 * The stage after road class: two frontages of equal length on streets of equal
 * class are settled by the lower `EdgeId`.
 *
 * primary_front_breaks_a_length_tie_on_road_class() above cannot reach this stage,
 * because it makes one street Primary and the class criterion settles the tie
 * there. Here both streets are Residential, so there is nothing left but the
 * identifiers.
 *
 * The lot outline starts at its north-east corner rather than at the origin, which
 * is what gives the test teeth: the frontages then come back in the order
 * (left street, bottom street), so the ring order and the `EdgeId` order DISAGREE.
 * Started at the origin the two orders agree, both criteria name the same
 * frontage, and deleting the `EdgeId` comparison outright would not move the
 * answer. A subdivider is free to start an outline anywhere, so this is a real
 * outline and not a contrivance.
 */
TEST(LotEdges, primary_front_breaks_a_class_tie_on_the_lower_edge_id) {
    const BlockFixture fixture = make_rect_block();   // all four streets Residential
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    // The same 20 m square corner lot as above, wound from (20, 0) instead of the
    // origin: the left street is ring segment 2 and the bottom street segment 3.
    LotOutline lot;
    lot.ring = {{20.0, 0.0}, {20.0, 20.0}, {0.0, 20.0}, {0.0, 0.0}};
    lot.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, fixture.block_edge_for_way(kLeftWay),
                        fixture.block_edge_for_way(kBottomWay)};

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_EQ(result.frontages.size(), size_t{2});
    CHECK_EQ(role_string(result), std::string("Side,Back,Front,Front"));
    if (result.frontages.size() != 2) {
        return;
    }

    // The left street comes FIRST round the ring, and loses anyway: way 10 was
    // added to the graph before way 13, so the bottom street has the lower EdgeId.
    CHECK_EQ(fixture.way_of(result.frontages[0]), kLeftWay);
    CHECK_EQ(fixture.way_of(result.frontages[1]), kBottomWay);
    CHECK((result.frontages[1].street) < (result.frontages[0].street));
    CHECK_NEAR(result.frontages[0].length, result.frontages[1].length, 1e-12);

    const LotFrontage* front = result.primary_front();
    CHECK_TRUE(front != nullptr);
    if (front == nullptr) {
        return;
    }
    CHECK_EQ(fixture.way_of(*front), kBottomWay);
    CHECK_NEAR(result.front_normal().x, 0.0, 1e-12);
    CHECK_NEAR(result.front_normal().y, -1.0, 1e-12);

    // Determinism is the whole reason the stage exists, so state it: the same
    // input classified again gives the same primary, not merely a valid one.
    const LotEdgeClassification again =
        classify_lot_edges(lot, fixture.block(), fixture.graph);
    CHECK_EQ(again.primary_frontage, result.primary_frontage);
}

/**
 * `primary_tie_epsilon` is a statement that ten centimetres of extra frontage does
 * not decide where the front door goes, and this is the only test that can see it.
 *
 * The lot has 20.0 m on the residential bottom street and 19.9 m on the Primary
 * left street. Inside the default 0.25 m band the two lengths are a TIE, so road
 * class settles it and the high street wins. With the band closed to zero the
 * 10 cm settles it instead and the residential street wins. Nothing but the
 * epsilon differs between the two halves, so an implementation that hard-coded the
 * band -- at any value -- fails one of them.
 */
TEST(LotEdges, primary_tie_epsilon_decides_whether_ten_centimetres_is_a_tie) {
    RectOptions options;
    options.left = RoadType::Primary;   // the bottom stays Residential
    const BlockFixture fixture = make_rect_block(options);
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const LotOutline lot = lot_on_corner(fixture.block_edge_for_way(kBottomWay),
                                         fixture.block_edge_for_way(kLeftWay), 20.0, 19.9);

    LotEdgeConfig banded;   // primary_tie_epsilon is 0.25
    const LotEdgeClassification by_class =
        classify_lot_edges(lot, fixture.block(), fixture.graph, banded);

    LotEdgeConfig exact;
    exact.primary_tie_epsilon = 0.0;
    const LotEdgeClassification by_length =
        classify_lot_edges(lot, fixture.block(), fixture.graph, exact);

    const LotFrontage* class_front = by_class.primary_front();
    const LotFrontage* length_front = by_length.primary_front();
    CHECK_TRUE(class_front != nullptr);
    CHECK_TRUE(length_front != nullptr);
    if (class_front == nullptr || length_front == nullptr) {
        return;
    }

    CHECK_EQ(fixture.way_of(*class_front), kLeftWay);     // the Primary road
    CHECK_EQ(fixture.way_of(*length_front), kBottomWay);  // the extra 10 cm

    // The role array is identical either way -- both edges are fronts -- so the
    // normal is what proves the primary actually moved.
    CHECK_NEAR(by_class.front_normal().x, -1.0, 1e-12);
    CHECK_NEAR(by_length.front_normal().y, -1.0, 1e-12);
}

/**
 * A cul-de-sac spur bounds its block TWICE, once up each side, so a lot at the head
 * of the spur fronts ONE street from two sides. This is the shape that separates
 * the two grouping keys the implementation could have used.
 *
 * Grouped by `Block::edges` index -- which is one half-edge, and is what the
 * implementation does -- the lot has TWO frontages, and each names the side of the
 * centreline it is on: the west arm is Left of the spur's own direction and the
 * east arm is Right. Grouped by `EdgeId` instead, the two merge into one frontage
 * of 44 m that claims to be on a single side of a centreline it straddles, and
 * whose length-weighted normal cancels to nothing.
 *
 * It is also the only shape in which the last stage of the primary tie-break is
 * reachable: the two frontages agree on length, on road class and on `EdgeId`, so
 * the primary is decided by which one the lot outline reaches first.
 *
 * What this test cannot do, and no test can: catch that stage being NEUTERED. The
 * frontages arrive in ring order already, so a tie-break that stops looking at the
 * ring index leaves the first one winning, which is the same answer. Reversing it
 * is caught, by the second half below. See the comment on pick_primary().
 */
TEST(LotEdges, a_lot_at_the_head_of_a_spur_fronts_one_street_from_both_sides) {
    const BlockFixture fixture = make_spur_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t west = fixture.block_edge_for_way_side(kSpurWay, true);
    const uint32_t east = fixture.block_edge_for_way_side(kSpurWay, false);
    CHECK((west) != (east));

    // A hammerhead lot wrapping the head of the spur: a U with the slot open to
    // the south, the spur running up the middle of the slot.
    LotOutline lot;
    lot.ring = {{40.0, 3.0},  {48.0, 3.0},  {48.0, 25.0}, {52.0, 25.0},
                {52.0, 3.0},  {60.0, 3.0},  {60.0, 30.0}, {40.0, 30.0}};
    lot.edge_sources = {kNoBoundaryEdge, west,            kNoBoundaryEdge, east,
                        kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_FALSE(result.landlocked);
    CHECK_EQ(result.frontages.size(), size_t{2});
    CHECK_EQ(role_string(result),
             std::string("Side,Front,Side,Front,Side,Interior,Side,Back"));
    if (result.frontages.size() != 2) {
        return;
    }

    // One street, two half-edges, two sides of one centreline.
    CHECK_EQ(fixture.way_of(result.frontages[0]), kSpurWay);
    CHECK_EQ(fixture.way_of(result.frontages[1]), kSpurWay);
    CHECK_EQ(result.frontages[0].street, result.frontages[1].street);
    CHECK_EQ(std::string(street_side_name(result.frontages[0].side)), std::string("Left"));
    CHECK_EQ(std::string(street_side_name(result.frontages[1].side)), std::string("Right"));
    CHECK_NEAR(result.frontages[0].length, 22.0, 1e-9);
    CHECK_NEAR(result.frontages[1].length, 22.0, 1e-9);

    // Equal length, equal class, equal EdgeId: the earlier ring edge takes the
    // front door, and the two arms face each other so the primary's normal says
    // which arm won. The west arm faces east, into the slot the spur runs up.
    CHECK_EQ(result.primary_frontage, uint32_t{0});
    CHECK_NEAR(result.front_normal().x, 1.0, 1e-12);
    CHECK_NEAR(result.front_normal().y, 0.0, 1e-12);
    CHECK_NEAR(result.front_midpoint().x, 48.0, 1e-9);
    CHECK_NEAR(result.front_midpoint().y, 14.0, 1e-9);

    // The SAME lot, wound from a different start vertex so the outline reaches the
    // east arm first. A subdivider is free to start an outline anywhere, and the
    // only thing separating these two arms is which one it reached first -- so the
    // front door moves with it, and moves the whole way, normal included. Without
    // this half the ring-index key could be read backwards and the first half would
    // still pass.
    LotOutline from_east;
    from_east.ring = {{52.0, 25.0}, {52.0, 3.0},  {60.0, 3.0},  {60.0, 30.0},
                      {40.0, 30.0}, {40.0, 3.0},  {48.0, 3.0},  {48.0, 25.0}};
    from_east.edge_sources = {east,            kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge,
                              kNoBoundaryEdge, kNoBoundaryEdge, west,            kNoBoundaryEdge};

    const LotEdgeClassification rotated =
        classify_lot_edges(from_east, fixture.block(), fixture.graph);

    CHECK_EQ(rotated.frontages.size(), size_t{2});
    CHECK_EQ(role_string(rotated),
             std::string("Front,Side,Back,Side,Interior,Side,Front,Side"));
    CHECK_EQ(rotated.primary_frontage, uint32_t{0});
    if (rotated.frontages.size() == 2) {
        CHECK_EQ(std::string(street_side_name(rotated.frontages[0].side)), std::string("Right"));
    }
    CHECK_NEAR(rotated.front_normal().x, -1.0, 1e-12);   // the east arm faces west
    CHECK_NEAR(rotated.front_normal().y, 0.0, 1e-12);
}

/**
 * The retail case: a corner shop puts its window on the high street even when the
 * side-street frontage is longer. Under the default rule the longer side wins, and
 * switching the rule is the ONLY difference between the two halves here -- the lot
 * and the block are identical, so a rule that is not being read fails one half.
 */
TEST(LotEdges, highest_road_class_rule_beats_a_longer_frontage) {
    RectOptions options;
    options.left = RoadType::Primary;
    const BlockFixture fixture = make_rect_block(options);
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    // 30 m on the residential bottom, 12 m on the Primary left.
    const LotOutline lot = lot_on_corner(fixture.block_edge_for_way(kBottomWay),
                                         fixture.block_edge_for_way(kLeftWay), 30.0, 12.0);

    const LotEdgeClassification by_length =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    LotEdgeConfig by_class_config;
    by_class_config.primary_rule = PrimaryFrontRule::HighestRoadClass;
    const LotEdgeClassification by_class =
        classify_lot_edges(lot, fixture.block(), fixture.graph, by_class_config);

    const LotFrontage* length_front = by_length.primary_front();
    const LotFrontage* class_front = by_class.primary_front();
    CHECK_TRUE(length_front != nullptr);
    CHECK_TRUE(class_front != nullptr);
    if (length_front == nullptr || class_front == nullptr) {
        return;
    }

    CHECK_EQ(fixture.way_of(*length_front), kBottomWay);
    CHECK_EQ(fixture.way_of(*class_front), kLeftWay);

    // The role array is identical either way -- both edges are fronts -- so the
    // normal is what proves the primary actually moved.
    CHECK_NEAR(by_length.front_normal().y, -1.0, 1e-12);
    CHECK_NEAR(by_class.front_normal().x, -1.0, 1e-12);
}

// ============================================================================
// Landlocked, And What Must Not Be Guessed
// ============================================================================

/**
 * A lot in the middle of a block touches no street. Every edge is INTERIOR, the
 * lot is flagged, and nothing is called a back -- there is no front for a back to
 * be opposite to.
 */
TEST(LotEdges, landlocked_lot_is_reported_not_classified_as_back) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const LotEdgeClassification result =
        classify_lot_edges(lot_interior(), fixture.block(), fixture.graph);

    CHECK_TRUE(result.landlocked);
    CHECK_FALSE(result.has_front());
    CHECK_TRUE(result.primary_front() == nullptr);
    CHECK_EQ(role_string(result), std::string("Interior,Interior,Interior,Interior"));
    CHECK_EQ(result.frontages.size(), size_t{0});

    // Zero rather than a plausible direction, so a caller that forgot to check
    // has_front() gets something obviously unusable.
    CHECK_NEAR(result.front_normal().x, 0.0, 1e-12);
    CHECK_NEAR(result.front_normal().y, 0.0, 1e-12);
}

/**
 * The proximity trap, stated as directly as it can be.
 *
 * This lot's rear edge sits at y=58, TWO METRES from the top street's centreline
 * at y=60 -- nearer to it than the lot's own front door is to the bottom street it
 * actually fronts, since the front edge is 58 m away along the lot. Any classifier
 * that measured a distance to a road centreline calls the rear edge a front, gives
 * the lot two fronts, and may well make the top street primary.
 *
 * Identity says the rear edge was inherited from nothing, so it is a back.
 */
TEST(LotEdges, a_street_two_metres_behind_the_lot_is_not_frontage) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot;
    lot.ring = {{20.0, 0.0}, {40.0, 0.0}, {40.0, 58.0}, {20.0, 58.0}};
    lot.edge_sources = {fixture.block_edge_for_way(kBottomWay), kNoBoundaryEdge, kNoBoundaryEdge,
                        kNoBoundaryEdge};

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_EQ(role_string(result), std::string("Front,Side,Back,Side"));
    CHECK_EQ(result.frontages.size(), size_t{1});

    const LotFrontage* front = result.primary_front();
    CHECK_TRUE(front != nullptr);
    if (front != nullptr) {
        CHECK_EQ(fixture.way_of(*front), kBottomWay);
    }
}

/**
 * A footway bounding the block is a boundary but not a street, so it is a SIDE and
 * a lot with nothing else is landlocked. Turning `paths_are_streets` on -- the
 * pedestrianised-centre case, and the mirror of BlockConfig::include_paths --
 * makes the same edge a front.
 *
 * Both halves classify the SAME lot against the SAME block, so the config is the
 * only variable.
 */
TEST(LotEdges, footway_boundary_is_a_side_until_paths_are_streets) {
    RectOptions options;
    options.top = RoadType::Footway;
    options.include_paths = true;   // otherwise the footway would not bound the block
    const BlockFixture fixture = make_rect_block(options);
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot;
    lot.ring = {{20.0, 40.0}, {40.0, 40.0}, {40.0, 60.0}, {20.0, 60.0}};
    lot.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, fixture.block_edge_for_way(kTopWay),
                        kNoBoundaryEdge};

    const LotEdgeClassification as_path =
        classify_lot_edges(lot, fixture.block(), fixture.graph);
    CHECK_TRUE(as_path.landlocked);
    CHECK_EQ(role_string(as_path), std::string("Interior,Interior,Side,Interior"));

    LotEdgeConfig config;
    config.paths_are_streets = true;
    const LotEdgeClassification as_street =
        classify_lot_edges(lot, fixture.block(), fixture.graph, config);
    CHECK_FALSE(as_street.landlocked);
    CHECK_EQ(role_string(as_street), std::string("Back,Side,Front,Side"));

    const LotFrontage* front = as_street.primary_front();
    CHECK_TRUE(front != nullptr);
    if (front != nullptr) {
        CHECK_EQ(fixture.way_of(*front), kTopWay);
        CHECK_EQ(std::string(street_side_name(front->side)), std::string("Right"));
    }
}

/**
 * Forty centimetres of contact with the boundary is a subdivision artefact, not a
 * frontage. It is demoted to SIDE and counted, and because it was this lot's only
 * contact the lot is reported landlocked rather than keeping a token front.
 *
 * Lowering the floor below the sliver restores the front, which is what proves the
 * configuration is read rather than the length hard-coded.
 */
TEST(LotEdges, sliver_frontage_is_demoted_and_the_lot_reported_landlocked) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot;
    lot.ring = {{20.0, 0.0}, {20.4, 0.0}, {20.4, 20.0}, {20.0, 20.0}};
    lot.edge_sources = {fixture.block_edge_for_way(kBottomWay), kNoBoundaryEdge, kNoBoundaryEdge,
                        kNoBoundaryEdge};

    const LotEdgeClassification demoted =
        classify_lot_edges(lot, fixture.block(), fixture.graph);
    CHECK_TRUE(demoted.landlocked);
    CHECK_EQ(demoted.demoted_frontages, uint32_t{1});
    CHECK_EQ(role_string(demoted), std::string("Side,Interior,Interior,Interior"));

    LotEdgeConfig config;
    config.min_frontage_length = 0.1;
    const LotEdgeClassification kept =
        classify_lot_edges(lot, fixture.block(), fixture.graph, config);
    CHECK_FALSE(kept.landlocked);
    CHECK_EQ(kept.demoted_frontages, uint32_t{0});
    CHECK_EQ(role_string(kept), std::string("Front,Side,Back,Side"));
}

/**
 * The floor is on the longest contiguous RUN of a frontage, not on the frontage
 * total, and this is the fixture that can tell the two apart.
 *
 * The lot touches the bottom street in two places, 60 cm each, with a notch
 * between them. Both are the rounding artefact the floor exists to suppress -- no
 * single stretch of kerb is long enough to put a door on -- but they are on one
 * half-edge, so they are ONE frontage whose total is 1.2 m. An implementation that
 * measured that total would clear the 1 m default and hand the lot a confident
 * FRONT on each sliver, with a midpoint in the gap between them.
 *
 * The second half lowers the floor under the RUN rather than under the total, so
 * the same lot keeps both slivers: it pins that the floor is read from the config
 * and that the demotion is not simply unconditional.
 */
TEST(LotEdges, two_sub_floor_runs_on_one_street_do_not_add_up_to_a_front) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);

    // Two 0.6 m contacts with the bottom street, separated by a 9.4 m notch.
    LotOutline lot;
    lot.ring = {{10.0, 0.0},  {10.6, 0.0},  {10.6, 2.0},  {20.0, 2.0},
                {20.0, 0.0},  {20.6, 0.0},  {20.6, 20.0}, {10.0, 20.0}};
    lot.edge_sources = {bottom,          kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge,
                        bottom,          kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    // Default floor of 1 m. Runs are 0.6 and 0.6; the total is 1.2.
    const LotEdgeClassification demoted =
        classify_lot_edges(lot, fixture.block(), fixture.graph);
    CHECK_TRUE(demoted.landlocked);
    CHECK_EQ(demoted.demoted_frontages, uint32_t{1});
    CHECK_EQ(demoted.frontages.size(), size_t{0});
    CHECK_EQ(demoted.count(LotEdgeRole::Front), size_t{0});
    CHECK_EQ(role_string(demoted),
             std::string("Side,Interior,Interior,Interior,Side,Interior,Interior,Interior"));

    // Floor under the run: both slivers survive, as one frontage of 1.2 m.
    LotEdgeConfig config;
    config.min_frontage_length = 0.5;
    const LotEdgeClassification kept =
        classify_lot_edges(lot, fixture.block(), fixture.graph, config);
    CHECK_FALSE(kept.landlocked);
    CHECK_EQ(kept.demoted_frontages, uint32_t{0});
    CHECK_EQ(kept.frontages.size(), size_t{1});
    CHECK_EQ(role_string(kept),
             std::string("Front,Side,Interior,Side,Front,Side,Back,Side"));
    if (!kept.frontages.empty()) {
        CHECK_NEAR(kept.frontages[0].length, 1.2, 1e-9);
    }
}

/**
 * The floor is a MINIMUM, not a threshold to exceed: a frontage exactly as long as
 * `min_frontage_length` is kept.
 *
 * The two answers differ by one edge role and by one counter, and nothing else in
 * the suite pins which side of `<` versus `<=` the comparison sits on. The metre
 * here is exact in binary, so the assertion is not resting on rounding.
 */
TEST(LotEdges, a_frontage_exactly_at_the_floor_is_kept) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot;
    lot.ring = {{20.0, 0.0}, {21.0, 0.0}, {21.0, 20.0}, {20.0, 20.0}};
    lot.edge_sources = {fixture.block_edge_for_way(kBottomWay), kNoBoundaryEdge, kNoBoundaryEdge,
                        kNoBoundaryEdge};

    LotEdgeConfig config;   // min_frontage_length is 1.0, the frontage is 1.0
    const LotEdgeClassification at_floor =
        classify_lot_edges(lot, fixture.block(), fixture.graph, config);
    CHECK_FALSE(at_floor.landlocked);
    CHECK_EQ(at_floor.demoted_frontages, uint32_t{0});
    CHECK_EQ(role_string(at_floor), std::string("Front,Side,Back,Side"));

    // A hair above the frontage length and the same lot is demoted, so the
    // assertion above is a boundary and not a floor that never fires.
    config.min_frontage_length = 1.000001;
    const LotEdgeClassification above_floor =
        classify_lot_edges(lot, fixture.block(), fixture.graph, config);
    CHECK_TRUE(above_floor.landlocked);
    CHECK_EQ(above_floor.demoted_frontages, uint32_t{1});
}

// ============================================================================
// Orientation
// ============================================================================

/**
 * A clockwise ring must be corrected, not trusted and not rejected.
 *
 * The role ARRAY cannot catch a missing correction: reversing the ring flips every
 * normal together, so every dot product against the front normal keeps its sign
 * and the roles come out the same either way. The front normal is the only thing
 * that changes, and it is what a facade and a setback both read -- so it is
 * asserted here, and the roles are asserted alongside only to show they did not
 * shuffle.
 */
TEST(LotEdges, clockwise_ring_is_corrected_rather_than_mirrored) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);

    // lot_on_bottom() reversed: the same four corners, wound the other way, with
    // the street edge now at index 2.
    LotOutline reversed;
    reversed.ring = {{20.0, 30.0}, {40.0, 30.0}, {40.0, 0.0}, {20.0, 0.0}};
    reversed.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, bottom, kNoBoundaryEdge};

    const LotEdgeClassification result =
        classify_lot_edges(reversed, fixture.block(), fixture.graph);

    CHECK_TRUE(result.orientation_corrected);
    CHECK_FALSE(result.malformed);
    CHECK_EQ(role_string(result), std::string("Back,Side,Front,Side"));

    // The load-bearing assertion. Without the correction this is (0, +1) and every
    // shopfront in the extract faces the back garden.
    CHECK_NEAR(result.front_normal().x, 0.0, 1e-12);
    CHECK_NEAR(result.front_normal().y, -1.0, 1e-12);
    CHECK_NEAR(result.front_midpoint().x, 30.0, 1e-9);
    CHECK_NEAR(result.front_midpoint().y, 0.0, 1e-9);
}

// ============================================================================
// Curves, Notches And The Fourth Role
// ============================================================================

/**
 * A curved street is one graph edge carrying several segments, and a lot along it
 * inherits several edges from ONE half-edge. They are one frontage, not several,
 * and its normal is the LENGTH-WEIGHTED average rather than the first segment's
 * and rather than the plain average of the two.
 *
 * The bend is off centre -- see make_bent_bottom_block() -- so the two segments are
 * 14.14 m and 90.55 m. Their normals are 45 degrees and 6.3 degrees off -Y on
 * opposite sides, and only the LENGTH-WEIGHTED average of them is exactly -Y:
 *
 *   - weighting by length gives (0, -1), asserted below;
 *   - a plain average gives (0.331, -0.944), 19 degrees off it;
 *   - `edges.front()`'s own normal gives (0.707, -0.707), 45 degrees off it.
 *
 * The first assertion therefore separates all three, which is the point of moving
 * the bend: at the midpoint the fixture was mirror-symmetric and the first two
 * answers were bit-identical.
 *
 * The frontage midpoint moves with it. It is no longer the apex of the bend -- a
 * vertex, which an implementation that returned the middle VERTEX would also hit --
 * but a point 38.2 m along the second segment.
 */
TEST(LotEdges, curved_street_is_one_frontage_with_one_length_weighted_normal) {
    const BlockFixture fixture = make_bent_bottom_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot;
    lot.ring = {{0.0, 0.0}, {10.0, 10.0}, {100.0, 0.0}, {100.0, 20.0}, {0.0, 20.0}};
    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);
    lot.edge_sources = {bottom, bottom, kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_EQ(result.frontages.size(), size_t{1});
    CHECK_EQ(role_string(result), std::string("Front,Front,Side,Back,Side"));
    if (result.frontages.empty()) {
        return;
    }

    const double first = std::sqrt(200.0);    // (0,0) -> (10,10)
    const double second = std::sqrt(8200.0);  // (10,10) -> (100,0)

    CHECK_EQ(result.frontages[0].edges.size(), size_t{2});
    CHECK_NEAR(result.frontages[0].length, first + second, 1e-9);

    // The load-bearing assertion: unreachable without the length weighting.
    CHECK_NEAR(result.front_normal().x, 0.0, 1e-12);
    CHECK_NEAR(result.front_normal().y, -1.0, 1e-12);

    // Half the frontage by arc length lands inside the long second segment.
    const double along_second = 0.5 * (first + second) - first;
    const double t = along_second / second;
    CHECK_NEAR(result.front_midpoint().x, 10.0 + 90.0 * t, 1e-9);
    CHECK_NEAR(result.front_midpoint().y, 10.0 - 10.0 * t, 1e-9);
}

/**
 * All four roles from one lot, which is the only way to show that INTERIOR is a
 * real classification and not just what a landlocked lot gets.
 *
 * The lot has a notch bitten out of its street edge. The far side of the notch
 * faces the street -- same normal as the frontage -- but touches nothing, which is
 * the rear lot of a flag-shaped subdivision looking at the back of the lot in
 * front. It is INTERIOR. The two walls of the notch are perpendicular to the
 * frontage and are SIDEs, and the rear edge is the BACK.
 */
TEST(LotEdges, notch_lot_exercises_all_four_roles) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);

    LotOutline lot;
    lot.ring = {{10.0, 0.0},  {30.0, 0.0},  {30.0, 5.0},  {50.0, 5.0},
                {50.0, 0.0},  {80.0, 0.0},  {80.0, 20.0}, {10.0, 20.0}};
    lot.edge_sources = {bottom,       kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge,
                        bottom,       kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_EQ(role_string(result),
             std::string("Front,Side,Interior,Side,Front,Side,Back,Side"));
    CHECK_EQ(result.count(LotEdgeRole::Front), size_t{2});
    CHECK_EQ(result.count(LotEdgeRole::Side), size_t{4});
    CHECK_EQ(result.count(LotEdgeRole::Back), size_t{1});
    CHECK_EQ(result.count(LotEdgeRole::Interior), size_t{1});

    // Both street edges came from the same half-edge, so they are ONE frontage and
    // the lot is not a corner lot.
    CHECK_EQ(result.frontages.size(), size_t{1});
}

/**
 * The notch splits the frontage into a 20 m run and a 30 m run, so the midpoint of
 * the frontage taken as a whole falls at 25 m of 50 -- five metres into the second
 * run, at x=55. The midpoint of the LONGEST run is at x=65.
 *
 * The two answers differ, which is the point: a lot whose street edge is broken has
 * no meaningful "middle" across the break, and an address marker placed there sits
 * inside the notch on a deeper fixture.
 */
TEST(LotEdges, front_midpoint_sits_on_the_longest_contiguous_run) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);

    LotOutline lot;
    lot.ring = {{10.0, 0.0},  {30.0, 0.0},  {30.0, 5.0},  {50.0, 5.0},
                {50.0, 0.0},  {80.0, 0.0},  {80.0, 20.0}, {10.0, 20.0}};
    lot.edge_sources = {bottom,       kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge,
                        bottom,       kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_NEAR(result.front_midpoint().x, 65.0, 1e-9);
    CHECK_NEAR(result.front_midpoint().y, 0.0, 1e-9);
    if (!result.frontages.empty()) {
        CHECK_NEAR(result.frontages[0].length, 50.0, 1e-9);
    }
}

/**
 * The BACK cone is configuration, not a constant. This lot's rear edge runs at 45
 * degrees, so it is a back inside the default 60 degree cone and a side inside a
 * 30 degree one. Nothing but the config differs between the two halves.
 */
TEST(LotEdges, back_cone_config_moves_a_slanted_rear_to_side) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot;
    lot.ring = {{20.0, 0.0}, {40.0, 0.0}, {40.0, 30.0}, {20.0, 50.0}};
    lot.edge_sources = {fixture.block_edge_for_way(kBottomWay), kNoBoundaryEdge, kNoBoundaryEdge,
                        kNoBoundaryEdge};

    const LotEdgeClassification wide =
        classify_lot_edges(lot, fixture.block(), fixture.graph);
    CHECK_EQ(role_string(wide), std::string("Front,Side,Back,Side"));

    LotEdgeConfig narrow_config;
    narrow_config.back_cone_degrees = 30.0;
    const LotEdgeClassification narrow =
        classify_lot_edges(lot, fixture.block(), fixture.graph, narrow_config);
    CHECK_EQ(role_string(narrow), std::string("Front,Side,Side,Side"));
    CHECK_EQ(narrow.count(LotEdgeRole::Back), size_t{0});
}

/**
 * The FRONT cone is configuration too, and it is the one an implementation is
 * most likely to hard-code because its default happens to partition the circle
 * neatly against the back cone.
 *
 * The notch lot's side edges are exactly perpendicular to its frontage, so they
 * sit on the boundary between the side band and the front cone. Widening the front
 * cone past 90 degrees pulls all four of them into INTERIOR while leaving the rear
 * edge a BACK, which is a different answer for every edge except the fronts and the
 * back.
 */
TEST(LotEdges, front_cone_config_moves_the_perpendicular_sides_to_interior) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);

    LotOutline lot;
    lot.ring = {{10.0, 0.0},  {30.0, 0.0},  {30.0, 5.0},  {50.0, 5.0},
                {50.0, 0.0},  {80.0, 0.0},  {80.0, 20.0}, {10.0, 20.0}};
    lot.edge_sources = {bottom,          kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge,
                        bottom,          kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    LotEdgeConfig config;
    config.front_cone_degrees = 100.0;
    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph, config);

    CHECK_EQ(role_string(result),
             std::string("Front,Interior,Interior,Interior,Front,Interior,Back,Interior"));
    CHECK_EQ(result.count(LotEdgeRole::Side), size_t{0});
    CHECK_EQ(result.count(LotEdgeRole::Back), size_t{1});
}

/**
 * The two cone tests above never make the cones OVERLAP, so between them they
 * leave the documented precedence free: BACK is tested first, and a swap of the
 * two branches changes nothing while the three bands still partition the circle.
 *
 * Widening BOTH cones to 120 degrees makes every direction satisfy both tests at
 * once. The plain mid-block lot then has its two side edges and its rear edge all
 * inside both cones, and BACK-first makes all three BACK. Testing INTERIOR first
 * would make the two sides INTERIOR and leave only the rear a BACK.
 *
 * The first half is the same lot under the default cones, so the pair also shows
 * the overlap is what moved the roles rather than the widening alone.
 */
TEST(LotEdges, overlapping_cones_are_resolved_back_first) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const LotOutline lot = lot_on_bottom(fixture.block_edge_for_way(kBottomWay));

    const LotEdgeClassification apart =
        classify_lot_edges(lot, fixture.block(), fixture.graph);
    CHECK_EQ(role_string(apart), std::string("Front,Side,Back,Side"));

    LotEdgeConfig overlapping;
    overlapping.back_cone_degrees = 120.0;
    overlapping.front_cone_degrees = 120.0;
    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph, overlapping);

    CHECK_EQ(role_string(result), std::string("Front,Back,Back,Back"));
    CHECK_EQ(result.count(LotEdgeRole::Interior), size_t{0});
    CHECK_EQ(result.count(LotEdgeRole::Side), size_t{0});
}

/**
 * A cone half-angle past 180 degrees is a configuration mistake, and it is clamped
 * so that it means "accept everything" rather than wrapping back round the circle
 * and rejecting the directions it is furthest from.
 *
 * 240 degrees unclamped has a cosine of -0.5, so the BACK test would admit every
 * direction EXCEPT the ones pointing most directly at the street -- the one edge
 * here that faces the frontage, which would come back INTERIOR. Clamped to 180 the
 * cosine is -1 and every interior edge is a BACK, which is what a half-angle of
 * 240 degrees was asking for however badly it asked.
 *
 * The notch's far side is tilted a few degrees off parallel on purpose: dead
 * parallel it would face the frontage at exactly 1.0 and the assertion would rest
 * on an exact floating-point tie.
 */
TEST(LotEdges, a_cone_wider_than_a_half_circle_is_clamped_not_wrapped) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);

    LotOutline lot;
    lot.ring = {{10.0, 0.0},  {30.0, 0.0},  {30.0, 5.0},  {50.0, 5.5},
                {50.0, 0.0},  {80.0, 0.0},  {80.0, 20.0}, {10.0, 20.0}};
    lot.edge_sources = {bottom,          kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge,
                        bottom,          kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    const LotEdgeClassification sane =
        classify_lot_edges(lot, fixture.block(), fixture.graph);
    CHECK_EQ(role_string(sane), std::string("Front,Side,Interior,Side,Front,Side,Back,Side"));

    LotEdgeConfig absurd;
    absurd.back_cone_degrees = 240.0;
    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph, absurd);

    CHECK_EQ(role_string(result), std::string("Front,Back,Back,Back,Front,Back,Back,Back"));
    CHECK_EQ(result.count(LotEdgeRole::Interior), size_t{0});
}

/**
 * Two runs of the SAME length is the case longest_run()'s tie-break exists for, and
 * front_midpoint_sits_on_the_longest_contiguous_run() cannot reach it -- its runs
 * are 20 m and 30 m, so the comparison never ties.
 *
 * Here the notch is centred, both runs are 20 m, and the address marker goes on the
 * first of them. Either answer is a real point on the kerb, so nothing but this
 * assertion stops the two swapping when an unrelated change reorders the scan, and
 * an address that moves to the far end of the lot between runs is exactly the kind
 * of instability the whole file is written to avoid.
 */
TEST(LotEdges, front_midpoint_breaks_a_run_tie_on_the_lower_start_index) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);

    LotOutline lot;
    lot.ring = {{10.0, 0.0},  {30.0, 0.0},  {30.0, 5.0},  {50.0, 5.0},
                {50.0, 0.0},  {70.0, 0.0},  {70.0, 20.0}, {10.0, 20.0}};
    lot.edge_sources = {bottom,          kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge,
                        bottom,          kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_EQ(result.frontages.size(), size_t{1});
    if (result.frontages.empty()) {
        return;
    }
    CHECK_EQ(result.frontages[0].edges.size(), size_t{2});
    CHECK_NEAR(result.frontages[0].length, 40.0, 1e-9);   // 20 m + 20 m, an exact tie

    CHECK_NEAR(result.front_midpoint().x, 20.0, 1e-9);    // middle of the FIRST run
    CHECK_NEAR(result.front_midpoint().y, 0.0, 1e-9);
}

/**
 * A block ringed by ONE closed way -- an estate loop road, a roundabout -- hands
 * every ring segment to the same half-edge. A lot filling that block is therefore
 * one frontage with no gap in it and no net direction, and it is the only shape
 * that reaches either of the two branches that handle those.
 *
 * The frontage closes on itself, so its length-weighted normal sums to exactly
 * zero. Without the fallback the lot comes back with a front whose normal is
 * (0, 0), which aims every facade nowhere; with it the normal is the longest
 * single segment's, and all four are 40 m, so the first one wins.
 *
 * The run of frontage covers the whole ring, so it has no first segment for the
 * cyclic scan to find. Without the all-set branch the scan reports no run at all,
 * and the frontage is then demoted as a sliver of zero length -- the lot that
 * fronts the street on all four sides comes back landlocked.
 */
TEST(LotEdges, a_lot_filling_a_loop_road_block_is_frontage_all_the_way_round) {
    const BlockFixture fixture = make_loop_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t loop = fixture.block_edge_for_way(kLoopWay);

    LotOutline lot;
    lot.ring = {{0.0, 0.0}, {40.0, 0.0}, {40.0, 40.0}, {0.0, 40.0}};
    lot.edge_sources = {loop, loop, loop, loop};

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_FALSE(result.malformed);
    CHECK_FALSE(result.landlocked);
    CHECK_EQ(result.demoted_frontages, uint32_t{0});
    CHECK_EQ(role_string(result), std::string("Front,Front,Front,Front"));
    CHECK_EQ(result.frontages.size(), size_t{1});
    if (result.frontages.empty()) {
        return;
    }
    CHECK_EQ(result.frontages[0].edges.size(), size_t{4});
    CHECK_NEAR(result.frontages[0].length, 160.0, 1e-9);

    // The cancelled-normal fallback: the longest segment, and on a tie the first.
    CHECK_NEAR(result.front_normal().x, 0.0, 1e-12);
    CHECK_NEAR(result.front_normal().y, -1.0, 1e-12);

    // Half of 160 m walked from ring[0] lands on the far corner of the second side.
    CHECK_NEAR(result.front_midpoint().x, 40.0, 1e-9);
    CHECK_NEAR(result.front_midpoint().y, 40.0, 1e-9);
}

// ============================================================================
// The Optional Source Inference
// ============================================================================

/**
 * A lot that lost its provenance is landlocked by default, and the inference
 * recovers it only when asked. The same lot, the same block, one config flag.
 */
TEST(LotEdges, source_inference_is_off_by_default_and_recovers_a_boundary_edge) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot = lot_on_bottom(kNoBoundaryEdge);   // provenance thrown away

    const LotEdgeClassification without =
        classify_lot_edges(lot, fixture.block(), fixture.graph);
    CHECK_TRUE(without.landlocked);
    CHECK_EQ(without.inferred_sources, uint32_t{0});

    LotEdgeConfig config;
    config.infer_sources_from_block_ring = true;
    const LotEdgeClassification with =
        classify_lot_edges(lot, fixture.block(), fixture.graph, config);
    CHECK_FALSE(with.landlocked);
    CHECK_EQ(with.inferred_sources, uint32_t{1});
    CHECK_EQ(role_string(with), std::string("Front,Side,Back,Side"));

    const LotFrontage* front = with.primary_front();
    CHECK_TRUE(front != nullptr);
    if (front != nullptr) {
        CHECK_EQ(fixture.way_of(*front), kBottomWay);
    }
}

/**
 * The inference adopts a boundary only when the lot edge runs the SAME way round
 * it. A reversed ring lies on exactly the same geometry and must not be adopted:
 * an edge running against the boundary belongs to whatever is on the other side.
 *
 * Without the direction test this lot comes back with a front, so the test fails
 * the moment that check is dropped.
 */
TEST(LotEdges, source_inference_refuses_an_edge_running_against_the_boundary) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline reversed;
    reversed.ring = {{20.0, 30.0}, {40.0, 30.0}, {40.0, 0.0}, {20.0, 0.0}};
    reversed.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    LotEdgeConfig config;
    config.infer_sources_from_block_ring = true;
    const LotEdgeClassification result =
        classify_lot_edges(reversed, fixture.block(), fixture.graph, config);

    CHECK_EQ(result.inferred_sources, uint32_t{0});
    CHECK_TRUE(result.landlocked);
}

/**
 * The inference is a containment test against the block ring, not a search for a
 * nearby road, so it cannot manufacture frontage for a lot in the middle of the
 * block however close a street runs.
 */
TEST(LotEdges, source_inference_cannot_invent_frontage_for_an_interior_lot) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    // One metre clear of the bottom street along its whole width.
    LotOutline lot;
    lot.ring = {{20.0, 1.0}, {40.0, 1.0}, {40.0, 20.0}, {20.0, 20.0}};
    lot.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    LotEdgeConfig config;
    config.infer_sources_from_block_ring = true;
    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph, config);

    CHECK_EQ(result.inferred_sources, uint32_t{0});
    CHECK_TRUE(result.landlocked);
    CHECK_EQ(role_string(result), std::string("Interior,Interior,Interior,Interior"));
}

/**
 * The containment half of the inference, which nothing else in the suite reaches.
 *
 * source_inference_cannot_invent_frontage_for_an_interior_lot() above puts its lot
 * one metre OFF the street, so the collinearity test rejects it and the span test
 * is never the deciding check. This lot lies exactly ON the bottom street's line,
 * runs the same way along it, and sits 20 m past the end of the block -- so
 * collinearity and direction both pass and containment is the only thing left to
 * refuse it.
 *
 * Without the span test the lot is handed a confident FRONT on a street it does not
 * touch, which is the proximity failure this whole file exists to prevent, arriving
 * through the one geometric routine in it.
 */
TEST(LotEdges, source_inference_refuses_a_lot_past_the_end_of_a_collinear_boundary) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    // The block's bottom street runs from x=0 to x=100 along y=0. This lot sits on
    // the same line from x=120 to x=140.
    LotOutline lot;
    lot.ring = {{120.0, 0.0}, {140.0, 0.0}, {140.0, 20.0}, {120.0, 20.0}};
    lot.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    LotEdgeConfig config;
    config.infer_sources_from_block_ring = true;
    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph, config);

    CHECK_EQ(result.inferred_sources, uint32_t{0});
    CHECK_TRUE(result.landlocked);
    CHECK_EQ(result.count(LotEdgeRole::Front), size_t{0});
    CHECK_EQ(role_string(result), std::string("Interior,Interior,Interior,Interior"));
}

/**
 * `source_snap_tolerance` is how far off the boundary a lot edge may sit and still
 * BE that boundary, and it is the only number in the inference. Nothing else in the
 * suite moves it, so a hard-coded centimetre passes the rest of the file.
 *
 * This lot is 5 cm north of the bottom street along its whole width -- the shape of
 * a subdivider that rounded its output to a coarser grid than the graph. At the
 * default centimetre it is not on the boundary and the lot is landlocked; at ten
 * centimetres it is, and the same edge is a front on the same street. The lot and
 * the block are identical between the two halves.
 */
TEST(LotEdges, source_snap_tolerance_sets_how_far_off_the_boundary_still_counts) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot;
    lot.ring = {{20.0, 0.05}, {40.0, 0.05}, {40.0, 20.0}, {20.0, 20.0}};
    lot.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    LotEdgeConfig tight;
    tight.infer_sources_from_block_ring = true;   // source_snap_tolerance stays 0.01
    const LotEdgeClassification refused =
        classify_lot_edges(lot, fixture.block(), fixture.graph, tight);
    CHECK_EQ(refused.inferred_sources, uint32_t{0});
    CHECK_TRUE(refused.landlocked);

    LotEdgeConfig loose;
    loose.infer_sources_from_block_ring = true;
    loose.source_snap_tolerance = 0.1;
    const LotEdgeClassification recovered =
        classify_lot_edges(lot, fixture.block(), fixture.graph, loose);
    CHECK_EQ(recovered.inferred_sources, uint32_t{1});
    CHECK_FALSE(recovered.landlocked);
    CHECK_EQ(role_string(recovered), std::string("Front,Side,Back,Side"));

    const LotFrontage* front = recovered.primary_front();
    CHECK_TRUE(front != nullptr);
    if (front != nullptr) {
        CHECK_EQ(fixture.way_of(*front), kBottomWay);
    }
}

// ============================================================================
// Malformed Input
// ============================================================================

/**
 * `edge_sources` one entry short of `ring` is rejected outright rather than
 * truncated. Truncating classifies most of the lot correctly and attributes the
 * rest to the neighbouring street, which is a plausible-looking answer nobody
 * would look twice at.
 */
TEST(LotEdges, mismatched_edge_sources_are_malformed) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot = lot_on_bottom(fixture.block_edge_for_way(kBottomWay));
    lot.edge_sources.pop_back();

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_TRUE(result.malformed);
    CHECK_EQ(result.roles.size(), size_t{0});
    CHECK_EQ(result.frontages.size(), size_t{0});
    CHECK_FALSE(result.has_front());
}

/**
 * Too few points, or three collinear ones, enclose no interior, so "outward" has no
 * meaning and every normal would be decided by rounding.
 *
 * The empty, one-point and two-point rings all reach the area guard with fewer
 * points than an area needs, and every one of them is rejected by it as well as by
 * the point-count guard -- signed_ring_area() returns 0 for any ring under three
 * points. No output can therefore separate the two guards, and this test does not
 * pretend to: it pins the OUTCOME for every degenerate shape, and the point-count
 * guard stands on its own as the precondition for the modulo arithmetic that
 * follows it, which an empty ring would otherwise divide by zero.
 */
TEST(LotEdges, degenerate_ring_is_malformed) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline empty;
    const LotEdgeClassification from_empty =
        classify_lot_edges(empty, fixture.block(), fixture.graph);
    CHECK_TRUE(from_empty.malformed);
    CHECK_EQ(from_empty.roles.size(), size_t{0});

    LotOutline one_point;
    one_point.ring = {{20.0, 0.0}};
    one_point.edge_sources = {kNoBoundaryEdge};
    CHECK_TRUE(classify_lot_edges(one_point, fixture.block(), fixture.graph).malformed);

    LotOutline two_points;
    two_points.ring = {{20.0, 0.0}, {40.0, 0.0}};
    two_points.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge};
    CHECK_TRUE(classify_lot_edges(two_points, fixture.block(), fixture.graph).malformed);

    LotOutline collinear;
    collinear.ring = {{20.0, 0.0}, {40.0, 0.0}, {60.0, 0.0}};
    collinear.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};
    CHECK_TRUE(classify_lot_edges(collinear, fixture.block(), fixture.graph).malformed);
}

/**
 * A source indexing past the end of `Block::edges` -- the lot was cut from a
 * different block, or an EdgeId was stored where a Block::edges index belongs --
 * is counted and dropped rather than read out of bounds or trusted.
 */
TEST(LotEdges, source_past_the_end_of_block_edges_is_counted_and_ignored) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    LotOutline lot = lot_on_bottom(999);

    const LotEdgeClassification result =
        classify_lot_edges(lot, fixture.block(), fixture.graph);

    CHECK_FALSE(result.malformed);
    CHECK_EQ(result.invalid_sources, uint32_t{1});
    CHECK_TRUE(result.landlocked);
    CHECK_EQ(role_string(result), std::string("Interior,Interior,Interior,Interior"));
}

/**
 * The other way a source can be out of range, and the one no lot can cause: the
 * lot's index into `Block::edges` is fine, and the half-edge it lands on names a
 * graph edge the graph does not have. That is a Block outliving the RoadGraph it
 * was extracted from, or a caller pairing a block with the wrong graph, and it is
 * counted in the same `invalid_sources` as the first kind because both mean the
 * same thing to a reader: this lot's provenance could not be resolved.
 *
 * The block here is a REAL extraction with one half-edge rewritten afterwards,
 * because nothing a lot can pass in reaches this branch -- the check exists for a
 * block that has gone stale, and a stale block is what it is handed.
 */
TEST(LotEdges, a_block_half_edge_pointing_past_the_graph_is_counted_and_ignored) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);

    Block stale = fixture.block();
    CHECK((bottom) < (stale.edges.size()));
    if (bottom >= stale.edges.size()) {
        return;
    }
    stale.edges[bottom].edge = static_cast<EdgeId>(9999);

    const LotEdgeClassification result =
        classify_lot_edges(lot_on_bottom(bottom), stale, fixture.graph);

    CHECK_FALSE(result.malformed);
    CHECK_EQ(result.invalid_sources, uint32_t{1});
    CHECK_TRUE(result.landlocked);

    // SIDE, not INTERIOR: the edge does bound the block, so it is a boundary the
    // classifier could not name rather than a cut the subdivider made.
    CHECK_EQ(role_string(result), std::string("Side,Interior,Interior,Interior"));
}

// ============================================================================
// The Batch Call
// ============================================================================

/**
 * The batch call reports what it saw, and ALL FOURTEEN counters are asserted.
 *
 * Seven lots go in: a plain one, a landlocked one, a malformed one, a corner one, a
 * clockwise one, a sliver one and one whose source indexes past `Block::edges`.
 * The last three are there for the four counters that nothing else would move --
 * `orientation_corrected`, `demoted_frontages`, `invalid_sources` and
 * `inferred_sources` -- which are precisely the silent-failure counters the stats
 * struct exists for. A run that corrected every ring's winding, or demoted every
 * frontage in the extract, returns exactly as many lots with exactly as many edges
 * as a correct one, and only these four say so.
 *
 * Every count is asserted against the roles the single-lot tests above already
 * pinned down, so the two cannot drift. The malformed lot is in the middle of the
 * list on purpose: it must not stop the lots after it being classified.
 */
TEST(LotEdges, batch_stats_count_every_lot_every_role_and_every_repair) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const uint32_t bottom = fixture.block_edge_for_way(kBottomWay);
    const uint32_t left = fixture.block_edge_for_way(kLeftWay);

    LotOutline broken = lot_on_bottom(bottom);
    broken.edge_sources.pop_back();

    // lot_on_bottom() wound the other way; see clockwise_ring_is_corrected...
    LotOutline clockwise;
    clockwise.ring = {{20.0, 30.0}, {40.0, 30.0}, {40.0, 0.0}, {20.0, 0.0}};
    clockwise.edge_sources = {kNoBoundaryEdge, kNoBoundaryEdge, bottom, kNoBoundaryEdge};

    // 40 cm of contact with the bottom street; see sliver_frontage_is_demoted...
    LotOutline sliver;
    sliver.ring = {{20.0, 0.0}, {20.4, 0.0}, {20.4, 20.0}, {20.0, 20.0}};
    sliver.edge_sources = {bottom, kNoBoundaryEdge, kNoBoundaryEdge, kNoBoundaryEdge};

    const std::vector<LotOutline> lots{lot_on_bottom(bottom),
                                       lot_interior(),
                                       broken,
                                       lot_on_corner(bottom, left, 30.0, 12.0),
                                       clockwise,
                                       sliver,
                                       lot_on_bottom(999)};

    const LotEdgeClassificationSet set =
        classify_block_lots(lots, fixture.block(), fixture.graph);

    CHECK_EQ(set.lots.size(), size_t{7});
    CHECK_EQ(set.stats.lots, size_t{7});
    CHECK_EQ(set.stats.classified, size_t{6});
    CHECK_EQ(set.stats.malformed, size_t{1});
    CHECK_EQ(set.stats.landlocked, size_t{3});   // interior, sliver, invalid source
    CHECK_EQ(set.stats.corner_lots, size_t{1});

    // The four silent-failure counters, one lot each.
    CHECK_EQ(set.stats.orientation_corrected, size_t{1});
    CHECK_EQ(set.stats.demoted_frontages, size_t{1});
    CHECK_EQ(set.stats.invalid_sources, size_t{1});
    CHECK_EQ(set.stats.inferred_sources, size_t{0});   // the inference is off here

    CHECK_EQ(set.landlocked_lots.size(), size_t{3});
    CHECK_EQ(set.malformed_lots.size(), size_t{1});
    if (set.landlocked_lots.size() == 3 && set.malformed_lots.size() == 1) {
        CHECK_EQ(set.landlocked_lots[0], uint32_t{1});
        CHECK_EQ(set.landlocked_lots[1], uint32_t{5});
        CHECK_EQ(set.landlocked_lots[2], uint32_t{6});
        CHECK_EQ(set.malformed_lots[0], uint32_t{2});
    }

    // Front:   1 (plain) + 2 (corner) + 1 (clockwise)             = 4
    // Side:    2 (plain) + 1 (corner) + 2 (clockwise) + 1 (sliver) = 6
    // Back:    1 (plain) + 1 (corner) + 1 (clockwise)             = 3
    // Interior:          4 (landlocked) + 3 (sliver) + 4 (invalid) = 11
    CHECK_EQ(set.stats.front_edges, size_t{4});
    CHECK_EQ(set.stats.side_edges, size_t{6});
    CHECK_EQ(set.stats.back_edges, size_t{3});
    CHECK_EQ(set.stats.interior_edges, size_t{11});

    // The classification after the malformed one is real, not a default.
    CHECK_EQ(role_string(set.lots[3]), std::string("Front,Side,Back,Front"));
    CHECK_EQ(role_string(set.lots[4]), std::string("Back,Side,Front,Side"));
    CHECK_EQ(role_string(set.lots[5]), std::string("Side,Interior,Interior,Interior"));
}

/**
 * `inferred_sources` is the one batch counter the test above cannot move, because
 * the inference is off by default and turning it on changes what the other lots
 * classify as. It gets its own pair: one lot, one block, the config flag the only
 * difference between the two halves.
 */
TEST(LotEdges, batch_stats_count_the_sources_the_inference_recovered) {
    const BlockFixture fixture = make_rect_block();
    CHECK_TRUE(fixture.ok());
    if (!fixture.ok()) {
        return;
    }

    const std::vector<LotOutline> lots{lot_on_bottom(kNoBoundaryEdge)};

    const LotEdgeClassificationSet without =
        classify_block_lots(lots, fixture.block(), fixture.graph);
    CHECK_EQ(without.stats.inferred_sources, size_t{0});
    CHECK_EQ(without.stats.landlocked, size_t{1});

    LotEdgeConfig config;
    config.infer_sources_from_block_ring = true;
    const LotEdgeClassificationSet with =
        classify_block_lots(lots, fixture.block(), fixture.graph, config);
    CHECK_EQ(with.stats.inferred_sources, size_t{1});
    CHECK_EQ(with.stats.landlocked, size_t{0});
    CHECK_EQ(with.stats.front_edges, size_t{1});
}

// ============================================================================
// Shared Predicates
// ============================================================================

/**
 * The predicates E3 and the rule engine will call have to mean here what they mean
 * there, and street_class_rank() leans on RoadType's declared order. The order is
 * pinned by static_asserts in lot_edges.cpp, which fail the BUILD; this covers the
 * part a static_assert cannot, which is that the predicate reads its argument.
 */
TEST(LotEdges, street_class_predicate_and_rank_agree_with_the_enum_order) {
    CHECK_TRUE(is_street_class(RoadType::Residential, false));
    CHECK_TRUE(is_street_class(RoadType::Service, false));
    CHECK_TRUE(is_street_class(RoadType::Unknown, false));   // an unmapped road strands lots
    CHECK_FALSE(is_street_class(RoadType::Footway, false));
    CHECK_FALSE(is_street_class(RoadType::Cycleway, false));
    CHECK_FALSE(is_street_class(RoadType::Path, false));

    CHECK_TRUE(is_street_class(RoadType::Footway, true));
    CHECK_TRUE(is_street_class(RoadType::Cycleway, true));
    CHECK_TRUE(is_street_class(RoadType::Path, true));

    CHECK((street_class_rank(RoadType::Motorway)) < (street_class_rank(RoadType::Primary)));
    CHECK((street_class_rank(RoadType::Primary)) < (street_class_rank(RoadType::Residential)));
    CHECK((street_class_rank(RoadType::Residential)) < (street_class_rank(RoadType::Service)));
    CHECK((street_class_rank(RoadType::Service)) < (street_class_rank(RoadType::Unknown)));
}
