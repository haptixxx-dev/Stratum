// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_lots.cpp
 * @brief Lot subdivision tests against blocks whose answer is known by hand
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Every block here is written out as a ring, not extracted from a fixture,
 * because the point of each test is an answer stated in advance -- "a 100 m
 * square with lotAreaMax 2500 is four lots of exactly 2500 m^2, centred on
 * (25,25), (25,75), (75,25) and (75,75)". A fixture would leave the count
 * assertable and nothing else.
 *
 * Two kinds of assertion appear repeatedly, and both exist because the failure
 * mode of a subdivision is silent:
 *
 *   - **Areas and positions, not counts.** "Four lots" is also what a broken
 *     split that cut along the wrong axis produces. Four lots of the stated area
 *     at the stated centroids is not.
 *   - **The negative case alongside the positive one.** A determinism test passes
 *     trivially against an implementation that ignores its seed, so the test that
 *     asserts two runs agree sits next to one asserting that two SEEDS disagree.
 *     The same pairing guards force_street_access, which is otherwise satisfied by
 *     never landlocking anything, and the seed-independence property, which is
 *     otherwise satisfied by a seed that reaches nothing at all.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests Lots
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/blocks.hpp"
#include "osm/road/lots.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
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
using stratum::osm::road::BlockEdge;
using stratum::osm::road::Lot;
using stratum::osm::road::LotMatchConfig;
using stratum::osm::road::LotParams;
using stratum::osm::road::LotSubdivision;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::block_key_from_ways;
using stratum::osm::road::extract_blocks;
using stratum::osm::road::kInvalidLotId;
using stratum::osm::road::kNoBlockEdge;
using stratum::osm::road::lot_id_for_key;
using stratum::osm::road::lot_root_key;
using stratum::osm::road::map_point_between_rings;
using stratum::osm::road::mean_value_coordinates;
using stratum::osm::road::signed_ring_area;
using stratum::osm::road::subdivide_block;
using stratum::osm::road::subdivide_ring;
using stratum::osm::road::transfer_lot_ids;

using Ring = std::vector<glm::dvec2>;

// ============================================================================
// Block Construction
// ============================================================================

/**
 * @brief Wrap a ring as a Block, tagging segment i with block edge i
 *
 * One block edge per ring segment, which is the simplest tagging that is still
 * distinguishable: a lot that ends up with the wrong tag names a different
 * street, and the tests below state which tags each lot must carry.
 */
Block make_block(Ring ring) {
    Block block;
    block.ring = std::move(ring);
    block.ring_edges.resize(block.ring.size());
    block.edges.resize(block.ring.size());

    for (size_t i = 0; i < block.ring.size(); ++i) {
        block.ring_edges[i] = static_cast<uint32_t>(i);
        block.edges[i] = BlockEdge{static_cast<uint32_t>(i), true};
    }

    block.area = signed_ring_area(block.ring);
    return block;
}

/// Anticlockwise axis-aligned rectangle with its lower-left corner at the origin
Ring rect(double width, double height) {
    return Ring{{0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height}};
}

/**
 * @brief A U: 60 wide, 90 tall, with a 20 x 60 notch cut down from the top
 *
 * The non-convex case, and specifically the one where a single cut across the
 * shape produces TWO pieces on one side. A convex-only implementation, or a
 * Sutherland-Hodgman clip, returns the two arms joined by a zero-width bridge and
 * every area below is wrong.
 */
Ring u_block() {
    return Ring{{0.0, 0.0},  {60.0, 0.0},  {60.0, 90.0}, {40.0, 90.0},
                {40.0, 30.0}, {20.0, 30.0}, {20.0, 90.0}, {0.0, 90.0}};
}

/// Parameters with every source of variation switched off unless a test asks
LotParams base_params() {
    LotParams params;
    params.corner_angle_max_deg = 0.0;
    params.corner_width = 12.0;
    params.irregularity = 0.0;
    params.force_street_access = 1.0;
    params.lot_area_min = 200.0;
    params.lot_area_max = 800.0;
    params.lot_width_min = 8.0;
    params.block_key = 0x0123456789ABCDEFull;
    params.seed = 7;
    return params;
}

// ============================================================================
// Assertion Helpers
// ============================================================================

bool points_equal(const glm::dvec2& a, const glm::dvec2& b, double eps = 1e-9) {
    return std::fabs(a.x - b.x) <= eps && std::fabs(a.y - b.y) <= eps;
}

/// Area centroid of a ring, first point not repeated
glm::dvec2 centroid_of(const Ring& ring) {
    double twice_area = 0.0;
    glm::dvec2 acc{0.0};
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % ring.size()];
        const double c = a.x * b.y - b.x * a.y;
        twice_area += c;
        acc += (a + b) * c;
    }
    if (std::fabs(twice_area) < 1e-12) {
        return glm::dvec2{0.0};
    }
    return acc / (3.0 * twice_area);
}

/// Spread of a ring along a direction, in metres
double extent_along(const Ring& ring, const glm::dvec2& axis) {
    if (ring.empty()) {
        return 0.0;
    }
    double lo = glm::dot(ring[0], axis);
    double hi = lo;
    for (const glm::dvec2& p : ring) {
        const double t = glm::dot(p, axis);
        lo = std::min(lo, t);
        hi = std::max(hi, t);
    }
    return hi - lo;
}

double total_area(const LotSubdivision& sub) {
    double total = 0.0;
    for (const Lot& lot : sub.lots) {
        total += lot.area;
    }
    return total;
}

std::vector<double> sorted_areas(const LotSubdivision& sub) {
    std::vector<double> areas;
    areas.reserve(sub.lots.size());
    for (const Lot& lot : sub.lots) {
        areas.push_back(lot.area);
    }
    std::sort(areas.begin(), areas.end());
    return areas;
}

/// Index of the lot whose centroid is within @p eps of @p point, or lots.size()
size_t lot_at(const std::vector<Lot>& lots, const glm::dvec2& point, double eps = 1e-6) {
    for (size_t i = 0; i < lots.size(); ++i) {
        if (points_equal(centroid_of(lots[i].ring), point, eps)) {
            return i;
        }
    }
    return lots.size();
}

/// Bit-for-bit ring equality. Nothing looser: determinism is a claim about bits.
bool rings_identical(const Ring& a, const Ring& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].x != b[i].x || a[i].y != b[i].y) {
            return false;
        }
    }
    return true;
}

/// Every segment tag of @p lot that names a block edge, sorted
std::vector<uint32_t> tagged_edges(const Lot& lot) {
    std::vector<uint32_t> tags;
    for (const uint32_t tag : lot.ring_edges) {
        if (tag != kNoBlockEdge) {
            tags.push_back(tag);
        }
    }
    std::sort(tags.begin(), tags.end());
    return tags;
}

double segment_length(const Ring& ring, size_t i) {
    const glm::dvec2 d = ring[(i + 1) % ring.size()] - ring[i];
    return std::sqrt(d.x * d.x + d.y * d.y);
}

/// Total length of a closed ring, first point not repeated
double ring_perimeter(const Ring& ring) {
    double total = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        total += segment_length(ring, i);
    }
    return total;
}

/// Length of the segments of @p lot that name block edge @p tag
double frontage_on_edge(const Lot& lot, uint32_t tag) {
    double total = 0.0;
    for (size_t i = 0; i < lot.ring.size(); ++i) {
        if (lot.ring_edges[i] == tag) {
            total += segment_length(lot.ring, i);
        }
    }
    return total;
}

double total_frontage(const LotSubdivision& sub) {
    double total = 0.0;
    for (const Lot& lot : sub.lots) {
        total += lot.frontage_length;
    }
    return total;
}

/// @p ring with its start moved to vertex @p start; same shape, same winding
Ring rotated(const Ring& ring, size_t start) {
    Ring out;
    out.reserve(ring.size());
    for (size_t i = 0; i < ring.size(); ++i) {
        out.push_back(ring[(start + i) % ring.size()]);
    }
    return out;
}

/// The corner lot with a vertex at @p apex, or lots.size()
size_t corner_lot_at(const std::vector<Lot>& lots, const glm::dvec2& apex) {
    for (size_t i = 0; i < lots.size(); ++i) {
        if (!lots[i].is_corner_lot) {
            continue;
        }
        for (const glm::dvec2& p : lots[i].ring) {
            if (points_equal(p, apex, 1e-9)) {
                return i;
            }
        }
    }
    return lots.size();
}

/**
 * @brief The one interior-cut segment of @p lot, as its two endpoints
 *
 * A piece of a single cut through a convex block has exactly one such segment, so
 * "more than one" is reported as failure rather than as the first one found.
 *
 * @return false when the lot does not have exactly one kNoBlockEdge segment
 */
bool cut_segment(const Lot& lot, glm::dvec2& from, glm::dvec2& to) {
    size_t found = lot.ring.size();
    for (size_t i = 0; i < lot.ring.size(); ++i) {
        if (lot.ring_edges[i] != kNoBlockEdge) {
            continue;
        }
        if (found != lot.ring.size()) {
            return false;
        }
        found = i;
    }
    if (found == lot.ring.size()) {
        return false;
    }
    from = lot.ring[found];
    to = lot.ring[(found + 1) % lot.ring.size()];
    return true;
}

/// A lot with a stated id and ring, for the transfer tests
Lot lot_with(uint64_t id, const Ring& ring) {
    Lot lot;
    lot.id = id;
    lot.ring = ring;
    lot.ring_edges.assign(ring.size(), kNoBlockEdge);
    lot.area = signed_ring_area(ring);
    return lot;
}

/// An axis-aligned square of side @p side centred on @p centre, anticlockwise
Ring square_at(const glm::dvec2& centre, double side) {
    const double h = 0.5 * side;
    return Ring{{centre.x - h, centre.y - h},
                {centre.x + h, centre.y - h},
                {centre.x + h, centre.y + h},
                {centre.x - h, centre.y + h}};
}

// ============================================================================
// Graph Construction, for the block key only
// ============================================================================

using NodeMap = std::unordered_map<NodeId, glm::dvec2>;

void add_way(ParsedOSMData& data, const NodeMap& positions, WayId id,
             const std::vector<NodeId>& ids) {
    Road road;
    road.osm_id = id;
    road.type = RoadType::Residential;
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

/// Build the graph, extract its single block, and return block_key_from_ways() of it
uint64_t key_of(const ParsedOSMData& data) {
    RoadGraph graph;
    graph.build(data);
    const auto extraction = extract_blocks(graph);
    if (extraction.blocks.size() != 1) {
        stratum::test::report_failure(__FILE__, __LINE__, "extraction has one block",
                                      "blocks: " + std::to_string(extraction.blocks.size()));
        return 0;
    }
    return block_key_from_ways(extraction.blocks[0], graph);
}

/**
 * @brief The four ways round a square, added in the order @p order names them
 *
 * `order[k]` is the index of the side to add k-th, so the same four ways can be
 * handed to the graph in any traversal order.
 */
ParsedOSMData square_ways(const NodeMap& positions, const std::vector<WayId>& ids,
                          const std::vector<size_t>& order) {
    const std::vector<std::vector<NodeId>> sides{{1, 2}, {2, 3}, {3, 4}, {4, 1}};
    ParsedOSMData data;
    for (const size_t side : order) {
        add_way(data, positions, ids[side], sides[side]);
    }
    return data;
}

} // namespace

// ============================================================================
// Shape of the Subdivision
// ============================================================================

/**
 * @brief The block that divides evenly, with every number stated in advance
 *
 * 100 m square, lotAreaMax 2500: two halvings of the long axis produce four lots
 * of exactly 2500 m^2 at exactly the quarter centroids. Splitting along the wrong
 * axis, or pivoting anywhere but the midpoint, changes the centroids while
 * leaving the count at four, which is why the centroids are asserted.
 */
TEST(Lots, square_block_divides_into_four_equal_lots) {
    LotParams params = base_params();
    params.lot_area_max = 2500.0;
    params.lot_area_min = 1000.0;
    params.lot_width_min = 10.0;

    const Block block = make_block(rect(100.0, 100.0));
    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{4});
    CHECK_EQ(sub.stats.lots, size_t{4});
    CHECK_EQ(sub.stats.splits, size_t{3});
    CHECK_EQ(sub.stats.dropped_pieces, size_t{0});
    CHECK_NEAR(total_area(sub), 10000.0, 1e-9);

    for (const Lot& lot : sub.lots) {
        CHECK_NEAR(lot.area, 2500.0, 1e-9);
        CHECK_EQ(lot.depth, uint32_t{2});
        CHECK_FALSE(lot.is_corner_lot);
        CHECK(lot.id != kInvalidLotId);
    }

    CHECK(lot_at(sub.lots, glm::dvec2{25.0, 25.0}) < sub.lots.size());
    CHECK(lot_at(sub.lots, glm::dvec2{25.0, 75.0}) < sub.lots.size());
    CHECK(lot_at(sub.lots, glm::dvec2{75.0, 25.0}) < sub.lots.size());
    CHECK(lot_at(sub.lots, glm::dvec2{75.0, 75.0}) < sub.lots.size());
}

/**
 * @brief A block under lotAreaMin is ONE lot, never zero
 *
 * lotAreaMin is a floor on splitting, not a filter on lots. Treating it as a
 * filter deletes the land, and the symptom downstream is a hole in the city that
 * nothing reports.
 */
TEST(Lots, block_below_lot_area_min_yields_one_lot) {
    LotParams params = base_params();
    params.lot_area_min = 200.0;
    params.lot_area_max = 800.0;

    const Block block = make_block(rect(10.0, 10.0));
    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{1});
    if (sub.lots.size() != 1) {
        return;
    }

    CHECK_NEAR(sub.lots[0].area, 100.0, 1e-12);
    CHECK_EQ(sub.lots[0].depth, uint32_t{0});
    CHECK_EQ(sub.lots[0].ring.size(), size_t{4});
    CHECK(sub.lots[0].id != kInvalidLotId);
    // The whole boundary is street, so the lot fronts all four sides.
    CHECK_NEAR(sub.lots[0].frontage_length, 40.0, 1e-12);
    CHECK_EQ(sub.stats.splits, size_t{0});
}

/**
 * @brief A long thin block is cut ACROSS its length, never shaved along it
 *
 * 200 x 10 with lotAreaMax 500 is four lots of 50 x 10. The failure this catches
 * is splitting along the smallest OBB edge instead of across the largest: that
 * would give 200 x 5 ribbons, which lotWidthMin then vetoes, and the block would
 * come back as one lot. Asserting the per-lot extents distinguishes the two --
 * asserting only the count and the areas would not, because four 200 x 2.5
 * ribbons also total 2000.
 */
TEST(Lots, long_thin_block_splits_across_the_long_axis) {
    LotParams params = base_params();
    params.lot_area_max = 500.0;
    params.lot_area_min = 100.0;
    params.lot_width_min = 8.0;

    const Block block = make_block(rect(200.0, 10.0));
    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{4});
    CHECK_NEAR(total_area(sub), 2000.0, 1e-9);

    for (const Lot& lot : sub.lots) {
        CHECK_NEAR(lot.area, 500.0, 1e-9);
        CHECK_NEAR(extent_along(lot.ring, glm::dvec2{1.0, 0.0}), 50.0, 1e-9);
        CHECK_NEAR(extent_along(lot.ring, glm::dvec2{0.0, 1.0}), 10.0, 1e-9);
    }

    CHECK(lot_at(sub.lots, glm::dvec2{25.0, 5.0}) < sub.lots.size());
    CHECK(lot_at(sub.lots, glm::dvec2{175.0, 5.0}) < sub.lots.size());
}

/**
 * @brief A cut across a U produces two pieces above the line and one below
 *
 * The non-convex case. The 60 x 90 U has area 4200; its minimum-area box is the
 * full 60 x 90 rectangle, so the first cut is horizontal at y = 45 and severs
 * both arms at once. The answer is one 2400 m^2 piece below and two 900 m^2 arms
 * above, and the areas are what distinguishes a correct multi-piece split from a
 * clip that stitched the two arms into one ring: that ring's area would be 1800,
 * the count would be two, and nothing else would look wrong.
 */
TEST(Lots, non_convex_block_split_yields_a_lot_on_each_arm) {
    LotParams params = base_params();
    params.lot_area_max = 2500.0;
    params.lot_area_min = 500.0;
    params.lot_width_min = 5.0;

    const Block block = make_block(u_block());
    CHECK_NEAR(block.area, 4200.0, 1e-9);

    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{3});
    CHECK_EQ(sub.stats.splits, size_t{1});
    CHECK_EQ(sub.stats.dropped_pieces, size_t{0});
    CHECK_NEAR(total_area(sub), 4200.0, 1e-9);

    const std::vector<double> areas = sorted_areas(sub);
    if (areas.size() == 3) {
        CHECK_NEAR(areas[0], 900.0, 1e-9);
        CHECK_NEAR(areas[1], 900.0, 1e-9);
        CHECK_NEAR(areas[2], 2400.0, 1e-9);
    }

    // The two arms are separate parcels, so their centroids sit on the two arms
    // and not on the bridge between them.
    CHECK(lot_at(sub.lots, glm::dvec2{10.0, 67.5}) < sub.lots.size());
    CHECK(lot_at(sub.lots, glm::dvec2{50.0, 67.5}) < sub.lots.size());
}

/// A ring with fewer than three points is not land and yields nothing
TEST(Lots, degenerate_block_yields_no_lots) {
    Block block;
    block.ring = Ring{{0.0, 0.0}, {10.0, 0.0}};
    block.ring_edges = {0, 0};
    block.edges.push_back(BlockEdge{0, true});

    const LotSubdivision sub = subdivide_block(block, base_params());
    CHECK_EQ(sub.lots.size(), size_t{0});
    CHECK_EQ(sub.stats.lots, size_t{0});
}

// ============================================================================
// Frontage Tagging
// ============================================================================

/**
 * @brief Each lot segment still names the block edge it was cut from
 *
 * C4 classifies frontage from Lot::ring_edges, so the tags have to be right and
 * not merely present. The lot at (25,25) of a 100 m square is bounded by the
 * bottom edge (tag 0) and the left edge (tag 3) and by two interior cuts; a
 * tagging bug that shifted every tag by one would leave the counts identical and
 * the streets wrong, so the tag VALUES are asserted.
 */
TEST(Lots, lot_segments_name_the_block_edge_they_came_from) {
    LotParams params = base_params();
    params.lot_area_max = 2500.0;
    params.lot_area_min = 1000.0;
    params.lot_width_min = 10.0;

    const Block block = make_block(rect(100.0, 100.0));
    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{4});

    double frontage_total = 0.0;
    for (const Lot& lot : sub.lots) {
        CHECK_EQ(lot.ring_edges.size(), lot.ring.size());

        const std::vector<uint32_t> tags = tagged_edges(lot);
        CHECK_EQ(tags.size(), size_t{2});
        for (const uint32_t tag : tags) {
            CHECK(tag < block.edges.size());
        }

        // Two street sides of 50 m each, and two cut sides fronting nothing.
        CHECK_NEAR(lot.frontage_length, 100.0, 1e-9);
        frontage_total += lot.frontage_length;
    }

    // Every metre of the block boundary ends up on exactly one lot.
    CHECK_NEAR(frontage_total, 400.0, 1e-9);

    const size_t corner = lot_at(sub.lots, glm::dvec2{25.0, 25.0});
    CHECK(corner < sub.lots.size());
    if (corner < sub.lots.size()) {
        const std::vector<uint32_t> tags = tagged_edges(sub.lots[corner]);
        CHECK_EQ(tags.size(), size_t{2});
        if (tags.size() == 2) {
            CHECK_EQ(tags[0], uint32_t{0});   // the bottom edge
            CHECK_EQ(tags[1], uint32_t{3});   // the left edge
        }
    }

    const size_t far = lot_at(sub.lots, glm::dvec2{75.0, 75.0});
    CHECK(far < sub.lots.size());
    if (far < sub.lots.size()) {
        const std::vector<uint32_t> tags = tagged_edges(sub.lots[far]);
        if (tags.size() == 2) {
            CHECK_EQ(tags[0], uint32_t{1});   // the right edge
            CHECK_EQ(tags[1], uint32_t{2});   // the top edge
        }
    }
}

/**
 * @brief A block edge lying ON the cut line keeps its tag
 *
 * The silent one. This block is 100 x 200 with a notch cut down to y = 100; its
 * minimum-area box has its long axis along y, so the first cut lands exactly on
 * the notch floor, the 20 m street segment from (60,100) to (40,100). The split
 * used to skip that span on the grounds that the chain pairing covers it anyway
 * -- and it does cover it, with a synthetic cut segment tagged kNoBlockEdge,
 * which destroys the street.
 *
 * Nothing else notices: the area still balances to 18000, dropped_pieces stays 0,
 * and the lot count is what you would expect. The only visible symptom is 20 m of
 * street missing from the frontage total, and C4 then classifying a lot that
 * genuinely fronts a street as fronting nothing there.
 *
 * Frontage against the block PERIMETER is the invariant that states it: every
 * metre of the block boundary belongs to exactly one lot, so the two numbers are
 * equal whether or not the boundary happens to coincide with a cut.
 */
TEST(Lots, frontage_survives_a_block_edge_lying_on_the_cut_line) {
    LotParams params = base_params();
    params.lot_area_max = 10000.0;
    params.lot_area_min = 1000.0;
    params.lot_width_min = 10.0;

    const Ring notch{{0.0, 0.0},   {100.0, 0.0},   {100.0, 200.0}, {60.0, 200.0},
                     {60.0, 100.0}, {40.0, 100.0}, {40.0, 200.0},  {0.0, 200.0}};
    const Block block = make_block(notch);
    CHECK_NEAR(block.area, 18000.0, 1e-9);
    CHECK_NEAR(ring_perimeter(notch), 800.0, 1e-9);

    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{3});
    CHECK_EQ(sub.stats.splits, size_t{1});
    CHECK_EQ(sub.stats.dropped_pieces, size_t{0});
    CHECK_NEAR(total_area(sub), 18000.0, 1e-9);

    // The claim, stated as the invariant rather than as a lot count.
    CHECK_NEAR(total_frontage(sub), ring_perimeter(notch), 1e-9);

    // And stated again as the one edge that used to vanish. Block edge 4 is the
    // notch floor, the segment from (60,100) to (40,100).
    double notch_floor = 0.0;
    size_t carriers = 0;
    for (const Lot& lot : sub.lots) {
        const double on_edge = frontage_on_edge(lot, 4);
        notch_floor += on_edge;
        if (on_edge > 0.0) {
            ++carriers;
        }
    }
    CHECK_NEAR(notch_floor, 20.0, 1e-9);
    CHECK_EQ(carriers, size_t{1});

    // It belongs to the land BELOW the notch, because the ring is anticlockwise
    // and that edge runs from x=60 to x=40, which puts the interior at -y. Giving
    // it to an arm above would conserve the total and still be wrong.
    const size_t below = lot_at(sub.lots, glm::dvec2{50.0, 50.0}, 1e-9);
    CHECK(below < sub.lots.size());
    if (below < sub.lots.size()) {
        CHECK_NEAR(sub.lots[below].area, 10000.0, 1e-9);
        CHECK_NEAR(frontage_on_edge(sub.lots[below], 4), 20.0, 1e-9);
        // Three full sides of street plus the notch floor.
        CHECK_NEAR(sub.lots[below].frontage_length, 320.0, 1e-9);
    }
}

/**
 * @brief forceStreetAccess 1 leaves no lot landlocked, and 0 does
 *
 * Both halves are asserted on purpose. "Every lot has frontage" is also true of
 * an implementation that never subdivides deeply enough to bury one, so the
 * second half proves the 300 m block really does produce interior lots when the
 * constraint is off.
 */
TEST(Lots, force_street_access_keeps_every_lot_on_a_street) {
    LotParams params = base_params();
    params.lot_area_max = 2500.0;
    params.lot_area_min = 200.0;
    params.lot_width_min = 5.0;

    const Block block = make_block(rect(300.0, 300.0));

    LotParams forced = params;
    forced.force_street_access = 1.0;
    const LotSubdivision on = subdivide_block(block, forced);

    CHECK(on.lots.size() > 4);
    CHECK_NEAR(total_area(on), 90000.0, 1e-6);
    for (const Lot& lot : on.lots) {
        CHECK(lot.frontage_length > 0.0);
    }
    CHECK(on.stats.rejected_access > 0);

    LotParams free_form = params;
    free_form.force_street_access = 0.0;
    const LotSubdivision off = subdivide_block(block, free_form);

    CHECK_NEAR(total_area(off), 90000.0, 1e-6);
    CHECK_EQ(off.stats.rejected_access, size_t{0});

    size_t landlocked = 0;
    for (const Lot& lot : off.lots) {
        if (lot.frontage_length <= 0.0) {
            ++landlocked;
        }
    }
    CHECK(landlocked > 0);
}

/**
 * @brief lotWidthMin refuses a cut that would produce a ribbon
 *
 * A 100 x 12 block with lotWidthMin 20 has no legal cut: across the long axis
 * gives 50 x 12, along it gives 100 x 6, and both are narrower than 20. The block
 * stays whole, and the two rejections are counted, so the test can tell "refused
 * twice" from "never tried".
 */
TEST(Lots, lot_width_min_refuses_a_ribbon_cut) {
    LotParams params = base_params();
    params.lot_area_max = 400.0;
    params.lot_area_min = 100.0;
    params.lot_width_min = 20.0;

    const Block block = make_block(rect(100.0, 12.0));
    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{1});
    CHECK_EQ(sub.stats.splits, size_t{0});
    CHECK_EQ(sub.stats.split_attempts, size_t{1});
    // Once for the cut across the long axis, once for the fallback along it.
    CHECK_EQ(sub.stats.rejected_width, size_t{2});
    CHECK_NEAR(total_area(sub), 1200.0, 1e-9);
}

/// max_depth is a backstop, and hitting it is counted rather than silent
TEST(Lots, max_depth_stops_the_recursion_and_says_so) {
    LotParams params = base_params();
    params.lot_area_max = 500.0;
    params.lot_area_min = 100.0;
    params.lot_width_min = 5.0;
    params.max_depth = 1;

    const Block block = make_block(rect(160.0, 120.0));
    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{2});
    CHECK_EQ(sub.stats.splits, size_t{1});
    CHECK_EQ(sub.stats.depth_limit_hits, size_t{2});
    CHECK_EQ(sub.stats.max_depth_reached, uint32_t{1});
    CHECK_NEAR(total_area(sub), 19200.0, 1e-9);

    // The lots are over lotAreaMax, which is exactly what a backstop looks like.
    for (const Lot& lot : sub.lots) {
        CHECK(lot.area > params.lot_area_max);
    }
}

// ============================================================================
// Corner Lots
// ============================================================================

/**
 * @brief Only a corner sharper than cornerAngleMax gets its own lot
 *
 * The quad below has one 33.69 degree corner at the origin and nothing else under
 * 90 degrees. With cornerWidth 10 the wedge is 0.5 * 10 * 10 * sin(33.69) =
 * 27.735 m^2, which is the number asserted -- a wedge cut at the wrong reach, or
 * measured from the wrong pair of edges, still comes out as "one corner lot".
 *
 * The square is the other half: 90 degree corners must be left completely alone,
 * or every ordinary block in the city would sprout four triangles.
 */
TEST(Lots, corner_lots_are_cut_only_from_sharp_corners) {
    LotParams params = base_params();
    params.corner_angle_max_deg = 40.0;
    params.corner_width = 10.0;
    params.lot_area_max = 1.0e6;   // the remainder must survive as one lot
    params.lot_area_min = 1.0;

    const Block wedge = make_block(Ring{{0.0, 0.0}, {120.0, 0.0}, {120.0, 40.0}, {60.0, 40.0}});
    CHECK_NEAR(wedge.area, 3600.0, 1e-9);

    const LotSubdivision sub = subdivide_block(wedge, params);

    CHECK_EQ(sub.stats.corner_lots, size_t{1});
    CHECK_EQ(sub.lots.size(), size_t{2});
    CHECK_NEAR(total_area(sub), 3600.0, 1e-9);

    size_t corner_index = sub.lots.size();
    for (size_t i = 0; i < sub.lots.size(); ++i) {
        if (sub.lots[i].is_corner_lot) {
            corner_index = i;
        }
    }
    CHECK(corner_index < sub.lots.size());
    if (corner_index < sub.lots.size()) {
        const Lot& corner = sub.lots[corner_index];
        CHECK_NEAR(corner.area, 27.7350098112614, 1e-9);
        CHECK_EQ(corner.ring.size(), size_t{3});
        // Two sides on the street, one chord across the block.
        CHECK_EQ(tagged_edges(corner).size(), size_t{2});
        CHECK(corner.frontage_length > 0.0);
    }

    const Block square = make_block(rect(100.0, 100.0));
    const LotSubdivision untouched = subdivide_block(square, params);
    CHECK_EQ(untouched.stats.corner_lots, size_t{0});
    CHECK_EQ(untouched.lots.size(), size_t{1});
    CHECK_NEAR(total_area(untouched), 10000.0, 1e-9);
}

/**
 * @brief cornerWidth is clamped to half of each side, and two corners prove it
 *
 * The test above has exactly ONE corner lot on a block with no short sides, so
 * three documented mechanisms are invisible to it: the clamp never engages, the
 * corner rank is always 0 whatever it is ranked from, and a single depth is
 * consistent with any rule.
 *
 * This rhombus has two 6.87 degree corners, at (0,0) and (100,0), and all four
 * sides are sqrt(2509) = 50.0899 m. cornerWidth 40 is well past half of that, so
 * the clamp is what decides the wedge: each reach becomes 25.04495 m, which lands
 * exactly on the side midpoints (25, +-1.5) and (75, +-1.5). Unclamped, two
 * wedges reaching 40 m from opposite ends of a 50.09 m side would overlap and the
 * remainder would be self-intersecting -- so the coordinates are asserted, not
 * just the area.
 */
TEST(Lots, corner_width_is_clamped_to_half_of_each_side) {
    LotParams params = base_params();
    params.corner_angle_max_deg = 20.0;
    params.corner_width = 40.0;
    params.lot_area_max = 1.0e6;   // the remainder must survive as one lot
    params.lot_area_min = 1.0;

    const Ring rhombus{{0.0, 0.0}, {50.0, -3.0}, {100.0, 0.0}, {50.0, 3.0}};
    const Block block = make_block(rhombus);
    CHECK_NEAR(block.area, 300.0, 1e-12);

    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.stats.corner_lots, size_t{2});
    CHECK_EQ(sub.lots.size(), size_t{3});
    // Overlapping wedges would break this, and it is the cheapest statement of
    // the clamp that a reader can check by hand: 300 - 37.5 - 37.5 = 225.
    CHECK_NEAR(total_area(sub), 300.0, 1e-12);

    const size_t west = corner_lot_at(sub.lots, glm::dvec2{0.0, 0.0});
    const size_t east = corner_lot_at(sub.lots, glm::dvec2{100.0, 0.0});
    CHECK(west < sub.lots.size());
    CHECK(east < sub.lots.size());
    if (west >= sub.lots.size() || east >= sub.lots.size()) {
        return;
    }

    CHECK_NEAR(sub.lots[west].area, 37.5, 1e-9);
    CHECK_NEAR(sub.lots[east].area, 37.5, 1e-9);

    // Half of sqrt(2509) along each side, which is the side midpoint. At the
    // unclamped 40 m these would be (39.93, +-2.40) and the two wedges would
    // overlap in the middle of the block.
    CHECK_EQ(sub.lots[west].ring.size(), size_t{3});
    CHECK_EQ(sub.lots[east].ring.size(), size_t{3});
    CHECK_TRUE(points_equal(sub.lots[west].ring[0], glm::dvec2{25.0, 1.5}, 1e-9));
    CHECK_TRUE(points_equal(sub.lots[west].ring[2], glm::dvec2{25.0, -1.5}, 1e-9));
    CHECK_TRUE(points_equal(sub.lots[east].ring[0], glm::dvec2{75.0, -1.5}, 1e-9));
    CHECK_TRUE(points_equal(sub.lots[east].ring[2], glm::dvec2{75.0, 1.5}, 1e-9));

    // A corner lot is taken off the block, not out of the recursion, so it sits at
    // the root's depth whatever the recursion below it does.
    CHECK_EQ(sub.lots[west].depth, uint32_t{0});
    CHECK_EQ(sub.lots[east].depth, uint32_t{0});

    // The remainder keeps every metre of street that the two wedges did not take.
    CHECK_NEAR(total_frontage(sub), ring_perimeter(rhombus), 1e-9);
}

/**
 * @brief A corner lot's id follows its corner, not the ring's start vertex
 *
 * Corners are ranked from the ring's lexicographically smallest vertex rather
 * than from vertex 0, and this is the test that can see the difference. Vertex 0
 * is wherever the face walk in blocks.cpp began, which changes for reasons that
 * have nothing to do with this block; the lexicographic start is a property of
 * the shape.
 *
 * The same rhombus is subdivided from each of its four start vertices. Emission
 * order is allowed to change -- it does -- but the id on the corner at (0,0) must
 * be the same value in all four runs, and so must the one at (100,0), and the two
 * must never be each other's. Ranking from vertex 0 swaps them on two of the four
 * rotations, which is an id landing on the wrong piece of land.
 */
TEST(Lots, corner_lot_ids_follow_the_corner_through_a_rotated_ring) {
    LotParams params = base_params();
    params.corner_angle_max_deg = 20.0;
    params.corner_width = 40.0;
    params.lot_area_max = 1.0e6;
    params.lot_area_min = 1.0;

    const Ring rhombus{{0.0, 0.0}, {50.0, -3.0}, {100.0, 0.0}, {50.0, 3.0}};

    uint64_t west_id = kInvalidLotId;
    uint64_t east_id = kInvalidLotId;
    bool order_changed = false;

    for (size_t start = 0; start < rhombus.size(); ++start) {
        const LotSubdivision sub = subdivide_block(make_block(rotated(rhombus, start)), params);
        CHECK_EQ(sub.stats.corner_lots, size_t{2});
        CHECK_EQ(sub.lots.size(), size_t{3});
        CHECK_NEAR(total_area(sub), 300.0, 1e-12);
        if (sub.lots.size() != 3) {
            continue;
        }

        const size_t west = corner_lot_at(sub.lots, glm::dvec2{0.0, 0.0});
        const size_t east = corner_lot_at(sub.lots, glm::dvec2{100.0, 0.0});
        CHECK(west < sub.lots.size());
        CHECK(east < sub.lots.size());
        if (west >= sub.lots.size() || east >= sub.lots.size()) {
            continue;
        }

        if (start == 0) {
            west_id = sub.lots[west].id;
            east_id = sub.lots[east].id;
            CHECK(west_id != east_id);
        } else {
            CHECK_EQ(sub.lots[west].id, west_id);
            CHECK_EQ(sub.lots[east].id, east_id);
            order_changed = order_changed || west > east;
        }
    }

    // Without this the test would also pass on an implementation whose emission
    // order never moved, and then it would be asserting nothing about ranking.
    CHECK_TRUE(order_changed);
}

// ============================================================================
// C7: Determinism
// ============================================================================

/**
 * @brief The same block and seed give byte-identical lots
 *
 * Bit equality, not CHECK_NEAR: the claim is that two machines agree, and a
 * tolerance would pass against an implementation whose traversal order had
 * drifted by an ulp per lot. Paired with the next test, which proves the seed is
 * actually consulted -- without it this passes against code that ignores its seed
 * entirely and is therefore trivially reproducible.
 */
TEST(Lots, subdivision_is_deterministic) {
    LotParams params = base_params();
    params.lot_area_max = 900.0;
    params.lot_area_min = 100.0;
    params.lot_width_min = 4.0;
    params.irregularity = 0.6;
    params.force_street_access = 0.5;
    params.corner_angle_max_deg = 40.0;

    const Block block = make_block(u_block());
    const LotSubdivision first = subdivide_block(block, params);
    const LotSubdivision second = subdivide_block(block, params);

    CHECK(first.lots.size() > 3);
    CHECK_EQ(first.lots.size(), second.lots.size());
    if (first.lots.size() != second.lots.size()) {
        return;
    }

    for (size_t i = 0; i < first.lots.size(); ++i) {
        CHECK_EQ(first.lots[i].id, second.lots[i].id);
        CHECK_EQ(first.lots[i].node_key, second.lots[i].node_key);
        CHECK_TRUE(rings_identical(first.lots[i].ring, second.lots[i].ring));
        CHECK_TRUE(first.lots[i].ring_edges == second.lots[i].ring_edges);
        CHECK(first.lots[i].area == second.lots[i].area);
    }
}

/**
 * @brief An irregular subdivision really does depend on its seed
 *
 * The guard on the test above. If irregularity never reached a coordinate, this
 * would fail and the determinism test would be meaningless.
 */
TEST(Lots, seed_changes_an_irregular_subdivision) {
    LotParams params = base_params();
    params.lot_area_max = 900.0;
    params.lot_area_min = 100.0;
    params.lot_width_min = 4.0;
    params.irregularity = 0.6;

    const Block block = make_block(rect(160.0, 120.0));

    LotParams a = params;
    a.seed = 1;
    LotParams b = params;
    b.seed = 2;

    const LotSubdivision first = subdivide_block(block, a);
    const LotSubdivision second = subdivide_block(block, b);

    CHECK(first.lots.size() > 1);

    bool geometry_differs = first.lots.size() != second.lots.size();
    for (size_t i = 0; i < first.lots.size() && i < second.lots.size(); ++i) {
        if (!rings_identical(first.lots[i].ring, second.lots[i].ring)) {
            geometry_differs = true;
        }
    }
    CHECK_TRUE(geometry_differs);

    // Land is conserved whatever the seed did.
    CHECK_NEAR(total_area(first), 19200.0, 1e-6);
    CHECK_NEAR(total_area(second), 19200.0, 1e-6);
}

/**
 * @brief irregularity is a MAGNITUDE, and both of its effects scale with it
 *
 * Every other irregularity test uses one non-zero value, so all of them pass
 * against an implementation that treats the parameter as a boolean. Both
 * documented effects -- how far the pivot slides along the long axis, and how far
 * the cut leans -- are a stated fraction of a stated maximum, and neither was
 * pinned.
 *
 * max_depth 1 makes the whole block one cut at the root node, so both runs draw
 * from the SAME node key and the only thing that differs is the parameter. The
 * draws are therefore common factors, and the two quantities must come out in the
 * exact ratio 0.9 / 0.2 = 4.5. A ratio pins the magnitude in a way that "the two
 * runs differ" cannot: dropping the factor from either expression makes the ratio
 * 1, and so does any rule that saturates.
 *
 * Both are also bounded from above, which is the contract kMaxPivotFraction and
 * kMaxCutSlope state: at irregularity r the pivot stays within 0.25 * r of the
 * long extent from the box centre and the cut leans by at most a slope of 0.3 * r.
 */
TEST(Lots, irregularity_scales_the_cut_rather_than_switching_it_on) {
    LotParams params = base_params();
    params.lot_area_max = 900.0;
    params.lot_area_min = 100.0;
    params.lot_width_min = 4.0;
    params.force_street_access = 0.0;
    params.max_depth = 1;

    // 160 x 120: the long axis is x, so the cut is near-vertical and its pivot
    // slides along x from the box centre at x = 80.
    const Block block = make_block(rect(160.0, 120.0));
    const double long_extent = 160.0;
    const double half = 0.5 * long_extent;

    double slope[2] = {0.0, 0.0};
    double slide[2] = {0.0, 0.0};
    const double levels[2] = {0.2, 0.9};

    for (size_t k = 0; k < 2; ++k) {
        LotParams run = params;
        run.irregularity = levels[k];
        const LotSubdivision sub = subdivide_block(block, run);

        CHECK_EQ(sub.lots.size(), size_t{2});
        CHECK_NEAR(total_area(sub), 19200.0, 1e-9);
        if (sub.lots.size() != 2) {
            return;
        }

        glm::dvec2 from{0.0};
        glm::dvec2 to{0.0};
        CHECK_TRUE(cut_segment(sub.lots[0], from, to));
        // The cut spans the full 120 m height of the block, so dx/dy is its lean
        // and the midpoint's x is the pivot.
        CHECK_NEAR(std::fabs(to.y - from.y), 120.0, 1e-9);
        if (std::fabs(to.y - from.y) < 1.0) {
            return;
        }

        slope[k] = (to.x - from.x) / (to.y - from.y);
        slide[k] = 0.5 * (from.x + to.x) - half;

        // The stated ceilings. Dropping the irregularity factor from either
        // expression breaks the r = 0.2 run here as well as breaking the ratio.
        CHECK(std::fabs(slope[k]) <= 0.3 * levels[k] + 1e-12);
        CHECK(std::fabs(slide[k]) <= 0.25 * levels[k] * long_extent + 1e-9);

        // Both effects have to be live, or the ratios below compare two zeroes.
        CHECK(std::fabs(slope[k]) > 1e-6);
        CHECK(std::fabs(slide[k]) > 1e-6);
    }

    const double ratio = levels[1] / levels[0];
    CHECK_NEAR(slope[1], slope[0] * ratio, 1e-12);
    CHECK_NEAR(slide[1], slide[0] * ratio, 1e-9);
}

/**
 * @brief forceStreetAccess 0.5 lands strictly between 0 and 1
 *
 * The existing test asserts the two ENDS, which any step function reproduces. The
 * parameter is documented as a probability, so a value in the middle has to land
 * in the middle, and halving the range unit_from() draws over -- which makes every
 * setting at or above 0.5 behave exactly like 1.0 -- passes an ends-only test.
 *
 * Counted over a spread of seeds rather than one, because a single block at one
 * seed is a single coin toss and would make this test a lottery.
 */
TEST(Lots, force_street_access_between_the_ends_lands_between_them) {
    LotParams params = base_params();
    params.lot_area_max = 2500.0;
    params.lot_area_min = 200.0;
    params.lot_width_min = 5.0;

    const Block block = make_block(rect(300.0, 300.0));

    size_t landlocked[3] = {0, 0, 0};
    size_t refused[3] = {0, 0, 0};
    const double levels[3] = {0.0, 0.5, 1.0};

    for (size_t k = 0; k < 3; ++k) {
        for (uint64_t seed = 1; seed <= 12; ++seed) {
            LotParams run = params;
            run.force_street_access = levels[k];
            run.seed = seed;
            const LotSubdivision sub = subdivide_block(block, run);

            CHECK_NEAR(total_area(sub), 90000.0, 1e-6);
            refused[k] += sub.stats.rejected_access;
            for (const Lot& lot : sub.lots) {
                if (lot.frontage_length <= 0.0) {
                    ++landlocked[k];
                }
            }
        }
    }

    // The ends, restated so the middle has something to sit between.
    CHECK_EQ(landlocked[2], size_t{0});
    CHECK_EQ(refused[0], size_t{0});
    CHECK(landlocked[0] > 0);
    CHECK(refused[2] > 0);

    // The middle. Both halves matter: an implementation that saturates at 0.5
    // gives landlocked[1] == 0, and one that ignores the parameter above 0 gives
    // landlocked[1] == landlocked[0].
    CHECK(landlocked[1] > 0);
    CHECK(landlocked[1] < landlocked[0]);
    CHECK(refused[1] > 0);
    CHECK(refused[1] < refused[2]);
}

/**
 * @brief lotAreaMin refuses a cut, and the refusal is counted
 *
 * stats.rejected_area is documented as how a caller finds out WHICH constraint
 * stopped a recursion, and nothing asserted it. A 40 x 20 block over a 500 m^2
 * cap has to be cut, but across the long axis it gives two 400 m^2 pieces and
 * along it two more, and lotAreaMin 450 refuses both. The block comes back whole
 * and the counter says why -- once for the primary cut, once for the fallback.
 */
TEST(Lots, lot_area_min_refuses_a_cut_and_says_so) {
    LotParams params = base_params();
    params.lot_area_max = 500.0;
    params.lot_area_min = 450.0;
    params.lot_width_min = 1.0;

    const Block block = make_block(rect(40.0, 20.0));
    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{1});
    CHECK_EQ(sub.stats.splits, size_t{0});
    CHECK_EQ(sub.stats.split_attempts, size_t{1});
    CHECK_EQ(sub.stats.rejected_area, size_t{2});
    CHECK_EQ(sub.stats.rejected_width, size_t{0});
    CHECK_EQ(sub.stats.rejected_degenerate, size_t{0});
    CHECK_NEAR(total_area(sub), 800.0, 1e-9);
    // Over the cap, which is what a refused cut looks like from outside.
    CHECK(sub.lots[0].area > params.lot_area_max);
}

/**
 * @brief With no irregularity the seed moves ids and nothing else
 *
 * The documented contract: at irregularity 0 and forceStreetAccess 0 or 1 nothing
 * consults a draw, so the geometry is a pure function of the block. Asserting the
 * ids DID change in the same breath keeps this from passing against a seed that
 * is wired to nothing at all.
 */
TEST(Lots, seed_moves_ids_but_not_geometry_when_regular) {
    LotParams params = base_params();
    params.lot_area_max = 2500.0;
    params.lot_area_min = 1000.0;
    params.lot_width_min = 10.0;
    params.irregularity = 0.0;
    params.force_street_access = 1.0;

    const Block block = make_block(rect(100.0, 100.0));

    LotParams a = params;
    a.seed = 11;
    LotParams b = params;
    b.seed = 12;

    const LotSubdivision first = subdivide_block(block, a);
    const LotSubdivision second = subdivide_block(block, b);

    CHECK_EQ(first.lots.size(), size_t{4});
    CHECK_EQ(second.lots.size(), first.lots.size());
    if (first.lots.size() != second.lots.size()) {
        return;
    }

    for (size_t i = 0; i < first.lots.size(); ++i) {
        CHECK_TRUE(rings_identical(first.lots[i].ring, second.lots[i].ring));
        CHECK(first.lots[i].id != second.lots[i].id);
    }
}

/**
 * @brief A subtree reproduces on its own, which a shared random stream cannot do
 *
 * THE test for per-lot seeding. The block is subdivided twice: once with
 * max_depth 1, which stops at the root's two children and hands back their rings
 * and their node keys, and once in full. Re-entering the recursion at one child's
 * key must reproduce that child's descendants exactly -- same ids, same
 * coordinates.
 *
 * Under a single shared random stream it could not: by the time the full run
 * reached that child it would have consumed draws for the root and for the other
 * child, so every pivot below would land somewhere else. That is the defect this
 * exists to catch, and there is no cheaper way to observe it from outside.
 *
 * max_depth 3 on this block, NOT the default 16. The ceiling has to BITE, or the
 * test cannot see the second half of the contract: max_depth is measured from the
 * root of the block, so a re-entry has to be told the depth it is re-entering at
 * and be given only the REMAINING budget. Re-entering at depth 0 handed a depth-1
 * child the whole budget again; at the default max_depth this block stops on
 * lot_area_max at depth 6 and the difference is invisible, and at max_depth 3 the
 * re-entry reproduced none of the full run's eight lots at all. Lot::depth is
 * compared for the same reason: it was short by the re-entry offset in every lot
 * even when the ceiling never fired.
 */
TEST(Lots, subtree_reproduces_standalone_from_its_node_key) {
    LotParams params = base_params();
    params.lot_area_max = 900.0;
    params.lot_area_min = 100.0;
    params.lot_width_min = 4.0;
    params.irregularity = 0.6;
    params.force_street_access = 0.0;
    params.max_depth = 3;

    const Block block = make_block(rect(160.0, 120.0));

    LotParams shallow = params;
    shallow.max_depth = 1;
    const LotSubdivision top = subdivide_block(block, shallow);
    CHECK_EQ(top.lots.size(), size_t{2});
    if (top.lots.size() != 2) {
        return;
    }
    for (const Lot& child : top.lots) {
        CHECK_EQ(child.depth, uint32_t{1});
    }

    const LotSubdivision full = subdivide_block(block, params);
    CHECK(full.lots.size() > 4);
    // 19200 m^2 against a 900 m^2 cap needs five levels; the ceiling stops it at
    // three, so every lot emitted is one the ceiling stopped. Without this the
    // re-entry budget is never exercised.
    CHECK_EQ(full.stats.depth_limit_hits, full.lots.size());
    CHECK_EQ(full.stats.max_depth_reached, uint32_t{3});

    std::map<uint64_t, const Lot*> by_id;
    for (const Lot& lot : full.lots) {
        by_id[lot.id] = &lot;
    }
    CHECK_EQ(by_id.size(), full.lots.size());   // ids are unique within a block

    size_t checked = 0;
    for (const Lot& child : top.lots) {
        const LotSubdivision sub = subdivide_ring(child.ring, child.ring_edges, child.node_key,
                                                  params, child.depth);
        CHECK(sub.lots.size() > 1);

        for (const Lot& lot : sub.lots) {
            const auto it = by_id.find(lot.id);
            CHECK(it != by_id.end());
            if (it == by_id.end()) {
                continue;
            }
            CHECK_TRUE(rings_identical(lot.ring, it->second->ring));
            CHECK_TRUE(lot.ring_edges == it->second->ring_edges);
            CHECK_EQ(lot.depth, it->second->depth);
            ++checked;
        }
    }

    CHECK_EQ(checked, full.lots.size());
}

/**
 * @brief Land is conserved through a deep, irregular, non-convex subdivision
 *
 * The safety net under the polygon/line split. Every piece of every cut is either
 * emitted or recursed into, so the areas must sum back to the block. A cut that
 * silently loses a piece -- the exact failure a multi-piece split is prone to --
 * shows up here and nowhere else, because a subdivision with one missing lot
 * looks completely ordinary.
 */
TEST(Lots, land_is_conserved_through_a_deep_irregular_subdivision) {
    LotParams params = base_params();
    params.lot_area_max = 300.0;
    params.lot_area_min = 50.0;
    params.lot_width_min = 3.0;
    params.irregularity = 0.7;
    params.force_street_access = 0.0;
    params.seed = 20260916;

    const Block block = make_block(u_block());
    const LotSubdivision sub = subdivide_block(block, params);

    CHECK(sub.lots.size() > 8);
    CHECK_EQ(sub.stats.dropped_pieces, size_t{0});
    CHECK_NEAR(total_area(sub), 4200.0, 1e-6);

    for (const Lot& lot : sub.lots) {
        CHECK(lot.ring.size() >= 3);
        CHECK_EQ(lot.ring_edges.size(), lot.ring.size());
        // Anticlockwise and with real area, the same contract Block::ring carries.
        CHECK(signed_ring_area(lot.ring) > 0.0);
        CHECK(lot.area > 0.0);
    }
}

// ============================================================================
// C7: Identity
// ============================================================================

/**
 * @brief The block key follows the OSM ways, not the traversal
 *
 * The root of every lot id. Adding the same four ways in the reverse order gives
 * the same face with its half-edges walked in a different order, and the key has
 * to be blind to that or every id in the block moves for no reason. Changing a
 * way id is a genuinely different block and must change it.
 */
TEST(Lots, block_key_follows_way_identity_not_traversal_order) {
    const NodeMap square{{1, {0.0, 0.0}}, {2, {100.0, 0.0}}, {3, {100.0, 100.0}}, {4, {0.0, 100.0}}};
    const std::vector<WayId> ways{10, 11, 12, 13};

    const ParsedOSMData forward = square_ways(square, ways, {0, 1, 2, 3});
    const ParsedOSMData backward = square_ways(square, ways, {3, 2, 1, 0});

    // Same ways, different geometry: a street was dragged.
    const NodeMap moved{{1, {0.0, 0.0}}, {2, {130.0, -4.0}}, {3, {120.0, 100.0}}, {4, {0.0, 100.0}}};
    const ParsedOSMData dragged = square_ways(moved, ways, {0, 1, 2, 3});

    // One way replaced: genuinely a different block.
    const ParsedOSMData renumbered = square_ways(square, {20, 11, 12, 13}, {0, 1, 2, 3});

    const uint64_t a = key_of(forward);
    const uint64_t b = key_of(backward);
    const uint64_t c = key_of(dragged);
    const uint64_t d = key_of(renumbered);

    CHECK(a != 0);
    CHECK_EQ(a, b);
    CHECK_EQ(a, c);
    CHECK(a != d);
}

/**
 * @brief A lot id is a pure function of the block key, the seed and the tree address
 *
 * lot_root_key() and lot_id_for_key() are exposed so that this contract is
 * checkable rather than merely claimed, and nothing called either of them. The
 * unsplit block is the case where the whole chain can be written out in one line:
 * the single lot's node_key IS the root key and its id IS lot_id_for_key() of it.
 *
 * The block key half is the one that matters most. Left out of the root, every
 * block in a city shares a root and two different blocks subdivided the same way
 * hand out the same ids -- and the geometry is identical either way, so nothing
 * downstream would look wrong until two blocks' attributes started overwriting
 * each other.
 */
TEST(Lots, a_lot_id_is_a_pure_function_of_its_tree_address) {
    LotParams params = base_params();
    params.lot_area_max = 1.0e6;   // one lot, so the tree is just its root

    const Block block = make_block(rect(100.0, 100.0));
    const LotSubdivision sub = subdivide_block(block, params);

    CHECK_EQ(sub.lots.size(), size_t{1});
    if (sub.lots.size() != 1) {
        return;
    }
    CHECK_EQ(sub.lots[0].node_key, lot_root_key(params));
    CHECK_EQ(sub.lots[0].id, lot_id_for_key(lot_root_key(params)));
    CHECK(lot_id_for_key(lot_root_key(params)) != kInvalidLotId);

    // Both halves of the root key are live, and they are not interchangeable.
    LotParams other_block = params;
    other_block.block_key = 0xFEDCBA9876543210ull;
    LotParams other_seed = params;
    other_seed.seed = params.seed + 1;

    CHECK(lot_root_key(other_block) != lot_root_key(params));
    CHECK(lot_root_key(other_seed) != lot_root_key(params));
    CHECK(lot_root_key(other_block) != lot_root_key(other_seed));
}

/**
 * @brief The block key reaches every lot id, and changes none of the geometry
 *
 * Identity mechanism number one, and until now it had no assertion on it at all:
 * base_params() sets block_key once and no test varied it, so lot_root_key()
 * could ignore the field entirely and the whole suite stayed green.
 *
 * Exactly the shape of seed_moves_ids_but_not_geometry_when_regular, because the
 * claim has the same two halves: the rings must be bit-identical, since the key
 * is not a coordinate, and every id must differ, since the key is the root of
 * every id.
 */
TEST(Lots, block_key_moves_every_id_and_no_geometry) {
    LotParams params = base_params();
    params.lot_area_max = 2500.0;
    params.lot_area_min = 1000.0;
    params.lot_width_min = 10.0;

    const Block block = make_block(rect(100.0, 100.0));

    LotParams a = params;
    a.block_key = 0x0123456789ABCDEFull;
    LotParams b = params;
    b.block_key = 0xFEDCBA9876543210ull;

    const LotSubdivision first = subdivide_block(block, a);
    const LotSubdivision second = subdivide_block(block, b);

    CHECK_EQ(first.lots.size(), size_t{4});
    CHECK_EQ(second.lots.size(), first.lots.size());
    if (first.lots.size() != second.lots.size()) {
        return;
    }

    for (size_t i = 0; i < first.lots.size(); ++i) {
        CHECK_TRUE(rings_identical(first.lots[i].ring, second.lots[i].ring));
        CHECK(first.lots[i].id != second.lots[i].id);
        CHECK(first.lots[i].node_key != second.lots[i].node_key);
    }

    // Not merely different lot by lot: no id of one block appears anywhere in the
    // other, which is the property that stops two blocks colliding.
    std::set<uint64_t> ids;
    for (const Lot& lot : first.lots) {
        ids.insert(lot.id);
    }
    for (const Lot& lot : second.lots) {
        CHECK(ids.count(lot.id) == 0);
    }
}

/**
 * @brief block_key_from_ways() is what the ids are actually built on
 *
 * The two halves of C7's first mechanism were tested separately and never joined
 * up: one test checked the key function in isolation, and every subdivision test
 * used a hand-written constant. This is the wiring -- the key comes out of the
 * graph and goes into LotParams::block_key, and the ids follow it.
 *
 * Reversing the traversal must leave every id where it was; replacing one
 * bounding way must move all of them, because that is a different block.
 */
TEST(Lots, block_key_from_ways_reaches_every_lot_id) {
    const NodeMap square{{1, {0.0, 0.0}}, {2, {100.0, 0.0}}, {3, {100.0, 100.0}}, {4, {0.0, 100.0}}};

    const uint64_t forward = key_of(square_ways(square, {10, 11, 12, 13}, {0, 1, 2, 3}));
    const uint64_t backward = key_of(square_ways(square, {10, 11, 12, 13}, {3, 2, 1, 0}));
    const uint64_t renumbered = key_of(square_ways(square, {20, 11, 12, 13}, {0, 1, 2, 3}));
    CHECK(forward != 0);

    LotParams params = base_params();
    params.lot_area_max = 2500.0;
    params.lot_area_min = 1000.0;
    params.lot_width_min = 10.0;

    const Block block = make_block(rect(100.0, 100.0));

    const auto ids_for = [&](uint64_t key) {
        LotParams run = params;
        run.block_key = key;
        const LotSubdivision sub = subdivide_block(block, run);
        std::vector<uint64_t> ids;
        for (const Lot& lot : sub.lots) {
            ids.push_back(lot.id);
        }
        return ids;
    };

    const std::vector<uint64_t> from_forward = ids_for(forward);
    const std::vector<uint64_t> from_backward = ids_for(backward);
    const std::vector<uint64_t> from_renumbered = ids_for(renumbered);

    CHECK_EQ(from_forward.size(), size_t{4});
    CHECK_TRUE(from_forward == from_backward);

    CHECK_EQ(from_renumbered.size(), from_forward.size());
    if (from_renumbered.size() != from_forward.size()) {
        return;
    }
    for (size_t i = 0; i < from_forward.size(); ++i) {
        CHECK(from_forward[i] != from_renumbered[i]);
    }
}

/**
 * @brief Moving a street leaves every lot id where it was
 *
 * The case that happens on every drag, and the one the whole identity scheme is
 * built for. The block is deliberately not square -- a square's minimum-area box
 * is a four-way tie, and any nudge resolves it, which would flip the first cut
 * and make this test prove nothing about small deformations.
 *
 * The final check is the one that matters: the ids are not merely the same
 * VALUES, they are on the same land, within a few metres of where they were. An
 * implementation that handed out ids in emission order would pass the first half
 * and fail this.
 */
TEST(Lots, lot_ids_survive_a_moved_street) {
    LotParams params = base_params();
    params.lot_area_max = 4000.0;
    params.lot_area_min = 500.0;
    params.lot_width_min = 10.0;

    const Block before = make_block(rect(140.0, 100.0));
    const Block after =
        make_block(Ring{{0.0, 0.0}, {140.0, 0.0}, {144.0, 97.0}, {0.0, 100.0}});

    const LotSubdivision first = subdivide_block(before, params);
    const LotSubdivision second = subdivide_block(after, params);

    CHECK_EQ(first.lots.size(), size_t{4});
    CHECK_EQ(second.lots.size(), first.lots.size());
    if (first.lots.size() != second.lots.size()) {
        return;
    }

    bool geometry_moved = false;
    for (size_t i = 0; i < first.lots.size(); ++i) {
        CHECK_EQ(first.lots[i].id, second.lots[i].id);
        if (!rings_identical(first.lots[i].ring, second.lots[i].ring)) {
            geometry_moved = true;
        }
        const glm::dvec2 drift =
            centroid_of(second.lots[i].ring) - centroid_of(first.lots[i].ring);
        CHECK(std::sqrt(drift.x * drift.x + drift.y * drift.y) < 5.0);
    }

    // Without this the test would also pass on a block that never changed.
    CHECK_TRUE(geometry_moved);
}

/**
 * @brief Mean-value coordinates sum to one and carry an affine map exactly
 *
 * The transfer rests on linear precision: if the coordinates did not reproduce an
 * affine deformation exactly, a block that was merely scaled would already
 * shuffle its lots. Checked on the U as well as the square, because a non-convex
 * ring is where a naive barycentric scheme stops being defined at all.
 */
TEST(Lots, mean_value_coordinates_reproduce_an_affine_map) {
    const Ring square = rect(100.0, 100.0);
    const glm::dvec2 point{25.0, 60.0};

    const std::vector<double> weights = mean_value_coordinates(square, point);
    CHECK_EQ(weights.size(), square.size());

    double sum = 0.0;
    for (const double w : weights) {
        sum += w;
    }
    CHECK_NEAR(sum, 1.0, 1e-12);

    CHECK_TRUE(points_equal(map_point_between_rings(square, square, point), point, 1e-9));

    // Scale, shear and translate. Affine, so the mapping must be exact.
    const auto affine = [](const glm::dvec2& p) {
        return glm::dvec2{2.0 * p.x + 0.5 * p.y + 17.0, 3.0 * p.y - 4.0};
    };

    Ring deformed;
    for (const glm::dvec2& p : square) {
        deformed.push_back(affine(p));
    }
    CHECK_TRUE(points_equal(map_point_between_rings(square, deformed, point), affine(point), 1e-9));

    const Ring u = u_block();
    const glm::dvec2 inside{30.0, 15.0};
    const std::vector<double> u_weights = mean_value_coordinates(u, inside);
    CHECK_EQ(u_weights.size(), u.size());
    double u_sum = 0.0;
    for (const double w : u_weights) {
        u_sum += w;
    }
    CHECK_NEAR(u_sum, 1.0, 1e-12);

    Ring u_deformed;
    for (const glm::dvec2& p : u) {
        u_deformed.push_back(affine(p));
    }
    CHECK_TRUE(points_equal(map_point_between_rings(u, u_deformed, inside), affine(inside), 1e-9));

    // Rings of different lengths have no correspondence, so the point comes back
    // untouched rather than interpolated against a made-up one.
    const Ring triangle{{0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0}};
    CHECK_TRUE(points_equal(map_point_between_rings(square, triangle, point), point, 1e-12));
}

/**
 * @brief Lots inherit their predecessors' ids through a deformed block
 *
 * The third identity mechanism, for when the provisional ids no longer line up.
 * The new block is the old one at twice the size and subdivided under a different
 * seed, so every provisional id differs -- asserted first, or the test would pass
 * against a transfer that did nothing -- and the transfer has to put the four old
 * ids back on the four corresponding quadrants.
 */
TEST(Lots, transfer_carries_ids_through_a_deformed_block) {
    LotParams old_params = base_params();
    old_params.lot_area_max = 4000.0;
    old_params.lot_area_min = 500.0;
    old_params.lot_width_min = 10.0;
    old_params.seed = 1;

    LotParams new_params = old_params;
    new_params.lot_area_max = 16000.0;   // four times the area, four times the cap
    new_params.lot_area_min = 2000.0;
    new_params.lot_width_min = 20.0;
    new_params.seed = 2;

    const Ring old_ring = rect(140.0, 100.0);
    Ring new_ring;
    for (const glm::dvec2& p : old_ring) {
        new_ring.push_back(p * 2.0);
    }

    const LotSubdivision previous = subdivide_block(make_block(old_ring), old_params);
    LotSubdivision current = subdivide_block(make_block(new_ring), new_params);

    CHECK_EQ(previous.lots.size(), size_t{4});
    CHECK_EQ(current.lots.size(), size_t{4});
    if (previous.lots.size() != 4 || current.lots.size() != 4) {
        return;
    }

    std::set<uint64_t> old_ids;
    for (const Lot& lot : previous.lots) {
        old_ids.insert(lot.id);
    }
    for (const Lot& lot : current.lots) {
        CHECK(old_ids.count(lot.id) == 0);   // nothing is inherited yet
    }

    const size_t matched =
        transfer_lot_ids(previous.lots, old_ring, new_ring, current.lots);
    CHECK_EQ(matched, size_t{4});

    // Every new lot now carries the id of the old lot covering the same land --
    // matched by position, so an off-by-one in the assignment is visible.
    for (const Lot& lot : current.lots) {
        const glm::dvec2 halved = centroid_of(lot.ring) * 0.5;
        const size_t twin = lot_at(previous.lots, halved, 1e-6);
        CHECK(twin < previous.lots.size());
        if (twin < previous.lots.size()) {
            CHECK_EQ(lot.id, previous.lots[twin].id);
        }
    }
}

/**
 * @brief New land keeps its own id rather than stealing an old one
 *
 * Eight lots replacing four. The transfer is one-to-one, so exactly four inherit
 * and the other four keep the id the subdivision gave them. A transfer that
 * assigned the nearest old id to everything would leave duplicates, and two lots
 * sharing an id is worse than a lot whose id changed: every attribute written to
 * one would appear on the other.
 */
TEST(Lots, transfer_leaves_new_land_with_its_own_id) {
    LotParams coarse = base_params();
    coarse.lot_area_max = 4000.0;
    coarse.lot_area_min = 500.0;
    coarse.lot_width_min = 10.0;

    LotParams fine = coarse;
    fine.lot_area_max = 1800.0;

    const Ring ring = rect(140.0, 100.0);
    const Block block = make_block(ring);

    const LotSubdivision previous = subdivide_block(block, coarse);
    LotSubdivision current = subdivide_block(block, fine);

    CHECK_EQ(previous.lots.size(), size_t{4});
    CHECK_EQ(current.lots.size(), size_t{8});
    if (previous.lots.size() != 4 || current.lots.size() != 8) {
        return;
    }

    const size_t matched = transfer_lot_ids(previous.lots, ring, ring, current.lots);
    CHECK_EQ(matched, size_t{4});

    std::set<uint64_t> ids;
    for (const Lot& lot : current.lots) {
        ids.insert(lot.id);
    }
    CHECK_EQ(ids.size(), size_t{8});

    size_t inherited = 0;
    for (const Lot& lot : current.lots) {
        for (const Lot& old : previous.lots) {
            if (lot.id == old.id) {
                ++inherited;
            }
        }
    }
    CHECK_EQ(inherited, size_t{4});
}

/**
 * @brief The three degenerate limits of the mean-value formula
 *
 * All three are 0/0 in the raw expression and all three have to be taken by hand.
 * The third one -- collinear with an edge but OFF its span -- was missing, and it
 * is not exotic: it cannot happen on a convex ring, but on any ring with a reflex
 * corner it happens for every interior point that lines up with a far edge, and a
 * block with a reflex corner is an ordinary block.
 *
 * (50,30) on the U is such a point: it is level with the notch floor from
 * (40,30) to (20,30) but 10 m to the right of it. The formula used to return a
 * vector of NaN there, which then flowed through map_point_between_rings() into
 * transfer_lot_ids() and defeated its distance gate, because a NaN distance is
 * not greater than the tolerance. Linear precision is the assertion, because it
 * is the property the header states and the one a NaN cannot fake.
 */
TEST(Lots, mean_value_coordinates_take_all_three_degenerate_limits) {
    const Ring square = rect(100.0, 100.0);

    // 1. On a vertex: one-hot, so the point maps to that vertex under any ring.
    const std::vector<double> on_vertex = mean_value_coordinates(square, square[2]);
    CHECK_EQ(on_vertex.size(), square.size());
    if (on_vertex.size() == square.size()) {
        CHECK_NEAR(on_vertex[2], 1.0, 1e-15);
        CHECK_NEAR(on_vertex[0] + on_vertex[1] + on_vertex[3], 0.0, 1e-15);
    }

    // 2. On an edge: the linear interpolation of that edge's two ends. (30,0) is
    // three tenths along the bottom edge, so the ends weigh 0.7 and 0.3.
    const std::vector<double> on_edge = mean_value_coordinates(square, glm::dvec2{30.0, 0.0});
    CHECK_EQ(on_edge.size(), square.size());
    if (on_edge.size() == square.size()) {
        CHECK_NEAR(on_edge[0], 0.7, 1e-15);
        CHECK_NEAR(on_edge[1], 0.3, 1e-15);
        CHECK_NEAR(on_edge[2], 0.0, 1e-15);
        CHECK_NEAR(on_edge[3], 0.0, 1e-15);
    }

    // 3. Collinear with an edge but outside its span. The edge subtends no angle
    // at the point, so it contributes nothing -- it is NOT the on-edge limit.
    const Ring u = u_block();
    const glm::dvec2 collinear{50.0, 30.0};
    const std::vector<double> off_span = mean_value_coordinates(u, collinear);
    CHECK_EQ(off_span.size(), u.size());
    if (off_span.size() != u.size()) {
        return;
    }

    double sum = 0.0;
    glm::dvec2 rebuilt{0.0};
    for (size_t i = 0; i < u.size(); ++i) {
        CHECK_TRUE(std::isfinite(off_span[i]));
        sum += off_span[i];
        rebuilt += u[i] * off_span[i];
    }
    CHECK_NEAR(sum, 1.0, 1e-12);
    // sum(w[i] * ring[i]) == point, the header's own statement of the contract.
    CHECK_TRUE(points_equal(rebuilt, collinear, 1e-9));

    // And it still carries an affine deformation exactly, which is the property
    // the whole transfer rests on.
    const auto affine = [](const glm::dvec2& p) {
        return glm::dvec2{2.0 * p.x + 0.5 * p.y + 17.0, 3.0 * p.y - 4.0};
    };
    Ring deformed;
    for (const glm::dvec2& p : u) {
        deformed.push_back(affine(p));
    }
    CHECK_TRUE(points_equal(map_point_between_rings(u, deformed, collinear), affine(collinear), 1e-9));
    CHECK_TRUE(points_equal(map_point_between_rings(u, u, collinear), collinear, 1e-9));
}

/**
 * @brief A lot too far from anything keeps its own id
 *
 * LotMatchConfig::max_centroid_drift is the only tunable of the only public
 * transfer config, and it appeared nowhere in this file. Both existing transfer
 * tests have as many old lots as new ones or fewer, so a one-to-one pairing
 * satisfies them on pigeonhole alone and the gate could be deleted outright.
 *
 * One old lot and one new one, so pigeonhole says nothing at all: whether the
 * pair is made is entirely the gate's decision. The tolerance is pinned from both
 * sides -- a lot just inside it inherits, the same lot just outside does not, and
 * widening the config makes the far one match again -- so neither "always match"
 * nor "never match" survives.
 */
TEST(Lots, transfer_refuses_a_lot_beyond_max_centroid_drift) {
    const Ring ring = rect(100.0, 100.0);

    const std::vector<Lot> previous{lot_with(0xA1u, square_at(glm::dvec2{20.0, 20.0}, 10.0))};

    // 40 m east of the old centroid, against a default tolerance of 30 m.
    const glm::dvec2 far_centre{60.0, 20.0};
    const glm::dvec2 near_centre{40.0, 20.0};

    std::vector<Lot> too_far{lot_with(0xC1u, square_at(far_centre, 10.0))};
    CHECK_EQ(transfer_lot_ids(previous, ring, ring, too_far), size_t{0});
    CHECK_EQ(too_far[0].id, uint64_t{0xC1u});

    std::vector<Lot> close_enough{lot_with(0xC1u, square_at(near_centre, 10.0))};
    CHECK_EQ(transfer_lot_ids(previous, ring, ring, close_enough), size_t{1});
    CHECK_EQ(close_enough[0].id, uint64_t{0xA1u});

    // The same far lot under a wider tolerance, which is what makes the first
    // case a statement about the parameter rather than about the geometry.
    LotMatchConfig wide;
    wide.max_centroid_drift = 45.0;
    std::vector<Lot> reachable{lot_with(0xC1u, square_at(far_centre, 10.0))};
    CHECK_EQ(transfer_lot_ids(previous, ring, ring, reachable, wide), size_t{1});
    CHECK_EQ(reachable[0].id, uint64_t{0xA1u});
}

/**
 * @brief Containment beats a nearer centroid, and beats the drift gate too
 *
 * The documented rule -- "a point inside an old lot is matched to it whatever the
 * distance" -- had no test that could see it. The existing deformation test is a
 * pure 2x scale, so every mapped centroid lands exactly on its twin and plain
 * distance already ranks correctly; replacing the containment test with `false`
 * leaves it green.
 *
 * Here the two criteria disagree on purpose. The new lot sits deep inside a large
 * old lot, 39.6 m from its centroid, while a small old lot's centroid is only 6 m
 * away. Distance alone picks the small one. Containment has to pick the large
 * one, and it has to do so although 39.6 m is past the default 30 m gate.
 */
TEST(Lots, transfer_prefers_containment_over_a_nearer_centroid) {
    const Ring ring = rect(100.0, 100.0);

    const Ring large{{0.0, 0.0}, {60.0, 0.0}, {60.0, 60.0}, {0.0, 60.0}};   // centroid (30,30)
    const Ring small = square_at(glm::dvec2{64.0, 2.0}, 4.0);               // centroid (64,2)
    const std::vector<Lot> previous{lot_with(0xA1u, large), lot_with(0xA2u, small)};

    const glm::dvec2 centre{58.0, 2.0};
    std::vector<Lot> current{lot_with(0xC1u, square_at(centre, 4.0))};

    // Stated rather than assumed: the two criteria really do disagree here.
    const double to_large = std::sqrt(28.0 * 28.0 + 28.0 * 28.0);
    const double to_small = 6.0;
    CHECK(to_small < to_large);
    CHECK(to_large > LotMatchConfig{}.max_centroid_drift);

    CHECK_EQ(transfer_lot_ids(previous, ring, ring, current), size_t{1});
    CHECK_EQ(current[0].id, uint64_t{0xA1u});
}

/**
 * @brief A block with a reflex corner does not hand out ids it has no claim to
 *
 * The end-to-end form of the mean-value NaN. The new lot's centroid at (50,30) is
 * collinear with the U's notch floor, so mapping it back through the block used
 * to produce a NaN point. Every gate in the matcher is a `>` against a tolerance,
 * and NaN is not greater than anything, so the candidate survived with a NaN cost
 * and the lot inherited the id of an old lot 44 m away -- with no counter moving,
 * no warning, and a NaN cost sitting in a std::sort comparator that then no
 * longer defines a strict weak ordering.
 *
 * The control is the point of the test: a lot one ten-millionth of a metre off
 * the same line was always refused correctly, so nothing but the collinear case
 * would ever have shown it.
 */
TEST(Lots, transfer_refuses_a_far_lot_whose_centroid_lies_on_a_ring_edge_line) {
    const Ring u = u_block();

    // Old lot near the bottom-left of the U, 44.7 m from the new lot's centroid.
    const std::vector<Lot> previous{lot_with(0x8888u, square_at(glm::dvec2{10.0, 10.0}, 10.0))};

    const glm::dvec2 collinear{50.0, 30.0};
    std::vector<Lot> current{lot_with(0x2001u, square_at(collinear, 10.0))};
    CHECK_EQ(transfer_lot_ids(previous, u, u, current), size_t{0});
    CHECK_EQ(current[0].id, uint64_t{0x2001u});

    // The control, which behaved correctly all along.
    std::vector<Lot> nudged{
        lot_with(0x2001u, square_at(collinear + glm::dvec2{0.0, 1.0e-7}, 10.0))};
    CHECK_EQ(transfer_lot_ids(previous, u, u, nudged), size_t{0});
    CHECK_EQ(nudged[0].id, uint64_t{0x2001u});

    // And the matcher still works on that ring: a lot that IS close inherits.
    std::vector<Lot> nearby{lot_with(0x2001u, square_at(glm::dvec2{12.0, 12.0}, 6.0))};
    CHECK_EQ(transfer_lot_ids(previous, u, u, nearby), size_t{1});
    CHECK_EQ(nearby[0].id, uint64_t{0x8888u});
}

/**
 * @brief A non-finite ring vertex refuses every match instead of poisoning the sort
 *
 * The guard behind the one above. A mapped point is a weighted sum of the OLD
 * ring's vertices, so one non-finite vertex there makes every mapped point
 * non-finite however good the weights are -- wgs84_to_local() on an unset origin
 * and a ring that reached the matcher before its block was validated both get
 * there.
 *
 * Every gate in the matcher is a `>` against a tolerance and nothing is greater
 * than NaN, so without the guard every candidate survives with a NaN cost, every
 * lot inherits an id at random, and the comparator that ranks those costs stops
 * being a strict weak ordering -- which is undefined behaviour in std::sort, not
 * a wrong answer. Refusing the match is the honest outcome: the lots keep their
 * provisional ids, which is what an unmatchable lot is supposed to get.
 */
TEST(Lots, transfer_refuses_every_match_when_a_ring_vertex_is_not_finite) {
    const Ring current_ring = rect(100.0, 100.0);

    const std::vector<Lot> previous{lot_with(0xA1u, square_at(glm::dvec2{20.0, 20.0}, 10.0)),
                                    lot_with(0xA2u, square_at(glm::dvec2{80.0, 80.0}, 10.0))};

    // Two metres away, so the pairing is obvious and only the guard can refuse it.
    const auto fresh = [] {
        return std::vector<Lot>{lot_with(0xC1u, square_at(glm::dvec2{22.0, 22.0}, 10.0)),
                                lot_with(0xC2u, square_at(glm::dvec2{78.0, 78.0}, 10.0))};
    };

    // The control: a sound ring matches both, so the refusals below are the
    // vertex and not the geometry.
    std::vector<Lot> sound = fresh();
    CHECK_EQ(transfer_lot_ids(previous, current_ring, current_ring, sound), size_t{2});

    Ring with_nan = current_ring;
    with_nan[2] = glm::dvec2{std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::quiet_NaN()};
    std::vector<Lot> against_nan = fresh();
    CHECK_EQ(transfer_lot_ids(previous, with_nan, current_ring, against_nan), size_t{0});
    CHECK_EQ(against_nan[0].id, uint64_t{0xC1u});
    CHECK_EQ(against_nan[1].id, uint64_t{0xC2u});

    Ring with_inf = current_ring;
    with_inf[2] = glm::dvec2{std::numeric_limits<double>::infinity(), 0.0};
    std::vector<Lot> against_inf = fresh();
    CHECK_EQ(transfer_lot_ids(previous, with_inf, current_ring, against_inf), size_t{0});
    CHECK_EQ(against_inf[0].id, uint64_t{0xC1u});
    CHECK_EQ(against_inf[1].id, uint64_t{0xC2u});
}

// ============================================================================
// Blocks with holes
// ============================================================================
//
// blocks.cpp grew Block::holes when its cut-edge handling was fixed: a face
// that reaches a loop across a cut edge -- an estate loop hanging off a single
// access road -- has that loop as a hole, and the ground inside it is not part
// of the block.
//
// Subdivision runs on the outer ring alone, so without the discard pass it lays
// lots straight over that hole. These pin the discard, and they also pin the
// LIMIT of it honestly: a lot straddling the boundary is not yet clipped.

namespace {

/// A 100 x 100 block with a 40 x 40 hole in the middle.
Block block_with_central_hole() {
    Block b;
    b.ring = { {0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}, {0.0, 100.0} };
    // Real edge ids, not kNoBlockEdge: force_street_access refuses every cut on
    // a ring with no frontage, so a block tagged kNoBlockEdge all round comes
    // back as exactly one lot however small lot_area_max is.
    b.ring_edges = { 0u, 1u, 2u, 3u };
    b.holes.push_back({ {30.0, 30.0}, {30.0, 70.0}, {70.0, 70.0}, {70.0, 30.0} });
    return b;
}

} // namespace

TEST(Lots, a_block_with_no_holes_discards_nothing) {
    Block b = block_with_central_hole();
    b.holes.clear();

    LotParams params;
    params.lot_area_max = 200.0;
    params.lot_area_min = 50.0;
    params.force_street_access = 0.0;
    const LotSubdivision result = subdivide_block(b, params);

    CHECK_EQ(result.stats.lots_in_holes, size_t{0});
    CHECK((result.lots.size()) > (size_t{1}));
}

TEST(Lots, lots_that_land_inside_a_hole_are_discarded_and_counted) {
    // force_street_access = 0 is what lets an INTERIOR lot form at all. With it
    // at 1 every lot reaches a street edge, so none ends up wholly inside the
    // hole and there is nothing to discard -- which is correct behaviour and
    // exactly why this test has to turn it off to have a subject.
    LotParams params;
    params.lot_area_max = 200.0;
    params.lot_area_min = 50.0;
    params.force_street_access = 0.0;

    const Block b = block_with_central_hole();
    const LotSubdivision with_hole = subdivide_block(b, params);

    Block solid = b;
    solid.holes.clear();
    const LotSubdivision without = subdivide_block(solid, params);

    // The hole costs lots, and the count says how many.
    CHECK((with_hole.stats.lots_in_holes) > (size_t{0}));
    CHECK_EQ(with_hole.lots.size() + with_hole.stats.lots_in_holes, without.lots.size());
    CHECK((with_hole.lots.size()) < (without.lots.size()));
}

TEST(Lots, no_surviving_lot_lies_wholly_inside_the_hole) {
    LotParams params;
    params.lot_area_max = 200.0;
    params.lot_area_min = 50.0;
    params.force_street_access = 0.0;

    const Block b = block_with_central_hole();
    const LotSubdivision result = subdivide_block(b, params);

    CHECK((result.lots.size()) > (size_t{0}));

    // WHOLLY inside, which is the rule the discard actually implements, and not
    // "centroid inside", which was the first rule tried and was wrong. A lot
    // that WRAPS the hole has its centroid in the middle of the hole while being
    // almost entirely real land; discarding on the centroid threw a whole
    // 10000 m2 block away as zero lots.
    //
    // Asserted per lot rather than as a count, so a discard that removed the
    // wrong lots still fails.
    for (const Lot& lot : result.lots) {
        bool every_vertex_in_hole = !lot.ring.empty();
        for (const glm::dvec2& v : lot.ring) {
            const bool in = v.x > 30.0 && v.x < 70.0 && v.y > 30.0 && v.y < 70.0;
            if (!in) {
                every_vertex_in_hole = false;
                break;
            }
        }
        CHECK_FALSE(every_vertex_in_hole);
    }
}

TEST(Lots, a_single_lot_wrapping_the_hole_is_kept_not_discarded) {
    // The case that killed the centroid rule, pinned so it cannot come back.
    // force_street_access at 1 on a ring with frontage keeps the block whole at
    // this size, so the one lot surrounds the hole and its centroid sits in the
    // middle of it.
    LotParams params;
    params.lot_area_max = 100000.0;
    params.lot_area_min = 200.0;

    const Block b = block_with_central_hole();
    const LotSubdivision result = subdivide_block(b, params);

    CHECK_EQ(result.lots.size(), size_t{1});
    CHECK_EQ(result.stats.lots_in_holes, size_t{0});
    CHECK((result.lots.front().area) > (5000.0));
}

// The documented limit, asserted so it cannot quietly change without someone
// noticing. A lot straddling the hole boundary keeps the part of its ring that
// lies over the hole; clipping it properly can split one lot into several or
// give it a hole of its own, and Lot::ring is a single ring.
TEST(Lots, a_lot_straddling_the_hole_boundary_is_kept_whole_for_now) {
    // Large targets, so lots are big enough to straddle the hole boundary.
    LotParams params;
    params.lot_area_max = 800.0;
    params.lot_area_min = 100.0;
    params.force_street_access = 0.0;

    const Block b = block_with_central_hole();
    const LotSubdivision result = subdivide_block(b, params);

    CHECK((result.lots.size()) > (size_t{0}));

    // Total lot area still exceeds the block's net area (10000 - 1600 = 8400),
    // because straddling lots are unclipped. When that stops being true, this
    // test should be REPLACED rather than relaxed -- it is pinning a known gap,
    // not a desirable property.
    double total = 0.0;
    for (const Lot& lot : result.lots) total += lot.area;
    CHECK((total) > (8400.0));
}
