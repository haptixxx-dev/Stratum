/**
 * @file test_road_graph.cpp
 * @brief RoadGraph topology tests against the hand-authored .osm fixtures
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Each test parses one fixture from tests/data with the real OSMParser, builds a
 * RoadGraph from the result, and asserts the topology documented in
 * tests/data/README.md.
 *
 * These tests never assert an absolute local metre coordinate. OSMParser recentres
 * processed geometry on its centre of mass, so a position is only ever compared
 * against the Road::polyline entry carrying the same NodeId.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadGraph
 * @endcode
 */

#include "framework.hpp"

#include "osm/parser.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"

#include <glm/glm.hpp>
#include <algorithm>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <vector>

#ifndef STRATUM_TEST_DATA_DIR
#error "STRATUM_TEST_DATA_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

namespace {

using stratum::osm::NodeId;
using stratum::osm::ParsedOSMData;
using stratum::osm::Road;
using stratum::osm::SideFlags;
using stratum::osm::WayId;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::GraphNode;
using stratum::osm::road::RoadGraph;

/**
 * @brief Absolute path of a fixture in tests/data
 *
 * @param filename Fixture file name, for example "t_junction.osm"
 * @return Absolute path to the fixture
 */
std::filesystem::path fixture_path(const char* filename) {
    return std::filesystem::path(STRATUM_TEST_DATA_DIR) / filename;
}

/// Every fixture in tests/data, in README table order
constexpr const char* kAllFixtures[] = {
    "four_way.osm", "t_junction.osm", "cul_de_sac.osm", "roundabout.osm",
    "motorway_link.osm", "rural_track.osm", "bridge_over.osm",
    "bridge_abutment.osm", "duplicate_node.osm",
};

/**
 * @brief Parse one fixture, reporting a failure when it cannot be read
 *
 * @param filename  Fixture file name in tests/data
 * @param simplify  Douglas-Peucker tolerance in metres, or 0 to leave geometry alone
 * @return Parsed data, or std::nullopt when the parse failed
 */
std::optional<ParsedOSMData> parse_fixture(const char* filename, double simplify = 0.0) {
    const auto path = fixture_path(filename);
    if (!std::filesystem::exists(path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture exists",
                                      "missing: " + path.string());
        return std::nullopt;
    }

    stratum::osm::OSMParser parser;
    stratum::osm::ParserConfig config;
    config.import_buildings = false;
    config.import_water = false;
    config.import_landuse = false;
    config.import_natural = false;
    config.simplify_geometry = simplify > 0.0;
    config.simplify_tolerance = simplify;
    parser.set_config(config);

    if (!parser.parse(path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "parser.parse(fixture)",
                                      path.string() + ": " + parser.get_error());
        return std::nullopt;
    }
    return parser.take_data();
}

/**
 * @brief Build a graph from a fixture in one step
 *
 * @param filename Fixture file name in tests/data
 * @param data     Receives the parsed data, which the graph's positions refer to
 * @param graph    Receives the built graph
 * @param simplify Douglas-Peucker tolerance in metres, or 0 to leave geometry alone
 * @return true when the fixture parsed and produced at least one road
 */
bool build_fixture(const char* filename, ParsedOSMData& data, RoadGraph& graph,
                   double simplify = 0.0) {
    auto parsed = parse_fixture(filename, simplify);
    if (!parsed.has_value()) return false;

    data = std::move(*parsed);
    if (data.roads.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture produced roads",
                                      std::string{filename} + " parsed to zero roads");
        return false;
    }
    graph.build(data);
    return true;
}

/// Render a SideFlags value for a failure message
const char* side_name(SideFlags side) {
    switch (side) {
        case SideFlags::Unknown: return "Unknown";
        case SideFlags::None:    return "None";
        case SideFlags::Left:    return "Left";
        case SideFlags::Right:   return "Right";
        case SideFlags::Both:    return "Both";
    }
    return "<invalid>";
}

/// Count graph nodes of exactly @p degree
size_t count_nodes_of_degree(const RoadGraph& graph, size_t degree) {
    return static_cast<size_t>(std::count_if(
        graph.nodes().begin(), graph.nodes().end(),
        [degree](const GraphNode& node) { return node.degree() == degree; }));
}

/// First graph node of exactly @p degree, or nullptr
const GraphNode* first_node_of_degree(const RoadGraph& graph, size_t degree) {
    for (const auto& node : graph.nodes()) {
        if (node.degree() == degree) return &node;
    }
    return nullptr;
}

/// First graph node with the given OSM node ID, or nullptr
const GraphNode* find_node(const RoadGraph& graph, NodeId osm_id) {
    for (const auto& node : graph.nodes()) {
        if (node.osm_id == osm_id) return &node;
    }
    return nullptr;
}

/// Every edge split out of one OSM way
std::vector<const GraphEdge*> edges_of_way(const RoadGraph& graph, WayId way) {
    std::vector<const GraphEdge*> result;
    for (const auto& edge : graph.edges()) {
        if (edge.source_way == way) result.push_back(&edge);
    }
    return result;
}

/**
 * @brief Local position of an OSM node, read back from the parsed road polylines
 *
 * Robust to OSMParser::recenter_on_features, which shifts all processed geometry.
 *
 * @param data    Parsed data to search
 * @param osm_id  Node to look for
 * @return Position in local metres, or std::nullopt when no road references it
 */
std::optional<glm::dvec2> node_position(const ParsedOSMData& data, NodeId osm_id) {
    for (const auto& road : data.roads) {
        if (road.node_ids.size() != road.polyline.size()) continue;
        for (size_t i = 0; i < road.node_ids.size(); ++i) {
            if (road.node_ids[i] == osm_id) return road.polyline[i];
        }
    }
    return std::nullopt;
}

/**
 * @brief The invariant every edge of every fixture must satisfy
 *
 * polyline and node_ids are parallel arrays, and an edge spans two graph nodes so
 * it can never be shorter than two points.
 *
 * @param graph Graph to check
 * @param label Fixture name, for the failure message
 */
void check_edge_invariants(const RoadGraph& graph, const char* label) {
    for (size_t i = 0; i < graph.edges().size(); ++i) {
        const auto& edge = graph.edges()[i];
        if (edge.polyline.size() != edge.node_ids.size()) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "edge.polyline.size() == edge.node_ids.size()",
                std::string{label} + " edge " + std::to_string(i) + ": polyline " +
                    std::to_string(edge.polyline.size()) + " vs node_ids " +
                    std::to_string(edge.node_ids.size()));
        }
        if (edge.polyline.size() < 2) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "edge.polyline.size() >= 2",
                std::string{label} + " edge " + std::to_string(i) + ": polyline " +
                    std::to_string(edge.polyline.size()));
        }
    }
}

} // namespace

// ============================================================================
// four_way.osm
// ============================================================================

TEST(RoadGraph, four_way_has_exactly_one_degree_4_node) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("four_way.osm", data, graph)) return;

    CHECK_EQ(count_nodes_of_degree(graph, 4), size_t{1});
    CHECK_EQ(graph.stats().junctions, size_t{1});

    const GraphNode* junction = first_node_of_degree(graph, 4);
    if (junction == nullptr) return;

    CHECK_EQ(junction->osm_id, NodeId{100});
    CHECK_EQ(junction->layer, 0);
}

TEST(RoadGraph, four_way_junction_carries_traffic_signals) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("four_way.osm", data, graph)) return;

    const GraphNode* junction = first_node_of_degree(graph, 4);
    if (junction == nullptr) {
        stratum::test::report_failure(__FILE__, __LINE__, "degree-4 node exists",
                                      "four_way.osm produced none");
        return;
    }
    CHECK_TRUE(junction->has_signals);
}

// ============================================================================
// t_junction.osm - the case endpoint clustering misses
// ============================================================================

TEST(RoadGraph, t_junction_on_interior_node_has_degree_3) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("t_junction.osm", data, graph)) return;

    CHECK_EQ(count_nodes_of_degree(graph, 3), size_t{1});
    CHECK_EQ(graph.stats().junctions, size_t{1});

    const GraphNode* junction = first_node_of_degree(graph, 3);
    if (junction == nullptr) return;

    // Node 203 is the middle entry of way 2000's node list, not an endpoint.
    CHECK_EQ(junction->osm_id, NodeId{203});
}

TEST(RoadGraph, t_junction_node_sits_on_the_shared_mid_way_node) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("t_junction.osm", data, graph)) return;

    const GraphNode* junction = first_node_of_degree(graph, 3);
    if (junction == nullptr) {
        stratum::test::report_failure(__FILE__, __LINE__, "degree-3 node exists",
                                      "t_junction.osm produced none");
        return;
    }

    const auto expected = node_position(data, NodeId{203});
    if (!expected.has_value()) {
        stratum::test::report_failure(__FILE__, __LINE__, "node 203 is in a road polyline",
                                      "parser dropped node_ids");
        return;
    }
    CHECK_NEAR(junction->position.x, expected->x, 1e-9);
    CHECK_NEAR(junction->position.y, expected->y, 1e-9);
}

TEST(RoadGraph, t_junction_interior_shape_points_are_not_graph_nodes) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("t_junction.osm", data, graph)) return;

    // 202 and 204 sit inside way 2000; 206 sits inside way 2010. Each is
    // referenced by exactly one way and is not an endpoint.
    CHECK(find_node(graph, NodeId{202}) == nullptr);
    CHECK(find_node(graph, NodeId{204}) == nullptr);
    CHECK(find_node(graph, NodeId{206}) == nullptr);

    // Endpoints 201, 205, 207 are dead ends.
    CHECK_EQ(graph.stats().dead_ends, size_t{3});
}

// ============================================================================
// cul_de_sac.osm
// ============================================================================

TEST(RoadGraph, cul_de_sac_dead_end_is_a_turning_circle) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("cul_de_sac.osm", data, graph)) return;

    size_t turning_circles = 0;
    const GraphNode* found = nullptr;
    for (const auto& node : graph.nodes()) {
        if (node.is_dead_end() && node.is_turning_circle) {
            ++turning_circles;
            if (found == nullptr) found = &node;
        }
    }

    CHECK_EQ(turning_circles, size_t{1});
    if (found == nullptr) return;

    CHECK_EQ(found->osm_id, NodeId{306});
    CHECK_EQ(found->degree(), size_t{1});
}

TEST(RoadGraph, cul_de_sac_plain_dead_ends_are_not_turning_circles) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("cul_de_sac.osm", data, graph)) return;

    for (NodeId osm_id : {NodeId{301}, NodeId{303}}) {
        const GraphNode* node = find_node(graph, osm_id);
        if (node == nullptr) {
            stratum::test::report_failure(__FILE__, __LINE__, "dead-end node exists",
                                          "missing node " + std::to_string(osm_id));
            continue;
        }
        CHECK_TRUE(node->is_dead_end());
        CHECK_FALSE(node->is_turning_circle);
    }

    // Node 302 is where the close leaves the through street.
    const GraphNode* branch = find_node(graph, NodeId{302});
    if (branch != nullptr) {
        CHECK_EQ(branch->degree(), size_t{3});
    }
}

// ============================================================================
// roundabout.osm
// ============================================================================

TEST(RoadGraph, roundabout_splits_into_a_closed_cycle_of_flagged_edges) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("roundabout.osm", data, graph)) return;

    const auto ring_edges = edges_of_way(graph, WayId{4000});
    CHECK_EQ(ring_edges.size(), size_t{3});
    CHECK_EQ(graph.stats().roundabout_edges, size_t{3});

    if (ring_edges.size() != 3) return;

    // Every ring edge is flagged, and no approach edge is.
    for (const auto* edge : ring_edges) {
        CHECK_TRUE(edge->is_roundabout);
    }
    for (const auto& edge : graph.edges()) {
        if (edge.source_way != 4000) {
            CHECK_FALSE(edge.is_roundabout);
        }
    }

    // Closed cycle: each ring node is the `from` of exactly one ring edge and the
    // `to` of exactly one ring edge, over exactly three distinct nodes.
    std::set<uint32_t> from_nodes;
    std::set<uint32_t> to_nodes;
    for (const auto* edge : ring_edges) {
        from_nodes.insert(edge->from);
        to_nodes.insert(edge->to);
    }
    CHECK_EQ(from_nodes.size(), size_t{3});
    CHECK_EQ(to_nodes.size(), size_t{3});
    CHECK(from_nodes == to_nodes);
}

TEST(RoadGraph, roundabout_approach_nodes_have_degree_3) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("roundabout.osm", data, graph)) return;

    for (NodeId osm_id : {NodeId{401}, NodeId{403}, NodeId{405}}) {
        const GraphNode* node = find_node(graph, osm_id);
        if (node == nullptr) {
            stratum::test::report_failure(__FILE__, __LINE__, "approach node exists",
                                          "missing node " + std::to_string(osm_id));
            continue;
        }
        CHECK_EQ(node->degree(), size_t{3});
        CHECK_TRUE(node->is_junction());
    }

    // Ring shape points carry no junction.
    CHECK(find_node(graph, NodeId{402}) == nullptr);
    CHECK(find_node(graph, NodeId{404}) == nullptr);
    CHECK(find_node(graph, NodeId{406}) == nullptr);

    CHECK_EQ(graph.stats().junctions, size_t{3});
}

// ============================================================================
// motorway_link.osm
// ============================================================================

TEST(RoadGraph, motorway_link_ramp_edges_are_flagged) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("motorway_link.osm", data, graph)) return;

    const auto ramp_edges = edges_of_way(graph, WayId{5010});
    CHECK_TRUE(!ramp_edges.empty());
    for (const auto* edge : ramp_edges) {
        CHECK_TRUE(edge->is_link);
    }

    const auto mainline_edges = edges_of_way(graph, WayId{5000});
    CHECK_EQ(mainline_edges.size(), size_t{2});   // split at interior node 503
    for (const auto* edge : mainline_edges) {
        CHECK_FALSE(edge->is_link);
    }
}

TEST(RoadGraph, motorway_link_merge_node_has_degree_3) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("motorway_link.osm", data, graph)) return;

    const GraphNode* merge = find_node(graph, NodeId{503});
    if (merge == nullptr) {
        stratum::test::report_failure(__FILE__, __LINE__, "merge node 503 exists",
                                      "motorway_link.osm produced none");
        return;
    }
    CHECK_EQ(merge->degree(), size_t{3});
    CHECK_EQ(graph.stats().junctions, size_t{1});
}

// ============================================================================
// rural_track.osm
// ============================================================================

TEST(RoadGraph, rural_track_keeps_surface_and_has_no_sidewalk) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("rural_track.osm", data, graph)) return;

    CHECK_EQ(graph.edges().size(), size_t{1});
    if (graph.edges().empty()) return;

    const auto& edge = graph.edges().front();
    CHECK_EQ(edge.surface, std::string{"gravel"});

    // Tag absent, so either the untouched Unknown or an explicit "no sidewalk on a
    // track" class default is correct. Anything else means a sidewalk was invented.
    const bool no_sidewalk =
        edge.sidewalk == SideFlags::Unknown || edge.sidewalk == SideFlags::None;
    if (!no_sidewalk) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "edge.sidewalk is Unknown or None",
            std::string{"actual: "} + side_name(edge.sidewalk));
    }

    CHECK_EQ(graph.stats().junctions, size_t{0});
    CHECK_EQ(graph.stats().dead_ends, size_t{2});
}

// ============================================================================
// bridge_over.osm - a shared node that must NOT become a junction
// ============================================================================

TEST(RoadGraph, bridge_over_splits_the_shared_node_by_layer) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("bridge_over.osm", data, graph)) return;

    std::vector<const GraphNode*> shared;
    for (const auto& node : graph.nodes()) {
        if (node.osm_id == 702) shared.push_back(&node);
    }

    CHECK_EQ(shared.size(), size_t{2});
    if (shared.size() != 2) return;

    // One per layer, and they must disagree.
    CHECK(shared[0]->layer != shared[1]->layer);
    const int low = std::min(shared[0]->layer, shared[1]->layer);
    const int high = std::max(shared[0]->layer, shared[1]->layer);
    CHECK_EQ(low, 0);
    CHECK_EQ(high, 1);

    CHECK_EQ(graph.stats().layer_split_nodes, size_t{1});
}

TEST(RoadGraph, bridge_over_invents_no_junction) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("bridge_over.osm", data, graph)) return;

    for (const auto& node : graph.nodes()) {
        if (node.osm_id != 702) continue;
        CHECK(node.degree() != size_t{4});
        // Each layer sees only its own way passing through: a continuation.
        CHECK_EQ(node.degree(), size_t{2});
        CHECK_FALSE(node.is_junction());
    }

    CHECK_EQ(graph.stats().junctions, size_t{0});
}

// ============================================================================
// bridge_abutment.osm - a layer change that must NOT disconnect the road
// ============================================================================

TEST(RoadGraph, bridge_abutment_does_not_split_the_node_by_layer) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("bridge_abutment.osm", data, graph)) return;

    // Every way ends on 902 and 903; none passes through. The differing layer is
    // the deck rising over its approach, not a grade separation.
    for (const NodeId abutment : {NodeId{902}, NodeId{903}}) {
        size_t matches = 0;
        for (const auto& node : graph.nodes()) {
            if (node.osm_id == abutment) ++matches;
        }
        if (matches != 1) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "abutment yields exactly one GraphNode",
                "node " + std::to_string(abutment) + ": " + std::to_string(matches));
        }
    }

    CHECK_EQ(graph.stats().layer_split_nodes, size_t{0});
    CHECK_EQ(graph.nodes().size(), size_t{4});
}

TEST(RoadGraph, bridge_abutment_deck_stays_joined_to_its_approaches) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("bridge_abutment.osm", data, graph)) return;

    CHECK_EQ(graph.edges().size(), size_t{3});

    // The road is continuous: only its two far ends are dead ends.
    CHECK_EQ(graph.stats().dead_ends, size_t{2});
    CHECK_EQ(graph.stats().continuations, size_t{2});
    CHECK_EQ(graph.stats().junctions, size_t{0});

    for (const NodeId abutment : {NodeId{902}, NodeId{903}}) {
        const GraphNode* node = find_node(graph, abutment);
        if (node == nullptr) {
            stratum::test::report_failure(__FILE__, __LINE__, "abutment node exists",
                                          "node " + std::to_string(abutment));
            continue;
        }
        CHECK_EQ(node->degree(), size_t{2});
        CHECK_FALSE(node->is_dead_end());
    }
}

// ============================================================================
// duplicate_node.osm - two node IDs at one coordinate
// ============================================================================

TEST(RoadGraph, duplicate_node_collapses_to_one_graph_node) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("duplicate_node.osm", data, graph)) return;

    // 802 and 803 sit on the same coordinate, so they are one place, and the
    // zero-length stretch of way 8000 between them is not an edge.
    CHECK_EQ(graph.nodes().size(), size_t{5});
    CHECK_EQ(graph.edges().size(), size_t{4});

    size_t duplicates = 0;
    for (const auto& node : graph.nodes()) {
        if (node.osm_id == 802 || node.osm_id == 803) ++duplicates;
    }
    CHECK_EQ(duplicates, size_t{1});
}

TEST(RoadGraph, duplicate_node_does_not_sever_the_way) {
    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("duplicate_node.osm", data, graph)) return;

    CHECK_EQ(count_nodes_of_degree(graph, 4), size_t{1});
    CHECK_EQ(graph.stats().junctions, size_t{1});
    CHECK_EQ(graph.stats().dead_ends, size_t{4});
    CHECK_EQ(graph.stats().continuations, size_t{0});

    const GraphNode* junction = first_node_of_degree(graph, 4);
    if (junction == nullptr) return;

    // Both halves of way 8000 have to touch that one node, otherwise there is no
    // path from 801 to 804 and the crossroads is two unrelated stubs.
    size_t way_8000_arms = 0;
    for (const auto& arm : junction->arms) {
        if (graph.edge(arm.edge).source_way == 8000) ++way_8000_arms;
    }
    CHECK_EQ(way_8000_arms, size_t{2});
}

// ============================================================================
// Simplification must not delete the nodes the graph is built from
//
// Douglas-Peucker guarantees only that the ends of a span survive. A junction
// node interior to a straight through road deviates 0 m from the chord, so an
// unconstrained pass deletes it and the T-junction becomes a dead end sitting on
// top of an uninterrupted edge. simplify_geometry is off by default, so nothing
// else in this file exercises the path.
// ============================================================================

TEST(RoadGraph, simplification_keeps_the_t_junction) {
    // Way 2000 is perfectly straight, so every interior point is 0 m from the
    // chord and an unprotected simplifier drops all of them, node 203 included.
    constexpr double kTolerance = 5.0;

    ParsedOSMData data;
    RoadGraph graph;
    if (!build_fixture("t_junction.osm", data, graph, kTolerance)) return;

    CHECK_EQ(count_nodes_of_degree(graph, 3), size_t{1});
    CHECK_EQ(graph.stats().junctions, size_t{1});
    CHECK_EQ(graph.stats().dead_ends, size_t{3});
    CHECK_EQ(graph.edges().size(), size_t{3});

    const GraphNode* junction = first_node_of_degree(graph, 3);
    if (junction == nullptr) return;
    CHECK_EQ(junction->osm_id, NodeId{203});
}

TEST(RoadGraph, simplification_still_drops_plain_shape_points) {
    // Guards the guard above: if the simplifier were simply disabled for roads,
    // simplification_keeps_the_t_junction would pass for the wrong reason.
    constexpr double kTolerance = 5.0;

    auto plain = parse_fixture("t_junction.osm");
    auto simplified = parse_fixture("t_junction.osm", kTolerance);
    if (!plain.has_value() || !simplified.has_value()) return;

    const auto count_points = [](const ParsedOSMData& d) {
        size_t total = 0;
        for (const auto& road : d.roads) total += road.polyline.size();
        return total;
    };
    CHECK_TRUE(count_points(*simplified) < count_points(*plain));

    // 202 and 204 are shape points of way 2000 alone: no graph node derives from
    // them, so they are exactly what the simplifier is allowed to remove.
    std::set<NodeId> surviving;
    for (const auto& road : simplified->roads) {
        for (const NodeId id : road.node_ids) surviving.insert(id);
    }
    CHECK(surviving.count(202) == size_t{0});
    CHECK(surviving.count(204) == size_t{0});
    CHECK(surviving.count(203) == size_t{1});
}

// ============================================================================
// Universal invariants across every fixture
// ============================================================================

TEST(RoadGraph, every_fixture_has_parallel_polyline_and_node_ids) {
    for (const char* filename : kAllFixtures) {
        ParsedOSMData data;
        RoadGraph graph;
        if (!build_fixture(filename, data, graph)) continue;

        if (graph.edges().empty()) {
            stratum::test::report_failure(__FILE__, __LINE__, "graph has edges",
                                          std::string{filename} + " produced none");
            continue;
        }
        check_edge_invariants(graph, filename);
    }
}

TEST(RoadGraph, every_fixture_reports_consistent_stats) {
    for (const char* filename : kAllFixtures) {
        ParsedOSMData data;
        RoadGraph graph;
        if (!build_fixture(filename, data, graph)) continue;

        const auto stats = graph.stats();
        CHECK_EQ(stats.nodes, graph.nodes().size());
        CHECK_EQ(stats.edges, graph.edges().size());

        // Every node is a dead end, a continuation, or a junction.
        CHECK_EQ(stats.dead_ends + stats.continuations + stats.junctions +
                     count_nodes_of_degree(graph, 0),
                 graph.nodes().size());

        // Each edge contributes exactly two arms, counting a loop edge twice.
        size_t arm_total = 0;
        for (const auto& node : graph.nodes()) arm_total += node.degree();
        CHECK_EQ(arm_total, graph.edges().size() * 2);
    }
}

// ============================================================================
// Parser invariant: Road::node_ids stays parallel to Road::polyline
//
// The graph silently skips a road whose two vectors disagree, so a parser
// regression here would not fail a topology assertion. It would just quietly
// empty the network. The invariant is therefore asserted on Road directly, not
// only on GraphEdge.
// ============================================================================

/**
 * @brief Assert the parallel-array invariant over every road of one parse
 *
 * @param data  Parsed data to check
 * @param label Fixture name plus parse mode, for the failure message
 * @return Total polyline points across every road
 */
namespace {

size_t check_road_invariants(const ParsedOSMData& data, const std::string& label) {
    size_t points = 0;
    for (size_t i = 0; i < data.roads.size(); ++i) {
        const Road& road = data.roads[i];
        points += road.polyline.size();

        if (road.node_ids.size() != road.polyline.size()) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "road.node_ids.size() == road.polyline.size()",
                label + " road " + std::to_string(i) + " (way " +
                    std::to_string(road.osm_id) + "): polyline " +
                    std::to_string(road.polyline.size()) + " vs node_ids " +
                    std::to_string(road.node_ids.size()));
        }
        if (road.polyline.size() < 2) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "road.polyline.size() >= 2",
                label + " road " + std::to_string(i) + ": polyline " +
                    std::to_string(road.polyline.size()));
        }
    }
    return points;
}

} // namespace

TEST(RoadGraph, parser_keeps_node_ids_parallel_on_every_fixture) {
    for (const char* filename : kAllFixtures) {
        auto data = parse_fixture(filename);
        if (!data.has_value()) continue;

        CHECK_TRUE(!data->roads.empty());
        check_road_invariants(*data, filename);
    }
}

TEST(RoadGraph, parser_keeps_node_ids_parallel_under_simplification) {
    // Douglas-Peucker is the one parser path that can desynchronise the two
    // vectors, and ParserConfig::simplify_geometry is off by default, so every
    // other test in this file leaves it unexercised. The tolerance is large
    // relative to the fixtures so the filter certainly fires.
    constexpr double kTolerance = 5.0;

    size_t plain_points = 0;
    size_t simplified_points = 0;

    for (const char* filename : kAllFixtures) {
        auto plain = parse_fixture(filename);
        auto simplified = parse_fixture(filename, kTolerance);
        if (!plain.has_value() || !simplified.has_value()) continue;

        plain_points += check_road_invariants(*plain, filename);
        simplified_points +=
            check_road_invariants(*simplified, std::string{filename} + " (simplified)");

        // Simplification drops points; it must never drop or add a road.
        CHECK_EQ(simplified->roads.size(), plain->roads.size());
    }

    // Guards the guard: if the filter never fired, the test above proved nothing.
    CHECK_TRUE(simplified_points < plain_points);
}
