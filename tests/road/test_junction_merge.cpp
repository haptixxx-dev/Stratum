/**
 * @file test_junction_merge.cpp
 * @brief What a near-coincident junction cluster owes the consumers keyed by node
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * collect_arms() merges two junction nodes closer together than one junction is
 * wide: the cluster's PRIMARY takes every real approach and every other member is
 * given no arms at all, so the ground is covered by one junction rather than by
 * two overlapping ones. That part was always right, and junction_trim.hpp
 * documents it.
 *
 * What was missing is that the merge was invisible from outside collect_arms().
 * Several consumers key off a node id rather than off a junction:
 *
 * - `RoadNetworkBuilder` gates build_approach_markings() on
 *   `node_has_junction[edge.from]` / `[edge.to]`, so an approach to an absorbed
 *   member got no stop line and no give-way bar -- while the identical approach
 *   to the primary, cut back by the very same solve, got both.
 * - dropped_kerb_spans() filters crossings on `Crossing::node`, which names the
 *   arm's OWN end node, so a junction crossing on an absorbed member's approach
 *   demanded a kerb drop from a ring that does not exist. The corridor still laid
 *   its own drop right up to the trim station, so the two kerbs met at the arm
 *   mouth with the full curb height between them -- the exact step
 *   corridor_kerb_drops() was added to remove.
 *
 * Both were silent. The fix reports the cluster: collect_arms() fills an optional
 * out-parameter on every path, JunctionBuilder publishes the node-to-primary map
 * as junction_owner(), and the builder resolves through it.
 *
 * The fixture is the one the defect was found on: two junction nodes 0.4 m apart
 * joined by a stub, each with two real approaches.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests JunctionMerge
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/crossings.hpp"
#include "osm/road/junction_builder.hpp"
#include "osm/road/junction_trim.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_network_builder.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::osm::NodeId;
using stratum::osm::ParsedOSMData;
using stratum::osm::Road;
using stratum::osm::RoadType;
using stratum::osm::WayId;
using stratum::osm::road::ArmRef;
using stratum::osm::road::Centerline;
using stratum::osm::road::Crossing;
using stratum::osm::road::CrossingConfig;
using stratum::osm::road::DroppedKerbSpan;
using stratum::osm::road::EdgeId;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::JunctionBuilder;
using stratum::osm::road::JunctionConfig;
using stratum::osm::road::kCoincidentRadius;
using stratum::osm::road::kInvalidId;
using stratum::osm::road::ResampleConfig;
using stratum::osm::road::RoadElevationSolver;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::build_centerline;
using stratum::osm::road::build_profile;
using stratum::osm::road::ProfileConfig;
using stratum::osm::road::collect_arms;
using stratum::osm::road::dropped_kerb_spans;

/// One residential way with its topology attached
Road way(WayId id, const std::vector<NodeId>& nodes, const std::vector<glm::dvec2>& points) {
    Road r;
    r.osm_id = id;
    r.polyline = points;
    r.node_ids = nodes;
    r.type = RoadType::Residential;
    r.lanes = 2;
    r.width = 7.0f;
    return r;
}

/**
 * @brief Two junction nodes 0.4 m apart, each with two real approaches
 *
 * Node 1 and node 2 are joined by a 0.4 m stub, so both are degree 3 and the
 * cluster radius (kCoincidentRadius, 1.0 m) merges them. Every approach is 60 m,
 * long enough that neither trim is clamped by TrimConfig::max_trim_fraction and
 * every one of them is a genuine approach to the merged junction.
 */
ParsedOSMData two_nodes_one_junction() {
    ParsedOSMData data;
    data.roads.push_back(way(1, {1, 2}, {{0.0, 0.0}, {0.4, 0.0}}));
    data.roads.push_back(way(2, {1, 10}, {{0.0, 0.0}, {0.0, -60.0}}));
    data.roads.push_back(way(3, {1, 11}, {{0.0, 0.0}, {-60.0, 0.0}}));
    data.roads.push_back(way(4, {2, 12}, {{0.4, 0.0}, {0.4, 60.0}}));
    data.roads.push_back(way(5, {2, 13}, {{0.4, 0.0}, {60.4, 0.0}}));
    data.stats.processed_roads = data.roads.size();
    return data;
}

/// The graph node at a given local position, or kInvalidId
GraphNodeId node_at(const RoadGraph& graph, glm::dvec2 where) {
    for (size_t i = 0; i < graph.nodes().size(); ++i) {
        if (glm::length(graph.nodes()[i].position - where) < 1e-6) {
            return static_cast<GraphNodeId>(i);
        }
    }
    return kInvalidId;
}

/// Triangles of @p piece carrying MaterialId::Markings
size_t markings_triangles(const RoadPiece& piece) {
    size_t total = 0;
    for (const stratum::SubMesh& range : piece.mesh.effective_submeshes()) {
        if (range.material == MaterialId::Markings) total += range.index_count / 3u;
    }
    return total;
}

} // namespace

// ============================================================================
// 1. The merge is discoverable
// ============================================================================

/**
 * collect_arms() reports the cluster on every path, so the absorption can be seen.
 *
 * Returning an empty arm list for an absorbed member is correct and deliberate.
 * Returning it with no way to tell "absorbed" from "failed to solve" is what the
 * consumers could not survive.
 */
TEST(JunctionMerge, collect_arms_reports_the_cluster_it_merged) {
    ParsedOSMData data = two_nodes_one_junction();
    RoadGraph graph;
    graph.build(data);

    const GraphNodeId west = node_at(graph, {0.0, 0.0});
    const GraphNodeId east = node_at(graph, {0.4, 0.0});
    CHECK_TRUE(west != kInvalidId);
    CHECK_TRUE(east != kInvalidId);
    if (west == kInvalidId || east == kInvalidId) return;

    CHECK_EQ(graph.node(west).degree(), size_t{3});
    CHECK_EQ(graph.node(east).degree(), size_t{3});

    std::vector<RoadProfile> profiles;
    ResampleConfig rc;
    rc.smooth = false;
    for (const GraphEdge& e : graph.edges()) {
        profiles.push_back(build_profile(e, ProfileConfig{}, nullptr,
                                         stratum::osm::SideFlags::None));
        (void)build_centerline(e.polyline, rc);
    }

    // The primary keeps all four real approaches; the internal stub is dropped.
    std::vector<GraphNodeId> primary_cluster;
    const std::vector<ArmRef> primary_arms =
        collect_arms(graph, profiles, west, kCoincidentRadius, &primary_cluster);

    // The absorbed member gets none -- and now says WHY.
    std::vector<GraphNodeId> member_cluster;
    const std::vector<ArmRef> member_arms =
        collect_arms(graph, profiles, east, kCoincidentRadius, &member_cluster);

    const GraphNodeId primary = std::min(west, east);
    const GraphNodeId absorbed = std::max(west, east);

    CHECK_EQ(collect_arms(graph, profiles, primary, kCoincidentRadius).size(), size_t{4});
    CHECK_EQ(collect_arms(graph, profiles, absorbed, kCoincidentRadius).size(), size_t{0});
    CHECK_EQ(primary_arms.size() + member_arms.size(), size_t{4});

    // Both members compute the SAME cluster, primary first, whichever end asked.
    CHECK_EQ(primary_cluster.size(), size_t{2});
    CHECK_EQ(member_cluster.size(), size_t{2});
    if (primary_cluster.size() == 2 && member_cluster.size() == 2) {
        CHECK_TRUE(primary_cluster == member_cluster);
        CHECK_EQ(primary_cluster.front(), primary);
        CHECK_EQ(primary_cluster.back(), absorbed);
    }

    // A node that merges with nothing still reports itself, so a caller never has
    // to tell "no cluster" apart from "not asked".
    const GraphNodeId dead_end = node_at(graph, {0.0, -60.0});
    if (dead_end != kInvalidId) {
        std::vector<GraphNodeId> lone;
        (void)collect_arms(graph, profiles, dead_end, kCoincidentRadius, &lone);
        CHECK_EQ(lone.size(), size_t{1});
        if (lone.size() == 1) CHECK_EQ(lone.front(), dead_end);
    }
}

/**
 * JunctionBuilder publishes the node-to-primary map the consumers resolve through.
 */
TEST(JunctionMerge, the_solver_publishes_which_junction_owns_each_node) {
    ParsedOSMData data = two_nodes_one_junction();
    RoadGraph graph;
    graph.build(data);

    std::vector<Centerline> centerlines;
    std::vector<RoadProfile> profiles;
    ResampleConfig rc;
    rc.smooth = false;
    for (const GraphEdge& e : graph.edges()) {
        centerlines.push_back(build_centerline(e.polyline, rc));
        profiles.push_back(build_profile(e, ProfileConfig{}, nullptr,
                                         stratum::osm::SideFlags::None));
    }

    RoadElevationSolver elevation;
    JunctionBuilder builder;
    CHECK_TRUE(builder.solve_trims(graph, centerlines, profiles, elevation, JunctionConfig{}));

    const std::vector<GraphNodeId>& owner = builder.junction_owner();
    CHECK_EQ(owner.size(), graph.nodes().size());
    if (owner.size() != graph.nodes().size()) return;

    const GraphNodeId west = node_at(graph, {0.0, 0.0});
    const GraphNodeId east = node_at(graph, {0.4, 0.0});
    if (west == kInvalidId || east == kInvalidId) return;

    const GraphNodeId primary = std::min(west, east);
    const GraphNodeId absorbed = std::max(west, east);

    CHECK_EQ(owner[primary], primary);
    CHECK_EQ(owner[absorbed], primary);   // the absorbed member names its owner

    // A dead end never merges and never participates.
    const GraphNodeId dead_end = node_at(graph, {0.0, -60.0});
    if (dead_end != kInvalidId) {
        CHECK_EQ(owner[dead_end], kInvalidId);
    }
}

// ============================================================================
// 2. The consumers
// ============================================================================

/**
 * Every approach to the merged junction is painted, not just the primary's.
 *
 * ### How this test fails without the fix
 *
 * `node_has_junction` was written only from the junctions the solver EMITTED, and
 * an absorbed member emits none, so the guard in front of build_approach_markings()
 * skipped both of its approaches. Ways 4 and 5 came back with strictly fewer
 * MaterialId::Markings triangles than ways 2 and 3 -- the same road, the same
 * junction, the same 9 m of trim, no stop line.
 */
TEST(JunctionMerge, an_absorbed_member_still_paints_its_approaches) {
    ParsedOSMData data = two_nodes_one_junction();

    RoadNetworkConfig cfg;
    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(data, cfg);
    const RoadGraph& graph = builder.graph();

    // Ways 2 and 3 approach the primary; ways 4 and 5 approach the node it
    // absorbed. All four are the same road cut back by the same solve.
    size_t at_primary = 0;
    size_t at_absorbed = 0;
    size_t pieces_at_primary = 0;
    size_t pieces_at_absorbed = 0;

    for (const RoadPiece& piece : network.pieces) {
        if (piece.edge == kInvalidId || piece.edge >= graph.edges().size()) continue;
        const WayId source = graph.edge(piece.edge).source_way;
        if (source == 2 || source == 3) {
            at_primary += markings_triangles(piece);
            ++pieces_at_primary;
        } else if (source == 4 || source == 5) {
            at_absorbed += markings_triangles(piece);
            ++pieces_at_absorbed;
        }
    }

    CHECK_EQ(pieces_at_primary, size_t{2});
    CHECK_EQ(pieces_at_absorbed, size_t{2});
    CHECK_TRUE(at_primary > 0);

    // The assertion. Four identical approaches to one junction carry the same
    // paint; before the fix the absorbed member's two carried strictly less.
    CHECK_EQ(at_absorbed, at_primary);
}

/**
 * A junction crossing on an absorbed member's approach cuts the PRIMARY's ring.
 *
 * ### How this test fails without the fix
 *
 * `Crossing::node` names the arm's own end node, so the span was demanded from a
 * node with no ring and the primary's ring -- the one the crossing actually
 * stands on -- never saw it. dropped_kerb_spans() now accepts the nodes the ring
 * covers, and the builder hands it the cluster from junction_owner().
 */
TEST(JunctionMerge, a_crossing_on_an_absorbed_arm_drops_the_primary_ring) {
    // Built directly rather than located, so the test states exactly the input
    // dropped_kerb_spans() has to survive: a junction crossing that names a node
    // which is not the one whose ring is being cut.
    const GraphNodeId primary = 0;
    const GraphNodeId absorbed = 1;

    Crossing c;
    c.at_junction = true;
    c.node = absorbed;
    c.edge = 4;
    c.position = glm::dvec2(0.0, 12.0);
    c.axis = glm::dvec2(1.0, 0.0);
    c.width = 7.0f;

    const std::vector<Crossing> crossings{c};
    CrossingConfig cfg;

    // Keyed on the primary alone: nothing, because the crossing names the member.
    const std::vector<DroppedKerbSpan> without =
        dropped_kerb_spans(crossings, primary, glm::dvec2(0.0, 0.0), cfg);
    CHECK_EQ(without.size(), size_t{0});

    // Told which nodes this ring covers: the drop lands where the kerb is.
    const std::vector<GraphNodeId> absorbed_nodes{absorbed};
    const std::vector<DroppedKerbSpan> with =
        dropped_kerb_spans(crossings, primary, glm::dvec2(0.0, 0.0), cfg, &absorbed_nodes);
    CHECK_TRUE(with.size() > 0);

    // And a crossing belonging to some unrelated junction is still refused.
    const std::vector<GraphNodeId> unrelated{7};
    CHECK_EQ(dropped_kerb_spans(crossings, primary, glm::dvec2(0.0, 0.0), cfg, &unrelated).size(),
             size_t{0});
}
