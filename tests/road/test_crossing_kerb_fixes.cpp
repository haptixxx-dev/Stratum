/**
 * @file test_crossing_kerb_fixes.cpp
 * @brief Regressions for the crossing and kerb defects found in the P5 review
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Five defects, each with the assertion that fails when its fix is reverted.
 *
 * 1. **The zebra did not reach the kerb at a mitred joint.** Crossing::width was
 *    the profile's lateral span, but the corridor puts every strip edge through
 *    offset_point(), which multiplies by Station::miter_scale. At a right-angle
 *    joint the carriageway is 1.414 times as wide on the ground as it is in the
 *    profile, so the paint stopped 1.45 m short of each kerb.
 * 2. **A junction setback walked a crossing into the opposite junction.** The
 *    setback was clamped to the whole edge instead of to the span the corridor
 *    occupies, so on a short block it landed past the far trim station, where
 *    there is no ribbon, no kerb and a stop line already painted.
 * 3. **A mid-block crossing dropped no kerb at all.** The only kerb the pipeline
 *    could cut belonged to a junction ring, and a mid-block crossing has none.
 *    corridor_kerb_drops() is the missing half.
 * 4. **A junction crossing's two drops did not meet.** The ring dropped from the
 *    arm's carriage corner inward; the arm's own corridor kerb ran up to that
 *    same corner at full height. The corridor run now reaches the trim station
 *    with no ramp, so the two are at one height where they meet.
 * 5. **A dropped kerb's face kept its full-height batter.** 20 mm of rise with
 *    20 mm of lean is a 45 degree wedge across the gutter, and it appeared and
 *    vanished down the ramp because the lateral never moved while the height did.
 *
 * The divided-road island is checked too. It was suspected and found correct: a
 * painted median is not an island and is painted straight across, a raised one
 * splits the run in two.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests CrossingKerbFixes
 * @endcode
 */

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/crossings.hpp"
#include "osm/road/junction_curb.hpp"
#include "osm/road/junction_polygon.hpp"
#include "osm/road/junction_trim.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::osm::NodeId;
using stratum::osm::OSMNode;
using stratum::osm::ParsedOSMData;
using stratum::osm::Road;
using stratum::osm::RoadType;
using stratum::osm::WayId;
using stratum::osm::road::ArmEnd;
using stratum::osm::road::ArmRef;
using stratum::osm::road::Centerline;
using stratum::osm::road::CorridorKerbDrop;
using stratum::osm::road::CorridorKerbProfile;
using stratum::osm::road::Crossing;
using stratum::osm::road::CrossingConfig;
using stratum::osm::road::CurbRing;
using stratum::osm::road::CurbRingConfig;
using stratum::osm::road::FilletConfig;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::JunctionPolygon;
using stratum::osm::road::KerbDrops;
using stratum::osm::road::ResampleConfig;
using stratum::osm::road::RoadElevationSolver;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::Station;
using stratum::osm::road::Strip;
using stratum::osm::road::StripKind;
using stratum::osm::road::TrimConfig;
using stratum::osm::road::build_centerline;
using stratum::osm::road::build_crossing;
using stratum::osm::road::build_curb_ring;
using stratum::osm::road::build_junction_polygon;
using stratum::osm::road::corridor_kerb_drops;
using stratum::osm::road::dropped_kerb_spans;
using stratum::osm::road::find_crossings;
using stratum::osm::road::offset_point;

namespace jt = stratum::test::junction;

// ============================================================================
// Fixtures
// ============================================================================

/// One way with topology attached
Road road_of(WayId id, const std::vector<NodeId>& nodes, const std::vector<glm::dvec2>& points) {
    Road r;
    r.osm_id = id;
    r.polyline = points;
    r.node_ids = nodes;
    r.type = RoadType::Residential;
    r.lanes = 2;
    r.width = 7.0f;
    return r;
}

void push_strip(RoadProfile& p, double width, double height_left, double height_right,
                MaterialId material, StripKind kind) {
    Strip s;
    s.width = static_cast<float>(width);
    s.height_left = static_cast<float>(height_left);
    s.height_right = static_cast<float>(height_right);
    s.material = material;
    s.kind = kind;
    p.strips.push_back(s);
}

/**
 * @brief Lanes with a gutter, a kerb and a footway on each side
 *
 * The profile a corridor kerb drop can act on. profile_has_kerb() in
 * crossings.cpp looks for exactly the CurbFace height change this carries.
 */
RoadProfile kerbed_profile(int lanes = 2, double lane_width = 3.5) {
    RoadProfile p;
    push_strip(p, 2.0, 0.15, 0.15, MaterialId::Sidewalk, StripKind::Sidewalk);
    push_strip(p, 0.15, 0.15, 0.15, MaterialId::Curb, StripKind::CurbTop);
    push_strip(p, 0.02, 0.15, 0.0, MaterialId::Curb, StripKind::CurbFace);
    push_strip(p, 0.3, 0.0, 0.0, MaterialId::Concrete, StripKind::Gutter);
    for (int i = 0; i < lanes; ++i) {
        push_strip(p, lane_width, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);
    }
    push_strip(p, 0.3, 0.0, 0.0, MaterialId::Concrete, StripKind::Gutter);
    push_strip(p, 0.02, 0.0, 0.15, MaterialId::Curb, StripKind::CurbFace);
    push_strip(p, 0.15, 0.15, 0.15, MaterialId::Curb, StripKind::CurbTop);
    push_strip(p, 2.0, 0.15, 0.15, MaterialId::Sidewalk, StripKind::Sidewalk);
    return p;
}

/// Two lanes each way either side of a median, raised or painted flush
RoadProfile divided_profile(bool raised_median, double median_width = 1.0) {
    RoadProfile p;
    push_strip(p, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);
    push_strip(p, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);
    if (raised_median) {
        push_strip(p, 0.02, 0.0, 0.15, MaterialId::Curb, StripKind::CurbFace);
        push_strip(p, median_width, 0.15, 0.15, MaterialId::Concrete, StripKind::Median);
        push_strip(p, 0.02, 0.15, 0.0, MaterialId::Curb, StripKind::CurbFace);
    } else {
        push_strip(p, median_width, 0.0, 0.0, MaterialId::Asphalt, StripKind::Median);
    }
    push_strip(p, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);
    push_strip(p, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);
    return p;
}

/// A graph, its centerlines and one profile per edge, with crossing nodes tagged
struct Located {
    ParsedOSMData data;
    RoadGraph graph;
    std::vector<Centerline> centerlines;
    std::vector<RoadProfile> profiles;
    RoadElevationSolver elevation;
    std::vector<Crossing> crossings;
};

Located locate(const std::vector<Road>& roads,
               const std::vector<NodeId>& crossing_nodes,
               const RoadProfile& profile,
               const CrossingConfig& cfg = {}) {
    Located out;
    out.data.roads = roads;
    out.data.stats.processed_roads = roads.size();
    for (NodeId id : crossing_nodes) {
        OSMNode node;
        node.id = id;
        node.tags["highway"] = "crossing";
        out.data.nodes[id] = node;
    }
    out.graph.build(out.data);

    ResampleConfig rc;
    rc.smooth = false;
    for (const GraphEdge& e : out.graph.edges()) {
        out.centerlines.push_back(build_centerline(e.polyline, rc));
    }
    out.profiles.assign(out.graph.edges().size(), profile);
    out.crossings = find_crossings(out.graph, out.data, out.centerlines, out.profiles,
                                   out.elevation, cfg);
    return out;
}

/// Force a trim onto every arm of every junction, without running the solver
void set_junction_trims(RoadGraph& graph, double trim) {
    for (size_t i = 0; i < graph.edges().size(); ++i) {
        auto& e = const_cast<GraphEdge&>(graph.edges()[i]);
        if (e.from != stratum::osm::road::kInvalidId && graph.node(e.from).degree() >= 3) {
            e.trim_from = trim;
        }
        if (e.to != stratum::osm::road::kInvalidId && graph.node(e.to).degree() >= 3) {
            e.trim_to = trim;
        }
    }
}

/// Station of @p cl whose arclength is nearest @p s
const Station* station_at(const Centerline& cl, double s) {
    const Station* best = nullptr;
    double best_gap = 1e30;
    for (const Station& st : cl.stations) {
        const double gap = std::fabs(st.arclength - s);
        if (gap < best_gap) {
            best_gap = gap;
            best = &st;
        }
    }
    return best;
}

} // namespace

// ============================================================================
// 1. The zebra reaches the kerb at a mitred joint
// ============================================================================

/**
 * @brief Crossing::width is the ground distance, miter and fold clamp included
 *
 * The way turns 90 degrees at the crossing node, so the joint mitres with
 * Station::miter_scale = sqrt(2) and the carriageway on the ground is 9.899 m
 * across, not the profile's 7 m. Before the fix Crossing::width was 7 and the
 * paint stopped 1.45 m short of each kerb.
 */
TEST(CrossingKerbFixes, zebra_width_follows_the_mitred_carriageway) {
    const std::vector<Road> roads = {
        road_of(1, {1, 2, 3}, {{-50.0, 0.0}, {0.0, 0.0}, {0.0, 50.0}}),
    };
    RoadProfile lanes;
    push_strip(lanes, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);
    push_strip(lanes, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);

    const Located net = locate(roads, {2}, lanes);
    CHECK_EQ(net.crossings.size(), size_t{1});
    if (net.crossings.size() != 1) return;

    const Crossing& c = net.crossings.front();
    const Centerline& cl = net.centerlines[c.edge];
    const Station* st = station_at(cl, c.arclength);
    CHECK_TRUE(st != nullptr);
    if (st == nullptr) return;

    // The corridor's own construction of the carriageway edge at this station.
    const glm::dvec2 left = offset_point(*st, 3.5);
    const glm::dvec2 right = offset_point(*st, -3.5);
    const double ground = glm::length(left - right);

    CHECK_NEAR(static_cast<double>(c.width), ground, 1e-3);
    // And it is genuinely wider than the profile span, or the fixture is wrong
    // and the test proves nothing.
    CHECK_TRUE(ground > 7.0 + 1.0);
}

/**
 * @brief No stripe crosses the true carriageway edge at that same joint
 *
 * The other half of the same defect: over-reaching is worse than under-reaching,
 * because paint on the footway is visible from every angle. Every stripe vertex
 * must sit within half the ground width of the crossing centre along the axis.
 */
TEST(CrossingKerbFixes, mitred_zebra_stays_inside_the_kerbs) {
    const std::vector<Road> roads = {
        road_of(1, {1, 2, 3}, {{-50.0, 0.0}, {0.0, 0.0}, {0.0, 50.0}}),
    };
    RoadProfile lanes;
    push_strip(lanes, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);
    push_strip(lanes, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);

    const Located net = locate(roads, {2}, lanes);
    if (net.crossings.size() != 1) {
        CHECK_EQ(net.crossings.size(), size_t{1});
        return;
    }

    const Crossing& c = net.crossings.front();
    const Mesh zebra = build_crossing(c, CrossingConfig{});
    CHECK_TRUE(!zebra.indices.empty());

    const double half = 0.5 * static_cast<double>(c.width);
    double worst = 0.0;
    for (const auto& v : zebra.vertices) {
        const glm::dvec2 local = jt::world_to_local(v.position) - c.position;
        worst = std::max(worst, std::fabs(glm::dot(local, c.axis)));
    }
    CHECK_TRUE(worst <= half + 1e-6);
}

// ============================================================================
// 2. The junction setback stays inside the trims
// ============================================================================

/**
 * @brief A crossing on a short block never lands past the far trim station
 *
 * Two crossroads 7 m apart, each trimming 2.8 m off the block. The corridor is
 * the 1.4 m between the two trim stations, and a 1.5 m setback measured from
 * either one overshoots it. Before the fix the setback was clamped to the whole
 * edge, so the crossing sat inside the opposite junction's polygon.
 */
TEST(CrossingKerbFixes, junction_setback_stays_inside_the_corridor) {
    constexpr double kBlock = 7.0;
    constexpr double kTrim = 2.8;

    const std::vector<Road> roads = {
        road_of(1, {10, 11, 12, 13},
                {{-100.0, 0.0}, {0.0, 0.0}, {kBlock, 0.0}, {100.0 + kBlock, 0.0}}),
        road_of(2, {20, 11, 21}, {{0.0, -100.0}, {0.0, 0.0}, {0.0, 100.0}}),
        road_of(3, {30, 12, 31}, {{kBlock, -100.0}, {kBlock, 0.0}, {kBlock, 100.0}}),
    };

    Located net;
    net.data.roads = roads;
    net.data.stats.processed_roads = roads.size();
    OSMNode node;
    node.id = 12;
    node.tags["highway"] = "crossing";
    net.data.nodes[12] = node;
    net.graph.build(net.data);

    ResampleConfig rc;
    rc.smooth = false;
    for (const GraphEdge& e : net.graph.edges()) {
        net.centerlines.push_back(build_centerline(e.polyline, rc));
    }
    net.profiles.assign(net.graph.edges().size(), kerbed_profile());
    set_junction_trims(net.graph, kTrim);

    net.crossings = find_crossings(net.graph, net.data, net.centerlines, net.profiles,
                                   net.elevation, CrossingConfig{});
    CHECK_TRUE(!net.crossings.empty());

    for (const Crossing& c : net.crossings) {
        const GraphEdge& e = net.graph.edges()[c.edge];
        const double length = net.centerlines[c.edge].length();
        CHECK_TRUE(c.arclength >= e.trim_from - 1e-6);
        CHECK_TRUE(c.arclength <= length - e.trim_to + 1e-6);
    }
}

// ============================================================================
// 3 and 4. The corridor kerb drops
// ============================================================================

/**
 * @brief A mid-block crossing cuts the corridor's own kerb, ramps and all
 *
 * The defect crossings.hpp named and never fixed. Before corridor_kerb_drops()
 * there was no run at all here, and the zebra ran into a 150 mm wall.
 */
TEST(CrossingKerbFixes, mid_block_crossing_drops_the_corridor_kerb) {
    const CrossingConfig cfg;
    const std::vector<Road> roads = {
        road_of(1, {10, 11, 12}, {{-100.0, 0.0}, {0.0, 0.0}, {100.0, 0.0}}),
    };
    const Located net = locate(roads, {11}, kerbed_profile(), cfg);
    CHECK_EQ(net.crossings.size(), size_t{1});
    if (net.crossings.size() != 1) return;
    CHECK_TRUE(!net.crossings.front().at_junction);

    const std::vector<CorridorKerbDrop> drops =
        corridor_kerb_drops(net.crossings, net.graph, net.centerlines, net.profiles, cfg);
    CHECK_EQ(drops.size(), size_t{1});
    if (drops.size() != 1) return;

    const CorridorKerbDrop& d = drops.front();
    const double s = net.crossings.front().arclength;
    // The flat is at least as wide as the painted corridor and is centred on the
    // crossing, and a mid-block drop ramps at BOTH ends.
    CHECK_NEAR(0.5 * (d.from + d.to), s, 1e-6);
    CHECK_TRUE(d.to - d.from >= static_cast<double>(cfg.crossing_depth) - 1e-6);
    CHECK_NEAR(d.ramp_from, static_cast<double>(cfg.dropped_kerb_ramp), 1e-9);
    CHECK_NEAR(d.ramp_to, static_cast<double>(cfg.dropped_kerb_ramp), 1e-9);

    const CorridorKerbProfile profile(drops, net.crossings.front().edge, 0.15);
    CHECK_TRUE(profile.active());

    // Full kerb clear of the run, the lip across it, and monotone down the ramp.
    CHECK_NEAR(profile.top_height(d.from - d.ramp_from - 1.0, true, 0.15), 0.15, 1e-9);
    CHECK_NEAR(profile.top_height(s, true, 0.15), static_cast<double>(d.height), 1e-9);
    CHECK_NEAR(profile.top_height(s, false, 0.15), static_cast<double>(d.height), 1e-9);

    double previous = 0.16;
    for (int k = 0; k <= 10; ++k) {
        const double at = d.from - d.ramp_from + d.ramp_from * (k / 10.0);
        const double h = profile.top_height(at, true, 0.15);
        CHECK_TRUE(h <= previous + 1e-9);
        previous = h;
    }

    // The ramp only reads as a slope if the centerline carries columns down it.
    const std::vector<double> wanted = profile.required_stations(0.0, 200.0);
    CHECK_TRUE(wanted.size() >= 8);
    CHECK_TRUE(std::is_sorted(wanted.begin(), wanted.end()));
    CHECK_TRUE(wanted.front() >= d.from - d.ramp_from - 1e-9);
    CHECK_TRUE(wanted.back() <= d.to + d.ramp_to + 1e-9);
}

/**
 * @brief An edge with no kerb demands no drop, and so costs no vertices
 *
 * A rural road with a verge has nothing to cut, and resampling it across a drop
 * would add columns to every such edge in an extract for no visible change.
 */
TEST(CrossingKerbFixes, kerbless_profile_demands_no_corridor_drop) {
    const CrossingConfig cfg;
    const std::vector<Road> roads = {
        road_of(1, {10, 11, 12}, {{-100.0, 0.0}, {0.0, 0.0}, {100.0, 0.0}}),
    };
    RoadProfile lanes;
    push_strip(lanes, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);
    push_strip(lanes, 3.5, 0.0, 0.0, MaterialId::Asphalt, StripKind::Lane);

    const Located net = locate(roads, {11}, lanes, cfg);
    CHECK_EQ(net.crossings.size(), size_t{1});
    CHECK_TRUE(corridor_kerb_drops(net.crossings, net.graph, net.centerlines,
                                   net.profiles, cfg)
                   .empty());
}

/**
 * @brief A junction crossing's drop runs to the trim station and does not ramp there
 *
 * That station is where the corridor stops and the junction ring's own dropped
 * span begins. Both are at the lip there. A ramp climbing back to full height in
 * the last metre before it would build the very step the drop exists to avoid.
 */
TEST(CrossingKerbFixes, junction_crossing_drop_meets_the_ring_at_the_trim) {
    const CrossingConfig cfg;
    const std::vector<Road> roads = {
        road_of(1, {10, 11, 12}, {{-100.0, 0.0}, {0.0, 0.0}, {100.0, 0.0}}),
        road_of(2, {20, 11, 21}, {{0.0, -100.0}, {0.0, 0.0}, {0.0, 100.0}}),
    };

    Located net;
    net.data.roads = roads;
    net.data.stats.processed_roads = roads.size();
    OSMNode node;
    node.id = 11;
    node.tags["highway"] = "crossing";
    net.data.nodes[11] = node;
    net.graph.build(net.data);

    ResampleConfig rc;
    rc.smooth = false;
    for (const GraphEdge& e : net.graph.edges()) {
        net.centerlines.push_back(build_centerline(e.polyline, rc));
    }
    net.profiles.assign(net.graph.edges().size(), kerbed_profile());
    set_junction_trims(net.graph, 6.0);

    net.crossings = find_crossings(net.graph, net.data, net.centerlines, net.profiles,
                                   net.elevation, cfg);
    CHECK_EQ(net.crossings.size(), size_t{1});
    if (net.crossings.size() != 1) return;
    CHECK_TRUE(net.crossings.front().at_junction);

    const std::vector<CorridorKerbDrop> drops =
        corridor_kerb_drops(net.crossings, net.graph, net.centerlines, net.profiles, cfg);
    CHECK_EQ(drops.size(), size_t{1});
    if (drops.size() != 1) return;

    const CorridorKerbDrop& d = drops.front();
    const GraphEdge& e = net.graph.edges()[d.edge];
    const double length = net.centerlines[d.edge].length();
    const double corridor_lo = e.trim_from;
    const double corridor_hi = length - e.trim_to;

    const bool at_from = std::fabs(d.from - corridor_lo) < 1e-6;
    const bool at_to = std::fabs(d.to - corridor_hi) < 1e-6;
    CHECK_TRUE(at_from || at_to);
    if (at_from) {
        CHECK_NEAR(d.ramp_from, 0.0, 1e-12);
        CHECK_TRUE(d.ramp_to > 0.0);
    } else {
        CHECK_NEAR(d.ramp_to, 0.0, 1e-12);
        CHECK_TRUE(d.ramp_from > 0.0);
    }

    // The kerb is AT the lip where it hands over, not on its way back up.
    const CorridorKerbProfile profile(drops, d.edge, 0.15);
    const double seam = at_from ? corridor_lo : corridor_hi;
    CHECK_NEAR(profile.top_height(seam, true, 0.15), static_cast<double>(d.height), 1e-9);
}

// ============================================================================
// 5. The dropped kerb's face keeps its batter
// ============================================================================

/**
 * @brief A ring's curb face leans in proportion to the height it still has
 *
 * CurbRingConfig::curb_face_batter is the lean over the FULL curb height. At a
 * drop the face is 20 mm tall, so it may lean 20/150ths of the batter and no
 * more. Before the fix it kept the whole 20 mm of lean over 20 mm of rise -- a
 * 45 degree wedge jutting into the gutter, appearing and vanishing down the ramp
 * because the lateral never moved while the height did.
 *
 * Measured as the outermost Curb vertex sitting at the lip: it is the outer edge
 * of the curb top, at `apron + face + curb_top_width` from the junction ring.
 */
TEST(CrossingKerbFixes, dropped_curb_face_keeps_its_batter) {
    const CurbRingConfig cfg;
    const CrossingConfig xcfg;

    jt::Fixture fixture = jt::symmetric_cross(2);
    const GraphNodeId node = jt::sole_node_of_degree(fixture.graph, 4);
    CHECK_TRUE(node != stratum::osm::road::kInvalidId);
    if (node == stratum::osm::road::kInvalidId) return;

    std::vector<ArmRef> arms;
    std::vector<ArmEnd> ends;
    jt::solve_node(fixture, node, TrimConfig{}, arms, ends);
    const JunctionPolygon poly = build_junction_polygon(arms, ends, FilletConfig{});
    CHECK_TRUE(poly.valid);
    if (!poly.valid) return;

    // One crossing per arm, at the shipping setback off each cut line.
    std::vector<Crossing> crossings;
    for (size_t k = 0; k < arms.size(); ++k) {
        Crossing c;
        c.at_junction = true;
        c.node = node;
        c.edge = arms[k].edge;
        c.position = ends[k].center + ends[k].direction * static_cast<double>(xcfg.setback);
        c.axis = glm::dvec2(-ends[k].direction.y, ends[k].direction.x);
        c.width = static_cast<float>(glm::length(ends[k].carriage_left - ends[k].carriage_right));
        crossings.push_back(c);
    }

    KerbDrops drops;
    drops.center = fixture.graph.node(node).position;
    drops.ramp_length = static_cast<double>(xcfg.dropped_kerb_ramp);
    drops.spans = dropped_kerb_spans(crossings, node, drops.center, xcfg);
    CHECK_TRUE(!drops.spans.empty());

    const CurbRing ring = build_curb_ring(poly, arms, ends, 0.0f, cfg, &drops);
    CHECK_TRUE(ring.valid);
    if (!ring.valid) return;

    const double lip = static_cast<double>(xcfg.dropped_kerb_height);
    const double fraction = lip / cfg.curb_height;
    const double allowed = cfg.curb_face_batter * fraction;

    // Every vertex column of the ring emits its curb-face BOTTOM at the
    // carriageway surface and its curb-face TOP at the drop height, one batter
    // apart laterally. Pairing them by nearest neighbour recovers the face
    // without leaning on the order the extruder happened to emit vertices in:
    // the next column along is 0.2 m away across a drop, two orders of magnitude
    // further than the face is deep, and the curb TOP's outer edge is a further
    // curb_top_width out and so falls outside the window.
    std::vector<glm::dvec2> bottoms;
    std::vector<glm::dvec2> tops;
    for (const auto& v : ring.mesh.vertices) {
        const double y = static_cast<double>(v.position.y);
        if (std::fabs(y) < 1e-6) {
            bottoms.push_back(jt::world_to_local(v.position));
        } else if (std::fabs(y - lip) < 1e-4) {
            tops.push_back(jt::world_to_local(v.position));
        }
    }
    CHECK_TRUE(!bottoms.empty());
    CHECK_TRUE(!tops.empty());

    /// Window that holds a curb face and excludes both its neighbours, metres
    constexpr double kFaceWindow = 0.05;

    size_t counted = 0;
    double worst = 0.0;
    for (const glm::dvec2& top : tops) {
        double nearest = 1e30;
        for (const glm::dvec2& bottom : bottoms) {
            nearest = std::min(nearest, glm::length(top - bottom));
        }
        // A column whose reach the offset pulled in has no face left to measure.
        if (nearest <= 1e-6 || nearest >= kFaceWindow) continue;
        ++counted;
        worst = std::max(worst, nearest);
    }

    CHECK_TRUE(counted > 0);
    CHECK_TRUE(worst <= allowed + 1e-3);
    // The unfixed value is the full batter. If the two were not far apart the
    // assertion above would pass either way and prove nothing.
    CHECK_TRUE(cfg.curb_face_batter - allowed > 0.01);
}

// ============================================================================
// The divided-road island: suspected, and found correct
// ============================================================================

/**
 * @brief A painted median is not an island; the zebra runs straight over it
 */
TEST(CrossingKerbFixes, painted_median_is_painted_straight_across) {
    const std::vector<Road> roads = {
        road_of(1, {10, 11, 12}, {{-100.0, 0.0}, {0.0, 0.0}, {100.0, 0.0}}),
    };
    const Located net = locate(roads, {11}, divided_profile(/*raised_median=*/false));
    CHECK_EQ(net.crossings.size(), size_t{1});
    if (net.crossings.size() != 1) return;

    const Crossing& c = net.crossings.front();
    CHECK_NEAR(static_cast<double>(c.island_left), static_cast<double>(c.island_right), 1e-9);
    CHECK_NEAR(static_cast<double>(c.width), 15.0, 1e-6);

    const Mesh zebra = build_crossing(c, CrossingConfig{});
    CHECK_TRUE(!zebra.indices.empty());

    // A stripe sits on the centreline, which is where the median is.
    bool over_the_median = false;
    for (const auto& v : zebra.vertices) {
        const glm::dvec2 local = jt::world_to_local(v.position) - c.position;
        if (std::fabs(glm::dot(local, c.axis)) < 0.5) {
            over_the_median = true;
            break;
        }
    }
    CHECK_TRUE(over_the_median);
}

/**
 * @brief A raised median IS an island; no paint is laid on it
 */
TEST(CrossingKerbFixes, raised_median_splits_the_zebra_in_two) {
    const std::vector<Road> roads = {
        road_of(1, {10, 11, 12}, {{-100.0, 0.0}, {0.0, 0.0}, {100.0, 0.0}}),
    };
    const Located net = locate(roads, {11}, divided_profile(/*raised_median=*/true));
    CHECK_EQ(net.crossings.size(), size_t{1});
    if (net.crossings.size() != 1) return;

    const Crossing& c = net.crossings.front();
    CHECK_TRUE(c.island_left - c.island_right > 0.9f);

    const Mesh zebra = build_crossing(c, CrossingConfig{});
    CHECK_TRUE(!zebra.indices.empty());

    for (const auto& v : zebra.vertices) {
        const glm::dvec2 local = jt::world_to_local(v.position) - c.position;
        const double lateral = glm::dot(local, c.axis);
        const bool inside_island = lateral > static_cast<double>(c.island_right) + 1e-6 &&
                                   lateral < static_cast<double>(c.island_left) - 1e-6;
        CHECK_TRUE(!inside_island);
    }
}
