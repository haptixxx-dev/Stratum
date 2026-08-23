/**
 * @file test_markings.cpp
 * @brief Painted lane markings: phase, plane, class rules, arrows, and the atlas
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Markings are the first geometry in this pipeline that is a scatter of discrete
 * quads rather than a swept ribbon, and every failure mode is a failure of
 * PLACEMENT rather than of shape. A dash in the wrong place still looks like a
 * dash. So the assertions here are almost all about where a quad is, measured in
 * the road's own frame, and about which sprite is on it.
 *
 * Four of them are worth naming, because each catches a defect that no amount of
 * looking at a screenshot would find:
 *
 * 1. **Dash phase.** Dashes are laid out on one arclength phase over the whole
 *    edge. An emitter that restarts the phase at each station instead produces a
 *    line that looks entirely correct until you measure it: the dashes are the
 *    right length, they are just spaced at the station spacing rather than at
 *    dash_length + dash_gap. ResampleConfig::max_spacing is 8 m and the dash
 *    period is 9 m, so the two are close enough to be invisible and far enough
 *    apart to assert.
 * 2. **The paint plane.** Markings must sit exactly
 *    MarkingConfig::height_above_surface above the carriageway and must share no
 *    vertex with it. Coplanar geometry z-fights, and a welded marking would be
 *    dragged into the corridor's own normals by P7.
 * 3. **The miter.** A marking is offset from the centerline exactly as a strip
 *    edge is, through offset_point(), which is the single definition of the
 *    miter in this codebase. An emitter that offsets by `normal * lateral` and
 *    forgets Station::miter_scale pinches every line inward at every corner, and
 *    because the pinch is INWARD the line stays inside the carriageway and no
 *    containment test catches it. The drift guard measures against the offset
 *    polyline itself.
 * 4. **turn:lanes.** A `turn:lanes` whose entry count disagrees with the profile
 *    is the common failure of that tag, and every way of guessing what was meant
 *    paints a false instruction on a real lane.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests Markings
 * @endcode
 */

#include "framework.hpp"
#include "road/p5_p6_fixtures.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/corridor.hpp"
#include "osm/road/marking_atlas.hpp"
#include "osm/road/markings.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::osm::RoadType;
using stratum::osm::TagMap;
using stratum::osm::road::Centerline;
using stratum::osm::road::Corridor;
using stratum::osm::road::CorridorConfig;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::MarkingConfig;
using stratum::osm::road::MarkingSprite;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::SpriteRect;
using stratum::osm::road::SpriteSize;
using stratum::osm::road::build_centerline;
using stratum::osm::road::build_corridor;
using stratum::osm::road::build_approach_markings;
using stratum::osm::road::build_edge_markings;
using stratum::osm::road::offset_point;
using stratum::osm::road::sprite_rect;
using stratum::osm::road::sprite_size;

namespace p5 = stratum::test::p5;
namespace jt = stratum::test::junction;

// ============================================================================
// Synthetic edges
// ============================================================================

/**
 * @brief A straight edge running due east from the origin
 *
 * Due east so that the road frame and the local frame coincide: the tangent is
 * +x, the left normal is +y, and a lateral offset is simply a y coordinate. That
 * makes every placement expectation in the straight tests exact arithmetic
 * rather than a projection.
 *
 * @param length Length in metres
 * @param type   Road classification, which decides the class marking rules
 * @param lanes  Total lane count
 * @param fwd    lanes:forward, or -1 when unspecified
 * @param bwd    lanes:backward, or -1 when unspecified
 * @param oneway One way in the direction of travel
 * @return The edge, with node ids 1 and 2 and no trims
 */
GraphEdge straight_edge(double length, RoadType type, int lanes, int fwd = -1, int bwd = -1,
                        bool oneway = false) {
    GraphEdge edge;
    edge.source_way = 1;
    edge.from = 0;
    edge.to = 1;
    edge.polyline = {{0.0, 0.0}, {length, 0.0}};
    edge.node_ids = {1, 2};
    edge.type = type;
    edge.lanes = lanes;
    edge.lanes_forward = fwd;
    edge.lanes_backward = bwd;
    edge.is_oneway = oneway;
    edge.width = static_cast<float>(3.5 * lanes);
    return edge;
}

/**
 * @brief An edge turning a square corner at the origin
 *
 * 100 m due east into the corner, then 100 m due north out of it. The exterior
 * turn is a right angle, so Station::miter_scale at the corner is
 * 1 / cos(45 degrees) = 1.4142 and an offset column 5 m out sits 2.07 m further
 * from the corner than an unmitred one would. That gap is the drift guard's
 * whole signal, and it is why the corner is square rather than gentle.
 *
 * @param type  Road classification
 * @param lanes Total lane count
 * @param fwd   lanes:forward
 * @param bwd   lanes:backward
 * @return The edge
 */
GraphEdge corner_edge(RoadType type, int lanes, int fwd, int bwd) {
    GraphEdge edge;
    edge.source_way = 2;
    edge.from = 0;
    edge.to = 1;
    edge.polyline = {{-100.0, 0.0}, {0.0, 0.0}, {0.0, 100.0}};
    edge.node_ids = {1, 2, 3};
    edge.type = type;
    edge.lanes = lanes;
    edge.lanes_forward = fwd;
    edge.lanes_backward = bwd;
    edge.width = static_cast<float>(3.5 * lanes);
    return edge;
}

/// Marking config with every optional pass off, so one rule can be isolated
MarkingConfig only(bool lane_lines, bool edge_lines) {
    MarkingConfig cfg;
    cfg.emit_lane_lines = lane_lines;
    cfg.emit_edge_lines = edge_lines;
    cfg.emit_stop_lines = false;
    cfg.emit_arrows = false;
    return cfg;
}

/// The extent of a component along the road, for a due-east straight edge
struct Run {
    double start = 0.0;
    double end = 0.0;
    double lateral = 0.0;
    MarkingSprite sprite = MarkingSprite::Count;
};

/**
 * @brief Every component of a due-east marking mesh as a longitudinal run
 *
 * Sorted by start, so consecutive entries of one line are adjacent. Only usable
 * where every component belongs to the SAME line; the callers that need several
 * lines at once filter by lateral first.
 *
 * @param mesh Marking geometry from a due-east straight edge
 * @return The runs, ascending by start
 */
std::vector<Run> runs_of(const Mesh& mesh) {
    std::vector<Run> out;
    for (const p5::Component& comp : p5::components_of(mesh)) {
        double lo = 0.0;
        double hi = 0.0;
        p5::extent_along(mesh, comp, glm::dvec2{0.0, 0.0}, glm::dvec2{1.0, 0.0}, lo, hi);
        Run run;
        run.start = lo;
        run.end = hi;
        run.lateral = p5::centroid_of(mesh, comp).y;
        run.sprite = p5::sprite_of(mesh, comp);
        out.push_back(run);
    }
    std::sort(out.begin(), out.end(),
              [](const Run& a, const Run& b) { return a.start < b.start; });
    return out;
}

/// True when a sprite is one of the broken-line sprites
bool is_dashed(MarkingSprite s) {
    return s == MarkingSprite::DashWhite || s == MarkingSprite::DashLongWhite ||
           s == MarkingSprite::DashedYellow;
}

/// True when a sprite is one of the continuous-line sprites
bool is_solid_line(MarkingSprite s) {
    return s == MarkingSprite::SolidWhite || s == MarkingSprite::SolidYellow ||
           s == MarkingSprite::DoubleSolidYellow;
}

} // namespace

// ============================================================================
// Dash phase
// ============================================================================

/**
 * One phase over the whole edge, not one phase per station.
 *
 * The centre line of a two-lane primary is the only Lane-to-Lane boundary in a
 * bare two-lane profile, and with edge lines off it is the only geometry the
 * emitter produces. Every component is therefore one dash, and the dashes have
 * to march at dash_length + dash_gap from the start of the edge to its end.
 *
 * The station spacing is 8 m and the dash period is 9 m. An emitter that
 * restarted the phase at each station would put a dash boundary at every
 * multiple of 8 rather than of 9, which drifts a whole metre per station and is
 * caught here by the third station.
 */
TEST(Markings, dash_phase_is_continuous_across_station_boundaries) {
    // 207 m is 23 whole dash periods, so no run is a partial one and every run
    // can be held to the full dash length. Whether a final partial dash is
    // dropped or clipped is not pinned by the header and is not what this test
    // is about.
    const GraphEdge edge = straight_edge(207.0, RoadType::Primary, 2, 1, 1);
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(2, 3.5);
    CHECK_TRUE(cl.is_valid());

    MarkingConfig cfg = only(true, false);
    const Mesh mesh = build_edge_markings(edge, cl, profile, p5::flat_heights(cl, 0.0f), cfg);

    const std::vector<Run> runs = runs_of(mesh);
    CHECK_TRUE(runs.size() >= 20);
    if (runs.size() < 3) return;

    const double period = static_cast<double>(cfg.dash_length) + static_cast<double>(cfg.dash_gap);
    for (size_t i = 0; i < runs.size(); ++i) {
        CHECK_NEAR(runs[i].end - runs[i].start, cfg.dash_length, 1e-3);
        CHECK_NEAR(runs[i].lateral, 0.0, 1e-3);
        if (i + 1 < runs.size()) {
            // The gap between two dashes, and the period between their starts.
            // Both are asserted: a phase restart keeps the gap and breaks the
            // period, a wrong gap breaks both.
            CHECK_NEAR(runs[i + 1].start - runs[i].end, cfg.dash_gap, 1e-3);
            CHECK_NEAR(runs[i + 1].start - runs[i].start, period, 1e-3);
        }
    }
}

/**
 * The pattern continues across a plain way split, which is not a junction.
 *
 * A street becomes a new GraphEdge wherever an OSM way ends, and a way ends at
 * every `name`, `ref` or `maxspeed` change. Such a split lands on a plain
 * degree-2 node: the profiles agree, so no taper is built, no trim is written,
 * and the two ribbons meet flush. Without a shared datum the second edge
 * restarts its dashes at its own zero, and the joint reads as a double-length
 * dash or a short gap depending on where the first edge's length fell in the
 * cycle -- uniformly distributed, so about a third of such splits are visibly
 * wrong.
 *
 * The fixture is the worst case, not the average one: 93 m is ten whole periods
 * plus one dash, so edge A's last dash ends flush with the split and a restart
 * would butt a second dash straight onto it. The datum is the arc length of the
 * street already travelled, which road_network_builder.cpp accumulates along
 * degree-2 continuations.
 */
TEST(Markings, the_dash_pattern_continues_across_a_plain_way_split) {
    const double split_at = 93.0;

    const GraphEdge first = straight_edge(split_at, RoadType::Primary, 2, 1, 1);
    const GraphEdge second = straight_edge(60.0, RoadType::Primary, 2, 1, 1);
    const Centerline cl_first = build_centerline(first.polyline, jt::fixture_resample());
    const Centerline cl_second = build_centerline(second.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(2, 3.5);
    CHECK_TRUE(cl_first.is_valid() && cl_second.is_valid());

    const MarkingConfig cfg = only(true, false);
    const double period = static_cast<double>(cfg.dash_length) + static_cast<double>(cfg.dash_gap);

    const Mesh a = build_edge_markings(first, cl_first, profile,
                                       p5::flat_heights(cl_first, 0.0f), cfg);
    const Mesh b = build_edge_markings(second, cl_second, profile,
                                       p5::flat_heights(cl_second, 0.0f), cfg, nullptr,
                                       /*dash_phase=*/split_at);

    const std::vector<Run> runs_a = runs_of(a);
    const std::vector<Run> runs_b = runs_of(b);
    CHECK_TRUE(!runs_a.empty() && !runs_b.empty());
    if (runs_a.empty() || runs_b.empty()) return;

    // The premise: the first edge really does end flush with a dash.
    CHECK_NEAR(runs_a.back().end, split_at, 1e-3);

    // The joint gap, measured across the split in the STREET's frame.
    const double joint = (split_at - runs_a.back().end) + runs_b.front().start;
    CHECK_NEAR(joint, cfg.dash_gap, 1e-3);
    CHECK_NEAR(runs_b.front().end - runs_b.front().start, cfg.dash_length, 1e-3);

    // And the pattern keeps its period the whole way down the second edge.
    for (size_t i = 0; i + 1 < runs_b.size(); ++i) {
        CHECK_NEAR(runs_b[i + 1].start - runs_b[i].start, period, 1e-3);
    }
}

/**
 * Every marking quad is a quad: four vertices, two triangles, no more.
 *
 * A dash mapping one atlas sub-rect cannot be subdivided, because an atlas rect
 * has no interior sample points to subdivide onto. See marking_atlas.hpp: a
 * marking that repeats is one quad per repeat.
 */
TEST(Markings, every_dash_is_one_quad) {
    const GraphEdge edge = straight_edge(120.0, RoadType::Primary, 2, 1, 1);
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(2, 3.5);

    const Mesh mesh = build_edge_markings(edge, cl, profile, p5::flat_heights(cl, 0.0f),
                                          only(true, false));
    const std::vector<p5::Component> comps = p5::components_of(mesh);
    CHECK_TRUE(!comps.empty());
    for (const p5::Component& comp : comps) {
        CHECK_EQ(comp.vertices.size(), size_t{4});
        CHECK_EQ(comp.triangles.size(), size_t{2});
        CHECK_EQ(comp.material, MaterialId::Markings);
    }
}

// ============================================================================
// The paint plane
// ============================================================================

/**
 * Markings sit exactly height_above_surface above the carriageway and touch it
 * nowhere.
 *
 * Both halves matter. Sharing a vertex would let P7's weld pass pull the paint
 * into the corridor's normals and average the two surfaces together; sitting at
 * the same height would z-fight. The corridor here is built from the same
 * centerline, the same profile and the same station heights the markings were,
 * so the two are directly comparable.
 */
TEST(Markings, paint_sits_above_the_carriageway_and_shares_no_vertex_with_it) {
    const GraphEdge edge = straight_edge(150.0, RoadType::Primary, 2, 1, 1);
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(2, 3.5);

    const float surface = 12.0f;
    const std::vector<float> heights = p5::flat_heights(cl, surface);

    MarkingConfig cfg = only(true, true);
    const Mesh mesh = build_edge_markings(edge, cl, profile, heights, cfg);
    CHECK_TRUE(!mesh.vertices.empty());

    CorridorConfig ccfg;
    ccfg.station_heights = heights;
    const Corridor corridor = build_corridor(cl, profile, ccfg);
    CHECK_TRUE(!corridor.mesh.vertices.empty());

    for (const auto& v : mesh.vertices) {
        CHECK_NEAR(v.position.y, surface + cfg.height_above_surface, 1e-4);
    }
    CHECK_TRUE(p5::share_no_vertex(mesh, corridor.mesh));

    // Every triangle in the Markings slot, and no triangle outside it.
    for (const jt::Tri2D& tri : jt::triangles_of(mesh)) {
        CHECK_EQ(tri.material, MaterialId::Markings);
    }
}

/**
 * A mis-sized station_heights vector degrades to flat paint at world Y 0.
 *
 * The documented failure mode, and the safe one: a mis-sized elevation solve
 * produces paint on the wrong plane rather than paint threaded through the
 * terrain at every second station.
 */
TEST(Markings, mis_sized_station_heights_fall_back_to_flat) {
    const GraphEdge edge = straight_edge(80.0, RoadType::Primary, 2, 1, 1);
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(2, 3.5);

    MarkingConfig cfg = only(true, false);
    const std::vector<float> wrong(cl.stations.size() + 3, 40.0f);
    const Mesh mesh = build_edge_markings(edge, cl, profile, wrong, cfg);
    CHECK_TRUE(!mesh.vertices.empty());
    for (const auto& v : mesh.vertices) {
        CHECK_NEAR(v.position.y, cfg.height_above_surface, 1e-4);
    }
}

// ============================================================================
// Class rules
// ============================================================================

/**
 * A single-lane residential street gets no centre line, and neither does a
 * two-lane one.
 *
 * Two separate reasons, and both have to hold. The single-lane profile has no
 * Lane-to-Lane boundary at all, so there is nothing to paint whatever the class
 * rule says. The two-lane profile has one, and the class rule is the only thing
 * stopping it: MarkingConfig::centre_line_on_minor_roads is off by default
 * because a painted divider makes an estate street read as a distributor road.
 */
TEST(Markings, residential_gets_no_centre_line) {
    const RoadProfile one_lane = jt::lane_profile(1, 3.5);
    const RoadProfile two_lane = jt::lane_profile(2, 3.5);

    const GraphEdge single = straight_edge(120.0, RoadType::Residential, 1);
    const Centerline cl_single = build_centerline(single.polyline, jt::fixture_resample());
    const Mesh no_boundary = build_edge_markings(single, cl_single, one_lane,
                                                 p5::flat_heights(cl_single, 0.0f),
                                                 only(true, false));
    CHECK_EQ(no_boundary.indices.size(), size_t{0});

    const GraphEdge pair = straight_edge(120.0, RoadType::Residential, 2, 1, 1);
    const Centerline cl_pair = build_centerline(pair.polyline, jt::fixture_resample());
    const Mesh class_rule = build_edge_markings(pair, cl_pair, two_lane,
                                                p5::flat_heights(cl_pair, 0.0f),
                                                only(true, false));
    CHECK_EQ(class_rule.indices.size(), size_t{0});

    // And the switch turns it back on, so the absence above is the rule rather
    // than an emitter that paints nothing on a residential street ever.
    MarkingConfig opted_in = only(true, false);
    opted_in.centre_line_on_minor_roads = true;
    const Mesh painted = build_edge_markings(pair, cl_pair, two_lane,
                                             p5::flat_heights(cl_pair, 0.0f), opted_in);
    CHECK_TRUE(painted.indices.size() > 0);
}

/**
 * A motorway gets edge lines, on both sides, inside the carriageway, and gets no
 * centre line.
 *
 * The edge line is offset inward from the outermost Lane strip's outer boundary
 * by MarkingConfig::edge_line_inset. With four 3.75 m lanes the carriageway runs
 * to plus and minus 7.5 m, so the paint is centred near plus and minus 7.3 and
 * must lie wholly inside 7.5: an edge line half over the gutter is the defect
 * the inset exists to prevent.
 */
TEST(Markings, motorway_gets_edge_lines_and_no_centre_line) {
    const GraphEdge edge = straight_edge(160.0, RoadType::Motorway, 4, 2, 2);
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(4, 3.75);

    MarkingConfig cfg = only(false, true);
    const Mesh mesh = build_edge_markings(edge, cl, profile, p5::flat_heights(cl, 0.0f), cfg);
    CHECK_TRUE(!mesh.vertices.empty());

    const double outer = 0.5 * 4.0 * 3.75;                         // 7.5
    const double expect = outer - static_cast<double>(cfg.edge_line_inset);  // 7.3

    bool found_left = false;
    bool found_right = false;
    for (const p5::Component& comp : p5::components_of(mesh)) {
        const double lateral = p5::centroid_of(mesh, comp).y;
        CHECK_TRUE(std::fabs(lateral) <= outer + 1e-6);
        if (std::fabs(lateral - expect) <= 0.15) found_left = true;
        if (std::fabs(lateral + expect) <= 0.15) found_right = true;
        for (const glm::dvec2& p : p5::locals_of(mesh, comp)) {
            CHECK_TRUE(std::fabs(p.y) <= outer + 1e-6);
        }
        CHECK_TRUE(is_solid_line(p5::sprite_of(mesh, comp)));
    }
    CHECK_TRUE(found_left);
    CHECK_TRUE(found_right);

    // Lane lines on as well: a motorway's opposing groups are separated by a
    // median, so no boundary of it is ever painted as a centre line and no
    // centre-line sprite may appear.
    const Mesh with_lanes = build_edge_markings(edge, cl, profile, p5::flat_heights(cl, 0.0f),
                                                only(true, true));
    for (const p5::Component& comp : p5::components_of(with_lanes)) {
        const MarkingSprite s = p5::sprite_of(with_lanes, comp);
        CHECK_TRUE(s != MarkingSprite::DashedYellow);
        CHECK_TRUE(s != MarkingSprite::SolidYellow);
        CHECK_TRUE(s != MarkingSprite::DoubleSolidYellow);
    }
}

/**
 * A footway is never painted, whatever its profile holds.
 */
TEST(Markings, footway_and_cycleway_edges_are_never_painted) {
    for (RoadType type : {RoadType::Footway, RoadType::Cycleway, RoadType::Path}) {
        GraphEdge edge = straight_edge(90.0, type, 2, 1, 1);
        const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
        const RoadProfile profile = jt::lane_profile(2, 3.5);
        const Mesh mesh = build_edge_markings(edge, cl, profile, p5::flat_heights(cl, 0.0f),
                                              only(true, true));
        CHECK_EQ(mesh.indices.size(), size_t{0});
    }
}

/**
 * A one-way edge has no centre line, because it has no opposing flow to divide.
 *
 * The lane lines WITHIN the direction of travel are still painted, so an empty
 * mesh would be the wrong answer: the assertion is that no CENTRE-line sprite
 * appears, not that nothing does.
 */
TEST(Markings, one_way_edge_has_lane_lines_but_no_centre_line) {
    GraphEdge edge = straight_edge(140.0, RoadType::Primary, 3, 3, 0, true);
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(3, 3.5);

    TagMap tags;
    tags["overtaking"] = "no";   // would make a centre line solid, if there were one

    const Mesh mesh = build_edge_markings(edge, cl, profile, p5::flat_heights(cl, 0.0f),
                                          only(true, false), &tags);
    CHECK_TRUE(!mesh.vertices.empty());
    for (const p5::Component& comp : p5::components_of(mesh)) {
        const MarkingSprite s = p5::sprite_of(mesh, comp);
        CHECK_TRUE(is_dashed(s));
        CHECK_TRUE(s != MarkingSprite::DashedYellow);
    }
}

// ============================================================================
// Which boundary is the centre line
// ============================================================================

/**
 * The traffic convention decides which boundary divides the two flows.
 *
 * The edge carries one forward lane and two backward ones, so the profile is
 * asymmetric and the dividing boundary is NOT the centreline of the road. Under
 * right-hand traffic the forward group sits on the RIGHT half, so the single
 * forward lane is the rightmost strip and the divider is at lateral -1.75. Under
 * left-hand traffic it is the leftmost strip and the divider is at +1.75.
 *
 * The two are told apart by which boundary is left DASHED. `overtaking=no` makes
 * the centre line solid over the whole edge under either convention, so whatever
 * dashed geometry remains is the ordinary lane line inside the two-lane group,
 * and it sits on the opposite side from the divider. That reading works whether
 * the solid centre line came out yellow, doubled, or as two white runs, which is
 * the part of the colour rule that differs between the conventions.
 */
TEST(Markings, traffic_convention_selects_the_dividing_boundary) {
    GraphEdge edge = straight_edge(120.0, RoadType::Primary, 3, 1, 2);
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(3, 3.5);
    const std::vector<float> heights = p5::flat_heights(cl, 0.0f);

    TagMap tags;
    tags["overtaking"] = "no";

    for (bool left_hand : {false, true}) {
        MarkingConfig cfg = only(true, false);
        cfg.left_hand_traffic = left_hand;
        const Mesh mesh = build_edge_markings(edge, cl, profile, heights, cfg, &tags);
        CHECK_TRUE(!mesh.vertices.empty());

        // The divider is at -1.75 under right-hand traffic, +1.75 under left.
        const double divider = left_hand ? 1.75 : -1.75;
        const double lane_line = -divider;

        size_t dashed = 0;
        size_t solid = 0;
        for (const p5::Component& comp : p5::components_of(mesh)) {
            const MarkingSprite s = p5::sprite_of(mesh, comp);
            const double lateral = p5::centroid_of(mesh, comp).y;
            if (is_dashed(s)) {
                ++dashed;
                CHECK_NEAR(lateral, lane_line, 0.3);
            } else if (is_solid_line(s)) {
                ++solid;
                CHECK_NEAR(lateral, divider, 0.3);
            }
        }
        CHECK_TRUE(dashed > 0);
        CHECK_TRUE(solid > 0);
    }
}

// ============================================================================
// The drift guard
// ============================================================================

/**
 * Every marking vertex follows the miter, and every one lies inside the
 * carriageway.
 *
 * Two assertions against one square corner, and the first is the one that
 * matters. offset_point() is the single definition of the miter in this
 * codebase, so the expected position of a line at lateral L is exactly
 * `offset_point(station, L)` at every station: a marking that agrees with it
 * followed the miter and one that does not, did not. At a right-angle corner an
 * unmitred column at 5 m sits 2.07 m short of the mitred one, which is 8 times
 * the tolerance here.
 *
 * The containment assertion is the second, and on its own it would prove almost
 * nothing: an unmitred line pinches INWARD, so it stays inside the carriageway
 * and passes. It is here because it is the property a reader of this file will
 * expect to see, and because it does catch the opposite error of a line pushed
 * out past the lane edge by a miter applied twice.
 */
TEST(Markings, markings_follow_the_miter_around_a_square_corner) {
    const GraphEdge edge = corner_edge(RoadType::Primary, 4, 2, 2);
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(4, 3.5);
    CHECK_TRUE(cl.is_valid());

    TagMap tags;
    tags["overtaking"] = "no";   // solid lines are per station band, so they carry corner vertices

    MarkingConfig cfg = only(true, true);
    const Mesh mesh = build_edge_markings(edge, cl, profile, p5::flat_heights(cl, 0.0f), cfg,
                                          &tags);
    CHECK_TRUE(!mesh.vertices.empty());

    // Every lateral a line may legitimately sit on: the three Lane-to-Lane
    // boundaries of a four-lane profile, and the two inset edge lines.
    const double outer = 0.5 * 4.0 * 3.5;                                   // 7.0
    const double inset = outer - static_cast<double>(cfg.edge_line_inset);  // 6.8
    const std::vector<double> laterals = {inset, 3.5, 0.0, -3.5, -inset};

    // The expected mitred track of each of them, one polyline per lateral.
    std::vector<std::vector<glm::dvec2>> tracks;
    tracks.reserve(laterals.size());
    for (double lateral : laterals) {
        std::vector<glm::dvec2> track;
        track.reserve(cl.stations.size());
        for (const auto& station : cl.stations) {
            track.push_back(offset_point(station, lateral));
        }
        tracks.push_back(std::move(track));
    }

    // A quad is at most half the widest line sprite off its own track, plus room
    // for a double line's outer run. Far below the 2.07 m an unmitred corner
    // would produce.
    const double allowance = 0.25;

    const std::vector<jt::Tri2D> lane_surface =
        p5::triangles_with(build_corridor(cl, profile, CorridorConfig{}).mesh,
                           MaterialId::Asphalt);
    CHECK_TRUE(!lane_surface.empty());

    size_t off_track = 0;
    size_t outside = 0;
    for (const auto& v : mesh.vertices) {
        const glm::dvec2 p = jt::world_to_local(v.position);

        double nearest = 1e300;
        for (const std::vector<glm::dvec2>& track : tracks) {
            for (size_t i = 0; i + 1 < track.size(); ++i) {
                nearest = std::min(nearest, jt::point_segment_distance(p, track[i], track[i + 1]));
            }
        }
        if (nearest > allowance) ++off_track;
        if (!jt::covered_in_plan(lane_surface, p)) ++outside;
    }
    CHECK_EQ(off_track, size_t{0});
    CHECK_EQ(outside, size_t{0});
}

// ============================================================================
// Approach markings from a real fixture
// ============================================================================

namespace {

/**
 * @brief tests/data/turn_lanes.osm, taken to the point of painting its approach
 *
 * The trims are set by hand rather than solved. The junction solver is P4's
 * contract and is tested in its own suites; coupling every arrow expectation to
 * its output would mean a change to the fillet radius moved every number here.
 * What build_approach_markings() needs from the trim is only a cut station, and
 * 10 m is a plausible one for a four-arm signalled junction of primaries.
 */
struct Approach {
    p5::Network net;
    stratum::osm::road::EdgeId edge_id = stratum::osm::road::kInvalidId;
    GraphEdge edge;
    double cut = 0.0;
    bool ok = false;
};

Approach load_approach(double trim = 10.0) {
    Approach a;
    a.net = p5::make_network("turn_lanes.osm");
    if (!a.net.ok) return a;

    a.edge_id = p5::sole_edge_of_way(a.net.graph, 12000);
    if (a.edge_id == stratum::osm::road::kInvalidId) {
        stratum::test::report_failure(__FILE__, __LINE__, "way 12000 is one edge",
                                      "the approach split unexpectedly");
        return a;
    }
    a.edge = a.net.graph.edge(a.edge_id);
    a.edge.trim_to = trim;
    a.cut = a.net.centerlines[a.edge_id].length() - trim;
    a.ok = true;
    return a;
}

} // namespace

/**
 * The approach is the way's `to` end, and the junction there is signalled.
 *
 * A precondition rather than a result, asserted on its own so that a failure in
 * the fixture reads as a fixture failure instead of as an arrow in the wrong
 * place.
 */
TEST(Markings, turn_lanes_fixture_approaches_a_signalled_four_way) {
    const Approach a = load_approach();
    if (!a.ok) return;

    CHECK_TRUE(a.edge.to != stratum::osm::road::kInvalidId);
    const auto& node = a.net.graph.node(a.edge.to);
    CHECK_EQ(node.osm_id, stratum::osm::NodeId{1203});
    CHECK_EQ(node.degree(), size_t{4});
    CHECK_TRUE(node.has_signals);
    CHECK_TRUE(a.edge.is_oneway);
    CHECK_EQ(a.edge.lanes, 3);

    // Three Lane strips, so the turn:lanes entry count matches the profile.
    size_t lanes = 0;
    for (const auto& strip : a.net.profiles[a.edge_id].strips) {
        if (strip.kind == stratum::osm::road::StripKind::Lane) ++lanes;
    }
    CHECK_EQ(lanes, size_t{3});
}

/**
 * Three arrows, left to right, matching turn:lanes=left|through|through;right.
 *
 * The order is the whole test. `turn:lanes` values are ordered left to right in
 * the direction of travel under both traffic conventions, so the leftmost arrow
 * is the left-turn one; an emitter that walked the profile right to left would
 * produce three plausible arrows instructing every driver to do the opposite of
 * what the data says.
 *
 * The arrows are also checked to be upstream of the cut, which catches the other
 * half of the same class of bug: an approach painted on the exit.
 */
TEST(Markings, turn_lanes_produces_three_arrows_in_left_to_right_order) {
    const Approach a = load_approach();
    if (!a.ok) return;

    const Centerline& cl = a.net.centerlines[a.edge_id];
    const RoadProfile& profile = a.net.profiles[a.edge_id];

    MarkingConfig cfg;
    cfg.emit_lane_lines = false;
    cfg.emit_edge_lines = false;
    cfg.emit_stop_lines = false;
    cfg.emit_arrows = true;

    const Mesh mesh = build_approach_markings(a.edge, cl, profile, /*at_start*/ false,
                                              /*has_signals*/ true, 0.0f, cfg,
                                              p5::tags_of(a.net.data, a.edge));

    std::vector<p5::Component> comps = p5::components_of(mesh);
    CHECK_EQ(comps.size(), size_t{3});
    if (comps.size() != 3) return;

    // Left of travel is the positive lateral, so descending lateral is left to
    // right across the carriageway.
    std::sort(comps.begin(), comps.end(),
              [&](const p5::Component& x, const p5::Component& y) {
                  return p5::lateral_of(cl, p5::centroid_of(mesh, x)) >
                         p5::lateral_of(cl, p5::centroid_of(mesh, y));
              });

    const MarkingSprite expected[3] = {
        MarkingSprite::ArrowLeft,
        MarkingSprite::ArrowStraight,
        MarkingSprite::ArrowStraightRight,
    };

    double left_edge = 0.0;
    double right_edge = 0.0;
    const bool has_lanes = p5::lane_span(profile, left_edge, right_edge);
    CHECK_TRUE(has_lanes);

    for (size_t i = 0; i < 3; ++i) {
        const MarkingSprite got = p5::sprite_of(mesh, comps[i]);
        if (got != expected[i]) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "arrow sprite matches turn:lanes entry",
                std::string("lane ") + std::to_string(i) + ": actual " + p5::sprite_name(got) +
                    "  expected " + p5::sprite_name(expected[i]));
        }
        CHECK_EQ(comps[i].material, MaterialId::Markings);

        for (const glm::dvec2& p : p5::locals_of(mesh, comps[i])) {
            const double lateral = p5::lateral_of(cl, p);
            CHECK_TRUE(lateral <= left_edge + 1e-3);
            CHECK_TRUE(lateral >= right_edge - 1e-3);

            // Upstream of the cut, and roughly arrow_spacing back from it. The
            // window is wide because the exact anchor -- arrow tip or arrow
            // centre, measured from the stop line's near or far edge -- is not
            // pinned by the header; being on the correct SIDE of the cut is.
            const double arc = p5::arclength_of(cl, p);
            CHECK_TRUE(arc < a.cut);
            CHECK_TRUE(arc > a.cut - 40.0);
            CHECK_TRUE(arc < a.cut - 20.0);
        }
    }
}

/**
 * The stop line at a signalled junction, one quad per approaching lane.
 *
 * Each spans its own lane laterally and MarkingConfig::stop_line_width along the
 * road, with its downstream edge stop_line_setback back from the cut. The
 * lateral extent is the assertion that says the line stops at the lane rather
 * than running the whole carriageway as one bar, which is what makes a
 * per-lane stop line usable when only one lane of an approach is stopped.
 */
TEST(Markings, signalled_approach_gets_one_stop_line_quad_per_lane) {
    const Approach a = load_approach();
    if (!a.ok) return;

    const Centerline& cl = a.net.centerlines[a.edge_id];
    const RoadProfile& profile = a.net.profiles[a.edge_id];

    MarkingConfig cfg;
    cfg.emit_lane_lines = false;
    cfg.emit_edge_lines = false;
    cfg.emit_stop_lines = true;
    cfg.emit_arrows = false;

    const Mesh mesh = build_approach_markings(a.edge, cl, profile, false, true, 0.0f, cfg,
                                              p5::tags_of(a.net.data, a.edge));
    const std::vector<p5::Component> comps = p5::components_of(mesh);
    CHECK_EQ(comps.size(), size_t{3});

    for (const p5::Component& comp : comps) {
        CHECK_EQ(p5::sprite_of(mesh, comp), MarkingSprite::StopLine);

        double lo = 1e300;
        double hi = -1e300;
        double arc_lo = 1e300;
        double arc_hi = -1e300;
        for (const glm::dvec2& p : p5::locals_of(mesh, comp)) {
            const double lateral = p5::lateral_of(cl, p);
            lo = std::min(lo, lateral);
            hi = std::max(hi, lateral);
            const double arc = p5::arclength_of(cl, p);
            arc_lo = std::min(arc_lo, arc);
            arc_hi = std::max(arc_hi, arc);
        }
        CHECK_NEAR(hi - lo, 3.5, 0.05);                      // one lane, not the carriageway
        CHECK_NEAR(arc_hi - arc_lo, cfg.stop_line_width, 0.05);
        CHECK_NEAR(arc_hi, a.cut - cfg.stop_line_setback, 0.05);
    }
}

/**
 * An unsignalled junction gets give-way triangles instead of a stop bar.
 */
TEST(Markings, priority_approach_gets_give_way_triangles) {
    const Approach a = load_approach();
    if (!a.ok) return;

    MarkingConfig cfg;
    cfg.emit_lane_lines = false;
    cfg.emit_edge_lines = false;
    cfg.emit_stop_lines = true;
    cfg.emit_arrows = false;

    const Mesh mesh = build_approach_markings(a.edge, a.net.centerlines[a.edge_id],
                                              a.net.profiles[a.edge_id], false,
                                              /*has_signals*/ false, 0.0f, cfg,
                                              p5::tags_of(a.net.data, a.edge));
    const std::vector<p5::Component> comps = p5::components_of(mesh);
    CHECK_TRUE(comps.size() >= 3);
    for (const p5::Component& comp : comps) {
        CHECK_EQ(p5::sprite_of(mesh, comp), MarkingSprite::GiveWayTriangles);
    }
}

/**
 * A one-way edge approached from its exit end paints nothing.
 *
 * Way 12000 is one way towards node 1203, so approaching it at its `from` end
 * means approaching against the flow: there is no backward lane group and
 * therefore nothing to stop or to instruct. An emitter that painted the forward
 * group anyway would put a stop line facing the wrong way down a one-way street.
 */
TEST(Markings, one_way_approached_from_its_exit_paints_nothing) {
    Approach a = load_approach();
    if (!a.ok) return;

    a.edge.trim_from = 10.0;
    MarkingConfig cfg;
    cfg.emit_lane_lines = false;
    cfg.emit_edge_lines = false;

    const Mesh mesh = build_approach_markings(a.edge, a.net.centerlines[a.edge_id],
                                              a.net.profiles[a.edge_id], /*at_start*/ true,
                                              true, 0.0f, cfg,
                                              p5::tags_of(a.net.data, a.edge));
    CHECK_EQ(mesh.indices.size(), size_t{0});
}

// ============================================================================
// Bad turn:lanes data
// ============================================================================

/**
 * A turn:lanes entry count that disagrees with the profile paints no arrow.
 *
 * There is no safe recovery. Shifting the entries by one paints "left only" over
 * a lane that goes straight on, and truncating them paints the first two of
 * three onto lanes that may be any two. So the whole approach goes unarrowed and
 * the mismatch is logged.
 *
 * Both halves of the test have to be here. The matching tag proving that arrows
 * DO appear for this edge is what makes the empty result above evidence of the
 * mismatch rule rather than of a broken setup, and the containment check is what
 * says no arrow landed on a lane the profile does not have.
 */
TEST(Markings, turn_lanes_with_a_mismatched_count_paints_no_arrow) {
    GraphEdge edge = straight_edge(120.0, RoadType::Primary, 2, 2, 0, true);
    edge.trim_to = 5.0;
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(2, 3.5);

    MarkingConfig cfg;
    cfg.emit_lane_lines = false;
    cfg.emit_edge_lines = false;
    cfg.emit_stop_lines = false;
    cfg.emit_arrows = true;

    TagMap too_many;
    too_many["turn:lanes"] = "left|through|through|right";
    const Mesh none = build_approach_markings(edge, cl, profile, false, true, 0.0f, cfg,
                                              &too_many);
    CHECK_EQ(none.indices.size(), size_t{0});

    TagMap too_few;
    too_few["turn:lanes"] = "left";
    const Mesh also_none = build_approach_markings(edge, cl, profile, false, true, 0.0f, cfg,
                                                   &too_few);
    CHECK_EQ(also_none.indices.size(), size_t{0});

    TagMap matching;
    matching["turn:lanes"] = "left|through";
    const Mesh painted = build_approach_markings(edge, cl, profile, false, true, 0.0f, cfg,
                                                 &matching);
    const std::vector<p5::Component> comps = p5::components_of(painted);
    CHECK_EQ(comps.size(), size_t{2});

    double left_edge = 0.0;
    double right_edge = 0.0;
    CHECK_TRUE(p5::lane_span(profile, left_edge, right_edge));
    for (const p5::Component& comp : comps) {
        for (const glm::dvec2& p : p5::locals_of(painted, comp)) {
            CHECK_TRUE(p.y <= left_edge + 1e-3);
            CHECK_TRUE(p.y >= right_edge - 1e-3);
        }
    }
}

/**
 * A plain `turn:lanes` on a TWO-WAY way paints no arrow on either approach.
 *
 * The undirected key describes the way, not an approach, and on a two-way way
 * there is no way to tell which of the two it was meant for. The entry-count
 * check catches only the well-formed case: a value per lane of the whole way
 * will not match one direction's group. It does not catch this fixture, which is
 * the one that does damage -- `lanes=4` with two values, matching a half group
 * exactly, so the FORWARD instructions get painted onto the opposing carriageway
 * at the far junction. A driver reading a left-turn arrow laid in the lane
 * coming the other way is the one failure this family of tags must not produce.
 *
 * The control is that the DIRECTIONAL key is still honoured on the same edge, so
 * the empty result above is evidence of the rule and not of a broken fixture.
 */
TEST(Markings, a_plain_turn_lanes_paints_no_arrow_on_a_two_way_approach) {
    GraphEdge edge = straight_edge(120.0, RoadType::Primary, 4, 2, 2, /*oneway=*/false);
    edge.trim_from = 5.0;
    edge.trim_to = 5.0;
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(4, 3.5);
    CHECK_TRUE(cl.is_valid());

    MarkingConfig cfg;
    cfg.emit_lane_lines = false;
    cfg.emit_edge_lines = false;
    cfg.emit_stop_lines = false;
    cfg.emit_arrows = true;

    // Two values on a four-lane way: exactly one carriageway's worth, so the
    // count check passes for BOTH groups.
    TagMap plain;
    plain["turn:lanes"] = "left|through";

    const Mesh at_to = build_approach_markings(edge, cl, profile, false, true, 0.0f, cfg, &plain);
    CHECK_EQ(at_to.indices.size(), size_t{0});

    const Mesh at_from = build_approach_markings(edge, cl, profile, true, true, 0.0f, cfg, &plain);
    CHECK_EQ(at_from.indices.size(), size_t{0});

    // The control: the directional key still paints, and only on its own end.
    TagMap directional;
    directional["turn:lanes:forward"] = "left|through";
    const Mesh painted =
        build_approach_markings(edge, cl, profile, false, true, 0.0f, cfg, &directional);
    CHECK_EQ(p5::components_of(painted).size(), size_t{2});

    const Mesh other_end =
        build_approach_markings(edge, cl, profile, true, true, 0.0f, cfg, &directional);
    CHECK_EQ(other_end.indices.size(), size_t{0});

    // A one-way way keeps the plain key: there it names the only direction there
    // is, and turn_lanes.osm depends on it.
    GraphEdge single = straight_edge(120.0, RoadType::Primary, 2, 2, 0, /*oneway=*/true);
    single.trim_to = 5.0;
    const RoadProfile two_lane = jt::lane_profile(2, 3.5);
    TagMap oneway_plain;
    oneway_plain["turn:lanes"] = "left|through";
    const Mesh one_way_painted =
        build_approach_markings(single, cl, two_lane, false, true, 0.0f, cfg, &oneway_plain);
    CHECK_EQ(p5::components_of(one_way_painted).size(), size_t{2});
}

/**
 * An approach follows the corridor, not the junction plane it starts from.
 *
 * The junction solver flattens each arm mouth onto the junction plane over the
 * TRIM, and no further. An approach reaches far outside that plateau: a turn
 * arrow sits MarkingConfig::arrow_spacing -- 25 m by default -- behind the stop
 * line, which on a 6% grade is a metre and a half of climb. Emitted on the one
 * junction plane, the arrows end up buried under the carriageway and are never
 * seen, while the stop line beside them is right because it is inside the
 * plateau.
 *
 * So the whole approach is emitted against the same per-station heights the
 * corridor was placed at. Inside the plateau those heights ARE the junction
 * plane, which is why the near elements still land on it.
 */
TEST(Markings, an_approach_follows_the_corridor_rather_than_the_junction_plane) {
    GraphEdge edge = straight_edge(120.0, RoadType::Primary, 3, 3, 0, /*oneway=*/true);
    edge.trim_to = 5.0;
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(3, 3.5);
    CHECK_TRUE(cl.is_valid());
    if (!cl.is_valid()) return;

    const double node_height = 100.0;
    const double grade = 0.06;
    const double back = cl.stations.back().arclength;

    // The solved grade, then the plateau road_network_builder.cpp stage 3d lays
    // over the trim, reproduced here so the test states the input it depends on.
    std::vector<float> heights(cl.stations.size(), 0.0f);
    for (size_t j = 0; j < heights.size(); ++j) {
        heights[j] =
            static_cast<float>(node_height - grade * (back - cl.stations[j].arclength));
    }
    for (size_t j = heights.size(); j-- > 0;) {
        heights[j] = static_cast<float>(node_height);
        if (back - cl.stations[j].arclength >= edge.trim_to) break;
    }

    /// The corridor surface at one arclength, the way the extruder interpolates it
    const auto surface_at = [&](double at) {
        const std::vector<stratum::osm::road::Station>& st = cl.stations;
        if (at <= st.front().arclength) return static_cast<double>(heights.front());
        if (at >= st.back().arclength) return static_cast<double>(heights.back());
        for (size_t j = 1; j < st.size(); ++j) {
            if (st[j].arclength < at) continue;
            const double lo = st[j - 1].arclength;
            const double hi = st[j].arclength;
            const double t = (hi - lo > 1e-9) ? (at - lo) / (hi - lo) : 0.0;
            return static_cast<double>(heights[j - 1]) +
                   t * (static_cast<double>(heights[j]) - static_cast<double>(heights[j - 1]));
        }
        return static_cast<double>(heights.back());
    };

    MarkingConfig cfg;
    cfg.emit_lane_lines = false;
    cfg.emit_edge_lines = false;
    cfg.emit_stop_lines = true;
    cfg.emit_arrows = true;

    TagMap tags;
    tags["turn:lanes"] = "left|through|through";

    const Mesh mesh = build_approach_markings(edge, cl, profile, false, true,
                                              static_cast<float>(node_height), cfg, &tags,
                                              &heights);
    CHECK_TRUE(!mesh.indices.empty());
    if (mesh.indices.empty()) return;

    // Every vertex sits on the corridor, to the millimetre. The edge runs due
    // east, so the local x coordinate IS the arclength.
    const double lift = static_cast<double>(cfg.height_above_surface);
    double lowest = 1e30;
    for (const auto& v : mesh.vertices) {
        const glm::dvec2 local = jt::world_to_local(v.position);
        const double expected = surface_at(local.x) + lift;
        CHECK_NEAR(static_cast<double>(v.position.y), expected, 1e-3);
        lowest = std::min(lowest, static_cast<double>(v.position.y));
    }

    // The premise, stated so the test cannot pass by emitting nothing far back:
    // the arrows really are more than a metre below the junction plane.
    CHECK_TRUE(lowest < node_height - 1.0);

    // And the stop line, which IS inside the plateau, is still on the junction
    // plane -- the property the flat emitter was written for.
    double highest = -1e30;
    for (const auto& v : mesh.vertices) {
        highest = std::max(highest, static_cast<double>(v.position.y));
    }
    CHECK_NEAR(highest, node_height + lift, 1e-3);
}

/**
 * No turn:lanes tag at all, and no tag map at all, paint no arrow.
 *
 * An invented arrow is a false instruction, so the absence of the tag is not a
 * licence to assume the lane goes straight on.
 */
TEST(Markings, no_turn_lanes_tag_paints_no_arrow) {
    GraphEdge edge = straight_edge(120.0, RoadType::Primary, 2, 2, 0, true);
    edge.trim_to = 5.0;
    const Centerline cl = build_centerline(edge.polyline, jt::fixture_resample());
    const RoadProfile profile = jt::lane_profile(2, 3.5);

    MarkingConfig cfg;
    cfg.emit_lane_lines = false;
    cfg.emit_edge_lines = false;
    cfg.emit_stop_lines = false;
    cfg.emit_arrows = true;

    CHECK_EQ(build_approach_markings(edge, cl, profile, false, true, 0.0f, cfg, nullptr)
                 .indices.size(),
             size_t{0});

    TagMap unrelated;
    unrelated["maxspeed"] = "50";
    CHECK_EQ(build_approach_markings(edge, cl, profile, false, true, 0.0f, cfg, &unrelated)
                 .indices.size(),
             size_t{0});
}

// ============================================================================
// The atlas table
// ============================================================================

namespace {

/// One row of the pixel layout documented at the top of marking_atlas.hpp
struct AtlasRow {
    MarkingSprite sprite;
    int x0;
    int x1;    ///< exclusive
    int y0;
    int y1;    ///< exclusive
    float width_m;
    float length_m;
};

/**
 * @brief The frozen sprite table, transcribed from marking_atlas.hpp
 *
 * Deliberately a second copy rather than a call into the thing under test. The
 * atlas is a DATA CONTRACT an artist authors a texture against, so the numbers
 * have to be pinned somewhere the implementation cannot move them from, and a
 * test that asked sprite_rect() what sprite_rect() returns would pin nothing.
 */
const AtlasRow kAtlasTable[] = {
    // Band A: longitudinal lines, 64 x 128 blocks on rows 0 and 1.
    {MarkingSprite::DashWhite,            0,   64,    0,  128, 0.15f, 3.0f},
    {MarkingSprite::DashLongWhite,       64,  128,    0,  128, 0.15f, 6.0f},
    {MarkingSprite::SolidWhite,         128,  192,    0,  128, 0.15f, 1.0f},
    {MarkingSprite::SolidYellow,        192,  256,    0,  128, 0.15f, 1.0f},
    {MarkingSprite::DoubleSolidYellow,  256,  320,    0,  128, 0.35f, 1.0f},
    {MarkingSprite::DashedYellow,       320,  384,    0,  128, 0.15f, 3.0f},
    // Band B: transverse and area markings, 128 x 128 blocks on rows 2 and 3.
    {MarkingSprite::StopLine,             0,  128,  128,  256, 1.0f,  0.4f},
    {MarkingSprite::GiveWayTriangles,   128,  256,  128,  256, 0.6f,  0.6f},
    {MarkingSprite::ZebraStripe,        256,  384,  128,  256, 0.5f,  3.0f},
    {MarkingSprite::BoxJunctionHatch,   384,  512,  128,  256, 2.0f,  2.0f},
    // Band C: turn arrows, 128 x 384 blocks on rows 4 to 9.
    {MarkingSprite::ArrowStraight,        0,  128,  256,  640, 1.4f,  4.2f},
    {MarkingSprite::ArrowLeft,          128,  256,  256,  640, 1.4f,  4.2f},
    {MarkingSprite::ArrowRight,         256,  384,  256,  640, 1.4f,  4.2f},
    {MarkingSprite::ArrowStraightLeft,  384,  512,  256,  640, 1.4f,  4.2f},
    {MarkingSprite::ArrowStraightRight, 512,  640,  256,  640, 1.4f,  4.2f},
    {MarkingSprite::ArrowUTurn,         640,  768,  256,  640, 1.4f,  4.2f},
    // Band D: lane symbols, 128 x 256 blocks on rows 10 to 13.
    {MarkingSprite::BikeSymbol,           0,  128,  640,  896, 1.0f,  2.0f},
    {MarkingSprite::BusSymbol,          128,  256,  640,  896, 1.0f,  2.0f},
};

} // namespace

/**
 * Every sprite rect is the documented block, inset by kAtlasInsetPixels.
 *
 * There is no marking texture in the repository, so this is the only thing that
 * holds the two halves of the contract together: the emitter generates UVs
 * against these numbers and an artist will one day paint against them. A rect
 * that moved by one cell would show as every arrow rendering as its neighbour,
 * which is not a failure anyone would attribute to a UV table.
 */
TEST(Markings, sprite_rects_match_the_frozen_pixel_layout) {
    const float size = static_cast<float>(stratum::osm::road::kAtlasSizePixels);
    const float inset = stratum::osm::road::kAtlasInsetPixels;

    for (const AtlasRow& row : kAtlasTable) {
        const SpriteRect got = sprite_rect(row.sprite);
        const std::string label = std::string("sprite_rect(") +
                                  stratum::osm::road::marking_sprite_name(row.sprite) + ")";

        const float want_u0 = (static_cast<float>(row.x0) + inset) / size;
        const float want_u1 = (static_cast<float>(row.x1) - inset) / size;
        const float want_v0 = (static_cast<float>(row.y0) + inset) / size;
        const float want_v1 = (static_cast<float>(row.y1) - inset) / size;

        if (std::fabs(got.u0 - want_u0) > 1e-6f || std::fabs(got.u1 - want_u1) > 1e-6f ||
            std::fabs(got.v0 - want_v0) > 1e-6f || std::fabs(got.v1 - want_v1) > 1e-6f) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "sprite rect matches the frozen layout",
                label + ": actual (" + std::to_string(got.u0) + ", " + std::to_string(got.v0) +
                    ", " + std::to_string(got.u1) + ", " + std::to_string(got.v1) +
                    ")  expected (" + std::to_string(want_u0) + ", " + std::to_string(want_v0) +
                    ", " + std::to_string(want_u1) + ", " + std::to_string(want_v1) + ")");
        }

        const SpriteSize got_size = sprite_size(row.sprite);
        CHECK_NEAR(got_size.width_m, row.width_m, 1e-6);
        CHECK_NEAR(got_size.length_m, row.length_m, 1e-6);
    }

    // The table above must cover every sprite. A new enumerator with no row here
    // would otherwise go unchecked.
    CHECK_EQ(sizeof(kAtlasTable) / sizeof(kAtlasTable[0]),
             static_cast<size_t>(MarkingSprite::Count));
}

/**
 * Every rect is normalised, non-empty, and disjoint from every other.
 *
 * Independent of the table above: this is the property that has to hold whatever
 * the layout is, and it is the one that fails if a future sprite is dropped into
 * a free run that turns out not to be free.
 */
TEST(Markings, sprite_rects_are_in_range_and_never_overlap) {
    const auto count = static_cast<uint8_t>(MarkingSprite::Count);

    for (uint8_t i = 0; i < count; ++i) {
        const auto s = static_cast<MarkingSprite>(i);
        const SpriteRect r = sprite_rect(s);
        CHECK_TRUE(r.u0 >= 0.0f);
        CHECK_TRUE(r.v0 >= 0.0f);
        CHECK_TRUE(r.u1 <= 1.0f);
        CHECK_TRUE(r.v1 <= 1.0f);
        CHECK_TRUE(r.u0 < r.u1);
        CHECK_TRUE(r.v0 < r.v1);
        CHECK_TRUE(r.du() > 0.0f);
        CHECK_TRUE(r.dv() > 0.0f);

        const SpriteSize sz = sprite_size(s);
        CHECK_TRUE(sz.width_m > 0.0f);
        CHECK_TRUE(sz.length_m > 0.0f);

        CHECK_TRUE(std::string(stratum::osm::road::marking_sprite_name(s)) != "Unknown");
    }

    for (uint8_t i = 0; i < count; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1); j < count; ++j) {
            const SpriteRect a = sprite_rect(static_cast<MarkingSprite>(i));
            const SpriteRect b = sprite_rect(static_cast<MarkingSprite>(j));
            const bool disjoint = a.u1 <= b.u0 || b.u1 <= a.u0 || a.v1 <= b.v0 || b.v1 <= a.v0;
            if (!disjoint) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "sprite rects are disjoint",
                    std::string(stratum::osm::road::marking_sprite_name(
                        static_cast<MarkingSprite>(i))) +
                        " overlaps " +
                        stratum::osm::road::marking_sprite_name(static_cast<MarkingSprite>(j)));
            }
        }
    }
}

/**
 * The sentinel and out-of-range values produce a degenerate rect, not a
 * neighbour's.
 *
 * A quad mapping {0, 0, 0, 0} collapses to a point and disappears. A quad
 * mapping whatever happened to be at index Count renders as a real marking of
 * the wrong kind, which is far worse and completely silent.
 */
TEST(Markings, out_of_range_sprites_are_degenerate_rather_than_aliased) {
    const SpriteRect sentinel = sprite_rect(MarkingSprite::Count);
    CHECK_NEAR(sentinel.u0, 0.0, 1e-9);
    CHECK_NEAR(sentinel.v0, 0.0, 1e-9);
    CHECK_NEAR(sentinel.u1, 0.0, 1e-9);
    CHECK_NEAR(sentinel.v1, 0.0, 1e-9);

    const SpriteSize sentinel_size = sprite_size(MarkingSprite::Count);
    CHECK_NEAR(sentinel_size.width_m, 0.0, 1e-9);
    CHECK_NEAR(sentinel_size.length_m, 0.0, 1e-9);

    const auto beyond = static_cast<MarkingSprite>(static_cast<uint8_t>(MarkingSprite::Count) + 7);
    const SpriteRect out = sprite_rect(beyond);
    CHECK_NEAR(out.u0, 0.0, 1e-9);
    CHECK_NEAR(out.u1, 0.0, 1e-9);
    CHECK_TRUE(std::string(stratum::osm::road::marking_sprite_name(beyond)) == "Unknown");
}
