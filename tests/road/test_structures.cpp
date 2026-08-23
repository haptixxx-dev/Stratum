/**
 * @file test_structures.cpp
 * @brief Bridge decks, parapets, piers, and the tunnel portals at the other end
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * P6 is the one phase whose output is routinely seen from BELOW. A road surface
 * with an inside-out triangle reads as a road; a bridge underside with one reads
 * as a hole in the deck from every angle a player ever drives under it. So this
 * suite spends more of its assertions on winding and on closure than any other
 * in the tree.
 *
 * The four that carry the most weight:
 *
 * 1. **The deck top is not emitted twice.** The corridor already owns the
 *    running surface for the whole bridge edge. A structure builder that emitted
 *    its own copy would z-fight across the entire span, and the artefact appears
 *    and disappears with the camera distance, which is the hardest kind of
 *    z-fighting to attribute to its cause.
 * 2. **The underside faces down.** Stated explicitly in bridge_builder.hpp, and
 *    the one winding rule that can be checked without knowing how the slab was
 *    tessellated. The world mapping `(x, y) -> (x, h, -y)` flips handedness, so
 *    a builder that derived its winding from the 2D ring gets exactly this
 *    surface backwards.
 * 3. **A tunnel portal is found, not assumed.** A `tunnel=*` way starts on open
 *    ground and enters the hillside some way along. tests/data/tunnel.osm puts
 *    140 m of ridge under 200 m of covered way precisely so that the portal
 *    station and the edge endpoint are 35 m apart and a builder that used the
 *    endpoint is caught.
 * 4. **The opening is a hole.** No plan-view predicate can tell a portal with a
 *    hole in it from one without, because a headwall projects to a line in plan.
 *    The only test that works is to fire a segment along the carriageway and
 *    check that it hits nothing.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests Structures
 * @endcode
 */

#include "framework.hpp"
#include "road/p5_p6_fixtures.hpp"

#include "osm/road/bridge_builder.hpp"
#include "osm/road/centerline.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/road/tunnel_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::osm::NodeId;
using stratum::osm::road::BridgeConfig;
using stratum::osm::road::Centerline;
using stratum::osm::road::EdgeId;
using stratum::osm::road::EdgeElevation;
using stratum::osm::road::ElevationConfig;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::GraphNodeId;
using stratum::osm::road::RoadElevationSolver;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::TunnelConfig;
using stratum::osm::road::TunnelPortalFootprint;
using stratum::osm::road::build_bridge;
using stratum::osm::road::build_tunnel_portals;
using stratum::osm::road::max_grade_for;
using stratum::osm::road::kInvalidId;

namespace p5 = stratum::test::p5;
namespace jt = stratum::test::junction;

using Sampler = std::function<float(double, double)>;

// ============================================================================
// A structure edge, solved and ready to build
// ============================================================================

/**
 * @brief One bridge or tunnel edge with everything build_bridge() needs
 *
 * The centerline is passed UNTRIMMED because every structure edge in the
 * fixtures meets its neighbours at degree-2 nodes, which the junction solver
 * does not trim. Handing a slice() of an untrimmed centerline would be the same
 * geometry and one more moving part.
 */
struct Structure {
    p5::Network net;
    RoadElevationSolver elevation;
    EdgeId edge_id = kInvalidId;
    Sampler terrain;
    bool ok = false;

    [[nodiscard]] const GraphEdge& edge() const { return net.graph.edge(edge_id); }
    [[nodiscard]] const Centerline& cl() const { return net.centerlines[edge_id]; }
    [[nodiscard]] const RoadProfile& profile() const { return net.profiles[edge_id]; }
    [[nodiscard]] const std::vector<float>& heights() const {
        return elevation.edge(edge_id).station_heights;
    }
};

/// The sole EdgeId whose is_bridge or is_tunnel flag is set
EdgeId sole_structure_edge(const stratum::osm::road::RoadGraph& graph, bool want_bridge) {
    EdgeId found = kInvalidId;
    size_t count = 0;
    for (size_t i = 0; i < graph.edges().size(); ++i) {
        const bool is = want_bridge ? graph.edges()[i].is_bridge : graph.edges()[i].is_tunnel;
        if (is) {
            found = static_cast<EdgeId>(i);
            ++count;
        }
    }
    return count == 1 ? found : kInvalidId;
}

/**
 * @brief Parse a fixture, solve its elevation against @p terrain, find its structure edge
 *
 * @param filename    Fixture in tests/data
 * @param want_bridge True to look for the bridge edge, false for the tunnel edge
 * @param terrain     Terrain sampler used for BOTH the elevation solve and the
 *                    structure build. Using two different ones is how a pier ends
 *                    up hovering.
 * @return The structure; `ok` false when the fixture did not parse or carried no
 *         single structure edge
 */
Structure load(const char* filename, bool want_bridge, const Sampler& terrain) {
    Structure s;
    s.net = p5::make_network(filename);
    if (!s.net.ok) return s;

    s.edge_id = sole_structure_edge(s.net.graph, want_bridge);
    if (s.edge_id == kInvalidId) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture has one structure edge",
                                      std::string(filename));
        return s;
    }
    s.terrain = terrain;
    s.elevation.solve(s.net.graph, s.net.centerlines, terrain, ElevationConfig{});
    if (!s.elevation.is_solved()) {
        stratum::test::report_failure(__FILE__, __LINE__, "elevation solved", std::string(filename));
        return s;
    }
    s.ok = true;
    return s;
}

/// tests/data/bridge_abutment.osm over flat ground at 20 m
Structure load_bridge(float ground = 20.0f) {
    return load("bridge_abutment.osm", true, p5::flat_sampler(ground));
}

/// World Y of the running surface nearest a plan position
double surface_at(const Structure& s, const glm::dvec2& p) {
    const std::vector<float>& h = s.heights();
    if (h.empty()) return 0.0;
    const size_t i = std::min(p5::nearest_station(s.cl(), p), h.size() - 1);
    return static_cast<double>(h[i]);
}

/// Lowest and highest running-surface height over the whole edge
void surface_range(const Structure& s, double& lo, double& hi) {
    lo = 1e300;
    hi = -1e300;
    for (float h : s.heights()) {
        lo = std::min(lo, static_cast<double>(h));
        hi = std::max(hi, static_cast<double>(h));
    }
}

/// Components of a mesh whose material is exactly @p material
std::vector<p5::Component> components_with(const Mesh& mesh, MaterialId material) {
    std::vector<p5::Component> out;
    for (const p5::Component& comp : p5::components_of(mesh)) {
        if (comp.material == material) out.push_back(comp);
    }
    return out;
}

/**
 * @brief One pier: every Concrete face sharing a plan footprint, merged
 *
 * See the note in the pier test for why plan position and not vertex
 * connectivity is the relation that makes a pier one thing.
 */
struct Pier {
    double lowest = 1e300;
    double highest = -1e300;
    double x_lo = 1e300;
    double x_hi = -1e300;
    double y_lo = 1e300;
    double y_hi = -1e300;
};

/**
 * @brief Group a mesh's MaterialId::Concrete geometry into piers by footprint
 *
 * Two Concrete components belong to the same pier when their PLAN BOUNDING BOXES
 * overlap, merged transitively. That relation needs no distance threshold and no
 * knowledge of how the box was tessellated: the six faces of one stub all lie on
 * or inside its own square footprint and so all overlap each other's boxes
 * directly or through the cap, while two stubs are a bay apart -- more than ten
 * metres on any real span -- and never do.
 *
 * @param mesh Structure mesh
 * @return One Pier per footprint, in first-encountered order
 */
std::vector<Pier> piers_of(const Mesh& mesh) {
    struct Box {
        double x_lo = 1e300;
        double x_hi = -1e300;
        double y_lo = 1e300;
        double y_hi = -1e300;
    };

    const std::vector<p5::Component> faces = components_with(mesh, MaterialId::Concrete);
    std::vector<Box> plan(faces.size());
    for (size_t i = 0; i < faces.size(); ++i) {
        for (const glm::dvec2& p : p5::locals_of(mesh, faces[i])) {
            plan[i].x_lo = std::min(plan[i].x_lo, p.x);
            plan[i].x_hi = std::max(plan[i].x_hi, p.x);
            plan[i].y_lo = std::min(plan[i].y_lo, p.y);
            plan[i].y_hi = std::max(plan[i].y_hi, p.y);
        }
    }

    // Slack for the float32 the positions are stored in: two faces meeting along
    // one edge of the box must still read as touching.
    const double slack = 1e-3;
    std::vector<size_t> parent(faces.size());
    for (size_t i = 0; i < parent.size(); ++i) parent[i] = i;
    const std::function<size_t(size_t)> find = [&](size_t v) {
        while (parent[v] != v) {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        return v;
    };
    for (size_t i = 0; i < faces.size(); ++i) {
        for (size_t j = i + 1; j < faces.size(); ++j) {
            const bool overlap = plan[i].x_lo <= plan[j].x_hi + slack &&
                                 plan[j].x_lo <= plan[i].x_hi + slack &&
                                 plan[i].y_lo <= plan[j].y_hi + slack &&
                                 plan[j].y_lo <= plan[i].y_hi + slack;
            if (!overlap) continue;
            const size_t ra = find(i);
            const size_t rb = find(j);
            if (ra != rb) parent[ra] = rb;
        }
    }

    std::vector<Pier> out;
    std::vector<size_t> slot(faces.size(), static_cast<size_t>(-1));
    for (size_t i = 0; i < faces.size(); ++i) {
        const size_t root = find(i);
        if (slot[root] == static_cast<size_t>(-1)) {
            slot[root] = out.size();
            out.push_back(Pier{});
        }
        Pier& pier = out[slot[root]];
        for (const glm::vec3& p : p5::positions_of(mesh, faces[i])) {
            pier.lowest = std::min(pier.lowest, static_cast<double>(p.y));
            pier.highest = std::max(pier.highest, static_cast<double>(p.y));
        }
        pier.x_lo = std::min(pier.x_lo, plan[i].x_lo);
        pier.x_hi = std::max(pier.x_hi, plan[i].x_hi);
        pier.y_lo = std::min(pier.y_lo, plan[i].y_lo);
        pier.y_hi = std::max(pier.y_hi, plan[i].y_hi);
    }
    return out;
}

/**
 * @brief Every triangle's stored vertex normals agree with its winding
 *
 * The winding check that does not need to know what surface a triangle belongs
 * to. A producer computes a face normal, writes it onto three vertices, and
 * emits the three indices; if the index order is the reverse of the one that
 * produces that normal by the right-hand rule, the two disagree and the triangle
 * renders black from the side it is supposed to be seen from. That is exactly
 * the failure the handedness flip in `(x, y) -> (x, h, -y)` produces.
 *
 * @param mesh Mesh to check
 * @return True when every triangle's geometric normal points the same way as the
 *         mean of its three stored vertex normals
 */
bool winding_agrees_with_normals(const Mesh& mesh) {
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const auto& v0 = mesh.vertices[mesh.indices[t + 0]];
        const auto& v1 = mesh.vertices[mesh.indices[t + 1]];
        const auto& v2 = mesh.vertices[mesh.indices[t + 2]];

        const glm::dvec3 geometric = glm::cross(glm::dvec3(v1.position) - glm::dvec3(v0.position),
                                                glm::dvec3(v2.position) - glm::dvec3(v0.position));
        if (glm::length(geometric) < 1e-12) return false;

        const glm::dvec3 stored =
            glm::dvec3(v0.normal) + glm::dvec3(v1.normal) + glm::dvec3(v2.normal);
        if (glm::length(stored) < 1e-9) return false;
        if (glm::dot(glm::normalize(geometric), glm::normalize(stored)) <= 0.0) return false;
    }
    return true;
}

/**
 * @brief Undirected edges of @p mesh used by other than exactly two triangles
 *
 * Positions are welded on a @p weld grid first, because every quad the builders
 * emit carries its own four vertices so its faces meet at hard creases. Vertex
 * INDICES therefore never coincide between two faces of one solid, and an index
 * based count would report every solid as an open shell.
 *
 * A closed solid has none. A shell with a missing face has one boundary loop per
 * hole; a shell whose faces meet at a T-junction has the long edge counted once
 * and the two short ones once each.
 */
size_t open_edges_of(const Mesh& mesh, double weld = 1e-3) {
    const auto key_of = [&](const glm::vec3& p) {
        const auto q = [&](float v) {
            return static_cast<long long>(std::llround(static_cast<double>(v) / weld));
        };
        return std::to_string(q(p.x)) + "," + std::to_string(q(p.y)) + "," + std::to_string(q(p.z));
    };

    std::map<std::string, size_t> welded;
    std::vector<size_t> id(mesh.vertices.size(), 0);
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const std::string key = key_of(mesh.vertices[i].position);
        const auto it = welded.find(key);
        if (it == welded.end()) {
            const size_t next = welded.size();
            welded.emplace(key, next);
            id[i] = next;
        } else {
            id[i] = it->second;
        }
    }

    std::map<std::pair<size_t, size_t>, size_t> uses;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const size_t v[3] = { id[mesh.indices[t + 0]], id[mesh.indices[t + 1]],
                              id[mesh.indices[t + 2]] };
        for (int k = 0; k < 3; ++k) {
            const size_t a = v[k];
            const size_t b = v[(k + 1) % 3];
            if (a == b) continue;
            ++uses[{ std::min(a, b), std::max(a, b) }];
        }
    }

    size_t open = 0;
    for (const auto& entry : uses) {
        if (entry.second != 2) ++open;
    }
    return open;
}

/// Every structural sanity property one call, used by both halves of the suite
void check_structure_sanity(const Mesh& mesh, const char* label) {
    if (!p5::mesh_is_finite(mesh)) {
        stratum::test::report_failure(__FILE__, __LINE__, "structure mesh is finite",
                                      std::string(label));
    }
    if (!p5::mesh_indices_are_sane(mesh)) {
        stratum::test::report_failure(__FILE__, __LINE__, "structure indices are in range",
                                      std::string(label));
    }
    if (!p5::no_degenerate_triangles(mesh, 1e-9)) {
        stratum::test::report_failure(__FILE__, __LINE__, "structure has no degenerate triangle",
                                      std::string(label));
    }
    if (!winding_agrees_with_normals(mesh)) {
        stratum::test::report_failure(__FILE__, __LINE__, "structure winding matches its normals",
                                      std::string(label));
    }
}

} // namespace

// ============================================================================
// The deck
// ============================================================================

/**
 * The fixture is a bridge, the solver lifted it, and the ground is under it.
 *
 * A precondition, so that a failure in the elevation solve reads as one rather
 * than as a deck of the wrong thickness.
 */
TEST(Structures, bridge_fixture_is_lifted_clear_of_its_terrain) {
    const float ground = 20.0f;
    const Structure s = load_bridge(ground);
    if (!s.ok) return;

    CHECK_TRUE(s.edge().is_bridge);
    CHECK_TRUE(s.elevation.edge(s.edge_id).is_bridge);
    CHECK_EQ(s.heights().size(), s.cl().stations.size());

    double lo = 0.0;
    double hi = 0.0;
    surface_range(s, lo, hi);
    const ElevationConfig cfg;
    CHECK_TRUE(lo >= static_cast<double>(ground) + cfg.bridge_clearance - 0.01);

    // Abutments at degree-2 nodes, so nothing trimmed the edge back.
    CHECK_NEAR(s.edge().trim_from, 0.0, 1e-12);
    CHECK_NEAR(s.edge().trim_to, 0.0, 1e-12);
}

/**
 * The underside sits exactly deck_thickness below the running surface, and it
 * faces down.
 *
 * The depth is the assertion the plan asks for. The direction is the one that
 * matters more: a bridge is seen from below more often than any other road
 * geometry, and the world mapping flips handedness, so an underside wound from
 * its 2D ring comes out facing into the slab.
 */
TEST(Structures, deck_underside_is_deck_thickness_below_the_surface_and_faces_down) {
    const Structure s = load_bridge();
    if (!s.ok) return;

    BridgeConfig cfg;
    const Mesh mesh = build_bridge(s.edge(), s.cl(), s.profile(), s.heights(), s.terrain, cfg);
    CHECK_TRUE(!mesh.vertices.empty());
    check_structure_sanity(mesh, "bridge_abutment deck");

    double lo = 0.0;
    double hi = 0.0;
    surface_range(s, lo, hi);

    const double underside = p5::min_height_of_material(mesh, MaterialId::BridgeDeck);
    CHECK_NEAR(underside, lo - static_cast<double>(cfg.deck_thickness), 1e-3);

    // Nothing in the deck slab reaches above the running surface plus whatever
    // the profile raises its own outer edge by. A slab poking through the road
    // would be the same defect as a doubled deck top, seen from the other side.
    double raised = 0.0;
    for (const auto& strip : s.profile().strips) {
        raised = std::max(raised, static_cast<double>(std::max(strip.height_left, strip.height_right)));
    }
    CHECK_TRUE(p5::max_height_of_material(mesh, MaterialId::BridgeDeck) <= hi + raised + 1e-3);

    // Every DECK triangle lying flat at the underside plane must face DOWN.
    //
    // Restricted to MaterialId::BridgeDeck on purpose. A pier's top face is also
    // flat and also within a centimetre of the underside plane -- bridge_builder
    // embeds it there deliberately, so the flat top meets a slab that is ruled
    // along the grade -- and it faces UP, correctly. Testing every material here
    // would report each pier cap as a flipped underside, which is a defect in the
    // test rather than in the slab.
    size_t checked = 0;
    for (const p5::Component& deck : components_with(mesh, MaterialId::BridgeDeck)) {
        for (size_t t : deck.triangles) {
            const glm::dvec3 a(mesh.vertices[mesh.indices[t * 3 + 0]].position);
            const glm::dvec3 b(mesh.vertices[mesh.indices[t * 3 + 1]].position);
            const glm::dvec3 c(mesh.vertices[mesh.indices[t * 3 + 2]].position);

            const double want_a =
                surface_at(s, jt::world_to_local(mesh.vertices[mesh.indices[t * 3]].position)) -
                static_cast<double>(cfg.deck_thickness);
            const bool on_underside = std::fabs(a.y - want_a) < 0.05 &&
                                      std::fabs(b.y - want_a) < 0.2 &&
                                      std::fabs(c.y - want_a) < 0.2;
            if (!on_underside) continue;

            const glm::dvec3 n = glm::cross(b - a, c - a);
            if (std::fabs(n.y) < 1e-9) continue;   // a vertical face touching the plane
            ++checked;
            if (n.y >= 0.0) {
                stratum::test::report_failure(__FILE__, __LINE__, "deck underside faces down",
                                              "triangle " + std::to_string(t) +
                                                  " has normal.y " + std::to_string(n.y));
            }
        }
    }
    CHECK_TRUE(checked > 0);
}

/**
 * The running surface is emitted once, by the corridor, and never by the
 * structure builder.
 *
 * Asserted as the absence of any upward-facing structure triangle lying on the
 * carriageway plane. That is the shape a second copy of the deck top takes, and
 * it is the only shape it can take: a duplicate offset by more than the
 * tolerance here would not z-fight and would not be this defect.
 *
 * Parapets are excluded because a parapet's own top face is legitimately
 * upward-facing, and it stands parapet_height above the plane rather than on it.
 */
TEST(Structures, the_deck_top_is_never_emitted_twice) {
    const Structure s = load_bridge();
    if (!s.ok) return;

    const Mesh mesh = build_bridge(s.edge(), s.cl(), s.profile(), s.heights(), s.terrain,
                                   BridgeConfig{});
    CHECK_TRUE(!mesh.vertices.empty());

    size_t coplanar = 0;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const glm::dvec3 a(mesh.vertices[mesh.indices[t + 0]].position);
        const glm::dvec3 b(mesh.vertices[mesh.indices[t + 1]].position);
        const glm::dvec3 c(mesh.vertices[mesh.indices[t + 2]].position);

        const glm::dvec3 n = glm::cross(b - a, c - a);
        if (n.y <= 0.0) continue;   // not upward facing

        bool on_plane = true;
        for (const glm::dvec3& p : {a, b, c}) {
            const double want = surface_at(s, glm::dvec2{p.x, -p.z});
            if (std::fabs(p.y - want) > 0.02) {
                on_plane = false;
                break;
            }
        }
        if (on_plane) ++coplanar;
    }
    CHECK_EQ(coplanar, size_t{0});
}

// ============================================================================
// Parapets
// ============================================================================

/**
 * A parapet on each side, at parapet_height, standing inboard of the slab.
 *
 * Both sides matter: a builder that walked the profile once and offset by
 * `+ half_width` gets one wall in the right place and one in the middle of the
 * road, and on a symmetric profile at a glance that reads as a central
 * reservation rather than as a bug.
 */
TEST(Structures, parapets_stand_on_both_edges_at_parapet_height) {
    const Structure s = load_bridge();
    if (!s.ok) return;

    BridgeConfig cfg;
    const Mesh mesh = build_bridge(s.edge(), s.cl(), s.profile(), s.heights(), s.terrain, cfg);
    if (mesh.vertices.empty()) return;

    double lo = 0.0;
    double hi = 0.0;
    surface_range(s, lo, hi);

    // BridgeConfig::parapet_height is measured "above the running surface AT THE
    // PROFILE'S OUTER EDGE", not above the carriageway. On a profile whose
    // outermost strip is a sidewalk raised 0.15 m, the coping therefore stands
    // 0.15 m higher than the carriageway plus parapet_height -- which is what a
    // real parapet does, since it guards the footway it stands beside. Taking the
    // higher of the two outer strips because max_height_of_material() does.
    const double outer_raise = std::max(
        static_cast<double>(s.profile().strips.front().height_left),
        static_cast<double>(s.profile().strips.back().height_right));
    const double top = p5::max_height_of_material(mesh, MaterialId::Parapet);
    CHECK_NEAR(top, hi + outer_raise + static_cast<double>(cfg.parapet_height), 0.05);

    const double half = 0.5 * static_cast<double>(s.profile().total_width());
    const double left_edge = static_cast<double>(s.profile().left_edge_offset());
    const double right_edge = left_edge - static_cast<double>(s.profile().total_width());

    bool on_left = false;
    bool on_right = false;
    for (const jt::Tri2D& tri : p5::triangles_with(mesh, MaterialId::Parapet)) {
        for (const glm::dvec2& p : {tri.a, tri.b, tri.c}) {
            const double lateral = p5::lateral_of(s.cl(), p);
            // Never overhanging the slab it stands on.
            CHECK_TRUE(lateral <= left_edge + 1e-3);
            CHECK_TRUE(lateral >= right_edge - 1e-3);
            if (lateral > 0.5 * half) on_left = true;
            if (lateral < -0.5 * half) on_right = true;
        }
    }
    CHECK_TRUE(on_left);
    CHECK_TRUE(on_right);

    BridgeConfig off = cfg;
    off.emit_parapets = false;
    const Mesh bare = build_bridge(s.edge(), s.cl(), s.profile(), s.heights(), s.terrain, off);
    CHECK_EQ(p5::triangles_with(bare, MaterialId::Parapet).size(), size_t{0});
    CHECK_TRUE(!bare.vertices.empty());   // the deck is still there
}

// ============================================================================
// Piers
// ============================================================================

/**
 * Piers stand at the documented interior divisions and reach the terrain.
 *
 * The count is derived here from the same rule bridge_builder.hpp states, rather
 * than hard-coded: the span is divided into `ceil(span / pier_spacing)` equal
 * parts and a pier goes at each INTERIOR division, so a span no longer than one
 * spacing gets none. Deriving it keeps the test correct if the fixture's deck
 * length is ever adjusted, and it is the rule that stops a stub landing 30 cm
 * from an abutment.
 */
TEST(Structures, piers_stand_at_the_interior_divisions_and_reach_the_ground) {
    const float ground = 20.0f;
    const Structure s = load_bridge(ground);
    if (!s.ok) return;

    BridgeConfig cfg;
    const Mesh mesh = build_bridge(s.edge(), s.cl(), s.profile(), s.heights(), s.terrain, cfg);
    if (mesh.vertices.empty()) return;

    const double span = s.cl().stations.back().arclength - s.cl().stations.front().arclength;
    const auto parts = static_cast<size_t>(
        std::ceil(span / static_cast<double>(cfg.pier_spacing) - 1e-9));
    const size_t expected = parts > 0 ? parts - 1 : 0;
    CHECK_TRUE(expected >= 1);   // the fixture deck is long enough to need one

    // Grouped by PLAN POSITION, not by p5::components_of().
    //
    // A pier is a closed box whose six faces meet at hard creases, so each face
    // carries its own four vertices and its own normal -- welding them would
    // round the box off. Vertex connectivity therefore reports six components per
    // pier, which is correct geometry and the wrong unit to count. What makes a
    // pier one thing is that its faces share a footprint; see piers_of().
    const std::vector<Pier> piers = piers_of(mesh);
    CHECK_EQ(piers.size(), expected);

    for (const Pier& pier : piers) {
        // The top is pushed a centimetre into the deck; the foot goes down to its
        // FOUNDATION, BridgeConfig::pier_foundation_depth below the sampled
        // ground, so a later terrain carve under the bridge cannot leave it
        // hanging in mid-air.
        CHECK_NEAR(pier.lowest,
                   static_cast<double>(ground) - static_cast<double>(cfg.pier_foundation_depth),
                   0.05);
        CHECK_TRUE(pier.highest > pier.lowest + cfg.min_pier_height);

        // Square in plan, at pier_width.
        const double diagonal = std::hypot(pier.x_hi - pier.x_lo, pier.y_hi - pier.y_lo);
        CHECK_NEAR(diagonal, std::sqrt(2.0) * cfg.pier_width, 0.05);
    }
}

/**
 * A pier foot is buried to its foundation, not set on the natural surface.
 *
 * BridgeConfig::height_sampler reads the NATURAL ground. The ground a pier ends
 * up standing on is whatever the terrain carve leaves, and a bridge spans
 * exactly the places other roads have carved: CarveRibbon::suppress is set for
 * the bridge's OWN ribbon and never for the ribbon of the road passing
 * underneath. A motorway in a cutting under an overpass drops the finished
 * ground by metres, and a foot placed at the sampled surface then hangs in the
 * air over the one view a bridge is always seen from.
 *
 * BridgeConfig::pier_foundation_depth is the order-independent answer, and this
 * is the assertion that it is actually applied: every pier reaches well below
 * the ground it was measured against.
 */
TEST(Structures, a_pier_foot_is_buried_to_its_foundation) {
    const float ground = 20.0f;
    const Structure s = load_bridge(ground);
    if (!s.ok) return;

    BridgeConfig cfg;
    CHECK_TRUE(cfg.pier_foundation_depth > 1.0f);

    const Mesh mesh = build_bridge(s.edge(), s.cl(), s.profile(), s.heights(), s.terrain, cfg);
    if (mesh.vertices.empty()) return;

    const std::vector<Pier> piers = piers_of(mesh);
    CHECK_TRUE(!piers.empty());

    for (const Pier& pier : piers) {
        CHECK_TRUE(pier.lowest < static_cast<double>(ground) - 1.0);
    }
}

/**
 * A short span is a clear span, and so is one with no terrain to stand on.
 *
 * Three ways to get no pier, all of which must leave the deck and its parapets
 * standing: a span shorter than one spacing, the switch, and a terrain callback
 * the caller did not supply. The last is the one that would otherwise crash.
 */
TEST(Structures, a_short_bridge_gets_no_piers_and_does_not_crash) {
    const Structure s = load_bridge();
    if (!s.ok) return;

    BridgeConfig wide;
    wide.pier_spacing = 1000.0f;   // longer than the deck
    const Mesh clear_span =
        build_bridge(s.edge(), s.cl(), s.profile(), s.heights(), s.terrain, wide);
    CHECK_TRUE(!clear_span.vertices.empty());
    CHECK_EQ(p5::triangles_with(clear_span, MaterialId::Concrete).size(), size_t{0});
    check_structure_sanity(clear_span, "clear span");

    BridgeConfig no_piers;
    no_piers.emit_piers = false;
    const Mesh switched =
        build_bridge(s.edge(), s.cl(), s.profile(), s.heights(), s.terrain, no_piers);
    CHECK_TRUE(!switched.vertices.empty());
    CHECK_EQ(p5::triangles_with(switched, MaterialId::Concrete).size(), size_t{0});

    const Sampler none;
    CHECK_FALSE(static_cast<bool>(none));
    const Mesh groundless =
        build_bridge(s.edge(), s.cl(), s.profile(), s.heights(), none, BridgeConfig{});
    CHECK_TRUE(!groundless.vertices.empty());
    CHECK_EQ(p5::triangles_with(groundless, MaterialId::Concrete).size(), size_t{0});
    check_structure_sanity(groundless, "no terrain sampler");
}

// ============================================================================
// Bad bridge data
// ============================================================================

/**
 * A deck below the terrain at every station is rejected; one abutment in the
 * embankment is not.
 *
 * The distinction is the whole rule. A bridge whose every station is underground
 * is bad data and burying a structure in a hill helps nobody, so nothing is
 * emitted. But an abutment legitimately springs from the embankment it meets, so
 * a deck below the terrain at ONE end must still be built: rejecting that would
 * delete a correct bridge from every extract that has an embankment.
 */
TEST(Structures, a_deck_buried_at_every_station_emits_nothing) {
    const Structure s = load_bridge();
    if (!s.ok) return;

    double lo = 0.0;
    double hi = 0.0;
    surface_range(s, lo, hi);

    // Terrain far above the deck everywhere.
    const Mesh buried = build_bridge(s.edge(), s.cl(), s.profile(), s.heights(),
                                     p5::flat_sampler(static_cast<float>(hi + 50.0)),
                                     BridgeConfig{});
    CHECK_EQ(buried.indices.size(), size_t{0});
    CHECK_EQ(buried.vertices.size(), size_t{0});

    // Terrain above the deck at one end and well below it at the other: an
    // embankment, not bad data.
    const glm::dvec2 start = s.cl().stations.front().position;
    const glm::dvec2 tangent = s.cl().stations.front().tangent;
    const double span = s.cl().stations.back().arclength - s.cl().stations.front().arclength;
    const auto grade = static_cast<float>(-(hi + 20.0 - lo) / std::max(span, 1.0));
    const Mesh embankment =
        build_bridge(s.edge(), s.cl(), s.profile(), s.heights(),
                     p5::slope_sampler(start, tangent, static_cast<float>(hi + 10.0), grade),
                     BridgeConfig{});
    CHECK_TRUE(!embankment.vertices.empty());
    check_structure_sanity(embankment, "abutment in an embankment");
}

/**
 * A non-bridge edge produces nothing, whatever the caller thought it was
 * handing over.
 */
TEST(Structures, an_ordinary_street_never_grows_a_parapet) {
    const Structure s = load_bridge();
    if (!s.ok) return;

    GraphEdge not_a_bridge = s.edge();
    not_a_bridge.is_bridge = false;
    CHECK_EQ(build_bridge(not_a_bridge, s.cl(), s.profile(), s.heights(), s.terrain,
                          BridgeConfig{})
                 .indices.size(),
             size_t{0});

    // A mis-sized height vector is the other way a caller gets this wrong, and a
    // guessed height produces a structure detached from its own road.
    const std::vector<float> wrong(s.cl().stations.size() + 2, 30.0f);
    CHECK_EQ(build_bridge(s.edge(), s.cl(), s.profile(), wrong, s.terrain, BridgeConfig{})
                 .indices.size(),
             size_t{0});
    CHECK_EQ(build_bridge(s.edge(), s.cl(), s.profile(), {}, s.terrain, BridgeConfig{})
                 .indices.size(),
             size_t{0});
}

// ============================================================================
// Tunnel portals
// ============================================================================

namespace {

/// Ground level away from the ridge, in the tunnel tests, world Y
constexpr float kTunnelGround = 10.0f;

/// Height of the ridge crest above kTunnelGround
constexpr float kRidgePeak = 25.0f;

/**
 * @brief tests/data/tunnel.osm with a ridge and a HAND-BUILT road profile
 *
 * ### Why the heights are not the solver's
 *
 * The solver's profile is exercised on its own, once, by
 * the_elevation_solver_brings_a_tunnel_to_the_surface_at_its_portals. Everything
 * else here is about the BUILDER, and the builder's contract is a road running
 * at grade, entering a hillside, and coming out the far side -- so these tests
 * hand it exactly that shape and nothing else. `station_heights` is an input to
 * build_tunnel_portals(), not something it derives, so supplying it is using the
 * documented interface rather than side-stepping it.
 *
 * Holding the road dead flat also keeps the portal STATION where the fixture
 * puts it, at the ridge flank. Under the solver's ramped profile the roadway
 * starts diving the moment it leaves the portal node, so the crossing is a few
 * metres inside the edge end rather than a fifth of the way along it, and the
 * "found, not assumed" assertion below would be testing arithmetic on the
 * descent instead of the search.
 *
 * The road is held flat at the approach level and the ridge covers the middle
 * four fifths of the edge, so the surface crosses TunnelConfig::min_portal_cover
 * below the terrain roughly a quarter of the half-length inside each end. The
 * ridge is sized as a FRACTION of the parsed edge rather than in metres, because
 * OSMParser projects through Web Mercator with no latitude correction and one
 * local unit is a true metre divided by cos(latitude).
 */
struct Tunnel {
    p5::Network net;
    EdgeId edge_id = kInvalidId;
    Sampler terrain;
    std::vector<float> heights;
    double half_length = 0.0;
    bool ok = false;

    [[nodiscard]] const GraphEdge& edge() const { return net.graph.edge(edge_id); }
    [[nodiscard]] const Centerline& cl() const { return net.centerlines[edge_id]; }
    [[nodiscard]] const RoadProfile& profile() const { return net.profiles[edge_id]; }
};

/**
 * @brief Load tunnel.osm and place a ridge over the middle of its covered way
 *
 * @param ridge_fraction Ridge half-length as a fraction of the edge half-length.
 *                       Below 1.0 the road runs in the open at each end, which is
 *                       what puts a portal inside the edge.
 * @param road_depth     How far below kTunnelGround to hold the running surface.
 *                       Zero is a road at grade; a large value buries both ends
 *                       and is the case that must produce no portal.
 * @return The loaded tunnel; `ok` false when the fixture did not parse
 */
Tunnel load_tunnel(double ridge_fraction = 0.8, float road_depth = 0.0f) {
    Tunnel t;
    t.net = p5::make_network("tunnel.osm");
    if (!t.net.ok) return t;

    t.edge_id = sole_structure_edge(t.net.graph, /*want_bridge*/ false);
    if (t.edge_id == kInvalidId) {
        stratum::test::report_failure(__FILE__, __LINE__, "tunnel.osm has one tunnel edge", "");
        return t;
    }

    const Centerline& cl = t.cl();
    const double span = cl.stations.back().arclength - cl.stations.front().arclength;
    t.half_length = 0.5 * span;

    const glm::dvec2 crest = cl.stations[cl.stations.size() / 2].position;
    const glm::dvec2 axis = cl.stations.front().tangent;
    t.terrain = p5::ridge_sampler(crest, axis, t.half_length * ridge_fraction, kTunnelGround,
                                  ridge_fraction > 0.0 ? kRidgePeak : 0.0f);
    t.heights.assign(cl.stations.size(), kTunnelGround + 0.05f - road_depth);
    t.ok = true;
    return t;
}

} // namespace

/**
 * The elevation solver brings a tunnel to the surface at its portal nodes.
 *
 * ### What this test used to say
 *
 * It was called `the_elevation_solver_currently_buries_a_tunnel_at_both_ends`
 * and it pinned the OLD behaviour of P3 step 4, which dropped a tunnel edge by
 * the maximum demand over its whole length so that even its two end stations sat
 * ElevationConfig::tunnel_depth underground. A road that starts below the ground
 * never crosses it, build_tunnel_portals() found no crossing to place a mouth
 * at, and every fixture in tests/data reported zero portals. That test existed
 * to make the conflict visible, and it named itself as the one to rewrite when
 * the solver was fixed.
 *
 * ### What it says now
 *
 * ElevationConfig::tunnel_portal_at_surface is the fix and it is on by default.
 * The portal NODES stay at the approach roadway surface, tied to their approach
 * edges exactly as a bridge's abutments are, and the roadway dives away from
 * them at the edge's own max_grade_for() until it reaches depth. So:
 *
 * 1. Neither end of the tunnel edge is buried.
 * 2. The middle of it is.
 * 3. The descent never exceeds the road class grade limit -- a tunnel that
 *    plunged at 40% because it is "just a tunnel" would be as wrong as a
 *    rollercoaster road.
 * 4. The end stations still terminate at exactly their node heights, so the
 *    tunnel is not severed from its approaches.
 * 5. build_tunnel_portals() now finds a genuine crossing at each end and emits
 *    both mouths, from the SOLVER's heights rather than the hand-built ones the
 *    portal tests below supply.
 *
 * Turning the flag off restores the old shape exactly, which is asserted at the
 * end so the two can still be diffed.
 */
TEST(Structures, the_elevation_solver_brings_a_tunnel_to_the_surface_at_its_portals) {
    const Tunnel t = load_tunnel();
    if (!t.ok) return;

    RoadElevationSolver solver;
    const ElevationConfig cfg;
    solver.solve(t.net.graph, t.net.centerlines, t.terrain, cfg);
    CHECK_TRUE(solver.is_solved());
    if (!solver.is_solved()) return;

    const EdgeElevation& elev = solver.edge(t.edge_id);
    const std::vector<float>& solved = elev.station_heights;
    CHECK_TRUE(elev.is_tunnel);
    CHECK_EQ(solved.size(), t.cl().stations.size());
    if (solved.size() < 3) return;

    const auto cover_at = [&](size_t i) {
        const glm::dvec2 p = t.cl().stations[i].position;
        return static_cast<double>(t.terrain(p.x, p.y)) - static_cast<double>(solved[i]);
    };

    // 1. Open at both ends. The only thing between the roadway and the sky there
    //    is ElevationConfig::surface_offset, which is why this is a threshold
    //    rather than an equality: the road surface is that far ABOVE the ground.
    CHECK_TRUE(cover_at(0) < 0.25);
    CHECK_TRUE(cover_at(solved.size() - 1) < 0.25);

    // 2. And buried under the ridge in between, by more than the min_portal_cover
    //    a mouth is placed at.
    CHECK_TRUE(cover_at(solved.size() / 2) > 5.0);

    // 3. Grade-limited all the way down. tunnel.osm is highway=secondary.
    const double limit = static_cast<double>(max_grade_for(t.edge().type, cfg));
    double steepest = 0.0;
    for (size_t i = 0; i + 1 < solved.size(); ++i) {
        const double ds = t.cl().stations[i + 1].arclength - t.cl().stations[i].arclength;
        if (ds < 1e-6) continue;
        steepest = std::max(steepest,
                            std::fabs(static_cast<double>(solved[i + 1])
                                      - static_cast<double>(solved[i])) / ds);
    }
    if (steepest > limit * 1.05) {
        stratum::test::report_failure(__FILE__, __LINE__,
                                      "the tunnel descent stays inside its class grade limit",
                                      "steepest " + std::to_string(steepest) + " against a "
                                          + std::to_string(limit) + " limit");
    }
    // And it really did dive: a profile that stayed flat would pass the line
    // above for the wrong reason.
    CHECK_TRUE(steepest > 0.5 * limit);

    // 4. Still attached to its approaches at both ends.
    CHECK_NEAR(static_cast<double>(solved.front()),
               static_cast<double>(solver.node_height(t.edge().from)) + cfg.surface_offset, 1e-4);
    CHECK_NEAR(static_cast<double>(solved.back()),
               static_cast<double>(solver.node_height(t.edge().to)) + cfg.surface_offset, 1e-4);

    // 5. Two mouths, from the solved profile.
    std::vector<TunnelPortalFootprint> portals;
    const Mesh mesh = build_tunnel_portals(t.edge(), t.cl(), t.profile(), solved, t.terrain,
                                           TunnelConfig{}, &portals);
    CHECK_EQ(portals.size(), size_t{2});
    CHECK_TRUE(!mesh.indices.empty());
    check_structure_sanity(mesh, "portals from the solved tunnel profile");

    // The escape hatch. With the flag off the solver buries both ends again and
    // no portal survives the "already buried at that end" rule, which is exactly
    // what this test asserted before the fix.
    ElevationConfig old_shape = cfg;
    old_shape.tunnel_portal_at_surface = false;

    RoadElevationSolver buried;
    buried.solve(t.net.graph, t.net.centerlines, t.terrain, old_shape);
    CHECK_TRUE(buried.is_solved());
    if (!buried.is_solved()) return;

    const std::vector<float>& sunk = buried.edge(t.edge_id).station_heights;
    CHECK_EQ(sunk.size(), t.cl().stations.size());
    if (sunk.empty()) return;

    const glm::dvec2 head = t.cl().stations.front().position;
    const glm::dvec2 tail = t.cl().stations.back().position;
    CHECK_TRUE(static_cast<double>(t.terrain(head.x, head.y)) - sunk.front() > 1.0);
    CHECK_TRUE(static_cast<double>(t.terrain(tail.x, tail.y)) - sunk.back() > 1.0);

    std::vector<TunnelPortalFootprint> none;
    const Mesh empty = build_tunnel_portals(t.edge(), t.cl(), t.profile(), sunk, t.terrain,
                                            TunnelConfig{}, &none);
    CHECK_EQ(none.size(), size_t{0});
    CHECK_EQ(empty.indices.size(), size_t{0});
}

/**
 * The tunnel fixture is one tunnel edge between two unsplit degree-2 nodes, with
 * open ground at each end and a hill over the middle.
 *
 * Preconditions for everything below. If the nodes had been split per layer the
 * covered way would be detached from its approaches, and if the ridge reached
 * the ends of the edge there would be no crossing to find and the portal rule
 * would take its first no-portal branch instead of doing its work.
 */
TEST(Structures, tunnel_fixture_runs_under_a_ridge_with_open_ground_at_each_end) {
    const Tunnel t = load_tunnel();
    if (!t.ok) return;

    CHECK_TRUE(t.edge().is_tunnel);
    CHECK_EQ(t.edge().layer, -1);
    CHECK_EQ(t.net.graph.stats().layer_split_nodes, size_t{0});

    const GraphNodeId west = jt::node_with_osm_id(t.net.graph, 1502);
    const GraphNodeId east = jt::node_with_osm_id(t.net.graph, 1503);
    CHECK_TRUE(west != kInvalidId);
    CHECK_TRUE(east != kInvalidId);
    if (west != kInvalidId) CHECK_EQ(t.net.graph.node(west).degree(), size_t{2});
    if (east != kInvalidId) CHECK_EQ(t.net.graph.node(east).degree(), size_t{2});

    CHECK_EQ(t.heights.size(), t.cl().stations.size());
    CHECK_TRUE(t.half_length > 100.0);

    // Open at both ends: the road is at or above the ground there.
    const glm::dvec2 head = t.cl().stations.front().position;
    const glm::dvec2 tail = t.cl().stations.back().position;
    CHECK_TRUE(static_cast<double>(t.terrain(head.x, head.y)) - t.heights.front() < 0.25);
    CHECK_TRUE(static_cast<double>(t.terrain(tail.x, tail.y)) - t.heights.back() < 0.25);

    // And buried under the crest in the middle.
    const glm::dvec2 middle = t.cl().stations[t.cl().stations.size() / 2].position;
    CHECK_TRUE(static_cast<double>(t.terrain(middle.x, middle.y)) - t.heights.front() > 20.0);
}

/**
 * A portal is at the station where the road passes under the hill, not at the
 * end of the way.
 *
 * The assertion this fixture exists for. The ridge covers the middle four fifths
 * of the covered way, so the crossing where the road first passes
 * TunnelConfig::min_portal_cover below the terrain lands well inside each end. A
 * builder that put its headwall at the edge endpoint would stand it in the open
 * with the hillside tens of metres behind it.
 *
 * The window is expressed as a fraction of the edge half-length rather than in
 * metres, for the same reason the ridge is: the parser's local unit is a true
 * metre divided by cos(latitude), so a metre figure here would be a figure about
 * where in the world the fixture happens to sit.
 */
TEST(Structures, tunnel_portals_sit_where_the_road_goes_under_not_at_the_way_end) {
    const Tunnel t = load_tunnel();
    if (!t.ok) return;

    TunnelConfig cfg;
    std::vector<TunnelPortalFootprint> portals;
    const Mesh mesh =
        build_tunnel_portals(t.edge(), t.cl(), t.profile(), t.heights, t.terrain, cfg, &portals);

    CHECK_TRUE(!mesh.vertices.empty());
    check_structure_sanity(mesh, "tunnel.osm portals");
    CHECK_EQ(portals.size(), size_t{2});
    if (portals.size() != 2) return;

    // One portal at each end, start first.
    CHECK_TRUE(portals[0].at_start);
    CHECK_FALSE(portals[1].at_start);

    const glm::dvec2 head = t.cl().stations.front().position;
    const glm::dvec2 tail = t.cl().stations.back().position;
    const glm::dvec2 tangent = t.cl().stations.front().tangent;

    const double from_head = glm::length(portals[0].center - head);
    const double from_tail = glm::length(portals[1].center - tail);

    // Well inside the edge: the whole point.
    CHECK_TRUE(from_head > 0.10 * t.half_length);
    CHECK_TRUE(from_head < 0.60 * t.half_length);
    CHECK_TRUE(from_tail > 0.10 * t.half_length);
    CHECK_TRUE(from_tail < 0.60 * t.half_length);

    // The axis points INTO the hillside, so the two portals face each other.
    CHECK_NEAR(glm::length(portals[0].axis), 1.0, 1e-9);
    CHECK_NEAR(glm::length(portals[1].axis), 1.0, 1e-9);
    CHECK_TRUE(glm::dot(portals[0].axis, tangent) > 0.9);
    CHECK_TRUE(glm::dot(portals[1].axis, tangent) < -0.9);
    CHECK_TRUE(glm::dot(portals[0].axis, portals[1].axis) < -0.9);

    for (const TunnelPortalFootprint& portal : portals) {
        // The opening is the profile plus its margin, and the surround widens it
        // further; it is never narrower than the road it frames.
        const double opening_half =
            0.5 * static_cast<double>(t.profile().total_width()) +
            static_cast<double>(cfg.portal_width_margin);
        CHECK_TRUE(portal.half_width >= opening_half - 1e-6);
        CHECK_TRUE(portal.half_width <= opening_half + 2.0);

        CHECK_NEAR(portal.crown_height, portal.surface_height + cfg.portal_height, 1e-4);
        CHECK_NEAR(portal.surface_height, t.heights.front(), 0.05);
        CHECK_NEAR(portal.depth,
                   std::max(static_cast<double>(cfg.portal_cut_depth),
                            static_cast<double>(cfg.wall_thickness)),
                   1e-6);

        // The terrain at the portal is above the road by at least the cover, and
        // not by much more: the crossing is where it FIRST passes that depth.
        const double ground = static_cast<double>(t.terrain(portal.center.x, portal.center.y));
        CHECK_TRUE(ground - portal.surface_height >= cfg.min_portal_cover - 0.05);
        CHECK_TRUE(ground - portal.surface_height < 2.0);
    }
}

/**
 * The opening is a hole the road passes through.
 *
 * A headwall projects to a line in plan, so no coverage predicate can tell an
 * opening from a solid wall. The only test that works is to fire a segment along
 * the carriageway through the portal, at the heights a vehicle occupies, and
 * check that it hits nothing. The control is that geometry DOES exist beside the
 * opening, so a miss means a hole rather than an empty mesh.
 */
TEST(Structures, the_portal_opening_is_a_hole_the_road_passes_through) {
    const Tunnel t = load_tunnel();
    if (!t.ok) return;

    TunnelConfig cfg;
    std::vector<TunnelPortalFootprint> portals;
    const Mesh mesh =
        build_tunnel_portals(t.edge(), t.cl(), t.profile(), t.heights, t.terrain, cfg, &portals);
    if (portals.empty()) return;

    const double opening_half = 0.5 * static_cast<double>(t.profile().total_width()) +
                                static_cast<double>(cfg.portal_width_margin);

    for (const TunnelPortalFootprint& portal : portals) {
        const glm::dvec2 a2 = portal.center - portal.axis * 8.0;
        const glm::dvec2 b2 = portal.center + portal.axis * 8.0;

        for (double lift : {0.4, 1.5, 3.0}) {
            const double y = static_cast<double>(portal.surface_height) + lift;
            const glm::dvec3 a(a2.x, y, -a2.y);
            const glm::dvec3 b(b2.x, y, -b2.y);
            if (p5::segment_hits_mesh(mesh, a, b)) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "the road passes through the portal opening",
                    "blocked at " + std::to_string(lift) + " m above the carriageway");
            }
        }
    }

    // The control: there IS a wall beside the opening. Without this the test
    // above would pass on an empty mesh.
    bool beyond_the_opening = false;
    for (const auto& v : mesh.vertices) {
        if (std::fabs(p5::lateral_of(t.cl(), jt::world_to_local(v.position))) >
            opening_half - 1e-6) {
            beyond_the_opening = true;
            break;
        }
    }
    CHECK_TRUE(beyond_the_opening);

    // And the wall reaches from the carriageway upward rather than floating.
    const double lowest = p5::min_height_of_material(mesh, MaterialId::Concrete);
    CHECK_TRUE(lowest <= static_cast<double>(portals.front().surface_height) + 0.1);
}

/**
 * The headwall is a closed solid, not a face with an intrados behind it.
 *
 * The mouth is placed where the ground is only TunnelConfig::min_portal_cover
 * (0.25 m) above the carriageway, while the wall is at least
 * `portal_height + wall_thickness` (5.5 m) tall. So roughly 5.25 m of every
 * headwall stands PROUD of the hillside it is set into, and every face of it is
 * reachable by a camera: the back of the wall faces the hill for the whole run
 * before the ground climbs to meet it, the top is seen from above, and the two
 * returns are seen edge-on.
 *
 * Emitting only the outward face and the intrados leaves 36 of 54 edges used
 * once. Backface culling is on, so from the hill side the wall disappears and
 * the camera looks into the un-emitted bore; edge-on it is a zero-thickness
 * sheet. The block is therefore closed all the way round -- rear face, top cap,
 * both returns, and the undersides of the two jambs -- which is the same
 * standard bridge_builder.cpp already holds its parapets to.
 *
 * The top cap is cut into the SAME columns as the faces either side of it, so
 * the closure introduces no T-junction along the crown.
 */
TEST(Structures, a_portal_headwall_is_a_closed_solid) {
    const Tunnel t = load_tunnel();
    if (!t.ok) return;

    TunnelConfig cfg;
    std::vector<TunnelPortalFootprint> portals;
    const Mesh mesh =
        build_tunnel_portals(t.edge(), t.cl(), t.profile(), t.heights, t.terrain, cfg, &portals);

    CHECK_EQ(portals.size(), size_t{2});
    if (portals.empty()) return;

    // The premise: the wall really does stand clear of the ground it is set into,
    // so its back and top are not buried and closure is not academic.
    const double ground =
        static_cast<double>(t.terrain(portals.front().center.x, portals.front().center.y));
    const double crest = p5::max_height_of_material(mesh, MaterialId::Concrete);
    CHECK_TRUE(crest > ground + 4.0);

    CHECK_EQ(open_edges_of(mesh), size_t{0});
}

/**
 * No portal where a headwall would be worse than none.
 *
 * Both no-portal rules, and the three input cases.
 *
 * NOTHING BURIED is the first rule: a road at grade under flat ground never
 * passes below it, so there is no crossing to find and no mouth to frame.
 * ALREADY BURIED is the second: a road held far below flat ground is underground
 * at its own ends, and a headwall there is a wall across the middle of the
 * tunnel rather than a mouth. That second case is not hypothetical either. It is
 * what RoadElevationSolver produces for a graph node INTERIOR to a longer bore,
 * where nothing but tunnel edges meet, and what it produced everywhere before
 * ElevationConfig::tunnel_portal_at_surface; see
 * the_elevation_solver_brings_a_tunnel_to_the_surface_at_its_portals, which
 * asserts both shapes.
 */
TEST(Structures, no_portal_without_a_hillside_to_cut) {
    // Nothing buried: no ridge at all, road at grade.
    const Tunnel open_air = load_tunnel(/*ridge_fraction*/ 0.0, /*road_depth*/ 0.0f);
    if (open_air.ok) {
        std::vector<TunnelPortalFootprint> portals;
        const Mesh mesh = build_tunnel_portals(open_air.edge(), open_air.cl(), open_air.profile(),
                                               open_air.heights, open_air.terrain, TunnelConfig{},
                                               &portals);
        CHECK_EQ(mesh.indices.size(), size_t{0});
        CHECK_EQ(portals.size(), size_t{0});
    }

    // Already buried: no ridge, and the road held 20 m under the ground.
    const Tunnel deep = load_tunnel(/*ridge_fraction*/ 0.0, /*road_depth*/ 20.0f);
    if (deep.ok) {
        std::vector<TunnelPortalFootprint> portals;
        const Mesh mesh = build_tunnel_portals(deep.edge(), deep.cl(), deep.profile(),
                                               deep.heights, deep.terrain, TunnelConfig{},
                                               &portals);
        CHECK_EQ(mesh.indices.size(), size_t{0});
        CHECK_EQ(portals.size(), size_t{0});
    }

    const Tunnel t = load_tunnel();
    if (!t.ok) return;

    // No terrain at all: nothing may be guessed.
    std::vector<TunnelPortalFootprint> portals{TunnelPortalFootprint{}};
    const Sampler none;
    const Mesh blind = build_tunnel_portals(t.edge(), t.cl(), t.profile(), t.heights, none,
                                            TunnelConfig{}, &portals);
    CHECK_EQ(blind.indices.size(), size_t{0});
    CHECK_EQ(portals.size(), size_t{0});   // the sink is cleared even on the empty path

    // The switch.
    TunnelConfig off;
    off.emit_portals = false;
    CHECK_EQ(build_tunnel_portals(t.edge(), t.cl(), t.profile(), t.heights, t.terrain, off,
                                 nullptr)
                 .indices.size(),
             size_t{0});

    // A misrouted edge.
    GraphEdge not_a_tunnel = t.edge();
    not_a_tunnel.is_tunnel = false;
    CHECK_EQ(build_tunnel_portals(not_a_tunnel, t.cl(), t.profile(), t.heights, t.terrain,
                                 TunnelConfig{}, nullptr)
                 .indices.size(),
             size_t{0});

    // A mis-sized height vector.
    const std::vector<float> wrong(t.cl().stations.size() + 1, 10.0f);
    CHECK_EQ(build_tunnel_portals(t.edge(), t.cl(), t.profile(), wrong, t.terrain, TunnelConfig{},
                                 nullptr)
                 .indices.size(),
             size_t{0});
}

/**
 * emit_bore is threaded through and ignored, and says so by changing nothing.
 *
 * The flag exists so the eventual bore has a switch already wired into
 * RoadNetworkConfig. Setting it must not fail and must not promise geometry that
 * is not there, so the first-pass output is identical either way.
 */
TEST(Structures, the_bore_switch_changes_nothing_yet) {
    const Tunnel t = load_tunnel();
    if (!t.ok) return;

    TunnelConfig plain;
    TunnelConfig with_bore;
    with_bore.emit_bore = true;

    const Mesh a = build_tunnel_portals(t.edge(), t.cl(), t.profile(), t.heights, t.terrain,
                                        plain, nullptr);
    const Mesh b = build_tunnel_portals(t.edge(), t.cl(), t.profile(), t.heights, t.terrain,
                                        with_bore, nullptr);
    CHECK_EQ(b.vertices.size(), a.vertices.size());
    CHECK_EQ(b.indices.size(), a.indices.size());
}

// ============================================================================
// Winding and finiteness over every structure the fixtures hold
// ============================================================================

/**
 * Every structure triangle in every fixture is wound correctly and finite.
 *
 * The sweep, run over both bridge fixtures and the tunnel one, so a defect that
 * only appears on a deck crossing another road or on an abutment shared with an
 * approach is still caught. A NaN vertex reaching the GPU neither crashes nor
 * renders: it corrupts the bounding box and breaks frustum culling for the whole
 * chunk, and it is the hardest failure in this pipeline to trace back.
 */
TEST(Structures, every_structure_triangle_is_finite_and_correctly_wound) {
    for (const char* filename : {"bridge_abutment.osm", "bridge_over.osm"}) {
        p5::Network net = p5::make_network(filename);
        if (!net.ok) continue;

        RoadElevationSolver elevation;
        const Sampler terrain = p5::flat_sampler(20.0f);
        elevation.solve(net.graph, net.centerlines, terrain, ElevationConfig{});
        if (!elevation.is_solved()) continue;

        size_t built = 0;
        for (size_t i = 0; i < net.graph.edges().size(); ++i) {
            const auto& edge = net.graph.edges()[i];
            if (!edge.is_bridge) continue;
            const auto id = static_cast<EdgeId>(i);
            const Mesh mesh = build_bridge(edge, net.centerlines[id], net.profiles[id],
                                           elevation.edge(id).station_heights, terrain,
                                           BridgeConfig{});
            if (mesh.vertices.empty()) continue;
            ++built;
            check_structure_sanity(mesh, filename);

            // Every triangle is in one of the three structure slots and in none
            // of the surface slots the corridor owns.
            for (const jt::Tri2D& tri : jt::triangles_of(mesh)) {
                const bool structural = tri.material == MaterialId::BridgeDeck ||
                                        tri.material == MaterialId::Parapet ||
                                        tri.material == MaterialId::Concrete;
                if (!structural) {
                    stratum::test::report_failure(
                        __FILE__, __LINE__, "bridge geometry is in a structure material slot",
                        std::string(filename) + ": " + stratum::material_id_name(tri.material));
                }
            }
        }
        CHECK_TRUE(built > 0);
    }

    const Tunnel tunnel = load_tunnel();
    if (!tunnel.ok) return;
    const Mesh portals = build_tunnel_portals(tunnel.edge(), tunnel.cl(), tunnel.profile(),
                                              tunnel.heights, tunnel.terrain, TunnelConfig{},
                                              nullptr);
    if (portals.vertices.empty()) return;
    check_structure_sanity(portals, "tunnel.osm");
    for (const jt::Tri2D& tri : jt::triangles_of(portals)) {
        CHECK_EQ(tri.material, MaterialId::Concrete);
    }
}
