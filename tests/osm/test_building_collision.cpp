// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_building_collision.cpp
 * @brief The building hull: closed, outward, and cheaper than the roof it replaces
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * build_building_collision_mesh() makes three promises, and each one fails
 * silently if it is not checked here.
 *
 * ### It is closed
 *
 * The hull shares one vertex set between its walls and both caps, so being
 * closed is a property of the INDEX buffer and can be read straight off it:
 * every undirected edge is used by exactly two triangles. An open hull is a
 * bucket -- anything that gets inside stands on the floor behind a wall it
 * cannot pass -- and nothing about a bucket shows up in a triangle count, a
 * bounding box or a screenshot. Most of the assertions below are this one
 * measured under a different input.
 *
 * ### It is wound outward
 *
 * The footprint is extruded, so every face's outward direction is decided by its
 * ring's winding, and Building's documented winding is a convention an OSM
 * extract does not honour. A ring that arrives reversed turns the building
 * inside out, which a backface-culling physics engine reports as no collider at
 * all. The tests feed the same building in both windings and assert that the
 * hull comes out the same way round.
 *
 * ### It costs less than the render mesh
 *
 * Only where the render mesh is expensive, which means the roof. A four-sided
 * flat-roofed box already renders as ten triangles and no hull beats that; the
 * saving is on a dome or a hipped roof, and the fixture here is a sixteen-sided
 * dome for exactly that reason.
 *
 * ### Courtyards
 *
 * MeshBuilder::build_building_mesh() extrudes walls from `footprint` only, so a
 * courtyard renders as a hole in the roof with no wall around it. The hull walks
 * `holes` too. That difference is asserted directly: a facade on the courtyard
 * boundary, facing into the courtyard, which the render mesh does not have.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests BuildingCollision
 * @endcode
 */

#include "framework.hpp"

#include "osm/building_collision.hpp"
#include "osm/mesh_builder.hpp"
#include "osm/road/collision_mesh.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

using stratum::Mesh;
using stratum::osm::Building;
using stratum::osm::BuildingCollisionConfig;
using stratum::osm::MeshBuilder;
using stratum::osm::RoofType;
using stratum::osm::build_building_collision_mesh;
using stratum::osm::kMinRingArea;

namespace {

constexpr float kHeight = 12.0f;
constexpr double kTol = 1e-3;

/// M_PI is not in the standard; the regular n-gon fixture needs pi and nothing else.
constexpr double kPi = 3.14159265358979323846;

/// A rectangle centred on the origin, counter-clockwise, with no closing duplicate.
std::vector<glm::dvec2> rectangle(double length = 20.0, double width = 10.0) {
    const double hl = length * 0.5;
    const double hw = width * 0.5;
    return { { -hl, -hw }, { hl, -hw }, { hl, hw }, { -hl, hw } };
}

/// The 4 x 4 courtyard used throughout, clockwise, as Building::holes documents.
std::vector<glm::dvec2> courtyard() {
    return { { -2.0, -2.0 }, { -2.0, 2.0 }, { 2.0, 2.0 }, { 2.0, -2.0 } };
}

/// A regular n-gon, counter-clockwise. The dome fixture's footprint.
std::vector<glm::dvec2> regular_polygon(size_t n, double radius) {
    std::vector<glm::dvec2> ring;
    ring.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const double t = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
        ring.push_back({ radius * std::cos(t), radius * std::sin(t) });
    }
    return ring;
}

Building make_building(std::vector<glm::dvec2> footprint,
                       std::vector<std::vector<glm::dvec2>> holes = {},
                       RoofType roof = RoofType::Flat) {
    Building b;
    b.osm_id = 1;
    b.footprint = std::move(footprint);
    b.holes = std::move(holes);
    b.height = kHeight;
    b.roof_type = roof;
    return b;
}

size_t triangle_count(const Mesh& mesh) { return mesh.indices.size() / 3u; }

/// Geometric normal of triangle @p t, unnormalised. Zero for a degenerate one.
glm::vec3 face_normal(const Mesh& mesh, size_t t) {
    const glm::vec3& a = mesh.vertices[mesh.indices[t * 3u + 0u]].position;
    const glm::vec3& b = mesh.vertices[mesh.indices[t * 3u + 1u]].position;
    const glm::vec3& c = mesh.vertices[mesh.indices[t * 3u + 2u]].position;
    return glm::cross(b - a, c - a);
}

glm::vec3 face_centroid(const Mesh& mesh, size_t t) {
    const glm::vec3& a = mesh.vertices[mesh.indices[t * 3u + 0u]].position;
    const glm::vec3& b = mesh.vertices[mesh.indices[t * 3u + 1u]].position;
    const glm::vec3& c = mesh.vertices[mesh.indices[t * 3u + 2u]].position;
    return (a + b + c) / 3.0f;
}

/**
 * @brief Undirected index edges used by a number of triangles other than two
 *
 * Zero means the surface is closed. The count is taken over INDICES rather than
 * over positions, which is only meaningful because the hull shares one vertex
 * set across its walls and both caps -- the property this is really measuring.
 */
size_t non_manifold_edges(const Mesh& mesh) {
    std::map<std::pair<uint32_t, uint32_t>, size_t> use;
    for (size_t i = 0; i + 2u < mesh.indices.size(); i += 3u) {
        for (size_t k = 0; k < 3u; ++k) {
            const uint32_t a = mesh.indices[i + k];
            const uint32_t b = mesh.indices[i + ((k + 1u) % 3u)];
            ++use[{ a < b ? a : b, a < b ? b : a }];
        }
    }
    size_t bad = 0;
    for (const auto& [edge, count] : use) {
        (void)edge;
        if (count != 2u) ++bad;
    }
    return bad;
}

/**
 * @brief Plan-view area covered by upward-facing triangles, square metres
 *
 * The geometric normal's Y component IS twice the plan area of the triangle, so
 * this is a sum of half of it over the faces that point up. It is the measure a
 * downward raycast sees: the roof cap contributes its full plan, a wall
 * contributes nothing, and a courtyard contributes nothing because nothing caps
 * it.
 */
double up_facing_plan_area(const Mesh& mesh) {
    double total = 0.0;
    for (size_t t = 0; t < triangle_count(mesh); ++t) {
        const float ny = face_normal(mesh, t).y;
        if (ny > 0.0f) total += 0.5 * static_cast<double>(ny);
    }
    return total;
}

double down_facing_plan_area(const Mesh& mesh) {
    double total = 0.0;
    for (size_t t = 0; t < triangle_count(mesh); ++t) {
        const float ny = face_normal(mesh, t).y;
        if (ny < 0.0f) total -= 0.5 * static_cast<double>(ny);
    }
    return total;
}

/// Near-vertical triangles standing in the plane x = @p x, facing @p sign in X
size_t vertical_faces_at_x(const Mesh& mesh, float x, float sign) {
    size_t found = 0;
    for (size_t t = 0; t < triangle_count(mesh); ++t) {
        bool in_plane = true;
        for (size_t k = 0; k < 3u; ++k) {
            if (std::fabs(mesh.vertices[mesh.indices[t * 3u + k]].position.x - x) > 1e-3f) {
                in_plane = false;
                break;
            }
        }
        if (!in_plane) continue;

        const glm::vec3 n = face_normal(mesh, t);
        const float len = glm::length(n);
        if (!(len > 0.0f)) continue;
        if (std::fabs(n.y / len) >= stratum::osm::road::kVerticalNormalY) continue;
        if (n.x * sign <= 0.0f) continue;
        ++found;
    }
    return found;
}

bool all_vertices_finite(const Mesh& mesh) {
    for (const auto& v : mesh.vertices) {
        if (!std::isfinite(v.position.x) || !std::isfinite(v.position.y)
            || !std::isfinite(v.position.z)) {
            return false;
        }
        if (!std::isfinite(v.normal.x) || !std::isfinite(v.normal.y)
            || !std::isfinite(v.normal.z)) {
            return false;
        }
        if (!std::isfinite(v.uv.x) || !std::isfinite(v.uv.y)) return false;
    }
    return true;
}

} // namespace

// ============================================================================
// Cost
// ============================================================================

TEST(BuildingCollision, a_dome_costs_far_more_to_render_than_to_collide_with) {
    // Sixteen sides and roof:shape=dome: six latitude bands, each a ring of
    // quads, which is where a building's triangles actually go. The hull is the
    // same sixteen wall quads plus two fourteen-triangle caps.
    const Building building = make_building(regular_polygon(16, 10.0), {}, RoofType::Dome);

    const Mesh render = MeshBuilder::build_building_mesh(building);
    const Mesh hull = build_building_collision_mesh(building);

    CHECK((triangle_count(render)) > size_t{100});
    CHECK((triangle_count(hull) * 2u) < triangle_count(render));
    CHECK((triangle_count(hull)) > size_t{0});
}

TEST(BuildingCollision, the_road_derivation_over_the_render_mesh_stays_larger) {
    // The alternative design: run osm::road::build_collision_mesh() over the
    // finished building. It deletes by MaterialId and build_building_mesh()
    // tags nothing, so nothing is deleted; it simplifies with locked borders,
    // and a roof's eaves ring is a border; and it holds near-vertical geometry
    // out of the simplification entirely, which on a building is the facade and
    // the steep lower bands of the dome. It cannot reach the roof, which is the
    // only expensive part.
    const Building building = make_building(regular_polygon(16, 10.0), {}, RoofType::Dome);

    const Mesh render = MeshBuilder::build_building_mesh(building);
    const Mesh derived = stratum::osm::road::build_collision_mesh(render);
    const Mesh hull = build_building_collision_mesh(building);

    CHECK((triangle_count(hull)) < triangle_count(derived));
}

TEST(BuildingCollision, the_hull_does_not_change_with_the_roof_shape) {
    // The hull caps at the eaves and never looks at RoofType. Stating it as a
    // test so that a later roof family cannot quietly start leaking into it.
    const std::vector<glm::dvec2> plan = rectangle();

    const Mesh flat = build_building_collision_mesh(make_building(plan, {}, RoofType::Flat));
    const Mesh dome = build_building_collision_mesh(make_building(plan, {}, RoofType::Dome));
    const Mesh gabled = build_building_collision_mesh(make_building(plan, {}, RoofType::Gabled));

    CHECK_EQ(triangle_count(flat), triangle_count(dome));
    CHECK_EQ(triangle_count(flat), triangle_count(gabled));
    CHECK_EQ(flat.vertices.size(), gabled.vertices.size());
    CHECK_NEAR(up_facing_plan_area(flat), up_facing_plan_area(dome), 1e-6);
}

// ============================================================================
// Shape
// ============================================================================

TEST(BuildingCollision, a_box_is_eight_walls_and_two_caps) {
    const Mesh hull = build_building_collision_mesh(make_building(rectangle()));

    // Two vertices per ring point, shared by the walls and both caps.
    CHECK_EQ(hull.vertices.size(), size_t{8});
    CHECK_EQ(triangle_count(hull), size_t{12});
    CHECK_EQ(hull.submeshes.size(), size_t{0});
    CHECK_EQ(hull.effective_submeshes().size(), size_t{1});
}

TEST(BuildingCollision, the_hull_spans_the_footprints_plan_extent) {
    // 20 x 10 centred on the origin. Local metres map to world as
    // (x, height, -y), so the footprint's y range becomes the world z range
    // negated -- a hull that did not flip it would sit mirrored across the road.
    const Mesh hull = build_building_collision_mesh(make_building(rectangle(20.0, 10.0)));

    CHECK_NEAR(hull.bounds.min.x, -10.0, kTol);
    CHECK_NEAR(hull.bounds.max.x, 10.0, kTol);
    CHECK_NEAR(hull.bounds.min.z, -5.0, kTol);
    CHECK_NEAR(hull.bounds.max.z, 5.0, kTol);
    CHECK_NEAR(up_facing_plan_area(hull), 200.0, kTol);
    CHECK_NEAR(down_facing_plan_area(hull), 200.0, kTol);
}

TEST(BuildingCollision, the_cap_sits_at_the_eaves_and_the_floor_on_the_footprint_plane) {
    const Mesh hull = build_building_collision_mesh(make_building(rectangle()));

    CHECK_NEAR(hull.bounds.max.y, kHeight, 1e-4);
    CHECK_NEAR(hull.bounds.min.y, 0.0, 1e-4);
}

TEST(BuildingCollision, a_foundation_sinks_the_floor_without_moving_the_cap) {
    BuildingCollisionConfig cfg;
    cfg.foundation_depth = 3.0f;
    const Mesh hull = build_building_collision_mesh(make_building(rectangle()), cfg);

    CHECK_NEAR(hull.bounds.min.y, -3.0, 1e-4);
    CHECK_NEAR(hull.bounds.max.y, kHeight, 1e-4);
    CHECK_EQ(non_manifold_edges(hull), size_t{0});
}

TEST(BuildingCollision, an_explicit_cap_height_overrides_the_eaves) {
    BuildingCollisionConfig cfg;
    cfg.cap_height = 18.5f;
    const Mesh hull = build_building_collision_mesh(make_building(rectangle()), cfg);

    CHECK_NEAR(hull.bounds.max.y, 18.5, 1e-4);
}

// ============================================================================
// Closure
// ============================================================================

TEST(BuildingCollision, a_closed_hull_has_no_unpaired_edge) {
    CHECK_EQ(non_manifold_edges(build_building_collision_mesh(make_building(rectangle()))),
             size_t{0});
    CHECK_EQ(non_manifold_edges(
                 build_building_collision_mesh(make_building(regular_polygon(16, 10.0)))),
             size_t{0});
    CHECK_EQ(non_manifold_edges(
                 build_building_collision_mesh(make_building(rectangle(), { courtyard() }))),
             size_t{0});
}

TEST(BuildingCollision, dropping_the_floor_opens_exactly_the_bottom_ring) {
    BuildingCollisionConfig cfg;
    cfg.close_bottom = false;
    const Mesh hull = build_building_collision_mesh(make_building(rectangle(), { courtyard() }),
                                                    cfg);

    // Four outer edges plus four courtyard edges, each now used by its wall
    // quad alone. Nothing else may have come unpaired with them.
    CHECK_EQ(non_manifold_edges(hull), size_t{8});
    CHECK_NEAR(down_facing_plan_area(hull), 0.0, kTol);
    CHECK_NEAR(up_facing_plan_area(hull), 184.0, kTol);
}

// ============================================================================
// Winding
// ============================================================================

TEST(BuildingCollision, every_face_of_a_convex_hull_points_away_from_its_centre) {
    // A convex prism has the property that every outward face's normal makes a
    // positive angle with the vector from the hull's centre to that face. Inside
    // out fails on every triangle at once, which is what a reversed footprint
    // would produce.
    const Mesh hull = build_building_collision_mesh(make_building(regular_polygon(12, 8.0)));
    const glm::vec3 centre(0.0f, kHeight * 0.5f, 0.0f);

    size_t inward = 0;
    for (size_t t = 0; t < triangle_count(hull); ++t) {
        const glm::vec3 n = face_normal(hull, t);
        if (glm::dot(n, face_centroid(hull, t) - centre) <= 0.0f) ++inward;
    }
    CHECK_EQ(inward, size_t{0});
}

TEST(BuildingCollision, a_reversed_footprint_still_builds_the_same_way_round) {
    // Building documents its outer ring counter-clockwise. An extract does not
    // honour that, and a ring taken on trust turns the building inside out.
    std::vector<glm::dvec2> reversed = regular_polygon(12, 8.0);
    std::reverse(reversed.begin(), reversed.end());

    const Mesh hull = build_building_collision_mesh(make_building(reversed));
    const glm::vec3 centre(0.0f, kHeight * 0.5f, 0.0f);

    size_t inward = 0;
    for (size_t t = 0; t < triangle_count(hull); ++t) {
        const glm::vec3 n = face_normal(hull, t);
        if (glm::dot(n, face_centroid(hull, t) - centre) <= 0.0f) ++inward;
    }
    CHECK_EQ(inward, size_t{0});
    CHECK_EQ(non_manifold_edges(hull), size_t{0});
}

TEST(BuildingCollision, a_ring_carrying_its_closing_duplicate_builds_the_same_hull) {
    // An OSM way closes by repeating its first node. Left in, the repeat is a
    // zero-width wall quad with no orientation and an extra unpaired edge.
    std::vector<glm::dvec2> closed = rectangle();
    closed.push_back(closed.front());

    const Mesh open_ring = build_building_collision_mesh(make_building(rectangle()));
    const Mesh closed_ring = build_building_collision_mesh(make_building(closed));

    CHECK_EQ(closed_ring.vertices.size(), open_ring.vertices.size());
    CHECK_EQ(triangle_count(closed_ring), triangle_count(open_ring));
    CHECK_EQ(non_manifold_edges(closed_ring), size_t{0});
}

// ============================================================================
// Courtyards
// ============================================================================

TEST(BuildingCollision, a_courtyard_gets_a_facade_the_render_mesh_never_emits) {
    // The courtyard's east wall stands in the plane x = 2 and faces -X, into the
    // courtyard. build_building_mesh() extrudes the outer ring only, so its
    // walls are at x = +-10 and nothing stands at x = 2 at all: a character on
    // that roof walks off the edge of the world.
    const Building building = make_building(rectangle(), { courtyard() });

    const Mesh hull = build_building_collision_mesh(building);
    const Mesh render = MeshBuilder::build_building_mesh(building);

    CHECK_EQ(vertical_faces_at_x(hull, 2.0f, -1.0f), size_t{2});
    CHECK_EQ(vertical_faces_at_x(render, 2.0f, -1.0f), size_t{0});
    CHECK_EQ(vertical_faces_at_x(render, 2.0f, 1.0f), size_t{0});
}

TEST(BuildingCollision, a_courtyard_is_cut_out_of_both_caps) {
    const Mesh hull = build_building_collision_mesh(
        make_building(rectangle(), { courtyard() }));

    // 20 x 10 less the 4 x 4 courtyard, on the roof and on the floor alike.
    CHECK_NEAR(up_facing_plan_area(hull), 184.0, kTol);
    CHECK_NEAR(down_facing_plan_area(hull), 184.0, kTol);
    CHECK_EQ(hull.vertices.size(), size_t{16});
}

TEST(BuildingCollision, a_courtyard_wound_the_wrong_way_is_re_wound) {
    // Building documents holes clockwise. A multipolygon relation names its
    // inner members without promising a direction, so the winding is forced
    // rather than trusted; taken on trust, this courtyard's facade would point
    // into the masonry.
    std::vector<glm::dvec2> reversed = courtyard();
    std::reverse(reversed.begin(), reversed.end());

    const Mesh hull = build_building_collision_mesh(make_building(rectangle(), { reversed }));

    CHECK_EQ(vertical_faces_at_x(hull, 2.0f, -1.0f), size_t{2});
    CHECK_NEAR(up_facing_plan_area(hull), 184.0, kTol);
    CHECK_EQ(non_manifold_edges(hull), size_t{0});
}

TEST(BuildingCollision, a_degenerate_courtyard_is_dropped_rather_than_triangulated) {
    // A zero-area hole handed to the triangulator produces a triangulation whose
    // boundary is no longer the rings, and the hull stops being closed. Dropping
    // it costs a courtyard nobody could stand in.
    const std::vector<glm::dvec2> collapsed = { { 1.0, 1.0 }, { 1.0, 1.0 }, { 1.0, 1.0 } };
    const std::vector<glm::dvec2> sliver = { { -1.0, 0.0 }, { 1.0, 0.0 }, { 3.0, 0.0 } };

    const Mesh hull =
        build_building_collision_mesh(make_building(rectangle(), { collapsed, sliver }));

    CHECK_EQ(non_manifold_edges(hull), size_t{0});
    CHECK_EQ(hull.vertices.size(), size_t{8});
    CHECK_NEAR(up_facing_plan_area(hull), 200.0, kTol);
}

// ============================================================================
// Degenerate input
// ============================================================================

TEST(BuildingCollision, a_degenerate_footprint_produces_an_empty_mesh) {
    const Mesh empty = build_building_collision_mesh(make_building({}));
    CHECK_EQ(empty.vertices.size(), size_t{0});
    CHECK_EQ(empty.indices.size(), size_t{0});

    const Mesh two_points =
        build_building_collision_mesh(make_building({ { 0.0, 0.0 }, { 10.0, 0.0 } }));
    CHECK_EQ(two_points.indices.size(), size_t{0});

    const Mesh collinear = build_building_collision_mesh(
        make_building({ { 0.0, 0.0 }, { 5.0, 0.0 }, { 10.0, 0.0 }, { 3.0, 0.0 } }));
    CHECK_EQ(collinear.indices.size(), size_t{0});

    const Mesh one_repeated_point = build_building_collision_mesh(
        make_building({ { 4.0, 4.0 }, { 4.0, 4.0 }, { 4.0, 4.0 }, { 4.0, 4.0 } }));
    CHECK_EQ(one_repeated_point.indices.size(), size_t{0});
}

TEST(BuildingCollision, a_footprint_under_the_minimum_ring_area_is_rejected) {
    // kMinRingArea is a square millimetre. A 0.1 mm square is under it; a 10 mm
    // square is over it and builds, however pointless the building is.
    const double under = 1e-4;
    const double over = 1e-2;
    CHECK((under * under) < kMinRingArea);
    CHECK((over * over) > kMinRingArea);

    CHECK_EQ(build_building_collision_mesh(make_building(rectangle(under, under))).indices.size(),
             size_t{0});
    CHECK((build_building_collision_mesh(make_building(rectangle(over, over))).indices.size())
          > size_t{0});
}

TEST(BuildingCollision, a_building_with_no_height_produces_an_empty_mesh) {
    Building flatpack = make_building(rectangle());
    flatpack.height = 0.0f;
    CHECK_EQ(build_building_collision_mesh(flatpack).indices.size(), size_t{0});

    // A cap below the floor would build an inside-out prism, which reads as no
    // collider at all on a backface-culling engine.
    BuildingCollisionConfig cfg;
    cfg.cap_height = -5.0f;
    CHECK_EQ(build_building_collision_mesh(make_building(rectangle()), cfg).indices.size(),
             size_t{0});
}

TEST(BuildingCollision, every_vertex_of_every_hull_is_finite) {
    BuildingCollisionConfig sunk;
    sunk.foundation_depth = 2.5f;

    const Mesh box = build_building_collision_mesh(make_building(rectangle()));
    const Mesh round_plan = build_building_collision_mesh(make_building(regular_polygon(16, 10.0)));
    const Mesh with_court =
        build_building_collision_mesh(make_building(rectangle(), { courtyard() }));
    const Mesh founded = build_building_collision_mesh(make_building(rectangle()), sunk);

    CHECK_TRUE(all_vertices_finite(box));
    CHECK_TRUE(all_vertices_finite(round_plan));
    CHECK_TRUE(all_vertices_finite(with_court));
    CHECK_TRUE(all_vertices_finite(founded));

    CHECK_TRUE(box.bounds.is_valid());
    CHECK_TRUE(with_court.bounds.is_valid());
}

TEST(BuildingCollision, a_non_finite_footprint_point_is_dropped_rather_than_carried) {
    // A NaN in a collision mesh takes the whole broadphase with it rather than
    // failing locally, so a non-finite coordinate is not a point.
    const double nan = std::nan("");
    std::vector<glm::dvec2> poisoned = rectangle();
    poisoned.push_back({ nan, 3.0 });

    const Mesh hull = build_building_collision_mesh(make_building(poisoned));

    CHECK_TRUE(all_vertices_finite(hull));
    CHECK_EQ(hull.vertices.size(), size_t{8});
    CHECK_EQ(non_manifold_edges(hull), size_t{0});
}

// ============================================================================
// Rings the triangulator cannot honour
// ============================================================================
//
// These four cases all reached the old emptiness-only guard and came out as
// walled shells with no cap -- the exact thing the file comment says is never
// emitted. earcut rarely refuses a bad ring outright; cureLocalIntersections()
// deletes points and returns a smaller triangulation whose boundary is no
// longer the input, so the walls end up built around a cap that does not span
// them. Counting triangles against V + 2H - 2 is what catches that, and these
// are the inputs that prove it.
//
// None of them is hypothetical. A closed OSM way that revisits a node satisfies
// Way::is_closed(), and OSMParser::process_buildings() applies no simplicity
// test beyond a point count, so the parser hands them over as-is.

TEST(BuildingCollision, a_figure_eight_footprint_is_refused_rather_than_left_open) {
    // A way that returns to its start and carries on into a second loop. Before
    // the triangle-count check this produced 24 triangles with 8 unpaired edges.
    const std::vector<glm::dvec2> figure_eight = {
        { 0.0, 0.0 },   { 10.0, 0.0 },   { 10.0, 10.0 }, { 0.0, 10.0 },
        { 0.0, 0.0 },   { -10.0, 0.0 },  { -10.0, -10.0 }, { 0.0, -10.0 },
    };

    const Mesh hull = build_building_collision_mesh(make_building(figure_eight));

    CHECK_EQ(triangle_count(hull), size_t{0});
    CHECK_TRUE(hull.vertices.empty());
}

TEST(BuildingCollision, a_footprint_with_a_spur_is_refused_rather_than_left_open) {
    // A ring that runs out along a zero-width spur and back. Previously 18
    // triangles with 10 unpaired edges.
    const std::vector<glm::dvec2> spur = {
        { 0.0, 0.0 }, { 10.0, 0.0 }, { 10.0, 5.0 }, { 5.0, 5.0 },
        { 5.0, 9.0 }, { 5.0, 5.0 },  { 0.0, 5.0 },
    };

    const Mesh hull = build_building_collision_mesh(make_building(spur));

    CHECK_EQ(triangle_count(hull), size_t{0});
}

TEST(BuildingCollision, a_hole_outside_the_outer_ring_is_refused) {
    // Nothing validates that a hole lies inside its outer ring, and earcut
    // silently drops the geometry it cannot place. Latent today only because no
    // parser path fills Building::holes yet.
    Building b = make_building(rectangle(20.0, 10.0));
    b.holes.push_back({ { 30.0, 30.0 }, { 30.0, 34.0 }, { 34.0, 34.0 }, { 34.0, 30.0 } });

    const Mesh hull = build_building_collision_mesh(b);

    CHECK_EQ(triangle_count(hull), size_t{0});
}

TEST(BuildingCollision, a_hole_straddling_the_outer_ring_is_refused) {
    // The worst of the four: this one used to produce 205 m2 of up-facing plan
    // area against a 200 m2 footprint -- collision roof hanging over open air
    // outside the walls -- as well as 8 unpaired edges.
    Building b = make_building(rectangle(20.0, 10.0));
    b.holes.push_back({ { 8.0, 3.0 }, { 8.0, 8.0 }, { 14.0, 8.0 }, { 14.0, 3.0 } });

    const Mesh hull = build_building_collision_mesh(b);

    CHECK_EQ(triangle_count(hull), size_t{0});
    CHECK_NEAR(up_facing_plan_area(hull), 0.0, 1e-9);
}

TEST(BuildingCollision, a_hole_strictly_inside_still_produces_a_closed_hull) {
    // The guard must refuse bad rings without refusing good ones. A courtyard
    // well inside the outer ring is the case the whole check exists to preserve.
    Building b = make_building(rectangle(20.0, 10.0));
    b.holes.push_back({ { -2.0, -2.0 }, { -2.0, 2.0 }, { 2.0, 2.0 }, { 2.0, -2.0 } });

    const Mesh hull = build_building_collision_mesh(b);

    CHECK((triangle_count(hull)) > (size_t{0}));
    CHECK_EQ(non_manifold_edges(hull), size_t{0});
    CHECK_TRUE(all_vertices_finite(hull));
}
