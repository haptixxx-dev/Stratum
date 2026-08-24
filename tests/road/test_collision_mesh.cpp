/**
 * @file test_collision_mesh.cpp
 * @brief The physics surface: what was deleted, what was bridged, and what must never open
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * build_collision_mesh() is a derivation by deletion, and every one of its
 * deletions is a chance to put a hole in the ground. The failure mode is not a
 * wrong triangle count -- it is a character controller dropping through the road
 * at a place a render-only test never looks at, hours into a play session, with
 * nothing in the mesh statistics to say why.
 *
 * So the assertions here are about COVERAGE and about STEPS, not about counts.
 *
 * ### Coverage
 *
 * Every walkable triangle of the render mesh -- carriageway, footway, kerb top --
 * has its centroid, in plan, inside some triangle of the collision mesh. A kerb
 * face is 150 mm tall and leans 20 mm outward, so deleting it outright would take
 * a 20 mm strip of ground with it, running the length of every kerbed street in
 * the network, and a downward wheel raycast finds that strip. The contract says
 * each deleted step is bridged with its own plan footprint laid flat; this is
 * that promise, measured.
 *
 * The strict per-point form of the test runs with simplification switched OFF,
 * because that isolates the deletion stage, which is what the bridging contract
 * is about. Simplification is checked separately, against the property its own
 * contract states: it is rejected outright if it loses plan area.
 *
 * ### Steps
 *
 * After the deletions the surface is discontinuous in Y wherever a kerb was, and
 * that is correct and intended: a 150 mm step is something every engine steps
 * over, a 150 mm wall is something every engine slides along. The test is that
 * every discontinuity is a STEP -- an open boundary edge with a partner edge
 * directly above or below it, no further than CollisionConfig::max_step_height --
 * and never an unpaired edge in the middle of the road.
 *
 * ### Materials
 *
 * The output carries none. `submeshes` is empty, which
 * Mesh::effective_submeshes() reports as exactly one implicit
 * MaterialId::Default range.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests CollisionMesh
 * @endcode
 */

#include "framework.hpp"
#include "road/p7_fixtures.hpp"

#include "osm/road/collision_mesh.hpp"
#include "osm/road/corridor.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::SubMesh;
using stratum::Vertex;
using stratum::osm::road::CollisionConfig;
using stratum::osm::road::Corridor;
using stratum::osm::road::build_collision_mesh;
using stratum::osm::road::kVerticalNormalY;

namespace p7 = stratum::test::p7;
namespace jt = stratum::test::junction;

/// Length of the fixture road, metres
constexpr double kRoadLength = 200.0;

/// Plan tolerance when asking whether two boundary edges sit above each other
constexpr double kStepPlanEps = 0.05;

/// A quad from four world-space corners, wound counter-clockwise seen from above
Mesh quad_from(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d) {
    Mesh mesh;
    const glm::vec3 corners[4] = { a, b, c, d };
    for (int i = 0; i < 4; ++i) {
        Vertex v{};
        v.position = corners[i];
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v.uv = glm::vec2(static_cast<float>(i & 1), static_cast<float>((i >> 1) & 1));
        v.color = glm::vec4(1.0f);
        v.tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        mesh.vertices.push_back(v);
    }
    mesh.indices = { 0, 1, 2, 0, 2, 3 };
    return mesh;
}

/**
 * @brief A finished render piece: kerbed corridor plus painted dashes
 *
 * The dashes float 5 mm above the carriageway, which is the depth-bias trick the
 * render mesh uses and the second floor the collision mesh must not inherit. They
 * are separate geometry sharing no vertex with the corridor, exactly as
 * markings.hpp promises, so a collision builder that keeps them shows up as a
 * duplicated surface rather than as a changed count.
 */
Mesh make_render_piece(Corridor& out_corridor) {
    out_corridor = p7::kerbed_corridor(kRoadLength);
    Mesh mesh = out_corridor.mesh;

    // Centre-line dashes: 2 m of paint every 8 m, 0.1 m wide, on the lane
    // boundary at lateral 0, which is where the corridor's two lanes meet.
    for (double s = 4.0; s + 2.0 < kRoadLength; s += 8.0) {
        const float y = 0.005f;
        const float x0 = static_cast<float>(s);
        const float x1 = static_cast<float>(s + 2.0);
        const Mesh dash = quad_from(glm::vec3(x0, y, 0.05f), glm::vec3(x1, y, 0.05f),
                                    glm::vec3(x1, y, -0.05f), glm::vec3(x0, y, -0.05f));
        mesh.append(dash, MaterialId::Markings);
    }
    mesh.sort_submeshes_by_material();
    mesh.compute_bounds();
    return mesh;
}

/// Cosine of a triangle's geometric normal against +Y
double normal_y_of(const jt::Tri2D& tri) {
    const double len = glm::length(tri.world_normal);
    if (!(len > 0.0)) return 0.0;
    return std::fabs(tri.world_normal.y) / len;
}

/// Every position of a mesh, quantised, as a set
std::set<p7::PosKey> position_set(const Mesh& mesh) {
    std::set<p7::PosKey> out;
    for (const Vertex& v : mesh.vertices) out.insert(p7::pos_key(v.position));
    return out;
}

/// Midpoint of a boundary edge, back in world metres
glm::dvec3 edge_midpoint(const p7::EdgeKey& e) {
    return glm::dvec3(
        0.5 * (static_cast<double>(e.a.x) + static_cast<double>(e.b.x)) * p7::kPositionQuantum,
        0.5 * (static_cast<double>(e.a.y) + static_cast<double>(e.b.y)) * p7::kPositionQuantum,
        0.5 * (static_cast<double>(e.a.z) + static_cast<double>(e.b.z)) * p7::kPositionQuantum);
}

/**
 * @brief A vertical strip at z = 0, running along +x, @p height tall
 *
 * Cut into 1 m quads, so the plan-view window that measures a face's local height
 * has vertices to find inside it. Wound so the outward normal is +z, which is
 * horizontal, so every triangle classifies as a face.
 */
Mesh vertical_strip(double x0, double x1, float height) {
    Mesh mesh;
    for (double x = x0; x + 1.0 <= x1 + 1e-9; x += 1.0) {
        const float a = static_cast<float>(x);
        const float b = static_cast<float>(x + 1.0);
        mesh.append(quad_from(glm::vec3(a, 0.0f, 0.0f), glm::vec3(b, 0.0f, 0.0f),
                              glm::vec3(b, height, 0.0f), glm::vec3(a, height, 0.0f)),
                    MaterialId::Curb);
    }
    return mesh;
}

/// Near-vertical collision triangles whose centroid falls in [x0, x1] at z = 0
size_t faces_kept_between(const Mesh& mesh, double x0, double x1) {
    size_t kept = 0;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const glm::vec3& a = mesh.vertices[mesh.indices[i]].position;
        const glm::vec3& b = mesh.vertices[mesh.indices[i + 1]].position;
        const glm::vec3& c = mesh.vertices[mesh.indices[i + 2]].position;
        const glm::vec3 face = glm::cross(b - a, c - a);
        const float len = glm::length(face);
        if (!(len > 0.0f)) continue;
        if (std::fabs(face.y / len) >= kVerticalNormalY) continue;
        const glm::vec3 centroid = (a + b + c) / 3.0f;
        if (std::fabs(centroid.z) > 0.01f) continue;
        if (centroid.x < x0 || centroid.x > x1) continue;
        ++kept;
    }
    return kept;
}

/// Plan projection of a world point, matching the pipeline's mapping
glm::dvec2 plan_of(const glm::dvec3& world) { return glm::dvec2(world.x, -world.z); }

} // namespace

// ============================================================================
// Deletions
// ============================================================================

/**
 * No paint survives into the physics surface.
 *
 * Marking quads never share a vertex with the corridor, so their positions are a
 * disjoint set and their survival is decidable by position alone. A collision
 * mesh that kept them has a second floor 5 mm above the road, and a wheel
 * raycast jitters between the two.
 */
TEST(CollisionMesh, markings_do_not_survive) {
    Corridor corridor;
    const Mesh render = make_render_piece(corridor);
    CHECK_TRUE(p7::triangles_with_material(render, MaterialId::Markings) > 0);

    // Positions that belong to paint and to nothing else.
    std::set<p7::PosKey> paint;
    std::set<p7::PosKey> surface;
    const std::vector<MaterialId> per_triangle = p7::triangle_materials(render);
    for (size_t t = 0; t < per_triangle.size(); ++t) {
        for (int k = 0; k < 3; ++k) {
            const p7::PosKey key = p7::pos_key(render.vertices[render.indices[t * 3 + k]].position);
            (per_triangle[t] == MaterialId::Markings ? paint : surface).insert(key);
        }
    }
    for (const p7::PosKey& key : surface) paint.erase(key);
    CHECK_TRUE(!paint.empty());

    const Mesh collision = build_collision_mesh(render);
    CHECK_TRUE(p7::triangle_count(collision) > 0);

    size_t survivors = 0;
    for (const p7::PosKey& key : position_set(collision)) {
        if (paint.count(key) != 0) ++survivors;
    }
    if (survivors != 0) {
        stratum::test::report_failure(__FILE__, __LINE__, "no marking vertex survives",
                                      std::to_string(survivors) + " of " +
                                          std::to_string(paint.size()) +
                                          " paint-only positions are still in the collision mesh");
    }
}

/**
 * The footway is kept or dropped on request, and dropping it removes exactly the
 * footway's plan area.
 *
 * The fixture's two 2 m footways over a 200 m run are 800 square metres, which is
 * a number the test can state rather than read back. A drop_* flag that deletes
 * the wrong thing shows up as the wrong area, not as a missing triangle.
 */
TEST(CollisionMesh, sidewalks_are_kept_or_dropped_on_request) {
    Corridor corridor;
    const Mesh render = make_render_piece(corridor);

    CollisionConfig keep;
    keep.simplify_ratio = 1.0f;
    CollisionConfig drop = keep;
    drop.include_sidewalk = false;

    const double with_footway = jt::plan_area(jt::triangles_of(build_collision_mesh(render, keep)));
    const double without = jt::plan_area(jt::triangles_of(build_collision_mesh(render, drop)));

    CHECK_TRUE(with_footway > without);
    // Two footways, 2.0 m each, over the swept length.
    CHECK_NEAR(with_footway - without, 2.0 * 2.0 * corridor.length, 0.05 * corridor.length);
}

/**
 * A piece that is nothing but paint yields an empty surface, not a mesh of zero
 * triangles with a stale vertex array behind it.
 */
TEST(CollisionMesh, an_all_paint_piece_yields_an_empty_surface) {
    Mesh paint;
    paint.append(p7::make_quad_mesh(), MaterialId::Markings);
    CHECK_EQ(p7::triangle_count(paint), size_t{2});

    const Mesh collision = build_collision_mesh(paint);
    CHECK_EQ(p7::triangle_count(collision), size_t{0});
    CHECK_TRUE(collision.indices.empty());
}

/**
 * A kerb that touches a wall is still a kerb.
 *
 * Vertical faces are grouped into connected patches by shared POSITION alone --
 * no material gate, no height gate -- and the patch only exists to bound the
 * window each local height is measured in. It must not be the unit of the
 * keep/delete decision: a 150 mm kerb and a 1.5 m parapet that meet at one shared
 * vertex are one patch, and a patch-wide maximum keeps the whole kerb as a wall
 * to slide along rather than deleting it and bridging it into a step.
 *
 * This is not hypothetical geometry. A bridge deck's end cap is deck_thickness
 * deep and is emitted from the same profile columns as the corridor's kerb face
 * at the first and last station, so the two share exact positions and unite. With
 * a patch-wide test every kerb on every bridge deck in the network is a wall, for
 * the whole length of the edge, because of two triangles at each end of it.
 */
TEST(CollisionMesh, a_kerb_touching_a_wall_is_still_a_step) {
    Mesh render;
    // Ground for the faces to stand on, so the piece is not all vertical.
    render.append(quad_from(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(18.0f, 0.0f, 0.0f),
                            glm::vec3(18.0f, 0.0f, -4.0f), glm::vec3(0.0f, 0.0f, -4.0f)),
                  MaterialId::Asphalt);
    // A 150 mm kerb face for 9 m, then a 1.5 m parapet for 9 m, meeting at x = 9.
    render.append(vertical_strip(0.0, 9.0, 0.15f), MaterialId::Curb);
    render.append(vertical_strip(9.0, 18.0, 1.5f), MaterialId::Parapet);
    render.sort_submeshes_by_material();
    render.compute_bounds();

    CollisionConfig cfg;
    cfg.simplify_ratio = 1.0f;      // isolate the deletion stage
    const Mesh collision = build_collision_mesh(render, cfg);

    // The parapet is a wall and stays, whole: 9 quads, 18 triangles.
    CHECK_EQ(faces_kept_between(collision, 9.0, 18.0), size_t{18});

    // The kerb is a step and goes. The one quad that shares x = 9 with the
    // parapet is exempt and is expected to stay: its own window still sees the
    // full 1.5 m, which is what stops the split cutting into the wall.
    const size_t kerb_kept = faces_kept_between(collision, 0.0, 8.0);
    if (kerb_kept != 0) {
        stratum::test::report_failure(__FILE__, __LINE__,
                                      "a kerb connected to a parapet is still deleted as a step",
                                      std::to_string(kerb_kept) +
                                          " kerb face triangles were kept as a wall");
    }
}

// ============================================================================
// Coverage
// ============================================================================

/**
 * THE NO-HOLE TEST.
 *
 * Every walkable triangle of the render mesh has its centroid, in plan, inside
 * the collision surface. Vertical triangles are excluded from the sample, since
 * they project to slivers and are what was deleted; their footprint is checked by
 * the fact that the sample points either side of them are still covered, and by
 * the step test below.
 *
 * Simplification is off here on purpose. This test is about the deletion and
 * bridging stage: whether removing the kerb faces left the 20 mm batter strip
 * open. Simplification has its own contract and its own test.
 */
TEST(CollisionMesh, no_hole_where_the_kerb_face_was) {
    Corridor corridor;
    const Mesh render = make_render_piece(corridor);

    CollisionConfig cfg;
    cfg.simplify_ratio = 1.0f;
    const Mesh collision = build_collision_mesh(render, cfg);
    CHECK_TRUE(p7::triangle_count(collision) > 0);
    if (collision.indices.empty()) return;

    const std::vector<jt::Tri2D> collision_tris = jt::triangles_of(collision);
    const std::vector<jt::Tri2D> render_tris = jt::triangles_of(render);

    size_t sampled = 0;
    size_t uncovered = 0;
    glm::dvec2 first_gap{ 0.0 };
    for (const jt::Tri2D& tri : render_tris) {
        if (tri.material == MaterialId::Markings) continue;      // paint, deliberately gone
        if (normal_y_of(tri) < static_cast<double>(kVerticalNormalY)) continue;   // a face, gone
        const glm::dvec2 centroid = (tri.a + tri.b + tri.c) / 3.0;
        ++sampled;
        if (!jt::covered_in_plan(collision_tris, centroid)) {
            if (uncovered == 0) first_gap = centroid;
            ++uncovered;
        }
    }
    CHECK_TRUE(sampled > 0);
    if (uncovered != 0) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "every walkable render triangle is covered in plan",
            std::to_string(uncovered) + " of " + std::to_string(sampled) +
                " sample points fall through; first at (" + std::to_string(first_gap.x) + ", " +
                std::to_string(first_gap.y) + ")");
    }
}

/**
 * Simplification is allowed to change the triangulation and is not allowed to
 * lose ground.
 *
 * Its own contract says a candidate that lost plan area or grew its boundary is
 * rejected and the unsimplified surface kept, so the plan area at the shipping
 * ratio can never be below the plan area with simplification off.
 */
TEST(CollisionMesh, simplification_never_loses_plan_area) {
    Corridor corridor;
    const Mesh render = make_render_piece(corridor);

    CollisionConfig off;
    off.simplify_ratio = 1.0f;
    CollisionConfig on;     // the shipping 0.3

    const Mesh unsimplified = build_collision_mesh(render, off);
    const Mesh simplified = build_collision_mesh(render, on);

    const double area_off = jt::plan_area(jt::triangles_of(unsimplified));
    const double area_on = jt::plan_area(jt::triangles_of(simplified));
    CHECK_TRUE(area_off > 0.0);
    CHECK_TRUE(area_on >= area_off - 1e-6 * area_off);

    // Boundary must not grow either: a new open edge is a new hole.
    CHECK_TRUE(p7::boundary_edges(simplified).size() <=
               p7::boundary_edges(unsimplified).size());
}

// ============================================================================
// Steps
// ============================================================================

/**
 * Every discontinuity left in the surface is a step, not a hole, and no step is
 * taller than CollisionConfig::max_step_height.
 *
 * An open boundary edge is legal in exactly two places: on the outer footprint of
 * the piece, where the road simply ends, and at the top or bottom of a deleted
 * kerb, where it must have a partner edge directly above or below it. Anything
 * else is an unpaired edge in the middle of the road, which is a hole.
 *
 * The partner's vertical separation is the step height, and it is the number
 * every character controller is configured against.
 */
TEST(CollisionMesh, boundary_edges_are_steps_within_the_step_height) {
    Corridor corridor;
    const Mesh render = make_render_piece(corridor);
    CHECK_TRUE(!corridor.outline.empty());

    CollisionConfig cfg;
    cfg.simplify_ratio = 1.0f;
    const Mesh collision = build_collision_mesh(render, cfg);
    if (collision.indices.empty()) {
        CHECK_TRUE(false);
        return;
    }

    const std::set<p7::EdgeKey> edges = p7::boundary_edges(collision);
    CHECK_TRUE(!edges.empty());

    // Midpoints, split into those on the piece's own footprint and the rest.
    std::vector<glm::dvec3> interior;
    for (const p7::EdgeKey& e : edges) {
        const glm::dvec3 mid = edge_midpoint(e);
        if (jt::point_ring_distance(corridor.outline, plan_of(mid)) <= 0.05) continue;
        interior.push_back(mid);
    }

    // The kerb faces were deleted, so there is something to find. A run with no
    // interior boundary at all would pass the loop below vacuously.
    CHECK_TRUE(!interior.empty());

    size_t unpaired = 0;
    double worst_step = 0.0;
    glm::dvec3 worst{ 0.0 };
    for (size_t i = 0; i < interior.size(); ++i) {
        double best = 1e300;
        for (size_t j = 0; j < interior.size(); ++j) {
            if (i == j) continue;
            const glm::dvec2 pi = plan_of(interior[i]);
            const glm::dvec2 pj = plan_of(interior[j]);
            if (glm::length(pi - pj) > kStepPlanEps) continue;
            const double dy = std::fabs(interior[i].y - interior[j].y);
            if (dy < 1e-9) continue;    // the same rim, not the other side of a step
            best = std::min(best, dy);
        }
        if (best > 1e299) {
            ++unpaired;
            continue;
        }
        if (best > worst_step) {
            worst_step = best;
            worst = interior[i];
        }
    }

    if (unpaired != 0) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "every interior boundary edge is one side of a step",
            std::to_string(unpaired) + " of " + std::to_string(interior.size()) +
                " interior boundary edges have no partner above or below them");
    }

    if (worst_step > static_cast<double>(cfg.max_step_height) + 1e-6) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "no step exceeds max_step_height",
            "worst step is " + std::to_string(worst_step) + " m at (" +
                std::to_string(worst.x) + ", " + std::to_string(worst.y) + ", " +
                std::to_string(worst.z) + "), limit is " +
                std::to_string(cfg.max_step_height));
    }

    // The fixture's kerb is 0.15 m, so that is the step that should be found.
    CHECK_NEAR(worst_step, 0.15, 1e-3);
}

// ============================================================================
// Output shape
// ============================================================================

/**
 * The physics surface carries one material range and no submeshes.
 *
 * Materials have collapsed by this point, and nothing downstream should branch on
 * them. An empty `submeshes` is how Mesh spells "one implicit whole-mesh range",
 * so effective_submeshes() must report exactly one.
 */
TEST(CollisionMesh, output_is_one_implicit_material_range) {
    Corridor corridor;
    const Mesh render = make_render_piece(corridor);
    CHECK_TRUE(render.submeshes.size() >= 3);

    const Mesh collision = build_collision_mesh(render);
    CHECK_TRUE(collision.submeshes.empty());

    const std::vector<SubMesh> ranges = collision.effective_submeshes();
    CHECK_EQ(ranges.size(), size_t{1});
    if (!ranges.empty()) {
        CHECK_EQ(ranges[0].index_offset, uint32_t{0});
        CHECK_EQ(ranges[0].index_count, static_cast<uint32_t>(collision.indices.size()));
        CHECK_TRUE(ranges[0].material == MaterialId::Default);
    }

    CHECK_TRUE(p7::indices_are_sane(collision));
    CHECK_TRUE(p7::mesh_is_finite(collision));
    CHECK_TRUE(collision.bounds.is_valid());
}

/**
 * Winding is carried through, so surviving triangles still face upward.
 *
 * A physics engine that culls backfaces on its collision geometry drives straight
 * through a road whose triangles were flipped, and nothing about the mesh
 * statistics says so.
 */
TEST(CollisionMesh, surviving_triangles_still_face_up) {
    Corridor corridor;
    const Mesh render = make_render_piece(corridor);

    CollisionConfig cfg;
    cfg.simplify_ratio = 1.0f;
    const Mesh collision = build_collision_mesh(render, cfg);

    size_t downward = 0;
    for (const jt::Tri2D& tri : jt::triangles_of(collision)) {
        const double len = glm::length(tri.world_normal);
        if (!(len > 0.0)) continue;
        if (tri.world_normal.y / len < -0.1) ++downward;
    }
    CHECK_EQ(downward, size_t{0});
}
