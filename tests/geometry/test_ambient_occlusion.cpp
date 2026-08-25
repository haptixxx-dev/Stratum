/**
 * @file test_ambient_occlusion.cpp
 * @brief The per-vertex ambient occlusion bake
 *
 * The bake is a Monte Carlo estimate, so nothing here asserts an exact value --
 * it asserts the ORDERING and the invariants, which is what the renderer depends
 * on and what a regression would break. An open plane must stay open; a vertex
 * with geometry over it must darken; nothing may leave [min_ao, 1].
 */

#include "framework.hpp"

#include "geometry/ambient_occlusion.hpp"

#include <algorithm>
#include <cmath>

using stratum::Mesh;
using stratum::Vertex;
using stratum::geometry::AOSettings;
using stratum::geometry::bake_ambient_occlusion;

namespace {

/// An upward-facing grid on y = @p height, @p span metres square, one quad.
void add_quad(Mesh& mesh, float height, float span, const glm::vec3& normal) {
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    const float h = span * 0.5f;
    const glm::vec3 corners[4] = {
        { -h, height, -h }, { h, height, -h }, { h, height, h }, { -h, height, h },
    };
    for (const glm::vec3& corner : corners) {
        Vertex v{};
        v.position = corner;
        v.normal = normal;
        mesh.vertices.push_back(v);
    }
    const uint32_t order[6] = { 0, 1, 2, 0, 2, 3 };
    for (const uint32_t i : order) mesh.indices.push_back(base + i);
}

/// The mean ao over @p count vertices starting at @p first.
float mean_ao(const Mesh& mesh, size_t first, size_t count) {
    float sum = 0.0f;
    for (size_t i = 0; i < count; ++i) sum += mesh.vertices[first + i].ao;
    return sum / static_cast<float>(count);
}

} // namespace

/**
 * @brief An isolated plane is fully open
 *
 * The baseline every other assertion is relative to. A plane with nothing above
 * it occludes none of its own hemisphere -- and it must not occlude ITSELF, which
 * is what the normal-offset bias exists to prevent. Without that bias every ray
 * would hit the triangle it started on and the whole mesh would bake to min_ao.
 */
TEST(AmbientOcclusion, an_open_plane_stays_fully_lit) {
    Mesh mesh;
    add_quad(mesh, 0.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));

    AOSettings settings;
    settings.ray_count = 32;
    bake_ambient_occlusion(mesh, settings);

    for (const Vertex& v : mesh.vertices) {
        CHECK(v.ao > 0.98f);
    }
}

/**
 * @brief A slab overhead darkens the ground under it
 *
 * The core case: a roof, a bridge deck, an overhang. The lower quad's hemisphere
 * is largely blocked by the upper one, and the upper one -- facing away from it --
 * is not.
 */
TEST(AmbientOcclusion, geometry_overhead_darkens_what_is_beneath_it) {
    // The ceiling is MUCH wider than the floor on purpose. Two quads of equal
    // span put every floor vertex at a CORNER of the ceiling, where roughly three
    // quarters of the sky is still open and the measured occlusion is mild -- a
    // true reading of that geometry, and a poor probe of whether overhead
    // occlusion works at all. Overhanging the ceiling puts the floor's vertices
    // genuinely underneath something.
    Mesh mesh;
    add_quad(mesh, 0.0f, 4.0f, glm::vec3(0.0f, 1.0f, 0.0f));    // floor,   0..3
    add_quad(mesh, 2.0f, 40.0f, glm::vec3(0.0f, 1.0f, 0.0f));   // ceiling, 4..7

    AOSettings settings;
    settings.ray_count = 32;
    settings.max_distance = 10.0f;
    bake_ambient_occlusion(mesh, settings);

    const float below = mean_ao(mesh, 0, 4);
    const float above = mean_ao(mesh, 4, 4);

    // ~0.55 in practice, not the ~0.05 a raw hit count would give: the distance
    // falloff is quadratic over max_distance, so an occluder at 2 m inside a 10 m
    // range contributes roughly half of a full hit. That is the intended
    // behaviour -- tightening max_distance is how you ask for a darker contact --
    // and a_nearer_occluder_darkens_more pins the falloff itself.
    CHECK(below < 0.65f);
    CHECK(above > 0.95f);
    CHECK(below < above - 0.3f);
}

/**
 * @brief Closer occluders darken more than distant ones
 *
 * The distance falloff, and it is not a detail: a binary hit count makes a wall
 * across a street occlude exactly as hard as one this vertex is touching, which
 * shades whole streets as though they were interiors.
 */
TEST(AmbientOcclusion, a_nearer_occluder_darkens_more) {
    AOSettings settings;
    settings.ray_count = 32;
    settings.max_distance = 10.0f;

    Mesh near_mesh;
    add_quad(near_mesh, 0.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    add_quad(near_mesh, 1.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    bake_ambient_occlusion(near_mesh, settings);

    Mesh far_mesh;
    add_quad(far_mesh, 0.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    add_quad(far_mesh, 6.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    bake_ambient_occlusion(far_mesh, settings);

    CHECK(mean_ao(near_mesh, 0, 4) < mean_ao(far_mesh, 0, 4));
}

/**
 * @brief An occluder past max_distance is not seen at all
 *
 * max_distance is the look control, so it has to actually cut off rather than
 * merely attenuate.
 */
TEST(AmbientOcclusion, an_occluder_beyond_max_distance_is_ignored) {
    Mesh mesh;
    add_quad(mesh, 0.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    add_quad(mesh, 12.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));

    AOSettings settings;
    settings.ray_count = 32;
    settings.max_distance = 5.0f;
    bake_ambient_occlusion(mesh, settings);

    CHECK(mean_ao(mesh, 0, 4) > 0.98f);
}

/**
 * @brief The analytic ground plane darkens a wall's base and not its top
 *
 * An OSM building is walls and a roof with NO FLOOR, so a wall's base has nothing
 * beneath it in its own mesh -- and the base is exactly where the eye expects
 * contact darkening. The plane closes the ground without welding a slab under
 * every footprint.
 */
TEST(AmbientOcclusion, the_ground_plane_darkens_the_foot_of_a_wall) {
    // A wall facing +X, standing from y = 0 to y = 6. Two vertices at the base,
    // two at the top, so the gradient along it is directly measurable.
    Mesh mesh;
    const glm::vec3 normal(1.0f, 0.0f, 0.0f);
    const glm::vec3 corners[4] = {
        { 0.0f, 0.0f, -5.0f }, { 0.0f, 0.0f, 5.0f },     // base:  0, 1
        { 0.0f, 6.0f, 5.0f },  { 0.0f, 6.0f, -5.0f },    // top:   2, 3
    };
    for (const glm::vec3& corner : corners) {
        Vertex v{};
        v.position = corner;
        v.normal = normal;
        mesh.vertices.push_back(v);
    }
    const uint32_t order[6] = { 0, 1, 2, 0, 2, 3 };
    for (const uint32_t i : order) mesh.indices.push_back(i);

    AOSettings settings;
    settings.ray_count = 32;
    settings.max_distance = 8.0f;
    settings.use_ground_plane = true;
    settings.ground_height = 0.0f;
    bake_ambient_occlusion(mesh, settings);

    const float base = (mesh.vertices[0].ao + mesh.vertices[1].ao) * 0.5f;
    const float top = (mesh.vertices[2].ao + mesh.vertices[3].ao) * 0.5f;

    CHECK(base < top);
    CHECK(top > 0.95f);   // 6 m up, the ground is most of a hemisphere away

    // With the plane off, the wall has nothing to occlude it and the gradient
    // disappears entirely -- which is the state this setting exists to fix.
    Mesh unoccluded = mesh;
    settings.use_ground_plane = false;
    bake_ambient_occlusion(unoccluded, settings);
    CHECK(unoccluded.vertices[0].ao > 0.98f);
}

/**
 * @brief Every baked value stays inside [min_ao, 1]
 *
 * min_ao is not a safety clamp, it is a look decision: a true 0 reads as a hole in
 * the mesh rather than as a corner. And nothing may exceed 1, which would BRIGHTEN
 * the ambient term.
 */
TEST(AmbientOcclusion, baked_values_stay_within_the_configured_range) {
    // A fully enclosed box: every vertex faces inward, so every hemisphere is shut.
    Mesh mesh;
    add_quad(mesh, 0.0f, 4.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    add_quad(mesh, 1.0f, 4.0f, glm::vec3(0.0f, -1.0f, 0.0f));

    AOSettings settings;
    settings.ray_count = 32;
    settings.min_ao = 0.2f;
    bake_ambient_occlusion(mesh, settings);

    for (const Vertex& v : mesh.vertices) {
        CHECK(v.ao >= 0.2f);
        CHECK(v.ao <= 1.0f);
    }
}

/**
 * @brief Strength 0 leaves the mesh exactly as it was
 *
 * The off switch has to be exact, not approximately 1: it is what a caller uses to
 * skip the bake, and a mesh that came back at 0.999 would be silently darker than
 * one that never went through here.
 */
TEST(AmbientOcclusion, strength_zero_leaves_every_vertex_open) {
    Mesh mesh;
    add_quad(mesh, 0.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    add_quad(mesh, 1.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));

    AOSettings settings;
    settings.strength = 0.0f;
    bake_ambient_occlusion(mesh, settings);

    for (const Vertex& v : mesh.vertices) {
        CHECK_EQ(v.ao, 1.0f);
    }
}

/**
 * @brief The bake does not depend on the thread count
 *
 * Vertices are independent and each is written once, so this is a property the
 * implementation should have for free -- which is exactly why it is worth pinning.
 * A shared accumulator or a per-thread RNG would break it, and the damage would be
 * a mesh that shades differently on a different machine.
 */
TEST(AmbientOcclusion, the_bake_is_deterministic_across_thread_counts) {
    Mesh single;
    add_quad(single, 0.0f, 20.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    add_quad(single, 2.0f, 12.0f, glm::vec3(0.0f, 1.0f, 0.0f));
    Mesh many = single;

    AOSettings settings;
    settings.ray_count = 16;
    settings.thread_count = 1;
    bake_ambient_occlusion(single, settings);

    settings.thread_count = 8;
    bake_ambient_occlusion(many, settings);

    CHECK_EQ(single.vertices.size(), many.vertices.size());
    for (size_t i = 0; i < single.vertices.size(); ++i) {
        CHECK_EQ(single.vertices[i].ao, many.vertices[i].ao);
    }
}

/**
 * @brief An empty or index-free mesh is left alone rather than crashed on
 *
 * Both arrive in practice: a footprint that triangulated to nothing, and a mesh
 * whose indices have not been filled in yet.
 */
TEST(AmbientOcclusion, a_mesh_with_no_triangles_is_left_open) {
    Mesh empty;
    bake_ambient_occlusion(empty);
    CHECK_EQ(empty.vertices.size(), size_t{0});

    Mesh no_indices;
    Vertex v{};
    v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    v.ao = 0.5f;   // must be overwritten with the "no occlusion" value
    no_indices.vertices.push_back(v);
    bake_ambient_occlusion(no_indices);
    CHECK_EQ(no_indices.vertices[0].ao, 1.0f);
}
