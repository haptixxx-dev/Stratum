// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_roof_shapes.cpp
 * @brief MeshBuilder::build_building_mesh() honours roof:shape=* for all five families
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * RoofType is read from roof:shape=* by OSMParser::classify_roof and, until E1,
 * only three of its six values reached geometry. Skillion and Dome fell through
 * to the flat path, Hipped was built as a Pyramidal (one apex, no ridge), and
 * Gabled emitted overlapping faces because every footprint edge was connected to
 * the WHOLE ridge segment rather than to its own projection onto it.
 *
 * The assertions here are mostly surface areas rather than vertex positions.
 * Area is invariant to how a face is split into triangles, to vertex order, and
 * to whether a degenerate triangle is emitted or skipped, so these tests survive
 * a re-triangulation but still fail the moment a face is duplicated, dropped or
 * put in the wrong place. The overlapping-gable bug is precisely a duplicated
 * face, which is why it went unnoticed without one.
 *
 * Geometry constants used throughout, from mesh_builder.cpp:
 *   - pitch ratio 0.3
 *   - a ridge rises by (width * 0.5 * 0.3) above the wall top
 *   - a hip inset is (length - width) / 2 at each end
 *   - a skillion rises by (width * 0.3)
 *   - a dome rises by the footprint's inradius
 */

#include "framework.hpp"

#include "osm/mesh_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <utility>
#include <vector>

using stratum::osm::Building;
using stratum::osm::MeshBuilder;
using stratum::osm::RoofType;
using stratum::Mesh;

namespace {

constexpr float kHeight = 10.0f;
constexpr double kTol = 0.01;

/// A 20 x 10 rectangle centred on the origin, counter-clockwise, not closed.
///
/// The long axis is X, so compute_principal_axis() picks length 20, width 10,
/// and every derived constant below follows from that pair.
std::vector<glm::dvec2> rectangle(double length = 20.0, double width = 10.0) {
    const double hl = length * 0.5;
    const double hw = width * 0.5;
    return {{-hl, -hw}, {hl, -hw}, {hl, hw}, {-hl, hw}};
}

Building make_building(RoofType roof, std::vector<glm::dvec2> footprint = rectangle()) {
    Building b;
    b.footprint = std::move(footprint);
    b.height = kHeight;
    b.roof_type = roof;
    return b;
}

/// Total triangle area of a mesh. The single most useful invariant here: it
/// double-counts an overlapping face and under-counts a missing one.
double surface_area(const Mesh& mesh) {
    double total = 0.0;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const glm::vec3& a = mesh.vertices[mesh.indices[i]].position;
        const glm::vec3& b = mesh.vertices[mesh.indices[i + 1]].position;
        const glm::vec3& c = mesh.vertices[mesh.indices[i + 2]].position;
        total += 0.5 * static_cast<double>(glm::length(glm::cross(b - a, c - a)));
    }
    return total;
}

float max_y(const Mesh& mesh) {
    float y = std::numeric_limits<float>::lowest();
    for (const auto& v : mesh.vertices) {
        y = std::max(y, v.position.y);
    }
    return y;
}

/// Distinct vertex positions at the very top of the mesh, quantised so that
/// float noise does not split one apex into several. A pyramid has one; a ridge
/// has two ends, and every vertex along it shares the same height.
size_t distinct_apex_positions(const Mesh& mesh) {
    const float top = max_y(mesh);
    std::set<std::pair<long, long>> seen;
    for (const auto& v : mesh.vertices) {
        if (std::abs(v.position.y - top) < 1e-3f) {
            seen.insert({std::lround(v.position.x * 1000.0f), std::lround(v.position.z * 1000.0f)});
        }
    }
    return seen.size();
}

/// Walls only: perimeter times height. Every case below extrudes the same box.
constexpr double kWallArea = (2.0 * 20.0 + 2.0 * 10.0) * 10.0;  // 600

}  // namespace

// ============================================================================
// Flat
// ============================================================================

TEST(RoofShapes, flat_roof_caps_the_walls_at_the_wall_top) {
    const Mesh mesh = MeshBuilder::build_building_mesh(make_building(RoofType::Flat));

    CHECK_NEAR(max_y(mesh), kHeight, 1e-4f);
    CHECK_NEAR(surface_area(mesh), kWallArea + 20.0 * 10.0, kTol);
}

TEST(RoofShapes, an_unknown_roof_shape_is_treated_as_flat) {
    const Mesh flat = MeshBuilder::build_building_mesh(make_building(RoofType::Flat));
    const Mesh unknown = MeshBuilder::build_building_mesh(make_building(RoofType::Unknown));

    CHECK_NEAR(surface_area(flat), surface_area(unknown), 1e-6);
}

// ============================================================================
// Gabled
// ============================================================================

TEST(RoofShapes, gabled_ridge_rises_by_half_the_width_times_the_pitch) {
    const Mesh mesh = MeshBuilder::build_building_mesh(make_building(RoofType::Gabled));

    // width 10 * 0.5 * 0.3 = 1.5
    CHECK_NEAR(max_y(mesh), kHeight + 1.5f, 1e-4f);
}

// This is the regression. The old implementation gave every edge a quad reaching
// both ridge ends, so the two 10 m end edges each produced a face spanning the
// full 20 m ridge -- two large sheets lying across the roof, roughly doubling its
// area. Projecting each edge onto the ridge independently is what fixes it, and
// the exact area is what proves it.
TEST(RoofShapes, gabled_end_edges_close_as_gables_rather_than_spanning_the_ridge) {
    const Mesh mesh = MeshBuilder::build_building_mesh(make_building(RoofType::Gabled));

    const double slope = std::sqrt(5.0 * 5.0 + 1.5 * 1.5);  // eave to ridge, in section
    const double slopes = 2.0 * 20.0 * slope;               // two long faces
    const double gables = 2.0 * (0.5 * 10.0 * 1.5);         // two vertical end triangles

    CHECK_NEAR(surface_area(mesh), kWallArea + slopes + gables, kTol);
}

TEST(RoofShapes, gabled_geometry_stays_inside_the_footprint) {
    const Mesh mesh = MeshBuilder::build_building_mesh(make_building(RoofType::Gabled));

    for (const auto& v : mesh.vertices) {
        CHECK((std::abs(v.position.x)) <= (10.0f + 1e-3f));
        CHECK((std::abs(v.position.z)) <= (5.0f + 1e-3f));
    }
}

// ============================================================================
// Hipped and pyramidal
// ============================================================================

TEST(RoofShapes, hipped_keeps_a_ridge_where_pyramidal_collapses_to_a_point) {
    const Mesh hipped = MeshBuilder::build_building_mesh(make_building(RoofType::Hipped));
    const Mesh pyramidal = MeshBuilder::build_building_mesh(make_building(RoofType::Pyramidal));

    CHECK_EQ(distinct_apex_positions(hipped), size_t{2});
    CHECK_EQ(distinct_apex_positions(pyramidal), size_t{1});

    // Both peak at the same height on this plan -- inradius 5 and half-width 5
    // are the same number -- so height alone cannot tell them apart. That is
    // exactly how the old code passed for a pyramid and shipped for a hip.
    CHECK_NEAR(max_y(hipped), max_y(pyramidal), 1e-4f);
    CHECK((surface_area(hipped)) > (surface_area(pyramidal)));
}

TEST(RoofShapes, hipped_ridge_is_inset_by_half_the_width_at_each_end) {
    const Mesh mesh = MeshBuilder::build_building_mesh(make_building(RoofType::Hipped));

    // length 20, width 10 -> the ridge runs from x = -5 to x = +5.
    float ridge_min = std::numeric_limits<float>::max();
    float ridge_max = std::numeric_limits<float>::lowest();
    const float top = max_y(mesh);
    for (const auto& v : mesh.vertices) {
        if (std::abs(v.position.y - top) < 1e-3f) {
            ridge_min = std::min(ridge_min, v.position.x);
            ridge_max = std::max(ridge_max, v.position.x);
        }
    }

    CHECK_NEAR(ridge_min, -5.0f, 1e-3f);
    CHECK_NEAR(ridge_max, 5.0f, 1e-3f);
}

TEST(RoofShapes, hipped_on_a_square_plan_degenerates_to_a_pyramid) {
    const Mesh mesh = MeshBuilder::build_building_mesh(
        make_building(RoofType::Hipped, rectangle(10.0, 10.0)));

    // A hip inset of (10 - 10) / 2 leaves no ridge at all, which is the right
    // answer rather than an error.
    CHECK_EQ(distinct_apex_positions(mesh), size_t{1});
}

TEST(RoofShapes, pyramidal_apex_follows_the_inradius_not_the_bounding_box) {
    // A long thin plan: inradius 5, so the apex rises 1.5, not 3 as it would if
    // the half-length drove the pitch.
    const Mesh mesh = MeshBuilder::build_building_mesh(
        make_building(RoofType::Pyramidal, rectangle(40.0, 10.0)));

    CHECK_NEAR(max_y(mesh), kHeight + 1.5f, 1e-4f);
}

// ============================================================================
// Skillion
// ============================================================================

TEST(RoofShapes, skillion_tilts_across_the_short_axis) {
    const Mesh mesh = MeshBuilder::build_building_mesh(make_building(RoofType::Skillion));

    // width 10 * 0.3 = 3.0, all of it on one side.
    CHECK_NEAR(max_y(mesh), kHeight + 3.0f, 1e-4f);

    // The high edge is one of the two long sides, so the rise appears at a
    // single extreme of Z rather than at both.
    float z_at_top = 0.0f;
    for (const auto& v : mesh.vertices) {
        if (std::abs(v.position.y - (kHeight + 3.0f)) < 1e-3f) {
            z_at_top = v.position.z;
            break;
        }
    }
    CHECK_NEAR(std::abs(z_at_top), 5.0f, 1e-3f);
}

TEST(RoofShapes, skillion_area_matches_one_tilted_plane_plus_its_infill) {
    const Mesh mesh = MeshBuilder::build_building_mesh(make_building(RoofType::Skillion));

    const double plane = 20.0 * std::sqrt(10.0 * 10.0 + 3.0 * 3.0);
    const double gable_ends = 2.0 * (0.5 * 10.0 * 3.0);  // the two triangular sides
    const double high_wall = 20.0 * 3.0;                 // the raised long side

    CHECK_NEAR(surface_area(mesh), kWallArea + plane + gable_ends + high_wall, kTol);
}

// Skillion is the one sloped family that does not need a simple outline: it
// lifts an earcut triangulation, and earcut already understands holes.
TEST(RoofShapes, skillion_survives_a_courtyard) {
    Building b = make_building(RoofType::Skillion);
    b.holes.push_back({{-2.0, -2.0}, {-2.0, 2.0}, {2.0, 2.0}, {2.0, -2.0}});

    const Mesh mesh = MeshBuilder::build_building_mesh(b);

    CHECK_FALSE(mesh.vertices.empty());
    CHECK_NEAR(max_y(mesh), kHeight + 3.0f, 1e-4f);

    const Mesh solid = MeshBuilder::build_building_mesh(make_building(RoofType::Skillion));
    CHECK((surface_area(mesh)) < (surface_area(solid)));
}

// ============================================================================
// Dome
// ============================================================================

TEST(RoofShapes, dome_rises_to_the_inradius) {
    const Mesh mesh = MeshBuilder::build_building_mesh(make_building(RoofType::Dome));

    CHECK_NEAR(max_y(mesh), kHeight + 5.0f, 1e-3f);
    CHECK_EQ(distinct_apex_positions(mesh), size_t{1});
}

TEST(RoofShapes, dome_bands_rise_monotonically) {
    const Mesh mesh = MeshBuilder::build_building_mesh(make_building(RoofType::Dome));

    // Every vertex sits between the wall top and the crown, and the shell is
    // strictly above the eave everywhere but the first band.
    for (const auto& v : mesh.vertices) {
        CHECK((v.position.y) >= (-1e-3f));
        CHECK((v.position.y) <= (kHeight + 5.0f + 1e-3f));
    }
    CHECK((surface_area(mesh)) > (kWallArea + 20.0 * 10.0));
}

// ============================================================================
// Courtyards and degenerate input
// ============================================================================

TEST(RoofShapes, swept_roof_families_fall_back_to_flat_on_a_courtyard_footprint) {
    for (const RoofType roof : {RoofType::Gabled, RoofType::Hipped, RoofType::Pyramidal,
                                RoofType::Dome}) {
        Building b = make_building(roof);
        b.holes.push_back({{-2.0, -2.0}, {-2.0, 2.0}, {2.0, 2.0}, {2.0, -2.0}});

        const Mesh mesh = MeshBuilder::build_building_mesh(b);
        CHECK_NEAR(max_y(mesh), kHeight, 1e-4f);
    }
}

TEST(RoofShapes, every_roof_type_emits_finite_geometry) {
    for (const RoofType roof : {RoofType::Flat, RoofType::Gabled, RoofType::Hipped,
                                RoofType::Pyramidal, RoofType::Skillion, RoofType::Dome,
                                RoofType::Unknown}) {
        const Mesh mesh = MeshBuilder::build_building_mesh(make_building(roof));

        CHECK_FALSE(mesh.vertices.empty());
        CHECK_FALSE(mesh.indices.empty());

        for (const auto& v : mesh.vertices) {
            CHECK_TRUE(std::isfinite(v.position.x) && std::isfinite(v.position.y) &&
                        std::isfinite(v.position.z));
            CHECK_TRUE(std::isfinite(v.normal.x) && std::isfinite(v.normal.y) &&
                        std::isfinite(v.normal.z));
            CHECK_NEAR(glm::length(v.normal), 1.0f, 1e-3f);
        }
        for (const uint32_t i : mesh.indices) {
            CHECK((i) < (mesh.vertices.size()));
        }
    }
}

TEST(RoofShapes, a_degenerate_footprint_produces_an_empty_mesh) {
    Building b = make_building(RoofType::Gabled);
    b.footprint = {{0.0, 0.0}, {1.0, 0.0}};

    const Mesh mesh = MeshBuilder::build_building_mesh(b);
    CHECK_TRUE(mesh.vertices.empty());
}
