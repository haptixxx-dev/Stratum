// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include <im3d.h>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace stratum {

// ============================================================================
// Colour-by-attribute viewport mode
//
// Everything below is wiring. Not one colour is decided here: every one comes
// from osm/attribute_palette.hpp, which is in stratum_core precisely so that the
// question "does Retail collide with Apartments" is answerable by a test rather
// than by squinting at a screenshot.
// ============================================================================

namespace {

/// Lift for a ground-level outline, in metres.
///
/// Areas are already built 0.01 to 0.03 above the ground to keep them out of a
/// z-fight with the terrain; the overlay has to clear THOSE as well, or the mode
/// disappears into the very geometry it is describing.
constexpr float kOverlayGroundLift = 0.25f;

/// Lift for a road centreline, in metres.
///
/// Higher than the ground lift because a road centreline is drawn over the road
/// SURFACE, which the corridor extruder gives a crown and a kerb reveal of its
/// own.
constexpr float kOverlayRoadLift = 0.5f;

/// Overlay line width in pixels. Im3d sizes lines in screen space, so this holds
/// up at any camera distance, which is the point of using it over real geometry.
constexpr float kOverlayLineWidth = 2.0f;

/// How far a building ring is pushed clear of the wall it sits on, in metres.
///
/// A lift alone does not save these two rings, because neither of them is
/// z-fighting the GROUND. build_building_mesh() extrudes the walls from the very
/// same footprint polygon, so the footprint ring lies exactly IN the wall plane
/// and the roof ring lies exactly ON the wall's top edge. The Im3d pipeline sets
/// enable_depth_bias = false and compares with GREATER, and Im3D_Render() runs
/// after the meshes have written depth, so a line fragment at a depth equal to
/// the wall's is discarded outright. The visible symptom is that every near-face
/// edge of a building drops out or flickers while the silhouette survives, the
/// silhouette being the only part where Im3d's 2 px screen-space expansion
/// spills past the mesh.
///
/// So the rings are moved out of the surface instead: the footprint ring outward
/// along the ring's own normal, the roof ring straight up off the eaves.
constexpr float kOverlayWallClearance = 0.08f;

/// Extra lift for the roof ring, in metres, above building.height.
constexpr float kOverlayRoofClearance = 0.15f;

/// 2D local metres to Y-up world space.
///
/// The Z flip is the whole convention and it is duplicated from
/// osm/mesh_builder.cpp's to_world() rather than shared, because sharing it would
/// mean exporting a renderer-facing helper out of stratum_core for the sake of
/// three characters. Getting it wrong mirrors the entire city about the origin,
/// which is at least unmissable.
Im3d::Vec3 overlay_point(const glm::dvec2& p, float y) {
    return Im3d::Vec3(static_cast<float>(p.x), y, static_cast<float>(-p.y));
}

Im3d::Color overlay_colour(const glm::vec4& c) {
    // Im3d::Color's float overload takes components in 0..1. The int overload
    // takes packed 0xRRGGBBAA and silently produces near-black from 0..255
    // components; see the note on the node grid above.
    return Im3d::Color(c.r, c.g, c.b, c.a);
}

/// Vertex count of a ring, ignoring the closing duplicate an OSM way carries.
size_t overlay_ring_span(const std::vector<glm::dvec2>& ring) {
    if (ring.size() > 2 && ring.front() == ring.back()) {
        return ring.size() - 1;
    }
    return ring.size();
}

/// One closed outline at a fixed height.
///
/// `inflate` pushes each vertex along the ring's outward normal, which is what
/// keeps a building ring out of the wall plane it would otherwise share. Zero
/// for anything that is not coplanar with a vertical surface -- an area polygon
/// has no walls, so inflating it would just misreport its extent.
void draw_overlay_ring(const std::vector<glm::dvec2>& ring, float y, Im3d::Color colour,
                       double inflate = 0.0) {
    const size_t n = overlay_ring_span(ring);
    if (n < 3) return;

    // Winding decides which side "outward" is on. The shoelace sign is the only
    // thing that knows, and an OSM outer ring is not reliably counter-clockwise.
    double area2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % n];
        area2 += a.x * b.y - b.x * a.y;
    }
    const double sign = area2 < 0.0 ? -1.0 : 1.0;

    Im3d::BeginLineLoop();
    for (size_t i = 0; i < n; ++i) {
        glm::dvec2 p = ring[i];

        if (inflate != 0.0) {
            // Bisector of the two edge normals at this vertex. Normalising each
            // edge first keeps a long edge from dominating a short one, which is
            // what makes a corner of a thin building bulge.
            const glm::dvec2& prev = ring[(i + n - 1) % n];
            const glm::dvec2& next = ring[(i + 1) % n];
            const glm::dvec2 e0 = p - prev;
            const glm::dvec2 e1 = next - p;

            glm::dvec2 bisector(0.0);
            if (glm::length(e0) > 1e-9) {
                const glm::dvec2 u = glm::normalize(e0);
                bisector += glm::dvec2(u.y, -u.x);
            }
            if (glm::length(e1) > 1e-9) {
                const glm::dvec2 u = glm::normalize(e1);
                bisector += glm::dvec2(u.y, -u.x);
            }
            if (glm::length(bisector) > 1e-9) {
                p += glm::normalize(bisector) * inflate * sign;
            }
        }

        Im3d::Vertex(overlay_point(p, y), colour);
    }
    Im3d::End();
}

/// One open polyline, lifted by `y` above the surface `sampler` reports.
///
/// The sampler is not optional decoration. Road geometry is DRAPED: whenever
/// terrain exists, make_road_network_config() hands the solver a height sampler
/// and RoadElevationSolver lifts every piece onto the surface. Road::polyline is
/// the raw 2D input and carries no elevation at all, so drawing it at a constant
/// world Y puts the whole overlay tens of metres under the hills -- and Im3d
/// depth-tests, so it is not merely wrong, it is invisible. The panel then reads
/// "Showing 4312" over an empty viewport, which is exactly the "everything is
/// unclassified" misreading this mode exists to prevent.
///
/// A null sampler means no terrain, in which case a flat lift is correct.
void draw_overlay_strip(const std::vector<glm::dvec2>& points, float y, Im3d::Color colour,
                        const osm::road::HeightSampler& sampler) {
    if (points.size() < 2) return;

    Im3d::BeginLineStrip();
    for (const glm::dvec2& p : points) {
        const float base = sampler ? sampler(p.x, p.y) : 0.0f;
        Im3d::Vertex(overlay_point(p, base + y), colour);
    }
    Im3d::End();
}

/// The twelve edges of a leaf's bounds.
void draw_overlay_box(const glm::vec3& mn, const glm::vec3& mx, Im3d::Color colour) {
    const glm::vec3 corners[8] = {
        {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z}, {mx.x, mn.y, mx.z}, {mn.x, mn.y, mx.z},
        {mn.x, mx.y, mn.z}, {mx.x, mx.y, mn.z}, {mx.x, mx.y, mx.z}, {mn.x, mx.y, mx.z},
    };
    static constexpr int kEdges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    };

    Im3d::BeginLines();
    for (const auto& edge : kEdges) {
        const glm::vec3& a = corners[edge[0]];
        const glm::vec3& b = corners[edge[1]];
        Im3d::Vertex(Im3d::Vec3(a.x, a.y, a.z), colour);
        Im3d::Vertex(Im3d::Vec3(b.x, b.y, b.z), colour);
    }
    Im3d::End();
}

} // namespace

glm::vec4 Editor::attribute_leaf_tint(const osm::QuadTreeNode& node) const {
    if (m_attribute_mode != osm::AttributeMode::Tile) {
        return glm::vec4(1.0f);
    }

    // A leaf's own rectangle is its tile, so dividing its centre by its own edge
    // length gives integer grid coordinates. That is what tile_colour_index()'s
    // no-two-neighbours-alike guarantee is stated over. Leaves at different depths
    // sit on different grids and can land on the same colour across a depth
    // change, which does not hide the seam: a depth change is a size change, and
    // the size is already visible.
    const double edge = node.half_size * 2.0;
    if (edge <= 0.0) {
        return glm::vec4(1.0f);
    }

    const auto tile_x = static_cast<int64_t>(std::floor(node.center.x / edge));
    const auto tile_z = static_cast<int64_t>(std::floor(node.center.y / edge));
    return osm::tile_colour(osm::tile_colour_index(tile_x, tile_z));
}

glm::vec4 Editor::attribute_chunk_tint(const procgen::TerrainChunkCoord& coord) const {
    if (m_attribute_mode != osm::AttributeMode::Tile) {
        return glm::vec4(1.0f);
    }
    return osm::tile_colour(osm::tile_colour_index(coord.x, coord.z));
}

void Editor::draw_attribute_overlay() {
    using osm::AttributeMode;

    m_attribute_features_drawn = 0;
    m_attribute_features_seen = 0;

    if (m_attribute_mode == AttributeMode::None || m_quadtree.leaf_count() == 0) {
        return;
    }

    const Frustum frustum = m_camera.get_frustum();
    const glm::vec3 cam_pos = m_camera.get_position();
    const float radius_sq = m_model.m_view_radius * m_model.m_view_radius;

    // Gathered and sorted front to back before anything is emitted, so that when
    // the budget runs out it runs out on the far side of the scene. Sorting a few
    // hundred leaf pointers costs nothing next to the Im3d upload it protects.
    std::vector<std::pair<float, osm::QuadTreeNode*>> visible;
    for (auto* leaf : m_quadtree.get_all_leaves()) {
        if (!leaf || !leaf->has_valid_bounds()) continue;
        if (m_model.m_use_tile_culling && !frustum.intersects_aabb(leaf->bounds_min, leaf->bounds_max)) {
            continue;
        }

        const glm::vec3 centre = (leaf->bounds_min + leaf->bounds_max) * 0.5f;
        const glm::vec3 to_cam = centre - cam_pos;
        const float dist_sq = glm::dot(to_cam, to_cam);
        if (m_model.m_use_distance_culling && dist_sq > radius_sq) continue;

        visible.emplace_back(dist_sq, leaf);
    }

    std::sort(visible.begin(), visible.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    // The SAME sampler the road solve was given, built once for the whole pass
    // rather than per road: make_terrain_height_sampler() constructs a
    // TerrainGenerator, and doing that a few thousand times a frame would cost
    // more than the overlay itself. Null when there is no terrain, which the
    // strip drawer reads as a flat world.
    const osm::road::HeightSampler overlay_height = make_terrain_height_sampler();

    Im3d::PushDrawState();
    Im3d::SetSize(kOverlayLineWidth);

    for (const auto& entry : visible) {
        const osm::QuadTreeNode& leaf = *entry.second;

        // The budget is checked per FEATURE rather than per leaf: a single dense
        // leaf can hold thousands, and stopping at a leaf boundary would drop a
        // whole city block at once.
        switch (m_attribute_mode) {
            case AttributeMode::Building:
                for (const osm::Building& building : leaf.buildings) {
                    ++m_attribute_features_seen;
                    if (m_attribute_features_drawn >= kAttributeOverlayBudget) continue;

                    const Im3d::Color colour =
                        overlay_colour(osm::building_type_colour(building.type));

                    // Both rings, not just the roof. The roof ring alone floats
                    // free of its own footprint at street level on anything taller
                    // than a house, and the footprint ring alone is invisible from
                    // above once the roof covers it.
                    draw_overlay_ring(building.footprint, kOverlayGroundLift, colour,
                                      kOverlayWallClearance);
                    draw_overlay_ring(building.footprint,
                                      building.height + kOverlayRoofClearance, colour,
                                      kOverlayWallClearance);
                    ++m_attribute_features_drawn;
                }
                break;

            case AttributeMode::Road:
                for (const osm::Road& road : leaf.roads) {
                    ++m_attribute_features_seen;
                    if (m_attribute_features_drawn >= kAttributeOverlayBudget) continue;

                    draw_overlay_strip(road.polyline, kOverlayRoadLift,
                                       overlay_colour(osm::road_type_colour(road.type)),
                                       overlay_height);
                    ++m_attribute_features_drawn;
                }
                break;

            case AttributeMode::Area:
                for (const osm::Area& area : leaf.areas) {
                    ++m_attribute_features_seen;
                    if (m_attribute_features_drawn >= kAttributeOverlayBudget) continue;

                    draw_overlay_ring(area.polygon, kOverlayGroundLift,
                                      overlay_colour(osm::area_type_colour(area.type)));
                    ++m_attribute_features_drawn;
                }
                break;

            case AttributeMode::Tile:
                // render_3d() has already tinted the solid geometry. All that is
                // left to add is where one tile stops and the next starts, which
                // the tint cannot show on its own where two neighbours are both
                // empty of geometry.
                ++m_attribute_features_seen;
                if (m_attribute_features_drawn < kAttributeOverlayBudget) {
                    draw_overlay_box(leaf.bounds_min, leaf.bounds_max,
                                     overlay_colour(attribute_leaf_tint(leaf)));
                    ++m_attribute_features_drawn;
                }
                break;

            case AttributeMode::None:
            case AttributeMode::Count:
                break;
        }
    }

    Im3d::PopDrawState();
}

} // namespace stratum
