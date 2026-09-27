// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>

namespace stratum {

void Editor::frame_camera_on_data(bool recenter_camera) {
    // Find geometry center for camera positioning
    glm::vec3 bounds_min, bounds_max;
    m_quadtree.get_bounds(bounds_min, bounds_max);
    bool found_geometry = (bounds_min.x < bounds_max.x || bounds_min.z < bounds_max.z);
    glm::vec3 data_center = (bounds_min + bounds_max) * 0.5f;

    if (!found_geometry) {
        spdlog::warn("No geometry found in quadtree!");
    }

    spdlog::info("Data center: ({}, {}, {})", data_center.x, data_center.y, data_center.z);

    // Center camera on data FIRST (before culling uses camera position)
    glm::vec3 focus_centre;
    float focus_radius = 0.0f;
    const bool have_focus = m_quadtree.get_focus(focus_centre, focus_radius);

    if (have_focus && recenter_camera) {
        // Frame where the features actually are, not the centre of their bounding
        // box. Those differ wildly for an Overpass export, whose box is stretched
        // by the nodes it pulls in for ways crossing the query area.
        data_center = focus_centre;

        // Scale to the data. The old fixed 300m/5000m numbers meant a large import
        // put the camera thousands of metres from anything, so distance culling
        // rejected every node, nothing was ever queued, and the viewport stayed
        // empty with no error shown.
        const float view_distance = std::clamp(focus_radius, 300.0f, 8000.0f);
        glm::vec3 cam_pos = data_center + glm::vec3(0.0f, view_distance * 0.8f, view_distance);

        m_camera.set_position(cam_pos);
        m_camera.set_target(data_center);
        // Depth precision is governed by far/near, so keep that ratio sane rather
        // than pairing a 0.1m near plane with a far plane tens of km out -- that
        // combination puts almost the whole depth buffer in the first few metres
        // and leaves coplanar roads, landuse and building footprints z-fighting.
        // Only draw as far as nodes are actually built, plus headroom.
        m_camera.m_far = std::clamp(view_distance * 6.0f, 20000.0f, 80000.0f);
        m_camera.m_near = std::clamp(m_camera.m_far / 20000.0f, 0.1f, 5.0f);
        m_camera.m_base_speed = std::clamp(focus_radius * 0.1f, 200.0f, 5000.0f);
        // Must reach past the camera's own distance from the data, or distance
        // culling rejects everything before it can be built. Bounded so a huge
        // import does not try to mesh the whole dataset at once -- the remainder
        // streams in as the camera moves.
        m_view_radius = std::clamp(view_distance * 3.0f, 5000.0f, 30000.0f);

        spdlog::info("Camera at ({:.0f}, {:.0f}, {:.0f}) looking at ({:.0f}, {:.0f}, {:.0f}), "
                     "focus radius {:.0f}m, view radius {:.0f}m",
                     cam_pos.x, cam_pos.y, cam_pos.z,
                     data_center.x, data_center.y, data_center.z,
                     focus_radius, m_view_radius);
    } else if (have_focus) {
        // A road rebuild after terrain generation. The geometry is the same data
        // in the same place, so re-framing it would only throw away wherever the
        // user was looking.
        data_center = focus_centre;
        spdlog::info("Road rebuild: leaving the camera where it is");
    } else if (found_geometry) {
        spdlog::warn("No populated quadtree leaves; leaving the camera where it is");
    }

    // Force camera matrix recalculation so frustum matches new position
    // (update() is normally called in draw_viewport, but we need it now for traversal)
    m_camera.update(1.0f); // aspect doesn't matter much, just need valid frustum
}

} // namespace stratum
