#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <array>

namespace stratum {

/**
 * @brief Frustum for culling - 6 planes extracted from view-projection matrix
 */
struct Frustum {
    // Plane equation: ax + by + cz + d = 0, stored as vec4(a,b,c,d)
    std::array<glm::vec4, 6> planes;

    // Extract frustum planes from view-projection matrix
    void extract(const glm::mat4& vp) {
        // Left
        planes[0] = glm::vec4(vp[0][3] + vp[0][0], vp[1][3] + vp[1][0],
                              vp[2][3] + vp[2][0], vp[3][3] + vp[3][0]);
        // Right
        planes[1] = glm::vec4(vp[0][3] - vp[0][0], vp[1][3] - vp[1][0],
                              vp[2][3] - vp[2][0], vp[3][3] - vp[3][0]);
        // Bottom
        planes[2] = glm::vec4(vp[0][3] + vp[0][1], vp[1][3] + vp[1][1],
                              vp[2][3] + vp[2][1], vp[3][3] + vp[3][1]);
        // Top
        planes[3] = glm::vec4(vp[0][3] - vp[0][1], vp[1][3] - vp[1][1],
                              vp[2][3] - vp[2][1], vp[3][3] - vp[3][1]);
        // Near
        planes[4] = glm::vec4(vp[0][3] + vp[0][2], vp[1][3] + vp[1][2],
                              vp[2][3] + vp[2][2], vp[3][3] + vp[3][2]);
        // Far
        planes[5] = glm::vec4(vp[0][3] - vp[0][2], vp[1][3] - vp[1][2],
                              vp[2][3] - vp[2][2], vp[3][3] - vp[3][2]);

        // Normalize planes
        for (auto& p : planes) {
            float len = glm::length(glm::vec3(p));
            if (len > 0.0001f) p /= len;
        }
    }

    // Test if AABB intersects frustum
    bool intersects_aabb(const glm::vec3& min, const glm::vec3& max) const {
        for (const auto& plane : planes) {
            glm::vec3 p = min;
            if (plane.x >= 0) p.x = max.x;
            if (plane.y >= 0) p.y = max.y;
            if (plane.z >= 0) p.z = max.z;

            if (glm::dot(glm::vec3(plane), p) + plane.w < 0) {
                return false; // Outside this plane
            }
        }
        return true; // Inside or intersecting all planes
    }
};

class Camera {
public:
    Camera();

    void update(float aspect_ratio);
    void handle_input(float dt);

    const glm::mat4& get_view() const { return m_view; }
    const glm::mat4& get_projection() const { return m_projection; }
    const glm::mat4& get_view_projection() const { return m_view_projection; }
    Frustum get_frustum() const { Frustum f; f.extract(m_view_projection); return f; }
    std::array<glm::vec4, 6> get_frustum_planes() const { return get_frustum().planes; }
    
    glm::vec3 get_position() const { return m_position; }
    glm::vec3 get_forward() const { return m_forward; }
    glm::vec3 get_right() const { return m_right; }
    glm::vec3 get_up() const { return m_up; }

    void set_position(const glm::vec3& pos) { m_position = pos; m_dirty = true; }
    void set_target(const glm::vec3& target);

    // Camera settings
    float m_fov = 45.0f;
    float m_near = 0.1f;
    float m_far = 1000.0f;
    float m_speed = 10.0f;
    float m_sensitivity = 0.1f;

private:
    void recalculate_view();
    void recalculate_projection(float aspect_ratio);

    glm::vec3 m_position{0.0f, 10.0f, 20.0f};
    glm::vec3 m_forward{0.0f, 0.0f, -1.0f};
    glm::vec3 m_up{0.0f, 1.0f, 0.0f};
    glm::vec3 m_right{1.0f, 0.0f, 0.0f};
    
    // Euler angles
    float m_yaw = -90.0f;
    float m_pitch = -25.0f;

    float m_yaw_old = m_yaw;
    float m_pitch_old = m_pitch;

    bool m_was_rotating = false;

    glm::mat4 m_view{1.0f};
    glm::mat4 m_projection{1.0f};
    glm::mat4 m_view_projection{1.0f};

    bool m_dirty = true;
};

} // namespace stratum
