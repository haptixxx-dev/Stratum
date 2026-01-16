#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace stratum {

class Camera {
public:
    Camera();

    void update(float aspect_ratio);
    void handle_input(float dt);

    const glm::mat4& get_view() const { return m_view; }
    const glm::mat4& get_projection() const { return m_projection; }
    const glm::mat4& get_view_projection() const { return m_view_projection; }
    
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

    glm::mat4 m_view{1.0f};
    glm::mat4 m_projection{1.0f};
    glm::mat4 m_view_projection{1.0f};

    bool m_dirty = true;
};

} // namespace stratum
