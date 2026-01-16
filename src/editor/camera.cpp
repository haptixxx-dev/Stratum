#define GLM_ENABLE_EXPERIMENTAL
#include "editor/camera.hpp"
#include <SDL3/SDL.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

namespace stratum {

Camera::Camera() {
    recalculate_view();
}

void Camera::set_target(const glm::vec3& target) {
    m_forward = glm::normalize(target - m_position);
    m_right = glm::normalize(glm::cross(m_forward, glm::vec3(0, 1, 0)));
    m_up = glm::normalize(glm::cross(m_right, m_forward));
    
    // Recalculate pitch/yaw from forward vector
    m_pitch = glm::degrees(asin(m_forward.y));
    m_yaw = glm::degrees(atan2(m_forward.z, m_forward.x));
    
    m_dirty = true;
}

void Camera::update(float aspect_ratio) {
    if (m_dirty) {
        recalculate_view();
    }
    recalculate_projection(aspect_ratio);
}

void Camera::handle_input(float dt) {
    // Only move if right mouse button is held (standard editor cam)
    auto mouse_state = SDL_GetMouseState(nullptr, nullptr);
    if (!(mouse_state & SDL_BUTTON_RMASK)) {
        return;
    }

    const bool* state = SDL_GetKeyboardState(nullptr);
    float speed = m_speed * dt;
    
    // Boost speed with Shift
    if (state[SDL_SCANCODE_LSHIFT]) {
        speed *= 2.0f;
    }

    glm::vec3 move_dir{0.0f};

    if (state[SDL_SCANCODE_W]) move_dir += m_forward;
    if (state[SDL_SCANCODE_S]) move_dir -= m_forward;
    if (state[SDL_SCANCODE_D]) move_dir += m_right;
    if (state[SDL_SCANCODE_A]) move_dir -= m_right;
    if (state[SDL_SCANCODE_E]) move_dir += glm::vec3(0, 1, 0); // Up
    if (state[SDL_SCANCODE_Q]) move_dir -= glm::vec3(0, 1, 0); // Down

    if (glm::length(move_dir) > 0.0f) {
        m_position += glm::normalize(move_dir) * speed;
        m_dirty = true;
    }

    // Mouse Rotation
    float xrel, yrel;
    SDL_GetRelativeMouseState(&xrel, &yrel);

    if (xrel != 0 || yrel != 0) {
        m_yaw += xrel * m_sensitivity;
        m_pitch -= yrel * m_sensitivity;

        // Clamp pitch
        m_pitch = std::clamp(m_pitch, -89.0f, 89.0f);

        // Update vectors
        glm::vec3 front;
        front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
        front.y = sin(glm::radians(m_pitch));
        front.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
        m_forward = glm::normalize(front);
        m_right = glm::normalize(glm::cross(m_forward, glm::vec3(0, 1, 0)));
        m_up = glm::normalize(glm::cross(m_right, m_forward));
        
        m_dirty = true;
    }
}

void Camera::recalculate_view() {
    m_view = glm::lookAt(m_position, m_position + m_forward, m_up);
    m_dirty = false;
    m_view_projection = m_projection * m_view;
}

void Camera::recalculate_projection(float aspect_ratio) {
    m_projection = glm::perspective(glm::radians(m_fov), aspect_ratio, m_near, m_far);
    m_view_projection = m_projection * m_view;
}

} // namespace stratum
