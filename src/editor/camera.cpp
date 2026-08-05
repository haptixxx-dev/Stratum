#define GLM_ENABLE_EXPERIMENTAL
#include "editor/camera.hpp"
#include <SDL3/SDL.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

namespace stratum {

Camera::Camera() {
    // Derive the basis from m_yaw/m_pitch before the first view matrix. Without
    // this the camera starts with the default m_forward of (0,0,-1) -- level --
    // while m_pitch claims -25 degrees, and the two only reconcile once the user
    // right-drags, because that was the sole place the basis got recomputed.
    update_orientation_from_angles();
    recalculate_view();
}

void Camera::update_orientation_from_angles() {
    m_pitch = std::clamp(m_pitch, -89.0f, 89.0f);

    glm::vec3 front;
    front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    front.y = sin(glm::radians(m_pitch));
    front.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));

    m_forward = glm::normalize(front);
    m_right = glm::normalize(glm::cross(m_forward, glm::vec3(0, 1, 0)));
    m_up = glm::normalize(glm::cross(m_right, m_forward));

    m_dirty = true;
}

void Camera::set_target(const glm::vec3& target) {
    const glm::vec3 to_target = target - m_position;
    if (glm::length(to_target) < 1e-6f) {
        return;  // Degenerate: normalize() would produce NaNs and wreck the view matrix
    }

    // Derive the angles from the requested direction, then rebuild the basis from
    // them, so the angles and the basis cannot disagree.
    const glm::vec3 dir = glm::normalize(to_target);
    m_pitch = glm::degrees(asin(std::clamp(dir.y, -1.0f, 1.0f)));
    m_yaw = glm::degrees(atan2(dir.z, dir.x));

    update_orientation_from_angles();

    // handle_input() accumulates from m_yaw_old/m_pitch_old, so leaving them stale
    // here would make the first right-drag snap back to the previous orientation.
    m_yaw_old = m_yaw;
    m_pitch_old = m_pitch;
}

void Camera::update(float aspect_ratio) {
    if (m_dirty) {
        recalculate_view();
    }
    recalculate_projection(aspect_ratio);
}

void Camera::adjust_speed(float scroll_delta) {
    // Multiply speed by 1.1 for scroll up, divide by 1.1 for scroll down
    const float scroll_factor = 1.15f;
    if (scroll_delta > 0) {
        m_speed_multiplier *= scroll_factor;
    } else if (scroll_delta < 0) {
        m_speed_multiplier /= scroll_factor;
    }
    // Clamp to reasonable range (0.1x to 100x base speed)
    m_speed_multiplier = std::clamp(m_speed_multiplier, 0.1f, 100.0f);
}

void Camera::handle_input(float dt) {
    // Only move if right mouse button is held (standard editor cam)
    auto mouse_state = SDL_GetMouseState(nullptr, nullptr);
    bool is_rotating = mouse_state & SDL_BUTTON_RMASK;

    if (!is_rotating) {
        m_was_rotating = false;
        return;
    }

    // First frame of right-click: flush relative mouse state to avoid jump
    if (!m_was_rotating) {
        SDL_GetRelativeMouseState(nullptr, nullptr);
        m_was_rotating = true;
    }

    const bool* state = SDL_GetKeyboardState(nullptr);
    float speed = m_base_speed * m_speed_multiplier * dt;
    
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

        // update current yaw/pitch using difference from last frame and current movement
        m_yaw = m_yaw_old + xrel * m_sensitivity;
        m_pitch = m_pitch_old - yrel * m_sensitivity;

        update_orientation_from_angles();

        // Store the clamped pitch, so dragging past the pole does not build up
        // an invisible offset that has to be dragged back off.
        m_yaw_old = m_yaw;
        m_pitch_old = m_pitch;
    }
}

void Camera::recalculate_view() {
    m_view = glm::lookAt(m_position, m_position + m_forward, m_up);
    m_dirty = false;
    m_view_projection = m_projection * m_view;
}

void Camera::recalculate_projection(float aspect_ratio) {
    // Reverse-Z: near and far are passed swapped, so the near plane maps to depth
    // 1.0 and the far plane to 0.0. Paired with the D32_FLOAT depth target this
    // spends float mantissa where it is needed instead of collapsing everything
    // beyond a few hundred metres into a handful of representable values.
    //
    // With the conventional mapping, depth at 8km resolved to about 3 metres, so
    // the 1-3cm separations MeshBuilder gives roads, landuse and building
    // footprints could not be represented and they z-fought into speckle.
    //
    // Requires GLM_FORCE_DEPTH_ZERO_TO_ONE (set in CMakeLists), depth clears to
    // 0.0, and a GREATER depth compare on every pipeline.
    m_projection = glm::perspective(glm::radians(m_fov), aspect_ratio, m_far, m_near);
    m_view_projection = m_projection * m_view;
}

} // namespace stratum
