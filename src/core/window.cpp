#include "core/window.hpp"
#include <spdlog/spdlog.h>

namespace stratum {

bool Window::init(const WindowConfig& config) {
    // Build window flags - use HIGH_PIXEL_DENSITY for crisp Retina rendering
    SDL_WindowFlags window_flags = SDL_WINDOW_HIGH_PIXEL_DENSITY;

    if (config.borderless) {
        window_flags |= SDL_WINDOW_BORDERLESS;
        window_flags |= SDL_WINDOW_RESIZABLE;
    } else if (config.resizable) {
        window_flags |= SDL_WINDOW_RESIZABLE;
    }
    if (config.fullscreen) {
        window_flags |= SDL_WINDOW_FULLSCREEN;
    }
    if (config.maximized) {
        window_flags |= SDL_WINDOW_MAXIMIZED;
    }

    // Set window size (logical)
    m_width = config.width;
    m_height = config.height;

    // Create SDL window
    m_window = SDL_CreateWindow(
        config.title.c_str(),
        m_width,
        m_height,
        window_flags
    );

    if (!m_window) {
        spdlog::error("Failed to create SDL window: {}", SDL_GetError());
        return false;
    }

    // Get pixel density (2.0 on Retina)
    m_scale = SDL_GetWindowPixelDensity(m_window);
    if (m_scale <= 0.0f) {
        m_scale = 1.0f;
    }

    // Center the window
    SDL_SetWindowPosition(m_window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);

    spdlog::info("Window created: {}x{} (scale: {:.1f}, borderless: {})",
                 m_width, m_height, m_scale, config.borderless);
    return true;
}

void Window::shutdown() {
    if (m_window) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }
    spdlog::info("Window destroyed");
}

bool Window::is_minimized() const {
    if (!m_window) return false;
    return (SDL_GetWindowFlags(m_window) & SDL_WINDOW_MINIMIZED) != 0;
}

void Window::update_size() {
    if (m_window) {
        SDL_GetWindowSize(m_window, &m_width, &m_height);
    }
}

} // namespace stratum
