#include "core/window.hpp"
#include <spdlog/spdlog.h>

namespace stratum {

bool Window::init(const WindowConfig& config) {
    // Get display scale for HiDPI support
    m_scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
    if (m_scale <= 0.0f) {
        m_scale = 1.0f;
    }

    // Calculate scaled dimensions
    m_width = static_cast<int>(config.width * m_scale);
    m_height = static_cast<int>(config.height * m_scale);

    // Build window flags
    SDL_WindowFlags window_flags = SDL_WINDOW_HIGH_PIXEL_DENSITY;
    if (config.resizable) {
        window_flags |= SDL_WINDOW_RESIZABLE;
    }
    if (config.maximized) {
        window_flags |= SDL_WINDOW_MAXIMIZED;
    }

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

    // Create SDL renderer
    m_renderer = SDL_CreateRenderer(m_window, nullptr);
    if (!m_renderer) {
        spdlog::error("Failed to create SDL renderer: {}", SDL_GetError());
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
        return false;
    }

    // Enable VSync if requested
    if (config.vsync) {
        SDL_SetRenderVSync(m_renderer, 1);
    }

    // Center the window
    SDL_SetWindowPosition(m_window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);

    spdlog::info("Window created: {}x{} (scale: {:.2f})", m_width, m_height, m_scale);
    return true;
}

void Window::shutdown() {
    if (m_renderer) {
        SDL_DestroyRenderer(m_renderer);
        m_renderer = nullptr;
    }
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

void Window::begin_frame() {
    // Update window dimensions in case of resize
    SDL_GetWindowSize(m_window, &m_width, &m_height);
}

void Window::end_frame() {
    SDL_RenderPresent(m_renderer);
}

} // namespace stratum
