#include "core/window.hpp"
#include <spdlog/spdlog.h>

namespace stratum {

// Hit test data for borderless window
struct HitTestData {
    int title_bar_height;
    int resize_border;
};

// Hit test callback for borderless window - enables drag and resize
static SDL_HitTestResult hit_test_callback(SDL_Window* window, const SDL_Point* pt, void* data) {
    auto* htd = static_cast<HitTestData*>(data);
    int w, h;
    SDL_GetWindowSize(window, &w, &h);

    const int border = htd->resize_border;
    const int title_height = htd->title_bar_height;

    // Check corners first (for diagonal resize)
    if (pt->y < border) {
        if (pt->x < border) return SDL_HITTEST_RESIZE_TOPLEFT;
        if (pt->x >= w - border) return SDL_HITTEST_RESIZE_TOPRIGHT;
        return SDL_HITTEST_RESIZE_TOP;
    }
    if (pt->y >= h - border) {
        if (pt->x < border) return SDL_HITTEST_RESIZE_BOTTOMLEFT;
        if (pt->x >= w - border) return SDL_HITTEST_RESIZE_BOTTOMRIGHT;
        return SDL_HITTEST_RESIZE_BOTTOM;
    }
    if (pt->x < border) return SDL_HITTEST_RESIZE_LEFT;
    if (pt->x >= w - border) return SDL_HITTEST_RESIZE_RIGHT;

    // Title bar area for dragging
    if (pt->y < title_height) {
        return SDL_HITTEST_DRAGGABLE;
    }

    return SDL_HITTEST_NORMAL;
}

// Static hit test data (needs to persist)
static HitTestData s_hit_test_data;

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

    // Note: SDL_SetWindowHitTest causes freezing on macOS, so we implement
    // window dragging manually in the editor via ImGui instead

    // Create SDL renderer
    m_renderer = SDL_CreateRenderer(m_window, nullptr);
    if (!m_renderer) {
        spdlog::error("Failed to create SDL renderer: {}", SDL_GetError());
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
        return false;
    }

    // Scale renderer to match pixel density - this fixes ImGui coordinate mismatch
    SDL_SetRenderScale(m_renderer, m_scale, m_scale);

    // Enable VSync if requested
    if (config.vsync) {
        SDL_SetRenderVSync(m_renderer, 1);
    }

    // Center the window
    SDL_SetWindowPosition(m_window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);

    spdlog::info("Window created: {}x{} (scale: {:.1f}, borderless: {})",
                 m_width, m_height, m_scale, config.borderless);
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
