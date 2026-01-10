/**
 * @file window.hpp
 * @brief SDL3 window management wrapper
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 * 
 * This file contains the Window class which provides a clean abstraction
 * over SDL3 window and renderer functionality with HiDPI support.
 */

#pragma once

#include <SDL3/SDL.h>
#include <string>

namespace stratum {

/**
 * @brief Configuration structure for Window initialization
 * 
 * Contains all parameters needed to create and configure a window.
 * Default values provide a reasonable 1280x800 resizable window.
 */
struct WindowConfig {
    std::string title = "Stratum";  ///< Window title displayed in title bar
    int width = 1280;                ///< Initial window width in logical pixels
    int height = 800;                ///< Initial window height in logical pixels
    bool resizable = true;           ///< Whether the window can be resized by the user
    bool maximized = false;          ///< Whether to start maximized
    bool borderless = false;         ///< Remove window decorations (title bar, etc.)
    bool fullscreen = false;         ///< Start in fullscreen desktop mode
    bool vsync = true;               ///< Enable vertical synchronization
    int title_bar_height = 32;       ///< Height of custom title bar for borderless windows
    int resize_border = 6;           ///< Width of resize border for borderless windows
};

/**
 * @brief SDL3 Window wrapper with renderer management
 * 
 * The Window class encapsulates SDL3 window creation, renderer setup,
 * and frame management. It provides:
 * - HiDPI/Retina display support with automatic scaling
 * - SDL_Renderer for 2D/ImGui rendering
 * - Frame lifecycle management (begin_frame/end_frame)
 * 
 * @note This class is non-copyable to prevent accidental resource duplication.
 * 
 * Example usage:
 * @code
 * stratum::Window window;
 * stratum::WindowConfig config;
 * config.title = "My App";
 * config.width = 1920;
 * config.height = 1080;
 * 
 * if (window.init(config)) {
 *     while (running) {
 *         window.begin_frame();
 *         // ... render ...
 *         window.end_frame();
 *     }
 *     window.shutdown();
 * }
 * @endcode
 */
class Window {
public:
    /**
     * @brief Default constructor
     * 
     * Creates an uninitialized Window object. Call init() before use.
     */
    Window() = default;
    
    /**
     * @brief Destructor
     * 
     * @warning Does not automatically call shutdown(). Ensure shutdown()
     * is called before destruction to properly release SDL resources.
     */
    ~Window() = default;

    /// @name Deleted Copy Operations
    /// @{
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    /// @}

    /**
     * @brief Initialize the window with the given configuration
     * 
     * Creates an SDL window and renderer with the specified settings.
     * Automatically handles HiDPI scaling by querying display scale factor.
     * 
     * @param config Window configuration parameters
     * @return true if initialization succeeded, false on error
     * 
     * @note SDL must be initialized before calling this function.
     * @see WindowConfig
     */
    bool init(const WindowConfig& config = {});
    
    /**
     * @brief Shutdown and release all window resources
     * 
     * Destroys the SDL renderer and window. Safe to call multiple times.
     * After calling shutdown(), the window can be re-initialized with init().
     */
    void shutdown();

    /// @name Accessors
    /// @{
    
    /**
     * @brief Get the underlying SDL window handle
     * @return Pointer to SDL_Window, or nullptr if not initialized
     */
    [[nodiscard]] SDL_Window* get_handle() const { return m_window; }
    
    /**
     * @brief Get the SDL renderer
     * @return Pointer to SDL_Renderer, or nullptr if not initialized
     */
    [[nodiscard]] SDL_Renderer* get_renderer() const { return m_renderer; }
    
    /**
     * @brief Get current window width
     * @return Width in physical pixels (scaled for HiDPI)
     */
    [[nodiscard]] int get_width() const { return m_width; }
    
    /**
     * @brief Get current window height
     * @return Height in physical pixels (scaled for HiDPI)
     */
    [[nodiscard]] int get_height() const { return m_height; }
    
    /**
     * @brief Get the display scale factor
     * @return Scale factor (1.0 for standard displays, 2.0 for Retina, etc.)
     */
    [[nodiscard]] float get_scale() const { return m_scale; }
    
    /**
     * @brief Check if window is minimized
     * @return true if window is currently minimized
     */
    [[nodiscard]] bool is_minimized() const;
    
    /// @}

    /// @name Frame Management
    /// @{
    
    /**
     * @brief Begin a new frame
     * 
     * Call at the start of each frame before rendering.
     * Updates internal window dimensions in case of resize.
     */
    void begin_frame();
    
    /**
     * @brief End the current frame
     * 
     * Call after all rendering is complete. Presents the rendered
     * content to the screen via SDL_RenderPresent.
     */
    void end_frame();
    
    /// @}

private:
    SDL_Window* m_window = nullptr;      ///< SDL window handle
    SDL_Renderer* m_renderer = nullptr;  ///< SDL renderer handle
    int m_width = 0;                     ///< Current window width in pixels
    int m_height = 0;                    ///< Current window height in pixels
    float m_scale = 1.0f;                ///< Display scale factor for HiDPI
};

} // namespace stratum
