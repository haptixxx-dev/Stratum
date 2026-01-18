/**
 * @file application.hpp
 * @brief Main application class for Stratum
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 * 
 * This file contains the Application class which serves as the main
 * entry point and coordinator for the Stratum application.
 */

#pragma once

#include "core/window.hpp"
#include "editor/editor.hpp"
#include "renderer/gpu_renderer.hpp"
#include <SDL3/SDL.h>

/**
 * @namespace stratum
 * @brief Root namespace for all Stratum engine code
 * 
 * The stratum namespace contains all classes, functions, and types
 * that make up the Stratum 3D map generation application.
 */
namespace stratum {

/**
 * @brief Main application class coordinating all subsystems
 * 
 * The Application class is the central coordinator for Stratum.
 * It manages:
 * - SDL initialization and shutdown
 * - Window creation and management
 * - ImGui setup and rendering
 * - Main application loop
 * - Event processing
 * 
 * @note This class is non-copyable as it manages unique system resources.
 * 
 * Typical usage:
 * @code
 * int main() {
 *     stratum::Application app;
 *     
 *     if (!app.init()) {
 *         return 1;
 *     }
 *     
 *     app.run();  // Blocks until application closes
 *     app.shutdown();
 *     
 *     return 0;
 * }
 * @endcode
 * 
 * @see Window
 */
class Application {
public:
    /**
     * @brief Default constructor
     * 
     * Creates an uninitialized Application. Call init() before run().
     */
    Application() = default;
    
    /**
     * @brief Destructor
     * 
     * @warning Does not automatically call shutdown(). Always call
     * shutdown() explicitly before the Application is destroyed.
     */
    ~Application() = default;

    /// @name Deleted Copy Operations
    /// @{
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;
    /// @}

    /**
     * @brief Initialize all application subsystems
     * 
     * Performs the following initialization sequence:
     * 1. Initialize SDL (video and gamepad subsystems)
     * 2. Create the main window
     * 3. Initialize Dear ImGui with SDL3 backend
     * 4. Configure ImGui styling and HiDPI scaling
     * 
     * @return true if all subsystems initialized successfully
     * @return false if any initialization step failed
     * 
     * @note Errors are logged via spdlog.
     */
    bool init();
    
    /**
     * @brief Run the main application loop
     * 
     * Enters the main loop which:
     * 1. Processes SDL events
     * 2. Updates application state
     * 3. Renders the frame
     * 
     * This function blocks until the application receives a quit signal
     * (window close, SDL_QUIT event, etc.)
     * 
     * @pre init() must have been called and returned true
     */
    void run();
    
    /**
     * @brief Shutdown all application subsystems
     * 
     * Performs cleanup in reverse order of initialization:
     * 1. Shutdown ImGui
     * 2. Destroy window
     * 3. Quit SDL
     * 
     * Safe to call multiple times. After shutdown(), the application
     * can be re-initialized with init().
     */
    void shutdown();

    /// @name Accessors
    /// @{
    
    /**
     * @brief Get the main application window
     * @return Reference to the Window object
     */
    [[nodiscard]] Window& get_window() { return m_window; }
    
    /**
     * @brief Get the main application window (const version)
     * @return Const reference to the Window object
     */
    [[nodiscard]] const Window& get_window() const { return m_window; }
    
    /**
     * @brief Check if the application is currently running
     * @return true if the main loop is active
     */
    [[nodiscard]] bool is_running() const { return m_running; }

    /**
     * @brief Request the application to quit
     */
    void request_quit() { m_running = false; }

    /// @}

private:
    /**
     * @brief Process all pending SDL events
     * 
     * Polls SDL event queue and:
     * - Forwards events to ImGui
     * - Handles quit events
     * - Handles window close events
     */
    void process_events();
    
    /**
     * @brief Update application state for current frame
     * 
     * Called once per frame before rendering.
     * Prepares ImGui for new frame.
     */
    void update();
    
    /**
     * @brief Render the current frame
     *
     * Renders all UI elements and presents to screen.
     */
    void render();

    Window m_window;            ///< Main application window
    GPURenderer m_gpu_renderer; ///< GPU renderer for 3D graphics
    Editor m_editor;            ///< Main editor interface
    bool m_running = false;     ///< Flag indicating if main loop is active
};

} // namespace stratum
