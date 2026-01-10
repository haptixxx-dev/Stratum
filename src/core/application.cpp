#include "core/application.hpp"
#include <spdlog/spdlog.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

namespace stratum {

bool Application::init() {
    // Initialize SDL
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        spdlog::error("Failed to initialize SDL: {}", SDL_GetError());
        return false;
    }
    spdlog::info("SDL initialized");

    // Create borderless window with custom controls
    WindowConfig config;
    config.title = "Stratum";
    config.width = 1400;
    config.height = 900;
    config.borderless = true;
    config.title_bar_height = 50;  // Height of menu bar for drag area
    config.resize_border = 10;     // Resize border width
    config.vsync = true;

    if (!m_window.init(config)) {
        spdlog::error("Failed to create window");
        SDL_Quit();
        return false;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Simple font config - 16px is readable on most displays
    ImFontConfig font_config;
    font_config.SizePixels = 16.0f;
    font_config.OversampleH = 2;
    font_config.OversampleV = 2;
    io.Fonts->AddFontDefault(&font_config);

    // Style tweaks
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 2.0f;
    style.ScrollbarRounding = 4.0f;

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForSDLRenderer(m_window.get_handle(), m_window.get_renderer());
    ImGui_ImplSDLRenderer3_Init(m_window.get_renderer());

    // Initialize the editor
    m_editor.init();
    m_editor.set_quit_callback([this]() { request_quit(); });
    m_editor.set_window_handle(m_window.get_handle());

    spdlog::info("ImGui initialized with docking");
    m_running = true;
    return true;
}

void Application::run() {
    while (m_running) {
        process_events();

        // Skip rendering if minimized
        if (m_window.is_minimized()) {
            SDL_Delay(100);
            continue;
        }

        update();
        render();
    }
}

void Application::process_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL3_ProcessEvent(&event);

        if (event.type == SDL_EVENT_QUIT) {
            m_running = false;
        }
        if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED &&
            event.window.windowID == SDL_GetWindowID(m_window.get_handle())) {
            m_running = false;
        }
    }
}

void Application::update() {
    m_window.begin_frame();

    // Start the Dear ImGui frame
    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
}

void Application::render() {
    // Render the editor UI
    m_editor.render();

    // Finalize ImGui frame
    ImGui::Render();

    // Clear and render
    SDL_Renderer* renderer = m_window.get_renderer();
    SDL_SetRenderDrawColor(renderer, 25, 25, 28, 255);
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);

    m_window.end_frame();
}

void Application::shutdown() {
    // Cleanup editor
    m_editor.shutdown();

    // Cleanup ImGui
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    spdlog::info("ImGui shutdown");

    // Cleanup window
    m_window.shutdown();

    // Cleanup SDL
    SDL_Quit();
    spdlog::info("SDL shutdown");
}

} // namespace stratum
