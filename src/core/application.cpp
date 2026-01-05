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

    // Create window
    WindowConfig config;
    config.title = "Stratum";
    config.width = 1280;
    config.height = 800;
    config.resizable = true;
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

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Scale ImGui for HiDPI
    ImGuiStyle& style = ImGui::GetStyle();
    float scale = m_window.get_scale();
    style.ScaleAllSizes(scale);

    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForSDLRenderer(m_window.get_handle(), m_window.get_renderer());
    ImGui_ImplSDLRenderer3_Init(m_window.get_renderer());

    spdlog::info("ImGui initialized");
    m_running = true;
    return true;
}

void Application::run() {
    while (m_running) {
        process_events();

        // Skip rendering if minimized
        if (m_window.is_minimized()) {
            SDL_Delay(10);
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
    ImGuiIO& io = ImGui::GetIO();

    // Demo window for testing
    static bool show_demo = true;
    if (show_demo) {
        ImGui::ShowDemoWindow(&show_demo);
    }

    // Main window
    ImGui::Begin("Stratum");
    ImGui::Text("Welcome to Stratum!");
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 
                1000.0f / io.Framerate, io.Framerate);
    ImGui::Checkbox("Show Demo Window", &show_demo);
    ImGui::End();

    // Render ImGui
    ImGui::Render();

    // Clear and render
    SDL_Renderer* renderer = m_window.get_renderer();
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);

    m_window.end_frame();
}

void Application::shutdown() {
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
