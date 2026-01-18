#include "core/application.hpp"
#include <spdlog/spdlog.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlgpu3.h>

namespace stratum {

bool Application::init() {
    // Initialize SDL
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        spdlog::error("Failed to initialize SDL: {}", SDL_GetError());
        return false;
    }
    spdlog::info("SDL initialized");

    // Create window (no renderer - GPU handles that)
    WindowConfig config;
    config.title = "Stratum";
    config.width = 1400;
    config.height = 900;
    config.borderless = true;
    config.title_bar_height = 50;
    config.resize_border = 10;

    if (!m_window.init(config)) {
        spdlog::error("Failed to create window");
        SDL_Quit();
        return false;
    }

    // Initialize GPU renderer
    if (!m_gpu_renderer.init(m_window.get_handle())) {
        spdlog::error("Failed to initialize GPU renderer");
        m_window.shutdown();
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

    // Simple font config
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
    ImGui_ImplSDL3_InitForSDLGPU(m_window.get_handle());

    // Initialize SDL_GPU backend for ImGui
    ImGui_ImplSDLGPU3_InitInfo gpu_init_info{};
    gpu_init_info.Device = m_gpu_renderer.get_device();
    gpu_init_info.ColorTargetFormat = m_gpu_renderer.get_swapchain_format();
    gpu_init_info.MSAASamples = SDL_GPU_SAMPLECOUNT_1;
    if (!ImGui_ImplSDLGPU3_Init(&gpu_init_info)) {
        spdlog::error("Failed to initialize ImGui SDL_GPU backend");
        m_gpu_renderer.shutdown();
        m_window.shutdown();
        SDL_Quit();
        return false;
    }

    // Initialize the editor
    m_editor.init();
    m_editor.set_quit_callback([this]() { request_quit(); });
    m_editor.set_window_handle(m_window.get_handle());
    m_editor.set_renderer(&m_gpu_renderer);

    // MSAA change callback disabled - runtime MSAA changes not supported
    // TODO: Implement offscreen MSAA rendering for proper runtime support

    spdlog::info("ImGui initialized with SDL_GPU backend");
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
    m_window.update_size();

    // Start the Dear ImGui frame
    ImGui_ImplSDLGPU3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
}

void Application::render() {
    // Update editor state (tile culling, etc.)
    m_editor.update();

    // Render the editor UI
    m_editor.render();

    // Finalize ImGui frame
    ImGui::Render();

    ImDrawData* draw_data = ImGui::GetDrawData();
    if (!draw_data) {
        return;
    }

    // Begin GPU frame - acquire command buffer and swapchain
    if (!m_gpu_renderer.begin_frame()) {
        return;  // Window minimized or error
    }

    // Prepare ImGui draw data BEFORE render pass
    ImGui_ImplSDLGPU3_PrepareDrawData(draw_data, m_gpu_renderer.get_command_buffer());

    // Begin render pass
    m_gpu_renderer.begin_render_pass();

    // Render 3D content - NOW handled via ImGui callback in viewport
    // m_editor.render_3d(m_gpu_renderer);

    // Render ImGui on top

    ImGui_ImplSDLGPU3_RenderDrawData(draw_data,
                                      m_gpu_renderer.get_command_buffer(),
                                      m_gpu_renderer.get_render_pass());

    // End frame and present
    m_gpu_renderer.end_frame();
}

void Application::shutdown() {
    // Wait for GPU to finish before cleanup
    if (m_gpu_renderer.get_device()) {
        SDL_WaitForGPUIdle(m_gpu_renderer.get_device());
    }

    // Cleanup editor
    m_editor.shutdown();

    // Cleanup ImGui
    ImGui_ImplSDLGPU3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    spdlog::info("ImGui shutdown");

    // Cleanup GPU renderer
    m_gpu_renderer.shutdown();

    // Cleanup window
    m_window.shutdown();

    // Cleanup SDL
    SDL_Quit();
    spdlog::info("SDL shutdown");
}

} // namespace stratum
