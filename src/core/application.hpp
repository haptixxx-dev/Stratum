#pragma once

#include <SDL3/SDL.h>

namespace stratum {

class Application {
public:
    Application() = default;
    ~Application() = default;
    
    bool init();
    void run();
    void shutdown();
    
private:
    SDL_Window* m_window = nullptr;
    SDL_GPUDevice* m_gpu_device = nullptr;
    bool m_running = false;
};

} // namespace stratum
