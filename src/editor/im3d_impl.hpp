#pragma once

#include "editor/camera.hpp"
#include <SDL3/SDL.h>

namespace stratum {

void Im3D_Init();
void Im3D_Shutdown();
void Im3D_NewFrame(float dt, const Camera& cam, float window_width, float window_height, bool has_focus);
void Im3D_Render(SDL_Renderer* renderer, const Camera& cam, float viewport_x, float viewport_y, float viewport_w, float viewport_h);
void Im3D_ProcessEvent(const SDL_Event* event);

} // namespace stratum
