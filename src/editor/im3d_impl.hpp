#pragma once

#include "editor/camera.hpp"
#include <SDL3/SDL.h>
#include <glm/glm.hpp>

namespace stratum {

class GPURenderer;

void Im3D_Init();
void Im3D_Shutdown();
void Im3D_NewFrame(float dt, const Camera& cam, float window_width, float window_height, bool has_focus);
void Im3D_ProcessEvent(const SDL_Event* event);

/**
 * @brief Create the SDL_GPU resources (shaders + pipelines) for Im3d rendering
 * @note Must be called AFTER the GPURenderer is initialized. Degrades gracefully:
 *       on failure Im3d simply renders nothing and the app keeps running.
 */
bool Im3D_InitGPU(GPURenderer& renderer);

/**
 * @brief Release the SDL_GPU resources created by Im3D_InitGPU()
 */
void Im3D_ShutdownGPU();

/**
 * @brief End the Im3d frame and upload its draw lists to the GPU
 * @note Must be called on the frame's command buffer with NO render pass active -
 *       it opens a copy pass. Safe no-op if Im3D_NewFrame() was not called.
 */
void Im3D_EndFrameAndUpload(GPURenderer& renderer);

/**
 * @brief Draw the geometry uploaded by Im3D_EndFrameAndUpload()
 * @param view_proj Combined view-projection matrix
 * @param viewport_w Viewport width in pixels (for pixel-size expansion)
 * @param viewport_h Viewport height in pixels
 * @note Must be called INSIDE the depth-enabled 3D render pass, after the
 *       viewport and scissor have been set.
 */
void Im3D_Render(GPURenderer& renderer, const glm::mat4& view_proj,
                 float viewport_w, float viewport_h);

} // namespace stratum
