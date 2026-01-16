#include "editor/im3d_impl.hpp"
#include <im3d.h>
#include <vector>
#include <cmath>

namespace stratum {

void Im3D_Init() {
    // Nothing special to init for CPU-based backend
}

void Im3D_Shutdown() {
    // Nothing to cleanup
}

void Im3D_ProcessEvent(const SDL_Event* event) {
    Im3d::AppData& ad = Im3d::GetAppData();
    
    // Process input if needed, though usually we poll in NewFrame
    // SDL events could be mapped here if we used event-based input
}

void Im3D_NewFrame(float dt, const Camera& cam, float window_width, float window_height, bool has_focus) {
    Im3d::AppData& ad = Im3d::GetAppData();

    ad.m_deltaTime = dt;
    ad.m_viewportSize = Im3d::Vec2(window_width, window_height);
    ad.m_viewOrigin = Im3d::Vec3(cam.get_position().x, cam.get_position().y, cam.get_position().z);
    ad.m_viewDirection = Im3d::Vec3(cam.get_forward().x, cam.get_forward().y, cam.get_forward().z);
    ad.m_worldUp = Im3d::Vec3(cam.get_up().x, cam.get_up().y, cam.get_up().z);
    ad.m_projOrtho = false; 
    
    // FOV is in radians for Im3D
    ad.m_projScaleY = 1.0f / tanf(glm::radians(cam.m_fov) * 0.5f);

    // Input handling
    // We only capture input if the viewport has focus to avoid stealing from ImGui
    if (has_focus) {
        auto mouse_state = SDL_GetMouseState(nullptr, nullptr);
        
        ad.m_keyDown[Im3d::Mouse_Left] = (mouse_state & SDL_BUTTON_LMASK) != 0;
        
        // Map other keys if gizmos are needed (Ctrl, Shift, etc)
        const bool* keys = SDL_GetKeyboardState(NULL);
        ad.m_keyDown[Im3d::Key_L] = keys[SDL_SCANCODE_L]; 
        ad.m_keyDown[Im3d::Key_T] = keys[SDL_SCANCODE_T];
        ad.m_keyDown[Im3d::Key_R] = keys[SDL_SCANCODE_R];
        ad.m_keyDown[Im3d::Key_S] = keys[SDL_SCANCODE_S];
    } else {
        ad.m_keyDown[Im3d::Mouse_Left] = false;
    }

    // Ray picking setup would go here (Screen point -> Ray)
    
    Im3d::NewFrame();
}

void Im3D_Render(SDL_Renderer* renderer, const Camera& cam, float viewport_x, float viewport_y, float viewport_w, float viewport_h) {
    Im3d::EndFrame();
    
    // Get draw lists
    const glm::mat4& view_proj = cam.get_view_projection();
    
    using namespace Im3d;
    const U32 draw_list_count = GetDrawListCount();
    
    for (U32 i = 0; i < draw_list_count; ++i) {
        const DrawList& draw_list = GetDrawLists()[i];
        
        // Helper lambda to project a vertex
        // Returns true if vertex is in front of camera (w > epsilon)
        struct ProjectedVert {
            SDL_Vertex v;
            bool valid;
        };

        auto project_vert = [&](const VertexData& vd) -> ProjectedVert {
            // 1. World Space -> Clip Space
            glm::vec4 world_pos(vd.m_positionSize.x, vd.m_positionSize.y, vd.m_positionSize.z, 1.0f);
            glm::vec4 clip_pos = view_proj * world_pos;
            
            // Near plane culling (epsilon)
            if (clip_pos.w < 0.1f) {
                return { {}, false };
            }
            
            // 2. Clip Space -> NDC
            glm::vec3 ndc = glm::vec3(clip_pos) / clip_pos.w;
            
            // 3. NDC -> Screen Space
            float screen_x = viewport_x + (ndc.x + 1.0f) * 0.5f * viewport_w;
            float screen_y = viewport_y + (1.0f - ndc.y) * 0.5f * viewport_h;
            
            // Color conversion
            U32 color = vd.m_color;
            SDL_FColor sdl_col;
            sdl_col.r = ((color >> 0) & 0xFF) / 255.0f;
            sdl_col.g = ((color >> 8) & 0xFF) / 255.0f;
            sdl_col.b = ((color >> 16) & 0xFF) / 255.0f;
            sdl_col.a = ((color >> 24) & 0xFF) / 255.0f;
            
            return { {{screen_x, screen_y}, sdl_col, {0, 0}}, true };
        };

        if (draw_list.m_primType == DrawPrimitive_Triangles) {
            std::vector<SDL_Vertex> sdl_vertices;
            sdl_vertices.reserve(draw_list.m_vertexCount);

            // Process triangles (batches of 3)
            for (U32 v = 0; v < draw_list.m_vertexCount; v += 3) {
                if (v + 2 >= draw_list.m_vertexCount) break;

                auto p0 = project_vert(draw_list.m_vertexData[v]);
                auto p1 = project_vert(draw_list.m_vertexData[v+1]);
                auto p2 = project_vert(draw_list.m_vertexData[v+2]);

                // Conservative culling: if ANY vertex is behind camera, discard triangle
                // (Proper solution requires clipping causing new triangles)
                if (p0.valid && p1.valid && p2.valid) {
                    sdl_vertices.push_back(p0.v);
                    sdl_vertices.push_back(p1.v);
                    sdl_vertices.push_back(p2.v);
                }
            }

            if (!sdl_vertices.empty()) {
                 SDL_RenderGeometry(
                    renderer,
                    nullptr,
                    sdl_vertices.data(),
                    static_cast<int>(sdl_vertices.size()),
                    nullptr,
                    0
                );
            }
        } 
        else if (draw_list.m_primType == DrawPrimitive_Lines) {
             for (U32 v = 0; v < draw_list.m_vertexCount; v += 2) {
                if (v + 1 >= draw_list.m_vertexCount) break;

                auto p0 = project_vert(draw_list.m_vertexData[v]);
                auto p1 = project_vert(draw_list.m_vertexData[v+1]);

                if (p0.valid && p1.valid) {
                     SDL_SetRenderDrawColor(renderer, 
                        (Uint8)(p0.v.color.r * 255), 
                        (Uint8)(p0.v.color.g * 255), 
                        (Uint8)(p0.v.color.b * 255), 
                        (Uint8)(p0.v.color.a * 255));
                     SDL_RenderLine(renderer, p0.v.position.x, p0.v.position.y, p1.v.position.x, p1.v.position.y);
                }
             }
        }
        else if (draw_list.m_primType == DrawPrimitive_Points) {
            for (U32 v = 0; v < draw_list.m_vertexCount; ++v) {
                auto p0 = project_vert(draw_list.m_vertexData[v]);
                if (p0.valid) {
                    SDL_SetRenderDrawColor(renderer, 
                        (Uint8)(p0.v.color.r * 255), 
                        (Uint8)(p0.v.color.g * 255), 
                        (Uint8)(p0.v.color.b * 255), 
                        (Uint8)(p0.v.color.a * 255));
                    SDL_RenderPoint(renderer, p0.v.position.x, p0.v.position.y);
                }
            }
        }
    }
}

} // namespace stratum
