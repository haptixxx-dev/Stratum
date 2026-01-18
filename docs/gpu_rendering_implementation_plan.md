# SDL_GPU Rendering Implementation Plan

## Overview

This document outlines the migration from SDL_Renderer (CPU-based 2D rendering) to SDL_GPU (hardware-accelerated 3D rendering with Vulkan backend). This change is necessary to handle large OSM datasets (entire cities) with acceptable performance.

### Current State

- **Rendering**: Im3d software rasterizer via SDL_Renderer
- **Problem**: All triangle drawing happens on CPU, causing high CPU usage
- **Bottleneck**: Large meshes (buildings, roads, areas) rendered per-frame on CPU

### Target State

- **Rendering**: SDL_GPU with Vulkan backend
- **Benefit**: GPU-accelerated rendering, massive performance improvement
- **Architecture**: GPURenderer class manages all GPU resources and draw calls

---

## Architecture Overview

```text
┌─────────────────────────────────────────────────────────────────┐
│                         Application                              │
├─────────────────────────────────────────────────────────────────┤
│  Window                                                          │
│  ├── SDL_Window                                                  │
│  └── SDL_GPUDevice (replaces SDL_Renderer)                      │
├─────────────────────────────────────────────────────────────────┤
│  GPURenderer                                                     │
│  ├── Graphics Pipeline (shaders, vertex format, states)         │
│  ├── Mesh Storage (GPU buffers for uploaded meshes)             │
│  ├── Frame Management (command buffers, render passes)          │
│  └── Uniform Management (MVP matrices, lighting)                │
├─────────────────────────────────────────────────────────────────┤
│  Editor                                                          │
│  ├── Camera (view/projection matrices)                          │
│  ├── TileManager (OSM data organized in tiles)                  │
│  └── ImGui (UI panels, uses imgui_impl_sdlgpu3)                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Files Created

The following files have already been created as part of this implementation:

### 1. GPURenderer Header (`src/renderer/gpu_renderer.hpp`)

- `GPUMesh` struct: Holds GPU buffer handles for uploaded meshes
- `MeshUniforms` struct: Uniform data pushed to vertex shader (MVP, model, tint)
- `GPURenderer` class: Main renderer with init/shutdown, mesh upload, draw calls

### 2. GPURenderer Implementation (`src/renderer/gpu_renderer.cpp`)

- Device creation with Vulkan preference
- Depth texture creation for 3D rendering
- Shader loading from SPIR-V files
- Graphics pipeline creation with vertex format matching `Vertex` struct
- Mesh upload via transfer buffers
- Frame rendering with command buffers and render passes

### 3. Shaders (`assets/shaders/`)

- `mesh.vert` / `mesh.vert.spv`: Vertex shader (transforms, passes attributes)
- `mesh.frag` / `mesh.frag.spv`: Fragment shader (basic diffuse lighting)

---

## Implementation Steps

### Phase 1: Window/Application Migration

#### Step 1.1: Modify Window Class

**File**: `src/core/window.hpp`

Replace SDL_Renderer with SDL_GPUDevice:

```cpp
// Before
SDL_Renderer* m_renderer = nullptr;

// After
SDL_GPUDevice* m_gpu_device = nullptr;
```

Add new methods:

```cpp
[[nodiscard]] SDL_GPUDevice* get_gpu_device() const { return m_gpu_device; }
```

**File**: `src/core/window.cpp`

In `Window::init()`:

```cpp
// Remove SDL_CreateRenderer call

// Add SDL_GPU device creation
m_gpu_device = SDL_CreateGPUDevice(
    SDL_GPU_SHADERFORMAT_SPIRV,
    true,  // debug mode
    nullptr
);

if (!m_gpu_device) {
    spdlog::error("Failed to create GPU device: {}", SDL_GetError());
    return false;
}

// Claim window for GPU
if (!SDL_ClaimWindowForGPUDevice(m_gpu_device, m_window)) {
    spdlog::error("Failed to claim window: {}", SDL_GetError());
    SDL_DestroyGPUDevice(m_gpu_device);
    return false;
}
```

In `Window::shutdown()`:

```cpp
// Remove SDL_DestroyRenderer

// Add GPU cleanup
if (m_gpu_device) {
    SDL_ReleaseWindowFromGPUDevice(m_gpu_device, m_window);
    SDL_DestroyGPUDevice(m_gpu_device);
    m_gpu_device = nullptr;
}
```

Remove `begin_frame()` and `end_frame()` - these will be handled by GPURenderer.

#### Step 1.2: Update Application Class

**File**: `src/core/application.hpp`

Add GPURenderer member:

```cpp
#include "renderer/gpu_renderer.hpp"

class Application {
    // ...
    GPURenderer m_gpu_renderer;
};
```

**File**: `src/core/application.cpp`

In `Application::init()`:

```cpp
// After window init
if (!m_gpu_renderer.init(m_window.get_handle())) {
    spdlog::error("Failed to initialize GPU renderer");
    return false;
}
```

In `Application::shutdown()`:

```cpp
m_gpu_renderer.shutdown();
// Then window shutdown
```

In `Application::run()` main loop:

```cpp
// Frame start
if (!m_gpu_renderer.begin_frame()) {
    continue;  // Window minimized or swapchain not ready
}

// Update and render
m_editor.update();
m_editor.render(m_gpu_renderer);  // Pass renderer to editor

// Frame end
m_gpu_renderer.end_frame();
```

---

### Phase 2: ImGui Migration

#### Step 2.1: Update CMakeLists.txt

Add the SDL_GPU ImGui backend files:

```cmake
# In the imgui section, add:
${EXTERNAL_DIR}/imgui/backends/imgui_impl_sdlgpu3.cpp
```

#### Step 2.2: Update ImGui Initialization

**File**: `src/core/application.cpp` (or wherever ImGui is initialized)

Replace:

```cpp
#include <imgui_impl_sdlrenderer3.h>
```

With:

```cpp
#include <imgui_impl_sdlgpu3.h>
```

In init:

```cpp
// Before
ImGui_ImplSDLRenderer3_Init(renderer);

// After
ImGui_ImplSDLGPU3_Init(m_gpu_renderer.get_device(), m_window.get_handle(),
                        SDL_GetGPUSwapchainTextureFormat(m_gpu_renderer.get_device(), m_window.get_handle()));
```

In shutdown:

```cpp
// Before
ImGui_ImplSDLRenderer3_Shutdown();

// After
ImGui_ImplSDLGPU3_Shutdown();
```

In frame start:

```cpp
// Before
ImGui_ImplSDLRenderer3_NewFrame();

// After
ImGui_ImplSDLGPU3_NewFrame();
```

In frame end (after ImGui::Render()):

```cpp
// Before
ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);

// After
// This is called within the render pass
ImGui_ImplSDLGPU3_RenderDrawData(ImGui::GetDrawData(), command_buffer, render_pass);
```

**Important**: ImGui rendering must happen within the same render pass as the 3D content, or in a separate render pass after. The GPURenderer needs to provide access to the command buffer and render pass, or handle ImGui rendering internally.

#### Step 2.3: Modify GPURenderer for ImGui

Add method to GPURenderer:

```cpp
void render_imgui() {
    if (m_render_pass && m_cmd_buffer) {
        ImGui_ImplSDLGPU3_RenderDrawData(ImGui::GetDrawData(), m_cmd_buffer, m_render_pass);
    }
}
```

Or expose command buffer/render pass:

```cpp
SDL_GPUCommandBuffer* get_command_buffer() const { return m_cmd_buffer; }
SDL_GPURenderPass* get_render_pass() const { return m_render_pass; }
```

---

### Phase 3: Editor Migration

#### Step 3.1: Remove Im3d Dependency

**File**: `src/editor/editor.hpp`

Remove Im3d-related includes and members:

```cpp
// Remove
#include "editor/im3d_impl.hpp"
std::vector<BatchedTriangle> m_batched_building_tris;
std::vector<BatchedTriangle> m_batched_road_tris;
std::vector<BatchedTriangle> m_batched_area_tris;
```

Add GPURenderer mesh IDs:

```cpp
// Per-tile GPU mesh IDs (or store in Tile struct)
std::unordered_map<osm::TileCoord, uint32_t> m_tile_road_mesh_ids;
std::unordered_map<osm::TileCoord, uint32_t> m_tile_building_mesh_ids;
std::unordered_map<osm::TileCoord, uint32_t> m_tile_area_mesh_ids;
```

#### Step 3.2: Modify Tile Structure

**File**: `src/osm/tile_manager.hpp`

Add GPU mesh IDs to Tile:

```cpp
struct Tile {
    // ... existing members ...

    // GPU mesh handles (0 = not uploaded)
    uint32_t gpu_road_mesh_id = 0;
    uint32_t gpu_building_mesh_id = 0;
    uint32_t gpu_area_mesh_id = 0;
    bool gpu_meshes_uploaded = false;
};
```

#### Step 3.3: Update Editor Rendering

**File**: `src/editor/editor.cpp`

Replace `rebuild_visible_batches()` with GPU mesh upload/draw:

```cpp
void Editor::render(GPURenderer& renderer) {
    // Set camera matrices
    renderer.set_view_projection(m_camera.get_view_matrix(), m_camera.get_projection_matrix());

    Frustum frustum = m_camera.get_frustum();

    for (const auto& coord : m_tile_manager.get_all_tiles()) {
        auto* tile = m_tile_manager.get_tile(coord);
        if (!tile || !tile->has_valid_bounds()) continue;

        // Frustum cull
        if (!frustum.intersects_aabb(tile->bounds_min, tile->bounds_max)) {
            continue;
        }

        // Ensure meshes are built (async)
        if (!tile->meshes_built && !tile->meshes_pending) {
            m_tile_manager.queue_tile_build_async(coord);
            continue;
        }
        if (!tile->meshes_built) continue;

        // Upload to GPU if not already
        if (!tile->gpu_meshes_uploaded) {
            upload_tile_to_gpu(tile, renderer);
        }

        // Draw
        if (m_render_areas && tile->gpu_area_mesh_id != 0) {
            renderer.draw_mesh(tile->gpu_area_mesh_id);
        }
        if (m_render_buildings && tile->gpu_building_mesh_id != 0) {
            renderer.draw_mesh(tile->gpu_building_mesh_id);
        }
        if (m_render_roads && tile->gpu_road_mesh_id != 0) {
            renderer.draw_mesh(tile->gpu_road_mesh_id);
        }
    }
}

void Editor::upload_tile_to_gpu(osm::Tile* tile, GPURenderer& renderer) {
    // Combine all area meshes into one
    if (!tile->area_meshes.empty()) {
        Mesh combined = combine_meshes(tile->area_meshes);
        tile->gpu_area_mesh_id = renderer.upload_mesh(combined);
    }

    // Same for buildings and roads
    if (!tile->building_meshes.empty()) {
        Mesh combined = combine_meshes(tile->building_meshes);
        tile->gpu_building_mesh_id = renderer.upload_mesh(combined);
    }

    if (!tile->road_meshes.empty()) {
        Mesh combined = combine_meshes(tile->road_meshes);
        tile->gpu_road_mesh_id = renderer.upload_mesh(combined);
    }

    tile->gpu_meshes_uploaded = true;
}

Mesh Editor::combine_meshes(const std::vector<Mesh>& meshes) {
    Mesh result;
    uint32_t vertex_offset = 0;

    for (const auto& mesh : meshes) {
        // Append vertices
        result.vertices.insert(result.vertices.end(),
                               mesh.vertices.begin(), mesh.vertices.end());

        // Append indices with offset
        for (uint32_t idx : mesh.indices) {
            result.indices.push_back(idx + vertex_offset);
        }

        vertex_offset += static_cast<uint32_t>(mesh.vertices.size());
    }

    result.compute_bounds();
    return result;
}
```

#### Step 3.4: Handle Tile Unloading

When tiles go out of view or are rebuilt, release GPU resources:

```cpp
void Editor::release_tile_gpu_meshes(osm::Tile* tile, GPURenderer& renderer) {
    if (tile->gpu_area_mesh_id != 0) {
        renderer.release_mesh(tile->gpu_area_mesh_id);
        tile->gpu_area_mesh_id = 0;
    }
    if (tile->gpu_building_mesh_id != 0) {
        renderer.release_mesh(tile->gpu_building_mesh_id);
        tile->gpu_building_mesh_id = 0;
    }
    if (tile->gpu_road_mesh_id != 0) {
        renderer.release_mesh(tile->gpu_road_mesh_id);
        tile->gpu_road_mesh_id = 0;
    }
    tile->gpu_meshes_uploaded = false;
}
```

---

### Phase 4: Debug Visualization

#### Step 4.1: Line Rendering Pipeline

For debug visualization (grid, tile bounds, gizmos), create a separate pipeline:

**File**: `src/renderer/gpu_renderer.hpp`

Add:

```cpp
struct LineVertex {
    glm::vec3 position;
    glm::vec4 color;
};

SDL_GPUGraphicsPipeline* m_line_pipeline = nullptr;
SDL_GPUBuffer* m_line_vertex_buffer = nullptr;
std::vector<LineVertex> m_line_vertices;

void draw_line(const glm::vec3& start, const glm::vec3& end, const glm::vec4& color);
void flush_lines();  // Called after all draw_line calls
```

#### Step 4.2: Line Shaders

Create `assets/shaders/line.vert` and `assets/shaders/line.frag` for simple position+color line rendering.

---

### Phase 5: Cleanup

#### Step 5.1: Remove Im3d

- Remove `src/editor/im3d_impl.cpp` and `src/editor/im3d_impl.hpp`
- Remove Im3d from CMakeLists.txt
- Remove Im3d include from `external/`
- Update editor to not call Im3d functions

#### Step 5.2: Remove SDL_Renderer References

- Remove any remaining `SDL_Renderer*` usage
- Remove `imgui_impl_sdlrenderer3` from CMakeLists.txt

---

## Shader Details

### Vertex Shader (`mesh.vert`)

```glsl
#version 450

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;
layout(location = 3) in vec4 in_color;

layout(set = 1, binding = 0) uniform MeshUniforms {
    mat4 mvp;
    mat4 model;
    vec4 color_tint;
} uniforms;

layout(location = 0) out vec3 frag_position;
layout(location = 1) out vec3 frag_normal;
layout(location = 2) out vec2 frag_uv;
layout(location = 3) out vec4 frag_color;

void main() {
    gl_Position = uniforms.mvp * vec4(in_position, 1.0);
    frag_position = (uniforms.model * vec4(in_position, 1.0)).xyz;
    frag_normal = mat3(uniforms.model) * in_normal;
    frag_uv = in_uv;
    frag_color = in_color * uniforms.color_tint;
}
```

### Fragment Shader (`mesh.frag`)

```glsl
#version 450

layout(location = 0) in vec3 frag_position;
layout(location = 1) in vec3 frag_normal;
layout(location = 2) in vec2 frag_uv;
layout(location = 3) in vec4 frag_color;

layout(location = 0) out vec4 out_color;

void main() {
    vec3 light_dir = normalize(vec3(0.5, 1.0, 0.3));
    vec3 normal = normalize(frag_normal);

    float ndotl = max(dot(normal, light_dir), 0.0);
    float ambient = 0.3;
    float diffuse = ndotl * 0.7;

    vec3 lit_color = frag_color.rgb * (ambient + diffuse);
    out_color = vec4(lit_color, frag_color.a);
}
```

### Compiling Shaders

```bash
cd assets/shaders
glslc mesh.vert -o mesh.vert.spv
glslc mesh.frag -o mesh.frag.spv
```

Or use `glslangValidator`:

```bash
glslangValidator -V mesh.vert -o mesh.vert.spv
glslangValidator -V mesh.frag -o mesh.frag.spv
```

---

## Performance Considerations

### Mesh Batching

Combine multiple small meshes into larger GPU buffers to reduce draw calls:

- All buildings in a tile → 1 draw call
- All roads in a tile → 1 draw call
- All areas in a tile → 1 draw call

### GPU Memory Management

- Upload meshes once, reuse GPU buffers
- Release GPU buffers when tiles are unloaded
- Consider a mesh cache with LRU eviction for very large maps

### Frame Pacing

- Use `SDL_SetGPUAllowedFramesInFlight()` to control latency vs throughput
- Default is typically 2-3 frames in flight

### Uniform Updates

- Push uniforms per-draw call (view-projection is the same, model changes per mesh)
- For instanced rendering (future), use storage buffers

---

## Testing Checklist

- [ ] GPU device creates successfully with Vulkan backend
- [ ] Shaders compile and load without errors
- [ ] Graphics pipeline creates successfully
- [ ] Mesh upload works (vertices and indices)
- [ ] Basic triangle renders correctly
- [ ] Depth testing works (no z-fighting)
- [ ] Camera movement updates view correctly
- [ ] ImGui renders on top of 3D content
- [ ] Tile frustum culling still works
- [ ] Async mesh building still works
- [ ] No GPU validation errors in debug mode
- [ ] Performance is improved vs Im3d

---

## Rollback Plan

If issues arise, the original Im3d-based rendering can be restored:

1. Revert Window to use SDL_Renderer
2. Revert ImGui to use imgui_impl_sdlrenderer3
3. Restore Im3d calls in Editor
4. Keep GPURenderer code for future migration attempt

---

## Future Improvements

### Level of Detail (LOD)

- Generate simplified meshes for distant tiles
- Switch LOD based on camera distance

### Instanced Rendering

- For repeated elements (trees, street lights)
- Use storage buffers for per-instance data

### Material System

- Multiple shaders for different material types
- Texture support for building facades

### Shadow Mapping

- Directional light shadows
- Cascaded shadow maps for large scenes

### Compute Shaders

- GPU-based frustum culling
- Mesh simplification on GPU

## SDL DOCS FOR REFERENCE

```text

SDL Wiki
[ front page | index | search | recent changes | git repo | offline html ]
CategoryGPU

The GPU API offers a cross-platform way for apps to talk to modern graphics hardware. It offers both 3D graphics and compute support, in the style of Metal, Vulkan, and Direct3D 12.

A basic workflow might be something like this:

The app creates a GPU device with SDL_CreateGPUDevice(), and assigns it to a window with SDL_ClaimWindowForGPUDevice()--although strictly speaking you can render offscreen entirely, perhaps for image processing, and not use a window at all.

Next, the app prepares static data (things that are created once and used over and over). For example:

    Shaders (programs that run on the GPU): use SDL_CreateGPUShader().
    Vertex buffers (arrays of geometry data) and other rendering data: use SDL_CreateGPUBuffer() and SDL_UploadToGPUBuffer().
    Textures (images): use SDL_CreateGPUTexture() and SDL_UploadToGPUTexture().
    Samplers (how textures should be read from): use SDL_CreateGPUSampler().
    Render pipelines (precalculated rendering state): use SDL_CreateGPUGraphicsPipeline()

To render, the app creates one or more command buffers, with SDL_AcquireGPUCommandBuffer(). Command buffers collect rendering instructions that will be submitted to the GPU in batch. Complex scenes can use multiple command buffers, maybe configured across multiple threads in parallel, as long as they are submitted in the correct order, but many apps will just need one command buffer per frame.

Rendering can happen to a texture (what other APIs call a "render target") or it can happen to the swapchain texture (which is just a special texture that represents a window's contents). The app can use SDL_WaitAndAcquireGPUSwapchainTexture() to render to the window.

Rendering actually happens in a Render Pass, which is encoded into a command buffer. One can encode multiple render passes (or alternate between render and compute passes) in a single command buffer, but many apps might simply need a single render pass in a single command buffer. Render Passes can render to up to four color textures and one depth texture simultaneously. If the set of textures being rendered to needs to change, the Render Pass must be ended and a new one must be begun.

The app calls SDL_BeginGPURenderPass(). Then it sets states it needs for each draw:

    SDL_BindGPUGraphicsPipeline()
    SDL_SetGPUViewport()
    SDL_BindGPUVertexBuffers()
    SDL_BindGPUVertexSamplers()
    etc

Then, make the actual draw commands with these states:

    SDL_DrawGPUPrimitives()
    SDL_DrawGPUPrimitivesIndirect()
    SDL_DrawGPUIndexedPrimitivesIndirect()
    etc

After all the drawing commands for a pass are complete, the app should call SDL_EndGPURenderPass(). Once a render pass ends all render-related state is reset.

The app can begin new Render Passes and make new draws in the same command buffer until the entire scene is rendered.

Once all of the render commands for the scene are complete, the app calls SDL_SubmitGPUCommandBuffer() to send it to the GPU for processing.

If the app needs to read back data from texture or buffers, the API has an efficient way of doing this, provided that the app is willing to tolerate some latency. When the app uses SDL_DownloadFromGPUTexture() or SDL_DownloadFromGPUBuffer(), submitting the command buffer with SDL_SubmitGPUCommandBufferAndAcquireFence() will return a fence handle that the app can poll or wait on in a thread. Once the fence indicates that the command buffer is done processing, it is safe to read the downloaded data. Make sure to call SDL_ReleaseGPUFence() when done with the fence.

The API also has "compute" support. The app calls SDL_BeginGPUComputePass() with compute-writeable textures and/or buffers, which can be written to in a compute shader. Then it sets states it needs for the compute dispatches:

    SDL_BindGPUComputePipeline()
    SDL_BindGPUComputeStorageBuffers()
    SDL_BindGPUComputeStorageTextures()

Then, dispatch compute work:

    SDL_DispatchGPUCompute()

For advanced users, this opens up powerful GPU-driven workflows.

Graphics and compute pipelines require the use of shaders, which as mentioned above are small programs executed on the GPU. Each backend (Vulkan, Metal, D3D12) requires a different shader format. When the app creates the GPU device, the app lets the device know which shader formats the app can provide. It will then select the appropriate backend depending on the available shader formats and the backends available on the platform. When creating shaders, the app must provide the correct shader format for the selected backend. If you would like to learn more about why the API works this way, there is a detailed blog post explaining this situation.

It is optimal for apps to pre-compile the shader formats they might use, but for ease of use SDL provides a separate project, SDL_shadercross , for performing runtime shader cross-compilation. It also has a CLI interface for offline precompilation as well.

This is an extremely quick overview that leaves out several important details. Already, though, one can see that GPU programming can be quite complex! If you just need simple 2D graphics, the Render API is much easier to use but still hardware-accelerated. That said, even for 2D applications the performance benefits and expressiveness of the GPU API are significant.

The GPU API targets a feature set with a wide range of hardware support and ease of portability. It is designed so that the app won't have to branch itself by querying feature support. If you need cutting-edge features with limited hardware support, this API is probably not for you.

Examples demonstrating proper usage of this API can be found here .
Performance considerations

Here are some basic tips for maximizing your rendering performance.

    Beginning a new render pass is relatively expensive. Use as few render passes as you can.
    Minimize the amount of state changes. For example, binding a pipeline is relatively cheap, but doing it hundreds of times when you don't need to will slow the performance significantly.
    Perform your data uploads as early as possible in the frame.
    Don't churn resources. Creating and releasing resources is expensive. It's better to create what you need up front and cache it.
    Don't use uniform buffers for large amounts of data (more than a matrix or so). Use a storage buffer instead.
    Use cycling correctly. There is a detailed explanation of cycling further below.
    Use culling techniques to minimize pixel writes. The less writing the GPU has to do the better. Culling can be a very advanced topic but even simple culling techniques can boost performance significantly.

In general try to remember the golden rule of performance: doing things is more expensive than not doing things. Don't Touch The Driver!
FAQ

Question: When are you adding more advanced features, like ray tracing or mesh shaders?

Answer: We don't have immediate plans to add more bleeding-edge features, but we certainly might in the future, when these features prove worthwhile, and reasonable to implement across several platforms and underlying APIs. So while these things are not in the "never" category, they are definitely not "near future" items either.

Question: Why is my shader not working?

Answer: A common oversight when using shaders is not properly laying out the shader resources/registers correctly. The GPU API is very strict with how it wants resources to be laid out and it's difficult for the API to automatically validate shaders to see if they have a compatible layout. See the documentation for SDL_CreateGPUShader() and SDL_CreateGPUComputePipeline() for information on the expected layout.

Another common issue is not setting the correct number of samplers, textures, and buffers in SDL_GPUShaderCreateInfo. If possible use shader reflection to extract the required information from the shader automatically instead of manually filling in the struct's values.

Question: My application isn't performing very well. Is this the GPU API's fault?

Answer: No. Long answer: The GPU API is a relatively thin layer over the underlying graphics API. While it's possible that we have done something inefficiently, it's very unlikely especially if you are relatively inexperienced with GPU rendering. Please see the performance tips above and make sure you are following them. Additionally, tools like RenderDoc can be very helpful for diagnosing incorrect behavior and performance issues.
System Requirements
Vulkan

SDL driver name: "vulkan" (for use in SDL_CreateGPUDevice() and SDL_PROP_GPU_DEVICE_CREATE_NAME_STRING)

Supported on Windows, Linux, Nintendo Switch, and certain Android devices. Requires Vulkan 1.0 with the following extensions and device features:

    VK_KHR_swapchain
    VK_KHR_maintenance1
    independentBlend
    imageCubeArray
    depthClamp
    shaderClipDistance
    drawIndirectFirstInstance
    sampleRateShading

You can remove some of these requirements to increase compatibility with Android devices by using these properties when creating the GPU device with SDL_CreateGPUDeviceWithProperties():

    SDL_PROP_GPU_DEVICE_CREATE_FEATURE_CLIP_DISTANCE_BOOLEAN
    SDL_PROP_GPU_DEVICE_CREATE_FEATURE_DEPTH_CLAMPING_BOOLEAN
    SDL_PROP_GPU_DEVICE_CREATE_FEATURE_INDIRECT_DRAW_FIRST_INSTANCE_BOOLEAN
    SDL_PROP_GPU_DEVICE_CREATE_FEATURE_ANISOTROPY_BOOLEAN

D3D12

SDL driver name: "direct3d12"

Supported on Windows 10 or newer, Xbox One (GDK), and Xbox Series X|S (GDK). Requires a GPU that supports DirectX 12 Feature Level 11_0 and Resource Binding Tier 2 or above.

You can remove the Tier 2 resource binding requirement to support Intel Haswell and Broadwell GPUs by using this property when creating the GPU device with SDL_CreateGPUDeviceWithProperties():

    SDL_PROP_GPU_DEVICE_CREATE_D3D12_ALLOW_FEWER_RESOURCE_SLOTS_BOOLEAN

Metal

SDL driver name: "metal"

Supported on macOS 10.14+ and iOS/tvOS 13.0+. Hardware requirements vary by operating system:

    macOS requires an Apple Silicon or Intel Mac2 family GPU
    iOS/tvOS requires an A9 GPU or newer
    iOS Simulator and tvOS Simulator are unsupported

Coordinate System

The GPU API uses a left-handed coordinate system, following the convention of D3D12 and Metal. Specifically:

    Normalized Device Coordinates: The lower-left corner has an x,y coordinate of (-1.0, -1.0). The upper-right corner is (1.0, 1.0). Z values range from [0.0, 1.0] where 0 is the near plane.
    Viewport Coordinates: The top-left corner has an x,y coordinate of (0, 0) and extends to the bottom-right corner at (viewportWidth, viewportHeight). +Y is down.
    Texture Coordinates: The top-left corner has an x,y coordinate of (0, 0) and extends to the bottom-right corner at (1.0, 1.0). +Y is down.

If the backend driver differs from this convention (e.g. Vulkan, which has an NDC that assumes +Y is down), SDL will automatically convert the coordinate system behind the scenes, so you don't need to perform any coordinate flipping logic in your shaders.
Uniform Data

Uniforms are for passing data to shaders. The uniform data will be constant across all executions of the shader.

There are 4 available uniform slots per shader stage (where the stages are vertex, fragment, and compute). Uniform data pushed to a slot on a stage keeps its value throughout the command buffer until you call the relevant Push function on that slot again.

For example, you could write your vertex shaders to read a camera matrix from uniform binding slot 0, push the camera matrix at the start of the command buffer, and that data will be used for every subsequent draw call.

It is valid to push uniform data during a render or compute pass.

Uniforms are best for pushing small amounts of data. If you are pushing more than a matrix or two per call you should consider using a storage buffer instead.
A Note On Cycling

When using a command buffer, operations do not occur immediately - they occur some time after the command buffer is submitted.

When a resource is used in a pending or active command buffer, it is considered to be "bound". When a resource is no longer used in any pending or active command buffers, it is considered to be "unbound".

If data resources are bound, it is unspecified when that data will be unbound unless you acquire a fence when submitting the command buffer and wait on it. However, this doesn't mean you need to track resource usage manually.

All of the functions and structs that involve writing to a resource have a "cycle" bool. SDL_GPUTransferBuffer, SDL_GPUBuffer, and SDL_GPUTexture all effectively function as ring buffers on internal resources. When cycle is true, if the resource is bound, the cycle rotates to the next unbound internal resource, or if none are available, a new one is created. This means you don't have to worry about complex state tracking and synchronization as long as cycling is correctly employed.

For example: you can call SDL_MapGPUTransferBuffer(), write texture data, SDL_UnmapGPUTransferBuffer(), and then SDL_UploadToGPUTexture(). The next time you write texture data to the transfer buffer, if you set the cycle param to true, you don't have to worry about overwriting any data that is not yet uploaded.

Another example: If you are using a texture in a render pass every frame, this can cause a data dependency between frames. If you set cycle to true in the SDL_GPUColorTargetInfo struct, you can prevent this data dependency.

Cycling will never undefine already bound data. When cycling, all data in the resource is considered to be undefined for subsequent commands until that data is written again. You must take care not to read undefined data.

Note that when cycling a texture, the entire texture will be cycled, even if only part of the texture is used in the call, so you must consider the entire texture to contain undefined data after cycling.

You must also take care not to overwrite a section of data that has been referenced in a command without cycling first. It is OK to overwrite unreferenced data in a bound resource without cycling, but overwriting a section of data that has already been referenced will produce unexpected results.
Debugging

At some point of your GPU journey, you will probably encounter issues that are not traceable with regular debugger - for example, your code compiles but you get an empty screen, or your shader fails in runtime.

For debugging such cases, there are tools that allow visually inspecting the whole GPU frame, every drawcall, every bound resource, memory buffers, etc. They are the following, per platform:

    For Windows/Linux, use RenderDoc
    For MacOS (Metal), use Xcode built-in debugger (Open XCode, go to Debug > Debug Executable..., select your application, set "GPU Frame Capture" to "Metal" in scheme "Options" window, run your app, and click the small Metal icon on the bottom to capture a frame)

Aside from that, you may want to enable additional debug layers to receive more detailed error messages, based on your GPU backend:

    For D3D12, the debug layer is an optional feature that can be installed via "Windows Settings -> System -> Optional features" and adding the "Graphics Tools" optional feature.
    For Vulkan, you will need to install Vulkan SDK on Windows, and on Linux, you usually have some sort of vulkan-validation-layers system package that should be installed.
    For Metal, it should be enough just to run the application from XCode to receive detailed errors or warnings in the output.

Don't hesitate to use tools as RenderDoc when encountering runtime issues or unexpected output on screen, quick GPU frame inspection can usually help you fix the majority of such problems.
Functions

    SDL_AcquireGPUCommandBuffer
    SDL_AcquireGPUSwapchainTexture
    SDL_BeginGPUComputePass
    SDL_BeginGPUCopyPass
    SDL_BeginGPURenderPass
    SDL_BindGPUComputePipeline
    SDL_BindGPUComputeSamplers
    SDL_BindGPUComputeStorageBuffers
    SDL_BindGPUComputeStorageTextures
    SDL_BindGPUFragmentSamplers
    SDL_BindGPUFragmentStorageBuffers
    SDL_BindGPUFragmentStorageTextures
    SDL_BindGPUGraphicsPipeline
    SDL_BindGPUIndexBuffer
    SDL_BindGPUVertexBuffers
    SDL_BindGPUVertexSamplers
    SDL_BindGPUVertexStorageBuffers
    SDL_BindGPUVertexStorageTextures
    SDL_BlitGPUTexture
    SDL_CalculateGPUTextureFormatSize
    SDL_CancelGPUCommandBuffer
    SDL_ClaimWindowForGPUDevice
    SDL_CopyGPUBufferToBuffer
    SDL_CopyGPUTextureToTexture
    SDL_CreateGPUBuffer
    SDL_CreateGPUComputePipeline
    SDL_CreateGPUDevice
    SDL_CreateGPUDeviceWithProperties
    SDL_CreateGPUGraphicsPipeline
    SDL_CreateGPUSampler
    SDL_CreateGPUShader
    SDL_CreateGPUTexture
    SDL_CreateGPUTransferBuffer
    SDL_DestroyGPUDevice
    SDL_DispatchGPUCompute
    SDL_DispatchGPUComputeIndirect
    SDL_DownloadFromGPUBuffer
    SDL_DownloadFromGPUTexture
    SDL_DrawGPUIndexedPrimitives
    SDL_DrawGPUIndexedPrimitivesIndirect
    SDL_DrawGPUPrimitives
    SDL_DrawGPUPrimitivesIndirect
    SDL_EndGPUComputePass
    SDL_EndGPUCopyPass
    SDL_EndGPURenderPass
    SDL_GDKResumeGPU
    SDL_GDKSuspendGPU
    SDL_GenerateMipmapsForGPUTexture
    SDL_GetGPUDeviceDriver
    SDL_GetGPUDeviceProperties
    SDL_GetGPUDriver
    SDL_GetGPUShaderFormats
    SDL_GetGPUSwapchainTextureFormat
    SDL_GetGPUTextureFormatFromPixelFormat
    SDL_GetNumGPUDrivers
    SDL_GetPixelFormatFromGPUTextureFormat
    SDL_GPUSupportsProperties
    SDL_GPUSupportsShaderFormats
    SDL_GPUTextureFormatTexelBlockSize
    SDL_GPUTextureSupportsFormat
    SDL_GPUTextureSupportsSampleCount
    SDL_InsertGPUDebugLabel
    SDL_MapGPUTransferBuffer
    SDL_PopGPUDebugGroup
    SDL_PushGPUComputeUniformData
    SDL_PushGPUDebugGroup
    SDL_PushGPUFragmentUniformData
    SDL_PushGPUVertexUniformData
    SDL_QueryGPUFence
    SDL_ReleaseGPUBuffer
    SDL_ReleaseGPUComputePipeline
    SDL_ReleaseGPUFence
    SDL_ReleaseGPUGraphicsPipeline
    SDL_ReleaseGPUSampler
    SDL_ReleaseGPUShader
    SDL_ReleaseGPUTexture
    SDL_ReleaseGPUTransferBuffer
    SDL_ReleaseWindowFromGPUDevice
    SDL_SetGPUAllowedFramesInFlight
    SDL_SetGPUBlendConstants
    SDL_SetGPUBufferName
    SDL_SetGPUScissor
    SDL_SetGPUStencilReference
    SDL_SetGPUSwapchainParameters
    SDL_SetGPUTextureName
    SDL_SetGPUViewport
    SDL_SubmitGPUCommandBuffer
    SDL_SubmitGPUCommandBufferAndAcquireFence
    SDL_UnmapGPUTransferBuffer
    SDL_UploadToGPUBuffer
    SDL_UploadToGPUTexture
    SDL_WaitAndAcquireGPUSwapchainTexture
    SDL_WaitForGPUFences
    SDL_WaitForGPUIdle
    SDL_WaitForGPUSwapchain
    SDL_WindowSupportsGPUPresentMode
    SDL_WindowSupportsGPUSwapchainComposition

Datatypes

    SDL_GPUBuffer
    SDL_GPUBufferUsageFlags
    SDL_GPUColorComponentFlags
    SDL_GPUCommandBuffer
    SDL_GPUComputePass
    SDL_GPUComputePipeline
    SDL_GPUCopyPass
    SDL_GPUDevice
    SDL_GPUFence
    SDL_GPUGraphicsPipeline
    SDL_GPURenderPass
    SDL_GPUSampler
    SDL_GPUShader
    SDL_GPUShaderFormat
    SDL_GPUTexture
    SDL_GPUTextureUsageFlags
    SDL_GPUTransferBuffer

Structs

    SDL_GPUBlitInfo
    SDL_GPUBlitRegion
    SDL_GPUBufferBinding
    SDL_GPUBufferCreateInfo
    SDL_GPUBufferLocation
    SDL_GPUBufferRegion
    SDL_GPUColorTargetBlendState
    SDL_GPUColorTargetDescription
    SDL_GPUColorTargetInfo
    SDL_GPUComputePipelineCreateInfo
    SDL_GPUDepthStencilState
    SDL_GPUDepthStencilTargetInfo
    SDL_GPUGraphicsPipelineCreateInfo
    SDL_GPUGraphicsPipelineTargetInfo
    SDL_GPUIndexedIndirectDrawCommand
    SDL_GPUIndirectDispatchCommand
    SDL_GPUIndirectDrawCommand
    SDL_GPUMultisampleState
    SDL_GPURasterizerState
    SDL_GPUSamplerCreateInfo
    SDL_GPUShaderCreateInfo
    SDL_GPUStencilOpState
    SDL_GPUStorageBufferReadWriteBinding
    SDL_GPUStorageTextureReadWriteBinding
    SDL_GPUTextureCreateInfo
    SDL_GPUTextureLocation
    SDL_GPUTextureRegion
    SDL_GPUTextureSamplerBinding
    SDL_GPUTextureTransferInfo
    SDL_GPUTransferBufferCreateInfo
    SDL_GPUTransferBufferLocation
    SDL_GPUVertexAttribute
    SDL_GPUVertexBufferDescription
    SDL_GPUVertexInputState
    SDL_GPUViewport
    SDL_GPUVulkanOptions

Enums

    SDL_GPUBlendFactor
    SDL_GPUBlendOp
    SDL_GPUCompareOp
    SDL_GPUCubeMapFace
    SDL_GPUCullMode
    SDL_GPUFillMode
    SDL_GPUFilter
    SDL_GPUFrontFace
    SDL_GPUIndexElementSize
    SDL_GPULoadOp
    SDL_GPUPresentMode
    SDL_GPUPrimitiveType
    SDL_GPUSampleCount
    SDL_GPUSamplerAddressMode
    SDL_GPUSamplerMipmapMode
    SDL_GPUShaderStage
    SDL_GPUStencilOp
    SDL_GPUStoreOp
    SDL_GPUSwapchainComposition
    SDL_GPUTextureFormat
    SDL_GPUTextureType
    SDL_GPUTransferBufferUsage
    SDL_GPUVertexElementFormat
    SDL_GPUVertexInputRate

Macros

    (none.)

CategoryAPICategory
[ edit | delete | history | feedback | raw ]

All wiki content is licensed under Creative Commons Attribution 4.0 International (CC BY 4.0).
Wiki powered by ghwikipp.

```
