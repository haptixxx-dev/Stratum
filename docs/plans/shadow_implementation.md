# Shadow Implementation Plan

## Overview

Add real-time shadow mapping to the PBR renderer using the directional sun light.

## Complexity Estimate

- **Basic with PCF**: Medium (~150-200 lines C++, ~50 lines GLSL)
- **Time**: 1-2 hours

## Implementation Steps

### 1. Shadow Map Texture

- Create depth-only texture (2048x2048 recommended)
- Format: `SDL_GPU_TEXTUREFORMAT_D32_FLOAT`
- Usage: `SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER`

### 2. Shadow Pipeline

Create a minimal pipeline for the shadow pass:

**Shadow Vertex Shader** (`shadow.vert`):

```glsl
#version 450

layout(location = 0) in vec3 in_position;

layout(set = 1, binding = 0) uniform ShadowUniforms {
    mat4 light_space_matrix;
} uniforms;

void main() {
    gl_Position = uniforms.light_space_matrix * vec4(in_position, 1.0);
}
```

**Shadow Fragment Shader** (`shadow.frag`):

```glsl
#version 450
// Empty - depth-only pass
void main() {}
```

### 3. Light-Space Matrix Calculation

For directional light (orthographic projection):

```cpp
glm::mat4 compute_light_space_matrix(const glm::vec3& light_dir, const glm::vec3& scene_center, float scene_radius) {
    glm::vec3 light_pos = scene_center - light_dir * scene_radius * 2.0f;
    glm::mat4 light_view = glm::lookAt(light_pos, scene_center, glm::vec3(0, 1, 0));
    glm::mat4 light_proj = glm::ortho(-scene_radius, scene_radius,
                                       -scene_radius, scene_radius,
                                       0.1f, scene_radius * 4.0f);
    return light_proj * light_view;
}
```

### 4. Shadow Render Pass

Before main render pass:

1. Bind shadow pipeline
2. Set shadow map as depth target
3. Push light-space matrix as vertex uniform
4. Render all shadow-casting meshes
5. End pass

### 5. Fragment Shader Changes (`mesh_pbr.frag`)

Add sampler and uniforms:

```glsl
// set = 2 is for fragment samplers/textures in SDL_GPU
layout(set = 2, binding = 0) uniform sampler2D shadow_map;

// Add to SceneUniforms:
mat4 light_space_matrix;
```

Add shadow calculation:

```glsl
float calculate_shadow(vec3 world_pos) {
    vec4 light_space_pos = scene.light_space_matrix * vec4(world_pos, 1.0);
    vec3 proj_coords = light_space_pos.xyz / light_space_pos.w;
    proj_coords = proj_coords * 0.5 + 0.5; // Transform to [0,1]

    if (proj_coords.z > 1.0) return 0.0; // Outside shadow map

    float current_depth = proj_coords.z;
    float bias = 0.005; // Prevents shadow acne

    // PCF soft shadows (3x3 kernel)
    float shadow = 0.0;
    vec2 texel_size = 1.0 / textureSize(shadow_map, 0);
    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            float pcf_depth = texture(shadow_map, proj_coords.xy + vec2(x, y) * texel_size).r;
            shadow += current_depth - bias > pcf_depth ? 1.0 : 0.0;
        }
    }
    return shadow / 9.0;
}
```

Apply in lighting:

```glsl
float shadow = calculate_shadow(frag_world_pos);
vec3 Lo = (diffuse + specular) * light_color * light_intensity * NdotL * (1.0 - shadow);
```

### 6. GPURenderer Changes

Add members:

```cpp
SDL_GPUTexture* m_shadow_map = nullptr;
SDL_GPUGraphicsPipeline* m_shadow_pipeline = nullptr;
SDL_GPUShader* m_shadow_vertex_shader = nullptr;
SDL_GPUShader* m_shadow_fragment_shader = nullptr;
SDL_GPUSampler* m_shadow_sampler = nullptr;
glm::mat4 m_light_space_matrix{1.0f};
```

Add to SceneUniforms struct:

```cpp
glm::mat4 light_space_matrix;  // 64 bytes
```

New methods:

```cpp
bool create_shadow_resources();
void render_shadow_pass();
void release_shadow_resources();
```

### 7. SDL_GPU Descriptor Set Reference

| Set | Stage | Purpose |
|-----|-------|---------|
| 0 | Vertex | Samplers/textures/storage |
| 1 | Vertex | **Uniform buffers** |
| 2 | Fragment | **Samplers/textures/storage** |
| 3 | Fragment | **Uniform buffers** |

## Future Enhancements

### Cascaded Shadow Maps (CSM)

For large outdoor scenes, split the view frustum into 2-4 cascades with separate shadow maps. Better quality at all distances.

### Variance Shadow Maps (VSM)

Store depth and depth² for softer, filter-friendly shadows. Reduces aliasing but can have light bleeding.

### Contact Shadows

Screen-space ray marching for small-scale shadows that shadow maps miss.

## Notes

- Shadow map resolution affects quality vs performance
- Bias value may need tuning to prevent shadow acne while avoiding peter-panning
- Consider culling meshes outside light frustum for performance
