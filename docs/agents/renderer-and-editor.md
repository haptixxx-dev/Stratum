# Renderer and editor

Orientation for agents new to the rendering/UI half of Stratum (C++20, SDL3 + SDL_GPU on Vulkan, Dear ImGui, EnTT). Written 2026-09-11 against branch `fix/road-junction-semantics` (post-lighting-v2, commit `00fb718`). The root `CLAUDE.md` is ~3 months stale — trust this file and the code over it. Notable stale claims: CLAUDE.md says "no tests exist" (there are ~48 test files under `tests/`, including `tests/renderer/`), lists panels that live elsewhere, and describes the uniform layout as "set 0 scene, set 1 per-mesh", which does not match the SDL_GPU set convention actually used (see Shaders section).

## Frame flow

## src/renderer

Files (all under `src/renderer/`):

- `gpu_renderer.{hpp,cpp}` — the whole SDL_GPU device/pipeline/frame layer. `GPURenderer` owns the device, all graphics pipelines (simple, PBR solid/wireframe, decal variants, sky, shadow, and the Im3d ones via its `load_shader()` helper), the shadow atlas, MSAA targets, the mesh pools and the upload staging system. The header is exceptionally heavily documented — read its doc comments before touching anything; most invariants below are stated there.
- `mesh.hpp` — header-only, compiled into BOTH `stratum_core` and `stratum_editor_lib`; any out-of-line definition added there must be `inline` (adding a `mesh.cpp` is an ODR/link trap, the header says so explicitly). CPU-side `Mesh` (vertices, indices, `SubMesh` material ranges), `Vertex`, `MaterialId`/`MaterialKey`.
- `material_library.hpp/cpp` — `MaterialLibrary`, `MaterialUniforms` (the real per-draw material block; three vec4s), material defs incl. decal properties (`alpha_blend`, `depth_bias`).
- `texture.hpp/cpp` — `GPUTextureManager`; owns textures, three built-in 1x1 neutral maps (albedo/normal/ORM), staged texture uploads flushed inside the renderer's per-frame copy pass.
- `gpu_buffer_pool.hpp/cpp` — vertex/index suballocation pools; `BufferAlloc` = {buffer, offset}. Exists because a city extract is thousands of meshes and Vulkan's `maxMemoryAllocationCount` (commonly 4096) is hit long before VRAM runs out.
- `procedural_texture.hpp/cpp` — generated textures (unverified detail; not read for this doc).

Key renderer facts:

- Two shader modes switchable at runtime via `set_shader_mode()`: `ShaderMode::Simple` (Blinn-Phong-ish, no materials, no sky, no shadows) and `ShaderMode::PBR` (Cook-Torrance, materials, analytic sky, cascaded shadows, ACES tone map, fog). `pbr_path_available()` (both PBR pipelines AND a texture manager present) is the single predicate gating every PBR decision; do not invent a second one.
- Materials: a draw is one draw call per `SubMesh` range; `bind_material(MaterialKey)` pushes `MaterialUniforms` and binds albedo/normal/ORM samplers, skipping redundant binds (cache invalidated whenever a render pass opens — SDL_GPU bindings do not survive a pass). Decal materials (road markings) need a separate pipeline per quantized depth bias because blend state and depth bias are pipeline state in SDL_GPU, not dynamic; cache capped at `kMaxDecalPipelines` (32).
- Reverse-Z: depth test is GREATER, near plane at depth 1. Any depth bias must carry the reverse-Z sign — see `GPURenderer::decal_depth_bias()` doc for the bug that motivated it (markings pushed away from camera).
- Frame stats (`get_frame_stats()`) report the last COMPLETED frame because UI panels draw before the 3D pass.
- MSAA changes rebuild all pipelines (sample count is baked in) and fire `set_msaa_changed_callback` so ImGui can reinit.

## GPU resources and handles

Handles are `uint32_t` mesh ids from `GPURenderer::upload_mesh(const Mesh&)`, released with `release_mesh(id)`. Never hold raw SDL pointers outside the renderer.

- **Pooled buffers.** A `GPUMesh` does NOT own SDL buffers; it holds two `BufferAlloc` ranges suballocated from the renderer's vertex and index pools (`gpu_buffer_pool.hpp`). Binding uses `SDL_GPUBufferBinding{buffer, offset}` and a vertex offset of 0; indices stay mesh-local and zero-based. `SubMesh::index_offset` passes straight through as `first_index`.
- **Deferred uploads.** `upload_mesh()` reserves ranges and copies bytes into a CPU staging arena immediately, but the GPU copy is DEFERRED into a per-frame batched flush (`flush_pending_uploads`, one transfer buffer per frame, budgeted — `plan_upload_batch()`). `GPUMesh::ready` is false until the copy ran; `draw_mesh()` silently skips not-ready meshes (they appear a frame later). The staging arena is append-only with amortised compaction (`staging_compaction_offset()`): only compacts when the dead prefix is ≥ half the arena.
- **Memory budget and eviction.** `MemoryBudget`: `max_resident_bytes` (default 768 MB, a policy number — SDL_GPU cannot query VRAM), `max_resident_meshes` (4096), `evict_under_pressure` (true; when false, over-budget uploads are refused instead and `upload_failures()` increments). The renderer knows no positions, so the owner (the quadtree traversal) registers `MeshDistanceFn`; a NEGATIVE distance means PINNED (never evicted — use for gizmos/grid/overlays). `evict_to_budget()` releases furthest-first; without a distance fn it refuses and logs (evicting at random would drop the road under the camera). Meshes with staged-but-unflushed uploads are never evicted. Evicted ranges go to a retirement queue and are freed only after in-flight frames complete.
- **Eviction callback.** `MeshEvictedFn` fires once per evicted mesh BEFORE the id is recycled so owners (quadtree nodes, ECS components) can forget the handle. Do NOT call back into the renderer from inside it — it runs mid-map-walk.
- **Vertex layout.** `Vertex` = position, normal, uv, color, tangent (w = bitangent sign), plus `ao` (baked ambient occlusion, defaults 1). 68 bytes — NOT a power of two, so `kVertexAlignment` in the pool is pinned at 64 rather than `sizeof(Vertex)` (68 would round to 128). Stride need not divide the offset because every draw binds at the mesh's first vertex.
- **Materials on meshes.** `Mesh::submeshes` empty = one implicit `MaterialId::Default` range; consume via `effective_submeshes()`, which always tiles `[0, indices.size())`. `(MaterialId, variant)` = `MaterialKey` is the lookup key; variant 0 = slot default; variants are data-driven from OSM tags (`osm/road/road_style.hpp`). `MaterialId` numeric values are a sort key and export identity — never reorder the enum. `sort_submeshes_by_material()` minimizes material binds; compare `FrameStats::material_binds` vs `draw_calls` to see if ranges are unsorted.

## Shaders and uniform sets

## src/editor

## src/editor/panels

## Lighting v2

Commit `00fb718` "feat(render): analytic sky, cascaded shadows, baked AO" (2026-08-25). Absent from all older docs. Sources: the commit message (long and precise — read it with `git log -1 00fb718`), `gpu_renderer.hpp`, `sky_common.glsl`, `src/geometry/ambient_occlusion.{hpp,cpp}`.

**Analytic sky + image-based lighting (IBL).** `assets/shaders/sky_common.glsl` is included by BOTH `sky.frag` and `mesh_pbr.frag`, so the sky dome and the ambient fill light are two evaluations of one function and cannot disagree — that is the design's whole point. Ambient diffuse is a hemisphere integral of that sky (upper lobe = sky, lower lobe = `ground_color` bounce, which is what lights a bridge soffit). Specular ambient is the split-sum with NO prefiltered cubemap and NO BRDF lookup table: the environment is analytic, evaluated directly at the reflection vector, with Karis's polynomial BRDF fit. Zero VRAM, no prefilter pass, tracks the sun instantly. Sky parameters live in `SceneUniforms` (`sky_zenith`, `sky_horizon`, `ground_color`, `ibl_params`) as scene-referred radiance, NOT display colours; exposure + ACES bring them to screen. There is deliberately no separate "ambient colour" — the fill light IS the sky. Fog can fade into the sky along the view ray (`ibl_params.z`, aerial perspective). `draw_sky()` runs inside the 3D pass BEFORE any geometry, after `set_view_projection()`/`set_camera_position()` (it reconstructs a per-pixel world ray from the inverse view-projection); it draws nothing in Simple mode.

**Cascaded shadow maps.** Three (up to `kMaxShadowCascades` = 4, mirrored by a GLSL array size in `mesh_pbr.frag` — change both together) sphere-fitted, texel-snapped cascades. Sphere fit because a sphere is rotation-invariant: turning the camera cannot resize the volume and make shadow edges crawl. The depth pass culls FRONT faces (puts stored depth a wall-thickness away, kills most acne), plus normal-offset bias scaled per-cascade by texel size, plus a constant bias authored in WORLD METRES (`ShadowConfig::depth_bias_metres`) converted per cascade. The cascades are tiles in ONE 2D `D32_FLOAT` depth atlas (width = `map_size * cascade_count`), because SDL_GPU rejects array textures with `DEPTH_STENCIL_TARGET` usage — and the atlas fills all cascades in a single pass with one clear anyway. Sampling uses a hardware comparison sampler (`COMPAREOP_LESS`), 3x3 PCF. Casters are LAST frame's `draw_mesh()` calls, replayed (`render_shadow_cascades()` between `begin_frame()` and `begin_render_pass()`); re-traversing the quadtree per cascade would fire LOD/streaming/stats three extra times. Known honest cost: a caster outside the camera frustum casts nothing into it — visible as a missing shadow at a screen edge with a low sun. `ShadowUniforms::shadow_params.x` is the live cascade count and 0 is the documented OFF switch (fully lit, no comparisons against uninitialised depth) — tests rely on it.

**Baked ambient occlusion.** New `Vertex::ao` channel (this is what pushed Vertex to 68 bytes). Multiplies the material/ORM ao, so it attenuates AMBIENT only — deliberately NOT folded into vertex colour, which would darken direct sunlight too. Baked, not screen-space: SSAO would need a depth prepass (the colour pass's depth is being written, and unsampleable under MSAA), and a baked value travels with the exported mesh, which is the point of the tool. Baker: `src/geometry/ambient_occlusion.{hpp,cpp}` — BVH + cosine-weighted hemisphere sampling, quadratic distance falloff, deterministic at any thread count. Buildings bake on the MERGED leaf mesh (so they occlude each other) with an analytic ground plane (OSM buildings have no floor; without it every wall foot baked as open as its top). Terrain bakes against itself at 60 m range.

**Correctness fixes shipped in the same commit** (regressions to not reintroduce):
1. `mesh_pbr.frag` computed the view vector as `normalize(-frag_world_pos)` ("camera near origin") — wrong by kilometres in a local-metres map; it also fed the double-sided branch, INVERTING normals on facades facing away from origin.
2. Exposure was written by the UI and read by nothing. Now `SceneUniforms::camera_position.w`.
3. Ambient ignored `sun_color` and metallic — metals read as grey plastic.
4. Editor bound the mesh pipeline (which pushes `SceneUniforms`) before publishing the camera — every frame shaded with last frame's eye. Order matters: camera first, then pipeline bind.
5. `update_scene_uniforms()` used `sun_direction.w <= 0` as "uninitialised", so dragging Sun Intensity to 0 reseeded the light block and the slider snapped back.

## Gotchas
