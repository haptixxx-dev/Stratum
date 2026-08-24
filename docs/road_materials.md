# Road materials

How a `MaterialKey` becomes a bound texture and a lit pixel, what the numbers in
it mean, and what has to be reconciled when `feat/road-optimization` merges.

Everything described here lives in `stratum_editor_lib`
(`src/renderer/**`, `src/editor/**`). `stratum_core` does not know that materials
exist; it emits geometry tagged with slots and variants and stops there.

---

## 1. What the system is

Road geometry has carried metre-based tiling UVs since P2 and material slots
since P0.3. Until this phase none of it reached a shader: `draw_mesh()` already
issued one `SDL_DrawGPUIndexedPrimitives` per submesh range, but every range drew
with the same pipeline, no textures, and a `material_id` parameter that was
threaded through and never read.

Five files close that gap.

| File | Owns |
|---|---|
| `src/renderer/texture.hpp/.cpp` | `SDL_GPUTexture` objects, the handle table, the shared samplers, the four built-in fallbacks, and the batched upload path |
| `src/renderer/procedural_texture.hpp/.cpp` | Pure pixel generators — asphalt, concrete, cobbles, setts, paving, gravel, dirt, grass, kerb, and the markings atlas. No device, no SDL, no I/O |
| `src/renderer/material_library.hpp/.cpp` | `MaterialKey -> MaterialDef` resolution, the built-in slot table, the variant table, and JSON material sets |
| `src/renderer/gpu_renderer.cpp` | `bind_material()`: the per-range uniform push and sampler bind, the decal pipelines, and the redundant-bind cache |
| `src/editor/panels/material_panel.cpp` | The Materials panel: live editing, diagnostics, set load/save |

Two things it deliberately does **not** have. There is no bindless path —
`SDL_GPU` has none, so textures are bound per draw through
`SDL_BindGPUFragmentSamplers`. And there is no material atlas for surfaces: the
plan's UV convention tiles in metres, one submesh per material, which is what
makes texel density uniform across a whole network.

### Upload discipline

`GPUTextureManager` does **not** own a command buffer. `create()` and `load()`
copy pixels into a staging arena, create the `SDL_GPUTexture`, and return a
handle that is *not yet readable*. `GPURenderer::flush_pending_uploads()` calls
`flush_uploads()` from inside the copy pass it already opens at the top of
`begin_frame()`, so textures ride the same one-command-buffer, one-copy-pass,
one-submit batch that meshes do. A texture path that opened its own command
buffer per load would bring back the exact fence-pool exhaustion
(`vkCreateFence VK_ERROR_OUT_OF_HOST_MEMORY`) the batching was written to fix.

Readiness is contingent on the **submit**, not on the recording:
`flush_uploads()` moves its entries to an in-flight list, and
`commit_uploads(submitted)` — called by `flush_pending_uploads()` with the result
of `SDL_SubmitGPUCommandBuffer` — either marks them ready and frees their staged
bytes, or puts them back on the front of the queue with their pixels intact.
`bind_texture()` substitutes the fallback for anything not ready, so a road that
is still streaming reads as *loading* rather than as static.

The one exception is `GPUTextureManager::init()`, which uploads the four
built-in fallbacks on its own command buffer and blocks on `SDL_WaitForGPUIdle()`
once, at startup, before any frame exists. Every later upload depends on those
being samplable. It is a documented single occurrence, not a precedent.

---

## 2. The MaterialKey resolution chain

```
MaterialKey { MaterialId material; uint16_t variant; }   ->   const MaterialDef&
```

`MaterialLibrary::resolve()` never fails and never returns null. It walks four
links, counting each in `resolve_stats()`:

| Link | Tried | Why |
|---|---|---|
| 1 | the exact `(slot, variant)` | the hit |
| 2 | `(slot, 0)` — the slot default | **the one that matters.** Variants are assigned by the geometry side; an unknown one must draw as plain asphalt, not as magenta |
| 3 | `(Default, 0)` | a slot with no entry at all |
| 4 | the built-in `m_fallback` | valid before `init()`, never removed, so a draw is never lost |

Link 2 is the entire reason the renderer can be written against a variant
numbering it does not control. A number that turns out to disagree with the
geometry side produces *a cobbled street drawn as plain asphalt*, reported by
`resolve_stats().variant_fallbacks`. It is never a crash and never a missing draw.

`MaterialDef` carries what a range needs: `base_color`, the four PBR scalars,
`uv_scale`, three `TextureHandle`s, a `SamplerKind`, and the two pipeline flags
`alpha_blend` and `depth_bias`.

### base_color is a TINT once a map exists

This is worth stating plainly because it changed:

* With the 1x1 white fallback bound, `base_color` **is** the surface albedo. The
  slot table authors it that way on purpose, so `load_defaults()` alone produces
  a usable, differentiated world with no assets present at all.
* With a real albedo map bound, the **map** is the surface albedo and
  `base_color` is a neutral tint.

`mesh_pbr.frag` computes `albedo = frag_color.rgb * base_color.rgb * albedo_tex.rgb`,
so keeping both meant multiplying two independently-authored full albedos —
Asphalt's `0.11` against a generated map authored at `~0.045` linear, giving
`0.005`, about a ninth of either. `install_procedural_textures()` therefore
resets `base_color` to white (preserving alpha, which is coverage rather than
colour) for every material it attaches a generated map to, and leaves it alone
for any material still bound to the white fallback.

### The unit ORM

Every built-in material binds `GPUTextureManager::default_orm()`, a 1x1 linear
texel. The shader multiplies **all three** channels into an authored scalar:

```glsl
ao        = material.pbr_params.z * orm.r
roughness = material.pbr_params.y * orm.g
metallic  = material.pbr_params.x * orm.b
```

so the default texel is `kDefaultOrmTexel = {255, 255, 255, 255}` — a true
multiplicative identity. A zero in any channel is an *annihilator*, not an
identity, and since every material binds this one texture it would silently zero
that scalar project-wide. It is a named constant with a test on it
(`TextureUpload.the_default_orm_texel_is_unit_in_every_channel`) for that reason.

### Materials off is not "bind nothing"

`GPURenderer::material_bind_mode()` is the whole decision, and it has three
answers rather than two:

| State | Mode |
|---|---|
| `ShaderMode::Simple`, or no PBR path | `Skip` — the simple shader declares no material resources |
| PBR, but materials disabled or no library | `Neutral` — a default `MaterialUniforms` and the three built-in 1x1 maps |
| PBR with a live library | `Full` — resolve the key |

The PBR fragment shader is created with `num_samplers = kMaterialSamplerCount`
and `num_uniform_buffers = kPbrFragmentUniformBufferCount`. SDL takes those
counts at their word and builds a pipeline layout from them, so **every** draw
through the PBR pipeline must leave all three sampler slots written.
`SDL_GPU_CheckGraphicsBindings()` fires
`SDL_assert_release(!"Missing fragment sampler binding!")` once per unbound slot
per draw under `debug_mode`, which `GPURenderer::init()` passes as a literal
`true` in every build — and `SDL_assert_release` survives `NDEBUG`. Returning
early from `bind_material()` therefore aborts the process, it does not merely
draw wrong.

For the same reason `set_shader_mode(ShaderMode::PBR)` refuses while there is no
`GPUTextureManager`: with no manager there is not one legal texture to fill those
slots with. `pbr_path_available()` is the single predicate behind
`set_shader_mode()`, `bind_mesh_pipeline()`, `draw_mesh()`'s uniform layout choice
and `bind_material()`, so they cannot disagree about which pipeline is bound.

---

## 3. Variant numbers — reconcile these at merge

> **These numbers are an assumption, not a contract.**

`SubMesh::variant` is a plain `uint16_t` whose meaning is assigned by the
tag-to-style mapping in `src/osm/road/road_style.hpp`, which lives on
`feat/road-optimization` and **does not exist in this branch**. Nothing in the
renderer reads that header and nothing here may — the two sides were written in
parallel.

`namespace stratum::material_variant` in `src/renderer/material_library.hpp` is
the **only** place a variant number appears in the renderer. Reconciling the two
numberings is an edit to these constants and nothing else.

| Constant | Value | Meaning |
|---|---|---|
| `kDefault` | 0 | The slot's default. Installed for every slot; never falls back |
| `kSmooth` | 1 | Fine, well-maintained, freshly laid. **Granite** for `Curb` |
| `kWorn` | 2 | Coarse, weathered, patched. **Worn concrete** for `Curb` |
| `kCobblestone` | 3 | Rounded cobbles in wide mortar joints |
| `kSett` | 4 | Squared setts in tight joints |
| `kPavingStones` | 5 | Rectangular paving slabs or blocks |
| `kTactilePaving` | 6 | Blister tactile paving. `Sidewalk` only |
| `kVariantCount` | 7 | First number never assigned |

One shared numbering across slots, not a private list per slot: the geometry side
derives a variant from `surface=`, and `surface=cobblestone` means the same thing
on a carriageway, a footway and a bridge deck.

**At merge time**, check `road_style.hpp`'s numbering against this table and edit
this table if they differ. Do not edit `road_style.hpp` to match the renderer —
the geometry side owns the meaning. The symptom of getting it wrong is cosmetic
and reported: `MaterialLibrary::resolve_stats().variant_fallbacks` climbs and
surfaces draw as their slot default.

The library is keyed on `MaterialKey` and works for **any** `(slot, variant)`
pair, including numbers it has never seen, so a mismatch cannot break a build or
a frame.

---

## 4. The UV convention the shaders implement

Frozen in `docs/plans/road_network_plan.md`. The renderer is its first consumer
and implements it exactly.

Surfaces tile in **metres**, one submesh per material, no atlas:

```
U = lateral_metres_within_strip     / tile_u_metres(material)
V = arclength_metres_along_road     / tile_v_metres(material)
```

| Material | tile U (m) | tile V (m) |
|---|---|---|
| Asphalt | 8.0 | 8.0 |
| Concrete, BridgeDeck | 4.0 | 4.0 |
| Sidewalk | 2.0 | 2.0 |
| Curb | 0.5 | 2.0 |
| Gravel, Dirt, Grass | 4.0 | 4.0 |

**Curb faces are vertical**, so their `U` runs *up* the face
(`U = height_up_face / 0.5`); the curb top strip uses the normal lateral
convention. Both share one texture, authored with the face on one half of `U` and
the top on the other. This is why `Curb`'s `uv_scale` must stay `{1, 1}`: any
other value slides the split off the geometry's 0.5 boundary.

**Markings are the only atlased material.** Their geometry carries explicit atlas
sub-rect UVs from `src/osm/road/marking_atlas.hpp` — a 1024x1024 atlas on a 16x16
grid of 64 px cells. An atlas rect cannot wrap, so `MaterialId::Markings` uses
`SamplerKind::ClampLinear` and must keep `uv_scale = {1, 1}`; scaling it samples
the neighbouring sprite. `MaterialLibrary::set()` warns if it is given anything
else. The atlas gets a deliberately **short** mip chain
(`kMarkingAtlasMipLevels = 4`): its one-pixel inset survives a couple of halvings
and does not survive ten.

`MaterialDef::uv_scale` is a **correction on top** of all this, normally `1.0`,
for a texture whose physical size does not match the convention. The generated
cobble and sett fields are authored at 4 m rather than Asphalt's 8 m — a
realistic 0.16 m sett across an 8 m tile is 50 stones, which at 512 texels is 10
texels a stone and reads as noise — so they carry `uv_scale = {2, 2}` to put them
back at the right world size. The shader applies it as
`vec2 uv = frag_uv * material.uv_params.xy;`.

---

## 5. Sets, bindings and slots

`SDL_GPU` numbers its own descriptor sets. For a **fragment** shader, sampler
slot *n* is `set = 2, binding = n`, and uniform slot *n* is
`set = 3, binding = n`. For a **vertex** shader, uniform slot 0 is
`set = 1, binding = 0`. The constants live in `material_library.hpp`, and
`assets/shaders/mesh_pbr.frag` is documented against them.

| Resource | C++ constant | GLSL | Contents |
|---|---|---|---|
| Mesh uniforms | vertex slot 0 | `set = 1, binding = 0` | `MeshUniformsPBR`: mvp, model, normal matrix, colour tint, camera position |
| Scene uniforms | `kSceneUniformSlot` = 0 | `set = 3, binding = 0` | `SceneUniforms`: camera, sun, fog. Frame-constant; pushed once per pipeline bind |
| Material uniforms | `kMaterialUniformSlot` = 1 | `set = 3, binding = 1` | `MaterialUniforms`: 3 vec4s, 48 bytes |
| Albedo | `kAlbedoSamplerSlot` = 0 | `set = 2, binding = 0` | sRGB colour |
| Normal | `kNormalSamplerSlot` = 1 | `set = 2, binding = 1` | Linear tangent-space normal |
| ORM | `kOrmSamplerSlot` = 2 | `set = 2, binding = 2` | Linear occlusion / roughness / metallic |

`MaterialUniforms` is three `vec4`s and nothing else — `base_color`,
`pbr_params` (metallic, roughness, ao, emissive) and `uv_params` (uv_scale.xy plus
two unused). Every member is a `vec4` so every member is already 16-byte aligned
and there is no std140 padding to get wrong. Do not "tidy" it into loose floats.
`tests/renderer/test_material_uniforms.cpp` asserts the C++ offsets against the
offsets `glslc` computed *in the shipped `.spv`*.

`SceneUniforms::pbr_params` is **reserved and unread**. The shader takes metallic,
roughness and ao from the material block; the three Render Settings sliders that
used to drive that member have been removed, because the compiled module contains
no access to it at all.

### Pipelines

Blending and depth bias are **pipeline** state in `SDL_GPU` —
`SDL_GPUColorTargetBlendState` and `SDL_GPURasterizerState` are baked into
`SDL_CreateGPUGraphicsPipeline` and there is no command to change either inside a
render pass. So a material that needs them is drawn through a second pipeline
rather than by setting a flag. `MaterialDef::needs_decal_pipeline()` selects it.

Because the bias **magnitude** is pipeline state too, `decal_pipeline_for()` keeps
one decal pipeline per distinct `depth_bias`, quantised to 1/16 over the panel's
`[-16, 16]` range and capped at 32 live pipelines. Otherwise the panel's slider
and a set file's `depth_bias` field would be stored, edited, serialised values
that changed no draw state.

**The sign of that bias is set by reverse-Z**, and both terms carry it. The depth
test is `GREATER` and the near plane is at depth 1, so "towards the viewer" is a
*larger* depth value. `GPURenderer::decal_depth_bias()` is the one place that
arithmetic lives. Vulkan computes `o = m * slopeFactor + r * constantFactor`, and
with `D32_FLOAT` the resolution term `r` is around `5e-10` while the slope term
`m` is around `4.5e-5` for a road surface seen near ground level — so a constant
that pulls one way beside a slope that pushes the other is not a partial fix, it
is worse than no bias at all.

Wireframe is deliberately exempt from the decal pipeline: in wireframe the user is
inspecting topology, and swapping half the draws onto a filled, blended pipeline
would hide the geometry they are looking at.

---

## 6. Replacing a procedural texture with an authored one

The generated textures exist so the editor is useful with **no assets present at
all**. They are meant to be replaced.

### From the Materials panel

1. Open **Materials** (View menu, or it opens itself the first time the library
   comes up).
2. Pick the slot, then the variant, in the left pane.
3. Under **Maps**, use the **Albedo**, **Normal** or **ORM** row's browse button.
   The row shows the source filename, or `procedural (not saved)` for a generated
   map, or `(uploading)` while its copy is still queued.
4. The change is live on the next frame.

Get the colour space right, because it is the mistake that looks merely "a bit
off" rather than broken:

| Map | Colour space | Why |
|---|---|---|
| Albedo | **sRGB** | Authored as colour; the hardware must linearise it before lighting |
| Normal | **linear** | A tangent vector, not a colour |
| ORM | **linear** | Three scalars, not a colour |

The panel and `load_map_from_file()` pick this automatically per map slot. If you
call `GPUTextureManager::load()` directly, pass `srgb = false` for normal and ORM.

KTX2 goes through libktx and keeps whatever compressed format and mip chain it was
authored with; PNG/JPG/TGA/BMP go through `stb_image`, are expanded to 8-bit RGBA,
and are given a generated chain. Prefer KTX2 with BC7 for albedo and BC5 for
normals — `GPUTextureManager` queries support for all three at `init()` and logs
what the device can sample.

### As a material set file

`MaterialLibrary::save_to_file()` and `load_from_file()` round-trip the whole
library as JSON. Texture paths are stored **relative to the set file**, so a set
and its textures move together.

```json
{
  "version": 1,
  "materials": [
    {
      "slot": "Asphalt",
      "variant": 0,
      "name": "Asphalt",
      "base_color": [1.0, 1.0, 1.0, 1.0],
      "metallic": 0.0,
      "roughness": 0.82,
      "ao": 1.0,
      "emissive": 0.0,
      "uv_scale": [1.0, 1.0],
      "albedo": "textures/asphalt_albedo.ktx2",
      "normal": "textures/asphalt_normal.ktx2",
      "orm":    "textures/asphalt_orm.ktx2",
      "sampler": "RepeatAniso",
      "alpha_blend": false,
      "depth_bias": 0.0
    }
  ]
}
```

A **generated** map has no source path, so `save_to_file()` omits its field and
the map is regenerated on load. That is the documented round-trip behaviour, and
the panel says `procedural (not saved)` so it is visible before the save rather
than after the reload. An unknown `"slot"`, an unparseable entry or a texture that
fails to load skips that field or entry and is logged; the rest of the set still
loads.

### Authoring notes

* **Tile size.** Author to the slot's `tile_*_metres` from §4. If you cannot,
  author to whatever size is natural and correct it with `uv_scale` — that is what
  the field is for. Never give `Curb` or `Markings` anything but `{1, 1}`.
* **A real ORM map.** Occlusion in red, roughness in green, metallic in blue,
  **linear**. It multiplies the material's scalars, so authoring a full ORM map
  usually means setting `roughness = 1.0` and `ao = 1.0` on the material and
  letting the texture carry the variation.
* **Metallic.** It works now (see §2). It is still zero for every road surface;
  it is there for parapets and railings.
* **Tileability.** Check the seam. `stratum::is_tileable()` in
  `procedural_texture.hpp` is the same measure the ProceduralTexture suite uses:
  it compares the step across the wrap against the steps the texture takes
  everywhere else, rather than comparing opposite edge columns, which is a test
  every correct tiling texture fails.
* **Regenerating the previews.** `build/material_preview/*.png` are dumps of the
  shipping generators at 512 (1024 for the atlas, and a magnified 4x4 of the 1x1
  built-in ORM). They are build output, not fixtures.

---

## 7. Tests

| Suite | Device? | Covers |
|---|---|---|
| `MaterialUniforms` | no | The C++ block against the offsets `glslc` computed in the shipped `.spv`, and the set/binding numbers |
| `MaterialLibrary` | no | Resolution, the fallback chain, the slot table, JSON round-trip |
| `MaterialBinding` | no | `material_bind_mode()`, the reverse-Z decal bias factors, the decal pipeline cache key |
| `ProceduralTexture` | no | Tileability, determinism, layout, the atlas, and the feature-period bounds at the smallest declared sizes |
| `TextureUpload` | **yes** | Readiness contingent on the submit, the built-in texels, what attaching a map does to `base_color` |
| `PbrShader` | **yes** | One pixel through the real `mesh_pbr` pipeline: degenerate tangent frames, and metallic/roughness/AO reaching the shader through the unit ORM |

The two device suites skip themselves with a line on stderr when there is no
usable backend, because failing a build for the absence of a GPU is worse than not
running. Run one with `./build/bin/stratum_gpu_tests <Suite>`.

`PbrShader` builds its pipeline from the checked-in
`assets/shaders/mesh_pbr.{vert,frag}.spv` — the exact bytes `GPURenderer` hands
`SDL_CreateGPUShader` — with the renderer's own five-attribute vertex layout,
uniform slots and sampler set, and reads back a 1x1 target. Its assertions are
relational (draw twice, change one input, assert the difference) so that tone
mapping and the GGX terms are not frozen by a test.
