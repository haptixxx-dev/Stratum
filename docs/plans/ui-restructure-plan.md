# ImGui editor restructure, theme and cleanup: implementation plan

**Status:** Proposed
**Date:** 2026-09-26
**Decider:** Sarah
**Follows:** `docs/plans/ui-toolkit-decision.md` (Option A chosen on 2026-09-26)
**Line references:** `master` at `be2b4a8`

---

## Goals

1. `src/editor/editor.cpp` (4,176 lines) becomes a façade of ~300 lines over an
   ImGui-free model, an ImGui-free import pipeline, a GPU residency layer, and
   one file per panel. No panel file contains business logic.
2. The import → roads → index → meshes → carve pipeline runs and is tested
   without a window or a GPU, in `stratum_tests`, against the fixtures in
   `tests/data/*.osm`. The same pipeline is what the planned headless CLI (J3)
   will call.
3. The editor looks like a product: real fonts, an icon font, a coherent
   palette and spacing, a property-grid layout in every panel, a status bar,
   correct DPI scaling on all three platforms, and no fake data anywhere.
4. Dead weight and stale documentation are gone: unused ImGui add-ons are
   unlinked, `docs/architecture.md` and `README.md` describe the code that
   exists, and `docs/agents/renderer-and-editor.md` gains its missing
   `src/editor` section.
5. **Zero behaviour change** in what the app imports, builds and renders.
   Every phase is verified against the same baseline numbers.

## Non-goals (sequenced after, not dropped)

- Wiring `src/scene` (command stack, layers, attributes, selection) into the
  panels. That is roadmap K1/K2 and the M7 prerequisite. This plan builds the
  seams it needs and leaves honest empty states where it will go.
- Viewport picking, gizmos, viewport handles (K5).
- Any change of UI toolkit.

---

## Target layout

```
src/app/                         NEW, compiled into stratum_core (engine-agnostic)
  import_pipeline.hpp/.cpp       the five-stage async state machine, road rebuild,
                                 carve, export; Listener interface; LogSink
  render_settings.hpp/.cpp       lighting / sky / fog / shadow settings as data,
                                 JSON round-trip (moved here only if it needs no
                                 renderer types; otherwise stays in src/editor)

src/editor/
  editor.hpp/.cpp                façade kept for Application: init/update/render/
                                 render_3d/publish_camera/shutdown; owns the parts
  editor_model.hpp/.cpp          ImGui-free session state: ImportOptions,
                                 ExportOptions, RuleAuthoring, MaterialPick,
                                 view toggles, console log
  scene_gpu_sync.hpp/.cpp        mesh ownership, upload/release/evict, road LOD
                                 residency, terrain chunk residency
  viewport_renderer.hpp/.cpp     render_3d(): HiDPI rect, traversal, draw calls, im3d
  viewport_overlay.hpp/.cpp      im3d overlay helpers and the attribute overlay
  camera_framing.hpp/.cpp        auto-frame the camera from quadtree focus
  file_dialogs.hpp/.cpp          SDL native dialog wrappers and their polling
  ui/theme.hpp/.cpp              fonts, palette, style, DPI scale
  ui/icons.hpp                   icon codepoints
  ui/widgets.hpp/.cpp            property rows, section headers, help markers,
                                 status chips, empty states
  ui/layout.hpp/.cpp             dockspace, default layout, reset, ini path
  panels/
    viewport_panel.cpp           camera input gate, overlay toggles, gizmo bar
    scene_panel.cpp              real layer list with visibility and counts
    inspector_panel.cpp          honest empty state until K1
    console_panel.cpp            bound to the real log
    import_panel.cpp             was "OSM": options, progress, stats, clear
    render_settings_panel.cpp
    memory_panel.cpp
    material_panel.cpp           existing, re-laid out
    rule_panel.cpp               existing, TextEditor-backed
    procgen_panel.cpp            existing, re-laid out
    status_bar.cpp
```

`Application` keeps calling the same nine `Editor` entry points it calls today
(`src/core/application.cpp:96-99,150-187,209`), so `src/core` does not change
beyond the theme call in `init()`.

---

## Phases

Each phase is one PR. Each PR description carries the verification block from
§Verification with the numbers filled in.

### Phase 0: baseline capture (0.5 day)

Before touching anything, record what the app does today so every later phase
can prove it did not change it.

1. Build `release` and `tests`; `ctest --preset tests` green (75 suites + GPU).
2. Import `~/Downloads/lucan.osm` through the GUI and copy the quadtree summary
   that `editor.cpp:3529-3534` logs (leaf count, roads, buildings, areas, max
   depth) plus the road junction stats the OSM panel shows. These are the
   **golden numbers**.
3. `timeout 6 ./build/release/bin/stratum 2>&1 | grep -c VUID` must be 0.
4. Screenshots of every panel with the crop-and-delete recipe from the
   verification memory (never leave a root capture on disk). Keep crops under
   `docs/images/ui-before/` for the before/after comparison.
5. `wc -l src/editor/*.cpp src/editor/panels/*.cpp` recorded.

### Phase 1: mechanical split (1 to 1.5 days)

No logic changes. Functions move to new files; `Editor` keeps every member.
The only edits inside moved bodies are `this->` qualifications and includes.

| Move | From (`editor.cpp`) | To |
|---|---|---|
| `setup_dockspace`, `draw_menu_bar` (menu items only) | 231-446 | `ui/layout.cpp`, `panels/menu_bar.cpp` |
| window drag block inside `draw_menu_bar`, `handle_window_resize`, `toggle_fullscreen` | 293-446 (drag part), 2089-2252 | `window_chrome.cpp` (deleted in Phase 4) |
| `draw_viewport` | 448-614 | `panels/viewport_panel.cpp` |
| `draw_scene_hierarchy`, `draw_properties`, `draw_console` | 616-754 | `panels/scene_panel.cpp`, `inspector_panel.cpp`, `console_panel.cpp` |
| `draw_chunk_lod_stats`, `draw_attribute_mode_selector` | 756-855, 1033-1127 | `panels/viewport_panel.cpp` |
| overlay helpers, `attribute_*_tint`, `draw_attribute_overlay` | 866-1031, 1129-1273 | `viewport_overlay.cpp` |
| `draw_osm_panel` | 1275-1775 | `panels/import_panel.cpp` |
| `draw_render_settings` | 1781-2087 | `panels/render_settings_panel.cpp` |
| `open_*_dialog`, `poll_file_dialog`, `poll_export_dir_dialog` | 2254-2552 | `file_dialogs.cpp` |
| `begin_road_export`, `poll_road_export` | 2554-2665 | `import_pipeline.cpp` (editor-local until Phase 3) |
| `draw_memory_panel` | 2680-2926 | `panels/memory_panel.cpp` |
| fingerprint helpers through `finish_osm_import` | 2932-3493 | `import_pipeline.cpp` |
| `begin_mesh_rebuild` | 3495-3629 | `import_pipeline.cpp` (quadtree part) and `camera_framing.cpp` (lines 3549-3595) |
| `upload_tracked_mesh` through `record_road_lod_residency` | 3647-3908 | `scene_gpu_sync.cpp` |
| `render_3d` | 3910-4174 | `viewport_renderer.cpp` |

Also: `s_viewport_rect` (`editor.cpp:30`) becomes a member of the viewport
renderer, set by the viewport panel through a method, so the two files that
share it name the dependency.

**Acceptance:** golden numbers identical; VUID count 0; `editor.cpp` under
600 lines; every new file compiles with `-Wall -Wextra` clean.

### Phase 2: state promotion and model extraction (1.5 to 2 days)

Turn hidden state into data and pull logic out of widget bodies.

1. **`RenderSettings`**: the 34 function-static locals at
   `editor.cpp:1817-1996` (sun, sky, fog, shadows) become one struct with
   defaults, `to_json`/`from_json` (nlohmann is already linked) and a
   `push_to(GPURenderer&)` that replaces the three `*_pushed` flags. Saved to
   the pref directory on change, loaded at start. This is the first time
   lighting can survive a restart.
2. **Import panel statics** (`editor.cpp:1424-1425`) and the mock statics in
   Properties (`:664-694`) are deleted with their panels' rewrite in Phase 5;
   the search buffer (`:620`) moves to the model.
3. **`clear_imported_data()`** extracted from the Clear Data button body
   (`:1729-1771`).
4. **Rule source I/O**: `load_rule_source(path)` / `save_rule_source(path)`
   extracted from `poll_file_dialog()` (`:2454-2489`).
5. **One `ExportOptions`**: the export UI duplicated in the memory panel
   (`:2680-2926`) and the import panel share one struct and one panel section.
6. **`ViewportInput`** struct (size, focused, hovered, mouse delta, wheel, dt)
   filled by the viewport panel and consumed by `Camera` and im3d, so
   `camera.cpp` and `viewport_overlay.cpp` stop calling `ImGui::` directly.
7. **Console log sink**: `m_console_buffer` (`ImGuiTextBuffer`) becomes a
   ring of `LogLine {level, time, text}` in the model with an spdlog sink
   feeding it, so the console shows real logs, not five hard-coded lines.
8. `Editor` members regroup into `EditorModel` (application state, ~49
   members) and view prefs (~37); derived caches stay with the component that
   owns them (`SceneGpuSync`, `ImportPipeline`).

**Acceptance:** golden numbers identical; a `RenderSettings` JSON round-trip
test in the non-device list of `stratum_gpu_tests` (or `stratum_tests` if the
struct needs no renderer type); VUID 0; the console panel shows the import log.

### Phase 3: pipeline extraction and tests (2 to 3 days)

The import state machine (`editor.cpp:3133-3493`, 728 ImGui-free lines) gets
an interface and moves into `stratum_core`.

1. `ImportPipeline` owns: parse future, road build future, carve index future,
   export future, stage enum, progress, the terrain fingerprint checks, the
   deferred-rebuild flags. It exposes `begin_import(path, ImportOptions)`,
   `tick()` (what the four `poll_*` calls do today), `request_road_rebuild()`,
   `begin_export(ExportOptions)`, `cancel()` where a stage can be cancelled,
   and `state()` for the panel to draw from.
2. A `Listener` with `on_stage(stage, fraction, message)`, `on_log(line)`,
   `on_before_quadtree_reset()` (the editor releases GPU leaves here, today at
   `editor.cpp:3509-3512`), `on_quadtree_ready()`, `on_carve_installed()`,
   `on_export_done(result)`. The editor's `SceneGpuSync` and `CameraFraming`
   subscribe; nothing in the pipeline knows about GPUs or cameras.
3. Move to `src/app/import_pipeline.*`, add `src/app` to `stratum_core` in
   `CMakeLists.txt`, and update the core directory list in `CLAUDE.md`.
   Everything it touches (`OSMParser`, `RoadNetworkBuilder`, `QuadTree`,
   `CarveInput`, `TerrainTileManager`, `export_road_network`) is already core.
4. New suite `tests/app/test_import_pipeline.cpp` in `stratum_tests`, using
   `tests/data/four_way.osm` and `dual_carriageway.osm`: stage order is
   Parse → Roads → Index → Meshes → Carve → Done; `tick()` never blocks; a
   `request_road_rebuild()` during import is honoured after Done (the
   `m_road_rebuild_owed` path); `cancel()` during Parse leaves the previous
   quadtree intact; leaf and road counts match the fixture.

**Acceptance:** golden numbers identical from the GUI; the new suite green;
`ctest --preset tests` green; the Lucan road stats reproduced by the suite's
harness path rather than the throwaway `g++` harness in the memory notes.

### Phase 4: window chrome and layout (1 day)

1. **Native decorations by default.** Drop `config.borderless = true`
   (`application.cpp:25`), delete the drag and edge-resize state machines
   (~250 lines: `editor.cpp:293-446` drag part, `2108-2252`) and the
   `title_bar_height` plumbing. Fullscreen stays on F11 and in the View menu.
   Rationale: Windows snap layouts, macOS traffic lights and Wayland
   server-side decorations come back for free, and the four seam bugs in the
   history all lived here. If Sarah wants the custom bar back, it returns as
   `window_chrome.cpp` on top of `SDL_SetWindowHitTest`, not on top of manual
   `SDL_SetWindowPosition` math.
2. **Layout v2** in `ui/layout.cpp`: left = Scene / Import tabs; centre =
   Viewport; right = Inspector / Render / Materials / Procgen / Memory tabs;
   bottom = Console / Rule Editor tabs; a status bar under everything. The
   `DockBuilder` block (`editor.cpp:258-282`) becomes `apply_default_layout()`
   callable from View → Reset Layout.
3. `io.IniFilename` points into `SDL_GetPrefPath(org, "Stratum")` so the
   layout stops landing in whatever the current directory is.
4. **Status bar**: import stage and progress, frame time, resident GPU MB,
   shader mode, last log line. All read from the model; nothing computed here.

**Acceptance:** app opens with the v2 layout on a fresh pref dir; Reset Layout
restores it; F11 works; VUID 0; screenshots.

### Phase 5: theme and panel polish (2 to 3 days)

1. **Fonts** under `assets/fonts/` with their licence files:
   Inter (UI, SIL OFL 1.1), JetBrains Mono (console and rule editor, OFL),
   Lucide icon font (ISC). Loaded in `ui/theme.cpp` via `AddFontFromFileTTF`
   with the icon font merged (`MergeMode`), sizes 15 UI / 14 mono. Generate
   `ui/icons.hpp` codepoints for the ~25 icons the panels use. The missing
   glyph bug (`89a1097`) cannot recur: every symbol comes from the icon font.
2. **Palette and style** in `ui/theme.cpp`: a small token set (four background
   levels, text, muted text, accent, ok/warn/error), applied to `ImGuiCol_*`
   in one place; rounding 4/3, frame padding 8x5, item spacing 8x4, subtle
   1px borders, tab bar without the default gradient, scrollbars 10px. One
   dark theme now; the token indirection makes a light theme a table later.
3. **DPI**: scale `style.FontSizeBase` and `ScaleAllSizes` by
   `SDL_GetWindowDisplayScale()` at init and on
   `SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED`. The current comment at
   `application.cpp:72-76` is right for macOS and Wayland fractional
   scaling, where the framebuffer scale carries it, and wrong on Windows,
   where SDL reports the scale separately and the UI is currently 1x at 150%.
   Verify on the Windows CI build by hand once.
4. **Widget helpers** in `ui/widgets.cpp`: `property_row(label, widget)` on a
   two-column table so labels align and controls fill the width;
   `section(title)` header; `help_marker(text)`; `empty_state(icon, text)`;
   `status_chip(level, text)`. Every panel is converted to these, which is
   most of what turns "sliders in a column" into a property grid.
5. **Honest panels.** Scene panel lists what exists: Terrain, Water,
   Buildings, Roads, Areas with the existing `m_render_*` visibility toggles,
   counts from the quadtree, and the chunk grid toggle. Inspector shows an
   empty state ("Select something in the viewport") until K1 lands. Console
   is bound to the real log with level filters and a copy button. Import
   panel renamed from "OSM" and split into Source / Options / Progress /
   Statistics sections. Gizmo buttons that do nothing are removed until K5.
6. **Menus**: dead File/Edit items are removed rather than left as stubs;
   Undo/Redo return with K1.

**Acceptance:** screenshots of every panel under `docs/images/ui-after/`;
golden numbers identical; VUID 0; nothing in the UI displays data that does
not exist.

### Phase 6: rule editor on ImGuiColorTextEdit (1 to 1.5 days)

1. Build `external/ImGuiColorTextEdit/TextEditor.cpp` as a STATIC target
   (today it is an INTERFACE target with nothing compiled,
   `external/CMakeLists.txt:79-80`) and link it into `stratum_editor_lib`.
   The vendored fork (`ca2f9f1`, 2025-11-24) exposes `SetLanguageDefinition`,
   `SetErrorMarkers` and `Render` (`TextEditor.h:188-197`).
2. `LanguageDefinition` for the rule language generated from
   `token_kind_name()` and the operation registry, not hand-copied, so a new
   operation highlights without an editor change.
3. Diagnostics map to `ErrorMarkers` by line (`Diagnostic` already carries
   line and column, `src/procgen/rules/lexer.hpp:33-48`); the caret rendering
   stays in the diagnostics list below the editor.
4. Mono font, line numbers, Ctrl+Enter to run, dirty marker in the tab
   title, Ctrl+S to save through `save_rule_source`.

**Acceptance:** the default rule source parses with no diagnostics and the
generated mass appears (the D10 check from 2026-09-18); an injected error
shows a marker on the right line; screenshots.

### Phase 7: dead weight and documentation (0.5 to 1 day)

1. Remove `ImGuizmo` and `imnodes` from `target_link_libraries`
   (`CMakeLists.txt:333,335`) and their targets from `external/CMakeLists.txt`.
   Remove the submodules unless Sarah wants them kept for K5; re-adding one
   is a single command.
2. `docs/architecture.md`: the ImGui backend is `imgui_impl_sdlgpu3`, not
   SDLRenderer3; the scene model is handle-based, not an EnTT ECS; the editor
   class diagram and layout sections rewritten against the new files.
3. `README.md` dependencies list: drop the three unused add-ons, add the
   fonts and their licences.
4. `docs/agents/renderer-and-editor.md`: write the `src/editor` and
   `src/editor/panels` sections (currently "still to be written").
5. `CLAUDE.md`: the source layout entry for `editor/` and the new `app/`
   directory in the core list.

### Phase 8: CI follow-up (0.5 day, optional but cheap)

Run `ctest` on the Windows and macOS jobs, not only Linux
(`.github/workflows/build.yml:112-244` build without testing today). The
pipeline suite from Phase 3 needs no GPU, so it runs everywhere; the GPU
suites self-skip with `skip_test()` where there is no device.

---

## Ordering constraints

- 0 → 1 → 2 → 3 in that order. Each lowers the risk of the next: the split
  makes the state visible, the promotion makes it movable, the extraction
  makes it testable.
- 4 and 5 need 1 (they edit panel files). 5 needs 4's layout. 6 needs 5's
  fonts. 7 can run any time after 1; its docs part is last.
- K1/K2 (scene wiring) starts after 3 and 5: it needs the model seam and the
  honest empty states.

## Estimate

| Phase | Days |
|---|---:|
| 0 baseline | 0.5 |
| 1 split | 1 to 1.5 |
| 2 model | 1.5 to 2 |
| 3 pipeline + tests | 2 to 3 |
| 4 chrome + layout | 1 |
| 5 theme + panels | 2 to 3 |
| 6 rule editor | 1 to 1.5 |
| 7 dead weight + docs | 0.5 to 1 |
| 8 CI | 0.5 |
| **Total** | **10 to 14 agent-assisted days** |

---

## Verification, every PR

```
ctest --preset tests                                  all green, count reported
timeout 6 ./build/release/bin/stratum 2>&1 | grep -c VUID   must print 0
lucan.osm import via GUI                              leaf/road/building/area/depth
                                                      and junction stats == Phase 0
wc -l src/editor/editor.cpp                           trending to < 300
screenshots (crop-and-delete recipe)                  attached for UI phases
cmake --build --preset release                        clean under -Wall -Wextra
```

Phase 3 replaces the manual Lucan step with the pipeline suite for the
fixture files; the Lucan GUI check stays as the end-to-end proof for UI phases.

## Risks

- **Silent behaviour drift during extraction.** Mitigated by the golden
  numbers and VUID check on every PR, and by Phase 1 moving code without
  editing it.
- **ImGui 1.92 font system.** Dynamic font loading changed in 1.92; merged
  icon fonts and `FontSizeBase` scaling need one careful pass on a Retina or
  fractional-scale display. Pinned submodule, so no upstream drift mid-work.
- **Windows DPI is unverified.** CI never runs the GUI; one manual check on a
  150% Windows display is required for Phase 5.
- **ImGuiColorTextEdit is a fork-heavy project.** Check the vendored header's
  API before writing against it; the three calls this plan needs exist at
  `TextEditor.h:188-197`.
- **`std::async` teardown order.** The pipeline's futures must stay the last
  members of their structs (`editor.hpp:392-399, 611-616`); the extraction
  keeps that invariant and the new suite's teardown exercises it.
- **Merge conflicts.** No live branch changes `src/editor` beyond ones already
  merged (checked 2026-09-26), but Phase 1 rewrites every line's location, so
  land it before any other editor work starts.

## Decisions needed

1. **Window chrome:** native decorations (recommended) or keep the custom
   title bar on `SDL_SetWindowHitTest`?
2. **Fonts:** Inter + JetBrains Mono + Lucide (OFL/OFL/ISC), or different
   choices? Material Symbols (Apache-2.0) is the alternative icon set if a
   licence match with the project matters more than file size.
3. **ImGuizmo / imnodes:** remove the submodules now, or keep unlinked until
   K5 decides on a gizmo library?
4. **`src/app` in `stratum_core`:** agree to the new directory and the
   `CLAUDE.md` update, or keep the pipeline in `stratum_editor_lib` and test
   it from `stratum_gpu_tests` without a device?
5. **Pref path:** the organisation string for `SDL_GetPrefPath`
   ("Haptixxx"?). It fixes where `imgui.ini` and `render_settings.json` live
   for every user from now on.
