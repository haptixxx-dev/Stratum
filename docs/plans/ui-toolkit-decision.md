# UI toolkit decision: stay on Dear ImGui, or migrate to Qt or a web UI?

**Status:** Proposed
**Date:** 2026-09-26
**Decider:** Sarah
**Basis:** a read of every file in `src/editor`, `src/core`, the ImGui parts of
`src/renderer`, the planning docs, CI, and the git history. Line references are
to `master` at `be2b4a8`. The raw audit reports this was distilled from are
listed in the appendix.

---

## Verdict in six lines

1. **The mess is architectural, not ImGui's.** `editor.cpp` is a 4,176-line
   monolith in which 46% of the lines mix widget calls with business logic and
   only 10% are pure widget code. No toolkit fixes that; extracting an
   ImGui-free `EditorModel` / controller does, and it is a prerequisite for
   every option below, including staying put.
2. **The editor is smaller and less finished than it looks.** Of ten panels,
   three (Scene Hierarchy, Properties, Console) are hard-coded mock-ups, the
   viewport has no picking, Undo/Redo are empty stubs, and nothing in
   `src/editor` includes anything from `src/scene`. The M2 scene model exists
   only in core. The UI that would actually be "migrated" is seven forms
   panels plus a viewport.
3. **Every migration option is gated by the viewport.** The 3D scene is drawn
   straight into the SDL_GPU swapchain underneath a transparent ImGui window.
   Qt and web UIs both need either a fragile foreign-window embed of SDL or a
   port of the renderer off SDL_GPU. The renderer port, not the widgets, is
   the dominant cost of leaving ImGui.
4. **A web UI is not viable while the renderer is SDL_GPU.** Embedded Chromium
   adds a 150 MB multi-process runtime and a CPU upload path for UI pixels;
   browser-first means rewriting the renderer in WebGPU and compiling core to
   WASM. Neither fits a months-long programme.
5. **Qt is viable but only via a QRhi renderer port**, costed at roughly 8 to
   12 agent-assisted weeks including the extraction, with the renderer port
   and the GPU test suite as the critical path. It buys mature docking, item
   models, a real code editor, native OS integration, accessibility, and DCC
   precedent (Houdini, Substance, Maya, Nuke, RenderDoc are Qt).
6. **Recommendation:** do the extraction now (1.5 to 2 agent-assisted weeks),
   wire the real scene model into Hierarchy/Properties as roadmap K1/K2 work,
   and re-decide the toolkit at a gate before M7. If cross-platform native
   feel and DCC-grade widgets are firm v1.0 requirements, Qt via QRhi is the
   only sound path and the cheapest time to pay it is between M6 and M7, before
   UI volume triples.

---

## 1. How the UI works today

### 1.1 Shape of the code

| File | LOC | `ImGui::` calls | Role |
|---|---:|---:|---|
| `src/editor/editor.cpp` | 4,176 | 569 | dockspace, menu bar, 8 panels, import/export pipeline, GPU bookkeeping, `render_3d()` |
| `src/editor/editor.hpp` | 1,222 | — | `Editor` class: ~145 member variables |
| `src/editor/panels/material_panel.cpp` | 843 | 120 | material library browser and PBR editor |
| `src/editor/panels/rule_panel.cpp` | 580 | 77 | rule-language text box, run, diagnostics |
| `src/editor/panels/procgen_panel.cpp` | 454 | 104 | terrain property sheet |
| `src/editor/im3d_impl.cpp` | 487 | 0 | im3d GPU backend (grid, axes, overlays) |
| `src/editor/camera.cpp` | 155 | 0 | camera, raw SDL polling |
| `src/core/application.cpp` | 228 | 8 | ImGui bootstrap, frame loop |
| `src/core/window.cpp` | 76 | 0 | SDL window, borderless |

878 ImGui call sites in total. The widget mix is forms: `Text`, `SliderFloat`,
`Checkbox`, `Button`, `Combo`, `TreeNodeEx`, `ColorEdit`, `ProgressBar`,
tables. No custom `ImDrawList` widgets of note, no drag-and-drop, one
`imgui_internal.h` include for `DockBuilder`.

### 1.2 Frame and compositing model

Per frame (`src/core/application.cpp`, `src/renderer/gpu_renderer.cpp`):
`SDL_PollEvent` → `ImGui_ImplSDL3_ProcessEvent` → `ImGui::NewFrame` →
`Editor::render()` (all widgets) → `ImGui::Render` → acquire swapchain →
im3d upload → shadow cascades → **pass 1**: 3D scene + im3d drawn directly
into the swapchain, viewport/scissor clipped to the panel rect → **pass 2**:
`load_op = LOAD`, no depth, `ImGui_ImplSDLGPU3_RenderDrawData` → submit.

There is no offscreen render target and no `ImGui::Image`. The "Viewport"
window is `NoBackground`; the 3D pixels show through from pass 1. The panel
rect is handed to the renderer through a file-static
`s_viewport_rect` (`editor.cpp:30`), multiplied by
`io.DisplayFramebufferScale` at `editor.cpp:3920-3939` (the HiDPI fix in
`be2b4a8`). Three drawing systems (mesh pass, im3d, ImGui) are ordered by call
sequence and depth-test flags rather than composited as layers.

Config flags: `DockingEnable`, keyboard and gamepad nav. Multi-viewport is
**off**; the app can only ever show one OS window. Default font only,
`FontSizeBase` pinned to 16. Dock layout is rebuilt by `DockBuilder` every run
(`editor.cpp:259-278`), so `imgui.ini` is effectively unused.

Input routing: no `WantCaptureMouse`/`WantCaptureKeyboard` anywhere. The camera
is gated purely on `IsWindowFocused`/`IsWindowHovered` of the Viewport panel
(`editor.cpp:453-454`) and then polls SDL directly.

The window is **borderless** (`application.cpp:25`). The title bar, window
drag and eight-edge resize are re-implemented in ImGui
(`editor.cpp:305-319`, `2103-2231`) with `SDL_SetWindowPosition`. This is
self-inflicted OS-chrome work and the source of commit `508db22`.

### 1.3 Panels: real versus mock

| Panel | Lines | Status |
|---|---:|---|
| Viewport | `editor.cpp:448-614` | real: camera, im3d overlays, attribute colouring; **no picking**, gizmo buttons are empty no-ops (`:596-609`) |
| OSM | `:1275-1775` | real, 501 lines; import options, five-stage progress bar, road stats, Clear Data |
| Render Settings | `:1781-2087` | real, 307 lines; lighting/sky/fog/shadow config held in **function-static locals** (`:1817-1995`), not members |
| GPU Memory | `:2680-2926` | real, 247 lines; budgets, pools, export job |
| Materials | `material_panel.cpp` | real; edits bypass the command stack |
| Rule Editor | `rule_panel.cpp` | real; plain `InputTextMultiline`, no highlighting, diagnostics as pre-formatted text |
| Procgen | `procgen_panel.cpp` | real; flat property sheet, synchronous generation |
| Scene Hierarchy | `:616-656` | **mock**: hard-coded tree ("Building_001", "Main Street") |
| Properties | `:658-701` | **mock**: static local floats, bound to nothing |
| Console | `:703-754` | **mock**: five hard-coded lines; `m_console_buffer` is appended at 11 sites and never read |

Menu bar: File/Edit items are stubs; Undo/Redo are `{}` (`editor.cpp:352-353`).

### 1.4 Coupling: UI versus logic inside `editor.cpp`

Classification of every function (audit B):

| Category | LOC | Share |
|---|---:|---:|
| (a) pure widget/draw code | 396 | 9.8% |
| (b) widget calls and business logic in the same bodies | 1,859 | 45.9% |
| (c) pure logic, zero ImGui calls | 728 | 18.0% |
| (d) glue to renderer/camera/input, no ImGui | 1,064 | 26.3% |

Logic that exists only inside UI code today:

- The whole import → road build → spatial index → mesh build → terrain carve
  pipeline is an `Editor`-only state machine (`editor.cpp:3133-3629`, ~730
  LOC). No core API says "import this file end to end".
- Rule file load/save is inline `ifstream`/`ofstream` in `poll_file_dialog()`
  (`:2454-2489`).
- Lighting/sky/fog settings are function-static locals in a draw function; they
  are not state and cannot be saved.
- "Clear Data" is ~40 lines of scene teardown inside an `if (ImGui::Button)`
  (`:1729-1771`).
- GPU mesh ownership and eviction (`m_mesh_owners` plus ~300 LOC) depends on
  `GPURenderer`, not ImGui, but lives in the editor.

Member state: ~145 variables; ~37 view-only, ~49 application state,
~59 cache/derived.

Dependency direction is clean: nothing in `src/scene`, `src/osm`,
`src/procgen` or `src/renderer` includes an editor header.

### 1.5 Async model

Four `std::async` sites (OSM parse, road build, carve index, road export), all
polled once per frame from `Editor::render()` with `wait_for(0s)`; results are
applied on the main thread. Native file dialogs use `SDL_ShowOpenFileDialog`
with a mutex-guarded result polled next frame. No worker touches ImGui, the
quadtree, the camera or GPU resources (stated invariant at
`editor.hpp:368-372`). **This subsystem is toolkit-neutral as written**; only
the per-tick driver would change.

### 1.6 The scene model is not wired in

`src/scene` (command stack, layer tree, attributes, selection, document
save/load) is complete in `stratum_core` with tests. `src/editor` references
none of it. Every mutation in the editor is a direct member write. This is the
single most important fact for the decision: the panels where a retained-mode
toolkit would show its strengths (hierarchy, inspector, layer tree, attribute
tables) have not been built yet, on any toolkit.

The scene model already exposes a monotonic `CommandStack::revision()`
(`src/scene/command.hpp:229`) and `Georeference::generation()`, which is enough
for a retained UI to poll for change without an observer system.

### 1.7 Vendored ImGui ecosystem: what is actually used

| Library | Built | Linked | Included in `src/` |
|---|---|---|---|
| imgui (docking branch, 1.92.6 WIP, pinned) | yes | yes | yes |
| im3d | yes | yes | yes (`im3d_impl`, viewport overlays) |
| ImGuizmo | yes | yes | **no** |
| imnodes | yes | yes | **no** |
| ImGuiColorTextEdit | interface only | no | **no** |
| ImGuiFileDialog | no | no | no (replaced by SDL native dialogs in `a229d70`) |

Three of the five ImGui add-ons are dead weight. The docs
(`docs/architecture.md`, `README.md`) describe them as integrated; they are
not. `docs/architecture.md` also still describes an `SDLRenderer3` ImGui
backend and an EnTT ECS scene; both are stale.

### 1.8 CI, tests, packaging

- CI (`.github/workflows/build.yml`): tests run on Linux/GCC only, with
  `SDL_VIDEODRIVER=offscreen`. Windows (MSVC) and macOS (Clang) jobs build the
  release preset and run nothing. No job ever opens a window.
- **Zero tests touch `src/editor`, `src/core`, or ImGui.** `stratum_gpu_tests`
  links `stratum_editor_lib` for the renderer, obtains an SDL_GPU device
  without a window, and exercises the renderer suites against SDL_GPU. A
  renderer port off SDL_GPU takes those suites with it.
- Release artefacts are raw `tar.gz`/`zip` of `build/<preset>/bin/*`. No
  CPack, no installer, no bundle. Assets resolve via `SDL_GetBasePath()`.
- Dependency policy is explicit: everything except zlib/bzip2/expat is a
  vendored submodule (`external/CMakeLists.txt:113-121`). No `#ifdef` on
  platform macros exists in `src/`.
- Python bindings (`STRATUM_ENABLE_PYTHON`) bind `stratum_core` only and are
  marked non-functional.

### 1.9 History: growth and ImGui-specific pain

132 commits, 2026-01-05 to 2026-09-22. 52 touch `src/editor` or `src/core`.
`editor.cpp` was flat at ~1,500 LOC from January to 5 August, then grew to
4,176 by 22 September. 13 of the last 15 editor commits also changed core or
renderer files.

Commits that exist only because of the ImGui ↔ SDL_GPU seam:

| Commit | Problem | Size |
|---|---|---|
| `be2b4a8` | viewport rect in logical points fed to a pixel-space scissor on HiDPI | +68 |
| `93ddbc8` | ImGui's SDL_GPU backend emitted validation errors inside the depth-attached pass; split into its own pass | small |
| `89a1097` | default font lacks U+26F6/U+2750; missing-glyph boxes on the fullscreen button | +6 |
| `508db22` | hand-rolled window drag/resize for the borderless window | medium |

All are fixed, all are small, and none recurred. They are seam bugs, not
evidence of instability in ImGui itself.

### 1.10 What the roadmap will ask of the UI

From `docs/plans/milestones.md` and `cityengine_parity.md`: 23 planned UI
features. The UI-dense milestones are **M7** (street draw tool, incremental
re-solve, and K1 to K5: inspector, layer panel, asset and rule browsers,
background regeneration with cancel, viewport handles), **M6** (asset library
browser with thumbnails) and **M9** (report dashboards with charts, terrain
brush). Widget kinds needed that do not exist today: live tree view with
drag/visibility/lock, property grid with multi-edit and attribute sources, a
code editor with error markers, a thumbnail grid browser, charts and tables,
gizmos and viewport handles, an optional node graph. Target platforms are
Linux, Windows and macOS; the headless CLI (J3) is planned on core only.

---

## 2. Diagnosis: what "messy" actually is

| Symptom | Fixed by changing toolkit? | Fixed by restructuring? |
|---|---|---|
| 4,176-line monolith with 10 panels and the import pipeline in one file | no | yes: split by panel, move pipeline to a controller |
| business logic inside `if (ImGui::Button)` blocks | no | yes |
| lighting config as function statics | no | yes: promote to a settings struct |
| mock panels and stub menus | no | no: this is unbuilt roadmap work (K1/K2) |
| scene model not wired; no undo in the UI | no | no: roadmap work, prerequisite for M7 |
| borderless window with ImGui-drawn chrome | yes, Qt gives native decorations | yes, `SDL_SetWindowHitTest` or native decorations |
| plain text box for the rule language | partly (Qt has `QPlainTextEdit`/`QSyntaxHighlighter`) | partly (ImGuiColorTextEdit is already vendored) |
| unused vendored add-ons, stale docs | no | yes: delete and rewrite the docs |
| "tool-like" look | partly | partly: fonts, icon font, theme |
| no UI tests on any layer | no | yes: a toolkit-free controller is testable in `stratum_tests` |

The extraction is the common factor. Whatever the toolkit, it must be done
first, and once done it is also what makes a later toolkit swap cheap: the
view layer would be roughly 1,100 to 1,300 LOC of thin widget wiring instead
of 4,176 lines.

---

## 3. The crux every option must answer: the viewport

The renderer (`gpu_renderer.cpp`, 2,927 LOC plus a 1,555-line header,
lighting v2 with cascaded shadows, baked AO, MSAA, a documented list of five
fixed regressions) is written against SDL_GPU, and SDL_GPU creates its
swapchain from an `SDL_Window`. Options for putting that image inside a
non-ImGui UI:

1. **Foreign-window embed.** SDL3 can wrap an existing native handle
   (`SDL_PROP_WINDOW_CREATE_WIN32_HWND_POINTER`, `..._X11_WINDOW_NUMBER`,
   `..._COCOA_VIEW_POINTER`, `..._WAYLAND_WL_SURFACE_POINTER`). Claiming such a
   window for SDL_GPU is not documented as supported and I found no working
   precedent. On Wayland the surface needs a custom role, SDL then handles
   "input and rendering" but no window state, and the host must forward size
   changes by hand. Two event loops (Qt's and SDL's) contend for the same
   window. Rate this as a spike with an unknown outcome, per platform.
2. **CPU readback.** Render offscreen with SDL_GPU, read back, upload as a
   `QImage` or web canvas. Simple and portable, but a full-frame copy every
   frame; unusable for a large city at interactive rates. Prototype only.
3. **GPU interop.** SDL_GPU exposes no public native texture handle, so there
   is no Vulkan external-memory path today.
4. **Port the renderer off SDL_GPU** to the host's GPU layer. For Qt that is
   QRhi (public since 6.6, `QRhiWidget` since 6.7), which abstracts Vulkan,
   Metal and D3D12 like SDL_GPU does and takes GLSL through `qsb`. This is the
   only robust path and it is a rewrite of the renderer, the ImGui-free im3d
   backend, and the renderer's GPU test suite.

For a web UI the same four apply, and options 1 and 4 are worse: a browser
cannot host a native Vulkan surface, and a WebGPU renderer means rewriting the
renderer in WGSL and compiling `stratum_core` (libosmium, draco, Clipper2,
meshoptimizer) to WASM. SDL_GPU has no upstream WebGPU backend that I could
verify.

---

## 4. Options

Cost units are **agent-assisted engineer-weeks (AW)**: one person directing
frontier coding models, at the pace this project has actually sustained.
Ranges are honest about unknowns, and ordering constraints matter more than
the totals.

### Option A: stay on Dear ImGui, restructure and polish

**Work**

| Item | AW | Notes |
|---|---:|---|
| Phase 0 extraction (see §6) | 1.5 to 2 | mandatory for all options |
| split `editor.cpp` into `panels/*.cpp`, one per panel | included | |
| promote lighting/sky/fog statics to a `RenderSettings` struct with save/load | included | |
| replace borderless custom chrome with native decorations or `SDL_SetWindowHitTest` | 0.2 | deletes ~150 LOC |
| theme: font (e.g. Inter), icon font, spacing, colours | 0.5 | most of the visual "mess" is default-ImGui styling |
| wire ImGuiColorTextEdit for the rule editor (already vendored) | 0.5 | highlighting, line numbers, error markers |
| delete ImGuizmo/imnodes if not adopted; fix stale docs | 0.2 | |
| **Total** | **3 to 3.5** | |

**Pros:** no renderer change, no CI change, all three OSes already build in
CI; immediate-mode is the lowest-ceremony fit for a single developer plus
agents; the vendored docking branch is pinned, so upstream churn cannot break
a build; ImGui is the incumbent for in-house game tooling and the whole
existing render path (im3d, viewport punch-through, HiDPI) keeps working.

**Cons:** ImGui docking is still a never-merged WIP branch whose author has
said he wants to rewrite it; no accessibility, no i18n, no native menus or
decorations; text editing, large tables and thumbnail browsers need manual
work (`ListClipper`, custom widgets); the look stays "tool" rather than
"product" unless deliberately themed; no established idiom for multi-window
without enabling multi-viewport, which brings its own bugs.

**Breaks:** nothing.

### Option B: Qt 6

Two variants. B1 is listed to be rejected explicitly.

**B1: Qt shell, SDL_GPU viewport via foreign-window embed.** Spike-grade.
Platform-specific behaviour, Wayland custom-role surface, two event loops on
one window, no documented SDL_GPU support. Not a plan; at most a two-day
curiosity spike.

**B2: Qt shell, renderer ported to QRhi, SDL removed.**

| Item | AW | Notes |
|---|---:|---|
| Phase 0 extraction | 1.5 to 2 | |
| renderer port SDL_GPU → QRhi (`QRhiWidget`) | 3 to 5 | 4,500 LOC of renderer, shaders through `qsb`, lighting v2 regressions re-risked, im3d backend rewritten |
| GPU test suite port | 1 to 2 | current suites obtain an SDL_GPU device offscreen; QRhi has an offscreen path (`QRhi::create` without a window on Vulkan) but every test's setup changes |
| view layer: ~1,200 LOC of ImGui wiring → Qt Widgets, docking via Qt-ADS (LGPL-2.1), models for hierarchy/inspector | 1.5 to 2 | Hierarchy/Properties are built fresh either way |
| camera/input adapter, file dialogs, window chrome → Qt | 0.5 | replaces SDL entirely |
| build and CI: `AUTOMOC`, Qt via `aqtinstall` in CI (Qt from source as a submodule is impractical), `windeployqt`/`macdeployqt`/AppImage or `linuxdeploy` | 1 | first exception to the vendored-submodule policy |
| LGPLv3 compliance: dynamic linking, ship relink information; avoid GPL-only modules | 0.2 | Qt Charts and Qt Data Visualization are GPL-only for open-source users; check Qt Graphs before relying on it for M9 dashboards |
| **Total** | **8.5 to 12.5** | critical path is the renderer port |

**Pros:** mature docking, `QAbstractItemModel` for large trees and tables
(M7 layer panel, attribute tables), `QPlainTextEdit` + `QSyntaxHighlighter`
for the rule editor, QtNodes (BSD) for the optional node graph, native menus,
decorations, dialogs, clipboard, drag-and-drop, HiDPI and accessibility for
free; every DCC-class tool Stratum is measured against is Qt; PySide6 would
let the planned Python scripting (J1 to J3) drive the UI too; UI code becomes
testable with `QTest` offscreen.

**Cons:** the renderer port is the biggest single-shot regression risk in the
project and it discards the SDL_GPU expertise and tests built since January;
retained-mode needs an explicit refresh discipline (poll `revision()` per
tick, or add observers); Qt adds 40 to 80 MB to the download and a platform
plugin layer with its own quirks (Wayland decorations, macOS bundle paths);
build and packaging complexity rises permanently; ImGui's "any state, any
frame" debug overlays are lost, and there is no shipped ImGui-on-QRhi backend
to keep them.

**Breaks:** `src/core` (Window, Application), `src/renderer`, `im3d_impl`,
every GPU test, the release packaging, the dependency policy.

### Option C: web UI

**C1: embedded Chromium (CEF) rendered offscreen into an SDL_GPU texture.**

| Item | AW | Notes |
|---|---:|---|
| Phase 0 extraction | 1.5 to 2 | |
| JS ↔ C++ bridge: typed command/query schema, JSON serialisation of scene state, undo integration | 1 to 2 | design work, not just code |
| CEF integration: binary distribution per OS (not a submodule), multi-process launch, offscreen paint → texture upload on the CPU path (SDL_GPU cannot import CEF's shared textures), input forwarding, HiDPI | 2 to 3 | |
| UI rebuild in TypeScript/React: docking (e.g. Dockview), Monaco for rules, React Flow for nodes, AG Grid/ECharts | 2 to 4 | best-in-class widgets, second toolchain |
| CI/toolchain: Node, bundler, CEF download and packaging | 1 | permanent dual-toolchain tax |
| **Total** | **7.5 to 12** | plus an ongoing tax every release |

**Pros:** the richest widget ecosystem of any option (Monaco alone beats every
C++ code editor), theming and layout are trivial, the same UI could later
serve a remote or hosted mode.

**Cons:** a 150 MB Chromium runtime and extra processes inside a desktop 3D
tool; UI pixels reach the GPU via CPU upload; the bridge turns every panel
into a protocol; two languages and toolchains for one developer; CEF has no
vendored-submodule story; startup time and memory rise sharply.

**C2: system webview side-by-side with a native SDL child window.** Inherits
the B1 foreign-window problem, plus three different engines (WebView2,
WebKitGTK, WKWebView) and no way to dock the viewport inside the web layout.
Rejected.

**C3: browser-first (core to WASM, renderer to WebGPU).** A re-platform of
the whole product. Out of scope for a months-long programme. Rejected.

### Option D: RmlUi over SDL_GPU (noted, not recommended)

RmlUi (MIT) is an HTML/CSS-style retained-mode UI that renders through your own
GPU backend; an SDL_GPU backend landed in 6.2, described upstream as "basic
rendering and transforms". It would give real styling and layout without
touching the renderer, but brings no docking, tree view, item models or code
editor, so those would be built by hand. It sits between A and B2 and delivers
less than either for its cost.

---

## 5. Side by side

| | A: ImGui + restructure | B2: Qt via QRhi | C1: CEF web UI |
|---|---|---|---|
| Total cost (AW) | 3 to 3.5 | 8.5 to 12.5 | 7.5 to 12, plus permanent tax |
| Renderer change | none | full port | none (CPU upload for UI) |
| Viewport risk | none | medium (port) | medium (composition, input) |
| Cross-platform status | builds on 3 OSes today | strong once deployment tooling is done | strong for UI, weak for packaging |
| Docking | WIP branch, pinned | Qt-ADS, mature | Dockview or similar |
| Large trees/tables (M7) | manual with `ListClipper` | item models, native | React virtualised lists |
| Code editor (rule language) | ImGuiColorTextEdit, adequate | `QPlainTextEdit`, good | Monaco, excellent |
| Node graph (optional) | imnodes, vendored | QtNodes | React Flow |
| Charts (M9) | implot (not vendored) | Qt Graphs/Charts licence check | ECharts, trivial |
| Accessibility / i18n / native chrome | none | yes | partial |
| Testability of UI | controller only | controller + `QTest` | controller + JS tests |
| Dependency policy | unchanged | one exception (Qt binaries) | broken (CEF binaries, Node) |
| Binary size delta | 0 | +40 to 80 MB | +150 MB and processes |
| Licence | MIT everywhere | LGPLv3 dynamic; some modules GPL-only | BSD (CEF) |
| Look and feel | "tool", improvable with theming | native desktop application | fully designed |
| Fits single dev + agents | best | good | worst (two stacks) |

---

## 6. Recommendation and phased plan

**Now, regardless of toolkit: Phase 0, the extraction (1.5 to 2 AW).**

1. Create `EditorModel` / `EditorController` in `src/editor` (or a new
   `src/app` if it should be reusable by the J3 headless CLI) with no ImGui
   include: the import/road/carve/export pipeline and its `poll_*` tick, GPU
   mesh ownership, a `RenderSettings` struct for lighting/sky/fog, rule
   load/save, `clear_imported_data()`, and a `ViewportInput` struct that the
   camera consumes instead of `ImGui::` calls.
2. Split `editor.cpp` so each panel is a file that only wires widgets to
   controller calls. Target: view code under ~1,300 LOC total.
3. Add controller tests to `stratum_tests` (no GPU needed for the pipeline
   state machine, the fingerprint checks, and settings round-trips).
4. Delete the borderless chrome in favour of native decorations or
   `SDL_SetWindowHitTest`; delete ImGuizmo/imnodes/ImGuiColorTextEdit unless
   adopted in the same change; correct `docs/architecture.md` and `README.md`.

**Then, as roadmap work, not migration work:** wire `src/scene` into the
editor. Real Scene Hierarchy over `LayerTree`, real Properties over
`AttributeStore` and `Selection`, Undo/Redo over `CommandStack`, picking in the
viewport. This is K1/K2 and the M7 prerequisite. Build it in ImGui first: it is
the fastest way to learn what the inspector and layer panel actually need, and
those two panels are the first place a retained toolkit would show a real
advantage, so they are the best evidence for the gate.

**Decision gate, before M7 starts.** Re-ask the toolkit question with the
extracted controller in hand and the first live scene panels built. Choose Qt
via QRhi (B2) if any of these are firm v1.0 requirements:

- native desktop look and accessibility on all three OSes;
- attribute tables and layer trees at thousands of rows with sorting,
  filtering and multi-edit;
- a code editor with LSP-grade diagnostics for the rule language;
- multiple OS windows (detached panels, second monitor);
- PySide-driven UI scripting.

Otherwise stay on ImGui through v1.0. If B2 is chosen, do it between M6 and
M7, because M7 to M9 add most of the remaining 23 UI features and every panel
built before the port is a panel ported twice.

**Do not pursue** B1, C2 or C3. Consider C1 only if a hosted or remote-UI mode
becomes a product goal, in which case the bridge from Phase 0's controller is
the starting point.

**On the "more stable cross-platform" premise.** SDL3 and ImGui already build
on Linux, Windows and macOS in CI, and the instability in the history is four
small, fixed seam bugs. What is genuinely unstable is that no layer of the UI
is tested and that Windows and macOS never run anything in CI. Phase 0 makes
the controller testable and a cheap follow-up runs `ctest` on all three
runners. Both improve stability more than a toolkit swap would, at a fraction
of the cost.

---

## 7. Open questions for Sarah

1. Is a native desktop look (menus, decorations, dialogs, accessibility) a
   requirement for v1.0, or is a well-themed tool UI acceptable for a
   game-developer audience?
2. Will the editor ever need more than one OS window?
3. Is a hosted or remote mode (UI in a browser, engine elsewhere) on the
   product horizon? That is the only thing that would make a web UI worth its
   cost.
4. Is the vendored-submodule policy negotiable for a prebuilt Qt in CI?
5. Are the renderer's GPU tests and lighting v2 something you would accept
   re-validating from scratch on a new GPU abstraction?

---

## Appendix: method and sources

Seven read-only audit passes, each written to a report; this document is the
vetted distillation and corrects a few overstatements in the raw reports.

- A: `editor.cpp`/`editor.hpp` structure, windows, frame flow, async, state
- B: UI-versus-logic classification of every function and member
- C: the three `panels/*.cpp` files and the add-on library usage
- D: ImGui bootstrap, render passes, input routing, vendored inventory
- E: planning docs and the future UI requirement list
- F: CI, tests, packaging, dependency policy, Python
- G: git history, growth of `editor.cpp`, ImGui-specific commits

Raw reports (session scratchpad, not checked in):
`/tmp/claude-1000/-home-sarah-Coding-Haptixxx-Stratum/68fd7e1b-d402-42ef-be82-60568e8a295c/scratchpad/ui-audit/`

External facts checked on 2026-09-26: SDL3 foreign-window properties and
Wayland custom-role behaviour (SDL wiki, README-wayland); Qt LGPLv3
obligations and GPL-only modules (qt.io licensing pages); QRhi and
`QRhiWidget` availability (doc.qt.io); RmlUi 6.2 SDL_GPU backend (RmlUi
releases); ImGui docking branch status (ocornut/imgui wiki).
