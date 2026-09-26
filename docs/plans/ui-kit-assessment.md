# A shared ImGui extension for Synesthesia Group: architectural assessment

**Status:** Proposed
**Date:** 2026-09-26
**Decider:** Sarah
**Follows:** `ui-toolkit-decision.md` (stay on ImGui), `ui-restructure-plan.md`
(phases 0 to 8), and the fidelity ladder rendered on 2026-09-26.

---

## The proposal as stated

Haptixxx sits inside the Synesthesia Group, a company to be. More software is
likely to be built on the same stack. Rather than polishing Stratum's UI in
place, build a custom Dear ImGui extension up front, for internal use across
products, so that custom controls, visualisation and the complex parts plain
ImGui does not cover integrate cleanly later.

## Verdict in five lines

1. **Yes to a shared extension. No to "up front" in the usual sense.** Build
   it as a separately named, separately compiled, app-free library target
   inside the Stratum repository now, with library discipline from the first
   commit, and move it to its own repository the day a second product needs
   it. That gives almost all the reuse benefit for about one extra day on the
   restructure plan.
2. **"Custom implementation" must mean extension, never fork.** An extension
   library on top of the public ImGui API is how the entire ImGui ecosystem
   works. A modified ImGui core is a maintenance sink that one developer
   cannot carry, and a new immediate-mode framework is a multi-year project.
3. **It does not raise the ceiling.** The extension makes Tier 3 repeatable
   and testable. It does not deliver subpixel text, accessibility, text
   shaping or native controls. Those stay out of reach on ImGui with or
   without an in-house layer.
4. **Licensing decides the repository layout, not architecture.** Stratum is
   public and Apache-2.0. If the extension is proprietary, Stratum cannot
   depend on it without breaking public builds. Either the extension is open
   source or Stratum stops being buildable by outsiders. Decide that first.
5. **Ranking:** in-repo custom kit beats the product tier, which beats a
   separate-repo kit built ahead of a second consumer. The product tier is
   not an alternative to the kit; it is the kit's first milestone.

---

## 1. Critique of the reasoning

### 1.1 Three things "custom implementation" could mean

| Reading | What it is | Verdict |
|---|---|---|
| **A. Extension library** | Theme tokens, fonts and icons, a widget kit, data-heavy widgets, and complex components built on the public ImGui API, packaged and versioned. Exactly what ImPlot, imnodes, ImGuizmo and ImGuiColorTextEdit are. | Right. This is the proposal that makes sense. |
| **B. Fork of ImGui** | Modified core: text shaping, retained layers, an animation system, accessibility hooks. | Wrong for this company size. Every upstream release becomes a merge; the docking branch already rebases weekly. The first product feature that needs a core patch becomes a permanent divergence. |
| **C. Own immediate-mode framework** | What Our Machinery did with The Machinery, or Unity with IMGUI then UI Toolkit. | Out of scope by years. Both examples had teams. |

The proposal is sound only as A. Write that boundary into the library's
README on day one: **extend, never fork**, with one measured escape hatch (a
`patches/` directory of upstream-bound patches applied at build time, each
with a test that fails when upstream absorbs it).

### 1.2 "Up front" is the risky word

Building a shared library before the second consumer exists is speculative
generality. The known failure mode: the library is designed for imagined
needs, the first product bends to fit the library, and a real second user
arrives a year later with needs that do not match. The rule of three exists
for this reason.

The mitigation is cheap and already in the restructure plan: `src/editor/ui/`
(theme, icons, widgets, layout) is the kit. Give it library discipline now
without giving it a repository:

- its own CMake target with a product-neutral name;
- no include of anything under `src/` outside itself; dependencies are
  `imgui`, FreeType, stb and nothing else;
- its own tests and its own docs;
- backend-agnostic: it never sees SDL_GPU, only `ImTextureID`.

Extraction to a separate repository is then a `git subtree split` and a
submodule, done when the second product is real. Design for extraction;
do not extract yet.

### 1.3 What "plain ImGui may not support" actually is

The complex parts named in the proposal (controls, visualisation) are all
achievable as extensions, and most already exist as MIT libraries:

| Need | Achievable as extension? | Existing MIT building block |
|---|---|---|
| Property grids, toggles, sliders, chips, sections, status bars | yes, Tier 3 shows it | none needed; ~1,000 lines of helpers |
| Virtualised tables with sort, filter, multi-select | yes | `ImGuiListClipper` + tables API |
| Trees with drag-drop, visibility, lock | yes | tables API + drag-drop API |
| Node graphs | yes | imnodes (vendored, unused), imgui-node-editor |
| 2D plots, dashboards | yes | ImPlot |
| Timelines, curve editors, gradients | yes | ImGuizmo ships ImSequencer, ImCurveEdit, ImGradient |
| Code editor with markers | yes | ImGuiColorTextEdit (vendored, unused) |
| Gizmos and viewport overlays | yes | ImGuizmo, im3d (used) |
| Animation, transitions, hover fades | yes, per-widget tween state keyed by `ImGuiID` | none; small helper |
| Command palette, toasts, notifications | yes | none; small helpers |
| Multiple OS windows | yes, multi-viewport flag on the vendored docking branch | ImGui itself; least stable feature |
| UI automation tests | yes | Dear ImGui Test Engine (dual-licensed; check terms) |

And what stays out of reach regardless of an in-house layer:

| Need | Why not |
|---|---|
| Subpixel text on 1× displays | greyscale antialiasing in the rasteriser; a fork-level change |
| Text shaping, right-to-left, complex scripts | no HarfBuzz integration in the font pipeline; the 1.92 loader interface makes a shaping loader conceivable but layout of shaped runs is core work |
| Accessibility tree | no OS accessibility API; a long-standing upstream discussion with no implementation |
| Native menus and controls | ImGui draws everything; only the window frame and file dialogs can be native |
| Blur and real compositing effects | backend render-pass work per product, not library work |
| Designer-editable layouts | there is no layout file format; the theme is code (tokens as data is the practical substitute) |

So the extension covers exactly the list in the proposal, and none of the
list above. State this in the proposal so nobody expects Substance-class
fidelity from it.

### 1.4 Reuse depends on what the second product is

The value of a shared ImGui kit assumes the next products are C++, desktop,
real-time, and shaped like "viewport plus panels". If the next product is a
web dashboard, a mobile app, or a plugin inside someone else's host, the kit
has no consumer.

If Synesthesia's other work is audio, haptic or cross-modal tooling, the
domain widgets differ from Stratum's (timelines, curve editors, waveform
views, meters, knobs, transport controls versus property grids, layer trees
and a 3D viewport). What is shared is the **foundation kit**: theme tokens,
fonts, icons, DPI, property grid, docking presets, tables, command palette,
toasts, testing harness. Domain widgets arrive with their product. Scope the
proposal to the foundation.

### 1.5 Licensing decides the repository, before architecture does

Stratum is public under Apache-2.0. A public repository cannot depend on a
private submodule without breaking every outside build. Therefore:

- **Open-source kit (Apache-2.0 or MIT):** it can live inside Stratum now
  and move out later. This matches the ImGui ecosystem norm and helps
  recruiting and coding agents.
- **Proprietary kit:** it must be a separate private repository from day
  one, and Stratum must either stop depending on it or stop being publicly
  buildable.

"Internal use" in the proposal reads as proprietary. That is the one
sentence that changes the whole plan. Recommend open source; the kit's
value is in the products built on it, not in the widgets themselves.

---

## 2. Pros and cons

### Pros

- **Consistency across products** from one token system, one icon set, one
  set of widget behaviours.
- **One place to fix the hard integration problems**: DPI scaling on
  Windows, font merging, glyph exclusions, dock persistence, ini locations.
  Each was a separate commit in Stratum's history.
- **Faster second-product bring-up**: a new tool starts at Tier 2 on day one.
- **Owned API surface**: products call the kit's namespace; the kit decides
  whether imnodes, ImPlot or an in-house widget sits behind it. Swapping a
  dependency is contained.
- **Testability**: the 590-line offscreen renderer written for the fidelity
  ladder is already a golden-image harness. Every widget gets a reference
  PNG and a diff test; the first UI tests this project has ever had.
- **Agent efficiency**: a documented, gallery-rendered widget kit is what
  makes coding agents produce consistent UI. This matters more here than in
  a conventional team.
- **Brand as data**: tokens in a struct with JSON overrides means a second
  product is a different token file, not a different code path.

### Cons

- **Cost before the second consumer**: roughly one extra week over the
  restructure plan for library discipline, docs and the harness. Real but
  bounded.
- **Speculative API**: any interface designed for a product that does not
  exist yet will be wrong. Mitigated by extracting from Stratum's needs only.
- **Fork temptation**: owning a UI layer invites patching ImGui core the
  first time a widget needs something the API lacks. The escape hatch must be
  narrow and audited.
- **Two things to maintain with one developer**: a library and a product,
  each with a version and a changelog. Semver discipline costs attention.
- **Cross-product coupling**: a breaking change in the kit blocks every
  product's upgrade. Requires pinning and a deprecation policy from v0.1.
- **Docking-branch exposure**: the kit's guarantees rest on a branch its
  author wants to rewrite. Pinned submodule contains it; it does not remove it.
- **Displaces roadmap work**: the stated schedule risk is ordering, not
  hours. A kit that grows ahead of product features delays M5 to M7.

---

## 3. Ranking: custom kit versus the product tier

Tier 1 (theme only) is excluded as requested. Scores are 1 to 5, 5 is best.

| Dimension | Product tier (Tier 2, stock widgets, helpers inside `src/editor/ui/`) | Custom kit, in-repo target, extract at second consumer | Custom kit, separate repo up front |
|---|---|---|---|
| Visual fidelity reachable | 3 | 5 | 5 |
| Consistency within a product | 4 | 5 | 5 |
| Reuse in a second product | 2 (copy and paste) | 4 (subtree split) | 5 |
| Time until Stratum benefits | 5 (2 to 3 days) | 4 (3 to 4 days plus one week of discipline spread over phases 4 to 6) | 2 (foundation first, product later) |
| Maintenance burden | 5 | 3 | 2 |
| Delivery risk | 5 | 4 | 2 |
| Testability | 2 | 5 (golden images) | 5 |
| Fit with M7 needs (K1 to K5, node graph, dashboards) | 3 (grows ad hoc) | 5 | 5 |
| Agent-friendliness | 3 | 5 | 5 |
| Fits one developer plus agents | 5 | 4 | 2 |
| **Total** | **37** | **44** | **38** |

Reading the table: the product tier is the floor and the kit's v0.1. The
in-repo kit wins by being the same work with a name, a boundary, tests and
docs. The separate-repo-first option scores like the product tier for the
wrong reasons: it trades delivery risk for reuse that has no consumer yet.

---

## 4. Targets, if the answer is yes

### Governance (day one)

- Product-neutral name and namespace; nothing under it includes Stratum
  headers. Enforced by a CMake target that links only `imgui`, FreeType and
  stb, and by a grep in CI.
- Extend, never fork. `imgui_internal.h` is included from exactly one shim
  file. Core patches live in `patches/`, each with a test that fails when
  upstream absorbs it.
- ImGui pinned as a submodule; a quarterly bump task; a canary CI job that
  builds against upstream docking HEAD and is allowed to fail.
- Semver from v0.1.0. Products pin a tag. Breaking changes carry a
  deprecation shim for one minor version.
- Licence decided and written in the README before the first commit.
- Every widget has: a doc entry, a gallery render, a golden image, a usage
  example. No exceptions; this is what makes the kit usable by agents.

### v0.1, delivered with Stratum restructure phases 4 to 6

- Theme tokens as a struct with JSON override; one `apply()`; dark first, a
  light table later.
- Fonts: UI, mono, icon font merged with PUA exclusion; DPI scale from the
  host; the icon codepoint header generated from the font's CSS.
- Widget kit: property row and grid, section header, toggle, slider with
  value, colour pill, chip, help marker, empty state, toolbar, status bar,
  search box, progress with cancel.
- Layout: dockspace host, layout presets, reset, ini path from the host.
- Harness: offscreen render to PNG (SDL_Renderer software backend), golden
  image diff with a tolerance, a gallery page generated from the harness.

### v0.2, delivered with Stratum K1 and K2

- Virtualised table with sort, filter, multi-select and column persistence.
- Tree with drag-drop, visibility and lock columns, filter.
- Inspector building blocks: multi-edit with mixed-value state, attribute
  source badges, edit-begin and edit-commit callbacks so a host maps a drag
  to one undo command rather than sixty.
- Command palette, toasts, notification centre.
- Thumbnail grid browser with async texture slots (`ImTextureID` only).

### v0.3, delivered with Stratum M7 to M9

- Node graph behind the kit's API, implemented on imnodes or
  imgui-node-editor, swappable.
- Viewport overlay layer: gizmo bar, stats pill, legends, snap constraint
  widget, on top of ImGuizmo and im3d.
- Charts behind the kit's API on ImPlot.
- Animation helper: per-`ImGuiID` tween state for hover, expand, progress.
- Workspace perspectives: named layouts with panel sets.

### Non-goals, written down

Forking ImGui core; text shaping; accessibility; native menus; a retained
scene graph; layout files; non-C++ bindings before a product asks (Python
via pybind11 is plausible later, given J1 to J3).

---

## 5. What would change the answer

- **The second product is not C++ desktop.** Then build Stratum's kit as
  product tier plus Tier 3 widgets where needed, skip the discipline, and
  revisit when a real C++ consumer appears.
- **The kit must be proprietary.** Then it cannot live in Stratum's public
  tree, and Stratum's public buildability must be given up or the kit made
  optional. That is a business decision, not a UI one.
- **A v1.0 requirement lands for accessibility, subpixel text or native
  controls.** Then the toolkit decision reopens (see `ui-toolkit-decision.md`
  §6) and the kit's API surface is what makes a Qt port bounded: the host
  calls the kit, not ImGui.

## 6. Decisions needed

1. Open-source kit inside Stratum now, or proprietary in a separate repo
   with the consequences in §1.5?
2. What is the most likely second product, in one sentence? It sets the
   v0.2 and v0.3 scope more than anything else.
3. Name and namespace.
4. Does the kit own the golden-image harness and gallery from v0.1, or do
   those wait for v0.2? Recommend v0.1; it is the cheapest time.
