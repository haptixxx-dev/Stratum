# ADR-0002: Fork-first UI platform for Synesthesia products

**Status:** Proposed. This ADR supersedes ADR-0001. Decisions 1, 2, 3, 4 and
5 were recorded on 2026-09-26. The kit name is open.
**Date:** 2026-09-26
**Deciders:** Synesthesia architecture team
**Decider preference:** Sarah prefers to build the fork first. Stratum and
other products integrate later.
**Supporting analysis:** `ui-kit-assessment.md`, `ui-toolkit-decision.md`,
`ui-restructure-plan.md`, ADR-0001

## Context

- The team decided to make a public fork of Dear ImGui.
- Time is not a constraint. The company has internal AI compute with no
  practical limit. Human review is the limited resource.
- Stratum stays on Dear ImGui. The Stratum restructure has eight phases.
  Phases 0 to 3 do not depend on the fork. Phases 4 to 6 build the UI.
- Stratum is public under Apache-2.0. Stratum vendors each dependency as a
  submodule.
- More C++ desktop products are likely in the Synesthesia Group. The second
  product is likely 3D-related software. The fork must cover general use,
  not 3D use only.

## What changes when time is not a constraint

These costs go away:

- The cost to write the fork features.
- The cost to resolve upstream merge conflicts.
- The cost to write widgets before a product needs them.
- The cost to rebuild a wrong abstraction.

These costs remain:

- Human review. A person must read each change to the ImGui core.
- Unknown requirements. The second product is not defined.
- Upstream acceptance. Upstream decides which pull requests it merges.
- Platform limits. macOS has no subpixel text. AI compute does not change
  this.
- Order of work. A product can pin a version only after the version passes
  its gates.

Because of this, the decision uses gates and not dates. A gate is a set of
automatic checks. A track moves to the next step when the gate is green.

## Decision

1. Make a public superset fork of Dear ImGui first.
2. Build the kit in a separate repository, on the fork, in parallel.
3. Integrate Stratum and other products later. A product integrates when
   the fork and the kit have tagged versions with green gates.
4. Start Stratum restructure phases 0 to 3 now. These phases do not depend
   on the fork. They prepare Stratum for the integration.

## Tracks and gates

| Track | Step | Gate |
|---|---|---|
| F1 fork | Create the fork from upstream docking. Add the parity job and the extension-compatibility job. Tag `1.92.x-syn.1`. | Parity job is green. All five extensions build. |
| F2 fork | Add the accessibility tree behind `IMGUI_ENABLE_ACCESSIBILITY`. Use AccessKit or the platform APIs. | Parity job is green. Test Engine scripts read the tree for each widget type. A screen reader reads the demo app. |
| F3 fork | Add subpixel text behind `IMGUI_ENABLE_LCD_TEXT`. Update the SDL_GPU backend shader. | Parity job is green. Golden images match on Windows at 1×. |
| F4 fork | Add text shaping behind `IMGUI_ENABLE_SHAPING`. Use HarfBuzz. | Parity job is green. Golden images match for Arabic, Hebrew and Devanagari. `InputText` cursor tests pass. |
| K1 kit | Create the kit repository. Add the reference app, the harness, the gallery and the widget kit from Stratum phase 5. Tag `0.1.0`. | Each widget has a doc entry, a gallery render, a golden image and a Test Engine script. The reference app passes its gates. |
| K2 kit | Add the table, the tree, the inspector, the command palette and the toasts. Tag `0.2.0`. | Same gate as K1. |
| K3 kit | Add the node graph, the viewport overlay layer, the charts and the perspectives. Tag `0.3.0`. | Same gate as K1. |
| S1 Stratum | Run restructure phases 0 to 3: baseline, split, model, pipeline. | `ctest` is green. Golden import numbers match. VUID count is 0. |
| S2 Stratum | Point `external/imgui` at the fork tag `-syn.1`. | Stratum builds on three operating systems. VUID count is 0. |
| S3 Stratum | Add the kit submodule at tag `0.1.0`. Run restructure phases 4 to 6 on the kit. | Same gate as S1, plus screenshots. |
| S4 Stratum | Move to later fork and kit tags when the product needs a feature. | Same gate as S1. |

F1 and S1 start now. F2, F3 and F4 run in this order. K1 can start after F1.
S2 can start after F1. S3 can start after K1 and S1.

## The reference app

The reference app is the consumer of the kit and the fork before a product
exists. It lives in the kit repository at `apps/refapp`. The kit owner owns
it. It starts from the fidelity demo of 2026-09-26.

The reference app has these properties:

- It is one C++20 executable. It uses SDL3. It has two modes. The window
  mode uses the SDL_GPU backend. The headless mode uses the SDL_Renderer
  software backend and writes PNG files. No window opens in headless mode.
- It has five pages. Each page is a gate.
- It uses fixed demo data. It has no random values without a fixed seed.
- It is not a product. It holds no domain widgets. It has no network code.

| Page | Contents | Gate |
|---|---|---|
| Gallery | Each kit widget in each state: normal, hover, active, disabled, error. The doc string is beside each widget. The page is generated from the widget registry. | Golden images match at 1×, 1.5× and 2×, in dark and light themes. |
| 3D tool mock | The Stratum layout: viewport stand-in, scene, import, render settings, console, rule editor, status bar. | Golden images match. Test Engine scripts drive each panel. |
| General tool mock | A layout with no viewport: a large table, a timeline, a node graph and a chart. This page proves general use. | Golden images match. Test Engine scripts drive each panel. |
| Fork features | Text at many sizes and weights. Latin, Cyrillic, Greek, CJK, Arabic, Hebrew and Devanagari text. Subpixel text beside greyscale text. `InputText` cursor and selection. Each widget type with its accessibility role and name. | Golden images match per operating system. The accessibility tree dump matches the expected dump. |
| Stress | A table with 100,000 rows. A tree with 10,000 nodes. A draw list with 1,000,000 vertices. | The frame time is under budget. The budget is in the repository. |

The headless mode has three commands:

- `refapp --headless --out <dir>` renders each page at each scale and theme.
- `refapp --test` runs the Test Engine scripts.
- `refapp --accept` replaces the golden images. A reviewer approves the
  change.

The golden images live in `tests/golden/<page>/<scale>/<theme>.png`. The
Fork features page has one golden image per operating system. The other
pages have one golden image for all operating systems. The comparison has a
tolerance for antialiasing.

## Naming

The fork is named **Chroma**. The decider chose this name on 2026-09-26
after a workshop of twenty candidates. The name does not contain "imgui".
These rules apply:

1. **The code identity does not change.** The headers stay `imgui.h` and
   `imgui_internal.h`. The namespace stays `ImGui`. The CMake target stays
   `imgui`. Constraint 1 requires this. The kit and the extensions link to
   the target `imgui`. An alias target `chroma::imgui` is optional.
2. **The project identity carries the name.** The repository is
   `synesthesia/chroma`. The tag pattern is `1.92.x-chroma.n`. The
   documentation, the CI names and the release notes use "Chroma".
3. **The README states the origin in the first sentence.** "Chroma is a
   superset fork of Dear ImGui." The MIT licence file keeps the upstream
   copyright line. The repository description on GitHub says "fork of Dear
   ImGui". This is a licence duty and it helps search.
4. **The kit gets its own name later.** The kit is not "Chroma". A product
   links two things: the fork and the kit. Two names prevent confusion.

Known collisions for "Chroma" on 2026-09-26:

| Project | Domain | Effect |
|---|---|---|
| ChromaDB, `chroma-core/chroma` | vector database, 26,000 stars | Search for "chroma" returns the database first. |
| `chroma.js` | colour library, JavaScript | Search for "chroma colour" returns it. |
| `chroma-react` | design system, React | Search for "chroma design system" returns it. |
| `zaneenders/chroma` | UI library, Swift | Same category as the fork. |
| Razer Chroma | RGB lighting SDK | Search for "chroma SDK" returns it. |

None of these is a legal block. The name is free on GitHub under the
`synesthesia` organisation. The cost is search visibility. The decider
accepts this cost. The phrase "Chroma ImGui" is unique and finds the fork.
The repository description and the README first sentence carry that phrase.

Accepted risk: "Chroma" is a colour term. A future Synesthesia product about
colour or vision cannot use this name. The decider accepts this.

## Constraints

1. **Superset, never diverge.** Changes to `imgui.h` add items only. The
   five extensions build unmodified: imnodes, ImPlot, ImGuizmo,
   ImGuiColorTextEdit and Test Engine.
2. **Each core feature has a flag.** With all flags off, the fork builds
   byte-identical to upstream. The parity job proves this on each commit.
3. **Track upstream on each release.** Decided on 2026-09-26. Each feature
   is one topic branch. The fork rebases each topic branch on each upstream
   release.
4. **Upstream first.** Each feature is written for upstream. The fork opens
   a pull request for each feature.
5. **Human review policy.** Two named reviewers approve each change to the
   ImGui core. One named reviewer approves each change to the kit API.
   Machine gates approve all other changes.
   Interim exception: `SeamusMullan` is the single reviewer for the fork
   core while the other reviewers are on leave. The exception ends when a
   second reviewer is named.
6. **The kit does not contain ImGui.** The product supplies the `imgui`
   CMake target. The kit CI checks out the fork at a pinned tag.
7. **A CMake cache variable points a product at a local copy of the kit.**
8. **Harness-driven, not consumer-driven.** A widget goes into the kit when
   it has a gallery entry, a golden image and a Test Engine script. A
   product PR is not required.
9. **Products pin tags.** Products do not pin branches.
10. **The three repositories use one CI template.**
11. **A product pins a version only when the gate for that version is
    green.**

## Options

Scores are 1 to 5. A higher score is better. The dimensions are the ones
that matter when effort is free.

| Dimension | A. Product tier only | B. Kit inside Stratum | D. Fork and kit, product first | E. Fork first, products later |
|---|---:|---:|---:|---:|
| Fidelity ceiling | 3 | 4 | 5 | 5 |
| Reuse across products | 2 | 3 | 5 | 5 |
| Correctness risk | 5 | 4 | 3 | 3 |
| Human review load | 5 | 4 | 2 | 2 |
| Reversibility | 5 | 4 | 3 | 3 |
| Fit with the decider preference | 1 | 2 | 3 | 5 |
| Fit with the order of gates | 3 | 3 | 4 | 5 |
| **Total** | **24** | **24** | **25** | **28** |

E is the decision. Its cost is the human review load. Constraint 5 sets the
policy for that load.

## Risks

| Risk | Mitigation |
|---|---|
| The kit API is wrong because no product uses it yet. | The kit has a reference app. The reference app copies the Stratum panels. The fidelity demo from 2026-09-26 is the seed. |
| Human review becomes the bottleneck. | Constraint 5. Machine gates approve all changes outside the core and the kit API. |
| An upstream release conflicts with a topic branch. | The rebase is cheap. The gate proves the result. A topic branch with a red gate leaves the main branch until the gate is green. |
| Upstream refuses a feature. | The feature stays as a topic branch. The team reviews it each year. |
| The second product is not C++ desktop. | The kit holds foundation widgets only. Domain widgets wait for their product. |
| Stratum waits for the fork. | Stratum phases 0 to 3 do not wait. Only phases 4 to 6 wait for K1. |
| macOS has no subpixel text. | F3 targets Windows at 1×. The gate says so. |

## Consequences

Easier:

- The fork gets all three core features. Cost is not a reason to drop one.
- The kit is complete before a product uses it.
- The licence question is settled. The fork is MIT. The kit is Apache-2.0.
- Stratum integrates against tagged, tested versions.

Harder:

- Two reviewers must read each core change.
- The kit can hold widgets that Stratum does not use.
- Stratum phases 4 to 6 wait for gate K1.

## Open decisions

1. Reviewers. Recorded: `SeamusMullan` is the interim single reviewer for
   the fork core. Open: the name of the second reviewer.
2. The second product. Recorded: likely 3D-related software. The fork covers
   general use.
3. Sync model. Recorded: track upstream on each release.
4. Name. Recorded: the fork is Chroma. Repository `synesthesia/chroma`.
   Tags `1.92.x-chroma.n`. Open: the kit name.
5. Location of the reference app. Recorded: the kit repository, at
   `apps/refapp`.
