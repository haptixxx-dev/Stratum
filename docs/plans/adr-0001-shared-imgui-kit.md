# ADR-0001: Shared Dear ImGui extension kit for Synesthesia products

**Status:** Superseded by ADR-0002 on 2026-09-26.
**Date:** 2026-09-26
**Deciders:** Synesthesia architecture team
**Supporting analysis:** `ui-kit-assessment.md`, `ui-toolkit-decision.md`,
`ui-restructure-plan.md`, fidelity ladder (rendered 2026-09-26)

## Context

- Stratum's editor stays on Dear ImGui. A restructure is planned in eight
  phases; phases 4 to 6 produce the theme, widget helpers and panels.
- Stock ImGui widgets reach product quality (Tier 2) in 2 to 3 days.
  Custom-drawn widgets (Tier 3) are the practical ceiling of an extension.
- Haptixxx is part of the Synesthesia Group. Further C++ desktop tools are
  likely and would share the stack.
- Stratum is public under Apache-2.0 with every dependency vendored.
- Review outcome: the team wants features an extension cannot deliver and
  is content to maintain a public fork of Dear ImGui to get them.

## Decision (revised)

Two layers, not one:

1. **A shared UI kit** as an extension library (Option B): app-free target,
   library discipline, widgets never in core.
2. **A public superset fork of Dear ImGui** (Option D) under it, carrying
   only the features that must live in core, each gated and upstreamable.

The fork does not replace the kit. Widgets, themes, layouts and domain
components stay in the kit; the fork exists for the three core features in
the classification below and nothing else.

## Which features need the fork

The premise of D is "more features". It holds for three. Everything else
the review discussed is kit or backend work, where a fork adds maintenance
and no capability.

| Feature | Where it lives | Needs the fork | Cost and notes |
|---|---|---|---|
| Subpixel (LCD) text | core: glyph coverage in the atlas, blend mode in every backend shader | yes | 1 to 2 weeks. Benefit only on 1× Windows displays; macOS has no subpixel AA. Conflicts in `imgui_draw.cpp` on most upstream releases. |
| Text shaping, bidi, complex scripts | core: text layout, word wrap, `InputText` cursor and selection by cluster | yes | months, invasive. Only justified by a localisation requirement for Arabic, Hebrew or Indic scripts. |
| Accessibility tree (AccessKit or platform APIs) | core: node emission per frame, action routing, platform adapter | yes | weeks for buttons, toggles, sliders and text; months for full. Upstream has wanted this for years. The one feature that makes a public fork worth its name. |
| Native menus and dialogs | app and platform code | no | SDL3 has no native menus; this is per-app platform code either way. |
| Blur, shadows, compositing | backend, via `ImDrawCallback` mid-list | no | a draw callback samples the framebuffer; no core change. |
| Animation, tweening | kit | no | per-`ImGuiID` state in `ImGuiStorage`. |
| Node graph, timeline, tables, trees, property grid | kit | no | `imgui_internal.h` already exposes what these need. |
| Theme tokens, layout presets, designer-editable styling | kit | no | |
| Multiple OS windows | docking branch flag | no | already in the vendored branch. |
| Idle power, partial redraw | app-level frame throttling | no | |

If the team's feature list contains none of the first three rows, Option D
has no benefit and should be dropped for B.

## Constraints

1. **Extend, never fork** is replaced for D by **superset, never diverge**:
   `imgui.h` changes are additive only, and every MIT extension the kit
   relies on (imnodes, ImPlot, ImGuizmo, ImGuiColorTextEdit, Test Engine)
   builds unmodified in CI.
2. **App-free kit.** Links only the fork, FreeType and stb. Enforced in CI.
3. **Backend-agnostic kit.** Sees `ImTextureID` only.
4. **Licence:** resolved by a public fork. The fork stays MIT with upstream
   attribution; the kit is Apache-2.0; Stratum remains publicly buildable.
5. **Every core feature behind `IMGUI_ENABLE_<FEATURE>`.** With all flags
   off, the fork builds byte-identical to upstream. A parity CI job proves it
   on every commit.
6. **Track upstream on every release.** The fork's main is upstream docking
   plus rebased topic branches, one per feature. A topic branch that cannot
   rebase within a day is dropped from main until it can.
7. **Upstream first.** Each feature is written to be upstreamable and a pull
   request is opened. A feature upstream refuses stays as a topic branch and
   is reviewed for removal annually.
8. **At least two named maintainers** for the fork. Below that, D collapses
   to B by policy, not by drift.
9. **Semver** for the kit from 0.1.0; the fork is versioned as
   `<upstream>-syn.<n>`.
10. **Every kit widget ships with** a doc entry, a gallery render, a golden
    image and a usage example.

## Options

Scores 1 to 5, higher is better. Tier 1 (theme only) excluded.

| Dimension | A. Product tier only | B. In-repo kit, extract later | C. Separate repo first | D. B plus public superset fork |
|---|---:|---:|---:|---:|
| Visual fidelity reachable | 3 | 5 | 5 | 5 |
| Consistency within a product | 4 | 5 | 5 | 5 |
| Reuse in a second product | 2 | 4 | 5 | 5 |
| Time until Stratum benefits | 5 | 4 | 2 | 2 |
| Maintenance burden | 5 | 3 | 2 | 1 |
| Delivery risk | 5 | 4 | 2 | 2 |
| Testability | 2 | 5 | 5 | 4 |
| Fit with M7 needs | 3 | 5 | 5 | 5 |
| Agent-friendliness | 3 | 5 | 5 | 3 |
| Fits one developer plus agents | 5 | 4 | 2 | 1 |
| **Total** | **37** | **44** | **38** | **33** |

D scores below B on cost and risk, and equal on everything the kit already
delivers. Its whole case is the three core features and the maintainer
capacity to carry them. With two or more maintainers and accessibility as a
product value, D is defensible; with one developer it is not.

## Scope

| Version | Ships with | Contents |
|---|---|---|
| kit 0.1 | Stratum restructure phases 4 to 6 | theme tokens, fonts, icons, DPI, widget kit, dock presets, golden-image harness, gallery |
| fork 1.92.x-syn.1 | after kit 0.1 | upstream parity build, CI, extension compatibility job, no features yet |
| kit 0.2 | Stratum K1 and K2 | virtualised table, tree, multi-edit inspector, command palette, toasts, thumbnail grid |
| fork -syn.2 | when a maintainer is free | accessibility tree, basic roles |
| fork -syn.3 | on demand | LCD text (Windows); shaping only with a localisation requirement |
| kit 0.3 | Stratum M7 to M9 | node graph, viewport overlay layer, charts, tween helper, perspectives |

**Non-goals:** widgets in core, a frozen fork that stops tracking upstream
(unless chosen explicitly in decision 3), a new framework, native menus in
the fork.

## Consequences

**Easier:** the licence question is settled; accessibility becomes possible;
a public fork with a real feature attracts contributors; the kit is
unchanged.

**Harder:** every upstream release becomes a merge in the exact files the
features touch; two maintainers are a standing cost; agents can run merges
but core text and input surgery needs review; Stratum's benefit is delayed
until the parity build exists.

## Risks

| Risk | Mitigation |
|---|---|
| The feature list is all kit-level | classify each requested feature with the table above before approving D |
| Merge conflicts every release | one topic branch per feature; drop from main when it cannot rebase in a day |
| Silent divergence from upstream | flags-off parity build in CI |
| Extension ecosystem breaks | compatibility job builds imnodes, ImPlot, ImGuizmo, ImGuiColorTextEdit against the fork |
| Maintainers leave | constraint 8: below two, D collapses to B |
| Fork work displaces product work | fork features are scheduled only after kit 0.1 and outside Stratum milestones |

## Repository layout (revision 3)

A fork is a separate repository. This is not a choice. Stratum uses the fork
through the `external/imgui` submodule.

The developers want the kit in a separate repository from the start. The
alternative is to build the kit inside Stratum first and move it later. Both
paths end with the same three repositories. The difference is only in the
first six weeks. In those weeks, the developers create the kit from the
Stratum panels. The kit changes every day.

Recommendation: use a separate repository for the kit now. Developers who do
not work on Stratum need a small repository. The kit repository builds in
seconds. Stratum has 1.4 GB of submodules. Apply the seven conditions below.

| Repository | Licence | Contents | Consumers |
|---|---|---|---|
| `synesthesia/imgui` (fork) | MIT | upstream docking plus gated topic branches; parity and extension-compat CI | kit, Stratum, future products |
| `Synaesthesia-Group/chroma-kit` | Apache-2.0 | theme, fonts, widgets, layout, harness, gallery; **no copy of ImGui** | Stratum, future products |
| `haptixxx/stratum` | Apache-2.0 | product; `external/imgui` submodule points at the fork; `external/ui-kit` submodule | |

Conditions:

1. **The kit does not contain ImGui.** The product supplies the `imgui`
   CMake target. The library imnodes uses this method. The kit CI checks out
   the fork at a pinned tag. Without this condition, two copies of ImGui
   collide when the linker runs.
2. **A CMake cache variable points Stratum at a local copy of the kit.** A
   developer tests a widget change in Stratum without a submodule update.
   The submodule update occurs when the PR merges.
3. **A widget goes into the kit only when a linked product PR uses it.** The
   gallery is a test harness. The gallery is not a product.
4. **Products pin kit tags.** Products do not pin kit branches. Kit 0.1.0
   gets its tag when Stratum phase 6 merges.
5. **The three repositories use one CI template.** The template runs the
   build, the tests, the golden images for the kit, and the parity and
   extension-compatibility jobs for the fork.
6. **The first step is the parity pin.** Point the Stratum submodule
   `external/imgui` at the fork at tag `-syn.1`. This tag has zero features.
   This step tests the pipeline before a feature lands.
7. **Ownership.** The fork has two named maintainers. The kit has one owner.
   Stratum has one owner. A change across two repositories is two linked
   PRs.

## Open decisions

1. ~~Licence~~ resolved: public MIT fork, Apache-2.0 kit.
2. **Feature list**, classified with the table. Which of the three core
   features, in what order. Recommendation: accessibility first, LCD text
   second, shaping only with a localisation requirement.
3. **Sync model:** track upstream on every release (recommended) or freeze
   at 1.92.x and own the code.
4. **Named maintainers** for the fork, at least two.
5. Name and namespace; second product in one sentence; harness in kit 0.1.
