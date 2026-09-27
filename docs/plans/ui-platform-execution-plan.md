# UI platform execution plan

**Status:** Active
**Date:** 2026-09-26
**Owner:** Sarah. Interim reviewer: `SeamusMullan`.
**Basis:** ADR-0002 (fork first), `ui-restructure-plan.md` (Stratum phases),
`ui-kit-assessment.md`.

## Rules for the execution

1. **The orchestrating model writes no code.** The main session (Fable)
   writes plans and workflow scripts, reads summaries, and decides gates.
   Agents write code, run builds, run tests, push branches and open PRs.
2. **Nothing merges without Sarah.** Each gate lands as a PR with a
   verification block. Sarah merges. Tags follow merges.
3. **One workflow at a time.** Each workflow has fewer than 50 agents. The
   main session reads the result before the next workflow starts.
4. **Sequential edits, parallel checks.** Agents that edit the same
   repository run one after another on one branch. Agents that only read,
   build or test run in parallel. Worktree isolation is used only when two
   agents must edit at the same time.
5. **Small returns.** Agents return structured data or a summary under 300
   words. The main session never reads a transcript.
6. **Resume, do not restart.** An interrupted workflow resumes from its run
   id. The unchanged prefix is cached.

## Model policy

The policy exists to keep usage inside the limits. Cheap models do the
volume. Expensive models do the judgment.

| Task kind | Model | Effort |
|---|---|---|
| repository scaffolding, CI YAML, documentation formatting, file moves, running a build or a test and reporting, golden image generation | haiku | low |
| implementation of a specified unit, a refactor with tests, a first-pass review, a CMake target, a shader edit | sonnet | default |
| design with judgment (kit API, accessibility architecture, shaping design), adversarial verification of a change to the ImGui core, gate sign-off | opus | high |
| plan, workflow scripts, gate decisions | main session | no code |

Machine facts on 2026-09-26: 16 CPUs, so 14 agents run at once; 356 GB
free; 62 GB RAM.

## Repositories

| Repository | Local clone | Branch model |
|---|---|---|
| `Synaesthesia-Group/chroma` | `/home/sarah/Coding/Synesthesia/chroma` | `main` starts at upstream tag `v1.92.9b-docking`. `upstream-docking` mirrors upstream. One topic branch per core feature. Tags `1.92.9b-chroma.n`. |
| `Synaesthesia-Group/chroma-kit` | `/home/sarah/Coding/Synesthesia/chroma-kit` | `main`. Tags `0.1.0`, `0.2.0`, `0.3.0`. No copy of ImGui inside. |
| `haptixxx-dev/Stratum` | `/home/sarah/Coding/Haptixxx/Stratum` | `master`. One branch and one PR per restructure phase. `external/imgui` points at `Synaesthesia-Group/chroma` tag `1.92.9b-chroma.1` (gate S2). |

Upstream facts on 2026-09-26: docking HEAD is `3bae66c73`, version
1.93.0 WIP. The latest docking release tag is `v1.92.9b-docking`. Stratum
pins `84a9d532b`, version 1.92.6 WIP. The move to 1.92.9b is gate S2.

Dear ImGui Test Engine: the free licence applies to derivative software
released under an OSI licence. Chroma is MIT. The kit is Apache-2.0. The
kit may use the Test Engine.

AccessKit: the C bindings ship prebuilt libraries and a header (release
0.23.1, 2026-09-26). The fork does not need a Rust toolchain for F2.

## Workflows

| Id | Track | Repository | Produces | Gate | Depends on |
|---|---|---|---|---|---|
| WF-0 | bootstrap | all three | fork created, kit created, clones, docs PR | PR open | none |
| WF-1 | F1 | chroma | `main` at `v1.92.9b-docking`; README, NOTICE, CONTRIBUTING, CHANGELOG; parity job; extension-compatibility job; examples build job; parity script | CI green on the PR | WF-0 |
| WF-2 | S1 | Stratum | phases 0 to 3 as four PRs: golden harness and baseline; mechanical split; state promotion; import pipeline in `src/app` with tests | `ctest` green; golden numbers equal; VUID count 0; per PR | WF-0 |
| WF-3 | K1 | chroma-kit | CMake with consumer-provided `imgui` target; theme tokens; fonts, icons, DPI; widget kit; layout presets; `apps/refapp` seeded from the fidelity demo; headless harness; golden images; gallery; CI on three operating systems | CI green; every widget has doc, gallery, golden, Test Engine script | WF-1 merged and tagged |
| WF-4 | S2, S3 | Stratum | submodule at `1.92.9b-chroma.1`; kit submodule at `0.1.0`; phases 4 to 6 on the kit | three-OS build; VUID 0; screenshots; golden numbers equal | WF-2, WF-3 tagged |
| WF-5a | F2 design | chroma | accessibility design: AccessKit versus platform APIs, node emission, id stability, action routing; test plan | opus panel agrees; Sarah approves the design | WF-1 |
| WF-5b | F2 build | chroma | `IMGUI_ENABLE_ACCESSIBILITY` topic branch; tree dump tests; Test Engine reads the tree | parity green; tests green; screen reader reads the refapp | WF-5a, WF-3 |
| WF-6 | F3 | chroma | `IMGUI_ENABLE_LCD_TEXT` topic branch; SDL_GPU backend blend; Windows goldens | parity green; goldens match on Windows at 1× | WF-1, WF-3 |
| WF-7a | F4 design | chroma | shaping design: HarfBuzz, run layout, `InputText` clusters, bidi | opus panel agrees; Sarah approves | WF-1 |
| WF-7b | F4 build | chroma | `IMGUI_ENABLE_SHAPING` topic branch; goldens for three scripts; cursor tests | parity green; goldens match | WF-7a, WF-3 |
| WF-8 | K2, K3 | chroma-kit | table, tree, inspector, palette, toasts, thumbnails; node graph, overlay layer, charts, perspectives | K1 gate per widget | WF-3 |
| WF-9 | S4 | Stratum | later fork and kit tags when a feature is needed | S1 gate | WF-4 |

Order: WF-0, then WF-1 and WF-2 in either order, then WF-3, then WF-4.
WF-5a, WF-6 and WF-7a can start after WF-1. WF-5b, WF-6 and WF-7b need
WF-3 for the refapp gates.

## Shape of one workflow

Each workflow follows the same shape:

1. **Scout.** Haiku agents read the target files and return a work list.
2. **Implement.** One sonnet agent per unit, in sequence on one branch, one
   commit per unit.
3. **Check.** Haiku agents build, run tests, count VUID lines, render
   goldens, in parallel.
4. **Verify.** One opus agent reviews the diff against the gate and the ADR
   constraints. For a change to the ImGui core, three opus agents try to
   refute the change; two refutations block it.
5. **Land.** One haiku agent pushes the branch, opens the PR with the
   verification block, and watches CI to a green or red result.

The main session reads the final summary and decides: next workflow, fix
loop, or stop.

## Facts the agents need

- Stratum verification: `ctest --preset tests`; `timeout 6
  ./build/release/bin/stratum 2>&1 | grep -c VUID` must print 0; the Lucan
  extract is at `/home/sarah/Downloads/lucan.osm`; the screenshot recipe
  crops and deletes the root capture in one command.
- The fidelity demo is at the session scratchpad
  `fidelity/fidelity_demo.cpp` with `icons.hpp` and `fonts/lucide.ttf`. WF-3
  copies it into `apps/refapp`.
- Inter and JetBrains Mono are installed under
  `/home/sarah/.local/share/fonts/`. The kit vendors font files with their
  licences.
- `cp` is aliased to `cp -i` in the interactive shell. Agents use
  `command cp -f`.
- Never run `git checkout` or `git pull` in a repository while a workflow
  edits it.
