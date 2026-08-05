#!/usr/bin/env bash
set -euo pipefail
SESSION="stratum-features"
BASE_DIR="$(cd "$(dirname "$0")" && pwd)"
# Keep worktrees OUTSIDE the repo. Pointing this inside BASE_DIR got the
# worktree dirs committed as stray gitlinks by an unrelated `git add`.
WT_DIR="${STRATUM_WT_DIR:-$(dirname "$BASE_DIR")/Stratum-Worktrees}"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n "entity-selection"
tmux send-keys -t "$SESSION:entity-selection" \
  "cd '$WT_DIR/feat-entity-selection' && claude -p 'Implement entity selection (click to select) in the editor viewport. Read CLAUDE.md for project context. Read docs/osm_procgen_plan.txt for requirements. The camera already has screen_to_world ray capability (editor/camera.cpp). Use the BVH spatial index for raycasting against mesh bounds. Add visual selection highlight (outline or color tint). Wire it into the scene panel so clicking an entity in viewport selects it in the hierarchy and vice versa.'" C-m
tmux new-window -t "$SESSION" -n "gizmos"
tmux send-keys -t "$SESSION:gizmos" \
  "cd '$WT_DIR/feat-transform-gizmos' && claude -p 'Implement transform gizmos (translate/rotate/scale) using the ImGuizmo library already linked in CMakeLists.txt. Read CLAUDE.md for project context. Read docs/osm_procgen_plan.txt for requirements. This depends on having a selected entity concept - add a selected_entity field to the Editor if not present. Render ImGuizmo manipulators in the viewport when an entity is selected. Apply transforms back to the entity. Add keyboard shortcuts: W=translate, E=rotate, R=scale.'" C-m
tmux new-window -t "$SESSION" -n "save-load"
tmux send-keys -t "$SESSION:save-load" \
  "cd '$WT_DIR/feat-project-save-load' && claude -p 'Implement project save/load (.stratum files) using nlohmann/json (already linked). Read CLAUDE.md for project context. Read docs/osm_procgen_plan.txt for requirements. Serialize: camera state, loaded OSM file path, procgen settings, scene entities with their components (transforms, mesh references, OSM metadata). Use ImGuiFileDialog (already linked) for file browser. Add File menu items: New, Open, Save, Save As. File extension: .stratum'" C-m
tmux new-window -t "$SESSION" -n "shadows"
tmux send-keys -t "$SESSION:shadows" \
  "cd '$WT_DIR/feat-shadow-mapping' && claude -p 'Implement shadow mapping for the PBR renderer. Read CLAUDE.md for project context. Read docs/plans/shadow_implementation.md for the FULL detailed implementation plan with code snippets - follow it closely. Create shadow pass with depth-only rendering, add PCF soft shadows to mesh_pbr.frag, compute light-space matrix from the directional sun light. Shadow map resolution 2048x2048. Add a toggle in the UI.'" C-m
tmux new-window -t "$SESSION" -n "pdf-docs"
tmux send-keys -t "$SESSION:pdf-docs" \
  "cd '$WT_DIR/feat-pdf-docs' && claude -p 'Fix and update the docs/ system so it can generate a PDF with rendered diagrams. Read CLAUDE.md for project context. There is a docs/Makefile and docs/generate-pdf.sh already - read them and fix/update as needed. The docs use Mermaid diagrams (architecture.md, roadmap.md, c4-diagrams.md). Ensure the pipeline: Mermaid->rendered images->PDF works. Use pandoc + mermaid-filter or mmdc (mermaid CLI) for rendering. Update the Makefile targets. Test that make -C docs pdf works. Update any broken diagram syntax.'" C-m
tmux select-window -t "$SESSION:entity-selection"
tmux attach -t "$SESSION"

