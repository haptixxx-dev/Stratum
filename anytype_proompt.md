# Anytype Task Import Prompt

Use this prompt to generate tasks importable into Anytype.

---

## Prompt

```text
Generate a list of granular tasks for [DESCRIBE FEATURE/PROJECT/ISSUE].

Output as individual markdown files in a folder called `anytype_tasks/`, then zip them.

Each file should have YAML frontmatter with these fields:
- Status: To Do
- Version: [version or milestone]
- Category: [group/area]
- Priority: High/Medium/Low
- Tag: [version, category]

Format:
---
Status: To Do
Version: v0.1
Category: Window Setup
Priority: High
Tag: [v0.1, Window Setup]
---

# Task name here

Requirements:
- Make tasks small and actionable (1-2 hours max)
- Be verbose - lots of tasks, not vague
- Use consistent Category names for kanban grouping
- Filename format: 001_Task_name_here.md
```

---

## Import into Anytype

1. Settings → Import to Space
2. Select **Markdown**
3. Choose the `.zip` file
4. Create a Set filtered to imported objects
5. Switch view to Kanban, group by Category or Status

---

## Example Categories

- **Code**: Window Setup, OSM Parsing, Rendering, Export
- **Issues**: Bug, Enhancement, Refactor, Docs
- **Project**: Research, Design, Implementation, Testing, Deploy
