# Example rule files

Eight rule files that run. Every one of them is executed by the `Examples`
test suite on each build, so an example that stops working fails CI rather
than sitting here misleading the next reader.

Open one in the rule editor panel (**View → Rule Editor → Load...**), or run it
headlessly through `full_operations()`.

| File | Seed that suits it | What it is for |
|---|---|---|
| `01_office_tower.rule` | 30 × 20 | `podium()` — a base with an inset tower stepping back above it, curtain-walled |
| `02_terrace.rule` | 24 × 12 | A row of houses whose gable ridges turn with each plot's own long axis |
| `03_courtyard_block.rule` | 60 × 40 | `courtyard()` — a perimeter block, hollow in the middle, glazed on both sides |
| `04_townhouse.rule` | 7 × 10 | One house in detail. **Read this one first.** |
| `05_tower_and_spire.rule` | 30 × 14 | Stacked masses: four roof kinds — pyramid, gable, dome, shed — in one file |
| `06_warehouse.rule` | 48 × 24 | One enormous span: structural bays, roller doors, a continuous clerestory |
| `07_by_attribute.rule` | 16 × 12 | One file, four building types, chosen by an `attr`. Copy this for a rule pack. |
| `08_stochastic_street.rule` | 60 × 14 | Variety that reproduces exactly. `choose`, `random.*`, and why the seeding works |

## Three things the examples are trying to teach

**The `~` floating size is what makes a facade work.** A bay is `0.4` of pier,
`0.6` of window and `~1.0` of pier; the floating part absorbs whatever the
fixed parts leave, so the same rule fits a 2 m bay and a 3 m bay. Every
example uses it.

**A rule cannot ask where it is.** There is no function that reads the scope's
position, so a rule has no way to know it is on the ground floor or the
eighteenth. Position comes from WHERE IT SITS IN THE SPLIT: the ground floor is
the ground floor because it is the first part of `split(y)`. Put `door()`
somewhere else in that split and you get a door into thin air.
`01_office_tower.rule` says why it has no entrance for exactly this reason.

**`window()` is what makes a window.** An empty `rule Window {}` marks a region
of the wall and nothing more, so every terminal is a coplanar quad and the
building renders as a flat box. `window()` cuts the opening and gives it a
reveal, a frame and a sill. The difference on one facade is 148 vertices
against 1540.

## Limits you will meet

- **No pitched roof over a courtyard.** The straight skeleton does not handle
  interior rings, so `shed` is the only kind that carries a hole. The others
  refuse rather than roof over land that does not exist — issue #127.
- **`wall_panel` will not project.** It refuses a negative inset, so a string
  course can be a thicker band in the same plane but not a cornice that juts
  out.
- **`taper` insets by its height on every edge.** A 10 m taper on a 6 m tower
  collapses the outline and is refused. For a spire, use a steep
  `roof("pyramid", 72)` instead, which takes a pitch and cannot collapse.
- **A reveal deeper than the wall is cut back**, with a warning naming both
  numbers.
