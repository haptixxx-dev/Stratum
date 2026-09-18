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
| `14_materials.rule` | 14 × 10 | Six materials on one building. **Needs PBR shader mode.** |

## Running rules on an imported city

`13_from_osm_tags.rule` is the one to point at an OSM import rather than at a
rectangle. In the rule editor set **Seed from** to **Imported buildings**; the
rule then runs once per footprint in the extract, seeded from the real outline
with the feature's tags attached as shape attributes.

| attribute | type | |
|---|---|---|
| `osm.id` | int | the OSM way or relation id |
| `osm.height` | float | metres, as mapped or as defaulted on import |
| `osm.levels` | int | storeys |
| `osm.type` | string | `house`, `apartments`, `retail`, `warehouse`, … |
| `osm.roof` | string | `flat`, `gable`, `hip`, `pyramid`, `shed`, `dome` |
| `osm.name` | string | only when the feature is named |

`osm.roof` is spelled the way `roof()` spells its kinds — OSM's *skillion* and
*pyramidal* are translated at the seam, so a rule writes
`roof(attrs.get("osm.roof", "hip"))` with no lookup table of its own. The full
list is in `src/osm/rule_seed.hpp`.

Measured against the Lucan extract: 24,576 buildings, of which 20,598 are
tagged `house` and 16,287 have a flat roof. About one in eight refuses a
pitched roof because real outlines have spike corners — issue #133, waiting on
`cleanup()`.

## Materials

**`14_materials.rule` is the example.** A brick building with a stone plinth,
a glazed shopfront, a metal-clad attic storey and a slate roof — six materials
on one building. Turn on **Render Settings → Shader Mode → PBR** first, or it
all draws grey.

`material(slot, variant)` tags the current shape's faces. The slots are
`renderer/mesh.hpp`'s, lower-cased: `default`, `asphalt`, `concrete`, `curb`,
`sidewalk`, `markings`, `gravel`, `dirt`, `grass`, `bridgedeck`, `parapet`,
`wall`, `roof`.

Three things worth knowing before you reach for it:

- **The facade operations already assign materials.** `wall_panel()`,
  `window()` and `door()` tag each part with its own variant of the `wall`
  slot — panel 0, reveal 1, frame 2, glass 3, sill 4 — so the glass is already
  separable from the brick without a single `material()` call.
- **Order matters against them.** A `material()` call *before* `wall_panel()`
  is thrown away, because the facade operation tags its own output.
  `extrude`, `split` and `roof` leave the material alone; only the facade
  family has this. Put `material()` after.
- **Nothing is textured in `ShaderMode::Simple`.** The simple shader has no
  samplers and no material uniform block, so the whole city draws flat grey
  however carefully its materials are set. Render Settings → Shader Mode →
  PBR. The rule editor says so when a preview exists and the mode is Simple.

UVs are already a planar projection **in metres** in each face's own basis,
with v running up the wall, so a texture tiles at real-world scale with no
projection call. D8 (#45) adds control over that projection; it is not needed
to have one.

## The recursive four

These are the ones worth reading for what the language can actually do. None
of them has a loop, a counter of floors, or a list of buildings — the shape of
the output falls out of a rule calling itself.

| File | Seed | Output | What it is for |
|---|---|---|---|
| `09_twisting_tower.rule` | 10 × 10 | 377 terminals, 7.5k tris, 49.8 m | A helix. `rotate` vs `rotate_scope` is the whole trick |
| `10_recursive_district.rule` | 120 × 80 | 3539 terminals, 37k tris, 23 buildings | A district that subdivides itself into plots |
| `11_fractal_tower.rule` | 32 × 32 | 3907 terminals, 76k tris, 40.8 m | Quarters itself and branches; 139 terraces where branches stopped |
| `12_stacked_modules.rule` | 24 × 14 | 195 terminals, 2.5k tris | Habitat 67. Modules that cantilever out past their own plot |

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

**Recursion is how you write a tall thing.** There is no loop. `Level` calls
itself with one less to go and the building is however deep the recursion
went; change one attribute from 14 to 40 and you have a skyscraper. The depth
and shape caps in `InterpreterLimits` sit behind that, and both report an
Error rather than truncating quietly.

**A recursion that shrinks its own input must stop on the GEOMETRY.** A
counter assumes every branch shrinks at the same rate, and a split with a
floating part does not. `11_fractal_tower.rule` at depth 12 is the same
40.8 m tower as at depth 5 — 4336 terminals against 3907 — because every
branch has already quartered itself past the size floor. `10_recursive_district.rule` says the same thing from the
other side: subdivision always makes a few slivers, and `offset(-1.5)` on a
sliver removes the whole shape and reports an error.

**`window()` is what makes a window.** An empty `rule Window {}` marks a region
of the wall and nothing more, so every terminal is a coplanar quad and the
building renders as a flat box. `window()` cuts the opening and gives it a
reveal, a frame and a sill. The difference on one facade is 148 vertices
against 1540.

## Limits you will meet

- **`cleanup()` repairs a footprint; it does not simplify one.** It merges
  points within a tolerance, drops spurs that double back, and drops collinear
  vertices — all of which enclose no area, so the footprint is unchanged. For
  fewer vertices at the cost of shape, that is `reduce()`, a different row.
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
- **A face that `select face` does not name is DROPPED.** Omit the `top` arm
  and you get a building with walls and no roof, with nothing reported —
  because nothing went wrong. The rule asked for four walls and got four
  walls. The `Examples` suite lints for this.
- **`wall_panel()` with no thickness is one quad.** Two triangles, one-sided,
  invisible from below or edge-on, and with no underside to export. A flat roof
  wants `wall_panel(0.0, 0.25)`, which is a closed slab of twelve.
- **`comp.size` does not measure in the same axes `split` divides.** `split(z)`
  uses the shape scope's z. `comp.size` measures a COMPONENT in its own frame,
  where a face's two in-plane axes are x and y and z is its thickness. On a
  ground footprint `comp.size("all","z")` is therefore **0**, and a recursion
  guarded on it terminates immediately with no error anywhere.
  `10_recursive_district.rule` has the table.
