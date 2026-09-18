# Procedural generation and the rule engine

Covers `src/procgen`. Namespace `stratum::procgen`; the rule engine is
`stratum::procgen::rules`. All of it is in `stratum_core`, so none of it may
include SDL, ImGui or renderer headers — `renderer/mesh.hpp` is the one
exception, and it is a plain data struct.

Two unrelated things live here. **Terrain** (`noise`, `terrain_generator`,
`terrain_mesh_builder`, `terrain_tile_manager`, `terrain_carve`,
`heightmap_io`) predates the rule engine and is independent of it. **The rule
engine** (`src/procgen/rules/`) is Track D, and is the larger half: ~13,500
lines across 8 file pairs.

## The rule engine pipeline

```
rule text → Lexer → tokens → Parser → RuleFile (AST)
                                         ↓
                    generate(file, seed_shape, options, ops, fns)
                                         ↓
                   GenerationResult { terminals, diagnostics, log, stats }
                                         ↓
                              build_mesh() → Mesh → GPU
```

`generate()` (`src/procgen/rules/interpreter.hpp:544`) is the whole public
entry point. A file that parsed **with** errors may still be passed: the faults
are reported again at run time, at the line that hits them, and the rest runs.

| File pair | Owns |
|---|---|
| `lexer.*` | Text to tokens, and `Diagnostic` with its caret rendering |
| `ast.hpp` | Node types, `RuleFile`, and `kBuiltinOperations` — the name catalogue |
| `parser.*` | Tokens to AST, name resolution, error recovery |
| `shape.*` | `Shape`, `Scope`, the geometry ops, `shape_to_mesh()` |
| `interpreter.*` | The evaluator, the two registries, seeding, the caps |
| `op_split.*` | The `split` sizing solver and the slab cut |
| `op_comp.*` | Component decomposition and `select` |
| `op_control.*` | Attributes, tags, and the `random.*` functions |

The file headers are long and are the real documentation. This page is the map
over them.

## The two registries, and the closed set

Operations and functions are **registries**; statements are **not**.

- `OperationTable` — statement-position built-ins (`extrude(3);`).
- `FunctionTable` — expression-position built-ins (`sqrt`, `comp.count`,
  `random.range`).

The idiom a feature follows: expose `register_<feature>_operations(
OperationTable&)` and a convenience `<feature>_operations()` that returns a
pre-built table, as `op_control.hpp:381` and `op_comp.hpp:596` do. Copy a
table, register into the copy, pass it to `generate()`. **No feature adds an
operation by editing `interpreter.cpp`.**

Statements — `split`, `select`, `if`, `choose`, `scope` — are variant
alternatives in `ast.hpp` and are wired directly into `State::exec()`. That is
the design, not a gap: the whole population is three or four cases, and a
registry would buy indirection and cost a readable control flow. What makes a
new statement family cheap is two helpers in `interpreter.cpp`:

- **`begin_child()`** — the single place a child shape is born. The copy, the
  sibling index, the seed mix, the shape cap and the depth cap. A rule call, a
  split slab and a `select` component are the same act, differing only in
  content.
- **`run_statement_body()`** — `invoke_rule()` minus parameter binding and
  minus the move of the lexical frame base, because a statement body belongs to
  the rule that wrote it.

## `kBuiltinOperations` is a parse-time contract

`ast.hpp:641` holds a sorted array of 47 rows: name, min args, max args,
description. **Registering a handler is only half of adding an operation.** A
name absent from that array does not resolve, and the rule file does not parse.

Of the 47, 17 have handlers. The rest are catalogue rows whose implementation
has not landed, and calling one is not a silent no-op — it reports
`operation 'roof' is not implemented in this build` at the line that called it.
That is deliberate: the catalogue is the published surface of the language, and
a row is a promise with a date on it.

Two traps:

- **`static_assert(builtin_operations_are_sorted())`** compares with `<` on
  `string_view`, which is an **ASCII byte** compare, not a locale collation.
  `_` is `0x5F` and sorts before every lowercase letter, so `set_pivot` comes
  before `setback` and `trim` comes before `wall_panel`. Checking the order
  with `sort -c` gives a false failure; use `LC_ALL=C sort -c`.
- **`CallStmt::operation` is an index into the array.** It is computed at parse
  time and never persisted, so inserting a row is safe — but nothing may cache
  that index across a parse.

## Geometry invariants

Four rules that `shape.cpp` keeps and every consumer reads. All four were
broken at some point, and none of the ~1,500 tests over the file noticed.

**Material is on the left of every directed edge.** The outer ring is wound
counter-clockwise and every hole clockwise, and that is the entire point of
the opposite winding: it means ONE rule covers both rings. The left normal
`{-dir.y, dir.x}` points into the material either way, so moving every edge
along it shrinks the material — which draws the outline in AND pushes a hole
boundary out. An inset of the face **grows** its holes. A per-ring sign here is
always a bug; it made `taper` on a courtyard footprint slope the hole wall the
wrong way and overhang the void.

**Handedness lives in the frame, and only in the frame.** `mirror_scope()`
makes `Scope::axes` left-handed on purpose and `orthonormalise()` preserves
that on purpose. The local face loops still keep their contract — wound
counter-clockwise seen from outside — so a local signed volume stays positive
whatever the frame is doing. `reframe()` re-winds on a handedness change to
make that true, and `append_shape_to_mesh()` emits the reversed triple when the
frame is left-handed. **Both are needed**: the two orders
(`extrude; mirror_scope` and `mirror_scope; extrude`) fail for opposite
reasons, so a patch to one alone cancels itself out.

**Everything planar goes through Clipper2 on a 1e-5 integer grid.** That is
`kClipperScale`, 10 µm, and it is the real resolution of `offset`, `setback`
and anything else that round-trips through `to_path()`. A distance below one
count moves nothing, so both operations refuse one rather than report success
having done nothing. Two consequences worth knowing: a Clipper round-trip with
no offset at all can still move a coordinate by up to a count, and a partition
assertion (`inner + border == original`) holds to about 1e-4 m² on a footprint
that is not grid-aligned, not to 1e-9. Every test dimension that is a round
number hides this.

**The scope is a tight box.** After any operation,
`min(positions) == (0,0,0)` and `max(positions) == scope.size`, with
orthonormal axes. `refit_scope()` maintains it and is called on every path that
touches geometry. The documented exception is a shape with NO geometry: its
scope is whatever was set, untouched, so a rule can build a frame and then
insert an asset into it. `scale` is the only operation that can set that
scope's size — it special-cases the empty shape for exactly this reason.

## Determinism

Same rule text, same seed, same input shape, byte-identical output on any
platform. Two rules make it hold, and both are load-bearing:

- **A shape's random state comes from its ADDRESS in the tree, never from a
  shared stream.** `Shape::seed_key` is mixed in `begin_child()` from the
  parent's key and either the sibling index (`kSaltChild`) or the component
  index (`kSaltComponent`). With a shared stream, adding one `choose` arm at
  the top of a file moves every shape generated after it, and a reproduction
  case stops reproducing. This is the same argument `osm/road/lots.cpp` makes.
- **Nothing is read out of a hash container.** Attributes are a `std::map`,
  both registries are `std::map`, constants evaluate in declaration order.

`GenerationResult::dump()` is the canonical text rendering the determinism
tests assert on. **It has been geometry-blind before** — a determinism suite
resting on a dump that did not include the thing under test passes for any
implementation. Check what `dump()` actually prints before trusting a test
built on it.

## Scoping

Names are **lexically** scoped. A rule body sees its own parameters, its own
`let`s, and the file's attributes and constants — and nothing of its caller. So
`rule A(w: float) { B(); }` does not let `B` read `w`. Call arguments are
evaluated in the **caller's** frame, before the callee exists, which is what
keeps `Floor(height - 1)` meaning the caller's `height`.

A `split` part body and a `select` arm body are **not** rules and do not move
the frame: they see the enclosing rule's bindings, which is what makes
`split(y) { storey : { Floor(storey); } }` work, and their own `let`s are
dropped at the end of the body.

This was dynamically scoped when D2 first landed and was caught in review. A
rule whose meaning depends on its caller cannot be read on its own, and the
fault surfaces inside the callee with nothing pointing at the caller.

## Runaway rules

Two caps in `InterpreterLimits`, both **reported**, never silent:

- A **depth cap**. A call that would exceed it emits the child as a terminal
  rather than refusing it, so a runaway tower shows truncated output — which
  points at the mistake — instead of an empty viewport, which does not.
- A **shape cap**, a total, catching a rule that recurses shallowly and
  branches widely. A refused child does not count against its parent, so a
  parent whose every child was refused still emits its own geometry.

Both are `Severity::Error`, so `GenerationResult::ok()` is false and no caller
mistakes truncated output for finished output.

## A rule file that works, end to end

This is `tests/procgen/test_rule_statements.cpp`, and it is the fastest way to
see the whole engine do something:

```
rule Main   { extrude(10.0); select face { front : { Facade(); } } }
rule Facade { split(y) { 4.0 : { Shopfront(); }
                         repeat { 2.0 : { UpperFloor(); } } } }
rule UpperFloor { split(x) { repeat { 1.5 : { Bay(); } } } }
rule Bay    { split(x) { 0.4 : { Pier(); } 0.6 : { Window(); } ~1.0 : { Pier(); } } }
```

37 terminals, 12 windows, 14.4 m² of glass, window centres at
`(1.5·bay + 0.7, 4 + 2·floor + 1, 4)`. The `~` floating size is what makes the
last pier absorb the remainder, and it is what makes CityEngine facades work.

## Tests

Eight suites, in `tests/procgen/`:

| Suite | Tests | File |
|---|---|---|
| `RuleParser` | 116 | `test_rule_parser.cpp` |
| `Interpreter` | 80 | `test_interpreter.cpp` |
| `OpComp` | 53 | `test_op_comp.cpp` |
| `HeightmapIO` | 52 | `test_heightmap_io.cpp` |
| `OpControl` | 42 | `test_op_control.cpp` |
| `OpSplit` | 41 | `test_op_split.cpp` |
| `RuleStatements` | 35 | `test_rule_statements.cpp` |
| `TerrainCarve` | 24 | `test_terrain_carve.cpp` |

The framework is `tests/framework.hpp`, **not** googletest. The macros are
exactly `TEST(Suite, snake_case_name)`, `CHECK`, `CHECK_TRUE`, `CHECK_FALSE`,
`CHECK_EQ`, `CHECK_NEAR`, plus `skip_test(reason)` and `register_teardown()`.
There is no `ASSERT_*`, no `CHECK_LT`, and no `<< message` streaming; write an
ordering as `CHECK((a) < (b))`.

To build and run one suite without touching the shared `build/` tree — which is
what every agent working in parallel here must do — compile directly. **The
include flags must be separate arguments**; in zsh an unquoted `$INC` holding
several `-I` flags passes as one argument and the compile fails in a way that
looks like a source error:

```bash
INC=(-Isrc -Itests -Iexternal -Iexternal/glm -Iexternal/spdlog/include
     -Iexternal/earcut/include -Iexternal/json/include -Iexternal/stb
     -Iexternal/Clipper2/CPP/Clipper2Lib/include)
g++ -std=c++20 -O1 -Wall -Wextra "${INC[@]}" \
    tests/framework.cpp tests/procgen/test_op_split.cpp \
    src/procgen/rules/lexer.cpp src/procgen/rules/parser.cpp \
    src/procgen/rules/shape.cpp src/procgen/rules/interpreter.cpp \
    src/procgen/rules/op_split.cpp src/procgen/rules/op_comp.cpp \
    src/procgen/rules/op_control.cpp \
    external/Clipper2/CPP/Clipper2Lib/src/*.cpp \
    -o "$SCRATCH/suite" && "$SCRATCH/suite"
```

`shape.cpp` calls Clipper2 for the outline offsets, so
`external/Clipper2/CPP/Clipper2Lib/src/*.cpp` has to be on the command line.
Leaving it off gives a wall of `undefined reference to Clipper2Lib::` from the
linker, which reads like a broken source tree and is not. List the rule sources
explicitly rather than globbing `src/procgen/rules/*.cpp`: while another agent
is mid-edit, a glob picks up a half-written file.

Verified 2026-09-18: that command builds `OpSplit` and prints
`41 passed, 0 failed`.

## The recurring defect

**Tests that cannot fail are this subsystem's most common review finding** —
found in nine consecutive review rounds, including a determinism suite resting
on a geometry-blind `dump()`. Before believing a suite, break the
implementation it covers and confirm the suite notices.
