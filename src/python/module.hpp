// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file module.hpp
 * @brief The `stratum` Python module: what it is, who owns what, and how to embed it
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this exists
 *
 * `STRATUM_ENABLE_PYTHON` has defaulted ON since the project started and built
 * nothing: pybind11 was vendored, `pybind11::embed` was linked, and not one
 * symbol was bound. The option cost a libpython dependency and bought nothing.
 *
 * J1 makes it real. The surface bound here is enough to do a full job with no
 * window open -- parse an extract, build a document, extract blocks, subdivide
 * them, export geometry -- because that is what J3 (headless CLI) and J4
 * (in-engine runtime) stand on. Everything bound lives in stratum_core, so a
 * script never needs SDL, a device or a display.
 *
 * ### Two entry points, one body
 *
 * register_module() is the whole binding body and knows nothing about how the
 * module came to exist. bindings.cpp wraps it twice:
 *
 *   - `PYBIND11_EMBEDDED_MODULE(stratum, m)` -- for an interpreter this process
 *     starts itself. J4 and the test driver take this path.
 *   - `PYBIND11_MODULE(stratum, m)`, behind `STRATUM_PYTHON_EXTENSION_MODULE` --
 *     for a `stratum.so` a system `python3` imports. Nothing builds this yet; it
 *     is here so that adding an extension-module target later is a CMake change
 *     and not a source change.
 *
 * The two cannot be compiled into the same binary: `PYBIND11_MODULE` defines
 * `PyInit_stratum`, which an embedded interpreter would find on the executable
 * and prefer to the embedded registration, giving two live copies of every bound
 * type in one process. Hence the `#ifdef`, not both.
 *
 * ### Ownership: Python owns, or Python borrows with a leash
 *
 * A dangling reference from Python into freed C++ storage is a segfault with no
 * traceback, and pybind11 will happily let one be written. Three rules, applied
 * everywhere, and each is enforced at the binding rather than documented at the
 * user:
 *
 *   1. **Anything a script keeps is owned by Python.** `parse_osm()` returns
 *      ParsedOSMData BY VALUE. The OSMParser that produced it is never bound at
 *      all, because `OSMParser::get_data()` hands out a reference into a parser
 *      the script could drop on the next line.
 *   2. **Anything read out of a mutable container is a COPY.** `Document.layer()`
 *      returns a copied Layer, not the `Layer*` that `LayerTree::find()` returns.
 *      That pointer dies when the layer is deleted, and "delete the layer, then
 *      read the object you took out of it earlier" is the most natural thing in
 *      the world to write.
 *   3. **What must borrow, borrows with `reference_internal`.** The one place a
 *      reference crosses is `Document.attribute_keys` and friends, where
 *      pybind11's keep_alive ties the borrower's life to the document's.
 *
 * The sharpest case is osm::SceneObject, which holds a RAW `const Mesh*`. Binding
 * it directly would let a script write `obj.mesh = m; del m; export(...)` and get
 * a crash. So the Python type named `stratum.SceneObject` is OwnedSceneObject
 * below -- it owns its Mesh by value -- and the raw-pointer form is built inside
 * export_scene() and destroyed before it returns. A script cannot hold one.
 *
 * ### The GIL
 *
 * Every call that does real work -- a parse, a face traversal, a subdivision, an
 * export, a rule parse -- releases the GIL around the C++ body, so a Python UI or
 * a worker pool calling in does not freeze. Two consequences that are easy to get
 * wrong:
 *
 *   - **No bound function takes a `std::string_view` or a borrowed buffer.** A
 *     view points into the Python object it was cast from; releasing the GIL
 *     while holding one lets another thread collect the object under it. Bound
 *     signatures take `std::string` and `std::filesystem::path` BY VALUE, which
 *     pybind11 materialises while the GIL is still held.
 *   - **The objects themselves are not thread-safe**, exactly as CommandStack and
 *     LayerTree say. Releasing the GIL means two Python threads CAN be inside one
 *     Document at once, and must not be. One thread per document.
 *
 * ### Errors
 *
 * Every refusal arrives as a Python exception carrying the C++ message. A failed
 * `Document.load()` raises `stratum.StratumError` with
 * `DocumentIoResult::error` verbatim -- "layers.roots[2].own_transform.scale:
 * expected 3 numbers" is worth more to a script author than a `False` return
 * that the script then ignores. A missing file raises `FileNotFoundError`, so
 * ordinary Python `except FileNotFoundError` works.
 *
 * The exception to the rule is a genuine yes/no question. `Document.undo()`
 * returns False when the history is empty, because "is there anything to undo"
 * is a query, not a refused operation. Anything naming a handle that does not
 * exist raises.
 *
 * ### Naming
 *
 * snake_case methods, properties for cheap reads, and enum members in UPPER_CASE
 * per PEP 8. A read that COPIES is a method and not a property:
 * `data.roads()` copies every road, and a property that silently costs O(n) turns
 * `for r in data.roads` into an O(n^2) loop nobody can see. Cheap scalars --
 * `data.road_count`, `doc.dirty` -- stay properties.
 */

#pragma once

// This header is only meaningful in a build that has pybind11. bindings.cpp
// guards its own include of it; nothing else in stratum_core includes it at all,
// so the Python dependency stays in exactly two translation units.
#include <pybind11/pybind11.h>

#include "osm/scene_export.hpp"
#include "renderer/mesh.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace stratum::python {

/// Name the module registers under, in both the embedded and extension forms.
inline constexpr const char* kModuleName = "stratum";

// ============================================================================
// Errors
// ============================================================================

/**
 * @brief Every refusal Stratum reports to Python
 *
 * Registered as `stratum.StratumError`, derived from Python's `RuntimeError` so
 * that a script which catches `RuntimeError` catches these too and a script that
 * wants to be specific can be.
 *
 * Derives from std::runtime_error rather than being a bare pybind11 exception
 * object so that C++ code in J3 and J4 can throw it from outside a binding
 * lambda and still have it arrive at Python correctly typed.
 */
class StratumError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// ============================================================================
// Scene objects Python may hold
// ============================================================================

/**
 * @brief osm::SceneObject with the mesh OWNED rather than pointed at
 *
 * osm::SceneObject holds `const Mesh* mesh` and says the mesh must outlive the
 * export. That contract is keepable in C++, where the caller builds the meshes
 * and the objects in the same scope. It is not keepable from Python: the
 * lifetime of the Mesh a script assigned is whatever the garbage collector
 * decides, and the failure mode is a segfault inside the exporter with a Python
 * traceback that points at the `export_scene` line and explains nothing.
 *
 * So the bound type owns its mesh. The copy is the price: one Mesh per object,
 * paid when the SceneObject is constructed, and it buys an API where
 *
 * @code
 *     obj = stratum.SceneObject(mesh, stratum.SceneObjectKind.BUILDING)
 *     del mesh
 *     stratum.export_scene([obj], out_dir)   # still correct
 * @endcode
 *
 * is simply true. export_scene() builds the pointer-carrying osm::SceneObject
 * array itself, inside the call, from objects it has pinned with a strong Python
 * reference for the duration.
 */
struct OwnedSceneObject {
    Mesh mesh;
    osm::SceneObjectKind kind = osm::SceneObjectKind::Unknown;
    std::string name;
    int64_t osm_id = 0;
    std::string layer;
    std::vector<std::pair<std::string, std::string>> metadata;
};

// ============================================================================
// Registration
// ============================================================================

/**
 * @brief Define every Stratum type and function on @p m
 *
 * Idempotent per interpreter only in the sense that pybind11 is: calling it
 * twice on the same module in one interpreter re-registers types and pybind11
 * will complain. It is called once, by whichever macro in bindings.cpp is
 * active.
 */
void register_module(pybind11::module_& m);

/**
 * @brief Pull bindings.cpp out of the stratum_core archive
 *
 * `PYBIND11_EMBEDDED_MODULE` works by running a static constructor that adds the
 * module to pybind11's embedded-module table. In a STATIC LIBRARY that
 * constructor is in an archive member, and a linker only extracts an archive
 * member that something references. Nothing references bindings.cpp -- the only
 * consumer is `import stratum`, which happens at run time and is invisible to
 * the linker -- so the object file is dropped, the constructor never runs, and
 * the failure is `ModuleNotFoundError: No module named 'stratum'` from a build
 * that linked cleanly and says pybind11 is enabled.
 *
 * Calling this before starting the interpreter forces the extraction. It does
 * nothing else and always returns true.
 *
 * @note An alternative is `--whole-archive` (or `/WHOLEARCHIVE`) on
 *       stratum_core, which pulls in every unreferenced object in the library to
 *       fix one. A function call is cheaper and is visible at the call site,
 *       where somebody deleting it will at least see what it claims to do.
 */
bool module_linked();

} // namespace stratum::python
