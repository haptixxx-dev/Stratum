// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file bindings.cpp
 * @brief The body of the `stratum` Python module
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Read module.hpp first: it states the ownership, GIL, error and naming rules
 * this file implements. What is written here is the application of those rules,
 * and the comments below say what each one is protecting against rather than
 * repeating the rule.
 *
 * ### Why the whole file is inside one `#ifdef`
 *
 * `STRATUM_ENABLE_PYTHON` is an option, and a build with it OFF must not merely
 * skip the Python parts -- it must not need pybind11 or `Python.h` to exist at
 * all. Guarding the translation unit rather than the CMake source list means
 * this file can be listed in stratum_core UNCONDITIONALLY: only the compile
 * definition and the `pybind11::embed` link are conditional. A conditional
 * source list is the version of this that rots, because the `if()` around it and
 * the `#ifdef` inside it drift apart and nobody notices until a platform build
 * fails.
 *
 * ### Layout
 *
 * One `register_*` function per subsystem, each called from register_module() in
 * the order a script would meet them: errors, geometry, OSM, the scene, the
 * street graph, the rule language, export. They are static and take
 * `py::module_&` so the reading order and the registration order are the same,
 * which matters -- pybind11 needs a type registered before a signature mentions
 * it, or the docstring says `handle` and the argument arrives unconverted.
 */

#ifdef STRATUM_ENABLE_PYTHON

#include "python/module.hpp"

#include <pybind11/embed.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "osm/mesh_builder.hpp"
#include "osm/parser.hpp"
#include "osm/road/blocks.hpp"
#include "osm/road/lots.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/scene_export.hpp"
#include "osm/types.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/parser.hpp"
#include "renderer/mesh.hpp"
#include "scene/attributes.hpp"
#include "scene/document.hpp"
#include "scene/layer.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// ============================================================================
// Vector type casters
// ============================================================================
//
// glm::dvec2 / dvec3 / vec3 cross as plain Python tuples rather than as bound
// classes, and that is an ownership decision, not a convenience one. A bound
// glm::dvec2 handed out of `block.ring` would either copy every point into a
// Python object (slow, and a list of a thousand bound objects is a thousand
// allocations) or hand back a reference into a vector the next call can
// reallocate. A tuple is neither: it is an immutable copy that owns itself, and
// there is no way to end up holding one that points at freed storage.
//
// The cost is that `p.x` does not work -- a tuple has no `x` -- which fails
// loudly with AttributeError rather than quietly with the wrong number.

PYBIND11_NAMESPACE_BEGIN(PYBIND11_NAMESPACE)
PYBIND11_NAMESPACE_BEGIN(detail)

/**
 * @brief Read an N-component vector out of any Python sequence of numbers
 *
 * `str` and `bytes` are rejected up front even though both satisfy
 * PySequence_Check: without that, the two-character string "ab" is a sequence of
 * length 2 and would be offered to the float conversion, and a three-character
 * one would be offered as a dvec3. It would fail on the element conversion
 * today, but only by accident -- b"\x01\x02" indexes to ints, converts happily,
 * and would silently become the point (1, 2).
 */
template <typename Vec, int N>
bool stratum_load_vec(handle src, Vec& out) {
    PyObject* obj = src.ptr();
    if (obj == nullptr || PyUnicode_Check(obj) || PyBytes_Check(obj) || PyByteArray_Check(obj)) {
        return false;
    }
    if (!PySequence_Check(obj)) {
        return false;
    }
    const Py_ssize_t size = PySequence_Size(obj);
    if (size != static_cast<Py_ssize_t>(N)) {
        // PySequence_Size sets an exception for an object that lies about being
        // a sequence. A caster must not leave one set: pybind11 tries the next
        // overload after a failed load, and a stale error surfaces there instead.
        PyErr_Clear();
        return false;
    }
    for (int i = 0; i < N; ++i) {
        object item = reinterpret_steal<object>(PySequence_GetItem(obj, i));
        if (!item) {
            PyErr_Clear();
            return false;
        }
        const double component = PyFloat_AsDouble(item.ptr());
        if (component == -1.0 && PyErr_Occurred() != nullptr) {
            PyErr_Clear();
            return false;
        }
        out[i] = static_cast<typename Vec::value_type>(component);
    }
    return true;
}

template <>
struct type_caster<glm::dvec2> {
    PYBIND11_TYPE_CASTER(glm::dvec2, const_name("tuple[float, float]"));

    bool load(handle src, bool) { return stratum_load_vec<glm::dvec2, 2>(src, value); }

    static handle cast(const glm::dvec2& v, return_value_policy, handle) {
        return pybind11::make_tuple(v.x, v.y).release();
    }
};

template <>
struct type_caster<glm::dvec3> {
    PYBIND11_TYPE_CASTER(glm::dvec3, const_name("tuple[float, float, float]"));

    bool load(handle src, bool) { return stratum_load_vec<glm::dvec3, 3>(src, value); }

    static handle cast(const glm::dvec3& v, return_value_policy, handle) {
        return pybind11::make_tuple(v.x, v.y, v.z).release();
    }
};

template <>
struct type_caster<glm::vec3> {
    PYBIND11_TYPE_CASTER(glm::vec3, const_name("tuple[float, float, float]"));

    bool load(handle src, bool) { return stratum_load_vec<glm::vec3, 3>(src, value); }

    static handle cast(const glm::vec3& v, return_value_policy, handle) {
        return pybind11::make_tuple(static_cast<double>(v.x), static_cast<double>(v.y),
                                    static_cast<double>(v.z))
            .release();
    }
};

PYBIND11_NAMESPACE_END(detail)
PYBIND11_NAMESPACE_END(PYBIND11_NAMESPACE)

namespace py = pybind11;

namespace {

using stratum::python::OwnedSceneObject;
using stratum::python::StratumError;

namespace osm = stratum::osm;
namespace road = stratum::osm::road;
namespace rules = stratum::procgen::rules;
namespace scene = stratum::scene;

namespace fs = std::filesystem;

// ============================================================================
// Shared helpers
// ============================================================================

/**
 * @brief Raise Python's own FileNotFoundError, not StratumError
 *
 * A script that wants to handle a missing input writes `except FileNotFoundError`
 * because that is what every other Python API raises. Mapping it onto
 * StratumError would make Stratum the one library where that idiom does not
 * work, and the alternative -- making the caller read the message text -- is not
 * an API.
 */
[[noreturn]] void raise_file_not_found(const std::string& context, const fs::path& path) {
    const std::string message = context + ": no such file: " + path.string();
    PyErr_SetString(PyExc_FileNotFoundError, message.c_str());
    throw py::error_already_set();
}

/**
 * @brief Raise StratumError carrying the numbers the refused export DID produce
 *
 * A destination that refused the write has to raise -- see module.hpp -- but
 * throwing the SceneExportStats away with it costs the caller six of its eight
 * counts. scene_export.hpp is explicit that a PARTIALLY written export still
 * counts what reached disk: `chunks`, `objects`, `vertices`, `triangles`,
 * `files` and `written_files` describe the half of a city that was written, and
 * a headless job that stopped halfway has nothing else to report with. Text in
 * a message is not that: a script cannot act on it without parsing English.
 *
 * So the whole stats object rides on the exception INSTANCE as `.stats`, and the
 * message stays the summary it already was. `except stratum.StratumError as e:
 * e.stats.written_files` is then a real recovery path.
 *
 * Built by hand rather than by throwing StratumError, because pybind11's
 * exception translator constructs the Python instance itself and never hands it
 * back, so there is no instance to attach anything to.
 *
 * The type is looked up on the module at raise time rather than cached: a
 * `py::object` with static storage duration is decref'd after the interpreter
 * that owns it has been finalised, which is a crash during exit rather than a
 * failure anybody can read. This is the error path, so one dict lookup in
 * sys.modules costs nothing.
 */
[[noreturn]] void raise_export_refused(const std::string& message,
                                       const osm::SceneExportStats& stats) {
    py::object error_type =
        py::module_::import(stratum::python::kModuleName).attr("StratumError");
    py::object error = error_type(message);
    // COPY, explicitly: `stats` is a local of the caller and is gone by the time
    // the script reads the attribute off the exception it caught.
    error.attr("stats") = py::cast(stats, py::return_value_policy::copy);
    PyErr_SetObject(error_type.ptr(), error.ptr());
    throw py::error_already_set();
}

/// Turn a negative Python index sentinel into LayerTree's kAppend.
///
/// kAppend is `size_t(-1)`, which as a Python default argument renders as
/// 18446744073709551615 in every docstring and help() page. -1 is what a Python
/// caller writes for "at the end", so that is what the signature takes.
[[nodiscard]] size_t sibling_index_from_python(int64_t index) {
    return index < 0 ? scene::kAppend : static_cast<size_t>(index);
}

/**
 * @brief Convert one Python value into an AttributeValue
 *
 * The bool test comes FIRST and that ordering is load-bearing: in Python `bool`
 * is a subclass of `int`, so `isinstance(True, int)` is true and an int-first
 * test would store `True` as the double 1.0. The attribute would then read back
 * as a float, `AttributeStore::resolve_as(..., Bool)` would report TypeMismatch,
 * and the rule that asked for a boolean would silently take its fallback.
 *
 * An empty list is REFUSED rather than guessed at. AttributeValue is typed --
 * BoolArray, DoubleArray and StringArray are three different types -- and an
 * empty Python list carries no element type at all. Picking one would make
 * `set_attribute(o, "k", [])` followed by `append` behave differently depending
 * on a choice the script never made.
 */
[[nodiscard]] scene::AttributeValue value_from_python(const py::handle& value) {
    // bool BEFORE int/float, and not the other way round. `isinstance(True, int)`
    // is true in Python, so the number test matches a bool and the value is
    // stored as the double 1.0. The first run of this file caught exactly that.
    if (py::isinstance<py::bool_>(value)) {
        return scene::AttributeValue::from_bool(value.cast<bool>());
    }
    if (py::isinstance<py::float_>(value) || py::isinstance<py::int_>(value)) {
        return scene::AttributeValue::from_double(value.cast<double>());
    }
    if (py::isinstance<py::str>(value)) {
        return scene::AttributeValue::from_string(value.cast<std::string>());
    }
    if (py::isinstance<py::list>(value) || py::isinstance<py::tuple>(value)) {
        const auto items = py::reinterpret_borrow<py::sequence>(value);
        const size_t count = py::len(items);
        if (count == 0) {
            throw py::value_error(
                "an empty list has no element type; pass a non-empty list, or clear the attribute");
        }

        const py::handle first = items[0];
        if (py::isinstance<py::bool_>(first)) {
            scene::AttributeValue::BoolArray out;
            out.reserve(count);
            for (size_t i = 0; i < count; ++i) {
                const py::handle item = items[i];
                if (!py::isinstance<py::bool_>(item)) {
                    throw py::type_error("list element " + std::to_string(i)
                                         + " is not a bool; a list attribute must be one type");
                }
                out.push_back(item.cast<bool>());
            }
            return scene::AttributeValue::from_bools(std::move(out));
        }
        if (py::isinstance<py::str>(first)) {
            scene::AttributeValue::StringArray out;
            out.reserve(count);
            for (size_t i = 0; i < count; ++i) {
                const py::handle item = items[i];
                if (!py::isinstance<py::str>(item)) {
                    throw py::type_error("list element " + std::to_string(i)
                                         + " is not a str; a list attribute must be one type");
                }
                out.push_back(item.cast<std::string>());
            }
            return scene::AttributeValue::from_strings(std::move(out));
        }
        if (py::isinstance<py::float_>(first) || py::isinstance<py::int_>(first)) {
            scene::AttributeValue::DoubleArray out;
            out.reserve(count);
            for (size_t i = 0; i < count; ++i) {
                const py::handle item = items[i];
                // bool again: [True, 1.0] must not become [1.0, 1.0].
                if (py::isinstance<py::bool_>(item)
                    || !(py::isinstance<py::float_>(item) || py::isinstance<py::int_>(item))) {
                    throw py::type_error("list element " + std::to_string(i)
                                         + " is not a number; a list attribute must be one type");
                }
                out.push_back(item.cast<double>());
            }
            return scene::AttributeValue::from_doubles(std::move(out));
        }
        throw py::type_error("unsupported list element type for an attribute value");
    }

    throw py::type_error("unsupported attribute value type; expected bool, float, str, or a "
                         "non-empty list of one of those");
}

/// The inverse. Never returns None for a value that exists: a missing attribute
/// is None, and a present one is always a real Python value.
[[nodiscard]] py::object value_to_python(const scene::AttributeValue& value) {
    switch (value.type()) {
        case scene::AttributeType::Bool:
            return py::cast(*value.as_bool());
        case scene::AttributeType::Double:
            return py::cast(*value.as_double());
        case scene::AttributeType::String:
            return py::cast(*value.as_string());
        case scene::AttributeType::BoolArray:
            return py::cast(*value.as_bool_array());
        case scene::AttributeType::DoubleArray:
            return py::cast(*value.as_double_array());
        case scene::AttributeType::StringArray:
            return py::cast(*value.as_string_array());
    }
    return py::none();
}

/// Validate an object handle at the boundary so that every deeper call can
/// assume it. A stale handle reaching AttributeStore is not a crash -- the
/// generation check catches it -- but it is a silent no-op, and a script that
/// keeps writing to a destroyed object should hear about it on the first write
/// and not on the empty export three hundred lines later.
void require_live_object(const scene::Document& document, scene::AttributeObject obj,
                         const char* context) {
    if (!document.attributes().is_valid(obj)) {
        throw StratumError(std::string(context) + ": object handle is stale (index "
                           + std::to_string(obj.index) + ", generation "
                           + std::to_string(obj.generation) + ")");
    }
}

/// Same idea for layers. LayerTree never reuses an id, so a stale id can only
/// ever mean "deleted", never "some other layer".
void require_layer(const scene::Document& document, scene::LayerId layer, const char* context) {
    if (!document.layers().contains(layer)) {
        throw StratumError(std::string(context) + ": no layer with id " + std::to_string(layer));
    }
}

/// Intern a key and set it through the command stack, or explain the refusal.
void set_attribute_target(scene::Document& document, const scene::AttributeTarget& target,
                          const py::handle& value, const char* context) {
    scene::AttributeValue converted = value_from_python(value);
    if (!scene::set_attribute(document.history(), document.attributes(), target,
                              std::move(converted))) {
        throw StratumError(std::string(context) + ": the store refused the write");
    }
}

// ============================================================================
// Rule-language summary
// ============================================================================

/**
 * @brief What a script gets back from parse_rules()
 *
 * Deliberately NOT the AST. rules::RuleFile is two arenas of variants plus four
 * declaration vectors whose ids index into those arenas, and every one of those
 * ids is meaningless without the RuleFile it came from. Binding it would create
 * exactly the borrow this module exists to avoid: a Python `Stmt` holding a
 * StmtId into a RuleFile that was collected.
 *
 * What a CLI (J3) actually needs from a parse is whether it succeeded and what
 * to print, so that is what crosses. Binding the AST is a separate decision to
 * make when something needs to walk it, and it will need a borrow-checked
 * design of its own.
 */
struct RuleParseSummary {
    bool ok = false;
    std::string report;                       ///< render_all(): every diagnostic, formatted
    std::vector<std::string> messages;        ///< One message per diagnostic, unformatted
    size_t error_count = 0;
    size_t warning_count = 0;
    std::vector<std::string> rule_names;
    std::vector<std::string> attribute_names;
    std::string start_rule;                   ///< "" when the file declares no @start rule
    std::string version;                      ///< From `version "..."`, "" when absent
};

// ============================================================================
// Errors and geometry
// ============================================================================

void register_errors(py::module_& m) {
    // register_exception installs the translator as well as the type, so a
    // StratumError thrown from anywhere inside a bound call -- including from
    // J3/J4 code that is not itself a lambda in this file -- arrives typed.
    py::exception<StratumError> error = py::register_exception<StratumError>(
        m, "StratumError", PyExc_RuntimeError);

    // register_exception takes no docstring, and a type with none is a type that
    // `help()` and every doc generator report as undocumented. Set it by hand
    // rather than leaving the one exception a script is told to catch as the one
    // name in the module with nothing to say about itself.
    error.attr("__doc__") =
        "Every refusal Stratum reports. Derives from RuntimeError, so an existing "
        "`except RuntimeError` catches it. A refused export also carries the "
        "SceneExportStats for whatever it managed to write, as `.stats`.";
}

// ============================================================================
// OSM
// ============================================================================

void register_osm_types(py::module_& m) {
    py::enum_<osm::RoadType>(m, "RoadType", "Classification taken from highway=*")
        .value("MOTORWAY", osm::RoadType::Motorway)
        .value("TRUNK", osm::RoadType::Trunk)
        .value("PRIMARY", osm::RoadType::Primary)
        .value("SECONDARY", osm::RoadType::Secondary)
        .value("TERTIARY", osm::RoadType::Tertiary)
        .value("RESIDENTIAL", osm::RoadType::Residential)
        .value("SERVICE", osm::RoadType::Service)
        .value("FOOTWAY", osm::RoadType::Footway)
        .value("CYCLEWAY", osm::RoadType::Cycleway)
        .value("PATH", osm::RoadType::Path)
        .value("UNKNOWN", osm::RoadType::Unknown);

    py::enum_<osm::BuildingType>(m, "BuildingType", "Classification taken from building=*")
        .value("RESIDENTIAL", osm::BuildingType::Residential)
        .value("COMMERCIAL", osm::BuildingType::Commercial)
        .value("INDUSTRIAL", osm::BuildingType::Industrial)
        .value("RETAIL", osm::BuildingType::Retail)
        .value("OFFICE", osm::BuildingType::Office)
        .value("APARTMENTS", osm::BuildingType::Apartments)
        .value("HOUSE", osm::BuildingType::House)
        .value("DETACHED", osm::BuildingType::Detached)
        .value("GARAGE", osm::BuildingType::Garage)
        .value("SHED", osm::BuildingType::Shed)
        .value("CHURCH", osm::BuildingType::Church)
        .value("SCHOOL", osm::BuildingType::School)
        .value("HOSPITAL", osm::BuildingType::Hospital)
        .value("WAREHOUSE", osm::BuildingType::Warehouse)
        .value("UNKNOWN", osm::BuildingType::Unknown);

    py::enum_<osm::AreaType>(m, "AreaType", "Classification taken from landuse=* and natural=*")
        .value("WATER", osm::AreaType::Water)
        .value("PARK", osm::AreaType::Park)
        .value("FOREST", osm::AreaType::Forest)
        .value("GRASS", osm::AreaType::Grass)
        .value("PARKING", osm::AreaType::Parking)
        .value("COMMERCIAL", osm::AreaType::Commercial)
        .value("RESIDENTIAL", osm::AreaType::Residential)
        .value("INDUSTRIAL", osm::AreaType::Industrial)
        .value("FARMLAND", osm::AreaType::Farmland)
        .value("CEMETERY", osm::AreaType::Cemetery)
        .value("UNKNOWN", osm::AreaType::Unknown);

    py::enum_<osm::RoofType>(m, "RoofType", "Classification taken from roof:shape=*")
        .value("FLAT", osm::RoofType::Flat)
        .value("GABLED", osm::RoofType::Gabled)
        .value("HIPPED", osm::RoofType::Hipped)
        .value("PYRAMIDAL", osm::RoofType::Pyramidal)
        .value("SKILLION", osm::RoofType::Skillion)
        .value("DOME", osm::RoofType::Dome)
        .value("UNKNOWN", osm::RoofType::Unknown);

    // UNKNOWN and NONE are distinct and the docstring says so, because collapsing
    // them is the classic sidewalk bug: "tag absent" may take a class default,
    // "sidewalk=no" may not.
    py::enum_<osm::SideFlags>(m, "SideFlags",
                              "Which side of the way direction carries a feature. UNKNOWN means "
                              "the tag was absent and a default may be inferred; NONE means the "
                              "tag said no and no default may be applied.")
        .value("UNKNOWN", osm::SideFlags::Unknown)
        .value("NONE", osm::SideFlags::None)
        .value("LEFT", osm::SideFlags::Left)
        .value("RIGHT", osm::SideFlags::Right)
        .value("BOTH", osm::SideFlags::Both);

    py::class_<osm::BoundingBox>(m, "BoundingBox", "Geographic extent in WGS84 degrees")
        .def(py::init<>())
        .def_readwrite("min_lat", &osm::BoundingBox::min_lat)
        .def_readwrite("max_lat", &osm::BoundingBox::max_lat)
        .def_readwrite("min_lon", &osm::BoundingBox::min_lon)
        .def_readwrite("max_lon", &osm::BoundingBox::max_lon)
        .def("expand", &osm::BoundingBox::expand, py::arg("lat"), py::arg("lon"))
        .def_property_readonly("is_valid", &osm::BoundingBox::is_valid)
        .def_property_readonly("width_meters", &osm::BoundingBox::width_meters)
        .def_property_readonly("height_meters", &osm::BoundingBox::height_meters)
        // Named for its ORDER. BoundingBox::center() returns (lat, lon) in a
        // dvec2, which reads as (x, y) to everything else in the codebase and
        // has been got backwards before. A Python name that says which is which
        // costs nothing and removes the trap from the binding entirely.
        .def_property_readonly("center_lat_lon", &osm::BoundingBox::center,
                               "Centre as (latitude, longitude) -- in that order")
        .def("__repr__", [](const osm::BoundingBox& b) {
            return "<stratum.BoundingBox lat " + std::to_string(b.min_lat) + ".."
                   + std::to_string(b.max_lat) + " lon " + std::to_string(b.min_lon) + ".."
                   + std::to_string(b.max_lon) + ">";
        });

    py::class_<osm::CoordinateSystem>(m, "CoordinateSystem",
                                      "Where the local metre grid sits on the Earth")
        .def(py::init<>())
        .def_readonly("origin_lat_lon", &osm::CoordinateSystem::origin_latlon)
        .def_readonly("origin_mercator", &osm::CoordinateSystem::origin_mercator)
        .def_readonly("scale", &osm::CoordinateSystem::scale);

    py::class_<osm::ParserConfig>(m, "ParserConfig", "What to import, and how to process it")
        .def(py::init<>())
        .def_readwrite("import_buildings", &osm::ParserConfig::import_buildings)
        .def_readwrite("import_roads", &osm::ParserConfig::import_roads)
        .def_readwrite("import_water", &osm::ParserConfig::import_water)
        .def_readwrite("import_landuse", &osm::ParserConfig::import_landuse)
        .def_readwrite("import_natural", &osm::ParserConfig::import_natural)
        .def_readwrite("import_amenities", &osm::ParserConfig::import_amenities)
        .def_readwrite("default_building_height", &osm::ParserConfig::default_building_height)
        .def_readwrite("meters_per_level", &osm::ParserConfig::meters_per_level)
        .def_readwrite("min_area_size", &osm::ParserConfig::min_area_size)
        .def_readwrite("simplify_geometry", &osm::ParserConfig::simplify_geometry)
        .def_readwrite("simplify_tolerance", &osm::ParserConfig::simplify_tolerance)
        .def_readwrite("filter_bounds", &osm::ParserConfig::filter_bounds);

    py::class_<osm::Road>(m, "Road", "One processed road centreline in local metres")
        .def_readonly("osm_id", &osm::Road::osm_id)
        .def_readonly("type", &osm::Road::type)
        .def_readonly("width", &osm::Road::width)
        .def_readonly("lanes", &osm::Road::lanes)
        .def_readonly("lanes_forward", &osm::Road::lanes_forward)
        .def_readonly("lanes_backward", &osm::Road::lanes_backward)
        .def_readonly("name", &osm::Road::name)
        .def_readonly("is_oneway", &osm::Road::is_oneway)
        .def_readonly("is_bridge", &osm::Road::is_bridge)
        .def_readonly("is_tunnel", &osm::Road::is_tunnel)
        .def_readonly("is_roundabout", &osm::Road::is_roundabout)
        .def_readonly("is_link", &osm::Road::is_link)
        .def_readonly("layer", &osm::Road::layer)
        .def_readonly("sidewalk", &osm::Road::sidewalk)
        .def_readonly("cycleway", &osm::Road::cycleway)
        .def_readonly("parking", &osm::Road::parking)
        .def_readonly("shoulder", &osm::Road::shoulder)
        .def_readonly("surface", &osm::Road::surface)
        .def_readonly("speed_limit", &osm::Road::speed_limit)
        .def_property_readonly("point_count",
                               [](const osm::Road& r) { return r.polyline.size(); })
        .def("polyline", [](const osm::Road& r) { return r.polyline; },
             "Centreline as a list of (x, y) tuples in local metres. Copies.")
        .def("node_ids", [](const osm::Road& r) { return r.node_ids; },
             "OSM node ids, parallel to polyline(). Copies.")
        .def("__repr__", [](const osm::Road& r) {
            return "<stratum.Road " + std::to_string(r.osm_id) + " "
                   + osm::road_type_name(r.type) + " " + std::to_string(r.polyline.size())
                   + " points>";
        });

    py::class_<osm::Building>(m, "Building", "One processed building footprint in local metres")
        .def_readonly("osm_id", &osm::Building::osm_id)
        .def_readonly("height", &osm::Building::height)
        .def_readonly("levels", &osm::Building::levels)
        .def_readonly("type", &osm::Building::type)
        .def_readonly("roof_type", &osm::Building::roof_type)
        .def_readonly("name", &osm::Building::name)
        .def_readonly("roof_color", &osm::Building::roof_color)
        .def_readonly("building_color", &osm::Building::building_color)
        .def_property_readonly("hole_count",
                               [](const osm::Building& b) { return b.holes.size(); })
        .def("footprint", [](const osm::Building& b) { return b.footprint; },
             "Outer ring as a list of (x, y) tuples. Copies.")
        .def("holes", [](const osm::Building& b) { return b.holes; },
             "Inner rings, each a list of (x, y) tuples. Copies.")
        .def("__repr__", [](const osm::Building& b) {
            return "<stratum.Building " + std::to_string(b.osm_id) + " "
                   + osm::building_type_name(b.type) + " h=" + std::to_string(b.height) + ">";
        });

    py::class_<osm::Area>(m, "Area", "One processed landuse, water or natural area")
        .def_readonly("osm_id", &osm::Area::osm_id)
        .def_readonly("type", &osm::Area::type)
        .def_readonly("name", &osm::Area::name)
        .def_property_readonly("hole_count", [](const osm::Area& a) { return a.holes.size(); })
        .def("polygon", [](const osm::Area& a) { return a.polygon; },
             "Outer ring as a list of (x, y) tuples. Copies.")
        .def("holes", [](const osm::Area& a) { return a.holes; })
        .def("__repr__", [](const osm::Area& a) {
            return "<stratum.Area " + std::to_string(a.osm_id) + " " + osm::area_type_name(a.type)
                   + ">";
        });

    py::class_<osm::ParsedOSMData::Statistics>(m, "ParseStatistics")
        .def_readonly("total_nodes", &osm::ParsedOSMData::Statistics::total_nodes)
        .def_readonly("total_ways", &osm::ParsedOSMData::Statistics::total_ways)
        .def_readonly("total_relations", &osm::ParsedOSMData::Statistics::total_relations)
        .def_readonly("processed_roads", &osm::ParsedOSMData::Statistics::processed_roads)
        .def_readonly("processed_buildings", &osm::ParsedOSMData::Statistics::processed_buildings)
        .def_readonly("processed_areas", &osm::ParsedOSMData::Statistics::processed_areas)
        .def_readonly("parse_time_ms", &osm::ParsedOSMData::Statistics::parse_time_ms)
        .def_readonly("process_time_ms", &osm::ParsedOSMData::Statistics::process_time_ms);

    // Python OWNS one of these: parse_osm() returns it by value. OSMParser is not
    // bound at all, precisely because its get_data() hands out a reference into a
    // parser a script can drop on the next line.
    py::class_<osm::ParsedOSMData>(
        m, "ParsedOSMData",
        "Everything one .osm or .pbf produced. Owned by Python; the parser that made it is gone.")
        .def(py::init<>())
        .def_property_readonly("road_count",
                               [](const osm::ParsedOSMData& d) { return d.roads.size(); })
        .def_property_readonly("building_count",
                               [](const osm::ParsedOSMData& d) { return d.buildings.size(); })
        .def_property_readonly("area_count",
                               [](const osm::ParsedOSMData& d) { return d.areas.size(); })
        .def_property_readonly("node_count",
                               [](const osm::ParsedOSMData& d) { return d.nodes.size(); })
        .def_property_readonly("way_count",
                               [](const osm::ParsedOSMData& d) { return d.ways.size(); })
        .def_property_readonly("relation_count",
                               [](const osm::ParsedOSMData& d) { return d.relations.size(); })
        .def_property_readonly("bounds", [](const osm::ParsedOSMData& d) { return d.bounds; })
        .def_property_readonly("coord_system",
                               [](const osm::ParsedOSMData& d) { return d.coord_system; })
        .def_property_readonly("stats", [](const osm::ParsedOSMData& d) { return d.stats; })
        // Methods, not properties, and the docstring says why: each one copies
        // every element, so `for r in data.roads()` is O(n) once and
        // `data.roads()[i]` in a loop is O(n^2). A property would hide that.
        .def("roads", [](const osm::ParsedOSMData& d) { return d.roads; },
             "Snapshot copy of every road. O(n) per call -- hoist it out of loops.")
        .def("buildings", [](const osm::ParsedOSMData& d) { return d.buildings; },
             "Snapshot copy of every building. O(n) per call.")
        .def("areas", [](const osm::ParsedOSMData& d) { return d.areas; },
             "Snapshot copy of every area. O(n) per call.")
        .def("way_tags",
             [](const osm::ParsedOSMData& d, osm::WayId id) -> std::optional<osm::TagMap> {
                 const auto it = d.ways.find(id);
                 if (it == d.ways.end()) return std::nullopt;
                 return it->second.tags;
             },
             py::arg("way_id"),
             "Tags of one raw way as a dict, or None. Road carries no tag map of its own.")
        .def("clear", &osm::ParsedOSMData::clear)
        .def("__repr__", [](const osm::ParsedOSMData& d) {
            return "<stratum.ParsedOSMData roads=" + std::to_string(d.roads.size())
                   + " buildings=" + std::to_string(d.buildings.size())
                   + " areas=" + std::to_string(d.areas.size()) + ">";
        });

    m.def(
        "parse_osm",
        [](const fs::path& path, const osm::ParserConfig& config) {
            // Checked before anything expensive and before the GIL goes, because
            // PyErr_SetString needs the GIL and because "the file is not there"
            // deserves the Python exception a script already handles.
            std::error_code ec;
            if (!fs::exists(path, ec) || ec) {
                raise_file_not_found("parse_osm", path);
            }

            osm::ParsedOSMData data;
            std::string error;
            bool ok = false;
            {
                // Safe to release: `path` and `config` are owned C++ copies that
                // pybind11 materialised while the GIL was held, `data` is local,
                // and no progress callback is bound, so nothing here re-enters
                // Python. See the note on the callback below.
                py::gil_scoped_release release;
                osm::OSMParser parser;
                parser.set_config(config);
                try {
                    ok = parser.parse(path);
                    if (ok) {
                        data = parser.take_data();
                    } else {
                        error = parser.get_error();
                    }
                } catch (const std::exception& e) {
                    // libosmium throws for a malformed or unreadable file. Letting
                    // it escape a GIL-released region would cross the pybind11
                    // dispatcher without the GIL held and abort the process.
                    ok = false;
                    error = e.what();
                } catch (...) {
                    ok = false;
                    error = "unrecognised failure inside the OSM parser";
                }
            }
            if (!ok) {
                throw StratumError("parse_osm: " + (error.empty() ? std::string("parse failed")
                                                                  : error));
            }
            return data;
        },
        py::arg("path"), py::arg("config") = osm::ParserConfig{},
        R"doc(Parse an .osm or .pbf file into a ParsedOSMData this script owns.

Raises FileNotFoundError when the path does not exist, and StratumError with the
parser's own message when the file exists but cannot be read.

No progress callback is exposed. A Python callable invoked from inside libosmium
would have to re-acquire the GIL per call, and a Python exception raised in it
would unwind through a parser that is not exception-neutral about it. A script
that needs progress should parse in a worker thread and poll.)doc");
}

// ============================================================================
// The scene
// ============================================================================

void register_scene(py::module_& m) {
    py::enum_<scene::LayerKind>(m, "LayerKind", "What a layer holds. Only GROUP may have children.")
        .value("GROUP", scene::LayerKind::Group)
        .value("SHAPE", scene::LayerKind::Shape)
        .value("GRAPH", scene::LayerKind::Graph)
        .value("MODEL", scene::LayerKind::Model)
        .value("MAP", scene::LayerKind::Map);

    py::enum_<scene::AttributeSource>(m, "AttributeSource",
                                      "Which rung of the resolution chain supplied a value")
        .value("NONE", scene::AttributeSource::None)
        .value("DEFAULT", scene::AttributeSource::Default)
        .value("LAYER", scene::AttributeSource::Layer)
        .value("OBJECT", scene::AttributeSource::Object)
        .value("USER", scene::AttributeSource::User);

    py::class_<scene::LayerTransform>(m, "LayerTransform",
                                      "A layer's own offset from its parent's space")
        .def(py::init<>())
        .def_readwrite("translation", &scene::LayerTransform::translation,
                       "(x, y, z) in metres, in the parent's space")
        .def_readwrite("rotation", &scene::LayerTransform::rotation,
                       "Euler YXZ in RADIANS -- yaw, pitch, roll, in a Y-up world")
        .def_readwrite("scale", &scene::LayerTransform::scale)
        .def_property_readonly("is_identity", &scene::LayerTransform::is_identity)
        .def(py::self == py::self)
        .def(py::self != py::self);

    // A COPY of a tree node, never a borrow. LayerTree::find() returns a Layer*
    // that dies with the layer, and "read the layer, delete it, read the object
    // you took out" is the first thing a script does when it writes a cleanup
    // pass.
    py::class_<scene::Layer>(m, "Layer",
                             "A snapshot copy of one layer. Editing it changes nothing; go "
                             "through the Document methods.")
        .def_readonly("id", &scene::Layer::id)
        .def_readonly("kind", &scene::Layer::kind)
        .def_readonly("name", &scene::Layer::name)
        .def_readonly("parent", &scene::Layer::parent)
        .def_readonly("children", &scene::Layer::children)
        .def_readonly("own_visible", &scene::Layer::own_visible)
        .def_readonly("own_locked", &scene::Layer::own_locked)
        .def_readonly("own_colour", &scene::Layer::own_colour)
        // A COPY, and NOT `def_readonly`, for the reason module.hpp gives about
        // OwnedSceneObject::mesh. pybind11 gives a data member of BOUND CLASS
        // type `return_value_policy::reference_internal`: the read hands back a
        // live Python LayerTransform wrapping THIS snapshot's own storage, and
        // pybind11 does not carry constness across, so
        //
        //     own = snapshot.own_transform
        //     own.translation = (99, 99, 99)
        //
        // rewrote the snapshot that the class docstring calls read-only. Worse,
        // a member borrow only reads correctly while its keep_alive holds, and
        // LayerTransform is three dvec3s of plain data: a lost keep_alive reads
        // freed bytes and still returns the right numbers, so no test of the
        // VALUE can ever catch it. Handing out a copy removes both problems at
        // once -- there is no borrow to lose a leash on, and there is nothing
        // to write through. The round trip a script actually wants still works:
        // read the copy, edit it, pass it to Document.set_layer_transform().
        //
        // A LayerTransform is 72 bytes of POD. The copy is not worth a property
        // that costs O(n), so unlike `SceneObject.mesh()` this one stays a
        // property rather than becoming a method.
        .def_property_readonly("own_transform",
                               [](const scene::Layer& l) { return l.own_transform; },
                               "A COPY of this snapshot's transform. Edit it and pass it to "
                               "Document.set_layer_transform() to apply it.")
        .def("__repr__", [](const scene::Layer& l) {
            return "<stratum.Layer " + std::to_string(l.id) + " "
                   + scene::layer_kind_name(l.kind) + " '" + l.name + "'>";
        });

    py::class_<scene::AttributeObject>(m, "AttributeObject",
                                       "An index and a generation. Stale handles compare unequal "
                                       "to whatever took their slot.")
        .def_readonly("index", &scene::AttributeObject::index)
        .def_readonly("generation", &scene::AttributeObject::generation)
        .def_property_readonly("valid", &scene::AttributeObject::valid)
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def("__hash__",
             [](const scene::AttributeObject& o) {
                 return static_cast<size_t>(o.index) * 1000003u
                        ^ static_cast<size_t>(o.generation);
             })
        .def("__repr__", [](const scene::AttributeObject& o) {
            return "<stratum.AttributeObject " + std::to_string(o.index) + ":"
                   + std::to_string(o.generation) + ">";
        });

    // Document is non-copyable AND non-movable, which is exactly what Python
    // needs: the default unique_ptr holder constructs it in place and never
    // moves it, so the raw pointers every Command on the history holds into its
    // LayerTree and AttributeStore stay valid for the object's whole life.
    py::class_<scene::Document>(
        m, "Document",
        R"doc(One open scene: layers, objects, attributes and the undo history.

Owned by Python. Not thread-safe: the bound calls release the GIL, so two Python
threads CAN be inside one Document at once, and must not be.

Every mutation goes through the command stack, which is why there is no direct
access to the layer tree or the attribute store. A mutation that skipped the
stack would be a hole in the history, and the symptom is an undo several steps
later restoring state that was never current.)doc")
        .def(py::init<>())

        // ── Layers ──────────────────────────────────────────────────────────
        .def(
            "create_layer",
            [](scene::Document& d, scene::LayerKind kind, const std::string& name,
               scene::LayerId parent, int64_t index) {
                auto command = std::make_unique<scene::CreateLayerCommand>(
                    d.layers(), kind, name, parent, sibling_index_from_python(index));
                const scene::LayerId id = command->layer();
                if (!d.history().execute(std::move(command))) {
                    throw StratumError("create_layer: refused -- parent " + std::to_string(parent)
                                       + " cannot hold children");
                }
                return id;
            },
            py::arg("kind"), py::arg("name"), py::arg("parent") = scene::kInvalidLayer,
            py::arg("index") = -1, "Create a layer and return its id. index=-1 means last.")
        .def(
            "delete_layer",
            [](scene::Document& d, scene::LayerId layer) {
                require_layer(d, layer, "delete_layer");
                if (!d.history().execute(
                        std::make_unique<scene::DeleteLayerCommand>(d.layers(), layer))) {
                    throw StratumError("delete_layer: refused for layer "
                                       + std::to_string(layer));
                }
            },
            py::arg("layer"), "Delete a layer and its whole subtree.")
        .def(
            "rename_layer",
            [](scene::Document& d, scene::LayerId layer, const std::string& name) {
                require_layer(d, layer, "rename_layer");
                if (!d.history().execute(
                        std::make_unique<scene::RenameLayerCommand>(d.layers(), layer, name))) {
                    throw StratumError("rename_layer: refused for layer " + std::to_string(layer));
                }
            },
            py::arg("layer"), py::arg("name"))
        .def(
            "reparent_layer",
            [](scene::Document& d, scene::LayerId layer, scene::LayerId new_parent,
               int64_t index) {
                require_layer(d, layer, "reparent_layer");
                if (!d.history().execute(std::make_unique<scene::ReparentLayerCommand>(
                        d.layers(), layer, new_parent, sibling_index_from_python(index)))) {
                    throw StratumError("reparent_layer: refused -- " + std::to_string(new_parent)
                                       + " cannot hold children, or the move would make layer "
                                       + std::to_string(layer) + " its own ancestor");
                }
            },
            py::arg("layer"), py::arg("new_parent"), py::arg("index") = -1)
        .def(
            "set_layer_visible",
            [](scene::Document& d, scene::LayerId layer, bool visible) {
                require_layer(d, layer, "set_layer_visible");
                if (!d.history().execute(std::make_unique<scene::SetLayerVisibleCommand>(
                        d.layers(), layer, visible))) {
                    throw StratumError("set_layer_visible: refused for layer "
                                       + std::to_string(layer));
                }
            },
            py::arg("layer"), py::arg("visible"))
        .def(
            "set_layer_locked",
            [](scene::Document& d, scene::LayerId layer, bool locked) {
                require_layer(d, layer, "set_layer_locked");
                if (!d.history().execute(
                        std::make_unique<scene::SetLayerLockedCommand>(d.layers(), layer, locked))) {
                    throw StratumError("set_layer_locked: refused for layer "
                                       + std::to_string(layer));
                }
            },
            py::arg("layer"), py::arg("locked"))
        .def(
            "set_layer_colour",
            [](scene::Document& d, scene::LayerId layer, std::optional<glm::vec3> colour) {
                require_layer(d, layer, "set_layer_colour");
                if (!d.history().execute(
                        std::make_unique<scene::SetLayerColourCommand>(d.layers(), layer, colour))) {
                    throw StratumError("set_layer_colour: refused for layer "
                                       + std::to_string(layer));
                }
            },
            py::arg("layer"), py::arg("colour"),
            "Pass None to clear, which restores inheritance rather than making the layer grey.")
        .def(
            "set_layer_transform",
            [](scene::Document& d, scene::LayerId layer, const scene::LayerTransform& transform) {
                require_layer(d, layer, "set_layer_transform");
                if (!d.history().execute(std::make_unique<scene::SetLayerTransformCommand>(
                        d.layers(), layer, transform))) {
                    throw StratumError("set_layer_transform: refused for layer "
                                       + std::to_string(layer));
                }
            },
            py::arg("layer"), py::arg("transform"))
        .def(
            "layer",
            [](const scene::Document& d, scene::LayerId id) -> std::optional<scene::Layer> {
                const scene::Layer* found = d.layers().find(id);
                if (found == nullptr) return std::nullopt;
                return *found;  // copy; see the note on stratum.Layer
            },
            py::arg("layer"), "A snapshot copy of the layer, or None when the id is stale.")
        .def("has_layer",
             [](const scene::Document& d, scene::LayerId id) { return d.layers().contains(id); },
             py::arg("layer"))
        .def_property_readonly("layer_count",
                               [](const scene::Document& d) { return d.layers().size(); })
        .def("roots", [](const scene::Document& d) { return d.layers().roots(); },
             "Top-level layer ids, in order.")
        .def("children",
             [](const scene::Document& d, scene::LayerId parent) {
                 return d.layers().children(parent);
             },
             py::arg("parent") = scene::kInvalidLayer)
        .def("effective_visible",
             [](const scene::Document& d, scene::LayerId id) {
                 return d.layers().effective_visible(id);
             },
             py::arg("layer"), "True only when this layer and every ancestor is visible.")
        .def("effective_locked",
             [](const scene::Document& d, scene::LayerId id) {
                 return d.layers().effective_locked(id);
             },
             py::arg("layer"), "True when this layer OR any ancestor is locked. True for a "
                               "stale id, which is the safe answer for an edit tool.")
        .def("effective_colour",
             [](const scene::Document& d, scene::LayerId id) {
                 return d.layers().effective_colour(id);
             },
             py::arg("layer"))

        // ── Objects ─────────────────────────────────────────────────────────
        .def("create_object", &scene::Document::create_object,
             py::arg("layer") = scene::kInvalidLayer,
             "Create a scene object. NOT an undoable edit: object lifetime is the "
             "document's, not the history's.")
        .def(
            "destroy_object",
            [](scene::Document& d, scene::AttributeObject obj) {
                if (!d.destroy_object(obj)) {
                    throw StratumError("destroy_object: handle is stale, was not created by this "
                                       "document, or its slot generation would wrap");
                }
            },
            py::arg("obj"))
        .def(
            "set_object_layer",
            [](scene::Document& d, scene::AttributeObject obj, scene::LayerId layer) {
                if (!d.set_object_layer(obj, layer)) {
                    throw StratumError("set_object_layer: object handle is stale or was not "
                                       "created by this document");
                }
            },
            py::arg("obj"), py::arg("layer"))
        .def("object_layer", &scene::Document::object_layer, py::arg("obj"),
             "The layer id, or 0 for a stale handle or an object in no layer.")
        .def("objects", &scene::Document::objects, "Live object handles, in slot-index order.")
        .def_property_readonly("object_count", &scene::Document::object_count)
        .def_property_readonly("slot_count", &scene::Document::slot_count)

        // ── Attributes ──────────────────────────────────────────────────────
        .def(
            "set_attribute",
            [](scene::Document& d, scene::AttributeObject obj, const std::string& name,
               const py::handle& value, bool user) {
                require_live_object(d, obj, "set_attribute");
                const scene::AttributeKey key = d.attributes().intern(name);
                const scene::AttributeTarget target =
                    user ? scene::AttributeTarget::user(obj, key)
                         : scene::AttributeTarget::object(obj, key);
                set_attribute_target(d, target, value, "set_attribute");
            },
            py::arg("obj"), py::arg("name"), py::arg("value"), py::arg("user") = false,
            "Write an attribute through the undo history. user=True writes the USER rung, "
            "which outranks and survives a rule re-run.")
        .def(
            "set_layer_attribute",
            [](scene::Document& d, scene::LayerId layer, const std::string& name,
               const py::handle& value) {
                require_layer(d, layer, "set_layer_attribute");
                const scene::AttributeKey key = d.attributes().intern(name);
                set_attribute_target(d, scene::AttributeTarget::layer(
                                            static_cast<scene::LayerRef>(layer), key),
                                     value, "set_layer_attribute");
            },
            py::arg("layer"), py::arg("name"), py::arg("value"))
        .def(
            "set_default_attribute",
            [](scene::Document& d, const std::string& name, const py::handle& value) {
                const scene::AttributeKey key = d.attributes().intern(name);
                set_attribute_target(d, scene::AttributeTarget::schema_default(key), value,
                                     "set_default_attribute");
            },
            py::arg("name"), py::arg("value"),
            "The document-wide fallback, used when neither the object nor its layer has one.")
        .def(
            "get_attribute",
            [](const scene::Document& d, scene::AttributeObject obj,
               const std::string& name) -> py::object {
                require_live_object(d, obj, "get_attribute");
                const scene::AttributeKey key = d.attributes().find_key(name);
                if (!key.valid()) return py::none();
                const scene::AttributeQuery q =
                    d.attributes().resolve(obj, key, d.object_layer_ref(obj));
                if (!q.ok() || q.value == nullptr) return py::none();
                return value_to_python(*q.value);
            },
            py::arg("obj"), py::arg("name"),
            "The resolved value -- user, then object, then layer, then default -- or None.")
        .def(
            "attribute_source",
            [](const scene::Document& d, scene::AttributeObject obj, const std::string& name) {
                require_live_object(d, obj, "attribute_source");
                const scene::AttributeKey key = d.attributes().find_key(name);
                if (!key.valid()) return scene::AttributeSource::None;
                return d.attributes().resolve(obj, key, d.object_layer_ref(obj)).source;
            },
            py::arg("obj"), py::arg("name"),
            "Which rung answered get_attribute(). NONE when nothing holds it.")
        .def(
            "clear_attribute",
            [](scene::Document& d, scene::AttributeObject obj, const std::string& name, bool user) {
                require_live_object(d, obj, "clear_attribute");
                const scene::AttributeKey key = d.attributes().find_key(name);
                if (!key.valid()) return false;
                const scene::AttributeTarget target =
                    user ? scene::AttributeTarget::user(obj, key)
                         : scene::AttributeTarget::object(obj, key);
                return scene::clear_attribute(d.history(), d.attributes(), target);
            },
            py::arg("obj"), py::arg("name"), py::arg("user") = false,
            "False when that rung held nothing. Not an error -- clearing an empty slot is a "
            "no-op, whereas naming a destroyed object raises.")
        .def(
            "attribute_names",
            [](const scene::Document& d, scene::AttributeObject obj) {
                require_live_object(d, obj, "attribute_names");
                std::vector<std::string> names;
                for (const scene::AttributeKey key :
                     d.attributes().keys_on(obj, d.object_layer_ref(obj))) {
                    names.emplace_back(d.attributes().key_name(key));
                }
                return names;
            },
            py::arg("obj"), "Every key that resolves on this object, in no particular order.")

        // ── History ─────────────────────────────────────────────────────────
        .def("undo", [](scene::Document& d) { return d.history().undo(); },
             "False when there was nothing to undo. That is a query, not a refusal, so it does "
             "not raise.")
        .def("redo", [](scene::Document& d) { return d.history().redo(); })
        .def_property_readonly("can_undo",
                               [](const scene::Document& d) { return d.history().can_undo(); })
        .def_property_readonly("can_redo",
                               [](const scene::Document& d) { return d.history().can_redo(); })
        .def_property_readonly("undo_label",
                               [](const scene::Document& d) { return d.history().undo_label(); })
        .def_property_readonly("redo_label",
                               [](const scene::Document& d) { return d.history().redo_label(); })
        .def_property_readonly("undo_depth",
                               [](const scene::Document& d) { return d.history().undo_depth(); })
        .def_property_readonly("redo_depth",
                               [](const scene::Document& d) { return d.history().redo_depth(); })
        .def_property_readonly("revision",
                               [](const scene::Document& d) { return d.history().revision(); })
        .def("seal", [](scene::Document& d) { d.history().seal(); },
             "End a gesture, so the next edit cannot merge into the last one.")
        .def("begin_transaction",
             [](scene::Document& d, const std::string& label) {
                 d.history().begin_transaction(label);
             },
             py::arg("label"), "Group the edits until commit into one undo step. Nestable.")
        // Both raise when nothing is open. CommandStack logs a warning and
        // returns, which is right for the editor -- a stray commit on mouse-up
        // must not take the application down -- and wrong here. A script's
        // transactions are balanced by hand, usually in a try/finally, and an
        // unbalanced pair means the edits the author thought were grouped are
        // not. The C++ symptom is one line in a log nobody reads; the Python
        // symptom is an undo that takes back a third of a change.
        .def("commit_transaction",
             [](scene::Document& d) {
                 if (!d.history().in_transaction()) {
                     throw StratumError("commit_transaction: no transaction is open");
                 }
                 d.history().commit_transaction();
             })
        .def("abort_transaction",
             [](scene::Document& d) {
                 if (!d.history().in_transaction()) {
                     throw StratumError("abort_transaction: no transaction is open");
                 }
                 d.history().abort_transaction();
             },
             "Revert whatever the innermost group applied, in reverse order.")
        .def_property_readonly("in_transaction",
                               [](const scene::Document& d) { return d.history().in_transaction(); })

        // ── Serialisation ───────────────────────────────────────────────────
        .def_property_readonly("dirty", &scene::Document::dirty)
        .def("mark_saved", &scene::Document::mark_saved)
        .def(
            "to_json",
            [](scene::Document& d) {
                std::string out;
                scene::DocumentIoResult result;
                {
                    py::gil_scoped_release release;
                    result = d.save_to_json(out);
                }
                if (!result.ok) throw StratumError("to_json: " + result.error);
                return out;
            },
            "Render the document as .stratum JSON. Does NOT clear dirty.")
        .def(
            "from_json",
            // Takes std::string BY VALUE on purpose. A string_view would point
            // into the Python str, and the GIL is released below -- another
            // thread could collect the str while the loader is reading it.
            [](scene::Document& d, std::string text) {
                scene::DocumentIoResult result;
                {
                    py::gil_scoped_release release;
                    result = d.load_from_json(text);
                }
                if (!result.ok) throw StratumError("from_json: " + result.error);
            },
            py::arg("text"),
            "Replace this document's contents. All or nothing: on failure it is exactly as it "
            "was. Every handle taken before the call belongs to the old contents.")
        .def(
            "save",
            [](scene::Document& d, const fs::path& path) {
                scene::DocumentIoResult result;
                {
                    py::gil_scoped_release release;
                    result = d.save_to_file(path);
                }
                if (!result.ok) throw StratumError("save: " + result.error);
            },
            py::arg("path"),
            "Write and then mark saved. Writes a sibling temporary and renames, so an "
            "interrupted write cannot truncate the user's only copy.")
        .def(
            "load",
            [](scene::Document& d, const fs::path& path) {
                std::error_code ec;
                if (!fs::exists(path, ec) || ec) {
                    raise_file_not_found("load", path);
                }
                scene::DocumentIoResult result;
                {
                    py::gil_scoped_release release;
                    result = d.load_from_file(path);
                }
                if (!result.ok) throw StratumError("load: " + result.error);
            },
            py::arg("path"))
        .def("__repr__", [](const scene::Document& d) {
            return "<stratum.Document layers=" + std::to_string(d.layers().size())
                   + " objects=" + std::to_string(d.object_count())
                   + (d.dirty() ? " dirty>" : ">");
        });
}

// ============================================================================
// Street graph, blocks and lots
// ============================================================================

void register_road(py::module_& m) {
    py::class_<road::RoadGraph::Stats>(m, "RoadGraphStats")
        .def_readonly("nodes", &road::RoadGraph::Stats::nodes)
        .def_readonly("edges", &road::RoadGraph::Stats::edges)
        .def_readonly("junctions", &road::RoadGraph::Stats::junctions)
        .def_readonly("dead_ends", &road::RoadGraph::Stats::dead_ends)
        .def_readonly("continuations", &road::RoadGraph::Stats::continuations)
        .def_readonly("roundabout_edges", &road::RoadGraph::Stats::roundabout_edges)
        .def_readonly("layer_split_nodes", &road::RoadGraph::Stats::layer_split_nodes);

    py::class_<road::GraphEdge>(m, "GraphEdge", "One street between two graph nodes. A copy.")
        .def_readonly("source_way", &road::GraphEdge::source_way)
        .def_readonly("from_node", &road::GraphEdge::from)
        .def_readonly("to_node", &road::GraphEdge::to)
        .def_readonly("type", &road::GraphEdge::type)
        .def_readonly("layer", &road::GraphEdge::layer)
        .def_readonly("width", &road::GraphEdge::width)
        .def_readonly("lanes", &road::GraphEdge::lanes)
        .def_readonly("is_oneway", &road::GraphEdge::is_oneway)
        .def_readonly("is_bridge", &road::GraphEdge::is_bridge)
        .def_readonly("is_tunnel", &road::GraphEdge::is_tunnel)
        .def_readonly("is_roundabout", &road::GraphEdge::is_roundabout)
        .def_readonly("is_link", &road::GraphEdge::is_link)
        .def_readonly("name", &road::GraphEdge::name)
        .def_property_readonly("length", &road::GraphEdge::length)
        .def("polyline", [](const road::GraphEdge& e) { return e.polyline; });

    py::class_<road::GraphNode>(m, "GraphNode", "One junction or endpoint. A copy.")
        .def_readonly("osm_id", &road::GraphNode::osm_id)
        .def_readonly("position", &road::GraphNode::position)
        .def_readonly("layer", &road::GraphNode::layer)
        .def_readonly("has_signals", &road::GraphNode::has_signals)
        .def_readonly("has_crossing", &road::GraphNode::has_crossing)
        .def_readonly("is_turning_circle", &road::GraphNode::is_turning_circle)
        .def_property_readonly("degree", &road::GraphNode::degree)
        .def_property_readonly("is_junction", &road::GraphNode::is_junction)
        .def_property_readonly("is_road_junction", &road::GraphNode::is_road_junction)
        .def_property_readonly("is_dead_end", &road::GraphNode::is_dead_end);

    py::class_<road::RoadGraph>(
        m, "RoadGraph",
        "The street network as a planar graph. Junctions come from shared OSM node IDENTITY, "
        "never from endpoint proximity.")
        .def(py::init<>())
        .def(
            "build",
            [](road::RoadGraph& g, const osm::ParsedOSMData& data) {
                // `data` is borrowed from a live Python object that the argument
                // tuple holds a reference to for the whole call, so releasing the
                // GIL cannot get it collected. Nothing bound here mutates
                // ParsedOSMData, so no other thread can change it underneath.
                py::gil_scoped_release release;
                g.build(data);
            },
            py::arg("data"))
        .def("clear", &road::RoadGraph::clear)
        .def_property_readonly("node_count",
                               [](const road::RoadGraph& g) { return g.nodes().size(); })
        .def_property_readonly("edge_count",
                               [](const road::RoadGraph& g) { return g.edges().size(); })
        .def("stats", &road::RoadGraph::stats)
        .def(
            "edge",
            [](const road::RoadGraph& g, size_t index) {
                if (index >= g.edges().size()) throw py::index_error("edge index out of range");
                return g.edges()[index];  // copy
            },
            py::arg("index"))
        .def(
            "node",
            [](const road::RoadGraph& g, size_t index) {
                if (index >= g.nodes().size()) throw py::index_error("node index out of range");
                return g.nodes()[index];  // copy
            },
            py::arg("index"));

    py::class_<road::BlockConfig>(m, "BlockConfig")
        .def(py::init<>())
        .def_readwrite("min_area", &road::BlockConfig::min_area)
        .def_readwrite("include_paths", &road::BlockConfig::include_paths)
        .def_readwrite("include_grade_separated", &road::BlockConfig::include_grade_separated)
        .def_readwrite("drop_duplicate_edges", &road::BlockConfig::drop_duplicate_edges);

    py::class_<road::BlockStats>(m, "BlockStats")
        .def_readonly("edges_considered", &road::BlockStats::edges_considered)
        .def_readonly("edges_used", &road::BlockStats::edges_used)
        .def_readonly("duplicate_edges", &road::BlockStats::duplicate_edges)
        .def_readonly("half_edges_walked", &road::BlockStats::half_edges_walked)
        .def_readonly("faces", &road::BlockStats::faces)
        .def_readonly("expected_faces", &road::BlockStats::expected_faces)
        .def_readonly("outer_faces", &road::BlockStats::outer_faces)
        .def_readonly("tree_faces", &road::BlockStats::tree_faces)
        .def_readonly("zero_area_faces", &road::BlockStats::zero_area_faces)
        .def_readonly("malformed_faces", &road::BlockStats::malformed_faces)
        .def_readonly("rejected_small", &road::BlockStats::rejected_small)
        .def_readonly("blocks", &road::BlockStats::blocks);

    py::class_<road::Block>(m, "Block", "One planar face of the street graph. A copy.")
        .def(py::init<>())
        .def_readonly("id", &road::Block::id)
        .def_readonly("area", &road::Block::area)
        .def_readonly("perimeter", &road::Block::perimeter)
        .def_readonly("has_grade_separated_edge", &road::Block::has_grade_separated_edge)
        .def_property_readonly("hole_count",
                               [](const road::Block& b) { return b.holes.size(); })
        .def_property_readonly("edge_count", [](const road::Block& b) { return b.edges.size(); })
        .def("ring", [](const road::Block& b) { return b.ring; },
             "Outer boundary as a list of (x, y) tuples, counter-clockwise. Copies.")
        .def("ring_edges", [](const road::Block& b) { return b.ring_edges; })
        .def("holes", [](const road::Block& b) { return b.holes; });

    py::class_<road::BlockExtraction>(m, "BlockExtraction")
        .def_property_readonly("block_count",
                               [](const road::BlockExtraction& e) { return e.blocks.size(); })
        // A COPY. See the note on Layer.own_transform: `def_readonly` on a
        // member of BOUND CLASS type is a borrow into the container with a
        // keep_alive holding it up, and BlockStats is twelve counters of plain
        // data -- freed POD bytes read back correctly, so a lost keep_alive is
        // invisible to every assertion about the VALUE and shows up later as a
        // use-after-free with no diagnostic. 96 bytes copied per read removes
        // the whole question.
        .def_property_readonly("stats",
                               [](const road::BlockExtraction& e) { return e.stats; },
                               "A COPY of the traversal counts, independent of this extraction.")
        .def("blocks", [](const road::BlockExtraction& e) { return e.blocks; },
             "Snapshot copy of every block. O(n) per call.")
        .def(
            "block",
            [](const road::BlockExtraction& e, size_t index) {
                if (index >= e.blocks.size()) throw py::index_error("block index out of range");
                return e.blocks[index];  // copy
            },
            py::arg("index"));

    m.def(
        "extract_blocks",
        [](const road::RoadGraph& graph, const road::BlockConfig& config) {
            py::gil_scoped_release release;
            return road::extract_blocks(graph, config);
        },
        py::arg("graph"), py::arg("config") = road::BlockConfig{},
        "Walk the face orbits of the street graph. Returns the bounded faces only -- the outer "
        "face and any tree-like tails are discarded and counted in stats.");

    py::class_<road::LotParams>(m, "LotParams")
        .def(py::init<>())
        .def_readwrite("lot_area_min", &road::LotParams::lot_area_min)
        .def_readwrite("lot_area_max", &road::LotParams::lot_area_max)
        .def_readwrite("lot_width_min", &road::LotParams::lot_width_min)
        .def_readwrite("force_street_access", &road::LotParams::force_street_access)
        .def_readwrite("irregularity", &road::LotParams::irregularity)
        .def_readwrite("seed", &road::LotParams::seed)
        .def_readwrite("corner_angle_max_deg", &road::LotParams::corner_angle_max_deg)
        .def_readwrite("corner_width", &road::LotParams::corner_width)
        .def_readwrite("block_key", &road::LotParams::block_key)
        .def_readwrite("max_depth", &road::LotParams::max_depth);

    py::class_<road::LotStats>(m, "LotStats")
        .def_readonly("lots", &road::LotStats::lots)
        .def_readonly("corner_lots", &road::LotStats::corner_lots)
        .def_readonly("splits", &road::LotStats::splits)
        .def_readonly("split_attempts", &road::LotStats::split_attempts)
        .def_readonly("rejected_area", &road::LotStats::rejected_area)
        .def_readonly("rejected_width", &road::LotStats::rejected_width)
        .def_readonly("rejected_access", &road::LotStats::rejected_access)
        .def_readonly("rejected_degenerate", &road::LotStats::rejected_degenerate)
        .def_readonly("dropped_pieces", &road::LotStats::dropped_pieces)
        .def_readonly("lots_in_holes", &road::LotStats::lots_in_holes)
        .def_readonly("depth_limit_hits", &road::LotStats::depth_limit_hits)
        .def_readonly("max_depth_reached", &road::LotStats::max_depth_reached);

    py::class_<road::Lot>(m, "Lot", "One parcel of a subdivided block. A copy.")
        .def_readonly("id", &road::Lot::id)
        .def_readonly("node_key", &road::Lot::node_key)
        .def_readonly("area", &road::Lot::area)
        .def_readonly("frontage_length", &road::Lot::frontage_length)
        .def_readonly("depth", &road::Lot::depth)
        .def_readonly("is_corner_lot", &road::Lot::is_corner_lot)
        .def("ring", [](const road::Lot& l) { return l.ring; })
        .def("ring_edges", [](const road::Lot& l) { return l.ring_edges; });

    py::class_<road::LotSubdivision>(m, "LotSubdivision")
        .def_property_readonly("lot_count",
                               [](const road::LotSubdivision& s) { return s.lots.size(); })
        // A COPY, for the same reason as BlockExtraction.stats above.
        .def_property_readonly("stats",
                               [](const road::LotSubdivision& s) { return s.stats; },
                               "A COPY of the subdivision counts, independent of this subdivision.")
        .def("lots", [](const road::LotSubdivision& s) { return s.lots; },
             "Snapshot copy of every lot. O(n) per call.");

    m.def(
        "subdivide_block",
        [](const road::Block& block, const road::LotParams& params) {
            py::gil_scoped_release release;
            return road::subdivide_block(block, params);
        },
        py::arg("block"), py::arg("params") = road::LotParams{},
        "Recursively cut a block into lots. Deterministic for a given params.seed and "
        "params.block_key.");

    m.def("signed_ring_area", &road::signed_ring_area, py::arg("ring"),
          "Positive for a counter-clockwise ring, negative for clockwise.");
}

// ============================================================================
// Meshes and export
// ============================================================================

void register_export(py::module_& m) {
    py::enum_<stratum::MaterialId>(m, "MaterialId", "Material slot of a submesh range")
        .value("DEFAULT", stratum::MaterialId::Default)
        .value("ASPHALT", stratum::MaterialId::Asphalt)
        .value("CONCRETE", stratum::MaterialId::Concrete)
        .value("CURB", stratum::MaterialId::Curb)
        .value("SIDEWALK", stratum::MaterialId::Sidewalk)
        .value("MARKINGS", stratum::MaterialId::Markings)
        .value("GRAVEL", stratum::MaterialId::Gravel)
        .value("DIRT", stratum::MaterialId::Dirt)
        .value("GRASS", stratum::MaterialId::Grass)
        .value("BRIDGE_DECK", stratum::MaterialId::BridgeDeck)
        .value("PARAPET", stratum::MaterialId::Parapet)
        .value("WALL", stratum::MaterialId::Wall)
        .value("ROOF", stratum::MaterialId::Roof);

    py::enum_<osm::SceneObjectKind>(m, "SceneObjectKind")
        .value("UNKNOWN", osm::SceneObjectKind::Unknown)
        .value("BUILDING", osm::SceneObjectKind::Building)
        .value("AREA", osm::SceneObjectKind::Area)
        .value("ROAD", osm::SceneObjectKind::Road)
        .value("TERRAIN", osm::SceneObjectKind::Terrain);

    py::enum_<osm::SceneExportFormat>(m, "SceneExportFormat")
        .value("OBJ", osm::SceneExportFormat::Obj)
        .value("GLTF", osm::SceneExportFormat::Gltf);

    py::class_<stratum::Mesh>(m, "Mesh",
                              "Plain vertex and index data. Owned by Python; copying one copies "
                              "its buffers.")
        .def(py::init<>())
        .def_property_readonly("vertex_count",
                               [](const stratum::Mesh& mesh) { return mesh.vertices.size(); })
        .def_property_readonly("index_count",
                               [](const stratum::Mesh& mesh) { return mesh.indices.size(); })
        .def_property_readonly("triangle_count",
                               [](const stratum::Mesh& mesh) { return mesh.indices.size() / 3; })
        .def_property_readonly("submesh_count",
                               [](const stratum::Mesh& mesh) { return mesh.submeshes.size(); })
        .def_property_readonly("is_valid", &stratum::Mesh::is_valid)
        .def_property_readonly("bounds_min",
                               [](const stratum::Mesh& mesh) { return mesh.bounds.min; })
        .def_property_readonly("bounds_max",
                               [](const stratum::Mesh& mesh) { return mesh.bounds.max; })
        .def("positions",
             [](const stratum::Mesh& mesh) {
                 std::vector<glm::vec3> out;
                 out.reserve(mesh.vertices.size());
                 for (const stratum::Vertex& v : mesh.vertices) out.push_back(v.position);
                 return out;
             },
             "Every vertex position as an (x, y, z) tuple. Copies.")
        .def("indices", [](const stratum::Mesh& mesh) { return mesh.indices; })
        .def("clear", &stratum::Mesh::clear)
        .def("__repr__", [](const stratum::Mesh& mesh) {
            return "<stratum.Mesh " + std::to_string(mesh.vertices.size()) + " verts, "
                   + std::to_string(mesh.indices.size() / 3) + " tris>";
        });

    m.def(
        "build_building_mesh",
        [](const osm::Building& building) {
            py::gil_scoped_release release;
            return osm::MeshBuilder::build_building_mesh(building);
        },
        py::arg("building"), "Extrude a footprint into a mesh Python owns.");

    m.def(
        "build_area_mesh",
        [](const osm::Area& area) {
            py::gil_scoped_release release;
            return osm::MeshBuilder::build_area_mesh(area);
        },
        py::arg("area"), "Triangulate a landuse or natural area into a flat mesh Python owns.");

    // See module.hpp: this is OwnedSceneObject, and the Python name is
    // SceneObject because it is the only scene object Python ever sees. The
    // pointer-carrying osm::SceneObject exists for the duration of export_scene()
    // and nowhere else.
    py::class_<OwnedSceneObject>(
        m, "SceneObject",
        R"doc(One thing to export: a mesh plus what it is called and what it came from.

Owns its mesh. The constructor COPIES the mesh you pass, so

    obj = stratum.SceneObject(mesh, stratum.SceneObjectKind.BUILDING)
    del mesh
    stratum.export_scene([obj], out_dir)

is correct. The C++ export type holds a borrowed mesh pointer; this one does
not, because a Python script has no way to guarantee the lifetime that borrow
needs.

`mesh()` hands back a COPY, so the geometry really is fixed at construction.
Use `triangle_count` and `vertex_count` when all you want is the size; they read
the owned mesh in place and copy nothing.)doc")
        .def(py::init([](const stratum::Mesh& mesh, osm::SceneObjectKind kind, std::string name,
                         int64_t osm_id, std::string layer) {
                 OwnedSceneObject obj;
                 obj.mesh = mesh;  // the copy the whole design rests on
                 obj.kind = kind;
                 obj.name = std::move(name);
                 obj.osm_id = osm_id;
                 obj.layer = std::move(layer);
                 return obj;
             }),
             py::arg("mesh"), py::arg("kind") = osm::SceneObjectKind::Unknown,
             py::arg("name") = std::string{}, py::arg("osm_id") = 0,
             py::arg("layer") = std::string{})
        // A METHOD returning a COPY, and both halves of that are deliberate.
        //
        // `def_readonly` was the obvious thing and is wrong here. pybind11 gives
        // a data member `return_value_policy::reference_internal`, which stops a
        // script REBINDING the field but hands back a live Python Mesh wrapping
        // the object's own storage -- and pybind11 does not carry constness into
        // Python, so `obj.mesh.clear()` emptied the mesh the exporter was about
        // to write. The first run of the probe suite did exactly that: ten
        // triangles before, zero after, no error anywhere, and an export that
        // silently produced an empty file. "The geometry is fixed at
        // construction" has to be enforced, not asserted in a comment.
        //
        // A copy costs one Mesh per read, which is why it is a method and not a
        // property: module.hpp's naming rule is that a read costing O(n) must
        // look like it costs O(n), or `for o in objs: o.mesh.triangle_count`
        // becomes a hidden deep copy per iteration. The two counts below are the
        // cheap answers to the question that loop was really asking.
        .def("mesh", [](const OwnedSceneObject& o) { return o.mesh; },
             "A COPY of the owned mesh. O(n) per call -- mutating it cannot reach the "
             "SceneObject, and `triangle_count` answers the common question for free.")
        .def_property_readonly("vertex_count",
                               [](const OwnedSceneObject& o) { return o.mesh.vertices.size(); })
        .def_property_readonly("triangle_count",
                               [](const OwnedSceneObject& o) { return o.mesh.indices.size() / 3; })
        .def_readwrite("kind", &OwnedSceneObject::kind)
        .def_readwrite("name", &OwnedSceneObject::name)
        .def_readwrite("osm_id", &OwnedSceneObject::osm_id)
        .def_readwrite("layer", &OwnedSceneObject::layer)
        .def_readwrite("metadata", &OwnedSceneObject::metadata)
        .def("__repr__", [](const OwnedSceneObject& o) {
            return "<stratum.SceneObject '" + o.name + "' "
                   + osm::scene_object_kind_name(o.kind) + " "
                   + std::to_string(o.mesh.indices.size() / 3) + " tris>";
        });

    py::class_<osm::SceneExportConfig>(m, "SceneExportConfig")
        .def(py::init<>())
        .def_readwrite("format", &osm::SceneExportConfig::format)
        .def_readwrite("chunk_size", &osm::SceneExportConfig::chunk_size,
                       "Metres per chunk; 0 writes one file for everything.")
        .def_readwrite("y_up", &osm::SceneExportConfig::y_up)
        .def_readwrite("material_prefix", &osm::SceneExportConfig::material_prefix)
        .def_readwrite("name_prefix", &osm::SceneExportConfig::name_prefix)
        .def_readwrite("write_metadata", &osm::SceneExportConfig::write_metadata);

    py::class_<osm::SceneExportStats>(m, "SceneExportStats")
        .def_readonly("chunks", &osm::SceneExportStats::chunks)
        .def_readonly("objects", &osm::SceneExportStats::objects)
        .def_readonly("vertices", &osm::SceneExportStats::vertices)
        .def_readonly("triangles", &osm::SceneExportStats::triangles)
        .def_readonly("dropped_triangles", &osm::SceneExportStats::dropped_triangles)
        .def_readonly("unwritten_triangles", &osm::SceneExportStats::unwritten_triangles)
        .def_readonly("files", &osm::SceneExportStats::files)
        .def_readonly("export_ms", &osm::SceneExportStats::export_ms)
        .def_readonly("written_files", &osm::SceneExportStats::written_files)
        .def("__repr__", [](const osm::SceneExportStats& s) {
            return "<stratum.SceneExportStats " + std::to_string(s.files) + " files, "
                   + std::to_string(s.triangles) + " tris>";
        });

    m.def(
        "scene_object_for_building",
        [](const osm::Building& building, const stratum::Mesh& mesh) {
            // describe_building() fills in the metadata table and sets
            // SceneObject::mesh to &mesh. That pointer is dropped here: only the
            // labels are taken across, and the mesh is copied into the owner.
            const osm::SceneObject described = osm::describe_building(building, mesh);
            OwnedSceneObject out;
            out.mesh = mesh;
            out.kind = described.kind;
            out.name = described.name;
            out.osm_id = described.osm_id;
            out.layer = described.layer;
            out.metadata = described.metadata;
            return out;
        },
        py::arg("building"), py::arg("mesh"),
        "A SceneObject with the name, id, layer and metadata the C++ exporter would give it.");

    m.def(
        "scene_object_for_area",
        [](const osm::Area& area, const stratum::Mesh& mesh) {
            const osm::SceneObject described = osm::describe_area(area, mesh);
            OwnedSceneObject out;
            out.mesh = mesh;
            out.kind = described.kind;
            out.name = described.name;
            out.osm_id = described.osm_id;
            out.layer = described.layer;
            out.metadata = described.metadata;
            return out;
        },
        py::arg("area"), py::arg("mesh"));

    m.def(
        "export_scene",
        [](const py::sequence& objects, const fs::path& out_dir,
           const osm::SceneExportConfig& config) {
            const size_t count = py::len(objects);

            // `pinned` holds a STRONG reference to each element for the whole
            // call. Relying on the sequence argument alone is not enough: the
            // GIL is released below, and another thread holding the same list
            // can clear it, drop the last reference to an element, and free the
            // OwnedSceneObject whose mesh `handles` points into. Holding our own
            // references makes that impossible.
            //
            // Declared BEFORE the release guard so that it is destroyed AFTER it
            // -- dropping a py::object without the GIL is a crash of its own.
            std::vector<py::object> pinned;
            std::vector<osm::SceneObject> handles;
            pinned.reserve(count);
            handles.reserve(count);

            for (size_t i = 0; i < count; ++i) {
                py::object item = objects[i];
                OwnedSceneObject* owned = nullptr;
                try {
                    owned = item.cast<OwnedSceneObject*>();
                } catch (const py::cast_error&) {
                    throw py::type_error("export_scene: element " + std::to_string(i)
                                         + " is not a stratum.SceneObject");
                }
                if (owned == nullptr) {
                    throw py::type_error("export_scene: element " + std::to_string(i) + " is None");
                }

                osm::SceneObject handle;
                handle.mesh = &owned->mesh;  // borrowed, and pinned just below
                handle.kind = owned->kind;
                handle.name = owned->name;
                handle.osm_id = owned->osm_id;
                handle.layer = owned->layer;
                handle.metadata = owned->metadata;
                handles.push_back(std::move(handle));
                pinned.push_back(std::move(item));
            }

            osm::SceneExportStats stats;
            {
                py::gil_scoped_release release;
                stats = osm::export_scene(handles, out_dir, config);
            }

            // scene_export.hpp says "a completely failed export comes back with
            // files == 0 rather than throwing". That is the right C++ contract
            // -- a partial export is still worth its stats -- and the wrong
            // Python one. A headless run that cannot write its output directory
            // would otherwise get a stats object full of zeroes, no exception
            // and exit status 0, which is precisely the silent failure this
            // module exists to prevent.
            //
            // unwritten_triangles is the unambiguous signal and the exporter
            // documents it as such: non-zero ONLY when a chunk's file could not
            // be opened. Geometry the exporter rejected lands in
            // dropped_triangles instead and is NOT a destination failure, so it
            // does not raise. An export of nothing writes nothing and is not an
            // error either -- unwritten_triangles stays 0 -- which keeps
            // `export_scene([], d)` the no-op it reads as.
            //
            // The stats go WITH the exception rather than being lost with it: a
            // half-written export is exactly the case the C++ contract was
            // shaped for, and "which files did I manage to write" is not
            // answerable from a sentence. See raise_export_refused().
            if (stats.unwritten_triangles > 0) {
                raise_export_refused("export_scene: could not write "
                                         + std::to_string(stats.unwritten_triangles)
                                         + " triangles into " + out_dir.string() + "; wrote "
                                         + std::to_string(stats.files)
                                         + " files. The destination refused the write -- check "
                                           "permissions, free space, and that the path is not a "
                                           "file. The full SceneExportStats is on this "
                                           "exception as `.stats`.",
                                     stats);
            }
            return stats;
        },
        py::arg("objects"), py::arg("out_dir"), py::arg("config") = osm::SceneExportConfig{},
        R"doc(Write every object to out_dir, chunked by config.chunk_size. Creates out_dir.

Raises StratumError when the destination refused any of the geometry -- see
SceneExportStats.unwritten_triangles. Geometry the exporter itself rejected is
reported in stats.dropped_triangles and does NOT raise: that is a complaint
about the input, and the caller is given the numbers to judge it by.

The refusal carries the numbers too. A partial export raises, and the
SceneExportStats for everything that DID reach disk is on the exception as
`.stats`, so

    try:
        stratum.export_scene(objects, out_dir)
    except stratum.StratumError as exc:
        print(exc.stats.files, "files written before the refusal")
        print(exc.stats.written_files)

still reports what the job managed to do.)doc");

    m.def(
        "export_scene_object",
        [](const OwnedSceneObject& object, const fs::path& out_path,
           const osm::SceneExportConfig& config) {
            // Separated from the write failure below on purpose. C++ returns one
            // bool for "empty mesh" and "could not open the file", and reporting
            // both as "could not write <path>" sends a script author to check
            // permissions on a path that was never the problem.
            if (object.mesh.indices.size() < 3) {
                throw StratumError("export_scene_object: '" + object.name
                                   + "' holds no triangles, so there is nothing to write to "
                                   + out_path.string());
            }

            osm::SceneObject handle;
            handle.mesh = &object.mesh;
            handle.kind = object.kind;
            handle.name = object.name;
            handle.osm_id = object.osm_id;
            handle.layer = object.layer;
            handle.metadata = object.metadata;

            bool ok = false;
            {
                py::gil_scoped_release release;
                ok = osm::export_scene_object(handle, out_path, config);
            }
            if (!ok) {
                throw StratumError("export_scene_object: could not write " + out_path.string()
                                   + "; check permissions, free space, and that the parent "
                                     "path is not a file.");
            }
        },
        py::arg("object"), py::arg("out_path"), py::arg("config") = osm::SceneExportConfig{},
        "Write one object to one named file, unchunked. Raises StratumError when the object "
        "has no triangles or the file could not be written.");
}

// ============================================================================
// Rule language
// ============================================================================

void register_rules(py::module_& m) {
    py::class_<RuleParseSummary>(m, "RuleParseSummary",
                                 "What a rule file parse produced. Not the AST -- see the note in "
                                 "bindings.cpp on why the AST does not cross.")
        .def_readonly("ok", &RuleParseSummary::ok)
        .def_readonly("report", &RuleParseSummary::report,
                      "Every diagnostic, formatted with carets, ready to print.")
        .def_readonly("messages", &RuleParseSummary::messages)
        .def_readonly("error_count", &RuleParseSummary::error_count)
        .def_readonly("warning_count", &RuleParseSummary::warning_count)
        .def_readonly("rule_names", &RuleParseSummary::rule_names)
        .def_readonly("attribute_names", &RuleParseSummary::attribute_names)
        .def_readonly("start_rule", &RuleParseSummary::start_rule,
                      "Name of the @start rule, or '' for a library file.")
        .def_readonly("version", &RuleParseSummary::version)
        .def("__repr__", [](const RuleParseSummary& s) {
            return "<stratum.RuleParseSummary " + std::string(s.ok ? "ok" : "failed") + ", "
                   + std::to_string(s.error_count) + " errors, "
                   + std::to_string(s.rule_names.size()) + " rules>";
        });

    m.def(
        "parse_rules",
        // std::string by value again, for the same reason from_json() takes one:
        // the GIL goes away while the parser is reading it.
        [](std::string source, std::string filename) {
            RuleParseSummary summary;
            {
                py::gil_scoped_release release;
                const rules::ParseResult result = rules::parse(source, filename);
                summary.ok = result.ok();
                summary.report = result.render_all(source);
                for (const rules::Diagnostic& d : result.diagnostics) {
                    summary.messages.push_back(d.message);
                    if (d.severity == rules::Severity::Error) {
                        ++summary.error_count;
                    } else {
                        ++summary.warning_count;
                    }
                }
                summary.version = result.file.version;
                for (const rules::RuleDecl& rule : result.file.rules) {
                    summary.rule_names.push_back(rule.name);
                }
                for (const rules::AttrDecl& attr : result.file.attributes) {
                    summary.attribute_names.push_back(attr.name);
                }
                if (result.file.start_rule != rules::kNoNode
                    && result.file.start_rule < result.file.rules.size()) {
                    summary.start_rule = result.file.rules[result.file.start_rule].name;
                }
            }
            return summary;
        },
        py::arg("source"), py::arg("filename") = std::string("<string>"),
        "Parse rule-language source. Never raises for a bad file -- a parse failure is data, "
        "and `summary.report` is what to print.");
}

} // namespace

// ============================================================================
// Module assembly
// ============================================================================

namespace stratum::python {

void register_module(py::module_& m) {
    m.doc() =
        "Stratum: OpenStreetMap to game-ready 3D city, driven from Python with no window open.";

    // `version_from_build` exists so that a TEST of `__version__` has something
    // to assert in EITHER build. Without it the only honest check is "it is a
    // str, and if it is not the sentinel then it is dotted", whose second half
    // is dead code in every tree where CMake has not passed the define -- which
    // is every tree today. The flag makes both branches assertable: a build that
    // supplied a version must not report the sentinel, and a build that did not
    // must report it in exactly the documented words and nothing else.
    //
    // A missing define is deliberately not a build error: the version is a
    // convenience the CMake side may or may not pass, and a module that refuses
    // to build over its own version string would be worse than one that says it
    // does not know.
#ifdef STRATUM_VERSION_STRING
    m.attr("__version__") = STRATUM_VERSION_STRING;
    m.attr("version_from_build") = true;
#else
    m.attr("__version__") = "0.0.0+unknown";
    m.attr("version_from_build") = false;
#endif

    // Order matters. pybind11 resolves a type in a signature at def() time, so a
    // function mentioning Block before Block is registered gets a docstring
    // saying "handle" and an argument that arrives unconverted.
    register_errors(m);
    register_osm_types(m);
    register_scene(m);
    register_road(m);
    register_export(m);
    register_rules(m);
}

bool module_linked() { return true; }

} // namespace stratum::python

// ============================================================================
// Entry points
// ============================================================================
//
// Exactly one of these compiles. See module.hpp: having both in one binary gives
// an embedded interpreter two ways to find the module and two live copies of
// every registered type.

#ifdef STRATUM_PYTHON_EXTENSION_MODULE
PYBIND11_MODULE(stratum, m) { stratum::python::register_module(m); }
#else
PYBIND11_EMBEDDED_MODULE(stratum, m) { stratum::python::register_module(m); }
#endif

#else // STRATUM_ENABLE_PYTHON

// A translation unit with no external symbols makes MSVC emit LNK4221 and makes
// some archivers warn. One inline function costs nothing and keeps the
// Python-disabled build quiet.
namespace stratum::python::detail {
inline void python_bindings_disabled() {}
} // namespace stratum::python::detail

#endif // STRATUM_ENABLE_PYTHON
