# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Seamus Mullan and the Stratum contributors

"""Tests for the ``stratum`` Python module.

Run through the embedded interpreter by ``tests/python/run_python_tests.cpp``,
which calls :func:`run_all` and turns the result into framework.hpp checks, so
this file joins the ctest suite like every other test in the tree.

What is asserted here, and what is deliberately not:

* **Ownership.** Several tests exist only to prove that a Python object which
  outlives the C++ object it came from still reads correctly -- the SceneObject
  that outlives its Mesh, the Layer snapshot that outlives its layer, the
  ParsedOSMData that outlives the parser (there is no parser to outlive: it is
  not bound). A binding that handed out a borrow instead of a copy would pass
  the value checks and then segfault here, which is the point.

* **Errors.** Every refusal is asserted by TYPE and by message content. A
  binding that returned ``None`` on failure would pass a test that only checked
  the happy path, and the script author would find out three hundred lines
  later, on an empty export.

* **Numbers with a known answer.** Block areas are cross-checked against a
  shoelace computed in Python over the same ring, so the assertion does not
  depend on the projection or on the fixture's coordinates. A test that only
  said ``area > 0`` would pass against a traversal that kept the wrong face.

Nothing here writes into the source tree: every fixture is generated into a
fresh temporary directory and removed afterwards.
"""

import gc
import math
import os
import shutil
import tempfile
import traceback

import stratum

# ============================================================================
# Harness
# ============================================================================

_TESTS = []


def test(fn):
    """Register a test. Order of definition is order of execution."""
    _TESTS.append(fn)
    return fn


def check(condition, message="condition was not true"):
    if not condition:
        raise AssertionError(message)


def check_eq(actual, expected, message=""):
    if actual != expected:
        raise AssertionError(
            "{}actual: {!r}  expected: {!r}".format(
                message + "  " if message else "", actual, expected
            )
        )


def check_near(actual, expected, eps, message=""):
    if not math.isfinite(actual) or abs(actual - expected) > eps:
        raise AssertionError(
            "{}actual: {!r}  expected: {!r}  tolerance: {!r}".format(
                message + "  " if message else "", actual, expected, eps
            )
        )


def check_raises(exc_type, fn, needle=None):
    """Assert that ``fn()`` raises ``exc_type``, optionally carrying ``needle``.

    The message check is not decoration. "raises something" is satisfied by a
    TypeError from a typo in the test itself, so every negative test here also
    says what the message must contain.
    """
    try:
        fn()
    except exc_type as exc:
        if needle is not None and needle not in str(exc):
            raise AssertionError(
                "raised {} as expected but message {!r} does not contain {!r}".format(
                    exc_type.__name__, str(exc), needle
                )
            )
        return exc
    except BaseException as exc:  # noqa: BLE001 - reporting the wrong type IS the point
        raise AssertionError(
            "expected {} but got {}: {}".format(
                exc_type.__name__, type(exc).__name__, exc
            )
        )
    raise AssertionError("expected {} but nothing was raised".format(exc_type.__name__))


def without_next_id(text):
    """The document JSON with the layer-allocator line removed.

    ``Document.to_json()`` TAKES a layer id every time it runs. document.hpp is
    explicit about why: LayerTree offers no way to read its id counter without
    taking from it, and recomputing the counter from the surviving layers would
    reissue every id the saved session had already spent on a since-deleted
    layer. So two renderings of one unchanged document are identical except for
    ``next_id``, which goes up by one per call. Tests that compare two renderings
    compare them without that field, and assert the field's behaviour separately.
    """
    return [line for line in text.splitlines() if '"next_id"' not in line]


def shoelace(ring):
    """Signed area of a ring of (x, y) tuples. Positive when counter-clockwise."""
    total = 0.0
    for i in range(len(ring)):
        ax, ay = ring[i]
        bx, by = ring[(i + 1) % len(ring)]
        total += ax * by - bx * ay
    return total * 0.5


# ============================================================================
# Fixtures
# ============================================================================

# A rectangle of four residential ways sharing their corner nodes, plus one
# building inside it. Four ways rather than one closed way, because the graph
# finds junctions by shared node IDENTITY and four separate ways make the four
# corners four graph nodes of degree two -- which is the smallest graph with
# exactly one bounded face.
_SQUARE_OSM = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6" generator="stratum-python-test">
  <bounds minlat="53.3400" minlon="-6.2600" maxlat="53.3410" maxlon="-6.2580"/>

  <node id="1" version="1" lat="53.3400" lon="-6.2600"/>
  <node id="2" version="1" lat="53.3400" lon="-6.2580"/>
  <node id="3" version="1" lat="53.3410" lon="-6.2580"/>
  <node id="4" version="1" lat="53.3410" lon="-6.2600"/>

  <node id="11" version="1" lat="53.3403" lon="-6.2595"/>
  <node id="12" version="1" lat="53.3403" lon="-6.2590"/>
  <node id="13" version="1" lat="53.3406" lon="-6.2590"/>
  <node id="14" version="1" lat="53.3406" lon="-6.2595"/>

  <way id="101" version="1">
    <nd ref="1"/><nd ref="2"/>
    <tag k="highway" v="residential"/>
    <tag k="name" v="South Side"/>
    <tag k="lanes" v="2"/>
    <tag k="surface" v="asphalt"/>
  </way>
  <way id="102" version="1">
    <nd ref="2"/><nd ref="3"/>
    <tag k="highway" v="residential"/>
    <tag k="name" v="East Side"/>
  </way>
  <way id="103" version="1">
    <nd ref="3"/><nd ref="4"/>
    <tag k="highway" v="residential"/>
    <tag k="name" v="North Side"/>
  </way>
  <way id="104" version="1">
    <nd ref="4"/><nd ref="1"/>
    <tag k="highway" v="residential"/>
    <tag k="name" v="West Side"/>
  </way>

  <way id="200" version="1">
    <nd ref="11"/><nd ref="12"/><nd ref="13"/><nd ref="14"/><nd ref="11"/>
    <tag k="building" v="house"/>
    <tag k="name" v="Test House"/>
    <tag k="building:levels" v="4"/>
  </way>
</osm>
"""

_VALID_RULES = (
    'version "0.1"\n'
    "@range(2.0, 40.0)\n"
    "attr height : float = 12.0\n"
    "const floor_height : float = 3.2\n"
    "@start\n"
    "rule Lot(setback_m : float = 2.0) {\n"
    "  setback(setback_m);\n"
    "  extrude(height);\n"
    "}\n"
)

_TMP_DIR = None
_SQUARE_PATH = None
_SQUARE_DATA = None


def tmp_path(name):
    return os.path.join(_TMP_DIR, name)


def square_data():
    """The parsed fixture, parsed once. Read-only for every test that uses it."""
    return _SQUARE_DATA


def square_block():
    graph = stratum.RoadGraph()
    graph.build(square_data())
    return stratum.extract_blocks(graph).block(0)


# ============================================================================
# Module surface
# ============================================================================


@test
def module_reports_a_version_string():
    check(isinstance(stratum.__version__, str), "__version__ must be a str")
    check(len(stratum.__version__) > 0, "__version__ must not be empty")


@test
def stratum_error_is_catchable_as_runtime_error():
    # A script that catches RuntimeError must catch ours, or every existing
    # `except RuntimeError` in a user's code silently stops working.
    check(
        issubclass(stratum.StratumError, RuntimeError),
        "StratumError must derive from RuntimeError",
    )


@test
def the_module_exposes_the_documented_surface():
    for name in (
        "parse_osm",
        "Document",
        "RoadGraph",
        "extract_blocks",
        "subdivide_block",
        "export_scene",
        "build_building_mesh",
        "parse_rules",
        "StratumError",
    ):
        check(hasattr(stratum, name), "stratum.{} is missing".format(name))


# ============================================================================
# OSM import
# ============================================================================


@test
def parse_osm_of_a_missing_file_raises_file_not_found():
    # FileNotFoundError and not StratumError: `except FileNotFoundError` is what
    # a script already writes, and a library that needs its own type for this is
    # a library that breaks the idiom.
    check_raises(
        FileNotFoundError,
        lambda: stratum.parse_osm(tmp_path("does_not_exist.osm")),
        "no such file",
    )


@test
def parse_osm_of_a_non_osm_file_raises_stratum_error_with_a_message():
    path = tmp_path("garbage.osm")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("this is not XML at all\n" * 10)
    exc = check_raises(stratum.StratumError, lambda: stratum.parse_osm(path))
    check(len(str(exc)) > len("parse_osm: "), "the refusal must carry a real message")


@test
def parse_osm_counts_roads_buildings_and_raw_elements():
    data = square_data()
    check_eq(data.road_count, 4, "four ways tagged highway")
    check_eq(data.building_count, 1, "one way tagged building")
    check_eq(data.area_count, 0)
    check_eq(data.node_count, 8, "four road corners plus four building corners")
    check_eq(data.way_count, 5)
    check_eq(data.stats.processed_roads, 4)
    check_eq(data.stats.processed_buildings, 1)


@test
def parsed_data_is_owned_by_python_and_survives_the_parser():
    # There is no parser object to outlive: OSMParser is not bound, exactly so
    # that this cannot be got wrong. Re-parsing and dropping the result proves
    # the returned value does not alias anything the call left behind.
    local = stratum.parse_osm(_SQUARE_PATH)
    roads = local.roads()
    del local
    gc.collect()
    check_eq(len(roads), 4, "roads must survive the ParsedOSMData they came from")
    check_eq(len(roads[0].polyline()), 2)


@test
def road_accessors_are_copies_not_borrows():
    data = square_data()
    first = data.roads()
    second = data.roads()
    check(first is not second, "roads() must return a fresh list each call")
    check_eq(len(first), len(second))


@test
def road_polyline_points_are_plain_tuples():
    road = square_data().roads()[0]
    point = road.polyline()[0]
    check(isinstance(point, tuple), "a point must be a tuple, got {}".format(type(point)))
    check_eq(len(point), 2)
    check(isinstance(point[0], float), "components must be floats")


@test
def road_carries_its_classification_and_name():
    names = sorted(road.name for road in square_data().roads())
    check_eq(names, ["East Side", "North Side", "South Side", "West Side"])
    for road in square_data().roads():
        check_eq(road.type, stratum.RoadType.RESIDENTIAL)
        check_eq(road.point_count, 2)


@test
def way_tags_reads_the_raw_tag_map_and_returns_none_for_an_unknown_way():
    data = square_data()
    tags = data.way_tags(101)
    check(isinstance(tags, dict), "way_tags must give a dict")
    check_eq(tags.get("highway"), "residential")
    check_eq(tags.get("surface"), "asphalt")
    check(data.way_tags(999999) is None, "an unknown way id must give None")


@test
def parser_config_can_turn_an_importer_off():
    config = stratum.ParserConfig()
    config.import_buildings = False
    data = stratum.parse_osm(_SQUARE_PATH, config)
    check_eq(data.building_count, 0, "import_buildings=False must drop buildings")
    check_eq(data.road_count, 4, "and must not drop roads")


@test
def bounding_box_centre_is_latitude_then_longitude():
    # The documented trap: BoundingBox::center() returns (lat, lon) in a dvec2,
    # which reads as (x, y) everywhere else. The binding names the property for
    # the order; this asserts the order is what the name says.
    centre = square_data().bounds.center_lat_lon
    check_near(centre[0], 53.3405, 1e-4, "first component must be the latitude")
    check_near(centre[1], -6.2590, 1e-4, "second component must be the longitude")


@test
def building_geometry_reads_back_as_a_closed_footprint():
    building = square_data().buildings()[0]
    check_eq(building.name, "Test House")
    check_eq(building.hole_count, 0)
    footprint = building.footprint()
    check(len(footprint) >= 4, "a rectangle needs at least four corners")
    check_near(building.height, 4 * 3.0, 0.5, "building:levels=4 at 3 m per level")


# ============================================================================
# The scene
# ============================================================================


@test
def a_new_document_is_empty_and_clean():
    doc = stratum.Document()
    check_eq(doc.layer_count, 0)
    check_eq(doc.object_count, 0)
    check(not doc.dirty, "a new document is not dirty")
    check(not doc.can_undo, "a new document has no history")
    check_eq(doc.roots(), [])


@test
def create_layer_returns_an_id_and_goes_through_the_history():
    doc = stratum.Document()
    layer = doc.create_layer(stratum.LayerKind.SHAPE, "Buildings")
    check(layer != 0, "a valid layer id is never 0")
    check_eq(doc.layer_count, 1)
    check_eq(doc.roots(), [layer])
    check(doc.can_undo, "creating a layer must be undoable")
    check_eq(doc.undo_label, "Create layer")
    check(doc.dirty, "creating a layer dirties the document")


@test
def undo_removes_the_layer_and_redo_puts_it_back_with_the_same_id():
    doc = stratum.Document()
    layer = doc.create_layer(stratum.LayerKind.GRAPH, "Streets")
    check(doc.undo(), "undo must report that it did something")
    check(not doc.has_layer(layer), "the layer is gone")
    check(doc.redo(), "redo must report that it did something")
    # The same id, not a new one. A redo that minted a fresh id would leave
    # every LayerId a script was holding pointing at nothing.
    check(doc.has_layer(layer), "redo must restore the same id")
    check_eq(doc.layer(layer).name, "Streets")


@test
def undo_on_an_empty_history_returns_false_rather_than_raising():
    # A query about emptiness is not a refused operation, so it does not raise.
    doc = stratum.Document()
    check(doc.undo() is False, "undo with no history returns False")
    check(doc.redo() is False, "redo with no history returns False")


@test
def deleting_a_group_takes_its_children_with_it_and_undo_restores_them():
    doc = stratum.Document()
    group = doc.create_layer(stratum.LayerKind.GROUP, "City")
    child = doc.create_layer(stratum.LayerKind.SHAPE, "Lots", group)
    check_eq(doc.children(group), [child])
    doc.delete_layer(group)
    check(not doc.has_layer(group), "the group is gone")
    check(not doc.has_layer(child), "and so is the child")
    check(doc.undo(), "the delete is undoable")
    check(doc.has_layer(child), "the child comes back")
    check_eq(doc.layer(child).parent, group, "and comes back under the same parent")


@test
def only_a_group_may_hold_children_and_the_refusal_names_the_parent():
    doc = stratum.Document()
    shape = doc.create_layer(stratum.LayerKind.SHAPE, "Footprints")
    check_raises(
        stratum.StratumError,
        lambda: doc.create_layer(stratum.LayerKind.SHAPE, "Nope", shape),
        "cannot hold children",
    )
    check_eq(doc.layer_count, 1, "the refused create must not leave a layer behind")


@test
def naming_a_layer_that_does_not_exist_raises_with_the_id_in_the_message():
    doc = stratum.Document()
    check_raises(stratum.StratumError, lambda: doc.delete_layer(4242), "4242")
    check_raises(stratum.StratumError, lambda: doc.rename_layer(4242, "x"), "4242")
    check_raises(stratum.StratumError, lambda: doc.set_layer_visible(4242, False), "4242")


@test
def a_reparent_that_would_make_a_layer_its_own_ancestor_is_refused():
    doc = stratum.Document()
    outer = doc.create_layer(stratum.LayerKind.GROUP, "Outer")
    inner = doc.create_layer(stratum.LayerKind.GROUP, "Inner", outer)
    check_raises(stratum.StratumError, lambda: doc.reparent_layer(outer, inner), "ancestor")
    check_eq(doc.layer(inner).parent, outer, "the tree is untouched after the refusal")


@test
def layer_is_a_snapshot_and_reading_it_after_a_delete_gives_none():
    doc = stratum.Document()
    layer = doc.create_layer(stratum.LayerKind.MODEL, "Props")
    snapshot = doc.layer(layer)
    check_eq(snapshot.name, "Props")
    doc.delete_layer(layer)
    # The snapshot still reads. If the binding handed out LayerTree::find()'s
    # pointer this line would be a use-after-free with no diagnostic.
    check_eq(snapshot.name, "Props", "a Layer snapshot outlives its layer")
    check(doc.layer(layer) is None, "but the document no longer knows the id")


@test
def visibility_and_lock_inherit_down_the_tree():
    doc = stratum.Document()
    group = doc.create_layer(stratum.LayerKind.GROUP, "Group")
    child = doc.create_layer(stratum.LayerKind.SHAPE, "Child", group)
    check(doc.effective_visible(child), "visible by default")
    doc.set_layer_visible(group, False)
    check(not doc.effective_visible(child), "a hidden ancestor hides the child")
    check(doc.layer(child).own_visible, "the child's OWN flag is untouched")
    doc.set_layer_locked(group, True)
    check(doc.effective_locked(child), "a locked ancestor locks the child")


@test
def effective_locked_is_true_for_a_stale_id_and_visible_is_false():
    # Both are the safe answer: nothing draws, and an edit tool refuses rather
    # than writing into a layer that is not there.
    doc = stratum.Document()
    check(doc.effective_locked(999), "a stale id must read as locked")
    check(not doc.effective_visible(999), "a stale id must read as invisible")


@test
def setting_a_layer_colour_to_none_restores_inheritance():
    doc = stratum.Document()
    group = doc.create_layer(stratum.LayerKind.GROUP, "Group")
    child = doc.create_layer(stratum.LayerKind.SHAPE, "Child", group)
    doc.set_layer_colour(group, (0.25, 0.5, 0.75))
    doc.set_layer_colour(child, (1.0, 0.0, 0.0))
    inherited = doc.effective_colour(child)
    check_near(inherited[0], 1.0, 1e-6)
    doc.set_layer_colour(child, None)
    inherited = doc.effective_colour(child)
    check_near(inherited[0], 0.25, 1e-6, "clearing must inherit, not go grey")
    check_near(inherited[1], 0.5, 1e-6)
    check(doc.layer(child).own_colour is None, "the own colour is cleared")


@test
def a_layer_transform_round_trips_as_tuples():
    doc = stratum.Document()
    layer = doc.create_layer(stratum.LayerKind.SHAPE, "Moved")
    transform = stratum.LayerTransform()
    check(transform.is_identity, "a fresh transform is the identity")
    transform.translation = (10.0, 0.0, -2.5)
    transform.scale = (2.0, 2.0, 2.0)
    doc.set_layer_transform(layer, transform)
    read_back = doc.layer(layer).own_transform
    check_eq(read_back.translation, (10.0, 0.0, -2.5))
    check_eq(read_back.scale, (2.0, 2.0, 2.0))
    check(not read_back.is_identity)


@test
def a_vector_argument_of_the_wrong_shape_is_a_type_error():
    # The caster must not accept a two-item sequence as a three-vector, and must
    # not accept a string of the right length either -- b"\x01\x02\x03" indexes
    # to ints and would otherwise become a perfectly plausible wrong point.
    transform = stratum.LayerTransform()

    def assign(value):
        transform.translation = value

    check_raises(TypeError, lambda: assign((1.0, 2.0)))
    check_raises(TypeError, lambda: assign((1.0, 2.0, 3.0, 4.0)))
    check_raises(TypeError, lambda: assign("abc"))
    check_raises(TypeError, lambda: assign(b"\x01\x02\x03"))
    check_raises(TypeError, lambda: assign(None))
    # And the failed assignments left the value alone.
    check_eq(transform.translation, (0.0, 0.0, 0.0))


@test
def renames_of_one_layer_merge_into_a_single_undo_step_until_sealed():
    doc = stratum.Document()
    layer = doc.create_layer(stratum.LayerKind.SHAPE, "original")
    check_eq(doc.undo_depth, 1)
    doc.rename_layer(layer, "first")
    doc.rename_layer(layer, "second")
    check_eq(doc.undo_depth, 2, "two renames of one layer are one step")
    doc.undo()
    check_eq(doc.layer(layer).name, "original", "one undo takes back both renames")

    doc.redo()
    doc.seal()
    doc.rename_layer(layer, "third")
    check_eq(doc.undo_depth, 3, "seal() ends the gesture")
    doc.undo()
    check_eq(doc.layer(layer).name, "second")


@test
def a_transaction_groups_its_edits_into_one_undo_step():
    doc = stratum.Document()
    doc.begin_transaction("Build district")
    check(doc.in_transaction)
    doc.create_layer(stratum.LayerKind.GROUP, "District")
    doc.create_layer(stratum.LayerKind.SHAPE, "Lots")
    doc.create_layer(stratum.LayerKind.GRAPH, "Streets")
    doc.commit_transaction()
    check(not doc.in_transaction)
    check_eq(doc.undo_depth, 1, "three creates, one step")
    check_eq(doc.layer_count, 3)
    doc.undo()
    check_eq(doc.layer_count, 0, "one undo takes the whole group back")


@test
def aborting_a_transaction_reverts_what_it_applied_and_records_nothing():
    doc = stratum.Document()
    doc.begin_transaction("Half a thing")
    doc.create_layer(stratum.LayerKind.GROUP, "Doomed")
    doc.abort_transaction()
    check_eq(doc.layer_count, 0, "the abort reverted the create")
    check_eq(doc.undo_depth, 0, "and left no step behind")
    check(not doc.can_undo)


# ============================================================================
# Objects and attributes
# ============================================================================


@test
def an_object_remembers_which_layer_it_is_in():
    doc = stratum.Document()
    layer = doc.create_layer(stratum.LayerKind.SHAPE, "Lots")
    obj = doc.create_object(layer)
    check(obj.valid)
    check_eq(doc.object_layer(obj), layer)
    check_eq(doc.object_count, 1)
    check_eq(doc.objects(), [obj])


@test
def a_destroyed_object_handle_is_stale_and_every_use_of_it_raises():
    doc = stratum.Document()
    obj = doc.create_object()
    doc.destroy_object(obj)
    check_eq(doc.object_count, 0)
    check_raises(stratum.StratumError, lambda: doc.destroy_object(obj), "stale")
    check_raises(stratum.StratumError, lambda: doc.get_attribute(obj, "h"), "stale")
    check_raises(stratum.StratumError, lambda: doc.set_attribute(obj, "h", 1.0), "stale")


@test
def a_recycled_slot_does_not_answer_to_the_old_handle():
    doc = stratum.Document()
    first = doc.create_object()
    doc.destroy_object(first)
    second = doc.create_object()
    check_eq(second.index, first.index, "the slot is reused")
    check(second.generation != first.generation, "but the generation moved on")
    check(first != second, "so the old handle is not the new object")
    check_raises(stratum.StratumError, lambda: doc.get_attribute(first, "h"), "stale")


@test
def a_bool_attribute_stays_a_bool_and_does_not_become_a_float():
    # In Python `bool` is a subclass of `int`, so a converter that tested for a
    # number before testing for a bool would store True as 1.0. It would read
    # back as a float, resolve_as(Bool) would report TypeMismatch, and the rule
    # that asked for a boolean would silently take its fallback.
    doc = stratum.Document()
    obj = doc.create_object()
    doc.set_attribute(obj, "visible", True)
    value = doc.get_attribute(obj, "visible")
    check(isinstance(value, bool), "expected bool, got {}".format(type(value).__name__))
    check_eq(value, True)

    doc.set_attribute(obj, "height", 1.0)
    check(
        not isinstance(doc.get_attribute(obj, "height"), bool),
        "1.0 must not come back as a bool",
    )


@test
def every_attribute_type_round_trips():
    doc = stratum.Document()
    obj = doc.create_object()
    cases = {
        "b": True,
        "d": 12.5,
        "s": "brick",
        "ba": [True, False, True],
        "da": [1.0, 2.5, -3.0],
        "sa": ["brick", "glass"],
    }
    for name, value in cases.items():
        doc.set_attribute(obj, name, value)
    for name, value in cases.items():
        check_eq(doc.get_attribute(obj, name), value, "attribute " + name)


@test
def an_int_is_stored_as_a_double_because_the_store_has_no_int_type():
    doc = stratum.Document()
    obj = doc.create_object()
    doc.set_attribute(obj, "levels", 4)
    value = doc.get_attribute(obj, "levels")
    check(isinstance(value, float), "an int becomes a double")
    check_eq(value, 4.0)


@test
def an_empty_list_attribute_is_refused_rather_than_guessed_at():
    doc = stratum.Document()
    obj = doc.create_object()
    check_raises(
        ValueError,
        lambda: doc.set_attribute(obj, "empty", []),
        "no element type",
    )


@test
def a_mixed_list_attribute_is_refused_and_names_the_element():
    doc = stratum.Document()
    obj = doc.create_object()
    check_raises(
        TypeError, lambda: doc.set_attribute(obj, "mixed", [1.0, "two"]), "element 1"
    )
    check_raises(
        TypeError, lambda: doc.set_attribute(obj, "mixed", [True, 2.0]), "element 1"
    )
    check_raises(TypeError, lambda: doc.set_attribute(obj, "bad", {"a": 1}))


@test
def attribute_resolution_walks_user_then_object_then_layer_then_default():
    doc = stratum.Document()
    layer = doc.create_layer(stratum.LayerKind.SHAPE, "Lots")
    obj = doc.create_object(layer)

    check(doc.get_attribute(obj, "height") is None, "nothing yet")
    check_eq(doc.attribute_source(obj, "height"), stratum.AttributeSource.NONE)

    doc.set_default_attribute("height", 1.0)
    check_eq(doc.get_attribute(obj, "height"), 1.0)
    check_eq(doc.attribute_source(obj, "height"), stratum.AttributeSource.DEFAULT)

    doc.set_layer_attribute(layer, "height", 2.0)
    check_eq(doc.get_attribute(obj, "height"), 2.0)
    check_eq(doc.attribute_source(obj, "height"), stratum.AttributeSource.LAYER)

    doc.set_attribute(obj, "height", 3.0)
    check_eq(doc.get_attribute(obj, "height"), 3.0)
    check_eq(doc.attribute_source(obj, "height"), stratum.AttributeSource.OBJECT)

    doc.set_attribute(obj, "height", 4.0, user=True)
    check_eq(doc.get_attribute(obj, "height"), 4.0)
    check_eq(doc.attribute_source(obj, "height"), stratum.AttributeSource.USER)

    # Clearing the user rung falls back to the object rung, not to nothing.
    check(doc.clear_attribute(obj, "height", user=True))
    check_eq(doc.get_attribute(obj, "height"), 3.0)


@test
def clearing_an_attribute_that_was_never_set_returns_false():
    doc = stratum.Document()
    obj = doc.create_object()
    check(doc.clear_attribute(obj, "never_set") is False)
    doc.set_attribute(obj, "here", 1.0)
    check(doc.clear_attribute(obj, "here") is True)
    check(doc.get_attribute(obj, "here") is None)


@test
def attribute_names_lists_what_resolves_on_the_object():
    doc = stratum.Document()
    layer = doc.create_layer(stratum.LayerKind.SHAPE, "Lots")
    obj = doc.create_object(layer)
    doc.set_attribute(obj, "height", 1.0)
    doc.set_layer_attribute(layer, "colour", "red")
    names = sorted(doc.attribute_names(obj))
    check_eq(names, ["colour", "height"])


@test
def an_attribute_write_is_undoable_and_restores_the_previous_value():
    doc = stratum.Document()
    obj = doc.create_object()
    doc.set_attribute(obj, "height", 10.0)
    doc.seal()
    doc.set_attribute(obj, "height", 20.0)
    check_eq(doc.get_attribute(obj, "height"), 20.0)
    doc.undo()
    check_eq(doc.get_attribute(obj, "height"), 10.0, "undo restores the old value")
    doc.undo()
    check(doc.get_attribute(obj, "height") is None, "undo of the first write clears it")


# ============================================================================
# Save and load
# ============================================================================


@test
def a_document_round_trips_through_a_file():
    doc = stratum.Document()
    group = doc.create_layer(stratum.LayerKind.GROUP, "City")
    lots = doc.create_layer(stratum.LayerKind.SHAPE, "Lots", group)
    obj = doc.create_object(lots)
    doc.set_attribute(obj, "height", 18.0)
    doc.set_attribute(obj, "material", "brick", user=True)

    path = tmp_path("roundtrip.stratum")
    doc.save(path)
    check(os.path.exists(path), "save must write the file")
    check(not doc.dirty, "save clears dirty")

    loaded = stratum.Document()
    loaded.load(path)
    check_eq(loaded.layer_count, 2)
    check_eq(loaded.layer(group).name, "City", "layer ids survive a round trip")
    check_eq(loaded.layer(lots).parent, group)
    check_eq(loaded.object_count, 1)
    restored = loaded.objects()[0]
    check_eq(restored, obj, "object handles survive, generation included")
    check_eq(loaded.get_attribute(restored, "height"), 18.0)
    check_eq(loaded.get_attribute(restored, "material"), "brick")
    check_eq(
        loaded.attribute_source(restored, "material"),
        stratum.AttributeSource.USER,
        "the rung a value was written on survives too",
    )


@test
def loading_clears_the_undo_history():
    # Not optional. A command holds a raw pointer into the tree it edits, and
    # after a load that tree holds different content, so undoing a pre-load edit
    # would apply an inverse to a scene that never had the original.
    doc = stratum.Document()
    doc.create_layer(stratum.LayerKind.SHAPE, "Before")
    path = tmp_path("history.stratum")
    doc.save(path)
    doc.create_layer(stratum.LayerKind.SHAPE, "After")
    check(doc.can_undo)
    doc.load(path)
    check(not doc.can_undo, "the history must be empty after a load")
    check(not doc.dirty, "and a freshly loaded document is not dirty")


@test
def to_json_and_from_json_round_trip_without_touching_the_disk():
    doc = stratum.Document()
    doc.create_layer(stratum.LayerKind.GRAPH, "Streets")
    text = doc.to_json()
    check(isinstance(text, str))
    check("stratum-document" in text, "the format tag identifies the file")

    other = stratum.Document()
    other.from_json(text)
    check_eq(other.layer_count, 1)
    check_eq(other.to_json(), text, "save -> load -> save must be byte-identical")


@test
def rendering_a_document_twice_differs_only_in_the_layer_allocator():
    # Asserted rather than assumed, because without_next_id() above relies on it
    # and a helper that quietly hides a real difference would disarm three tests
    # at once.
    doc = stratum.Document()
    doc.create_layer(stratum.LayerKind.SHAPE, "Only")
    first = doc.to_json()
    second = doc.to_json()
    check(first != second, "to_json() takes a layer id, so the two differ")
    check_eq(
        without_next_id(first),
        without_next_id(second),
        "and next_id is the ONLY thing that differs",
    )


@test
def loading_a_missing_file_raises_file_not_found():
    doc = stratum.Document()
    check_raises(
        FileNotFoundError, lambda: doc.load(tmp_path("nope.stratum")), "no such file"
    )


@test
def a_failed_load_leaves_the_document_exactly_as_it_was():
    doc = stratum.Document()
    layer = doc.create_layer(stratum.LayerKind.SHAPE, "Keep me")
    before = doc.to_json()

    path = tmp_path("corrupt.stratum")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write('{"format": "not-a-stratum-document"}')

    exc = check_raises(stratum.StratumError, lambda: doc.load(path))
    check(len(str(exc)) > len("load: "), "the refusal must say what was wrong")
    check_eq(doc.layer_count, 1, "the document is untouched")
    check_eq(doc.layer(layer).name, "Keep me")
    check(doc.can_undo, "down to its undo history")
    check_eq(
        without_next_id(doc.to_json()),
        without_next_id(before),
        "down to the last byte -- see without_next_id() for the one field that moves",
    )


@test
def from_json_of_nonsense_raises_and_names_the_problem():
    doc = stratum.Document()
    check_raises(stratum.StratumError, lambda: doc.from_json("{"), "from_json")
    check_raises(stratum.StratumError, lambda: doc.from_json(""), "from_json")


# ============================================================================
# Street graph, blocks and lots
# ============================================================================


@test
def a_road_graph_finds_junctions_by_shared_node_identity():
    graph = stratum.RoadGraph()
    graph.build(square_data())
    check_eq(graph.node_count, 4, "four shared corners, four graph nodes")
    check_eq(graph.edge_count, 4)
    stats = graph.stats()
    check_eq(stats.junctions, 0, "a corner of degree two is not a junction")
    check_eq(stats.continuations, 4)
    check_eq(stats.dead_ends, 0)


@test
def graph_edges_and_nodes_are_copies_and_index_out_of_range_raises():
    graph = stratum.RoadGraph()
    graph.build(square_data())
    edge = graph.edge(0)
    check(edge.length > 0.0)
    check_eq(len(edge.polyline()), 2)
    node = graph.node(0)
    check_eq(node.degree, 2)
    check(isinstance(node.position, tuple))
    check_raises(IndexError, lambda: graph.edge(99))
    check_raises(IndexError, lambda: graph.node(99))

    # The copies outlive the graph they came from.
    graph.clear()
    check_eq(graph.edge_count, 0)
    check_eq(len(edge.polyline()), 2, "a GraphEdge copy survives graph.clear()")


@test
def a_rectangle_of_four_ways_yields_exactly_one_block():
    graph = stratum.RoadGraph()
    graph.build(square_data())
    extraction = stratum.extract_blocks(graph)
    check_eq(extraction.block_count, 1, "one bounded face")
    stats = extraction.stats
    check_eq(stats.faces, 2, "the bounded face and the outer one")
    check_eq(stats.outer_faces, 1, "and the outer one is discarded")
    check_eq(stats.tree_faces, 0)
    check_eq(stats.zero_area_faces, 0)
    check_eq(stats.malformed_faces, 0)
    check_eq(stats.rejected_small, 0)
    check_eq(stats.faces, stats.expected_faces, "Euler's formula must agree")


@test
def the_block_ring_is_counter_clockwise_and_its_area_matches_a_shoelace():
    # Cross-checked against a shoelace computed here, so the assertion does not
    # depend on the projection or on the fixture's latitude. A traversal that
    # kept the OUTER face would give the same corners and a negative area.
    block = square_block()
    ring = block.ring()
    check_eq(len(ring), 4)
    area = shoelace(ring)
    check(area > 0.0, "a kept face must be counter-clockwise, got area {}".format(area))
    check_near(block.area, area, abs(area) * 1e-9, "Block.area must be the ring's area")
    check_near(
        stratum.signed_ring_area(ring),
        area,
        abs(area) * 1e-9,
        "signed_ring_area must agree with a shoelace",
    )
    check_eq(block.hole_count, 0)
    check_eq(block.edge_count, 4)
    check(not block.has_grade_separated_edge)
    check(block.perimeter > 0.0)


@test
def a_block_out_of_range_raises_index_error():
    extraction = stratum.extract_blocks(stratum.RoadGraph())
    check_eq(extraction.block_count, 0, "an empty graph has no faces")
    check_raises(IndexError, lambda: extraction.block(0))


@test
def subdividing_a_block_produces_lots_that_tile_it():
    block = square_block()
    params = stratum.LotParams()
    params.lot_area_min = block.area / 12.0
    params.lot_area_max = block.area / 4.0
    params.seed = 42
    subdivision = stratum.subdivide_block(block, params)

    check(subdivision.lot_count >= 2, "a big block must split")
    check_eq(subdivision.stats.lots, subdivision.lot_count)
    lots = subdivision.lots()
    total = sum(lot.area for lot in lots)
    for lot in lots:
        check(lot.area > 0.0, "a lot with no area is not a lot")
        check(len(lot.ring()) >= 3, "a lot ring needs at least three corners")
    # The lots partition the block: they cannot add up to more than it, and a
    # subdivision that lost a piece would come out well under.
    check(total <= block.area * (1.0 + 1e-9), "lots cannot exceed the block")
    check_near(total, block.area, block.area * 0.02, "and must very nearly fill it")


@test
def subdivision_is_deterministic_for_the_same_seed():
    block = square_block()

    def rings(seed):
        params = stratum.LotParams()
        params.lot_area_min = block.area / 12.0
        params.lot_area_max = block.area / 4.0
        params.seed = seed
        return [lot.ring() for lot in stratum.subdivide_block(block, params).lots()]

    check_eq(rings(7), rings(7), "the same seed must give the same lots")


@test
def lot_ids_are_stable_and_unique_within_a_subdivision():
    block = square_block()
    params = stratum.LotParams()
    params.lot_area_min = block.area / 12.0
    params.seed = 3
    lots = stratum.subdivide_block(block, params).lots()
    ids = [lot.id for lot in lots]
    check_eq(len(set(ids)), len(ids), "lot ids must be unique")
    check(all(lot_id != 0 for lot_id in ids), "0 is the invalid lot id")


# ============================================================================
# Meshes and export
# ============================================================================


@test
def a_building_footprint_becomes_a_mesh():
    building = square_data().buildings()[0]
    mesh = stratum.build_building_mesh(building)
    check(mesh.is_valid, "a rectangle must produce geometry")
    check(mesh.vertex_count > 0)
    check(mesh.triangle_count > 0)
    check_eq(mesh.index_count, mesh.triangle_count * 3)
    positions = mesh.positions()
    check_eq(len(positions), mesh.vertex_count)
    check_eq(len(positions[0]), 3)


@test
def a_scene_object_owns_its_mesh_and_outlives_it():
    # The whole reason stratum.SceneObject is not osm::SceneObject. That type
    # holds `const Mesh*` and says the mesh must outlive the export; a script
    # cannot promise that, so the bound type copies.
    building = square_data().buildings()[0]
    mesh = stratum.build_building_mesh(building)
    triangles = mesh.triangle_count

    obj = stratum.scene_object_for_building(building, mesh)
    del mesh
    gc.collect()

    check_eq(obj.mesh.triangle_count, triangles, "the object kept its own copy")
    check_eq(obj.kind, stratum.SceneObjectKind.BUILDING)
    check_eq(obj.osm_id, 200)
    check(len(obj.name) > 0, "describe_building names the object")


@test
def export_scene_writes_files_and_reports_what_it_wrote():
    building = square_data().buildings()[0]
    mesh = stratum.build_building_mesh(building)
    obj = stratum.scene_object_for_building(building, mesh)

    out_dir = tmp_path("export_scene_out")
    stats = stratum.export_scene([obj], out_dir)
    check(stats.files >= 1, "at least one file")
    check_eq(len(stats.written_files), stats.files)
    check(stats.triangles > 0)
    check_eq(stats.objects, 1)
    check_eq(stats.unwritten_triangles, 0, "every routed triangle must reach a file")
    for name in stats.written_files:
        check(
            os.path.exists(os.path.join(out_dir, name)),
            "written_files names a file that is not there: " + name,
        )


@test
def export_scene_rejects_a_list_of_the_wrong_thing():
    out_dir = tmp_path("export_reject_out")
    check_raises(
        TypeError, lambda: stratum.export_scene([1, 2, 3], out_dir), "element 0"
    )
    check_raises(TypeError, lambda: stratum.export_scene([None], out_dir), "element 0")
    check(
        not os.path.exists(out_dir),
        "a refused export must not have created the directory",
    )


@test
def export_scene_of_nothing_writes_nothing_and_does_not_raise():
    stats = stratum.export_scene([], tmp_path("export_empty_out"))
    check_eq(stats.files, 0)
    check_eq(stats.objects, 0)


@test
def export_scene_object_writes_one_named_file():
    building = square_data().buildings()[0]
    mesh = stratum.build_building_mesh(building)
    obj = stratum.scene_object_for_building(building, mesh)

    out_path = tmp_path("single.obj")
    stratum.export_scene_object(obj, out_path)
    check(os.path.exists(out_path), "the file must be where we asked for it")
    with open(out_path, "r", encoding="utf-8") as handle:
        text = handle.read()
    check(
        any(line.startswith("v ") for line in text.splitlines()),
        "the .obj must carry vertex lines",
    )


@test
def a_scene_object_can_be_built_by_hand_from_a_mesh():
    building = square_data().buildings()[0]
    mesh = stratum.build_building_mesh(building)
    obj = stratum.SceneObject(
        mesh, stratum.SceneObjectKind.AREA, "Hand made", 7, "custom_layer"
    )
    check_eq(obj.name, "Hand made")
    check_eq(obj.osm_id, 7)
    check_eq(obj.layer, "custom_layer")
    check_eq(obj.kind, stratum.SceneObjectKind.AREA)
    obj.name = "Renamed"
    check_eq(obj.name, "Renamed", "the labels are writable")


# ============================================================================
# Rule language
# ============================================================================


@test
def parse_rules_accepts_a_valid_file_and_names_what_is_in_it():
    summary = stratum.parse_rules(_VALID_RULES, "test.srl")
    check(summary.ok, "a valid file must parse: " + summary.report)
    check_eq(summary.error_count, 0)
    check_eq(summary.version, "0.1")
    check_eq(summary.rule_names, ["Lot"])
    check_eq(summary.start_rule, "Lot")
    check_eq(summary.attribute_names, ["height"])


@test
def parse_rules_reports_errors_as_data_rather_than_raising():
    # A parse failure is a result, not an exception: a linter wants every
    # diagnostic, and raising on the first one gives it exactly one.
    summary = stratum.parse_rules("rule { this is not a rule }", "broken.srl")
    check(not summary.ok, "a broken file must not parse")
    check(summary.error_count > 0, "and must produce at least one error")
    check(len(summary.messages) >= summary.error_count)
    check("broken.srl" in summary.report, "the report names the file")


@test
def a_library_file_with_no_start_rule_reports_an_empty_start_rule():
    summary = stratum.parse_rules("rule A { trim(); }\nrule B { trim(); }\n", "lib.srl")
    check(summary.ok, summary.report)
    check_eq(summary.rule_names, ["A", "B"])
    check_eq(summary.start_rule, "", "a library file has no start rule")


# ============================================================================
# Runner
# ============================================================================


def _setup():
    global _TMP_DIR, _SQUARE_PATH, _SQUARE_DATA
    _TMP_DIR = tempfile.mkdtemp(prefix="stratum_py_test_")
    _SQUARE_PATH = os.path.join(_TMP_DIR, "square.osm")
    with open(_SQUARE_PATH, "w", encoding="utf-8") as handle:
        handle.write(_SQUARE_OSM)
    _SQUARE_DATA = stratum.parse_osm(_SQUARE_PATH)


def _teardown():
    if _TMP_DIR is not None:
        shutil.rmtree(_TMP_DIR, ignore_errors=True)


def run_all():
    """Run every registered test and report what happened.

    Returns a dict with ``run``, ``passed`` and ``failures``. The C++ driver
    checks all three: a run count of zero has to FAIL the suite, because a
    harness that silently registers nothing looks exactly like a pass.
    """
    failures = []
    passed = 0

    try:
        _setup()
    except BaseException:  # noqa: BLE001 - a broken fixture must be reported, not hidden
        return {
            "run": 0,
            "passed": 0,
            "failures": ["<setup>: " + traceback.format_exc()],
        }

    try:
        for fn in _TESTS:
            try:
                fn()
                passed += 1
            except BaseException:  # noqa: BLE001 - one failure must not stop the rest
                failures.append("{}: {}".format(fn.__name__, traceback.format_exc()))
    finally:
        _teardown()

    return {"run": len(_TESTS), "passed": passed, "failures": failures}
