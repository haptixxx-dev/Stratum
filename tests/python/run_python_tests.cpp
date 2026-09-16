// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file run_python_tests.cpp
 * @brief Driver that runs tests/python/test_bindings.py in an embedded interpreter
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The Python bindings have to be tested from Python -- a C++ test of a pybind11
 * layer tests the C++ underneath it and nothing about the conversion, the
 * ownership or the exception mapping, which is the entire feature. So the real
 * assertions live in test_bindings.py and this file is the bridge that puts them
 * into the ctest suite with everything else.
 *
 * ### Why the suite still exists when Python is off
 *
 * `STRATUM_ENABLE_PYTHON` can be OFF, and the tests CMakeLists lists this file
 * unconditionally. Without the `#else` branch below, `ctest -R PythonBindings`
 * would run a binary containing no test named PythonBindings, and framework.cpp
 * exits NON-ZERO when a filter matches nothing -- correctly, because a filter
 * that silently runs nothing reads as a pass. A build with Python off would
 * therefore fail a suite it was never asked to run. The `#else` registers one
 * test that prints a SKIPPED line and asserts nothing, exactly as the GPU suites
 * do when there is no backend.
 *
 * ### Why there is one test and not thirty
 *
 * `py::scoped_interpreter` starts and finalises CPython. Doing that once per
 * TEST would finalise and restart it thirty times in one process, and pybind11's
 * embedded-module and exception-translator registrations hold handles that do
 * not survive a finalise -- the second interpreter would read a dangling
 * PyObject* for `stratum.StratumError` on the first raise. One interpreter, one
 * test, and the granularity lives inside the Python file, whose failures are
 * reported individually below.
 *
 * ### The floor on the test count
 *
 * kMinimumPythonTests is not decoration. Every other failure mode here is loud,
 * but a test FILE that stops registering tests is silent: if `@test` stopped
 * appending, or an exception at import time truncated the list, `run_all()`
 * would return zero failures and the suite would pass while testing nothing.
 * That failure has been found in this project five times in five review rounds,
 * so the count is asserted against a floor and the floor is below the current
 * number only far enough to allow ordinary editing.
 *
 * ### registered, run, passed, failures
 *
 * Four numbers and three checks between them, and the checks are only worth
 * making because run_all() derives the numbers independently:
 *
 *   - `registered` is `len(_TESTS)`, and is what the floor is held against.
 *   - `run` counts ENTRIES into the loop body.
 *   - `passed` and `len(failures)` count the two ways out of it.
 *
 * `passed + failures == run` therefore catches a test that took the harness down
 * rather than returning, and `run == registered` catches a loop that stopped
 * short. Both were tautologies until run_all() stopped reporting `len(_TESTS)`
 * for all of them: the driver asserted arithmetic that could not come out wrong,
 * which is this project's most-found defect and was sitting in its own guard
 * against that defect.
 */

#include "framework.hpp"

#ifdef STRATUM_ENABLE_PYTHON

#include "python/module.hpp"

#include <pybind11/embed.h>
#include <pybind11/eval.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace py = pybind11;
namespace fs = std::filesystem;

namespace {

/**
 * @brief Fewest tests test_bindings.py may register before the suite fails
 *
 * See the note in the file comment. Raise it when the file grows; never lower it
 * to make a run go green.
 */
constexpr size_t kMinimumPythonTests = 76;

/**
 * @brief Where test_bindings.py might be, in the order worth trying
 *
 * STRATUM_PYTHON_TEST_DIR is what the tests CMakeLists should define, and is
 * tried first. The other two are fallbacks so that this suite works in a tree
 * where that define has not been added yet, and when the binary is run by hand
 * from tests/ rather than through ctest:
 *
 *   - STRATUM_TEST_DATA_DIR is already defined for every suite and sits at
 *     tests/data, so its sibling is tests/python.
 *   - The bare relative path covers ctest, which sets WORKING_DIRECTORY to
 *     tests/.
 *
 * A missing file is reported as a failure and not as a skip: the suite exists to
 * run that file, and "could not find it" is the loudest way this can break.
 */
[[nodiscard]] fs::path find_script() {
    std::vector<fs::path> candidates;

#ifdef STRATUM_PYTHON_TEST_DIR
    candidates.emplace_back(fs::path(STRATUM_PYTHON_TEST_DIR) / "test_bindings.py");
#endif
#ifdef STRATUM_TEST_DATA_DIR
    candidates.emplace_back(fs::path(STRATUM_TEST_DATA_DIR).parent_path() / "python"
                            / "test_bindings.py");
#endif
    candidates.emplace_back(fs::path("python") / "test_bindings.py");

    for (const fs::path& candidate : candidates) {
        std::error_code ec;
        if (fs::exists(candidate, ec) && !ec) {
            return candidate;
        }
    }
    return {};
}

/// Read one entry of the result dict, failing the test rather than throwing when
/// the Python side returned something of the wrong shape.
[[nodiscard]] size_t dict_size_t(const py::dict& result, const char* key, bool& ok) {
    if (!result.contains(key)) {
        ok = false;
        return 0;
    }
    try {
        return result[key].cast<size_t>();
    } catch (const py::cast_error&) {
        ok = false;
        return 0;
    }
}

} // namespace

/**
 * Runs every test in test_bindings.py and reports each Python failure as its own
 * framework failure, with the Python traceback attached. A single CHECK on
 * "failures is empty" would report one line for thirty broken bindings.
 */
TEST(PythonBindings, test_bindings_py_passes_in_the_embedded_interpreter) {
    const fs::path script = find_script();
    CHECK_FALSE(script.empty());
    if (script.empty()) {
        std::fprintf(stderr,
                     "PythonBindings: could not find tests/python/test_bindings.py; define "
                     "STRATUM_PYTHON_TEST_DIR or run from the tests/ directory\n");
        return;
    }

    // Before the interpreter, and load-bearing. bindings.cpp registers the
    // embedded module from a static constructor, and in a static library a
    // linker drops an archive member nothing references. Without this call the
    // object file is never extracted, the constructor never runs, and the
    // failure is ModuleNotFoundError from a build that linked cleanly.
    //
    // Deliberately NOT wrapped in a CHECK. module_linked() returns true
    // unconditionally -- its whole job is to be a symbol worth extracting -- so
    // `CHECK_TRUE(module_linked())` is a check that cannot fail, and this
    // project has found one of those in every review round it has run. The
    // import below is the assertion that actually proves the linkage.
    (void)stratum::python::module_linked();

    // One interpreter for the whole suite; see the file comment.
    py::scoped_interpreter interpreter;

    // The linkage assertion, separated from the script so that its failure reads
    // as what it is. If the archive member above was dropped, this is where it
    // surfaces, and a bare traceback out of eval_file() would not say which of
    // the two dozen things in the file went wrong.
    try {
        py::module_::import("stratum");
    } catch (const py::error_already_set& e) {
        CHECK_TRUE(false);
        std::fprintf(stderr,
                     "PythonBindings: `import stratum` failed, so the embedded module was never "
                     "registered. The usual cause is the linker dropping bindings.cpp from the "
                     "stratum_core archive; module_linked() exists to prevent exactly that.\n%s\n",
                     e.what());
        return;
    }

    size_t registered = 0;
    size_t run = 0;
    size_t passed = 0;
    std::vector<std::string> failures;
    bool shape_ok = true;

    try {
        py::dict globals;
        globals["__name__"] = "stratum_test_bindings";
        // __file__ is what puts the real path into every Python traceback. A
        // traceback pointing at "<string>" is most of the value of this suite
        // thrown away.
        globals["__file__"] = script.string();

        py::eval_file(script.string(), globals);

        if (!globals.contains("run_all")) {
            CHECK_TRUE(false);
            std::fprintf(stderr, "PythonBindings: %s defines no run_all()\n",
                         script.string().c_str());
            return;
        }

        const py::dict result = globals["run_all"]().cast<py::dict>();
        registered = dict_size_t(result, "registered", shape_ok);
        run = dict_size_t(result, "run", shape_ok);
        passed = dict_size_t(result, "passed", shape_ok);

        if (result.contains("failures")) {
            for (const py::handle item : result["failures"]) {
                failures.push_back(py::str(item).cast<std::string>());
            }
        } else {
            shape_ok = false;
        }
    } catch (const py::error_already_set& e) {
        // An exception that escaped run_all() is a broken harness or a broken
        // import, not a failed assertion, and it must not be reported as "0
        // tests failed".
        CHECK_TRUE(false);
        std::fprintf(stderr, "PythonBindings: Python raised out of the harness:\n%s\n", e.what());
        return;
    }

    CHECK_TRUE(shape_ok);

    // Say how many. The framework can only report this suite as one test, so
    // without this line a run of eighty-two Python assertions and a run of none
    // both print "1 passed" -- which is the shape of the GPU-suite defect that
    // skip_test() exists to prevent, in a place skip_test() cannot reach.
    std::printf("PythonBindings: %zu of %zu registered Python tests ran, %zu passed, %zu failed\n",
                run, registered, passed, failures.size());

    // Each Python failure gets its own framework failure line, with the
    // traceback on stderr next to it.
    for (const std::string& failure : failures) {
        CHECK_TRUE(failure.empty());
        std::fprintf(stderr, "PythonBindings failure:\n%s\n", failure.c_str());
    }

    CHECK_EQ(failures.size(), size_t{0});

    // Every test that STARTED must have finished as a pass or as a failure.
    // `run` counts entries into the loop and `passed`/`failures` count exits, so
    // this catches a test that took the harness down with it -- a bare os._exit,
    // an exception raised while a traceback was being formatted, the loop
    // breaking early. It was a tautology while run_all() reported len(_TESTS)
    // for all three; see the note on run_all().
    CHECK_EQ(passed + failures.size(), run);

    // And every test that was REGISTERED must have started. A `for` loop that
    // stopped short reports fewer runs than registrations and nothing else here
    // would notice, because the tests it never reached also never failed.
    CHECK_EQ(run, registered);
    if (run != registered) {
        std::fprintf(stderr,
                     "PythonBindings: %zu tests are registered but only %zu ran -- the harness "
                     "loop in test_bindings.py did not reach the end of _TESTS\n",
                     registered, run);
    }

    // The floor. See the file comment: a harness that registers nothing is the
    // one failure mode that otherwise looks exactly like success. Held against
    // `registered` rather than `run`, so a file that stopped registering is
    // reported as that and not as a truncated loop.
    CHECK(registered >= kMinimumPythonTests);
    if (registered < kMinimumPythonTests) {
        std::fprintf(stderr,
                     "PythonBindings: only %zu tests are registered, expected at least %zu -- has "
                     "the registration in test_bindings.py stopped working?\n",
                     registered, kMinimumPythonTests);
    }
}

#else // STRATUM_ENABLE_PYTHON

/**
 * The suite must still EXIST when Python is off, or `ctest -R PythonBindings`
 * fails on a filter that matches no test.
 *
 * skip_test(), and NOT a bare return with a message on stderr. A test that
 * asserts nothing and returns is counted as a PASS, and the run says "1 passed"
 * for a build containing no bindings at all -- the same defect that let 123 GPU
 * tests report green on a machine with no GPU for months. skip_test() makes the
 * summary say "0 passed, 0 failed, 1 skipped" and print the reason, so the two
 * builds cannot be confused for one another.
 */
TEST(PythonBindings, skipped_because_stratum_enable_python_is_off) {
    // The reason names the define rather than saying "Python is off", because
    // the likeliest cause of this branch compiling is NOT that somebody turned
    // the option off. STRATUM_ENABLE_PYTHON is attached per target: a tree that
    // puts it on stratum_editor_lib and forgets stratum_tests builds this branch
    // forever and reports one cheerful SKIP for a feature that is fully
    // implemented and fully tested. A skip that names its define can be checked.
    ::stratum::test::skip_test("STRATUM_ENABLE_PYTHON is not defined for this TARGET, so the "
                               "bindings were not compiled into it -- check that the tests "
                               "target has the definition and not only stratum_editor_lib");
}

#endif // STRATUM_ENABLE_PYTHON
