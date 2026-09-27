# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Seamus Mullan and the Stratum contributors
#
# ctest -P script for one "golden" test: run stratum_golden on one fixture and
# diff its stdout, byte for byte, against the committed tests/golden/<name>.json.
# stratum_golden's own output is documented as deterministic (no timestamps, no
# pointers, no wall-clock timings), so a mismatch here means either the fixture
# changed or a core stage's behaviour did -- never test flakiness.
#
# Required cache-style variables (set with -D on the `cmake -P` invocation):
#   GOLDEN_EXE     path to the stratum_golden binary
#   OSM_FILE       path to the .osm fixture to run it on
#   EXPECTED_JSON  path to the golden tests/golden/<name>.json to diff against

if(NOT DEFINED GOLDEN_EXE)
    message(FATAL_ERROR "compare_golden.cmake: GOLDEN_EXE not set")
endif()
if(NOT DEFINED OSM_FILE)
    message(FATAL_ERROR "compare_golden.cmake: OSM_FILE not set")
endif()
if(NOT DEFINED EXPECTED_JSON)
    message(FATAL_ERROR "compare_golden.cmake: EXPECTED_JSON not set")
endif()

if(NOT EXISTS "${EXPECTED_JSON}")
    message(FATAL_ERROR "no golden file for ${OSM_FILE}: ${EXPECTED_JSON} does not exist "
                         "-- generate it with stratum_golden --osm ${OSM_FILE} > ${EXPECTED_JSON}")
endif()

execute_process(
    COMMAND "${GOLDEN_EXE}" --osm "${OSM_FILE}"
    OUTPUT_VARIABLE ACTUAL_OUTPUT
    ERROR_VARIABLE ACTUAL_ERROR
    RESULT_VARIABLE GOLDEN_RESULT
)

if(NOT GOLDEN_RESULT EQUAL 0)
    message(FATAL_ERROR "stratum_golden exited ${GOLDEN_RESULT} on ${OSM_FILE}\n${ACTUAL_ERROR}")
endif()

file(READ "${EXPECTED_JSON}" EXPECTED_OUTPUT)

if(NOT ACTUAL_OUTPUT STREQUAL EXPECTED_OUTPUT)
    message(FATAL_ERROR
        "golden mismatch for ${OSM_FILE} against ${EXPECTED_JSON}\n"
        "--- expected ---\n${EXPECTED_OUTPUT}\n"
        "--- actual ---\n${ACTUAL_OUTPUT}\n"
        "--- end ---\n"
        "If this change is intended, regenerate with: "
        "${GOLDEN_EXE} --osm ${OSM_FILE} > ${EXPECTED_JSON}")
endif()

message(STATUS "golden OK: ${OSM_FILE}")
