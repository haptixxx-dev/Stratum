/**
 * @file framework.cpp
 * @brief Registry storage, runner, and main() for the stratum test framework
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The only translation unit in the test binary that defines main(). Test files
 * include framework.hpp and define test bodies with the TEST macro; the macro's
 * static Registrar objects call register_test() during static initialisation, so
 * the registry must survive being written to before main() runs.
 */

#include "framework.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <vector>

namespace stratum::test {

namespace {

/**
 * @brief The registry, as a function-local static
 *
 * Registrars run during static initialisation in an unspecified order across
 * translation units, so a namespace-scope vector could be written to before its
 * own constructor ran. A function-local static is constructed on first use, which
 * is by definition the first registration.
 */
std::vector<TestCase>& registry() {
    static std::vector<TestCase> cases;
    return cases;
}

/// Failure count for the test currently running, reset by the runner
int g_current_failures = 0;

/// Total failing checks across the whole run
int g_total_failures = 0;

} // namespace

void register_test(const TestCase& test_case) {
    registry().push_back(test_case);
}

void report_failure(const char* file, int line, const char* expr, const std::string& detail) {
    ++g_current_failures;
    ++g_total_failures;

    std::cout << "  " << (file != nullptr ? file : "<unknown>") << ':' << line
              << ": FAILED: " << (expr != nullptr ? expr : "") << '\n';
    if (!detail.empty()) {
        std::cout << "      " << detail << '\n';
    }
    std::cout.flush();
}

int run_all(int argc, char** argv) {
    const char* suite_filter = (argc > 1) ? argv[1] : nullptr;

    // Group by suite while preserving registration order within a suite, so the
    // output reads suite by suite regardless of static initialisation order.
    std::vector<TestCase> selected;
    selected.reserve(registry().size());
    for (const auto& test_case : registry()) {
        if (suite_filter != nullptr && std::strcmp(test_case.suite, suite_filter) != 0) {
            continue;
        }
        selected.push_back(test_case);
    }
    std::stable_sort(selected.begin(), selected.end(),
                     [](const TestCase& a, const TestCase& b) {
                         return std::strcmp(a.suite, b.suite) < 0;
                     });

    if (selected.empty()) {
        if (suite_filter != nullptr) {
            std::cout << "no test matched suite filter '" << suite_filter << "'\n";
        } else {
            std::cout << "no test registered\n";
        }
        // A filter that silently runs nothing reads as a pass, so it is a failure.
        return 1;
    }

    g_total_failures = 0;
    int passed = 0;
    int failed = 0;
    const char* current_suite = nullptr;

    for (const auto& test_case : selected) {
        if (current_suite == nullptr || std::strcmp(current_suite, test_case.suite) != 0) {
            current_suite = test_case.suite;
            std::cout << "[" << current_suite << "]\n";
        }

        std::cout << "  RUN  " << test_case.name << '\n';
        std::cout.flush();

        g_current_failures = 0;
        if (test_case.fn != nullptr) {
            test_case.fn();
        } else {
            report_failure(test_case.file, test_case.line, "test body is null", std::string{});
        }

        if (g_current_failures == 0) {
            ++passed;
            std::cout << "  PASS " << test_case.name << '\n';
        } else {
            ++failed;
            std::cout << "  FAIL " << test_case.name << " (" << g_current_failures
                      << (g_current_failures == 1 ? " check)" : " checks)") << '\n';
        }
    }

    std::cout << '\n' << passed << " passed, " << failed << " failed";
    if (g_total_failures > 0) {
        std::cout << " (" << g_total_failures << " failing checks)";
    }
    std::cout << '\n';
    std::cout.flush();

    return failed == 0 ? 0 : 1;
}

} // namespace stratum::test

/**
 * @brief Test binary entry point
 *
 * @param argc Argument count
 * @param argv argv[1], when present, is a suite name filter
 * @return 0 when every check passed, non-zero otherwise
 */
int main(int argc, char** argv) {
    return stratum::test::run_all(argc, argv);
}
