/**
 * @file framework.hpp
 * @brief Minimal dependency-free unit test framework for stratum_core
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Nothing is vendored for testing and no submodule is being added, so this is a
 * deliberately small hand-rolled framework: self-registering test cases, a
 * handful of non-fatal check macros, and a runner with an optional suite filter.
 *
 * Usage:
 * @code
 *     #include "framework.hpp"
 *     #include "osm/road/road_graph.hpp"
 *
 *     TEST(RoadGraph, t_junction_on_interior_node_has_degree_3) {
 *         stratum::osm::road::RoadGraph graph;
 *         graph.build(make_t_junction());
 *         CHECK_EQ(graph.stats().junctions, size_t{1});
 *         CHECK_TRUE(graph.node(0).is_junction());
 *         CHECK_NEAR(graph.edge(0).length(), 100.0, 1e-9);
 *     }
 * @endcode
 *
 * Running:
 * @code
 *     ./stratum_tests            # every test
 *     ./stratum_tests RoadGraph  # only tests whose suite is exactly "RoadGraph"
 * @endcode
 *
 * The executable exits 0 when every check passed and non-zero otherwise.
 *
 * Checks are non-fatal: a failing CHECK reports and lets the test body continue,
 * so one run reports every failure rather than only the first. A test that must
 * stop early should `return` after checking a precondition.
 *
 * This header is complete except for the registry storage, the runner body, and
 * main(), which all live in the single translation unit tests/framework.cpp.
 * Test files include this header only and define no main() of their own.
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <iosfwd>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

namespace stratum::test {

// ============================================================================
// Registration
// ============================================================================

/**
 * @brief One registered test case
 *
 * The string pointers are literals produced by the TEST macro and have static
 * storage duration, so the registry stores them without copying.
 */
struct TestCase {
    const char* suite = "";     ///< Suite name, the first TEST argument
    const char* name = "";      ///< Test name, the second TEST argument
    const char* file = "";      ///< File the test was defined in
    int line = 0;               ///< Line the test was defined on
    void (*fn)() = nullptr;     ///< Test body
};

/**
 * @brief Add a test case to the global registry
 *
 * Called during static initialisation by the TEST macro. Defined in
 * framework.cpp, which owns the registry storage.
 *
 * @param test_case Case to register; its string pointers must outlive the run
 */
void register_test(const TestCase& test_case);

/**
 * @brief Record a failed check against the test currently running
 *
 * Prints `file:line: FAILED: expr` followed by @p detail when it is non-empty,
 * increments the failure count, and returns. Defined in framework.cpp.
 *
 * @param file   Source file of the failing check
 * @param line   Line of the failing check
 * @param expr   Stringified expression that failed
 * @param detail Extra context such as actual and expected values; may be empty
 */
void report_failure(const char* file, int line, const char* expr, const std::string& detail);

/**
 * @brief Run the registered tests and print a summary
 *
 * Defined in framework.cpp, which also defines the only main() in the test
 * binary as `int main(int argc, char** argv) { return run_all(argc, argv); }`.
 *
 * @param argc Argument count as passed to main
 * @param argv Argument vector. argv[1], when present, is a suite name filter:
 *             only tests whose suite matches it exactly are run. Further
 *             arguments are ignored.
 * @return 0 when every check in every run test passed, non-zero otherwise. Also
 *         non-zero when a suite filter matched no test at all, since a filter
 *         that silently runs nothing reads as a pass.
 */
int run_all(int argc, char** argv);

/**
 * @brief Static-initialisation helper that registers one test case
 *
 * Not used directly; the TEST macro instantiates it.
 */
struct Registrar {
    explicit Registrar(const TestCase& test_case) { register_test(test_case); }
};

// ============================================================================
// Value Stringification
// ============================================================================

namespace detail {

/// Trait: true when `std::ostream << T` compiles
template <typename T, typename = void>
struct is_streamable : std::false_type {};

template <typename T>
struct is_streamable<T, std::void_t<decltype(std::declval<std::ostream&>()
                                             << std::declval<const T&>())>>
    : std::true_type {};

} // namespace detail

/**
 * @brief Render a value for a failure message
 *
 * Streamable types are formatted with `operator<<`. bool is spelled "true" and
 * "false" rather than 1 and 0. Anything else renders as "<unprintable>", which
 * still leaves the stringified expression and the file:line to work from.
 *
 * @tparam T Type of the value
 * @param value Value to render
 * @return Human-readable rendering of @p value
 */
template <typename T>
[[nodiscard]] std::string stringify(const T& value) {
    if constexpr (std::is_same_v<std::decay_t<T>, bool>) {
        return value ? "true" : "false";
    } else if constexpr (detail::is_streamable<T>::value) {
        std::ostringstream out;
        out << value;
        return out.str();
    } else {
        return "<unprintable>";
    }
}

} // namespace stratum::test

// ============================================================================
// Test Definition Macro
// ============================================================================

/// @cond INTERNAL
#define STRATUM_TEST_CONCAT_INNER(a, b) a##b
#define STRATUM_TEST_CONCAT(a, b) STRATUM_TEST_CONCAT_INNER(a, b)
/// @endcond

/**
 * @brief Define and self-register a test case
 *
 * Expands to a function definition, so it is followed by a body:
 * `TEST(Miter, offset_scales_with_half_angle) { ... }`.
 *
 * @param suite_name Suite identifier, also the argv[1] filter value. Must be a
 *                   bare identifier, not a string.
 * @param test_name  Test identifier, unique within the suite. Must be a bare
 *                   identifier.
 *
 * @note Suite and test names are pasted into a function name, so the pair must be
 *       unique across the whole test binary.
 */
#define TEST(suite_name, test_name)                                                  \
    static void STRATUM_TEST_CONCAT(suite_name, STRATUM_TEST_CONCAT(_, test_name))();\
    namespace {                                                                      \
    const ::stratum::test::Registrar STRATUM_TEST_CONCAT(                             \
        stratum_test_registrar_,                                                     \
        STRATUM_TEST_CONCAT(suite_name, STRATUM_TEST_CONCAT(_, test_name)))(         \
        ::stratum::test::TestCase{                                                   \
            #suite_name, #test_name, __FILE__, __LINE__,                             \
            &STRATUM_TEST_CONCAT(suite_name, STRATUM_TEST_CONCAT(_, test_name))});   \
    }                                                                                \
    static void STRATUM_TEST_CONCAT(suite_name, STRATUM_TEST_CONCAT(_, test_name))()

// ============================================================================
// Check Macros
//
// All checks are non-fatal: they report and let the test body continue.
// ============================================================================

/**
 * @brief Check that an expression is true
 * @param expr Expression contextually convertible to bool
 */
#define CHECK(expr)                                                                  \
    do {                                                                             \
        if (!(expr)) {                                                               \
            ::stratum::test::report_failure(__FILE__, __LINE__, #expr,                \
                                            std::string{});                          \
        }                                                                            \
    } while (false)

/**
 * @brief Check that an expression is true. Alias of CHECK, for symmetry with CHECK_FALSE.
 * @param expr Expression contextually convertible to bool
 */
#define CHECK_TRUE(expr) CHECK(expr)

/**
 * @brief Check that an expression is false
 * @param expr Expression contextually convertible to bool
 */
#define CHECK_FALSE(expr)                                                            \
    do {                                                                             \
        if ((expr)) {                                                                \
            ::stratum::test::report_failure(__FILE__, __LINE__, "!(" #expr ")",       \
                                            std::string{});                          \
        }                                                                            \
    } while (false)

/**
 * @brief Check that two values compare equal with `operator==`
 *
 * Both operands are evaluated exactly once. The failure message reports both
 * values via stringify(), so prefer this over CHECK(a == b).
 *
 * @param a Actual value
 * @param b Expected value
 *
 * @note Mixed signedness compares as the language does and may warn. Make the
 *       types agree at the call site, for example `CHECK_EQ(v.size(), size_t{3})`.
 */
#define CHECK_EQ(a, b)                                                               \
    do {                                                                             \
        const auto& stratum_check_lhs_ = (a);                                        \
        const auto& stratum_check_rhs_ = (b);                                        \
        if (!(stratum_check_lhs_ == stratum_check_rhs_)) {                           \
            ::stratum::test::report_failure(                                          \
                __FILE__, __LINE__, #a " == " #b,                                    \
                "actual: " + ::stratum::test::stringify(stratum_check_lhs_) +         \
                    "  expected: " + ::stratum::test::stringify(stratum_check_rhs_)); \
        }                                                                            \
    } while (false)

/**
 * @brief Check that two numbers are within an absolute tolerance of each other
 *
 * Compares `std::fabs(double(a) - double(b)) <= double(eps)`. All three operands
 * are evaluated exactly once.
 *
 * @param a   Actual value, convertible to double
 * @param b   Expected value, convertible to double
 * @param eps Absolute tolerance, convertible to double
 */
#define CHECK_NEAR(a, b, eps)                                                        \
    do {                                                                             \
        const double stratum_check_lhs_ = static_cast<double>(a);                    \
        const double stratum_check_rhs_ = static_cast<double>(b);                    \
        const double stratum_check_eps_ = static_cast<double>(eps);                  \
        const double stratum_check_diff_ =                                           \
            std::fabs(stratum_check_lhs_ - stratum_check_rhs_);                      \
        if (!(stratum_check_diff_ <= stratum_check_eps_)) {                          \
            ::stratum::test::report_failure(                                          \
                __FILE__, __LINE__, #a " ~= " #b,                                    \
                "actual: " + ::stratum::test::stringify(stratum_check_lhs_) +         \
                    "  expected: " + ::stratum::test::stringify(stratum_check_rhs_) + \
                    "  diff: " + ::stratum::test::stringify(stratum_check_diff_) +    \
                    "  tolerance: " + ::stratum::test::stringify(stratum_check_eps_));\
        }                                                                            \
    } while (false)
