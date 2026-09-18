// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "procgen/rules/op_cleanup.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace stratum::procgen::rules {
namespace {

/**
 * @brief How parallel two unit directions must be to count as a spur or a line
 *
 * 1e-9 on the dot product, which is about 45 microradians of turn. Well inside
 * anything a survey distinguishes and well outside the rounding of a coordinate
 * transform, so a corner that is genuinely a corner is never mistaken for one
 * of these.
 *
 * NOT scaled by the caller's tolerance. A spur is a spur whether it is a
 * micrometre or ten metres long, and scaling would leave the long ones -- which
 * are exactly the corners the skeleton cannot solve.
 */
constexpr double kDirectionEpsilon = 1e-9;

[[nodiscard]] double length_of(const glm::dvec3& v) {
    return std::sqrt(glm::dot(v, v));
}

/// Remove consecutive points within @p tolerance of each other, wrapping
[[nodiscard]] uint32_t merge_close_points(std::vector<glm::dvec3>& ring, double tolerance) {
    if (ring.size() < 2) return 0;
    uint32_t removed = 0;
    std::vector<glm::dvec3> out;
    out.reserve(ring.size());
    for (const glm::dvec3& p : ring) {
        if (!out.empty() && length_of(p - out.back()) <= tolerance) {
            ++removed;
            continue;
        }
        out.push_back(p);
    }
    // The wrap-around pair. Checked last, and only while three points remain,
    // so a ring is never emptied by closing it.
    while (out.size() >= 3 && length_of(out.front() - out.back()) <= tolerance) {
        out.pop_back();
        ++removed;
    }
    ring = std::move(out);
    return removed;
}

/**
 * @brief Remove one spur or collinear vertex, if there is one
 *
 * One at a time, because removing a vertex changes its neighbours' directions
 * and can expose another behind it. The caller loops.
 *
 * @return true when a vertex was removed
 */
[[nodiscard]] bool remove_one_degenerate(std::vector<glm::dvec3>& ring,
                                         bool& was_spur) {
    const size_t n = ring.size();
    if (n < 3) return false;

    for (size_t i = 0; i < n; ++i) {
        const glm::dvec3& previous = ring[(i + n - 1) % n];
        const glm::dvec3& vertex = ring[i];
        const glm::dvec3& next = ring[(i + 1) % n];

        const glm::dvec3 in = vertex - previous;
        const glm::dvec3 out = next - vertex;
        const double in_len = length_of(in);
        const double out_len = length_of(out);
        if (!(in_len > 0.0) || !(out_len > 0.0)) {
            // A zero-length edge that survived the merge pass, which happens
            // when the tolerance is smaller than the duplicate. It still has no
            // direction, so the corner still cannot be solved.
            was_spur = true;
            ring.erase(ring.begin() + static_cast<long>(i));
            return true;
        }

        const double turn = glm::dot(in / in_len, out / out_len);
        if (turn <= -1.0 + kDirectionEpsilon) {
            // Doubles straight back: the tip of a zero-width spike. It encloses
            // no area, so dropping it changes the footprint by nothing.
            was_spur = true;
            ring.erase(ring.begin() + static_cast<long>(i));
            return true;
        }
        if (turn >= 1.0 - kDirectionEpsilon) {
            // On the straight line between its neighbours: carries no shape.
            was_spur = false;
            ring.erase(ring.begin() + static_cast<long>(i));
            return true;
        }
    }
    return false;
}

/// Clean one ring in place; false when it is left unusable
[[nodiscard]] bool clean_ring(std::vector<glm::dvec3>& ring, double tolerance,
                              CleanupReport& report) {
    report.merged_points += merge_close_points(ring, tolerance);

    // Until it stops changing. Bounded by the vertex count, since every pass
    // that does anything removes exactly one point.
    for (size_t guard = ring.size() + 1; guard > 0; --guard) {
        bool was_spur = false;
        if (!remove_one_degenerate(ring, was_spur)) break;
        if (was_spur) ++report.removed_spurs;
        else ++report.removed_collinear;
        if (ring.size() < 3) break;
    }

    if (ring.size() < 3) {
        ++report.dropped_rings;
        return false;
    }
    return true;
}

[[nodiscard]] bool op_number(OperationArgs& context, size_t index, double& out) {
    if (index >= context.args.size() || !context.args[index].is_number()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a number for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_number();
    return true;
}

void op_cleanup(OperationArgs& context) {
    double tolerance = 1e-3;
    if (!context.args.empty()) {
        if (!op_number(context, 0, tolerance)) {
            return;
        }
    }

    CleanupReport report;
    const OpResult result = cleanup_shape(context.shape, tolerance, report);
    if (!result.ok) {
        context.interpreter.fail_shape(context.loc, "'cleanup': " + result.message);
    }
}

} // namespace

OpResult cleanup_shape(Shape& shape, double tolerance, CleanupReport& report) {
    report = CleanupReport{};

    if (!(tolerance > 0.0)) {
        return OpResult::failure("the tolerance must be positive, not " +
                                 format_number(tolerance));
    }
    if (shape.geometry.faces.empty()) {
        return OpResult::failure("the shape has no faces to clean");
    }

    // Rebuilt beside the old geometry and swapped in, so a shape that turns out
    // to clean away to nothing is left exactly as it was for the diagnostic to
    // describe.
    ShapeGeometry built;

    auto read_ring = [&](const std::vector<uint32_t>& loop) {
        std::vector<glm::dvec3> points;
        points.reserve(loop.size());
        for (const uint32_t index : loop) {
            if (index < shape.geometry.positions.size()) {
                points.push_back(shape.geometry.positions[index]);
            }
        }
        return points;
    };

    auto push_ring = [&](const std::vector<glm::dvec3>& points) {
        std::vector<uint32_t> loop;
        loop.reserve(points.size());
        for (const glm::dvec3& p : points) {
            loop.push_back(static_cast<uint32_t>(built.positions.size()));
            built.positions.push_back(p);
        }
        return loop;
    };

    for (const Face& face : shape.geometry.faces) {
        std::vector<glm::dvec3> outer = read_ring(face.loop);
        if (!clean_ring(outer, tolerance, report)) {
            // The outer ring carries the face. A hole without it is meaningless.
            ++report.dropped_faces;
            continue;
        }

        Face cleaned;
        cleaned.material = face.material;
        cleaned.loop = push_ring(outer);

        for (const std::vector<uint32_t>& hole : face.holes) {
            std::vector<glm::dvec3> inner = read_ring(hole);
            if (!clean_ring(inner, tolerance, report)) {
                // A courtyard that was only ever a survey artefact. Dropping it
                // is the right answer and is counted, not silent.
                continue;
            }
            cleaned.holes.push_back(push_ring(inner));
        }
        built.faces.push_back(std::move(cleaned));
    }

    if (built.faces.empty()) {
        return OpResult::failure(
            "every face cleaned away to nothing, so there is no shape left to build on");
    }

    shape.geometry = std::move(built);
    refit_scope(shape);
    return OpResult::success();
}

void register_cleanup_operations(OperationTable& table) {
    table.register_operation("cleanup", op_cleanup);
}

const OperationTable& cleanup_operations() {
    static const OperationTable table = [] {
        OperationTable t = standard_operations();
        register_cleanup_operations(t);
        return t;
    }();
    return table;
}

} // namespace stratum::procgen::rules
