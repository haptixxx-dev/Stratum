// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_op_split.cpp
 * @brief Split operations: the sizing solver, repetition, the slab cut and the AST bridge
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 * A split is the operation the language exists for, and a split that is subtly
 * wrong builds a whole city subtly wrong. Each of the following is deliberate:
 *
 *   - **Every sizing assertion is a number computed by hand, in the comment above
 *     it**, never a number read back out of the solver and compared with itself.
 *     A solver that ignored `~` entirely would still make the pieces tile the
 *     extent; only the hand-computed share catches it.
 *
 *   - **The four size kinds are constructed from the SAME literal**, exactly as
 *     the parser suite does, so an implementation that dropped the kind and used
 *     the number cannot pass by producing something plausible.
 *
 *   - **Repetition is asserted on the EQUALITY of the copies as well as on their
 *     count.** "Six bays" is satisfied by six bays of six different widths, which
 *     is the visible defect the leftover rule exists to prevent, so every bay is
 *     checked against the same hand-computed width.
 *
 *   - **Geometry is checked by ENCLOSED VOLUME, SURFACE AREA and CLOSEDNESS**,
 *     integrated over the faces, never by reading `scope.size` back. A cut that
 *     clipped the walls and forgot to cap the ends leaves every scope correct and
 *     fails on all three. A cut that capped the ends of the split as well as the
 *     interior leaves the volume correct and fails only on area.
 *
 *   - **The axis test is run on a shape whose scope is TURNED**, so that a cut
 *     made along the world axis and a cut made along the scope axis land in
 *     different places. Splitting an unrotated box cannot tell them apart.
 *
 *   - **The determinism test asserts that the compared thing is substantial and
 *     that a perturbation changes it** before comparing two runs byte for byte,
 *     and separately counts the vertices on the cut plane -- which is the one
 *     assertion that fails if each face computes its own copy of a shared cut
 *     point.
 *
 *   - **The solids are not all boxes, and that is the point.** A convex box cut
 *     anywhere gives a cap of one rectangle, so every rule that sorts cap rings
 *     out -- parity, containment, which outline owns which hole, which way a hole
 *     is wound -- is unreachable on one, and five separate breakages of that
 *     block once left this suite at 28 passed, 0 failed. So there is an L whose
 *     own wall lies in the cut plane, a hollow column, a column standing inside
 *     another column's bore, a box with every vertex duplicated, and half a box
 *     with no lid. Each one is here because it reaches something no box does.
 *
 *   - **Closedness is asserted by EDGE PAIRING on the non-convex shapes.**
 *     every_face_points_outward() compares a face normal against the direction
 *     from the vertex centroid, which is sound for a convex solid about a
 *     well-centred point and wrong for an L or a bore. edges_pair_up() is a
 *     statement about the surface instead, so it holds for all of them -- and it
 *     counts hole rings, so a cap whose bore was never attached fails it while
 *     still enclosing the right volume.
 *
 * Nothing here needs a GPU, a window or a file on disk, so nothing here skips.
 */

#include "framework.hpp"

#include "procgen/rules/ast.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/op_split.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

using stratum::procgen::rules::CallStmt;
using stratum::procgen::rules::Expr;
using stratum::procgen::rules::expr_kind;
using stratum::procgen::rules::ExprId;
using stratum::procgen::rules::ExprKind;
using stratum::procgen::rules::extrude_shape;
using stratum::procgen::rules::Face;
using stratum::procgen::rules::face_area;
using stratum::procgen::rules::face_normal;
using stratum::procgen::rules::geometry_area;
using stratum::procgen::rules::geometry_volume;
using stratum::MaterialId;
using stratum::MaterialKey;
using stratum::procgen::rules::kNoNode;
using stratum::procgen::rules::kNoPart;
using stratum::procgen::rules::NumberExpr;
using stratum::procgen::rules::OpResult;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::refit_scope;
using stratum::procgen::rules::rotate_scope_shape;
using stratum::procgen::rules::RuleFile;
using stratum::procgen::rules::ScopeAxis;
using stratum::procgen::rules::Shape;
using stratum::procgen::rules::shape_from_polygon;
using stratum::procgen::rules::shape_from_rect;
using stratum::procgen::rules::shape_from_rings;
using stratum::procgen::rules::ShapeGeometry;
using stratum::procgen::rules::SizeKind;
using stratum::procgen::rules::slice_shape;
using stratum::procgen::rules::solve_split;
using stratum::procgen::rules::SplitAxis;
using stratum::procgen::rules::split_body_for;
using stratum::procgen::rules::split_entry_for;
using stratum::procgen::rules::split_parts_from_statement;
using stratum::procgen::rules::split_shape;
using stratum::procgen::rules::SplitEntry;
using stratum::procgen::rules::SplitLayout;
using stratum::procgen::rules::SplitPart;
using stratum::procgen::rules::SplitPiece;
using stratum::procgen::rules::SplitStmt;
using stratum::procgen::rules::Stmt;
using stratum::procgen::rules::StmtId;
using stratum::procgen::rules::to_scope_axis;

namespace {

// ============================================================================
// Helpers
// ============================================================================

[[nodiscard]] SplitPart fixed_part(double metres) {
    SplitPart part;
    part.kind = SizeKind::Absolute;
    part.value = metres;
    return part;
}

[[nodiscard]] SplitPart percent_part(double percent) {
    SplitPart part;
    part.kind = SizeKind::Relative;
    part.value = percent;
    return part;
}

[[nodiscard]] SplitPart float_part(double nominal) {
    SplitPart part;
    part.kind = SizeKind::Floating;
    part.value = nominal;
    return part;
}

[[nodiscard]] SplitPart float_percent_part(double percent) {
    SplitPart part;
    part.kind = SizeKind::FloatingRelative;
    part.value = percent;
    return part;
}

[[nodiscard]] SplitPart repeat_part(std::vector<SplitPart> children) {
    SplitPart part;
    part.is_repeat = true;
    part.children = std::move(children);
    return part;
}

/// A 2 x 3 x 4 box, the same one test_interpreter.cpp builds: the rectangle 2 by
/// 4 in the xz plane, extruded 3 up y.
[[nodiscard]] Shape make_box() {
    Shape shape = shape_from_rect(2.0, 4.0);
    const OpResult result = extrude_shape(shape, ScopeAxis::Y, 3.0);
    CHECK_TRUE(result.ok);
    return shape;
}

void check_vec3(const glm::dvec3& actual, const glm::dvec3& expected, double eps) {
    CHECK_NEAR(actual.x, expected.x, eps);
    CHECK_NEAR(actual.y, expected.y, eps);
    CHECK_NEAR(actual.z, expected.z, eps);
}

/// The sum of every face's vector area. Zero for a closed surface with consistent
/// winding, and for nothing else -- so this is what says a slab was capped.
[[nodiscard]] glm::dvec3 total_vector_area(const ShapeGeometry& geometry) {
    glm::dvec3 total{0.0};
    for (const Face& face : geometry.faces) {
        total += face_normal(geometry, face) * face_area(geometry, face);
    }
    return total;
}

/// Does every face normal point away from the interior? Catches a cap wound the
/// wrong way round, which leaves the volume, the area and the scope all correct.
[[nodiscard]] bool every_face_points_outward(const ShapeGeometry& geometry) {
    if (geometry.faces.empty() || geometry.positions.empty()) {
        return false;
    }
    glm::dvec3 centroid{0.0};
    for (const glm::dvec3& position : geometry.positions) {
        centroid += position;
    }
    centroid /= static_cast<double>(geometry.positions.size());

    for (const Face& face : geometry.faces) {
        if (face.loop.empty()) {
            return false;
        }
        glm::dvec3 face_centre{0.0};
        for (const uint32_t index : face.loop) {
            face_centre += geometry.positions[index];
        }
        face_centre /= static_cast<double>(face.loop.size());
        if (glm::dot(face_normal(geometry, face), face_centre - centroid) <= 0.0) {
            return false;
        }
    }
    return true;
}

/**
 * @brief The world-space bounding box of a shape's vertices
 *
 * Through scope.to_world() rather than from origin and size, because a scope that
 * has drifted away from its geometry is exactly the fault this catches: a slab of
 * the right SIZE in the wrong PLACE.
 */
void world_bounds(const Shape& shape, glm::dvec3& min_out, glm::dvec3& max_out) {
    min_out = glm::dvec3{0.0};
    max_out = glm::dvec3{0.0};
    if (shape.geometry.positions.empty()) {
        return;
    }
    min_out = glm::dvec3{1.0e300};
    max_out = glm::dvec3{-1.0e300};
    for (const glm::dvec3& local : shape.geometry.positions) {
        const glm::dvec3 world = shape.scope.to_world(local);
        min_out = glm::min(min_out, world);
        max_out = glm::max(max_out, world);
    }
}

/// Does any note contain @p needle?
[[nodiscard]] bool a_note_says(const SplitLayout& layout, const std::string& needle) {
    for (const std::string& note : layout.notes) {
        if (note.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// How many vertices sit exactly on the plane `coord == value` along @p axis
[[nodiscard]] int vertices_on_plane(const ShapeGeometry& geometry, int axis, double value) {
    int count = 0;
    for (const glm::dvec3& position : geometry.positions) {
        if (position[axis] == value) {
            ++count;
        }
    }
    return count;
}

/// Are two geometries byte-identical? Positions with ==, not with a tolerance:
/// the promise is identical output and a tolerance hides the drift it is for.
[[nodiscard]] bool geometry_is_identical(const ShapeGeometry& a, const ShapeGeometry& b) {
    if (a.positions.empty() || a.positions != b.positions) {
        return false;
    }
    if (a.faces.size() != b.faces.size()) {
        return false;
    }
    for (size_t i = 0; i < a.faces.size(); ++i) {
        if (a.faces[i].loop != b.faces[i].loop || a.faces[i].holes != b.faces[i].holes) {
            return false;
        }
    }
    return true;
}

/// Parse @p source, asserting the parse as a rendered string so a typo in the
/// rule text prints the parser's own caret instead of "false != true".
[[nodiscard]] ParseResult must_parse(const std::string& source) {
    ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return parsed;
}

/// The first split statement in a parsed file, or nullptr
[[nodiscard]] const SplitStmt* first_split(const RuleFile& file) {
    for (const Stmt& stmt : file.stmts) {
        if (const SplitStmt* split = std::get_if<SplitStmt>(&stmt.node)) {
            return split;
        }
    }
    return nullptr;
}

/**
 * @brief Evaluate a split size that is a plain literal
 *
 * Enough for every rule file here, and deliberately NOT the interpreter's
 * evaluator: the point of taking a callback is that the solver can be driven
 * without standing up a generation.
 */
[[nodiscard]] double literal_size(const RuleFile& file, ExprId id) {
    if (id == kNoNode || id >= file.exprs.size()) {
        return 0.0;
    }
    const Expr& expr = file.expr(id);
    if (expr_kind(expr) != ExprKind::Number) {
        return 0.0;
    }
    return std::get<NumberExpr>(expr.node).value;
}

/// The name called by the single statement of a split entry's body, or ""
[[nodiscard]] std::string body_calls(const RuleFile& file, StmtId body) {
    if (body == kNoNode || body >= file.stmts.size()) {
        return std::string{};
    }
    const auto* block = std::get_if<stratum::procgen::rules::BlockStmt>(&file.stmt(body).node);
    if (block == nullptr || block->statements.size() != 1) {
        return std::string{};
    }
    const StmtId inner = block->statements[0];
    if (inner >= file.stmts.size()) {
        return std::string{};
    }
    const auto* call = std::get_if<CallStmt>(&file.stmt(inner).node);
    return call == nullptr ? std::string{} : call->callee.name;
}

/**
 * @brief Is every directed edge matched by exactly one edge the other way?
 *
 * The sound closedness test, and the one every_face_points_outward() above
 * cannot be: it is a statement about the SURFACE rather than about where a
 * centroid happens to fall, so it holds for a bore, for an L and for a box
 * alike. The non-convex tests below use this one; the convex box tests keep the
 * centroid test, which is correct there and reads more directly.
 *
 * Hole rings count, because a hole bounds the surface exactly as an outline
 * does. A cap whose bore was never attached encloses the right volume, sums to
 * the right vector area and fails here.
 *
 * Vertices are welded by EXACT position first. A face may carry its own copy of
 * a shared corner -- shape.cpp's extruder produces them, and so does any
 * imported mesh -- and comparing indices would call a perfectly closed solid
 * open.
 */
[[nodiscard]] bool edges_pair_up(const ShapeGeometry& geometry) {
    if (geometry.faces.empty() || geometry.positions.empty()) {
        return false;
    }
    std::map<std::tuple<double, double, double>, size_t> welded;
    std::vector<size_t> id(geometry.positions.size(), 0);
    for (size_t i = 0; i < geometry.positions.size(); ++i) {
        const glm::dvec3& position = geometry.positions[i];
        const auto key = std::make_tuple(position.x, position.y, position.z);
        const auto found = welded.find(key);
        if (found == welded.end()) {
            id[i] = welded.size();
            welded.emplace(key, id[i]);
        } else {
            id[i] = found->second;
        }
    }

    std::map<std::pair<size_t, size_t>, int> count;
    const auto add_ring = [&](const std::vector<uint32_t>& ring) {
        const size_t n = ring.size();
        for (size_t i = 0; i < n; ++i) {
            ++count[{id[ring[i]], id[ring[(i + 1) % n]]}];
        }
    };
    for (const Face& face : geometry.faces) {
        add_ring(face.loop);
        for (const std::vector<uint32_t>& hole : face.holes) {
            add_ring(hole);
        }
    }
    for (const std::pair<const std::pair<size_t, size_t>, int>& entry : count) {
        if (entry.second != 1) {
            return false;
        }
        const auto reverse = count.find({entry.first.second, entry.first.first});
        if (reverse == count.end() || reverse->second != 1) {
            return false;
        }
    }
    return true;
}

/// Closed, consistently wound, and facing OUT. A solid whose every face was
/// flipped pairs its edges just as well and encloses a negative volume, which is
/// what the second half catches.
[[nodiscard]] bool is_closed_solid(const ShapeGeometry& geometry) {
    return edges_pair_up(geometry) && geometry_volume(geometry) > 0.0;
}

/// Does any ring visit the same vertex twice running, the wrap included?
[[nodiscard]] bool a_ring_repeats_a_vertex(const ShapeGeometry& geometry) {
    const auto repeats = [](const std::vector<uint32_t>& ring) {
        const size_t n = ring.size();
        for (size_t i = 0; i < n; ++i) {
            if (ring[i] == ring[(i + 1) % n]) {
                return true;
            }
        }
        return false;
    };
    for (const Face& face : geometry.faces) {
        if (repeats(face.loop)) {
            return true;
        }
        for (const std::vector<uint32_t>& hole : face.holes) {
            if (repeats(hole)) {
                return true;
            }
        }
    }
    return false;
}

/// The faces lying entirely in the plane `coord == value` along @p axis, and
/// their total area
///
/// What the cut left at one end of a slab: a cap, a wall that was already there,
/// or -- when the cap closed around a hole instead of beside it -- one face
/// where there should be two.
void faces_in_plane(const ShapeGeometry& geometry,
                    int axis,
                    double value,
                    size_t& count_out,
                    double& area_out) {
    count_out = 0;
    area_out = 0.0;
    for (const Face& face : geometry.faces) {
        bool all_on_plane = !face.loop.empty();
        for (const uint32_t index : face.loop) {
            if (geometry.positions[index][axis] != value) {
                all_on_plane = false;
            }
        }
        if (all_on_plane) {
            ++count_out;
            area_out += face_area(geometry, face);
        }
    }
}

/// Total length of every piece.
///
/// NOT SplitLayout::used, which is the far end of the last piece: a repeat group
/// that covered half of its span still lands its last piece where the span ends,
/// so `used` reports the whole extent and the hole in the middle goes unseen.
/// This is the number that notices.
[[nodiscard]] double covered_length(const SplitLayout& layout) {
    double total = 0.0;
    for (const SplitPiece& piece : layout.pieces) {
        total += piece.length;
    }
    return total;
}

/// The L-shaped footprint {(0,0),(4,0),(4,2),(2,2),(2,4),(0,4)} extruded 1.
///
/// The simplest NON-CONVEX solid, and the simplest one with a wall of its own
/// lying in a plane a split cuts at: area 40, volume 12.
[[nodiscard]] Shape make_L_prism() {
    Shape shape = shape_from_polygon({{0.0, 0.0},
                                      {4.0, 0.0},
                                      {4.0, 2.0},
                                      {2.0, 2.0},
                                      {2.0, 4.0},
                                      {0.0, 4.0}});
    const OpResult result = extrude_shape(shape, ScopeAxis::Y, 1.0);
    CHECK_TRUE(result.ok);
    return shape;
}

/// A 4 x 4 column with a 2 x 2 bore, three high. A cut through it has an outline
/// AND a hole, which no box can stand in for.
[[nodiscard]] Shape make_hollow_column() {
    Shape shape = shape_from_rings({{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}},
                                   {{{1.0, 1.0}, {3.0, 1.0}, {3.0, 3.0}, {1.0, 3.0}}});
    const OpResult result = extrude_shape(shape, ScopeAxis::Y, 3.0);
    CHECK_TRUE(result.ok);
    return shape;
}

/// Copy a second geometry into the first, shifted by @p offset
///
/// For a solid with more rings than shape_from_rings() can express: a hollow
/// column standing inside the bore of another one, which is the only shape here
/// whose cut has a ring nested three deep.
void append_geometry(ShapeGeometry& into, const ShapeGeometry& from, const glm::dvec3& offset) {
    const uint32_t base = static_cast<uint32_t>(into.positions.size());
    for (const glm::dvec3& position : from.positions) {
        into.positions.push_back(position + offset);
    }
    for (const Face& face : from.faces) {
        Face copy = face;
        for (uint32_t& index : copy.loop) {
            index += base;
        }
        for (std::vector<uint32_t>& hole : copy.holes) {
            for (uint32_t& index : hole) {
                index += base;
            }
        }
        into.faces.push_back(std::move(copy));
    }
}

/// Give every face its own copy of every corner it uses
///
/// What an imported mesh, and shape.cpp's own add_ring(), hand over: coincident
/// vertices at different indices. The cut has to weld them by position or its
/// cap ring is a handful of open chains.
[[nodiscard]] Shape with_duplicated_vertices(const Shape& shape) {
    Shape copy = shape;
    ShapeGeometry split;
    for (const Face& face : shape.geometry.faces) {
        Face rebuilt;
        rebuilt.material = face.material;
        for (const uint32_t index : face.loop) {
            rebuilt.loop.push_back(static_cast<uint32_t>(split.positions.size()));
            split.positions.push_back(shape.geometry.positions[index]);
        }
        for (const std::vector<uint32_t>& hole : face.holes) {
            std::vector<uint32_t> ring;
            for (const uint32_t index : hole) {
                ring.push_back(static_cast<uint32_t>(split.positions.size()));
                split.positions.push_back(shape.geometry.positions[index]);
            }
            rebuilt.holes.push_back(std::move(ring));
        }
        split.faces.push_back(std::move(rebuilt));
    }
    copy.geometry = std::move(split);
    refit_scope(copy);
    return copy;
}

}  // namespace

// ============================================================================
// Absolute and relative sizes
// ============================================================================

TEST(OpSplit, absolute_parts_are_laid_out_in_order_at_the_metres_they_ask_for) {
    // 3, 4 and 2 of an extent of 10, so the boundaries are at 0, 3, 7 and 9 and
    // one metre is left over at the far end. Distinct sizes, so a reordering
    // shows up as a wrong number rather than as a right one in the wrong place.
    const SplitLayout layout =
        solve_split({fixed_part(3.0), fixed_part(4.0), fixed_part(2.0)}, 10.0);

    CHECK_EQ(layout.pieces.size(), size_t{3});
    CHECK_TRUE(layout.clean());
    CHECK_NEAR(layout.overflow, 0.0, 0.0);
    if (layout.pieces.size() != 3) {
        return;
    }
    CHECK_NEAR(layout.pieces[0].begin, 0.0, 1e-12);
    CHECK_NEAR(layout.pieces[0].length, 3.0, 1e-12);
    CHECK_NEAR(layout.pieces[1].begin, 3.0, 1e-12);
    CHECK_NEAR(layout.pieces[1].length, 4.0, 1e-12);
    CHECK_NEAR(layout.pieces[2].begin, 7.0, 1e-12);
    CHECK_NEAR(layout.pieces[2].length, 2.0, 1e-12);
    CHECK_NEAR(layout.used, 9.0, 1e-12);

    // Absolute means absolute: the same parts against a wider extent are the same
    // three lengths, and only the leftover grows. A solver that normalised its
    // parts to the extent passes the block above and fails here.
    const SplitLayout wider =
        solve_split({fixed_part(3.0), fixed_part(4.0), fixed_part(2.0)}, 20.0);
    CHECK_EQ(wider.pieces.size(), size_t{3});
    if (wider.pieces.size() == 3) {
        CHECK_NEAR(wider.pieces[1].length, 4.0, 1e-12);
        CHECK_NEAR(wider.used, 9.0, 1e-12);
    }
}

TEST(OpSplit, a_relative_size_is_a_percentage_of_the_extent_and_not_a_length) {
    // 25% of 8 is 2. The SAME literal as an absolute size is 25 metres, so a
    // solver that ignored the kind cannot pass both halves of this test.
    const SplitLayout relative =
        solve_split({percent_part(25.0), percent_part(50.0), percent_part(25.0)}, 8.0);
    CHECK_EQ(relative.pieces.size(), size_t{3});
    if (relative.pieces.size() == 3) {
        CHECK_NEAR(relative.pieces[0].length, 2.0, 1e-12);
        CHECK_NEAR(relative.pieces[1].length, 4.0, 1e-12);
        CHECK_NEAR(relative.pieces[2].length, 2.0, 1e-12);
        CHECK_NEAR(relative.pieces[2].begin, 6.0, 1e-12);
    }
    CHECK_NEAR(relative.used, 8.0, 1e-12);

    const SplitLayout absolute = solve_split({fixed_part(25.0)}, 8.0);
    CHECK_EQ(absolute.pieces.size(), size_t{1});
    if (!absolute.pieces.empty()) {
        // Clipped at the extent, because 25 metres do not fit in 8.
        CHECK_NEAR(absolute.pieces[0].length, 8.0, 1e-12);
    }
    CHECK_NEAR(absolute.overflow, 17.0, 1e-12);
}

// ============================================================================
// The floating size -- the one the feature exists for
// ============================================================================

TEST(OpSplit, one_floating_part_takes_everything_the_fixed_parts_left) {
    // 1 + float + 1 of an extent of 10: the float is 8, whatever its nominal
    // says. The nominal here is 3, so a solver that used the nominal as a length
    // would give 3 and leave a five-metre hole in the middle of the facade.
    const SplitLayout layout =
        solve_split({fixed_part(1.0), float_part(3.0), fixed_part(1.0)}, 10.0);

    CHECK_EQ(layout.pieces.size(), size_t{3});
    CHECK_TRUE(layout.clean());
    if (layout.pieces.size() != 3) {
        return;
    }
    CHECK_NEAR(layout.pieces[0].length, 1.0, 1e-12);
    CHECK_NEAR(layout.pieces[1].begin, 1.0, 1e-12);
    CHECK_NEAR(layout.pieces[1].length, 8.0, 1e-12);
    CHECK_NEAR(layout.pieces[2].begin, 9.0, 1e-12);
    CHECK_NEAR(layout.pieces[2].length, 1.0, 1e-12);
    CHECK_NEAR(layout.used, 10.0, 1e-12);

    // The piers keep their metres on a lot of any width, which is the whole
    // claim: at 30 the float is 28, not 3 and not 8.
    const SplitLayout wide =
        solve_split({fixed_part(1.0), float_part(3.0), fixed_part(1.0)}, 30.0);
    CHECK_EQ(wide.pieces.size(), size_t{3});
    if (wide.pieces.size() == 3) {
        CHECK_NEAR(wide.pieces[0].length, 1.0, 1e-12);
        CHECK_NEAR(wide.pieces[1].length, 28.0, 1e-12);
        CHECK_NEAR(wide.pieces[2].length, 1.0, 1e-12);
    }
}

TEST(OpSplit, several_floating_parts_share_the_remainder_in_proportion_to_their_weights) {
    // 2 fixed + float(1) + float(3) + 2 fixed, extent 12. The remainder is 8 and
    // the weights are 1 and 3, so the shares are 8/4 = 2 and 24/4 = 6 -- NOT four
    // and four, which is what an implementation that split the remainder evenly
    // between the floats would give, and NOT one and three, which is what one
    // that used the nominals as lengths would give.
    const SplitLayout layout = solve_split(
        {fixed_part(2.0), float_part(1.0), float_part(3.0), fixed_part(2.0)}, 12.0);

    CHECK_EQ(layout.pieces.size(), size_t{4});
    CHECK_NEAR(layout.elastic_weight, 4.0, 1e-12);
    if (layout.pieces.size() != 4) {
        return;
    }
    CHECK_NEAR(layout.pieces[1].begin, 2.0, 1e-12);
    CHECK_NEAR(layout.pieces[1].length, 2.0, 1e-12);
    CHECK_NEAR(layout.pieces[2].begin, 4.0, 1e-12);
    CHECK_NEAR(layout.pieces[2].length, 6.0, 1e-12);
    CHECK_NEAR(layout.pieces[3].begin, 10.0, 1e-12);
    CHECK_NEAR(layout.used, 12.0, 1e-12);
}

TEST(OpSplit, a_floating_relative_weight_is_a_percentage_and_mixes_with_a_plain_float) {
    // ~25% of 10 is a nominal of 2.5; ~1 is a nominal of 1. Nothing is fixed, so
    // the remainder is the whole 10 and the shares are 10 * 2.5 / 3.5 and
    // 10 * 1 / 3.5 -- computed here rather than pasted, from the two nominals the
    // author wrote, so the numbers cannot drift apart from the reasoning.
    const SplitLayout layout = solve_split({float_percent_part(25.0), float_part(1.0)}, 10.0);

    const double first = 10.0 * 2.5 / 3.5;
    const double second = 10.0 * 1.0 / 3.5;
    CHECK_EQ(layout.pieces.size(), size_t{2});
    CHECK_NEAR(layout.elastic_weight, 3.5, 1e-12);
    if (layout.pieces.size() == 2) {
        CHECK_NEAR(layout.pieces[0].length, first, 1e-12);
        CHECK_NEAR(layout.pieces[1].length, second, 1e-12);
        CHECK_NEAR(layout.pieces[1].begin, first, 1e-12);
    }
    CHECK_NEAR(layout.used, 10.0, 1e-12);

    // The four kinds are distinguishable from the SAME literal, exactly as the
    // parser suite checks the parser's half of it. Four 2s, four answers.
    const SplitLayout kinds =
        solve_split({fixed_part(2.0), percent_part(2.0), float_part(2.0), float_percent_part(2.0)},
                    100.0);
    CHECK_EQ(kinds.pieces.size(), size_t{4});
    if (kinds.pieces.size() == 4) {
        CHECK_NEAR(kinds.pieces[0].length, 2.0, 1e-12);   // 2 metres
        CHECK_NEAR(kinds.pieces[1].length, 2.0, 1e-12);   // 2% of 100
        // 96 left over, weights 2 and 2% of 100 = 2, so 48 each.
        CHECK_NEAR(kinds.pieces[2].length, 48.0, 1e-12);
        CHECK_NEAR(kinds.pieces[3].length, 48.0, 1e-12);
    }
}

// ============================================================================
// Repetition
// ============================================================================

TEST(OpSplit, the_facade_case_keeps_every_pier_fixed_and_every_bay_the_same) {
    // A fixed corner, a repeating band, a fixed corner -- the shape every facade
    // in the product will be. Extent 20, corners 1 each, so the band gets 18.
    // The period is 2.5 + 0.4 = 2.9 and floor(18 / 2.9) is 6, so six copies of
    // exactly 3.0. Inside a copy the pier keeps its 0.4 and the bay takes the
    // rest: 2.6. Not 2.5, which is the nominal, and not 3.0 minus nothing.
    const SplitLayout layout = solve_split(
        {fixed_part(1.0), repeat_part({float_part(2.5), fixed_part(0.4)}), fixed_part(1.0)}, 20.0);

    CHECK_TRUE(layout.clean());
    CHECK_EQ(layout.repeat_copies, uint32_t{6});
    CHECK_FALSE(layout.compressed);
    // Two corners plus six copies of two children.
    CHECK_EQ(layout.pieces.size(), size_t{14});
    if (layout.pieces.size() != 14) {
        return;
    }

    CHECK_NEAR(layout.pieces[0].begin, 0.0, 1e-12);
    CHECK_NEAR(layout.pieces[0].length, 1.0, 1e-12);
    CHECK_EQ(layout.pieces[0].part, size_t{0});
    CHECK_EQ(layout.pieces[0].child, kNoPart);

    for (uint32_t copy = 0; copy < 6; ++copy) {
        const SplitPiece& bay = layout.pieces[1 + 2 * copy];
        const SplitPiece& pier = layout.pieces[2 + 2 * copy];
        const double copy_begin = 1.0 + 3.0 * static_cast<double>(copy);

        // Every bay the same width as every other bay. Six bays of six widths
        // would satisfy a count and is the defect this rule exists to prevent.
        CHECK_NEAR(bay.begin, copy_begin, 1e-12);
        CHECK_NEAR(bay.length, 2.6, 1e-12);
        CHECK_EQ(bay.part, size_t{1});
        CHECK_EQ(bay.child, size_t{0});
        CHECK_EQ(bay.copy, copy);

        // The pier keeps its exact metres, which is the other half of the claim.
        CHECK_NEAR(pier.begin, copy_begin + 2.6, 1e-12);
        CHECK_NEAR(pier.length, 0.4, 1e-12);
        CHECK_EQ(pier.child, size_t{1});
    }

    CHECK_NEAR(layout.pieces[13].begin, 19.0, 1e-12);
    CHECK_NEAR(layout.pieces[13].length, 1.0, 1e-12);
    CHECK_NEAR(layout.used, 20.0, 1e-12);
}

TEST(OpSplit, the_leftover_of_a_repeat_is_distributed_into_the_copies_and_leaves_no_gap) {
    // Ten copies of a one-metre part in ten and a half metres. floor is 10, and
    // the half metre goes INTO the copies: ten bays of 1.05, not ten of 1.0 with
    // a 0.5 gap at the end and not eleven of 0.9545.
    const SplitLayout layout = solve_split({repeat_part({fixed_part(1.0)})}, 10.5);

    CHECK_EQ(layout.repeat_copies, uint32_t{10});
    CHECK_EQ(layout.pieces.size(), size_t{10});
    for (const SplitPiece& piece : layout.pieces) {
        CHECK_NEAR(piece.length, 1.05, 1e-12);
    }
    // No gap: the copies tile the extent exactly.
    CHECK_NEAR(layout.used, 10.5, 1e-12);
    if (layout.pieces.size() == 10) {
        CHECK_NEAR(layout.pieces[9].begin + layout.pieces[9].length, 10.5, 1e-12);
    }

    // A copy whose parts are ALL fixed is scaled as a whole, so the parts keep
    // their proportions: 1 and 2 become 10/9 and 20/9, still exactly 1 to 2.
    const SplitLayout proportional =
        solve_split({repeat_part({fixed_part(1.0), fixed_part(2.0)})}, 10.0);
    CHECK_EQ(proportional.repeat_copies, uint32_t{3});
    CHECK_EQ(proportional.pieces.size(), size_t{6});
    if (proportional.pieces.size() == 6) {
        const double small = proportional.pieces[0].length;
        CHECK_NEAR(small, 10.0 / 9.0, 1e-12);
        CHECK_NEAR(proportional.pieces[1].length, 2.0 * small, 1e-12);
        CHECK_NEAR(proportional.pieces[5].begin + proportional.pieces[5].length, 10.0, 1e-12);
    }
    CHECK_NEAR(proportional.used, 10.0, 1e-12);
}

TEST(OpSplit, a_repeat_takes_as_many_whole_copies_as_fit_and_never_zero) {
    // 10 / 3 is 3.33, so three copies and not four: "as many whole copies as fit".
    const SplitLayout three = solve_split({repeat_part({fixed_part(3.0)})}, 10.0);
    CHECK_EQ(three.repeat_copies, uint32_t{3});
    CHECK_FALSE(three.compressed);

    // A group wider than the space it was given is COMPRESSED to one copy rather
    // than dropped. A facade that silently loses every window because the lot
    // came out narrow is a bug report with nothing in it.
    const SplitLayout squeezed = solve_split({repeat_part({fixed_part(4.0)})}, 3.0);
    CHECK_EQ(squeezed.repeat_copies, uint32_t{1});
    CHECK_TRUE(squeezed.compressed);
    CHECK_TRUE(a_note_says(squeezed, "compressed copy"));
    CHECK_EQ(squeezed.pieces.size(), size_t{1});
    if (!squeezed.pieces.empty()) {
        // Squeezed to fit, not truncated: the part is 3 long, not 4 clipped to 3
        // with the rest of the group lost.
        CHECK_NEAR(squeezed.pieces[0].length, 3.0, 1e-12);
    }

    // And a compressed copy of several fixed parts keeps their proportions too,
    // so the last bay narrows instead of vanishing.
    const SplitLayout narrow =
        solve_split({repeat_part({fixed_part(1.0), fixed_part(3.0)})}, 2.0);
    CHECK_EQ(narrow.pieces.size(), size_t{2});
    if (narrow.pieces.size() == 2) {
        CHECK_NEAR(narrow.pieces[0].length, 0.5, 1e-12);
        CHECK_NEAR(narrow.pieces[1].length, 1.5, 1e-12);
    }
}

TEST(OpSplit, a_repeat_shares_the_remainder_with_the_floats_beside_it) {
    // A repeat group is elastic, and its weight is its period. Period 2, float
    // nominal 2, nothing fixed, extent 12: six each. The repeat's six become
    // three copies of 2.
    const SplitLayout layout =
        solve_split({repeat_part({fixed_part(2.0)}), float_part(2.0)}, 12.0);

    CHECK_NEAR(layout.elastic_weight, 4.0, 1e-12);
    CHECK_EQ(layout.repeat_copies, uint32_t{3});
    CHECK_EQ(layout.pieces.size(), size_t{4});
    if (layout.pieces.size() == 4) {
        CHECK_NEAR(layout.pieces[0].length, 2.0, 1e-12);
        CHECK_NEAR(layout.pieces[2].begin, 4.0, 1e-12);
        CHECK_NEAR(layout.pieces[3].begin, 6.0, 1e-12);
        CHECK_NEAR(layout.pieces[3].length, 6.0, 1e-12);
    }
    CHECK_NEAR(layout.used, 12.0, 1e-12);
}

TEST(OpSplit, a_repeat_copy_count_is_capped_rather_than_allowed_to_exhaust_memory) {
    // A period that came out near zero -- `repeat { ~width/100 : Bay(); }` with a
    // small width -- is floor(span / tiny) copies. 10 / 0.0001 is a hundred
    // thousand pieces, and without the cap that is an out-of-memory kill before
    // InterpreterLimits::max_shapes gets to refuse a single shape.
    const SplitLayout layout = solve_split({repeat_part({fixed_part(0.0001)})}, 10.0);
    CHECK_EQ(layout.pieces.size(), size_t{4096});
    CHECK_EQ(layout.repeat_copies, uint32_t{4096});
    CHECK_TRUE(a_note_says(layout, "capped"));
    // Capped, and still tiling: the copies are stretched to fill the extent.
    CHECK_NEAR(layout.used, 10.0, 1e-12);
}

TEST(OpSplit, a_relative_child_of_a_repeat_fills_its_copy_rather_than_a_fraction_of_it) {
    // `repeat { 50% : Bay(); }` over an extent of 4. The period is 50% of 4 = 2,
    // so the group tiles twice and every copy is 2 long. The copy holds one bay
    // and nothing else, so the bay is the whole copy: 2, at 0 and at 2.
    //
    // A bay of 1 -- "50% of a two-metre copy", read literally -- leaves half of
    // every copy empty. That is four lines of valid rule text losing half a
    // facade, with layout.clean() true, no overflow, and used reporting the whole
    // extent; covered_length() is what sees it.
    const SplitLayout half = solve_split({repeat_part({percent_part(50.0)})}, 4.0);
    CHECK_TRUE(half.clean());
    CHECK_EQ(half.repeat_copies, uint32_t{2});
    CHECK_EQ(half.pieces.size(), size_t{2});
    if (half.pieces.size() == 2) {
        CHECK_NEAR(half.pieces[0].begin, 0.0, 1e-12);
        CHECK_NEAR(half.pieces[0].length, 2.0, 1e-12);
        CHECK_NEAR(half.pieces[1].begin, 2.0, 1e-12);
        CHECK_NEAR(half.pieces[1].length, 2.0, 1e-12);
    }
    CHECK_NEAR(covered_length(half), 4.0, 1e-12);

    // 25% of 8 is a period of 2 as well, so four copies of 2 -- and four pieces
    // of 2, not of 0.5. The percentage decides HOW MANY copies there are; inside
    // a copy it is a ratio to the siblings, and a lone child's ratio is all of it.
    const SplitLayout quarter = solve_split({repeat_part({percent_part(25.0)})}, 8.0);
    CHECK_EQ(quarter.repeat_copies, uint32_t{4});
    CHECK_EQ(quarter.pieces.size(), size_t{4});
    for (const SplitPiece& piece : quarter.pieces) {
        CHECK_NEAR(piece.length, 2.0, 1e-12);
    }
    CHECK_NEAR(covered_length(quarter), 8.0, 1e-12);

    // Beside a fixed sibling the ratio is the one the PERIOD was built from: 50%
    // of the extent of 10 is 5 against the pier's 1, a period of 6, and floor of
    // 10/6 is one copy of 10. So the copy divides as 10 * 5/6 and 10 * 1/6.
    const SplitLayout mixed =
        solve_split({repeat_part({percent_part(50.0), fixed_part(1.0)})}, 10.0);
    CHECK_EQ(mixed.repeat_copies, uint32_t{1});
    CHECK_EQ(mixed.pieces.size(), size_t{2});
    if (mixed.pieces.size() == 2) {
        CHECK_NEAR(mixed.pieces[0].length, 10.0 * 5.0 / 6.0, 1e-12);
        CHECK_NEAR(mixed.pieces[1].begin, 10.0 * 5.0 / 6.0, 1e-12);
        CHECK_NEAR(mixed.pieces[1].length, 10.0 * 1.0 / 6.0, 1e-12);
        CHECK_EQ(mixed.pieces[1].child, size_t{1});
    }
    CHECK_NEAR(covered_length(mixed), 10.0, 1e-12);

    // Beside something ELASTIC it is read the other way round -- a percentage of
    // the copy -- because the float can absorb whatever is left. The period is
    // 25% of 10 plus a nominal 1, so 3.5; floor(10 / 3.5) is 2 copies of 5; and
    // in a copy of 5 the bay is 25% of 5 = 1.25 with 3.75 of glass beside it.
    const SplitLayout elastic =
        solve_split({repeat_part({percent_part(25.0), float_part(1.0)})}, 10.0);
    CHECK_EQ(elastic.repeat_copies, uint32_t{2});
    CHECK_EQ(elastic.pieces.size(), size_t{4});
    if (elastic.pieces.size() == 4) {
        CHECK_NEAR(elastic.pieces[0].length, 1.25, 1e-12);
        CHECK_NEAR(elastic.pieces[1].length, 3.75, 1e-12);
        CHECK_NEAR(elastic.pieces[2].begin, 5.0, 1e-12);
        CHECK_NEAR(elastic.pieces[2].length, 1.25, 1e-12);
        CHECK_NEAR(elastic.pieces[3].length, 3.75, 1e-12);
    }
    CHECK_NEAR(covered_length(elastic), 10.0, 1e-12);

    // The same shape of rule through the whole geometric path: a 2 x 3 x 4 box
    // split along x by a lone relative child of a repeat. Two slabs of 1 x 3 x 4,
    // and the volumes still add up to the box -- which is the half of this the
    // solver numbers above cannot show, because a piece the solver made too short
    // still cuts a perfectly valid slab out of the middle of nowhere.
    const Shape box = make_box();
    const std::vector<Shape> slabs = split_shape(box, ScopeAxis::X, half);
    CHECK_EQ(slabs.size(), size_t{2});
    double total = 0.0;
    for (const Shape& slab : slabs) {
        total += geometry_volume(slab.geometry);
    }
    CHECK_NEAR(total, 24.0, 1e-12);
}

TEST(OpSplit, a_repeat_does_not_invent_an_overflow_the_author_never_wrote) {
    // `repeat { 200% : Bay(); }` over an extent of 1. The period is twice the
    // extent, so the group does not fit even once, holds one compressed copy of
    // 1, and says THAT -- and only that.
    //
    // Laying the copy out by re-solving `200%` against the period resolves it
    // against 2 and asks for 4, which used to come back as "the fixed parts of
    // this split ask for 4 of an extent of 2": an overflow of a number the author
    // never typed, against an extent that is not the one they split.
    const SplitLayout layout = solve_split({repeat_part({percent_part(200.0)})}, 1.0);
    CHECK_TRUE(layout.compressed);
    CHECK_TRUE(a_note_says(layout, "compressed copy"));
    CHECK_FALSE(a_note_says(layout, "cut off at the far end"));
    CHECK_EQ(layout.notes.size(), size_t{1});
    CHECK_NEAR(layout.overflow, 0.0, 0.0);
    CHECK_EQ(layout.pieces.size(), size_t{1});
    if (!layout.pieces.empty()) {
        CHECK_NEAR(layout.pieces[0].length, 1.0, 1e-12);
    }
}

TEST(OpSplit, a_note_is_said_once_however_many_parts_say_it) {
    // Two dead repeat groups in one split. The interpreter reports a note at the
    // SPLIT's location, so a second copy of the sentence is indistinguishable
    // from the first and buries whatever else the solve had to say.
    const SplitLayout layout =
        solve_split({repeat_part({fixed_part(0.0)}), repeat_part({fixed_part(0.0)})}, 10.0);

    CHECK_TRUE(a_note_says(layout, "cannot tile"));
    CHECK_EQ(layout.notes.size(), size_t{1});
    CHECK_TRUE(layout.pieces.empty());

    // And it is a DEDUPE, not a limit of one: a second, different fault is still
    // reported beside the first.
    const SplitLayout both = solve_split(
        {repeat_part({fixed_part(0.0)}), repeat_part({fixed_part(0.0)}), fixed_part(-1.0)}, 10.0);
    CHECK_TRUE(a_note_says(both, "cannot tile"));
    CHECK_TRUE(a_note_says(both, "negative size"));
    CHECK_EQ(both.notes.size(), size_t{2});
}

TEST(OpSplit, a_repeat_with_no_room_left_produces_no_copy_rather_than_an_empty_one) {
    // Twelve metres of fixed part in an extent of ten leaves the repeat group a
    // span of zero. A zero-length copy would put an empty bay in the shape tree
    // for a group the split had no space for, and would report "wider than the
    // space it was given" beside the overflow note that already explains it.
    const SplitLayout layout =
        solve_split({fixed_part(12.0), repeat_part({fixed_part(2.0)})}, 10.0);

    CHECK_EQ(layout.repeat_copies, uint32_t{0});
    CHECK_FALSE(layout.compressed);
    CHECK_EQ(layout.pieces.size(), size_t{1});
    CHECK_TRUE(a_note_says(layout, "cut off at the far end"));
    CHECK_FALSE(a_note_says(layout, "compressed copy"));
    CHECK_EQ(layout.notes.size(), size_t{1});
    if (!layout.pieces.empty()) {
        CHECK_EQ(layout.pieces[0].part, size_t{0});
        CHECK_NEAR(layout.pieces[0].length, 10.0, 1e-12);
    }
}

// ============================================================================
// The splits a person writes by accident
// ============================================================================

TEST(OpSplit, a_split_with_no_parts_produces_nothing_and_says_so) {
    const SplitLayout layout = solve_split({}, 10.0);
    CHECK_TRUE(layout.pieces.empty());
    CHECK_FALSE(layout.clean());
    CHECK_TRUE(a_note_says(layout, "no parts"));
    CHECK_NEAR(layout.used, 0.0, 0.0);
}

TEST(OpSplit, absolutes_that_exceed_the_extent_are_cut_off_at_the_far_end_and_said_so) {
    // Three four-metre floors in ten metres. The first two fit, the third is
    // truncated to two, and the overflow is two. Nothing is scaled: a storey
    // height that quietly is not the storey height is worse than a visibly
    // truncated top floor, so 4 stays 4.
    const SplitLayout layout =
        solve_split({fixed_part(4.0), fixed_part(4.0), fixed_part(4.0)}, 10.0);

    CHECK_EQ(layout.pieces.size(), size_t{3});
    CHECK_NEAR(layout.overflow, 2.0, 1e-12);
    CHECK_TRUE(a_note_says(layout, "cut off at the far end"));
    if (layout.pieces.size() == 3) {
        CHECK_NEAR(layout.pieces[0].length, 4.0, 1e-12);
        CHECK_NEAR(layout.pieces[1].length, 4.0, 1e-12);
        CHECK_NEAR(layout.pieces[2].begin, 8.0, 1e-12);
        CHECK_NEAR(layout.pieces[2].length, 2.0, 1e-12);
    }
    CHECK_NEAR(layout.used, 10.0, 1e-12);

    // A part entirely past the end is zero-length rather than negative-length or
    // missing, so the part indices still line up with what the author wrote.
    const SplitLayout four = solve_split(
        {fixed_part(4.0), fixed_part(4.0), fixed_part(4.0), fixed_part(4.0)}, 10.0);
    CHECK_EQ(four.pieces.size(), size_t{4});
    if (four.pieces.size() == 4) {
        CHECK_NEAR(four.pieces[3].length, 0.0, 0.0);
        CHECK_NEAR(four.pieces[3].begin, 10.0, 1e-12);
        CHECK_EQ(four.pieces[3].part, size_t{3});
    }
    CHECK_NEAR(four.overflow, 6.0, 1e-12);

    // A float beside an overflow gets nothing rather than a negative share.
    const SplitLayout starved = solve_split({fixed_part(12.0), float_part(1.0)}, 10.0);
    CHECK_EQ(starved.pieces.size(), size_t{2});
    if (starved.pieces.size() == 2) {
        CHECK_NEAR(starved.pieces[0].length, 10.0, 1e-12);
        CHECK_NEAR(starved.pieces[1].length, 0.0, 0.0);
    }
}

TEST(OpSplit, a_zero_width_part_is_kept_so_the_parts_after_it_keep_their_indices) {
    const SplitLayout layout =
        solve_split({fixed_part(0.0), fixed_part(3.0), float_part(1.0)}, 10.0);

    CHECK_EQ(layout.pieces.size(), size_t{3});
    if (layout.pieces.size() != 3) {
        return;
    }
    CHECK_NEAR(layout.pieces[0].length, 0.0, 0.0);
    CHECK_NEAR(layout.pieces[0].begin, 0.0, 0.0);
    CHECK_EQ(layout.pieces[0].part, size_t{0});
    // The 3 did not slide up into the zero part's place.
    CHECK_NEAR(layout.pieces[1].begin, 0.0, 0.0);
    CHECK_NEAR(layout.pieces[1].length, 3.0, 1e-12);
    CHECK_EQ(layout.pieces[1].part, size_t{1});
    CHECK_NEAR(layout.pieces[2].length, 7.0, 1e-12);
    CHECK_EQ(layout.pieces[2].part, size_t{2});
}

TEST(OpSplit, a_split_of_an_axis_with_no_extent_produces_empty_parts_rather_than_nothing) {
    // `split(y)` of a footprint. Every piece is empty, but the pieces EXIST, so
    // the caller reports one diagnostic; producing no pieces at all would make
    // the mistake look exactly like a rule that was never called.
    const SplitLayout layout = solve_split({fixed_part(3.0), float_part(1.0)}, 0.0);

    CHECK_EQ(layout.pieces.size(), size_t{2});
    CHECK_TRUE(a_note_says(layout, "no extent"));
    for (const SplitPiece& piece : layout.pieces) {
        CHECK_NEAR(piece.length, 0.0, 0.0);
        CHECK_NEAR(piece.begin, 0.0, 0.0);
    }
    CHECK_NEAR(layout.used, 0.0, 0.0);

    // A negative extent is the same case and not a mirror-image layout.
    const SplitLayout negative = solve_split({fixed_part(3.0)}, -5.0);
    CHECK_EQ(negative.pieces.size(), size_t{1});
    CHECK_NEAR(negative.extent, 0.0, 0.0);
    if (!negative.pieces.empty()) {
        CHECK_NEAR(negative.pieces[0].length, 0.0, 0.0);
    }
}

TEST(OpSplit, a_negative_size_counts_as_zero_and_says_so) {
    const SplitLayout layout = solve_split({fixed_part(-3.0), float_part(1.0)}, 10.0);

    CHECK_EQ(layout.pieces.size(), size_t{2});
    CHECK_TRUE(a_note_says(layout, "negative size"));
    if (layout.pieces.size() == 2) {
        CHECK_NEAR(layout.pieces[0].length, 0.0, 0.0);
        // The float still gets the whole extent: a negative part did not steal
        // three metres of remainder by counting as -3.
        CHECK_NEAR(layout.pieces[1].length, 10.0, 1e-12);
    }
}

TEST(OpSplit, a_repeat_group_with_no_size_produces_nothing_and_says_so) {
    const SplitLayout layout =
        solve_split({repeat_part({fixed_part(0.0)}), float_part(1.0)}, 10.0);

    CHECK_TRUE(a_note_says(layout, "cannot tile"));
    CHECK_EQ(layout.repeat_copies, uint32_t{0});
    // The float beside it still gets its share, so one bad group does not cost
    // the rest of the split.
    CHECK_EQ(layout.pieces.size(), size_t{1});
    if (!layout.pieces.empty()) {
        CHECK_NEAR(layout.pieces[0].length, 10.0, 1e-12);
    }

    const SplitLayout empty_group = solve_split({repeat_part({})}, 10.0);
    CHECK_TRUE(empty_group.pieces.empty());
    CHECK_FALSE(empty_group.clean());
}

// ============================================================================
// The cut
// ============================================================================

TEST(OpSplit, splitting_a_box_gives_closed_solids_whose_volumes_sum_to_the_original) {
    // The 2 x 3 x 4 box, split along x into 0.5 and the rest. Volumes are
    // 0.5 * 3 * 4 = 6 and 1.5 * 3 * 4 = 18, which sum to the original 24. Volume
    // is integrated over the faces, so a cut that clipped the walls and forgot to
    // cap the ends leaves both scopes correct and fails here.
    const Shape box = make_box();
    CHECK_NEAR(geometry_volume(box.geometry), 24.0, 1e-12);

    const SplitLayout layout = solve_split({fixed_part(0.5), float_part(1.0)}, 2.0);
    const std::vector<Shape> slabs = split_shape(box, ScopeAxis::X, layout);

    CHECK_EQ(slabs.size(), size_t{2});
    if (slabs.size() != 2) {
        return;
    }

    CHECK_NEAR(geometry_volume(slabs[0].geometry), 6.0, 1e-12);
    CHECK_NEAR(geometry_volume(slabs[1].geometry), 18.0, 1e-12);
    CHECK_NEAR(geometry_volume(slabs[0].geometry) + geometry_volume(slabs[1].geometry), 24.0,
               1e-12);

    for (const Shape& slab : slabs) {
        // Six faces, not "at least six": a cut that capped an end twice encloses
        // the right volume and has seven.
        CHECK_EQ(slab.geometry.faces.size(), size_t{6});
        // Closed and consistently wound, which the volume alone does not prove.
        check_vec3(total_vector_area(slab.geometry), glm::dvec3{0.0}, 1e-12);
        CHECK_TRUE(every_face_points_outward(slab.geometry));
    }

    // And the scopes are the two sub-boxes, in the right places.
    check_vec3(slabs[0].scope.origin, glm::dvec3{0.0, 0.0, 0.0}, 1e-12);
    check_vec3(slabs[0].scope.size, glm::dvec3{0.5, 3.0, 4.0}, 1e-12);
    check_vec3(slabs[1].scope.origin, glm::dvec3{0.5, 0.0, 0.0}, 1e-12);
    check_vec3(slabs[1].scope.size, glm::dvec3{1.5, 3.0, 4.0}, 1e-12);
}

TEST(OpSplit, the_ends_of_a_split_are_not_capped_a_second_time) {
    // The near end of the first piece is the box's own face, already there. A cut
    // that capped every plane regardless would emit a second copy of it: the
    // volume would still be right and every scope would still be right, and only
    // the surface area moves. 1 x 3 x 4 has an area of 2 * (3 + 4 + 12) = 38; a
    // doubled end face makes it 50.
    const Shape box = make_box();
    const Shape slab = slice_shape(box, ScopeAxis::X, 0.0, 1.0);

    CHECK_EQ(slab.geometry.faces.size(), size_t{6});
    CHECK_NEAR(geometry_area(slab.geometry), 38.0, 1e-12);
    CHECK_NEAR(geometry_volume(slab.geometry), 12.0, 1e-12);
    check_vec3(total_vector_area(slab.geometry), glm::dvec3{0.0}, 1e-12);

    // A slab cut out of the middle is capped at BOTH ends, and its area says so:
    // 1 x 3 x 4 again, and the same 38.
    const Shape middle = slice_shape(box, ScopeAxis::X, 0.5, 1.5);
    CHECK_EQ(middle.geometry.faces.size(), size_t{6});
    CHECK_NEAR(geometry_area(middle.geometry), 38.0, 1e-12);
    CHECK_NEAR(geometry_volume(middle.geometry), 12.0, 1e-12);
    CHECK_TRUE(every_face_points_outward(middle.geometry));
    check_vec3(middle.scope.origin, glm::dvec3{0.5, 0.0, 0.0}, 1e-12);
    check_vec3(middle.scope.size, glm::dvec3{1.0, 3.0, 4.0}, 1e-12);
}

TEST(OpSplit, a_cut_point_is_computed_once_and_shared_by_every_face_that_needs_it) {
    // Four edges of the box cross the plane x = 1, and each is shared by three
    // faces. Computing the intersection per face gives twelve vertices where
    // there should be four, and the cap ring then has cracks in it that no
    // tolerance can close without also welding a thin mullion shut.
    const Shape box = make_box();
    const Shape slab = slice_shape(box, ScopeAxis::X, 0.0, 1.0);

    CHECK_EQ(vertices_on_plane(slab.geometry, 0, 1.0), 4);
    // Eight in total: the four original corners at x = 0 and the four cut points.
    CHECK_EQ(slab.geometry.positions.size(), size_t{8});
}

TEST(OpSplit, splitting_a_facade_quad_partitions_its_area) {
    // A flat 4 by 3 footprint, area 12, split into 1, a float and 1 -- so 1, 2, 1
    // -- giving areas of 3, 6 and 3. A flat shape has no cut to cap, so this is
    // the half of the clipper that must NOT invent a face.
    const Shape quad = shape_from_rect(4.0, 3.0);
    CHECK_NEAR(geometry_area(quad.geometry), 12.0, 1e-12);

    const SplitLayout layout =
        solve_split({fixed_part(1.0), float_part(1.0), fixed_part(1.0)}, 4.0);
    const std::vector<Shape> pieces = split_shape(quad, ScopeAxis::X, layout);

    CHECK_EQ(pieces.size(), size_t{3});
    if (pieces.size() != 3) {
        return;
    }
    double total = 0.0;
    for (const Shape& piece : pieces) {
        CHECK_EQ(piece.geometry.faces.size(), size_t{1});
        total += geometry_area(piece.geometry);
        // Still facing up: a clip that reversed a ring turns the facade inside out.
        check_vec3(face_normal(piece.geometry, piece.geometry.faces[0]),
                   glm::dvec3{0.0, 1.0, 0.0}, 1e-12);
    }
    CHECK_NEAR(geometry_area(pieces[0].geometry), 3.0, 1e-12);
    CHECK_NEAR(geometry_area(pieces[1].geometry), 6.0, 1e-12);
    CHECK_NEAR(geometry_area(pieces[2].geometry), 3.0, 1e-12);
    CHECK_NEAR(total, 12.0, 1e-12);

    glm::dvec3 min{0.0};
    glm::dvec3 max{0.0};
    world_bounds(pieces[1], min, max);
    check_vec3(min, glm::dvec3{1.0, 0.0, 0.0}, 1e-12);
    check_vec3(max, glm::dvec3{3.0, 0.0, 3.0}, 1e-12);
}

TEST(OpSplit, a_hole_that_straddles_the_cut_still_subtracts_its_area) {
    // A 4 by 4 outline with a 2 by 2 hole in the middle: area 12. Cut down the
    // middle, each half keeps 8 of outline and 2 of hole, so 6 and 6. Dropping
    // the straddling hole instead would fill the notch in solid and give 8 and 8,
    // which is the one answer that is visibly wrong in the viewport.
    const std::vector<glm::dvec2> outer = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}};
    const std::vector<std::vector<glm::dvec2>> holes = {
        {{1.0, 1.0}, {3.0, 1.0}, {3.0, 3.0}, {1.0, 3.0}}};
    const Shape annulus = shape_from_rings(outer, holes);
    CHECK_NEAR(geometry_area(annulus.geometry), 12.0, 1e-12);

    const Shape left = slice_shape(annulus, ScopeAxis::X, 0.0, 2.0);
    const Shape right = slice_shape(annulus, ScopeAxis::X, 2.0, 4.0);

    CHECK_NEAR(geometry_area(left.geometry), 6.0, 1e-12);
    CHECK_NEAR(geometry_area(right.geometry), 6.0, 1e-12);
    CHECK_EQ(left.geometry.faces.size(), size_t{1});
    if (!left.geometry.faces.empty()) {
        CHECK_EQ(left.geometry.faces[0].holes.size(), size_t{1});
    }

    // A hole wholly inside a slab comes through untouched, which is the control:
    // the whole shape sliced to itself still has its hole and its area.
    const Shape whole = slice_shape(annulus, ScopeAxis::X, 0.0, 4.0);
    CHECK_NEAR(geometry_area(whole.geometry), 12.0, 1e-12);
}

TEST(OpSplit, the_cut_follows_the_scope_axis_and_not_the_world_axis) {
    // The scope is turned a quarter turn about y, so the scope's x axis runs
    // along the world's NEGATIVE z. A cut along scope x must therefore partition
    // the box's world z extent and leave its world x extent whole. A cut made in
    // world space partitions x instead, and every volume below still adds up.
    Shape box = make_box();
    rotate_scope_shape(box, glm::dvec3{0.0, 90.0, 0.0});
    check_vec3(box.scope.axes[0], glm::dvec3{0.0, 0.0, -1.0}, 0.0);
    check_vec3(box.scope.size, glm::dvec3{4.0, 3.0, 2.0}, 0.0);

    const SplitLayout layout = solve_split({fixed_part(1.0), float_part(1.0)}, 4.0);
    const std::vector<Shape> slabs = split_shape(box, ScopeAxis::X, layout);
    CHECK_EQ(slabs.size(), size_t{2});
    if (slabs.size() != 2) {
        return;
    }

    glm::dvec3 near_min{0.0};
    glm::dvec3 near_max{0.0};
    glm::dvec3 far_min{0.0};
    glm::dvec3 far_max{0.0};
    world_bounds(slabs[0], near_min, near_max);
    world_bounds(slabs[1], far_min, far_max);

    // Scope x runs along world -z, so the first metre of the scope is the LAST
    // metre of world z. Both slabs keep the full world x and y.
    check_vec3(near_min, glm::dvec3{0.0, 0.0, 3.0}, 1e-12);
    check_vec3(near_max, glm::dvec3{2.0, 3.0, 4.0}, 1e-12);
    check_vec3(far_min, glm::dvec3{0.0, 0.0, 0.0}, 1e-12);
    check_vec3(far_max, glm::dvec3{2.0, 3.0, 3.0}, 1e-12);

    CHECK_NEAR(geometry_volume(slabs[0].geometry), 6.0, 1e-12);
    CHECK_NEAR(geometry_volume(slabs[1].geometry), 18.0, 1e-12);
    for (const Shape& slab : slabs) {
        check_vec3(total_vector_area(slab.geometry), glm::dvec3{0.0}, 1e-12);
    }
}

TEST(OpSplit, a_zero_length_piece_is_an_empty_shape_with_the_slab_for_a_scope) {
    // `0 : Wall();` and every part of a split along a flat axis land here. The
    // shape exists, carries the parent's attributes, has no geometry and has a
    // scope positioned where the piece is -- which is what lets D4 insert an
    // asset into a gap a split left.
    Shape box = make_box();
    box.attributes["style"] = stratum::procgen::rules::Value::text("brick");

    const Shape empty = slice_shape(box, ScopeAxis::X, 1.25, 1.25);
    CHECK_TRUE(empty.geometry.faces.empty());
    CHECK_TRUE(empty.geometry.positions.empty());
    check_vec3(empty.scope.origin, glm::dvec3{1.25, 0.0, 0.0}, 1e-12);
    CHECK_NEAR(empty.scope.size.x, 0.0, 0.0);
    CHECK_NEAR(empty.scope.size.y, 3.0, 1e-12);
    CHECK_EQ(empty.attributes.size(), size_t{1});

    // A whole split along an axis with no extent: pieces, all empty, no crash.
    const Shape footprint = shape_from_rect(4.0, 3.0);
    const SplitLayout flat = solve_split({fixed_part(3.0), float_part(1.0)}, 0.0);
    const std::vector<Shape> slabs = split_shape(footprint, ScopeAxis::Y, flat);
    CHECK_EQ(slabs.size(), size_t{2});
    for (const Shape& slab : slabs) {
        CHECK_TRUE(slab.geometry.faces.empty());
        CHECK_NEAR(slab.scope.size.y, 0.0, 0.0);
    }
}

TEST(OpSplit, splitting_a_split_is_the_facade_pattern_and_conserves_the_solid) {
    // A facade into floors, then the ground floor into bays: the two-level
    // decomposition the language exists for. The volumes of every leaf must still
    // add up to the box they came from, which no amount of losing a slab satisfies.
    const Shape box = make_box();

    const SplitLayout floors = solve_split({fixed_part(1.0), float_part(1.0)}, 3.0);
    const std::vector<Shape> storeys = split_shape(box, ScopeAxis::Y, floors);
    CHECK_EQ(storeys.size(), size_t{2});
    if (storeys.size() != 2) {
        return;
    }
    CHECK_NEAR(geometry_volume(storeys[0].geometry), 8.0, 1e-12);
    CHECK_NEAR(geometry_volume(storeys[1].geometry), 16.0, 1e-12);

    const SplitLayout bays =
        solve_split({repeat_part({fixed_part(0.5)})}, storeys[0].scope.extent(ScopeAxis::X));
    CHECK_EQ(bays.repeat_copies, uint32_t{4});
    const std::vector<Shape> pieces = split_shape(storeys[0], ScopeAxis::X, bays);

    CHECK_EQ(pieces.size(), size_t{4});
    double total = 0.0;
    for (const Shape& piece : pieces) {
        CHECK_EQ(piece.geometry.faces.size(), size_t{6});
        check_vec3(total_vector_area(piece.geometry), glm::dvec3{0.0}, 1e-12);
        total += geometry_volume(piece.geometry);
    }
    CHECK_NEAR(total, 8.0, 1e-12);
    if (pieces.size() == 4) {
        check_vec3(pieces[3].scope.origin, glm::dvec3{1.5, 0.0, 0.0}, 1e-12);
        check_vec3(pieces[3].scope.size, glm::dvec3{0.5, 1.0, 4.0}, 1e-12);
    }
}

TEST(OpSplit, the_same_split_of_the_same_shape_is_byte_identical_and_notices_a_change) {
    const Shape box = make_box();
    const SplitLayout layout =
        solve_split({fixed_part(0.4), repeat_part({float_part(0.5), fixed_part(0.1)})}, 2.0);

    const std::vector<Shape> first = split_shape(box, ScopeAxis::X, layout);
    const std::vector<Shape> second = split_shape(box, ScopeAxis::X, layout);

    // Substantial before identical: two empty results compare equal, and a
    // determinism test that passes for an implementation producing nothing is
    // the defect this project finds most often.
    CHECK_EQ(first.size(), second.size());
    CHECK((first.size()) >= size_t{5});
    size_t vertices = 0;
    for (const Shape& slab : first) {
        vertices += slab.geometry.positions.size();
    }
    CHECK((vertices) >= size_t{40});

    bool identical = first.size() == second.size();
    for (size_t i = 0; i < first.size() && i < second.size(); ++i) {
        if (!geometry_is_identical(first[i].geometry, second[i].geometry)) {
            identical = false;
        }
        if (first[i].scope.origin != second[i].scope.origin ||
            first[i].scope.size != second[i].scope.size) {
            identical = false;
        }
    }
    CHECK_TRUE(identical);

    // And the comparison is sensitive: a hair's width more extent moves it.
    const SplitLayout nudged =
        solve_split({fixed_part(0.4000001), repeat_part({float_part(0.5), fixed_part(0.1)})}, 2.0);
    const std::vector<Shape> different = split_shape(box, ScopeAxis::X, nudged);
    bool any_difference = different.size() != first.size();
    for (size_t i = 0; i < first.size() && i < different.size(); ++i) {
        if (!geometry_is_identical(first[i].geometry, different[i].geometry)) {
            any_difference = true;
        }
    }
    CHECK_TRUE(any_difference);
}

TEST(OpSplit, a_cut_along_a_wall_the_shape_already_has_replaces_it_instead_of_doubling_it) {
    // The L: a 4 x 4 footprint with the quadrant x > 2, z > 2 taken out, extruded
    // 1. Split it at x = 2 and the plane slices the base while CONTAINING the
    // wall of the upper arm -- which is what any non-convex massing does, and
    // what a box cannot be made to do.
    //
    // A point on the plane is inside both half-spaces, so that wall survived into
    // BOTH slabs: the near one had the cap laid over the top of it and the far
    // one was left holding a face of a solid that is not in it. An L of volume 12
    // came out as slabs of 9.33 and 4, neither of them closed, with nothing said.
    const Shape shape = make_L_prism();
    CHECK_NEAR(geometry_volume(shape.geometry), 12.0, 1e-12);
    CHECK_NEAR(geometry_area(shape.geometry), 40.0, 1e-12);
    CHECK_TRUE(is_closed_solid(shape.geometry));

    const SplitLayout layout = solve_split({fixed_part(2.0), float_part(1.0)}, 4.0);
    const std::vector<Shape> slabs = split_shape(shape, ScopeAxis::X, layout);
    CHECK_EQ(slabs.size(), size_t{2});
    if (slabs.size() != 2) {
        return;
    }

    // The near slab is the whole 2 by 4 half of the L: volume 8, and a surface of
    // 8 + 8 for the two faces plus 12 of perimeter one metre high -- 28. The
    // wall that was in the plane is gone and the cap stands in its place, so
    // there are six faces and not seven, and 28 of area and not 30.
    CHECK_EQ(slabs[0].geometry.faces.size(), size_t{6});
    CHECK_NEAR(geometry_volume(slabs[0].geometry), 8.0, 1e-12);
    CHECK_NEAR(geometry_area(slabs[0].geometry), 28.0, 1e-12);
    check_vec3(total_vector_area(slabs[0].geometry), glm::dvec3{0.0}, 1e-12);
    CHECK_TRUE(is_closed_solid(slabs[0].geometry));

    // The far slab is the 2 by 2 square: volume 4, surface 4 + 4 + 8 = 16. The
    // same wall is dropped from THIS slab for the opposite reason -- the solid
    // beyond x = 2 does not reach it -- and here nothing replaces it.
    CHECK_EQ(slabs[1].geometry.faces.size(), size_t{6});
    CHECK_NEAR(geometry_volume(slabs[1].geometry), 4.0, 1e-12);
    CHECK_NEAR(geometry_area(slabs[1].geometry), 16.0, 1e-12);
    check_vec3(total_vector_area(slabs[1].geometry), glm::dvec3{0.0}, 1e-12);
    CHECK_TRUE(is_closed_solid(slabs[1].geometry));

    CHECK_NEAR(geometry_volume(slabs[0].geometry) + geometry_volume(slabs[1].geometry), 12.0,
               1e-12);

    // The far slab is 2 by 2 and SAYS 2 by 2. Clipping a non-convex ring that
    // touches the plane leaves a zero-area tail running out to z = 4: it encloses
    // nothing, so every number above is still right, and the scope -- which is
    // derived from the positions -- reports a slab twice as deep as it is.
    check_vec3(slabs[1].scope.origin, glm::dvec3{2.0, 0.0, 0.0}, 1e-12);
    check_vec3(slabs[1].scope.size, glm::dvec3{2.0, 1.0, 2.0}, 1e-12);
    CHECK_EQ(slabs[1].geometry.positions.size(), size_t{8});
    for (const glm::dvec3& position : slabs[1].geometry.positions) {
        CHECK((position.z) <= 2.0);
    }

    // Ten in the near slab: eight corners of the 2 by 4 box plus the two the
    // original wall left standing at z = 2. Every one of them is a vertex the
    // shape already had or a cut point shared by every face that needs it; an
    // edge that ENDS on the plane must reuse the vertex that is there rather than
    // compute a second copy of it.
    CHECK_EQ(slabs[0].geometry.positions.size(), size_t{10});
    check_vec3(slabs[0].scope.size, glm::dvec3{2.0, 1.0, 4.0}, 1e-12);
}

TEST(OpSplit, a_cut_through_a_hollow_column_gives_an_outline_with_a_bore) {
    // The header's own example, and the shape the whole ring-classification block
    // exists for: an outline, a hole inside it, and a cap that has to tell them
    // apart. Every other solid in this suite is a box, for which the block is
    // one rectangle and cannot be wrong.
    const Shape column = make_hollow_column();
    CHECK_NEAR(geometry_volume(column.geometry), 36.0, 1e-12);  // (16 - 4) * 3
    CHECK_TRUE(is_closed_solid(column.geometry));

    const Shape slab = slice_shape(column, ScopeAxis::Y, 1.0, 2.0);

    // Four outer walls, four bore walls and two caps. A bore emitted as a second
    // OUTLINE instead of as a hole gives twelve faces; a bore never attached at
    // all gives ten and 56 of area.
    CHECK_EQ(slab.geometry.faces.size(), size_t{10});
    CHECK_NEAR(geometry_volume(slab.geometry), 12.0, 1e-12);
    // 16 of outer wall, 8 of bore wall, and two caps of 16 - 4 = 12.
    CHECK_NEAR(geometry_area(slab.geometry), 48.0, 1e-12);
    check_vec3(total_vector_area(slab.geometry), glm::dvec3{0.0}, 1e-12);
    CHECK_TRUE(is_closed_solid(slab.geometry));
    check_vec3(slab.scope.origin, glm::dvec3{0.0, 1.0, 0.0}, 1e-12);
    check_vec3(slab.scope.size, glm::dvec3{4.0, 1.0, 4.0}, 1e-12);

    // Exactly two faces have a bore, they have one each, and they face opposite
    // ways along the axis: the near cap back along it, the far cap along it.
    size_t capped = 0;
    double facing = 0.0;
    for (const Face& face : slab.geometry.faces) {
        if (face.holes.empty()) {
            continue;
        }
        ++capped;
        CHECK_EQ(face.holes.size(), size_t{1});
        CHECK_NEAR(face_area(slab.geometry, face), 12.0, 1e-12);
        const glm::dvec3 normal = face_normal(slab.geometry, face);
        CHECK_NEAR(std::fabs(normal.y), 1.0, 1e-12);
        facing += normal.y;
    }
    CHECK_EQ(capped, size_t{2});
    CHECK_NEAR(facing, 0.0, 1e-12);
}

TEST(OpSplit, a_bore_inside_a_bore_is_attached_to_the_ring_that_owns_it) {
    // An 8 by 8 column with a 6 by 6 bore, and a 4 by 4 column with a 2 by 2 bore
    // standing inside that bore. A cut gives four rings nested one inside the
    // next, and the two rules that sort them out are both load-bearing only here:
    // PARITY says which rings are outlines (the 8 and the 4) and which are holes
    // (the 6 and the 2), and the SMALLEST containing ring says whose hole the
    // innermost is.
    //
    // Pick the largest container instead and the innermost bore is subtracted
    // from the outer cap: the total area is unchanged -- 28 + 12 becomes 24 + 16
    // -- and one cap carries two holes while the other carries none.
    Shape outer = shape_from_rings({{0.0, 0.0}, {8.0, 0.0}, {8.0, 8.0}, {0.0, 8.0}},
                                   {{{1.0, 1.0}, {7.0, 1.0}, {7.0, 7.0}, {1.0, 7.0}}});
    Shape inner = shape_from_rings({{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}},
                                   {{{1.0, 1.0}, {3.0, 1.0}, {3.0, 3.0}, {1.0, 3.0}}});
    const OpResult outer_up = extrude_shape(outer, ScopeAxis::Y, 3.0);
    const OpResult inner_up = extrude_shape(inner, ScopeAxis::Y, 3.0);
    CHECK_TRUE(outer_up.ok);
    CHECK_TRUE(inner_up.ok);
    append_geometry(outer.geometry, inner.geometry, glm::dvec3{2.0, 0.0, 2.0});
    refit_scope(outer);
    CHECK_TRUE(is_closed_solid(outer.geometry));

    const Shape slab = slice_shape(outer, ScopeAxis::Y, 1.0, 2.0);

    // Sixteen walls and four caps: an outline and its bore at each plane.
    CHECK_EQ(slab.geometry.faces.size(), size_t{20});
    // (64 - 36) + (16 - 4) = 40 of cross-section, one metre thick.
    CHECK_NEAR(geometry_volume(slab.geometry), 40.0, 1e-12);
    // 80 of wall -- 32, 24, 16 and 8 of perimeter -- and 40 of cap at each end.
    CHECK_NEAR(geometry_area(slab.geometry), 160.0, 1e-12);
    check_vec3(total_vector_area(slab.geometry), glm::dvec3{0.0}, 1e-12);
    CHECK_TRUE(is_closed_solid(slab.geometry));

    size_t wide = 0;
    size_t narrow = 0;
    for (const Face& face : slab.geometry.faces) {
        if (face.holes.empty()) {
            continue;
        }
        // One hole each: the ring that owns it, not the ring that merely contains it.
        CHECK_EQ(face.holes.size(), size_t{1});
        const double area = face_area(slab.geometry, face);
        if (area > 20.0) {
            ++wide;
            CHECK_NEAR(area, 28.0, 1e-12);
        } else {
            ++narrow;
            CHECK_NEAR(area, 12.0, 1e-12);
        }
    }
    CHECK_EQ(wide, size_t{2});
    CHECK_EQ(narrow, size_t{2});
}

TEST(OpSplit, a_cut_across_a_light_well_closes_around_the_well) {
    // A courtyard block split into two halves, which is an ordinary thing to ask
    // for and the case where the cut plane meets a HOLE. The well's ring bounds
    // the surface exactly as the outline does; harvest only the outline and the
    // cap roofs the well over, leaving a slab that is sealed, heavier than the
    // solid it came from, and open along the seam.
    const Shape column = make_hollow_column();
    const SplitLayout layout = solve_split({float_part(1.0), float_part(1.0)}, 4.0);
    const std::vector<Shape> halves = split_shape(column, ScopeAxis::X, layout);
    CHECK_EQ(halves.size(), size_t{2});
    if (halves.size() != 2) {
        return;
    }

    for (const Shape& half : halves) {
        // Cross-section 2 x 4 less half the bore, 1 x 2, so 6 -- three metres up.
        CHECK_NEAR(geometry_volume(half.geometry), 18.0, 1e-12);
        // 12 of face, 24 of outer wall, 12 of the three bore walls, and a cut
        // face of two strips 1 by 3: 54. Roofing the well over instead gives 22
        // of volume on one side, 60 of area, and a surface that does not close.
        CHECK_NEAR(geometry_area(half.geometry), 54.0, 1e-12);
        check_vec3(total_vector_area(half.geometry), glm::dvec3{0.0}, 1e-12);
        CHECK_EQ(half.geometry.faces.size(), size_t{10});
    }
    CHECK_NEAR(geometry_volume(halves[0].geometry) + geometry_volume(halves[1].geometry), 36.0,
               1e-12);

    // TWO faces at the cut, of 3 each, and not one of 12: the strips either side
    // of the well's mouth. This is the assertion the vector area cannot make on
    // its own, because a cap that roofed the well over AND a slab that lost a
    // wall would still sum to zero between them.
    size_t cut_faces = 0;
    double cut_area = 0.0;
    faces_in_plane(halves[0].geometry, 0, 2.0, cut_faces, cut_area);
    CHECK_EQ(cut_faces, size_t{2});
    CHECK_NEAR(cut_area, 6.0, 1e-12);
    faces_in_plane(halves[1].geometry, 0, 0.0, cut_faces, cut_area);
    CHECK_EQ(cut_faces, size_t{2});
    CHECK_NEAR(cut_area, 6.0, 1e-12);

    // What is NOT claimed here is edge pairing. The face that reaches the well
    // comes back in the form op_split.hpp documents -- a hole touching the
    // outline rather than a notch in it -- so its outline still runs in one edge
    // from one side of the mouth to the other while the cap is two strips. The
    // surface closes, and it closes across a T-vertex. Merging the two rings is
    // the boolean that header refuses to do, and asserting edges_pair_up() here
    // would be asserting that it happens.
    CHECK_FALSE(edges_pair_up(halves[0].geometry));

    // And a cut ALONG the wall of the well, which is the two cases at once: the
    // near slab is solid all the way across and takes an ordinary cap, while the
    // far slab must drop the well's wall -- the solid past x = 1 does not reach
    // it -- and cap only the two strips beside the mouth.
    const Shape solid = slice_shape(column, ScopeAxis::X, 0.0, 1.0);
    CHECK_EQ(solid.geometry.faces.size(), size_t{6});
    CHECK_NEAR(geometry_volume(solid.geometry), 12.0, 1e-12);
    CHECK_NEAR(geometry_area(solid.geometry), 38.0, 1e-12);
    CHECK_TRUE(is_closed_solid(solid.geometry));

    size_t solid_cut_faces = 0;
    double solid_cut_area = 0.0;
    faces_in_plane(solid.geometry, 0, 1.0, solid_cut_faces, solid_cut_area);
    CHECK_EQ(solid_cut_faces, size_t{1});
    CHECK_NEAR(solid_cut_area, 12.0, 1e-12);

    const Shape rest = slice_shape(column, ScopeAxis::X, 1.0, 4.0);
    CHECK_NEAR(geometry_volume(rest.geometry), 24.0, 1e-12);
    // 70, not the 82 of a slab that kept the well's wall as well as roofing the
    // mouth over, and not the 76 of one that dropped the wall and roofed it.
    CHECK_NEAR(geometry_area(rest.geometry), 70.0, 1e-12);
    check_vec3(total_vector_area(rest.geometry), glm::dvec3{0.0}, 1e-12);
    CHECK_NEAR(geometry_volume(solid.geometry) + geometry_volume(rest.geometry), 36.0, 1e-12);

    size_t rest_cut_faces = 0;
    double rest_cut_area = 0.0;
    faces_in_plane(rest.geometry, 0, 0.0, rest_cut_faces, rest_cut_area);
    CHECK_EQ(rest_cut_faces, size_t{2});
    CHECK_NEAR(rest_cut_area, 6.0, 1e-12);

    // The same cut twice, byte for byte. This is the cut with the most ordering
    // in it: the segments are split by distance along themselves and the
    // retraced ones are cancelled against a budget spent in segment order, and
    // neither may come out differently on a second run.
    const Shape again = slice_shape(column, ScopeAxis::X, 1.0, 4.0);
    CHECK_TRUE(geometry_is_identical(rest.geometry, again.geometry));
}

TEST(OpSplit, a_cut_whose_edges_do_not_close_is_not_capped) {
    // Half a box: a 2 x 1 x 1 prism with its top face taken off. The plane at
    // x = 1 crosses three faces and gets three cut edges that run out at both
    // ends, so there is no ring to cap and the slab stays as open as the surface
    // it was cut from. Keeping the chain anyway puts a face across a slab that
    // has no lid -- geometry invented to hide an input the cut cannot answer for.
    Shape open_box = shape_from_rect(2.0, 1.0);
    const OpResult extruded = extrude_shape(open_box, ScopeAxis::Y, 1.0);
    CHECK_TRUE(extruded.ok);
    CHECK_EQ(open_box.geometry.faces.size(), size_t{6});
    for (size_t i = 0; i < open_box.geometry.faces.size(); ++i) {
        if (face_normal(open_box.geometry, open_box.geometry.faces[i]).y > 0.5) {
            open_box.geometry.faces.erase(open_box.geometry.faces.begin() +
                                          static_cast<std::ptrdiff_t>(i));
            break;
        }
    }
    refit_scope(open_box);
    CHECK_EQ(open_box.geometry.faces.size(), size_t{5});
    CHECK_FALSE(edges_pair_up(open_box.geometry));

    const Shape slab = slice_shape(open_box, ScopeAxis::X, 0.0, 1.0);

    // Four clipped walls and no fifth face. 1 + 1 + 1 + 1 of area, against 2 with
    // a cap over the cut.
    CHECK_EQ(slab.geometry.faces.size(), size_t{4});
    CHECK_NEAR(geometry_area(slab.geometry), 4.0, 1e-12);
    CHECK_FALSE(edges_pair_up(slab.geometry));
    for (const Face& face : slab.geometry.faces) {
        bool in_the_plane = true;
        for (const uint32_t index : face.loop) {
            if (slab.geometry.positions[index].x != 1.0) {
                in_the_plane = false;
            }
        }
        CHECK_FALSE(in_the_plane);
    }
}

TEST(OpSplit, coincident_vertices_are_welded_so_the_cut_still_chains_into_a_ring) {
    // The same 2 x 3 x 4 box with every face carrying its own copy of every
    // corner: twenty-four positions where there were eight, which is what an
    // imported mesh hands over and what shape.cpp's own add_ring() produces.
    //
    // By index the four cut points are then twelve, and the cap ring is eight
    // open chains that close nothing. The chaining welds by POSITION, and this is
    // the only shape here that can tell whether it does.
    const Shape box = with_duplicated_vertices(make_box());
    CHECK_EQ(box.geometry.positions.size(), size_t{24});
    CHECK_EQ(box.geometry.faces.size(), size_t{6});
    CHECK_TRUE(is_closed_solid(box.geometry));

    const Shape slab = slice_shape(box, ScopeAxis::X, 0.5, 1.5);
    CHECK_EQ(slab.geometry.faces.size(), size_t{6});
    CHECK_NEAR(geometry_volume(slab.geometry), 12.0, 1e-12);
    CHECK_NEAR(geometry_area(slab.geometry), 38.0, 1e-12);
    check_vec3(total_vector_area(slab.geometry), glm::dvec3{0.0}, 1e-12);
    CHECK_TRUE(is_closed_solid(slab.geometry));
}

TEST(OpSplit, the_cap_takes_the_material_of_the_shape_it_closes) {
    // A cut face of a brick wall is brick. Nothing else in the file carries a
    // material across the cut, so a cap that defaulted would come back as
    // MaterialId::Default and land in the wrong submesh -- visible in the
    // viewport, and invisible to every volume and area assertion here.
    Shape box = make_box();
    const MaterialKey brick{MaterialId::Wall, 7};
    for (Face& face : box.geometry.faces) {
        face.material = brick;
    }

    const Shape slab = slice_shape(box, ScopeAxis::X, 0.5, 1.5);
    CHECK_EQ(slab.geometry.faces.size(), size_t{6});
    size_t caps = 0;
    for (const Face& face : slab.geometry.faces) {
        CHECK_TRUE(face.material == brick);
        if (std::fabs(face_normal(slab.geometry, face).x) > 0.5) {
            ++caps;
        }
    }
    // Both ends of this slab are cut, so both of its x-facing faces are caps.
    CHECK_EQ(caps, size_t{2});
}

TEST(OpSplit, a_clipped_ring_never_visits_a_vertex_twice) {
    // The clip walks the ring from its first vertex and emits an intersection as
    // it leaves the slab. When the ring STARTS on the cut plane, the closing edge
    // comes back to that same vertex and emits it a second time -- so the ring
    // ends on the vertex it began with, and that is a wrap-around duplicate that
    // consecutive de-duplication cannot see. The face keeps the same area and the
    // same normal, and carries a degenerate triangle into every triangulation.
    //
    // Which vertex a ring starts at is a property of the ROTATION of the ring,
    // and shape_from_polygon() keeps the rotation it is handed, so this shape is
    // built vertex by vertex: it has to start at (2, 0), the corner that sits on
    // the plane, with an outside vertex immediately before it. No rotation of the
    // same quad through shape_from_polygon() reaches this case, which is how it
    // went untested.
    Shape quad;
    quad.geometry.positions = {
        {2.0, 0.0, 0.0}, {4.0, 0.0, 0.0}, {4.0, 0.0, 4.0}, {0.0, 0.0, 4.0}};
    Face face;
    face.loop = {0, 1, 2, 3};
    quad.geometry.faces.push_back(face);
    refit_scope(quad);
    CHECK_NEAR(geometry_area(quad.geometry), 12.0, 1e-12);

    const Shape slab = slice_shape(quad, ScopeAxis::X, 2.0, 4.0);
    CHECK_EQ(slab.geometry.faces.size(), size_t{1});
    if (!slab.geometry.faces.empty()) {
        // Four, not five: (2,0), (4,0), (4,4) and the cut at (2,4).
        CHECK_EQ(slab.geometry.faces[0].loop.size(), size_t{4});
    }
    CHECK_FALSE(a_ring_repeats_a_vertex(slab.geometry));
    CHECK_EQ(slab.geometry.positions.size(), size_t{4});
    CHECK_NEAR(geometry_area(slab.geometry), 8.0, 1e-12);
}

// ============================================================================
// The AST bridge
// ============================================================================

TEST(OpSplit, to_scope_axis_maps_each_axis_to_the_one_the_author_named) {
    CHECK_TRUE(to_scope_axis(SplitAxis::X) == ScopeAxis::X);
    CHECK_TRUE(to_scope_axis(SplitAxis::Y) == ScopeAxis::Y);
    CHECK_TRUE(to_scope_axis(SplitAxis::Z) == ScopeAxis::Z);
}

TEST(OpSplit, a_parsed_facade_rule_drives_the_solver_and_names_the_rule_for_every_piece) {
    // Rule TEXT to a solved layout to the rule each piece must run: everything
    // the interpreter's split case needs, short of creating the child shapes.
    const ParseResult parsed = must_parse(
        "rule Corner { trim(); }\n"
        "rule Bay { trim(); }\n"
        "rule Pier { trim(); }\n"
        "@start\n"
        "rule Facade {\n"
        "  split(x) {\n"
        "    1.0 : Corner();\n"
        "    repeat {\n"
        "      ~2.5 : Bay();\n"
        "      0.4  : Pier();\n"
        "    }\n"
        "    1.0 : Corner();\n"
        "  }\n"
        "}\n");
    const RuleFile& file = parsed.file;

    const SplitStmt* split = first_split(file);
    CHECK_TRUE(split != nullptr);
    if (split == nullptr) {
        return;
    }
    CHECK_TRUE(to_scope_axis(split->axis) == ScopeAxis::X);

    const std::vector<SplitPart> parts = split_parts_from_statement(
        *split, [&](ExprId id, const stratum::procgen::rules::SourceLoc&) {
            return literal_size(file, id);
        });
    CHECK_EQ(parts.size(), size_t{3});
    if (parts.size() != 3) {
        return;
    }
    // The kinds survived the crossing. A bridge that lost `~` would make the bay
    // a fixed 2.5 and the whole facade would be one width.
    CHECK_TRUE(parts[0].kind == SizeKind::Absolute);
    CHECK_TRUE(parts[1].is_repeat);
    CHECK_EQ(parts[1].children.size(), size_t{2});
    CHECK_TRUE(parts[1].children[0].kind == SizeKind::Floating);
    CHECK_NEAR(parts[1].children[0].value, 2.5, 0.0);
    CHECK_TRUE(parts[1].children[1].kind == SizeKind::Absolute);
    CHECK_NEAR(parts[1].children[1].value, 0.4, 0.0);

    // The same numbers the solver suite computes by hand, reached from rule text.
    const SplitLayout layout = solve_split(parts, 20.0);
    CHECK_EQ(layout.pieces.size(), size_t{14});
    CHECK_EQ(layout.repeat_copies, uint32_t{6});

    // And every piece can name the rule it must run. Corner, then six bay-and-pier
    // pairs, then Corner -- in that order, which is the assertion that catches a
    // piece pointing at the wrong entry's body.
    std::string sequence;
    for (const SplitPiece& piece : layout.pieces) {
        const std::string name = body_calls(file, split_body_for(*split, piece));
        CHECK_FALSE(name.empty());
        sequence += name;
        sequence += " ";
    }
    CHECK_EQ(sequence,
             std::string{"Corner Bay Pier Bay Pier Bay Pier Bay Pier Bay Pier Bay Pier Corner "});
}

TEST(OpSplit, a_repeat_nested_in_a_repeat_is_carried_across_rather_than_evaluated) {
    // The parser rejects a repeat inside a repeat, so this cannot arrive from
    // rule text. split_parts_from_statement() is public, takes a SplitStmt, and
    // is exactly what a caller building one of its own reaches for.
    //
    // A repeat entry has no size expression. Evaluating its children's sizes
    // anyway hands the callback kNoNode, and the interpreter's evaluator reports
    // that as "a split size is not a number" at a SourceLoc of nothing -- a
    // diagnostic naming line 0 of a rule the author never wrote.
    SplitStmt stmt;
    SplitEntry group;
    group.is_repeat = true;
    SplitEntry nested;
    nested.is_repeat = true;
    group.children.push_back(nested);
    SplitEntry sized;
    sized.size.kind = SizeKind::Absolute;
    sized.size.value = 7;  // any live ExprId; the evaluator below answers for it
    group.children.push_back(sized);
    stmt.entries.push_back(group);

    std::vector<ExprId> asked;
    const std::vector<SplitPart> parts = split_parts_from_statement(
        stmt, [&](ExprId id, const stratum::procgen::rules::SourceLoc&) {
            asked.push_back(id);
            return 2.0;
        });

    // Asked once, for the one child that HAS a size.
    CHECK_EQ(asked.size(), size_t{1});
    if (!asked.empty()) {
        CHECK_EQ(asked[0], ExprId{7});
    }
    CHECK_EQ(parts.size(), size_t{1});
    if (parts.size() != 1 || parts[0].children.size() != 2) {
        return;
    }
    CHECK_TRUE(parts[0].is_repeat);
    CHECK_TRUE(parts[0].children[0].is_repeat);
    CHECK_FALSE(parts[0].children[1].is_repeat);
    CHECK_NEAR(parts[0].children[1].value, 2.0, 0.0);

    // And the solver leaves the nested group out rather than laying a phantom
    // piece for it: the period is the sized child's 2 alone, so an extent of 10
    // tiles five copies of it, every piece naming child 1.
    const SplitLayout layout = solve_split(parts, 10.0);
    CHECK_EQ(layout.repeat_copies, uint32_t{5});
    CHECK_EQ(layout.pieces.size(), size_t{5});
    CHECK_TRUE(a_note_says(layout, "cannot tile"));
    for (const SplitPiece& piece : layout.pieces) {
        CHECK_EQ(piece.child, size_t{1});
        CHECK_NEAR(piece.length, 2.0, 1e-12);
    }
}

TEST(OpSplit, an_entry_lookup_refuses_a_piece_that_does_not_belong_to_the_statement) {
    // A layout and a statement that do not match is a caller bug, and answering
    // it with the nearest entry would run a rule the author never wrote there.
    const ParseResult parsed = must_parse(
        "rule A { trim(); }\n"
        "@start\n"
        "rule R { split(y) { 1 : A(); repeat { 2 : A(); } } }\n");
    const SplitStmt* split = first_split(parsed.file);
    CHECK_TRUE(split != nullptr);
    if (split == nullptr) {
        return;
    }

    SplitPiece past_the_end;
    past_the_end.part = 7;
    CHECK_TRUE(split_entry_for(*split, past_the_end) == nullptr);
    CHECK_EQ(split_body_for(*split, past_the_end), kNoNode);

    // Part 0 is not a repeat, so naming a child of it is nonsense.
    SplitPiece wrong_child;
    wrong_child.part = 0;
    wrong_child.child = 0;
    CHECK_TRUE(split_entry_for(*split, wrong_child) == nullptr);

    // Part 1 IS a repeat, so a piece with no child names no entry either.
    SplitPiece missing_child;
    missing_child.part = 1;
    missing_child.child = kNoPart;
    CHECK_TRUE(split_entry_for(*split, missing_child) == nullptr);

    SplitPiece good;
    good.part = 1;
    good.child = 0;
    const SplitEntry* entry = split_entry_for(*split, good);
    CHECK_TRUE(entry != nullptr);
    if (entry != nullptr) {
        CHECK_FALSE(entry->is_repeat);
        CHECK_EQ(body_calls(parsed.file, entry->body), std::string{"A"});
    }
}
