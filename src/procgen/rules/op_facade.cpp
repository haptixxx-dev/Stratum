// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_facade.cpp
 * @brief E3: window, door and wall_panel
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The argument for every choice made here is in op_facade.hpp. What is in this
 * file is the arithmetic, and it is all axis-aligned arithmetic in ONE frame:
 * orient_panel() puts the shape in a frame where x runs across the panel, y runs
 * up it and z points out of the wall, and after that every corner of every part
 * is a sum of the panel size, the inset, the reveal, the frame width and the
 * ledge. No rotation, no projection, no trigonometry.
 *
 * The one piece of machinery worth naming is add_quad(). Every quad in a facade
 * has an outward direction that is known before the quad is built -- a jamb
 * faces into the opening, a sill's underside faces down -- and getting a winding
 * backwards produces a face that renders black from the street and is invisible
 * in every scope assertion. So add_quad() takes the outward direction and
 * REVERSES the loop when Newell disagrees with it, exactly as extrude_shape()
 * does for its caps. There is no way to write an inside-out face here, and the
 * tests assert every normal rather than trusting that.
 */

#include "procgen/rules/op_facade.hpp"

#include "osm/road/road_style.hpp"

#include "procgen/rules/op_comp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace stratum::procgen::rules {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/**
 * @brief A face with less area than this is not a face
 *
 * A square micron. Used to decide how many faces a shape really has, which is
 * the check that stands between `window()` and a solid box.
 */
constexpr double kFacadeAreaEpsilon = 1.0e-12;

/**
 * @brief How far out of plane a panel may be, relative to its own size
 *
 * A bent quad has no single plane to cut an opening in, and op_comp.hpp already
 * warns about them when a `select face` produces one. Relative rather than
 * absolute because a 100 m wall and a 1 m tile do not have the same idea of
 * flat, and because reframe() re-expresses every vertex through a transpose and
 * leaves rounding proportional to the coordinates.
 */
constexpr double kPanelFlatnessTolerance = 1.0e-9;

/// Squared length below which a Newell normal is not a direction
constexpr double kDegenerateNormalSq = 1.0e-24;

// ============================================================================
// Quads
// ============================================================================

/**
 * @brief Append one quad, wound so that its normal agrees with @p outward
 *
 * See the file comment for why the direction is an argument rather than a
 * convention. A quad with no area is DROPPED rather than appended: the callers
 * below emit strips whose width is often exactly zero -- a window with no inset
 * has no surround at all -- and a zero-area face is something every consumer
 * downstream has to special-case.
 *
 * @return true when a face was appended
 */
bool add_quad(ShapeGeometry& geometry,
              const glm::dvec3& a,
              const glm::dvec3& b,
              const glm::dvec3& c,
              const glm::dvec3& d,
              const glm::dvec3& outward,
              FacadePart part) {
    const std::array<glm::dvec3, 4> corners{a, b, c, d};

    // Newell, the same method face_normal() uses, so the two can never disagree
    // about which way this face ends up pointing.
    glm::dvec3 normal{0.0};
    for (size_t i = 0; i < 4; ++i) {
        const glm::dvec3& p = corners[i];
        const glm::dvec3& q = corners[(i + 1) % 4];
        normal.x += (p.y - q.y) * (p.z + q.z);
        normal.y += (p.z - q.z) * (p.x + q.x);
        normal.z += (p.x - q.x) * (p.y + q.y);
    }
    if (glm::dot(normal, normal) <= kDegenerateNormalSq) {
        return false;
    }

    const bool flip = glm::dot(normal, outward) < 0.0;
    const uint32_t base = static_cast<uint32_t>(geometry.positions.size());
    for (const glm::dvec3& corner : corners) {
        geometry.positions.push_back(corner);
    }

    Face face;
    face.material = facade_material(part);
    if (flip) {
        face.loop = {base + 3, base + 2, base + 1, base};
    } else {
        face.loop = {base, base + 1, base + 2, base + 3};
    }
    geometry.faces.push_back(std::move(face));
    return true;
}

/// A rectangle in a z = @p z plane, facing @p dir along z. Skipped when it has no area.
void add_rect_z(ShapeGeometry& geometry,
                double x0,
                double y0,
                double x1,
                double y1,
                double z,
                double dir,
                FacadePart part) {
    if (x1 - x0 <= kFacadeEpsilon || y1 - y0 <= kFacadeEpsilon) {
        return;
    }
    (void)add_quad(geometry, {x0, y0, z}, {x1, y0, z}, {x1, y1, z}, {x0, y1, z},
                   glm::dvec3{0.0, 0.0, dir}, part);
}

/// A rectangle in an x = @p x plane, facing @p dir along x
void add_rect_x(ShapeGeometry& geometry,
                double x,
                double y0,
                double z0,
                double y1,
                double z1,
                double dir,
                FacadePart part) {
    if (y1 - y0 <= kFacadeEpsilon || z1 - z0 <= kFacadeEpsilon) {
        return;
    }
    (void)add_quad(geometry, {x, y0, z0}, {x, y1, z0}, {x, y1, z1}, {x, y0, z1},
                   glm::dvec3{dir, 0.0, 0.0}, part);
}

/// A rectangle in a y = @p y plane, facing @p dir along y
void add_rect_y(ShapeGeometry& geometry,
                double y,
                double x0,
                double z0,
                double x1,
                double z1,
                double dir,
                FacadePart part) {
    if (x1 - x0 <= kFacadeEpsilon || z1 - z0 <= kFacadeEpsilon) {
        return;
    }
    (void)add_quad(geometry, {x0, y, z0}, {x1, y, z0}, {x1, y, z1}, {x0, y, z1},
                   glm::dvec3{0.0, dir, 0.0}, part);
}

/**
 * @brief A closed axis-aligned box with outward normals
 *
 * The sill and the threshold are solids, not surfaces: they are seen from below
 * by anyone standing in the street, which is the one viewing angle a flat ledge
 * gives itself away from.
 */
void add_box(ShapeGeometry& geometry,
             const glm::dvec3& min,
             const glm::dvec3& max,
             FacadePart part) {
    if (max.x - min.x <= kFacadeEpsilon || max.y - min.y <= kFacadeEpsilon ||
        max.z - min.z <= kFacadeEpsilon) {
        return;
    }
    add_rect_x(geometry, min.x, min.y, min.z, max.y, max.z, -1.0, part);
    add_rect_x(geometry, max.x, min.y, min.z, max.y, max.z, 1.0, part);
    add_rect_y(geometry, min.y, min.x, min.z, max.x, max.z, -1.0, part);
    add_rect_y(geometry, max.y, min.x, min.z, max.x, max.z, 1.0, part);
    add_rect_z(geometry, min.x, min.y, max.x, max.y, min.z, -1.0, part);
    add_rect_z(geometry, min.x, min.y, max.x, max.y, max.z, 1.0, part);
}

// ============================================================================
// Finding the panel
// ============================================================================

/// Copy one face, and only that face, into a geometry of its own
[[nodiscard]] ShapeGeometry extract_face(const ShapeGeometry& geometry, const Face& face) {
    ShapeGeometry out;
    Face copy;
    copy.material = face.material;
    for (const uint32_t index : face.loop) {
        copy.loop.push_back(static_cast<uint32_t>(out.positions.size()));
        out.positions.push_back(geometry.positions[index]);
    }
    for (const std::vector<uint32_t>& hole : face.holes) {
        std::vector<uint32_t> ring;
        for (const uint32_t index : hole) {
            ring.push_back(static_cast<uint32_t>(out.positions.size()));
            out.positions.push_back(geometry.positions[index]);
        }
        copy.holes.push_back(std::move(ring));
    }
    out.faces.push_back(std::move(copy));
    return out;
}

/**
 * @brief The one face a facade operation works on
 *
 * Exactly one face with area. Two is not "pick the biggest": a rule that hands a
 * solid box to `window()` meant to select a wall first and has a bug, and
 * silently glazing whichever face happened to be largest would put the window on
 * a different side of the building the moment the box changed proportion.
 */
[[nodiscard]] bool find_panel_face(const Shape& shape, size_t& index_out, std::string& why) {
    size_t found = 0;
    size_t count = 0;
    for (size_t i = 0; i < shape.geometry.faces.size(); ++i) {
        if (face_area(shape.geometry, shape.geometry.faces[i]) > kFacadeAreaEpsilon) {
            if (count == 0) {
                found = i;
            }
            ++count;
        }
    }
    if (count == 0) {
        why = "needs a panel with some area to work on, and this shape has none";
        return false;
    }
    if (count > 1) {
        why = "needs one face to work on, and this shape has " + std::to_string(count) +
              "; pick the wall with `select face` first";
        return false;
    }
    index_out = found;
    return true;
}

// ============================================================================
// Resolving the numbers
// ============================================================================

/// The wall thickness, in the order op_facade.hpp's header note gives
[[nodiscard]] double resolve_thickness(const Shape& shape,
                                       const OpeningParams& params,
                                       FacadeReport& report) {
    // A thickness of zero is not a wall, so zero is treated as absent here even
    // though zero is a meaningful value for every other length in OpeningParams.
    if (params.thickness > 0.0) {
        return params.thickness;
    }

    const auto found = shape.attributes.find(kWallThicknessAttribute);
    if (found != shape.attributes.end()) {
        if (!found->second.is_number()) {
            report.warnings.push_back(std::string{"'"} + kWallThicknessAttribute + "' is a " +
                                      found->second.type_name() +
                                      " and not a number, so the default wall thickness of " +
                                      format_number(kDefaultWallThickness) + " was used");
        } else if (found->second.as_number() <= 0.0) {
            report.warnings.push_back(std::string{"'"} + kWallThicknessAttribute + "' is " +
                                      format_number(found->second.as_number()) +
                                      " and not a positive length, so the default wall thickness of " +
                                      format_number(kDefaultWallThickness) + " was used");
        } else {
            return found->second.as_number();
        }
    }
    return kDefaultWallThickness;
}

// ============================================================================
// The opening
// ============================================================================

/// Everything one opening needs, solved, in the panel frame
struct OpeningPlan {
    double panel_width = 0.0;
    double panel_height = 0.0;
    double x0 = 0.0;   ///< Opening minimum across the panel
    double x1 = 0.0;   ///< Opening maximum across the panel
    double y0 = 0.0;   ///< Opening minimum up the panel
    double y1 = 0.0;   ///< Opening maximum up the panel
    double reveal = 0.0;
    double frame = 0.0;
    double ledge = 0.0;            ///< Sill or threshold projection out past the wall face
    double ledge_thickness = 0.0;  ///< Its vertical thickness
    bool door = false;

    /**
     * @brief The frame is wide enough to swallow its own opening
     *
     * Decided by the caller and carried here rather than recomputed, because the
     * caller has to warn about it and the two answers must be the same one. A
     * second copy of the comparison is a second chance to get the door's
     * three-member frame wrong.
     */
    bool frame_fills = false;
};

/**
 * @brief Build every face of one opening into @p out
 *
 * The order is the order a reader would draw it in -- surround, reveal, frame,
 * glazing, ledge -- and that order is part of the output: dump_shape() prints
 * the faces in index order, so reordering this function changes the golden text
 * for every facade in the project.
 */
void build_opening(const OpeningPlan& plan, ShapeGeometry& out) {
    const double w = plan.panel_width;
    const double h = plan.panel_height;
    const double x0 = plan.x0;
    const double x1 = plan.x1;
    const double y0 = plan.y0;
    const double y1 = plan.y1;
    const double back = -plan.reveal;
    const bool has_ledge = plan.ledge > kFacadeEpsilon;

    // ---- the surround: the wall left around the opening ----
    //
    // Four strips of the panel's own box rather than the panel outline with a
    // hole in it, because a door's opening TOUCHES the bottom edge and a polygon
    // with a hole cannot express that. See the header note on why the panel has
    // to be a rectangle for this to be exact.
    //
    // The bottom strip stops short of the ledge, which occupies the wall from
    // y0 - thickness up to y0. Without that the ledge solid and the wall surface
    // would interpenetrate along the one edge a passer-by looks straight at.
    const double surround_bottom =
        has_ledge ? std::max(0.0, y0 - plan.ledge_thickness) : y0;
    add_rect_z(out, 0.0, 0.0, x0, h, 0.0, 1.0, FacadePart::Panel);
    add_rect_z(out, x1, 0.0, w, h, 0.0, 1.0, FacadePart::Panel);
    add_rect_z(out, x0, 0.0, x1, surround_bottom, 0.0, 1.0, FacadePart::Panel);
    add_rect_z(out, x0, y1, x1, h, 0.0, 1.0, FacadePart::Panel);

    // ---- the reveal: the returns from the wall face back to the glazing line ----
    //
    // Each one faces INTO the opening, which is what makes the depth read from
    // the street: the jamb on the left of the opening is lit from the right.
    if (plan.reveal > kFacadeEpsilon) {
        add_rect_x(out, x0, y0, back, y1, 0.0, 1.0, FacadePart::Reveal);
        add_rect_x(out, x1, y0, back, y1, 0.0, -1.0, FacadePart::Reveal);
        add_rect_y(out, y1, x0, back, x1, 0.0, -1.0, FacadePart::Reveal);
        // The bottom return is the ledge's top face when there is a ledge, so
        // emitting both would put two surfaces in the same place.
        if (!has_ledge) {
            add_rect_y(out, y0, x0, back, x1, 0.0, 1.0, FacadePart::Reveal);
        }
    }

    // ---- the frame, and the glazing or leaf it holds ----
    const double f = plan.frame;
    const FacadePart infill = plan.door ? FacadePart::Leaf : FacadePart::Glass;

    if (f <= kFacadeEpsilon) {
        // No frame at all: the opening is filled edge to edge.
        add_rect_z(out, x0, y0, x1, y1, back, 1.0, infill);
    } else if (plan.frame_fills) {
        // The frame is wide enough to swallow its own opening. Filling the
        // opening with frame is the honest answer: it is what the numbers say,
        // it is visible, and the warning names it.
        add_rect_z(out, x0, y0, x1, y1, back, 1.0, FacadePart::Frame);
    } else {
        add_rect_z(out, x0, y0, x0 + f, y1, back, 1.0, FacadePart::Frame);
        add_rect_z(out, x1 - f, y0, x1, y1, back, 1.0, FacadePart::Frame);
        if (!plan.door) {
            add_rect_z(out, x0 + f, y0, x1 - f, y0 + f, back, 1.0, FacadePart::Frame);
        }
        add_rect_z(out, x0 + f, y1 - f, x1 - f, y1, back, 1.0, FacadePart::Frame);

        const double infill_bottom = plan.door ? y0 : y0 + f;
        add_rect_z(out, x0 + f, infill_bottom, x1 - f, y1 - f, back, 1.0, infill);
    }

    // ---- the sill or the threshold ----
    //
    // It spans the opening exactly and does not oversail it sideways. A real
    // sill does oversail, and a sill that oversails here would push through the
    // surround strips beside it -- this is a render mesh, but a wall that
    // visibly grows out of its own window is worse than a sill that stops short.
    if (has_ledge) {
        add_box(out, glm::dvec3{x0, y0 - plan.ledge_thickness, back},
                glm::dvec3{x1, y0, plan.ledge},
                plan.door ? FacadePart::Threshold : FacadePart::Sill);
    }
}

// ============================================================================
// Shared body of window and door
// ============================================================================

[[nodiscard]] OpResult build_facade_opening(Shape& shape,
                                            const OpeningParams& params,
                                            bool door,
                                            FacadeReport* report) {
    FacadeReport scratch;
    FacadeReport& out = report != nullptr ? *report : scratch;
    out = FacadeReport{};

    std::string why;
    if (!orient_panel(shape, why)) {
        return OpResult::failure(why);
    }

    // ---- the panel has to be a rectangle; see the header note ----
    const Face& face = shape.geometry.faces.front();
    const double w = shape.scope.size.x;
    const double h = shape.scope.size.y;
    if (w <= kFacadeEpsilon || h <= kFacadeEpsilon) {
        return OpResult::failure("needs a panel with width and height, and this one is " +
                                 format_number(w) + " by " + format_number(h));
    }
    const double area = face_area(shape.geometry, face);
    const double box_area = w * h;
    if (!face.holes.empty() || face.loop.size() != 4 ||
        std::fabs(area - box_area) > kPanelFlatnessTolerance * box_area) {
        return OpResult::failure(
            "needs a rectangular panel; this face has " + std::to_string(face.loop.size()) +
            " corners, " + std::to_string(face.holes.size()) + " holes and an area of " +
            format_number(area) + " where its box " + format_number(w) + " by " +
            format_number(h) + " has " + format_number(box_area) +
            "; split it into rectangles first");
    }
    out.panel_width = w;
    out.panel_height = h;

    // ---- the lengths ----
    const double thickness = resolve_thickness(shape, params, out);
    out.wall_thickness = thickness;

    double reveal = params.reveal < 0.0 ? thickness * kDefaultRevealFraction : params.reveal;
    if (reveal > thickness) {
        out.warnings.push_back("a reveal of " + format_number(reveal) +
                               " is deeper than the wall is thick (" + format_number(thickness) +
                               "), so it was cut back to the wall thickness");
        out.reveal_clamped = true;
        reveal = thickness;
    }
    out.reveal = reveal;

    const double frame = params.frame < 0.0 ? kDefaultFrameWidth : params.frame;
    const double default_ledge =
        door ? kDefaultThresholdProjection : kDefaultSillProjection;
    const double ledge = params.ledge < 0.0 ? default_ledge : params.ledge;

    // ---- the opening ----
    //
    // A door's inset is applied to the left, the right and the head and never to
    // the foot, because a door stands on the floor. That is the whole geometric
    // difference between this function's two callers.
    OpeningPlan plan;
    plan.panel_width = w;
    plan.panel_height = h;
    plan.x0 = params.inset;
    plan.x1 = w - params.inset;
    plan.y0 = door ? 0.0 : params.inset;
    plan.y1 = h - params.inset;
    plan.reveal = reveal;
    plan.frame = frame;
    plan.ledge = ledge;
    plan.ledge_thickness = door ? kThresholdThickness : kSillThickness;
    plan.door = door;

    if (plan.x0 < 0.0 || plan.y0 < 0.0 || plan.x1 > w || plan.y1 > h) {
        out.warnings.push_back(
            "an opening " + format_number(plan.x1 - plan.x0) + " by " +
            format_number(plan.y1 - plan.y0) + " does not fit a panel " + format_number(w) +
            " by " + format_number(h) + ", so it was clipped to the panel");
        out.opening_clipped = true;
        plan.x0 = std::max(0.0, plan.x0);
        plan.y0 = std::max(0.0, plan.y0);
        plan.x1 = std::min(w, plan.x1);
        plan.y1 = std::min(h, plan.y1);
    }

    ShapeGeometry built;

    if (plan.x1 - plan.x0 <= kFacadeEpsilon || plan.y1 - plan.y0 <= kFacadeEpsilon) {
        // Nothing left to cut. A blank wall panel, not a failure and not a hole:
        // see the header note.
        out.warnings.push_back("an inset of " + format_number(params.inset) +
                               " leaves no opening in a panel " + format_number(w) + " by " +
                               format_number(h) + ", so a blank wall panel was made instead");
        out.opening_empty = true;
        out.opening = glm::dvec4{0.0};
        add_rect_z(built, 0.0, 0.0, w, h, 0.0, 1.0, FacadePart::Panel);
    } else {
        out.opening = glm::dvec4{plan.x0, plan.y0, plan.x1, plan.y1};

        // A door's frame has three members, not four: the leaf reaches the
        // floor, so only the head eats into the opening's height.
        const double opening_w = plan.x1 - plan.x0;
        const double opening_h = plan.y1 - plan.y0;
        const double frame_height_needed = door ? frame : 2.0 * frame;
        plan.frame_fills = frame > kFacadeEpsilon &&
                           (2.0 * frame >= opening_w - kFacadeEpsilon ||
                            frame_height_needed >= opening_h - kFacadeEpsilon);
        if (plan.frame_fills) {
            out.warnings.push_back("a frame " + format_number(frame) +
                                   " wide fills an opening " + format_number(opening_w) +
                                   " by " + format_number(opening_h) +
                                   ", so there is no glazing in it");
            out.frame_filled_opening = true;
        }

        build_opening(plan, built);
    }

    shape.geometry = std::move(built);
    refit_scope(shape);
    return OpResult::success();
}

// ============================================================================
// Argument helpers
//
// Copies of interpreter.cpp's, which are in an anonymous namespace there. A
// shared header for four lines of argument checking would couple every
// operation family to every other one's idea of a diagnostic.
// ============================================================================

[[nodiscard]] bool op_number(OperationArgs& context, size_t index, double& value) {
    if (index >= context.args.size() || !context.args[index].is_number()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a number for argument " +
                             std::to_string(index + 1));
        return false;
    }
    value = context.args[index].as_number();
    return true;
}

/// A length argument that may not be negative
[[nodiscard]] bool op_length(OperationArgs& context,
                             size_t index,
                             const char* what,
                             double& value) {
    if (!op_number(context, index, value)) {
        return false;
    }
    if (value < 0.0) {
        context.interpreter.fail_shape(context.loc, "'" + std::string{context.name} + "' wants " +
                                                        what + " of zero or more, not " +
                                                        format_number(value));
        return false;
    }
    return true;
}

/// Turn every warning the operation recorded into a diagnostic at the call site
void report_warnings(OperationArgs& context, const FacadeReport& report) {
    for (const std::string& warning : report.warnings) {
        context.interpreter.report(Severity::Warning, context.loc,
                                   "'" + std::string{context.name} + "': " + warning);
    }
}

// ============================================================================
// Handlers
// ============================================================================

void op_window(OperationArgs& context) {
    OpeningParams params;
    if (!context.args.empty() && !op_number(context, 0, params.inset)) {
        return;
    }
    if (context.args.size() >= 2 && !op_length(context, 1, "a reveal", params.reveal)) {
        return;
    }
    if (context.args.size() >= 3 && !op_length(context, 2, "a frame width", params.frame)) {
        return;
    }
    if (context.args.size() >= 4 && !op_length(context, 3, "a sill projection", params.ledge)) {
        return;
    }
    if (context.args.size() >= 5) {
        if (!op_number(context, 4, params.thickness)) {
            return;
        }
        if (params.thickness <= 0.0) {
            context.interpreter.fail_shape(context.loc,
                                           "'window' wants a positive wall thickness, not " +
                                               format_number(params.thickness));
            return;
        }
    }

    FacadeReport report;
    const OpResult result = window_shape(context.shape, params, &report);
    report_warnings(context, report);
    if (!result.ok) {
        context.interpreter.fail_shape(context.loc, "'window' " + result.message);
    }
}

void op_door(OperationArgs& context) {
    OpeningParams params;
    if (!context.args.empty() && !op_number(context, 0, params.inset)) {
        return;
    }
    if (context.args.size() >= 2 && !op_length(context, 1, "a reveal", params.reveal)) {
        return;
    }
    if (context.args.size() >= 3 && !op_length(context, 2, "a frame width", params.frame)) {
        return;
    }
    if (context.args.size() >= 4 &&
        !op_length(context, 3, "a threshold projection", params.ledge)) {
        return;
    }

    FacadeReport report;
    const OpResult result = door_shape(context.shape, params, &report);
    report_warnings(context, report);
    if (!result.ok) {
        context.interpreter.fail_shape(context.loc, "'door' " + result.message);
    }
}

void op_wall_panel(OperationArgs& context) {
    double inset = 0.0;
    double thickness = 0.0;
    if (!context.args.empty() && !op_length(context, 0, "an inset", inset)) {
        return;
    }
    if (context.args.size() >= 2 && !op_length(context, 1, "a thickness", thickness)) {
        return;
    }

    FacadeReport report;
    const OpResult result = wall_panel_shape(context.shape, inset, thickness, &report);
    report_warnings(context, report);
    if (!result.ok) {
        context.interpreter.fail_shape(context.loc, "'wall_panel' " + result.message);
    }
}

[[nodiscard]] OperationTable build_facade_operations() {
    OperationTable table = standard_operations();
    register_facade_operations(table);
    return table;
}

} // namespace

// ============================================================================
// Materials
// ============================================================================

MaterialKey facade_material(FacadePart part) {
    // NOT `variant = part`. The Wall slot's variants are a published
    // vocabulary -- osm/road/road_style.hpp names them kWallDefault 0,
    // kWallBrick 1, kWallStone 2, kWallConcrete 3, kWallRender 4, kWallGlass 5,
    // kWallMetal 6, kWallWood 7 -- and the OSM importer has been writing them
    // for every mapped building since P0.3.
    //
    // Casting the part ordinal put the facade parts straight on top of that:
    // Glass is part 3, so a window's glazing asked for kWallConcrete; Frame is
    // part 2, so a frame asked for kWallStone; Sill is part 4, so a sill asked
    // for kWallRender. In ShaderMode::Simple nothing is bound and it never
    // showed, which is why it survived E3's whole suite.
    //
    // So each part maps to the variant that actually describes it. The mapping
    // is injective, which matters beyond correctness: the tests use this
    // function to find a part's faces, and two parts sharing a key would make
    // them indistinguishable.
    //
    // kWallBrick is deliberately left unused here. Brick is a decision about a
    // whole building, not about a window part, and leaving it free is what lets
    // a rule say `material("wall", 1)` and mean it.
    switch (part) {
        case FacadePart::Panel:     return MaterialKey{MaterialId::Wall, osm::road::variants::kWallDefault};
        case FacadePart::Reveal:    return MaterialKey{MaterialId::Wall, osm::road::variants::kWallRender};
        case FacadePart::Frame:     return MaterialKey{MaterialId::Wall, osm::road::variants::kWallMetal};
        case FacadePart::Glass:     return MaterialKey{MaterialId::Wall, osm::road::variants::kWallGlass};
        case FacadePart::Sill:      return MaterialKey{MaterialId::Wall, osm::road::variants::kWallStone};
        case FacadePart::Leaf:      return MaterialKey{MaterialId::Wall, osm::road::variants::kWallWood};
        case FacadePart::Threshold: return MaterialKey{MaterialId::Wall, osm::road::variants::kWallConcrete};
    }
    return MaterialKey{MaterialId::Wall, osm::road::variants::kWallDefault};
}

const char* facade_part_name(FacadePart part) {
    switch (part) {
        case FacadePart::Panel: return "panel";
        case FacadePart::Reveal: return "reveal";
        case FacadePart::Frame: return "frame";
        case FacadePart::Glass: return "glass";
        case FacadePart::Sill: return "sill";
        case FacadePart::Leaf: return "leaf";
        case FacadePart::Threshold: return "threshold";
    }
    return "unknown";
}

// ============================================================================
// The panel frame
// ============================================================================

bool orient_panel(Shape& shape, std::string& why) {
    size_t index = 0;
    if (!find_panel_face(shape, index, why)) {
        return false;
    }

    const glm::dvec3 normal = face_normal(shape.geometry, shape.geometry.faces[index]);
    if (glm::dot(normal, normal) < 0.5) {
        // face_normal() returns a unit vector or exactly zero, so anything short
        // of a half is the degenerate answer.
        why = "cannot take a frame from this face: it has no normal";
        return false;
    }

    // Everything but the panel is dropped here. find_panel_face() has already
    // established that there is nothing else with area, so what goes is
    // degenerate faces, and keeping those would put zero-area triangles in the
    // middle of a facade.
    shape.geometry = extract_face(shape.geometry, shape.geometry.faces[index]);

    // op_comp.hpp's frame, not a second copy of it: z is the face normal, y is
    // the in-plane up, x is across. A `select face` component is ALREADY in this
    // frame, so the comparison below is true for every tile a facade rule
    // produces and reframe() -- a transpose and a multiply per vertex -- never
    // runs on the common path. That is not a speed argument: it keeps the
    // vertices of an ordinary facade exact.
    const glm::dmat3 basis = face_component_axes(normal);
    if (basis != glm::dmat3{1.0}) {
        reframe(shape, shape.scope.axes * basis);
    } else {
        refit_scope(shape);
    }

    const double span = std::max({1.0, shape.scope.size.x, shape.scope.size.y});
    if (shape.scope.size.z > kPanelFlatnessTolerance * span) {
        why = "needs a flat panel, and this face is " + format_number(shape.scope.size.z) +
              " out of plane";
        return false;
    }
    return true;
}

// ============================================================================
// The operations
// ============================================================================

OpResult window_shape(Shape& shape, const OpeningParams& params, FacadeReport* report) {
    return build_facade_opening(shape, params, false, report);
}

OpResult door_shape(Shape& shape, const OpeningParams& params, FacadeReport* report) {
    return build_facade_opening(shape, params, true, report);
}

OpResult wall_panel_shape(Shape& shape, double inset, double thickness, FacadeReport* report) {
    FacadeReport scratch;
    FacadeReport& out = report != nullptr ? *report : scratch;
    out = FacadeReport{};

    std::string why;
    if (!orient_panel(shape, why)) {
        return OpResult::failure(why);
    }

    if (inset > kFacadeEpsilon) {
        // offset_shape() rather than four strips: a panel need not be a
        // rectangle, because nothing is being cut out of it.
        const OpResult offset = offset_shape(shape, -inset, OffsetSelector::Inside);
        if (!offset.ok) {
            // A WARNING and the un-inset panel, not a failure. An inset wide
            // enough to consume a 0.4 m pier is the same author mistake as an
            // inset wide enough to close a window, arriving through the same
            // attribute; window() answers it with a blank panel and a warning,
            // and the two disagreeing meant `wall_panel(0.5)` deleted its
            // terminal and failed the whole generation while `window(0.5)` on
            // the tile beside it survived. See the header note.
            //
            // offset_shape() builds into a scratch geometry and only moves it on
            // success, so `shape` here is exactly the panel orient_panel() left,
            // and carrying on with it needs no copy and no undo.
            out.warnings.push_back("cannot inset this panel by " + format_number(inset) + " (" +
                                   offset.message + "), so the panel was left whole");
            out.inset_dropped = true;
        }
    }

    out.panel_width = shape.scope.size.x;
    out.panel_height = shape.scope.size.y;

    // Recorded BEFORE the thickness, and recorded at all, because this is the
    // one number D8 cannot recover: once a sill or a thickness has grown the
    // scope, the panel's own extent is gone.
    shape.attributes[kPanelWidthAttribute] = Value::number(out.panel_width);
    shape.attributes[kPanelHeightAttribute] = Value::number(out.panel_height);

    if (thickness > kFacadeEpsilon) {
        // Along -z: INTO the wall. Out of it would put the panel in front of the
        // windows in the tiles beside it.
        const OpResult extruded = extrude_shape(shape, ScopeAxis::Z, -thickness);
        if (!extruded.ok) {
            return OpResult::failure("cannot give this panel a thickness of " +
                                     format_number(thickness) + ": " + extruded.message);
        }
    }

    for (Face& face : shape.geometry.faces) {
        face.material = facade_material(FacadePart::Panel);
    }
    refit_scope(shape);
    return OpResult::success();
}

// ============================================================================
// Registration
// ============================================================================

void register_facade_operations(OperationTable& table) {
    table.register_operation("door", op_door);
    table.register_operation("wall_panel", op_wall_panel);
    table.register_operation("window", op_window);
}

const OperationTable& facade_operations() {
    static const OperationTable table = build_facade_operations();
    return table;
}

} // namespace stratum::procgen::rules
