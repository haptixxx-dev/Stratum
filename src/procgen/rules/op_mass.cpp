// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_mass.cpp
 * @brief Mass modelling (E2): the footprint reader, the stacked prism, and the three handlers
 *
 * The design, the preconditions and every rejected alternative are in
 * op_mass.hpp. What is here is the arithmetic, and the two places it is easy to
 * get wrong are the ring winding -- which follows shape.cpp's build_prism()
 * exactly, because a mass built to a different convention meets an extruded one
 * in the same scene -- and the tier boundaries, which are absolute and never
 * accumulated.
 */

#include "procgen/rules/op_mass.hpp"

#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace stratum::procgen::rules {

namespace {

// ============================================================================
// Ring helpers
// ============================================================================

/// Reverse a copy of a ring of indices
[[nodiscard]] std::vector<uint32_t> reversed_ring(std::vector<uint32_t> ring) {
    std::reverse(ring.begin(), ring.end());
    return ring;
}

/// The ring wound so its face normal is +y
[[nodiscard]] std::vector<glm::dvec2> as_outer(std::vector<glm::dvec2> ring) {
    if (ring_newell_y(ring) < 0.0) {
        std::reverse(ring.begin(), ring.end());
    }
    return ring;
}

/// The ring wound against an outer ring, which is Face's contract for a hole
[[nodiscard]] std::vector<glm::dvec2> as_hole(std::vector<glm::dvec2> ring) {
    if (ring_newell_y(ring) > 0.0) {
        std::reverse(ring.begin(), ring.end());
    }
    return ring;
}

/**
 * @brief Append a ring at a given height, returning its new vertex indices
 *
 * The local counterpart of shape.cpp's add_ring(), which is in an anonymous
 * namespace there. Vertices are duplicated per ring rather than shared between
 * the tiers that meet at a boundary, exactly as build_prism() duplicates them:
 * shape_to_mesh() duplicates per face anyway for flat shading, and a shared
 * vertex would tie two tiers' geometry together for no gain.
 */
[[nodiscard]] std::vector<uint32_t> append_ring(ShapeGeometry& geometry,
                                                const std::vector<glm::dvec2>& ring,
                                                double y) {
    std::vector<uint32_t> indices;
    indices.reserve(ring.size());
    for (const glm::dvec2& p : ring) {
        indices.push_back(static_cast<uint32_t>(geometry.positions.size()));
        geometry.positions.push_back(glm::dvec3{p.x, y, p.y});
    }
    return indices;
}

/**
 * @brief One side quad per boundary edge
 *
 * `(low i, low j, high j, high i)`, which is build_prism()'s formula and which
 * works for both windings: an outer ring wound for a +y normal gives an outward
 * wall, and a hole ring wound against it gives a wall facing into the shaft,
 * which is also outward for the solid. One formula, both cases, because the
 * winding of the rings already carries the difference.
 */
void append_walls(ShapeGeometry& geometry,
                  const std::vector<uint32_t>& low,
                  const std::vector<uint32_t>& high,
                  MaterialKey material) {
    for (size_t i = 0; i < low.size(); ++i) {
        const size_t j = (i + 1) % low.size();
        Face wall;
        wall.material = material;
        wall.loop = {low[i], low[j], high[j], high[i]};
        geometry.faces.push_back(std::move(wall));
    }
}

/// Is @p p on the segment ab, to within kMassLengthEpsilon?
[[nodiscard]] bool point_on_segment(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 ab = b - a;
    const glm::dvec2 ap = p - a;
    const double len2 = ab.x * ab.x + ab.y * ab.y;
    if (len2 <= kMassLengthEpsilon * kMassLengthEpsilon) {
        return std::fabs(ap.x) <= kMassLengthEpsilon && std::fabs(ap.y) <= kMassLengthEpsilon;
    }
    // The cross product is an AREA, so the perpendicular distance test has to be
    // scaled by the segment length to be a DISTANCE. Comparing the raw cross
    // product against a length tolerance compares distance * length against it
    // instead, which makes a SHORT edge far more forgiving than a long one: a
    // point a micrometre off a millimetre edge counts as lying on it, while a
    // point a nanometre off a kilometre edge does not. A footprint has both --
    // a road frontage in the hundreds of metres and a chamfer in the
    // millimetres -- so the scaling is the difference between one tolerance and
    // a tolerance that varies by nine orders of magnitude across one outline.
    const double area = ab.x * ap.y - ab.y * ap.x;
    if (std::fabs(area) > kMassLengthEpsilon * std::sqrt(len2)) {
        return false;
    }
    const double t = (ab.x * ap.x + ab.y * ap.y) / len2;
    return t >= -kMassLengthEpsilon && t <= 1.0 + kMassLengthEpsilon;
}

/// Crossing-number test. A point ON the boundary is NOT strictly inside.
[[nodiscard]] bool point_strictly_inside(const std::vector<glm::dvec2>& ring,
                                         const glm::dvec2& p) {
    if (ring.size() < 3) {
        return false;
    }
    bool inside = false;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        if (point_on_segment(p, ring[j], ring[i])) {
            return false;
        }
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[j];
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x;
            if (p.x < x) {
                inside = !inside;
            }
        }
    }
    return inside;
}

/// Sign of the cross product (b - a) x (c - a)
[[nodiscard]] double orientation(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/// Do ab and cd cross at a point interior to both? Touching does not count.
[[nodiscard]] bool segments_properly_cross(const glm::dvec2& a,
                                           const glm::dvec2& b,
                                           const glm::dvec2& c,
                                           const glm::dvec2& d) {
    const double d1 = orientation(c, d, a);
    const double d2 = orientation(c, d, b);
    const double d3 = orientation(a, b, c);
    const double d4 = orientation(a, b, d);
    return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
           ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
}

// ============================================================================
// Argument helpers for the handlers
//
// Copies of interpreter.cpp's, which are in an anonymous namespace there. The
// wording is copied with them: an author who mistypes an argument to `podium`
// should read the same sentence they would read for `scale`.
// ============================================================================

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

[[nodiscard]] bool op_bool(OperationArgs& context, size_t index, bool& out) {
    if (index >= context.args.size() || !context.args[index].is_bool()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a boolean for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_bool();
    return true;
}

/// Turn an OpResult failure into a reported fault that abandons the shape
void check(OperationArgs& context, const OpResult& result) {
    if (!result.ok) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "': " + result.message);
    }
}

} // namespace

// ============================================================================
// Ring arithmetic
// ============================================================================

double ring_newell_y(const std::vector<glm::dvec2>& ring) {
    if (ring.size() < 3) {
        return 0.0;
    }
    // The y term of Newell's method for the ring lifted into the xz plane, with
    // (ring.x, ring.y) read as (x, z). Computed here rather than as a shoelace
    // so that the sign convention is literally the one face_normal() uses and
    // the two cannot drift apart.
    double total = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % ring.size()];
        total += (a.y - b.y) * (a.x + b.x);
    }
    return total;
}

double ring_area(const std::vector<glm::dvec2>& ring) {
    return std::fabs(ring_newell_y(ring)) * 0.5;
}

bool ring_strictly_contains(const std::vector<glm::dvec2>& outer,
                            const std::vector<glm::dvec2>& inner) {
    if (outer.size() < 3 || inner.size() < 3) {
        return false;
    }
    for (const glm::dvec2& p : inner) {
        if (!point_strictly_inside(outer, p)) {
            return false;
        }
    }
    // Test 1 passes a ring whose vertices are all inside but whose edges bulge
    // out between them, which is exactly the shape an inset produces at a reflex
    // corner when the miter limit squares it off.
    for (size_t i = 0; i < inner.size(); ++i) {
        const glm::dvec2& a = inner[i];
        const glm::dvec2& b = inner[(i + 1) % inner.size()];
        for (size_t j = 0; j < outer.size(); ++j) {
            const glm::dvec2& c = outer[j];
            const glm::dvec2& d = outer[(j + 1) % outer.size()];
            if (segments_properly_cross(a, b, c, d)) {
                return false;
            }
        }
    }
    // And tests 1 and 2 together pass the two rings the wrong way round: every
    // vertex of a large ring can be inside a small one only if they cross, but
    // two rings that are the same ring do not cross and every vertex of each
    // lies on the other -- which test 1 already refuses -- while two NESTED
    // rings handed over swapped do not.
    for (const glm::dvec2& p : outer) {
        if (point_strictly_inside(inner, p)) {
            return false;
        }
    }
    return true;
}

OpResult inset_ring(const std::vector<glm::dvec2>& ring,
                    double distance,
                    std::vector<std::vector<glm::dvec2>>& out) {
    out.clear();
    if (ring.size() < 3) {
        return OpResult::failure("the outline has fewer than three points");
    }
    if (!(distance > kMassLengthEpsilon)) {
        return OpResult::failure("the inset distance must be positive");
    }

    Shape scratch = shape_from_polygon(ring, 0.0);
    if (scratch.geometry.faces.empty()) {
        return OpResult::failure("the outline encloses no area");
    }

    const OpResult offset = offset_shape(scratch, -distance, OffsetSelector::Inside);
    if (!offset.ok) {
        return offset;
    }

    // to_world() is the invariant across the refit offset_shape() performs, so
    // reading the result back through it gives coordinates in the frame the ring
    // arrived in. Reading `positions` directly would be off by however far the
    // inset moved the box minimum, which is a translation of the whole tower.
    for (const Face& face : scratch.geometry.faces) {
        std::vector<glm::dvec2> result;
        result.reserve(face.loop.size());
        for (const uint32_t index : face.loop) {
            const glm::dvec3 world = scratch.scope.to_world(scratch.geometry.positions[index]);
            result.push_back(glm::dvec2{world.x, world.z});
        }
        // The area filter is defensive, and its reach is worth being honest
        // about: paths_to_faces() already drops a path of fewer than three
        // points, and it treats a zero-area ring as an OUTER (its hole test is
        // `area < 0`), so a degenerate ring that got that far would arrive here
        // as a courtyard or a tier outline enclosing nothing. Clipper2's
        // Union(NonZero) is not known to produce one -- a search over dumbbells
        // insetting through their neck, bars insetting through half their width,
        // spikes against the miter limit, and triangles and squares collapsing
        // to a point found no input that reaches this branch. It stays because
        // external/Clipper2 is a submodule that will be upgraded, and because a
        // zero-area outline is the one failure that costs nothing to refuse
        // here and builds a wall of no width if it is not.
        if (result.size() >= 3 && ring_area(result) > kMassAreaEpsilon) {
            out.push_back(std::move(result));
        }
    }

    if (out.empty()) {
        return OpResult::failure("the inset removed the whole outline");
    }
    return OpResult::success();
}

// ============================================================================
// Footprint
// ============================================================================

bool shape_is_footprint(const Shape& shape) {
    // A shape with NO geometry is not a footprint however flat its scope says it
    // is: shape.hpp exempts it from the tight-box invariant, so its size.y is
    // whatever somebody set and is not a measurement of anything.
    if (shape.geometry.faces.empty() || shape.geometry.positions.empty()) {
        return false;
    }
    return shape.scope.size.y <= kMassFlatTolerance;
}

OpResult read_footprint(const Shape& shape, std::vector<FootprintFace>& out) {
    out.clear();
    if (shape.geometry.faces.empty() || shape.geometry.positions.empty()) {
        return OpResult::failure("the shape has no geometry to build a mass on");
    }
    if (shape.scope.size.y > kMassFlatTolerance) {
        return OpResult::failure(
            "the shape already has a height of " + format_number(shape.scope.size.y) +
            ", and a mass operation builds on a flat footprint");
    }

    for (const Face& face : shape.geometry.faces) {
        if (face.loop.size() < 3) {
            continue;
        }
        FootprintFace entry;
        entry.material = face.material;
        // Every vertex is within kMassFlatTolerance of one plane, so the first
        // answers for all of them -- the same vertex face_basis() takes its
        // origin from.
        entry.plane_y = shape.geometry.positions[face.loop[0]].y;

        std::vector<glm::dvec2> outer;
        outer.reserve(face.loop.size());
        for (const uint32_t index : face.loop) {
            const glm::dvec3& p = shape.geometry.positions[index];
            outer.push_back(glm::dvec2{p.x, p.z});
        }
        if (ring_area(outer) <= kMassAreaEpsilon) {
            continue;  // a degenerate face makes no mass, exactly as it extrudes to none
        }
        entry.outer = as_outer(std::move(outer));

        for (const std::vector<uint32_t>& hole : face.holes) {
            if (hole.size() < 3) {
                continue;
            }
            std::vector<glm::dvec2> ring;
            ring.reserve(hole.size());
            for (const uint32_t index : hole) {
                const glm::dvec3& p = shape.geometry.positions[index];
                ring.push_back(glm::dvec2{p.x, p.z});
            }
            if (ring_area(ring) <= kMassAreaEpsilon) {
                continue;
            }
            entry.holes.push_back(as_hole(std::move(ring)));
        }

        out.push_back(std::move(entry));
    }

    if (out.empty()) {
        return OpResult::failure("no face of the shape encloses any area");
    }
    return OpResult::success();
}

// ============================================================================
// The stacked prism
// ============================================================================

OpResult build_mass_stack(const std::vector<MassTier>& tiers,
                          const std::vector<std::vector<glm::dvec2>>& holes,
                          double base,
                          MaterialKey material,
                          ShapeGeometry& out) {
    if (tiers.empty()) {
        return OpResult::failure("the mass has no tiers");
    }

    std::vector<std::vector<glm::dvec2>> outlines;
    outlines.reserve(tiers.size());
    double previous_top = base;
    for (size_t k = 0; k < tiers.size(); ++k) {
        const std::string where = "tier " + std::to_string(k);
        if (tiers[k].outline.size() < 3) {
            return OpResult::failure("the outline at " + where + " has fewer than three points");
        }
        if (ring_area(tiers[k].outline) <= kMassAreaEpsilon) {
            return OpResult::failure("the outline at " + where + " encloses no area");
        }
        if (!(tiers[k].top > previous_top + kMassLengthEpsilon)) {
            return OpResult::failure("the top of " + where + " at " +
                                     format_number(tiers[k].top) + " is not above the " +
                                     format_number(previous_top) + " below it");
        }
        previous_top = tiers[k].top;
        outlines.push_back(as_outer(tiers[k].outline));
    }

    std::vector<std::vector<glm::dvec2>> shafts;
    for (const std::vector<glm::dvec2>& hole : holes) {
        if (hole.size() < 3 || ring_area(hole) <= kMassAreaEpsilon) {
            continue;
        }
        shafts.push_back(as_hole(hole));
    }

    // The condition the whole single-face terrace rests on. See op_mass.hpp:
    // with the shafts inside every outline, the horizontal face between tier k
    // and tier k+1 is exactly O_k minus O_(k+1), one loop and one hole.
    for (size_t k = 0; k < outlines.size(); ++k) {
        for (const std::vector<glm::dvec2>& shaft : shafts) {
            if (!ring_strictly_contains(outlines[k], shaft)) {
                return OpResult::failure("the outline at tier " + std::to_string(k) +
                                         " does not contain the courtyard");
            }
        }
    }

    std::vector<std::vector<uint32_t>> low(outlines.size());
    std::vector<std::vector<uint32_t>> high(outlines.size());
    for (size_t k = 0; k < outlines.size(); ++k) {
        const double bottom_y = (k == 0) ? base : tiers[k - 1].top;
        low[k] = append_ring(out, outlines[k], bottom_y);
        high[k] = append_ring(out, outlines[k], tiers[k].top);
        append_walls(out, low[k], high[k], material);
    }

    // One wall per shaft edge for the WHOLE stack rather than one per tier: a
    // light well does not step, so cutting it at every terrace would hand a
    // facade rule seams the building does not have.
    std::vector<std::vector<uint32_t>> shaft_low;
    std::vector<std::vector<uint32_t>> shaft_high;
    for (const std::vector<glm::dvec2>& shaft : shafts) {
        shaft_low.push_back(append_ring(out, shaft, base));
        shaft_high.push_back(append_ring(out, shaft, tiers.back().top));
        append_walls(out, shaft_low.back(), shaft_high.back(), material);
    }

    // The underside: the lowest outline reversed, so its normal points down.
    Face bottom;
    bottom.material = material;
    bottom.loop = reversed_ring(low.front());
    for (const std::vector<uint32_t>& shaft : shaft_low) {
        bottom.holes.push_back(reversed_ring(shaft));
    }
    out.faces.push_back(std::move(bottom));

    // The roof: the highest outline as it stands, with the shafts open.
    Face top;
    top.material = material;
    top.loop = high.back();
    for (const std::vector<uint32_t>& shaft : shaft_high) {
        top.holes.push_back(shaft);
    }
    out.faces.push_back(std::move(top));

    // A terrace per step-back: the lower outline with the upper one as its one
    // hole. The shafts are NOT holes here -- they are inside the upper outline,
    // so the hole already covers them, and listing them again would subtract
    // them twice from the face's area.
    for (size_t k = 0; k + 1 < outlines.size(); ++k) {
        Face terrace;
        terrace.material = material;
        terrace.loop = high[k];
        terrace.holes.push_back(reversed_ring(low[k + 1]));
        out.faces.push_back(std::move(terrace));
    }

    return OpResult::success();
}

// ============================================================================
// floors
// ============================================================================

double FloorPlan::total_height() const {
    if (count == 0) {
        return 0.0;
    }
    return ground_height + static_cast<double>(count - 1) * upper_height;
}

OpResult solve_floor_plan(double count, double height, double ground_height, FloorPlan& out) {
    if (!std::isfinite(count)) {
        return OpResult::failure("the floor count is not a number");
    }
    // The language has no integer type, so a count is rounded -- away from zero
    // on a tie, which is what std::llround does and what an author reading
    // "2.5 floors" expects to become three.
    const double rounded = std::round(count);
    if (rounded < 1.0) {
        return OpResult::failure("the floor count must be at least 1, not " +
                                 format_number(rounded));
    }
    if (rounded > static_cast<double>(kMaxFloors)) {
        return OpResult::failure("the floor count of " + format_number(rounded) +
                                 " is above the limit of " + std::to_string(kMaxFloors) +
                                 "; check the units of the value that produced it");
    }
    if (!(height > kMassLengthEpsilon)) {
        return OpResult::failure("the floor height must be positive, not " +
                                 format_number(height));
    }
    if (!(ground_height > kMassLengthEpsilon)) {
        return OpResult::failure("the ground floor height must be positive, not " +
                                 format_number(ground_height));
    }

    out.count = static_cast<uint32_t>(rounded);
    out.upper_height = height;
    out.ground_height = ground_height;
    return OpResult::success();
}

OpResult floors_shape(Shape& shape, const FloorPlan& plan) {
    if (plan.count < 1) {
        return OpResult::failure("the floor plan has no floors in it");
    }
    if (!shape_is_footprint(shape)) {
        std::vector<FootprintFace> unused;
        return read_footprint(shape, unused);  // for the sentence it writes
    }

    const OpResult extruded = extrude_shape(shape, ScopeAxis::Y, plan.total_height());
    if (!extruded.ok) {
        return extruded;
    }

    // The structure, so that a facade rule can cut exactly these floors without
    // the storey height being written out a second time. See op_mass.hpp.
    shape.attributes[std::string{kFloorCountAttribute}] =
        Value::number(static_cast<double>(plan.count));
    shape.attributes[std::string{kFloorHeightAttribute}] = Value::number(plan.upper_height);
    shape.attributes[std::string{kGroundHeightAttribute}] = Value::number(plan.ground_height);
    return OpResult::success();
}

// ============================================================================
// courtyard
// ============================================================================

OpResult courtyard_shape(Shape& shape,
                         const CourtyardOptions& options,
                         CourtyardReport* report) {
    if (report != nullptr) {
        *report = CourtyardReport{};
    }
    if (!(options.depth > kMassLengthEpsilon)) {
        return OpResult::failure("the courtyard depth must be positive, not " +
                                 format_number(options.depth));
    }

    std::vector<FootprintFace> faces;
    const OpResult read = read_footprint(shape, faces);
    if (!read.ok) {
        return read;
    }

    ShapeGeometry out;
    for (const FootprintFace& entry : faces) {
        std::vector<std::vector<glm::dvec2>> rings;

        if (options.keep_existing && !entry.holes.empty()) {
            // The second half of the catalogue row. An OSM building that already
            // has an inner ring already has its courtyard; carving a second one
            // inside the ring the first left is not what `courtyard(6)` asked.
            rings = entry.holes;
            if (report != nullptr) {
                ++report->kept;
            }
        } else {
            std::vector<std::vector<glm::dvec2>> carved;
            const OpResult inset = inset_ring(entry.outer, options.depth, carved);
            double area = 0.0;
            for (const std::vector<glm::dvec2>& ring : carved) {
                area += ring_area(ring);
            }
            if (!inset.ok || carved.empty()) {
                if (report != nullptr) {
                    ++report->no_room;
                }
            } else if (area <= kMassAreaEpsilon || area < options.min_area) {
                if (report != nullptr) {
                    ++report->too_small;
                }
            } else {
                for (std::vector<glm::dvec2>& ring : carved) {
                    rings.push_back(as_hole(std::move(ring)));
                }
                if (report != nullptr) {
                    ++report->carved;
                    report->area += area;
                }
            }
        }

        Face face;
        face.material = entry.material;
        face.loop = append_ring(out, entry.outer, entry.plane_y);
        for (const std::vector<glm::dvec2>& ring : rings) {
            face.holes.push_back(append_ring(out, ring, entry.plane_y));
        }
        out.faces.push_back(std::move(face));
    }

    shape.geometry = std::move(out);
    // The outline did not move, so the box does not either -- but the call is
    // made rather than reasoned about, because "this operation happens not to
    // change the bounds" is exactly the assumption that stops being true.
    refit_scope(shape);
    return OpResult::success();
}

// ============================================================================
// podium
// ============================================================================

double PodiumPlan::step_height() const {
    if (steps == 0) {
        return 0.0;
    }
    return tower_height / static_cast<double>(steps);
}

OpResult solve_podium_plan(double base_height,
                           double tower_height,
                           double inset,
                           double steps,
                           PodiumPlan& out) {
    if (!(base_height > kMassLengthEpsilon)) {
        return OpResult::failure("the podium height must be positive, not " +
                                 format_number(base_height));
    }
    if (!(tower_height > kMassLengthEpsilon)) {
        return OpResult::failure("the tower height must be positive, not " +
                                 format_number(tower_height));
    }
    if (!(inset > kMassLengthEpsilon)) {
        // A podium with no inset is an extrude, and the terrace face it would
        // build has a hole exactly the size of its own outline. `setback` is the
        // planar operation; this one is vertical and needs a step to take.
        return OpResult::failure("the podium inset must be positive, not " +
                                 format_number(inset) +
                                 "; a podium with no inset is an extrude");
    }
    if (!std::isfinite(steps)) {
        return OpResult::failure("the podium step count is not a number");
    }
    const double rounded = std::round(steps);
    if (rounded < 1.0) {
        return OpResult::failure("the podium step count must be at least 1, not " +
                                 format_number(rounded));
    }
    if (rounded > static_cast<double>(kMaxPodiumSteps)) {
        return OpResult::failure("the podium step count of " + format_number(rounded) +
                                 " is above the limit of " + std::to_string(kMaxPodiumSteps));
    }

    out.base_height = base_height;
    out.tower_height = tower_height;
    out.inset = inset;
    out.steps = static_cast<uint32_t>(rounded);
    return OpResult::success();
}

OpResult podium_shape(Shape& shape, const PodiumPlan& plan) {
    PodiumPlan checked{};
    const OpResult valid = solve_podium_plan(plan.base_height, plan.tower_height, plan.inset,
                                             static_cast<double>(plan.steps), checked);
    if (!valid.ok) {
        return valid;
    }

    std::vector<FootprintFace> faces;
    const OpResult read = read_footprint(shape, faces);
    if (!read.ok) {
        return read;
    }

    ShapeGeometry out;
    for (const FootprintFace& entry : faces) {
        std::vector<MassTier> tiers;
        tiers.reserve(checked.steps + 1);

        MassTier base_tier;
        base_tier.outline = entry.outer;
        base_tier.top = entry.plane_y + checked.base_height;
        tiers.push_back(std::move(base_tier));

        for (uint32_t step = 1; step <= checked.steps; ++step) {
            std::vector<std::vector<glm::dvec2>> inset;
            const OpResult cut = inset_ring(tiers.back().outline, checked.inset, inset);
            if (!cut.ok) {
                return OpResult::failure("the inset at tier " + std::to_string(step) + " of " +
                                         format_number(checked.inset) + " removes the whole outline");
            }
            if (inset.size() != 1) {
                // Which of the pieces carries on upward is a question the rule
                // did not answer, and picking the largest would make a one-
                // centimetre change to the footprint move the tower.
                return OpResult::failure("the inset at tier " + std::to_string(step) +
                                         " splits the outline into " +
                                         std::to_string(inset.size()) + " parts");
            }

            // ABSOLUTE, not accumulated: the last tier's top is the podium plus
            // the whole tower with no rounding in between, so scope.size.y after
            // the refit is exactly base_height + tower_height and the facade
            // split divides a number that was built by multiplying.
            const double risen = (step == checked.steps)
                                     ? checked.tower_height
                                     : checked.tower_height * static_cast<double>(step) /
                                           static_cast<double>(checked.steps);

            MassTier tier;
            tier.outline = std::move(inset.front());
            tier.top = entry.plane_y + checked.base_height + risen;
            tiers.push_back(std::move(tier));
        }

        const OpResult built =
            build_mass_stack(tiers, entry.holes, entry.plane_y, entry.material, out);
        if (!built.ok) {
            return built;
        }
    }

    shape.geometry = std::move(out);
    refit_scope(shape);

    shape.attributes[std::string{kPodiumHeightAttribute}] = Value::number(checked.base_height);
    shape.attributes[std::string{kTowerHeightAttribute}] = Value::number(checked.tower_height);
    shape.attributes[std::string{kPodiumInsetAttribute}] = Value::number(checked.inset);
    shape.attributes[std::string{kPodiumStepsAttribute}] =
        Value::number(static_cast<double>(checked.steps));
    return OpResult::success();
}

// ============================================================================
// Operation handlers
// ============================================================================

namespace {

void op_courtyard(OperationArgs& context) {
    CourtyardOptions options;
    if (!op_number(context, 0, options.depth)) {
        return;
    }
    if (context.args.size() >= 2 && !op_number(context, 1, options.min_area)) {
        return;
    }
    if (context.args.size() >= 3 && !op_bool(context, 2, options.keep_existing)) {
        return;
    }

    CourtyardReport report;
    const OpResult result = courtyard_shape(context.shape, options, &report);
    if (!result.ok) {
        check(context, result);
        return;
    }

    // Permissive, but never silent. A Warning leaves GenerationResult::ok() true,
    // so a city rule that meets three hundred narrow lots still finishes, and
    // report_once() keeps it to one line per site rather than one per lot.
    if (report.no_room > 0) {
        context.interpreter.report_once(
            Severity::Warning, context.loc,
            "'courtyard': the footprint is narrower than twice the depth of " +
                format_number(options.depth) + ", so it is left solid");
    }
    if (report.too_small > 0) {
        context.interpreter.report_once(
            Severity::Warning, context.loc,
            "'courtyard': the courtyard would be smaller than the minimum area of " +
                format_number(options.min_area) + ", so the footprint is left solid");
    }
}

void op_floors(OperationArgs& context) {
    double count = 0.0;
    if (!op_number(context, 0, count)) {
        return;
    }
    double height = kDefaultFloorHeight;
    if (context.args.size() >= 2 && !op_number(context, 1, height)) {
        return;
    }
    // The ground storey follows the storey height unless the rule gives it one,
    // so `floors(6, 3.5)` is six equal storeys and not five plus a three-metre
    // shop nobody asked for.
    double ground = height;
    if (context.args.size() >= 3 && !op_number(context, 2, ground)) {
        return;
    }

    FloorPlan plan;
    const OpResult solved = solve_floor_plan(count, height, ground, plan);
    if (!solved.ok) {
        check(context, solved);
        return;
    }
    check(context, floors_shape(context.shape, plan));
}

void op_podium(OperationArgs& context) {
    double base_height = 0.0;
    double tower_height = 0.0;
    if (!op_number(context, 0, base_height) || !op_number(context, 1, tower_height)) {
        return;
    }
    double inset = kDefaultPodiumInset;
    if (context.args.size() >= 3 && !op_number(context, 2, inset)) {
        return;
    }
    double steps = 1.0;
    if (context.args.size() >= 4 && !op_number(context, 3, steps)) {
        return;
    }

    PodiumPlan plan;
    const OpResult solved =
        solve_podium_plan(base_height, tower_height, inset, steps, plan);
    if (!solved.ok) {
        check(context, solved);
        return;
    }
    check(context, podium_shape(context.shape, plan));
}

[[nodiscard]] OperationTable build_mass_operations() {
    OperationTable table = standard_operations();
    register_mass_operations(table);
    return table;
}

} // namespace

void register_mass_operations(OperationTable& table) {
    table.register_operation("courtyard", op_courtyard);
    table.register_operation("floors", op_floors);
    table.register_operation("podium", op_podium);
}

const OperationTable& mass_operations() {
    static const OperationTable table = build_mass_operations();
    return table;
}

} // namespace stratum::procgen::rules
