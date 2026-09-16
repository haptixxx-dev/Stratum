// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_comp.cpp
 * @brief Component split: classification, canonical ordering, oriented scopes
 *
 * The design and every decision behind it are in op_comp.hpp. What is here is
 * the arithmetic, and three notes that only matter to someone reading the code:
 *
 *   - Nothing in this file calls an inverse trigonometric function. Angles are
 *     compared as cosines, through shape.hpp's deg_sin_cos(), which is exact on
 *     the quarter turns. An acos() on the wrong side of a boolean is a
 *     classification that can differ between two C libraries, and therefore a
 *     building that differs between two machines.
 *   - The ordering key is quantised to integers rather than compared with an
 *     epsilon, because epsilon-equality is not transitive and std::sort with a
 *     non-transitive comparator is undefined behaviour.
 *   - Component shapes are built by handing the face's vertices to reframe(),
 *     which is shape.hpp's one sanctioned way to change a frame under existing
 *     geometry. Writing Scope::axes here and refitting by hand would duplicate
 *     the one function that has to stay right.
 */

#include "procgen/rules/op_comp.hpp"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace stratum::procgen::rules {

namespace {

// ============================================================================
// Small vector arithmetic
//
// shape.cpp has these in an anonymous namespace of its own and they are not
// exported, so they are repeated here rather than widening shape.hpp's surface
// for two three-line functions.
// ============================================================================

/// Below this a vector is treated as having no direction
constexpr double kDirectionEpsilon = 1e-12;

[[nodiscard]] double length_of(const glm::dvec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/// Unit vector, or exactly zero when there is no direction to speak of
[[nodiscard]] glm::dvec3 safe_normalize(const glm::dvec3& v) {
    const double length = length_of(v);
    if (length <= kDirectionEpsilon) {
        return glm::dvec3{0.0};
    }
    return v / length;
}

/// Is this the zero vector safe_normalize() hands back for a degenerate input?
[[nodiscard]] bool is_direction(const glm::dvec3& v) {
    return length_of(v) > 0.5;
}

[[nodiscard]] glm::dvec3 axis_vector(ScopeAxis axis) {
    switch (axis) {
        case ScopeAxis::X: return glm::dvec3{1.0, 0.0, 0.0};
        case ScopeAxis::Y: return glm::dvec3{0.0, 1.0, 0.0};
        case ScopeAxis::Z: return glm::dvec3{0.0, 0.0, 1.0};
    }
    return glm::dvec3{0.0, 1.0, 0.0};
}

// ============================================================================
// Canonical ordering
// ============================================================================

/**
 * @brief One coordinate as an integer on the ordering grid
 *
 * Clamped before the division so that a coordinate no sane shape has cannot
 * overflow the int64 and wrap the order round. A shape a thousand kilometres
 * across is already past everything else this engine assumes.
 */
[[nodiscard]] int64_t quantise(double value) {
    if (!(value > -1.0e9)) {
        return -1000000000000000LL;
    }
    if (!(value < 1.0e9)) {
        return 1000000000000000LL;
    }
    return static_cast<int64_t>(std::llround(value / kComponentOrderQuantum));
}

/**
 * @brief The sort key: position, then direction, then buffer position
 *
 * A TOTAL order, not merely a strict weak one, because @p tiebreak is unique per
 * component. That matters for more than std::sort's contract: it means two
 * components that are geometrically identical still get a defined order rather
 * than whichever one the sort happened to move.
 */
struct OrderKey {
    int64_t centre[3]{};     ///< centroid, in the compare order y, x, z
    int64_t normal[3]{};     ///< normal, same order
    int64_t direction[3]{};  ///< edge direction, same order; zero for a face
    uint64_t tiebreak = 0;   ///< position in the parent's buffers

    [[nodiscard]] bool operator<(const OrderKey& other) const {
        for (int i = 0; i < 3; ++i) {
            if (centre[i] != other.centre[i]) {
                return centre[i] < other.centre[i];
            }
        }
        for (int i = 0; i < 3; ++i) {
            if (normal[i] != other.normal[i]) {
                return normal[i] < other.normal[i];
            }
        }
        for (int i = 0; i < 3; ++i) {
            if (direction[i] != other.direction[i]) {
                return direction[i] < other.direction[i];
            }
        }
        return tiebreak < other.tiebreak;
    }
};

/// Fill one triple of the key. y first, so the order reads bottom-to-top.
void fill_key(int64_t (&slot)[3], const glm::dvec3& v) {
    slot[0] = quantise(v.y);
    slot[1] = quantise(v.x);
    slot[2] = quantise(v.z);
}

// ============================================================================
// Gathering
// ============================================================================

/// One component before it has been turned into a Shape
struct RawComponent {
    OrderKey key{};
    glm::dvec3 normal{0.0};     ///< face normal, in the parent's local frame
    glm::dvec3 direction{0.0};  ///< edge direction; zero for a face
    glm::dvec3 a{0.0};          ///< edge start
    glm::dvec3 b{0.0};          ///< edge end
    double measure = 0.0;
    double planarity = 0.0;
    uint32_t face = 0;  ///< index into the parent's faces, for a face component
};

/// Mean of a ring's vertices. The ordering anchor and the plane's point.
[[nodiscard]] glm::dvec3 ring_centroid(const ShapeGeometry& geometry,
                                       const std::vector<uint32_t>& ring) {
    glm::dvec3 total{0.0};
    for (const uint32_t index : ring) {
        total += geometry.positions[index];
    }
    return total / static_cast<double>(ring.size());
}

/**
 * @brief Worst distance from the face's vertices to the plane through its centroid
 *
 * The plane is anchored at the CENTROID and not at the first vertex, so the
 * answer does not change when the same ring is written starting from a different
 * corner -- which OSM-derived rings do all the time.
 */
[[nodiscard]] double face_planarity(const ShapeGeometry& geometry,
                                    const Face& face,
                                    const glm::dvec3& normal,
                                    const glm::dvec3& centroid) {
    double worst = 0.0;
    for (const uint32_t index : face.loop) {
        worst = std::max(worst, std::fabs(glm::dot(geometry.positions[index] - centroid, normal)));
    }
    for (const std::vector<uint32_t>& hole : face.holes) {
        for (const uint32_t index : hole) {
            worst =
                std::max(worst, std::fabs(glm::dot(geometry.positions[index] - centroid, normal)));
        }
    }
    return worst;
}

/// Every face of @p geometry that has a normal and an area, unsorted
[[nodiscard]] std::vector<RawComponent> gather_faces(const ShapeGeometry& geometry,
                                                     uint32_t& degenerate) {
    std::vector<RawComponent> out;
    out.reserve(geometry.faces.size());
    for (size_t i = 0; i < geometry.faces.size(); ++i) {
        const Face& face = geometry.faces[i];
        if (face.loop.size() < 3) {
            ++degenerate;
            continue;
        }
        const glm::dvec3 normal = face_normal(geometry, face);
        const double area = face_area(geometry, face);
        if (!is_direction(normal) || !(area > kComponentAreaEpsilon)) {
            // No normal means no direction to classify by and no plane to build
            // a frame on; no area means the same thing by a different route, and
            // a face whose holes ate it is the case that produces it.
            ++degenerate;
            continue;
        }

        RawComponent raw;
        const glm::dvec3 centroid = ring_centroid(geometry, face.loop);
        raw.normal = normal;
        raw.measure = area;
        raw.planarity = face_planarity(geometry, face, normal, centroid);
        raw.face = static_cast<uint32_t>(i);
        fill_key(raw.key.centre, centroid);
        fill_key(raw.key.normal, normal);
        raw.key.tiebreak = static_cast<uint64_t>(i) << 32;
        out.push_back(raw);
    }
    return out;
}

/// Append one ring's segments as edge components
void gather_ring_edges(const ShapeGeometry& geometry,
                       const std::vector<uint32_t>& ring,
                       const glm::dvec3& normal,
                       uint32_t face_index,
                       uint32_t& ordinal,
                       uint32_t& degenerate,
                       std::vector<RawComponent>& out) {
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec3 a = geometry.positions[ring[i]];
        const glm::dvec3 b = geometry.positions[ring[(i + 1) % ring.size()]];
        const glm::dvec3 span = b - a;
        const double length = length_of(span);
        const uint32_t here = ordinal++;
        if (!(length > kComponentEdgeEpsilon)) {
            // A repeated vertex. It has no direction, so it cannot orient a
            // scope, and a zero-length edge is not a thing a rule can decorate.
            ++degenerate;
            continue;
        }

        RawComponent raw;
        raw.normal = normal;
        raw.direction = span / length;
        raw.a = a;
        raw.b = b;
        raw.measure = length;
        raw.face = face_index;
        fill_key(raw.key.centre, (a + b) * 0.5);
        fill_key(raw.key.normal, normal);
        fill_key(raw.key.direction, raw.direction);
        raw.key.tiebreak = (static_cast<uint64_t>(face_index) << 32) | here;
        out.push_back(raw);
    }
}

/**
 * @brief Every boundary segment of every face, holes included
 *
 * Per face, so an edge shared by two faces appears twice. op_comp.hpp says why:
 * the two copies carry different frames, and picking one of them would mean
 * picking a face, which would put the buffer order back into the answer.
 */
[[nodiscard]] std::vector<RawComponent> gather_edges(const ShapeGeometry& geometry,
                                                     uint32_t& degenerate) {
    std::vector<RawComponent> out;
    for (size_t i = 0; i < geometry.faces.size(); ++i) {
        const Face& face = geometry.faces[i];
        if (face.loop.size() < 2) {
            ++degenerate;
            continue;
        }
        const glm::dvec3 normal = face_normal(geometry, face);
        if (!is_direction(normal)) {
            ++degenerate;
            continue;
        }
        uint32_t ordinal = 0;
        gather_ring_edges(geometry, face.loop, normal, static_cast<uint32_t>(i), ordinal,
                          degenerate, out);
        for (const std::vector<uint32_t>& hole : face.holes) {
            gather_ring_edges(geometry, hole, normal, static_cast<uint32_t>(i), ordinal, degenerate,
                              out);
        }
    }
    return out;
}

// ============================================================================
// Building the component shape
// ============================================================================

/// Copy one ring's vertices into @p out, returning the new indices
[[nodiscard]] std::vector<uint32_t> copy_ring(const ShapeGeometry& source,
                                              const std::vector<uint32_t>& ring,
                                              ShapeGeometry& out) {
    std::vector<uint32_t> remapped;
    remapped.reserve(ring.size());
    for (const uint32_t index : ring) {
        remapped.push_back(static_cast<uint32_t>(out.positions.size()));
        out.positions.push_back(source.positions[index]);
    }
    return remapped;
}

/**
 * @brief The face as a shape of its own, in a frame oriented to the face
 *
 * The parent's attributes and pivot FRACTION travel with it. A fraction of the
 * box means the same thing in any box -- which is the whole reason shape.hpp
 * stores the pivot that way -- so there is nothing to recompute.
 */
[[nodiscard]] Shape build_face_shape(const Shape& parent,
                                     const Face& face,
                                     const glm::dvec3& normal) {
    Shape child;
    child.scope = parent.scope;
    child.attributes = parent.attributes;

    ShapeGeometry geometry;
    Face copy;
    copy.material = face.material;
    copy.loop = copy_ring(parent.geometry, face.loop, geometry);
    for (const std::vector<uint32_t>& hole : face.holes) {
        // The hole rings keep their winding, which Face documents as opposite to
        // the loop. Re-winding them here would turn a courtyard into a second
        // outer boundary and the triangulation would fill it in.
        copy.holes.push_back(copy_ring(parent.geometry, hole, geometry));
    }
    geometry.faces.push_back(std::move(copy));
    child.geometry = std::move(geometry);

    // The vertices are NOT projected onto the face plane. A non-planar face keeps
    // its thickness, which then shows up honestly as scope.size.z; flattening it
    // would move vertices the neighbouring face still shares.
    reframe(child, parent.scope.axes * face_component_axes(normal));
    return child;
}

/**
 * @brief The edge as a shape of its own
 *
 * Two positions and NO face. An edge has no surface: giving it a two-vertex
 * Face would break Face's contract that a loop is a ring, and every consumer
 * -- face_normal(), triangulate_face(), shape_to_mesh() -- would have to special
 * case it. What carries the meaning is the SCOPE, which is the thing a rule
 * inserts a cornice or a downpipe into.
 */
[[nodiscard]] Shape build_edge_shape(const Shape& parent, const RawComponent& raw) {
    Shape child;
    child.scope = parent.scope;
    child.attributes = parent.attributes;
    child.geometry.positions = {raw.a, raw.b};
    reframe(child, parent.scope.axes * edge_component_axes(raw.direction, raw.normal));
    return child;
}

// ============================================================================
// Filtering
// ============================================================================

/// Does the normal sit within [min, max] degrees of +axis?
[[nodiscard]] bool angle_admits(const glm::dvec3& normal_local,
                                ScopeAxis axis,
                                double min_degrees,
                                double max_degrees) {
    const glm::dvec3 n = safe_normalize(normal_local);
    if (!is_direction(n)) {
        return false;
    }
    // cos is decreasing over [0, 180], so an angle inside [min, max] is a dot
    // product inside [cos(max), cos(min)]. Comparing cosines keeps the only libm
    // call on the AUTHOR's constants, where deg_sin_cos() is exact for 0, 90 and
    // 180 -- which is what a rule file asks for -- instead of on the geometry.
    double sink = 0.0;
    double cos_min = 0.0;
    double cos_max = 0.0;
    deg_sin_cos(std::clamp(min_degrees, 0.0, 180.0), sink, cos_min);
    deg_sin_cos(std::clamp(max_degrees, 0.0, 180.0), sink, cos_max);

    constexpr double kBoundEpsilon = 1e-12;
    const double dot = glm::dot(n, axis_vector(axis));
    return dot >= cos_max - kBoundEpsilon && dot <= cos_min + kBoundEpsilon;
}

} // namespace

// ============================================================================
// Selection helpers
// ============================================================================

ComponentSelection select_angle(ScopeAxis axis, double min_degrees, double max_degrees) {
    ComponentSelection selection;
    selection.use_angle = true;
    selection.angle_axis = axis;
    selection.angle_min_degrees = min_degrees;
    selection.angle_max_degrees = max_degrees;
    return selection;
}

// ============================================================================
// Classification
// ============================================================================

ComponentSelector component_direction(const glm::dvec3& normal_local) {
    const glm::dvec3 n = safe_normalize(normal_local);
    if (!is_direction(n)) {
        return ComponentSelector::All;
    }
    const double ax = std::fabs(n.x);
    const double ay = std::fabs(n.y);
    const double az = std::fabs(n.z);

    // Ties go to y, then to x. Written out rather than left to the comparison
    // order, because a face at exactly 45 degrees is a roof pitch far more often
    // than it is a leaning wall, and because an undocumented tie-break is a
    // classification nobody can predict.
    if (ay >= ax && ay >= az) {
        return n.y >= 0.0 ? ComponentSelector::Top : ComponentSelector::Bottom;
    }
    if (ax >= az) {
        return n.x >= 0.0 ? ComponentSelector::Right : ComponentSelector::Left;
    }
    return n.z >= 0.0 ? ComponentSelector::Front : ComponentSelector::Back;
}

bool selector_admits(ComponentSelector selector,
                     const glm::dvec3& normal_local,
                     double axis_tolerance_degrees) {
    const glm::dvec3 n = safe_normalize(normal_local);
    if (!is_direction(n)) {
        // A component with no direction answers to no word at all, `all`
        // included. It never reaches here in practice -- the gather drops it --
        // but a caller classifying a normal by hand deserves the same answer.
        return false;
    }

    switch (selector) {
        case ComponentSelector::All:
            return true;
        case ComponentSelector::Front:
        case ComponentSelector::Back:
        case ComponentSelector::Left:
        case ComponentSelector::Right:
        case ComponentSelector::Top:
        case ComponentSelector::Bottom:
            return component_direction(n) == selector;
        case ComponentSelector::Side: {
            const ComponentSelector direction = component_direction(n);
            return direction == ComponentSelector::Front || direction == ComponentSelector::Back ||
                   direction == ComponentSelector::Left || direction == ComponentSelector::Right;
        }
        case ComponentSelector::Vertical:
        case ComponentSelector::Horizontal:
        case ComponentSelector::Aslant: {
            double sine = 0.0;
            double cosine = 0.0;
            deg_sin_cos(std::clamp(axis_tolerance_degrees, 0.0, 90.0), sine, cosine);
            const double up = std::fabs(n.y);
            const bool horizontal = up >= cosine;
            const bool vertical = up <= sine;
            if (selector == ComponentSelector::Vertical) {
                return vertical;
            }
            if (selector == ComponentSelector::Horizontal) {
                return horizontal;
            }
            return !vertical && !horizontal;
        }
    }
    return false;
}

glm::dmat3 face_component_axes(const glm::dvec3& normal_local) {
    const glm::dvec3 z = safe_normalize(normal_local);
    if (!is_direction(z)) {
        return glm::dmat3{1.0};
    }
    // The in-plane up. The parent's +y is what a facade means by up; a face too
    // close to horizontal has no +y in its plane, and falls back to the parent's
    // +z so that a roof's frame runs along the shape rather than nowhere.
    const glm::dvec3 reference = std::fabs(z.y) <= kComponentPoleCosine
                                     ? glm::dvec3{0.0, 1.0, 0.0}
                                     : glm::dvec3{0.0, 0.0, 1.0};
    const glm::dvec3 y = safe_normalize(reference - z * glm::dot(z, reference));
    if (!is_direction(y)) {
        return glm::dmat3{1.0};
    }
    // x = cross(y, z) makes (x, y, z) right-handed: cross(x, y) == z.
    return glm::dmat3{glm::cross(y, z), y, z};
}

glm::dmat3 edge_component_axes(const glm::dvec3& direction_local,
                               const glm::dvec3& normal_local) {
    const glm::dvec3 x = safe_normalize(direction_local);
    const glm::dvec3 n = safe_normalize(normal_local);
    if (!is_direction(x) || !is_direction(n)) {
        return glm::dmat3{1.0};
    }
    // The edge direction is kept exactly and the normal is orthogonalised against
    // it, not the other way round: the length a rule measures along the edge has
    // to be the edge's own length. An edge of a non-planar face is not exactly in
    // that face's plane, so without this the frame would not be orthonormal and
    // Scope::to_local() -- a transpose, not an inverse -- would be quietly wrong.
    const glm::dvec3 z = safe_normalize(n - x * glm::dot(x, n));
    if (!is_direction(z)) {
        return glm::dmat3{1.0};
    }
    // y = cross(z, x) points INTO the face, because Face::loop is wound
    // counter-clockwise about the normal.
    return glm::dmat3{x, glm::cross(z, x), z};
}

// ============================================================================
// Spellings
// ============================================================================

bool parse_component_selector(const std::string& text, ComponentSelector& out) {
    // Compared against ast.hpp's own spellings, so the two can never drift: a
    // selector added there is accepted here the day it is added.
    for (uint8_t i = 0; i <= static_cast<uint8_t>(ComponentSelector::Aslant); ++i) {
        const auto candidate = static_cast<ComponentSelector>(i);
        if (text == component_selector_name(candidate)) {
            out = candidate;
            return true;
        }
    }
    return false;
}

bool parse_component_domain(const std::string& text, ComponentDomain& out) {
    for (uint8_t i = 0; i <= static_cast<uint8_t>(ComponentDomain::Object); ++i) {
        const auto candidate = static_cast<ComponentDomain>(i);
        if (text == component_domain_name(candidate)) {
            out = candidate;
            return true;
        }
    }
    return false;
}

// ============================================================================
// The split
// ============================================================================

std::vector<Component> split_components(const Shape& shape,
                                        ComponentDomain domain,
                                        const ComponentSelection& selection,
                                        ComponentSplitReport* report) {
    ComponentSplitReport discarded;
    ComponentSplitReport& out = report != nullptr ? *report : discarded;
    out = ComponentSplitReport{};

    switch (domain) {
        case ComponentDomain::Face:
        case ComponentDomain::Edge:
            break;
        case ComponentDomain::Vertex:
        case ComponentDomain::Object:
            // Reported, never silently empty. A vertex has no frame to orient a
            // scope by without inventing one from its neighbours, and `object`
            // addresses inserted assets, which this build has no notion of.
            out.problems.push_back("the '" + std::string{component_domain_name(domain)} +
                                   "' component domain is not implemented in this build");
            return {};
    }

    if (shape.geometry.faces.empty() || shape.geometry.positions.empty()) {
        out.problems.push_back("this shape has no geometry to split into components");
        return {};
    }

    std::vector<RawComponent> raw = domain == ComponentDomain::Face
                                        ? gather_faces(shape.geometry, out.degenerate)
                                        : gather_edges(shape.geometry, out.degenerate);

    // Sorted BEFORE numbering, so an index addresses a place in the shape rather
    // than a place in the buffer. See point 3 of op_comp.hpp.
    std::sort(raw.begin(), raw.end(),
              [](const RawComponent& a, const RawComponent& b) { return a.key < b.key; });

    out.total = static_cast<uint32_t>(raw.size());

    if (selection.use_angle && selection.angle_min_degrees > selection.angle_max_degrees) {
        out.problems.push_back("this angle range runs from " +
                               format_number(selection.angle_min_degrees) + " to " +
                               format_number(selection.angle_max_degrees) +
                               " degrees, which selects nothing");
    }

    std::vector<Component> selected;
    for (size_t i = 0; i < raw.size(); ++i) {
        const RawComponent& entry = raw[i];
        if (entry.planarity > kComponentPlanarityEpsilon) {
            ++out.non_planar;
            out.worst_planarity = std::max(out.worst_planarity, entry.planarity);
        }
        if (!selector_admits(selection.selector, entry.normal, selection.axis_tolerance_degrees)) {
            continue;
        }
        if (selection.use_angle && !angle_admits(entry.normal, selection.angle_axis,
                                                 selection.angle_min_degrees,
                                                 selection.angle_max_degrees)) {
            continue;
        }

        Component component;
        component.index = static_cast<uint32_t>(i);
        component.normal = entry.normal;
        component.measure = entry.measure;
        component.planarity = entry.planarity;
        component.shape = domain == ComponentDomain::Face
                              ? build_face_shape(shape, shape.geometry.faces[entry.face],
                                                 entry.normal)
                              : build_edge_shape(shape, entry);
        selected.push_back(std::move(component));
    }

    if (!selection.indices.empty()) {
        std::vector<Component> picked;
        picked.reserve(selection.indices.size());
        for (const uint32_t wanted : selection.indices) {
            if (wanted >= selected.size()) {
                // Reported, not dropped. A rule asking for the fourth wall of a
                // triangular tower has a mistake in it, and quietly producing
                // three walls hides it.
                out.problems.push_back("component " + std::to_string(wanted) +
                                       " was asked for, and this selection has " +
                                       std::to_string(selected.size()));
                continue;
            }
            // An index named twice produces the component twice. That is the
            // literal reading of the list and the useful one: `{0, 0}` is how a
            // rule puts two things on one wall.
            picked.push_back(selected[wanted]);
        }
        selected = std::move(picked);
    }

    out.selected = static_cast<uint32_t>(selected.size());
    return selected;
}

uint32_t emit_components(Interpreter& interpreter,
                         const Shape& shape,
                         ComponentDomain domain,
                         const ComponentSelection& selection,
                         const SourceLoc& loc,
                         const std::string& role_prefix) {
    ComponentSplitReport report;
    std::vector<Component> components = split_components(shape, domain, selection, &report);

    for (const std::string& problem : report.problems) {
        interpreter.report(Severity::Error, loc, problem);
    }
    if (report.degenerate > 0) {
        // A Warning, not an Error: a degenerate face is usually something the
        // INPUT geometry did, not something the rule author did. It still gets a
        // line, because a facade that came out one panel short with nothing said
        // about it is the failure this whole file is written to avoid.
        interpreter.report(Severity::Warning, loc,
                           std::to_string(report.degenerate) +
                               " components were dropped for having no area, no length or no "
                               "direction");
    }
    if (report.non_planar > 0) {
        interpreter.report(Severity::Warning, loc,
                           std::to_string(report.non_planar) +
                               " of these faces do not lie flat; the worst is out of plane by " +
                               format_number(report.worst_planarity) +
                               ", which is the depth of its component scope");
    }

    uint32_t emitted = 0;
    for (Component& component : components) {
        std::string role = role_prefix + "." + component_domain_name(domain) + "[" +
                           std::to_string(component.index) + "]";
        if (!interpreter.emit_derived_terminal(shape, std::move(component.shape), std::move(role))) {
            // The shape cap refused it, and it has already said so. Every later
            // component would be refused too.
            break;
        }
        ++emitted;
    }
    return emitted;
}

// ============================================================================
// The `comp` expression namespace
// ============================================================================

namespace {

/**
 * @brief Refuse a component query that has no shape to query
 *
 * The same argument interpreter.cpp's fn_needs_shape() makes: `FunctionArgs::shape`
 * is a default-constructed Shape while an attribute default or a constant is
 * being resolved, and every query over it answers zero. Zero is a plausible
 * number, so it does not look like a fault -- it looks like a blank wall.
 */
[[nodiscard]] bool comp_needs_shape(const FunctionArgs& context) {
    if (context.interpreter.has_current_shape()) {
        return true;
    }
    context.interpreter.fail_shape(
        context.loc, "'" + std::string{context.name} +
                         "' reads the current shape, which an attribute default or a constant "
                         "does not have");
    return false;
}

[[nodiscard]] bool comp_text_arg(const FunctionArgs& context, size_t index, std::string& out) {
    if (index >= context.args.size()) {
        context.interpreter.fail_shape(context.loc, "'" + std::string{context.name} +
                                                        "' needs " + std::to_string(index + 1) +
                                                        " arguments and was given " +
                                                        std::to_string(context.args.size()));
        return false;
    }
    if (!context.args[index].is_text()) {
        context.interpreter.fail_shape(context.loc,
                                       "'" + std::string{context.name} + "' argument " +
                                           std::to_string(index + 1) + " is a " +
                                           context.args[index].type_name() + ", and a word such as "
                                           "\"front\" was expected");
        return false;
    }
    out = context.args[index].as_text();
    return true;
}

/**
 * @brief The domain and selector words a `comp` query was called with
 *
 * Two shapes of call, because both read naturally: `comp.count("front")` is the
 * faces, which is what a rule asks for nine times out of ten, and
 * `comp.count("edge", "top")` names the domain when it is not faces.
 */
[[nodiscard]] bool comp_query_args(const FunctionArgs& context,
                                   size_t first,
                                   size_t last,
                                   ComponentDomain& domain,
                                   ComponentSelector& selector) {
    domain = ComponentDomain::Face;
    selector = ComponentSelector::All;

    const size_t count = std::min(context.args.size(), last + 1);
    if (count <= first) {
        return true;
    }

    std::string text;
    if (count == first + 1) {
        if (!comp_text_arg(context, first, text)) {
            return false;
        }
        if (!parse_component_selector(text, selector)) {
            context.interpreter.fail_shape(context.loc, "'" + std::string{context.name} +
                                                            "' does not know the selector '" +
                                                            text + "'");
            return false;
        }
        return true;
    }

    if (!comp_text_arg(context, first, text) || !parse_component_domain(text, domain)) {
        context.interpreter.fail_shape(context.loc, "'" + std::string{context.name} +
                                                        "' does not know the component domain '" +
                                                        text + "'");
        return false;
    }
    if (!comp_text_arg(context, first + 1, text)) {
        return false;
    }
    if (!parse_component_selector(text, selector)) {
        context.interpreter.fail_shape(context.loc, "'" + std::string{context.name} +
                                                        "' does not know the selector '" + text +
                                                        "'");
        return false;
    }
    return true;
}

/// Report every fault a query's split found, and say whether the query may go on
[[nodiscard]] bool comp_report(const FunctionArgs& context, const ComponentSplitReport& report) {
    if (report.problems.empty()) {
        return true;
    }
    context.interpreter.fail_shape(context.loc,
                                   "'" + std::string{context.name} + "': " + report.problems[0]);
    return false;
}

Value fn_comp_count(const FunctionArgs& context) {
    if (!comp_needs_shape(context)) {
        return Value::number(0.0);
    }
    ComponentSelection selection;
    ComponentDomain domain = ComponentDomain::Face;
    if (!comp_query_args(context, 0, 1, domain, selection.selector)) {
        return Value::number(0.0);
    }
    ComponentSplitReport report;
    const std::vector<Component> components =
        split_components(context.shape, domain, selection, &report);
    if (!comp_report(context, report)) {
        return Value::number(0.0);
    }
    return Value::number(static_cast<double>(components.size()));
}

Value fn_comp_area(const FunctionArgs& context) {
    if (!comp_needs_shape(context)) {
        return Value::number(0.0);
    }
    ComponentSelection selection;
    ComponentDomain domain = ComponentDomain::Face;
    if (!comp_query_args(context, 0, 1, domain, selection.selector)) {
        return Value::number(0.0);
    }
    ComponentSplitReport report;
    const std::vector<Component> components =
        split_components(context.shape, domain, selection, &report);
    if (!comp_report(context, report)) {
        return Value::number(0.0);
    }
    double total = 0.0;
    for (const Component& component : components) {
        total += component.measure;
    }
    return Value::number(total);
}

Value fn_comp_size(const FunctionArgs& context) {
    if (!comp_needs_shape(context)) {
        return Value::number(0.0);
    }
    ComponentSelection selection;
    ComponentDomain domain = ComponentDomain::Face;
    if (!comp_query_args(context, 0, 0, domain, selection.selector)) {
        return Value::number(0.0);
    }

    std::string axis_text;
    if (!comp_text_arg(context, 1, axis_text)) {
        return Value::number(0.0);
    }
    ScopeAxis axis = ScopeAxis::X;
    if (!parse_scope_axis(axis_text, axis)) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' does not know the axis '" +
                             axis_text + "'; the axes of a component are \"x\", \"y\" and \"z\"");
        return Value::number(0.0);
    }

    ComponentSplitReport report;
    const std::vector<Component> components =
        split_components(context.shape, domain, selection, &report);
    if (!comp_report(context, report)) {
        return Value::number(0.0);
    }
    if (components.empty()) {
        // Not zero. A rule reading the width of a wall that is not there has a
        // mistake in it, and a zero width propagates into a split that produces
        // nothing, several operations away from the line that asked.
        context.interpreter.fail_shape(context.loc,
                                       "'" + std::string{context.name} + "' found no '" +
                                           component_selector_name(selection.selector) +
                                           "' component to measure");
        return Value::number(0.0);
    }
    // The extent along the COMPONENT's own axis, which is why this exists:
    // `comp.size("front", "x")` is the width of the wall measured across the
    // wall, and shape.sx after a component split is the width of the whole mass.
    return Value::number(components.front().shape.scope.extent(axis));
}

[[nodiscard]] FunctionTable build_component_functions() {
    FunctionTable table = standard_functions();
    register_component_functions(table);
    return table;
}

} // namespace

void register_component_functions(FunctionTable& table) {
    table.register_function("comp.area", fn_comp_area);
    table.register_function("comp.count", fn_comp_count);
    table.register_function("comp.size", fn_comp_size);
}

const FunctionTable& component_functions() {
    static const FunctionTable table = build_component_functions();
    return table;
}

} // namespace stratum::procgen::rules
