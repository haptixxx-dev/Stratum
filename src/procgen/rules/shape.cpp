// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file shape.cpp
 * @brief Shapes, scopes and the D2 geometry and scope operations
 *
 * The reasoning lives in shape.hpp. What is here is the arithmetic, plus the
 * local notes on the three places it is not obvious: the winding convention that
 * every prism depends on, the Clipper2 bridge, and the miter offset that taper
 * needs and offset must not use.
 */

#include "procgen/rules/shape.hpp"

#include <clipper2/clipper.h>
#include <mapbox/earcut.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

// Earcut adapter for glm::dvec2. The same four lines as osm/mesh_builder.cpp,
// osm/building_collision.cpp and osm/road/junction_polygon.cpp: earcut resolves a
// point's components through this trait, glm does not provide std::tuple_element,
// and the trait is a specialisation rather than a header so that including
// earcut.hpp stays a local decision of the file that triangulates.
namespace mapbox {
namespace util {

template <>
struct nth<0, glm::dvec2> {
    inline static double get(const glm::dvec2& t) { return t.x; }
};

template <>
struct nth<1, glm::dvec2> {
    inline static double get(const glm::dvec2& t) { return t.y; }
};

} // namespace util
} // namespace mapbox

namespace stratum::procgen::rules {

namespace {

/// Below this, two local-metre coordinates are the same point
constexpr double kPointEpsilon = 1e-9;

/// Below this, a direction has no direction and a plane has no normal
constexpr double kDirectionEpsilon = 1e-12;

/**
 * @brief Integer units per scope unit for every Clipper2 call in this file
 *
 * Clipper2 works in int64 and squares coordinates internally, so the usable
 * coordinate range is sqrt(2^63) ~ 3.03e9 counts. At 1e5 counts per metre that
 * is 30 kilometres, against shape geometry that is local to its own scope and
 * therefore the size of a building. The precision bought is 10 micrometres,
 * which is four orders of magnitude finer than any facade detail a rule
 * describes.
 *
 * The same integer-scaled Path64 pattern as osm/road/zoning.cpp, and for the
 * same reason: integer arithmetic gives the same answer on every platform,
 * where Clipper2's double-coordinate API scales to int64 internally with a
 * precision it chooses.
 */
constexpr double kClipperScale = 1e5;

/**
 * @brief Miter limit for Clipper2 offsets
 *
 * 2.0 is Clipper2's own default. A sharper spike than twice the offset distance
 * is squared off instead of being allowed to shoot away from the outline, which
 * is what an unbounded miter does at a near-degenerate corner and what turns a
 * 30 cm setback into a 40 m spike in an OSM footprint with a sliver in it.
 */
constexpr double kMiterLimit = 2.0;

// ============================================================================
// Small vector helpers
// ============================================================================

[[nodiscard]] double length_of(const glm::dvec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * @brief Unit vector, or (0,0,0) when the input is too short to have a direction
 *
 * glm::normalize divides by zero on a zero vector and hands back NaN, which then
 * propagates into every coordinate downstream and, because NaN never compares
 * equal to itself, makes two identical runs compare different. Returning zero
 * lets the caller test and refuse.
 */
[[nodiscard]] glm::dvec3 safe_normalize(const glm::dvec3& v) {
    const double len = length_of(v);
    if (len <= kDirectionEpsilon) {
        return glm::dvec3{0.0};
    }
    return v / len;
}

[[nodiscard]] double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

/// Signed area of a closed 2D ring, first point not repeated. Positive when CCW.
[[nodiscard]] double ring_signed_area(const std::vector<glm::dvec2>& ring) {
    if (ring.size() < 3) {
        return 0.0;
    }
    double twice = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % ring.size()];
        twice += cross2(a, b);
    }
    return twice * 0.5;
}

// ============================================================================
// Face plane basis
// ============================================================================

/**
 * @brief An orthonormal frame on a face's own plane
 *
 * (u, v, n) is right-handed, so a face loop wound counter-clockwise about n --
 * which Face documents as the contract -- projects to a ring with POSITIVE
 * signed area in (u, v). Every 2D step in this file relies on that: it is how
 * the offset code knows which side of an edge the material is on.
 */
struct FaceBasis {
    glm::dvec3 origin{0.0};
    glm::dvec3 u{1.0, 0.0, 0.0};
    glm::dvec3 v{0.0, 0.0, 1.0};
    glm::dvec3 n{0.0, 1.0, 0.0};
    bool valid = false;

    [[nodiscard]] glm::dvec2 project(const glm::dvec3& p) const {
        const glm::dvec3 d = p - origin;
        return glm::dvec2{glm::dot(d, u), glm::dot(d, v)};
    }

    [[nodiscard]] glm::dvec3 unproject(const glm::dvec2& p) const {
        return origin + u * p.x + v * p.y;
    }
};

/**
 * @brief Build a plane frame from a normal
 *
 * The seed axis is chosen by the normal alone -- the world axis the normal is
 * least aligned with -- so the frame is a pure function of the geometry and does
 * not depend on the order the face was built in. The same trick, for the same
 * reason, as the tangent fallback in renderer/mesh.hpp.
 */
[[nodiscard]] FaceBasis basis_from_normal(const glm::dvec3& origin, const glm::dvec3& normal) {
    FaceBasis basis;
    const glm::dvec3 n = safe_normalize(normal);
    if (length_of(n) <= 0.5) {
        return basis;  // valid stays false
    }
    const glm::dvec3 seed = std::fabs(n.x) < 0.9 ? glm::dvec3{1.0, 0.0, 0.0}
                                                 : glm::dvec3{0.0, 1.0, 0.0};
    const glm::dvec3 u = safe_normalize(seed - n * glm::dot(n, seed));
    if (length_of(u) <= 0.5) {
        return basis;
    }
    basis.origin = origin;
    basis.n = n;
    basis.u = u;
    basis.v = glm::cross(n, u);
    basis.valid = true;
    return basis;
}

[[nodiscard]] FaceBasis face_basis(const ShapeGeometry& geometry, const Face& face) {
    if (face.loop.empty()) {
        return FaceBasis{};
    }
    return basis_from_normal(geometry.positions[face.loop[0]], face_normal(geometry, face));
}

/// Project one ring of vertex indices into the face plane
[[nodiscard]] std::vector<glm::dvec2> project_ring(const ShapeGeometry& geometry,
                                                   const std::vector<uint32_t>& ring,
                                                   const FaceBasis& basis) {
    std::vector<glm::dvec2> out;
    out.reserve(ring.size());
    for (const uint32_t index : ring) {
        out.push_back(basis.project(geometry.positions[index]));
    }
    return out;
}

// ============================================================================
// Ring and face construction
// ============================================================================

/// Append a ring of 3D points, returning their new indices
[[nodiscard]] std::vector<uint32_t> add_ring(ShapeGeometry& geometry,
                                             const std::vector<glm::dvec3>& points) {
    std::vector<uint32_t> indices;
    indices.reserve(points.size());
    for (const glm::dvec3& p : points) {
        indices.push_back(static_cast<uint32_t>(geometry.positions.size()));
        geometry.positions.push_back(p);
    }
    return indices;
}

/// A loop wound the other way round
[[nodiscard]] std::vector<uint32_t> reversed(const std::vector<uint32_t>& loop) {
    return std::vector<uint32_t>(loop.rbegin(), loop.rend());
}

/// Flip a face's winding, outer ring and holes together
void reverse_face(Face& face) {
    face.loop = reversed(face.loop);
    for (auto& hole : face.holes) {
        hole = reversed(hole);
    }
}

// ============================================================================
// Clipper2 bridge
// ============================================================================

[[nodiscard]] Clipper2Lib::Path64 to_path(const std::vector<glm::dvec2>& ring) {
    Clipper2Lib::Path64 path;
    path.reserve(ring.size());
    for (const glm::dvec2& p : ring) {
        path.emplace_back(static_cast<int64_t>(std::llround(p.x * kClipperScale)),
                          static_cast<int64_t>(std::llround(p.y * kClipperScale)));
    }
    return path;
}

[[nodiscard]] std::vector<glm::dvec2> from_path(const Clipper2Lib::Path64& path) {
    std::vector<glm::dvec2> ring;
    ring.reserve(path.size());
    for (const Clipper2Lib::Point64& p : path) {
        ring.push_back(glm::dvec2{static_cast<double>(p.x) / kClipperScale,
                                  static_cast<double>(p.y) / kClipperScale});
    }
    return ring;
}

/**
 * @brief Rotate a ring so it starts at its lexicographically smallest point
 *
 * Clipper2 is integer arithmetic and is deterministic for identical input, but
 * WHERE in a ring it starts is an implementation detail that a library version
 * bump is free to change. Rotating to a canonical start makes the vertex order
 * of an offset outline a function of the outline alone, which is what the
 * "two runs agree byte for byte" test actually asserts. It costs one linear
 * scan per ring.
 */
void canonicalise_path(Clipper2Lib::Path64& path) {
    if (path.size() < 2) {
        return;
    }
    size_t best = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        if (path[i].x < path[best].x || (path[i].x == path[best].x && path[i].y < path[best].y)) {
            best = i;
        }
    }
    std::rotate(path.begin(), path.begin() + static_cast<std::ptrdiff_t>(best), path.end());
}

/// Canonical ordering of a path set: rings rotated, then sorted by their first point
void canonicalise_paths(Clipper2Lib::Paths64& paths) {
    for (auto& path : paths) {
        canonicalise_path(path);
    }
    std::sort(paths.begin(), paths.end(),
              [](const Clipper2Lib::Path64& a, const Clipper2Lib::Path64& b) {
                  if (a.empty() || b.empty()) {
                      return a.size() < b.size();
                  }
                  if (a[0].x != b[0].x) {
                      return a[0].x < b[0].x;
                  }
                  if (a[0].y != b[0].y) {
                      return a[0].y < b[0].y;
                  }
                  return a.size() < b.size();
              });
}

/**
 * @brief Turn a Clipper2 solution into faces, nesting holes inside their outers
 *
 * Outers are the positive-area rings and holes the negative-area ones, which is
 * how Clipper2 winds a NonZero solution. A hole is assigned to the SMALLEST outer
 * that contains it, because a hole inside a hole's island is inside two outers
 * and only the inner one owns it.
 *
 * @param paths     Solution, already canonicalised
 * @param basis     Plane to lift back into
 * @param material  Material to give every face produced
 * @param geometry  Receives the positions and faces, appended
 * @return Number of faces appended
 */
size_t paths_to_faces(const Clipper2Lib::Paths64& paths,
                      const FaceBasis& basis,
                      MaterialKey material,
                      ShapeGeometry& geometry) {
    struct Ring {
        const Clipper2Lib::Path64* path = nullptr;
        double area = 0.0;
        bool is_hole = false;
        size_t owner = static_cast<size_t>(-1);
    };

    std::vector<Ring> rings;
    rings.reserve(paths.size());
    for (const auto& path : paths) {
        if (path.size() < 3) {
            continue;
        }
        Ring ring;
        ring.path = &path;
        ring.area = Clipper2Lib::Area(path);
        ring.is_hole = ring.area < 0.0;
        rings.push_back(ring);
    }

    for (size_t h = 0; h < rings.size(); ++h) {
        if (!rings[h].is_hole) {
            continue;
        }
        const Clipper2Lib::Point64& probe = (*rings[h].path)[0];
        double best_area = 0.0;
        for (size_t o = 0; o < rings.size(); ++o) {
            if (rings[o].is_hole) {
                continue;
            }
            if (Clipper2Lib::PointInPolygon(probe, *rings[o].path) ==
                Clipper2Lib::PointInPolygonResult::IsOutside) {
                continue;
            }
            if (rings[h].owner == static_cast<size_t>(-1) || rings[o].area < best_area) {
                rings[h].owner = o;
                best_area = rings[o].area;
            }
        }
    }

    // Outers in the order they appear (already canonical), so the face order is a
    // function of the geometry rather than of Clipper2's internal traversal.
    std::vector<size_t> outer_to_face(rings.size(), static_cast<size_t>(-1));
    size_t added = 0;
    for (size_t o = 0; o < rings.size(); ++o) {
        if (rings[o].is_hole) {
            continue;
        }
        Face face;
        face.material = material;
        std::vector<glm::dvec3> points;
        for (const glm::dvec2& p : from_path(*rings[o].path)) {
            points.push_back(basis.unproject(p));
        }
        face.loop = add_ring(geometry, points);
        outer_to_face[o] = geometry.faces.size();
        geometry.faces.push_back(std::move(face));
        ++added;
    }

    for (size_t h = 0; h < rings.size(); ++h) {
        if (!rings[h].is_hole || rings[h].owner == static_cast<size_t>(-1)) {
            continue;
        }
        const size_t face_index = outer_to_face[rings[h].owner];
        if (face_index == static_cast<size_t>(-1)) {
            continue;
        }
        std::vector<glm::dvec3> points;
        for (const glm::dvec2& p : from_path(*rings[h].path)) {
            points.push_back(basis.unproject(p));
        }
        geometry.faces[face_index].holes.push_back(add_ring(geometry, points));
    }

    return added;
}

/// The face's outer ring and holes as one Clipper2 subject, unioned and canonical
[[nodiscard]] Clipper2Lib::Paths64 face_to_paths(const ShapeGeometry& geometry,
                                                 const Face& face,
                                                 const FaceBasis& basis) {
    Clipper2Lib::Paths64 subject;
    subject.push_back(to_path(project_ring(geometry, face.loop, basis)));
    for (const auto& hole : face.holes) {
        if (hole.size() >= 3) {
            subject.push_back(to_path(project_ring(geometry, hole, basis)));
        }
    }
    Clipper2Lib::Paths64 unioned =
        Clipper2Lib::Union(subject, Clipper2Lib::FillRule::NonZero);
    canonicalise_paths(unioned);
    return unioned;
}

// ============================================================================
// Miter offset, for taper
// ============================================================================

/**
 * @brief Inset a ring by moving each edge along its inward normal and re-intersecting
 *
 * Preserves the vertex count, which is the whole reason this exists next to a
 * perfectly good Clipper2 offset: taper's side walls are one quad per base edge,
 * so the tapered ring has to have the same vertices in the same order. A robust
 * offset is exactly the operation that does not promise that.
 *
 * The price is that a miter offset is not robust, so the failure is checked
 * rather than hoped for: if any edge comes out pointing the other way, the
 * outline has folded through itself and this returns false.
 *
 * @param ring     Ring in the face plane, first point not repeated
 * @param distance Inset distance, positive
 * @param out      Receives the inset ring, same size as @p ring
 * @return false when the ring is degenerate or the inset folds it
 */
[[nodiscard]] bool miter_inset(const std::vector<glm::dvec2>& ring,
                               double distance,
                               std::vector<glm::dvec2>& out) {
    const size_t n = ring.size();
    if (n < 3) {
        return false;
    }

    const double area = ring_signed_area(ring);
    if (std::fabs(area) <= kPointEpsilon) {
        return false;
    }
    // A counter-clockwise ring (positive area) has its material on the LEFT of
    // each edge, so the inward normal is the edge direction turned a quarter
    // turn anticlockwise. A hole is wound the other way and its material is on
    // the right, so the sign flips -- which is what makes an inset of the FACE
    // grow the hole, as it must.
    const double side = area > 0.0 ? 1.0 : -1.0;

    struct Line {
        glm::dvec2 normal{0.0};
        double offset = 0.0;
        glm::dvec2 direction{0.0};
        bool degenerate = true;
    };

    std::vector<Line> lines(n);
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2 a = ring[i];
        const glm::dvec2 b = ring[(i + 1) % n];
        const glm::dvec2 d = b - a;
        const double len = std::sqrt(d.x * d.x + d.y * d.y);
        if (len <= kPointEpsilon) {
            continue;  // stays degenerate; the neighbours carry the corner
        }
        const glm::dvec2 dir = d / len;
        Line& line = lines[i];
        line.direction = dir;
        line.normal = glm::dvec2{-dir.y, dir.x} * side;
        line.offset = glm::dot(a, line.normal) + distance;
        line.degenerate = false;
    }

    out.assign(n, glm::dvec2{0.0});
    for (size_t i = 0; i < n; ++i) {
        // Vertex i is where edge (i-1 -> i) meets edge (i -> i+1).
        const Line& prev = lines[(i + n - 1) % n];
        const Line& next = lines[i];
        if (prev.degenerate && next.degenerate) {
            return false;
        }
        if (prev.degenerate) {
            out[i] = ring[i] + next.normal * distance;
            continue;
        }
        if (next.degenerate) {
            out[i] = ring[i] + prev.normal * distance;
            continue;
        }
        const double det = cross2(prev.normal, next.normal);
        if (std::fabs(det) <= 1e-12) {
            // Collinear edges: the corner does not turn, so both lines are the
            // same line and the vertex simply slides along the normal.
            out[i] = ring[i] + next.normal * distance;
            continue;
        }
        out[i] = glm::dvec2{(prev.offset * next.normal.y - next.offset * prev.normal.y) / det,
                            (next.offset * prev.normal.x - prev.offset * next.normal.x) / det};
    }

    // The fold check. An edge that reverses means the two vertices bounding it
    // crossed over each other, which is a self-intersection whether or not the
    // total area is still the right sign.
    for (size_t i = 0; i < n; ++i) {
        if (lines[i].degenerate) {
            continue;
        }
        const glm::dvec2 d = out[(i + 1) % n] - out[i];
        if (glm::dot(d, lines[i].direction) <= 0.0) {
            return false;
        }
    }

    const double new_area = ring_signed_area(out);
    return (new_area > 0.0) == (area > 0.0) && std::fabs(new_area) > kPointEpsilon;
}

} // namespace

// ============================================================================
// Deterministic mixing
// ============================================================================

uint64_t seed_mix64(uint64_t x) noexcept {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

uint64_t seed_mix2(uint64_t a, uint64_t b) noexcept {
    return seed_mix64(a ^ seed_mix64(b + 0x165667B19E3779F9ull));
}

double seed_unit(uint64_t hash) noexcept {
    return static_cast<double>(hash >> 11) * 0x1.0p-53;
}

// ============================================================================
// Scope
// ============================================================================

const char* scope_axis_name(ScopeAxis axis) {
    switch (axis) {
        case ScopeAxis::X: return "x";
        case ScopeAxis::Y: return "y";
        case ScopeAxis::Z: return "z";
    }
    return "?";
}

bool parse_scope_axis(const std::string& text, ScopeAxis& out) {
    if (text == "x") { out = ScopeAxis::X; return true; }
    if (text == "y") { out = ScopeAxis::Y; return true; }
    if (text == "z") { out = ScopeAxis::Z; return true; }
    return false;
}

void Scope::orthonormalise() {
    // Plain Gram-Schmidt, which PRESERVES handedness: a scope mirrored by
    // mirror_scope() is left-handed on purpose, and a "fix" that forced
    // det == +1 here would quietly undo it.
    glm::dvec3 c0 = axes[0];
    glm::dvec3 c1 = axes[1];
    glm::dvec3 c2 = axes[2];

    const double len0 = length_of(c0);
    c0 = len0 > kDirectionEpsilon ? c0 / len0 : glm::dvec3{1.0, 0.0, 0.0};

    c1 -= c0 * glm::dot(c0, c1);
    const double len1 = length_of(c1);
    if (len1 > kDirectionEpsilon) {
        c1 /= len1;
    } else {
        const glm::dvec3 seed =
            std::fabs(c0.x) < 0.9 ? glm::dvec3{1.0, 0.0, 0.0} : glm::dvec3{0.0, 1.0, 0.0};
        c1 = safe_normalize(seed - c0 * glm::dot(c0, seed));
    }

    const double handedness = glm::dot(c2, glm::cross(c0, c1)) < 0.0 ? -1.0 : 1.0;
    c2 -= c0 * glm::dot(c0, c2);
    c2 -= c1 * glm::dot(c1, c2);
    const double len2 = length_of(c2);
    c2 = len2 > kDirectionEpsilon ? c2 / len2 : glm::cross(c0, c1) * handedness;

    axes = glm::dmat3{c0, c1, c2};
}

// ============================================================================
// Geometry queries
// ============================================================================

glm::dvec3 face_normal(const ShapeGeometry& geometry, const Face& face) {
    if (face.loop.size() < 3) {
        return glm::dvec3{0.0};
    }
    // Newell's method. It works for a non-planar polygon and, unlike "cross two
    // edges", does not return a zero normal for a face whose first three
    // vertices happen to be collinear -- which every rectangle produced by a
    // split with a redundant vertex in it is.
    glm::dvec3 n{0.0};
    for (size_t i = 0; i < face.loop.size(); ++i) {
        const glm::dvec3& a = geometry.positions[face.loop[i]];
        const glm::dvec3& b = geometry.positions[face.loop[(i + 1) % face.loop.size()]];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return safe_normalize(n);
}

namespace {

/// Twice the vector area of one ring, by Newell. Its length is twice the area.
[[nodiscard]] glm::dvec3 ring_vector_area(const ShapeGeometry& geometry,
                                          const std::vector<uint32_t>& ring) {
    glm::dvec3 n{0.0};
    if (ring.size() < 3) {
        return n;
    }
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec3& a = geometry.positions[ring[i]];
        const glm::dvec3& b = geometry.positions[ring[(i + 1) % ring.size()]];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return n * 0.5;
}

} // namespace

double face_area(const ShapeGeometry& geometry, const Face& face) {
    const glm::dvec3 outer = ring_vector_area(geometry, face.loop);
    const double outer_area = length_of(outer);
    if (outer_area <= 0.0) {
        return 0.0;
    }
    const glm::dvec3 normal = outer / outer_area;

    double area = outer_area;
    for (const auto& hole : face.holes) {
        // A hole is wound against the outer ring, so its vector area projects
        // NEGATIVELY onto the face normal. Taking the projection rather than the
        // magnitude is what makes a ring handed in the wrong way subtract
        // nothing instead of subtracting twice.
        area += glm::dot(ring_vector_area(geometry, hole), normal);
    }
    return area > 0.0 ? area : 0.0;
}

double geometry_area(const ShapeGeometry& geometry) {
    double total = 0.0;
    for (const Face& face : geometry.faces) {
        total += face_area(geometry, face);
    }
    return total;
}

double geometry_volume(const ShapeGeometry& geometry) {
    double six_volume = 0.0;
    for (const Face& face : geometry.faces) {
        const std::vector<uint32_t> tri = triangulate_face(geometry, face);
        for (size_t i = 0; i + 2 < tri.size(); i += 3) {
            const glm::dvec3& a = geometry.positions[tri[i]];
            const glm::dvec3& b = geometry.positions[tri[i + 1]];
            const glm::dvec3& c = geometry.positions[tri[i + 2]];
            six_volume += glm::dot(a, glm::cross(b, c));
        }
    }
    return six_volume / 6.0;
}

bool geometry_bounds(const ShapeGeometry& geometry, glm::dvec3& min, glm::dvec3& max) {
    if (geometry.positions.empty()) {
        return false;
    }
    min = geometry.positions[0];
    max = geometry.positions[0];
    for (const glm::dvec3& p : geometry.positions) {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }
    return true;
}

// ============================================================================
// Values
// ============================================================================

const char* Value::type_name() const {
    switch (data.index()) {
        case 0: return "float";
        case 1: return "bool";
        case 2: return "string";
        default: return "array";
    }
}

std::string format_number(double value) {
    if (std::isnan(value)) {
        return "nan";
    }
    if (std::isinf(value)) {
        return value > 0.0 ? "inf" : "-inf";
    }
    // -0.0 == 0.0 compares true but prints differently, and a dump that differs
    // for two runs that agree is a false failure nobody can reproduce.
    if (value == 0.0) {
        value = 0.0;
    }
    char buffer[40];
    std::snprintf(buffer, sizeof(buffer), "%.10g", value);
    return std::string{buffer};
}

std::string Value::to_text() const {
    switch (data.index()) {
        case 0: return format_number(as_number());
        case 1: return as_bool() ? "true" : "false";
        case 2: return as_text();
        default: break;
    }
    std::string out = "[";
    const ValueArray& items = as_array();
    for (size_t i = 0; i < items.size(); ++i) {
        if (i != 0) {
            out += ", ";
        }
        out += items[i].to_text();
    }
    out += "]";
    return out;
}

// ============================================================================
// Frame maintenance
// ============================================================================

void refit_scope(Shape& shape) {
    glm::dvec3 min{0.0};
    glm::dvec3 max{0.0};
    if (!geometry_bounds(shape.geometry, min, max)) {
        // The documented exception: a shape with no geometry keeps whatever
        // scope was built for it, so D4 can set up a frame and then insert.
        return;
    }
    if (min != glm::dvec3{0.0}) {
        for (glm::dvec3& p : shape.geometry.positions) {
            p -= min;
        }
        shape.scope.origin += shape.scope.axes * min;
    }
    shape.scope.size = max - min;
    shape.scope.size = glm::max(shape.scope.size, glm::dvec3{0.0});
}

void reframe(Shape& shape, const glm::dmat3& new_axes) {
    Scope target = shape.scope;
    target.axes = new_axes;
    target.orthonormalise();

    if (!shape.geometry.positions.empty()) {
        const glm::dmat3 inverse = glm::transpose(target.axes);
        for (glm::dvec3& p : shape.geometry.positions) {
            const glm::dvec3 world = shape.scope.to_world(p);
            p = inverse * (world - target.origin);
        }
    }

    shape.scope.axes = target.axes;
    refit_scope(shape);
}

// ============================================================================
// Construction
// ============================================================================

Shape shape_from_rings(const std::vector<glm::dvec2>& outer,
                       const std::vector<std::vector<glm::dvec2>>& holes,
                       double y) {
    Shape shape;
    if (outer.size() < 3) {
        return shape;
    }

    // The winding is FIXED here rather than trusted. A ring handed in the wrong
    // way round produces a face whose normal points down, and every prism built
    // on it is inside out -- a failure that shows up as a black building in the
    // renderer, three subsystems away from the caller that got the ring order
    // wrong. shape_from_polygon() and shape_from_rings() are the only doors into
    // this file, so this is the one place that has to be right.
    //
    // A ring that is counter-clockwise in (x, z) has a NEGATIVE y component by
    // Newell, because (x, z) as plotted is a left-handed view of the xz plane.
    // So the ring is reversed when its signed area in (x, z) is positive.
    std::vector<glm::dvec2> ring = outer;
    if (ring_signed_area(ring) > 0.0) {
        std::reverse(ring.begin(), ring.end());
    }

    std::vector<glm::dvec3> points;
    points.reserve(ring.size());
    for (const glm::dvec2& p : ring) {
        points.push_back(glm::dvec3{p.x, y, p.y});
    }

    Face face;
    face.loop = add_ring(shape.geometry, points);

    for (const auto& hole : holes) {
        if (hole.size() < 3) {
            continue;
        }
        std::vector<glm::dvec2> hole_ring = hole;
        if (ring_signed_area(hole_ring) < 0.0) {
            std::reverse(hole_ring.begin(), hole_ring.end());
        }
        std::vector<glm::dvec3> hole_points;
        hole_points.reserve(hole_ring.size());
        for (const glm::dvec2& p : hole_ring) {
            hole_points.push_back(glm::dvec3{p.x, y, p.y});
        }
        face.holes.push_back(add_ring(shape.geometry, hole_points));
    }

    shape.geometry.faces.push_back(std::move(face));
    refit_scope(shape);
    return shape;
}

Shape shape_from_polygon(const std::vector<glm::dvec2>& ring, double y) {
    return shape_from_rings(ring, {}, y);
}

Shape shape_from_rect(double size_x, double size_z, double y) {
    const std::vector<glm::dvec2> ring = {
        {0.0, 0.0}, {size_x, 0.0}, {size_x, size_z}, {0.0, size_z}};
    return shape_from_polygon(ring, y);
}

// ============================================================================
// Extrude
// ============================================================================

namespace {

/**
 * @brief Build one prism from one face
 *
 * @param source    Geometry the face indexes into
 * @param face      Face to extrude, already oriented so its normal agrees with @p dir
 * @param dir       Unit direction, in local coordinates
 * @param height    Distance along @p dir, positive
 * @param out       Receives the prism's faces, appended
 */
void build_prism(const ShapeGeometry& source,
                 const Face& face,
                 const glm::dvec3& dir,
                 double height,
                 ShapeGeometry& out) {
    const glm::dvec3 lift = dir * height;

    // One pair of rings per boundary loop. The bottom ring keeps the source
    // positions; the top ring is the same ring lifted.
    auto make_pair = [&](const std::vector<uint32_t>& ring,
                         std::vector<uint32_t>& bottom,
                         std::vector<uint32_t>& top) {
        std::vector<glm::dvec3> low;
        std::vector<glm::dvec3> high;
        low.reserve(ring.size());
        high.reserve(ring.size());
        for (const uint32_t index : ring) {
            low.push_back(source.positions[index]);
            high.push_back(source.positions[index] + lift);
        }
        bottom = add_ring(out, low);
        top = add_ring(out, high);
    };

    std::vector<uint32_t> outer_bottom;
    std::vector<uint32_t> outer_top;
    make_pair(face.loop, outer_bottom, outer_top);

    std::vector<std::vector<uint32_t>> hole_bottoms;
    std::vector<std::vector<uint32_t>> hole_tops;
    for (const auto& hole : face.holes) {
        if (hole.size() < 3) {
            continue;
        }
        std::vector<uint32_t> b;
        std::vector<uint32_t> t;
        make_pair(hole, b, t);
        hole_bottoms.push_back(std::move(b));
        hole_tops.push_back(std::move(t));
    }

    // The cap at the far end keeps the source winding, so its normal still
    // points along dir -- outward, because dir points away from the solid there.
    Face cap_top;
    cap_top.material = face.material;
    cap_top.loop = outer_top;
    cap_top.holes = hole_tops;

    // The cap where the face was is the same ring reversed, so its normal points
    // back against dir -- outward at that end.
    Face cap_bottom;
    cap_bottom.material = face.material;
    cap_bottom.loop = reversed(outer_bottom);
    cap_bottom.holes.reserve(hole_bottoms.size());
    for (const auto& hole : hole_bottoms) {
        cap_bottom.holes.push_back(reversed(hole));
    }

    out.faces.push_back(std::move(cap_bottom));
    out.faces.push_back(std::move(cap_top));

    // A side quad for each boundary edge, wound (bottom i, bottom j, top j,
    // top i). For a ring wound counter-clockwise about dir that gives an
    // outward-facing wall; for a hole ring, which is wound the other way, the
    // same expression gives a wall facing INTO the hole, which is also outward
    // for the solid. One formula, both cases, because the winding of the rings
    // already carries the difference.
    auto add_walls = [&](const std::vector<uint32_t>& bottom,
                         const std::vector<uint32_t>& top) {
        for (size_t i = 0; i < bottom.size(); ++i) {
            const size_t j = (i + 1) % bottom.size();
            Face wall;
            wall.material = face.material;
            wall.loop = {bottom[i], bottom[j], top[j], top[i]};
            out.faces.push_back(std::move(wall));
        }
    };

    add_walls(outer_bottom, outer_top);
    for (size_t h = 0; h < hole_bottoms.size(); ++h) {
        add_walls(hole_bottoms[h], hole_tops[h]);
    }
}

/// extrude, with the direction already resolved to local coordinates
[[nodiscard]] OpResult extrude_along(Shape& shape, const glm::dvec3& dir_local, double distance) {
    if (shape.geometry.faces.empty()) {
        return OpResult::failure("the shape has no faces to extrude");
    }
    if (std::fabs(distance) <= kPointEpsilon) {
        // Two coincident caps and a ring of zero-area quads is not a shape, and
        // it is never what the author meant. Saying so beats emitting geometry
        // that disappears in the renderer for no stated reason.
        return OpResult::failure("the extrusion distance is zero");
    }
    const glm::dvec3 dir = safe_normalize(dir_local);
    if (length_of(dir) <= 0.5) {
        return OpResult::failure("the extrusion axis has no direction");
    }

    // Fold the sign of the distance into the direction. Building the prism
    // "forwards" and then reversing every face for a negative distance is the
    // same result by a longer route, and the longer route has a step that is
    // easy to forget.
    const double height = std::fabs(distance);
    const glm::dvec3 travel = distance < 0.0 ? -dir : dir;

    ShapeGeometry out;
    for (const Face& face : shape.geometry.faces) {
        const glm::dvec3 n = face_normal(shape.geometry, face);
        if (length_of(n) <= 0.5) {
            continue;  // a degenerate face extrudes to nothing
        }
        const double along = glm::dot(n, travel);
        if (std::fabs(along) <= 1e-9) {
            return OpResult::failure("a face lies parallel to the extrusion axis");
        }
        Face oriented = face;
        if (along < 0.0) {
            reverse_face(oriented);
        }
        build_prism(shape.geometry, oriented, travel, height, out);
    }

    if (out.faces.empty()) {
        return OpResult::failure("every face was degenerate");
    }

    shape.geometry = std::move(out);
    refit_scope(shape);
    return OpResult::success();
}

/// The largest face, ties broken by the lowest index so the choice is a function
/// of the geometry and not of the order the faces were built in.
[[nodiscard]] size_t dominant_face(const ShapeGeometry& geometry) {
    size_t best = static_cast<size_t>(-1);
    double best_area = 0.0;
    for (size_t i = 0; i < geometry.faces.size(); ++i) {
        const double area = face_area(geometry, geometry.faces[i]);
        if (best == static_cast<size_t>(-1) || area > best_area) {
            best = i;
            best_area = area;
        }
    }
    return best;
}

} // namespace

OpResult extrude_shape(Shape& shape, ScopeAxis axis, double distance) {
    glm::dvec3 dir{0.0};
    dir[static_cast<int>(axis)] = 1.0;
    return extrude_along(shape, dir, distance);
}

OpResult extrude_shape_along_normal(Shape& shape, double distance) {
    const size_t index = dominant_face(shape.geometry);
    if (index == static_cast<size_t>(-1)) {
        return OpResult::failure("the shape has no faces to extrude");
    }
    const glm::dvec3 n = face_normal(shape.geometry, shape.geometry.faces[index]);
    if (length_of(n) <= 0.5) {
        return OpResult::failure("the dominant face has no normal to extrude along");
    }
    return extrude_along(shape, n, distance);
}

// ============================================================================
// Offset and setback
// ============================================================================

bool parse_offset_selector(const std::string& text, OffsetSelector& out) {
    if (text == "inside") { out = OffsetSelector::Inside; return true; }
    if (text == "border") { out = OffsetSelector::Border; return true; }
    if (text == "all") { out = OffsetSelector::All; return true; }
    return false;
}

namespace {

/**
 * @brief Offset one face and split the result into the inner part and the border
 *
 * @param geometry Geometry the face indexes into
 * @param face     Face to offset
 * @param distance Signed offset distance; negative insets
 * @param inside   Receives the offset outline, appended
 * @param border   Receives the ring between the two outlines, appended. May be
 *                 the same object as @p inside, in which case both land in it.
 * @return false when the face has no usable plane
 */
bool offset_one_face(const ShapeGeometry& geometry,
                     const Face& face,
                     double distance,
                     ShapeGeometry* inside,
                     ShapeGeometry* border) {
    const FaceBasis basis = face_basis(geometry, face);
    if (!basis.valid) {
        return false;
    }

    const Clipper2Lib::Paths64 subject = face_to_paths(geometry, face, basis);
    if (subject.empty()) {
        return false;
    }

    Clipper2Lib::Paths64 grown =
        Clipper2Lib::InflatePaths(subject, distance * kClipperScale,
                                  Clipper2Lib::JoinType::Miter,
                                  Clipper2Lib::EndType::Polygon, kMiterLimit);
    grown = Clipper2Lib::Union(grown, Clipper2Lib::FillRule::NonZero);
    canonicalise_paths(grown);

    if (inside != nullptr) {
        paths_to_faces(grown, basis, face.material, *inside);
    }
    if (border != nullptr) {
        // Xor rather than Difference, so the caller does not have to know which
        // of the two outlines is the larger one. An outward offset's border is
        // the ring outside the original; an inward offset's is the ring inside
        // it; the symmetric difference is that ring either way.
        Clipper2Lib::Paths64 ring =
            Clipper2Lib::Xor(subject, grown, Clipper2Lib::FillRule::NonZero);
        canonicalise_paths(ring);
        paths_to_faces(ring, basis, face.material, *border);
    }
    return true;
}

} // namespace

OpResult offset_shape(Shape& shape, double distance, OffsetSelector selector) {
    if (shape.geometry.faces.empty()) {
        return OpResult::failure("the shape has no faces to offset");
    }
    if (std::fabs(distance) <= kPointEpsilon) {
        return OpResult::failure("the offset distance is zero");
    }

    ShapeGeometry out;
    bool any_plane = false;
    for (const Face& face : shape.geometry.faces) {
        ShapeGeometry* inside = selector == OffsetSelector::Border ? nullptr : &out;
        ShapeGeometry* border = selector == OffsetSelector::Inside ? nullptr : &out;
        any_plane = offset_one_face(shape.geometry, face, distance, inside, border) || any_plane;
    }

    if (!any_plane) {
        return OpResult::failure("no face has a usable plane to offset in");
    }
    if (out.faces.empty()) {
        return OpResult::failure("the offset removed the whole shape");
    }

    shape.geometry = std::move(out);
    refit_scope(shape);
    return OpResult::success();
}

OpResult setback_shape(Shape& shape, double distance, ShapeGeometry& border, bool keep_border) {
    border.clear();

    if (shape.geometry.faces.empty()) {
        return OpResult::failure("the shape has no faces to set back");
    }
    if (!(distance > kPointEpsilon)) {
        // A setback that grows the shape has no removed border to hand back, so
        // the operation would silently become offset() with a misleading name.
        return OpResult::failure("the setback distance must be positive");
    }

    ShapeGeometry inner;
    ShapeGeometry ring;
    bool any_plane = false;
    for (const Face& face : shape.geometry.faces) {
        any_plane = offset_one_face(shape.geometry, face, -distance, &inner,
                                    keep_border ? &ring : nullptr) ||
                    any_plane;
    }

    if (!any_plane) {
        return OpResult::failure("no face has a usable plane to set back in");
    }
    if (inner.faces.empty()) {
        return OpResult::failure("the setback removed the whole shape");
    }

    shape.geometry = std::move(inner);
    if (keep_border) {
        border = std::move(ring);
    }
    refit_scope(shape);
    return OpResult::success();
}

// ============================================================================
// Taper
// ============================================================================

OpResult taper_shape(Shape& shape, double height) {
    if (shape.geometry.faces.empty()) {
        return OpResult::failure("the shape has no faces to taper");
    }
    if (!(height > kPointEpsilon)) {
        return OpResult::failure("the taper height must be positive");
    }

    ShapeGeometry out;
    for (const Face& face : shape.geometry.faces) {
        const FaceBasis basis = face_basis(shape.geometry, face);
        if (!basis.valid) {
            continue;
        }

        // Each face rises along ITS OWN normal, so a footprint tapers upward and
        // a wall tapers outward without the operation needing an axis argument
        // -- which is what ast.hpp's one-argument signature commits it to.
        const glm::dvec3 lift = basis.n * height;

        // The rings, inset in the plane. Every ring must succeed: half a tapered
        // face is not a shape, and reporting it is the whole point of the fold
        // check inside miter_inset().
        std::vector<glm::dvec2> outer_flat = project_ring(shape.geometry, face.loop, basis);
        std::vector<glm::dvec2> outer_inset;
        if (!miter_inset(outer_flat, height, outer_inset)) {
            return OpResult::failure("a taper of " + format_number(height) +
                                     " collapses the outline");
        }

        std::vector<std::vector<glm::dvec2>> hole_flats;
        std::vector<std::vector<glm::dvec2>> hole_insets;
        for (const auto& hole : face.holes) {
            if (hole.size() < 3) {
                continue;
            }
            std::vector<glm::dvec2> flat = project_ring(shape.geometry, hole, basis);
            std::vector<glm::dvec2> inset;
            if (!miter_inset(flat, height, inset)) {
                return OpResult::failure("a taper of " + format_number(height) +
                                         " collapses a hole in the outline");
            }
            hole_flats.push_back(std::move(flat));
            hole_insets.push_back(std::move(inset));
        }

        auto lift_ring = [&](const std::vector<glm::dvec2>& flat, bool raised) {
            std::vector<glm::dvec3> points;
            points.reserve(flat.size());
            for (const glm::dvec2& p : flat) {
                points.push_back(raised ? basis.unproject(p) + lift : basis.unproject(p));
            }
            return add_ring(out, points);
        };

        const std::vector<uint32_t> outer_bottom = lift_ring(outer_flat, false);
        const std::vector<uint32_t> outer_top = lift_ring(outer_inset, true);

        std::vector<std::vector<uint32_t>> hole_bottoms;
        std::vector<std::vector<uint32_t>> hole_tops;
        for (size_t h = 0; h < hole_flats.size(); ++h) {
            hole_bottoms.push_back(lift_ring(hole_flats[h], false));
            hole_tops.push_back(lift_ring(hole_insets[h], true));
        }

        Face cap_top;
        cap_top.material = face.material;
        cap_top.loop = outer_top;
        cap_top.holes = hole_tops;

        Face cap_bottom;
        cap_bottom.material = face.material;
        cap_bottom.loop = reversed(outer_bottom);
        for (const auto& hole : hole_bottoms) {
            cap_bottom.holes.push_back(reversed(hole));
        }

        out.faces.push_back(std::move(cap_bottom));
        out.faces.push_back(std::move(cap_top));

        // Same wall winding as build_prism(); see the note there.
        auto add_walls = [&](const std::vector<uint32_t>& bottom,
                             const std::vector<uint32_t>& top) {
            for (size_t i = 0; i < bottom.size(); ++i) {
                const size_t j = (i + 1) % bottom.size();
                Face wall;
                wall.material = face.material;
                wall.loop = {bottom[i], bottom[j], top[j], top[i]};
                out.faces.push_back(std::move(wall));
            }
        };
        add_walls(outer_bottom, outer_top);
        for (size_t h = 0; h < hole_bottoms.size(); ++h) {
            add_walls(hole_bottoms[h], hole_tops[h]);
        }
    }

    if (out.faces.empty()) {
        return OpResult::failure("no face has a usable plane to taper in");
    }

    shape.geometry = std::move(out);
    refit_scope(shape);
    return OpResult::success();
}

// ============================================================================
// Trigonometry
// ============================================================================

void deg_sin_cos(double degrees, double& sin_out, double& cos_out) {
    // Reduce to [0, 360) by integer arithmetic on the quarter-turn count where
    // the angle IS a whole number of quarter turns, so the common case never
    // reaches libm at all and is bit-identical everywhere.
    const double quarters = degrees / 90.0;
    const double rounded = std::nearbyint(quarters);
    if (std::fabs(quarters - rounded) <= 1e-12 && std::fabs(rounded) < 1e15) {
        auto quadrant = static_cast<int64_t>(rounded) % 4;
        if (quadrant < 0) {
            quadrant += 4;
        }
        switch (quadrant) {
            case 0: sin_out = 0.0;  cos_out = 1.0;  return;
            case 1: sin_out = 1.0;  cos_out = 0.0;  return;
            case 2: sin_out = 0.0;  cos_out = -1.0; return;
            default: sin_out = -1.0; cos_out = 0.0; return;
        }
    }
    const double radians = degrees * 0.017453292519943295769236907684886;
    sin_out = std::sin(radians);
    cos_out = std::cos(radians);
}

namespace {

/// Rotation about local x, then y, then z. Order is fixed; see rotate_shape().
[[nodiscard]] glm::dmat3 euler_matrix(const glm::dvec3& degrees) {
    double sx = 0.0;
    double cx = 1.0;
    double sy = 0.0;
    double cy = 1.0;
    double sz = 0.0;
    double cz = 1.0;
    deg_sin_cos(degrees.x, sx, cx);
    deg_sin_cos(degrees.y, sy, cy);
    deg_sin_cos(degrees.z, sz, cz);

    // glm is column-major: each triple below is a COLUMN.
    const glm::dmat3 rx{1.0, 0.0, 0.0, 0.0, cx, sx, 0.0, -sx, cx};
    const glm::dmat3 ry{cy, 0.0, -sy, 0.0, 1.0, 0.0, sy, 0.0, cy};
    const glm::dmat3 rz{cz, sz, 0.0, -sz, cz, 0.0, 0.0, 0.0, 1.0};
    return rz * ry * rx;
}

} // namespace

// ============================================================================
// Scope operations
// ============================================================================

void translate_shape(Shape& shape, const glm::dvec3& delta) {
    for (glm::dvec3& p : shape.geometry.positions) {
        p += delta;
    }
    if (shape.geometry.positions.empty()) {
        shape.scope.origin += shape.scope.axes * delta;
        return;
    }
    refit_scope(shape);
}

void rotate_shape(Shape& shape, const glm::dvec3& degrees) {
    const glm::dmat3 rotation = euler_matrix(degrees);
    const glm::dvec3 pivot = shape.scope.pivot_local();
    for (glm::dvec3& p : shape.geometry.positions) {
        p = pivot + rotation * (p - pivot);
    }
    // A rotation with a negative determinant would flip the winding, but an
    // Euler rotation never has one, so no face needs reversing here. mirror()
    // is the operation that does.
    refit_scope(shape);
}

void rotate_scope_shape(Shape& shape, const glm::dvec3& degrees) {
    const glm::dmat3 rotation = euler_matrix(degrees);
    // The new axes are the old ones turned. Turning the FRAME by R is the same
    // as turning everything in it by R inverse, so the geometry is re-expressed
    // rather than moved -- which is exactly what reframe() does.
    reframe(shape, shape.scope.axes * rotation);
}

OpResult scale_shape(Shape& shape, const glm::dvec3& new_size) {
    if (new_size.x < 0.0 || new_size.y < 0.0 || new_size.z < 0.0) {
        return OpResult::failure("a scope size cannot be negative");
    }

    glm::dvec3 factor{1.0};
    std::string skipped;
    for (int i = 0; i < 3; ++i) {
        const double current = shape.scope.size[i];
        if (current > kPointEpsilon) {
            factor[i] = new_size[i] / current;
            continue;
        }
        // A flat footprint has no thickness to multiply. Dividing anyway puts an
        // infinity in a coordinate, and the failure then surfaces as a missing
        // mesh several operations later with nothing pointing back here.
        if (!skipped.empty()) {
            skipped += ", ";
        }
        skipped += scope_axis_name(static_cast<ScopeAxis>(i));
    }

    const glm::dvec3 pivot = shape.scope.pivot_local();
    for (glm::dvec3& p : shape.geometry.positions) {
        p = pivot + (p - pivot) * factor;
    }
    refit_scope(shape);

    if (!skipped.empty()) {
        return OpResult::failure("the shape has no extent on " + skipped +
                                 ", so that axis was left alone");
    }
    return OpResult::success();
}

bool parse_pivot_anchor(const std::string& text, PivotAnchor& out) {
    if (text == "origin" || text == "min") { out = PivotAnchor::Origin; return true; }
    if (text == "center" || text == "centre") { out = PivotAnchor::Center; return true; }
    if (text == "max") { out = PivotAnchor::Max; return true; }
    if (text == "center_bottom" || text == "bottom") { out = PivotAnchor::CenterBottom; return true; }
    if (text == "center_top" || text == "top") { out = PivotAnchor::CenterTop; return true; }
    return false;
}

void set_pivot_shape(Shape& shape, PivotAnchor anchor) {
    switch (anchor) {
        case PivotAnchor::Origin: shape.scope.pivot = glm::dvec3{0.0}; return;
        case PivotAnchor::Center: shape.scope.pivot = glm::dvec3{0.5}; return;
        case PivotAnchor::Max: shape.scope.pivot = glm::dvec3{1.0}; return;
        case PivotAnchor::CenterBottom: shape.scope.pivot = glm::dvec3{0.5, 0.0, 0.5}; return;
        case PivotAnchor::CenterTop: shape.scope.pivot = glm::dvec3{0.5, 1.0, 0.5}; return;
    }
}

bool parse_align_mode(const std::string& text, AlignMode& out) {
    if (text == "world") { out = AlignMode::World; return true; }
    if (text == "y_up") { out = AlignMode::YUp; return true; }
    if (text == "geometry" || text == "face") { out = AlignMode::Geometry; return true; }
    return false;
}

OpResult align_scope_shape(Shape& shape, AlignMode mode) {
    switch (mode) {
        case AlignMode::World: {
            reframe(shape, glm::dmat3{1.0});
            return OpResult::success();
        }
        case AlignMode::YUp: {
            const glm::dvec3 up{0.0, 1.0, 0.0};
            const glm::dvec3 current_x = shape.scope.axes[0];
            const glm::dvec3 flat = safe_normalize(current_x - up * glm::dot(up, current_x));
            if (length_of(flat) <= 0.5) {
                return OpResult::failure(
                    "the scope's x axis is vertical, so there is no horizontal direction to keep");
            }
            reframe(shape, glm::dmat3{flat, up, glm::cross(flat, up)});
            return OpResult::success();
        }
        case AlignMode::Geometry: break;
    }

    const size_t index = dominant_face(shape.geometry);
    if (index == static_cast<size_t>(-1)) {
        return OpResult::failure("the shape has no face to align to");
    }
    const Face& face = shape.geometry.faces[index];
    const glm::dvec3 n = face_normal(shape.geometry, face);
    if (length_of(n) <= 0.5) {
        return OpResult::failure("the dominant face has no normal to align to");
    }

    // The longest edge of the dominant face gives x. Ties go to the lowest edge
    // index, so two edges of the same length cannot make the frame depend on
    // which one the triangulator happened to visit first.
    glm::dvec3 best_dir{0.0};
    double best_len = 0.0;
    for (size_t i = 0; i < face.loop.size(); ++i) {
        const glm::dvec3& a = shape.geometry.positions[face.loop[i]];
        const glm::dvec3& b = shape.geometry.positions[face.loop[(i + 1) % face.loop.size()]];
        const glm::dvec3 edge = b - a;
        const double len = length_of(edge);
        if (len > best_len + kPointEpsilon) {
            best_len = len;
            best_dir = edge;
        }
    }

    const glm::dvec3 x = safe_normalize(best_dir - n * glm::dot(n, best_dir));
    if (length_of(x) <= 0.5) {
        return OpResult::failure("the dominant face has no edge to align to");
    }
    reframe(shape, glm::dmat3{x, n, glm::cross(x, n)});
    return OpResult::success();
}

void mirror_shape(Shape& shape, ScopeAxis axis) {
    const int i = static_cast<int>(axis);
    const double plane = shape.scope.pivot_local()[i];
    for (glm::dvec3& p : shape.geometry.positions) {
        p[i] = 2.0 * plane - p[i];
    }
    // A reflection has determinant -1, so every face comes out inside in. Without
    // this the mirrored half of a symmetric building renders black and nothing
    // else about it looks wrong.
    for (Face& face : shape.geometry.faces) {
        reverse_face(face);
    }
    refit_scope(shape);
}

void mirror_scope_shape(Shape& shape, ScopeAxis axis) {
    glm::dmat3 axes = shape.scope.axes;
    axes[static_cast<int>(axis)] = -axes[static_cast<int>(axis)];
    reframe(shape, axes);
}

void reverse_normals_shape(Shape& shape) {
    for (Face& face : shape.geometry.faces) {
        reverse_face(face);
    }
}

// ============================================================================
// Output
// ============================================================================

std::vector<uint32_t> triangulate_face(const ShapeGeometry& geometry, const Face& face) {
    std::vector<uint32_t> out;
    if (face.loop.size() < 3) {
        return out;
    }
    const FaceBasis basis = face_basis(geometry, face);
    if (!basis.valid) {
        return out;
    }

    std::vector<std::vector<glm::dvec2>> polygon;
    std::vector<uint32_t> flat;

    polygon.push_back(project_ring(geometry, face.loop, basis));
    flat.insert(flat.end(), face.loop.begin(), face.loop.end());
    for (const auto& hole : face.holes) {
        if (hole.size() < 3) {
            continue;
        }
        polygon.push_back(project_ring(geometry, hole, basis));
        flat.insert(flat.end(), hole.begin(), hole.end());
    }

    const std::vector<uint32_t> tri = mapbox::earcut<uint32_t>(polygon);
    out.reserve(tri.size());
    for (const uint32_t local : tri) {
        if (local < flat.size()) {
            out.push_back(flat[local]);
        }
    }
    if (out.size() % 3 != 0) {
        out.clear();
    }
    return out;
}

void append_shape_to_mesh(const Shape& shape, Mesh& mesh) {
    for (const Face& face : shape.geometry.faces) {
        const std::vector<uint32_t> tri = triangulate_face(shape.geometry, face);
        if (tri.empty()) {
            continue;
        }
        const FaceBasis basis = face_basis(shape.geometry, face);
        const glm::vec3 normal = glm::vec3(shape.scope.axes * basis.n);

        // Vertices are duplicated per face, which gives flat shading. A wall
        // meets a roof at a crease; sharing a vertex between them would average
        // the two normals and round the crease off.
        std::map<uint32_t, uint32_t> remap;
        auto vertex_for = [&](uint32_t index) {
            const auto found = remap.find(index);
            if (found != remap.end()) {
                return found->second;
            }
            const glm::dvec3& local = shape.geometry.positions[index];
            Vertex vertex;
            vertex.position = glm::vec3(shape.scope.to_world(local));
            vertex.normal = normal;
            const glm::dvec2 uv = basis.project(local);
            vertex.uv = glm::vec2(static_cast<float>(uv.x), static_cast<float>(uv.y));
            const auto slot = static_cast<uint32_t>(mesh.vertices.size());
            mesh.vertices.push_back(vertex);
            remap.emplace(index, slot);
            return slot;
        };

        const auto range_start = static_cast<uint32_t>(mesh.indices.size());
        for (const uint32_t index : tri) {
            mesh.indices.push_back(vertex_for(index));
        }

        if (face.material.material != MaterialId::Default || face.material.variant != 0) {
            // Only a non-default material earns a submesh. renderer/mesh.hpp
            // defines an empty submesh list as one implicit Default range, so a
            // shape made entirely of default faces stays in the cheap form every
            // pre-existing consumer already handles.
            if (!mesh.submeshes.empty() &&
                mesh.submeshes.back().material == face.material.material &&
                mesh.submeshes.back().variant == face.material.variant &&
                mesh.submeshes.back().index_offset + mesh.submeshes.back().index_count ==
                    range_start) {
                mesh.submeshes.back().index_count +=
                    static_cast<uint32_t>(mesh.indices.size()) - range_start;
            } else {
                SubMesh sub;
                sub.index_offset = range_start;
                sub.index_count = static_cast<uint32_t>(mesh.indices.size()) - range_start;
                sub.material = face.material.material;
                sub.variant = face.material.variant;
                mesh.submeshes.push_back(sub);
            }
        }
    }
}

Mesh shape_to_mesh(const Shape& shape) {
    Mesh mesh;
    append_shape_to_mesh(shape, mesh);
    mesh.compute_bounds();
    mesh.compute_tangents();
    return mesh;
}

std::string dump_shape(const Shape& shape) {
    auto vec3 = [](const glm::dvec3& v) {
        return "(" + format_number(v.x) + " " + format_number(v.y) + " " + format_number(v.z) + ")";
    };

    std::string out = "shape " + (shape.rule.empty() ? std::string{"<root>"} : shape.rule);
    out += " depth=" + std::to_string(shape.depth);
    out += " index=" + std::to_string(shape.index) + "\n";
    out += "  origin=" + vec3(shape.scope.origin) + " size=" + vec3(shape.scope.size) +
           " pivot=" + vec3(shape.scope.pivot) + "\n";
    out += "  axes x=" + vec3(shape.scope.axes[0]) + " y=" + vec3(shape.scope.axes[1]) +
           " z=" + vec3(shape.scope.axes[2]) + "\n";

    for (const auto& entry : shape.attributes) {
        out += "  attr " + entry.first + "=" + entry.second.to_text() + "\n";
    }

    for (size_t i = 0; i < shape.geometry.faces.size(); ++i) {
        const Face& face = shape.geometry.faces[i];
        out += "  face " + std::to_string(i);
        out += " normal=" + vec3(face_normal(shape.geometry, face));
        out += " area=" + format_number(face_area(shape.geometry, face));
        out += "\n";
        for (const uint32_t index : face.loop) {
            out += "    " + vec3(shape.geometry.positions[index]) + "\n";
        }
        for (size_t h = 0; h < face.holes.size(); ++h) {
            out += "    hole " + std::to_string(h) + "\n";
            for (const uint32_t index : face.holes[h]) {
                out += "      " + vec3(shape.geometry.positions[index]) + "\n";
            }
        }
    }
    return out;
}

} // namespace stratum::procgen::rules
