// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_split.cpp
 * @brief The split solver and the slab cut. See op_split.hpp for the contract.
 */

#include "procgen/rules/op_split.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace stratum::procgen::rules {

namespace {

/**
 * @brief Most copies one repeat group may produce
 *
 * A guard, not a design limit: 4096 one-metre bays is a facade four kilometres
 * long, and no author writes that on purpose. What they DO write is a period
 * that came out near zero -- `repeat { ~width/100 : Bay(); }` with `width`
 * small -- and without a cap that is `floor(span / 1e-9)` pieces, which is an
 * out-of-memory kill before InterpreterLimits::max_shapes ever gets to refuse a
 * single shape. The cap is reported, so the author sees the mistake rather than
 * a facade that mysteriously stops.
 */
constexpr uint32_t kMaxRepeatCopies = 4096;

/// Marker in the compaction remap: this source vertex has not been kept yet
constexpr uint32_t kUnusedVertex = 0xFFFFFFFFu;

[[nodiscard]] double non_negative(double value) {
    // NaN falls through to 0 as well, which is the answer a size of NaN deserves:
    // it would otherwise poison every cursor after it and turn one bad expression
    // into a whole building of empty shapes with no clue where it started.
    return value > 0.0 ? value : 0.0;
}

/// Add a note unless an identical one is already there. Several repeat groups in
/// one split, each reporting the same fault about its children, would otherwise
/// say the same sentence once per group and bury everything else; the interpreter
/// reports a note at the SPLIT's location, so the copies would be indistinguishable.
void add_note(SplitLayout& layout, std::string note) {
    for (const std::string& existing : layout.notes) {
        if (existing == note) {
            return;
        }
    }
    layout.notes.push_back(std::move(note));
}

/// Is this part one of the two that absorb the remainder?
[[nodiscard]] bool is_floating(const SplitPart& part) {
    return !part.is_repeat &&
           (part.kind == SizeKind::Floating || part.kind == SizeKind::FloatingRelative);
}

// ============================================================================
// The slab cut
// ============================================================================

/**
 * @brief Lexicographic ordering on a position, so cut points can be welded by value
 *
 * Exact equality is the right test here and a tolerance would be the wrong one.
 * Slicer::shared_cut() guarantees that one edge crossing one plane yields one
 * identical double from every face that uses it, and a vertex duplicated by
 * shape.cpp's add_ring() is a bit-for-bit copy. So exact matching welds
 * everything that ought to be welded; a tolerance would additionally weld the
 * two sides of a thin mullion, and the cap would come out with the mullion
 * filled in.
 */
struct PositionLess {
    [[nodiscard]] bool operator()(const glm::dvec3& a, const glm::dvec3& b) const {
        if (a.x != b.x) {
            return a.x < b.x;
        }
        if (a.y != b.y) {
            return a.y < b.y;
        }
        return a.z < b.z;
    }
};

// Nothing normalises -0.0 to 0.0 on the way into that map, and it would be dead
// code if it did: the comparator is built out of != and <, and IEEE-754 makes
// -0.0 == 0.0, so the two are already the same key. A normalisation step here
// would be a line that no test could ever fail against, which is worse than no
// line at all. shape.hpp's format_number() normalises for a different reason --
// it PRINTS, and "-0" and "0" are different text.

/// Signed area of a ring projected onto the (u, v) plane. Positive means the
/// ring's normal points along the REMAINING axis, because (u, v, axis) with
/// u = axis+1 and v = axis+2 is a cyclic permutation of (x, y, z).
[[nodiscard]] double ring_signed_area(const std::vector<glm::dvec3>& positions,
                                      const std::vector<uint32_t>& ring,
                                      int u,
                                      int v) {
    double total = 0.0;
    const size_t n = ring.size();
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec3& a = positions[ring[i]];
        const glm::dvec3& b = positions[ring[(i + 1) % n]];
        total += a[u] * b[v] - b[u] * a[v];
    }
    return total * 0.5;
}

/// Crossing-number point-in-polygon in the (u, v) plane. Used only to tell a cap's
/// outer ring from the ring of a hole through it.
[[nodiscard]] bool point_in_ring(const glm::dvec3& point,
                                 const std::vector<glm::dvec3>& positions,
                                 const std::vector<uint32_t>& ring,
                                 int u,
                                 int v) {
    bool inside = false;
    const size_t n = ring.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const glm::dvec3& a = positions[ring[i]];
        const glm::dvec3& b = positions[ring[j]];
        if ((a[v] > point[v]) != (b[v] > point[v])) {
            const double span = b[v] - a[v];
            if (span != 0.0) {
                const double x = a[u] + (point[v] - a[v]) / span * (b[u] - a[u]);
                if (point[u] < x) {
                    inside = !inside;
                }
            }
        }
    }
    return inside;
}

/**
 * @brief Clips one geometry to a slab between two planes and caps the cut
 *
 * The state that makes it a class rather than four functions is the cut-point
 * cache: it has to be shared across every face, because two faces meeting at an
 * edge must be handed the same intersection vertex or the cap loop will not
 * close. See the header's determinism note.
 */
class Slicer {
public:
    Slicer(const ShapeGeometry& source, int axis, double lo, double hi)
        : source_(source), axis_(axis), lo_(lo), hi_(hi) {
        // Every source position is carried over and the unreferenced ones are
        // compacted away at the end. Clipping index-by-index rather than
        // point-by-point is what lets the cap loops be chained by index equality.
        out_.positions = source.positions;
    }

    void run() {
        clip_faces();
        wall_faces_ = out_.faces.size();
        replaced_.assign(wall_faces_, false);
        build_cap(lo_, false);
        build_cap(hi_, true);
        drop_replaced_walls();
        compact();
    }

    [[nodiscard]] ShapeGeometry take() { return std::move(out_); }

private:
    using CutKey = std::tuple<uint32_t, uint32_t, int>;

    [[nodiscard]] double coord(uint32_t index) const { return out_.positions[index][axis_]; }

    [[nodiscard]] bool inside(uint32_t index, double plane, bool keep_above) const {
        // A point exactly on the plane counts as inside for BOTH half-spaces, so
        // an edge that ends on the plane produces no intersection at all and the
        // ring keeps the vertex it already had. Treating it as outside would add
        // a duplicate vertex at the same position on every such edge, and every
        // one of those is a zero-length segment in the cap chaining.
        return keep_above ? coord(index) >= plane : coord(index) <= plane;
    }

    /**
     * @brief The vertex where an edge meets a plane, computed once per edge
     *
     * Keyed on the endpoint indices in ASCENDING order, so the two faces that
     * share the edge interpolate from the same end and get bit-identical
     * doubles. Interpolating from each face's own traversal direction gives two
     * points that differ in the last bit, and then the cap ring has a crack in it.
     */
    uint32_t shared_cut(uint32_t a, uint32_t b, double plane, int plane_id) {
        if (coord(a) == plane) {
            return a;
        }
        if (coord(b) == plane) {
            return b;
        }
        const uint32_t low = std::min(a, b);
        const uint32_t high = std::max(a, b);
        const CutKey key{low, high, plane_id};
        const auto found = cuts_.find(key);
        if (found != cuts_.end()) {
            return found->second;
        }

        const glm::dvec3 p0 = out_.positions[low];
        const glm::dvec3 p1 = out_.positions[high];
        const double c0 = p0[axis_];
        const double c1 = p1[axis_];
        glm::dvec3 point = p0;
        if (c1 != c0) {
            point = p0 + (p1 - p0) * ((plane - c0) / (c1 - c0));
        }
        // ASSIGNED, not interpolated. The cap search is an equality test against
        // the plane, and an interpolated coordinate lands a bit off it.
        point[axis_] = plane;

        const uint32_t index = static_cast<uint32_t>(out_.positions.size());
        out_.positions.push_back(point);
        cuts_.emplace(key, index);
        if (plane_id == 0) {
            crossed_lo_ = true;
        } else {
            crossed_hi_ = true;
        }
        return index;
    }

    /// Sutherland-Hodgman against one half-space, in vertex indices
    [[nodiscard]] std::vector<uint32_t> clip(const std::vector<uint32_t>& ring,
                                             double plane,
                                             bool keep_above,
                                             int plane_id) {
        std::vector<uint32_t> result;
        const size_t n = ring.size();
        if (n < 3) {
            return result;
        }
        result.reserve(n + 2);
        for (size_t i = 0; i < n; ++i) {
            const uint32_t current = ring[i];
            const uint32_t next = ring[(i + 1) % n];
            const bool current_in = inside(current, plane, keep_above);
            const bool next_in = inside(next, plane, keep_above);
            if (current_in) {
                result.push_back(current);
            }
            if (current_in != next_in) {
                result.push_back(shared_cut(current, next, plane, plane_id));
            }
        }
        dedupe(result);
        drop_reversals(result);
        return result;
    }

    /**
     * @brief Drop a vertex the ring doubles straight back over
     *
     * Sutherland-Hodgman on a NON-CONVEX ring whose boundary touches the clip
     * plane emits a zero-area appendage. The L-shaped footprint
     * {(0,0),(4,0),(4,2),(2,2),(2,4),(0,4)} clipped to x >= 2 comes back as
     * (2,0),(4,0),(4,2),(2,2),(2,4): the correct rectangle, plus a tail that
     * runs out to (2,4) along the plane and straight back.
     *
     * The tail encloses nothing, so every area and every volume it appears in is
     * still right and nothing downstream reports a fault. What it does instead is
     * two silent things. It puts a vertex the slab does not contain into the
     * position list, and refit_scope() derives the scope from the POSITIONS, so
     * the slab claims an extent it does not occupy -- the exact staleness
     * shape.hpp's second invariant exists to prevent. And it hands build_cap() a
     * segment that runs the whole width of the tail, so the cap ring closes
     * around the wrong cross-section.
     *
     * The test is EXACT and it is a REVERSAL, not mere collinearity: the cross
     * product must be exactly zero and the two edges must point opposite ways. A
     * vertex that is merely collinear -- the (2,2) that the same clip leaves in
     * the middle of the other slab's edge -- is kept, because another face may
     * meet the ring there and dropping it would open a crack at a T-vertex.
     */
    void drop_reversals(std::vector<uint32_t>& ring) const {
        bool changed = true;
        while (changed && ring.size() >= 3) {
            changed = false;
            const size_t n = ring.size();
            for (size_t i = 0; i < n; ++i) {
                const glm::dvec3& before = out_.positions[ring[(i + n - 1) % n]];
                const glm::dvec3& here = out_.positions[ring[i]];
                const glm::dvec3& after = out_.positions[ring[(i + 1) % n]];
                const glm::dvec3 incoming = here - before;
                const glm::dvec3 outgoing = after - here;
                if (glm::cross(incoming, outgoing) == glm::dvec3{0.0} &&
                    glm::dot(incoming, outgoing) < 0.0) {
                    ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(i));
                    changed = true;
                    break;
                }
            }
        }
    }

    /// Drop consecutive repeats, wrap-around included. A ring that collapses to
    /// fewer than three distinct vertices is dropped by the caller.
    static void dedupe(std::vector<uint32_t>& ring) {
        std::vector<uint32_t> out;
        out.reserve(ring.size());
        for (const uint32_t index : ring) {
            if (!out.empty() && out.back() == index) {
                continue;
            }
            out.push_back(index);
        }
        while (out.size() > 1 && out.front() == out.back()) {
            out.pop_back();
        }
        ring = std::move(out);
    }

    void clip_faces() {
        for (const Face& face : source_.faces) {
            std::vector<uint32_t> loop = clip(face.loop, lo_, true, 0);
            loop = clip(loop, hi_, false, 1);
            if (loop.size() < 3) {
                continue;
            }

            Face kept;
            kept.material = face.material;
            kept.loop = std::move(loop);
            for (const std::vector<uint32_t>& hole : face.holes) {
                // A hole is clipped exactly as the outline is. One wholly outside
                // the slab disappears; one wholly inside comes through untouched;
                // one that STRADDLES a plane comes through as a ring that now
                // touches the outline along the cut.
                //
                // That last case is the set-correct answer in a non-canonical
                // form: a notch in the outline expressed as a hole against it. It
                // is kept that way rather than merged into the outline because
                // face_area() subtracts the hole and therefore reports the right
                // number either way, while a merge is a boolean operation on two
                // rings and a wrong merge emits a self-intersecting face that
                // looks right until it is exported. Dropping the hole instead
                // would fill the notch in solid, which is the one answer that is
                // visibly wrong.
                std::vector<uint32_t> cut_hole = clip(hole, lo_, true, 0);
                cut_hole = clip(cut_hole, hi_, false, 1);
                if (cut_hole.size() >= 3) {
                    kept.holes.push_back(std::move(cut_hole));
                }
            }
            out_.faces.push_back(std::move(kept));
        }
    }

    struct Segment {
        uint32_t from = 0;
        uint32_t to = 0;
    };

    /**
     * @brief Split a cap segment wherever another segment ENDS inside it
     *
     * The chaining walks head to tail and can only turn off a segment at its
     * ends, so a segment that runs past the end of another one is a wall the
     * chain cannot get through. That is exactly what a hole reaching the cut
     * plane produces: the outline of a courtyard block clipped at x = 1 hands
     * over one segment spanning the whole four metres of the cross-section,
     * while the well's own ring hands over the two metres in the middle of it.
     * Split the long one at the short one's ends and the two retraced metres can
     * cancel, leaving the two solid strips the cap really is.
     *
     * Exact arithmetic, no tolerance: the split point must be exactly on the
     * segment. An architectural cut is axis-aligned and its coordinates are
     * assigned rather than interpolated (see shared_cut()), so the test lands
     * exactly; a tolerance would instead invent a split on any segment that
     * happened to pass near a vertex, and the chain would then turn a corner
     * that is not there.
     */
    void split_at_touching_ends(std::vector<Segment>& segments) const {
        std::vector<uint32_t> ends;
        ends.reserve(segments.size() * 2);
        for (const Segment& segment : segments) {
            ends.push_back(segment.from);
            ends.push_back(segment.to);
        }
        std::sort(ends.begin(), ends.end());
        ends.erase(std::unique(ends.begin(), ends.end()), ends.end());

        std::vector<Segment> split;
        split.reserve(segments.size());
        std::vector<std::pair<double, uint32_t>> inside;
        for (const Segment& segment : segments) {
            const glm::dvec3 from = out_.positions[segment.from];
            const glm::dvec3 along = out_.positions[segment.to] - from;
            const double length2 = glm::dot(along, along);
            inside.clear();
            for (const uint32_t end : ends) {
                if (end == segment.from || end == segment.to || length2 <= 0.0) {
                    continue;
                }
                const glm::dvec3 offset = out_.positions[end] - from;
                if (glm::cross(along, offset) != glm::dvec3{0.0}) {
                    continue;
                }
                const double t = glm::dot(offset, along);
                if (t > 0.0 && t < length2) {
                    inside.emplace_back(t, end);
                }
            }
            // By distance along the segment, and no two distinct points on one
            // line share a distance, so the order is total and reproducible.
            std::sort(inside.begin(), inside.end());
            uint32_t cursor = segment.from;
            for (const std::pair<double, uint32_t>& point : inside) {
                split.push_back(Segment{cursor, point.second});
                cursor = point.second;
            }
            split.push_back(Segment{cursor, segment.to});
        }
        segments = std::move(split);
    }

    /**
     * @brief Drop pairs of cap segments that retrace each other
     *
     * A segment and its exact reverse bound a strip of no width: the surface
     * folds back on itself along the plane and there is nothing between the two.
     * That is what an outline and the ring of a hole touching it come to after
     * the split above, and chaining them would close a ring around a region the
     * slab's solid does not occupy.
     *
     * The budget is computed before anything is dropped and spent in segment
     * order, so three copies of an edge against two of its reverse leave one
     * copy of the edge -- and leave the same one on every run.
     */
    void cancel_retraced(std::vector<Segment>& segments) const {
        std::map<std::pair<uint32_t, uint32_t>, int> count;
        for (const Segment& segment : segments) {
            ++count[{segment.from, segment.to}];
        }
        std::map<std::pair<uint32_t, uint32_t>, int> budget;
        for (const std::pair<const std::pair<uint32_t, uint32_t>, int>& entry : count) {
            const auto reverse = count.find({entry.first.second, entry.first.first});
            if (reverse != count.end()) {
                budget[entry.first] = std::min(entry.second, reverse->second);
            }
        }
        std::vector<Segment> kept;
        kept.reserve(segments.size());
        for (const Segment& segment : segments) {
            const auto spend = budget.find({segment.from, segment.to});
            if (spend != budget.end() && spend->second > 0) {
                --spend->second;
                continue;
            }
            kept.push_back(segment);
        }
        segments = std::move(kept);
    }

    /**
     * @brief Close the shape where the plane cut it
     *
     * Only when the plane actually CROSSED an edge. A plane that merely touches
     * the shape's boundary -- which is exactly what happens at the two ends of a
     * split, where the first piece begins at the box minimum and the last ends at
     * the maximum -- has a face lying in it already, and capping there would
     * emit a second copy of that face and double its area.
     *
     * ### A face that lies IN a plane the cut crossed is replaced, not kept
     *
     * "Crossed somewhere" and "lies in the plane here" are not exclusive, and a
     * non-convex shape reaches both at once: cut the L-shaped footprint above at
     * x = 2 and the plane slices the base while CONTAINING the wall of the upper
     * arm. That wall is kept by the clip -- a point on the plane is inside both
     * half-spaces, so it survives into BOTH slabs -- and the cap is then laid
     * over the top of it in the near slab and left behind as a stray face in the
     * far one. The measured result before this was handled: an L of volume 12
     * split into slabs of 9.33 and 4, neither of them closed.
     *
     * So a wall lying entirely in a plane this cap covers is dropped and the cap
     * stands in its place. The cap is the authoritative surface there: it is
     * chained from the cut cross-section, so it covers exactly the part of the
     * plane the slab's solid reaches, which is the whole of the coplanar wall in
     * the slab that owns it and none of it in the slab that does not.
     *
     * The drop happens only when a cap is actually emitted, so the two ends of an
     * ordinary split -- where nothing crossed and no cap is built -- keep the
     * faces they came with.
     */
    void build_cap(double plane, bool far_side) {
        if (far_side ? !crossed_hi_ : !crossed_lo_) {
            return;
        }

        // Weld by position so that a source geometry with duplicated coincident
        // vertices -- which shape.cpp's add_ring() produces freely -- still
        // chains into closed rings.
        std::map<glm::dvec3, uint32_t, PositionLess> canonical;
        const auto weld = [&](uint32_t index) {
            const glm::dvec3& key = out_.positions[index];
            const auto found = canonical.find(key);
            if (found != canonical.end()) {
                return found->second;
            }
            canonical.emplace(key, index);
            return index;
        };

        std::vector<Segment> segments;
        std::vector<size_t> coplanar;
        for (size_t f = 0; f < wall_faces_; ++f) {
            if (replaced_[f]) {
                continue;  // already stood in for by the other plane's cap
            }
            const std::vector<uint32_t>& loop = out_.faces[f].loop;
            bool all_on_plane = true;
            for (const uint32_t index : loop) {
                if (coord(index) != plane) {
                    all_on_plane = false;
                    break;
                }
            }
            if (all_on_plane) {
                // This face IS in the cut plane; it is not a wall crossing it. Its
                // edges must not become cap segments -- they would close the ring
                // around the whole cross-section instead of around the cut -- and
                // the face itself is what the cap replaces. See the note above.
                coplanar.push_back(f);
                continue;
            }
            const auto harvest = [&](const std::vector<uint32_t>& ring) {
                const size_t n = ring.size();
                for (size_t i = 0; i < n; ++i) {
                    const uint32_t from = ring[i];
                    const uint32_t to = ring[(i + 1) % n];
                    if (coord(from) == plane && coord(to) == plane) {
                        segments.push_back(Segment{weld(from), weld(to)});
                    }
                }
            };
            harvest(loop);
            for (const std::vector<uint32_t>& hole : out_.faces[f].holes) {
                // A HOLE bounds the surface exactly as the outline does, so where
                // it reaches the cut plane it bounds the cap too. Cutting a
                // courtyard block across its light well is the ordinary case:
                // without this the cap roofs the well over, and the slab comes
                // back sealed, heavier than the solid it was cut from, and open
                // at the seam -- measured at 82 of surface against a true 70.
                harvest(hole);
            }
        }
        split_at_touching_ends(segments);
        cancel_retraced(segments);
        if (segments.size() < 3) {
            return;
        }

        // Chain head to tail. Segments are visited in face order and the next one
        // is always the lowest unused index, so the rings come out the same on
        // every run and on every platform.
        std::map<uint32_t, std::vector<size_t>> by_start;
        for (size_t i = 0; i < segments.size(); ++i) {
            by_start[segments[i].from].push_back(i);
        }
        std::vector<bool> used(segments.size(), false);
        std::vector<std::vector<uint32_t>> loops;
        for (size_t i = 0; i < segments.size(); ++i) {
            if (used[i] || segments[i].from == segments[i].to) {
                used[i] = true;
                continue;
            }
            used[i] = true;
            std::vector<uint32_t> ring{segments[i].from};
            uint32_t current = segments[i].to;
            bool closed = false;
            while (ring.size() <= segments.size()) {
                if (current == ring.front()) {
                    closed = true;
                    break;
                }
                ring.push_back(current);
                size_t next = kNoPart;
                const auto candidates = by_start.find(current);
                if (candidates != by_start.end()) {
                    for (const size_t candidate : candidates->second) {
                        if (!used[candidate]) {
                            next = candidate;
                            break;
                        }
                    }
                }
                if (next == kNoPart) {
                    break;  // an open chain: the cut did not close, so it is not a cap
                }
                used[next] = true;
                current = segments[next].to;
            }
            if (closed && ring.size() >= 3) {
                loops.push_back(std::move(ring));
            }
        }
        if (loops.empty()) {
            return;
        }

        const int u = (axis_ + 1) % 3;
        const int v = (axis_ + 2) % 3;
        std::vector<double> areas(loops.size(), 0.0);
        for (size_t i = 0; i < loops.size(); ++i) {
            areas[i] = ring_signed_area(out_.positions, loops[i], u, v);
        }

        // Nesting by parity: a ring inside an odd number of other rings is a hole,
        // a ring inside an even number is an outline. That is the only rule that
        // survives a cut through a hollow column, where the bore's ring is inside
        // the outer ring and anything inside the bore is solid again.
        std::vector<size_t> depth(loops.size(), 0);
        std::vector<size_t> owner(loops.size(), kNoPart);
        for (size_t i = 0; i < loops.size(); ++i) {
            const glm::dvec3& probe = out_.positions[loops[i][0]];
            double smallest = 0.0;
            for (size_t j = 0; j < loops.size(); ++j) {
                if (i == j || !point_in_ring(probe, out_.positions, loops[j], u, v)) {
                    continue;
                }
                ++depth[i];
                const double area = std::fabs(areas[j]);
                if (owner[i] == kNoPart || area < smallest) {
                    owner[i] = j;
                    smallest = area;
                }
            }
        }

        // The near plane's cap faces back along the axis, the far plane's faces
        // along it -- outward for the slab in both cases.
        const bool want_positive = far_side;
        const MaterialKey material =
            source_.faces.empty() ? MaterialKey{} : source_.faces.front().material;

        // `areas[i] == 0.0` is a guard, not a case. A loop that encloses nothing
        // would be a face with no area, no normal and no triangles in it. It is
        // also, against every input that could be constructed for it, unreachable:
        // a chain of collinear segments is taken apart by split_at_touching_ends()
        // and cancel_retraced() before it can close, so what got here would have to
        // be a ring crossing itself into two lobes of exactly equal and opposite
        // area. It is kept rather than deleted because the alternative to a line no
        // test can reach is a degenerate face in the mesh on the day one does.
        std::map<size_t, size_t> cap_of_loop;  // loop index -> index into out_.faces
        for (size_t i = 0; i < loops.size(); ++i) {
            if (depth[i] % 2 != 0 || areas[i] == 0.0) {
                continue;
            }
            Face cap;
            cap.material = material;
            cap.loop = loops[i];
            if ((areas[i] > 0.0) != want_positive) {
                std::reverse(cap.loop.begin(), cap.loop.end());
            }
            cap_of_loop.emplace(i, out_.faces.size());
            out_.faces.push_back(std::move(cap));
        }
        for (size_t i = 0; i < loops.size(); ++i) {
            if (depth[i] % 2 == 0 || areas[i] == 0.0 || owner[i] == kNoPart) {
                continue;
            }
            const auto cap = cap_of_loop.find(owner[i]);
            if (cap == cap_of_loop.end()) {
                continue;
            }
            std::vector<uint32_t> ring = loops[i];
            if ((areas[i] > 0.0) == want_positive) {
                std::reverse(ring.begin(), ring.end());
            }
            out_.faces[cap->second].holes.push_back(std::move(ring));
        }

        // Only now, with a cap on the plane, does the face that was already lying
        // in it become a duplicate of part of that cap.
        if (!cap_of_loop.empty()) {
            for (const size_t face : coplanar) {
                replaced_[face] = true;
            }
        }
    }

    /// Erase the walls a cap stood in for. Done once, after both caps, so that
    /// the second cap's harvesting sees the same face indices the first did.
    void drop_replaced_walls() {
        std::vector<Face> kept;
        kept.reserve(out_.faces.size());
        for (size_t f = 0; f < out_.faces.size(); ++f) {
            if (f < replaced_.size() && replaced_[f]) {
                continue;
            }
            kept.push_back(std::move(out_.faces[f]));
        }
        out_.faces = std::move(kept);
    }

    /**
     * @brief Drop the positions no face refers to
     *
     * Not an optimisation. shape.hpp's refit_scope() derives the scope from
     * geometry_bounds(), which reads the POSITION LIST and not the faces, so a
     * slab that carried the whole parent's vertex buffer would get the whole
     * parent's box -- every piece of a split reporting the extent of the thing it
     * was cut out of.
     */
    void compact() {
        std::vector<uint32_t> remap(out_.positions.size(), kUnusedVertex);
        std::vector<glm::dvec3> kept;
        const auto map_ring = [&](std::vector<uint32_t>& ring) {
            for (uint32_t& index : ring) {
                if (remap[index] == kUnusedVertex) {
                    remap[index] = static_cast<uint32_t>(kept.size());
                    kept.push_back(out_.positions[index]);
                }
                index = remap[index];
            }
        };
        for (Face& face : out_.faces) {
            map_ring(face.loop);
            for (std::vector<uint32_t>& hole : face.holes) {
                map_ring(hole);
            }
        }
        out_.positions = std::move(kept);
    }

    const ShapeGeometry& source_;
    int axis_ = 0;
    double lo_ = 0.0;
    double hi_ = 0.0;
    ShapeGeometry out_;
    std::map<CutKey, uint32_t> cuts_;
    size_t wall_faces_ = 0;

    /// One flag per clipped wall face: a cap covers this face's plane, so the cap
    /// is the surface there and this face is a duplicate of part of it.
    std::vector<bool> replaced_;

    bool crossed_lo_ = false;
    bool crossed_hi_ = false;
};

// ============================================================================
// Repetition
// ============================================================================

void expand_repeat(SplitLayout& layout,
                   const SplitPart& group,
                   size_t part_index,
                   double begin,
                   double span,
                   double extent);

}  // namespace

// ============================================================================
// Sizing
// ============================================================================

double nominal_size(const SplitPart& part, double extent) {
    if (part.is_repeat) {
        double period = 0.0;
        for (const SplitPart& child : part.children) {
            if (child.is_repeat) {
                continue;  // the parser rejects a nested repeat; ignored rather than trusted
            }
            period += nominal_size(child, extent);
        }
        return period;
    }

    const double value = non_negative(part.value);
    const double span = non_negative(extent);
    switch (part.kind) {
        case SizeKind::Absolute:
        case SizeKind::Floating:
            return value;
        case SizeKind::Relative:
        case SizeKind::FloatingRelative:
            // value * span / 100, not value * 0.01 * span: 25% of 10 is then
            // exactly 2.5 rather than 2.5 to within a rounding of 0.01.
            return value * span / 100.0;
    }
    return 0.0;
}

SplitLayout solve_split(const std::vector<SplitPart>& parts, double extent) {
    SplitLayout layout;
    layout.extent = non_negative(extent);

    if (parts.empty()) {
        add_note(layout, "this split has no parts, so it produces nothing");
        return layout;
    }
    if (!(layout.extent > 0.0)) {
        // Not a refusal. The pieces are still produced, all of them empty, so the
        // caller emits one diagnostic and the rule still runs; refusing would make
        // `split(y)` of a footprint look exactly like a rule that was never called.
        add_note(layout,
                 "the scope has no extent along the split axis, so every part of this split "
                 "is empty");
    }

    // Pass one: what is fixed, what is elastic, and what does each part want?
    std::vector<double> nominal(parts.size(), 0.0);
    double fixed = 0.0;
    bool negative_seen = false;
    for (size_t i = 0; i < parts.size(); ++i) {
        const SplitPart& part = parts[i];
        if (!part.is_repeat && part.value < 0.0) {
            negative_seen = true;
        }
        nominal[i] = nominal_size(part, layout.extent);
        if (part.is_repeat || is_floating(part)) {
            layout.elastic_weight += nominal[i];
        } else {
            fixed += nominal[i];
        }
    }
    if (negative_seen) {
        add_note(layout,
                 "a part of this split asks for a negative size, and a negative size counts "
                 "as zero");
    }

    double remainder = layout.extent - fixed;
    if (remainder < 0.0) {
        layout.overflow = -remainder;
        remainder = 0.0;
        // Said, not silently absorbed. Scaling every part down to fit would turn
        // the author's 3.0 into 2.4 with nothing on screen to say so.
        add_note(layout, "the fixed parts of this split ask for " + format_number(fixed) +
                             " of an extent of " + format_number(layout.extent) +
                             ", so the surplus is cut off at the far end");
    }

    // Pass two: lay them out in order.
    double cursor = 0.0;
    for (size_t i = 0; i < parts.size(); ++i) {
        const SplitPart& part = parts[i];
        double length = nominal[i];
        if (part.is_repeat || is_floating(part)) {
            length = layout.elastic_weight > 0.0
                         ? remainder * nominal[i] / layout.elastic_weight
                         : 0.0;
        }

        // Clipped at the far end rather than allowed to run past it. Whatever was
        // lost is already counted in layout.overflow and reported above.
        const double available = layout.extent - cursor;
        if (length > available) {
            length = available > 0.0 ? available : 0.0;
        }

        if (part.is_repeat) {
            expand_repeat(layout, part, i, cursor, length, layout.extent);
        } else {
            SplitPiece piece;
            piece.begin = cursor;
            piece.length = length;
            piece.part = i;
            layout.pieces.push_back(piece);
        }
        cursor += length;
    }
    layout.used = cursor;
    return layout;
}

namespace {

/**
 * @brief The children of a repeat group at the sizes its PERIOD was built from
 *
 * Every child becomes an Absolute part of exactly the length nominal_size()
 * summed into the period, so that laying the list out over the period covers it
 * exactly -- by construction, not by hope. That is the property the copy's scale
 * divides by, and the property the old code did not have: nominal_size() resolves
 * a relative child against the SPLIT's extent, while solving the same child
 * against the period resolves it against the period, and the two disagree by
 * exactly the factor that was left as a gap.
 *
 * It also stops the copy inventing a diagnostic of its own. `repeat { 200% :
 * Bay(); }` has a period of twice the extent, and re-solving `200%` against that
 * period asks for four times the extent inside a two-extent copy -- an overflow
 * the author never wrote, reported at the split's own location.
 */
[[nodiscard]] std::vector<SplitPart> natural_children(const SplitPart& group, double extent) {
    std::vector<SplitPart> natural;
    natural.reserve(group.children.size());
    for (const SplitPart& child : group.children) {
        SplitPart part = child;
        if (!child.is_repeat) {
            part.kind = SizeKind::Absolute;
            // The NEGATIVE value is carried through as the author wrote it rather
            // than clamped. nominal_size() would hand back a clean zero, and then
            // solve_split() would have nothing left to notice and the author's
            // mistake would vanish on the way into the copy. It still lays out as
            // zero -- non_negative() sees to that -- and it still says so.
            part.value = child.value < 0.0 ? child.value : nominal_size(child, extent);
        }
        natural.push_back(std::move(part));
    }
    return natural;
}

void expand_repeat(SplitLayout& layout,
                   const SplitPart& group,
                   size_t part_index,
                   double begin,
                   double span,
                   double extent) {
    // The period is checked BEFORE the span, and the order matters. A group whose
    // children have no size between them gets an elastic weight of zero, so it is
    // also handed a span of zero -- and reporting only "it had no room" would name
    // the symptom while the author's mistake was writing `repeat { 0 : Bay(); }`.
    const double period = nominal_size(group, extent);
    if (!(period > 0.0)) {
        add_note(layout,
                 "a repeat group in this split has no size of its own, so it cannot tile and "
                 "produces nothing");
        return;
    }

    if (!(span > 0.0)) {
        // No room at all. Emitting a zero-length copy would put an empty bay in
        // the shape tree for a group the split had no space for.
        //
        // This used to return silently in EVERY case, on the reasoning that the
        // overflow note already said why there was no room. It does -- when the
        // split overflows. It says nothing when the fixed parts fit EXACTLY, and
        // that is the likelier mistake by far: `extrude(5.0)` with
        // `split(y) { 5.0 : Shopfront(); repeat { 3.0 : Floor(); } }` leaves the
        // repeat precisely zero, produces one quad, and reported nothing at all.
        // The author sees a flat wall and has no thread to pull.
        //
        // So the note is added only when there was no overflow to explain it,
        // which keeps the overflow case at one note rather than two saying the
        // same thing.
        if (!(layout.overflow > 0.0)) {
            add_note(layout,
                     "a repeat group in this split was left no room: the fixed parts "
                     "use the whole extent of " +
                         format_number(extent) + ", so the repeat produces nothing");
        }
        return;
    }

    // As many WHOLE copies as fit, and never fewer than one. See the header for
    // why a group that does not fit even once is compressed rather than dropped.
    const double whole = std::floor(span / period);
    uint32_t copies = 1;
    if (whole >= 1.0) {
        copies = whole >= static_cast<double>(kMaxRepeatCopies)
                     ? kMaxRepeatCopies
                     : static_cast<uint32_t>(whole);
        if (whole > static_cast<double>(kMaxRepeatCopies)) {
            add_note(layout, "a repeat group in this split asks for more than " +
                                 format_number(static_cast<double>(kMaxRepeatCopies)) +
                                 " copies, so it is capped there");
        }
    } else {
        layout.compressed = true;
        add_note(layout,
                 "a repeat group is wider than the space it was given, so it holds one "
                 "compressed copy rather than none");
    }

    // The leftover goes INTO the copies: every copy is the same length and no gap
    // opens. This is the decision the header argues for at length.
    const double copy_length = span / static_cast<double>(copies);

    // Solved ONCE, not once per copy: every copy has the same length, so every
    // copy has the same internal layout and only its offset differs. That is also
    // the property the tests assert -- the bays are the same width as each other.
    SplitLayout inner = solve_split(group.children, copy_length);
    double scale = 1.0;
    if (!(inner.elastic_weight > 0.0)) {
        // Nothing inside the copy can absorb the stretch, so the copy is laid out
        // at its NATURAL size and then scaled as a whole. Solving against
        // copy_length instead would leave a gap at the end of every copy when
        // stretching, and would TRUNCATE the last child when compressing -- a bay
        // that vanishes instead of a bay that narrows.
        //
        // The natural size is natural_children()'s and not the children's own. A
        // relative child resolves against whatever extent it is solved at, so
        // solving `repeat { 50% : Bay(); }` against its own period of 2 produced a
        // bay of 1 in a copy of 2 and then scaled it by copy_length / period == 1.
        // The group covered half the facade with no overflow, no note, and a
        // layout.used that reported the whole extent.
        //
        // natural_children() lays the children out at the very lengths the period
        // was summed from, so the natural solve covers the period exactly. The
        // scale is written against what that solve PRODUCED rather than against
        // the period it was asked for: the two are equal by construction, and
        // dividing by the number the pieces actually came from is what keeps them
        // equal if either side of it is ever changed.
        inner = solve_split(natural_children(group, extent), period);
        scale = inner.used > 0.0 ? copy_length / inner.used : 1.0;
    }
    for (const std::string& note : inner.notes) {
        add_note(layout, note);
    }

    for (uint32_t copy = 0; copy < copies; ++copy) {
        const double copy_begin = begin + static_cast<double>(copy) * copy_length;
        for (const SplitPiece& sub : inner.pieces) {
            SplitPiece piece;
            piece.begin = copy_begin + sub.begin * scale;
            piece.length = sub.length * scale;
            piece.part = part_index;
            piece.child = sub.part;
            piece.copy = copy;
            layout.pieces.push_back(piece);
        }
        ++layout.repeat_copies;
    }
}

}  // namespace

// ============================================================================
// The cut
// ============================================================================

ScopeAxis to_scope_axis(SplitAxis axis) {
    // shape.hpp promises these two enums agree; this is where the promise is kept
    // rather than assumed, because a mismatch splits a facade along the wrong
    // axis and every geometric assertion downstream still passes.
    static_assert(static_cast<uint8_t>(SplitAxis::X) == static_cast<uint8_t>(ScopeAxis::X));
    static_assert(static_cast<uint8_t>(SplitAxis::Y) == static_cast<uint8_t>(ScopeAxis::Y));
    static_assert(static_cast<uint8_t>(SplitAxis::Z) == static_cast<uint8_t>(ScopeAxis::Z));
    return static_cast<ScopeAxis>(static_cast<uint8_t>(axis));
}

Shape slice_shape(const Shape& shape, ScopeAxis axis, double begin, double end) {
    Shape out = shape;
    out.geometry.clear();

    const int index = static_cast<int>(axis);
    if (end > begin && !shape.geometry.empty()) {
        Slicer slicer(shape.geometry, index, begin, end);
        slicer.run();
        out.geometry = slicer.take();
    }

    out.scope = shape.scope;
    if (out.geometry.faces.empty()) {
        // No geometry in this slab. shape.hpp's documented exception to the
        // tight-box invariant applies: the scope is whatever was set, so the slab
        // itself is set, and D4 can insert an asset into the gap a split left.
        out.geometry.clear();
        glm::dvec3 corner{0.0};
        corner[index] = begin;
        out.scope.origin = shape.scope.to_world(corner);
        out.scope.size = shape.scope.size;
        out.scope.size[index] = end > begin ? end - begin : 0.0;
        out.scope.size = glm::max(out.scope.size, glm::dvec3{0.0});
        return out;
    }

    // With geometry, the box is tight -- invariant 2, the same as every operation
    // in shape.cpp. The slab's own width is NOT forced onto the scope: a piece
    // whose geometry does not fill it reports the extent it really has, and a
    // scope that claims an extent its geometry does not occupy is exactly the
    // staleness shape.hpp's invariant exists to prevent.
    refit_scope(out);
    return out;
}

std::vector<Shape> split_shape(const Shape& shape, ScopeAxis axis, const SplitLayout& layout) {
    std::vector<Shape> slabs;
    slabs.reserve(layout.pieces.size());
    for (const SplitPiece& piece : layout.pieces) {
        slabs.push_back(slice_shape(shape, axis, piece.begin, piece.begin + piece.length));
    }
    return slabs;
}

// ============================================================================
// The AST bridge
// ============================================================================

std::vector<SplitPart> split_parts_from_statement(const SplitStmt& stmt,
                                                  const SplitSizeEvaluator& evaluate) {
    std::vector<SplitPart> parts;
    parts.reserve(stmt.entries.size());
    for (const SplitEntry& entry : stmt.entries) {
        SplitPart part;
        part.is_repeat = entry.is_repeat;
        if (entry.is_repeat) {
            part.children.reserve(entry.children.size());
            for (const SplitEntry& child : entry.children) {
                SplitPart inner;
                inner.is_repeat = child.is_repeat;
                if (!child.is_repeat) {
                    inner.kind = child.size.kind;
                    inner.value = evaluate ? evaluate(child.size.value, child.size.loc) : 0.0;
                }
                // A repeat entry has NO size expression, so a nested one -- which
                // the parser rejects, and which any caller assembling a SplitStmt
                // of its own can still hand over -- would otherwise have kNoNode
                // evaluated as its size. The interpreter's evaluator reports that
                // as "a split size is not a number" against a SourceLoc of nothing,
                // naming a line the author never wrote. Carried across as a repeat
                // instead, nominal_size() skips it exactly as it skips one in the
                // parts it is given, and solve_split() says it cannot tile.
                part.children.push_back(std::move(inner));
            }
        } else {
            part.kind = entry.size.kind;
            part.value = evaluate ? evaluate(entry.size.value, entry.size.loc) : 0.0;
        }
        parts.push_back(std::move(part));
    }
    return parts;
}

const SplitEntry* split_entry_for(const SplitStmt& stmt, const SplitPiece& piece) {
    if (piece.part >= stmt.entries.size()) {
        return nullptr;
    }
    const SplitEntry& entry = stmt.entries[piece.part];
    if (!entry.is_repeat) {
        // A piece that names a child of a part that has none is a layout built
        // from a different statement, and answering it would run the wrong rule.
        return piece.child == kNoPart ? &entry : nullptr;
    }
    if (piece.child >= entry.children.size()) {
        return nullptr;
    }
    return &entry.children[piece.child];
}

StmtId split_body_for(const SplitStmt& stmt, const SplitPiece& piece) {
    const SplitEntry* entry = split_entry_for(stmt, piece);
    return entry == nullptr ? kNoNode : entry->body;
}

} // namespace stratum::procgen::rules
