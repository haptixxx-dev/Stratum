/**
 * @file sidewalk_dedup.cpp
 * @brief Implementation of the doubly-mapped sidewalk query
 *
 * The shape of the problem is all-pairs: every footway edge against every
 * carriageway edge. A city extract carries tens of thousands of each, so the
 * quadratic form is not an option and the whole file is built around avoiding
 * it.
 *
 * The index is a uniform grid over the BANDS of the carriageway centerlines --
 * consecutive station pairs -- rather than over whole edges. Indexing whole
 * edges would only narrow the search to a road, leaving a linear scan along its
 * stations for the closest point; indexing bands answers "which piece of which
 * road is nearest this point" in one cell lookup. Bands are inserted under their
 * own bounding box and the query inflates instead, which keeps insertion tight
 * on a long diagonal road.
 *
 * The test itself is per STATION of the footway, never per endpoint. A footway
 * crossing a road at its midpoint has both endpoints far from it and would pass
 * any endpoint-proximity test; sampling along its length is what tells a
 * crossing (a handful of stations, perpendicular) from a sidewalk (most of its
 * stations, parallel).
 */

#include "osm/road/sidewalk_dedup.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Constants
// ============================================================================

/// Smallest grid cell, metres. Below this the cell overhead beats the pruning.
constexpr double kMinCellSize = 8.0;

/// Hard ceiling on cells per axis, so a sparse extract cannot allocate a huge grid
constexpr int kMaxGridDim = 2048;

/**
 * @brief Smallest median lateral offset that names a side, metres
 *
 * A footway whose median offset is under this runs down the carriageway
 * centreline, which is a digitising artefact rather than a sidewalk on either
 * side. Picking a side from the sign of a number this small is a coin toss, so
 * no side is picked at all.
 */
constexpr double kMinLateralOffset = 0.05;

/// Sentinel for "no strip of that kind"
constexpr size_t kNoStrip = static_cast<size_t>(-1);

// ============================================================================
// Small geometry helpers
// ============================================================================

/**
 * @brief One band of a carriageway centerline: stations [station, station + 1]
 *
 * Eight bytes, so the grid stores them by value and stays contiguous.
 */
struct Band {
    EdgeId edge = kInvalidId;
    uint32_t station = 0;
};

/// Result of dropping a point onto one band
struct Projection {
    glm::dvec2 direction{1.0, 0.0}; ///< unit direction of travel along the band
    double distance = 0.0;          ///< metres from the point to the band
    double lateral = 0.0;           ///< signed, positive to the LEFT of direction
    double along = 0.0;             ///< metres from the band's first station
    bool valid = false;             ///< false for a degenerate band
};

/**
 * @brief Drop @p p onto the segment between two stations
 *
 * The frame is taken from the BAND rather than from Station::normal. A station's
 * normal is the miter bisector, which at a joint is tilted away from the
 * perpendicular by half the turn angle; the lateral offset measured against it
 * would then vary with the corner rather than with the footway. The band's own
 * perpendicular is what the extruded ribbon edge actually runs along.
 *
 * Left is (-direction.y, direction.x), matching the convention fixed on
 * Station::normal and on Strip lateral coordinates: positive is LEFT of travel.
 *
 * @param a First station of the band
 * @param b Second station of the band
 * @param p Query point in the same local metres
 * @return The projection, or valid == false when the band has no length
 */
[[nodiscard]] Projection project_onto_band(const Station& a, const Station& b,
                                           const glm::dvec2& p) {
    Projection r;

    const glm::dvec2 d = b.position - a.position;
    const double len2 = glm::dot(d, d);
    // A bevel pair shares one position, so a zero-length band is expected here
    // rather than exceptional.
    if (!(len2 > 1.0e-12)) return r;

    double t = glm::dot(p - a.position, d) / len2;
    t = std::min(std::max(t, 0.0), 1.0);

    const glm::dvec2 closest = a.position + d * t;
    const glm::dvec2 delta = p - closest;

    r.direction = d / std::sqrt(len2);
    r.lateral = delta.x * -r.direction.y + delta.y * r.direction.x;
    r.distance = std::sqrt(glm::dot(delta, delta));
    r.along = t * std::sqrt(len2);
    r.valid = true;
    return r;
}

/**
 * @brief Median of a scratch buffer, which is reordered in place
 *
 * The median is used rather than the mean because a footway that swings around
 * a corner at one end, or steps out around a bus stop, would otherwise average
 * its way onto the wrong side of the road.
 *
 * @param values Non-empty buffer; reordered by the selection
 * @return The lower median for an even count, the middle value for an odd one
 */
[[nodiscard]] double median_of(std::vector<double>& values) {
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<ptrdiff_t>(mid), values.end());
    return values[mid];
}

/**
 * @brief Merge one more side into a suppression mask
 *
 * Unknown is never produced: on a suppression mask it would read as "no tag,
 * infer a default", which is the opposite of what this vector means.
 */
[[nodiscard]] SideFlags add_side(SideFlags mask, bool left, bool right) {
    const bool has_left = side_has_left(mask) || left;
    const bool has_right = side_has_right(mask) || right;
    if (has_left && has_right) return SideFlags::Both;
    if (has_left) return SideFlags::Left;
    if (has_right) return SideFlags::Right;
    return SideFlags::None;
}

// ============================================================================
// Candidate classification
// ============================================================================

/**
 * @brief Whether a road class is a footway candidate
 *
 * GraphEdge carries no tag map, so `footway=sidewalk` cannot be read here and
 * the class is the whole filter. Path is included because highway=track and
 * highway=path both land there and both are mapped alongside rural roads.
 */
[[nodiscard]] bool is_footway_class(RoadType type) {
    return type == RoadType::Footway || type == RoadType::Cycleway || type == RoadType::Path;
}

/**
 * @brief Which sides of a profile carry a synthesised Sidewalk strip
 *
 * The carriageway span is the inclusive range from the first Lane-or-Median
 * strip to the last, exactly as RoadProfile::left_edge_offset() defines it.
 * A Sidewalk strip before that span is the left sidewalk and one after it is the
 * right.
 *
 * A profile with no Lane and no Median at all is a bare footway. Its Sidewalk
 * strip IS the way rather than a kerbside strip beside one, so there is nothing
 * to suppress and the answer is None.
 *
 * @param profile Provisional profile for the edge
 * @return The sides carrying a suppressible sidewalk; None when there are none
 */
[[nodiscard]] SideFlags profile_sidewalk_sides(const RoadProfile& profile) {
    size_t first = kNoStrip;
    size_t last = kNoStrip;
    for (size_t i = 0; i < profile.strips.size(); ++i) {
        const StripKind kind = profile.strips[i].kind;
        if (kind == StripKind::Lane || kind == StripKind::Median) {
            if (first == kNoStrip) first = i;
            last = i;
        }
    }
    if (first == kNoStrip) return SideFlags::None;

    bool left = false;
    bool right = false;
    for (size_t i = 0; i < profile.strips.size(); ++i) {
        if (profile.strips[i].kind != StripKind::Sidewalk) continue;
        if (i < first) left = true;
        else if (i > last) right = true;
    }
    return add_side(SideFlags::None, left, right);
}

// ============================================================================
// Uniform grid over carriageway bands
// ============================================================================

/**
 * @brief Uniform grid of centerline bands, stored compressed-row
 *
 * Two counting passes rather than a vector of vectors: the cell contents end up
 * contiguous, which matters because the query walks a 2x2 or 3x3 neighbourhood
 * per footway station and there are millions of those on a city extract.
 *
 * Bands are inserted under their own bounding box and the QUERY is inflated by
 * the reach. Inflating at insertion instead would multiply the entry count of
 * every band by the cells the reach spills into, for no gain: the query has to
 * walk a neighbourhood either way.
 */
class BandGrid {
public:
    /**
     * @brief Index every band of every listed edge
     *
     * @param centerlines Parallel to graph.edges()
     * @param edges       Carriageway candidate edge IDs
     * @param reach       Query radius the cell size is chosen against, metres
     */
    void build(const std::vector<Centerline>& centerlines, const std::vector<EdgeId>& edges,
               double reach) {
        clear();
        if (edges.empty()) return;

        // Bounds over every station of every candidate.
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        size_t bands = 0;
        for (const EdgeId e : edges) {
            const std::vector<Station>& st = centerlines[e].stations;
            if (st.size() < 2) continue;
            for (const Station& s : st) {
                if (!std::isfinite(s.position.x) || !std::isfinite(s.position.y)) continue;
                min_x = std::min(min_x, s.position.x);
                min_y = std::min(min_y, s.position.y);
                max_x = std::max(max_x, s.position.x);
                max_y = std::max(max_y, s.position.y);
            }
            bands += st.size() - 1;
        }
        if (bands == 0 || min_x > max_x) return;

        m_min = glm::dvec2{min_x, min_y};

        // One cell holds a whole query neighbourhood, so a lookup touches 2x2
        // cells on a straight and never more than 3x3.
        m_cell = std::max(kMinCellSize, reach);
        const double span_x = std::max(max_x - min_x, 1.0e-6);
        const double span_y = std::max(max_y - min_y, 1.0e-6);
        m_nx = static_cast<int>(std::ceil(span_x / m_cell)) + 1;
        m_ny = static_cast<int>(std::ceil(span_y / m_cell)) + 1;
        if (m_nx > kMaxGridDim || m_ny > kMaxGridDim) {
            // A sparse extract spread over a huge area. Grow the cell instead of
            // the table; the query degrades to scanning more bands per cell,
            // which is far cheaper than allocating the dense grid.
            m_cell = std::max(m_cell, std::max(span_x, span_y) / static_cast<double>(kMaxGridDim));
            m_nx = std::min(kMaxGridDim, static_cast<int>(std::ceil(span_x / m_cell)) + 1);
            m_ny = std::min(kMaxGridDim, static_cast<int>(std::ceil(span_y / m_cell)) + 1);
        }

        const size_t cells = static_cast<size_t>(m_nx) * static_cast<size_t>(m_ny);
        m_start.assign(cells + 1, 0);

        // Pass 1: count.
        for_each_band(centerlines, edges, [&](EdgeId, uint32_t, int x0, int y0, int x1, int y1) {
            for (int y = y0; y <= y1; ++y) {
                for (int x = x0; x <= x1; ++x) {
                    ++m_start[cell_index(x, y) + 1];
                }
            }
        });
        for (size_t i = 1; i < m_start.size(); ++i) {
            m_start[i] += m_start[i - 1];
        }

        // Pass 2: fill, using a cursor copy so m_start survives as the offsets.
        m_items.resize(m_start.back());
        std::vector<uint32_t> cursor(m_start.begin(), m_start.end() - 1);
        for_each_band(centerlines, edges,
                      [&](EdgeId e, uint32_t station, int x0, int y0, int x1, int y1) {
                          for (int y = y0; y <= y1; ++y) {
                              for (int x = x0; x <= x1; ++x) {
                                  m_items[cursor[cell_index(x, y)]++] = Band{e, station};
                              }
                          }
                      });
    }

    /// Drop every band and every cell
    void clear() {
        m_start.clear();
        m_items.clear();
        m_nx = 0;
        m_ny = 0;
    }

    /// True when nothing was indexed
    [[nodiscard]] bool empty() const { return m_items.empty(); }

    /**
     * @brief Call @p fn for every band whose cell is within @p reach of @p p
     *
     * A band reported here is a CANDIDATE: it shares a cell neighbourhood with
     * the point, not necessarily the distance. The caller still measures.
     */
    template <typename Fn>
    void visit(const glm::dvec2& p, double reach, Fn&& fn) const {
        if (m_items.empty()) return;

        const int x0 = clamp_axis(axis_index(p.x - reach, m_min.x), m_nx);
        const int x1 = clamp_axis(axis_index(p.x + reach, m_min.x), m_nx);
        const int y0 = clamp_axis(axis_index(p.y - reach, m_min.y), m_ny);
        const int y1 = clamp_axis(axis_index(p.y + reach, m_min.y), m_ny);

        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const size_t cell = cell_index(x, y);
                for (uint32_t k = m_start[cell]; k < m_start[cell + 1]; ++k) {
                    fn(m_items[k]);
                }
            }
        }
    }

private:
    /// Walk every band once, handing its clamped cell range to @p fn
    template <typename Fn>
    void for_each_band(const std::vector<Centerline>& centerlines,
                       const std::vector<EdgeId>& edges, Fn&& fn) const {
        for (const EdgeId e : edges) {
            const std::vector<Station>& st = centerlines[e].stations;
            for (size_t i = 0; i + 1 < st.size(); ++i) {
                const glm::dvec2& a = st[i].position;
                const glm::dvec2& b = st[i + 1].position;
                if (!std::isfinite(a.x) || !std::isfinite(a.y)) continue;
                if (!std::isfinite(b.x) || !std::isfinite(b.y)) continue;

                const int x0 = clamp_axis(axis_index(std::min(a.x, b.x), m_min.x), m_nx);
                const int x1 = clamp_axis(axis_index(std::max(a.x, b.x), m_min.x), m_nx);
                const int y0 = clamp_axis(axis_index(std::min(a.y, b.y), m_min.y), m_ny);
                const int y1 = clamp_axis(axis_index(std::max(a.y, b.y), m_min.y), m_ny);
                fn(e, static_cast<uint32_t>(i), x0, y0, x1, y1);
            }
        }
    }

    [[nodiscard]] int axis_index(double value, double origin) const {
        const double scaled = (value - origin) / m_cell;
        if (!std::isfinite(scaled)) return 0;
        // Clamped before the cast so the double never overflows the int.
        const double bounded = std::min(std::max(scaled, -1.0), static_cast<double>(kMaxGridDim));
        return static_cast<int>(std::floor(bounded));
    }

    [[nodiscard]] static int clamp_axis(int index, int count) {
        return std::min(std::max(index, 0), std::max(0, count - 1));
    }

    [[nodiscard]] size_t cell_index(int x, int y) const {
        return static_cast<size_t>(y) * static_cast<size_t>(m_nx) + static_cast<size_t>(x);
    }

    glm::dvec2 m_min{0.0};
    double m_cell = kMinCellSize;
    int m_nx = 0;
    int m_ny = 0;
    std::vector<uint32_t> m_start;  ///< cells + 1 offsets into m_items
    std::vector<Band> m_items;
};

/// Per-carriageway accumulation for one footway, reused across footways
struct Claim {
    EdgeId edge = kInvalidId;
    std::vector<double> offsets;

    /// Arc length along the CARRIAGEWAY of the first and last station claimed
    double arc_lo = std::numeric_limits<double>::max();
    double arc_hi = std::numeric_limits<double>::lowest();

    [[nodiscard]] double covered() const {
        return (arc_hi > arc_lo) ? (arc_hi - arc_lo) : 0.0;
    }
};

} // namespace

// ============================================================================
// mask_side
// ============================================================================

SideFlags mask_side(SideFlags tagged, SideFlags suppress) {
    if (suppress == SideFlags::None || suppress == SideFlags::Unknown) return tagged;

    // Unknown means "no tag, a class default may be inferred". Subtracting a side
    // from it has to RESOLVE it, so it is expanded to Both first: the default
    // that would have been inferred is a sidewalk on every side not suppressed.
    // Leaving it Unknown would let build_profile() infer the default straight
    // back and re-synthesise exactly the sidewalk that was suppressed.
    const bool has_left = (tagged == SideFlags::Unknown) || side_has_left(tagged);
    const bool has_right = (tagged == SideFlags::Unknown) || side_has_right(tagged);

    const bool left = has_left && !side_has_left(suppress);
    const bool right = has_right && !side_has_right(suppress);

    if (left && right) return SideFlags::Both;
    if (left) return SideFlags::Left;
    if (right) return SideFlags::Right;
    return SideFlags::None;
}

// ============================================================================
// dedup_sidewalks
// ============================================================================

DedupResult dedup_sidewalks(const RoadGraph& graph, const std::vector<Centerline>& centerlines,
                            const std::vector<RoadProfile>& profiles, const DedupConfig& cfg) {
    const std::vector<GraphEdge>& edges = graph.edges();

    DedupResult result;
    result.suppress_side.assign(edges.size(), SideFlags::None);

    // suppress_side is sized to the graph whatever happens, so a caller may index
    // it unconditionally. Everything below is the only thing the switch turns off.
    if (!cfg.enabled) return result;
    if (edges.empty()) return result;
    if (centerlines.size() < edges.size() || profiles.size() < edges.size()) return result;

    const double max_offset = cfg.max_offset;
    if (!(max_offset > 0.0) || !std::isfinite(max_offset)) return result;

    const double fraction = std::min(std::max(cfg.min_parallel_fraction, 0.0), 1.0);
    const double coverage_limit = std::min(std::max(cfg.min_edge_coverage, 0.0), 1.0);

    // Compared as a cosine rather than an angle: acos per band per station is the
    // hot loop of this file, and the comparison is exactly equivalent because
    // acos is monotone over [0, 1].
    const double bearing_limit =
        std::min(std::max(cfg.max_bearing_delta, 0.0), 3.14159265358979323846 * 0.5);
    const double cos_limit = std::cos(bearing_limit);

    // ------------------------------------------------------------------------
    // Candidates
    // ------------------------------------------------------------------------
    // A carriageway candidate has a synthesised sidewalk to lose. sidewalk=no and
    // sidewalk=separate therefore cost nothing here: build_profile() emitted no
    // Sidewalk strip, so the edge never enters the index.
    std::vector<EdgeId> carriageways;
    std::vector<EdgeId> footways;
    std::vector<SideFlags> carriageway_sides(edges.size(), SideFlags::None);

    carriageways.reserve(edges.size() / 2 + 1);
    footways.reserve(edges.size() / 4 + 1);

    for (size_t i = 0; i < edges.size(); ++i) {
        if (!centerlines[i].is_valid()) continue;
        const EdgeId id = static_cast<EdgeId>(i);

        if (is_footway_class(edges[i].type)) {
            footways.push_back(id);
            continue;
        }
        const SideFlags sides = profile_sidewalk_sides(profiles[i]);
        if (sides == SideFlags::None) continue;
        carriageway_sides[i] = sides;
        carriageways.push_back(id);
    }
    if (carriageways.empty() || footways.empty()) return result;

    BandGrid grid;
    grid.build(centerlines, carriageways, max_offset);
    if (grid.empty()) return result;

    // ------------------------------------------------------------------------
    // Per footway: assign each station to its nearest parallel carriageway
    // ------------------------------------------------------------------------
    std::vector<Claim> claims;
    std::vector<double> scratch;

    for (const EdgeId f : footways) {
        const GraphEdge& footway = edges[f];
        const std::vector<Station>& stations = centerlines[f].stations;
        if (stations.empty()) continue;

        claims.clear();
        size_t matched_stations = 0;

        for (const Station& s : stations) {
            if (!std::isfinite(s.position.x) || !std::isfinite(s.position.y)) continue;

            EdgeId best_edge = kInvalidId;
            double best_distance = std::numeric_limits<double>::max();
            double best_lateral = 0.0;
            double best_arc = 0.0;

            grid.visit(s.position, max_offset, [&](const Band& band) {
                const GraphEdge& road = edges[band.edge];

                // A footbridge over a road is not that road's sidewalk. The graph
                // already keeps the two apart topologically; this keeps them apart
                // geometrically, where they are metres from each other in plan.
                if (road.layer != footway.layer) return;

                const std::vector<Station>& road_stations = centerlines[band.edge].stations;
                if (static_cast<size_t>(band.station) + 1 >= road_stations.size()) return;

                const Projection p = project_onto_band(road_stations[band.station],
                                                       road_stations[band.station + 1],
                                                       s.position);
                if (!p.valid) return;
                if (p.distance > max_offset) return;
                if (p.distance >= best_distance) return;

                // Modulo pi, not modulo 2*pi: OSM way direction is arbitrary, and a
                // sidewalk digitised against its carriageway is still its sidewalk.
                const double alignment = std::fabs(glm::dot(s.tangent, p.direction));
                if (alignment < cos_limit) return;

                best_edge = band.edge;
                best_distance = p.distance;
                best_lateral = p.lateral;
                best_arc = road_stations[band.station].arclength + p.along;
            });

            if (best_edge == kInvalidId) continue;
            ++matched_stations;

            Claim* claim = nullptr;
            for (Claim& c : claims) {
                if (c.edge == best_edge) {
                    claim = &c;
                    break;
                }
            }
            if (claim == nullptr) {
                claims.push_back(Claim{best_edge, {}});
                claim = &claims.back();
            }
            claim->offsets.push_back(best_lateral);
            claim->arc_lo = std::min(claim->arc_lo, best_arc);
            claim->arc_hi = std::max(claim->arc_hi, best_arc);
        }

        // The parallel-fraction gate, measured over the footway AS A WHOLE rather
        // than against one carriageway edge. Carriageways are split at their own
        // junctions, so a footway running the length of a block is routinely
        // shared between two or three edges of the same street and would clear no
        // per-edge threshold while plainly being that street's sidewalk. What the
        // gate has to reject is the footway that merely touches a road -- a
        // crossing, a driveway, a path leaving a junction -- and those fail on the
        // aggregate just as surely.
        if (claims.empty()) continue;
        const double matched = static_cast<double>(matched_stations) /
                               static_cast<double>(stations.size());
        if (matched < fraction) continue;

        ++result.matched_footways;

        for (Claim& claim : claims) {
            // How much of the CARRIAGEWAY this footway actually runs alongside.
            // Suppression is per edge and per side, because RoadProfile is one
            // cross-section for the whole edge, so a footway covering part of an
            // edge would otherwise delete the synthesised sidewalk -- and the
            // curb face and curb top with it -- along all of it. A 120 m surveyed
            // path beside a 200 m edge would leave 80 m of street with paving
            // from neither source.
            const std::vector<Station>& road_stations = centerlines[claim.edge].stations;
            const double road_length =
                road_stations.empty()
                    ? 0.0
                    : (road_stations.back().arclength - road_stations.front().arclength);
            if (road_length > kMinLateralOffset &&
                claim.covered() < coverage_limit * road_length) {
                continue;
            }

            scratch.assign(claim.offsets.begin(), claim.offsets.end());
            const double lateral = median_of(scratch);
            if (std::fabs(lateral) < kMinLateralOffset) continue;

            // The footway sits to the LEFT of the carriageway when its lateral
            // offset is positive, so that is the side whose synthesis stops.
            const bool on_left = lateral > 0.0;
            const SideFlags available = carriageway_sides[claim.edge];
            if (on_left && !side_has_left(available)) continue;
            if (!on_left && !side_has_right(available)) continue;

            SideFlags& mask = result.suppress_side[claim.edge];
            const SideFlags before = mask;
            mask = add_side(mask, on_left, !on_left);
            if (before == SideFlags::None && mask != SideFlags::None) {
                ++result.suppressed_edges;
            }
        }
    }

    return result;
}

} // namespace stratum::osm::road
