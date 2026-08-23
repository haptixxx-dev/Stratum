/**
 * @file crossings.cpp
 * @brief Implementation of pedestrian crossings and the kerb drops they need
 *
 * Three problems, solved in this order.
 *
 *   1. **Locating a crossing.** A `highway=crossing` node is almost never a
 *      GraphNode. GraphNode exists only where ways MEET, and a crossing node is
 *      an ordinary interior vertex of the carriageway way unless a footway
 *      happens to share it. So the search is over every EDGE's `node_ids`, not
 *      over the node list -- the same interior-node insight that made P1's
 *      T-junction detection work. The vertex index found there is turned into an
 *      arclength by projecting the vertex POSITION onto the resampled
 *      centerline, never by accumulating polyline chords: build_centerline()
 *      smooths and resamples, so chord arclength and station arclength are not
 *      the same number.
 *
 *   2. **Painting it.** Stripes are fitted INSIDE the carriageway rather than
 *      laid out and clipped afterwards. See fit_stripes().
 *
 *   3. **Dropping the kerb.** No geometry is produced here. Both span functions
 *      return angular runs of the junction's kerb ring measured from the
 *      junction centre, which junction_curb.cpp turns into a height modulation
 *      of the ring's existing cross-section.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 */

#include "osm/road/crossings.hpp"

#include "osm/coordinates.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/// Lengths at or below this count as zero, metres
constexpr double kZeroLength = 1e-9;

/// Slack on arclength comparisons, metres
constexpr double kArclengthEpsilon = 1e-6;

/// Squared length of a raw face cross product below which a triangle is dropped
constexpr double kDegenerateCrossSq = 1e-16;

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kPi = 3.141592653589793238462643383279;

// ============================================================================
// Small helpers
// ============================================================================

/**
 * @brief The codebase-wide 2D-to-3D mapping, Y up
 *
 * (x, y_2d) -> (x, height, -y_2d). The same mapping corridor.cpp,
 * junction_polygon.cpp and junction_curb.cpp use.
 */
[[nodiscard]] inline glm::vec3 to_world(const glm::dvec2& p, double height) {
    return glm::vec3(static_cast<float>(p.x),
                     static_cast<float>(height),
                     static_cast<float>(-p.y));
}

/// Unit direction, or the fallback when @p v is too short to normalise
[[nodiscard]] inline glm::dvec2 safe_normalise(const glm::dvec2& v, const glm::dvec2& fallback) {
    const double len = glm::length(v);
    return (len > kZeroLength) ? (v / len) : fallback;
}

/// Rotate a left-of-travel normal back into the direction of travel
[[nodiscard]] inline glm::dvec2 travel_of(const glm::dvec2& axis) {
    // normal = (-t.y, t.x), so t = (n.y, -n.x).
    return glm::dvec2(axis.y, -axis.x);
}

[[nodiscard]] inline bool is_finite(const glm::dvec2& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

/// Wrap an angle into [0, 2pi)
[[nodiscard]] inline double wrap_positive(double a) {
    a = std::fmod(a, kTwoPi);
    if (a < 0.0) {
        a += kTwoPi;
    }
    return a;
}

/// Tag lookup returning nullptr when absent
[[nodiscard]] const std::string* find_tag(const TagMap& tags, const char* key) {
    const auto it = tags.find(key);
    return (it == tags.end()) ? nullptr : &it->second;
}

// ============================================================================
// Which roads and which OSM shapes take part
// ============================================================================

/**
 * @brief A road class a zebra may be painted across
 *
 * Footway, Cycleway and Path are excluded by the header contract: a crossing
 * mapped as a node on a footway is the footway crossing something else, and
 * painting stripes along the footway itself is the wrong road.
 */
[[nodiscard]] bool is_carriageway(RoadType type) {
    switch (type) {
        case RoadType::Footway:
        case RoadType::Cycleway:
        case RoadType::Path:
            return false;
        default:
            return true;
    }
}

/// The profile actually has a running surface to paint on
[[nodiscard]] bool profile_has_lane(const RoadProfile& profile) {
    for (const Strip& s : profile.strips) {
        if (s.kind == StripKind::Lane && s.width > 0.0f) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Lateral extent a zebra spans, and any raised island inside it
 *
 * All in the profile's own lateral frame: positive LEFT of travel, zero on the
 * centreline.
 */
struct CrossExtent {
    double half = 0.0;          ///< Half the Lane-to-Median inclusive span
    double island_left = 0.0;   ///< Left edge of the raised island, if any
    double island_right = 0.0;  ///< Right edge of it; equal to island_left when none
    bool valid = false;
};

/**
 * @brief Measure the carriageway envelope of a profile
 *
 * The extent is the INCLUSIVE span from the first Lane-or-Median strip to the
 * last, which is the span RoadProfile::left_edge_offset() centres on the
 * centreline. RoadProfile::carriageway_width() sums Lane strips alone, so on a
 * divided road it is short by the median and the curb faces beside it, and a run
 * fitted to it stops metres inside each kerb.
 *
 * A RAISED island inside that span is measured separately. It is the union of
 * the interior strips that are not Lanes and stand above the carriageway plane
 * -- the median and its two curb faces -- and nothing is painted across it. A
 * flush median has no height and so is not an island: paint runs straight over
 * it, which is what a flush median is for.
 */
[[nodiscard]] CrossExtent measure_extent(const RoadProfile& profile) {
    CrossExtent out;

    /// Above this a strip is a kerb or an island rather than part of the surface
    constexpr double kFlushHeight = 0.01;

    const size_t none = profile.strips.size();
    size_t first = none;
    size_t last = 0;
    for (size_t i = 0; i < profile.strips.size(); ++i) {
        const StripKind kind = profile.strips[i].kind;
        if (kind == StripKind::Lane || kind == StripKind::Median) {
            if (first == none) {
                first = i;
            }
            last = i;
        }
    }
    if (first == none) {
        return out;
    }

    double lateral = static_cast<double>(profile.left_edge_offset());
    double lat_left_first = 0.0;
    double lat_right_last = 0.0;
    bool island = false;
    double island_left = 0.0;
    double island_right = 0.0;

    for (size_t i = 0; i < profile.strips.size(); ++i) {
        const Strip& strip = profile.strips[i];
        const double lat_left = lateral;
        const double lat_right = lateral - static_cast<double>(strip.width);
        lateral = lat_right;

        if (i == first) {
            lat_left_first = lat_left;
        }
        if (i == last) {
            lat_right_last = lat_right;
        }
        if (i <= first || i >= last || strip.kind == StripKind::Lane) {
            continue;
        }

        const double raised = std::max(std::fabs(static_cast<double>(strip.height_left)),
                                       std::fabs(static_cast<double>(strip.height_right)));
        if (raised <= kFlushHeight || !(strip.width > 0.0f)) {
            continue;
        }
        if (!island) {
            island = true;
            island_left = lat_left;
            island_right = lat_right;
        } else {
            island_left = std::max(island_left, lat_left);
            island_right = std::min(island_right, lat_right);
        }
    }

    out.half = 0.5 * (lat_left_first - lat_right_last);
    if (!(out.half > kZeroLength)) {
        return out;
    }
    if (island) {
        out.island_left = island_left;
        out.island_right = island_right;
    }
    out.valid = true;
    return out;
}

/// `highway=crossing` on the OSM node
[[nodiscard]] bool is_crossing_node(const OSMNode& node) {
    const std::string* highway = find_tag(node.tags, "highway");
    return highway != nullptr && *highway == "crossing";
}

/**
 * @brief `highway=footway` + `footway=crossing` on the OSM way
 *
 * The cycleway and path spellings of the same thing are accepted too. They
 * describe the same physical crossing and resolve through exactly the same
 * shared-node path; refusing them would drop every cycle crossing in an extract
 * that tags them that way.
 */
[[nodiscard]] bool is_crossing_way(const OSMWay& way) {
    const std::string* highway = find_tag(way.tags, "highway");
    if (highway == nullptr) {
        return false;
    }
    if (*highway == "footway") {
        const std::string* footway = find_tag(way.tags, "footway");
        return footway != nullptr && *footway == "crossing";
    }
    if (*highway == "cycleway") {
        const std::string* cycleway = find_tag(way.tags, "cycleway");
        return cycleway != nullptr && *cycleway == "crossing";
    }
    if (*highway == "path") {
        const std::string* path = find_tag(way.tags, "path");
        return path != nullptr && *path == "crossing";
    }
    return false;
}

/**
 * @brief The edge is a driveway, parking aisle or alley meeting a parent road
 *
 * GraphEdge carries no `service=*` field, so the subtype comes from the parent
 * way's raw tags. A `highway=service` way with no subtype at all is accepted:
 * an untagged service road meeting a street has the same dropped kerb as a
 * driveway does, and refusing it would miss most of them.
 */
[[nodiscard]] bool is_driveway(const GraphEdge& edge, const ParsedOSMData& data) {
    if (edge.type != RoadType::Service) {
        return false;
    }
    const auto it = data.ways.find(edge.source_way);
    if (it == data.ways.end()) {
        return true;
    }
    const std::string* service = find_tag(it->second.tags, "service");
    if (service == nullptr) {
        return true;
    }
    return *service == "driveway" || *service == "parking_aisle" || *service == "alley";
}

// ============================================================================
// Centerline queries
// ============================================================================

/**
 * @brief A point resolved onto a centerline
 */
struct CenterlineSample {
    glm::dvec2 position{0.0};
    glm::dvec2 tangent{1.0, 0.0};   ///< unit, direction of travel
    glm::dvec2 normal{0.0, 1.0};    ///< unit, LEFT of travel
    size_t band = 0;                ///< index of the station the band starts at
    double t = 0.0;                 ///< fraction along the band, [0, 1]
    bool valid = false;
};

/**
 * @brief Arclength of the point of @p cl nearest to @p p
 *
 * Bands of zero length are skipped: a bevel joint is represented as two stations
 * sharing one arclength, and projecting onto that pair says nothing about where
 * along the road the query point sits.
 *
 * @param cl       Centerline to project onto
 * @param p        Query point in 2D local metres
 * @param out_dist Receives the distance from @p p to the centerline, metres
 * @return Arclength of the nearest point, in the centerline's own parameterisation
 */
[[nodiscard]] double project_to_centerline(const Centerline& cl,
                                           const glm::dvec2& p,
                                           double& out_dist) {
    out_dist = std::numeric_limits<double>::max();
    if (!cl.is_valid()) {
        return 0.0;
    }

    double best_arclength = cl.stations.front().arclength;
    double best_dist_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i + 1 < cl.stations.size(); ++i) {
        const Station& a = cl.stations[i];
        const Station& b = cl.stations[i + 1];
        const glm::dvec2 d = b.position - a.position;
        const double len_sq = glm::dot(d, d);
        if (len_sq <= kZeroLength * kZeroLength) {
            continue;
        }

        double t = glm::dot(p - a.position, d) / len_sq;
        t = std::clamp(t, 0.0, 1.0);

        const glm::dvec2 foot = a.position + d * t;
        const glm::dvec2 delta = p - foot;
        const double dist_sq = glm::dot(delta, delta);
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_arclength = a.arclength + t * (b.arclength - a.arclength);
        }
    }

    out_dist = (best_dist_sq == std::numeric_limits<double>::max())
                   ? std::numeric_limits<double>::max()
                   : std::sqrt(best_dist_sq);
    return best_arclength;
}

/**
 * @brief The frame of a centerline at one arclength
 *
 * The normal is the interpolated MITER BISECTOR, per the Crossing::axis
 * contract, so on a straight it is exactly the left perpendicular of the
 * tangent and at a joint it leans with the joint. Where the two bracketing
 * bisectors cancel -- a hairpin -- the tangent's own left perpendicular takes
 * over rather than leaving a zero-length axis.
 */
[[nodiscard]] CenterlineSample sample_centerline(const Centerline& cl, double arclength) {
    CenterlineSample out;
    if (!cl.is_valid()) {
        return out;
    }

    const double lo = cl.stations.front().arclength;
    const double hi = cl.stations.back().arclength;
    const double s = std::clamp(arclength, lo, hi);

    size_t band = 0;
    double t = 0.0;
    bool found = false;

    for (size_t i = 0; i + 1 < cl.stations.size(); ++i) {
        const double a = cl.stations[i].arclength;
        const double b = cl.stations[i + 1].arclength;
        if (b - a <= kZeroLength) {
            continue;   // a bevel pair; it spans no arclength to land in
        }
        if (s <= b + kArclengthEpsilon) {
            band = i;
            t = std::clamp((s - a) / (b - a), 0.0, 1.0);
            found = true;
            break;
        }
        band = i;
        t = 1.0;
        found = true;
    }

    if (!found) {
        return out;     // every band was a bevel pair: no usable geometry
    }

    const Station& a = cl.stations[band];
    const Station& b = cl.stations[band + 1];

    out.band = band;
    out.t = t;
    out.position = a.position + (b.position - a.position) * t;
    out.tangent = safe_normalise(b.position - a.position, a.tangent);
    out.normal = safe_normalise(a.normal + (b.normal - a.normal) * t,
                                glm::dvec2(-out.tangent.y, out.tangent.x));
    out.valid = true;
    return out;
}

/**
 * @brief Road surface height at a sampled station, or 0 when unsolved
 */
[[nodiscard]] float height_at(const RoadElevationSolver& elevation,
                              EdgeId edge,
                              const CenterlineSample& sample) {
    if (!elevation.is_solved() || edge >= elevation.edges().size()) {
        return 0.0f;
    }
    const std::vector<float>& heights = elevation.edge(edge).station_heights;
    if (sample.band + 1 >= heights.size()) {
        return heights.empty() ? 0.0f : heights.back();
    }
    const float a = heights[sample.band];
    const float b = heights[sample.band + 1];
    return a + (b - a) * static_cast<float>(sample.t);
}

// ============================================================================
// Angular spans
// ============================================================================

/**
 * @brief One run of the ring, as a start angle and a counter-clockwise sweep
 *
 * Held in angles rather than in the pair of unit vectors DroppedKerbSpan
 * publishes, because merging overlapping runs is an interval problem and
 * intervals need a scalar. The vectors are recovered on the way out.
 */
struct Arc {
    double start = 0.0;     ///< in [0, 2pi)
    double sweep = 0.0;     ///< in (0, 2pi)
};

/**
 * @brief The arc subtended at @p center by the segment from @p a to @p b
 *
 * Always the SHORT way round. A kerb drop is a couple of metres on a ring tens
 * of metres in circumference, so a sweep past pi means the two endpoints were
 * taken in the wrong order, never that the drop really wraps most of the
 * junction.
 *
 * @return false when either endpoint sits on the centre, or the two collapse
 */
[[nodiscard]] bool arc_between(const glm::dvec2& center,
                               const glm::dvec2& a,
                               const glm::dvec2& b,
                               Arc& out) {
    const glm::dvec2 da = a - center;
    const glm::dvec2 db = b - center;
    if (glm::length(da) <= kZeroLength || glm::length(db) <= kZeroLength) {
        return false;
    }

    const double theta_a = std::atan2(da.y, da.x);
    const double theta_b = std::atan2(db.y, db.x);

    double start = theta_a;
    double sweep = wrap_positive(theta_b - theta_a);
    if (sweep > kPi) {
        start = theta_b;
        sweep = kTwoPi - sweep;
    }
    if (sweep <= 1e-9) {
        return false;
    }

    out.start = wrap_positive(start);
    out.sweep = sweep;
    return true;
}

/**
 * @brief Merge overlapping arcs, wrap included
 *
 * Two crossings on adjacent arms sharing a corner must produce one continuous
 * drop rather than two that fight over the same kerb stones, and the pair that
 * straddles angle zero is the one a naive sort-and-sweep misses.
 */
[[nodiscard]] std::vector<Arc> merge_arcs(std::vector<Arc> arcs) {
    if (arcs.size() < 2) {
        return arcs;
    }

    std::sort(arcs.begin(), arcs.end(),
              [](const Arc& x, const Arc& y) { return x.start < y.start; });

    std::vector<Arc> merged;
    merged.reserve(arcs.size());
    for (const Arc& a : arcs) {
        if (merged.empty()) {
            merged.push_back(a);
            continue;
        }
        Arc& last = merged.back();
        const double last_end = last.start + last.sweep;
        if (a.start <= last_end + 1e-9) {
            last.sweep = std::max(last_end, a.start + a.sweep) - last.start;
        } else {
            merged.push_back(a);
        }
    }

    // The wrap: the last arc may run back over the first one across angle zero.
    while (merged.size() >= 2) {
        Arc& first = merged.front();
        Arc& last = merged.back();
        if (last.start + last.sweep < first.start + kTwoPi - 1e-9) {
            break;
        }
        Arc joined;
        joined.start = last.start;
        joined.sweep = std::max(last.start + last.sweep,
                                first.start + first.sweep + kTwoPi) - last.start;
        merged.erase(merged.begin());
        merged.back() = joined;
    }

    // A drop covering the whole ring would emit from == to, which the header
    // reserves for an EMPTY span. Leave a hair of full-height kerb instead.
    for (Arc& a : merged) {
        a.sweep = std::min(a.sweep, kTwoPi - 1e-6);
        a.start = wrap_positive(a.start);
    }
    std::sort(merged.begin(), merged.end(),
              [](const Arc& x, const Arc& y) { return x.start < y.start; });

    return merged;
}

/// Turn merged arcs into the published span form
[[nodiscard]] std::vector<DroppedKerbSpan> to_spans(const std::vector<Arc>& arcs, float height) {
    std::vector<DroppedKerbSpan> out;
    out.reserve(arcs.size());
    for (const Arc& a : arcs) {
        DroppedKerbSpan s;
        s.from = glm::dvec2(std::cos(a.start), std::sin(a.start));
        s.to = glm::dvec2(std::cos(a.start + a.sweep), std::sin(a.start + a.sweep));
        s.height = height;
        out.push_back(s);
    }
    return out;
}

// ============================================================================
// Stripe layout
// ============================================================================

/**
 * @brief Lateral centres of the stripes of one crossing
 *
 * ### The clipping rule
 *
 * Stripes are FITTED INSIDE the carriageway, never laid out and clipped
 * afterwards. The largest whole number of stripes whose total width fits in
 * @p width is taken, and the run is centred, so the outermost stripe's outer
 * edge sits at most @p width / 2 from the run's own centre whatever the numbers
 * are. The caller passes one run per carriageway -- two on a road divided by a
 * raised island -- and each run's bound is a kerb line, so no stripe can reach
 * the gutter, ride over the curb, climb the island, or appear on the footway,
 * which is the classic failure this crossing code exists to avoid.
 *
 * A partial stripe is dropped rather than clipped, which is the same rule read
 * from the other end: it keeps the pattern symmetric about the centreline and
 * stops a half-width stripe appearing against one kerb and not the other.
 */
[[nodiscard]] std::vector<double> fit_stripes(double width, double stripe, double gap) {
    std::vector<double> centres;
    const double pitch = stripe + gap;
    if (!(width > 0.0) || !(stripe > kZeroLength) || !(pitch > kZeroLength)) {
        return centres;
    }
    if (stripe > width) {
        return centres;     // too narrow for even one full stripe
    }

    const auto count = static_cast<long long>(std::floor((width + gap) / pitch + 1e-9));
    if (count < 1) {
        return centres;
    }

    const double total = static_cast<double>(count) * stripe +
                         static_cast<double>(count - 1) * gap;
    const double first = -0.5 * (total - stripe);

    centres.reserve(static_cast<size_t>(count));
    for (long long k = 0; k < count; ++k) {
        centres.push_back(first + static_cast<double>(k) * pitch);
    }
    return centres;
}

} // namespace

// ============================================================================
// find_crossings
// ============================================================================

std::vector<Crossing> find_crossings(const RoadGraph& graph,
                                     const ParsedOSMData& data,
                                     const std::vector<Centerline>& centerlines,
                                     const std::vector<RoadProfile>& profiles,
                                     const RoadElevationSolver& elevation,
                                     const CrossingConfig& cfg) {
    std::vector<Crossing> out;

    const std::vector<GraphEdge>& edges = graph.edges();
    if (edges.empty()) {
        return out;
    }
    if (centerlines.size() != edges.size() || profiles.size() != edges.size()) {
        spdlog::warn("find_crossings: {} edges but {} centerlines and {} profiles; "
                     "refusing to locate crossings",
                     edges.size(), centerlines.size(), profiles.size());
        return out;
    }

    // ------------------------------------------------------------------------
    // Which edges may carry a crossing at all.
    // ------------------------------------------------------------------------
    std::vector<uint8_t> eligible(edges.size(), 0u);
    size_t eligible_count = 0;
    for (size_t i = 0; i < edges.size(); ++i) {
        const bool ok = is_carriageway(edges[i].type) && centerlines[i].is_valid() &&
                        profile_has_lane(profiles[i]) &&
                        edges[i].node_ids.size() == edges[i].polyline.size();
        eligible[i] = ok ? 1u : 0u;
        eligible_count += ok ? 1u : 0u;
    }
    if (eligible_count == 0) {
        return out;
    }

    // ------------------------------------------------------------------------
    // Shape 1: highway=crossing nodes.
    // ------------------------------------------------------------------------
    std::unordered_set<NodeId> crossing_nodes;
    for (const auto& [id, node] : data.nodes) {
        if (is_crossing_node(node)) {
            crossing_nodes.insert(id);
        }
    }

    // ------------------------------------------------------------------------
    // Shape 2: footway=crossing ways, indexed by the nodes they run through.
    //
    // A crossing way is resolved by SHARED NODE, never by proximity. A way
    // sharing no node with any carriageway is discarded: snapping it to the
    // nearest road is exactly the mistake the whole graph was built to avoid,
    // and it paints a zebra across the wrong road.
    // ------------------------------------------------------------------------
    struct WayRef {
        WayId way = 0;
        size_t index = 0;
    };
    std::unordered_map<NodeId, WayRef> crossing_way_nodes;
    for (const auto& [id, way] : data.ways) {
        if (!is_crossing_way(way)) {
            continue;
        }
        for (size_t i = 0; i < way.node_refs.size(); ++i) {
            const NodeId nid = way.node_refs[i];
            const auto it = crossing_way_nodes.find(nid);
            if (it == crossing_way_nodes.end()) {
                crossing_way_nodes.emplace(nid, WayRef{ id, i });
            } else if (id < it->second.way) {
                // data.ways is unordered; keeping the lower WayId makes the
                // choice reproducible run to run.
                it->second = WayRef{ id, i };
            }
        }
    }

    if (crossing_nodes.empty() && crossing_way_nodes.empty()) {
        return out;
    }

    // ------------------------------------------------------------------------
    // One pass over the edges' node_ids. This is where a crossing is actually
    // found: it is an INTERIOR vertex of a carriageway way in the common case,
    // so it has no GraphNode of its own and cannot be found in the node list.
    // ------------------------------------------------------------------------
    struct Candidate {
        EdgeId edge = kInvalidId;
        size_t vertex = 0;
        NodeId osm_node = 0;
        bool from_node_tag = false;     ///< false: derived from a crossing WAY
        WayRef way;
    };
    std::vector<Candidate> candidates;

    // One crossing per OSM node. A crossing node shared with a footway becomes a
    // graph node, which SPLITS the carriageway there, so the same physical
    // crossing would otherwise be found once as the `to` end of one edge and
    // again as the `from` end of the next.
    std::unordered_set<NodeId> claimed;

    for (size_t e = 0; e < edges.size(); ++e) {
        if (eligible[e] == 0u) {
            continue;
        }
        const GraphEdge& edge = edges[e];
        for (size_t v = 0; v < edge.node_ids.size(); ++v) {
            const NodeId nid = edge.node_ids[v];
            if (claimed.count(nid) != 0) {
                continue;
            }

            Candidate c;
            c.edge = static_cast<EdgeId>(e);
            c.vertex = v;
            c.osm_node = nid;

            if (crossing_nodes.count(nid) != 0) {
                c.from_node_tag = true;
            } else {
                const auto it = crossing_way_nodes.find(nid);
                if (it == crossing_way_nodes.end()) {
                    continue;
                }
                c.from_node_tag = false;
                c.way = it->second;
            }

            claimed.insert(nid);
            candidates.push_back(c);
        }
    }

    if (candidates.empty()) {
        return out;
    }

    // ------------------------------------------------------------------------
    // Resolve each candidate onto its edge's centerline.
    // ------------------------------------------------------------------------
    out.reserve(candidates.size());

    for (const Candidate& cand : candidates) {
        const GraphEdge& edge = edges[cand.edge];
        const Centerline& cl = centerlines[cand.edge];
        const RoadProfile& profile = profiles[cand.edge];

        const glm::dvec2 vertex = edge.polyline[cand.vertex];
        if (!is_finite(vertex)) {
            continue;
        }

        // The vertex index is turned into an arclength by PROJECTION, not by
        // accumulating polyline chords. build_centerline() smooths and
        // resamples, so the two parameterisations differ; the smoothed curve
        // still interpolates the surveyed vertices, so the projection lands
        // within ResampleConfig::max_smoothing_offset of the vertex.
        double dist = 0.0;
        double arclength = project_to_centerline(cl, vertex, dist);
        if (!std::isfinite(arclength)) {
            continue;
        }

        const double length = cl.length();
        const double lo = cl.stations.front().arclength;

        // --------------------------------------------------------------------
        // Is it at a junction, and if so which one?
        //
        // A crossing node at a vertex END of the edge is at a graph node by
        // construction: P1 splits every way at every graph node, so an interior
        // vertex never is one.
        // --------------------------------------------------------------------
        GraphNodeId node = kInvalidId;
        if (cand.vertex == 0) {
            node = edge.from;
        } else if (cand.vertex + 1 == edge.node_ids.size()) {
            node = edge.to;
        }

        /// A node a junction is actually solved at, and so cut back around
        const auto is_junction_node = [&](GraphNodeId n) {
            return n != kInvalidId && n < graph.nodes().size() && graph.node(n).degree() >= 3;
        };

        bool at_junction = is_junction_node(node);

        // "Inside the trim" needs a trim to be inside, AND the end it is measured
        // from has to be a junction. With the junction solve disabled every trim
        // is zero, and a crossing node that merely SPLIT the carriageway -- which
        // is what a footway sharing a node does, at a plain degree-2 node --
        // would otherwise land at arclength 0 and be read as a junction crossing,
        // then be set back 1.5 m from a junction that is not there.
        //
        // A non-zero trim is not by itself evidence of a junction: a degree-2
        // node whose two profiles differ gets a TAPER, and build_profile_taper()
        // writes trims of up to 30 m a side through the same GraphEdge fields. An
        // ordinary lane drop would otherwise teleport a mid-block crossing tens
        // of metres down the road, to a node with no junction plane to be set
        // back from and no curb ring to drop.
        const double trim_from = std::max(0.0, edge.trim_from);
        const double trim_to = std::max(0.0, edge.trim_to);
        const bool near_from = trim_from > kZeroLength && is_junction_node(edge.from) &&
                               arclength <= lo + trim_from + kArclengthEpsilon;
        const bool near_to = trim_to > kZeroLength && is_junction_node(edge.to) &&
                             arclength >= length - trim_to - kArclengthEpsilon;

        bool at_from_end = false;
        if (near_from && near_to) {
            at_from_end = (arclength - lo) <= (length - arclength);
        } else {
            at_from_end = near_from;
        }

        if (near_from || near_to) {
            at_junction = true;
            node = at_from_end ? edge.from : edge.to;
        } else if (at_junction) {
            at_from_end = (cand.vertex == 0);
        }

        // Push a junction crossing back off the trim station, so it and the
        // stop line on the same arm stack rather than overlap.
        if (at_junction) {
            const double setback = std::max(0.0, static_cast<double>(cfg.setback));
            const double want = at_from_end ? (lo + trim_from + setback)
                                            : (length - trim_to - setback);
            arclength = std::clamp(want, lo, length);
        }

        const CenterlineSample sample = sample_centerline(cl, arclength);
        if (!sample.valid) {
            continue;
        }

        // --------------------------------------------------------------------
        // The axis across the road.
        //
        // ALWAYS the carriageway's own left normal at the crossing, never the
        // bearing of the footway the crossing was found from. That is the frozen
        // contract in crossings.hpp, and it is what real paint does: a zebra is
        // laid square to the road it crosses however the footway approaching it
        // was drawn, because the stripes divide the traffic lanes, not the
        // pedestrian desire line. The skew of a footway is preserved by the
        // crossing being ACCEPTED at all, not by the stripes leaning.
        //
        // Keeping the surveyed angle instead would also make Crossing::width
        // something other than RoadProfile::carriageway_width(), since an oblique
        // run spans further along its own axis for the same road, and every
        // consumer of that field -- dropped_kerb_spans() above all -- takes it to
        // BE the carriageway.
        // --------------------------------------------------------------------
        const glm::dvec2 axis = sample.normal;

        // The stripes reach exactly across the carriageway envelope, which is the
        // span RoadProfile::left_edge_offset() centres on the centreline: the
        // axis is square to the road, so the run along the axis and the run
        // across the carriageway are the same distance and each stripe quad is
        // extruded along the road rather than leaning off it. build_crossing()
        // fits whole stripes inside that span, and around any raised island in
        // the middle of it.
        const CrossExtent extent = measure_extent(profile);
        if (!extent.valid) {
            continue;
        }
        const double usable = 2.0 * extent.half;

        Crossing c;
        c.node = at_junction ? node : ((cand.vertex == 0 || cand.vertex + 1 == edge.node_ids.size())
                                           ? node
                                           : kInvalidId);
        c.edge = cand.edge;
        c.arclength = arclength;
        c.position = sample.position;
        c.axis = axis;
        c.width = static_cast<float>(usable);
        c.island_left = static_cast<float>(extent.island_left);
        c.island_right = static_cast<float>(extent.island_right);
        c.height = height_at(elevation, cand.edge, sample);
        c.at_junction = at_junction;

        out.push_back(c);
    }

    // ------------------------------------------------------------------------
    // Deduplicate and order.
    //
    // The same physical crossing is frequently mapped twice, as a node on the
    // carriageway AND as a way through it. The two land at different OSM nodes,
    // so the per-node claim above does not catch them; they land within a stripe
    // width of each other on the same edge, which this does.
    // ------------------------------------------------------------------------
    std::sort(out.begin(), out.end(), [](const Crossing& a, const Crossing& b) {
        if (a.edge != b.edge) {
            return a.edge < b.edge;
        }
        if (a.arclength != b.arclength) {
            return a.arclength < b.arclength;
        }
        // Node-derived first, so the dedup below keeps it.
        return a.node != kInvalidId && b.node == kInvalidId;
    });

    const double dedup_window = std::max(static_cast<double>(cfg.stripe_width),
                                         kArclengthEpsilon);
    std::vector<Crossing> deduped;
    deduped.reserve(out.size());
    for (const Crossing& c : out) {
        if (!deduped.empty() && deduped.back().edge == c.edge &&
            std::fabs(deduped.back().arclength - c.arclength) <= dedup_window) {
            continue;
        }
        deduped.push_back(c);
    }

    return deduped;
}

// ============================================================================
// build_crossing
// ============================================================================

Mesh build_crossing(const Crossing& c, const CrossingConfig& cfg) {
    Mesh mesh;

    if (!cfg.emit_zebra) {
        return mesh;
    }
    if (!is_finite(c.position) || !is_finite(c.axis) || !std::isfinite(c.height) ||
        !std::isfinite(c.width)) {
        return mesh;
    }

    const glm::dvec2 axis = safe_normalise(c.axis, glm::dvec2(0.0));
    if (glm::length(axis) < 0.5) {
        return mesh;
    }
    const glm::dvec2 travel = travel_of(axis);

    const double stripe = std::max(0.0, static_cast<double>(cfg.stripe_width));
    const double gap = std::max(0.0, static_cast<double>(cfg.stripe_gap));
    const double depth = std::max(0.0, static_cast<double>(cfg.crossing_depth));
    if (stripe <= kZeroLength || depth <= kZeroLength) {
        return mesh;
    }

    // One run across the whole extent, or two when a raised island splits it.
    // Nothing is painted over an island: the stripes would sit under its top and
    // inside its curb faces, invisible from every angle.
    const double half_extent = 0.5 * static_cast<double>(c.width);
    struct Run {
        double lo = 0.0;
        double hi = 0.0;
    };
    std::vector<Run> runs;

    const double island_left = static_cast<double>(c.island_left);
    const double island_right = static_cast<double>(c.island_right);
    const bool split = std::isfinite(island_left) && std::isfinite(island_right) &&
                       island_left - island_right > kZeroLength &&
                       island_left < half_extent && island_right > -half_extent;
    if (split) {
        runs.push_back(Run{ island_left, half_extent });
        runs.push_back(Run{ -half_extent, island_right });
    } else {
        runs.push_back(Run{ -half_extent, half_extent });
    }

    std::vector<double> centres;
    for (const Run& run : runs) {
        const double run_width = run.hi - run.lo;
        const double run_mid = 0.5 * (run.lo + run.hi);
        for (const double lateral : fit_stripes(run_width, stripe, gap)) {
            centres.push_back(run_mid + lateral);
        }
    }
    if (centres.empty()) {
        return mesh;
    }

    // One quad per repeat, each mapping the FULL sprite rect: an atlas sub-rect
    // cannot wrap, so a zebra is never one stretched quad. See marking_atlas.hpp.
    const SpriteRect rect = sprite_rect(MarkingSprite::ZebraStripe);

    const double half_stripe = stripe * 0.5;
    const double half_depth = depth * 0.5;
    const double height = static_cast<double>(c.height);

    mesh.vertices.reserve(centres.size() * 4);
    mesh.indices.reserve(centres.size() * 6);

    for (const double lateral : centres) {
        const glm::dvec2 mid = c.position + axis * lateral;

        // Corner naming follows the atlas orientation contract:
        //   u0 -> LEFT of travel, u1 -> RIGHT of travel
        //   v0 -> DOWNSTREAM end, v1 -> UPSTREAM end
        // Left and right come from Crossing::axis, which is the profile's own
        // left-of-travel direction, NOT from a cross product -- the world
        // mapping (x, y) -> (x, h, -y) flips handedness, and deriving the side
        // from a cross product mirrors every asymmetric sprite.
        const glm::dvec2 ld = mid + axis * half_stripe + travel * half_depth;
        const glm::dvec2 rd = mid - axis * half_stripe + travel * half_depth;
        const glm::dvec2 ru = mid - axis * half_stripe - travel * half_depth;
        const glm::dvec2 lu = mid + axis * half_stripe - travel * half_depth;

        const auto base = static_cast<uint32_t>(mesh.vertices.size());

        auto push = [&](const glm::dvec2& p, float u, float v) {
            Vertex vert{};
            vert.position = to_world(p, height);
            vert.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            vert.uv = glm::vec2(u, v);
            vert.color = glm::vec4(1.0f);
            mesh.vertices.push_back(vert);
        };

        push(ld, rect.u0, rect.v0);     // base + 0
        push(rd, rect.u1, rect.v0);     // base + 1
        push(ru, rect.u1, rect.v1);     // base + 2
        push(lu, rect.u0, rect.v1);     // base + 3

        // Winding: cross(p1 - p0, p2 - p0) must point +Y for an upward face
        // once (x, y) -> (x, h, -y) has flipped handedness. That is the reverse
        // of the local-2D cycle, which is why these read 0,2,1 and 0,3,2.
        const glm::dvec3 p0(mesh.vertices[base + 0].position);
        const glm::dvec3 p1(mesh.vertices[base + 2].position);
        const glm::dvec3 p2(mesh.vertices[base + 1].position);
        const glm::dvec3 face = glm::cross(p1 - p0, p2 - p0);
        if (glm::dot(face, face) <= kDegenerateCrossSq) {
            mesh.vertices.resize(base);
            continue;
        }

        mesh.indices.push_back(base + 0);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base + 1);

        mesh.indices.push_back(base + 0);
        mesh.indices.push_back(base + 3);
        mesh.indices.push_back(base + 2);
    }

    if (mesh.indices.empty()) {
        mesh.clear();
        return mesh;
    }

    mesh.submeshes.push_back(
        SubMesh{ 0u, static_cast<uint32_t>(mesh.indices.size()), MaterialId::Markings });
    mesh.sort_submeshes_by_material();
    mesh.compute_bounds();
    mesh.compute_tangents();
    return mesh;
}

// ============================================================================
// dropped_kerb_spans
// ============================================================================

std::vector<DroppedKerbSpan> dropped_kerb_spans(const std::vector<Crossing>& crossings,
                                                GraphNodeId node,
                                                glm::dvec2 junction_center,
                                                const CrossingConfig& cfg) {
    std::vector<DroppedKerbSpan> out;

    if (!cfg.emit_dropped_kerbs || node == kInvalidId || !is_finite(junction_center)) {
        return out;
    }

    const double half_drop = std::max(0.0, static_cast<double>(cfg.dropped_kerb_width)) * 0.5;
    if (half_drop <= kZeroLength) {
        return out;
    }

    std::vector<Arc> arcs;
    arcs.reserve(crossings.size() * 2);

    for (const Crossing& c : crossings) {
        if (!c.at_junction || c.node != node) {
            continue;
        }
        if (!(c.width > 0.0f) || !is_finite(c.position) || !is_finite(c.axis)) {
            continue;
        }

        const glm::dvec2 axis = safe_normalise(c.axis, glm::dvec2(0.0));
        if (glm::length(axis) < 0.5) {
            continue;
        }
        const double half_width = static_cast<double>(c.width) * 0.5;

        // --------------------------------------------------------------------
        // Where the ring actually is.
        //
        // The crossing stands CrossingConfig::setback metres OUTSIDE the arm's
        // cut line and its kerb point is at half the carriageway. Measured from
        // the junction centre, that bearing is strictly inside the arm's own
        // MOUTH -- the sector between the arm's two carriage corners -- and
        // build_curb_ring() emits nothing there: the ring runs from one arm's
        // corner round the fillet to the next arm's corner. A span laid on the
        // crossing itself therefore asks for a drop in a gap, and gets none.
        //
        // So the span is laid where there is kerb to cut. Walking back along the
        // arm by the same setback that pushed the crossing out lands on the arm's
        // carriage corner; the span then runs from that corner INTO the fillet by
        // the drop width. The fillet is tangent to the arm's kerb line at the
        // corner, so a step along the arm is a step along the ring to first
        // order. That is the kerb a pedestrian at this crossing steps off.
        //
        // A drop in the arm's own CORRIDOR kerb -- the strip running past the
        // corner, away from the junction -- is not expressible here at all; see
        // the note on mid-block crossings in the header.
        // --------------------------------------------------------------------
        const glm::dvec2 to_center = junction_center - c.position;
        const double centre_distance = std::sqrt(glm::dot(to_center, to_center));

        glm::dvec2 along_kerb = travel_of(axis);
        bool oriented = false;
        if (centre_distance > kZeroLength) {
            const glm::dvec2 inward = to_center / centre_distance;
            if (glm::dot(along_kerb, inward) < 0.0) {
                along_kerb = -along_kerb;
            }
            oriented = true;
        }

        // Never walk past the junction centre: the drop belongs to this arm's
        // corner, not to the far side of the node.
        const double setback = std::max(0.0, static_cast<double>(cfg.setback));
        const double reach = oriented ? std::max(0.0, glm::dot(to_center, along_kerb)) : 0.0;
        const double step = std::min(setback, reach);
        const glm::dvec2 mouth = oriented ? (c.position + along_kerb * step) : c.position;
        const double walk = std::min(2.0 * half_drop, std::max(0.0, (reach - step) * 0.9));

        // A crossing needs a drop at BOTH ends, one per side of the carriageway.
        for (const double side : { 1.0, -1.0 }) {
            const glm::dvec2 corner = mouth + axis * (side * half_width);

            // Without an orientation there is no arm to walk along, so the span
            // stays symmetric about the kerb point, which is the best that can be
            // said about a crossing sitting on the junction centre.
            const glm::dvec2 a = (oriented && walk > kZeroLength)
                                     ? corner
                                     : corner - along_kerb * half_drop;
            const glm::dvec2 b = (oriented && walk > kZeroLength)
                                     ? corner + along_kerb * walk
                                     : corner + along_kerb * half_drop;

            Arc arc;
            if (arc_between(junction_center, a, b, arc)) {
                arcs.push_back(arc);
            }
        }
    }

    if (arcs.empty()) {
        return out;
    }

    return to_spans(merge_arcs(std::move(arcs)), cfg.dropped_kerb_height);
}

// ============================================================================
// driveway_kerb_spans
// ============================================================================

std::vector<DroppedKerbSpan> driveway_kerb_spans(const RoadGraph& graph,
                                                 const ParsedOSMData& data,
                                                 GraphNodeId node,
                                                 glm::dvec2 junction_center,
                                                 const std::vector<RoadProfile>& profiles,
                                                 const CrossingConfig& cfg) {
    std::vector<DroppedKerbSpan> out;

    if (!cfg.emit_dropped_kerbs || node == kInvalidId || node >= graph.nodes().size()) {
        return out;
    }
    if (!is_finite(junction_center)) {
        return out;
    }
    if (profiles.size() != graph.edges().size()) {
        return out;
    }

    const GraphNode& n = graph.node(node);
    if (n.degree() < 3) {
        return out;     // no ring is built below degree 3, so there is no kerb to cut
    }

    // A node every one of whose arms is a driveway has no parent road, so it has
    // no kerb line that a driveway interrupts.
    bool has_parent = false;
    for (const Arm& arm : n.arms) {
        if (arm.edge < graph.edges().size() && !is_driveway(graph.edge(arm.edge), data)) {
            has_parent = true;
            break;
        }
    }
    if (!has_parent) {
        return out;
    }

    const double flare = std::max(0.0, static_cast<double>(cfg.dropped_kerb_width));

    std::vector<Arc> arcs;
    arcs.reserve(n.arms.size());

    for (const Arm& arm : n.arms) {
        if (arm.edge >= graph.edges().size()) {
            continue;
        }
        const GraphEdge& edge = graph.edge(arm.edge);
        if (!is_driveway(edge, data)) {
            continue;
        }

        const glm::dvec2 dir = graph.arm_direction(arm);
        if (glm::length(dir) < 0.5) {
            continue;
        }

        double half_width = 0.5 * static_cast<double>(profiles[arm.edge].carriageway_width());
        if (!(half_width > 0.0)) {
            half_width = 0.5 * static_cast<double>(edge.width);
        }
        if (!(half_width > 0.0)) {
            continue;
        }

        // The mouth sits at the arm's trim station, which is where the junction
        // polygon -- and so the ring's inner boundary -- crosses the arm. With no
        // solved trim, fall back to the arm's own half width, which is the
        // distance a square junction of that arm alone would trim to.
        const double trim = arm.at_start ? edge.trim_from : edge.trim_to;
        const double reach = (trim > kZeroLength) ? trim : half_width;

        const glm::dvec2 mouth = n.position + dir * reach;
        const glm::dvec2 along_kerb(-dir.y, dir.x);
        const double half_span = half_width + flare;

        Arc arc;
        if (arc_between(junction_center, mouth - along_kerb * half_span,
                        mouth + along_kerb * half_span, arc)) {
            arcs.push_back(arc);
        }
    }

    if (arcs.empty()) {
        return out;
    }

    return to_spans(merge_arcs(std::move(arcs)), cfg.dropped_kerb_height);
}

} // namespace stratum::osm::road
