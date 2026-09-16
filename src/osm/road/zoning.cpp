// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "osm/road/zoning.hpp"

#include <clipper2/clipper.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace stratum::osm::road {

using scene::AttributeKey;
using scene::AttributeObject;
using scene::AttributeQuery;
using scene::AttributeSource;
using scene::AttributeStatus;
using scene::AttributeStore;
using scene::AttributeTarget;
using scene::AttributeValue;
using scene::CommandStack;
using scene::LayerRef;

namespace {

// ============================================================================
// The one invariant tying the enum to its table
// ============================================================================

/**
 * @brief kAllZoneTypes[i] must be the enumerator whose ordinal is i
 *
 * The solver indexes its per-zone pools by `static_cast<size_t>(zone)` and reads
 * the winner back as `kAllZoneTypes[best]`. Those two are only the same zone
 * while the table is in ordinal order, and an enumerator inserted in the middle
 * of ZoneType without the same edit to the table would silently start zoning
 * every block as its neighbour in the enumeration -- a wrong answer that looks
 * entirely plausible. Caught here at compile time instead.
 */
[[nodiscard]] constexpr bool zone_table_is_ordinal_ordered() {
    for (size_t i = 0; i < kZoneTypeCount; ++i) {
        if (static_cast<size_t>(kAllZoneTypes[i]) != i) {
            return false;
        }
    }
    return true;
}

static_assert(kAllZoneTypes.size() == kZoneTypeCount,
              "kZoneTypeCount must match kAllZoneTypes");
static_assert(zone_table_is_ordinal_ordered(),
              "kAllZoneTypes must list ZoneType enumerators in ordinal order");

// ============================================================================
// Clipper2 bridge
// ============================================================================

/**
 * @brief Largest local coordinate that survives the scale to int64, in scaled units
 *
 * Clipper2 works in int64 and squares coordinates internally, so the usable
 * range is nearer 2^31 than 2^63. The same cap junction_curb.cpp applies, for
 * the same reason: a corrupt coordinate is refused rather than silently wrapped
 * into a polygon on the other side of the world, which would zone a block from
 * a landuse area a thousand kilometres away.
 */
constexpr double kMaxScaledCoordinate = 1.0e9;

/**
 * @brief Convert a ring of metres into a Clipper2 path
 *
 * Rounds to nearest rather than truncating. Truncation biases every coordinate
 * towards zero by up to one unit, which shrinks every polygon slightly and
 * biases every coverage fraction downwards -- small, but systematic, and a
 * systematic bias against coverage is a systematic bias towards BelowFloor.
 *
 * @return False when the ring cannot be represented, or collapses to under
 *         three distinct points after quantisation.
 */
[[nodiscard]] bool to_path(const std::vector<glm::dvec2>& ring, double scale,
                           Clipper2Lib::Path64& out) {
    out.clear();
    if (ring.size() < 3 || !(scale > 0.0)) {
        return false;
    }

    out.reserve(ring.size());
    for (const glm::dvec2& p : ring) {
        const double sx = p.x * scale;
        const double sy = p.y * scale;
        if (!std::isfinite(sx) || !std::isfinite(sy) || std::fabs(sx) > kMaxScaledCoordinate
            || std::fabs(sy) > kMaxScaledCoordinate) {
            return false;
        }
        const Clipper2Lib::Point64 q(static_cast<int64_t>(std::llround(sx)),
                                     static_cast<int64_t>(std::llround(sy)));
        // Millimetre rounding can collapse two ring vertices onto one point.
        if (!out.empty() && out.back().x == q.x && out.back().y == q.y) {
            continue;
        }
        out.push_back(q);
    }

    // The closing edge can be a duplicate too, whether or not the caller
    // repeated the first point.
    while (out.size() >= 2 && out.front().x == out.back().x && out.front().y == out.back().y) {
        out.pop_back();
    }

    return out.size() >= 3;
}

/**
 * @brief The whole of a source area as clip paths: outer ring, then holes
 *
 * Holes go in as plain paths in whatever orientation they arrive in. The clip is
 * run with `FillRule::EvenOdd`, which treats a ring inside another ring as a
 * hole regardless of winding -- OSM multipolygon members carry no orientation
 * guarantee, and a NonZero fill with a same-wound inner ring would fill the hole
 * in rather than cut it out, so a park would swallow its own lake.
 */
[[nodiscard]] bool to_clip_paths(const ZoneSourceArea& source, double scale,
                                 Clipper2Lib::Paths64& out) {
    out.clear();
    Clipper2Lib::Path64 outer;
    if (!to_path(source.ring, scale, outer)) {
        return false;
    }
    out.push_back(std::move(outer));

    for (const std::vector<glm::dvec2>& hole : source.holes) {
        Clipper2Lib::Path64 path;
        // A hole that cannot be represented is dropped, not fatal: losing a hole
        // over-zones a little, losing the whole area zones nothing at all.
        if (to_path(hole, scale, path)) {
            out.push_back(std::move(path));
        }
    }
    return true;
}

/// Axis-aligned box in metres. The cheap reject in front of every Clipper2 call.
struct Aabb {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    bool valid = false;

    [[nodiscard]] bool overlaps(const Aabb& other) const {
        return valid && other.valid && min_x <= other.max_x && other.min_x <= max_x
               && min_y <= other.max_y && other.min_y <= max_y;
    }
};

[[nodiscard]] Aabb ring_box(const std::vector<glm::dvec2>& ring) {
    Aabb box;
    for (const glm::dvec2& p : ring) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            return Aabb{};
        }
        if (!box.valid) {
            box = Aabb{p.x, p.y, p.x, p.y, true};
            continue;
        }
        box.min_x = std::min(box.min_x, p.x);
        box.min_y = std::min(box.min_y, p.y);
        box.max_x = std::max(box.max_x, p.x);
        box.max_y = std::max(box.max_y, p.y);
    }
    return box;
}

// ============================================================================
// Per-zone accumulation
// ============================================================================

/**
 * @brief Everything one zone contributed to one target
 *
 * Indexed by ZoneType ordinal, never keyed by a hash map. The winner is chosen
 * by scanning this array in enumerator order, so the scan itself introduces no
 * dependence on the order the sources arrived in -- which is the property the
 * file comment promises and which an unordered_map would quietly break.
 */
struct ZonePool {
    /// Intersection pieces, merged at the end rather than summed. See finish_pool().
    Clipper2Lib::Paths64 parts;

    /**
     * @brief Area of the largest single source that contributed, in SQUARE METRES
     *
     * Square metres, like `area` below. An earlier version held scaled units
     * squared while the epsilon it was compared against was in metres -- a factor
     * of a million out, so that comparison silently meant something different
     * from the one in pool_beats() and no test could see either. The epsilon is
     * gone now; one unit for every area in this file is what keeps the next one
     * from coming back.
     */
    double best_source_area = 0.0;

    /// That source's OSM id: the provenance, and the order-independent tie-break
    int64_t best_source_id = 0;

    bool present = false;

    /// Merged area in square metres. Filled by finish_pool().
    double area = 0.0;
};

/**
 * @brief Union the pieces instead of summing their areas
 *
 * Summing is the obvious thing and it double-counts. Real OSM has overlapping
 * landuse polygons of the same type -- an estate mapped twice, a district
 * polygon plus the sub-areas inside it -- and a sum would report a block as 180%
 * residential. Clamping that to 100% hides the symptom and still leaves the
 * runner-up comparison wrong, because the winner's inflated figure beats a
 * genuine competitor it should have lost to. One union per zone that
 * contributed, at most twelve per target, is cheap and is simply correct.
 *
 * `FillRule::NonZero` is right here, and EvenOdd would not be: these paths come
 * out of Clipper2 as solution polygons, where an outer and its holes are wound
 * oppositely, and NonZero is the rule that convention is built for.
 */
void finish_pool(ZonePool& pool, double scale) {
    if (!pool.present || pool.parts.empty()) {
        pool.area = 0.0;
        return;
    }
    const Clipper2Lib::Paths64 merged =
        Clipper2Lib::Union(pool.parts, Clipper2Lib::FillRule::NonZero);
    pool.area = std::fabs(Clipper2Lib::Area(merged)) / (scale * scale);
}

/**
 * @brief Order two pools: more area first, then smaller source id, then enumerator
 *
 * Nothing here reads the position of a source in the caller's vector. That is
 * deliberate and is the whole point: the input order is the importer's iteration
 * order, and a zoning that flips when a hash map rehashes is not reproducible
 * and cannot be golden-tested.
 *
 * The area comparison is EXACT, with no tolerance, and that is the same point
 * made twice. "Equal to within epsilon" is not transitive: three pools a few
 * tenths of a square millimetre apart can have a ~ b, b ~ c and c > a, and a
 * maximum taken over a non-transitive relation depends on the order the
 * candidates are visited in. An exact comparison is a total order on
 * (area, source id, enumerator), so the winner is the same however the sources
 * arrived. Clipper2 computes these areas from integer coordinates, so two
 * genuinely identical overlaps produce identical doubles and a real tie is still
 * recognised as one -- which is what makes paying nothing for the tolerance
 * affordable.
 */
[[nodiscard]] bool pool_beats(const ZonePool& a, ZoneType a_zone, const ZonePool& b,
                              ZoneType b_zone) {
    if (!a.present) {
        return false;
    }
    if (!b.present) {
        return true;
    }
    if (a.area != b.area) {
        return a.area > b.area;
    }
    if (a.best_source_id != b.best_source_id) {
        return a.best_source_id < b.best_source_id;
    }
    return static_cast<uint8_t>(a_zone) < static_cast<uint8_t>(b_zone);
}

// ============================================================================
// Attribute decoding
// ============================================================================

/**
 * @brief Turn an attribute read into a zone read
 *
 * The three-way split the file comment describes lives here and nowhere else, so
 * resolve_zone() and peek_zone() cannot disagree about what "unzoned" means.
 */
[[nodiscard]] ZoneQuery decode_zone(const AttributeQuery& query) {
    ZoneQuery out;

    if (query.status == AttributeStatus::Missing || query.value == nullptr
        || query.source == AttributeSource::None) {
        // Unzoned: nothing on any rung. Distinct from a rung holding "unknown".
        return out;
    }

    out.source = query.source;
    out.layer = query.layer;

    const std::string* text = query.value->as_string();
    if (text == nullptr) {
        // A number or a bool in the zoning slot. Report what it is rather than
        // converting: a caller seeing Unzoned here would go looking for a
        // missing inference instead of for the code that wrote a double.
        out.status = ZoneStatus::Unrecognised;
        out.raw = query.value->to_display_string();
        return out;
    }

    // The string itself, unquoted -- to_display_string() adds quotes so that the
    // string "true" is distinguishable from the bool, which is right for the
    // inspector and wrong for a value a caller may want to compare.
    out.raw = *text;

    const std::optional<ZoneType> parsed = zone_type_from_name(*text);
    if (!parsed.has_value()) {
        out.status = ZoneStatus::Unrecognised;
        return out;
    }

    out.status = ZoneStatus::Zoned;
    out.zone = *parsed;
    return out;
}

[[nodiscard]] AttributeValue zone_value(ZoneType zone) {
    return AttributeValue::from_string(std::string(zone_type_name(zone)));
}

} // namespace

// ============================================================================
// Zone vocabulary
// ============================================================================

std::string_view zone_type_name(ZoneType zone) {
    switch (zone) {
    case ZoneType::Residential: return "residential";
    case ZoneType::Commercial:  return "commercial";
    case ZoneType::Retail:      return "retail";
    case ZoneType::Industrial:  return "industrial";
    case ZoneType::Park:        return "park";
    case ZoneType::Forest:      return "forest";
    case ZoneType::Grass:       return "grass";
    case ZoneType::Farmland:    return "farmland";
    case ZoneType::Cemetery:    return "cemetery";
    case ZoneType::Parking:     return "parking";
    case ZoneType::Water:       return "water";
    case ZoneType::Unknown:     return "unknown";
    }
    // Unreachable for a declared enumerator. "unknown" is the honest answer: it
    // is a real zone meaning "classified as nothing in particular", which is
    // what a value outside the enumeration is.
    return "unknown";
}

std::optional<ZoneType> zone_type_from_name(std::string_view name) {
    // Scans the name table rather than carrying a second, reversed table. A
    // reversed table is a second place to forget an enumerator, and the two
    // would drift silently: the write path would produce a spelling the read
    // path reported as Unrecognised. Twelve string compares is not a cost worth
    // that risk, and the caller that cares hoists the parse.
    for (const ZoneType zone : kAllZoneTypes) {
        if (zone_type_name(zone) == name) {
            return zone;
        }
    }
    return std::nullopt;
}

std::optional<ZoneType> zone_from_area_type(AreaType type) {
    switch (type) {
    case AreaType::Water:       return ZoneType::Water;
    case AreaType::Park:        return ZoneType::Park;
    case AreaType::Forest:      return ZoneType::Forest;
    case AreaType::Grass:       return ZoneType::Grass;
    case AreaType::Parking:     return ZoneType::Parking;
    case AreaType::Commercial:  return ZoneType::Commercial;
    case AreaType::Residential: return ZoneType::Residential;
    case AreaType::Industrial:  return ZoneType::Industrial;
    case AreaType::Farmland:    return ZoneType::Farmland;
    case AreaType::Cemetery:    return ZoneType::Cemetery;
    case AreaType::Unknown:     break;
    }
    // An area the parser could not classify zones nothing. Mapping it to
    // ZoneType::Unknown would be a lie of a specific and damaging kind: every
    // unclassified polygon in the extract would start actively claiming the land
    // under it as "deliberately unzoned", beating a real landuse area next to it
    // whenever it happened to cover more of the block.
    return std::nullopt;
}

std::string_view zone_status_name(ZoneStatus status) {
    switch (status) {
    case ZoneStatus::Unzoned:      return "Unzoned";
    case ZoneStatus::Zoned:        return "Zoned";
    case ZoneStatus::Unrecognised: return "Unrecognised";
    }
    return "?";
}

std::string_view zone_outcome_name(ZoneOutcome outcome) {
    switch (outcome) {
    case ZoneOutcome::NoOverlap:  return "NoOverlap";
    case ZoneOutcome::BelowFloor: return "BelowFloor";
    case ZoneOutcome::Degenerate: return "Degenerate";
    case ZoneOutcome::Inferred:   return "Inferred";
    }
    return "?";
}

// ============================================================================
// Reading a zone off the store
// ============================================================================

AttributeKey zone_key(AttributeStore& store) { return store.intern(kZoneAttributeName); }

AttributeKey find_zone_key(const AttributeStore& store) {
    return store.find_key(kZoneAttributeName);
}

ZoneQuery resolve_zone(const AttributeStore& store, AttributeObject object, LayerRef layer) {
    const AttributeKey key = find_zone_key(store);
    if (!key.valid()) {
        // Nothing has ever written a zone into this document, so there is no key
        // to resolve. Unzoned, and deliberately NOT an error: an empty document
        // is a legitimate state.
        return ZoneQuery{};
    }
    return decode_zone(store.resolve(object, key, layer));
}

ZoneQuery peek_zone(const AttributeStore& store, const AttributeTarget& target) {
    return decode_zone(store.peek(target));
}

AttributeTarget painted_zone_target(AttributeStore& store, AttributeObject object) {
    return AttributeTarget::user(object, zone_key(store));
}

AttributeTarget inferred_zone_target(AttributeStore& store, AttributeObject object) {
    return AttributeTarget::object(object, zone_key(store));
}

AttributeTarget layer_zone_target(AttributeStore& store, LayerRef layer) {
    return AttributeTarget::layer(layer, zone_key(store));
}

AttributeTarget default_zone_target(AttributeStore& store) {
    return AttributeTarget::schema_default(zone_key(store));
}

// ============================================================================
// Writing a zone
// ============================================================================

bool paint_zone(CommandStack& stack, AttributeStore& store, AttributeObject object, ZoneType zone) {
    return scene::set_attribute(stack, store, painted_zone_target(store, object),
                                zone_value(zone));
}

bool clear_painted_zone(CommandStack& stack, AttributeStore& store, AttributeObject object) {
    return scene::clear_attribute(stack, store, painted_zone_target(store, object));
}

bool set_inferred_zone(CommandStack& stack, AttributeStore& store, AttributeObject object,
                       ZoneType zone) {
    return scene::set_attribute(stack, store, inferred_zone_target(store, object),
                                zone_value(zone));
}

bool clear_inferred_zone(CommandStack& stack, AttributeStore& store, AttributeObject object) {
    return scene::clear_attribute(stack, store, inferred_zone_target(store, object));
}

bool set_layer_zone(CommandStack& stack, AttributeStore& store, LayerRef layer, ZoneType zone) {
    return scene::set_attribute(stack, store, layer_zone_target(store, layer), zone_value(zone));
}

bool set_default_zone(CommandStack& stack, AttributeStore& store, ZoneType zone) {
    return scene::set_attribute(stack, store, default_zone_target(store), zone_value(zone));
}

bool load_inferred_zone(AttributeStore& store, AttributeObject object, ZoneType zone) {
    return store.load_value(inferred_zone_target(store, object), zone_value(zone));
}

// ============================================================================
// Inference input
// ============================================================================

std::vector<ZoneSourceArea> zone_sources_from_areas(const std::vector<Area>& areas) {
    std::vector<ZoneSourceArea> sources;
    sources.reserve(areas.size());

    for (const Area& area : areas) {
        const std::optional<ZoneType> zone = zone_from_area_type(area.type);
        if (!zone.has_value() || area.polygon.size() < 3) {
            continue;
        }
        ZoneSourceArea source;
        source.zone = *zone;
        source.ring = area.polygon;
        source.holes = area.holes;
        source.source_id = area.osm_id;
        sources.push_back(std::move(source));
    }

    return sources;
}

ZoneTarget zone_target_from_block(const Block& block, AttributeObject object) {
    ZoneTarget target;
    target.object = object;
    target.ring = block.ring;
    // The holes are not optional decoration. blocks.hpp says Block::area already
    // has them subtracted, so a target that carried the ring alone would be a
    // LARGER parcel than the block -- and the land it gained belongs to the
    // block inside the cul-de-sac bulb, whose own landuse would then vote here.
    // The earlier version of this function dropped them and nothing noticed,
    // because nothing tested this function at all.
    target.holes = block.holes;
    return target;
}

// ============================================================================
// Inference
// ============================================================================

namespace {

/**
 * @brief The solve for one target, with the source boxes already computed
 *
 * Shared by infer_zone() and infer_zones() so the single-target and batch paths
 * cannot drift into giving different answers for the same input -- which is the
 * usual fate of a "fast path" written alongside a "simple path".
 *
 * @param clips Incremented once per Clipper2 intersection actually run
 */
[[nodiscard]] ZoneInference infer_one(const std::vector<glm::dvec2>& ring,
                                      const std::vector<std::vector<glm::dvec2>>& holes,
                                      const std::vector<ZoneSourceArea>& sources,
                                      const std::vector<Aabb>& source_boxes,
                                      const ZoningConfig& config, size_t& clips) {
    ZoneInference result;

    Clipper2Lib::Path64 outer;
    if (!to_path(ring, config.clipper_scale, outer)) {
        result.outcome = ZoneOutcome::Degenerate;
        return result;
    }

    // Outer ring first, then the holes, exactly as to_clip_paths() assembles a
    // source. The subject and the clip go through the same door so that "a hole
    // is land this polygon does not own" means one thing on both sides of the
    // intersection.
    Clipper2Lib::Paths64 subject_paths;
    subject_paths.reserve(holes.size() + 1);
    subject_paths.push_back(std::move(outer));
    for (const std::vector<glm::dvec2>& hole : holes) {
        Clipper2Lib::Path64 path;
        // Same trade as to_clip_paths(): a hole that cannot be represented is
        // dropped rather than made fatal. Losing a target hole over-states the
        // parcel by the bulb of one cul-de-sac; refusing the target zones the
        // whole block as nothing, and an unzoned block generates nothing at all.
        if (to_path(hole, config.clipper_scale, path)) {
            subject_paths.push_back(std::move(path));
        }
    }

    const double scale_sq = config.clipper_scale * config.clipper_scale;

    // Measured under the SAME fill rule as every numerator below. The signed
    // shoelace area of the outer path would be cheaper and would be a different
    // number: it ignores the holes, and for a self-intersecting ring -- which
    // blocks.hpp says happens whenever has_grade_separated_edge is set -- it
    // counts the doubled lobe of a bowtie once with each sign. An asymmetric
    // bowtie measured 3000 m^2 by shoelace and 3400 m^2 by the even-odd clip, so
    // coverage came out at 1.13 and was silently clamped to 1; a symmetric one
    // measured zero and was reported Degenerate although it encloses real land.
    // One rule for the numerator and the denominator, and both of those go away.
    const double target_area =
        std::fabs(Clipper2Lib::Area(
            Clipper2Lib::Union(subject_paths, Clipper2Lib::FillRule::EvenOdd)))
        / scale_sq;
    if (!(target_area > 0.0)) {
        // Three collinear points survive to_path() and enclose nothing, and so
        // does a ring a hole cancels exactly. Dividing by it would give an
        // infinite coverage.
        result.outcome = ZoneOutcome::Degenerate;
        return result;
    }
    result.target_area = target_area;

    // Holes only shrink the region, so the outer ring's box still contains it.
    const Aabb target_box = ring_box(ring);

    std::array<ZonePool, kZoneTypeCount> pools{};

    Clipper2Lib::Paths64 clip_paths;
    for (size_t i = 0; i < sources.size(); ++i) {
        const ZoneSourceArea& source = sources[i];
        if (!target_box.overlaps(source_boxes[i])) {
            continue;
        }
        if (!to_clip_paths(source, config.clipper_scale, clip_paths)) {
            continue;
        }

        ++clips;
        Clipper2Lib::Paths64 solution = Clipper2Lib::Intersect(
            subject_paths, clip_paths, Clipper2Lib::FillRule::EvenOdd);
        if (solution.empty()) {
            continue;
        }

        // Square metres from here on, so this comparison and the one in
        // pool_beats() are in the same unit. They were not always: this used to
        // stay in scaled units and be compared against a threshold in metres.
        const double piece = std::fabs(Clipper2Lib::Area(solution)) / scale_sq;
        if (!(piece > 0.0)) {
            // Touching along an edge only. Real: two landuse polygons sharing a
            // street boundary with the block between them.
            continue;
        }

        ZonePool& pool = pools[static_cast<size_t>(source.zone)];
        const bool first = !pool.present;
        // Largest piece, then smallest id -- compared exactly, for the reason
        // pool_beats() gives at length. This is the comparison that most needs
        // it: the sources are visited in the CALLER'S order, so a non-transitive
        // "near enough" here would let the importer's iteration order pick the
        // provenance. An exact `==` between two doubles is the intent, not an
        // oversight: both sides are Clipper2 areas off the same integer grid.
        if (first || piece > pool.best_source_area
            || (piece == pool.best_source_area && source.source_id < pool.best_source_id)) {
            pool.best_source_area = piece;
            pool.best_source_id = source.source_id;
        }
        pool.present = true;
        for (Clipper2Lib::Path64& path : solution) {
            pool.parts.push_back(std::move(path));
        }
    }

    for (ZonePool& pool : pools) {
        finish_pool(pool, config.clipper_scale);
    }

    size_t best = kZoneTypeCount;
    size_t second = kZoneTypeCount;
    for (size_t i = 0; i < kZoneTypeCount; ++i) {
        if (!pools[i].present || !(pools[i].area > 0.0)) {
            continue;
        }
        const ZoneType zone = kAllZoneTypes[i];
        if (best == kZoneTypeCount || pool_beats(pools[i], zone, pools[best], kAllZoneTypes[best])) {
            second = best;
            best = i;
        } else if (second == kZoneTypeCount
                   || pool_beats(pools[i], zone, pools[second], kAllZoneTypes[second])) {
            second = i;
        }
    }

    if (best == kZoneTypeCount) {
        result.outcome = ZoneOutcome::NoOverlap;
        return result;
    }

    // Not clamped, and deliberately not. Every pool area is the area of an
    // intersection WITH the even-odd subject, unioned, so it is a subset of the
    // region target_area measures and the ratio cannot exceed 1. The previous
    // std::min(1.0, ...) hid the fact that it could: the denominator was a
    // shoelace area under a different fill rule, a bowtie block reached 1.13, and
    // clamping made coverage and runner_up_coverage non-comparable, so the
    // contested flag was wrong too. Fixing the denominator is what makes the
    // clamp unnecessary; keeping it as well would only re-hide the next one.
    result.zone = kAllZoneTypes[best];
    result.coverage = pools[best].area / target_area;
    result.source_id = pools[best].best_source_id;

    if (second != kZoneTypeCount) {
        result.runner_up = kAllZoneTypes[second];
        result.runner_up_coverage = pools[second].area / target_area;
        result.contested =
            (result.coverage - result.runner_up_coverage) <= config.contested_margin;
    }

    if (result.coverage < config.min_coverage) {
        // The winner is kept in `zone` for diagnosis. The outcome is what stops
        // it being written; see the warning on ZoneInference.
        result.outcome = ZoneOutcome::BelowFloor;
        return result;
    }

    result.outcome = ZoneOutcome::Inferred;
    return result;
}

[[nodiscard]] std::vector<Aabb> source_boxes_of(const std::vector<ZoneSourceArea>& sources) {
    std::vector<Aabb> boxes;
    boxes.reserve(sources.size());
    for (const ZoneSourceArea& source : sources) {
        boxes.push_back(ring_box(source.ring));
    }
    return boxes;
}

} // namespace

ZoneInference infer_zone(const std::vector<glm::dvec2>& ring,
                         const std::vector<ZoneSourceArea>& sources, const ZoningConfig& config) {
    const std::vector<Aabb> boxes = source_boxes_of(sources);
    size_t clips = 0;
    static const std::vector<std::vector<glm::dvec2>> kNoHoles;
    return infer_one(ring, kNoHoles, sources, boxes, config, clips);
}

ZoneInference infer_target_zone(const ZoneTarget& target,
                                const std::vector<ZoneSourceArea>& sources,
                                const ZoningConfig& config) {
    const std::vector<Aabb> boxes = source_boxes_of(sources);
    size_t clips = 0;
    return infer_one(target.ring, target.holes, sources, boxes, config, clips);
}

ZoningReport infer_zones(const std::vector<ZoneTarget>& targets,
                         const std::vector<ZoneSourceArea>& sources, const ZoningConfig& config) {
    ZoningReport report;
    report.inferences.reserve(targets.size());
    report.stats.targets = targets.size();
    report.stats.sources = sources.size();
    report.stats.pairs = targets.size() * sources.size();

    // Computed once for the whole batch. This is the difference between a
    // prefilter and a per-pair bounding box rebuild, and on a city extract it is
    // the difference between the prefilter being free and it costing more than
    // the clips it saves.
    const std::vector<Aabb> boxes = source_boxes_of(sources);

    for (const ZoneTarget& target : targets) {
        ZoneInference inference =
            infer_one(target.ring, target.holes, sources, boxes, config, report.stats.clips);

        switch (inference.outcome) {
        case ZoneOutcome::Inferred:
            ++report.stats.inferred;
            if (inference.contested) {
                ++report.stats.contested;
            }
            break;
        case ZoneOutcome::NoOverlap:  ++report.stats.no_overlap; break;
        case ZoneOutcome::BelowFloor: ++report.stats.below_floor; break;
        case ZoneOutcome::Degenerate: ++report.stats.degenerate; break;
        }

        report.inferences.push_back(inference);
    }

    return report;
}

// ============================================================================
// Applying an inference
// ============================================================================

namespace {

/**
 * @brief Guard the pairing of a report with the targets it was made from
 *
 * The two vectors are parallel by contract and nothing in the types enforces it.
 * Handing the wrong report in would zone every block with its neighbour's
 * answer, which looks entirely plausible in a viewport and is almost
 * undebuggable, so it is refused loudly instead.
 */
[[nodiscard]] bool report_matches(const std::vector<ZoneTarget>& targets,
                                  const ZoningReport& report, const char* who) {
    if (report.inferences.size() == targets.size()) {
        return true;
    }
    spdlog::error("{}: report holds {} inferences for {} targets; refusing to zone anything", who,
                  report.inferences.size(), targets.size());
    return false;
}

} // namespace

size_t load_inferred_zones(AttributeStore& store, const std::vector<ZoneTarget>& targets,
                           const ZoningReport& report) {
    if (!report_matches(targets, report, "load_inferred_zones")) {
        return 0;
    }

    size_t written = 0;
    for (size_t i = 0; i < targets.size(); ++i) {
        if (!report.inferences[i].inferred()) {
            continue;
        }
        if (load_inferred_zone(store, targets[i].object, report.inferences[i].zone)) {
            ++written;
        }
    }
    return written;
}

size_t apply_inferred_zones(CommandStack& stack, AttributeStore& store,
                            const std::vector<ZoneTarget>& targets, const ZoningReport& report) {
    if (!report_matches(targets, report, "apply_inferred_zones")) {
        return 0;
    }

    // One transaction for the whole re-zone. Per-block undo steps would be
    // technically correct and useless: a user who re-runs zoning and dislikes the
    // result wants one undo, not four thousand.
    stack.begin_transaction("Apply zoning");

    size_t written = 0;
    for (size_t i = 0; i < targets.size(); ++i) {
        if (!report.inferences[i].inferred()) {
            continue;
        }
        const ZoneType zone = report.inferences[i].zone;

        // Read the Object rung before writing it. scene::set_attribute() never
        // compares -- SetAttributeCommand::apply() always succeeds -- so an
        // unchanged zone written here would still put a command in the
        // transaction and still record a step. The documented workflow is
        // re-running zoning after editing one street, where nearly every block
        // decides the same zone as last time, and the user would get an "Apply
        // zoning" entry whose undo visibly does nothing. Skipping leaves the
        // transaction empty and commit_transaction() then records no step.
        //
        // Deliberately compared as a ZoneType and not as text: a rung holding a
        // number, or a spelling this build cannot parse, reads as Unrecognised
        // and must be OVERWRITTEN, not mistaken for an unchanged value.
        const ZoneQuery current = peek_zone(store, inferred_zone_target(store, targets[i].object));
        if (current.status == ZoneStatus::Zoned && current.zone == zone) {
            continue;
        }

        if (set_inferred_zone(stack, store, targets[i].object, zone)) {
            ++written;
        }
    }

    // An empty transaction is not a step; see CommandStack::commit_transaction().
    stack.commit_transaction();
    return written;
}

} // namespace stratum::osm::road
