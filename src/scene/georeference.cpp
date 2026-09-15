// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "scene/georeference.hpp"

#include "osm/coordinates.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <utility>

namespace stratum::scene {

namespace {

/**
 * @brief How far a loaded Mercator origin may sit from a fresh projection of its
 *        own lat/lon before load() says so
 *
 * A millimetre. Double precision at a northing of 7.0e6 m resolves about 1e-9 m,
 * and the projection is a handful of ulps wide, so anything approaching a
 * millimetre is not rounding -- it is a different projection, a different
 * constant, or a hand-edited file. Warning at a tighter bound would fire on
 * noise; a looser one would let a metre of drift through, and a metre is a lane.
 */
constexpr double kMercatorDriftWarnMetres = 1.0e-3;

} // namespace

// ============================================================================
// GeoPoint
// ============================================================================

bool operator==(const GeoPoint& a, const GeoPoint& b) {
    return a.lat == b.lat && a.lon == b.lon;
}

bool operator!=(const GeoPoint& a, const GeoPoint& b) {
    return !(a == b);
}

GeoPoint centre_of(const osm::BoundingBox& bounds) {
    // BoundingBox::center() packs (lat, lon) into a dvec2 -- the one dvec2 in
    // Stratum that is not (x, y). Unpacking it here, once, is the whole reason
    // this function exists; see the note on GeoPoint.
    const glm::dvec2 packed = bounds.center();
    return GeoPoint{packed.x, packed.y};
}

// ============================================================================
// Validity
// ============================================================================

bool is_valid_origin(const GeoPoint& origin) {
    if (!std::isfinite(origin.lat) || !std::isfinite(origin.lon)) return false;
    // Refused rather than clamped. wgs84_to_mercator() clamps, so an origin at
    // 89 N would quietly become 85.051128 N and anchor the document a few
    // hundred kilometres from where the caller asked.
    if (std::fabs(origin.lat) > kMaxMercatorLatitudeDeg) return false;
    // Mercator easting is unbounded and happily projects 400 E, so an unwrapped
    // longitude produces coordinates that look fine and are not on the planet.
    return std::fabs(origin.lon) <= 180.0;
}

bool is_valid_scale(double scale) {
    return std::isfinite(scale) && scale > 0.0;
}

bool is_valid_frame(const GeoreferenceFrame& frame) {
    if (!is_valid_origin(frame.origin())) return false;
    if (!is_valid_scale(frame.scale)) return false;
    // The third field, and the reason this is not just the two checks above
    // spelled out at the call site. origin_mercator is stored rather than
    // derived on use, so no later step re-checks it: a non-finite one leaves
    // is_established() true and makes every conversion NaN.
    return std::isfinite(frame.origin_mercator.x) && std::isfinite(frame.origin_mercator.y);
}

// ============================================================================
// GeoreferenceFrame
// ============================================================================

GeoreferenceFrame GeoreferenceFrame::at(const GeoPoint& origin, double scale) {
    GeoreferenceFrame frame;
    frame.origin_lat = origin.lat;
    frame.origin_lon = origin.lon;
    frame.origin_mercator = osm::CoordinateConverter::wgs84_to_mercator(origin.lat, origin.lon);
    frame.scale = scale;
    return frame;
}

glm::dvec2 GeoreferenceFrame::to_local(const GeoPoint& p) const {
    const glm::dvec2 mercator = osm::CoordinateConverter::wgs84_to_mercator(p.lat, p.lon);
    return (mercator - origin_mercator) * scale;
}

GeoPoint GeoreferenceFrame::to_wgs84(const glm::dvec2& local) const {
    // Divide by scale before adding the origin, not after: the origin is in
    // Mercator metres and the local coordinate is in scaled ones, so they are
    // not in the same units until the division has happened.
    const glm::dvec2 mercator = (local / scale) + origin_mercator;

    // mercator_to_wgs84() returns (lat, lon) packed in a dvec2. Naming the two
    // components here is the point of returning a GeoPoint.
    const glm::dvec2 packed = osm::CoordinateConverter::mercator_to_wgs84(mercator.x, mercator.y);
    return GeoPoint{packed.x, packed.y};
}

glm::dvec3 GeoreferenceFrame::to_world(const GeoPoint& p, double world_y) const {
    const glm::dvec2 local = to_local(p);
    // Negated z: local northing grows north, world z grows south. See the header.
    return glm::dvec3{local.x, world_y, -local.y};
}

GeoPoint GeoreferenceFrame::world_to_wgs84(const glm::dvec3& world) const {
    // The same flip, undone. world.y carries height and has no bearing on where
    // on Earth the point is.
    return to_wgs84(glm::dvec2{world.x, -world.z});
}

double GeoreferenceFrame::ground_metres_per_unit() const {
    // Web Mercator's point scale factor is 1 / cos(latitude) in every direction,
    // so a local unit spans cos(latitude) true ground metres. Dividing by scale
    // undoes the frame's own multiplier on top of that.
    return std::cos(origin_lat * osm::DEG_TO_RAD) / scale;
}

bool GeoreferenceFrame::osm_converter_compatible() const {
    // Exact, not approximate. The question is "does the OSM chain produce the
    // same numbers", and it does so only when there is no scaling at all to
    // ignore; a scale of 1.0000001 is still a disagreement, just a small one.
    return scale == 1.0;
}

osm::CoordinateSystem GeoreferenceFrame::to_coordinate_system() const {
    osm::CoordinateSystem system;
    // origin_latlon is (lat, lon), matching CoordinateConverter::set_origin().
    system.origin_latlon = glm::dvec2{origin_lat, origin_lon};
    system.origin_mercator = origin_mercator;
    system.scale = scale;
    return system;
}

GeoreferenceFrame GeoreferenceFrame::from_coordinate_system(const osm::CoordinateSystem& system) {
    GeoreferenceFrame frame;
    frame.origin_lat = system.origin_latlon.x;
    frame.origin_lon = system.origin_latlon.y;
    // Verbatim, not re-derived: this is what the importer's own local
    // coordinates were measured against.
    frame.origin_mercator = system.origin_mercator;
    frame.scale = system.scale;
    return frame;
}

bool operator==(const GeoreferenceFrame& a, const GeoreferenceFrame& b) {
    return a.origin_lat == b.origin_lat && a.origin_lon == b.origin_lon &&
           a.origin_mercator == b.origin_mercator && a.scale == b.scale;
}

bool operator!=(const GeoreferenceFrame& a, const GeoreferenceFrame& b) {
    return !(a == b);
}

double mercator_origin_drift(const GeoreferenceFrame& frame) {
    const glm::dvec2 fresh =
        osm::CoordinateConverter::wgs84_to_mercator(frame.origin_lat, frame.origin_lon);
    return glm::length(fresh - frame.origin_mercator);
}

// ============================================================================
// Georeference
// ============================================================================

std::optional<GeoPoint> Georeference::origin() const {
    if (!m_frame) return std::nullopt;
    return m_frame->origin();
}

std::optional<glm::dvec2> Georeference::to_local(const GeoPoint& p) const {
    if (!m_frame) return std::nullopt;
    return m_frame->to_local(p);
}

std::optional<glm::dvec2> Georeference::to_local(double lat, double lon) const {
    return to_local(GeoPoint{lat, lon});
}

std::optional<GeoPoint> Georeference::to_wgs84(const glm::dvec2& local) const {
    if (!m_frame) return std::nullopt;
    return m_frame->to_wgs84(local);
}

std::optional<glm::dvec3> Georeference::to_world(const GeoPoint& p, double world_y) const {
    if (!m_frame) return std::nullopt;
    return m_frame->to_world(p, world_y);
}

std::optional<GeoPoint> Georeference::world_to_wgs84(const glm::dvec3& world) const {
    if (!m_frame) return std::nullopt;
    return m_frame->world_to_wgs84(world);
}

std::optional<double> Georeference::ground_metres_per_unit() const {
    if (!m_frame) return std::nullopt;
    return m_frame->ground_metres_per_unit();
}

std::optional<osm::CoordinateSystem> Georeference::coordinate_system() const {
    if (!m_frame) return std::nullopt;
    return m_frame->to_coordinate_system();
}

bool Georeference::load(std::optional<GeoreferenceFrame> frame) {
    if (frame && !is_valid_frame(*frame)) {
        // Refused, not logged-and-installed. Installing it left is_established()
        // true over a frame that cannot convert: a scale of 0 collapses every
        // point in the document onto the origin and makes to_wgs84() NaN, and a
        // NaN origin leaves ground_metres_per_unit() NaN while to_local() goes
        // on answering off origin_mercator. Both are the plausible-looking wrong
        // answer this file exists to refuse, and load() is the one path that
        // does not run the commands' own is_valid_origin()/is_valid_scale().
        //
        // The alternative considered was to keep the origin and substitute a
        // scale of 1.0. That is worse: the coordinates then look right and are
        // wrong by whatever factor the file asked for, which is the silent
        // failure again with a repair in front of it.
        //
        // The frame is cleared rather than left as it was, because load() names
        // the whole state of a document being opened -- keeping the previous
        // document's frame would interpret the new one's coordinates against it.
        spdlog::error("Georeference: refusing a loaded frame at {}, {} with Mercator origin "
                      "({}, {}) and scale {}; the document is left NOT georeferenced rather "
                      "than georeferenced onto a point",
                      frame->origin_lat, frame->origin_lon, frame->origin_mercator.x,
                      frame->origin_mercator.y, frame->scale);
        set_frame(std::nullopt);
        return false;
    }

    if (frame) {
        // Drift is not invalidity, so it is warned about and kept. The stored
        // Mercator origin is adopted as it stands: repairing it would move every
        // loaded vertex relative to the frame. This only makes a projection that
        // has changed under the document visible instead of silent.
        const double drift = mercator_origin_drift(*frame);
        if (drift > kMercatorDriftWarnMetres) {
            spdlog::warn("Georeference: the loaded Mercator origin is {} m from a fresh "
                         "projection of {}, {}; keeping the loaded value, because the "
                         "document's coordinates were built against it",
                         drift, frame->origin_lat, frame->origin_lon);
        }
    }

    set_frame(std::move(frame));
    return true;
}

void Georeference::set_frame(std::optional<GeoreferenceFrame> frame) {
    m_frame = std::move(frame);
    // Bumped even when the new frame equals the old one. The counter answers
    // "were my cached coordinates derived under the frame that is current now",
    // and a command that ran is a break in that chain whatever the values were.
    ++m_generation;
}

// ============================================================================
// GeoreferenceCommand
// ============================================================================

void GeoreferenceCommand::set_frame(std::optional<GeoreferenceFrame> frame) {
    m_georeference->set_frame(std::move(frame));
}

// ============================================================================
// EstablishGeoreferenceCommand
// ============================================================================

EstablishGeoreferenceCommand::EstablishGeoreferenceCommand(Georeference& georeference,
                                                           const GeoPoint& origin, double scale)
    : GeoreferenceCommand(georeference) {
    m_valid = is_valid_origin(origin) && is_valid_scale(scale);
    if (m_valid) {
        m_frame = GeoreferenceFrame::at(origin, scale);
    }
    // An invalid request leaves m_frame default-constructed, which is a frame at
    // 0 N 0 E. It is never installed: apply() checks m_valid first. Validating
    // here rather than in apply() is what lets a dialog grey out its OK button
    // from valid() without executing anything.
}

bool EstablishGeoreferenceCommand::apply() {
    if (!m_valid) {
        spdlog::warn("EstablishGeoreferenceCommand: refusing an origin outside the Web "
                     "Mercator band or a non-positive scale");
        return false;
    }
    if (georeference().is_established()) {
        // Not an error, and not something to force through. Re-anchoring has to
        // move the geometry that was built against the old origin, and this
        // command has no delta for the caller to apply.
        spdlog::warn("EstablishGeoreferenceCommand: the document is already georeferenced; "
                     "use RebaseGeoreferenceCommand to move the origin");
        return false;
    }
    set_frame(m_frame);
    return true;
}

void EstablishGeoreferenceCommand::revert() {
    const std::optional<GeoreferenceFrame>& current = georeference().frame();
    if (!current || *current != m_frame) {
        // Reported and then done anyway, because revert() must not refuse
        // (command.hpp). Reaching here means load() replaced the frame while this
        // step was still on the undo stack, so undoing it is about to discard a
        // frame this command never installed -- the loaded one. The caller's fix
        // is CommandStack::clear() beside the load; see Georeference::load().
        spdlog::warn("EstablishGeoreferenceCommand: reverting over a frame this command did "
                     "not install; the history was not cleared after a load");
    }
    // Back to "not georeferenced", which is a different state from "at 0, 0" and
    // is the state the document was in: apply() refuses unless it was.
    set_frame(std::nullopt);
}

std::string EstablishGeoreferenceCommand::describe() const {
    return "Establish georeference";
}

// ============================================================================
// RebaseGeoreferenceCommand
// ============================================================================

RebaseGeoreferenceCommand::RebaseGeoreferenceCommand(Georeference& georeference,
                                                     const GeoPoint& new_origin)
    : GeoreferenceCommand(georeference) {
    const std::optional<GeoreferenceFrame>& current = georeference.frame();
    if (!current) return;                     // nothing to rebase; establish first
    if (!is_valid_origin(new_origin)) return; // refused, not clamped
    if (current->origin() == new_origin) {
        // A no-op recorded as an undo step is a menu entry that does nothing when
        // a user picks it. Compared on the origin the caller asked for, not on
        // the whole frame, because the origin is what a rebase is about.
        return;
    }

    m_old_frame = *current;
    // Scale is carried across, never taken as a parameter. See the file comment.
    m_new_frame = GeoreferenceFrame::at(new_origin, m_old_frame.scale);

    // local = (mercator - origin) * scale, so
    //   local_new - local_old = (old_origin - new_origin) * scale
    // for every point, independent of the point. That independence is the whole
    // reason a rebase can be handed to a caller as one vector.
    m_delta = (m_old_frame.origin_mercator - m_new_frame.origin_mercator) * m_old_frame.scale;
    m_valid = true;
}

bool RebaseGeoreferenceCommand::apply() {
    if (!m_valid) return false;

    const std::optional<GeoreferenceFrame>& current = georeference().frame();
    if (!current || *current != m_old_frame) {
        // local_delta() was measured against m_old_frame and has very likely
        // already been handed to a caller. Applying it now, against a frame that
        // has moved since, would shift the scene by the difference between the
        // two -- silently, and by a distance that looks like a plausible city.
        // Refusing leaves both the frame and the history untouched.
        spdlog::warn("RebaseGeoreferenceCommand: the frame changed after this command "
                     "measured it; refusing rather than applying a stale delta");
        return false;
    }

    set_frame(m_new_frame);
    return true;
}

void RebaseGeoreferenceCommand::revert() {
    const std::optional<GeoreferenceFrame>& current = georeference().frame();
    if (!current || *current != m_new_frame) {
        // Same hole as the establish revert, and the same reason it can only be
        // reported: apply() refuses a stale frame, but revert() has no such
        // option. Undoing here reinstates m_old_frame over whatever a load put
        // in its place, which is the frame the file was meant to replace.
        spdlog::warn("RebaseGeoreferenceCommand: reverting over a frame this command did not "
                     "install; the history was not cleared after a load");
    }
    // Restores the frame only. The caller's geometry shift is its own command and
    // is reverted by its own revert(); putting the two in one transaction is what
    // keeps them in step.
    set_frame(m_old_frame);
}

std::string RebaseGeoreferenceCommand::describe() const {
    return "Move georeference origin";
}

// ============================================================================
// ClearGeoreferenceCommand
// ============================================================================

ClearGeoreferenceCommand::ClearGeoreferenceCommand(Georeference& georeference)
    : GeoreferenceCommand(georeference) {}

bool ClearGeoreferenceCommand::apply() {
    const std::optional<GeoreferenceFrame>& current = georeference().frame();
    if (!current) return false;

    // Captured on every apply rather than only the first. A redo runs against the
    // frame its own undo put back, so the value is the same either way, and this
    // way m_previous can never be engaged while the command is not applied.
    m_previous = *current;
    set_frame(std::nullopt);
    return true;
}

void ClearGeoreferenceCommand::revert() {
    if (!m_previous) {
        // revert() must not fail (command.hpp). Reaching here means revert ran
        // without a matching apply, which is a bug in the stack, not in the frame.
        spdlog::error("ClearGeoreferenceCommand: revert with nothing captured; the frame "
                      "is left as it is");
        return;
    }
    set_frame(m_previous);
}

std::string ClearGeoreferenceCommand::describe() const {
    return "Clear georeference";
}

// ============================================================================
// Convenience
// ============================================================================

bool establish_georeference(CommandStack& stack, Georeference& georeference,
                            const GeoPoint& origin, double scale) {
    return stack.execute(std::make_unique<EstablishGeoreferenceCommand>(georeference, origin, scale));
}

bool rebase_georeference(CommandStack& stack, Georeference& georeference,
                         const GeoPoint& new_origin, glm::dvec2& delta_out) {
    auto command = std::make_unique<RebaseGeoreferenceCommand>(georeference, new_origin);

    // Read before execute(), while the command is still ours. Copied out only
    // after the stack accepts it, so a refused rebase cannot leave the caller
    // holding a delta it is about to apply to the geometry.
    const glm::dvec2 delta = command->local_delta();

    if (!stack.execute(std::move(command))) return false;

    delta_out = delta;
    return true;
}

bool clear_georeference(CommandStack& stack, Georeference& georeference) {
    return stack.execute(std::make_unique<ClearGeoreferenceCommand>(georeference));
}

} // namespace stratum::scene
