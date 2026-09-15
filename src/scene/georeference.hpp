// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file georeference.hpp
 * @brief Where the document sits on Earth, established once and true for the
 *        whole session
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### The failure this closes
 *
 * `osm::CoordinateConverter::wgs84_to_local()` returns raw Web Mercator when
 * `set_origin()` was never called. Nothing is logged and nothing fails: the
 * caller gets coordinates in the millions of metres instead of the thousands,
 * the geometry is built kilometres from the camera, and the first symptom is an
 * empty viewport that looks like a rendering bug. Every diagnosis of that starts
 * in the wrong subsystem.
 *
 * The fault is not the fallback, it is that the frame lives on a converter that
 * anybody can default-construct. A projection origin is not a property of a
 * converter; it is a property of the DOCUMENT. Pick it once, at import, and
 * every local coordinate in the session means something. Fail to pick it and
 * nothing does -- which is a state worth being able to name.
 *
 * So this type has no fallback. Every conversion returns `std::optional`, and an
 * ungeoreferenced document returns `std::nullopt` rather than a plausible
 * number. A caller that ignores it gets a compiler warning from `[[nodiscard]]`
 * and a crash from `operator*`, both of which are better than being 5,900 km
 * from where you meant to be.
 *
 * ### Two states, not one
 *
 * "Not yet georeferenced" and "georeferenced at 0 N, 0 E" are different
 * documents. The second is a real place (the Gulf of Guinea) and a legitimate
 * frame; the first means no place has been chosen. They are trivially confused
 * because a default-constructed origin is `(0, 0)` in both, which is why the
 * establishment flag is not a field beside the origin but the `std::optional`
 * WRAPPING it: there is no way to read the origin without having answered the
 * question first.
 *
 * `Georeference{}.to_local(0.0, 0.0)` is `nullopt`. `Georeference` established
 * at `{0, 0}` converts `{0, 0}` to `(0, 0)` local. Both are correct and they do
 * not look alike.
 *
 * ### Local units are Mercator metres, not ground metres
 *
 * Web Mercator is conformal, not equal-area: it stretches by `1 / cos(latitude)`
 * in every direction. At Dublin that is 1.676, so a 10 m local unit spans 5.97 m
 * of real ground. The existing chain has always done this -- `MeshBuilder`,
 * `QuadTree` and the road solver all work in these units and agree with each
 * other -- so this type does NOT silently correct it, because correcting it
 * would move every building relative to geometry built before the change.
 *
 * What it does instead is name it. `ground_metres_per_unit()` is the factor, and
 * anything that reports a distance or an area to a user -- a measure tool, an
 * export summary -- multiplies by it. A road length printed straight out of
 * local coordinates is 68% too long in Dublin and 100% too long in Reykjavik,
 * and nobody notices, because it is plausible.
 *
 * ### Changing the origin after geometry exists
 *
 * Allowed, and defined exactly, rather than refused.
 *
 * Refusing sounds safe until you have two extracts to merge, or a user who
 * framed the import on the wrong suburb; then the only recovery is to throw the
 * document away. The reason a rebase can be defined at all is that the frame is
 * a pure translation: `local = (mercator - origin_mercator) * scale`, so moving
 * the origin from O1 to O2 moves every local coordinate by
 * `(O1 - O2) * scale`, exactly, with no rotation and no distortion.
 *
 * So `RebaseGeoreferenceCommand` exposes that translation as
 * `local_delta()`, readable BEFORE execute() -- the same trick
 * `CreateLayerCommand` uses for its id -- and the contract is:
 *
 *   > Everything already built against the old origin must have `local_delta()`
 *   > added to it, in the same transaction, or it stays where it was and now
 *   > names the wrong place on Earth.
 *
 * Georeference owns no geometry and cannot do that shift itself. Two things stop
 * it being a silent trap anyway. The delta is a mandatory out-parameter of
 * `rebase_georeference()`, so a caller has to name a variable for it. And
 * `generation()` is bumped on every change of frame, undo and load included, so
 * a cache holding local coordinates -- the quadtree, an uploaded mesh, a
 * baked AO map -- can compare one integer and know it is stale.
 *
 * Scale is deliberately NOT rebasable: `RebaseGeoreferenceCommand` has no scale
 * parameter and carries the old one across. A scale change is not a translation,
 * it multiplies every coordinate, and there is no delta a caller can apply to a
 * GPU buffer or a quadtree's node extents to compensate. Re-import instead.
 *
 * ### Not antimeridian-aware
 *
 * Web Mercator easting is not periodic, and nothing here wraps it. An origin at
 * 180 E and one at 180 W are both accepted, they name the same meridian, and a
 * frame built on either is perfectly self-consistent -- but a rebase from
 * 179.9 E to 179.9 W, two origins 22 km apart on the ground, has a
 * `local_delta()` of 4.0e7 m and leaves every local coordinate out near 4.0e7 as
 * well. Consecutive floats are 4 m apart at that magnitude and the renderer
 * uploads floats, so the geometry quantises to lane widths while every number
 * involved stays correct.
 *
 * The decision here is to name that rather than wrap it: wrapping would make
 * `local_delta()` a lie for the points on the far side of the seam, and a frame
 * whose translation is not uniform is not a frame any caller can shift geometry
 * against. An extract that straddles the antimeridian is imported against an
 * origin on its own side, not rebased across. Pinned by
 * `a_rebase_across_the_antimeridian_is_not_wrapped`.
 *
 * ### What A2 needs
 *
 * `GeoreferenceFrame` is the whole serialisable state: two doubles of origin,
 * two of Mercator origin, one of scale. A2 writes all five.
 *
 * Writing `origin_mercator` looks redundant -- it is derived -- and it is
 * written anyway, because on load it is what every local coordinate in the file
 * was computed against. `Georeference::load()` therefore takes the stored value
 * VERBATIM and does not recompute it. Recomputing would re-derive the origin
 * from a libm that may round differently from the one that saved the file, and
 * every vertex in the document would shift relative to the frame by that
 * difference: small, silent, and impossible to attribute later. The stored value
 * is checked against a fresh projection and `mercator_origin_drift()` reports
 * the disagreement, so a projection change surfaces as a warning instead of as
 * drift.
 *
 * ### Every mutation is a Command
 *
 * The frame is set through `GeoreferenceCommand`, which is `Georeference`'s only
 * friend, exactly as `LayerCommand` is `LayerTree`'s. `load()` is the single
 * audited exception, matching `AttributeStore::load_value()`: opening a file is
 * not an edit and must not fill the undo stack.
 */

#pragma once

#include "scene/command.hpp"
#include "osm/types.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace stratum::scene {

// ============================================================================
// Geographic point
// ============================================================================

/**
 * @brief A WGS84 position, with the two numbers named
 *
 * Deliberately NOT a `glm::dvec2`. Every other dvec2 in Stratum is `(x, y)`, and
 * `osm::BoundingBox::center()` is the exception that packs `(lat, lon)` into
 * one -- so `bounds.center().x` is a latitude while `local.x` is an easting, and
 * the two read identically at the call site. Getting them the wrong way round
 * near Dublin puts you at 6.26 S, 53.35 E, which is 4,000 km into the Indian
 * Ocean and still a perfectly valid coordinate.
 *
 * A named field cannot be swapped by accident, so the whole public surface of
 * this file speaks GeoPoint and never a dvec2 of angles. Use centre_of() to get
 * one out of a BoundingBox rather than unpacking it at the call site.
 */
struct GeoPoint {
    double lat = 0.0;  ///< Degrees north, WGS84
    double lon = 0.0;  ///< Degrees east, WGS84
};

/// Exact componentwise equality. Answers "is this the same origin", not "are
/// these close" -- a tolerance here would let a rebase of half a metre look
/// like a no-op and be refused.
[[nodiscard]] bool operator==(const GeoPoint& a, const GeoPoint& b);
[[nodiscard]] bool operator!=(const GeoPoint& a, const GeoPoint& b);

/**
 * @brief Centre of an OSM bounding box, with the packing undone
 *
 * `BoundingBox::center()` returns `(lat, lon)` in a dvec2. This is the one
 * audited place that unpacks it, so the order is decided once rather than at
 * every import site.
 *
 * @note Returns `{0, 0}` for an invalid (never-expanded) box. That is a real
 *       place, so check `bounds.is_valid()` first; establishing a frame at the
 *       Gulf of Guinea because the extract was empty is exactly the kind of
 *       plausible-looking wrong answer this file exists to prevent.
 */
[[nodiscard]] GeoPoint centre_of(const osm::BoundingBox& bounds);

// ============================================================================
// Validity
// ============================================================================

/**
 * @brief Latitude beyond which Web Mercator is unusable
 *
 * `osm::CoordinateConverter::wgs84_to_mercator()` CLAMPS to this rather than
 * failing, so projecting 89 N silently gives you 85.051128 N. That is fine for a
 * stray node in a data set; it is not fine for an origin, because the whole
 * document would then be anchored somewhere the caller did not ask for. So an
 * origin past this is refused instead of clamped.
 */
inline constexpr double kMaxMercatorLatitudeDeg = 85.051128;

/// True when @p origin can anchor a frame: finite, and inside the Mercator band.
[[nodiscard]] bool is_valid_origin(const GeoPoint& origin);

/// True when @p scale can be used: finite, strictly positive. A zero scale
/// collapses the document to a point and is not a degenerate case worth
/// supporting; a negative one mirrors it, which no caller means.
[[nodiscard]] bool is_valid_scale(double scale);

// ============================================================================
// The frame
// ============================================================================

/**
 * @brief The projection frame: everything needed to place the scene on Earth
 *
 * Plain data, copyable, and the unit A2 serialises. Holding one means the
 * document IS georeferenced -- the "not yet" state is the absence of a frame,
 * not a flag inside it -- so the methods here are unconditional and none of them
 * returns an optional.
 *
 * Construct through at(), which derives `origin_mercator`, or through
 * from_coordinate_system() when restoring. Aggregate-initialising all four
 * fields by hand is possible and is how A2's loader rebuilds a saved frame; it
 * is also how you get a frame whose Mercator origin does not match its lat/lon,
 * which mercator_origin_drift() exists to catch.
 */
struct GeoreferenceFrame {
    double origin_lat = 0.0;                  ///< Degrees north, WGS84
    double origin_lon = 0.0;                  ///< Degrees east, WGS84

    /**
     * @brief Web Mercator (EPSG:3857) projection of the origin, in metres
     *
     * Derived from the origin, and stored rather than recomputed on use. See
     * the note on load() in the file comment: this is the number every local
     * coordinate in the document was measured against, and re-deriving it on a
     * different machine is how a document drifts.
     */
    glm::dvec2 origin_mercator{0.0, 0.0};

    /**
     * @brief Uniform multiplier on the horizontal local frame
     *
     * 1.0 means one local unit is one Mercator metre, which is what the importer
     * and every existing consumer assume.
     *
     * @warning `osm::CoordinateConverter` declares this field on
     *          `osm::CoordinateSystem` and then IGNORES it -- see
     *          coordinates.cpp:64. A frame whose scale is not 1.0 therefore
     *          disagrees with anything that converts through the OSM chain.
     *          osm_converter_compatible() is that check, spelled out.
     */
    double scale = 1.0;

    /**
     * @brief Build a frame anchored at @p origin
     *
     * @param origin Anchor. Must satisfy is_valid_origin(); an invalid one
     *               produces a frame with a clamped Mercator origin, which is
     *               why the commands validate before they call this.
     * @param scale  See the field. Must satisfy is_valid_scale().
     */
    [[nodiscard]] static GeoreferenceFrame at(const GeoPoint& origin, double scale = 1.0);

    /// @return The origin as a GeoPoint, which is the safe spelling of it.
    [[nodiscard]] GeoPoint origin() const { return GeoPoint{origin_lat, origin_lon}; }

    /// WGS84 to local metres, `(x = easting, y = northing)`.
    [[nodiscard]] glm::dvec2 to_local(const GeoPoint& p) const;

    /// Local metres back to WGS84. The exact inverse of to_local() to within the
    /// round-trip accuracy documented on Georeference::to_wgs84().
    [[nodiscard]] GeoPoint to_wgs84(const glm::dvec2& local) const;

    /**
     * @brief WGS84 to Y-up world space
     *
     * The horizontal pair becomes `(x, z)` with **z negated**, because local
     * northing is +y in a Z-up 2D frame and -z in the Y-up world the renderer
     * uses. Dropping that flip mirrors the whole city north-south, which reads
     * as "the import is wrong" rather than as an axis bug.
     *
     * @param world_y Height, in world units, passed through untouched. It never
     *                went through the projection, so `scale` does not apply to
     *                it -- the caller supplies a world-space height.
     *
     * @note This convention is duplicated in `osm/mesh_builder.cpp:170` and
     *       `osm/building_collision.cpp:124`, both file-local. Three copies is
     *       two too many; hoisting them is its own change.
     */
    [[nodiscard]] glm::dvec3 to_world(const GeoPoint& p, double world_y) const;

    /// Y-up world space back to WGS84. The `y` component is ignored.
    [[nodiscard]] GeoPoint world_to_wgs84(const glm::dvec3& world) const;

    /**
     * @brief True ground metres spanned by one local unit
     *
     * `cos(origin_lat) / scale`. Multiply a distance measured in local
     * coordinates by this to report it to a user. About 0.597 at Dublin, 0.658
     * at Paris, 1.0 at the equator.
     *
     * @note One number for the whole document, taken at the origin. Mercator's
     *       stretch varies with latitude, so this is exact only along the
     *       origin's own parallel. Over a city extract the error is well under a
     *       part in a thousand; over a whole country it is not, and a measure
     *       tool that needs better should compute a geodesic instead.
     */
    [[nodiscard]] double ground_metres_per_unit() const;

    /**
     * @brief True when this frame converts exactly as `osm::CoordinateConverter`
     *        does
     *
     * False only for a non-unit scale, which the OSM converter ignores. Worth
     * asserting anywhere scene coordinates and importer coordinates meet.
     */
    [[nodiscard]] bool osm_converter_compatible() const;

    /// The frame as the OSM chain spells it, for seeding an importer.
    [[nodiscard]] osm::CoordinateSystem to_coordinate_system() const;

    /// Adopt a frame the OSM chain produced. Takes `origin_mercator` verbatim
    /// for the reason load() does: it is what that data was built against.
    [[nodiscard]] static GeoreferenceFrame from_coordinate_system(
        const osm::CoordinateSystem& system);
};

/// Exact componentwise equality. Used to detect a frame that moved under a
/// command that had already measured it, so a tolerance would defeat the point.
[[nodiscard]] bool operator==(const GeoreferenceFrame& a, const GeoreferenceFrame& b);
[[nodiscard]] bool operator!=(const GeoreferenceFrame& a, const GeoreferenceFrame& b);

/**
 * @brief Distance, in Mercator metres, between a frame's stored Mercator origin
 *        and a fresh projection of its lat/lon
 *
 * Zero for a frame built by at(). Non-zero after a load means the stored value
 * and the projection no longer agree -- a different libm, a changed constant, or
 * a hand-edited file. Load() logs when this exceeds a millimetre; a caller that
 * wants to surface it in the UI reads it directly.
 */
[[nodiscard]] double mercator_origin_drift(const GeoreferenceFrame& frame);

/**
 * @brief True when @p frame can convert at all
 *
 * All four fields, not the two the commands validate on the way in: a valid
 * origin, a valid scale, and a finite Mercator origin. Every frame from at() or
 * from a command satisfies this. A frame read off disk or aggregate-initialised
 * by hand need not, which is the whole reason Georeference::load() asks.
 *
 * `origin_mercator` is checked here and nowhere else, because it is stored
 * rather than derived on use, so nothing downstream ever recomputes it: a
 * non-finite one makes every conversion NaN while is_established() stays true.
 *
 * Deliberately NOT a drift check. A stored Mercator origin that disagrees with a
 * fresh projection of its own lat/lon is still usable -- it is precisely the
 * frame the document's coordinates were built against, and
 * mercator_origin_drift() reports the disagreement. Unusable means the
 * arithmetic cannot name a place on Earth at all.
 */
[[nodiscard]] bool is_valid_frame(const GeoreferenceFrame& frame);

// ============================================================================
// The document's georeference
// ============================================================================

class GeoreferenceCommand;

/**
 * @brief The document's frame: one per open document, for the whole session
 *
 * Queries are public; the frame is set only through a Command. Not thread-safe,
 * for the same reason CommandStack is not: edits come from the UI thread.
 *
 * Every conversion returns `std::optional` and yields `nullopt` when no frame
 * has been established. That is the whole point of the type -- see the file
 * comment -- and it is why there is no `frame()` that returns a reference to a
 * default-constructed frame.
 */
class Georeference {
public:
    Georeference() = default;

    // -- State ---------------------------------------------------------------

    /// True once an origin has been chosen. False on a fresh document, and false
    /// again after ClearGeoreferenceCommand.
    [[nodiscard]] bool is_established() const { return m_frame.has_value(); }

    /// The frame, or nullopt when none has been established. A2 saves this.
    [[nodiscard]] const std::optional<GeoreferenceFrame>& frame() const { return m_frame; }

    /// The anchor, or nullopt when none has been established.
    [[nodiscard]] std::optional<GeoPoint> origin() const;

    /**
     * @brief Counter bumped on every change of frame
     *
     * Establish, rebase, clear, undo, redo and load() all bump it; nothing else
     * does. It never decreases, so undoing a rebase does not restore the old
     * value -- the question it answers is "are my cached local coordinates still
     * measured against the current frame", and after an undo they are not, even
     * though the frame has a value it held before.
     *
     * Anything holding local coordinates it did not derive this frame -- the
     * quadtree, uploaded meshes, baked AO -- stores the generation it was built
     * at and rebuilds when this differs.
     */
    [[nodiscard]] uint64_t generation() const { return m_generation; }

    // -- Conversion ----------------------------------------------------------

    /// WGS84 to local metres. nullopt when the document is not georeferenced.
    [[nodiscard]] std::optional<glm::dvec2> to_local(const GeoPoint& p) const;

    /// @copydoc to_local(const GeoPoint&) const
    /// @note LATITUDE FIRST, matching `osm::CoordinateConverter::wgs84_to_local()`
    ///       so a migrating call site keeps its argument order. Prefer the
    ///       GeoPoint overload in new code.
    [[nodiscard]] std::optional<glm::dvec2> to_local(double lat, double lon) const;

    /**
     * @brief Local metres back to WGS84
     *
     * Round trip through to_local() and back is accurate to better than
     * 1e-8 metres over a 10 km extract at Dublin's latitude -- see
     * tests/scene/test_georeference.cpp, which measures it rather than assuming
     * it. The limit is double precision at a Mercator northing of 7.0e6 m, where
     * one ulp is about 1e-9 m, not the projection maths.
     *
     * @return nullopt when the document is not georeferenced.
     */
    [[nodiscard]] std::optional<GeoPoint> to_wgs84(const glm::dvec2& local) const;

    /// WGS84 to Y-up world space. See GeoreferenceFrame::to_world() for the flip.
    [[nodiscard]] std::optional<glm::dvec3> to_world(const GeoPoint& p, double world_y) const;

    /// Y-up world space back to WGS84, ignoring height.
    [[nodiscard]] std::optional<GeoPoint> world_to_wgs84(const glm::dvec3& world) const;

    /// True ground metres per local unit, or nullopt when not georeferenced.
    /// Optional rather than 1.0, because a silent 1.0 is a distance reported 68%
    /// long at Dublin and nobody checks a plausible number.
    [[nodiscard]] std::optional<double> ground_metres_per_unit() const;

    /// The frame as the OSM chain spells it, for seeding an importer's converter.
    [[nodiscard]] std::optional<osm::CoordinateSystem> coordinate_system() const;

    // -- Load ----------------------------------------------------------------

    /**
     * @brief Install a frame with NO undo history
     *
     * @warning A deliberate hole in the history, and the only one, matching
     *          `AttributeStore::load_value()`. It exists for A2's loader and for
     *          the importer establishing the frame before the document is ever
     *          shown. Recording those as commands would put "open a file" on the
     *          undo stack, and undoing it would leave a document full of
     *          geometry with no frame to interpret it.
     *
     *          Anything that runs while a user is looking at the document builds
     *          a command. A mutation that skips the stack is a hole whose
     *          symptom is an undo three steps later restoring state that was
     *          never current -- see command.hpp.
     *
     * @warning The caller must `CommandStack::clear()` in the same breath. Every
     *          step already on the stack was measured against the frame being
     *          replaced: undoing an establish after a load discards the LOADED
     *          frame and leaves the document with none, and undoing a rebase
     *          after a load reinstates the frame the file replaced. A command
     *          cannot defend itself here -- RebaseGeoreferenceCommand::apply()
     *          refuses a stale redo, but revert() must not refuse (command.hpp)
     *          and can only log. Clearing the history is the fix, and there is
     *          no other.
     *
     * @param frame The frame verbatim, INCLUDING its stored `origin_mercator`,
     *              which is not recomputed. Pass nullopt to load an
     *              ungeoreferenced document.
     *
     * @return false when @p frame was refused, which is exactly when
     *         is_valid_frame() is false. A refused frame is neither installed nor
     *         repaired: the document is left reading "not georeferenced", a state
     *         every caller already handles, instead of "georeferenced onto a
     *         point", which none of them do. The commands refuse the same frames
     *         on the way in and this is the one path around the commands, so it
     *         asks the same question. The frame is cleared rather than left as it
     *         was, because load() names the whole state of a document being
     *         opened -- keeping the previous document's frame would interpret the
     *         new one's coordinates against it.
     *
     * Logs a warning when the stored Mercator origin and a fresh projection of
     * the stored lat/lon disagree by more than a millimetre. That is drift, not
     * invalidity: the frame is still the one the document's coordinates were
     * built against, so it is kept. Bumps generation(), a refusal included -- a
     * refused load still replaces the frame with nothing, and a cache of local
     * coordinates is stale either way.
     */
    [[nodiscard]] bool load(std::optional<GeoreferenceFrame> frame);

private:
    /**
     * @brief The one door into the mutating API
     *
     * GeoreferenceCommand forwards this to its subclasses as a protected helper,
     * so "every mutation goes through the command stack" is enforced by the
     * compiler in one place instead of being a rule in a comment. Same shape as
     * LayerTree's friendship with LayerCommand.
     */
    friend class GeoreferenceCommand;

    void set_frame(std::optional<GeoreferenceFrame> frame);

    std::optional<GeoreferenceFrame> m_frame;

    /// Never decreases, not even on undo. See generation().
    uint64_t m_generation = 0;
};

// ============================================================================
// Commands
// ============================================================================

/**
 * @brief Base for every change of frame
 *
 * Owns the forwarding into Georeference's private API. A new georeference
 * operation subclasses this; it does not touch Georeference directly, and it
 * cannot -- GeoreferenceCommand is the only friend.
 */
class GeoreferenceCommand : public Command {
protected:
    explicit GeoreferenceCommand(Georeference& georeference)
        : m_georeference(&georeference) {}

    [[nodiscard]] Georeference& georeference() const { return *m_georeference; }

    /// Forwarder into Georeference::set_frame(). See the matching private member.
    void set_frame(std::optional<GeoreferenceFrame> frame);

    Georeference* m_georeference;
};

/**
 * @brief Choose the document's origin, once
 *
 * Refused when the document is ALREADY georeferenced. Re-anchoring is a
 * different operation with different consequences for existing geometry, and
 * spelling both as "set the origin" is how a rebase gets performed by accident
 * with nothing shifted to match. Use RebaseGeoreferenceCommand for that.
 *
 * Also refused for an origin outside the Mercator band or a non-positive scale,
 * rather than clamping the way the OSM converter does.
 */
class EstablishGeoreferenceCommand : public GeoreferenceCommand {
public:
    /**
     * @param origin Anchor, typically centre_of(the extract's bounds)
     * @param scale  See GeoreferenceFrame::scale. Leave at 1.0 to stay
     *               compatible with the OSM chain.
     */
    EstablishGeoreferenceCommand(Georeference& georeference, const GeoPoint& origin,
                                 double scale = 1.0);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

    /**
     * @brief Bytes this command holds, for CommandStackConfig::max_bytes
     *
     * Overridden because Command::footprint() defaults to sizeof(Command) -- the
     * vtable pointer, 8 bytes -- which under-reports this by 8x and leaves the
     * stack's memory bound counting something unrelated to what it holds. No
     * heap state, so the object's own size is the whole cost.
     */
    [[nodiscard]] size_t footprint() const override {
        return sizeof(EstablishGeoreferenceCommand);
    }

    /**
     * @brief The frame this command installs
     *
     * Readable before execute(), so a caller can derive local coordinates for
     * the geometry it is about to create inside the same transaction. Meaningful
     * only when valid() is true.
     */
    [[nodiscard]] const GeoreferenceFrame& frame() const { return m_frame; }

    /// False when the origin or scale was rejected in the constructor, in which
    /// case apply() refuses. Public so a dialog can disable its OK button.
    [[nodiscard]] bool valid() const { return m_valid; }

private:
    GeoreferenceFrame m_frame;
    bool m_valid = false;
};

/**
 * @brief Move the origin of an established frame
 *
 * The hard case. Read the file comment before using this: the command changes
 * the frame and NOTHING ELSE, and every local coordinate already built against
 * the old origin has to have local_delta() added to it, in the same transaction,
 * or it silently comes to name a different place on Earth.
 *
 * Refused when the document is not georeferenced (establish first), when the new
 * origin equals the current one (a no-op recorded as an undo step is a menu
 * entry that does nothing), when the new origin is outside the Mercator band,
 * and when the frame has changed since this command was constructed -- because
 * local_delta() was measured against the frame that was current THEN, and
 * applying it against a different one would shift the scene by the difference.
 *
 * Scale is carried across unchanged. There is deliberately no parameter for it;
 * see the file comment.
 */
class RebaseGeoreferenceCommand : public GeoreferenceCommand {
public:
    RebaseGeoreferenceCommand(Georeference& georeference, const GeoPoint& new_origin);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

    /**
     * @brief Bytes this command holds, for CommandStackConfig::max_bytes
     *
     * Overridden because Command::footprint() defaults to sizeof(Command) -- the
     * vtable pointer, 8 bytes -- which under-reports this by 15x and leaves the
     * stack's memory bound counting something unrelated to what it holds. No
     * heap state, so the object's own size is the whole cost.
     */
    [[nodiscard]] size_t footprint() const override {
        return sizeof(RebaseGeoreferenceCommand);
    }

    /**
     * @brief What every existing local coordinate must have ADDED to it
     *
     * `(old_origin_mercator - new_origin_mercator) * scale`. Readable before
     * execute(), so the caller can build the geometry shift into the same
     * transaction as the rebase. `(0, 0)` when valid() is false.
     *
     * Undoing the rebase undoes the frame change only. The caller's geometry
     * shift is its own command and is undone by its own revert() -- which is
     * exactly why the two belong in one transaction.
     */
    [[nodiscard]] glm::dvec2 local_delta() const { return m_delta; }

    /// The frame that will be installed. Meaningful only when valid() is true.
    [[nodiscard]] const GeoreferenceFrame& frame() const { return m_new_frame; }

    /// False when the constructor rejected the request: not georeferenced, a
    /// no-op, or an out-of-band origin. apply() then refuses.
    [[nodiscard]] bool valid() const { return m_valid; }

private:
    GeoreferenceFrame m_new_frame;

    /// The frame local_delta() was measured against. apply() refuses if the
    /// document no longer holds exactly this.
    GeoreferenceFrame m_old_frame;

    glm::dvec2 m_delta{0.0, 0.0};
    bool m_valid = false;
};

/**
 * @brief Return the document to "not georeferenced"
 *
 * Refused when there is no frame to clear. Local coordinates keep the numbers
 * they have -- nothing moves -- they simply stop naming places on Earth, which
 * is the correct meaning of unlinking a document from the world. Undo restores
 * the exact frame, Mercator origin included.
 */
class ClearGeoreferenceCommand : public GeoreferenceCommand {
public:
    explicit ClearGeoreferenceCommand(Georeference& georeference);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

    /**
     * @brief Bytes this command holds, for CommandStackConfig::max_bytes
     *
     * Overridden because Command::footprint() defaults to sizeof(Command) -- the
     * vtable pointer, 8 bytes -- which under-reports this by 8x and leaves the
     * stack's memory bound counting something unrelated to what it holds. No
     * heap state, so the object's own size is the whole cost.
     */
    [[nodiscard]] size_t footprint() const override {
        return sizeof(ClearGeoreferenceCommand);
    }

private:
    /// What apply() removed. Always engaged between an apply() and its revert().
    std::optional<GeoreferenceFrame> m_previous;
};

// ============================================================================
// Convenience
// ============================================================================

/// Build an EstablishGeoreferenceCommand and run it through @p stack.
/// @return the stack's answer: false when the document was already georeferenced
///         or the origin was rejected.
bool establish_georeference(CommandStack& stack, Georeference& georeference,
                            const GeoPoint& origin, double scale = 1.0);

/**
 * @brief Build a RebaseGeoreferenceCommand and run it through @p stack
 *
 * @param delta_out Receives RebaseGeoreferenceCommand::local_delta(): what every
 *                  existing local coordinate must have added to it. A reference
 *                  and not an optional pointer ON PURPOSE -- a caller cannot
 *                  reach this function without naming a variable to catch the
 *                  delta in, which is the closest the type system gets to
 *                  "you are not done yet". Left untouched when the rebase is
 *                  refused.
 *
 * @return false when the rebase was refused. @p delta_out is then unmodified.
 *
 * @warning Call inside a CommandStack transaction together with the command that
 *          shifts the geometry, so undo takes both back as one step.
 */
bool rebase_georeference(CommandStack& stack, Georeference& georeference,
                         const GeoPoint& new_origin, glm::dvec2& delta_out);

/// Build a ClearGeoreferenceCommand and run it through @p stack.
/// @return false when the document was not georeferenced.
bool clear_georeference(CommandStack& stack, Georeference& georeference);

} // namespace stratum::scene
