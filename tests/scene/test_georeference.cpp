// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_georeference.cpp
 * @brief Georeference: the two states, the accuracy of the round trip, the
 *        rebase contract, and what a load has to preserve
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Almost everything this type can get wrong produces a plausible number rather
 * than a failure, so the tests are chosen for what they would catch, not for
 * coverage:
 *
 *   - **The missing frame.** `osm::CoordinateConverter::wgs84_to_local()`
 *     returns raw Web Mercator when no origin was set. The tests here assert
 *     `nullopt`, and assert it against the SAME coordinates that would have
 *     produced a believable Mercator pair, so a reintroduced fallback fails
 *     instead of passing quietly.
 *   - **The two states.** "Not georeferenced" and "georeferenced at 0 N 0 E" are
 *     checked side by side in one test, because a single-state implementation
 *     passes either of them alone.
 *   - **Mercator distortion.** Every distance test is at Dublin
 *     (53.3498 N, 6.2603 W), where the stretch factor is 1.675. At the equator
 *     it is 1.0, so an implementation that forgot the correction entirely would
 *     pass an equatorial test perfectly. There is an equatorial test here, and
 *     its only job is to contrast.
 *   - **The rebase delta's SIGN.** Checking that a rebase moved the coordinates
 *     by the delta it advertised passes for both signs, because the check and
 *     the implementation share the mistake. The test that does not is
 *     rebase_keeps_a_point_naming_the_same_place_on_earth: it feeds the shifted
 *     coordinate back through the NEW frame and demands the original latitude
 *     and longitude, which only the correct sign produces.
 *
 * Measured, not assumed: the WGS84 round trip over a 10 km box at Dublin is
 * worst-case 1.6e-9 m of ground error, and 1.2e-10 local metres when the
 * recovered position is re-projected. The tolerances below sit two orders above
 * those, to leave room for a different libm, and the tests that measure them
 * print nothing -- see the value in the CHECK when one fails.
 *
 * Every mutation goes through a CommandStack, never through a back door, because
 * "the only way to change the frame is a command" is one of the things under
 * test.
 *
 * ### The predicates the assertions lean on are tested first
 *
 * Six tests here assert "the frame was restored exactly" as
 * `*frame() == original`, and RebaseGeoreferenceCommand's stale-frame guard is
 * the same operator. A comparison that read only `origin_lat` would satisfy all
 * of them, because nothing else in the suite ever built two frames differing in
 * one field -- so the assertions would read as exact and be latitude checks.
 * frame_equality_compares_every_field_not_just_the_latitude and
 * geo_point_equality_compares_both_components pin the predicates themselves,
 * one field at a time, before anything else relies on them.
 *
 * For the same reason the fixtures below are deliberately awkward where the
 * default would hide a mistake: a frame from establish() has zero Mercator
 * drift and a scale of 1.0, so a revert that re-derived the Mercator origin
 * from lat/lon, or one that rebuilt the frame at scale 1.0, would be
 * bit-identical to a correct one. The restore tests therefore start from a
 * LOADED frame carrying 5 m of drift at a non-unit scale.
 */

#include "framework.hpp"

#include "scene/command.hpp"
#include "scene/georeference.hpp"

#include "osm/coordinates.hpp"
#include "osm/types.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

using stratum::scene::centre_of;
using stratum::scene::ClearGeoreferenceCommand;
using stratum::scene::clear_georeference;
using stratum::scene::CommandStack;
using stratum::scene::establish_georeference;
using stratum::scene::EstablishGeoreferenceCommand;
using stratum::scene::Georeference;
using stratum::scene::GeoreferenceFrame;
using stratum::scene::GeoPoint;
using stratum::scene::is_valid_frame;
using stratum::scene::is_valid_origin;
using stratum::scene::is_valid_scale;
using stratum::scene::kMaxMercatorLatitudeDeg;
using stratum::scene::mercator_origin_drift;
using stratum::scene::rebase_georeference;
using stratum::scene::RebaseGeoreferenceCommand;

using stratum::osm::BoundingBox;
using stratum::osm::CoordinateConverter;
using stratum::osm::DEG_TO_RAD;
using stratum::osm::METERS_PER_DEG_LAT;

namespace {

// ============================================================================
// Fixtures
// ============================================================================

/// The city this project is developed against, and the extract in
/// ~/Downloads/lucan.osm. Chosen over a synthetic origin because Mercator
/// distortion is 1.0 at the equator: a test near (0, 0) cannot tell a correct
/// implementation from one that ignores latitude altogether.
constexpr GeoPoint kDublin{53.3498, -6.2603};

/// Mercator stretch at Dublin: 1 / cos(53.3498 deg) = 1.6753. A local unit is
/// 0.596928 true ground metres there.
constexpr double kDublinGroundPerUnit = 0.596928038804;

/// A point roughly 2 km north-east of the origin: Clontarf.
constexpr GeoPoint kClontarf{53.3640, -6.2100};

/// Roughly the Lucan extract's western end, to keep a second real place around.
constexpr GeoPoint kLucan{53.3560, -6.4490};

/// Bounds shaped like a small city extract, in the order OSM reports them.
BoundingBox dublin_bounds() {
    BoundingBox bounds;
    bounds.expand(53.3000, -6.3500);
    bounds.expand(53.4000, -6.1500);
    return bounds;
}

/// A georeferenced document plus the stack every change to it goes through.
struct Document {
    CommandStack stack;
    Georeference georeference;
};

/// Establish @p origin through the stack, as a caller would. Returns the frame.
GeoreferenceFrame establish(Document& doc, GeoPoint origin = kDublin, double scale = 1.0) {
    const bool ok = establish_georeference(doc.stack, doc.georeference, origin, scale);
    // Both halves checked, so a caller that ignores the return still gets a
    // default frame rather than a dereferenced empty optional. Checks here are
    // non-fatal by design (framework.hpp) and a crash would take every suite
    // after this one with it.
    const std::optional<GeoreferenceFrame> frame = doc.georeference.frame();
    if (!ok || !frame) return GeoreferenceFrame{};
    return *frame;
}

/// Great-circle-free ground distance between two WGS84 points, good enough over
/// a city and derived WITHOUT the Mercator maths under test, so it can act as an
/// independent expectation.
double ground_metres_between(GeoPoint a, GeoPoint b) {
    const double mid_lat = (a.lat + b.lat) / 2.0;
    const double dy = (b.lat - a.lat) * METERS_PER_DEG_LAT;
    const double dx = (b.lon - a.lon) * CoordinateConverter::meters_per_degree_lon(mid_lat);
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

// ============================================================================
// The two states
// ============================================================================

TEST(Georeference, a_fresh_document_refuses_to_convert_rather_than_guessing) {
    const Georeference georeference;

    CHECK_FALSE(georeference.is_established());
    CHECK_FALSE(georeference.frame().has_value());
    CHECK_FALSE(georeference.origin().has_value());
    CHECK_FALSE(georeference.ground_metres_per_unit().has_value());
    CHECK_FALSE(georeference.coordinate_system().has_value());

    // The whole point of the type. osm::CoordinateConverter::wgs84_to_local()
    // answers this exact call with raw Web Mercator -- a believable pair of
    // numbers 7,000 km from the origin -- and nothing here may do the same.
    CHECK_FALSE(georeference.to_local(kDublin).has_value());
    CHECK_FALSE(georeference.to_local(kDublin.lat, kDublin.lon).has_value());
    CHECK_FALSE(georeference.to_wgs84(glm::dvec2{0.0, 0.0}).has_value());
    CHECK_FALSE(georeference.to_world(kDublin, 0.0).has_value());
    CHECK_FALSE(georeference.world_to_wgs84(glm::dvec3{0.0, 0.0, 0.0}).has_value());
}

TEST(Georeference, the_fallback_this_type_exists_to_remove_is_still_there_in_the_osm_chain) {
    // Not a test of scene code. It pins the behaviour the scene layer refuses to
    // inherit, so that if osm::CoordinateConverter is ever fixed, this fails and
    // somebody re-reads georeference.hpp's file comment rather than leaving it
    // describing a trap that no longer exists.
    const CoordinateConverter converter;  // set_origin() deliberately not called
    CHECK_FALSE(converter.is_initialized());

    const glm::dvec2 local = converter.wgs84_to_local(kDublin.lat, kDublin.lon);
    const glm::dvec2 mercator = CoordinateConverter::wgs84_to_mercator(kDublin.lat, kDublin.lon);
    CHECK_NEAR(local.x, mercator.x, 1e-9);
    CHECK_NEAR(local.y, mercator.y, 1e-9);

    // And it is enormous: 7,000 km of northing that looks like a coordinate.
    CHECK((std::fabs(local.y)) > (7.0e6));
}

TEST(Georeference, not_georeferenced_is_not_the_same_as_georeferenced_at_zero) {
    const Georeference never;

    Document at_null_island;
    establish(at_null_island, GeoPoint{0.0, 0.0});

    // Both hold an origin whose stored numbers are 0, 0. Only one of them is a
    // place. Checking these side by side is what a one-state implementation --
    // an origin field with no optional around it -- cannot pass.
    CHECK_FALSE(never.is_established());
    CHECK_TRUE(at_null_island.georeference.is_established());

    CHECK_FALSE(never.to_local(GeoPoint{0.0, 0.0}).has_value());

    const auto local = at_null_island.georeference.to_local(GeoPoint{0.0, 0.0});
    CHECK_TRUE(local.has_value());
    if (local) {
        CHECK_NEAR(local->x, 0.0, 1e-9);
        CHECK_NEAR(local->y, 0.0, 1e-9);
    }

    // And the equator frame really does convert: a degree east is 111 km out.
    const auto east = at_null_island.georeference.to_local(GeoPoint{0.0, 1.0});
    CHECK_TRUE(east.has_value());
    if (east) CHECK_NEAR(east->x, 111319.4908, 1e-3);
}

// ============================================================================
// Conversion at a real place
// ============================================================================

TEST(Georeference, the_origin_converts_to_the_local_origin) {
    Document doc;
    establish(doc);

    const auto local = doc.georeference.to_local(kDublin);
    CHECK_TRUE(local.has_value());
    if (!local) return;
    CHECK_NEAR(local->x, 0.0, 1e-9);
    CHECK_NEAR(local->y, 0.0, 1e-9);

    const auto origin = doc.georeference.origin();
    CHECK_TRUE(origin.has_value());
    if (origin) {
        CHECK_NEAR(origin->lat, kDublin.lat, 0.0);
        CHECK_NEAR(origin->lon, kDublin.lon, 0.0);
    }
}

TEST(Georeference, north_is_positive_y_and_east_is_positive_x_at_dublin) {
    Document doc;
    establish(doc);

    const auto north = doc.georeference.to_local(GeoPoint{kDublin.lat + 0.01, kDublin.lon});
    const auto east = doc.georeference.to_local(GeoPoint{kDublin.lat, kDublin.lon + 0.01});
    CHECK_TRUE(north.has_value());
    CHECK_TRUE(east.has_value());
    if (!north || !east) return;

    // Signs, not magnitudes: a transposed or negated projection still produces
    // the right distances and puts the city in the wrong quadrant.
    CHECK((north->y) > (0.0));
    CHECK_NEAR(north->x, 0.0, 1e-6);
    CHECK((east->x) > (0.0));
    CHECK_NEAR(east->y, 0.0, 1e-6);
}

TEST(Georeference, dublin_round_trips_to_a_nanometre_over_a_ten_kilometre_extract) {
    Document doc;
    const GeoreferenceFrame frame = establish(doc);

    // A 10 km by 10 km box, which is the size of the Lucan extract, sampled on a
    // 101 x 101 grid so the corners and the origin are both covered.
    double worst_ground_metres = 0.0;
    double worst_local_metres = 0.0;

    for (int i = -50; i <= 50; ++i) {
        for (int j = -50; j <= 50; ++j) {
            const GeoPoint p{kDublin.lat + i * 0.001, kDublin.lon + j * 0.0015};

            const auto local = doc.georeference.to_local(p);
            CHECK_TRUE(local.has_value());
            if (!local) return;

            const auto back = doc.georeference.to_wgs84(*local);
            CHECK_TRUE(back.has_value());
            if (!back) return;

            worst_ground_metres = std::max(worst_ground_metres, ground_metres_between(p, *back));

            // The other direction of the same question: does the recovered
            // position project back to the coordinate it came from?
            worst_local_metres =
                std::max(worst_local_metres, glm::length(frame.to_local(*back) - *local));
        }
    }

    // Measured worst case is 1.6e-9 m of ground and 1.2e-10 local m; the bound is
    // two orders looser so a different libm does not fail the build, and still
    // three orders tighter than anything that would matter to geometry.
    CHECK((worst_ground_metres) < (1.0e-7));
    CHECK((worst_local_metres) < (1.0e-8));
}

TEST(Georeference, the_two_argument_overload_takes_latitude_first) {
    Document doc;
    establish(doc);

    const auto by_point = doc.georeference.to_local(kClontarf);
    const auto by_pair = doc.georeference.to_local(kClontarf.lat, kClontarf.lon);
    CHECK_TRUE(by_point.has_value());
    CHECK_TRUE(by_pair.has_value());
    if (!by_point || !by_pair) return;
    CHECK_NEAR(by_pair->x, by_point->x, 0.0);
    CHECK_NEAR(by_pair->y, by_point->y, 0.0);

    // And the swapped pair is somewhere else entirely -- 53 E, 6 S, in the
    // Indian Ocean -- so a reversed overload could not pass the check above.
    const auto swapped = doc.georeference.to_local(kClontarf.lon, kClontarf.lat);
    CHECK_TRUE(swapped.has_value());
    if (swapped) CHECK((glm::length(*swapped - *by_point)) > (1.0e6));
}

// ============================================================================
// Mercator distortion
// ============================================================================

TEST(Georeference, local_distances_at_dublin_are_mercator_metres_not_ground_metres) {
    Document doc;
    establish(doc);

    const auto south = doc.georeference.to_local(GeoPoint{kDublin.lat - 0.005, kDublin.lon});
    const auto north = doc.georeference.to_local(GeoPoint{kDublin.lat + 0.005, kDublin.lon});
    CHECK_TRUE(south.has_value());
    CHECK_TRUE(north.has_value());
    if (!south || !north) return;

    // 0.01 degrees of latitude is 1113.2 m of real ground anywhere on Earth.
    // In this frame it measures 1864.9 local units, because Web Mercator
    // stretches by 1 / cos(53.3498 deg) = 1.6753.
    const double local_span = glm::length(*north - *south);
    CHECK_NEAR(local_span, 1864.87, 0.05);

    // Stated as an inequality as well, so the test cannot pass by the two
    // numbers happening to agree: the local span is emphatically NOT the ground
    // span, and any implementation that quietly corrected it would fail here.
    CHECK((local_span) > (1800.0));
    CHECK((local_span) > (1.5 * 0.01 * METERS_PER_DEG_LAT));
}

TEST(Georeference, ground_metres_per_unit_turns_a_dublin_span_back_into_real_metres) {
    Document doc;
    establish(doc);

    const auto factor = doc.georeference.ground_metres_per_unit();
    CHECK_TRUE(factor.has_value());
    if (!factor) return;
    CHECK_NEAR(*factor, kDublinGroundPerUnit, 1e-9);

    const auto south = doc.georeference.to_local(GeoPoint{kDublin.lat - 0.005, kDublin.lon});
    const auto north = doc.georeference.to_local(GeoPoint{kDublin.lat + 0.005, kDublin.lon});
    if (!south || !north) return;

    const double ground = glm::length(*north - *south) * *factor;

    // 1113.2 m, to 5 cm over 1.1 km. The three wrong answers are all far outside
    // that: no correction gives 1864.9, the reciprocal gives 3124.1, and a
    // correction taken at the equator gives 1864.9 again.
    CHECK_NEAR(ground, 0.01 * METERS_PER_DEG_LAT, 0.05);
}

TEST(Georeference, the_distortion_factor_depends_on_latitude) {
    // The contrast test. Everything above would pass at the equator for an
    // implementation that returned a constant 1.0, because 1 / cos(0) is 1.
    Document equator;
    establish(equator, GeoPoint{0.0, 0.0});

    Document dublin;
    establish(dublin, kDublin);

    Document tromso;
    establish(tromso, GeoPoint{69.6496, 18.9560});

    const auto at_equator = equator.georeference.ground_metres_per_unit();
    const auto at_dublin = dublin.georeference.ground_metres_per_unit();
    const auto at_tromso = tromso.georeference.ground_metres_per_unit();
    CHECK_TRUE(at_equator.has_value());
    CHECK_TRUE(at_dublin.has_value());
    CHECK_TRUE(at_tromso.has_value());
    if (!at_equator || !at_dublin || !at_tromso) return;

    CHECK_NEAR(*at_equator, 1.0, 1e-12);
    CHECK_NEAR(*at_dublin, kDublinGroundPerUnit, 1e-9);
    CHECK_NEAR(*at_tromso, std::cos(69.6496 * DEG_TO_RAD), 1e-12);

    // Strictly decreasing with latitude, and by a lot: a constant factor fails.
    CHECK((*at_dublin) < (*at_equator));
    CHECK((*at_tromso) < (*at_dublin));
    CHECK((*at_equator - *at_dublin) > (0.3));
}

// ============================================================================
// Y-up world space
// ============================================================================

TEST(Georeference, world_space_puts_north_at_negative_z_and_passes_height_through) {
    Document doc;
    establish(doc);

    const auto north = doc.georeference.to_world(GeoPoint{kDublin.lat + 0.01, kDublin.lon}, 12.5);
    const auto east = doc.georeference.to_world(GeoPoint{kDublin.lat, kDublin.lon + 0.01}, -3.0);
    CHECK_TRUE(north.has_value());
    CHECK_TRUE(east.has_value());
    if (!north || !east) return;

    // The Z flip is the whole convention, and dropping it mirrors the city
    // north-south -- a change that keeps every distance correct.
    CHECK((north->z) < (0.0));
    CHECK_NEAR(north->x, 0.0, 1e-6);
    CHECK_NEAR(north->y, 12.5, 0.0);

    CHECK((east->x) > (0.0));
    CHECK_NEAR(east->z, 0.0, 1e-6);
    CHECK_NEAR(east->y, -3.0, 0.0);

    // Magnitude unchanged by the flip: only the sign moved.
    const auto north_local = doc.georeference.to_local(GeoPoint{kDublin.lat + 0.01, kDublin.lon});
    if (north_local) CHECK_NEAR(north->z, -north_local->y, 1e-9);
}

TEST(Georeference, a_world_position_converts_back_to_the_place_it_came_from) {
    Document doc;
    establish(doc);

    const auto world = doc.georeference.to_world(kClontarf, 42.0);
    CHECK_TRUE(world.has_value());
    if (!world) return;

    const auto back = doc.georeference.world_to_wgs84(*world);
    CHECK_TRUE(back.has_value());
    if (!back) return;

    CHECK_NEAR(back->lat, kClontarf.lat, 1e-12);
    CHECK_NEAR(back->lon, kClontarf.lon, 1e-12);

    // Height is not part of the position on Earth, so changing it must not move
    // the answer. Without the flip being symmetric this drifts.
    const auto lifted = doc.georeference.world_to_wgs84(glm::dvec3{world->x, 900.0, world->z});
    if (lifted) {
        CHECK_NEAR(lifted->lat, kClontarf.lat, 1e-12);
        CHECK_NEAR(lifted->lon, kClontarf.lon, 1e-12);
    }
}

// ============================================================================
// The predicates the rest of the file asserts through
// ============================================================================

TEST(Georeference, geo_point_equality_compares_both_components) {
    CHECK_TRUE(kDublin == kDublin);
    CHECK_FALSE(kDublin != kDublin);

    // A nanodegree is 0.1 mm. Not a tolerance, on purpose: the no-op guard in
    // RebaseGeoreferenceCommand compares GeoPoints, so anything that read a
    // small deliberate move as "the origin it already has" would drop it.
    const GeoPoint lat_moved{kDublin.lat + 1e-9, kDublin.lon};
    const GeoPoint lon_moved{kDublin.lat, kDublin.lon + 1e-9};
    const GeoPoint swapped{kDublin.lon, kDublin.lat};

    CHECK_FALSE(kDublin == lat_moved);
    CHECK_TRUE(kDublin != lat_moved);
    CHECK_FALSE(kDublin == lon_moved);
    CHECK_TRUE(kDublin != lon_moved);
    CHECK_FALSE(kDublin == swapped);
    CHECK_TRUE(kDublin != swapped);

    // operator!= spelled out against a match as well as against a miss, because
    // an inverted one -- `return a == b;` -- passes every check that only ever
    // asks it about points that differ.
    CHECK_FALSE(lat_moved != lat_moved);
    CHECK_TRUE(lat_moved != lon_moved);
}

TEST(Georeference, frame_equality_compares_every_field_not_just_the_latitude) {
    // Two of the four fields never move on their own anywhere else in this file:
    // a rebase moves lat, lon and origin_mercator together, and no test but this
    // one changes scale alone. A comparison that dropped origin_lon,
    // origin_mercator or scale would therefore pass the whole suite -- and it is
    // the stale-frame guard, so a field it does not read is a field the document
    // can change in while a held command still believes it measured the current
    // frame.
    const GeoreferenceFrame base = GeoreferenceFrame::at(kDublin, 2.0);

    CHECK_TRUE(base == base);
    CHECK_FALSE(base != base);

    GeoreferenceFrame lat = base;
    lat.origin_lat += 1e-9;
    GeoreferenceFrame lon = base;
    lon.origin_lon += 1e-9;
    GeoreferenceFrame mercator_x = base;
    mercator_x.origin_mercator.x += 5.0;
    GeoreferenceFrame mercator_y = base;
    mercator_y.origin_mercator.y += 5.0;
    GeoreferenceFrame scale = base;
    scale.scale = 2.5;

    CHECK_FALSE(base == lat);
    CHECK_TRUE(base != lat);
    CHECK_FALSE(base == lon);
    CHECK_TRUE(base != lon);
    CHECK_FALSE(base == mercator_x);
    CHECK_TRUE(base != mercator_x);
    CHECK_FALSE(base == mercator_y);
    CHECK_TRUE(base != mercator_y);
    CHECK_FALSE(base == scale);
    CHECK_TRUE(base != scale);

    // A copy of the same five doubles is equal, so the checks above are about
    // the fields and not about frames being compared by identity.
    GeoreferenceFrame copy;
    copy.origin_lat = base.origin_lat;
    copy.origin_lon = base.origin_lon;
    copy.origin_mercator = base.origin_mercator;
    copy.scale = base.scale;
    CHECK_TRUE(base == copy);
    CHECK_FALSE(base != copy);
}

// ============================================================================
// Bounding box unpacking
// ============================================================================

TEST(Georeference, centre_of_unpacks_the_lat_lon_order_a_bounding_box_packs) {
    const BoundingBox bounds = dublin_bounds();
    const GeoPoint centre = centre_of(bounds);

    // The trap: BoundingBox::center() returns (lat, lon) in a dvec2, while every
    // other dvec2 in Stratum is (x, y). Reading .x as an easting here would give
    // a longitude of 53.35, which is a valid coordinate in Kazakhstan.
    CHECK_NEAR(centre.lat, 53.35, 1e-12);
    CHECK_NEAR(centre.lon, -6.25, 1e-12);

    CHECK((centre.lat) > (0.0));
    CHECK((centre.lon) < (0.0));

    // And it agrees with the packed form, component by component, in that order.
    const glm::dvec2 packed = bounds.center();
    CHECK_NEAR(centre.lat, packed.x, 0.0);
    CHECK_NEAR(centre.lon, packed.y, 0.0);
}

TEST(Georeference, a_frame_established_on_extract_bounds_puts_the_extract_around_the_origin) {
    Document doc;
    establish(doc, centre_of(dublin_bounds()));

    // Every corner of the extract lands within a few kilometres of the origin.
    // This is the check that fails loudly when lat and lon are swapped anywhere
    // in the chain: the corners would be thousands of kilometres out.
    for (const GeoPoint corner : {GeoPoint{53.3000, -6.3500}, GeoPoint{53.4000, -6.3500},
                                  GeoPoint{53.3000, -6.1500}, GeoPoint{53.4000, -6.1500}}) {
        const auto local = doc.georeference.to_local(corner);
        CHECK_TRUE(local.has_value());
        if (local) CHECK((glm::length(*local)) < (20000.0));
    }

    const auto lucan = doc.georeference.to_local(kLucan);
    CHECK_TRUE(lucan.has_value());
    if (lucan) CHECK((glm::length(*lucan)) < (30000.0));
}

// ============================================================================
// Establish
// ============================================================================

TEST(Georeference, a_create_knows_its_frame_before_it_is_executed) {
    Document doc;
    auto command = std::make_unique<EstablishGeoreferenceCommand>(doc.georeference, kDublin);

    CHECK_TRUE(command->valid());
    const GeoreferenceFrame planned = command->frame();
    CHECK_NEAR(planned.origin_lat, kDublin.lat, 0.0);
    CHECK_FALSE(doc.georeference.is_established());

    CHECK_TRUE(doc.stack.execute(std::move(command)));
    CHECK_TRUE(doc.georeference.is_established());
    const auto frame = doc.georeference.frame();
    CHECK_TRUE(frame.has_value());
    if (frame) CHECK_TRUE(*frame == planned);
}

TEST(Georeference, establishing_twice_is_refused_and_records_nothing) {
    Document doc;
    const GeoreferenceFrame first = establish(doc, kDublin);
    const size_t depth = doc.stack.undo_depth();
    const uint64_t generation = doc.georeference.generation();

    CHECK_FALSE(establish_georeference(doc.stack, doc.georeference, kClontarf));

    CHECK_EQ(doc.stack.undo_depth(), depth);
    const auto frame = doc.georeference.frame();
    CHECK_TRUE(frame.has_value());
    if (frame) CHECK_TRUE(*frame == first);
    CHECK_EQ(doc.georeference.generation(), generation);
}

TEST(Georeference, an_origin_outside_the_mercator_band_is_refused_rather_than_clamped) {
    // The OSM converter clamps: these two project to the same point, so a
    // document anchored at 89 N would silently be anchored at 85.051128 N,
    // 435 km away, and every coordinate in it would be consistent and wrong.
    const glm::dvec2 asked = CoordinateConverter::wgs84_to_mercator(89.0, 0.0);
    const glm::dvec2 clamped = CoordinateConverter::wgs84_to_mercator(kMaxMercatorLatitudeDeg, 0.0);
    CHECK_NEAR(asked.y, clamped.y, 1e-6);

    CHECK_FALSE(is_valid_origin(GeoPoint{89.0, 0.0}));
    CHECK_FALSE(is_valid_origin(GeoPoint{-89.0, 0.0}));
    CHECK_FALSE(is_valid_origin(GeoPoint{53.3498, 200.0}));
    CHECK_TRUE(is_valid_origin(kDublin));
    CHECK_TRUE(is_valid_origin(GeoPoint{0.0, 0.0}));

    // Both boundaries, to the ulp. Sampling 89 and 200 leaves the edge itself
    // undecided -- `>` and `>=` both pass, and so do `<= 180` and `< 180` -- and
    // where a refusal starts is a decision somebody will have to reproduce.
    // Decided here: exactly on the band edge is usable, exactly on the
    // antimeridian is usable, one ulp past either is not.
    CHECK_TRUE(is_valid_origin(GeoPoint{kMaxMercatorLatitudeDeg, 0.0}));
    CHECK_TRUE(is_valid_origin(GeoPoint{-kMaxMercatorLatitudeDeg, 0.0}));
    CHECK_FALSE(is_valid_origin(GeoPoint{std::nextafter(kMaxMercatorLatitudeDeg, 90.0), 0.0}));
    CHECK_FALSE(is_valid_origin(GeoPoint{std::nextafter(-kMaxMercatorLatitudeDeg, -90.0), 0.0}));
    CHECK_TRUE(is_valid_origin(GeoPoint{0.0, 180.0}));
    CHECK_TRUE(is_valid_origin(GeoPoint{0.0, -180.0}));
    CHECK_FALSE(is_valid_origin(GeoPoint{0.0, std::nextafter(180.0, 181.0)}));
    CHECK_FALSE(is_valid_origin(GeoPoint{0.0, std::nextafter(-180.0, -181.0)}));

    Document doc;
    const uint64_t generation = doc.georeference.generation();
    CHECK_FALSE(establish_georeference(doc.stack, doc.georeference, GeoPoint{89.0, 0.0}));
    CHECK_FALSE(doc.georeference.is_established());
    CHECK_EQ(doc.stack.undo_depth(), size_t{0});
    CHECK_EQ(doc.georeference.generation(), generation);

    // And a refusal is not a partial application: the document still converts
    // nothing, rather than converting against a clamped frame.
    CHECK_FALSE(doc.georeference.to_local(kDublin).has_value());
}

TEST(Georeference, a_non_finite_origin_is_refused) {
    // Not a hypothetical. centre_of() on an empty extract, a failed parse, or a
    // 0/0 in a caller's own averaging all produce one, and NaN propagates
    // through the projection silently: every comparison against it is false, so
    // a range check written as `lat > 85` lets it straight through and the
    // document ends up anchored nowhere at all.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    CHECK_FALSE(is_valid_origin(GeoPoint{nan, -6.2603}));
    CHECK_FALSE(is_valid_origin(GeoPoint{53.3498, nan}));
    CHECK_FALSE(is_valid_origin(GeoPoint{inf, 0.0}));
    CHECK_FALSE(is_valid_origin(GeoPoint{0.0, -inf}));
    CHECK_FALSE(is_valid_scale(nan));
    CHECK_FALSE(is_valid_scale(inf));

    Document doc;
    CHECK_FALSE(establish_georeference(doc.stack, doc.georeference, GeoPoint{nan, nan}));
    CHECK_FALSE(establish_georeference(doc.stack, doc.georeference, kDublin, nan));
    CHECK_FALSE(doc.georeference.is_established());
    CHECK_EQ(doc.stack.undo_depth(), size_t{0});

    // A rebase is the same question a second time, and the frame it would land
    // on is one no conversion could recover from.
    const GeoreferenceFrame frame = establish(doc, kDublin);
    const uint64_t generation = doc.georeference.generation();
    glm::dvec2 delta{0.0, 0.0};
    CHECK_FALSE(rebase_georeference(doc.stack, doc.georeference, GeoPoint{nan, 0.0}, delta));

    const auto after = doc.georeference.frame();
    CHECK_TRUE(after.has_value());
    if (after) CHECK_TRUE(*after == frame);
    CHECK_EQ(doc.georeference.generation(), generation);
}

TEST(Georeference, a_non_positive_scale_is_refused) {
    CHECK_FALSE(is_valid_scale(0.0));
    CHECK_FALSE(is_valid_scale(-1.0));
    CHECK_TRUE(is_valid_scale(1.0));
    CHECK_TRUE(is_valid_scale(0.001));

    Document doc;
    CHECK_FALSE(establish_georeference(doc.stack, doc.georeference, kDublin, 0.0));
    CHECK_FALSE(establish_georeference(doc.stack, doc.georeference, kDublin, -2.0));
    CHECK_FALSE(doc.georeference.is_established());
    CHECK_EQ(doc.stack.undo_depth(), size_t{0});
    CHECK_EQ(doc.georeference.generation(), uint64_t{0});
}

TEST(Georeference, undo_of_establish_returns_the_document_to_not_georeferenced) {
    Document doc;
    establish(doc);
    CHECK_TRUE(doc.stack.undo());

    // Not "back to 0, 0". The distinction is the feature: an implementation that
    // reverted to a default frame would leave is_established() true and go on
    // converting, which is the silent failure this whole file is about.
    CHECK_FALSE(doc.georeference.is_established());
    CHECK_FALSE(doc.georeference.frame().has_value());
    CHECK_FALSE(doc.georeference.to_local(kDublin).has_value());
}

TEST(Georeference, redo_of_establish_restores_the_frame_exactly) {
    Document doc;
    // Scale 3.0 and not the default. At 1.0 the scale field is compared against
    // its own default, so a redo that rebuilt the frame from the origin alone --
    // GeoreferenceFrame::at(origin) -- would restore a frame that is not the one
    // it removed and no check here would notice.
    const GeoreferenceFrame original = establish(doc, kDublin, 3.0);
    CHECK_NEAR(original.scale, 3.0, 0.0);

    CHECK_TRUE(doc.stack.undo());
    CHECK_TRUE(doc.stack.redo());

    CHECK_TRUE(doc.georeference.is_established());
    const auto frame = doc.georeference.frame();
    CHECK_TRUE(frame.has_value());
    if (!frame) return;

    // Exact, not near: a frame restored to within a metre is a document whose
    // geometry has moved a metre.
    CHECK_TRUE(*frame == original);
    CHECK_NEAR(frame->scale, 3.0, 0.0);
    CHECK_NEAR(frame->origin_mercator.x, original.origin_mercator.x, 0.0);
    CHECK_NEAR(frame->origin_mercator.y, original.origin_mercator.y, 0.0);
}

// ============================================================================
// Rebase
// ============================================================================

TEST(Georeference, a_rebase_knows_its_delta_before_it_is_executed) {
    Document doc;
    const GeoreferenceFrame before = establish(doc);

    const GeoPoint moved{kDublin.lat, kDublin.lon + 0.01};
    auto command = std::make_unique<RebaseGeoreferenceCommand>(doc.georeference, moved);
    CHECK_TRUE(command->valid());

    const glm::dvec2 delta = command->local_delta();
    // The origin moved east, so everything is now further west of it.
    CHECK((delta.x) < (0.0));
    CHECK_NEAR(delta.y, 0.0, 1e-9);
    CHECK_NEAR(delta.x, -1113.194908, 1e-3);

    // Reading it changed nothing.
    const auto unchanged = doc.georeference.frame();
    CHECK_TRUE(unchanged.has_value());
    if (unchanged) CHECK_TRUE(*unchanged == before);

    CHECK_TRUE(doc.stack.execute(std::move(command)));
    const auto origin = doc.georeference.origin();
    CHECK_TRUE(origin.has_value());
    if (origin) CHECK_NEAR(origin->lon, moved.lon, 1e-12);
}

TEST(Georeference, a_rebase_moves_every_local_coordinate_by_the_delta_it_advertised) {
    Document doc;
    establish(doc);

    const GeoPoint samples[] = {kDublin, kClontarf, kLucan, GeoPoint{53.30, -6.35}};
    glm::dvec2 before[4];
    for (int i = 0; i < 4; ++i) {
        const auto local = doc.georeference.to_local(samples[i]);
        CHECK_TRUE(local.has_value());
        if (!local) return;
        before[i] = *local;
    }

    glm::dvec2 delta{0.0, 0.0};
    CHECK_TRUE(rebase_georeference(doc.stack, doc.georeference, kClontarf, delta));

    // One vector for every point, whatever its distance from either origin. That
    // independence is what makes a rebase something a caller can apply.
    for (int i = 0; i < 4; ++i) {
        const auto after = doc.georeference.to_local(samples[i]);
        CHECK_TRUE(after.has_value());
        if (!after) return;
        CHECK_NEAR(after->x, before[i].x + delta.x, 1e-6);
        CHECK_NEAR(after->y, before[i].y + delta.y, 1e-6);
    }

    // The new origin is now the local origin, which is the visible consequence.
    const auto at_new_origin = doc.georeference.to_local(kClontarf);
    if (at_new_origin) CHECK((glm::length(*at_new_origin)) < (1e-6));
}

TEST(Georeference, rebase_keeps_a_point_naming_the_same_place_on_earth) {
    Document doc;
    establish(doc);

    // The test that a sign error cannot survive. Checking "the coordinates moved
    // by the delta" passes for both signs, because the check and the bug share
    // the same delta. This one takes the shifted coordinate back through the NEW
    // frame and demands the ORIGINAL latitude and longitude.
    const auto before = doc.georeference.to_local(kLucan);
    CHECK_TRUE(before.has_value());
    if (!before) return;

    glm::dvec2 delta{0.0, 0.0};
    CHECK_TRUE(rebase_georeference(doc.stack, doc.georeference, kClontarf, delta));

    const auto recovered = doc.georeference.to_wgs84(*before + delta);
    CHECK_TRUE(recovered.has_value());
    if (!recovered) return;

    CHECK_NEAR(recovered->lat, kLucan.lat, 1e-9);
    CHECK_NEAR(recovered->lon, kLucan.lon, 1e-9);

    // Shifting the other way, which is what an inverted delta would do, lands
    // several kilometres from Lucan. Named here so the tolerance above is
    // clearly doing work.
    const auto wrong_way = doc.georeference.to_wgs84(*before - delta);
    if (wrong_way) CHECK((ground_metres_between(*wrong_way, kLucan)) > (1000.0));
}

TEST(Georeference, a_rebase_carries_the_scale_across_and_scales_its_delta) {
    Document doc;
    establish(doc, kDublin, 2.0);

    const auto established = doc.georeference.frame();
    CHECK_TRUE(established.has_value());
    if (!established) return;
    const glm::dvec2 origin_before = established->origin_mercator;

    glm::dvec2 delta{0.0, 0.0};
    CHECK_TRUE(rebase_georeference(doc.stack, doc.georeference, kClontarf, delta));

    const auto frame = doc.georeference.frame();
    CHECK_TRUE(frame.has_value());
    if (!frame) return;

    // No parameter for it, so it cannot change. A rebase that reset the scale to
    // 1.0 would halve every coordinate in the document and still look like a
    // translation.
    CHECK_NEAR(frame->scale, 2.0, 0.0);

    // The delta is in LOCAL units, which are scaled ones. Computing it in
    // Mercator metres and handing it over unscaled would move the geometry half
    // as far as the frame moved.
    const glm::dvec2 mercator_shift = origin_before - frame->origin_mercator;
    CHECK_NEAR(delta.x, mercator_shift.x * 2.0, 1e-6);
    CHECK_NEAR(delta.y, mercator_shift.y * 2.0, 1e-6);
    CHECK((std::fabs(delta.x - mercator_shift.x)) > (1.0));
}

TEST(Georeference, rebasing_a_document_that_was_never_georeferenced_is_refused) {
    Document doc;
    glm::dvec2 delta{7.0, 9.0};

    CHECK_FALSE(rebase_georeference(doc.stack, doc.georeference, kDublin, delta));
    CHECK_FALSE(doc.georeference.is_established());
    CHECK_EQ(doc.stack.undo_depth(), size_t{0});

    // A refused rebase must not hand back a delta. The caller is about to add it
    // to every vertex it owns.
    CHECK_NEAR(delta.x, 7.0, 0.0);
    CHECK_NEAR(delta.y, 9.0, 0.0);
}

TEST(Georeference, rebasing_to_the_origin_it_already_has_is_refused) {
    Document doc;
    const GeoreferenceFrame frame = establish(doc);
    const size_t depth = doc.stack.undo_depth();
    const uint64_t generation = doc.georeference.generation();

    glm::dvec2 delta{0.0, 0.0};
    CHECK_FALSE(rebase_georeference(doc.stack, doc.georeference, kDublin, delta));

    CHECK_EQ(doc.stack.undo_depth(), depth);
    const auto after = doc.georeference.frame();
    CHECK_TRUE(after.has_value());
    if (after) CHECK_TRUE(*after == frame);
    CHECK_EQ(doc.georeference.generation(), generation);
}

TEST(Georeference, a_rebase_measured_against_a_stale_frame_refuses) {
    Document doc;
    establish(doc);

    // Built now, against the Dublin frame, and held.
    auto stale = std::make_unique<RebaseGeoreferenceCommand>(doc.georeference, kClontarf);
    CHECK_TRUE(stale->valid());
    const glm::dvec2 stale_delta = stale->local_delta();

    // Somebody else moves the origin first.
    glm::dvec2 other_delta{0.0, 0.0};
    CHECK_TRUE(rebase_georeference(doc.stack, doc.georeference, kLucan, other_delta));
    const auto moved = doc.georeference.frame();
    CHECK_TRUE(moved.has_value());
    if (!moved) return;
    const GeoreferenceFrame after_other = *moved;
    const size_t depth = doc.stack.undo_depth();
    const uint64_t generation = doc.georeference.generation();

    // Applying the held command now would shift the scene by the difference
    // between two deltas, silently. It refuses instead, and refusing leaves the
    // frame and the history exactly as they were.
    CHECK_FALSE(doc.stack.execute(std::move(stale)));
    const auto unchanged = doc.georeference.frame();
    CHECK_TRUE(unchanged.has_value());
    if (unchanged) CHECK_TRUE(*unchanged == after_other);
    CHECK_EQ(doc.stack.undo_depth(), depth);
    CHECK_EQ(doc.georeference.generation(), generation);

    // The two deltas really do disagree, so the refusal is not vacuous.
    CHECK((glm::length(stale_delta - other_delta)) > (100.0));
}

TEST(Georeference, undo_of_a_rebase_restores_the_previous_frame_exactly) {
    Document doc;

    // Loaded rather than established, so the starting frame carries 5 m of
    // Mercator drift and a scale of 2.0. Against a frame from establish() a
    // revert that rebuilt it with GeoreferenceFrame::at(origin) instead of
    // restoring the one it measured is bit-identical, and "restores the previous
    // frame exactly" would be asserting nothing about either field.
    GeoreferenceFrame original = GeoreferenceFrame::at(kDublin, 2.0);
    original.origin_mercator.x += 5.0;
    CHECK_TRUE(doc.georeference.load(original));

    const auto before = doc.georeference.to_local(kLucan);

    glm::dvec2 delta{0.0, 0.0};
    CHECK_TRUE(rebase_georeference(doc.stack, doc.georeference, kClontarf, delta));
    CHECK_TRUE(doc.stack.undo());

    const auto restored = doc.georeference.frame();
    CHECK_TRUE(restored.has_value());
    if (!restored) return;
    CHECK_TRUE(*restored == original);
    CHECK_NEAR(mercator_origin_drift(*restored), 5.0, 1e-9);
    CHECK_NEAR(restored->scale, 2.0, 0.0);

    const auto after = doc.georeference.to_local(kLucan);
    CHECK_TRUE(before.has_value());
    CHECK_TRUE(after.has_value());
    if (before && after) {
        CHECK_NEAR(after->x, before->x, 0.0);
        CHECK_NEAR(after->y, before->y, 0.0);
    }

    CHECK_TRUE(doc.stack.redo());
    const auto origin = doc.georeference.origin();
    CHECK_TRUE(origin.has_value());
    if (origin) CHECK_NEAR(origin->lat, kClontarf.lat, 0.0);
}

TEST(Georeference, a_rebase_refuses_when_only_the_stored_mercator_origin_moved_under_it) {
    Document doc;
    const GeoreferenceFrame at_dublin = establish(doc, kDublin);

    // Built now, against the Dublin frame, and held.
    auto stale = std::make_unique<RebaseGeoreferenceCommand>(doc.georeference, kClontarf);
    CHECK_TRUE(stale->valid());

    // The frame is then replaced by one at the SAME lat and lon, differing only
    // in its stored Mercator origin -- which is what opening a saved document
    // does. The held command's delta is 5 m wrong against it, a lane's width on
    // every vertex the caller is about to shift, and a guard that compared
    // origins rather than whole frames would find nothing to refuse.
    GeoreferenceFrame drifted = at_dublin;
    drifted.origin_mercator.x += 5.0;
    doc.stack.clear();
    CHECK_TRUE(doc.georeference.load(drifted));
    CHECK_TRUE(drifted.origin() == at_dublin.origin());

    CHECK_FALSE(doc.stack.execute(std::move(stale)));
    const auto after = doc.georeference.frame();
    CHECK_TRUE(after.has_value());
    if (after) CHECK_TRUE(*after == drifted);
    CHECK_EQ(doc.stack.undo_depth(), size_t{0});
}

TEST(Georeference, a_rebase_across_the_antimeridian_is_not_wrapped) {
    // Documented behaviour, not an accident, and pinned here so that changing it
    // is a decision rather than a surprise. See "Not antimeridian-aware" in
    // georeference.hpp: Web Mercator easting is not periodic and nothing here
    // wraps it, because a wrapped delta would be a lie for the points on the far
    // side of the seam and a non-uniform translation is not something a caller
    // can shift geometry by.
    Document doc;
    establish(doc, GeoPoint{0.0, 179.9});

    glm::dvec2 delta{0.0, 0.0};
    CHECK_TRUE(rebase_georeference(doc.stack, doc.georeference, GeoPoint{0.0, -179.9}, delta));

    // The two origins are 0.2 degrees apart on the ground -- 22 km at the
    // equator -- and the local frame moved three orders further than that to
    // follow them, the long way round the planet.
    const double ground_apart = 0.2 * CoordinateConverter::meters_per_degree_lon(0.0);
    CHECK((ground_apart) < (25000.0));
    CHECK((std::fabs(delta.x)) > (1000.0 * ground_apart));
    CHECK((std::fabs(delta.x)) > (4.0e7));

    // Every number stays correct: the frame is still self-consistent and a point
    // near the seam still round-trips to the coordinate it came from.
    const auto local = doc.georeference.to_local(GeoPoint{0.0, 179.95});
    CHECK_TRUE(local.has_value());
    if (!local) return;
    const auto back = doc.georeference.to_wgs84(*local);
    CHECK_TRUE(back.has_value());
    if (back) CHECK_NEAR(back->lon, 179.95, 1e-9);

    // The damage is the magnitude, and it is in the renderer, which uploads
    // floats: consecutive floats out at 4.0e7 are metres apart, so the geometry
    // quantises to lane widths while the maths above stays exact. Re-import
    // against an origin on the extract's own side rather than rebasing across.
    CHECK((std::fabs(local->x)) > (4.0e7));
    const float as_float = static_cast<float>(local->x);
    const double float_step = static_cast<double>(std::nextafterf(as_float, 1.0e30f)) -
                              static_cast<double>(as_float);
    CHECK((float_step) > (1.0));
}

// ============================================================================
// Generation
// ============================================================================

TEST(Georeference, the_generation_moves_on_every_change_of_frame_including_undo) {
    Document doc;
    const uint64_t fresh = doc.georeference.generation();

    establish(doc);
    const uint64_t after_establish = doc.georeference.generation();
    CHECK((fresh) < (after_establish));

    glm::dvec2 delta{0.0, 0.0};
    CHECK_TRUE(rebase_georeference(doc.stack, doc.georeference, kClontarf, delta));
    const uint64_t after_rebase = doc.georeference.generation();
    CHECK((after_establish) < (after_rebase));

    // The point of the counter: after an undo, a cache of local coordinates is
    // stale even though the frame holds a value it held before. A counter that
    // tracked the VALUE rather than the change would go backwards here, or not
    // move at all, and a cache would keep serving coordinates from the frame the
    // undo just left.
    CHECK_TRUE(doc.stack.undo());
    const uint64_t after_undo = doc.georeference.generation();
    CHECK((after_rebase) < (after_undo));

    CHECK_TRUE(doc.stack.redo());
    CHECK((after_undo) < (doc.georeference.generation()));

    const uint64_t before_load = doc.georeference.generation();
    CHECK_TRUE(doc.georeference.load(std::nullopt));
    CHECK((before_load) < (doc.georeference.generation()));

    // Even when the frame is set to a value equal to the one it already holds.
    // The counter answers "were my cached coordinates derived under the frame
    // that is current now", and a reload is a break in that chain whatever the
    // numbers are -- so a counter that compared values before bumping would sit
    // still here and leave a stale cache looking fresh.
    const GeoreferenceFrame same = GeoreferenceFrame::at(kDublin);
    CHECK_TRUE(doc.georeference.load(same));
    const uint64_t after_first_load = doc.georeference.generation();
    CHECK_TRUE(doc.georeference.load(same));
    const auto reloaded = doc.georeference.frame();
    CHECK_TRUE(reloaded.has_value());
    if (reloaded) CHECK_TRUE(*reloaded == same);
    CHECK((after_first_load) < (doc.georeference.generation()));

    // A REFUSED load moves it too, and has to: the frame it leaves behind is
    // nothing, which is not the frame any cache was measured against. This is
    // the one way the counter differs from a refused command, which leaves the
    // frame alone and therefore leaves the counter alone.
    const uint64_t before_refusal = doc.georeference.generation();
    GeoreferenceFrame broken = same;
    broken.scale = 0.0;
    CHECK_FALSE(doc.georeference.load(broken));
    CHECK_FALSE(doc.georeference.is_established());
    CHECK((before_refusal) < (doc.georeference.generation()));
}

TEST(Georeference, a_refused_command_does_not_move_the_generation) {
    Document doc;
    establish(doc);
    const uint64_t generation = doc.georeference.generation();

    CHECK_FALSE(establish_georeference(doc.stack, doc.georeference, kClontarf));
    glm::dvec2 delta{0.0, 0.0};
    CHECK_FALSE(rebase_georeference(doc.stack, doc.georeference, kDublin, delta));

    CHECK_EQ(doc.georeference.generation(), generation);
}

// ============================================================================
// Clear
// ============================================================================

TEST(Georeference, clear_returns_to_not_georeferenced_and_undo_restores_the_frame) {
    Document doc;

    // Loaded, not established, so the frame carries 5 m of drift and a scale of
    // 2.0. This is what makes the claim below checkable at all: a frame from
    // establish() has zero drift, so a revert that re-derived the Mercator
    // origin with GeoreferenceFrame::at(origin, scale) would restore a frame
    // bit-identical to the right one and nothing would fail.
    GeoreferenceFrame original = GeoreferenceFrame::at(kDublin, 2.0);
    original.origin_mercator.x += 5.0;
    CHECK_TRUE(doc.georeference.load(original));

    CHECK_TRUE(clear_georeference(doc.stack, doc.georeference));
    CHECK_FALSE(doc.georeference.is_established());
    CHECK_FALSE(doc.georeference.to_local(kDublin).has_value());

    CHECK_TRUE(doc.stack.undo());
    CHECK_TRUE(doc.georeference.is_established());
    const auto restored = doc.georeference.frame();
    CHECK_TRUE(restored.has_value());
    if (!restored) return;

    // Mercator origin included: restoring only lat and lon would re-derive it and
    // move every coordinate by the difference.
    CHECK_TRUE(*restored == original);
    CHECK_NEAR(mercator_origin_drift(*restored), 5.0, 1e-9);
    CHECK_NEAR(restored->scale, 2.0, 0.0);

    // The difference, stated as coordinates: this document's own origin converts
    // to (-10, 0) -- 5 Mercator metres at a scale of 2 -- and would convert to
    // (0, 0) if the Mercator origin had been re-derived on the way back.
    const auto at_origin = doc.georeference.to_local(kDublin);
    CHECK_TRUE(at_origin.has_value());
    if (at_origin) {
        CHECK_NEAR(at_origin->x, -10.0, 1e-6);
        CHECK_NEAR(at_origin->y, 0.0, 1e-6);
    }

    CHECK_TRUE(doc.stack.redo());
    CHECK_FALSE(doc.georeference.is_established());
}

TEST(Georeference, clearing_a_document_that_was_never_georeferenced_is_refused) {
    Document doc;
    CHECK_FALSE(clear_georeference(doc.stack, doc.georeference));
    CHECK_EQ(doc.stack.undo_depth(), size_t{0});
    CHECK_FALSE(doc.georeference.is_established());
    CHECK_EQ(doc.georeference.generation(), uint64_t{0});
}

// ============================================================================
// Save and load: what A2 needs
// ============================================================================

TEST(Georeference, a_saved_frame_restores_the_same_coordinates) {
    // A non-unit scale and 3 m of drift on purpose. In a frame straight out of
    // establish() all five doubles are derivable from two, so a loader that
    // dropped origin_mercator or the scale would round-trip perfectly and the
    // test would be checking the projection rather than the save.
    GeoreferenceFrame on_disk = GeoreferenceFrame::at(kDublin, 2.5);
    on_disk.origin_mercator.y -= 3.0;

    Georeference saved;
    CHECK_TRUE(saved.load(on_disk));
    const auto expected = saved.to_local(kLucan);
    CHECK_TRUE(expected.has_value());
    if (!expected) return;

    // A2 writes five doubles and reads them back. Reconstructed field by field,
    // the way a loader would, rather than by copying the object.
    GeoreferenceFrame restored;
    restored.origin_lat = on_disk.origin_lat;
    restored.origin_lon = on_disk.origin_lon;
    restored.origin_mercator = on_disk.origin_mercator;
    restored.scale = on_disk.scale;

    Georeference loaded;
    CHECK_TRUE(loaded.load(restored));

    CHECK_TRUE(loaded.is_established());
    const auto frame = loaded.frame();
    CHECK_TRUE(frame.has_value());
    if (!frame) return;
    CHECK_TRUE(*frame == on_disk);
    CHECK_NEAR(mercator_origin_drift(*frame), 3.0, 1e-9);
    CHECK_NEAR(frame->scale, 2.5, 0.0);

    const auto actual = loaded.to_local(kLucan);
    CHECK_TRUE(actual.has_value());
    if (actual) {
        CHECK_NEAR(actual->x, expected->x, 0.0);
        CHECK_NEAR(actual->y, expected->y, 0.0);
    }
}

TEST(Georeference, load_keeps_the_stored_mercator_origin_instead_of_recomputing_it) {
    GeoreferenceFrame frame = GeoreferenceFrame::at(kDublin);
    CHECK_NEAR(mercator_origin_drift(frame), 0.0, 1e-9);

    // Stand in for a projection that has changed under a saved document: the
    // stored origin is 5 m from where this build's maths would put it.
    frame.origin_mercator.x += 5.0;

    Georeference loaded;
    CHECK_TRUE(loaded.load(frame));

    const auto stored = loaded.frame();
    CHECK_TRUE(stored.has_value());
    if (!stored) return;
    // Drift is not invalidity: 5 m of disagreement is warned about and kept,
    // because it is the frame the document's coordinates were built against.
    CHECK_NEAR(mercator_origin_drift(*stored), 5.0, 1e-9);

    // Recomputing on load would silently move every vertex in the document by
    // those 5 m relative to the frame. Keeping the stored value means the
    // document's own origin still converts to (-5, 0), which is the truth about
    // that file and is reported rather than hidden.
    const auto at_origin = loaded.to_local(kDublin);
    CHECK_TRUE(at_origin.has_value());
    if (at_origin) {
        CHECK_NEAR(at_origin->x, -5.0, 1e-6);
        CHECK_NEAR(at_origin->y, 0.0, 1e-6);
    }
}

TEST(Georeference, loading_writes_no_undo_history) {
    Document doc;
    CHECK_TRUE(doc.georeference.load(GeoreferenceFrame::at(kDublin)));

    // Opening a file is not an edit. An undo stack that starts with "open the
    // document" offers the user a step that leaves geometry with no frame.
    CHECK_EQ(doc.stack.undo_depth(), size_t{0});
    CHECK_FALSE(doc.stack.can_undo());
    CHECK_TRUE(doc.georeference.is_established());

    // And loading nothing is how an ungeoreferenced document is opened.
    CHECK_TRUE(doc.georeference.load(std::nullopt));
    CHECK_FALSE(doc.georeference.is_established());
    CHECK_EQ(doc.stack.undo_depth(), size_t{0});
}

TEST(Georeference, load_refuses_a_frame_whose_scale_would_collapse_the_document) {
    GeoreferenceFrame broken = GeoreferenceFrame::at(kDublin);
    broken.scale = 0.0;

    Georeference loaded;
    const uint64_t before = loaded.generation();

    // Installed with an error logged beside it -- which is what happened -- this
    // frame leaves is_established() true, to_local() returning (0, 0) for every
    // point in the document, to_wgs84() returning NaN and
    // ground_metres_per_unit() infinite. A document that reads as georeferenced
    // and converts to nonsense is the exact failure this file exists to refuse,
    // and load() is the one path that does not run the commands' validation.
    CHECK_FALSE(loaded.load(broken));
    CHECK_FALSE(loaded.is_established());
    CHECK_FALSE(loaded.frame().has_value());
    CHECK_FALSE(loaded.to_local(kClontarf).has_value());
    CHECK_FALSE(loaded.to_wgs84(glm::dvec2{100.0, 100.0}).has_value());
    CHECK_FALSE(loaded.ground_metres_per_unit().has_value());
    CHECK_FALSE(loaded.coordinate_system().has_value());

    // The frame was replaced -- by nothing -- so a cache measured against the
    // old one is stale and the counter has to say so.
    CHECK((before) < (loaded.generation()));

    // Every scale the command stack refuses, load() refuses the same way. The
    // two have to agree: a scale reachable through load() but not through
    // establish() is a document no dialog could have produced.
    for (const double scale : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN()}) {
        GeoreferenceFrame frame = GeoreferenceFrame::at(kDublin);
        frame.scale = scale;
        CHECK_FALSE(is_valid_frame(frame));

        Georeference target;
        CHECK_FALSE(target.load(frame));
        CHECK_FALSE(target.is_established());

        Document doc;
        CHECK_FALSE(establish_georeference(doc.stack, doc.georeference, kDublin, scale));
    }
}

TEST(Georeference, load_refuses_a_frame_whose_origin_is_not_a_place) {
    const double nan = std::numeric_limits<double>::quiet_NaN();

    // A NaN origin left is_established() true, ground_metres_per_unit() NaN and
    // to_local() still answering -- off origin_mercator -- so the document went
    // on converting, and every answer was measured from nowhere.
    GeoreferenceFrame nan_origin = GeoreferenceFrame::at(kDublin);
    nan_origin.origin_lat = nan;
    Georeference from_nan_origin;
    CHECK_FALSE(from_nan_origin.load(nan_origin));
    CHECK_FALSE(from_nan_origin.is_established());

    // Outside the Mercator band. establish() refuses this origin rather than
    // letting the projection clamp it 435 km south, and load() must agree.
    GeoreferenceFrame out_of_band = GeoreferenceFrame::at(kDublin);
    out_of_band.origin_lat = 89.0;
    Georeference from_out_of_band;
    CHECK_FALSE(from_out_of_band.load(out_of_band));
    CHECK_FALSE(from_out_of_band.is_established());

    // An unwrapped longitude, which projects to a believable easting that is not
    // on the planet.
    GeoreferenceFrame unwrapped = GeoreferenceFrame::at(kDublin);
    unwrapped.origin_lon = 200.0;
    Georeference from_unwrapped_lon;
    CHECK_FALSE(from_unwrapped_lon.load(unwrapped));
    CHECK_FALSE(from_unwrapped_lon.is_established());

    // And the field is_valid_origin() cannot speak for, because it is stored
    // rather than derived and nothing downstream recomputes it: a non-finite
    // Mercator origin makes every conversion NaN while the lat and lon beside it
    // read as a real place.
    GeoreferenceFrame nan_mercator = GeoreferenceFrame::at(kDublin);
    nan_mercator.origin_mercator.y = nan;
    CHECK_TRUE(is_valid_origin(nan_mercator.origin()));
    CHECK_FALSE(is_valid_frame(nan_mercator));
    Georeference from_nan_mercator;
    CHECK_FALSE(from_nan_mercator.load(nan_mercator));
    CHECK_FALSE(from_nan_mercator.is_established());

    // The control: the same origin, untouched, loads. Without this the checks
    // above would pass for a load() that refused everything.
    Georeference good;
    CHECK_TRUE(is_valid_frame(GeoreferenceFrame::at(kDublin)));
    CHECK_TRUE(good.load(GeoreferenceFrame::at(kDublin)));
    CHECK_TRUE(good.is_established());
    CHECK_TRUE(good.to_local(kClontarf).has_value());
}

TEST(Georeference, a_refused_load_clears_the_frame_rather_than_keeping_the_old_one) {
    Document doc;
    establish(doc, kDublin);
    // What a loader does either way: the history was measured against the frame
    // being replaced. See the warning on Georeference::load().
    doc.stack.clear();

    GeoreferenceFrame broken = GeoreferenceFrame::at(kLucan, 2.0);
    broken.scale = -2.0;
    CHECK_FALSE(doc.georeference.load(broken));

    // NOT "the Dublin frame survives". load() names the whole state of the
    // document being opened, so keeping the previous document's frame would
    // interpret the new document's coordinates against it -- and every number
    // would look reasonable. Refusing the frame means refusing the document's
    // georeference, which is a state every caller already handles.
    CHECK_FALSE(doc.georeference.is_established());
    CHECK_FALSE(doc.georeference.to_local(kDublin).has_value());
    CHECK_FALSE(doc.georeference.origin().has_value());
}

// ============================================================================
// The undo menu, and the stack's memory bound
// ============================================================================

TEST(Georeference, each_command_names_itself_for_the_undo_menu) {
    Document doc;
    establish(doc);
    // These strings are pasted straight into the menu by CommandStack, so an
    // empty describe() gives the user an undo entry with no name. Nothing else
    // in the suite calls undo_label() or redo_label() at all.
    CHECK_EQ(doc.stack.undo_label(), std::string{"Establish georeference"});

    glm::dvec2 delta{0.0, 0.0};
    CHECK_TRUE(rebase_georeference(doc.stack, doc.georeference, kClontarf, delta));
    CHECK_EQ(doc.stack.undo_label(), std::string{"Move georeference origin"});

    // Distinct from the establish label, because "the origin moved" and "the
    // origin was chosen" undo to different documents.
    CHECK_TRUE(doc.stack.undo_label() != doc.stack.redo_label());

    CHECK_TRUE(doc.stack.undo());
    CHECK_EQ(doc.stack.redo_label(), std::string{"Move georeference origin"});
    CHECK_EQ(doc.stack.undo_label(), std::string{"Establish georeference"});

    CHECK_TRUE(doc.stack.redo());
    CHECK_TRUE(clear_georeference(doc.stack, doc.georeference));
    CHECK_EQ(doc.stack.undo_label(), std::string{"Clear georeference"});

    // Present tense and no leading "Undo", which is command.hpp's contract for
    // describe(): the menu supplies the verb.
    CHECK_FALSE(doc.stack.undo_label().empty());
}

TEST(Georeference, a_recorded_command_reports_its_own_size_to_the_stacks_memory_bound) {
    // CommandStack::bytes() is the sum of footprint() over the steps it holds,
    // and Command::footprint() defaults to sizeof(Command) -- 8 bytes, the vtable
    // pointer alone. Left at the default these commands under-report by 8x to
    // 15x, and CommandStackConfig::max_bytes bounds a number with no relation to
    // what the stack is holding.
    Document doc;
    CHECK_EQ(doc.stack.bytes(), size_t{0});

    establish(doc);
    CHECK_EQ(doc.stack.bytes(), sizeof(EstablishGeoreferenceCommand));

    glm::dvec2 delta{0.0, 0.0};
    CHECK_TRUE(rebase_georeference(doc.stack, doc.georeference, kClontarf, delta));
    CHECK_EQ(doc.stack.bytes(),
             sizeof(EstablishGeoreferenceCommand) + sizeof(RebaseGeoreferenceCommand));

    CHECK_TRUE(clear_georeference(doc.stack, doc.georeference));
    CHECK_EQ(doc.stack.bytes(),
             sizeof(EstablishGeoreferenceCommand) + sizeof(RebaseGeoreferenceCommand) +
                 sizeof(ClearGeoreferenceCommand));

    // And the figure is the object, not the vtable pointer it would have been.
    CHECK((sizeof(EstablishGeoreferenceCommand)) > (size_t{8}));
}

// ============================================================================
// Agreement with the OSM chain
// ============================================================================

TEST(Georeference, a_unit_scale_frame_converts_exactly_as_the_osm_converter_does) {
    Document doc;
    const GeoreferenceFrame frame = establish(doc, kDublin);
    CHECK_TRUE(frame.osm_converter_compatible());

    CoordinateConverter converter;
    converter.set_origin(kDublin.lat, kDublin.lon);

    for (const GeoPoint p : {kDublin, kClontarf, kLucan}) {
        const auto ours = doc.georeference.to_local(p);
        const glm::dvec2 theirs = converter.wgs84_to_local(p.lat, p.lon);
        CHECK_TRUE(ours.has_value());
        if (!ours) return;
        // Bit for bit. Anything looser would let a different projection through,
        // and the two have to agree or imported geometry and scene geometry sit
        // in different frames.
        CHECK_NEAR(ours->x, theirs.x, 0.0);
        CHECK_NEAR(ours->y, theirs.y, 0.0);
    }
}

TEST(Georeference, a_scaled_frame_does_not_agree_with_the_osm_converter) {
    Document doc;
    const GeoreferenceFrame frame = establish(doc, kDublin, 4.0);

    // CoordinateSystem::scale exists and CoordinateConverter ignores it. Naming
    // that disagreement is better than every call site discovering it.
    CHECK_FALSE(frame.osm_converter_compatible());

    CoordinateConverter converter;
    converter.set_origin(kDublin.lat, kDublin.lon);

    const auto ours = doc.georeference.to_local(kClontarf);
    const glm::dvec2 theirs = converter.wgs84_to_local(kClontarf.lat, kClontarf.lon);
    CHECK_TRUE(ours.has_value());
    if (!ours) return;

    CHECK_NEAR(ours->x, theirs.x * 4.0, 1e-6);
    CHECK_NEAR(ours->y, theirs.y * 4.0, 1e-6);
    CHECK((glm::length(*ours - theirs)) > (1000.0));

    // The frame is still self-consistent: it round trips against itself.
    const auto back = doc.georeference.to_wgs84(*ours);
    if (back) {
        CHECK_NEAR(back->lat, kClontarf.lat, 1e-9);
        CHECK_NEAR(back->lon, kClontarf.lon, 1e-9);
    }

    // And the ground factor accounts for the scale as well as the latitude.
    const auto factor = doc.georeference.ground_metres_per_unit();
    if (factor) CHECK_NEAR(*factor, kDublinGroundPerUnit / 4.0, 1e-9);
}

TEST(Georeference, a_frame_round_trips_through_the_osm_coordinate_system) {
    // Drift injected and a scale that is not 1.0, because both of
    // from_coordinate_system()'s documented guarantees are invisible against a
    // frame from at(kDublin, 1.0): a re-derived Mercator origin is bit-identical
    // to the stored one, and a hardcoded scale of 1.0 is the value being
    // compared against.
    GeoreferenceFrame frame = GeoreferenceFrame::at(kDublin, 2.5);
    frame.origin_mercator.y += 7.0;

    const auto system = frame.to_coordinate_system();

    // origin_latlon is (lat, lon), the same packing BoundingBox::center() uses.
    CHECK_NEAR(system.origin_latlon.x, kDublin.lat, 0.0);
    CHECK_NEAR(system.origin_latlon.y, kDublin.lon, 0.0);
    CHECK_NEAR(system.origin_mercator.x, frame.origin_mercator.x, 0.0);
    CHECK_NEAR(system.origin_mercator.y, frame.origin_mercator.y, 0.0);
    CHECK_NEAR(system.scale, 2.5, 0.0);

    const GeoreferenceFrame back = GeoreferenceFrame::from_coordinate_system(system);
    CHECK_TRUE(back == frame);
    // Verbatim, not re-derived: the 7 m survives the round trip, which is the
    // whole reason origin_mercator is carried rather than recomputed from the
    // lat/lon beside it.
    CHECK_NEAR(mercator_origin_drift(back), 7.0, 1e-9);
    CHECK_NEAR(back.scale, 2.5, 0.0);

    // A converter seeded from the frame agrees with the frame, which is what
    // makes this the supported way to hand a document's frame to the importer.
    // At unit scale, because CoordinateConverter ignores CoordinateSystem::scale.
    const GeoreferenceFrame unit = GeoreferenceFrame::at(kDublin, 1.0);
    const auto unit_system = unit.to_coordinate_system();
    CoordinateConverter converter;
    converter.set_origin(unit_system.origin_latlon.x, unit_system.origin_latlon.y);
    const glm::dvec2 theirs = converter.wgs84_to_local(kLucan.lat, kLucan.lon);
    const glm::dvec2 ours = unit.to_local(kLucan);
    CHECK_NEAR(ours.x, theirs.x, 0.0);
    CHECK_NEAR(ours.y, theirs.y, 0.0);
}

TEST(Georeference, coordinate_system_hands_over_the_frame_the_document_holds) {
    // The one path by which the frame leaves the scene layer, and the only test
    // that had touched it asked has_value() and nothing else. A body returning a
    // default CoordinateSystem anchors the importer at Null Island; one that
    // swapped lat and lon anchors it in Kazakhstan. Both are frames the importer
    // would build a whole city against without complaint.
    Document doc;
    const GeoreferenceFrame frame = establish(doc, kDublin, 3.0);

    const auto system = doc.georeference.coordinate_system();
    CHECK_TRUE(system.has_value());
    if (!system) return;

    CHECK_NEAR(system->origin_latlon.x, kDublin.lat, 0.0);
    CHECK_NEAR(system->origin_latlon.y, kDublin.lon, 0.0);
    CHECK_NEAR(system->origin_mercator.x, frame.origin_mercator.x, 0.0);
    CHECK_NEAR(system->origin_mercator.y, frame.origin_mercator.y, 0.0);
    // Non-unit, so the field is not being compared against its own default.
    CHECK_NEAR(system->scale, 3.0, 0.0);

    // Spelled out as places rather than as numbers: the origin is north of the
    // equator and west of Greenwich, and it is not (0, 0).
    CHECK((system->origin_latlon.x) > (0.0));
    CHECK((system->origin_latlon.y) < (0.0));
    CHECK((std::fabs(system->origin_mercator.y)) > (1.0e6));

    // And a converter seeded from it puts a point exactly where the document
    // does, which is the only reason the function exists. Unit scale for that
    // comparison, because the OSM converter ignores the scale field.
    Document unit;
    establish(unit, kDublin, 1.0);
    const auto unit_system = unit.georeference.coordinate_system();
    CHECK_TRUE(unit_system.has_value());
    if (!unit_system) return;

    CoordinateConverter converter;
    converter.set_origin(unit_system->origin_latlon.x, unit_system->origin_latlon.y);
    for (const GeoPoint p : {kDublin, kClontarf, kLucan}) {
        const glm::dvec2 theirs = converter.wgs84_to_local(p.lat, p.lon);
        const auto ours = unit.georeference.to_local(p);
        CHECK_TRUE(ours.has_value());
        if (!ours) return;
        CHECK_NEAR(ours->x, theirs.x, 0.0);
        CHECK_NEAR(ours->y, theirs.y, 0.0);
    }
}
