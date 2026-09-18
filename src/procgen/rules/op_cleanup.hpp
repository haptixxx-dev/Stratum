// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_cleanup.hpp
 * @brief `cleanup(tolerance)` -- make a real-world footprint usable
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS IS FOR
 * ================================================================================
 *
 * Synthetic footprints are clean. Surveyed ones are not: a hand-traced building
 * outline routinely repeats a vertex, carries a spur that doubles back on
 * itself, or puts two points a millimetre apart. None of that is visible in a
 * viewport and none of it stops a wall being built.
 *
 * It stops a ROOF being built. `vertex_offset_velocity()` -- the straight
 * skeleton's corner solver, which `roof()` uses for every kind but shed --
 * needs each corner to have a mitre direction, and a corner has none when
 * either of its edges is zero length or when the two edges are exactly
 * opposite. It returns false, `roof()` refuses, and the building comes out with
 * walls and no roof.
 *
 * Measured over the Lucan extract: 53 of 400 real footprints, over an eighth,
 * with the message "corner 0 of the outline is a spike with no mitre". That was
 * issue #133, and this is the operation the diagnostic was waiting for:
 *
 *     rule Cap { cleanup(); align_scope("y_up"); roof("hip", 35.0); }
 *
 * ================================================================================
 * WHAT IT REMOVES, AND WHAT IT REFUSES TO
 * ================================================================================
 *
 * Three things, in one pass per ring, repeated until the ring stops changing --
 * because removing one spur can expose another behind it:
 *
 *   1. **Points closer together than the tolerance.** A zero-length edge has no
 *      direction, so no normal, so no corner.
 *   2. **Spurs.** A vertex whose incoming and outgoing directions are opposite
 *      is the tip of a zero-width spike. It encloses no area, so removing it
 *      changes the footprint by nothing at all -- which is why this is a repair
 *      rather than a simplification.
 *   3. **Collinear vertices.** A point sitting on the straight line between its
 *      neighbours carries no shape. It is harmless to the skeleton, and dropping
 *      it makes every later operation cheaper.
 *
 * It does NOT simplify. There is no Douglas-Peucker here and no tolerance that
 * moves a vertex: every point that survives is exactly where it was. A rule that
 * wants fewer vertices at the cost of shape wants `reduce()`, which is a
 * different row in the catalogue and a different promise.
 *
 * A ring left with fewer than three points is DROPPED. A face whose outer ring
 * is dropped goes with it; a hole that collapses is simply gone, which is the
 * right answer for a courtyard that was only ever a survey artefact.
 *
 * If cleaning removes every face, the operation FAILS rather than leaving an
 * empty shape, because a rule that cleans a footprint and silently gets nothing
 * has lost a building with no way to know.
 *
 * ================================================================================
 * THE TOLERANCE
 * ================================================================================
 *
 * Default 1e-3, one millimetre, in whatever units the shape is in -- metres,
 * for anything that came through the OSM importer. That is deliberately far
 * below anything a survey means and far above the 1e-9 at which coordinates
 * stop being distinguishable, so the default repairs genuine duplicates and
 * touches nothing an author drew on purpose.
 *
 * The angle tests are NOT scaled by it. A spur is a spur whether it is a
 * micrometre or ten metres long, and a tolerance that made a long spur survive
 * would leave exactly the corners the roof cannot solve.
 */

#pragma once

#include "procgen/rules/interpreter.hpp"

namespace stratum::procgen::rules {

/// Register `cleanup` into @p table
void register_cleanup_operations(OperationTable& table);

/// standard_operations() plus register_cleanup_operations()
[[nodiscard]] const OperationTable& cleanup_operations();

/**
 * @brief What one cleanup did, for a test and for a diagnostic
 */
struct CleanupReport {
    uint32_t merged_points = 0;      ///< Removed for being within the tolerance of a neighbour
    uint32_t removed_spurs = 0;      ///< Removed for doubling back
    uint32_t removed_collinear = 0;  ///< Removed for sitting on a straight line
    uint32_t dropped_rings = 0;      ///< Rings left with fewer than three points
    uint32_t dropped_faces = 0;      ///< Faces whose outer ring was dropped

    [[nodiscard]] uint32_t total_removed() const {
        return merged_points + removed_spurs + removed_collinear;
    }
    [[nodiscard]] bool changed_anything() const {
        return total_removed() > 0 || dropped_rings > 0 || dropped_faces > 0;
    }
};

/**
 * @brief Clean every ring of @p shape in place
 *
 * @param shape     Shape to clean, modified on success
 * @param tolerance Points closer than this merge. Must be positive.
 * @param report    Receives what happened. Cleared first.
 * @return failure when the tolerance is not positive, or when cleaning would
 *         leave the shape with no faces at all
 */
[[nodiscard]] OpResult cleanup_shape(Shape& shape, double tolerance, CleanupReport& report);

} // namespace stratum::procgen::rules
