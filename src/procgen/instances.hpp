// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file instances.hpp
 * @brief F2: where an asset goes, as data -- a transform list per asset, never merged geometry
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHY THIS FILE EXISTS AT ALL
 * ================================================================================
 *
 * CityEngine's `i()` inserts an asset and then MERGES it into the building
 * geometry, grouped by material. After a generate there is no bench any more:
 * there are triangles that used to be a bench, ten thousand copies of them, and
 * an exporter has no way to recover the fact that they were one mesh placed ten
 * thousand times.
 *
 * A game engine wants exactly the opposite. One mesh for the window frame, one
 * draw call, ten thousand transforms in an instance buffer. Unity, Unreal and
 * every renderer in between are built around that, and it is the difference
 * between a city that runs and a city that is a hundred million unique
 * triangles. docs/plans/cityengine_parity.md F2 puts it in one line -- "Engines
 * want instances; CE merges and loses them" -- and this file is the answer.
 *
 * So the placement is kept as DATA. Nothing here builds a triangle, nothing
 * here loads a file, and `GenerationResult::build_mesh()` never sees an inserted
 * asset. The mesh a rule produces and the instances it places are two outputs,
 * and a consumer that wants the building takes both.
 *
 * ================================================================================
 * AN ASSET IS A PATH, AND THAT IS THE WHOLE LAYER BOUNDARY
 * ================================================================================
 *
 * An asset is identified here by a STRING. Not a handle, not a pointer, not a
 * loaded mesh. Resolving that string to geometry is F1 (#56), the asset library,
 * and F1 is editor-side: it imports `.glb`, `.fbx` and `.obj` through assimp,
 * which links into `stratum_editor_lib` and must never link into
 * `stratum_core`.
 *
 * That split is deliberate and it is what makes this file useful headless. A
 * batch job on a build machine can generate a city, write every placement to a
 * file and never open a window, because placement data has no dependency on
 * anything that can draw. The moment this struct held a mesh, `stratum_core`
 * would need a model importer and the split would be gone.
 *
 * The consequence a reader should expect: NOTHING here validates that the path
 * names a file that exists. An asset that F1 cannot resolve is F1's diagnostic
 * to report, at import time, against the whole set -- not a per-placement error
 * raised in the middle of generating four thousand lots.
 *
 * ================================================================================
 * THE TRANSFORM, AND THE TWO WAYS TO GET IT WRONG
 * ================================================================================
 *
 * A placement is a scope: an origin, three orthonormal axes and a size. That is
 * already a transform, and InstanceTransform is those three fields and nothing
 * else. The two questions a reader will have are the two that put a bench
 * through a wall, so both are answered here rather than left to be discovered.
 *
 * ### 1. What lands on the origin? (the pivot)
 *
 * The transform maps the UNIT CUBE `[0,1]^3` onto the scope box:
 *
 *     world = origin + axes * (unit * size)
 *
 * so unit (0,0,0) lands on `origin` and unit (1,1,1) lands on the opposite
 * corner. `Scope::origin` is the world position of scope-local (0,0,0), which by
 * shape.hpp's second invariant is the box MINIMUM corner -- so the asset's own
 * bounding-box minimum corner goes to the scope's minimum corner, and the asset
 * is fitted to the box.
 *
 * The consumer's half of the contract is therefore: normalise the asset by its
 * own bounding box into `[0,1]^3` first, then apply to_matrix(). That is the
 * only split that lets core hold the placement without knowing the asset's
 * size. It is also CityEngine's `i()` semantics, so a rule written against CGA
 * lands where its author expects.
 *
 * `Scope::pivot` is deliberately NOT carried. It is the fraction that `rotate`
 * and `scale` turn about while a rule is being written; it says nothing about
 * where the asset sits once the rule has finished. Carrying it would offer a
 * consumer a second, contradictory answer to "what is the origin", and the two
 * would be wrong in different places.
 *
 * ### 2. Which way round is it? (the handedness)
 *
 * `axes` holds the local x, y and z axes as COLUMNS, in world space, exactly as
 * `Scope::axes` does. It is orthonormal but NOT necessarily right-handed:
 * `mirror_scope` negates one axis on purpose, and a facade that mirrors its
 * left half onto its right is the ordinary way to write one.
 *
 * A left-handed frame is a negative determinant, which to_matrix() carries
 * through as a negative scale. An exporter that ignores it writes an asset that
 * renders inside out -- back faces to the street -- and the symptom is a window
 * that is invisible from outside and solid from inside. right_handed() exists so
 * that a consumer can ask rather than guess, and dump() prints the axes in full
 * so a test can see it.
 *
 * ### A flat or point-sized scope is not an error here
 *
 * A facade tile has `size.z == 0`: it is a rectangle on a wall, and inserting a
 * window frame into it is THE case this feature was built for. So a zero extent
 * is recorded, not refused. flat_axes() and is_point() let a consumer decide
 * what to do -- fit the asset flat, or fall back to its native size and use the
 * origin and axes alone. op_insert.hpp owns the question of when that is worth
 * a diagnostic.
 *
 * ### There is no quaternion, and no translate-rotate-scale triple
 *
 * An exporter wants a TRS, and decomposing `axes` into a rotation would be four
 * lines. It is left to the consumer anyway, because extracting a quaternion
 * needs a square root and an inverse cosine of a value derived from a
 * COORDINATE, and the last bit of those is not guaranteed to agree across C
 * libraries. shape.hpp names rotation as the ONE place libm may touch a
 * coordinate in the rule layer, and this file is not going to be the second.
 *
 * `axes` and to_matrix() carry everything a decomposition would, exactly, and
 * the consumer that needs a quaternion is already outside the determinism
 * contract -- it is writing a file for an engine that stores floats.
 *
 * ================================================================================
 * DETERMINISM
 * ================================================================================
 *
 * Same rule text, same seed, byte-identical placements in byte-identical order.
 * Two rules, the same two the rest of the tree keeps:
 *
 *   - **std::map, never a hash container.** Assets iterate in path order, which
 *     is a property of the paths and not of an allocator. `std::unordered_map`
 *     would give a consumer a different asset order on a different libstdc++,
 *     and an exporter that writes assets in iteration order would produce a
 *     different file for the same city.
 *   - **Within one asset, the order is the order the placements were ADDED**,
 *     which for a generation is the order the evaluator reached them, which is
 *     source order. Never sorted by position: a sort key built from a double
 *     coordinate re-orders two placements that differ in the last bit, and a
 *     tie-break that falls back to insertion order makes the sort pointless.
 *
 * Nothing in this file calls libm on a coordinate. A transform is copied out of
 * a scope and copied into a buffer; the arithmetic is the +, - and * of
 * to_matrix() and unit_to_world(), which IEEE-754 rounds identically everywhere.
 *
 * dump() is the observable form of all of that, and it prints every number of
 * every placement. A determinism test written against a dump that summarised --
 * counts, or a bounding box -- would pass for an implementation that placed
 * every bench at the origin, which is the exact defect this project has found
 * nine times.
 */

#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace stratum::procgen {

// ============================================================================
// One placement
// ============================================================================

/**
 * @brief Where one copy of one asset goes
 *
 * Doubles, matching the rule layer end to end. `Scope` is double, the whole
 * operation chain that produced it is double, and a city at real metres loses
 * millimetres in float long before it reaches an exporter. The conversion to
 * float, if a consumer wants one, happens at the consumer -- the same place
 * shape_to_mesh() does it and for the same reason.
 *
 * Copied out of a `Scope` by op_insert.hpp's instance_transform(). It is a plain
 * aggregate on purpose: it is written to a file, sent to an instance buffer and
 * compared in a test, and none of those want an invariant to maintain.
 */
struct InstanceTransform {
    /// World position of the asset's bounding-box minimum corner. See the header.
    glm::dvec3 origin{0.0};

    /**
     * @brief Local x, y and z as COLUMNS, in world space
     *
     * Orthonormal. Right-handed unless a `mirror_scope` made it otherwise, which
     * right_handed() reports.
     */
    glm::dmat3 axes{1.0};

    /// Extent along each local axis, in metres. Never negative. May be zero.
    glm::dvec3 size{0.0};

    /**
     * @brief Map a point of the unit cube into world space
     *
     * `unit` (0,0,0) is the box minimum, (1,1,1) the maximum, (0.5,0.5,0.5) the
     * centre. A consumer normalises the asset into that cube and calls this, or
     * uses to_matrix() to do it on the GPU.
     */
    [[nodiscard]] glm::dvec3 unit_to_world(const glm::dvec3& unit) const {
        return origin + axes * (unit * size);
    }

    /// World position of the centre of the placement box
    [[nodiscard]] glm::dvec3 center() const { return unit_to_world(glm::dvec3{0.5}); }

    /**
     * @brief The unit cube to world space, as a 4x4
     *
     * Column k is `axes[k] * size[k]`, and the translation is `origin`. So the
     * matrix carries the rotation, the non-uniform scale and the handedness in
     * one object, which is what an exporter and an instance buffer both want.
     *
     * A zero extent gives a zero column, and therefore a singular matrix. That
     * is the honest answer for a facade tile with no depth; see flat_axes().
     */
    [[nodiscard]] glm::dmat4 to_matrix() const;

    /**
     * @brief Is the frame right-handed?
     *
     * The determinant of @p axes, which for an orthonormal matrix is +1 or -1.
     * False means a `mirror_scope` is in the chain and the asset must be drawn
     * with a negative scale, or it renders inside out.
     *
     * The comparison is STRICT, so a degenerate axis set -- which the orthonormal
     * invariant excludes, but which a hand-built Shape carries straight through
     * op_insert's finite check -- answers false rather than true. "Right-handed"
     * is never claimed for a frame that has no handedness: an exporter told that
     * writes a collapsed instance with no sign that anything was wrong.
     */
    [[nodiscard]] bool right_handed() const;

    /// How many of the three extents are exactly zero: 0, 1, 2 or 3
    [[nodiscard]] int flat_axes() const;

    /// Every extent is zero, so the scope carries an orientation and no size
    [[nodiscard]] bool is_point() const { return flat_axes() == 3; }

    /**
     * @brief Bit-exact equality
     *
     * Exact, not within a tolerance, because the thing this type promises is
     * that two runs agree bit for bit. A near-equality here would make the
     * determinism tests unable to fail.
     */
    [[nodiscard]] bool operator==(const InstanceTransform& other) const;
    [[nodiscard]] bool operator!=(const InstanceTransform& other) const {
        return !(*this == other);
    }
};

// ============================================================================
// Bounds
// ============================================================================

/**
 * @brief World-space axis-aligned bounds of a set of placements
 *
 * Its own type rather than `renderer/mesh.hpp`'s BoundingBox3D, which is float:
 * a float box around a city at metre scale quantises to roughly a centimetre at
 * the far corner, and this box is the input to culling and to an export offset
 * where that shows.
 *
 * Empty by construction and stays empty until something is added, which
 * `valid()` reports. A box that defaulted to (0,0,0)-(0,0,0) would silently
 * include the origin in the bounds of an empty set.
 */
struct InstanceBounds {
    glm::dvec3 min{0.0};
    glm::dvec3 max{0.0};
    bool populated = false;

    [[nodiscard]] bool valid() const { return populated; }

    void expand(const glm::dvec3& point);

    /// Grow to contain all eight corners of @p transform's box
    void expand(const InstanceTransform& transform);

    /// (0,0,0) when empty
    [[nodiscard]] glm::dvec3 center() const {
        return populated ? (min + max) * 0.5 : glm::dvec3{0.0};
    }

    /// (0,0,0) when empty
    [[nodiscard]] glm::dvec3 extent() const {
        return populated ? (max - min) : glm::dvec3{0.0};
    }
};

// ============================================================================
// The set
// ============================================================================

/**
 * @brief Every placement of every asset, grouped by asset
 *
 * The shape a consumer wants: one entry per distinct asset path, holding every
 * transform for it. That is one mesh and one instance buffer per entry, which is
 * exactly one draw call per entry.
 *
 * Grouping happens on the way IN rather than in a pass afterwards, so two
 * different rules that insert the same path land in the same list without either
 * rule knowing about the other. That is the second case the brief names, and it
 * needs no special handling here: `add()` is keyed on the path.
 */
class InstanceSet {
public:
    /// The stored form: asset path to its transforms, in path order
    using Storage = std::map<std::string, std::vector<InstanceTransform>, std::less<>>;

    /**
     * @brief Record one placement
     *
     * Appends to @p asset's list, creating it when this is the first placement
     * of that path. An empty path is REFUSED and reported, rather than creating
     * an anonymous bucket that a consumer would have to guess the meaning of;
     * op_insert.cpp rejects it one layer earlier, with a source location, and
     * this is the backstop for a caller that did not.
     *
     * @return false when @p asset is empty; nothing is stored
     */
    bool add(std::string_view asset, const InstanceTransform& transform);

    /// Every asset and its transforms, in path order
    [[nodiscard]] const Storage& assets() const { return assets_; }

    /// Number of distinct asset paths
    [[nodiscard]] size_t asset_count() const { return assets_.size(); }

    /// Number of placements across every asset
    [[nodiscard]] size_t placement_count() const;

    /// @return The transforms for @p asset, or nullptr when it has none
    [[nodiscard]] const std::vector<InstanceTransform>* find(std::string_view asset) const;

    /// Number of placements of @p asset, zero when it has none
    [[nodiscard]] size_t count_of(std::string_view asset) const;

    /// World bounds of every placement box. Not valid() when the set is empty.
    [[nodiscard]] InstanceBounds bounds() const;

    /// World bounds of one asset's placements. Not valid() when it has none.
    [[nodiscard]] InstanceBounds bounds_of(std::string_view asset) const;

    /// Take every placement of @p other, keeping each asset's relative order
    void merge(const InstanceSet& other);

    void clear() { assets_.clear(); }

    [[nodiscard]] bool empty() const { return assets_.empty(); }

    /**
     * @brief A canonical text rendering, for a test and for a dump
     *
     * Deterministic and platform-independent: every number goes through
     * rules::format_number(), assets iterate in the map's key order, and no
     * address or iteration accident reaches the output.
     *
     * It prints the origin, all three axes and the size of EVERY placement, not
     * a summary. See the determinism note in the header: a dump that summarised
     * could not tell a working implementation from one that placed everything at
     * the origin, and a determinism test resting on it could not fail.
     */
    [[nodiscard]] std::string dump() const;

private:
    Storage assets_;
};

} // namespace stratum::procgen
