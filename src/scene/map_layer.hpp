// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file map_layer.hpp
 * @brief Typed map layers: what drives procedural behaviour from a raster
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### What a map layer is for
 *
 * CityEngine drives generation from images. An obstacle layer says where a
 * street may not grow, a water layer says where the ground is wet, a texture
 * layer is a basemap to drape, and a generic scalar layer is any per-location
 * number a rule wants to read. B8 (street growth), C6 (zoning) and F5
 * (vegetation scatter) each need exactly one question answered:
 *
 *     "what is the value of this layer at this world position?"
 *
 * Everything in this file exists to answer that question cheaply, and to make
 * the three ways it can fail to have an answer -- no such layer, a layer with
 * no data, a position outside the layer -- impossible to confuse with a real
 * answer of zero.
 *
 * ### How this relates to A1's layer tree
 *
 * A map layer IS a Layer -- specifically a `LayerKind::Map` one -- and the
 * pixels live in a side table keyed by its LayerId. It is not a Layer subclass
 * and it is not a parallel tree. Three reasons, in order of weight:
 *
 *   - **`Layer` is copied wholesale by the history.** DeleteLayerCommand keeps
 *     `std::vector<Layer>` of the whole subtree as its undo state, and
 *     LayerTree::detach() returns layers by value. A 64 MB basemap inside
 *     `Layer` would be copied by every structural edit that passes near it,
 *     and `footprint()` -- the thing that bounds the undo stack -- would start
 *     reporting hundreds of megabytes per deleted group.
 *   - **A3 already set the precedent.** AttributeStore keys layer-scope values
 *     by layer id in a table of its own rather than widening `Layer`. A second
 *     payload that invents a second mechanism would leave the codebase with
 *     two answers to "where does a layer's extra data live".
 *   - **The tree stays serialisable as plain data.** A2 writes `Layer` as it
 *     stands; pixels want their own chunk with their own format, not a blob
 *     inlined in the tree.
 *
 * What keeps the side table from drifting into that parallel universe is that
 * every binding is checked against the tree: BindMapLayerCommand refuses a
 * stale id and refuses any kind but `LayerKind::Map`, and active_map_layers()
 * -- the entry point a consumer actually calls -- resolves visibility through
 * `LayerTree::effective_visible()`. A map layer's name, its colour, its place
 * in the hierarchy, whether it is hidden or locked: all of that is A1's, and
 * none of it is duplicated here.
 *
 * ### Sampling ignores visibility, on purpose
 *
 * sample_scalar() and friends do NOT walk the tree. Street growth samples an
 * obstacle layer millions of times in one run, and an ancestor walk per sample
 * -- which allocates, in LayerTree::self_and_ancestors() -- would dominate the
 * cost of the thing it is guarding.
 *
 * So the split is: **filter once, sample many**. A consumer calls
 * active_map_layers() before its loop to get the layers that exist, are Map
 * kind, are effectively visible and actually hold data; inside the loop it
 * samples those and nothing else. A hidden obstacle layer is excluded by that
 * filter, not by the sampler.
 *
 * ### What a sample costs
 *
 * With the record pointer hoisted (MapLayerStore::find() once, outside the
 * loop) and nearest filtering on a float image, one sample_scalar() is: two
 * subtractions, two multiplications, two std::floor calls, two index clamps,
 * one multiply-add and one load. No allocation, no virtual call, no map
 * lookup, no branch that is not perfectly predicted across a run.
 *
 *   - Bilinear costs three more loads and three lerps.
 *   - An 8-bit image costs one integer-to-float conversion and one multiply.
 *   - The LayerId-taking overloads on MapLayerStore add a `std::map` lookup --
 *     a handful of pointer chases -- per call. They are for one-off queries
 *     and for tests. A loop hoists.
 *   - A function layer costs an indirect call through `std::function`, which
 *     is why function layers are for authoring and for tests rather than for
 *     the innermost loop of B8.
 *
 * That is the whole reason the samplers are free inline functions taking a
 * `const MapLayerData&` rather than members of the store: the hot path must be
 * inlinable into the caller's loop.
 *
 * ### Bounds are the contract
 *
 * A raster covers a rectangle of the world and nothing else, and what happens
 * off the edge of it is a decision, not an accident. MapPlacement states the
 * rectangle in world metres with named fields -- `min_x`, `min_z`, `size_x`,
 * `size_z`, never a `dvec2` whose `.y` secretly means z, which is the exact
 * trap `BoundingBox::center()` already sets elsewhere in this tree.
 * MapSampling::edge states what a position outside it means:
 *
 *   - `MapEdge::Outside` (default) reports `MapSampleStatus::Outside` and
 *     hands back `outside_value`, so an ignored status still yields a sane
 *     number, and so "everything outside the mask is buildable" and
 *     "everything outside the mask is blocked" are both one field away.
 *   - `MapEdge::Clamp` extends the border texel to infinity and reports `Ok`.
 *     A coarse global water mask wants this.
 *
 * `MapEdge::Wrap` is deliberately absent. A georeferenced image that repeats
 * has no meaning, and a procedural field that tiles is what a function layer
 * is for.
 *
 * The rectangle is half-open: `min_x <= x < min_x + size_x`. Two map layers
 * laid edge to edge therefore cover the seam exactly once instead of both
 * claiming it.
 *
 * ### No data is not zero
 *
 * A bound layer whose image has not been loaded yet and a bound layer whose
 * image is entirely zero are different states that read the same if sampling
 * returns a bare float. They are the single easiest thing to confuse here, so:
 *
 *   - `MapLayerStore::is_bound()` distinguishes "not a map layer at all" from
 *     "a map layer".
 *   - `MapLayerData::has_data()` distinguishes "bound, empty" from "bound,
 *     loaded".
 *   - Every sampler returns a status beside its value. `NoLayer`, `NoData` and
 *     `Ok` with a value of zero are three different answers.
 *
 * ### One source at a time
 *
 * A record is backed by an image or by a callable, never by both. The setters
 * refuse rather than letting one shadow the other, because a precedence rule
 * produces a layer that shows an image in the panel and samples from a
 * function -- the exact class of confusion this file exists to prevent.
 * Switching source is two commands inside one transaction: clear the old,
 * set the new.
 *
 * ### Mutation goes through the stack
 *
 * MapLayerStore's mutating API is private and MapLayerCommand is its only
 * friend, exactly as LayerTree does it, for exactly the reason A4 gives: a
 * mutation that skips the stack is a hole in the history.
 *
 * The one deliberate exception is load(), for A2's loader, which populates a
 * document before there is a history to record into.
 *
 * ### This file does not decode images
 *
 * G1 owns decoding (GeoTIFF, PNG16, ASC) and georeferencing. It hands over a
 * plain buffer, its dimensions, and the world rectangle it landed on. In
 * particular G1 owns the row-order flip: the buffer arrives with **row 0 at
 * minimum world z**, whatever the file format's own convention was.
 */

#pragma once

#include "scene/command.hpp"
#include "scene/layer.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace stratum::scene {

// ============================================================================
// Kinds of map layer
// ============================================================================

/**
 * @brief What the numbers in a map layer mean
 *
 * The type fixes how many channels the image may have and which samplers will
 * answer, so a caller cannot read a basemap as a scalar field by accident.
 *
 * Obstacle and Water are both single-channel masks and are deliberately
 * separate enumerators rather than one "Mask" with a label: B8 asks for
 * obstacles and F5 asks for water, and neither should have to filter a list of
 * masks by a string to find the one it means.
 */
enum class MapLayerType : uint8_t {
    Obstacle,  ///< 1 channel. Non-zero (past the threshold) means "do not build here".
    Water,     ///< 1 channel. Non-zero (past the threshold) means "this is water".
    Scalar,    ///< 1 channel. Any per-location number: density, suitability, height.
    Texture    ///< 3 or 4 channels. A basemap to drape. Sampled as colour, never as a scalar.
};

/// Number of MapLayerType enumerators, for tables indexed by type.
inline constexpr size_t kMapLayerTypeCount = 4;

/// Stable spelling of a type, for the panel and for A2. Never localised.
[[nodiscard]] const char* map_layer_type_name(MapLayerType type);

/// True when @p type is a single-channel mask, i.e. Obstacle or Water.
[[nodiscard]] constexpr bool is_mask_type(MapLayerType type) {
    return type == MapLayerType::Obstacle || type == MapLayerType::Water;
}

/// True when an image of @p channels channels may back a layer of @p type.
[[nodiscard]] constexpr bool channels_match_type(MapLayerType type, uint8_t channels) {
    return type == MapLayerType::Texture ? (channels == 3 || channels == 4) : (channels == 1);
}

// ============================================================================
// Pixels
// ============================================================================

/**
 * @brief Element type of an image's buffer
 *
 * Derived from which buffer is populated rather than stored, so the tag can
 * never disagree with the data it describes.
 */
enum class MapElement : uint8_t {
    U8,  ///< Unsigned 8-bit, normalised: 0 reads as 0.0f, 255 reads as exactly 1.0f.
    F32  ///< IEEE single precision, used verbatim.
};

/**
 * @brief A decoded raster: a plain buffer, its dimensions and its channel count
 *
 * Row-major, rows contiguous, channels interleaved. **Row 0 is at minimum
 * world z** and column 0 is at minimum world x; see the file comment on who
 * owns the flip.
 *
 * Two element types, not one, because the difference is a factor of four in
 * resident memory and both are real: a 4096x4096 RGBA basemap is 64 MB as
 * bytes and 256 MB as floats, while a GeoTIFF elevation or analysis raster is
 * float data that quantising to 8 bits would destroy. Exactly one of the two
 * buffers is populated; both empty means the image is empty.
 */
struct MapImage {
    std::vector<uint8_t> u8;  ///< Populated for a U8 image, else empty
    std::vector<float> f32;   ///< Populated for an F32 image, else empty

    uint32_t width = 0;    ///< Columns, i.e. samples along world x
    uint32_t height = 0;   ///< Rows, i.e. samples along world z
    uint8_t channels = 1;  ///< Interleaved components per texel: 1, 3 or 4

    /// True when there are no pixels at all. Distinct from "every pixel is 0".
    [[nodiscard]] bool empty() const {
        return width == 0 || height == 0 || (u8.empty() && f32.empty());
    }

    /// Which buffer is live. Meaningless, and reported as F32, for an empty image.
    [[nodiscard]] MapElement element() const {
        return u8.empty() ? MapElement::F32 : MapElement::U8;
    }

    /// Texels times channels. Computed in size_t: width * height * channels
    /// overflows 32 bits at a little over 4096x4096 RGBA.
    [[nodiscard]] size_t value_count() const {
        return static_cast<size_t>(width) * static_cast<size_t>(height) *
               static_cast<size_t>(channels);
    }

    /// Heap bytes held, for a command's footprint().
    [[nodiscard]] size_t heap_bytes() const {
        return u8.capacity() * sizeof(uint8_t) + f32.capacity() * sizeof(float);
    }

    /**
     * @brief One channel of one texel, with NO bounds check
     *
     * The caller must already have clamped @p col, @p row and @p channel into
     * range; detail::locate() and detail::read() are the only intended callers
     * and they do. Unchecked because this is the innermost operation of a loop
     * that runs millions of times and the clamp has already happened -- a
     * second check here would be paid on every tap of every sample.
     */
    [[nodiscard]] float fetch(uint32_t col, uint32_t row, uint32_t channel) const {
        const size_t index =
            (static_cast<size_t>(row) * static_cast<size_t>(width) + static_cast<size_t>(col)) *
                static_cast<size_t>(channels) +
            static_cast<size_t>(channel);
        // One predictable branch: a given layer keeps its element type for the
        // whole of a run, so this is the same way every time through the loop.
        return u8.empty() ? f32[index] : static_cast<float>(u8[index]) * (1.0f / 255.0f);
    }
};

/// Build an 8-bit image. Returns an empty image when @p pixels does not hold
/// exactly width * height * channels values, so a truncated decode cannot be
/// mistaken for a loaded one.
[[nodiscard]] MapImage make_u8_image(uint32_t width, uint32_t height, uint8_t channels,
                                     std::vector<uint8_t> pixels);

/// Build a float image, with the same size check as make_u8_image().
[[nodiscard]] MapImage make_f32_image(uint32_t width, uint32_t height, uint8_t channels,
                                      std::vector<float> pixels);

/**
 * @brief Is this image usable?
 *
 * True for an EMPTY image as well as for a well-formed one, because "no image"
 * is a legal state a caller reaches deliberately by setting MapImage{} to
 * unload one. False only for a malformed image: a zero dimension paired with
 * pixels, a channel count outside {1, 3, 4}, both buffers populated, or a
 * buffer whose length is not exactly value_count().
 */
[[nodiscard]] bool image_well_formed(const MapImage& image);

// ============================================================================
// Placement
// ============================================================================

/**
 * @brief Where an image sits in the world
 *
 * An axis-aligned rectangle on the world XZ ground plane, in metres, matching
 * procgen::Heightmap's origin-plus-cell-size convention. Named scalar fields,
 * not vectors: `.y` meaning z is a documented trap elsewhere in this tree and
 * this is a file where an x/z mix-up produces a plausible-looking wrong answer
 * rather than a crash.
 *
 * Rotation is deliberately absent. A rotated raster needs an inverse rotation
 * per sample -- two more multiplies and two more adds in the hot loop -- for a
 * case that georeferenced data does not produce: G1 reprojects into the
 * world's axes. A rotated overlay is a renderer concern, not a sampling one.
 */
struct MapPlacement {
    double min_x = 0.0;    ///< World x of the column-0 edge, metres
    double min_z = 0.0;    ///< World z of the row-0 edge, metres
    double size_x = 0.0;   ///< World width, metres. Must be finite and > 0 for an image.
    double size_z = 0.0;   ///< World depth, metres. Must be finite and > 0 for an image.

    /**
     * @brief False when this placement covers the entire world
     *
     * Only a function layer may be unbounded: a callable has a value
     * everywhere, while an image without an extent has no way to turn a world
     * position into a texel.
     */
    bool bounded = true;

    [[nodiscard]] static MapPlacement rect(double min_x, double min_z, double size_x,
                                           double size_z);

    /// A placement that contains every position. Function layers only.
    [[nodiscard]] static MapPlacement unbounded();

    /**
     * @brief Is @p world_x, @p world_z inside?
     *
     * Half-open on both axes, so tiles laid edge to edge cover the seam once.
     * Always true when unbounded. Always FALSE for a non-finite coordinate: a
     * position that is not a number is not a position, and letting NaN through
     * is how an index gets computed from it.
     */
    [[nodiscard]] bool contains(double world_x, double world_z) const;

    /// True when this placement can address an image: bounded, finite, positive.
    [[nodiscard]] bool valid_for_image() const;
};

[[nodiscard]] bool operator==(const MapPlacement& a, const MapPlacement& b);
[[nodiscard]] bool operator!=(const MapPlacement& a, const MapPlacement& b);

// ============================================================================
// Sampling policy
// ============================================================================

/// How a world position between texel centres is resolved.
enum class MapFilter : uint8_t {
    Nearest,  ///< The texel the position falls in. One load.
    Bilinear  ///< Weighted from the four surrounding texel CENTRES. Four loads.
};

/// What a position outside the placement means.
enum class MapEdge : uint8_t {
    Outside,  ///< Report Outside and hand back outside_value / outside_colour.
    Clamp     ///< Extend the border texel outward and report Ok.
};

/**
 * @brief Everything about how a layer answers, as one movable unit
 *
 * Grouped rather than spread across four commands because these four fields
 * are edited together -- a panel that changes the filter usually changes the
 * threshold in the same gesture -- and because one merging command for the
 * whole policy is one undo step for the whole adjustment.
 */
struct MapSampling {
    /// Nearest by default. A mask that is bilinearly filtered reports values
    /// between "blocked" and "free" at every edge texel, which then depend on
    /// the threshold in a way nobody set out to configure. A caller that wants
    /// a smooth field asks for it.
    MapFilter filter = MapFilter::Nearest;

    MapEdge edge = MapEdge::Outside;

    /// Value reported outside the placement under MapEdge::Outside. It is fed
    /// through `threshold` like any other value, so a mask whose outside is
    /// blocked is this field set to 1.
    float outside_value = 0.0f;

    /// Colour reported outside the placement, for a Texture layer. Transparent
    /// black by default so an unfilled basemap does not paint the world.
    glm::vec4 outside_colour{0.0f, 0.0f, 0.0f, 0.0f};

    /// A mask texel counts as set when its value is >= this. Half by default,
    /// which is the right answer for both an 8-bit 0/255 mask and a float 0/1 one.
    float threshold = 0.5f;
};

/// Exact equality. Answers "did the value change", which is what a command's
/// no-op guard needs; a tolerance would swallow a deliberate small nudge.
[[nodiscard]] bool operator==(const MapSampling& a, const MapSampling& b);
[[nodiscard]] bool operator!=(const MapSampling& a, const MapSampling& b);

// ============================================================================
// The record
// ============================================================================

/**
 * @brief A callable backing a map layer
 *
 * Takes a world position on the XZ plane and returns the layer's value there.
 * Costs almost nothing to support and is what makes every consumer of this
 * header testable without an image, a decoder or a file on disk: a keep-out
 * circle for B8 is three lines.
 *
 * Not available for MapLayerType::Texture -- a procedural basemap is not a
 * case anything needs, and supporting it would mean a second callable
 * signature returning a colour.
 */
using MapFunction = std::function<float(double world_x, double world_z)>;

/// Where a record's values come from. Derived from state, never stored.
enum class MapSource : uint8_t {
    None,      ///< Bound but empty. Not the same as an image of zeros.
    Image,     ///< Backed by pixels.
    Function   ///< Backed by a callable.
};

/**
 * @brief Everything attached to one map layer
 *
 * Plain copyable data: the unit A2 serialises and the unit a command keeps as
 * its undo state. The LayerId that owns it is the key in MapLayerStore, not a
 * field here, so a record cannot disagree with where it is filed.
 */
struct MapLayerData {
    MapLayerType type = MapLayerType::Scalar;
    MapPlacement placement;
    MapSampling sampling;

    /// Populated for an Image-backed record. Empty otherwise.
    MapImage image;

    /// Populated for a Function-backed record. Null otherwise. Never both this
    /// and `image` -- see the file comment on one source at a time.
    MapFunction function;

    /// The callable is tested first so that this agrees with sample_scalar(),
    /// which also tests it first. A record holding both cannot arise -- every
    /// command refuses it and MapLayerStore::load() rejects it -- but if one
    /// ever did, source() reporting Image while the sampler read the function
    /// would be a lie nobody could debug.
    [[nodiscard]] MapSource source() const {
        if (function) return MapSource::Function;
        if (!image.empty()) return MapSource::Image;
        return MapSource::None;
    }

    /// True when this layer can answer at all. False for a freshly bound layer
    /// and true for one whose image is entirely zero.
    [[nodiscard]] bool has_data() const { return source() != MapSource::None; }

    /// Bytes held, for a command's footprint(). The callable's captures are not
    /// visible to us and are not counted; a capturing lambda that owns a lot of
    /// memory is the caller's to bound.
    [[nodiscard]] size_t footprint() const { return sizeof(MapLayerData) + image.heap_bytes(); }
};

/**
 * @brief Does this record obey the one-source-at-a-time rule?
 *
 * False only for a record holding BOTH an image and a callable. The commands
 * refuse to build one, so this exists for MapLayerStore::load() -- the loader
 * path, which does not go through a command and is therefore the only way such
 * a record could otherwise reach the store.
 *
 * A malformed image is not checked here: it is caught by image_well_formed()
 * at the point an image is installed, and folding the two would give one
 * failure two meanings.
 */
[[nodiscard]] inline bool record_consistent(const MapLayerData& record) {
    return !(record.function && !record.image.empty());
}

// ============================================================================
// Sample results
// ============================================================================

/**
 * @brief Why a sample does or does not have an answer
 *
 * The whole point of returning this beside the value: `Ok` with a value of 0
 * and `NoData` with a value of 0 are different facts about the world, and a
 * bare float cannot tell them apart.
 */
enum class MapSampleStatus : uint8_t {
    Ok,            ///< The value came from the layer's data.
    Outside,       ///< The position is outside the placement; the value is `outside_value`.
    NoLayer,       ///< No map layer is bound to that id.
    NoData,        ///< Bound, but holding neither an image nor a function.
    WrongType,     ///< A colour asked of a mask, or a scalar asked of a texture.
    BadPlacement   ///< An image-backed record whose placement has no usable extent.
};

/// Stable spelling of a status, for logs and test failures. Never localised.
[[nodiscard]] const char* map_sample_status_name(MapSampleStatus status);

/// A scalar answer. `value` is meaningful for Ok and Outside, and 0 otherwise.
struct MapScalarSample {
    MapSampleStatus status = MapSampleStatus::NoLayer;
    float value = 0.0f;

    /// True only for Ok. Outside is a real answer but a deliberately different
    /// one, so a caller that means "inside the data" says ok().
    [[nodiscard]] bool ok() const { return status == MapSampleStatus::Ok; }

    /// True when `value` came from somewhere, i.e. Ok or Outside.
    [[nodiscard]] bool has_value() const {
        return status == MapSampleStatus::Ok || status == MapSampleStatus::Outside;
    }
};

/// A mask answer: the scalar run through the layer's threshold.
struct MapBoolSample {
    MapSampleStatus status = MapSampleStatus::NoLayer;

    /// False for every status that is not Ok or Outside, so a caller that
    /// ignores the status gets "no, this is not blocked" rather than a value
    /// derived from nothing.
    bool flag = false;

    [[nodiscard]] bool ok() const { return status == MapSampleStatus::Ok; }
};

/// A colour answer, always four components. Alpha is 1 for a 3-channel image.
struct MapColourSample {
    MapSampleStatus status = MapSampleStatus::NoLayer;
    glm::vec4 colour{0.0f, 0.0f, 0.0f, 0.0f};

    [[nodiscard]] bool ok() const { return status == MapSampleStatus::Ok; }
};

// ============================================================================
// Sampling
//
// Free functions taking the record by reference, and inline, so B8's loop can
// inline the whole thing after hoisting the pointer once. See the cost note in
// the file comment.
// ============================================================================

namespace detail {

/**
 * @brief Floor to an integer index without undefined behaviour
 *
 * `static_cast<long long>` of a huge or NaN double is UB, and under
 * MapEdge::Clamp a sample a thousand kilometres from the placement produces
 * exactly such a double. Clamping the double first keeps that a border read
 * instead of a poisoned index.
 *
 * NaN takes the low branch because `!(NaN > -kIndexLimit)` is true, which is
 * the point of writing the comparison inverted.
 */
[[nodiscard]] inline long long floor_index(double value) {
    constexpr double kIndexLimit = 1.0e9;
    if (!(value > -kIndexLimit)) return -1;
    if (value > kIndexLimit) return static_cast<long long>(kIndexLimit);
    return static_cast<long long>(std::floor(value));
}

/// Pull a continuous index into [0, count - 1]. One place, so the border rule
/// cannot drift between the nearest and the bilinear path.
[[nodiscard]] inline uint32_t clamp_texel(long long index, uint32_t count) {
    if (index <= 0) return 0;
    const long long last = static_cast<long long>(count) - 1;
    return static_cast<uint32_t>(index >= last ? last : index);
}

/**
 * @brief The texels one sample reads, and the weights between them
 *
 * Resolved once per sample so a four-channel colour read does the world-to-
 * texel arithmetic once instead of four times.
 *
 * For MapFilter::Nearest, `col1`/`row1` are unused and `bilinear` is false.
 */
struct TexelRead {
    MapSampleStatus status = MapSampleStatus::NoData;
    uint32_t col0 = 0;
    uint32_t col1 = 0;
    uint32_t row0 = 0;
    uint32_t row1 = 0;
    float tx = 0.0f;
    float tz = 0.0f;
    bool bilinear = false;
};

/// Resolve a world position against an image-backed record.
[[nodiscard]] inline TexelRead locate(const MapLayerData& map, double world_x, double world_z) {
    TexelRead read;

    if (map.image.empty()) {
        read.status = MapSampleStatus::NoData;
        return read;
    }
    if (!map.placement.valid_for_image()) {
        // Unreachable through the commands, which validate. Reachable through
        // MapLayerStore::load(), i.e. a document A2 read off disk.
        read.status = MapSampleStatus::BadPlacement;
        return read;
    }

    // A position that is not a number is not a position, and it is rejected
    // BEFORE the edge rule rather than by it. MapEdge::Clamp means "off the
    // edge of the map", and NaN is not off any edge: letting it reach the
    // arithmetic below makes floor_index() produce -1, which clamps to texel
    // (0, 0) and reports Ok -- a confident answer derived from nothing.
    if (!std::isfinite(world_x) || !std::isfinite(world_z)) {
        read.status = MapSampleStatus::Outside;
        return read;
    }

    // Containment in WORLD space, because that is where the half-open rule is
    // stated, and before any division, because that is what keeps a position a
    // thousand kilometres away from turning into a huge texel index.
    if (!map.placement.contains(world_x, world_z)) {
        if (map.sampling.edge == MapEdge::Outside) {
            read.status = MapSampleStatus::Outside;
            return read;
        }
        // Clamp: falling off the edge is not an error, the border texel is the
        // answer, and the clamps below produce it.
    }

    const MapImage& image = map.image;
    const double fx = (world_x - map.placement.min_x) / map.placement.size_x *
                      static_cast<double>(image.width);
    const double fz = (world_z - map.placement.min_z) / map.placement.size_z *
                      static_cast<double>(image.height);

    read.status = MapSampleStatus::Ok;

    if (map.sampling.filter == MapFilter::Nearest) {
        read.col0 = clamp_texel(floor_index(fx), image.width);
        read.row0 = clamp_texel(floor_index(fz), image.height);
        return read;
    }

    // Bilinear. Texel i covers [i, i+1) in fx, so its CENTRE is at i + 0.5;
    // subtracting the half texel puts the interpolation nodes on the centres.
    // Without it a sample at a texel centre returns a blend of that texel and
    // its neighbour, which reads as a half-texel shift of the whole image --
    // visible as terrain that is offset from the basemap it was built from.
    const double bx = fx - 0.5;
    const double bz = fz - 0.5;
    const long long ix = floor_index(bx);
    const long long iz = floor_index(bz);

    read.col0 = clamp_texel(ix, image.width);
    read.col1 = clamp_texel(ix + 1, image.width);
    read.row0 = clamp_texel(iz, image.height);
    read.row1 = clamp_texel(iz + 1, image.height);

    // Weights from the UNCLAMPED position, then clamped into [0, 1]: past the
    // border both taps are the same texel, so the weight no longer matters, but
    // a weight outside [0, 1] would extrapolate if it ever did.
    const double fx_frac = bx - static_cast<double>(ix);
    const double fz_frac = bz - static_cast<double>(iz);
    read.tx = static_cast<float>(fx_frac < 0.0 ? 0.0 : (fx_frac > 1.0 ? 1.0 : fx_frac));
    read.tz = static_cast<float>(fz_frac < 0.0 ? 0.0 : (fz_frac > 1.0 ? 1.0 : fz_frac));
    read.bilinear = true;
    return read;
}

/// Read one channel through a resolved TexelRead. @p read must have status Ok.
[[nodiscard]] inline float read_channel(const MapImage& image, const TexelRead& read,
                                        uint32_t channel) {
    const float v00 = image.fetch(read.col0, read.row0, channel);
    if (!read.bilinear) return v00;

    const float v10 = image.fetch(read.col1, read.row0, channel);
    const float v01 = image.fetch(read.col0, read.row1, channel);
    const float v11 = image.fetch(read.col1, read.row1, channel);

    const float top = v00 + (v10 - v00) * read.tx;
    const float bottom = v01 + (v11 - v01) * read.tx;
    return top + (bottom - top) * read.tz;
}

} // namespace detail

/**
 * @brief The value of a single-channel layer at a world position
 *
 * Valid for Obstacle, Water and Scalar, whether image-backed or function-
 * backed. A Texture layer answers WrongType: a basemap's "scalar value" would
 * have to be an invented luminance, and inventing one here means every
 * consumer silently gets a number where it should have got a refusal.
 */
[[nodiscard]] inline MapScalarSample sample_scalar(const MapLayerData& map, double world_x,
                                                   double world_z) {
    MapScalarSample sample;

    if (map.type == MapLayerType::Texture) {
        sample.status = MapSampleStatus::WrongType;
        return sample;
    }

    if (map.function) {
        if (!map.placement.contains(world_x, world_z)) {
            sample.status = MapSampleStatus::Outside;
            sample.value = map.sampling.outside_value;
            return sample;
        }
        sample.status = MapSampleStatus::Ok;
        sample.value = map.function(world_x, world_z);
        return sample;
    }

    const detail::TexelRead read = detail::locate(map, world_x, world_z);
    sample.status = read.status;
    if (read.status == MapSampleStatus::Outside) {
        sample.value = map.sampling.outside_value;
    } else if (read.status == MapSampleStatus::Ok) {
        sample.value = detail::read_channel(map.image, read, 0);
    }
    return sample;
}

/**
 * @brief Is the mask set at a world position?
 *
 * Valid for Obstacle and Water. `flag` is the scalar compared against
 * `sampling.threshold`, and an Outside sample is thresholded too -- so
 * `outside_value = 1` means "everywhere off the map is blocked", which is a
 * mask a caller genuinely wants and which a hard `false` outside would make
 * impossible to express.
 */
[[nodiscard]] inline MapBoolSample sample_mask(const MapLayerData& map, double world_x,
                                               double world_z) {
    MapBoolSample sample;
    if (!is_mask_type(map.type)) {
        sample.status = MapSampleStatus::WrongType;
        return sample;
    }

    const MapScalarSample scalar = sample_scalar(map, world_x, world_z);
    sample.status = scalar.status;
    sample.flag = scalar.has_value() && scalar.value >= map.sampling.threshold;
    return sample;
}

/// True when @p map is an Obstacle layer that is set at this position. Every
/// other case -- wrong type, no data, below threshold -- is false, because the
/// question "may I build here" has a safe default and this is it.
[[nodiscard]] inline bool is_obstacle(const MapLayerData& map, double world_x, double world_z) {
    return map.type == MapLayerType::Obstacle && sample_mask(map, world_x, world_z).flag;
}

/// True when @p map is a Water layer that is set at this position.
[[nodiscard]] inline bool is_water(const MapLayerData& map, double world_x, double world_z) {
    return map.type == MapLayerType::Water && sample_mask(map, world_x, world_z).flag;
}

/**
 * @brief The colour of a Texture layer at a world position
 *
 * Alpha is 1 for a 3-channel image, so a caller never has to know how many
 * channels the file happened to carry.
 */
[[nodiscard]] inline MapColourSample sample_colour(const MapLayerData& map, double world_x,
                                                   double world_z) {
    MapColourSample sample;
    if (map.type != MapLayerType::Texture) {
        sample.status = MapSampleStatus::WrongType;
        return sample;
    }

    const detail::TexelRead read = detail::locate(map, world_x, world_z);
    sample.status = read.status;
    if (read.status == MapSampleStatus::Outside) {
        sample.colour = map.sampling.outside_colour;
        return sample;
    }
    if (read.status != MapSampleStatus::Ok) return sample;

    sample.colour.r = detail::read_channel(map.image, read, 0);
    sample.colour.g = detail::read_channel(map.image, read, 1);
    sample.colour.b = detail::read_channel(map.image, read, 2);
    sample.colour.a = map.image.channels >= 4 ? detail::read_channel(map.image, read, 3) : 1.0f;
    return sample;
}

// ============================================================================
// The store
// ============================================================================

class MapLayerCommand;

/**
 * @brief Every map layer's payload, keyed by the LayerId that owns it
 *
 * Queries are public; every mutation is private and reachable only through a
 * Command, with load() the one documented exception. Not thread-safe, for the
 * same reason CommandStack is not.
 *
 * A record OUTLIVES the deletion of its layer, deliberately. DeleteLayerCommand
 * takes the Layer out of the tree and keeps it as undo state; if the record
 * went with it, the two histories would have to coordinate to bring the pixels
 * back together with the layer. Leaving the record in place means undoing the
 * layer delete restores a working map layer with no further work, and because
 * A1's ids are never reused the orphan can never be misattributed to a
 * different layer. The cost is that the pixels stay resident, which
 * drop_records_for_deleted_layers() reclaims once the history is gone.
 */
class MapLayerStore {
public:
    MapLayerStore() = default;

    /// Non-copyable, like AttributeStore: a copy would silently double the
    /// resident cost of every basemap in the document.
    MapLayerStore(const MapLayerStore&) = delete;
    MapLayerStore& operator=(const MapLayerStore&) = delete;
    MapLayerStore(MapLayerStore&&) = default;
    MapLayerStore& operator=(MapLayerStore&&) = default;

    // ── Lookup ──────────────────────────────────────────────────────────────

    /// True when a record is filed under @p layer. Says nothing about whether
    /// that record holds data -- see MapLayerData::has_data().
    [[nodiscard]] bool is_bound(LayerId layer) const;

    /**
     * @brief The record for @p layer, or nullptr
     *
     * **Hoist this out of a sampling loop.** The LayerId-taking samplers below
     * are one map lookup each; this pointer plus the free sample_*() functions
     * is the path that costs what the file comment claims.
     *
     * Storage is node-based, so the pointer survives binds and unbinds of OTHER
     * layers -- but not an unbind of this one, and not a command that replaces
     * this record's image. Do not hold it across an edit.
     */
    [[nodiscard]] const MapLayerData* find(LayerId layer) const;

    [[nodiscard]] size_t size() const { return m_records.size(); }
    [[nodiscard]] bool empty() const { return m_records.empty(); }

    /// Every bound layer, in id order, which is binding order.
    [[nodiscard]] std::vector<LayerId> bound_layers() const;

    /// Every bound layer of @p type, in id order. Ignores the tree entirely:
    /// for the list a consumer should actually loop over, use active_map_layers().
    [[nodiscard]] std::vector<LayerId> bound_layers_of_type(MapLayerType type) const;

    // ── One-off sampling ────────────────────────────────────────────────────
    //
    // One map lookup per call. For a single query -- a tooltip, a test, a
    // property panel -- and never for a loop.

    [[nodiscard]] MapScalarSample sample_scalar(LayerId layer, double world_x,
                                               double world_z) const;
    [[nodiscard]] MapBoolSample sample_mask(LayerId layer, double world_x, double world_z) const;
    [[nodiscard]] MapColourSample sample_colour(LayerId layer, double world_x,
                                                double world_z) const;

    // ── Maintenance ─────────────────────────────────────────────────────────

    /**
     * @brief Install a record outside the history
     *
     * For A2's loader only: a document being read has no history to record
     * into, and routing load through commands would make opening a file
     * undoable back to an empty scene.
     *
     * @return false when @p layer is kInvalidLayer, and false for a record that
     *         breaks the one-source-at-a-time rule -- see record_consistent().
     *         This is the only door into the store that is not a command, so it
     *         is the only place that rule could otherwise be bypassed.
     *
     * @note The tree is deliberately NOT consulted: the loader may well install
     *       records before it has built the tree. active_map_layers() is where
     *       the `LayerKind::Map` check happens for anything that arrived here.
     */
    bool load(LayerId layer, MapLayerData data);

    /**
     * @brief Drop records whose layer is no longer in @p tree
     *
     * **Only after CommandStack::clear().** A record kept for a deleted layer
     * is what makes undoing that delete restore a working map layer; dropping
     * it while that delete is still on the undo stack turns the undo into a
     * layer with no pixels.
     *
     * @return How many records were dropped.
     */
    size_t drop_records_for_deleted_layers(const LayerTree& tree);

    /// Forget everything. Used on load, and on close.
    void clear();

private:
    /**
     * @brief The one door into the mutating API
     *
     * Same shape as LayerTree's friendship with LayerCommand, and for the same
     * reason: "every mutation goes through the command stack" is enforced by
     * the compiler in one place rather than asserted in a comment.
     */
    friend class MapLayerCommand;

    /// File an empty record of @p type. False when one is already filed.
    bool bind(LayerId layer, MapLayerType type);

    /// Take the record out and hand it back. Empty when nothing was filed.
    std::optional<MapLayerData> unbind(LayerId layer);

    MapLayerData* mutable_record(LayerId layer);

    /// Node-based, so find() survives unrelated edits, and ordered by id, so
    /// iteration is binding order rather than a hash's whim. Matches LayerTree.
    std::map<LayerId, MapLayerData> m_records;
};

// ============================================================================
// Consumer entry point
// ============================================================================

/**
 * @brief The map layers a generator should actually read, filtered once
 *
 * Returns the ids, in tree order over @p tree, of every layer that is
 *
 *   1. still in the tree -- an orphaned record whose layer was deleted must not
 *      keep steering street growth,
 *   2. of `LayerKind::Map` -- load() cannot check this, so it is checked here,
 *   3. effectively visible, i.e. neither it nor any ancestor is hidden, so
 *      hiding a group really does take its obstacle maps out of the
 *      computation,
 *   4. bound with a record of @p type, and
 *   5. actually holding data, so a consumer does not have to re-check NoData
 *      inside its loop.
 *
 * Call this ONCE, before the loop; then hoist MapLayerStore::find() for each id
 * and sample through the free functions. That is the whole performance
 * contract of this header -- see the cost note in the file comment.
 */
[[nodiscard]] std::vector<LayerId> active_map_layers(const LayerTree& tree,
                                                     const MapLayerStore& store,
                                                     MapLayerType type);

// ============================================================================
// Commands
// ============================================================================

/**
 * @brief Base for every map layer edit
 *
 * Holds the tree, the store and the target, and owns the forwarding into
 * MapLayerStore's private API. A new operation subclasses this; it cannot
 * reach the store directly, because MapLayerCommand is the store's only friend.
 *
 * The tree is held as well as the store because the binding invariant -- a
 * record belongs to an existing `LayerKind::Map` layer -- can only be checked
 * against the tree, and checking it at apply() time is what stops the side
 * table drifting away from the hierarchy it is supposed to decorate.
 */
class MapLayerCommand : public Command {
public:
    /// The layer this command acts on.
    [[nodiscard]] LayerId layer() const { return m_layer; }

protected:
    MapLayerCommand(LayerTree& tree, MapLayerStore& store, LayerId layer)
        : m_tree(&tree), m_store(&store), m_layer(layer) {}

    [[nodiscard]] const LayerTree& tree() const { return *m_tree; }
    [[nodiscard]] MapLayerStore& store() const { return *m_store; }

    /// True when @p other edits the same layer of the same tree and the same
    /// store. Without all three, two documents open at once would coalesce each
    /// other's edits.
    [[nodiscard]] bool same_target(const MapLayerCommand& other) const {
        return m_tree == other.m_tree && m_store == other.m_store && m_layer == other.m_layer;
    }

    /// True when the target is a layer that is present and of kind Map.
    [[nodiscard]] bool target_is_map_layer() const;

    // Forwarders into MapLayerStore. See the matching private members there.

    bool bind_record(MapLayerType type);
    std::optional<MapLayerData> unbind_record();
    MapLayerData* mutable_record();

    LayerTree* m_tree;
    MapLayerStore* m_store;
    LayerId m_layer;
};

/**
 * @brief Attach an empty map record to an existing `LayerKind::Map` layer
 *
 * Does NOT create the layer. A caller that wants both runs CreateLayerCommand
 * and this inside one transaction, which is the pattern A1 documents for
 * creating a layer and configuring it as one undo step.
 *
 * Refuses a stale id, a layer of any other kind, and a layer that is already
 * bound.
 */
class BindMapLayerCommand : public MapLayerCommand {
public:
    BindMapLayerCommand(LayerTree& tree, MapLayerStore& store, LayerId layer, MapLayerType type);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

private:
    MapLayerType m_type;
};

/**
 * @brief Detach a map record, keeping it for the undo
 *
 * The undo state is the whole record, pixels included, which is why
 * footprint() reports the image: unbinding a 64 MB basemap puts 64 MB on the
 * undo stack, and the stack's byte bound exists precisely to see that.
 */
class UnbindMapLayerCommand : public MapLayerCommand {
public:
    UnbindMapLayerCommand(LayerTree& tree, MapLayerStore& store, LayerId layer);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;

private:
    /// Absent until the first apply(). The record, verbatim.
    std::optional<MapLayerData> m_record;
};

/**
 * @brief Replace a layer's pixels
 *
 * An empty MapImage is legal and means "unload", returning the layer to the
 * no-data state. A malformed one is refused, as is an image whose channel
 * count does not match the layer's type, and as is any image at all on a
 * function-backed layer.
 *
 * The image is SWAPPED rather than copied, so neither apply() nor revert()
 * duplicates a large buffer, and the command holds exactly one image at a time
 * -- always the one that is not currently installed. That also makes redo
 * free: apply() and revert() are the same swap.
 *
 * Two identical images are not detected. Refusing a no-op would mean a memcmp
 * of up to a quarter of a gigabyte on an operation a user performs by picking
 * a file, which loses to letting one redundant step onto the stack.
 */
class SetMapLayerImageCommand : public MapLayerCommand {
public:
    SetMapLayerImageCommand(LayerTree& tree, MapLayerStore& store, LayerId layer, MapImage image);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;

private:
    /// Before the first apply(), the incoming image. Afterwards, the one it
    /// displaced. revert() swaps back, so at every apply() this is the incoming
    /// image again.
    MapImage m_image;
};

/**
 * @brief Move or resize the rectangle an image covers
 *
 * Merges with a later placement change of the same layer, so dragging the
 * placement gizmo is one undo step rather than one per mouse-move; seal() on
 * mouse-up ends it.
 *
 * Refuses a placement that cannot address an image -- unbounded, zero-sized,
 * non-finite -- unless the record is function-backed, where unbounded is the
 * whole point.
 */
class SetMapLayerPlacementCommand : public MapLayerCommand {
public:
    SetMapLayerPlacementCommand(LayerTree& tree, MapLayerStore& store, LayerId layer,
                                MapPlacement placement);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] bool merge(const Command& next) override;

private:
    MapPlacement m_placement;
    MapPlacement m_old_placement;
};

/**
 * @brief Change filter, edge rule, outside value or threshold
 *
 * One command for the whole policy, and it merges, so dragging a threshold
 * slider is one undo step. Refuses a change that changes nothing, following
 * A1's rule that an undo menu full of steps that do nothing reads as undo being
 * broken.
 */
class SetMapLayerSamplingCommand : public MapLayerCommand {
public:
    SetMapLayerSamplingCommand(LayerTree& tree, MapLayerStore& store, LayerId layer,
                               MapSampling sampling);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] bool merge(const Command& next) override;

private:
    MapSampling m_sampling;
    MapSampling m_old_sampling;
};

/**
 * @brief Give a layer a callable, or take it away
 *
 * A null MapFunction clears it, returning the layer to the no-data state.
 * Refused on a Texture layer, and refused on an image-backed layer -- clear the
 * image first, in the same transaction if you like.
 *
 * Unlike every other setter here there is no equal-value guard: `std::function`
 * has no equality operator, and there is no way to ask whether two callables do
 * the same thing. Setting the same lambda twice therefore records two steps.
 * The alternative -- a user-supplied identity tag to compare -- costs every
 * caller a field to get a guard that only matters for an operation nobody
 * performs by dragging.
 */
class SetMapLayerFunctionCommand : public MapLayerCommand {
public:
    SetMapLayerFunctionCommand(LayerTree& tree, MapLayerStore& store, LayerId layer,
                               MapFunction function);

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;

private:
    /// Swapped, like the image: at every apply() this holds the incoming
    /// callable and afterwards the one it displaced.
    MapFunction m_function;
};

// ============================================================================
// Convenience
//
// Thin wrappers that build the command and execute it, matching A3's
// set_attribute()/clear_attribute(). Each returns what the stack returned, so a
// refusal is visible at the call site.
// ============================================================================

bool bind_map_layer(CommandStack& stack, LayerTree& tree, MapLayerStore& store, LayerId layer,
                    MapLayerType type);

bool unbind_map_layer(CommandStack& stack, LayerTree& tree, MapLayerStore& store, LayerId layer);

bool set_map_layer_image(CommandStack& stack, LayerTree& tree, MapLayerStore& store, LayerId layer,
                         MapImage image);

bool set_map_layer_placement(CommandStack& stack, LayerTree& tree, MapLayerStore& store,
                             LayerId layer, MapPlacement placement);

bool set_map_layer_sampling(CommandStack& stack, LayerTree& tree, MapLayerStore& store,
                            LayerId layer, MapSampling sampling);

bool set_map_layer_function(CommandStack& stack, LayerTree& tree, MapLayerStore& store,
                            LayerId layer, MapFunction function);

} // namespace stratum::scene
