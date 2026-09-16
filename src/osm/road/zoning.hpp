// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file zoning.hpp
 * @brief What a block or a lot is FOR: inferred from landuse, overridden by hand
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### The decision this file makes: a zone is an ATTRIBUTE, not a field
 *
 * The obvious shape is `Block::zone`, an enum on the struct, plus a second
 * `Block::zone_is_painted` flag so a re-import does not eat a user's edit. That
 * shape was rejected, and the reasons are worth stating because "just add a
 * field" will be proposed again:
 *
 *   - **A field dies with the extraction.** `blocks.hpp` says it outright:
 *     `Block::id` is "stable within one extraction only". Edit one street and
 *     `extract_blocks()` renumbers everything, so a zone living on the Block
 *     struct is destroyed by an edit two streets away. An AttributeObject is
 *     minted once and outlives every re-extraction, which is precisely the
 *     property a hand-painted zone needs.
 *   - **The second field is A3's chain, rebuilt badly.** `zone` plus
 *     `zone_is_painted` is a two-rung precedence ladder. `scene/attributes.hpp`
 *     already has a five-rung one -- User, Object, Layer, Default, None -- that
 *     is tested, undoable, serialisable and, critically, REPORTS WHICH RUNG
 *     ANSWERED. Building a second, parallel notion of precedence for one value
 *     would leave the codebase with two answers to "did my edit stick?".
 *   - **Everything downstream already speaks attributes.** Track D's rule
 *     language reads attributes by name (`attributes.hpp` names `zoning` in its
 *     own file comment as an example). A2 serialises AttributeStore. K1's
 *     inspector renders attribute rows with their source. Track I's reports
 *     aggregate attributes over a selection. A field needs a bespoke bridge to
 *     each of those; an attribute needs none.
 *
 * The mapping is exact, which is the tell that this is the right model:
 *
 *   | Zoning idea                  | A3 rung                    |
 *   |------------------------------|----------------------------|
 *   | Painted by hand              | `AttributeSource::User`    |
 *   | Inferred from a landuse area | `AttributeSource::Object`  |
 *   | "this whole layer is retail" | `AttributeSource::Layer`   |
 *   | Document-wide fallback       | `AttributeSource::Default` |
 *   | Unzoned                      | `AttributeSource::None`    |
 *
 * **Painted beats inferred for free**, because A3 already ranks User above
 * Object, and for the identical reason: the rule engine rewrites Object values
 * on every re-evaluation, and re-running inference after a road edit is exactly
 * such a re-evaluation. Two slots is what keeps a hand edit alive across it.
 * Nothing in this file implements precedence; `resolve_zone()` asks the store.
 *
 * The cost is honest and small: a read is a string compare rather than an enum
 * compare, plus one hash lookup to find the key. Callers in a hot loop hoist the
 * key -- `attributes.hpp` guarantees an interned key is never removed and stays
 * valid for the store's lifetime, which is what makes hoisting safe.
 *
 * ### Why the value is a string and not a number
 *
 * `AttributeType` has no integer (see attributes.hpp for why), so the choice is
 * String or Double. A Double holding an enum ordinal is a saved file that breaks
 * the day someone inserts an enumerator, and it is unreadable in a rule --
 * `zoning == 3` means nothing. OSM tags are strings, CGA-style rules compare
 * strings, and the inspector shows strings. So the store holds `"residential"`,
 * and `zone_type_name()` / `zone_type_from_name()` are the only two places that
 * know the spelling.
 *
 * ### Unzoned is not "zoned as unknown"
 *
 * Three states, and collapsing any two of them makes a real bug unreportable:
 *
 *   - **Unzoned** -- no rung holds `zoning` at all. `ZoneStatus::Unzoned`.
 *     Nothing inferred a zone and nobody painted one.
 *   - **Zoned as unknown** -- a rung holds the string `"unknown"`, meaning a
 *     human or the importer deliberately said "this is land I am not
 *     classifying". `ZoneStatus::Zoned` with `ZoneType::Unknown`.
 *   - **Unrecognised** -- a rung holds something that is not a zone: a double, or
 *     the string `"resedential"` out of a hand-edited save file.
 *     `ZoneStatus::Unrecognised`, with the offending text in `ZoneQuery::raw`.
 *
 * `ZoneType::Unknown` is therefore a REAL zone value, not a null. A caller that
 * reads `ZoneQuery::zone` without reading `ZoneQuery::status` cannot tell an
 * unzoned block from one deliberately marked unknown, which is the exact
 * confusion this section exists to prevent. Test `zoned()`, never `zone`.
 *
 * Note what follows for parsing: `zone_type_from_name("unknown")` SUCCEEDS and
 * yields `ZoneType::Unknown`, while `zone_type_from_name("resedential")` returns
 * nullopt. A parse is strict and case-sensitive on purpose. Writers go through
 * `zone_type_name()`, so the store only ever holds canonical spellings; a
 * non-canonical one came from outside and surfacing it as Unrecognised with its
 * raw text is visible and fixable, where silently normalising `"Residential"`
 * would hide a broken importer forever.
 *
 * ### A block overlapping two landuse areas: largest wins, never refuse
 *
 * Refusing was considered and rejected. The alternative to a zone is Unzoned,
 * and an unzoned block generates nothing at all, so refusing throws away a
 * usable answer to avoid being slightly wrong -- and it refuses most often in
 * dense city centres, where the data is best and the user cares most.
 *
 * So the largest overlap wins, and three refinements keep that from being naive:
 *
 *   - **Overlaps are POOLED BY ZONE, not ranked by polygon.** Two adjacent
 *     `landuse=residential` polygons covering 30% of a block each lose to a
 *     single 45% `landuse=commercial` one under per-polygon ranking. That is
 *     wrong: the block is 60% residential. Real OSM splits districts across many
 *     polygons -- one per surveyor session, per estate, per administrative
 *     boundary -- so per-polygon ranking loses to a mapping accident.
 *   - **A coverage floor.** `ZoningConfig::min_coverage`, a quarter of the block
 *     by default. Below it the landuse area is clipping a corner and the block
 *     is mostly something else; the outcome is `BelowFloor`, which still carries
 *     the best candidate so a user can see what was nearly chosen.
 *   - **Contested is reported, not refused.** When the runner-up is within
 *     `ZoningConfig::contested_margin` of the winner, `ZoneInference::contested`
 *     is set. 51/49 is a coin flip that will land the other way the moment the
 *     geometry is nudged, and a user who can find those blocks can paint them.
 *     Silently picking one and saying nothing is the failure mode here.
 *
 * **Ties break independently of input order.** Equal pooled areas are settled by
 * the smaller contributing source id, then by the enumerator. The tie-break is
 * arbitrary; what matters is that it does not consult the order the sources
 * arrived in, because that order is the importer's iteration order and a zoning
 * that changes when the importer's hash map rehashes is not reproducible.
 *
 * "Equal" here means exactly equal, with no tolerance, and that is part of the
 * same promise rather than sloppiness. Equality within a tolerance is not
 * transitive, and a winner picked by scanning a vector under a non-transitive
 * comparison depends on which end of the vector the scan started from. Clipper2
 * computes these areas from integer coordinates, so two genuinely identical
 * overlaps do produce identical doubles and a real tie is still seen as one.
 *
 * ### Water is a zone
 *
 * `AreaType::Water` maps to `ZoneType::Water` rather than being dropped from the
 * source set. Dropping it looks tidy and is a trap: a block bounded by bridges
 * over a river is 90% water and 10% clipped by a residential polygon on the
 * bank, and with water excluded that block comes out residential. Keeping water
 * in lets it win, and a caller that wants to skip water blocks can.
 *
 * ### What this does NOT do
 *
 *   - **No lots.** C2 owns lot subdivision and this file does not include it.
 *     A lot is zoned by handing its ring in as a `ZoneTarget`, the same struct a
 *     block uses, which is the whole reason `ZoneTarget` is a ring plus a handle
 *     and not a `Block`.
 *   - **No map layers.** `scene/map_layer.hpp` names C6 as a consumer, and a
 *     painted zoning raster is a real second source. It is deliberately not
 *     wired here: it would be a third rung with no place in A3's chain, and the
 *     right shape for it is a `ZoneSourceArea` producer, not a special case
 *     inside the resolver.
 *   - **No writes on its own.** Inference is pure (`infer_zones()` touches no
 *     store), and applying it is a separate, explicit call. That is what lets a
 *     test assert the decision without a store, and what lets the importer
 *     choose the history-free path.
 *
 * Everything here lives in stratum_core: glm, Clipper2 and the standard library.
 * No SDL, no ImGui, no renderer.
 */

#pragma once

#include "osm/road/blocks.hpp"
#include "osm/types.hpp"
#include "scene/attributes.hpp"
#include "scene/command.hpp"

#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Zone vocabulary
// ============================================================================

/**
 * @brief What a piece of land is for
 *
 * A closed vocabulary. It is deliberately NOT `AreaType`: `AreaType` describes
 * what OSM drew (`natural=water`, `leisure=park`), while a zone describes what
 * a block is FOR, and the two only partly overlap. `AreaType` has no Retail;
 * a zone must, because a user paints one. A zone has no equivalent of an
 * unclassifiable area, because `zone_from_area_type()` declines to zone from one
 * rather than inventing a zone for it.
 *
 * @warning `Unknown` is a real, deliberate value -- "this is zoned, as nothing
 *          in particular" -- and NOT a null. A target with no zone at all is
 *          `ZoneStatus::Unzoned` and carries no ZoneType. See the file comment.
 */
enum class ZoneType : uint8_t {
    Residential = 0,
    Commercial,
    Retail,
    Industrial,
    Park,
    Forest,
    Grass,
    Farmland,
    Cemetery,
    Parking,
    Water,
    Unknown,
};

/// Number of ZoneType enumerators. Used to size the per-zone pools in the solver.
inline constexpr size_t kZoneTypeCount = 12;

/**
 * @brief Every enumerator, in declaration order
 *
 * Exposed so a UI combo box, a report and a totality test all enumerate from one
 * list. A new enumerator added to ZoneType and not to this array is caught by
 * the Zoning suite, which checks the array's length against kZoneTypeCount and
 * round-trips every entry.
 */
inline constexpr std::array<ZoneType, kZoneTypeCount> kAllZoneTypes = {
    ZoneType::Residential, ZoneType::Commercial, ZoneType::Retail,   ZoneType::Industrial,
    ZoneType::Park,        ZoneType::Forest,     ZoneType::Grass,    ZoneType::Farmland,
    ZoneType::Cemetery,    ZoneType::Parking,    ZoneType::Water,    ZoneType::Unknown,
};

/**
 * @brief Canonical spelling, e.g. "residential"
 *
 * This is the text that goes INTO the attribute store and into a save file, so
 * changing one of these strings changes the on-disk format. Lowercase, matching
 * the OSM tag values they came from.
 */
[[nodiscard]] std::string_view zone_type_name(ZoneType zone);

/**
 * @brief Parse a canonical spelling
 *
 * Strict and case-sensitive; see the file comment for why. `"unknown"` parses
 * successfully to `ZoneType::Unknown` -- that is a zone, not a parse failure.
 *
 * @return The zone, or nullopt when @p name is not a canonical spelling.
 */
[[nodiscard]] std::optional<ZoneType> zone_type_from_name(std::string_view name);

/**
 * @brief The zone a landuse area lends to whatever falls inside it
 *
 * `AreaType::Unknown` yields nullopt: an area the parser could not classify must
 * not zone anything, because "I do not know what this is" is not the same claim
 * as "this is land of no particular type" and only the latter is `ZoneType::
 * Unknown`.
 *
 * @note There is no `AreaType::Retail`. `parser.cpp` folds `landuse=retail` into
 *       `AreaType::Commercial`, so retail cannot be INFERRED today and can only
 *       be painted. That is an upstream loss, recorded here rather than papered
 *       over by guessing retail from something else.
 */
[[nodiscard]] std::optional<ZoneType> zone_from_area_type(AreaType type);

// ============================================================================
// Reading a zone off the store
// ============================================================================

/**
 * @brief The attribute name a zone lives under
 *
 * One spelling, in one place. Track D's rules and A2's save files both use it,
 * so it is part of the format.
 */
inline constexpr std::string_view kZoneAttributeName = "zoning";

/// Which of the three states a read landed in. See the file comment.
enum class ZoneStatus : uint8_t {
    Unzoned = 0,    ///< No rung holds `zoning`
    Zoned,          ///< A rung holds a canonical zone name
    Unrecognised,   ///< A rung holds something that is not a zone name
};

/// "Unzoned", "Zoned", "Unrecognised". Mirrors attribute_status_name().
[[nodiscard]] std::string_view zone_status_name(ZoneStatus status);

/**
 * @brief What a zone read found, and where it came from
 *
 * @warning Test `zoned()`. Reading `zone` without `status` cannot tell an
 *          unzoned target from one deliberately marked `ZoneType::Unknown`,
 *          because both read back `Unknown`. This is the same shape of warning
 *          `AttributeQuery` carries about its `value` pointer, for the same
 *          reason.
 */
struct ZoneQuery {
    ZoneStatus status = ZoneStatus::Unzoned;

    /// Meaningful only when status is Zoned.
    ZoneType zone = ZoneType::Unknown;

    /// Which rung answered. `None` exactly when status is Unzoned.
    scene::AttributeSource source = scene::AttributeSource::None;

    /// Which layer supplied it. Meaningful only when `source` is Layer.
    scene::LayerRef layer = scene::kNoLayer;

    /**
     * @brief What the attribute actually held, rendered for a human
     *
     * Empty when Unzoned. On Unrecognised this is the whole point: it is what
     * lets an inspector say *"this block's zoning reads 'resedential', which is
     * not a zone"* instead of silently showing it as unzoned. Zone names are
     * short enough to live in a std::string's small-buffer, so populating it on
     * every read costs no allocation.
     */
    std::string raw;

    [[nodiscard]] bool zoned() const { return status == ZoneStatus::Zoned; }
    [[nodiscard]] explicit operator bool() const { return zoned(); }
};

/**
 * @brief Intern the zoning key, creating it on first use
 *
 * Hoist this out of any loop. The returned key is valid for the store's
 * lifetime -- attributes.hpp guarantees keys are never removed.
 */
[[nodiscard]] scene::AttributeKey zone_key(scene::AttributeStore& store);

/// The zoning key if anything ever interned it, else an invalid key.
[[nodiscard]] scene::AttributeKey find_zone_key(const scene::AttributeStore& store);

/**
 * @brief Resolve a zone through the whole chain: User, Object, Layer, Default
 *
 * @param store  Document store
 * @param object Target's attribute record. A default-constructed handle skips
 *               the object rungs, which answers "what would the layer give".
 * @param layer  Layer to inherit from, or `scene::kNoLayer`. Required rather
 *               than defaulted, for the reason AttributeStore::resolve() states:
 *               a forgotten layer argument silently loses inheritance and the
 *               result gives no sign of it.
 */
[[nodiscard]] ZoneQuery resolve_zone(const scene::AttributeStore& store,
                                     scene::AttributeObject object, scene::LayerRef layer);

/**
 * @brief Read ONE rung, with no inheritance
 *
 * What an inspector needs to grey out "Revert to inferred", and what a test
 * needs to prove that re-running inference wrote the Object rung even though
 * the User rung is what resolves.
 */
[[nodiscard]] ZoneQuery peek_zone(const scene::AttributeStore& store,
                                  const scene::AttributeTarget& target);

/// The User rung of @p object: where a hand-painted zone lands.
[[nodiscard]] scene::AttributeTarget painted_zone_target(scene::AttributeStore& store,
                                                         scene::AttributeObject object);

/// The Object rung of @p object: where an inferred zone lands.
[[nodiscard]] scene::AttributeTarget inferred_zone_target(scene::AttributeStore& store,
                                                          scene::AttributeObject object);

/// The Layer rung: "everything in this layer is retail unless it says otherwise".
[[nodiscard]] scene::AttributeTarget layer_zone_target(scene::AttributeStore& store,
                                                       scene::LayerRef layer);

/// The Default rung: the document-wide fallback.
[[nodiscard]] scene::AttributeTarget default_zone_target(scene::AttributeStore& store);

// ============================================================================
// Writing a zone
// ============================================================================

/**
 * @brief Paint a zone by hand. Undoable, and beats anything inferred.
 *
 * Lands on the User rung, so a later `infer_zones()` + `apply_inferred_zones()`
 * cannot overwrite it. That is the requirement, and it is met by A3's ranking
 * rather than by anything in this file.
 *
 * @return The stack's answer: false when the command refused.
 */
bool paint_zone(scene::CommandStack& stack, scene::AttributeStore& store,
                scene::AttributeObject object, ZoneType zone);

/**
 * @brief Take the hand-painted zone away so the inferred one shows through
 *
 * Not the same as painting the inferred value: clearing leaves the object owning
 * nothing at the User rung, so a later re-inference is visible again. Painting
 * the same value pins it. attributes.hpp makes the same distinction for the same
 * reason.
 *
 * @return false when nothing was painted, in which case no undo step is
 *         recorded -- a menu entry that changes nothing reads as broken undo.
 */
bool clear_painted_zone(scene::CommandStack& stack, scene::AttributeStore& store,
                        scene::AttributeObject object);

/**
 * @brief Write an inferred zone as an undoable edit
 *
 * The interactive path: a user re-runs zoning from a panel and expects one undo
 * to take it back. The importer uses `load_inferred_zone()` instead.
 */
bool set_inferred_zone(scene::CommandStack& stack, scene::AttributeStore& store,
                       scene::AttributeObject object, ZoneType zone);

/// Remove the inferred zone. False when there was none.
bool clear_inferred_zone(scene::CommandStack& stack, scene::AttributeStore& store,
                         scene::AttributeObject object);

/// Zone a whole layer. Loses to anything on the object itself.
bool set_layer_zone(scene::CommandStack& stack, scene::AttributeStore& store,
                    scene::LayerRef layer, ZoneType zone);

/// Document-wide fallback zone. Loses to everything.
bool set_default_zone(scene::CommandStack& stack, scene::AttributeStore& store, ZoneType zone);

/**
 * @brief Write an inferred zone with NO undo history
 *
 * @warning A deliberate hole in the history, and the only one here. It forwards
 *          to `AttributeStore::load_value()`, whose own warning applies verbatim:
 *          this is for the importer and for A2's loader, which run before the
 *          document is ever shown and have no history to write into. Recording a
 *          city's worth of inferred zones as commands would fill the undo stack
 *          with the act of opening a file.
 *
 *          Anything running while a user is looking at the document uses
 *          `set_inferred_zone()`.
 */
bool load_inferred_zone(scene::AttributeStore& store, scene::AttributeObject object, ZoneType zone);

// ============================================================================
// Inference input
// ============================================================================

/**
 * @brief A piece of land that can be zoned
 *
 * Deliberately a ring, its holes and a handle, and NOT a `Block`. Lots (C2) are
 * zoned by the same code and this file does not include `lots.hpp`; a struct
 * that named `Block` would force one of the two to be converted at every call.
 *
 * `ring` is in local metres, anticlockwise, first point NOT repeated -- the same
 * convention as `Block::ring` and `Corridor::outline`, so no transform is needed
 * between them. Clockwise is accepted and measured identically: the solver uses
 * absolute areas, because refusing a clockwise ring would reject a lot cut by
 * code that happened to wind the other way, for no benefit.
 *
 * @note The rings are held BY VALUE, so a vector of targets for a city extract
 *       is a second copy of every block boundary. That is deliberate -- a target
 *       outlives the `BlockGraph` it came from whenever a panel re-solves after
 *       an edit, and a span into a rebuilt graph would dangle -- but it is a real
 *       cost and a caller zoning hundreds of thousands of lots should know it.
 */
struct ZoneTarget {
    /// Attribute record the zone is written to. An invalid handle is skipped by the appliers.
    scene::AttributeObject object{};

    /// Boundary in local metres, first point not repeated
    std::vector<glm::dvec2> ring;

    /**
     * @brief Land the ring encloses but the target does not own, in local metres
     *
     * The same thing `Block::holes` is: the turning bulb of a cul-de-sac, the
     * loop road of an estate. Blocks.hpp is explicit that `Block::area` already
     * has these subtracted, so a target carrying the ring WITHOUT them is a
     * different parcel from the block it came from -- larger, and zoned partly on
     * evidence lying in land that belongs to another block.
     *
     * The reasoning is exactly `ZoneSourceArea::holes`', applied to the other
     * side of the clip, and the solver treats them the same way: the subject goes
     * into Clipper2 as outer plus holes under an even-odd fill, so a hole cuts
     * whichever way round it is wound. `Block::holes` arrives clockwise; a lot
     * cutter has no such obligation, and neither has to be normalised here.
     *
     * Empty for the overwhelming majority of targets.
     */
    std::vector<std::vector<glm::dvec2>> holes;
};

/**
 * @brief A landuse area that lends its type to the land inside it
 *
 * Built from `osm::Area` by `zone_sources_from_areas()`, or by hand.
 */
struct ZoneSourceArea {
    ZoneType zone = ZoneType::Unknown;

    /// Outer boundary in local metres, first point not repeated
    std::vector<glm::dvec2> ring;

    /**
     * @brief Inner rings, in local metres
     *
     * Land inside a hole is NOT inside the area. A park with a lake cut out of
     * it does not zone the lake as park. Orientation is not required to be
     * opposite the outer ring: the solver clips with an even-odd fill rule, so a
     * hole excludes whichever way round it is wound -- OSM multipolygons do not
     * guarantee an orientation and trusting one is how a park swallows its lake.
     */
    std::vector<std::vector<glm::dvec2>> holes;

    /**
     * @brief OSM id of the way or relation this came from
     *
     * Two jobs. It is the provenance an inspector shows -- "residential, from
     * way 12345" -- and it is the ORDER-INDEPENDENT tie-break when two zones
     * cover exactly the same area. See the file comment on ties.
     */
    int64_t source_id = 0;
};

/**
 * @brief Convert parsed areas into zoning sources, dropping the unclassifiable
 *
 * Areas whose `AreaType` maps to no zone are dropped, as are areas with fewer
 * than three points. The result is safe to hand straight to `infer_zones()`.
 */
[[nodiscard]] std::vector<ZoneSourceArea> zone_sources_from_areas(const std::vector<Area>& areas);

/**
 * @brief Pair a block's boundary with the attribute record that will hold its zone
 *
 * Copies `Block::ring` AND `Block::holes`. Both, because `Block::area` is
 * documented as having the holes subtracted: a target built from the ring alone
 * measures a parcel the block does not own, and the land inside a cul-de-sac
 * bulb -- which is its own block, with its own zone -- would vote in this one's
 * inference. Nothing else about the block is carried; `Block::id` deliberately
 * is not, because it is stable within one extraction only.
 */
[[nodiscard]] ZoneTarget zone_target_from_block(const Block& block, scene::AttributeObject object);

// ============================================================================
// Inference configuration and output
// ============================================================================

/**
 * @brief Tunables of the overlap solve
 */
struct ZoningConfig {
    /**
     * @brief Fraction of the target the winner must cover, in [0, 1]
     *
     * A quarter by default. Two failure modes bound this from either side. Set
     * it too high and a block straddling a district boundary comes out unzoned,
     * generating nothing, which is the worse outcome -- most of a real extract
     * has no landuse mapping at all, so the ones that do are precious. Set it to
     * zero and a block that shares a single corner with a car park is zoned as
     * parking.
     *
     * A quarter says: a landuse area that covers less than a corner of the block
     * is describing something else. It is a floor on a GUESS, not a confidence
     * threshold, and a caller that wants every scrap of evidence used sets it
     * to 0.
     */
    double min_coverage = 0.25;

    /**
     * @brief Coverage gap below which the winner is reported as contested
     *
     * Reporting only -- it never changes the zone chosen. 60/40 is a decision;
     * 51/49 is a coin flip that lands the other way when a road moves a metre,
     * and a user who can list the contested blocks can paint them once instead
     * of re-discovering the flip after every edit.
     */
    double contested_margin = 0.10;

    /**
     * @brief Metres-to-integer factor for Clipper2
     *
     * Clipper2 is integer-based. 1000 is millimetres, matching what
     * `junction_curb.cpp` uses, which keeps quantisation error far below any
     * area a zoning decision turns on. Coordinates beyond 1e9 after scaling --
     * a thousand kilometres of local extent, twenty times the widest extract
     * anyone imports -- are refused rather than silently wrapped.
     */
    double clipper_scale = 1000.0;
};

/// Why a target ended up with the zone it has, or with none.
enum class ZoneOutcome : uint8_t {
    NoOverlap = 0,  ///< No source area touches the target at all
    BelowFloor,     ///< Something overlapped, but the best was under min_coverage
    /// The target is not a polygon: under three points, unrepresentable, or
    /// enclosing no land once its holes are taken out
    Degenerate,
    Inferred,       ///< A zone was chosen
};

/// "NoOverlap", "BelowFloor", "Degenerate", "Inferred".
[[nodiscard]] std::string_view zone_outcome_name(ZoneOutcome outcome);

/**
 * @brief The decision for one target, and the evidence behind it
 *
 * Carried rather than logged and forgotten, for the reason `BlockStats` gives:
 * an inference that zoned four blocks where it should have zoned five looks
 * exactly like a correct one.
 *
 * @warning `zone` is meaningful only when `inferred()`. On `BelowFloor` it names
 *          the best candidate, for diagnosis. On `NoOverlap` and `Degenerate` it
 *          is `ZoneType::Unknown`, which is a REAL zone, so a caller that writes
 *          `zone` without checking `inferred()` marks every unzoned block as
 *          deliberately unknown.
 */
struct ZoneInference {
    ZoneOutcome outcome = ZoneOutcome::NoOverlap;

    /// Winner. See the warning above.
    ZoneType zone = ZoneType::Unknown;

    /// Winner's pooled share of the target, in [0, 1].
    double coverage = 0.0;

    /// Largest single source contributing to the winner. 0 when there is no winner.
    int64_t source_id = 0;

    /// Second-placed zone, or `Unknown` with zero coverage when there is none.
    ZoneType runner_up = ZoneType::Unknown;

    /// Runner-up's pooled share of the target, in [0, 1].
    double runner_up_coverage = 0.0;

    /// True when `coverage - runner_up_coverage` is within ZoningConfig::contested_margin.
    bool contested = false;

    /**
     * @brief The target's own area in square metres, measured after quantisation
     *
     * Holes subtracted, and measured under the SAME even-odd fill rule as every
     * overlap that is divided by it. That matters for a self-intersecting ring,
     * which `blocks.hpp` says occurs whenever `Block::has_grade_separated_edge`
     * is set: its signed shoelace area and its even-odd area are different
     * numbers, and mixing the two put `coverage` above 1 on a real bowtie.
     */
    double target_area = 0.0;

    [[nodiscard]] bool inferred() const { return outcome == ZoneOutcome::Inferred; }
};

/**
 * @brief What the solve did, for logging and for tests
 */
struct ZoningStats {
    size_t targets = 0;         ///< Targets considered
    size_t sources = 0;         ///< Source areas considered
    size_t inferred = 0;        ///< Targets that got a zone
    size_t no_overlap = 0;      ///< Targets no source touched
    size_t below_floor = 0;     ///< Targets whose best candidate missed min_coverage
    size_t degenerate = 0;      ///< Targets whose ring is not a polygon
    size_t contested = 0;       ///< Inferred targets flagged contested

    /// Target-source pairs the solve looked at. Equals targets * sources.
    size_t pairs = 0;

    /**
     * @brief Polygon intersections actually run, after the bounding-box reject
     *
     * Strictly fewer than `pairs` on any real extract. Carried because the
     * prefilter is the only thing standing between this and an O(blocks *
     * landuse areas) pile of Clipper2 calls, and a prefilter that silently stops
     * filtering is a performance cliff nothing else would notice.
     */
    size_t clips = 0;
};

/// Per-target decisions plus the evidence that they cover every target.
struct ZoningReport {
    /// Parallel to the targets passed in: `inferences[i]` is `targets[i]`'s.
    std::vector<ZoneInference> inferences;

    ZoningStats stats;
};

// ============================================================================
// Inference
// ============================================================================

/**
 * @brief Decide one ring's zone. Touches no store.
 *
 * Pure, so a test can assert the decision without building an AttributeStore,
 * and so a caller can show a preview before committing anything.
 *
 * Measures the ring and nothing else. A parcel with holes -- anything that came
 * from a `Block` -- uses `infer_target_zone()` below, which subtracts them.
 *
 * @param ring    Target boundary in local metres, first point not repeated
 * @param sources Landuse areas. Order does not affect the result; see the file
 *                comment on ties.
 * @param config  Coverage floor, contested margin and the Clipper2 scale
 */
[[nodiscard]] ZoneInference infer_zone(const std::vector<glm::dvec2>& ring,
                                       const std::vector<ZoneSourceArea>& sources,
                                       const ZoningConfig& config = {});

/**
 * @brief Decide one target's zone, holes included. Touches no store.
 *
 * The same solve `infer_zones()` runs per target, exposed for one target so a
 * caller previewing a single block does not have to build a one-element vector
 * and a report -- and so the hole handling is reachable from a test without one.
 * `ZoneTarget::object` is not read.
 *
 * A second name rather than an overload of `infer_zone()`. `AttributeObject` is
 * an aggregate of two integers, so `ZoneTarget` is brace-initialisable from the
 * same `{{0, 0}, {1, 0}, {1, 1}}` a caller writes for a ring, and an overload
 * pair would be ambiguous exactly where a ring literal is most natural.
 */
[[nodiscard]] ZoneInference infer_target_zone(const ZoneTarget& target,
                                              const std::vector<ZoneSourceArea>& sources,
                                              const ZoningConfig& config = {});

/**
 * @brief Decide every target's zone. Touches no store.
 *
 * Bounding boxes of the sources are computed once and reused, so the cost is one
 * box test per pair plus one Clipper2 intersection per pair that actually
 * overlaps. `ZoningStats::clips` against `ZoningStats::pairs` is how a caller
 * sees that working.
 *
 * @return One inference per target, in the same order, plus counts.
 */
[[nodiscard]] ZoningReport infer_zones(const std::vector<ZoneTarget>& targets,
                                       const std::vector<ZoneSourceArea>& sources,
                                       const ZoningConfig& config = {});

// ============================================================================
// Applying an inference
// ============================================================================

/**
 * @brief Write every inferred zone to the Object rung, with NO undo history
 *
 * The import path. Only `ZoneInference::inferred()` decisions are written, so a
 * NoOverlap target is left genuinely unzoned rather than stamped with
 * `ZoneType::Unknown`.
 *
 * Targets whose `object` handle is invalid or stale are skipped. The User rung
 * is never touched, so this cannot destroy a painted zone -- though on the
 * import path there is nothing painted yet.
 *
 * @param store   Document store
 * @param targets The same vector handed to `infer_zones()`
 * @param report  That call's result
 * @return How many zones were written.
 */
size_t load_inferred_zones(scene::AttributeStore& store, const std::vector<ZoneTarget>& targets,
                           const ZoningReport& report);

/**
 * @brief Write every inferred zone to the Object rung as ONE undo step
 *
 * The interactive path: re-running zoning from a panel after editing streets.
 * Everything goes in a single transaction labelled "Apply zoning", so one undo
 * takes the whole re-zone back rather than one undo per block.
 *
 * Painted zones survive untouched, because these writes land on Object and a
 * painted zone lives on User. That is the property the Zoning suite pins down.
 *
 * A target whose Object rung ALREADY holds the inferred zone is skipped. That is
 * not an optimisation, it is the undo contract: `scene::set_attribute()` never
 * compares against the current value, so writing an unchanged zone still puts a
 * command in the transaction and still records a step. On the documented
 * workflow -- re-running zoning after editing one street -- almost every block is
 * unchanged, and the user would get an "Apply zoning" entry whose undo visibly
 * does nothing. Skipping leaves the transaction genuinely empty, and
 * `CommandStack::commit_transaction()` then records no step at all.
 *
 * @return How many zones CHANGED. Zero on a re-solve that decided nothing new,
 *         which is also how a caller can tell that no undo step was recorded.
 *         `load_inferred_zones()` counts differently -- it has no history to
 *         protect and writes unconditionally.
 */
size_t apply_inferred_zones(scene::CommandStack& stack, scene::AttributeStore& store,
                            const std::vector<ZoneTarget>& targets, const ZoningReport& report);

} // namespace stratum::osm::road
