/**
 * @file road_style.hpp
 * @brief The single tag-to-appearance mapping: OSM tags in, MaterialKey out
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * MaterialId is a coarse slot -- "this is carriageway", "this is kerb", "this is
 * a facade". It cannot say WHICH carriageway, and OSM knows the difference:
 * `surface=asphalt`, `surface=cobblestone` and `surface=gravel` are all driveable
 * running surfaces and three completely different-looking ones. `MaterialKey`
 * (renderer/mesh.hpp) adds that second axis as `uint16_t variant`, and THIS FILE
 * is the only place in the build that decides a variant.
 *
 * The split of responsibility is deliberate and is a contract with the renderer:
 *
 * - **Here (geometry side):** OSM tags -> MaterialKey. No textures, no colours, no
 *   PBR parameters, no file paths. This half is engine-agnostic and lives in
 *   stratum_core.
 * - **There (renderer side):** MaterialKey -> textures and PBR parameters. It
 *   never reads an OSM tag.
 *
 * Anything that wants to know what a surface looks like asks for a MaterialKey
 * here and resolves it there. Nothing else in the pipeline is allowed to invent a
 * variant number inline.
 *
 * ### Why buildings are in a road header
 *
 * They are not built by this module -- `osm/mesh_builder.cpp` extrudes them --
 * but the mapping from tags to appearance must exist exactly once, or two
 * half-tables drift apart and a brick house and a brick wall stop matching. The
 * building and amenity entry points at the bottom of this file are therefore
 * declared here alongside the road ones. Only the road entry points are called from
 * inside this module; the building ones are for `osm/mesh_builder.cpp` to call.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API. renderer/mesh.hpp is included for MaterialId and MaterialKey only; that
 * header is pure glm and is already compiled into stratum_core.
 */

#pragma once

#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Variant numbering
// ============================================================================

/**
 * @brief Named variants for each material slot
 *
 * ### The numbering rule
 *
 * Read this before adding a value. Variant numbers are written into exported
 * meshes and into saved material sets, and a saved scene outlives any build that
 * produced it.
 *
 * 1. **Zero is always the slot's default.** Every producer written before the
 *    variant axis existed leaves `SubMesh::variant` at 0, and that must keep
 *    meaning "the ordinary appearance of this slot". `kAsphaltDefault`,
 *    `kSidewalkDefault` and every other `k*Default` are 0 by definition.
 * 2. **A variant is scoped to its slot.** Variant 3 of `MaterialId::Asphalt` and
 *    variant 3 of `MaterialId::Sidewalk` are unrelated and may collide freely.
 *    The lookup key is always the PAIR; see `MaterialKey::packed()`.
 * 3. **Values are dense from 0.** Within one slot the values run
 *    `0 .. variant_count(slot) - 1` with no holes, so the renderer can hold its
 *    material library as a flat array per slot rather than a hash map. A value
 *    that is retired keeps its number and its name and is simply never emitted;
 *    the hole is never reused and never removed.
 * 4. **Append only, at the end of the slot's block.** Never renumber, never
 *    insert in the middle, never repurpose. A new variant takes the next free
 *    number in its own slot.
 * 5. **No ordering semantics beyond rule 1.** Values are assigned in the order
 *    they were introduced. Do not read "higher means rougher" or any other
 *    gradient into the numbers -- the finish grades happen to come first only
 *    because they were written first.
 *
 * ### Where a variant comes from
 *
 * Two sources, and both are legitimate:
 *
 * - **Tag-derived.** `surface=cobblestone` -> `kCobblestone`. Resolved by
 *   surface_material() and strip_material().
 * - **Use-derived.** A narrow raised median is a pedestrian refuge and is not
 *   painted like a carriageway slab, so it is `kConcreteIsland` even though no
 *   tag says so. Assigned by the builder that knows the use -- the profile
 *   builder, the crossing pass, the roundabout apron builder -- and documented
 *   per value below.
 *
 * When both apply, the tag wins, with one exception spelled out in
 * surface_material(): a smoothness grade refines only a variant that is still the
 * slot default.
 */
namespace variants {

// ----------------------------------------------------------------------------
// MaterialId::Default
// ----------------------------------------------------------------------------

/// Untagged geometry. The Default slot has no variants and never will.
inline constexpr uint16_t kDefaultOnly = 0;

// ----------------------------------------------------------------------------
// MaterialId::Asphalt -- the bound running surface of a carriageway
//
// Cobblestone, sett and paving stones live in this slot rather than in Concrete
// because they are what a vehicle drives on and they take the same lane markings,
// the same wear and the same wetness response. The slot is "running surface",
// not "the substance bitumen".
// ----------------------------------------------------------------------------

inline constexpr uint16_t kAsphaltDefault   = 0;  ///< Ordinary bituminous carriageway
inline constexpr uint16_t kAsphaltWorn      = 1;  ///< smoothness=bad or worse; cracked, patched, faded paint
inline constexpr uint16_t kAsphaltSmooth    = 2;  ///< smoothness=excellent; recently laid, dark, crisp paint
inline constexpr uint16_t kCobblestone      = 3;  ///< surface=cobblestone, unhewn_cobblestone; rounded stones
inline constexpr uint16_t kSett             = 4;  ///< surface=sett, cobblestone:flattened; squared blocks
inline constexpr uint16_t kPavingStones     = 5;  ///< surface=paving_stones, bricks; flat rectangular units
inline constexpr uint16_t kChipseal         = 6;  ///< surface=chipseal, tar, bitumen; loose aggregate dressing
inline constexpr uint16_t kAsphaltColoured  = 7;  ///< Use-derived: coloured surfacing on a bus or cycle lane

// ----------------------------------------------------------------------------
// MaterialId::Concrete -- rigid pavement, junction slabs, hard standing
// ----------------------------------------------------------------------------

inline constexpr uint16_t kConcreteDefault  = 0;  ///< Poured concrete carriageway or slab
inline constexpr uint16_t kConcreteWorn     = 1;  ///< smoothness=bad or worse; stained, spalled, jointed
inline constexpr uint16_t kConcreteSmooth   = 2;  ///< smoothness=excellent; clean, uniform
inline constexpr uint16_t kConcretePlates   = 3;  ///< surface=concrete:plates; visible slab joints
inline constexpr uint16_t kConcreteLanes    = 4;  ///< surface=concrete:lanes; two wheel tracks, gap between
inline constexpr uint16_t kConcreteIsland   = 5;  ///< Use-derived: top of a narrow raised median or refuge

// ----------------------------------------------------------------------------
// MaterialId::Curb -- kerb face and kerb top
//
// Face and top share one slot and therefore one texture, authored with the face
// on one side and the top on the other; see corridor.hpp's UV convention. A
// variant changes both together, which is correct -- a granite kerb is granite on
// both surfaces.
// ----------------------------------------------------------------------------

inline constexpr uint16_t kCurbDefault      = 0;  ///< Precast concrete kerb
inline constexpr uint16_t kCurbGranite      = 1;  ///< kerb:material or material = granite, stone
inline constexpr uint16_t kCurbAsphalt      = 2;  ///< Bituminous or rolled kerb, no distinct unit
inline constexpr uint16_t kCurbDropped      = 3;  ///< Use-derived: lowered at a crossing or driveway mouth
inline constexpr uint16_t kCurbMountable    = 4;  ///< Use-derived: splayed apron kerb, roundabout centre

// ----------------------------------------------------------------------------
// MaterialId::Sidewalk -- footway surface
// ----------------------------------------------------------------------------

inline constexpr uint16_t kSidewalkDefault  = 0;  ///< Ordinary footway; concrete or asphalt, unremarkable
inline constexpr uint16_t kSidewalkPaved    = 1;  ///< surface=paving_stones; laid rectangular units
inline constexpr uint16_t kSidewalkTactile  = 2;  ///< Use-derived: tactile paving at a dropped kerb
inline constexpr uint16_t kSidewalkAsphalt  = 3;  ///< surface=asphalt footway
inline constexpr uint16_t kSidewalkConcrete = 4;  ///< surface=concrete; poured, jointed slabs
inline constexpr uint16_t kSidewalkBrick    = 5;  ///< surface=bricks, sett, cobblestone

// ----------------------------------------------------------------------------
// MaterialId::Markings -- painted paint
//
// The SPRITE is chosen by the marking atlas through UVs (marking_atlas.hpp), not
// by the variant. A variant here changes the PAINT, that is its colour and its
// condition, across every sprite drawn from the same range.
// ----------------------------------------------------------------------------

inline constexpr uint16_t kMarkingsDefault  = 0;  ///< White thermoplastic
inline constexpr uint16_t kMarkingsYellow   = 1;  ///< Use-derived: yellow paint (box junction, bus lane, no-waiting)
inline constexpr uint16_t kMarkingsWorn     = 2;  ///< Use-derived: faded, on a carriageway resolved as kAsphaltWorn

// ----------------------------------------------------------------------------
// MaterialId::Gravel -- loose or compacted mineral surface
// ----------------------------------------------------------------------------

inline constexpr uint16_t kGravelDefault    = 0;  ///< surface=gravel; loose stone
inline constexpr uint16_t kGravelCompacted  = 1;  ///< surface=compacted; rolled hardcore, driveable
inline constexpr uint16_t kGravelFine       = 2;  ///< surface=fine_gravel; small even aggregate
inline constexpr uint16_t kGravelPebble     = 3;  ///< surface=pebblestone; rounded loose pebbles
inline constexpr uint16_t kGravelUnpaved    = 4;  ///< surface=unpaved; unspecified, assume worn mixed stone

// ----------------------------------------------------------------------------
// MaterialId::Dirt -- unbound earth surface
// ----------------------------------------------------------------------------

inline constexpr uint16_t kDirtDefault      = 0;  ///< Unspecified bare earth
inline constexpr uint16_t kDirtGround       = 1;  ///< surface=ground; earth with patchy vegetation
inline constexpr uint16_t kDirtEarth        = 2;  ///< surface=dirt, earth, soil; bare worked earth
inline constexpr uint16_t kDirtMud          = 3;  ///< surface=mud; wet, rutted, dark
inline constexpr uint16_t kDirtSand         = 4;  ///< surface=sand; loose, pale

// ----------------------------------------------------------------------------
// MaterialId::Grass -- verges, medians and planting
// ----------------------------------------------------------------------------

inline constexpr uint16_t kGrassDefault     = 0;  ///< Verge grass, unspecified
inline constexpr uint16_t kGrassMown        = 1;  ///< Use-derived: maintained urban verge, short and even
inline constexpr uint16_t kGrassRough       = 2;  ///< Use-derived: rural verge, long and mixed
inline constexpr uint16_t kGrassPaver       = 3;  ///< surface=grass_paver; reinforced grass grid
inline constexpr uint16_t kGrassPlanted     = 4;  ///< Use-derived: shrubs on a wide central reservation

// ----------------------------------------------------------------------------
// MaterialId::BridgeDeck
// ----------------------------------------------------------------------------

inline constexpr uint16_t kBridgeDeckDefault  = 0;  ///< Concrete deck slab
inline constexpr uint16_t kBridgeDeckSteel    = 1;  ///< surface=metal; steel plate or grating
inline constexpr uint16_t kBridgeDeckWood     = 2;  ///< surface=wood; timber decking
inline constexpr uint16_t kBridgeDeckStone    = 3;  ///< Masonry arch deck

// ----------------------------------------------------------------------------
// MaterialId::Parapet
// ----------------------------------------------------------------------------

inline constexpr uint16_t kParapetDefault   = 0;  ///< Concrete parapet
inline constexpr uint16_t kParapetSteel     = 1;  ///< Steel railing or barrier solid
inline constexpr uint16_t kParapetStone     = 2;  ///< Masonry parapet, matching a stone arch
inline constexpr uint16_t kParapetConcrete  = 3;  ///< Explicitly concrete where the default is overridden

// ----------------------------------------------------------------------------
// MaterialId::Wall -- building facade
// ----------------------------------------------------------------------------

inline constexpr uint16_t kWallDefault      = 0;  ///< Unremarkable painted or rendered masonry
inline constexpr uint16_t kWallBrick        = 1;  ///< building:material=brick
inline constexpr uint16_t kWallStone        = 2;  ///< building:material=stone, granite, limestone, sandstone
inline constexpr uint16_t kWallConcrete     = 3;  ///< building:material=concrete; exposed panels
inline constexpr uint16_t kWallRender       = 4;  ///< building:material=plaster, render, stucco
inline constexpr uint16_t kWallGlass        = 5;  ///< building:material=glass; curtain wall
inline constexpr uint16_t kWallMetal        = 6;  ///< building:material=metal; profiled cladding
inline constexpr uint16_t kWallWood         = 7;  ///< building:material=wood, timber

// ----------------------------------------------------------------------------
// MaterialId::Roof -- building roof surface
// ----------------------------------------------------------------------------

inline constexpr uint16_t kRoofDefault      = 0;  ///< Unspecified roof
inline constexpr uint16_t kRoofTile         = 1;  ///< roof:material=roof_tiles, tiles; pitched clay or concrete
inline constexpr uint16_t kRoofSlate        = 2;  ///< roof:material=slate
inline constexpr uint16_t kRoofMetal        = 3;  ///< roof:material=metal, copper, zinc
inline constexpr uint16_t kRoofMembrane     = 4;  ///< Flat roof: bitumen felt, gravel ballast, single ply
inline constexpr uint16_t kRoofConcrete     = 5;  ///< roof:material=concrete; exposed slab
inline constexpr uint16_t kRoofThatch       = 6;  ///< roof:material=thatch
inline constexpr uint16_t kRoofGlass        = 7;  ///< roof:material=glass; atrium or conservatory

} // namespace variants

/**
 * @brief Number of variants defined for a slot
 *
 * Exactly the count guaranteed dense by rule 3 of the numbering rule: valid
 * variants of @p slot are `0 .. variant_count(slot) - 1`. The renderer sizes its
 * per-slot material array with this; the editor sizes its variant dropdown with
 * it.
 *
 * @param slot Material slot to measure
 * @return Number of defined variants; 1 for a slot with only a default, and 0 for
 *         MaterialId::Count and out-of-range values
 */
[[nodiscard]] size_t variant_count(MaterialId slot);

// ============================================================================
// Roads
// ============================================================================

/**
 * @brief Resolve a `surface=*` value ALONE, with no road-class fallback
 *
 * The same table surface_material() consults in its step 1, exposed so that a
 * caller with its own fallback can share the one table instead of keeping a
 * second copy of it. road_profile.cpp is that caller: its class rules give a
 * footway MaterialId::Sidewalk and a track MaterialId::Dirt where an untagged
 * carriageway gets Asphalt, which surface_material()'s step 2 does not
 * reproduce, so it needs to know whether the TAG resolved at all.
 *
 * Keeping the slot decision in one place is not a tidiness point. When the two
 * tables disagreed, the profile builder put a cobbled carriageway in the
 * Sidewalk slot while this file's policy put it in Asphalt, so strip_material()
 * -- which may never change the slot it is given -- resolved it to the footway's
 * brick variant, the carriageway merged into one draw range with the pavement
 * beside it, and variants::kCobblestone, kSett and kPavingStones could not be
 * emitted by any way in any extract.
 *
 * @param surface Lowercased `surface=*` value; empty when the tag is absent
 * @param out     Receives slot and variant on a hit; UNTOUCHED on a miss
 * @return True when @p surface named a running surface the table knows
 */
[[nodiscard]] bool surface_material_from_tag(const std::string& surface, MaterialKey& out);

/**
 * @brief Resolve the running-surface material of a carriageway from OSM tags
 *
 * The one function that turns `surface=*` into a look. Everything that paints a
 * driveable surface goes through it, so a road, a junction fill and a bridge deck
 * approach never disagree about what the same way is made of.
 *
 * ### Resolution order
 *
 * 1. **@p surface decides the slot AND the variant** when it is a value this
 *    table knows. `gravel` resolves to `{Gravel, kGravelDefault}` -- note that
 *    the SLOT changes, not just the variant, because a gravel road is not asphalt
 *    with a different texture.
 * 2. **@p type decides both** when @p surface is empty or unrecognised. Every
 *    sealed road class resolves to `{Asphalt, kAsphaltDefault}`;
 *    RoadType::Path resolves to `{Dirt, kDirtGround}`, because a path or track
 *    with no surface tag is not sealed.
 * 3. **@p smoothness refines the variant, and only then.** It is applied last,
 *    and ONLY when the variant resolved so far is still the slot default, and
 *    ONLY on a slot that defines worn and smooth grades (Asphalt and Concrete).
 *    `surface=cobblestone` with `smoothness=bad` stays `kCobblestone`: the
 *    cobbles are the reason it is rough, and replacing them with worn asphalt
 *    would lose the actual surface.
 *
 * @param type       Road class, for the default when no surface tag resolves
 * @param surface    Raw `surface=*` value, lowercased. May be empty.
 * @param smoothness Raw `smoothness=*` value, lowercased. May be empty; picks
 *                   worn (`bad`, `very_bad`, `horrible`, `very_horrible`,
 *                   `impassable`) versus smooth (`excellent`). `good` and
 *                   `intermediate` leave the default alone.
 * @return The running-surface key. Never invalid: an unrecognised @p surface
 *         falls through to the class default rather than failing.
 *
 * @note Both string parameters are expected ALREADY LOWERCASED and trimmed. This
 *       function does not normalise, because the parser already holds the tag in
 *       a normalised form and doing it twice per way on a 63 MB extract is
 *       measurable.
 * @note A multi-value tag (`surface=asphalt;gravel`) is not split. It does not
 *       match, so the class default applies.
 * @note The slots this can return are the RUNNING-SURFACE ones only: Asphalt,
 *       Concrete, Gravel, Dirt and Grass. `surface=metal` and `surface=wood` are
 *       real OSM values, but they describe a bridge DECK rather than a
 *       carriageway, and resolving them here would move an ordinary road into the
 *       BridgeDeck SubMesh range. They are reached through strip_material() with
 *       a slot of MaterialId::BridgeDeck instead, which is where the structure
 *       builder already knows it is looking at a deck.
 */
[[nodiscard]] MaterialKey surface_material(RoadType type,
                                           const std::string& surface,
                                           const std::string& smoothness);

/**
 * @brief Resolve the material of one non-carriageway strip
 *
 * So that a granite kerb and a concrete kerb differ, a paving-stone footway and a
 * poured-concrete one differ, and a refuge island is not painted like a
 * carriageway slab.
 *
 * ### The slot is not negotiable
 *
 * @p slot is what the profile builder already decided this strip is, and this
 * function NEVER changes it. If @p surface names a substance belonging to another
 * slot -- `surface=gravel` on a strip the profile made `MaterialId::Sidewalk` --
 * the nearest variant within @p slot is chosen, or the slot default when there is
 * no near match. Returning a different slot would move the strip into a different
 * SubMesh range than the one the extruder opened for it.
 *
 * ### Composition with use-derived variants
 *
 * This function reads TAGS only. A variant marked "use-derived" in the variants
 * namespace -- `kCurbDropped`, `kSidewalkTactile`, `kGrassMown`,
 * `kConcreteIsland` -- is assigned by the builder that knows the use. The
 * composition rule is: call this first, and substitute the use-derived variant
 * ONLY when it came back as the slot default. A tag that named a real substance
 * always beats an inference.
 *
 * The one exception is @p kind, which this function does read, because a strip's
 * kind is structural rather than inferred: StripKind::Median on
 * MaterialId::Concrete is `kConcreteIsland` outright, since the profile builder
 * only ever emits that pairing for a raised refuge.
 *
 * @param kind    What the strip is, from the profile
 * @param slot    Material slot the profile assigned. Authoritative; see above.
 * @param surface Raw `surface=*` value of the parent way, lowercased. May be
 *                empty. Note this is the CARRIAGEWAY's surface tag: OSM rarely
 *                tags a kerb or a verge separately, so it is weak evidence and is
 *                only consulted for kinds where it plausibly applies.
 * @param tags    Optional raw way tags, for the narrow keys that are not promoted
 *                onto GraphEdge -- `kerb:material`, `material`, `footway:surface`,
 *                `sidewalk:both:surface`. May be null.
 * @return The strip's key. `slot` is always preserved.
 */
[[nodiscard]] MaterialKey strip_material(StripKind kind,
                                         MaterialId slot,
                                         const std::string& surface,
                                         const TagMap* tags);

// ============================================================================
// Naming and enumeration
// ============================================================================

/**
 * @brief Human-readable, globally unique name for a (slot, variant)
 *
 * For editor material lists and for exported material names, which is why it is
 * unique across slots and carries no spaces: `road_export.cpp` writes it into an
 * OBJ `usemtl` line and a glTF material name.
 *
 * Format:
 *
 * - Variant 0: the slot name alone -- `"Asphalt"`, `"Sidewalk"`, `"Wall"`. This
 *   is deliberate, so every mesh produced before the variant axis existed keeps
 *   exactly the name it had.
 * - Non-zero: `"<Slot>.<Variant>"` -- `"Asphalt.Cobblestone"`,
 *   `"Sidewalk.Tactile"`, `"Curb.Granite"`. The variant part is the constant's
 *   name with its `k` prefix and its redundant slot prefix removed.
 * - Out of range: `"<Slot>.Variant<N>"` -- `"Asphalt.Variant97"`. A mesh saved by
 *   a newer build and opened by an older one still names its materials
 *   distinguishably instead of collapsing them all onto the default.
 * - Slot out of range: `"Unknown"` when the variant is 0, `"Unknown.VariantN"`
 *   otherwise.
 *
 * These names are FROZEN. They travel in exported files, so renaming one silently
 * breaks every material assignment a user has already made in their engine.
 *
 * @param key Slot and variant to name
 * @return The name; never empty
 */
[[nodiscard]] std::string material_key_name(MaterialKey key);

/**
 * @brief Every material key this build can emit
 *
 * So the renderer can pre-build a complete material library at startup without
 * waiting to see which keys an extract happens to produce, and so the editor can
 * list every slot and variant whether or not the loaded scene uses it.
 *
 * Contents: every variant of every slot from `MaterialId::Default` through the
 * last slot before `MaterialId::Count`, that is,
 * `sum over slots of variant_count(slot)` entries. It is a static enumeration of
 * what is DECLARED, not a survey of what a particular network built -- a key
 * appears here even if no way in the world carries the tag that produces it.
 *
 * @return Every key, sorted ascending by `MaterialKey::packed()`, which orders by
 *         slot first and then by variant. No duplicates.
 */
[[nodiscard]] std::vector<MaterialKey> all_material_keys();

// ============================================================================
// Buildings
// ============================================================================
//
// Buildings are extruded by osm/mesh_builder.cpp, not by this module. Their
// mapping lives here anyway so that there is exactly ONE table from OSM tags to
// appearance in the whole build; see the file header.
//
// The two functions below are the only sanctioned source of a Wall or Roof
// variant. mesh_builder.cpp is expected to call them and tag its SubMesh ranges
// with the result, exactly as the corridor extruder does.
// ============================================================================

/**
 * @brief Resolve a building facade material from its type and tags
 *
 * ### Resolution order
 *
 * 1. `building:material=*` when present and recognised: `brick` / `brick_block`
 *    -> `kWallBrick`; `stone` / `granite` / `limestone` / `sandstone` ->
 *    `kWallStone`; `concrete` -> `kWallConcrete`; `plaster` / `render` /
 *    `stucco` -> `kWallRender`; `glass` -> `kWallGlass`; `metal` -> `kWallMetal`;
 *    `wood` / `timber` -> `kWallWood`.
 * 2. Otherwise the class default from @p type: House, Detached, Residential,
 *    Apartments, School and Hospital -> `kWallBrick`; Commercial, Office and
 *    Retail -> `kWallGlass`; Industrial, Warehouse, Garage and Shed ->
 *    `kWallMetal`; Church -> `kWallStone`; Unknown -> `kWallDefault`.
 *
 * `building:colour` is NOT read. Colour is a renderer-side tint over a resolved
 * material and does not belong on the variant axis, which is finite and dense.
 *
 * @param type Building class from `building=*`
 * @param tags Optional raw way tags. May be null, in which case only @p type is used.
 * @return A key whose slot is always MaterialId::Wall
 */
[[nodiscard]] MaterialKey building_wall_material(BuildingType type, const TagMap* tags);

/**
 * @brief Resolve a building roof material from its shape and tags
 *
 * ### Resolution order
 *
 * 1. `roof:material=*` when present and recognised: `roof_tiles` / `tiles` /
 *    `tile` -> `kRoofTile`; `slate` -> `kRoofSlate`; `metal` / `copper` / `zinc`
 *    -> `kRoofMetal`; `tar_paper` / `bitumen` / `gravel` -> `kRoofMembrane`;
 *    `concrete` -> `kRoofConcrete`; `thatch` -> `kRoofThatch`; `glass` ->
 *    `kRoofGlass`.
 * 2. Otherwise the shape default from @p roof: Flat -> `kRoofMembrane`, because a
 *    flat roof is a waterproofing layer and never tiles; Gabled, Hipped,
 *    Pyramidal and Skillion -> `kRoofTile`, the pitched default; Dome ->
 *    `kRoofMetal`; Unknown -> `kRoofDefault`.
 *
 * `roof:colour` is not read, for the same reason `building:colour` is not.
 *
 * @param roof Roof shape from `roof:shape=*`
 * @param tags Optional raw way tags. May be null, in which case only @p roof is used.
 * @return A key whose slot is always MaterialId::Roof
 */
[[nodiscard]] MaterialKey building_roof_material(RoofType roof, const TagMap* tags);

// ============================================================================
// Amenities -- forward hook
// ============================================================================

/**
 * @brief Stable style identifiers for street furniture and amenities
 *
 * ### This is a FORWARD HOOK, not a feature
 *
 * Nothing in this branch builds a bench, a lamp post or a bus shelter. Prop
 * placement needs an asset system -- a catalogue of models, an instancing path,
 * and a placement pass over OSM nodes -- and none of that exists yet.
 *
 * What DOES exist is the mapping problem, and it is the same one this file
 * already solves for surfaces: a tag on the left, a stable identifier on the
 * right. Putting the identifiers here now means the placement pass, whenever it
 * is written, reads its table from the one place every other tag mapping lives,
 * rather than growing a second one that drifts.
 *
 * These ids obey the same numbering rule as the material variants: 0 is "none",
 * values are dense, and they are append-only because they will end up in saved
 * scenes the moment anything consumes them.
 *
 * They are NOT MaterialKey variants. A prop is a model, not a surface, so it has
 * no slot to belong to.
 */
namespace amenity_styles {

inline constexpr uint16_t kNone              = 0;   ///< Not a recognised amenity; place nothing
inline constexpr uint16_t kBench             = 1;   ///< amenity=bench
inline constexpr uint16_t kWasteBasket       = 2;   ///< amenity=waste_basket
inline constexpr uint16_t kStreetLamp        = 3;   ///< highway=street_lamp
inline constexpr uint16_t kTrafficSignal     = 4;   ///< highway=traffic_signals
inline constexpr uint16_t kStopSign          = 5;   ///< highway=stop
inline constexpr uint16_t kGiveWaySign       = 6;   ///< highway=give_way
inline constexpr uint16_t kBollard           = 7;   ///< barrier=bollard
inline constexpr uint16_t kGate              = 8;   ///< barrier=gate, lift_gate
inline constexpr uint16_t kBusStop           = 9;   ///< highway=bus_stop, public_transport=platform
inline constexpr uint16_t kShelter           = 10;  ///< amenity=shelter
inline constexpr uint16_t kPostBox           = 11;  ///< amenity=post_box
inline constexpr uint16_t kTelephone         = 12;  ///< amenity=telephone
inline constexpr uint16_t kDrinkingWater     = 13;  ///< amenity=drinking_water
inline constexpr uint16_t kBicycleParking    = 14;  ///< amenity=bicycle_parking
inline constexpr uint16_t kParkingMeter      = 15;  ///< amenity=vending_machine + vending=parking_tickets
inline constexpr uint16_t kFireHydrant       = 16;  ///< emergency=fire_hydrant
inline constexpr uint16_t kTree              = 17;  ///< natural=tree
inline constexpr uint16_t kPlanter           = 18;  ///< amenity=planter
inline constexpr uint16_t kFountain          = 19;  ///< amenity=fountain
inline constexpr uint16_t kPicnicTable       = 20;  ///< leisure=picnic_table
inline constexpr uint16_t kStreetCabinet     = 21;  ///< man_made=street_cabinet
inline constexpr uint16_t kUtilityPole       = 22;  ///< power=pole, man_made=utility_pole

} // namespace amenity_styles

/**
 * @brief Classify an OSM node's tags into an amenity style id
 *
 * The forward hook described in the amenity_styles namespace. Model and prop
 * selection is NOT geometry this branch builds; this returns a stable id that an
 * asset system resolves to a model later, and returns
 * `amenity_styles::kNone` for anything it does not recognise.
 *
 * Matching is by the first key that hits, in the order the amenity_styles values
 * are declared, so a node carrying both `amenity=shelter` and
 * `highway=bus_stop` resolves as the bus stop. A node with no recognised key
 * resolves to kNone, which is by far the common case: most OSM nodes are geometry
 * carriers with no tags at all, and this is called per node.
 *
 * @param tags Raw node tags. May be null, which returns kNone.
 * @return A value from the amenity_styles namespace; kNone when unrecognised
 */
[[nodiscard]] uint16_t amenity_style_id(const TagMap* tags);

} // namespace stratum::osm::road
