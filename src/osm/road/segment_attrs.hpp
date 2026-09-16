// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file segment_attrs.hpp
 * @brief Editing a segment's tags through the command stack, and saying what that breaks
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### What this file is, and what it is not
 *
 * Every field it edits already exists. SegmentAttributes in graph_edit.hpp
 * mirrors the tag-derived fields on osm::Road and on GraphEdge, and
 * EditableGraph already stores one per segment. Nothing here models anything
 * new. What was missing is the three things an editor needs on top of a field:
 *
 *   1. **A command per logical edit**, so a change is undoable and so a gesture
 *      -- a spinner held down, a name typed in -- collapses into ONE undo step
 *      instead of forty.
 *   2. **Validation that refuses**, so a lane count of 0 or a width of 0 never
 *      reaches the profile builder. `apply()` returning false makes
 *      CommandStack::execute() record nothing, which is exactly what a refusal
 *      should leave behind: the history it started with.
 *   3. **A statement of what the edit invalidated.** Changing a width changes
 *      the corridor and the junction at each end. Changing a name changes a
 *      label. B4's incremental re-solve is only possible if the difference is
 *      recorded at the moment of the edit, by the code that knows which field
 *      moved; recovering it afterwards means diffing two solves, which costs
 *      more than the solve it was meant to skip.
 *
 * ### The mutation is a replacement, and that is deliberate
 *
 * EditableGraph's mutating API is private with GraphEditCommand as its only
 * friend, and it has no `set_segment_attributes()`. Rather than widen that wall,
 * an edit here removes the segment and puts back a copy of it carrying the new
 * attributes. `insert_segment()` takes the handle FROM the record it is given
 * -- that is how DeleteSegmentCommand's undo restores a segment rather than
 * creating a lookalike -- so the SegmentId, the `source_way` and the whole
 * `node_ids` list survive the round trip unchanged. No caller's handle goes
 * stale, and no geometry moves.
 *
 * One consequence has to be stated rather than discovered. `erase_segment()`
 * followed by `insert_segment()` marks every node of the segment dirty, and
 * marking a node marks every OTHER segment on it too, so after a pure rename
 * EditableGraph::dirty_segments() names the neighbours as well. That set is
 * therefore CONSERVATIVE here: it over-reports, never under-reports, so a
 * consumer that trusts it re-solves too much and is never wrong. The precise
 * answer is `SegmentAttributeCommand::invalidation()`, which is what B4 should
 * read. The alternative -- a narrower mutator on EditableGraph that touched
 * only the attributes -- is a better long-term shape and is a change to
 * graph_edit.hpp, so it is not made here.
 *
 * ### Refusing rather than clamping
 *
 * A clamp is a silent edit. Type 0 into a lane field, get 1, and the number in
 * the box is not the number you typed; worse, the undo step records a change
 * nobody asked for. Every rule below therefore refuses, and `refusal()` says
 * which rule fired so the UI can put the reason next to the field.
 *
 * An edit is judged on what it INTRODUCES, not on the state it leaves. An
 * extract can contain a way already tagged `lanes=0`, and if the test were
 * "is the result valid" then that one bad field would freeze every other field
 * on the segment: the name could not be fixed until the lane count was. So the
 * error mask of the result has the error mask of the original subtracted from
 * it, and only what is left refuses the edit. A width of 0 is still refused on a
 * segment whose lane count is already broken, because the two are different bits
 * -- which is why validate_attributes() returns a mask and not a first-error.
 *
 * ### SideFlags::Unknown is not SideFlags::None
 *
 * Unknown means the tag was ABSENT, and build_profile() may then infer a
 * sidewalk from the road class. None means the tag said `sidewalk=no`, and
 * nothing may be inferred over it (road_profile.hpp states the same rule from
 * the consuming side). They are different edits, they produce different
 * cross-sections, and an inspector that offers one "off" state destroys the
 * difference on the first save. SetSegmentSideCommand treats them as the two
 * distinct values they are: setting Unknown over None is a real edit, and so is
 * the reverse.
 *
 * ### Merging, and where a gesture ends
 *
 * Every command here merges with a later edit of the SAME field of the SAME
 * segment, so holding a spinner is one step. It never merges across fields or
 * across segments. The end of a gesture is the caller's to declare, with
 * CommandStack::seal() on mouse-up or focus loss -- merging on a clock would
 * make the history depend on machine speed, as command.hpp explains.
 *
 * A merge that would leave the command a no-op is refused, which is the fix A1
 * needed in RenameLayerCommand and MoveNodeCommand needed after it. Drag a width
 * from 6 to 9 and back to 6: absorbing the return leg leaves a step whose undo
 * does nothing visible and whose redo refuses, and CommandStack::redo() drops a
 * refused step -- taking the rest of the redo branch with it.
 *
 * ### Not covered
 *
 * `osm::Road::smoothness` has no counterpart on SegmentAttributes, so there is
 * nothing here to edit; adding it is a change to graph_edit.hpp. Re-solving is
 * B4's: this file marks, and never rebuilds.
 */

#pragma once

#include "osm/road/graph_edit.hpp"
#include "osm/types.hpp"
#include "scene/command.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace stratum::osm::road {

// ============================================================================
// What an edit invalidates
// ============================================================================

/**
 * @brief What must be recomputed after an edit, as a flag set
 *
 * The flags name work, not implication. Corridor does NOT imply Appearance:
 * whoever rebuilds a corridor re-runs the marking and material pass over the
 * new profile as part of that rebuild, so Appearance carries information only
 * when Corridor is CLEAR -- there it is the flag that says a re-skin alone is
 * enough and no vertex needs to move.
 *
 * NO field is Appearance-only today, and it is worth saying why the flag is
 * still here. The design nominated Surface for that role and was wrong: a
 * surface edit can grow or drop a verge, which moves vertices (see
 * invalidation_of()). Surface therefore sets Appearance ALONGSIDE Corridor,
 * which is the difference between "re-skin as well" and "re-skin instead". The
 * first field that will set it alone is a smoothness or style field -- one that
 * chooses a material and touches no width -- and SegmentAttributes has no such
 * field yet.
 *
 * Metadata is the floor and is set by every field, because every field reaches
 * the export.
 */
enum class SegmentInvalidation : uint32_t {
    Nothing = 0,

    /// Export, labels and the inspector. Nothing to re-solve and nothing to redraw.
    Metadata = 1u << 0,

    /// The marking and material pass has to run again. On its own -- with
    /// Corridor clear -- it also says the existing vertices are still correct.
    Appearance = 1u << 1,

    /// This segment's own carriageway mesh: its profile, and the sweep along it.
    Corridor = 1u << 2,

    /**
     * @brief The junction solve at BOTH endpoints
     *
     * Not just this segment's. A junction's trims and fillet ring are measured
     * from every arm's RoadProfile -- junction_trim.cpp fills each arm's
     * half_width from `RoadProfile::total_width() * 0.5` -- so widening one arm
     * moves the geometry of all of them. An invalidation that named only the
     * edited segment would leave its neighbours ending at a fillet that no
     * longer exists.
     */
    Junctions = 1u << 3,

    /// Bridge deck or tunnel bore. Set by the fields that decide a structure exists.
    Structures = 1u << 4,
};

[[nodiscard]] constexpr SegmentInvalidation operator|(SegmentInvalidation a,
                                                      SegmentInvalidation b) {
    return static_cast<SegmentInvalidation>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

[[nodiscard]] constexpr SegmentInvalidation operator&(SegmentInvalidation a,
                                                      SegmentInvalidation b) {
    return static_cast<SegmentInvalidation>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

/**
 * @brief Is @p flag set in @p set?
 *
 * `has(x, SegmentInvalidation::Nothing)` is false for every x, since Nothing is
 * the empty set. Compare against Nothing directly to ask "is anything set".
 */
[[nodiscard]] constexpr bool has(SegmentInvalidation set, SegmentInvalidation flag) {
    return (static_cast<uint32_t>(set) & static_cast<uint32_t>(flag)) != 0u;
}

// ============================================================================
// Why an edit was refused
// ============================================================================

/**
 * @brief Every reason apply() can say no, as a flag set
 *
 * A mask rather than one code, so an edit can be judged on the errors it ADDS
 * to a segment that was already imperfect. See the file comment.
 *
 * UnknownSegment and NoChange are refusals but not attribute errors:
 * validate_attributes() never returns either.
 */
enum class SegmentRefusal : uint32_t {
    None = 0,

    /// lanes < 1. Zero running lanes is not a road, and the profile builder
    /// silently substitutes max(1, lanes), which would hide the bad value.
    LanesBelowOne = 1u << 0,

    /// width <= 0, or a width that is not finite. A zero-width carriageway
    /// degenerates every offset the corridor and the trim solve take from it.
    WidthNotPositive = 1u << 1,

    /// lanes:forward or lanes:backward below -1. -1 is the one negative with a
    /// meaning: "not specified". Anything else is a typo.
    DirectionalLanesNegative = 1u << 2,

    /// The specified directional lanes total more than lanes=*. OSM's own
    /// identity is lanes = forward + backward + both_ways, so a pair exceeding
    /// the total cannot be satisfied by any both_ways value.
    DirectionalLanesExceedTotal = 1u << 3,

    /// The segment is gone, or was never in this graph.
    UnknownSegment = 1u << 4,

    /// The edit sets the field to what it already holds. Refused so that the
    /// undo menu never fills with steps that do nothing when picked.
    NoChange = 1u << 5,
};

[[nodiscard]] constexpr SegmentRefusal operator|(SegmentRefusal a, SegmentRefusal b) {
    return static_cast<SegmentRefusal>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

[[nodiscard]] constexpr SegmentRefusal operator&(SegmentRefusal a, SegmentRefusal b) {
    return static_cast<SegmentRefusal>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

[[nodiscard]] constexpr SegmentRefusal operator~(SegmentRefusal a) {
    return static_cast<SegmentRefusal>(~static_cast<uint32_t>(a));
}

/// Is @p flag set in @p set? False for SegmentRefusal::None; compare directly for that.
[[nodiscard]] constexpr bool has(SegmentRefusal set, SegmentRefusal flag) {
    return (static_cast<uint32_t>(set) & static_cast<uint32_t>(flag)) != 0u;
}

/**
 * @brief Every attribute rule @p attributes breaks, as a mask
 *
 * Pure: it looks at one struct and nothing else. Returns None for a set that
 * breaks no rule, which is the common case and the one worth being fast.
 *
 * Only the four attribute bits can be returned. UnknownSegment and NoChange are
 * decided against the graph, not against the values, so they belong to apply().
 */
[[nodiscard]] SegmentRefusal validate_attributes(const SegmentAttributes& attributes);

/// Human-readable list of the set bits, comma separated, for a log or a tooltip.
/// "none" when nothing is set, so the string is never empty.
[[nodiscard]] std::string describe_refusal(SegmentRefusal refusal);

// ============================================================================
// Fields
// ============================================================================

/**
 * @brief One editable field of SegmentAttributes
 *
 * The merge key and the invalidation key, both. Two commands coalesce only when
 * their field matches, so a rename cannot absorb a width drag.
 *
 * The lane triple -- lanes, lanes:forward, lanes:backward -- is ONE field,
 * because its rule is a rule about all three at once. Editing them one at a time
 * through separate commands means a legal end state is unreachable whenever the
 * path to it passes through an illegal one (lowering a total below a forward
 * count that is about to be lowered too), and refusing half a gesture is worse
 * than not offering it.
 *
 * Count is a sentinel for iteration. It is never a valid field.
 */
enum class SegmentField : uint8_t {
    Type,       ///< SegmentAttributes::type
    Layer,      ///< SegmentAttributes::layer
    Width,      ///< SegmentAttributes::width
    Lanes,      ///< lanes, lanes_forward and lanes_backward together
    Oneway,     ///< SegmentAttributes::is_oneway
    Bridge,     ///< SegmentAttributes::is_bridge
    Tunnel,     ///< SegmentAttributes::is_tunnel
    Roundabout, ///< SegmentAttributes::is_roundabout
    Link,       ///< SegmentAttributes::is_link
    Sidewalk,   ///< SegmentAttributes::sidewalk
    Cycleway,   ///< SegmentAttributes::cycleway
    Parking,    ///< SegmentAttributes::parking
    Shoulder,   ///< SegmentAttributes::shoulder
    Surface,    ///< SegmentAttributes::surface
    Name,       ///< SegmentAttributes::name
    Count       ///< Sentinel: number of fields. Not a valid field.
};

/// Stable identifier for logs and tests, e.g. "Width". "Unknown" for Count and
/// for a value outside the enum, so an unmapped field is visible rather than silent.
[[nodiscard]] const char* field_name(SegmentField field);

/// Lower-case phrase for the undo menu, e.g. "width" in "Set width".
[[nodiscard]] const char* field_label(SegmentField field);

/**
 * @brief What editing @p field invalidates
 *
 * The table that makes B4 possible, and the reasoning behind it:
 *
 * - Name alone moves nothing: it reaches the export and the labels only.
 * - Every other field is read by build_profile() (road_profile.hpp lists the
 *   fields it takes off the edge), so it changes the cross-section, so it
 *   changes both the corridor sweep and every junction the segment is an arm of.
 * - Surface is one of those fields, and this table wrongly exempted it once. It
 *   picks a material variant through road_style.hpp, which is the visible half;
 *   it also decides `unpaved`, and an unpaved carriageway grows a Verge on every
 *   side that has neither a kerb nor a shoulder (road_profile.hpp states that
 *   rule from the consuming side). Paved-to-unpaved therefore changes
 *   total_width(), which is the one number junction_trim.cpp halves into every
 *   arm's half-width. A surface edit taken as a re-skin leaves a stale corridor
 *   and stale fillets on screen. It returns Appearance too, so the material pass
 *   is still named for a consumer that wants to run only that part.
 * - Layer, Bridge and Tunnel additionally decide whether a deck or a bore
 *   exists, and where in the vertical stack it sits.
 *
 * The table is per FIELD and not per edit, and Surface is where that costs
 * something: asphalt to concrete moves nothing, and this still asks for a
 * re-solve. The alternative was an invalidation computed from before() and
 * after() -- exact, and it would answer Appearance alone for that pair. It lost
 * on two counts. It cannot be read before execute(), which is where a caller
 * that wants to know the price of an edge case asks; and it makes the answer
 * depend on surface_is_unpaved()'s vocabulary, so a surface the table does not
 * recognise yet would be classified as "no geometry change" by a function whose
 * whole job is to be conservative. Over-reporting costs a re-solve. The other
 * direction costs a stale corridor on screen.
 *
 * Returns Nothing for Count and for an out-of-range value; every real field
 * returns at least Metadata.
 */
[[nodiscard]] SegmentInvalidation invalidation_of(SegmentField field);

// ============================================================================
// Values
// ============================================================================

/**
 * @brief The lane triple, edited as one value
 *
 * @see SegmentField::Lanes for why the three travel together.
 */
struct LaneCounts {
    int total = 2;      ///< lanes=*. At least 1.
    int forward = -1;   ///< lanes:forward=*, or -1 for "not specified"
    int backward = -1;  ///< lanes:backward=*, or -1 for "not specified"
};

/// Which boolean SetSegmentFlagCommand writes.
enum class SegmentFlag : uint8_t { Oneway, Bridge, Tunnel, Roundabout, Link };

/// Which SideFlags member SetSegmentSideCommand writes.
enum class SideFeature : uint8_t { Sidewalk, Cycleway, Parking, Shoulder };

/// Which string SetSegmentTextCommand writes.
enum class SegmentText : uint8_t { Surface, Name };

/**
 * @brief Trim and lower-case a surface value the way the parser does
 *
 * OSMParser normalises `surface=*` to trimmed lower-case before it reaches a
 * Road, and road_style.hpp matches against lower-case literals. A value typed
 * into an inspector has to go through the same door or "Asphalt" becomes a
 * surface that matches no material and silently renders as the default.
 *
 * ASCII only, exactly like the parser: OSM surface values are an ASCII
 * vocabulary, and a locale-aware fold would make the result depend on the
 * machine.
 */
[[nodiscard]] std::string normalise_surface(std::string_view value);

// ============================================================================
// Commands
// ============================================================================

/**
 * @brief Base for every per-segment attribute edit
 *
 * Owns the whole procedure -- capture, validate, refuse a no-op, replace,
 * restore, coalesce -- so a subclass is just "which field, and what value".
 * That is the point: the rules live in ONE apply(), and a new field cannot
 * accidentally acquire its own subtly different idea of what a no-op is.
 *
 * Reversal is by wholesale restore of the attributes captured before the first
 * apply(), so revert() cannot fail on a segment that still exists, and a merged
 * gesture undoes to where the gesture began rather than to its last step.
 */
class SegmentAttributeCommand : public GraphEditCommand {
public:
    [[nodiscard]] SegmentId segment() const { return m_segment; }

    /// Which field this command writes. The merge key and the invalidation key.
    [[nodiscard]] virtual SegmentField field() const = 0;

    /// What a successful apply() invalidated. Constant per field, so it reads
    /// before execute() as well as after.
    [[nodiscard]] SegmentInvalidation invalidation() const { return invalidation_of(field()); }

    /**
     * @brief Why the last apply() refused
     *
     * None after a successful apply, so it is also the answer to "did that
     * work". A refusal leaves the graph and the history untouched, and this is
     * the only record of the reason.
     */
    [[nodiscard]] SegmentRefusal refusal() const { return m_refusal; }

    /// The attributes as they were before the first apply(). Only meaningful
    /// once one has succeeded; before that it is a default-constructed set.
    [[nodiscard]] const SegmentAttributes& before() const { return m_before; }

    /// The attributes the command installs, including everything a merge absorbed.
    [[nodiscard]] const SegmentAttributes& after() const { return m_after; }

    bool apply() override;
    void revert() override;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] size_t footprint() const override;
    [[nodiscard]] bool merge(const stratum::scene::Command& next) override;

protected:
    SegmentAttributeCommand(EditableGraph& graph, SegmentId segment);

    /**
     * @brief Write this command's value into a copy of the current attributes
     *
     * Called on the FIRST apply() only. Afterwards the command replays the
     * result it computed then, which is what lets a merge change the outcome
     * without every subclass having to know how.
     */
    virtual void write_value(SegmentAttributes& attributes) const = 0;

private:
    /// Put @p attributes on the segment, keeping its handle, way and nodes.
    bool replace(const SegmentAttributes& attributes);

    SegmentId m_segment;

    SegmentAttributes m_before;  ///< Undo state. Captured once, by the first apply().
    SegmentAttributes m_after;   ///< Redo state, and what a merge overwrites.

    /// False until an apply() has succeeded. Guards the capture so that a redo
    /// replays the merged result instead of recomputing the first one.
    bool m_captured = false;

    SegmentRefusal m_refusal = SegmentRefusal::None;
};

/// Change the road class. Unknown is a legal class and is not refused.
class SetSegmentTypeCommand : public SegmentAttributeCommand {
public:
    SetSegmentTypeCommand(EditableGraph& graph, SegmentId segment, RoadType type);

    [[nodiscard]] SegmentField field() const override { return SegmentField::Type; }

protected:
    void write_value(SegmentAttributes& attributes) const override;

private:
    RoadType m_type;
};

/**
 * @brief Change layer=*
 *
 * Not validated. OSM puts no bound on layer=* and a deep stack of viaducts is
 * real data, so a rule here would refuse a map rather than catch a typo.
 */
class SetSegmentLayerCommand : public SegmentAttributeCommand {
public:
    SetSegmentLayerCommand(EditableGraph& graph, SegmentId segment, int layer);

    [[nodiscard]] SegmentField field() const override { return SegmentField::Layer; }

protected:
    void write_value(SegmentAttributes& attributes) const override;

private:
    int m_layer;
};

/// Change the carriageway width, in metres. Refuses anything not above zero,
/// which includes a NaN: the test is `width > 0`, and NaN fails every comparison.
class SetSegmentWidthCommand : public SegmentAttributeCommand {
public:
    SetSegmentWidthCommand(EditableGraph& graph, SegmentId segment, float width);

    [[nodiscard]] SegmentField field() const override { return SegmentField::Width; }

protected:
    void write_value(SegmentAttributes& attributes) const override;

private:
    float m_width;
};

/// Change the lane triple as one value. Refuses a total below 1, a directional
/// count below -1, and a pair of directions totalling more than the total.
class SetSegmentLanesCommand : public SegmentAttributeCommand {
public:
    SetSegmentLanesCommand(EditableGraph& graph, SegmentId segment, LaneCounts lanes);

    [[nodiscard]] SegmentField field() const override { return SegmentField::Lanes; }

protected:
    void write_value(SegmentAttributes& attributes) const override;

private:
    LaneCounts m_lanes;
};

/// Set one of the boolean tags. Which one is part of the command's identity, so
/// a bridge toggle never merges with a tunnel toggle.
class SetSegmentFlagCommand : public SegmentAttributeCommand {
public:
    SetSegmentFlagCommand(EditableGraph& graph, SegmentId segment, SegmentFlag flag, bool value);

    [[nodiscard]] SegmentField field() const override;

protected:
    void write_value(SegmentAttributes& attributes) const override;

private:
    SegmentFlag m_flag;
    bool m_value;
};

/**
 * @brief Set sidewalk, cycleway, parking or shoulder
 *
 * SideFlags::Unknown and SideFlags::None are different values and this command
 * keeps them different. Unknown lets build_profile() infer from the road class;
 * None forbids it. A UI that shows one "off" checkbox for the pair turns every
 * un-surveyed road into an explicitly-no road the first time it is saved.
 */
class SetSegmentSideCommand : public SegmentAttributeCommand {
public:
    SetSegmentSideCommand(EditableGraph& graph, SegmentId segment, SideFeature feature,
                          SideFlags value);

    [[nodiscard]] SegmentField field() const override;

protected:
    void write_value(SegmentAttributes& attributes) const override;

private:
    SideFeature m_feature;
    SideFlags m_value;
};

/**
 * @brief Set the surface or the name
 *
 * A surface is normalised on the way in by normalise_surface(); a name is
 * stored exactly as given, because a street name's capitalisation is data.
 * Normalisation happens BEFORE the no-op test, so re-typing "ASPHALT" over
 * "asphalt" is correctly seen as no change at all.
 */
class SetSegmentTextCommand : public SegmentAttributeCommand {
public:
    SetSegmentTextCommand(EditableGraph& graph, SegmentId segment, SegmentText text,
                          std::string value);

    [[nodiscard]] SegmentField field() const override;
    [[nodiscard]] size_t footprint() const override;

protected:
    void write_value(SegmentAttributes& attributes) const override;

private:
    SegmentText m_text;
    std::string m_value;
};

// ============================================================================
// Helpers
// ============================================================================

/**
 * @brief The segment's lane triple
 *
 * The starting point for an edit to one of the three. Returns a default-built
 * LaneCounts for a segment the graph does not have; the command built from it
 * then refuses with UnknownSegment, so the miss is reported once, where it
 * happens, rather than twice.
 */
[[nodiscard]] LaneCounts lane_counts_of(const EditableGraph& graph, SegmentId segment);

/**
 * @brief Set lanes=* and leave the directional counts alone
 *
 * What a lane spinner calls. Refuses, through the command, when the directions
 * no longer fit inside the new total -- lowering 4 to 3 under `2 + 2` is not a
 * change the caller can have meant.
 *
 * @return the stack's answer: false when the edit was refused and nothing was
 *         recorded.
 */
bool set_lane_total(stratum::scene::CommandStack& stack, EditableGraph& graph, SegmentId segment,
                    int total);

/// Set lanes:forward and lanes:backward, keeping lanes=*. -1 means "not specified".
bool set_directional_lanes(stratum::scene::CommandStack& stack, EditableGraph& graph,
                           SegmentId segment, int forward, int backward);

} // namespace stratum::osm::road
