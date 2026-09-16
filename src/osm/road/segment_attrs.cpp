// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file segment_attrs.cpp
 * @brief Implementation of the per-segment attribute edits
 *
 * Three rules shape the file, all stated in segment_attrs.hpp:
 *
 *   1. One apply(). Every field goes through the same capture, the same
 *      validation, the same no-op test and the same replacement, so there is
 *      exactly one place where those rules can be wrong.
 *   2. Refuse, never clamp. apply() returning false leaves the graph and the
 *      history as they were, and refusal() says which rule fired.
 *   3. An edit is judged on the errors it INTRODUCES, so an already-broken tag
 *      elsewhere on the segment cannot freeze the rest of it.
 */

#include "osm/road/segment_attrs.hpp"

#include <spdlog/spdlog.h>

#include <cctype>
#include <cmath>
#include <memory>
#include <utility>

namespace stratum::osm::road {

namespace {

/// Bytes an attribute set holds on the heap, for Command::footprint()
size_t attributes_heap_bytes(const SegmentAttributes& attributes) {
    return attributes.surface.capacity() + attributes.name.capacity();
}

/// Append @p name to @p out with a comma separator. Keeps describe_refusal()
/// from growing a "first item" flag at every call site.
void append_reason(std::string& out, const char* name) {
    if (!out.empty()) out += ", ";
    out += name;
}

} // namespace

// ============================================================================
// Validation
// ============================================================================

SegmentRefusal validate_attributes(const SegmentAttributes& attributes) {
    SegmentRefusal refusal = SegmentRefusal::None;

    if (attributes.lanes < 1) {
        // Not clamped to 1. build_profile() already substitutes max(1, lanes)
        // so that a bad extract still produces a mesh, which means a 0 stored
        // here would never show up as anything except a road one lane narrower
        // than its tags claim.
        refusal = refusal | SegmentRefusal::LanesBelowOne;
    }

    // `> 0` and not `!= 0`: this is also the NaN test, since every comparison
    // against a NaN is false. A NaN width propagates into the profile, the
    // trims and finally into vertex positions, where it is a whole afternoon to
    // trace back to the field that produced it.
    if (!(attributes.width > 0.0f) || !std::isfinite(attributes.width)) {
        refusal = refusal | SegmentRefusal::WidthNotPositive;
    }

    if (attributes.lanes_forward < -1 || attributes.lanes_backward < -1) {
        refusal = refusal | SegmentRefusal::DirectionalLanesNegative;
    }

    // Unspecified counts as nothing, not as zero-and-therefore-fine: with
    // lanes=2 and lanes:forward=3 the single specified direction already
    // exceeds the total, and that has to be caught whether or not its partner
    // was surveyed.
    //
    // Skipped entirely when the total is itself invalid, and that guard is
    // load-bearing. Reporting BOTH bits on a `lanes=0` way would make the
    // conflict pre-existing, and the introduced-error rule would then happily
    // accept a repair to `lanes=2` under `2 + 1` -- swapping one piece of
    // nonsense for another and calling it a fix. With the guard, repairing the
    // total surfaces the conflict and the user is made to resolve it.
    const int forward = attributes.lanes_forward > 0 ? attributes.lanes_forward : 0;
    const int backward = attributes.lanes_backward > 0 ? attributes.lanes_backward : 0;
    if (attributes.lanes >= 1 && forward + backward > attributes.lanes) {
        refusal = refusal | SegmentRefusal::DirectionalLanesExceedTotal;
    }

    return refusal;
}

std::string describe_refusal(SegmentRefusal refusal) {
    std::string out;
    if (has(refusal, SegmentRefusal::LanesBelowOne)) append_reason(out, "lanes below 1");
    if (has(refusal, SegmentRefusal::WidthNotPositive)) append_reason(out, "width not above 0");
    if (has(refusal, SegmentRefusal::DirectionalLanesNegative)) {
        append_reason(out, "directional lanes below -1");
    }
    if (has(refusal, SegmentRefusal::DirectionalLanesExceedTotal)) {
        append_reason(out, "directional lanes exceed the total");
    }
    if (has(refusal, SegmentRefusal::UnknownSegment)) append_reason(out, "no such segment");
    if (has(refusal, SegmentRefusal::NoChange)) append_reason(out, "no change");
    if (out.empty()) out = "none";
    return out;
}

// ============================================================================
// Fields
// ============================================================================

const char* field_name(SegmentField field) {
    switch (field) {
        case SegmentField::Type:       return "Type";
        case SegmentField::Layer:      return "Layer";
        case SegmentField::Width:      return "Width";
        case SegmentField::Lanes:      return "Lanes";
        case SegmentField::Oneway:     return "Oneway";
        case SegmentField::Bridge:     return "Bridge";
        case SegmentField::Tunnel:     return "Tunnel";
        case SegmentField::Roundabout: return "Roundabout";
        case SegmentField::Link:       return "Link";
        case SegmentField::Sidewalk:   return "Sidewalk";
        case SegmentField::Cycleway:   return "Cycleway";
        case SegmentField::Parking:    return "Parking";
        case SegmentField::Shoulder:   return "Shoulder";
        case SegmentField::Surface:    return "Surface";
        case SegmentField::Name:       return "Name";
        case SegmentField::Count:      break;
    }
    return "Unknown";
}

const char* field_label(SegmentField field) {
    switch (field) {
        case SegmentField::Type:       return "road class";
        case SegmentField::Layer:      return "layer";
        case SegmentField::Width:      return "width";
        case SegmentField::Lanes:      return "lane counts";
        case SegmentField::Oneway:     return "one-way";
        case SegmentField::Bridge:     return "bridge";
        case SegmentField::Tunnel:     return "tunnel";
        case SegmentField::Roundabout: return "roundabout";
        case SegmentField::Link:       return "link";
        case SegmentField::Sidewalk:   return "sidewalk";
        case SegmentField::Cycleway:   return "cycleway";
        case SegmentField::Parking:    return "parking";
        case SegmentField::Shoulder:   return "shoulder";
        case SegmentField::Surface:    return "surface";
        case SegmentField::Name:       return "name";
        case SegmentField::Count:      break;
    }
    return "attribute";
}

SegmentInvalidation invalidation_of(SegmentField field) {
    // Every edit reaches the export, so Metadata is the floor. What separates
    // the fields is whether the cross-section changes, because the cross-section
    // is what both the corridor sweep and the junction trims are measured from.
    constexpr SegmentInvalidation kMeta = SegmentInvalidation::Metadata;
    constexpr SegmentInvalidation kGeometry =
        SegmentInvalidation::Metadata | SegmentInvalidation::Corridor
        | SegmentInvalidation::Junctions;
    constexpr SegmentInvalidation kStructural = kGeometry | SegmentInvalidation::Structures;

    switch (field) {
        // The class table drives lane width, sidewalk synthesis and the material
        // family at once, so changing it changes everything downstream of the
        // profile.
        case SegmentField::Type:       return kGeometry;

        // A grade change moves the road vertically and decides what passes over
        // what, so the structure builders have to run again as well.
        case SegmentField::Layer:      return kStructural;
        case SegmentField::Bridge:     return kStructural;
        case SegmentField::Tunnel:     return kStructural;

        case SegmentField::Width:      return kGeometry;

        // lane_count() feeds total_width(), which is the arm half-width every
        // junction at both ends is solved from.
        case SegmentField::Lanes:      return kGeometry;

        // Not appearance-only, although the markings are the visible part: a
        // one-way suppresses the median strip, and the median is part of the
        // carriageway envelope the trim solve intersects.
        case SegmentField::Oneway:     return kGeometry;

        // The roundabout flag is how the special-junction pass finds its cycles,
        // so toggling it can create or destroy a whole junction treatment.
        case SegmentField::Roundabout: return kGeometry;

        // Read by build_profile() alongside the rest of the class evidence.
        case SegmentField::Link:       return kGeometry;

        // Each of these adds or removes strips outboard of the carriageway, so
        // the total width changes and the curb ring at each junction with it.
        case SegmentField::Sidewalk:   return kGeometry;
        case SegmentField::Cycleway:   return kGeometry;
        case SegmentField::Parking:    return kGeometry;
        case SegmentField::Shoulder:   return kGeometry;

        // NOT the cheap re-skin it looks like, and the table said it was until a
        // review found the coupling. build_profile() reads the surface TWICE:
        // once for the material slot, and once through surface_is_unpaved(),
        // which feeds `rural_verge = cfg.rural_verges && rules.rural_verge &&
        // unpaved`. rural_verge is set for primary, secondary, tertiary,
        // residential and service, and ProfileConfig::rural_verges defaults on,
        // so crossing the paved/unpaved line ADDS or DROPS a Verge strip on every
        // side that carries neither a kerb nor a shoulder. That changes
        // RoadProfile::total_width(), and junction_trim.cpp's fill_widths()
        // halves exactly that number into every arm's half_width -- so the edit
        // moves vertices on the segment AND at both junctions. Appearance is kept
        // alongside, so a consumer doing the material pass still learns the
        // variant changed rather than inferring it from a corridor rebuild.
        case SegmentField::Surface:    return kGeometry | SegmentInvalidation::Appearance;

        // A label and an export field. Nothing geometric, nothing visual.
        case SegmentField::Name:       return kMeta;

        case SegmentField::Count:      break;
    }
    return SegmentInvalidation::Nothing;
}

// ============================================================================
// Values
// ============================================================================

std::string normalise_surface(std::string_view value) {
    // Mirrors normalize_value() in parser.cpp: trim, then ASCII lower-case. The
    // two must agree, or a surface typed into the inspector and the identical
    // surface read from the extract compare as different values.
    size_t begin = 0;
    size_t end = value.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(value[begin]))) ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) --end;

    std::string out;
    out.reserve(end - begin);
    for (size_t i = begin; i < end; ++i) {
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(value[i]))));
    }
    return out;
}

// ============================================================================
// SegmentAttributeCommand
// ============================================================================

SegmentAttributeCommand::SegmentAttributeCommand(EditableGraph& graph, SegmentId segment)
    : GraphEditCommand(graph), m_segment(segment) {}

bool SegmentAttributeCommand::apply() {
    const EditableSegment* record = graph().find_segment(m_segment);
    if (record == nullptr) {
        m_refusal = SegmentRefusal::UnknownSegment;
        return false;
    }

    const SegmentAttributes current = record->attributes;

    // On the first apply the subclass computes the result. On a redo the command
    // replays the result it computed then, which is what a merged gesture stored
    // -- recomputing would replay only the first step of the drag.
    SegmentAttributes candidate = m_captured ? m_after : current;
    if (!m_captured) write_value(candidate);

    // Judged on what it INTRODUCES. An extract already tagged lanes=0 must not
    // make its own name unfixable, and a width of 0 must still be refused on
    // that same segment -- which only works because these are bits in a mask and
    // not a single first-error code.
    const SegmentRefusal introduced = validate_attributes(candidate)
                                      & ~validate_attributes(current);
    if (introduced != SegmentRefusal::None) {
        m_refusal = introduced;
        return false;
    }

    // Tested on EVERY apply, not just the first. A merge that absorbed a return
    // to the starting value would leave a step whose redo is a no-op; refusing
    // here makes CommandStack::redo() drop that step loudly instead of carrying
    // a do-nothing entry in the undo menu. The merge() guard below stops it
    // arising, and this is the second line of the same defence.
    if (candidate == current) {
        m_refusal = SegmentRefusal::NoChange;
        return false;
    }

    if (!m_captured) {
        m_before = current;
        m_after = candidate;
        m_captured = true;
    }

    m_refusal = SegmentRefusal::None;
    return replace(m_after);
}

void SegmentAttributeCommand::revert() {
    // Wholesale, not field by field: the command restores the attribute set it
    // captured, so a field the edit never touched cannot be left behind at a
    // value the subclass forgot to put back.
    if (!replace(m_before)) {
        spdlog::error("SegmentAttributeCommand: segment {} is gone at undo", m_segment);
    }
}

std::string SegmentAttributeCommand::describe() const {
    return std::string("Set ") + field_label(field());
}

size_t SegmentAttributeCommand::footprint() const {
    // sizeof of the base, not of the most-derived object: a subclass adds a
    // scalar or two and the stack's bound is in megabytes. What must not be
    // missed is the heap, which is the two attribute copies' strings.
    // SetSegmentTextCommand carries a third and adds it in its own override.
    return sizeof(SegmentAttributeCommand) + attributes_heap_bytes(m_before)
           + attributes_heap_bytes(m_after);
}

bool SegmentAttributeCommand::merge(const stratum::scene::Command& next) {
    const auto* other = dynamic_cast<const SegmentAttributeCommand*>(&next);
    if (other == nullptr) return false;

    // Same document, same segment, same field. Without the first, two graphs
    // open at once coalesce each other's edits; without the third, a rename
    // typed straight after a width drag disappears into the drag's undo step.
    if (!same_graph(*other)) return false;
    if (other->m_segment != m_segment) return false;
    if (other->field() != field()) return false;

    // The successor must have applied, or there is nothing to take over: its
    // m_after is still a default-constructed SegmentAttributes, and absorbing it
    // would make this command's redo install a zero width and an empty name.
    // CommandStack::execute() applies before it offers a merge, so it cannot
    // reach this line; a caller driving merge() directly can, and does in
    // a_merge_refuses_a_successor_that_never_applied.
    if (!other->m_captured) return false;

    // Refuse a merge that would leave this command a no-op. Spin a lane count up
    // and back down and the gesture ends where it started; absorbing that leaves
    // old and new equal, undo does nothing visible, and the redo hits apply()'s
    // own equal-value guard, so CommandStack drops the step and the whole redo
    // branch with it. Found first in A1's RenameLayerCommand, then in
    // MoveNodeCommand, and it is the same fix here.
    if (other->m_after == m_before) return false;

    // The successor has already applied, so the graph holds its result. Taking
    // it over means this command's revert() still returns to m_before, which is
    // where the gesture began.
    m_after = other->m_after;
    return true;
}

bool SegmentAttributeCommand::replace(const SegmentAttributes& attributes) {
    const EditableSegment* record = graph().find_segment(m_segment);
    if (record == nullptr) return false;

    // Copied whole and put back whole. insert_segment() takes the handle from
    // the record it is given -- that is how an undo restores a deleted segment
    // rather than a lookalike -- so the SegmentId, the source_way and the node
    // list all survive verbatim and no caller's handle goes stale.
    EditableSegment copy = *record;
    copy.attributes = attributes;

    if (!erase_segment(m_segment)) return false;
    if (!insert_segment(std::move(copy))) {
        // Unreachable: the id was just freed, and insert_segment()'s other two
        // guards are on a node list that the graph accepted when the segment was
        // created and that nothing between here and there can have changed.
        // Logged rather than ignored because if it ever does fire, the segment
        // has been deleted by an edit that claimed to be a rename.
        spdlog::error("SegmentAttributeCommand: segment {} could not be put back", m_segment);
        return false;
    }
    return true;
}

// ============================================================================
// SetSegmentTypeCommand
// ============================================================================

SetSegmentTypeCommand::SetSegmentTypeCommand(EditableGraph& graph, SegmentId segment, RoadType type)
    : SegmentAttributeCommand(graph, segment), m_type(type) {}

void SetSegmentTypeCommand::write_value(SegmentAttributes& attributes) const {
    attributes.type = m_type;
}

// ============================================================================
// SetSegmentLayerCommand
// ============================================================================

SetSegmentLayerCommand::SetSegmentLayerCommand(EditableGraph& graph, SegmentId segment, int layer)
    : SegmentAttributeCommand(graph, segment), m_layer(layer) {}

void SetSegmentLayerCommand::write_value(SegmentAttributes& attributes) const {
    // The SEGMENT's layer tag only. EditableNode::layer is a separate field and
    // is not touched: node identity, and the grade separation that rests on it,
    // is decided at load and is not a per-way attribute edit.
    attributes.layer = m_layer;
}

// ============================================================================
// SetSegmentWidthCommand
// ============================================================================

SetSegmentWidthCommand::SetSegmentWidthCommand(EditableGraph& graph, SegmentId segment, float width)
    : SegmentAttributeCommand(graph, segment), m_width(width) {}

void SetSegmentWidthCommand::write_value(SegmentAttributes& attributes) const {
    attributes.width = m_width;
}

// ============================================================================
// SetSegmentLanesCommand
// ============================================================================

SetSegmentLanesCommand::SetSegmentLanesCommand(EditableGraph& graph, SegmentId segment,
                                               LaneCounts lanes)
    : SegmentAttributeCommand(graph, segment), m_lanes(lanes) {}

void SetSegmentLanesCommand::write_value(SegmentAttributes& attributes) const {
    // All three at once. Writing them one command at a time would make a legal
    // end state unreachable whenever the path to it crosses an illegal one.
    attributes.lanes = m_lanes.total;
    attributes.lanes_forward = m_lanes.forward;
    attributes.lanes_backward = m_lanes.backward;
}

// ============================================================================
// SetSegmentFlagCommand
// ============================================================================

SetSegmentFlagCommand::SetSegmentFlagCommand(EditableGraph& graph, SegmentId segment,
                                             SegmentFlag flag, bool value)
    : SegmentAttributeCommand(graph, segment), m_flag(flag), m_value(value) {}

SegmentField SetSegmentFlagCommand::field() const {
    switch (m_flag) {
        case SegmentFlag::Oneway:     return SegmentField::Oneway;
        case SegmentFlag::Bridge:     return SegmentField::Bridge;
        case SegmentFlag::Tunnel:     return SegmentField::Tunnel;
        case SegmentFlag::Roundabout: return SegmentField::Roundabout;
        case SegmentFlag::Link:       return SegmentField::Link;
    }
    return SegmentField::Count;
}

void SetSegmentFlagCommand::write_value(SegmentAttributes& attributes) const {
    switch (m_flag) {
        case SegmentFlag::Oneway:     attributes.is_oneway = m_value; return;
        case SegmentFlag::Bridge:     attributes.is_bridge = m_value; return;
        case SegmentFlag::Tunnel:     attributes.is_tunnel = m_value; return;
        case SegmentFlag::Roundabout: attributes.is_roundabout = m_value; return;
        case SegmentFlag::Link:       attributes.is_link = m_value; return;
    }
}

// ============================================================================
// SetSegmentSideCommand
// ============================================================================

SetSegmentSideCommand::SetSegmentSideCommand(EditableGraph& graph, SegmentId segment,
                                             SideFeature feature, SideFlags value)
    : SegmentAttributeCommand(graph, segment), m_feature(feature), m_value(value) {}

SegmentField SetSegmentSideCommand::field() const {
    switch (m_feature) {
        case SideFeature::Sidewalk: return SegmentField::Sidewalk;
        case SideFeature::Cycleway: return SegmentField::Cycleway;
        case SideFeature::Parking:  return SegmentField::Parking;
        case SideFeature::Shoulder: return SegmentField::Shoulder;
    }
    return SegmentField::Count;
}

void SetSegmentSideCommand::write_value(SegmentAttributes& attributes) const {
    // m_value is stored, never folded. Unknown and None arrive here as the two
    // different values they are and leave as the same two: Unknown lets
    // build_profile() infer from the road class, None forbids it, and collapsing
    // the pair turns every un-surveyed road into an explicitly-no road.
    switch (m_feature) {
        case SideFeature::Sidewalk: attributes.sidewalk = m_value; return;
        case SideFeature::Cycleway: attributes.cycleway = m_value; return;
        case SideFeature::Parking:  attributes.parking = m_value; return;
        case SideFeature::Shoulder: attributes.shoulder = m_value; return;
    }
}

// ============================================================================
// SetSegmentTextCommand
// ============================================================================

SetSegmentTextCommand::SetSegmentTextCommand(EditableGraph& graph, SegmentId segment,
                                             SegmentText text, std::string value)
    : SegmentAttributeCommand(graph, segment), m_text(text) {
    // Normalised HERE and not in write_value(), so that the value the command
    // holds is the value it will store. Normalising later would mean the no-op
    // test in apply() compared a raw string against a normalised one, and
    // re-typing "ASPHALT" over "asphalt" would record an undo step that changed
    // nothing.
    m_value = (text == SegmentText::Surface) ? normalise_surface(value) : std::move(value);
}

SegmentField SetSegmentTextCommand::field() const {
    return m_text == SegmentText::Surface ? SegmentField::Surface : SegmentField::Name;
}

size_t SetSegmentTextCommand::footprint() const {
    return SegmentAttributeCommand::footprint() + m_value.capacity();
}

void SetSegmentTextCommand::write_value(SegmentAttributes& attributes) const {
    // A name keeps its capitalisation: "Ashton Road" is data, not a lookup key.
    if (m_text == SegmentText::Surface) {
        attributes.surface = m_value;
    } else {
        attributes.name = m_value;
    }
}

// ============================================================================
// Helpers
// ============================================================================

LaneCounts lane_counts_of(const EditableGraph& graph, SegmentId segment) {
    const EditableSegment* record = graph.find_segment(segment);
    if (record == nullptr) return LaneCounts{};

    LaneCounts counts;
    counts.total = record->attributes.lanes;
    counts.forward = record->attributes.lanes_forward;
    counts.backward = record->attributes.lanes_backward;
    return counts;
}

bool set_lane_total(stratum::scene::CommandStack& stack, EditableGraph& graph, SegmentId segment,
                    int total) {
    LaneCounts counts = lane_counts_of(graph, segment);
    counts.total = total;
    return stack.execute(std::make_unique<SetSegmentLanesCommand>(graph, segment, counts));
}

bool set_directional_lanes(stratum::scene::CommandStack& stack, EditableGraph& graph,
                           SegmentId segment, int forward, int backward) {
    LaneCounts counts = lane_counts_of(graph, segment);
    counts.forward = forward;
    counts.backward = backward;
    return stack.execute(std::make_unique<SetSegmentLanesCommand>(graph, segment, counts));
}

} // namespace stratum::osm::road
