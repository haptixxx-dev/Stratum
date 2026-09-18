// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file instances.cpp
 * @brief F2: the transform list per asset
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The argument for every choice is in instances.hpp. What is here is the
 * arithmetic, and there is very little of it: a placement is copied out of a
 * scope, and everything below is bookkeeping around a std::map.
 *
 * The one include worth explaining is `procgen/rules/shape.hpp`, which is pulled
 * in for `format_number()` alone and only in this translation unit -- the header
 * stays free of the rule layer, so a future caller that has placements and no
 * rules (an OSM prop pass, F5's scatter) does not drag the interpreter in.
 * Spelling a double the same way as dump_shape() does matters more than the
 * dependency costs: two formatters would eventually disagree about a trailing
 * digit, and the failure would look like a determinism bug in the geometry.
 */

#include "procgen/instances.hpp"

#include "procgen/rules/shape.hpp"

#include <cmath>

namespace stratum::procgen {

namespace {

/// Spell a vector the way dump_shape() does, so the two dumps read alike
[[nodiscard]] std::string dump_vec3(const glm::dvec3& v) {
    return "(" + rules::format_number(v.x) + " " + rules::format_number(v.y) + " " +
           rules::format_number(v.z) + ")";
}

} // namespace

// ============================================================================
// InstanceTransform
// ============================================================================

glm::dmat4 InstanceTransform::to_matrix() const {
    // Built column by column rather than as a product of a rotation and a scale,
    // because the product would need the scale as a diagonal matrix and would
    // round a value that is already exact. Column k is the world direction of
    // local axis k, stretched to that axis's extent; the fourth column is the
    // translation. A zero extent therefore gives a zero column, which is a
    // singular matrix and is the honest answer for a flat scope.
    glm::dmat4 out{1.0};
    for (int k = 0; k < 3; ++k) {
        const glm::dvec3 column = axes[k] * size[k];
        out[k] = glm::dvec4{column, 0.0};
    }
    out[3] = glm::dvec4{origin, 1.0};
    return out;
}

bool InstanceTransform::right_handed() const {
    // The determinant of the AXES, never of to_matrix(): a flat scope makes
    // to_matrix() singular, so its determinant is zero and says nothing about
    // which way round the frame is. The axes are orthonormal whatever the
    // extents are, so theirs is +1 or -1 and always answers the question.
    return glm::determinant(axes) > 0.0;
}

int InstanceTransform::flat_axes() const {
    int flat = 0;
    for (int k = 0; k < 3; ++k) {
        // Exactly zero, not a tolerance. A scope extent is either derived by
        // refit_scope() from geometry that is genuinely flat -- a difference of
        // two identical doubles, which is exactly zero -- or set outright by
        // `scale`. A tolerance here would call a one-micron reveal flat and
        // would make the count depend on a constant nobody chose.
        if (size[k] == 0.0) {
            ++flat;
        }
    }
    return flat;
}

bool InstanceTransform::operator==(const InstanceTransform& other) const {
    if (origin != other.origin || size != other.size) {
        return false;
    }
    for (int k = 0; k < 3; ++k) {
        if (axes[k] != other.axes[k]) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// InstanceBounds
// ============================================================================

void InstanceBounds::expand(const glm::dvec3& point) {
    if (!populated) {
        min = point;
        max = point;
        populated = true;
        return;
    }
    min = glm::min(min, point);
    max = glm::max(max, point);
}

void InstanceBounds::expand(const InstanceTransform& transform) {
    // All eight corners, not the origin and the far corner. The frame is rotated
    // in general, so the world box of a rotated placement is not the box of two
    // of its corners -- taking only those two gives a box that misses the other
    // six, and a culling pass built on it pops props in and out at the edges.
    for (int corner = 0; corner < 8; ++corner) {
        const glm::dvec3 unit{static_cast<double>((corner >> 0) & 1),
                              static_cast<double>((corner >> 1) & 1),
                              static_cast<double>((corner >> 2) & 1)};
        expand(transform.unit_to_world(unit));
    }
}

// ============================================================================
// InstanceSet
// ============================================================================

bool InstanceSet::add(std::string_view asset, const InstanceTransform& transform) {
    if (asset.empty()) {
        return false;
    }

    // find-then-insert rather than operator[], because operator[] needs a
    // std::string key and would allocate one on every call including the common
    // case where the asset is already present.
    auto it = assets_.find(asset);
    if (it == assets_.end()) {
        it = assets_.emplace(std::string{asset}, std::vector<InstanceTransform>{}).first;
    }
    it->second.push_back(transform);
    return true;
}

size_t InstanceSet::placement_count() const {
    size_t total = 0;
    for (const auto& entry : assets_) {
        total += entry.second.size();
    }
    return total;
}

const std::vector<InstanceTransform>* InstanceSet::find(std::string_view asset) const {
    const auto it = assets_.find(asset);
    return it == assets_.end() ? nullptr : &it->second;
}

size_t InstanceSet::count_of(std::string_view asset) const {
    const std::vector<InstanceTransform>* list = find(asset);
    return list == nullptr ? 0 : list->size();
}

InstanceBounds InstanceSet::bounds() const {
    InstanceBounds box;
    for (const auto& entry : assets_) {
        for (const InstanceTransform& transform : entry.second) {
            box.expand(transform);
        }
    }
    return box;
}

InstanceBounds InstanceSet::bounds_of(std::string_view asset) const {
    InstanceBounds box;
    const std::vector<InstanceTransform>* list = find(asset);
    if (list == nullptr) {
        return box;
    }
    for (const InstanceTransform& transform : *list) {
        box.expand(transform);
    }
    return box;
}

void InstanceSet::merge(const InstanceSet& other) {
    for (const auto& entry : other.assets_) {
        for (const InstanceTransform& transform : entry.second) {
            // Through add(), so an asset this set has not seen is created the
            // one way, and so a merge of a set that somehow holds an empty key
            // cannot introduce one here.
            (void)add(entry.first, transform);
        }
    }
}

std::string InstanceSet::dump() const {
    std::string out = "instances assets=" + std::to_string(asset_count()) +
                      " placements=" + std::to_string(placement_count()) + "\n";
    for (const auto& entry : assets_) {
        out += "asset " + entry.first + " count=" + std::to_string(entry.second.size()) + "\n";
        for (const InstanceTransform& transform : entry.second) {
            out += "  origin=" + dump_vec3(transform.origin);
            out += " size=" + dump_vec3(transform.size) + "\n";
            out += "  axes x=" + dump_vec3(transform.axes[0]);
            out += " y=" + dump_vec3(transform.axes[1]);
            out += " z=" + dump_vec3(transform.axes[2]) + "\n";
        }
    }
    return out;
}

} // namespace stratum::procgen
