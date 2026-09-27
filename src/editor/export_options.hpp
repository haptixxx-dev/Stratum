// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file export_options.hpp
 * @brief Every setting the road network export panel edits
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Was four separate `Editor` members (`m_export_config`, `m_export_build_collision`,
 * `m_export_build_lods`, `m_export_dir`), read by `Editor::begin_road_export()` and
 * written from two ImGui panels that drew the same controls twice. Grouping them
 * into one struct gives the export UI a single place to live; the OSM import panel
 * is now the only panel that draws it.
 */

#pragma once

#include "osm/road/road_export.hpp"

namespace stratum {

/**
 * @brief Plain data: destination, format/chunking and the collision/LOD flags
 *
 * `config` already carries the format and chunk-size fields the export UI edits
 * (`osm::road::ExportConfig::format`, `::chunk_size`, `::lod_levels`); this struct
 * does not duplicate them, it just gives them, `build_collision`, `build_lods` and
 * the destination directory one owner.
 */
struct ExportOptions {
    /// Format, chunk size and LOD level count, edited by the OSM import panel
    osm::road::ExportConfig config;

    /// Fill RoadPiece::collision during the export re-solve
    bool build_collision = false;

    /// Fill RoadPiece::lods during the export re-solve
    bool build_lods = false;

    /// Destination directory, typed or chosen. Empty until one is picked.
    char dir[512] = "";
};

} // namespace stratum
