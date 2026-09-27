// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include "renderer/material_library.hpp"
#include <spdlog/spdlog.h>
#include <SDL3/SDL.h>
#include <cstdio>
#include <mutex>
#include <string>
#include <utility>

namespace stratum {

void Editor::open_osm_file_dialog() {
    open_file_dialog(FilePickTarget::OsmFile);
}

void Editor::open_file_dialog(FilePickTarget target) {
    if (m_file_pick.pending) return;  // a dialog is already up

    // These must outlive the call: SDL requires the filter array stay valid until
    // the callback fires, and this function returns immediately.
    static const SDL_DialogFileFilter kOsmFilters[] = {
        { "OpenStreetMap data", "osm;pbf;osm.bz2;osm.gz" },
        { "OSM XML",            "osm" },
        { "OSM PBF",            "pbf" },
        { "All files",          "*" },
    };
    // KTX2 first because it is the format the texture manager reads without
    // recompressing; the stb formats are accepted but arrive uncompressed.
    static const SDL_DialogFileFilter kTextureFilters[] = {
        { "Textures",     "ktx2;ktx;png;jpg;jpeg;tga;bmp;hdr" },
        { "KTX2",         "ktx2;ktx" },
        { "All files",    "*" },
    };
    static const SDL_DialogFileFilter kMaterialSetFilters[] = {
        { "Stratum material set", "json" },
        { "All files",            "*" },
    };
    static const SDL_DialogFileFilter kRuleFilters[] = {
        { "Stratum rule file", "rule" },
        { "All files",         "*" },
    };

    const SDL_DialogFileFilter* filters = kOsmFilters;
    int filter_count = static_cast<int>(SDL_arraysize(kOsmFilters));
    switch (target) {
        case FilePickTarget::MaterialAlbedo:
        case FilePickTarget::MaterialNormal:
        case FilePickTarget::MaterialOrm:
            filters = kTextureFilters;
            filter_count = static_cast<int>(SDL_arraysize(kTextureFilters));
            break;
        case FilePickTarget::MaterialSetLoad:
        case FilePickTarget::MaterialSetSave:
            filters = kMaterialSetFilters;
            filter_count = static_cast<int>(SDL_arraysize(kMaterialSetFilters));
            break;
        case FilePickTarget::RuleFileLoad:
        case FilePickTarget::RuleFileSave:
            filters = kRuleFilters;
            filter_count = static_cast<int>(SDL_arraysize(kRuleFilters));
            break;
        case FilePickTarget::OsmFile:
            break;
    }

    // Set the target BEFORE marking pending: poll_file_dialog() only reads it once
    // a result has landed, and a result cannot land before the dialog is shown.
    m_file_pick_target = target;
    m_file_pick.pending = true;

    // One callback for every target. It may run on a different thread than the
    // main loop, so it does nothing but record the outcome; poll_file_dialog()
    // applies it on the main thread next frame.
    const SDL_DialogFileCallback callback =
        [](void* userdata, const char* const* filelist, int /*filter*/) {
            auto* self = static_cast<Editor*>(userdata);
            std::lock_guard<std::mutex> lock(self->m_file_pick.mutex);
            self->m_file_pick.has_result = true;
            self->m_file_pick.path.clear();
            self->m_file_pick.error.clear();

            if (!filelist) {
                const char* err = SDL_GetError();
                self->m_file_pick.error = (err && *err) ? err : "file dialog failed";
            } else if (filelist[0]) {
                self->m_file_pick.path = filelist[0];  // single-select
            }
            // filelist non-null with a null first entry means the user cancelled;
            // both strings stay empty and the UI simply does nothing.
        };

    auto* parent = static_cast<SDL_Window*>(m_window_handle);  // for modality

    if (target == FilePickTarget::RuleFileSave) {
        // Same argument as the material set below: start where the file came
        // from, so a re-save lands beside the rule file rather than in $HOME.
        SDL_ShowSaveFileDialog(callback, this, parent, filters, filter_count,
                               m_rule_path.empty() ? nullptr : m_rule_path.c_str());
    } else if (target == FilePickTarget::MaterialSetSave) {
        // Start in the directory the set was last saved to or loaded from, so a
        // re-save lands beside its textures rather than in the home directory --
        // the paths inside the file are written RELATIVE to it.
        SDL_ShowSaveFileDialog(callback, this, parent, filters, filter_count,
                               m_material_set_path.empty() ? nullptr
                                                           : m_material_set_path.c_str());
    } else {
        SDL_ShowOpenFileDialog(callback, this, parent, filters, filter_count,
                               nullptr,  // platform default location
                               false);   // single selection
    }
}

void Editor::poll_file_dialog() {
    std::string path, error;
    {
        std::lock_guard<std::mutex> lock(m_file_pick.mutex);
        if (!m_file_pick.has_result) return;
        m_file_pick.has_result = false;
        m_file_pick.pending = false;
        path = std::move(m_file_pick.path);
        error = std::move(m_file_pick.error);
        m_file_pick.path.clear();
        m_file_pick.error.clear();
    }

    if (!error.empty()) {
        // Most likely on Linux with no XDG desktop portal and no zenity/kdialog.
        // The OSM path field is still there to type into, so this is not fatal
        // there; for the material targets it means the button simply does nothing,
        // which is why the reason is put on the console rather than only in the log.
        spdlog::error("File dialog unavailable: {}", error);
        char msg[512];
        snprintf(msg, sizeof(msg),
                 "[Editor] File dialog unavailable (%s) - type a path instead\n",
                 error.c_str());
        m_console_buffer.append(msg);
        m_console_scroll_to_bottom = true;
        // Report into the panel that opened the dialog. Before the rule targets
        // existed this was "anything but OSM means the material panel", which
        // would now put a rule-file failure under the material set.
        switch (m_file_pick_target) {
            case FilePickTarget::RuleFileLoad:
            case FilePickTarget::RuleFileSave:
                m_rule_status = "file dialog unavailable: " + error;
                break;
            case FilePickTarget::OsmFile:
                break;
            default:
                m_material_set_status = "file dialog unavailable: " + error;
                break;
        }
        return;
    }

    // Cancelled. Every target treats that as "do nothing", so it is handled once
    // here rather than in each branch below.
    if (path.empty()) return;

    switch (m_file_pick_target) {
        case FilePickTarget::OsmFile:
            std::snprintf(m_osm_filepath, sizeof(m_osm_filepath), "%s", path.c_str());
            spdlog::info("Selected OSM file: {}", path);
            break;

        case FilePickTarget::MaterialAlbedo:
        case FilePickTarget::MaterialNormal:
        case FilePickTarget::MaterialOrm: {
            if (!m_material_library) break;
            const auto map =
                m_file_pick_target == FilePickTarget::MaterialAlbedo
                    ? MaterialLibrary::TextureMap::Albedo
                    : m_file_pick_target == FilePickTarget::MaterialNormal
                          ? MaterialLibrary::TextureMap::Normal
                          : MaterialLibrary::TextureMap::Orm;
            // Goes through the library, not through GPUTextureManager directly, so
            // the source path is recorded and survives the next save. See
            // MaterialLibrary::load_map_from_file().
            if (m_material_library->load_map_from_file(m_material_pick_key, map, path)) {
                m_material_set_status = "loaded " + path;
            } else {
                m_material_set_status = "failed to load " + path;
            }
            break;
        }

        case FilePickTarget::MaterialSetLoad: {
            if (!m_material_library) break;
            if (m_material_library->load_from_file(path)) {
                m_material_set_path = path;
                m_material_set_status =
                    "loaded " + std::to_string(m_material_library->size()) + " materials";
                // The new set is a different set: counts collected against the old
                // one describe materials that no longer exist.
                m_material_library->reset_resolve_stats();
            } else {
                m_material_set_status = "load failed - see console";
            }
            break;
        }

        case FilePickTarget::MaterialSetSave: {
            if (!m_material_library) break;
            if (m_material_library->save_to_file(path)) {
                m_material_set_path = path;
                m_material_set_status = "saved to " + path;
            } else {
                m_material_set_status = "save failed - see console";
            }
            break;
        }

        case FilePickTarget::RuleFileLoad:
            load_rule_source(path);
            break;

        case FilePickTarget::RuleFileSave:
            save_rule_source(path);
            break;
    }
}

void Editor::open_export_dir_dialog() {
    if (m_dir_pick.pending) return;  // a dialog is already up

    m_dir_pick.pending = true;

    SDL_ShowOpenFolderDialog(
        [](void* userdata, const char* const* filelist, int /*filter*/) {
            // May run on a different thread than the main loop, so record the
            // outcome and nothing else. poll_export_dir_dialog() applies it.
            auto* self = static_cast<Editor*>(userdata);
            std::lock_guard<std::mutex> lock(self->m_dir_pick.mutex);
            self->m_dir_pick.has_result = true;
            self->m_dir_pick.path.clear();
            self->m_dir_pick.error.clear();

            if (!filelist) {
                const char* err = SDL_GetError();
                self->m_dir_pick.error = (err && *err) ? err : "folder dialog failed";
            } else if (filelist[0]) {
                self->m_dir_pick.path = filelist[0];
            }
            // A non-null list with a null first entry is a cancel: both strings
            // stay empty and the UI does nothing.
        },
        this,
        static_cast<SDL_Window*>(m_window_handle),  // parent, for modality
        nullptr,                                    // platform default location
        false);                                     // single selection
}

void Editor::poll_export_dir_dialog() {
    std::string path, error;
    {
        std::lock_guard<std::mutex> lock(m_dir_pick.mutex);
        if (!m_dir_pick.has_result) return;
        m_dir_pick.has_result = false;
        m_dir_pick.pending = false;
        path = std::move(m_dir_pick.path);
        error = std::move(m_dir_pick.error);
        m_dir_pick.path.clear();
        m_dir_pick.error.clear();
    }

    if (!error.empty()) {
        // Most likely on Linux with no XDG desktop portal and no zenity/kdialog.
        // The path field is still there to type into, so this is not fatal.
        spdlog::error("Folder dialog unavailable: {}", error);
        m_export_status = "Folder dialog unavailable - type a path instead";
        return;
    }

    if (!path.empty()) {
        std::snprintf(m_export_dir, sizeof(m_export_dir), "%s", path.c_str());
        spdlog::info("Export directory: {}", path);
    }
}

} // namespace stratum
