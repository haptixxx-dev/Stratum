# Changelog

All notable changes to Stratum are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and Stratum uses
[semantic versioning](https://semver.org/spec/v2.0.0.html).

Releases map to the milestones in `docs/plans/milestones.md`.

## [Unreleased]

### Added

- Apache-2.0 licensing: `LICENSE`, `NOTICE`, `THIRD_PARTY_LICENSES.md`, and
  SPDX headers across every file in `src/` and `tests/`.
- `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, issue forms and a
  pull request template.
- CI now builds *and runs the test suite* on Linux via the `ci` preset. The
  test step had been commented out since the workflow was written.
- CI caches compiler output with ccache (sccache on Windows), cancels
  superseded runs, and refuses to publish a release whose tag disagrees with
  `VERSION`.
- `docs/plans/cityengine_feature_inventory.md` — full ArcGIS CityEngine 2026.0
  feature catalogue with Stratum's status against each item.
- `docs/plans/cityengine_parity.md` — the parity backlog, tracks A–K.
- `docs/plans/milestones.md` — ten milestones from here to v1.0.0.

### Fixed

- Windows CI has failed at the configure step since August 2026 with
  `CMAKE_CXX_COMPILER: cl is not a full path`. Ninja plus `cl` needs the MSVC
  environment on `PATH`; the workflow now sets it up before configuring.

### Removed

- 29 stale `.gitmodules` entries — 28 under the non-existent
  `submodules/external/` prefix plus `basis_universal`, none of which was
  registered in the git index.

## [0.1.0]

Initial development. OSM import, road network solve, terrain generation,
SDL_GPU renderer with PBR and cascaded shadows, ImGui editor.
