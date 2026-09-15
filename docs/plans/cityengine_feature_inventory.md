# ArcGIS CityEngine — full feature inventory

Reference catalogue of everything CityEngine does, with Stratum's status against
each item. Companion to `docs/plans/cityengine_parity.md`, which turns the gaps
into ordered work. This file is the *what*; that file is the *when*.

## Provenance and version facts

- **Latest release: ArcGIS CityEngine 2026.0**, shipped July 2026. There is no
  2027 release. Esri moved to a calendar-year naming scheme, so 2026.0 is the
  current product and 2026.1 is the expected next point release.
- Preceding releases relevant here: **2025.0** (introduced Street Designer and
  the Visual CGA Editor's maturity), **2025.1** (Python 3 API in beta),
  **2026.0** (Python 3 out of beta, street node cleanup tools, per-corner curb
  radii, Node Browser, licensing change).
- Everything below is drawn from Esri's own documentation and release notes,
  listed under Sources at the end. Where a doc page did not enumerate something,
  it is marked *(not enumerated in public docs)* rather than guessed at.

## Status legend

| Tag | Meaning |
|---|---|
| ✅ | Stratum has this |
| 🟡 | Stratum has part of this |
| ❌ | Stratum has nothing |
| ⛔ | Deliberate non-goal for Stratum |

---

# 1. Product and platform

| Feature | Detail | Stratum |
|---|---|---|
| Desktop app, Windows + Linux | 2026.0 replaced the Linux installer with a standard RPM package; added RHEL 10 support and Wayland | ✅ Linux desktop, Arch |
| Named User licensing only | 2026.0 retired Single Use and Concurrent Use, removed ArcGIS Administrator and the Flexera FlexNet system; you sign in to ArcGIS Online/Enterprise at launch | ⛔ No licensing server; Stratum is not license-gated |
| Eclipse RCP application shell | Perspectives, dockable windows, preferences tree | 🟡 ImGui docking, no perspective presets |
| Workspace + project model | Projects on disk with `assets/`, `data/`, `maps/`, `models/`, `rules/`, `scenes/`, `images/` conventions; 2026.0 added `pyproject.toml` per project to track the Python environment | ❌ No project concept |
| Self-signed Portal certificate support, HTTPS doc/project fetch | 2026.0 | ⛔ |

# 2. Scene model

The CityEngine scene (`.cej`) is a typed layer tree. This is the substrate every
other feature stands on.

| Feature | Detail | Stratum |
|---|---|---|
| **Shape layer** | Holds shapes — 2D footprints and 3D geometry used as initial shapes for CGA generation | ❌ |
| **Graph layer** | Holds street networks and blocks: edges, nodes, dynamic shapes (street shapes, blocks, parcels), and generated models | 🟡 Road graph exists in memory, not as a scene layer |
| **Static Model layer** | Holds imported models (e.g. Collada) that cannot be processed further by CGA | ❌ |
| **Map layer** | Holds images used to globally drive parameters; the scene terrain is a map layer | ❌ |
| **Analysis layer** | Holds analysis objects — viewsheds, view domes, view corridors | ❌ |
| **Group layer** | Nests other layers for scene organisation | ❌ |
| Scenarios | Alternative variants of a scene managed alongside it, for comparing design options | ❌ |
| Scene Editor operations | Delete, duplicate, merge layers; cut/copy/paste objects between layers; drag-reorder; layer colours with inheritance through the hierarchy; lock/unlock; visibility; rename; frame in viewport; multi-select; wildcard search and filter; import/export layers as `.obj`, `.dxf`, `.cej`; unique-name enumeration | 🟡 Scene hierarchy panel only, no operations |
| Object attributes | Arbitrary typed key/values attached to any scene object, imported alongside GIS features | ❌ |
| Array attributes | Float, string and Boolean arrays as both rule and object attributes | ❌ |
| Attribute sources and precedence | Every attribute resolves from one of: Default, User, Object, Shape, Layer — shown in the Inspector's Sources section | ❌ |
| Connection Editor | Maps an attribute to a source: a map layer, another attribute, or an expression | ❌ |
| Undo/redo across all edits | Including street edits, which 2026.0 improved to keep blocks and selections stable through undo | ❌ |
| Save/load scene, georeferenced | `.cej` scene file with a coordinate reference system | ❌ |

# 3. Import

| Format / source | Notes | Stratum |
|---|---|---|
| OSM (`.osm`) | Native OSM import | ✅ Plus `.pbf`, which CE does not read natively |
| Esri Shapefile (`.shp`) | Points, lines, polygons, limited multipatch; attributes land in Object Attributes | ❌ |
| Esri File Geodatabase (FileGDB) | Points, lines, polygons, multipatch, raster | ⛔ Esri-specific |
| DXF (Autodesk) | Vector import; 2026.0 gives imported street graphs sensible default lane widths and uses street/sidewalk width attributes when present | ❌ |
| DWG (Autodesk) | | ❌ |
| FBX (Autodesk) | Import as shapes or static models; 2025.0 fixed FBX colour-space conversion to linear | ❌ |
| OBJ (Wavefront) | | ❌ |
| Collada (`.dae`) | 2025.0 fixed opacity handling for legacy Maya/SketchUp files and texture search in the geometry file's directory | ❌ |
| USD | 2025.0 fixed cube-primitive materials and added auto-UV generation for textured faces | ❌ |
| glTF | | ❌ |
| IFC (buildingSMART) | | ❌ |
| KML / KMZ | | ❌ |
| Heightmap images | GeoTIFF and standard image formats as terrain map layers | ❌ Terrain is procedural noise only |
| Texture / satellite imagery | As texture map layers | ❌ |
| ArcGIS Feature Services | From Enterprise portals v11+ | ⛔ |
| Get Map Data | Fetch basemap and elevation for an extent straight from ArcGIS Online | ❌ (equivalent: fetch OSM + DEM for a bbox) |
| Overture Maps | Via the new Python 3 API example shipped in 2026.0 | ❌ |
| TGA images | Added 2025.0 | 🟡 Depends on stb/assimp coverage |
| Import as shapes vs static models | The core distinction: shapes can be generated on; static models cannot | ❌ |

# 4. Export

CityEngine's export list is long, and the *options* matter as much as the
formats. Stratum currently exports the road network only.

## Formats

| Category | Formats | Stratum |
|---|---|---|
| 3D | ABC (Alembic), DAE (Collada), DATASMITH (Unreal + Twinmotion), DWG, FBX, glTF, IFC, OBJ, USD, VOB (e-on VUE) | 🟡 OBJ + glTF, roads only |
| 3D GIS | FileGDB, KMZ/KML | ⛔ |
| ArcGIS Online | MSPK (mobile scene package), SLPK (scene layer package), 3VR (360 VR Experience) | ⛔ |
| Material | CGA Material | ❌ |
| Custom | Python script-based export | ❌ |

Esri's own notes on the formats: FBX is the main exchange format for Unreal and
Unity but lacks PBR material support and metadata transport; USD is the
low-loss interchange format for large scenes in VFX pipelines; Alembic carries
geometry plus materials, CGA reports and object attributes; glTF allows
triangulated geometry only.

## Export options (common to most exporters)

| Group | Options | Stratum |
|---|---|---|
| General | Output Path, Base Name, Export Geometry, Terrain Layers, Simplify Terrain Meshes, Terrain Mesh Resolution | 🟡 Path and chunking only |
| Granularity | Memory Budget; **Mesh Granularity**: *Do not merge any meshes* / *Merge meshes by material* / *Reuse asset instances, merge generated meshes by material*; **Feature Granularity**: *One Feature Per Shape* / *One Feature Per Leaf Shape* | ❌ |
| Geometry | Vertex Normals, Normals Indexing, Texture Coordinates, Local Offset, Global Offset, Vertex Precision, Merge within precision, Triangulate Meshes, Faces with holes | 🟡 Some are implicit |
| Materials and textures | Include Materials, Create Texture Atlas, Texture Atlas Max Dimension, Texture Atlas Border | ❌ No atlas |
| Advanced | File Type, Embed Textures, Shape Name Delimiter, Existing Files, Script | ❌ |

**Stratum's existing edge:** the road exporter's "whole triangles only, assigned
exactly once" chunk invariant (`src/osm/road/road_export.hpp`) has no CityEngine
equivalent — CE merges by material or by shape, not by spatial chunk, so it does
not produce engine-ready streaming tiles.

# 5. Streets

The largest and most actively developed part of CityEngine, and the part
Stratum is closest to on geometry and furthest from on authoring.

## 5.1 Graph model

| Feature | Detail | Stratum |
|---|---|---|
| Graph of edges and nodes | The street network substrate | ✅ `road::RoadGraph` |
| Blocks | Derived automatically from graph cycles; the input to lot subdivision | ❌ |
| Dynamic shapes | Street shapes, sidewalk shapes, junction shapes, crossing shapes, blocks and parcels, all regenerated when the graph changes | 🟡 Geometry is generated but is not editable shapes |
| Street node precision | Default reduced from 0.5 to 0.4 in 2026.0 | n/a |
| Node types | Junction shape driven by node type and per-node parameters | 🟡 Junction classification exists, no user override |

## 5.2 Street creation tools

| Tool | Detail | Stratum |
|---|---|---|
| Polygonal Street Creation | Draw streets with defined vertices | ❌ |
| Freehand Street Creation | Draw streets without angular constraints | ❌ |
| Grow Streets | Procedural network generation — see 5.5 | ❌ |
| Snapping | To nodes, segments and existing geometry, with preview lines | ❌ |

## 5.3 Street editing tools

| Tool | Detail | Stratum |
|---|---|---|
| Edit Streets/Curves | Move nodes and segments, reshape curves; 2026.0 added direct node merging via the yellow disc / orange ball handles | ❌ |
| **Merge Nodes** *(new in 2026.0)* | Select nodes by click or drag and merge into an existing node, a street segment, or any viewport location, with preview and snapping | ❌ |
| **Remove Nodes** *(new in 2026.0)* | Delete nodes by click or drag-over, with optional curve refitting; *Remove joints only* restricts deletion to two-segment nodes so intersections survive a bulk selection | ❌ |
| Toolbar grouping | 2026.0 grouped Edit Streets/Curves, Merge Nodes and Remove Nodes | n/a |
| Snap to terrain while dragging | 2026.0 extended this to Transform Move and Edit Streets/Curves | ❌ |
| Undo/redo that keeps blocks stable | 2026.0 fix | ❌ |

## 5.4 Street Designer *(introduced 2025.0, extended 2026.0)*

This is CityEngine's answer to "a street is not one ribbon". Stratum's
`road_profile.*` solves the same problem from tags instead of from UI.

| Feature | Detail | Stratum |
|---|---|---|
| Roadbed + lanes model | A street is a centreline plus an ordered set of lanes | ✅ Strip profile model |
| Lane types | Road lane, bus lane, pedestrian lane, tree/vegetation lane, lamp lane, furniture lane, parking, and more (the full set is not enumerated in public docs; it is the contents of `ESRI.lib/rules/Complete_Lanes/`) | 🟡 Carriageway, sidewalk, cycleway, parking, shoulder |
| Add Lanes / Remove Lanes / Edit Lanes | Direct 3D viewport manipulation of individual lanes | ❌ |
| Street Configurations | Save a lane set as a named reusable configuration; 2025.0 shipped 15 built-in configurations, plus a `Generic_Street_Configurations.cej` scene | ❌ |
| Per-lane CGA rules | Each lane can carry its own rule file for detailed visualisation | ❌ |
| **Per-corner curb radius** *(new in 2026.0)* | A separate curb radius for every corner of an intersection, set in the Inspector | 🟡 One analytic fillet radius per junction |
| Transit and pedestrian curbside content *(new in 2026.0)* | `Curbzone/Bay_Lane.cga` (bus bays), `Curbzone/Bulb_Lane.cga` (bus bulbs), `Curbzone/Transit_Shelter_Lane.cga`, `Crosswalk/Pedestrian_Island_Lane.cga`, `Crosswalk/Zebra_Crosswalk_Lane.cga`, `Transition/Rounded_Ending_Lane.cga` (rounded lane endings at intersection entrances) | 🟡 Zebra crossings exist in `road/crossings.*`; no bays, bulbs, shelters or islands |
| Lane selection UX | 2026.0: single-click lane selection at small scales, better highlighting, bold parent items in Scene Editor, HiDPI handle scaling | ❌ |
| Street segment centre lines follow geometry exactly | 2026.0 fix | ✅ |

## 5.5 Grow Streets — procedural network generation

Three patterns, combinable, applied separately to **major** streets (which
enclose quarters) and **minor** streets (which subdivide quarters).

**Patterns:** Organic, Raster, Radial.

| Group | Parameters |
|---|---|
| Basic | Number of streets; Pattern of major streets; Pattern of minor streets; Long length; Long length deviation; Short length; Short length deviation |
| Advanced | Snapping distance (minimal distance between any two nodes); Minimal angle; Street to crossing ratio (controls average quarter size via node density); Development center preference; Angle offset of major streets; Angle offset of minor streets |
| Environment | Adapt to elevation; Critical slope; Maximal slope; Adaption angle; Heightmap; Obstaclemap |
| Organic | Max. bend angle (organic) |
| Radial | City center x; City center y; Max. bend angle (radial); Street Alignment (radial vs centripetal) |
| Street | Calculate width using street integration; Minimum/Maximum number of roadbed lanes; Minimum/Maximum sidewalk width |
| Randomly distributed | Minimum/Maximum major roadbed lanes; Sidewalk width of major streets; Sidewalk width deviation of major streets; Minimum/Maximum minor roadbed lanes; Sidewalk width of minor streets; Sidewalk width deviation of minor streets |
| Block | Block Subdivision: Recursive / Offset / Skeleton / None / Disabled |

**Stratum: ❌ across the whole table.** This is the single biggest missing
authoring feature.

# 6. Blocks, lots and parcels

Derived from the graph; the bridge from streets to buildings. Stratum has
nothing here.

## Subdivision algorithms

| Algorithm | Behaviour |
|---|---|
| **Recursive** | Creates rectangular lots by repeatedly splitting the block. Computes the minimum-area oriented bounding box (OBB) of the lot; by default the pivot is the midpoint of the largest OBB edge and the split direction is that of the smallest edge. Recurses while the constraints hold. |
| **Offset** | Computes an inward offset of the block contour and subdivides the strip between contour and offset, so lots sit within a fixed distance of the street edges. Recursive parameters then apply to the strip. |
| **Skeleton** | Uses the **straight skeleton** to find centre lines, producing street-aligned lots that always have street access, with lot sides perpendicular to the adjacent road. |
| **No Subdivision** | One lot for the whole block. |

## Parameters

| Group | Parameters (internal name in brackets) |
|---|---|
| General | Terrain Alignment `alignment` [0–3]; Create Shape `shapeCreation` |
| Recursive | Force Street Access `forceStreetAccess` [0.0–1.0]; Lot Area Min `lotAreaMin`; Lot Area Max `lotAreaMax`; Lot Width Min `lotWidthMin`; Subdivision Irregularity `irregularity` [0.0–1.0]; Subdivision Seed `seed`; Block Corner Angle Threshold `cornerAngleMax`; Block Corner Length `cornerWidth` |
| Offset | Offset Width `offsetWidth`, plus all recursive parameters |
| Skeleton | Shallow Lot Fraction `shallowLotFrac`; Corner Alignment `cornerAlignment` (priority by street width or by length); Simplify `simplify` [0.0–1.0]; Lot Area Min; Lot Width Min (ideal street frontage); Subdivision Irregularity; Subdivision Seed |

**Consistent indexing.** CityEngine maps lots through barycentric coordinates so
lot ordering survives regeneration. Anything Stratum builds here needs the same
property or rule assignment will shuffle on every edit.

Also: manual lot editing, lot isolation (2026.0 fixed isolated lots causing a
whole block to reappear), and parcel edge classification used by rules.

**Stratum: ❌ across this entire section.**

# 7. Map layers

| Layer type | Purpose | Stratum |
|---|---|---|
| **Terrain layer** | Textured terrain from a heightmap image plus a texture image; the scene's elevation model | 🟡 Procedural heightmap, no image import, no texture drape |
| **Texture layer** | Flat horizontal plane in the scene, typically a basemap or water body | ❌ |
| **Obstacle layer** | Boolean map marking where Grow Streets may not build; commonly a land/water mask. Also usable to select objects by location | ❌ |
| **Mapping layer** | Controls a building's height or usage type by its location | ❌ |
| **Function layer** | Generates a value per location from a mathematical function | 🟡 Noise exists but is not a scene-visible layer |
| Parameters | Height map image, texture image, map files, positioning bounds, elevation offset, Boolean attributes for image-map selection | ❌ |
| Constraints | Map layers are selected in the Scene Editor only; they can be moved and scaled but not rotated | n/a |
| Image layer mipmaps | Added 2025.0 | n/a |

# 8. CGA — the shape grammar

The heart of the product. Below is the complete published operation, function and
keyword catalogue for CityEngine 2026.0. **Stratum has no equivalent of any of
it.**

## 8.1 Operations

**Geometry creation**
`i` (insert) · `extrude` · `envelope` · `taper` · `roofGable` · `roofHip` ·
`roofPyramid` · `roofRidge` · `roofShed` · `offset` · `primitiveQuad` ·
`primitiveDisk` · `primitiveCube` · `primitiveSphere` · `primitiveCylinder` ·
`primitiveCone` · `insertAlongUV` · `footprint`

**Geometry subdivision**
`split` · `splitArea` · `splitAndSetbackPerimeter` · `comp` · `innerRectangle` ·
`setback` · `setbackPerEdge` · `setbackToArea` · `shapeL` · `shapeU` · `shapeO` ·
`scatter`

**Geometry manipulation**
`cleanupGeometry` · `convexify` · `deleteHoles` · `reverseNormals` ·
`softenNormals` · `setNormals` · `mirror` · `rectify` · `reduceGeometry` ·
`resetGeometry` · `trim` · `modify`

**Geometry tagging**
`tag` · `deleteTags` · `setTagsFromEdgeAttrs`

**Rule inlining and boolean 3D**
`inline` · `union` · `subtract` · `intersect`

**Texturing**
`texture` · `setupProjection` · `projectUV` · `translateUV` · `scaleUV` ·
`normalizeUV` · `tileUV` · `rotateUV` · `deleteUV` · `copyUV`

**Transformations**
`t` (scope translate) · `translate` · `s` (scope size) · `r` (scope rotate) ·
`rotate` · `center`

**Scope**
`alignScopeToAxes` · `alignScopeToGeometry` · `alignScopeToGeometryBBox` ·
`rotateScope` · `setPivot` · `mirrorScope`

**Flow control**
`pop` · `push` · `NIL`

**Context**
`label`

**Attributes**
`set` · `color` · `resetMaterial` · `setMaterial` · `report` · `print`

## 8.2 Shape attributes

`comp` · `initialShape` · `material` · `pivot` · `scope` · `seedian` · `split` ·
`trim`

## 8.3 Built-in functions

**Math** `abs` `acos` `asin` `atan` `atan2` `ceil` `cos` `exp` `floor` `isinf`
`isnan` `ln` `log10` `pow` `rint` `sin` `sqrt` `tan`

**Probability** `p` · `rand`

**Conversion** `bool` `boolArray` `float` `floatArray` `isNull` `sel` `str`
`stringArray`

**String** `count` `find` `len` `splitString` `substring`

**Geometry** `geometry.angle` `geometry.area` `geometry.boundaryLength`
`geometry.du` `geometry.dv` `geometry.groups` `geometry.hasTags`
`geometry.hasUVs` `geometry.height` `geometry.isClosedSurface`
`geometry.isConcave` `geometry.isInstanced` `geometry.isOriented`
`geometry.isPlanar` `geometry.isRectangular` `geometry.materials`
`geometry.nEdges` `geometry.nFaces` `geometry.nHoles` `geometry.nVertices`
`geometry.tags` `geometry.uMin` `geometry.uMax` `geometry.vMin` `geometry.vMax`
`geometry.volume`

**Material** `getMaterial` · `readMaterial`

**File** `fileExists` `fileSearch` `filesSearch` `readFloatTable`
`readStringTable` `readTextFile`

**Asset and image** `assetInfo` `assetMetadata` `assetNamingInfo`
`assetNamingInfos` `assetsSortRatio` `assetsSortSize` `imageInfo`
`imagesSortRatio`

**Occlusion** `inside` · `overlaps` · `touches`

**Context** `minimumDistance` · `contextCompare` · `contextCount`

**Array** array initialisation · colon operator · `comp` · `findFirst` · index
operator · `nColumns` · `nRows` · `setElems` · `size` · `sortIndices` ·
`sortRowIndices` · `sum` · `transpose`

**Edge attributes** `edgeAttr.getFloat` · `edgeAttr.getString` ·
`edgeAttr.getBool`

**Misc** `convert` · `getGeoCoord` · `getTreeKey` · `print` · simple-type
operators · array-type operators

## 8.4 Keywords

`attr` · `const` · `extension` · `import` · `start` · `style` · `with` ·
`version`

## 8.5 CGA utility function library (`ce.lib`)

**String** `findFirst` `findLast` `getPrefix` `getRange` `getSuffix` `replace`

**String list** `listAdd` `listClean` `listCount` `listFirst` `listFromArray`
`listIndex` `listItem` `listLast` `listRandom` `listRange` `listRemove`
`listRemoveAll` `listRetainAll` `listSize` `listTerminate` `listToArray`

**File, asset, image** `assetApproxRatio` `assetApproxSize` `assetBestRatio`
`assetBestSize` `assetFitSize` `fileBaseName` `fileDirectory` `fileExtension`
`fileName` `fileRandom` `imageApproxRatio` `imageBestRatio`

**Math** `clamp` `max` `min`

**Colour** `colorHexToB` `colorHexToG` `colorHexToH` `colorHexToO` `colorHexToR`
`colorHexToS` `colorHexToV` `colorHSVToHex` `colorHSVOToHex` `colorRamp`
`colorRGBToHex` `colorRGBOToHex`

## 8.6 CGA language capabilities beyond the operator list

| Capability | Detail |
|---|---|
| Rule packages (RPK) | Compiled, distributable rule bundles. 2026.0 raised the limit past 2 GB |
| Styles | `style` keyword: variant rule sets over a shared base |
| Dynamic and static imports | With consistent random-number behaviour across both as of 2026.0 |
| Stochastic rules | Weighted alternatives via `p` |
| Attribute annotations | `@Range`, `@Order`, `@Hidden`, `@MaterialFile`, `@Color`, `@Group`, and descriptions surfaced in the Inspector |
| Reporting | `report()` accumulates numbers up the shape tree; consumed by the Inspector and dashboards |
| Occlusion and context queries | `inside` / `overlaps` / `touches` plus `minimumDistance`, `contextCompare`, `contextCount` — used for shared party walls and balcony trimming |
| Boolean 3D | `union` / `subtract` / `intersect`, with simplified syntax and robustness fixes in 2025.0 and 2026.0 |
| Edge tagging | Auto-tagging on intersection edges; `bool.cut` auto tags on boolean cut edges |
| Deterministic seeds | `seedian` shape attribute keeps generation reproducible |
| Large-scene generation performance | Explicitly improved in 2026.0 |

# 9. Authoring environments

## 9.1 CGA Editor (text)

| Feature | Detail |
|---|---|
| Code completion *(redesigned 2026.0)* | Suggests relevant operations and functions and shows their documentation beside the suggestion |
| Hover documentation | Pop-ups over existing code |
| Jump to CGA Reference | Context-menu command into the reference window |
| Offline CGA Reference | Refreshed with a modern look in 2026.0 |
| Editor preferences page | New in 2026.0; font, line spacing, auto-indent after Enter |
| Show Definition, Select Shapes | Context-menu navigation between code and the model |

## 9.2 Visual CGA Editor (node graph)

**Directly relevant to Stratum's Q1.** Esri ships *both* a text DSL and a node
graph over the same language — the node graph compiles to CGA rules, it is not a
separate system.

| Feature | Detail |
|---|---|
| Node Browser *(new in 2026.0)* | Resizable sidebar replacing the old Add Node dialog; Home / Description / Search views; list and grid modes; visual previews per component; folder hierarchy navigation; advanced search filtering by input shapes, attributes and extensions |
| Add and replace | `+Add` button or drag-and-drop; Replace button swaps a selected component in place |
| Component descriptions | Include all attributes and extensions |
| Mini-inspector | Per-node attribute editing inline in the graph |
| Annotation editor | `@Order`, `@Hidden`, `@MaterialFile`, `@Color` support |
| Initial Shapes node | Renameable; defines the start rule |
| Extension slots | Visualised, including NIL generation |
| Find Node dialog | Added 2025.0 |
| Auto-reload | Outdated components refresh when a VCGA design is opened |
| Exports to CGA | The graph produces rule files |

## 9.3 Inspector

| Section | Shows / edits |
|---|---|
| Parameters | Street and object parameters — street width, node type, lane widths, per-corner curb radius |
| Rules | The assigned CGA rule and its attributes — colour, building style, floor count, zoning type |
| Object Attributes | Attributes attached to the selected scene objects |
| Array Attributes | Float, string and Boolean arrays for rule and object attributes |
| Sources | Where each attribute's value comes from: Default, User, Object, Shape, Layer |
| Reports | Every reported variable for the selection, with Name, Count, Percentage, Sum, Average, Min, Max and NaN occurrences |

**Stratum:** 🟡 a properties panel exists; none of these six sections do.

# 10. ESRI.lib — the shipped content library

CityEngine's stock rules are a large fraction of its practical value. A parity
effort that ships an engine and no content is not at parity.

| Area | Content |
|---|---|
| Architecture: windows | Redesigned in 2026.0 around a Structure → Frame → Partitioning → Operation → Panel workflow; railing components; arched and round opening components |
| Architecture: balconies | Initial balcony component set, new in 2026.0 |
| Architecture: massing | Massing models updated to work on slanted parcels |
| Component thumbnails | Added per component in 2026.0 for the Node Browser |
| Street lane rules | `Complete_Lanes/` rule files per lane type; curbzone rules (bus bay, bulb, transit shelter); crosswalk rules (zebra, pedestrian island); transition rules (rounded ending) |
| Street configurations | 15 built-in configurations plus `Generic_Street_Configurations.cej`; named examples include `Downtown_MajorSt_4VL_39m` and `Simple_Street_Generic_2VL_17.20m`, both with bus-stop variants |
| Vegetation | `PlantLoader.cga` and a plant asset set |
| Examples | Contemporary Architecture Park; VCGA Playground; CityEngine Tour; Stylizing Scenes with generative AI (2026.0); Overture Maps import via Python (2026.0); Publishing custom metrics to ArcGIS Urban (2026.0) |

**Stratum:** ❌ no content library. `road_style.*` is the closest thing and it is
code, not editable content.

# 11. Assets, materials and rendering

| Feature | Detail | Stratum |
|---|---|---|
| Asset library with search paths | Models and textures resolved from project asset folders | ❌ |
| Asset preview and framing | 2026.0 fixed preview framing | ❌ |
| Material Browser | Double-click assigns a material and closes the browser (2025.0) | 🟡 Material panel exists |
| Texture atlas creation on export | With max dimension and border controls | ❌ |
| Instancing | `i()` inserted assets can be preserved as instances on export via the *Reuse asset instances* granularity mode | ❌ |
| Viewport rendering | Anti-aliasing and anisotropic filtering on by default (2025.0), adjustable anisotropy, wireframe quality improvements on transparent models and dark-theme scene grid (2026.0) | ✅ MSAA; 🟡 no wireframe mode |
| Snapshots | Capped at 10,000 × 10,000 px, with improved shadow quality | ❌ |
| Navigation and gizmos | Transform move/rotate/scale; tumble gizmo removed from Rotate and Dolly in 2025.0 | ✅ im3d gizmos |
| Shadows | Present, quality improved in snapshots | ✅ Cascaded shadow maps |
| PBR materials | CityEngine's own material model; note Esri's admission that FBX export loses PBR | ✅ Cook-Torrance PBR |

# 12. Analysis and reporting

| Feature | Detail | Stratum |
|---|---|---|
| CGA reports | `report()` accumulates values up the shape tree; rule-based calculation of floor area, unit counts, cost, parking counts | ❌ |
| Inspector Reports section | Name, Count, Percentage, Sum, Average, Min, Max, NaN occurrences | ❌ |
| Dashboard | Cards over report data, updating live as models change; errors when a card references a report that no longer exists | ❌ |
| CSV export of reports | Numerical reports to `.csv` for Excel | ❌ |
| Viewshed Creation | Visibility from a camera-like observer over a limited field of view; visible area green, occluded red by default | ❌ |
| View Dome Creation | Same, with 360° field of view | ❌ |
| View Corridor Creation | Protected view corridor; geometry visible in the corridor is highlighted | ❌ |
| Analyze visibility by layer | Per-layer contribution to visibility, shown in the Inspector | ❌ |
| Visibility settings tool | Show/hide analysis objects | ❌ |
| Analysis layers | Scene layer type holding analysis objects | ❌ |
| **Solar / shadow analysis** | **CityEngine does not have this.** Users move data to ArcGIS Pro for solar radiation and shadow studies | ❌ — and a genuine opening, since Stratum already has the shadow pipeline |

# 13. Python scripting

2026.0's headline. Stratum has pybind11 wired into the build and non-functional.

| Feature | Detail | Stratum |
|---|---|---|
| Python 3 API out of beta | Full feature parity with the legacy Jython API; requires Python 3.11+ | ❌ |
| `cityengine` package | Installable into an external environment; interchangeable with the old `scripting` module | ❌ |
| Third-party ecosystem | Full PyPI and Anaconda access; combinable with `arcgis` and `arcpy` | ❌ |
| Environment management | Create, add, edit and delete venv and Conda environments in-app; run pip and conda commands; reuse ArcGIS Pro environments; `workspace/.venvs` storage; per-project `pyproject.toml` | ❌ |
| Concurrent API sessions | Multiple consoles and scripts at once (2026.0) | ❌ |
| Startup scripts | Run a selected script when CityEngine launches (2026.0) | ❌ |
| Python Console | Redesigned toolbar icons | ❌ |
| Custom UIs | Python UI libraries for data entry, visualisation and generative-AI applications | ❌ |
| Scriptable street editing | `addLane`, `moveLane`, `removeLane`, `isLane`, `createStreetConfiguration`, `duplicateStreetConfiguration`, `deleteStreetConfiguration`, `applyStreetConfigurationToSegment`, `createGraphSegments` with an initial configuration, `CE.getSubOID` | ❌ |
| Scriptable subdivision | `SubdivideShapesSettings` and the rest of the scene API | ❌ |
| Script-based export | "Python" as an export format | ❌ |
| Web scene sharing from Python | 2025.0 | ⛔ |

# 14. ArcGIS ecosystem integration

Mostly non-goals for Stratum, listed for completeness.

| Feature | Stratum |
|---|---|
| ArcGIS Urban round-trip: import plans, publish custom rule-based metrics via the Urban GraphQL API | ⛔ |
| Share As Web Scene, WebScene exporter access levels | ⛔ |
| Feature Services from Enterprise portals (v11+) | ⛔ |
| Get Map Data (basemap + elevation for an extent) | 🟡 worth an OSM/DEM equivalent |
| SLPK / MSPK / i3s scene layers | ⛔ |
| 360 VR Experience (3VR) export | ⛔ |
| Basemaps and georeferencing against Esri services | ⛔ |

# 15. Ecosystem SDKs and plugins

The part of CityEngine that Stratum's `stratum_core` split is already shaped to
compete with.

| Product | What it is | Stratum equivalent |
|---|---|---|
| **CityEngine SDK / PRT** | C++ SDK around the Procedural Runtime: takes an initial geometry plus a rule package and generates detailed 3D geometry, without running CityEngine. Proprietary, licensed | `stratum_core` — already engine-agnostic, already headless, and unencumbered |
| **PyPRT** | Python bindings for PRT | Track J |
| **ArcGIS CityEngine for Unreal Engine (Vitruvio)** | UE plugin generating buildings from CGA rules in-editor and at runtime | ❌ |
| **ArcGIS CityEngine for Houdini** | Executes CGA rules as a surface operator inside Houdini networks | ❌ |
| **ArcGIS CityEngine for Rhino** | Commands and Grasshopper components running CityEngine rules in Rhino | ❌ |

**The strategic read:** Esri's runtime story is a licensed binary SDK. A
permissively licensed, headless, game-engine-first `stratum_core` that generates
at runtime is the one thing CityEngine structurally cannot offer.

# 16. Version timeline

| Version | Notable |
|---|---|
| 2025.0 | Street Designer (lanes, configurations, per-lane rules); Visual CGA Editor Find Node and annotations; CGA boolean-3D syntax simplification, edge auto-tagging, component-split scope alignment, connected-component extraction; TGA import; USD and Collada fixes; FBX colour space; anti-aliasing defaults |
| 2025.1 | Python 3 API in beta |
| 2026.0 | Python 3 API out of beta with env/package management and startup scripts; Merge Nodes and Remove Nodes; per-corner curb radii; transit and pedestrian curbside lane rules; Node Browser with previews; redesigned CGA Editor completion; balcony and redesigned window components; RPK > 2 GB; large-scene generation performance; Named User licensing only; RHEL 10, Wayland, RPM installer |

# 17. What this changes in the parity plan

Six corrections and additions to `docs/plans/cityengine_parity.md`:

1. **CityEngine has no solar or shadow analysis.** Item I4 was scoped as parity;
   it is actually a differentiator, and Stratum's cascaded shadow pipeline is
   most of the work already.
2. **Esri ships a node graph *and* a text DSL over one language.** Q1's three
   options are not exclusive — the Visual CGA Editor compiles to CGA. The right
   answer is very likely "text DSL first, node graph over it later", which is
   what Esri converged on.
3. **Lot indexing must be stable across regeneration.** CityEngine uses
   barycentric mapping to keep lot ordering consistent. Add this to C7 — without
   it, rule assignments shuffle on every street edit.
4. **Skeleton subdivision and the roof operations share the straight skeleton.**
   C3 and D5 should be planned as one piece of work with two consumers.
5. **Export granularity is a first-class feature, not a checkbox.** CE exposes
   mesh granularity, feature granularity, memory budget, vertex precision and
   atlas controls. H3 should be sized up accordingly.
6. **Content is a deliverable.** ESRI.lib is a large shipped asset. D11 needs to
   grow to cover lane rules, street configurations, window/balcony/massing
   component sets and a plant loader, not just six building rule files.

# Sources

- [What's new in CityEngine 2026.0](https://doc.arcgis.com/en/cityengine/latest/whats-new/cityengine-whats-new.htm)
- [CityEngine 2026.0 release notes](https://doc.arcgis.com/en/cityengine/latest/whats-new/cityengine-release-notes.htm)
- [CityEngine 2025.0 release notes](https://doc.arcgis.com/en/cityengine/latest/whats-new/cityengine-2025-0-release-notes.htm)
- [What's New in ArcGIS CityEngine 2026.0 (Esri blog)](https://www.esri.com/arcgis-blog/products/city-engine/3d-gis/whats-new-in-arcgis-cityengine-2026-0)
- [Esri releases CityEngine 2026 (CG Channel)](https://www.cgchannel.com/2026/07/esri-releases-cityengine-2026/)
- [CGA Shape Grammar Reference](https://doc.arcgis.com/en/cityengine/latest/cga/cityengine-cga-introduction.htm)
- [Essential shape operations](https://doc.arcgis.com/en/cityengine/latest/help/help-essential-shape-operations-overview.htm)
- [Export a model / export formats and options](https://doc.arcgis.com/en/cityengine/latest/help/help-export-models-overview.htm)
- [Graph networks](https://doc.arcgis.com/en/cityengine/latest/help/help-graph-overview.htm)
- [Generate street networks (Grow Streets)](https://doc.arcgis.com/en/cityengine/latest/help/help-grow-a-street.htm)
- [Street Designer overview](https://doc.arcgis.com/en/cityengine/latest/help/street-designer-overview.htm)
- [Work with street configurations](https://doc.arcgis.com/en/cityengine/2025.0/help/street-designer-street-configuration.htm)
- [Blocks and block parameters](https://doc.arcgis.com/en/cityengine/latest/help/help-layers-block-parameters.htm)
- [Map Layer overview](https://doc.arcgis.com/en/cityengine/latest/help/help-map-layer-overview.htm)
- [Scene Editor](https://doc.arcgis.com/en/cityengine/latest/help/help-scene-editor.htm)
- [Inspector](https://doc.arcgis.com/en/cityengine/latest/help/help-inspector.htm)
- [Visibility analysis overview](https://doc.arcgis.com/en/cityengine/latest/help/help-visibility-analysis-overview.htm)
- [Import SHP / Import FileGDB / Prepare data for import](https://doc.arcgis.com/en/cityengine/latest/help/help-import-preparing-data.htm)
- [Procedural Runtime Plugins (Esri open source)](https://esri.github.io/cityengine/)
- [CityEngine SDK on GitHub](https://github.com/Esri/cityengine-sdk)
