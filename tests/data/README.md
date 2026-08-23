# Road network test fixtures

Hand-authored minimal `.osm` XML extracts. Every file is a complete, valid OSM XML
document that libosmium can read directly through `stratum::osm::OSMParser`.

They are deliberately tiny and hand-written rather than clipped from a real
download, so that every node ID, tag and expected count in `tests/road/` can be
stated exactly. Coordinates are real, plausible WGS84 around Dublin
(~53.33-53.35 N, ~6.24-6.27 W); each fixture occupies its own small patch and its
own node/way ID block, so the files can be concatenated for ad-hoc experiments
without ID collisions.

The parser recentres processed geometry on the centre of mass of its features
(`OSMParser::recenter_on_features`), so tests must never assert an absolute local
metre coordinate. Compare a graph node position against the parsed `Road::polyline`
entry for the same `NodeId` instead.

| Fixture | ID block | What it is | What it must produce |
|---|---|---|---|
| `four_way.osm` | 1xx / 10xx | Two roads crossing at shared node 100, tagged `highway=traffic_signals` | Exactly one `GraphNode` of degree 4, `osm_id == 100`, `has_signals == true` |
| `t_junction.osm` | 2xx / 20xx | Way 2010 starts at node 203, the **middle** node of way 2000 | Exactly one `GraphNode` of degree 3, at node 203. Nodes 202, 204, 206 get no `GraphNode` |
| `cul_de_sac.osm` | 3xx / 30xx | Residential dead end, `highway=turning_circle` on node 306 | One degree-1 `GraphNode` with `is_turning_circle == true`, `osm_id == 306` |
| `roundabout.osm` | 4xx / 40xx | Closed way 4000 tagged `junction=roundabout`, three approach roads | Three edges, all `is_roundabout`, forming a closed cycle; every approach node (401, 403, 405) has degree 3 |
| `motorway_link.osm` | 5xx / 50xx | `highway=motorway` mainline with a `highway=motorway_link` ramp joining at interior node 503 | Every edge of way 5010 has `is_link == true`; no edge of way 5000 does; node 503 has degree 3 |
| `rural_track.osm` | 6xx / 60xx | Isolated `highway=track`, `surface=gravel`, no sidewalk tag | One edge, `surface == "gravel"`, `sidewalk` is `SideFlags::Unknown` or `SideFlags::None` |
| `bridge_over.osm` | 7xx / 70xx | `bridge=yes layer=1` road crossing a `layer=0` road, **sharing** node 702 | Node 702 yields **two** `GraphNode`s with different `layer`; neither has degree 4; `Stats::layer_split_nodes == 1` |
| `duplicate_node.osm` | 8xx / 80xx | Nodes 802 and 803 carry identical lat/lon; way 8000 lists both, ways 8010 and 8020 tee off one each | 802 and 803 resolve to **one** `GraphNode` of degree 4; 5 nodes, 4 edges, 1 junction, 4 dead ends |
| `bridge_abutment.osm` | 9xx / 90xx | One primary road as approach (`layer` absent), `bridge=yes layer=1` deck, approach again | Abutments 902 and 903 are **one** `GraphNode` each, of degree 2; `Stats::layer_split_nodes == 0`; 4 nodes, 3 edges, 2 dead ends |
| `crossing.osm` | 10xx / 100xx | A `highway=crossing` node INTERIOR to way 10000, plus a skew `footway=crossing` way over way 10010 sharing node 1012 | Exactly two `Crossing`s: one on way 10000 with `at_junction == false` and `node == kInvalidId`, one on way 10010 with `node` the graph node of 1012. No crossing on the footway edge |
| `sidewalk_dup.osm` | 11xx / 110xx | Residential `sidewalk=both`, a `footway=sidewalk` way 5.03 local units to its LEFT over most of it, and a `footway=crossing` way square across it | `DedupResult::suppress_side` is `Left` on the western edge of way 11000 and `None` everywhere else; `suppressed_edges == 1`, `matched_footways == 1` |
| `turn_lanes.osm` | 12xx / 120xx | One-way three-lane primary tagged `turn:lanes=left\|through\|through;right`, approaching signalled node 1203 | Three arrows, left to right `ArrowLeft`, `ArrowStraight`, `ArrowStraightRight`; one `StopLine` quad per lane; node 1203 degree 4 with `has_signals` |
| `dual_carriageway.osm` | 13xx / 130xx | Four-lane two-way PRIMARY with no `median=*` tag, and a four-lane SECONDARY as the control | Way 13000 gets one raised `StripKind::Median` in `MaterialId::Grass` with a `CurbFace` either side; way 13010 gets none |
| `service_roads.osm` | 14xx / 140xx | `service=driveway`, `service=parking_aisle` and `service=alley` leaving a residential street at three interior nodes | Three degree-3 nodes; each service edge has no Sidewalk, Curb or Gutter strip; widths 3.0 m, 5.5 m and 3.5 m; `driveway_kerb_spans()` returns a span at each |
| `tunnel.osm` | 15xx / 150xx | `tunnel=yes layer=-1` covered way of 334 local units, between two at-grade approaches | One `is_tunnel` edge; nodes 1502 and 1503 stay single nodes of degree 2 with `layer_split_nodes == 0`; under `ridge_sampler()` over its middle four fifths and a road held at grade, exactly two portals, each well INSIDE its end of the edge. Under `RoadElevationSolver` instead, NO portal: see the note below |

## Notes on individual fixtures

### `t_junction.osm` is the important one

The junction detection this replaces clustered way **endpoints** within 2 m. Way
2010's shared node 203 is interior to way 2000, so endpoint clustering finds no
junction at all here. Only node identity does. If exactly one test survives from
this directory, it should be this one.

### `bridge_over.osm` is the inverse case

Endpoint clustering also invents junctions that do not exist. Node 702 is genuinely
shared in the data, but the ways are on different layers and never meet. Merging
them would produce a four-way intersection floating in mid-air where a road passes
under a bridge.

### `bridge_abutment.osm` is the inverse of the inverse

The layer split has to be conditional. `bridge_over.osm` shares a node that is
**interior** to both ways, which is a grade separation and must split. This fixture
shares nodes that every way merely **ends** on, which is the ordinary way a bridge
is tagged, and must not. Splitting unconditionally disconnects every bridge deck in
an extract from its approaches and reports four dead ends per bridge.

### `duplicate_node.osm` is a data defect, not a topology

Real extracts contain distinct node IDs at one coordinate. The zero-length stretch
between them cannot become a `GraphEdge`, because an arm with no direction is
useless to the junction solver, but the two IDs still have to resolve to one graph
node. Otherwise the way is cut in half at a point where the data says it is
continuous.

### `roundabout.osm` ring topology

Ring way 4000 lists node 401 twice, first and last. Nodes 402, 404 and 406 are
shape points referenced only by the ring, so they are not graph nodes; the ring
therefore splits into exactly three edges at 401, 403 and 405.

### `crossing.osm` is about the two shapes, not about one crossing

Node 1003 is referenced by one way and is not a way endpoint, so it never becomes
a `GraphNode`. Locating it means walking `GraphEdge::node_ids`, which is the only
route that works for the commonest shape in the data. Way 10020 is the other
shape and is deliberately SKEW, about 53 degrees off square, because a
desire-line crossing usually is and an implementation that only accepted a
perpendicular crossing way would drop it silently.

The doubly-mapped case, one physical crossing carrying both shapes at once, is
deliberately absent. Node 1012 splits way 10010 into two `GraphEdge`s, so the
node-derived and way-derived results could legitimately land on either of them
and the expected count would be ambiguous rather than wrong.

### Mind the units: local units are not true metres

`OSMParser` projects through EPSG:3857 Web Mercator and applies no latitude
correction, so **one local unit is one true metre divided by cos(latitude)**. At
the 53.3 N these fixtures sit at, that factor is **1.6744**. A way spanning
0.002 degrees of longitude measures 133 true metres on the ground and comes out
**222.6 local units** in `GraphEdge::polyline`.

Every width in the pipeline is a plain number of local units: `ProfileConfig`
lane widths, `DedupConfig::max_offset`, `CrossingConfig::stripe_width`,
`BridgeConfig::pier_spacing`. So a fixture author placing a node a known number of
TRUE metres from another gets an offset 1.67 times larger than intended, and any
expectation written in true metres is wrong by that factor. Place nodes by the
LOCAL offset wanted, and derive the degrees from it.

Worth flagging separately: this factor means a surveyed 3 m gap between two OSM
ways is built as a 5 m gap beside a road whose 7 m carriageway is still 7 units
wide. That is a pipeline-wide scale mismatch, not a property of these fixtures,
and it gets worse with latitude.

### `sidewalk_dup.osm` puts the footway inside the synthesised strip

The offset has to put the surveyed footway ON the synthesised one, or the
duplication the feature removes is not there to remove. `build_profile()` lays
out 3.5 of lane, 0.3 of gutter, 0.02 of curb face and 0.15 of curb top before the
sidewalk starts, so the synthesised strip spans 3.97 to 5.97 from the centreline.
The footway nodes are 0.0000270 degrees north, which is 3.01 true metres and
**5.03 local units**, landing in the middle of that band. A fixture author who
wrote five true metres would get 8.37 and miss the strip entirely.

The crossing footway is the control. It is a `highway=footway` too, so it passes
the `RoadType` filter `dedup_sidewalks()` uses, and it shares a node with the
carriageway so it is as near as any sidewalk. Only the bearing test separates
them.

### `dual_carriageway.osm` uses a primary, and a secondary as its control

`ProfileConfig::urban_dual_median` gives an untagged four-lane two-way PRIMARY a
median, on the reasoning that such a road is a dual carriageway far more often
than it is four undivided lanes. Secondary and below need an explicit `median=*`
or `divider=*`. Both rows are in the fixture, because a builder that gave every
wide road a median would pass on the primary alone.

### `tunnel.osm` carries no terrain, and could not

There is no terrain in an `.osm` extract, and `OSMParser::recenter_on_features()`
moves every coordinate, so a hill cannot be written into the file at all. The
ridge is supplied by `ridge_sampler()` in `tests/road/p5_p6_fixtures.hpp`,
positioned against the PARSED centerline of the tunnel edge and sized as a
fraction of it rather than in metres, for the units reason above.

The ridge is deliberately shorter than the covered way, covering the middle four
fifths of the edge, so the station where the road passes under the terrain is
well inside each end. A portal builder that used the edge endpoint instead is off
by that much.

**The elevation solver does not produce this shape.** `RoadElevationSolver` step
4 drops a tunnel edge until every one of its stations is
`ElevationConfig::tunnel_depth` below the terrain, taking the maximum demand over
the whole edge, so a solved tunnel is 8 m underground at its own ends and never
surfaces inside its own span. Under that solve `build_tunnel_portals()` emits no
portal at either end, by its own "already buried at that end" rule. That is a
conflict between P3 as implemented and the P6 header as frozen. The portal tests
in `tests/road/test_structures.cpp` therefore supply their own station heights,
and one test there asserts the solver's current behaviour explicitly so the
conflict is visible; its doc comment names it as the test to delete if the solver
is changed.

## Universal invariant

Every fixture must satisfy, for every `GraphEdge`:

    edge.polyline.size() == edge.node_ids.size()
    edge.polyline.size() >= 2

## Adding a fixture

1. Use a fresh ID block and a fresh patch of coordinates.
2. Tag it the way a real mapper would; no synthetic tag values.
3. Add a row to the table above stating what it must produce, then write the
   assertion in `tests/road/`. A fixture with no documented expectation is not a
   test, it is a file.
