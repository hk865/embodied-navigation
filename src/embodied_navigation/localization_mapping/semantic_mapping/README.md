# Provenance-bound semantic mapping

This CPU-only package owns a small observed map. It has no ROS, simulator,
detector, network or model-loading dependency. The live adapter supplies actual
sensor points, a same-time pose and per-point classification evidence. Root's
DynaNav adapter is responsible for the real sensor/topic wiring.

`LayeredMap` keeps durable static contributions separate from dynamic and unknown
points. `integrate()` always returns the complete owned navigation cloud when a
mask is missing, stale or incompatible; those points are quarantined as unknown
for static persistence. The existing obstacle consumer must keep receiving the
complete cloud. A static-only frontend topic must be a separate branch.

## Live adapter example

```python
from embodied_navigation.localization_mapping.semantic_mapping import (
    Observation, PointMask, LayeredMap, SemanticScheduler, STATIC, DYNAMIC, UNKNOWN,
)

store = LayeredMap(max_observations=512, max_points_per_observation=8192)
semantics = SemanticScheduler(interval_ns=2_000_000_000,
                              event_min_interval_ns=500_000_000)

# xyz and states/instance_paths MUST have the same final sampled-point order.
# Pixel projection, self masks and stride have already been applied consistently.
obs = Observation(epoch=epoch, sequence=sequence, source_ns=source_ns,
                  pose_source_ns=source_ns, points_sensor=xyz,
                  T_map_sensor=T_map_imu, anchor_id=f"frame:{sequence}",
                  frame_id="go2_imu", calibration_id="rgbd:v1",
                  pose_source="sim_oracle")  # Diagnostic GT map; not a SLAM result.
proof = PointMask(epoch=epoch, sequence=sequence, source_ns=source_ns,
                  produced_ns=source_ns, frame_id="go2_imu", calibration_id="rgbd:v1",
                  states=states, instance_ids=tuple(instance_paths), labels=labels,
                  provenance={"kind": "sim_oracle", "producer": "isaac.instances",
                              "instance_id_scope": "epoch"})
split = store.integrate(obs, proof, now_ns=source_ns)
# publish the full navigation source; do not replace it with static_points.
navigation_xyz = split["navigation_points_sensor"]
static_xyz = split["static_points"]
static_source_indices = split["static_indices"]  # also index the original RGB

# Schedule expensive/class annotations separately from the cheap frame mask.
if semantics.request(source_ns, epoch=epoch, event=novel_scene):
    # Submit one bounded asynchronous job; no inference runs in this package.
    # On completion/cancellation, release only the admitted source/epoch token.
    semantics.complete(source_ns, epoch=epoch)

compact = store.summary(now_ns=source_ns)  # no full-cloud materialization/history
store.save("scene_metadata.json", now_ns=source_ns, include_points=False)
# Once at final export, or when explicitly requested:
static_map_xyz = store.static_points()
objects = store.query(label="person", center=[0, 0, 0], radius=5,
                      now_ns=source_ns, include_history=False)
```

`pose_source="slam"` is for an actual SLAM pose. A simulator pose is explicitly
`sim_oracle`; mask provenance separately identifies `sim_oracle`, `learned` or
`geometric` plus its producer/version fields. Snapshot provenance reports oracle
use and never relabels an oracle result as learned perception. Matching assigned
headers is not proof of render acquisition timing; the sensor adapter must verify
actual RGB/depth timing and calibration. Truth-only scoring NPZ poses must not be
silently passed as SLAM estimates.

## Mask, identity and revision rules

- Mask epoch, sequence, source timestamp, frame, calibration and point count must
  match the observation. Production time must lie between measurement and the
  supplied current simulation clock, inside `max_mask_age_ns`. Missing/invalid
  evidence yields UNKNOWN, never an assumed STATIC label. A previous image's
  mask cannot classify a new image merely because the point counts agree.
- STATIC=1, DYNAMIC=2, UNKNOWN=0. A group containing dynamic points, or labelled
  person/pedestrian/human, is excluded from static integration as a whole.
  Standing people therefore remain movable. Labels and masks are evidence,
  not guarantees of detector recall or simulation performance.
- `instance_id_scope="epoch"` explicitly promises persistent source identity.
  IDs are bound by epoch, source kind, producer and source instance path: an
  oracle USD path keeps its internal object ID across occlusion, centroid drift
  and class updates. Distinct paths cannot be merged by spatial association.
  The default `frame` scope uses class-compatible 3D nearest association with a
  gate; ambiguous ties allocate a new ID. This is not general visual re-ID.
- Query objects and every history entry retain the original instance ID,
  epoch/frame/calibration, source timestamp, pose and mask provenance. Full XYZ
  association prevents equal XY on separate floors being merged by projection.
  No inferred room/stair traversability or real rigid-body velocity is claimed.
  `velocity` is the finite difference of observed visible centroids in revised
  map coordinates; partial visibility and pose correction can change it.
- `revise_mask(observation_id, proof, now_ns=...)` replaces that retained
  observation's contribution. When a tracked object becomes dynamic, older
  contributions associated with that object are also excluded. Rebuild a
  persistent export from `static_points()` after revision; permanently appending
  earlier delta clouds to an unrelated archive would defeat this separation.
- `update_anchor(anchor_id, T_map_sensor, revision=...)` requires an increasing
  pose revision, reprojects points and history and preserves object IDs. Use one
  source sensor/keyframe anchor per actual pose. A new clock epoch requires a
  new map segment; mixed epochs are rejected rather than spliced together.

## Bounded storage and artifact use

Call `integrate` on selected map keyframes, while keeping full-rate navigation
and mask publication independent. At 1–2 Hz, 90 seconds fits the default 512
observation cap; this is a capacity calculation, not a measured runtime budget.
The per-observation point cap is 8192; tracks are capped at 256 with 32 trend
entries each. Capacity rejection is counted and marks the map incomplete; it
does not drop the returned complete navigation cloud or evict accepted geometry.

`complete` concerns only the retained observations' mask coverage and capacity;
it never means the entire scene was explored or semantic perception is correct.
`mask_coverage_complete` says every retained point has valid non-UNKNOWN mask
evidence. The legacy `semantic_coverage_complete` name is kept as an explicitly
marked alias of that mask flag. Use `semantic_label_coverage_complete` to ask
whether retained known points also have instance IDs and non-unknown class
labels. Generic oracle labels such as `scene_geometry` can satisfy that label
flag; it does not establish learned semantics or detailed room/object meaning.

`summary()` produces compact counts, provenance and current objects, without
building/serializing the point map. `snapshot(include_points=False)` retains
observation/history evidence but omits full geometry. `save` atomically replaces
one self-contained JSON revision; its default includes geometry for small maps,
so live telemetry should explicitly use `include_points=False`. `read_snapshot`
loads that committed representation for agent queries/inspection. IDs persist
in exported artifacts; resuming mutable tracking from a checkpoint is not part
of this API. Query is read-only and grants no navigation/actuator authority.

## Agent query entry

```sh
python3 scripts/query_scene_map.py /data/run/scene_map.json --label person \
  --near 0 0 1 --radius 5
python3 scripts/query_scene_map.py /data/run/scene_map.json --include-expired --context
```

The default is deterministic JSON with filtered objects and original map/evidence
metadata; bulk static points are omitted. `--context` emits compact deterministic
structured text. The query uses the snapshot's recorded simulation query time,
not the machine's current wall clock; it does not claim the map is live. Filters
require a finite three-dimensional centre plus nonnegative radius in the recorded
map frame. Coordinate/clock/pose/mask provenance, UNKNOWN counts and capacity flags
remain visible. No missing floor labels, object relations or scene completeness
are inferred. `--include-expired` includes tracks already marked expired by the
snapshot owner; querying does not mutate the map or re-identify objects.

Only this package and its new CPU tests are owned by this implementation. Filtering
an archive after SLAM does not establish that FAST-LIVO's internal geometric and
visual maps excluded dynamic objects. Frontend mask admission and actual topic
wiring require their own integration and simulation checks.

## CPU checks

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m unittest discover \
  -s tests/unit -p test_layered_mapping.py -v
```

Tests cover quarantine, full navigation retention, late revocation, standing and
moving objects, source-scoped IDs, floor separation, pose reprojection, compact
atomic artifacts, capacity bounds and interval/event/epoch scheduling. They do
not run a semantic model, ROS, Isaac, native SLAM or a robot.
