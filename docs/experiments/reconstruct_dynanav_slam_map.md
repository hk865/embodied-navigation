# Reconstruct a map from recorded LIO estimates

`scripts/reconstruct_dynanav_slam_map.py` is an offline CPU adapter for the
existing `LayeredMap`. It reads selected RGB-D keyframes and a finalized SQLite
ROS bag. Each retained keyframe must have its own measured
`nav_msgs/msg/Odometry` estimate on
`/dynanav/lio_observer/aft_mapped_to_init`, with parent frame
`dynanav_lio_map` and child frame `go2_imu`. The current LIO capture uses identity
LiDAR-to-IMU extrinsics, so that child pose transforms the archived `go2_imu`
points. This tool performs no simulator launch, bag replay, localization, or
control.

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/reconstruct_dynanav_slam_map.py \
  --keyframes /data/capture/mapping/keyframes \
  --bag /data/observer/bag \
  --observer-label pre-time-origin-fix-first-flight \
  --observer-metadata /data/observer/launch.json \
  --output /data/estimated-map-new
python3 scripts/query_scene_map.py /data/estimated-map-new/scene_map.json \
  --include-expired --context
```

The output directory must not exist. Use an explicit run/build label and pass
its actual launch/build metadata with `--observer-metadata` (repeatable). The
label is recorded as a user-supplied identity, not interpreted as evidence of
localization quality. Different estimator versions must use different output
directories and labels.

Pairing uses the odometry **header sensor timestamp**, never bag receipt time.
Exact matching is the default. An explicit `--stamp-tolerance-ns` between 0 and
1,000 permits only numeric timestamp roundoff. There is no interpolation,
extrapolation, or truth-pose fallback. Ambiguous equidistant estimates,
duplicate odometry timestamps, bad poses, mismatched frames, and unmatched
keyframes are rejected and counted. Initialization keyframes without estimates
are skipped. A run with no matched keyframes produces no map.

`Observation` requires equal sensor and pose timestamps. For an explicitly
accepted tiny numeric delta, the adapter uses the sensor timestamp for that
contract and retains the actual odometry header timestamp and signed delta in
both mask provenance and the pairing receipt. It does not change the measured
pose. The complete per-keyframe point order, mask states, and source instance
paths go into `LayeredMap`; its movable-object rules can strengthen a point
group to dynamic.

Only `points_sensor`, masks, identity, timestamps, and recorded semantic metadata
are read from keyframe archives. The simulator `T_map_sensor` field is never
accessed. New archives can carry `labels_json` and `mask_provenance_json`; the
adapter validates their instance oracle provenance and uses only recorded
classes. Older archives lack label metadata: their absolute paths receive the
explicit generic fallback `scene_geometry`, except the known
`/World/DynamicPedestrian` root and descendants receive `person`. Override or
repeat `--pedestrian-root` only to describe a known captured root. The receipt
distinguishes recorded labels from this legacy fallback. No detailed object or
room classes are guessed from USD names.

The new directory contains:

- `static_map.npz`: voxelized static points in `dynanav_lio_map`, with estimated
  pose and semantic oracle flags.
- `scene_map.json`: compact `LayeredMap.summary()` with source identities,
  current objects, coordinates, provenance, and coverage counts. Bulk geometry
  and object histories are omitted; the existing query CLI can read it.
- `receipt.json`: every keyframe's match/skip reason, source/odometry timestamp
  and delta, input/code SHA-256 digests, metadata hashes, decoder counts,
  observer label, and output hashes.

Inputs must stay finalized throughout reconstruction. A live/uncheckpointed
SQLite WAL or changing source digest is rejected. Files are opened read-only.
Every output explicitly states `no_GT_pose_used=true`,
`semantic_oracle=true`, and `complete_full_scene=false`. Producing these
artifacts does not verify localization accuracy or the estimator's internal
dynamic filtering. Generic oracle label coverage does not establish learned
perception or complete scene coverage.

Pure pairing and reconstruction contract tests do not import ROS:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m unittest discover \
  -s tests/unit -p test_dynanav_slam_reconstruction.py -v
```
