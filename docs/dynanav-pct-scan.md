# Measured PCT global + native SCAN local navigation

This integration changes the static DynaNav exploration launcher default to
`NAVIGATION_BACKEND=pct_scan`. It preserves RTX complete-scan FAST-LIVO2,
post-VIO navigation feedback, the existing goto/cancel/stop mailbox, and the
RoboGauge policy. `NAVIGATION_BACKEND=native_route` explicitly selects the old
straight-segment RouteTracker for diagnostic comparison.

## Data and execution path

- `planning/pct/measured_tomogram.py` builds a multilayer support grid from this
  worker process's accumulated measured map-frame XYZ only. Actual source-stamp
  matched LIO cloud/pose are validated by `simulation/dynanav/pct_scan_worker.py`.
  The LIO cloud is already map-transformed and is not transformed again.
- Each populated support cell is a measured vertical cluster with measured XY
  neighbors. Unknown cells stay blocked in strict mode. The explicit near-start
  model below is the only inferred support exception. The trajectory is provenance, not a
  way to invent missing support. No scene tomogram, triangles, simulator truth,
  previous-run map, goal snapping or direct-line fallback is loaded.
- The actual vendored native PCT A* executable searches that grid. There is no
  minimum goal-distance rule. Its query `(layer,col,row)` and result
  `(layer,row,col)` conventions are converted explicitly. Support elevations
  remain metric, rather than interpreting a layer index as a physical floor.
- The body/IMU-origin support offset is explicit configuration. The launcher
  default is nominal base-support 0.32 m plus recorded IMU mount z, giving
  0.3148910446837544 m. This is a robot-height assumption, not measured floor
  certification or a scene prior. `PCT_BODY_SUPPORT_OFFSET_M` overrides it.
- RTX cannot directly observe floor in the immediate sensor blind region. The
  launcher explicitly opts into `PCT_START_BLIND_RADIUS_M=1.5`; setting it to 0
  retains strict measured-only support. A robust plane is fitted to surrounding
  current-generation measured support, with at least 20 cells/four quadrants,
  p90 residual <=3.5 cm and slope <=0.15. Only empty, start-connected cells within
  the radius may be inferred. Measured cells remain unchanged and incompatible
  measured heights retain a one-cell exclusion halo. The inferred cell list and
  fit evidence are recorded with `observed_free=false`, `support_certified=false`.
  This assumes the observed surrounding plane continues under the blind region;
  an invisible hole cannot be ruled out. It is not generic unknown-gap filling.
- The worker owns a native SCAN child for the active request. It publishes the
  global PCT reference, measured post-VIO body pose, and paired LIO sensor/cloud
  inputs. SCAN's Path interface adds `grid_map.body_height` to reference z, so
  the worker subtracts that same offset at this interface only.
- Only genuine native Bspline output associated with native trajectory metadata,
  the current run/request epoch and committed reference stamp is sampled into
  `planner_path.json`. B-spline evaluation uses its knot vector, not straight
  lines between control points. A source receipt binds the binary to this
  checkout's vendored SCAN tree; stale/unverified sibling binaries are refused.
- `PctScanWaypointTracker` keeps the native measured-pose/frame/freshness and
  request receipt behavior. `ContinuousScanFollower` projects onto the current
  sampled curve and tracks a spatial lookahead. Samples are not intermediate
  goals and do not invoke capture/brake/turn stops. Only the final operator goal
  uses measured XYZ proximity and a stationary source-time dwell.
- Replans replace the local curve without resetting the operator goal. A local
  horizon ending is not goal completion and is never extrapolated. Native
  emergency-stop output revokes the current path, including a command cached on
  a duplicate odometry tick. Missing planner output cannot cause direct-line
  motion. Cancel clears execution; stop still enters normal terminal parking.

A goal is acknowledged before planning. Missing measured support or a planner
failure is reported as a planning outcome, not an added goto admission check.
No heartbeat, action TTL, map-ready, floor classification or model-proof gate
was added.

## Native limits that remain

PCT is currently a point-footprint global planner. It does not certify load
bearing, robot footprint clearance, or eliminate its upstream diagonal corner
cutting. Sparse real support measurements can yield no route. The start-region plane
model is a bounded inference with explicit provenance, not an observation or
physical-support guarantee.

SCAN uses its native inflated **3D voxel occupancy**, but z optimization is
constrained to the linear reference profile and its z gradients are zeroed. It
is not unconstrained 3D height avoidance. Changing global route elevation is a
PCT/reference task; collision checking is not proof that the robot can traverse
stairs or retain support.

The sidecar validates exact-stamp LIO pairs, but native SCAN's separate cloud
and sensor-pose callbacks consume the latest pose. There is no native
source-pair-consumption acknowledgment; adapter pairing must not be described as
proof of exact child callback pairing. Native global collision/path safety and
real-time control are not certified by CPU or synthetic ROS tests.

## Launch and evidence

Run from the repository root in the normal configured ROS/Isaac environment:

```sh
DURATION_SIM_S=120 ROS_DOMAIN_ID=223 \
  bash scripts/run_dynanav_native_exploration.sh /new/external/run/path
```

The launcher builds the current six-package SCAN closure outside the checkout,
sources its generated ROS messages, owns the worker and capture/estimator/map
processes, and keeps the finite renderer in its persistent foreground session.
Use a free Fast DDS domain within 0..232. The worker's build-only operation does
not create the future capture output directory.

Per-run `capture/interactive/` adds:

- `planner_request.json`: active goal, estimator frames, run and sequence.
- `planner_path.json`: native spline samples, revision, exact native metadata
  and binary/source provenance, or explicit pending/hold/failed status.
- `planner_measured_map.npz`: current-generation measured voxel representatives,
  paired source stamps, measured trajectory and provenance, for observation and
  review. It is not a simulator map or a waypoint prerequisite.
- `scan_<seq>.log`, `scan_parameters_<seq>.yaml`: actual owned native child log
  and parameters. The parent run has `planner.log` and `planner_build.log`.

The existing observations, receipts, control/policy journals, front RGB,
operator maps and terminal parking records remain separate evidence. A native
path publication, accepted goal, CLI exit zero or CPU pass is not a reached
physical goal or full exploration coverage.

## Verification

Pure execution tests are in `tests/unit/test_dynanav_pct_scan_waypoints.py`.
Measured-grid tests in `test_measured_pct_tomogram.py` run the real C++ PCT A*,
including asymmetric axes, unknown gaps, obstacle detour and overlapping layers
with a body-origin goal at z=4 m. Worker tests cover spline evaluation and active
request association. Synthetic ROS smoke validates real native SCAN output and
replans, but uses synthetic messages and is not live-map evidence.

Live Isaac results, physical contacts, measured speed, reached goals and any
blockers must be recorded with the corresponding run, not inferred from the
implementation or synthetic fixtures.
