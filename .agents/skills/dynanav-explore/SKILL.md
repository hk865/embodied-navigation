---
name: dynanav-explore
description: Autonomously explore a static Isaac DynaNav scene using current RGB, measured RTX/LIO clouds, SLAM maps and post-VIO poses, choosing sparse waypoints for the existing native RouteTracker. Use for sensor-driven simulation exploration, not real-robot actuation or oracle-map route replay.
---

# Explore DynaNav from live sensors

Current project update: the launcher now defaults to measured PCT global + native SCAN
local planning, continuous curve following, and the frozen Phase 4 stabilized SLAM backend
preset (`configs/localization_mapping/slam_backend_stabilized.json`, copied into
`$RUN/configuration/slam_backend_config.json`; override via `--backend-config`). Live backend
observability is published on `/dynanav/backend/status` (`adopted`, `loops`, `keyframes`,
`config_path`, `stats.{travel_m,travel_since_last_loop_m,last_loop_age_s,zupt_active}`).
See the repository's `docs/dynanav-pct-scan.md` and `docs/dynanav-waypoint-utility.md`. The
read-only `python3 scripts/dynanav_exploration.py candidates --help` prepares DSH candidates;
DSH still selects and explicitly sends goto. It never dispatches or adds goto gates.
Native RouteTracker-specific descriptions below apply to explicit
`NAVIGATION_BACKEND=native_route`; sensor, source and receipt rules still apply.

Replace the human waypoint choice, preserving `NativeWaypointTracker → RouteTracker → RoboGauge` and its native control law. This skill does not authorize starting another experiment or operating real hardware. Read [the native interface and executable SOP](references/native-interface.md) for the current checkout's commands, evidence products and receipt semantics, and [the reusable two-floor exploration prompt template](references/two-floor-exploration-prompt.md) when launching an autonomous multi-floor exploration session.

## Bind the current generation

- Check run ownership before starting a simulator; use a new run directory and free ROS domain. Keep the finite run in a persistent foreground session. Default requested command ceiling is **1 m/s**; it is not achieved cruising speed. The native entry accepts **8..1800 simulated seconds** (default 300); motion ends before the final parking allowance. There is no checkpoint/resume interface: a fresh spawn is a new generation.
- Choose targets using only this generation's actual front RGB, measured RTX/LIO cloud, accumulated measured SLAM occupancy and post-VIO estimated pose. Do not read simulator truth, original USD geometry, scene tomograms, older maps, old coordinates or historical routes to choose targets. A simulator may load a scene to render sensors; that is not a chooser map input.
- Keep scene/collision maintenance separate from exploration decisions. Preserve an authorized, verified session repair as environment provenance; do not turn repaired prims, world coordinates or a maintenance inspection into navigation evidence. Starting a new session does not prove that its session-layer repair persisted.
- Map LIO deskewed clouds with same-stage, exact source-stamp LIO poses, without a second transform. Navigate with post-VIO. Record generation, map/child frames, source times and partial collection interval. Never silently align generations or retimestamp stale poses.

## Observe in 3D, use 2D as an aid

Read the existing client's observation, then actually view the front RGB image named by `sim_frames` and its source record. Inspect the current measured cloud as **XY top, XZ and YZ side projections, and oblique 3D**, with metre axes/scale and estimated robot pose-origin/heading. Retain both a robot-centred local ROI and the full measured cloud domain so the crop cannot hide a distant branch or upper structure. The supplied recorder/renderer and parameterised [SOP script](scripts/dynanav_sop.py) produce these artifacts without consulting scene geometry.

Also inspect the current operator map and per-height-chart 2D exports. Relate them to the 3D returns and RGB to assess a stair flight, landing, overhead obstacle or opening. Chart indices are height groups, not a physical floor count; do not merge projected floors into one traversable plane. Grid cells absent from an export remain unknown in that grid; missing sensor returns or points omitted from a display do not by themselves label space free or unknown. Surface/floor z is not the estimated child-pose-origin z used by a waypoint.

A fixed absolute estimator-map z band does not identify a physical floor: map z can vary along a floor and overlap another floor's returns. For a layered auxiliary display, use this generation's source-time windows before/during/after actually observed stair crossings, estimated trajectory and RGB, with an explicit ROI, to identify the displayed subsets. Preserve the complete retained input cloud and its existing voxel/downsampling provenance; record source intervals, filters, input/included/excluded point counts and display sampling provenance. Here a raw/input count means points before this view's filtering, not every original sensor return. A blank caused by those display choices has no free/unknown occupancy meaning; these display rules are not goto admission checks.

Map age, missing views, partial cloud intervals, floor classification and coverage gaps inform judgment and reporting. They are **not new goto admission checks**. If evidence is absent or inconsistent, state that limitation and reread the current sources; do not fabricate geometry or add a gate to the native execution path. Existing native source/frame/freshness checks remain authoritative.

## Decide, send and learn

Choose a useful visible opening, nearby vantage or unexplored branch from current sensors; prefer a clear native line segment and a small number of meaningful waypoints. The follower is not an obstacle-avoidance planner. A door appearance does not establish passage; point projections do not certify support, clearance or room completeness. Use the side/oblique views for stairs and landings while keeping target choice grounded in the current RGB/cloud.

Express each target in the current estimator map frame. For a deliberate body-relative `(forward,left)` displacement, use measured yaw: `dx=forward*cos(yaw)-left*sin(yaw)`, `dy=forward*sin(yaw)+left*cos(yaw)`. Add these to the current measured pose. Target z refers to that same estimated child-pose origin, not world/floor z. Never replay an earlier route or reuse an old generation's coordinates.

Send normal exploration decisions with `scripts/explore_waypoint_client.py goto --pose-source <active_pose_source>`. Use `cancel` to abort a stalled or undesired active leg, and `stop` to terminate the run cleanly.

### Active sensing, ZUPT pause hygiene & multi-view 360° `scan`

- **ZUPT-friendly pauses during thinking**:
  - When pausing to inspect RGB, 3D clouds, or operator maps between waypoints, keep the robot idle (do not send tiny micro-nudges). Under the default `slam_backend_stabilized.json` preset (`zupt_enabled=true`, `zupt_full_stop_rot_rad=0.0`, `zupt_skip_loop_search=true`), the SLAM backend applies geometric `pivot` zero-translation anchoring whenever LiDAR geometry supports zero translation (`stats.zupt_active` / `stats.zupt_pivot_count`), bounding backend map drift during pauses. Note that default control still navigates on raw `livo_post_vio` (unless `--backend-pose` is explicitly enabled), so avoid unnecessarily long idle waits.
  - Avoid parking `< 0.6 m` nose-first facing a blank, textureless wall for `> 15 s`; stop in a position with open 3D wall/floor geometry and visible visual texture whenever practical.
- **Stop-and-spin panoramic `scan` for multi-view visual coverage & loop closure**:
  - At key topological anchors — entering a new open room center, arriving at a multi-way junction or stair landing, or when `/dynanav/backend/status` reports `stats.travel_since_last_loop_m > 15.0` — stop cleanly and run an in-place slow 360° scan (`<= 0.15 rad/s` per step; requires the default PCT/SCAN worker for rotation-space clearance; pass generous wall-clock `--ttl` and `--timeout-per-step` matched to slow simulation RTF when RViz/rosbag recording is active):
    `PYTHONPATH=src python3 scripts/explore_waypoint_client.py scan --run-dir <capture> --pose-source <active_pose_source> --steps 6 --step-deg 60 --settle-s 0.8 --ttl 90 --timeout-per-step 90`
    (or `--steps 8 --step-deg 45 --settle-s 0.8 --ttl 75 --timeout-per-step 75` for smaller per-step turns).
  - Because `zupt_skip_loop_search=true` suppresses loop queries during stationary `pivot` frames, a panoramic `scan` primarily enriches multi-heading RGB + LiDAR keyframes and visual evidence; loop closure itself fires once the robot resumes moving into previously visited space. Each `scan` writes non-overwriting per-step snapshots under `interactive/scan/seq_<first_seq>/`.
  - Inspect the resulting `sim_frames` / `scan_dir` snapshots before picking the next branch. Do not run `scan` inside a narrow doorway or `< 0.6 m` from an obstacle where rotation-space clearance would block `yaw`; step into open space first (or use `maneuver` to clear the doorway).
- **Visual-First annotated 2D occupancy map & zero custom probe scripts**:
  - `dynanav_sop.py snapshot` automatically overlays Go2's real body footprint (`0.70 m × 0.31 m` rotated by `yaw`), the `0.60 m` in-place turning safety circle (green = clear for `scan`, red = tight), the same-floor visited trajectory, and programmatically projected 2D-reachable candidate markers (`F1..F6` unvisited frontiers, `W1..W4` open directions) onto BOTH the global floor map (`height_chart_000.png`) and a `±6 m` zoomed close-up (`height_chart_000_local.png`), and prints their exact `(x, y, z)` table in `products.height_charts[].candidates` alongside `products.backend_status`.
  - **Do NOT write custom ad-hoc Python scripts to inspect `.npz` files or run manual `PctPlanner` grids.** Decide directly from Front RGB + Global & Local annotated 2D maps + the printed `F1..F6 / W1..W4` candidate table, then send one navigation command (`goto` / `scan` / `maneuver`).
- **Multi-floor PCT grid budget & direct `maneuver` stair climbing**:
  - For full two-floor runs (including the large upper hall & rooms), launch `scripts/run_dynanav_native_exploration.sh` with `--until-stopped --backend-pose --pct-max-grid-cells 8000000` (giving an explicit `3600 s` target sim budget in the mission prompt and connecting the stabilized SLAM backend directly to navigation).
  - Once Front RGB + the 2D map confirm arrival at the stairs and you decide to climb, **bypass PCT/SCAN stair checks and climb directly with `maneuver` pulses** for fast response:
    1. Align the robot nose with the stair centerline using short `maneuver --yaw-rate ±0.15 --duration 1.0 --wait` pulses while checking Front RGB;
    2. Climb smoothly with `maneuver --forward 0.45 --yaw-rate <±0.05 centering> --duration 2.0 --wait` pulses (~`0.9 m` per pulse), checking Front RGB after each pulse to stay centered between the handrails;
    3. At the mid-landing, align with the second flight via short `maneuver` yaw pulses, then climb onto the upper floor landing; once on the flat upper floor, resume normal `goto` + `F1..F6` visual exploration.
- **Closed-loop room sweeps**:
  - When exploring a large room or hallway loop, prefer closing a physical loop back through a previously observed doorway/junction rather than purely dead-reckoning open spurs, so the SLAM backend closes geometric/visual loops and bounds global drift.

### Anti-trapping & direct recovery (`maneuver`, bypasses PCT/SCAN)

- **Avoid narrow-door / low-ceiling traps proactively**:
  - When entering or exiting a hinged door, place intermediate waypoints on the **doorway centerline** (at least 1.0 m before and 1.0 m past the threshold) rather than hugging the door leaf hinge or cutting sharply behind an open door panel.
  - Never drive into wedges where overhead clearance narrows (e.g., under-stair sloped ceilings); exit such corridors along the reverse centerline in short legs.
- **Direct recovery when wedged against a door leaf or tight corner**:
  - If `goto` stalls or `yaw` is blocked by rotation-space occupancy, follow `cancel -> maneuver -> observe -> resume goto`:
    1. Clear any active goal first: `PYTHONPATH=src python3 scripts/explore_waypoint_client.py cancel --run-dir <capture>` and verify `cancel_accepted` (pause ~0.5 s or check `observe` so the single-slot mailbox is processed before submitting the next request).
    2. Invoke the native **direct body-velocity recovery** action `maneuver` (bypasses PCT/SCAN planning and rotation-space checks; bounded to `|forward| <= speed`, `|lateral| <= 0.05 m/s`, `|yaw_rate| <= 0.15 rad/s`, `duration in [0.1, 2.0] s`; mild contact with door panels is tolerated by `body_contact_report_only` in simulation):
       `PYTHONPATH=src python3 scripts/explore_waypoint_client.py maneuver --run-dir <capture> --pose-source <active_pose_source> --forward -0.35 --yaw-rate 0.15 --duration 1.5 --wait`
    3. Verify `maneuver_accepted` and `maneuver_complete` in receipts, then run `observe` and check front RGB/cloud before chaining another pulse.
  - Chain 1–3 short `maneuver` pulses (e.g. straight reverse `-0.35`, then reverse+yaw `-0.25 / ±0.15`) until `observe` + RGB confirm clear separation from the door leaf/corner, then resume normal `goto` exploration. Do not abandon a multi-floor run solely because a door-leaf corner blocks in-place `yaw`.

Record request sequence, acceptance, measured progress/command and the closing receipt. **A request file or client exit 0 is not an executed/reached goal.** Native `stop_accepted` and `cancel_accepted` must match the mailbox run id and sequence. Stop ack starts terminal parking; verify parking, capture exit and owned-process cleanup separately. Cancel ack clears active/queued work and permits another decision.

On timeout or failure, inspect the existing sequence's raw receipts and `observation.failure/terminal_reason`; do not blindly resubmit. A native `waypoint_failed` closes only that sequence: command becomes zero, active/queued goals clear, and simulation/SLAM continue in idle. `waypoints.last_failure` retains its reason and evidence. Refresh actual RGB, cloud/3D and the relevant height charts, then explicitly select a new goal. A CLI action failure does not mean capture exited. Explicit stop and the finite run budget remain terminal. Preserve the original RouteTracker arrival/turn/stall behavior and stale-measurement zero command.

## Keep an honest execution and coverage record

Maintain a concise current-generation journal: observation/view references and source times; selected opening/vantage and evidence; seq/ack/reached/failed/pending outcomes; and remaining gaps (unseen branch, occluded landing, unvisited side, uncertain support, or unavailable sensor evidence). Resolve a gap only when new sensor evidence supports it. This ledger supports decisions and review; it is not a waypoint gate or an automatic coverage score.

Static-native body speed, attitude and position envelope thresholds are diagnostics. Preserve motor, clock, finite-state and native measurement checks. Do not silently restore a speed/tilt/height admission gate; do not treat disabled envelope assertions or `body_contact_report_only` as evidence of physical success. Explicitly report actual falls, body contact/support failure, stalls and interrupted runs. Native physical state belongs in labelled after-run diagnostics, not online target selection.

Finish with an acknowledged stop, native parking and confirmed capture exit/cleanup. Report requested ceiling versus measured speed, accepted versus reached goals, estimated displacement and newly observed area separately. A trajectory, point footprint, height-chart count or path length is not full-building/room coverage. State remaining gaps and partial collection limits; do not claim live success from CPU checks or a fresh run as checkpoint continuation.
