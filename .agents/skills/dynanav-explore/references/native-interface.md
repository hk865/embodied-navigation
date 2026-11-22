# Native exploration interface and executable SOP

Use the actual checkout selected for the task (`${PROJECT_ROOT}`), preserving unrelated work. The bundled scripts have no run directory, ROS domain or waypoint coordinates baked in. All examples below use explicit variables supplied from the current session; do not substitute a past run or route.

## Set the generation once

Set `REPO` to the checkout, `SKILL` to this skill directory, `RUN` to the explicit generation root, and `EVIDENCE` to a writable directory for this generation. The native client takes **`$RUN/capture`**, while the SOP takes **`$RUN`**. The native mailbox currently calls its protocol run id `capture`; the SOP also records the outer generation name to keep otherwise identical mailbox names separate.

```sh
SOP="$SKILL/scripts/dynanav_sop.py"
python3 "$SOP" --help
python3 "$SOP" --repo "$REPO" --run-dir "$RUN" --evidence-dir "$EVIDENCE" status
```

For an existing run, verify its ownership/foreground session rather than starting a replacement. The text pointer `/tmp/embodied-navigation-native-exploration-current` may identify the latest launch, but it is not authority to switch an ongoing task to another generation: read the pointer's contents and verify the selected run.

Only when a new simulation experiment is authorized, choose a **new** run path and a **free explicit** ROS domain. Run this in a persistent foreground execution session:

```sh
python3 "$SOP" --repo "$REPO" --run-dir "$RUN" --evidence-dir "$EVIDENCE" \
  launch --ros-domain-id "$DOMAIN" --duration-sim-s 300 --speed-mps 1.0
```

Optional `--duration-sim-s 1800` remains finite. The original launcher enforces `8..1800` simulated seconds, defaults to 300, allocates `max(900, ceil(3*duration))` wall seconds, and owns its simulator/estimator/map children under the existing lock/cleanup. The current motion deadline is `duration−5s`; duration is not all motion time. Native speed requests must be within `(0,1] m/s`. Keep the original launcher and RouteTracker; no planner replacement or automatic goal queue is introduced. No resume/checkpoint option exists.

`SIMULATION_SCENE` and any session collision repair concern the renderer/physics environment only. Maintain repair identity and verification separately; do not inspect USD or transplant maintenance world coordinates into a target list. A session-layer repair may require separately authorized reapplication in a new simulation session.

## Record actual cloud/pose pairs

In the run's matching ROS environment/domain, start the finite **read-only** collector in a separate persistent session. Use an empty generation-specific `CLOUD` directory:

```sh
ROS_DOMAIN_ID="$DOMAIN" python3 "$SKILL/scripts/record_current_lio_cloud.py" \
  --run-id "$(basename "$RUN")" --output-dir "$CLOUD" \
  --budget-wall-s "$RECORDER_WALL_BUDGET" --save-every-wall-s 5
```

`RECORDER_WALL_BUDGET` is a finite positive wall-time budget chosen for this run; it is not the simulated duration. The ROS setup/build environment must make `rclpy` and the message types available. The collector subscribes only; it publishes no command. It pairs `/dynanav/rtx_livo/cloud_registered_full` with `/dynanav/rtx_livo/aft_mapped_to_init` by exact integer source stamp. These are LIO deskewed map-frame points with same-stage LIO poses; no extra pose transform is applied. The output refuses to append to a populated or differently labelled generation. Stop the collector by its finite budget or by creating `"$CLOUD/stop_recorder"`; this is not a robot stop.

Retained products are `measured_partial_cloud.npz/.ply`, `latest_paired_cloud.npz`, `paired_source_stamps.jsonl`, and `summary.json`. A collector started after the run began is a **partial interval**, not the full SLAM map. Run labels alone do not prove the ROS domain: verify the explicit domain/run ownership and source interval in `summary.json`.

## Observe, view and record gaps

```sh
python3 "$SOP" --repo "$REPO" --run-dir "$RUN" --evidence-dir "$EVIDENCE" \
  snapshot --tag "$TAG" --cloud-dir "$CLOUD" --roi-span 20 --z-span 12
```

This reads the official client and stores the unretimestamped raw observation, the actual front RGB/source record, operator-map metadata, each height chart separately, the copied measured cloud/summary, local and full XY/XZ/YZ/oblique views, and a provenance manifest. Axes use metres and the robot marker is the estimated child-pose origin, with measured yaw shown as a 1 m horizontal arrow. Side arrows are yaw projections, not inferred body pitch. The 3D renderer preserves distant measured returns in its full-domain view and records any display sampling; it never fills unknown space or makes a mesh/clearance certificate.

Actually open the RGB and both 3D multi-view PNGs; inspect the relevant per-height-chart PNGs for stairs, landings and overhead structure. Adjust the visual ROI as needed. Missing or inconsistent evidence is recorded rather than invented. Snapshot/view readiness, map age and gap bookkeeping do not become conditions in `goto`.

An absolute map-z filter is a display subset, not a physical-floor classification: estimator-map z can vary with position and overlap returns from another floor. When a lower/upper auxiliary view needs separation, use source-time windows before/during/after this generation's actually observed stair transition and its estimated trajectory, corroborated by RGB, together with a stated ROI. Label it as the observed interval/subset rather than claiming a complete physical floor; do not import a historical route or scene geometry.

Keep the complete retained input NPZ/PLY unchanged, including its existing voxel/downsampling provenance. For each filtered or sampled view, record the source generation/file/frame and source-time selection, ROI and other filters, input/included/excluded point counts, and sampling method/count. Here a raw/input count is the count before this view's filtering, not a claim to retain every original sensor return. State what timestamps represent (for example, paired-cloud source times or voxel first-seen times); a render time or first-seen stamp is not proof of physical-floor membership. Cropped, filtered, downsampled or no-return blanks do not assign free or unknown occupancy labels. These display rules support map reading only and add no `goto` admission condition.

The current map contract:

- `capture/interactive/observation.json`: post-VIO source time/frame/pose, yaw, command, active sequence, progress, failure and terminal reason.
- `capture/interactive/receipts.jsonl`: native request/arrival/cancel/stop/failure events. Read the original events, not only a tail or CLI exit code.
- `capture/interactive_frames/index.jsonl` and `rgb_*.png`: actual front RGB/source times; do not use an overview panel to choose targets.
- `capture/operator_map/latest.json`: measured accumulation, last-frame acceptance/reasons, level/floor evidence and export references. Floor classification is mapping evidence, not new native admission.
- `operator_grid/*json` and referenced NPZ: `ix,iy,code`, cell centre `(index+.5)*resolution_m`, code 0 free / 1 occupied; cells absent from this export are unknown in this grid. That grid definition does not label points omitted from a plot or absent sensor returns. Height charts can span stair/platform heights and are not floor counts. Use active/current context, retaining older same-generation charts as labelled history.
- Display `map` is a generic label; `odometry_map_frame` names the actual estimator frame. Do not invent a coordinate transform because these labels differ. LIO clouds arrive in their map frame under the source contract.
- `capture/interactive_waypoint_control.jsonl` records native commands and measured-input receipts. `capture/policy_frames.jsonl` includes simulator-native state; do not feed the whole file into target choice.
- `/dynanav/backend/status` (`std_msgs/String` JSON) and `$RUN/configuration/slam_backend_config.json`: frozen SLAM backend config (defaults to `configs/localization_mapping/slam_backend_stabilized.json`; override via `--backend-config`) and live read-only status (`adopted`, `loops`, `keyframes`, `config_path`, `stats.{travel_m,travel_since_last_loop_m,last_loop_age_s,zupt_active,zupt_full_stop_count,zupt_pivot_count}`).
- Direct bounded actions via `scripts/explore_waypoint_client.py`:
  - Panoramic multi-view `scan` (in-place slow `yaw` sweep requiring the default PCT/SCAN worker; pass wall-clock `--ttl` and `--timeout-per-step` matched to slow sim RTF; writes non-overwriting snapshots under `interactive/scan/seq_<first_seq>/`): `PYTHONPATH=src python3 scripts/explore_waypoint_client.py scan --run-dir "$RUN/capture" --pose-source <active_pose_source> --steps 6 --step-deg 60 --settle-s 0.8 --ttl 90 --timeout-per-step 90`.
  - Direct recovery `maneuver` (bypasses PCT/SCAN and rotation-space checks when wedged against a door leaf/corner; run `cancel` and verify `cancel_accepted` first): `PYTHONPATH=src python3 scripts/explore_waypoint_client.py maneuver --run-dir "$RUN/capture" --pose-source <active_pose_source> --forward -0.35 --yaw-rate 0.15 --duration 1.5 --wait`. Verify `maneuver_accepted` and `maneuver_complete` receipts and fresh RGB/cloud before resuming `goto`.
- Multi-floor PCT grid budget & 2D frontier audit: pass `--pct-max-grid-cells 8000000` to `run_dynanav_native_exploration.sh` for two-floor runs, and run `PYTHONPATH=src python3 scripts/dynanav_exploration.py candidates --run-dir "$RUN/capture" --frontier-json "$RUN/capture/operator_map/level_<key>.json" --frontier-z <z> --lidar-offset 0 0 0 --pct-max-grid-cells 8000000` (read-only advisory) before leaving a floor. See [two-floor-exploration-prompt.md](two-floor-exploration-prompt.md) for the reusable multi-floor mission prompt template.

Record a meaningful remaining gap or decision with current evidence references:

```sh
python3 "$SOP" --repo "$REPO" --run-dir "$RUN" --evidence-dir "$EVIDENCE" \
  note --kind gap --state open --evidence-tag "$TAG" --text "$GAP_FROM_CURRENT_SENSORS"
```

Do not infer room completeness from point count, trajectory, mapped area or chart count. Record which branch/landing/side remains unseen, what evidence is missing, and whether a revisit actually resolved it. The SOP does not select a next target.

## Send one chosen target and verify the execution

Coordinates are a deliberate current-sensor decision in the current post-VIO map frame; `Z` is the estimator child-pose origin, not a measured floor height. One operator owns the atomic request and increasing sequence state.

```sh
python3 "$SOP" --repo "$REPO" --run-dir "$RUN" --evidence-dir "$EVIDENCE" \
  goto --x "$X" --y "$Y" --z "$Z" --timeout 20 \
  --evidence-tag "$TAG" --note "$CURRENT_SENSOR_BASIS"
```

The wrapper invokes the **official** `scripts/explore_waypoint_client.py goto --pose-source livo_post_vio` once, records its payload/seq, then reads native receipts. `--ack-only` returns at `waypoint_accepted`, explicitly **not arrival**; otherwise the wrapper waits for `waypoint_reached`, failure or timeout. A timeout does not cancel the action. Continue observing the **same** sequence:

```sh
python3 "$SOP" --repo "$REPO" --run-dir "$RUN" --evidence-dir "$EVIDENCE" \
  wait --action goto --seq "$SEQ" --timeout 20
```

The official client's `goto --wait` and the SOP both recognize native `waypoint_failed` for the exact sequence. It reports the goal's failure, commands zero and returns idle while simulation/SLAM continue. Active/queued goals clear, with closing receipts for dropped queued choices; no automatic retry occurs. `waypoints.last_failure` and `failed_goals` retain the failed choice even after a newer request. Run `failure`/`terminal_reason` are not set by a single-goal failure. Explicit stop and the finite run budget still terminate. Neither a failed wait nor a CLI timeout authorizes an automatic repeat. If submission itself is uncertain, inspect `request.json`, `operator-state.json`, raw receipts and observation before deciding anything further.

The adapter does not use heartbeat, legacy TTL, floor clearance, map/coverage/perception readiness or model proofs as waypoint admission. It still checks original measured source, frame, finite values and source freshness; stale input yields zero without a forged timestamp. RouteTracker keeps its original turn, arrival and stall logic. Do not add another direct velocity/yaw controller.

## Cancel, stop and close honestly

```sh
python3 "$SOP" --repo "$REPO" --run-dir "$RUN" --evidence-dir "$EVIDENCE" cancel --timeout 20
python3 "$SOP" --repo "$REPO" --run-dir "$RUN" --evidence-dir "$EVIDENCE" stop --timeout 20
python3 "$SOP" --repo "$REPO" --run-dir "$RUN" --evidence-dir "$EVIDENCE" verify-exit
```

Use the action needed; these lines are not an instruction to cancel before every stop. Native `cancel_accepted`/`stop_accepted` must match the exact mailbox run id and submitted seq. The official stop/cancel CLI has no wait option and its exit 0 confirms only submission. Cancel clears active/pending goals and returns idle. Stop begins normal terminal parking; ack alone is not parking or process exit. The original parking logic requires at least 5 s after terminal, native linear speed `<0.02m/s` and angular speed `<0.05rad/s` held at least 1 s, with final `navigation_terminal_and_parked` evidence. Then verify the retained foreground return, `capture_exit.txt`, lifecycle cleanup, and owned children. The SOP never kills an uncertain/shared process; PID reuse can require checking the retained session.

After the run, inspect clearly labelled native diagnostics separately from navigation evidence. In static-native mode the body speed, attitude and position envelope thresholds are diagnostic only; body-contact handling follows the configured `body_contact_report_only`. An actual fall, body contact/support failure, motor fault, non-finite state, clock fault or stall must be reported even if an envelope assertion did not fire. Preserve motor, clock and finite checks and original RouteTracker behavior.

The journal distinguishes submission, acceptance, reached, failed, cancelled, terminal, pending/uncertain and stop ack. Report requested speed ceiling versus measured speed, estimated motion, capture/parking outcome and remaining coverage gaps separately. Preserve failed attempts; never call a new spawn checkpoint continuation.
