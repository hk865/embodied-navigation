# Native controller autonomous exploration, 2026-11-11

## Outcome

Run `native_exploration_20261111T075212Z`, ROS domain 220, completed **five model-selected waypoints**, followed by stop seq6 and normal terminal parking. The model actually viewed this generation's front RGB images and read current measured occupancy and post-VIO poses before choosing each target. It progressively discovered the vending-machine-side corridor entrance from the lobby. This is a finite lobby/corridor-approach prototype, not complete basement, stair, room or building exploration.

Run root: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/native_exploration_20261111T075212Z`.

- seq1–5: five `waypoint_accepted` and five `waypoint_reached` receipts, no rejected/expired waypoint.
- Native controller reached receipts at current simulation times 59.435, 141.395, 191.535, 232.555 and 276.515 seconds. Targets were chosen online, not preloaded.
- Stop seq6 produced 5.10 seconds of zero policy commands, `end_reason=navigation_terminal_and_parked`, arrived=true and failure=null. Capture exit0; 58,880 physics steps, final policy time294.395s, loop365.978wall seconds. All recorded owned PIDs and SDK/native-estimator processes were absent after cleanup.
- Settled one-second endpoint windows give estimated net XY **7.790895m**, independently native-scored net XY **7.777343m**. These are net displacements, not total path length or explored area.

## Exact reused chain and removed additions

The previously successful double-stair DynaNav path was `SlamRouteTracker → RouteTracker → RoboGauge`, not the separate SCAN/PCT stack. Its old reference used measured repeat-spawn goals, but the historical static-cloud frontend still had oracle classification; this prototype does not reuse that frontend or any old route.

The current static entry is:

`live RTX + front RGB → FAST-LIVO2 post-VIO → NativeWaypointTracker → existing RouteTracker → existing RoboGauge policy`.

`NativeWaypointTracker` is only a mailbox/measured-pose adapter. It passes the model's live-estimator-frame target and current measured pose to the original RouteTracker, without manufacturing a historical SLAM reference or aligning to world coordinates. A reached target returns idle for the next target. Native bearing-error turning, arrival/braking, source-time dwell, stall and command bounds remain unchanged.

The static entry no longer invokes the later InteractiveWaypointTracker or its perception/floor-clearance rejection, heartbeat, action TTL, model-proof/readiness or extra segment restrictions. The existing client goto/cancel/stop mailbox is reused. Legacy TTL metadata does not expire a native goal. Original pose/source/frame/age checks and original physical/policy checks remain. Two stale pose ticks, totaling0.04s, correctly commanded zero.

A preceding 0.5m/s integration attempt, `native_exploration_20261111T074910Z` / domain219, accepted the model's first target but stopped at41.715sim. The later RTX camera check compared source41.6s renderer metadata to current41.715s body mount; its roughly4.4cm cross-epoch residual exceeded3cm during movement. This is not a same-epoch extrinsic error. For native static exploration that residual is now explicitly diagnostic-only, while original per-RP event/buffer binding and timestamps remain unchanged. The failed run is retained; its exit1 is not counted as navigation success. This change does **not** certify exact pixel-time physical camera poses or perfect camera latency.

## Sensor-only decisions and truth boundary

This run used real rendered front RGB, official RTX LiDAR returns, measured same-stage LIO clouds/poses for occupancy, and live post-VIO navigation poses. `semantic_provider=none`, internal oracle mapping disabled, map `ground_truth_used=false`. No original USD geometry, scene tomogram, world route, simulator truth pose or previous generation's map was used to choose waypoints. The simulator still loads its world to generate physics and sensor data.

Original policy proprioception and physical protections still consume simulator state, as in the established controller. Native trajectory/velocity was inspected only after the model finished all decisions and the run exited, for clearly separate offline scoring. Recorded decisions and inspected source dataflow support the decision-input provenance; a self-written declaration is not an absolute access audit.

The initial heading was explicitly reset to0 for this new generation. That reset is not credited as autonomous turning. Two later roughly70-degree turns were executed by the original RouteTracker through the selected segments.

## The requested 1m/s target

The successful generation used the user's requested **1.0m/s command ceiling**, not a permanent0.3/0.5 limit. The first0.5 run was only an integration diagnostic. The existing make_profile formula set the original physical guard to1.5m/s for this1.0 command setting; no new stair-specific limit was added.

Offline world-horizontal speed scoring rotates native body-frame velocity by its recorded quaternion before taking XY magnitude:

- Actual command forward peak:1.0m/s.
- Forward command≥.95m/s:234 samples,4.68s total sample coverage.
- Actual world-XY speed in those samples: mean**0.854114**, p95**0.999515**, peak**1.020572m/s**.
- Actual speed≥.9m/s longest continuous endpoint span:1.14s. Requiring command≥.95 simultaneously gives1.02s.

The run shows short near-1m/s execution on sensor-selected segments. It does not establish sustained1m/s cruise. Native turns, short targets and proportional approach braking naturally reduce speed; changing these control laws was not part of this prototype.

## Verification and reusable skill

- Final unit suite:1,035 tests, OK,3 skips. Regression:105 tests, OK,5 skips.
- Focused native adapter tests compare commands against the same RouteTracker, verify duplicate-source dwell, stale-pose zero, protocol validation, arrival→idle and absence of perception/floor/heartbeat/TTL prerequisites.
- All14,720 policy/control timestamps match exactly; request, controller command and actual policy command maximum difference0. All2,527 nonzero command samples use post-VIO. Controller positions equal the navigation-source positions.
- Core controller/source snapshots match their saved manifest hashes. Source preservation does not imply the entire pre-existing dirty worktree is committed.
- Video encoder exit0; ffprobe:1024×512,10fps,2,943frames,294.3s,29,973,313bytes. Final map:2,419 accepted/29 rejected frames; its final stale metadata after shutdown is not current navigation authority or room coverage.
- Original contact and COM-velocity conventions remain unvalidated. Shutdown retained SDK pending-writer warnings; normal capture exit/video encoding and process cleanup were verified separately.
- No commit or push.

Reusable skill installed at `/home/hyh001/.codex/skills/dynanav-explore/SKILL.md`, with `references/native-interface.md`. It covers sensor/map inputs, coordinate conversion, sparse goal choice, native control and actual feedback, stop/parking, provenance and measured-speed reporting. Skill validation and independent read-only forward-testing passed.

Reproduce an authorized finite run from the checkout:

```sh
ROS_DOMAIN_ID=<free-domain> SPEED_MPS=1.0 DURATION_SIM_S=300 bash scripts/run_dynanav_native_exploration.sh
```

The launcher starts with no target. The model/human then chooses goals from live sensors and sends them through `scripts/explore_waypoint_client.py`. It does not automatically choose a fixed coordinate sequence.

Evidence in the run: `model_decisions.jsonl`, `native_exploration_audit.json`, `independent_native_review.json`, `capture/interactive/receipts.jsonl`, `capture/interactive_waypoint_control.jsonl`, `capture/interactive_frames/`, `capture/operator_map/`, `operator_grid/`, `capture/policy_frames.jsonl`, source snapshot/manifest, video, original logs, capture_exit and lifecycle. Recompute offline metrics with `python3 scripts/audit_dynanav_native_exploration.py <run>`.
