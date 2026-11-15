# Async DSH movement after camera binding v2, 2026-11-11

## Result

Run `rtx_async_dsh_camera_v2_retry1_20261111T014449Z` (domain 198) completed the original 180 simulation-second budget, 36,000 physics steps, and exited normally. This was a **limited genuine model-selected movement**, not complete basement exploration. The sole accepted waypoint expired at its original 30-wall-second TTL and was not reached. No target capture, room transition, return loop, 0.5 m/s cruise, or requested 1 m/s cruise is claimed.

One-second settled endpoint windows, [20.915,21.915) and [179,180) simulation seconds, give **0.44363 m native-body net XY** and **0.44163 m post-VIO IMU net XY**. These remove initial spawn settling and use the same time windows. At exact common VIO source epochs 21.8 and 41.3, the corresponding numbers are 0.44523/0.44399 m; native positions are linearly interpolated between <=5 ms samples for offline scoring only. The earlier 0.45771 m partial number compared the last nonzero command sample, before post-motion settling, and is not the final settled displacement. Body and IMU origins are different; this comparison is not a full localization accuracy claim.

## Frozen setup and safety

- Repaired original Office scene, original spawn, explicit yaw-0 **new-generation reset**; no old observations or maps reused. The local map level +0.02 is an estimator label, not proof of full basement/floor coverage.
- Asynchronous simulation, no nonrealtime lockstep, no official zero-delay treatment, no heavy getter observer. The default-off startup wrapper read actual waitIdle=false and completion order=-100; one original constructor and no added SDK settings.
- Command ceiling 0.5 m/s; original physical speed guard explicitly fixed at 0.65 m/s. Original TTL 30 wall seconds, loss timeout 5 wall seconds, source/wall/simulation age, unknown-space, 3 cm / 0.04 rad camera-mount and original controller checks unchanged.
- RTX SLAM, native complete-scan watermark enabled, front RGB, simulated downward safety depth enabled in joint safety perception. Downward depth does not feed SLAM. `nearfield_depth_control_enabled=false` in raw capture means that sensor does not itself write commands, not that the bridge omitted safety fusion.
- Heavy native detail window 10000..10001, outside this budget. Complete publisher source-CDR and independent receiver archive, light native trace, video, map, poses and GPU sampling retained.
- Camera release SHA c916aec2064063cb79dcd3a580944123f68a7723987c9e62e9b8ef924a802405, binding manifest 918581639c72877cb8e6060eaa75b7486469ebe11a4c524bfa05ededdac76a50. All approved source hashes remained unchanged. No production edits in this run.

## Genuine model and control chain

DSH task `2026-11-11T01-45-00-390Z-16135e29` used the authorized read-only helper and successful native `read_image` calls. Full original model outputs and public tool proof, not internal reasoning, are retained in `dsh_rounds/` and `dsh_round_audit.json`.

1. Round 1 read state at 01:45:21.779 UTC and actual RGB at 01:45:23.991 UTC. It independently chose `[1.0054,-0.0431,0.0041]` in measured post-VIO coordinates, about 1 m forward through currently observed floor. Prearmed completion watcher relayed once in about 97 ms, producing **seq 1**. No host-selected coordinate or automatic action chain was used. First nonzero command 21.915 sim, last 40.335 sim; original receipt `waypoint_expired`, not reached.
2. Round 2 read new state at 01:49:45.430 UTC (170.3 source) and new image at 01:49:51.288 UTC (170.4 source). It independently selected a new 0.7 m forward point `[1.1430,-0.1843,0.0156]`, noting fresh local floor but a stale, non-actionable global frontier snapshot. It completed at 01:50:07.975 UTC. The watcher called relay within about 75 ms; original controller was already terminal at motion budget 175 sim, so relay refused, without claim, seq 2, or retry. Offline read-proof validation later confirmed actual live reads; that does not make a later request permissible.

Round 2 was submitted too late because parent/worker orchestration between rounds left a long idle interval. This is a coordination failure, not model inactivity or a reason to extend TTL or run budget. No third turn was forced into the budget. A separate CPU-only finite-loop preparation is owned by the parent; this run did not execute it.

## Actual motion and holds

- 144 of 9,000 policy ticks had nonzero commands, totaling 2.88 sim seconds. The precise first-nonzero to observed TTL-clear window is **21.915 <= sim < 40.515**, 930 control/policy ticks with exact command agreement and **15.48% nonzero duty**. No active_seq filtering is used, because hold/duplicate records may omit that field.
- Full TTL-clear window: 423 `perception_stale`, 340 `perception_fusion_invalid`, 121 fresh-measurement and 46 fresh-duplicate ticks. Of those 46 duplicates, 23 carried cached nonzero commands and 23 were zero. Status labels alone are not counted as motion.
- The earlier `motion_audit.json` first-to-last-nonzero window is 21.915 through 40.335 inclusive: 922 ticks, 144 nonzero (15.62%), 418 stale and 337 fusion-invalid. This is a deliberately shorter interval, not the whole TTL-clear window. `motion_seq1_ttl_window_audit.json` records the exact full-window reconciliation; neither interval is conflated with command-active wall time.
- Forward command mean 0.36845, max 0.49400 m/s. Integral 1.06114 m is a **command integral**, not traveled distance.
- Native actual planar speed over the whole action span: mean 0.04192, median 0.02341, p95 0.14377, peak 0.36907 m/s. Native 3D peak 0.39903 m/s, below the unchanged 0.65 guard. Initial spawn settling is excluded.
- During nonzero-command intervals only, native planar mean 0.05619, p95 0.21586, peak 0.35101 m/s. These intervals omit coast/settling after commands, so their peak can be below the whole action-span peak.
- 1,403 safety-perception publications: 773 accepted after compute, only 664 independently still fresh at post-publication read. Rejections: 629 cloud simulation-age expiry and one cloud wall-age expiry after compute. The freshness bottleneck remains; this is not a real-time fix claim.

## Camera binding and original mount guard

1,799 front and 1,799 downward frames passed per-RP event rational/reference, exact original render-results bindings, before/after stability, and required producer gates. All downward NPZ SHA checks pass; original arrival fields remain unchanged. No quarantine or mount-guard stop occurred.

Front callback lag median 15 ms, p95/max about 110 ms simulation time; processing lag p95/max about 115 ms. These are measured original source lags, not re-stamped current data. Across the motion window, maximum mount position error was 9.849 mm and forward-angle error 0.01209 rad, below the unchanged 30 mm / 0.04 rad guard. Whole-run maxima 29.104 mm / 0.037873 rad occurred during startup settling; do not call them movement maxima.

This is successful finite moving-camera runtime evidence for the binding repair under the observed motion. It does not prove arbitrary faster motion, perfect render latency, or true hardware realtime.

## Source completeness, native continuity, and recording cost

- Publisher archive: 77,395 records, 5,024,884,436 bytes; 36,000 clock, 36,000 IMU, 1,799 RGB, 1,798 raw RTX, 1,798 typed complete scans. Full SHA and semantic decoding pass, zero archive overflow/write errors, explicit terminal flush complete.
- Independent receiver archive: 81,317 records, 6,213,641,032 bytes, all retained SHA/semantics pass. It received all clock/IMU but missed 256 RGB, 383 raw RTX and 400 typed DDS messages relative to publisher. **Archive complete is not DDS complete.** Cross-end matched messages are semantically equal; publisher CDR is equivalent reserialization, not a claim of identical wire padding.
- Native input trace separately records all 36,000 IMU, 1,799 RGB and 1,798 lidar inputs. 1,757 continuous LIO/post-VIO pairs from 4.3 to 179.9 sim, original 100 ms increments with the preserved +/-1 ns floating conversion pattern. No later output gaps or complete-scan fault appears in the retained diagnostics/stderr. One original initialization `[Data Cut] Throw one image frame!` warning at about source 0.1 is retained and not hidden; input arrival is not equivalent to every frame contributing an update.
- Native diagnostics wrote 88,875 records / 67,123,464 bytes, zero queue drop or I/O failure. Heavy detail stayed inactive. Source archive overhead distributions remain in `full_cdr_audit.json`.
- Loop wall time 297.32342 s, budget/loop RTF 0.60540. This includes the chosen recording and live safety processing; no single-variable performance causal claim against old lockstep runs.
- Sampled whole-card VRAM peak 5,278 MiB, including desktop. Owned experiment-process sum peak 4,045 MiB. One-second sampling does not prove instantaneous peak.
- The full run occupies about 15 GiB; disk remaining about 181 GiB after this run.

## Artifacts and lifecycle

Run root: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_async_dsh_camera_v2_retry1_20261111T014449Z`.

- `motion_audit.json`, `motion_seq1_ttl_window_audit.json`, `full_cdr_audit.json`, `dsh_round_audit.json`, `final_lifecycle_audit.json`
- `capture/interactive/receipts.jsonl`, `capture/interactive_waypoint_control.jsonl`, `capture/policy_frames.jsonl`, `capture/truth_scoring_stream.jsonl`
- `capture/camera_render_pose.jsonl`, `capture/nearfield_depth/`, `capture/interactive_frames/`
- `capture/multiwaypoint05_overview_and_rgb.mp4` (1,799 frames, 10 fps, encoder exit 0), `capture/operator_map/`, `operator_grid/`
- Final partial operator map: 1,403 accepted / 2 rejected frames; 54,651 exported free and 4,746 occupied grid cells. Unknown cells are not exported as free. Snapshot ended stale/non-actionable and is not explored-room coverage.
- `source_input_cdr/`, `received_cdr/`, `observer/diagnostics/`, full original stdout/stderr and `gpu_usage.jsonl`.

Capture exit 0; model task completed; all recorded owned PIDs and SDK/native/telemetry processes are absent. No new generation was started during audit.

The preceding domain 197 first attempt is separately retained as a first-RTX native startup crash: 19 clock/IMU messages, no RGB/cloud, no DSH submission or movement. Domain 198 was the sole authorized same-configuration clean retry. Its success does not erase the crash or claim a crash fix.
