# Asynchronous DSH trial: real request, limited motion, camera guard stop

## Result and scope

Run `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_async_dsh_speed05_20261111T001350Z`, domain191. One genuine model-selected waypoint reached the original controller and produced limited physical motion. The target was not reached. The run failed closed at physics95.315s on the unchanged RGB camera-mount pose assertion, before the waypoint's30wall-second TTL expired. All owned capture/native/ROS processes exited; no new model or GPU trial is running.

This is asynchronous simulation, not the earlier nonrealtime lockstep mode. Requested budget180sim seconds, command ceiling0.5m/s, physical speed guard explicitly fixed at0.65m/s, original TTL30wall, unknown/freshness/source/IMU and hold5wall gates unchanged. The user’s1m/s navigation goal is not achieved. Same original spawn and yaw0 are an explicit new-generation reset, not navigation progress; no earlier map was reused.

Native complete-scan watermark and producer envelopes were enabled, with final combined manifest `85c17501a54d75b995ac380e290ba2a356dc62e858782c0ba7da443c1d3604c3`. The51 production capture snapshot files were checked unchanged at the end. Own DSH parser/watch wrapper files changed as separately archived below; the broader355-file initial approval cannot be described as unchanged. Heavy native detail was10000–10001s, outside the entire180sim budget. Light timing/ingress, original source CDR, video/map/trajectory and bounded receiver archives remained enabled.

The capture result’s `nearfield_depth_control_enabled=false` means the raw camera recorder did not itself write control. The bridge did enable depth safety fusion; all actions still passed through the original controller.

## Actual model observations and exactly-once request

DSH task `2026-11-11T00-14-09-277Z-99ff2659`. The model used the new generation’s actual RGB, source pose/perception, local map and explicitly historical recent-usability evidence. Ground-truth world pose was not provided for navigation. A configured target speed printed in video is an upper limit, not proof of a motion command.

- First round was submitted00:14:19.6UTC. Successful native image reads occurred00:14:23.227 and00:14:36.080, while capture was live. The model chose a roughly1m forward goal, but its result was a sole JSON-fenced object. The old strict parser rejected it before any claim or request. No old intent was later replayed after its evidence expired.
- Parent authorized a narrow own-interface format fix: accept only a whole sole JSON-fenced object, with no second object, extra commands or evaluation. Original schema, finite-value checks, generation/source binding, TTL and claim rules remain.40 offline tests passed. The old result/rejection are preserved in `dsh_rounds/20261111T001409Z-7594319e`; the new script snapshot and hashes are in `intent_interface_revision_fenced_json/`.
- A fresh second round was submitted00:16:18.698. It reread the compact state and real `rgb_000041` image, then independently selected goal `[1.01005,-0.10831,0.00701]` in local estimator coordinates. The original local front joint-floor support and measured clearance supported this direction. The map was historical and was not presented as a current global frontier.
- Model completion00:16:31.642, already-armed watcher observation00:16:31.727, relay finished00:16:31.940. One controller request, seq1, was accepted. The host did not select or modify the goal. First nonzero command occurred90.115sim. This was not a fixed micro-action chain.

The second round and original result/receipt are in `dsh_rounds/20261111T001600Z-f0410abf`. No goal renewal, second request or reached event occurred. Queueing an intent never bypassed the controller’s live per-tick gates.

## Measured command and motion

Pre-action policy baseline90.095sim to last policy95.295sim spans260 control ticks.37 ticks were nonzero, or0.74sim seconds and14.23% duty. Holds included127 `perception_stale` and94 `perception_fusion_invalid`. Command maximum0.5m/s; forward command integral0.349872m is not displacement.

Physical truth net XY displacement over that policy-time interval was0.127290m. Native actual3D speed across the window had median0.040508, p95 0.138077 and maximum0.334880m/s. During nonzero-command intervals, mean0.057052, median0.046406, p95 0.128156, maximum0.309364m/s. There were zero physical speed guard breaches. Standing-policy joint actuation is not counted as navigation.

The latest post-VIO net XY value0.093068m uses delayed source stamps89.9→95.1s, not the policy-time endpoints. Controller final odometry and cleanup collector final post-VIO both ended at95.1s. At those exact source epochs, physical truth net XY is0.096712m. The earlier0.127/0.093 difference is therefore not a localization error. Even the matched-time net difference is not a calibrated accuracy measure because map axes, initialization attitude and sensor/base frames still differ. These truth comparisons are offline scoring only.

Evidence: `motion_failure_audit.json`, `same_epoch_displacement_audit.json`, original control/action/pose journals. No settled endpoint was available because capture aborted. Neither net metric means the goal was reached or a room explored.

## Camera epoch failure and known limits

Original exception: `RTX RGB renderer pose does not follow the physical camera mount`, position difference0.0305224m versus unchanged0.03m limit; forward cosine0.99999429. Physics95.315s, cached RGB ReferenceTime95.2s, acquisition physics95.31s, reported acquisition lag115ms.

Read-only5ms truth reconstruction with the fixed camera mount shows the renderer matrix exactly matches the physical camera at95.115s, position residual8.9e-16m. At the reported RGB source95.2s the position difference is22.951mm. This is not enough to justify merely comparing against a historical pose at95.2s, nor to restamp the image. Pixel/frame/renderer epoch binding needs a separately authorized diagnostic. No production threshold, asset, timestamp or validation was changed to pass the run.

`camera_pose_epoch_readonly_audit.json` retains the full failure payload and temporal comparison. `capture/camera_render_pose.jsonl` and `capture/truth_scoring_stream.jsonl` are original evidence. The failing95.2s RGB was rejected before publication; its pixels were not saved in the source CDR. Published original RGB ends at95.1s. No static-geometry pixel epoch proof for the rejected frame is claimed.

## Input archive and cleanup integrity

Source CDR archive is complete:40,974 messages, consisting of19,060 original clock,19,060 IMU,952 raw PointCloud2,952 typed embedded-scan envelopes and950 RGB frames. All stored CDR hashes, offsets, lengths, headers, fields, raw pixel/point bytes and semantic checks passed, including all952 embedded-raw joins. Original publisher CDR is equivalent reserialization, not a promise of identical DDS padding. Queue overflow0, write errors0;2,664,139,344 source bytes, peak queue8,059,612bytes/4records. A terminal hook flushed before fast shutdown, without extra physics/render/control ticks.

Native ingress independently received all19,060 IMU,952 typed clouds and950 images.909 continuous native LIO/post-VIO pairs cover4.3–95.1s, with preserved original±1ns conversion differences. Thread-safe queue failure is empty and orderly drain reports no pending work. The independent raw=True receiver has separately counted DDS gaps and is not double-ended input completeness; every received input matches the publisher semantics. Its queue also has zero overflow/errors.

The light sync/boundary file is7,516,204bytes, below its8MiB cap, with6,685 sync rows, no row gaps and final summary. Future full180sim trials could reach this cap and must report partial trace if they do.46 exact-same-ns native full-cloud/LIO pairs during90.2–95.1s are indexed in `motion_native_full_cloud_index.json`; they are not substituted raw scans or post-VIO poses and include holds as well as motion.

`full_cdr_audit.json` and the preserved native audits hold detailed counts and overhead. The video decoded successfully:950 frames,95.0seconds. Total run archive about7.7GiB. Lifecycle confirms capture exit1 and all owned cleanup at00:16:44UTC. No files were deleted, no original assets changed and no commit/push was made. Subsequent camera investigation is separate; no unapproved restart has occurred.
