# Camera binding: runtime validation and limits

## Valid default-off baseline196

Run `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_camera_binding_v2_off_retry1_20261111T013556Z`. Source release `c916aec2064063cb79dcd3a580944123f68a7723987c9e62e9b8ef924a802405`; camera manifest `918581639c72877cb8e6060eaa75b7486469ebe11a4c524bfa05ededdac76a50`.345 approved hashes matched before and after. Parent validation1110passed/8skipped and independent review precede this runtime test; CPU tests alone were not RTX acceptance.

New same-spawn yaw0 reset,12sim budget, no map reuse, no DSH/waypoint requests. Command ceiling0.1m/s, original physicalguard0.65, all original source/arrival/freshness/unknown/TTL and3cm/0.04rad mount checks unchanged. Native complete-scan watermark enabled. No lockstep, extra camera getter, heavy camera observer, accept wrapper, additional rendering/physics call or camera setting change. Native heavy detail10000–10001 outside budget.

The same future comparison startup wrapper was explicitly off. Actual settings were `waitIdle=false` and `checkForHydraRenderComplete=-100`; one original app constructor and empty extra-argument delta. No on/treatment run has been authorized or executed here.

Capture completed exit0,2400 physical steps and600 controller/600 policy body commands, every command zero. Original motion budget ended at7sim then parked to12sim. Loop25.066276wall seconds,RTF0.478731. Original runtime step assertions and final+.005/+1 receipt passed; no independent per-render counter observer was added. All owned processes, including the SDK telemetry child, exited. Initial placement settling and later stationary sway are not navigation.

## Frame identity and remaining pose limitation

Front120 and nearfield120 frames cover source0.1–12.0s. Every frame's event rational equals its RP Fabric rational, every original event renderResults matches its data-node results, before/after states are identical, and all five front/nine nearfield data-output checks pass. The revised traversal stops at the exact RP dispatcher rather than treating its scheduling pulse or the global dispatcher's pulse as persistent data completion. Exec state is auxiliary evidence, not independent freshness proof.

All120 nearfield immutable NPZ hashes passed. Original callback arrivals are preserved rather than refreshed at reading/drain time. No quarantined or failed frame occurred. Callback source lag median15ms, p95/max about110ms; processing/drain source lag median15ms, p95/max115ms. Nearfield queue residence median48.93wall ms, max100.58ms. These are specifically callback/drain measures; exact pixel-copy completion latency was not sampled.

Maximum current-mount residual29.104mm and forward angle0.037873rad occurred at source0.2, processing0.315, during initial falling/settling. At source>=6sim, maximum position residual2.761mm and angle0.006110rad. All original checks passed, but this zero-command run does not certify moving-camera physical endpoints or pixel-tick pose. A future moving test could still hit the unchanged guard. Stable>=2sim actual3D speed peaked0.03504m/s; the all-run1.3345m/s maximum at0.155sim was almost entirely vertical placement settling with zero command, not cruising or a requested robot movement.

## Source availability and archives

Native ingress contains all2400 IMU,120 RGB and119 typed scans.78 continuous LIO/post-VIO pairs cover4.3–12.0, retaining the original two±1ns conversion deltas. Source CDR is fully flushed:5158 messages, including2400clock,2400IMU,120RGB,119raw and119typed. All stored CDR SHA/offset/length and complete semantic/raw-byte checks pass; all119 typed embedded clouds equal raw clouds. The independent receiver retains2400clock/IMU each but has separate DDS gaps in RGB/raw/typed (103/89/92 received respectively). Received bytes/semantics pass; receiver gaps are not native input loss or source-archive incompleteness.

62 independent post-publication perception observations include61 postcompute accepts and59 still-fresh samples. Three-source sim-age median0.210s; depth wall-age median0.24944s and maximum0.41110s. Normal idle control gates passed19/50 ticks in6<=sim<7, not all ticks. This is an availability measurement, not motion or proof of realtime hardware readiness. No source gate was relaxed.

Evidence: `runtime_binding_audit.json`, `full_cdr_audit.json`, `zero_delay_startup.jsonl`, original camera/nearfield journals and immutable files, native traces, control/policy logs, source/receiver CDR, lifecycle and telemetry cleanup records.

## Excluded versions and corrected crash interpretation

-193 (`rtx_camera_binding_baseline_20261111T012445Z`) crashed natively near the first RTX render. Persisted `stage=asset_loading` was stale. Independent receiver and native diagnostics prove19clock/IMU through0.109999999s, despite unflushed source index0 and no valid RGB/cloud. The initial claim that physics/all inputs were zero is retracted. stderr and surviving Python stack are retained; SDK uploaded/deleted its ZIP before it could be copied, so no local ZIP is claimed.
-194 (`rtx_camera_binding_baseline_retry1_20261111T012648Z`) reached actual0.1s camera data, then failed closed because the first binding implementation included global/RP scheduling exec pulses in completion checks. Both complete failed bundles are quarantined as invalid. The subsequent reviewed v2 fixes only traversal scope;194 is not an accepted baseline.
-195 (`rtx_camera_binding_v2_off_20261111T013409Z`) has the same corrected early-render description and19IMU evidence as193, not physics0. Its actual539847-byte dump ZIP is locally saved, SHA `074276860709f5e3d9ea6f4c9ef651c7929ba03eed51cffd0dda274cf052bb46`, with TOML, Python stack and stderr. The crash reporter later hung with heap-corruption text; owned cleanup followed. Its single same-configuration clean retry is the valid196 above. No cache/settings change was used to obtain success.

Previous anonymous probes and the heavy read-only Office observation established the original one-frame pixel/metadata mixing, but their added getters and archive overhead are not a performance comparator for196. All failed and diagnostic generations remain separate and preserved. No production source, original asset, cache or guard was changed by the runtime owner; no commit/push or new navigation occurred.
