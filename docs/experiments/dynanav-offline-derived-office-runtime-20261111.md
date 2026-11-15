# Offline-derived Office: one zero-action runtime test

Run: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_offline_candidate_zero_20261111T032644Z` (domain 206). Reference: `rtx_waitidle_single_zero_20261111T024420Z` (202). Completed one authorized 20-s simulation; no retry, DSH task, waypoint, or navigation movement.

## Scope and provenance

The intentionally changed scene input was `office_deinstanced_flattened.usdc`, 6,334,972 bytes, SHA256 `26954f819bc4b3441254a3f1e966e5e1abc55092972537f57ab04dff51c1a38f`. It was selected through the original `--scene` option. Original USD `c4a14588…` and all 905 original asset-bundle files passed final hash verification. All 373 approved runtime source hashes also remained unchanged. No production file, original asset, driver, cache or security setting was changed.

The author manifest SHA is `ab632cf105118ef9a95be6e613ba455190bc6c941d68133b3b2201f7c3b7abf0`; independent review manifest SHA is `0b3f86fb539bee7af45cd204daf359328be566b4ce0b258c9b59226d4c2f110b`. CPU selected-state comparison covered 21,189 prims, 3,785 meshes and 653,541 points. It does not establish full authoring/metadata equivalence: 118 declared differences remain (115 redundant LightAPI authored declarations, 2 no_delete, 1 hide_in_stage_window), and composition was flattened.

Runtime preparation correctly expanded **0 instances**, since the derived file was already deinstanced. It did not claim the original 3,413-instance session operation. The actual source manifest includes candidate/origin SHA and provenance; RGB/capture/map directories retain `scene_candidate_provenance.json` labels.

## Unchanged execution and safety

20-s budget, motion end 15 s, .1 m/s command ceiling, physical guard .65 m/s, heading0 new reset, empty requests. Native typed complete-scan watermark, original camera-v2 and downward safety depth remained enabled. Auxiliary fusion was synchronous/OFF for the experimental async flag. Original 30-wall-second TTL, source-age/clock/unknown/pose gates, 3-cm/.04-rad mount guard and all source arrivals/stamps were unchanged. Heavy diagnostics stayed outside the budget at 10000–10001 s.

Actual startup arguments and seven readback settings exactly matched reference202: waitIdle=true, completion order=-100, derived post-SDG order=-98. No zero-delay bundle, lockstep, extra SDK getter/evaluate/render/physics call or new pre-App USD import was added. CPU preparation included 17 wrapper tests, six negative manifest cases and a real candidate/base-entry/dependency preflight with zero SDK constructors.

## Runtime and binding

- Capture exit0, completed 4,000 physics steps and 1,000 controller plus 1,000 policy samples. Every navigation command was zero. The 15-s motion-budget terminal was expected; capture finished its original 20-s budget.
- 199 front RGB and 199 nearfield frames, source0.1–19.9s. Every original RP/time/render-results binding passed (5 front / 9 nearfield completion gates); nearfield original arrivals and all NPZ hashes passed. No camera quarantine.
- Maximum original front mount errors: 0.02910415 m and 0.03787268 rad, below unchanged limits. Callback source lag median .015 s, max .110 s; processing lag max .115 s remains.
- Simulation loop 36.605722 wall seconds, RTF 0.546363. This is a single non-real-time simulation sample, not a demonstrated startup-reliability or real-hardware fix.

## Actual RGB and local RTX geometry

The retained source-CDR RGB images at 9.5 s show corresponding red exit door, black glass doors, white wall, tiled ceiling and grey floor. At this one source epoch the reported renderer position and forward vector were exactly equal between runs. Pixel bytes were not equal: mean absolute channel difference 0.263128/255. This is not a whole-scene visual-equivalence claim.

The separate RTX scan used S=9.5/E=9.6 s, not an image/LIO fusion pairing at9.5. Offline-only canonical stair-mesh scoring used native coordinate-pose coverage **9.515–9.6 s** and excluded **4,806 points per run** outside that interval; no pose/time extrapolation or source edits.

- Original202: 31,428 raw points; 9,001 retained candidate rays; 6,626 front-face stair intersections; 6,444 within5cm; **0** more than20cm behind the expected stair; 182 materially nearer returns.
- Derived206: 31,438 raw points; 9,005 retained candidate rays; 6,630 stair intersections; 6,454 within5cm; **0** more than20cm behind; 176 materially nearer returns.

Median absolute stair-range residual was about1.17mm in both. The nearer returns are possible foreground occlusions, not evidence of missing stair geometry. This checks only these retained local rays, not missing/no-return rays or all Office surfaces. Ground-truth/native endpoint poses were used solely for offline scoring, never navigation or published clouds.

Exact raw/header/certificate/geometry-file and image SHA plus both binding payloads are in `scene_comparison/comparison.json`; original, unedited decoded images are `original202_rgb_9p5.png` and `derived206_rgb_9p5.png`.

## Native inputs, freshness and recording limits

Native ingress recorded 4,000 IMU, 199 images and 198 typed scans. There were 157 continuous LIO and post-VIO pairs, source4.3–19.9s, preserving native100ms±1ns values. Native pipeline drained normally, all20,444 accepted packets delivered/committed, no pending/uncommitted/canceled/capacity rejection and no recorded failure. The one initialization image-cut message remains in stderr; no post-initialization pair gap was found. Diagnostic writer wrote8,875/8,875 records,0 drops,6,553,864bytes.

Synchronous perception produced127 observed versions:125 accepted after computation,122 independently still fresh after publication. Two rejected versions were genuinely cloud-sim-expired. Whole-run three-source post-publication sim-age median .210s, max .310s. Depth wall age median .2535s/max .3962s; RTX .1148/.2398s; LIO .1166/.2410s.

In the same **6≤sim<15** interval, idle original gate passes were136/450 (30.22%), versus202's134/450 (29.78%). This small single-run difference does not establish a performance improvement; stale/unknown holds remain.

Publisher CDR:8,595 messages,555,678,292bytes,complete flush,0 recorder drops/errors. Every recorded CDR SHA and semantic value passed, including typed/raw embedded-cloud joins. Independent receiver:8,926 records,645,701,654bytes,complete recorder flush and0 local drops/errors, but DDS omitted25 RGB,42 raw clouds and50 typed messages relative to publication. All4,000 clock and IMU were received. Recorder completeness is not transport completeness. The198 typed scans are actual published inputs, not a claimed missing199th pending scan.

## Final state and conclusion

All owned simulator/native/bridge/observer/telemetry processes exited. Disk free: 190071697408bytes. Evidence: `runtime_binding_audit.json`, `full_cdr_audit.json`, `final_lifecycle_audit.json`, `original_asset_final_verification.json`, original logs/video/trajectory/map/CDR.

This candidate passed this one static zero-action runtime and the limited visual/stair-ray correspondence checks. It has not demonstrated reliable startup across trials, async-ON fusion, moving-camera behavior, full-scene equivalence or basement exploration. No subsequent run was started.
