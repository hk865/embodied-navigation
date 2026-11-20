# One process-local waitIdle diagnostic, 2026-11-11

## Outcome and limits

Run `rtx_waitidle_single_zero_20261111T024420Z`, domain202, completed the authorized **single**20-simulation-second zero-request diagnostic. Capture exit0; all owned/SDK/native/telemetry processes exited. No automatic retry, second arm, async-fusion-on run, DSH post or motion request was performed.

This one waitIdle=true trial crossed the first RTX render and completed, whereas newly archived false trials199/201 crashed there. **That is not proof of the crash cause, a stable fix, or a valid fusion-performance A/B.** Earlier false trials196/198 had also completed. No driver, SDK installation, cache, asset, security, IOMMU, ROS thread or global setting changed.

## Controlled startup configuration

Original20sim/motion-end15/.1command-ceiling/.65physical-guard, heading0 explicit new-generation reset, native typed-watermark=true, originalcamera v2 and downward safety depth, all source/arrival/clock/unknown/age/TTL/mount guards retained. Async fusion stayed OFF, heavy diagnostic window10000..10001 inactive, no lockstep or getter observer.

Independent process-local wrapper `.dsh-subagents/wait_idle_single_20261111/` defaults off and accepts only the exact authorized mode. It delegates to the unchanged strict20sim base entry. Thirteen CPU mocks passed, including original constructor once, unchanged arguments except the one added flag, original return/exception and factory cleanup, conflicting/duplicate keys rejected, no added evaluate/render/app.update/physics/get_data calls.

`single_variable_audit.json` confirms:
- The original8extra_args exactly match A199. The sole append is `--/app/hydraEngine/waitIdle=true`.
- Actual7settings readback differs only at waitIdle false→true. Completion order remains-100; Fabric/multi-tick/200Hz/persistence settings are unchanged.
- Both logs show the same305extension multiset. One original constructor call each.
- Post-SDG consumer order remains-98 **as derived from frozen source and actual completion-order readback**, not an independently instrumented callback-order measurement.
- Final release SHA455af5d88d73c3beeefe40e96a58c032da04a11604f975fc0048ac71bee88059 and all362approved source/build/helper hashes matched before launch and after run. Old entries remain unchanged.

## Real sensors, original guards and zero commands

-4000physics steps,1000controller records and1000policy records; every command is zero. No request.json or DSH task exists for this generation.
- Last original step assertion records20.010→20.015s and+1step. Original200Hz source pipeline remains unchanged; this was not an independent per-render counter probe.
-199front and199nearfield frames, sources0.1–19.9sim, all event/RP rational/renderResults/beforeafter bindings and required5/9producer gates pass. Original nearfield arrival fields preserved; every immutable NPZ SHA passes. No quarantine or mount stop.
- Front callback lag median15ms and max110ms simulation time; processing max115ms. waitIdle did not eliminate those measured source lags.
- Whole-run mount maxima29.104mm/.037873rad occur during startup settling, below unchanged30mm/.04rad guard. Common6–15sim maxima4.565mm/.013267rad. Stable after6sim native3Dspeed peak.05571m/s is physical stand motion, not navigation. Spawn-to-final XY drift.02562m is also not navigation.
- Loop36.94014wall seconds, budget/loopRTF.54142. The parent's external120wall-without-new-physics/frame stall threshold was never approached; actual progress reached7.3sim and finished well within120wall. No added heartbeat or internal clock change. Existing540wall hardcap was unchanged.

## Source ages and safety availability

Common6<=sim<15 has450original controller ticks:134idle gate passes(29.78%),103perception_stale and213perception_fusion_invalid. All remain zero commands. Gate-pass status is not permission for an unissued movement.

119perception publications:117accepted after compute,111still fresh against the independently received post-publication clock;2true post-compute cloud-sim-age expiries remained unknown. This is synchronous fusion, not a new async worker result.

All three source sim ages at post-publication: median.210000s, p95.255000s, max.310000s. The over-limit records are retained, not re-stamped. Wall age median/max: depth.256034/.378383s, RTX.116485/.194624s, LIOpose.118314/.195104s. The normal controller still spends most ticks held; no realtime or high-speed readiness claim.

## Input/archive boundaries

Publisher:8595records,555676628bytes, explicit terminal flush complete, no writer drop/error. Counts4000clock+4000IMU+199RGB+198rawRTX+198typed. Full original header/fields/bytes/certificate semantics and individualCDR SHA verified. PublisherCDR is equivalent reserialization, not literal wire-padding identity.

Receiver:8884retained records, allSHA/semantics pass. Allclock/IMU matched, but independently subscribed DDS missed32RGB/64raw/61typed compared with actual published messages. A completed archive is not a complete DDS stream. The absent nominal200thRGB/199thscan was **not published** and is not counted as a DDS loss.

Native diagnostics separately contain all4000IMU/199RGB/198lidar inputs.157continuous LIO/post-VIO pairs4.3–19.9sim, original100ms±1ns conversion retained. One initial `[Data Cut] Throw one image frame!` warning remains in fullstderr; input reception does not imply everyframe caused an update. Native diagnostics8875written/0dropped,6553864bytes, finalwriterstatsnoIOerror. No later native source gap/fault found.

Peak sampledVRAM:wholecard5272MiB includesdesktop; owned experiment sum4043MiB.1Hzsampling does not guarantee instantaneous peak. Remaining disk192180961280bytes after archive, about178.98GiB.

## Artifacts

Run root: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_waitidle_single_zero_20261111T024420Z`

- `wait_idle_startup.jsonl`, `single_variable_audit.json`, `mode.json`, `cpu_tests.log`
- `runtime_binding_audit.json`, `full_cdr_audit.json`, `final_lifecycle_audit.json`
- Original `capture/` RGB/video/map/truth/controller/policy/nearfield journals and arrays
- `source_input_cdr/`, `received_cdr/`, `observer/diagnostics/`, fullstderr and GPU samples

No subsequent run is authorized by this result. Further choices remain with the parent after this evidence review.
