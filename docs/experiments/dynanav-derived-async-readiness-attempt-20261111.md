# Derived Office async exploration attempt, domain 207

## Outcome

Run: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_candidate_async_dsh_20261111T034100Z`.

The SDK, derived scene, typed RTX input, camera binding and asynchronous safety fusion ran. The owner readiness adapter incorrectly expected a perception stamp in successful controller journal rows. It therefore never dispatched the model. At 30 simulated seconds it requested the authorized zero-action stop; the original controller parked and capture completed at physics time 35.015 s. The run was not restarted or resumed.

There were **0 model tasks/posts, 0 waypoint requests, and 0 nonzero navigation commands**. This is not exploration or finite-loop runtime validation. The interface error is separate from genuine source-age rejections, which also occurred.

## Configuration and preservation

- Fixed 180 s budget / 175 s motion end; actual early stop, not extended budget. Explicit heading-zero reset at the original spawn. No cross-run map merge.
- Desired speed ceiling 0.5 m/s, physical guard 0.65 m/s, original 30 wall-second request TTL and all existing age, unknown, pose, mount, contact and clock protections unchanged.
- Process-only waitIdle=true; completion order -100 / post-SDG -98. No zero-delay bundle, lockstep or extra getter observer. Heavy diagnostics outside the budget at 10000–10001 s.
- Capture async environment 0; host bridge async fusion ON. Frozen v4 combined release SHA `455af5d88d73c3beeefe40e96a58c032da04a11604f975fc0048ac71bee88059`.
- Derived scene SHA `26954f819bc4b3441254a3f1e966e5e1abc55092972537f57ab04dff51c1a38f`, selected through the original scene argument. Runtime expanded instances 0 is the expected baked state. The original scene remains SHA `c4a1458847c51c3b3afd65dc6e6d8a50929ad350a135601a67100e35e1f59950`.
- All 403 approved source/dependency hashes and all 905 original asset hashes were rechecked unchanged after the run. The derived scene retains the previously documented 118 metadata exceptions and flattened reference structure; no full authoring equivalence is claimed.

## Actual safeguards and runtime

- 7000 physics steps, 1750 controller samples and 1750 policy samples; all navigation commands zero.
- 35.0 s stepped simulation / 61.247077 wall seconds = RTF 0.571456. Initial absolute clock offset accounts for the final physics timestamp 35.015 s.
- 349 front frames and 349 nearfield frames passed their original 5/9 producer-node binding checks, with exact event/RP reference and render-result agreement. All nearfield NPZ hashes and original arrival values verified.
- Maximum front mount error 29.104 mm and 0.037873 rad remained inside the unchanged 30 mm / 0.04 rad guards. No camera quarantine. This zero-navigation-command result is not a moving-camera certification.
- Native input: 7000 IMU, 349 RGB, 348 typed scans. 307 LIO and post-VIO outputs have identical source vectors, continuous 4.3–34.9 s at 100 ms ±1 ns. The native pipeline committed all 29623 accepted packets and drained normally with no failure or pending packets.
- Diagnostic writer: 16374/16374 records, zero drops, 12,231,744 bytes; heavy detail inactive.

## Readiness adapter defect

The frozen `phasegate.py` read the final controller row and required:

`round((last.get('perception') or {}).get('stamp_s', -3) * 1e9) == ns`

Successful `tracking` rows omit the `perception` object; hold rows include it. All 63 successful rows between 15 and 30 s lacked that field, so the left side was always -3000000000. Native continuity checking was downstream of this false condition, so the phase's `native_continuous=false` was an unexecuted check, not a native input gap.

The prior CPU tests covered decision booleans and wrapper behavior, but did not exercise extraction from the actual successful journal schema. Original code and evidence are preserved; a separate CPU worker is preparing a future correction. Removing the field check alone is not a valid fix: nearby successful control rows, post-publication samples and periodic observation snapshots do not establish a simultaneous authoritative readiness proof.

See `phasegate_interface_failure_audit.json` and `phasegate_schema_fixtures/` for exact success/hold and source-linked examples.

## Actual asynchronous safety observations

The independent watcher saved 220 file versions. These contain 110 distinct non-failclosed, front-floor-supported source samples; 107 were also independently fresh when observed. All 220 retain an accepted-post-compute flag, because an invalidated version can preserve original diagnostic flags. That flag alone is not usable safety evidence.

- Whole run: 112/1750 original idle gate passes, 984 stale holds, 294 fusion-invalid holds, 60 clearance-unknown holds and 300 not-ready holds.
- Common 6 ≤ sim < 15: 31/450 passes; 28 distinct supported sources, 27 independently fresh. Observed depth wall age median 0.363 s / maximum 0.520 s; RTX 0.264 / 0.404 s; LIO pose 0.265 / 0.405 s. Shared simulation age median 0.215 s / maximum 0.340 s.
- Readiness 15 ≤ sim < 30: 63/750 passes and 57 supported independently fresh source samples. These are separate observations, not proof of a simultaneous stage0 decision.
- One-per-supported-source compute time over the full run: median 110.149 ms, p95 136.893 ms, maximum 217.860 ms.

The last bridge snapshot reports computed=124, compute_errors=118, expired_jobs=118, published=110, pre_replace_rejected=14, pending_replaced=6, cached_invalidated=110. `computed` counts error-free results; expired jobs are a subset of compute errors. `completed` is a 0/1 result-slot occupancy. Published samples can subsequently be invalidated to unknown. These are overlapping stages, not additive success/failure denominators. This is the last live snapshot, not final worker-join accounting. Two-second snapshots do not provide a complete callback-gap trace.

This run is not a controlled off/on performance pair with 206: its profile and stage0 observation workload differ. It demonstrates actual async execution and conservative expiry, not a performance improvement or reliable SDK-startup fix.

## Source archives and lifecycle

Publisher CDR: 15045 records / 975,642,376 bytes, complete flush, zero archive drops/errors, all SHA and semantic checks passed. Clock and IMU each 7000, RGB349, raw348, typed348. The final pending rendered scan was not published; the 349/348 difference is not an archive or DDS loss.

Independent receiver CDR: 15722 records / 1,182,028,371 bytes, complete local flush, zero local drops/errors, all SHA and semantic checks passed. It missed 52 published RGB, 67 raw and 80 typed messages at DDS reception. All clock/IMU messages matched. These receiver gaps are not native input losses.

The foreground harness returned 0. All recorded supervisor, capture, observer/native, bridge, collector, receiver, GPU sampler, heartbeat, phase and sidecar PIDs exited. A final scan found no SDK or native processes. Available disk after audit: 187,102,236,672 bytes.

Primary machine-readable proof: `final_run_audit.json`, `runtime_binding_audit.json`, `full_cdr_audit.json`, `original_asset_final_verification.json`. Original video, maps, trajectory, source CDR, bridge snapshots and phase records remain in the run directory. No new run or model dispatch is authorized by this report.
