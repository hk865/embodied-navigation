# waitIdle TRUE / async-ON diagnostic: startup crash

Run: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_waitidle_async_on_20261111T025010Z` (domain 203). This was one authorized 20-second simulation probe, with no retry and no DSH or waypoint requests.

## Controlled configuration

The valid reference is domain 202, with waitIdle TRUE and synchronous auxiliary fusion. Domain 203 used the same capture entry, profile, renderer settings and frozen source. The sole functionality change was the host bridge `--enable-async-depth-fusion`. The capture subprocess retained `ASYNC_DEPTH_FUSION=0`; the host bridge had the ON flag. Actual routing is archived in `routing_evidence.json`. Startup readback was waitIdle TRUE, completion order -100, and derived post-SDG consumer order -98. No zero-delay bundle, lockstep, new getter, physics/render call or safety-threshold change was made. All 363 approved source hashes still match.

## Observed outcome

The first RTX render crashed at 02:50:29 UTC. Independent received data and native ingress show 19 clock and 19 IMU messages through 109999999 ns. Five standing-policy samples have zero actual navigation commands. There were no control requests, camera binding records, RGB/depth frames, native pose pairs, or completed auxiliary-fusion jobs. The async worker existed, with active/completed/pending all zero in the last recorded snapshot. It never entered the common 6–15 second comparison window.

The archived minidump independently identifies SIGSEGV in `librtx.scenedb.plugin.so`, relative RIP 0x1f170e and frame returns 0x1f1a12 / 0x1f4dbd, matching the prior startup signature. The later heap-corruption text is secondary. This does not identify the trigger or implicate a completed async geometry job.

## Retained evidence and limits

The actual ZIP is `startup_crash_archive/03a72fea-9299-4c8c-f153e484-167a3b77.dmp.zip`: 544962 bytes, SHA256 `5ae9eade129ecf932360e389379c437a1b49402e0f9815fe52941152b564526a`, CRC verified. Actual TOML, Python stack and complete stderr were retained.

The producer CDR archive did not flush and is incomplete. Its empty index is not evidence of zero physics or zero IMU publication. The independent receiver archived 38 messages (6384 bytes), with all recorded CDR hashes verified and no recorder drops/errors. This is not an end-to-end complete input record.

Only confirmed owned capture-group and detached telemetry processes were terminated after crash evidence preservation. The harness cleaned up remaining owned services. Exit 143 records that cleanup, not the original native fault. All owned processes exited. Remaining disk space was 192159596544 bytes. See `startup_failure_evidence.json`, `final_lifecycle_audit.json`, and `owner_crash_cleanup.json`.

## Conclusion

No valid async-ON performance sample was obtained; common-window gate throughput, compute/source ages and RTF comparison are unavailable. Domain 202 TRUE success followed by domain 203 TRUE failure shows waitIdle is not a reliable demonstrated fix. No retry, motion, additional model submission, cache modification, driver change or new setting was attempted.
