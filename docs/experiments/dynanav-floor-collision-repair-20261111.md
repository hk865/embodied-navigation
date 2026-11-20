# Static Office floor collision repair, 2026-11-11

## Repair

The previous `native_exploration_20261111T095757Z` fell after crossing the collision coverage edge at world Y=-9. The rendered neighboring `/Root/SM_Floor_6m_17/SM_Floor_6m/SM_Floor_6m` existed at Z≈-3.3 but had no collision. The seven-floor session whitelist did not include it. That was a scene-physics gap, not a navigational cliff visible to RGB/RTX.

`configs/simulation/dynanav_basement_collision.json` now lists **119 audited active, visible horizontal original SM_Floor_3m/SM_Floor_6m meshes**: 34 basement, 84 level-zero, 1 upper-level. The 15 carpet decoration meshes are excluded. All seven legacy floor paths and all four original lower-stair paths remain. Existing first-flight support samples and tolerances are unchanged.

`session_collision.py` enables exact mesh collision only in the anonymous USD session, validates the explicitly selected original geometry, preserves rollback/edit-target restoration, and keeps the original GroundPlane disabled. It creates no platform, fills no hole, and changes no source geometry or saved USD layer. Floor-list validation is no longer hard-coded to seven; the four-stair contract remains.

No waypoint admission gate, target selection rule, command limit or controller was added or changed by this repair. Oracle scene coordinates below are offline physical-validation evidence, not exploration-agent waypoint inputs.

## Static verification

CPU USD inspection and an in-memory session repair verified all119 colliders, exact approximation=none, original stairs unchanged, GroundPlane=false, source layer text unchanged, and root dirty=false before/after.

The frozen source USDC file SHA256 remained:
`26954f819bc4b3441254a3f1e966e5e1abc55092972537f57ab04dff51c1a38f`.

The missing6m_17 and adjacent original floors are included. Independent source geometry checks and the runtime exported collider bounds agree on the repaired area; no broad name-only runtime selector is used.

## Short actual crossing

Run: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/floor_collision_crossing_20261111T101711Z`.

- ROS domain223, maximum60sim-second budget, original native RouteTracker/RoboGauge interface, command ceiling1m/s.
- Controlled regression target reused the prior failed run's estimated-map XY `[3.8,-4.0]`, with target Z from the current measured post-VIO pose. This was a scene-physics test, not an autonomous exploration decision. No truth pose entered the route follower.
- Actual receipts: seq1 accepted and reached; seq2 stop accepted. Normal terminal parking, capture exit0, outcome completed, error=null, `navigation_terminal_and_parked`.
- 7,880 physics steps, final simulation39.415s, loop52.9856wall seconds. The run stopped after arrival rather than consuming the whole budget.
- At27.225s the native base crossed world Y=-9: XYZ `[-13.07827,-9.00250,-2.94260]`, contacts `[body,FR,FL,RR,RL]=[0,1,0,0,1]`.
- For the remaining12.19sim seconds, base Z stayed between**-2.960665 and-2.942603m**. All2,439 samples had at least one foot contact, none had body contact, and there was no fall below the floor. Native3D speed peak in that window was**1.013025m/s**.
- Final native XYZ `[-11.05606,-11.55699,-2.95209]`, four-foot contact, nearly zero velocity. The dog therefore physically entered and parked on the previously non-colliding6m_17 floor.
- Actual runtime bounds now include6m_17 with X[-15,-9],Y[-15,-9],Z≈-3.3. Session receipt reports119 enabled floors, source_saved=false and source_layers_unchanged=true. The source file SHA was rechecked after the run and remained identical.
- All recorded owned PIDs exited. Original SDK fast shutdown leaves app_close_returned=false; exit0, encoded video and outer cleanup were verified separately.

Video: `capture/multiwaypoint05_overview_and_rgb.mp4`,394frames,10fps,39.4s, encoder exit0. Evidence: `floor_seam_validation.json`, `floor_collision_validation_audit.json`, `static_floor_inventory.json`, `static_floor_verified_session.json`, source snapshot/manifest, actual collider bounds, native/contact streams and original logs.

## Tests and boundary

Focused actual-USD tests cover valid repair, idempotence, invalid/hidden/tilted/degenerate floor refusal and session rollback. Route tests preserve the original path/height/tolerance assertions while checking the seven legacy floors remain a subset of the expanded set.

Final full suites: unit1,048 tests OK (3 skipped); regression108 tests OK (8 skipped). Actual-USD focused suite7 passed; diff-check passed.

All119 floors were checked statically; actual robot traversal was tested only across the previously failing seam. This is not a full-building collision/traversal certification or a strict same-budget performance A/B. Do not mask similar asset defects using truth-coordinate waypoint blacklists.

No commit or push.
