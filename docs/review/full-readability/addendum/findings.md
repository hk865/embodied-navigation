> Root 最终复验已完成：54 项单元通过；回归 57 通过、5 项缺少 Torch 跳过；304 项当前来源指纹与原 VIO 补丁检查通过，PCT CPU 编译/合成路径通过。以下保留 DSH 分工阶段记录，其中“待 root 刷新/复验”已由[最终验证记录](../verification.json)接续；完整 ROS/仿真/实机/训练仍未验证。

# Addendum readability review — findings

Scope: the 14 third-party files in `scope-addendum.json` that the two main
readability tasks do not own — 8 `third_party/livox_ros_driver2/launch_ROS1/*.launch`,
2 `third_party/scan/scan_planner_msgs/msg/*.idl`, 4 raw patch artifacts.
Baseline commit `5e7982ea3c82f2ae263b46e156a90cafca86780c`.
Machine-readable per-file record: `coverage.json` (same directory).

Result: **14/14 files read end-to-end and classified; 10 changed, 4 kept
byte-exact (`locked_exact`), 0 `already_clear`, 0 pending, 0 duplicates.**
Read-only source evidence was taken from the vendored driver (`src/`), the
vendored planner (`third_party/scan`), `README.md` files, `SOURCES.json` and
`docs/dependencies/vendored-source-manifest.json`. No ROS, simulation, robot,
install or Git command that mutates history was run.

## 1. Launch XML (8 files)

All eight are the same skeleton in two profile families across four LiDAR
configurations: 13 `<arg>`, 11 `<param>`, the `livox_lidar_publisher2` driver
node, an optional rviz group and an optional rosbag group (28 XML elements for
`msg_*`, 30 for `rviz_*`).

| profile | xfer_format default | rviz_enable default | rviz group | config |
| --- | --- | --- | --- | --- |
| `msg_HAP` / `msg_MID360` / `msg_MID360s` / `msg_mixed` | 1 (Livox CustomMsg) | false | absent | `*_config.json` |
| `rviz_*` counterparts | 0 (PointCloud2) | true | present | same four configs |

Work applied to all eight: file-purpose header (model, profile, selected
`user_config_path`), documented argument semantics, documented parameter
consumption and the `output_type` → `output_data_type` rename, labelled the
driver / rviz / rosbag nodes, normalised mixed tabs-and-spaces indentation to
two spaces and removed trailing whitespace. Tag names, attribute names and
order, attribute values, child order, `if` conditions and node startup
behaviour are bit-for-bit equivalent at the parsed-XML level.

Observations worth knowing (documented in comments, **not** changed):

1. `rviz_enable` is declared in every `msg_*` file but referenced by no node or
   group there — it is dead interface surface copied from the `rviz_*` profile.
   Behaviour is unaffected; removing it would change the launch interface, so it
   was left as is.
2. The driver only reads `xfer_format`, `multi_topic`, `data_src`,
   `publish_freq`, `output_data_type`, `frame_id`, `enable_lidar_bag`,
   `enable_imu_bag` and `user_config_path`
   (`src/livox_ros_driver2.cpp`). `cmdline_str`, `cmdline_file_path` and the
   node `args` (the `bd_list` broadcast-code list) are compatibility fields that
   this snapshot never consumes; the LiDAR/network setup comes from
   `user_config_path`.
   `output_type=1` selects a legacy driver-bag branch, but the entry point
   leaves `bag_=nullptr` and never calls `CreateBagFile`. The bag flags do not
   open files; actual recording uses the separate `rosbag_enable` node.
3. `data_src` is effectively pinned: `livox_ros_driver2.cpp` accepts only
   `kSourceRawLidar == 0` and errors for anything else, so the declared
   `lvx_file_path`/LVX playback path is not reachable in this snapshot.
4. `MID360s` profiles exist and select `config/MID360s_config.json`
   (`{"lidar_summary_info", "Mid360s", "lidar_configs"}`), although the upstream
   README launch table documents only HAP, MID360 and mixed. This is documented
   in the two `MID360s` headers rather than "fixed".

## 2. IDL interfaces (2 files)

Both are checked-in rosidl artifacts (`generated from rosidl_adapter/...`); the
build (`CMakeLists.txt`) generates interfaces from the sibling `.msg` files, so
the `.msg` remains the true generation source and is outside this task's write
scope. Changes are comment-only.

* `Bspline.idl` — 7 fields kept in order and type
  (`int32 order`, `int64 traj_id`, `builtin_interfaces::msg::Time start_time`,
  `sequence<double> knots`, `sequence<geometry_msgs::msg::Point> pos_pts`,
  `sequence<double> yaw_pts`, `double yaw_dt`). Comments record the verified
  semantics: the producer (`scan_replan_fsm.cpp`) emits `order = 3`, sets
  `start_time`/`traj_id` from `local_data_`, fills `pos_pts` from
  `position_traj_.getControlPoint()` and `knots` from `getKnot()`; the knot
  length is control points + order + 1 per
  `UniformBspline::setUniformBspline` (`m_ = n_ + p_ + 1`, `u_` sized `m_ + 1`).
  `yaw_pts`/`yaw_dt` are declared but never populated by this producer, which
  the comments say explicitly instead of inventing a meaning.
* `DataDisp.idl` — 6 fields kept in order and type. The only assignment in the
  vendored producer is `data_disp_.header.stamp = node_->now()` before
  publishing on `planning/data_display` (`scan_replan_fsm.cpp:1010`); `a..e`
  are never touched anywhere in `third_party/scan`, so for this producer's
  messages they keep `0.0` and carry no definition. The comments say that and
  also state that the interface neither names nor reserves the fields, so
  another publisher may assign them — a producer-scoped fact, not a global
  prohibition. This postimage is newly authored over the unchanged baseline
  interface; it does not reproduce the earlier, unrecovered comment checkpoint.

Token-level identity was proven by a comment-aware tokenizer (59 tokens for
`Bspline`, 40 for `DataDisp`), so interface fields, order, types and all other
tokens are unchanged.

## 3. Raw patches (4 files, byte-locked)

Not modified by even one byte (`sha256` equal to the addendum baseline; worktree
clean for all four). Editing them, including "formatting", would risk their
applicability and their recorded identity, so the review records behaviour,
baseline, tests and provenance instead.

1. `third_party/fast_livo2/patches/0001-consistent-negative-voxel-floor.patch`
   (4 hunks in `fast_livo2_core/src/vio.cpp`, 6 added / 6 removed lines).
   Replaces `loc_xyz[j] = p[j] / voxel_size; if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0;`
   with `floor(p[j] / voxel_size)`. The legacy formulas differ by site. Hunks 1/4 store a float quotient,
   decrement when negative, then convert to int64_t: for exactly representable
   examples, -1.5 becomes int(-2.5)=-2 (correct floor), but -2 becomes -3
   instead of floor(-2)=-2. This is not floor(x)-1 for every negative quotient;
   float rounding near boundaries also matters. Hunks 2/3 already assign floor
   to an int array and then subtract an extra unit for negative x. The fix
   unifies these sites on floor. No runtime failure rate was measured. Already applied in
   the baseline snapshot: the HEAD blob of `vio.cpp` matches every hunk
   post-image line exactly (verified at new-side lines 255/435/563/1154).
   Manifest entry marks `vio.cpp` `modified: true` with this file as `patch` and
   `patched_source` provenance.
2. `third_party/fastdds/heartbeat_2_14_6.patch` (2 hunks, 11 added / 5 removed).
   Production backport for Fast DDS 2.14.6 `SharedMemGlobal.hpp`: listener side
   keeps the closed/data fast paths and then waits without a predicate so a
   data-less health notification still refreshes the heartbeat; checker side
   notifies idle listeners and retries after 1 ms instead of sleeping the full
   `port_wait_timeout_ms`. Total health timeout (`healthy_check_timeout_ms` =
   1000), data/close fast paths, reliable QoS and SHM+UDP topology are
   unchanged. Baseline: upstream commit `f052f239...` pinned in
   `third_party/fastdds/SOURCES.json`; backports PR #6559 (reviewed head
   `2184ef9d...`, issue #6535), recorded as open/unmerged on 2026-10-25, so it is
   a local private backport, not an upstream release.
3. `third_party/fastdds/heartbeat_2_14_6_tests.patch` (3 hunks, 206 added / 0
   removed). Test-only: `prepare_idle_health_check` (forces the watchdog
   counter-equality precondition) and `wait_pop_legacy` (exact pre-backport
   predicate as negative control) plus five `TEST_F` cases — cold open with 1
   and 8 concurrent checkers, legacy-listener timing, stopped-listener total
   timeout, and normal data plus close. Applied second in the documented order.
4. `third_party/fastdds/heartbeat_fixture_r2.patch` (6 hunks, 45 added / 8
   removed). Test-only hardening: the fixture also resets
   `last_listeners_status_check_time_ms` under `empty_cv_mutex` (mirroring the
   real WatchTask) and requires `is_port_ok`, so counter equality alone cannot
   make the watchdog declare a live listener dead; `same_live_node` checks UUID
   identity + `is_opened_for_reading` + `num_listeners == 1` so a regenerated
   port handle cannot pass as the original; `first_error` and per-checker error
   text are propagated and fatal lifecycle checks switched from `EXPECT_*` to
   `ASSERT_*`. Applied third.

Applicability of the Fast DDS patches cannot be re-run locally: the target tree
is not vendored (archives are fetched at build time from the `SOURCES.json`
pins). Byte-identity plus hunk validation establishes unchanged patch material,
not application to a changed target tree. A concurrently reformatted vio.cpp
may require the separate exact formatting layer before reversing the floor fix.

## 4. Over-strict or redundant checks — evidence and suggestions only

No parameter or threshold was changed anywhere. Two observations:

1. **Timing-based assertions in `heartbeat_2_14_6_tests.patch`** are wall-clock
   sensitive: `EXPECT_GE(elapsed, 100.0)`/`EXPECT_LT(elapsed, 1500.0)` for the
   legacy listener, `EXPECT_GE(elapsed, 900.0)`/`EXPECT_LT(elapsed, 2500.0)` for
   the stopped listener, `EXPECT_LT(elapsed[i], 250.0)` per concurrent checker,
   over 16 rounds with a 23 ms sleep each. Scheduling delays may affect upper elapsed-time bounds, but no failure
   or timing distribution was measured here. Lower bounds enforce negative-
   control and total-timeout behavior; blanket widening weakens these tests.
   Suggestion (report-only): collect distributions and separate scheduling
   margins from semantic lower bounds before considering threshold changes
   or deterministic fixture/clock options. The patch is
   byte-locked, so this belongs to whoever owns the backport.
2. **Dead compatibility surface** (`cmdline_str`, `cmdline_file_path`, node
   `args`, and `rviz_enable` in `msg_*` launches) is not a validation rule but
   reads like one: it suggests LVX-file playback and rviz toggling that this
   snapshot cannot honour. Evidence: only 9 `getParam` calls in
   `src/livox_ros_driver2.cpp`; `data_src != 0` errors out. Suggestion
   (report-only): document upstream or remove in a separate change, not inside a
   readability pass that must preserve values.

No over-strict threshold was found inside the 8 launch files or the 2 IDL files
themselves; `publish_freq`'s `[0.5, 100.0]` clamp is a driver-side guard, not a
review threshold.

## 5. Validation performed

Per format, with thresholds:

* **Launch XML** — `xmllint --noout` rc = 0; exact deep equality against the
  HEAD blob of `(tag, ordered attributes, children)` (0 differences); comment-
  stripped content-line multiset equality (0 differences); every `$(arg NAME)`
  resolves to a declared arg (0 unresolved); arg names unique; element counts
  equal to baseline (13/11/2-3).
* **IDL** — non-comment token sequence equality (0 token differences); struct
  `(type, name)` order equality (0 differences). No `idlc`/`omniidl`/`fastddsgen`
  is installed, so this is a token-identity proof, not a compile.
* **Patches** — `sha256` and byte count equal to the addendum baseline (0 bytes
  different); every `@@` hunk header's old/new counts equal its body (0
  mismatches); `fast_livo2` post-image matches the HEAD blob exactly;
  Fast DDS patch hashes match `SOURCES.json` `backport.patch_sha256`.
* **Scope** — the 10 owned content files are the only ones this task changed;
  all four `.patch` files are untouched; concurrent project/vendor task changes
  in the worktree were observed, not written and not reverted.

Aggregate: 0 validation problems across all 14 entries
(`scratch/validation-restore.json`, produced by `scratch/validate_restore.py`).

## 6. Limitations

* No `roslaunch`/ROS/simulation run in this source-readability task, so launch equivalence is
  proven on XML structure and text, not by starting nodes.
* No IDL compiler available; interface equality is token-level.
* The `.idl` files are generated artifacts: regenerating from `.msg` would drop
  the comments. The comments are documentation of the checked-in artifact.
* Fast DDS hunks are not re-applied locally (no vendored target tree).
* Only the 14 addendum files were reviewed; the other two tasks own 575 files
  (310 project + 265 vendor); these 14 bring the total to 589; those 575 files were not re-reviewed here.

## 7. Follow-ups for the parent / main task (not done here — out of write scope)

`docs/dependencies/vendored-source-manifest.json` lists 10 of the changed files
as destinations and this addendum invalidates their fingerprints: `vendored_sha256`, `bytes` and
`modified` must be refreshed while `source`/`source_sha256`/`change`/`patch`
identity is preserved. All four patch artifacts remain byte-locked; only the three FastDDS patches
are direct manifest destinations. The floor patch is linked by the vio.cpp
record. Other concurrent tasks may invalidate additional destination hashes. The eight launch rows are under
`third_party/livox_ros_driver2/launch_ROS1/`. Exact new values:

| path | old sha256 (baseline) | new sha256 (final) | bytes (old -> new) |
| --- | --- | --- | --- |
| `msg_HAP.launch` | `2bb9adeab763da7a414fe756fa0569e41ab4795722817ad950a39450f97813ef` | `c2286adf949f245c88de33ed92f6245af3194debba7475eaac37b29af2030f97` | 1743 -> 3272 |
| `msg_MID360.launch` | `b4a86ae5e58d354e454140fe6e3b0032193826f764ed8da9f6d1519ba99f60dd` | `3adbbb71afed30733a59903ccdd09f97a818f298b5e2d6a2a92d6e9ff516a8f3` | 1744 -> 3281 |
| `msg_MID360s.launch` | `623d0aac93ff6bb54688fb132f441248b33d7331376df30ebb77e736023da850` | `92f92c8095aff162e4be116f411d4d6c131e9d874bc75382d4a836866454394f` | 1745 -> 3341 |
| `msg_mixed.launch` | `77c8f7eed8d2702211f52db053f792f2b5906123e8ab647a913313bb015c6ab8` | `dc9409a354f1af98f47faa09875cca7f98c76754435e0f8501c4cb22198ead6b` | 1754 -> 3296 |
| `rviz_HAP.launch` | `be4b09a282eb005128933ec802d0658d273ebe5b91dc65d0413a78c603ee3f78` | `515dec690b09f257ef4c443c78a65ee972df6703191417a55995123b2f35be05` | 1930 -> 3499 |
| `rviz_MID360.launch` | `03a622737d58ac346261ad5bf87df109d3c339184c1baeee11ba8371396f6578` | `24722510c644377d632bb36839d9cc78bc2c308fa76e1f7bf6c3f9b2f683734e` | 1933 -> 3508 |
| `rviz_MID360s.launch` | `0786538975f8cb314680f747d3ddacd4d093093e944d94b1d9f10a67bccb2b95` | `dca8b5474c6c84ae7920c43c308c2dc751860fcdd5531b1b113ae20265ea7eb2` | 1934 -> 3568 |
| `rviz_mixed.launch` | `bb9830540646897bb52e4abd76947232a751b27f0f9ec7c4fedc953bb4fbcb79` | `63df18a8d45993fe1f7456b833df7f0a3181a2980bac54da4daae9a10aad4b24` | 1943 -> 3523 |
| `third_party/scan/scan_planner_msgs/msg/Bspline.idl` | `470fbc58062b48b37225fd86e8dba61c450fb9755139234d84c384768082c61d` | `0fef041dfe996e9e1afb3cb6b85ffe70379cd271bd1674d7625dc915a67d5984` | 550 -> 1436 |
| `third_party/scan/scan_planner_msgs/msg/DataDisp.idl` | `93cc21404dd9b557e9af8a52c65a4368d6bd6ab2a28d069dfb00bec311c4074e` | `a01293ba83eddc49aaaea82c4aa4d13e3ab2b44a5fd8dc744c050993c5f50d5b` | 398 -> 1545 |

Full hashes are in `coverage.json` (`baseline_sha256`/`final_sha256`) and
`scratch/validation-restore.json`. No manifest, README, docs or vendor source file was
modified by this task to keep it disjoint from the project/vendor teams.

## Independent fact review and serial restoration (completed)

An independent fact review corrected the DSH draft's descriptions (floor
arithmetic per site, the 575 + 14 = 589 scope count, patch applicability versus
byte identity, the ROS1 bag branch that never opens a file, the producer-scoped
IDL yaw facts, manifest membership, and the timing-threshold advice). It also
supplied nine reviewed comment candidates that keep XML values/structure and IDL
tokens intact.

After both main DSH tasks completed, the previously reverted files were restored
serially. Before writing, all ten content files were confirmed to be at their
`scope-addendum.json` baseline SHA, so no unknown concurrent edit was
overwritten. The eight launch files and `Bspline.idl` were written byte-exact from
the reviewed candidates. `DataDisp.idl`'s earlier comment checkpoint was not
recoverable; it was instead re-authored from the unchanged baseline interface with
accurate producer-scoped comments and has a **new** final SHA
(`a01293ba83eddc49aaaea82c4aa4d13e3ab2b44a5fd8dc744c050993c5f50d5b`) that does not claim to reproduce the old one. All four original patch
artifacts remained byte-identical.

This report is the post-review, post-restore current-source receipt: every
`final_sha256`/byte count above was recomputed from disk and all 14 entries were
re-validated (`scratch/validation-restore.json`, produced by
`scratch/validate_restore.py`); baseline SHAs were preserved. No vendor or
project file was restored, reverted or rewritten by this task.
