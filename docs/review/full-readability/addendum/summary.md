> Root 最终复验已完成：54 项单元通过；回归 57 通过、5 项缺少 Torch 跳过；304 项当前来源指纹与原 VIO 补丁检查通过，PCT CPU 编译/合成路径通过。以下保留 DSH 分工阶段记录，其中“待 root 刷新/复验”已由[最终验证记录](../verification.json)接续；完整 ROS/仿真/实机/训练仍未验证。

# Summary — third-party readability addendum (14 files)

Baseline `5e7982ea3c82f2ae263b46e156a90cafca86780c`, worktree
`/home/hyh001/projects/1.Project/embodied-navigation`. This task covers only the
14 files that the project (310) and vendor (265) readability tasks do not own:
8 `.launch` XML, 2 `.idl`, 4 raw `.patch`. Per-file record: `coverage.json`;
detailed evidence and follow-ups: `findings.md`.

## Counts

| metric | value |
| --- | --- |
| files in scope / read end-to-end | 14 / 14 |
| changed | 10 (8 launch + 2 IDL) |
| already clear (no edit) | 0 |
| byte-locked, not edited | 4 patches |
| pending / missing / duplicate | 0 / 0 / 0 |
| validation problems | 0 |

## What changed

* **8 `third_party/livox_ros_driver2/launch_ROS1/*.launch`** — added a purpose
  header per profile (LiDAR model, msg vs rviz, selected `user_config_path`),
  documented all 13 launch arguments and 11 ROS parameters (including the
  `output_type` → `output_data_type` rename and the compatibility-only
  `cmdline_str`/`cmdline_file_path`/`args` fields). The legacy `output_type=1`
  branch never opens a driver bag here; use `rosbag_enable` for the separate
  recorder. Labelled the driver, rviz and
  rosbag nodes, normalised indentation to two spaces and removed trailing
  whitespace. Parsed XML is identical to baseline: same tags, attribute order and
  values, child order, `if` conditions and node start behaviour.
* **2 `third_party/scan/scan_planner_msgs/msg/*.idl`** — comment-only: noted that
  they are generated from the sibling `.msg` and are wire contracts, then
  documented the producer-scoped facts (`Bspline`: `order`=3, `traj_id`,
  `start_time`, knots length = control points + order + 1, `pos_pts`, and that
  `yaw_pts`/`yaw_dt` are declared but unpopulated by this producer; `DataDisp`:
  only `header.stamp` is set, so `a..e` stay 0.0 with no definition from this
  producer, while the interface neither names nor reserves them). Token stream
  identical (59 and 40 tokens), field order/types identical. `DataDisp.idl`
  has a newly authored comment postimage and a new final SHA; it does not
  claim to reproduce the earlier, unrecovered comment checkpoint.
* **4 patches untouched, byte-exact** — reviewed and explained instead of
  reformatted: the fast_livo2 voxel-floor fix (already applied in the baseline,
  post-image verified line by line), the two-sided Fast DDS 2.14.6 SHM heartbeat
  backport, its regression tests, and the r2 fixture hardening. Patch materials are unchanged; applicability to changed target sources
  requires separate verification.

## Validation

`xmllint --noout` + exact XML tree equality + comment-stripped line-multiset
equality + `$(arg)` resolution for the eight launch files; comment-aware token
equality + struct field equality for the two IDLs; sha256/byte identity +
unified-diff hunk-count integrity + post-image check for the patches (target
sources not vendored for the three Fast DDS patches, so those are validated by
byte identity, structure and `SOURCES.json` hash cross-check). Scope check: the
10 changed files are exactly this task's write set, all four patches are
unmodified, and concurrent project/vendor task changes were neither written nor
reverted. All 14 entries: pass, 0 problems
(`.dsh-subagents/2026-11-08T20-08-38-612Z-dc583daa/scratch/validation-restore.json`).

## Honest limits

No `roslaunch`/ROS/simulation/robot run and no install/Git mutation was performed,
so this is static equivalence, not a runtime proof. No IDL compiler is installed,
so interface equality is token-level. The `.idl` files are generated artifacts
whose comments would be lost on regeneration (`.msg` is the source and is outside
this write scope). Only the 14 addendum files were reviewed; the other two tasks own 575 files
(310 project + 265 vendor); these 14 bring the total to 589.

## Follow-up for the parent (not done here)

`docs/dependencies/vendored-source-manifest.json` has 10 destination
fingerprints invalidated by this addendum (8 launch + 2 IDL) — refresh `vendored_sha256`, `bytes`, `modified`
only; exact new values are tabulated in `findings.md` §7 and `coverage.json`.
All four patch artifacts remain byte-locked; only the three FastDDS patches
are direct import entries (the floor patch is linked by vio.cpp). Report-only observations: unused
`rviz_enable` in `msg_*` launches, dead `cmdline_*` compatibility surface, and
wall-clock-sensitive thresholds in the Fast DDS test patch, left unchanged:
no timing distribution was measured, so any adjustment needs data first.

## Independent review and serial restoration

This is the post-independent-fact-review, post-restore receipt. After the two main
DSH tasks completed, the files that had been concurrently reverted to baseline
were confirmed to still be at their `scope-addendum.json` SHA and then restored
serially: the eight launch files and `Bspline.idl` byte-exact from the reviewed
candidates, and `DataDisp.idl` re-authored from its unchanged baseline interface
with a new final SHA (its earlier comment checkpoint was not recoverable, so no
claim is made about it). All four original patch artifacts stayed byte-identical,
baseline SHAs were preserved, every final hash/byte count was recomputed from
disk, and all 14 entries were re-validated with 0 problems. No vendor or project
file was restored, reverted or rewritten by this task.
