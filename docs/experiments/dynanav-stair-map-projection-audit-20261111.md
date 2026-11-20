# Latest operator map: stair projection audit

Date: 2026-11-11. CPU-only inspection of the latest saved domain207 run. No simulator, model dispatch, control request, production edit, scene edit, map mutation or guard change was made.

## Finding

The 2D operator occupancy map cannot currently express stair traversability. It projects returns 0.15–2.2 m above the established floor into OCCUPIED cells. Measured stair treads/risers above that band start therefore appear occupied, while unobserved treads stay UNKNOWN. The renderer copies cells directly and does not inflate them. This is expected conservative occupancy behavior, but is insufficient as a legged-robot terrain/traversability representation.

This does not prove that the robot is physically unable to climb stairs. Earlier measured post-VIO route experiments completed two flights, with preset route/reference and oracle static-cloud limitations described in `dynanav-20261109-continuation.md`. The original `dynanav-post-vio-final-capture-20261109T055946Z/audit/independent-review.json` was also inspected: all 13 observed-run checks passed, with arrival, parking, three stages and two turns. The command ceiling was 0.15 m/s. That historical result does not certify current DSH autonomous stair detection, the current RTX pipeline, or arbitrary stairs.

The exact map/image the user was referring to has not been identified. This audit covers the newest saved domain207 export, rather than asserting that every previous map has identical behavior.

## Actual inputs and measurements

Run: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_candidate_async_dsh_20261111T034100Z`.

- Grid: `operator_grid/level_+0.01_operator_map_partial.npz`, SHA256 `4b97021413a44ab072299ea23378f74ae55418512ac56f751b7d1feef1d57ea0`. The manifest hash matched the actual NPZ before inspection.
- Frame: `dynanav_lio_map`. Resolution: 0.05 m. Classification floor: −0.342991124 m. Global cells: 37,171 free / 4,007 occupied. FREE uses actual floor endpoints, not ray-carved space. Last exact LIO cloud/pose/reference stamp is 34.8 s.
- Display: black = OCCUPIED, light grey = FREE, grey = UNKNOWN. White exists only outside the export ROI. There is no robot marker. The final manifest is non-actionable because wall-age expired; the saved map is historical geometry, not live permission to move.
- Coarse rectangle around the left stair-shaped measured structure: x ∈ [−6.5, −0.6] m, y ∈ [−1.4, 1.4] m. Counts by cell center: 1,849 occupied, 0 free, 4,759 unknown, total 6,608. This is a rough rectangle including surrounding/occluded cells, not an exact stair semantic mask or a stair passability percentage.
- Native `collector/cloud_full_008.npz` source stamp is exactly 5.3 s and has an exact matching measured LIO pose in `collector/received.jsonl`. No ground-truth/scene geometry was used for this analysis.
- In that rectangle, 7,614 native endpoints were measured. Of these, 7,603 satisfy the original 0.15–2.2 m floor-relative obstacle rule, projecting onto 967 cells. All 967 are actually OCCUPIED in the exported map.
- The central strip shows successive measured heights near 0.16, 0.31, 0.46, 0.62, and 0.80 m above the classification floor as x moves along the stair-shaped structure. These are observed step-like geometry, not a terrain capability test. Missing/occluded intervals are not interpolated.

## Code evidence

- `src/embodied_navigation/simulation/dynanav/operator_map.py`: `MIN_OBSTACLE_CLEARANCE_M = 0.15`, `DEFAULT_MAX_HEIGHT_M = 2.2`; the classification near lines 2328–2341 marks XY endpoints in that height band occupied and floor endpoints free. It uses one established floor height for that measured level, not per-cell stair support heights.
- `LevelGrid._mark`, near lines 984–1001: occupied updates are monotone; later floor returns do not clear an existing occupied cell. This is intentional conservatism, not evidence that a stair classifier ran and failed.
- `scripts/render_dynanav_operator_grid.py`: `render_grid` draws each original exported cell with a fixed colour. No morphological operation, footprint expansion or costmap inflation exists in this rendering path.
- `src/embodied_navigation/simulation/dynanav/exploration_waypoints.py`: the local follower accepts bounded measured-frame XYZ segments and produces body velocity/yaw commands. It is not a stair detector or grid-based global terrain planner. The comment expecting stairs to use short segments and the 4 m segment vertical-jump bound do not certify a climbable riser height.
- Movement still separately requires current pose, source freshness, complete floor support, clearance and direction coverage. Changing the displayed grid would not establish those conditions or permit bypassing them.

## Appropriate fix direction

Keep the current conservative occupancy evidence. Add a separate measured 2.5D terrain layer rather than relabeling occupied/unknown cells FREE:

1. Retain per-cell support-surface height, point count, local roughness/normal, height uncertainty, source time and observed/unknown state. Preserve actual multiple surfaces/vertical obstructions where one XY cell contains both tread and wall/rail.
2. Infer candidate stair sequences from repeated measured riser/tread geometry and continuous support. Check tread depth, riser height, width, slope, discontinuities, overhead/rail/body clearance, and the robot's tested motion envelope. RGB semantics can propose stairs but cannot by themselves authorize traversal.
3. Build distinct terrain costs and cross-level transitions for the planner, including UNKNOWN and not-yet-validated stair candidates. Do not apply one global threshold increase to remove stairs and walls together.
4. Integrate a terrain-aware execution/admission contract whose observed stair support is explicit. Preserve all existing safety gates until a replacement path is designed and independently validated, rather than disabling the present floor/unknown guards.
5. Validate offline on the saved measured clouds and synthetic stair/wall/drop/occlusion fixtures, then in a bounded simulation with actual trajectory and guard receipts. Gate/controller readiness is an independent blocker and must also be resolved.

No traversability layer or altered stair controller was deployed in this audit. The immediate basement exploration remains unfinished, with latest207 model/waypoint/nonzero command counts all zero.

## Reproduction and checks

The read-only audit script, machine-readable counts/profile/hash evidence, and rendered map are under repository `.dsh-subagents/stair_map_audit_20261111/`. Run `PYTHONPATH=src python3 -B .dsh-subagents/stair_map_audit_20261111/audit_stair_projection.py` from the repository root.

- Exact grid SHA and cloud/LIO stamp/frame checks passed.
- All 967 classified native obstacle cells were found occupied in the exported grid.
- Synthetic LevelGrid checks confirmed occupied cells stay occupied after a free update and neighbouring cells remain unknown, with no implicit inflation.
- `test_operator_map.py`: 105 tests passed.
- `test_operator_grid_export.py`: 11 tests passed.

These are CPU data/contract checks, not live navigation or stair traversal validation.
