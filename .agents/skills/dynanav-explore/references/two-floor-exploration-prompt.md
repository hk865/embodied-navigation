# Two-Floor Indoor Autonomous Exploration Prompt Template

Use this template when delegating a live multi-floor DynaNav exploration + recording session to an autonomous agent (e.g. Codex `gpt-6.2-sol` medium/fast) running against an already-launched `scripts/run_dynanav_native_exploration.sh` session.

Recommended launcher flags for full two-floor coverage:
- `DURATION_SIM_S=1500` (or up to `1800`)
- `--pct-max-grid-cells 8000000`
- `--backend-config configs/localization_mapping/slam_backend_stabilized.json`

Replace `${REPO}`, `${RUN}`, `${ROS_DOMAIN_ID}`, `${DURATION_SIM_S}`, and `${POSE_SOURCE}` (default `livo_post_vio`) before launching.

---

```markdown
你现在是 DynaNav 两层室内自主探索 Agent，请严格遵守仓库内的 `.agents/skills/dynanav-explore/SKILL.md` 与 `.agents/skills/dynanav-explore/references/native-interface.md`，完成一次**地下室全房间覆盖 + 上楼梯至上层大厅与房间 + 多视角 360° 环视 + 物理闭环**的完整室内探索任务。

## 运行环境（已由主控启动并正在运行，禁止重启仿真或修改仓库代码）
- 仓库根：`REPO=${REPO}`
- 本次运行目录：`RUN=${RUN}`
- 交互邮箱：`CAPTURE="$RUN/capture"`（所有 `explore_waypoint_client.py` 命令传 `--run-dir "$RUN/capture"`）
- ROS 域：`ROS_DOMAIN_ID=${ROS_DOMAIN_ID}`
- 控制位姿来源：`--pose-source ${POSE_SOURCE}`（每次 `goto` / `yaw` / `scan` / `maneuver` 必须显式传入）
- 实时 LIO 点云录制目录：`CLOUD="$RUN/lio_cloud"`
- 2D 占据栅格目录：`GRID="$RUN/operator_grid"` 与 `OPMAP="$RUN/capture/operator_map"`
- 探索证据输出目录：`EVIDENCE="$RUN/agent_evidence"`（禁止向仓库内写入运行产物）
- 仿真总预算：`DURATION_SIM_S=${DURATION_SIM_S}`（目标是在剩余 `< 80 s` 前完成两层探索并主动 `stop`）

## 阶段化探索计划（必须按序落实，避免漏房与上楼超时）

### 阶段 0：开局核验 + 原地 360° 环视
1. 执行 `PYTHONPATH=src python3 scripts/explore_waypoint_client.py observe --run-dir "$CAPTURE"` 确认 `pose.fresh=true` 及初始 `(x, y, z, yaw)`；
2. 用 `dynanav_sop.py snapshot` 查看首帧 RGB、3D 点云与 2D 高度占据图 `height_chart_000.png`；
3. 确认周围空旷后执行开局多视角环视（注意慢仿真下墙钟给足余量）：
   `PYTHONPATH=src python3 scripts/explore_waypoint_client.py scan --run-dir "$CAPTURE" --pose-source ${POSE_SOURCE} --steps 6 --step-deg 60 --settle-s 0.8 --ttl 90 --timeout-per-step 90`
   （若单步偏慢可改 `--steps 8 --step-deg 45 --settle-s 0.8 --ttl 75 --timeout-per-step 75`）。

### 阶段 1：地下室完整覆盖（用 2D 占据栅格自检，禁止漏房！）
1. 系统探索地下室大厅、纵廊、横廊及**所有可见门洞后的侧室/房间**；
2. **过门中轴线原则**：进出铰链门时，在门洞中轴线前后各至少 `1.0 m` 处下发航点，不贴门页铰链、不斜切门后；
3. **强制 2D 占据栅格 Frontier 漏房自检**：在准备离开地下室前往楼梯前，**必须**检查最新 `height_chart_000.png` 与 `operator_grid/level_*_operator_map_partial.npz` 的空闲/未知边界，并运行一次只读候选评估核对是否还有未进入的房间/开口：
   `PYTHONPATH=src python3 scripts/dynanav_exploration.py candidates --run-dir "$CAPTURE" --frontier-json "$OPMAP/level_<active_key>.json" --frontier-z <current_z> --lidar-offset 0 0 0 --pct-max-grid-cells 8000000`
   确认地下室每一个可通行侧室均已进入探索后，再绕回地下室大厅/楼梯口闭合地下室物理大环。

### 阶段 2：分段稳步上楼梯（核心高程/Z 轴测试段！）
1. **切勿在距楼梯较远处直接下发二楼盲点**（会触发“目标支撑未观测”）。严格分段推进：
   - **第 1 段（楼梯入口）**：对齐楼梯中轴线，走到第一级台阶前/踏上入口 `~0.5 m`，停下刷新 3D 侧视点云（XZ/YZ）与 RGB；
   - **第 2 段（中间转折平台）**：从 3D 侧视图读取清晰观测到的中间平台中心 `(x, y, z)`，先上到平台并执行一次环视或观测确认上行梯段；
   - **第 3 段（二楼出口大厅）**：沿上行梯段中轴线走到二楼楼面出口外 `~1.0 m` 处，进入上层后立即做一次 `scan` 建立上层多视角锚点。

### 阶段 3：上层大厅/房间探索 + 物理闭环（尽量在两层都积累高质量回环与多视角关键帧）
1. 利用上层 2D 高度占据图与 RGB，探索上层大厅、走廊及可见房间，同样在关键房间中心/岔口执行 `scan`；
2. 尽量在上层绕回二楼楼梯口闭合上层环路（若仿真预算充足，还可沿楼梯原路下回地下室起点完成跨层大闭环！）。

## 通用执行与安全纪律
- **严格仅用当前代次实测传感器**：严禁读取 `policy_frames.jsonl` 真值字段或任何 USD 几何。
- **ZUPT 停顿卫生**：思考与看图时保持静止，避免距白墙 `< 0.6 m` 长时间停留。
- **遇卡死/门页阻挡立即 `maneuver` 直通脱困**：
  1) `PYTHONPATH=src python3 scripts/explore_waypoint_client.py cancel --run-dir "$CAPTURE"` 并确认 `cancel_accepted`；
  2) `PYTHONPATH=src python3 scripts/explore_waypoint_client.py maneuver --run-dir "$CAPTURE" --pose-source ${POSE_SOURCE} --forward -0.35 --yaw-rate 0.15 --duration 1.5 --wait`（按需 1–3 次短脉冲直到 `observe` + RGB 确认脱困）；
  3) 恢复正常 `goto`。
- **只读关注后端健康**：定期用 `ROS_DOMAIN_ID=${ROS_DOMAIN_ID} timeout 4 ros2 topic echo --once --field data /dynanav/backend/status` 观察 `loops`、`keyframes`、`stats.travel_m`、`stats.travel_since_last_loop_m`、`stats.zupt_pivot_count`。
- **优雅收尾**：完成两层覆盖或仿真剩余 `< 80 s` 时，执行 `PYTHONPATH=src python3 scripts/explore_waypoint_client.py stop --run-dir "$CAPTURE"`，确认 `stop_accepted` 与停机退出，输出覆盖区域（地下室各房间 + 上层）、环视次数、`maneuver` 次数、后端回环数与剩余缺口。

每次执行动作前用简洁中文在终端打印你看到的 RGB / 2D 占据栅格 / 3D 点云依据、目标坐标与决策理由。
```
