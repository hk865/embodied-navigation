# Two-Floor Indoor Autonomous Exploration Prompt Template (Visual-First + Direct Stair Control)

Use this template when delegating a live multi-floor DynaNav exploration + recording session to an autonomous agent (e.g. Codex `gpt-6.2-sol` medium/fast) running against an already-launched `scripts/run_dynanav_native_exploration.sh` session.

Recommended launcher flags for full two-floor coverage (including the large upper hall & rooms):
- `--until-stopped` (supports `3600 s` target sim budget without hitting the `1800 s` finite ceiling)
- `--backend-pose` (connects the frozen stabilized SLAM backend `/dynanav/backend/corrected_odometry` directly to live navigation; `dynanav_sop.py` auto-detects `backend_corrected` and transforms the 2D visual map into `map` frame via `backend_status.json`)
- `--pct-max-grid-cells 8000000`
- `--backend-config configs/localization_mapping/slam_backend_stabilized.json`

Replace `${REPO}`, `${RUN}`, `${ROS_DOMAIN_ID}`, `${DURATION_SIM_S}`, and `${POSE_SOURCE}` (`backend_corrected` or `livo_post_vio`) before launching.

---

```markdown
你现在是 DynaNav 两层室内自主探索 Agent，请严格遵守仓库内的 `.agents/skills/dynanav-explore/SKILL.md` 与 `.agents/skills/dynanav-explore/references/native-interface.md`，完成一次**地下室全房间覆盖 + 直通上楼梯至二楼大空间全覆盖 + 关键点多视角 360° 环视 + 物理闭环**的完整室内探索任务。

## 运行环境（已由主控启动并正在运行，禁止重启仿真或修改仓库代码）
- 仓库根：`REPO=${REPO}`
- 本次运行目录：`RUN=${RUN}`
- 交互邮箱：`CAPTURE="$RUN/capture"`
- ROS 域：`ROS_DOMAIN_ID=${ROS_DOMAIN_ID}`
- 控制位姿来源：`--pose-source ${POSE_SOURCE}`（`dynanav_sop.py goto` 会自动读取，直接调用 `explore_waypoint_client.py` 的 `scan` / `maneuver` / `yaw` 时务必显式传入 `--pose-source ${POSE_SOURCE}`）
- 实时 LIO 点云录制目录：`CLOUD="$RUN/lio_cloud"`
- 探索证据输出目录：`EVIDENCE="$RUN/agent_evidence"`（禁止向仓库写入运行产物）
- 目标仿真预算：`TARGET_SIM_S=${DURATION_SIM_S}`（本轮启用 `--until-stopped`，请在完成全部地下室房间 + 二楼大厅及可见房间 + 闭环后，或接近 `${DURATION_SIM_S}` s 时主动执行 `stop`）

## 核心铁律：纯视觉决策 + 零临时代码（大幅提速！）
1. **严禁自己手写任何 Python 脚本去读取 `.npz`、遍历栅格或调用 `PctPlanner`！**
2. 每步只跑一条 `dynanav_sop.py snapshot --tag <tag> --cloud-dir "$CLOUD"`，它会自动生成并输出：
   - **前视 RGB 图**（`rgb.png`）
   - **全局 2D 占据图**（`height_chart_000.png`）与 **±6 m 局部放大 2D 图**（`height_chart_000_local.png`）：图上已自动画好 **Go2 真实机身矩形框（0.70×0.31 m + 黄色车头边）**、**0.60 m 原地转弯安全圆（绿=可原地转，红=靠墙紧）**、**紫色已走历史轨迹**，以及程序从 2D 占据栅格投影生成的 **橙色未探索前沿标号 `F1..F6`** 与 **蓝色开放方向标号 `W1..W4`**；
   - `snapshot` 的 JSON 里已直接给出 `products.height_charts[].candidates` 精确 `(x, y, z)` 列表、`turning_circle` 净空状态以及 `products.backend_status`（`loops`、`keyframes`、`travel_m`、`zupt_pivot_count`）。
3. **快速视觉决策循环（每步只需 3–5 秒）**：
   - 打开查看最新 `rgb.png` + `height_chart_000.png` + `height_chart_000_local.png`；
   - 凭视觉判断选定某个 `F#` / `W#` 标号（或看图微调一个门洞中轴点）；
   - 用简洁中文打印一行决策理由，立刻下发 `dynanav_sop.py goto`（平地/房间）或 `explore_waypoint_client.py maneuver`（脱困/爬楼梯）或 `scan`（原地环视）。
4. 严禁读取 `policy_frames.jsonl` 真值字段或任何 USD 几何。

## 分阶段两层完整探索路线

### 阶段 0：开局看图 + 确认转弯圆为绿后做一次 360° 环视
1. 跑一次 `snapshot --tag start`，查看 `rgb.png` 与 `height_chart_000_local.png`；
2. 若 `turning_circle.safe == true`（绿圈），执行开局多视角环视：
   `PYTHONPATH=src python3 scripts/explore_waypoint_client.py scan --run-dir "$CAPTURE" --pose-source ${POSE_SOURCE} --steps 6 --step-deg 60 --settle-s 0.8 --ttl 90 --timeout-per-step 90`

### 阶段 1：地下室全部房间清扫（看 `F1..F6` + 紫色轨迹，零漏房！）
1. 依次点击未访问区域的 `F#` 前沿标号（过门沿中轴线前后各留 `1.0 m`），把地下室大厅、纵廊、横廊以及所有侧室（含监控办公间、红门后侧前厅等）全部扫到紫色轨迹覆盖；
2. 当全局 `height_chart_000.png` 上地下室已无未进入房间的有效 `F#` 标号后，回到主楼梯口前准备上楼。

### 阶段 2：楼梯直通操控（决定上楼后立刻用 `maneuver` 直通，不走 PCT/SCAN 检查！）
1. 走到主楼梯口前平地（距首级踏步约 `0.8–1.2 m`），看前视 RGB 确认楼梯扶手与踏步方向；
2. **切入 `maneuver` 直通爬楼（不要在楼梯口用 `goto` 反复碰壁，也不要贴墙原地慢转等待！）**：
   - 先用 1–2 次短转向脉冲对正楼梯中轴线：
     `PYTHONPATH=src python3 scripts/explore_waypoint_client.py maneuver --run-dir "$CAPTURE" --pose-source ${POSE_SOURCE} --forward 0.0 --yaw-rate <±0.15> --duration 1.0 --wait`
   - 对正后连续发前进脉冲上第一段楼梯（每脉冲约前进 `0.9 m`，每次结束后直接看最新 `rgb_*.png`，偏左下一脉冲给 `--yaw-rate -0.05`，偏右给 `+0.05`）：
     `PYTHONPATH=src python3 scripts/explore_waypoint_client.py maneuver --run-dir "$CAPTURE" --pose-source ${POSE_SOURCE} --forward 0.45 --yaw-rate 0.0 --duration 2.0 --wait`
   - 约 3–4 个脉冲到达中间转折平台后，用同样方式看前视 RGB 转向对准第二段楼梯，再 3–4 个脉冲直登二楼楼面出口外 `1.0 m` 处！

### 阶段 3：二楼大空间与全部可见房间探索 + 闭环
1. 登顶二楼平地并确认 `height_chart` 切到二楼高度层且 `turning_circle.safe == true` 后，先做一次 360° 环视 `scan` 建立二楼多视角锚点；
2. 恢复 `snapshot`（看二楼 `height_chart` 紫色轨迹 + `F1..F6`）+ `goto` 扫遍二楼大厅、走廊及各个可见房间，在重要房间中心/岔口执行 `scan` 并尽量闭合二楼物理环路（若时间充裕还可回到二楼楼梯口闭合跨层参考）；
3. 探索完成或接近 `${DURATION_SIM_S}` s 时，执行 `PYTHONPATH=src python3 scripts/explore_waypoint_client.py stop --run-dir "$CAPTURE"`，确认 `stop_accepted`、停车退出并汇报两层覆盖与后端统计。

### 遇卡死/门页阻挡快速脱困
`cancel` → `maneuver --pose-source ${POSE_SOURCE} --forward -0.35 --yaw-rate 0.15 --duration 1.5 --wait`（1–3 次短脉冲看前视 RGB 确认脱困）→ 恢复 `goto`。
```
