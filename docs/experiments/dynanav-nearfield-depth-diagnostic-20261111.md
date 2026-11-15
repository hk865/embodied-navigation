# 新增仿真下视深度：先诊断，未授予通行

## 边界

这是一枚**新增的理想、无噪声、全局快门仿真深度相机**，不是原 Go2 硬件能力，也不是 RGB 推断或 RTX LiDAR。真实机器人需要对应下视深度传感器、真实标定与测量误差验证。默认关闭。`--simulated-nearfield-depth` 只启用归档，不接 SLAM、不发布原 native cloud、不改 perception.json、不向 operator map 加点、不产生控制许可。前视 RGB 保持 512²，RTX 扫描/时序不变，原 floor、TTL、pose、freshness 门不变。

单测中可见整个前向扇区不代表实地连续地面。默认朝向面对楼梯，远处踏面不能当作同层平地；未知、机器人遮挡后和无回波仍未知。左右后没有新增通行依据。

## 本机 API 已查证

本机 `/home/hyh001/isaacsim`，仅读源码，未启动 GPU：

- `extscache/omni.replicator.core-1.13.27+110.1.1.lx64.r.cp312/.../scripts/annotators_default.py` 注册 `distance_to_image_plane`、`CameraParams`、`instance_id_segmentation_fast` 和 `instance_id_segmentation`。
- 最终使用 `instance_id_segmentation`：SDK `OgnInstanceIdSegmentationLegacy.py` 将 fast 节点的 IDs/prim-path labels 包装为同图 `data` + `info.idToLabels`。无需给环境附加语义，不读取环境 mesh，不根据路径判断地面。
- `isaacsim.sensors.experimental.rtx.RtxCamera(path, tick_rate=10.0, reset_xform_op_properties=False)` 注册 FSD multi-tick 传感器。
- 复用已验证前视 RGB 的 native NEW_FRAME / ReferenceTime 路径。仅同 render product 事件进入回调；event reference 与 per-product reference 先交叉验证；同步读取 depth/instance/CameraParams 后，再读取 reference 并验证有理数相等。以 `get_simulation_time_at_time(native_reference_time)` 转秒后 round 到 ns，绝不以当前 physics clock 或旧 RGB 时间代替。

新增模块：`capture/nearfield.py`（纯 CPU 校准/反投影/掩码/归档）、`capture/nearfield_sensor.py`（SDK 装置/原生回调）。`sensors.py` 仅提取可覆盖的 native frame 收集方法，原 RGB 操作不变。`capture_go2_rgbd.py` / `frames.py` 接线；`experiment.py` 提供 opt-in 和完整 source manifest。

## 传感器与输出契约

- 256×256，10 Hz，轴向距离 .05–5 m，focal=12 mm，水平/垂直 aperture=42 mm；独立新传感器分辨率，不是原云下采样。所有有效端点保留，无点预算。
- base 静态 mount=[.32,0,.18] m，前向下俯55°。光学+X右/+Y下/+Z前。完整 R/K 和像素约定由 `calibration_receipt()` 提供。native CameraParams 的分辨率、焦距、aperture、offset、pinhole model、米制单位逐帧复核；世界位姿不参与反投影。
- 只掩码同帧 instance-ID 对应 `/World/Go2` 或其子路径。旧 x±.5/y±.3/z[-.45,.3] selfbox 不使用。未知ID、BACKGROUND、UNLABELLED、无效深度和 self 像素都是 unknown，绝不补遮挡后地面。
- `capture/nearfield_depth/frame_%019d.npz`：stamp_ns、depth_m、instance_ids、pixel_valid/pixel_self/pixel_unknown、points_base_m、point_pixel_flat_indices。无 pickle。每帧 immutable，不覆写。
- 同 stem immutable JSON + 原子 latest.json：schema=`dynanav-sim-nearfield-depth/v1`，source=`simulated_downward_depth_camera`，原生 stamp/source_time/arrival_wall(perf_counter)、各 reference、后读 reference、转换公式、K/R/t、完整 ID→prim 映射和 SHA、NPZ路径与 SHA、掩码计数、耗时/排队/physics lag；控制/SLAM/map 均 false。
- `nearfield_depth_frames.jsonl` 逐帧收据；`nearfield_renderer_pose_diagnostic.jsonl` 另存 renderer view，仅与同 stamp 前视相机和固定外参做诊断核对，绝不进入导航 NPZ/JSON。
- 4 帧队列上限保留 fail-closed：时间倒退、跨帧、校准不符、队列溢出拒绝，不丢帧后伪造新鲜度。

## 唯一 GPU owner 的首轮运行

复用已持久化的前台 interactive RTX 启动模板，capture 参数额外加 `--simulated-nearfield-depth`，并用12秒无 waypoint / 无 DSH dispatch 的零命令站立。保留 heartbeat、原生 observer、原生 perception/map，但**不启 depth fusion**。interactive 的合法 speed ceiling 可仍 .1，零命令来自未提交 waypoint，不能用非法 `--speed 0` 绕参数。

capture 命令参数（shell 环境/ROS domain/已有编译前缀由 GPU owner 模板负责）：

```bash
/home/hyh001/isaacsim/python.sh .dsh-subagents/rtx-prod-harness/capture_entry_default.py \
  --output "$RUN/capture" --point-cloud-source rtx_lidar \
  --interactive --interactive-pose-source livo_post_vio \
  --odometry-topic /dynanav/rtx_livo/post_vio_odometry --render-backend fabric \
  --speed .1 --duration 12 --interactive-perception-owner rtx_deskewed_cloud \
  --simulated-nearfield-depth
```

只读 CPU 审计（可在 GPU 完成后做）：

```bash
PYTHONPATH=src python3 -B scripts/audit_dynanav_nearfield_depth.py "$RUN/capture" \
  > "$RUN/nearfield_audit.json"
```

脚本复算所有点/掩码、校验 hash、检查原生单调 cadence、排队/归档耗时、前扇区9个10°子扇区的端点计数、两摄像头同 stamp 静态外参误差及实际 renderer 位移范围。**端点计数不判断地面/障碍**。floor support 要由严格源配对后的独立 floor 验证提供，不能凭该脚本宣布能走。全链实际 RTF/内存/零控制由同 run producer/physics 收据补足。

## 当前验证

新增18项CPU单测通过，RTX相关80项与capture orchestration22项通过；后者预期 failure injection traceback 不代表失败。py_compile、git diff --check通过。source snapshot / 全量检查以最后 freeze 后实跑为准。本实现worker未启动 Isaac/GPU/ROS，真实 coverage、mask、cadence、FSD跟随及算力尚待唯一 GPU owner 实测。
