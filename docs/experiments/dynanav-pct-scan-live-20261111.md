# DynaNav：PCT + SCAN 单个 4 米目标实时核验

本次只验证静态办公室内一个约 4 米目标的规划、连续曲线跟随、到点与正常停车。
不代表楼梯、多楼层、复杂障碍避让、全覆盖或实机验收。没有 commit/push。

## 已完成的实际结果

新代 run（UTC 命名）：`pct_scan_live_validated_20261111T182343Z`，位于
`PROJECT_DATA_ROOT/embodied-navigation/experiments/`。ROS domain 223，有限预算
180 模拟秒，命令上限 1 m/s。使用当前 launcher 默认 `pct_scan` 后端、默认
1.5 m 起点盲区模型和默认 IMU-origin 支撑偏移；只把当前源码 SCAN 构建前缀
显式指定为 `/tmp/dynanav-scan-native/install`。

- 1 个 waypoint accepted / 1 reached / 0 failed。
- 本代 post-VIO 目标 `[3.9897002101, -0.4532674435, 0.0081657541]`。
- 最终实测停止位置 `[3.7244811282, -0.5111081694, 0.0182777116]`。
- XY 距离 0.27145 m、z 误差 0.01011 m，满足保留的 0.30 m 最终到点范围和
  0.18 m 高度范围；不是精确走到目标坐标。
- 首条原生 SCAN 参考在 61.415 模拟秒被消费；最终测量停止 dwell 在 91.1 秒完成。
- 共消费 27 条真实 SCAN 运动参考。样条采样点未触发中间 capture/brake/turn。
  目标执行区间记录 1440 个 cruise tick、44 个最终 brake tick、1 个 planning tick。
  连续曲线模式不等于全程不间断前进，朝向纠正和输入复用仍存在。
- post-VIO 估计净位移 3.7450 m、估计路径长度 5.1425 m。路径长度不等于覆盖。
- 请求上限 1 m/s；离线物理水平速度均值 0.1580 m/s、p95 0.7112 m/s、峰值
  0.9845 m/s。估计器差分速度均值 0.1799、p95 0.7353、峰值 1.0388 m/s，
  不能把估计噪声、命令上限和物理速度混为一谈。
- 本次记录 body-contact steps = 0；没有以禁用姿态/速度包络断言代替物理证据。
- stop seq2 被确认，`end_reason=navigation_terminal_and_parked`，capture exit0。
  最后约 1 秒最大水平速度 0.000175 m/s、最大角速度 0.002686 rad/s。
  四个 owned PID 和原生 SCAN 子进程均已确认不存在。

## 选点与输入来源

实际查看本代 front RGB、当前实测云的 full/local XY、XZ、YZ、斜视 3D，以及
operator map/高度组导出。目标从本代 post-VIO 当前位置和测得 yaw 前向移动 4 m
计算；可见前方地面和本代测量范围支持这次测试选择。未读取 USD/原始场景几何、
仿真真值地图、旧代坐标或旧路线来选点。实际 body/真值日志只在结束后用于评分。

run 的 `live_review/` 保存 RGB/cloud 决策引用、冻结实测云、图和 `decision.json`。
`capture/interactive/planner_measured_map.npz` 中的云仍为 LIO map 坐标，未再乘 pose。
它保留逐点来源 stamp、精确配对 LIO pose、测量区间和 downsampling 来源。
原 full 显示的 stride 可能漏画极值返回；事后补充 `full_domain_verified.png`，强制保留
XYZ 极值并按完整输入范围设轴，未删除约 x=-60.8 m 的孤立返回。局部选点图与保留
测量输入未改，不能把显示裁剪当成自由空间或支撑证据。

PCT 计划使用 80,394 个累计实测 voxel 代表点，12,257 个实测支撑格。起点附近
RTX 地面盲区由 surrounding measured plane 明确推断 191 个起点连通格；原实测格
未改。平面来自 196 个实测支撑格、194 inliers，p90 残差 2.52 mm、坡度 0.00257。
这些推断格明确记录 `observed_free=false`、`support_certified=false`，不能排除不可见
坑洞。不存在其它未知区域通用填洞或直线 fallback，见[接入说明](../dynanav-pct-scan.md)。

## 失败先例与恢复

首个真实 run `pct_scan_integration_20261111T180723Z` 接收 4 米目标后报告
`planner_start_support_unobserved`，未输出运动命令。随后 stop/parking/exit0 与清理
均完成。原因是原严格点支撑图无法跨过 RTX 近场测不到地面的起点洞。

新增可关闭的起点局部平面延续模型后，先对这个已冻结真实点云作原生 PCT 核验：
27 点路线成功，10 项真实原生 PCT 测试通过。随后才启动本次新代重新观察与选点，
没有把旧代坐标作为新代导航输入，也没有声称 checkpoint continuation。

更早两次启动没有进入 Isaac：一次 Fast DDS domain 超出默认端口映射范围，一次
命令 sandbox 禁止网络接口枚举、已有 camera-parameter service 未就绪。修正 domain
校验并按授权使用可用本地 DDS 执行环境后启动成功，均未绕过访问拒绝读取场景数据。

## 测试与事实边界

- 已补回 worker 的 unittest 发现入口，包含 malformed goal、active seq/cancel、原生
  knot-vector 曲线采样与无效样条测试。目标相关 27 项 CPU 测试通过。
- 完整 unit：1102 tests，3 skipped；regression：108 tests，8 skipped。
  skips 保留原环境相关语义，不算通过。
- 真正当前源码 PCT → 原生 SCAN 的独立 ROS synthetic smoke 得到多次重规划。
  注入合成测量障碍后收到匹配 metadata 的 emergency hold；cancel/TERM 关闭 owned
  子进程且无孤儿。该 synthetic 域为 213，与实时域 223 分离，不能充作真实地图证据。
- 当前 native log 记录 27 次 `final_plan_success=1` 和 78 次候选拒绝。末端多次候选
  因加速度可行性超限被原生 SCAN 拒绝；没有把候选拒绝当成已执行轨迹或目标失败。
  当前 follower 保持已消费有效参考并完成最终测量停止。
- PCT 仍为 point-footprint 全局图，有上游 diagonal corner cutting 等限制；局部 SCAN
  采用原生 3D inflated occupancy，但 z 按局部线性参考保留、优化器 z gradient 为零，
  不是任意高度避障或最大坡度认证。
- adapter 做 exact-stamp LIO cloud/pose 配对，native GridMap 内部仍使用 latest pose，
  没有 native 精确消费 ACK。轨迹 `source_stamp_s` 只是 adapter 当时收到的最新 post-VIO。
- 原生动态碰撞 guard 按样条时间检查，开始时检查前 2/3；实际空间 follower 进度不等于
  原生时间进度。此接入没有认证“整条 follower 路径持续全覆盖检查”。
- 当前 capture 的历史末尾硬编码 `scan_connected=false` 会遮蔽真实消费证据。本次原始
  result 保留不改；真实来源由 27 个 `native_scan_reference`、原生 metadata/PID/hash 和
  原生日志交叉确认。后续源码已改为按真实消费事件生成该诊断值，不新增控制门禁。
- 航点信息增益/旅行代价优化尚未实施，下一阶段不能以本次单目标到点替代。

## 可核对的产物

- run `live_review/validation_summary.json`：结构化实测指标及终止/清理结果。
- `capture/interactive/receipts.jsonl`：accepted、reached、stop_accepted。
- `capture/interactive_waypoint_control.jsonl`、`capture/result.json`：测量与执行事件。
- `capture/interactive/planner_path.json`、`scan_1.log`、`scan_parameters_1.yaml`：原生参考、
  exact reference stamp、trajectory metadata 和当前源 hash。
- `capture/source_snapshot/`、`source_manifest.json`：运行所用源码快照，不用之后代码
  替代本次事实。
- `capture/interactive_frames/`、`camera_frames.jsonl`：本代实际 front 图像与来源时间。
- `capture/truth_scoring_stream.jsonl`、`policy_frames.jsonl`：明确隔离的离线物理评分。
- `capture/multiwaypoint05_overview_and_rgb.mp4`：保留视频，名字沿用现有捕获器。
- `capture_exit.txt`、`lifecycle.log`、`owned_pids.txt`：退出与进程归属。

本报告保留边界与失败记录，不把单个静态平面目标成功扩展成楼梯/全楼导航能力。
