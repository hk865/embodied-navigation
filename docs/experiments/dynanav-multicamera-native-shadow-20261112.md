# 方案 A：真实 native 快照、曝光分离与六路旁路视觉修正

已完成一次 20 仿真秒的真实 Isaac 六相机采集，导出同代 native LIO 状态与 VIO 查询几何，并在冻结数据上调用原生 FAST-LIVO2 求解器运行六个视觉分支。两种曝光设置下，各相机都在参考缓存建立后的 **14/14 帧保留了视觉数值更新**。

这是**真实传感器数据的离线旁路对照**，不是实时六路融合或运动精度验收。正式定位仍由原单前相机链路产生，没有回写任何旁路或 CI 结果；没有发送导航目标，未修改驱动、安全配置或原缓存二进制，未提交或推送 Git。

## native 快照接口

新增 `visual_snapshot.h/.cpp`，在 `LIVMapper::handleLIO` 完成当前状态与地图更新后、同一对 VIO 之前调用。未设置 `FASTLIVO_VISUAL_SNAPSHOT_DIR` 时立即返回。显式设置后，仅为已有 camera_0 原始归档的源时刻导出数据。

CBOR 快照保存：

- 原始图像源整数纳秒和 native 浮点目标转换所得整数纳秒，两者分开记录；generation 使用完整 capture 路径，避免仅叫 `capture` 的目录名碰撞。
- 原生旋转、位置、速度、bias、gravity、前相机逆曝光，以及**完整原生 19×19 内部协方差**。没有读取 ROS unavailable pose covariance。
- 当前 VIO 输入点的世界位置、原生法向、原生世界点协方差。
- 原生 voxel root key、octree 层级、分支、中心、quarter-length、初始化状态，以及 VIO 实际查询的平面中心、法向、6×6 平面协方差、距离、半径和支持点数。

这是精确的 **VIO 只读查询快照**，不是可继续更新的 LIO map checkpoint：没有序列化 LIO 更新所需的原始临时点池/计数，也不复制原前相机的私有视觉 patch 历史。各相机从自己的真实图像建立并保留自己的 patch 缓存。

快照在 owner 的稳定 LIO 阶段复制，CBOR 中不含指针。离线解码分配独立 octree，保留 `find_correspond` 的实际查询结构；每轮六分支执行前后将状态/点/平面树重新编码，验证数据完全一致。相机分支只串行调用，未声称原生全局 Frame counter 或 pointer-valued map 已支持并发。

接口有点数、节点数、深度和文件字节预算，原子重命名完成文件；不会覆盖同名已完成快照。导出异常记入 `[VISUAL_SNAPSHOT_ERROR]`，不替换数据或改写估计状态。本轮无此异常。当前导出编码与写盘在 owner 内同步执行，是显式诊断模式；尚未做异步队列或实时采集优化。

## 公共物理状态与曝光分离

原生 19 维顺序为 `[rotation(3), position(3), exposure(1), velocity(3), gyro_bias(3), acc_bias(3), gravity(3)]`。公共物理状态是去掉 index 6 后的 18 维。

实现通过索引选择取得原生 **18×18 边缘协方差 P_xx**，保留所有物理状态之间的交叉项。它没有使用 `P_xx - P_xe P_ee^-1 P_ex`，后者是“已知曝光”的条件协方差，会不当地减少公共物理不确定性。

每个相机、每张图再引入独立的曝光 nuisance。对照设置为：

1. `fixed`：固定单位曝光，关闭曝光 Jacobian。
2. `exposure`：每帧独立逆曝光 `e_c,t ~ N(1,0.1)`，开启原生曝光估计；使用每相机自己的参考 patch 曝光值。

第二种是明确声明的实验模型，**0.1 是 nuisance 模型先验方差，不是硬件实测方差，也不是 ROS 占位值**。不把前相机曝光均值/方差/交叉项转给其他相机，不把各相机曝光融合成一个公共变量。每次联合视觉更新后，再对曝光做边缘化，输出物理 18×18 后验及其中的 pose marginal。未实现带时间相关的每相机曝光 random walk 或跨时刻的完整 nuisance cross-covariance。

真实 15 份原生先验全部数值 SPD，其物理-前曝光交叉项范数在 `1.91e-8..7.97e-8`，并非全部为零。最后时刻前曝光均值 0.9977188、方差 `5.0915e-5`；代码正确取得物理边缘分布后才构造新的相机 nuisance。合成单元测试另外用较强交叉项验证，防止这个区别被真实数据中较小交叉项掩盖。

## 真实运行与来源

run：`/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/multicamera_native_shadow_20261112/snapshot_20s`，ROS domain 223。沿用原 Office 场景、原 spawn、yaw 0、200Hz IMU、RTX OS1 complete scan、512×512 六向相机。机器人执行站立策略，4,000 物理步中 1,000 个导航控制 tick 的命令均为零。

使用隔离目录中的新核心，原缓存 runner 逐字节复制；仅重编译含导出调用的 `LIVMapper.cpp` 和新导出模块。其余旧对象的对应源码、原有头文件及对象校验和逐一验证后复用，没有改动原缓存对象或二进制。仓库 CMake 也已加入新源文件，常规构建可重建全部源码。

- 新核心 SHA256：`3eb11213b6862025ef158181c1ae0eec66efa48c5f7b922f84cdfde45dedc1fc`。
- `/proc/<mapper>/maps` 确认运行时加载该隔离库，不只是检查构建文件存在。
- capture exit 0，`outcome=completed`；发布 IMU 4,000、RGB 199、RTX/complete scan 各 198。
- 每台相机接受 199 个原生绑定帧，归档 20 对 RGB/深度，无无效绑定帧、采集错误或未清空 pending frame。
- 原生快照 15 份，共 38,524,172 字节，源时间 **5.2..19.3s**。早期图像没有成熟 LIO 对应，因此未伪造快照。
- 每份包含 1,814..2,364 个 octree 节点。最后一份 7,556 个实测点，7,260 个具有非零原生法向，2,267 个 root voxel / 2,364 个 octree 节点。
- 六相机源时间偏差、图像与 native 目标整数纳秒偏差在本数据中均为 **0ns**。没有使用最近位姿替代或重写源时间。
- 主循环墙时 46.2863s（20/墙时约 0.4321）；整卡约一秒采样峰值 6,753MiB，包含桌面。主循环没有运行六个 shadow solver，因此这不是实时六视觉分支性能数据。

已有正式单前相机仍正常运行；`handleLIO` 只读导出没有接入旁路结果。快照中的物理先验可能包含正式前相机此前帧的信息，六分支因此仍相关，不能当独立估计器输出。

## 相同冻结数据上的对照

各分支使用原生 VIO 的 normal 开启、四层 pyramid、原相机内外参及私有 patch 历史。六路与独立单前相机 control 都从相同 native 快照和相同归档图像运行；没有新增 IMU 积分或各跑一份 LIO。

每路处理 15 个源时刻。5.2s 首帧仅建立参考缓存，后续 14 帧均为 retained numerical visual update。下表为跨 15 帧累计 tracked point-frame 实例（同一物理点多次跟踪会重复计数，不是独立场景点）：

| 相机方向 | 固定曝光 | 每帧独立曝光 | 两模式有效更新帧 |
| --- | ---: | ---: | ---: |
| 0° | 1,458 | 1,458 | 14 / 14 |
| 60° | 1,262 | 1,261 | 14 / 14 |
| 120° | 763 | 762 | 14 / 14 |
| 180° | 308 | 308 | 14 / 14 |
| 240° | 542 | 531 | 14 / 14 |
| 300° | 1,346 | 1,347 | 14 / 14 |

后向比前向少很多观测，但并非仅输出了图像而没有视觉求解。独立曝光下，各相机逆曝光估计范围分别约为 0.99692..1、0.99612..1、0.99335..1、0.99453..1、0.99539..1、0.99629..1；它们是各自的 nuisance，没有共享一个曝光后验。

为避免只用布尔标志或 tracked-point 数判断，最后 **19.3s** 开启原生已有的细节日志。相机分别绑定 stage 100..105，control 为 200。下表取最细 level 0 的最后一个 accepted iteration：

| 方向 | 固定曝光有效像素项 | 独立曝光有效像素项 | 固定曝光 photometric MSE | 独立曝光 photometric MSE |
| --- | ---: | ---: | ---: | ---: |
| 0° | 7,744 | 7,744 | 12.4121 | 12.3619 |
| 60° | 6,656 | 6,656 | 10.8183 | 10.7483 |
| 120° | 4,480 | 4,480 | 8.8183 | 8.6170 |
| 180° | 1,280 | 1,280 | 6.1711 | 6.0021 |
| 240° | 2,816 | 2,752 | 3.6117 | 3.5355 |
| 300° | 7,104 | 7,104 | 5.2720 | 5.2582 |

这里的 MSE 单位是灰度强度平方，不是米制定位误差；240° 两模式的保留点集也有不同，不能把差值当成严格相同样本的精度提升。像素项之间存在 patch/空间/历史相关，不是独立观测数。

两模式的独立单前相机 control 与六路中的前相机，pose increment 和完整物理 18D covariance 的最大差异均为 **0**。每一时刻六路执行前后的源状态、输入点和全部 VIO-query plane tree 编码也完全相同。

原生细节日志固定曝光写出 960/960 记录、独立曝光 959/959 记录，均无队列丢弃或 I/O 错误。对同一数据另保留未开启细节日志的结果；观察开/关的 pose increment 与物理协方差差异均为 0。

15 个时刻中六个 solver 的累计调用墙时约为固定曝光 0.278s、独立曝光 0.306s。这只包含离线调用，排除了快照解码/编码、I/O、进程启动、ROS 和渲染，并包含最后时刻的诊断；不能据此外推完整系统实时率。

## 协方差与 CI 输出的边界

物理先验来自实际 native 19D 内部状态；分支输出来自实际联合视觉更新后对曝光的边缘化。每模式 84 个 retained 分支结果的物理 18×18 marginal 均为数值 SPD，没有正则化填补或使用 unavailable covariance。

但数值 SPD **不等于统计可信**。native 模型 pose 标准差很小（位置约 0.14..0.73mm、姿态约 `1.96e-5..2.95e-5 rad`），本轮没有实测误差/NEES/覆盖率证据来证明这些数值。地图、历史 patch 和共享先验带来的未建模相关仍在。

审计对每个文件绑定实际 SHA256、generation、原始源时间和相机标定。两个模式各生成 14 份六路 CI 诊断，首帧没有视觉更新时不生成融合测量。pose covariance 先变换至共同先验的右扰动 Log 坐标图，再按前述 CI 方法计算；未重复把 LIO prior 加作独立观测。

所有结果明确为 `confidence_calibrated=false`、`estimator_feedback_enabled=false`。CI 只能在输入协方差满足相应一致性前提下处理未知交叉相关，不能使未经校准的原生模型自动成为真实误差界；详见前一份[分支设计报告](dynanav-multicamera-shadow-design-20261112.md)。这里也未把六维 CI 增量写回完整物理状态、地图或 patch 历史。

## 测试、源码与复现

- 新分支隔离与快照/曝光单元测试：2/2 native 通过。
- 使用本次隔离核心运行原有 LIO、post-VIO、Ouster、同步、same-pair、complete scan 回归：6/6 通过。
- 融合接口测试 6/6、探索入口测试 12/12 通过。
- `verify_native_sources.py` 校验 312 个来源记录通过；新增 `0009-read-only-visual-state-snapshot.patch` 的反向 dry-run 通过。
- 前一阶段六相机采集/投影实现未修改。当前增量只追加快照模块与调用、旁路曝光分离/回放/审计及对应测试与记录。

原生工具现在可由仓库 `tests/native/dynanav_lio` CMake 构建 `replay_multicamera_shadow`。在正确 native/dependency prefix 环境下，对本次冻结数据：

```sh
RUN=/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/multicamera_native_shadow_20261112/snapshot_20s
replay_multicamera_shadow "$RUN/capture" "$RUN/native_snapshots" "$NEW_FIXED_JSONL" fixed
replay_multicamera_shadow "$RUN/capture" "$RUN/native_snapshots" "$NEW_EXPOSURE_JSONL" exposure
python3 -B scripts/dynanav_exploration.py shadow-audit \
  --capture "$RUN/capture" --snapshots "$RUN/native_snapshots" \
  --replay "$NEW_FIXED_JSONL" --replay "$NEW_EXPOSURE_JSONL" --output "$NEW_AUDIT_DIR"
```

新的实际采集需要先使用包含本次模块的新 core，并在启动前显式设置：

```sh
export FASTLIVO_VISUAL_SNAPSHOT_DIR="$NEW_RUN/native_snapshots"
export FASTLIVO_VISUAL_SNAPSHOT_GENERATION="$NEW_RUN/capture"
export FASTLIVO_VISUAL_SNAPSHOT_CAMERA_ARCHIVE="$NEW_RUN/capture/multicamera"
MULTICAMERA_OBSERVATION=1 NAVIGATION_BACKEND=native_route DURATION_SIM_S=20 \
  BUILD_PREFIX="$NEW_NATIVE_PREFIX" ROS_DOMAIN_ID="$UNUSED_DOMAIN" \
  bash scripts/run_dynanav_native_exploration.sh "$NEW_RUN"
```

`NEW_RUN` 应为完整绝对路径且不存在；该启动器同时保留原项目数据根目录约束。普通未设置上述变量的运行不会启用导出。归档约 1Hz、原相机约 10Hz；本次没有伪造丢掉的时刻，也没有把 1Hz 回放当 10Hz 在线分支。

本轮证据：`/home/hyh001/Documents/Codex/2026-11-12/task/multicamera_real`。

- `real_validation_summary.json`、`native_prior_inspection.json`：实际 run、曝光交叉项、节点/点数、像素测量、观测开关对照及资源。
- `audit/audit.json`、`source_integrity.json`、`ci_diagnostics.jsonl`：每相机结果、源文件 SHA256 与只读 CI。
- `fixed.jsonl` / `exposure.jsonl` 及 `*_detail.jsonl`：完整原生分支结果；`fixed_detail/`、`exposure_detail/` 为原始 diagnostics。
- `isolated_build_manifest.json`、`actual_loaded_core.txt`、native 构建/回归/来源校验日志。
- `six_views/six_views.png`：已实际打开检查的本次 19.3s 六向原图。
- 原 run 的 `native_snapshots/*.cbor`、`capture/multicamera/`、`capture/result.json` 和原始 observer/capture 日志均保留。

当前无权限阻塞，所有本次仿真/估计器进程已退出。剩余工作是移动场景、10Hz 实时旁路调度与资源测试、时间相关曝光模型、真实协方差一致性，以及回写完整物理状态和地图历史之前的设计验证。本轮不声称这些已完成。
