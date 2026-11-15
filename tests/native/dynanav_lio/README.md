# DynaNav native LIO / post-VIO fixtures

本目录直接测试真实 C++ `LIVMapper` / `ImuProcess`，使用小型合成点云与 IMU fixture。需要 ROS Jazzy 的 `rclcpp` 和已编译的本仓库 `fast_livo2_core` 及公共依赖；不启动 Isaac、相机参数服务或外部传感发布进程。测试会创建本地 ROS node，不是无 ROS 依赖的 NumPy 测试。

## 2026-11-10 RTX Ouster 增量检查

新增 `test_ouster_mode.cpp` / `dynanav_ouster_native`。此前两个 fixture 不变，新构建实测 **3/3 通过，0.78 s**，使用独立 prefix `build_cache/dynanav_rtx_ouster_20261110T134330Z/install/ros`，没有覆盖旧 binary。

- 默认拒绝 Ouster；显式 `common.allow_ouster_lidar=true` 只允许 type=3，不解禁其他或未知类型。仍拒绝 GPS/HILTI、禁用 IMU/LiDAR、未 opt-in 的 image-free LIO；Ouster 必须完整 IMU、非 feature 模式、有效 scan_line/point_filter_num。
- 直接读取最小 PointCloud2 字段：float32 x/y/z/intensity、uint32 t(ns)、uint8 ring。缺失/错误 timing 或布局失败；丢弃非有限坐标/强度、越界 ring 和 blind returns。保留 scan-start header、每点 offset、强度和原始消息，排序后 ns→ms，不填造时间/点。
- 实际 serial ingress → preprocess → ONLY_LIO sync → IMU initialization/deskew。转动 fixture 覆盖首点 t=0、帧间隙、完整 IMU 等待、原始点所有权；解析旋转期望误差阈值 2e-5 m。
- 实际 Ouster+RGB `LIVMapper` 构造、订阅和 image-time scan slicing；本地明确 camera 参数，无外部参数服务。完整 IMU 前不消费证据，切片保留后半扫描。
- 仅新增 Ouster 路径校正初始化传播起点、ONLY_LIO 工作副本时间基准、LIVO 合并点时间顺序与首点边界。generic 和其他未开放类型的原 fixture 继续通过。

构建方法与下文相同，`CORE_PREFIX` 改为上述新 prefix。记录位于该 build 根目录的 `source-manifest.json`、`build-identity.json`、`test-result.txt` 和 `build/test/Testing/Temporary/LastTest.log`。补丁见 `third_party/fast_livo2/patches/0006-opt-in-timed-ouster.patch`。

这些是 native adapter/同步/IMU fixture，不证明 RTX 场景质量、视觉前端精度、地图漂移或导航闭环；真实 Isaac 运行由独立集成实验验证。下方 2/2 数字保留为此前 post-VIO 构建的历史记录。

## 目录结构

```text
tests/native/dynanav_lio/
├── CMakeLists.txt          # 注册 dynanav_lio_native 与 dynanav_post_vio_native 两个 CTest（29 行）
├── README.md               # 本文件：构建/运行命令、实测结果与边界
├── test_lio_mode.cpp       # ONLY_LIO/lifecycle/点时间校正断言（194 行）
└── test_post_vio_output.cpp # post-VIO 发布契约的本地 pub/sub fixture（137 行）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（测试） | [`test_lio_mode.cpp`](test_lio_mode.cpp) | 单进程 `main()` 内用 `require(bool, reason)` 断言：非零 `0xA5` poisoned storage 上 placement-construct/析构 `VIOManager`（要求 `visual_submap` 为 null、未经 camera 初始化即可析构）；默认配置拒绝未显式 opt-in 的 image-free LIO，允许模式不订阅图像/GPS、保留 cloud/IMU 输入；sparse cloud 的成对队列丢弃、buffer mutex 互斥及随后有效点/IMU 所有权不变；实际 IMU 非零加速度/角速度传播中 generic 瞬时点参考时间正确、原始点序/强度/offset 不被修改；非 generic 点云保持原有 timed offsets 与所有权路径。 |
| 代码（测试） | [`test_post_vio_output.cpp`](test_post_vio_output.cpp) | 本地 pub/sub fixture 注入“已结束估计”的状态，检查显式 opt-in、原 image stamp、map/child frame、同帧 status、no-op 的 false 标记、不可用 covariance/twist、重复/倒序/非有限/LIO-stage 拒绝，以及发布不改估计器状态；它不运行图像前端，不证明视觉精度或控制闭环。 |
| 代码（构建） | [`CMakeLists.txt`](CMakeLists.txt) | 两个 `add_executable` + 两个 `add_test`。只构建 fixture，不下载依赖、不重建 core；依赖由调用方通过 `CMAKE_PREFIX_PATH`/`LD_LIBRARY_PATH` 指定。 |
| 日志/输出 | 外部 `TEST_BUILD` 构建目录 | 不写仓库内文件。CTest 的 stdout/stderr 是唯一输出；成功判定为 `ctest` 退出码 0（2/2）。构建/运行日志落在 `PROJECT_DATA_ROOT/embodied-navigation/build_cache/native-lio-check-<UTC>/` 这类外部目录，不入 Git。 |
| 配置 | `CMakeLists.txt` 的调用参数 | 本目录没有 `configs/` 读取点；运行配置全部是 CMake 变量（`CMAKE_PREFIX_PATH`、`Python3_EXECUTABLE`）与环境变量（`LD_LIBRARY_PATH`、`PROJECT_DATA_ROOT`）。测试内部通过 `rclcpp::NodeOptions` 注入 ROS 参数，属于 fixture 内联配置，不是仓库 YAML。 |

测试夹具的输入（点云/IMU 序列）在 `.cpp` 内合成，不依赖 `configs/simulation/dynanav/*.yaml`；不要把本目录当成“验证该 YAML 生效”的手段。

## 模块内文件耦合

- 依赖方向：`CMakeLists.txt → test_lio_mode.cpp / test_post_vio_output.cpp`（一对一可执行文件，互不链接）；两个 `.cpp` 都只依赖已安装的 `fast_livo2_core`/ROS 头与各自 fixture，不互相 include。**无环**。
- 外部依赖是单向的“本目录 → 已安装 core/依赖 prefix”；本目录不修改、不重建 core。README 说明的 `CORE_PREFIX`/`DEPENDENCY_PREFIX` 与启动器 [`scripts/run_dynanav_lio.sh`](../../../scripts/run_dynanav_lio.sh) 的默认 prefix 相同，但两边各自硬编码默认值，改名时要同步。
- 隐式契约：断言依赖 native 代码的**字符串/结构**（例如 `VIOManager did not initialize visual_submap ownership` 的失败语义、`common.allow_lio_only`/`publish.post_vio_odometry_en` 参数键、`final_stage`/`lio_vio_skipped` 快照字段）；native 侧改名或改语义会让 fixture 编译失败或断言失效，而不是静默通过。

| 文件 | 行数 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `CMakeLists.txt` | 29 | CMake/ROS Jazzy | 调用方 `cmake` | 清楚：两个目标一一对应 |
| `test_lio_mode.cpp` | 194 | `fast_livo2_core`、`rclcpp`、`image_transport` | CTest `dynanav_lio_native` | 清楚：断言组与 README 一一对应 |
| `test_post_vio_output.cpp` | 137 | `rclcpp`、`fast_livo2_ros` 发布器 | CTest `dynanav_post_vio_native` | 清楚：发布契约单测 |
| `README.md` | 本文件 | 无 | 人 | 清楚 |

结论：耦合**清楚且合适**：两个 fixture 各自独立、失败信息直接指向被破坏的契约；唯一需要维护的是“README 命令/prefix 默认值”与启动器默认值两份字符串。

## 可读性评估

- `test_lio_mode.cpp`：**可接受**。194 行、7 行 `//` 注释、30 处 `require`/断言；`require(value, reason)` 把失败原因写成字符串，可读性好。缺口是 `main()` 里所有断言组串在一个函数里，没有 “断言组 → 行号/README 条目” 的对照表；`NodeOptions`/`imu()` 等 helper 命名清楚。
- `test_post_vio_output.cpp`：**可接受**。137 行、8 行注释、14 处断言；fixture 的 pub/sub 生命周期比前者短，仍是单 `main()`。
- `CMakeLists.txt`：**良好**。29 行、两个 target + 两个 test，无多余宏。
- 整体：没有魔法数滥用（30 帧/2.9 s/8 s 等数值对应 native 契约，README 有解释）；文档缺口是“每个断言组对应 native 哪条契约”只在 README 清单里，源码内没有编号。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 README 的断言清单前加“与 native 契约/补丁的对应表”（如 lifecycle ↔ `0004`、post-VIO ↔ `0005`） | 现在读者要在长文与源码间来回找 | 无（纯文档） | 人工对照 `third_party/fast_livo2/patches/0004*`、`0005*` |
| P2 | 把 `test_lio_mode.cpp` 的 `main()` 拆成命名断言组函数（`check_lifecycle()`、`check_image_free_optin()`…） | 单函数承载 5 组断言，失败只能从 reason 字符串定位 | 低：只重构测试内部结构，不改断言 | 重构后 `ctest` 2/2 仍通过，失败注入（临时改断言）确认 reason 仍可读 |
| P3 | 在 `CMakeLists.txt` 用 `option`/cache 变量暴露 `CORE_PREFIX` 默认，与启动器共享同一份默认前缀 | 现在 README、脚本、CMake 三处默认值需人工同步 | 低；只影响测试构建 | `cmake -S ... -B ...` 默认值下仍 2/2 通过 |

没有 P0/P1：fixture 不进入运行路径，最坏后果是测试编译/断言失败；结构与断言都直接对应 native 契约，不值得为“测试代码美观”做结构性改造。

## 对已安装的新 core 构建/运行 fixture

从仓库根目录运行。以下使用已验证的新 core prefix；fixture 写入另一个新外部构建目录，不覆盖原 core。

```sh
source /opt/ros/jazzy/setup.bash
export PROJECT_DATA_ROOT='/media/hyh001/新加卷/6.project_data'
CORE_PREFIX="$PROJECT_DATA_ROOT/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/install/ros"
DEPENDENCY_PREFIX="$PROJECT_DATA_ROOT/embodied-navigation/build_cache/native_validation_20261108T124116Z/project/install/ros"
TEST_BUILD="$PROJECT_DATA_ROOT/embodied-navigation/build_cache/native-lio-check-$(date -u +%Y%m%dT%H%M%SZ)"
export LD_LIBRARY_PATH="$CORE_PREFIX/lib:$DEPENDENCY_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cmake -S tests/native/dynanav_lio -B "$TEST_BUILD" \
  -DCMAKE_PREFIX_PATH="$CORE_PREFIX;$DEPENDENCY_PREFIX;/opt/ros/jazzy" \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$TEST_BUILD" --parallel 2
ctest --test-dir "$TEST_BUILD" --output-on-failure
```

`fast_livo2_core` 必须是已从本仓库源码构建并安装的相应版本；本目录只构建 fixture，不下载依赖或重建整个 core。其它机器可显式替换两个 prefix。完整原生依赖构建见 [BUILD.md](../../../docs/dependencies/BUILD.md)，估计器入口见 [LIO/LIVO README](../../../configs/simulation/dynanav/README_LIO.md)。

## 实测与旧 core 反例

当前 post-VIO core 的 [build-identity.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/build-identity.json) 和 [test-result.txt](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/test-result.txt) 记录 **2/2 CTest 通过，0.51 s**。它包括已修复的 LIO/lifecycle 和新 final-pose 发布契约，不是物理楼梯、视觉定位精度或完整导航验收。先前 lifetime prefix 的 1/1、0.28 s 是历史阶段结果，原二进制未被覆盖。

旧 offset core SHA256 `9f13c15ad9f326c55772066c12adc6378ddd463a96bff6b487c417c3c4913c51` 的构造器没有初始化 submap 指针；`ONLY_LIO` 又跳过 `initializeVIO`，析构会 delete 不确定指针。真实 gdb 曾定位析构 SIGSEGV，切换 DDS transport 没有消除根因。相同 poisoned-storage fixture 对旧 core 确定性失败，输出 `VIOManager did not initialize visual_submap ownership`，而不是让偶然清零的 allocator 隐藏错误。

证据见[原始生命周期诊断](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-slam-landing-sdk200/verification/native-lifecycle-diagnosis/lifecycle-diagnosis.json)及同目录反例日志。[0004 patch](../../../third_party/fast_livo2/patches/0004-safe-uninitialized-vio-lifecycle.patch) 仅建立 null ownership，正常 LIVO 仍按原初始化分配 submap，没有改变估计数学。旧 core/prefix 与旧运行哈希保留，未被新构建覆盖。

## 限制

- 本目录只证明 fixture 契约，不证明真实 sensor SLAM 的漂移、动态过滤或整条路线；`2/2` 通过也不是 `--check` 的替代。
- 需要本机构建产物与 ROS Jazzy；未构建 core 时测试无法编译，不能把“跑不了”当成通过。
- 本轮文档任务未重新执行 CTest；上述 2/2、0.51 s 来自原 README 记录的外部 `test-result.txt`，本轮未复核该文件当前内容。

## 2026-11-10 同时间 pair 阶段交接

独立构建 `build_cache/dynanav_same_pair_20261110T171010Z/install/ros` 的 **5/5 CTest 通过，1.29 s**。原三个 fixture 保留；新增 observation fixture 和 `test_same_pair_handoff.cpp`。后者直接执行真实 `Process2/handleLIO/handleVIO`，验证：

- 初始化最后一次返回即使清除了 `imu_need_init` 也必须 skip；故意放入旧 deskew/发布点不能恢复旧 pair。正常 LIO 完成后 raw/IMU 新队列同时为空，同目标 VIO 仍可选择。
- 仅 selected flag 或仅 IMU 处理完成不够；错图像时间、错原 stamp 1 ns、换图对象、缺原 stamp、空点、停止/异常不允许 VIO。每 pair 一次消费/skip，VIO 未结束不选择下一 LIO。
- VIO 的零时长 Process2 后 state/cov 与 LIO posterior 逐元素相等，state_propagat 等于该 posterior；last IMU 样本及传播终点、当前/下一切片和原点时间不变。未跳过整段 VIO IMU 准备。
- 同 binary `common.same_pair_vio_handoff=false/true` 的正常结果 state/cov/prior 逐元素相同；false 保留旧初始化、raw 空和 IMU 空门。默认配置/ONLY_LIO 原测试仍通过。
- 小容量 image/primary-IMU/prop-IMU 测试恰在界限拒绝新增样本，不丢旧数据、不部分更新 timestamp、mutex 不泄漏；缺省三个限制仍为0。生产RTX配置明确32/8192/8192。
- observation fixture 仍验证前段 point coverage 与 IMU 端点不足时等待且不消费。两个新增 fixture 的观测 OFF/ON 签名相同，ON 连续记录分别 7/91 行，OFF 不创建文件。

记录：build root 的 `test-result.txt`、`artifact/observation-result.json`、`artifact/*-on.jsonl`、`source-manifest.json` 和 `build-identity.json`。补丁为 `0007-same-pair-vio-handoff.patch`，保留 `0006` 和旧 prefixes。合成 native fixtures 不证明 RTX 运动精度、真实延迟改善或完整导航闭环；本构建未运行 Isaac/GPU。详见 [实现与边界](../../../docs/experiments/dynanav-same-pair-handoff-20261110.md)。

## Explicit complete-scan CPU fixture

`test_complete_scan.cpp` adds typed certificate/continuity/identity/phase faults,
owner-only failure persistence, pending/raw gates, early-image cold startup and
zero-return/fully-filtered recovery. It compares real sync/Process2/LIO/VIO state,
covariance, source points/float curvature and IMU lists with the switch off/on.
`COMPLETE_SCAN_REPLAY_BIN` additionally consumes an SHA-verified stored source
fragment and starts from a fresh initial state with blank images, not the original
full trajectory/map/VIO.
See the dedicated complete-scan experiment document and isolated build manifests.
