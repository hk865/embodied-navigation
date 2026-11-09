# DynaNav native LIO / post-VIO fixtures

本目录直接测试真实 C++ `LIVMapper` / `ImuProcess`，使用小型合成点云与 IMU fixture。需要 ROS Jazzy 的 `rclcpp` 和已编译的本仓库 `fast_livo2_core` 及公共依赖；不启动 Isaac、相机参数服务或外部传感发布进程。测试会创建本地 ROS node，不是无 ROS 依赖的 NumPy 测试。

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
