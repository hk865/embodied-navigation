# DynaNav 估计器配置

本目录保存办公室 RGB-D/IMU 估计器的显式实验配置，不包含场景资产、策略权重、日志或地图。

## 目录结构

```text
configs/simulation/dynanav/
├── README.md                              # 本文件：配置索引、消费点与边界
├── README_LIO.md                          # LIO/LIVO 观察器长文说明与实验记录（354 行）
├── camera_rgbd.yaml                       # 512×512 pinhole 内参基准（18 行）
├── fastlivo_lio.yaml                      # ONLY_LIO 模式参数（100 行）
├── fastlivo_lio_gravity_estimation.yaml   # 单变量 gravity 对照覆盖（6 行）
├── fastlivo_livo.yaml                     # RGB 辅助 LIVO 模式参数（129 行）
└── office.yaml                            # 历史场景/资产路径记录，当前无代码读取（5 行）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 配置 | [`fastlivo_lio.yaml`](fastlivo_lio.yaml) | `--mode lio` 的参数文件：`common.img_en: 0`、`common.allow_lio_only: true`，输入 `/dynanav/lidar_static` 与 `/dynanav/imu`，`publish.post_vio_odometry_en` 为关闭。由 [`scripts/run_dynanav_lio.sh`](../../../scripts/run_dynanav_lio.sh) 拼成 `fastlivo_${mode}.yaml` 后交给 native runner；preflight 会校验“模式与 post-VIO 开关一致”。 |
| 配置 | [`fastlivo_livo.yaml`](fastlivo_livo.yaml) | `--mode livo` 的参数文件：`common.img_en: 1`、`allow_lio_only: false`，增加 `/dynanav/camera`；`publish.post_vio_odometry_en: true`。视觉参数（`img_point_cov=100`、`outlier_threshold=100`、曝光估计开启等）是本轮实验设置，不是精度验收结论。 |
| 配置 | [`camera_rgbd.yaml`](camera_rgbd.yaml) | 当前 capture 的 512×512 pinhole 内参（`fx=fy=512×24/42`、`cx=cy=256`、`scale=1`）。只在 `--mode livo` 下作为相机参数服务的 params 文件与基线；脚本随后用 `--capture-result` 的 `sensor_installation` 覆盖安装标定。`--mode lio` 不使用该文件。 |
| 配置 | [`fastlivo_lio_gravity_estimation.yaml`](fastlivo_lio_gravity_estimation.yaml) | 单变量覆盖文件，只把 `imu.gravity_est_en` 改为 `true`，通过 `--parameter-override` 追加到 base 参数之后；默认 YAML 仍为 `false`，已有对照未显示改善。 |
| 配置（无读取点） | [`office.yaml`](office.yaml) | 记录 `scene_relative`、`robot_asset_relative`、`physics_dt_s`、`policy_dt_s` 与 `verified_local_isaac_build`。`grep -rn "office.yaml" src scripts tests` 无代码读取；实际场景/机器人路径硬编码在 [`simulation/dynanav/experiment.py`](../../../src/embodied_navigation/simulation/dynanav/experiment.py) 的 `data_root()/dynanav/.../office.usd`，因此本文件是历史记录，不是生效配置。 |
| 代码 | 无 | 本目录没有 Python/C++ 代码；唯一读取点是 shell 启动器 `scripts/run_dynanav_lio.sh`（参数文件 + 相机文件 + 可选 override）和它的回归测试 [`tests/regression/test_dynanav_lio_launcher.py`](../../../tests/regression/test_dynanav_lio_launcher.py)。 |
| 日志/输出 | 外部 experiments 目录 | 本目录不写日志。运行产物（`preflight.json`、`launch.json`、`capture_calibration.yaml`、`camera_parameter_ready.json`、`camera_parameters.log`、`runner.log`、native diagnostics）都写在 `--output-dir` 指定的外部 run 目录；全局布局见 [configs/README.md](../../README.md) 与 [readability 报告 §9](../../../docs/review/dsh-workflow-and-readability-20261109.md)。 |

`office.yaml` 之外的四个 YAML 都需要显式 `--mode`/`--parameter-override` 选择，没有“启动即自动读取本目录所有 YAML”的机制；改配置后“没变化”时先确认模式与启动参数。

## 模块内文件耦合

- 依赖方向：`README.md → README_LIO.md` 与四个 YAML 是文档索引关系，无 import。运行期只有 shell 启动器按下述规则选文件，YAML 之间不互相 include；`--parameter-override` 用 ROS 2 参数文件“追加”语义叠加，而不是文本继承。
- 选择规则（`scripts/run_dynanav_lio.sh`）：`parameters="$repo_dir/configs/simulation/dynanav/fastlivo_${mode}.yaml"`；`office.yaml` 不在任何选择分支；`camera_rgbd.yaml` 只有 livo 分支读取。
- 隐式契约：`fastlivo_lio.yaml` 的 `publish.post_vio_odometry_en=false` 与 `fastlivo_livo.yaml` 的 `true` 必须分别匹配 `--mode lio/livo`，否则 preflight 直接失败（`Observer mode and explicit post-VIO output configuration disagree`）；`camera_rgbd.yaml` 的字段名必须与相机参数服务/node 的期望参数一致，否则 `--mode livo` 的标定校验会拒绝。

| 文件 | 行数 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `fastlivo_lio.yaml` | 100 | native `fast_livo2_ros` 参数契约 | `run_dynanav_lio.sh --mode lio`、回归测试 | 清楚：模式单一、输入固定 |
| `fastlivo_livo.yaml` | 129 | 同上 + `/dynanav/camera` + post-VIO 输出 | `run_dynanav_lio.sh --mode livo`、回归测试 | 清楚，但实验参数多，需 README_LIO 解释 |
| `camera_rgbd.yaml` | 18 | 相机参数服务字段名 | livo preflight / 参数服务 | 偏脆弱：字段名与 native 服务是字符串契约 |
| `fastlivo_lio_gravity_estimation.yaml` | 6 | 覆盖 base 的 `imu.gravity_est_en` | 人工 `--parameter-override` | 清楚：单变量 |
| `office.yaml` | 5 | 无 | 无代码读取点 | 可疑（孤儿配置）：易被误当成 DynaNav 场景注册表 |

结论：配置之间**没有循环或隐藏继承**，耦合主要是“YAML 键 ↔ native 参数契约”的字符串耦合，属于可接受范围；唯一明确问题是 `office.yaml` 无读取点却看起来像生效配置。

## 可读性评估

- `fastlivo_lio.yaml` / `fastlivo_livo.yaml`：**可接受**。两者都有 3 行头部注释说明模式、输入与“不是精度声明”的边界；参数按 `common/lidar/imu/publish/...` 分组，键名可读。缺口是没有注明哪些键会被启动器覆盖（如 `camera` 相关、post-VIO 开关），只能到 shell 脚本里找。
- `camera_rgbd.yaml`：**良好**（相对体量）。3 行注释直接给出 512×512、焦距公式、中心与“LIO 模式不加载相机模型”的边界，避免把内参当成 LIO 输入。
- `fastlivo_lio_gravity_estimation.yaml`：**良好**。6 行内写清单变量目的与“不作精度声明”。
- `office.yaml`：**需改进**。5 行、0 注释、无读取点；字段名像运行配置，实际是历史路径记录（与 `experiment.py` 硬编码重复）。这是文档/语义问题，不是结构问题。
- `README_LIO.md`：**需改进（可读性）**。原 294 行把启动命令、标定契约、post-VIO 评分、生命周期修复和实验哈希混在一条线性叙述里；本次已加 5 节索引（当前 354 行）；信息真实但缺少目录/分节入口，新读者难以定位“当前默认是什么”。见下节 P2。

没有发现魔法数滥用：数值均来自实验配置且 README_LIO 有对应解释。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `office.yaml` 顶部加 3 行注释，明确“历史路径记录，无代码读取点，实际路径在 `experiment.py` 的 `data_root()`”，或移到 `docs/` 归档 | 现在它和生效配置同目录同格式，最容易被误读 | 无（注释/移动不涉代码） | `grep -rn "office.yaml" src scripts tests` 仍为空；文档链接指向新位置 |
| P2 | 在 `README.md`（本文件）补“配置键 → 覆盖者”小表：哪些键被启动器/preflight/`--capture-result` 覆盖 | 读 YAML 无法知道哪些值不是本次有效值 | 无（纯文档） | 对照 `scripts/run_dynanav_lio.sh` 的 preflight 字段清单 |
| P2 | 给 `README_LIO.md` 加“当前默认 / 历史对比 / 生命周期记录”三类分节标题与文首目录 | 原 294 行线性叙述，当前默认被历史证据淹没（本次已加索引，正文未重排） | 无（只加标题/目录，不改正文事实） | 人工按目录跳转一遍；事实段落原样保留 |
| P3 | 统一 `camera_rgbd.yaml` 与 `--capture-result` 的标定来源说明（哪些字段来自文件、哪些被覆盖），并在两个 README 用同一措辞 | 现有说明分散在 README_LIO 与脚本注释里 | 无 | 对照脚本第 151–170 行的覆盖字段 |

没有 P0/P1：本目录无代码、无写操作；最坏后果是选错模式或误读实验参数，已有 preflight 失败保护，不需要结构调整。

## 配置清单与入口（原有内容）

- [fastlivo_lio.yaml](fastlivo_lio.yaml)：默认图像关闭的 `ONLY_LIO`，输入静态 RGB-D 点云和 IMU；须显式允许 image-free 模式。
- [fastlivo_livo.yaml](fastlivo_livo.yaml)：RGB 辅助 LIVO，增加 `/dynanav/camera`，禁止回退成 image-free LIO。启用最终 post-VIO pose/status 输出；视觉参数是本轮实验设置，结果须看实际保留的更新与独立评分。
- [camera_rgbd.yaml](camera_rgbd.yaml)：512×512 pinhole 相机参数基准。LIVO launcher 还从实际 capture 读取安装标定并校验参数服务，不把硬编码 fallback 当作标定成功。
- [fastlivo_lio_gravity_estimation.yaml](fastlivo_lio_gravity_estimation.yaml)：仅打开 gravity estimation 的对照覆盖文件；默认仍关闭，已有对照未显示改善。
- [README_LIO.md](README_LIO.md)：LIO/LIVO 启动、真实标定约定、隔离 namespace、post-VIO 评分、构建身份和生命周期修复记录。

入口为 [run_dynanav_lio.sh](../../../scripts/run_dynanav_lio.sh)。它不启动模拟器，也不直接发运动命令；默认 GT 诊断不会使用其估计，显式 `--slam-reference` 路线才按参考来源消费 live LIO 或 dedicated post-VIO pose。完整导航云与静态估计云分别保留，动态图像掩码尚未实现。

已完成的双楼梯旁路对照、live post-VIO 契约和当前闭环状态见[续篇](../../../docs/experiments/dynanav-20261109-continuation.md)，先前失败保留于[历史实验记录](../../../docs/experiments/dynanav-20261109.md)。不要由配置存在或 `--check` 通过推断路线/视觉精度已通过。

显式在线person模式另选`--lidar-topic /dynanav/lidar_perception_filtered`；该输入是临时UNKNOWN候选，不是认证static cloud。capture的`--pose-launch-receipt`必须匹配此observer的实际输入、post-VIO输出与ROS domain；默认legacy oracle-static输入保留。worker/observer/capture启动和独立评估范围见[本阶段报告](../../../docs/experiments/dynanav-online-person-20261109.md)。
