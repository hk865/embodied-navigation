# 当前配置与有效参数

本目录保存普通入口的唯一可编辑profile及base资源。运行先读`navigation.json`，再由生成器写本次参数到`runs/`；不要把base文件、旧profile或字段旁注当作最终有效值。

## 配置层次

| 文件/生成器 | 职责 |
| --- | --- |
| `navigation.json` | 默认mission、速度档、控制/策略、特性、runtime路径、DDS环境 |
| `slam/fastlivo.yaml`、`camera.yaml` | 前端base参数与相机内参 |
| `dds_transport.xml` | loopback UDP+SHM拓扑，64MiB SHM、512 port queue、健康检查1000ms |
| `terrain_targets/lower12.json` | 保留的地形目标/策略provider信息 |
| `src/runtime_inputs.prepare` | 生成world、外参/SLAM覆盖、解析定位地图、SCAN参数和路线 |
| `src/sampling.override_sensors` | 当前LiDAR/RGB采样覆盖 |
| `src/scan_runtime_parameters.parameters` | 当前原生planner覆盖 |

## 当前重要覆盖

- LiDAR/RGB名义30Hz、64×480；IMU由`simulation/prepare.py`设200Hz。
- IMU初始化600样本、≥2.9s静止；在线bias和gravity开启。base YAML的300/false不代表日常值。
- VIO `img_point_cov=1000`覆盖base 1e8；曝光、inverse composition、raycast关闭；map sliding关闭。
- SCAN .08m voxel、10×10×5m、.65m horizon、vmax=.5m/s、amax=.15m/s²；tolerance仍在代码中。
- 主控制使用`cascade`，不是顶层legacy `pid`；Student当前有效命令界[.6,.2,.3]，唯一slew[.6,.6,.8]/s。
- 停车首次T0后5s，10Hz因果pose，XY/yaw漂移.05m/.1rad，源龄/状态覆盖300ms。

## 使用与调试

先在根运行`python3 -B start.py --check`做有限文件检查；`--prepare-only`会写生成物，可人工核对`navigation_profile.json`、`navigation_fastlivo.yaml`、`v34_scan_overrides.yaml`和world。两条均不等于完整运行通过；这里列命令没有在本次文档任务实际执行。

`navigation.json`保留许多历史字段/兼容结构及debug开关；是否被当前daily路径消费以源码为准。day环境还显式关闭部分探针，即便profile写1也不一定开启。修改需跟进读取/覆盖位置，别恢复旧hash/版本批准链。VIO线程的环境意图1与编译宏4存在差异，配置不能单独认证载入二进制团队。

默认mission是相对初始化的F3目标；goal Z不是地面真值。frame/安装/路线变更必须同步核对pose/cloud/Path约定，不能只改一个Z。说明见[技术报告](../docs/technical_report.zh-CN.md)与[结构文档](../docs/source_architecture.zh-CN.md)。
