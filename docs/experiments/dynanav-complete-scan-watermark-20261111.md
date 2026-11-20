# RTX 完整扫描区间证据与 pending 释放

## 范围

本修改用显式完整采集区间证据代替 opt-in 模式的尾有效 ray 启发式，允许已经保留在 pending 的原点继续进入原 LIO/VIO 路径。默认关闭。没有改变 header、逐点 t、IMU stamp、point<T 分割、curvature 浮点重基准、预处理、IMU/ESIKF 数学、物理速度、source-age、unknown、TTL 或原 USD。没有提交或 push。

- 原基线保留：`build_cache/dynanav_same_pair_20261110T171010Z`。
- 新独立构建：`build_cache/dynanav_complete_scan_20261110T231145Z`，包含修改前 `baseline/`、冻结 `source/`、日志、测试和 manifest。
- rosidl 的 CMake `file(STRINGS)` 在中文路径截断 IDL tuple；新构建使用同一目录的 ASCII symlink `/home/hyh001/.cache/dynanav_complete_scan_20261110T231145Z`，不修改旧构建。

## 显式契约

`dynanav_sensor_msgs/CompleteLidarScan` 把原 PointCloud2 和 typed 证明放在同一原子消息里。没有第二 metadata topic 的 latest join，不重载 frame_id 或 is_dense，不添假点。

Producer 必须显式 `DYNANAV_RTX_COMPLETE_SCAN=1`，核验官方冻结 OS1_REV6_32ch10hz1024res USD 的 SHA256 `ec5b732ef18521efe33ccdfc4e1242e9c3ce066e9482a91d29b3cea91fc7000b`，并只读比较构造后的所有冻结 sensor 属性（含 firing schedule）。固定条件为 32 通道、32 emitters、10Hz、10240Hz pattern、accumulateOutputs=true。只有 scanComplete=1、原 GMO S 和原 offset 合法、无 header rebase、帧/sequence/session 连续才出证明。

此固定生产相位中，坐标 frameStart=S+15ms、frameEnd=S+100ms。坐标 frameStart **不是扫起点**。这个窄相位约束来自两次实录的全部288扫；它不是所有 GMO 传感器的通用语义。其他相位拒绝，不能凭 10Hz 或尾 ray 推断完成。完整扫积累与 scanComplete、固定配置、相位共同支持 [S,E) 的完成认证，绝不把无回波当 free space。

Native 默认 `common.complete_scan_watermark=false`。开启时为 typed-only：先解析基 topic 的 ROS exact remap，再订阅 `/complete_scan` 后缀。它核验 schema、固定 sensor/config/frame、S/E、坐标frame相位、全部原点 offset 在 [S,E)、完整点布局/长度及 atomic shared_ptr 绑定。首个区间以前不被声明覆盖；只在下一 S==上一 E、sequence+1、native frame 增长、session不变时推进。缺证书、partial、缺包、重复、乱序、错配或 restart 在该模式失败关闭，不能退回尾点路径。receiver只报线程安全 ingress failure，owner维护 watermark，失败必须开新 owned run。

覆盖随 raw→pending 持久化。raw 为空时，必须有实际非空 pending、全部pending原点在目标前、目标在连续区间内，并满足原 image identity、旧图、完整IMU和pair生命周期，才允许继续。冷启动只在 previous<0 时从第一实际S播种，早于S的图按原OldImageDropped丢弃；已非负previous<S不重播种、不桥接缺口。没有完整证明时不得用未来尾点跨过。

收到实际完整证书的空packet/全部过滤packet，可以更早执行原下游 EmptyCloud/Skip；这与旧 RawTailEmpty 等下一包的等待时序不同。仍不产生假LioReady/VIO、假点或IMU，空pending不能单凭watermark继续。测试覆盖空包后下一非空包的原IMU传播和pair生命周期。

## 验证与边界

新 native fixture 除原5套外，覆盖21类证书/布局/相位错误、失败持久化、实际ordered ingress partial/gap/越界失败、无metadata两包及direct callback、raw/pending精确E、缺IMU、旧图、首S之前previous、早camera冷启动、pending点精确边界、空pending、空包/全过滤及恢复。合成双路用相同初态比较原点次序/float curvature、全IMU列表、last_imu、prop_end、deskew、state/cov、VIO输入和source ns。

两个原run各保存9.0–10.1s的12个连续原始record NPZ。`scripts/prepare_complete_scan_replay.py`直接取NPY payload（避免结构数组padding重建），与receipt point_sha256核对，再提取原IMU。双路片段测试比较22个目标：原current点/顺序/float curvature、全IMU列表、deskew、last_imu/prop_end、state/cov、VIO输入source stamp。它使用同一新初态和空白图像，**不是原run全轨迹、历史地图或真实图像VIO重放**。旧diagnostics没有全量原点且没有原图像历史，不得声称原run数值完全等价。

最终测试结果以新build的 `test-result.txt`、`artifact/*/replay.log`、`kit-python-type-support.log`、`source-manifest.json`、`build-identity.json` 为准。Kit Python实际3.12.13的生成类型import/构造/CDR序列化往返已独立CPU检查，无Kit app、GPU或ROS实例。

## 运行开关

必须由唯一GPUowner发起新的异步零动作诊断。不要直接继续移动。新prefix仍显式指定，旧默认prefix不切换。

Capture环境（在已有ROS/Isaac环境上追加）：

```sh
source /opt/ros/jazzy/setup.bash
source /home/hyh001/isaacsim/setup_python_env.sh
PREFIX=/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_complete_scan_20261110T231145Z
export PYTHONPATH="$PREFIX/install/ros/lib/python3.12/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$PREFIX/install/ros/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="$PREFIX/install/ros:$AMENT_PREFIX_PATH"
export DYNANAV_RTX_COMPLETE_SCAN=1
```

Observer维持RTX LIVO与same-pair参数，显式 `--binary-prefix "$PREFIX"`，在现有run override中加 `common.complete_scan_watermark: true`；或用独立 `configs/simulation/dynanav/fastlivo_rtx_complete_scan.yaml`。单个override槽不要覆盖其他已授权run参数。`--check`会拒绝不含新能力的旧core。capture现有legacy cloud仍原样发布，native开启时只消费atomic envelope，trace分别记录两个真实publish边界。

仍未验证：实际SDK live prim属性认证、实时DDS完整流、source age与控制占空改善、1m/s导航。完整扫描采集/渲染等待和约80ms几何计算仍在。本修改不是实时导航验收。
