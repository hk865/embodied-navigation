# simulation/world

本目录负责从仓库资产生成一次运行所需的仿真世界与传感器契约，不启动 Gazebo，也不是运行时节点。

- [prepare.py](prepare.py)：以 `assets/world/go2_converted.sdf` 和 `assets/world/three_floors.sdf` 为源，生成 `world.sdf`、`sensor_contract.json`、`sensor_bridge.yaml`、`frames.urdf`、`asset_manifest.json`；剔除原 CHAMP/ros2_control 等执行链，只保留新的关节执行器插件。支持 `--terrain`（flat/ramp/step 等）、`--sensors`、`--camera-only`、`--overview-pose/--overview-fov`、`--camera-rate`、`--real-time-factor`、`--render-engine`。
- 生成的 `sensor_contract.json` 记录 IMU/LiDAR/camera 的 `body_xyz`、`body_rpy`、频率、frame 与 topic，供 FAST-LIVO2 外参、点云过滤和桥配置复用；`asset_manifest.json` 记录源文件与输出 SHA256、关节顺序、默认角和限位。`sensor_bridge.yaml` 实际是 JSON（JSON 也是合法 YAML），逐条列出 GZ 到 ROS 的 topic、类型和 QoS。
- `frames.urdf` 是只读的 frame/关节树：脚本会删掉 `ros2_control`、`gazebo`、`transmission`，避免与执行器插件争抢控制；碰撞摩擦统一写成 0.7（注释说明原始 URDF 是 0.2，属显式受控基线，不隐藏改动）。
- 调用方是 [../../launch/prepare.py](../../launch/prepare.py) 的 `prepare()`：它用 `--sensors --camera-rate 10 --render-engine ogre2` 调本脚本，随后改写机器人 pose 与执行器 `.so` 绝对路径；相关运行配置见 `configs/launch/settings.json`。

本模块无专用 unittest；`prepare.py` 只在 `start.py --prepare-only` 等真实准备路径执行。生成的 SDF 与契约是后续仿真输入，不代表仿真已经验证可运行。
