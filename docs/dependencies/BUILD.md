# 独立原生构建与运行边界

## Docker 原生层

在本工程根目录执行：

```bash
# 只打印计划，不联网、不编译、不启动节点。
python3 -B docker/build_native.py

# 构建 CPU 原生依赖，需要 Docker 和访问公开 apt/GitHub 源。
# 独立原生依赖层；完整应用另用仓库根目录Dockerfile。
docker build -f docker/Dockerfile.native --target native_runtime \
  --build-arg BUILD_JOBS=2 -t embodied-navigation-native:jazzy .

# 只查看已安装 ROS 包，不启动导航或仿真。
docker run --rm embodied-navigation-native:jazzy ros2 pkg list
```

`Dockerfile.native.dockerignore` 限制构建上下文为 `third_party/` 和 `docker/`；本机数据、模型、训练目录和历史记录不会被该镜像偷偷打包。原生层只提供运行依赖和 `install/`。主应用源码、配置、部署模型和入口应在最终应用镜像中显式加入，不应声称此基础层单独即可运行完整场景。

基线为官方 `ros:jazzy-ros-base`（Ubuntu 24.04）、C++17、ROS Jazzy、Eigen/PCL/OpenCV/Sophus 和 ROS 发行版包。系统 apt 包与 ROS 基础镜像标签未锁定到快照或 digest，因此是版本明确的可恢复配方，不是位级完全可复现镜像。构建会记录实际 dpkg 包版本。若要长期严格复现，应在已验证构建后固定镜像 digest 和软件包快照。

公开源码固定为：

- FastDDS 2.14.6：`f052f239538ebc0df56835fdb57f7628628010bc`。
- Asio：`ed6aa8a13d51dfc6c00ae453fc9fb7df5d6ea963`。
- Livox SDK2：`f5d9375f84efe2b15bc0a052d3e18482ed13adf4`（原构建标注 v1.3.1）。

下载 URL、SHA256 在 `docker/source-pins.json`。SHA256 来源于已有源码构建回执，构建器会校验下载，不依赖运行时历史哈希。Livox 保留 GCC13 的 `<cstdint>` 兼容修复。FastDDS 先构建并执行已有 SHM 心跳回归测试，再构建 ROS 包。完整构建限 1–4 个 CPU job，默认 2，不使用 GPU。

已有 ROS Jazzy 和上述开发包的 Linux 环境也可执行：

```bash
source /opt/ros/jazzy/setup.bash
python3 -B docker/build_native.py --jobs 2 --execute
source install/ros/local_setup.bash
```

构建要求新的 `build/native` 和 `install/{dds,livox,ros,pct}`，不会复用旧工程的对象文件。不要在运行导航期间重建。实际输出为：

- `install/ros/lib/fast_livo2_ros/fastlivo_mapping`
- `install/ros/lib/scan_planner/scan_planner_node`
- `install/ros/lib/scan_planner/closed_loop_controller`
- `install/ros/` 中生成的 `scan_planner_msgs` C++/Python 消息
- `install/pct/pct_astar`
- `install/dds/lib/libfastrtps.so.2.14.6`
- `install/livox/` 中的 SDK2 库和头文件

`docker/native_entrypoint.sh` 加载 ROS 和本工程 overlay，并优先使用本工程 DDS/Livox 库。应用仍需显式配置 DDS profile、ROS_DOMAIN_ID 和数据位置。原 `USE_PROJECT_DDS=1` 模式要保留本工程 DDS profile；不能只设置库路径却忽略通讯配置。

## 快速离线验证

```bash
python3 -B docker/verify_native_sources.py --compile-pct
```

此检查无网络、无 ROS 节点、无仿真：校验复制哈希、XML 包名、CMake 语法和静态源文件引用、Python 语法、VIO 补丁及纯 CPU PCT 编译/简单网格规划。临时编译文件放在 `docs/dependencies/` 的临时目录中，结束时移除。

## 不包含的运行环境

- 原恢复脚本实际使用的 Isaac Sim 是 `6.0.1-rc.7+release.42383.32955d8d.gl`，含 experimental API。该安装不是本工程源码，也不能从 `ros:jazzy-ros-base` 获得。这个精确 RC 版本的公开可获取性尚未验证。需要按 NVIDIA 正式授权方式安装兼容 Isaac Sim、GPU 驱动和容器运行时，并重新测试；不能未经验证就换成 5.x 并宣称等价。
- 原生镜像不包含 Isaac Sim、RTX/PhysX、场景资产、Student 模型或可选视觉模型。完整 Isaac 运行仍未验证。
- 根目录完整应用镜像现已包含官方OSRF Gazebo Harmonic (`gz-sim8`、`gz-plugin2`)、CPU `torch==2.7.1`，并从本仓库编译actuator/backend/writer_qos/Go2HV四个自有原生模块。它与Isaac流程是不同入口。
- 完整应用Docker已从公开ROS基础镜像构建，11个ROS包及四个自有原生模块编译通过，数据只读窄挂载下preflight通过。最终源码测试/短时软件栈结果见docs/verification；短时运行不等于完整路线、Building分支、实机或Isaac验收。

## 完整应用构建

根目录Dockerfile是完整Native Gazebo应用，不需要挂载其它本机自建工程的代码或install目录。默认使用官方Ubuntu源；若网络需要，可显式指定仍执行APT签名校验的Ubuntu镜像，例如本次验证使用：

```bash
docker build --build-arg BUILD_JOBS=2 \
  --build-arg UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu \
  -t embodied-navigation:local .
```

该参数只修改镜像内APT源，不改主机网络或软件源。公开源码下载仍按source-pins.json验证SHA256。构建产物记录实际系统包版本；基础镜像标签和APT仓库不是历史快照，不能承诺跨日期字节一致。代码和数据的运行边界、可选私有资产与许可问题见根目录README和LICENSE-STATUS.md。
