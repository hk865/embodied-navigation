# 实体源码依赖

本目录包含可随工程提交的实际 C++/ROS 源文件，不是指向本机其它工程的链接，也不包含旧 `build/`、`install/`、`.git/` 或编译产物。

- `fast_livo2/`：FAST-LIVO2 算法库和 ROS 2 节点。两个包保持相邻，保留其 CMake 相对头文件引用。
- `scan/`：局部规划的五个包与 `scan_planner_msgs`。
- `pct/`：实际调用的原生 A* CLI 依赖闭包，不包含未使用的 Python 绑定和 OSQP 平滑模块。
- `vikit/`：FAST-LIVO2 的相机模型、视觉工具和 ROS 2 参数加载依赖。
- `livox_ros_driver2/`：FAST-LIVO2 编译依赖的消息和驱动包；仿真不需要启动硬件驱动。
- `fastdds/`：固定公开上游版本、实际修改补丁和许可文本；构建时从公开上游获取 FastDDS/Asio，独立安装到本工程。

唯一有意改变的算法源文件是 `fast_livo2/fast_livo2_core/src/vio.cpp`，与迁移前实际使用的隔离修复版字节一致。补丁保存在 `fast_livo2/patches/`，处理四处负数体素坐标向下取整；构建不再依赖旧目标文件或旧 CMake 缓存。

完整来源与逐文件 SHA256 见 `../docs/dependencies/vendored-source-manifest.json`。哈希用于复制和来源校验，不作为机器人运行或导航验收条件。

原许可继续适用。特别是 Vikit 的独立许可文本缺失、项目自有代码和模型的对外授权尚未统一，见 `../docs/dependencies/LICENSE-STATUS.md`。不能把本目录统一标记为 MIT/Apache，也不能把可构建等同于已解决再分发授权。
