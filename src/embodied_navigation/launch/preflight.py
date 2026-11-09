"""只做文件级依赖检查；不要求 ROS、PyTorch 或本地构建产物已经可用。

本模块回答"启动前这些文件在不在"，不验证文件内容、版本、ROS 域占用或系统可
运行性。它不启动任何进程、不建目录、不写文件；`--check` 的退出码由调用方
`launch/main.py` 决定（就绪 0，缺项 2）。
"""

from embodied_navigation.common.config import ROOT, path


def check():
    """列出普通导航启动需要的文件，只检查不启动。

    标为 configured 的条目来自 :func:`embodied_navigation.common.config.path`
    （即 `configs/launch/settings.json` 的 `runtime_paths`，支持
    `${PROJECT_ROOT}` 展开与 `actor_python='auto'` 哨兵替换）；其余是固定的工程
    相对路径与构建产物。`ready` 仅当每一项都是已存在的普通文件时为 True——用
    的是 `Path.is_file`，所以目录、指向目录的符号链接都不算。项目内的依赖报告
    相对 `ROOT` 的路径，项目外（例如仓库外的 actor 解释器）保留绝对路径。这里
    没有进程、仿真或 socket，因此 `simulation_started` 恒为 False。
    """
    files = {
        "Student": path("student_model"),  # configured runtime path
        "Actor Python": path("actor_python"),  # configured runtime path ('auto' sentinel)
        "Go2 URDF": ROOT / "assets/world/go2_measured.urdf",
        "Go2 SDF": ROOT / "assets/world/go2_converted.sdf",
        "world": ROOT / "assets/world/three_floors.sdf",
        "route": ROOT / "assets/geometry/route.json",
        "L1 mesh": ROOT / "assets/meshes/L1_4D_Lidar.dae",
        "actuator": path("actuator"),  # configured runtime path
        "writer helper": ROOT / "install/communication/lib/writer_qos.so",
        "patched DDS": ROOT / "install/dds/lib/libfastrtps.so.2.14.6",
        "ROS install": path("ros_prefix") / "local_setup.bash",
        "SLAM": path("ros_prefix") / "lib/fast_livo2_ros/fastlivo_mapping",
        "SCAN": path("ros_prefix") / "lib/scan_planner/scan_planner_node",
    }
    result = {
        name: dict(
            # 项目内路径保持相对形式，报告更稳定；仓库外的保留绝对路径。
            path=str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p),
            present=p.is_file(),
        )
        for name, p in files.items()
    }
    return dict(
        ready=all(v["present"] for v in result.values()),
        dependencies=result,
        simulation_started=False,
    )
