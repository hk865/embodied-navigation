"""用一个共享的 `run_dir` 参数依次 include 四个模块的 launch 文件。

逻辑名到包路径的映射：`localization` 对应 `localization_mapping` 包；另外三个
逻辑名同时也是包名。本文件只声明参数与 include 清单，不启动进程、不读写 run
文件、不构造环境；真正的组件清单在各模块的 `launch/components.py`。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from embodied_navigation.common.config import ROOT


def generate_launch_description():
    """返回含 `run_dir` 参数声明与四个 include 动作的 `LaunchDescription`。

    `run_dir` 没有默认值，必须由启动方提供：`launch/main.py` 以
    `ros2 launch .../demo.launch.py run_dir:=<run目录>` 传入。每个被 include 的
    子 launch 文件都必须接受同名 `run_dir` 参数，否则 include 虽成功但参数落不
    到组件。四个文件按列表顺序 include；路径由 :data:`ROOT` 拼成绝对路径，不
    依赖当前工作目录。
    """
    modules = [
        # (逻辑名, launch 文件名)。只有 localization 的逻辑名与包目录名不同。
        ("localization", "slam"),
        ("planning", "scan"),
        ("control", "control"),
        ("simulation", "simulation"),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("run_dir"),
            *[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(
                            ROOT
                            / "src/embodied_navigation"
                            / ("localization_mapping" if module == "localization" else module)
                            / "launch"
                            / (name + ".launch.py")
                        )
                    ),
                    # 把同一个 run_dir 传给每个子 launch，保证四个模块写同一目录。
                    launch_arguments={"run_dir": LaunchConfiguration("run_dir")}.items(),
                )
                for module, name in modules
            ],
        ]
    )
