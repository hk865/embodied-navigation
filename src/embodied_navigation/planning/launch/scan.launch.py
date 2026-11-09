"""Launch the SCAN planning stack for one prepared run directory.

``run_dir`` is the only argument: every child process is defined by
``planning.launch.components`` and reads its parameters from that directory.
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from embodied_navigation.planning.launch.components import components


def generate_launch_description():
    """Resolve ``run_dir`` at launch time and hand the actions to launch."""

    def start(context):
        return components(Path(LaunchConfiguration("run_dir").perform(context)).resolve())

    return LaunchDescription([DeclareLaunchArgument("run_dir"), OpaqueFunction(function=start)])
