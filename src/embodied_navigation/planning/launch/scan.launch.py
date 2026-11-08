from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from embodied_navigation.planning.launch.components import components

def generate_launch_description():
    def start(context):return components(Path(LaunchConfiguration('run_dir').perform(context)).resolve())
    return LaunchDescription([DeclareLaunchArgument('run_dir'),OpaqueFunction(function=start)])
