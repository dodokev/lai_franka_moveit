import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import yaml
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("franka")
        .robot_description(file_path="config/fr3.urdf.xacro")
        .robot_description_semantic(file_path="config/fr3.srdf")
        .to_moveit_configs()
    )

    franka_node = Node(
        package="franka_moveit",
        executable="franka_moveit",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        emulate_tty=True,
    )

    return LaunchDescription([franka_node])