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
    # SPHERE: r, CYLINDER: rh, BOX: lwh
    DeclareLaunchArgument(
        'size',
        default_value=''
    ),

    object_finder = Node(
        package="franka_moveit",
        executable="object_finder",
        output="screen",
        parameters=[{
            "size": LaunchConfiguration('size'),
        }],
        emulate_tty=True,
    )

    # outlier_filter = Node(
    #     package="franka_moveit",
    #     executable="outlier_filter",
    #     output="screen",
    #     parameters=[{
    #     }],
    #     emulate_tty=True,
    # )

    return LaunchDescription([object_finder])