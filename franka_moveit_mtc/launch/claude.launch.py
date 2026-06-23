import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    namespace_parameter_name = 'namespace'
    load_gripper_parameter_name = 'load_gripper'
    ee_id_parameter_name = 'ee_id'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    ee_id = LaunchConfiguration(ee_id_parameter_name)

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        description='Hostname or IP address of the robot.')

    namespace_arg = DeclareLaunchArgument(
        namespace_parameter_name,
        default_value='',
        description='Namespace for the robot.'
    )
    load_gripper_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value='true',
        description='Whether to load the gripper or not (true or false)'
    )
    ee_id_arg = DeclareLaunchArgument(
        ee_id_parameter_name,
        default_value='franka_hand',
        description='The end-effector id to use. Available options: none, franka_hand, cobot_pump'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'robot_ip': robot_ip,
                          use_fake_hardware_parameter_name: use_fake_hardware,
                          'namespace': namespace}.items(),
    )

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    # planning_context
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, ' ee_id:=', ee_id, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands, ' ros2_control:=true'])

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=', load_gripper, ' ee_id:=', ee_id]
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    moveit_config = (
        MoveItConfigsBuilder("franka")
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True, publish_planning_scene=True, publish_geometry_updates=True,
            publish_state_updates=True, publish_transforms_updates=True,
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "task"]
        )
        .to_moveit_configs()
    )

    cartesian_lim_yaml = load_yaml("franka_moveit_config", "config/pilz_cartesian_limits.yaml")
    joint_lim_yaml = load_yaml("franka_moveit_config", "config/joint_limits.yaml")

    package = "franka_moveit_mtc"
    package_shared_path = get_package_share_directory(package)
    node = Node(
        package=package,
        executable="claude",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            joint_lim_yaml,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_scene_monitor,
            moveit_config.trajectory_execution,
            cartesian_lim_yaml,
            moveit_config.planning_pipelines,
        ],
        # prefix="gdb -ex run --args",
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_arg,
        namespace_arg,
        load_gripper_arg,
        ee_id_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        db_arg,
        node])
