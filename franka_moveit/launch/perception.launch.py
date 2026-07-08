# Copyright 2023 RealSense, Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch realsense2_camera node."""
import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import LaunchConfiguration, PythonExpression

configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'camera_namespace',             'default': 'camera', 'description': 'namespace for camera'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},
                           {'name': 'accelerate_gpu_with_glsl',     'default': "false", 'description': 'enable GPU acceleration with GLSL'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'rosbag_loop',                  'default': 'false', 'description': 'Enable loop playback when playing a bagfile'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'rgb_camera.color_profile',     'default': '0,0,0', 'description': 'color stream profile'},
                           {'name': 'rgb_camera.color_format',      'default': 'RGB8', 'description': 'color stream format'},
                           {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'enable_infra',                 'default': 'false', 'description': 'enable infra0 stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'depth_module.depth_profile',   'default': '0,0,0', 'description': 'depth stream profile'},
                           {'name': 'depth_module.depth_format',    'default': 'Z16', 'description': 'depth stream format'},
                           {'name': 'depth_module.infra_profile',   'default': '0,0,0', 'description': 'infra streams (0/1/2) profile'},
                           {'name': 'depth_module.infra_format',    'default': 'RGB8', 'description': 'infra0 stream format'},
                           {'name': 'depth_module.infra1_format',   'default': 'Y8', 'description': 'infra1 stream format'},
                           {'name': 'depth_module.infra2_format',   'default': 'Y8', 'description': 'infra2 stream format'},
                           {'name': 'depth_module.color_profile',   'default': '0,0,0', 'description': 'Depth module color stream profile for d405'},
                           {'name': 'depth_module.color_format',    'default': 'RGB8', 'description': 'color stream format for d405'},
                           {'name': 'depth_module.exposure',        'default': '8500', 'description': 'Depth module manual exposure value'},
                           {'name': 'depth_module.gain',            'default': '16', 'description': 'Depth module manual gain value'},
                           {'name': 'depth_module.hdr_enabled',     'default': 'false', 'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},
                           {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'depth_module.exposure.1',      'default': '7500', 'description': 'Depth module first exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.1',          'default': '16', 'description': 'Depth module first gain value. Used for hdr_merge filter'},
                           {'name': 'depth_module.exposure.2',      'default': '1', 'description': 'Depth module second exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.2',          'default': '16', 'description': 'Depth module second gain value. Used for hdr_merge filter'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': "'enable sync mode'"},
                           {'name': 'depth_module.inter_cam_sync_mode',               'default': "0", 'description': '[0-Default, 1-Master, 2-Slave]'},
                           {'name': 'enable_rgbd',                  'default': 'false', 'description': "'enable rgbd topic'"},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "'enable gyro stream'"},
                           {'name': 'enable_accel',                 'default': 'false', 'description': "'enable accel stream'"},                           
                           {'name': 'enable_motion',                'default': 'false', 'description': "'enable motion stream (IMU) for DDS devices'"},
                           {'name': 'gyro_fps',                     'default': '0', 'description': "''"},
                           {'name': 'accel_fps',                    'default': '0', 'description': "''"},
                           {'name': 'motion_fps',                   'default': '0', 'description': "'motion stream samples per second'"},
                           {'name': 'unite_imu_method',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'clip_distance',                'default': '-2.', 'description': "''"},
                           {'name': 'angular_velocity_cov',         'default': '0.01', 'description': "''"},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'publish_tf',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': '[double] rate in Hz for publishing dynamic TF'},
                           {'name': 'pointcloud.enable',            'default': 'true', 'description': ''},
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'pointcloud.ordered_pc',        'default': 'false', 'description': ''},
                           {'name': 'pointcloud.allow_no_texture_points', 'default': 'false', 'description': "''"},
                           {'name': 'align_depth.enable',           'default': 'false', 'description': 'enable align depth filter'},
                           {'name': 'colorizer.enable',             'default': 'false', 'description': 'enable colorizer filter'},
                           {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'enable_decimation_filter'},
                           {'name': 'rotation_filter.enable',       'default': 'false', 'description': 'enable rotation_filter'},
                           {'name': 'rotation_filter.rotation',     'default': '0.0',   'description': 'rotation value: 0.0, 90.0, -90.0, 180.0'},
                           {'name': 'spatial_filter.enable',        'default': 'true', 'description': 'enable_spatial_filter'},
                           {'name': 'temporal_filter.enable',       'default': 'true', 'description': 'enable_temporal_filter'},
                           {'name': 'temporal_filter.filter_smooth_alpha',       'default': '0.05', 'description': 'alpha_temporal_filter'},
                           {'name': 'temporal_filter.filter_smooth_delta',       'default': '100', 'description': 'delta_temporal_filter'},
                           {'name': 'temporal_filter.frames_queue_size',       'default': '32', 'description': 'persistency_temporal_filter'},
                           {'name': 'disparity_filter.enable',      'default': 'false', 'description': 'enable_disparity_filter'},
                           {'name': 'hole_filling_filter.enable',   'default': 'false', 'description': 'enable_hole_filling_filter'},
                           {'name': 'hdr_merge.enable',             'default': 'false', 'description': 'hdr_merge filter enablement flag'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                           {'name': 'base_frame_id',                'default': 'link', 'description': 'Root frame of the sensors transform tree'},
                           {'name': 'tf_prefix',                    'default': '', 'description': 'prefix to be prepended to all frame IDs'},
                           {'name': 'decimation_filter.filter_magnitude', 'default': '2', 'description': 'decimation filter magnitude'},
                           {'name': 'enable_safety',                'default': 'false', 'description': "'enable safety stream'"},
                           {'name': 'safety_camera.safety_mode',    'default': '0', 'description': '[int] 0-Run, 1-Standby, 2-Service'},
                           {'name': 'enable_labeled_point_cloud',   'default': 'false', 'description': "'enable labeled point cloud stream'"},
                           {'name': 'depth_mapping_camera.labeled_point_cloud_profile', 'default': '0,0,0', 'description': "'Label PointCloud stream profile'"},
                           {'name': 'enable_occupancy',             'default': 'false', 'description': "'enable occupancy stream'"},
                           {'name': 'depth_mapping_camera.occupancy_profile', 'default': '0,0,0', 'description': "'Occupancy stream profile'"},
                          ]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    # Get list of supported parameters
    supported_params = set(param['name'] for param in configurable_parameters)
    
    # Check for unsupported parameters in command line arguments
    # Warn for any launch arguments not in supported_params
    for param_name in context.launch_configurations.keys():
        if param_name not in supported_params:
            print(f"\033[33mWarning: Parameter '{param_name}' is not supported. Supported parameters are:\n{sorted(supported_params)}\033[0m")
    
    # Check for unsupported parameters in config file
    if params_from_file:
        for param_name in params_from_file.keys():
            if param_name not in supported_params:
                print(f"\033[33mWarning: Parameter '{param_name}' in config file is not supported. Supported parameters are:\n{sorted(supported_params)}\033[0m")

    _output = LaunchConfiguration('output' + param_name_suffix)
    node_action = launch_ros.actions.Node

    if(os.getenv('ROS_DISTRO') == 'foxy'):
        # Foxy doesn't support output as substitution object (LaunchConfiguration object)
        # but supports it as string, so we fetch the string from this substitution object
        # see related PR that was merged for humble, iron, rolling: https://github.com/ros2/launch/pull/577
        _output = context.perform_substitution(_output)

    return [
        node_action(
            package='realsense2_camera',
            namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),
            name=LaunchConfiguration('camera_name' + param_name_suffix),
            executable='realsense2_camera_node',
            parameters=[params, params_from_file],
            output=_output,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
            emulate_tty=True,
            )
    ]

def generate_launch_description():
    description_name_arg = DeclareLaunchArgument(
        'description_name',
        default_value='/robot_description'
    )
    zero_for_removed_points_arg = DeclareLaunchArgument(
        'zero_for_removed_points',
        default_value='false'
    )
    lidar_sensor_type_arg = DeclareLaunchArgument(
        'lidar_sensor_type',
        default_value='0'
    )
    in_pointcloud_topic_arg = DeclareLaunchArgument(
        'in_pointcloud_topic',
        default_value='/camera/camera/depth/color/points'
    )
    out_pointcloud_topic_arg = DeclareLaunchArgument(
        'out_pointcloud_topic',
        default_value='/points_filtered'
    )
    robot_description_arg = DeclareLaunchArgument(
        'robot_description'
    )
    filter_config_arg = DeclareLaunchArgument(
        'filter_config',
        default_value='/home/labrob/franka_ros2_ws/src/lai_franka_moveit/franka_moveit_config/config/robot_filter.yaml',
    )
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # Keep default as true for standalone use
        description='Use simulation (Gazebo) clock if true'
    )

    # Create a log action to print the config
    log_config = LogInfo(msg=LaunchConfiguration('filter_config'))

    self_filter_node = Node(
        package='robot_self_filter',
        executable='self_filter',
        name='self_filter',
        output='screen',
        parameters=[
            LaunchConfiguration('filter_config'),  # loads the YAML file
            {
                'lidar_sensor_type': LaunchConfiguration('lidar_sensor_type'),
                'robot_description': ParameterValue(
                    LaunchConfiguration('robot_description'),
                    value_type=str
                ),
                'zero_for_removed_points': LaunchConfiguration('zero_for_removed_points'),
                'use_sim_time': LaunchConfiguration('use_sim_time') # Use the launch argument
            },
        ],
        remappings=[
            ('/robot_description', LaunchConfiguration('description_name')),
            ('/cloud_in', LaunchConfiguration('in_pointcloud_topic')),
            ('/cloud_out', LaunchConfiguration('out_pointcloud_topic')),
        ],
        emulate_tty=True,
    )

    cloud_world = Node(
        package="franka_moveit",
        executable="cloud_transform",
        output="screen",
        parameters=[{
            "input_topic": "/points_filtered",
            "output_topic": "/points_filtered_world",
            "target_frame": "world"
        }],
        emulate_tty=True
    )

    tag_config = PathJoinSubstitution([
        FindPackageShare('franka_moveit_config'),
        'config',
        'apriltag.yaml'
    ])

    april = Node(
        package='apriltag_ros',
        executable='apriltag_node',   # ✅ same as CLI
        name='apriltag',
        output='screen',
        parameters=[tag_config],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
    )

    # DONT FORGET TO CHANGE TF STATIC !!!!!!!!!
    april_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        # DONT FORGET TO CHANGE THIS PART
        arguments=["-0.155", "0.0", "0.0", "0.0", "0.0", "0.0", "tag36h11:15", "world"],
    )

    outlier_node = Node(
        package="franka_moveit",
        executable="outlier_filter",
        name="outlier_filter",
        output="log",
        emulate_tty=True
    )

    finder_node = Node(
        package="franka_moveit",
        executable="object_finder",
        name="object_finder",
        output="log",
        emulate_tty=True
    )

    remover_node = Node(
        package="franka_moveit",
        executable="object_remover",
        name="object_remover",
        output="log",
        emulate_tty=True
    )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        april,
        april_frame,
        OpaqueFunction(function=launch_setup, kwargs = {'params' : set_configurable_parameters(configurable_parameters)}),
        outlier_node,
        finder_node,
        remover_node,

        description_name_arg,
        zero_for_removed_points_arg,
        lidar_sensor_type_arg,
        in_pointcloud_topic_arg,
        out_pointcloud_topic_arg,
        robot_description_arg,
        filter_config_arg,
        use_sim_time_arg, # Add to launch description
        log_config,
        self_filter_node,
        cloud_world,
    ])
