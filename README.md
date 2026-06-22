## franka_moveit

This package use moveit2 and some part of franka_ros2 package (robot description)


## Those are the command to use for robot_self_filter package
# USE THIS IF WANT PRIMITIVE SHAPE AS FILTER
ros2 launch robot_self_filter self_filter.launch.py robot_description:="$(xacro /home/labrob/franka_ros2_ws/src/franka_description/robots/fr3/fr3_filter.urdf.xacro)" filter_config:=/home/labrob/franka_ros2_ws/src/robot_self_filter/params/example.yaml

# WHAT RECORD BAG
ros2 bag record /points_filtered_world -o franka_bag

