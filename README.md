# LAI ROBOTICS INTERNSHIP PROJECT
This package has the goal to move the Emika FR3 Arm Robot while avoiding obstacle.

## Package Dependencies

This package requires multiple packages to be fully used, some aren't mandatory but are the ones used for the project.
- [franka_ros2](https://github.com/frankarobotics/franka_ros2) - Mandatory
- [moveit.picknik.ai](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html), Follow the guide to install everything needed for moveit2 usage. - Mandatory
- [apriltag_ros](https://github.com/christianrauch/apriltag_ros) AND [apriltag_msgs](https://github.com/christianrauch/apriltag_msgs) - Optional
- [robot_self_filter](https://github.com/leggedrobotics/robot_self_filter) - Optional
- [realsense-ros](https://github.com/realsenseai/realsense-ros) - Optional

If the optional packages aren't installed you will need to provide your point cloud message and topic. The input topic inside the franka_moveit/src/outlier_filter.cpp must be yours, the output topic is used by others nodes and plugins, so it doesn't need to be modify.

Moreover the rs.launch.py file and the self_filter.launch.py file won't be usable, because it requires the realsense_ros package and the robot_self_filter package respectively.

# Console command to use the package (Considering the optional packages are installed)
## Perception


The robot self filter requires the robot urdf and the parameters (scale, padding) of the collision geometries.
```
ros2 launch franka_moveit self_filter.launch.py robot_description:="$(xacro /{path}/{robot_name}.urdf.xacro)" filter_config:=/{path}/{parameters}.yaml
```

(/home/labrob/franka_ros2_ws/src/franka_description/robots/fr3/fr3_filter.urdf.xacro)
(/home/labrob/franka_ros2_ws/src/lai_franka_moveit/franka_moveit_config/config/robot_filter.yaml)

ros2 launch franka_moveit self_filter.launch.py robot_description:="$(xacro /home/labrob/franka_ros2_ws/src/franka_description/robots/fr3/fr3_filter.urdf.xacro)" filter_config:=/home/labrob/franka_ros2_ws/src/lai_franka_moveit/franka_moveit_config/config/robot_filter.yaml

The parameters can be modified in the franka_moveit_config/config/robot_filter.yaml. It is useful only if your robot urdf has simple shape as collision geometry (Sphere, Cylinder, Box). If you use meshes as collision geometry then the padding and scale doesn't work.

---

The april tag detection is already used by the camera launch file. However you will need to modify the franka_moveit_config/config/apriltag.yaml and the static transformation (l.187 in franka_moveit/launch/rs.launch.py).
The parameters to change, in the yaml file, are the tag's family and his size, otherwise the detection will not work or the detection will be offset.

---

After launching the previous nodes, you will need to run and/or launch two others nodes.
```
ros2 run franka_moveit outlier_filter
```
The outlier filter will reduced the point cloud and remove some outliers clusters. It is also launch with the camera. So you don't need the run it in a separate terminal.

```
ros2 run franka_moveit object_finder
ros2 run franka_moveit object_remover
```

To add a object to find, you will need to send a message to the /add_lost_obj topic.

```
ros2 topic pub /add_lost_obj std_msgs/msg/String "{data: 'SIZE'}"
```

SIZE has to be replaced by the size in meters of your object. The dimensions are separated by a comma. Each shapes have their syntax fot the variable.
Here some examples : 

```
// -- SPHERE : [radius]
ros2 topic pub /add_lost_obj std_msgs/msg/String "{data: '0.2'}"

// -- CYLINDER : [height,radius]
ros2 topic pub /add_lost_obj std_msgs/msg/String "{data: '0.206,0.034'}"

// -- BOX : [longest,..,shortest]
ros2 topic pub /add_lost_obj std_msgs/msg/String "{data: '0.255,0.153,0.112'}"
```

To launch the perception pipeline you will need to launch the self filter launch file and the rs.launch.py
```
ros2 launch franka_moveit rs.launch.py
```
This launch file regroups everything needed to run the camera and the April Tag detection. Plus the two nodes for object detection and removal. 

## MoveIt2

To plan or do task, we need to launch the components for MoveIt2. As we use the fr3 Emika Robot, only the launch file for the fr3 is created.

```
ros2 launch fr3_moveit_controller.launch.py robot_ip:=IP use_fake_hardware:=[true|false]
```

When the FCI of the franka robot is activate, only use the robot_ip argument and change IP by the IP used by the robot.
If you want stay in simulation, use the use_fake_hardware argument and put anything in robot_ip argument.

Example :
```
// -- FCI
ros2 launch fr3_moveit_controller.launch.py robot_ip:=172.16.0.2

// -- Simulation
ros2 launch fr3_moveit_controller.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

The gripper used in the simulation is different from the real gripper of the franka_ros2 package. So if you send an action or anything related to the gripper in simulation, you have to modify your code for the real gripper.

### Simple Planning

To use the planning part, you have two choices, using the rviz2 interface or using some code.
In rviz2, there is a component called MotionPlanning. From the dedicated panel, you can change the planning algorithm and parameters, modify the scene to add obstacles, and of course plan and execute a path.

There is launch files in the franka_moveit package that allow to plan and execute.

If you want a simple movement (from the current pose to a goal pose). This command will be used, however the goal has to be modified manually in the code.
```
ros2 launch franka_moveit simple.launch.py
```

### Exploring the nullspace

This launch file allow to explore the robot nullspace and plan the path from the current configuration to a goal pose. The start pose and the goal are hardcoded in the 

// REVIEW THE EXPLORE NULLSPACE EXECUTABLE

### Franka Service Class

If you are annoyed to hard coded the goal pose, then you can launch the service node for MoveIt
```
ros2 launch franka_moveit franka.launch.py
```

Then you can send a topic message to /target_pose and /start_planning.
```
// -- To set a goal pose
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: X, y: Y, z: Z}, orientation: {x: X, y: Y, z: Z, w: W}}" --once

// -- To launch the planning
ros2 topic pub /start_planning std_msgs/msg/Empty "{}" --once
```

### Configuration Planner

To change the parameters of the different planner protocols, you have to modify the files named like these {planning_pipeline}_planning.yaml with planning_pipeline the name of pipeline (ompl, chomp, pilz_industrial_motion_planner).

## MoveIt Task Constructor

