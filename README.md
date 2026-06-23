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

The camera needs only to be connected through USB port and use the ros2 launch command.
```
ros2 launch franka_moveit rs.launch.py
```

---

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
ros2 run franka_moveit outlier_filter.cpp
```
The outlier filter will reduced the point cloud and remove some outliers clusters.

```
ros2 launch franka_moveit object_finder.launch.py size:="{size}"
```
The object finder needs a parameter to find and remove a object from the point cloud. {size} is a string of 1 to 3 dimensions expressed in meters. The dimensions are separeted with a comma, moreover the number of dimensions determined the shape of the object (Only three shapes possibles).

```
-- SPHERE : "radius"
ros2 launch franka_moveit object_finder.launch.py size:="0.2" 

-- CYLINDER : "heigth,radius"
ros2 launch franka_moveit object_finder.launch.py size:="0.206,0.034" 

-- BOX : "longest,mid,shortest"
ros2 launch franka_moveit object_finder.launch.py size:="0.255,0.153,0.112" 
```

    