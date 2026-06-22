cmake_minimum_required(VERSION 3.10)
project(franka_moveit_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(task_planner SHARED
  src/task_planner_manager.cpp
  src/task_planning_context.cpp
)

ament_target_dependencies(task_planner
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
)

target_link_libraries(task_planner
  ${rclcpp_LIBRARIES}
)

pluginlib_export_plugin_description_file(
  moveit_core
  plugin_description.xml
)

install(TARGETS task_planner
  LIBRARY DESTINATION lib
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()