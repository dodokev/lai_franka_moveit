cmake_minimum_required(VERSION 3.8)
project(franka_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Custom MSG
find_package(franka_moveit_msg REQUIRED)

# ================================================================================
# Create executabla

add_executable(statistic_exp src/statistic_exp.cpp)
target_link_libraries(statistic_exp xlsxwriter)

add_executable(franka_moveit src/franka_moveit.cpp)
add_executable(explore_nullspace src/explore_nullspace.cpp)
add_executable(simple_movement src/simple_movement.cpp)

add_executable(cloud_transform src/cloud_transform.cpp)
add_executable(outlier_filter src/outlier_filter.cpp)
add_executable(octomap_generator src/octomap_generator.cpp)

add_executable(object_finder src/object_finder.cpp)
add_executable(object_remover src/object_remover.cpp)
add_executable(bridge_franka src/bridge_franka.cpp)

# ================================================================================
# --- Include directories ---

target_include_directories(statistic_exp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(franka_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(explore_nullspace PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(simple_movement PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(cloud_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(outlier_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(octomap_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_include_directories(object_finder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(object_remover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(bridge_franka PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
# ================================================================================
# --- Dependencies ---

ament_target_dependencies(
  statistic_exp
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
)

ament_target_dependencies(
  franka_moveit
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
)
 
ament_target_dependencies(
  explore_nullspace
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
)
  
ament_target_dependencies(
  simple_movement
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  franka_moveit_msg
)

ament_target_dependencies(
  cloud_transform
  "rclcpp"
  "moveit_ros_planning_interface"
)
ament_target_dependencies(
  outlier_filter
  "rclcpp"
  pcl_ros
)
ament_target_dependencies(
  octomap_generator
  "rclcpp"
  "moveit_ros_planning_interface"
  pcl_ros
  )
  
ament_target_dependencies(
  object_finder
  "rclcpp"
  "moveit_ros_planning_interface"
  pcl_ros
  franka_moveit_msg
)
ament_target_dependencies(
  object_remover
  "rclcpp"
  "moveit_ros_planning_interface"
  tf2
  pcl_ros
)
ament_target_dependencies(
  bridge_franka
  "rclcpp"
  "moveit_ros_planning_interface"
  franka_moveit_msg
)

# ================================================================================
# --- Compile features ---

target_compile_features(statistic_exp PUBLIC c_std_99 cxx_std_17)

target_compile_features(franka_moveit PUBLIC c_std_99 cxx_std_17)
target_compile_features(explore_nullspace PUBLIC c_std_99 cxx_std_17)
target_compile_features(simple_movement PUBLIC c_std_99 cxx_std_17)

target_compile_features(cloud_transform PUBLIC c_std_99 cxx_std_17)
target_compile_features(outlier_filter PUBLIC c_std_99 cxx_std_17)
target_compile_features(octomap_generator PUBLIC c_std_99 cxx_std_17)

target_compile_features(object_finder PUBLIC c_std_99 cxx_std_17)
target_compile_features(object_remover PUBLIC c_std_99 cxx_std_17)
target_compile_features(bridge_franka PUBLIC c_std_99 cxx_std_17)

# ================================================================================
# --- Install executable to lib ---

install(TARGETS franka_moveit explore_nullspace simple_movement
  cloud_transform outlier_filter octomap_generator
  object_finder object_remover bridge_franka statistic_exp
DESTINATION lib/${PROJECT_NAME})

# ================================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
