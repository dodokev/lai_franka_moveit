cmake_minimum_required(VERSION 3.8)
project(franka_moveit_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(franka_msgs REQUIRED)

# Custom MSG
find_package(franka_moveit_msg REQUIRED)

add_executable(modular_mtc src/modular_mtc.cpp)
target_include_directories(modular_mtc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(modular_mtc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  modular_mtc
  "moveit_task_constructor_core"
  "rclcpp"
  "franka_msgs"
  franka_moveit_msg
)

add_executable(palette_mtc src/palette_mtc.cpp)
target_include_directories(palette_mtc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(palette_mtc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  palette_mtc
  "moveit_task_constructor_core"
  "rclcpp"
  "franka_msgs"
  franka_moveit_msg
)

install(TARGETS modular_mtc palette_mtc
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
