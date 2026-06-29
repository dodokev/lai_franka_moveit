#include <memory>
#include <thread> 

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/empty.hpp>

#include "franka_moveit/utils.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "franka",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  static auto const LOGGER = rclcpp::get_logger("franka");

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("tcp_trajectory", 10);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  const std::string plannerGroup = "fr3_arm";

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, plannerGroup);
  move_group.startStateMonitor();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
  // const moveit::core::JointModelGroup* joint_model = robot_state->getJointModelGroup(plannerGroup);

  move_group.setPlanningTime(30.0);

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRTConnectkConfigDefault");
   
  auto goal_sub = node->create_subscription<geometry_msgs::msg::Pose>("target_pose", 10,
    [&move_group](geometry_msgs::msg::Pose p)
    {
      RCLCPP_INFO(LOGGER, "Received target pose");
      move_group.setPoseTarget(p);
    }
  );

  namespace rvt = rviz_visual_tools;
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto plan_sub = node->create_subscription<std_msgs::msg::Empty>("start_planning", 10,
    [&move_group, robot_state, &plan, &moveit_visual_tools, &marker_pub, &plannerGroup](std_msgs::msg::Empty){ 
      move_group.setStartStateToCurrentState();
      
      bool pathFound = false;
      unsigned int iter = 0;
      do
      {
        ++iter;
        
        pathFound = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        RCLCPP_INFO(LOGGER, "Path %d : %s", iter, pathFound ? "SUCCESS" : "FAILED");
      } while (iter < 5 && !pathFound);

      RCLCPP_INFO(LOGGER, "Let's draw ?");
      
      robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), plannerGroup);
      rt.setRobotTrajectoryMsg(*robot_state, plan.trajectory_);

      publishTcpTrajectory(rt, "fr3_hand_tcp", marker_pub);
      moveit_visual_tools.trigger();
    }
  );

  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    rate.sleep();
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}