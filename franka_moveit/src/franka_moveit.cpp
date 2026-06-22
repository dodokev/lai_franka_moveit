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

  auto const LOGGER = rclcpp::get_logger("franka");

  // Publisher for path of another link
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("tcp_trajectory", 10);
  auto start_pub = node->create_publisher<geometry_msgs::msg::Pose>("start_pose", 10);
  
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(node);
  // auto spinner = std::thread([&executor]() { executor.spin(); });
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  const std::string plannerGroup = "fr3_arm";

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, plannerGroup);
  move_group.startStateMonitor();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
  const moveit::core::JointModelGroup* joint_model = robot_state->getJointModelGroup(plannerGroup);

  // moveit::core::RobotModelPtr robot_model = move_group.getRobotModel();
  // Eigen::Vector3d ref_pt(0.0, 0.0, 0.0);
  // Eigen::MatrixXd jacobian;
  // robot_state->getJacobian(joint_model, robot_state->getLinkModel(joint_model->getLinkModelNames().back()),
  //                        ref_pt, jacobian);
  // RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

  // Settings
  double Ts = 0.1;

  move_group.setPlanningTime(30.0);

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRTConnectkConfigDefault");
  
  // RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  // move_group.setEndEffectorLink("panda_tool");
  // RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  
  auto goal_sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "target_pose",
    10,
    [&move_group, robot_state, joint_model, &LOGGER](geometry_msgs::msg::Pose p)
    {
      RCLCPP_INFO(LOGGER, "Received target pose");
      bool foundIK = robot_state->setFromIK(joint_model, p);
      if (!foundIK)
        RCLCPP_ERROR(LOGGER, "Not joint configuration found");
      else
      {
        std::vector<double> joints_positions;
        robot_state->copyJointGroupPositions(joint_model, joints_positions);

        move_group.setJointValueTarget(joints_positions);
        RCLCPP_INFO(LOGGER, "Target pose done");
      }
    }
  );

  auto start_sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "start_pose",
    10,
    [&move_group, robot_state, joint_model, &LOGGER](geometry_msgs::msg::Pose p)
    {
      RCLCPP_INFO(LOGGER, "Received start pose");
      bool foundIK = robot_state->setFromIK(joint_model, p);
      if (!foundIK)
        RCLCPP_ERROR(LOGGER, "Not joint configuration found");
      else
      {
        move_group.setStartState(*robot_state);
        RCLCPP_INFO(LOGGER, "Start pose done");
      }
    }
  );

  namespace rvt = rviz_visual_tools;
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  bool pathFound = false;
  auto plan_sub = node->create_subscription<std_msgs::msg::Empty>(
    "start_planning",
    10,
    [&move_group, robot_state, joint_model, &plan, &LOGGER, &pathFound, &Ts, &moveit_visual_tools, &marker_pub, &plannerGroup](std_msgs::msg::Empty){ 
      pathFound = false;
      unsigned int iter = 0;
      do
      {
        ++iter;
        
        pathFound = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        RCLCPP_INFO(LOGGER, "Path %d : %s", iter, pathFound ? "SUCCESS" : "FAILED");
      } while (iter < 5 && !pathFound);

      RCLCPP_INFO(LOGGER, "Let's draw ?");
      
      // moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model);
      robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), plannerGroup);
      // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

      rt.setRobotTrajectoryMsg(*robot_state, plan.trajectory_);

      publishTcpTrajectory(rt, "fr3_hand_tcp", marker_pub);
      moveit_visual_tools.trigger();
    }
  );

  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    // robot_state = move_group.getCurrentState();
    rate.sleep();
  }

  // ================================================================================
  
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // std::vector<std::string> objects_id;

  // moveit_msgs::msg::CollisionObject collision_obj;
  // collision_obj.header.frame_id = "world";
  // collision_obj.id = "simple_wall";

  // shape_msgs::msg::SolidPrimitive solid;
  // solid.type = solid.BOX;
  // solid.dimensions.resize(3);
  // solid.dimensions[solid.BOX_X] = 0.5;
  // solid.dimensions[solid.BOX_Y] = 0.05;
  // solid.dimensions[solid.BOX_Z] = 1.0;

  // geometry_msgs::msg::Pose wall_pose;
  // wall_pose.orientation.x = 0.0;
  // wall_pose.orientation.y = 0.0;
  // wall_pose.orientation.z = 0.0;
  // wall_pose.orientation.w = 0.0;
  // wall_pose.position.x = 0.6;
  // wall_pose.position.y = 0.0;
  // wall_pose.position.z = 0.1;

  // collision_obj.primitives.emplace_back(solid);
  // collision_obj.primitive_poses.emplace_back(wall_pose);
  // collision_obj.operation = collision_obj.ADD;
  
  // objects_id.push_back(collision_obj.id);

  // planning_scene_interface.applyCollisionObject(collision_obj);
  // moveit_visual_tools.prompt("Wall constructed, press Next to plan");
  // RCLCPP_INFO(LOGGER, "Wall constructed");
  
  // ================================================================================

  rclcpp::shutdown();
  spinner.join();
  return 0;
}