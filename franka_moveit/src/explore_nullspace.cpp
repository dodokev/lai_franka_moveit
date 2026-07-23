#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "franka_moveit/utils.hpp"

/**
 * Realized a path finding with nullspace exploration
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const LOGGER = rclcpp::get_logger("franka");
  RCLCPP_INFO(LOGGER, "RCLCPP ON ...");

  auto const node = std::make_shared<rclcpp::Node>(
      "franka",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO(LOGGER, "NODE OK ...");

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("tcp_trajectory", 10);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor](){ executor.spin(); });

  const std::string plannerGroup = "fr3_arm";
  const std::string ee = "fr3_hand_tcp";

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, plannerGroup);
  move_group.startStateMonitor();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
  const moveit::core::JointModelGroup *joint_model = robot_state->getJointModelGroup(plannerGroup);

  move_group.setPlanningTime(1.0);
  move_group.setNumPlanningAttempts(10);

  move_group.setPlanningPipelineId("task");

  namespace rvt = rviz_visual_tools;
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // ================================================================================================================================================================

  auto current_pose = move_group.getCurrentPose().pose;
  geometry_msgs::msg::Pose start_pose;
  start_pose = current_pose;
  
  // ----- Remove this if not usage in simulation, can be changed otherwise
  start_pose.position.x += 0.20;
  start_pose.position.z -= 0.40;
  
  bool foundIK = robot_state->setFromIK(joint_model, start_pose);
  if (!foundIK)
  RCLCPP_ERROR(LOGGER, "Not joint configuration found");
  else
  {
    move_group.setStartState(*robot_state);
    RCLCPP_INFO(LOGGER, "Start pose done");
  }
  
  Eigen::VectorXd start_joint;
  robot_state->copyJointGroupPositions(joint_model, start_joint);
  
  moveit_visual_tools.publishRobotState(*robot_state);
  moveit_visual_tools.trigger();
  // -----
  
  // Set a target Pose
  geometry_msgs::msg::Pose target_pose;
  target_pose = start_pose;

  target_pose.position.x += 0.20;

  move_group.setPoseTarget(target_pose);

  // ================================================================================================================================================================
  // === Nullspace exploration ===

  Eigen::VectorXd joints_positions;
  robot_state->copyJointGroupPositions(joint_model, joints_positions);

  Eigen::VectorXd dq_o(7);

  Eigen::Vector3d start_p(start_pose.position.x, start_pose.position.y, start_pose.position.z);
  Eigen::Vector3d start_quat(start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z);

  Eigen::VectorXd err(6);

  for (int i = 0; i < 7; i++)
    dq_o(i) = 1;

  bool pathFound = false;
  unsigned int iter = 0;
  double Ts = 0.1;

  std::vector<geometry_msgs::msg::Pose> pts;
  pts.push_back(start_pose);
  pts.push_back(target_pose);

  unsigned int counter = 0;
  unsigned int maxPath = 500;

  moveit_visual_tools.prompt("Begin plannnig");

  do
  {
    ++iter;

    // Plan one time by turn
    pathFound = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (pathFound)
    {
      RCLCPP_INFO(LOGGER, "OMPL Path %d : %s", iter, pathFound ? "SUCCESS" : "FAILED");
      ++counter;
      continue;
    }
    
    // Get the jacobian
    Eigen::Vector3d ref_pt(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    robot_state->getJacobian(joint_model, robot_state->getLinkModel(joint_model->getLinkModelNames().back()), ref_pt, jacobian);

    // Compute the inverse jacobian
    Eigen::MatrixXd J_pinv = eig_pinv(jacobian, 0.01, 0.001);

    /**
     *  Formula to compute the next configuration :
     *    dp = inv(J) * K * e + N * dq_o
     */ 

    // Compute the term N
    auto N = Eigen::MatrixXd::Identity(7, 7) - J_pinv * jacobian;

    // Compute the orientation (quaternion)
    auto current_tf = robot_state->getGlobalLinkTransform(ee);
    Eigen::Quaterniond tmp_quat(current_tf.rotation());
    Eigen::Vector3d current_quat(tmp_quat.x(), tmp_quat.y(), tmp_quat.z());

    // Compute the error between the start pose and the current pose
    Eigen::Vector3d pos_err = start_p - current_tf.translation();
    Eigen::Vector3d quat_err = (start_pose.orientation.w * -current_quat) + (tmp_quat.w() * start_quat) + start_quat.cross(current_quat);

    err.head<3>() = pos_err;
    err.tail<3>() = quat_err;

    // Compute dq with K = 0.3
    auto dq = J_pinv * (0.3 * err) + N * dq_o;
    // Compute the joints position for the timestep
    joints_positions += Ts * dq;

    // Set the new state configuration
    robot_state->setJointGroupPositions(joint_model, joints_positions);

    // Check the joints position bound
    if (!robot_state->satisfiesBounds(joint_model))
    {
      RCLCPP_INFO(LOGGER, "%d", iter);
      // Reset the position of the robot to the start
      robot_state->setJointGroupPositions(joint_model, start_joint);
      joints_positions = start_joint;

      // Inverse the direction of the exploration
      dq_o = -dq_o;
    }

    // Set the new state of the move group
    move_group.setStartState(*robot_state);

    robot_state->update();
    moveit_visual_tools.publishRobotState(*robot_state);
    moveit_visual_tools.trigger();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  } while (iter < maxPath && !pathFound); // Repeat until found path or exhaust of iteration

  RCLCPP_INFO(LOGGER, "Error position : %f meter(s)", err.head<3>().norm());
  RCLCPP_INFO_STREAM(LOGGER, "Error quaternion : \n"
                                 << err.tail<3>() << "\n");

  RCLCPP_INFO(LOGGER, "Nb waypoints OMPL : %ld", plan.trajectory_.joint_trajectory.points.size());

  // ================================================================================================================================================================
  // === Visualization of found path ===

  moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model);
  robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), plannerGroup);
  rt.setRobotTrajectoryMsg(*robot_state, plan.trajectory_);
  publishTcpTrajectory(rt, ee, marker_pub);
  moveit_visual_tools.trigger();

  // ================================================================================================================================================================
  // === Compute path from start to success configuration, recompute path from new configuration to target pose ===

  // moveit::planning_interface::MoveGroupInterface::Plan from_old2new;

  // // Home State
  // move_group.setStartStateToCurrentState();

  // // New State Target
  // std::vector<double> jp;
  // for (int i = 0; i < joints_positions.size(); i++)
  //   jp.push_back(joints_positions(i));

  // move_group.setJointValueTarget(jp);

  // do
  // {
  //   pathFound = move_group.plan(from_old2new) == moveit::core::MoveItErrorCode::SUCCESS;
  // } while (!pathFound);

  // moveit::planning_interface::MoveGroupInterface::Plan replan;

  // // New State
  // robot_state->setJointGroupPositions(joint_model, joints_positions);
  // robot_state->update();
  // move_group.setStartState(*robot_state);

  // // Target Pose
  // move_group.setPoseTarget(target_pose);

  // do
  // {
  //   pathFound = move_group.plan(replan) == moveit::core::MoveItErrorCode::SUCCESS;
  // } while (!pathFound);

  // ================================================================================================================================================================
  // === Draw the full path ===

  // moveit_visual_tools.publishTrajectoryLine(from_old2new.trajectory_, joint_model);
  // moveit_visual_tools.trigger();
  // moveit_visual_tools.publishTrajectoryLine(replan.trajectory_, joint_model);
  // moveit_visual_tools.trigger();
  // robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), plannerGroup);
  // rt.setRobotTrajectoryMsg(*robot_state, replan.trajectory_);
  // publishTcpTrajectory(rt, ee, marker_pub);
  // moveit_visual_tools.trigger();

  // ================================================================================================================================================================
  // === Execute trajectory ===
  
  // moveit_visual_tools.prompt("Next to go from Start State to New State");
  // move_group.execute(from_old2new);
  // moveit_visual_tools.prompt("Next to go to the Target Pose");
  // move_group.execute(replan);

  // ================================================================================================================================================================
    
  RCLCPP_INFO(LOGGER, "RCLCPP SHUTDOWN ...");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}