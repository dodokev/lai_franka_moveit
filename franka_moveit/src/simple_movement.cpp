#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "franka_moveit_msg/srv/set_trajectory.hpp"
#include "franka_moveit/utils.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>(
        "franka",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const LOGGER = rclcpp::get_logger("franka");

    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("tcp_trajectory", 10);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    const std::string plannerGroup = "fr3_arm";
    const std::string ee = "fr3_hand_tcp";

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, plannerGroup);
    move_group.startStateMonitor();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
    const moveit::core::JointModelGroup *joint_model = robot_state->getJointModelGroup(plannerGroup);

    // Settings
    move_group.setPlanningTime(1.0);
    move_group.setNumPlanningAttempts(10);

    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");

    RCLCPP_INFO(LOGGER, "EE name : %s", move_group.getEndEffector().c_str());
    RCLCPP_INFO(LOGGER, "EE link : %s", move_group.getEndEffectorLink().c_str());

    namespace rvt = rviz_visual_tools;
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "fr3_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // ================================================================================
    // ================================================================================

    auto current_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose start_pose;
    start_pose = current_pose;
    
    move_group.setStartStateToCurrentState();

    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose = start_pose;

    target_pose.position.x += 0.25;
    // target_pose.position.y = 0.0;
    // target_pose.position.z = 0.0;

    // target_pose.orientation.x = 0.0;
    // target_pose.orientation.y = 0.0;
    // target_pose.orientation.z = 0.0;
    // target_pose.orientation.w = 0.0;


    move_group.setPoseTarget(target_pose);

    // ================================================================================
    // ================================================================================

    bool pathFound = false;
    unsigned int iter = 0;
    do
    {
        ++iter;
        pathFound = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    } while (!pathFound);

    RCLCPP_INFO(LOGGER, "Path found at iteration : %d", iter);

    moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model);
    robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), plannerGroup);
    rt.setRobotTrajectoryMsg(*robot_state, plan.trajectory_);
    publishTcpTrajectory(rt, ee, marker_pub);
    moveit_visual_tools.trigger();
    
    // ==================================================================================================
    // -- SERVICE TO SEND CARTESIAN TRAJECTORY TO CUSTOM CONTROLLER --
    rclcpp::Client<franka_moveit_msg::srv::SetTrajectory>::SharedPtr client =
        node->create_client<franka_moveit_msg::srv::SetTrajectory>("robot_traj");
    
    auto request = std::make_shared<franka_moveit_msg::srv::SetTrajectory::Request>();
    request->trajectory = plan.trajectory_;
    
    using namespace std::chrono_literals;
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(LOGGER, "service not available, waiting again...");
    }
    
    auto result = client->async_send_request(request);
    // Wait for the result.
    while (rclcpp::ok())
    {
        auto status = result.wait_for(std::chrono::milliseconds(100));
        if (status == std::future_status::ready)
        {
            auto response = result.get();
            RCLCPP_INFO(LOGGER, "RESULT: %s", response->success ? "SUCCES" : "FAILED");
            break;
        }
    }
    // ==================================================================================================
    
    // moveit_visual_tools.prompt("Press next to execute the trajectory");
    // move_group.execute(plan);
                    
    rclcpp::shutdown();
    spinner.join();
    return 0;
}