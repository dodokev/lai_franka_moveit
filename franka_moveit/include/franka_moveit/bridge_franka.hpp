#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "franka_moveit_msg/srv/set_trajectory.hpp"

class BridgeFranka : public rclcpp::Node
{
private:
    /* data */
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;

    moveit_msgs::msg::RobotTrajectory traj;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::Service<franka_moveit_msg::srv::SetTrajectory>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> pose_traj;

    void callback();
public:
    BridgeFranka(/* args */);
    ~BridgeFranka() = default;

    void init();
    void handle_service(
        const std::shared_ptr<franka_moveit_msg::srv::SetTrajectory::Request> request,
        std::shared_ptr<franka_moveit_msg::srv::SetTrajectory::Response> response);

    bool frankaLoop();
};