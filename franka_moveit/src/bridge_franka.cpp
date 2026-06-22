#include "franka_moveit/bridge_franka.hpp"

#include <moveit/robot_trajectory/robot_trajectory.h>

static auto const LOGGER = rclcpp::get_logger("bridge_franka");

using namespace std::chrono_literals;

BridgeFranka::BridgeFranka()
: Node("bridge_franka", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    service_ = this->create_service<franka_moveit_msg::srv::SetTrajectory>(
        "robot_traj",
        std::bind(&BridgeFranka::handle_service, this,
                std::placeholders::_1,
                std::placeholders::_2)
    );
    
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    timer_ = this->create_wall_timer(1ms, std::bind(&BridgeFranka::callback, this));
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", qos);
}

void BridgeFranka::handle_service(
    const std::shared_ptr<franka_moveit_msg::srv::SetTrajectory::Request> request,
    std::shared_ptr<franka_moveit_msg::srv::SetTrajectory::Response> response)
{
    RCLCPP_INFO(LOGGER, "Bridge Franka Called ...");
    traj = request->trajectory;
    response->success = true;

    frankaLoop();
}

void BridgeFranka::callback()
{
    if (!pose_traj.empty())
    {
        pub_->publish(pose_traj.front());
        pose_traj.erase(pose_traj.begin());
    }
}

bool BridgeFranka::frankaLoop()
{
    RCLCPP_INFO(LOGGER, "Loop ...");
    robot_trajectory::RobotTrajectory rt(robot_model_, "fr3_arm");
    rt.setRobotTrajectoryMsg(*robot_state_, traj);

    rclcpp::Time start_time = shared_from_this()->get_clock()->now();

    for (std::size_t i = 0; i < rt.getWayPointCount(); i++)
    {
        geometry_msgs::msg::PoseStamped tmp;
        auto st = rt.getWayPoint(i);
        double time_from_start = rt.getWayPointDurationFromStart(i);

        auto tf = st.getGlobalLinkTransform("fr3_link8");

        tmp.pose.position.x = tf.translation().x();
        tmp.pose.position.y = tf.translation().y();
        tmp.pose.position.z = tf.translation().z();

        auto quat = Eigen::Quaterniond(tf.rotation());

        tmp.pose.orientation.x = quat.x();
        tmp.pose.orientation.y = quat.y();
        tmp.pose.orientation.z = quat.z();
        tmp.pose.orientation.w = quat.w();

        tmp.header.frame_id = "world";

        rclcpp::Duration dt = rclcpp::Duration::from_seconds(time_from_start);
        tmp.header.stamp = start_time + dt;

        // pub_->publish(tmp);
        pose_traj.push_back(tmp);
    }

    RCLCPP_INFO(LOGGER, "Finish ... %ld waypoints", rt.getWayPointCount());
    return true;
}

void BridgeFranka::init()
{
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
    robot_model_ = robot_model_loader.getModel();

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

    RCLCPP_INFO(this->get_logger(), "Robot model loaded: %s", robot_model_->getName().c_str());
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BridgeFranka>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}