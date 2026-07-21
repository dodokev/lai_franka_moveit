#include <memory>
#include <thread> 

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "franka",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    static auto const LOGGER = rclcpp::get_logger("additional_setup");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    moveit_msgs::msg::CollisionObject object;
    object.id = "static0";
    object.header.frame_id = "world";
    object.primitives.resize(2);

    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[1].type = shape_msgs::msg::SolidPrimitive::BOX;
    
    object.primitives[0].dimensions = {0.1, 0.1, 0.1};
    object.primitives[1].dimensions = {0.2, 0.2, 0.2};

    geometry_msgs::msg::Pose p;
    
    p.position.x = 0.0;
    p.position.y = 0.0;
    p.position.z = 0.0;

    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 1.0;

    object.primitive_poses.resize(2);
    object.primitive_poses[0] = p;

    p.position.x = 1.0;
    p.position.y = 1.0;
    p.position.z = 1.0;

    object.primitive_poses[1] = p;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);

    rclcpp::shutdown();
    spinner.join();
    return 0;
}