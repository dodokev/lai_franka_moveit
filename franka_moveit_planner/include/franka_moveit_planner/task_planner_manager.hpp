#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <rclcpp/rclcpp.hpp>
static auto const LOGGER = rclcpp::get_logger("task_planner_manager");

namespace task_planner
{

class TaskPlannerManager : public planning_interface::PlannerManager
{
public:
  TaskPlannerManager() {
    RCLCPP_WARN(LOGGER, "Planner Constructed");
  }

  bool initialize (const moveit::core::RobotModelConstPtr &model,
    const rclcpp::Node::SharedPtr &node,
    const std::string &parameter_namespace) override;

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override;

  std::string getDescription() const override
  {
    return "Task Cartesian + OMPL Planner";
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;

  int max_iterations_;
  double step_size_;
  double goal_bias_;
  double goal_tolerance_;
  double clearance_;
};

}  // namespace task_planner