#include "franka_moveit_planner/task_planner_manager.hpp"
#include "franka_moveit_planner/task_planning_context.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace task_planner
{

bool TaskPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                const rclcpp::Node::SharedPtr& node,
                const std::string& ns) {
  RCLCPP_WARN(LOGGER, "TaskPlannerManager initialised.");
  robot_model_ = model;
  node_ = node;

  node_->get_parameter(ns + ".config.step_size", step_size_);
  node_->get_parameter(ns + ".config.max_iterations", max_iterations_);
  node_->get_parameter(ns + ".config.goal_bias", goal_bias_);
  node_->get_parameter(ns + ".config.goal_tolerance", goal_tolerance_);
  node_->get_parameter(ns + ".config.clearance", clearance_);

  RCLCPP_WARN_STREAM(LOGGER, "Parameters : step:" << step_size_
    << " | iter:" << max_iterations_
    << " | bias:" << goal_bias_ 
    << " | tol:" << goal_tolerance_
    << " | clr:" << clearance_
  );

  return true;
}

planning_interface::PlanningContextPtr TaskPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes&          error_code) const
{
  if (!robot_model_)
  {
    RCLCPP_ERROR(LOGGER, "Robot model is null — was initialize() called?");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return nullptr;
  }

  if (!robot_model_->hasJointModelGroup(req.group_name))
  {
    RCLCPP_ERROR(LOGGER, "Unknown planning group: '%s'", req.group_name.c_str());
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return nullptr;
  }

  auto context = std::make_shared<TaskPlanningContext>(
      "task_context", req.group_name, robot_model_, planning_scene);

  context->setParams(max_iterations_, step_size_, goal_bias_, goal_tolerance_, clearance_);

  context->setMotionPlanRequest(req);

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return context;
}

bool TaskPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
  // Accept any group that exists in the model.
  return robot_model_ && robot_model_->hasJointModelGroup(req.group_name);
}

void TaskPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& /*pcs*/)
{
  // No per-planner config needed; extend here if you add ROS parameters later.
}

}  // namespace task_planner

PLUGINLIB_EXPORT_CLASS(task_planner::TaskPlannerManager,
                       planning_interface::PlannerManager)