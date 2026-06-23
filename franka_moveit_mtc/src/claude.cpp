#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <moveit/task_constructor/cost_terms.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/move.hpp>
#include <franka_msgs/action/grasp.hpp>

#include <moveit/task_constructor/solvers/multi_planner.h>

#include <functional>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  enum class SHAPE { NONE, SPHERE, CYLINDER, BOX };

  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();
  bool setupPlanner();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createPickTask();
  mtc::Task createPlaceTask();

  
  // Predicate used by PredicateFilter stages to reject solutions that
  // get too close to obstacles or violate joint bounds.
  std::function<bool(const mtc::SolutionBase&, std::string&)> makeClearancePredicate();
  
  // Builds a fresh Task via `factory`, plans it, and executes the first
  // solution. On execution FAILURE (not planning failure) it rebuilds the
  // task from `factory` again -- since each task starts from the real,
  // monitored planning scene (CurrentState), this naturally replans only
  // from wherever the robot/scene actually ended up, instead of restarting
  // the whole multi-task pipeline.
  bool planAndExecute(const std::string& label, const std::function<mtc::Task()>& factory,
  int max_plan_attempts = 3, int max_execute_retries = 2);
  
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
  std::shared_ptr<mtc::solvers::MultiPlanner> multipipeline_planner_;
  
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr model_{nullptr};

  double x_{0.0};
  double y_{0.0};
  double z_{0.0};

  double margin_{0.002};
  double hand_max{0.034};

  geometry_msgs::msg::Point position_;
  geometry_msgs::msg::Quaternion orientation_;

  SHAPE type_{SHAPE::NONE};

  std::string arm_group_name_;
  std::string hand_group_name_;
  std::string hand_frame_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  position_.x = 0.5;
  position_.y = -0.25;
  position_.z = 0.0;

  orientation_.x = 0;
  orientation_.y = 0;
  orientation_.z = 0;
  orientation_.w = 1;

  arm_group_name_ = "fr3_arm";
  hand_group_name_ = "fr3_hand";
  hand_frame_ = "fr3_hand_tcp";
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);

  // object.primitives[0].dimensions = {0.206, 0.034}; // bigger cylinder
  object.primitives[0].dimensions = { 0.185, 0.029 };  // smaller cylinder

  // object.primitives[0].dimensions = {0.06, 0.077, 0.284}; // box

  type_ = static_cast<SHAPE>(object.primitives[0].dimensions.size());

  if (type_ == SHAPE::CYLINDER)
  {
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    z_ = object.primitives[0].dimensions[0];
    x_ = object.primitives[0].dimensions[1];
  }
  else if (type_ == SHAPE::BOX)
  {
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    x_ = object.primitives[0].dimensions[0];
    y_ = object.primitives[0].dimensions[1];
    z_ = object.primitives[0].dimensions[2];
  }

  geometry_msgs::msg::Pose pose;
  position_.z = z_ / 2 + margin_;
  pose.position = position_;
  pose.orientation = orientation_;

  object.primitive_poses.push_back(pose);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

bool MTCTaskNode::setupPlanner()
{
  auto rrtstar_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  rrtstar_planner->setMaxVelocityScalingFactor(0.1);
  rrtstar_planner->setMaxAccelerationScalingFactor(0.1);
  rrtstar_planner->setTimeout(10.0);
  rrtstar_planner->setPlannerId("RRTstarkConfigDefault");

  auto rrtconnect_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  rrtconnect_planner->setMaxVelocityScalingFactor(0.1);
  rrtconnect_planner->setMaxAccelerationScalingFactor(0.1);
  rrtconnect_planner->setPlannerId("RRTConnectkConfigDefault");

  auto pilz_planner =
      std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
  pilz_planner->setMaxVelocityScalingFactor(0.1);
  pilz_planner->setMaxAccelerationScalingFactor(0.1);
  pilz_planner->setPlannerId("PTP");

  multipipeline_planner_ = std::make_shared<mtc::solvers::MultiPlanner>();
  multipipeline_planner_->push_back(pilz_planner);
  multipipeline_planner_->push_back(rrtstar_planner);
  multipipeline_planner_->push_back(rrtconnect_planner);

  interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(0.15);
  cartesian_planner_->setMaxAccelerationScalingFactor(0.15);
  cartesian_planner_->setStepSize(.005);

  return true;
}

std::function<bool(const mtc::SolutionBase&, std::string&)> MTCTaskNode::makeClearancePredicate()
{
  // Same logic you had duplicated in createPickTask/createPlaceTask, now in one place.
  return [this](const mtc::SolutionBase& solution, std::string&) {
    auto scene = solution.end()->scene();
    const moveit::core::RobotState& state = scene->getCurrentState();
    const auto& model = scene->getRobotModel();

    std::set<const moveit::core::LinkModel*> links;
    const moveit::core::JointModelGroup* joint_group = model->getJointModelGroup(arm_group_name_);

    const std::string& ee_name = joint_group->getLinkModelNames().back();
    if (const auto* lm = model->getLinkModel(ee_name))
      links.insert(lm);

    std::vector<const moveit::core::AttachedBody*> attached;
    state.getAttachedBodies(attached);

    for (const auto* body : attached)
    {
      const std::string& parent = body->getAttachedLinkName();
      if (joint_group->hasLinkModel(parent))
      {
        if (parent == "fr3_link0" || parent == "fr3_link1" || parent == "fr3_link2")
          continue;
        if (const auto* lm = model->getLinkModel(parent))
          links.insert(lm);
      }
    }

    collision_detection::DistanceRequest req;
    req.active_components_only = &links;
    req.type = collision_detection::DistanceRequestType::GLOBAL;
    req.enable_nearest_points = false;
    req.acm = &(scene->getAllowedCollisionMatrix());

    collision_detection::DistanceResult res;
    scene->getCollisionEnv()->distanceRobot(req, res, state);

    const double min_dist = 0.02;
    if (res.minimum_distance.distance < min_dist)
      return false;

    const auto& sub = dynamic_cast<const mtc::SubTrajectory&>(solution);
    const auto& traj = sub.trajectory();

    for (std::size_t i = 0; i < traj->getWayPointCount(); ++i)
    {
      const auto& wp = traj->getWayPoint(i);
      scene->getCollisionEnv()->distanceRobot(req, res, wp);

      if (res.minimum_distance.distance < min_dist)
        return false;

      if (!wp.satisfiesBounds(0.005))
      {
        RCLCPP_WARN(LOGGER, "Waypoint violates joint bounds");
        return false;
      }
    }
    return true;
  };
}

bool MTCTaskNode::planAndExecute(const std::string& label, const std::function<mtc::Task()>& factory,
                                  int max_plan_attempts, int max_execute_retries)
{
  for (int exec_attempt = 0; exec_attempt <= max_execute_retries; ++exec_attempt)
  {
    if (exec_attempt > 0)
      RCLCPP_WARN(LOGGER, "[%s] Re-planning from current state (retry %d/%d)", label.c_str(),
                  exec_attempt, max_execute_retries);

    // Build a brand-new Task each attempt. Its CurrentState stage will pick up
    // whatever the *real* robot/planning-scene state is right now, so this
    // naturally resumes from wherever the previous (failed) execution left off.
    mtc::Task task;
    
    if (model_ == nullptr)
    {
      RCLCPP_WARN(LOGGER, "Load the robot model one time");
      task.loadRobotModel(node_);
      model_ = task.getRobotModel();
    }
    
    task = factory();
    
    try
    {
      task.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "[" << label << "] Init failed: " << e);
      continue;
    }

    if (!task.plan(max_plan_attempts))
    {
      RCLCPP_ERROR(LOGGER, "[%s] Planning failed", label.c_str());
      continue;
    }

    task.introspection().publishSolution(*task.solutions().front());

    auto result = task.execute(*task.solutions().front());
    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(LOGGER, "[%s] Execution succeeded", label.c_str());
      return true;
    }

    RCLCPP_ERROR(LOGGER, "[%s] Execution failed (error code %d)", label.c_str(), result.val);
    // loop -> rebuild + replan this same subtask from the now-current state
  }

  RCLCPP_ERROR(LOGGER, "[%s] Giving up after %d execution retries", label.c_str(), max_execute_retries);
  return false;
}

mtc::Task MTCTaskNode::createPickTask()
{
  mtc::Task task;
  task.setRobotModel(model_);

  task.setProperty("group", arm_group_name_);
  task.setProperty("eef", hand_group_name_);
  task.setProperty("ik_frame", hand_frame_);

  if (!task.getRobotModel())
  {
    RCLCPP_ERROR(LOGGER, "Robot model not loaded");
    return task;
  }

  auto clearance_predicate = makeClearancePredicate();

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  {
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner_);
    stage_open_hand->setGroup(hand_group_name_);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));
  }

  {
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, multipipeline_planner_ } });
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);

    auto wrapper = std::make_unique<mtc::stages::PredicateFilter>("filter pick", std::move(stage_move_to_pick));
    wrapper->setPredicate(clearance_predicate);
    task.add(std::move(wrapper));
  }

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto alternate = std::make_unique<mtc::Alternatives>("orientation variation");
      task.properties().exposeTo(alternate->properties(), { "eef", "group", "ik_frame" });
      alternate->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      struct Approach
      {
        const char* name;
        Eigen::AngleAxisd rot;
        double z_offset;
        double x_offset;
      };

      std::vector<Approach> approaches = {
        { "pick from above", Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()), z_ / 2 - hand_max + 0.01, 0.0 },
        { "pick from below", Eigen::AngleAxisd(2 * M_PI, Eigen::Vector3d::UnitX()), z_ / 2 - hand_max + 0.01, 0.0 },
      };

      for (const auto& approach : approaches)
      {
        auto alt = std::make_unique<mtc::SerialContainer>(approach.name);
        alternate->properties().exposeTo(alt->properties(), { "eef", "group", "ik_frame" });
        alt->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose("open");
        stage->setObject("object");
        if (type_ == SHAPE::BOX)
          stage->setAngleDelta(M_PI / 2);
        else if (type_ == SHAPE::CYLINDER)
          stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr);

        Eigen::Isometry3d grasp_frame_transform;
        grasp_frame_transform.linear() = approach.rot.matrix();
        grasp_frame_transform.translation().z() = approach.z_offset;
        grasp_frame_transform.translation().x() = approach.x_offset;

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

        {
          auto approach_stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner_);
          approach_stage->properties().set("marker_ns", "approach_object");
          approach_stage->properties().set("link", hand_frame_);
          approach_stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          approach_stage->setMinMaxDistance(0.1, 0.15);

          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = hand_frame_;
          vec.vector.z = 1.0;
          approach_stage->setDirection(vec);
          alt->add(std::move(approach_stage));
        }

        alt->add(std::move(wrapper));
        alternate->add(std::move(alt));
      }

      // "pick from side" kept separate since its transform differs in shape.
      {
        auto alt = std::make_unique<mtc::SerialContainer>("pick from side");
        alternate->properties().exposeTo(alt->properties(), { "eef", "group", "ik_frame" });
        alt->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose("open");
        stage->setObject("object");
        if (type_ == SHAPE::BOX)
          stage->setAngleDelta(M_PI / 2);
        else if (type_ == SHAPE::CYLINDER)
          stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr);

        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();
        grasp_frame_transform.translation().z() = -z_ / 2 + 2 * hand_max + 2 * 0.0005 + std::max(x_, y_);
        grasp_frame_transform.translation().x() = -z_ / 4;

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

        {
          auto approach_stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner_);
          approach_stage->properties().set("marker_ns", "approach_object");
          approach_stage->properties().set("link", hand_frame_);
          approach_stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          approach_stage->setMinMaxDistance(0.1, 0.15);

          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = hand_frame_;
          vec.vector.z = 1.0;
          approach_stage->setDirection(vec);
          alt->add(std::move(approach_stage));
        }

        alt->add(std::move(wrapper));
        alternate->add(std::move(alt));
      }

      grasp->insert(std::move(alternate));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          "object", task.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner_);
      stage->setGroup(hand_group_name_);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame_);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner_);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.15);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  return task;
}

mtc::Task MTCTaskNode::createPlaceTask()
{
  mtc::Task task;
  task.setRobotModel(model_);

  task.setProperty("group", arm_group_name_);
  task.setProperty("eef", hand_group_name_);
  task.setProperty("ik_frame", hand_frame_);

  auto clearance_predicate = makeClearancePredicate();

  // IMPORTANT: this task no longer reaches into the pick task's
  // attach_object_stage_. By execution time the object is *really* attached
  // in the monitored planning scene, so a CurrentState stage here sees it
  // attached too. GeneratePlacePose monitors this stage instead.
  mtc::Stage* current_state_ptr = nullptr;
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, multipipeline_planner_ } });
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);

    auto wrapper = std::make_unique<mtc::stages::PredicateFilter>("filter place", std::move(stage_move_to_place));
    wrapper->setPredicate(clearance_predicate);
    task.add(std::move(wrapper));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("drop object", cartesian_planner_);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.15);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "drop_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = position_.x;
      target_pose_msg.pose.position.y = 0.25;
      target_pose_msg.pose.position.z = position_.z;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(current_state_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner_);
      stage->setGroup(hand_group_name_);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
          "object", task.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame_);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner_);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", multipipeline_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner_);
    stage->setGroup(hand_group_name_);
    stage->setGoal("close");
    task.add(std::move(stage));
  }

  return task;
}

void MTCTaskNode::doTask()
{
  // Each subtask is independently plan-and-execute-with-retry. If "pick"
  // execution fails partway, only "pick" gets rebuilt and replanned from the
  // robot's real current state -- "place" is never touched. If "pick"
  // succeeds but "place" fails, only "place" gets retried.
  if (!planAndExecute("pick", [this] { return createPickTask(); }))
  {
    RCLCPP_ERROR(LOGGER, "Pick failed after all retries, aborting.");
    return;
  }

  if (!planAndExecute("place", [this] { return createPlaceTask(); }))
  {
    RCLCPP_ERROR(LOGGER, "Place failed after all retries, aborting.");
    return;
  }

  RCLCPP_INFO(LOGGER, "Pick and place completed successfully.");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanner();
  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}