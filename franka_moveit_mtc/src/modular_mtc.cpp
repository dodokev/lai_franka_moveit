#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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

#include <moveit/task_constructor/solvers/multi_planner.h>

#include <functional>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

// #include <moveit_task_constructor_msgs/msg/solution_info.hpp>
// #include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

using ExecuteTaskSolutionAction = moveit_task_constructor_msgs::action::ExecuteTaskSolution;

class MTCTaskNode {
 public:
  enum class SHAPE { NONE, SPHERE, CYLINDER, BOX };

  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  bool setupPlanner();
  
  bool doTask();

private:
  // Compose an MTC task from a series of stages.
  void createPickTask();
  void createPlaceTask();

  void fillTask();
  bool executeTask();

  void addCurrentStage();

  mtc::Stage* current_state_ptr_{nullptr};
  mtc::Stage* pick_stage_ptr_{nullptr};

  // Predicate used by Connect/PredicateFilter stages to reject solutions that
  // get too close to obstacles or violate joint bounds.
  std::function<bool(const mtc::SolutionBase&, std::string&)> makeClearancePredicate();

  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
  std::shared_ptr<mtc::solvers::MultiPlanner> multipipeline_planner_;

  rclcpp::Node::SharedPtr node_;

  double x_{0.0};
  double y_{0.0};
  double z_{0.0};

  double margin_{0.0035};
  double hand_max{0.034};

  geometry_msgs::msg::Point position_;
  geometry_msgs::msg::Quaternion orientation_;

  SHAPE type_{SHAPE::NONE};

  mtc::Task task_;
  moveit::core::RobotModelConstPtr model_;

  std::string arm_group_name_;
  std::string hand_group_name_;
  std::string hand_frame_;

  std::string stage_failed_{""};
  unsigned int recovery_allowed_{0};
  unsigned int recovery_done_{0};
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{std::make_shared<rclcpp::Node>("mtc_node", options)} {
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

  task_.loadRobotModel(node_);
  model_ = task_.getRobotModel();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface() {
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene() {
  moveit_msgs::msg::CollisionObject object;
  object.id = "object0";
  object.header.frame_id = "world";
  object.primitives.resize(1);

  // object.primitives[0].dimensions = {0.206, 0.034}; // bigger cylinder
  object.primitives[0].dimensions = {0.185, 0.029};  // smaller cylinder

  // object.primitives[0].dimensions = {0.06, 0.077, 0.284}; // box

  type_ = static_cast<SHAPE>(object.primitives[0].dimensions.size());

  if (type_ == SHAPE::CYLINDER) {
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    z_ = object.primitives[0].dimensions[0];
    x_ = object.primitives[0].dimensions[1];
  } else if (type_ == SHAPE::BOX) {
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
  // psi.applyCollisionObject(object);
}

bool MTCTaskNode::setupPlanner() {
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
  multipipeline_planner_->push_back(rrtconnect_planner);
  multipipeline_planner_->push_back(pilz_planner);
  multipipeline_planner_->push_back(rrtstar_planner);

  interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(0.05);
  cartesian_planner_->setMaxAccelerationScalingFactor(0.05);
  cartesian_planner_->setStepSize(.005);

  return true;
}

std::function<bool(const mtc::SolutionBase&, std::string&)> MTCTaskNode::makeClearancePredicate() {
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

    for (const auto* body : attached) {
      const std::string& parent = body->getAttachedLinkName();
      if (joint_group->hasLinkModel(parent)) {
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

    for (std::size_t i = 0; i < traj->getWayPointCount(); ++i) {
      const auto& wp = traj->getWayPoint(i);
      scene->getCollisionEnv()->distanceRobot(req, res, wp);

      if (res.minimum_distance.distance < min_dist)
        return false;

      if (!wp.satisfiesBounds(0.005)) {
        RCLCPP_WARN(LOGGER, "Waypoint violates joint bounds");
        return false;
      }
    }
    return true;
  };
}

void MTCTaskNode::addCurrentStage() {
  task_.setRobotModel(model_);

  task_.setProperty("group", arm_group_name_);
  task_.setProperty("eef", hand_group_name_);
  task_.setProperty("ik_frame", hand_frame_);

  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr_ = stage_state_current.get();
    task_.add(std::move(stage_state_current));
  }
}

void MTCTaskNode::createPickTask() {
  task_.setRobotModel(model_);

  task_.setProperty("group", arm_group_name_);
  task_.setProperty("eef", hand_group_name_);
  task_.setProperty("ik_frame", hand_frame_);

  if (!task_.getRobotModel()) {
    RCLCPP_ERROR(LOGGER, "Robot model not loaded");
    return;
  }

  auto clearance_predicate = makeClearancePredicate();

  auto serial = std::make_unique<mtc::SerialContainer>("PickTask");
  task_.properties().exposeTo(serial->properties(), {"eef", "group", "ik_frame"});
  serial->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

  {
    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("openHand", interpolation_planner_);
    stage_open_hand->setGroup(hand_group_name_);
    stage_open_hand->setGoal("open");
    serial->insert(std::move(stage_open_hand));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "CONNECT",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name_, multipipeline_planner_}});
    stage->properties().configureInitFrom(mtc::Stage::PARENT);

    auto wrapper = std::make_unique<mtc::stages::PredicateFilter>("move2pick", std::move(stage));
    wrapper->setPredicate(clearance_predicate);
    serial->insert(std::move(wrapper));
  }

  {
    auto alternate = std::make_unique<mtc::Alternatives>("pickOrientation");
    task_.properties().exposeTo(alternate->properties(), {"eef", "group", "ik_frame"});
    alternate->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    struct Approach {
      const char* name;
      Eigen::AngleAxisd rot;
      double z_offset;
      double x_offset;
    };

    std::vector<Approach> approaches = {
        {"pickAbove", Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()), z_ / 2 - hand_max + 0.01,
         0.0},
        {"pickBelow", Eigen::AngleAxisd(2 * M_PI, Eigen::Vector3d::UnitX()),
         z_ / 2 - hand_max + 0.01, 0.0},
    };

    for (const auto& approach : approaches) {
      auto alt = std::make_unique<mtc::SerialContainer>(approach.name);
      alternate->properties().exposeTo(alt->properties(), {"eef", "group", "ik_frame"});
      alt->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generationGrasp");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      if (type_ == SHAPE::BOX)
        stage->setAngleDelta(M_PI / 2);
      else if (type_ == SHAPE::CYLINDER)
        stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr_);

      Eigen::Isometry3d grasp_frame_transform;
      grasp_frame_transform.linear() = approach.rot.matrix();
      grasp_frame_transform.translation().z() = approach.z_offset;
      grasp_frame_transform.translation().x() = approach.x_offset;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("graspIK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

      {
        auto approach_stage =
            std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner_);
        approach_stage->properties().set("marker_ns", "approach");
        approach_stage->properties().set("link", hand_frame_);
        approach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
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
      auto alt = std::make_unique<mtc::SerialContainer>("pickSide");
      alternate->properties().exposeTo(alt->properties(), {"eef", "group", "ik_frame"});
      alt->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generateGrasp");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      if (type_ == SHAPE::BOX)
        stage->setAngleDelta(M_PI / 2);
      else if (type_ == SHAPE::CYLINDER)
        stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr_);

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() =
          -z_ / 2 + 2 * hand_max + 2 * 0.0005 + std::max(x_, y_);
      grasp_frame_transform.translation().x() = -z_ / 4;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("graspIK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

      {
        auto approach_stage =
            std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner_);
        approach_stage->properties().set("marker_ns", "approach");
        approach_stage->properties().set("link", hand_frame_);
        approach_stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
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

    serial->insert(std::move(alternate));
  }

  {
    auto container = std::make_unique<mtc::SerialContainer>("pickObject");
    task_.properties().exposeTo(container->properties(), {"eef", "group", "ik_frame"});
    container->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allowCollision");
      stage->allowCollisions("object",
                             task_.getRobotModel()
                                 ->getJointModelGroup(hand_group_name_)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      container->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("closeHand", interpolation_planner_);
      stage->setGroup(hand_group_name_);
      stage->setGoal("close");
      container->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attachObject");
      stage->attachObject("object", hand_frame_);
      pick_stage_ptr_ = stage.get();
      container->insert(std::move(stage));
    }

    serial->insert(std::move(container));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.05, 0.15);
    stage->setIKFrame(hand_frame_);
    stage->properties().set("marker_ns", "lift");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    serial->insert(std::move(stage));
  }

  task_.add(std::move(serial));
}

void MTCTaskNode::createPlaceTask() {
  task_.setRobotModel(model_);

  task_.setProperty("group", arm_group_name_);
  task_.setProperty("eef", hand_group_name_);
  task_.setProperty("ik_frame", hand_frame_);

  auto clearance_predicate = makeClearancePredicate();

  auto serial = std::make_unique<mtc::SerialContainer>("PlaceTask");
  task_.properties().exposeTo(serial->properties(), {"eef", "group", "ik_frame"});
  serial->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "CONNECT",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name_, multipipeline_planner_}});
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);

    auto wrapper = std::make_unique<mtc::stages::PredicateFilter>("move2place",
                                                                  std::move(stage_move_to_place));
    wrapper->setPredicate(clearance_predicate);
    serial->insert(std::move(wrapper));
  }

  {
    auto container = std::make_unique<mtc::SerialContainer>("poseObject");
    task_.properties().exposeTo(container->properties(), {"eef", "group", "ik_frame"});
    container->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("place", cartesian_planner_);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.15);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "place");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      container->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generatePose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = 0.5;
      target_pose_msg.pose.position.y = 0.25;
      target_pose_msg.pose.position.z = z_/2 + margin_;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(pick_stage_ptr_);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("poseIK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      container->insert(std::move(wrapper));
    }

    serial->insert(std::move(container));
  }

  {
    auto container = std::make_unique<mtc::SerialContainer>("leaveObject");
    task_.properties().exposeTo(container->properties(), {"eef", "group", "ik_frame"});
    container->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("openHand", interpolation_planner_);
      stage->setGroup(hand_group_name_);
      stage->setGoal("open");
      container->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbidCollision");
      stage->allowCollisions("object",
                             task_.getRobotModel()
                                 ->getJointModelGroup(hand_group_name_)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      container->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detachObject");
      stage->detachObject("object", hand_frame_);
      container->insert(std::move(stage));
    }

    serial->insert(std::move(container));
  }
  
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.1, 0.15);
    stage->setIKFrame(hand_frame_);
    stage->properties().set("marker_ns", "retreat");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame_;
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    serial->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("returnHome", multipipeline_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("ready");
    serial->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("closeHand", interpolation_planner_);
    stage->setGroup(hand_group_name_);
    stage->setGoal("close");
    serial->insert(std::move(stage));
  }

  task_.add(std::move(serial));
}

void MTCTaskNode::fillTask()
{
  addCurrentStage();
  if (stage_failed_ == "PickTask" || stage_failed_ == "") {
    createPickTask();
    createPlaceTask();
  }
  else if (stage_failed_ == "PlaceTask") {
    createPlaceTask();
  }
}

bool MTCTaskNode::executeTask()
{
  auto solution = task_.solutions().front();

  auto container = task_.stages();
  std::vector<std::string> stage_names;
  RCLCPP_WARN(LOGGER, "NB Stage : %ld", container->numChildren());
  for (std::size_t i = 0; i < container->numChildren(); i++) {
    RCLCPP_INFO(LOGGER, "Stage %ld name : %s", i, ((*container)[i]->name()).c_str());
    stage_names.push_back((*container)[i]->name());
  }

  const auto* seq = dynamic_cast<const mtc::SolutionSequence*>(solution.get());
  for (auto* s : seq->solutions())
  {
    const auto* c = s->creator();
    stage_failed_ = c->name();
    RCLCPP_WARN(LOGGER, "Stage executing : %s", stage_failed_.c_str());

    auto result = task_.execute(*s);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    { 
      RCLCPP_WARN(LOGGER, "Stage failed : %s", stage_failed_.c_str());
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      return false;
    }
  }

  return true;
}

bool MTCTaskNode::doTask() {
  task_.clear();

  fillTask();

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  if (!task_.plan(1)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return false;
  }

  if(!executeTask())
  {
    if (recovery_done_ < recovery_allowed_)
    {
      ++recovery_done_;
      RCLCPP_WARN(LOGGER, "Task Recovery %d out of %d", recovery_done_, recovery_allowed_);
      return true;
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "No more recovery possible");
      return false;
    }
  }

  RCLCPP_INFO(LOGGER, "Execution done");
  return false;
}

int main(int argc, char** argv) {
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
  bool value{true};
  do{
    value = mtc_task_node->doTask();
  } while(value);

  RCLCPP_INFO(LOGGER, "STOP");
  rclcpp::shutdown();
  spin_thread->join();
  RCLCPP_INFO(LOGGER, "STOP SPIN");
  return 0;
}