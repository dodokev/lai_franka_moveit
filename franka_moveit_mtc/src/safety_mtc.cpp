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

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
  public:
  enum class SHAPE {NONE, SPHERE, CYLINDER, BOX};
  
  MTCTaskNode(const rclcpp::NodeOptions &options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  double x_;
  double y_;
  double z_;

  double margin_{0.002};
  double hand_max{0.034};

  geometry_msgs::msg::Point position_;
  geometry_msgs::msg::Quaternion orientation_;

  SHAPE type_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
  position_.x = 0.5; 
  position_.y = -0.25;
  position_.z = 0.0;

  orientation_.x = 0;
  orientation_.y = 0;
  orientation_.z = 0;
  orientation_.w = 1;
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
  object.primitives[0].dimensions = {0.185, 0.029}; // smaller cylinder
  
  // object.primitives[0].dimensions = {0.06, 0.077, 0.284}; // box
  // object.primitives[0].dimensions = {0.284, 0.06, 0.077};
  
  type_ = static_cast<SHAPE>(object.primitives[0].dimensions.size());
  // type_ = static_cast<SHAPE>(2);

  if (type_ == SHAPE::CYLINDER)
  {
    // Cylinder Standing
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    z_ = object.primitives[0].dimensions[0];
    x_ = object.primitives[0].dimensions[1];
    // z_ = 0.206;
    // x_ = 0.034;
  }
  else if (type_ == SHAPE::BOX)
  {
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    x_ = object.primitives[0].dimensions[0];
    y_ = object.primitives[0].dimensions[1];
    z_ = object.primitives[0].dimensions[2];
  }

  geometry_msgs::msg::Pose pose;
  position_.z = z_/2 + margin_;
  pose.position = position_;

  pose.orientation = orientation_;
  
  object.primitive_poses.push_back(pose);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);

  // RCLCPP_INFO(LOGGER, "Create Object");
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(3))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  // task_.introspection().publishSolution(*task_.solutions().front());

  // Send to first sequence to execute

  // auto result = task_.execute(*task_.solutions().front());
  // if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  // {
  //   RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
  //   return;
  // }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto &arm_group_name = "fr3_arm";
  const auto &hand_group_name = "fr3_hand";
  const auto &hand_frame = "fr3_hand_tcp";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic pop

  if (!task.getRobotModel()) {
    RCLCPP_ERROR(LOGGER, "Robot model not loaded");
    return task;
  }

  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto cart_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "task");
  cart_planner->setMaxVelocityScalingFactor(0.1);
  cart_planner->setMaxAccelerationScalingFactor(0.1);

  auto star_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  star_planner->setMaxVelocityScalingFactor(0.1);
  star_planner->setMaxAccelerationScalingFactor(0.1);
  star_planner->setTimeout(10.0);
  star_planner->setPlannerId("RRTstarkConfigDefault");
  
  auto connect_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  connect_planner->setMaxVelocityScalingFactor(0.1);
  connect_planner->setMaxAccelerationScalingFactor(0.1);
  connect_planner->setPlannerId("RRTConnectkConfigDefault");

  auto pilz_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
  pilz_planner->setMaxVelocityScalingFactor(0.1);
  pilz_planner->setMaxAccelerationScalingFactor(0.1);
  pilz_planner->setPlannerId("PTP");

  auto multi_planner = std::make_shared<mtc::solvers::MultiPlanner>();
  multi_planner->push_back(cart_planner);
  multi_planner->push_back(pilz_planner);
  multi_planner->push_back(star_planner);
  multi_planner->push_back(connect_planner);

  for (const auto& planner : *multi_planner) {
  RCLCPP_INFO(LOGGER, "planner id = %s",
              planner->getPlannerId().c_str());
  }

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.15);
  cartesian_planner->setMaxAccelerationScalingFactor(0.15);
  cartesian_planner->setStepSize(.005);

  auto security_collision = [this](const moveit::task_constructor::SolutionBase& solution, std::string&){
    RCLCPP_WARN(LOGGER, "Clearance check ...");
    auto _scene = solution.end()->scene(); //->diff();
    const moveit::core::RobotState& _state = _scene->getCurrentState();

    const auto& _model = _scene->getRobotModel();

    std::set<const moveit::core::LinkModel*> _links;
    const moveit::core::JointModelGroup* joint_group = _model->getJointModelGroup("fr3_arm");

    const std::string& ee_name = joint_group->getLinkModelNames().back();
    if (const auto* lm = _model->getLinkModel(ee_name))
      _links.insert(lm);

    std::vector<const moveit::core::AttachedBody*> attached;
    _state.getAttachedBodies(attached);

    for (const auto* body : attached)
    {
      const std::string& parent = body->getAttachedLinkName();
      // Only include if the parent link belongs to our planning group
      if (joint_group->hasLinkModel(parent))
      {
        if(parent == "fr3_link0" || parent == "fr3_link1" || parent == "fr3_link2")
          continue;
        if (const auto* lm = _model->getLinkModel(parent))
          _links.insert(lm);
      }
    }

    collision_detection::DistanceRequest _req;
    _req.active_components_only = &_links;
    _req.type = collision_detection::DistanceRequestType::GLOBAL; 
    _req.enable_nearest_points = false; // Optional: Gets 3D coordinates
    _req.acm = &(solution.end()->scene()->getAllowedCollisionMatrix());

    collision_detection::DistanceResult _res;
    _scene->getCollisionEnv()->distanceRobot(_req, _res, _state);

    const double _min_dist = 0.02;

    if (_res.minimum_distance.distance < _min_dist)
    {
      // std::string link = _res.minimum_distance.link_names[0];
      // std::string obs = _res.minimum_distance.link_names[1];
      // double dist = _res.minimum_distance.distance;

      // RCLCPP_WARN_STREAM(LOGGER, "The closest obstacle is: " << obs);
      // RCLCPP_WARN_STREAM(LOGGER, "It is closest to robot link: " << link);
      // RCLCPP_WARN_STREAM(LOGGER, "Distance remaining: " << dist);
      return false;
    }

    const auto& _sub = dynamic_cast<const mtc::SubTrajectory&>(solution);
    const auto& _traj = _sub.trajectory();

    for (std::size_t i = 0; i < _traj->getWayPointCount(); ++i) {
      const auto& _st = _traj->getWayPoint(i);
      _scene->getCollisionEnv()->distanceRobot(_req, _res, _st);

      if (_res.minimum_distance.distance < _min_dist)
      {
        // std::string link = _res.minimum_distance.link_names[0];
        // std::string obs = _res.minimum_distance.link_names[1];
        // double dist = _res.minimum_distance.distance;

        // RCLCPP_WARN_STREAM(LOGGER, "The closest obstacle is: " << obs);
        // RCLCPP_WARN_STREAM(LOGGER, "It is closest to robot link: " << link);
        // RCLCPP_WARN_STREAM(LOGGER, "Distance remaining: " << dist);
        return false;
      }

      if(!_st.satisfiesBounds(0.005))
      {
        RCLCPP_WARN(LOGGER, "Not bounds");
        return false;
      }
    }

    return true;
  };

  // --------------------------------------------------------------------------------
  // Open the hand
  {
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));
  }
  // --------------------------------------------------------------------------------
  // Connect result stage before and after
  {
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick", mtc::stages::Connect::GroupPlannerVector{{arm_group_name, multi_planner}});
    // stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    stage_move_to_pick->setCostTerm(std::make_shared<mtc::cost::LinkMotion>("fr3_hand_tcp"));

    auto wrapper = std::make_unique<mtc::stages::PredicateFilter>("filter pick", std::move(stage_move_to_pick));
    wrapper->setPredicate(security_collision);

    task.add(std::move(wrapper));
    // task.add(std::move(stage_move_to_pick));
  }

  // --------------------------------------------------------------------------------
  mtc::Stage *attach_object_stage = nullptr; // Forward attach_object_stage to place pose generator

  // --------------------------------------------------------------------------------
  // PICK THE OBJECT
  {
    // Serial Container : groups stages
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto alternate = std::make_unique<mtc::Alternatives>("orientation variation");
      task.properties().exposeTo(alternate->properties(), {"eef", "group", "ik_frame"});
      alternate->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

      // --------------------------------------------------------------------------------
      // COMPUTE APPROACH AND IK FROM ABOVE
      {
        // ================================================================================
        // Sample grasp pose
        auto alt = std::make_unique<mtc::SerialContainer>("pick from above");
        alternate->properties().exposeTo(alt->properties(), {"eef", "group", "ik_frame"});
        alt->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose("open");
        stage->setObject("object");
        if (type_ == SHAPE::BOX)
          stage->setAngleDelta(M_PI / 2);
        else if (type_ == SHAPE::CYLINDER)
          stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr); // Hook into current state

        // ================================================================================
        // Frame for generation
        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();

        // IF USED THE FULL ALTERNATIVE USE THIS TRANSLATION
        // grasp_frame_transform.translation().x() = x_ - hand_max;
        grasp_frame_transform.translation().z() = z_/2 - hand_max + 0.01;


        // ================================================================================
        // Compute IK for generated grasp Pose
        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        
        // grasp->insert(std::move(wrapper));
        {
          // ================================================================================
          // Move relative to a position
          auto stage =
              std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
          stage->properties().set("marker_ns", "approach_object");
          stage->properties().set("link", hand_frame);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
          stage->setMinMaxDistance(0.1, 0.15);

          // ================================================================================
          // Set hand forward direction
          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = hand_frame;
          vec.vector.z = 1.0;
          stage->setDirection(vec);
          alt->add(std::move(stage));
        }

        alt->add(std::move(wrapper));
        alternate->add(std::move(alt));
      }

      // --------------------------------------------------------------------------------
      // COMPUTE APPROACH AND IK FROM BELOW
      {
        // ================================================================================
        // Sample grasp pose
        auto alt = std::make_unique<mtc::SerialContainer>("pick from below");
        alternate->properties().exposeTo(alt->properties(), {"eef", "group", "ik_frame"});
        alt->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose("open");
        stage->setObject("object");
        if (type_ == SHAPE::BOX)
          stage->setAngleDelta(M_PI / 2);
        else if (type_ == SHAPE::CYLINDER)
          stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr); // Hook into current state

        // ================================================================================
        // Frame for generation
        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q = Eigen::AngleAxisd(2*M_PI, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();

        // IF USED THE FULL ALTERNATIVE USE THIS TRANSLATION
        // grasp_frame_transform.translation().x() = x_ - hand_max;
        grasp_frame_transform.translation().z() = z_/2 - hand_max + 0.01;


        // ================================================================================
        // Compute IK for generated grasp Pose
        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        
        // grasp->insert(std::move(wrapper));
        {
          // ================================================================================
          // Move relative to a position
          auto stage =
              std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
          stage->properties().set("marker_ns", "approach_object");
          stage->properties().set("link", hand_frame);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
          stage->setMinMaxDistance(0.1, 0.15);

          // ================================================================================
          // Set hand forward direction
          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = hand_frame;
          vec.vector.z = 1.0;
          stage->setDirection(vec);
          alt->add(std::move(stage));
        }

        alt->add(std::move(wrapper));
        alternate->add(std::move(alt));
      }

      // --------------------------------------------------------------------------------
      // COMPUTE APPROACH AND IK FROM SIDE
      {
        // ================================================================================
        // Sample grasp pose
        auto alt = std::make_unique<mtc::SerialContainer>("pick frome side");
        alternate->properties().exposeTo(alt->properties(), {"eef", "group", "ik_frame"});
        alt->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose("open");
        stage->setObject("object");
        if (type_ == SHAPE::BOX)
          stage->setAngleDelta(M_PI / 2);
        else if (type_ == SHAPE::CYLINDER)
          stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr); // Hook into current state

        // ================================================================================
        // Frame for generation
        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI /2 , Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(M_PI /2, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(M_PI /2, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();
        grasp_frame_transform.translation().z() = -z_/2 + 2*hand_max + 2*0.0005 + std::max(x_, y_);
        grasp_frame_transform.translation().x() = -z_/4;

        // ================================================================================
        // Compute IK for generated grasp Pose
        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        
        {
          // ================================================================================
          // Move relative to a position
          auto stage =
              std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
          stage->properties().set("marker_ns", "approach_object");
          stage->properties().set("link", hand_frame);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
          stage->setMinMaxDistance(0.1, 0.15);

          // ================================================================================
          // Set hand forward direction
          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = hand_frame;
          vec.vector.z = 1.0;
          stage->setDirection(vec);
          alt->add(std::move(stage));
        }

        // grasp->insert(std::move(wrapper));
        alt->add(std::move(wrapper));
        alternate->add(std::move(alt));
      }

      grasp->insert(std::move(alternate));
    }

    {
      // ================================================================================
      // Enable collision of hand and object
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    {
      // ================================================================================
      // Use MoveTo to close the hand
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);

      // std::map<std::string, double> target;
      // target["fr3_finger_joint1"] = 0.03;  // half width
      // target["fr3_finger_joint2"] = 0.03;
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      // ================================================================================
      // Stage to modify planning scene, attach the objet to the hand frame (pick)
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // ================================================================================
      // Lift the object
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
    // END OF THE SERIAL CONTAINER
  }

  // --------------------------------------------------------------------------------
  // Connect pick and place stages
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>("move to place", mtc::stages::Connect::GroupPlannerVector{{arm_group_name, multi_planner}});
    // stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    stage_move_to_place->setCostTerm(std::make_shared<mtc::cost::LinkMotion>("fr3_hand_tcp"));
    
    auto wrapper = std::make_unique<mtc::stages::PredicateFilter>("filter pick", std::move(stage_move_to_place));
    wrapper->setPredicate(security_collision);

    task.add(std::move(wrapper));
    // task.add(std::move(stage_move_to_place));
  }

  // --------------------------------------------------------------------------------
  // Container for place sequence
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      // ================================================================================
      // drop the object
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("drop object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "drop_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    {
      // ================================================================================
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = position_.x;
      target_pose_msg.pose.position.y = 0.25;
      target_pose_msg.pose.position.z = position_.z; // + 0.15;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

      // ================================================================================
      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    {
      // ================================================================================
      // Open Hand (setGoal use state name in srdf)
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      // ================================================================================
      // Forbid collision now
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      place->insert(std::move(stage));
    }

    {
      // ================================================================================
      // Detach object from hand_frame
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
    // END OF THE CONTAINER
  }

  // --------------------------------------------------------------------------------
  // Return HOME
  {

    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", multi_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setCostTerm(std::make_shared<mtc::cost::LinkMotion>("fr3_hand_tcp"));
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  {
    // ================================================================================
    // Use MoveTo to close the hand
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
                                                   {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}