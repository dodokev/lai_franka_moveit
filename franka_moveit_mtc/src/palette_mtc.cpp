#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

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
#include "franka_moveit_msg/srv/enable_create.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;
using namespace std::chrono_literals;

/**
 * FIND THE FUCKING SEGEMTATIONFAULT, WHY !
 */

class MTCTaskNode {
public:
    enum class SHAPE { NONE, SPHERE, CYLINDER, BOX };

    MTCTaskNode(const rclcpp::NodeOptions& options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
    void setupPlanner();
    void setupPlanningScene();
    void palette();
    
private:
    rclcpp::Client<franka_moveit_msg::srv::EnableCreate>::SharedPtr client_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool grasped_{false};
    bool moved_{false};
    double threshold_{0.05};
    void checkObjectPosition();
    void setupObjectPose(std::string& object_name);
    bool setupObjectInformation(std::string& object_name);
    
    // Compose an MTC task from a series of stages.
    void createPickTask(std::string& object_name);
    void createPlaceTask(std::string& object_name, mtc::Stage* monitored, geometry_msgs::msg::PoseStamped& place);
    
    bool doTask(std::string& object_name, geometry_msgs::msg::PoseStamped& place);
    void fillTask(std::string& object_name, geometry_msgs::msg::PoseStamped& place);
    bool executeTask();

    void addCurrentStage();
    void returnHome();
    void addLiftStage(double min, double max);

    bool sendRequest(bool req);

    mtc::Stage* current_state_ptr_{nullptr};
    mtc::Stage* attach_stage_ptr_{nullptr};

    std::function<bool(const mtc::SolutionBase&, std::string&)> makeClearancePredicate();

    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
    std::shared_ptr<mtc::solvers::MultiPlanner> multipipeline_planner_;

    rclcpp::Node::SharedPtr node_;

    double voxel_size_{0.02};
    double hand_max_{0.034};

    mtc::Task task_;
    moveit::core::RobotModelConstPtr model_;

    std::string arm_group_name_;
    std::string hand_group_name_;
    std::string hand_frame_;

    std::string stage_failed_{""};
    unsigned int recovery_allowed_{2};
    unsigned int recovery_done_{0};

    // -- Object
    SHAPE type_{SHAPE::NONE};
    std::string current_obj_;
    geometry_msgs::msg::Pose object_pose_;
    std::vector<double> dimension_;

    double x_{0};
    double y_{0};
    double z_{0};
    // ---

    moveit_msgs::msg::Constraints constraints_;

    int object_number_;
    std::pair<int, int> slot_;
    std::pair<double, double> first_pose_;
    double rotation_;
    double offset_;
};

void MTCTaskNode::setupPlanningScene() {
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject obj_msg;
    moveit_msgs::msg::CollisionObject obj_msg_2;
    
    obj_msg.id = "object0";
    obj_msg.header.frame_id = "base";
    obj_msg.primitives.resize(1);
    obj_msg.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    obj_msg.primitives[0].dimensions.resize(2);

    obj_msg.primitives[0].dimensions[0] = 0.185; // height
    obj_msg.primitives[0].dimensions[1] = 0.029; // radius
    obj_msg.operation = obj_msg.ADD;

    obj_msg_2 = obj_msg;
    
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;

    pose.position.x = 0.3;
    pose.position.z = 0.185;
    // pose.position.x = 0.5;
    // pose.position.y = -0.25;
    // pose.position.z = obj_msg.primitives[0].dimensions[0]/2 + voxel_size_/2;

    // 9.5x6
    obj_msg.primitive_poses.resize(1);
    obj_msg.primitive_poses[0] = pose;
    psi.applyCollisionObject(obj_msg);

    obj_msg_2.id = "object1";
    pose.position.x = 0.35;
    pose.position.y = 0.1;

    // 14.5x3
    // obj_msg_2.primitive_poses.resize(1);
    // obj_msg_2.primitive_poses[0] = pose;
    // psi.applyCollisionObject(obj_msg_2);

    task_.setRobotModel(model_);

    task_.setProperty("group", arm_group_name_);
    task_.setProperty("eef", hand_group_name_);
    task_.setProperty("ik_frame", hand_frame_);
    task_.setProperty("path_constraints", constraints_);

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task_.add(std::move(stage_state_current));

    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attachObject");
    stage->attachObject("object0", hand_frame_);
    task_.add(std::move(stage));

    task_.init();
    task_.plan(1);

    auto solution = task_.solutions().front();
    auto result = task_.execute(*solution);
}

bool MTCTaskNode::sendRequest(bool req) {
    auto request = std::make_shared<franka_moveit_msg::srv::EnableCreate::Request>();
    request->enable = req;

    auto result = client_->async_send_request(request);
    while (rclcpp::ok()) {
        auto status = result.wait_for(std::chrono::milliseconds(100));
        if (status == std::future_status::ready) {
            auto response = result.get();
            return response->success;
            // break;
        }
    }

    return false;
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)} {
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(500),
                                        std::bind(&MTCTaskNode::checkObjectPosition, this));

    arm_group_name_ = "fr3_arm";
    hand_group_name_ = "fr3_hand";
    hand_frame_ = "fr3_hand_tcp";

    task_.loadRobotModel(node_);
    model_ = task_.getRobotModel();
    client_ = node_->create_client<franka_moveit_msg::srv::EnableCreate>("enable_create");

    // node_->declare_parameter<std::string>("nb_obj");
    // node_->declare_parameter<std::string>("slot");
    // node_->declare_parameter<std::string>("edge");
    // node_->declare_parameter<std::string>("ang", "0");

    object_number_ = node_->get_parameter("nb_obj").as_int();
    std::string tmp_slot = node_->get_parameter("slot").as_string();
    std::string tmp_edge = node_->get_parameter("first_pose").as_string();
    rotation_ = node_->get_parameter("rotation").as_double();
    offset_ = node_->get_parameter("offset").as_double();

    std::vector<std::string> _param_split;
    boost::split(_param_split, tmp_slot, boost::is_any_of(","));
    slot_ = {std::stoi(_param_split[0]), std::stoi(_param_split[1])};
    boost::split(_param_split, tmp_edge, boost::is_any_of(","));
    first_pose_ = {std::stod(_param_split[0]), std::stod(_param_split[1])};

    // while (!client_->wait_for_service(1s)) {
    // if (!rclcpp::ok()) {
    //     RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
    // }
    // RCLCPP_INFO(LOGGER, "service not available, waiting again...");
    // }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface() {
    return node_->get_node_base_interface();
}

void MTCTaskNode::setupObjectPose(std::string& object_name)
{
    std::vector<std::string> names{object_name};
    moveit::planning_interface::PlanningSceneInterface psi;
    while (rclcpp::ok()) {
        auto pose_map = psi.getObjectPoses(names);
        if (!pose_map.empty()) {
            object_pose_ = pose_map.begin()->second;
            return;
        }
        rclcpp::sleep_for(100ms);
    }
}

bool MTCTaskNode::setupObjectInformation(std::string& object_name) {
    std::vector<std::string> names{object_name};
     moveit::planning_interface::PlanningSceneInterface psi;
    auto obj_map = psi.getObjects(names);
    if (obj_map.empty())
    {
        RCLCPP_ERROR(LOGGER, "Object doesn't exist");
        return false;
    }

    for (const auto& p : obj_map)
    {
        auto& obj_msg = p.second;
        object_pose_ = obj_msg.pose;
        
        if (obj_msg.primitives.empty())
        {
            RCLCPP_ERROR(LOGGER, "No primitives");
            return false;
        }

        auto dims_tmp = obj_msg.primitives[0].dimensions;
        for (const auto& dim : dims_tmp)
            dimension_.push_back(dim);
        type_ = static_cast<SHAPE>(dimension_.size());
    }

    if (type_ == SHAPE::CYLINDER) {
        z_ = dimension_[0];
        x_ = y_ = dimension_[1];
    }
    else if (type_ == SHAPE::BOX) {
        /**
         * REVIEW WHICH IS WHAT
         */
        z_ = dimension_[0];
        y_ = dimension_[1];
        x_ = dimension_[2];
    }

    sendRequest(true);
    setupObjectPose(object_name);
    return true;
}

void MTCTaskNode::setupPlanner() {
    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = "world";
    oc.link_name = "fr3_hand_tcp";

    oc.orientation.x = 0.0;
    oc.orientation.y = 1.0;
    oc.orientation.z = 0.0;
    oc.orientation.w = 0.0;

    oc.absolute_x_axis_tolerance = 0.785;
    oc.absolute_y_axis_tolerance = 0.785;
    oc.absolute_z_axis_tolerance = M_PI;  // free rotation around tool axis

    oc.weight = 1.0;

    constraints_.orientation_constraints.push_back(oc);

    auto task_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "task");
    task_planner->setMaxVelocityScalingFactor(0.2);
    task_planner->setMaxAccelerationScalingFactor(0.2);

    auto rrtstar_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    rrtstar_planner->setMaxVelocityScalingFactor(0.2);
    rrtstar_planner->setMaxAccelerationScalingFactor(0.2);
    rrtstar_planner->setPlannerId("RRTstarkConfigDefault");

    auto rrtconnect_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    rrtconnect_planner->setMaxVelocityScalingFactor(0.2);
    rrtconnect_planner->setMaxAccelerationScalingFactor(0.2);
    rrtconnect_planner->setPlannerId("RRTConnectkConfigDefault");

    auto lin_planner =
        std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
    lin_planner->setMaxVelocityScalingFactor(0.2);
    lin_planner->setMaxAccelerationScalingFactor(0.2);
    lin_planner->setPlannerId("LIN");

    multipipeline_planner_ = std::make_shared<mtc::solvers::MultiPlanner>();
    // multipipeline_planner_->push_back(lin_planner);
    multipipeline_planner_->push_back(task_planner);
    multipipeline_planner_->push_back(rrtstar_planner);
    // multipipeline_planner_->push_back(rrtconnect_planner);

    interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner_->setMaxVelocityScalingFactor(0.05);
    cartesian_planner_->setMaxAccelerationScalingFactor(0.05);
    cartesian_planner_->setStepSize(.005);
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
    task_.setProperty("path_constraints", constraints_);

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr_ = stage_state_current.get();
    task_.add(std::move(stage_state_current));
}

void MTCTaskNode::createPickTask(std::string& object_name) {
    RCLCPP_ERROR(LOGGER, "current ptr : %s", current_state_ptr_->name().c_str());
    RCLCPP_ERROR(LOGGER, "hand : %s", hand_frame_.c_str());
    RCLCPP_ERROR(LOGGER, "obj : %s", object_name.c_str());

    task_.setRobotModel(model_);

    task_.setProperty("group", arm_group_name_);
    task_.setProperty("eef", hand_group_name_);
    task_.setProperty("ik_frame", hand_frame_);
    task_.setProperty("path_constraints", constraints_);

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
        stage->setTimeout(10.0);
        stage->setPathConstraints(constraints_);

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
            {"pickAbove", Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()), z_ / 2 - hand_max_, 0.0},
            {"pickBelow", Eigen::AngleAxisd(2 * M_PI, Eigen::Vector3d::UnitX()), z_ / 2 - hand_max_, 0.0},
        };

        for (const auto& approach : approaches) {
            auto alt = std::make_unique<mtc::SerialContainer>(approach.name);
            alternate->properties().exposeTo(alt->properties(), {"eef", "group", "ik_frame"});
            alt->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generationGrasp");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp");
            stage->setPreGraspPose("open");
            stage->setObject(object_name);
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(current_state_ptr_);

            Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
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
                auto approach_stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner_);
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
            stage->setObject(object_name);
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
                -z_ / 2 + 2 * hand_max_ + 2 * 0.0005 + std::max(x_, y_);
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

    task_.add(std::move(serial));

    {
        auto container = std::make_unique<mtc::SerialContainer>("pickObject");
        task_.properties().exposeTo(container->properties(), {"eef", "group", "ik_frame"});
        container->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allowCollision");
            stage->allowCollisions(object_name,
                                    task_.getRobotModel()
                                        ->getJointModelGroup(hand_group_name_)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    true);
            container->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("closeHand", interpolation_planner_);
            stage->setGroup(hand_group_name_);
            stage->setGoal("grasp");
            container->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attachObject");
            stage->attachObject(object_name, hand_frame_);
            attach_stage_ptr_ = stage.get();
            container->insert(std::move(stage));
        }

        task_.add(std::move(container));
    }
}

void MTCTaskNode::createPlaceTask(std::string& object_name, mtc::Stage* monitored, geometry_msgs::msg::PoseStamped& place) {
    task_.setRobotModel(model_);

    task_.setProperty("group", arm_group_name_);
    task_.setProperty("eef", hand_group_name_);
    task_.setProperty("ik_frame", hand_frame_);
    task_.setProperty("path_constraints", constraints_);

    auto clearance_predicate = makeClearancePredicate();

    auto serial = std::make_unique<mtc::SerialContainer>("PlaceTask");
    task_.properties().exposeTo(serial->properties(), {"eef", "group", "ik_frame"});
    serial->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
            "CONNECT",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name_, multipipeline_planner_}});
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
        stage_move_to_place->setTimeout(10.0);
        stage_move_to_place->setPathConstraints(constraints_);

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
            stage->setObject(object_name);

            stage->setPose(place);
            stage->setMonitoredStage(monitored);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("poseIK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(object_name);
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
            stage->allowCollisions(object_name,
                                    task_.getRobotModel()
                                        ->getJointModelGroup(hand_group_name_)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    false);
            container->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detachObject");
            stage->detachObject(object_name, hand_frame_);
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

    task_.add(std::move(serial));
}

void MTCTaskNode::addLiftStage(double min, double max) {
    task_.setRobotModel(model_);

    task_.setProperty("group", arm_group_name_);
    task_.setProperty("eef", hand_group_name_);
    task_.setProperty("ik_frame", hand_frame_);
    task_.setProperty("path_constraints", constraints_);

    auto stage = std::make_unique<mtc::stages::MoveRelative>("liftObject", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(min, max);
    stage->setIKFrame(hand_frame_);
    stage->properties().set("marker_ns", "lift");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task_.add(std::move(stage));
}

void MTCTaskNode::returnHome() {
    task_.clear();
    addCurrentStage();

    task_.setRobotModel(model_);

    task_.setProperty("group", arm_group_name_);
    task_.setProperty("eef", hand_group_name_);
    task_.setProperty("ik_frame", hand_frame_);
    task_.setProperty("path_constraints", constraints_);

    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("returnHome", multipipeline_planner_);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setGoal("ready");
        task_.add(std::move(stage));
    }

    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("openHand", interpolation_planner_);
        stage->setGroup(hand_group_name_);
        stage->setGoal("open");
        task_.add(std::move(stage));
    }

    try {
        task_.init();
    }
    catch (mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, e);
    }

    try {
        task_.plan(1);
    }
    catch (mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, e);
    }

    auto solution = task_.solutions().front();
    task_.execute(*solution);
}

void MTCTaskNode::fillTask(std::string& object_name, geometry_msgs::msg::PoseStamped& place) {
    addCurrentStage();
    if (stage_failed_ == "PickTask" || stage_failed_ == "pickObject" || stage_failed_ == "") {
        grasped_ = false;
        createPickTask(object_name);
        addLiftStage(0.05, 0.15);
        createPlaceTask(object_name, attach_stage_ptr_, place);
    } else if (stage_failed_ == "liftObject") {
        addLiftStage(0.05, 0.15);
        createPlaceTask(object_name, current_state_ptr_, place);
    } else if (stage_failed_ == "PlaceTask") {
        createPlaceTask(object_name, current_state_ptr_, place);
    }
}

Eigen::Vector3d getPosition(geometry_msgs::msg::Pose p) {
    return Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
}

void MTCTaskNode::checkObjectPosition() {
    /**
     * Add Prtoection - if no object_pose_
     */
    Eigen::Vector3d tmp_position(object_pose_.position.x, object_pose_.position.y, object_pose_.position.z);
    if (tmp_position == Eigen::Vector3d::Zero())
        return;

    if (!grasped_) {
        std::vector<std::string> names{current_obj_};
        moveit::planning_interface::PlanningSceneInterface psi;
        auto pose_map = psi.getObjectPoses(names);

        geometry_msgs::msg::Pose current_pose;

        for (const auto& p : pose_map)
        {
            current_pose = p.second;
            
            Eigen::Vector3d old_position = getPosition(object_pose_);
            Eigen::Vector3d current_position = getPosition(current_pose);

            double distance = (old_position - current_position).norm();
            // RCLCPP_WARN(LOGGER, "Distance : %f", distance);

            if (distance > threshold_ && !moved_) {
                RCLCPP_WARN(LOGGER, "Object Position deviates too much");
                RCLCPP_WARN(LOGGER, "Stop Execution ...");

                task_.preempt();
                moved_ = true;
                break;
            }
        }
    }
}

bool MTCTaskNode::executeTask() {
    auto solution = task_.solutions().front();

    auto container = task_.stages();
    std::vector<std::string> stage_names;
    RCLCPP_WARN(LOGGER, "NB Stage : %ld", container->numChildren());
    for (std::size_t i = 0; i < container->numChildren(); i++) {
        RCLCPP_INFO(LOGGER, "Stage %ld name : %s", i, ((*container)[i]->name()).c_str());
        stage_names.push_back((*container)[i]->name());
    }

    const auto* seq = dynamic_cast<const mtc::SolutionSequence*>(solution.get());
    for (auto* s : seq->solutions()) {
        const auto* c = s->creator();
        stage_failed_ = c->name();
        RCLCPP_WARN(LOGGER, "Stage executing : %s", stage_failed_.c_str());

        if (stage_failed_ == "pickObject")
          sendRequest(false);

        // ===============================================================================================================
        // --- Execution Code ---
        auto result = task_.execute(*s);
        RCLCPP_ERROR(LOGGER, "MoveIt error code: %d", result.val);
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_WARN(LOGGER, "Stage failed : %s", stage_failed_.c_str());
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return false;
        }
        // ===============================================================================================================

        if (stage_failed_ == "PlaceTask")
          sendRequest(true);
        if (stage_failed_ == "pickObject")
        grasped_ = true;
    }

    return true;
}

bool MTCTaskNode::doTask(std::string& object_name, geometry_msgs::msg::PoseStamped& place) {
    // task_ = mtc::Task();
    task_.clear();

    RCLCPP_WARN(LOGGER, "FIlling");
    fillTask(object_name, place);

    RCLCPP_WARN(LOGGER, "Init");
    try {
        task_.init();
    } catch (mtc::InitStageException& e) {
        RCLCPP_ERROR(LOGGER, "Init Failed");
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return false;
    }

    RCLCPP_WARN(LOGGER, "Plan");
    try 
    {
        if(!task_.plan(1))
        {
            task_.printState();
            RCLCPP_WARN(LOGGER, "Plan Failed, Retry ...");
            return true;
        }
        RCLCPP_WARN(LOGGER, "Plan finish");
        if (moved_)
        {
            RCLCPP_WARN(LOGGER, "Object Moved");
            rclcpp::sleep_for(std::chrono::milliseconds(2000));
            setupObjectPose(object_name);
            moved_ = false;
            return true;
        }
    }
    catch (mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return false;
    }

    RCLCPP_WARN(LOGGER, "Introspection");
    task_.introspection().publishSolution(*task_.solutions().front());

    RCLCPP_WARN(LOGGER, "Execute");
    // if (!executeTask()) {
    //     if (moved_) {
    //         RCLCPP_WARN(LOGGER, "Object Moved");
    //         rclcpp::sleep_for(std::chrono::milliseconds(2000));
    //         setupObjectPose(object_name);
    //         moved_ = false;
    //         return true;
    //     } else if (recovery_done_ < recovery_allowed_) {
    //         ++recovery_done_;
    //         RCLCPP_WARN(LOGGER, "Task Recovery %d out of %d", recovery_done_, recovery_allowed_);
    //         return true;
    //     } else {
    //         RCLCPP_ERROR(LOGGER, "No more recovery possible");
    //         return false;
    //     }
    // }

    RCLCPP_INFO(LOGGER, "Execution done");
    return false;
}

void MTCTaskNode::palette()
{
    std::string name_base{"object"};
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "world";
    
    // Static orientation
    target.pose.orientation.x = 0.0;
    target.pose.orientation.y = 0.0;
    target.pose.orientation.z = 0.0;
    target.pose.orientation.w = 1.0;

    int c_r{0};
    int c_c{0};
    for (int counter = 0; counter < object_number_; counter++)
    {
        RCLCPP_WARN_STREAM(LOGGER, "Slot : " << c_r << "x" << c_c);
        stage_failed_ = "";
        std::string obj_name = name_base + std::to_string(counter);
        current_obj_ = obj_name;
        RCLCPP_WARN(LOGGER, "Current Object : %s", obj_name.c_str());    
    
        if(!setupObjectInformation(obj_name))
            return;
        
        setupObjectPose(obj_name);

        auto e_x = (2 * x_ + offset_) * c_r;
        auto e_y = (2 * y_ + offset_) * c_c;

        target.pose.position.x = first_pose_.first + cos(rotation_) * e_x - sin(rotation_) * e_y;
        target.pose.position.y = first_pose_.second + cos(rotation_) * e_y + sin(rotation_) * e_x;
        // RCLCPP_WARN_STREAM(LOGGER, "Position : " << target.pose.position.x << " | " << target.pose.position.y);
        target.pose.position.z = z_/2 + voxel_size_/2;

        bool replan{true};
        do {
            replan = doTask(obj_name, target);
        } while (replan);

        if (c_c != slot_.second)
            ++c_c;
        if (c_c == slot_.second)
        {
            c_c = 0;
            ++c_r;
        }
        if (c_r == slot_.first && (counter+1) < object_number_)
        {
            RCLCPP_ERROR(LOGGER, "No more spaces for objects");
            return;
        }

        RCLCPP_WARN(LOGGER, "Next Object");
    }

    // returnHome();
    RCLCPP_WARN(LOGGER, "Palette Finish");
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

    mtc_task_node->setupPlanningScene();
    // mtc_task_node->setupPlanner();
    // mtc_task_node->palette();

    RCLCPP_INFO(LOGGER, "STOP");
    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}