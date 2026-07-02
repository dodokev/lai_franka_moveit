#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <xlsxwriter.h>

static auto const LOGGER = rclcpp::get_logger("test");

bool cartesianDistance(Eigen::Vector3d& current_position_,
  Eigen::Vector3d& last_position_, double& cartesian_dist_)
{  
  cartesian_dist_ += (last_position_ - current_position_).norm();
  last_position_ = current_position_;

  return true;
}

double jointTotalDistance(Eigen::VectorXd& joint_dist_)
{
  return joint_dist_.sum();
}

bool jointDistance(unsigned int& nb_active_joint_,
  const moveit::core::JointModelGroup* jmg_, const moveit::core::RobotState& st_,
  Eigen::VectorXd& last_joint_value_, Eigen::VectorXd& joint_dist_)
{
  Eigen::VectorXd current_joint_value;
  st_.copyJointGroupPositions(jmg_, current_joint_value);
  
  for (unsigned int i = 0; i < nb_active_joint_; i++)
    joint_dist_(i) += abs(last_joint_value_(i) - current_joint_value(i));

  last_joint_value_ = current_joint_value;

  return true;
}

bool dataComputation(robot_trajectory::RobotTrajectory& rt_, 
  const std::string& ee_link_,
  const moveit::core::JointModelGroup* jmg_,
  double& cartesian_dist, Eigen::VectorXd& joint_dist)
{
  // Just to be sure
  cartesian_dist = 0;

  unsigned int nb_active_joint = jmg_->getActiveVariableCount();
  for (unsigned int TMP = 0; TMP < nb_active_joint; TMP++)
    joint_dist(TMP) = 0.0;

  Eigen::Vector3d last_position(0.0, 0.0, 0.0);
  Eigen::VectorXd last_joint_value;


  Eigen::Vector3d current_position;

  std::size_t n_wp = rt_.getWayPointCount();
  for (std::size_t iWPt = 0; iWPt < n_wp; iWPt++)
  {
    const moveit::core::RobotState& st = rt_.getWayPoint(iWPt);

    auto tmp_pose = st.getGlobalLinkTransform(ee_link_);
    if (!iWPt)
    {
      last_position(0) = tmp_pose.translation().x();
      last_position(1) = tmp_pose.translation().y();
      last_position(2) = tmp_pose.translation().z();
    }

    if (!iWPt)
      st.copyJointGroupPositions(jmg_, last_joint_value);

    current_position(0) = tmp_pose.translation().x();
    current_position(1) = tmp_pose.translation().y();
    current_position(2) = tmp_pose.translation().z();

    cartesianDistance(current_position, last_position, cartesian_dist);
    jointDistance(nb_active_joint, jmg_, st, last_joint_value, joint_dist);
  }

  RCLCPP_DEBUG(LOGGER, "Cartesian Distance : %f", cartesian_dist);
  RCLCPP_DEBUG_STREAM(LOGGER, "Joint Distance :\n" << joint_dist << "\n");
  
  return true;
}

void saveDataExperiment(std::vector<std::string>* t_plannerId_, std::vector<geometry_msgs::msg::Pose>* t_pts_, unsigned int nb_runs_,
  std::vector<double>* t_cart_dist_, std::vector<double>* t_Tjoint_dist_,
  std::vector<double>* t_time_, std::vector<unsigned int>* t_success_)
{
  lxw_workbook* workbook  = workbook_new("data.xlsx");
  lxw_worksheet* worksheet = workbook_add_worksheet(workbook, "DATA");

  std::size_t nb_stat = 3;
  std::size_t nb_planner = t_plannerId_->size();
  std::size_t nb_pts = t_pts_->size();

  // --------------------------------------------------------------------------------
  // POINT COORDINATE - ORIENTATION
  
  worksheet_write_string(worksheet, 0, 0, "Point", NULL);
  worksheet_write_string(worksheet, 0, 1, "3D-x", NULL);
  worksheet_write_string(worksheet, 0, 2, "3D-y", NULL);
  worksheet_write_string(worksheet, 0, 3, "3D-z", NULL);
  worksheet_write_string(worksheet, 0, 4, "Quat-x", NULL);
  worksheet_write_string(worksheet, 0, 5, "Quat-y", NULL);
  worksheet_write_string(worksheet, 0, 6, "Quat-z", NULL);
  worksheet_write_string(worksheet, 0, 7, "Quat-w", NULL);
  
  for (std::size_t iPt = 0; iPt < nb_pts; iPt++)
  {
    std::string pt_string("Point_" + std::to_string(iPt + 1));
    double x = t_pts_->at(iPt).position.x;
    double y = t_pts_->at(iPt).position.y;
    double z = t_pts_->at(iPt).position.z;
    
    double i = t_pts_->at(iPt).orientation.x;
    double j = t_pts_->at(iPt).orientation.y;
    double k = t_pts_->at(iPt).orientation.z;
    double w = t_pts_->at(iPt).orientation.w;
    
    worksheet_write_string(worksheet, iPt+1, 0, pt_string.c_str(), NULL);
    worksheet_write_number(worksheet, iPt+1, 1, x, NULL);
    worksheet_write_number(worksheet, iPt+1, 2, y, NULL);
    worksheet_write_number(worksheet, iPt+1, 3, z, NULL);
    worksheet_write_number(worksheet, iPt+1, 4, i, NULL);
    worksheet_write_number(worksheet, iPt+1, 5, j, NULL);
    worksheet_write_number(worksheet, iPt+1, 6, k, NULL);
    worksheet_write_number(worksheet, iPt+1, 7, w, NULL);
  }

  // --------------------------------------------------------------------------------
  // Values
  
  for (std::size_t iStat = 0; iStat < nb_stat; iStat++)
  {
    auto iCol = iStat * (nb_stat + nb_planner) - iStat;
    worksheet_write_string(worksheet, nb_pts+2, iCol, "Goal", NULL);
 
    unsigned int start_index = 0;
    for (std::size_t iPt = 0; iPt < nb_pts; iPt++)
    {
      std::string goal_string("Goal_" + std::to_string(iPt + 1));

      for (std::size_t iPlanner = 0; iPlanner < nb_planner; iPlanner++)
      {
        auto success_index = iPt * nb_planner + iPlanner;

        auto nb_value = t_success_->at(success_index);
        auto end_index = start_index + nb_value;

        auto plannerId = t_plannerId_->at(iPlanner).c_str();

        worksheet_write_string(worksheet, nb_pts+2, iCol + iPlanner + 1, plannerId, NULL);

        // For each planner write every values
        for (std::size_t iValue = start_index; iValue < end_index; iValue++)
        {
          auto iRow = iPt * nb_runs_ + iValue - start_index + nb_pts + 3;
          switch (iStat)
          {
            case 0:
              worksheet_write_number(worksheet, iRow, iCol + iPlanner + 1, t_time_->at(iValue), NULL);
              break;
            case 1:
              worksheet_write_number(worksheet, iRow, iCol + iPlanner + 1, t_cart_dist_->at(iValue), NULL);
              break;
            case 2:
              worksheet_write_number(worksheet, iRow, iCol + iPlanner + 1, t_Tjoint_dist_->at(iValue), NULL);
              break;
            default:
            break;
          }
        }

        start_index = end_index;
      }

      // --------------------------------------------------------------------------------
      // Write goal name
      for (size_t i = 0; i < nb_runs_; i++)
      {
        auto row = iPt * nb_runs_ + i + nb_pts + 3;
        worksheet_write_string(worksheet, row, iCol, goal_string.c_str(), NULL);
      }
    }
  }

  RCLCPP_INFO(LOGGER, "DATA SAVED ....");
  workbook_close(workbook);
}

double mean_vector(std::vector<double>* v_)
{
  double value = 0;
  for (std::size_t i = 0; i < v_->size(); i++)
    value += v_->at(i);

  return value / v_->size();
}

void runPlanner(unsigned int nb_runs_, moveit::planning_interface::MoveGroupInterface* mg_,
  const moveit::core::JointModelGroup* jmg_, const std::string& ee_link_,
  std::vector<double>* t_cart_dist_, std::vector<double>* t_Tjoint_dist_,
  std::vector<double>* t_time_, std::vector<unsigned int>* t_success_)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr robot_state = mg_->getCurrentState();

  unsigned int success = 0;
  double cartesian_dist;

  unsigned int nb_active_joint = jmg_->getActiveVariableCount();
  Eigen::VectorXd joint_dist(nb_active_joint);

  bool pathFound = false;
  unsigned int iter = 0;
  do
  {
    ++iter;
    pathFound = mg_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;

    if (pathFound)
    {
      ++success;
      
      robot_trajectory::RobotTrajectory rt(mg_->getRobotModel(), mg_->getName());
      rt.setRobotTrajectoryMsg(*robot_state, plan.trajectory_);

      dataComputation(rt, ee_link_, jmg_, cartesian_dist, joint_dist);
    
      t_cart_dist_->push_back(cartesian_dist);
      t_Tjoint_dist_->push_back(joint_dist.sum());
      t_time_->push_back(plan.planning_time_);
    }
  } while (iter < nb_runs_);

  t_success_->push_back(success);
}

void runAllPlanner(unsigned int nb_runs_, std::vector<std::string>* t_plannerId_,
  moveit::planning_interface::MoveGroupInterface* mg_,
  const moveit::core::JointModelGroup* jmg_, const std::string& ee_link_,
  std::vector<double>* t_cart_dist_, std::vector<double>* t_Tjoint_dist_,
  std::vector<double>* t_time_, std::vector<unsigned int>* t_success_)
{
  for (std::size_t i = 0; i < t_plannerId_->size(); i++)
  {
    mg_->setPlannerId(t_plannerId_->at(i));
    runPlanner(nb_runs_, mg_, jmg_, ee_link_, t_cart_dist_, t_Tjoint_dist_, t_time_, t_success_);
  }
}

void runExperiment(unsigned int nb_runs_, std::vector<geometry_msgs::msg::Pose>* t_pts_,
  std::vector<std::string>* t_plannerId_,
  moveit::planning_interface::MoveGroupInterface* mg_,
  const moveit::core::JointModelGroup* jmg_, const std::string& ee_link_,
  std::vector<double>* t_cart_dist_, std::vector<double>* t_Tjoint_dist_,
  std::vector<double>* t_time_, std::vector<unsigned int>* t_success_)
{
  for (std::size_t i = 0; i < t_pts_->size(); i++)
  {
    mg_->setPoseTarget(t_pts_->at(i));
    runAllPlanner(nb_runs_, t_plannerId_, mg_, jmg_, ee_link_, t_cart_dist_, t_Tjoint_dist_, t_time_, t_success_);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // ================================================================================

  const std::string plannerGroup = "fr3_arm";
  const std::string ee_link = "fr3_hand_tcp";

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, plannerGroup);
  move_group.startStateMonitor();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
  const moveit::core::JointModelGroup *joint_model = robot_state->getJointModelGroup(plannerGroup);

  namespace rvt = rviz_visual_tools;
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // ================================================================================
  
  geometry_msgs::msg::Pose start_pose;
  start_pose = move_group.getCurrentPose().pose;
  move_group.setStartStateToCurrentState();

  // start_pose.position.x = 0.3071037828922272;
  // start_pose.position.y = -0.141447052359581;
  // start_pose.position.z = 0.4798051714897156;
  
  // start_pose.orientation.x = 0.9999999403953552;
  // start_pose.orientation.y = 0.0003082542971242219;
  // start_pose.orientation.z = -7.609510066686198e-05;
  // start_pose.orientation.w = -1.9570914446376264e-05;
  
  bool foundIK = robot_state->setFromIK(joint_model, start_pose);
  if (!foundIK)
    RCLCPP_ERROR(LOGGER, "Not joint configuration found ...");
  else
  {
    robot_state->update();
    move_group.setStartState(*robot_state);
    RCLCPP_INFO(LOGGER, "Start pose done ...");
  }

  // ================================================================================
  
  // geometry_msgs::msg::Pose p1, p2, p3, p4, p5;
  // p1 = start_pose;
  // p2 = start_pose;
  // p3 = start_pose;
  // p4 = start_pose;
  // p5 = start_pose;

  // // FrontHighObstacle
  // p1.position.x = 0.4799172580242157;
  // p1.position.y = 0.1604018211364746;
  // p1.position.z = 0.4799172580242157;
  
  // // RightLowObstacle
  // p2.position.x = 0.41365736722946167;
  // p2.position.y = 0.42433586716651917;
  // p2.position.z = 0.023705018684267998;
  
  // // AboveBox
  // p3.position.x = 0.0359041653573513;
  // p3.position.y = -0.2730054259300232;
  // p3.position.z = 0.07460930198431015;
  
  // // BehindLowMetal
  // p4.position.x = -0.06198626384139061;
  // p4.position.y = -0.5353711247444153;
  // p4.position.z = 0.07460089027881622;

  // // LowStart
  // p5.position.x = 0.3071461319923401;
  // p5.position.y = -0.2932298183441162;
  // p5.position.z = 0.03889982029795647;
  
  // // FrontHighObstacle
  // p1.position.x = 0.63645;
  // p1.position.y = 0.21405;
  // p1.position.z = 0.47991;
  
  // // RightLowObstacle
  // p2.position.x = 0.53582;
  // p2.position.y = -0.38445;
  // p2.position.z = 0.14078;
  
  // // Low
  // p3.position.x = 0.40746;
  // p3.position.y = -0.045378;
  // p3.position.z = 0.023722;

  std::vector<geometry_msgs::msg::Pose> tab_pts;
  // tab_pts.push_back(p1);
  // tab_pts.push_back(p2);
  // tab_pts.push_back(p3);
  // tab_pts.push_back(p4);
  // tab_pts.push_back(p5);
  
  // ================================================================================
  
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setPlanningPipelineId("ompl");
  
  std::vector<std::string> tab_plannerId;
  tab_plannerId.push_back("RRTConnectkConfigDefault");
  tab_plannerId.push_back("KPIECEkConfigDefault");
  tab_plannerId.push_back("RRTstarkConfigDefault");
  tab_plannerId.push_back("BFMTkConfigDefault");

  std::vector<unsigned int> tab_success;
  std::vector<double> tab_cartesian_dist;
  std::vector<double> tab_total_joint_dist;
  std::vector<double> tab_time;

  unsigned int nb_runs = 100;

  runExperiment(nb_runs, &tab_pts, &tab_plannerId, &move_group, joint_model, ee_link,
    &tab_cartesian_dist, &tab_total_joint_dist, &tab_time, &tab_success);

  saveDataExperiment(&tab_plannerId, &tab_pts, nb_runs, &tab_cartesian_dist,
    &tab_total_joint_dist, &tab_time, &tab_success);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}