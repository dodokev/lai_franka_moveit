/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael Goerner, Robert Haschke
   Desc:    generate and validate a straight-line Cartesian path
*/

#include <moveit/task_constructor/solvers/multi_planner.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <chrono>

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

namespace moveit {
namespace task_constructor {
namespace solvers {

void MultiPlanner::init(const core::RobotModelConstPtr& robot_model) {
	for (const auto& p : *this)
		p->init(robot_model);
}

bool cartesianDistance(Eigen::Vector3d& current_position_,
  Eigen::Vector3d& last_position_, double& cartesian_dist_)
{  
  cartesian_dist_ += (last_position_ - current_position_).norm();
  last_position_ = current_position_;

  return true;
}

bool jointDistance(int nb_active_joint_,
  const moveit::core::JointModelGroup* jmg_, const moveit::core::RobotState& st_,
  Eigen::VectorXd& last_joint_value_, Eigen::VectorXd& joint_dist_)
{
  Eigen::VectorXd current_joint_value;
  st_.copyJointGroupPositions(jmg_, current_joint_value);
  
  for (int i = 0; i < nb_active_joint_; i++)
    joint_dist_(i) += abs(last_joint_value_(i) - current_joint_value(i));

  last_joint_value_ = current_joint_value;

  return true;
}

PlannerInterface::Result MultiPlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                                            const planning_scene::PlanningSceneConstPtr& to,
                                            const moveit::core::JointModelGroup* jmg, double timeout,
                                            robot_trajectory::RobotTrajectoryPtr& result,
                                            const moveit_msgs::msg::Constraints& path_constraints) {
	double remaining_time = std::min(timeout, properties().get<double>("timeout"));
	auto start_time = std::chrono::steady_clock::now();

	std::vector<PlannerInterface::Result> _ress;
	std::vector<robot_trajectory::RobotTrajectory> _trajs;
	std::vector<std::size_t> _lens;
	std::vector<std::string> _ids;

	// std::string comment = "No planner specified";
	std::string comment = "No path found";
	
	// for (const auto& p : *this) {
	// 	if (remaining_time < 0)
	// 		return { false, "timeout" };
	// 	if (result)
	// 		result->clear();
	// 	auto r = p->plan(from, to, jmg, remaining_time, result, path_constraints);
	// 	if (r)
	// 		return r;
	// 	else
	// 		comment = r.message;

	// 	auto now = std::chrono::steady_clock::now();
	// 	remaining_time -= std::chrono::duration<double>(now - start_time).count();
	// 	start_time = now;
	// }
	// return { false, comment };
	
	for (const auto& p : *this) {
		auto r = p->plan(from, to, jmg, remaining_time, result, path_constraints);
		if (r)
		{
			_ress.push_back(r);
			_ids.push_back(p->getPlannerId());
			_trajs.push_back(*result);
			_lens.push_back(result->size());
		}
	}
	if (_lens.empty())
		return { false, comment };
	
	std::vector<double> _dists;
	std::vector<double> _joint_dists;

	Eigen::Vector3d last;
	Eigen::Vector3d current;
	
	Eigen::VectorXd last_joint;
	
	for (auto& t : _trajs)
	{
		double tmp_dist = 0;
		Eigen::VectorXd joint_dist(7);
		for (unsigned int TMP = 0; TMP < 7; TMP++)
    		joint_dist(TMP) = 0.0;
		
		for (std::size_t iWPt = 0; iWPt < t.size(); iWPt++)
		{
			const moveit::core::RobotState& st = t.getWayPoint(iWPt);
			auto tmp_pose = st.getGlobalLinkTransform("fr3_hand_tcp");

			if (!iWPt)
			{
				last(0) = tmp_pose.translation().x();
				last(1) = tmp_pose.translation().y();
				last(2) = tmp_pose.translation().z();
			}

			if (!iWPt)
      			st.copyJointGroupPositions(jmg, last_joint);

			current(0) = tmp_pose.translation().x();
			current(1) = tmp_pose.translation().y();
			current(2) = tmp_pose.translation().z();

			cartesianDistance(last, current, tmp_dist);
			jointDistance(7, jmg, st, last_joint, joint_dist);
		}
		_dists.push_back(tmp_dist);
		_joint_dists.push_back(joint_dist.maxCoeff());
	}

	for (std::size_t i = 0; i < _dists.size(); i++)
		RCLCPP_WARN(LOGGER, "Planner : %s | d = %f", _ids[i].c_str(), _dists[i]);

	auto min_t = std::min_element(_dists.begin(), _dists.end());
	int index = std::distance(_dists.begin(), min_t);

	RCLCPP_INFO(LOGGER, "Planner's path used : %s", _ids[index].c_str());

	result = std::make_shared<robot_trajectory::RobotTrajectory>(_trajs[index]);

	comment = _ress[index].message;
	return _ress[index];
}

PlannerInterface::Result MultiPlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                                            const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
                                            const Eigen::Isometry3d& target, const moveit::core::JointModelGroup* jmg,
                                            double timeout, robot_trajectory::RobotTrajectoryPtr& result,
                                            const moveit_msgs::msg::Constraints& path_constraints) {
	double remaining_time = std::min(timeout, properties().get<double>("timeout"));
	auto start_time = std::chrono::steady_clock::now();

	std::string comment = "No planner specified";
	for (const auto& p : *this) {
		if (remaining_time < 0)
			return { false, "timeout" };
		if (result)
			result->clear();
		auto r = p->plan(from, link, offset, target, jmg, remaining_time, result, path_constraints);
		if (r)
			return r;
		else
			comment = r.message;

		auto now = std::chrono::steady_clock::now();
		remaining_time -= std::chrono::duration<double>(now - start_time).count();
		start_time = now;
	}
	return { false, comment };
}
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
