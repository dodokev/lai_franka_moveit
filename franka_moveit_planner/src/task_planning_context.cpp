#include "franka_moveit_planner/task_planning_context.hpp"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <optional>
#include <unordered_set>

static auto const LOGGER = rclcpp::get_logger("task_planner");

using Clock     = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Ms        = std::chrono::duration<double, std::milli>;

namespace task_planner
{

// ============================================================
// Constructor
// ============================================================
TaskPlanningContext::TaskPlanningContext(
    const std::string&                           name,
    const std::string&                           group,
    const moveit::core::RobotModelConstPtr&      model,
    const planning_scene::PlanningSceneConstPtr& scene)
  : PlanningContext(name, group)
  , planning_scene_(scene)
  , robot_model_(model)
{}

// ============================================================
// solve() — main entry point called by MoveIt
// ============================================================
bool TaskPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;

  const TimePoint t_total_start = Clock::now();

  joint_group_ = robot_model_->getJointModelGroup(getGroupName());
  if (!joint_group_)
  {
    RCLCPP_ERROR(LOGGER, "Joint model group '%s' not found.", getGroupName().c_str());
    return false;
  }

  // Build clearance_links_ + d_req_cache_ once — reused by every checkSegment call.
  buildDistanceRequest();

  Eigen::Vector3d start_pos, goal_pos;
  if (!getStartPosition(start_pos))
  {
    RCLCPP_ERROR(LOGGER, "Failed to resolve start EE position.");
    return false;
  }
  if (!getGoalPosition(goal_pos))
  {
    RCLCPP_ERROR(LOGGER, "Failed to resolve goal EE position.");
    return false;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Start EE: " << start_pos.transpose());
  RCLCPP_INFO_STREAM(LOGGER, "Goal  EE: " << goal_pos.transpose());

  // --- RRT ---
  const TimePoint t_rrt_start = Clock::now();
  auto cartesian_path = computeCartesianPath(start_pos, goal_pos);
  const double t_rrt_ms = Ms(Clock::now() - t_rrt_start).count();

  if (cartesian_path.empty())
  {
    RCLCPP_ERROR(LOGGER, "RRT failed to find a Cartesian path (%.1f ms).", t_rrt_ms);
    return false;
  }
  RCLCPP_INFO(LOGGER, "[RRT]      %.1f ms  →  %zu waypoints (incl. smoothing)",
              t_rrt_ms, cartesian_path.size());

  // --- IK ---
  const TimePoint t_ik_start = Clock::now();

  std::optional<moveit::core::RobotState> goal_state_opt;
  const bool is_joint_goal =
      !request_.goal_constraints.empty() &&
      !request_.goal_constraints[0].joint_constraints.empty() &&
       request_.goal_constraints[0].position_constraints.empty();

  if (is_joint_goal)
  {
    goal_state_opt = buildGoalState();
    RCLCPP_INFO(LOGGER, "Joint-space goal detected — injecting exact state at last waypoint.");
  }

  std::vector<moveit::core::RobotState> joint_path;
  const moveit::core::RobotState* goal_override =
      goal_state_opt.has_value() ? &goal_state_opt.value() : nullptr;

  if (!computeIKPath(cartesian_path, joint_path, goal_override))
  {
    const double t_ik_ms = Ms(Clock::now() - t_ik_start).count();
    RCLCPP_ERROR(LOGGER, "IK failed along the Cartesian path (%.1f ms elapsed).", t_ik_ms);
    return false;
  }

  const double t_ik_ms = Ms(Clock::now() - t_ik_start).count();
  RCLCPP_INFO(LOGGER, "[IK]       %.1f ms  →  %zu joint states", t_ik_ms, joint_path.size());

  // --- TOTG ---
  const TimePoint t_totg_start = Clock::now();

  auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(
      robot_model_, getGroupName());
  for (auto& state : joint_path)
    traj->addSuffixWayPoint(state, 0.0);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(*traj))
  {
    RCLCPP_ERROR(LOGGER, "Time parameterisation failed.");
    return false;
  }

  const double t_totg_ms  = Ms(Clock::now() - t_totg_start).count();
  const double t_total_ms = Ms(Clock::now() - t_total_start).count();

  RCLCPP_INFO(LOGGER, "[TOTG]     %.1f ms  →  trajectory duration %.3f s",
              t_totg_ms, traj->getDuration());
  RCLCPP_INFO(LOGGER, "[TOTAL]    %.1f ms  (RRT %.1f | IK %.1f | TOTG %.1f)",
              t_total_ms, t_rrt_ms, t_ik_ms, t_totg_ms);

  res.trajectory_     = traj;
  res.planning_time_  = t_total_ms / 1000.0;
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool TaskPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  const TimePoint t_start = Clock::now();
  planning_interface::MotionPlanResponse simple_res;
  const bool ok = solve(simple_res);
  const double elapsed_s = std::chrono::duration<double>(Clock::now() - t_start).count();

  res.error_code_ = simple_res.error_code_;
  if (ok && simple_res.trajectory_)
  {
    res.trajectory_.push_back(simple_res.trajectory_);
    res.description_.emplace_back("task_rrt");
    res.processing_time_.push_back(elapsed_s);
  }
  return ok;
}

bool TaskPlanningContext::terminate() { return true; }
void TaskPlanningContext::clear()     {}

// ============================================================
// getStartPosition
// ============================================================
bool TaskPlanningContext::getStartPosition(Eigen::Vector3d& out) const
{
  moveit::core::RobotState start_state = planning_scene_->getCurrentState();
  if (!moveit::core::robotStateMsgToRobotState(
          request_.start_state, start_state, /*copy_attached_bodies=*/true))
    RCLCPP_WARN(LOGGER, "Could not overlay start state msg; using scene state as-is.");

  start_state.update();
  out = start_state.getGlobalLinkTransform(
            joint_group_->getLinkModelNames().back()).translation();
  return true;
}

// ============================================================
// buildGoalState
// ============================================================
moveit::core::RobotState TaskPlanningContext::buildGoalState() const
{
  moveit::core::RobotState goal_state = planning_scene_->getCurrentState();
  for (const auto& jc : request_.goal_constraints[0].joint_constraints)
  {
    const double val = jc.position;
    goal_state.setJointPositions(jc.joint_name, &val);
  }
  goal_state.update();
  return goal_state;
}

// ============================================================
// getGoalPosition
// ============================================================
bool TaskPlanningContext::getGoalPosition(Eigen::Vector3d& out) const
{
  if (request_.goal_constraints.empty())
  {
    RCLCPP_ERROR(LOGGER, "No goal constraints in request.");
    return false;
  }
  const auto& gc = request_.goal_constraints[0];

  if (!gc.position_constraints.empty())
  {
    const auto& pose =
        gc.position_constraints[0].constraint_region.primitive_poses[0].position;
    out = Eigen::Vector3d(pose.x, pose.y, pose.z);
    return true;
  }

  if (!gc.joint_constraints.empty())
  {
    const moveit::core::RobotState goal_state = buildGoalState();
    out = goal_state.getGlobalLinkTransform(
              joint_group_->getLinkModelNames().back()).translation();
    return true;
  }

  RCLCPP_ERROR(LOGGER, "Goal constraint has neither position nor joint constraints.");
  return false;
}

// ============================================================
// computeCartesianPath  —  RRT
// ============================================================
std::vector<Eigen::Vector3d> TaskPlanningContext::computeCartesianPath(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal)
{
  moveit::core::RobotState seed = planning_scene_->getCurrentState();
  moveit::core::robotStateMsgToRobotState(request_.start_state, seed, true);
  seed.update();

  std::uniform_real_distribution<double> dist_x(ws_min_.x(), ws_max_.x());
  std::uniform_real_distribution<double> dist_y(ws_min_.y(), ws_max_.y());
  std::uniform_real_distribution<double> dist_z(ws_min_.z(), ws_max_.z());
  std::uniform_real_distribution<double> dist_01(0.0, 1.0);

  std::vector<RRTNode> tree;
  tree.reserve(RRT_MAX_ITER);
  tree.emplace_back(start, -1, 1e9);

  const TimePoint t_rrt_loop = Clock::now();

  for (int iter = 0; iter < RRT_MAX_ITER; ++iter)
  {
    Eigen::Vector3d sample = (dist_01(rng_) < RRT_GOAL_BIAS)
        ? goal
        : Eigen::Vector3d(dist_x(rng_), dist_y(rng_), dist_z(rng_));

    int near_idx = nearestNeighbour(tree, sample);
    Eigen::Vector3d new_pos = steer(tree[near_idx].position, sample);

    // Both collision-free AND above clearance floor
    double clearance = 0.0;
    if (!collisionDistance(tree[near_idx].position, new_pos, seed, clearance))
      continue;

    int new_idx = static_cast<int>(tree.size());
    tree.emplace_back(new_pos, near_idx, clearance);

    if ((new_pos - goal).norm() < RRT_GOAL_TOLERANCE)
    {
      RCLCPP_INFO(LOGGER, "RRT reached goal at iteration %d (%.1f ms).",
                  iter, Ms(Clock::now() - t_rrt_loop).count());

      if ((new_pos - goal).norm() > 1e-6)
      {
        tree.emplace_back(goal, new_idx, clearance);
        new_idx = static_cast<int>(tree.size()) - 1;
      }

      auto raw_path = extractPath(tree, new_idx);

      const TimePoint t_smooth = Clock::now();
      auto result = smoothPath(raw_path, seed);
      RCLCPP_INFO(LOGGER, "Path smoothing: %.1f ms  (%zu → %zu waypoints)",
                  Ms(Clock::now() - t_smooth).count(),
                  raw_path.size(), result.size());
      return result;
    }
  }

  RCLCPP_WARN(LOGGER, "RRT exhausted %d iterations without reaching goal.", RRT_MAX_ITER);
  return {};
}

// ============================================================
// computeIKPath
// ============================================================
bool TaskPlanningContext::computeIKPath(
    const std::vector<Eigen::Vector3d>&    path,
    std::vector<moveit::core::RobotState>& joint_path,
    const moveit::core::RobotState*        goal_state_override)
{
  joint_path.clear();
  joint_path.reserve(path.size());

  moveit::core::RobotState current = planning_scene_->getCurrentState();
  moveit::core::robotStateMsgToRobotState(request_.start_state, current, true);
  current.update();

  const std::string& ee_link  = joint_group_->getLinkModelNames().back();
  const std::size_t  last_idx = path.size() - 1;

  const Eigen::Quaterniond q_start(current.getGlobalLinkTransform(ee_link).linear());
  Eigen::Quaterniond q_goal = q_start;
  if (goal_state_override)
    q_goal = Eigen::Quaterniond(
        goal_state_override->getGlobalLinkTransform(ee_link).linear());

  for (std::size_t i = 0; i < path.size(); ++i)
  {
    // Last waypoint: inject exact joint state for joint-space goals
    if (i == last_idx && goal_state_override)
    {
      if (planning_scene_->isStateColliding(*goal_state_override, joint_group_->getName()))
      {
        RCLCPP_ERROR(LOGGER, "Goal joint state is in collision — aborting.");
        return false;
      }
      joint_path.push_back(*goal_state_override);
      break;
    }

    const double t = (path.size() > 1)
        ? static_cast<double>(i) / static_cast<double>(last_idx)
        : 0.0;

    Eigen::Isometry3d target_tf;
    target_tf.linear()      = q_start.slerp(t, q_goal).toRotationMatrix();
    target_tf.translation() = path[i];

    const bool ik_ok = current.setFromIK(
        joint_group_, target_tf, ee_link, 0.1,
        [this](moveit::core::RobotState* state,
               const moveit::core::JointModelGroup* jmg,
               const double*) -> bool {
          state->update();
          return !planning_scene_->isStateColliding(*state, jmg->getName());
        });

    if (!ik_ok)
    {
      RCLCPP_WARN(LOGGER, "IK failed at waypoint %zu/%zu (t=%.2f).", i, path.size(), t);
      return false;
    }
    current.update();
    joint_path.push_back(current);
  }
  return true;
}

// ============================================================
// RRT helpers
// ============================================================
int TaskPlanningContext::nearestNeighbour(
    const std::vector<RRTNode>& tree,
    const Eigen::Vector3d&      sample) const
{
  int    best_idx  = 0;
  double best_dist = (tree[0].position - sample).squaredNorm();
  for (int i = 1; i < static_cast<int>(tree.size()); ++i)
  {
    const double d = (tree[i].position - sample).squaredNorm();
    if (d < best_dist) { best_dist = d; best_idx = i; }
  }
  return best_idx;
}

Eigen::Vector3d TaskPlanningContext::steer(
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to) const
{
  const Eigen::Vector3d diff = to - from;
  const double dist = diff.norm();
  return (dist <= RRT_STEP_SIZE) ? to : from + (diff / dist) * RRT_STEP_SIZE;
}

// ===========================================================================
// Collision helpers
// ===========================================================================

// ---------------------------------------------------------------------------
// buildDistanceRequest
//
// Called once at the start of solve().  Populates:
//
//   clearance_links_  — the set of LinkModel* the distance engine will test.
//                       We include only the EE link + any bodies currently
//                       attached to the group's links.  Base and shoulder
//                       links are not included because they never get close
//                       to workspace obstacles during normal operation.
//
//   d_req_cache_      — a DistanceRequest configured with:
//                         type = GLOBAL   → single scalar out, no map traversal
//                         active_components_only → only clearance_links_
//                         enable_nearest_points  → false (not needed)
//                         acm                    → scene ACM (prunes always-
//                                                  allowed self pairs)
//
// Using GLOBAL + active_components_only means the broadphase tests only the
// listed links against all world objects and returns the single minimum
// distance across that restricted set.  There is no per-pair result map to
// iterate, so inner loop cost drops to O(1) regardless of scene complexity.
// ---------------------------------------------------------------------------
void TaskPlanningContext::buildDistanceRequest()
{
  clearance_links_.clear();

  // Always include the end-effector link — it is the part closest to
  // obstacles in typical manipulation tasks.
  const std::string& ee_name = joint_group_->getLinkModelNames().back();
  if (const auto* lm = robot_model_->getLinkModel(ee_name))
    clearance_links_.insert(lm);

  // Include every body attached to any link in the group.
  // This covers the grasped object — when an object is attached it inherits
  // the link name of the attachment parent, and its geometry is added to the
  // collision world, so distanceRobot will see it through the link's transform.
  const moveit::core::RobotState& scene_state = planning_scene_->getCurrentState();
  std::vector<const moveit::core::AttachedBody*> attached;
  scene_state.getAttachedBodies(attached);

  for (const auto* body : attached)
  {
    const std::string& parent = body->getAttachedLinkName();
    // Only include if the parent link belongs to our planning group
    if (joint_group_->hasLinkModel(parent))
    {
      if(parent == "fr3_link0" || parent == "fr3_link1" || parent == "fr3_link2")
        continue;
      if (const auto* lm = robot_model_->getLinkModel(parent))
        clearance_links_.insert(lm);
    }
  }

  RCLCPP_DEBUG(LOGGER, "clearance_links_ has %zu entries.", clearance_links_.size());

  // Pre-build the request — reused across every checkSegment / pointClearance call.
  d_req_cache_ = collision_detection::DistanceRequest{};
  d_req_cache_.type                   = collision_detection::DistanceRequestType::GLOBAL;
  d_req_cache_.enable_nearest_points  = false;
  d_req_cache_.acm                    = &planning_scene_->getAllowedCollisionMatrix();

  /**
   * GET ATTACHED OBJECT PUT SAME IN THE ALLOWED COLLISION MATRIX
   */

  d_req_cache_.active_components_only = &clearance_links_;
}

// ---------------------------------------------------------------------------
// checkSegment — unified IK + clearance kernel
//
// need_distance=false  →  binary collision check via isStateColliding (no
//                          distance query at all; used by shortcutPath and
//                          the binary path of nudgeClearance).
// need_distance=true   →  GLOBAL distance query over clearance_links_ only;
//                          returns one scalar, no map walking.
// ---------------------------------------------------------------------------
bool TaskPlanningContext::checkSegment(
    const Eigen::Vector3d&          a,
    const Eigen::Vector3d&          b,
    const moveit::core::RobotState& seed_state,
    double&                          min_clearance,
    bool                             need_distance) const
{
  const std::string& ee_link = joint_group_->getLinkModelNames().back();
  moveit::core::RobotState probe = seed_state;
  min_clearance = std::numeric_limits<double>::max();

  // RCLCPP_INFO(LOGGER, "Anothrt");

  for (int s = 1; s <= SEGMENT_SUBDIVISIONS; ++s)
  {
    const double t = static_cast<double>(s) / SEGMENT_SUBDIVISIONS;
    Eigen::Isometry3d tf = probe.getGlobalLinkTransform(ee_link);
    tf.translation() = a + t * (b - a);

    if (!probe.setFromIK(joint_group_, tf, ee_link, 0.05))
      return false;
    probe.update();

      // Fast path: binary only — no distance query overhead.
    if (planning_scene_->isStateColliding(probe, joint_group_->getName()))
      return false;

    if (!need_distance)
      continue;

    std::vector<moveit_msgs::msg::AttachedCollisionObject> all_attached;
    collision_detection::AllowedCollisionMatrix copy = planning_scene_->getAllowedCollisionMatrix();
    planning_scene_->getAttachedCollisionObjectMsgs(all_attached);
    for (const auto& att : all_attached)
    {
      copy.setEntry(att.object.id, att.object.id, true);
      const auto* body = probe.getAttachedBody(att.object.id);
      const auto& tfs = body->getGlobalCollisionBodyTransforms();
  
      for (size_t i = 0; i < tfs.size(); ++i)
      {
        RCLCPP_WARN_STREAM(LOGGER, "Position" << tfs[i].translation());
      }
  
      auto ee_tf = probe.getGlobalLinkTransform("fr3_hand_tcp");
      RCLCPP_WARN(LOGGER, "EE : %f", ee_tf.translation().z());
    }

   
    collision_detection::DistanceRequest d_req_copy = d_req_cache_;
    d_req_copy.type = collision_detection::DistanceRequestType::ALL;
    
    collision_detection::DistanceResult d_res;
    planning_scene_->getCollisionEnv()->distanceRobot(d_req_copy, d_res, probe);
    const double d = d_res.minimum_distance.distance;

    if (min_clearance != std::numeric_limits<double>::max())
      RCLCPP_ERROR(LOGGER, "NEXT : %f", d);
    for (const auto& [pair, data] : d_res.distances)
      RCLCPP_WARN(LOGGER, "Pair : %s | %s | %f", pair.first.c_str(), pair.second.c_str(), data[0].distance); 

    if (d <= 0.0)
      return false;   // in collision

    if (d < RRT_CLEARANCE)
      return false;   // below hard clearance floor

    if (d < min_clearance)
      min_clearance = d;
  }

  if (min_clearance == std::numeric_limits<double>::max())
    min_clearance = 0.0;

  return true;
}

// ---------------------------------------------------------------------------
// isSegmentCollisionFree — thin wrapper, binary answer, no distance overhead
// ---------------------------------------------------------------------------
bool TaskPlanningContext::isSegmentCollisionFree(
    const Eigen::Vector3d&          a,
    const Eigen::Vector3d&          b,
    const moveit::core::RobotState& seed) const
{
  double unused = 0.0;
  return checkSegment(a, b, seed, unused, /*need_distance=*/false);
}

// ---------------------------------------------------------------------------
// collisionDistance — wrapper that also returns min clearance along [a,b]
// ---------------------------------------------------------------------------
bool TaskPlanningContext::collisionDistance(
    const Eigen::Vector3d&          a,
    const Eigen::Vector3d&          b,
    const moveit::core::RobotState& seed,
    double&                          distance) const
{
  return checkSegment(a, b, seed, distance, /*need_distance=*/true);
}

// ---------------------------------------------------------------------------
// pointClearance — single-point version used by nudgeClearance
//
// Reuses checkSegment with a=b so that only one subdivision point is probed.
// The probe is built fresh from `seed` — callers in nudgeClearance call this
// up to 7× per waypoint, so we accept the IK cost but avoid redundant
// distance queries from the old double-check pattern.
// ---------------------------------------------------------------------------
double TaskPlanningContext::pointClearance(
    const Eigen::Vector3d&          p,
    const moveit::core::RobotState& seed) const
{
  double clr = 0.0;
  // Pass a=b: checkSegment will probe exactly one point (at t=1).
  if (!checkSegment(p, p, seed, clr, /*need_distance=*/true))
    return -1.0;
  return clr;
}

// ============================================================
// extractPath
// ============================================================
std::vector<Eigen::Vector3d> TaskPlanningContext::extractPath(
    const std::vector<RRTNode>& tree,
    int                          goal_idx) const
{
  std::vector<Eigen::Vector3d> path;
  for (int idx = goal_idx; idx != -1; idx = tree[idx].parent_idx)
    path.push_back(tree[idx].position);
  std::reverse(path.begin(), path.end());
  return path;
}

// ============================================================
// smoothPath  —  shortcut  then  clearance nudge
// ============================================================
std::vector<Eigen::Vector3d> TaskPlanningContext::smoothPath(
    const std::vector<Eigen::Vector3d>& path,
    const moveit::core::RobotState&     seed) const
{
  auto after_shortcut = shortcutPath(path, seed);
  return nudgeClearance(after_shortcut, seed);
}

// ------------------------------------------------------------
// Pass 1: greedy shortcutting
// ------------------------------------------------------------
std::vector<Eigen::Vector3d> TaskPlanningContext::shortcutPath(
    const std::vector<Eigen::Vector3d>& path,
    const moveit::core::RobotState&     seed) const
{
  if (path.size() <= 2)
    return path;

  std::vector<Eigen::Vector3d> result;
  result.push_back(path.front());

  std::size_t i = 0;
  while (i < path.size() - 1)
  {
    std::size_t j = path.size() - 1;
    while (j > i + 1 && !isSegmentCollisionFree(path[i], path[j], seed))
      --j;
    result.push_back(path[j]);
    i = j;
  }
  return result;
}

// ------------------------------------------------------------
// Pass 2: clearance gradient nudge
//
// For each interior waypoint w_i we approximate the clearance gradient
// using six-point finite differences along ±x, ±y, ±z, then take a step
// in the gradient direction scaled by SMOOTH_NUDGE_STEP.
//
// A move is accepted only when it:
//   (a) keeps clearance ≥ RRT_CLEARANCE, and
//   (b) reduces the cost   C = (1-W)*length - W*clearance
//       (negative clearance term because we want to maximise it)
//
// Start and goal are pinned — they are never moved.
// ------------------------------------------------------------
std::vector<Eigen::Vector3d> TaskPlanningContext::nudgeClearance(
    const std::vector<Eigen::Vector3d>& path,
    const moveit::core::RobotState&     seed) const
{
  if (path.size() <= 2 || SMOOTH_NUDGE_ITERS <= 0)
    return path;

  std::vector<Eigen::Vector3d> wp = path;   // working copy
  const std::size_t N = wp.size();

  // Helper: path cost = (1-W)*total_length - W*sum_clearance
  //   Lower is better (we minimise).
  auto segmentCost = [&](const Eigen::Vector3d& a,
                          const Eigen::Vector3d& b,
                          double clr_a, double clr_b) -> double
  {
    const double len = (b - a).norm();
    const double clr = 0.5 * (clr_a + clr_b);   // average clearance over segment
    return (1.0 - SMOOTH_CLEARANCE_W) * len
           - SMOOTH_CLEARANCE_W * clr;
  };

  // Cache clearance for each waypoint (skip i=0 and i=N-1, they won't move).
  std::vector<double> clr(N, 0.0);
  for (std::size_t i = 1; i + 1 < N; ++i)
    clr[i] = pointClearance(wp[i], seed);   // may be -1 on IK fail → treated as 0

  for (int iter = 0; iter < SMOOTH_NUDGE_ITERS; ++iter)
  {
    bool any_moved = false;

    for (std::size_t i = 1; i + 1 < N; ++i)
    {
      if (clr[i] < 0.0)
        continue;   // IK unreachable — skip

      // ---- Finite-difference clearance gradient ----
      const double eps = SMOOTH_NUDGE_STEP;
      Eigen::Vector3d grad = Eigen::Vector3d::Zero();
      const Eigen::Vector3d axes[3] = {
          {eps, 0, 0}, {0, eps, 0}, {0, 0, eps}
      };

      for (int ax = 0; ax < 3; ++ax)
      {
        const double c_pos = pointClearance(wp[i] + axes[ax], seed);
        const double c_neg = pointClearance(wp[i] - axes[ax], seed);
        // Use one-sided difference if the other side is invalid
        if (c_pos >= 0.0 && c_neg >= 0.0)
          grad[ax] = (c_pos - c_neg) / (2.0 * eps);
        else if (c_pos >= 0.0)
          grad[ax] = (c_pos - clr[i]) / eps;
        else if (c_neg >= 0.0)
          grad[ax] = (clr[i] - c_neg) / eps;
        // else both sides invalid → leave gradient at 0 for this axis
      }

      if (grad.norm() < 1e-9)
        continue;   // flat clearance landscape — nothing to do

      const Eigen::Vector3d candidate = wp[i] + SMOOTH_NUDGE_STEP * grad.normalized();

      // ---- Reject if below clearance floor ----
      const double clr_cand = pointClearance(candidate, seed);
      if (clr_cand < RRT_CLEARANCE)
        continue;

      // ---- Reject if either adjacent segment becomes collision-blocked ----
      if (!isSegmentCollisionFree(wp[i - 1], candidate, seed))
        continue;
      if (!isSegmentCollisionFree(candidate, wp[i + 1], seed))
        continue;

      // ---- Accept only if cost improves ----
      const double cost_before =
          segmentCost(wp[i - 1], wp[i],     clr[i - 1], clr[i]    ) +
          segmentCost(wp[i],     wp[i + 1], clr[i],     clr[i + 1]);
      const double cost_after =
          segmentCost(wp[i - 1], candidate,  clr[i - 1], clr_cand  ) +
          segmentCost(candidate,  wp[i + 1], clr_cand,   clr[i + 1]);

      if (cost_after < cost_before)
      {
        wp[i]  = candidate;
        clr[i] = clr_cand;
        any_moved = true;
      }
    }

    if (!any_moved)
      break;   // converged early
  }

  return wp;
}

}  // namespace task_planner