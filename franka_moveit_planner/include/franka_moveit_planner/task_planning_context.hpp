#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <Eigen/Geometry>
#include <optional>
#include <random>
#include <unordered_set>
#include <vector>

namespace task_planner
{

// ---------------------------------------------------------------------------
// Internal RRT node — lives only inside the planning context
// ---------------------------------------------------------------------------
struct RRTNode
{
  Eigen::Vector3d position;
  int             parent_idx;   // index into the tree vector (-1 = root)
  double          clearance;    // minimum obstacle clearance along the edge

  RRTNode(const Eigen::Vector3d& p, int parent, double clr = 0.0)
  : position(p), parent_idx(parent), clearance(clr) {}
};

// ---------------------------------------------------------------------------
// TaskPlanningContext
// ---------------------------------------------------------------------------
class TaskPlanningContext : public planning_interface::PlanningContext
{
public:
  TaskPlanningContext(const std::string&                           name,
                        const std::string&                           group,
                        const moveit::core::RobotModelConstPtr&      model,
                        const planning_scene::PlanningSceneConstPtr& scene);

  // PlanningContext interface
  bool solve(planning_interface::MotionPlanResponse&         res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
  bool terminate() override;
  void clear()     override;

  void setParams(int iter, double step, double bias, double tol, double clr)
  {
    RRT_MAX_ITER       = iter;
    RRT_STEP_SIZE      = step;
    RRT_GOAL_BIAS      = bias;
    RRT_GOAL_TOLERANCE = tol;
    RRT_CLEARANCE      = clr;
  }

private:
  // ------------------------------------------------------------------
  // Injected (owned externally)
  // ------------------------------------------------------------------
  planning_scene::PlanningSceneConstPtr    planning_scene_;
  moveit::core::RobotModelConstPtr         robot_model_;

  // Set once per solve() call
  const moveit::core::JointModelGroup*     joint_group_ = nullptr;

  // ------------------------------------------------------------------
  // RRT parameters
  // ------------------------------------------------------------------
  int    RRT_MAX_ITER       { 5000  };
  double RRT_STEP_SIZE      { 0.05  };   // metres
  double RRT_GOAL_BIAS      { 0.10  };   // 10 % goal bias
  double RRT_GOAL_TOLERANCE { 0.02  };   // metres
  double RRT_CLEARANCE      { 0.02  };   // hard minimum clearance (metres)

  // ------------------------------------------------------------------
  // Pipeline stages
  // ------------------------------------------------------------------

  /// Seed from planning scene + overlay request start state (MTC-safe).
  bool getStartPosition(Eigen::Vector3d& out) const;

  /// Extract goal EE position; handles both Cartesian and joint-space goals.
  bool getGoalPosition(Eigen::Vector3d& out) const;

  /// Build a fully-seeded RobotState for the joint-space goal.
  moveit::core::RobotState buildGoalState() const;

  /// RRT in Cartesian space — returns smoothed waypoints or empty on failure.
  std::vector<Eigen::Vector3d> computeCartesianPath(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& goal);

  /// IK for every Cartesian waypoint → joint-space path.
  bool computeIKPath(const std::vector<Eigen::Vector3d>&    path,
                     std::vector<moveit::core::RobotState>& joint_path,
                     const moveit::core::RobotState*        goal_state_override);

  // ------------------------------------------------------------------
  // RRT parameters — number of IK probes per segment check
  // ------------------------------------------------------------------
  // Kept separate from RRT_STEP_SIZE so callers of isSegmentCollisionFree
  // (shortcutting, nudging) and the RRT expansion loop share the same value.
  static constexpr int SEGMENT_SUBDIVISIONS = 5;

  // ------------------------------------------------------------------
  // RRT helpers
  // ------------------------------------------------------------------
  int nearestNeighbour(const std::vector<RRTNode>& tree,
                       const Eigen::Vector3d&       sample) const;

  Eigen::Vector3d steer(const Eigen::Vector3d& from,
                        const Eigen::Vector3d& to) const;

  // Builds clearance_links_ and d_req_cache_ once per solve().
  // Must be called after joint_group_ is set.
  void buildDistanceRequest();

  // Core segment-checking kernel.
  // Runs IK + distance query at SEGMENT_SUBDIVISIONS interpolated points.
  //   need_distance=false → fast binary collision check (isSegmentCollisionFree)
  //   need_distance=true  → GLOBAL distance query over clearance_links_ only,
  //                         returns one scalar with no per-pair map traversal.
  bool checkSegment(const Eigen::Vector3d&          a,
                    const Eigen::Vector3d&          b,
                    const moveit::core::RobotState& seed,
                    double&                          min_clearance,
                    bool                             need_distance) const;

  /// Binary wrapper — no distance overhead.
  bool isSegmentCollisionFree(const Eigen::Vector3d&          a,
                              const Eigen::Vector3d&          b,
                              const moveit::core::RobotState& seed) const;

  /// Returns true AND fills `distance` with the minimum clearance along [a,b].
  bool collisionDistance(const Eigen::Vector3d&          a,
                         const Eigen::Vector3d&          b,
                         const moveit::core::RobotState& seed,
                         double&                          distance) const;

  /// Clearance at a single Cartesian point; -1.0 on IK failure / collision.
  double pointClearance(const Eigen::Vector3d&          p,
                        const moveit::core::RobotState& seed) const;

  std::vector<Eigen::Vector3d> extractPath(const std::vector<RRTNode>& tree,
                                           int goal_idx) const;

  // ------------------------------------------------------------------
  // Smoothing
  // ------------------------------------------------------------------

  /// Two-pass smoother:
  ///   Pass 1 — greedy shortcutting (removes redundant waypoints).
  ///   Pass 2 — clearance gradient nudge (pushes surviving waypoints away
  ///             from obstacles without violating the hard clearance floor).
  /// Start and goal waypoints are never moved.
  std::vector<Eigen::Vector3d> smoothPath(
      const std::vector<Eigen::Vector3d>& path,
      const moveit::core::RobotState&     seed) const;

  /// Pass 1: greedy shortcutting — same as before.
  std::vector<Eigen::Vector3d> shortcutPath(
      const std::vector<Eigen::Vector3d>& path,
      const moveit::core::RobotState&     seed) const;

  // ------------------------------------------------------------------
  // Workspace bounds
  // ------------------------------------------------------------------
  Eigen::Vector3d ws_min_{ -1.2, -1.2, 0.0 };
  Eigen::Vector3d ws_max_{  1.2,  1.2, 1.5 };

  mutable std::mt19937 rng_{ std::random_device{}() };

  // Links whose distance to the environment is measured for clearance.
  // Built once per solve() from the EE link + any attached bodies.
  // Kept as a set so buildDistanceRequest() can hand a pointer to
  // DistanceRequest::active_components_only without extra copies.
  mutable std::set<const moveit::core::LinkModel*> clearance_links_;

  // Pre-built DistanceRequest reused across every checkSegment call.
  // GLOBAL type + clearance_links_ means the engine tests only the
  // listed links against all obstacles and returns one scalar — no
  // per-pair map to walk.
  mutable collision_detection::DistanceRequest d_req_cache_;
};

}  // namespace Task_planner