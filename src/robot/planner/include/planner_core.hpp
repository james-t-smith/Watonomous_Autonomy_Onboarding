#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

enum state
{
  IDLE,
  TRACKING
};

struct pose
{
  double target_x = 0.0;
  double target_y = 0.0;
  double target_yaw = 0.0;

  std::string frame_id;
};

struct MapMeta
{
  int height;
  int width;
  double resolution;
};

struct MapNode
{
  int cost = 0;     // occupancy/inflation cost of this cell (higher = closer to or is an obstacle)
  double f = 0.0;    // A* score: f = g + h (total estimated cost through this cell)
  double g = 0.0;    // A* score: known cost of the cheapest path from start to this cell so far
  int parent = -1;   // flattened index (y * width + x) of predecessor, -1 if none (used to
                      // walk backward from goal to start once a path is found)
};

struct Grid
{
  double stamp = 0.0;
  double nano = 0.0;

  // x, y, yaw of this grid's origin in the overall map/world frame.
  //
  // For an OccupancyGrid this is the pose of the bottom-left
  // corner of the grid.
  pose origin_pose;

  int width = 0;
  int height = 0;
  float resolution = 0.0f;

  std::vector<MapNode> data;
};

class PlannerCore
{
public:
  explicit PlannerCore(const rclcpp::Logger & logger);

  // Returns the pose `ahead` steps along the current path. Position is
  // path[ahead] (clamped to the last point); yaw is the heading from
  // that point toward the next one (path[ahead + 1], also clamped).
  pose get_carrot(int ahead) const;

  // Sets the goal pose and moves the planner into TRACKING state.
  void set_target(pose target_pose);

  // Runs A* on `grid` from `start` to the current `target` member,
  // stores the resulting waypoints in `path`, and returns them.
  // Returns an empty vector if no path is found.
  std::vector<pose> plan(const Grid & grid, const pose & start);

  // Cheap check meant to run between full replans: walks the existing
  // `path` and confirms every point's grid cost stays at or below
  // `max_cost` (e.g. distance-from-obstacle inflation, not occupancy
  // itself). Does NOT look for a more optimal path. On the first point
  // that violates this (or that has fallen outside the grid), triggers
  // a fresh `plan()` from `start` and returns false. Returns true if
  // the existing path is still valid and was left untouched.
  bool verify_path(const Grid & grid, const pose & start, int max_cost);

  // Returns the most recently planned path. Only meaningful while
  // robot_state == TRACKING; the caller should check get_state() (or
  // rely on the fact that `path` is empty in IDLE) before using it.
  std::vector<pose> get_path() const;

  // Read-only accessor so callers outside this class (e.g. PlannerNode)
  // can check the planner's state without needing private access.
  state get_state() const { return robot_state; }

private:
  pose target;              // current goal pose that plan() paths toward
  state robot_state = IDLE; // IDLE or TRACKING
  std::vector<pose> path;   // most recent planned path, start -> goal
  rclcpp::Logger logger_;
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_