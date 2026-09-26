#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace robot
{

namespace
{

// The grid is stored as a flat 1D vector (row-major), so every helper
// that talks about "grid coordinates" needs a way to turn an (x, y)
// cell into that vector's index and back. This is the (x, y) -> index
// direction.
inline int toIndex(int x, int y, int width)
{
  return y * width + x;
}

// Straight-line distance between two grid cells. Used as the A*
// heuristic (the "h" in f = g + h) and also to seed the cost of the
// start node. Because moves are 1 unit (orthogonal) or sqrt(2) units
// (diagonal) apart, straight-line distance never overestimates the
// true remaining cost, which keeps the search optimal.
inline double heuristic(int x0, int y0, int x1, int y1)
{
  const double dx = static_cast<double>(x1 - x0);
  const double dy = static_cast<double>(y1 - y0);
  return std::sqrt(dx * dx + dy * dy);
}

// Converts a world-frame (meters) position into a grid cell (gx, gy).
// The grid's origin_pose is the world position of the grid's
// bottom-left corner, so we subtract that off and divide by cell size
// (resolution) to get a cell index. Truncating with static_cast<int>
// floors toward the cell the point falls inside.
inline void worldToGrid(const Grid & grid, double wx, double wy, int & gx, int & gy)
{
  gx = static_cast<int>((wx - grid.origin_pose.target_x) / grid.resolution);
  gy = static_cast<int>((wy - grid.origin_pose.target_y) / grid.resolution);
}

// The inverse of worldToGrid: turns a grid cell back into a world-frame
// position, placing the point at the CENTER of the cell (the "+ 0.5")
// rather than its corner, so waypoints sit in the middle of each cell.
inline void gridToWorld(const Grid & grid, int gx, int gy, double & wx, double & wy)
{
  wx = grid.origin_pose.target_x + (gx + 0.5) * grid.resolution;
  wy = grid.origin_pose.target_y + (gy + 0.5) * grid.resolution;
}

// Sanity check that the grid's declared width/height actually matches
// how much data it's carrying, so we never index off the end of
// grid.data further down. Cheap, so both plan() and verify_path() call
// this before touching the grid.
inline bool gridDataValid(const Grid & grid)
{
  return grid.width > 0 && grid.height > 0 &&
         static_cast<int>(grid.data.size()) == grid.width * grid.height;
}

}  // namespace

PlannerCore::PlannerCore(const rclcpp::Logger & logger)
: robot_state(IDLE), logger_(logger)
{
}

void PlannerCore::set_target(pose target_pose)
{
  target = target_pose;

  // Acquiring a new goal is what puts the planner into TRACKING mode;
  // plan()/verify_path() are meaningless without a target, and
  // path_publish() (on the node side) gates on this state.
  robot_state = TRACKING;
}

// Returns a point further along the already-planned `path`, for a
// controller to chase ("pure pursuit" style). `ahead` is how many
// waypoints forward of the start of the path to look.
pose PlannerCore::get_carrot(int ahead) const
{
  pose carrot{0.0, 0.0, 0.0};

  if (path.empty())
  {
    RCLCPP_WARN(logger_, "get_carrot called with an empty path.");
    return carrot;
  }

  // Clamp so we never read past the last waypoint, even if `ahead` is
  // bigger than the path itself -- in that case we just return the
  // final point.
  const int last = static_cast<int>(path.size()) - 1;
  const int idx = std::clamp(ahead, 0, last);
  const int idx_next = std::clamp(ahead + 1, 0, last);

  // Position: just the waypoint at `idx`.
  carrot.target_x = path[idx].target_x;
  carrot.target_y = path[idx].target_y;

  // Orientation: point toward the NEXT waypoint (idx_next), so the
  // returned pose tells the robot which way to be facing as it
  // arrives at (or passes through) this carrot point.
  const double dx = path[idx_next].target_x - path[idx].target_x;
  const double dy = path[idx_next].target_y - path[idx].target_y;

  // Special case: idx == idx_next happens only at the very end of the
  // path (nothing left to point toward). Rather than atan2(0, 0),
  // which is degenerate, reuse the heading of the final segment so
  // the yaw still means something.
  carrot.target_yaw = (dx == 0.0 && dy == 0.0 && idx > 0)
    ? std::atan2(path[idx].target_y - path[idx - 1].target_y,
                 path[idx].target_x - path[idx - 1].target_x)
    : std::atan2(dy, dx);

  return carrot;
}

std::vector<pose> PlannerCore::get_path() const
{
  if (robot_state == TRACKING)
  {
    return path;
  }
  else
  {
    RCLCPP_ERROR(logger_, "Path not available while idling.");
    return path;  // empty
  }
}

// Full A* search over `grid`, from `start` to the class's `target`
// member (the goal). This is the expensive/"slow loop" planner --
// verify_path() below is the cheap check that runs between calls to
// this.
std::vector<pose> PlannerCore::plan(const Grid & grid, const pose & start)
{
  path.clear();

  const int width = grid.width;
  const int height = grid.height;
  const int total = width * height;

  if (!gridDataValid(grid))
  {
    RCLCPP_ERROR(logger_, "Grid dimensions do not match the provided data.");
    return path;
  }

  // Convert the world-frame start/goal poses into grid cells -- A*
  // itself only ever works in grid coordinates.
  int start_x, start_y, goal_x, goal_y;
  worldToGrid(grid, start.target_x, start.target_y, start_x, start_y);
  worldToGrid(grid, target.target_x, target.target_y, goal_x, goal_y);

  const bool start_in_bounds =
    start_x >= 0 && start_x < width && start_y >= 0 && start_y < height;
  const bool goal_in_bounds =
    goal_x >= 0 && goal_x < width && goal_y >= 0 && goal_y < height;

  if (!start_in_bounds || !goal_in_bounds)
  {
    RCLCPP_ERROR(logger_, "Start or goal pose falls outside the grid.");
    return path;
  }

  // Any cell with cost >= this is treated as a hard obstacle and is
  // never expanded into. This is a coarser, binary check than
  // verify_path()'s max_cost, which allows a range of "close to an
  // obstacle but still technically driveable" costs.
  constexpr int kObstacleCost = 50;

  // Work on a local copy of the grid's nodes so we can freely stash
  // A* bookkeeping (g, f, parent) into each cell without mutating the
  // caller's grid.
  std::vector<MapNode> nodes = grid.data;
  for (auto & node : nodes)
  {
    // "Infinity" means "not reached by the search yet" -- any real
    // path found later will always be cheaper than this, so the
    // first time we touch a cell we're guaranteed to update it.
    node.g = std::numeric_limits<double>::infinity();
    node.f = std::numeric_limits<double>::infinity();
    node.parent = -1;
  }

  // Cells we've fully processed and don't need to look at again.
  std::vector<bool> closed(total, false);

  // Min-heap of (f-score, cell index), so open.top() is always the
  // most promising cell to expand next. std::priority_queue is a
  // max-heap by default, hence the ">" comparator to flip it.
  using QueueEntry = std::pair<double, int>;  // (f, index)
  auto cmp = [](const QueueEntry & a, const QueueEntry & b) { return a.first > b.first; };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(cmp)> open(cmp);

  const int start_idx = toIndex(start_x, start_y, width);
  const int goal_idx = toIndex(goal_x, goal_y, width);

  // Seed the search: the start cell costs 0 to reach (g = 0), and its
  // f-score is pure heuristic since g is 0.
  nodes[start_idx].g = 0.0;
  nodes[start_idx].f = heuristic(start_x, start_y, goal_x, goal_y);
  open.push({nodes[start_idx].f, start_idx});

  // 8-connected neighbor offsets: the first 4 are orthogonal moves,
  // the last 4 are diagonal moves.
  static constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;

  while (!open.empty())
  {
    const int current = open.top().second;
    open.pop();

    // A cell can be pushed onto `open` more than once (if we find a
    // cheaper way to reach it after it was already queued). Skip
    // stale duplicates once the cheapest version has been processed.
    if (closed[current])
    {
      continue;
    }
    closed[current] = true;

    // Standard A* early-exit: since we always expand the
    // lowest-f-score cell first, the first time we pop the goal it's
    // guaranteed to be via the cheapest path.
    if (current == goal_idx)
    {
      found = true;
      break;
    }

    const int cx = current % width;
    const int cy = current / width;

    // Look at all 8 neighbors of the current cell.
    for (int n = 0; n < 8; ++n)
    {
      const int nx = cx + kDx[n];
      const int ny = cy + kDy[n];

      if (nx < 0 || nx >= width || ny < 0 || ny >= height)
      {
        continue;  // off the grid
      }

      const int nidx = toIndex(nx, ny, width);

      if (closed[nidx] || nodes[nidx].cost >= kObstacleCost)
      {
        continue;  // already finalized, or blocked by an obstacle
      }

      // Diagonal moves cover more ground (sqrt(2) cells) than
      // orthogonal moves (1 cell), so they cost proportionally more.
      const double step_cost = (kDx[n] != 0 && kDy[n] != 0) ? std::sqrt(2.0) : 1.0;
      const double tentative_g = nodes[current].g + step_cost;

      // If reaching this neighbor via `current` is cheaper than any
      // way we've reached it before, record that as its new best
      // route (this is the "relaxation" step of the search).
      if (tentative_g < nodes[nidx].g)
      {
        nodes[nidx].g = tentative_g;
        nodes[nidx].f = tentative_g + heuristic(nx, ny, goal_x, goal_y);
        nodes[nidx].parent = current;
        open.push({nodes[nidx].f, nidx});
      }
    }
  }

  if (!found)
  {
    RCLCPP_WARN(logger_, "A* failed to find a path to the goal.");
    return path;  // still empty
  }

  // Walk backward from the goal to the start by following each cell's
  // `parent`, then reverse so the path reads start -> goal.
  std::vector<int> index_path;
  for (int idx = goal_idx; idx != -1; idx = nodes[idx].parent)
  {
    index_path.push_back(idx);
  }
  std::reverse(index_path.begin(), index_path.end());

  // Convert the grid-cell path back into world-frame poses for the
  // rest of the codebase to consume.
  path.reserve(index_path.size());
  for (int idx : index_path)
  {
    pose p{0.0, 0.0, 0.0};
    gridToWorld(grid, idx % width, idx / width, p.target_x, p.target_y);
    path.push_back(p);
  }

  // Convert the grid-cell path back into world-frame poses.
path.reserve(index_path.size());

for (std::size_t i = 0; i < index_path.size(); ++i)
{
  const int idx = index_path[i];

  pose p{0.0, 0.0, 0.0};

  // Convert current grid cell to world coordinates.
  gridToWorld(
    grid,
    idx % width,
    idx / width,
    p.target_x,
    p.target_y);

  // ------------------------------------------------------------
  // Calculate yaw so the pose points toward the next path node.
  // ------------------------------------------------------------

  if (i + 1 < index_path.size())
  {
    const int next_idx = index_path[i + 1];

    double next_x;
    double next_y;

    gridToWorld(
      grid,
      next_idx % width,
      next_idx / width,
      next_x,
      next_y);

    p.target_yaw =
      std::atan2(
        next_y - p.target_y,
        next_x - p.target_x);
  }
  else if (i > 0)
  {
    // Final node: keep the same heading as the previous segment.
    p.target_yaw = path.back().target_yaw;
  }
  else
  {
    // Path contains only one point.
    p.target_yaw = 0.0;
  }

  path.push_back(p);
}

  return path;


  
}

// Cheap, high-frequency check meant to run between full plan() calls.
// It does NOT search for a better path -- it only asks "has anything
// in the world changed enough that the path we already have is no
// longer safe?" If so, it falls back to a full replan.
bool PlannerCore::verify_path(const Grid & grid, const pose & start, int max_cost)
{
  if (!gridDataValid(grid))
  {
    // Can't trust this grid at all -- treat it like a worst-case
    // conflict and replan immediately.
    RCLCPP_ERROR(logger_, "verify_path: grid dimensions do not match the provided data.");
    plan(grid, start);
    return false;
  }

  if (path.empty())
  {
    // Nothing to verify; let the caller's normal replan cadence handle it.
    return true;
  }

  bool conflict = false;

  // Walk every waypoint already in `path` and make sure the cell it
  // sits in is still within an acceptable cost. `cost` here is meant
  // to represent something like "distance from the nearest obstacle"
  // (an inflation layer), not raw occupancy -- so this is a stricter,
  // continuous check compared to plan()'s binary kObstacleCost.
  for (const auto & p : path)
  {
    int gx, gy;
    worldToGrid(grid, p.target_x, p.target_y, gx, gy);

    // The grid may have shifted or shrunk since this path was
    // planned; a waypoint that's fallen off the edge counts as a
    // conflict just like a high-cost cell would.
    if (gx < 0 || gx >= grid.width || gy < 0 || gy >= grid.height)
    {
      conflict = true;
      break;
    }

    if (grid.data[toIndex(gx, gy, grid.width)].cost > max_cost)
    {
      conflict = true;
      break;
    }
  }

  if (!conflict)
  {
    // Existing path is still good -- leave it untouched.
    return true;
  }

  // Something along the path is no longer safe: fall back to a full
  // A* replan and overwrite `path` with the new result.
  RCLCPP_WARN(logger_, "verify_path: path crosses a cell above max_cost, replanning.");
  plan(grid, start);
  return false;
}

}  // namespace robot