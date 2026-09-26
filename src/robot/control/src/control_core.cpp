#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger)
{
}

geometry_msgs::msg::Twist ControlCore::get_vel()
{
  geometry_msgs::msg::Twist velocity;

  RCLCPP_INFO(
    logger_,
    "get_vel(): path=%zu current=(%.3f, %.3f, %.3f)",
    path.size(),
    current.target_x,
    current.target_y,
    current.target_yaw);

  if (path.empty())
  {
    RCLCPP_WARN(logger_, "PATH IS EMPTY");
    return velocity;
  }

  // ------------------------------------------------------------
  // 1. Find the path node closest to the robot
  // ------------------------------------------------------------

  int closest_index = 0;
  double closest_distance =
    std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < path.size(); ++i)
  {
    const double dx =
      path[i].target_x - current.target_x;

    const double dy =
      path[i].target_y - current.target_y;

    const double distance_squared =
      dx * dx + dy * dy;

    if (distance_squared < closest_distance)
    {
      closest_distance = distance_squared;
      closest_index = static_cast<int>(i);
    }
  }

  // ------------------------------------------------------------
  // 2. Select the node ahead of the robot
  // ------------------------------------------------------------

  int target_index =
    closest_index + look_ahead;

  if (target_index >= static_cast<int>(path.size()))
  {
    target_index =
      static_cast<int>(path.size()) - 1;
  }

  const pose& target = path[target_index];

  // ------------------------------------------------------------
  // 3. Calculate direction ONLY from robot position
  //    to target node position.
  //
  //    target.target_yaw is intentionally NOT used.
  // ------------------------------------------------------------

  const double dx =
    target.target_x - current.target_x;

  const double dy =
    target.target_y - current.target_y;

  const double target_angle =
    std::atan2(dy, dx);

  // ------------------------------------------------------------
  // 4. Calculate heading error
  // ------------------------------------------------------------

  double heading_error =
    target_angle - current.target_yaw;

  while (heading_error > M_PI)
  {
    heading_error -= 2.0 * M_PI;
  }

  while (heading_error < -M_PI)
  {
    heading_error += 2.0 * M_PI;
  }

  // ------------------------------------------------------------
  // 5. Calculate velocity
  //
  //    Angular velocity is based ONLY on how far the robot's
  //    heading is from the direction of the target position.
  // ------------------------------------------------------------

  velocity.angular.z =
    angular_gain * heading_error;

  velocity.angular.z =
    std::clamp(
      velocity.angular.z,
      -max_angular_speed,
      max_angular_speed);

  // Scale forward speed down as the heading error grows, so the
  // robot turns to face the target instead of driving at full
  // speed while turning at the max rate (which traces a circle
  // around the target instead of converging onto it -- the
  // "spinning around" behavior). Full speed when dead-on, ramping
  // to zero once the target is ~90 degrees or more off-heading.
  const double heading_scale =
    std::clamp(std::cos(heading_error), 0.0, 1.0);

  velocity.linear.x =
    linear_speed * heading_scale;

  // ------------------------------------------------------------
  // 6. Slow down near the final pose
  // ------------------------------------------------------------

  const int final_index =
    static_cast<int>(path.size()) - 1;

  const int nodes_remaining =
    final_index - closest_index;

  if (nodes_remaining <= 2)
  {
    const double speed_scale =
      static_cast<double>(nodes_remaining) / 4.0;

    velocity.linear.x *= speed_scale;
  }

  // ------------------------------------------------------------
  // 7. Stop at the final node
  // ------------------------------------------------------------

  if (nodes_remaining == 0)
  {
    velocity.linear.x = 0.0;
    velocity.angular.z = 0.0;
  }

  RCLCPP_INFO(
    logger_,
    "closest=%d target=%d remaining=%d "
    "target_angle=%.3f heading_error=%.3f "
    "linear=%.3f angular=%.3f",
    closest_index,
    target_index,
    nodes_remaining,
    target_angle,
    heading_error,
    velocity.linear.x,
    velocity.angular.z);

  return velocity;
}

void ControlCore::set_path(std::vector<pose> new_path)
{
  RCLCPP_INFO(
    logger_,
    "ControlCore::set_path() received %zu points",
    new_path.size());

  path = std::move(new_path);

  RCLCPP_INFO(
    logger_,
    "ControlCore path now contains %zu points",
    path.size());
}

void ControlCore::set_current(pose new_position)
{
  RCLCPP_INFO(
    logger_,
    "ControlCore::set_current(): x=%.3f y=%.3f yaw=%.3f",
    new_position.target_x,
    new_position.target_y,
    new_position.target_yaw);

  current = new_position;

  RCLCPP_INFO(
    logger_,
    "ControlCore current: x=%.3f y=%.3f yaw=%.3f",
    current.target_x,
    current.target_y,
    current.target_yaw);
}

}  // namespace robot