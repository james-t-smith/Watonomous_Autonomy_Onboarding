#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

namespace robot
{

struct pose
{
  double target_x = 0.0;
  double target_y = 0.0;
  double target_yaw = 0.0;

  std::string frame_id;
};

class ControlCore
{
public:
  // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
  ControlCore(const rclcpp::Logger& logger);

  geometry_msgs::msg::Twist get_vel();

  void set_current(pose new_position);
  
  void set_path(std::vector<pose> new_path);


private:
  rclcpp::Logger logger_;

  std::vector<pose> path;
  pose current;

  int look_ahead = 3;

  // Controller parameters
  double linear_speed = 0.5;
  double max_angular_speed = .60;
  double angular_gain = 2.0;
};

}  // namespace robot

#endif