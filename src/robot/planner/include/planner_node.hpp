#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  // Subscription callbacks
  void mapCallback(nav_msgs::msg::OccupancyGrid new_map);
  void point_aquired(geometry_msgs::msg::PointStamped new_point);
  void set_current_pose(nav_msgs::msg::Odometry recent);

  // Timer callback
  void path_publish();

  // Conversion helpers
  static robot::pose odometryToPose(const nav_msgs::msg::Odometry & odom);
  robot::pose pointStampedToPose(
    const geometry_msgs::msg::PointStamped & point,
    const std::string & target_frame);
  robot::pose poseStampedToPose(
    const geometry_msgs::msg::PoseStamped & input,
    const std::string & target_frame);

  // recent_map / recent_pos are written from subscription callbacks
  // and read from the timer callback (path_publish), so both sides
  // must lock before touching them.
  std::mutex map_protection_mutex;
  std::mutex pose_protection_mutex;

  nav_msgs::msg::OccupancyGrid recent_map;
  nav_msgs::msg::Odometry recent_pos;
  geometry_msgs::msg::PoseArray newest_path;

  robot::PlannerCore planner_;

  std::string world_map_frame_;
  std::string robot_map_frame_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr path_pub_timer_;
};

#endif  // PLANNER_NODE_HPP_
