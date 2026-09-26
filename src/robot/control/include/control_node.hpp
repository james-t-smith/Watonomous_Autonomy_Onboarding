#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode();

private:

  void update_position(
    const nav_msgs::msg::Odometry::SharedPtr msg);

  void update_path(
    const geometry_msgs::msg::PoseArray::SharedPtr msg);

  void follow();

  robot::pose odometry_to_pose(
    const geometry_msgs::msg::PoseStamped& input);

  std::vector<robot::pose> path_to_poses(
    const geometry_msgs::msg::PoseArray& path);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    command_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
    path_sub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  rclcpp::TimerBase::SharedPtr
    path_pub_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string robot_frame_;

  std::string world_frame_;

  robot::ControlCore control_;
};

#endif  // CONTROL_NODE_HPP