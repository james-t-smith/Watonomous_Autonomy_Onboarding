#include "control_node.hpp"

#include <cmath>
#include <chrono>

ControlNode::ControlNode()
  : Node("control"),
    control_(robot::ControlCore(this->get_logger()))
{
  // ------------------------------------------------------------
  // Parameters
  // ------------------------------------------------------------

  robot_frame_ =
    this->declare_parameter<std::string>(
      "robot_frame",
      "robot/chassis/lidar");

  world_frame_ =
    this->declare_parameter<std::string>(
      "world_frame",
      "sim_world");

  // ------------------------------------------------------------
  // TF
  // ------------------------------------------------------------

  tf_buffer_ =
    std::make_shared<tf2_ros::Buffer>(
      this->get_clock());

  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_);

  // ------------------------------------------------------------
  // Publisher
  // ------------------------------------------------------------

  command_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10);

  // ------------------------------------------------------------
  // Path subscriber
  // ------------------------------------------------------------

  path_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/path",
      10,
      std::bind(
        &ControlNode::update_path,
        this,
        std::placeholders::_1));

  // ------------------------------------------------------------
  // Odometry subscriber
  // ------------------------------------------------------------

  odom_sub_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered",
      10,
      std::bind(
        &ControlNode::update_position,
        this,
        std::placeholders::_1));

  // ------------------------------------------------------------
  // Control timer
  // ------------------------------------------------------------

  path_pub_timer_ =
    this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(
        &ControlNode::follow,
        this));
}


// ============================================================
// Update robot position
// ============================================================

void ControlNode::update_position(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // The odometry message is only being used as a trigger.
  // The actual robot pose comes from TF.
  (void)msg;

  geometry_msgs::msg::PoseStamped robot_origin_stamped;

  robot_origin_stamped.header.frame_id = robot_frame_;
  robot_origin_stamped.header.stamp = rclcpp::Time(0);

  robot_origin_stamped.pose.position.x = 0.0;
  robot_origin_stamped.pose.position.y = 0.0;
  robot_origin_stamped.pose.position.z = 0.0;

  robot_origin_stamped.pose.orientation.x = 0.0;
  robot_origin_stamped.pose.orientation.y = 0.0;
  robot_origin_stamped.pose.orientation.z = 0.0;
  robot_origin_stamped.pose.orientation.w = 1.0;

  robot::pose current =
    odometry_to_pose(robot_origin_stamped);

  if (current.frame_id.empty())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not get robot pose from TF");

    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "CURRENT ROBOT: x=%.3f y=%.3f yaw=%.3f",
    current.target_x,
    current.target_y,
    current.target_yaw);

  control_.set_current(current);
}


// ============================================================
// Convert path
// ============================================================

std::vector<robot::pose> ControlNode::path_to_poses(
  const geometry_msgs::msg::PoseArray& path)
{
  std::vector<robot::pose> result;

  result.reserve(path.poses.size());

  for (const auto& ros_pose : path.poses)
  {
    geometry_msgs::msg::PoseStamped input_pose;

    input_pose.header = path.header;
    input_pose.pose = ros_pose;

    try
    {
      geometry_msgs::msg::PoseStamped transformed_pose =
        tf_buffer_->transform(
          input_pose,
          world_frame_);

      robot::pose p;

      p.target_x =
        transformed_pose.pose.position.x;

      p.target_y =
        transformed_pose.pose.position.y;

      const auto& q =
        transformed_pose.pose.orientation;

      p.target_yaw =
        std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));

      p.frame_id =
        world_frame_;

      result.push_back(p);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform path pose from '%s' to '%s': %s "
        "(skipping this pose, keeping the rest of the path)",
        path.header.frame_id.c_str(),
        world_frame_.c_str(),
        ex.what());

      // Skip just this pose instead of discarding the entire
      // path -- a single transient TF failure shouldn't wipe
      // out an otherwise-valid path.
      continue;
    }
  }

  return result;
}


// ============================================================
// Convert robot pose using TF
// ============================================================

robot::pose ControlNode::odometry_to_pose(
  const geometry_msgs::msg::PoseStamped& input)
{
  robot::pose result;

  try
  {
    geometry_msgs::msg::PoseStamped transformed_pose =
      tf_buffer_->transform(
        input,
        world_frame_);

    result.target_x =
      transformed_pose.pose.position.x;

    result.target_y =
      transformed_pose.pose.position.y;

    const auto& q =
      transformed_pose.pose.orientation;

    result.target_yaw =
      std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    result.frame_id =
      world_frame_;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not transform robot pose from '%s' to '%s': %s",
      input.header.frame_id.c_str(),
      world_frame_.c_str(),
      ex.what());
  }

  return result;
}


// ============================================================
// Receive path
// ============================================================

void ControlNode::update_path(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::vector<robot::pose> path =
    path_to_poses(*msg);

  RCLCPP_INFO(
    this->get_logger(),
    "ControlNode::set_path() received %zu points",
    path.size());

  control_.set_path(path);

  RCLCPP_INFO(
    this->get_logger(),
    "ControlNode path now contains %zu points",
    path.size());
}


// ============================================================
// Control loop
// ============================================================

void ControlNode::follow()
{
  geometry_msgs::msg::Twist velocity =
    control_.get_vel();

  RCLCPP_INFO(
    this->get_logger(),
    "cmd_vel: linear.x=%.3f angular.z=%.3f",
    velocity.linear.x,
    velocity.angular.z);

  command_pub_->publish(velocity);
}


// ============================================================
// Main
// ============================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ControlNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}