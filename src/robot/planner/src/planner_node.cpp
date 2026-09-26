#include "planner_node.hpp"

#include <cmath>
#include <cstddef>

namespace
{

robot::Grid occupancyGridToGrid(const nav_msgs::msg::OccupancyGrid & ros_grid);
geometry_msgs::msg::PoseArray posesToPoseArray(
  const std::vector<robot::pose> & poses,
  const std::string & frame_id);

}  // namespace

PlannerNode::PlannerNode()
: Node("planner"),
  planner_(this->get_logger())
{
  this->declare_parameter("resolution", 0.2);
  this->declare_parameter("width", 100);
  this->declare_parameter("height", 100);
  this->declare_parameter("world_map_frame", "sim_world");
  this->declare_parameter("robot_map_frame", "robot/chassis/lidar");

  world_map_frame_ = this->get_parameter("world_map_frame").as_string();
  robot_map_frame_ = this->get_parameter("robot_map_frame").as_string();

  // Publisher
  path_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/path",
      10);

  // Subscribers
  map_sub_ =
    this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      10,
      std::bind(
        &PlannerNode::mapCallback,
        this,
        std::placeholders::_1));

  point_sub_ =
    this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point",
      10,
      std::bind(
        &PlannerNode::point_aquired,
        this,
        std::placeholders::_1));

  odom_sub_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filter",
      10,
      std::bind(
        &PlannerNode::set_current_pose,
        this,
        std::placeholders::_1));

  path_pub_timer_ =
    this->create_wall_timer(
      std::chrono::milliseconds(2000),
      std::bind(
        &PlannerNode::path_publish,
        this));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PlannerNode::set_current_pose(nav_msgs::msg::Odometry recent)
{
  std::lock_guard<std::mutex> lock(pose_protection_mutex);
  recent_pos = recent;
}

void PlannerNode::mapCallback(nav_msgs::msg::OccupancyGrid new_map)
{
  std::lock_guard<std::mutex> lock(map_protection_mutex);
  recent_map = new_map;
}

void PlannerNode::point_aquired(geometry_msgs::msg::PointStamped new_point)
{
  robot::pose target_point = pointStampedToPose(new_point, world_map_frame_);
  planner_.set_target(target_point);  // also moves the planner into TRACKING
}

void PlannerNode::path_publish()
{
  // Nothing to do until a goal has been set via point_aquired().
  if (planner_.get_state() != robot::TRACKING)
  {
    return;
  }

  // Copy out the shared state under lock, then release the lock before
  // doing any real work (grid conversion + A*), so the subscription
  // callbacks aren't blocked while planning runs.
  nav_msgs::msg::OccupancyGrid map_copy;
  {
    std::lock_guard<std::mutex> lock(map_protection_mutex);
    map_copy = recent_map;
  }

  // Rather than trusting whatever frame_id happens to be stamped on
  // the incoming Odometry message, ask TF directly: "where is the
  // origin of robot_map_frame_ (e.g. robot_chasis/lidar) right now,
  // expressed in world_map_frame_ (sim_world)?" An identity pose
  // (zero translation, zero rotation) stamped in robot_map_frame_ IS
  // the origin of that frame, so transforming it into world_map_frame_
  // gives exactly that frame's current position/orientation in the
  // world -- driven entirely by the TF tree, not by the odometry
  // message's own contents.
  geometry_msgs::msg::PoseStamped robot_origin_stamped;
  robot_origin_stamped.header.frame_id = robot_map_frame_;
  robot_origin_stamped.pose.orientation.w = 1.0;  // identity quaternion

  // Ask for the *latest available* transform rather than a specific
  // timestamp. Requesting an exact stamp requires TF to have samples
  // bracketing that precise instant; any small clock/rate skew between
  // whatever publishes robot_map_frame_'s TF and this node causes
  // tf2::ExtrapolationException on effectively every call. Time zero
  // tells tf2 "give me whatever you last have."
  robot_origin_stamped.header.stamp = rclcpp::Time(0);

  robot::pose current_pose = poseStampedToPose(robot_origin_stamped, world_map_frame_);

  // poseStampedToPose only sets frame_id on the success path (it
  // returns early, before touching frame_id, whenever the TF lookup
  // throws). So an empty frame_id here is not "the robot is at the
  // world origin" -- it's the signal that the transform failed and
  // this pose is a meaningless default. Planning from it anyway is
  // exactly what was producing paths that always start at the center
  // of sim_world, so skip the cycle instead.
  if (current_pose.frame_id.empty())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not transform current pose from '%s' into '%s'; skipping this planning cycle.",
      robot_map_frame_.c_str(),
      world_map_frame_.c_str());
    return;
  }

  robot::Grid current_grid = occupancyGridToGrid(map_copy);

  // plan()'s signature is (Grid, pose) -- grid first, then start pose.
  std::vector<robot::pose> path = planner_.plan(current_grid, current_pose);

  if (path.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Planner produced an empty path; not publishing.");
    return;
  }

  newest_path = posesToPoseArray(path, world_map_frame_);

  RCLCPP_INFO(
    this->get_logger(),
    "Publishing path with %zu poses.",
    path.size());

  path_pub_->publish(newest_path);
}

robot::pose PlannerNode::odometryToPose(const nav_msgs::msg::Odometry & odom)
{
  robot::pose result;

  // Position
  result.target_x = odom.pose.pose.position.x;
  result.target_y = odom.pose.pose.position.y;

  // Quaternion -> yaw
  const auto & q = odom.pose.pose.orientation;

  result.target_yaw =
    std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Odometry pose is expressed in the odom header frame
  result.frame_id = odom.header.frame_id;

  return result;
}

robot::pose PlannerNode::pointStampedToPose(
  const geometry_msgs::msg::PointStamped & point,
  const std::string & target_frame)
{
  robot::pose result;

  geometry_msgs::msg::PointStamped transformed_point;

  try
  {
    transformed_point =
      tf_buffer_->transform(point, target_frame);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "TF transform failed: %s",
      ex.what());

    return result;
  }

  result.target_x = transformed_point.point.x;
  result.target_y = transformed_point.point.y;
  result.target_yaw = 0.0;
  result.frame_id = target_frame;

  return result;
}

robot::pose PlannerNode::poseStampedToPose(
  const geometry_msgs::msg::PoseStamped & input,
  const std::string & target_frame)
{
  robot::pose result;

  geometry_msgs::msg::PoseStamped transformed_pose;

  try
  {
    transformed_pose =
      tf_buffer_->transform(input, target_frame);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "TF transform failed: %s",
      ex.what());

    return result;
  }

  result.target_x = transformed_pose.pose.position.x;
  result.target_y = transformed_pose.pose.position.y;

  const auto & q = transformed_pose.pose.orientation;

  // Quaternion -> yaw
  result.target_yaw =
    std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  result.frame_id = target_frame;

  return result;
}

namespace
{

robot::Grid occupancyGridToGrid(const nav_msgs::msg::OccupancyGrid & ros_grid)
{
  robot::Grid grid;

  // Timestamp
  grid.stamp = static_cast<double>(ros_grid.header.stamp.sec);
  grid.nano = static_cast<double>(ros_grid.header.stamp.nanosec);

  // Map dimensions
  grid.width = static_cast<int>(ros_grid.info.width);
  grid.height = static_cast<int>(ros_grid.info.height);

  // Resolution
  grid.resolution = ros_grid.info.resolution;

  // Origin position
  grid.origin_pose.target_x = ros_grid.info.origin.position.x;
  grid.origin_pose.target_y = ros_grid.info.origin.position.y;

  // Convert origin quaternion -> yaw
  const auto & q = ros_grid.info.origin.orientation;

  grid.origin_pose.target_yaw =
    std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Allocate grid nodes
  grid.data.resize(ros_grid.data.size());

  // Convert occupancy values to MapNodes
  for (std::size_t i = 0; i < ros_grid.data.size(); ++i)
  {
    robot::MapNode & node = grid.data[i];

    // OccupancyGrid:
    //   -1 = unknown
    //    0 = free
    //  100 = occupied
    node.cost = static_cast<int>(ros_grid.data[i]);

    // Reset A* fields
    node.f = 0.0;
    node.g = 0.0;
    node.parent = -1;
  }

  return grid;
}

geometry_msgs::msg::PoseArray posesToPoseArray(
  const std::vector<robot::pose> & poses,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseArray msg;

  msg.header.frame_id = frame_id;

  for (const auto & p : poses)
  {
    geometry_msgs::msg::Pose ros_pose;

    // Position
    ros_pose.position.x = p.target_x;
    ros_pose.position.y = p.target_y;
    ros_pose.position.z = 0.0;

    // Convert yaw -> quaternion
    ros_pose.orientation.x = 0.0;
    ros_pose.orientation.y = 0.0;
    ros_pose.orientation.z = std::sin(p.target_yaw / 2.0);
    ros_pose.orientation.w = std::cos(p.target_yaw / 2.0);

    msg.poses.push_back(ros_pose);
  }

  return msg;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}