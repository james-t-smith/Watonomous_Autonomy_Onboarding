#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

// -- params

    this->declare_parameter("resolution", 0.5);
    this->declare_parameter("width", 60);
    this->declare_parameter("height", 60);
    this->declare_parameter("world_map_frame", "sim_world");


    robot::MapMeta map_memory_load;
    map_memory_load.resolution = get_parameter("resolution").as_double();
    map_memory_load.width = get_parameter("width").as_int();
    map_memory_load.height = get_parameter("height").as_int();

    map_memory_.set_params(map_memory_load);


    // Publisher
    map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map",
            10
        );

    // Subscriber
    costmap_sub_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap",
            10,
            std::bind(
                &MapMemoryNode::costmapCallback,
                this,
                std::placeholders::_1
            )
        );

    map_publish_timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(
                &MapMemoryNode::publishMap,
                this
            )
        );

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    
}

void MapMemoryNode::publishMap()
{

    nav_msgs::msg::OccupancyGrid msg = grid_to_occupancy_grid(map_memory_.get_current(),get_parameter("world_map_frame").as_string());

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing Map: '%d'",
        msg.header.stamp.sec
    );

    map_pub_->publish(msg);

}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

    RCLCPP_INFO(
        this->get_logger(),
        "Received: '%d'",
        msg->header.stamp.sec
    );

    robot::Grid costmap = occupancy_grid_to_grid(*msg,get_parameter("world_map_frame").as_string());

    map_memory_.remember_costmap(costmap);
    
}


robot::Grid MapMemoryNode::occupancy_grid_to_grid(
    const nav_msgs::msg::OccupancyGrid& msg,
    const std::string& world_frame)
{
    robot::Grid grid{};

    // Basic grid information
    grid.width = static_cast<int>(msg.info.width);
    grid.height = static_cast<int>(msg.info.height);
    grid.resolution = msg.info.resolution;
    grid.data = msg.data;

    // ROS timestamp
    grid.stamp = static_cast<double>(msg.header.stamp.sec);
    grid.nano = static_cast<double>(msg.header.stamp.nanosec);

    /*
     * The OccupancyGrid's origin is the bottom-left
     * corner of the grid.
     *
     * We need to express that origin in the world frame.
     */

    tf2::Transform grid_origin;
    tf2::fromMsg(msg.info.origin, grid_origin);

    geometry_msgs::msg::TransformStamped transform;

    try
    {
        transform = tf_buffer_->lookupTransform(
            world_frame,
            msg.header.frame_id,
            msg.header.stamp);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not transform OccupancyGrid from '%s' to '%s': %s",
            msg.header.frame_id.c_str(),
            world_frame.c_str(),
            ex.what());

        return grid;
    }

    tf2::Transform world_tf;
    tf2::fromMsg(transform.transform, world_tf);

    /*
     * Compose:
     *
     * world -> costmap frame
     * with
     * costmap frame -> OccupancyGrid origin
     *
     * The resulting transform gives the OccupancyGrid
     * origin directly in the world frame.
     */
    tf2::Transform world_origin =
        world_tf * grid_origin;

    /*
     * Store the bottom-left origin pose:
     *
     * origin_pose[0] = x
     * origin_pose[1] = y
     * origin_pose[2] = yaw
     */

    grid.origin_pose[0] = world_origin.getOrigin().x();
    grid.origin_pose[1] = world_origin.getOrigin().y();

    const tf2::Quaternion q = world_origin.getRotation();

    const double yaw =
    std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    grid.origin_pose[2] = yaw;
        

    return grid;
}


nav_msgs::msg::OccupancyGrid MapMemoryNode::grid_to_occupancy_grid(
    const robot::Grid& grid,
    const std::string& frame_id)
{
    nav_msgs::msg::OccupancyGrid msg;

    msg.header.frame_id = frame_id;

    // Timestamp
    msg.header.stamp.sec =
        static_cast<int32_t>(grid.stamp);

    msg.header.stamp.nanosec =
        static_cast<uint32_t>(grid.nano);

    // Grid dimensions
    msg.info.width =
        static_cast<uint32_t>(grid.width);

    msg.info.height =
        static_cast<uint32_t>(grid.height);

    msg.info.resolution =
        grid.resolution;

    // Copy data
    msg.data = grid.data;

    /*
     * Grid::origin_pose already represents the
     * bottom-left origin of the grid.
     *
     * No center conversion is necessary.
     */

    msg.info.origin.position.x =
        grid.origin_pose[0];

    msg.info.origin.position.y =
        grid.origin_pose[1];

    msg.info.origin.position.z = 0.0;

    /*
     * Convert origin yaw -> quaternion.
     */
    tf2::Quaternion q;

    q.setRPY(
        0.0,
        0.0,
        grid.origin_pose[2]);

    msg.info.origin.orientation =
        tf2::toMsg(q);

    return msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
