#include <chrono>
#include <memory>

#include "costmap_node.hpp"

#include "costmap_core.hpp"

CostmapNode::CostmapNode()
    : Node("costmap"),
      costmap_(robot::CostmapCore(this->get_logger()))
{

    // -- params

    this->declare_parameter("resolution", 0.2);
    this->declare_parameter("width", 100);
    this->declare_parameter("height", 100);
    this->declare_parameter("lidar_topic", "/lidar");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("gaussian_sigma", 1.0);
    this->declare_parameter("bleed_radius", 2.0);

    costmap_ = robot::CostmapCore(this->get_logger());  

    // Publisher
    string_pub_ =
        this->create_publisher<std_msgs::msg::String>(
            "/test_topic",
            10
        );

    costmap_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/costmap",
            10
        );

    // Subscriber
    lidar_sub_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar",
            10,
            std::bind(
                &CostmapNode::lidarCallback,
                this,
                std::placeholders::_1
            )
        );

    // Timer
    timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(
                &CostmapNode::publishMessage,
                this
            )
        );
}

void CostmapNode::publishMessage()
{
    auto message = std_msgs::msg::String();

    message.data = "Hello, ROS 2!";

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing: '%s'",
        message.data.c_str()
    );

    string_pub_->publish(message);
}

void CostmapNode::publishCostmap(const nav_msgs::msg::OccupancyGrid msg)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Publishing Costmap: '%d'",
        msg.header.stamp.sec
    );

    costmap_pub_->publish(msg);
}

void CostmapNode::lidarCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{



    RCLCPP_INFO(
        this->get_logger(),
        "Received: '%d'",
        msg->header.stamp.sec
    );

    robot::LaserScanData scan;
    robot::Grid grid;
    grid.height = get_parameter("height").as_int();
    grid.width = get_parameter("width").as_int();
    grid.resolution = get_parameter("resolution").as_double();
    grid.data.resize(grid.width * grid.height);
    grid.bleed_radius = get_parameter("bleed_radius").as_double();
    grid.sigma = get_parameter("gaussian_sigma").as_double();





    scan = disassembleScanMessage(*msg);

    grid = costmap_.laserToCostmap(scan,grid);

    nav_msgs::msg::OccupancyGrid occupancyGridMessage = assembleCostmapMessage(grid);
    occupancyGridMessage.header.stamp = msg->header.stamp;
    occupancyGridMessage.header.frame_id = msg->header.frame_id;

    publishCostmap(occupancyGridMessage);


}

// --- conversion  helpers ----

nav_msgs::msg::OccupancyGrid assembleCostmapMessage(
    const robot::Grid& grid)
{
    nav_msgs::msg::OccupancyGrid msg;

    //

    msg.info.resolution = grid.resolution;
    msg.info.width = grid.width;
    msg.info.height = grid.height;

    // Place the grid so that (0, 0) is at the center
    msg.info.origin.position.x =
        -(grid.width * grid.resolution) / 2.0;

    msg.info.origin.position.y =
        -(grid.height * grid.resolution) / 2.0;

    msg.info.origin.position.z = 0.0;

    // No rotation
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    // Grid cells
    msg.data = grid.data;

    return msg;

}

robot::LaserScanData disassembleScanMessage(
    const sensor_msgs::msg::LaserScan& msg)
{
    robot::LaserScanData scan;

    scan.angle_min = msg.angle_min;
    scan.angle_increment = msg.angle_increment;
    scan.ranges = msg.ranges;

    return scan;
}

// --- main ----

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<CostmapNode>()
    );

    rclcpp::shutdown();

    return 0;
}