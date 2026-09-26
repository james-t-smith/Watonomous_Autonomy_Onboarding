#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

 
#include "costmap_core.hpp"

nav_msgs::msg::OccupancyGrid assembleCostmapMessage(
    const robot::Grid& grid);
    
robot::LaserScanData disassembleScanMessage(
    const sensor_msgs::msg::LaserScan& msg);
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    void publishCostmap(const nav_msgs::msg::OccupancyGrid msg);

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif 