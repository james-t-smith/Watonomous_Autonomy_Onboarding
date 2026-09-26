#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();



    robot::Grid occupancy_grid_to_grid(
    const nav_msgs::msg::OccupancyGrid& msg,
    const std::string& world_frame);


    nav_msgs::msg::OccupancyGrid grid_to_occupancy_grid(
        const robot::Grid& grid,
        const std::string& frame_id);

    void publishMap();

    void costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg);




  private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr map_publish_timer_;

        
};

#endif 
