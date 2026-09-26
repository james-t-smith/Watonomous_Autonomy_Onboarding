#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <array>
#include <cstdint>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

struct MapMeta
{
    int height;
    int width;
    double resolution;
};

struct Grid
{
    double stamp;
    double nano;

    // x, y, yaw of this grid's origin in the overall map/world frame.
    //
    // For an OccupancyGrid this is the pose of the bottom-left
    // corner of the grid.
    std::array<double, 3> origin_pose;

    int width;
    int height;
    float resolution;

    std::vector<int8_t> data;
};

class MapMemoryCore
{
public:

    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Configure the intrinsic properties of the persistent map.
    void set_params(const MapMeta& data);

    // Transform and merge an incoming costmap into the
    // persistent world map.
    void remember_costmap(const Grid& costmap);

    // Return the current persistent map.
    Grid get_current() const;

    Grid normalize_grid(const Grid& grid);

private:

    MapMeta mapData{};
    Grid world_map{};

    rclcpp::Logger logger_;

    bool params_set = false;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_