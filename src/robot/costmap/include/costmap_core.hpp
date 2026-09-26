#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"


namespace robot
{

struct DistanceNode
{
    int x;
    int y;
    float distance;
};

struct DistanceNodeCompare
{
    bool operator()(
        const DistanceNode& a,
        const DistanceNode& b
    ) const
    {
        return a.distance > b.distance;
    }
};

struct LaserScanData
{

    double stamp;
    double nano;

    float angle_min;
    float angle_increment;
    std::vector<float> ranges;
};

struct Grid
{

    double stamp;
    double nano;

    int width;
    int height;
    float resolution;
    std::vector<int8_t> data;

    float sigma;          // Gaussian standard deviation, meters
    float bleed_radius;   // Maximum distance to propagate, meters

};

void gaussianBleed(Grid& grid);

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    Grid laserToCostmap(const LaserScanData& scan, Grid grid);

  private:
    rclcpp::Logger logger_;

};

}  

#endif  