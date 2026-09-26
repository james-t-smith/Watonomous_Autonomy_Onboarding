#include <vector>
#include <queue>
#include <cmath>
#include <cstdint>
#include <limits>
#include <algorithm>


#include "costmap_core.hpp"

namespace robot
{

void gaussianBleed(Grid& grid)
{
    const int width  = grid.width;
    const int height = grid.height;

    const float resolution = grid.resolution;
    const float sigma      = grid.sigma;
    const float maxRadius  = grid.bleed_radius;

    // ------------------------------------------------------------
    // Validate parameters
    // ------------------------------------------------------------

    if (width <= 0 || height <= 0)
        return;

    if (resolution <= 0.0f)
        return;

    if (sigma <= 0.0f)
        return;

    if (maxRadius <= 0.0f)
        return;

    if (grid.data.size() !=
        static_cast<size_t>(width * height))
    {
        return;
    }

    // ------------------------------------------------------------
    // Distance field
    //
    // distance[index] = distance in meters to nearest obstacle
    // ------------------------------------------------------------

    constexpr float INF =
        std::numeric_limits<float>::infinity();

    std::vector<float> distance(
        width * height,
        INF
    );

    std::priority_queue<
        DistanceNode,
        std::vector<DistanceNode>,
        DistanceNodeCompare
    > open;

    // ------------------------------------------------------------
    // Put ALL obstacle cells into the queue.
    //
    // These are the sources for the distance field.
    // ------------------------------------------------------------

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const int index =
                y * width + x;

            if (grid.data[index] == 100)
            {
                distance[index] = 0.0f;

                open.push({
                    x,
                    y,
                    0.0f
                });
            }
        }
    }

    // ------------------------------------------------------------
    // 8-connected neighborhood
    // ------------------------------------------------------------

    constexpr int dx[8] =
    {
        -1,  0,  1,
        -1,  1,
        -1,  0,  1
    };

    constexpr int dy[8] =
    {
        -1, -1, -1,
         0,  0,
         1,  1,  1
    };

    // ------------------------------------------------------------
    // Distance field propagation
    // ------------------------------------------------------------

    while (!open.empty())
    {
        const DistanceNode current =
            open.top();

        open.pop();

        const int x = current.x;
        const int y = current.y;

        const int index =
            y * width + x;

        // This queue entry is stale.
        if (current.distance > distance[index])
            continue;

        // No need to propagate beyond bleed radius.
        if (current.distance >= maxRadius)
            continue;

        for (int i = 0; i < 8; ++i)
        {
            const int nx = x + dx[i];
            const int ny = y + dy[i];

            // Outside grid
            if (nx < 0 || nx >= width ||
                ny < 0 || ny >= height)
            {
                continue;
            }

            const int neighborIndex =
                ny * width + nx;

            // ----------------------------------------------------
            // Physical distance between cells.
            //
            // Orthogonal = resolution
            // Diagonal   = resolution * sqrt(2)
            // ----------------------------------------------------

            const bool diagonal =
                dx[i] != 0 && dy[i] != 0;

            const float step =
                diagonal
                    ? resolution * std::sqrt(2.0f)
                    : resolution;

            const float newDistance =
                current.distance + step;

            // Don't propagate further than necessary.
            if (newDistance > maxRadius)
                continue;

            // Found a shorter path to this cell.
            if (newDistance < distance[neighborIndex])
            {
                distance[neighborIndex] =
                    newDistance;

                open.push({
                    nx,
                    ny,
                    newDistance
                });
            }
        }
    }

    // ------------------------------------------------------------
    // Convert distance field into Gaussian cost.
    // ------------------------------------------------------------

    const float twoSigmaSquared =
        2.0f * sigma * sigma;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const int index =
                y * width + x;

            const float d =
                distance[index];

            // Not within bleed radius.
            if (!std::isfinite(d))
                continue;

            // Preserve actual obstacle.
            if (grid.data[index] == 100)
                continue;

            // Gaussian:
            //
            // C(d) = 100 * exp(-d² / (2σ²))
            //
            const float gaussian =
                100.0f *
                std::exp(
                    -(d * d) /
                    twoSigmaSquared
                );

            // Convert to integer cost.
            const int cost =
                static_cast<int>(
                    std::round(gaussian)
                );

            // Never exceed obstacle cost.
            grid.data[index] =
                static_cast<int8_t>(
                    std::clamp(cost, 0, 100)
                );
        }
    }
}

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

Grid CostmapCore::laserToCostmap(
    const LaserScanData& scan,
    Grid grid)
{
    // Make sure the grid starts empty
    std::fill(grid.data.begin(), grid.data.end(), -1);

    grid.stamp = scan.stamp;
    grid.nano = scan.nano;

    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        float range = scan.ranges[i];

        // Ignore invalid laser measurements
        if (!std::isfinite(range))
            continue;

        // Calculate angle for this measurement
        float angle =
            scan.angle_min +
            i * scan.angle_increment;

        // Laser polar coordinates -> Cartesian coordinates
        float x = range * std::cos(angle);
        float y = range * std::sin(angle);

        // Cartesian coordinates -> grid coordinates
        int grid_x =
            static_cast<int>(x / grid.resolution)
            + grid.width / 2;

        int grid_y =
            static_cast<int>(y / grid.resolution)
            + grid.height / 2;

        // Check if point is inside grid
        if (grid_x < 0 ||
            grid_x >= grid.width ||
            grid_y < 0 ||
            grid_y >= grid.height)
        {
            continue;
        }

        // 2D coordinates -> 1D array index
        int index =
            grid_y * grid.width + grid_x;

        // Mark obstacle
        grid.data[index] = 100;
    }

    gaussianBleed(grid);

    return grid;
}
}