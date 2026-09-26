#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : logger_(logger)
{
}

void MapMemoryCore::set_params(const MapMeta& data)
{
    mapData = data;

    /*
     * Initialize the persistent map from its intrinsic parameters.
     *
     * The persistent map is assumed to be aligned with the overall
     * map/world frame, so its origin is (0, 0) in that frame.
     */
    world_map.width = mapData.width;
    world_map.height = mapData.height;
    world_map.resolution =
        static_cast<float>(mapData.resolution);

    world_map.origin_pose = {
        0.0,
        0.0,
        0.0
    };

    world_map.stamp = 0.0;
    world_map.nano = 0.0;

    const std::size_t map_size =
        static_cast<std::size_t>(world_map.width) *
        static_cast<std::size_t>(world_map.height);

    world_map.data.assign(map_size, -1);

    world_map = normalize_grid(world_map);

    params_set = true;
}

Grid MapMemoryCore::get_current() const
{
    return world_map;
}

void MapMemoryCore::remember_costmap(const Grid& costmap)
{
    if (!params_set)
    {
        RCLCPP_WARN(
            logger_,
            "MapMemoryCore parameters have not been set"
        );

        return;
    }

    if (costmap.width <= 0 ||
        costmap.height <= 0)
    {
        RCLCPP_WARN(
            logger_,
            "Received invalid costmap dimensions: %d x %d",
            costmap.width,
            costmap.height
        );

        return;
    }

    if (costmap.resolution <= 0.0f)
    {
        RCLCPP_WARN(
            logger_,
            "Received invalid costmap resolution: %f",
            costmap.resolution
        );

        return;
    }

    if (mapData.resolution <= 0.0)
    {
        RCLCPP_WARN(
            logger_,
            "Persistent map has invalid resolution: %f",
            mapData.resolution
        );

        return;
    }

    const std::size_t expected_size =
        static_cast<std::size_t>(costmap.width) *
        static_cast<std::size_t>(costmap.height);

    if (costmap.data.size() != expected_size)
    {
        RCLCPP_WARN(
            logger_,
            "Costmap data size mismatch. Expected %zu cells, got %zu",
            expected_size,
            costmap.data.size()
        );

        return;
    }

    /*
     * Incoming costmap origin pose in the overall map/world frame.
     *
     * origin_pose[0] = origin x
     * origin_pose[1] = origin y
     * origin_pose[2] = origin yaw
     */
    const double origin_x =
        costmap.origin_pose[0];

    const double origin_y =
        costmap.origin_pose[1];

    const double origin_yaw =
        costmap.origin_pose[2];

    const double cos_yaw =
        std::cos(origin_yaw);

    const double sin_yaw =
        std::sin(origin_yaw);

    for (int y = 0; y < costmap.height; ++y)
    {
        for (int x = 0; x < costmap.width; ++x)
        {
            const std::size_t costmap_index =
                static_cast<std::size_t>(y) *
                static_cast<std::size_t>(costmap.width) +
                static_cast<std::size_t>(x);

            /*
             * Position of the cell center relative to the
             * incoming costmap origin.
             *
             * OccupancyGrid coordinates start at the bottom-left
             * origin, so there is no need to calculate a costmap
             * center.
             */
            const double local_x =
                (static_cast<double>(x) + 0.5) *
                costmap.resolution;

            const double local_y =
                (static_cast<double>(y) + 0.5) *
                costmap.resolution;

            /*
             * Transform the cell from the incoming grid frame
             * into the overall map/world frame.
             *
             *      [ cos -sin ] [ local_x ] + [ origin_x ]
             *      [ sin  cos ] [ local_y ]   [ origin_y ]
             */
            const double world_x =
                origin_x +
                cos_yaw * local_x -
                sin_yaw * local_y;

            const double world_y =
                origin_y +
                sin_yaw * local_x +
                cos_yaw * local_y;

            /*
             * Convert the map/world position into a cell index
             * in the persistent map.
             *
             * The persistent map origin is (0, 0), so its position
             * in the map frame is determined entirely by x/y and
             * the persistent map resolution.
             */
            const int map_x = static_cast<int>(
                std::floor((world_x - world_map.origin_pose[0]) / mapData.resolution));

            const int map_y = static_cast<int>(
                std::floor((world_y - world_map.origin_pose[1]) / mapData.resolution));

                        /*
             * Ignore cells outside the persistent map.
             */
            if (map_x < 0 ||
                map_x >= world_map.width ||
                map_y < 0 ||
                map_y >= world_map.height)
            {
                continue;
            }

            const std::size_t world_index =
                static_cast<std::size_t>(map_y) *
                static_cast<std::size_t>(world_map.width) +
                static_cast<std::size_t>(map_x);

            const int8_t cost =
                costmap.data[costmap_index];

            /*
             * Ignore unknown incoming cells.
             *
             * ROS OccupancyGrid convention:
             *   -1 = unknown
             */
            if (cost < 0)
            {
                continue;
            }

            /*
             * Remember the highest observed cost.
             */
            if (cost > world_map.data[world_index])
            {
                world_map.data[world_index] = cost;
            }
        }
    }

    /*
     * Preserve the timestamp of the most recently received map.
     */
    world_map.stamp = costmap.stamp;
    world_map.nano = costmap.nano;
}

Grid MapMemoryCore::normalize_grid(const Grid& grid)
{
    Grid normalized = grid;

    // Center of the new grid in cells.
    const int center_x = grid.width / 2;
    const int center_y = grid.height / 2;

    // Start with an empty map.
    normalized.data.assign(
        grid.width * grid.height,
        -1);

    /*
     * The old grid origin (cell 0,0) is moved to
     * (center_x, center_y) in the new grid.
     *
     * Therefore:
     *
     * new_x = old_x + center_x
     * new_y = old_y + center_y
     */
    for (int old_y = 0; old_y < grid.height; ++old_y)
    {
        for (int old_x = 0; old_x < grid.width; ++old_x)
        {
            const int new_x = old_x + center_x;
            const int new_y = old_y + center_y;

            if (new_x < 0 ||
                new_x >= grid.width ||
                new_y < 0 ||
                new_y >= grid.height)
            {
                continue;
            }

            const int old_index =
                old_y * grid.width + old_x;

            const int new_index =
                new_y * grid.width + new_x;

            normalized.data[new_index] =
                grid.data[old_index];
        }
    }

    /*
     * The new bottom-left corner is now shifted
     * backwards by center_x / center_y cells.
     *
     * This keeps the physical location of the old
     * origin unchanged.
     */
    normalized.origin_pose[0] =
        grid.origin_pose[0] -
        center_x * grid.resolution;

    normalized.origin_pose[1] =
        grid.origin_pose[1] -
        center_y * grid.resolution;

    normalized.origin_pose[2] =
        grid.origin_pose[2];

    return normalized;
}

}  // namespace robot

