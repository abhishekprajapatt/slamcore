#include "slamcore/slam.hpp"
#include <pcl/octree/octree_search.h>
#include <pcl/io/pcd_io.h>

namespace slamcore
{

    MapBuilder::MapBuilder()
        : grid_resolution_(0.1),
          max_range_(20.0)
    {
        occupancy_grid_.header.frame_id = "map";
        occupancy_grid_.info.resolution = grid_resolution_;
        occupancy_grid_.info.width = 400;
        occupancy_grid_.info.height = 400;
        occupancy_grid_.data.resize(400 * 400, -1);
    }

    void MapBuilder::updateMap(const Frame &frame)
    {
        for (const auto &pt : frame.lidar_points)
        {
            if (pt.norm() < max_range_)
            {
                all_points_.push_back(pt);
            }
        }
        buildOccupancyGrid();
    }

    void MapBuilder::buildOccupancyGrid()
    {
        std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1);

        occupancy_grid_.info.origin.position.x = -20.0;
        occupancy_grid_.info.origin.position.y = -20.0;

        for (const auto &pt : all_points_)
        {
            int grid_x = static_cast<int>((pt.x() + 20.0) / grid_resolution_);
            int grid_y = static_cast<int>((pt.y() + 20.0) / grid_resolution_);

            if (grid_x >= 0 && grid_x < 400 && grid_y >= 0 && grid_y < 400)
            {
                int idx = grid_y * 400 + grid_x;
                occupancy_grid_.data[idx] = 100;
            }
        }
    }

    void MapBuilder::publishOccupancyGrid()
    {
    }

    void MapBuilder::publishPointCloud()
    {
    }

    nav_msgs::msg::OccupancyGrid MapBuilder::getOccupancyGrid() const
    {
        return occupancy_grid_;
    }

    sensor_msgs::msg::PointCloud2 MapBuilder::getPointCloud() const
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        return cloud_msg;
    }

    void MapBuilder::voxelizePoints()
    {
    }

} // namespace slamcore
