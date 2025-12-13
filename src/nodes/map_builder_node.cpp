#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "slamcore/slam.hpp"

class MapBuilderNode : public rclcpp::Node
{
public:
    MapBuilderNode() : Node("map_builder_node")
    {
        declare_parameter("scan_topic", "/scan");
        declare_parameter("map_topic", "/map");
        declare_parameter("cloud_topic", "/map_cloud");
        declare_parameter("grid_resolution", 0.1);

        std::string scan_topic = get_parameter("scan_topic").as_string();
        std::string map_topic = get_parameter("map_topic").as_string();
        std::string cloud_topic = get_parameter("cloud_topic").as_string();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10,
            std::bind(&MapBuilderNode::scanCallback, this, std::placeholders::_1));

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, 10);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, 10);

        map_builder_ = std::make_unique<slamcore::MapBuilder>();

        RCLCPP_INFO(get_logger(), "MapBuilderNode initialized");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        slamcore::Frame frame;
        frame.timestamp = rclcpp::Time(msg->header.stamp).seconds();

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            if (std::isfinite(range))
            {
                float angle = msg->angle_min + i * msg->angle_increment;
                Eigen::Vector3d pt(
                    range * std::cos(angle),
                    range * std::sin(angle),
                    0.0);
                frame.lidar_points.push_back(pt);
            }
        }

        map_builder_->updateMap(frame);

        auto grid = map_builder_->getOccupancyGrid();
        grid.header.stamp = get_clock()->now();
        map_pub_->publish(grid);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::unique_ptr<slamcore::MapBuilder> map_builder_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapBuilderNode>());
    rclcpp::shutdown();
    return 0;
}
