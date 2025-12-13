#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "slamcore/slam.hpp"

class LidarOdometryNode : public rclcpp::Node
{
public:
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
        declare_parameter("scan_topic", "/scan");
        declare_parameter("odom_topic", "/odometry/lidar");
        declare_parameter("icp_max_iterations", 50.0);
        declare_parameter("icp_fitness_threshold", 0.01);

        std::string scan_topic = get_parameter("scan_topic").as_string();
        std::string odom_topic = get_parameter("odom_topic").as_string();
        double icp_max_iter = get_parameter("icp_max_iterations").as_double();
        double fitness_threshold = get_parameter("icp_fitness_threshold").as_double();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10,
            std::bind(&LidarOdometryNode::scanCallback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        frontend_ = std::make_unique<slamcore::FrontendLidar>();
        frontend_->setParameters(icp_max_iter, fitness_threshold);

        frame_id_ = 0;
        RCLCPP_INFO(get_logger(), "LidarOdometryNode initialized");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        slamcore::Frame frame;
        frame.id = frame_id_++;
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

        slamcore::Frame output_frame;
        if (frontend_->processScan(frame, output_frame))
        {
            publishOdometry(output_frame);
            publishTransform(output_frame);
        }
    }

    void publishOdometry(const slamcore::Frame &frame)
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = get_clock()->now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = frame.pose(0, 3);
        odom_msg.pose.pose.position.y = frame.pose(1, 3);
        odom_msg.pose.pose.position.z = frame.pose(2, 3);

        Eigen::Matrix3d R = frame.pose.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_pub_->publish(odom_msg);
    }

    void publishTransform(const slamcore::Frame &frame)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = frame.pose(0, 3);
        transform.transform.translation.y = frame.pose(1, 3);
        transform.transform.translation.z = frame.pose(2, 3);

        Eigen::Matrix3d R = frame.pose.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<slamcore::FrontendLidar> frontend_;

    uint64_t frame_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
