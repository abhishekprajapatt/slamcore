#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "slamcore/slam.hpp"

class BackendOptimizerNode : public rclcpp::Node
{
public:
    BackendOptimizerNode() : Node("backend_optimizer_node")
    {
        declare_parameter("odom_topic", "/odometry/filtered");
        declare_parameter("optimization_frequency", 5.0);

        std::string odom_topic = get_parameter("odom_topic").as_string();
        double opt_freq = get_parameter("optimization_frequency").as_double();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 50,
            std::bind(&BackendOptimizerNode::odomCallback, this, std::placeholders::_1));

        optimizer_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/optimized", 10);

        optimizer_ = std::make_unique<slamcore::BackendOptimizer>();

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / opt_freq)),
            std::bind(&BackendOptimizerNode::optimizationCallback, this));

        RCLCPP_INFO(get_logger(), "BackendOptimizerNode initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        slamcore::Frame frame;
        frame.timestamp = rclcpp::Time(msg->header.stamp).seconds();

        frame.pose = Eigen::Matrix4d::Identity();
        frame.pose(0, 3) = msg->pose.pose.position.x;
        frame.pose(1, 3) = msg->pose.pose.position.y;
        frame.pose(2, 3) = msg->pose.pose.position.z;

        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        frame.pose.block<3, 3>(0, 0) = q.toRotationMatrix();

        optimizer_->addFrame(frame);
    }

    void optimizationCallback()
    {
        optimizer_->optimize();

        auto trajectory = optimizer_->getOptimizedTrajectory();
        if (!trajectory.empty())
        {
            const auto &latest = trajectory.back();
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = get_clock()->now();
            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position.x = latest.pose(0, 3);
            odom_msg.pose.pose.position.y = latest.pose(1, 3);
            odom_msg.pose.pose.position.z = latest.pose(2, 3);

            Eigen::Matrix3d R = latest.pose.block<3, 3>(0, 0);
            Eigen::Quaterniond q(R);
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            optimizer_pub_->publish(odom_msg);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimizer_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<slamcore::BackendOptimizer> optimizer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BackendOptimizerNode>());
    rclcpp::shutdown();
    return 0;
}
