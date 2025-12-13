#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include "slamcore/slam.hpp"

class VisualOdometryNode : public rclcpp::Node
{
public:
    VisualOdometryNode() : Node("visual_odometry_node")
    {
        declare_parameter("image_topic", "/camera/image_raw");
        declare_parameter("odom_topic", "/odometry/visual");
        declare_parameter("max_features", 200);
        declare_parameter("feature_quality", 0.01);

        std::string image_topic = get_parameter("image_topic").as_string();
        std::string odom_topic = get_parameter("odom_topic").as_string();
        int max_features = get_parameter("max_features").as_int();
        double quality = get_parameter("feature_quality").as_double();

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&VisualOdometryNode::imageCallback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

        frontend_ = std::make_unique<slamcore::FrontendVisual>();
        frontend_->setParameters(max_features, quality);

        frame_id_ = 0;
        RCLCPP_INFO(get_logger(), "VisualOdometryNode initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        slamcore::Frame frame;
        frame.id = frame_id_++;
        frame.timestamp = rclcpp::Time(msg->header.stamp).seconds();
        frame.image = cv_ptr->image;

        slamcore::Frame output_frame;
        if (frontend_->processImage(frame, output_frame))
        {
            publishOdometry(output_frame);
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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<slamcore::FrontendVisual> frontend_;

    uint64_t frame_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
