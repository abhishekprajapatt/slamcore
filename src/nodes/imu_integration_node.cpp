#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "slamcore/slam.hpp"

class IMUIntegrationNode : public rclcpp::Node
{
public:
    IMUIntegrationNode() : Node("imu_integration_node")
    {
        declare_parameter("imu_topic", "/imu/data");

        std::string imu_topic = get_parameter("imu_topic").as_string();

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&IMUIntegrationNode::imuCallback, this, std::placeholders::_1));

        imu_integration_ = std::make_unique<slamcore::IMUIntegration>();

        RCLCPP_INFO(get_logger(), "IMUIntegrationNode initialized");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        slamcore::Frame frame;
        frame.timestamp = rclcpp::Time(msg->header.stamp).seconds();

        frame.imu_accel.push_back(msg->linear_acceleration.x);
        frame.imu_accel.push_back(msg->linear_acceleration.y);
        frame.imu_accel.push_back(msg->linear_acceleration.z);

        frame.imu_gyro.push_back(msg->angular_velocity.x);
        frame.imu_gyro.push_back(msg->angular_velocity.y);
        frame.imu_gyro.push_back(msg->angular_velocity.z);

        imu_integration_->preintegrate(frame);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::unique_ptr<slamcore::IMUIntegration> imu_integration_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUIntegrationNode>());
    rclcpp::shutdown();
    return 0;
}
