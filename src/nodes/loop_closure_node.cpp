#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "slamcore/slam.hpp"

class LoopClosureNode : public rclcpp::Node
{
public:
    LoopClosureNode() : Node("loop_closure_node")
    {
        declare_parameter("odom_topic", "/odometry/filtered");
        declare_parameter("min_loop_closure_score", 0.7);
        declare_parameter("min_keyframe_separation", 10);

        std::string odom_topic = get_parameter("odom_topic").as_string();
        double min_score = get_parameter("min_loop_closure_score").as_double();
        int min_sep = get_parameter("min_keyframe_separation").as_int();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 50,
            std::bind(&LoopClosureNode::odomCallback, this, std::placeholders::_1));

        loop_edges_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            "/loop_closure_edges", 10);

        detector_ = std::make_unique<slamcore::LoopClosureDetector>();
        detector_->setParameters(min_score, min_sep);

        frame_id_ = 0;
        RCLCPP_INFO(get_logger(), "LoopClosureNode initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        slamcore::Frame frame;
        frame.id = frame_id_++;
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

        std::vector<slamcore::LoopClosureConstraint> candidates;
        if (detector_->detectLoopClosure(frame, candidates))
        {
            RCLCPP_INFO(get_logger(), "Loop closure detected: %zu candidates",
                        candidates.size());
            publishLoopEdges(candidates);
        }

        detector_->addKeyframe(frame);
    }

    void publishLoopEdges(const std::vector<slamcore::LoopClosureConstraint> &edges)
    {
        geometry_msgs::msg::PoseArray edges_msg;
        edges_msg.header.stamp = get_clock()->now();
        edges_msg.header.frame_id = "map";

        for (const auto &edge : edges)
        {
            geometry_msgs::msg::Pose p;
            p.position.x = edge.relative_pose(0, 3);
            p.position.y = edge.relative_pose(1, 3);
            p.position.z = edge.relative_pose(2, 3);

            Eigen::Quaterniond q(edge.relative_pose.block<3, 3>(0, 0));
            p.orientation.x = q.x();
            p.orientation.y = q.y();
            p.orientation.z = q.z();
            p.orientation.w = q.w();

            edges_msg.poses.push_back(p);
        }

        loop_edges_pub_->publish(edges_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr loop_edges_pub_;
    std::unique_ptr<slamcore::LoopClosureDetector> detector_;

    uint64_t frame_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoopClosureNode>());
    rclcpp::shutdown();
    return 0;
}
