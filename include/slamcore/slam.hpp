#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <vector>

namespace slamcore
{

    struct Frame
    {
        uint64_t id;
        double timestamp;

        Eigen::Matrix4d pose;
        Eigen::Matrix4d pose_covariance;

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cv::Mat image;

        std::vector<double> imu_gyro;
        std::vector<double> imu_accel;

        Frame() : id(0), timestamp(0.0), pose(Eigen::Matrix4d::Identity()),
                  pose_covariance(Eigen::Matrix4d::Identity()) {}
    };

    struct LoopClosureConstraint
    {
        uint64_t frame_a;
        uint64_t frame_b;
        Eigen::Matrix4d relative_pose;
        Eigen::Matrix<double, 6, 6> information_matrix;
        double score;

        LoopClosureConstraint() : frame_a(0), frame_b(0),
                                  relative_pose(Eigen::Matrix4d::Identity()),
                                  information_matrix(Eigen::Matrix<double, 6, 6>::Identity()),
                                  score(0.0) {}
    };

    class FrontendLidar
    {
    public:
        FrontendLidar();

        bool processScan(const Frame &current_frame, Frame &output_frame);
        void setParameters(double icp_max_iter, double fitness_threshold);

    private:
        double icp_max_iterations_;
        double icp_fitness_threshold_;
        Frame last_frame_;
        bool is_initialized_;

        Eigen::Matrix4d icpRegistration(const std::vector<Eigen::Vector3d> &source,
                                        const std::vector<Eigen::Vector3d> &target);
    };

    class FrontendVisual
    {
    public:
        FrontendVisual();

        bool processImage(const Frame &current_frame, Frame &output_frame);
        void setParameters(int max_features, double quality);

    private:
        int max_features_;
        double feature_quality_;
        cv::Mat last_image_;
        std::vector<cv::KeyPoint> last_keypoints_;
        bool is_initialized_;

        void detectFeatures(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints);
        Eigen::Matrix4d estimateMotion(const cv::Mat &img1, const cv::Mat &img2);
    };

    class IMUIntegration
    {
    public:
        IMUIntegration();

        void preintegrate(const Frame &frame);
        Eigen::Vector3d getVelocityIncrement() const;
        Eigen::Vector3d getPositionIncrement() const;

    private:
        Eigen::Vector3d vel_increment_;
        Eigen::Vector3d pos_increment_;
        Eigen::Vector3d gyro_bias_;
        Eigen::Vector3d accel_bias_;
    };

    class BackendOptimizer
    {
    public:
        BackendOptimizer();

        void addFrame(const Frame &frame);
        void addLoopClosureConstraint(const LoopClosureConstraint &constraint);
        void optimize();

        std::vector<Frame> getOptimizedTrajectory() const;

    private:
        std::vector<Frame> frames_;
        std::vector<LoopClosureConstraint> constraints_;

        void buildPoseGraph();
        void solveGraph();
    };

    class LoopClosureDetector
    {
    public:
        LoopClosureDetector();

        bool detectLoopClosure(const Frame &current_frame,
                               std::vector<LoopClosureConstraint> &candidates);
        void addKeyframe(const Frame &frame);
        void setParameters(double min_score, int min_separation);

    private:
        std::vector<Frame> keyframes_;
        int min_keyframe_separation_;
        double loop_closure_score_threshold_;

        double computeSimilarity(const Frame &frame_a, const Frame &frame_b);
    };

    class MapBuilder
    {
    public:
        MapBuilder();

        void updateMap(const Frame &frame);
        void publishOccupancyGrid();
        void publishPointCloud();

        nav_msgs::msg::OccupancyGrid getOccupancyGrid() const;
        sensor_msgs::msg::PointCloud2 getPointCloud() const;

    private:
        std::vector<Eigen::Vector3d> all_points_;
        nav_msgs::msg::OccupancyGrid occupancy_grid_;
        double grid_resolution_;
        double max_range_;

        void buildOccupancyGrid();
        void voxelizePoints();
    };

} 
