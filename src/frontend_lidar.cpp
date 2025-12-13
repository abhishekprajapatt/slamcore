#include "slamcore/slam.hpp"
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace slamcore
{

    FrontendLidar::FrontendLidar()
        : icp_max_iterations_(50),
          icp_fitness_threshold_(0.01),
          is_initialized_(false) {}

    void FrontendLidar::setParameters(double icp_max_iter, double fitness_threshold)
    {
        icp_max_iterations_ = icp_max_iter;
        icp_fitness_threshold_ = fitness_threshold;
    }

    bool FrontendLidar::processScan(const Frame &current_frame, Frame &output_frame)
    {
        output_frame = current_frame;

        if (!is_initialized_)
        {
            last_frame_ = current_frame;
            is_initialized_ = true;
            return false;
        }

        Eigen::Matrix4d relative_pose = icpRegistration(
            current_frame.lidar_points, last_frame_.lidar_points);

        output_frame.pose = last_frame_.pose * relative_pose;
        last_frame_ = output_frame;

        return true;
    }

    Eigen::Matrix4d FrontendLidar::icpRegistration(
        const std::vector<Eigen::Vector3d> &source,
        const std::vector<Eigen::Vector3d> &target)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());

        for (const auto &pt : source)
        {
            source_cloud->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
        }
        for (const auto &pt : target)
        {
            target_cloud->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
        }

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);
        icp.setMaximumIterations(static_cast<int>(icp_max_iterations_));
        icp.setTransformationEpsilon(icp_fitness_threshold_);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);

        Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
        return icp_transform.cast<double>();
    }

} // namespace slamcore
