#include "slamcore/slam.hpp"

namespace slamcore
{

    Frame::Frame()
        : id(0), timestamp(0.0),
          pose(Eigen::Matrix4d::Identity()),
          pose_covariance(Eigen::Matrix4d::Identity()) {}

} // namespace slamcore
