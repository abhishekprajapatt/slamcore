#include "slamcore/slam.hpp"

namespace slamcore
{

    IMUIntegration::IMUIntegration()
        : vel_increment_(Eigen::Vector3d::Zero()),
          pos_increment_(Eigen::Vector3d::Zero()),
          gyro_bias_(Eigen::Vector3d::Zero()),
          accel_bias_(Eigen::Vector3d::Zero()) {}

    void IMUIntegration::preintegrate(const Frame &frame)
    {
        if (frame.imu_accel.size() < 3 || frame.imu_gyro.size() < 3)
        {
            return;
        }

        Eigen::Vector3d accel(frame.imu_accel[0], frame.imu_accel[1], frame.imu_accel[2]);
        Eigen::Vector3d gyro(frame.imu_gyro[0], frame.imu_gyro[1], frame.imu_gyro[2]);

        accel = accel - accel_bias_;
        gyro = gyro - gyro_bias_;

        Eigen::Vector3d gravity(0, 0, -9.81);

        double dt = 0.01;
        pos_increment_ += vel_increment_ * dt + 0.5 * (accel + gravity) * dt * dt;
        vel_increment_ += (accel + gravity) * dt;
    }

    Eigen::Vector3d IMUIntegration::getVelocityIncrement() const
    {
        return vel_increment_;
    }

    Eigen::Vector3d IMUIntegration::getPositionIncrement() const
    {
        return pos_increment_;
    }

} // namespace slamcore
