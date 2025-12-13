#include <gtest/gtest.h>
#include "slamcore/slam.hpp"
#include <Eigen/Dense>

TEST(FrameTest, DefaultConstruction)
{
    slamcore::Frame frame;
    EXPECT_EQ(frame.id, 0);
    EXPECT_EQ(frame.timestamp, 0.0);
    EXPECT_TRUE(frame.pose.isIdentity());
    EXPECT_TRUE(frame.lidar_points.empty());
    EXPECT_TRUE(frame.keypoints.empty());
}

TEST(FrameTest, PoseAssignment)
{
    slamcore::Frame frame;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = 1.0;
    pose(1, 3) = 2.0;
    pose(2, 3) = 3.0;

    frame.pose = pose;

    EXPECT_DOUBLE_EQ(frame.pose(0, 3), 1.0);
    EXPECT_DOUBLE_EQ(frame.pose(1, 3), 2.0);
    EXPECT_DOUBLE_EQ(frame.pose(2, 3), 3.0);
}

TEST(LoopClosureConstraintTest, DefaultConstruction)
{
    slamcore::LoopClosureConstraint constraint;
    EXPECT_EQ(constraint.frame_a, 0);
    EXPECT_EQ(constraint.frame_b, 0);
    EXPECT_TRUE(constraint.relative_pose.isIdentity());
    EXPECT_DOUBLE_EQ(constraint.score, 0.0);
}

TEST(FrontendLidarTest, Initialization)
{
    slamcore::FrontendLidar frontend;
    frontend.setParameters(50.0, 0.01);

    slamcore::Frame frame;
    slamcore::Frame output;

    bool result = frontend.processScan(frame, output);
    EXPECT_FALSE(result);
}

TEST(IMUIntegrationTest, Initialization)
{
    slamcore::IMUIntegration imu;

    auto vel = imu.getVelocityIncrement();
    auto pos = imu.getPositionIncrement();

    EXPECT_EQ(vel.norm(), 0.0);
    EXPECT_EQ(pos.norm(), 0.0);
}

TEST(BackendOptimizerTest, EmptyOptimization)
{
    slamcore::BackendOptimizer optimizer;
    optimizer.optimize();

    auto trajectory = optimizer.getOptimizedTrajectory();
    EXPECT_TRUE(trajectory.empty());
}

TEST(LoopClosureDetectorTest, Initialization)
{
    slamcore::LoopClosureDetector detector;
    detector.setParameters(0.7, 10);

    slamcore::Frame frame;
    std::vector<slamcore::LoopClosureConstraint> candidates;

    bool result = detector.detectLoopClosure(frame, candidates);
    EXPECT_FALSE(result);
}

TEST(MapBuilderTest, Initialization)
{
    slamcore::MapBuilder map_builder;

    auto grid = map_builder.getOccupancyGrid();
    EXPECT_EQ(grid.header.frame_id, "map");
    EXPECT_GT(grid.data.size(), 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
