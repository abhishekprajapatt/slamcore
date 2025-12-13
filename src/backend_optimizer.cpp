#include "slamcore/slam.hpp"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/linear_solver/linear_solver_dense.h>
#include <g2o/solvers/structure_only_solver.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

namespace slamcore
{

    BackendOptimizer::BackendOptimizer() {}

    void BackendOptimizer::addFrame(const Frame &frame)
    {
        frames_.push_back(frame);
    }

    void BackendOptimizer::addLoopClosureConstraint(
        const LoopClosureConstraint &constraint)
    {
        constraints_.push_back(constraint);
    }

    void BackendOptimizer::optimize()
    {
        if (frames_.size() < 2)
            return;

        buildPoseGraph();
        solveGraph();
    }

    void BackendOptimizer::buildPoseGraph()
    {
    }

    void BackendOptimizer::solveGraph()
    {
        auto solver = std::make_unique<g2o::OptimizationAlgorithmDogleg>(
            std::make_unique<g2o::BlockSolverD6>(
                std::make_unique<g2o::LinearSolverDenseD6>()));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver.release());

        for (size_t i = 0; i < frames_.size(); ++i)
        {
            auto v = new g2o::VertexSE3();
            v->setId(i);
            v->setEstimate(g2o::SE3Quat(
                frames_[i].pose.block<3, 3>(0, 0),
                frames_[i].pose.block<3, 1>(0, 3)));

            if (i == 0)
                v->setFixed(true);
            optimizer.addVertex(v);
        }

        for (size_t i = 1; i < frames_.size(); ++i)
        {
            auto edge = new g2o::EdgeSE3();
            edge->setId(i - 1);
            edge->setVertex(0, optimizer.vertex(i - 1));
            edge->setVertex(1, optimizer.vertex(i));

            Eigen::Matrix4d relative = frames_[i - 1].pose.inverse() * frames_[i].pose;
            edge->setMeasurement(g2o::SE3Quat(
                relative.block<3, 3>(0, 0),
                relative.block<3, 1>(0, 3)));

            Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6, 6);
            edge->setInformation(info);
            optimizer.addEdge(edge);
        }

        for (const auto &constraint : constraints_)
        {
            auto edge = new g2o::EdgeSE3();
            edge->setVertex(0, optimizer.vertex(constraint.frame_a));
            edge->setVertex(1, optimizer.vertex(constraint.frame_b));
            edge->setMeasurement(g2o::SE3Quat(
                constraint.relative_pose.block<3, 3>(0, 0),
                constraint.relative_pose.block<3, 1>(0, 3)));
            edge->setInformation(constraint.information_matrix.block<6, 6>(0, 0));
            optimizer.addEdge(edge);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(20);

        for (size_t i = 0; i < frames_.size(); ++i)
        {
            auto v = dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
            if (v)
            {
                g2o::SE3Quat pose = v->estimate();
                frames_[i].pose = Eigen::Matrix4d::Identity();
                frames_[i].pose.block<3, 3>(0, 0) = pose.rotation().toRotationMatrix();
                frames_[i].pose.block<3, 1>(0, 3) = pose.translation();
            }
        }
    }

    std::vector<Frame> BackendOptimizer::getOptimizedTrajectory() const
    {
        return frames_;
    }

} // namespace slamcore
