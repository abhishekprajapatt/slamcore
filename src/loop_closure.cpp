#include "slamcore/slam.hpp"
#include <opencv2/features2d.hpp>

namespace slamcore
{

    LoopClosureDetector::LoopClosureDetector()
        : min_keyframe_separation_(10),
          loop_closure_score_threshold_(0.7) {}

    void LoopClosureDetector::setParameters(double min_score, int min_separation)
    {
        loop_closure_score_threshold_ = min_score;
        min_keyframe_separation_ = min_separation;
    }

    void LoopClosureDetector::addKeyframe(const Frame &frame)
    {
        keyframes_.push_back(frame);
    }

    bool LoopClosureDetector::detectLoopClosure(
        const Frame &current_frame,
        std::vector<LoopClosureConstraint> &candidates)
    {

        bool found = false;

        for (size_t i = 0; i < keyframes_.size(); ++i)
        {
            if (current_frame.id - keyframes_[i].id < min_keyframe_separation_)
            {
                continue;
            }

            double similarity = computeSimilarity(current_frame, keyframes_[i]);

            if (similarity > loop_closure_score_threshold_)
            {
                LoopClosureConstraint constraint;
                constraint.frame_a = keyframes_[i].id;
                constraint.frame_b = current_frame.id;
                constraint.score = similarity;
                constraint.relative_pose = keyframes_[i].pose.inverse() * current_frame.pose;
                constraint.information_matrix = Eigen::Matrix<double, 6, 6>::Identity() * 100;

                candidates.push_back(constraint);
                found = true;
            }
        }

        return found;
    }

    double LoopClosureDetector::computeSimilarity(const Frame &frame_a,
                                                  const Frame &frame_b)
    {
        if (frame_a.descriptors.empty() || frame_b.descriptors.empty())
        {
            return 0.0;
        }

        cv::BFMatcher matcher(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> matches;

        try
        {
            matcher.match(frame_a.descriptors, frame_b.descriptors, matches);
        }
        catch (...)
        {
            return 0.0;
        }

        if (matches.empty())
            return 0.0;

        double avg_distance = 0.0;
        for (const auto &match : matches)
        {
            avg_distance += match.distance;
        }
        avg_distance /= matches.size();

        double similarity = 1.0 / (1.0 + avg_distance / 100.0);
        return std::max(0.0, std::min(1.0, similarity));
    }

} // namespace slamcore
