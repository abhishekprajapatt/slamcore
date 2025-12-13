#include "slamcore/slam.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>

namespace slamcore
{

    FrontendVisual::FrontendVisual()
        : max_features_(200),
          feature_quality_(0.01),
          is_initialized_(false) {}

    void FrontendVisual::setParameters(int max_features, double quality)
    {
        max_features_ = max_features;
        feature_quality_ = quality;
    }

    bool FrontendVisual::processImage(const Frame &current_frame, Frame &output_frame)
    {
        output_frame = current_frame;

        if (!is_initialized_)
        {
            last_image_ = current_frame.image.clone();
            detectFeatures(last_image_, last_keypoints_);
            is_initialized_ = true;
            return false;
        }

        std::vector<cv::KeyPoint> current_keypoints;
        detectFeatures(current_frame.image, current_keypoints);

        Eigen::Matrix4d relative_pose = estimateMotion(last_image_, current_frame.image);
        output_frame.pose = last_frame_.pose * relative_pose;

        last_image_ = current_frame.image.clone();
        last_keypoints_ = current_keypoints;
        last_frame_ = output_frame;

        return true;
    }

    void FrontendVisual::detectFeatures(const cv::Mat &image,
                                        std::vector<cv::KeyPoint> &keypoints)
    {
        if (image.empty())
            return;

        cv::Mat gray;
        if (image.channels() == 3)
        {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        }
        else
        {
            gray = image.clone();
        }

        cv::Ptr<cv::ORB> orb = cv::ORB::create(max_features_);
        orb->detect(gray, keypoints);
    }

    Eigen::Matrix4d FrontendVisual::estimateMotion(const cv::Mat &img1,
                                                   const cv::Mat &img2)
    {
        std::vector<cv::Point2f> corners1, corners2;
        std::vector<cv::Point2f> prev_pts, curr_pts;
        std::vector<uchar> status;
        std::vector<float> errors;

        cv::Mat gray1, gray2;
        if (img1.channels() == 3)
            cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
        else
            gray1 = img1.clone();
        if (img2.channels() == 3)
            cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);
        else
            gray2 = img2.clone();

        cv::goodFeaturesToTrack(gray1, prev_pts, max_features_, feature_quality_, 10);

        if (prev_pts.empty())
        {
            return Eigen::Matrix4d::Identity();
        }

        cv::calcOpticalFlowPyrLK(gray1, gray2, prev_pts, curr_pts, status, errors);

        std::vector<cv::Point2f> good_prev, good_curr;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                good_prev.push_back(prev_pts[i]);
                good_curr.push_back(curr_pts[i]);
            }
        }

        if (good_prev.size() < 4)
        {
            return Eigen::Matrix4d::Identity();
        }

        cv::Mat essential = cv::findEssentialMat(good_prev, good_curr, 500,
                                                 cv::Point2d(320, 240), cv::RANSAC);

        cv::Mat mask;
        cv::Mat R, t;
        cv::recoverPose(essential, good_prev, good_curr, cv::Mat::eye(3, 3, CV_64F),
                        R, t, mask);

        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                pose(i, j) = R.at<double>(i, j);
            }
            pose(i, 3) = t.at<double>(i, 0) * 0.1;
        }

        return pose;
    }

} // namespace slamcore
