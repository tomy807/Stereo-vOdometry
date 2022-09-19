#include <gtest/gtest.h>
#include <iostream>
#include "myslam/dataset.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frame.h"
#include "myslam/common_include.h"
#include "myslam/algorithm.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

using namespace myslam;
using namespace std;
using namespace cv;


cv::Mat PlotFrameImage(Frame::Ptr current_frame_) {
    cv::Mat img_out;
    cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto feat = current_frame_->features_left_[i];
        if(i==0){
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(255, 0, 0),2);
        }else{
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),2);

        }
    }
    return img_out;
}

cv::Mat PlotFrameImageRight(Frame::Ptr current_frame_) {
    cv::Mat img_out;
    cv::cvtColor(current_frame_->right_img_, img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->features_right_.size(); ++i) {
        auto feat = current_frame_->features_right_[i];
        
        if(feat !=nullptr){
            if(i==0){
                cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(250, 0, 0),2);
            }else{
                cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),2);   
            }
        }
    }
    return img_out;
}

int FindFeaturesInRight(Frame::Ptr current_frame_) {
    std::vector<cv::Point2f> kps_left, kps_right;
    // use LK flow to estimate points in the right image
    for (auto &kp : current_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
        // if dont Fill in kps_rights error
        kps_right.push_back(kp->position_.pt);
    }
    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }
    cout << "Find " << num_good_pts << " in the right image." << endl;
    return num_good_pts;
}

TEST(FEATURE,Detech_LeftImg_Features){

    std::string config_file = "/home/tomy807/streoSlam/config/default.yaml";
    Config::SetParameterFile(config_file);
    auto dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    dataset_->Init();
    
    cv::Ptr<cv::GFTTDetector> gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    
    Frame::Ptr new_frame = dataset_->NextFrame();
    std::vector<cv::KeyPoint> keypoints;

    cv::Mat mask(new_frame->left_img_.size(), CV_8UC1, 255);
    
    for (auto &feat : new_frame->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED );
    }

    gftt_->detect(new_frame->left_img_, keypoints, mask);

    cout << keypoints.size() << endl;
    int cnt_detected=0;
    for (auto &kp : keypoints) {
        new_frame->features_left_.push_back(
            Feature::Ptr(new Feature(new_frame, kp)));
        cnt_detected++;
    }
    cout << cnt_detected << endl;

    cv::Mat img = PlotFrameImage(new_frame);
    cv::imshow("image", img);
    cv::waitKey(0);

}




TEST(FEATURE,Detech_RightImg_Features){
    std::string config_file = "/home/tomy807/streoSlam/config/default.yaml";
    Config::SetParameterFile(config_file);
    auto dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    dataset_->Init();
    
    cv::Ptr<cv::GFTTDetector> gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    
    Frame::Ptr new_frame = dataset_->NextFrame();
    std::vector<cv::KeyPoint> keypoints;

    cv::Mat mask(new_frame->left_img_.size(), CV_8UC1, 255);
    
    for (auto &feat : new_frame->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED );
    }

    gftt_->detect(new_frame->left_img_, keypoints, mask);

    cout << keypoints.size() << endl;
    int cnt_detected=0;
    for (auto &kp : keypoints) {
        new_frame->features_left_.push_back(
            Feature::Ptr(new Feature(new_frame, kp)));
        cnt_detected++;
    }
    FindFeaturesInRight(new_frame);
    cout << new_frame->features_right_.size() << endl;
    cv::Mat img1 = PlotFrameImage(new_frame);
    cv::Mat img2 = PlotFrameImageRight(new_frame);
    cv::imshow("imageLeft", img1);
    cv::imshow("imageRight", img2);
    cv::waitKey(0);
}

TEST(FEATURE,Triangular){
    std::string config_file = "/home/tomy807/streoSlam/config/default.yaml";
    Config::SetParameterFile(config_file);
    auto dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    dataset_->Init();
    
    cv::Ptr<cv::GFTTDetector> gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    
    Frame::Ptr new_frame = dataset_->NextFrame();
    std::vector<cv::KeyPoint> keypoints;

    cv::Mat mask(new_frame->left_img_.size(), CV_8UC1, 255);
    
    for (auto &feat : new_frame->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED );
    }

    gftt_->detect(new_frame->left_img_, keypoints, mask);

    cout << keypoints.size() << endl;
    int cnt_detected=0;
    for (auto &kp : keypoints) {
        new_frame->features_left_.push_back(
            Feature::Ptr(new Feature(new_frame, kp)));
        cnt_detected++;
    }
    FindFeaturesInRight(new_frame);
    Camera::Ptr camera_left_=dataset_->GetCamera(0);
    Camera::Ptr camera_right_=dataset_->GetCamera(1);
    
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    
    std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(new_frame->features_left_[0]->position_.pt.x,
                     new_frame->features_left_[0]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(new_frame->features_right_[0]->position_.pt.x,
                     new_frame->features_right_[0]->position_.pt.y))
    };
    
    cout << new_frame->features_right_.size() << endl;
    cout << new_frame->features_left_.size() << endl;

    Vec3 pworld = Vec3::Zero();

    triangulation(poses, points, pworld);
    cout << pworld << endl;

}