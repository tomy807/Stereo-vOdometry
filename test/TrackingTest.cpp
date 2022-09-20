#include <gtest/gtest.h>
#include <iostream>
#include "myslam/core.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>


using namespace myslam;
using namespace std;


int FindFeaturesInRight(Frame::Ptr second_frame_) {
    std::vector<cv::Point2f> kps_left, kps_right;
    // use LK flow to estimate points in the right image
    for (auto &kp : second_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
        // if dont Fill in kps_rights error
        kps_right.push_back(kp->position_.pt);
    }
    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        second_frame_->left_img_, second_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(second_frame_, kp));
            feat->is_on_left_image_ = false;
            second_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            second_frame_->features_right_.push_back(nullptr);
        }
    }
    cout << "Find " << num_good_pts << " in the right image." << endl;
    return num_good_pts;
}


cv::Mat PlotFrameImage(Frame::Ptr frame) {
    cv::Mat img_out;
    cv::cvtColor(frame->left_img_, img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < frame->features_left_.size(); ++i) {
        auto feat = frame->features_left_[i];
        if(i==0){
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(255, 0, 0),2);
        }else{
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),2);

        }
    }
    return img_out;
}





TEST(TRACKING,MakeLastFrame){
    std::string config_file = "/home/tomy807/streoSlam/config/default.yaml";
    Config::SetParameterFile(config_file);
    Dataset::Ptr dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    dataset_->Init();
    Camera::Ptr camera_left_=dataset_->GetCamera(0);
    Camera::Ptr camera_right_=dataset_->GetCamera(1);
    cv::Ptr<cv::GFTTDetector> gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    
// Frist Frame
    Frame::Ptr first_frame = dataset_->NextFrame();
    cout << "Frame " << first_frame->id_ << "--------------------" << endl;
    std::cout << first_frame->Pose().matrix() << std::endl;
    std::vector<cv::KeyPoint> keypoints;

// Last Frame Left Features(Detect)
    cv::Mat mask(first_frame->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : first_frame->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED );
    }
    gftt_->detect(first_frame->left_img_, keypoints, mask);

    int cnt_detected=0;
    for (auto &kp : keypoints) {
        first_frame->features_left_.push_back(Feature::Ptr(new Feature(first_frame, kp)));
        cnt_detected++;
    }
    cout << "Find " << cnt_detected << " in the left image" << endl;
// Last Frame Right Features(calcOpticalFlowPyrLK)
    FindFeaturesInRight(first_frame);

// Triangulation And Mapping on the Map
    Map::Ptr map_ = Map::Ptr(new Map());

    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};

    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < first_frame->features_left_.size(); ++i) {
        if (first_frame->features_right_[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(first_frame->features_left_[i]->position_.pt.x,
                     first_frame->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(first_frame->features_right_[i]->position_.pt.x,
                     first_frame->features_right_[i]->position_.pt.y))
        };
        Vec3 pworld = Vec3::Zero();

        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(first_frame->features_left_[i]);
            new_map_point->AddObservation(first_frame->features_right_[i]);
            first_frame->features_left_[i]->map_point_ = new_map_point;
            first_frame->features_right_[i]->map_point_ = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
    }

    first_frame->SetKeyFrame();
    map_->InsertKeyFrame(first_frame);

    cout <<"First Frame Landmarks: " << cnt_init_landmarks << endl;
    auto c1 = map_->GetAllMapPoints();

    for (auto it = c1.begin();it != c1.end(); ++it)
        std::cout << it->first << std::endl << "[" << it->second.get()->Pos().transpose() << "]" << std::endl;
    std::cout << std::endl;
//------------------------------------------------------------------------------------------
// Second Frame
    Frame::Ptr second_frame = dataset_->NextFrame();
    cout  << "Frame " << second_frame->id_ << "--------------------" << endl;
    // Current Frame Left Features(calcOpticalFlowPyrLK) using Last Frame Left Features
    std::vector<cv::Point2f> kps_last, kps_current;

    for (auto &kp : first_frame->features_left_) {
        if (kp->map_point_.lock()) {
            // use project point
            auto mp = kp->map_point_.lock();
            auto px = camera_left_->world2pixel(mp->pos_, second_frame->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        first_frame->left_img_, second_frame->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(second_frame, kp));
            feature->map_point_ = first_frame->features_left_[i]->map_point_;
            second_frame->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    cout << "Find " << num_good_pts << " in the last image." << endl;

    cv::Mat last_img=PlotFrameImage(first_frame);
    cv::Mat current_img=PlotFrameImage(second_frame);

    cv::imshow("LastImg", last_img);
    cv::imshow("CurrentImg", current_img);
    cv::waitKey(0);


// Calcuate Current Pose
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(second_frame->Pose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < second_frame->features_left_.size(); ++i) {
        auto mp = second_frame->features_left_[i]->map_point_.lock();
        if (mp) {
            features.push_back(second_frame->features_left_[i]);
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(toVec2(second_frame->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(second_frame->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    second_frame->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << second_frame->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }

    auto c2 = map_->GetAllMapPoints();

    for (auto it = c2.begin();it != c2.end(); ++it)
        std::cout << it->first << std::endl << "[" << it->second.get()->Pos().transpose() << "]" << std::endl;
    std::cout << std::endl;

}