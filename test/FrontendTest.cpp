#include <gtest/gtest.h>
#include <iostream>
#include "myslam/core.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

using namespace myslam;
using namespace std;


// Init-> Detect Left Features -> Optical Flow Right Features -> Triangular(Make Mappoints and Add Map)
// 1. Not KeyFrame
//  1-1. Optical Flow Currnet Left Features(form Last Left Features)
//  1-2. Estimate Current Pose
// 2. KeyFrame
//  2-1. Optical Flow Current Left Features(from Last Left Features)
//  2-2. Estimate Current Pose
//  2-3. Beacause Less Inliers, Detect Current Left Features 
//  2-4. Oprical Flow Right Features
//  2-5. Triangular and Make Mappoints and Add Map

TEST(FRONTEND,KeyFrame){

    std::string config_file = "/home/tomy807/streoSlam/config/default.yaml";
    Config::SetParameterFile(config_file);
    Dataset::Ptr dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    dataset_->Init();

    Frontend::Ptr frontend_ = Frontend::Ptr(new Frontend);
    Map::Ptr map_ = Map::Ptr(new Map);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
    frontend_->SetMap(map_);

    for(int i=0; i<7;i++){
        Frame::Ptr new_frame = dataset_->NextFrame();
        bool success = frontend_->AddFrame(new_frame);
    }
}