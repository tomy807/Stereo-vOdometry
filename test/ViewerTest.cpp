#include <gtest/gtest.h>
#include <iostream>
#include "myslam/core.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

using namespace myslam;
using namespace std;

TEST(Viwer,ShowFeatureImgs){
    std::string config_file = "/home/tomy807/streoSlam/config/default.yaml";
    Config::SetParameterFile(config_file);
    Dataset::Ptr dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    dataset_->Init();

    Frontend::Ptr frontend_ = Frontend::Ptr(new Frontend);
    Map::Ptr map_ = Map::Ptr(new Map);
    Viewer::Ptr viewer_ =Viewer::Ptr(new Viewer);
    
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    
    viewer_->SetMap(map_);

    while(1){
        Frame::Ptr new_frame = dataset_->NextFrame();
        if(new_frame==nullptr){
            break;
        }
        bool success = frontend_->AddFrame(new_frame);
        if(success==false){
            break;
        }
    }
    viewer_->Close();
    std::cout << "exit" << std::endl;
}