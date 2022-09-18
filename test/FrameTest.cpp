#include <gtest/gtest.h>
#include <iostream>
#include "myslam/dataset.h"
#include "myslam/config.h"

using namespace myslam;
using namespace std;

TEST(DATASET,GetImagesAndCameras){
    // Get Dataset directory From Config
    std::string config_file = "/home/tomy807/streoSlam/config/default.yaml";
    Config::SetParameterFile(config_file);
    string dataset_path =  Config::Get<string>("dataset_dir");

    Dataset InitData(dataset_path);
    InitData.Init();

    Camera::Ptr camera0=InitData.GetCamera(0);
    cout << camera0->pose().matrix3x4()<< endl;

    Camera::Ptr camera1=InitData.GetCamera(1);
    cout << camera1->pose().matrix3x4() << endl;

    auto new_frame0=InitData.NextFrame();
    cout << new_frame0->id_ << endl;
    cout << new_frame0->left_img_.size() << endl;

    auto new_frame1=InitData.NextFrame();
    cout << new_frame1->id_ << endl;
    cout << new_frame1->left_img_.size() << endl;

}

TEST(DATASET,GetImagesAll){
    std::string config_file = "/home/tomy807/streoSlam/config/default.yaml";
    Config::SetParameterFile(config_file);
    auto dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    dataset_->Init();
    for(int i=0; i<10;i++){
        Frame::Ptr new_frame = dataset_->NextFrame();
        cout << new_frame->id_ << endl;
    }

}