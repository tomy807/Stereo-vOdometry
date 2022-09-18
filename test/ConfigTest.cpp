#include <gtest/gtest.h>
#include <iostream>
#include "myslam/config.h"

using namespace myslam;
using namespace std;
TEST(CONFIG,FILEOPEN){
    std::string config_file = "/home/tomy807/streoSlam/config/default.yaml";
    Config::SetParameterFile(config_file);
    EXPECT_EQ(Config::Get<double>("camera.fx"),718.856);
    EXPECT_EQ(Config::Get<double>("camera.fy"),718.856);
    EXPECT_EQ(Config::Get<double>("camera.cx"),607.1928);
    EXPECT_EQ(Config::Get<double>("camera.cy"),185.2157);
}