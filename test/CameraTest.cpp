#include <gtest/gtest.h>
#include <iostream>
#include "myslam/camera.h"


using namespace myslam;
using namespace std;

TEST(CAMERA,CAMETA_INTRINSICS){
    double fx= 718.856;
    double fy= 718.856;
    double cx= 607.1928;
    double cy= 185.2157;

    Vec3 t;
    t << 1.,2.,3.;
    Camera::Ptr newCamera(new Camera(fx,fy,fx,cy,0,SE3(SO3(),t)));

    cout <<"Camera Pose: "<< endl << newCamera->pose().matrix() << endl;

    cout << endl;

    cout << "Camera Intrinsic: "<< endl << newCamera->K().matrix() << endl;
}

TEST(CAMERA,CAMERA_CALIBRATION){
    double fx= 718.856;
    double fy= 718.856;
    double cx= 607.1928;
    double cy= 185.2157;

    Vec3 t;
    t << 1.,2.,3.;
    Camera::Ptr newCamera(new Camera(fx,fy,fx,cy,0,SE3(SO3(),t)));

//  Point x,y,z 
    Vec3 p_w;
    p_w << 3.,4.,5.;
    
    Vec3 t1;
    t1 << 6.,7.,8.;
    Mat33 so3;
    so3 << 0.,-1.,0.,
           1.,0.,0.,
           0.,0.,1.;

// Frame Pose
    SE3 T_c_w(SO3(so3),t1);

    std::cout <<newCamera->world2pixel(p_w,T_c_w) << std::endl;
    std::cout <<newCamera->world2camera(p_w,T_c_w) << std::endl;
}