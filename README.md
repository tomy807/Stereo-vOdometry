~~~
                                dataset.h
                                    ↑
common_include.h -> 1.camera.h -> frame.h ---|-------------|
                    2.config.h       |       |             |--->frontend.h---| 
                    3.feature.h      |       |---->map.h---|                 |
                    4.mappoint.h ------------|      ↓                       |----->visual_odometry.h
                    5.g2o_types.h    |----------->viewer.h-------------------|
~~~

# 1.Camera

fx,fy,cx,cy,pose

# 2.Config

File->dataset_dir,fx,fy,cx,cy,num_features,num_features_init,
num_features_tracking



# 3.Feature<Many>

Frame

KeyPoint(pixel)

MapPoint(3rdPose)

Outlier(T,F)

InImage(T,F)

    ↕

# 4.MapPoint<One>

Id

Outlier(T,F)

Pose

ObservedTime

vector<Feature>



# 5.Frame

Id

KeyFrameId

IsKeyFrame(T,F)

Pose

LeftImg

RightImg

vector<LeftFeature>

vector<RightFeature>


# 6.Dataset

Dataset_path

vector<Camera>(4)

Get_NextFrame


# 7.Map

Landmarks(map<MapPoint>)

KeyFrames(map<Frames>)

Num_Active_Keyframes_(show 7 boxes)


# 8.Frontend

CurrentFrame

LastFrame

LeftCamera

RightCamera


Tracking_inliers

Target=RelativeMotion(Last,Current)


Backend

Viewer

# 9.Visual_Odometry

Frontend

Backend

Map

Viewer

Dataset


VisualOdometry->Config->Dataset

Frontend(Dataset->Camera0,Dataset->Camera1)

