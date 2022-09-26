add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        viewer.cpp
        dataset.cpp
        )
find_package( Sophus REQUIRED )
include_directories("/usr/local/include/sophus")
target_link_libraries(myslam
        ${THIRD_PARTY_LIBS} Sophus::Sophus)
