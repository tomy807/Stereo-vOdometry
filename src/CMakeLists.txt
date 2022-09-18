add_library(myslam SHARED
        config.cpp
        camera.cpp
        dataset.cpp
        frame.cpp
        )
find_package( Sophus REQUIRED )
include_directories("/usr/local/include/sophus")
target_link_libraries(myslam
        ${THIRD_PARTY_LIBS} Sophus::Sophus)
