SET(TEST_SOURCES ConfigTest CameraTest FrameTest FeatureTest Map_MapPointTest)

FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} ${test_src}.cpp)
    TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS} myslam)
    ADD_TEST(${test_src} ${test_src})
ENDFOREACH (test_src)
