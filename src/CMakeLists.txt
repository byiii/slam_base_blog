add_executable(main main.cpp)
target_link_libraries(main
                      slam_base ${OpenCV_LIBS} ${PCL_LIBRARIES}
                      ${Libconfig_LIBS})


set(SLAM_BASE_FILES slambase.cpp
                    rgbdSource.cpp
                    visualOdometry.cpp
                    pointCloudFusion.cpp
                    visualizer.cpp
                    frame.cpp
                    slamparameters.cpp
                    point2dTo3d.cpp
                    commonDefinitions.cpp
                    generatePointCloud.cpp
                    featureExtraction.cpp
                    featureMatching.cpp
                    transformationEstimation.cpp)
add_library(slam_base ${SLAM_BASE_FILES})
target_link_libraries(slam_base
                      ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Libconfig_LIBS})
