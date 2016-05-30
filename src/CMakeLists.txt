add_executable(main main.cpp)
target_link_libraries(main slam_base ${OpenCV_LIBS} ${PCL_LIBRARIES})


set(SLAM_BASE_FILES slambase.cpp
                    slamparameters.cpp
                    configparameters.cpp
                    point2dTo3d.cpp
                    commonDefinitions.cpp
                    generatePointCloud.cpp)
add_library(slam_base ${SLAM_BASE_FILES})
target_link_libraries(slam_base ${OpenCV_LIBS} ${PCL_LIBRARIES})
