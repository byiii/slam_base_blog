set(G2O_LIBS g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension)

add_executable(main main.cpp)
target_link_libraries(main
                      slam_base ${OpenCV_LIBS} ${PCL_LIBRARIES}
                      ${Libconfig_LIBS} ${G2O_LIBS} ${CSPARSE_LIBRARY})


set(SLAM_BASE_FILES slambase.cpp
                    axis_angle.cpp
                    axis_angle.hpp
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
                      ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Libconfig_LIBS} ${G2O_LIBS} ${CSPARSE_LIBRARY})
