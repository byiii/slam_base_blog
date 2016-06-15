set(G2O_LIBS
g2o_cli
g2o_core
g2o_csparse_extension
g2o_ext_csparse
g2o_ext_freeglut_minimal
g2o_incremental
g2o_interactive
g2o_interface
g2o_opengl_helper
g2o_parser
g2o_simulator
g2o_solver_cholmod
g2o_solver_csparse
g2o_solver_dense
g2o_solver_eigen
g2o_solver_pcg
g2o_solver_slam2d_linear
g2o_solver_structure_only
g2o_stuff
g2o_types_data
g2o_types_icp
g2o_types_sba
g2o_types_sclam2d
g2o_types_sim3
g2o_types_slam2d_addons
g2o_types_slam2d
g2o_types_slam3d_addons
g2o_types_slam3d
g2o_viewer
camd
cxsparse
csparse
cholmod
ccolamd
suitesparseconfig)

add_executable(main main.cpp)
target_link_libraries(main
                      slam_base ${OpenCV_LIBS} ${PCL_LIBRARIES}
                      ${Libconfig_LIBS}
${G2O_LIBS}
)


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
                      ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Libconfig_LIBS}
${G2O_LIBS}
)
