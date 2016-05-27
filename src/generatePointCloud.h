#ifndef GENERATE_POINTCLOUD_H
#define GENERATE_POINTCLOUD_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "commonDefinitions.h"

PointCloudT::Ptr generatePointCloud(const cv::Mat & rgbImage,
                                    const cv::Mat & depthImage,
                                    const camera_intrinsic_parameters &camera);


#endif // GENERATE_POINTCLOUD_H
