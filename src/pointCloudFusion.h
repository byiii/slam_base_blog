#ifndef POINTCLOUDFUSION_H
#define POINTCLOUDFUSION_H

#include "commonDefinitions.h"
#include <Eigen/Geometry>

class pointCloudFusion
{
public:
    pointCloudFusion();
};

void fusingPointCloud(PointCloudT &base,
                      PointCloudT &addition,
                      Eigen::Affine3f &transformation);

void fusingPointCloud(PointCloudT &in_base,
                      PointCloudT &in_addition,
                      PointCloudT &output,
                      Eigen::Affine3f &transformation);

#endif // POINTCLOUDFUSION_H
