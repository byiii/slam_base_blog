#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class camera_intrinsic_parameters
{
public:
    double cx;
    double cy;
    double fx;
    double fy;
    double scale;

    camera_intrinsic_parameters()
        : cx(0.0),
          cy(0.0),
          fx(1.0),
          fy(1.0),
          scale(1.0)
    {
    }
};

#endif // COMMON_DEFINITIONS_H
