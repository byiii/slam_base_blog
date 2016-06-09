#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudT_Ptr;


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

    bool operator !=(const camera_intrinsic_parameters& another)
    {
        return !(operator ==(another));
    }

    bool operator ==(const camera_intrinsic_parameters& another)
    {
        bool result = false;
        result = (cx==another.cx) && (cy==another.cy) && (fx==another.fx)
                && (fy==another.fy) && (scale==another.scale);
        return result;
    }
};


struct SixDegreeTransformation
{
    cv::Mat rotation_vector;
    cv::Mat translate_vector;

    SixDegreeTransformation()
    {

    }
    ~SixDegreeTransformation()
    {
        this->release();
    }

    void release()
    {
        rotation_vector.release();
        translate_vector.release();
    }
};


struct ResultOfPNP
{
    SixDegreeTransformation transformation;
    int number_of_inliers;
};

#endif // COMMON_DEFINITIONS_H
