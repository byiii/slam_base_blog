#ifndef FRAME_H
#define FRAME_H

#include <opencv2/core/core.hpp>
#include "commonDefinitions.h"

////////////////////////////////////////////////////////////
/// \brief The frame class
///
class frame
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;
    typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudT_Ptr;

    struct parameters
    {
        std::string descriptor_type;
        std::string detector_type;

        parameters()
        {
            descriptor_type = "SIFT";
            detector_type = "SIFT";
        }
    };

protected:
    cv::Mat rgb;
    cv::Mat depth;
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    SixDegreeTransformation transformationToRF;

    PointCloudT_Ptr pointCloud;

    camera_intrinsic_parameters camera;
    parameters param;

public:
    frame()
    {
    }

    frame(const cv::Mat& in_rgb,
          const cv::Mat& in_depth,
          const camera_intrinsic_parameters& in_camera)
    {
        in_rgb.copyTo(this->rgb);
        in_depth.copyTo(this->depth);
        camera = in_camera;
    }

    ~frame()
    {
        this->release();
    }

    void release();

    void computePointCloud();
    void computeKeypointsAndDescriptors();
    void setParameters(const parameters p);

    PointCloudT_Ptr getPointCloud()
    {
        return this->pointCloud;
    }

    friend void estimateMotionFrameToFrame(frame& frame1,
                                           frame& frame2,
                                           ResultOfPNP& result,
                                           camera_intrinsic_parameters &camera);

    friend void estimateMotionFrameToFrame(frame& frame1,
                                           frame& frame2,
                                           ResultOfPNP& result);
};


////////////////////////////////////////////////////////////
/// \brief estimationMotionFrameToFrame
/// \param frame1
/// \param frame2
/// \param result
/// \param camera
///
void estimateMotionFrameToFrame(frame& frame1,
                                frame& frame2,
                                ResultOfPNP& result,
                                camera_intrinsic_parameters &camera);

////////////////////////////////////////////////////////////
/// \brief estimationMotionFrameToFrame
/// \param frame1
/// \param frame2
/// \param camera
///
void estimateMotionFrameToFrame(frame& frame1,
                                frame& frame2,
                                ResultOfPNP& result);

#endif // FRAME_H
