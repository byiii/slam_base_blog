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
    Eigen::Affine3f affine_transformation;

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

    frame& operator=(const frame &another_frame)
    {
        another_frame.rgb.copyTo(this->rgb);
        another_frame.depth.copyTo(this->depth);
        this->camera = another_frame.camera;

        return *this;
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

    friend void estimateMotion_3dTo2d(frame &frame1,
                                      frame &frame2,
                                      ResultOfPNP &result,
                                      camera_intrinsic_parameters &camera);

    friend void estimateMotion_3dTo2d(frame &frame1,
                                      frame &frame2,
                                      ResultOfPNP &result);

    friend void estimateMotion_3dTo3d(frame &frame1,
                                      frame &frame2,
                                      Eigen::Affine3f &affine);
};

////////////////////////////////////////////////////////////
void generateAFrame(frame &aframe,
                    const char* colorFile,
                    const char* depthFile,
                    const camera_intrinsic_parameters& camera);

////////////////////////////////////////////////////////////
void estimateMotion_3dTo2d(frame &frame1,
                           frame &frame2,
                           ResultOfPNP &result,
                           camera_intrinsic_parameters &camera);

////////////////////////////////////////////////////////////
void estimateMotion_3dTo2d(frame &frame1,
                           frame &frame2,
                           ResultOfPNP& result);

////////////////////////////////////////////////////////////
/// \brief estimateMotion_3dTo3d
/// align frame1 to frame2
/// \param frame1
/// \param frame2
/// \param affine
///
void estimateMotion_3dTo3d(frame &frame1,
                           frame &frame2,
                           Eigen::Affine3f &affine);

#endif // FRAME_H
