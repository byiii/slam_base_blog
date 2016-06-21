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
    float generateTime;
    cv::Mat rgb;
    cv::Mat depth;
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;

    Eigen::Isometry3f pose;

    PointCloudT_Ptr pointCloud;

    camera_intrinsic_parameters camera;
    parameters param;

public:
    frame()
    {
    }

    frame(const cv::Mat& in_rgb,
          const cv::Mat& in_depth,
          const camera_intrinsic_parameters& in_camera,
          float frame_time)
    {
        generateTime = frame_time;
        in_rgb.copyTo(this->rgb);
        in_depth.copyTo(this->depth);
        camera = in_camera;

        pointCloud.reset(new PointCloudT);
    }

    frame& operator=(const frame &another_frame)
    {
        another_frame.rgb.copyTo(this->rgb);
        another_frame.depth.copyTo(this->depth);
        this->camera = another_frame.camera;
        this->generateTime = another_frame.generateTime;
        this->pointCloud = another_frame.pointCloud->makeShared();
        another_frame.descriptors.copyTo(this->descriptors);
        this->keypoints = another_frame.keypoints;

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
    PointCloudT_Ptr getPointCloudCopy()
    {
        return this->pointCloud->makeShared();
    }
    camera_intrinsic_parameters getCamera()
    {
        return this->camera;
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
