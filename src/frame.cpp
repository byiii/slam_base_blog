#include "frame.h"

#include <iostream>

#include "featureExtraction.h"
#include "featureMatching.h"
#include "transformationEstimation.h"

#include "generatePointCloud.h"

void frame::release()
{
    rgb.release();
    depth.release();
    descriptors.release();
    keypoints.clear();

    pointCloud->points.clear();
    pointCloud->clear();

    transformationToRF.release();
}


void frame::computePointCloud()
{
    pointCloud = generatePointCloud(this->rgb,
                                    this->depth,
                                    this->camera);
}

void frame::computeKeypointsAndDescriptors()
{
    using std::cout;
    using std::endl;

    int res = featureExtraction(rgb,
                                keypoints,
                                descriptors,
                                param.detector_type.c_str(),
                                param.descriptor_type.c_str());

    if(res==EXIT_FAILURE)
        cout << "failed to compute keypoints and descriptors." << endl;

    return;
}


void frame::setParameters(const parameters p)
{
    param.descriptor_type = p.descriptor_type;
    param.detector_type = p.detector_type;

    return;
}


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
                                  camera_intrinsic_parameters &camera)
{
    std::vector< cv::DMatch > matches;
    featureMatching(frame1.descriptors,
                    frame2.descriptors,
                    matches);

    cv::Mat rotationVec;
    cv::Mat translationVec;
    cv::Mat inliers;
    transformationEstimation(frame1.depth,
                             frame1.keypoints,
                             frame2.keypoints,
                             camera,
                             matches,
                             rotationVec,
                             translationVec,
                             inliers);

    rotationVec.copyTo(result.transformation.rotation_vector);
    translationVec.copyTo(result.transformation.translate_vector);
    result.number_of_inliers = inliers.rows;
}


////////////////////////////////////////////////////////////
/// \brief estimationMotionFrameToFrame
/// \param frame1
/// \param frame2
/// \param result
/// \param camera
///
void estimateMotionFrameToFrame(frame& frame1,
                                  frame& frame2,
                                  ResultOfPNP& result)
{
    if(frame1.camera!=frame2.camera)
    {
        std::cout << "error, frame1.camera does not match frame2.camera."
                  << std::endl;
        return;
    }
    std::vector< cv::DMatch > matches;
    featureMatching(frame1.descriptors,
                    frame2.descriptors,
                    matches);

    cv::Mat rotationVec;
    cv::Mat translationVec;
    cv::Mat inliers;
    transformationEstimation(frame1.depth,
                             frame1.keypoints,
                             frame2.keypoints,
                             frame1.camera,
                             matches,
                             rotationVec,
                             translationVec,
                             inliers);

    rotationVec.copyTo(result.transformation.rotation_vector);
    translationVec.copyTo(result.transformation.translate_vector);
    result.number_of_inliers = inliers.rows;
}
