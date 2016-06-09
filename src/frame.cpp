#include "frame.h"

#include <iostream>

#include "featureExtraction.h"
#include "featureMatching.h"
#include "transformationEstimation.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "generatePointCloud.h"

void frame::release()
{
    rgb.release();
    depth.release();
    descriptors.release();
    keypoints.clear();

    pointCloud->points.clear();
    pointCloud->clear();

    poseInVectors.release();
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
void estimateMotion_3dTo2d(frame& frame1,
                           frame& frame2,
                           ResultOfPNP& result,
                           camera_intrinsic_parameters &camera)
{
    using std::cout;
    using std::endl;

    std::vector< cv::DMatch > matches;
    featureMatching(frame1.descriptors,
                    frame2.descriptors,
                    matches);

    // 显示 good matches
    cv::Mat imgMatches;
    cout<<"good matches="<<matches.size()<<endl;
    cv::drawMatches(frame1.rgb, frame1.keypoints,
                    frame2.rgb, frame2.keypoints,
                    matches,
                    imgMatches);
    cv::imshow( "good matches", imgMatches );
    cv::imwrite( "./data/good_matches.png", imgMatches );
    cv::waitKey(0);

    cv::Mat rotationVec;
    cv::Mat translationVec;
    cv::Mat inliers;
    transformationEstimation_3dTo2d(frame1.depth,
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

    cout << "rotation: " << result.transformation.rotation_vector << endl
         << "translation: " << result.transformation.translate_vector << endl;
}


////////////////////////////////////////////////////////////
void estimateMotion_3dTo2d(frame& frame1,
                           frame& frame2,
                           ResultOfPNP& result)
{
    using std::cout;
    using std::endl;

    if(frame1.camera!=frame2.camera)
    {
        std::cout << "error, frame1.camera does not match frame2.camera."
                  << std::endl;
        return;
    }

    estimateMotion_3dTo2d(frame1,
                          frame2,
                          result,
                          frame1.camera);
}


////////////////////////////////////////////////////////////
void estimateMotion_3dTo3d(frame &frame1,
                           frame &frame2,
                           Eigen::Affine3f &affine)
{
    using std::cout;
    using std::endl;

    std::vector< cv::DMatch > matches;
    featureMatching(frame1.descriptors,
                    frame2.descriptors,
                    matches);

    transformationEstimation_3dTo3d(frame1.depth,
                                    frame1.keypoints,
                                    frame2.depth,
                                    frame2.keypoints,
                                    frame1.camera,
                                    matches,
                                    affine);
}
