#ifndef TRANSFORMATIONESTIMATION_H
#define TRANSFORMATIONESTIMATION_H

#include "commonDefinitions.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "frame.h"

////////////////////////////////////////////////////////////
/// \brief transformationEstimation
/// estimate the rigid 6-dof transformation bewteen to sequencial image
/// \param first_depth: input, depth image of the first frame
/// \param first_kps: input, keypoint of the first frame
/// \param second_kps: input, keypoint of the second frame
/// \param camera: input, intrinsic camera parameters
/// \param matches: input, description matches
/// \param rotationVec: output, rotation vector
/// \param translationVec: output, translation vector
/// \param inliers: output, inliers
///
void transformationEstimation(cv::Mat &first_depth,
                              std::vector<cv::KeyPoint> &first_kps,
                              std::vector<cv::KeyPoint> &second_kps,
                              camera_intrinsic_parameters &camera,
                              std::vector<cv::DMatch> &matches,
                              cv::Mat &rotationVec,
                              cv::Mat &translationVec,
                              cv::Mat &inliers);



#endif // TRANSFORMATIONESTIMATION_H
