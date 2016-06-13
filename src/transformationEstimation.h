#ifndef TRANSFORMATIONESTIMATION_H
#define TRANSFORMATIONESTIMATION_H

#include "commonDefinitions.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <Eigen/Dense>

////////////////////////////////////////////////////////////
/// \brief transformationEstimation_3dTo2d
/// \param first_depth
/// \param first_kps
/// \param second_kps
/// \param camera
/// \param matches
/// \param rotationVec
/// \param translationVec
/// \param inliers
///
void transformationEstimation_3dTo2d(const cv::Mat &first_depth,
                                     const std::vector<cv::KeyPoint> &first_kps,
                                     const std::vector<cv::KeyPoint> &second_kps,
                                     const camera_intrinsic_parameters &camera,
                                     const std::vector<cv::DMatch> &matches,
                                     cv::Mat &rotationVec,
                                     cv::Mat &translationVec,
                                     cv::Mat &inliers);


void transformationEstimation_3dTo3d(const cv::Mat &first_depth,
                                     const std::vector<cv::KeyPoint> &first_kps,
                                     const cv::Mat &second_depth,
                                     const std::vector<cv::KeyPoint> &second_kps,
                                     const camera_intrinsic_parameters &camera,
                                     const std::vector<cv::DMatch> &matches,
                                     Eigen::Affine3f &transformation);


///
/// TODO: add ransac based transformation estimation function with absolute orientation.
///

#endif // TRANSFORMATIONESTIMATION_H
