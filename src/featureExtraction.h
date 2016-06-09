#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

////////////////////////////////////////////////////////////
/// \brief featureExtraction
/// extract keypoints of the input image, and then extract the feature descriptors
/// at every keypoint. (default feature is SIFT)
/// \param img: input image
/// \param keypoints: output
/// \param descriptors: output, sift feature descriptor as default.
///
void featureExtraction(cv::Mat &img,
                       std::vector<cv::KeyPoint> &keypoints,
                       cv::Mat &descriptors);

////////////////////////////////////////////////////////////
/// \brief featureExtraction
/// \param img
/// \param keypoints
/// \param descriptors
/// \param detector_type
/// \param descriptor_type
///
int featureExtraction(cv::Mat &img,
                       std::vector<cv::KeyPoint> &keypoints,
                       cv::Mat &descriptors,
                       const char* detector_type,
                       const char* descriptor_type);

#endif // FEATUREEXTRACTOR_H
