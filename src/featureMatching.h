#ifndef FEATUREMATCHING_H
#define FEATUREMATCHING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

////////////////////////////////////////////////////////////
/// \brief featureMatching
/// match feature descriptors, filter out those whose matching descriptors'
/// distance is two large.
/// \param descriptor1: input
/// \param descriptor2: input
/// \param good_matches: output
///
void featureMatching(cv::Mat &descriptor1,
                     cv::Mat &descriptor2,
                     std::vector<cv::DMatch> &good_matches);

#endif // FEATUREMATCHING_H
