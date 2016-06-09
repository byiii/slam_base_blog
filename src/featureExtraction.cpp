#include <iostream>
#include "featureExtraction.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

////////////////////////////////////////////////////////////
/// \brief FeatureExtraction
/// extract keypoints of the input image, and then extract the feature descriptors
/// at every keypoint. (default feature is SIFT)
/// \param img: input image
/// \param keypoints: output
/// \param descriptors: output, sift feature descriptor as default.
///
void featureExtraction(cv::Mat &img,
                       std::vector<cv::KeyPoint> &keypoints,
                       cv::Mat &descriptors)
{
    using std::cout;
    using std::endl;

    // 声明特征提取器与描述子提取器
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    // 构建提取器，默认两者都为sift
    // 构建sift, surf之前要初始化nonfree模块
    cv::initModule_nonfree();
    _detector = cv::FeatureDetector::create("SIFT");
    _descriptor = cv::DescriptorExtractor::create("SIFT");

    _detector->detect(img, keypoints);  //提取关键点

    cout << "Key points of the input image: "<< keypoints.size() << endl;

    // 计算描述子
    _descriptor->compute(img, keypoints, descriptors);

    cout << "descriptor' size: " << descriptors.size() << endl;
}

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
                       const char* descriptor_type)
{
    using std::cout;
    using std::endl;

    // 声明特征提取器与描述子提取器
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    // 构建提取器，默认两者都为sift
    // 构建sift, surf之前要初始化nonfree模块
    cv::initModule_nonfree();
    _detector = cv::FeatureDetector::create(detector_type);
    _descriptor = cv::DescriptorExtractor::create(descriptor_type);

    if(!_detector)
    {
        cout << "Error, unsupported detector type."
             << endl;
        return EXIT_FAILURE;
    }
    if(!_descriptor)
    {
        cout << "Error, unsupported descriptor type."
             << endl;
        return EXIT_FAILURE;
    }

    _detector->detect(img, keypoints);  //提取关键点
    cout << "Key points of the input image: "<< keypoints.size() << endl;

    // 计算描述子
    _descriptor->compute(img, keypoints, descriptors);

    cout << "descriptor' size: " << descriptors.size() << endl;

    return EXIT_SUCCESS;
}
