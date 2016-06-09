#include "featureMatching.h"

#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

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
                     std::vector<cv::DMatch> &good_matches)
{
    using namespace std;

    std::vector< cv::DMatch > matches;
    cv::FlannBasedMatcher matcher;
    // 寻找匹配对
    matcher.match( descriptor1, descriptor2, matches );
    cout<<"Find total "<<matches.size()<<" matches."<<endl;

    // 筛选匹配，把距离太大的去掉
    // 这里使用的准则是去掉大于四倍最小距离的匹配
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 4*minDis)
            good_matches.push_back(matches[i]);
    }

}
