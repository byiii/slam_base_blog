#include <iostream>

#include "slambase.h"
#include "slamparameters.h"

#include <pcl/io/pcd_io.h>

const double camera_factor = 1000.0;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

int main(int argc, char ** argv)
{
    slamParameters param;

    param.configure("../config/parameters.cfg");
    param.displayParameters();

    return 0;
}

int main_tutorial3(int argc, char ** argv)
{
    using namespace std;

    // 设定相机内参数
    camera_intrinsic_parameters camera;
    camera.cx = camera_cx;
    camera.cy = camera_cy;
    camera.fx = camera_fx;
    camera.fy = camera_fy;
    camera.scale = camera_factor;

    // 声明并从data文件夹里读取两个rgb与深度图
    cv::Mat rgb1 = cv::imread( "../data/rgb1.png");
    cv::Mat rgb2 = cv::imread( "../data/rgb2.png");
    cv::Mat depth1 = cv::imread( "../data/depth1.png", -1);
    cv::Mat depth2 = cv::imread( "../data/depth2.png", -1);

    // 计算关键点和SIFT描述子
    vector< cv::KeyPoint > kp1, kp2; //关键点
    cv::Mat desp1, desp2;
    featureExtraction(rgb1, kp1, desp1);
    featureExtraction(rgb2, kp2, desp2);

    // 可视化， 显示关键点
    cv::Mat imgShow;
    cv::drawKeypoints( rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow( "keypoints", imgShow );
    cv::waitKey(0); //暂停等待一个按键

    // 匹配描述子
    vector< cv::DMatch > matches;
    featureMatching(desp1, desp2, matches);

    // 显示 good matches
    cv::Mat imgMatches;
    cout<<"good matches="<<matches.size()<<endl;
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
    cv::imshow( "good matches", imgMatches );
    cv::imwrite( "./data/good_matches.png", imgMatches );
    cv::waitKey(0);

    // 计算图像间的运动关系
    cv::Mat rotationVec;
    cv::Mat translationVec;
    cv::Mat inliers;

    transformationEstimation(depth1, kp1, kp2, camera, matches,
                             rotationVec, translationVec, inliers);

    // 画出inliers匹配
    vector< cv::DMatch > matchesShow;
    for (size_t i=0; i<inliers.rows; i++)
    {
        matchesShow.push_back( matches[inliers.ptr<int>(i)[0]] );
    }
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
    cv::imshow( "inlier matches", imgMatches );
    cv::waitKey( 0 );

    return 0;
}



int main_tutorial2(int argc, char ** argv)
{
    using std::cout;
    using std::endl;
    cout << "Hello, SLAM!" << endl;

    // 读取./data/rgb.png和./data/depth.png，并转化为点云
    // 图像矩阵
    cv::Mat rgb, depth;
    // 使用cv::imread()来读取图像
    rgb = cv::imread( "../data/rgb.png" );
    // rgb 图像是8UC3的彩色图像
    // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
    depth = cv::imread( "../data/depth.png", -1 );

    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloudT::Ptr cloud ( new PointCloudT );

    camera_intrinsic_parameters camera;
    camera.cx = camera_cx;
    camera.cy = camera_cy;
    camera.fx = camera_fx;
    camera.fy = camera_fy;
    camera.scale = camera_factor;

    cloud = generatePointCloud(rgb, depth, camera);

    pcl::io::savePCDFile( "../data/pointcloud.pcd", *cloud );
    // 清除数据并退出
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;

    return 0;
}

