#include <iostream>

#include "slambase.h"
#include "slamparameters.h"

#include <pcl/io/pcd_io.h>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/common/transforms.h>

int main( int argc, char** argv )
{
    using std::cout;
    using std::endl;

    //本节要拼合data中的两对图像
    slamParameters param;
    param.configure("../config/parameters.cfg");

    // camera model
    camera_intrinsic_parameters camera;
    camera.cx = param.camera_cx;
    camera.cy = param.camera_cy;
    camera.fx = param.camera_fx;
    camera.fy = param.camera_fy;
    camera.scale = param.camera_factor;

    frame::parameters frame_param;
    frame_param.descriptor_type = "SIFT";
    frame_param.detector_type = "SIFT";

    //读取图像
    // 声明并从data文件夹里读取两个rgb与深度图
    cv::Mat rgb1 = cv::imread( "../data/rgb1.png");
    cv::Mat rgb2 = cv::imread( "../data/rgb2.png");
    cv::Mat depth1 = cv::imread( "../data/depth1.png", -1);
    cv::Mat depth2 = cv::imread( "../data/depth2.png", -1);

    // 声明两个帧，FRAME结构请见include/slamBase.h
    frame frame1(rgb1,depth1, camera);
    frame1.setParameters(frame_param);
    frame frame2(rgb2, depth2, camera);
    frame2.setParameters(frame_param);

    // compute point cloud
    frame1.computePointCloud();
    frame2.computePointCloud();

    // 提取特征并计算描述子
    cout<<"extracting features"<<endl;
    frame1.computeKeypointsAndDescriptors();
    frame2.computeKeypointsAndDescriptors();

    cout<<"solving pnp"<<endl;
    // 求解pnp
    ResultOfPNP result;
    estimateMotionFrameToFrame(frame1, frame2, result);

    cout << "rotation: " << result.transformation.rotation_vector << endl
         << "translation: " << result.transformation.translate_vector << endl;

    // 处理result
    // 将旋转向量转化为旋转矩阵
    cv::Mat R;
    cv::Rodrigues(result.transformation.rotation_vector, R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    cout<<"translation"<<endl;
    Eigen::Translation<double,3> trans(result.transformation.translate_vector.at<double>(0,0),
                                       result.transformation.translate_vector.at<double>(0,1),
                                       result.transformation.translate_vector.at<double>(0,2));
    T = angle;
    T(0,3) = result.transformation.translate_vector.at<double>(0,0);
    T(1,3) = result.transformation.translate_vector.at<double>(0,1);
    T(2,3) = result.transformation.translate_vector.at<double>(0,2);

    // 转换点云
    cout<<"converting image to clouds"<<endl;
    frame::PointCloudT_Ptr cloud1 = frame1.getPointCloud();
    frame::PointCloudT_Ptr cloud2 = frame2.getPointCloud();

    // 合并点云
    cout<<"combining clouds"<<endl;
    frame::PointCloudT_Ptr output (new pcl::PointCloud<frame::PointT>());
    pcl::transformPointCloud( *cloud2, *output, T.matrix() );
    *output += *cloud1;
    pcl::io::savePCDFile("../data/result.pcd", *output);
    cout<<"Final result saved."<<endl;

    visualizer_simple viewer("view");
    viewer.showPointCloud(output);
}

int main_param(int argc, char ** argv)
{
    slamParameters param;

    param.configure("../config/parameters.cfg");
    param.displayParameters();

    return 0;
}


float camera_cx=325.5;
float camera_cy=253.5;
float camera_fx=518.0;
float camera_fy=519.0;
float camera_factor=1000.0;

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

