#include <iostream>

#include "commonDefinitions.h"
#include "generatePointCloud.h"

#include <pcl/io/pcd_io.h>

const double camera_factor = 1000.0;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

int main(int argc, char ** argv)
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

