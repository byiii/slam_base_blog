#include "generatePointCloud.h"

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>

PointCloudT::Ptr generatePointCloud(const cv::Mat & rgbImage,
                                    const cv::Mat & depthImage,
                                    const camera_intrinsic_parameters &camera)
{
    using std::cout;
    using std::endl;

    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloudT::Ptr cloud ( new PointCloudT );
    // 遍历深度图
    for (int m = 0; m < depthImage.rows; m++)
        for (int n=0; n < depthImage.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depthImage.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgbImage.ptr<uchar>(m)[n*3];
            p.g = rgbImage.ptr<uchar>(m)[n*3+1];
            p.r = rgbImage.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;

    return cloud;
}


