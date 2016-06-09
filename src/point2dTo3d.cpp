#include "point2dTo3d.h"

////////////////////////////////////////////////////////////
/// \brief point2dTo3d
/// convert 2d point in an rgb-d input to a 3d point with color information
/// \param rgbImage: rgb color image
/// \param depthImage: depth image
/// \param point2d: 2d point in the rgb-d image,
///                 [row, col]
/// \param point3d: point in the 3d space with color information
///                 point3d[0,1,2]=[x,y,z],
///                 point3d[3,4,5]=[r,g,b]
///
void point2dTo3d(cv::Mat rgbImage,
                 cv::Mat depthImage,
                 const camera_intrinsic_parameters& camera,
                 const unsigned *point2d,
                 double *point3d)
{
    unsigned m = point2d[0];
    unsigned n = point2d[1];
    // 获取深度图中(m,n)处的值
    ushort d = depthImage.ptr<ushort>(m)[n];
    // d 可能没有值，若如此，跳过此点
    if (d == 0)
    {
        point3d[0] = 0.0;
        point3d[1] = 0.0;
        point3d[2] = 0.0;
        return;
    }

    // 计算这个点的空间坐标
    point3d[2] = double(d) / camera.scale;
    point3d[0] = (n - camera.cx) * point3d[2] / camera.fx;
    point3d[1] = (m - camera.cy) * point3d[2] / camera.fy;

    // 从rgb图像中获取它的颜色
    // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
    point3d[5] = rgbImage.ptr<uchar>(m)[n*3];
    point3d[4] = rgbImage.ptr<uchar>(m)[n*3+1];
    point3d[3] = rgbImage.ptr<uchar>(m)[n*3+2];
}


////////////////////////////////////////////////////////////
/// \brief point2dTo3d
/// get 3d point position for the 2d query point in a depth image
/// \param depthImage
/// \param point2d: [row, col]
/// \param point3d: [x,y,z]
///
void point2dTo3d(cv::Mat depthImage,
                 const camera_intrinsic_parameters& camera,
                 const unsigned *point2d,
                 double *point3d)
{
    unsigned m = point2d[0];
    unsigned n = point2d[1];
    // 获取深度图中(m,n)处的值
    ushort d = depthImage.ptr<ushort>(m)[n];
    // d 可能没有值，若如此，跳过此点
    if (d == 0)
    {
        point3d[0] = 0.0;
        point3d[1] = 0.0;
        point3d[2] = 0.0;
        return;
    }

    // 计算这个点的空间坐标
    point3d[2] = double(d) / camera.scale;
    point3d[0] = (n - camera.cx) * point3d[2] / camera.fx;
    point3d[1] = (m - camera.cy) * point3d[2] / camera.fy;
}



////////////////////////////////////////////////////////////
/// \brief point2dTo3d
/// \param camera
/// \param point2d
/// \param depth
/// \param point3d
///
void point2dTo3d(const camera_intrinsic_parameters& camera,
                 const unsigned *point2d,
                 double depth,
                 double *point3d)
{
    unsigned m = point2d[0];
    unsigned n = point2d[1];
    // d 可能没有值，若如此，跳过此点
    if (depth == 0)
    {
        point3d[0] = 0.0;
        point3d[1] = 0.0;
        point3d[2] = 0.0;
        return;
    }

    // 计算这个点的空间坐标
    point3d[2] = double(depth) / camera.scale;
    point3d[0] = (n - camera.cx) * point3d[2] / camera.fx;
    point3d[1] = (m - camera.cy) * point3d[2] / camera.fy;
}


////////////////////////////////////////////////////////////
/// \brief point2dTo3d
/// \param input
/// \param output
/// \param camera
///
void point2dTo3d(const cv::Point3f &input,
                 cv::Point3f &output,
                 const camera_intrinsic_parameters& camera)
{
    output.z = double( input.z ) / camera.scale;
    output.x = ( input.x - camera.cx) * output.z / camera.fx;
    output.y = ( input.y - camera.cy) * output.z / camera.fy;
    return;
}

////////////////////////////////////////////////////////////
/// \brief point2dTo3d
/// \param input
/// \param output
/// \param camera
///
void point2dTo3d(Eigen::Vector3f &input,
                 Eigen::Vector3f &output,
                 const camera_intrinsic_parameters& camera)
{
    float x, y, z;
    z = double( input(2) ) / camera.scale;
    x = ( input(0) - camera.cx) * z / camera.fx;
    y = ( input(1) - camera.cy) * z / camera.fy;
    output = Eigen::Vector3f(x,y,z);

    return;
}
