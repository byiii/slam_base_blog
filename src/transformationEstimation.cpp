#include "transformationEstimation.h"

#include "point2dTo3d.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

////////////////////////////////////////////////////////////
/// \brief transformationEstimation
/// estimate the rigid 6-dof transformation bewteen to sequencial image
/// \param first_depth: input, depth image of the first frame
/// \param first_kps: input, keypoint of the first frame
/// \param second_kps: input, keypoint of the second frame
/// \param camera: input, intrinsic camera parameters
/// \param matches: input, description matches
/// \param rotationVec: output, rotation vector
/// \param translationVec: output, translation vector
/// \param inliers: output, inliers
///
void transformationEstimation(cv::Mat &first_depth,
                              std::vector<cv::KeyPoint> &first_kps,
                              std::vector<cv::KeyPoint> &second_kps,
                              camera_intrinsic_parameters &camera,
                              std::vector<cv::DMatch> &matches,
                              cv::Mat &rotationVec,
                              cv::Mat &translationVec,
                              cv::Mat &inliers)
{
    using namespace std;
    // 计算图像间的运动关系
    // 关键函数：cv::solvePnPRansac()
    // 为调用此函数准备必要的参数

    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    for (size_t i=0; i< matches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = first_kps[matches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = first_depth.ptr<ushort>(int(p.y))[int(p.x)];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( second_kps[matches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        unsigned *tmp_position = new unsigned[2];
        tmp_position[0] = static_cast<unsigned>(p.x);
        tmp_position[1] = static_cast<unsigned>(p.y);

        double tmp_xyz[3] = {0.0};
        point2dTo3d(camera, tmp_position, (double)d, tmp_xyz);

        cv::Point3f pd(tmp_xyz[0], tmp_xyz[1], tmp_xyz[2]);
        pts_obj.push_back(pd);
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0,         camera.cx},
        {0,         camera.fy, camera.cy},
        {0,         0,         1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(),
                        rotationVec, translationVec,
                        true, 100, 5.0, 100, inliers );

    cout << "inliers: " << inliers.rows << endl;
    cout << "R= " << rotationVec << endl;
    cout << "t= " << translationVec << endl;
}
