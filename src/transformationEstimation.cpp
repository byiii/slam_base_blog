#include "transformationEstimation.h"

#include "point2dTo3d.h"

#include <opencv2/core/eigen.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/transforms.h>

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
void transformationEstimation_3dTo2d(const cv::Mat &first_depth,
                                     const std::vector<cv::KeyPoint> &first_kps,
                                     const std::vector<cv::KeyPoint> &second_kps,
                                     const camera_intrinsic_parameters &camera,
                                     const std::vector<cv::DMatch> &matches,
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
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd;
        point2dTo3d(pt, pd, camera);
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
                        true, 100, 10.0, 100, inliers );

    cout << "inliers: " << inliers.rows << endl;
    cout << "R= " << rotationVec << endl;
    cout << "t= " << translationVec << endl;
}


void transformationEstimation_3dTo3d(const cv::Mat &first_depth,
                                     const std::vector<cv::KeyPoint> &first_kps,
                                     const cv::Mat &second_depth,
                                     const std::vector<cv::KeyPoint> &second_kps,
                                     const camera_intrinsic_parameters &camera,
                                     const std::vector<cv::DMatch> &matches,
                                     Eigen::Affine3f &transformation)
{
    using namespace std;
    // 计算图像间的运动关系

    pcl::TransformationFromCorrespondences estimator;

    for (size_t i=0; i< matches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p_first = first_kps[matches[i].queryIdx].pt;
        cv::Point2f p_second = second_kps[matches[i].trainIdx].pt;

        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d_first = first_depth.ptr<ushort>(int(p_first.y))[int(p_first.x)];
        ushort d_second = second_depth.ptr<ushort>(int(p_second.y))[int(p_second.x)];
        if (d_first == 0 || d_second==0)
            continue;

        // 将(u,v,d)转成(x,y,z)
        Eigen::Vector3f tmp1(p_first.x, p_first.y, d_first);
        Eigen::Vector3f first_vec;
        point2dTo3d(tmp1, first_vec, camera);

        tmp1 = Eigen::Vector3f(p_second.x, p_second.y, d_second);
        Eigen::Vector3f second_vec;
        point2dTo3d(tmp1, second_vec, camera);

        estimator.add(first_vec, second_vec);
    }

    transformation = estimator.getTransformation();

    cout << "pcl transformation: \n" << transformation.matrix() << endl
         << "number of correspondences: " << estimator.getNoOfSamples() << endl;
}
