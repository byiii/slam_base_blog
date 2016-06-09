#ifndef POINT2DTO3D_H
#define POINT2DTO3D_H

#include "commonDefinitions.h"

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

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
                 double *point3d);


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
                 double *point3d);


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
                 double *point3d);


////////////////////////////////////////////////////////////
/// \brief point2dTo3d
/// \param input
/// \param output
/// \param camera
///
void point2dTo3d(const cv::Point3f &input,
                 cv::Point3f &output,
                 const camera_intrinsic_parameters& camera);

////////////////////////////////////////////////////////////
/// \brief point2dTo3d
/// \param input
/// \param output
/// \param camera
///
void point2dTo3d(Eigen::Vector3f &input,
                 Eigen::Vector3f &output,
                 const camera_intrinsic_parameters& camera);

#endif // POINT2DTO3D_H
