#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cfloat>
#include <cmath>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudT_Ptr;


#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

class camera_intrinsic_parameters
{
public:
    double cx;
    double cy;
    double fx;
    double fy;
    double scale;

    camera_intrinsic_parameters()
        : cx(0.0),
          cy(0.0),
          fx(1.0),
          fy(1.0),
          scale(1.0)
    {
    }

    bool operator !=(const camera_intrinsic_parameters& another)
    {
        return !(operator ==(another));
    }

    bool operator ==(const camera_intrinsic_parameters& another)
    {
        bool result = false;
        result = (cx==another.cx) && (cy==another.cy) && (fx==another.fx)
                && (fy==another.fy) && (scale==another.scale);
        return result;
    }
};


struct SixDegreeTransformation
{
    Eigen::Vector3f rotat_vec;
    Eigen::Vector3f trans_vec;

    SixDegreeTransformation()
    {
        rotat_vec = Eigen::Vector3f(.0f, .0f, .0f);
        trans_vec = Eigen::Vector3f(.0f, .0f, .0f);
    }

    SixDegreeTransformation(Eigen::Matrix4f &mat)
    {
        fromMatrix4f(mat);
    }

    void fromMatrix4f(Eigen::Matrix4f &mat);
    void toMatrix4f(Eigen::Matrix4f &mat);
};


struct ResultOfPNP
{
    SixDegreeTransformation transformation;
    int number_of_inliers;
};

#endif // COMMON_DEFINITIONS_H
