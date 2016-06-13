#include "commonDefinitions.h"
#include "axis_angle.h"

void SixDegreeTransformation::fromMatrix4f(Eigen::Matrix4f &mat)
{
    this->trans_vec(0) = mat(0,3);
    this->trans_vec(1) = mat(1,3);
    this->trans_vec(2) = mat(2,3);

    Eigen::Matrix3f rotmat = mat.block<3,3>(0,0);
    rotationMatrixToEulerAngle_XYZ(rotmat, this->rotat_vec);
}

void SixDegreeTransformation::toMatrix4f(Eigen::Matrix4f &mat)
{
    Eigen::Matrix3f r;
    eulerAnglesToRotationMatrix_XYZ(this->rotat_vec, r);

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    Eigen::AngleAxisf angle(r);
    T = angle;
    T(0,3) = this->trans_vec(0);
    T(1,3) = this->trans_vec(1);
    T(2,3) = this->trans_vec(2);

    mat = T.matrix();
}
