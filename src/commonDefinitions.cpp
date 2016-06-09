#include "commonDefinitions.h"

#include <iostream>
#include "axis_angle.h"

void transformation::matrix_to_vector()
{
    Eigen::Matrix3f mat = matrix_form.matrix().block<3,3>(0,0);
    Eigen::Vector3f angles;
}

void transformation::vector_to_matrix()
{

}
