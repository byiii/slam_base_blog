#include "loopClosure.h"

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <cmath>

loopClosure::loopClosure()
{
}


LOOPCLOSURE_CHECK_RESULT checkKeyframes(float norm,
                                        bool is_loops)
{
    double max_norm = 0.3;
    double keyframe_threshold = 0.3;
    double max_norm_lp = 0.4;

    if(is_loops==false)
    {
        if(norm>=max_norm)
            return TOO_FAR_AWAY;
    }
    else
    {
        if(norm>=max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if(norm <= keyframe_threshold)
        return TOO_CLOSE; // too closed

    return KEYFRAME;
}

void checkNearbyLoops(std::vector<frame> &frames,
                      frame &currentFrame,
                      g2o::SparseOptimizer &opt)
{
    int nearby_loops = 10;

    if(frames.size()<=nearby_loops)
    {
        for(size_t i=0; i<frames.size(); ++i)
        {
            // 求解pnp
            Eigen::Affine3f affine;
            // align currFrame to lastFrame
            estimateMotion_3dTo3d(currentFrame, frames[i], affine);

            // 计算运动范围是否太大
            SixDegreeTransformation transform_vec(affine.matrix());
            float norm = temp_normofTransform(transform_vec);
            if(KEYFRAME == checkKeyframes(norm, true))
            {
                // todo add vertex and edge to g2o optimizer.
            }
        }
    }
    else
    {
        for(size_t i=frames.size()-nearby_loops; i<frames.size(); ++i)
        {
            // 求解pnp
            Eigen::Affine3f affine;
            // align currFrame to lastFrame
            estimateMotion_3dTo3d(currentFrame, frames[i], affine);

            // 计算运动范围是否太大
            SixDegreeTransformation transform_vec(affine.matrix());
            float norm = temp_normofTransform(transform_vec);
            if(KEYFRAME == checkKeyframes(norm, true))
            {
                // todo add vertex and edge to g2o optimizer.
            }
        }
    }
}

void checkRandomLoops(std::vector<frame> &frames,
                      frame &currentFrame,
                      g2o::SparseOptimizer &opt)
{
    int random_loops = 10;
    srand((unsigned)time(NULL));

    if(frames.size()<=random_loops)
    {
        for(size_t i=0; i<frames.size(); ++i)
        {
            // 求解pnp
            Eigen::Affine3f affine;
            // align currFrame to lastFrame
            estimateMotion_3dTo3d(currentFrame, frames[i], affine);

            // 计算运动范围是否太大
            SixDegreeTransformation transform_vec(affine.matrix());
            float norm = temp_normofTransform(transform_vec);
            if(KEYFRAME == checkKeyframes(norm, true))
            {
                // todo add vertex and edge to g2o optimizer.
            }
        }
    }
    else
    {
        for(int i=0; i<random_loops; ++i)
        {
            int index = rand()%frames.size();
            // 求解pnp
            Eigen::Affine3f affine;
            // align currFrame to lastFrame
            estimateMotion_3dTo3d(currentFrame, frames[index], affine);

            // 计算运动范围是否太大
            SixDegreeTransformation transform_vec(affine.matrix());
            float norm = temp_normofTransform(transform_vec);
            if(KEYFRAME == checkKeyframes(norm, true))
            {
                // todo add vertex and edge to g2o optimizer.
            }
        }
    }
}
