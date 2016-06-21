#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

#include "commonDefinitions.h"
#include "frame.h"

class loopClosure
{
public:
    loopClosure();
};

enum LOOPCLOSURE_CHECK_RESULT
{
    NOT_MATCHED=0,
    TOO_FAR_AWAY,
    TOO_CLOSE,
    KEYFRAME
};


// todo: solve the redundency of this function which is also in visualOdometry class
inline float temp_normofTransform(SixDegreeTransformation &transform)
{
    return fabs(fmin(transform.rotat_vec.norm(), (float)2*M_PI-transform.rotat_vec.norm()))
            + fabs(transform.trans_vec.norm());
}

LOOPCLOSURE_CHECK_RESULT checkKeyframes(float norm,
                                        bool is_loops = false);

void checkNearbyLoops(std::vector<frame> &frames,
                      frame &currentFrame,
                      g2o::SparseOptimizer &opt);

void checkRandomLoops(std::vector<frame> &frames,
                      frame &currentFrame,
                      g2o::SparseOptimizer &opt);

#endif // LOOPCLOSURE_H
