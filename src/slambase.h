#ifndef SLAMBASE_H
#define SLAMBASE_H

#include "commonDefinitions.h"
#include "slamparameters.h"

#include "generatePointCloud.h"
#include "point2dTo3d.h"

#include "featureExtraction.h"
#include "featureMatching.h"
#include "transformationEstimation.h"

#include "frame.h"
#include "visualizer.h"

class slamBase
{
public:
    slamBase();
};

#endif // SLAMBASE_H
