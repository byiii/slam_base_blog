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
#include "pointCloudFusion.h"
#include "rgbdSource.h"
#include "visualizer.h"
#include "axis_angle.h"

#include "loopClosure.h"

#include "visualOdometry.h"

class slamBase
{
public:
    slamBase();
};

#endif // SLAMBASE_H
