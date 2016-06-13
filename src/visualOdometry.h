#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "commonDefinitions.h"
#include "rgbdSource.h"

class visualOdometry
{
    fileSource *file_source;
    camera_intrinsic_parameters camera;
public:
    struct parameters
    {
        unsigned start_index;
        unsigned end_index;
        char *rgb_dir;
        char *depth_dir;
        char *rgb_marker;
        char *depth_marker;
        char *rgb_extension;
        char *depth_extension;
        float voxel_grid;
        bool visualize_pointCloud;
        unsigned min_good_match;
        unsigned min_inliers;
        float max_norm;
    };

    visualOdometry();

    void run();
    float normofTransform(SixDegreeTransformation &transform);
};

#endif // VISUALODOMETRY_H
