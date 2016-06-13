#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "commonDefinitions.h"
#include "rgbdSource.h"

class visualOdometry
{
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
        char *source_dir;
        float voxel_grid;
        bool visualize_pointCloud;
        unsigned min_good_match;
        unsigned min_inliers;
        float max_norm;

        parameters()
        {
            start_index = 1;
            end_index = 1;
            source_dir = "../data";
            rgb_dir = source_dir;
            depth_dir = source_dir;
            rgb_marker = "rgb";
            depth_marker = "depth";
            rgb_extension = ".png";
            depth_extension = ".png";

            voxel_grid = 0.003;
            visualize_pointCloud = true;
            min_good_match = 10;
            min_inliers = 8;
            max_norm = 0.3;
        }
    };

protected:
    fileSource *file_source;
    camera_intrinsic_parameters camera;
    parameters param;

public:
    visualOdometry();

    void run();
    void run_with_g2o();
    void setCamera(const camera_intrinsic_parameters &c)
    {
        camera = c;
    }

    void setParams(const visualOdometry::parameters &p)
    {
        param = p;
    }

    float normofTransform(SixDegreeTransformation &transform);
};

#endif // VISUALODOMETRY_H
