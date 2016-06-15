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
        float voxel_grid;
        bool visualize_pointCloud;
        unsigned min_good_match;
        unsigned min_inliers;
        float max_norm;

        parameters()
        {
            start_index = 1;
            end_index = 1;
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
    void setImageSource(fileSource *f)
    {
        file_source = f;
    }

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
