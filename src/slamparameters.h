#ifndef SLAMPARAMETERS_H
#define SLAMPARAMETERS_H

#include <vector>

class slamParameters
{
public:
    //------------------------------------------------------------
    // default camera intrinsic parameters
    double camera_factor;
    double camera_cx;
    double camera_cy;
    double camera_fx;
    double camera_fy;

    //------------------------------------------------------------
    //camera distortion parameters
    // vector of distortion coefficients
    // (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements.
    // If the vector is NULL/empty, the zero distortion coefficients are assumed.
    std::vector<float> camera_distCoeffs;

    //------------------------------------------------------------
    // matches whose distance ratio to the minimum distance among all matches
    // is smaller than this threshold will be regard as a goos match.
    int match_goodMatchThreshold;

    //------------------------------------------------------------
    // pnp ransac solver parameters

    // number of the max ransac iterations
    int ransac_iterationsCount;

    // Inlier threshold value used by the RANSAC procedure. The parameter value
    // is the maximum allowed distance between the observed and computed point
    // projections to consider it an inlier.
    float ransac_reprojectionError;

    // Number of inliers. If the algorithm at some stage finds more inliers than
    // this number, it finishes.
    int ransac_minInliersCount;


    //------------------------------------------------------------
    // constructor
    slamParameters()
    {
        camera_factor = 1000.0;
        camera_cx = 325.5;
        camera_cy = 253.5;
        camera_fx = 518.0;
        camera_fy = 519.0;

        camera_distCoeffs.resize(4);
        camera_distCoeffs[0] = 0.0;
        camera_distCoeffs[1] = 0.0;
        camera_distCoeffs[2] = 0.0;
        camera_distCoeffs[3] = 0.0;

        match_goodMatchThreshold = 5;

        ransac_iterationsCount = 100;
        ransac_minInliersCount = 100;
        ransac_reprojectionError = 8.0;
    }

    ~slamParameters()
    {
        camera_distCoeffs.clear();
    }

    //------------------------------------------------------------
    // perform configure
    int configure(const char *configFile);

    //------------------------------------------------------------
    // display parameters
    void displayParameters();
};

#endif // SLAMPARAMETERS_H
