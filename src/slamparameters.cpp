#include "slamparameters.h"

#include <iostream>
#include <iomanip>
#include <cstdlib>

#include <libconfig.h++>

int slamParameters::configure(const char *configFile)
{
    using namespace std;

    //------------------------------------------------------------
    libconfig::Config cfg;

    //------------------------------------------------------------
    // Read the file. If there is an error, report it and exit.
    try
    {
        cfg.readFile(configFile);
    }
    catch(const libconfig::FileIOException &fioex)
    {
        cerr << "I/O error while reading file." << endl;
        cerr << fioex.what() << endl;

        return(EXIT_FAILURE);
    }
    catch(const libconfig::ParseException &pex)
    {
        cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
             << " - " << pex.getError() << endl;
        return(EXIT_FAILURE);
    }

    //------------------------------------------------------------

    cout << "\n------------------------------------------------------------\n"
         << "read configure file: " << configFile << endl;

    //------------------------------------------------------------
    // perform configuring
    try
    {
        const libconfig::Setting& root = cfg.getRoot();

        // configure camera intrinsic parameters
        const libconfig::Setting& camera = root["camera"];
        camera.lookupValue("scale", camera_factor);
        camera.lookupValue("cx", camera_cx);
        camera.lookupValue("cy", camera_cy);
        camera.lookupValue("fx", camera_fx);
        camera.lookupValue("fy", camera_fy);

        // configure camera distortion coefficients
        // (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements.
        camera_distCoeffs.clear(); // clear original data first
        camera_distCoeffs.resize(8);
        const libconfig::Setting& camera_dist = root["camera_distortion"];
        if(camera_dist.exists("k_1"))
            camera_dist.lookupValue("k_1", camera_distCoeffs[0]);
        if(camera_dist.exists("k_2"))
            camera_dist.lookupValue("k_2", camera_distCoeffs[1]);
        if(camera_dist.exists("p_1"))
            camera_dist.lookupValue("p_1", camera_distCoeffs[2]);
        if(camera_dist.exists("p_2"))
            camera_dist.lookupValue("p_2", camera_distCoeffs[3]);
        if(camera_dist.exists("k_3"))
            camera_dist.lookupValue("k_3", camera_distCoeffs[4]);
        if(camera_dist.exists("k_4"))
            camera_dist.lookupValue("k_4", camera_distCoeffs[5]);
        if(camera_dist.exists("k_5"))
            camera_dist.lookupValue("k_5", camera_distCoeffs[6]);
        if(camera_dist.exists("k_6"))
            camera_dist.lookupValue("k_6", camera_distCoeffs[7]);

        // configure feature matching parameters
        const libconfig::Setting& match = root["match"];
        match.lookupValue("good_match_threshold", match_goodMatchThreshold);

        // configure pnp ransac parameters
        const libconfig::Setting& pnp_ransac = root["pnp_ransac"];
        pnp_ransac.lookupValue("max_iteration", ransac_iterationsCount);
        pnp_ransac.lookupValue("reprojection_error", ransac_reprojectionError);
        pnp_ransac.lookupValue("min_inliers", ransac_minInliersCount);
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        cerr << "some settings not found, default value will be used." << endl;
        cerr << nfex.what() << endl;
    }
}


void slamParameters::displayParameters()
{
    using namespace std;
    cout << "\n------------------------------------------------------------\n";
    cout << "# camera intrinsic parameters #" << endl;
    cout << "scale factor: " << camera_factor << endl;
    cout << "cx: " << camera_cx << endl;
    cout << "cy: " << camera_cy << endl;
    cout << "fx: " << camera_fx << endl;
    cout << "fy: " << camera_fy << endl;
    cout << "# camera distortion coefficients #" << endl;
    for(size_t i=0; i<camera_distCoeffs.size(); ++i)
        cout << camera_distCoeffs[i] << ", ";
    cout << endl;
    cout << "# feature matching #" << endl;
    cout << " good match threshold: " << match_goodMatchThreshold << endl;
    cout << "# pnp ransac #" << endl;
    cout << ransac_iterationsCount << endl;
    cout << ransac_reprojectionError << endl;
    cout << ransac_minInliersCount << endl;

    cout << endl;
}
