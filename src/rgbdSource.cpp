#include "rgbdSource.h"

#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

rgbdSource::rgbdSource()
{
}

//------------------------------------------------------------
int fileSource::generateNewFrame(frame &aframe)
{
    if(frame_start+frame_count > frame_end)
    {
        std::cout << "\n----------------------------------------\n"
                     "last frame reached."
                  << std::endl;
        return EXIT_FAILURE;
    }

    char colorFile[100] = {0};
    char depthFile[100] = {0};
    sprintf(colorFile, "%s/%d%s",
            rgb_dir.c_str(), frame_start+frame_count, rgb_extension.c_str());
    sprintf(depthFile, "%s/%d%s",
            depth_dir.c_str(), frame_start+frame_count, depth_extension.c_str());

    current_time += frame_time_interval;
    frame_count += 1;
    std::cout << "current time: " << current_time << "s" << std::endl
              << "reading files: " << colorFile << ", " << depthFile
              << "\nframe number #" << frame_count << std::endl;

    cv::Mat rgb = cv::imread(colorFile);
    cv::Mat depth = cv::imread(depthFile, -1);
    if(!rgb.data || !depth.data)
    {
        std::cout << "Error during reading files "
                  << colorFile << ", " << depthFile
                  << "." << std::endl;
        return EXIT_FAILURE;
    }

    aframe = frame(rgb, depth, camera, current_time);

    rgb.release();
    depth.release();

    return EXIT_SUCCESS;
}

