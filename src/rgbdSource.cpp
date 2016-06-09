#include "rgbdSource.h"

#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

rgbdSource::rgbdSource()
{
}

////////////////////////////////////////////////////////////
int fileSource::generateNewFrame(frame &aframe)
{
    char colorFile[30] = {0};
    char depthFile[30] = {0};
    sprintf(colorFile, "%s/rgb%d.png", source_dir.c_str(), frame_count);
    sprintf(depthFile, "%s/depth%d.png", source_dir.c_str(), frame_count);

    current_time += frame_time_interval;
    frame_count += 1;
    std::cout << "current time: " << current_time << "s" << std::endl
              << "reading files: " << colorFile << ", " << depthFile
              << "\nframe number #" << frame_count-1 << std::endl;

    cv::Mat rgb = cv::imread( colorFile);
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
