#ifndef RGBDSOURCE_H
#define RGBDSOURCE_H

#include "commonDefinitions.h"
#include "frame.h"
#include <iostream>

#include <string>

class rgbdSource
{
public:
    rgbdSource();
};

class fileSource
{
public:
    std::string rgb_dir;
    std::string depth_dir;
    std::string rgb_marker;
    std::string depth_marker;
    std::string rgb_extension;
    std::string depth_extension;
    std::string source_dir;
protected:
    float frame_rate;
    float frame_time_interval;
    float current_time;
    unsigned frame_count;
    unsigned frame_start;
    unsigned frame_end;

    camera_intrinsic_parameters camera;
public:
    fileSource(const char* str, float fr=1.0, float ct=0.0)
    {
        source_dir = std::string(str);
        std::cout << "file source dir: " << source_dir << std::endl;

        if(fr==0.0)
            frame_rate = 1.0;
        else
            frame_rate = fr;
        current_time = ct;
        frame_time_interval = 1/frame_rate;
        frame_count = 0;
        frame_start = 0;
        frame_end = 0;
    }

    float getCurrentTime()
    {
        return current_time;
    }

    unsigned getCurrentFrameIndex()
    {
        return (frame_start+frame_count);
    }

    void setSourceDir(const char* str)
    {
        source_dir = str;
    }

    void setCamera(camera_intrinsic_parameters &c)
    {
        this->camera = c;
    }

    void setFrameRate(float f)
    {
        if(f!=0)
        {
            frame_rate = f;
            frame_time_interval = 1/f;
        }
        else
        {
            frame_rate = 1;
            frame_time_interval = 1/frame_rate;
        }
    }

    void setCurrentTime(float t)
    {
        current_time = t;
    }

    void setStartIndex(unsigned n)
    {
        frame_start = n;
    }

    void setEndIndex(unsigned n)
    {
        frame_end = n;
    }

    int generateNewFrame(frame &aframe);
};

class openniDeviceSource
{
};

#endif // RGBDSOURCE_H
