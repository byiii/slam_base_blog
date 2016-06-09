#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <string>
#include "commonDefinitions.h"

class visualizer_simple
{
protected:
    std::string name;
public:
    visualizer_simple(const char* str)
    {
        name = str;
    }

    void showPointCloud(PointCloudT_Ptr pc);
};

#endif // VISUALIZER_H
