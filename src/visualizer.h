#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class visualizer_simple
{
protected:
    std::string name;
public:
    visualizer_simple(const char* str)
    {
        name = str;
    }

    void showPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc);
};

#endif // VISUALIZER_H
