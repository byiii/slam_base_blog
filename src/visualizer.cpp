#include "visualizer.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>

void visualizer_simple::showPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc)
{
    pcl::visualization::PCLVisualizer viewer(name.c_str());
    viewer.setBackgroundColor(0.3, 0.4, 0.4);
    viewer.addPointCloud(pc);

    while(!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    viewer.close();
}
