#include "pointCloudFusion.h"

#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

pointCloudFusion::pointCloudFusion()
{
}

void fusingPointCloud(PointCloudT &base,
                      PointCloudT &addition,
                      Eigen::Affine3f &transformation)
{
    std::cout << "# base cloud size: " << base.points.size() << std::endl
              << "# addition cloud size: " << addition.points.size() << std::endl;

    PointCloudT_Ptr addition_trans(new PointCloudT);
    pcl::transformPointCloud(addition, *addition_trans, transformation.matrix());
    base += *addition_trans;

    PointCloudT_Ptr tmp = base.makeShared();

    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setLeafSize(0.003, 0.003, 0.003); // 3 mm
    voxelGrid.setInputCloud(tmp);
    voxelGrid.filter(base);

    std::cout << "# fuse cloud size: " << base.points.size() << std::endl;
}

void fusingPointCloud(PointCloudT &in_base,
                      PointCloudT &in_addition,
                      PointCloudT &output,
                      Eigen::Affine3f &transformation)
{
    std::cout << "# base cloud size: " << in_base.points.size() << std::endl
              << "# addition cloud size: " << in_addition.points.size() << std::endl;

    PointCloudT_Ptr addition_trans(new PointCloudT);
    pcl::transformPointCloud(in_addition, *addition_trans, transformation.matrix());
    in_base += *addition_trans;

    PointCloudT_Ptr tmp = in_base.makeShared();

    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setLeafSize(0.003, 0.003, 0.003); // 3 mm
    voxelGrid.setInputCloud(tmp);
    voxelGrid.filter(output);

    std::cout << "# fuse cloud size: " << output.points.size() << std::endl;
}
