#include "pointCloudFusion.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

pointCloudFusion::pointCloudFusion()
{
}

void fusingPointCloud(PointCloudT &base,
                      PointCloudT &addition,
                      Eigen::Affine3f &transformation)
{
    PointCloudT_Ptr addition_trans(new PointCloudT);
    pcl::transformPointCloud(addition, *addition_trans, transformation.matrix());
    base += *addition_trans;

    PointCloudT_Ptr tmp = base.makeShared();

    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setLeafSize(0.003, 0.003, 0.003); // 3 mm
    voxelGrid.setInputCloud(tmp);
    voxelGrid.filter(base);
}

void fusingPointCloud(PointCloudT &in_base,
                      PointCloudT &in_addition,
                      PointCloudT &output,
                      Eigen::Affine3f &transformation)
{
    PointCloudT_Ptr addition_trans(new PointCloudT);
    pcl::transformPointCloud(in_addition, *addition_trans, transformation.matrix());
    in_base += *addition_trans;

    PointCloudT_Ptr tmp = in_base.makeShared();

    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setLeafSize(0.003, 0.003, 0.003); // 3 mm
    voxelGrid.setInputCloud(tmp);
    voxelGrid.filter(output);
}
