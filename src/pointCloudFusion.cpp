#include "pointCloudFusion.h"

#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

pointCloudFusion::pointCloudFusion()
{
}

float leafSize = 0.010;

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
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize); // leaf size
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
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize); // leaf size
    voxelGrid.setInputCloud(tmp);
    voxelGrid.filter(output);

    std::cout << "# fuse cloud size: " << output.points.size() << std::endl;
}


void refinePoseAndFusingPointCloud(PointCloudT &base,
                                   PointCloudT &addition,
                                   Eigen::Affine3f &camera_pose,
                                   Eigen::Affine3f &pose_increment)
{
    PointCloudT_Ptr addition_trans(new PointCloudT);
    pcl::transformPointCloud(addition, *addition_trans, (camera_pose*pose_increment).matrix());

    PointCloudT_Ptr tmp = base.makeShared();
    // The Iterative Closest Point algorithm
    int max_iteration = 8;
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations (max_iteration);
    icp.setInputSource (addition_trans);
    icp.setInputTarget (tmp);
    icp.align (base, Eigen::Matrix4f::Identity());
    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "ICP transformation " << max_iteration << " : addition -> base\n" << std::endl;
        pose_increment = pose_increment * icp.getFinalTransformation ();
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
    }
    camera_pose = camera_pose * pose_increment;
    fusingPointCloud(base, addition, camera_pose);
}
