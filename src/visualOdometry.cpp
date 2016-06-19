#include "visualOdometry.h"
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <unistd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "slambase.h"

#include <g2o/types/slam3d/types_slam3d.h> //顶点类型
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


G2O_USE_TYPE_GROUP(slam3d);

void eigen_fmat_to_dmat(Eigen::Matrix4f &fmat, Eigen::Matrix4d &dmat)
{
    for(int i=0; i<4; ++i)
        for(int j=0; j<4; ++j)
            dmat(i,j) = (double)fmat(i,j);
}

visualOdometry::visualOdometry()
{
}

float visualOdometry::normofTransform(SixDegreeTransformation &transform)
{
    return fabs(fmin(transform.rotat_vec.norm(), (float)2*M_PI-transform.rotat_vec.norm()))
            + fabs(transform.trans_vec.norm());
}

void visualOdometry::run_with_g2o()
{
    frame::parameters frame_param;
    frame_param.descriptor_type = "SIFT";
    frame_param.detector_type = "SIFT";

    frame lastFrame;
    if(file_source->generateNewFrame(lastFrame)==1)
    {
        cout << "failed to read image, exit." << endl;
        return;
    }
    unsigned current_index = 0;

    lastFrame.setParameters(frame_param);
    lastFrame.computePointCloud();
    lastFrame.computeKeypointsAndDescriptors();
    PointCloudT_Ptr cloud = lastFrame.getPointCloudCopy();

    /*******************************
    // 新增:有关g2o的初始化
    *******************************/
    // 选择优化方法
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    globalOptimizer.setAlgorithm( solver );
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(current_index);
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );

    pcl::visualization::CloudViewer viewer("viewer");
    // 是否显示点云
    bool visualize = this->param.visualize_pointCloud;
    int min_inliers = this->param.min_inliers;
    float max_norm = this->param.max_norm;

    Eigen::Affine3f toWorldTransform = Eigen::Affine3f::Identity();
    frame currFrame;
    unsigned last_index = current_index;
    while (file_source->generateNewFrame(currFrame)==0)
    {
        ++current_index;
        currFrame.setParameters(frame_param);
        currFrame.computePointCloud();
        currFrame.computeKeypointsAndDescriptors();

        // 求解pnp
        Eigen::Affine3f affine;
        // align currFrame to lastFrame
        estimateMotion_3dTo3d(currFrame, lastFrame, affine);

        // 计算运动范围是否太大
        SixDegreeTransformation transform_vec(affine.matrix());
        float norm = normofTransform(transform_vec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
        {
            cout << "current transformation norm >= max_norm, skip this frame."
                 << endl;
            continue;
        }

        // 向g2o中增加这个顶点与上一帧联系的边
        // 顶点部分
        // 顶点只需设定id即可
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(current_index);
        v->setEstimate( Eigen::Isometry3d::Identity() );
        globalOptimizer.addVertex(v);
        // 边部分
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->vertices() [0] = globalOptimizer.vertex( last_index );
        edge->vertices() [1] = globalOptimizer.vertex( current_index );
        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation( information );
        // 边的估计即是pnp求解之结果
        Eigen::Matrix4d tmp_dmat;
        eigen_fmat_to_dmat(affine.matrix(), tmp_dmat);
        Eigen::Isometry3d tt(tmp_dmat);
        edge->setMeasurement( tt );
        // 将此边加入图中
        globalOptimizer.addEdge(edge);

        // 转换点云
        cout<< "converting image to clouds" << endl;
        PointCloudT_Ptr current_cloud = currFrame.getPointCloud();

        // 合并点云
        cout<<"combining clouds"<<endl;
        toWorldTransform = toWorldTransform * affine;
        fusingPointCloud(*cloud, *current_cloud, *cloud, toWorldTransform);
        if ( visualize == true )
            viewer.showCloud( cloud );

        lastFrame = currFrame;
        last_index = current_index;
    }

    //pcl::io::savePCDFile( "data/result.pcd", *cloud );
    // 优化所有边
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "./data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    globalOptimizer.clear();

    return;
}

void visualOdometry::run()
{
    frame::parameters frame_param;
    frame_param.descriptor_type = "SIFT";
    frame_param.detector_type = "SIFT";

    frame lastFrame;
    if(file_source->generateNewFrame(lastFrame)==1)
    {
        cout << "failed to read image, exit." << endl;
        return;
    }
    lastFrame.setParameters(frame_param);
    lastFrame.computePointCloud();
    lastFrame.computeKeypointsAndDescriptors();
    PointCloudT_Ptr cloud = lastFrame.getPointCloudCopy();

    // 是否显示点云
    pcl::visualization::CloudViewer viewer("viewer");
    bool visualize = this->param.visualize_pointCloud;

    int min_inliers = this->param.min_inliers;
    float max_norm = this->param.max_norm;

    Eigen::Affine3f toWorldTransform = Eigen::Affine3f::Identity();
    frame currFrame;
    while (file_source->generateNewFrame(currFrame)==0)
    {
        currFrame.setParameters(frame_param);
        currFrame.computePointCloud();
        currFrame.computeKeypointsAndDescriptors();

        // 求解pnp
        Eigen::Affine3f affine;
        // align currFrame to lastFrame
        estimateMotion_3dTo3d(currFrame, lastFrame, affine);

        // 计算运动范围是否太大
        SixDegreeTransformation transform_vec(affine.matrix());
        float norm = normofTransform(transform_vec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
        {
            cout << "current transformation norm >= max_norm, skip this frame."
                 << endl;
            continue;
        }

        // 转换点云
        cout<< "converting image to clouds" << endl;
        PointCloudT_Ptr current_cloud = currFrame.getPointCloud();

        // 合并点云
        cout<<"combining clouds"<<endl;
        //        toWorldTransform = toWorldTransform*affine;
        //        fusingPointCloud(*cloud, *current_cloud, toWorldTransform);
        refinePoseAndFusingPointCloud(*cloud, *current_cloud, toWorldTransform, affine);

        if ( visualize == true )
            viewer.showCloud(cloud);

        lastFrame = currFrame;
    }

    pcl::io::savePCDFile( "../data/result.pcd", *cloud );
    return;
}
