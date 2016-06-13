#include "visualOdometry.h"
#include <math.h>

visualOdometry::visualOdometry()
{
}

float visualOdometry::normofTransform(SixDegreeTransformation &transform)
{
    return fabs(fmin(transform.rotat_vec.norm(), (float)2*M_PI-transform.rotat_vec.norm()))
            + fabs(transform.trans_vec.norm());
}

void visualOdometry::run()
{

//    // initialize
//    cout<<"Initializing ..."<<endl;
//    int currIndex = startIndex; // 当前索引为currIndex
//    FRAME lastFrame = readFrame( currIndex, pd ); // 上一帧数据
//    // 我们总是在比较currFrame和lastFrame
//    string detector = pd.getData( "detector" );
//    string descriptor = pd.getData( "descriptor" );
//    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
//    computeKeyPointsAndDesp( lastFrame, detector, descriptor );
//    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );

//    pcl::visualization::CloudViewer viewer("viewer");

//    // 是否显示点云
//    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

//    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
//    double max_norm = atof( pd.getData("max_norm").c_str() );

//    for ( currIndex=startIndex+1; currIndex<endIndex; currIndex++ )
//    {
//        cout<<"Reading files "<<currIndex<<endl;
//        FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame
//        computeKeyPointsAndDesp( currFrame, detector, descriptor );
//        // 比较currFrame 和 lastFrame
//        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera );
//        if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
//            continue;
//        // 计算运动范围是否太大
//        double norm = normofTransform(result.rvec, result.tvec);
//        cout<<"norm = "<<norm<<endl;
//        if ( norm >= max_norm )
//            continue;
//        Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
//        cout<<"T="<<T.matrix()<<endl;

//        //cloud = joinPointCloud( cloud, currFrame, T.inverse(), camera );
//        cloud = joinPointCloud( cloud, currFrame, T, camera );

//        if ( visualize == true )
//            viewer.showCloud( cloud );

//        lastFrame = currFrame;
//    }

//    //pcl::io::savePCDFile( "data/result.pcd", *cloud );
    //    return;
}
