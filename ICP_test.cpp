#include <iostream>
#include <string>
#include <sophus/se3.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/ia_fpcs.h> // 4PCS算法
#include <pcl/filters/passthrough.h>

using namespace Sophus;
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main (int argc, char* argv[])
{
    // //initial guess of transformation
    // Quaterniond q(2.20761e-05, -3.34606e-05, -7.20881e-05, 0.000335889);
    // Eigen::Vector3d t(-0.708202, 0.000602459, 0.706009);
    // Sophus::SE3 SE3_qt(q,t);
    // Eigen::Matrix4f matrix;
    // matrix = SE3_qt.matrix().cast<float>();


    // The point clouds we will be using
    PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

    int iterations = 1;  // Default number of ICP iterations

    // load files
    pcl::console::TicToc time;
    time.tic ();
    if (pcl::io::loadPCDFile ("/home/luo/EX_ws/src/my_pcl/rosbag/RefPointCloud/337.200400032.pcd", *cloud_in) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file " << "bun000.ply" << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;
    time.tic ();
    if (pcl::io::loadPCDFile ("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1490.499353030.pcd", *cloud_tr) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_tr file!");
        return (-1);
    }
    std::cout << "\nLoaded file " << "top3.ply" << " (" << cloud_tr->size () << " points) in " << time.toc () << " ms\n" << std::endl;

    



    //-------Point cloud cutting---------
    pcl::console::TicToc time_cutting;
    time_cutting.tic();
    //cloud_in
    // 创建直通滤波（模板）类对象，点数据类型为 pcl::PointXYZRGB
    pcl::PassThrough<PointT> pt;
    // 设置输入点云，注意：此处传入的是点云类对象的智能指针
    pt.setInputCloud(cloud_in);
    // 设置通道名称 x/y/z
    pt.setFilterFieldName("z");
    // 设置当前裁剪通道的最值区间，默认保留区间内的数据
    pt.setFilterLimits(0.0f,0.2f);
    // 是否对区间范围取反，即保留区间外的数据
    pt.setFilterLimitsNegative(false);
    // 执行过滤，并带出处理后的点云数据，注意：此处传入的是点云类对象
    pt.filter(*cloud_in);
    //Filter in x direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(0.0f,2.5f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
    //Filter in y direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(-1.0f,2.5f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);

    //cloud_tr
    //Filter in z direction
    pt.setInputCloud(cloud_tr);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(0.0f,0.2f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_tr);
    //Filter in x direction
    pt.setInputCloud(cloud_tr);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(1.3f,1.6f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_tr);
    //Filter in y direction
    pt.setInputCloud(cloud_tr);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(-1.0f, 2.5f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_tr);
    cout << "Point cloud cutting in " << time_cutting.toc() << " ms\n"
         << endl;

    cloud_icp = cloud_tr;

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();


    //remove NaN points
    cloud_in->is_dense = false;
    cloud_tr->is_dense = false;
    cloud_icp->is_dense = false;
    std::vector<int> mapping_in;
    std::vector<int> mapping_tr;
    std::vector<int> mapping_icp;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, mapping_in);
    pcl::removeNaNFromPointCloud(*cloud_tr, *cloud_tr, mapping_tr);
    pcl::removeNaNFromPointCloud(*cloud_icp, *cloud_icp, mapping_icp);

    //Coarse registration by 4PCS
    time.tic();
    //--------------初始化4PCS配准对象-------------------
    pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;
    fpcs.setInputSource(cloud_icp);  // 源点云
    fpcs.setInputTarget(cloud_in);  // 目标点云
    fpcs.setApproxOverlap(0.7);         // 设置源和目标之间的近似重叠度。
    fpcs.setDelta(0.01);                // 设置配准后对应点之间的距离（以米为单位）。
    fpcs.setNumberOfSamples(50);       // 设置验证配准效果时要使用的采样点数量
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);
    fpcs.align(*pcs);                   // 计算变换矩阵
    cout << "FPCS配准用时： " << time.toc() << " ms" << endl;
    cout << "变换矩阵：" << fpcs.getFinalTransformation() << endl;



    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);
    icp.align (*cloud_icp);
    icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-0.0172, -0.0936, -0.734,  -0.0461723, 0.970603, -0.235889, 0.0124573);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();

        // The user pressed "space" :
        if (next_iteration)
        {
            // The Iterative Closest Point algorithm
            time.tic ();
            icp.align (*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

            if (icp.hasConverged ())
            {
                printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR ("\nICP has not converged.\n");
                return (-1);
            }
        }
        next_iteration = false;
    }
    return (0);
}