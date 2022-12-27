//PCL 4PCS算法实现点云粗配准
#include <iostream>
#include <string>
#include <sophus/se3.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/ia_fpcs.h> // 4PCS算法

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void _4PCS(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud);


int main()
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

    int iterations = 100;  // Default number of ICP iterations

    // load files
    pcl::console::TicToc time;
    time.tic ();
    if (pcl::io::loadPLYFile ("/home/luo/Group_UAV/Lidar/dataset/bunny/data/bun000.ply", *cloud_in) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file " << "bun000.ply" << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;
    time.tic ();
    if (pcl::io::loadPLYFile ("/home/luo/Group_UAV/Lidar/dataset/bunny/data/top3.ply", *cloud_tr) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_tr file!");
        return (-1);
    }
    std::cout << "\nLoaded file " << "top3.ply" << " (" << cloud_tr->size () << " points) in " << time.toc () << " ms\n" << std::endl;

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

    _4PCS(cloud_in,cloud_icp);

    return 0;
}




void _4PCS(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud)
{
    pcl::console::TicToc time;
    time.tic();
    //--------------初始化4PCS配准对象-------------------
    pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;
    fpcs.setInputSource(source_cloud);  // 源点云
    fpcs.setInputTarget(target_cloud);  // 目标点云
    fpcs.setApproxOverlap(0.7);         // 设置源和目标之间的近似重叠度。
    fpcs.setDelta(0.01);                // 设置配准后对应点之间的距离（以米为单位）。
    fpcs.setNumberOfSamples(100);       // 设置验证配准效果时要使用的采样点数量
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);
    fpcs.align(*pcs);                   // 计算变换矩阵
    cout << "FPCS配准用时： " << time.toc() << " ms" << endl;
    cout << "变换矩阵：" << fpcs.getFinalTransformation() << endl;
    // 使用创建的变换对为输入点云进行变换
    pcl::transformPointCloud(*source_cloud, *pcs, fpcs.getFinalTransformation());

     // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two vertically separated viewports
    int v2 (1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (source_cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (source_cloud, cloud_in_color_h, "cloud_in_v2", v2);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (pcs, 180, 20, 20);
    viewer.addPointCloud (pcs, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);


    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size


    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce (100);
    }


}
