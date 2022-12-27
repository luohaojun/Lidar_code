#include <iostream>
#include <algorithm>
#include <string>
#include <unistd.h>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <boost/thread/thread.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <cmath>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;




typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int iterations = 1;  // Default number of ICP iterations
void Run_ICP(PointCloudT::Ptr cloud_s, PointCloudT::Ptr cloud_t, Eigen::Matrix4d tra_mat);

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

Eigen::Vector3f pointXYZ2Vec(pcl::PointXYZ pXYZ)
{
    return(Eigen::Vector3f(pXYZ.x,pXYZ.y,pXYZ.z));
}

pcl::PointXYZ Vec2pointXYZ(Eigen::Vector3f pXYZ)
{
    pcl::PointXYZ pxyz;
    pxyz.x = pXYZ[0];
    pxyz.x = pXYZ[1];
    pxyz.x = pXYZ[2];
    return pxyz;
}

void Segmentation(PointCloudT::Ptr cloud_in);


int main()
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_1 (new PointCloudT);  // point cloud 1
    PointCloudT::Ptr cloud_2 (new PointCloudT);  // point cloud 2
    PointCloudT::Ptr cloud_3 (new PointCloudT);  // point cloud 3
    PointCloudT::Ptr cloud_4 (new PointCloudT);  // point cloud 4
    PointCloudT::Ptr cloud_5(new PointCloudT);   // point cloud 5
    PointCloudT::Ptr cloud_6 (new PointCloudT);  // point cloud 6
    PointCloudT::Ptr cloud_7 (new PointCloudT);  // point cloud 7
    PointCloudT::Ptr cloud_8 (new PointCloudT);  // point cloud 8
    PointCloudT::Ptr cloud_9 (new PointCloudT);  // point cloud 9
    PointCloudT::Ptr cloud_10(new PointCloudT);   // point cloud 10
    PointCloudT::Ptr cloud_sum(new PointCloudT); // point cloud sum
    PointCloudT::Ptr cloud_sum2(new PointCloudT); // point cloud sum

    //-----load files-------
    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1499.500400032.pcd", *cloud_1) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_1->size() << " points)" << std::endl;

    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1499.600400032.pcd", *cloud_2) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_2->size() << " points)" << std::endl;

    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1499.700249990.pcd", *cloud_3) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_3->size() << " points)" << std::endl;

    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1499.800099710.pcd", *cloud_4) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_4->size() << " points)" << std::endl;

    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1499.899961910.pcd", *cloud_5) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_5->size() << " points)" << std::endl;
    //---------6-10-------
    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1499.999790450.pcd", *cloud_6) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_6->size() << " points)" << std::endl;

    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1500.099655230.pcd", *cloud_7) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_7->size() << " points)" << std::endl;

    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1500.199886430.pcd", *cloud_8) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_8->size() << " points)" << std::endl;

    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1500.300400032.pcd", *cloud_9) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_9->size() << " points)" << std::endl;

    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1500.400360790.pcd", *cloud_10) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_10->size() << " points)" << std::endl;

    
    //----------segmentation-----------
    Segmentation(cloud_1);
    Segmentation(cloud_2);
    Segmentation(cloud_3);
    Segmentation(cloud_4);
    Segmentation(cloud_5);
    Segmentation(cloud_6);
    Segmentation(cloud_7);
    Segmentation(cloud_8);
    Segmentation(cloud_9);
    Segmentation(cloud_10);
    *cloud_sum2 = *cloud_1 + *cloud_2 + *cloud_3 + *cloud_4+ *cloud_5 + *cloud_6 + *cloud_7 + *cloud_8 + *cloud_9+ *cloud_10;
    //----------use G-ICP---------------
    pcl::IterativeClosestPoint<PointT, PointT> gicp;
    gicp.setMaximumIterations(6);
    gicp.setInputSource(cloud_1);
    gicp.setInputTarget(cloud_5);
    gicp.align(*cloud_1);
    gicp.setInputSource(cloud_2);
    gicp.setInputTarget(cloud_5);
    gicp.align(*cloud_2);
    gicp.setInputSource(cloud_3);
    gicp.setInputTarget(cloud_5);
    gicp.align(*cloud_3);
    gicp.setInputSource(cloud_4);
    gicp.setInputTarget(cloud_5);
    gicp.align(*cloud_4);
    gicp.setInputSource(cloud_6);
    gicp.setInputTarget(cloud_5);
    gicp.align(*cloud_6);
    gicp.setInputSource(cloud_7);
    gicp.setInputTarget(cloud_5);
    gicp.align(*cloud_7);
    gicp.setInputSource(cloud_8);
    gicp.setInputTarget(cloud_5);
    gicp.align(*cloud_8);
    gicp.setInputSource(cloud_9);
    gicp.setInputTarget(cloud_5);
    gicp.align(*cloud_9);
    gicp.setInputSource(cloud_10);
    gicp.setInputTarget(cloud_5);
    gicp.align(*cloud_10);
    

    // Run_ICP(cloud_1, cloud_5, tra_mat1);
    // Run_ICP(cloud_2, cloud_5, tra_mat2);
    // Run_ICP(cloud_3, cloud_5, tra_mat3);
    // Run_ICP(cloud_4, cloud_5, tra_mat4);
    
    //----------splicing----------------
    *cloud_sum = *cloud_1 + *cloud_2 + *cloud_3 + *cloud_4+ *cloud_5 + *cloud_6 + *cloud_7 + *cloud_8 + *cloud_9+ *cloud_10;

    // pcl::StatisticalOutlierRemoval::applyFileter()
    pcl::visualization::CloudViewer viewer("3DViewer"); //创建窗口
    viewer.showCloud(cloud_sum);
    // viewer.showCloud(cloud_sum2);

    pause(); //按任意键继续...

    return 0;
}

// void Run_ICP(PointCloudT::Ptr cloud_s, PointCloudT::Ptr cloud_t, Eigen::Matrix4d &tra_mat)
// {
//     pcl::IterativeClosestPoint<PointT, PointT> gicp;
//     gicp.setMaximumIterations(iterations);
//     gicp.setInputSource(cloud_s);
//     gicp.setInputTarget(cloud_t);
//     gicp.align(*cloud_s);
//     gicp.setMaximumIterations(6);
//     tra_mat = gicp.getFinalTransformation().cast<double>();
// }

void Segmentation(PointCloudT::Ptr cloud_in)
{
    pcl::PassThrough<PointT> pt;
    // Filter in z direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(0.0f, 0.5f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
    // Filter in x direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(0.0f, 3.0f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
    // Filter in y direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(-0.5f, 2.5f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
}
