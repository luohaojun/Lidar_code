#include <iostream>
#include <algorithm>
#include <string>
#include <sophus/se3.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/ia_fpcs.h> // 4PCS算法
#include <boost/thread/thread.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>

#include <vector>
#include <cmath>
#include <opencv4/opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>          //kdtree近邻搜索
#include <pcl/features/normal_3d.h> //Calculate normal vector of one point



using namespace std;
using namespace cv;




typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

Eigen::Vector3f pointXYZ2Vec(pcl::PointXYZ pXYZ)
{
    return(Eigen::Vector3f(pXYZ.x,pXYZ.y,pXYZ.z));
}

cv::Point3f pointXYZ2Point(pcl::PointXYZ pXYZ)
{
    return (Point3f(pXYZ.x,pXYZ.y,pXYZ.z));
}


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}

float CalCurvatureChange(Eigen::Matrix3f _C);
Eigen::Vector3d NormalVec(PointCloudT::Ptr _cloud_source, int _indexies, int _K, float &_curveture);
void CalDescriptor(PointCloudT::Ptr _cloud, vector<Eigen::Vector3f> &_descriptor);
void pose_estimation_3d3d(const vector<Point3f> &pts1,
                          const vector<Point3f> &pts2,
                          Mat &R, Mat &t);
void Matching(PointCloudT::Ptr cloud_s,
              PointCloudT::Ptr cloud_t,
              vector<Eigen::Vector3f> descrp_s,
              vector<Eigen::Vector3f> descrp_t,
              vector<Point3f> &pts1,
              vector<Point3f> &pts2);
float getDegAngle3d(const Eigen::Vector3d v1, const Eigen::Vector3d v2);




void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

double MINIMUM_RANGE = 0.1; //parameter for removing closed points
int K = 4; //设置需要查找的近邻点个数
int l = 4; //number of neibor point to calculate normal vector
float tao = 0.7; //threshold of the distance between to descriptor

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}


// 实现argsort功能
template<typename T> std::vector<int> argsort(const std::vector<T>& array)
{
    const int array_len(array.size());
    std::vector<int> array_index(array_len, 0);
    for (int i = 0; i < array_len; ++i)
        array_index[i] = i;

    std::sort(array_index.begin(), array_index.end(),
        [&array](int pos1, int pos2) {return (array[pos1] < array[pos2]); });

    return array_index;
}

void Segmentation(PointCloudT::Ptr cloud_in);




int main()
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
    PointCloudT::Ptr cloud_1 (new PointCloudT);  // ICP output point cloud
    PointCloudT::Ptr cloud_2 (new PointCloudT);  // ICP output point cloud
    PointCloudT::Ptr cloud_3 (new PointCloudT);  // ICP output point cloud
    PointCloudT::Ptr cloud_4 (new PointCloudT);  // ICP output point cloud
    PointCloudT::Ptr cloud_5 (new PointCloudT);  // ICP output point cloud
    // PointCloudT::Ptr cloud_6 (new PointCloudT);  // point cloud 6
    // PointCloudT::Ptr cloud_7 (new PointCloudT);  // point cloud 7
    // PointCloudT::Ptr cloud_8 (new PointCloudT);  // point cloud 8
    // PointCloudT::Ptr cloud_9 (new PointCloudT);  // point cloud 9
    // PointCloudT::Ptr cloud_10(new PointCloudT);   // point cloud 10

    int iterations = 1;  // Default number of ICP iterations

    //-----load files-------
    pcl::console::TicToc time_LoadFiles;
    time_LoadFiles.tic();
    if (pcl::io::loadPLYFile("/home/luo/Group_UAV/Lidar_code/rosbag/0908/0908_asc.ply", *cloud_in) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_in->size() << " points) in " << time_LoadFiles.toc() << " ms\n"
              << std::endl;

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
    // //---------6-10-------
    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1499.999790450.pcd", *cloud_6) < 0)
    // {
    //     PCL_ERROR("Cloudn't read cloud_in file!");
    //     return (-1);
    // }
    // std::cout << "\nLoaded file "
    //           << "bun_zipper.pcd"
    //           << " (" << cloud_6->size() << " points)" << std::endl;

    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1500.099655230.pcd", *cloud_7) < 0)
    // {
    //     PCL_ERROR("Cloudn't read cloud_in file!");
    //     return (-1);
    // }
    // std::cout << "\nLoaded file "
    //           << "bun_zipper.pcd"
    //           << " (" << cloud_7->size() << " points)" << std::endl;

    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1500.199886430.pcd", *cloud_8) < 0)
    // {
    //     PCL_ERROR("Cloudn't read cloud_in file!");
    //     return (-1);
    // }
    // std::cout << "\nLoaded file "
    //           << "bun_zipper.pcd"
    //           << " (" << cloud_8->size() << " points)" << std::endl;

    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1500.300400032.pcd", *cloud_9) < 0)
    // {
    //     PCL_ERROR("Cloudn't read cloud_in file!");
    //     return (-1);
    // }
    // std::cout << "\nLoaded file "
    //           << "bun_zipper.pcd"
    //           << " (" << cloud_9->size() << " points)" << std::endl;

    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1500.400360790.pcd", *cloud_10) < 0)
    // {
    //     PCL_ERROR("Cloudn't read cloud_in file!");
    //     return (-1);
    // }
    // std::cout << "\nLoaded file "
    //           << "bun_zipper.pcd"
    //           << " (" << cloud_10->size() << " points)" << std::endl;


    //---------scala-------
    for(int i=0; i<cloud_in->size(); i++)
    {
        cloud_in->points[i].x = cloud_in->points[i].x/1000.0;
        cloud_in->points[i].y = cloud_in->points[i].y/1000.0;
        cloud_in->points[i].z = cloud_in->points[i].z/1000.0;
    }
    //-------Point cloud segmentation---------
    pcl::console::TicToc time_cutting;
    time_cutting.tic();
    Segmentation(cloud_1);
    Segmentation(cloud_2);
    Segmentation(cloud_3);
    Segmentation(cloud_4);
    Segmentation(cloud_5);
    // Segmentation(cloud_6);
    // Segmentation(cloud_7);
    // Segmentation(cloud_8);
    // Segmentation(cloud_9);
    // Segmentation(cloud_10);
    cout<<"Point cloud cutting in "<<time_cutting.toc()<<" ms\n"<<endl;

    //----------use G-ICP to splice point cloud---------------
    pcl::IterativeClosestPoint<PointT, PointT> gicp2;
    gicp2.setMaximumIterations(3);
    gicp2.setInputSource(cloud_1);
    gicp2.setInputTarget(cloud_5);
    gicp2.align(*cloud_1);
    gicp2.setInputSource(cloud_2);
    gicp2.setInputTarget(cloud_5);
    gicp2.align(*cloud_2);
    gicp2.setInputSource(cloud_3);
    gicp2.setInputTarget(cloud_5);
    gicp2.align(*cloud_3);
    gicp2.setInputSource(cloud_4);
    gicp2.setInputTarget(cloud_5);
    gicp2.align(*cloud_4);
    // gicp2.setInputSource(cloud_6);
    // gicp2.setInputTarget(cloud_5);
    // gicp2.align(*cloud_6);
    // gicp2.setInputSource(cloud_7);
    // gicp2.setInputTarget(cloud_5);
    // gicp2.align(*cloud_7);
    // gicp2.setInputSource(cloud_8);
    // gicp2.setInputTarget(cloud_5);
    // gicp2.align(*cloud_8);
    // gicp2.setInputSource(cloud_9);
    // gicp2.setInputTarget(cloud_5);
    // gicp2.align(*cloud_9);
    // gicp2.setInputSource(cloud_10);
    // gicp2.setInputTarget(cloud_5);
    // gicp2.align(*cloud_10);

    //----------splicing----------------
    // *cloud_tr = *cloud_1 + *cloud_2 + *cloud_3 + *cloud_4 + *cloud_5 + *cloud_6 + *cloud_7 + *cloud_8 + *cloud_9+ *cloud_10;
    *cloud_tr = *cloud_1 + *cloud_2 + *cloud_3 + *cloud_4 + *cloud_5;

    //-------remove NaNs points & closed points---------
    pcl::console::TicToc time_removeNaN;
    time_removeNaN.tic();
    cloud_in->is_dense = false;
    cloud_tr->is_dense = false;
    cloud_icp->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
    removeClosedPointCloud(*cloud_in, *cloud_in, MINIMUM_RANGE);
    pcl::removeNaNFromPointCloud(*cloud_tr, *cloud_tr, indices);
    // cout<<"cloud_tr size after removeNaN: "<<(*cloud_tr).size()<<endl;
    removeClosedPointCloud(*cloud_tr, *cloud_tr, MINIMUM_RANGE);
    // cout<<"cloud_tr size after removeNaN and closedPoint: "<<(*cloud_tr).size()<<endl;
    // cloud_icp = cloud_tr;
    cout<<"remove NaNs points & closed points in "<<time_removeNaN.toc()<<" ms\n"<<endl;

    //----------down sampling----------
    pcl::console::TicToc time_ds;
    time_ds.tic();
    PointCloudT::Ptr cloud_in_DS (new PointCloudT);
    PointCloudT::Ptr cloud_tr_DS (new PointCloudT);
    pcl::VoxelGrid<PointT> downSizeFilter;
    downSizeFilter.setInputCloud(cloud_in);
    downSizeFilter.setLeafSize(0.03, 0.03, 0.03);
    downSizeFilter.filter(*cloud_in_DS);
    downSizeFilter.setInputCloud(cloud_tr);
    downSizeFilter.filter(*cloud_tr_DS);
    downSizeFilter.setInputCloud(cloud_tr);
    downSizeFilter.filter(*cloud_icp);
    // pcl::removeNaNFromPointCloud(*cloud_in_DS, *cloud_in_DS, indices);
    // cout<<"cloud_tr size after removeNaN, closedPoint and downSizeFilter: "<<(*cloud_tr_DS).size()<<endl;
    cout<<"down sampling in "<<time_ds.toc()<<" ms\n"<<endl;
    // for(int i=0; i<cloud_icp->points.size(); i++)
    // {
    //     cout<<cloud_icp->points[i]<<endl;
    // }
    

    //---------Caculate descriptor---------
    vector<Eigen::Vector3f> descriptor_source;
    vector<Eigen::Vector3f> descriptor_target;
    pcl::console::TicToc time_descrp;
    time_descrp.tic();
    CalDescriptor(cloud_icp, descriptor_source);
    CalDescriptor(cloud_in_DS, descriptor_target);
    // cout<<"descriptor of point"<<"["<<1<<"]"<<": "<<descriptor_source[0]<<endl;
    cout<<"Caculate descriptor in "<<time_descrp.toc()<<" ms\n"<<endl;
    cout<<"descriptor_source size: "<<descriptor_source.size()<<endl;
    cout<<"descriptor_target size: "<<descriptor_target.size()<<endl;
    // cout<<"descriptor_target 100: "<<descriptor_target[100]<<endl;


    //---------------matching---------->>------
    vector<Point3f> pts_source;
    vector<Point3f> pts_target;
    pcl::console::TicToc time_matching;
    time_matching.tic();
    Matching(cloud_icp,cloud_in_DS,descriptor_source,descriptor_target,pts_source,pts_target);
    cout<<"totally matching "<<pts_source.size()<<" points in "<<time_matching.toc()<<" ms\n"<<endl;
    // cout<<"pts_source 100: "<<pts_source[100]<<endl;
    // cout<<"pts_target 100: "<<pts_target[100]<<endl;

    //--------------perform 3d-3d method----------
    Mat R_mat, t_mat;
    pcl::console::TicToc time_SVD;
    time_SVD.tic();
    pose_estimation_3d3d(pts_target, pts_source, R_mat, t_mat);
    cout<<"perform 3d-3d method in "<<time_SVD.toc()<<" ms\n"<<endl;
    cout<<"R: \n"<<R_mat<<endl;
    cout<<"t: \n"<<t_mat<<endl;
    Eigen::Matrix3f R_3f;
    Eigen::Vector3f t_3f;
    cv::cv2eigen(R_mat, R_3f);
    cv::cv2eigen(t_mat, t_3f);
    Eigen::Matrix4f T_coarse = Eigen::Matrix4f::Identity ();
    T_coarse.block(0,0,3,3) = R_3f;
    T_coarse.block(0,3,3,1) = t_3f;


    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    

    // //Coarse registration by 4PCS
    // time_LoadFiles.tic();
    // //--------------初始化4PCS配准对象-------------------
    // pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;
    // fpcs.setInputSource(cloud_icp);  // 源点云
    // fpcs.setInputTarget(cloud_in_DS);  // 目标点云
    // fpcs.setApproxOverlap(0.9);         // 设置源和目标之间的近似重叠度。
    // fpcs.setDelta(0.01);                // 设置配准后对应点之间的距离（以米为单位）。
    // fpcs.setNumberOfSamples(100);       // 设置验证配准效果时要使用的采样点数量
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);
    // fpcs.align(*pcs);                   // 计算变换矩阵
    // cout << "FPCS配准用时： " << time_LoadFiles.toc() << " ms" << endl;
    // cout << "变换矩阵：" << fpcs.getFinalTransformation() << endl;


    //----------use G-ICP---------------
    time_LoadFiles.tic();
    pcl::IterativeClosestPoint<PointT, PointT> gicp;
    gicp.setMaximumIterations(iterations);
    gicp.setInputSource(cloud_icp);
    gicp.setInputTarget(cloud_in_DS);
    gicp.align(*cloud_icp,T_coarse);
    gicp.setMaximumIterations (1);
    cout << "G-ICP配准用时： " << time_LoadFiles.toc() << " ms" << endl;
    cout << "变换矩阵：" << gicp.getFinalTransformation() << endl;
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time_LoadFiles.toc () << " ms" << std::endl;

    if (gicp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << gicp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = gicp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nG-ICP has not converged.\n");
        return (-1);
    }


    //------------Visualization-----------------
    pcl::visualization::PCLVisualizer viewer ("G-ICP demo");
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in_DS, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_in_DS, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in_DS, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr_DS, 20, 180, 20);
    viewer.addPointCloud (cloud_tr_DS, cloud_tr_color_h, "cloud_tr_v1", v1);

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
            time_LoadFiles.tic ();
            gicp.align (*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time_LoadFiles.toc () << " ms" << std::endl;

            if (gicp.hasConverged ())
            {
                printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nICP has converged, score is %+.0e\n", gicp.getFitnessScore ());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= gicp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
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



float getDegAngle3d(const Eigen::Vector3d v1, const Eigen::Vector3d v2)
{
	double radian = atan2(v1.cross(v2).norm(), v1.transpose() * v2); //弧度
	if (v1.cross(v2).z() < 0)
	{
		radian = 2*M_PI - radian;
	}

	return float(radian); //角度
}




float CalCurvatureChange(Eigen::Matrix3f _C)
{
    float min = 0;

    Eigen::EigenSolver<Eigen::Matrix3f> es12(_C);
    Eigen::MatrixXf value= es12.eigenvalues().real();
    min = value(0);
    for(int i=1;i<value.size();i++)
 	{
 		if(value(i) < min)
        {
            min = value(i);
        }
	}
    return (min /(value(0)+value(1)+value(2)));
};

Eigen::Vector3d NormalVec(PointCloudT::Ptr _cloud_source, int _indexies, int _K, float &_curveture)
{
    pcl::PointXYZ _searchPoint;
    vector<int> _pointIdxNKNSearch(_K);         //保存每个近邻点的索引
    vector<float> _pointNKNSquaredDistance(_K); //保存每个近邻点与查找 点之间的欧式距离平⽅
    pcl::KdTreeFLANN<pcl::PointXYZ> _kdtree;
    _searchPoint = _cloud_source->points[_indexies];
    _kdtree.setInputCloud(_cloud_source);
    int _a; //number of neighbors found
    _a = _kdtree.nearestKSearch(_searchPoint, _K, _pointIdxNKNSearch, _pointNKNSquaredDistance);//search neibordpoint around intereting point
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> _pointNormalEstimation;//法向量求解器
    Eigen::Vector4f _PlaneParam ;//法向量 head(3)
    _pointNormalEstimation.computePointNormal(*_cloud_source, _pointIdxNKNSearch, _PlaneParam, _curveture);
    Eigen::Vector3d n = _PlaneParam.head(3).cast<double>();
    return n;
}

void CalDescriptor(PointCloudT::Ptr _cloud, vector<Eigen::Vector3f> &_descriptor)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> _kdtree; //建⽴kdtree对象
    _kdtree.setInputCloud(_cloud);        //设置需要建⽴kdtree的点云指针
    pcl::PointXYZ searchPoint;
    int a; // number of neighbors found
    Eigen::Vector3f g_i;
    Eigen::Vector3f g_neibor;
    float cv_1;
    float cv_2;

    for (int i = 0; i < _cloud->points.size(); i++)
    {
        Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
        float sum_d = 0;
        float sum_th = 0;
        Eigen::Vector3d n_i;
        Eigen::Vector3d n_j;
        searchPoint = _cloud->points[i];
        vector<int> pointIdxNKNSearch(K);         //保存每个近邻点的索引
        vector<float> pointNKNSquaredDistance(K); //保存每个近邻点与查找 点之间的欧式距离平⽅
        // cout << "K nearest neighbor search at (" << searchPoint.x
        //      << " " << searchPoint.y
        //      << " " << searchPoint.z
        //      << ") with K=" << K << endl;

        a = _kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);//search neibordpoint around intereting point
        // cout << "point"
        //      << " (" << searchPoint.x 
        //      << " " << searchPoint.y 
        //      << " " << searchPoint.z
        //      << ") " << "近邻点个数：" 
        //      << pointIdxNKNSearch.size() << endl;
        
        for(int k=0; k<a; k++)
        {
            g_i += pointXYZ2Vec(_cloud->points[pointIdxNKNSearch[k]]);
        }
        g_i = g_i/a;
        
        n_i = NormalVec(_cloud,i,l, cv_1);//get normal vector
        for(int j=0; j<a; j++)
        {
            g_neibor = pointXYZ2Vec(_cloud->points[pointIdxNKNSearch[j]]);
            C += (g_neibor-g_i) * ((g_neibor-g_i).transpose());
            sum_d += sqrt(pointNKNSquaredDistance[j]);
            n_j = NormalVec(_cloud,pointIdxNKNSearch[j],l,cv_2);//get normal vector
            sum_th += getDegAngle3d(n_i,n_j);
        }
        // Eigen::Vector4f d = {sum_d/a, CalCurvatureChange(C/a), cv_1, sum_th};
        Eigen::Vector3f d = {sum_d/a, cv_1, sum_th};
        _descriptor.push_back(d);
    }

}

void pose_estimation_3d3d(const vector<Point3f> &pts1,
                          const vector<Point3f> &pts2,
                          Mat &R, Mat &t) {
  Point3f p1, p2;     // center of mass
  int N = pts1.size();
  for (int i = 0; i < N; i++) {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 = Point3f(Vec3f(p1) / N);
  p2 = Point3f(Vec3f(p2) / N);
  vector<Point3f> q1(N), q2(N); // remove the center
  for (int i = 0; i < N; i++) {
    q1[i] = pts1[i] - p1;
    q2[i] = pts2[i] - p2;
  }

  // compute q1*q2^T
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (int i = 0; i < N; i++) {
    W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
  }
  cout << "W=" << W << endl;

  // SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  cout << "U=" << U << endl;
  cout << "V=" << V << endl;

  Eigen::Matrix3d R_ = U * (V.transpose());
  if (R_.determinant() < 0) {
    R_ = -R_;
  }
  Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

  // convert to cv::Mat
  R = (Mat_<double>(3, 3) <<
    R_(0, 0), R_(0, 1), R_(0, 2),
    R_(1, 0), R_(1, 1), R_(1, 2),
    R_(2, 0), R_(2, 1), R_(2, 2)
  );
  t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

void Matching(PointCloudT::Ptr cloud_s,
                PointCloudT::Ptr cloud_t,
                vector<Eigen::Vector3f> descrp_s, 
                vector<Eigen::Vector3f> descrp_t, 
                vector<Point3f> &pts1, 
                vector<Point3f> &pts2)
{
    vector<float> dist;
    vector<vector<int>> matches1; //ID of corresponding points pair matched from 1 to 2
    vector<vector<int>> matches2; // ID of corresponding points pair matched from 2 to 1
    vector<vector<int>> matches;  // ID of corresponding points pair matched from both
    // vector<float> c_1;            // confidence of matches 1
    // vector<float> c_2;            // confidence of matches 2
    // vector<float> c;              // confidence of matches (symmetric match)

    for (int i = 0; i < descrp_s.size(); i++)
    {
        for (int j = 0; j < descrp_t.size(); j++)
        {
            dist.push_back((descrp_s[i] - descrp_t[j]).norm());
        }
        std::vector<int> index = argsort(dist);
        sort(dist.begin(), dist.end());
        if ((dist[1] != 0) && (sqrt(dist[0] / dist[1]) < tao)) // checking dist[1] to prevent dividing by zero
        {
            vector<int> ind_temp;
            ind_temp.push_back(i);
            ind_temp.push_back(index[0]);
            matches1.push_back(ind_temp);
            ind_temp.clear();
            // c_1.push_back(dist[0] / dist[1]);
        }
        dist.clear();
    }
    // Matchinf features from image 2 to image 1
    for (int i = 0; i < descrp_t.size(); i++)
    {
        for (int j = 0; j < descrp_s.size(); j++)
        {
            dist.push_back((descrp_t[i] - descrp_s[j]).norm());
        }
        std::vector<int> index = argsort(dist);
        sort(dist.begin(), dist.end());
        if ((dist[1] != 0) && (sqrt(dist[0] / dist[1]) < tao)) // checking dist[1] to prevent dividing by zero
        {
            vector<int> ind_temp;
            ind_temp.push_back(index[0]);
            ind_temp.push_back(i);
            matches2.push_back(ind_temp);
            ind_temp.clear();
            // c_2.push_back(dist[0] / dist[1]);
        }
        dist.clear();
        index.clear();
    }

    // Looking for Symmetric Matches that occured in both cases
    for (int i = 0; i < matches1.size(); i++)
    {
        for (int j = 0; j < matches2.size(); j++)
        {
            if ((matches1[i][0] == matches2[j][0]) && (matches1[i][1] == matches2[j][1]))
            {
                vector<int> ind_temp;
                ind_temp.push_back(matches1[i][0]);
                ind_temp.push_back(matches1[i][1]);
                matches.push_back(ind_temp);
                ind_temp.clear();
                // if(c_1[i] > c_2[j]) // selecting the lower of the two confidences (larger value)
                // {
                //     c.push_back(c_1[i]);
                // }
                // else{
                //     c.push_back(c_2[j]);
                // }
            }
        }
    }

    

    // convert PointXYZ to Point
    for(int i = 0; i<matches.size(); i++)
    {
        pts1.push_back(pointXYZ2Point(cloud_s->points[matches[i][0]]));
        pts2.push_back(pointXYZ2Point(cloud_t->points[matches[i][1]]));
    }

}


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