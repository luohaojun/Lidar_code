#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //使用随机样本一致性来识别inliers

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;


void Segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    pcl::PassThrough<pcl::PointXYZ> pt;
    // Filter in z direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(-0.7f, 2.0f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
    // Filter in x direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(-1.0f, 5.0f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
    // Filter in y direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(-1.0f, 2.5f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
}



pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cl)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    Segmentation(cloud_cl);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud_cl);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //*

    // Creating the KdTree object for the search method of the extraction  
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

        int j = 0;
    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->header = cloud_filtered->header;
        cloud_cluster->sensor_origin_ = cloud_filtered->sensor_origin_;
        cloud_cluster->sensor_orientation_ = cloud_filtered->sensor_orientation_;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        cloud_out = cloud_cluster;
        j++;
    }
    return cloud_out;
}





int main(int argc, char **argv)
{
    // 加载第一次扫描点云数据作为目标云
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("0908_asc.ply", *target) == -1)
    {
        PCL_ERROR("读取目标点云失败 \n");
        return (-1);
    }
    cout << "从目标点云中读取 " << target->size() << " 个点" << endl;

    // 加载从新视角得到的第二次扫描点云数据作为源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("1493.199455810.pcd", *source) == -1)
    {
        PCL_ERROR("读取源标点云失败 \n");
        return (-1);
    }
    cout << "从源点云中读取 " << source->size() << " 个点" << endl;

    //---------point cloud process--------
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    float leaf = 35.0f;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(target);
    grid.filter(*target);
    for (int i = 0; i < target->size(); i++)
    {
        target->points[i].x = target->points[i].x / 1000.0;
        target->points[i].y = target->points[i].y / 1000.0;
        target->points[i].z = target->points[i].z / 1000.0;
    }
    Segmentation(source);
    source = cluster(source);


    //---------初始化对象获取匹配点对----------------------
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
    core.setInputSource(source);
    core.setInputTarget(target);
    boost::shared_ptr<pcl::Correspondences> correspondence_all(new pcl::Correspondences);
    core.determineCorrespondences(*correspondence_all); //确定输入点云与目标点云之间的对应关系
    //------------RANSAC筛选内点----------------------------
    boost::shared_ptr<pcl::Correspondences> correspondence_inlier(new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac;
    ransac.setInputSource(source);
    ransac.setInputTarget(target);
    ransac.setMaximumIterations(200); //设置最大迭代次数
    ransac.setInlierThreshold(0.05);  //设置对应点之间的最大距离
    // ransac.setRefineModel(true);//指定是否应该使用inliers的方差在内部细化模型
    ransac.getRemainingCorrespondences(*correspondence_all, *correspondence_inlier);
    //-----------输出必要信息到控制台------------------------
    cout << "ransac前的匹配点对有：" << correspondence_all->size() << "对" << endl;
    cout << "ransac后的匹配点对有：" << correspondence_inlier->size() << "对" << endl;

    cout << "ransac前的匹配点对：\n ";
    for (int i = 0; i < correspondence_all->size(); i++)
    {
        cout << i << "\tindex_match:\t" << correspondence_all->at(i).index_match << "\tindex_query:\t" << correspondence_all->at(i).index_query << "\n";
    }

    cout << "ransac后的匹配点对：\n ";
    for (int i = 0; i < correspondence_inlier->size(); i++)
    {
        cout << i << "\tindex_match:\t" << correspondence_inlier->at(i).index_match << "\tindex_query:\t" << correspondence_inlier->at(i).index_query << "\n";
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));
    viewer->setBackgroundColor(0, 0, 0); //设置背景颜色为黑色
    // 对目标点云着色可视化 (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    // 对源点云着色可视化 (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(source, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(source, input_color, "input cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
    //对应关系可视化
    viewer->addCorrespondences<pcl::PointXYZ>(source, target, *correspondence_inlier, "correspondence");
    // viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
