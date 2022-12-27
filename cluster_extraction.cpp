#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>//mls
#include <pcl/filters/bilateral.h>  
#include <pcl/search/flann_search.h> 
 


void Segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);


int main()
{
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read("1493.199455810.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*

    Segmentation(cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //*

    // // Create the segmentation object for the planar model and set all the parameters
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(100);
    // seg.setDistanceThreshold(0.02);

    // int nr_points = (int)cloud_filtered->size();
    // while (cloud_filtered->size() > 0.3 * nr_points)
    // {
    //     // Segment the largest planar component from the remaining cloud
    //     seg.setInputCloud(cloud_filtered);
    //     seg.segment(*inliers, *coefficients);
    //     if (inliers->indices.size() == 0)
    //     {
    //         std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    //         break;
    //     }

    //     // Extract the planar inliers from the input cloud
    //     pcl::ExtractIndices<pcl::PointXYZ> extract;
    //     extract.setInputCloud(cloud_filtered);
    //     extract.setIndices(inliers);
    //     extract.setNegative(false);

    //     // Get the points associated with the planar surface
    //     extract.filter(*cloud_plane);
    //     std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

    //     // Remove the planar inliers, extract the rest
    //     extract.setNegative(true);
    //     extract.filter(*cloud_f);
    //     *cloud_filtered = *cloud_f;
    // }

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


        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ra(new pcl::PointCloud<pcl::PointXYZ>);
        outrem.setInputCloud(cloud_cluster);
        outrem.setRadiusSearch(0.03);
        outrem.setMinNeighborsInRadius(2);
        outrem.filter(*cloud_ra);



        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bi(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::BilateralFilter<pcl::PointXYZ> bf;
        // tree1->setInputCloud(cloud_cluster);
        // bf.setInputCloud(cloud_cluster);
        // bf.setSearchMethod(tree1);
        // bf.setHalfSize(0.01);
        // bf.setStdDev(1);
        // bf.filter(*cloud_bi);

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; // 定义最小二乘实现的对象mls
        // mls.setComputeNormals(false);                //设置在最小二乘计算中是否需要存储计算的法线
        // mls.setInputCloud(cloud_filtered);           //设置待处理点云
        // mls.setPolynomialOrder(2);                   // 拟合2阶多项式拟合
        // mls.setPolynomialFit(false);                 // 设置为false可以 加速 smooth
        // mls.setSearchMethod(tree);           // 设置KD-Tree作为搜索方法
        // mls.setSearchRadius(0.05);                   // 单位m.设置用于拟合的K近邻半径
        // mls.process(*cloud_smoothed);                //输出

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fi(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud(cloud_cluster);
        // sor.setMeanK(50);
        // sor.setStddevMulThresh (1.0);
        // sor.filter(*cloud_fi);

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        std::cout << "PointCloud representing the cloud_ra: " << cloud_ra->size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
        ss.clear();
        ss << "cloud_ra" << j << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_ra, false); //*
        j++;
    }

    return (0);
}

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