#include <iostream>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>          //kdtree近邻搜索
#include <pcl/io/pcd_io.h>                    //⽂件输⼊输出
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>                  //点类型相关定义
#include <pcl/visualization/pcl_visualizer.h> //可视化相关定义
#include <boost/thread/thread.hpp>
#include <pcl/filters/filter.h>
using namespace std;

int main()
{
    //读取点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/luo/Group_UAV/Lidar/dataset/bunny/data/bun045.ply", *cloud) == -1)
    {
        PCL_ERROR("Cloudn't read file!");
        return -1;
    }
    cloud->is_dense = false;
    //remove NaN points
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

    //Convert PointXYZ to PointXYZRGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_R(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZRGB  p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        p.r = 255;
        p.g = 255;
        p.b = 255;
        cloud_R->points.push_back(p);
    }
    cout<<"cloud_R size: "<<cloud_R->points.size()<<endl;
    cout<<"point[28011] in cloud: \n"<<cloud->points[28011]<<endl;

    //建⽴kd-tree
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree; //建⽴kdtree对象
    kdtree.setInputCloud(cloud_R);               //设置需要建⽴kdtree的点云指针

    // K近邻搜索
    pcl::PointXYZRGB searchPoint = cloud_R->points[20000]; //设置查找点
    cloud_R->points[20000].r = 0;                          //查询点着⾊（绿⾊）
    cloud_R->points[20000].g = 255;
    cloud_R->points[20000].b = 0;
    int K = 1000;                              //设置需要查找的近邻点个数
    vector<int> pointIdxNKNSearch(K);         //保存每个近邻点的索引
    vector<float> pointNKNSquaredDistance(K); //保存每个近邻点与查找 点之间的欧式距离平⽅

    cout << "K nearest neighbor search at (" << searchPoint.x
         << " " << searchPoint.y
         << " " << searchPoint.z
         << ") with K=" << K << endl;

    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 1; i < pointIdxNKNSearch.size(); ++i)
        {                                                // i=1时不包含被查询点本身，i=0时包含被查询点
            cloud_R->points[pointIdxNKNSearch[i]].r = 255; //查询点邻 域内的点着⾊
            cloud_R->points[pointIdxNKNSearch[i]].g = 0;
            cloud_R->points[pointIdxNKNSearch[i]].b = 0;
        }
    }
    cout << "K = 100近邻点个数：" << pointIdxNKNSearch.size() << endl;
    //可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_R);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_R, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云⼤⼩
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }

    return 0;
}


