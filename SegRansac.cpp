#include <iostream>
#include <algorithm>
#include <string>
#include <unistd.h>
#include <unordered_set>
#include <chrono>

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
#include <pcl/filters/crop_box.h>
#include <boost/thread/thread.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <cmath>

#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Segment
{
public:
    //constructor
    Segment(){};
    //deconstructor
    ~Segment(){};
    void numPoints(PointCloudT::Ptr cloud);

    vector<boost::filesystem::path> streamPcd(string dataPath);
    PointCloudT::Ptr loadPcd(string file);
    // PointCloudT::Ptr FilterCloud(PointCloudT::Ptr cloud_load,float filterRes,Vector4f minPoint,Vector4f maxPoint);
    PointCloudT::Ptr FilterCloud(PointCloudT::Ptr cloud_RemoveZero);
    PointCloudT::Ptr RemoveZeroPoints(PointCloudT::Ptr cloud_load);
    PointCloudT::Ptr RansacSeg(PointCloudT::Ptr cloud_filtered);

private:
    float _R = 0.9; //the max edge of sphere
    float _r = 0.4; //the min edge of sphere
    // int maxIterations = 100; // the number of RANSAC iterations;
    int NumThreshold = 10; // the number threshold of points between _r and _R
    float filterRes = 0.01; // Radius of VoxelGrid
    int inliersthre = 40;// the min number of inliers
};





int main()
{
    cout << "starting enviroment" << endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    Segment *segment = new Segment();
    vector<boost::filesystem::path> stream= segment->streamPcd("/home/luo/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_3");
    auto streamIterator{stream.begin()};
    PointCloudT::Ptr cloud;


    while (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        cloud = segment->loadPcd((*streamIterator).string());
        // PointCloudT::Ptr cloud_RemoveZero = segment->RemoveZeroPoints(cloud);
        // PointCloudT::Ptr cloud_filtered = segment->FilterCloud(cloud_RemoveZero);
        // PointCloudT::Ptr cloud_Seg = segment->RansacSeg(cloud_filtered);
        viewer->addPointCloud(cloud,"cloud_Segment");
        streamIterator++;
        if (streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }
        viewer->spinOnce(100);
    }

    return 0;
}

vector<boost::filesystem::path> Segment::streamPcd(string dataPath)
{
    vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

PointCloudT::Ptr Segment::loadPcd(string file)
{
    PointCloudT::Ptr cloud_load (new PointCloudT);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud_load) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud_load->points.size () << " data points from "+file << std::endl;

    return cloud_load;
}

PointCloudT::Ptr Segment::RemoveZeroPoints(PointCloudT::Ptr cloud_load)
{
    PointCloudT::Ptr cloud_RemoveZero (new PointCloudT());
    for (int i = 0; i<cloud_load->points.size(); i++)
    {
        if(cloud_load->points[i].x == 0 && cloud_load->points[i].y == 0 && cloud_load->points[i].z == 0)
            continue;
        else{
            cloud_RemoveZero->points.push_back(cloud_load->points[i]);
        }
    }

    return cloud_RemoveZero;
}


// Segment::FilterCloud(PointCloudT::Ptr cloud_load,float filterRes,Vector4f minPoint,Vector4f maxPoint)
PointCloudT::Ptr Segment::FilterCloud(PointCloudT::Ptr cloud_RemoveZero)
{
    // PointCloudT::Ptr cloud_downsampled(new PointCloudT());
    // PointCloudT::Ptr cloud_nearremoved(new PointCloudT());
    // PointCloudT::Ptr cloud_filtered(new PointCloudT());
    PointCloudT::Ptr cloud_PassThrough(new PointCloudT());
    auto startTime{chrono::steady_clock::now()};

    pcl::PassThrough<PointT> pt;
    // Filter in z direction
    pt.setInputCloud(cloud_RemoveZero);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(-0.5f, 3.0f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_RemoveZero);
    // Filter in x direction
    pt.setInputCloud(cloud_RemoveZero);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(-1.0f, 3.5f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_PassThrough);

    // pcl::VoxelGrid<PointT> vg;
    // vg.setInputCloud(cloud_PassThrough);
    // vg.setLeafSize(filterRes, filterRes, filterRes);
    // vg.filter(*cloud_downsampled);

    // pcl::CropBox<PointT> region(true);
    // region.setInputCloud(cloud_downsampled);
    // region.setMin(Vector4f(-2.0, -1.5, -2, 1));
    // region.setMax(Vector4f(2.7, 1.5, 0, 1));
    // region.setNegative(true);
    // region.filter(*cloud_nearremoved);

    // pcl::CropBox<PointT> ro(true);
    // ro.setInputCloud(cloud_nearremoved);
    // ro.setMin(minPoint);
    // ro.setMax(maxPoint);
    // ro.filter(*cloud_filtered);

    auto endTime(chrono::steady_clock::now());
    auto elapsedTime{chrono::duration_cast<chrono::milliseconds>(endTime - startTime)};
    cout<<"size of cloud_PassThrough: "<<cloud_PassThrough->points.size()<<endl;
    cout << "filtering took " << elapsedTime.count() << " ms" <<endl;

    return cloud_PassThrough;
}

PointCloudT::Ptr Segment::RansacSeg(PointCloudT::Ptr cloud_filtered)
{
    PointCloudT::Ptr cloud_seg(new PointCloudT());
    auto startTime = chrono::steady_clock::now();
    std::unordered_set<int> center_hist;
    srand(time(NULL));
    int maxIterations = 1000;
    while (maxIterations--)
    {
        // cout<<"Iteration of Ransac "<<100-maxIterations<<endl;
        int center;
        int near_count = 0;
        std::unordered_set<int> inliersResult;
        std::unordered_set<int> inliers;
        center=rand() % (cloud_filtered->points.size());
        while (center_hist.count(center))
        {
            center=(rand() % (cloud_filtered->points.size()));
        }

        center_hist.insert(center);
        cout<<"size of center_hist: "<<center_hist.size()<<endl;

        float x0, y0, z0;

        x0 = cloud_filtered->points[center].x;
        y0 = cloud_filtered->points[center].y;
        z0 = cloud_filtered->points[center].z;

        for (int index = 0; index < cloud_filtered->points.size(); index++)
        {
            PointT point = cloud_filtered->points[index];
            float x1 = point.x;
            float y1 = point.y;
            float z1 = point.z;

            float dis = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1) + (z0-z1)*(z0-z1));
            // cout<<"distance: "<<dis<<endl;

            if (dis <= _r)
            {
                inliers.insert(index);
            }

            if (dis >= _r && dis <= _R)
            {
                near_count ++;
            }
        }
        // cout<<"size of inliers: "<<inliers.size()<<endl;
        // cout<<"size of near_count: "<<near_count<<endl;
        if(near_count <= NumThreshold)
        {
            inliersResult = inliers;
        }
        if (inliersResult.size() >= inliersthre)
        {
            for (int index1 = 0; index1 < cloud_filtered->points.size(); index1++)
            {
                PointT point1 = cloud_filtered->points[index1];
                if (inliersResult.count(index1))
                    cloud_seg->points.push_back(point1);
            }
            break;
        }
        cout<<"size of inliersResult: "<<inliersResult.size()<<endl;
    }
    
    cout<<"size of cloud_seg: "<<cloud_seg->points.size()<<endl;
    return cloud_seg;
}
