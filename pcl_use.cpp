//pcd文件可视化
#include <iostream> 
#include <unistd.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


void Segmentation(PointCloudT::Ptr cloud_in);
PointCloudT::Ptr RemoveZeroPoints(PointCloudT::Ptr cloud_load);

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float vg_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(vg_size,vg_size, vg_size);
    vg.filter(*cloud_out);
}



int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("1493.199455810.pcd", *cloud) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read file part0.pcd \n");
        pause();
        return (-1);
    }
        std::cout << "\nLoaded file "
              << "bun045.pcd"
              << " (" << cloud->size() << " points) " << " ms\n"
              << std::endl;

    
    // Segmentation(cloud);
    // std::vector<int> indices;
    //     pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    //     // cout<<"cloud_tr size after removeNaN: "<<(*cloud_tr).size()<<endl;
    //     downsample(cloud, cloud, 40.0);
    //     std::cout << "cloud_rf size: " << cloud->points.size() << endl;
    //     //---------scale---------
    //     for (int i = 0; i < cloud->size(); i++)
    //     {
    //         cloud->points[i].x = cloud->points[i].x / 1000.0;
    //         cloud->points[i].y = cloud->points[i].y / 1000.0;
    //         cloud->points[i].z = cloud->points[i].z / 1000.0;
    //     }
    
    // cloud_out = RemoveZeroPoints(cloud);
    cout<<"size after remove NAN: "<<cloud->points.size()<<endl;
    // cout<<"size after remove NAN: "<<cloud_out->points.size()<<endl;

    // for (int i = 0; i < cloud_out->points.size(); i++)
    // {
    //     cout << "point " << i << " : " << cloud_out->points[i] << endl;
    // }

    // pcl::StatisticalOutlierRemoval::applyFileter()
    pcl::visualization::CloudViewer viewer("3DViewer"); //创建窗口
    viewer.showCloud(cloud);

    pause(); //按任意键继续...
    return (0);
}


void Segmentation(PointCloudT::Ptr cloud_in)
{
    pcl::PassThrough<PointT> pt;
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

PointCloudT::Ptr RemoveZeroPoints(PointCloudT::Ptr cloud_load)
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