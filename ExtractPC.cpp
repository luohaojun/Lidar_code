#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char **argv)
{

    // 定义变量
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        cloud_3(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        cloud_4(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        cloud_5(new pcl::PointCloud<pcl::PointXYZRGB>());

    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/RefPointCloud/338.100142210.pcd", *cloud_1))
    // {
    //     std::cerr << "ERROR: Cannot open file " << std::endl;
    //     return 0;
    // }
    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/RefPointCloud/338.199982530.pcd", *cloud_2))
    // {
    //     std::cerr << "ERROR: Cannot open file " << std::endl;
    //     return 0;
    // }
    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/RefPointCloud/338.299814810.pcd", *cloud_3))
    // {
    //     std::cerr << "ERROR: Cannot open file " << std::endl;
    //     return 0;
    // }
    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/RefPointCloud/338.399651690.pcd", *cloud_4))
    // {
    //     std::cerr << "ERROR: Cannot open file " << std::endl;
    //     return 0;
    // }
    // if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/RefPointCloud/338.499484190.pcd", *cloud_5))
    // {
    //     std::cerr << "ERROR: Cannot open file " << std::endl;
    //     return 0;
    // }

    // *cloud_sum = *cloud_1 + *cloud_2 + *cloud_3 + *cloud_4 + *cloud_5;

    if (pcl::io::loadPCDFile("/home/luo/EX_ws/src/my_pcl/rosbag/ScanPointCloud/1499.700249990.pcd", *cloud_1) < 0)
    {
        PCL_ERROR("Cloudn't read cloud_in file!");
        return (-1);
    }
    std::cout << "\nLoaded file "
              << "bun_zipper.pcd"
              << " (" << cloud_1->size() << " points)" << std::endl;

    // 创建直通滤波（模板）类对象，点数据类型为 pcl::PointXYZRGB
    pcl::PassThrough<pcl::PointXYZRGB> pt;
    // 设置输入点云，注意：此处传入的是点云类对象的智能指针
    pt.setInputCloud(cloud_1);
    // 设置通道名称 x/y/z
    pt.setFilterFieldName("z");
    // 设置当前裁剪通道的最值区间，默认保留区间内的数据
    pt.setFilterLimits(0.0f,0.2f);
    // 是否对区间范围取反，即保留区间外的数据
    pt.setFilterLimitsNegative(false);
    // 执行过滤，并带出处理后的点云数据，注意：此处传入的是点云类对象
    pt.filter(*cloud_filtered);

    //Filter in x direction
    pt.setInputCloud(cloud_filtered);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(0.0f,3.0f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_filtered);

    //Filter in y direction
    pt.setInputCloud(cloud_filtered);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(-0.5f,2.5f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_filtered);

    

    // 创建可视化对象
    pcl::visualization::PCLVisualizer viewer("viewer");

    // 将当前窗口，拆分成横向的2个可视化窗口，以viewport区分(v1/v2)
    int v1; 
    int v2;
    //窗口参数分别对应 x_min, y_min, x_max, y_max, viewport
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);  
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // 添加2d文字标签
    viewer.addText("v1", 10,10, 20, 1,0,0, "viewport_v1", v1);
    viewer.addText("v2", 10,10, 20, 0,1,0, "viewport_v2", v2);

    // 添加坐标系
    viewer.addCoordinateSystem(0.5);    // 单位：m

    // 设置可视化窗口背景色
    viewer.setBackgroundColor(1,1,1);     // r,g,b  0~1之间

    // 向v1窗口中添加点云
    viewer.addPointCloud(cloud_sum,"cloud_src",v1);
    // 根据点云id，设置点云可视化属性，此处将可视化窗口中的点大小调整为2级
    viewer.setPointCloudRenderingProperties \
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_sum");

    // 向v2窗口中添加点云
    viewer.addPointCloud(cloud_filtered,"cloud_filtered",v2);
    // 根据点云id，设置点云可视化属性，此处将可视化窗口中的点大小调整为2级
    viewer.setPointCloudRenderingProperties \ 
        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered");

    // 关闭窗口则退出
    while(!viewer.wasStopped()){
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
