#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


int main()
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_0(new PointCloudT); // ICP output point cloud
    PointCloudT::Ptr cloud_1(new PointCloudT); // ICP output point cloud

    

    //-----load files-------
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/luo/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2/0000000017.pcd", *cloud_0) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read file part0.pcd \n");
        pause();
        return (-1);
    }
        std::cout << "\nLoaded file "
              << "bun045.pcd"
              << " (" << cloud_0->size() << " points) " << " ms\n"
              << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/luo/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2/0000000018.pcd", *cloud_1) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read file part0.pcd \n");
        pause();
        return (-1);
    }
        std::cout << "\nLoaded file "
              << "bun045.pcd"
              << " (" << cloud_1->size() << " points) " << " ms\n"
              << std::endl;






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
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h (cloud_0, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_0, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_1, cloud_in_color_h, "cloud_in_v2", v2);


    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);


    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce (100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));

    }



    return 0;
}