#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace pcl::io;

int main (int argc, char **argv)
{
    pcl::PLYReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read<pcl::PointXYZ>("0908_asc.ply", *cloud);
    pcl::io::savePCDFile("11_05.pcd",*cloud);



    return 0;
}