#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    std::string path = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/merge.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *map) == -1)
    {
        PCL_ERROR("Couldn't read file map.pcd\n");
        return (-1);
    }

    pcl::visualization::PCLVisualizer viewer("Map Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(map, 0, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(map, color_handler,"map");

    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    return 0;
}
