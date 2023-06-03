#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    // Load  point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);    
    std::string path = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/_data_simulation/test_simu_1_5/pcd/test_simu_1_5.pcd";

    pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
    std::cout << "Size before cut: "<< cloud->size() << std::endl;

    // Apply a pass-through filter to remove points outside a specified range 
    //Z
    pcl::PassThrough<pcl::PointXYZ> pass_filter;

    pass_filter.setInputCloud(cloud);
    pass_filter.setFilterFieldName("x");
    pass_filter.setFilterLimits(-0.2f, 9.5752f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cutcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*cutcloud);
    std::cout << "Size after cut: "<< cutcloud->size() << std::endl;
    // Save filtered point cloud to new .pcd file
    std::string output_filename = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/output/test_simu_1_5_cutX.pcd";
    pcl::io::savePCDFileASCII(output_filename, *cutcloud);
    //View
    pcl::visualization::PCLVisualizer viewer("Map Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    //viewer.addPointCloud<pcl::PointXYZ>(map, "map");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cutcloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cutcloud, color_handler,"cut_cloud");
    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return 0;
}
