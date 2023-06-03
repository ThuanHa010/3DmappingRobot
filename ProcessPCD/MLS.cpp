#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <iomanip>
#include <sstream>



std::string getCurrentDateTimeAsString()
{
    // Get current system time
    auto now = std::chrono::system_clock::now();

    // Convert to a time_t (seconds since epoch)
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

    // Convert to a tm struct
    std::tm* currentTimeStruct = std::localtime(&currentTime);

    // Format the date and time as a string
    std::stringstream ss;
    ss << std::put_time(currentTimeStruct, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}


std::ofstream fw("/home/thuan12ha/_3Drobot_ws/ProcessPCD/output/output.txt", std::ofstream::out);

int main()
{
    std::string dataTime = get_current_dir_name();
    std::string fileName = "out_test_11_04.pcd";
    std::string dir = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/input_jetson/";
    std::string path = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/input_jetson/test_jetson_13_05/main_13_5_v04_2obj.pcd";
    std::string outpath = dir + dataTime + fileName;
    // Load the point cloud map from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *map) == -1)
    {
        PCL_ERROR("Couldn't read file map.pcd\n");
        return (-1);
    }
    std::cout << "Size of map " << map->size() << std::endl;
    // Downsample the point cloud using a voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(map);
    voxel_grid_filter.setLeafSize(0.02f, 0.02f, 0.02f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid_filter.filter(*map_downsampled);

    // Apply a pass-through filter to remove points outside a specified range 
    //Z
    pcl::PassThrough<pcl::PointXYZ> pass_filter;

    pass_filter.setInputCloud(map_downsampled);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(0.0f, 4.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_filteredZ(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*map_filteredZ);

    //Y
    pass_filter.setInputCloud(map_filteredZ);
    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(-2.0f, 2.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_filteredY(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*map_filteredY);    
    std::cout << "Size of filtered points " << (map->size() - map_filteredY->size()) << std::endl;
    // Compute normals for the filtered point cloud
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimator.setSearchMethod(search_tree);
    normal_estimator.setInputCloud(map_filteredY);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimator.setRadiusSearch(0.1f);
    normal_estimator.compute(*normals);

    // Apply an MLS filter to smooth the point cloud
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls_filter;
    mls_filter.setInputCloud(map_filteredY);
    mls_filter.setPolynomialFit(true);
    mls_filter.setSearchMethod(search_tree);
    mls_filter.setSearchRadius(0.1f);
    mls_filter.setComputeNormals(true);
    mls_filter.setPolynomialOrder(2);
    mls_filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls_filter.setUpsamplingRadius(0.1f);
    mls_filter.setUpsamplingStepSize(0.05f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_smoothed(new pcl::PointCloud<pcl::PointXYZ>);
    mls_filter.process(*map_smoothed);
    std::cout << "Size of improved map " << map_smoothed->size() << std::endl;

    // Save filtered point cloud to new .pcd file
    // std::string filtered_filename = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/output/out_7_5.pcd";
    // pcl::io::savePCDFileASCII(filtered_filename, *map_smoothed);
    // std::cout << "Saved filtered point cloud to " << filtered_filename << std::endl;
    //View
    pcl::visualization::PCLVisualizer viewer("Map Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    //viewer.addPointCloud<pcl::PointXYZ>(map, "map");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(map_smoothed, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(map_smoothed, color_handler,"map_smoothed");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map_smoothed");
    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    return 0;
}
