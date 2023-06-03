#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <typeinfo>

int main(int argc, char** argv)
{
    // Load the point cloud map from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/input_jetson/test_jetson_7_5/main_7_5.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *map) == -1)
    {
        PCL_ERROR("Couldn't read file map.pcd\n");
        return (-1);
    }
    std::cout << "Size of map \t\t" << map->size() << std::endl;
    // Downsample the point cloud using a voxel grid filter
    // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    // voxel_grid_filter.setInputCloud(map);
    // voxel_grid_filter.setLeafSize(0.1f, 0.1f, 0.1f);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr map_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    // voxel_grid_filter.filter(*map_downsampled);
    // std::cout << "Size of map after voxel filer \t" << map_downsampled->size() << std::endl;
    // Apply a pass-through filter to remove points outside a specified range 
    //X
    pcl::PassThrough<pcl::PointXYZ> pass_filter;

    pass_filter.setInputCloud(map);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(-0.1f, 4.5f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_filteredZ(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*map_filteredZ); 
    std::cout << "Size of map after pass through Z filer \t" << map_filteredZ->size() << std::endl;

    pass_filter.setInputCloud(map_filteredZ);
    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(-2.0f, 1.7f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_filteredY(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*map_filteredY); 
    std::cout << "Size of map after pass through Y filer \t" << map_filteredY->size() << std::endl;
    // // Apply statistical outlier removal filter to remove noise
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
    // sor_filter.setInputCloud(map_filteredX);
    // sor_filter.setMeanK(50);
    // sor_filter.setStddevMulThresh(1.0);
    // sor_filter.filter(*cloud_sor);
    // int remove = (cloud_sor->size())-(map_filteredX->size());
    // std::cout << "Removed " <<  remove << " outlier points" << std::endl; 

    // Compute normals for the filtered point cloud
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // normal_estimator.setSearchMethod(search_tree);
    // normal_estimator.setInputCloud(map_filteredZ);
    // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // normal_estimator.setRadiusSearch(0.1f);
    // normal_estimator.compute(*normals);

    // Apply an MLS filter to smooth the point cloud
    // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls_filter;
    // mls_filter.setInputCloud(map_filteredZ);
    // mls_filter.setPolynomialFit(true);
    // mls_filter.setSearchMethod(search_tree);
    // mls_filter.setSearchRadius(0.1f);
    // mls_filter.setComputeNormals(true);
    // mls_filter.setPolynomialOrder(2);
    // mls_filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // mls_filter.setUpsamplingRadius(0.1f);
    // mls_filter.setUpsamplingStepSize(0.05f);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr map_smoothed(new pcl::PointCloud<pcl::PointXYZ>);
    // mls_filter.process(*map_smoothed);
    
    auto output_map = map_filteredY ;
    std::cout << "Size of improved map \t" << output_map->size() << std::endl;
    // Save filtered point cloud to new .pcd file
    // std::string output_filename = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/output/process_jetson_7_5/out_7_5.pcd";
    // pcl::io::savePCDFileASCII(output_filename, *output_map);
    // std::cout << "Saved filtered point cloud to " << output_filename << std::endl;
    
    //View
    pcl::visualization::PCLVisualizer viewer("Map Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(output_map, 0, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(output_map, color_handler,"map_smoothed");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map_smoothed");
    viewer.addCoordinateSystem(0.5);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    return 0;
}