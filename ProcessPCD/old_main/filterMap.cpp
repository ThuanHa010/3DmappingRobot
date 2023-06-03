#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
  // Read in point cloud from .pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string filename = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/input_simu/test_simu_1_5.pcd";
  pcl::io::loadPCDFile(filename, *cloud);
  std::cout << "Loaded " << cloud->size() << " data points from " << filename << std::endl;

  // Apply voxel grid filter to downsample the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel_filter.filter(*cloud_downsampled);
  std::cout << "Downsampled cloud to " << cloud_downsampled->size() << " data points" << std::endl;

  // Apply pass-through filter to extract a region of interest
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_downsampled);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-2.0f, 3.0f);
  pass.filter(*cloud_filtered);
  std::cout << "Filtered cloud to " << cloud_filtered->size() << " data points" << std::endl;
  
  // Apply statistical outlier removal filter to remove noise
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
  sor_filter.setInputCloud(cloud_filtered);
  sor_filter.setMeanK(50);
  sor_filter.setStddevMulThresh(1.0);
  sor_filter.filter(*cloud_filtered2);
  std::cout << "Removed " << cloud_filtered->size() - cloud_filtered2->size() << " outlier points" << std::endl;

  // Save filtered point cloud to new .pcd file
  std::string filtered_filename = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/output/out_filtered_2_5.pcd";
  pcl::io::savePCDFileASCII(filtered_filename, *cloud_filtered);
  std::cout << "Saved filtered point cloud to " << filtered_filename << std::endl;

  // Visualize the original and filtered point clouds
  pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");
  viewer.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 255, 255);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, color_handler, "cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler2(cloud_filtered, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, color_handler2, "cloud_filtered");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");
  viewer.addCoordinateSystem(0.5);
  viewer.initCameraParameters();
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }

  return 0;
}
