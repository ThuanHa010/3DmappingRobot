#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    // Load reference and comparison point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_comp (new pcl::PointCloud<pcl::PointXYZ>);
    
    std::string ref_path = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/table/mapA.pcd";
    std::string comp_path = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/table/mapA2.pcd";

    pcl::io::loadPCDFile<pcl::PointXYZ>(ref_path, *cloud_ref);
    pcl::io::loadPCDFile<pcl::PointXYZ>(comp_path, *cloud_comp);

    // Remove outliers (optional)
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(cloud_ref);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(1.0);
    // sor.filter(*cloud_ref);
    // sor.setInputCloud(cloud_comp);
    // sor.filter(*cloud_comp);
    
    // If data_jetson, using pass-through filter

    // Downsample using Voxel Grid filter
    // pcl::VoxelGrid<pcl::PointXYZ> vg;
    // vg.setInputCloud(cloud_ref);
    // vg.setLeafSize(0.01f, 0.01f, 0.01f);
    // vg.filter(*cloud_ref);
    // vg.setInputCloud(cloud_comp);
    // vg.filter(*cloud_comp);

    // Perform ICP registration
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_comp);
    icp.setInputTarget(cloud_ref);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*cloud_aligned);

    // Compute difference between aligned and reference point clouds
    pcl::SegmentDifferences<pcl::PointXYZ> seg_diff;
    seg_diff.setInputCloud(cloud_aligned);
    seg_diff.setTargetCloud(cloud_ref);
    seg_diff.setDistanceThreshold(0.02);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ>);
    seg_diff.segment(*cloud_diff);
    std::cout << "Size aligned map : "<< cloud_aligned->size() << std::endl;

    // Visualize reference point cloud in blue and change points in red
    pcl::visualization::PCLVisualizer viewer("ICP registration");
    viewer.addPointCloud(cloud_ref, "cloud_ref");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud_diff, 255, 0, 0);
    viewer.addPointCloud(cloud_diff,color_handler, "cloud_diff");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_diff");
    viewer.spin();

    return 0;
}
