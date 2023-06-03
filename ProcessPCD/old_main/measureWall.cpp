#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/moment_of_inertia_estimation.h>

int main(int argc, char** argv)
{
    // Load point cloud from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/thuan12ha/_3Drobot_ws/ProcessPCD/input_simu/test_simu_1_5.pcd", *cloud);

    // Estimate the moment of inertia of the point cloud
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    // Get the minimum and maximum points of the moment of inertia's bounding box
    pcl::PointXYZ min_point, max_point;
    feature_extractor.getAABB(min_point, max_point);

    // Compute the length of the wall
    double length = max_point.x - min_point.x;

    // Create a crop box filter to extract a smaller point cloud containing only the wall
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(cloud);
    crop_filter.setMin(Eigen::Vector4f(min_point.x, min_point.y, min_point.z, 1.0));
    crop_filter.setMax(Eigen::Vector4f(max_point.x, max_point.y, max_point.z, 1.0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_filter.filter(*wall_cloud);

    // Compute the density of points on the wall
    double wall_area = length * (max_point.y - min_point.y);
    double wall_density = wall_cloud->size() / wall_area;

    std::cout << "Wall length: " << length << std::endl;
    std::cout << "Wall area: " << wall_area << std::endl;
    std::cout << "Number of points on wall: " << wall_cloud->size() << std::endl;
    std::cout << "Point density on wall: " << wall_density << " points/m^2" << std::endl;

    return 0;
}
