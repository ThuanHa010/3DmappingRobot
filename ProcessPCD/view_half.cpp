#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    std::string path = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_7_5/pcd/main_7_5_obj_L.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map2(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *map) == -1)
    {
        PCL_ERROR("Couldn't read file map.pcd\n");
        return (-1);
    }
    for(auto& p: *map)
    {
       if(p.y >= 0)
        {
            pcl::PointXYZ point;
            point.x = p.x ;
            point.y = p.y ;
            point.z = p.z ;
            map2->push_back(point);
        }
    }
    auto outmap = map2;
    pcl::visualization::PCLVisualizer viewer("Map Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(outmap, 0, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(outmap, color_handler,"map");

    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    return 0;
}
