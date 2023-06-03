#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <iostream>

#define PI 3.14159265359
using namespace std;

int main(int argc, char** argv)
{
    std::string pathMapA = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/table/mapA.pcd";
    std::string pathMapB = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/table/mapB.pcd";
    std::string pathMapC = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/table/mapC.pcd";
    std::string pathMapD= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/table/mapD.pcd";


    pcl::PointCloud<pcl::PointXYZ>::Ptr mapA(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapB(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapD(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapA, *mapA);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapB, *mapB);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapC, *mapC);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapD, *mapD);

    // old : 3.0, 3.2, 7x0.002
    /*
    A->B->C->D, HCN (axb)
    x = x1*cos(th) - y1*sin(th) + x0
    y = x1*sin(th) + y1*cos(th) + y0
    A-> B : th = 0, x0 = 0, y0= 0
    B-> C : th = -pi/2, x0 = a;y0=0
    C-> D : th = -pi , x0 = a; y0 = -b
    D-> A : th = -3*pi/2 , x0 = 0; y0 = -b    
    */
    int a = 3.0 ;
    int b = 3.2 ;
    float th = 0;
    float x0 = 0;
    float y0 = 0;
    float z0 = 0;
    /*A */
    th = 0;
    x0 = 0;
    y0 = 0;
    for(auto& p: *mapA)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z;
        merged->push_back(point);
    }
    /*B*/
    th = PI/2;
    x0 = 8.17263;
    y0 = 0;
    for(auto& p: *mapB)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z;
        merged->push_back(point);
    }
    /* C*/
    th = PI;
    x0 = 8.17263;
    y0 = 7.53936;
    for(auto& p: *mapC)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z;
        merged->push_back(point);
    }
    /* D*/
    th = 3*PI/2;
    x0 = 8.17263 - 8.18781;
    y0 = 7.53936;
    for(auto& p: *mapD)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z;
        merged->push_back(point);
    }
 

    auto outmap = merged; 
    // Save filtered point cloud to new .pcd file
    // std::string outpath = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/table/merge.pcd";
    // pcl::io::savePCDFileASCII(outpath, *outmap);

    cout <<"size of merged map: "<< outmap->size()<<endl;
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
