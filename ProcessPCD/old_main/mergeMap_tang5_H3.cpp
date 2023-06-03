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
    std::string pathMapA = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapA.pcd";
    std::string pathMapB = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapB.pcd";
    std::string pathMapC = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapC.pcd";
    std::string pathMapD= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapD.pcd";
    std::string pathMapE= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapE.pcd";
    std::string pathMapF= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapF.pcd";
    std::string pathMapG= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapG.pcd";
    std::string pathMapH= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapH.pcd";
    std::string pathMapI= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapI.pcd";
    std::string pathMapJ= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapJ.pcd";
    std::string pathMapK= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapK.pcd";
    std::string pathMapL= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapL.pcd";
    std::string pathMapM= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapM.pcd";
    std::string pathMapN= "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/mapN.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr mapA(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapB(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapD(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapE(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapF(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapG(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapH(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapI(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapJ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapK(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapL(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapM(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapN(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapA, *mapA);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapB, *mapB);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapC, *mapC);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapD, *mapD);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapE, *mapE);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapF, *mapF);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapG, *mapG);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapH, *mapH);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapI, *mapI);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapJ, *mapJ);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapK, *mapK);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapL, *mapL);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapM, *mapM);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pathMapN, *mapN);
    // old : 3.0, 3.2, 7x0.002

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
    th = 0;
    x0 = 10;
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
    th = 0;
    x0 = 20;
    y0 = 0;
    for(auto& p: *mapC)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z;
        merged->push_back(point);
    }
    /* D*/
    th = -PI/2;
    x0 = 20+0.838;
    y0 = 0.6;
    for(auto& p: *mapD)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z;
        merged->push_back(point);
    }
    /* E*/
    th = -PI;
    x0 = 20+0.838;
    y0 = 0.6-12-0.47;
    z0 = -0.2;
    for(auto& p: *mapE)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    /* F*/
    th = -PI;
    x0 = 20+0.838-10;
    y0 = 0.6-12-0.47;
    z0 = -0.2;
    for(auto& p: *mapF)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    /* G*/
    th = -PI;
    x0 = 20+0.838-10-10;
    y0 = 0.6-12-0.47;
    z0 = -0.2;
    for(auto& p: *mapG)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    /* H*/
    th = -PI;
    x0 = 20+0.838-10-10-10;
    y0 = 0.6-12-0.47;
    z0 = -0.2;
    for(auto& p: *mapH)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    /* I*/
    th = -PI;
    x0 = 20+0.838-10*4;
    y0 = 0.6-12-0.47;
    z0 = -0.2;
    for(auto& p: *mapI)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    /* J*/
    th = -PI;
    x0 = 20+0.838-10*5;
    y0 = 0.6-12-0.47;
    z0 = -0.2;
    for(auto& p: *mapJ)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    /* K*/
    th = -3*PI/2;
    x0 = 20+0.838-10*3+2*0.6+0.185;
    y0 = 0.6-12-0.47+0.895+0.6*2;
    z0 = 0;
    for(auto& p: *mapK)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    /* L*/
    th = PI;
    x0 = 0;
    y0 = 0;
    z0 = 0;
    for(auto& p: *mapL)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    /* M*/
    th = PI;
    x0 = -10;
    y0 = 0;
    z0 = 0;
    for(auto& p: *mapM)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    /* N*/
    th = PI;
    x0 = -10*2;
    y0 = 0;
    z0 = 0;
    for(auto& p: *mapN)
    {
        pcl::PointXYZ point;
        point.x = p.x*cos(th) - p.y*sin(th)+x0;
        point.y = p.x*sin(th) + p.y*cos(th)+y0;
        point.z = p.z+z0;
        merged->push_back(point);
    }
    // /*E*/
    // th = PI/2;
    // x0 = (25*0.4+25*0.002)*2 + 5.5*0.4+5*0.002;
    // y0 = 0;
    // for(auto& p: *mapE)
    // {
    //     pcl::PointXYZ point;
    //     point.x = p.x*cos(th) - p.y*sin(th)+x0;
    //     point.y = p.x*sin(th) + p.y*cos(th)+y0;
    //     point.z = p.z;
    //     merged->push_back(point);
    // }

    auto outmap = merged; 
    // Save filtered point cloud to new .pcd file
    std::string outpath = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_28_05/pcd/tang5_H3/merge.pcd";
    pcl::io::savePCDFileASCII(outpath, *outmap);

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
