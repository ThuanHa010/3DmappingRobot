#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

std::ofstream fw("/home/thuan12ha/_3Drobot_ws/ProcessPCD/output_txtNpcd/out_simu_1_5.pcd", std::ofstream::out);
    // if (cloud->points[i].x>7.8 && cloud->points[i].x<9.2 
    //     && cloud->points[i].y < -0.7 && cloud->points[i].y > -0.9 )
typedef struct {
  float xMin;
  float xMax;
  float yMin;
  float yMax;
  float zMax;
  float zMin;
}edge;

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  /*Read from raw data*/
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/thuan12ha/_3Drobot_ws/ProcessPCD/input_simu/test_simu_1_5.pcd", *cloud);
  edge myEdge;
  myEdge.xMax = -0.7;
  myEdge.xMin = -1.6;
  myEdge.yMax = 4.3;
  myEdge.yMin = 4.15;
  myEdge.zMax = 0.9;
  myEdge.zMin = 0.05;

  if (fw.is_open())
  {
      fw <<"# .PCD v0.7 - Point Cloud Data file format\n"
        <<"VERSION 0.7\n"
        <<"FIELDS x y z intensity\n"
        <<"SIZE 4 4 4 4\n"
        <<"TYPE F F F F\n"
        <<"WIDTH X\n"
        <<"HEIGHT 1\n"
        <<"VIEWPOINT 0 0 0 1 0 0 0\n"
        <<"POINTS X\n"
        <<"DATA ascii\n";
  }
  else std::cout<<"Cant open file txt"<<std::endl;

  int count = 0;
  for (int i = 0; i < (*cloud).size(); i++)
  {
    float X = cloud->points[i].x; 
    float Y = cloud->points[i].y;
    float Z = cloud->points[i].z;
    if (cloud->points[i].x>-1.7 && cloud->points[i].x<-0.4  
      && cloud->points[i].y>4.1  && cloud->points[i].y<4.4
      && cloud->points[i].z>0.04 && cloud->points[i].z<2)
    {
      if(X > myEdge.xMax) myEdge.xMax = X;
      if(X < myEdge.xMin) myEdge.xMin = X;
      if(Y > myEdge.yMax) myEdge.yMax = Y;
      if(Y < myEdge.yMin) myEdge.yMin = Y;
      if(Z > myEdge.zMax) myEdge.zMax = Z;
      if(Z < myEdge.zMin) myEdge.zMin = Z;
      if (fw.is_open())
      {
        //store array contents to text file
        fw << cloud->points[i].x <<" "<< cloud->points[i].y<<" "<<cloud->points[i].z<<" 47"<<"\n";
        count++;
        std::cout<<"SAVED : "<<count<<std::endl;
        //if(end > 200000) fw.close();
      }
      else std::cout<<"Cant open file pcd"<<std::endl;
                  
    }

  }
  fw.close();
  std::cout<<"X: "<<myEdge.xMin<<" -> "<<myEdge.xMax<<"\t"
            <<"Y: "<<myEdge.yMin<<" -> "<<myEdge.yMax<<"\t"
            <<"Z: "<<myEdge.zMin<<" -> "<<myEdge.zMax<<"\n";

  //pcl::io::savePCDFileASCII ("/home/thuan12ha/_3Drobot_ws/ProcessPCD/output/out_18_04.pcd", *cloud_cut);
  return 0;
}
