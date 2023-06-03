#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>


int main()
{
    std::string path = "/home/thuan12ha/_3Drobot_ws/_data_jetson/test_7_5/pcd/main_7_5_obj_L.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *map) == -1)
    {
        PCL_ERROR("Couldn't read file map.pcd\n");
        return (-1);
    }
    float thickMica = -0.15 ;
    float lenX = 0.5;
    float lenZ = 0.3;
    float pointMica[] = {3.8, 1.1, 0};

    float ymax = 0;
    float ymin = 1.1;
    pcl::PointXYZ pointAtmax;
    pcl::PointXYZ pointAtmin;
    for(auto& p: *map)
    {
       // length in x axis
        if( p.x >= pointMica[0] && p.x <= pointMica[0]+lenX &&
            p.y <= pointMica[1] && p.y >= pointMica[1] + thickMica &&
            p.z >= pointMica[2] && p.z <= pointMica[2]+lenZ)
        {
            if(p.y > ymax) 
            {
                ymax = p.y;
                pointAtmax.x = p.x;
                pointAtmax.y = p.y;
                pointAtmax.z = p.z;
            }
            if(p.y < ymin) 
            {
                ymin = p.y;
                pointAtmin.x = p.x;
                pointAtmin.y = p.y;
                pointAtmin.z = p.z;

            }
        }
    }
    std::cout <<"Min y: "<< ymin<<"Max y:"<<ymax<<std::endl;
    std::cout <<"Min point: "<<pointAtmin<<"Max point"<<pointAtmax<<std::endl;
    auto outmap = map;

    //View cloud 
    pcl::visualization::PCLVisualizer viewer("Map Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(outmap, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(outmap, color_handler,"upsampling_map");

    // View mica line to check space to analyze
    /*
    12 edge
    000 - 100,100-110,110-010,010-000
    001 - 101,101-111,111-011,011-001
    000 - 001,100-101,110-111,010-011
    */
    float thick = -0.15 ;
    float len = 1;
    float point[] = {1, -1.15, 0.2};
    pcl::PointXYZ wall000(point[0], point[1], point[2]);
    pcl::PointXYZ wall100(point[0]+len,point[1], point[2]);
    pcl::PointXYZ wall110(point[0]+len,point[1]+thick, point[2]);
    pcl::PointXYZ wall101(point[0]+len,point[1], point[2]+len);
    pcl::PointXYZ wall111(point[0]+len,point[1]+thick, point[2]+len);
    pcl::PointXYZ wall010(point[0],point[1]+thick, point[2]);
    pcl::PointXYZ wall011(point[0],point[1]+thick, point[2]+len);
    pcl::PointXYZ wall001(point[0],point[1], point[2]+len);

    viewer.addLine(wall000,wall100,"line1");
    viewer.addLine(wall001,wall101,"line2");
    viewer.addLine(wall000,wall001,"line3");
    viewer.addLine(wall100,wall110,"line4");
    viewer.addLine(wall101,wall111,"line5");
    viewer.addLine(wall100,wall101,"line6");
    viewer.addLine(wall110,wall010,"line7");
    viewer.addLine(wall111,wall011,"line8");
    viewer.addLine(wall110,wall111,"line9");
    viewer.addLine(wall010,wall000,"line10");
    viewer.addLine(wall011,wall001,"line11");
    viewer.addLine(wall010,wall011,"line12");

    // Set the color of the line to red
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line1");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line2");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line3");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line4");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line5");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line6");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line7");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line8");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line9");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line10");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line11");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line12");
    // View line on mica to check space to analyze
    /*
    12 edge
    000 - 100,100-110,110-010,010-000
    001 - 101,101-111,111-011,011-001
    000 - 001,100-101,110-111,010-011
    */

    pcl::PointXYZ mica000(pointMica[0], pointMica[1], pointMica[2]);
    pcl::PointXYZ mica100(pointMica[0]+lenX,pointMica[1], pointMica[2]);
    pcl::PointXYZ mica110(pointMica[0]+lenX,pointMica[1]+thickMica, pointMica[2]);
    pcl::PointXYZ mica101(pointMica[0]+lenX,pointMica[1], pointMica[2]+lenZ);
    pcl::PointXYZ mica111(pointMica[0]+lenX,pointMica[1]+thickMica, pointMica[2]+lenZ);
    pcl::PointXYZ mica010(pointMica[0],pointMica[1]+thickMica, pointMica[2]);
    pcl::PointXYZ mica011(pointMica[0],pointMica[1]+thickMica, pointMica[2]+lenZ);
    pcl::PointXYZ mica001(pointMica[0],pointMica[1], pointMica[2]+lenZ);

    viewer.addLine(mica000,mica100,"lineMica1");
    viewer.addLine(mica001,mica101,"lineMica2");
    viewer.addLine(mica000,mica001,"lineMica3");
    viewer.addLine(mica100,mica110,"lineMica4");
    viewer.addLine(mica101,mica111,"lineMica5");
    viewer.addLine(mica100,mica101,"lineMica6");
    viewer.addLine(mica110,mica010,"lineMica7");
    viewer.addLine(mica111,mica011,"lineMica8");
    viewer.addLine(mica110,mica111,"lineMica9");
    viewer.addLine(mica010,mica000,"lineMica10");
    viewer.addLine(mica011,mica001,"lineMica11");
    viewer.addLine(mica010,mica011,"lineMica12");

    // Set the color of the line to red
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica1");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica2");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica3");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica4");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica5");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica6");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica7");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica8");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica9");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica10");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica11");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica12");

    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    return 0;
}
