#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>


std::string getCurrentDateTime()
{
    // Get current time
    auto now = std::chrono::system_clock::now();

    // Convert to local time
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    struct tm local_tm = *localtime(&t);

    // Format the date and time as a string
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << local_tm.tm_mday << "_"
        << std::setfill('0') << std::setw(2) << (local_tm.tm_mon + 1) << "_"
        << std::setfill('0') << std::setw(2) << local_tm.tm_hour << "_"
        << std::setfill('0') << std::setw(2) << local_tm.tm_min << "_"
        << std::setfill('0') << std::setw(2) << local_tm.tm_sec << "_"; 
    return oss.str();
}

float calcLenMica(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    float len = 0.0;

    float thickMica = -0.3 ;
    float lenX = 1.5;
    float lenZ = 0.3;
    float pointMica[] = {2.0, -0.5, 0.2};

    float offset = 0.004;
    // assum
    float minX = 3.0 ; 
    float maxX = 2.6 ;
    // true length is 15 cm
    for(auto& p: *cloud)
    {
       // length in x axis
        if( p.x >= pointMica[0] && p.x <= pointMica[0]+lenX &&
            p.y <= pointMica[1] && p.y >= pointMica[1] + thickMica &&
            p.z >= pointMica[2] && p.z <= pointMica[2]+lenZ)
        {
            if(p.x > maxX) maxX = p.x;
            if(p.x < minX) minX = p.x;
        }

    }
    std::cout<< minX <<" "<<maxX << std::endl;
    len = maxX - minX;
    return len;
}
float calcDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    int count = 0;
    float thick = -0.3 ;
    float len = 0.3;
    float point[] = {2.5, -0.5, 0.2};
    for(auto& p: *cloud)
    {
       if(p.x >= point[0] && p.x <= point[0]+len &&
         p.y <= point[1] && p.y >= point[1]+thick &&
         p.z >= point[2] && p.z <= point[2]+len )
        {
            count++;
        }
    }
    return (float)100*count/90;
}

int main()
{
    std::string dateTime = getCurrentDateTime();
    std::string fileName = "test_simu_1_5_cutX.pcd";
    std::string anlyzeName = "test_simu_1_5_cutX.txt";
    // dir
    std::string analyzed_dir = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/output/analyzeDataSimu/";
    std::string dir = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/output/";
    
    // input pcd
    std::string path = "/home/thuan12ha/_3Drobot_ws/ProcessPCD/_data_simulation/test_simu_1_5/pcd/test_simu_1_5_cutX.pcd";
    // ouput pcd
    std::string outpath = dir + dateTime + fileName;
    // out analyze txt
    std::string outAnalyzePath = analyzed_dir + dateTime +anlyzeName ;
    std::ofstream fw(outAnalyzePath, std::ofstream::out);

    // initial analyzed data
    float Raw[] = {0.0,0.0,0.0};
    float PassThrough[] = {0.0,0.0,0.0};
    float Voxel[] = {0.0, 0.0 ,0.0};
    float Outlier[] = {0.0,0.0,0.0};
    float MLS[] = {0.0,0.0,0.0};

    // Load the point cloud map from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *map) == -1)
    {
        PCL_ERROR("Couldn't read file map.pcd\n");
        return (-1);
    }
    // Before process
    std::cout << "Size of map before processing: " << map->size() << std::endl;
    cloud = map ;
    Raw[0] = map->size();
    Raw[1] = (float)calcDensity(cloud);
    Raw[2] = calcLenMica(cloud);

    // Apply a pass-through filter to remove points outside a specified range 
        //Z
    pcl::PassThrough<pcl::PointXYZ> pass_filter;

    pass_filter.setInputCloud(map);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(-5.0f, 10.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_filteredZ(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*map_filteredZ);

        //Y
    pass_filter.setInputCloud(map_filteredZ);
    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(-20.0f, 20.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_filteredY(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*map_filteredY);    
    std::cout << "Size of map after pass-through filter : " <<  map_filteredY->size() << std::endl;

    cloud = map_filteredY;
    PassThrough[0] = cloud->size();
    PassThrough[1] = (float)calcDensity(cloud);
    PassThrough[2] = calcLenMica(cloud);

    // // Downsample the point cloud using a voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(map_filteredY);
    voxel_grid_filter.setLeafSize(0.02f, 0.02f, 0.02f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid_filter.filter(*map_downsampled);
    std::cout << "Size of map after voxel filter : " <<  map_downsampled->size() << std::endl;

    cloud = map_downsampled;
    Voxel[0] = cloud->size();
    Voxel[1] = (float)calcDensity(cloud);
    Voxel[2] = calcLenMica(cloud);

    // Apply statistical_outlier_removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(map_downsampled);
    sor.setMeanK(5);
    sor.setStddevMulThresh(2.0);
    sor.filter(*map_downsampled);
    std::cout << "Size of map after statistical_outlier_removal : " <<  map_downsampled->size() << std::endl;

    // cloud = map_downsampled;
    Outlier[0] = cloud->size();
    Outlier[1] = (float)calcDensity(cloud);
    Outlier[2] = calcLenMica(cloud);
    // Compute normals for the filtered point cloud
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimator.setSearchMethod(search_tree);
    normal_estimator.setInputCloud(map_downsampled);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimator.setRadiusSearch(0.1f);
    normal_estimator.compute(*normals);

    // Apply an MLS filter to smooth the point cloud
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls_filter;
    mls_filter.setInputCloud(map_downsampled);
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
    std::cout << "Size of map after MLS : " << map_smoothed->size() << std::endl;

    cloud = map_smoothed;
    MLS[0] = cloud->size();
    MLS[1] = (float)calcDensity(cloud);
    MLS[2] = calcLenMica(cloud);

    auto outmap = map_smoothed;

    // Save filtered point cloud to new .pcd file
    // pcl::io::savePCDFileASCII(outpath, *outmap);
    // std::cout << "Saved filtered point cloud to " << filtered_filename << std::endl;

    // Export analyzed data
    if (fw.is_open())
    {
        fw  <<"Raw(sum,density,delta)\n"
            << round(Raw[0]*100)/100 << "\t" << round(Raw[1]*100)/100 << "\t"<< round(Raw[2]*100)/100 << "\n"
            << "PassThrough(sum,density,delta)\n"
            << round(PassThrough[0]*100)/100 << "\t" << round(PassThrough[1]*100)/100 << "\t"<< round(PassThrough[2]*100)/100 << "\n"
            << "Voxel(sum,density,delta)\n"
            << round(Voxel[0]*100)/100 << "\t" << round(Voxel[1]*100)/100 << "\t"<< round(Voxel[2]*100)/100 << "\n"
            << "Remove outlier(sum,density,delta)\n"
            << round(Outlier[0]*100)/100 << "\t" << round(Outlier[1]*100)/100 << "\t"<< round(Outlier[2]*100)/100 << "\n"
            << "MLS(sum,density,delta)\n"
            << round(MLS[0]*100)/100 << "\t" << round(MLS[1]*100)/100 << "\t"<< round(MLS[2]*100)/100 << "\n";
    }
    fw.close();
    //View cloud 
    pcl::visualization::PCLVisualizer viewer("Map Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(outmap, 255, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(outmap, color_handler,"upsampling_map");

    // View mica line to check space to analyze
    /*
    12 edge
    000 - 100,100-110,110-010,010-000
    001 - 101,101-111,111-011,011-001
    000 - 001,100-101,110-111,010-011
    */
    // float thick = -0.3 ;
    // float len = 0.3;
    // float point[] = {2.5, -0.5, 0.2};
    // pcl::PointXYZ wall000(point[0], point[1], point[2]);
    // pcl::PointXYZ wall100(point[0]+len,point[1], point[2]);
    // pcl::PointXYZ wall110(point[0]+len,point[1]+thick, point[2]);
    // pcl::PointXYZ wall101(point[0]+len,point[1], point[2]+len);
    // pcl::PointXYZ wall111(point[0]+len,point[1]+thick, point[2]+len);
    // pcl::PointXYZ wall010(point[0],point[1]+thick, point[2]);
    // pcl::PointXYZ wall011(point[0],point[1]+thick, point[2]+len);
    // pcl::PointXYZ wall001(point[0],point[1], point[2]+len);

    // viewer.addLine(wall000,wall100,"line1");
    // viewer.addLine(wall001,wall101,"line2");
    // viewer.addLine(wall000,wall001,"line3");
    // viewer.addLine(wall100,wall110,"line4");
    // viewer.addLine(wall101,wall111,"line5");
    // viewer.addLine(wall100,wall101,"line6");
    // viewer.addLine(wall110,wall010,"line7");
    // viewer.addLine(wall111,wall011,"line8");
    // viewer.addLine(wall110,wall111,"line9");
    // viewer.addLine(wall010,wall000,"line10");
    // viewer.addLine(wall011,wall001,"line11");
    // viewer.addLine(wall010,wall011,"line12");

    // // Set the color of the line to red
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line1");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line2");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line3");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line4");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line5");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line6");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line7");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line8");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line9");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line10");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line11");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line12");
    // //View line on mica to check space to analyze
    // /*
    // 12 edge
    // 000 - 100,100-110,110-010,010-000
    // 001 - 101,101-111,111-011,011-001
    // 000 - 001,100-101,110-111,010-011
    // */
    // float thickMica = -0.3 ;
    // float lenX = 1.5;
    // float lenZ = 0.3;
    // float pointMica[] = {2.0, -0.5, 0.2};
    // pcl::PointXYZ mica000(pointMica[0], pointMica[1], pointMica[2]);
    // pcl::PointXYZ mica100(pointMica[0]+lenX,pointMica[1], pointMica[2]);
    // pcl::PointXYZ mica110(pointMica[0]+lenX,pointMica[1]+thickMica, pointMica[2]);
    // pcl::PointXYZ mica101(pointMica[0]+lenX,pointMica[1], pointMica[2]+lenZ);
    // pcl::PointXYZ mica111(pointMica[0]+lenX,pointMica[1]+thickMica, pointMica[2]+lenZ);
    // pcl::PointXYZ mica010(pointMica[0],pointMica[1]+thickMica, pointMica[2]);
    // pcl::PointXYZ mica011(pointMica[0],pointMica[1]+thickMica, pointMica[2]+lenZ);
    // pcl::PointXYZ mica001(pointMica[0],pointMica[1], pointMica[2]+lenZ);

    // viewer.addLine(mica000,mica100,"lineMica1");
    // viewer.addLine(mica001,mica101,"lineMica2");
    // viewer.addLine(mica000,mica001,"lineMica3");
    // viewer.addLine(mica100,mica110,"lineMica4");
    // viewer.addLine(mica101,mica111,"lineMica5");
    // viewer.addLine(mica100,mica101,"lineMica6");
    // viewer.addLine(mica110,mica010,"lineMica7");
    // viewer.addLine(mica111,mica011,"lineMica8");
    // viewer.addLine(mica110,mica111,"lineMica9");
    // viewer.addLine(mica010,mica000,"lineMica10");
    // viewer.addLine(mica011,mica001,"lineMica11");
    // viewer.addLine(mica010,mica011,"lineMica12");

    // // Set the color of the line to red
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica1");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica2");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica3");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica4");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica5");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica6");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica7");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica8");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica9");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica10");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica11");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "lineMica12");

    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    return 0;
}
