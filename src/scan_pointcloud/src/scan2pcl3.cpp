#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <fstream>


std::ofstream fw("/home/thuan12ha/_3Drobot_ws/src/scan_pointcloud/outFromScan2Pcl/output.txt", std::ofstream::out);

class PointCloudPublisher
{
    public:
        PointCloudPublisher() : nh_("~"), tf_listener_(nh_)
        {
            scan_sub_ = nh_.subscribe("/scan", 10, &PointCloudPublisher::scan_callback, this);
            //pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud2", 10);        
            start = 0;
            end = 0;
        }

        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {

            // Transform the point cloud to the "odom" frame
            tf::StampedTransform transform;
            try
            {
                tf_listener_.waitForTransform("odom", "rplidar_link", scan_msg->header.stamp, ros::Duration(0.1));
                tf_listener_.lookupTransform("odom", "rplidar_link", scan_msg->header.stamp, transform);
            }
            catch (tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                return;
            }

            pcl::PointCloud<pcl::PointXYZ> pcl_pc ;
            float angle_min = scan_msg->angle_min;
            float angle_increment = scan_msg->angle_increment;
            float range_max = scan_msg->range_max;
            int delta = 0;

            for (int i = 0; i < scan_msg->ranges.size(); ++i)
            {
                float angle = angle_min + angle_increment * i;
                float range = scan_msg->ranges[i];
                //float intens = scan_msg->intensities[i];

                if (std::isfinite(range))
                {
                    pcl::PointXYZ point;
                    point.x = range * std::cos(angle);
                    point.y = range * std::sin(angle);
                    point.z= 0.0;
                    //point.data[3] = intens;
                    pcl_pc.push_back(point);
                    end++;
                    delta++;
                }
            }
            ROS_INFO("Start: %d Delta: %d End: %d",start,delta,end);
            start += delta;
            // Transform the point cloud to the "odom" frame
            pcl::PointCloud<pcl::PointXYZ> pcl_transformed;
            pcl_ros::transformPointCloud(pcl_pc, pcl_transformed, transform);
            ROS_INFO("Size of pcl_transformed %d",pcl_transformed.points.size());
            //save to txt file
            if (fw.is_open())
            {
                //store array contents to text file
                for (int i = 0; i < delta; i++) {
                    fw << pcl_transformed.points[i].x <<" "<< pcl_transformed.points[i].y<<" "<<pcl_transformed.points[i].z<<" 47"<<"\n";
                }
                ROS_INFO("SAVED");
                //if(end > 200000) fw.close();
            }
            // Convert the point cloud to a PointCloud2 message
            // sensor_msgs::PointCloud2 pc2_msg;
            // pcl::toROSMsg(pcl_transformed, pc2_msg);
            // pc2_msg.header.frame_id = "odom";
            // pc2_msg.header.stamp = scan_msg->header.stamp;

            // // Publish the PointCloud2 message
            // pcl_pub_.publish(pc2_msg);
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        ros::Publisher pcl_pub_;
        tf::TransformListener tf_listener_;
        int start ;
        int end;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_pointcloud");
    PointCloudPublisher pc_publisher;
    ros::spin();
    return 0;
}
