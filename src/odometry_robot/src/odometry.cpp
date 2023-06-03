#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>

#define PI 3.1416
// Define global variables
float left_vel = 0.0;
float right_vel = 0.0;
float yaw = 0.0;
float D = 0.085;

void dataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // Store data from STM32 in global variables
  left_vel = (float)msg->data[2]*D*PI/60;
  right_vel = (float)msg->data[1]*D*PI/60;
  yaw = (float)msg->data[0]*PI/180;
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle nh;

  // Create publisher for odometry data
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  // Subscribe to data topic from STM32
  ros::Subscriber sub = nh.subscribe("read_from_stm32", 50, dataCallback);

  // Set up transform broadcaster
  tf::TransformBroadcaster broadcaster;

  // Initialize odom message
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  ros::Rate loop_rate(10);  // 10 Hz loop rate

  while (ros::ok())
  {
    // Calculate linear and angular velocities from left and right wheel velocities
    double linear_vel = (left_vel + right_vel) / 2.0;
    double angular_vel = (right_vel - left_vel) / D;  // 0.085 is the wheelbase of the vehicle

    // Calculate change in position using odometry model
    double delta_x = linear_vel * cos(yaw) * loop_rate.expectedCycleTime().toSec();
    double delta_y = linear_vel * sin(yaw) * loop_rate.expectedCycleTime().toSec();
    double delta_yaw = angular_vel * loop_rate.expectedCycleTime().toSec();

    // Update position estimate
    odom.pose.pose.position.x += delta_x;
    odom.pose.pose.position.y += delta_y;
    yaw += delta_yaw;

    // Set orientation quaternion
    tf::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();

    // Set linear and angular velocities
    odom.twist.twist.linear.x = linear_vel;
    odom.twist.twist.angular.z = angular_vel;

    // Set timestamps
    odom.header.stamp = ros::Time::now();
    //odom.twist.header.stamp = odom.header.stamp;

    // Publish the odom message
    odom_pub.publish(odom);

    // Broadcast transform from "odom" to "base_link"
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0));
    transform.setRotation(quat);
    broadcaster.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "odom", "base_link"));

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
