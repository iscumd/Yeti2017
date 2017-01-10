#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pcl_pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
laser_geometry::LaserProjection projector_;

void scan_to_XY (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 cloud;

  //convert scan data into point cloud
  projector_.projectLaser(*scan_in, cloud);

 //publish point cloud
  pcl_pub.publish(cloud);  

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "scan_to_xy");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("Converting /scan to XY point cloud");

  // Create a ROS subscriber for the input laser scan
  ros::Subscriber scanSub;
  scanSub = nh.subscribe("scan", 1, scan_to_XY);

  // Create a ROS publisher for the output point cloud
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

  // Spin
  ros::spin ();
}