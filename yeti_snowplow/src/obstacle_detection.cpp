#include "ros/ros.h"
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <yeti_snowplow/robot_position.h>
#include <yeti_snowplow/obstacle.h>
#include <yeti_snowplow/obstacles.h>

using namespace std;


ros::Publisher obstaclePub;

void foo();


void localizationCallback(const yeti_snowplow::robot_position::ConstPtr& position) // Mike will update code that publishes a robot position callback here

{
    /* This fires every time a new robot position is published */
    foo();

    return;
}

void scanCallback(const sensor_msgs::PointCloud::ConstPtr& location)
{
    /* This fires every time a new obstacle scan is published */
    //location contains an array of points, which contains an x and a y relative to the robot

    for (int i = 0; i < location->points.size(); ++i)
    {
 //       location->points.x;
  //      location->points.x;
    }

    return;
}

void foo()
{
    yeti_snowplow::obstacles msg;


    yeti_snowplow::obstacles bar;// = new vector<yeti_snowplow::obstacle_position>();

    yeti_snowplow::obstacle temp;

    temp.x = 3;
    temp.y = 1;
    temp.r = 2;
    temp.dynamic = true;


    bar.obstacles.push_back(temp);

    obstaclePub.publish(msg);
}




int main(int argc, char **argv){
	ros::init(argc, argv, "obstacle_detection");

	ros::NodeHandle n;

    // input: robot position relative to field

    // input: array of lidar scan data

    // output: array of obstacles (x,y,r(size), moving(boolean))



    ros::Subscriber localizationSub = n.subscribe("robot_location", 1, localizationCallback);

    ros::Subscriber scanSub = n.subscribe("laser_scan_point_cloud", 1, scanCallback);

    ros::Publisher obstaclePub;//ROS obstacle publisher

    obstaclePub = n.advertise<yeti_snowplow::obstacles>("obstacles", 100);

	ros::spin();
	
	return 0;
}
