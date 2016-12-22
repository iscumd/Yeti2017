#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "waypoint_selection");

	ros::spin();
	
	return 0;
}