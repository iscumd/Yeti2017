#include "ros/ros.h"
#include "yeti_snowplow/joystick.h"

#include <sstream>

void joystickCallback(const yeti_snowplow::joystick::ConstPtr& msg){
	std::stringstream ss;

	if(msg->A){ ss << " A"; }
	if(msg->B){ ss << " B"; }
	if(msg->X){ ss << " X"; }
	if(msg->Y){ ss << " Y"; }
	ROS_INFO("The joystick says:%s", ss.str().c_str());
}

int main(int argc, char **argv){
	ros::init(argc, argv, "drive_mode_control");

	ros::NodeHandle n;

	ros::Subscriber joystickSub = n.subscribe("joystick", 1000, joystickCallback);

	ros::spin();
	
	return 0;
}