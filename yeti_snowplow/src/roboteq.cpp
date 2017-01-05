#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "yeti_snowplow/joystick.h"

ros::Publisher roboteqPub;

void joystickCallback(const yeti_snowplow::joystick::ConstPtr& joy){	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */
	
	geometry_msgs::Twist msg;
	msg.angular.z = joy->A ? joy->LeftStick_UD : 0; //left
	msg.linear.x = joy->A ? joy->LeftStick_UD : 0; //right
	ROS_INFO("%s left=%f right=%f", joy->A ? "on" : "off", msg.angular.z, msg.linear.x);
	roboteqPub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "roboteq");

	ros::NodeHandle n;

	roboteqPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::Subscriber joystickSub = n.subscribe("joystick", 1000, joystickCallback);

	ros::spin();
	
	return 0;
}