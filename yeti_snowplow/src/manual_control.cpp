#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "yeti_snowplow/joystick.h"

#include <string>

ros::Publisher manualPub;

void joystickCallback(const yeti_snowplow::joystick::ConstPtr& joy){	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */

	float joySpeed = 0.0, joyTurn = 0.0;

	joySpeed = joy->LeftStick_UD;
	joyTurn = joy->LeftStick_LR;
	
	geometry_msgs::Twist msg;
	msg.linear.x = joy->RB ? joySpeed : 0;
	msg.angular.z = joy->RB ? joyTurn : 0;
	manualPub.publish(msg);

	ROS_INFO("Manual Control: %s linear.x=%f angular.z=%f", joy->RB ? "on" : "off", msg.linear.x, msg.angular.z);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "manual_control");

	ros::NodeHandle n;

	manualPub = n.advertise<geometry_msgs::Twist>("manualControl", 5);

	ros::Subscriber joystickSub = n.subscribe("joystick", 5, joystickCallback);

	ros::spin();
	
	return 0;
}