#include "ros/ros.h"
#include "yeti_snowplow/joystick.h"
#include "sensor_msgs/Joy.h"

// Xbox 360 wireless uses WIRED indexes http://wiki.ros.org/joy
#define BUTTON_A joy->buttons[0]
#define BUTTON_B joy->buttons[1]
#define BUTTON_X joy->buttons[2]
#define BUTTON_Y joy->buttons[3]
#define BUTTON_LB joy->buttons[4]
#define BUTTON_RB joy->buttons[5]
#define BUTTON_BACK joy->buttons[6]
#define BUTTON_START joy->buttons[7]
#define BUTTON_GUIDE joy->buttons[8]
#define BUTTON_LS joy->buttons[9]
#define BUTTON_RS joy->buttons[10]

#define AXIS_L_LR joy->axes[0]
#define AXIS_L_UD joy->axes[1]
#define AXIS_LT joy->axes[2]
#define AXIS_R_LR joy->axes[3]
#define AXIS_R_UD joy->axes[4]
#define AXIS_RT joy->axes[5]
#define AXIS_DPAD_LR joy->axes[6]
#define AXIS_DPAD_UD joy->axes[7]

ros::Publisher joystick_pub;

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */
	
	yeti_snowplow::joystick msg;
	msg.A = BUTTON_A;
	msg.B = BUTTON_B;
	msg.X = BUTTON_X;
	msg.Y = BUTTON_Y;
	// ROS_INFO("%s", strs.str().c_str());
	joystick_pub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "joystick");

	ros::NodeHandle n;

	joystick_pub = n.advertise<yeti_snowplow::joystick>("joystick", 1000);
	ros::Subscriber sub = n.subscribe("joy", 1000, joystickCallback);

	ros::spin();
	
	return 0;
}