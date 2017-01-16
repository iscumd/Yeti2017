#include "ros/ros.h"
#include "yeti_snowplow/obstacles.h"
#include "yeti_snowplow/robot_position.h"
#include "yeti_snowplow/turn.h"

ros::Publisher turnPub;

void obstaclePositionsCallback(const yeti_snowplow::obstacles::ConstPtr& obstaclePositions){
	// geometry_msgs::Twist msg;
	// msg.angular.z = joy->A ? joy->LeftStick_UD : 0; //left
	// msg.linear.x = joy->A ? joy->LeftStick_UD : 0; //right
	// ROS_INFO("%s left=%f right=%f", joy->A ? "on" : "off", msg.angular.z, msg.linear.x);
	// turnPub.publish(msg);
}

void robotPositionCallback(const yeti_snowplow::robot_position::ConstPtr& robotPosition){
	// geometry_msgs::Twist msg;
	// msg.angular.z = joy->A ? joy->LeftStick_UD : 0; //left
	// msg.linear.x = joy->A ? joy->LeftStick_UD : 0; //right
	// ROS_INFO("%s left=%f right=%f", joy->A ? "on" : "off", msg.angular.z, msg.linear.x);
	// turnPub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "obstacle_reaction");

	ros::NodeHandle n;

	turnPub = n.advertise<yeti_snowplow::turn>("obstacle_reaction_turn", 1000);

	ros::Subscriber obstaclePositionsSub = n.subscribe("obstacles", 1000, obstaclePositionsCallback);
    ros::Subscriber robotPositionSub = n.subscribe("robot_position", 1000, robotPositionCallback);

	ros::spin();
	
	return 0;
}