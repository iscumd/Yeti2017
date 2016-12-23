#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include <sstream>

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

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

ros::Publisher chatter_pub;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& joy)
{	
  /* This fires every time a button is pressed/released
  and when an axis changes (even if it doesn't leave the
  deadzone). */

  std_msgs::String msg;

  std::stringstream ss;

	if(BUTTON_A){ ss << " A";	}
  if(BUTTON_B){ ss << " B";	}
  if(BUTTON_X){ ss << " X";	}
  if(BUTTON_Y){ ss << " Y";	}
  if(BUTTON_LB){ ss << " LB";	}
  if(BUTTON_RB){ ss << " RB";	}
  if(BUTTON_BACK){ ss << " BACK";	}
  if(BUTTON_START){ ss << " START";	}
  if(BUTTON_GUIDE){ ss << " GUIDE";	}
  if(BUTTON_LS){ ss << " LS";	}
  if(BUTTON_RS){ ss << " RS";	}

  if(AXIS_L_LR != 0 || AXIS_L_UD != 0){ ss << " LSTICK: " << AXIS_L_LR << " " << AXIS_L_UD;	}
  if(AXIS_R_LR != 0 || AXIS_R_UD != 0){ ss << " RSTICK: " << AXIS_R_LR << " " << AXIS_R_UD;	}
  if(AXIS_LT < 1 || AXIS_RT < 1){ ss << " TRIGGER: " << AXIS_LT << " " << AXIS_RT;	}
  if(AXIS_DPAD_LR != 0 || AXIS_DPAD_UD != 0){ ss << " DPAD: " << AXIS_DPAD_LR << " " << AXIS_DPAD_UD;	}

  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());

  chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;


  chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = n.subscribe("joy", 1000, chatterCallback);

  ros::spin();

  return 0;
}
