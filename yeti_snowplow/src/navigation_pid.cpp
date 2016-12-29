#include "ros/ros.h"
#include "yeti_snowplow/waypoint.h"

#include <string>
#include <vector>
using namespace std;

#include "containers/Target.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "navigation_pid");

	ros::NodeHandle n;

	ros::ServiceClient waypointClient = n.serviceClient<yeti_snowplow::waypoint>("waypoint");
    yeti_snowplow::waypoint waypointSrv;
    for (int i=0; i < 13; i++){        
        waypointSrv.request.ID = i;
        ROS_INFO("Sent request: %i", waypointSrv.request.ID);
        while (waypointClient.call(waypointSrv) == false){
            ROS_ERROR("Failed to call the waypoint service");
            ROS_INFO("Sent request: %i", waypointSrv.request.ID);
        }

        ROS_INFO("Received response: x=%f y=%f heading=%f dir=%i PID=%s speed=%f", waypointSrv.response.x, waypointSrv.response.y, waypointSrv.response.heading, waypointSrv.response.dir, waypointSrv.response.PID?"true":"false", waypointSrv.response.speed);
    }
    //ros::spin();
	
	return 0;
}