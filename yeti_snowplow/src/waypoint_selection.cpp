#include "ros/ros.h"
#include "yeti_snowplow/waypoint.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
using namespace std;

#include "containers/Target.h"

vector<Target> targetLocationList;

//because C++ doesn't have built in string splitting http://stackoverflow.com/a/236803
void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


vector<Target> ReadFile(string filename){
	string str;
	ifstream file;
	vector<Target> navigationPoints = vector<Target>();
	int pointCount = 0;

	file.open(filename.c_str());
	while(getline(file, str)){
		ROS_DEBUG("Read file line: %s", str.c_str());
		vector<string> lineFields = split(str, ' '); //x y direction PID speed
		if(lineFields.size() == 5){ //ignore if too short or starts with "// "
			Target currentLidarPoint = Target(atof(lineFields[0].c_str()), atof(lineFields[1].c_str()), 0.0, atoi(lineFields[2].c_str()), (atoi(lineFields[3].c_str()) > 0), atof(lineFields[4].c_str()));
			currentLidarPoint.location.id = pointCount;
			pointCount++;

			navigationPoints.push_back(currentLidarPoint);
		}
	}
	file.close();

	return navigationPoints;
}

bool waypoint(yeti_snowplow::waypoint::Request  &req,
              yeti_snowplow::waypoint::Response &res){
	ROS_INFO("Request: %i", req.ID);
	if (req.ID > -1 && req.ID < targetLocationList.size()){
		res.x = targetLocationList[req.ID].location.x;
		res.y = targetLocationList[req.ID].location.y;
		res.heading = targetLocationList[req.ID].location.heading;
		res.dir = targetLocationList[req.ID].dir;
		res.PID = targetLocationList[req.ID].PID;
		res.speed = targetLocationList[req.ID].speed;
		ROS_INFO("Response: x=%f y=%f heading=%f dir=%i PID=%s speed=%f", res.x, res.y, res.heading, res.dir, res.PID?"true":"false", res.speed);
		return true;
	}
	else {
		return false;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "waypoint_selection");

	ros::NodeHandle n;
	
	std::string navigationFile;
	if (ros::param::get("navigationFile", navigationFile)){
		ROS_INFO("Using navigationFile %s", navigationFile.c_str());

		targetLocationList = ReadFile(navigationFile);

		for(int i = 0; i < targetLocationList.size(); i++){
			targetLocationList[i].Print();
		}

		ros::ServiceServer service = n.advertiseService("waypoint", waypoint);

		ros::spin();
	}
	else{
		ROS_FATAL("No navigationFile parameter specified!");
	}
	
	return 0;
}