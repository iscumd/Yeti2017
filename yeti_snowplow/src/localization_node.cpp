#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud.h>
#include "yeti_snowplow/robot_position.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

ros::Publisher pub;//ROS publisher

sensor_msgs::PointCloud landmarkLocationsTxt;//holds landmark locations from text file
yeti_snowplow::robot_position prev_robot_location;//holds previous robot location

//find the landmark locations, with respect to where Yeti was. 
void scanLandmarks(sensor_msgs::PointCloud landmarkLocsTXT, sensor_msgs::PointCloud* scan_pt_cloud, yeti_snowplow::robot_position prev_robot_location)
{
	for( int i=0; i < scan_pt_cloud->points.size(); i++)
	{
		//loop through points
	}
}

//Used to parse strings. because C++ doesn't have built in string splitting http://stackoverflow.com/a/236803
void split(const std::string &s, char delim, std::vector<std::string> &elems) 
{
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

std::vector<std::string> split(const std::string &s, char delim) 
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

//gater landmarks from text file. Text file is in ROS PARAM.
sensor_msgs::PointCloud importLandMarks(string filename)
{
	//setup necassry string parsing and file parameters
	string str;
	ifstream file;
	file.open(filename.c_str());
	ROS_INFO("Read file line: %s", str.c_str());

	int numLandmarks = 0;
	while(getline(file, str))//check to see how many landmarks there are
	{
		numLandmarks++;
	}
	ROS_INFO("There are %d landmarks. They are: ", numLandmarks);
	file.close();

	sensor_msgs::PointCloud importedLandmarkLocations;//allocate space for landmark points
	int landmarkNum=0;// initialize iterator
	file.open(filename.c_str());//reopen file

	while(getline(file, str))//loop through file and save landmark locations. 
	{
		//ROS_INFO("Read file line: %s", str.c_str());
		vector<string> lineFields = split(str, ' '); //x y direction PID speed
		//ROS_INFO("Line %d has %ld many fields.", landmarkNum, lineFields.size());
		
		if(lineFields.size() == 2) //ignore if too short or starts with "// "
		{ 
			geometry_msgs::Point32 landmarkPoint;
			landmarkPoint.x=atof(lineFields[0].c_str());
			landmarkPoint.y=atof(lineFields[1].c_str());
			// importedLandmarkLocations.points[landmarkNum].x = atof(lineFields[0].c_str());
			// importedLandmarkLocations.points[landmarkNum].y = atof(lineFields[1].c_str());
			ROS_INFO("Landmark %d: \tX: %f\tY:%f",landmarkNum, landmarkPoint.x, landmarkPoint.y);
			landmarkNum++;

			importedLandmarkLocations.points.push_back(landmarkPoint);
		}
	}
	file.close();

	return landmarkLocationsTxt;
}


//finds landmarks, finds robot locations from landmark location, then publishes robot location
void localizeCallBack (const sensor_msgs::PointCloud::ConstPtr& cloud_in)
{
		
		int i = 0;
		//ROS_INFO_STREAM(cloud_in->points[i]);
		for(i=0 ; i <  cloud_in->channels.size(); i++)
		{
			ROS_INFO_STREAM("size of cloud is " << cloud_in->points.size());
			//ROS_INFO_STREAM(cloud_in->channels[i].name);
			ROS_INFO_STREAM("Point ["<<i<< "] - X: " << cloud_in->points[i].x << "\t Y: " << cloud_in->points[i].x);
		}
}


main (int argc, char** argv)
{
	ros::init (argc, argv, "localization");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Finding Landmarks, and Robot Position");

	//import landmark locations from text file
	std::string landmarkLocationFile;
	if (ros::param::get("landmarkLocationFile", landmarkLocationFile))
	{
		ROS_INFO("Using landmarkLocationFile %s", landmarkLocationFile.c_str());
	}
	landmarkLocationsTxt = importLandMarks(landmarkLocationFile);
	
	// Create a ROS subscriber for the pointcloud published by laser geometry
 	ros::Subscriber scanSub;
 	scanSub = nh.subscribe("laser_scan_point_cloud", 1, localizeCallBack);

 	 // Create a ROS publisher for the output point cloud
 	 pub = nh.advertise<sensor_msgs::PointCloud>("robot_location", 1);

	ros::spin();
return 0;
}






