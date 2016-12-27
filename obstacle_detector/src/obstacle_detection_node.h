#ifndef OBSTACLE_DETECTION_NODE_H
#define OBSTACLE_DETECTION_NODE_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>

// Custom message includes. Auto-generated from msg/ directory.
#include "obstacle_detector/SegmentObstacle.h"
#include "obstacle_detector/Obstacles.h"

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

// Dynamic reconfigure includes.
// #include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
// #include <node_example/node_example_paramsConfig.h>

using std::string;

class ObstacleDetection
{
public:
    //! Constructor.
    ObstacleDetection();

    //! Destructor.
    ~ObstacleDetection();

private:
    //! Publish the message.
    void publishObstacles();
    
    //! Classify laser scan into obstacles
    // TODO This should be implemented in functional style
    void detectObstacles();

    //! Callback function for subscriber.
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher obstacles_pub_;

    //! Container for laserscan points
    std::list<geometry_msgs::Point> input_points_;
    std::list<obstacle_detector::SegmentObstacle> segments_;
};

#endif // OBSTACLE_DETECTION_NODE_H
