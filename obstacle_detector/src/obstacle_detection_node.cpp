#include "obstacle_detection_node.h"

/* Some of the concepts for this node were copied from:
 * https://github.com/tysik/obstacle_detector/blob/master/src/obstacle_detector.cpp
 * And most of the code structure was derived from ROS tutorials
 * http://wiki.ros.org/ROSNodeTutorialC%2B%2B
 * */

typedef obstacle_detector::Obstacles Obstacles;
using obstacle_detector::SegmentObstacle;
using sensor_msgs::LaserScan;
typedef std::list<geometry_msgs::Point> PointList;
typedef std::list<obstacle_detector::SegmentObstacle> SegmentList;

int main(int argc, char** argv) {
    std::cout << "starting obstacle detection node...\n";
    ros::init(argc, argv, "obstacle_detector");
    ObstacleDetection od;
    return 0;
}

/*--------------------------------------------------------------------
 * ObstacleDetection()
 * Constructor.
 *------------------------------------------------------------------*/

ObstacleDetection::ObstacleDetection()
{
    // Setup ROS subscription object
    scan_sub_ = nh_.subscribe("/mybot/front_laser/scan", 10, &ObstacleDetection::laserCallback, this);   
    // Setup ROS publihs object
    obstacles_pub_ = nh_.advertise<Obstacles>("obstacles", 10);
    // Run ObstacleDetection object
    ros::spin();
} // end ObstacleDetection()

/*--------------------------------------------------------------------
 * ~ObstacleDetection()
 * Destructor.
 *------------------------------------------------------------------*/

ObstacleDetection::~ObstacleDetection()
{
} // end ~ObstacleDetection()

/* publishMessage()  {{{1
 * Publish list of obstacles.
 *------------------------------------------------------------------*/

void ObstacleDetection::publishObstacles()
{
    Obstacles obstacles;
    obstacles.header.stamp = ros::Time::now();
    for (SegmentList::const_iterator ci = segments_.begin(); ci != segments_.end(); ++ci)
    {
        SegmentObstacle segment;
        segment.first_point.x = ci->first_point.x;
        segment.first_point.y = ci->first_point.y;
        segment.last_point.x = ci->last_point.x;
        segment.last_point.y = ci->last_point.y;
        obstacles.segments.push_back(segment);
    }
    obstacles_pub_.publish(obstacles);
} // end publishObstacles()

/* laserCallback()  {{{1
 * Purpose: Callback function for subscriber to process laser data
 * Inputs: subscription to laser topic defined in Constructor
 * Outputs: Saves valid laser data to list of cartesian points
 */

void ObstacleDetection::laserCallback(const LaserScan::ConstPtr& scan)
{
    std::cout << "laserCallback...\n";
    // Note: scan->ranges[] are laser readings
    double phi = scan->angle_min;
    for (int i = 0; i < scan->ranges.size(); i++)
    {
        double r = scan->ranges[i];
        geometry_msgs::Point point;
        // TODO CONVERT TO CART in robot coordinate frame
        if (r < scan->range_max)
        {
            point.x = r * cos(phi);
            point.y = r * sin(phi);
            std::cout << "object: " << i 
                      << " x: " << point.x
                      << " y: " << point.y << "\n";
            input_points_.push_back(point);
        }
        phi += scan->angle_increment;
    }
    detectObstacles();
    publishObstacles();
} // end publishCallback()

/* detectObstacles()  {{{1
 * Purpose: Detect obstacles from laser scan points
 * Inputs: list of points from laser
 * Outputs: list of objects identified in laser scan
 */

void ObstacleDetection::detectObstacles()  // {{{1
{
    segments_.clear();
    // TODO Classify points as objects (lines, circles)
    // RANSAC TODO MAKE THIS A FUNCTION
    // Initialize
    int maxIterations = 50;
/*
    // RANSAC Algorithm
    while (k < maxIterations)
    {
        // RANDOMLY SELECT TWO POINTS FROM LIST OF POINTS
        int numPoints = input_points_.size();
        // randomInt = min + (rand() % (int)(max - min + 1))
        int point1Index = rand() % (int)(numPoints + 1);
        geometry_msgs::Point point1 = input_points_[point1Index];
        int point2Index = rand() % (int)(numPoints + 1);
        while (point1Index == point2Index)
        {
            point2Index = rand() % (int)(numPoints + 1);
        }
        geometry_msgs::Point point2 = input_points_[point1Index];

        // FIT A LINE THROUGH THE TWO POINTS
        // COMPUTE DISTANCES OF ALL OTHER POINTS TO THIS LINE
        // CONSTRUCT THE INLIER SET
        // STORE THE INLIERS
    }
    // CHOOSE SET WITH MAXIMUM NUMBER OF INLIERS AS SOLUTION
*/
    for (PointList::const_iterator ci = input_points_.begin(); ci != input_points_.end(); ++ci)
    {
        SegmentObstacle segment;
        segment.first_point.x = ci->x;
        segment.first_point.y = ci->y;
        segment.last_point.x = ci->x;
        segment.last_point.y = ci->y;
        segments_.push_back(segment);
    }
}

