using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Yeti2015.RobotService
{
    public class Obstacle
    {
        //Every saved obstacle has the following information:
        public double X { get; private set; }// holds the X coordinate in the field of the obstacle
        public double Y { get; private set; }// holds the Y coordinate in the field of the obstacle
        public double Dist { get; private set; }//holds how far obstacle is from robot
        public int obj_start_index = 0;//Lidar scan index where object starts
        public int obj_end_index = 0;//Lidar scan index where object end
        public double rough_size_of_obs { get; private set; }
        //this is the size of the obstacle based on the sum of the distance between the points which make up the obstacle
        public double line_size_of_obs { get; private set; }
        //this is the distance between the first and last point of the obstace. 
        //If it is = to rough size, then we know the obstacle is a flat surface(stop sign)
        public bool moving { get; private set; }//flag, True if obstacle is moving, False if Obstacle is static
        public Hardware.Lidar.LidarPoint start_point;//starting point of  object
        public Hardware.Lidar.LidarPoint end_point;//end point of object
        
        //This is the list of the Found obstacles, each which holds the above information
        public static List<Obstacle> obstacles = new List<Obstacle>();//holds all found obstacles

        //Metadata and temporary variables about the list of obstacles
        static int numObstacles = 0;//stores obstacle count
        static int linked_count = 0;//holds the number of linked points together of an object
        static double sum = 0;//holds the sum of the distances from the obstace. This will be used to find avg dist to the object
        static double obj_size_sum=0;//temporarily holds sum of distance between points that make up an obstacle, finds rough size of obstacle
        static int obj_start_ind=0;//temporarilyholds the index of the lidar data which the object started at 
        static int obj_end_ind=0;//temporarilyholds the index of the lidar data which the object ended at 

        //Thresholds for determinig if a object is an obstacle
        static double maxRadius;// holds teh maximum distance an obstacle can be, otehrwise it is just ignored.
        static int highThresh;//holds the maximum number of lidar points an obstacle can have in order to still be considered an obstacle(and not a wall)
        static int lowThresh;//holds the minimum number of lidar points an obstacle must have in order to still be considered an obstacle(filtering out random noise such as snow)
        static double NONSEPARATION_THRESH;//maximum distance between to neighboring LiDAR points for the points to be considered part of the same obstacle

        //thresholds for determining what type of object the obstacle is (IE moving or static)
        static double is_this_landmark_thresh = .30;// radius around landmark center point. If obstacle is within the radius, it is considered a landmark.
        static double moving_obs_size=.45;//the width of the moving obsatcle in meters
        static double moving_obs_size_thresh = .07;//threshold for difference between detection of moving obstacle.
        static double line_size_diff_thresh=.02;
        // threshold for difference between the distance between start and endpoint of the obstacle, and the rough size (each points distsnace sum)
        static int forgive_count = 3;// this is the number of points in front of the current point which obstacle detection will look.
        //IE if the lidar point 3 away from this one is still close, then that point is still considered an obstacle.

        static double MM2M = 0.001;//conversion factor for Millimeters to meters
        static int M2MM = 1000;//conversion factor for meters to millimeters
        

        public Obstacle()
        {
            
        }

        public static void ObstacleInitialize(double _maxRadius = 10.0, int _highThresh = 50, int _lowThresh = 4, int _nonSepThresh = 200)
        {
            numObstacles = 0; // intitallize the number of obstacles found to zero.
            linked_count = 0; // initialize the number of linked points in an obstacle to zero.
            sum = 0;// initialze the distance between each point sum to zero
            maxRadius = _maxRadius;//sets a maximum distance from the robot which obstacles will be detected. 
            //IE if a obstacle is detected further away from this point then it is not considered an obstacle yet.
            NONSEPARATION_THRESH = _nonSepThresh;// the distance in mm that a point can be from the next to still be considered on the same object
            highThresh = _highThresh;//set the maxmimum number of points an obstacle can be to be considered an "obstacle"
            lowThresh = _lowThresh;//set the minimum number of points an obstacle can be to be considered an "obstacle"
        }

        static void ClearState()//called once an obstacle has been found and added, or something is deemed not an obsatcle
        {
            //clear the temporary variables used when searching for obstacles
            obj_size_sum = 0;//holds sum of distance between points that make up an obstacle, finds rough size of obstacle
            obj_start_ind = 0;//holds the index of the lidar data which the object started at 
            obj_end_ind = 0;//holds end inted of the LiDAR data the object ends at
            sum = 0;//holds the sum of distances of the obstacle
            linked_count = 0;//holds number of points which are linked for this obstacle
        }

        static void LinkPoint(int i, double dist, double point_dist)//when two points are close to one another then link them togehter
        {
            linked_count = linked_count + 1;//incrememnt the number of linked points
            sum = sum + dist;//update the size of the object
            obj_size_sum = obj_size_sum + point_dist;
        }

       
        static void AddObstacle(int z, LocationPoint robot, List<Hardware.Lidar.LidarPoint> LMSdata, List<LocationPoint> landmarks)
        {
            double index = z - linked_count / 2;// save the center index of the obstacle in this variable
            double mag = sum / linked_count;//this finds the average distance from the robot of all the points which make up the obstacle. Saved in meters

            double this_x= robot.X + mag * Math.Sin(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);//save estimated x value of the center of the obstacle in the field
            double this_y = robot.Y + mag * Math.Cos(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);//save estimated Y value of the center of the obstacle in the field

            //if obstacle is larger than max radius, or the number of linked points too large, or the number of linked points is too small. 
            if (mag > maxRadius || linked_count > highThresh || linked_count < lowThresh)
            {//this is not an obstacle.
                ClearState();//obstacle is not worth acknowledging, clear the state
            }
            else//This is an obscale which is not to small or big, and is less close enough to care
            {
                //figure out where the obstacle is in the XY field
                double obj_x= robot.X + mag * Math.Sin(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);// save this objects x value
                double obj_y = robot.Y + mag * Math.Cos(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);// save this objects y value

                bool this_is_a_landmark_flag=false;//reset the "this is a landmark" flag
                bool outside_of_field = false;// reset the "the obstacle is outside of the field" flag

                //Figure out if this is a landmark or not
                for (int k = 0; k < landmarks.Count(); k++)// go through each landmark position from the list of known landmarks
                {
                    if( Math.Sqrt( Math.Pow((obj_x - landmarks[k].X),2) + Math.Pow((obj_y - landmarks[k].Y), 2) ) < is_this_landmark_thresh )
                    {
                        this_is_a_landmark_flag = true;//this is a landmark!
                    }
                }
                
                //figure out if the object is within the plowing field or not
                if (obj_x > 4.75 || obj_x < -1.750 || obj_y > 11.75 || obj_y < -2.75)//check if obstacle is outside of Triple Ifield
                //if (obj_x > 1.75 || obj_x < -1.750 || obj_y > 11.75 || obj_y < -2.75)//check if obstacle is outside of Single Ifield
                { outside_of_field = true; }

                //if the object is in the field, and is not a landmark
                if (!this_is_a_landmark_flag && ! outside_of_field)
                {
                    Obstacle newObstacle = new Obstacle(); //create a new obstacle
                    newObstacle.X = robot.X + mag * Math.Sin(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);// save the obstacles X value
                    newObstacle.Y = robot.Y + mag * Math.Cos(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);// save the obstacles X value

                    //save start and end indexes
                    newObstacle.obj_start_index = obj_start_ind;// save the obstacles starting index
                    newObstacle.obj_end_index = obj_end_ind;//save the obstacles ending index
                    
                    //convert start index into a point
                    newObstacle.start_point = LMSdata[obj_start_ind];
                    newObstacle.end_point = LMSdata[obj_end_ind];

                    //save characteristics of the obstacle
                    newObstacle.Dist = mag;// save the objects distance from the robot
                    newObstacle.rough_size_of_obs = obj_size_sum;// save the sum of distances of the points (EG would hold the arc length of a landmark
                    newObstacle.line_size_of_obs = newObstacle.start_point.DistanceBetween(newObstacle.end_point);// This is distance from start point to end point.
                    //if the obstacle is a straight line then the rough size and line size will be the same

                    if (mag < 5)// check if the obstacle is within 5 meters of the robot.
                        //this is done only to react to the moving obstacle if it is within 5meters. 
                    {
                        //if (Math.Abs(newObstacle.line_size_of_obs - newObstacle.rough_size_of_obs) < line_size_diff_thresh)//check if the object is flat(NOt used)
                        //{
                        if (Math.Abs(newObstacle.line_size_of_obs - moving_obs_size) < moving_obs_size_thresh)// check if object is the same size as the moving obstacle.
                        {
                            newObstacle.moving = true;//we have a moving obstacle
                            Console.WriteLine("THIS OBSTACLE IS MOVING!");//print that it has been seen
                        }
                        else { newObstacle.moving = false; }// we know the obstacle isn't moving
                    }
                    else// if the obstacle is more than 5 meters away don't stop for it yet.
                    {
                        newObstacle.moving = false;
                    }
                    numObstacles = numObstacles + 1;//increment number of found obstacles
                    obstacles.Add(newObstacle);//add the new obstacle to the list of obstacles
                }//end if landmark or or outside field
            }//end check to add obstacle
        }//end add object
    
        static void ClearObstacles()
        {
            obstacles.Clear();
        }

        public static void printObstacles(List<Obstacle> obs)
        {
            Console.WriteLine("\n\nThere are " + obs.Count() + "obstacles!");
            for (int i=0; i< obs.Count; i++)
            {
                Console.WriteLine("\n\nObstacle location   X: " + obs[i].X + "\tY: " + obs[i].Y);
                Console.WriteLine("Start Index: " + obs[i].obj_start_index + "\tEnd Index: " + obs[i].obj_end_index);
                Console.WriteLine("Start point   X: " + obs[i].start_point.X + "\tY: " + obs[i].start_point.Y);
                Console.WriteLine("end point   X: " + obs[i].end_point.X + "\tY: " + obs[i].end_point.Y);
                Console.WriteLine("Distance to obstacle: " + obs[i].Dist);
                Console.WriteLine("distance between points = " + obs[i].line_size_of_obs);
                Console.WriteLine("size of object = " + obs[i].rough_size_of_obs);
            }
            
        }

        public static List<Obstacle> FindObstacles(List<int> data, LocationPoint robot, 
            List<Hardware.Lidar.LidarPoint> LMSdata, List<LocationPoint> landmarks)//data is a list of lidar points
        {
            ClearObstacles();// clear obstacles found
            ClearState();// reset the state machine for finding obstacles

            int i = 0;
            bool flag_already_linking = false;//once two points are linked this flag is set to true. And is set to false after the end of an obstacle is found.

            // loop through each point of non-empty lidar data
            //loop through each point which is in front of yeti, not to the side. IE only points with a positive Y value
            for (int j = 360; j < data.Count - 361; j++) 
            {
                var this_point = LMSdata[j];//create an XY point for this distance from the LiDAR Data.
                var flag_points_linked = false;//set to 1 when points are part of the same obstacle, and set to 0 at each increase of j

                if (this_point.Distance < maxRadius)//if the LiDAR point is closer than the max Radius (typically 15M as this is max size of the field.)
                {
                    for (i = 1; i < forgive_count + 1; i++) // look at the next forgive_count number of points in the lidar scan to see if they are close to this point
                    {

                        var next_point = LMSdata[j + i]; // create and XY point from the next LiDAR point

                        double this_dist = this_point.DistanceBetween(next_point); // find a distance between the current lidar point (i) and the next lidar point(i+j)
                        if (this_point.DistanceBetween(next_point) < NONSEPARATION_THRESH * i * MM2M) 
                            // if the distance between the points is less than Nonseperation thresh * the index away from the jth lidar point(converted to meters)
                        {
                            //it has been determined that point j and point i+j are part of the same obstacle, so we need to link them togethjer
                            LinkPoint(j, this_point.Distance, this_point.DistanceBetween(next_point));//link the points together!
                            flag_points_linked = true;// we set this flag to true to indicate that points have been linked so we can assemble a 
                            if (!flag_already_linking)//if this Lidar point j is the first Lidar point in an obstacle
                            {
                                obj_start_ind = j;//save the starting index of the obstacles
                                flag_already_linking = true;//set the flag so that it is known that the lidar points are currently being linked.
                            }
                            break;// we have found a point at most forgive_count away from lidar point j, so we can stop set j = to j+i now, and keep looking at future points
                        }
                    }//end for
                }
                if (flag_points_linked == false)// if lidar point j+i+forgive_count through j+1 are not close enough to lidar point j to be considered the same obstalce
                {
                    if (flag_already_linking)//the linked points from previous iterations were linking so we need to close the obstacle.
                    {
                        obj_end_ind = j;// store  the last lidar point index of the obstacle
                        AddObstacle(j, robot, LMSdata,landmarks);// add the obstacle to the obstacle list!
                    }
                    flag_already_linking = false;//flag that the obstacle has ended
                    ClearState();//reset the state of variables used in "link_points"
                }
                else// if lidar point j is clsoe enough to lidar point j+i, then we continue to link the points
                {
                    j = j + i - 1;//update what j should be (used if i > 0), subtract 1 because j will be incremented at start of forloop
                    if (j > data.Count - 361)// if the lidar point j is behind the robot (IE the point has a negative Y value)
                    {
                        if (flag_already_linking)// if the points were being linked 
                        {
                            obj_end_ind = j;// store  the last lidar point index of the obstacle
                            AddObstacle(j, robot, LMSdata,landmarks);// add the obstacle to the obstacle list!
                            ClearState();// clear the state variables used in link_points
                        }
                    }
                }
            }
            //printObstacles(obstacles);
            return obstacles;//return the list of obstacles.
        }//end find obstacles

        public static void ObstaclesInit()
        {
            ////free(obstacles);
            ////free(Prev_Objects);
            ////free(Vis_Objects);
            ////Prev_Objects = (Obstacle*)malloc(MAX_OBJECTS * sizeof(Obstacle));
            ////Vis_Objects = (Obstacle*)malloc(MAX_OBJECTS * sizeof(Obstacle));
            ////obstacles = (Obstacle*)malloc(MAX_OBJECTS * sizeof(Obstacle));
            numObstacles = 0;
        }

    }
}
