using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Yeti2015.RobotService
{
    public class Obstacle
    {
        public double X { get; private set; }// holds the X coordinate in the field of the obstacle
        public double Y { get; private set; }// holds the Y coordinate in the field of the obstacle
        public double Dist { get; private set; }//old
        public double Phi { get; private set; }//old
        public double Theta { get; private set; }//old
        public double Lmx { get; private set; }//old
        public double Lmy { get; private set; }//old
        public double rough_size_of_obs { get; private set; }//this is the size of the obstacle based on the sum of the distance between the points which make up the obstacle
        public double line_size_of_obs { get; private set; }//this is the distance between the first and last point of the obstace. If it is = to rough size, then we know the obstacle is a flat surface(stop sign)
        public bool moving { get; private set; }//flag, True if obstacle is moving, False if Obstacle is static
        public int obj_start_index = 0;//Lidar scan index where object starts
        public int obj_end_index = 0;//Lidar scan index where object end
        public Hardware.Lidar.LidarPoint start_point;//starting point of  object
        public Hardware.Lidar.LidarPoint end_point;//end point of object

        public static List<Obstacle> obstacles = new List<Obstacle>();//holds all found obstacles
        static int numObstacles = 0;//stores obstacle count
        static int linked_count = 0;//holds the number of linked points together of an object
        static double sum = 0;//holds the sum of the distances from the obstace. This will be used to find avg dist to the object
        static double obj_size_sum=0;//holds sum of distance between points that make up an obstacle, finds rough size of obstacle
        static int obj_start_ind=0;//holds the index of the lidar data which the object started at 
        static int obj_end_ind=0;//holds the index of the lidar data which the object ended at 
        static double maxRadius;
        static int highThresh;
        static int lowThresh;
        static double NONSEPARATION_THRESH;
        static double is_this_landmark_thresh = .30;
        static double moving_obs_size=.45;//the width of the moving obsatcle
        static double line_size_diff_thresh=.02;// threshold for difference between the distance between start and endpoint of the obstacle, and the rough size (each points distsnace sum)
        static double moving_obs_size_thresh = .07;//threshold for difference between detection of moving obstacle.
        static double MM2M = 0.001;
        static int M2MM = 1000;
        static int forgive_count = 3;

        public Obstacle()
        {
            
        }

        public static void ObstacleInitialize(double _maxRadius = 10.0, int _highThresh = 50, int _lowThresh = 4, int _nonSepThresh = 200)
        {
            ////this.obstacles = new List<Obstacle>();
            numObstacles = 0;
            linked_count = 0;
            sum = 0;
            maxRadius = _maxRadius;
            highThresh = _highThresh;
            lowThresh = _lowThresh;
            NONSEPARATION_THRESH = _nonSepThresh;
            ////this.MM2M = 0.001;
            ////this.M2MM = 1000;
        }

        static void ClearState()
        {
            obj_size_sum = 0;//holds sum of distance between points that make up an obstacle, finds rough size of obstacle
            obj_start_ind = 0;//holds the index of the lidar data which the object started at 
            obj_end_ind = 0;
            sum = 0;
            linked_count = 0;
        }

        static void LinkPoint(int i, double dist, double point_dist)//when two points are close to one another then link them togehter
        {
            linked_count = linked_count + 1;//incrememnt the number of linked points
            sum = sum + dist;//update the size of the object
            obj_size_sum = obj_size_sum + point_dist;
        }

        static void AddObstacle(int z, LocationPoint robot, List<Hardware.Lidar.LidarPoint> LMSdata, List<LocationPoint> landmarks)
        {
            double index = z - linked_count / 2;
            double mag = sum / linked_count;//sum/linked;//this finds the average distance between points in meters

            double this_x= robot.X + mag * Math.Sin(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);
            double this_y = robot.Y + mag * Math.Cos(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);
           
            if(mag > maxRadius || linked_count > highThresh || linked_count < lowThresh)//if obstacle is larger than max radius, or the number of linked points is large, or the number of linked points is too small. 
            {
                ClearState();//obstacle is not worth acknowledging, clear the state
            }
            else//This is an obscale!!!!
            {
                double obj_x= robot.X + mag * Math.Sin(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);
                double obj_y = robot.Y + mag * Math.Cos(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);

                bool this_is_a_landmark_flag=false;
                bool outside_of_field = false;
                for (int k = 0; k < landmarks.Count(); k++)
                {
                    if(    Math.Sqrt( Math.Pow((obj_x - landmarks[k].X),2) + Math.Pow((obj_y - landmarks[k].Y), 2) ) < is_this_landmark_thresh )
                    {
                        this_is_a_landmark_flag = true;//this is a landmark!
                    }
                }
                
                if (obj_x > 4.75 || obj_x < -1.750 || obj_y > 11.75 || obj_y < -2.75)//check if obstacle is outside of Triple Ifield
                //if (obj_x > 1.75 || obj_x < -1.750 || obj_y > 11.75 || obj_y < -2.75)//check if obstacle is outside of Single Ifield
                { outside_of_field = true; }

                if (!this_is_a_landmark_flag && ! outside_of_field)
                {
                    Obstacle newObstacle = new Obstacle();
                    newObstacle.X = robot.X + mag * Math.Sin(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);
                    newObstacle.Y = robot.Y + mag * Math.Cos(((135 - index * 0.25) * (Math.PI / 180.0)) + robot.Heading);

                    newObstacle.obj_start_index = obj_start_ind;
                    newObstacle.obj_end_index = obj_end_ind;
                    newObstacle.Dist = mag;
                    newObstacle.rough_size_of_obs = obj_size_sum;

                    //Console.WriteLine(mag);// obstacle size = mag

                    //convert start index into a point

                    newObstacle.start_point = LMSdata[obj_start_ind];
                    newObstacle.end_point = LMSdata[obj_end_ind];


                    newObstacle.line_size_of_obs = newObstacle.start_point.DistanceBetween(newObstacle.end_point);

                    if (mag < 5)
                    {
                        //if (Math.Abs(newObstacle.line_size_of_obs - newObstacle.rough_size_of_obs) < line_size_diff_thresh)//check if the object is flat
                        //{
                        if (Math.Abs(newObstacle.line_size_of_obs - moving_obs_size) < moving_obs_size_thresh)// check if object is close to moving obstacle size
                        {
                            newObstacle.moving = true;
                            //Console.WriteLine("start point X: " + start_point.X + "\tY: " + start_point.Y);
                            //Console.WriteLine("end point   X: " + end_point.X + "\tY: " + end_point.Y);
                            Console.WriteLine("THIS OBSTACLE IS MOVING!");
                        }
                        else { newObstacle.moving = false; }
                        // }
                        //else { newObstacle.moving = false; }
                    }
                    else
                    {
                        newObstacle.moving = false;
                    }


                    //convert end index into a point
                    //find distance between start point and end points
                    //newObstacle.Phi = Math.Atan2(newObstacle.X, newObstacle.Y);
                    numObstacles = numObstacles + 1;
                    obstacles.Add(newObstacle);
                }
            }
        }
    
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

        public static List<Obstacle> FindObstacles(List<int> data, LocationPoint robot, List<Hardware.Lidar.LidarPoint> LMSdata, List<LocationPoint> landmarks)//data is a list of lidar points which are not empty(IE returned a value).
        {
            ClearObstacles();// clear obstacles found
            ClearState();// reset the state machine for finding obstacles

            int i = 0;
            bool flag_already_linking = false;
            // loop through each point of non-empty lidar data
            for (int j = 360; j < data.Count - 361; j++)//loop through each point which is in front of yeti, not to the side. 
            {
                var this_point = LMSdata[j];
                var flag_points_linked = false;
                if (this_point.Distance < maxRadius)
                {
                    for (i = 1; i < forgive_count + 1; i++)
                    {

                        var next_point = LMSdata[j + i];

                        double this_dist = this_point.DistanceBetween(next_point);
                        if (this_point.DistanceBetween(next_point) < NONSEPARATION_THRESH * i * MM2M)
                        {
                            LinkPoint(j, this_point.Distance, this_point.DistanceBetween(next_point));
                            flag_points_linked = true;
                            if (!flag_already_linking)
                            {
                                obj_start_ind = j;
                                flag_already_linking = true;
                            }
                            break;
                        }
                    }//end for
                }
                if (flag_points_linked == false)
                {
                    if (flag_already_linking)
                    {
                        obj_end_ind = j;
                        AddObstacle(j, robot, LMSdata,landmarks);
                    }
                    flag_already_linking = false;
                    ClearState();
                }
                else
                {
                    j = j + i - 1;
                    if (j > data.Count - 361)
                    {
                        if (flag_already_linking)
                        {
                            obj_end_ind = j;
                            AddObstacle(j, robot, LMSdata,landmarks);
                            ClearState();
                        }
                    }
                }
            }
            //printObstacles(obstacles);
            return obstacles;
        }

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
