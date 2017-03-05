using System;
using System.Collections.Generic;
using System.Linq;
//using System.List;
using System.Text;
using System.Threading.Tasks;
using Yeti2015.Hardware.Lidar;
namespace Yeti2015.RobotService
{
    static class Buffer
    {
        //Buffer distances definitions
        private static double combinedBufferWidth = 0.55; //0.425; // 425;
        static double combinedBufferLength = 2.5; //2000.0; // the distance in which obstacle avoidance begins if an obstacle is within this distance. \
        //Also defines the required distance away any obstacle must be for the robot to consider turning toward that direction

        static double combinedHalfRobotWidth = 0.35; //350.0;// this is the width of the robot/2. Should algin with the outsides of the wheel.

        static int badCount = 0;// temporary counter to hold the number of LiDAR points in the way of a turn angle. used in CombinedVectorSCan

        //Sampling Thresholds
        const double SAMPLING_DISTANCE = 0.1; //100.0;
        const int BADCOUNTTHRESH = 5;// maximum number of LiDAR points associated with a turn angle to be considered not sage (IE if bad count > BADCOUNTTHRESH, then the robot will hit an obstacle. 
        const double ANGLE_SAMPLES = 18;// the number of angles to try to avoid an obstacle
        const double SAMPLING_ANGLE = Math.PI/ANGLE_SAMPLES;// figures out the angle between each sample angle

        public const double DOOM = (-135.0) * 2.0 * Math.PI / 360.0;//this is bad....

        static List<LocationPoint> combinedBufPoints = new List<LocationPoint>();//stores the list of LiDAR points which are within the buffer of the robot.

        static double adjust_angle(double angle, double circle)//this function limits the angle between (-pi & pi) or (-180 & 180)
        {                               //circle = 2pi for radians, 360 for degrees
            // Subtract multiples of circle
            angle -= Math.Floor(angle / circle) * circle;
            angle -= Math.Floor(2 * angle / circle) * circle;

            return angle;
        }
        
        //this function checks a turn angle (provided by wheel scans) to see if there is anything in the way of the desired turn angle.
        //It returns true if the robot can turn that direction without hitting an obstacle, and returns false if there is something in
        //the way of the entered turn angle.
        static bool combinedVectorScan(LocationPoint source, LocationPoint destination)
        {
            double target_angle = adjust_angle(Math.Atan2(destination.X - source.X, destination.Y - source.Y), 2.0*Math.PI); //limit angle between (-pi & pi) or (-180 & 180)
            double target_dist = destination.DistanceBetween(source); //find distance between corner of wheel to desired target. this equals Buffer length
            double dist;
            int i = 0, j = 0;//set up two iterators
            double tempDist = 0.0;

            LocationPoint sample_point= new LocationPoint();// create temporary point which is updated each iteration
               
            do// find out if there is anything in the way of the 
            {
                sample_point.X = (i * SAMPLING_DISTANCE) * Math.Sin(target_angle);//create temporary XY point along the desired target angle at varying distances
                sample_point.Y = (i * SAMPLING_DISTANCE) * Math.Cos(target_angle);//create temporary XY point along the desired target angle at varying distances

                foreach (var cpoint in combinedBufPoints)//look through each point point in the buffer LiDAR points to see if any are by the temporary point
                {
                    if (cpoint.Y < 0) //if it's behind the robot, skip it
                    {
                        continue;
                    }
                    dist = sample_point.DistanceBetween(cpoint); //figure out distance between the temporary point and this point in the buffer LiDAR points
                    if (dist < combinedBufferWidth) // if the dcurrent LiDAR point is close to the temporary point
                    {
                        badCount++;//increment the number of LiDAR points in the way of this target angle
                        if (badCount > BADCOUNTTHRESH) //if there's too many lidar points in the way then this is not a valid turn angle
                        {
                            return false;//return false to indicate that this turn angle is not good.
                        }//end BAD COUNT THRESH
                    }//END dist<combinedBufferWidth
                }//end For each LiDAR point
                i++;
            } while (SAMPLING_DISTANCE * i < target_dist); // while the sampling distance along the target angle is less than the target distance

            return true;//return true when it is possible for the robot to take this turn angle without hitting obstacles
        }//end CombinedVectorScan

        //this function checks to make sure both wheels can make the desired turn angle!
        static bool combinedCheckAngle(double target_angle/*, double distance*/)
        {
            //create some points to double check the turn angles
            LocationPoint leftWheel = new LocationPoint();
            LocationPoint rightWheel = new LocationPoint();
            LocationPoint target = new LocationPoint();

            // assign the points X and Y posititons
            leftWheel.Y = rightWheel.Y = 0;
            leftWheel.X = -combinedHalfRobotWidth;
            rightWheel.X = combinedHalfRobotWidth;
            target.X = combinedBufferLength/*distance*/ * Math.Sin(target_angle);
            target.Y = combinedBufferLength/*distance*/ * Math.Cos(target_angle);

            //returns true if both wheels can make this turn without hitting an obstaacle
            return (combinedVectorScan(leftWheel, target) && combinedVectorScan(rightWheel, target));
        }//end combined Check angle

        //This function searched for a right turn angle which the robot can avoid an obstacle to the right.
        // The function starts out by looking at the target point in, and finds a turn to the right of that angle, not to the right of straight forward.
        public static  double combinedRightWheelScan(LidarPoint target)
        {
            LocationPoint source = new LocationPoint();// source is the point at the front right corner of the right wheel
            double target_angle = adjust_angle(Math.Atan2(target.X, target.Y), 2.0*Math.PI);
            // this is the desired angle to the target point, //this function limits the angle between (-pi & pi) or (-180 & 180)

            double sample_phi; // temporary variable which holds various angles to look at
            int i = 0;// set up iterator to initailize at 0

            source.X = combinedHalfRobotWidth;// assigning source point to the front right corner of the right wheel
            source.Y = 0; //assigning the source point to a value of Y=0.
            
            do// find the closest turn angle on the right of the target turn angle which will avoid obstacles
            {
                LocationPoint sample_point = new LocationPoint(); // create a new point which might be the point we can go towards
                sample_point.X = combinedBufferLength * Math.Sin(target_angle - SAMPLING_ANGLE * i);//assign the sample point X Value
                sample_point.Y = combinedBufferLength * Math.Cos(target_angle - SAMPLING_ANGLE * i);//assign sample point Y Value
                sample_phi = adjust_angle(Math.Atan2(sample_point.X, sample_point.Y), 2.0 * Math.PI);//this function limits the angle between (-pi & pi) or (-180 & 180)

                //check if there is anything in the way of this turn angle. Combined Vector Scan will return
                //true when it is possible for the robot to take the input turn angle, and false if there is something in the way.
                if (combinedVectorScan(source, sample_point))
                {
                    //the turn angle of sample_phi is safe for this wheel
                    if (combinedCheckAngle(sample_phi)) //double check this turn angle is safe for both wheels
                    {
                        if (i == 0)//if this is the first itration (IE the entered desired turn angle is safe), then return 0.
                        {
                            return 0;
                        }

                        return sample_phi;//otherwise return the turn angle needed to avoid the obstacle
                    }//end Combined Check ANgle
                }//end COmbined Vector Scan
                i++;
            } while (i < ANGLE_SAMPLES);
            return DOOM;//if it is impossible to avoid an obstacle then return doom which is a large turn angle....
        }//end Combined Right Wheel Scan

        public static double combinedLeftWheelScan(LidarPoint target)// see right wheel scan for comments
        {
            LocationPoint source = new LocationPoint(); 
            double target_angle = adjust_angle(Math.Atan2(target.X, target.Y), 2*Math.PI);//this function limits the angle between (-pi & pi) or (-180 & 180)
            double sample_phi;
            int i = 0;

            source.X = -combinedHalfRobotWidth;
            source.Y = 0.0;

            do
            {
                LocationPoint sample_point = new LocationPoint();
                sample_point.X = combinedBufferLength * Math.Sin(target_angle + SAMPLING_ANGLE * i);
                sample_point.Y = combinedBufferLength * Math.Cos(target_angle + SAMPLING_ANGLE * i);
                sample_phi = adjust_angle(Math.Atan2(sample_point.X, sample_point.Y), 2*Math.PI);//this function limits the angle between (-pi & pi) or (-180 & 180)
                if (combinedVectorScan(source, sample_point))
                {
                    if (combinedCheckAngle(sample_phi))
                    {
                        if (i == 0)
                        {
                            return 0;
                        }

                        return sample_phi;
                    }
                }

                i++;
            } while (i < ANGLE_SAMPLES);
            //    return DOOM;
            return DOOM;
        }

        //Combined Update Points takes in the LiDAR Sweep, and throws out any point which is too far away, or has a negative Y Value
        public static void combinedUpdatePoints(List<LidarPoint> LMSdata )
        {            
            combinedBufPoints.Clear();//clear the previous buffer points
            badCount = 0;

            foreach (LidarPoint thispoint in LMSdata)//loop through each LiDAR point
            {
                if (Math.Sqrt(thispoint.X * thispoint.X + thispoint.Y * thispoint.Y) < combinedBufferLength) // check if the point is far away
                {
                    if (thispoint.Y > 0)//check that the point is in front of the robot, not in it's periferal view
                    {
                        combinedBufPoints.Add(new LocationPoint(thispoint.X, thispoint.Y, 0));//add the point to the buffer points
                    }//end if this point Y >0
                }//end if point is < combined Buffer Length
            }//end for each lidar point
        }// end combinde points update
    }//end Buffer Class
}//end yeti namespace