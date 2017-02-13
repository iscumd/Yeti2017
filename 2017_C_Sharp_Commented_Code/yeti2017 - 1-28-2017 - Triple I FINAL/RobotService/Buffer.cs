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

        static double combinedTargDist;

        private static double combinedBufferWidth = 0.55; //0.425; // 425;
        static double combinedBufferLength = 2.5; //2000.0;
        static double combinedHalfRobotWidth = 0.35; //350.0;
        //static int combinedNumPoints = 0;
        static int badCount = 0;

        const double SAMPLING_DISTANCE = 0.1; //100.0;
        const int BADCOUNTTHRESH = 5;// 1;

        const double ANGLE_SAMPLES = 18;
        const double SAMPLING_ANGLE = Math.PI/ANGLE_SAMPLES;

        public const double DOOM = (-135.0) * 2.0 * Math.PI / 360.0;

        static List<LocationPoint> combinedBufPoints = new List<LocationPoint>();

        static double adjust_angle(double angle, double circle)
        {                               //circle = 2pi for radians, 360 for degrees
            // Subtract multiples of circle
            angle -= Math.Floor(angle / circle) * circle;
            angle -= Math.Floor(2 * angle / circle) * circle;

            return angle;
        }
        
        static bool combinedVectorScan(LocationPoint source, LocationPoint destination)
        {
            //double target_angle = adjust_angle(Math.Atan2(destination.Y - source.X, destination.Y - source.Y), 2.0*Math.PI);
            double target_angle = adjust_angle(Math.Atan2(destination.X - source.X, destination.Y - source.Y), 2.0*Math.PI);
            double target_dist = destination.DistanceBetween(source);
            double dist;
            int i = 0, j = 0;
            double tempDist = 0.0;
            LocationPoint sample_point= new LocationPoint();
               
            do
            {
                sample_point.X = (i * SAMPLING_DISTANCE) * Math.Sin(target_angle);
                sample_point.Y = (i * SAMPLING_DISTANCE) * Math.Cos(target_angle);

                foreach (var cpoint in combinedBufPoints)
                //for (j = 0; j < combinedNumPoints; j++)
                {
                    if (cpoint.Y < 0) //if it's behind the robot, skip it
                    {
                        continue;
                    }

                    /* //added since last year code, combinedTargDist ever defied
                     * 
                    tempDist = Math.Sqrt(cpoint.X * cpoint.X + cpoint.Y * cpoint.Y) * 0.001;//MM2M;
                    if (tempDist > combinedTargDist) continue;
                    */
                    dist = sample_point.DistanceBetween(cpoint);
                    //if(dist < combinedBufferWidth + combinedHalfRobotWidth) 
                    if (dist < combinedBufferWidth)
                    {
                        badCount++;
                        if (badCount > BADCOUNTTHRESH) //if there's too many lidar points too close, that's bad
                        {
                            return false;
                        }
                    }
                }

                i++;
            } while (SAMPLING_DISTANCE * i < target_dist);

            return true;
        }

        static bool combinedCheckAngle(double target_angle/*, double distance*/)
        {
            LocationPoint leftWheel = new LocationPoint();
            LocationPoint rightWheel = new LocationPoint();
            LocationPoint target = new LocationPoint();

            leftWheel.Y = rightWheel.Y = 0;
            leftWheel.X = -combinedHalfRobotWidth;
            rightWheel.X = combinedHalfRobotWidth;
            target.X = combinedBufferLength/*distance*/ * Math.Sin(target_angle);
            target.Y = combinedBufferLength/*distance*/ * Math.Cos(target_angle);

            return (combinedVectorScan(leftWheel, target) && combinedVectorScan(rightWheel, target));
        }


        //these should be combined, just add direction!
        public static  double combinedRightWheelScan(LidarPoint target)
        {
            LocationPoint source = new LocationPoint();
            double target_angle = adjust_angle(Math.Atan2(target.X, target.Y), 2.0*Math.PI);
            double sample_phi;
            int i = 0;
            source.X = combinedHalfRobotWidth;
            source.Y = 0;

            do
            {
                LocationPoint sample_point = new LocationPoint();
                sample_point.X = combinedBufferLength * Math.Sin(target_angle - SAMPLING_ANGLE * i);
                sample_point.Y = combinedBufferLength * Math.Cos(target_angle - SAMPLING_ANGLE * i);
                sample_phi = adjust_angle(Math.Atan2(sample_point.X, sample_point.Y), 2.0 * Math.PI);
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
        public static double combinedLeftWheelScan(LidarPoint target)
        {
            LocationPoint source = new LocationPoint(); 
            double target_angle = adjust_angle(Math.Atan2(target.X, target.Y), 2*Math.PI);
            double sample_phi;
            int i = 0;

            source.X = -combinedHalfRobotWidth;
            source.Y = 0.0;

            do
            {
                LocationPoint sample_point = new LocationPoint();
                sample_point.X = combinedBufferLength * Math.Sin(target_angle + SAMPLING_ANGLE * i);
                sample_point.Y = combinedBufferLength * Math.Cos(target_angle + SAMPLING_ANGLE * i);
                sample_phi = adjust_angle(Math.Atan2(sample_point.X, sample_point.Y), 2*Math.PI);
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
        public static void combinedUpdatePoints(List<LidarPoint> LMSdata ){
            
            combinedBufPoints.Clear();
            badCount = 0;
            
            //int i;
            //int tempdata;
            //combinedNumPoints = 0;
            foreach (LidarPoint thispoint in LMSdata)
            //for(i=0;i<1081;i++){
        
                if(Math.Sqrt(thispoint.X * thispoint.X + thispoint.Y * thispoint.Y ) <  combinedBufferLength)
                {
                    if(thispoint.Y>0)
                    {
                        combinedBufPoints.Add(new LocationPoint(thispoint.X, thispoint.Y ,0 ));
                    }
                }
            }
        }


    }







