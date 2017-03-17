using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Yeti2015.RobotService
{
    static class Control
    {
        static double lastTime, thisTime;//need to save the previous time navigation was called and the current time.
        static double maxIntErr = 0.5; // this is the maximum allowed Integral error
        public static CVAR cvar = new CVAR();//Create control object which holds all PID controller definitions
        const double TICKS_PER_SECOND = 10000000;// needed to calculate time
        static double destinationThresh = 0.5; // radius threshold around waypoint to consider robot has "hit" the waypoint.

        //correctangle into range of (pi,-pi) or (180, -180)
       static double adjust_angle(double angle, double circle)
        {                               //circle = 2pi for radians, 360 for degrees
            // Subtract multiples of circle
            angle -= Math.Floor(angle / circle) * circle;
            angle -= Math.Floor(2 * angle / circle) * circle;

            return angle;
        }

        //resets PID errors to 0, and resets the clock so that errors accumulate from this point on.
        //called at the begining of the waypoint navigation and after a waypoint it hit.
        public static void initGuide()
        {
            lastTime = System.DateTime.Now.Ticks / (TICKS_PER_SECOND);
            cvar.pErr = cvar.iErr = cvar.dErr = 0;
        }

        /// the function previously known as guide, returns true when the robot has reached it's waypoint.
        public static bool areWeThereYetAndTurnPID(LocationPoint robot, Target currentTarget)
        {
            double heading = robot.Heading; // save the robots heading in the field locally
            int dir = (int)currentTarget.dir; //obtain if the robot should go forward or backward (based on read in text file)
            double dx, dy; //Delta X & Delta Y. Difference in X & Y coordinates the robot is from the target
            //double s, c;//Sin and cosine of the robots heading
            double dt;//Delta time. Holds the difference in time from the last time this function was called to this time. 
            double desiredAngle;// the desired heading. The heading which would cause the robot to directly face the target
            bool reachedTarget;//flag to detect if the robot has made it to its waypoint

            if (dir < 0) //if direction from text file says to go backward, turn heading around
            {
                heading = heading - Math.PI * Math.Sign(heading);
            }

            //FIND DISTANCE IN X & Y COORDINATES TO TARGET
            dx = currentTarget.location.X - robot.X;//find difference in X coordinate from robot locaiton to target location
            dy = currentTarget.location.Y - robot.Y;//find difference in X coordinate from robot locaiton to target location

            //FIND DISTANCE AND ANGLE TO DESTINATION
            cvar.targdist = Math.Sqrt(dx * dx + dy * dy);//current distance from the robot to the target
            // desired angle is the desired Heading the robot should have at this instance if it were to be facing the target.
            desiredAngle = adjust_angle(Math.Atan2(dx, dy), 2.0*Math.PI);
            //DEBUG(heading); DEBUG(desiredAngle);


            //USED FOR WAYPOINT NAVIGATION
            //cvar.right = dx * c - dy * s;
            //cvar.front = dy * c + dx * s;
            //c = Math.Cos(heading);//find Cosine term of the robots heading
            //s = Math.Sin(heading);//find sine term of the robots heading

            //RETRIEVE HOW FAST THE ROBOT SHOULD BE MOVING TOWARDS THIS WAYPOINT
            cvar.speed = currentTarget.speed;//retrive the speed for this waypoint from navigation text file
            
            //TIMING UPDATES FOR PID
            thisTime = System.DateTime.Now.Ticks / (TICKS_PER_SECOND);//find and save the current time
            dt = thisTime - lastTime;//save the difference from the last time to the current time. 

            /*Current Target Heading PID Calculations*/
            cvar.lastpErr = cvar.pErr;//save the last proportional error
            cvar.pErr = adjust_angle(heading - desiredAngle, 2.0 * Math.PI);//calculate the current propotional error betweem our current heading and the target heading
            cvar.iErr = cvar.iErr + cvar.pErr * dt;// increase the cumulated error.
            cvar.iErr = Math.Sign(cvar.iErr) * Math.Min(Math.Abs(cvar.iErr), maxIntErr);//limit the maxmium integral error

            if (dt != 0)//if the time has changed since the last iteration of guide. (cannot divide by 0).
            {
                cvar.dErr = (cvar.pErr - cvar.lastpErr) / dt;// calculate the derrivative error
            }
            if (Math.Cos(cvar.pErr) > 0.5) // if the robot is not facing more than +-60 degrees away from the target
            {
                //cvar.kP = 3 / Math.Max(cvar.targdist, 3); //  * 0.7112
                ////cvar.kP = 1;
                cvar.kP = 0.5;
                cvar.turn = -(cvar.kP * Math.Sin(cvar.pErr) *2 + cvar.kI * cvar.iErr + cvar.kD * cvar.dErr);  // calulate how much the robot should turn at this instant.Nattu
                ////cvar.turn = -(cvar.kP * cvar.pErr + cvar.kI * cvar.iErr + cvar.kD * cvar.dErr);
            }
            else//if the robot is facing more than 60 degrees away from the target
            {
                cvar.turn = -0.5 * Math.Sign(cvar.pErr); //if you need to turnin place, then ignore PID temporarily
            }
            lastTime = thisTime;//update the times

            //inLastTarget = (lastDist < leaveTargetThresh);
            //approachingTarget = (cvar.targdist < approachingThresh);
            reachedTarget = (cvar.targdist < destinationThresh);//if the robot is very close to the target, then it has hit the target.

            return reachedTarget;//return true if the robot is close to the target.
        }



    }
}
