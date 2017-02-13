using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Yeti2015.RobotService
{
    static class Control
    {
        static double lastTime, thisTime;
        static double maxIntErr = 0.5;
       public static CVAR cvar = new CVAR();
       const double TICKS_PER_SECOND = 10000000;
        static bool approachingTarget;//, inLastTarget;
        static double destinationThresh = 0.5, approachingThresh, leaveTargetThresh;


       static double adjust_angle(double angle, double circle)
        {                               //circle = 2pi for radians, 360 for degrees
            // Subtract multiples of circle
            angle -= Math.Floor(angle / circle) * circle;
            angle -= Math.Floor(2 * angle / circle) * circle;

            return angle;
        }


        public static void initGuide()
        {
            lastTime = System.DateTime.Now.Ticks / (TICKS_PER_SECOND);
            cvar.pErr = cvar.iErr = cvar.dErr = 0;
        }


        /// <summary>
        /// the function previously known as guide
        /// </summary>
        /// <param name="robot"></param>
        /// <param name="currentTarget"></param>
        /// <returns></returns>
        public static bool areWeThereYetAndTurnPID(LocationPoint robot, Target currentTarget)
        {

            double heading = robot.Heading;
            int dir = (int)currentTarget.dir; //forward or backward
            double dx, dy, s, c, /*nx, ny,*/ dt;//, temp;
            //double dlastx, dlasty, lastDist;
            double desiredAngle;

            bool reachedTarget;

            if (dir < 0) //if direction from text file says to go backward, turn heading around
            {
                heading = heading - Math.PI * Math.Sign(heading);
            }

            //how far we are from the current location
            dx = currentTarget.location.X - robot.X;
            dy = currentTarget.location.Y - robot.Y;

            //dlastx = lastTarget.X - robot.X;
            //dlasty = lastTarget.Y - robot.Y;
            //lastDist = Math.Sqrt(dlastx * dlastx + dlasty * dlasty);

            c = Math.Cos(heading);
            s = Math.Sin(heading);

            cvar.targdist = Math.Sqrt(dx * dx + dy * dy);
            //cvar.right = dx * c - dy * s;
            //cvar.front = dy * c + dx * s;
            desiredAngle = adjust_angle(Math.Atan2(dx, dy), 2.0*Math.PI);

            cvar.speed = currentTarget.speed;

            //    DEBUG(heading); DEBUG(desiredAngle); DEBUGN(nextAngle);

            /*PID Calculations cvar3 Heading*/

            //nx = -(currentTarget.Y - lastTarget.Y); // -Delta Y
            //ny = (currentTarget.X - lastTarget.X); //  Delta X
            //temp = Math.Sqrt(nx * nx + ny * ny) + 1e-3; // To avoid division by zero
            //nx /= temp;
            //ny /= temp;

            thisTime = System.DateTime.Now.Ticks / (TICKS_PER_SECOND);
            dt = thisTime - lastTime;

            /*Current Target Heading PID Calculations*/
            cvar.lastpErr = cvar.pErr;
            cvar.pErr = adjust_angle(heading - desiredAngle, 2.0 * Math.PI);
            cvar.iErr = cvar.iErr + cvar.pErr * dt;
            cvar.iErr = Math.Sign(cvar.iErr) * Math.Min(Math.Abs(cvar.iErr), maxIntErr);
            if (dt != 0)
            {
                cvar.dErr = (cvar.pErr - cvar.lastpErr) / dt;
            }
            if (Math.Cos(cvar.pErr) > 0.5) // +-60 degrees
            {
                //cvar.kP = 3 / Math.Max(cvar.targdist, 3); //  * 0.7112
                ////cvar.kP = 1;
                cvar.kP = 0.5;
                cvar.turn = -(cvar.kP * Math.Sin(cvar.pErr) *2 + cvar.kI * cvar.iErr + cvar.kD * cvar.dErr);  // Nattu
                ////cvar.turn = -(cvar.kP * cvar.pErr + cvar.kI * cvar.iErr + cvar.kD * cvar.dErr);
            }
            else
            {
                cvar.turn = -0.5 * Math.Sign(cvar.pErr); //if you need to turnin place, then ignore PID
            }
            lastTime = thisTime;

            //inLastTarget = (lastDist < leaveTargetThresh);
            //approachingTarget = (cvar.targdist < approachingThresh);
            reachedTarget = (cvar.targdist < destinationThresh);

            return reachedTarget;
        }



    }
}
