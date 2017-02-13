using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Yeti2015.Hardware.Lidar;

namespace Yeti2015.RobotService
{
    public static class ExtensionMethods
    {
        public static LocationPoint ToLocationPoint(this LidarPoint initialPoint, LocationPoint robotLocation)
        {
            double headingValue = initialPoint.Heading + robotLocation.Heading;
            double xValue = robotLocation.X + initialPoint.Distance * Math.Sin(headingValue);
            double yValue = robotLocation.Y + initialPoint.Distance * Math.Cos(headingValue);

            return new LocationPoint(xValue, yValue, headingValue);
        }

        public static LidarPoint ToLidarPoint(this LocationPoint finalPoint, LocationPoint robotLocation)
        {
            double headingValue = finalPoint.Heading - robotLocation.Heading;
            double distance = (finalPoint.X - robotLocation.X) / Math.Sin(finalPoint.Heading);

            if (double.IsInfinity(distance))
            {
                distance = (finalPoint.Y - robotLocation.Y) / Math.Cos(finalPoint.Heading);
            }

            double xValue = distance * Math.Sin(headingValue);
            double yValue = distance * Math.Cos(headingValue);

            return new LidarPoint(xValue, yValue, distance);
        }
    }
}
