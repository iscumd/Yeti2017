using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Yeti2015.RobotService
{
    public class LocationPoint
    {

        private double x;
        private double y;
        private double distance;
        private double heading;

        public double correctedX;
        public double correctedY;

        public LocationPoint()
        {
            X = 0;
            Y = 0;
            Distance = 0;
            Heading = 0;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="_x">X Position / Radius</param>
        /// <param name="_y">Y Position / Angle in degrees</param>
        /// <param name="polarCoordinateForm"></param>
        public LocationPoint(double _x, double _y, double _heading)
        {
            X = _x;
            Y = _y;
            Distance = Math.Sqrt(Math.Pow(X, 2) + Math.Pow(Y, 2));
            Heading = _heading;
        }

        public LocationPoint(double _x, double _y, double _distance, double _heading)
        {
            X = _x;
            Y = _y;
            Distance = _distance;
            Heading = _heading;
        }

        public double DistanceBetween(LocationPoint p2)
        {
            return (Math.Sqrt((this.X - p2.X) * (this.X - p2.X) + (this.Y - p2.Y) * (this.Y - p2.Y))  );
        }

        public int id
        {
            get;
            set;
        } 

        public double X 
        {
            get
            {
                return x;
            }
            set
            {
                if (value != x)
                {
                    x = value;
                }
            }
        }

       public double Y
        {
            get
            {
                return y;
            }
            set
            {
                if (value != y)
                {
                    y = value;
                }
            }
        }

       public double Distance
       {
           get
           {
               return distance;
           }
           set
           {
               if (value != distance)
               {
                   distance = value;
               }
           }
       }

       public double Heading
        {
            get
            {
               return heading;
            }
            set
            {
                if (value != heading)
                {
                    heading = value;
                }
            }
        }

        public override string ToString()
        {
            return string.Format("({0}, {1}, {2})", this.X, this.Y, this.heading);
        }

        public static List<LocationPoint> ReadLocationData(string fileName)
        {
            StreamReader navigationFile = new StreamReader(fileName);

            List<LocationPoint> navigationPoints = new List<LocationPoint>();

            int pointCount = 0;

            while (!navigationFile.EndOfStream)
            {
                string currentLine = navigationFile.ReadLine();
                string[] currentLineStrings = currentLine.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

                if (currentLineStrings.Length == 5)
                {
                    double currentXValue = Convert.ToDouble(currentLineStrings[0]);
                    double currentYValue = Convert.ToDouble(currentLineStrings[1]);
                    ////int currentDirection = Convert.ToInt32(currentLineStrings[2]);
                    ////int currentPID = Convert.ToInt32(currentLineStrings[3]);
                    ////int currentSpeed = Convert.ToInt32(currentLineStrings[4]);

                    var currentLidarPoint = new LocationPoint(currentXValue, currentYValue, 0);
                    currentLidarPoint.id = pointCount;
                    pointCount++;

                    navigationPoints.Add(currentLidarPoint);
                }
            }

            navigationFile.Close();

            return navigationPoints;
        }
    }
}
