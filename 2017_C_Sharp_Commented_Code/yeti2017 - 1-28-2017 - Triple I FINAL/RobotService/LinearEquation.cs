using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Yeti2015.Hardware.Lidar;

namespace Yeti2015.RobotService
{
    public class LinearEquation
    {
        
        /// <summary>
        /// whe this line is calculate from 2 point we store the midpoint of those 2 points here  
        /// </summary>
        public LidarPoint midpoint
        {
            get;
            set;

        }
        public LidarPoint endpoint1
        {
            get;
            set;

        }
        public LidarPoint endpoint2
        {
            get;
            set;

        }

        public double Slope
        { 
            get; 
            set;
        }

        public double Intercept
        {
            get;
            set;
        }

        public LinearEquation(double m, double b)
        {
            Slope = m;
            Intercept = b;
        }

        public LinearEquation(LidarPoint point1, LidarPoint point2, bool perpendicular = false)
        {
            midpoint = new LidarPoint((point1.X + point2.X) / 2, (point1.Y + point2.Y) / 2);
            endpoint1 = point1;
            endpoint2 = point2;
            double pointSlope = (point2.Y - point1.Y) / (point2.X - point1.X);

            if (perpendicular)
            {
                Slope = -(1 / pointSlope);
            }
            else
            {
                Slope = pointSlope;
            }
            
            Intercept = midpoint.Y - (Slope * midpoint.X);
        }

        
        public LidarPoint Intersect(LinearEquation equation2)
        {
            if (this.Slope == equation2.Slope)
            {
                throw new InvalidOperationException("Equations are parallel.");
            }

            double X = -(equation2.Intercept - this.Intercept) / (equation2.Slope - this.Slope);
            double Y = ((this.Intercept * equation2.Slope) - (equation2.Intercept * this.Slope)) / (equation2.Slope - this.Slope);
            var pointToReturn = new LidarPoint(X, Y);
            return pointToReturn;
        }
    }
}
