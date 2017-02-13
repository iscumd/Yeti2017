using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Yeti2015.Hardware.Lidar
{

    public class LidarPointCompare : IEqualityComparer<LidarPoint>
    {
        public bool Equals(LidarPoint x, LidarPoint y)
        {
            return x.id == y.id;
        }
        public int GetHashCode(LidarPoint codeh)
        {
            return codeh.id;
        }
    }
    
    public class LidarPoint
    {

        private double x;
        private double y;
        private double distance;
        private double headingCache;
        private bool headingCacheCalculated;

        public double correctedX;
        public double correctedY;

        public LidarPoint()
        {
            X = 0;
            Y = 0;
        }



        /// <summary>
        /// 
        /// </summary>
        /// <param name="_x">X Position / Radius</param>
        /// <param name="_y">Y Position / Angle in degrees</param>
        /// <param name="polarCoordinateForm"></param>
        public LidarPoint(double _x, double _y, bool polarCoordinateForm = false, double conversionFactor = 1)
        {
            if (polarCoordinateForm)
            {
                X = Math.Cos(_y * (Math.PI / 180.0)) * _x * conversionFactor;
                Y = Math.Sin(_y * (Math.PI / 180.0)) * _x * conversionFactor;
            }
            else
            {
                X = _x * conversionFactor;
                Y = _y * conversionFactor;
            }

            Distance = Math.Sqrt(Math.Pow(this.X, 2) + Math.Pow(this.Y, 2));
        }

        public LidarPoint(double _x, double _y, double _distance, bool polarCoordinateForm = false, double conversionFactor = 1)
        {
            if (polarCoordinateForm)
            {
                X = Math.Cos(_y * (Math.PI / 180.0)) * _x * conversionFactor;
                Y = Math.Sin(_y * (Math.PI / 180.0)) * _x * conversionFactor;
            }
            else
            {
                X = _x * conversionFactor;
                Y = _y * conversionFactor;
            }

            Distance = _distance;
        }

        public double DistanceBetween (LidarPoint p2)
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
                    headingCacheCalculated = false;
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
                    headingCacheCalculated = false;
                }
            }
        }

        public double Heading
        {
            get
            {
                if (!headingCacheCalculated)
                {
                    headingCache = Math.Atan2(this.X, this.Y);
                    headingCacheCalculated = true;
                }

               return headingCache;
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
                distance = value;
            }
        }

        public override string ToString()
        {
            return string.Format("({0}, {1}, {2})", this.X, this.Y, this.Distance);
        }
    }
}
