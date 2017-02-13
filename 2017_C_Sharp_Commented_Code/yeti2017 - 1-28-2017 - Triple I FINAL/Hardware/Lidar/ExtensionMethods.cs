using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Yeti2015.Hardware.Lidar
{
    public static class ExtensionMethods
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="radii"></param>
        /// <returns></returns>
        public static List<LidarPoint> ToLidarPoints(this int?[] radii, int maxDistanceFromOrigin = 8000)
        {
            var lidarPoints = new List<LidarPoint>();

            for (int i = 0; i < radii.Length; i++)
            {
                if (radii[i] != null && radii[i] < maxDistanceFromOrigin)
                {
                    var point = new LidarPoint(radii[i].Value / 1000.0, (i / 4.0) - 45.0, true);
                    point.id = i;
                    lidarPoints.Add(point); 
                }
            }

            return lidarPoints;
        }

        public static List<LidarPoint> ToLidar1081Points(this int?[] radii, int maxDistanceFromOrigin = 8000)
        {
            var lidarPoints = new List<LidarPoint>();

            for (int i = 0; i < radii.Length; i++)
            {
                if (radii[i] == null)
                {
                    var point = new LidarPoint(20, (i / 4.0) - 45.0, true);
                    point.id = i; 
                    lidarPoints.Add(point);
                }
                else
                {
                    var point = new LidarPoint(radii[i].Value / 1000.0, (i / 4.0) - 45.0, true);
                    point.id = i;
                    lidarPoints.Add(point);
                }
            }

            return lidarPoints;
        }
    }
}
