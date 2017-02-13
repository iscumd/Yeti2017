using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Yeti2015.Hardware.Lidar;

namespace Yeti2015.RobotService
{

    public class BucketSizeComparer : IComparer<intersectBucket>
    {
        public int Compare(intersectBucket a, intersectBucket b)
        {
            if (a.points.Count() == b.points.Count())
                return 0;
            if (a.points.Count() < b.points.Count())
                return -1;

            return 1;
        }
    }
    
    
    
    public class intersectBucket
    {

        private static double my_maxDistanceFromMidpoint;


        public static double maxDistanceFromMidpoint
        {
            get
            {
                return my_maxDistanceFromMidpoint;
            }
            set
            {
                if (value <= 0)
                    throw new Exception("maxDistanceFromMidpoint must e greater then 0");

                my_maxDistanceFromMidpoint = value;
            }
        }


        public intersectBucket(LidarPoint firstPoint)
        {
            points = new List<LidarPoint>();
            points.Add(firstPoint);
            my_midpoint = firstPoint;
        }

        public bool tryAddPoint(LidarPoint p)
        {
            if (p.DistanceBetween(midpoint) <= maxDistanceFromMidpoint)
            { 

                my_midpoint = new LidarPoint((points.Count() * midpoint.X + p.X)/(points.Count()+1) ,(points.Count() * midpoint.Y + p.Y)/(points.Count()+1) );
                points.Add(p);
                return true;
            }
            else
            {
                return false;
            }


        }

        private void AddPoint(LidarPoint p)
        {
            
                my_midpoint = new LidarPoint((points.Count() * midpoint.X + p.X) / (points.Count() + 1), (points.Count() * midpoint.Y + p.Y) / (points.Count() + 1));
                points.Add(p);

        }
        
        public List<LidarPoint> points;

        private LidarPoint my_midpoint; 

        public LidarPoint midpoint
        {

            get
            {
                return my_midpoint;
            }
        }

        public override string ToString()
        {
            return string.Format("({0}, {1})", midpoint.X, midpoint.Y);
        }

        


        /// <summary>
        /// geerates the set itersectio. that is the poits commo to oth uckets
        /// </summary>
        /// <param name="Bucket1"></param>
        /// <param name="Bucket2"></param>
        /// <returns></returns>
        
        static private List<LidarPoint> setItersection(intersectBucket Bucket1, intersectBucket Bucket2)
        {
            return Bucket1.points.Intersect(Bucket2.points, new LidarPointCompare()).ToList();
            /*
            List<LidarPoint> returvalue = new List<LidarPoint>();
            foreach(LidarPoint p1 in Bucket1.points)
            {
                foreach (LidarPoint p2 in Bucket2.points)
                {
                    if (p1.id == p2.id)
                    {
                        returvalue.Add(p1);
                        break;
                    }


                }
            }

            return returvalue;
    */
        }

        static private List<LidarPoint> setSubtraction(intersectBucket LeftBucket, intersectBucket rightBucket)
        {


            return LeftBucket.points.Except(rightBucket.points, new LidarPointCompare()).ToList();

            /*
            List<LidarPoint> returvalue = new List<LidarPoint>();
            foreach (LidarPoint p1 in LeftBucket.points)
            {
                bool found = false;
                foreach (LidarPoint p2 in rightBucket.points)
                {
                     
                    if (p1.id == p2.id)
                    {
                        found = true;
                        break;
                    }

                    if (!found)
                    {
                        returvalue.Add(p1);
                    }
                }
            }

            return returvalue;
             */
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="Bucket2"></param>
        /// <returns>an numBer Between 0 and 100. 0 -> no point in common. 100 -> the smaller list is a suset of the larger list. Other value is a percetage of commonality</returns>

        public static int compare (intersectBucket Bucket1, intersectBucket Bucket2)
        {

            

            List<LidarPoint> itersection = setItersection(Bucket1, Bucket2);
            if (Bucket1.points.Count() < Bucket2.points.Count())
            {
                return 100 * itersection.Count() / Bucket1.points.Count();

            }
            else
            {
                return 100 * itersection.Count() / Bucket2.points.Count();
            
            }

            /*
              Unable to cast object of type '<IntersectIterator>d__92`1[Yeti2015.Hardware.Lidar.LidarPoint]' to type 'System.Collections.Generic.List`1[Yeti2015.Hardware.Lidar.LidarPoint]'.
             */

        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="passedList"></param>
        /// <param name="index"></param>
        /// <param name="minCommonality"></param>
        /// <returns>the intersectBucket is created By comBing the passedList[idex] with all passedList[index + i] that have a min percentage points ie common. the int is the first idex nOT comined     </returns>
        private static Tuple<intersectBucket, int> combinefoward(List<intersectBucket> passedList, int index, int minCommonality)
        {
            intersectBucket returnBucket = passedList[index];
            int i = index +1;
            int consecutiveMisses = 0;
            for (i = index +1; consecutiveMisses <= 3 && i < passedList.Count(); i++)
            {
                int comparevalue =  compare(returnBucket, passedList[i]);
                if (comparevalue > minCommonality)
                {
                    List<LidarPoint> pointsToAdd = setSubtraction( passedList[i], returnBucket);
                    foreach (LidarPoint thispoint in pointsToAdd)
                    {
                        returnBucket.AddPoint(thispoint);
                    }
                    consecutiveMisses = 0;
                }
                else
                {
                    consecutiveMisses++;
                }

            }
            return new Tuple<intersectBucket, int>(returnBucket, i - consecutiveMisses);  

        }

        /// <summary>
        /// takes the iitial list of intersectBuckets and comBines the ones with nearly the same poits     
        /// </summary>
        /// <param name="passedList">iitial list of intersectBuckets</param>
        /// <param name="minCommonality">a value from 1 - 100 idicatig the min % Commonality the lists must have to Be comBined</param>
        /// <returns>a cosolidated list of intersectBucket </returns>
        public static List<intersectBucket> consolidate(List<intersectBucket> passedList, int minCommonality = 50)
        {
            
            List<intersectBucket> returnList = new List<intersectBucket>();
            
            
            int i = 0;
            while (i < passedList.Count)
            {

                Tuple<intersectBucket, int> combinefoward_result = combinefoward(passedList, i, minCommonality);
                returnList.Add(combinefoward_result.Item1);
                i = combinefoward_result.Item2;

            }
            return returnList;


        }

    }
}
