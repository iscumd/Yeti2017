using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Yeti2015.Hardware.Lidar;

namespace Yeti2015.RobotService
{
    public static class Localization
    {
        static private LocationPoint previousRobotLocation = new LocationPoint();//this holds where the robot previously was so that the determine robot location can start its approach from this location
        static int LM_POINTS_THRESH = 3;// this is the minimum number of LiDAR points needed for a landmark to be considered a landmark
       

        public static LocationPoint DetermineRobotLocation(List<LocationPoint> CLM, LocationPoint robotLocation, double tolerance, float Lspeed, float Rspeed, float minSpeed)
        {
            int JMAX = 15;

            double mu = 0.1; // (Only read)

            ////LocationPoint previousRobotLocation = new LocationPoint(robotLocation.X, robotLocation.Y, robotLocation.Heading);
            ////double px = 0.0;  // Previous Robot X
            ////double pt = 0.0;  // Previous Robot Theta 
            ////double py = 0.0;  // Previous Robot Y
            double maxx = 0.25;
            double minx = 0.0001;
            double maxy = 0.25;
            double miny = 0.0001;
            double maxt = 5.0 * (Math.PI / 180.0);
            double mint = 0.1 * (Math.PI / 180.0);

            bool updateh = false;
            double[,] H = new double[3, 3]; // Hessian of 2nd Derivatives
            double[,] H_lm = new double[3, 3];
            double[,] H_inv = new double[3, 3];  // Matrix Inverse of H_lm
            double x_err = 0;  // Gradient E [0]
            double y_err = 0;  // Gradient E [1] 
            double t_err = 0;  // Gradient E [2]
            double ex;  // Landmark error in X
            double ey;  // Landmark error in Y
            double dx;  // Robot Delta X
            double dy;  // Robot Delta Y 
            double dt;  // Robot Delta Theta
            double det;  // Determinant of H_lm

            var thisRobotLocation = new LocationPoint(robotLocation.X, robotLocation.Y, robotLocation.Heading);
            ////double tx = robotLocation.X;  // Robot X
            ////double ty = robotLocation.Y;  // Robot Y
            ////double tt = robotLocation.Heading;  // Robot Theta
            double errsqrd = 0.0;  // E
            double derr = 0.0;
            double lasterrsqrd = 99999.0;//was 0.0
            double lambda = 10.0;

            /*Set Hessian To Zero*/
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    H[i, j] = H_lm[i, j] = H_inv[i, j] = 0;
                }
            }

            /*Initalize Diag*/
            for (int j = 0; j < JMAX; j++)
            {
                errsqrd = 0;
                derr = 0;

                if (j == 0)
                {
                    updateh = true;

                    //foreach (var currentLandmark in CLM)
                    //{
                    //    ex = currentLandmark.X - thisRobotLocation.X - currentLandmark.Distance * Math.Sin(thisRobotLocation.Heading + currentLandmark.Heading);  // Formula 4
                    //    ey = currentLandmark.Y - thisRobotLocation.Y - currentLandmark.Distance * Math.Cos(thisRobotLocation.Heading + currentLandmark.Heading);  // Formula 5
                    lasterrsqrd = 99999.0; //Math.Pow(ex, 2) + Math.Pow(ey, 2);  // Formula 3 (Inside Summation)
                    //}
                }

                if (updateh)
                {
                    for (int i = 0; i < 3; i++)
                        for (int k = 0; k < 3; k++)
                            H[i, k] = 0.0;

                    H[0, 0] = H[1, 1] = H[2, 2] = mu * CLM.Count;  // Formula 21
                    x_err = mu * CLM.Count * (previousRobotLocation.X - thisRobotLocation.X);
                    y_err = mu * CLM.Count * (previousRobotLocation.Y - thisRobotLocation.Y);
                    t_err = mu * CLM.Count * (previousRobotLocation.Heading - thisRobotLocation.Heading);

                    foreach (var currentLandmark in CLM)
                    {
                        /*First Derivative*/
                        ex = currentLandmark.X - thisRobotLocation.X - 
                            currentLandmark.Distance * Math.Sin(thisRobotLocation.Heading + currentLandmark.Heading);  // Formula 4
                        ey = currentLandmark.Y - thisRobotLocation.Y 
                            - currentLandmark.Distance * Math.Cos(thisRobotLocation.Heading + currentLandmark.Heading);  // Formula 5

                        currentLandmark.correctedX = currentLandmark.X - ex;
                        currentLandmark.correctedY = currentLandmark.Y - ey;

                        derr += Math.Abs(ex) +Math.Abs( ey);
                        errsqrd += Math.Pow(ex, 2) + Math.Pow(ey, 2);  // Formula 3
                        x_err += ex;
                        y_err += ey;
                        t_err += ex * (currentLandmark.Distance * Math.Cos(thisRobotLocation.Heading + currentLandmark.Heading)) - ey * (currentLandmark.Distance * Math.Sin(thisRobotLocation.Heading + currentLandmark.Heading));

                        H[0, 0] += 1;  // Formula 9.2, Formula 21
                        H[0, 1] += 0;  // Formula 10.2, Formula 21
                        H[0, 2] += (currentLandmark.Distance * Math.Cos(thisRobotLocation.Heading + currentLandmark.Heading));  // Formula 11.2, Formula 22
                        H[1, 0] += 0;  // Formula 12.2, Formula 21
                        H[1, 1] += 1;  // Formula 13.2,, Formula 21
                        H[1, 2] += -(currentLandmark.Distance * Math.Sin(thisRobotLocation.Heading + currentLandmark.Heading));  // Formula 14.2, Formula 23
                        H[2, 0] += (currentLandmark.Distance * Math.Cos(thisRobotLocation.Heading + currentLandmark.Heading));  // Formula 15.2, Formula 22
                        H[2, 1] += -(currentLandmark.Distance * Math.Sin(thisRobotLocation.Heading + currentLandmark.Heading));  // Formula 16.2, Formula 23
                        H[2, 2] += Math.Pow(currentLandmark.Distance, 2);  // Formula 17.2, Formula 24
                    }
                }

                for (int i = 0; i < 3; i++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        H_lm[i, k] = H[i, k];

                        if (i == k)
                        {
                            H_lm[i, k] += lambda * 1.0;
                        }
                    }
                }

                det = H_lm[0, 0] * (H_lm[2, 2] * H_lm[1, 1] - H_lm[2, 1] * H_lm[1, 2]) -
                    H_lm[1, 0] * (H_lm[2, 2] * H_lm[0, 1] - H_lm[2, 1] * H_lm[0, 2]) +
                    H_lm[2, 0] * (H_lm[1, 2] * H_lm[0, 1] - H_lm[1, 1] * H_lm[0, 2]);


                // Find inverse of H_lm
                H_inv[0, 0] = (H_lm[2, 2] * H_lm[1, 1] - H_lm[2, 1] * H_lm[1, 2]) / det;
                H_inv[0, 1] = -(H_lm[2, 2] * H_lm[0, 1] - H_lm[2, 1] * H_lm[0, 2]) / det;
                H_inv[0, 2] = (H_lm[1, 2] * H_lm[0, 1] - H_lm[1, 1] * H_lm[0, 2]) / det;
                H_inv[1, 0] = -(H_lm[2, 2] * H_lm[1, 0] - H_lm[2, 0] * H_lm[1, 2]) / det;
                H_inv[1, 1] = (H_lm[2, 2] * H_lm[0, 0] - H_lm[2, 0] * H_lm[0, 2]) / det;
                H_inv[1, 2] = -(H_lm[1, 2] * H_lm[0, 0] - H_lm[1, 0] * H_lm[0, 2]) / det;
                H_inv[2, 0] = (H_lm[2, 1] * H_lm[1, 0] - H_lm[2, 0] * H_lm[1, 1]) / det;
                H_inv[2, 1] = -(H_lm[2, 1] * H_lm[0, 0] - H_lm[2, 0] * H_lm[0, 1]) / det;
                H_inv[2, 2] = (H_lm[1, 1] * H_lm[0, 0] - H_lm[1, 0] * H_lm[0, 1]) / det;


                /* Update Here*/
                /* Hessian Inverse times Gradiant*/
                dx = (H_inv[0, 0] * x_err) + (H_inv[0, 1] * y_err) + (H_inv[0, 2] * t_err);  // Formula 25 (After plus)
                dy = (H_inv[1, 0] * x_err) + (H_inv[1, 1] * y_err) + (H_inv[1, 2] * t_err);  // Formula 25 (After plus)
                dt = (H_inv[2, 0] * x_err) + (H_inv[2, 1] * y_err) + (H_inv[2, 2] * t_err);  // Formula 25 (After plus)


                if (Math.Abs(derr) < tolerance)
                {
                    j = JMAX;  // break;
                }

                if (errsqrd < lasterrsqrd)
                {
                    updateh = true;
                    lambda /= 10;
                    lasterrsqrd = errsqrd;
                    thisRobotLocation.X += dx;  // Formula 25
                    thisRobotLocation.Y += dy;  // Formula 25
                    thisRobotLocation.Heading += dt;  // Formula 25
                }
                else
                {
                    updateh = false;
                    lambda *= 10;
                }

                //Console.WriteLine("x=%f\ty=%f\tt=%f\td=%02f\r", thisRobotLocation.X, thisRobotLocation.Y, thisRobotLocation.Heading, derr);
            }

            previousRobotLocation.X = thisRobotLocation.X;
            previousRobotLocation.Y = thisRobotLocation.Y;
            previousRobotLocation.Heading = thisRobotLocation.Heading; 

            return thisRobotLocation; // Skip the rest of stuff
       
            LocationPoint newRobotLocation = new LocationPoint(robotLocation.X, robotLocation.Y, robotLocation.Heading);
            if (Math.Abs(Lspeed) > minSpeed && Math.Abs(Rspeed) > minSpeed)
            {
                double deltaX = thisRobotLocation.X - previousRobotLocation.X;
                double deltaY = thisRobotLocation.Y - previousRobotLocation.Y;
                double deltaHeading = thisRobotLocation.Heading - previousRobotLocation.Heading;
                if (Math.Abs(thisRobotLocation.X - previousRobotLocation.X) < maxx && Math.Abs(thisRobotLocation.Y - previousRobotLocation.Y) < maxy && Math.Abs(thisRobotLocation.Heading - previousRobotLocation.Heading) < maxt)
                {
                    if (Math.Abs(deltaX) > minx)
                    {
                ////        ////*rx = previousRobotLocation.X = thisRobotLocation.X; 
                        previousRobotLocation.X = thisRobotLocation.X; 
                        newRobotLocation.X = thisRobotLocation.X;
                    }


                    if (Math.Abs(deltaY) > miny)
                    {

                        ////        ////*ry = previousRobotLocation.Y = thisRobotLocation.Y;
                        previousRobotLocation.Y = thisRobotLocation.Y;
                        newRobotLocation.Y = thisRobotLocation.Y;
                    }


                    if (Math.Abs(deltaHeading) > mint)
                    {
                ////        ////*rt = previousRobotLocation.Heading = ADJUST_RADIANS(thisRobotLocation.Heading);

                        double adjustedTT = ((thisRobotLocation.Heading + Math.PI) % (2 * Math.PI)) - Math.PI; // Adjusting to fit between -pi and pi.
                        newRobotLocation.Heading = adjustedTT;
                        previousRobotLocation.Heading = adjustedTT;

                    }

                    return newRobotLocation;
                }

            }




            return robotLocation; 
        }

        //Scan landmarks is used to find all of the surrounding landmark locations. To do this the LiDAR data is needed, the list of known landmark locations
        //from a text file are needed, and the robots current location (to convert each point in the LiDAR data to an XY coordinate in the field).
        public static List<LocationPoint> ScanLandmarks(List<LidarPoint> LMSdata, List<LidarPoint> KLM, LocationPoint robotLocation)
        {
            var currentLandmarks = new List<LocationPoint>();//anytime a landmark is found in a scan the landmark is saved to this arrray

            foreach (var knownLandmark in KLM)//loop through each known landmark in the list of imported landmarks
            {
                //smallestSeperation is the maximum distance a point can be from where a landmark should be to be considered part of the landmark
                //double smallestSeparation = 0.55;  ////landmark_tolerance;
                double smallestSeparation = 0.45;  ////landmark_tolerance;
                //double smallestSeparation = 1;  ////landmark_tolerance;
                //double smallestSeparation = 1.75;
                
                LocationPoint matchedLandmark = null;// create a point to match the landmark too if it is found
                
                //The following variables must be reset to zero at the beginning of each landmark search.
                double Xsum = 0, ysum = 0, headingsum = 0;//everytime a point is found to be part of a landmark the X, Y heading
                // are summed together, then divded by number of points found to find the X, Y and heading averages for the points associated with the landmark
                int found_points = 0;//save the number of points that become associated with  the landmark
                int twiceDist_points = 0; //used for debuging

                //loop through every point in the LiDAR scan
                foreach (var currentPoint in LMSdata)
                {
                    if (currentPoint == null)//check if the LiDAR point did not return because there was nothing for the laser to bounce off of
                        continue;//forget about this point

                    var landmarkLocationPoint = currentPoint.ToLocationPoint(robotLocation);// convert the current point we are looking at
                    //into a point in the XY field based off of where the point is in the scan and where the robot is in the field.

                    // figure out the difference between where the landmark should be and where the landmark is in the X coordinate
                    double xSeparation = knownLandmark.X - landmarkLocationPoint.X;
                    // figure out the difference between where the landmark should be and where the landmark is in the Y coordinate 
                    double ySeparation = knownLandmark.Y - landmarkLocationPoint.Y;
                    
                    // figure out the distance between the LiDAR point we are looking at compared to where
                    double currentSeparation = Math.Sqrt(Math.Pow(xSeparation, 2) + Math.Pow(ySeparation, 2));
                    //the landmark should be

                    // if the seperation is beneath the set tolerance. IE the point is close enough to the landmarks expected position to be considered part
                    if (currentSeparation < smallestSeparation)
                        //of the landmark
                    {
                        Xsum +=currentPoint.X;//add the point to the Xsum
                        ysum +=currentPoint.Y;//add the point to the Ysum
                        headingsum += currentPoint.Heading;//add the point to the heading sum
                        found_points++;//increment the number of points found
                    }
                }//end loop through LiDAR points


                //now check if the points which were associated with the landmark were enough to be considered a landmark
                // if the number of points is low (less than thresh), then this likely isnt a landmark and it should not be assumed that this is a landmark
                if (found_points > LM_POINTS_THRESH) 
                {
                    //This is considered a landmark!
                        double XAverage = Xsum/ (double)found_points;//find average of all X values of the points associated with the landmark
                        double YAverage = ysum/ (double)found_points;//find average of all Y values of the points associated with the landmark
                        double headingAverage = headingsum/ (double)found_points;//find average of all heading values of the points associated with the landmark
                        double distanceAverage = Math.Sqrt(XAverage*XAverage + YAverage*YAverage);//find average distance from the robot to the landmark point
                    if (distanceAverage < 15)  ////MAX_LANDMARKS_DISTANCE)// check if the landmark is less than 15M away
                    {
                        matchedLandmark = new LocationPoint();// we have a new landmark!
                        matchedLandmark.X = knownLandmark.X;//save the landmarks X value
                        matchedLandmark.Y = knownLandmark.Y;//save the landmarks Y value

                        //the next part is saving the distance between the robot and the landmark.
                        //40% is the distance average, and 60% is the distance from the point & the robot
                        //this is done because neither of the distance vaues are exactly correct due to the fact that
                        //the landmark locations are not exactly at the center of the landmark(as they should be)
                        //so it is assumed that the distance from the selected point is slightly more correct 
                        //than the distance average(which is the sum of the distancese from all of the found points)
                        //40% of the distance from the landmark is the distance average
                        matchedLandmark.Distance = 0.4 * distanceAverage;
                        //60% of the distance is the distance from the found landmarks position from the robot
                        matchedLandmark.Distance += 0.6 * robotLocation.DistanceBetween(matchedLandmark); 
                        matchedLandmark.Heading = headingAverage;// heading average is saved
                        currentLandmarks.Add(matchedLandmark);//yay we have a new landmark so add it to our list of landmarks!
                        continue;
                    }
                }
            }//end for each landmark loops
            return currentLandmarks;//yay here are all of our landmarks!
        }

        public static List<LidarPoint> KnownLandmarksInit(string fileName){
            float tempx, tempy;
            int u = 0;

            StreamReader landmarkFile = new StreamReader(fileName);

            List<LidarPoint> knownLandmarks = new List<LidarPoint>();

            while (!landmarkFile.EndOfStream)
            {
                string currentLine = landmarkFile.ReadLine();
                string[] currentLineStrings = currentLine.Split(new []{' '}, StringSplitOptions.RemoveEmptyEntries);

                double currentXValue = Convert.ToDouble(currentLineStrings[0]);
                double currentYValue = Convert.ToDouble(currentLineStrings[1]);

                var currentLidarPoint = new LidarPoint(currentXValue, currentYValue);

                knownLandmarks.Add(currentLidarPoint);
            }

            landmarkFile.Close();

            return knownLandmarks;


            ////FILE *fp;
            ////if ((fp = fopen(fileName, "r")) == null) {
            ////    fprintf(stderr, "Could not open landmark file\n");
            ////}
            ////numLandmarks = 0;
            ////while (!feof(fp)) {
            ////    fscanf(fp, "%f %f", &tempx, &tempy);
            ////    KLM[u].x = tempx*M2MM;
            ////    KLM[u].y = tempy*M2MM;
            ////    KLM[u].found = false;
            ////    KLM[u].rejected = false;
            ////    KLM[u].phi = atan2(KLM[u].x, KLM[u].y);
            ////    KLM[u].dist=D(KLM[u].x, KLM[u].y);
            ////    u++;
            ////}
            ////num_landmarks = u;
            ////fclose(fp);
        }



      
        ////}

        /// <summary>
        /// inital landmark scan correction
        /// </summary>
        /// <param name="CLM">landmarks found in lidar scan</param>
        /// <param name="KLM">landmarks found in text file</param>
        public static void CorrectLandmarks(List<LocationPoint> CLM, ref List<LidarPoint> KLM)
        {
            //loop through each landmark found in the list of known landmarks which were read in from a file
            foreach (var knownLandmark in KLM)
            {
                double minMatchDist = 15;// the inital (purposefully large) distance between a KLM and a CLM. This is updated
                //to the distance between the CLM and the KLM each time the distance is smaller than any previous distance.

                double maxSeperationDist = .5;// the maximum distance between the KLM and CLM for a the CLM to be asscocaited with the KLM
                LocationPoint closestLandmark = null; // create local temporary landmark

                //look through landmarks that were found in the LiDAR scan in attempt to associate one to the list of known landmarks
                foreach (var currentLandmark in CLM)
                {
                    //calculate the distance between the current known landmark to the current found landmark
                    double dist_KLM__to_CLM = Math.Sqrt( Math.Pow(knownLandmark.X - currentLandmark.X,2) + Math.Pow(knownLandmark.Y - currentLandmark.Y,2));

                    //if the distance between the CLM and the KLM is smaller than the smallest distance found between the current KLM and any other CLMs
                    if (dist_KLM__to_CLM < minMatchDist)
                    {
                        //if the distance between the CLM and the KLM is below the seperation threshold
                        if (dist_KLM__to_CLM < maxSeperationDist)
                        {
                            minMatchDist = dist_KLM__to_CLM;// update the matched distance to the distance from this CLM and this KLM
                            closestLandmark = currentLandmark;// update the closest landmark to this CLM
                        }
                    }
                }
                if (closestLandmark != null)//if a CLM was found close enough to a KLM and is associated to it.
                {
                    knownLandmark.X = closestLandmark.correctedX;//update the KLM X value
                    knownLandmark.Y = closestLandmark.correctedY;//Update the KLM Y value
                }
                else { //dont update, keep landmark the same
                }
            }//end CLM loop
        }//emd KLM loop
    }//endCorrect Landmarks
}//end Localization
