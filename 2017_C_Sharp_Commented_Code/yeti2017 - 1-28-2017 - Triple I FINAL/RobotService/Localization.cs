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



        /// <summary>
        /// //The brains of Yeti!
        //Localizatino revolves around this function entirely, which is by far the most complicated thing about yeti,
        // There are really three necassary things for this to work:CLM, Robot Location, Tolerance
        /// </summary>
        /// <param name="CLM">list of landmarks which the robot currently sees</param>
        /// <param name="robotLocation">the previously calculated position of the robot</param>
        /// <param name="tolerance">the tolerance of the derivative of change in sum of error squared to be considered a local minimum</param>
        /// <param name="Lspeed">not needed, but is used to determine if the robot is trying to move</param>
        /// <param name="Rspeed">not needed, but is used to determine if the robot is trying to move</param>
        /// <param name="minSpeed">not needed, but is used to determine if the robot is trying to move</param>
        /// <returns></returns>
        public static LocationPoint DetermineRobotLocation(List<LocationPoint> CLM, LocationPoint robotLocation, double tolerance, float Lspeed, float Rspeed, float minSpeed)
        {
            int JMAX = 15;//Maximum number of attempts to converge on a local minimum of the robots location

            double mu = 0.1; //initial learning rate(used in Gradient Descent)

            bool updateh = false;//flag for it the hessian matrix should be updated
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

            //initially use the last robots location as the intial guess of where the robot is.
            var thisRobotLocation = new LocationPoint(robotLocation.X, robotLocation.Y, robotLocation.Heading);
            
            double errsqrd = 0.0;  // Sum of Errors Squared
            double derr = 0.0; // Sum of Errors (not squared)
            double lasterrsqrd = 99999.0;//inital error is set to infinity.
            double lambda = 10.0;// Levenburg Marquardt step Value

            /*INitialize Hessian Matrix To Zero*/
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    H[i, j] = H_lm[i, j] = H_inv[i, j] = 0;
                }
            }

            /*Attempt to Converge on robot location*/
            for (int j = 0; j < JMAX; j++)
            {
                errsqrd = 0; // reset the error squared for the new position which is to be calculated
                derr = 0; // reset the errors derivative.

                if (j == 0)// on first attempt to converge
                {
                    updateh = true;// it is necassary to update the hessian matrix
                    lasterrsqrd = 99999.0; //initial error squared is set to inifinity so that first attempt at convergence is accepted
                }

                if (updateh) //update the hessian matrix(IE Calculate new position entirely)
                {
                    //reset the hessian matrix to zeros
                    for (int i = 0; i < 3; i++)
                        for (int k = 0; k < 3; k++)
                            H[i, k] = 0.0;

                    //place learning rate on diagonal of the hessian
                    H[0, 0] = H[1, 1] = H[2, 2] = mu * CLM.Count;  

                    //initialize error to non-zero (further away robot positions should be punished a little more heavily than previous
                    x_err = mu * CLM.Count * (previousRobotLocation.X - thisRobotLocation.X);
                    y_err = mu * CLM.Count * (previousRobotLocation.Y - thisRobotLocation.Y);
                    t_err = mu * CLM.Count * (previousRobotLocation.Heading - thisRobotLocation.Heading);

                    foreach (var currentLandmark in CLM)//loop through each landmark yeti can see
                    {
                        //figure out the cost in x and cost in y
                        ex = currentLandmark.X - thisRobotLocation.X - 
                            currentLandmark.Distance * Math.Sin(thisRobotLocation.Heading + currentLandmark.Heading);  // this is delta X for landmark i (equation 2 in ION GNSS Paper)
                        ey = currentLandmark.Y - thisRobotLocation.Y 
                            - currentLandmark.Distance * Math.Cos(thisRobotLocation.Heading + currentLandmark.Heading);  //this is delta y for landmark i(equation 3 in ION GNSS Paper)

                        //saving corrected landmark locations for correct landmarks
                        currentLandmark.correctedX = currentLandmark.X - ex;// save the correct X coordinate of the landmark for the correct landmarks function
                        currentLandmark.correctedY = currentLandmark.Y - ey;// save the correct Y coordinate of the landmark for the correct landmarks function. 
                        //Note that this is for this robot location, which may be rejected.
                        
                        derr += Math.Abs(ex) +Math.Abs( ey);// add the errors in both coordinates
                        errsqrd += Math.Pow(ex, 2) + Math.Pow(ey, 2);  // add this error to the sum of error squared. (equation 1 in ION GNSS Paper)
                        
                        x_err += ex;//increment the total error in the X coordinate for this robot location.
                        y_err += ey;//increment the total erro rin the Y coordinate for this robot location
                        //increment the total error in of theta
                        t_err += ex * (currentLandmark.Distance * Math.Cos(thisRobotLocation.Heading + currentLandmark.Heading)) -
                                ey * (currentLandmark.Distance * Math.Sin(thisRobotLocation.Heading + currentLandmark.Heading));

                        //Updating the Hessian Matrix
                        H[0, 0] += 1;  // (equation 9 in ION GNSS Paper)
                        H[0, 1] += 0;  // (equation 10 in ION GNSS Paper)
                        H[0, 2] += (currentLandmark.Distance * Math.Cos(thisRobotLocation.Heading + currentLandmark.Heading));  // (equation 11 in ION GNSS Paper)
                        H[1, 0] += 0;  // (equation 12 in ION GNSS Paper)
                        H[1, 1] += 1;  // (equation 13 in ION GNSS Paper)
                        H[1, 2] += -(currentLandmark.Distance * Math.Sin(thisRobotLocation.Heading + currentLandmark.Heading));  // (equation 14 in ION GNSS Paper)
                        H[2, 0] += (currentLandmark.Distance * Math.Cos(thisRobotLocation.Heading + currentLandmark.Heading));  // (equation 15 in ION GNSS Paper)
                        H[2, 1] += -(currentLandmark.Distance * Math.Sin(thisRobotLocation.Heading + currentLandmark.Heading));  // (equation 16 in ION GNSS Paper)
                        H[2, 2] += Math.Pow(currentLandmark.Distance, 2);  // Formula 17
                    }//end for each CLM
                }//end update h

                // calculating the Matrix H * lamba(Diagonal Matrix). (necassary for equation 19 in ION GNSS Paper) 
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

                //finding the determinant , which is used to calculate the inverse of the Matrix H * lamba(Diagonal Matrix)
                det = H_lm[0, 0] * (H_lm[2, 2] * H_lm[1, 1] - H_lm[2, 1] * H_lm[1, 2]) -
                    H_lm[1, 0] * (H_lm[2, 2] * H_lm[0, 1] - H_lm[2, 1] * H_lm[0, 2]) +
                    H_lm[2, 0] * (H_lm[1, 2] * H_lm[0, 1] - H_lm[1, 1] * H_lm[0, 2]);


                // Find inverse of Matrix H * lamba(Diagonal Matrix). (Part of equation 19 in ION GNSS Paper)
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
                /* Hessian Inverse times Gradiant! (equation 19 in ION GNSS Paper)*/
                // this is the change in the robot location from the intial guess used.
                dx = (H_inv[0, 0] * x_err) + (H_inv[0, 1] * y_err) + (H_inv[0, 2] * t_err);  // (equation 19 in ION GNSS Paper)
                dy = (H_inv[1, 0] * x_err) + (H_inv[1, 1] * y_err) + (H_inv[1, 2] * t_err);  // (equation 19 in ION GNSS Paper)
                dt = (H_inv[2, 0] * x_err) + (H_inv[2, 1] * y_err) + (H_inv[2, 2] * t_err);  // (equation 19 in ION GNSS Paper)

                // if the error(not squared) is less than tolerance, meaning the position guess is almost exactly correct, so there is no need to converge further.
                if (Math.Abs(derr) < tolerance)
                {
                    j = JMAX;  // break;
                }

                //compared this robot location guess sum of error squared to the last
                if (errsqrd < lasterrsqrd)
                {
                    //if this position has better error than the last
                    updateh = true;// if it does, then recalculate the hessian matrix for a new position.
                    lambda /= 10;// divide lambda by 10, and slow the learning rate to converge on a local minimum slower.
                    lasterrsqrd = errsqrd;// save the error squared so it can be compared to next iteration.
                    thisRobotLocation.X += dx;  // (equation 19 in ION GNSS Paper)
                    thisRobotLocation.Y += dy;  // (equation 19 in ION GNSS Paper)
                    thisRobotLocation.Heading += dt;  // (equation 19 in ION GNSS Paper)
                }
                else// this position has more error than the last
                {
                    updateh = false;//do not calculate new hessian matrix
                    lambda *= 10;//just increase lambda
                }
            }//end Convergence attempts. (J=MAX or Derivative of Error is 0).

            previousRobotLocation.X = thisRobotLocation.X;
            previousRobotLocation.Y = thisRobotLocation.Y;
            previousRobotLocation.Heading = thisRobotLocation.Heading; 

            return thisRobotLocation; 
        }//end determine robot location

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
