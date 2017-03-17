using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using Yeti2015.Hardware.Lidar;
using Yeti2015.Hardware.Roboteq;
//using Yeti2015.Control;
namespace Yeti2015.RobotService
{
    public class Program
    {
     //Create Booleans for Autonomous MOde and turning the GUI on and off
        public static bool AutonomousEnabled = true;
        public static bool EnableGUICalls = true;

    //Create input and output ports, as well as items which are read into the program
        private static LidarController lidarController;
        private static MotorController motorController;
        private static List<LidarPoint> knownLandmarks;
        public static LocationPoint YetiLocation;

     //Initialization for Slip Detection
        public static int YetiHistorySize = 250;//160; //80;
        public static Queue<LocationPoint> YetiHistory;
        
    //Initialization for waypoint navigation
        private static List<Target> TargetLocationList;
        public static Target TargetLocation;
        private static Target PrevTargetLocation;

    //holds time inbetween scans for GUI
        private static Timer scanTimer = new Timer(ScanThread, null, Timeout.Infinite, 40);

     //Initialize events for the GUI
        public static event EventHandler NewLidarData;
        public static event EventHandler NewLandmarkData;
        public static event EventHandler NewObstacleData;
        public static event EventHandler NewLocationPoint;
        public static event EventHandler StallDetected;

    //Initialize Driving Proportion controllers
        const double turnBoost = -1.2; //-0.75;
        //const double turnBoost = -.5;
        const double maxSpeed = .7;// 1; //0.95; //0.85; //0.85; // 0.40

    //Initialize tolerance thresholds and time saving for Slip detection
        const double NoMovementTolerance = 0.1;// 10 cm
        const double NoRotationTolerance = 5.0; //5 degrees
        private static DateTime lastStuckTime; //save last stuck time
     //private static TimeSpan reverseDuration = new TimeSpan(0, 0, 0, 0, 500); // 0.5 Seconds
        private static TimeSpan reverseDuration = new TimeSpan(0, 0, 0, 0, 800); // 0.5 Seconds
        private static float reverseSpeed = -0.5f;

    //specify whicih COM port Roboteq Motor Controller is on
        static string motorPort = "COM6"; 

    //Create variables for speeds of each motor
        static float last_lSpeed = 0;
        static float last_rSpeed = 0;

    // For debugging
        private static bool keepCorrecting = false;
        static float debug1;

        public static void Main(string[] args)
        {        
        //Connect to LiDAR
            lidarController = new LidarController("192.168.0.100", 2111); // Real Robot

         //COnnect to Motor Controller
            motorController = new MotorController(motorPort);
            motorController.Connect();

        //initialize obstacle detection tolerances
            Obstacle.ObstacleInitialize(15, 150, 5, 200);//maxrarius of 15m, 

        //ImportLandmark Locations from Text file
            knownLandmarks = Localization.KnownLandmarksInit(@"..\..\landmarksDay2.txt");

        //Specify which XY Location the robot will start in the field at
            //YetiLocation = new LocationPoint(0.5, -1, 0);
           // YetiLocation = new LocationPoint(0.25, -1, 0);
            //YetiLocation = new LocationPoint(0.0, -1, 0);
            YetiLocation = new LocationPoint(0.0, 0, 0);

        //IMPORT the locations of the Waypoint navigation targets
            //TargetLocationList = Target.ReadLocationData(@"..\..\navigationQualification.txt");
            //TargetLocationList = Target.ReadLocationData(@"..\..\navigationDay1.txt");
            TargetLocationList = Target.ReadLocationData(@"..\..\navigationTripleIFinal.txt"); 
            TargetLocation = TargetLocationList[0];//set first target as first from txt file
            PrevTargetLocation = new Target();//initialize previous target to null

        //Initialize the Yeti Location history for Stall detection
            YetiHistory = new Queue<LocationPoint>(YetiHistorySize); //stall detection
            
        //this is flag which specifies if this is the first lidar scan. This is set to false after the first LiDAR scan
            keepCorrecting = true;//do once 
            
        //Main loop where all of the fun stuff happens
            while (AutonomousEnabled)
            {
                ScanThread();
            }
        }//end main

        
        private static void ScanThread(object sender = null)
        {
        //READ IN NEW LIDAR DATA                        
            int?[] lidarData = lidarController.ScanData();
            if (EnableGUICalls) { NewLidarData.Invoke(lidarData, EventArgs.Empty); }//print LiDAR data to GUI

        //create list of lidar points which actually returned a value.(This list of points will be smaller than 1081, because all points which did not return are not added to this list
            var lidarDataNonNull = lidarData.Select(x => x.HasValue ? x.Value : int.MaxValue).ToList();

            ////OutputToFile(lidarDataNonNull);//output LiDAR data to file if you want to replay it later

        //LOCALIZATION
            var currentLandmarks = Localization.ScanLandmarks(lidarData.ToLidarPoints(19000), knownLandmarks, YetiLocation);//find the landmarks
            YetiLocation = Localization.DetermineRobotLocation(currentLandmarks, YetiLocation, 0.001, last_lSpeed, last_rSpeed, (float)0.01);//find where robot is based of landmarks

        //FIND OBSTACLEs
            var myObstacles = Obstacle.FindObstacles(lidarDataNonNull, YetiLocation, lidarData.ToLidar1081Points(19000),currentLandmarks);//returns a list of obstacles
            var movingObstacles = (from p in myObstacles where p.moving == true select true).ToArray().FirstOrDefault();//checks if any of the found obstacles were moving.
            if (EnableGUICalls) { NewObstacleData.Invoke(myObstacles, EventArgs.Empty); }//print the obstacles to the GUI

        //SLIP Dection
            //check that the Queue which saves yeti location is full. Only Dequeue old location if it is full
            if (YetiHistory.Count > YetiHistorySize)
            {
                YetiHistory.Dequeue();// if it is full then remove the oldest location
            }
            YetiHistory.Enqueue(new LocationPoint(YetiLocation.X, YetiLocation.Y, YetiLocation.Heading));//add the newest location

            //check that the Queue which saves yeti location is full. Stall detection should only trigger if the queue is full!
            if (YetiHistory.Count >= YetiHistorySize)
            {
                //Find Maximum and Minimums of X, Y and Heading 
                var minX = YetiHistory.Min(item => item.X);
                var maxX = YetiHistory.Max(item => item.X);
                var minY = YetiHistory.Min(item => item.Y);
                var maxY = YetiHistory.Max(item => item.Y);
                var minHeading = YetiHistory.Min(item => item.Heading);
                var maxHeading = YetiHistory.Max(item => item.Heading);

                // if no XY movement, no rotation, and beyond first waypoint.Beyond first way point is key!!!! 
                //Otherwise yeti will back up at the beginning if you wait to long!
                if ((Math.Pow(maxX - minX, 2) + Math.Pow(maxY - minY, 2) < NoMovementTolerance) 
                            && maxHeading - minHeading < NoRotationTolerance && TargetLocation.location.id > 1 && !movingObstacles)
                {
                    lastStuckTime = DateTime.Now;//save the time which slipping was detected 
                    YetiHistory.Clear();// clear the QUeue so that stall detection cannot occur again until the queue is full
                    if (EnableGUICalls) { StallDetected.Invoke(null, EventArgs.Empty); }//print the Stall info to the GUI
                }
            }

        //Correct the list of known landmarks from the text file to reflect where the landmarks are actually seen in the field
            if (keepCorrecting)//if this is the first LiDAR scan
            {
                Localization.CorrectLandmarks(currentLandmarks, ref knownLandmarks); //inital landmark snapshot is corrected
                keepCorrecting = false;// flag to signal that the landmarks have been corrected, and don't need to be corrected again.
            }
            if (EnableGUICalls) { NewLandmarkData.Invoke(currentLandmarks, EventArgs.Empty); }//print the Landmark locations to the GUI

        //OBSTACLE AVOIDANCE
            Buffer.combinedUpdatePoints(lidarData.ToLidarPoints(19000));
            //Creates List of LiDAR points which have a positive Y value, and are within the Buffer distance threshold
            
            //look at angle needed to go to target waypoint, if there is an obstacle in the way, then find what turn angle is needed to avoid it to the right. 
            double Right = Buffer.combinedRightWheelScan(TargetLocation.location.ToLidarPoint(YetiLocation));
            //look at angle needed to go to target waypoint, if there is an obstacle in the way, then find what turn angle is needed to avoid it to the left. 
            double Left = Buffer.combinedLeftWheelScan(TargetLocation.location.ToLidarPoint(YetiLocation));
            
        //WAYPOINT NAVIGATION
            double turn = Control.cvar.turn;//create local variable, and initialized it to the last turn angle
            int dir = (int)TargetLocation.dir;//read in the direction(forward or backward) which was read in from the navigation waypoint text file

            //Navigate!
            if (Control.areWeThereYetAndTurnPID(YetiLocation, TargetLocation))//has Yeti arrived at the target waypoint?
            {//true when target is reached.
                //reached target, change target to next waypoint from waypoint navigation list
                Control.initGuide();//reset PD controller errors to 0
                PrevTargetLocation = TargetLocation;//update previous target the the just reached target

                //assign new target from the next waypoint in the list of waypoints
                var newTargetLocation = TargetLocationList.SingleOrDefault(x => x.location.id == PrevTargetLocation.location.id + 1);
                if (newTargetLocation != null) //if it is null, we've hit the last target so don't update the target
                {
                    TargetLocation = newTargetLocation;//otherwise go towards new target
                }
            }


        //DECIDING WHERE TO GO/TURN based off of found information
            //create local variables to assemble wheel speeds.
            double speed;
            float lSpeed;
            float rSpeed;

            if (movingObstacles)//if we see a movin obstacle, then stop no matter what
            {
                speed = 0;
                turn = 0;
                lSpeed = 0;
                rSpeed = 0;
            }
            else if (Right == 0 && Left == 0)// if there no obstacles in our way, then we are good to go and can turn to wherever waypoint navigation wants to go!
            {
                //no obsticals to avoid
                turn = Control.cvar.turn;
                //speed slower as turn steeper 
                speed = 1 / (1 + 1 * Math.Abs(turn)) * (double)dir;
                speed = (double)dir * Math.Min(Math.Abs(speed), 1.0);
                lSpeed = (float)((speed + turnBoost * turn) * maxSpeed * Control.cvar.speed);//controlvarspeed is read in from text file, and limits speed by a percentage
                rSpeed = (float)((speed - turnBoost * turn) * maxSpeed * Control.cvar.speed); 
            }
            else if (Right == Buffer.DOOM && Left == Buffer.DOOM )//There is no way to avoid anything to the left or the right, so back up.
            {
                lSpeed = reverseSpeed * (float)maxSpeed * (float) .25;
                rSpeed = reverseSpeed * (float)maxSpeed * (float) .25;
                Console.WriteLine("I reached DOOM!");
            }
            else // there is obstacles to avoid, so the avoidance turn angle which deviates least from the waypoint is selected
            {
                if(Math.Abs(Right - turn) <= Math.Abs(Left - turn))
                {
                    //move right of obstacle
                    turn = Right;
                }
                else if (Math.Abs(Right - turn) > Math.Abs(Left - turn))
                {
                    //move Left of obstacle
                    turn = Left;
                }
                //speed slower as turn steeper 
                speed = 1 / (1 + 1 * Math.Abs(turn)) * (double)dir;
                speed = (double)dir * Math.Min(Math.Abs(speed), 1.0);
                lSpeed = (float)((speed + turnBoost * .5 *  turn) * maxSpeed  * Control.cvar.speed);
                rSpeed = (float)((speed - turnBoost * .5* turn) * maxSpeed *   Control.cvar.speed);
            }

        //Send Wheel Speeds and Control Yeti
            //If we should still be backing up because of slip detection, then continue backing up. And double check we actually want to be moving.
            if (lastStuckTime.Add(reverseDuration).CompareTo(DateTime.Now) > 0 && Control.cvar.speed > 0) 
            {
                motorController.SetMotorValues(reverseSpeed * (float)maxSpeed, reverseSpeed * (float)maxSpeed);//go backwards
            }
            else//otherwise send wheel speeds to yeti
            {
                motorController.SetMotorValues(lSpeed, rSpeed);//use normal values
            }
            //save previous wheel speeds.
            last_lSpeed = lSpeed;
            last_rSpeed = rSpeed;

            //print motor speed information to GUI
            if (EnableGUICalls)
            {
                double[] dataToSend = { YetiLocation.X, YetiLocation.Y, /*debug1*/ YetiLocation.Heading, Left, Right, turn, lSpeed, rSpeed, currentLandmarks.Count, Control.cvar.pErr };
                NewLocationPoint.Invoke(dataToSend, EventArgs.Empty);
            }
         }//end Scan thread

        private static void OutputToFile(List<int> lidarDataNonNull)
        {
            var outputFile = new FileStream("lidarData2.txt", FileMode.Create);
            byte[] result = new byte[lidarDataNonNull.Count * sizeof (int)];
            System.Buffer.BlockCopy(lidarDataNonNull.ToArray(), 0, result, 0, result.Length);
            outputFile.Write(result, 0, result.Length);
            outputFile.Close();
        }//End output to file
    }//end class Program
}
