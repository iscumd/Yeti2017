using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.Runtime.InteropServices;



namespace Yeti2015.RobotService
{
    public class Target
    {

       public enum dirType
       {
           forward = 1,
           backward = -1
       }

      
        
      public dirType dir;
      public LocationPoint location;
      public bool PID;
      public double speed;

      public Target()
      {
          dir = dirType.forward;
          location = new LocationPoint(); 

          PID = true;
          speed = 0;

      }

      public Target(LocationPoint _location, dirType _dir = dirType.forward, bool _PID = true, double _speed = 0)
      {
          dir = _dir;
          location = _location;
          PID = _PID;
          speed = _speed;
  
      }

      public Target(LocationPoint _location, dirType _dir, int _PID, double _speed)
      {
          dir = _dir;
          location = _location;
          PID = (_PID > 0);
          speed = _speed;

      }

      public Target(LocationPoint _location, int _dir, bool _PID = true, double _speed = 0)
      {
          if (_dir >= 0)
              dir = dirType.forward;
          else if (_dir < 0)
              dir = dirType.backward;
          
          location = _location;

          PID = _PID;
          speed = _speed;
  
      }

      public Target(LocationPoint _location, int _dir, int _PID, double _speed)
      {
          if (_dir >= 0)
              dir = dirType.forward;
          else if (_dir < 0)
              dir = dirType.backward;

          location = _location;

          PID = (_PID > 0);
          speed = _speed;

      }


      public Target(double _x, double _y, double _heading, dirType _dir = dirType.forward, bool _PID = true, double _speed = 0)
      {
          dir = _dir;
          location = new LocationPoint(_x, _y, _heading);
          PID = _PID;
          speed = _speed;
  
      }

      public Target(double _x, double _y, double _heading, dirType _dir, int _PID, double _speed)
      {
          dir = _dir;
          location = new LocationPoint(_x, _y, _heading);
          PID = (_PID > 0) ;
          speed = _speed;

      }

      public Target(double _x, double _y, double _heading, int _dir, bool _PID = true, double _speed = 0)
      {
          if (_dir > 0)
              dir = dirType.forward;
          else if (_dir <= 0)
              dir = dirType.backward;
          location = new LocationPoint(_x, _y, _heading);
          PID = _PID;
          speed = _speed;
  
      }

      public Target(double _x, double _y, double _heading, int _dir, int _PID, double _speed)
      {
          if (_dir >= 0)
              dir = dirType.forward;
          else if (_dir < 0)
              dir = dirType.backward;
          location = new LocationPoint(_x, _y, _heading);
          PID = (_PID > 0);
          speed = _speed;

      }

      public Target(double _x, double _y, dirType _dir = dirType.forward, bool _PID = true, double _speed = 0)
      {
          dir = _dir;
          location = new LocationPoint(_x, _y, 0);
          PID = _PID ;
          speed = _speed;
      }
      public Target(double _x, double _y, dirType _dir, int _PID, double _speed)
      {
          dir = _dir;
          location = new LocationPoint(_x, _y, 0);
          PID = (_PID >0);
          speed = _speed;
      }


      public Target(double _x, double _y, int _dir, bool _PID = true, double _speed = 0)
      {
          if (_dir >= 0)
              dir = dirType.forward;
          else if (_dir < 0)
              dir = dirType.backward;
          location = new LocationPoint(_x, _y,0);
          PID = _PID;
          speed = _speed;
      
      }

      public Target(double _x, double _y, int _dir, int _PID, double _speed)
      {
          if (_dir >= 0)
              dir = dirType.forward;
          else if (_dir < 0)
              dir = dirType.backward;
          location = new LocationPoint(_x, _y, 0);
          PID = (_PID > 0);
          speed = _speed;

      }
        public static List<Target> ReadLocationData(string fileName)
        {
            StreamReader navigationFile = new StreamReader(fileName);

            List<Target> navigationPoints = new List<Target>();

            int pointCount = 0;

            while (!navigationFile.EndOfStream)
            {
                string currentLine = navigationFile.ReadLine();
                string[] currentLineStrings = currentLine.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

                if (currentLineStrings.Length == 5)
                {
                    double currentXValue = Convert.ToDouble(currentLineStrings[0]);
                    double currentYValue = Convert.ToDouble(currentLineStrings[1]);
                    int currentDirection = Convert.ToInt32(currentLineStrings[2]);
                    int currentPID = Convert.ToInt32(currentLineStrings[3]);
                    double currentSpeed = Convert.ToDouble(currentLineStrings[4]);

                    var currentLidarPoint = new Target(currentXValue, currentYValue, 0.0, currentDirection, (currentPID >0), currentSpeed);
                    currentLidarPoint.location.id = pointCount;
                    pointCount++;

                    navigationPoints.Add(currentLidarPoint);
                }
            }

            navigationFile.Close();

            return navigationPoints;
        }
    }

}

