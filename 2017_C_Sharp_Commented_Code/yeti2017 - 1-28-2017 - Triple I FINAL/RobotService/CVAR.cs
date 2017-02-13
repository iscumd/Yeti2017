using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Yeti2015.RobotService
{
    class CVAR
    {
       public double targdist;
       public double targbearing;
       public double front;
       public double right;
       public double speed;               // between -1 to 1
       public double turn;                // between -1 to 1
       public double pErr;
       public double lastpErr;
       public double kP;
       public double dErr;
       public double kD;
       public double iErr;
       public double kI;
       public double lookAhead;
    }
}
