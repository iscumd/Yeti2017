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
       public double speed; // between -1 to 1
       public double turn; // between -1 to 1
      
        //PID CONTROL
        //u(t) = Kp * e(t) + Ki * Integral of e(t) + Kd * derivative of e(t)
        //cvar.turn = -(cvar.kP * Math.Sin(cvar.pErr) *2 + cvar.kI * cvar.iErr + cvar.kD * cvar.dErr);
            //Proportional
            //P accounts for present values of the error.For example, if the error is large 
            //and positive, the control output will also be large and positive.
            public double kP; // Proportional Term of PID
            public double pErr; // Current proportional Error
            public double lastpErr; //Last proportional Error

            //DERIVATIVE
            //D accounts for possible future trends of the error, based on its current rate of change.
            public double kD; //Derivative Term of PID
                    public double dErr; // calculated Derrivative Error

            //INTEGRAL
            //I accounts for past values of the error.For example, if the current 
            //output is not sufficiently strong, the integral of the error will 
            //accumulate over time, and the controller will respond by applying a stronger action.
                public double iErr;// 
                public double kI; // Integral Term
    }
}
