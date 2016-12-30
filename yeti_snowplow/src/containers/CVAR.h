class CVAR {
public:
	CVAR();
	~CVAR();

    double targdist;
    double targbearing;
    double front;
    double right;
    double speed;               // between -1 to 1
    double turn;                // between -1 to 1
    double pErr;
    double lastpErr;
    double kP;
    double dErr;
    double kD;
    double iErr;
    double kI;
    double lookAhead;
};