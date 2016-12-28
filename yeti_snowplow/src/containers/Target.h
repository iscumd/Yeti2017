#include "ros/ros.h"

#include "LocationPoint.h"

class Target {
public:
	Target();
	Target(double _x, double _y, double _heading, int _dir, bool _PID, double _speed);
	~Target();

	enum dirType {
		forward = 1,
		backward = -1
	};

	dirType dir;
	LocationPoint location;
	bool PID;
	double speed;

	void Print();
};