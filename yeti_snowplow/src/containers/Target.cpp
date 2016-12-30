#include "Target.h"

Target::Target(){
	location = LocationPoint(0, 0, 0);
	dir = forward;
	PID = true;
	speed = 0;
}

Target::Target(double _x, double _y, double _heading, int _dir, bool _PID, double _speed){
	location = LocationPoint(_x, _y, _heading);
	dir = static_cast<dirType>(_dir);
	PID = _PID;
	speed = _speed;
}

Target::~Target(){
}

void Target::Print(){
	ROS_INFO("Target #%i: %f %f %i %i %f", location.id, location.x, location.y, dir, PID, speed);
}