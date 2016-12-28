#include "LocationPoint.h"

LocationPoint::LocationPoint(){
}

LocationPoint::LocationPoint(double _x, double _y, double _heading){
	x = _x;
	y = _y;
	distance = sqrt(pow(x, 2) + pow(y, 2));
	heading = _heading;
}

LocationPoint::~LocationPoint(){
}