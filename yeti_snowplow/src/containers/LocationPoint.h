#include <math.h>

class LocationPoint {
public:
	LocationPoint();
	LocationPoint(double _x, double _y, double _heading);
	~LocationPoint();

	double x;
	double y;
	double distance;
	double heading;

	double correctedX;
	double correctedY;

	int id;
};