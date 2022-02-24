#ifndef COORDINATE_H
#define COORDINATE_H

#include <iostream>

class coordinate{
	
	public:
		double x; // x-Coordinate
		double y; // y-Coordinate

		coordinate();
		coordinate(double value);
		coordinate(double ix, double iy);
		void showcoordinate();
		void set(coordinate inp);
		double dot(coordinate secondone);

};

#endif
