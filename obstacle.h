#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <fstream>
#include "segment.h"
#include "coordinate.h"

/* the cross-shaped obstacles that the tracer particles collide with.
   each is given by 12 segments, a center of origin, and an angular orientation*/

class Obstacle {
	
	public:
		std::vector<segment> segs;		// obstacle segments
		coordinate center;				// center coordinate
		double sigma;						// angular orientation
		
		Obstacle();
		Obstacle(double w, double l, double centerx, double centery);
		coordinate getCenter();
		void setCenter(double cx, double cy);
		double getSigma();
		void setSigma(double insigma);
		void manipulate(double dx, double dy, double dtheta);//this is something
		void print();
		void print(std::string iname);  

};

#endif
