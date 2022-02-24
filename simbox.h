#ifndef SIMBOX_H
#define SIMBOX_H

#include <vector>
#include <random>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "obstacle.h"
#include "coordinate.h"
#include "segment.h"

class SimBox {

	private:
		double Lx;				// dimensions of box
		double Ly;
		double nObstacles;	// no. of obstacles
		double wObstacle;    // dimensions of obstacles
		double lObstacle;
	
	public:
		std::vector<segment> boundaries;			// box boundaries
		std::vector<Obstacle> obstacle_list;	// obstacle list
		SimBox();
		SimBox(double iL);
		SimBox(double iLx, double iLy);
		SimBox(double iLx, double iLy, int inObstacles, double iwObstacle, double ilObstacle, int randomseed);
		double getLx();
		double getLy();
		int getnObstacles();
		void print(std::string iname);

};

#endif
