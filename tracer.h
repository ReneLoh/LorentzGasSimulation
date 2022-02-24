#ifndef TRACER_H
#define TRACER_H

#include <cmath>
#include <iostream>
#include <algorithm>
#include <string>
#include "simbox.h"
#include "coordinate.h"
#include "segment.h"
#include "event.h"

class tracer {
	
	private:
		coordinate ro; 	// origin of the gyration circle
		double R;  			// radius of the gyration circle
		double theta; 		// angular phase
		int nx;				// in which image of the periodic box?
		int ny;

	
	public:
		double epsilon;	// used for "epsilon trick" (see thesis)
		bool engaged;		// is the tracer engaged with obstacles, or trapped in free circular motion?
		
		tracer();
		tracer(coordinate iro,double iR, double itheta);
		tracer(double xo, double yo,double iR, double itheta);
		coordinate getro();
		double getr();
		double getangle();
		void setro(coordinate value);
		void setro(double xvalue, double yvalue);
		void setr(double value);
		void setangle(double value);
		void show();
		coordinate getpos(double help_angle=0);
		coordinate getpos_unfolded(SimBox* simulationbox, double help_angle=0);
		void showxy();
		double signum(double x);
		double find_nextcollision_single_seg(segment seg);
		event find_nextcollision_single_simbox(SimBox* simulationbox);
		double anglefinder(double thetatarget, double currenttheta);
		void move(double someangle);
		bool safemove(event* nextevent);
		void collide(segment* seg, SimBox* simbox);
		coordinate getvelocity(double help_angle=0);
		double velocitydot(double vectorx,double vectory);
		void refindangle(coordinate currentposition);
		bool isinside(Obstacle iobstacle); //returns true if the tracer is inside the obstacle */
		bool is_engaged(tracer input_tracer, double delta_angle, SimBox* simulationbox);
		bool isinsidesimbox(SimBox* simbox);
		double distance2(tracer* othertracer,SimBox* simulationbox);

};

#endif
