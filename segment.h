#ifndef SEGMENT_H
#define SEGMENT_H

#include <iostream>
#include "coordinate.h"

class segment{
  
	public:
		coordinate rb; // r beginning 
		coordinate re; // r end
		char segtype; // defines which sort of segment it is. standard options are o: obstacle, t: top wall, b: bottom wall, l: left wall, r: right wall
		
		segment();
		segment(coordinate irb, coordinate ire);
		segment(double xb, double yb, double xe, double ye);
		void set(coordinate irb, coordinate ire);
		void set(double xb, double yb, double xe, double ye);
		segment(coordinate irb, coordinate ire, char isegtype);
		segment(double xb, double yb, double xe, double ye, char isegtype);
		void set(coordinate irb, coordinate ire, char isegtype);
		void set(double xb, double yb, double xe, double ye, char isegtype);

};

#endif

