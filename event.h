#ifndef EVENT_H
#define EVENT_H

#include <iostream>
#include "segment.h"

// collision "events"

class event{

	public:
		double angle; 	// angular distance to collision
		segment* seg;	// segment to collide with
		int obstInd;	// index of corresponding obstacle
		
		event();
		event(double iangle, segment* isegmentID, int obstacle_index);

};

#endif
