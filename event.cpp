#include "event.h"

static constexpr double minfinity = -100000000; // 10^-12

using namespace std;

event::event(){
	angle=minfinity;
	seg = nullptr;
	obstInd=-1;
}


event::event(double iangle, segment *isegment, int obstacle_index){
	angle=iangle;
	seg = isegment;
	obstInd=obstacle_index;
}

