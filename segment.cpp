#include "segment.h"

using namespace std;

segment::segment(){
	rb.x=0.0; // r beginning
	rb.y=0.0; 
	re.x=0.0; // r end
	re.y=0.0;
	segtype='o';
}


segment::segment(coordinate irb, coordinate ire){
	rb.set(irb);
	re.set(ire);
	segtype = 'o';
}


segment::segment(double xb, double yb, double xe, double ye){
	rb.x=xb;
	rb.y=yb;
	re.x=xe;
	re.y=ye;
	segtype = 'o';
}


segment::segment(coordinate irb, coordinate ire, char isegtype){
	rb.set(irb);
	re.set(ire);
	segtype = isegtype;
}


segment::segment(double xb, double yb, double xe, double ye, char isegtype){
	rb.x=xb;
	rb.y=yb;
	re.x=xe;
	re.y=ye;
	segtype = isegtype;
}


void segment::set(coordinate irb, coordinate ire){
	rb.set(irb);
	re.set(ire);                                                                                                                     
	segtype = 'o';
}


void segment::set(double xb, double yb, double xe, double ye){
	rb.x=xb;
	rb.y=yb;
	re.x=xe;
	re.y=ye;
	segtype = 'o';
}


void segment::set(coordinate irb, coordinate ire, char isegtype){
	rb.set(irb);
	re.set(ire);                                                                                                                        
	segtype = isegtype;
}


void segment::set(double xb, double yb, double xe, double ye, char isegtype){
	rb.x=xb;
	rb.y=yb;
	re.x=xe;
	re.y=ye;
	segtype = isegtype;
}

