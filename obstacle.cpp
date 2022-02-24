#include "obstacle.h"

using namespace std;

Obstacle::Obstacle(){
	segs.resize(12);
	double w = 0;
	double l = 0;
	double offsety = 0;
	double offsetx = 0;
	center.x = 0;
	center.y = 0;
	sigma = 0; 
    
	segs[0].set(-1. * w + offsetx, l + offsety, w + offsetx, l + offsety);
	segs[1].set(w + offsetx, l + offsety, w + offsetx, w + offsety);
	segs[2].set(w + offsetx, w + offsety, l + offsetx, w + offsety);
	segs[3].set(l + offsetx, w + offsety, l + offsetx, -1. * w + offsety);
	segs[4].set(l + offsetx, -1. * w + offsety, w + offsetx, -1. * w + offsety);
	segs[5].set(w + offsetx, -1. * w + offsety, w + offsetx, -1. * l + offsety);
	segs[6].set(w + offsetx, -1. * l + offsety, -1. * w + offsetx, -1. * l + offsety);
	segs[7].set(-1. * w + offsetx, -1. * l + offsety, -1. * w + offsetx, -1. * w + offsety);
	segs[8].set(-1. * w + offsetx, -1. * w + offsety, -1. * l + offsetx, -1. * w + offsety);
	segs[9].set(-1. * l + offsetx, -1. * w + offsety, -1. * l + offsetx, w + offsety);
	segs[10].set(-1. * l + offsetx, w + offsety, -1. * w + offsetx, w + offsety);
	segs[11].set(-1. * w + offsetx, w + offsety, -1. * w + offsetx, l + offsety);
}


Obstacle::Obstacle(double w, double l, double offsetx, double offsety){
	segs.resize(12);
	center.x = offsetx;
	center.y = offsety;
	sigma = sqrt(l*l + w*w);
	
	segs[0].set( -1. * w + offsetx, l + offsety, w + offsetx, l + offsety);
	segs[1].set(w + offsetx, l + offsety, w + offsetx, w + offsety);
	segs[2].set(w + offsetx, w + offsety, l + offsetx, w + offsety);
	segs[3].set(l + offsetx, w + offsety, l + offsetx, -1. * w + offsety);
	segs[4].set(l + offsetx, -1. * w + offsety, w + offsetx, -1. * w + offsety);
	segs[5].set(w + offsetx, -1. * w + offsety, w + offsetx, -1. * l + offsety);
	segs[6].set(w + offsetx, -1. * l + offsety, -1. * w + offsetx, -1. * l + offsety);
	segs[7].set(-1. * w + offsetx, -1. * l + offsety, -1. * w + offsetx, -1. * w + offsety);
	segs[8].set(-1. * w + offsetx, -1. * w + offsety, -1. * l + offsetx, -1. * w + offsety);
	segs[9].set(-1. * l + offsetx, -1. * w + offsety, -1. * l + offsetx, w + offsety);
	segs[10].set(-1. * l + offsetx, w + offsety, -1. * w + offsetx, w + offsety);
	segs[11].set(-1. * w + offsetx, w + offsety, -1. * w + offsetx, l + offsety);
}


void Obstacle::manipulate(double dx, double dy, double dtheta){
		
	double xb,yb,xe,ye;
	center.x += dx;
	center.y += dy;
	
	for (int i=0; i < segs.size(); ++i){
		xb = segs.at(i).rb.x; /*later clean up this mess, use coordinate instead"*/
		yb = segs.at(i).rb.y;
		xe = segs.at(i).re.x;
		ye = segs.at(i).re.y;

		/*rotation*/
		segs.at(i).rb.x = xb*cos(dtheta) - yb*sin(dtheta);
 		segs.at(i).rb.y = xb*sin(dtheta) + yb*cos(dtheta);
		segs.at(i).re.x = xe*cos(dtheta) - ye*sin(dtheta);
		segs.at(i).re.y = xe*sin(dtheta) + ye*cos(dtheta);

		/*translation*/
		segs.at(i).rb.x += dx;
		segs.at(i).rb.y += dy;
		segs.at(i).re.x += dx;
		segs.at(i).re.y += dy;
	}
}


coordinate Obstacle::getCenter(){
	return center;
}


void Obstacle::setCenter(double cx, double cy){
	center.x = cx;
	center.y = cy;
}


void Obstacle::setSigma(double insigma){
	sigma = insigma;
}


double Obstacle::getSigma(){
	return sigma;
}


void Obstacle::print(){  
	for (segment iseg:segs){
		cout << iseg.rb.x <<" "<< iseg.rb.y << "\n";
		cout << iseg.re.x <<" "<< iseg.re.y << "\n";
	}
	cout<<endl;// a blank line after each obstacle
}  


void Obstacle::print(std::string iname){  
	std::ofstream outputfile;
	outputfile.open (iname, std::ofstream::out | std::ofstream::app);

	for (segment iseg:segs){
		outputfile << std::fixed << setprecision(15)<<iseg.rb.x <<" "<< iseg.rb.y << "\n";
		outputfile << std::fixed << setprecision(15)<<iseg.re.x <<" "<< iseg.re.y << "\n";
	}
	outputfile<<endl;// a blank line after each obstacle
	outputfile.close();
}  
