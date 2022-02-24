#include "tracer.h"

using namespace std;

static constexpr double pi = 3.1415926535;   
static constexpr double pi2 = 6.28318530718;    
static constexpr double pihalf = 1.5707963268;    
static constexpr double infinity = 1e8; 
static constexpr double minfinity = -1e8; 

tracer::tracer(){
	ro.x = 0.0; 
	ro.y = 0.0;
	R = 0.0; 		
	theta = 0.0; 	
	engaged = false; 
	nx=ny=0;				
	epsilon = 1e-9;	
}


tracer::tracer(coordinate iro,double iR, double itheta){
	ro.x = iro.x;
	ro.y = iro.y;
	R=iR;
	theta=itheta;
	engaged = false;
	nx=ny=0;
	epsilon = 1e-9;
}


tracer::tracer(double xo, double yo,double iR, double itheta){
	ro.x = xo;
	ro.y = yo;
	R=iR;
	theta=itheta;
	engaged = false;
	nx=ny=0;
	epsilon = 1e-9;
}


coordinate tracer::getro(){
	return ro;
}


double tracer::getr(){ 
	return R;
}


double tracer::getangle(){
	return theta - floor( theta/pi2 )*pi2;
}


void tracer::setro(coordinate value){
	ro.x = value.x;
	ro.y = value.y;
}


void tracer::setro(double xvalue, double yvalue){
	ro.x = xvalue;
	ro.y = yvalue;
}


void tracer::setr(double value){
	R=value;
}


void tracer::setangle(double value){
	theta = value - floor(value/pi2)*pi2;
}


void tracer::show(){
	cout << ro.x<<" "<<ro.y<<" "<<R<<" "<<theta<< "\n";
}


coordinate tracer::getpos(double help_angle){
	coordinate temp(ro.x+R*cos(theta+help_angle),ro.y+R*sin(theta+help_angle));
	return temp;
}

coordinate tracer::getpos_unfolded(SimBox* simulationbox, double help_angle){ 																				//gives unfolded position after moving by help_angle
	coordinate temp(ro.x+nx*simulationbox->getLx()+R*cos(theta+help_angle), ro.y+ny*simulationbox->getLy()+R*sin(theta+help_angle));
	return temp;
}


void tracer::showxy(){
	coordinate temp;
	temp=getpos();
	cout << temp.x <<" "<<temp.y<< "\n";
}


coordinate tracer::getvelocity(double help_angle){ 											//gives velocity after moving by help_angle
	coordinate temp(cos(theta+help_angle+pihalf), sin(theta+help_angle+pihalf));
	return temp;
}
  
  
double  tracer::velocitydot(double vectorx, double vectory){
	return vectorx*cos(theta+pihalf)+vectory*sin(theta+pihalf);
}
  
  
double tracer::anglefinder(double thetatarget, double currenttheta){
	// finds how far should the tracer move to reach the target from current;

	return (thetatarget-currenttheta>0) ? thetatarget-currenttheta : pi2 + thetatarget-currenttheta ;
}


void tracer::move(double inputangle){
	if (inputangle < 0) {cerr << "negative angle in move()?!"<<endl;} else { 
	theta = theta + inputangle;
	theta = theta - floor(theta/pi2)*pi2;
	if(!(theta<pi2 && theta>0)){cerr << "angle out of bounds in move()!?" << endl;}
	}
}


bool tracer::safemove(event *nextevent){
	/* checks whether sign of scalar product changes between two collisions to see
      whether tracer got trapped within an obstacle after last collision (see thesis) */

	if (nextevent->angle < 0) {cerr << "negative angle in safemove()?!" << endl; } 
	else {
		bool result=true;    
		double dx=(nextevent->seg->re.x-nextevent->seg->rb.x);
		double dy=(nextevent->seg->re.y-nextevent->seg->rb.y);
		double nx=dy;
		double ny=-1.0*dx;
		double help_before=velocitydot(nx,ny);
		theta = theta + nextevent->angle;
		theta = theta - floor(theta/pi2)*pi2;
		if(!(theta<pi2 && theta>0)){cerr << "angle out of bounds in safemove()?!" << endl;}
		double help_after=velocitydot(nx,ny);
		if (help_before*help_after < 0) result=true;
		else result=false;
		
		return result;	
	}
	
}

void tracer::refindangle(coordinate currentposition){
	double deltax=currentposition.x-ro.x;
	double deltay=currentposition.y-ro.y;
	double cosv = (deltax)/sqrt(deltax*deltax+deltay*deltay);
	double sinv = (deltay)/sqrt(deltax*deltax+deltay*deltay);
	if (sinv>0){
		theta = acos(cosv);
	}
	else{
		theta = pi2 - acos(cosv);
	}
}


void tracer::collide(segment* seg, SimBox* simbox){ 
	// assigns new center of rotation and the corresponding new angle upon hard collision with a segment
   
   double eps_min = 1e-10;
	
	switch (seg->segtype){ 

		case 'o':{															// this is an "obstacle" segment, i.e. perform hard collision
			double lx, ly, neworiginx, neworiginy, helpQ;
			lx = seg->re.x - seg->rb.x;
			ly = seg->re.y - seg->rb.y;
			coordinate q(R*cos(theta),R*sin(theta));
			coordinate position_before_collision = getpos();
			helpQ = (q.x*lx+q.y*ly)/(lx*lx+ly*ly);
    
			neworiginx = ro.x + 2*helpQ*lx;
			neworiginy = ro.y + 2*helpQ*ly;
			ro.x = neworiginx;
			ro.y = neworiginy;
			refindangle(position_before_collision);
			break;
		}

		case 'r':{															// this is right-hand box wall, i.e. execute periodic boundaries (same for left, top, bottom)
			double help_theta = theta;
			double help_rox = ro.x;			
			bool inObstacle = true;
			bool outsideSimbox = false;
			while(inObstacle && epsilon > eps_min){ // epsilon condition prevents infinite loop
				move(epsilon);
				ro.x -= simbox->getLx();
				
				// check wether tracer ended up in an obstacle
				for(int obst_id=0; obst_id < simbox->obstacle_list.size(); ++obst_id){
					if(simbox->obstacle_list.at(obst_id).getCenter().x < (-1) * simbox->getLx()/2 + 2*epsilon + simbox->obstacle_list.at(obst_id).getSigma()){  // only consider close obstacles	
						if(isinside(simbox->obstacle_list.at(obst_id))){
							theta = help_theta;
							ro.x = help_rox;
							epsilon = 0.9*epsilon;
							inObstacle = true;							
							break;	
						}			
					}
					inObstacle = false;			
				}	
			
			}
						
			if(inObstacle){ 
				epsilon = epsilon/0.9;
				move(epsilon);
				ro.x -= simbox->getLx();
				cout << "# tracer ended up in obstacle after colliding with righthand wall!" << endl; 
			}	
			nx++;
			break;
		}

		case 'l':{
			double help_theta = theta;
			double help_rox = ro.x;	
			bool inObstacle = true;
			bool outsideSimbox = false;
						
			while(inObstacle && epsilon > eps_min){	
				move(epsilon);
				ro.x += simbox->getLx();
				for(int obst_id=0; obst_id < simbox->obstacle_list.size(); ++obst_id){
					if(simbox->obstacle_list.at(obst_id).getCenter().x > simbox->getLx()/2 - 2*epsilon - simbox->obstacle_list.at(obst_id).getSigma()){			
						if(isinside(simbox->obstacle_list.at(obst_id))){
							theta = help_theta;
							ro.x = help_rox;
							epsilon = 0.9*epsilon;
							inObstacle = true;							
							break;								
						}			
					}
					inObstacle = false;									
				}
			}	
							
			if(inObstacle){
				epsilon = epsilon/0.9;
				move(epsilon);
				ro.x += simbox->getLx();				
				cout << "# tracer ended up in obstacle after colliding with lefthand wall!" << endl;
			}	
			nx--;
			break;
		}

		case 'b':{
			double help_theta = theta;
			double help_roy = ro.y;		
			bool inObstacle = true;
			bool outsideSimbox = false;
			
			while(inObstacle && epsilon > eps_min){						
				move(epsilon);		
				ro.y += simbox->getLy();
				for(int obst_id=0; obst_id < simbox->obstacle_list.size(); ++obst_id){
					if(simbox->obstacle_list.at(obst_id).getCenter().y > simbox->getLy()/2 - 2*epsilon - simbox->obstacle_list.at(obst_id).getSigma()){			
						if(isinside(simbox->obstacle_list.at(obst_id))){						
							theta = help_theta;
							ro.y = help_roy;
							epsilon = 0.9*epsilon;
							inObstacle = true;							
							break;	
						}				
					}	
					inObstacle = false;		
				}
			}	
					
			if(inObstacle){
				epsilon = epsilon/0.9;
				move(epsilon);
				ro.y += simbox->getLy();				
				cout << "# tracer ended up in obstacle after colliding with bottom wall!" << endl;
			}	
			ny--;

			break;
		}

		case 't':{
			double help_theta = theta;
			double help_roy = ro.y;			
			bool inObstacle = true;
			bool outsideSimbox = false;
						
			while(inObstacle && epsilon > eps_min){							
				move(epsilon);
				ro.y -= simbox->getLy();
				for(int obst_id=0; obst_id < simbox->obstacle_list.size(); ++obst_id){
					if(simbox->obstacle_list.at(obst_id).getCenter().y < (-1) * simbox->getLy()/2 + 2*epsilon + simbox->obstacle_list.at(obst_id).getSigma()){			
						if(isinside(simbox->obstacle_list.at(obst_id))){
							theta = help_theta;
							ro.y = help_roy;
							epsilon = 0.9*epsilon;
							inObstacle = true;
							break;	
						}			
					}
					inObstacle = false;				
				}
			}
		
			if(inObstacle){
				epsilon = epsilon/0.9;
				move(epsilon);
				ro.y -= simbox->getLy();
				cout << "# tracer ended up in obstacle after colliding with top wall!" << endl;
			}	
			ny++;
			break;
		}

		default:{
			cerr << "# collision with unknown segment type!" << endl;
			return;
		}
	}  // end of switch

}


event tracer::find_nextcollision_single_simbox(SimBox* simulationbox){
	/* this routine searches for the next collision, i.e. event, of the tracer */

	double anghelp = infinity;
	Obstacle* cobst; 			//current obstacle
	event nextevent;
	double ang, dx, dy, ds1, ds2, dr;
	
		for (int obstacleID=0; obstacleID<simulationbox->obstacle_list.size(); ++obstacleID){ // loop obstacles in the simulation box
		
			cobst=&(simulationbox->obstacle_list.at(obstacleID)); 									  // ptr to the current obstacle
			dx = ro.x - cobst->getCenter().x;
			dy = ro.y - cobst->getCenter().y;
			dr = sqrt(dx*dx + dy*dy);
	
			if(dr <= R + cobst->getSigma()   &&   dr >= R - cobst->getSigma()){ // if obstacle is close enough to be considered
				
				for (int segID=0; segID < cobst->segs.size(); ++segID){ 			  // going through the segments of the current obstacle
					ang=find_nextcollision_single_seg(cobst->segs.at(segID));
					if (ang > 0 && ang < anghelp) {
						anghelp=ang;
						nextevent.angle=ang;
						nextevent.seg = &(cobst->segs.at(segID)); //returns the pointer to the segment
						nextevent.obstInd=obstacleID;
					}
				} //end of loop over segments of current obstacle
			
			} //end of proximity check
		
		} //end loop over obstacles
  
		for (int segid=0; segid < simulationbox->boundaries.size(); ++segid){  // also check box boundaries
			ang=find_nextcollision_single_seg(simulationbox->boundaries[segid]);
			if (ang>0 && ang<anghelp) {
				anghelp=ang;
				nextevent.angle=ang;
				nextevent.seg=&(simulationbox->boundaries[segid]);
				nextevent.obstInd=-1;
			}
		}// end of boundary loop
	
	return nextevent;
}


double tracer::signum(double x){
	if (x > 0) return 1;
	if (x < 0) return -1;
	else {
			cout<<"NaN or 0 in collision calculation!! check!"<<endl;
			return 0;
	}	
}


double  tracer::find_nextcollision_single_seg(segment seg){
	/* calculates the angle a tracer has to move in order to collide with given segment */	
	
	double a = pow( seg.rb.x - seg.re.x , 2 ) + pow( seg.rb.y - seg.re.y , 2 );											 // help quantities
	double b = 2 * ( (seg.re.x - seg.rb.x) * (seg.rb.x - ro.x ) + (seg.re.y - seg.rb.y) * (seg.rb.y - ro.y ) );
	double c = pow( seg.rb.x - ro.x , 2) + pow( seg.rb.y - ro.y , 2) - R * R;
	double delta = (pow(b,2) - 4 * a * c);
	coordinate rc1;
	coordinate rc2;
	
	double theta1,theta2;
	theta1 = 0.0; 
	theta2 = 0.0; 
	bool collision1 = false;
	bool collision2 = false;
	double collisionangle1,collisionangle2,collisionangle;
	
	if ( delta < 0 ){ 		// return a negative value for no collision
		return  minfinity; 
	}
	else{
		
		double sqdelta = pow(delta , 0.5);
		double s1 = ( (-1) * b - signum(b) * sqdelta) / (2*a);
		if (s1 > 0 && s1 < 1) {
			rc1.x = seg.rb.x + s1 * (seg.re.x - seg.rb.x);
			rc1.y = seg.rb.y + s1 * (seg.re.y - seg.rb.y);
			theta1 = atan2(rc1.y-ro.y, rc1.x-ro.x);				
			if(theta1 < 0){ theta1 += pi2; }
			if( !( theta1 < pi2 && theta1 >= 0 )){cerr << "# something went wrong with atan2. check!" << endl;}					
			collision1 = true;
		}  // end of if (s1>0 ...
      
      double s2 = c / (a*s1) ;
		if (s2 > 0 && s2 < 1) { 
			rc2.x = seg.rb.x + s2 * (seg.re.x - seg.rb.x);
			rc2.y = seg.rb.y + s2 * (seg.re.y - seg.rb.y);
			theta2 = atan2(rc2.y-ro.y, rc2.x-ro.x);
			if(theta2 < 0){ theta2+=pi2; };
			if( !( theta2 < pi2 && theta2 >= 0 )){cerr << "# something went wrong with atan2. check!" << endl;}
			collision2 = true;
		}  // end of if (s2>0 ...

		// up to here we have found the two angles of collision; the next step is to find out which of them is happening first.
		if ( collision1 == true && collision2 == true){ // if both collisions happen
			collisionangle1 = anglefinder(theta1,theta);
			collisionangle2 = anglefinder(theta2,theta);			
			if (collisionangle1 > collisionangle2){
				collisionangle = collisionangle2;
			}
			else {
				collisionangle = collisionangle1;
			}
		}  //end of if both collision happens
		else if (collision1 == true) collisionangle = anglefinder(theta1,theta); // if collision one is happening
		else if (collision2 == true) collisionangle = anglefinder(theta2,theta); // if collision two is happening
		else collisionangle = minfinity; 													 // if none of them are happening
	
		return collisionangle;

	}  //end of else for delta<0
}


bool tracer::is_engaged(tracer input_tracer, double delta_angle, SimBox* simulationbox){ 
	// check whether tracer is engaged with obstacles (recursion ensures that periodic boundaries are properly considered)

	tracer help_tracer = input_tracer;
	event next_event = help_tracer.find_nextcollision_single_simbox(simulationbox);
	if (next_event.angle == minfinity){ return false; }
	else if (next_event.seg->segtype == 'o'){ 		
		return true;
	 }
	else {
		delta_angle += next_event.angle;
		if (delta_angle >= pi2) {
			return false;		
		}
		else {
			help_tracer.move(next_event.angle);	
			help_tracer.collide(next_event.seg, simulationbox);
			return is_engaged(help_tracer, delta_angle, simulationbox);		
		}
	}
}	


bool  tracer::isinside(Obstacle iobstacle){
	/* checks whether a tracer is within an obstacle.
	   first each segment is presented as f(x,y)=0 where f(x,y)=y-a*x-c.
	   then f(xtracer,ytracer) is evaluated for parallel segments
	   and the sign is checked in both cases.  */

	double xt = getpos().x; // position of tracer
	double yt = getpos().y; 
  
	double dx,dy,f0,f6,f1,f11,f3,f9,f2,f4,a,c; 
  
	dx = iobstacle.segs.at(0).re.x-iobstacle.segs.at(0).rb.x;
	dy = iobstacle.segs.at(0).re.y-iobstacle.segs.at(0).rb.y;
	if (dx != 0){
		a = dy / dx;
		c = iobstacle.segs.at(0).rb.y - a * iobstacle.segs.at(0).rb.x;
		f0 = yt - a * xt - c;
	}
	else{
		f0 = xt - iobstacle.segs.at(0).rb.x;
	}


	dx = iobstacle.segs.at(6).re.x - iobstacle.segs.at(6).rb.x;
	dy = iobstacle.segs.at(6).re.y - iobstacle.segs.at(6).rb.y;
	if (dx != 0){
		a = dy / dx;
		c = iobstacle.segs.at(6).rb.y - a * iobstacle.segs.at(6).rb.x;
		f6 = yt - a * xt - c;
	}
	else{
		f6 = xt - iobstacle.segs.at(6).rb.x;
	}


	dx = iobstacle.segs.at(1).re.x - iobstacle.segs.at(1).rb.x;
	dy = iobstacle.segs.at(1).re.y - iobstacle.segs.at(1).rb.y;
	if (dx != 0){
		a = dy / dx;
		c = iobstacle.segs.at(1).rb.y - a * iobstacle.segs.at(1).rb.x;
		f1 = yt - a * xt - c;
	}
	else{
		f1 = xt - iobstacle.segs.at(1).rb.x;
	}


	dx = iobstacle.segs.at(11).re.x - iobstacle.segs.at(11).rb.x;
	dy = iobstacle.segs.at(11).re.y - iobstacle.segs.at(11).rb.y;
	if (dx != 0){
		a = dy / dx;
		c = iobstacle.segs.at(11).rb.y - a * iobstacle.segs.at(11).rb.x;
		f11 = yt - a * xt - c;
	}
	else{
		f11 = xt - iobstacle.segs.at(11).rb.x;
	}


	dx = iobstacle.segs.at(3).re.x - iobstacle.segs.at(3).rb.x;
	dy = iobstacle.segs.at(3).re.y - iobstacle.segs.at(3).rb.y;
	if (dx != 0){
		a = dy / dx;
		c = iobstacle.segs.at(3).rb.y - a * iobstacle.segs.at(3).rb.x;
		f3 = yt - a * xt - c;
	}
	else{
		f3 = xt - iobstacle.segs.at(3).rb.x;
	}


	dx = iobstacle.segs.at(9).re.x - iobstacle.segs.at(9).rb.x;
	dy = iobstacle.segs.at(9).re.y - iobstacle.segs.at(9).rb.y;
	if (dx != 0){
		a = dy / dx;
		c = iobstacle.segs.at(9).rb.y - a * iobstacle.segs.at(9).rb.x;
		f9 = yt - a * xt - c;
	}
	else{
		f9 = xt - iobstacle.segs.at(9).rb.x;
	}


	dx = iobstacle.segs.at(2).re.x - iobstacle.segs.at(2).rb.x;
	dy = iobstacle.segs.at(2).re.y - iobstacle.segs.at(2).rb.y;
	if (dx != 0){
		a = dy / dx;
		c = iobstacle.segs.at(2).rb.y - a * iobstacle.segs.at(2).rb.x;
		f2 = yt - a * xt - c;
	}
	else{
		f2 = xt - iobstacle.segs.at(2).rb.x;
	}


	dx = iobstacle.segs.at(4).re.x - iobstacle.segs.at(4).rb.x;
	dy = iobstacle.segs.at(4).re.y - iobstacle.segs.at(4).rb.y;
	if (dx != 0){
		a = dy / dx;
		c = iobstacle.segs.at(4).rb.y - a * iobstacle.segs.at(4).rb.x;
		f4 = yt - a * xt - c;
	}
	else{
		f4 = xt - iobstacle.segs.at(4).rb.x;
	}

  
	if ( ((f0*f6<0)&&(f1*f11<0)) || ((f3*f9<0)&&(f2*f4<0)) ){
		return true;
	}
	else{
		return false;
	}
    
}


bool  tracer::isinsidesimbox(SimBox* simbox){

	double xt = getpos().x;
	double yt = getpos().y; 

	double Lxmax,Lxmin,Lymax,Lymin;
	Lxmax = 1 * (simbox->getLx())/2.0;
	Lxmin =-1 * (simbox->getLx())/2.0;
	Lymax = 1 * (simbox->getLy())/2.0;
	Lymin =-1 * (simbox->getLy())/2.0;
	if ( (xt<Lxmax) && (xt>Lxmin)  &&    (yt<Lymax) && (yt>Lymin) ){
		return true;
	}
	else{
		return false;
	}
    
}


double tracer::distance2(tracer* othertracer, SimBox* simulationbox){
	double dx = getpos_unfolded(simulationbox).x - othertracer->getpos_unfolded(simulationbox).x;
	double dy = getpos_unfolded(simulationbox).y - othertracer->getpos_unfolded(simulationbox).y;
	return dx*dx + dy*dy;
}


