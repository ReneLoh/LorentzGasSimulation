#include "simulation.h"

using namespace std;

Simulation::Simulation(SimBox S, Fluid F, double life, int msd_nr_in, double msd_scale_in, double acf_step_in, int acf_origins_in){

	fluid = F;						
	simulationbox = S;			
	
	lifespan = life;			
	
	msd_nr = msd_nr_in;												
	msd_scale = msd_scale_in;										
	double msd_scale2 = pow( lifespan/msd_scale, 1./msd_nr );	
	msd = measurement( msd_scale, msd_scale2, 2 );					
	msd_x = measurement( msd_scale, msd_scale2, 2 );				
	msd_y = measurement( msd_scale, msd_scale2, 2 );				
	 
	acf_origins = acf_origins_in; 										
	acf_step = acf_step_in;												
	int nrentries_acf = int( floor( lifespan/acf_step ) ) + 1;
	acf_xx = measurement( acf_step, 1, nrentries_acf );			
	acf_yy = measurement( acf_step, 1, nrentries_acf );
	acf_xy = measurement( acf_step, 1, nrentries_acf );
	acf_yx = measurement( acf_step, 1, nrentries_acf );	

}


void Simulation::run(){
	// runs simulation for constructed fluid and obstacle maze
	
	vector <coordinate> position; // in case one wants to print out a single trajectory for visualization 
	vector <double> pos_times;		// corresponding parts in code below need to be commented in/out.
	
	/****** trajectory calculation ******/
	double t, life, dx, dy, help_angle, delta_angle;
	int timestampid_msd;	
	int timestampid_V;
	int coll_ctr = 0;
	tracer* singletracer;
	tracer tr0;
	bool collided, acf_switch, msd_switch;

	for (int tracerid=0; tracerid<fluid.nr(); ++tracerid){  // loop over tracers

		delta_angle = 0;
		singletracer = &fluid.tracers.at(tracerid);	
		singletracer->engaged = singletracer->is_engaged(*singletracer, delta_angle, &simulationbox);
		
		if (singletracer->engaged == true){		

			tr0 = fluid.tracers.at(tracerid);
			life = 0;
			timestampid_msd = 1;
			timestampid_V = 1;
			measurement Vx(acf_step, 1, 2);
			Vx.values.at(0) = tr0.getvelocity().x;
			measurement Vy(acf_step, 1, 2);
			Vy.values.at(0) = tr0.getvelocity().y;
			
/*			position.clear();  						// comment in only when interested in trajectory of tracer
			pos_times.clear();						// ensures that only the very last tracer is stored
			position.push_back(tr0.getpos());
			pos_times.push_back(0.0);
			*/
			
			coll_ctr = 0;			
			
			event lastevent;
			event nextevent;  // needs to be constructed in this scope
			acf_switch = true;
			msd_switch = true;

			while (life < lifespan) {  // simulate trajectory until life > lifespan
			
				nextevent = singletracer->find_nextcollision_single_simbox(&simulationbox);
				t = nextevent.angle * singletracer->getr();	
				
				collided = false;

		      while(collided == false){  // loop ensures that next collision is searched only after particle has collided 

					if ( life+t>msd_x.times.at(timestampid_msd) && msd_switch==true ){

						help_angle = (msd_x.times.at(timestampid_msd) - life) / singletracer->getr(); 
						dy = singletracer->getpos_unfolded(&simulationbox, help_angle).y - tr0.getpos_unfolded(&simulationbox).y;
						dx = singletracer->getpos_unfolded(&simulationbox, help_angle).x - tr0.getpos_unfolded(&simulationbox).x;    	  					
						msd_y.values.at(timestampid_msd) += dy*dy; 
						msd_x.values.at(timestampid_msd) += dx*dx;					
						msd.values.at(timestampid_msd) += dx*dx + dy*dy;
																	
						++timestampid_msd;
		
						if (timestampid_msd >= msd_x.n){
							msd.growsize();
							msd_y.growsize();
							msd_x.growsize();					
						}
						
						if(msd_x.times.at(timestampid_msd) > lifespan){
							msd_switch = false;			
						}					

					}
					
					else if (life+t > Vx.times.at(timestampid_V) && acf_switch == true){
						
						help_angle = (Vx.times.at(timestampid_V) - life) / singletracer->getr();
						Vx.values.at(timestampid_V) = singletracer->getvelocity(help_angle).x;
						Vy.values.at(timestampid_V) = singletracer->getvelocity(help_angle).y;
		
/*						position.push_back(singletracer->getpos(help_angle));  // comment in only when interested in trajectory of tracer
						pos_times.push_back(Vx.times.at(timestampid_V));*/									

						++timestampid_V;	
						      
						if(timestampid_V >= Vx.n){
							Vx.growsize();
							Vy.growsize();
						}
										
						if(Vx.times.at(timestampid_V) > lifespan){
							acf_switch = false;							
						}

					}	
						  
					else{  //collision occurs
											
						if (lastevent.seg != nextevent.seg){  // particle is going to collide with another segment.						

							singletracer -> move(nextevent.angle);
							
/*							position.push_back(singletracer->getpos());  // comment in only when interested in trajectory of tracer
							pos_times.push_back(life+t);*/							
													
							singletracer->collide(nextevent.seg, &simulationbox);  // execute collision
							
							lastevent = nextevent;
							if(nextevent.seg->segtype == 't' || nextevent.seg->segtype == 'b' 
								|| nextevent.seg->segtype == 'l' || nextevent.seg->segtype == 'r'){ // collision with box boundary					
								
								t += singletracer->epsilon * singletracer->getr();  // this adjusts the lifetime in accordance with "epsilon trick" performed at the 
																									 // collisions with box boundaries, see collide() function in tracer.cpp

								singletracer->epsilon = 1e-9; //restores default 
							}
							else {
								coll_ctr++;								
							}
															
						} //end other-segment if
						
						else{ //particle collides with same segment as before 
							
							if(nextevent.seg->segtype == 't' || nextevent.seg->segtype == 'b' 
							   || nextevent.seg->segtype == 'l' || nextevent.seg->segtype == 'r'){ // collision happens with box boundary
								
								singletracer->move(nextevent.angle);

/*								position.push_back(singletracer -> getpos());  // comment in only when interested in trajectory of tracer
								pos_times.push_back(life+t);*/										
														
								singletracer->collide(nextevent.seg, &simulationbox);
								
								t += singletracer->epsilon * singletracer->getr();
								singletracer->epsilon = 1e-9;
							}	
							else if (singletracer->safemove(&nextevent) == true){  // admissible collision (i.e. tracer did not get stuck in obstacle)
								singletracer->collide(nextevent.seg, &simulationbox);
								coll_ctr++;								
							}						
						
						} // end same-segment collision else

						collided = true;
						life += t;

/*						position.push_back(singletracer -> getpos());  // comment in only when interested in trajectory of tracer
						pos_times.push_back(life);*/						

					} //end collision else
					 
				} //end of collision loop

			} //end of lifespan loop 


			/****** take time averages of acf's ******/					
			double sum_xx, sum_yy, sum_xy, sum_yx;	
			int kmax;		
			for (int i=0; i<acf_xx.n-1; ++i){  

				sum_xx = 0;
				sum_yy = 0;
				sum_xy = 0;
				sum_yx = 0;

				kmax = floor( (Vx.n-2-i) / double(acf_origins) );
				
				for (int k=0; k<=kmax; ++k){
					sum_xx += Vx.values.at(k*acf_origins) * Vx.values.at(k*acf_origins+i);
					sum_yy += Vy.values.at(k*acf_origins) * Vy.values.at(k*acf_origins+i);
					sum_xy += Vx.values.at(k*acf_origins) * Vy.values.at(k*acf_origins+i);
					sum_yx += Vy.values.at(k*acf_origins) * Vx.values.at(k*acf_origins+i);				
				}
				
				acf_xx.values.at(i) += 1./(kmax+1) * sum_xx;
				acf_yy.values.at(i) += 1./(kmax+1) * sum_yy;
				acf_xy.values.at(i) += 1./(kmax+1) * sum_xy;
				acf_yx.values.at(i) += 1./(kmax+1) * sum_yx;			

			} //end acf loop	
														
		} //end if engaged
		
		cout << "# particle " << tracerid << " done!" << endl;

	} //end tracer loop
	
	/*	ofstream positions {"trajectory.dat"};  								// comment in only when interested in trajectory of tracer
	for(int i = 0; i < position.size(); ++i){
		positions << fixed << setprecision(15) << pos_times.at(i) 
		<< " " << position.at(i).x << " " << position.at(i).y << endl;
	}
	positions.close()*/;

}


void Simulation::print_results(){
	// prints results as stored in member measurements obtained by simulation.run()	
	
	if (fluid.nrengaged() > 0){
		cout << "# nr of engaged particles: "<< fluid.nrengaged() << endl;
		double nrengaged_inv = 1.0/fluid.nrengaged();
		msd.fileprint( nrengaged_inv, msd.n-2, "msd.csv" );
		msd_x.fileprint( nrengaged_inv, msd_x.n-2, "msd_x.csv" );				
		msd_y.fileprint( nrengaged_inv, msd_x.n-2, "msd_y.csv" );
	
		acf_yx.fileprint( nrengaged_inv, acf_yx.n-2, "acf_yx.csv" );		
		acf_xy.fileprint( nrengaged_inv, acf_yx.n-2, "acf_xy.csv" );
		acf_xx.fileprint( nrengaged_inv, acf_yx.n-2, "acf_xx.csv" );
		acf_yy.fileprint( nrengaged_inv, acf_yx.n-2, "acf_yy.csv" );
	}
	else{
		cout << "# no particle is engaged with any obstacle:\n"    
				  "# either increase the number of tracers, or\n"    
				  "# increase the density of obstacles. " << endl;
	}

}