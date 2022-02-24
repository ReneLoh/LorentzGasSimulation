#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include "segment.h"
#include "simbox.h"
#include "fluid.h"
#include "measurement.h"

class Simulation{
	
	public:
		Fluid fluid;				// fluid of tracers
		SimBox simulationbox;	// obstacle maze with periodic boundaries
		
		double lifespan;			// allowed tracer lifespan
		int msd_nr;					// no. of msd measurements to be taken
		double msd_scale;			// 1st scale parameter for msd time axis
		double acf_step;			// time interval between acf measurements
		int acf_origins;			/* no. of different time origins for time averages 
											for autocorrelation funcitons (acf)*/
		measurement msd;			// measurements for msd
		measurement msd_x;		// ... for msd in x-direction
		measurement msd_y;
		measurement acf_xx;		// measurement for acf xx.
		measurement acf_yy;
		measurement acf_xy;
		measurement acf_yx;	

		Simulation(SimBox S, Fluid F, double life, int msd_nr_in, double msd_scale_in, double acf_step_in, int acf_origins_in);
		void run();
		void print_results();

};

#endif

