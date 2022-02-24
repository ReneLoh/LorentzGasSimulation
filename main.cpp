#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <chrono>
#include "coordinate.h"
#include "tracer.h"
#include "segment.h"
#include "event.h"
#include "simbox.h"
#include "obstacle.h"
#include "fluid.h"
#include "measurement.h"
#include "simulation.h"

using namespace std;
constexpr double pi = 3.1415926535;    
static constexpr double infinity = 1e8; 


int main(int argc, char *argv[]){
	
	chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();	
	cout << fixed << setprecision(15);	

	/****** Input Parameters ******/
	coordinate Origin(0.0, 0.0);

	double lifespan = atof(argv[1]);
	int ntracers = atoi(argv[2]);
	double gyrationradius = atof(argv[3]);
	int randomseed = atoi(argv[4]);
  
  	int msd_nr = 701;
	double msd_scale = 0.001;
	double acf_step = 0.3;
	int acf_origins = 530;
  

	/****** Creating the simulation box with the obstacles *****/
	cout << "building the box:" << endl;
   SimBox simulationbox(200, 100, 3750, 0.225, 1.0, randomseed);
   cout << simulationbox.getnObstacles() << endl;

	cout << "# Lx/2: " << simulationbox.getLx()/2 << endl;
	cout << "# Ly/2: " << simulationbox.getLy()/2 << endl;
//	simulationbox.print("obstacles.dat"); 


	/****** Creating the tracers ******/
   cout << "building the fluid" << endl;
	Fluid fluid(ntracers, gyrationradius, &simulationbox, randomseed);

	Simulation simu(simulationbox, fluid, lifespan, msd_nr, msd_scale, acf_step, acf_origins);
	simu.run();
	simu.print_results();

	chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();
	chrono::duration <double> time_span = chrono::duration_cast <chrono::duration <double> > (t2-t1);
	cout << "# program took " << time_span.count() << " seconds!" << endl;
	
	return 0;
}