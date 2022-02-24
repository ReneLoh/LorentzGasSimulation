#ifndef FLUID_H
#define FLUID_H

#include <vector>
#include <random>
#include <iostream>
#include "simbox.h"
#include "tracer.h"

class Fluid {

	private:
		int ntracers;		// no. of tracers
		double gyrius;		// gyration radius

	public:
		std::vector<tracer> tracers;		// stores fluid tracers
		
		Fluid(int ntracers, double igyrius, SimBox* simulationbox, int randomseed);
		Fluid();
		int nr();
		int nrengaged();

};

#endif
