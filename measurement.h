#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

/* measurement objects consist of two double vectors, one storing times
	the other storing dynamical values. logarithmic time axis supported via member b (see constructors) */

class measurement{
	
	public:
		int n;
		double a;
		double b;
		std::vector<double> times; 
		std::vector<double> values; 
		
		measurement();
		measurement(double ia,double ib,int in);
		void print();
		void print(double scaley);
		void print(double scalex, double scaley);
		void fileprint(double scaley, int max_index, std::string fname);
		void growsize();

};

#endif
