#include "measurement.h"

measurement::measurement(){
	n = 0;
	a = 0;
	b = 0;
}


measurement::measurement(double ia, double ib, int in){

  n=in;
  a=ia;
  b=ib;
  times.resize(in,0);
  values.resize(in,0);
  times.at(0)=0;
  for(int i=1; i<in; ++i){
		if (b==1.0){
			times.at(i)=a*i;
		}
		else{ 
			times.at(i)=a*pow(b,i);
		}
	}
}


void measurement::print(){
  for (int i=0; i<n-2; ++i){
		std::cout<<times.at(i)<<" "<<values.at(i)<<std::endl;
	}
}

void measurement::print(double scalex, double scaley){
	for (int i=0; i<n-2; ++i){
		std::cout<<times.at(i)*scalex<<" "<<values.at(i)*scaley<<std::endl;
	}
}


void measurement::fileprint(double scaley, int max_index, std::string fname){
	std::ofstream outputfile {fname};
	for (int i=0; i<=max_index; ++i){
      outputfile<<std::setprecision(12)<<times.at(i)<<" "<<values.at(i)*scaley<<std::endl;
	}
   outputfile.close();
}


void measurement::growsize(){
	if (b==1){
		times.push_back(a*n);
		values.push_back(0.0);
	}
	else{
		times.push_back(a*pow(b,n));
		values.push_back(0.0);
	}
	n++;
}


