#include "fluid.h"

using namespace std;

static constexpr double pi2 = 6.28318530718;


Fluid::Fluid(int intracers, double inputgyrius, SimBox* simulationbox, int randomseed){
	// constructor. places tracers on free spots in obstacle maze of simulationbox

	gyrius = inputgyrius;	// circular motion of tracers
	ntracers = intracers;	// number of tracers

	/*creating two random numbers for the seeds using seed_seq*/
	std::seed_seq seq{1,20,3200,403,5*randomseed+1,12000,73667,9474+randomseed,19151-randomseed};
	std::vector<std::uint32_t> seeds(3);
	seq.generate(seeds.begin(), seeds.end());
	std::mt19937 genx(seeds.at(0)); 																										// standard Mersenne Twister engine seeded with rd()
	std::mt19937 geny(seeds.at(1));
	std::mt19937 gentheta(seeds.at(2));
	std::uniform_real_distribution<> disx( -1 * (simulationbox->getLx())/2.0, (simulationbox->getLx())/2.0);
	std::uniform_real_distribution<> disy( -1 * (simulationbox->getLy())/2.0, (simulationbox->getLy())/2.0);
	std::uniform_real_distribution<> distheta(0,pi2); 

	double ranx,rany,rantheta;
	bool inside,insidethesimbox,found;
	int obstid;
	
	if (tracers.size() != 0){
		cerr<<"Trying to create fluid when fluid already exists?!"<<endl;
	}

	tracers.resize(intracers); //setting the size to intracers.
	int nrtrials = 0;
		
	for (int tracerid=0; tracerid < tracers.size(); ++tracerid){
		
		tracers.at(tracerid).setr(gyrius);
		inside = true;

		while (inside == true){
			
			ranx = disx(genx);					// assign random origin for circular motion
			rany = disy(geny);
			rantheta = distheta(gentheta);   // ... and relative angle
			++nrtrials;
			tracers.at(tracerid).setro(ranx-gyrius*cos(rantheta), rany-gyrius*sin(rantheta)); // obtain origin from position and phase
			tracers.at(tracerid).setangle(rantheta);
      
			obstid = 0;
			found = false; 
			inside = true;
			while ( obstid < simulationbox->obstacle_list.size()  && found == false){ 	// check whether tracer was accidentally placed inside obstacle
			
				if (tracers.at(tracerid).isinside(simulationbox->obstacle_list.at(obstid)) == true){
					found = true;
				}
				++obstid;
		
			}

			if (found == false)  {inside = false;}   // tracer did not end up in obstacle, i.e. positioning successful
		
		} //end of while
	
	}  //end of tracer loop
	
	cout << "# Succesfully finished initialization of tracers" << endl;
	cout << "# with total number of " << nrtrials << " trials for initiliazing " << tracers.size() << " tracers" << endl;

}


Fluid::Fluid(){
	ntracers = 0;
	gyrius = 0;
}


int Fluid::nr(){
	return tracers.size();
}


int Fluid::nrengaged(){
	// counts engaged tracers (i.e. that are not trapped in free circle motion)
	
	int inthilfe = 0;
	for (int i=0; i < tracers.size(); ++i){
		if (tracers.at(i).engaged == true){
			++inthilfe;
		}
	}
	
	return inthilfe;
}
