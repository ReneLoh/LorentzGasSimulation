#include "simbox.h"

static constexpr double pi2 = 6.28318530718; 

using namespace std;

SimBox::SimBox(){
	Lx = 0;
	Ly = 0;
	nObstacles = 0;
	wObstacle = 0;
	lObstacle = 0;
}


SimBox::SimBox(double iL){
	Lx = Ly = iL;
	boundaries.resize(4);
	boundaries[0].set(-1.0 * Lx/2.0, -1.0 * Ly/2.0, -1.0 * Lx/2.0, Ly/2.0, 'l');
	boundaries[1].set(-1.0 * Lx/2.0, Ly/2.0, Lx/2.0, Ly/2.0, 't');
	boundaries[2].set(Lx/2.0, Ly/2.0, Lx/2.0, -1.0 * Ly/2.0, 'r');
	boundaries[3].set(Lx/2.0, -1.0 * Ly/2.0, -1.0 * Lx/2.0, -1.0 * Ly/2.0, 'b');
}


SimBox::SimBox(double iLx,double iLy){
	Lx = iLx;
	Ly = iLy;
	boundaries.resize(4);
	boundaries[0].set(-1.0 * Lx/2.0, -1.0 * Ly/2.0, -1.0 * Lx/2.0, Ly/2.0, 'l');
	boundaries[1].set(-1.0 * Lx/2.0, Ly/2.0, Lx/2.0, Ly/2.0, 't');
	boundaries[2].set(Lx/2.0, Ly/2.0, Lx/2.0, -1.0 * Ly/2.0, 'r');
	boundaries[3].set(Lx/2.0, -1.0 * Ly/2.0, -1.0 * Lx/2.0, -1.0 * Ly/2.0, 'b');
}


SimBox::SimBox(double iLx, double iLy, int inObstacles, double iwObstacle, double ilObstacle, int randomseed){

	Lx = iLx;
	Ly = iLy;
	nObstacles = inObstacles;
	wObstacle = iwObstacle;
	lObstacle = ilObstacle;
	boundaries.resize(4);
	boundaries[0].set(-1.0 * Lx/2.0, -1.0 * Ly/2.0, -1.0 * Lx/2.0, Ly/2.0, 'l');
	boundaries[1].set(-1.0 * Lx/2.0, Ly/2.0, Lx/2.0, Ly/2.0, 't');
	boundaries[2].set(Lx/2.0, Ly/2.0, Lx/2.0, -1.0 * Ly/2.0, 'r');
	boundaries[3].set(Lx/2.0, -1.0 * Ly/2.0, -1.0 * Lx/2.0, -1.0 * Ly/2.0, 'b');
	
	/*creating three random number for the seeds using seed_seq*/
	std::seed_seq seq{61396, 67454, 72163, 82624 + randomseed, 481841 + 5 * randomseed + 1, 12000 - randomseed};
	std::vector<std::uint32_t> seeds(3);
	seq.generate(seeds.begin(), seeds.end());
	std::mt19937 genx(seeds.at(0));
	std::mt19937 geny(seeds.at(1));
	std::mt19937 gentheta(seeds.at(2));
	std::uniform_real_distribution<> disx(-1 * Lx/2.0, Lx/2.0);
	std::uniform_real_distribution<> disy(-1 * Ly/2.0, Ly/2.0);
	std::uniform_real_distribution<> distheta(0, pi2);
  
	double dx = 0;
	double dy = 0;
	double dtheta = 0;
	Obstacle sampleObstacle(wObstacle, lObstacle, 0, 0);
	Obstacle hilfeObstacle;	
	int counter = 0;

	while (counter < nObstacles){

		dx = disx(genx);
		dy = disy(geny);
		dtheta = distheta(gentheta);
		obstacle_list.push_back(sampleObstacle);
		obstacle_list.at(counter).manipulate(dx, dy, dtheta);

		++counter;
		
	}
	
	// now take account of periodic boundaries
	
	bool left, right, top, bot, topright, botright, topleft, botleft; 
	//these make sure that a single obstacle does not get unnecessary many images.
	//they save whether an image has been created at the left-hand side etc.	

	for (int obsti=0; obsti < nObstacles; ++obsti){
		
		vector <coordinate> coordinates_obstacle;		
		left = false;
		right = false;
		top = false;
		bot = false;
		topright = false;
		botright = false;
		topleft = false;
		botleft = false;		
		
		for (int segi=0; segi < obstacle_list.at(obsti).segs.size(); ++segi){		
			coordinates_obstacle.push_back(obstacle_list.at(obsti).segs.at(segi).rb);		
		}
		
		for (int coord_k=0; coord_k < coordinates_obstacle.size(); ++coord_k){  		// loop over all 12 points of the obstacle
			
			// create fitting images whenever an obstacle coordinate is outside the box
			
			if (coordinates_obstacle.at(coord_k).x > Lx / 2.0){				
				if (coordinates_obstacle.at(coord_k).y > Ly / 2.0){								//	an image needs to be created at the bottom left corner of the box				
																													// only create the image if the obstacle doesn't already have an image there		
					if(botleft == false){ 					
						hilfeObstacle = obstacle_list.at(obsti);
						hilfeObstacle.manipulate(-1.0 * Lx, -1.0 * Ly, 0);									
						obstacle_list.push_back(hilfeObstacle);
						botleft = true;
					}				
				}
				else if (coordinates_obstacle.at(coord_k).y < -1.0 * Ly / 2.0){
					if(topleft == false){
						hilfeObstacle = obstacle_list.at(obsti);
						hilfeObstacle.manipulate(-1.0 * Lx, Ly, 0);									
						obstacle_list.push_back(hilfeObstacle);					
						topleft = true;
					}				
				}
				else if (left == false){
					hilfeObstacle = obstacle_list.at(obsti);					
					hilfeObstacle.manipulate(-1.0 * Lx, 0, 0);
					obstacle_list.push_back(hilfeObstacle);
					left = true;				
				}			
			}
			
			else if (coordinates_obstacle.at(coord_k).x < -1.0 * Lx / 2.0){
				if (coordinates_obstacle.at(coord_k).y > Ly / 2.0){
					if(botright == false){
						hilfeObstacle = obstacle_list.at(obsti);
						hilfeObstacle.manipulate(Lx, -1.0 * Ly, 0);									
						obstacle_list.push_back(hilfeObstacle);
						botright = true;
					}				
				}
				else if (coordinates_obstacle.at(coord_k).y < -1.0 * Ly / 2.0){
					if(topright == false){					
						hilfeObstacle = obstacle_list.at(obsti);
						hilfeObstacle.manipulate(Lx, Ly, 0);									
						obstacle_list.push_back(hilfeObstacle);
						topright = true;			
					}		
				}
				else if (right == false){
					hilfeObstacle = obstacle_list.at(obsti);
					hilfeObstacle.manipulate(Lx, 0, 0);
					obstacle_list.push_back(hilfeObstacle);
					right = true;
				}										
			}
			
			else if (coordinates_obstacle.at(coord_k).y > Ly / 2.0){
				if(bot == false){
					hilfeObstacle = obstacle_list.at(obsti);
					hilfeObstacle.manipulate(0, -1.0 * Ly, 0);
					obstacle_list.push_back(hilfeObstacle);
					bot = true;		
				}					
			}
			
			else if (coordinates_obstacle.at(coord_k).y < -1.0 * Ly / 2.0){
				if(top == false){
					hilfeObstacle = obstacle_list.at(obsti);
					hilfeObstacle.manipulate(0, Ly, 0);
					obstacle_list.push_back(hilfeObstacle);
					top = true;	
				}						
			}
				
		}
	}
	  
	cout << "# Number of obstacles that were placed: " + std::to_string(nObstacles) + "\n";
	cout << "# Number of additional obstacle images due to periodic boundaries: " << obstacle_list.size() - nObstacles << endl;
}


void SimBox::print(std::string iname){
	std::ofstream outputfile;
	outputfile.open (iname, std::ofstream::out | std::ofstream::trunc);
	outputfile.close();
	for (int i=0; i < obstacle_list.size();  ++i){
		obstacle_list.at(i).print(iname);
	}
}


double SimBox::getLx(){
	return Lx;
}


double SimBox::getLy(){
	return Ly;
}


int SimBox::getnObstacles(){
	return nObstacles;
}
