#include <iostream>
#include <fstream>	// file I/O
#include <sstream>	// ostringstream
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy
#include <limits>   // numeric_limits
#include <iomanip>  // std::setw

#include "VRPlib.h"

using namespace std;


/**** Problem data variables ****/
extern int N;		// number of vertices [1..N]
extern int NVeh;	// number of vehicles [1..NVeh]
extern int Q;		// vehicle max capacity
extern double 	**c;
extern double 	*coordX; 
extern double 	*coordY;
extern float 	*demand;	// demand et each vertex
extern int 	*number;	// number of the customer (in the test instance)
extern int 	*reqTime;	// time at which the request is revealed (dynamic context)

extern int Nactive;
extern int NVehActive;

// Dynamic context
extern int NVertices;			// number of customer vertices (different from N, because of time slots, there can be several requests per vertex)
extern int nTimeSlots;			// number of time bins
extern int *reqProba;			// probabitlity for that request to appear online (for a given time slot)



/* Handle instance files of Cordeau-Laporte vrp/old */
void readInstanceFileCordeauLaporteVRPold(const char *filename) {
	int i;
	std::string line;
	std::ifstream instance_file;
	instance_file.open(filename);
	if (instance_file.fail()) {
		std::cerr << "Error: Unable to open " << filename << endl;
		exit (8);
	}

	instance_file >> i; 	// skip first integer
	instance_file >> NVeh;	// retrieve number of vehicles
	instance_file >> N;		// retrieve instance size
	c = new double*[NVeh+N+1];
	for(int i=0; i<NVeh+N+1; i++) c[i] = new double[NVeh+N+1];

	coordX = new double[NVeh+N+1]; 
	coordY = new double[NVeh+N+1];
	demand = new float[NVeh+N+1];
	number = new int[NVeh+N+1];
	reqTime = new int[NVeh+N+1];
	for (int i=1; i<NVeh+N+1; i++) reqTime[i] = -1;		// fully deterministic instance

	std::getline(instance_file, line); // skip end of line

	instance_file >> i;			// skip int
	instance_file >> Q;			// retrieve max capacity

	// DEPOTS
	instance_file >> i;					// skip int
	instance_file >> coordX[1]; 		// retrieve x coordinate
	instance_file >> coordY[1]; 		// retrieve y coordinate
	std::getline(instance_file, line);	// skip end of line
	for (int r=2; r<NVeh+1; r++) {
		coordX[r] = coordX[1];
		coordY[r] = coordY[1];
	} 

	// REQUESTS
	for (int j=NVeh+1; j<NVeh+N+1; j++) {
		instance_file >> number[j]; 	// retrieve customer number
		instance_file >> coordX[j]; 	// retrieve x coordinate
		instance_file >> coordY[j]; 	// retrieve y coordinate
		instance_file >> i;				// skip int
		instance_file >> demand[j];		// retrieve demand
		std::getline(instance_file, line);	// skip end of line
	}

	instance_file.close();

	for (int i=1; i<NVeh+N+1; i++)
		for (int j=1; j<NVeh+N+1; j++)
			c[i][j] = std::sqrt(std::pow((coordX[i] - coordX[j]),2) + std::pow((coordY[i] - coordY[j]),2)); // compute Euclidean distances
}