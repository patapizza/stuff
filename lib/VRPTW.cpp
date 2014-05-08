#include <iostream>
#include <fstream>	// file I/O
#include <sstream>	// ostringstream
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy
#include <limits>   // numeric_limits
#include <iomanip>	// setw, setfill
#include <algorithm>// std::max
#include <limits> 	// numeric_limits

#include "VRPlib.h"

using namespace std;


/* Problem data variables */
extern int N;			// number of customer vertices, numbered: [NVeh+1..NVeh+N]
extern int NVeh;		// number of vehicles, numbered: [1..NVeh]
extern int Q;			// vehicle max capacity
extern double **c;		// travel costs/durations
extern float *coordX; 
extern float *coordY;
extern float *demand;		// demand et each vertex
float *duration;	// service duration at each vertex
int   *e;			// start of time window at each vertex
int   *l;			// end of time window at each vertex

extern int Nactive;

/* miscellaous functions */
struct misc {
	inline static const string redExpr(bool expr) {
		if (expr) return "\033[0;31m";
		else return ""; 
	}
	inline static const string greenExpr(bool expr) {
		if (expr) return "\033[0;32m";
		else return ""; 
	}
	inline static const string resetColor() {
		return "\033[0;0m";
	}
};



/* Handles instance files of Cordeau-Laporte in data/vrptw/old */
void readInstanceFileCordeauLaporteVRPTWold(const char *filename) {
	int i;
	std::string line;
	std::ifstream instance_file;
	instance_file.open(filename);
	if (instance_file.fail()) {
		std::cerr << endl << " Error: Unable to open " << filename << endl;
		exit (8);
	}

	instance_file >> i; 	// skip first integer
	instance_file >> NVeh;	// retrieve number of vehicles
	instance_file >> N;		// retrieve instance size
	c = new double*[NVeh+N+1];
	for(int i=0; i<NVeh+N+1; i++) c[i] = new double[NVeh+N+1];

	coordX 		= new float[NVeh+N+1]; 
	coordY 		= new float[NVeh+N+1];
	duration 	= new float[NVeh+N+1];
	demand 		= new float[NVeh+N+1];
	e 			= new int[NVeh+N+1]; 
	l 			= new int[NVeh+N+1];

	std::getline(instance_file, line); // skip end of line

	instance_file >> i;			// skip int
	instance_file >> Q;			// retrieve max capacity

	// DEPOTS
	instance_file >> i;			// skip int
	instance_file >> coordX[1]; 	// retrieve x coordinate
	instance_file >> coordY[1]; 	// retrieve y coordinate
	instance_file >> duration[1];	// retrieve service duration
	instance_file >> demand[1];		// retrieve demand
	instance_file >> i; 			// skip int
	instance_file >> i; 			// skip int
	//instance_file >> i; 			// skip int 	---->    /!\ One int less to skip in the depot line
	instance_file >> e[1];			// retrieve start TW
	instance_file >> l[1];			// retrieve end TW
	for (int r=2; r<NVeh+1; r++) {
		coordX[r] = coordX[1];
		coordY[r] = coordY[1];
		duration[r] = duration[1];	
		demand[r] = demand[1];		
		e[r] = e[1];				
		l[r] = l[1];				
	} 

	// REQUESTS
	for (int j=1; j<N+1; j++) {
		instance_file >> i; 				// vertex number - skip it
		instance_file >> coordX[j+NVeh]; 	// retrieve x coordinate
		instance_file >> coordY[j+NVeh]; 	// retrieve y coordinate
		instance_file >> duration[j+NVeh];	// retrieve service duration
		instance_file >> demand[j+NVeh];	// retrieve demand
		instance_file >> i; 				// skip int
		instance_file >> i; 				// skip int
		instance_file >> i; 				// skip int
		instance_file >> e[j+NVeh];			// retrieve start TW
		instance_file >> l[j+NVeh];			// retrieve end TW
	}

	instance_file.close();

	for (int i=1; i<NVeh+N+1; i++)
		for (int j=1; j<NVeh+N+1; j++)
			c[i][j] = std::sqrt(std::pow((coordX[i] - coordX[j]),2) + std::pow((coordY[i] - coordY[j]),2)); // compute Euclidean distances

	//for (int i=NVeh+1; i<NVeh+N+1; i++)
	//	cout << "demand[" << i-NVeh << "]=" << demand[i] << endl;
}

/*
inline std::ostream &operator << (std::ostream &out_file, solutionTSP& s) {
	if (s.getViolations() > 0) out_file << "Infeasible ! ";
	out_file << "Cost=" << s.getCost() << " ";
	for(int i=1; i<N+1; i++)
		out_file << s.step[i] << " ";
	return (out_file);
}*/


/* class solutionVRPTW  ********************************************************************************************/

solutionVRPTW::solutionVRPTW() {									// constructor ***
	fixed = new bool[NVeh+N+1];	// tells whether a customer vertex is fixed in the sol (i.e. cannot be moved)
	for (int i=1; i<NVeh+N+1; i++) fixed[i] = false;

	h = new float[NVeh+N+1];		// arrival times for every vehicle depot and customer vertices
	b = new float[NVeh+N+1];		// service times for every vehicle depot and customer vertices
}
solutionVRPTW::solutionVRPTW(const solutionVRPTW& old_solution) : solutionVRPTW() {		// copy constructor ***
	*this = old_solution;
}
solutionVRPTW& solutionVRPTW::operator = (const solutionVRPTW& sol) {
	solutionVRP::operator = (sol);
	memcpy(fixed, sol.fixed, (NVeh+N+1)*sizeof(bool));
	memcpy(h, sol.h, (NVeh+N+1)*sizeof(float));
	memcpy(b, sol.b, (NVeh+N+1)*sizeof(float));
	return (*this);
}
solutionVRPTW::~solutionVRPTW() {										// destructor ***
	delete [] fixed;
	delete [] h;
	delete [] b;
}



/* Tells whether it is possible to insert a vertex before another vertex, w.r.t the constraints */
inline bool solutionVRPTW::feasibleInsertion(int vertex, int before_i) {
	float arrival_time = b[previous[before_i]] + duration[previous[before_i]] + c[previous[before_i]][vertex];
	float service_time = max((float) e[vertex], arrival_time);
	float arrival_time_before_i = service_time + duration[vertex] + c[vertex][before_i];
	float service_time_before_i = max((float) e[before_i], arrival_time_before_i);

	return 
		solutionVRP::feasibleInsertion(vertex, before_i) && 
		service_time <= l[vertex]+0.00001 &&
		service_time_before_i <= l[before_i]+0.00001;
}

// recomputes load, arrival and service times at route k (k==0: every routes)
inline void solutionVRPTW::updateRouteInfos(int k) {	
	solutionVRP::updateRouteInfos(k);

	for (int r=1; r<NVeh+1; r++) {
		if (k != 0 && k != r) continue;

		// Update arrival and service times upon route k
		h[r] = 0;
		b[r] = 0;
		int i = next[r];
		while (i != r) {
			h[i] = b[previous[i]] + duration[previous[i]] + c[previous[i]][i];
			b[i] = max((float) e[i], h[i]);
			i = next[i];
		} 
		h[r] = b[previous[r]] + duration[previous[r]] + c[previous[r]][r];
		b[r] = max((float) e[r], h[r]);
	}

}


inline int solutionVRPTW::getViolations(int constraint) {
	int v = solutionVRP::getViolations(constraint);

	if(constraint == TW_CONSTRAINT || constraint == ALL_CONSTRAINT) 
		for (int i=1; i<NVeh+N+1; i++) 
			if (active[i] && b[i]+0.00001 > l[i]) v ++;

	return (v);
}	


string& solutionVRPTW::toString() {

	ostringstream out; 
	out.setf(std::ios::fixed);
	out.precision(2);
	
	
	//out << "Capacity violations: " << getViolations(1) << endl << "TW violations: " << getViolations(2) << endl << flush;
	if (getViolations() > 0) out << "\033[0;31mInfeasible ! Violations: " << getViolations() << "\033[0;0m" << endl << flush;
	out << "Cost = " << getCost() << " \n";
	for(int r=1; r<NVeh+1; r++) {
		out << "Route " << setw(2) << r << " (c:" << setw(7) << getCost(r) 
			<< " q:" << misc::redExpr(routeLoad[r]>Q) << setw(7) << routeLoad[r] << misc::resetColor() << "): D \t";
		for(int i=next[r], n=1; i!=r; i=next[i], n++) {
			out << setw(3) << misc::greenExpr(fixed[i]) << i-NVeh << misc::resetColor() << "[" << misc::redExpr(b[i]>l[i]) 
				<< setw(7) << setfill('0') << b[i] << misc::resetColor() << "]" << " " << setfill(' ');
			if ((n%7) == 0) out << endl << "\t\t\t\t\t";
		}
		out << misc::greenExpr(fixed[r]) << "  D" << misc::resetColor() << "[" << misc::redExpr(b[r]>l[r]) << setw(7) 
			<< setfill('0') << b[r] << misc::resetColor() << "]" << endl << setfill(' ');
	}
	static string str = ""; str = out.str();
	return (str);
}

/* test if hard constraints are respected */
void solutionVRPTW::checkSolutionConsistency() {
	solutionVRP::checkSolutionConsistency();

	// arrival times must precede service times
	for (int i=NVeh+1; i<NVeh+N+1; i++)
		ASSERT(!active[i] || h[i] <= b[i], "i="<<i-NVeh << " h="<<h[i]<<" b="<<b[i]); // active --> good TW
}


/* dynamic context: 
 * any customer i already serviced of about to be serviced at time t becomes fixed
 */
void solutionVRPTW::fixSolution(float t) {	
	// depots vertices: because of time windows, depot can also be fixed (end of workday)
	for (int k=1; k<NVeh+1; k++) 
		if (t + duration[previous[k]] + c[previous[k]][k] >= l[k]) fixed[k] = true;
	// customer vertices
	for (int i=NVeh+1; i<NVeh+N+1; i++) 
		if (t + duration[previous[i]] + c[previous[i]][i] >= b[i]) fixed[i] = true;
}


