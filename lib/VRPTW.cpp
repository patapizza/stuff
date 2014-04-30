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
int N;			// number of customer vertices, numbered: [NVeh+1..NVeh+N]
int NVeh;		// number of vehicles, numbered: [1..NVeh]
int Q;			// vehicle max capacity
double **c;		// travel costs/durations
float *coordX; 
float *coordY;
float *duration;	// service duration at each vertex
float *demand;		// demand et each vertex
int   *e;			// start of time window at each vertex
int   *l;			// end of time window at each vertex



/* miscellaous functions */
struct misc {
	inline static const string redExpr(bool expr) {
		if (expr) return "\033[0;31m";
		else return ""; 
	}
	inline static const string resetColor() {
		return "\033[0;0m";
	}};



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
}

/*
inline std::ostream &operator << (std::ostream &out_file, solutionTSP& s) {
	if (s.getViolations() > 0) out_file << "Infeasible ! ";
	out_file << "Cost=" << s.getCost() << " ";
	for(int i=1; i<N+1; i++)
		out_file << s.step[i] << " ";
	return (out_file);
}*/


/* class solutionVRPTW ********************************************************************************************
*	defines a solution for a TSP 
*/
solutionVRPTW::solutionVRPTW() {											// constructor ***
	previous = new int[NVeh+N+1];	// vertices from 1 to NVeh + N (i.e. for every vehicle depot and customer vertices)
	next = new int[NVeh+N+1];
	vehicle = new int[NVeh+N+1];	// vertices from 1 to Nveh + Nveh 	  	vehicle[i] = i iff i is a depot
									//										vehicle[i] = 0 iff i is not inserted
	b = new float[NVeh+N+1];		// service times for every vehicle depot and customer vertices

	cout << "Generating initial solution ..." << flush;
	generateInitialSolution();
	cout << "Done.\n" << flush; 
}
solutionVRPTW::solutionVRPTW(const solutionVRPTW& old_solution) {				// copy constructor ***
	previous = new int[NVeh+N+1];
	next = new int[NVeh+N+1];
	vehicle = new int[NVeh+N+1];
	b = new float[NVeh+N+1];
	*this = old_solution;
}
solutionVRPTW& solutionVRPTW::operator = (const solutionVRPTW& sol) {
	memcpy(previous, sol.previous, (NVeh+N+1)*sizeof(int));
	memcpy(next, sol.next, (NVeh+N+1)*sizeof(int));
	memcpy(vehicle, sol.vehicle, (NVeh+N+1)*sizeof(int));
	memcpy(b, sol.b, (NVeh+N+1)*sizeof(float));
	return (*this);
}
solutionVRPTW::~solutionVRPTW() {										// destructor ***
	delete [] previous;
	delete [] next;
	delete [] vehicle;
	delete [] b;
}

void solutionVRPTW::generateInitialSolution() {	// BEST INSERTION INITIAL SOL
												// todo: random vertex selection for insertion
	for(int r=1; r<NVeh+1; r++)	{
		vehicle[r] = r;
		previous[r] = r;
		next[r] = r;
	}

	/* To begin with, no customer vertex inserted */
	for (int i=NVeh+1; i<NVeh+N+1; i++) vehicle[i] = 0; 

	/* Then assign best insertion to each customer vertex in turn */
	for (int i=NVeh+1; i<NVeh+N+1; i++) {
		int before_i = bestInsertion(i);

		insertVertex(i, before_i, false);

	}
	computeServiceTimes();
}

/*
void solutionVRPTW::generateInitialSolution() {		// RANDOM INITIAL SOL
	int *last = new int [NVeh+1];
	for(int r=1; r<NVeh+1; r++)	{
		vehicle[r] = r;
		last[r] = r;
	}
	
	// Assign random values 
	for (int i=NVeh+1; i<NVeh+N+1; i++) {
		int r = rand() % NVeh + 1;
		vehicle[i] = r;
		previous[i] = last[r];
		last[r] = i;
	}
	for (int r=1; r<NVeh+1; r++) {
		int n = r;
		int i = last[r];
		previous[r] = last[r];
		while (i != r) {
			next[i] = n;
			n = i;
			i=previous[i];
		} next[r] = n;
	}
	computeServiceTimes();
	delete [] last;
}*/

/* bestInsertion(int vertex)
	Find the best position to REinstert a single vertex "vertex"
*/
int solutionVRPTW::bestInsertion(int vertex) {
	int before_i;
	float min_delta = numeric_limits<float>::max();
	for (int i=1; i<NVeh+N+1; i++) {
		float delta = 0.0;
		if (i == vertex || i == next[vertex]) continue;	// skip if same position
		if (vehicle[i] == 0) continue; // skip if vertex i not inserted yet
		delta =   c[previous[i]][i] 
				+ c[previous[i]][vertex]
				+ c[vertex][i];
		if (delta < min_delta) {
			before_i = i;
			min_delta = delta;
		} 
	} 
	return before_i;
}


void solutionVRPTW::insertVertex(int vertex, int before_i, bool remove) {
	//cout << "moving " << vertex-NVeh << " to before " << before_i-NVeh << endl;
	int from_route = vehicle[vertex];		// remember the route

	// Remove from current position
	if (remove) {
		previous[next[vertex]] = previous[vertex];
		next[previous[vertex]] = next[vertex];
	}

	// Insert elsewhere
	vehicle[vertex] = vehicle[before_i];
	next[vertex] = before_i;
	previous[vertex] = previous[before_i];
	previous[before_i] = vertex;
	next[previous[vertex]] = vertex;

	// notify solution that some routes changed
	routeChange(from_route); 
	if (from_route != vehicle[before_i]) 
		routeChange(vehicle[before_i]);
}

inline void solutionVRPTW::computeServiceTimes(int k) {	// recomputes service times at route k (k==0: every routes)
	for (int r=1; r<NVeh+1; r++) {
		if (k != 0 && k != r) continue;
		b[r] = 0;
		int i = next[r];
		while (i != r) {
			b[i] = max((double) e[i], b[previous[i]] + duration[previous[i]] + c[previous[i]][i]);
			i = next[i];
		} b[r] = max((double) e[r], b[previous[r]] + duration[previous[r]] + c[previous[r]][r]);
	}
}

inline float solutionVRPTW::getCostR(int k) {	// returns cost of route k (all route cost of k==0)
	float cost = 0.0;
	for (int r=1; r<NVeh+1; r++) {
		if (k != 0 && k != r) continue;
		int i = previous[r];
		while (i != r) {
			cost += c[i][next[i]];
			i=previous[i];
		} cost += c[i][next[i]];
	}
	return (cost);
}
inline float solutionVRPTW::getCost() {	// returns cost of route k
	return getCostR();
}
inline int solutionVRPTW::getViolations(int constraint) {
	int v = 0;
	float *load = new float [NVeh+1];	// load[i] = x iff vehicle i has a cumulative load of x
	
	for (int r=1; r<NVeh+1; r++) {	// compute vehicle loads
		int i = previous[r];
		load[r] = 0;
		while (i != r) {
			load[r] += demand[i];
			i=previous[i];
		} 
	}

	if(constraint == 1 || constraint == 0) for (int r=1; r<NVeh+1; r++) if (load[r] > Q) v ++; // to try:    v += load[r] - Q
	if(constraint == 2 || constraint == 0) for (int i=1; i<NVeh+N+1; i++) if (b[i]+0.000001 > l[i]) v ++;

	return (v);
}	


string& solutionVRPTW::toString() {

	ostringstream out; 
	out.setf(std::ios::fixed);
	out.precision(2);
	float *load = new float [NVeh+1];	// load[i] = x iff vehicle i has a cumulative load of x
	
	//for (int i=1; i<NVeh+N+1; i++)
	//	cout << "(i:" << i-NVeh << ",v:" << vehicle[i]-NVeh << "," << previous[i]-NVeh << "," 
	//			<< next[i]-NVeh << ",q:" << demand[i] << ",x:" << coordX[i] << ",y:" << coordY[i] << ") " << endl;
	for (int r=1; r<NVeh+1; r++) {	// compute vehicle loads
		int i = previous[r];
		load[r] = 0;
		while (i != r) {
			load[r] += demand[i]; 
			i=previous[i];
		} 
	}
	if (getViolations() > 0) out << "\033[0;31mInfeasible ! Violations: " << getViolations() << "\033[0;0m" << endl;
	out << "Cost = " << getCost() << " \n";
	for(int r=1; r<NVeh+1; r++) {
		out << "Route " << setw(2) << r << " (c:" << setw(7) << getCostR(r) 
			<< " q:" << misc::redExpr(load[r]>Q) << setw(7) << load[r] << misc::resetColor() << "): D \t";
		for(int i=next[r], n=1; i!=r; i=next[i], n++) {
			out << setw(3) << i-NVeh << "[" << misc::redExpr(b[i]>l[i]) << setw(7) << setfill('0') 
				<< b[i] << misc::resetColor() << "]" << " " << setfill(' ');
			if ((n%7) == 0) out << endl << "\t\t\t\t\t";
		}
		out << "  D" << "[" << misc::redExpr(b[r]>l[r]) << setw(7) << setfill('0') << b[r] << misc::resetColor() << "]" << endl << setfill(' ');
	}
	static string str = ""; str = out.str();
	return (str);
}



