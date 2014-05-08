#include <iostream>
#include <fstream>	// file I/O
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy

#include "LSBase.h"

#ifndef VRPlib_H
#define	VRPlib_H

using namespace CBLS;

#define	CAPACITY_CONSTRAINT	1
#define	TW_CONSTRAINT		2
#define ALL_CONSTRAINT		0

/* Handles instance files of Cordeau-Laporte in data/vrp/old */
void readInstanceFileCordeauLaporteVRPold(const char *filename);

/* Handles instance files of Cordeau-Laporte in data/vrptw/old */
void readInstanceFileCordeauLaporteVRPTWold(const char *filename);



class solutionVRP {
protected:
	/* Decision variables */
	bool *active;	// tells whether a customer vertex is active in the solution (i.e. must be serviced)
	int *previous;		// previous[i] = j iff vertex j is visited before i in the same route
	int *next;			// !!! -> Numbered from 1 to N
	int *vehicle;		// vehicle[i] = j iff vertex i is serviced by vehicle j
	int *routeLoad;		// routeLoad[i] = x iff vehicle i has a cumulative load of x
	double *routeCost;	// routeCost[r] = cost of route r 
public:
	solutionVRP();												// constructor ***
	solutionVRP(const solutionVRP& old_solution);				// copy constructor ***
	solutionVRP& operator = (const solutionVRP& sol);
	~solutionVRP();

	/* dynamic context */
	void sampleInstance(float active_ratio);

	/* general methods */
	void generateInitialSolution();
	int bestInsertion(int vertex);
	bool feasibleInsertion(int vertex, int before_i);
	void insertVertex(int vertex, int before_i, bool remove);
	
	double getCost(int k = 0);
	int getViolations(int constraint = ALL_CONSTRAINT);
	int* getpPrevious() { return previous; }
	int* getpNext() { return next; }
	int* getpVehicle() { return vehicle; }
	int nbConstraints() { return 1; }
	bool isActive(int v) { return active[v]; }
	
	virtual void updateRouteInfos(int k = 0);
	void routeChange(int k = 0) { updateRouteInfos(k); }

	std::string& toString();
	void checkSolutionConsistency();
};


class solutionVRPTW: public solutionVRP {
protected:
	bool *fixed;		// tells whether a customer vertex is fixed in the solution (i.e. cannot be moved)
	float *h;			// arrival times
	float *b;			// service times
public:
	solutionVRPTW();												// constructor ***
	solutionVRPTW(const solutionVRPTW& old_solution);				// copy constructor ***
	solutionVRPTW& operator = (const solutionVRPTW& sol);
	~solutionVRPTW();

	/* dynamic context */
	void fixSolution(float t);

	/* general methods */
	bool feasibleInsertion(int vertex, int before_i);
	int getViolations(int constraint = ALL_CONSTRAINT);

	int nbConstraints() { return 2; }
	void updateRouteInfos(int k = 0);

	std::string& toString();
	void checkSolutionConsistency();
};


#endif