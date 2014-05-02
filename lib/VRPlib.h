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


/* Handles instance files of Cordeau-Laporte in data/vrp/old */
void readInstanceFileCordeauLaporteVRPold(const char *filename);

/* Handles instance files of Cordeau-Laporte in data/vrptw/old */
void readInstanceFileCordeauLaporteVRPTWold(const char *filename);



class solutionVRP {
protected:
	/* Decision variables */
	int *previous;		// previous[i] = j iff vertex j is visited before i in the same route
	int *next;			// !!! -> Numbered from 1 to N
	int *vehicle;		// vehicle[i] = j iff vertex i is serviced by vehicle j
public:
	solutionVRP();												// constructor ***
	solutionVRP(const solutionVRP& old_solution);				// copy constructor ***
	solutionVRP& operator = (const solutionVRP& sol);
	~solutionVRP();
	//friend std::ostream &operator << (std::ostream &out_file, solutionVRP& s);

	/* general methods */
	void generateInitialSolution();
	int bestInsertion(int vertex);
	void insertVertex(int vertex, int before_i, bool remove);
	float getCost(int k = 0);
	int getViolations(int constraint = 0);

	/* interaction methods */
	int nbConstraints() { return 1; }
	int* getpPrevious() { return previous; }
	int* getpNext() { return next; }
	int* getpVehicle() { return vehicle; }
	virtual void routeChange(int k = 0) {}

	/* VRP specific methods */
	std::string& toString();
};


class solutionVRPTW: public solutionVRP {
private:
	/* Decision variables */
	/*
	int *previous;		// previous[i] = j iff vertex j is visited before i in the same route
	int *next;			// !!! -> Numbered from 1 to N
	int *vehicle;		// vehicle[i] = j iff vertex i is serviced by vehicle j
	*/
	float *b;			// service times
public:
	solutionVRPTW();												// constructor ***
	solutionVRPTW(const solutionVRPTW& old_solution);				// copy constructor ***
	solutionVRPTW& operator = (const solutionVRPTW& sol);
	~solutionVRPTW();
	//friend std::ostream &operator << (std::ostream &out_file, solutionVRPTW& s);

	/* general methods */
	int bestInsertion(int vertex);
	int getViolations(int constraint = 0);

	/* interaction methods */
	int nbConstraints() { return 2; }
	virtual void routeChange(int k = 0) { computeServiceTimes(k); }

	/* VRPTW specific methods */
	void computeServiceTimes(int k = 0);
	std::string& toString();
};


#endif