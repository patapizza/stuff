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



class solutionVRP: public solution {
private:
	/* Decision variables */
	int *previous;		// previous[i] = j iff vertex j is visited before i in the same route
	int *next;			// !!! -> Numbered from 1 to N
	int *vehicle;		// vehicle[i] = j iff vertex j is serviced by vehicle i
public:
	solutionVRP();												// constructor ***
	solutionVRP(const solutionVRP& old_solution);				// copy constructor ***
	solutionVRP& operator = (const solutionVRP& sol);
	~solutionVRP();
	//friend std::ostream &operator << (std::ostream &out_file, solutionVRP& s);

	void generateInitialSolution();
	void shakeSolution();
	float getCost();
	int getViolations(int c = 0);
	int nbConstraints();
	std::string& toString();
};


class solutionVRPTW: public solution {
private:
	/* Decision variables */
	int *previous;		// previous[i] = j iff vertex j is visited before i in the same route
	int *next;			// !!! -> Numbered from 1 to N
	int *vehicle;		// vehicle[i] = j iff vertex j is serviced by vehicle i
	float *b;			// service times
public:
	solutionVRPTW();												// constructor ***
	solutionVRPTW(const solutionVRPTW& old_solution);				// copy constructor ***
	solutionVRPTW& operator = (const solutionVRPTW& sol);
	~solutionVRPTW();
	friend std::ostream &operator << (std::ostream &out_file, solutionVRPTW& s);

	/* virtual method instanciations */
	void generateInitialSolution();
	void shakeSolution();
	float getCost();
	int getViolations(int c = 0);
	int nbConstraints();

	/* specific methods */
	void computeServiceTimes(int k = 0);
	float getCostR(int k = 0);
	std::string& toString();
};


#endif