#include <iostream>
#include <fstream>	// file I/O
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy

#include "LSBase.h"

using namespace CBLS;

/* Handle instance files of Cordeau-Laporte vrp/old */
void readInstanceFileCordeauLaporteVRPold(const char *filename);

/* class solutionTSP
*	defines a solution for a TSP 
*/
class solutionTSP: public solution {
private:
	/* Decision variables */
	int *step;			// step[i] = j iff vertex j is visited in the i-th position
						// !!! -> Numbered from 1 to N
public:
	solutionTSP();												// constructor ***
	solutionTSP(const solutionTSP& old_solution);				// copy constructor ***
	solutionTSP& operator = (const solutionTSP& sol);
	~solutionTSP();
	//friend std::ostream &operator << (std::ostream &out_file, solutionTSP& s);

	void generateInitialSolution();
	void shakeSolution();
	float getCost();
	float getViolations();
	std::string& toString();
};

//inline std::ostream &operator << (std::ostream &out_file, solutionTSP& s);

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
	float getViolations();
	std::string& toString();
};