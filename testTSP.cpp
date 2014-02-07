#include <iostream>
#include <fstream>	// file I/O
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy

#include "LSBase.h"

using namespace CBLS;
using namespace std;

/* Problem data variables */
int N;		// number of vertices
double **c;
float *coordX; float *coordY;

void printInstance() {
	cout << "N: " << N << endl << flush;
}
void printDist(int i, int j) {
	cout << "c[" << i << "," << j << "]: " << c[i][j] << endl;
}


/*
	LSProgram specific implementation
	- readInstanceFile()
	- terminationCondition()
*/

/* Handle instance files of Cordeau-Laporte vrp/old */
void readInstanceFile(const char *filename) {
	int i;
	std::string line;
	std::ifstream instance_file;
	instance_file.open(filename);
	if (instance_file.fail()) {
		std::cerr << "Unable to open " << filename;
		exit (8);
	}

	instance_file >> i; instance_file >> i;		// skip first two integers
	instance_file >> N;		// retrieve instance size
	c = new double*[N+1];
	for(int i=0; i<N+1; i++) c[i] = new double[N+1];

	coordX = new float[N+1]; 
	coordY = new float[N+1];

	std::getline(instance_file, line); std::getline(instance_file, line); std::getline(instance_file, line); // skip lines
	for (int j=1; j<N+1; j++) {
		instance_file >> i; 				// skip int
		instance_file >> coordX[j]; 		// retrieve x coordinate
		instance_file >> coordY[j]; 		// retrieve y coordinate
		std::getline(instance_file, line);	// skip line
	}

	instance_file.close();

	for (int i=1; i<N+1; i++)
		for (int j=1; j<N+1; j++)
			c[i][j] = std::sqrt(std::pow((coordX[i] - coordX[j]),2) + std::pow((coordY[i] - coordY[j]),2)); // compute Euclidean distances
}

/* class solutionTSP
*	defines a solution for a TSP 
*/
class solutionTSP: public solution {
private:
	/* Decision variables */
	int *step;			// step[i] = j iff vertex j is visited in the i-th position
						// !!! -> Numbered from 1 to N
	/* Other */
	struct restoreStruct {	// for restore operation
		int i1, i2;		
	} r;
public:
	solutionTSP() {											// constructor ***
		step = new int[N+1];	// vertices from 1 to N

		cout << "Generating initial solution ...";
		generateInitialSolution();
		cout << "Done.\n"; 
	}
	solutionTSP(const solutionTSP& old_solution) {				// copy constructor ***
		step = new int[N+1];
		copyFrom(old_solution);	
	}
	solutionTSP& operator = (const solutionTSP& sol) {
		memcpy(step, sol.step, (N+1)*sizeof(int));
		return (*this);
	}
	void copyFrom(const solutionTSP& old_solution) {
		memcpy(step, old_solution.step, (N+1)*sizeof(int));
		//for (int i=1; i<=N; i++) step[i] = old_solution.step[i];
	}
	~solutionTSP() {										// destructor ***
		delete [] step;
	}
	friend std::ostream &operator << (std::ostream &out_file, solutionTSP& s);

	void generateInitialSolution() {
		/* Assign trivial values */
		for (int i=1; i<=N; i++) step[i] = i;
	}
	void shakeSolution() {
		/* Swap two random steps */
		int swp, i1, i2;

		i1 = rand() % N + 1; // generate random number in [1..N]
		i2 = rand() % N + 1; 

		swp = step[i1];
		step[i1] = step[i2];
		step[i2] = swp;

		r.i1 = i1; 
		r.i2 = i2;
	}
	void restore() {
		/* Swap back */
		int swp;

		swp = step[r.i1];
		step[r.i1] = step[r.i2];
		step[r.i2] = swp;
	}
	float getCost() {
		float cost = 0.0;
		for (int i=1; i<N; i++) cost += c[step[i]][step[i+1]];
		cost += c[step[N]][step[1]];

		return (cost);
	}
	float getViolations() {
		return (0.0);
	}
	float getEval() {					// bizarre que je doive faire ça ... il doit y avoir un moyen de faire autrement
		return solution::getEval();		// et j'ai envie de laisser getEval comme un membre de solution (plutot que solutionTSP),
	}									// parce que pour moi une eval c'est d'office cost+violations_penalty

};
inline std::ostream &operator << (std::ostream &out_file, solutionTSP& s) {
	if (s.getViolations() > 0) cout << "Infeasible ! ";
	out_file << "Cost=" << s.getCost() << " ";
	for(int i=1; i<N+1; i++)
		out_file << s.step[i] << " ";
	return (out_file);
}

template <class S> class LSProgramTSP : public LSProgram<S> {
public:
	LSProgramTSP(S *initialSolution) : LSProgram<S>(initialSolution) {}

	bool acceptanceCriterion(S& candidateSolution, S& incumbentSolution) {
		if (candidateSolution.getEval() < incumbentSolution.getEval()) return true;
		else return (rand() % 100 <= 0);	// 1% diversification
	}
	bool terminationCondition() {
		return (this->iter >= 1000000);		// this needed to access parent protected variables
	}
};



int main() {
	readInstanceFile("p00"); printInstance(); printDist(3,4);
	solutionTSP s;
	cout << "Initial solution: " << endl << s << endl;

	srand(time(NULL));

	LSProgramTSP<solutionTSP> p(&s);
	cout << "Running LS program" << endl;
	p.run();
	cout << endl << "Terminated. Best solution found: \n" << s << endl;
}
