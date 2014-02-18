#include <iostream>
#include <fstream>	// file I/O
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy

#include "../lib/VRPlib.h"
#import "../lib/LSBase.h"

using namespace CBLS;
using namespace std;


/*
	LSProgram specific implementation
	- acceptanceCriterion()
	- terminationCondition()
*/

template <class S> class LSProgramTSP : public LSProgram<S> {
public:
	LSProgramTSP(S *initialSolution) : LSProgram<S>(initialSolution) {}

	bool acceptanceCriterion(S& candidateSolution, S& incumbentSolution) {
		if (LSProgram<S>::computeSolutionEval(candidateSolution) < LSProgram<S>::computeSolutionEval(incumbentSolution)) return true;
		else return (rand() % 100 <= 2);	// 3% diversification
	}
	bool terminationCondition() {
		return (this->iter >= 10000000);		// this needed to access parent protected variables
	}
	float computeSolutionEval(S& solution) {			// overload
		return solution.getCost() + 10*solution.getViolations();
	}
};



int main() {
	long t = time(NULL);
	readInstanceFileCordeauLaporteVRPold("p01"); 
	solutionVRP s;
	cout << "Initial solution: " << endl << s.toString() << endl << flush;
	srand(t);

	LSProgramTSP<solutionVRP> p(&s);
	cout << "Running LS program" << endl;
	p.run();
	cout << endl << "Terminated. Best solution found: \n" << s.toString() << endl;
}
