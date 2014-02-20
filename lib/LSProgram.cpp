#include "../lib/VRPlib.h"
#import "../lib/LSBase.h"

using namespace CBLS;
using namespace std;


/*
	LSProgram specific implementation
	- acceptanceCriterion()
	- terminationCondition()
*/

// LSProgramBasic: Basic local search
LSProgramTSP(S *initialSolution) : LSProgram<S>(initialSolution, max_iter, diversification) {
	this->max_iter = max_iter;
	this->diversification = diversification;
}

bool acceptanceCriterion(S& candidateSolution, S& incumbentSolution) {
	if (LSProgram<S>::computeSolutionEval(candidateSolution) < LSProgram<S>::computeSolutionEval(incumbentSolution)) return true;
	else return (rand() % 100 + 1 <= diversification);	// % diversification
}
bool terminationCondition() {
	return (this->iter >= 10000000);		// this needed to access parent protected variables
}
float computeSolutionEval(S& solution) {			// overload
	return solution.getCost() + 10*solution.getViolations();
}