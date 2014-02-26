#include "../lib/VRPlib.h"
#include "../lib/LSBase.h"

#ifndef LSProgram_H
#define	LSProgram_H

using namespace CBLS;
using namespace std;


/*
	LSProgram specific implementation
	- acceptanceCriterion()
	- terminationCondition()
*/

// Basic local search
template <class S> class LSProgramBasic : public LSProgram<S> {
private:
	int max_iter;
	int diversification;
public:
	LSProgramBasic(S *initialSolution, int max_iter, int diversification) : LSProgram<S>(initialSolution) {
		this->max_iter = max_iter;
		this->diversification = diversification;
	}

	bool acceptanceCriterion(S& candidateSolution, S& incumbentSolution) {
		if (LSProgram<S>::computeSolutionEval(candidateSolution) < LSProgram<S>::computeSolutionEval(incumbentSolution)) return true;
		else return (rand() % 100 + 1 <= diversification);	// % diversification
	}
	bool terminationCondition() {
		return (this->iter >= max_iter);		// this needed to access parent protected variables
	}
	float computeSolutionEval(S& solution) {			// overload
		return solution.getCost() + 100*solution.getViolations();
	}
};

#endif