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
protected:
	int max_iter;
	int diversification;
public:
	LSProgramBasic(S *initialSolution, int max_iter, int diversification) : LSProgram<S>(initialSolution) {
		this->max_iter = max_iter;
		this->diversification = diversification;
	}

	bool acceptanceCriterion(S& candidateSolution, S& currentSolution) {
		if (LSProgram<S>::computeSolutionEval(candidateSolution) 
			< LSProgram<S>::computeSolutionEval(currentSolution)) return true;
		else return (rand() % 100 + 1 <= diversification);	// % diversification
	}
	bool terminationCondition() {
		return (this->iter >= max_iter);		// "this->" needed to access parent protected variables
	}
	float computeSolutionEval(S& solution, S& currentSolution) {			// overload
		return solution.getCost() + 100*solution.getViolations();
	}
};

// Basic local search, extended with dynamic soft constraint weights adjustments
template <class S> class LSProgramBasicDynamicWeights : public LSProgramBasic<S> {
private:	
	int nc;				// number of soft constraints in S
	float *c_weight;	// soft constraints respective weights (indiced from 1 to nc)
	float delta;		// delta for weight adjustments
public:
	LSProgramBasicDynamicWeights(S *initialSolution, int max_iter, int diversification, float delta) : 
	LSProgramBasic<S>(initialSolution, max_iter, diversification) {
		nc = initialSolution->nbConstraints();
		c_weight = new float [nc+1];
		for (int i=1; i<=nc; i++) c_weight[i] = 1.0;
		this->delta = delta;
	}
	bool acceptanceCriterion(S& candidateSolution, S& currentSolution) {
		if (LSProgram<S>::computeSolutionEval(candidateSolution) 
			<= LSProgram<S>::computeSolutionEval(currentSolution)) return true;
		else if (rand() % 100 + 1 <= this->diversification) return true;	// % diversification
		updatePenaltyWeights(currentSolution);
		return false;
	}
	float computeSolutionEval(S& solution) {
		float penalty = 0.0;
		for (int c=1; c<=nc; c++) 
			penalty += solution.getViolations(c) * c_weight[c];
		return solution.getCost() + penalty;
	}
	void updatePenaltyWeights(S& currentSolution) {
		for (int c=1; c<=nc; c++) {
			if (currentSolution.getViolations(c) > 0) 	// if constraint c is violated,
				c_weight[c] *= 1 + delta; 				// delta% increase of its penalty weight for the next evaluation
			else 										// otherwise
				c_weight[c] /= 1 + delta; 				// delta% decrease
		}
	}
	void printWeights() {
		for (int c=1; c<=nc; c++)
			cout << "c_weight[" << c << "]: " << c_weight[c] << "\t";
		cout << "\tdelta: " << delta << endl;
	}
};



#endif